use anyhow::{Result, bail};
use fastlio_types::{Mat3, PointXYZI, SurfelConfig, Vec3};
use smallvec::SmallVec;

#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Copy, Clone)]
pub struct VoxelKey {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl VoxelKey {
    const AXIS_BITS: u32 = 21;
    const AXIS_MASK: u64 = (1 << Self::AXIS_BITS) - 1;
    const AXIS_BIAS: i64 = 1 << 20;

    #[inline]
    pub fn new(point: &PointXYZI, voxel_size: f32) -> Result<Self> {
        Self::from_vec3(point.to_vec3(), voxel_size)
    }

    #[inline]
    pub fn from_vec3(point: Vec3<f32>, voxel_size: f32) -> Result<Self> {
        if !voxel_size.is_normal() || voxel_size <= 0.0 {
            bail!("surfel voxel_size must be finite and positive");
        }

        Ok(Self {
            x: (point.x / voxel_size).floor() as i32,
            y: (point.y / voxel_size).floor() as i32,
            z: (point.z / voxel_size).floor() as i32,
        })
    }

    #[inline]
    fn encode_axis(value: i32) -> u64 {
        let encoded = value as i64 + Self::AXIS_BIAS;
        debug_assert!(
            (0..=Self::AXIS_MASK as i64).contains(&encoded),
            "surfel voxel axis exceeds 21-bit packed range"
        );
        encoded as u64
    }

    #[inline]
    fn decode_axis(value: u64) -> i32 {
        (value as i64 - Self::AXIS_BIAS) as i32
    }

    #[inline]
    pub fn pack(self) -> u64 {
        let x = Self::encode_axis(self.x);
        let y = Self::encode_axis(self.y);
        let z = Self::encode_axis(self.z);
        (x << 42) | (y << 21) | z
    }

    #[inline]
    pub fn unpack(key: u64) -> Self {
        Self {
            x: Self::decode_axis((key >> 42) & Self::AXIS_MASK),
            y: Self::decode_axis((key >> 21) & Self::AXIS_MASK),
            z: Self::decode_axis(key & Self::AXIS_MASK),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeometryClass {
    Plane,
    Line,
    Scatter,
    Degenerate,
    Growing,
}

#[derive(Debug, Default)]
pub struct Surfel {
    pub mean_w: Vec3<f64>,
    pub eigenvectors: Mat3<f64>,
    pub eigenvalues: Vec3<f64>,
    pub m2: Mat3<f64>,
    pub count: usize,
    pub last_refit: usize,
    pub indexed_voxels: SmallVec<[u64; 8]>,
}

impl Surfel {
    pub fn from_first_point(point: &PointXYZI) -> Self {
        Self {
            mean_w: point.to_vec3_f64(),
            eigenvectors: Mat3::<f64>::identity(),
            eigenvalues: Vec3::<f64>::zeros(),
            m2: Mat3::<f64>::zeros(),
            count: 1,
            last_refit: 0,
            indexed_voxels: SmallVec::new(),
        }
    }

    pub fn plane_distance(&self, point: &PointXYZI) -> f32 {
        let normal_w = self.eigenvectors.column(0);
        ((point.to_vec3_f64() - self.mean_w).dot(&normal_w).abs()) as f32
    }

    pub fn within_plane_distance(&self, point: &PointXYZI, config: &SurfelConfig) -> bool {
        self.plane_distance(point) <= config.max_plane_distance
    }

    pub fn support_aabb_extent(&self, config: &SurfelConfig) -> Vec3<f64> {
        let support_sigma = config.max_mahalanobis_distance as f64;
        let covariance_floor = config.covariance_eigenvalue_floor as f64;
        let lambda_0 = support_sigma * support_sigma * self.eigenvalues[0].max(covariance_floor);
        let lambda_1 = support_sigma * support_sigma * self.eigenvalues[1].max(covariance_floor);
        let lambda_2 = support_sigma * support_sigma * self.eigenvalues[2].max(covariance_floor);

        let t0 = self.eigenvectors.column(0);
        let t1 = self.eigenvectors.column(1);
        let t2 = self.eigenvectors.column(2);

        Vec3::new(
            (lambda_0 * t0.x * t0.x + lambda_1 * t1.x * t1.x + lambda_2 * t2.x * t2.x).sqrt(),
            (lambda_0 * t0.y * t0.y + lambda_1 * t1.y * t1.y + lambda_2 * t2.y * t2.y).sqrt(),
            (lambda_0 * t0.z * t0.z + lambda_1 * t1.z * t1.z + lambda_2 * t2.z * t2.z).sqrt(),
        )
    }

    pub fn mahalanobis_squared(&self, point: &PointXYZI, config: &SurfelConfig) -> f32 {
        let delta = point.to_vec3_f64() - self.mean_w;
        let d0 = self.eigenvectors.column(0).dot(&delta);
        let d1 = self.eigenvectors.column(1).dot(&delta);
        let d2 = self.eigenvectors.column(2).dot(&delta);

        let covariance_floor = config.covariance_eigenvalue_floor as f64;
        let l0 = self.eigenvalues[0].max(covariance_floor);
        let l1 = self.eigenvalues[1].max(covariance_floor);
        let l2 = self.eigenvalues[2].max(covariance_floor);

        (d0 * d0 / l0 + d1 * d1 / l1 + d2 * d2 / l2) as f32
    }

    pub fn within_support(&self, point: &PointXYZI, config: &SurfelConfig) -> bool {
        let support_sigma = config.max_mahalanobis_distance;
        self.mahalanobis_squared(point, config) <= support_sigma * support_sigma
    }

    pub fn within_tangent_support(&self, point: &PointXYZI, config: &SurfelConfig) -> bool {
        let delta = point.to_vec3_f64() - self.mean_w;
        let d1 = self.eigenvectors.column(1).dot(&delta);
        let d2 = self.eigenvectors.column(2).dot(&delta);

        let covariance_floor = config.covariance_eigenvalue_floor as f64;
        let l1 = self.eigenvalues[1].max(covariance_floor);
        let l2 = self.eigenvalues[2].max(covariance_floor);
        let support_sigma = config.max_mahalanobis_distance as f64;

        d1 * d1 / l1 + d2 * d2 / l2 <= support_sigma * support_sigma
    }

    pub fn planarity(&self) -> f64 {
        if self.eigenvalues[1] <= 0.0 {
            return f64::INFINITY;
        }
        self.eigenvalues[0] / self.eigenvalues[1]
    }

    fn linearity(&self) -> f64 {
        if self.eigenvalues[2] <= 0.0 {
            return 0.0;
        }
        (self.eigenvalues[2] - self.eigenvalues[1]) / self.eigenvalues[2]
    }

    fn scattering(&self) -> f64 {
        if self.eigenvalues[2] <= 0.0 {
            return 0.0;
        }
        self.eigenvalues[0] / self.eigenvalues[2]
    }

    pub fn is_growing(&self, min_mature_surfel_count: usize) -> bool {
        self.count < min_mature_surfel_count
    }

    pub fn is_planar(&self, config: &SurfelConfig) -> bool {
        self.count >= config.min_mature_surfel_count
            && self.eigenvalues[1] >= config.min_plane_spread_eigenvalue as f64
            && self.planarity() <= config.max_planarity_ratio as f64
    }

    pub fn geometry_class(&self, config: &SurfelConfig) -> GeometryClass {
        if self.count < config.min_mature_surfel_count {
            return GeometryClass::Growing;
        }

        let planarity = self.planarity();
        let linearity = self.linearity();
        let scattering = self.scattering();
        if planarity < config.max_planarity_ratio as f64
            && self.eigenvalues[1] > config.min_plane_spread_eigenvalue as f64
        {
            GeometryClass::Plane
        } else if linearity > config.min_linearity as f64
            && self.eigenvalues[2] > config.min_line_spread_eigenvalue as f64
        {
            GeometryClass::Line
        } else if scattering > config.min_scattering as f64 {
            GeometryClass::Scatter
        } else {
            GeometryClass::Degenerate
        }
    }

    pub fn within_growing_radius(&self, point: &PointXYZI, config: &SurfelConfig) -> bool {
        let dist2 = (point.to_vec3_f64() - self.mean_w).norm_squared();
        let radius = config.growing_radius as f64;
        dist2 <= radius * radius
    }
}
