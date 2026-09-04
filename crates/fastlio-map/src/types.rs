use anyhow::Result;
use fastlio_types::{Mat3, PointXYZI, SurfelConfig, Vec3};
use smallvec::SmallVec;

#[derive(PartialEq, Eq, PartialOrd, Ord, Copy, Clone)]
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
        if !voxel_size.is_normal() || voxel_size < 0.0 {
            anyhow::bail!("voxel_size should be a valid number(f32).")
        }
        let x = (point.x / voxel_size).floor() as i32;
        let y = (point.y / voxel_size).floor() as i32;
        let z = (point.z / voxel_size).floor() as i32;
        Ok(Self { x, y, z })
    }

    #[inline]
    pub fn from_vec3(point: Vec3<f32>, voxel_size: f32) -> Result<Self> {
        if !voxel_size.is_normal() || voxel_size < 0.0 {
            anyhow::bail!("voxel_size should be a valid number(f32).")
        }

        let x = (point[0] / voxel_size).floor() as i32;
        let y = (point[1] / voxel_size).floor() as i32;
        let z = (point[2] / voxel_size).floor() as i32;
        Ok(Self { x, y, z })
    }

    #[inline]
    pub(crate) fn encode_axis(value: i32) -> u64 {
        let encoded = value as i64 + Self::AXIS_BIAS;

        debug_assert!(
            (0..=Self::AXIS_MASK as i64).contains(&encoded),
            "map axis exceed max: 21 bit"
        );
        encoded as u64
    }

    #[inline]
    pub(crate) fn decode_axis(value: u64) -> i32 {
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

pub enum GeometryClass {
    Plane,
    Line,
    Scatter,
    Degenerate,
    Growing,
}

#[derive(Default)]
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
        let mean_w = point.to_vec3().cast();
        let m2 = Mat3::<f64>::zeros();
        let init_vec3 = Vec3::<f64>::zeros();

        Surfel {
            mean_w,
            eigenvectors: m2,
            eigenvalues: init_vec3,
            m2,
            count: 1,
            last_refit: 0,
            indexed_voxels: SmallVec::new(),
        }
    }

    pub fn plane_distance(&self, point: &PointXYZI) -> f32 {
        let norm_w = self.eigenvectors.column(0);
        ((point.to_vec3().cast::<f64>() - self.mean_w)
            .dot(&norm_w)
            .abs()) as f32
    }

    pub fn within_plane_distance(&self, point: &PointXYZI, config: &SurfelConfig) -> bool {
        let max_distance = config.max_plane_distance;
        self.plane_distance(point) <= max_distance
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
        let delta = point.to_vec3().cast::<f64>() - self.mean_w;
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

    pub(crate) fn planarity(&self) -> f64 {
        self.eigenvalues[0] / self.eigenvalues[1]
    }

    fn linearity(&self) -> f64 {
        (self.eigenvalues[2] - self.eigenvalues[1]) / self.eigenvalues[2]
    }

    fn scattering(&self) -> f64 {
        self.eigenvalues[0] / self.eigenvalues[2]
    }

    pub(crate) fn is_growing(&self, min_mature_surfel_count: usize) -> bool {
        self.count < min_mature_surfel_count
    }

    pub(crate) fn is_planar(&self, config: &SurfelConfig) -> bool {
        if self.eigenvalues[1] < config.min_plane_spread_eigenvalue as f64 {
            return false;
        }
        self.planarity() <= config.max_planarity_ratio as f64
    }

    pub fn geometry_class(&self, config: &SurfelConfig) -> GeometryClass {
        let planarity = self.planarity();
        let linearity = self.linearity();
        let scattering = self.scattering();
        if self.count < config.min_mature_surfel_count {
            GeometryClass::Growing
        } else if planarity < config.max_planarity_ratio as f64
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

    pub(crate) fn within_growing_radius(&self, point: &PointXYZI, config: &SurfelConfig) -> bool {
        let dist2 = (point.to_vec3().cast::<f64>() - self.mean_w).norm_squared();
        let radius = config.growing_radius as f64;
        dist2 <= radius * radius
    }

    // merge an similar surfel
    // pub(crate) fn merge_surfel(&mut self, rhs: &Surfel) {
    //     let na = self.count as f32;
    //     let nb = rhs.count as f32;
    //     let n = na + nb;

    //     let delta = rhs.mean_w - self.mean_w;

    //     self.mean_w += delta * (nb / n) as f64;
    //     self.m2 += rhs.m2 + delta * delta.transpose() * (na * nb / n) as f64;
    //     self.count += rhs.count;
    // }
}
