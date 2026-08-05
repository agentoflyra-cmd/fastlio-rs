use crate::VoxelKey;
use anyhow::Result;
use fastlio_types::{PointCloud, PointXYZI, Vec3};
use hashbrown::HashMap;

pub struct VoxelMap {
    voxel_size: f32,
    voxels: HashMap<u64, PointCloud>,
}

impl VoxelMap {
    pub fn new(voxel_size: f32) -> Self {
        Self {
            voxel_size,
            voxels: HashMap::new(),
        }
    }

    pub fn insert(&mut self, points: impl IntoIterator<Item = PointXYZI>) -> Result<()> {
        for point in points {
            if !point.is_valid() {
                continue;
            }
            let voxel_key = VoxelKey::new(&point, self.voxel_size)?.pack();
            self.voxels.entry(voxel_key).or_default().push(point);
        }
        Ok(())
    }

    /// query_w should be normal f32 Vec3, which garanteened by builder.
    /// In some scenarios, point clouds may need to return or potentially receive invalid values ​​
    /// such as `inf` or `nan`. While this can be avoided by using `new`
    /// with the existing `voxel key`, the implementation in this function does not use `VoxelKey::new`.
    /// Therefore, it's necessary to ensure that the value passed to
    /// this function is not invalid.
    /// This is supposed to be guaranteed by its caller; here,
    /// to ensure semantics, a debug assertion is used again to guarantee this.
    pub fn nearby_points(&self, query_w: &Vec3<f32>, radius: f32) -> PointCloud {
        debug_assert!(query_w.iter().all(|v| v.is_finite()));
        debug_assert!(radius.is_finite() && radius >= 0.0);
        let voxel_size = self.voxel_size;
        let min_x = ((query_w[0] - radius) / voxel_size).floor() as i32;
        let max_x = ((query_w[0] + radius) / voxel_size).floor() as i32;
        let min_y = ((query_w[1] - radius) / voxel_size).floor() as i32;
        let max_y = ((query_w[1] + radius) / voxel_size).floor() as i32;
        let min_z = ((query_w[2] - radius) / voxel_size).floor() as i32;
        let max_z = ((query_w[2] + radius) / voxel_size).floor() as i32;

        let radius_squared = radius * radius;
        let mut result = Vec::new();

        for x in min_x..=max_x {
            for y in min_y..=max_y {
                for z in min_z..=max_z {
                    let x = VoxelKey::encode_axis(x);
                    let y = VoxelKey::encode_axis(y);
                    let z = VoxelKey::encode_axis(z);
                    let voxel_key = (x << 42) | (y << 21) | z;
                    let Some(cell) = self.voxels.get(&voxel_key) else {
                        continue;
                    };
                    result.extend(
                        cell.iter()
                            .filter(|p| (p.to_vec3() - *query_w).norm_squared() <= radius_squared)
                            .cloned(),
                    );
                }
            }
        }
        result
    }

    /// query_w should be normal f32 Vec3, which garanteed by builder.
    /// In some scenarios, point clouds may need to return or potentially receive invalid values ​​
    /// such as `inf` or `nan`. While this can be avoided by using `new`
    /// with the existing `voxel key`, the implementation in this function does not use `VoxelKey::new`.
    /// Therefore, it's necessary to ensure that the value passed to
    /// this function is not invalid.
    /// This is supposed to be guaranteed by its caller; here,
    /// to ensure semantics, a debug assertion is used again to guarantee this.
    pub fn nearby_points_iter(
        &self,
        query_w: &Vec3<f32>,
        radius: f32,
    ) -> impl Iterator<Item = &PointXYZI> {
        debug_assert!(query_w.iter().all(|v| v.is_finite()));
        debug_assert!(radius.is_finite() && radius >= 0.0);
        let voxel_size = self.voxel_size;
        let min_x = ((query_w[0] - radius) / voxel_size).floor() as i32;
        let max_x = ((query_w[0] + radius) / voxel_size).floor() as i32;
        let min_y = ((query_w[1] - radius) / voxel_size).floor() as i32;
        let max_y = ((query_w[1] + radius) / voxel_size).floor() as i32;
        let min_z = ((query_w[2] - radius) / voxel_size).floor() as i32;
        let max_z = ((query_w[2] + radius) / voxel_size).floor() as i32;

        let radius_squared = radius * radius;

        (min_x..=max_x)
            .flat_map(move |x| {
                (min_y..=max_y).flat_map(move |y| (min_z..=max_z).map(move |z| (x, y, z)))
            })
            .filter_map(move |(x, y, z)| {
                let x = VoxelKey::encode_axis(x);
                let y = VoxelKey::encode_axis(y);
                let z = VoxelKey::encode_axis(z);
                let voxel_key = (x << 42) | (y << 21) | z;
                self.voxels.get(&voxel_key)
            })
            .flat_map(|cell| cell.iter())
            .filter(move |p| (p.to_vec3() - query_w).norm_squared() <= radius_squared)
    }

    /// center_w should be normal f32 Vec3, which garanteed by builder.
    /// In some scenarios, point clouds may need to return or potentially receive invalid values ​​
    /// such as `inf` or `nan`. While this can be avoided by using `new`
    /// with the existing `voxel key`, the implementation in this function does not use `VoxelKey::new`.
    /// Therefore, it's necessary to ensure that the value passed to
    /// this function is not invalid.
    /// This is supposed to be guaranteed by its caller; here,
    /// to ensure semantics, a debug assertion is used again to guarantee this.
    pub fn crop_around(&mut self, center_w: &Vec3<f32>, radius: f32) -> usize {
        debug_assert!(center_w.iter().all(|v| v.is_finite()));
        debug_assert!(radius.is_finite() && radius >= 0.0);
        let radius_squared = radius * radius;
        let before = self.voxels.len();
        let voxel_size = self.voxel_size;
        self.voxels.retain(|&key, _| {
            let voxel = VoxelKey::unpack(key);

            let x = (voxel.x as f32 + 0.5) * voxel_size;
            let y = (voxel.y as f32 + 0.5) * voxel_size;
            let z = (voxel.z as f32 + 0.5) * voxel_size;

            let dx = x - center_w.x;
            let dy = y - center_w.y;
            let dz = z - center_w.z;
            dx * dx + dy * dy + dz * dz <= radius_squared
        });

        before - self.voxels.len()
    }

    /// `nearest_n` returns the n nearest points within a radius of the target point `query_w`.
    /// This is not guaranteed to be ordered, to avoid the extra overhead of sorting,
    /// although this overhead is small.
    /// If order is required, you can call `nearest_n_sorted`.
    /// This function simply sorts the return value of this function
    /// instead of sorting first and then retrieving the n points;
    /// therefore, the returned set of points will always be identical.
    #[inline]
    pub fn nearest_n(
        &self,
        query_w: &Vec3<f32>,
        radius: f32,
        count: usize,
    ) -> Vec<NearestPoint<'_>> {
        if count == 0 {
            return Vec::new();
        }
        let mut iters = self
            .nearby_points_iter(query_w, radius)
            .map(|p| {
                let squared_distance = (p.to_vec3() - query_w).norm_squared();
                NearestPoint::new(p, squared_distance)
            })
            .collect::<Vec<_>>();
        if iters.len() > count {
            iters.select_nth_unstable_by(count - 1, |a, b| {
                a.squared_distance.total_cmp(&b.squared_distance)
            });
            iters.truncate(count);
        }
        iters
    }

    /// `nearest_n_sorted` returns the n nearest points within a radius of the target point `query_w`.
    pub fn nearest_n_sorted(
        &self,
        query_w: &Vec3<f32>,
        radius: f32,
        count: usize,
    ) -> Vec<NearestPoint<'_>> {
        let mut iters = self.nearest_n(query_w, radius, count);
        iters.sort_unstable_by(|a, b| a.squared_distance.total_cmp(&b.squared_distance));
        iters
    }

    /// number of voxel keys
    #[inline]
    pub fn len(&self) -> usize {
        self.voxels.len()
    }

    /// return 0 if voxel keys are empty
    #[must_use]
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// number of points in voxel map
    #[inline]
    pub fn point_count(&self) -> usize {
        self.voxels.values().map(PointCloud::len).sum()
    }
}

pub struct NearestPoint<'a> {
    pub point: &'a PointXYZI,
    pub squared_distance: f32,
}

impl<'a> NearestPoint<'a> {
    pub fn new(point: &'a PointXYZI, squared_distance: f32) -> Self {
        Self {
            point,
            squared_distance,
        }
    }
}
