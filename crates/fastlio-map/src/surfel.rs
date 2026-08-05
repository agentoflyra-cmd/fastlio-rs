use crate::VoxelKey;
use crate::types::{GeometryClass, Surfel};
use anyhow::Result;
use fastlio_types::{Mat3, PointXYZI, SurfelConfig, SurfelMapConfig, Vec3};
use hashbrown::HashMap;
use slotmap::{SlotMap, new_key_type};
use smallvec::SmallVec;

new_key_type! {pub struct SurfelID;}

#[derive(Default)]
pub struct SurfelMap {
    pub(super) buckets: HashMap<u64, SmallVec<[SurfelID; 4]>>,
    pub(super) surfels: SlotMap<SurfelID, Surfel>,
    surfel_map_config: SurfelMapConfig,
    surfel_config: SurfelConfig,
}

/// A planar surface observation returned by [`SurfelMap::query`].
///
/// It describes the *plane* attached to the best-matching planar surfel
/// near a query point, expressed entirely in the world / map frame `W`:
///
/// - `mean_w`: a point on the plane (the surfel centroid).
/// - `norm_w`: a unit plane normal, expressed in `W`.
/// - `eigenvalues`: the surfel covariance eigenvalues sorted ascending; for
///   a plane the smallest entry is near zero while `eigenvalues[1]` measures
///   spread along the plane.
/// - `plane_distance`: the perpendicular distance from the queried point to
///   the plane: `|(query_w - mean_w) · norm_w|`.
/// - `planarity`: the ratio `eigenvalues[0] / eigenvalues[1]`; smaller means
///   more planar.
///
/// All coordinates and vectors are in meters and expressed in the world
/// frame. The sign of `norm_w` is arbitrary (it follows the principal
/// eigenvector) and must not be depended upon; use its absolute value or
/// align it against a known reference.
#[derive(Debug, Clone)]
pub struct SurfelObservation {
    pub surfel_id: SurfelID,
    pub mean_w: Vec3<f64>,
    pub norm_w: Vec3<f64>,
    pub eigenvalues: Vec3<f64>,
    pub plane_distance: f64,
    pub planarity: f64,
    pub signed_residual: f64,
}

impl SurfelObservation {
    pub fn new(
        surfel_id: SurfelID,
        mean_w: Vec3<f64>,
        norm_w: Vec3<f64>,
        eigenvalues: Vec3<f64>,
        plane_distance: f64,
        planarity: f64,
        signed_residual: f64,
    ) -> Self {
        Self {
            surfel_id,
            mean_w,
            norm_w,
            eigenvalues,
            plane_distance,
            planarity,
            signed_residual,
        }
    }
}

fn helper_interval(count: usize) -> usize {
    match count {
        0..=16 => 4,
        17..=64 => 8,
        _ => 16,
    }
}

impl SurfelMap {
    pub fn new(map_config: SurfelMapConfig, surfel_config: SurfelConfig) -> Self {
        let buckets = HashMap::new();
        let surfels = SlotMap::with_key();
        Self {
            buckets,
            surfels,
            surfel_map_config: map_config,
            surfel_config,
        }
    }

    pub fn query(&self, point: &PointXYZI) -> Result<Option<SurfelObservation>> {
        let radius = self.surfel_map_config.search_radius;
        if !point.is_valid() {
            return Ok(None);
        }
        let mut voxel_key = VoxelKey::new(point, self.surfel_map_config.voxel_size)?;
        let x = voxel_key.x;
        let y = voxel_key.y;
        let z = voxel_key.z;
        let candidate_ids = (x - radius..=x + radius)
            .flat_map(move |x| {
                (y - radius..=y + radius).flat_map(move |y| {
                    (z - radius..=z + radius).map(move |z| {
                        voxel_key.x = x;
                        voxel_key.y = y;
                        voxel_key.z = z;
                        voxel_key.pack()
                    })
                })
            })
            .filter_map(|v| self.buckets.get(&v))
            .flat_map(|bucket| bucket.iter().copied())
            .filter_map(|id| self.surfels.get(id).map(|s| (id, s)))
            .filter(|(_, surfel)| {
                surfel.is_planar(&self.surfel_config)
                    && surfel.within_support(point, &self.surfel_config)
                    && surfel.within_plane_distance(point, &self.surfel_config)
            });
        let mut best: Option<(SurfelID, f32)> = None;
        for (id, surfel) in candidate_ids {
            match surfel.geometry_class(&self.surfel_config) {
                GeometryClass::Plane => {
                    let score = surfel.plane_distance(point);
                    if best.is_none_or(|(_, best)| score < best) {
                        best = Some((id, score));
                    }
                }
                _ => continue,
            }
        }

        if let Some((best_id, score)) = best {
            // best_id is selected from candidate_ids, which only yields ids that were
            let surfel = self.surfels.get(best_id).expect(
                "UnexpectedError: on query: Logically, there should be no missing value here.",
            );
            let mean_w = surfel.mean_w.cast();
            let norm_w = surfel.eigenvectors.column(0).into_owned().cast();
            let eigenvalues = surfel.eigenvalues.cast::<f64>();
            let plane_distance = score as f64;
            let planarity = surfel.planarity();
            let signed_residual = norm_w.dot(&(point.to_vec3_f64() - mean_w));
            Ok(Some(SurfelObservation::new(
                best_id,
                mean_w,
                norm_w,
                eigenvalues,
                plane_distance,
                planarity,
                signed_residual,
            )))
        } else {
            Ok(None)
        }
    }

    pub fn insert(&mut self, points: impl Iterator<Item = PointXYZI>) -> Result<()> {
        let radius = self.surfel_map_config.search_radius;
        for point in points {
            if !point.is_valid() {
                continue;
            }
            let mut voxel_key = VoxelKey::new(&point, self.surfel_map_config.voxel_size)?;
            let x = voxel_key.x;
            let y = voxel_key.y;
            let z = voxel_key.z;
            let candidate_ids = (x - radius..=x + radius)
                .flat_map(move |x| {
                    (y - radius..=y + radius).flat_map(move |y| {
                        (z - radius..=z + radius).map(move |z| {
                            voxel_key.x = x;
                            voxel_key.y = y;
                            voxel_key.z = z;
                            voxel_key.pack()
                        })
                    })
                })
                .filter_map(|v| self.buckets.get(&v))
                .flat_map(|bucket| bucket.iter().copied())
                .filter_map(|id| self.surfels.get(id).map(|s| (id, s)))
                .filter(|(_, surfel)| {
                    if surfel.is_growing(self.surfel_config.min_mature_surfel_count) {
                        surfel.within_growing_radius(&point, &self.surfel_config)
                    } else {
                        surfel.within_support(&point, &self.surfel_config)
                    }
                });
            let mut best_mature: Option<(SurfelID, f64)> = None;
            let mut best_growing: Option<(SurfelID, f64)> = None;

            for (id, s) in candidate_ids {
                if s.count >= self.surfel_config.min_mature_surfel_count {
                    let score = s.plane_distance(&point) as f64;
                    if best_mature.is_none_or(|(_, best)| score < best) {
                        best_mature = Some((id, score));
                    }
                } else {
                    let score = (point.to_vec3().cast::<f64>() - s.mean_w).norm_squared();
                    if best_growing.is_none_or(|(_, best)| score < best) {
                        best_growing = Some((id, score));
                    }
                }
            }

            if let Some((best_id, _)) = best_mature {
                self.update_surfel(best_id, &point)?;
            } else if let Some((best_id, _)) = best_growing {
                self.update_surfel(best_id, &point)?;
            } else {
                let _id = self.create_surfel(&point)?;
            }
        }
        Ok(())
    }

    fn create_surfel(&mut self, point: &PointXYZI) -> Result<SurfelID> {
        let surfel = Surfel::from_first_point(point);
        let id = self.surfels.insert(surfel);
        self.reindex_surfel(id)?;
        Ok(id)
    }

    fn update_surfel(&mut self, id: SurfelID, point: &PointXYZI) -> Result<()> {
        // best_id is selected from candidate_ids, which only yields ids that were
        let surfel = self.surfels.get_mut(id).expect(
            "UnexpectedError: on update: Logically, there should be no missing value here.",
        );
        let mut needs_reindex = { surfel.is_growing(self.surfel_config.min_mature_surfel_count) };

        surfel.count += 1;
        let delta = point.to_vec3().cast::<f64>() - surfel.mean_w;
        let n = surfel.count as f64;
        surfel.mean_w += delta / n;
        surfel.m2 += delta * delta.transpose() * ((n - 1.0) / n);
        if surfel.count >= self.surfel_config.min_mature_surfel_count
            && (surfel.count - surfel.last_refit) > helper_interval(surfel.count)
        {
            let eigen = (surfel.m2 / (surfel.count - 1) as f64).symmetric_eigen();
            let mut eigen_pair = [
                (
                    eigen.eigenvalues[0],
                    eigen.eigenvectors.column(0).into_owned(),
                ),
                (
                    eigen.eigenvalues[1],
                    eigen.eigenvectors.column(1).into_owned(),
                ),
                (
                    eigen.eigenvalues[2],
                    eigen.eigenvectors.column(2).into_owned(),
                ),
            ];
            eigen_pair.sort_by(|a, b| a.0.total_cmp(&b.0));
            surfel.eigenvectors =
                Mat3::from_columns(&[eigen_pair[0].1, eigen_pair[1].1, eigen_pair[2].1]);
            surfel.eigenvalues = Vec3::new(eigen_pair[0].0, eigen_pair[1].0, eigen_pair[2].0);
            surfel.last_refit = surfel.count;
            needs_reindex = true;
        }

        if needs_reindex {
            self.reindex_surfel(id)?;
        }

        Ok(())
    }

    fn reindex_surfel(&mut self, id: SurfelID) -> Result<()> {
        let (old_keys, new_keys) = {
            let surfel = self.surfels.get(id).expect("surfel must exist");

            let extent = if surfel.is_growing(self.surfel_config.min_mature_surfel_count) {
                Vec3::repeat(self.surfel_config.growing_radius as f64)
            } else {
                surfel.support_aabb_extent(&self.surfel_config)
            };

            let min_w = surfel.mean_w - extent;
            let max_w = surfel.mean_w + extent;

            let min_key = VoxelKey::from_vec3(min_w.cast(), self.surfel_map_config.voxel_size)?;
            let max_key = VoxelKey::from_vec3(max_w.cast(), self.surfel_map_config.voxel_size)?;

            let mut new_keys = SmallVec::<[u64; 8]>::new();

            for x in min_key.x..=max_key.x {
                for y in min_key.y..=max_key.y {
                    for z in min_key.z..=max_key.z {
                        new_keys.push(VoxelKey { x, y, z }.pack());
                    }
                }
            }

            (surfel.indexed_voxels.clone(), new_keys)
        };

        if old_keys == new_keys {
            return Ok(());
        }

        for &key in &old_keys {
            let should_remove = if let Some(bucket) = self.buckets.get_mut(&key) {
                bucket.retain(|sid| *sid != id);
                bucket.is_empty()
            } else {
                false
            };
            if should_remove {
                self.buckets.remove(&key);
            }
        }

        for &key in &new_keys {
            self.buckets.entry(key).or_default().push(id);
        }

        self.surfels
            .get_mut(id)
            .expect("surfel must exist")
            .indexed_voxels = new_keys;

        Ok(())
    }

    // pub fn merge_compatible_mature_surfels(&mut self) -> Result<()> {

    // }

    pub fn surfel_map_config(&self) -> &SurfelMapConfig {
        &self.surfel_map_config
    }

    pub fn surfel_config(&self) -> &SurfelConfig {
        &self.surfel_config
    }

    pub fn surfels(&self) -> impl Iterator<Item = &Surfel> {
        self.surfels.values()
    }
}
