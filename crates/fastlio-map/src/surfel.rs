use std::time::Instant;

use anyhow::{Result, bail};
use fastlio_types::{Mat3, PointXYZI, SurfelConfig, SurfelMapConfig, Vec3};
use hashbrown::HashMap;
use slotmap::{Key, SlotMap, new_key_type};
use smallvec::SmallVec;

use crate::surfel_types::{GeometryClass, Surfel, VoxelKey};
use crate::{
    PlaneFit, PlaneFitError, PointToPlaneConfig, PointToPlaneError, PointToPlaneMatch,
    PointToPlaneMatchTimings, point_to_plane_weight,
};

new_key_type! { pub struct SurfelId; }

#[derive(Default)]
pub struct SurfelMap {
    buckets: HashMap<u64, SmallVec<[SurfelId; 4]>>,
    surfels: SlotMap<SurfelId, Surfel>,
    surfel_map_config: SurfelMapConfig,
    surfel_config: SurfelConfig,
}

#[derive(Default)]
pub struct SurfelMapQueryScratch {
    seen_generation: Vec<u32>,
    current_generation: u32,
    candidates: SmallVec<[SurfelId; 64]>,
    last_raw_candidate_count: usize,
    last_stats: SurfelMapQueryStats,
}

/// Planar surface observation from the surfel map.
///
/// All coordinates are world/map-frame values in meters. The residual sign is:
/// `signed_residual = norm_w.dot(query_w - mean_w)`.
#[derive(Debug, Clone)]
pub struct SurfelObservation {
    pub surfel_id: SurfelId,
    pub mean_w: Vec3<f64>,
    pub norm_w: Vec3<f64>,
    pub eigenvalues: Vec3<f64>,
    pub plane_distance: f64,
    pub planarity: f64,
    pub signed_residual: f64,
    pub is_growing_weak_constraint: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CandidateIdStats {
    pub raw_count: usize,
    pub unique_count: usize,
    pub duplicate_count: usize,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SurfelMapQueryStats {
    pub primary_raw_candidates: usize,
    pub primary_unique_candidates: usize,
    pub fallback_raw_candidates: usize,
    pub fallback_unique_candidates: usize,
    pub fallback_queries: usize,
    pub fallback_hits: usize,
    pub planar_candidates: usize,
    pub growing_candidates: usize,
    pub accepted_growing_weak: usize,
}

/// One surfel as a renderable point for the three.js surfel viewer.
///
/// Matches the dev-branch `SurfelPcdPoint` layout: centroid position, point
/// count as intensity, plane normal, and geometry class id in `GeometryClass`
/// order (Plane=0, Line=1, Scatter=2, Degenerate=3, Growing=4).
#[derive(Debug, Clone, Copy)]
pub struct SurfelOutputPoint {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
    pub normal_x: f32,
    pub normal_y: f32,
    pub normal_z: f32,
    pub class_id: f32,
}

impl SurfelMap {
    pub fn new(map_config: SurfelMapConfig, surfel_config: SurfelConfig) -> Self {
        Self {
            buckets: HashMap::new(),
            surfels: SlotMap::with_key(),
            surfel_map_config: map_config,
            surfel_config,
        }
    }

    pub fn len(&self) -> usize {
        self.surfels.len()
    }

    pub fn is_empty(&self) -> bool {
        self.surfels.is_empty()
    }

    pub fn surfel_point_count(&self) -> usize {
        self.surfels.values().map(|surfel| surfel.count).sum()
    }

    pub fn output_points(&self) -> Vec<PointXYZI> {
        self.surfels
            .values()
            .map(|surfel| PointXYZI {
                x: surfel.mean_w.x as f32,
                y: surfel.mean_w.y as f32,
                z: surfel.mean_w.z as f32,
                intensity: surfel.count.min(u16::MAX as usize) as f32,
            })
            .collect()
    }

    /// Centroid + normal + geometry class for the three.js surfel viewer
    /// (dev-branch compatible binary PCD with `normal_*` and `class_id`).
    pub fn output_surfel_points(&self) -> Vec<SurfelOutputPoint> {
        self.surfels
            .values()
            .map(|surfel| {
                let normal = surfel.eigenvectors.column(0);
                SurfelOutputPoint {
                    x: surfel.mean_w.x as f32,
                    y: surfel.mean_w.y as f32,
                    z: surfel.mean_w.z as f32,
                    intensity: surfel.count.min(u16::MAX as usize) as f32,
                    normal_x: normal.x as f32,
                    normal_y: normal.y as f32,
                    normal_z: normal.z as f32,
                    class_id: surfel.geometry_class(&self.surfel_config).class_id() as f32,
                }
            })
            .collect()
    }

    pub fn surfel_map_config(&self) -> &SurfelMapConfig {
        &self.surfel_map_config
    }

    pub fn surfel_config(&self) -> &SurfelConfig {
        &self.surfel_config
    }

    pub fn surfels(&self) -> impl Iterator<Item = &Surfel> {
        self.surfels.values()
    }

    pub fn create_query_scratch(&self) -> SurfelMapQueryScratch {
        SurfelMapQueryScratch::default()
    }

    pub fn insert<I>(&mut self, points: I) -> Result<()>
    where
        I: IntoIterator<Item = PointXYZI>,
    {
        let radius = self.surfel_map_config.insert_search_radius;
        if radius < 0 {
            bail!("surfel insert_search_radius must be non-negative");
        }

        let mut scratch = self.create_query_scratch();
        for point in points {
            if !point.is_valid() {
                continue;
            }
            let voxel_key = VoxelKey::new(&point, self.surfel_map_config.voxel_size)?;
            self.fill_candidate_ids_with_scratch(voxel_key, radius, &mut scratch);
            let mut best_mature: Option<(SurfelId, f64)> = None;
            let mut best_growing: Option<(SurfelId, f64)> = None;

            for id in scratch.candidates.iter().copied() {
                let Some(surfel) = self.surfels.get(id) else {
                    continue;
                };
                let in_support = if surfel.is_growing(self.surfel_config.min_mature_surfel_count) {
                    surfel.within_growing_radius(&point, &self.surfel_config)
                } else {
                    surfel.within_support(&point, &self.surfel_config)
                };
                if !in_support {
                    continue;
                }

                if surfel.count >= self.surfel_config.min_mature_surfel_count {
                    let score = surfel.plane_distance(&point) as f64;
                    if best_mature.is_none_or(|(_, best)| score < best) {
                        best_mature = Some((id, score));
                    }
                } else {
                    let score = (point.to_vec3_f64() - surfel.mean_w).norm_squared();
                    if best_growing.is_none_or(|(_, best)| score < best) {
                        best_growing = Some((id, score));
                    }
                }
            }

            if let Some((id, _)) = best_mature.or(best_growing) {
                self.update_surfel(id, &point)?;
            } else {
                self.create_surfel(&point)?;
            }
        }

        Ok(())
    }

    pub fn query(&self, point: &PointXYZI) -> Result<Option<SurfelObservation>> {
        let mut scratch = self.create_query_scratch();
        self.query_with_scratch(point, &mut scratch)
    }

    pub fn query_with_scratch(
        &self,
        point: &PointXYZI,
        scratch: &mut SurfelMapQueryScratch,
    ) -> Result<Option<SurfelObservation>> {
        scratch.last_stats = SurfelMapQueryStats::default();
        if let Some(observation) =
            self.query_with_radius(point, self.surfel_map_config.search_radius, scratch, false)?
        {
            return Ok(Some(observation));
        }

        if let Some(radius) = self.surfel_map_config.fallback_search_radius
            && radius > self.surfel_map_config.search_radius
        {
            scratch.last_stats.fallback_queries += 1;
            let observation = self.query_with_radius(point, radius, scratch, true)?;
            if observation.is_some() {
                scratch.last_stats.fallback_hits += 1;
            }
            return Ok(observation);
        }

        Ok(None)
    }

    fn query_with_radius(
        &self,
        point: &PointXYZI,
        radius: i32,
        scratch: &mut SurfelMapQueryScratch,
        is_fallback: bool,
    ) -> Result<Option<SurfelObservation>> {
        if !point.is_valid() {
            return Ok(None);
        }
        if radius < 0 {
            bail!("surfel search_radius must be non-negative");
        }

        let voxel_key = VoxelKey::new(point, self.surfel_map_config.voxel_size)?;
        let mut best: Option<(SurfelId, f32)> = None;
        let mut best_growing: Option<(SurfelId, f64)> = None;
        let candidate_stats = self.fill_candidate_ids_with_scratch(voxel_key, radius, scratch);
        if is_fallback {
            scratch.last_stats.fallback_raw_candidates += candidate_stats.raw_count;
            scratch.last_stats.fallback_unique_candidates += candidate_stats.unique_count;
        } else {
            scratch.last_stats.primary_raw_candidates += candidate_stats.raw_count;
            scratch.last_stats.primary_unique_candidates += candidate_stats.unique_count;
        }
        let candidate_ids = scratch.candidates.clone();
        let mut planar_candidates = 0;
        let mut growing_candidates = 0;
        for id in candidate_ids.iter().copied() {
            let Some(surfel) = self.surfels.get(id) else {
                continue;
            };
            match surfel.geometry_class(&self.surfel_config) {
                GeometryClass::Plane => {
                    planar_candidates += 1;
                    // Cheap absolute distance check first: most candidates are
                    // rejected on plane distance, so deferring the heavier
                    // tangent-support (Mahalanobis with eigenvalue divisions)
                    // until after it saves work on every rejected candidate.
                    if !surfel.within_plane_distance(point, &self.surfel_config)
                        || !surfel.within_tangent_support(point, &self.surfel_config)
                    {
                        continue;
                    }

                    let score = surfel.plane_distance(point);
                    if best.is_none_or(|(_, best_score)| score < best_score) {
                        best = Some((id, score));
                    }
                }
                GeometryClass::Growing => {
                    growing_candidates += 1;
                    if self.surfel_config.allow_growing_constraints
                        && surfel.within_growing_radius(point, &self.surfel_config)
                    {
                        let distance = (point.to_vec3_f64() - surfel.mean_w).norm();
                        if best_growing.is_none_or(|(_, best_distance)| distance < best_distance) {
                            best_growing = Some((id, distance));
                        }
                    }
                }
                _ => {}
            }
        }
        scratch.last_stats.planar_candidates += planar_candidates;
        scratch.last_stats.growing_candidates += growing_candidates;

        if let Some((surfel_id, score)) = best {
            let Some(surfel) = self.surfels.get(surfel_id) else {
                bail!("surfel candidate disappeared during immutable query");
            };
            let norm_w = surfel.eigenvectors.column(0).into_owned().normalize();
            let mean_w = surfel.mean_w;
            let signed_residual = norm_w.dot(&(point.to_vec3_f64() - mean_w));
            return Ok(Some(SurfelObservation {
                surfel_id,
                mean_w,
                norm_w,
                eigenvalues: surfel.eigenvalues,
                plane_distance: score as f64,
                planarity: surfel.planarity(),
                signed_residual,
                is_growing_weak_constraint: false,
            }));
        }

        let Some((surfel_id, distance)) = best_growing else {
            return Ok(None);
        };
        let Some(surfel) = self.surfels.get(surfel_id) else {
            bail!("surfel candidate disappeared during growing query");
        };
        if distance <= 1.0e-9 {
            return Ok(None);
        }
        scratch.last_stats.accepted_growing_weak += 1;
        let mean_w = surfel.mean_w;
        let norm_w = (point.to_vec3_f64() - mean_w) / distance;
        let surface_point_w = mean_w + norm_w * self.surfel_config.growing_radius as f64;
        Ok(Some(SurfelObservation {
            surfel_id,
            mean_w: surface_point_w,
            norm_w,
            eigenvalues: Vec3::repeat(self.surfel_config.growing_radius as f64),
            plane_distance: distance,
            planarity: 0.0,
            signed_residual: distance - self.surfel_config.growing_radius as f64,
            is_growing_weak_constraint: true,
        }))
    }

    pub fn point_to_plane_match_attempt(
        &self,
        scan_point_w: Vec3<f64>,
        config: PointToPlaneConfig,
    ) -> (
        Result<PointToPlaneMatch, PointToPlaneError>,
        PointToPlaneMatchTimings,
    ) {
        let mut scratch = self.create_query_scratch();
        self.point_to_plane_match_attempt_with_scratch(scan_point_w, config, &mut scratch)
    }

    pub fn point_to_plane_match_attempt_with_scratch(
        &self,
        scan_point_w: Vec3<f64>,
        config: PointToPlaneConfig,
        scratch: &mut SurfelMapQueryScratch,
    ) -> (
        Result<PointToPlaneMatch, PointToPlaneError>,
        PointToPlaneMatchTimings,
    ) {
        let mut timings = PointToPlaneMatchTimings::default();
        if !scan_point_w.x.is_finite() || !scan_point_w.y.is_finite() || !scan_point_w.z.is_finite()
        {
            return (Err(PointToPlaneError::NonFiniteScanPoint), timings);
        }
        if config.nearest_count == 0
            || !config.max_neighbour_distance.is_finite()
            || config.max_neighbour_distance < 0.0
            || !config.max_absolute_residual.is_finite()
            || config.max_absolute_residual < 0.0
        {
            return (Err(PointToPlaneError::InvalidConfig), timings);
        }

        let point = PointXYZI {
            x: scan_point_w.x as f32,
            y: scan_point_w.y as f32,
            z: scan_point_w.z as f32,
            intensity: 0.0,
        };

        let nearest_start = Instant::now();
        let observation = match self.query_with_scratch(&point, scratch) {
            Ok(Some(observation)) => observation,
            Ok(None) => {
                timings.nearest = nearest_start.elapsed();
                return (Err(PointToPlaneError::NoPlanarSurfel), timings);
            }
            Err(_) => {
                timings.nearest = nearest_start.elapsed();
                return (Err(PointToPlaneError::InvalidConfig), timings);
            }
        };
        timings.nearest = nearest_start.elapsed();

        let residual_start = Instant::now();
        let plane = PlaneFit {
            centroid_w: observation.mean_w,
            normal_w: observation.norm_w,
            offset: -observation.norm_w.dot(&observation.mean_w),
            eigenvalues: observation.eigenvalues,
            planarity_ratio: observation.planarity,
        };
        let residual = plane.normal_w.dot(&scan_point_w) + plane.offset;
        if residual.abs() > config.max_absolute_residual {
            timings.residual = residual_start.elapsed();
            return (
                Err(PointToPlaneError::ResidualTooLarge {
                    residual,
                    max_absolute_residual: config.max_absolute_residual,
                }),
                timings,
            );
        }
        let weight = if observation.is_growing_weak_constraint {
            let residual_score = if config.max_absolute_residual <= 0.0 {
                1.0
            } else {
                1.0 - residual.abs() / config.max_absolute_residual
            };
            residual_score.clamp(0.0, 1.0) * self.surfel_config.growing_constraint_weight as f64
        } else {
            point_to_plane_weight(residual, plane.planarity_ratio, &config)
        };
        timings.residual = residual_start.elapsed();

        (
            Ok(PointToPlaneMatch {
                plane,
                residual,
                weight,
            }),
            timings,
        )
    }

    #[cfg(test)]
    fn candidate_ids(&self, key: VoxelKey, radius: i32) -> SmallVec<[SurfelId; 64]> {
        let mut scratch = self.create_query_scratch();
        self.fill_candidate_ids_with_scratch(key, radius, &mut scratch);
        scratch.candidates.clone()
    }

    fn fill_candidate_ids_with_scratch(
        &self,
        key: VoxelKey,
        radius: i32,
        scratch: &mut SurfelMapQueryScratch,
    ) -> CandidateIdStats {
        scratch.begin_query();
        for surfel_id in self.candidate_bucket_ids(key, radius) {
            scratch.push_candidate_if_unseen(surfel_id);
        }
        CandidateIdStats {
            raw_count: scratch.last_raw_candidate_count,
            unique_count: scratch.candidates.len(),
            duplicate_count: scratch
                .last_raw_candidate_count
                .saturating_sub(scratch.candidates.len()),
        }
    }

    #[cfg(test)]
    fn candidate_id_stats(&self, key: VoxelKey, radius: i32) -> CandidateIdStats {
        let mut scratch = self.create_query_scratch();
        self.fill_candidate_ids_with_scratch(key, radius, &mut scratch)
    }

    fn candidate_bucket_ids(
        &self,
        key: VoxelKey,
        radius: i32,
    ) -> impl Iterator<Item = SurfelId> + '_ {
        let buckets = &self.buckets;
        (key.x - radius..=key.x + radius)
            .flat_map(move |x| {
                (key.y - radius..=key.y + radius).flat_map(move |y| {
                    (key.z - radius..=key.z + radius).map(move |z| VoxelKey { x, y, z }.pack())
                })
            })
            .filter_map(move |key| buckets.get(&key))
            .flat_map(|bucket| bucket.iter().copied())
    }

    fn create_surfel(&mut self, point: &PointXYZI) -> Result<SurfelId> {
        let id = self.surfels.insert(Surfel::from_first_point(point));
        self.reindex_surfel(id)?;
        Ok(id)
    }

    fn update_surfel(&mut self, id: SurfelId, point: &PointXYZI) -> Result<()> {
        let surfel = self
            .surfels
            .get_mut(id)
            .ok_or_else(|| anyhow::anyhow!("surfel id missing during update"))?;
        let mut needs_reindex = surfel.is_growing(self.surfel_config.min_mature_surfel_count);

        surfel.count += 1;
        let delta = point.to_vec3_f64() - surfel.mean_w;
        let n = surfel.count as f64;
        surfel.mean_w += delta / n;
        surfel.m2 += delta * delta.transpose() * ((n - 1.0) / n);
        if surfel.count >= self.surfel_config.min_mature_surfel_count
            && (surfel.count - surfel.last_refit) > helper_interval(surfel.count)
        {
            let eigen = (surfel.m2 / (surfel.count - 1) as f64).symmetric_eigen();
            let mut eigen_pairs = [
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
            eigen_pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
            surfel.eigenvectors =
                Mat3::from_columns(&[eigen_pairs[0].1, eigen_pairs[1].1, eigen_pairs[2].1]);
            surfel.eigenvalues = Vec3::new(eigen_pairs[0].0, eigen_pairs[1].0, eigen_pairs[2].0);
            surfel.last_refit = surfel.count;
            needs_reindex = true;
        }

        if needs_reindex {
            self.reindex_surfel(id)?;
        }

        Ok(())
    }

    fn reindex_surfel(&mut self, id: SurfelId) -> Result<()> {
        let (old_keys, new_keys) = {
            let surfel = self
                .surfels
                .get(id)
                .ok_or_else(|| anyhow::anyhow!("surfel id missing during reindex"))?;
            let extent = if surfel.is_growing(self.surfel_config.min_mature_surfel_count) {
                Vec3::repeat(self.surfel_config.growing_radius as f64)
            } else {
                surfel.support_aabb_extent(&self.surfel_config)
            };

            let min_key = VoxelKey::from_vec3(
                (surfel.mean_w - extent).cast(),
                self.surfel_map_config.voxel_size,
            )?;
            let max_key = VoxelKey::from_vec3(
                (surfel.mean_w + extent).cast(),
                self.surfel_map_config.voxel_size,
            )?;
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

        for key in old_keys {
            let remove_bucket = if let Some(bucket) = self.buckets.get_mut(&key) {
                bucket.retain(|surfel_id| *surfel_id != id);
                bucket.is_empty()
            } else {
                false
            };
            if remove_bucket {
                self.buckets.remove(&key);
            }
        }
        for &key in &new_keys {
            self.buckets.entry(key).or_default().push(id);
        }
        self.surfels
            .get_mut(id)
            .ok_or_else(|| anyhow::anyhow!("surfel id missing after reindex"))?
            .indexed_voxels = new_keys;

        Ok(())
    }
}

impl SurfelMapQueryScratch {
    fn begin_query(&mut self) {
        self.current_generation = self.current_generation.wrapping_add(1);
        if self.current_generation == 0 {
            self.seen_generation.fill(0);
            self.current_generation = 1;
        }
        self.last_raw_candidate_count = 0;
        self.candidates.clear();
    }

    fn push_candidate_if_unseen(&mut self, surfel_id: SurfelId) {
        self.last_raw_candidate_count += 1;
        let index = surfel_slot_index(surfel_id);
        if self.seen_generation.len() <= index {
            self.seen_generation.resize(index + 1, 0);
        }
        if self.seen_generation[index] == self.current_generation {
            return;
        }

        self.seen_generation[index] = self.current_generation;
        self.candidates.push(surfel_id);
    }

    pub fn last_stats(&self) -> SurfelMapQueryStats {
        self.last_stats
    }
}

fn surfel_slot_index(surfel_id: SurfelId) -> usize {
    (surfel_id.data().as_ffi() & 0xffff_ffff) as usize
}

fn helper_interval(count: usize) -> usize {
    match count {
        0..=16 => 4,
        17..=64 => 8,
        _ => 16,
    }
}

impl From<PlaneFitError> for PointToPlaneError {
    fn from(value: PlaneFitError) -> Self {
        PointToPlaneError::PlaneFit(value)
    }
}

#[cfg(test)]
mod tests {
    use fastlio_types::{SurfelConfig, SurfelMapConfig};

    use super::*;

    fn point(x: f32, y: f32, z: f32) -> PointXYZI {
        PointXYZI {
            x,
            y,
            z,
            intensity: 1.0,
        }
    }

    #[test]
    fn surfel_map_merges_planar_points_into_queryable_plane() {
        let mut map = SurfelMap::new(
            SurfelMapConfig {
                voxel_size: 0.5,
                insert_search_radius: 1,
                search_radius: 1,
                fallback_search_radius: None,
            },
            SurfelConfig {
                min_mature_surfel_count: 4,
                max_plane_distance: 0.2,
                growing_radius: 1.0,
                ..SurfelConfig::default()
            },
        );
        map.insert(vec![
            point(-0.3, -0.3, 0.0),
            point(0.3, -0.3, 0.0),
            point(-0.3, 0.3, 0.0),
            point(0.3, 0.3, 0.0),
            point(0.0, 0.0, 0.0),
            point(0.1, 0.2, 0.0),
        ])
        .unwrap();

        let observation = map.query(&point(0.1, 0.1, 0.03)).unwrap().unwrap();

        assert!(observation.norm_w.z.abs() > 1.0 - 1.0e-6);
        assert!(observation.plane_distance < 0.05);
        assert!(observation.planarity < 1.0e-6);
    }

    #[test]
    fn surfel_map_point_to_plane_match_reuses_stored_plane() {
        let mut map = SurfelMap::new(
            SurfelMapConfig {
                voxel_size: 0.5,
                insert_search_radius: 1,
                search_radius: 1,
                fallback_search_radius: None,
            },
            SurfelConfig {
                min_mature_surfel_count: 4,
                max_plane_distance: 0.2,
                growing_radius: 1.0,
                ..SurfelConfig::default()
            },
        );
        map.insert(vec![
            point(-0.3, -0.3, 0.0),
            point(0.3, -0.3, 0.0),
            point(-0.3, 0.3, 0.0),
            point(0.3, 0.3, 0.0),
            point(0.0, 0.0, 0.0),
        ])
        .unwrap();

        let (matched, timings) = map.point_to_plane_match_attempt(
            Vec3::new(0.0, 0.0, 0.04),
            PointToPlaneConfig {
                max_absolute_residual: 0.2,
                ..PointToPlaneConfig::default()
            },
        );

        let matched = matched.unwrap();
        assert!(matched.plane.normal_w.z.abs() > 1.0 - 1.0e-6);
        assert!(matched.residual.abs() < 0.05);
        assert!(matched.weight > 0.0);
        assert_eq!(timings.plane_fit, std::time::Duration::ZERO);
    }

    #[test]
    fn radius_candidate_query_reports_duplicate_surfel_ids() {
        let mut map = SurfelMap::new(
            SurfelMapConfig {
                voxel_size: 0.2,
                insert_search_radius: 1,
                search_radius: 1,
                fallback_search_radius: None,
            },
            SurfelConfig {
                min_mature_surfel_count: 4,
                max_plane_distance: 0.2,
                growing_radius: 1.0,
                ..SurfelConfig::default()
            },
        );
        map.insert(vec![
            point(-0.5, -0.5, 0.0),
            point(0.5, -0.5, 0.0),
            point(-0.5, 0.5, 0.0),
            point(0.5, 0.5, 0.0),
            point(0.0, 0.0, 0.0),
            point(0.2, 0.1, 0.0),
        ])
        .unwrap();

        let key = VoxelKey::new(&point(0.0, 0.0, 0.03), 0.2).unwrap();
        let radius_one_stats = map.candidate_id_stats(key, 1);
        let radius_one_unique = map.candidate_ids(key, 1);
        let radius_two_stats = map.candidate_id_stats(key, 2);
        let radius_two_unique = map.candidate_ids(key, 2);

        assert_eq!(radius_one_stats.unique_count, radius_one_unique.len());
        assert_eq!(radius_one_stats.raw_count, 54);
        assert_eq!(radius_one_stats.unique_count, 2);
        assert_eq!(radius_one_stats.duplicate_count, 52);
        assert_eq!(radius_two_stats.unique_count, radius_two_unique.len());
        assert_eq!(radius_two_stats.raw_count, 250);
        assert_eq!(radius_two_stats.unique_count, 2);
        assert_eq!(radius_two_stats.duplicate_count, 248);
    }

    #[test]
    fn growing_surfel_constraint_is_explicit_and_low_weight() {
        let map_config = SurfelMapConfig {
            voxel_size: 0.5,
            insert_search_radius: 1,
            search_radius: 1,
            fallback_search_radius: None,
        };
        let base_config = SurfelConfig {
            min_mature_surfel_count: 8,
            growing_radius: 1.0,
            growing_constraint_weight: 0.1,
            ..SurfelConfig::default()
        };
        let mut map = SurfelMap::new(map_config, base_config.clone());
        map.insert(vec![point(0.0, 0.0, 0.0), point(0.1, 0.0, 0.0)])
            .unwrap();

        let mut scratch = map.create_query_scratch();
        let (disabled, _) = map.point_to_plane_match_attempt_with_scratch(
            Vec3::new(0.5, 0.0, 0.0),
            PointToPlaneConfig::default(),
            &mut scratch,
        );
        assert_eq!(disabled.unwrap_err(), PointToPlaneError::NoPlanarSurfel);
        assert_eq!(scratch.last_stats().accepted_growing_weak, 0);

        let mut enabled_config = base_config;
        enabled_config.allow_growing_constraints = true;
        let mut map = SurfelMap::new(map_config, enabled_config);
        map.insert(vec![point(0.0, 0.0, 0.0), point(0.1, 0.0, 0.0)])
            .unwrap();

        let mut scratch = map.create_query_scratch();
        let (enabled, _) = map.point_to_plane_match_attempt_with_scratch(
            Vec3::new(0.5, 0.0, 0.0),
            PointToPlaneConfig {
                max_absolute_residual: 1.0,
                ..PointToPlaneConfig::default()
            },
            &mut scratch,
        );
        let enabled = enabled.unwrap();
        assert!(enabled.weight > 0.0);
        assert!(enabled.weight <= 0.1);
        assert_eq!(scratch.last_stats().accepted_growing_weak, 1);
        assert!(scratch.last_stats().growing_candidates > 0);
    }
}
