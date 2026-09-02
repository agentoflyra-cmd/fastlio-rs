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
    /// Mature surfels touched by recent inserts, waiting for an incremental
    /// merge check against voxel neighbours. Spreads merge cost across frames
    /// instead of a periodic full-map sweep (GC-pause style).
    pending_merge: hashbrown::HashSet<SurfelId>,
}

/// Growing-fragment merge test: two immature surfels within `growing_radius`
/// of each other combine so their counts sum and they may reach maturity.
/// Growing eigenvectors are unreliable (few points, rare refits), so only
/// spatial proximity is checked. A post-merge refit + geometry classification
/// guards quality once the merged patch matures.
pub fn should_merge_growing(a: &Surfel, b: &Surfel, growing_radius: f32) -> bool {
    (a.mean_w - b.mean_w).norm() <= growing_radius as f64
}

/// Merge compatibility test for two mature surfels.
///
/// - Normal angle <= 5 deg: planes are rigid; larger angles mean distinct
///   surfaces. Alignment uses |cos| because eigenvectors carry arbitrary sign
///   (the same surface may store +n or -n).
/// - Coplanarity <= 0.1 m along the normal: within wall thickness/flatness.
/// - Centroid distance <= 2.0 m: locality guard against distant merges.
///
/// Coplanarity under these conditions keeps the merged patch plane-like, so
/// no post-merge planarity re-check is needed.
pub fn should_merge(a: &Surfel, b: &Surfel) -> bool {
    let normal_alignment = a
        .eigenvectors
        .column(0)
        .dot(&b.eigenvectors.column(0))
        .abs();
    if normal_alignment < 5.0_f64.to_radians().cos() {
        return false;
    }
    let plane_dist = (a.mean_w - b.mean_w).dot(&a.eigenvectors.column(0)).abs();
    if plane_dist > 0.1 {
        return false;
    }
    (a.mean_w - b.mean_w).norm() <= 2.0
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

/// Line observation from a line-class surfel.
///
/// The line is stored in world/map frame as `mean_w + s * direction_w`.
/// Point-to-line residual is represented by two scalar residuals along
/// `normal0_w` and `normal1_w`, both perpendicular to `direction_w`.
#[derive(Debug, Clone)]
pub struct SurfelLineObservation {
    pub surfel_id: SurfelId,
    pub mean_w: Vec3<f64>,
    pub direction_w: Vec3<f64>,
    pub normal0_w: Vec3<f64>,
    pub normal1_w: Vec3<f64>,
    pub residual0: f64,
    pub residual1: f64,
    pub distance: f64,
    pub linearity: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfelConstraintKind {
    Plane,
    Line,
    GrowingWeak,
}

/// Read-only consistency check for a point about to be inserted into a surfel map.
///
/// The query point and returned distance are both in world/map frame meters.
/// This is diagnostic-only: it reuses the same surfel geometry gates as
/// scan-to-map association and does not mutate the map.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfelInsertionConsistency {
    pub kind: SurfelConstraintKind,
    pub distance: f64,
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
    pub line_candidates: usize,
    pub growing_candidates: usize,
    pub accepted_line_constraints: usize,
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
            pending_merge: hashbrown::HashSet::new(),
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
        self.query_with_scratch_allow_growing(
            point,
            scratch,
            self.surfel_config.allow_growing_constraints,
        )
    }

    fn query_with_scratch_allow_growing(
        &self,
        point: &PointXYZI,
        scratch: &mut SurfelMapQueryScratch,
        allow_growing: bool,
    ) -> Result<Option<SurfelObservation>> {
        if let Some(observation) = self.query_with_radius(
            point,
            self.surfel_map_config.search_radius,
            scratch,
            false,
            allow_growing,
        )? {
            return Ok(Some(observation));
        }

        if let Some(radius) = self.surfel_map_config.fallback_search_radius
            && radius > self.surfel_map_config.search_radius
        {
            scratch.last_stats.fallback_queries += 1;
            let observation =
                self.query_with_radius(point, radius, scratch, true, allow_growing)?;
            if observation.is_some() {
                scratch.last_stats.fallback_hits += 1;
            }
            return Ok(observation);
        }

        Ok(None)
    }

    pub fn insertion_consistency_with_scratch(
        &self,
        point: &PointXYZI,
        scratch: &mut SurfelMapQueryScratch,
    ) -> Result<Option<SurfelInsertionConsistency>> {
        if let Some(observation) = self.query_with_scratch(point, scratch)? {
            let kind = if observation.is_growing_weak_constraint {
                SurfelConstraintKind::GrowingWeak
            } else {
                SurfelConstraintKind::Plane
            };
            return Ok(Some(SurfelInsertionConsistency {
                kind,
                distance: observation.signed_residual.abs(),
            }));
        }

        if !self.surfel_config.enable_line_constraints {
            return Ok(None);
        }
        let Some(observation) = self.query_line(point.to_vec3_f64(), scratch)? else {
            return Ok(None);
        };
        Ok(Some(SurfelInsertionConsistency {
            kind: SurfelConstraintKind::Line,
            distance: observation.distance,
        }))
    }

    fn query_with_radius(
        &self,
        point: &PointXYZI,
        radius: i32,
        scratch: &mut SurfelMapQueryScratch,
        is_fallback: bool,
        allow_growing: bool,
    ) -> Result<Option<SurfelObservation>> {
        if !point.is_valid() {
            return Ok(None);
        }
        if radius < 0 {
            bail!("surfel search_radius must be non-negative");
        }
        if self.surfel_config.allow_growing_constraints
            && (!self
                .surfel_config
                .max_growing_constraint_distance
                .is_finite()
                || self.surfel_config.max_growing_constraint_distance <= 0.0
                || !self.surfel_config.growing_constraint_weight.is_finite()
                || self.surfel_config.growing_constraint_weight < 0.0)
        {
            bail!("invalid surfel growing constraint config");
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
                    if allow_growing && surfel.within_growing_radius(point, &self.surfel_config) {
                        let distance = (point.to_vec3_f64() - surfel.mean_w).norm();
                        if distance > self.surfel_config.max_growing_constraint_distance as f64 {
                            continue;
                        }
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
        Ok(Some(SurfelObservation {
            surfel_id,
            mean_w,
            norm_w,
            eigenvalues: Vec3::repeat(self.surfel_config.max_growing_constraint_distance as f64),
            plane_distance: distance,
            planarity: 0.0,
            signed_residual: distance,
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
        scratch.last_stats = SurfelMapQueryStats::default();
        self.point_to_plane_match_attempt_with_scratch_allow_growing(
            scan_point_w,
            config,
            scratch,
            self.surfel_config.allow_growing_constraints,
        )
    }

    fn point_to_plane_match_attempt_with_scratch_allow_growing(
        &self,
        scan_point_w: Vec3<f64>,
        config: PointToPlaneConfig,
        scratch: &mut SurfelMapQueryScratch,
        allow_growing: bool,
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
        let observation = match self.query_with_scratch_allow_growing(
            &point,
            scratch,
            allow_growing && self.surfel_config.allow_growing_constraints,
        ) {
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
                let max_residual =
                    self.surfel_config
                        .max_growing_constraint_distance
                        .min(config.max_absolute_residual as f32) as f64;
                1.0 - residual.abs() / max_residual
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

    pub fn point_to_plane_or_line_matches_attempt_with_scratch(
        &self,
        scan_point_w: Vec3<f64>,
        config: PointToPlaneConfig,
        scratch: &mut SurfelMapQueryScratch,
    ) -> (
        Result<SmallVec<[PointToPlaneMatch; 2]>, PointToPlaneError>,
        PointToPlaneMatchTimings,
    ) {
        scratch.last_stats = SurfelMapQueryStats::default();
        let (plane_match, mut timings) = self
            .point_to_plane_match_attempt_with_scratch_allow_growing(
                scan_point_w,
                config,
                scratch,
                false,
            );
        match plane_match {
            Ok(plane_match) => {
                let mut matches = SmallVec::new();
                matches.push(plane_match);
                (Ok(matches), timings)
            }
            Err(PointToPlaneError::NoPlanarSurfel)
                if self.surfel_config.enable_line_constraints =>
            {
                let nearest_start = Instant::now();
                let line_observation = match self.query_line(scan_point_w, scratch) {
                    Ok(Some(observation)) => observation,
                    Ok(None) => {
                        timings.nearest += nearest_start.elapsed();
                        return (Err(PointToPlaneError::NoPlanarSurfel), timings);
                    }
                    Err(_) => {
                        timings.nearest += nearest_start.elapsed();
                        return (Err(PointToPlaneError::InvalidConfig), timings);
                    }
                };
                timings.nearest += nearest_start.elapsed();

                let residual_start = Instant::now();
                if line_observation.distance > self.surfel_config.max_line_distance as f64 {
                    timings.residual += residual_start.elapsed();
                    return (
                        Err(PointToPlaneError::ResidualTooLarge {
                            residual: line_observation.distance,
                            max_absolute_residual: self.surfel_config.max_line_distance as f64,
                        }),
                        timings,
                    );
                }
                let residual_score = if self.surfel_config.max_line_distance <= 0.0 {
                    1.0
                } else {
                    1.0 - line_observation.distance / self.surfel_config.max_line_distance as f64
                };
                let linearity_score = if self.surfel_config.min_linearity >= 1.0 {
                    1.0
                } else {
                    (line_observation.linearity - self.surfel_config.min_linearity as f64)
                        / (1.0 - self.surfel_config.min_linearity as f64)
                };
                let weight = residual_score.clamp(0.0, 1.0)
                    * linearity_score.clamp(0.0, 1.0)
                    * self.surfel_config.line_constraint_weight as f64;
                let eigenvalues = Vec3::new(0.0, 0.0, line_observation.distance);
                let mut matches = SmallVec::new();
                for (normal_w, residual) in [
                    (line_observation.normal0_w, line_observation.residual0),
                    (line_observation.normal1_w, line_observation.residual1),
                ] {
                    matches.push(PointToPlaneMatch {
                        plane: PlaneFit {
                            centroid_w: line_observation.mean_w,
                            normal_w,
                            offset: -normal_w.dot(&line_observation.mean_w),
                            eigenvalues,
                            planarity_ratio: 0.0,
                        },
                        residual,
                        weight,
                    });
                }
                scratch.last_stats.accepted_line_constraints += matches.len();
                timings.residual += residual_start.elapsed();
                (Ok(matches), timings)
            }
            Err(PointToPlaneError::NoPlanarSurfel)
                if self.surfel_config.allow_growing_constraints =>
            {
                let (growing_match, growing_timings) = self
                    .point_to_plane_match_attempt_with_scratch_allow_growing(
                        scan_point_w,
                        config,
                        scratch,
                        true,
                    );
                timings.nearest += growing_timings.nearest;
                timings.plane_fit += growing_timings.plane_fit;
                timings.residual += growing_timings.residual;
                match growing_match {
                    Ok(growing_match) => {
                        let mut matches = SmallVec::new();
                        matches.push(growing_match);
                        (Ok(matches), timings)
                    }
                    Err(err) => (Err(err), timings),
                }
            }
            Err(err) => (Err(err), timings),
        }
    }

    fn query_line(
        &self,
        scan_point_w: Vec3<f64>,
        scratch: &mut SurfelMapQueryScratch,
    ) -> Result<Option<SurfelLineObservation>> {
        if !scan_point_w.iter().all(|value| value.is_finite()) {
            return Ok(None);
        }
        if !self.surfel_config.max_line_distance.is_finite()
            || self.surfel_config.max_line_distance < 0.0
            || !self.surfel_config.line_constraint_weight.is_finite()
            || self.surfel_config.line_constraint_weight < 0.0
        {
            bail!("invalid surfel line constraint config");
        }

        let point = PointXYZI {
            x: scan_point_w.x as f32,
            y: scan_point_w.y as f32,
            z: scan_point_w.z as f32,
            intensity: 0.0,
        };
        self.query_line_with_radius(&point, self.surfel_map_config.search_radius, scratch, false)?
            .map_or_else(
                || {
                    if let Some(radius) = self.surfel_map_config.fallback_search_radius
                        && radius > self.surfel_map_config.search_radius
                    {
                        scratch.last_stats.fallback_queries += 1;
                        let observation =
                            self.query_line_with_radius(&point, radius, scratch, true)?;
                        if observation.is_some() {
                            scratch.last_stats.fallback_hits += 1;
                        }
                        Ok(observation)
                    } else {
                        Ok(None)
                    }
                },
                |observation| Ok(Some(observation)),
            )
    }

    fn query_line_with_radius(
        &self,
        point: &PointXYZI,
        radius: i32,
        scratch: &mut SurfelMapQueryScratch,
        is_fallback: bool,
    ) -> Result<Option<SurfelLineObservation>> {
        if !point.is_valid() {
            return Ok(None);
        }
        if radius < 0 {
            bail!("surfel search_radius must be non-negative");
        }

        let voxel_key = VoxelKey::new(point, self.surfel_map_config.voxel_size)?;
        let candidate_stats = self.fill_candidate_ids_with_scratch(voxel_key, radius, scratch);
        if is_fallback {
            scratch.last_stats.fallback_raw_candidates += candidate_stats.raw_count;
            scratch.last_stats.fallback_unique_candidates += candidate_stats.unique_count;
        } else {
            scratch.last_stats.primary_raw_candidates += candidate_stats.raw_count;
            scratch.last_stats.primary_unique_candidates += candidate_stats.unique_count;
        }
        let candidate_ids = scratch.candidates.clone();
        let mut line_candidates = 0;
        let mut best: Option<(SurfelId, f64)> = None;
        for id in candidate_ids.iter().copied() {
            let Some(surfel) = self.surfels.get(id) else {
                continue;
            };
            if surfel.geometry_class(&self.surfel_config) != GeometryClass::Line {
                continue;
            }
            line_candidates += 1;
            if !surfel.within_line_distance(point, &self.surfel_config)
                || !surfel.within_line_support(point, &self.surfel_config)
            {
                continue;
            }
            let distance = surfel.line_distance(point);
            if best.is_none_or(|(_, best_distance)| distance < best_distance) {
                best = Some((id, distance));
            }
        }
        scratch.last_stats.line_candidates += line_candidates;

        let Some((surfel_id, distance)) = best else {
            return Ok(None);
        };
        let Some(surfel) = self.surfels.get(surfel_id) else {
            bail!("surfel candidate disappeared during line query");
        };
        let delta = point.to_vec3_f64() - surfel.mean_w;
        let normal0_w = surfel.eigenvectors.column(0).into_owned().normalize();
        let normal1_w = surfel.eigenvectors.column(1).into_owned().normalize();
        Ok(Some(SurfelLineObservation {
            surfel_id,
            mean_w: surfel.mean_w,
            direction_w: surfel.eigenvectors.column(2).into_owned().normalize(),
            normal0_w,
            normal1_w,
            residual0: normal0_w.dot(&delta),
            residual1: normal1_w.dot(&delta),
            distance,
            linearity: surfel.line_quality(),
        }))
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

        // Queue the touched surfel for an incremental merge check against its
        // voxel neighbours. Growing fragments are queued too: neighbouring
        // growing fragments merge (count sums) and may cross the maturity
        // threshold, reducing fragment noise and growing useful planes.
        self.pending_merge.insert(id);

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

    /// Incrementally merge mature surfels touched by recent inserts.
    ///
    /// Processes up to `budget` pending surfels per call, checking each
    /// against its voxel neighbours with [`should_merge`]. Absorbed surfels
    /// are dropped; the absorbing surfel is re-queued so a later pass can keep
    /// growing it. Returns how many surfels were absorbed. Called every frame
    /// with a small budget, the merge cost is spread across frames instead of
    /// running a periodic full-map sweep (GC-pause style).
    pub fn merge_incremental(&mut self, budget: usize) -> usize {
        if budget == 0 || self.pending_merge.is_empty() {
            return 0;
        }
        let min_mature = self.surfel_config.min_mature_surfel_count;
        let growing_radius = self.surfel_config.growing_radius;
        let batch: Vec<SurfelId> = self.pending_merge.iter().take(budget).copied().collect();
        for &id in &batch {
            self.pending_merge.remove(&id);
        }
        let mut merged = 0;
        for &id in &batch {
            if !self.surfels.contains_key(id) {
                continue; // absorbed by an earlier candidate in this batch
            }
            let Some(surfel) = self.surfels.get(id) else {
                continue;
            };
            let id_mature = surfel.count >= min_mature;
            let candidates = self.candidate_surfel_ids_near(surfel.mean_w, 3);
            for candidate in candidates {
                if candidate == id || !self.surfels.contains_key(candidate) {
                    continue;
                }
                let Some(other) = self.surfels.get(candidate) else {
                    continue;
                };
                let other_mature = other.count >= min_mature;
                // Mature-mature pairs use the strict coplanarity test;
                // growing-growing pairs merge on proximity alone (growing
                // eigenvectors are unreliable) to accumulate count toward
                // maturity. Mixed pairs are left to the insert path.
                let compatible = match (id_mature, other_mature) {
                    (true, true) => should_merge(surfel, other),
                    (false, false) => should_merge_growing(surfel, other, growing_radius),
                    _ => false,
                };
                if !compatible {
                    continue;
                }
                let (merged_mean, merged_m2, merged_count) = merge_surfel_stats(
                    surfel.mean_w,
                    surfel.m2,
                    surfel.count,
                    other.mean_w,
                    other.m2,
                    other.count,
                );
                // Absorb `other` into `id` and refit the combined statistics.
                {
                    let dst = self.surfels.get_mut(id).expect("id still present");
                    dst.mean_w = merged_mean;
                    dst.m2 = merged_m2;
                    dst.count = merged_count;
                    dst.last_refit = merged_count;
                    let covariance = merged_m2 / (merged_count - 1) as f64;
                    let eigen = covariance.symmetric_eigen();
                    let mut pairs = [
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
                    pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
                    dst.eigenvectors = Mat3::from_columns(&[pairs[0].1, pairs[1].1, pairs[2].1]);
                    dst.eigenvalues = Vec3::new(pairs[0].0, pairs[1].0, pairs[2].0);
                }
                self.pending_merge.remove(&candidate);
                self.remove_surfel_from_index(candidate);
                self.surfels.remove(candidate);
                merged += 1;
                let _ = self.reindex_surfel(id);
                // Re-queue the grown surfel for further merging in later frames.
                self.pending_merge.insert(id);
                break;
            }
        }
        merged
    }

    fn remove_surfel_from_index(&mut self, id: SurfelId) {
        let Some(surfel) = self.surfels.get(id) else {
            return;
        };
        let keys = surfel.indexed_voxels.clone();
        for key in keys {
            if let Some(bucket) = self.buckets.get_mut(&key) {
                bucket.retain(|sid| *sid != id);
                if bucket.is_empty() {
                    self.buckets.remove(&key);
                }
            }
        }
    }

    fn candidate_surfel_ids_near(&self, mean_w: Vec3<f64>, radius: i32) -> Vec<SurfelId> {
        let mut seen: hashbrown::HashSet<SurfelId> = hashbrown::HashSet::default();
        let mut out = Vec::new();
        if let Ok(key) = VoxelKey::from_vec3(mean_w.cast(), self.surfel_map_config.voxel_size) {
            for x in key.x - radius..=key.x + radius {
                for y in key.y - radius..=key.y + radius {
                    for z in key.z - radius..=key.z + radius {
                        if let Some(bucket) = self.buckets.get(&VoxelKey { x, y, z }.pack()) {
                            for &sid in bucket {
                                if seen.insert(sid) {
                                    out.push(sid);
                                }
                            }
                        }
                    }
                }
            }
        }
        out
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

/// Exact combination of two Welford-style statistics (mean, m2, count).
fn merge_surfel_stats(
    mean_a: Vec3<f64>,
    m2_a: Mat3<f64>,
    count_a: usize,
    mean_b: Vec3<f64>,
    m2_b: Mat3<f64>,
    count_b: usize,
) -> (Vec3<f64>, Mat3<f64>, usize) {
    let na = count_a as f64;
    let nb = count_b as f64;
    let n = na + nb;
    let delta = mean_b - mean_a;
    let mean = (mean_a * na + mean_b * nb) / n;
    let m2 = m2_a + m2_b + delta * delta.transpose() * (na * nb / n);
    (mean, m2, count_a + count_b)
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
    fn incremental_merge_combines_coplanar_neighbours() {
        let mut map = SurfelMap::new(
            SurfelMapConfig {
                voxel_size: 0.5,
                insert_search_radius: 0,
                search_radius: 0,
                fallback_search_radius: None,
            },
            SurfelConfig {
                min_mature_surfel_count: 16,
                max_plane_distance: 0.2,
                growing_radius: 1.0,
                ..SurfelConfig::default()
            },
        );
        // Two dense clusters on the same plane. Cluster A straddles the x
        // voxel boundary (voxel 0.5 m) so it fragments into multiple mature
        // surfels; merge should combine the coplanar fragments.
        let mut points = Vec::new();
        for dx in -12..=12 {
            for dz in -4..=4 {
                points.push(point(0.30 + dx as f32 * 0.03, 0.0, dz as f32 * 0.08));
            }
        }
        for dx in -12..=12 {
            for dz in -4..=4 {
                points.push(point(1.55 + dx as f32 * 0.03, 0.0, dz as f32 * 0.08));
            }
        }
        map.insert(points).unwrap();
        let before = map.len();
        let merged = map.merge_incremental(64);
        let after = map.len();
        assert!(merged > 0, "expected coplanar fragments to merge");
        assert!(after < before, "merge must reduce surfel count");
        // The merged region must remain queryable as a plane.
        let observation = map.query(&point(0.4, 0.0, 0.0)).unwrap();
        assert!(observation.is_some(), "merged surfel should be queryable");
        if let Some(obs) = observation {
            assert!(obs.norm_w.y.abs() > 1.0 - 1.0e-3);
        }
    }

    #[test]
    fn incremental_merge_preserves_insert_index() {
        let mut map = SurfelMap::new(
            SurfelMapConfig {
                voxel_size: 0.5,
                insert_search_radius: 0,
                search_radius: 0,
                fallback_search_radius: None,
            },
            SurfelConfig {
                min_mature_surfel_count: 16,
                max_plane_distance: 0.2,
                growing_radius: 1.0,
                ..SurfelConfig::default()
            },
        );
        let mut points = Vec::new();
        for dx in -12..=12 {
            for dz in -4..=4 {
                points.push(point(0.30 + dx as f32 * 0.03, 0.0, dz as f32 * 0.08));
            }
        }
        map.insert(points).unwrap();
        let after_first = map.len();
        let merged = map.merge_incremental(64);
        assert!(merged > 0);
        // Insert the same points again: every point must be absorbed by an
        // existing surfel (grown), not create new ones.
        let before_second = map.len();
        let mut points = Vec::new();
        for dx in -12..=12 {
            for dz in -4..=4 {
                points.push(point(0.30 + dx as f32 * 0.03, 0.0, dz as f32 * 0.08));
            }
        }
        map.insert(points).unwrap();
        let after_second = map.len();
        assert!(
            after_second <= before_second + 2,
            "re-insert created {} new surfels (was {before_second})",
            after_second.saturating_sub(before_second)
        );
        assert!(
            after_second < after_first + 10,
            "map must not fragment after merge"
        );
    }

    #[test]
    fn incremental_merge_matures_growing_fragments() {
        let mut map = SurfelMap::new(
            SurfelMapConfig {
                voxel_size: 0.5,
                insert_search_radius: 0,
                search_radius: 0,
                fallback_search_radius: None,
            },
            SurfelConfig {
                min_mature_surfel_count: 16,
                max_plane_distance: 0.2,
                growing_radius: 1.0,
                ..SurfelConfig::default()
            },
        );
        // Two small clusters on the same plane, each well below the maturity
        // threshold (16) but within growing_radius of each other.
        let mut points = Vec::new();
        for dx in -2..=2 {
            for dz in -2..=2 {
                points.push(point(0.20 + dx as f32 * 0.05, 0.0, dz as f32 * 0.05));
            }
        }
        for dx in -2..=2 {
            for dz in -2..=2 {
                points.push(point(0.60 + dx as f32 * 0.05, 0.0, dz as f32 * 0.05));
            }
        }
        map.insert(points).unwrap();
        let before = map.len();
        // Merge growing fragments repeatedly until maturity is reached.
        let mut merged_total = 0;
        for _ in 0..10 {
            merged_total += map.merge_incremental(64);
        }
        assert!(merged_total > 0, "expected growing fragments to merge");
        let after = map.len();
        assert!(after < before, "merge must reduce surfel count");
        // The combined patch should have crossed the maturity threshold.
        assert!(
            map.surfels().any(|s| s.count >= 16),
            "merged fragments must reach maturity"
        );
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
            Vec3::new(0.2, 0.0, 0.0),
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
            Vec3::new(0.2, 0.0, 0.0),
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

        let mut scratch = map.create_query_scratch();
        let (too_far, _) = map.point_to_plane_match_attempt_with_scratch(
            Vec3::new(0.5, 0.0, 0.0),
            PointToPlaneConfig {
                max_absolute_residual: 1.0,
                ..PointToPlaneConfig::default()
            },
            &mut scratch,
        );
        assert_eq!(too_far.unwrap_err(), PointToPlaneError::NoPlanarSurfel);
    }

    #[test]
    fn line_surfel_returns_two_scalar_constraints_when_enabled() {
        let mut map = SurfelMap::new(
            SurfelMapConfig {
                voxel_size: 0.5,
                insert_search_radius: 1,
                search_radius: 1,
                fallback_search_radius: None,
            },
            SurfelConfig {
                min_mature_surfel_count: 4,
                max_plane_distance: 0.05,
                enable_line_constraints: true,
                max_line_distance: 0.3,
                line_constraint_weight: 0.5,
                growing_radius: 2.0,
                ..SurfelConfig::default()
            },
        );
        map.insert(vec![
            point(-1.0, 0.0, 0.0),
            point(-0.5, 0.0, 0.0),
            point(0.5, 0.0, 0.0),
            point(1.0, 0.0, 0.0),
            point(0.0, 0.0, 0.0),
        ])
        .unwrap();

        let mut scratch = map.create_query_scratch();
        let (matches, _) = map.point_to_plane_or_line_matches_attempt_with_scratch(
            Vec3::new(0.25, 0.1, 0.05),
            PointToPlaneConfig {
                max_absolute_residual: 0.2,
                ..PointToPlaneConfig::default()
            },
            &mut scratch,
        );
        let matches = matches.unwrap();

        assert_eq!(matches.len(), 2);
        assert_eq!(scratch.last_stats().accepted_line_constraints, 2);
        assert!(scratch.last_stats().line_candidates > 0);
        assert!(matches.iter().all(|matched| matched.weight > 0.0));
        assert!(matches.iter().all(|matched| matched.weight <= 0.5));
    }

    #[test]
    fn insertion_consistency_classifies_plane_and_line_matches() {
        let mut plane_map = SurfelMap::new(
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
                enable_line_constraints: true,
                max_line_distance: 0.3,
                ..SurfelConfig::default()
            },
        );
        plane_map
            .insert(vec![
                point(-0.3, -0.3, 0.0),
                point(0.3, -0.3, 0.0),
                point(-0.3, 0.3, 0.0),
                point(0.3, 0.3, 0.0),
                point(0.0, 0.0, 0.0),
            ])
            .unwrap();
        let mut scratch = plane_map.create_query_scratch();
        let plane = plane_map
            .insertion_consistency_with_scratch(&point(0.1, 0.1, 0.03), &mut scratch)
            .unwrap()
            .unwrap();
        assert_eq!(plane.kind, SurfelConstraintKind::Plane);
        assert!(plane.distance < 0.05);

        let mut line_map = SurfelMap::new(
            SurfelMapConfig {
                voxel_size: 0.5,
                insert_search_radius: 1,
                search_radius: 1,
                fallback_search_radius: None,
            },
            SurfelConfig {
                min_mature_surfel_count: 4,
                max_plane_distance: 0.05,
                enable_line_constraints: true,
                max_line_distance: 0.3,
                line_constraint_weight: 0.5,
                growing_radius: 2.0,
                ..SurfelConfig::default()
            },
        );
        line_map
            .insert(vec![
                point(-1.0, 0.0, 0.0),
                point(-0.5, 0.0, 0.0),
                point(0.5, 0.0, 0.0),
                point(1.0, 0.0, 0.0),
                point(0.0, 0.0, 0.0),
            ])
            .unwrap();
        let mut scratch = line_map.create_query_scratch();
        let line = line_map
            .insertion_consistency_with_scratch(&point(0.25, 0.1, 0.05), &mut scratch)
            .unwrap()
            .unwrap();
        assert_eq!(line.kind, SurfelConstraintKind::Line);
        assert!(line.distance < 0.12);
    }
}
