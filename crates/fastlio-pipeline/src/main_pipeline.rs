use anyhow::{Context, Result};
use fastlio_estimator::iesekf::{
    ErrorStateCovariance, Iesekf, IesekfConfig, IesekfPointToPlaneFactor, IesekfUpdateReport,
};
use fastlio_imu::ImuIntegrator;
use fastlio_map::{
    LocalMap, PointToPlaneConfig, PointToPlaneError,
    surfel::{SurfelMap, SurfelMapQueryStats, SurfelOutputPoint},
};
use fastlio_pointcloud::preprocess::preprocess;
use fastlio_types::{
    ImuSample, LidarFrame, LidarImuExtrinsic, MeasureGroup, PointXYZI, PreprocessConfig, Vec3,
};
use std::time::{Duration, Instant};

use crate::deskew::{build_motion_segments, deskew};

/// Main FAST-LIO front-end pipeline status for the company-version path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PipelineMode {
    Initializing,
    BootstrapMap,
    Tracking,
    TrackingLost,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateRejectionReason {
    FinalCostIncreased,
    TranslationStepTooLarge,
    RotationStepTooLarge,
    UpdateCorrectionTooLarge,
    UpdateRotationCorrectionTooLarge,
}

impl GateRejectionReason {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::FinalCostIncreased => "final_cost_increased",
            Self::TranslationStepTooLarge => "translation_step_too_large",
            Self::RotationStepTooLarge => "rotation_step_too_large",
            Self::UpdateCorrectionTooLarge => "update_correction_too_large",
            Self::UpdateRotationCorrectionTooLarge => "update_rotation_correction_too_large",
        }
    }
}

/// Rejected IESEKF update with the numeric evidence that triggered the gate.
#[derive(Debug, Clone, Copy)]
pub struct GateRejection {
    pub reason: GateRejectionReason,
    pub initial_cost: f64,
    pub final_cost: f64,
    pub translation_step: f64,
    pub max_translation_step: Option<f64>,
    pub rotation_step_rad: f64,
    pub max_rotation_step_rad: Option<f64>,
    pub correction: f64,
    pub max_correction: Option<f64>,
    pub rotation_correction_rad: f64,
    pub max_rotation_correction_rad: Option<f64>,
}

pub struct PipelineConfig {
    pub preprocess: PreprocessConfig,
    pub point_to_plane: PointToPlaneConfig,
    pub iesekf: IesekfConfig,
    pub min_effective_observations: usize,
    pub max_tracking_translation_step: Option<f64>,
    pub max_tracking_rotation_step_rad: Option<f64>,
    pub max_update_translation_correction: Option<f64>,
    pub max_update_rotation_correction_rad: Option<f64>,
    pub map_crop_radius: Option<f64>,
    pub insert_scan_points: bool,
    pub max_factor_points: Option<usize>,
    pub max_map_insert_points: Option<usize>,
    pub map_insert_min_distance: Option<f64>,
    pub initialization_groups: usize,
    /// Extra scan-to-map reassociation passes: after each IESEKF update the
    /// association is rebuilt at the corrected pose and updated again. The
    /// first association at the predicted pose misses points when pose error
    /// exceeds the match tolerance (e.g. 0.3-0.5 m on loop-back); reassociation
    /// at the corrected pose recovers them with a clean (strict) tolerance
    /// instead of globally relaxing it.
    pub max_reassociation_passes: usize,
}

pub enum PipelineMap {
    Kiddo(LocalMap),
    Surfel(SurfelMap),
}

impl PipelineMap {
    pub fn len(&self) -> usize {
        match self {
            Self::Kiddo(map) => map.len(),
            Self::Surfel(map) => map.len(),
        }
    }

    pub fn is_empty(&self) -> bool {
        match self {
            Self::Kiddo(map) => map.is_empty(),
            Self::Surfel(map) => map.is_empty(),
        }
    }

    pub fn output_points(&self) -> Vec<PointXYZI> {
        self.points()
    }

    pub fn points(&self) -> Vec<PointXYZI> {
        match self {
            Self::Kiddo(map) => map
                .points()
                .iter()
                .map(|point| PointXYZI {
                    x: point.x,
                    y: point.y,
                    z: point.z,
                    intensity: point.intensity,
                })
                .collect(),
            Self::Surfel(map) => map.output_points(),
        }
    }

    /// Surfel centroids with normals and geometry classes for the three.js
    /// viewer (binary PCD with `normal_*` + `class_id`). `None` for kiddo.
    pub fn output_surfel_points(&self) -> Option<Vec<SurfelOutputPoint>> {
        match self {
            Self::Kiddo(_) => None,
            Self::Surfel(map) => Some(map.output_surfel_points()),
        }
    }

    pub fn insert_points<I>(&mut self, points: I, min_distance_m: Option<f64>) -> Result<()>
    where
        I: IntoIterator<Item = PointXYZI>,
    {
        match self {
            Self::Kiddo(map) => {
                if let Some(min_distance) = min_distance_m {
                    map.insert_points_with_min_distance(points, min_distance);
                } else {
                    map.insert_points(points);
                }
                Ok(())
            }
            Self::Surfel(map) => map.insert(points),
        }
    }

    pub fn crop_by_center_radius(&mut self, center_w: Vec3<f64>, radius_m: f64) {
        match self {
            Self::Kiddo(map) => map.crop_by_center_radius(center_w, radius_m),
            Self::Surfel(_) => {
                let _ = (center_w, radius_m);
            }
        }
    }

    pub fn backend_name(&self) -> &'static str {
        match self {
            Self::Kiddo(_) => "kiddo",
            Self::Surfel(_) => "surfel",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PipelineFrameReport {
    pub mode: PipelineMode,
    pub preprocessed_points: usize,
    pub effective_observations: usize,
    pub association_stats: PipelineAssociationStats,
    pub map_points_before: usize,
    pub map_points_after: usize,
    pub update: Option<IesekfUpdateReport>,
    pub timings: PipelineStageTimings,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PipelineAssociationStats {
    pub sampled_points: usize,
    pub accepted_observations: usize,
    pub non_finite_scan_points: usize,
    pub invalid_configs: usize,
    pub no_planar_surfel: usize,
    pub neighbour_too_far: usize,
    pub plane_fit_errors: usize,
    pub residual_too_large: usize,
    pub surfel_primary_raw_candidates: usize,
    pub surfel_primary_unique_candidates: usize,
    pub surfel_fallback_raw_candidates: usize,
    pub surfel_fallback_unique_candidates: usize,
    pub surfel_fallback_queries: usize,
    pub surfel_fallback_hits: usize,
    pub surfel_planar_candidates: usize,
    pub surfel_growing_candidates: usize,
    pub surfel_accepted_growing_weak: usize,
    pub normal_sum_x: f64,
    pub normal_sum_y: f64,
    pub normal_sum_z: f64,
}

impl PipelineAssociationStats {
    fn record_error(&mut self, err: &PointToPlaneError) {
        match err {
            PointToPlaneError::NonFiniteScanPoint => self.non_finite_scan_points += 1,
            PointToPlaneError::InvalidConfig => self.invalid_configs += 1,
            PointToPlaneError::NoPlanarSurfel => self.no_planar_surfel += 1,
            PointToPlaneError::NeighbourTooFar { .. } => self.neighbour_too_far += 1,
            PointToPlaneError::PlaneFit(_) => self.plane_fit_errors += 1,
            PointToPlaneError::ResidualTooLarge { .. } => self.residual_too_large += 1,
        }
    }

    fn record_surfel_query(&mut self, stats: SurfelMapQueryStats) {
        self.surfel_primary_raw_candidates += stats.primary_raw_candidates;
        self.surfel_primary_unique_candidates += stats.primary_unique_candidates;
        self.surfel_fallback_raw_candidates += stats.fallback_raw_candidates;
        self.surfel_fallback_unique_candidates += stats.fallback_unique_candidates;
        self.surfel_fallback_queries += stats.fallback_queries;
        self.surfel_fallback_hits += stats.fallback_hits;
        self.surfel_planar_candidates += stats.planar_candidates;
        self.surfel_growing_candidates += stats.growing_candidates;
        self.surfel_accepted_growing_weak += stats.accepted_growing_weak;
    }

    fn record_normal(&mut self, normal_w: Vec3<f64>) {
        self.normal_sum_x += normal_w.x;
        self.normal_sum_y += normal_w.y;
        self.normal_sum_z += normal_w.z;
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PipelineStageTimings {
    pub total: Duration,
    pub imu_boundary: Duration,
    pub initialization: Duration,
    pub motion_segments: Duration,
    pub predict: Duration,
    pub deskew: Duration,
    pub preprocess: Duration,
    pub association: Duration,
    pub association_nearest: Duration,
    pub association_plane_fit: Duration,
    pub association_residual: Duration,
    pub association_factor_build: Duration,
    pub update: Duration,
    pub map_insert: Duration,
    pub map_crop: Duration,
}

/// Stateful front-end pipeline.
///
/// Current processing order:
///
/// 1. predict nominal state and covariance through the synchronized IMU samples;
/// 2. deskew LiDAR points to the scan end frame;
/// 3. preprocess/filter/voxel the deskewed scan;
/// 4. associate scan points against the local map and build point-to-plane IESEKF factors;
/// 5. update the IESEKF if enough factors survive;
/// 6. insert the accepted scan into the local map and optionally crop around current position.
///
/// Scan points are interpreted in `LiDAR(end)` after deskew. They are converted
/// to IMU frame with `T_IL` (`LidarImuExtrinsic`) before IESEKF update and map
/// insertion. Map points are stored in the world/map frame `W`.
pub struct FastLioPipeline {
    pub filter: Iesekf,
    pub local_map: PipelineMap,
    pub imu_integrator: ImuIntegrator,
    pub extrinsic: LidarImuExtrinsic,
    pub config: PipelineConfig,
    last_imu_for_deskew: Option<ImuSample>,
    initializer: ImuInitializer,
    initialized: bool,
    tracking_established: bool,
    tracking_lost: bool,
}

impl FastLioPipeline {
    pub fn new(
        filter: Iesekf,
        local_map: LocalMap,
        imu_integrator: ImuIntegrator,
        extrinsic: LidarImuExtrinsic,
        config: PipelineConfig,
    ) -> Self {
        Self {
            filter,
            local_map: PipelineMap::Kiddo(local_map),
            imu_integrator,
            extrinsic,
            config,
            last_imu_for_deskew: None,
            initializer: ImuInitializer::default(),
            initialized: false,
            tracking_established: false,
            tracking_lost: false,
        }
    }

    pub fn new_with_surfel_map(
        filter: Iesekf,
        surfel_map: SurfelMap,
        imu_integrator: ImuIntegrator,
        extrinsic: LidarImuExtrinsic,
        config: PipelineConfig,
    ) -> Self {
        Self {
            filter,
            local_map: PipelineMap::Surfel(surfel_map),
            imu_integrator,
            extrinsic,
            config,
            last_imu_for_deskew: None,
            initializer: ImuInitializer::default(),
            initialized: false,
            tracking_established: false,
            tracking_lost: false,
        }
    }

    pub fn process_measurement_group(
        &mut self,
        mut measure_group: MeasureGroup,
    ) -> Result<PipelineFrameReport> {
        let total_start = Instant::now();
        let imu_boundary_start = Instant::now();
        let imu_for_initialization = measure_group.imu.clone();
        let current_tail_imu = measure_group.imu.last().cloned();
        if let Some(last_imu) = self.last_imu_for_deskew.clone()
            && measure_group
                .imu
                .first()
                .is_none_or(|first| first.time_stamp_sec > last_imu.time_stamp_sec)
        {
            measure_group.imu.insert(0, last_imu);
        }
        if let Some(current_tail_imu) = current_tail_imu {
            self.last_imu_for_deskew = Some(current_tail_imu);
        }
        if let Some(first_imu) = measure_group.imu.first().cloned()
            && first_imu.time_stamp_sec > measure_group.lidar.base_timestamp_sec
        {
            let mut begin_imu = first_imu;
            begin_imu.time_stamp_sec = measure_group.lidar.base_timestamp_sec;
            measure_group.imu.insert(0, begin_imu);
        }
        if let Some(last_imu) = measure_group.imu.last().cloned()
            && last_imu.time_stamp_sec < measure_group.lidar.end_timestamp_sec()
        {
            let mut end_imu = last_imu;
            end_imu.time_stamp_sec = measure_group.lidar.end_timestamp_sec();
            measure_group.imu.push(end_imu);
        }
        let imu_boundary_duration = imu_boundary_start.elapsed();

        if !self.initialized && self.config.initialization_groups > 0 {
            let initialization_start = Instant::now();
            self.initializer.accumulate(&imu_for_initialization);
            let mut timings = PipelineStageTimings {
                imu_boundary: imu_boundary_duration,
                initialization: initialization_start.elapsed(),
                ..PipelineStageTimings::default()
            };
            if self.initializer.group_count < self.config.initialization_groups {
                timings.total = total_start.elapsed();
                return Ok(PipelineFrameReport {
                    mode: PipelineMode::Initializing,
                    preprocessed_points: 0,
                    effective_observations: 0,
                    association_stats: PipelineAssociationStats::default(),
                    map_points_before: self.local_map.len(),
                    map_points_after: self.local_map.len(),
                    update: None,
                    timings,
                });
            }

            let initialized = self
                .initializer
                .finish()
                .context("failed to initialize IMU state")?;
            eprintln!(
                "imu_init gravity_aligned={} acc_cov_max={:.6} mean_acc_norm={:.3}",
                initialized.gravity_aligned,
                self.initializer
                    .cov_acc
                    .iter()
                    .copied()
                    .fold(f64::NEG_INFINITY, f64::max),
                self.initializer.mean_acc.norm()
            );
            self.filter.state.gravity = initialized.gravity;
            self.filter.state.orientation = initialized.orientation;
            self.filter.state.gyro_bias = initialized.gyro_bias;
            self.imu_integrator
                .set_accel_scale(initialized.accel_scale)
                .context("failed to set IMU acceleration scale")?;
            self.initialized = true;

            timings.initialization = initialization_start.elapsed();
            timings.total = total_start.elapsed();
            return Ok(PipelineFrameReport {
                mode: PipelineMode::Initializing,
                preprocessed_points: 0,
                effective_observations: 0,
                association_stats: PipelineAssociationStats::default(),
                map_points_before: self.local_map.len(),
                map_points_after: self.local_map.len(),
                update: None,
                timings,
            });
        }

        let map_points_before = self.local_map.len();
        let previous_state = self.filter.state.clone();
        let previous_covariance = self.filter.covariance;
        let predicted_start_state = previous_state.clone();
        let motion_segments_start = Instant::now();
        let segments = build_motion_segments(
            &measure_group,
            predicted_start_state.clone(),
            &self.imu_integrator,
        )
        .context("failed to build deskew motion segments")?;
        let motion_segments_duration = motion_segments_start.elapsed();

        let predict_start = Instant::now();
        let (predicted_state, predicted_covariance) = predict_through_imu(
            &self.imu_integrator,
            predicted_start_state,
            self.filter.covariance,
            &measure_group,
        )
        .context("failed to predict state and covariance through IMU samples")?;
        let predicted_state_for_gate = predicted_state.clone();
        self.filter
            .set_predicted(predicted_state, predicted_covariance)
            .map_err(|err| anyhow::anyhow!("failed to set predicted IESEKF state: {err:?}"))?;
        let predict_duration = predict_start.elapsed();

        let deskew_start = Instant::now();
        deskew(&mut measure_group.lidar, &segments, &self.extrinsic)
            .context("failed to deskew lidar frame")?;
        let deskew_duration = deskew_start.elapsed();

        let preprocess_start = Instant::now();
        let frame_end_ts = measure_group.lidar.end_timestamp_sec();
        let preprocessed = preprocess(&self.config.preprocess, measure_group.lidar)
            .context("failed to preprocess deskewed lidar frame")?;
        let preprocessed_points = preprocessed.points.len();
        let preprocess_duration = preprocess_start.elapsed();

        let association_start = Instant::now();
        let mut first_build: Option<IesekfFactorBuild> = None;
        let mut factors = Vec::new();
        let mut association_breakdown = AssociationTimings::default();
        let mut association_stats = PipelineAssociationStats::default();
        if !self.local_map.is_empty() {
            let factor_build = build_iesekf_factors(
                &self.filter,
                &self.local_map,
                &self.extrinsic,
                &preprocessed,
                self.config.point_to_plane,
                self.config.max_factor_points,
            );
            factors = factor_build.factors.clone();
            association_breakdown = factor_build.timings;
            association_stats = factor_build.stats;
            first_build = Some(factor_build);
        }
        let effective_observations = factors.len();
        let association_duration = association_start.elapsed();

        let update_start = Instant::now();
        let (mode, update) = if effective_observations >= self.config.min_effective_observations {
            // Attempt the update whenever observations are sufficient, including
            // while latched TrackingLost: a healthy update passes the gate and
            // recovers tracking; a rejected one keeps the latch.
            let first_update = self
                .filter
                .update_point_to_plane_iterated(&factors, self.config.iesekf)
                .map_err(|err| anyhow::anyhow!("IESEKF point-to-plane update failed: {err:?}"))?;
            let mut final_update = first_update;
            let mut final_observation_count = effective_observations;
            // Reassociation passes: re-query only the points missed at the
            // predicted pose, now at the corrected pose, then update with the
            // combined factor set. Already-matched factors are unchanged. A
            // failed/insufficient later pass keeps the previous result.
            for _ in 0..self.config.max_reassociation_passes {
                let Some(first_build) = &first_build else {
                    break;
                };
                if first_build.missed_sample_indices.is_empty() || self.local_map.is_empty() {
                    break;
                }
                let re_build = build_missing_iesekf_factors(
                    &self.filter,
                    &self.local_map,
                    &self.extrinsic,
                    &preprocessed,
                    self.config.point_to_plane,
                    &first_build.missed_sample_indices,
                );
                let mut combined = first_build.factors.clone();
                combined.extend(re_build.factors);
                if combined.len() < self.config.min_effective_observations {
                    break;
                }
                association_breakdown += re_build.timings;
                association_stats.accepted_observations = association_stats
                    .accepted_observations
                    .saturating_add(re_build.stats.accepted_observations);
                final_observation_count = combined.len();
                // The first pass already converged from the predicted pose; the
                // reassociation factor set differs by only the recovered
                // points, so a short second pass suffices. Capping iterations
                // keeps the per-frame update cost near the single-pass level.
                let reassociation_config = IesekfConfig {
                    max_iterations: 2,
                    ..self.config.iesekf
                };
                let Ok(re_update) = self
                    .filter
                    .update_point_to_plane_iterated(&combined, reassociation_config)
                else {
                    break;
                };
                final_update = re_update;
            }
            if self.tracking_established || self.tracking_lost {
                if let Some(rejection) = update_passes_acceptance_gate(
                    &previous_state,
                    &predicted_state_for_gate,
                    &self.filter.state,
                    &final_update,
                    &self.config,
                ) {
                    eprintln!(
                        "gate_reject t={:.6} obs={} reason={} cost={:.6}->{:.6} step_t={:.4}/{:?} step_r={:.5}/{:?} corr_t={:.4}/{:?} corr_r={:.5}/{:?}",
                        frame_end_ts,
                        final_observation_count,
                        rejection.reason.as_str(),
                        rejection.initial_cost,
                        rejection.final_cost,
                        rejection.translation_step,
                        rejection.max_translation_step,
                        rejection.rotation_step_rad,
                        rejection.max_rotation_step_rad,
                        rejection.correction,
                        rejection.max_correction,
                        rejection.rotation_correction_rad,
                        rejection.max_rotation_correction_rad,
                    );
                    self.filter
                        .set_predicted(previous_state, previous_covariance)
                        .map_err(|err| {
                            anyhow::anyhow!(
                                "failed to restore state after rejected update: {err:?}"
                            )
                        })?;
                    self.tracking_lost = true;
                    (PipelineMode::TrackingLost, None)
                } else {
                    self.tracking_established = true;
                    self.tracking_lost = false;
                    (PipelineMode::Tracking, Some(final_update))
                }
            } else {
                self.tracking_established = true;
                (PipelineMode::Tracking, Some(final_update))
            }
        } else if self.tracking_established || self.tracking_lost {
            self.filter
                .set_predicted(previous_state, previous_covariance)
                .map_err(|err| anyhow::anyhow!("failed to restore last tracked state: {err:?}"))?;
            self.tracking_lost = true;
            (PipelineMode::TrackingLost, None)
        } else {
            (PipelineMode::BootstrapMap, None)
        };
        let update_duration = update_start.elapsed();

        let map_insert_start = Instant::now();
        if self.config.insert_scan_points && mode != PipelineMode::TrackingLost {
            let map_points = transform_lidar_frame_to_world_points(
                &preprocessed,
                &self.extrinsic,
                self.filter.state.orientation,
                self.filter.state.position,
                self.config.max_map_insert_points,
            );
            self.local_map
                .insert_points(map_points, self.config.map_insert_min_distance)
                .context("failed to insert scan points into map")?;
        }
        let map_insert_duration = map_insert_start.elapsed();

        let map_crop_start = Instant::now();
        if let Some(radius) = self.config.map_crop_radius {
            self.local_map
                .crop_by_center_radius(self.filter.state.position, radius);
        }
        let map_crop_duration = map_crop_start.elapsed();

        Ok(PipelineFrameReport {
            mode,
            preprocessed_points,
            effective_observations,
            association_stats,
            map_points_before,
            map_points_after: self.local_map.len(),
            update,
            timings: PipelineStageTimings {
                total: total_start.elapsed(),
                imu_boundary: imu_boundary_duration,
                initialization: Duration::ZERO,
                motion_segments: motion_segments_duration,
                predict: predict_duration,
                deskew: deskew_duration,
                preprocess: preprocess_duration,
                association: association_duration,
                association_nearest: association_breakdown.nearest,
                association_plane_fit: association_breakdown.plane_fit,
                association_residual: association_breakdown.residual,
                association_factor_build: association_breakdown.factor_build,
                update: update_duration,
                map_insert: map_insert_duration,
                map_crop: map_crop_duration,
            },
        })
    }
}

#[derive(Default)]
struct ImuInitializer {
    group_count: usize,
    sample_count: usize,
    mean_acc: Vec3<f64>,
    mean_gyr: Vec3<f64>,
    /// Incremental per-axis variance of the measured acceleration
    /// (`M2/N`), used to gate gravity alignment on platform staticness.
    cov_acc: Vec3<f64>,
}

struct ImuInitialization {
    gravity: Vec3<f64>,
    /// `R_WI` aligning the world +Z (anti-gravity) axis with the mean measured
    /// acceleration direction in the IMU frame. Without this the initial
    /// orientation is identity and gravity is misaligned with the body, so the
    /// prediction accumulates a residual vertical acceleration and `z` drifts
    /// quadratically once LiDAR constraints weaken.
    orientation: nalgebra::UnitQuaternion<f64>,
    /// True when gravity alignment ran (static platform). When false the mean
    /// acceleration is contaminated by linear motion and the identity
    /// orientation / default gravity are kept, exactly as FAST-LIO skips
    /// alignment under the same condition.
    gravity_aligned: bool,
    gyro_bias: Vec3<f64>,
    accel_scale: f64,
}

impl ImuInitializer {
    fn accumulate(&mut self, imu_samples: &[ImuSample]) {
        self.group_count += 1;
        for imu in imu_samples {
            self.sample_count += 1;
            let n = self.sample_count as f64;
            let delta = imu.accel - self.mean_acc;
            self.mean_acc += delta / n;
            let delta2 = imu.accel - self.mean_acc;
            // FAST-LIO's incremental population variance: M2 / N.
            self.cov_acc = self.cov_acc * ((n - 1.0) / n)
                + delta2.component_mul(&delta2) * ((n - 1.0) / (n * n));
            self.mean_gyr += (imu.gyro - self.mean_gyr) / n;
        }
    }

    fn finish(&self) -> Result<ImuInitialization> {
        if self.sample_count == 0 {
            anyhow::bail!("cannot initialize IMU without samples");
        }
        let acc_norm = self.mean_acc.norm();
        if !acc_norm.is_finite() || acc_norm <= 1.0e-6 {
            anyhow::bail!("invalid mean acceleration norm for IMU initialization: {acc_norm}");
        }
        let acc_unit = self.mean_acc / acc_norm;

        // Static gate from the reference FAST-LIO: when the platform moves, the
        // mean acceleration includes linear acceleration, so aligning gravity
        // to it would corrupt roll/pitch. Only align when the acceleration
        // variance per axis stays below the static threshold.
        let acc_cov_max = self
            .cov_acc
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        if acc_cov_max <= 0.02 {
            // Align IMU measured up (mean_acc) to world +Z, then strip the
            // hidden yaw of FromTwoVectors so only roll/pitch change and the
            // map-frame heading (identity on first init) is preserved.
            let align_rot =
                nalgebra::Rotation3::rotation_between(&acc_unit, &Vec3::new(0.0, 0.0, 1.0))
                    .expect("mean_acc is non-zero by construction");
            let align_yaw = align_rot[(1, 0)].atan2(align_rot[(0, 0)]);
            let yaw_rot =
                nalgebra::Rotation3::from_axis_angle(&nalgebra::Vector3::z_axis(), -align_yaw);
            let orientation =
                nalgebra::UnitQuaternion::from_rotation_matrix(&(yaw_rot * align_rot));
            Ok(ImuInitialization {
                // After alignment R_WI * acc_unit = +Z, so gravity is -Z * g.
                gravity: Vec3::new(0.0, 0.0, -9.81),
                orientation,
                gravity_aligned: true,
                gyro_bias: self.mean_gyr,
                accel_scale: 9.81 / acc_norm,
            })
        } else {
            Ok(ImuInitialization {
                gravity: Vec3::new(0.0, 0.0, -9.81),
                orientation: nalgebra::UnitQuaternion::identity(),
                gravity_aligned: false,
                gyro_bias: self.mean_gyr,
                accel_scale: 9.81 / acc_norm,
            })
        }
    }
}

fn predict_through_imu(
    imu_integrator: &ImuIntegrator,
    mut state: fastlio_types::NavState,
    mut covariance: ErrorStateCovariance,
    measure_group: &MeasureGroup,
) -> Result<(fastlio_types::NavState, ErrorStateCovariance)> {
    for imu_pair in measure_group.imu.windows(2) {
        let imu_prev = &imu_pair[0];
        let imu_curr = &imu_pair[1];
        covariance = imu_integrator.propagate_covariance(&state, covariance, imu_prev, imu_curr)?;
        imu_integrator.propagate_nominal_state_mut(&mut state, imu_prev, imu_curr)?;
    }

    Ok((state, covariance))
}

#[derive(Debug, Clone, Default)]
struct IesekfFactorBuild {
    factors: Vec<IesekfPointToPlaneFactor>,
    timings: AssociationTimings,
    stats: PipelineAssociationStats,
    /// Indices (into `preprocessed.points`) of sampled points that produced no
    /// factor in this pass. The reassociation pass re-queries only these at
    /// the corrected pose instead of re-querying every sampled point.
    missed_sample_indices: Vec<usize>,
}

#[derive(Debug, Clone, Copy, Default)]
struct AssociationTimings {
    nearest: Duration,
    plane_fit: Duration,
    residual: Duration,
    factor_build: Duration,
}

impl std::ops::AddAssign for AssociationTimings {
    fn add_assign(&mut self, rhs: Self) {
        self.nearest += rhs.nearest;
        self.plane_fit += rhs.plane_fit;
        self.residual += rhs.residual;
        self.factor_build += rhs.factor_build;
    }
}

fn build_iesekf_factors(
    filter: &Iesekf,
    local_map: &PipelineMap,
    extrinsic: &LidarImuExtrinsic,
    preprocessed: &LidarFrame,
    config: PointToPlaneConfig,
    max_factor_points: Option<usize>,
) -> IesekfFactorBuild {
    let total_points = preprocessed.points.len();
    let sampled = (0..total_points)
        .filter(|point_index| should_sample_point(*point_index, total_points, max_factor_points))
        .collect::<Vec<_>>();
    build_factors_for_indices(filter, local_map, extrinsic, preprocessed, config, sampled)
}

/// Reassociation pass: query only the points that missed in the first pass.
/// Already-matched factors are unchanged (same scan point, same map plane), so
/// only missed points need a fresh query at the corrected pose.
fn build_missing_iesekf_factors(
    filter: &Iesekf,
    local_map: &PipelineMap,
    extrinsic: &LidarImuExtrinsic,
    preprocessed: &LidarFrame,
    config: PointToPlaneConfig,
    missed_sample_indices: &[usize],
) -> IesekfFactorBuild {
    build_factors_for_indices(
        filter,
        local_map,
        extrinsic,
        preprocessed,
        config,
        missed_sample_indices.to_vec(),
    )
}

fn build_factors_for_indices(
    filter: &Iesekf,
    local_map: &PipelineMap,
    extrinsic: &LidarImuExtrinsic,
    preprocessed: &LidarFrame,
    config: PointToPlaneConfig,
    point_indices: Vec<usize>,
) -> IesekfFactorBuild {
    let mut kiddo_query_scratch = match local_map {
        PipelineMap::Kiddo(map) => Some(map.create_query_scratch()),
        PipelineMap::Surfel(_) => None,
    };
    let mut surfel_query_scratch = match local_map {
        PipelineMap::Kiddo(_) => None,
        PipelineMap::Surfel(map) => Some(map.create_query_scratch()),
    };
    let mut build = IesekfFactorBuild::default();
    for point_index in point_indices {
        let timed_point = &preprocessed.points[point_index];
        build.stats.sampled_points += 1;
        let factor_start = Instant::now();
        let point_i = extrinsic.transform_point(&timed_point.point.to_vec3_f64());
        let point_w = filter.state.orientation * point_i + filter.state.position;
        build.timings.factor_build += factor_start.elapsed();

        let (matched, match_timings) = match (
            local_map,
            &mut kiddo_query_scratch,
            &mut surfel_query_scratch,
        ) {
            (PipelineMap::Kiddo(map), Some(scratch), _) => {
                map.point_to_plane_match_attempt_with_scratch(point_w, config, scratch)
            }
            (PipelineMap::Surfel(map), _, Some(scratch)) => {
                map.point_to_plane_match_attempt_with_scratch(point_w, config, scratch)
            }
            (PipelineMap::Kiddo(_), None, _) => unreachable!("kiddo map requires query scratch"),
            (PipelineMap::Surfel(_), _, None) => unreachable!("surfel map requires query scratch"),
        };
        build.timings.nearest += match_timings.nearest;
        build.timings.plane_fit += match_timings.plane_fit;
        build.timings.residual += match_timings.residual;
        if let Some(scratch) = &surfel_query_scratch {
            build.stats.record_surfel_query(scratch.last_stats());
        }

        let matched = match matched {
            Ok(matched) => matched,
            Err(err) => {
                build.stats.record_error(&err);
                build.missed_sample_indices.push(point_index);
                continue;
            }
        };
        build.stats.accepted_observations += 1;
        build.stats.record_normal(matched.plane.normal_w);

        let factor_start = Instant::now();
        build.factors.push(IesekfPointToPlaneFactor {
            point_i,
            plane_w: matched.plane,
            weight: matched.weight,
        });
        build.timings.factor_build += factor_start.elapsed();
    }
    build
}

fn update_passes_acceptance_gate(
    previous_state: &fastlio_types::NavState,
    predicted_state: &fastlio_types::NavState,
    updated_state: &fastlio_types::NavState,
    update: &IesekfUpdateReport,
    config: &PipelineConfig,
) -> Option<GateRejection> {
    let translation_step = (updated_state.position - previous_state.position).norm();
    let rotation_step_rad = previous_state
        .orientation
        .angle_to(&updated_state.orientation);
    let correction = (updated_state.position - predicted_state.position).norm();
    let rotation_correction_rad = predicted_state
        .orientation
        .angle_to(&updated_state.orientation);
    let max_translation_step = config.max_tracking_translation_step;
    let max_rotation_step_rad = config.max_tracking_rotation_step_rad;
    let max_correction = config.max_update_translation_correction;
    let max_rotation_correction_rad = config.max_update_rotation_correction_rad;

    let reason = if update.final_cost > update.initial_cost {
        Some(GateRejectionReason::FinalCostIncreased)
    } else if let Some(max_step) = max_translation_step
        && translation_step > max_step
    {
        Some(GateRejectionReason::TranslationStepTooLarge)
    } else if let Some(max_step) = max_rotation_step_rad
        && rotation_step_rad > max_step
    {
        Some(GateRejectionReason::RotationStepTooLarge)
    } else if let Some(max_correction) = max_correction
        && correction > max_correction
    {
        Some(GateRejectionReason::UpdateCorrectionTooLarge)
    } else if let Some(max_correction) = max_rotation_correction_rad
        && rotation_correction_rad > max_correction
    {
        Some(GateRejectionReason::UpdateRotationCorrectionTooLarge)
    } else {
        None
    };

    reason.map(|reason| GateRejection {
        reason,
        initial_cost: update.initial_cost,
        final_cost: update.final_cost,
        translation_step,
        max_translation_step,
        rotation_step_rad,
        max_rotation_step_rad,
        correction,
        max_correction,
        rotation_correction_rad,
        max_rotation_correction_rad,
    })
}

fn transform_lidar_frame_to_world_points<'a>(
    lidar: &'a LidarFrame,
    extrinsic: &'a LidarImuExtrinsic,
    rotation_wi: nalgebra::UnitQuaternion<f64>,
    position_wi: Vec3<f64>,
    max_map_insert_points: Option<usize>,
) -> impl Iterator<Item = PointXYZI> + 'a {
    let total_points = lidar.points.len();
    lidar
        .points
        .iter()
        .enumerate()
        .filter(move |(point_index, _)| {
            should_sample_point(*point_index, total_points, max_map_insert_points)
        })
        .map(move |timed_point| {
            let timed_point = timed_point.1;
            let point_i = extrinsic.transform_point(&timed_point.point.to_vec3_f64());
            let point_w = rotation_wi * point_i + position_wi;
            PointXYZI {
                x: point_w.x as f32,
                y: point_w.y as f32,
                z: point_w.z as f32,
                intensity: timed_point.point.intensity,
            }
        })
}

fn should_sample_point(point_index: usize, total_points: usize, max_points: Option<usize>) -> bool {
    let Some(max_points) = max_points else {
        return true;
    };
    if max_points == 0 {
        return false;
    }
    let step = total_points.div_ceil(max_points).max(1);
    point_index.is_multiple_of(step)
}

#[cfg(test)]
mod tests {
    use super::*;
    use fastlio_estimator::iesekf::ErrorStateCovariance;
    use fastlio_types::{ImuSample, LidarType, NavState, PointXYZI, Pose3, TimedPoint};
    use nalgebra::UnitQuaternion;

    fn point(offset_time_sec: f64, x: f32, y: f32, z: f32) -> TimedPoint {
        TimedPoint {
            offset_time_sec,
            point: PointXYZI {
                x,
                y,
                z,
                intensity: 1.0,
            },
            tag: 0,
            line: 0,
        }
    }

    fn imu(time_stamp_sec: f64) -> ImuSample {
        imu_with_motion(time_stamp_sec, Vec3::zeros(), Vec3::zeros())
    }

    fn imu_with_motion(time_stamp_sec: f64, gyro: Vec3<f64>, accel: Vec3<f64>) -> ImuSample {
        ImuSample {
            time_stamp_sec,
            gyro,
            accel,
        }
    }

    fn lidar(points: Vec<TimedPoint>) -> LidarFrame {
        LidarFrame::new(10.0, 10.1, points)
    }

    fn navstate(position: Vec3<f64>) -> NavState {
        NavState {
            position,
            orientation: UnitQuaternion::identity(),
            velocity: Vec3::zeros(),
            gyro_bias: Vec3::zeros(),
            accel_bias: Vec3::zeros(),
            gravity: Vec3::zeros(),
        }
    }

    fn config(min_effective_observations: usize) -> PipelineConfig {
        PipelineConfig {
            preprocess: PreprocessConfig {
                lidar_type: LidarType::Avia,
                scan_line: None,
                blind_zone: 0.0,
                voxel_size: None,
                max_range: None,
            },
            point_to_plane: PointToPlaneConfig {
                nearest_count: 5,
                max_neighbour_distance: 5.0,
                max_absolute_residual: 1.0,
                ..PointToPlaneConfig::default()
            },
            iesekf: IesekfConfig {
                measurement_noise_variance: 1.0e-4,
                ..IesekfConfig::default()
            },
            min_effective_observations,
            max_tracking_translation_step: None,
            max_tracking_rotation_step_rad: None,
            max_update_translation_correction: None,
            max_update_rotation_correction_rad: None,
            map_crop_radius: None,
            insert_scan_points: true,
            max_factor_points: None,
            max_map_insert_points: None,
            map_insert_min_distance: None,
            initialization_groups: 0,
            max_reassociation_passes: 0,
        }
    }

    #[test]
    fn point_sampling_respects_none_zero_and_finite_limits() {
        let all: Vec<_> = (0..5)
            .filter(|idx| should_sample_point(*idx, 5, None))
            .collect();
        let none: Vec<_> = (0..5)
            .filter(|idx| should_sample_point(*idx, 5, Some(0)))
            .collect();
        let limited: Vec<_> = (0..10)
            .filter(|idx| should_sample_point(*idx, 10, Some(3)))
            .collect();

        assert_eq!(all, vec![0, 1, 2, 3, 4]);
        assert!(none.is_empty());
        assert_eq!(limited, vec![0, 4, 8]);
    }

    #[test]
    fn initialization_groups_do_not_insert_map_points() {
        let mut pipeline = init_pipeline(2);
        let group = MeasureGroup {
            imu: vec![
                imu_with_motion(10.0, Vec3::zeros(), Vec3::new(0.0, 0.0, 9.81)),
                imu_with_motion(10.1, Vec3::zeros(), Vec3::new(0.0, 0.0, 9.81)),
            ],
            lidar: lidar(vec![point(0.0, 1.0, 0.0, 0.0)]),
        };

        let report = pipeline.process_measurement_group(group).unwrap();

        assert_eq!(report.mode, PipelineMode::Initializing);
        assert_eq!(report.map_points_after, 0);
        assert_eq!(pipeline.local_map.len(), 0);
    }

    #[test]
    fn initialization_sets_gravity_gyro_bias_and_accel_scale() {
        let mut pipeline = init_pipeline(2);
        let gyro = Vec3::new(0.01, -0.02, 0.03);
        let accel = Vec3::new(0.0, 0.0, 9.7);

        for group_idx in 0..2 {
            let scan_begin = 10.0 + group_idx as f64 * 0.1;
            let group = MeasureGroup {
                imu: vec![
                    imu_with_motion(scan_begin, gyro, accel),
                    imu_with_motion(scan_begin + 0.1, gyro, accel),
                ],
                lidar: LidarFrame::new(
                    scan_begin,
                    scan_begin + 0.1,
                    vec![point(0.0, 1.0, 0.0, 0.0)],
                ),
            };

            let report = pipeline.process_measurement_group(group).unwrap();
            assert_eq!(report.mode, PipelineMode::Initializing);
            assert_eq!(pipeline.local_map.len(), 0);
        }

        assert!((pipeline.filter.state.gravity - Vec3::new(0.0, 0.0, -9.81)).norm() < 1.0e-12);
        assert!((pipeline.filter.state.gyro_bias - gyro).norm() < 1.0e-12);
        assert!((pipeline.imu_integrator.accel_scale - 9.81 / 9.7).abs() < 1.0e-12);
    }

    fn pipeline(initial_position: Vec3<f64>, local_map: LocalMap) -> FastLioPipeline {
        let filter = Iesekf::new(
            navstate(initial_position),
            ErrorStateCovariance::identity() * 0.1,
        )
        .unwrap();
        FastLioPipeline::new(
            filter,
            local_map,
            ImuIntegrator::init(0.0, 0.0, 0.0, 0.0),
            Pose3::new(UnitQuaternion::identity(), Vec3::zeros()),
            config(3),
        )
    }

    fn init_pipeline(initialization_groups: usize) -> FastLioPipeline {
        let filter = Iesekf::new(
            navstate(Vec3::zeros()),
            ErrorStateCovariance::identity() * 0.1,
        )
        .unwrap();
        let mut config = config(3);
        config.initialization_groups = initialization_groups;
        FastLioPipeline::new(
            filter,
            LocalMap::new(),
            ImuIntegrator::init(0.0, 0.0, 0.0, 0.0),
            Pose3::new(UnitQuaternion::identity(), Vec3::zeros()),
            config,
        )
    }

    fn horizontal_map() -> LocalMap {
        LocalMap::from_points(vec![
            map_point(-1.0, -1.0, 0.0),
            map_point(1.0, -1.0, 0.0),
            map_point(-1.0, 1.0, 0.0),
            map_point(1.0, 1.0, 0.0),
            map_point(0.0, 0.0, 0.0),
        ])
    }

    fn map_point(x: f32, y: f32, z: f32) -> PointXYZI {
        PointXYZI {
            x,
            y,
            z,
            intensity: 1.0,
        }
    }

    #[test]
    fn empty_map_bootstraps_without_update() {
        let mut pipeline = pipeline(Vec3::zeros(), LocalMap::new());
        let group = MeasureGroup {
            imu: vec![imu(10.0), imu(10.1)],
            lidar: lidar(vec![point(0.0, 0.0, 0.0, 0.0), point(0.1, 1.0, 0.0, 0.0)]),
        };

        let report = pipeline.process_measurement_group(group).unwrap();

        assert_eq!(report.mode, PipelineMode::BootstrapMap);
        assert_eq!(report.effective_observations, 0);
        assert!(report.update.is_none());
        assert_eq!(pipeline.local_map.len(), 2);
    }

    #[test]
    fn existing_map_builds_observations_and_updates_filter() {
        let mut pipeline = pipeline(Vec3::new(0.0, 0.0, 0.2), horizontal_map());
        let group = MeasureGroup {
            imu: vec![imu(10.0), imu(10.1)],
            lidar: lidar(vec![
                point(0.0, -0.5, -0.5, 0.0),
                point(0.03, 0.5, -0.5, 0.0),
                point(0.06, -0.5, 0.5, 0.0),
                point(0.1, 0.5, 0.5, 0.0),
            ]),
        };

        let report = pipeline.process_measurement_group(group).unwrap();

        assert_eq!(report.mode, PipelineMode::Tracking);
        assert!(report.effective_observations >= 3);
        assert!(report.update.is_some());
        assert!(pipeline.filter.state.position.z.abs() < 1.0e-2);
        assert!(pipeline.local_map.len() > report.map_points_before);
    }

    #[test]
    fn map_crop_runs_after_scan_insertion() {
        let mut pipeline = pipeline(Vec3::zeros(), LocalMap::new());
        pipeline.config.map_crop_radius = Some(0.5);
        let group = MeasureGroup {
            imu: vec![imu(10.0), imu(10.1)],
            lidar: lidar(vec![point(0.0, 0.0, 0.0, 0.0), point(0.1, 2.0, 0.0, 0.0)]),
        };

        let report = pipeline.process_measurement_group(group).unwrap();

        assert_eq!(report.mode, PipelineMode::BootstrapMap);
        assert_eq!(pipeline.local_map.len(), 1);
        assert_eq!(pipeline.local_map.points()[0].x, 0.0);
    }

    #[test]
    fn tracking_lost_restores_last_tracked_state_and_skips_map_insert() {
        let mut pipeline = pipeline(Vec3::new(0.0, 0.0, 0.2), horizontal_map());
        let tracking_group = MeasureGroup {
            imu: vec![imu(10.0), imu(10.1)],
            lidar: lidar(vec![
                point(0.0, -0.5, -0.5, 0.0),
                point(0.03, 0.5, -0.5, 0.0),
                point(0.06, -0.5, 0.5, 0.0),
                point(0.1, 0.5, 0.5, 0.0),
            ]),
        };
        let tracking_report = pipeline.process_measurement_group(tracking_group).unwrap();
        assert_eq!(tracking_report.mode, PipelineMode::Tracking);

        let last_tracked_position = pipeline.filter.state.position;
        let last_tracked_covariance = pipeline.filter.covariance;
        let map_len = pipeline.local_map.len();
        let lost_group = MeasureGroup {
            imu: vec![
                imu_with_motion(10.1, Vec3::zeros(), Vec3::new(10.0, 0.0, 0.0)),
                imu_with_motion(10.2, Vec3::zeros(), Vec3::new(10.0, 0.0, 0.0)),
            ],
            lidar: lidar(vec![point(0.0, 100.0, 100.0, 100.0)]),
        };

        let lost_report = pipeline.process_measurement_group(lost_group).unwrap();

        assert_eq!(lost_report.mode, PipelineMode::TrackingLost);
        assert_eq!(lost_report.effective_observations, 0);
        assert_eq!(pipeline.local_map.len(), map_len);
        assert_eq!(pipeline.filter.state.position, last_tracked_position);
        assert_eq!(pipeline.filter.covariance, last_tracked_covariance);
    }

    #[test]
    fn rejected_iekf_update_restores_last_tracked_state_and_skips_map_insert() {
        let mut pipeline = pipeline(Vec3::new(0.0, 0.0, 0.2), horizontal_map());
        let tracking_group = MeasureGroup {
            imu: vec![imu(10.0), imu(10.1)],
            lidar: lidar(vec![
                point(0.0, -0.5, -0.5, 0.0),
                point(0.03, 0.5, -0.5, 0.0),
                point(0.06, -0.5, 0.5, 0.0),
                point(0.1, 0.5, 0.5, 0.0),
            ]),
        };
        let tracking_report = pipeline.process_measurement_group(tracking_group).unwrap();
        assert_eq!(tracking_report.mode, PipelineMode::Tracking);

        pipeline.config.max_update_translation_correction = Some(1.0e-12);
        let last_tracked_position = pipeline.filter.state.position;
        let last_tracked_covariance = pipeline.filter.covariance;
        let map_len = pipeline.local_map.len();
        let rejected_group = MeasureGroup {
            imu: vec![imu(10.1), imu(10.2)],
            lidar: lidar(vec![
                point(0.0, -0.5, -0.5, 0.0),
                point(0.03, 0.5, -0.5, 0.0),
                point(0.06, -0.5, 0.5, 0.0),
                point(0.1, 0.5, 0.5, 0.0),
            ]),
        };

        let rejected_report = pipeline.process_measurement_group(rejected_group).unwrap();

        assert_eq!(rejected_report.mode, PipelineMode::TrackingLost);
        assert!(rejected_report.effective_observations >= 3);
        assert_eq!(pipeline.local_map.len(), map_len);
        assert_eq!(pipeline.filter.state.position, last_tracked_position);
        assert_eq!(pipeline.filter.covariance, last_tracked_covariance);
    }

    #[test]
    fn tracking_lost_recovers_when_gate_passes_and_map_resumes() {
        let mut pipeline = pipeline(Vec3::new(0.0, 0.0, 0.2), horizontal_map());
        let tracking_group = MeasureGroup {
            imu: vec![imu(10.0), imu(10.1)],
            lidar: lidar(vec![
                point(0.0, -0.5, -0.5, 0.0),
                point(0.03, 0.5, -0.5, 0.0),
                point(0.06, -0.5, 0.5, 0.0),
                point(0.1, 0.5, 0.5, 0.0),
            ]),
        };
        assert_eq!(
            pipeline
                .process_measurement_group(tracking_group)
                .unwrap()
                .mode,
            PipelineMode::Tracking
        );

        pipeline.config.max_update_translation_correction = Some(1.0e-12);
        let rejected_group = MeasureGroup {
            imu: vec![imu(10.1), imu(10.2)],
            lidar: lidar(vec![
                point(0.0, -0.5, -0.5, 0.0),
                point(0.03, 0.5, -0.5, 0.0),
                point(0.06, -0.5, 0.5, 0.0),
                point(0.1, 0.5, 0.5, 0.0),
            ]),
        };
        let map_len_before_reject = pipeline.local_map.len();
        let rejected = pipeline.process_measurement_group(rejected_group).unwrap();
        assert_eq!(rejected.mode, PipelineMode::TrackingLost);
        // A lost frame restores the last tracked state and does not insert.
        assert_eq!(pipeline.local_map.len(), map_len_before_reject);

        pipeline.config.max_update_translation_correction = None;
        let latched_position = pipeline.filter.state.position;
        let good_group_after_loss = MeasureGroup {
            imu: vec![imu(10.2), imu(10.3)],
            lidar: lidar(vec![
                point(0.0, -0.5, -0.5, 0.0),
                point(0.03, 0.5, -0.5, 0.0),
                point(0.06, -0.5, 0.5, 0.0),
                point(0.1, 0.5, 0.5, 0.0),
            ]),
        };

        let report = pipeline
            .process_measurement_group(good_group_after_loss)
            .unwrap();

        // Sufficient observations with a passing gate recover tracking and
        // resume map insertion.
        assert_eq!(report.mode, PipelineMode::Tracking);
        assert!(report.effective_observations >= 3);
        assert!(pipeline.local_map.len() > map_len_before_reject);
        assert_ne!(pipeline.filter.state.position, latched_position);
    }
}
