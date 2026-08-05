use anyhow::{Context, Result};
use fastlio_estimator::iesekf::{
    ErrorStateCovariance, Iesekf, IesekfConfig, IesekfPointToPlaneFactor, IesekfUpdateReport,
};
use fastlio_imu::ImuIntegrator;
use fastlio_map::{LocalMap, PointToPlaneConfig};
use fastlio_pointcloud::preprocess::preprocess;
use fastlio_types::{
    ImuSample, LidarFrame, LidarImuExtrinsic, MeasureGroup, PointXYZI, PreprocessConfig, Vec3,
};

use crate::deskew::{build_motion_segments, deskew};

/// Main FAST-LIO front-end pipeline status for the company-version path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PipelineMode {
    BootstrapMap,
    Tracking,
}

pub struct PipelineConfig {
    pub preprocess: PreprocessConfig,
    pub point_to_plane: PointToPlaneConfig,
    pub iesekf: IesekfConfig,
    pub min_effective_observations: usize,
    pub map_crop_radius: Option<f64>,
    pub insert_scan_points: bool,
    pub max_factor_points: Option<usize>,
    pub max_map_insert_points: Option<usize>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PipelineFrameReport {
    pub mode: PipelineMode,
    pub preprocessed_points: usize,
    pub effective_observations: usize,
    pub map_points_before: usize,
    pub map_points_after: usize,
    pub update: Option<IesekfUpdateReport>,
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
    pub local_map: LocalMap,
    pub imu_integrator: ImuIntegrator,
    pub extrinsic: LidarImuExtrinsic,
    pub config: PipelineConfig,
    last_imu_for_deskew: Option<ImuSample>,
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
            local_map,
            imu_integrator,
            extrinsic,
            config,
            last_imu_for_deskew: None,
        }
    }

    pub fn process_measurement_group(
        &mut self,
        mut measure_group: MeasureGroup,
    ) -> Result<PipelineFrameReport> {
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

        let map_points_before = self.local_map.len();
        let predicted_start_state = self.filter.state.clone();
        let segments = build_motion_segments(
            &measure_group,
            predicted_start_state.clone(),
            &self.imu_integrator,
        )
        .context("failed to build deskew motion segments")?;

        let (predicted_state, predicted_covariance) = predict_through_imu(
            &self.imu_integrator,
            predicted_start_state,
            self.filter.covariance,
            &measure_group,
        )
        .context("failed to predict state and covariance through IMU samples")?;
        self.filter
            .set_predicted(predicted_state, predicted_covariance)
            .map_err(|err| anyhow::anyhow!("failed to set predicted IESEKF state: {err:?}"))?;

        deskew(&mut measure_group.lidar, &segments, &self.extrinsic)
            .context("failed to deskew lidar frame")?;
        let preprocessed = preprocess(&self.config.preprocess, measure_group.lidar)
            .context("failed to preprocess deskewed lidar frame")?;
        let preprocessed_points = preprocessed.points.len();

        let mut factors = Vec::new();
        if !self.local_map.is_empty() {
            factors = build_iesekf_factors(
                &self.filter,
                &self.local_map,
                &self.extrinsic,
                &preprocessed,
                self.config.point_to_plane,
                self.config.max_factor_points,
            );
        }
        let effective_observations = factors.len();

        let (mode, update) = if effective_observations >= self.config.min_effective_observations {
            let update = self
                .filter
                .update_point_to_plane_iterated(&factors, self.config.iesekf)
                .map_err(|err| anyhow::anyhow!("IESEKF point-to-plane update failed: {err:?}"))?;
            (PipelineMode::Tracking, Some(update))
        } else {
            (PipelineMode::BootstrapMap, None)
        };

        if self.config.insert_scan_points {
            let map_points = transform_lidar_frame_to_world(
                &preprocessed,
                &self.extrinsic,
                self.filter.state.orientation,
                self.filter.state.position,
                self.config.max_map_insert_points,
            );
            self.local_map.insert_points(map_points);
        }

        if let Some(radius) = self.config.map_crop_radius {
            self.local_map
                .crop_by_center_radius(self.filter.state.position, radius);
        }

        Ok(PipelineFrameReport {
            mode,
            preprocessed_points,
            effective_observations,
            map_points_before,
            map_points_after: self.local_map.len(),
            update,
        })
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

fn build_iesekf_factors(
    filter: &Iesekf,
    local_map: &LocalMap,
    extrinsic: &LidarImuExtrinsic,
    preprocessed: &LidarFrame,
    config: PointToPlaneConfig,
    max_factor_points: Option<usize>,
) -> Vec<IesekfPointToPlaneFactor> {
    let total_points = preprocessed.points.len();
    preprocessed
        .points
        .iter()
        .enumerate()
        .filter(move |(point_index, _)| {
            should_sample_point(*point_index, total_points, max_factor_points)
        })
        .filter_map(|timed_point| {
            let timed_point = timed_point.1;
            let point_i = extrinsic.transform_point(&timed_point.point.to_vec3_f64());
            let point_w = filter.state.orientation * point_i + filter.state.position;
            let observation = local_map.point_to_plane_observation(point_w, config).ok()?;

            Some(IesekfPointToPlaneFactor {
                point_i,
                plane_w: observation.plane,
                weight: observation.weight,
            })
        })
        .collect()
}

fn transform_lidar_frame_to_world(
    lidar: &LidarFrame,
    extrinsic: &LidarImuExtrinsic,
    rotation_wi: nalgebra::UnitQuaternion<f64>,
    position_wi: Vec3<f64>,
    max_map_insert_points: Option<usize>,
) -> Vec<PointXYZI> {
    let total_points = lidar.points.len();
    lidar
        .points
        .iter()
        .enumerate()
        .filter(move |(point_index, _)| {
            should_sample_point(*point_index, total_points, max_map_insert_points)
        })
        .map(|timed_point| {
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
        .collect()
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
        ImuSample {
            time_stamp_sec,
            gyro: Vec3::zeros(),
            accel: Vec3::zeros(),
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
            map_crop_radius: None,
            insert_scan_points: true,
            max_factor_points: None,
            max_map_insert_points: None,
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
}
