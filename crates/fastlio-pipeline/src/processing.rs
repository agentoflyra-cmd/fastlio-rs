use anyhow::Result;
use fastlio_imu::ImuIntegrator;
use fastlio_pointcloud::preprocess::preprocess;
use fastlio_types::{LidarFrame, LidarImuExtrinsic, MeasureGroup, NavState, PreprocessConfig};

use crate::deskew::{build_motion_segments, deskew};

/// Run the current company-version LiDAR frame preparation flow.
///
/// The input `MeasureGroup` must contain raw LiDAR points whose
/// `offset_time_sec` values are sorted in non-decreasing order. The supplied
/// `navstate_at_first_imu` must represent the nominal IMU state at
/// `measure_group.imu[0].time_stamp_sec`.
///
/// Processing order is intentionally:
///
/// 1. build IMU motion segments,
/// 2. deskew raw LiDAR points into the LiDAR frame at scan end,
/// 3. apply point filtering and voxel downsampling to the deskewed coordinates.
pub fn deskew_then_preprocess(
    measure_group: MeasureGroup,
    navstate_at_first_imu: NavState,
    imu_integrator: &ImuIntegrator,
    extrinsic: &LidarImuExtrinsic,
    preprocess_config: &PreprocessConfig,
) -> Result<LidarFrame> {
    let segments = build_motion_segments(&measure_group, navstate_at_first_imu, imu_integrator)?;
    let mut lidar = measure_group.lidar;

    deskew(&mut lidar, &segments, extrinsic)?;
    preprocess(preprocess_config, lidar)
}

#[cfg(test)]
mod tests {
    use super::*;
    use fastlio_types::{ImuSample, LidarType, PointXYZI, TimedPoint, Vec3};
    use nalgebra::UnitQuaternion;

    fn point(offset_time_sec: f64, x: f32) -> TimedPoint {
        TimedPoint {
            offset_time_sec,
            point: PointXYZI {
                x,
                y: 0.0,
                z: 0.0,
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

    fn navstate_with_velocity(velocity: Vec3<f64>) -> NavState {
        NavState {
            position: Vec3::zeros(),
            orientation: UnitQuaternion::identity(),
            velocity,
            gyro_bias: Vec3::zeros(),
            accel_bias: Vec3::zeros(),
            gravity: Vec3::zeros(),
        }
    }

    fn identity_extrinsic() -> LidarImuExtrinsic {
        LidarImuExtrinsic::new(UnitQuaternion::identity(), Vec3::zeros())
    }

    fn preprocess_config(voxel_size: Option<f32>) -> PreprocessConfig {
        PreprocessConfig {
            lidar_type: LidarType::Avia,
            scan_line: None,
            blind_zone: 0.0,
            voxel_size,
            max_range: None,
        }
    }

    #[test]
    fn pipeline_deskews_before_voxel_downsampling() {
        let measure_group = MeasureGroup {
            imu: vec![imu(10.0), imu(10.2)],
            lidar: LidarFrame::new(10.0, 10.2, vec![point(0.0, 1.0), point(0.2, 1.0)]),
        };
        let navstate = navstate_with_velocity(Vec3::new(1.0, 0.0, 0.0));
        let imu_integrator = ImuIntegrator::init(0.0, 0.0, 0.0, 0.0);
        let extrinsic = identity_extrinsic();
        let config = preprocess_config(Some(1.0));

        let result = deskew_then_preprocess(
            measure_group,
            navstate,
            &imu_integrator,
            &extrinsic,
            &config,
        )
        .unwrap();

        assert_eq!(result.points.len(), 2);
        assert!((result.points[0].point.x - 0.8).abs() < 1.0e-6);
        assert!((result.points[1].point.x - 1.0).abs() < 1.0e-6);
    }

    #[test]
    fn pipeline_rejects_unsorted_offsets_before_preprocess() {
        let measure_group = MeasureGroup {
            imu: vec![imu(10.0), imu(10.2)],
            lidar: LidarFrame::new(10.0, 10.2, vec![point(0.2, 1.0), point(0.0, 1.0)]),
        };
        let navstate = navstate_with_velocity(Vec3::zeros());
        let imu_integrator = ImuIntegrator::init(0.0, 0.0, 0.0, 0.0);
        let extrinsic = identity_extrinsic();
        let config = preprocess_config(None);

        let err = deskew_then_preprocess(
            measure_group,
            navstate,
            &imu_integrator,
            &extrinsic,
            &config,
        )
        .unwrap_err();

        assert!(err.to_string().contains("offsets must be sorted"));
    }
}
