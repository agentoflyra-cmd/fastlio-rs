use anyhow::{Result, anyhow};
use fastlio_imu::ImuIntegrator;
use fastlio_types::{LidarFrame, LidarImuExtrinsic, MeasureGroup, NavState, Pose3};
use nalgebra::UnitQuaternion;

use crate::trajectory::MotionSegment;

pub fn build_motion_segments(
    measure_group: &MeasureGroup,
    mut navstate: NavState,
    imu_inte: &ImuIntegrator,
) -> Result<Vec<MotionSegment>> {
    let imu_frame = &measure_group.imu;

    if imu_frame.len() < 2 {
        return Ok(Vec::new());
    }

    let mut segments = Vec::with_capacity(imu_frame.len() - 1);

    for imu_pair in imu_frame.windows(2) {
        let imu_prev = &imu_pair[0];
        let imu_curr = &imu_pair[1];

        let begin_time = imu_prev.time_stamp_sec;
        let end_time = imu_curr.time_stamp_sec;
        let dt = end_time - begin_time;

        if dt <= 1e-7 {
            continue;
        }

        // State at t_i.
        let pose = Pose3::new(navstate.orientation, navstate.position);
        let velocity = navstate.velocity;

        // Midpoint IMU measurements.
        let omega_mid = 0.5 * (imu_prev.gyro + imu_curr.gyro) - navstate.gyro_bias;

        let acc_mid = 0.5 * (imu_prev.accel + imu_curr.accel) - navstate.accel_bias;

        // Midpoint orientation.
        let half_delta_rotation = UnitQuaternion::from_scaled_axis(omega_mid * (0.5 * dt));

        let r_mid = navstate.orientation * half_delta_rotation;

        // World-frame acceleration.
        let acceleration_world = r_mid * acc_mid + navstate.gravity;

        segments.push(MotionSegment::new(
            begin_time,
            end_time,
            pose,
            velocity,
            omega_mid,
            acceleration_world,
        ));

        // Advance state:
        // NavState(t_i) -> NavState(t_{i+1})
        imu_inte.propagate_nominal_state_mut(&mut navstate, imu_prev, imu_curr)?;
    }

    Ok(segments)
}

pub fn deskew(
    lidar: &mut LidarFrame,
    segments: &[MotionSegment],
    extrinsic: &LidarImuExtrinsic,
) -> Result<()> {
    if segments.is_empty() {
        return Err(anyhow!("deskew failed: motion segments are empty"));
    }

    let base_time = lidar.base_timestamp_sec;
    let end_time = lidar.end_timestamp_sec();

    let end_segment = segments
        .iter()
        .find(|segment| segment.contains(end_time))
        .ok_or_else(|| {
            anyhow!("deskew failed: no motion segment covers lidar end time {end_time}")
        })?;

    let pose_end = end_segment.propagate_to(end_time);
    let pose_end_inv = pose_end.inverse();
    let extrinsic_inv = extrinsic.inverse();

    let mut segment_idx = 0;

    for point in &mut lidar.points {
        let time = base_time + point.offset_time_sec;

        while segment_idx + 1 < segments.len() && time > segments[segment_idx].end_time {
            segment_idx += 1;
        }

        let segment = &segments[segment_idx];

        if !segment.contains(time) {
            return Err(anyhow!(
                "deskew failed: no motion segment covers point time {time}"
            ));
        }

        // LiDAR(t) -> IMU(t)
        let p_i_t = extrinsic.transform_point(&point.point.to_vec3_f64());

        // IMU(t) -> World
        let pose_at_point = segment.propagate_to(time);
        let p_w = pose_at_point.transform_point(&p_i_t);

        // World -> IMU(end)
        let p_i_end = pose_end_inv.transform_point(&p_w);

        // IMU(end) -> LiDAR(end)
        let p_l_end = extrinsic_inv.transform_point(&p_i_end);

        point.point.x = p_l_end.x as f32;
        point.point.y = p_l_end.y as f32;
        point.point.z = p_l_end.z as f32;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use fastlio_imu::ImuIntegrator;
    use fastlio_types::{ImuSample, NavState, PointXYZI, TimedPoint, Vec3};
    use nalgebra::UnitQuaternion;
    use std::f64::consts as f64c;

    fn pt(offset: f64, x: f32, y: f32, z: f32) -> TimedPoint {
        TimedPoint {
            offset_time_sec: offset,
            point: PointXYZI {
                x,
                y,
                z,
                intensity: 10.0,
            },
            tag: 0x1,
            line: 1,
        }
    }

    fn identity_extrinsic() -> LidarImuExtrinsic {
        Pose3::new(UnitQuaternion::identity(), Vec3::zeros())
    }

    fn identity_pose() -> Pose3 {
        Pose3::new(UnitQuaternion::identity(), Vec3::zeros())
    }

    fn seg(
        begin: f64,
        end: f64,
        pose: Pose3,
        vel: Vec3<f64>,
        ang_vel: Vec3<f64>,
        acc_w: Vec3<f64>,
    ) -> MotionSegment {
        MotionSegment::new(begin, end, pose, vel, ang_vel, acc_w)
    }

    fn lidar_frame(base: f64, end: f64, points: Vec<TimedPoint>) -> LidarFrame {
        LidarFrame::new(base, end, points)
    }

    fn assert_point_eq(p: &PointXYZI, expected: (f32, f32, f32)) {
        assert_relative_eq!(p.x, expected.0, epsilon = 1e-6);
        assert_relative_eq!(p.y, expected.1, epsilon = 1e-6);
        assert_relative_eq!(p.z, expected.2, epsilon = 1e-6);
    }

    // -----------------------------------------------------------------------
    // 静止
    // -----------------------------------------------------------------------

    #[test]
    fn deskew_stationary_leaves_points_unchanged() {
        let segments = vec![seg(
            0.0,
            1.0,
            identity_pose(),
            Vec3::zeros(),
            Vec3::zeros(),
            Vec3::zeros(),
        )];
        let mut frame = lidar_frame(0.0, 1.0, vec![pt(0.5, 1.0, 2.0, 3.0)]);
        deskew(&mut frame, &segments, &identity_extrinsic()).unwrap();
        assert_point_eq(&frame.points[0].point, (1.0, 2.0, 3.0));
    }

    // -----------------------------------------------------------------------
    // 纯平移
    // -----------------------------------------------------------------------

    #[test]
    fn deskew_pure_translation() {
        let vel = Vec3::new(1.0, 0.0, 0.0);
        let segments = vec![seg(
            0.0,
            1.0,
            identity_pose(),
            vel,
            Vec3::zeros(),
            Vec3::zeros(),
        )];
        let mut frame = lidar_frame(0.0, 1.0, vec![pt(0.0, 2.0, 0.0, 0.0)]);
        deskew(&mut frame, &segments, &identity_extrinsic()).unwrap();
        assert_point_eq(&frame.points[0].point, (1.0, 0.0, 0.0));
    }

    // -----------------------------------------------------------------------
    // 纯旋转
    // -----------------------------------------------------------------------

    #[test]
    fn deskew_pure_rotation() {
        let ang_vel = Vec3::new(0.0, 0.0, f64c::FRAC_PI_2);
        let segments = vec![seg(
            0.0,
            1.0,
            identity_pose(),
            Vec3::zeros(),
            ang_vel,
            Vec3::zeros(),
        )];
        let mut frame = lidar_frame(0.0, 1.0, vec![pt(0.0, 1.0, 0.0, 0.0)]);
        deskew(&mut frame, &segments, &identity_extrinsic()).unwrap();
        assert_point_eq(&frame.points[0].point, (0.0, -1.0, 0.0));
    }

    // -----------------------------------------------------------------------
    // point_time == begin
    // -----------------------------------------------------------------------

    #[test]
    fn deskew_point_at_scan_begin() {
        let vel = Vec3::new(1.0, 0.0, 0.0);
        let segments = vec![seg(
            0.0,
            1.0,
            identity_pose(),
            vel,
            Vec3::zeros(),
            Vec3::zeros(),
        )];
        let mut frame = lidar_frame(0.0, 1.0, vec![pt(0.0, 2.0, 0.0, 0.0)]);
        deskew(&mut frame, &segments, &identity_extrinsic()).unwrap();
        assert_point_eq(&frame.points[0].point, (1.0, 0.0, 0.0));
    }

    // -----------------------------------------------------------------------
    // point_time == end
    // -----------------------------------------------------------------------

    #[test]
    fn deskew_point_at_scan_end() {
        let vel = Vec3::new(1.0, 0.0, 0.0);
        let segments = vec![seg(
            0.0,
            1.0,
            identity_pose(),
            vel,
            Vec3::zeros(),
            Vec3::zeros(),
        )];
        let mut frame = lidar_frame(0.0, 1.0, vec![pt(1.0, 2.0, 0.0, 0.0)]);
        deskew(&mut frame, &segments, &identity_extrinsic()).unwrap();
        assert_point_eq(&frame.points[0].point, (2.0, 0.0, 0.0));
    }

    // -----------------------------------------------------------------------
    // 非单位 T_LI
    // -----------------------------------------------------------------------

    #[test]
    fn deskew_non_identity_extrinsic() {
        let rot = UnitQuaternion::from_scaled_axis(Vec3::new(0.0, 0.0, f64c::FRAC_PI_2));
        let trans = Vec3::new(1.0, 2.0, 3.0);
        let extrinsic = Pose3::new(rot, trans);
        let vel = Vec3::new(1.0, 0.0, 0.0);
        let segments = vec![seg(
            0.0,
            1.0,
            identity_pose(),
            vel,
            Vec3::zeros(),
            Vec3::zeros(),
        )];
        let mut frame = lidar_frame(0.0, 1.0, vec![pt(0.0, 0.0, 0.0, 0.0)]);
        deskew(&mut frame, &segments, &extrinsic).unwrap();
        assert_point_eq(&frame.points[0].point, (0.0, 1.0, 0.0));
    }

    // -----------------------------------------------------------------------
    // 点时间越界
    // -----------------------------------------------------------------------

    #[test]
    fn deskew_point_time_before_segments_is_error() {
        let segments = vec![seg(
            0.5,
            1.0,
            identity_pose(),
            Vec3::zeros(),
            Vec3::zeros(),
            Vec3::zeros(),
        )];
        let mut frame = lidar_frame(0.0, 1.0, vec![pt(0.0, 1.0, 0.0, 0.0)]);
        assert!(deskew(&mut frame, &segments, &identity_extrinsic()).is_err());
    }

    #[test]
    fn deskew_point_time_after_segments_is_error() {
        let segments = vec![
            seg(
                0.0,
                0.5,
                identity_pose(),
                Vec3::zeros(),
                Vec3::zeros(),
                Vec3::zeros(),
            ),
            seg(
                0.5,
                1.0,
                identity_pose(),
                Vec3::zeros(),
                Vec3::zeros(),
                Vec3::zeros(),
            ),
        ];
        let mut frame = lidar_frame(0.0, 1.0, vec![pt(1.5, 1.0, 0.0, 0.0)]);
        assert!(deskew(&mut frame, &segments, &identity_extrinsic()).is_err());
    }

    #[test]
    fn deskew_empty_segments_is_error() {
        let mut frame = lidar_frame(0.0, 1.0, vec![pt(0.5, 1.0, 0.0, 0.0)]);
        assert!(deskew(&mut frame, &[], &identity_extrinsic()).is_err());
    }

    // -----------------------------------------------------------------------
    // build_motion_segments
    // -----------------------------------------------------------------------

    fn imu_sample(t: f64, gyro: Vec3<f64>, accel: Vec3<f64>) -> ImuSample {
        ImuSample {
            time_stamp_sec: t,
            gyro,
            accel,
        }
    }

    fn measure_group(imu: Vec<ImuSample>) -> MeasureGroup {
        MeasureGroup {
            imu,
            lidar: LidarFrame::new(0.0, 0.0, vec![]),
        }
    }

    fn navstate_zero() -> NavState {
        NavState {
            position: Vec3::zeros(),
            orientation: UnitQuaternion::identity(),
            velocity: Vec3::zeros(),
            gyro_bias: Vec3::zeros(),
            accel_bias: Vec3::zeros(),
            gravity: Vec3::zeros(),
        }
    }

    fn zero_integrator() -> ImuIntegrator {
        ImuIntegrator::init(0.0, 0.0, 0.0, 0.0)
    }

    #[test]
    fn build_segments_constant_angular_velocity() {
        let gyro = Vec3::new(0.0, 0.0, 1.0);
        let imu = vec![
            imu_sample(0.0, gyro, Vec3::zeros()),
            imu_sample(1.0, gyro, Vec3::zeros()),
        ];
        let group = measure_group(imu);
        let segments = build_motion_segments(&group, navstate_zero(), &zero_integrator()).unwrap();
        assert_eq!(segments.len(), 1);

        let s = &segments[0];
        assert_relative_eq!(s.begin_time, 0.0);
        assert_relative_eq!(s.end_time, 1.0);
        assert_relative_eq!(s.angular_velocity.x, 0.0);
        assert_relative_eq!(s.angular_velocity.y, 0.0);
        assert_relative_eq!(s.angular_velocity.z, 1.0);

        let p0 = s.propagate_to(0.0);
        assert_relative_eq!(p0.rotation.angle(), 0.0, epsilon = 1e-10);

        let p1 = s.propagate_to(1.0);
        let expected = UnitQuaternion::from_scaled_axis(Vec3::new(0.0, 0.0, 1.0));
        assert_relative_eq!(
            (p1.rotation * expected.inverse()).angle(),
            0.0,
            epsilon = 1e-10
        );
    }

    #[test]
    fn build_segments_constant_acceleration() {
        let accel = Vec3::new(1.0, 0.0, 0.0);
        let imu = vec![
            imu_sample(0.0, Vec3::zeros(), accel),
            imu_sample(1.0, Vec3::zeros(), accel),
        ];
        let group = measure_group(imu);
        let segments = build_motion_segments(&group, navstate_zero(), &zero_integrator()).unwrap();
        assert_eq!(segments.len(), 1);

        let s = &segments[0];
        assert_relative_eq!(s.begin_time, 0.0);
        assert_relative_eq!(s.end_time, 1.0);
        assert_relative_eq!(s.angular_velocity.x, 0.0);
        assert_relative_eq!(s.angular_velocity.y, 0.0);
        assert_relative_eq!(s.angular_velocity.z, 0.0);
        assert_relative_eq!(s.acceleration_world.x, 1.0);
        assert_relative_eq!(s.acceleration_world.y, 0.0);
        assert_relative_eq!(s.acceleration_world.z, 0.0);

        let p = s.propagate_to(1.0);
        assert_relative_eq!(p.translation.x, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn build_segments_empty_imu_returns_empty() {
        let group = measure_group(vec![imu_sample(0.0, Vec3::zeros(), Vec3::zeros())]);
        let segments = build_motion_segments(&group, navstate_zero(), &zero_integrator()).unwrap();
        assert!(segments.is_empty());
    }

    #[test]
    fn build_segments_includes_gravity_in_acceleration_world() {
        let gravity = Vec3::new(0.0, 0.0, -9.81);
        let accel = Vec3::new(0.0, 0.0, 0.0);
        let state = NavState {
            gravity,
            ..navstate_zero()
        };
        let imu = vec![
            imu_sample(0.0, Vec3::zeros(), accel),
            imu_sample(1.0, Vec3::zeros(), accel),
        ];
        let group = measure_group(imu);
        let segments = build_motion_segments(&group, state, &zero_integrator()).unwrap();
        assert_eq!(segments.len(), 1);
        assert_relative_eq!(segments[0].acceleration_world.z, -9.81, epsilon = 1e-10);
    }

    #[test]
    fn build_segments_gyro_bias_is_subtracted() {
        let gyro = Vec3::new(0.0, 0.0, 3.0);
        let state = NavState {
            gyro_bias: Vec3::new(0.0, 0.0, 1.0),
            ..navstate_zero()
        };
        let imu = vec![
            imu_sample(0.0, gyro, Vec3::zeros()),
            imu_sample(1.0, gyro, Vec3::zeros()),
        ];
        let group = measure_group(imu);
        let segments = build_motion_segments(&group, state, &zero_integrator()).unwrap();
        assert_eq!(segments.len(), 1);
        assert_relative_eq!(segments[0].angular_velocity.z, 2.0, epsilon = 1e-10);
    }
}
