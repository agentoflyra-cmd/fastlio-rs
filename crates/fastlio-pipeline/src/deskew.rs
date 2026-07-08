use anyhow::{anyhow, Result};
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
