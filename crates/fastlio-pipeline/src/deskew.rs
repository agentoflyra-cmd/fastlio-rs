use anyhow::{Result, anyhow};
use fastlio_imu::ImuIntegrator;
use fastlio_types::{LidarFrame, LidarImuExtrinsic, MeasureGroup, NavState, Pose3};
use nalgebra::UnitQuaternion;

use crate::trajectory::MotionSegment;

const TIME_EPS: f64 = 1.0e-6;

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

/// Deskew one raw LiDAR scan into the LiDAR frame at scan end time.
///
/// Input point coordinates are interpreted in `LiDAR(t_point)`, where
/// `t_point = lidar.base_timestamp_sec + point.offset_time_sec`.
/// Output point coordinates are written back in place and are expressed in
/// `LiDAR(lidar.end_timestamp_sec())`.
///
/// The input scan must keep points sorted by non-decreasing offset time. After
/// this function succeeds, point offsets are retained only as metadata; later
/// geometric stages such as voxel filtering must use the deskewed coordinates,
/// not the original per-point acquisition time.
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
    validate_lidar_offsets_for_deskew(lidar)?;

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

fn validate_lidar_offsets_for_deskew(lidar: &LidarFrame) -> Result<()> {
    let base_time = lidar.base_timestamp_sec;
    let end_time = lidar.end_timestamp_sec();

    if !base_time.is_finite() || !end_time.is_finite() {
        return Err(anyhow!(
            "deskew failed: lidar frame timestamp is not finite: base={base_time}, end={end_time}"
        ));
    }

    if end_time + TIME_EPS < base_time {
        return Err(anyhow!(
            "deskew failed: lidar frame end time {end_time} is before base time {base_time}"
        ));
    }

    let scan_duration = end_time - base_time;
    let mut last_offset = None;

    for (idx, point) in lidar.points.iter().enumerate() {
        let offset = point.offset_time_sec;
        if !offset.is_finite() {
            return Err(anyhow!(
                "deskew failed: point {idx} offset time is not finite: {offset}"
            ));
        }

        if offset + TIME_EPS < 0.0 {
            return Err(anyhow!(
                "deskew failed: point {idx} offset time {offset} is negative"
            ));
        }

        if offset > scan_duration + TIME_EPS {
            return Err(anyhow!(
                "deskew failed: point {idx} offset time {offset} exceeds scan duration {scan_duration}"
            ));
        }

        if let Some(prev_offset) = last_offset
            && offset + TIME_EPS < prev_offset
        {
            return Err(anyhow!(
                "deskew failed: point offsets must be sorted, point {idx} offset {offset} is before previous offset {prev_offset}"
            ));
        }

        last_offset = Some(offset);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use fastlio_types::{PointXYZI, TimedPoint, Vec3};

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

    fn identity_extrinsic() -> LidarImuExtrinsic {
        Pose3::new(UnitQuaternion::identity(), Vec3::zeros())
    }

    fn segment(begin_time: f64, end_time: f64, velocity: Vec3<f64>) -> MotionSegment {
        MotionSegment::new(
            begin_time,
            end_time,
            Pose3::new(UnitQuaternion::identity(), Vec3::zeros()),
            velocity,
            Vec3::zeros(),
            Vec3::zeros(),
        )
    }

    #[test]
    fn deskew_rejects_unsorted_offsets() {
        let mut lidar = LidarFrame::new(
            10.0,
            10.2,
            vec![
                point(0.0, 1.0, 0.0, 0.0),
                point(0.2, 2.0, 0.0, 0.0),
                point(0.1, 3.0, 0.0, 0.0),
            ],
        );
        let segments = vec![segment(10.0, 10.2, Vec3::zeros())];

        let err = deskew(&mut lidar, &segments, &identity_extrinsic()).unwrap_err();
        assert!(err.to_string().contains("offsets must be sorted"));
    }

    #[test]
    fn deskew_rejects_point_time_outside_scan() {
        let mut lidar = LidarFrame::new(10.0, 10.2, vec![point(0.3, 1.0, 0.0, 0.0)]);
        let segments = vec![segment(10.0, 10.2, Vec3::zeros())];

        let err = deskew(&mut lidar, &segments, &identity_extrinsic()).unwrap_err();
        assert!(err.to_string().contains("exceeds scan duration"));
    }

    #[test]
    fn deskew_rejects_non_finite_offset() {
        let mut lidar = LidarFrame::new(10.0, 10.2, vec![point(f64::NAN, 1.0, 0.0, 0.0)]);
        let segments = vec![segment(10.0, 10.2, Vec3::zeros())];

        let err = deskew(&mut lidar, &segments, &identity_extrinsic()).unwrap_err();
        assert!(err.to_string().contains("not finite"));
    }

    #[test]
    fn stationary_deskew_preserves_points() {
        let mut lidar = LidarFrame::new(
            10.0,
            10.2,
            vec![
                point(0.0, 1.0, 2.0, 3.0),
                point(0.1, 4.0, 5.0, 6.0),
                point(0.2, 7.0, 8.0, 9.0),
            ],
        );
        let segments = vec![segment(10.0, 10.2, Vec3::zeros())];

        deskew(&mut lidar, &segments, &identity_extrinsic()).unwrap();

        assert_eq!(lidar.points[0].point.x, 1.0);
        assert_eq!(lidar.points[0].point.y, 2.0);
        assert_eq!(lidar.points[0].point.z, 3.0);
        assert_eq!(lidar.points[1].point.x, 4.0);
        assert_eq!(lidar.points[1].point.y, 5.0);
        assert_eq!(lidar.points[1].point.z, 6.0);
        assert_eq!(lidar.points[2].point.x, 7.0);
        assert_eq!(lidar.points[2].point.y, 8.0);
        assert_eq!(lidar.points[2].point.z, 9.0);
    }

    #[test]
    fn constant_velocity_deskew_outputs_lidar_end_frame() {
        let mut lidar = LidarFrame::new(
            10.0,
            10.2,
            vec![
                point(0.0, 1.0, 0.0, 0.0),
                point(0.1, 1.0, 0.0, 0.0),
                point(0.2, 1.0, 0.0, 0.0),
            ],
        );
        let segments = vec![segment(10.0, 10.2, Vec3::new(1.0, 0.0, 0.0))];

        deskew(&mut lidar, &segments, &identity_extrinsic()).unwrap();

        assert!((lidar.points[0].point.x - 0.8).abs() < 1.0e-6);
        assert!((lidar.points[1].point.x - 0.9).abs() < 1.0e-6);
        assert!((lidar.points[2].point.x - 1.0).abs() < 1.0e-6);
    }
}
