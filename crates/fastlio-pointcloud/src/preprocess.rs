use anyhow::Result;
use fastlio_types::{LidarFrame, PointXYZI, PreprocessConfig, TimedPoint};

#[derive(PartialEq, Eq, PartialOrd, Ord, Copy, Clone)]
struct VoxelKey {
    x: i32,
    y: i32,
    z: i32,
}

impl VoxelKey {
    #[inline]
    fn new(point: &PointXYZI, voxel_size: f32) -> Result<Self> {
        if !voxel_size.is_normal() || voxel_size < 0.0 {
            anyhow::bail!("voxel_size should be a valid number(f32).")
        }
        let x = (point.x / voxel_size).floor() as i32;
        let y = (point.y / voxel_size).floor() as i32;
        let z = (point.z / voxel_size).floor() as i32;
        Ok(Self { x, y, z })
    }
}

pub fn voxel(points: Vec<TimedPoint>, voxel_size: f32) -> Result<Vec<TimedPoint>> {
    let mut voxel_map: Vec<(VoxelKey, TimedPoint)> = points
        .into_iter()
        .map(|p| {
            let voxel_key = VoxelKey::new(&p.point, voxel_size)?;
            Ok((voxel_key, p))
        })
        .collect::<Result<Vec<_>>>()?;
    voxel_map.sort_unstable_by_key(|x| x.0);
    let mut iter = voxel_map.into_iter().peekable();
    let mut result = Vec::<TimedPoint>::new();
    while let Some((voxel_key, timed_point)) = iter.next() {
        let mut acc = timed_point;
        let mut count = 1;
        while let Some((next_key, _)) = iter.peek() {
            if next_key != &voxel_key {
                break;
            }
            // peek() ensure here never panic, so it is safe to use unwrap()
            let (_, next_point) = iter.next().unwrap();
            count += 1;
            acc.add(&next_point);
        }

        acc.point.x /= count as f32;
        acc.point.y /= count as f32;
        acc.point.z /= count as f32;
        acc.point.intensity /= count as f32;
        result.push(acc);
    }
    Ok(result)
}

pub fn preprocess(
    preprocess_config: &PreprocessConfig,
    lidar_frame: LidarFrame,
) -> Result<LidarFrame> {
    let base_timestamp_sec = lidar_frame.base_timestamp_sec;
    let end_timestamp_sec = lidar_frame.end_timestamp_sec();
    let points = lidar_frame.points;
    let max_distance = preprocess_config.max_range;
    let blind_zone = preprocess_config.blind_zone;
    let voxel_size = preprocess_config.voxel_size;
    let n_scans = preprocess_config.scan_line;
    // TODO: implement for different lidar type.
    // let lidar_type = preprocess_config.lidar_type;

    let mut points = points
        .into_iter()
        .filter(|p| {
            let distance = p.point.squared_distance();
            p.is_valid()
                && distance >= (blind_zone * blind_zone)
                && (max_distance.is_none_or(|max_distance| distance <= max_distance * max_distance))
                && (n_scans.is_none_or(|n_scans| p.line < n_scans)
                    && ((p.tag & 0x30) == 0x10 || (p.tag & 0x30) == 0x00))
        })
        .collect();
    if let Some(voxel_size) = voxel_size {
        points = voxel(points, voxel_size)?;
    }
    Ok(LidarFrame::new(
        base_timestamp_sec,
        end_timestamp_sec,
        points,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use fastlio_types::LidarType;

    fn point(x: f32, y: f32, z: f32, intensity: f32) -> PointXYZI {
        PointXYZI { x, y, z, intensity }
    }

    fn tp(offset: f64, x: f32, y: f32, z: f32, intensity: f32) -> TimedPoint {
        TimedPoint {
            offset_time_sec: offset,
            point: point(x, y, z, intensity),
            tag: 0x1,
            line: 1,
        }
    }

    fn config(
        blind_zone: f32,
        max_range: Option<f32>,
        voxel_size: Option<f32>,
    ) -> PreprocessConfig {
        PreprocessConfig {
            lidar_type: LidarType::Avia,
            scan_line: None,
            blind_zone,
            voxel_size,
            max_range,
        }
    }

    fn frame(base: f64, end: f64, points: Vec<TimedPoint>) -> LidarFrame {
        LidarFrame::new(base, end, points)
    }

    fn fpoints() -> Vec<TimedPoint> {
        vec![
            tp(0.0, 1.0, 0.0, 0.0, 10.0),
            tp(0.1, 0.0, 2.0, 0.0, 20.0),
            tp(0.2, 0.0, 0.0, 3.0, 30.0),
        ]
    }

    // ---------------------------------------------------------------------------
    // Non-finite value filtering
    // ---------------------------------------------------------------------------

    #[test]
    fn filter_nan_coordinates() {
        let points = vec![
            tp(0.0, 1.0, 0.0, 0.0, 10.0),
            tp(0.1, f32::NAN, 0.0, 0.0, 10.0),
            tp(0.2, 0.0, f32::NAN, 0.0, 10.0),
            tp(0.3, 0.0, 0.0, f32::NAN, 10.0),
            tp(0.4, 1.0, 1.0, 1.0, 10.0),
        ];
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.5, points)).unwrap();
        assert_eq!(result.points.len(), 2);
        assert_eq!(result.points[0].offset_time_sec, 0.0);
        assert_eq!(result.points[1].offset_time_sec, 0.4);
    }

    #[test]
    fn filter_inf_coordinates() {
        let points = vec![
            tp(0.0, 1.0, 0.0, 0.0, 10.0),
            tp(0.1, f32::INFINITY, 0.0, 0.0, 10.0),
            tp(0.2, 0.0, f32::NEG_INFINITY, 0.0, 10.0),
            tp(0.3, 0.0, 0.0, f32::INFINITY, 10.0),
            tp(0.4, 1.0, 1.0, 1.0, 10.0),
        ];
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.5, points)).unwrap();
        assert_eq!(result.points.len(), 2);
        assert_eq!(result.points[0].offset_time_sec, 0.0);
        assert_eq!(result.points[1].offset_time_sec, 0.4);
    }

    #[test]
    fn filter_nan_intensity() {
        let points = vec![
            tp(0.0, 1.0, 0.0, 0.0, f32::NAN),
            tp(0.1, 1.0, 1.0, 1.0, 10.0),
        ];
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.points.len(), 1);
        assert_eq!(result.points[0].point.intensity, 10.0);
    }

    #[test]
    fn filter_negative_intensity() {
        let points = vec![tp(0.0, 1.0, 0.0, 0.0, -1.0), tp(0.1, 1.0, 1.0, 1.0, 10.0)];
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.points.len(), 1);
        assert_eq!(result.points[0].point.intensity, 10.0);
    }

    #[test]
    fn filter_inf_intensity() {
        let points = vec![
            tp(0.0, 1.0, 0.0, 0.0, f32::INFINITY),
            tp(0.1, 1.0, 1.0, 1.0, 10.0),
        ];
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.points.len(), 1);
        assert_eq!(result.points[0].point.intensity, 10.0);
    }

    #[test]
    fn filter_nan_offset_time() {
        let points = vec![
            tp(f64::NAN, 1.0, 0.0, 0.0, 10.0),
            tp(0.1, 1.0, 1.0, 1.0, 10.0),
        ];
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.points.len(), 1);
        assert_eq!(result.points[0].offset_time_sec, 0.1);
    }

    #[test]
    fn filter_inf_offset_time() {
        let points = vec![
            tp(f64::INFINITY, 1.0, 0.0, 0.0, 10.0),
            tp(0.1, 1.0, 1.0, 1.0, 10.0),
        ];
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.points.len(), 1);
        assert_eq!(result.points[0].offset_time_sec, 0.1);
    }

    // ---------------------------------------------------------------------------
    // Blind zone
    // ---------------------------------------------------------------------------

    #[test]
    fn filter_blind_zone_inside() {
        let points = vec![
            tp(0.0, 0.0, 0.0, 0.0, 10.0),
            tp(0.1, 0.4, 0.0, 0.0, 10.0),
            tp(0.2, 1.0, 0.0, 0.0, 10.0),
        ];
        let result = preprocess(&config(0.5, None, None), frame(0.0, 0.3, points)).unwrap();
        assert_eq!(result.points.len(), 1);
        assert_eq!(result.points[0].offset_time_sec, 0.2);
    }

    #[test]
    fn filter_blind_zone_boundary_kept() {
        let points = vec![
            tp(0.0, 0.5, 0.0, 0.0, 10.0),
            tp(0.1, 0.5_f32.next_down(), 0.0, 0.0, 10.0),
        ];
        let result = preprocess(&config(0.5, None, None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.points.len(), 1);
        assert_eq!(result.points[0].offset_time_sec, 0.0);
    }

    #[test]
    fn filter_blind_zone_zero_keeps_all() {
        let points = vec![tp(0.0, 0.0, 0.0, 0.0, 10.0), tp(0.1, 0.1, 0.0, 0.0, 10.0)];
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.points.len(), 2);
    }

    // ---------------------------------------------------------------------------
    // Max range
    // ---------------------------------------------------------------------------

    #[test]
    fn filter_max_range_beyond() {
        let points = vec![tp(0.0, 1.0, 0.0, 0.0, 10.0), tp(0.1, 6.0, 0.0, 0.0, 10.0)];
        let result = preprocess(&config(0.0, Some(5.0), None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.points.len(), 1);
        assert_eq!(result.points[0].offset_time_sec, 0.0);
    }

    #[test]
    fn filter_max_range_boundary_kept() {
        let points = vec![
            tp(0.0, 5.0, 0.0, 0.0, 10.0),
            tp(0.1, 5.0_f32.next_up(), 0.0, 0.0, 10.0),
        ];
        let result = preprocess(&config(0.0, Some(5.0), None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.points.len(), 1);
        assert_eq!(result.points[0].offset_time_sec, 0.0);
    }

    #[test]
    fn filter_max_range_none_keeps_distant() {
        let points = vec![tp(0.0, 1.0, 0.0, 0.0, 10.0), tp(0.1, 100.0, 0.0, 0.0, 10.0)];
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.points.len(), 2);
    }

    // ---------------------------------------------------------------------------
    // Voxel downsampling
    // ---------------------------------------------------------------------------

    #[test]
    fn voxel_distinct_cells_kept_separate() {
        let points = vec![
            tp(0.0, 0.5, 0.0, 0.0, 10.0),
            tp(0.1, 1.5, 0.0, 0.0, 20.0),
            tp(0.2, 2.5, 0.0, 0.0, 30.0),
        ];
        let result = voxel(points, 1.0).unwrap();
        assert_eq!(result.len(), 3);
        assert_eq!(result[0].point.intensity, 10.0);
        assert_eq!(result[1].point.intensity, 20.0);
        assert_eq!(result[2].point.intensity, 30.0);
    }

    #[test]
    fn voxel_accumulates_same_cell() {
        let points = vec![tp(0.0, 1.0, 2.0, 3.0, 10.0), tp(0.1, 4.0, 5.0, 6.0, 20.0)];
        let result = voxel(points, 10.0).unwrap();
        assert_eq!(result.len(), 1);
        let p = &result[0];
        assert_eq!(p.point.x, 2.5);
        assert_eq!(p.point.y, 3.5);
        assert_eq!(p.point.z, 4.5);
        assert_eq!(p.point.intensity, 15.0);
    }

    #[test]
    fn voxel_preserves_first_offset_time() {
        let points = vec![tp(1.5, 0.2, 0.0, 0.0, 10.0), tp(2.5, 0.5, 0.0, 0.0, 20.0)];
        let result = voxel(points, 1.0).unwrap();
        assert_eq!(result.len(), 1);
        assert_eq!(result[0].offset_time_sec, 1.5);
    }

    #[test]
    fn voxel_empty_input() {
        let result = voxel(Vec::new(), 1.0).unwrap();
        assert!(result.is_empty());
    }

    #[test]
    fn voxel_reduces_count() {
        let mut pts = Vec::new();
        for i in 0..5 {
            for _ in 0..10 {
                pts.push(tp((i * 10i32) as f64, i as f32 + 0.1, 0.0, 0.0, 1.0));
            }
        }
        let result = voxel(pts, 1.0).unwrap();
        assert_eq!(result.len(), 5);
    }

    #[test]
    fn voxel_different_z_separates() {
        let points = vec![
            tp(0.0, 0.5, 0.0, 0.5, 10.0),
            tp(0.1, 0.5, 0.0, 1.5, 20.0),
            tp(0.2, 0.5, 0.0, 2.5, 30.0),
        ];
        let result = voxel(points, 1.0).unwrap();
        assert_eq!(result.len(), 3);
    }

    #[test]
    fn voxel_negative_coordinates_group_correctly() {
        let points = vec![
            tp(0.0, -0.5, -0.5, -0.5, 10.0),
            tp(0.1, -0.3, -0.3, -0.3, 20.0),
            tp(0.2, 0.5, 0.5, 0.5, 30.0),
        ];
        let result = voxel(points, 1.0).unwrap();
        assert_eq!(result.len(), 2);
    }

    // ---------------------------------------------------------------------------
    // Offset preservation through preprocess
    // ---------------------------------------------------------------------------

    #[test]
    fn preserve_offset_time_on_surviving_points() {
        let points = fpoints();
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.3, points)).unwrap();
        assert_eq!(result.points.len(), 3);
        assert_eq!(result.points[0].offset_time_sec, 0.0);
        assert_eq!(result.points[1].offset_time_sec, 0.1);
        assert_eq!(result.points[2].offset_time_sec, 0.2);
    }

    #[test]
    fn preserve_lidar_frame_timestamps() {
        let points = vec![tp(0.05, 1.0, 0.0, 0.0, 10.0)];
        let result = preprocess(&config(0.0, None, None), frame(1000.0, 1000.1, points)).unwrap();
        assert_eq!(result.base_timestamp_sec, 1000.0);
        assert_eq!(result.end_timestamp_sec(), 1000.1);
    }

    // ---------------------------------------------------------------------------
    // Empty input
    // ---------------------------------------------------------------------------

    #[test]
    fn preprocess_empty_frame() {
        let result = preprocess(
            &config(0.5, Some(10.0), Some(1.0)),
            frame(0.0, 0.1, Vec::new()),
        )
        .unwrap();
        assert!(result.points.is_empty());
        assert_eq!(result.base_timestamp_sec, 0.0);
        assert_eq!(result.end_timestamp_sec(), 0.1);
    }

    // ---------------------------------------------------------------------------
    // All filtered to empty
    // ---------------------------------------------------------------------------

    #[test]
    fn all_points_filtered_by_validity() {
        let points = vec![
            tp(0.0, f32::NAN, 0.0, 0.0, 10.0),
            tp(0.1, f32::INFINITY, 0.0, 0.0, 10.0),
            tp(0.2, 0.0, f32::NAN, 0.0, 10.0),
        ];
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.3, points)).unwrap();
        assert!(result.points.is_empty());
    }

    #[test]
    fn all_points_filtered_by_blind_zone() {
        let points = vec![tp(0.0, 0.1, 0.0, 0.0, 10.0), tp(0.1, 0.0, 0.2, 0.0, 10.0)];
        let result = preprocess(&config(1.0, None, None), frame(0.0, 0.2, points)).unwrap();
        assert!(result.points.is_empty());
    }

    #[test]
    fn all_points_filtered_by_max_range() {
        let points = vec![tp(0.0, 10.0, 0.0, 0.0, 10.0), tp(0.1, 0.0, 20.0, 0.0, 10.0)];
        let result = preprocess(&config(0.0, Some(1.0), None), frame(0.0, 0.2, points)).unwrap();
        assert!(result.points.is_empty());
    }

    // ---------------------------------------------------------------------------
    // Determinism
    // ---------------------------------------------------------------------------

    #[test]
    fn preprocess_is_deterministic() {
        let points_a = fpoints();
        let points_b = fpoints();
        let res_a = preprocess(&config(0.0, None, None), frame(0.0, 0.3, points_a)).unwrap();
        let res_b = preprocess(&config(0.0, None, None), frame(0.0, 0.3, points_b)).unwrap();
        assert_eq!(res_a.points.len(), res_b.points.len());
        for (a, b) in res_a.points.iter().zip(res_b.points.iter()) {
            assert_eq!(a.offset_time_sec, b.offset_time_sec);
            assert_eq!(a.point.x, b.point.x);
            assert_eq!(a.point.y, b.point.y);
            assert_eq!(a.point.z, b.point.z);
            assert_eq!(a.point.intensity, b.point.intensity);
        }
    }

    #[test]
    fn voxel_is_deterministic() {
        let points_a = vec![
            tp(0.0, 0.5, 0.0, 0.0, 10.0),
            tp(0.1, 0.5, 0.0, 0.0, 20.0),
            tp(0.2, 1.5, 0.0, 0.0, 30.0),
        ];
        let points_b = vec![
            tp(0.0, 0.5, 0.0, 0.0, 10.0),
            tp(0.1, 0.5, 0.0, 0.0, 20.0),
            tp(0.2, 1.5, 0.0, 0.0, 30.0),
        ];
        let res_a = voxel(points_a, 1.0).unwrap();
        let res_b = voxel(points_b, 1.0).unwrap();
        assert_eq!(res_a.len(), res_b.len());
        for (a, b) in res_a.iter().zip(res_b.iter()) {
            assert_eq!(a.offset_time_sec, b.offset_time_sec);
            assert_eq!(a.point.x, b.point.x);
            assert_eq!(a.point.y, b.point.y);
            assert_eq!(a.point.z, b.point.z);
            assert_eq!(a.point.intensity, b.point.intensity);
        }
    }

    // ---------------------------------------------------------------------------
    // Voxel integration through preprocess
    // ---------------------------------------------------------------------------

    #[test]
    fn preprocess_with_voxel_downsampling() {
        let points = vec![
            tp(0.0, 0.2, 1.0, 0.0, 10.0),
            tp(0.1, 0.8, 1.0, 0.0, 20.0),
            tp(0.2, 3.0, 1.0, 0.0, 30.0),
            tp(0.3, f32::NAN, 1.0, 0.0, 40.0),
            tp(0.4, 0.1, 0.1, 0.0, 50.0),
        ];
        let result =
            preprocess(&config(0.5, Some(10.0), Some(1.0)), frame(0.0, 0.5, points)).unwrap();
        assert_eq!(result.points.len(), 2);
    }
}
