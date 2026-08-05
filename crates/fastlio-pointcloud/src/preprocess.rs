use anyhow::Result;
use fastlio_map::VoxelKey;
use fastlio_types::{LidarFrame, PointCloud, PointXYZI, PreprocessConfig};

pub fn voxel(points: PointCloud, voxel_size: f32) -> Result<PointCloud> {
    let mut voxel_map: Vec<(VoxelKey, PointXYZI)> = points
        .into_iter()
        .map(|p| {
            let voxel_key = VoxelKey::new(&p, voxel_size)?;
            Ok((voxel_key, p))
        })
        .collect::<Result<Vec<_>>>()?;
    voxel_map.sort_unstable_by_key(|x| x.0);
    let mut iter = voxel_map.into_iter().peekable();
    let mut result = Vec::<PointXYZI>::new();
    while let Some((voxel_key, point)) = iter.next() {
        let mut acc = point;
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

        acc.x /= count as f32;
        acc.y /= count as f32;
        acc.z /= count as f32;
        acc.intensity /= count as f32;
        result.push(acc);
    }
    Ok(result)
}

pub struct PointCloudFrame {
    pub timestamp_sec: f64,
    pub point_cloud: PointCloud,
}

impl PointCloudFrame {
    pub fn new(timestamp_sec: f64, point_cloud: PointCloud) -> Self {
        Self {
            timestamp_sec,
            point_cloud,
        }
    }
}

pub fn preprocess(
    preprocess_config: &PreprocessConfig,
    lidar_frame: LidarFrame,
) -> Result<PointCloudFrame> {
    let timestamp_sec = lidar_frame.end_timestamp_sec();
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
        .map(|tp| tp.point)
        .collect::<PointCloud>();
    if let Some(voxel_size) = voxel_size {
        points = voxel(points, voxel_size)?;
    }
    Ok(PointCloudFrame::new(timestamp_sec, points))
}

#[cfg(test)]
mod tests {
    use super::*;
    use fastlio_types::{LidarType, TimedPoint};

    fn pt(x: f32, y: f32, z: f32, intensity: f32) -> PointXYZI {
        PointXYZI { x, y, z, intensity }
    }

    fn tp(offset: f64, x: f32, y: f32, z: f32, intensity: f32) -> TimedPoint {
        TimedPoint {
            offset_time_sec: offset,
            point: pt(x, y, z, intensity),
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
        assert_eq!(result.point_cloud.len(), 2);
        assert_eq!(result.point_cloud[0].x, 1.0);
        assert_eq!(result.point_cloud[1].x, 1.0);
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
        assert_eq!(result.point_cloud.len(), 2);
        assert_eq!(result.point_cloud[0].x, 1.0);
        assert_eq!(result.point_cloud[1].x, 1.0);
    }

    #[test]
    fn filter_nan_intensity() {
        let points = vec![
            tp(0.0, 1.0, 0.0, 0.0, f32::NAN),
            tp(0.1, 1.0, 1.0, 1.0, 10.0),
        ];
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.point_cloud.len(), 1);
        assert_eq!(result.point_cloud[0].intensity, 10.0);
    }

    #[test]
    fn filter_negative_intensity() {
        let points = vec![tp(0.0, 1.0, 0.0, 0.0, -1.0), tp(0.1, 1.0, 1.0, 1.0, 10.0)];
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.point_cloud.len(), 1);
        assert_eq!(result.point_cloud[0].intensity, 10.0);
    }

    #[test]
    fn filter_inf_intensity() {
        let points = vec![
            tp(0.0, 1.0, 0.0, 0.0, f32::INFINITY),
            tp(0.1, 1.0, 1.0, 1.0, 10.0),
        ];
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.point_cloud.len(), 1);
        assert_eq!(result.point_cloud[0].intensity, 10.0);
    }

    #[test]
    fn filter_nan_offset_time() {
        let points = vec![
            tp(f64::NAN, 1.0, 0.0, 0.0, 10.0),
            tp(0.1, 1.0, 1.0, 1.0, 10.0),
        ];
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.point_cloud.len(), 1);
        assert_eq!(result.point_cloud[0].x, 1.0);
    }

    #[test]
    fn filter_inf_offset_time() {
        let points = vec![
            tp(f64::INFINITY, 1.0, 0.0, 0.0, 10.0),
            tp(0.1, 1.0, 1.0, 1.0, 10.0),
        ];
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.point_cloud.len(), 1);
        assert_eq!(result.point_cloud[0].x, 1.0);
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
        assert_eq!(result.point_cloud.len(), 1);
        assert_eq!(result.point_cloud[0].x, 1.0);
    }

    #[test]
    fn filter_blind_zone_boundary_kept() {
        let points = vec![
            tp(0.0, 0.5, 0.0, 0.0, 10.0),
            tp(0.1, 0.5_f32.next_down(), 0.0, 0.0, 10.0),
        ];
        let result = preprocess(&config(0.5, None, None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.point_cloud.len(), 1);
        assert_eq!(result.point_cloud[0].x, 0.5);
    }

    #[test]
    fn filter_blind_zone_zero_keeps_all() {
        let points = vec![tp(0.0, 0.0, 0.0, 0.0, 10.0), tp(0.1, 0.1, 0.0, 0.0, 10.0)];
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.point_cloud.len(), 2);
    }

    // ---------------------------------------------------------------------------
    // Max range
    // ---------------------------------------------------------------------------

    #[test]
    fn filter_max_range_beyond() {
        let points = vec![tp(0.0, 1.0, 0.0, 0.0, 10.0), tp(0.1, 6.0, 0.0, 0.0, 10.0)];
        let result = preprocess(&config(0.0, Some(5.0), None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.point_cloud.len(), 1);
        assert_eq!(result.point_cloud[0].x, 1.0);
    }

    #[test]
    fn filter_max_range_boundary_kept() {
        let points = vec![
            tp(0.0, 5.0, 0.0, 0.0, 10.0),
            tp(0.1, 5.0_f32.next_up(), 0.0, 0.0, 10.0),
        ];
        let result = preprocess(&config(0.0, Some(5.0), None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.point_cloud.len(), 1);
        assert_eq!(result.point_cloud[0].x, 5.0);
    }

    #[test]
    fn filter_max_range_none_keeps_distant() {
        let points = vec![tp(0.0, 1.0, 0.0, 0.0, 10.0), tp(0.1, 100.0, 0.0, 0.0, 10.0)];
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.2, points)).unwrap();
        assert_eq!(result.point_cloud.len(), 2);
    }

    // ---------------------------------------------------------------------------
    // Voxel downsampling
    // ---------------------------------------------------------------------------

    #[test]
    fn voxel_distinct_cells_kept_separate() {
        let points = vec![
            pt(0.5, 0.0, 0.0, 10.0),
            pt(1.5, 0.0, 0.0, 20.0),
            pt(2.5, 0.0, 0.0, 30.0),
        ];
        let result = voxel(points, 1.0).unwrap();
        assert_eq!(result.len(), 3);
        assert_eq!(result[0].intensity, 10.0);
        assert_eq!(result[1].intensity, 20.0);
        assert_eq!(result[2].intensity, 30.0);
    }

    #[test]
    fn voxel_accumulates_same_cell() {
        let points = vec![pt(1.0, 2.0, 3.0, 10.0), pt(4.0, 5.0, 6.0, 20.0)];
        let result = voxel(points, 10.0).unwrap();
        assert_eq!(result.len(), 1);
        let p = &result[0];
        assert_eq!(p.x, 2.5);
        assert_eq!(p.y, 3.5);
        assert_eq!(p.z, 4.5);
        assert_eq!(p.intensity, 15.0);
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
                pts.push(pt(i as f32 + 0.1, 0.0, 0.0, 1.0));
            }
        }
        let result = voxel(pts, 1.0).unwrap();
        assert_eq!(result.len(), 5);
    }

    #[test]
    fn voxel_different_z_separates() {
        let points = vec![
            pt(0.5, 0.0, 0.5, 10.0),
            pt(0.5, 0.0, 1.5, 20.0),
            pt(0.5, 0.0, 2.5, 30.0),
        ];
        let result = voxel(points, 1.0).unwrap();
        assert_eq!(result.len(), 3);
    }

    #[test]
    fn voxel_negative_coordinates_group_correctly() {
        let points = vec![
            pt(-0.5, -0.5, -0.5, 10.0),
            pt(-0.3, -0.3, -0.3, 20.0),
            pt(0.5, 0.5, 0.5, 30.0),
        ];
        let result = voxel(points, 1.0).unwrap();
        assert_eq!(result.len(), 2);
    }

    // ---------------------------------------------------------------------------
    // Offset / timestamp preservation through preprocess
    // ---------------------------------------------------------------------------

    #[test]
    fn preserve_point_count_after_preprocess() {
        let points = fpoints();
        let result = preprocess(&config(0.0, None, None), frame(0.0, 0.3, points)).unwrap();
        assert_eq!(result.point_cloud.len(), 3);
    }

    #[test]
    fn preserve_lidar_frame_timestamp() {
        let points = vec![tp(0.05, 1.0, 0.0, 0.0, 10.0)];
        let result = preprocess(&config(0.0, None, None), frame(1000.0, 1000.1, points)).unwrap();
        assert_eq!(result.timestamp_sec, 1000.1);
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
        assert!(result.point_cloud.is_empty());
        assert_eq!(result.timestamp_sec, 0.1);
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
        assert!(result.point_cloud.is_empty());
    }

    #[test]
    fn all_points_filtered_by_blind_zone() {
        let points = vec![tp(0.0, 0.1, 0.0, 0.0, 10.0), tp(0.1, 0.0, 0.2, 0.0, 10.0)];
        let result = preprocess(&config(1.0, None, None), frame(0.0, 0.2, points)).unwrap();
        assert!(result.point_cloud.is_empty());
    }

    #[test]
    fn all_points_filtered_by_max_range() {
        let points = vec![tp(0.0, 10.0, 0.0, 0.0, 10.0), tp(0.1, 0.0, 20.0, 0.0, 10.0)];
        let result = preprocess(&config(0.0, Some(1.0), None), frame(0.0, 0.2, points)).unwrap();
        assert!(result.point_cloud.is_empty());
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
        assert_eq!(res_a.point_cloud.len(), res_b.point_cloud.len());
        for (a, b) in res_a.point_cloud.iter().zip(res_b.point_cloud.iter()) {
            assert_eq!(a.x, b.x);
            assert_eq!(a.y, b.y);
            assert_eq!(a.z, b.z);
            assert_eq!(a.intensity, b.intensity);
        }
    }

    #[test]
    fn voxel_is_deterministic() {
        let points_a = vec![
            pt(0.5, 0.0, 0.0, 10.0),
            pt(0.5, 0.0, 0.0, 20.0),
            pt(1.5, 0.0, 0.0, 30.0),
        ];
        let points_b = vec![
            pt(0.5, 0.0, 0.0, 10.0),
            pt(0.5, 0.0, 0.0, 20.0),
            pt(1.5, 0.0, 0.0, 30.0),
        ];
        let res_a = voxel(points_a, 1.0).unwrap();
        let res_b = voxel(points_b, 1.0).unwrap();
        assert_eq!(res_a.len(), res_b.len());
        for (a, b) in res_a.iter().zip(res_b.iter()) {
            assert_eq!(a.x, b.x);
            assert_eq!(a.y, b.y);
            assert_eq!(a.z, b.z);
            assert_eq!(a.intensity, b.intensity);
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
        assert_eq!(result.point_cloud.len(), 2);
    }
}
