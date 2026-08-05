pub mod plane;
pub mod surfel;
pub mod types;
pub mod voxelmap;

pub use types::VoxelKey;
pub use voxelmap::{NearestPoint, VoxelMap};

#[cfg(test)]
mod test {
    use std::collections::HashSet;

    use crate::voxelmap::NearestPoint;
    use crate::{VoxelKey, VoxelMap};
    use fastlio_types::{PointXYZI, Vec3};

    fn pt(x: f32, y: f32, z: f32) -> PointXYZI {
        PointXYZI {
            x,
            y,
            z,
            intensity: 0.0,
        }
    }

    fn key_at(x: f32, y: f32, z: f32) -> VoxelKey {
        VoxelKey::new(&pt(x, y, z), 1.0).expect("valid voxel size")
    }

    fn sorted(points: &[PointXYZI]) -> Vec<[f32; 4]> {
        let mut v: Vec<[f32; 4]> = points
            .iter()
            .map(|p| [p.x, p.y, p.z, p.intensity])
            .collect();
        v.sort_by(|a, b| {
            a[0].total_cmp(&b[0])
                .then(a[1].total_cmp(&b[1]))
                .then(a[2].total_cmp(&b[2]))
                .then(a[3].total_cmp(&b[3]))
        });
        v
    }

    fn sample_points() -> Vec<PointXYZI> {
        vec![
            pt(0.1, 0.1, 0.1),
            pt(0.15, 0.12, 0.11),
            pt(0.1, 0.1, 0.1),
            pt(-1.3, 0.4, 2.2),
            pt(3.0, -3.0, 3.0),
            pt(-0.05, -0.05, -0.05),
            pt(10.0, 10.0, 10.0),
        ]
    }

    #[test]
    fn nearby_points_and_nearby_points_iter_return_the_same_set() {
        let mut map = VoxelMap::new(0.5);
        map.insert(sample_points()).unwrap();

        let queries = [
            (Vec3::new(0.0f32, 0.0, 0.0), 0.0f32),
            (Vec3::new(0.0, 0.0, 0.0), 0.2),
            (Vec3::new(0.0, 0.0, 0.0), 5.0),
            (Vec3::new(-1.0, 0.5, 2.0), 3.0),
            (Vec3::new(-10.0, -10.0, -10.0), 1.0),
            (Vec3::new(10.0, 10.0, 10.0), 0.01),
        ];

        for (q, r) in queries {
            let owned = sorted(&map.nearby_points(&q, r));
            let borrowed: Vec<_> = map.nearby_points_iter(&q, r).cloned().collect();
            assert_eq!(
                owned,
                sorted(&borrowed),
                "mismatch at query ({}, {}, {}) radius {}",
                q.x,
                q.y,
                q.z,
                r
            );
        }

        let q = Vec3::new(0.0f32, 0.0, 0.0);
        assert!(map.nearby_points(&q, 0.0).is_empty());
        assert_eq!(map.nearby_points(&q, 0.2).len(), 3);
    }

    #[test]
    fn nearby_radius_boundary_is_inclusive() {
        let mut map = VoxelMap::new(1.0);
        map.insert([pt(2.0, 0.0, 0.0), pt(-2.0, 0.0, 0.0), pt(2.5, 0.0, 0.0)])
            .unwrap();

        let q = Vec3::new(0.0f32, 0.0, 0.0);
        let owned = map.nearby_points(&q, 2.0);
        assert_eq!(owned.len(), 2);
        assert!(owned.iter().all(|p| p.x.abs() <= 2.0));
        assert_eq!(map.nearby_points_iter(&q, 2.0).count(), 2);
    }

    #[test]
    fn negative_coordinate_query_returns_expected_points() {
        let mut map = VoxelMap::new(1.0);
        map.insert([
            pt(-3.2, -4.4, -5.6),
            pt(-0.6, -0.6, -0.6),
            pt(5.0, 5.0, 5.0),
        ])
        .unwrap();

        let q = Vec3::new(-1.0f32, -1.0, -1.0);
        let owned = map.nearby_points(&q, 2.0);
        assert_eq!(owned.len(), 1);
        assert_eq!((owned[0].x, owned[0].y, owned[0].z), (-0.6, -0.6, -0.6));
        assert_eq!(map.nearby_points_iter(&q, 2.0).count(), 1);
    }

    #[test]
    fn voxel_key_pack_layout_matches_bias_offset_encoding() {
        const BIAS: u64 = 1 << 20;

        assert_eq!(
            key_at(0.0, 0.0, 0.0).pack(),
            (BIAS << 42) | (BIAS << 21) | BIAS
        );
        assert_eq!(
            key_at(-1.0, -1.0, -1.0).pack(),
            ((BIAS - 1) << 42) | ((BIAS - 1) << 21) | (BIAS - 1)
        );
    }

    #[test]
    fn voxel_key_pack_unpack_round_trips_across_sign_boundaries() {
        const MIN: i32 = -(1 << 20);
        const MAX: i32 = (1 << 20) - 1;
        let axis_values = [MIN, MIN + 1, -1, 0, 1, MAX - 1, MAX];

        let mut packed = HashSet::new();
        for &x in &axis_values {
            for &y in &axis_values {
                for &z in &axis_values {
                    let key = key_at(x as f32, y as f32, z as f32);
                    let raw = key.pack();
                    assert_eq!(VoxelKey::unpack(raw).pack(), raw);
                    packed.insert(raw);
                }
            }
        }
        assert_eq!(
            packed.len(),
            axis_values.len() * axis_values.len() * axis_values.len()
        );
    }

    #[test]
    fn crop_around_keeps_voxels_by_center_not_stored_points() {
        let mut map = VoxelMap::new(1.0);
        map.insert([pt(0.25, 0.25, 0.25), pt(0.1, 0.2, 0.3)])
            .unwrap();
        // Stored point lies inside the crop sphere, but its voxel center does not.
        map.insert([pt(1.05, 0.5, 0.5)]).unwrap();
        assert_eq!(map.len(), 2);

        let center = Vec3::new(0.5f32, 0.5, 0.5);
        let removed = map.crop_around(&center, 0.75);

        assert_eq!(removed, 1);
        assert_eq!(map.len(), 1);
        let kept = map.nearby_points(&center, 100.0);
        assert_eq!(kept.len(), 2);
        assert!(kept.iter().all(|p| p.x <= 0.3));
    }

    #[test]
    fn invalid_voxel_size_is_rejected_on_key_creation_and_insert() {
        let point = pt(1.0, 2.0, 3.0);
        for bad in [0.0, -1.0, f32::NAN, f32::INFINITY, f32::from_bits(1)] {
            assert!(VoxelKey::new(&point, bad).is_err());

            let mut map = VoxelMap::new(bad);
            assert!(map.insert([point.clone()]).is_err());
            assert!(map.is_empty());
        }
    }

    #[test]
    fn point_count_counts_points_across_shared_voxels() {
        let mut map = VoxelMap::new(1.0);
        assert_eq!(map.point_count(), 0);

        map.insert([pt(0.1, 0.1, 0.1), pt(0.2, 0.3, 0.4), pt(0.9, 0.9, 0.9)])
            .unwrap();
        assert_eq!(map.len(), 1);
        assert_eq!(map.point_count(), 3);

        map.insert([pt(-5.5, 0.0, 0.0), pt(-5.7, 0.1, 0.0)])
            .unwrap();
        assert_eq!(map.len(), 2);
        assert_eq!(map.point_count(), 5);
    }

    #[test]
    fn crop_around_on_empty_map_is_noop() {
        let mut map = VoxelMap::new(1.0);
        let center = Vec3::new(0.0f32, 0.0, 0.0);

        assert_eq!(map.crop_around(&center, 10.0), 0);
        assert_eq!(map.crop_around(&center, 0.0), 0);
        assert!(map.is_empty());
    }

    #[test]
    fn crop_around_single_voxel_boundary_is_inclusive() {
        let mut map = VoxelMap::new(1.0);
        map.insert([pt(0.25, 0.25, 0.25)]).unwrap();
        let voxel_center = Vec3::new(0.5f32, 0.5, 0.5);

        assert_eq!(map.crop_around(&voxel_center, 0.0), 0);
        assert_eq!(map.len(), 1);

        let far_center = Vec3::new(2.5f32, 0.5, 0.5);
        assert_eq!(map.crop_around(&far_center, 2.0), 0);
        assert_eq!(map.len(), 1);

        assert_eq!(map.crop_around(&far_center, 1.999), 1);
        assert!(map.is_empty());
    }

    #[test]
    fn insert_skips_non_finite_points_and_keeps_valid_ones() {
        let mut map = VoxelMap::new(1.0);
        map.insert([pt(1.0, 1.0, 1.0)]).unwrap();
        let voxels_before = map.len();
        let points_before = map.point_count();

        let bad_intensity = PointXYZI {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            intensity: f32::NAN,
        };
        let invalid_cases = [
            pt(f32::NAN, 0.0, 0.0),
            pt(f32::INFINITY, 0.0, 0.0),
            pt(0.0, f32::NEG_INFINITY, 0.0),
            pt(0.0, 0.0, f32::NAN),
            bad_intensity,
        ];

        map.insert(invalid_cases).unwrap();
        assert_eq!(map.len(), voxels_before);
        assert_eq!(map.point_count(), points_before);

        map.insert([pt(4.0, 4.0, 4.0)]).unwrap();
        assert_eq!(map.len(), voxels_before + 1);
        assert_eq!(map.point_count(), points_before + 1);

        map.insert([pt(2.0, 2.0, 2.0), pt(f32::NAN, 5.0, 5.0)])
            .unwrap();
        assert_eq!(map.len(), voxels_before + 2);
        assert_eq!(map.point_count(), points_before + 2);
    }

    fn pseudo_random_points(n: usize) -> Vec<PointXYZI> {
        let mut s = 0x1234_5678_u32;
        (0..n)
            .map(|_| {
                let mut axis = || {
                    s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    (s >> 8) as f32 / (1 << 24) as f32 * 20.0 - 10.0
                };
                pt(axis(), axis(), axis())
            })
            .collect()
    }

    fn nearest_tuples(result: &[NearestPoint<'_>]) -> Vec<(f32, [f32; 4])> {
        result
            .iter()
            .map(|np| {
                (
                    np.squared_distance,
                    [np.point.x, np.point.y, np.point.z, np.point.intensity],
                )
            })
            .collect()
    }

    fn bruteforce_nearest(
        map: &VoxelMap,
        q: &Vec3<f32>,
        radius: f32,
        count: usize,
    ) -> Vec<(f32, [f32; 4])> {
        let mut candidates: Vec<(f32, [f32; 4])> = map
            .nearby_points_iter(q, radius)
            .map(|p| {
                (
                    (p.to_vec3() - *q).norm_squared(),
                    [p.x, p.y, p.z, p.intensity],
                )
            })
            .collect();
        candidates.sort_by(|a, b| a.0.total_cmp(&b.0));
        candidates.truncate(count);
        candidates
    }

    #[test]
    fn nearest_n_returns_points_sorted_by_distance() {
        let mut map = VoxelMap::new(1.0);
        map.insert([
            pt(3.0, 0.0, 0.0),
            pt(0.0, 2.0, 0.0),
            pt(1.5, 1.5, 1.5),
            pt(-1.0, -1.0, -1.0),
            pt(0.5, 0.5, 0.0),
            pt(0.1, 0.1, 0.1),
            pt(0.1, 0.1, 0.1),
        ])
        .unwrap();
        let q = Vec3::new(0.0f32, 0.0, 0.0);

        // 候选数 > count：走 select_nth 截断路径
        let top3 = map.nearest_n_sorted(&q, 10.0, 3);
        assert_eq!(top3.len(), 3);
        let dump = format!("{:?}", nearest_tuples(&top3));
        for w in top3.windows(2) {
            assert!(
                w[0].squared_distance <= w[1].squared_distance,
                "result not sorted: {dump}"
            );
        }
        // 最近的是重复点对 (0.1,0.1,0.1)，第三近的是 (0.5,0.5,0)
        assert_eq!(top3[0].point.x, 0.1);
        assert_eq!(top3[1].point.x, 0.1);
        assert_eq!(top3[2].point.x, 0.5);

        // 候选数 <= count：不截断也必须有序
        let all = map.nearest_n_sorted(&q, 10.0, 100);
        assert_eq!(all.len(), 7);
        let dump = format!("{:?}", nearest_tuples(&all));
        for w in all.windows(2) {
            assert!(
                w[0].squared_distance <= w[1].squared_distance,
                "result not sorted: {dump}"
            );
        }
        assert_eq!(all[0].point.x, 0.1);
        assert_eq!(all[6].point.x, 3.0);
    }

    #[test]
    fn nearest_n_respects_radius() {
        let mut map = VoxelMap::new(1.0);
        map.insert([
            pt(1.0, 0.0, 0.0),
            pt(2.0, 0.0, 0.0),
            pt(1.9, 0.57, 0.0),
            pt(1.99, 0.5, 0.0),
            pt(5.0, 0.0, 0.0),
        ])
        .unwrap();
        let q = Vec3::new(0.0f32, 0.0, 0.0);

        let result = map.nearest_n(&q, 2.0, 10);
        assert_eq!(result.len(), 3);
        assert!(result.iter().all(|np| np.squared_distance <= 4.0));
        // (2,0,0) 距离恰为半径，必须包含
        assert!(result.iter().any(|np| np.point.x == 2.0));
        // (1.99,0.5,0) 与 (1.9,0.57,0) 同在被扫描的 voxel 内，
        // 但前者在球外，必须被真实距离过滤掉
        let xs: Vec<f32> = result.iter().map(|np| np.point.x).collect();
        assert!(!xs.contains(&1.99));
        assert!(!xs.contains(&5.0));
    }

    #[test]
    fn nearest_n_respects_count() {
        let mut map = VoxelMap::new(1.0);
        map.insert([
            pt(1.0, 0.0, 0.0),
            pt(2.0, 0.0, 0.0),
            pt(3.0, 0.0, 0.0),
            pt(4.0, 0.0, 0.0),
            pt(5.0, 0.0, 0.0),
        ])
        .unwrap();
        let q = Vec3::new(0.0f32, 0.0, 0.0);

        let top3 = map.nearest_n(&q, 10.0, 3);
        assert_eq!(top3.len(), 3);
        let mut distances: Vec<f32> = top3.iter().map(|np| np.squared_distance).collect();
        distances.sort_by(f32::total_cmp);
        assert_eq!(distances, [1.0, 4.0, 9.0]);

        let all = map.nearest_n(&q, 10.0, 10);
        assert_eq!(all.len(), 5);

        let capped_by_radius = map.nearest_n(&q, 1.5, 10);
        assert_eq!(capped_by_radius.len(), 1);
    }

    #[test]
    fn nearest_n_zero_count_and_empty_result_return_empty() {
        let mut map = VoxelMap::new(1.0);
        map.insert([pt(1.0, 1.0, 1.0)]).unwrap();
        let q = Vec3::new(0.0f32, 0.0, 0.0);

        assert!(map.nearest_n(&q, 5.0, 0).is_empty());

        let empty_map = VoxelMap::new(1.0);
        assert!(empty_map.nearest_n(&q, 5.0, 4).is_empty());

        assert!(map.nearest_n(&q, 0.1, 4).is_empty());
    }

    #[test]
    fn nearest_n_returns_borrowed_points_without_cloning() {
        let mut map = VoxelMap::new(1.0);
        map.insert(pseudo_random_points(16)).unwrap();
        let q = Vec3::new(0.0f32, 0.0, 0.0);

        let stored: HashSet<*const PointXYZI> = map
            .nearby_points_iter(&q, 30.0)
            .map(|p| p as *const PointXYZI)
            .collect();
        assert_eq!(stored.len(), 16);

        for np in map.nearest_n(&q, 30.0, 5) {
            let addr = np.point as *const PointXYZI;
            assert!(
                stored.contains(&addr),
                "returned point is not borrowed from map storage"
            );
        }
    }

    #[test]
    fn nearest_n_matches_bruteforce_reference() {
        let mut map = VoxelMap::new(1.0);
        map.insert(pseudo_random_points(48)).unwrap();

        let cases = [
            (Vec3::new(1.234f32, -2.345, 3.456), 7.89f32, 1usize),
            (Vec3::new(1.234, -2.345, 3.456), 7.89, 5),
            (Vec3::new(1.234, -2.345, 3.456), 7.89, 100),
            (Vec3::new(0.0, 0.0, 0.0), 0.5, 10),
            (Vec3::new(-5.5, -5.5, -5.5), 15.0, 7),
        ];

        for (q, r, k) in cases {
            // 此处只验证“选出的集合正确”，排序正确性由专用测试负责
            let mut actual = nearest_tuples(&map.nearest_n(&q, r, k));
            actual.sort_by(|a, b| a.0.total_cmp(&b.0));
            assert_eq!(
                actual,
                bruteforce_nearest(&map, &q, r, k),
                "query ({}, {}, {}) radius {} count {}",
                q.x,
                q.y,
                q.z,
                r,
                k
            );
        }
    }
}

#[cfg(test)]
mod surfel_test {
    use crate::VoxelKey;
    use crate::surfel::SurfelMap;
    use crate::types::{GeometryClass, Surfel};
    use fastlio_types::{Mat3, PointXYZI, SurfelConfig, SurfelMapConfig, Vec3};
    use smallvec::SmallVec;

    fn pt(x: f32, y: f32, z: f32) -> PointXYZI {
        PointXYZI {
            x,
            y,
            z,
            intensity: 0.0,
        }
    }

    fn map_config() -> SurfelMapConfig {
        SurfelMapConfig {
            voxel_size: 1.0,
            search_radius: 4,
        }
    }

    #[allow(clippy::field_reassign_with_default)]
    fn surfel_config(f: impl FnOnce(&mut SurfelConfig)) -> SurfelConfig {
        let mut c = SurfelConfig::default();
        c.min_mature_surfel_count = 8;
        f(&mut c);
        c
    }

    fn only_surfel(map: &SurfelMap) -> &Surfel {
        assert_eq!(map.surfels.len(), 1, "expected exactly one surfel");
        map.surfels.values().next().expect("one surfel expected")
    }

    #[test]
    fn growing_surfel_accepts_nearby_point() {
        let mut map = SurfelMap::new(map_config(), surfel_config(|c| c.growing_radius = 0.5));
        map.insert([pt(0.0, 0.0, 0.0), pt(0.1, 0.0, 0.0), pt(0.0, 0.1, 0.0)].into_iter())
            .unwrap();
        assert_eq!(map.surfels.len(), 1);
        assert_eq!(only_surfel(&map).count, 3);

        map.insert([pt(0.05, 0.05, 0.0)].into_iter()).unwrap();
        assert_eq!(map.surfels.len(), 1, "nearby point must join, not split");
        assert_eq!(only_surfel(&map).count, 4);
    }

    #[test]
    fn surfel_processes_all_inserted_points() {
        let mut map = SurfelMap::new(map_config(), surfel_config(|c| c.growing_radius = 2.0));
        let points = [
            pt(0.0, 0.0, 0.0),
            pt(0.2, 0.0, 0.0),
            pt(0.0, 0.2, 0.0),
            pt(0.1, 0.1, 0.0),
            pt(-0.1, 0.0, 0.1),
            pt(0.0, -0.1, -0.1),
        ];
        let n = points.len();
        map.insert(points.into_iter()).unwrap();

        let total: usize = map.surfels.values().map(|s| s.count).sum();
        assert_eq!(total, n, "every inserted point must be processed");
        let s = only_surfel(&map);
        assert_eq!(s.count, n, "points should cluster into a single surfel");
    }

    #[test]
    fn surfel_reaches_mature_state() {
        let mut map = SurfelMap::new(map_config(), surfel_config(|c| c.growing_radius = 2.0));
        let pts = [
            pt(0.0, 0.0, 0.0),
            pt(0.2, 0.0, 0.0),
            pt(0.0, 0.2, 0.0),
            pt(0.2, 0.2, 0.0),
            pt(0.1, 0.0, 0.0),
            pt(0.0, 0.1, 0.0),
            pt(0.3, 0.1, 0.0),
            pt(0.2, 0.3, 0.0),
        ];
        map.insert(pts.into_iter()).unwrap();
        let s = only_surfel(&map);
        assert_eq!(s.count, 8);
        assert!(!matches!(
            s.geometry_class(map.surfel_config()),
            GeometryClass::Growing
        ));
    }

    #[test]
    fn planar_surfel_is_classified_as_plane() {
        let mut map = SurfelMap::new(map_config(), surfel_config(|c| c.growing_radius = 3.0));
        map.insert(
            [
                pt(-1.0, -1.0, 0.0),
                pt(1.0, -1.0, 0.0),
                pt(-1.0, 1.0, 0.0),
                pt(1.0, 1.0, 0.0),
                pt(0.0, -1.0, 0.0),
                pt(0.0, 1.0, 0.0),
                pt(-1.0, 0.0, 0.0),
                pt(1.0, 0.0, 0.0),
            ]
            .into_iter(),
        )
        .unwrap();
        assert!(matches!(
            only_surfel(&map).geometry_class(map.surfel_config()),
            GeometryClass::Plane
        ));
    }

    #[test]
    fn linear_surfel_is_classified_as_line() {
        let mut map = SurfelMap::new(map_config(), surfel_config(|c| c.growing_radius = 3.5));
        map.insert(
            [
                pt(-1.5, 0.1, 0.06),
                pt(-1.0, -0.1, 0.06),
                pt(-0.5, 0.1, 0.06),
                pt(0.0, -0.1, -0.06),
                pt(0.5, 0.1, -0.06),
                pt(1.0, -0.1, -0.06),
                pt(1.5, 0.1, 0.06),
                pt(0.0, -0.1, -0.06),
            ]
            .into_iter(),
        )
        .unwrap();
        assert!(matches!(
            only_surfel(&map).geometry_class(map.surfel_config()),
            GeometryClass::Line
        ));
    }

    #[test]
    fn scattered_surfel_is_classified_as_scatter() {
        let mut map = SurfelMap::new(map_config(), surfel_config(|c| c.growing_radius = 3.5));
        map.insert(
            [
                pt(-1.2, -1.0, -0.8),
                pt(1.0, -1.0, 0.7),
                pt(0.3, 1.1, -0.9),
                pt(-0.8, 0.9, 1.0),
                pt(1.1, 0.2, -1.0),
                pt(-0.5, -1.1, 0.8),
                pt(0.9, 0.8, 0.9),
                pt(-1.0, 0.4, -0.2),
            ]
            .into_iter(),
        )
        .unwrap();
        assert!(matches!(
            only_surfel(&map).geometry_class(map.surfel_config()),
            GeometryClass::Scatter
        ));
    }

    #[test]
    fn incremental_covariance_matches_batch_covariance() {
        let mut map = SurfelMap::new(map_config(), surfel_config(|c| c.growing_radius = 3.0));
        let points = [
            pt(1.0, 0.0, 0.1),
            pt(1.2, 0.3, -0.1),
            pt(0.8, 0.1, 0.2),
            pt(0.9, -0.2, 0.0),
            pt(1.1, 0.2, 0.1),
            pt(1.0, 0.4, -0.2),
        ];

        for p in &points {
            map.insert(std::iter::once(p.clone())).unwrap();
        }
        let s = only_surfel(&map);
        assert_eq!(s.count, points.len());

        let mut mean = Vec3::<f32>::zeros();
        for p in &points {
            mean += p.to_vec3();
        }
        mean /= points.len() as f32;
        let mut batch = Mat3::<f32>::zeros();
        for p in &points {
            let d = p.to_vec3() - mean;
            batch += d * d.transpose();
        }
        let batch_cov = batch / (points.len() - 1) as f32;
        let inc_cov = s.m2 / (s.count - 1) as f64;

        for i in 0..3 {
            assert!(
                (s.mean_w[i] - mean[i] as f64).abs() < 1e-4,
                "mean mismatch at {i}: {} vs {}",
                s.mean_w[i],
                mean[i]
            );
            for j in 0..3 {
                assert!(
                    (inc_cov[(i, j)] - batch_cov[(i, j)] as f64).abs() < 1e-3,
                    "covariance mismatch at [{i}][{j}]: {} vs {}",
                    inc_cov[(i, j)],
                    batch_cov[(i, j)]
                );
            }
        }
    }

    /// Build an 8-point patch on the plane through `center` with `normal`,
    /// spanning roughly `[-1, 1]` in two tangent directions.
    fn plane_cluster(center: Vec3<f32>, normal: Vec3<f32>) -> Vec<PointXYZI> {
        let n = normal.normalize();
        let mut t1 = Vec3::new(1.0, 0.0, 0.0);
        if t1.dot(&n).abs() > 0.9 {
            t1 = Vec3::new(0.0, 1.0, 0.0);
        }
        let t1 = (t1 - t1.dot(&n) * n).normalize();
        let t2 = n.cross(&t1);
        let grid = [
            (-1.0, -1.0),
            (1.0, -1.0),
            (-1.0, 1.0),
            (1.0, 1.0),
            (0.0, -1.0),
            (0.0, 1.0),
            (-1.0, 0.0),
            (1.0, 0.0),
        ];
        grid.into_iter()
            .map(|(u, v)| {
                let p = center + t1 * u + t2 * v;
                pt(p.x, p.y, p.z)
            })
            .collect()
    }

    fn assert_none(map: &SurfelMap, point: &PointXYZI) {
        assert!(
            map.query(point).expect("query must not error").is_none(),
            "expected query to return None for point ({}, {}, {})",
            point.x,
            point.y,
            point.z
        );
    }

    #[test]
    fn query_returns_planar_surfel() {
        let mut map = SurfelMap::new(map_config(), surfel_config(|c| c.growing_radius = 3.5));
        let pts = plane_cluster(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
        map.insert(pts.into_iter()).unwrap();

        let obs = map
            .query(&pt(0.0, 0.0, 0.0))
            .expect("query must not error")
            .expect("planar surfel expected");
        assert!(obs.norm_w.norm().is_finite(), "normal must be finite");
        assert!(
            (obs.norm_w.norm() - 1.0).abs() < 1e-3,
            "normal must be unit"
        );
        assert!(obs.plane_distance < 1e-3);
    }

    #[test]
    fn query_returns_normal_and_mean_in_world_frame() {
        let normal = Vec3::new(1.0f32, 2.0, 2.0);
        let mut map = SurfelMap::new(map_config(), surfel_config(|c| c.growing_radius = 3.5));
        let pts = plane_cluster(Vec3::new(0.0, 0.0, 0.0), normal);
        map.insert(pts.iter().cloned()).unwrap();

        // Query a point that lies on the plane: pick a stored sample point.
        let sample = &pts[0];
        let obs = map
            .query(sample)
            .expect("query must not error")
            .expect("planar surfel expected");

        // Mean sits on the plane and matches the world-frame centroid.
        assert!(obs.mean_w.norm() < 1e-2, "mean should be near origin");
        // Normal is parallel (up to sign) to the plane normal.
        let n = obs.norm_w;
        assert!(
            n.cross(&normal.cast::<f64>()).norm() < 1e-2,
            "normal {n:?} not parallel to {}",
            normal
        );
        // Normal is perpendicular to every in-plane offset from the mean.
        for stored in &pts {
            let delta = stored.to_vec3().cast::<f64>() - obs.mean_w;
            assert!(
                n.dot(&delta).abs() < 1e-2,
                "normal not perpendicular to plane at stored point"
            );
        }
        // plane_distance is the perpendicular distance from query to the plane.
        let d = (sample.to_vec3().cast::<f64>() - obs.mean_w).dot(&n).abs();
        assert!((obs.plane_distance - d).abs() < 1e-3);
    }

    #[test]
    fn query_rejects_growing_surfel() {
        let mut map = SurfelMap::new(map_config(), surfel_config(|c| c.growing_radius = 2.0));
        // Too few points to mature: a partial coplanar set stays Growing.
        map.insert(
            [
                pt(0.0, 0.0, 0.0),
                pt(0.2, 0.0, 0.0),
                pt(0.0, 0.2, 0.0),
                pt(0.2, 0.2, 0.0),
            ]
            .into_iter(),
        )
        .unwrap();
        assert_eq!(only_surfel(&map).count, 4);
        assert_none(&map, &pt(0.1, 0.1, 0.0));
    }

    #[test]
    fn query_rejects_line_surfel() {
        let mut map = SurfelMap::new(map_config(), surfel_config(|c| c.growing_radius = 3.5));
        map.insert(
            [
                pt(-1.5, 0.1, 0.06),
                pt(-1.0, -0.1, 0.06),
                pt(-0.5, 0.1, 0.06),
                pt(0.0, -0.1, -0.06),
                pt(0.5, 0.1, -0.06),
                pt(1.0, -0.1, -0.06),
                pt(1.5, 0.1, 0.06),
                pt(0.0, -0.1, -0.06),
            ]
            .into_iter(),
        )
        .unwrap();
        assert!(matches!(
            only_surfel(&map).geometry_class(map.surfel_config()),
            GeometryClass::Line
        ));
        assert_none(&map, &pt(0.0, 0.0, 0.0));
    }

    #[test]
    fn query_rejects_scatter_surfel() {
        let mut map = SurfelMap::new(map_config(), surfel_config(|c| c.growing_radius = 3.5));
        map.insert(
            [
                pt(-1.2, -1.0, -0.8),
                pt(1.0, -1.0, 0.7),
                pt(0.3, 1.1, -0.9),
                pt(-0.8, 0.9, 1.0),
                pt(1.1, 0.2, -1.0),
                pt(-0.5, -1.1, 0.8),
                pt(0.9, 0.8, 0.9),
                pt(-1.0, 0.4, -0.2),
            ]
            .into_iter(),
        )
        .unwrap();
        assert!(matches!(
            only_surfel(&map).geometry_class(map.surfel_config()),
            GeometryClass::Scatter
        ));
        assert_none(&map, &pt(0.0, 0.0, 0.0));
    }

    #[test]
    fn query_is_gated_by_three_dimensional_support() {
        let mut map = SurfelMap::new(
            map_config(),
            surfel_config(|c| {
                c.growing_radius = 3.5;
                // Loosened plane distance alone must NOT widen association:
                // the 3D Mahalanobis support is the binding constraint.
                c.max_plane_distance = 6.0;
            }),
        );
        let mut all = plane_cluster(Vec3::new(0.0f32, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
        all.extend(plane_cluster(
            Vec3::new(0.0f32, 0.0, 5.0),
            Vec3::new(0.0, 0.0, 1.0),
        ));
        map.insert(all.into_iter()).unwrap();
        assert_eq!(map.surfels.len(), 2, "two separated planes expected");

        // In-plane query point matches, and only the nearer plane qualifies
        // (the distant plane is outside both supports' Gaussian slab).
        let obs = map
            .query(&pt(0.0, 0.0, 0.0))
            .expect("query must not error")
            .expect("plane candidate expected");
        assert!(
            obs.mean_w.z.abs() < 0.5,
            "should match plane at z=0, got z={}",
            obs.mean_w.z
        );

        // 1 m off the plane: within max_plane_distance, but outside the
        // 3D support (a perfect plane has ~zero extent along its normal).
        assert_none(&map, &pt(0.0, 0.0, 1.0));
        assert_none(&map, &pt(0.0, 0.0, 2.5));

        // On the second plane: matches it instead.
        let obs = map
            .query(&pt(0.0, 0.0, 5.0))
            .expect("query must not error")
            .expect("plane candidate expected");
        assert!(
            (obs.mean_w.z - 5.0).abs() < 0.5,
            "should match plane at z=5, got z={}",
            obs.mean_w.z
        );
    }

    #[test]
    fn query_returns_none_for_invalid_point() {
        let mut map = SurfelMap::new(map_config(), surfel_config(|c| c.growing_radius = 3.5));
        map.insert(
            plane_cluster(Vec3::new(0.0f32, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)).into_iter(),
        )
        .unwrap();
        // sanity: valid point is accepted
        assert!(map.query(&pt(0.0, 0.0, 0.0)).unwrap().is_some());

        let bad_coords = [pt(f32::NAN, 0.0, 0.0), pt(0.0, f32::INFINITY, 0.0)];
        for p in &bad_coords {
            assert_none(&map, p);
        }
        let bad_intensity = PointXYZI {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            intensity: f32::NAN,
        };
        assert_none(&map, &bad_intensity);
        let neg_intensity = PointXYZI {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            intensity: -1.0,
        };
        assert_none(&map, &neg_intensity);
    }

    /// A surfel shaped like a perfect horizontal plane: degenerate along x,
    /// full spread along y and z, eigenvectors aligned with the axes.
    #[allow(clippy::field_reassign_with_default)]
    fn axis_plane_surfel(degenerate_eigenvalue: f32) -> Surfel {
        let mut s = Surfel::default();
        s.mean_w = Vec3::zeros();
        s.eigenvectors = Mat3::identity();
        s.eigenvalues = Vec3::new(degenerate_eigenvalue as f64, 1.0, 1.0);
        s.count = 8;
        s
    }

    #[test]
    fn within_support_gates_normal_deviation_in_three_dimensions() {
        let config = surfel_config(|_| {});
        // max_mahalanobis_distance = 2.0, so sigma^2 = 4.
        // tangent directions resolve as dx/1 + dy/1 <= 4.
        let s = axis_plane_surfel(0.0);

        // Tangent-only judgment (what a 2D version would do):
        assert!(s.within_support(&pt(0.0, 0.9, 1.5), &config));
        assert!(
            s.within_support(&pt(0.0, 2.0, 0.0), &config),
            "tangent boundary is inclusive"
        );
        assert!(!s.within_support(&pt(0.0, 2.1, 0.0), &config));

        // Same tangent position, but tiny deviation along the degenerate axis:
        // rejected because the plane has ~zero extent along its normal.
        assert!(!s.within_support(&pt(0.01, 0.9, 1.5), &config));
        assert!(!s.within_support(&pt(-0.01, 0.9, 1.5), &config));

        // Pure normal deviation: tolerance is bounded by the covariance floor.
        assert!(s.within_support(&pt(0.001, 0.0, 0.0), &config));
        assert!(!s.within_support(&pt(0.003, 0.0, 0.0), &config));
    }

    #[test]
    fn within_support_covariance_regularization_floor_boundary() {
        let config = surfel_config(|_| {});

        // Probe just past the floor-limited tolerance, at |normal offset|^2 = 6.25e-6.
        // With the default covariance floor active the Mahalanobis term is 6.25 > sigma^2 (4),
        // so the point is rejected; the boundary is exactly at offsets below 1e-3*sigma.
        let probe = pt(2.5e-3, 0.0, 0.0);

        // Degenerate eigenvalue below/at the floor: identical clamped behavior.
        for degenerate in [0.0f32, 5.0e-7, 1.0e-6] {
            let s = axis_plane_surfel(degenerate);
            assert!(
                !s.within_support(&probe, &config),
                "degenerate eigenvalue {degenerate} must be clamped to the configured floor"
            );
        }

        // Just above the floor the real eigenvalue is used: 6.25e-6 / 2e-6 = 3.125 < 4.
        let s = axis_plane_surfel(2.0e-6);
        assert!(s.within_support(&probe, &config));

        // Bracket the floor-limited tolerance itself: sqrt(sigma^2 * floor) ~= 2e-3.
        let floor = axis_plane_surfel(0.0);
        assert!(floor.within_support(&pt(1.9e-3, 0.0, 0.0), &config));
        assert!(!floor.within_support(&pt(2.1e-3, 0.0, 0.0), &config));
    }

    #[test]
    #[allow(clippy::field_reassign_with_default)]
    fn within_support_nonzero_normal_eigenvalue_and_rotation() {
        let config = surfel_config(|_| {});

        // 90 deg rotation about z. Columns of R:
        //   col0 = normal  axis, ev = 0.25 -> (0, 1, 0)  (world +y)
        //   col1 = tangent axis, ev = 1.0  -> (-1, 0, 0) (world -x)
        //   col2 = tangent axis, ev = 1.0  -> (0, 0, 1)
        // Eigen-basis coords: y = R^T * delta = (delta.y, -delta.x, delta.z).
        let mut surfel = Surfel::default();
        surfel.mean_w = Vec3::zeros();
        surfel.eigenvectors = Mat3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        surfel.eigenvalues = Vec3::new(0.25, 1.0, 1.0);
        surfel.count = 8;

        // Along the rotated tangent axis (world x): boundary at sqrt(4 * 1) = 2.0.
        assert!(!surfel.within_support(&pt(2.1, 0.0, 0.0), &config));
        assert!(
            surfel.within_support(&pt(2.0, 0.0, 0.0), &config),
            "tangent boundary inclusive"
        );
        assert!(surfel.within_support(&pt(0.5, 0.0, 0.0), &config));

        // Along the rotated normal axis (world +y): boundary at sqrt(4 * 0.25) = 1.0.
        // A world-axis-based implementation would treat +y as a tangent axis
        // (allowing up to 2.0) and accept 1.5 here: must be rejected.
        assert!(
            surfel.within_support(&pt(0.0, 1.0, 0.0), &config),
            "normal boundary inclusive"
        );
        assert!(!surfel.within_support(&pt(0.0, 1.2, 0.0), &config));
        assert!(!surfel.within_support(&pt(0.0, 1.5, 0.0), &config));

        // Diagonal mix of both axes: sum of two Mahalanobis terms, not min/max.
        assert!(surfel.within_support(&pt(0.8, 0.8, 0.0), &config)); // 0.64/0.25 + 0.64 = 3.2
        assert!(!surfel.within_support(&pt(1.0, 1.0, 0.0), &config)); // 4 + 1 = 5

        // Rotation covariance: within_support(R * p) == within_support_identity(p).
        let identity = axis_plane_surfel(0.25);
        for p in [[0.5f32, 0.0, 0.0], [0.0, 2.0, 0.0]] {
            let (x, y, z) = (p[0], p[1], p[2]);
            let rotated = pt(-y, x, z); // R * p for a +90 deg z-rotation
            assert_eq!(
                surfel.within_support(&rotated, &config),
                identity.within_support(&pt(x, y, z), &config)
            );
        }
    }

    #[test]
    #[allow(clippy::field_reassign_with_default)]
    fn support_aabb_extent_covers_three_dimensional_support() {
        let config = surfel_config(|_| {});
        let sigma = config.max_mahalanobis_distance;
        let rot90 = Mat3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);

        // (eigenvectors, eigenvalues) with/without a real normal eigenvalue,
        // axis-aligned or rotated.
        let cases = [
            (Mat3::identity(), Vec3::new(0.0, 1.0, 1.0)),
            (rot90, Vec3::new(0.0, 1.0, 1.0)),
            (Mat3::identity(), Vec3::new(0.25, 1.0, 1.0)),
            (rot90, Vec3::new(0.25, 1.0, 1.0)),
        ];

        for (eigenvectors, eigenvalues) in cases {
            let mut s = Surfel::default();
            s.mean_w = Vec3::zeros();
            s.eigenvectors = eigenvectors;
            s.eigenvalues = eigenvalues;
            s.count = 8;

            let extent = s.support_aabb_extent(&config);
            for i in 0..3 {
                // Boundary point along eigen-direction i: within_support is
                // inclusive at radius sqrt(sigma^2 * ev[i]).
                let p =
                    s.mean_w + s.eigenvectors.column(i) * (sigma as f64 * eigenvalues[i].sqrt());
                let point = PointXYZI {
                    x: p.x as f32,
                    y: p.y as f32,
                    z: p.z as f32,
                    intensity: 0.0,
                };
                assert!(
                    s.within_support(&point, &config),
                    "case ev0={} rot={}: boundary point along eigen {i} must be in support",
                    eigenvalues[0],
                    !(eigenvectors == Mat3::identity())
                );
                assert!(
                    p.x.abs() <= extent.x && p.y.abs() <= extent.y && p.z.abs() <= extent.z,
                    "case ev0={} rot={}: AABB extent {extent:?} misses eigen-direction {i} \
                     boundary point {p:?}",
                    eigenvalues[0],
                    !(eigenvectors == Mat3::identity())
                );
            }
        }
    }

    /// Mirror of `reindex_surfel`'s extent→voxel-key logic for test assertions.
    fn expected_indexed_keys(
        surfel: &Surfel,
        voxel_size: f32,
        surfel_config: &SurfelConfig,
    ) -> SmallVec<[u64; 8]> {
        let extent = if surfel.is_growing(surfel_config.min_mature_surfel_count) {
            Vec3::repeat(surfel_config.growing_radius as f64)
        } else {
            surfel.support_aabb_extent(surfel_config)
        };
        let min_w = surfel.mean_w - extent;
        let max_w = surfel.mean_w + extent;
        let min_key = VoxelKey::from_vec3(min_w.cast(), voxel_size).unwrap();
        let max_key = VoxelKey::from_vec3(max_w.cast(), voxel_size).unwrap();
        let mut keys = SmallVec::new();
        for x in min_key.x..=max_key.x {
            for y in min_key.y..=max_key.y {
                for z in min_key.z..=max_key.z {
                    keys.push(VoxelKey { x, y, z }.pack());
                }
            }
        }
        keys
    }

    /// Run all index-consistency invariants on the current map state.
    fn assert_index_consistency(map: &SurfelMap, surfel_ids: &[crate::surfel::SurfelID]) {
        let voxel_size = map.surfel_map_config().voxel_size;
        let sc = map.surfel_config();

        for &id in surfel_ids {
            let s = map.surfels.get(id).unwrap();
            let expected = expected_indexed_keys(s, voxel_size, sc);

            // (a) indexed_voxels matches the AABB-derived expected keys exactly.
            let mut got = s.indexed_voxels.clone();
            got.sort_unstable();
            let mut exp = expected.clone();
            exp.sort_unstable();
            assert_eq!(
                got, exp,
                "indexed_voxels mismatch for surfel {id:?}: got {got:?}, expected {exp:?}"
            );

            // (b) Every key in indexed_voxels exists in buckets and contains this surfel.
            for &key in s.indexed_voxels.iter() {
                let bucket = map
                    .buckets
                    .get(&key)
                    .unwrap_or_else(|| panic!("bucket for key {key} must exist"));
                assert!(
                    bucket.contains(&id),
                    "bucket for key {key} must contain surfel {id:?}"
                );
            }
        }

        // (c) Every bucket entry references a valid surfel.
        for (key, bucket) in map.buckets.iter() {
            for sid in bucket {
                assert!(
                    map.surfels.get(*sid).is_some(),
                    "bucket {key} references invalid surfel {sid:?}"
                );
            }
        }

        // (d) Total bucket entries == sum of indexed_voxels lengths.
        let total_entries: usize = map.buckets.values().map(|b| b.len()).sum();
        let total_indexed: usize = map.surfels.values().map(|s| s.indexed_voxels.len()).sum();
        assert_eq!(
            total_entries, total_indexed,
            "bucket entry count must equal sum of indexed_voxels lengths"
        );

        // (e) No surfel appears twice in the same bucket.
        for (key, bucket) in map.buckets.iter() {
            let mut ids: Vec<_> = bucket.iter().copied().collect();
            ids.sort();
            ids.dedup();
            assert_eq!(
                ids.len(),
                bucket.len(),
                "bucket {key} contains duplicate surfel IDs: {bucket:?}"
            );
        }
    }

    #[test]
    fn surfel_incremental_insert_keeps_index_consistent() {
        let mut map = SurfelMap::new(map_config(), surfel_config(|c| c.growing_radius = 3.5));

        // ============================================================
        // Phase 1: build a planar surfel and two growing neighbours.
        //
        // refit timing (min_mature=8, helper_interval: 0..=16→4):
        //   point 1  → create_surfel → reindex (last_refit=0)
        //   point 8  → count=8, 8>=8 && 8-0=8>4 → refit (last_refit=8)
        //
        // Use a wide planar patch so the AABB is large enough for
        // subsequent points to fall within Mahalanobis support.
        // ============================================================
        let surfel0: crate::surfel::SurfelID;
        {
            let pts = plane_cluster(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
            map.insert(pts.into_iter()).unwrap();
            surfel0 = map.surfels.keys().next().unwrap();
        }

        {
            let base = pt(5.0, 0.0, 0.0);
            let pts: Vec<PointXYZI> = (0..4)
                .map(|i| pt(base.x + i as f32 * 0.1, base.y, base.z))
                .collect();
            map.insert(pts.into_iter()).unwrap();
        }

        {
            let base = pt(0.0, 5.0, 0.0);
            let pts: Vec<PointXYZI> = (0..4)
                .map(|i| pt(base.x, base.y + i as f32 * 0.1, base.z))
                .collect();
            map.insert(pts.into_iter()).unwrap();
        }

        let ids: Vec<_> = map.surfels.keys().collect();
        assert_eq!(ids.len(), 3);
        assert_eq!(map.surfels.get(surfel0).unwrap().count, 8);
        assert!(!map.surfels.get(surfel0).unwrap().indexed_voxels.is_empty());
        assert_index_consistency(&map, &ids);

        // ============================================================
        // Phase 2: second refit at count 13  (13-8=5 > helper(13)=4).
        // Insert 5 points within surfel0's Mahalanobis support so they
        // are absorbed and trigger a refit with updated eigenvalues.
        // ============================================================
        let pre_refit_keys = map.surfels.get(surfel0).unwrap().indexed_voxels.clone();

        {
            let extra = [
                pt(1.5, 0.0, 0.0),
                pt(-1.5, 0.0, 0.0),
                pt(0.0, 1.5, 0.0),
                pt(0.0, -1.5, 0.0),
                pt(1.2, 1.2, 0.0),
            ];
            map.insert(extra.into_iter()).unwrap();
        }

        let s0 = map.surfels.get(surfel0).unwrap();
        assert_eq!(s0.count, 13);
        // The refit should have updated indexed_voxels.
        assert_ne!(
            s0.indexed_voxels, pre_refit_keys,
            "second refit must replace old indexed_voxels"
        );
        assert_index_consistency(&map, &ids);

        // Old keys that are no longer in indexed_voxels must not reference
        // this surfel in their buckets.
        let old_only: SmallVec<[u64; 8]> = pre_refit_keys
            .iter()
            .filter(|k| !s0.indexed_voxels.contains(k))
            .copied()
            .collect();
        for key in &old_only {
            if let Some(bucket) = map.buckets.get(key) {
                assert!(
                    !bucket.contains(&surfel0),
                    "old key {key} still references surfel {surfel0:?} after refit"
                );
            }
        }

        // ============================================================
        // Phase 3: third refit at count 22  (22-13=9 > helper(22)=4).
        // Insert 9 more points within support.
        // ============================================================
        let pre_refit_keys = s0.indexed_voxels.clone();

        {
            let extra = [
                pt(1.0, -1.3, 0.0),
                pt(-1.0, 1.3, 0.0),
                pt(1.3, 0.5, 0.0),
                pt(-1.3, -0.5, 0.0),
                pt(0.5, 1.4, 0.0),
                pt(-0.5, -1.4, 0.0),
                pt(0.7, -0.9, 0.0),
                pt(-0.7, 0.9, 0.0),
                pt(0.0, 0.0, 0.0),
            ];
            map.insert(extra.into_iter()).unwrap();
        }

        let s0 = map.surfels.get(surfel0).unwrap();
        assert_eq!(s0.count, 22);
        assert_ne!(
            s0.indexed_voxels, pre_refit_keys,
            "third refit must replace old indexed_voxels"
        );
        assert_index_consistency(&map, &ids);

        let old_only: SmallVec<[u64; 8]> = pre_refit_keys
            .iter()
            .filter(|k| !s0.indexed_voxels.contains(k))
            .copied()
            .collect();
        for key in &old_only {
            if let Some(bucket) = map.buckets.get(key) {
                assert!(
                    !bucket.contains(&surfel0),
                    "old key {key} still references surfel {surfel0:?} after refit"
                );
            }
        }
    }

    /// Two points separated by a voxel boundary (0.99 vs 1.01 with
    /// voxel_size=1.0) must be absorbed into the same growing surfel
    /// regardless of search_radius.  Growing surfels index themselves
    /// over mean ± growing_radius, so even radius=0 can find the
    /// cross-voxel neighbour.
    #[test]
    fn growing_surfel_crosses_voxel_boundary_with_zero_search_radius() {
        for radius in [0, 1] {
            let mut map = SurfelMap::new(
                SurfelMapConfig {
                    voxel_size: 1.0,
                    search_radius: radius,
                },
                surfel_config(|c| c.growing_radius = 3.5),
            );
            map.insert([pt(0.99, 0.0, 0.0), pt(1.01, 0.0, 0.0)].into_iter())
                .unwrap();
            assert_eq!(
                map.surfels.len(),
                1,
                "radius={radius}: cross-voxel neighbours must merge into one surfel"
            );
            let s = map.surfels.values().next().unwrap();
            assert_eq!(
                s.count, 2,
                "radius={radius}: merged surfel must have both points"
            );
        }
    }
}
