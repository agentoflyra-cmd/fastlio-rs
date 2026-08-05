use fastlio_types::{Mat3, PointXYZI, Vec3};
use kiddo::{DonnellyCyclicSimdFull, KdTree, QueryResultItem, SquaredEuclidean, VecOfArrays};
use std::cmp::Ordering;
use std::collections::HashMap;
use std::num::NonZeroUsize;

type MapKdTree =
    KdTree<f64, u32, DonnellyCyclicSimdFull<4>, VecOfArrays<f64, u32, 3, 4096>, 3, 4096>;

/// Local scan-to-map storage with owned world-frame points and a k-d tree index.
///
/// Points inserted into this map are expected to already be expressed in the
/// world/map frame `W`, in meters. The map owns the points; the `kiddo` index is
/// rebuilt after mutations and stores point indexes back into `points`.
pub struct LocalMap {
    points: Vec<PointXYZI>,
    index: MapKdTree,
    duplicate_filter: Option<MapDuplicateFilter>,
}

struct MapDuplicateFilter {
    voxel_size: f64,
    buckets: HashMap<[i32; 3], Vec<usize>>,
}

/// A nearest-neighbour hit returned by [`LocalMap`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NearestPoint {
    pub index: usize,
    pub squared_distance: f64,
}

/// Tunable checks for local plane fitting.
#[derive(Debug, Clone, Copy)]
pub struct PlaneFitConfig {
    pub min_points: usize,
    pub min_spread_eigenvalue: f64,
    pub max_planarity_ratio: f64,
}

impl Default for PlaneFitConfig {
    fn default() -> Self {
        Self {
            min_points: 3,
            min_spread_eigenvalue: 1.0e-9,
            max_planarity_ratio: 0.1,
        }
    }
}

/// Plane fitted to a local map neighbourhood.
///
/// The plane is represented in the world/map frame as:
/// `normal_w.dot(point_w) + offset = 0`.
#[derive(Debug, Clone, PartialEq)]
pub struct PlaneFit {
    pub centroid_w: Vec3<f64>,
    pub normal_w: Vec3<f64>,
    pub offset: f64,
    pub eigenvalues: Vec3<f64>,
    pub planarity_ratio: f64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PlaneFitError {
    NotEnoughPoints { actual: usize, required: usize },
    NonFinitePoint { index: usize },
    DegenerateNeighbourhood,
    NotPlanar,
}

/// Configuration for scan-to-map point-to-plane observation construction.
#[derive(Debug, Clone, Copy)]
pub struct PointToPlaneConfig {
    pub nearest_count: usize,
    pub max_neighbour_distance: f64,
    pub max_absolute_residual: f64,
    pub plane: PlaneFitConfig,
}

impl Default for PointToPlaneConfig {
    fn default() -> Self {
        Self {
            nearest_count: 5,
            max_neighbour_distance: 1.0,
            max_absolute_residual: 0.2,
            plane: PlaneFitConfig::default(),
        }
    }
}

/// One accepted point-to-plane observation in the world/map frame.
///
/// Residual sign convention:
/// `residual = plane.normal_w.dot(scan_point_w) + plane.offset`.
#[derive(Debug, Clone, PartialEq)]
pub struct PointToPlaneObservation {
    pub scan_point_w: Vec3<f64>,
    pub plane: PlaneFit,
    pub residual: f64,
    pub weight: f64,
    pub neighbour_indices: Vec<usize>,
}

/// Lightweight point-to-plane match for the scan-to-map hot path.
///
/// Unlike [`PointToPlaneObservation`], this omits debug-only neighbour indexes
/// and the repeated scan point, so callers can build estimator factors without
/// extra per-point allocations.
#[derive(Debug, Clone, PartialEq)]
pub struct PointToPlaneMatch {
    pub plane: PlaneFit,
    pub residual: f64,
    pub weight: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum PointToPlaneError {
    NonFiniteScanPoint,
    InvalidConfig,
    NeighbourTooFar {
        squared_distance: f64,
        max_squared_distance: f64,
    },
    PlaneFit(PlaneFitError),
    ResidualTooLarge {
        residual: f64,
        max_absolute_residual: f64,
    },
}

impl LocalMap {
    pub fn new() -> Self {
        Self::from_points(Vec::new())
    }

    pub fn from_points(points: Vec<PointXYZI>) -> Self {
        let index = build_index(&points);
        Self {
            points,
            index,
            duplicate_filter: None,
        }
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn points(&self) -> &[PointXYZI] {
        &self.points
    }

    pub fn insert_points<I>(&mut self, points: I)
    where
        I: IntoIterator<Item = PointXYZI>,
    {
        for point in points {
            self.insert_point(point);
        }
    }

    pub fn insert_points_with_min_distance<I>(&mut self, points: I, min_distance_m: f64)
    where
        I: IntoIterator<Item = PointXYZI>,
    {
        if !min_distance_m.is_finite() || min_distance_m <= 0.0 {
            self.insert_points(points);
            return;
        }

        let min_squared_distance = min_distance_m * min_distance_m;
        for point in points {
            if !point.is_valid() {
                continue;
            }
            self.ensure_duplicate_filter(min_distance_m);
            if !self.has_near_duplicate(&point, min_squared_distance) {
                self.insert_point(point);
            }
        }
    }

    pub fn nearest_n(&self, query_w: Vec3<f64>, count: usize) -> Vec<NearestPoint> {
        if count == 0 || self.points.is_empty() {
            return Vec::new();
        }

        self.index
            .query(&[query_w.x, query_w.y, query_w.z])
            .nearest_n::<SquaredEuclidean<f64>>(NonZeroUsize::new(count).unwrap())
            .execute()
            .into_iter()
            .map(nearest_point_from_kiddo)
            .collect()
    }

    pub fn crop_by_center_radius(&mut self, center_w: Vec3<f64>, radius_m: f64) {
        if radius_m < 0.0 || !radius_m.is_finite() {
            self.points.clear();
            self.rebuild_index();
            self.rebuild_duplicate_filter();
            return;
        }

        let radius_squared = radius_m * radius_m;
        let old_len = self.points.len();
        self.points
            .retain(|point| squared_distance(point, &center_w) <= radius_squared);
        if self.points.len() != old_len {
            self.rebuild_index();
            self.rebuild_duplicate_filter();
        }
    }

    pub fn fit_plane_from_nearest(
        &self,
        query_w: Vec3<f64>,
        count: usize,
        config: PlaneFitConfig,
    ) -> Result<PlaneFit, PlaneFitError> {
        let neighbours = self.nearest_n(query_w, count);
        self.fit_plane_from_neighbours(&neighbours, config)
    }

    pub fn point_to_plane_observation(
        &self,
        scan_point_w: Vec3<f64>,
        config: PointToPlaneConfig,
    ) -> Result<PointToPlaneObservation, PointToPlaneError> {
        let (matched, neighbours) =
            self.point_to_plane_match_with_neighbours(scan_point_w, config)?;
        let neighbour_indices = neighbours
            .into_iter()
            .map(|neighbour| neighbour.index)
            .collect();

        Ok(PointToPlaneObservation {
            scan_point_w,
            plane: matched.plane,
            residual: matched.residual,
            weight: matched.weight,
            neighbour_indices,
        })
    }

    pub fn point_to_plane_match(
        &self,
        scan_point_w: Vec3<f64>,
        config: PointToPlaneConfig,
    ) -> Result<PointToPlaneMatch, PointToPlaneError> {
        self.point_to_plane_match_with_neighbours(scan_point_w, config)
            .map(|(matched, _)| matched)
    }

    fn point_to_plane_match_with_neighbours(
        &self,
        scan_point_w: Vec3<f64>,
        config: PointToPlaneConfig,
    ) -> Result<(PointToPlaneMatch, Vec<NearestPoint>), PointToPlaneError> {
        if !scan_point_w.x.is_finite() || !scan_point_w.y.is_finite() || !scan_point_w.z.is_finite()
        {
            return Err(PointToPlaneError::NonFiniteScanPoint);
        }
        if config.nearest_count == 0
            || !config.max_neighbour_distance.is_finite()
            || config.max_neighbour_distance < 0.0
            || !config.max_absolute_residual.is_finite()
            || config.max_absolute_residual < 0.0
        {
            return Err(PointToPlaneError::InvalidConfig);
        }

        let neighbours = self.nearest_n(scan_point_w, config.nearest_count);
        if neighbours.len() < config.plane.min_points.max(3) {
            return Err(PointToPlaneError::PlaneFit(
                PlaneFitError::NotEnoughPoints {
                    actual: neighbours.len(),
                    required: config.plane.min_points.max(3),
                },
            ));
        }

        let max_squared_distance = config.max_neighbour_distance * config.max_neighbour_distance;
        let farthest_squared_distance = neighbours
            .last()
            .map(|neighbour| neighbour.squared_distance)
            .unwrap_or(f64::INFINITY);
        if farthest_squared_distance > max_squared_distance {
            return Err(PointToPlaneError::NeighbourTooFar {
                squared_distance: farthest_squared_distance,
                max_squared_distance,
            });
        }

        let plane = self
            .fit_plane_from_neighbours(&neighbours, config.plane)
            .map_err(PointToPlaneError::PlaneFit)?;
        let residual = plane.normal_w.dot(&scan_point_w) + plane.offset;

        if residual.abs() > config.max_absolute_residual {
            return Err(PointToPlaneError::ResidualTooLarge {
                residual,
                max_absolute_residual: config.max_absolute_residual,
            });
        }

        let weight = point_to_plane_weight(residual, plane.planarity_ratio, &config);

        Ok((
            PointToPlaneMatch {
                plane,
                residual,
                weight,
            },
            neighbours,
        ))
    }

    fn rebuild_index(&mut self) {
        self.index = build_index(&self.points);
    }

    fn insert_point(&mut self, point: PointXYZI) -> Option<usize> {
        let point_index = self.points.len();
        if self
            .index
            .add(
                &[point.x as f64, point.y as f64, point.z as f64],
                point_index as u32,
            )
            .is_ok()
        {
            self.points.push(point);
            self.insert_duplicate_filter_entry(point_index);
            Some(point_index)
        } else {
            None
        }
    }

    fn ensure_duplicate_filter(&mut self, voxel_size: f64) {
        let needs_rebuild = self
            .duplicate_filter
            .as_ref()
            .is_none_or(|filter| filter.voxel_size != voxel_size);
        if needs_rebuild {
            self.duplicate_filter = Some(MapDuplicateFilter::build(&self.points, voxel_size));
        }
    }

    fn rebuild_duplicate_filter(&mut self) {
        let Some(voxel_size) = self
            .duplicate_filter
            .as_ref()
            .map(|duplicate_filter| duplicate_filter.voxel_size)
        else {
            return;
        };
        self.duplicate_filter = Some(MapDuplicateFilter::build(&self.points, voxel_size));
    }

    fn insert_duplicate_filter_entry(&mut self, point_index: usize) {
        let Some(filter) = &mut self.duplicate_filter else {
            return;
        };
        let point = &self.points[point_index];
        if point.is_valid() {
            filter.insert(point, point_index);
        }
    }

    fn has_near_duplicate(&self, point: &PointXYZI, min_squared_distance: f64) -> bool {
        let Some(filter) = &self.duplicate_filter else {
            return false;
        };
        filter.has_near_duplicate(&self.points, point, min_squared_distance)
    }

    fn fit_plane_from_neighbours(
        &self,
        neighbours: &[NearestPoint],
        config: PlaneFitConfig,
    ) -> Result<PlaneFit, PlaneFitError> {
        fit_plane_from_indices(
            &self.points,
            neighbours.iter().map(|neighbour| neighbour.index),
            config,
        )
    }
}

impl MapDuplicateFilter {
    fn build(points: &[PointXYZI], voxel_size: f64) -> Self {
        let mut filter = Self {
            voxel_size,
            buckets: HashMap::with_capacity(points.len()),
        };
        for (point_index, point) in points.iter().enumerate() {
            if point.is_valid() {
                filter.insert(point, point_index);
            }
        }
        filter
    }

    fn insert(&mut self, point: &PointXYZI, point_index: usize) {
        self.buckets
            .entry(voxel_key(point, self.voxel_size))
            .or_default()
            .push(point_index);
    }

    fn has_near_duplicate(
        &self,
        map_points: &[PointXYZI],
        point: &PointXYZI,
        min_squared_distance: f64,
    ) -> bool {
        let center = voxel_key(point, self.voxel_size);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let key = [center[0] + dx, center[1] + dy, center[2] + dz];
                    let Some(candidates) = self.buckets.get(&key) else {
                        continue;
                    };
                    for &candidate_index in candidates {
                        if squared_distance_points(&map_points[candidate_index], point)
                            < min_squared_distance
                        {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }
}

impl Default for LocalMap {
    fn default() -> Self {
        Self::new()
    }
}

fn build_index(points: &[PointXYZI]) -> MapKdTree {
    let mut index = MapKdTree::default();
    for (point_index, point) in points.iter().enumerate() {
        index
            .add(
                &[point.x as f64, point.y as f64, point.z as f64],
                point_index as u32,
            )
            .ok();
    }
    index
}

fn nearest_point_from_kiddo(neighbour: QueryResultItem<(), u32, f64>) -> NearestPoint {
    NearestPoint {
        index: neighbour.item as usize,
        squared_distance: neighbour.distance,
    }
}

fn squared_distance(point: &PointXYZI, query_w: &Vec3<f64>) -> f64 {
    let dx = point.x as f64 - query_w.x;
    let dy = point.y as f64 - query_w.y;
    let dz = point.z as f64 - query_w.z;
    dx * dx + dy * dy + dz * dz
}

fn squared_distance_points(a: &PointXYZI, b: &PointXYZI) -> f64 {
    let dx = a.x as f64 - b.x as f64;
    let dy = a.y as f64 - b.y as f64;
    let dz = a.z as f64 - b.z as f64;
    dx * dx + dy * dy + dz * dz
}

fn voxel_key(point: &PointXYZI, voxel_size: f64) -> [i32; 3] {
    [
        (point.x as f64 / voxel_size).floor() as i32,
        (point.y as f64 / voxel_size).floor() as i32,
        (point.z as f64 / voxel_size).floor() as i32,
    ]
}

pub fn fit_plane(points: &[&PointXYZI], config: PlaneFitConfig) -> Result<PlaneFit, PlaneFitError> {
    fit_plane_from_points(points.iter().copied(), points.len(), config)
}

fn fit_plane_from_indices(
    map_points: &[PointXYZI],
    point_indices: impl Iterator<Item = usize> + Clone,
    config: PlaneFitConfig,
) -> Result<PlaneFit, PlaneFitError> {
    let count = point_indices.clone().count();
    fit_plane_from_points(
        point_indices.map(|point_index| &map_points[point_index]),
        count,
        config,
    )
}

fn fit_plane_from_points<'a>(
    points: impl Iterator<Item = &'a PointXYZI> + Clone,
    point_count: usize,
    config: PlaneFitConfig,
) -> Result<PlaneFit, PlaneFitError> {
    let required = config.min_points.max(3);
    if point_count < required {
        return Err(PlaneFitError::NotEnoughPoints {
            actual: point_count,
            required,
        });
    }

    let mut centroid_w = Vec3::zeros();
    for (idx, point) in points.clone().enumerate() {
        if !point.is_valid() {
            return Err(PlaneFitError::NonFinitePoint { index: idx });
        }
        centroid_w += point.to_vec3_f64();
    }
    centroid_w /= point_count as f64;

    let mut covariance = Mat3::<f64>::zeros();
    for point in points {
        let delta = point.to_vec3_f64() - centroid_w;
        covariance += delta * delta.transpose();
    }
    covariance /= point_count as f64;

    let eigen = covariance.symmetric_eigen();
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
    eigen_pairs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal));

    let smallest = eigen_pairs[0].0.max(0.0);
    let middle = eigen_pairs[1].0.max(0.0);
    let largest = eigen_pairs[2].0.max(0.0);

    if middle <= config.min_spread_eigenvalue || largest <= config.min_spread_eigenvalue {
        return Err(PlaneFitError::DegenerateNeighbourhood);
    }

    let planarity_ratio = smallest / middle;
    if planarity_ratio > config.max_planarity_ratio {
        return Err(PlaneFitError::NotPlanar);
    }

    let normal_w = eigen_pairs[0].1.normalize();
    let offset = -normal_w.dot(&centroid_w);

    Ok(PlaneFit {
        centroid_w,
        normal_w,
        offset,
        eigenvalues: Vec3::new(smallest, middle, largest),
        planarity_ratio,
    })
}

fn point_to_plane_weight(residual: f64, planarity_ratio: f64, config: &PointToPlaneConfig) -> f64 {
    let residual_score = if config.max_absolute_residual <= 0.0 {
        1.0
    } else {
        1.0 - residual.abs() / config.max_absolute_residual
    };
    let planarity_score = if config.plane.max_planarity_ratio <= 0.0 {
        1.0
    } else {
        1.0 - planarity_ratio / config.plane.max_planarity_ratio
    };

    residual_score.clamp(0.0, 1.0) * planarity_score.clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
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
    fn empty_map_returns_no_neighbours() {
        let map = LocalMap::new();
        assert!(map.nearest_n(Vec3::zeros(), 3).is_empty());
    }

    #[test]
    fn nearest_n_returns_points_ordered_by_squared_distance() {
        let map = LocalMap::from_points(vec![
            point(10.0, 0.0, 0.0),
            point(1.0, 0.0, 0.0),
            point(3.0, 0.0, 0.0),
        ]);

        let hits = map.nearest_n(Vec3::zeros(), 3);

        assert_eq!(hits.len(), 3);
        assert_eq!(hits[0].index, 1);
        assert_eq!(hits[0].squared_distance, 1.0);
        assert_eq!(hits[1].index, 2);
        assert_eq!(hits[1].squared_distance, 9.0);
        assert_eq!(hits[2].index, 0);
        assert_eq!(hits[2].squared_distance, 100.0);
    }

    #[test]
    fn nearest_n_clamps_to_map_size() {
        let map = LocalMap::from_points(vec![point(1.0, 0.0, 0.0)]);

        let hits = map.nearest_n(Vec3::zeros(), 10);

        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].index, 0);
    }

    #[test]
    fn insert_points_rebuilds_index() {
        let mut map = LocalMap::from_points(vec![point(10.0, 0.0, 0.0)]);
        map.insert_points(vec![point(1.0, 0.0, 0.0)]);

        let hits = map.nearest_n(Vec3::zeros(), 1);

        assert_eq!(map.len(), 2);
        assert_eq!(hits[0].index, 1);
    }

    #[test]
    fn insert_points_with_min_distance_skips_near_duplicates() {
        let mut map = LocalMap::new();

        map.insert_points_with_min_distance(
            vec![
                point(0.0, 0.0, 0.0),
                point(0.05, 0.0, 0.0),
                point(0.2, 0.0, 0.0),
            ],
            0.1,
        );

        assert_eq!(map.len(), 2);
        let hits = map.nearest_n(Vec3::new(0.05, 0.0, 0.0), 2);
        assert_eq!(hits[0].index, 0);
        assert_eq!(hits[1].index, 1);
    }

    #[test]
    fn index_build_accepts_many_points_with_same_axis_value() {
        let points = (0..128)
            .map(|idx| point(1.0, idx as f32 * 0.01, 0.0))
            .collect();
        let map = LocalMap::from_points(points);

        let hits = map.nearest_n(Vec3::new(1.0, 0.0, 0.0), 5);

        assert_eq!(map.len(), 128);
        assert_eq!(hits.len(), 5);
    }

    #[test]
    fn crop_by_center_radius_removes_outside_points_and_rebuilds_index() {
        let mut map = LocalMap::from_points(vec![
            point(0.0, 0.0, 0.0),
            point(1.0, 0.0, 0.0),
            point(3.0, 0.0, 0.0),
        ]);

        map.crop_by_center_radius(Vec3::zeros(), 1.5);

        assert_eq!(map.len(), 2);
        let hits = map.nearest_n(Vec3::new(3.0, 0.0, 0.0), 1);
        assert_eq!(hits[0].index, 1);
        assert_eq!(map.points()[0].x, 0.0);
        assert_eq!(map.points()[1].x, 1.0);
    }

    #[test]
    fn negative_crop_radius_clears_map() {
        let mut map = LocalMap::from_points(vec![point(0.0, 0.0, 0.0)]);

        map.crop_by_center_radius(Vec3::zeros(), -1.0);

        assert!(map.is_empty());
        assert!(map.nearest_n(Vec3::zeros(), 1).is_empty());
    }

    #[test]
    fn duplicate_filter_stays_consistent_after_clearing_crop() {
        let mut map = LocalMap::from_points(vec![point(0.0, 0.0, 0.0)]);
        map.insert_points_with_min_distance(vec![point(0.2, 0.0, 0.0)], 0.1);

        map.crop_by_center_radius(Vec3::zeros(), -1.0);
        map.insert_points_with_min_distance(vec![point(0.0, 0.0, 0.0)], 0.1);

        assert_eq!(map.len(), 1);
        assert_eq!(map.points()[0].x, 0.0);
    }

    #[test]
    fn fit_plane_accepts_perfect_horizontal_plane() {
        let points = [
            point(-1.0, -1.0, 2.0),
            point(1.0, -1.0, 2.0),
            point(-1.0, 1.0, 2.0),
            point(1.0, 1.0, 2.0),
            point(0.0, 0.0, 2.0),
        ];
        let refs: Vec<_> = points.iter().collect();

        let plane = fit_plane(&refs, PlaneFitConfig::default()).unwrap();

        assert!(plane.normal_w.z.abs() > 1.0 - 1.0e-9);
        assert!((plane.offset.abs() - 2.0).abs() < 1.0e-9);
        assert!(plane.planarity_ratio < 1.0e-9);
        for point in &points {
            let residual = plane.normal_w.dot(&point.to_vec3_f64()) + plane.offset;
            assert!(residual.abs() < 1.0e-9);
        }
    }

    #[test]
    fn fit_plane_rejects_not_enough_points() {
        let points = [point(0.0, 0.0, 0.0), point(1.0, 0.0, 0.0)];
        let refs: Vec<_> = points.iter().collect();

        let err = fit_plane(&refs, PlaneFitConfig::default()).unwrap_err();

        assert_eq!(
            err,
            PlaneFitError::NotEnoughPoints {
                actual: 2,
                required: 3
            }
        );
    }

    #[test]
    fn fit_plane_rejects_collinear_points() {
        let points = [
            point(0.0, 0.0, 0.0),
            point(1.0, 0.0, 0.0),
            point(2.0, 0.0, 0.0),
            point(3.0, 0.0, 0.0),
        ];
        let refs: Vec<_> = points.iter().collect();

        let err = fit_plane(&refs, PlaneFitConfig::default()).unwrap_err();

        assert_eq!(err, PlaneFitError::DegenerateNeighbourhood);
    }

    #[test]
    fn fit_plane_rejects_non_finite_point() {
        let points = [
            point(0.0, 0.0, 0.0),
            point(1.0, 0.0, 0.0),
            point(f32::NAN, 1.0, 0.0),
        ];
        let refs: Vec<_> = points.iter().collect();

        let err = fit_plane(&refs, PlaneFitConfig::default()).unwrap_err();

        assert_eq!(err, PlaneFitError::NonFinitePoint { index: 2 });
    }

    #[test]
    fn fit_plane_rejects_non_planar_neighbourhood() {
        let points = [
            point(1.0, 0.0, 0.0),
            point(-1.0, 0.0, 0.0),
            point(0.0, 1.0, 0.0),
            point(0.0, -1.0, 0.0),
            point(0.0, 0.0, 1.0),
            point(0.0, 0.0, -1.0),
        ];
        let refs: Vec<_> = points.iter().collect();

        let err = fit_plane(&refs, PlaneFitConfig::default()).unwrap_err();

        assert_eq!(err, PlaneFitError::NotPlanar);
    }

    #[test]
    fn fit_plane_from_nearest_uses_local_map_neighbourhood() {
        let map = LocalMap::from_points(vec![
            point(0.0, 0.0, 1.0),
            point(1.0, 0.0, 1.0),
            point(0.0, 1.0, 1.0),
            point(1.0, 1.0, 1.0),
            point(100.0, 100.0, 100.0),
        ]);

        let plane = map
            .fit_plane_from_nearest(Vec3::new(0.5, 0.5, 1.0), 4, PlaneFitConfig::default())
            .unwrap();

        assert!(plane.normal_w.z.abs() > 1.0 - 1.0e-9);
        assert!((plane.offset.abs() - 1.0).abs() < 1.0e-9);
    }

    #[test]
    fn point_to_plane_observation_accepts_point_near_plane() {
        let map = LocalMap::from_points(vec![
            point(-1.0, -1.0, 2.0),
            point(1.0, -1.0, 2.0),
            point(-1.0, 1.0, 2.0),
            point(1.0, 1.0, 2.0),
            point(0.0, 0.0, 2.0),
        ]);
        let config = PointToPlaneConfig {
            nearest_count: 5,
            max_neighbour_distance: 3.0,
            max_absolute_residual: 0.2,
            plane: PlaneFitConfig::default(),
        };

        let observation = map
            .point_to_plane_observation(Vec3::new(0.25, -0.25, 2.05), config)
            .unwrap();

        assert_eq!(observation.neighbour_indices.len(), 5);
        assert!((observation.residual.abs() - 0.05).abs() < 1.0e-6);
        assert!(observation.weight > 0.0);
        assert!(observation.weight <= 1.0);
    }

    #[test]
    fn point_to_plane_observation_rejects_large_residual() {
        let map = LocalMap::from_points(vec![
            point(-1.0, -1.0, 0.0),
            point(1.0, -1.0, 0.0),
            point(-1.0, 1.0, 0.0),
            point(1.0, 1.0, 0.0),
            point(0.0, 0.0, 0.0),
        ]);
        let config = PointToPlaneConfig {
            max_neighbour_distance: 3.0,
            max_absolute_residual: 0.1,
            ..PointToPlaneConfig::default()
        };

        let err = map
            .point_to_plane_observation(Vec3::new(0.0, 0.0, 0.5), config)
            .unwrap_err();

        assert!(matches!(
            err,
            PointToPlaneError::ResidualTooLarge {
                max_absolute_residual: 0.1,
                ..
            }
        ));
    }

    #[test]
    fn point_to_plane_observation_rejects_far_neighbourhood() {
        let map = LocalMap::from_points(vec![
            point(10.0, 10.0, 0.0),
            point(11.0, 10.0, 0.0),
            point(10.0, 11.0, 0.0),
            point(11.0, 11.0, 0.0),
            point(10.5, 10.5, 0.0),
        ]);
        let config = PointToPlaneConfig {
            max_neighbour_distance: 1.0,
            max_absolute_residual: 1.0,
            ..PointToPlaneConfig::default()
        };

        let err = map
            .point_to_plane_observation(Vec3::zeros(), config)
            .unwrap_err();

        assert!(matches!(err, PointToPlaneError::NeighbourTooFar { .. }));
    }

    #[test]
    fn point_to_plane_observation_propagates_plane_degeneracy() {
        let map = LocalMap::from_points(vec![
            point(0.0, 0.0, 0.0),
            point(1.0, 0.0, 0.0),
            point(2.0, 0.0, 0.0),
            point(3.0, 0.0, 0.0),
            point(4.0, 0.0, 0.0),
        ]);
        let config = PointToPlaneConfig {
            max_neighbour_distance: 10.0,
            max_absolute_residual: 1.0,
            ..PointToPlaneConfig::default()
        };

        let err = map
            .point_to_plane_observation(Vec3::new(0.5, 0.0, 0.0), config)
            .unwrap_err();

        assert_eq!(
            err,
            PointToPlaneError::PlaneFit(PlaneFitError::DegenerateNeighbourhood)
        );
    }

    #[test]
    fn point_to_plane_observation_rejects_non_finite_scan_point() {
        let map = LocalMap::new();
        let err = map
            .point_to_plane_observation(
                Vec3::new(f64::NAN, 0.0, 0.0),
                PointToPlaneConfig::default(),
            )
            .unwrap_err();

        assert_eq!(err, PointToPlaneError::NonFiniteScanPoint);
    }
}
