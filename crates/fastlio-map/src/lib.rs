use fastlio_types::{Mat3, PointXYZI, Vec3};
use kiddo::{KdTree, NearestNeighbour, SquaredEuclidean};
use std::cmp::Ordering;

/// Local scan-to-map storage with owned world-frame points and a k-d tree index.
///
/// Points inserted into this map are expected to already be expressed in the
/// world/map frame `W`, in meters. The map owns the points; the `kiddo` index is
/// rebuilt after mutations and stores point indexes back into `points`.
#[derive(Debug)]
pub struct LocalMap {
    points: Vec<PointXYZI>,
    index: KdTree<f64, 3>,
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

impl LocalMap {
    pub fn new() -> Self {
        Self::from_points(Vec::new())
    }

    pub fn from_points(points: Vec<PointXYZI>) -> Self {
        let index = build_index(&points);
        Self { points, index }
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
        self.points.extend(points);
        self.rebuild_index();
    }

    pub fn nearest_n(&self, query_w: Vec3<f64>, count: usize) -> Vec<NearestPoint> {
        if count == 0 || self.points.is_empty() {
            return Vec::new();
        }

        self.index
            .nearest_n::<SquaredEuclidean>(&[query_w.x, query_w.y, query_w.z], count)
            .into_iter()
            .map(nearest_point_from_kiddo)
            .collect()
    }

    pub fn crop_by_center_radius(&mut self, center_w: Vec3<f64>, radius_m: f64) {
        if radius_m < 0.0 || !radius_m.is_finite() {
            self.points.clear();
            self.rebuild_index();
            return;
        }

        let radius_squared = radius_m * radius_m;
        self.points
            .retain(|point| squared_distance(point, &center_w) <= radius_squared);
        self.rebuild_index();
    }

    pub fn fit_plane_from_nearest(
        &self,
        query_w: Vec3<f64>,
        count: usize,
        config: PlaneFitConfig,
    ) -> Result<PlaneFit, PlaneFitError> {
        let neighbours = self.nearest_n(query_w, count);
        let points: Vec<_> = neighbours
            .iter()
            .map(|neighbour| &self.points[neighbour.index])
            .collect();
        fit_plane(points.as_slice(), config)
    }

    fn rebuild_index(&mut self) {
        self.index = build_index(&self.points);
    }
}

impl Default for LocalMap {
    fn default() -> Self {
        Self::new()
    }
}

fn build_index(points: &[PointXYZI]) -> KdTree<f64, 3> {
    let entries: Vec<[f64; 3]> = points
        .iter()
        .map(|point| [point.x as f64, point.y as f64, point.z as f64])
        .collect();
    (&entries).into()
}

fn nearest_point_from_kiddo(neighbour: NearestNeighbour<f64, u64>) -> NearestPoint {
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

pub fn fit_plane(points: &[&PointXYZI], config: PlaneFitConfig) -> Result<PlaneFit, PlaneFitError> {
    let required = config.min_points.max(3);
    if points.len() < required {
        return Err(PlaneFitError::NotEnoughPoints {
            actual: points.len(),
            required,
        });
    }

    let mut centroid_w = Vec3::zeros();
    for (idx, point) in points.iter().enumerate() {
        if !point.is_valid() {
            return Err(PlaneFitError::NonFinitePoint { index: idx });
        }
        centroid_w += point.to_vec3_f64();
    }
    centroid_w /= points.len() as f64;

    let mut covariance = Mat3::<f64>::zeros();
    for point in points {
        let delta = point.to_vec3_f64() - centroid_w;
        covariance += delta * delta.transpose();
    }
    covariance /= points.len() as f64;

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
}
