use fastlio_types::{PointXYZI, Vec3};
use kiddo::{KdTree, NearestNeighbour, SquaredEuclidean};

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
}
