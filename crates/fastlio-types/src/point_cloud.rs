use crate::Vec3;

/// LiDAR point coordinates and intensity.
///
/// Coordinates are expressed in the LiDAR frame unless stated otherwise.
/// Coordinates are meters. Intensity is sensor-specific but must be finite and
/// non-negative for valid points.
#[derive(Debug)]
pub struct PointXYZI {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
}

pub type PointCloud = Vec<PointXYZI>;

impl PointXYZI {
    pub fn to_voxel_key(&self, voxel_size: f32) -> [i32; 3] {
        [
            (self.x / voxel_size).floor() as i32,
            (self.y / voxel_size).floor() as i32,
            (self.z / voxel_size).floor() as i32,
        ]
    }

    pub fn to_vec3(&self) -> Vec3<f32> {
        Vec3::new(self.x, self.y, self.z)
    }

    pub fn to_vec3_f64(&self) -> Vec3<f64> {
        Vec3::new(self.x as f64, self.y as f64, self.z as f64)
    }

    #[inline]
    pub fn is_valid(&self) -> bool {
        self.x.is_finite()
            && self.y.is_finite()
            && self.z.is_finite()
            && self.intensity >= 0.0
            && self.intensity.is_finite()
    }

    #[inline]
    pub fn is_nan(&self) -> bool {
        self.x.is_nan() || self.y.is_nan() || self.z.is_nan()
    }

    #[inline]
    pub fn is_infinite(&self) -> bool {
        self.x.is_infinite() || self.y.is_infinite() || self.z.is_infinite()
    }

    /// provide a function for user, to clamp the infinite number to defualt
    pub fn sanitize(&mut self) {
        if !self.x.is_finite() {
            self.x = 0.0;
        }
        if !self.y.is_finite() {
            self.y = 0.0;
        }
        if !self.z.is_finite() {
            self.z = 0.0;
        }
        if !self.intensity.is_finite() {
            self.intensity = 0.0;
        }
        if self.intensity < 0.0 {
            self.intensity = 0.0;
        }
    }

    pub fn clamp(&mut self, min: f32, max: f32) {
        self.x = self.x.clamp(min, max);
        self.y = self.y.clamp(min, max);
        self.z = self.z.clamp(min, max);
        self.intensity = self.intensity.clamp(0.0, f32::MAX);
    }

    pub fn squared_distance(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }
}

/// LiDAR point with scan-relative acquisition time.
#[derive(Debug)]
pub struct TimedPoint {
    /// expected non-negative
    pub offset_time_sec: f64,
    pub point: PointXYZI,
    pub tag: u8,
    pub line: u8,
}

impl TimedPoint {
    pub fn add(&mut self, rhs: &Self) {
        self.point.x += rhs.point.x;
        self.point.y += rhs.point.y;
        self.point.z += rhs.point.z;
        self.point.intensity += rhs.point.intensity;
    }

    pub fn is_valid(&self) -> bool {
        self.point.is_valid() && self.offset_time_sec.is_finite()
    }
}

/// One LiDAR scan with owned points.
///
/// `base_timestamp_sec` is the scan start time in seconds. Each point's absolute
/// acquisition time is:
/// `base_timestamp + point.offset_time`.
#[derive(Debug)]
pub struct LidarFrame {
    /// Scan start time in seconds, using the dataset time base
    pub base_timestamp_sec: f64,
    /// Scan end time in seconds, using the dataset time end
    end_timestamp_sec: f64,
    /// Owned points in this scan.
    pub points: Vec<TimedPoint>,
}

impl LidarFrame {
    pub fn new(base_timestamp_sec: f64, end_timestamp_sec: f64, points: Vec<TimedPoint>) -> Self {
        Self {
            base_timestamp_sec,
            end_timestamp_sec,
            points,
        }
    }

    pub fn end_timestamp_sec(&self) -> f64 {
        self.end_timestamp_sec
    }

    /// Shifts the scan start and end timestamps by `offset_sec`.
    ///
    /// Point-level `offset_time_sec` values are scan-relative and are not
    /// changed. Use this when adapting a LiDAR message timestamp into another
    /// clock domain before synchronization or deskew.
    pub fn shift_timestamp_sec(&mut self, offset_sec: f64) {
        self.base_timestamp_sec += offset_sec;
        self.end_timestamp_sec += offset_sec;
    }
}

#[cfg(test)]
mod tests {
    use super::{LidarFrame, PointXYZI, TimedPoint};

    #[test]
    fn lidar_frame_shift_time_preserves_offsets_and_duration() {
        let mut frame = LidarFrame::new(
            10.0,
            10.2,
            vec![TimedPoint {
                offset_time_sec: 0.15,
                point: PointXYZI {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                    intensity: 4.0,
                },
                tag: 0,
                line: 0,
            }],
        );

        frame.shift_timestamp_sec(0.01);

        assert!((frame.base_timestamp_sec - 10.01).abs() < 1.0e-12);
        assert!((frame.end_timestamp_sec() - 10.21).abs() < 1.0e-12);
        assert_eq!(frame.points[0].offset_time_sec, 0.15);
        assert!((frame.end_timestamp_sec() - frame.base_timestamp_sec - 0.2).abs() < 1.0e-12);
    }
}
