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
    pub fn to_vec3(&self) -> Vec3<f32> {
        Vec3::new(self.x, self.y, self.z)
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
}

/// LiDAR point with scan-relative acquisition time.
#[derive(Debug)]
pub struct TimedPoint {
    /// expected non-negative
    pub offset_time_sec: f64,
    pub point: PointXYZI,
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
    /// Scan end time in seconds, using the dataset time base
    pub end_timestamp_sec: f64,
    /// Owned points in this scan.
    pub points: Vec<TimedPoint>,
}
