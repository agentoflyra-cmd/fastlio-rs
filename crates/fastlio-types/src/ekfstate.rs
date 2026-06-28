use nalgebra::UnitQuaternion;

use crate::{Vec3, imu::ImuSample, point_cloud::PointCloud};

/// Nominal navigation state without covariance.
///
/// Covariance is intentionally kept outside this type. The estimator crate will
/// own covariance and error-state layout later.
///
/// `orientation` is `R_WI`, mapping vectors from the IMU/body frame `I` into
/// the world/map frame `W`.
pub struct NavState {
    pub position: Vec3<f64>,
    /// Orientation `R_WI`, mapping vectors from IMU frame `I` into world frame `W`.
    pub orientation: UnitQuaternion<f64>,
    pub velocity: Vec3<f64>,
    pub gyro_bias: Vec3<f64>,
    pub accel_bias: Vec3<f64>,
    pub gravity: Vec3<f64>,
}

pub struct MeasureGroup {
    pub lidar_beg_time: f64,
    pub lidar_end_time: f64,
    pub imu: Vec<ImuSample>,
    pub lidar: PointCloud,
}
