use nalgebra::UnitQuaternion;

use crate::{LidarFrame, Vec3, imu::ImuSample};

/// Nominal navigation state without covariance.
///
/// Covariance is intentionally kept outside this type. The estimator crate will
/// own covariance and error-state layout later.
///
/// `orientation` is `R_WI`, mapping vectors from the IMU/body frame `I` into
/// the world/map frame `W`.
/// NavState only save IMU nominal state.
/// `δx = [δθ_I, δp_I, δv_I, δbω, δba, δg, δθ_LI, δp_LI]`
/// δθ_LI/δp_LI represents the LIDAR-IMU extrinsic calibration estimation error.
/// In pure IMU prediction, the external parameter error block remains identity
#[derive(Clone, PartialEq)]
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
    pub lidar: LidarFrame,
}
