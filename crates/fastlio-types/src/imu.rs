use nalgebra::UnitQuaternion;

use crate::{Mat4, Vec3};

/// One IMU sample in core units.
///
/// Time is expressed in seconds. Angular velocity is in rad/s and acceleration
/// is in m/s^2. Both vectors are expressed in the IMU frame.
#[derive(Debug)]
pub struct ImuSample {
    // time_stamp: sec
    pub time_stamp_sec: f64,
    pub gyro: Vec3<f64>,
    pub accel: Vec3<f64>,
}

/// in this project, use T_BA means from B to A, T_AB means from A to B.
/// Pose3 T_BA: from B to A, as fastlio source code
/// Rotation_BA * Point_B + Translation
pub struct Pose3 {
    pub rotation: UnitQuaternion<f64>,
    pub translation: Vec3<f64>,
}

impl Pose3 {
    pub fn new(rotation: UnitQuaternion<f64>, translation: Vec3<f64>) -> Self {
        Pose3 {
            rotation,
            translation,
        }
    }

    /// apply rotation * point + translation
    pub fn transform_point(&self, rhs: &Vec3<f64>) -> Vec3<f64> {
        self.rotation * rhs + self.translation
    }

    /// apply T_AB * T_BC = T_AC
    pub fn transform(&self, rhs: &Pose3) -> Self {
        let rotation = self.rotation * rhs.rotation;
        let translation = self.rotation * rhs.translation + self.translation;
        Self {
            rotation,
            translation,
        }
    }

    // apply rotation * point only
    pub fn transform_vector(&self, rhs: &Vec3<f64>) -> Vec3<f64> {
        self.rotation * rhs
    }

    /// inverse transform, $T^(-1)$
    pub fn inverse(&self) -> Self {
        let rotation = self.rotation.inverse();
        let translation = -(rotation * self.translation);
        Self {
            rotation,
            translation,
        }
    }

    /// Reserved interface, converted to mat4, may be used for future optimization
    pub fn to_mat4d(&self) -> Mat4<f64> {
        let mut result = Mat4::<f64>::identity();
        result
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(self.rotation.to_rotation_matrix().matrix());
        result
            .fixed_view_mut::<3, 1>(0, 3)
            .copy_from(&self.translation);
        result
    }
}

/// LiDAR-to-IMU extrinsic transform `T_LI`.
///
/// This project uses `T_LI` to mean LiDAR frame to IMU frame:
/// `p_I = R_LI * p_L + t_LI`.
///
/// `t_LI` is the LiDAR origin expressed in the IMU frame.
pub type LidarImuExtrinsic = Pose3;
