use fastlio_types::{Pose3, Vec3};
use nalgebra::UnitQuaternion;

const TIME_EPS_SEC: f64 = 5.0e-6;

pub struct MotionSegment {
    /// Absolute timestamp sec of this interval's beginning.
    pub begin_time: f64,

    /// Absolute timestamp sec of this interval's end.
    pub end_time: f64,

    /// IMU pose in the world frame at `begin_time`.
    pub pose: Pose3,

    /// World-frame velocity at `begin_time`.
    pub velocity: Vec3<f64>,

    /// Bias-corrected body-frame angular velocity used over this interval.
    pub angular_velocity: Vec3<f64>,

    /// World-frame acceleration used over this interval.
    pub acceleration_world: Vec3<f64>,
}

impl MotionSegment {
    /// begin: imu_sample timestamp sec at segmentation begin tk.
    /// end: imu_sample time from t_k, t_k+1.
    /// pose: imu pose in the world frame at begin time, from navstate at t_k. orientation and position.
    /// velocity: navstate velocity at t_k. navstate.velocity
    /// angular_velocity: from imu measurement minus gyro bias. w_I = gyro - b_g
    /// acceleration_world: R_WI_mid * (accel_mid - b_a) + g
    pub fn new(
        begin_time: f64,
        end_time: f64,
        pose: Pose3,
        velocity: Vec3<f64>,
        angular_velocity: Vec3<f64>,
        acceleration_world: Vec3<f64>,
    ) -> Self {
        Self {
            begin_time,
            end_time,
            pose,
            velocity,
            angular_velocity,
            acceleration_world,
        }
    }

    pub fn contains(&self, time: f64) -> bool {
        self.begin_time <= time + TIME_EPS_SEC && time <= self.end_time + TIME_EPS_SEC
    }

    pub fn propagate_to(&self, time: f64) -> Pose3 {
        debug_assert!(self.contains(time));

        let dt = time - self.begin_time;

        let rotation =
            self.pose.rotation * UnitQuaternion::from_scaled_axis(self.angular_velocity * dt);
        let translation =
            self.pose.translation + self.velocity * dt + 0.5 * self.acceleration_world * dt * dt;

        Pose3 {
            rotation,
            translation,
        }
    }
}
