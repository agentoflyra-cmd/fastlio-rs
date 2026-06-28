pub mod ekfstate;
pub mod imu;
pub mod point_cloud;
pub mod time_stamp;

pub type Vec3<T> = nalgebra::Vector3<T>;
pub type Vec4<T> = nalgebra::Vector4<T>;
pub type Mat3<T> = nalgebra::Matrix3<T>;
pub type Mat4<T> = nalgebra::Matrix4<T>;

pub use ekfstate::*;
pub use imu::*;
pub use point_cloud::*;
pub use time_stamp::*;

#[cfg(test)]
mod test {

    use std::f64;

    use approx::assert_relative_eq;
    use nalgebra::UnitQuaternion;

    use crate::{PointXYZI, Pose3, Vec3, transfer_from_header};

    #[test]
    fn test_transfer_time() {
        let sec = 19214123;
        let nano: u32 = 123456789;
        let result = transfer_from_header(sec, nano);
        assert!(result == 19214123.123456789, "result = {}", result)
    }

    #[test]
    fn test_point_rejects_non_finite_coordinates() {
        let invalid_point = PointXYZI {
            x: f32::NAN,
            y: f32::NAN,
            z: f32::NAN,
            intensity: 1.0,
        };
        assert!(!invalid_point.is_valid());
        let invalid_point = PointXYZI {
            x: f32::INFINITY,
            y: 21.0,
            z: 1921.0,
            intensity: 0.1,
        };
        assert!(!invalid_point.is_valid());
        assert!(invalid_point.is_infinite());
    }

    #[test]
    fn test_pose_identity_preserves_pose() {
        let translation = Vec3::<f64>::new(0.0, 0.0, 0.0);
        let unit_pose3 = Pose3::new(UnitQuaternion::<f64>::identity(), translation);
        let point = Vec3::<f64>::new(5.0, 4.0, 3.0);
        let quat = UnitQuaternion::<f64>::identity();
        let pose3 = Pose3::new(quat, point);
        let result = pose3.transform(&unit_pose3);
        assert_relative_eq!(result.rotation, pose3.rotation, epsilon = 1e-9);
        assert_relative_eq!(result.translation, pose3.translation, epsilon = 1e-9);
    }

    #[test]
    fn test_pose_identity_preserves_point() {
        let translation = Vec3::<f64>::new(0.0, 0.0, 0.0);
        let unit_pose3 = Pose3::new(UnitQuaternion::<f64>::identity(), translation);
        let point = Vec3::<f64>::new(5.0, 4.0, 3.0);
        assert!(unit_pose3.transform_point(&point) == point)
    }

    #[test]
    fn test_lidar_to_imu_transform_matches_expected() {
        let r_li_extrinsic = UnitQuaternion::<f64>::new(Vec3::<f64>::x() * f64::consts::FRAC_PI_2);
        let t_li_extrinsic = Vec3::<f64>::new(2.0, 3.0, 4.0);
        let point_i = Vec3::<f64>::new(0.0, 1.0, 0.0);
        let result = r_li_extrinsic * point_i + t_li_extrinsic;
        let expected = Vec3::<f64>::new(2.0, 3.0, 5.0);
        assert_relative_eq!(expected, result, epsilon = 1.0e-9);
    }

    #[test]
    fn test_lidar_to_imu_inverse_round_trip() {
        let point_lidar = Vec3::<f64>::new(2.0, 3.0, 4.0);
        let rotation = UnitQuaternion::<f64>::new(Vec3::<f64>::y() * f64::consts::FRAC_PI_2);
        let translation = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let t_lidar_to_imu = Pose3::new(rotation, translation);
        let t_imu_to_lidar = t_lidar_to_imu.inverse();
        let point_lidar_inverse_roundtrip =
            t_imu_to_lidar.transform_point(&t_lidar_to_imu.transform_point(&point_lidar));

        assert_relative_eq!(
            point_lidar_inverse_roundtrip,
            point_lidar,
            epsilon = 1e-9 // "point_lidar_inverse_roundtrip: {point_lidar_inverse_roundtrip}, point_lidar: {point_lidar}"
        );
    }

    #[test]
    fn test_pose_inverse_round_trip() {
        let rotation = UnitQuaternion::<f64>::new(Vec3::<f64>::y() * f64::consts::FRAC_PI_2);
        let translation = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let t = Pose3::new(rotation, translation);
        let rotation = UnitQuaternion::<f64>::new(Vec3::<f64>::x() * f64::consts::FRAC_PI_2);
        let t2 = Pose3::new(rotation, translation);
        let t3 = t2.transform(&t);
        let t2_inverse_round_trip = t3.transform(&t.inverse());
        assert_relative_eq!(t2.rotation, t2_inverse_round_trip.rotation, epsilon = 1e-9);
        assert_relative_eq!(
            t2.translation,
            t2_inverse_round_trip.translation,
            epsilon = 1e-9
        );
    }
}
