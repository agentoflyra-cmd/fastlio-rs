pub mod ekfstate;
pub mod imu;
pub mod point_cloud;
pub mod time_stamp;

pub type Vec3<T> = nalgebra::Vector3<T>;
pub type Vec4<T> = nalgebra::Vector4<T>;
pub type Mat3<T> = nalgebra::Matrix3<T>;
pub type Mat4<T> = nalgebra::Matrix4<T>;

use anyhow::Result;
pub use ekfstate::*;
pub use imu::*;
pub use point_cloud::*;
use serde::de::Error;
use serde::{Deserialize, Deserializer};
use std::fs;
use std::path::Path;
pub use time_stamp::*;

fn deserialize_vec3<'de, D, T>(deserializer: D) -> Result<Vec3<T>, D::Error>
where
    D: Deserializer<'de>,
    T: Deserialize<'de> + nalgebra::Scalar,
{
    let [x, y, z] = <[T; 3]>::deserialize(deserializer)?;
    Ok(Vec3::new(x, y, z))
}

fn deserialize_mat3<'de, D, T>(deserializer: D) -> Result<Mat3<T>, D::Error>
where
    D: Deserializer<'de>,
    T: Deserialize<'de> + nalgebra::Scalar,
{
    let values = <[T; 9]>::deserialize(deserializer)?;
    Ok(Mat3::from_row_slice(&values))
}

#[derive(Deserialize)]
pub struct CommonConfig {
    pub lid_topic: String,
    pub imu_topic: String,
    pub time_sync_en: Option<bool>,
    pub time_offset_lidar_to_imu: Option<f64>,
}

pub enum LidarType {
    Avia,
    Velodyne,
    Ouster,
    Mid360,
}

impl<'de> Deserialize<'de> for LidarType {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let value = u8::deserialize(deserializer)?;

        match value {
            1 => Ok(Self::Avia),
            2 => Ok(Self::Velodyne),
            3 => Ok(Self::Ouster),
            4 => Ok(Self::Mid360),
            _ => Err(D::Error::custom(format!("unsupported lidar type: {value}"))),
        }
    }
}

#[derive(Deserialize)]
#[serde(default)]
pub struct SurfelMapConfig {
    pub voxel_size: f32,
    pub search_radius: i32,
}

impl Default for SurfelMapConfig {
    fn default() -> Self {
        Self {
            voxel_size: 0.4,
            search_radius: 2,
        }
    }
}

#[derive(Deserialize)]
#[serde(default)]
pub struct SurfelConfig {
    pub max_plane_distance: f32,
    pub max_planarity_ratio: f32,
    pub min_plane_spread_eigenvalue: f32,
    #[serde(default = "default_covariance_eigenvalue_floor")]
    pub covariance_eigenvalue_floor: f32,
    pub min_linearity: f32,
    pub min_line_spread_eigenvalue: f32,
    pub min_scattering: f32,
    pub min_mature_surfel_count: usize,
    pub max_mahalanobis_distance: f32,
    pub growing_radius: f32,
}

impl Default for SurfelConfig {
    fn default() -> Self {
        Self {
            max_plane_distance: 0.08,
            max_planarity_ratio: 0.10,
            min_plane_spread_eigenvalue: 0.025,
            covariance_eigenvalue_floor: default_covariance_eigenvalue_floor(),
            min_linearity: 0.70,
            min_line_spread_eigenvalue: 0.0025,
            min_scattering: 0.60,
            min_mature_surfel_count: 8,
            max_mahalanobis_distance: 2.0,
            growing_radius: 0.2,
        }
    }
}

fn default_covariance_eigenvalue_floor() -> f32 {
    1e-6
}

#[derive(Deserialize)]
pub struct PreprocessConfig {
    pub lidar_type: LidarType,
    pub scan_line: Option<u8>,
    #[serde(rename = "blind")]
    pub blind_zone: f32,
    pub voxel_size: Option<f32>,
    pub max_range: Option<f32>,
}

#[derive(Deserialize)]
pub struct MappingConfig {
    pub acc_cov: f64,
    pub gyr_cov: f64,
    pub b_acc_cov: f64,
    pub b_gyr_cov: f64,
    pub extrinsic_est_en: bool,
    #[serde(deserialize_with = "deserialize_vec3", rename = "extrinsic_T")]
    pub extrinsic_t: Vec3<f64>,
    #[serde(deserialize_with = "deserialize_mat3", rename = "extrinsic_R")]
    pub extrinsic_r: Mat3<f64>,
}

#[derive(Deserialize)]
pub struct Config {
    pub common: CommonConfig,
    pub preprocess: PreprocessConfig,
    pub mapping: MappingConfig,
    #[serde(default)]
    pub surfel_config: Option<SurfelConfig>,
    #[serde(default)]
    pub surfel_map_config: Option<SurfelMapConfig>,
}

pub fn read_from_config_path<P: AsRef<Path>>(path: P) -> Result<Config> {
    let text = fs::read_to_string(path)?;
    let config = serde_yaml::from_str(&text)?;
    Ok(config)
}

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
