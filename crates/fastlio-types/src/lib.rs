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
    pub debug_mode: bool,
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

#[derive(Deserialize, Clone)]
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

#[derive(Deserialize, Clone)]
#[serde(default)]
pub struct SurfelConfig {
    pub max_plane_distance: f32,
    pub max_planarity_ratio: f32,
    pub min_plane_spread_eigenvalue: f32,
    #[serde(default = "default_covariance_eigenvalue_floor")]
    pub covariance_eigenvalue_floor: f32,
    /// Optional variance floor in m^2 for mature surfel insertion support.
    /// When omitted, insertion uses `covariance_eigenvalue_floor`.
    #[serde(default)]
    pub insertion_covariance_eigenvalue_floor: Option<f32>,
    pub min_linearity: f32,
    pub min_line_spread_eigenvalue: f32,
    pub min_scattering: f32,
    pub min_mature_surfel_count: usize,
    pub max_mahalanobis_distance: f32,
    pub growing_radius: f32,
    /// Enable surfel-first association. Plane query is used when no surfel
    /// matches; far points on planar support use only the normal constraint.
    #[serde(alias = "query_surfel_fallback")]
    pub query_surfel_first: bool,
    /// Variance floor in m^2 for surfel association, not map insertion.
    pub association_covariance_eigenvalue_floor: f32,
}

impl Default for SurfelConfig {
    fn default() -> Self {
        Self {
            max_plane_distance: 0.30,
            max_planarity_ratio: 0.10,
            min_plane_spread_eigenvalue: 0.025,
            covariance_eigenvalue_floor: default_covariance_eigenvalue_floor(),
            insertion_covariance_eigenvalue_floor: None,
            min_linearity: 0.70,
            min_line_spread_eigenvalue: 0.0025,
            min_scattering: 0.60,
            min_mature_surfel_count: 8,
            max_mahalanobis_distance: 2.0,
            growing_radius: 0.2,
            query_surfel_first: false,
            association_covariance_eigenvalue_floor: 0.25,
        }
    }
}

fn default_covariance_eigenvalue_floor() -> f32 {
    1e-6
}

/// Iterated-EKF solver knobs, read from the `iekf:` section of the YAML config.
///
/// `min_delta_norm` is the convergence threshold on the norm of the 24-dim
/// error-state correction `dx`. That norm mixes units (rotation in rad,
/// position in m, velocity in m/s, biases, gravity in m/s^2), so the value is
/// only meaningful as an order-of-magnitude stopping rule, not as a physical
/// tolerance. FAST-LIO uses `max_iteration = 4` with a per-component
/// `|dx_j| < 1e-3` check; the default here is a `|dx| < 1e-3` norm check.
///
/// Measured on the yuanqu bag: the previous `1e-6` threshold left 57% of
/// frames hitting the iteration cap without converging (mean 8.51 iterations,
/// 60138 map queries/frame) and cost ~1.8x wall time, while the trajectory
/// stayed within 2.1 cm median / 8.3 cm max of the `1e-6` result.
#[derive(Deserialize, Clone, Copy, Debug)]
#[serde(default)]
pub struct IekfConfigSection {
    /// Maximum relinearization/association iterations per LiDAR frame.
    pub max_iterations: usize,
    /// Convergence threshold on `|dx|`; see the type-level docs for units.
    pub min_delta_norm: f64,
    /// Lower bound on the point-to-plane measurement variance (variance floor).
    pub measurement_variance_floor: f64,
    /// Minimum number of matched observations required to accept an update.
    pub min_observations: usize,
}

impl Default for IekfConfigSection {
    fn default() -> Self {
        Self {
            max_iterations: 10,
            min_delta_norm: 1.0e-3,
            measurement_variance_floor: 1.0e-6,
            min_observations: 0,
        }
    }
}

impl IekfConfigSection {
    /// Rejects values that would make the filter loop forever or stagnate.
    pub fn validate(&self) -> Result<()> {
        if self.max_iterations < 1 {
            anyhow::bail!(
                "iekf.max_iterations must be >= 1, got {}",
                self.max_iterations
            );
        }
        if !self.min_delta_norm.is_finite() || self.min_delta_norm <= 0.0 {
            anyhow::bail!(
                "iekf.min_delta_norm must be finite and > 0, got {}",
                self.min_delta_norm
            );
        }
        if !self.measurement_variance_floor.is_finite() || self.measurement_variance_floor <= 0.0 {
            anyhow::bail!(
                "iekf.measurement_variance_floor must be finite and > 0, got {}",
                self.measurement_variance_floor
            );
        }
        Ok(())
    }
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
    /// Solver parameters for the iterated EKF update. Omitted fields fall back
    /// to [`IekfConfigSection::default`].
    #[serde(default)]
    pub iekf: IekfConfigSection,
    #[serde(default)]
    pub surfel_config: Option<SurfelConfig>,
    #[serde(default)]
    pub surfel_map_config: Option<SurfelMapConfig>,
}

pub fn read_from_config_path<P: AsRef<Path>>(path: P) -> Result<Config> {
    let text = fs::read_to_string(path)?;
    let config: Config = serde_yaml::from_str(&text)?;
    config.iekf.validate()?;
    Ok(config)
}

#[cfg(test)]
mod test {

    use std::f64;

    use approx::assert_relative_eq;
    use nalgebra::UnitQuaternion;

    use crate::{PointXYZI, Pose3, Vec3, transfer_from_header};

    /// The shipped default must stay on the FAST-LIO-style convergence rule:
    /// a loose `|dx|` threshold that lets the loop exit once the correction is
    /// sub-millimetre instead of grinding to `1e-6` for four extra iterations.
    #[test]
    fn iekf_section_defaults_use_fastlio_convergence_threshold() {
        let config = crate::IekfConfigSection::default();
        assert_eq!(config.max_iterations, 10);
        assert_eq!(config.min_delta_norm, 1.0e-3);
        assert_eq!(config.measurement_variance_floor, 1.0e-6);
        assert_eq!(config.min_observations, 0);
        assert!(config.validate().is_ok());
    }

    #[test]
    fn iekf_section_rejects_degenerate_values() {
        let zero_iterations = crate::IekfConfigSection {
            max_iterations: 0,
            ..Default::default()
        };
        assert!(zero_iterations.validate().is_err());

        let non_finite = crate::IekfConfigSection {
            min_delta_norm: f64::NAN,
            ..Default::default()
        };
        assert!(non_finite.validate().is_err());

        let negative = crate::IekfConfigSection {
            min_delta_norm: -1.0,
            ..Default::default()
        };
        assert!(negative.validate().is_err());

        let zero_variance = crate::IekfConfigSection {
            measurement_variance_floor: 0.0,
            ..Default::default()
        };
        assert!(zero_variance.validate().is_err());
    }

    /// A partial `iekf:` section must override only the listed fields, and a
    /// config without the section must fall back to the documented default.
    #[test]
    fn iekf_yaml_section_overrides_only_listed_fields() {
        let yaml = r#"
common:
  debug_mode: false
  lid_topic: "/livox/lidar"
  imu_topic: "/livox/imu"
preprocess:
  lidar_type: 1
  blind: 0.5
mapping:
  acc_cov: 0.1
  gyr_cov: 0.1
  b_acc_cov: 0.0001
  b_gyr_cov: 0.0001
  extrinsic_est_en: false
  extrinsic_T: [0.0, 0.0, 0.0]
  extrinsic_R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
iekf:
  min_delta_norm: 5.0e-4
"#;
        let config: crate::Config = serde_yaml::from_str(yaml).unwrap();
        config.iekf.validate().unwrap();
        assert_eq!(config.iekf.min_delta_norm, 5.0e-4);
        assert_eq!(
            config.iekf.max_iterations,
            crate::IekfConfigSection::default().max_iterations
        );

        let without_section = yaml.replace("iekf:\n  min_delta_norm: 5.0e-4\n", "");
        let config: crate::Config = serde_yaml::from_str(&without_section).unwrap();
        assert_eq!(
            config.iekf.min_delta_norm,
            crate::IekfConfigSection::default().min_delta_norm
        );
    }

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
