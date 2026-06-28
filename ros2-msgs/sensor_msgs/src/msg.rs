#![allow(unused_imports)]
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct BatteryState {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub voltage: f32,
    #[allow(missing_docs)]
    pub temperature: f32,
    #[allow(missing_docs)]
    pub current: f32,
    #[allow(missing_docs)]
    pub charge: f32,
    #[allow(missing_docs)]
    pub capacity: f32,
    #[allow(missing_docs)]
    pub design_capacity: f32,
    #[allow(missing_docs)]
    pub percentage: f32,
    #[allow(missing_docs)]
    pub power_supply_status: u8,
    #[allow(missing_docs)]
    pub power_supply_health: u8,
    #[allow(missing_docs)]
    pub power_supply_technology: u8,
    #[allow(missing_docs)]
    pub present: bool,
    #[allow(missing_docs)]
    pub cell_voltage: Vec<f32>,
    #[allow(missing_docs)]
    pub cell_temperature: Vec<f32>,
    #[allow(missing_docs)]
    pub location: std::string::String,
    #[allow(missing_docs)]
    pub serial_number: std::string::String,
}

impl BatteryState {
    pub const POWER_SUPPLY_STATUS_UNKNOWN: u8 = 0;
    pub const POWER_SUPPLY_STATUS_CHARGING: u8 = 1;
    pub const POWER_SUPPLY_STATUS_DISCHARGING: u8 = 2;
    pub const POWER_SUPPLY_STATUS_NOT_CHARGING: u8 = 3;
    pub const POWER_SUPPLY_STATUS_FULL: u8 = 4;
    pub const POWER_SUPPLY_HEALTH_UNKNOWN: u8 = 0;
    pub const POWER_SUPPLY_HEALTH_GOOD: u8 = 1;
    pub const POWER_SUPPLY_HEALTH_OVERHEAT: u8 = 2;
    pub const POWER_SUPPLY_HEALTH_DEAD: u8 = 3;
    pub const POWER_SUPPLY_HEALTH_OVERVOLTAGE: u8 = 4;
    pub const POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: u8 = 5;
    pub const POWER_SUPPLY_HEALTH_COLD: u8 = 6;
    pub const POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE: u8 = 7;
    pub const POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE: u8 = 8;
    pub const POWER_SUPPLY_TECHNOLOGY_UNKNOWN: u8 = 0;
    pub const POWER_SUPPLY_TECHNOLOGY_NIMH: u8 = 1;
    pub const POWER_SUPPLY_TECHNOLOGY_LION: u8 = 2;
    pub const POWER_SUPPLY_TECHNOLOGY_LIPO: u8 = 3;
    pub const POWER_SUPPLY_TECHNOLOGY_LIFE: u8 = 4;
    pub const POWER_SUPPLY_TECHNOLOGY_NICD: u8 = 5;
    pub const POWER_SUPPLY_TECHNOLOGY_LIMN: u8 = 6;
    pub const POWER_SUPPLY_TECHNOLOGY_TERNARY: u8 = 7;
    pub const POWER_SUPPLY_TECHNOLOGY_VRLA: u8 = 8;
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct CameraInfo {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub height: u32,
    #[allow(missing_docs)]
    pub width: u32,
    #[allow(missing_docs)]
    pub distortion_model: std::string::String,
    #[allow(missing_docs)]
    pub d: Vec<f64>,
    #[allow(missing_docs)]
    pub k: [f64; 9],
    #[allow(missing_docs)]
    pub r: [f64; 9],
    #[allow(missing_docs)]
    pub p: [f64; 12],
    #[allow(missing_docs)]
    pub binning_x: u32,
    #[allow(missing_docs)]
    pub binning_y: u32,
    #[allow(missing_docs)]
    pub roi: RegionOfInterest,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct ChannelFloat32 {
    #[allow(missing_docs)]
    pub name: std::string::String,
    #[allow(missing_docs)]
    pub values: Vec<f32>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct CompressedImage {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub format: std::string::String,
    #[allow(missing_docs)]
    pub data: Vec<u8>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct FluidPressure {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub fluid_pressure: f64,
    #[allow(missing_docs)]
    pub variance: f64,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Illuminance {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub illuminance: f64,
    #[allow(missing_docs)]
    pub variance: f64,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Image {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub height: u32,
    #[allow(missing_docs)]
    pub width: u32,
    #[allow(missing_docs)]
    pub encoding: std::string::String,
    #[allow(missing_docs)]
    pub is_bigendian: u8,
    #[allow(missing_docs)]
    pub step: u32,
    #[allow(missing_docs)]
    pub data: Vec<u8>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Imu {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub orientation: geometry_msgs::msg::Quaternion,
    #[allow(missing_docs)]
    pub orientation_covariance: [f64; 9],
    #[allow(missing_docs)]
    pub angular_velocity: geometry_msgs::msg::Vector3,
    #[allow(missing_docs)]
    pub angular_velocity_covariance: [f64; 9],
    #[allow(missing_docs)]
    pub linear_acceleration: geometry_msgs::msg::Vector3,
    #[allow(missing_docs)]
    pub linear_acceleration_covariance: [f64; 9],
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct JointState {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub name: Vec<std::string::String>,
    #[allow(missing_docs)]
    pub position: Vec<f64>,
    #[allow(missing_docs)]
    pub velocity: Vec<f64>,
    #[allow(missing_docs)]
    pub effort: Vec<f64>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Joy {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub axes: Vec<f32>,
    #[allow(missing_docs)]
    pub buttons: Vec<i32>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct JoyFeedback {
    #[allow(missing_docs)]
    pub r#type: u8,
    #[allow(missing_docs)]
    pub id: u8,
    #[allow(missing_docs)]
    pub intensity: f32,
}

impl JoyFeedback {
    pub const TYPE_LED: u8 = 0;
    pub const TYPE_RUMBLE: u8 = 1;
    pub const TYPE_BUZZER: u8 = 2;
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct JoyFeedbackArray {
    #[allow(missing_docs)]
    pub array: Vec<JoyFeedback>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct LaserEcho {
    #[allow(missing_docs)]
    pub echoes: Vec<f32>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct LaserScan {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub angle_min: f32,
    #[allow(missing_docs)]
    pub angle_max: f32,
    #[allow(missing_docs)]
    pub angle_increment: f32,
    #[allow(missing_docs)]
    pub time_increment: f32,
    #[allow(missing_docs)]
    pub scan_time: f32,
    #[allow(missing_docs)]
    pub range_min: f32,
    #[allow(missing_docs)]
    pub range_max: f32,
    #[allow(missing_docs)]
    pub ranges: Vec<f32>,
    #[allow(missing_docs)]
    pub intensities: Vec<f32>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct MagneticField {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub magnetic_field: geometry_msgs::msg::Vector3,
    #[allow(missing_docs)]
    pub magnetic_field_covariance: [f64; 9],
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct MultiDOFJointState {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub joint_names: Vec<std::string::String>,
    #[allow(missing_docs)]
    pub transforms: Vec<geometry_msgs::msg::Transform>,
    #[allow(missing_docs)]
    pub twist: Vec<geometry_msgs::msg::Twist>,
    #[allow(missing_docs)]
    pub wrench: Vec<geometry_msgs::msg::Wrench>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct MultiEchoLaserScan {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub angle_min: f32,
    #[allow(missing_docs)]
    pub angle_max: f32,
    #[allow(missing_docs)]
    pub angle_increment: f32,
    #[allow(missing_docs)]
    pub time_increment: f32,
    #[allow(missing_docs)]
    pub scan_time: f32,
    #[allow(missing_docs)]
    pub range_min: f32,
    #[allow(missing_docs)]
    pub range_max: f32,
    #[allow(missing_docs)]
    pub ranges: Vec<LaserEcho>,
    #[allow(missing_docs)]
    pub intensities: Vec<LaserEcho>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct NavSatFix {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub status: NavSatStatus,
    #[allow(missing_docs)]
    pub latitude: f64,
    #[allow(missing_docs)]
    pub longitude: f64,
    #[allow(missing_docs)]
    pub altitude: f64,
    #[allow(missing_docs)]
    pub position_covariance: [f64; 9],
    #[allow(missing_docs)]
    pub position_covariance_type: u8,
}

impl NavSatFix {
    pub const COVARIANCE_TYPE_UNKNOWN: u8 = 0;
    pub const COVARIANCE_TYPE_APPROXIMATED: u8 = 1;
    pub const COVARIANCE_TYPE_DIAGONAL_KNOWN: u8 = 2;
    pub const COVARIANCE_TYPE_KNOWN: u8 = 3;
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct NavSatStatus {
    #[allow(missing_docs)]
    pub status: i8,
    #[allow(missing_docs)]
    pub service: u16,
}

impl NavSatStatus {
    pub const STATUS_UNKNOWN: i8 = -2;
    pub const STATUS_NO_FIX: i8 = -1;
    pub const STATUS_FIX: i8 = 0;
    pub const STATUS_SBAS_FIX: i8 = 1;
    pub const STATUS_GBAS_FIX: i8 = 2;
    pub const SERVICE_UNKNOWN: u16 = 0;
    pub const SERVICE_GPS: u16 = 1;
    pub const SERVICE_GLONASS: u16 = 2;
    pub const SERVICE_COMPASS: u16 = 4;
    pub const SERVICE_GALILEO: u16 = 8;
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct PointCloud {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub points: Vec<geometry_msgs::msg::Point32>,
    #[allow(missing_docs)]
    pub channels: Vec<ChannelFloat32>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct PointCloud2 {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub height: u32,
    #[allow(missing_docs)]
    pub width: u32,
    #[allow(missing_docs)]
    pub fields: Vec<PointField>,
    #[allow(missing_docs)]
    pub is_bigendian: bool,
    #[allow(missing_docs)]
    pub point_step: u32,
    #[allow(missing_docs)]
    pub row_step: u32,
    #[allow(missing_docs)]
    pub data: Vec<u8>,
    #[allow(missing_docs)]
    pub is_dense: bool,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct PointField {
    #[allow(missing_docs)]
    pub name: std::string::String,
    #[allow(missing_docs)]
    pub offset: u32,
    #[allow(missing_docs)]
    pub datatype: u8,
    #[allow(missing_docs)]
    pub count: u32,
}

impl PointField {
    pub const INT8: u8 = 1;
    pub const UINT8: u8 = 2;
    pub const INT16: u8 = 3;
    pub const UINT16: u8 = 4;
    pub const INT32: u8 = 5;
    pub const UINT32: u8 = 6;
    pub const FLOAT32: u8 = 7;
    pub const FLOAT64: u8 = 8;
    pub const INT64: u8 = 9;
    pub const UINT64: u8 = 10;
    pub const BOOL: u8 = 11;
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Range {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub radiation_type: u8,
    #[allow(missing_docs)]
    pub field_of_view: f32,
    #[allow(missing_docs)]
    pub min_range: f32,
    #[allow(missing_docs)]
    pub max_range: f32,
    #[allow(missing_docs)]
    pub range: f32,
    #[allow(missing_docs)]
    pub variance: f32,
}

impl Range {
    pub const ULTRASOUND: u8 = 0;
    pub const INFRARED: u8 = 1;
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct RegionOfInterest {
    #[allow(missing_docs)]
    pub x_offset: u32,
    #[allow(missing_docs)]
    pub y_offset: u32,
    #[allow(missing_docs)]
    pub height: u32,
    #[allow(missing_docs)]
    pub width: u32,
    #[allow(missing_docs)]
    pub do_rectify: bool,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct RelativeHumidity {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub relative_humidity: f64,
    #[allow(missing_docs)]
    pub variance: f64,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Temperature {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub temperature: f64,
    #[allow(missing_docs)]
    pub variance: f64,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct TimeReference {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub time_ref: builtin_interfaces::msg::Time,
    #[allow(missing_docs)]
    pub source: std::string::String,
}
