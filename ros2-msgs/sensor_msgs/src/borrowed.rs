#![allow(unused_imports)]
use crate::msg::*;
use crate::srv::*;

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct BatteryState<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
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
    pub cell_voltage: cdr_runtime::PrimitiveSeq<'a, f32>,
    #[allow(missing_docs)]
    pub cell_temperature: cdr_runtime::PrimitiveSeq<'a, f32>,
    #[allow(missing_docs)]
    pub location: &'a str,
    #[allow(missing_docs)]
    pub serial_number: &'a str,
}

impl<'a> BatteryState<'a> {
    pub fn to_owned(&self) -> crate::msg::BatteryState {
        crate::msg::BatteryState {
            header: self.header.to_owned(),
            voltage: self.voltage,
            temperature: self.temperature,
            current: self.current,
            charge: self.charge,
            capacity: self.capacity,
            design_capacity: self.design_capacity,
            percentage: self.percentage,
            power_supply_status: self.power_supply_status,
            power_supply_health: self.power_supply_health,
            power_supply_technology: self.power_supply_technology,
            present: self.present,
            cell_voltage: self.cell_voltage.to_vec(),
            cell_temperature: self.cell_temperature.to_vec(),
            location: self.location.to_string(),
            serial_number: self.serial_number.to_string(),
        }
    }
}

impl<'a> BatteryState<'a> {
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
pub struct CameraInfo<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub height: u32,
    #[allow(missing_docs)]
    pub width: u32,
    #[allow(missing_docs)]
    pub distortion_model: &'a str,
    #[allow(missing_docs)]
    pub d: cdr_runtime::PrimitiveSeq<'a, f64>,
    #[allow(missing_docs)]
    pub k: cdr_runtime::PrimitiveArray<'a, f64, 9>,
    #[allow(missing_docs)]
    pub r: cdr_runtime::PrimitiveArray<'a, f64, 9>,
    #[allow(missing_docs)]
    pub p: cdr_runtime::PrimitiveArray<'a, f64, 12>,
    #[allow(missing_docs)]
    pub binning_x: u32,
    #[allow(missing_docs)]
    pub binning_y: u32,
    #[allow(missing_docs)]
    pub roi: RegionOfInterest<'a>,
}

impl<'a> CameraInfo<'a> {
    pub fn to_owned(&self) -> crate::msg::CameraInfo {
        crate::msg::CameraInfo {
            header: self.header.to_owned(),
            height: self.height,
            width: self.width,
            distortion_model: self.distortion_model.to_string(),
            d: self.d.to_vec(),
            k: self.k.to_array(),
            r: self.r.to_array(),
            p: self.p.to_array(),
            binning_x: self.binning_x,
            binning_y: self.binning_y,
            roi: self.roi.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ChannelFloat32<'a> {
    #[allow(missing_docs)]
    pub name: &'a str,
    #[allow(missing_docs)]
    pub values: cdr_runtime::PrimitiveSeq<'a, f32>,
}

impl<'a> ChannelFloat32<'a> {
    pub fn to_owned(&self) -> crate::msg::ChannelFloat32 {
        crate::msg::ChannelFloat32 {
            name: self.name.to_string(),
            values: self.values.to_vec(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CompressedImage<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub format: &'a str,
    #[allow(missing_docs)]
    pub data: &'a [u8],
}

impl<'a> CompressedImage<'a> {
    pub fn to_owned(&self) -> crate::msg::CompressedImage {
        crate::msg::CompressedImage {
            header: self.header.to_owned(),
            format: self.format.to_string(),
            data: self.data.to_vec(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FluidPressure<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub fluid_pressure: f64,
    #[allow(missing_docs)]
    pub variance: f64,
}

impl<'a> FluidPressure<'a> {
    pub fn to_owned(&self) -> crate::msg::FluidPressure {
        crate::msg::FluidPressure {
            header: self.header.to_owned(),
            fluid_pressure: self.fluid_pressure,
            variance: self.variance,
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Illuminance<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub illuminance: f64,
    #[allow(missing_docs)]
    pub variance: f64,
}

impl<'a> Illuminance<'a> {
    pub fn to_owned(&self) -> crate::msg::Illuminance {
        crate::msg::Illuminance {
            header: self.header.to_owned(),
            illuminance: self.illuminance,
            variance: self.variance,
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Image<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub height: u32,
    #[allow(missing_docs)]
    pub width: u32,
    #[allow(missing_docs)]
    pub encoding: &'a str,
    #[allow(missing_docs)]
    pub is_bigendian: u8,
    #[allow(missing_docs)]
    pub step: u32,
    #[allow(missing_docs)]
    pub data: &'a [u8],
}

impl<'a> Image<'a> {
    pub fn to_owned(&self) -> crate::msg::Image {
        crate::msg::Image {
            header: self.header.to_owned(),
            height: self.height,
            width: self.width,
            encoding: self.encoding.to_string(),
            is_bigendian: self.is_bigendian,
            step: self.step,
            data: self.data.to_vec(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Imu<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub orientation: geometry_msgs::borrowed::Quaternion<'a>,
    #[allow(missing_docs)]
    pub orientation_covariance: cdr_runtime::PrimitiveArray<'a, f64, 9>,
    #[allow(missing_docs)]
    pub angular_velocity: geometry_msgs::borrowed::Vector3<'a>,
    #[allow(missing_docs)]
    pub angular_velocity_covariance: cdr_runtime::PrimitiveArray<'a, f64, 9>,
    #[allow(missing_docs)]
    pub linear_acceleration: geometry_msgs::borrowed::Vector3<'a>,
    #[allow(missing_docs)]
    pub linear_acceleration_covariance: cdr_runtime::PrimitiveArray<'a, f64, 9>,
}

impl<'a> Imu<'a> {
    pub fn to_owned(&self) -> crate::msg::Imu {
        crate::msg::Imu {
            header: self.header.to_owned(),
            orientation: self.orientation.to_owned(),
            orientation_covariance: self.orientation_covariance.to_array(),
            angular_velocity: self.angular_velocity.to_owned(),
            angular_velocity_covariance: self.angular_velocity_covariance.to_array(),
            linear_acceleration: self.linear_acceleration.to_owned(),
            linear_acceleration_covariance: self.linear_acceleration_covariance.to_array(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointState<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub name: Vec<std::string::String>,
    #[allow(missing_docs)]
    pub position: cdr_runtime::PrimitiveSeq<'a, f64>,
    #[allow(missing_docs)]
    pub velocity: cdr_runtime::PrimitiveSeq<'a, f64>,
    #[allow(missing_docs)]
    pub effort: cdr_runtime::PrimitiveSeq<'a, f64>,
}

impl<'a> JointState<'a> {
    pub fn to_owned(&self) -> crate::msg::JointState {
        crate::msg::JointState {
            header: self.header.to_owned(),
            name: self.name.clone(),
            position: self.position.to_vec(),
            velocity: self.velocity.to_vec(),
            effort: self.effort.to_vec(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Joy<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub axes: cdr_runtime::PrimitiveSeq<'a, f32>,
    #[allow(missing_docs)]
    pub buttons: cdr_runtime::PrimitiveSeq<'a, i32>,
}

impl<'a> Joy<'a> {
    pub fn to_owned(&self) -> crate::msg::Joy {
        crate::msg::Joy {
            header: self.header.to_owned(),
            axes: self.axes.to_vec(),
            buttons: self.buttons.to_vec(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JoyFeedback<'a> {
    #[allow(missing_docs)]
    pub r#type: u8,
    #[allow(missing_docs)]
    pub id: u8,
    #[allow(missing_docs)]
    pub intensity: f32,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> JoyFeedback<'a> {
    pub fn to_owned(&self) -> crate::msg::JoyFeedback {
        crate::msg::JoyFeedback {
            r#type: self.r#type,
            id: self.id,
            intensity: self.intensity,
        }
    }
}

impl<'a> JoyFeedback<'a> {
    pub const TYPE_LED: u8 = 0;
    pub const TYPE_RUMBLE: u8 = 1;
    pub const TYPE_BUZZER: u8 = 2;
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JoyFeedbackArray<'a> {
    #[allow(missing_docs)]
    pub array: Vec<JoyFeedback<'a>>,
}

impl<'a> JoyFeedbackArray<'a> {
    pub fn to_owned(&self) -> crate::msg::JoyFeedbackArray {
        crate::msg::JoyFeedbackArray {
            array: self.array.iter().map(|item| item.to_owned()).collect(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LaserEcho<'a> {
    #[allow(missing_docs)]
    pub echoes: cdr_runtime::PrimitiveSeq<'a, f32>,
}

impl<'a> LaserEcho<'a> {
    pub fn to_owned(&self) -> crate::msg::LaserEcho {
        crate::msg::LaserEcho {
            echoes: self.echoes.to_vec(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LaserScan<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
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
    pub ranges: cdr_runtime::PrimitiveSeq<'a, f32>,
    #[allow(missing_docs)]
    pub intensities: cdr_runtime::PrimitiveSeq<'a, f32>,
}

impl<'a> LaserScan<'a> {
    pub fn to_owned(&self) -> crate::msg::LaserScan {
        crate::msg::LaserScan {
            header: self.header.to_owned(),
            angle_min: self.angle_min,
            angle_max: self.angle_max,
            angle_increment: self.angle_increment,
            time_increment: self.time_increment,
            scan_time: self.scan_time,
            range_min: self.range_min,
            range_max: self.range_max,
            ranges: self.ranges.to_vec(),
            intensities: self.intensities.to_vec(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MagneticField<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub magnetic_field: geometry_msgs::borrowed::Vector3<'a>,
    #[allow(missing_docs)]
    pub magnetic_field_covariance: cdr_runtime::PrimitiveArray<'a, f64, 9>,
}

impl<'a> MagneticField<'a> {
    pub fn to_owned(&self) -> crate::msg::MagneticField {
        crate::msg::MagneticField {
            header: self.header.to_owned(),
            magnetic_field: self.magnetic_field.to_owned(),
            magnetic_field_covariance: self.magnetic_field_covariance.to_array(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MultiDOFJointState<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub joint_names: Vec<std::string::String>,
    #[allow(missing_docs)]
    pub transforms: Vec<geometry_msgs::borrowed::Transform<'a>>,
    #[allow(missing_docs)]
    pub twist: Vec<geometry_msgs::borrowed::Twist<'a>>,
    #[allow(missing_docs)]
    pub wrench: Vec<geometry_msgs::borrowed::Wrench<'a>>,
}

impl<'a> MultiDOFJointState<'a> {
    pub fn to_owned(&self) -> crate::msg::MultiDOFJointState {
        crate::msg::MultiDOFJointState {
            header: self.header.to_owned(),
            joint_names: self.joint_names.clone(),
            transforms: self.transforms.iter().map(|item| item.to_owned()).collect(),
            twist: self.twist.iter().map(|item| item.to_owned()).collect(),
            wrench: self.wrench.iter().map(|item| item.to_owned()).collect(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MultiEchoLaserScan<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
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
    pub ranges: Vec<LaserEcho<'a>>,
    #[allow(missing_docs)]
    pub intensities: Vec<LaserEcho<'a>>,
}

impl<'a> MultiEchoLaserScan<'a> {
    pub fn to_owned(&self) -> crate::msg::MultiEchoLaserScan {
        crate::msg::MultiEchoLaserScan {
            header: self.header.to_owned(),
            angle_min: self.angle_min,
            angle_max: self.angle_max,
            angle_increment: self.angle_increment,
            time_increment: self.time_increment,
            scan_time: self.scan_time,
            range_min: self.range_min,
            range_max: self.range_max,
            ranges: self.ranges.iter().map(|item| item.to_owned()).collect(),
            intensities: self
                .intensities
                .iter()
                .map(|item| item.to_owned())
                .collect(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavSatFix<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub status: NavSatStatus<'a>,
    #[allow(missing_docs)]
    pub latitude: f64,
    #[allow(missing_docs)]
    pub longitude: f64,
    #[allow(missing_docs)]
    pub altitude: f64,
    #[allow(missing_docs)]
    pub position_covariance: cdr_runtime::PrimitiveArray<'a, f64, 9>,
    #[allow(missing_docs)]
    pub position_covariance_type: u8,
}

impl<'a> NavSatFix<'a> {
    pub fn to_owned(&self) -> crate::msg::NavSatFix {
        crate::msg::NavSatFix {
            header: self.header.to_owned(),
            status: self.status.to_owned(),
            latitude: self.latitude,
            longitude: self.longitude,
            altitude: self.altitude,
            position_covariance: self.position_covariance.to_array(),
            position_covariance_type: self.position_covariance_type,
        }
    }
}

impl<'a> NavSatFix<'a> {
    pub const COVARIANCE_TYPE_UNKNOWN: u8 = 0;
    pub const COVARIANCE_TYPE_APPROXIMATED: u8 = 1;
    pub const COVARIANCE_TYPE_DIAGONAL_KNOWN: u8 = 2;
    pub const COVARIANCE_TYPE_KNOWN: u8 = 3;
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavSatStatus<'a> {
    #[allow(missing_docs)]
    pub status: i8,
    #[allow(missing_docs)]
    pub service: u16,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> NavSatStatus<'a> {
    pub fn to_owned(&self) -> crate::msg::NavSatStatus {
        crate::msg::NavSatStatus {
            status: self.status,
            service: self.service,
        }
    }
}

impl<'a> NavSatStatus<'a> {
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
pub struct PointCloud<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub points: Vec<geometry_msgs::borrowed::Point32<'a>>,
    #[allow(missing_docs)]
    pub channels: Vec<ChannelFloat32<'a>>,
}

impl<'a> PointCloud<'a> {
    pub fn to_owned(&self) -> crate::msg::PointCloud {
        crate::msg::PointCloud {
            header: self.header.to_owned(),
            points: self.points.iter().map(|item| item.to_owned()).collect(),
            channels: self.channels.iter().map(|item| item.to_owned()).collect(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PointCloud2<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub height: u32,
    #[allow(missing_docs)]
    pub width: u32,
    #[allow(missing_docs)]
    pub fields: Vec<PointField<'a>>,
    #[allow(missing_docs)]
    pub is_bigendian: bool,
    #[allow(missing_docs)]
    pub point_step: u32,
    #[allow(missing_docs)]
    pub row_step: u32,
    #[allow(missing_docs)]
    pub data: &'a [u8],
    #[allow(missing_docs)]
    pub is_dense: bool,
}

impl<'a> PointCloud2<'a> {
    pub fn to_owned(&self) -> crate::msg::PointCloud2 {
        crate::msg::PointCloud2 {
            header: self.header.to_owned(),
            height: self.height,
            width: self.width,
            fields: self.fields.iter().map(|item| item.to_owned()).collect(),
            is_bigendian: self.is_bigendian,
            point_step: self.point_step,
            row_step: self.row_step,
            data: self.data.to_vec(),
            is_dense: self.is_dense,
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PointField<'a> {
    #[allow(missing_docs)]
    pub name: &'a str,
    #[allow(missing_docs)]
    pub offset: u32,
    #[allow(missing_docs)]
    pub datatype: u8,
    #[allow(missing_docs)]
    pub count: u32,
}

impl<'a> PointField<'a> {
    pub fn to_owned(&self) -> crate::msg::PointField {
        crate::msg::PointField {
            name: self.name.to_string(),
            offset: self.offset,
            datatype: self.datatype,
            count: self.count,
        }
    }
}

impl<'a> PointField<'a> {
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
pub struct Range<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
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

impl<'a> Range<'a> {
    pub fn to_owned(&self) -> crate::msg::Range {
        crate::msg::Range {
            header: self.header.to_owned(),
            radiation_type: self.radiation_type,
            field_of_view: self.field_of_view,
            min_range: self.min_range,
            max_range: self.max_range,
            range: self.range,
            variance: self.variance,
        }
    }
}

impl<'a> Range<'a> {
    pub const ULTRASOUND: u8 = 0;
    pub const INFRARED: u8 = 1;
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RegionOfInterest<'a> {
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
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> RegionOfInterest<'a> {
    pub fn to_owned(&self) -> crate::msg::RegionOfInterest {
        crate::msg::RegionOfInterest {
            x_offset: self.x_offset,
            y_offset: self.y_offset,
            height: self.height,
            width: self.width,
            do_rectify: self.do_rectify,
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RelativeHumidity<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub relative_humidity: f64,
    #[allow(missing_docs)]
    pub variance: f64,
}

impl<'a> RelativeHumidity<'a> {
    pub fn to_owned(&self) -> crate::msg::RelativeHumidity {
        crate::msg::RelativeHumidity {
            header: self.header.to_owned(),
            relative_humidity: self.relative_humidity,
            variance: self.variance,
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Temperature<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub temperature: f64,
    #[allow(missing_docs)]
    pub variance: f64,
}

impl<'a> Temperature<'a> {
    pub fn to_owned(&self) -> crate::msg::Temperature {
        crate::msg::Temperature {
            header: self.header.to_owned(),
            temperature: self.temperature,
            variance: self.variance,
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TimeReference<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub time_ref: builtin_interfaces::borrowed::Time<'a>,
    #[allow(missing_docs)]
    pub source: &'a str,
}

impl<'a> TimeReference<'a> {
    pub fn to_owned(&self) -> crate::msg::TimeReference {
        crate::msg::TimeReference {
            header: self.header.to_owned(),
            time_ref: self.time_ref.to_owned(),
            source: self.source.to_string(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCameraInfoRequest<'a> {
    #[allow(missing_docs)]
    pub camera_info: CameraInfo<'a>,
}

impl<'a> SetCameraInfoRequest<'a> {
    pub fn to_owned(&self) -> crate::srv::SetCameraInfoRequest {
        crate::srv::SetCameraInfoRequest {
            camera_info: self.camera_info.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetCameraInfoResponse<'a> {
    #[allow(missing_docs)]
    pub success: bool,
    #[allow(missing_docs)]
    pub status_message: &'a str,
}

impl<'a> SetCameraInfoResponse<'a> {
    pub fn to_owned(&self) -> crate::srv::SetCameraInfoResponse {
        crate::srv::SetCameraInfoResponse {
            success: self.success,
            status_message: self.status_message.to_string(),
        }
    }
}
