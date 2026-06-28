#[allow(unused_imports)]
use crate::msg::*;
#[allow(unused_imports)]
use crate::srv::*;
#[allow(unused_imports)]
pub use cdr_runtime::{
    CdrDecoder, CdrError, CdrResult, DecodeCdr, Endianness, WChar16, WChar32, decode_from_bytes,
};

impl DecodeCdr for BatteryState {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            voltage: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            temperature: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            current: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            charge: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            capacity: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            design_capacity: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            percentage: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            power_supply_status: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            power_supply_health: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            power_supply_technology: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            present: <bool as DecodeCdr>::decode_cdr(decoder)?,
            cell_voltage: decoder.read_f32_seq()?,
            cell_temperature: decoder.read_f32_seq()?,
            location: <std::string::String as DecodeCdr>::decode_cdr(decoder)?,
            serial_number: <std::string::String as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for CameraInfo {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            height: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            width: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            distortion_model: <std::string::String as DecodeCdr>::decode_cdr(decoder)?,
            d: decoder.read_f64_seq()?,
            k: decoder.read_f64_array::<9>()?,
            r: decoder.read_f64_array::<9>()?,
            p: decoder.read_f64_array::<12>()?,
            binning_x: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            binning_y: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            roi: <RegionOfInterest as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for ChannelFloat32 {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            name: <std::string::String as DecodeCdr>::decode_cdr(decoder)?,
            values: decoder.read_f32_seq()?,
        })
    }
}

impl DecodeCdr for CompressedImage {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            format: <std::string::String as DecodeCdr>::decode_cdr(decoder)?,
            data: decoder.read_octet_seq()?,
        })
    }
}

impl DecodeCdr for FluidPressure {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            fluid_pressure: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            variance: <f64 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Illuminance {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            illuminance: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            variance: <f64 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Image {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            height: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            width: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            encoding: <std::string::String as DecodeCdr>::decode_cdr(decoder)?,
            is_bigendian: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            step: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            data: decoder.read_octet_seq()?,
        })
    }
}

impl DecodeCdr for Imu {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            orientation: <geometry_msgs::msg::Quaternion as DecodeCdr>::decode_cdr(decoder)?,
            orientation_covariance: decoder.read_f64_array::<9>()?,
            angular_velocity: <geometry_msgs::msg::Vector3 as DecodeCdr>::decode_cdr(decoder)?,
            angular_velocity_covariance: decoder.read_f64_array::<9>()?,
            linear_acceleration: <geometry_msgs::msg::Vector3 as DecodeCdr>::decode_cdr(decoder)?,
            linear_acceleration_covariance: decoder.read_f64_array::<9>()?,
        })
    }
}

impl DecodeCdr for JointState {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            name: decoder.read_seq::<std::string::String>()?,
            position: decoder.read_f64_seq()?,
            velocity: decoder.read_f64_seq()?,
            effort: decoder.read_f64_seq()?,
        })
    }
}

impl DecodeCdr for Joy {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            axes: decoder.read_f32_seq()?,
            buttons: decoder.read_i32_seq()?,
        })
    }
}

impl DecodeCdr for JoyFeedback {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            r#type: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            id: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            intensity: <f32 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for JoyFeedbackArray {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            array: decoder.read_seq::<JoyFeedback>()?,
        })
    }
}

impl DecodeCdr for LaserEcho {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            echoes: decoder.read_f32_seq()?,
        })
    }
}

impl DecodeCdr for LaserScan {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            angle_min: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            angle_max: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            angle_increment: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            time_increment: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            scan_time: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            range_min: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            range_max: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            ranges: decoder.read_f32_seq()?,
            intensities: decoder.read_f32_seq()?,
        })
    }
}

impl DecodeCdr for MagneticField {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            magnetic_field: <geometry_msgs::msg::Vector3 as DecodeCdr>::decode_cdr(decoder)?,
            magnetic_field_covariance: decoder.read_f64_array::<9>()?,
        })
    }
}

impl DecodeCdr for MultiDOFJointState {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            joint_names: decoder.read_seq::<std::string::String>()?,
            transforms: decoder.read_seq::<geometry_msgs::msg::Transform>()?,
            twist: decoder.read_seq::<geometry_msgs::msg::Twist>()?,
            wrench: decoder.read_seq::<geometry_msgs::msg::Wrench>()?,
        })
    }
}

impl DecodeCdr for MultiEchoLaserScan {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            angle_min: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            angle_max: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            angle_increment: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            time_increment: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            scan_time: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            range_min: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            range_max: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            ranges: decoder.read_seq::<LaserEcho>()?,
            intensities: decoder.read_seq::<LaserEcho>()?,
        })
    }
}

impl DecodeCdr for NavSatFix {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            status: <NavSatStatus as DecodeCdr>::decode_cdr(decoder)?,
            latitude: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            longitude: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            altitude: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            position_covariance: decoder.read_f64_array::<9>()?,
            position_covariance_type: <u8 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for NavSatStatus {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            status: <i8 as DecodeCdr>::decode_cdr(decoder)?,
            service: <u16 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for PointCloud {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            points: decoder.read_seq::<geometry_msgs::msg::Point32>()?,
            channels: decoder.read_seq::<ChannelFloat32>()?,
        })
    }
}

impl DecodeCdr for PointCloud2 {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            height: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            width: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            fields: decoder.read_seq::<PointField>()?,
            is_bigendian: <bool as DecodeCdr>::decode_cdr(decoder)?,
            point_step: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            row_step: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            data: decoder.read_octet_seq()?,
            is_dense: <bool as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for PointField {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            name: <std::string::String as DecodeCdr>::decode_cdr(decoder)?,
            offset: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            datatype: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            count: <u32 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Range {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            radiation_type: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            field_of_view: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            min_range: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            max_range: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            range: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            variance: <f32 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for RegionOfInterest {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            x_offset: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            y_offset: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            height: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            width: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            do_rectify: <bool as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for RelativeHumidity {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            relative_humidity: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            variance: <f64 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Temperature {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            temperature: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            variance: <f64 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for TimeReference {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            time_ref: <builtin_interfaces::msg::Time as DecodeCdr>::decode_cdr(decoder)?,
            source: <std::string::String as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for SetCameraInfoRequest {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            camera_info: <CameraInfo as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for SetCameraInfoResponse {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            success: <bool as DecodeCdr>::decode_cdr(decoder)?,
            status_message: <std::string::String as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}
