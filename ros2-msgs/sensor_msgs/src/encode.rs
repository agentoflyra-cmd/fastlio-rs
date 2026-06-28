#[allow(unused_imports)]
use crate::msg::*;
#[allow(unused_imports)]
use crate::srv::*;
#[allow(unused_imports)]
pub use cdr_runtime::{
    CdrEncoder, CdrError, CdrResult, EncodeCdr, Endianness, WChar16, WChar32, encode_to_vec,
};

impl EncodeCdr for BatteryState {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.voltage, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.temperature, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.current, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.charge, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.capacity, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.design_capacity, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.percentage, encoder)?;
        <u8 as EncodeCdr>::encode_cdr(&self.power_supply_status, encoder)?;
        <u8 as EncodeCdr>::encode_cdr(&self.power_supply_health, encoder)?;
        <u8 as EncodeCdr>::encode_cdr(&self.power_supply_technology, encoder)?;
        <bool as EncodeCdr>::encode_cdr(&self.present, encoder)?;
        encoder.write_f32_seq(&self.cell_voltage)?;
        encoder.write_f32_seq(&self.cell_temperature)?;
        <std::string::String as EncodeCdr>::encode_cdr(&self.location, encoder)?;
        <std::string::String as EncodeCdr>::encode_cdr(&self.serial_number, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for CameraInfo {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.height, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.width, encoder)?;
        <std::string::String as EncodeCdr>::encode_cdr(&self.distortion_model, encoder)?;
        encoder.write_f64_seq(&self.d)?;
        encoder.write_f64_array(&self.k)?;
        encoder.write_f64_array(&self.r)?;
        encoder.write_f64_array(&self.p)?;
        <u32 as EncodeCdr>::encode_cdr(&self.binning_x, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.binning_y, encoder)?;
        <RegionOfInterest as EncodeCdr>::encode_cdr(&self.roi, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for ChannelFloat32 {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std::string::String as EncodeCdr>::encode_cdr(&self.name, encoder)?;
        encoder.write_f32_seq(&self.values)?;
        Ok(())
    }
}

impl EncodeCdr for CompressedImage {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <std::string::String as EncodeCdr>::encode_cdr(&self.format, encoder)?;
        encoder.write_octet_bytes(&self.data)?;
        Ok(())
    }
}

impl EncodeCdr for FluidPressure {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.fluid_pressure, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.variance, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Illuminance {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.illuminance, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.variance, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Image {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.height, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.width, encoder)?;
        <std::string::String as EncodeCdr>::encode_cdr(&self.encoding, encoder)?;
        <u8 as EncodeCdr>::encode_cdr(&self.is_bigendian, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.step, encoder)?;
        encoder.write_octet_bytes(&self.data)?;
        Ok(())
    }
}

impl EncodeCdr for Imu {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <geometry_msgs::msg::Quaternion as EncodeCdr>::encode_cdr(&self.orientation, encoder)?;
        encoder.write_f64_array(&self.orientation_covariance)?;
        <geometry_msgs::msg::Vector3 as EncodeCdr>::encode_cdr(&self.angular_velocity, encoder)?;
        encoder.write_f64_array(&self.angular_velocity_covariance)?;
        <geometry_msgs::msg::Vector3 as EncodeCdr>::encode_cdr(&self.linear_acceleration, encoder)?;
        encoder.write_f64_array(&self.linear_acceleration_covariance)?;
        Ok(())
    }
}

impl EncodeCdr for JointState {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        encoder.write_seq::<std::string::String>(&self.name)?;
        encoder.write_f64_seq(&self.position)?;
        encoder.write_f64_seq(&self.velocity)?;
        encoder.write_f64_seq(&self.effort)?;
        Ok(())
    }
}

impl EncodeCdr for Joy {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        encoder.write_f32_seq(&self.axes)?;
        encoder.write_i32_seq(&self.buttons)?;
        Ok(())
    }
}

impl EncodeCdr for JoyFeedback {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <u8 as EncodeCdr>::encode_cdr(&self.r#type, encoder)?;
        <u8 as EncodeCdr>::encode_cdr(&self.id, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.intensity, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for JoyFeedbackArray {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        encoder.write_seq::<JoyFeedback>(&self.array)?;
        Ok(())
    }
}

impl EncodeCdr for LaserEcho {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        encoder.write_f32_seq(&self.echoes)?;
        Ok(())
    }
}

impl EncodeCdr for LaserScan {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.angle_min, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.angle_max, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.angle_increment, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.time_increment, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.scan_time, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.range_min, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.range_max, encoder)?;
        encoder.write_f32_seq(&self.ranges)?;
        encoder.write_f32_seq(&self.intensities)?;
        Ok(())
    }
}

impl EncodeCdr for MagneticField {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <geometry_msgs::msg::Vector3 as EncodeCdr>::encode_cdr(&self.magnetic_field, encoder)?;
        encoder.write_f64_array(&self.magnetic_field_covariance)?;
        Ok(())
    }
}

impl EncodeCdr for MultiDOFJointState {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        encoder.write_seq::<std::string::String>(&self.joint_names)?;
        encoder.write_seq::<geometry_msgs::msg::Transform>(&self.transforms)?;
        encoder.write_seq::<geometry_msgs::msg::Twist>(&self.twist)?;
        encoder.write_seq::<geometry_msgs::msg::Wrench>(&self.wrench)?;
        Ok(())
    }
}

impl EncodeCdr for MultiEchoLaserScan {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.angle_min, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.angle_max, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.angle_increment, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.time_increment, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.scan_time, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.range_min, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.range_max, encoder)?;
        encoder.write_seq::<LaserEcho>(&self.ranges)?;
        encoder.write_seq::<LaserEcho>(&self.intensities)?;
        Ok(())
    }
}

impl EncodeCdr for NavSatFix {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <NavSatStatus as EncodeCdr>::encode_cdr(&self.status, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.latitude, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.longitude, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.altitude, encoder)?;
        encoder.write_f64_array(&self.position_covariance)?;
        <u8 as EncodeCdr>::encode_cdr(&self.position_covariance_type, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for NavSatStatus {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <i8 as EncodeCdr>::encode_cdr(&self.status, encoder)?;
        <u16 as EncodeCdr>::encode_cdr(&self.service, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for PointCloud {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        encoder.write_seq::<geometry_msgs::msg::Point32>(&self.points)?;
        encoder.write_seq::<ChannelFloat32>(&self.channels)?;
        Ok(())
    }
}

impl EncodeCdr for PointCloud2 {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.height, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.width, encoder)?;
        encoder.write_seq::<PointField>(&self.fields)?;
        <bool as EncodeCdr>::encode_cdr(&self.is_bigendian, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.point_step, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.row_step, encoder)?;
        encoder.write_octet_bytes(&self.data)?;
        <bool as EncodeCdr>::encode_cdr(&self.is_dense, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for PointField {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std::string::String as EncodeCdr>::encode_cdr(&self.name, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.offset, encoder)?;
        <u8 as EncodeCdr>::encode_cdr(&self.datatype, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.count, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Range {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <u8 as EncodeCdr>::encode_cdr(&self.radiation_type, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.field_of_view, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.min_range, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.max_range, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.range, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.variance, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for RegionOfInterest {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <u32 as EncodeCdr>::encode_cdr(&self.x_offset, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.y_offset, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.height, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.width, encoder)?;
        <bool as EncodeCdr>::encode_cdr(&self.do_rectify, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for RelativeHumidity {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.relative_humidity, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.variance, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Temperature {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.temperature, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.variance, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for TimeReference {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <builtin_interfaces::msg::Time as EncodeCdr>::encode_cdr(&self.time_ref, encoder)?;
        <std::string::String as EncodeCdr>::encode_cdr(&self.source, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for SetCameraInfoRequest {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <CameraInfo as EncodeCdr>::encode_cdr(&self.camera_info, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for SetCameraInfoResponse {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <bool as EncodeCdr>::encode_cdr(&self.success, encoder)?;
        <std::string::String as EncodeCdr>::encode_cdr(&self.status_message, encoder)?;
        Ok(())
    }
}
