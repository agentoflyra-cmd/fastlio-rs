#[allow(unused_imports)]
use crate::msg::*;
#[allow(unused_imports)]
use crate::srv::*;
#[allow(unused_imports)]
pub use cdr_runtime::{
    CdrEncoder, CdrError, CdrResult, EncodeCdr, Endianness, WChar16, WChar32, encode_to_vec,
};

impl EncodeCdr for Accel {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <Vector3 as EncodeCdr>::encode_cdr(&self.linear, encoder)?;
        <Vector3 as EncodeCdr>::encode_cdr(&self.angular, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for AccelStamped {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <Accel as EncodeCdr>::encode_cdr(&self.accel, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for AccelWithCovariance {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <Accel as EncodeCdr>::encode_cdr(&self.accel, encoder)?;
        encoder.write_f64_array(&self.covariance)?;
        Ok(())
    }
}

impl EncodeCdr for AccelWithCovarianceStamped {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <AccelWithCovariance as EncodeCdr>::encode_cdr(&self.accel, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Inertia {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <f64 as EncodeCdr>::encode_cdr(&self.m, encoder)?;
        <Vector3 as EncodeCdr>::encode_cdr(&self.com, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.ixx, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.ixy, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.ixz, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.iyy, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.iyz, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.izz, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for InertiaStamped {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <Inertia as EncodeCdr>::encode_cdr(&self.inertia, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Point {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <f64 as EncodeCdr>::encode_cdr(&self.x, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.y, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.z, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Point32 {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <f32 as EncodeCdr>::encode_cdr(&self.x, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.y, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.z, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for PointStamped {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <Point as EncodeCdr>::encode_cdr(&self.point, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Polygon {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        encoder.write_seq::<Point32>(&self.points)?;
        Ok(())
    }
}

impl EncodeCdr for PolygonInstance {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <Polygon as EncodeCdr>::encode_cdr(&self.polygon, encoder)?;
        <i64 as EncodeCdr>::encode_cdr(&self.id, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for PolygonInstanceStamped {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <PolygonInstance as EncodeCdr>::encode_cdr(&self.polygon, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for PolygonStamped {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <Polygon as EncodeCdr>::encode_cdr(&self.polygon, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Pose {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <Point as EncodeCdr>::encode_cdr(&self.position, encoder)?;
        <Quaternion as EncodeCdr>::encode_cdr(&self.orientation, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for PoseArray {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        encoder.write_seq::<Pose>(&self.poses)?;
        Ok(())
    }
}

impl EncodeCdr for PoseStamped {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <Pose as EncodeCdr>::encode_cdr(&self.pose, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for PoseWithCovariance {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <Pose as EncodeCdr>::encode_cdr(&self.pose, encoder)?;
        encoder.write_f64_array(&self.covariance)?;
        Ok(())
    }
}

impl EncodeCdr for PoseWithCovarianceStamped {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <PoseWithCovariance as EncodeCdr>::encode_cdr(&self.pose, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Quaternion {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <f64 as EncodeCdr>::encode_cdr(&self.x, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.y, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.z, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.w, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for QuaternionStamped {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <Quaternion as EncodeCdr>::encode_cdr(&self.quaternion, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Transform {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <Vector3 as EncodeCdr>::encode_cdr(&self.translation, encoder)?;
        <Quaternion as EncodeCdr>::encode_cdr(&self.rotation, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for TransformStamped {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <std::string::String as EncodeCdr>::encode_cdr(&self.child_frame_id, encoder)?;
        <Transform as EncodeCdr>::encode_cdr(&self.transform, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Twist {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <Vector3 as EncodeCdr>::encode_cdr(&self.linear, encoder)?;
        <Vector3 as EncodeCdr>::encode_cdr(&self.angular, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for TwistStamped {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <Twist as EncodeCdr>::encode_cdr(&self.twist, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for TwistWithCovariance {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <Twist as EncodeCdr>::encode_cdr(&self.twist, encoder)?;
        encoder.write_f64_array(&self.covariance)?;
        Ok(())
    }
}

impl EncodeCdr for TwistWithCovarianceStamped {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <TwistWithCovariance as EncodeCdr>::encode_cdr(&self.twist, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Vector3 {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <f64 as EncodeCdr>::encode_cdr(&self.x, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.y, encoder)?;
        <f64 as EncodeCdr>::encode_cdr(&self.z, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Vector3Stamped {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <Vector3 as EncodeCdr>::encode_cdr(&self.vector, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for VelocityStamped {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <std::string::String as EncodeCdr>::encode_cdr(&self.body_frame_id, encoder)?;
        <std::string::String as EncodeCdr>::encode_cdr(&self.reference_frame_id, encoder)?;
        <Twist as EncodeCdr>::encode_cdr(&self.velocity, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for VelocityWithCovarianceStamped {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <std::string::String as EncodeCdr>::encode_cdr(&self.body_frame_id, encoder)?;
        <std::string::String as EncodeCdr>::encode_cdr(&self.reference_frame_id, encoder)?;
        <TwistWithCovariance as EncodeCdr>::encode_cdr(&self.velocity, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Wrench {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <Vector3 as EncodeCdr>::encode_cdr(&self.force, encoder)?;
        <Vector3 as EncodeCdr>::encode_cdr(&self.torque, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for WrenchStamped {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <Wrench as EncodeCdr>::encode_cdr(&self.wrench, encoder)?;
        Ok(())
    }
}
