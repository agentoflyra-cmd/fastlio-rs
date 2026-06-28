#[allow(unused_imports)]
use crate::msg::*;
#[allow(unused_imports)]
use crate::srv::*;
#[allow(unused_imports)]
pub use cdr_runtime::{
    CdrDecoder, CdrError, CdrResult, DecodeCdr, Endianness, WChar16, WChar32, decode_from_bytes,
};

impl DecodeCdr for Accel {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            linear: <Vector3 as DecodeCdr>::decode_cdr(decoder)?,
            angular: <Vector3 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for AccelStamped {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            accel: <Accel as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for AccelWithCovariance {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            accel: <Accel as DecodeCdr>::decode_cdr(decoder)?,
            covariance: decoder.read_f64_array::<36>()?,
        })
    }
}

impl DecodeCdr for AccelWithCovarianceStamped {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            accel: <AccelWithCovariance as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Inertia {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            m: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            com: <Vector3 as DecodeCdr>::decode_cdr(decoder)?,
            ixx: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            ixy: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            ixz: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            iyy: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            iyz: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            izz: <f64 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for InertiaStamped {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            inertia: <Inertia as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Point {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            x: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            y: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            z: <f64 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Point32 {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            x: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            y: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            z: <f32 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for PointStamped {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            point: <Point as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Polygon {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            points: decoder.read_seq::<Point32>()?,
        })
    }
}

impl DecodeCdr for PolygonInstance {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            polygon: <Polygon as DecodeCdr>::decode_cdr(decoder)?,
            id: <i64 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for PolygonInstanceStamped {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            polygon: <PolygonInstance as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for PolygonStamped {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            polygon: <Polygon as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Pose {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            position: <Point as DecodeCdr>::decode_cdr(decoder)?,
            orientation: <Quaternion as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for PoseArray {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            poses: decoder.read_seq::<Pose>()?,
        })
    }
}

impl DecodeCdr for PoseStamped {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            pose: <Pose as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for PoseWithCovariance {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            pose: <Pose as DecodeCdr>::decode_cdr(decoder)?,
            covariance: decoder.read_f64_array::<36>()?,
        })
    }
}

impl DecodeCdr for PoseWithCovarianceStamped {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            pose: <PoseWithCovariance as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Quaternion {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            x: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            y: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            z: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            w: <f64 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for QuaternionStamped {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            quaternion: <Quaternion as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Transform {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            translation: <Vector3 as DecodeCdr>::decode_cdr(decoder)?,
            rotation: <Quaternion as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for TransformStamped {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            child_frame_id: <std::string::String as DecodeCdr>::decode_cdr(decoder)?,
            transform: <Transform as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Twist {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            linear: <Vector3 as DecodeCdr>::decode_cdr(decoder)?,
            angular: <Vector3 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for TwistStamped {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            twist: <Twist as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for TwistWithCovariance {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            twist: <Twist as DecodeCdr>::decode_cdr(decoder)?,
            covariance: decoder.read_f64_array::<36>()?,
        })
    }
}

impl DecodeCdr for TwistWithCovarianceStamped {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            twist: <TwistWithCovariance as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Vector3 {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            x: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            y: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            z: <f64 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Vector3Stamped {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            vector: <Vector3 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for VelocityStamped {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            body_frame_id: <std::string::String as DecodeCdr>::decode_cdr(decoder)?,
            reference_frame_id: <std::string::String as DecodeCdr>::decode_cdr(decoder)?,
            velocity: <Twist as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for VelocityWithCovarianceStamped {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            body_frame_id: <std::string::String as DecodeCdr>::decode_cdr(decoder)?,
            reference_frame_id: <std::string::String as DecodeCdr>::decode_cdr(decoder)?,
            velocity: <TwistWithCovariance as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Wrench {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            force: <Vector3 as DecodeCdr>::decode_cdr(decoder)?,
            torque: <Vector3 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for WrenchStamped {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            wrench: <Wrench as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}
