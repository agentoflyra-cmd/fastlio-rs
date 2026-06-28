#[allow(unused_imports)]
use crate::borrowed::*;
#[allow(unused_imports)]
pub use cdr_runtime::{
    BorrowDecodeCdr, CdrDecoder, CdrError, CdrResult, DecodeCdr, Endianness, WChar16, WChar32,
    borrow_decode_from_bytes,
};

impl<'a> BorrowDecodeCdr<'a> for Accel<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            linear: <Vector3<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            angular: <Vector3<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for AccelStamped<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            accel: <Accel<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for AccelWithCovariance<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            accel: <Accel<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            covariance: decoder.read_primitive_array_borrowed::<f64, 36>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for AccelWithCovarianceStamped<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            accel: <AccelWithCovariance<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Inertia<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            m: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            com: <Vector3<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            ixx: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            ixy: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            ixz: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            iyy: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            iyz: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            izz: <f64 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for InertiaStamped<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            inertia: <Inertia<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Point<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            x: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            y: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            z: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Point32<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            x: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            y: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            z: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for PointStamped<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            point: <Point<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Polygon<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            points: decoder.read_borrow_seq::<Point32<'a>>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for PolygonInstance<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            polygon: <Polygon<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            id: <i64 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for PolygonInstanceStamped<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            polygon: <PolygonInstance<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for PolygonStamped<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            polygon: <Polygon<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Pose<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            position: <Point<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            orientation: <Quaternion<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for PoseArray<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            poses: decoder.read_borrow_seq::<Pose<'a>>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for PoseStamped<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            pose: <Pose<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for PoseWithCovariance<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            pose: <Pose<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            covariance: decoder.read_primitive_array_borrowed::<f64, 36>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for PoseWithCovarianceStamped<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            pose: <PoseWithCovariance<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Quaternion<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            x: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            y: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            z: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            w: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for QuaternionStamped<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            quaternion: <Quaternion<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Transform<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            translation: <Vector3<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            rotation: <Quaternion<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for TransformStamped<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            child_frame_id: decoder.read_str()?,
            transform: <Transform<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Twist<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            linear: <Vector3<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            angular: <Vector3<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for TwistStamped<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            twist: <Twist<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for TwistWithCovariance<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            twist: <Twist<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            covariance: decoder.read_primitive_array_borrowed::<f64, 36>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for TwistWithCovarianceStamped<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            twist: <TwistWithCovariance<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Vector3<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            x: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            y: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            z: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Vector3Stamped<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            vector: <Vector3<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for VelocityStamped<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            body_frame_id: decoder.read_str()?,
            reference_frame_id: decoder.read_str()?,
            velocity: <Twist<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for VelocityWithCovarianceStamped<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            body_frame_id: decoder.read_str()?,
            reference_frame_id: decoder.read_str()?,
            velocity: <TwistWithCovariance<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Wrench<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            force: <Vector3<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            torque: <Vector3<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for WrenchStamped<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            wrench: <Wrench<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}
