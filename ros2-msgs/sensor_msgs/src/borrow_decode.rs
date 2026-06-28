#[allow(unused_imports)]
use crate::borrowed::*;
#[allow(unused_imports)]
pub use cdr_runtime::{
    BorrowDecodeCdr, CdrDecoder, CdrError, CdrResult, DecodeCdr, Endianness, WChar16, WChar32,
    borrow_decode_from_bytes,
};

impl<'a> BorrowDecodeCdr<'a> for BatteryState<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
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
            cell_voltage: decoder.read_primitive_seq_borrowed::<f32>()?,
            cell_temperature: decoder.read_primitive_seq_borrowed::<f32>()?,
            location: decoder.read_str()?,
            serial_number: decoder.read_str()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for CameraInfo<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            height: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            width: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            distortion_model: decoder.read_str()?,
            d: decoder.read_primitive_seq_borrowed::<f64>()?,
            k: decoder.read_primitive_array_borrowed::<f64, 9>()?,
            r: decoder.read_primitive_array_borrowed::<f64, 9>()?,
            p: decoder.read_primitive_array_borrowed::<f64, 12>()?,
            binning_x: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            binning_y: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            roi: <RegionOfInterest<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for ChannelFloat32<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            name: decoder.read_str()?,
            values: decoder.read_primitive_seq_borrowed::<f32>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for CompressedImage<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            format: decoder.read_str()?,
            data: decoder.read_octet_slice()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for FluidPressure<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            fluid_pressure: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            variance: <f64 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Illuminance<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            illuminance: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            variance: <f64 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Image<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            height: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            width: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            encoding: decoder.read_str()?,
            is_bigendian: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            step: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            data: decoder.read_octet_slice()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Imu<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            orientation:
                <geometry_msgs::borrowed::Quaternion<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                    decoder,
                )?,
            orientation_covariance: decoder.read_primitive_array_borrowed::<f64, 9>()?,
            angular_velocity:
                <geometry_msgs::borrowed::Vector3<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                    decoder,
                )?,
            angular_velocity_covariance: decoder.read_primitive_array_borrowed::<f64, 9>()?,
            linear_acceleration:
                <geometry_msgs::borrowed::Vector3<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                    decoder,
                )?,
            linear_acceleration_covariance: decoder.read_primitive_array_borrowed::<f64, 9>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for JointState<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            name: decoder.read_seq::<std::string::String>()?,
            position: decoder.read_primitive_seq_borrowed::<f64>()?,
            velocity: decoder.read_primitive_seq_borrowed::<f64>()?,
            effort: decoder.read_primitive_seq_borrowed::<f64>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Joy<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            axes: decoder.read_primitive_seq_borrowed::<f32>()?,
            buttons: decoder.read_primitive_seq_borrowed::<i32>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for JoyFeedback<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            r#type: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            id: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            intensity: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for JoyFeedbackArray<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            array: decoder.read_borrow_seq::<JoyFeedback<'a>>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for LaserEcho<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            echoes: decoder.read_primitive_seq_borrowed::<f32>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for LaserScan<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            angle_min: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            angle_max: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            angle_increment: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            time_increment: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            scan_time: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            range_min: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            range_max: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            ranges: decoder.read_primitive_seq_borrowed::<f32>()?,
            intensities: decoder.read_primitive_seq_borrowed::<f32>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for MagneticField<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            magnetic_field:
                <geometry_msgs::borrowed::Vector3<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                    decoder,
                )?,
            magnetic_field_covariance: decoder.read_primitive_array_borrowed::<f64, 9>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for MultiDOFJointState<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            joint_names: decoder.read_seq::<std::string::String>()?,
            transforms: decoder.read_borrow_seq::<geometry_msgs::borrowed::Transform<'a>>()?,
            twist: decoder.read_borrow_seq::<geometry_msgs::borrowed::Twist<'a>>()?,
            wrench: decoder.read_borrow_seq::<geometry_msgs::borrowed::Wrench<'a>>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for MultiEchoLaserScan<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            angle_min: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            angle_max: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            angle_increment: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            time_increment: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            scan_time: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            range_min: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            range_max: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            ranges: decoder.read_borrow_seq::<LaserEcho<'a>>()?,
            intensities: decoder.read_borrow_seq::<LaserEcho<'a>>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for NavSatFix<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            status: <NavSatStatus<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            latitude: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            longitude: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            altitude: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            position_covariance: decoder.read_primitive_array_borrowed::<f64, 9>()?,
            position_covariance_type: <u8 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for NavSatStatus<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            status: <i8 as DecodeCdr>::decode_cdr(decoder)?,
            service: <u16 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for PointCloud<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            points: decoder.read_borrow_seq::<geometry_msgs::borrowed::Point32<'a>>()?,
            channels: decoder.read_borrow_seq::<ChannelFloat32<'a>>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for PointCloud2<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            height: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            width: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            fields: decoder.read_borrow_seq::<PointField<'a>>()?,
            is_bigendian: <bool as DecodeCdr>::decode_cdr(decoder)?,
            point_step: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            row_step: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            data: decoder.read_octet_slice()?,
            is_dense: <bool as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for PointField<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            name: decoder.read_str()?,
            offset: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            datatype: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            count: <u32 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Range<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            radiation_type: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            field_of_view: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            min_range: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            max_range: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            range: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            variance: <f32 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for RegionOfInterest<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            x_offset: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            y_offset: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            height: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            width: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            do_rectify: <bool as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for RelativeHumidity<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            relative_humidity: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            variance: <f64 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Temperature<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            temperature: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            variance: <f64 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for TimeReference<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            time_ref:
                <builtin_interfaces::borrowed::Time<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                    decoder,
                )?,
            source: decoder.read_str()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for SetCameraInfoRequest<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            camera_info: <CameraInfo<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for SetCameraInfoResponse<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            success: <bool as DecodeCdr>::decode_cdr(decoder)?,
            status_message: decoder.read_str()?,
        })
    }
}
