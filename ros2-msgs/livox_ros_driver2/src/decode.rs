#[allow(unused_imports)]
use crate::msg::*;
#[allow(unused_imports)]
use crate::srv::*;
#[allow(unused_imports)]
pub use cdr_runtime::{
    CdrDecoder, CdrError, CdrResult, DecodeCdr, Endianness, WChar16, WChar32, decode_from_bytes,
};

impl DecodeCdr for CustomMsg {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::msg::Header as DecodeCdr>::decode_cdr(decoder)?,
            timebase: <u64 as DecodeCdr>::decode_cdr(decoder)?,
            point_num: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            lidar_id: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            rsvd: decoder.read_byte_array::<3>()?,
            points: decoder.read_seq::<CustomPoint>()?,
        })
    }
}

impl DecodeCdr for CustomPoint {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            offset_time: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            x: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            y: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            z: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            reflectivity: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            tag: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            line: <u8 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}
