#[allow(unused_imports)]
use crate::borrowed::*;
#[allow(unused_imports)]
pub use cdr_runtime::{
    BorrowDecodeCdr, CdrDecoder, CdrError, CdrResult, DecodeCdr, Endianness, WChar16, WChar32,
    borrow_decode_from_bytes,
};

impl<'a> BorrowDecodeCdr<'a> for CustomMsg<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            header: <std_msgs::borrowed::Header<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                decoder,
            )?,
            timebase: <u64 as DecodeCdr>::decode_cdr(decoder)?,
            point_num: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            lidar_id: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            rsvd: decoder.read_byte_array::<3>()?,
            points: decoder.read_borrow_seq::<CustomPoint<'a>>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for CustomPoint<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            offset_time: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            x: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            y: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            z: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            reflectivity: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            tag: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            line: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}
