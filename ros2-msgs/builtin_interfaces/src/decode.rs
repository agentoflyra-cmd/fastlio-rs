#[allow(unused_imports)]
use crate::msg::*;
#[allow(unused_imports)]
use crate::srv::*;
#[allow(unused_imports)]
pub use cdr_runtime::{
    CdrDecoder, CdrError, CdrResult, DecodeCdr, Endianness, WChar16, WChar32, decode_from_bytes,
};

impl DecodeCdr for Duration {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            sec: <i32 as DecodeCdr>::decode_cdr(decoder)?,
            nanosec: <u32 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Time {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            sec: <i32 as DecodeCdr>::decode_cdr(decoder)?,
            nanosec: <u32 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}
