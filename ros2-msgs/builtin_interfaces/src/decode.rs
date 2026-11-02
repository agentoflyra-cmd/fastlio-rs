#[allow(unused_imports)]
pub use cdr_runtime::{decode_from_bytes, CdrDecoder, CdrError, CdrResult, DecodeCdr, Endianness, WChar16, WChar32};
#[allow(unused_imports)]
use crate::action::*;
#[allow(unused_imports)]
use crate::msg::*;
#[allow(unused_imports)]
use crate::srv::*;

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