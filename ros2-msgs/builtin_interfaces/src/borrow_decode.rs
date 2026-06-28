#[allow(unused_imports)]
use crate::borrowed::*;
#[allow(unused_imports)]
pub use cdr_runtime::{
    BorrowDecodeCdr, CdrDecoder, CdrError, CdrResult, DecodeCdr, Endianness, WChar16, WChar32,
    borrow_decode_from_bytes,
};

impl<'a> BorrowDecodeCdr<'a> for Duration<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            sec: <i32 as DecodeCdr>::decode_cdr(decoder)?,
            nanosec: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Time<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            sec: <i32 as DecodeCdr>::decode_cdr(decoder)?,
            nanosec: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}
