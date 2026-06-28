#[allow(unused_imports)]
use crate::msg::*;
#[allow(unused_imports)]
use crate::srv::*;
#[allow(unused_imports)]
pub use cdr_runtime::{
    CdrEncoder, CdrError, CdrResult, EncodeCdr, Endianness, WChar16, WChar32, encode_to_vec,
};

impl EncodeCdr for Duration {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <i32 as EncodeCdr>::encode_cdr(&self.sec, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.nanosec, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Time {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <i32 as EncodeCdr>::encode_cdr(&self.sec, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.nanosec, encoder)?;
        Ok(())
    }
}
