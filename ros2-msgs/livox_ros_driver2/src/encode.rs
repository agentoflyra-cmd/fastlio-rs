#[allow(unused_imports)]
use crate::msg::*;
#[allow(unused_imports)]
use crate::srv::*;
#[allow(unused_imports)]
pub use cdr_runtime::{
    CdrEncoder, CdrError, CdrResult, EncodeCdr, Endianness, WChar16, WChar32, encode_to_vec,
};

impl EncodeCdr for CustomMsg {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std_msgs::msg::Header as EncodeCdr>::encode_cdr(&self.header, encoder)?;
        <u64 as EncodeCdr>::encode_cdr(&self.timebase, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.point_num, encoder)?;
        <u8 as EncodeCdr>::encode_cdr(&self.lidar_id, encoder)?;
        encoder.write_byte_array(&self.rsvd)?;
        encoder.write_seq::<CustomPoint>(&self.points)?;
        Ok(())
    }
}

impl EncodeCdr for CustomPoint {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <u32 as EncodeCdr>::encode_cdr(&self.offset_time, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.x, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.y, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.z, encoder)?;
        <u8 as EncodeCdr>::encode_cdr(&self.reflectivity, encoder)?;
        <u8 as EncodeCdr>::encode_cdr(&self.tag, encoder)?;
        <u8 as EncodeCdr>::encode_cdr(&self.line, encoder)?;
        Ok(())
    }
}
