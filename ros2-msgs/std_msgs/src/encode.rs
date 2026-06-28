#[allow(unused_imports)]
use crate::msg::*;
#[allow(unused_imports)]
use crate::srv::*;
#[allow(unused_imports)]
pub use cdr_runtime::{
    CdrEncoder, CdrError, CdrResult, EncodeCdr, Endianness, WChar16, WChar32, encode_to_vec,
};

impl EncodeCdr for Bool {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <bool as EncodeCdr>::encode_cdr(&self.data, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Byte {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <u8 as EncodeCdr>::encode_cdr(&self.data, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for ByteMultiArray {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <MultiArrayLayout as EncodeCdr>::encode_cdr(&self.layout, encoder)?;
        encoder.write_octet_bytes(&self.data)?;
        Ok(())
    }
}

impl EncodeCdr for Char {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <u8 as EncodeCdr>::encode_cdr(&self.data, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for ColorRGBA {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <f32 as EncodeCdr>::encode_cdr(&self.r, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.g, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.b, encoder)?;
        <f32 as EncodeCdr>::encode_cdr(&self.a, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Empty {
    fn encode_cdr(&self, _encoder: &mut CdrEncoder) -> CdrResult<()> {
        Ok(())
    }
}

impl EncodeCdr for Float32 {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <f32 as EncodeCdr>::encode_cdr(&self.data, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Float32MultiArray {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <MultiArrayLayout as EncodeCdr>::encode_cdr(&self.layout, encoder)?;
        encoder.write_f32_seq(&self.data)?;
        Ok(())
    }
}

impl EncodeCdr for Float64 {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <f64 as EncodeCdr>::encode_cdr(&self.data, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Float64MultiArray {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <MultiArrayLayout as EncodeCdr>::encode_cdr(&self.layout, encoder)?;
        encoder.write_f64_seq(&self.data)?;
        Ok(())
    }
}

impl EncodeCdr for Header {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <builtin_interfaces::msg::Time as EncodeCdr>::encode_cdr(&self.stamp, encoder)?;
        <std::string::String as EncodeCdr>::encode_cdr(&self.frame_id, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Int16 {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <i16 as EncodeCdr>::encode_cdr(&self.data, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Int16MultiArray {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <MultiArrayLayout as EncodeCdr>::encode_cdr(&self.layout, encoder)?;
        encoder.write_i16_seq(&self.data)?;
        Ok(())
    }
}

impl EncodeCdr for Int32 {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <i32 as EncodeCdr>::encode_cdr(&self.data, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Int32MultiArray {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <MultiArrayLayout as EncodeCdr>::encode_cdr(&self.layout, encoder)?;
        encoder.write_i32_seq(&self.data)?;
        Ok(())
    }
}

impl EncodeCdr for Int64 {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <i64 as EncodeCdr>::encode_cdr(&self.data, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Int64MultiArray {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <MultiArrayLayout as EncodeCdr>::encode_cdr(&self.layout, encoder)?;
        encoder.write_i64_seq(&self.data)?;
        Ok(())
    }
}

impl EncodeCdr for Int8 {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <i8 as EncodeCdr>::encode_cdr(&self.data, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for Int8MultiArray {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <MultiArrayLayout as EncodeCdr>::encode_cdr(&self.layout, encoder)?;
        encoder.write_seq::<i8>(&self.data)?;
        Ok(())
    }
}

impl EncodeCdr for MultiArrayDimension {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std::string::String as EncodeCdr>::encode_cdr(&self.label, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.size, encoder)?;
        <u32 as EncodeCdr>::encode_cdr(&self.stride, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for MultiArrayLayout {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        encoder.write_seq::<MultiArrayDimension>(&self.dim)?;
        <u32 as EncodeCdr>::encode_cdr(&self.data_offset, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for String {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <std::string::String as EncodeCdr>::encode_cdr(&self.data, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for UInt16 {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <u16 as EncodeCdr>::encode_cdr(&self.data, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for UInt16MultiArray {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <MultiArrayLayout as EncodeCdr>::encode_cdr(&self.layout, encoder)?;
        encoder.write_u16_seq(&self.data)?;
        Ok(())
    }
}

impl EncodeCdr for UInt32 {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <u32 as EncodeCdr>::encode_cdr(&self.data, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for UInt32MultiArray {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <MultiArrayLayout as EncodeCdr>::encode_cdr(&self.layout, encoder)?;
        encoder.write_u32_seq(&self.data)?;
        Ok(())
    }
}

impl EncodeCdr for UInt64 {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <u64 as EncodeCdr>::encode_cdr(&self.data, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for UInt64MultiArray {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <MultiArrayLayout as EncodeCdr>::encode_cdr(&self.layout, encoder)?;
        encoder.write_u64_seq(&self.data)?;
        Ok(())
    }
}

impl EncodeCdr for UInt8 {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <u8 as EncodeCdr>::encode_cdr(&self.data, encoder)?;
        Ok(())
    }
}

impl EncodeCdr for UInt8MultiArray {
    fn encode_cdr(&self, encoder: &mut CdrEncoder) -> CdrResult<()> {
        <MultiArrayLayout as EncodeCdr>::encode_cdr(&self.layout, encoder)?;
        encoder.write_octet_bytes(&self.data)?;
        Ok(())
    }
}
