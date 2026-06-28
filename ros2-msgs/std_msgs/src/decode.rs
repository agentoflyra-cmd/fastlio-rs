#[allow(unused_imports)]
use crate::msg::*;
#[allow(unused_imports)]
use crate::srv::*;
#[allow(unused_imports)]
pub use cdr_runtime::{
    CdrDecoder, CdrError, CdrResult, DecodeCdr, Endianness, WChar16, WChar32, decode_from_bytes,
};

impl DecodeCdr for Bool {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            data: <bool as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Byte {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            data: <u8 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for ByteMultiArray {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout as DecodeCdr>::decode_cdr(decoder)?,
            data: decoder.read_octet_seq()?,
        })
    }
}

impl DecodeCdr for Char {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            data: <u8 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for ColorRGBA {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            r: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            g: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            b: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            a: <f32 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Empty {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        let _ = decoder;
        Ok(Self {})
    }
}

impl DecodeCdr for Float32 {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            data: <f32 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Float32MultiArray {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout as DecodeCdr>::decode_cdr(decoder)?,
            data: decoder.read_f32_seq()?,
        })
    }
}

impl DecodeCdr for Float64 {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            data: <f64 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Float64MultiArray {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout as DecodeCdr>::decode_cdr(decoder)?,
            data: decoder.read_f64_seq()?,
        })
    }
}

impl DecodeCdr for Header {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            stamp: <builtin_interfaces::msg::Time as DecodeCdr>::decode_cdr(decoder)?,
            frame_id: <std::string::String as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Int16 {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            data: <i16 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Int16MultiArray {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout as DecodeCdr>::decode_cdr(decoder)?,
            data: decoder.read_i16_seq()?,
        })
    }
}

impl DecodeCdr for Int32 {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            data: <i32 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Int32MultiArray {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout as DecodeCdr>::decode_cdr(decoder)?,
            data: decoder.read_i32_seq()?,
        })
    }
}

impl DecodeCdr for Int64 {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            data: <i64 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Int64MultiArray {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout as DecodeCdr>::decode_cdr(decoder)?,
            data: decoder.read_i64_seq()?,
        })
    }
}

impl DecodeCdr for Int8 {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            data: <i8 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for Int8MultiArray {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout as DecodeCdr>::decode_cdr(decoder)?,
            data: decoder.read_seq::<i8>()?,
        })
    }
}

impl DecodeCdr for MultiArrayDimension {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            label: <std::string::String as DecodeCdr>::decode_cdr(decoder)?,
            size: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            stride: <u32 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for MultiArrayLayout {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            dim: decoder.read_seq::<MultiArrayDimension>()?,
            data_offset: <u32 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for String {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            data: <std::string::String as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for UInt16 {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            data: <u16 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for UInt16MultiArray {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout as DecodeCdr>::decode_cdr(decoder)?,
            data: decoder.read_u16_seq()?,
        })
    }
}

impl DecodeCdr for UInt32 {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            data: <u32 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for UInt32MultiArray {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout as DecodeCdr>::decode_cdr(decoder)?,
            data: decoder.read_u32_seq()?,
        })
    }
}

impl DecodeCdr for UInt64 {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            data: <u64 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for UInt64MultiArray {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout as DecodeCdr>::decode_cdr(decoder)?,
            data: decoder.read_u64_seq()?,
        })
    }
}

impl DecodeCdr for UInt8 {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            data: <u8 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl DecodeCdr for UInt8MultiArray {
    fn decode_cdr(decoder: &mut CdrDecoder<'_>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout as DecodeCdr>::decode_cdr(decoder)?,
            data: decoder.read_octet_seq()?,
        })
    }
}
