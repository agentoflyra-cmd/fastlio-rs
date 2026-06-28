#[allow(unused_imports)]
use crate::borrowed::*;
#[allow(unused_imports)]
pub use cdr_runtime::{
    BorrowDecodeCdr, CdrDecoder, CdrError, CdrResult, DecodeCdr, Endianness, WChar16, WChar32,
    borrow_decode_from_bytes,
};

impl<'a> BorrowDecodeCdr<'a> for Bool<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            data: <bool as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Byte<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            data: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for ByteMultiArray<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            data: decoder.read_octet_slice()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Char<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            data: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for ColorRGBA<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            r: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            g: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            b: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            a: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Empty<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        let _ = decoder;
        Ok(Self {
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Float32<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            data: <f32 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Float32MultiArray<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            data: decoder.read_primitive_seq_borrowed::<f32>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Float64<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            data: <f64 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Float64MultiArray<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            data: decoder.read_primitive_seq_borrowed::<f64>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Header<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            stamp:
                <builtin_interfaces::borrowed::Time<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(
                    decoder,
                )?,
            frame_id: decoder.read_str()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Int16<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            data: <i16 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Int16MultiArray<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            data: decoder.read_primitive_seq_borrowed::<i16>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Int32<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            data: <i32 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Int32MultiArray<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            data: decoder.read_primitive_seq_borrowed::<i32>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Int64<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            data: <i64 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Int64MultiArray<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            data: decoder.read_primitive_seq_borrowed::<i64>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Int8<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            data: <i8 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for Int8MultiArray<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            data: decoder.read_seq::<i8>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for MultiArrayDimension<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            label: decoder.read_str()?,
            size: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            stride: <u32 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for MultiArrayLayout<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            dim: decoder.read_borrow_seq::<MultiArrayDimension<'a>>()?,
            data_offset: <u32 as DecodeCdr>::decode_cdr(decoder)?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for String<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            data: decoder.read_str()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for UInt16<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            data: <u16 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for UInt16MultiArray<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            data: decoder.read_primitive_seq_borrowed::<u16>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for UInt32<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            data: <u32 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for UInt32MultiArray<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            data: decoder.read_primitive_seq_borrowed::<u32>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for UInt64<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            data: <u64 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for UInt64MultiArray<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            data: decoder.read_primitive_seq_borrowed::<u64>()?,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for UInt8<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            data: <u8 as DecodeCdr>::decode_cdr(decoder)?,
            _phantom: std::marker::PhantomData,
        })
    }
}

impl<'a> BorrowDecodeCdr<'a> for UInt8MultiArray<'a> {
    fn borrow_decode_cdr(decoder: &mut CdrDecoder<'a>) -> CdrResult<Self> {
        Ok(Self {
            layout: <MultiArrayLayout<'a> as BorrowDecodeCdr<'a>>::borrow_decode_cdr(decoder)?,
            data: decoder.read_octet_slice()?,
        })
    }
}
