#![allow(unused_imports)]
use crate::msg::*;
use crate::srv::*;

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Bool<'a> {
    #[allow(missing_docs)]
    pub data: bool,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> Bool<'a> {
    pub fn to_owned(&self) -> crate::msg::Bool {
        crate::msg::Bool { data: self.data }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Byte<'a> {
    #[allow(missing_docs)]
    pub data: u8,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> Byte<'a> {
    pub fn to_owned(&self) -> crate::msg::Byte {
        crate::msg::Byte { data: self.data }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ByteMultiArray<'a> {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout<'a>,
    #[allow(missing_docs)]
    pub data: &'a [u8],
}

impl<'a> ByteMultiArray<'a> {
    pub fn to_owned(&self) -> crate::msg::ByteMultiArray {
        crate::msg::ByteMultiArray {
            layout: self.layout.to_owned(),
            data: self.data.to_vec(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Char<'a> {
    #[allow(missing_docs)]
    pub data: u8,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> Char<'a> {
    pub fn to_owned(&self) -> crate::msg::Char {
        crate::msg::Char { data: self.data }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ColorRGBA<'a> {
    #[allow(missing_docs)]
    pub r: f32,
    #[allow(missing_docs)]
    pub g: f32,
    #[allow(missing_docs)]
    pub b: f32,
    #[allow(missing_docs)]
    pub a: f32,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> ColorRGBA<'a> {
    pub fn to_owned(&self) -> crate::msg::ColorRGBA {
        crate::msg::ColorRGBA {
            r: self.r,
            g: self.g,
            b: self.b,
            a: self.a,
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Empty<'a> {
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> Empty<'a> {
    pub fn to_owned(&self) -> crate::msg::Empty {
        crate::msg::Empty {}
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Float32<'a> {
    #[allow(missing_docs)]
    pub data: f32,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> Float32<'a> {
    pub fn to_owned(&self) -> crate::msg::Float32 {
        crate::msg::Float32 { data: self.data }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Float32MultiArray<'a> {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout<'a>,
    #[allow(missing_docs)]
    pub data: cdr_runtime::PrimitiveSeq<'a, f32>,
}

impl<'a> Float32MultiArray<'a> {
    pub fn to_owned(&self) -> crate::msg::Float32MultiArray {
        crate::msg::Float32MultiArray {
            layout: self.layout.to_owned(),
            data: self.data.to_vec(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Float64<'a> {
    #[allow(missing_docs)]
    pub data: f64,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> Float64<'a> {
    pub fn to_owned(&self) -> crate::msg::Float64 {
        crate::msg::Float64 { data: self.data }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Float64MultiArray<'a> {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout<'a>,
    #[allow(missing_docs)]
    pub data: cdr_runtime::PrimitiveSeq<'a, f64>,
}

impl<'a> Float64MultiArray<'a> {
    pub fn to_owned(&self) -> crate::msg::Float64MultiArray {
        crate::msg::Float64MultiArray {
            layout: self.layout.to_owned(),
            data: self.data.to_vec(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Header<'a> {
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::borrowed::Time<'a>,
    #[allow(missing_docs)]
    pub frame_id: &'a str,
}

impl<'a> Header<'a> {
    pub fn to_owned(&self) -> crate::msg::Header {
        crate::msg::Header {
            stamp: self.stamp.to_owned(),
            frame_id: self.frame_id.to_string(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Int16<'a> {
    #[allow(missing_docs)]
    pub data: i16,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> Int16<'a> {
    pub fn to_owned(&self) -> crate::msg::Int16 {
        crate::msg::Int16 { data: self.data }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Int16MultiArray<'a> {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout<'a>,
    #[allow(missing_docs)]
    pub data: cdr_runtime::PrimitiveSeq<'a, i16>,
}

impl<'a> Int16MultiArray<'a> {
    pub fn to_owned(&self) -> crate::msg::Int16MultiArray {
        crate::msg::Int16MultiArray {
            layout: self.layout.to_owned(),
            data: self.data.to_vec(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Int32<'a> {
    #[allow(missing_docs)]
    pub data: i32,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> Int32<'a> {
    pub fn to_owned(&self) -> crate::msg::Int32 {
        crate::msg::Int32 { data: self.data }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Int32MultiArray<'a> {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout<'a>,
    #[allow(missing_docs)]
    pub data: cdr_runtime::PrimitiveSeq<'a, i32>,
}

impl<'a> Int32MultiArray<'a> {
    pub fn to_owned(&self) -> crate::msg::Int32MultiArray {
        crate::msg::Int32MultiArray {
            layout: self.layout.to_owned(),
            data: self.data.to_vec(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Int64<'a> {
    #[allow(missing_docs)]
    pub data: i64,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> Int64<'a> {
    pub fn to_owned(&self) -> crate::msg::Int64 {
        crate::msg::Int64 { data: self.data }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Int64MultiArray<'a> {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout<'a>,
    #[allow(missing_docs)]
    pub data: cdr_runtime::PrimitiveSeq<'a, i64>,
}

impl<'a> Int64MultiArray<'a> {
    pub fn to_owned(&self) -> crate::msg::Int64MultiArray {
        crate::msg::Int64MultiArray {
            layout: self.layout.to_owned(),
            data: self.data.to_vec(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Int8<'a> {
    #[allow(missing_docs)]
    pub data: i8,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> Int8<'a> {
    pub fn to_owned(&self) -> crate::msg::Int8 {
        crate::msg::Int8 { data: self.data }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Int8MultiArray<'a> {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout<'a>,
    #[allow(missing_docs)]
    pub data: Vec<i8>,
}

impl<'a> Int8MultiArray<'a> {
    pub fn to_owned(&self) -> crate::msg::Int8MultiArray {
        crate::msg::Int8MultiArray {
            layout: self.layout.to_owned(),
            data: self.data.clone(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MultiArrayDimension<'a> {
    #[allow(missing_docs)]
    pub label: &'a str,
    #[allow(missing_docs)]
    pub size: u32,
    #[allow(missing_docs)]
    pub stride: u32,
}

impl<'a> MultiArrayDimension<'a> {
    pub fn to_owned(&self) -> crate::msg::MultiArrayDimension {
        crate::msg::MultiArrayDimension {
            label: self.label.to_string(),
            size: self.size,
            stride: self.stride,
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MultiArrayLayout<'a> {
    #[allow(missing_docs)]
    pub dim: Vec<MultiArrayDimension<'a>>,
    #[allow(missing_docs)]
    pub data_offset: u32,
}

impl<'a> MultiArrayLayout<'a> {
    pub fn to_owned(&self) -> crate::msg::MultiArrayLayout {
        crate::msg::MultiArrayLayout {
            dim: self.dim.iter().map(|item| item.to_owned()).collect(),
            data_offset: self.data_offset,
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct String<'a> {
    #[allow(missing_docs)]
    pub data: &'a str,
}

impl<'a> String<'a> {
    pub fn to_owned(&self) -> crate::msg::String {
        crate::msg::String {
            data: self.data.to_string(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UInt16<'a> {
    #[allow(missing_docs)]
    pub data: u16,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> UInt16<'a> {
    pub fn to_owned(&self) -> crate::msg::UInt16 {
        crate::msg::UInt16 { data: self.data }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UInt16MultiArray<'a> {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout<'a>,
    #[allow(missing_docs)]
    pub data: cdr_runtime::PrimitiveSeq<'a, u16>,
}

impl<'a> UInt16MultiArray<'a> {
    pub fn to_owned(&self) -> crate::msg::UInt16MultiArray {
        crate::msg::UInt16MultiArray {
            layout: self.layout.to_owned(),
            data: self.data.to_vec(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UInt32<'a> {
    #[allow(missing_docs)]
    pub data: u32,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> UInt32<'a> {
    pub fn to_owned(&self) -> crate::msg::UInt32 {
        crate::msg::UInt32 { data: self.data }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UInt32MultiArray<'a> {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout<'a>,
    #[allow(missing_docs)]
    pub data: cdr_runtime::PrimitiveSeq<'a, u32>,
}

impl<'a> UInt32MultiArray<'a> {
    pub fn to_owned(&self) -> crate::msg::UInt32MultiArray {
        crate::msg::UInt32MultiArray {
            layout: self.layout.to_owned(),
            data: self.data.to_vec(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UInt64<'a> {
    #[allow(missing_docs)]
    pub data: u64,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> UInt64<'a> {
    pub fn to_owned(&self) -> crate::msg::UInt64 {
        crate::msg::UInt64 { data: self.data }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UInt64MultiArray<'a> {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout<'a>,
    #[allow(missing_docs)]
    pub data: cdr_runtime::PrimitiveSeq<'a, u64>,
}

impl<'a> UInt64MultiArray<'a> {
    pub fn to_owned(&self) -> crate::msg::UInt64MultiArray {
        crate::msg::UInt64MultiArray {
            layout: self.layout.to_owned(),
            data: self.data.to_vec(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UInt8<'a> {
    #[allow(missing_docs)]
    pub data: u8,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> UInt8<'a> {
    pub fn to_owned(&self) -> crate::msg::UInt8 {
        crate::msg::UInt8 { data: self.data }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UInt8MultiArray<'a> {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout<'a>,
    #[allow(missing_docs)]
    pub data: &'a [u8],
}

impl<'a> UInt8MultiArray<'a> {
    pub fn to_owned(&self) -> crate::msg::UInt8MultiArray {
        crate::msg::UInt8MultiArray {
            layout: self.layout.to_owned(),
            data: self.data.to_vec(),
        }
    }
}
