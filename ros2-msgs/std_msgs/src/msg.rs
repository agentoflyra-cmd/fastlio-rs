#![allow(unused_imports)]
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Bool {
    #[allow(missing_docs)]
    pub data: bool,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Byte {
    #[allow(missing_docs)]
    pub data: u8,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct ByteMultiArray {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout,
    #[allow(missing_docs)]
    pub data: Vec<u8>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Char {
    #[allow(missing_docs)]
    pub data: u8,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct ColorRGBA {
    #[allow(missing_docs)]
    pub r: f32,
    #[allow(missing_docs)]
    pub g: f32,
    #[allow(missing_docs)]
    pub b: f32,
    #[allow(missing_docs)]
    pub a: f32,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Empty;

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Float32 {
    #[allow(missing_docs)]
    pub data: f32,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Float32MultiArray {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout,
    #[allow(missing_docs)]
    pub data: Vec<f32>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Float64 {
    #[allow(missing_docs)]
    pub data: f64,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Float64MultiArray {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout,
    #[allow(missing_docs)]
    pub data: Vec<f64>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Header {
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,
    #[allow(missing_docs)]
    pub frame_id: std::string::String,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Int16 {
    #[allow(missing_docs)]
    pub data: i16,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Int16MultiArray {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout,
    #[allow(missing_docs)]
    pub data: Vec<i16>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Int32 {
    #[allow(missing_docs)]
    pub data: i32,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Int32MultiArray {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout,
    #[allow(missing_docs)]
    pub data: Vec<i32>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Int64 {
    #[allow(missing_docs)]
    pub data: i64,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Int64MultiArray {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout,
    #[allow(missing_docs)]
    pub data: Vec<i64>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Int8 {
    #[allow(missing_docs)]
    pub data: i8,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Int8MultiArray {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout,
    #[allow(missing_docs)]
    pub data: Vec<i8>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct MultiArrayDimension {
    #[allow(missing_docs)]
    pub label: std::string::String,
    #[allow(missing_docs)]
    pub size: u32,
    #[allow(missing_docs)]
    pub stride: u32,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct MultiArrayLayout {
    #[allow(missing_docs)]
    pub dim: Vec<MultiArrayDimension>,
    #[allow(missing_docs)]
    pub data_offset: u32,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct String {
    #[allow(missing_docs)]
    pub data: std::string::String,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct UInt16 {
    #[allow(missing_docs)]
    pub data: u16,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct UInt16MultiArray {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout,
    #[allow(missing_docs)]
    pub data: Vec<u16>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct UInt32 {
    #[allow(missing_docs)]
    pub data: u32,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct UInt32MultiArray {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout,
    #[allow(missing_docs)]
    pub data: Vec<u32>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct UInt64 {
    #[allow(missing_docs)]
    pub data: u64,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct UInt64MultiArray {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout,
    #[allow(missing_docs)]
    pub data: Vec<u64>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct UInt8 {
    #[allow(missing_docs)]
    pub data: u8,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct UInt8MultiArray {
    #[allow(missing_docs)]
    pub layout: MultiArrayLayout,
    #[allow(missing_docs)]
    pub data: Vec<u8>,
}
