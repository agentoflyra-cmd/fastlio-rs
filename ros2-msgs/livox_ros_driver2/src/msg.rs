#![allow(unused_imports)]
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct CustomMsg {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub timebase: u64,
    #[allow(missing_docs)]
    pub point_num: u32,
    #[allow(missing_docs)]
    pub lidar_id: u8,
    #[allow(missing_docs)]
    pub rsvd: [u8; 3],
    #[allow(missing_docs)]
    pub points: Vec<CustomPoint>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct CustomPoint {
    #[allow(missing_docs)]
    pub offset_time: u32,
    #[allow(missing_docs)]
    pub x: f32,
    #[allow(missing_docs)]
    pub y: f32,
    #[allow(missing_docs)]
    pub z: f32,
    #[allow(missing_docs)]
    pub reflectivity: u8,
    #[allow(missing_docs)]
    pub tag: u8,
    #[allow(missing_docs)]
    pub line: u8,
}
