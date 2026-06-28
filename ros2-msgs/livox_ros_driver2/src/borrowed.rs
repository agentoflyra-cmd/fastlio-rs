#![allow(unused_imports)]
use crate::msg::*;
use crate::srv::*;

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CustomMsg<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub timebase: u64,
    #[allow(missing_docs)]
    pub point_num: u32,
    #[allow(missing_docs)]
    pub lidar_id: u8,
    #[allow(missing_docs)]
    pub rsvd: [u8; 3],
    #[allow(missing_docs)]
    pub points: Vec<CustomPoint<'a>>,
}

impl<'a> CustomMsg<'a> {
    pub fn to_owned(&self) -> crate::msg::CustomMsg {
        crate::msg::CustomMsg {
            header: self.header.to_owned(),
            timebase: self.timebase,
            point_num: self.point_num,
            lidar_id: self.lidar_id,
            rsvd: self.rsvd,
            points: self.points.iter().map(|item| item.to_owned()).collect(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CustomPoint<'a> {
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
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> CustomPoint<'a> {
    pub fn to_owned(&self) -> crate::msg::CustomPoint {
        crate::msg::CustomPoint {
            offset_time: self.offset_time,
            x: self.x,
            y: self.y,
            z: self.z,
            reflectivity: self.reflectivity,
            tag: self.tag,
            line: self.line,
        }
    }
}
