#![allow(unused_imports)]
use crate::msg::*;
use crate::srv::*;

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Duration<'a> {
    #[allow(missing_docs)]
    pub sec: i32,
    #[allow(missing_docs)]
    pub nanosec: u32,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> Duration<'a> {
    pub fn to_owned(&self) -> crate::msg::Duration {
        crate::msg::Duration {
            sec: self.sec,
            nanosec: self.nanosec,
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Time<'a> {
    #[allow(missing_docs)]
    pub sec: i32,
    #[allow(missing_docs)]
    pub nanosec: u32,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> Time<'a> {
    pub fn to_owned(&self) -> crate::msg::Time {
        crate::msg::Time {
            sec: self.sec,
            nanosec: self.nanosec,
        }
    }
}
