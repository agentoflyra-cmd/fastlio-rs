#![allow(unused_imports)]
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Duration {
    #[allow(missing_docs)]
    pub sec: i32,
    #[allow(missing_docs)]
    pub nanosec: u32,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Time {
    #[allow(missing_docs)]
    pub sec: i32,
    #[allow(missing_docs)]
    pub nanosec: u32,
}
