#![allow(unused_imports)]
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
pub use crate::msg::*;

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct SetCameraInfoRequest {
    #[allow(missing_docs)]
    pub camera_info: CameraInfo,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct SetCameraInfoResponse {
    #[allow(missing_docs)]
    pub success: bool,
    #[allow(missing_docs)]
    pub status_message: std::string::String,
}