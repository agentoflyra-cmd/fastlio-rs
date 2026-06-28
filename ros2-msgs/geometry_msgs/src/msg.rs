#![allow(unused_imports)]
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Accel {
    #[allow(missing_docs)]
    pub linear: Vector3,
    #[allow(missing_docs)]
    pub angular: Vector3,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct AccelStamped {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub accel: Accel,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct AccelWithCovariance {
    #[allow(missing_docs)]
    pub accel: Accel,
    #[allow(missing_docs)]
    #[cfg_attr(feature = "serde", serde(with = "serde_big_array::BigArray"))]
    pub covariance: [f64; 36],
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct AccelWithCovarianceStamped {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub accel: AccelWithCovariance,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Inertia {
    #[allow(missing_docs)]
    pub m: f64,
    #[allow(missing_docs)]
    pub com: Vector3,
    #[allow(missing_docs)]
    pub ixx: f64,
    #[allow(missing_docs)]
    pub ixy: f64,
    #[allow(missing_docs)]
    pub ixz: f64,
    #[allow(missing_docs)]
    pub iyy: f64,
    #[allow(missing_docs)]
    pub iyz: f64,
    #[allow(missing_docs)]
    pub izz: f64,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct InertiaStamped {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub inertia: Inertia,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Point {
    #[allow(missing_docs)]
    pub x: f64,
    #[allow(missing_docs)]
    pub y: f64,
    #[allow(missing_docs)]
    pub z: f64,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Point32 {
    #[allow(missing_docs)]
    pub x: f32,
    #[allow(missing_docs)]
    pub y: f32,
    #[allow(missing_docs)]
    pub z: f32,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct PointStamped {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub point: Point,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Polygon {
    #[allow(missing_docs)]
    pub points: Vec<Point32>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct PolygonInstance {
    #[allow(missing_docs)]
    pub polygon: Polygon,
    #[allow(missing_docs)]
    pub id: i64,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct PolygonInstanceStamped {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub polygon: PolygonInstance,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct PolygonStamped {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub polygon: Polygon,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Pose {
    #[allow(missing_docs)]
    pub position: Point,
    #[allow(missing_docs)]
    pub orientation: Quaternion,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct PoseArray {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub poses: Vec<Pose>,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct PoseStamped {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub pose: Pose,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct PoseWithCovariance {
    #[allow(missing_docs)]
    pub pose: Pose,
    #[allow(missing_docs)]
    #[cfg_attr(feature = "serde", serde(with = "serde_big_array::BigArray"))]
    pub covariance: [f64; 36],
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct PoseWithCovarianceStamped {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub pose: PoseWithCovariance,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Quaternion {
    #[allow(missing_docs)]
    pub x: f64,
    #[allow(missing_docs)]
    pub y: f64,
    #[allow(missing_docs)]
    pub z: f64,
    #[allow(missing_docs)]
    pub w: f64,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct QuaternionStamped {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub quaternion: Quaternion,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Transform {
    #[allow(missing_docs)]
    pub translation: Vector3,
    #[allow(missing_docs)]
    pub rotation: Quaternion,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct TransformStamped {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub child_frame_id: std::string::String,
    #[allow(missing_docs)]
    pub transform: Transform,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Twist {
    #[allow(missing_docs)]
    pub linear: Vector3,
    #[allow(missing_docs)]
    pub angular: Vector3,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct TwistStamped {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub twist: Twist,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct TwistWithCovariance {
    #[allow(missing_docs)]
    pub twist: Twist,
    #[allow(missing_docs)]
    #[cfg_attr(feature = "serde", serde(with = "serde_big_array::BigArray"))]
    pub covariance: [f64; 36],
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct TwistWithCovarianceStamped {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub twist: TwistWithCovariance,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Vector3 {
    #[allow(missing_docs)]
    pub x: f64,
    #[allow(missing_docs)]
    pub y: f64,
    #[allow(missing_docs)]
    pub z: f64,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Vector3Stamped {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub vector: Vector3,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct VelocityStamped {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub body_frame_id: std::string::String,
    #[allow(missing_docs)]
    pub reference_frame_id: std::string::String,
    #[allow(missing_docs)]
    pub velocity: Twist,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct VelocityWithCovarianceStamped {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub body_frame_id: std::string::String,
    #[allow(missing_docs)]
    pub reference_frame_id: std::string::String,
    #[allow(missing_docs)]
    pub velocity: TwistWithCovariance,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Wrench {
    #[allow(missing_docs)]
    pub force: Vector3,
    #[allow(missing_docs)]
    pub torque: Vector3,
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct WrenchStamped {
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,
    #[allow(missing_docs)]
    pub wrench: Wrench,
}
