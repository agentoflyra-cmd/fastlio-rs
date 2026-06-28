#![allow(unused_imports)]
use crate::msg::*;
use crate::srv::*;

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Accel<'a> {
    #[allow(missing_docs)]
    pub linear: Vector3<'a>,
    #[allow(missing_docs)]
    pub angular: Vector3<'a>,
}

impl<'a> Accel<'a> {
    pub fn to_owned(&self) -> crate::msg::Accel {
        crate::msg::Accel {
            linear: self.linear.to_owned(),
            angular: self.angular.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AccelStamped<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub accel: Accel<'a>,
}

impl<'a> AccelStamped<'a> {
    pub fn to_owned(&self) -> crate::msg::AccelStamped {
        crate::msg::AccelStamped {
            header: self.header.to_owned(),
            accel: self.accel.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AccelWithCovariance<'a> {
    #[allow(missing_docs)]
    pub accel: Accel<'a>,
    #[allow(missing_docs)]
    pub covariance: cdr_runtime::PrimitiveArray<'a, f64, 36>,
}

impl<'a> AccelWithCovariance<'a> {
    pub fn to_owned(&self) -> crate::msg::AccelWithCovariance {
        crate::msg::AccelWithCovariance {
            accel: self.accel.to_owned(),
            covariance: self.covariance.to_array(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AccelWithCovarianceStamped<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub accel: AccelWithCovariance<'a>,
}

impl<'a> AccelWithCovarianceStamped<'a> {
    pub fn to_owned(&self) -> crate::msg::AccelWithCovarianceStamped {
        crate::msg::AccelWithCovarianceStamped {
            header: self.header.to_owned(),
            accel: self.accel.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Inertia<'a> {
    #[allow(missing_docs)]
    pub m: f64,
    #[allow(missing_docs)]
    pub com: Vector3<'a>,
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

impl<'a> Inertia<'a> {
    pub fn to_owned(&self) -> crate::msg::Inertia {
        crate::msg::Inertia {
            m: self.m,
            com: self.com.to_owned(),
            ixx: self.ixx,
            ixy: self.ixy,
            ixz: self.ixz,
            iyy: self.iyy,
            iyz: self.iyz,
            izz: self.izz,
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct InertiaStamped<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub inertia: Inertia<'a>,
}

impl<'a> InertiaStamped<'a> {
    pub fn to_owned(&self) -> crate::msg::InertiaStamped {
        crate::msg::InertiaStamped {
            header: self.header.to_owned(),
            inertia: self.inertia.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Point<'a> {
    #[allow(missing_docs)]
    pub x: f64,
    #[allow(missing_docs)]
    pub y: f64,
    #[allow(missing_docs)]
    pub z: f64,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> Point<'a> {
    pub fn to_owned(&self) -> crate::msg::Point {
        crate::msg::Point {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Point32<'a> {
    #[allow(missing_docs)]
    pub x: f32,
    #[allow(missing_docs)]
    pub y: f32,
    #[allow(missing_docs)]
    pub z: f32,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> Point32<'a> {
    pub fn to_owned(&self) -> crate::msg::Point32 {
        crate::msg::Point32 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PointStamped<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub point: Point<'a>,
}

impl<'a> PointStamped<'a> {
    pub fn to_owned(&self) -> crate::msg::PointStamped {
        crate::msg::PointStamped {
            header: self.header.to_owned(),
            point: self.point.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Polygon<'a> {
    #[allow(missing_docs)]
    pub points: Vec<Point32<'a>>,
}

impl<'a> Polygon<'a> {
    pub fn to_owned(&self) -> crate::msg::Polygon {
        crate::msg::Polygon {
            points: self.points.iter().map(|item| item.to_owned()).collect(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PolygonInstance<'a> {
    #[allow(missing_docs)]
    pub polygon: Polygon<'a>,
    #[allow(missing_docs)]
    pub id: i64,
}

impl<'a> PolygonInstance<'a> {
    pub fn to_owned(&self) -> crate::msg::PolygonInstance {
        crate::msg::PolygonInstance {
            polygon: self.polygon.to_owned(),
            id: self.id,
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PolygonInstanceStamped<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub polygon: PolygonInstance<'a>,
}

impl<'a> PolygonInstanceStamped<'a> {
    pub fn to_owned(&self) -> crate::msg::PolygonInstanceStamped {
        crate::msg::PolygonInstanceStamped {
            header: self.header.to_owned(),
            polygon: self.polygon.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PolygonStamped<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub polygon: Polygon<'a>,
}

impl<'a> PolygonStamped<'a> {
    pub fn to_owned(&self) -> crate::msg::PolygonStamped {
        crate::msg::PolygonStamped {
            header: self.header.to_owned(),
            polygon: self.polygon.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Pose<'a> {
    #[allow(missing_docs)]
    pub position: Point<'a>,
    #[allow(missing_docs)]
    pub orientation: Quaternion<'a>,
}

impl<'a> Pose<'a> {
    pub fn to_owned(&self) -> crate::msg::Pose {
        crate::msg::Pose {
            position: self.position.to_owned(),
            orientation: self.orientation.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PoseArray<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub poses: Vec<Pose<'a>>,
}

impl<'a> PoseArray<'a> {
    pub fn to_owned(&self) -> crate::msg::PoseArray {
        crate::msg::PoseArray {
            header: self.header.to_owned(),
            poses: self.poses.iter().map(|item| item.to_owned()).collect(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PoseStamped<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub pose: Pose<'a>,
}

impl<'a> PoseStamped<'a> {
    pub fn to_owned(&self) -> crate::msg::PoseStamped {
        crate::msg::PoseStamped {
            header: self.header.to_owned(),
            pose: self.pose.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PoseWithCovariance<'a> {
    #[allow(missing_docs)]
    pub pose: Pose<'a>,
    #[allow(missing_docs)]
    pub covariance: cdr_runtime::PrimitiveArray<'a, f64, 36>,
}

impl<'a> PoseWithCovariance<'a> {
    pub fn to_owned(&self) -> crate::msg::PoseWithCovariance {
        crate::msg::PoseWithCovariance {
            pose: self.pose.to_owned(),
            covariance: self.covariance.to_array(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PoseWithCovarianceStamped<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub pose: PoseWithCovariance<'a>,
}

impl<'a> PoseWithCovarianceStamped<'a> {
    pub fn to_owned(&self) -> crate::msg::PoseWithCovarianceStamped {
        crate::msg::PoseWithCovarianceStamped {
            header: self.header.to_owned(),
            pose: self.pose.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Quaternion<'a> {
    #[allow(missing_docs)]
    pub x: f64,
    #[allow(missing_docs)]
    pub y: f64,
    #[allow(missing_docs)]
    pub z: f64,
    #[allow(missing_docs)]
    pub w: f64,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> Quaternion<'a> {
    pub fn to_owned(&self) -> crate::msg::Quaternion {
        crate::msg::Quaternion {
            x: self.x,
            y: self.y,
            z: self.z,
            w: self.w,
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct QuaternionStamped<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub quaternion: Quaternion<'a>,
}

impl<'a> QuaternionStamped<'a> {
    pub fn to_owned(&self) -> crate::msg::QuaternionStamped {
        crate::msg::QuaternionStamped {
            header: self.header.to_owned(),
            quaternion: self.quaternion.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Transform<'a> {
    #[allow(missing_docs)]
    pub translation: Vector3<'a>,
    #[allow(missing_docs)]
    pub rotation: Quaternion<'a>,
}

impl<'a> Transform<'a> {
    pub fn to_owned(&self) -> crate::msg::Transform {
        crate::msg::Transform {
            translation: self.translation.to_owned(),
            rotation: self.rotation.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TransformStamped<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub child_frame_id: &'a str,
    #[allow(missing_docs)]
    pub transform: Transform<'a>,
}

impl<'a> TransformStamped<'a> {
    pub fn to_owned(&self) -> crate::msg::TransformStamped {
        crate::msg::TransformStamped {
            header: self.header.to_owned(),
            child_frame_id: self.child_frame_id.to_string(),
            transform: self.transform.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Twist<'a> {
    #[allow(missing_docs)]
    pub linear: Vector3<'a>,
    #[allow(missing_docs)]
    pub angular: Vector3<'a>,
}

impl<'a> Twist<'a> {
    pub fn to_owned(&self) -> crate::msg::Twist {
        crate::msg::Twist {
            linear: self.linear.to_owned(),
            angular: self.angular.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TwistStamped<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub twist: Twist<'a>,
}

impl<'a> TwistStamped<'a> {
    pub fn to_owned(&self) -> crate::msg::TwistStamped {
        crate::msg::TwistStamped {
            header: self.header.to_owned(),
            twist: self.twist.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TwistWithCovariance<'a> {
    #[allow(missing_docs)]
    pub twist: Twist<'a>,
    #[allow(missing_docs)]
    pub covariance: cdr_runtime::PrimitiveArray<'a, f64, 36>,
}

impl<'a> TwistWithCovariance<'a> {
    pub fn to_owned(&self) -> crate::msg::TwistWithCovariance {
        crate::msg::TwistWithCovariance {
            twist: self.twist.to_owned(),
            covariance: self.covariance.to_array(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TwistWithCovarianceStamped<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub twist: TwistWithCovariance<'a>,
}

impl<'a> TwistWithCovarianceStamped<'a> {
    pub fn to_owned(&self) -> crate::msg::TwistWithCovarianceStamped {
        crate::msg::TwistWithCovarianceStamped {
            header: self.header.to_owned(),
            twist: self.twist.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Vector3<'a> {
    #[allow(missing_docs)]
    pub x: f64,
    #[allow(missing_docs)]
    pub y: f64,
    #[allow(missing_docs)]
    pub z: f64,
    #[allow(dead_code)]
    pub(crate) _phantom: std::marker::PhantomData<&'a ()>,
}

impl<'a> Vector3<'a> {
    pub fn to_owned(&self) -> crate::msg::Vector3 {
        crate::msg::Vector3 {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Vector3Stamped<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub vector: Vector3<'a>,
}

impl<'a> Vector3Stamped<'a> {
    pub fn to_owned(&self) -> crate::msg::Vector3Stamped {
        crate::msg::Vector3Stamped {
            header: self.header.to_owned(),
            vector: self.vector.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VelocityStamped<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub body_frame_id: &'a str,
    #[allow(missing_docs)]
    pub reference_frame_id: &'a str,
    #[allow(missing_docs)]
    pub velocity: Twist<'a>,
}

impl<'a> VelocityStamped<'a> {
    pub fn to_owned(&self) -> crate::msg::VelocityStamped {
        crate::msg::VelocityStamped {
            header: self.header.to_owned(),
            body_frame_id: self.body_frame_id.to_string(),
            reference_frame_id: self.reference_frame_id.to_string(),
            velocity: self.velocity.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VelocityWithCovarianceStamped<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub body_frame_id: &'a str,
    #[allow(missing_docs)]
    pub reference_frame_id: &'a str,
    #[allow(missing_docs)]
    pub velocity: TwistWithCovariance<'a>,
}

impl<'a> VelocityWithCovarianceStamped<'a> {
    pub fn to_owned(&self) -> crate::msg::VelocityWithCovarianceStamped {
        crate::msg::VelocityWithCovarianceStamped {
            header: self.header.to_owned(),
            body_frame_id: self.body_frame_id.to_string(),
            reference_frame_id: self.reference_frame_id.to_string(),
            velocity: self.velocity.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Wrench<'a> {
    #[allow(missing_docs)]
    pub force: Vector3<'a>,
    #[allow(missing_docs)]
    pub torque: Vector3<'a>,
}

impl<'a> Wrench<'a> {
    pub fn to_owned(&self) -> crate::msg::Wrench {
        crate::msg::Wrench {
            force: self.force.to_owned(),
            torque: self.torque.to_owned(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct WrenchStamped<'a> {
    #[allow(missing_docs)]
    pub header: std_msgs::borrowed::Header<'a>,
    #[allow(missing_docs)]
    pub wrench: Wrench<'a>,
}

impl<'a> WrenchStamped<'a> {
    pub fn to_owned(&self) -> crate::msg::WrenchStamped {
        crate::msg::WrenchStamped {
            header: self.header.to_owned(),
            wrench: self.wrench.to_owned(),
        }
    }
}
