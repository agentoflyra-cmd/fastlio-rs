use iced::widget::Shader;
use iced::{Element, Length};
use std::sync::Arc;

use crate::camera::CameraView;
use crate::wgpu_point_cloud::{GpuPoint, PointCloudProgram};

pub fn point_cloud_scene<'a, Message: 'a>(
    points: Arc<[GpuPoint]>,
    camera: &'a CameraView,
    scale: f32,
) -> Element<'a, Message> {
    Shader::new(PointCloudProgram::new(points, camera, scale))
        .width(Length::Fill)
        .height(Length::Fill)
        .into()
}
