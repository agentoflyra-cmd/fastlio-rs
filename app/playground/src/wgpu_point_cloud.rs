use iced::mouse;
use iced::widget::shader::{self, Pipeline, Primitive};
use iced::{Rectangle, wgpu};
use std::sync::Arc;
use wgpu::util::DeviceExt;

use crate::camera::CameraView;
use crate::point_cloud::PointXYZI;

const POINT_RADIUS: f32 = 2.0;
const FLOOR_RADIUS: f32 = 4.5;
const WALL_RADIUS: f32 = 3.8;

pub struct PointCloudProgram<'a> {
    points: Arc<[GpuPoint]>,
    camera: &'a CameraView,
    scale: f32,
}

impl<'a> PointCloudProgram<'a> {
    pub fn new(points: Arc<[GpuPoint]>, camera: &'a CameraView, scale: f32) -> Self {
        Self {
            points,
            camera,
            scale,
        }
    }
}

impl<Message> shader::Program<Message> for PointCloudProgram<'_> {
    type State = ();
    type Primitive = PointCloudPrimitive;

    fn draw(
        &self,
        _state: &Self::State,
        _cursor: mouse::Cursor,
        bounds: Rectangle,
    ) -> Self::Primitive {
        PointCloudPrimitive {
            points: self.points.clone(),
            uniforms: Uniforms::new(self.camera, self.scale, bounds),
        }
    }
}

#[derive(Debug)]
pub struct PointCloudPrimitive {
    points: Arc<[GpuPoint]>,
    uniforms: Uniforms,
}

impl Primitive for PointCloudPrimitive {
    type Pipeline = PointCloudPipeline;

    fn prepare(
        &self,
        pipeline: &mut Self::Pipeline,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _bounds: &Rectangle,
        _viewport: &shader::Viewport,
    ) {
        pipeline.prepare(device, queue, self.points.clone(), &self.uniforms);
    }

    fn draw(&self, pipeline: &Self::Pipeline, render_pass: &mut wgpu::RenderPass<'_>) -> bool {
        pipeline.draw(render_pass);
        true
    }
}

pub struct PointCloudPipeline {
    pipeline: wgpu::RenderPipeline,
    uniform_buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    point_buffer: Option<wgpu::Buffer>,
    point_source: Option<(usize, usize)>,
    point_count: u32,
}

impl Pipeline for PointCloudPipeline {
    fn new(device: &wgpu::Device, _queue: &wgpu::Queue, format: wgpu::TextureFormat) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("point cloud shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("wgpu_point_cloud.wgsl").into()),
        });

        let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("point cloud uniforms"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("point cloud bind group layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: wgpu::BufferSize::new(std::mem::size_of::<Uniforms>() as u64),
                },
                count: None,
            }],
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("point cloud bind group"),
            layout: &bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            }],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("point cloud pipeline layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("point cloud pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[GpuPoint::buffer_layout()],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                ..Default::default()
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        Self {
            pipeline,
            uniform_buffer,
            bind_group,
            point_buffer: None,
            point_source: None,
            point_count: 0,
        }
    }
}

impl PointCloudPipeline {
    fn prepare(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        points: Arc<[GpuPoint]>,
        uniforms: &Uniforms,
    ) {
        queue.write_buffer(&self.uniform_buffer, 0, bytemuck::bytes_of(uniforms));
        self.point_count = points.len().min(u32::MAX as usize) as u32;

        if points.is_empty() {
            self.point_buffer = None;
            self.point_source = None;
            return;
        }

        let point_source = (
            Arc::as_ptr(&points) as *const GpuPoint as usize,
            points.len(),
        );
        if self.point_source == Some(point_source) {
            return;
        }

        self.point_buffer = Some(
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("point cloud points"),
                contents: bytemuck::cast_slice(&points),
                usage: wgpu::BufferUsages::VERTEX,
            }),
        );
        self.point_source = Some(point_source);
    }

    fn draw(&self, render_pass: &mut wgpu::RenderPass<'_>) {
        let Some(point_buffer) = &self.point_buffer else {
            return;
        };

        render_pass.set_pipeline(&self.pipeline);
        render_pass.set_bind_group(0, &self.bind_group, &[]);
        render_pass.set_vertex_buffer(0, point_buffer.slice(..));
        render_pass.draw(0..6, 0..self.point_count);
    }
}

#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct GpuPoint {
    position: [f32; 3],
    intensity: f32,
    color: [f32; 3],
    radius: f32,
}

impl GpuPoint {
    const ATTRIBUTES: [wgpu::VertexAttribute; 4] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32, 2 => Float32x3, 3 => Float32];

    fn buffer_layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as u64,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &Self::ATTRIBUTES,
        }
    }

    pub fn from_point(point: &PointXYZI, height: f32) -> Self {
        let (color, radius) = point_style(point, height.clamp(0.0, 1.0));
        Self {
            position: [point.x, point.y, point.z],
            intensity: point.intensity,
            color,
            radius,
        }
    }
}

impl From<&PointXYZI> for GpuPoint {
    fn from(point: &PointXYZI) -> Self {
        Self::from_point(point, 0.5)
    }
}

fn point_style(point: &PointXYZI, height: f32) -> ([f32; 3], f32) {
    match point.normal.map(normalize) {
        Some([_, _, nz]) if nz.abs() >= 0.85 => ([0.20, 0.85, 0.35], FLOOR_RADIUS),
        Some([nx, ny, nz]) if nz.abs() <= 0.30 && (nx * nx + ny * ny).sqrt() >= 0.75 => {
            ([0.95, 0.58, 0.25], WALL_RADIUS)
        }
        _ => height_color(height),
    }
}

fn height_color(height: f32) -> ([f32; 3], f32) {
    let low = [0.20, 0.45, 1.00];
    let mid = [0.35, 1.00, 0.85];
    let high = [1.00, 0.85, 0.25];

    let color = if height < 0.5 {
        mix_color(low, mid, height * 2.0)
    } else {
        mix_color(mid, high, (height - 0.5) * 2.0)
    };

    (color, POINT_RADIUS)
}

fn mix_color(a: [f32; 3], b: [f32; 3], t: f32) -> [f32; 3] {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]
}

fn normalize(normal: [f32; 3]) -> [f32; 3] {
    let [x, y, z] = normal;
    let length = (x * x + y * y + z * z).sqrt();

    if length > 0.0 {
        [x / length, y / length, z / length]
    } else {
        normal
    }
}

#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
struct Uniforms {
    view: [[f32; 4]; 4],
    viewport: [f32; 2],
    scale: f32,
    radius: f32,
}

impl Uniforms {
    fn new(camera: &CameraView, scale: f32, bounds: Rectangle) -> Self {
        Self {
            view: camera.get_view_matrix(),
            viewport: [bounds.width.max(1.0), bounds.height.max(1.0)],
            scale,
            radius: POINT_RADIUS,
        }
    }
}
