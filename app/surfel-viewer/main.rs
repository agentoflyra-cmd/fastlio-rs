use anyhow::{Context, Result};
use iced::wgpu::util::DeviceExt;
use iced::widget::shader::{self, Pipeline, Primitive};
use iced::widget::{Shader, column, text};
use iced::{Element, Event, Length, Point, Rectangle, Subscription, event, mouse};
use pcd_rs::{DynReader, Field};
use std::sync::Arc;

const INITIAL_RADIUS: f32 = 20.0;
const POINT_SCALE: f32 = 35.0;
const MIN_RADIUS: f32 = 0.1;
const MAX_RADIUS: f32 = 1000.0;

#[allow(clippy::enum_variant_names)]
#[derive(Debug, Clone)]
enum Message {
    MousePressed(DragMode),
    MouseReleased,
    MouseMoved(Point),
    MouseWheel(f32),
}

#[derive(Debug, Clone, Copy)]
enum DragMode {
    Orbit,
    Pan,
}

#[derive(Debug, Clone, Copy)]
struct Vec3 {
    x: f32,
    y: f32,
    z: f32,
}

impl Vec3 {
    const Z: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    };

    fn sub(self, rhs: Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }

    fn add(self, rhs: Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }

    fn scale(self, value: f32) -> Self {
        Self {
            x: self.x * value,
            y: self.y * value,
            z: self.z * value,
        }
    }

    fn dot(self, rhs: Self) -> f32 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    fn cross(self, rhs: Self) -> Self {
        Self {
            x: self.y * rhs.z - self.z * rhs.y,
            y: self.z * rhs.x - self.x * rhs.z,
            z: self.x * rhs.y - self.y * rhs.x,
        }
    }

    fn normalize(self) -> Self {
        let norm = self.dot(self).sqrt();
        if norm > 0.0 {
            self.scale(1.0 / norm)
        } else {
            self
        }
    }
}

struct Camera {
    eye: Vec3,
    target: Vec3,
    up: Vec3,
    radius: f32,
    yaw: f32,
    pitch: f32,
}

impl Camera {
    fn new(target: Vec3, radius: f32) -> Self {
        let mut camera = Self {
            eye: target,
            target,
            up: Vec3::Z,
            radius,
            yaw: 0.0,
            pitch: 0.0,
        };
        camera.update_eye();
        camera
    }

    fn update_eye(&mut self) {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        self.eye = self.target.add(Vec3 {
            x: self.radius * cp * sy,
            y: self.radius * cp * cy,
            z: self.radius * sp,
        });
    }

    fn orbit(&mut self, dx: f32, dy: f32) {
        self.yaw += dx * 0.005;
        self.pitch = (self.pitch - dy * 0.005).clamp(-1.48, 1.48);
        self.update_eye();
    }

    fn zoom(&mut self, delta: f32) {
        self.radius = (self.radius - delta).clamp(MIN_RADIUS, MAX_RADIUS);
        self.update_eye();
    }

    fn pan(&mut self, dx: f32, dy: f32) {
        let forward = self.eye.sub(self.target).normalize();
        let right = forward.cross(self.up).normalize();
        let true_up = right.cross(forward);
        let shift = right
            .scale(-dx / POINT_SCALE)
            .add(true_up.scale(dy / POINT_SCALE));
        self.target = self.target.add(shift);
        self.eye = self.eye.add(shift);
    }

    fn view(&self) -> [[f32; 4]; 4] {
        let forward = self.eye.sub(self.target).normalize();
        let right = forward.cross(self.up).normalize();
        let up = right.cross(forward);
        [
            [right.x, right.y, right.z, -right.dot(self.eye)],
            [up.x, up.y, up.z, -up.dot(self.eye)],
            [forward.x, forward.y, forward.z, -forward.dot(self.eye)],
            [0.0, 0.0, 0.0, 1.0],
        ]
    }
}

#[derive(Clone, Copy)]
struct SurfelPoint {
    position: Vec3,
    normal: Vec3,
    count: f32,
    class_id: f32,
}

struct AppState {
    points: Arc<[GpuPoint]>,
    camera: Camera,
    drag_mode: Option<DragMode>,
    last_mouse: Option<Point>,
    source: String,
    counts: [usize; 5],
}

impl AppState {
    fn load() -> Result<Self> {
        let path = std::env::args()
            .nth(1)
            .context("usage: fastlio-surfel-viewer <surfel-map.pcd>")?;
        let surfels = read_surfels(&path)?;
        let mut counts = [0; 5];
        for surfel in &surfels {
            let class = surfel.class_id.round().clamp(0.0, 4.0) as usize;
            counts[class] += 1;
        }
        let target = centroid(&surfels);
        let radius = surfels
            .iter()
            .map(|surfel| {
                surfel
                    .position
                    .sub(target)
                    .dot(surfel.position.sub(target))
                    .sqrt()
            })
            .fold(INITIAL_RADIUS, f32::max)
            * 1.5;
        let points = surfels
            .iter()
            .map(GpuPoint::from_surfel)
            .collect::<Vec<_>>()
            .into();
        Ok(Self {
            points,
            camera: Camera::new(target, radius),
            drag_mode: None,
            last_mouse: None,
            source: path,
            counts,
        })
    }

    fn subscription(&self) -> Subscription<Message> {
        event::listen_with(|event, _, _| match event {
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                Some(Message::MousePressed(DragMode::Orbit))
            }
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Middle)) => {
                Some(Message::MousePressed(DragMode::Pan))
            }
            Event::Mouse(mouse::Event::ButtonReleased(_)) => Some(Message::MouseReleased),
            Event::Mouse(mouse::Event::CursorMoved { position }) => {
                Some(Message::MouseMoved(position))
            }
            Event::Mouse(mouse::Event::WheelScrolled { delta }) => {
                Some(Message::MouseWheel(match delta {
                    mouse::ScrollDelta::Lines { y, .. } => y,
                    mouse::ScrollDelta::Pixels { y, .. } => y / 30.0,
                }))
            }
            _ => None,
        })
    }

    fn update(&mut self, message: Message) {
        match message {
            Message::MousePressed(mode) => {
                self.drag_mode = Some(mode);
                self.last_mouse = None;
            }
            Message::MouseReleased => {
                self.drag_mode = None;
                self.last_mouse = None;
            }
            Message::MouseMoved(position) => {
                if let (Some(mode), Some(last)) = (self.drag_mode, self.last_mouse) {
                    let dx = position.x - last.x;
                    let dy = position.y - last.y;
                    match mode {
                        DragMode::Orbit => self.camera.orbit(dx, dy),
                        DragMode::Pan => self.camera.pan(dx, dy),
                    }
                }
                self.last_mouse = Some(position);
            }
            Message::MouseWheel(delta) => self.camera.zoom(delta),
        }
    }

    fn view(&self) -> Element<'_, Message> {
        let [plane, line, scatter, degenerate, growing] = self.counts;
        column![
            text(format!(
                "{} | surfels: {} | Growing: {} Plane: {} Line: {} Scatter: {} Degenerate: {}",
                self.source,
                self.points.len(),
                growing,
                plane,
                line,
                scatter,
                degenerate
            )),
            Shader::new(SurfelProgram::new(self.points.clone(), &self.camera))
                .width(Length::Fill)
                .height(Length::Fill),
        ]
        .into()
    }
}

impl Default for AppState {
    fn default() -> Self {
        Self::load().unwrap_or_else(|error| {
            eprintln!("{error:#}");
            std::process::exit(2)
        })
    }
}

fn field_index(fields: &[(usize, String)], name: &str) -> Option<usize> {
    fields
        .iter()
        .find(|(_, field)| field.eq_ignore_ascii_case(name))
        .map(|(index, _)| *index)
}

fn field_f32(field: &Field) -> Option<f32> {
    match field {
        Field::F32(values) => values.first().copied(),
        Field::F64(values) => values.first().map(|v| *v as f32),
        Field::U32(values) => values.first().map(|v| *v as f32),
        Field::I32(values) => values.first().map(|v| *v as f32),
        Field::U8(values) => values.first().map(|v| *v as f32),
        Field::U16(values) => values.first().map(|v| *v as f32),
        Field::U64(values) => values.first().map(|v| *v as f32),
        Field::I8(values) => values.first().map(|v| *v as f32),
        Field::I16(values) => values.first().map(|v| *v as f32),
        Field::I64(values) => values.first().map(|v| *v as f32),
    }
}

fn read_surfels(path: &str) -> Result<Vec<SurfelPoint>> {
    let reader =
        DynReader::open(path).with_context(|| format!("failed to open surfel PCD `{path}`"))?;
    let fields = reader
        .meta()
        .field_defs
        .iter()
        .filter(|field| !field.is_padding())
        .enumerate()
        .map(|(i, f)| (i, f.name.clone()))
        .collect::<Vec<_>>();
    let required =
        |name: &str| field_index(&fields, name).with_context(|| format!("PCD is missing `{name}`"));
    let x = required("x")?;
    let y = required("y")?;
    let z = required("z")?;
    let nx = field_index(&fields, "normal_x");
    let ny = field_index(&fields, "normal_y");
    let nz = field_index(&fields, "normal_z");
    let count = field_index(&fields, "intensity");
    let class = field_index(&fields, "class_id");
    reader
        .map(|record| {
            let fields = record?.0;
            let get =
                |index: usize| field_f32(&fields[index]).context("PCD field has no scalar value");
            Ok(SurfelPoint {
                position: Vec3 {
                    x: get(x)?,
                    y: get(y)?,
                    z: get(z)?,
                },
                normal: Vec3 {
                    x: nx.map(&get).transpose()?.unwrap_or(0.0),
                    y: ny.map(&get).transpose()?.unwrap_or(0.0),
                    z: nz.map(&get).transpose()?.unwrap_or(1.0),
                },
                count: count.map(&get).transpose()?.unwrap_or(1.0),
                class_id: class.map(get).transpose()?.unwrap_or(0.0),
            })
        })
        .collect::<std::result::Result<Vec<_>, anyhow::Error>>()
}

fn centroid(points: &[SurfelPoint]) -> Vec3 {
    if points.is_empty() {
        return Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
    }
    let sum = points.iter().fold(
        Vec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        |sum, p| sum.add(p.position),
    );
    sum.scale(1.0 / points.len() as f32)
}

#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
struct GpuPoint {
    position: [f32; 3],
    color: [f32; 3],
    radius: f32,
}

impl GpuPoint {
    fn from_surfel(surfel: &SurfelPoint) -> Self {
        let normal_z = surfel.normal.normalize().z.abs();
        let color = match surfel.class_id.round() as i32 {
            0 => [0.15 + 0.35 * normal_z, 0.55 + 0.35 * normal_z, 0.30],
            1 => [0.95, 0.60, 0.15],
            2 => [0.95, 0.20, 0.20],
            3 => [0.75, 0.20, 0.85],
            _ => [0.45, 0.55, 0.95],
        };
        let radius = 2.0 + surfel.count.ln_1p().clamp(0.0, 5.0);
        Self {
            position: [surfel.position.x, surfel.position.y, surfel.position.z],
            color,
            radius,
        }
    }

    const ATTRIBUTES: [iced::wgpu::VertexAttribute; 3] =
        iced::wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32];
    fn layout() -> iced::wgpu::VertexBufferLayout<'static> {
        iced::wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Self>() as u64,
            step_mode: iced::wgpu::VertexStepMode::Instance,
            attributes: &Self::ATTRIBUTES,
        }
    }
}

struct SurfelProgram<'a> {
    points: Arc<[GpuPoint]>,
    camera: &'a Camera,
}
impl<'a> SurfelProgram<'a> {
    fn new(points: Arc<[GpuPoint]>, camera: &'a Camera) -> Self {
        Self { points, camera }
    }
}
impl<Message> shader::Program<Message> for SurfelProgram<'_> {
    type State = ();
    type Primitive = SurfelPrimitive;
    fn draw(&self, _: &Self::State, _: mouse::Cursor, bounds: Rectangle) -> Self::Primitive {
        SurfelPrimitive {
            points: self.points.clone(),
            uniforms: Uniforms {
                view: self.camera.view(),
                viewport: [bounds.width.max(1.0), bounds.height.max(1.0)],
                scale: POINT_SCALE,
                _padding: 0.0,
            },
        }
    }
}

#[derive(Debug)]
struct SurfelPrimitive {
    points: Arc<[GpuPoint]>,
    uniforms: Uniforms,
}
impl Primitive for SurfelPrimitive {
    type Pipeline = SurfelPipeline;
    fn prepare(
        &self,
        pipeline: &mut Self::Pipeline,
        device: &iced::wgpu::Device,
        queue: &iced::wgpu::Queue,
        _: &Rectangle,
        _: &shader::Viewport,
    ) {
        pipeline.prepare(device, queue, self.points.clone(), &self.uniforms);
    }
    fn draw(&self, pipeline: &Self::Pipeline, pass: &mut iced::wgpu::RenderPass<'_>) -> bool {
        pipeline.draw(pass);
        true
    }
}

struct SurfelPipeline {
    pipeline: iced::wgpu::RenderPipeline,
    uniform: iced::wgpu::Buffer,
    bind: iced::wgpu::BindGroup,
    buffer: Option<iced::wgpu::Buffer>,
    source: Option<(usize, usize)>,
    count: u32,
}
impl Pipeline for SurfelPipeline {
    fn new(
        device: &iced::wgpu::Device,
        _: &iced::wgpu::Queue,
        format: iced::wgpu::TextureFormat,
    ) -> Self {
        let shader = device.create_shader_module(iced::wgpu::ShaderModuleDescriptor {
            label: Some("surfel shader"),
            source: iced::wgpu::ShaderSource::Wgsl(include_str!("surfel.wgsl").into()),
        });
        let uniform = device.create_buffer(&iced::wgpu::BufferDescriptor {
            label: Some("surfel uniforms"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: iced::wgpu::BufferUsages::UNIFORM | iced::wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let layout = device.create_bind_group_layout(&iced::wgpu::BindGroupLayoutDescriptor {
            label: Some("surfel bind layout"),
            entries: &[iced::wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: iced::wgpu::ShaderStages::VERTEX,
                ty: iced::wgpu::BindingType::Buffer {
                    ty: iced::wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: iced::wgpu::BufferSize::new(
                        std::mem::size_of::<Uniforms>() as u64
                    ),
                },
                count: None,
            }],
        });
        let bind = device.create_bind_group(&iced::wgpu::BindGroupDescriptor {
            label: Some("surfel bind"),
            layout: &layout,
            entries: &[iced::wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform.as_entire_binding(),
            }],
        });
        let pipeline_layout =
            device.create_pipeline_layout(&iced::wgpu::PipelineLayoutDescriptor {
                label: Some("surfel pipeline layout"),
                bind_group_layouts: &[&layout],
                push_constant_ranges: &[],
            });
        let pipeline = device.create_render_pipeline(&iced::wgpu::RenderPipelineDescriptor {
            label: Some("surfel pipeline"),
            layout: Some(&pipeline_layout),
            vertex: iced::wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[GpuPoint::layout()],
                compilation_options: Default::default(),
            },
            fragment: Some(iced::wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                targets: &[Some(iced::wgpu::ColorTargetState {
                    format,
                    blend: Some(iced::wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: iced::wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: iced::wgpu::PrimitiveState::default(),
            depth_stencil: None,
            multisample: Default::default(),
            multiview: None,
            cache: None,
        });
        Self {
            pipeline,
            uniform,
            bind,
            buffer: None,
            source: None,
            count: 0,
        }
    }
}
impl SurfelPipeline {
    fn prepare(
        &mut self,
        device: &iced::wgpu::Device,
        queue: &iced::wgpu::Queue,
        points: Arc<[GpuPoint]>,
        uniforms: &Uniforms,
    ) {
        queue.write_buffer(&self.uniform, 0, bytemuck::bytes_of(uniforms));
        self.count = points.len().min(u32::MAX as usize) as u32;
        let source = (
            Arc::as_ptr(&points) as *const GpuPoint as usize,
            points.len(),
        );
        if self.source == Some(source) {
            return;
        }
        self.buffer = (!points.is_empty()).then(|| {
            device.create_buffer_init(&iced::wgpu::util::BufferInitDescriptor {
                label: Some("surfel points"),
                contents: bytemuck::cast_slice(&points),
                usage: iced::wgpu::BufferUsages::VERTEX,
            })
        });
        self.source = Some(source);
    }
    fn draw(&self, pass: &mut iced::wgpu::RenderPass<'_>) {
        let Some(buffer) = &self.buffer else { return };
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &self.bind, &[]);
        pass.set_vertex_buffer(0, buffer.slice(..));
        pass.draw(0..6, 0..self.count);
    }
}

#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
struct Uniforms {
    view: [[f32; 4]; 4],
    viewport: [f32; 2],
    scale: f32,
    _padding: f32,
}

fn main() -> iced::Result {
    iced::application(AppState::default, AppState::update, AppState::view)
        .title("Surfel Viewer")
        .subscription(AppState::subscription)
        .run()
}
