use iced::widget::{column, text};
use iced::{Element, Event::Mouse, Point, Subscription, event, mouse};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::camera::{CameraView, OrbitCamera};
use crate::math::Vector3;
use crate::point_cloud::{CoordsObservation, PointXYZI, random_point_cloud, read_pcd};
use crate::scene::point_cloud_scene;
use crate::stream::{StreamMessage, StreamReceiver, spawn_point_stream_listener};
use crate::wgpu_point_cloud::GpuPoint;

const MOUSE_ROTATION_SPEED: f32 = 0.005;
const INITIAL_CAMERA_RADIUS: f32 = 10.0;
const INITIAL_POINT_SCALE: f32 = 35.0;
const MIN_POINT_SCALE: f32 = 1.0;
const MAX_POINT_SCALE: f32 = 1000.0;
const POINT_SCALE_SCROLL_FACTOR: f32 = 1.1;
const INITIAL_POINT_COUNT: usize = 1000;
const STREAM_TICK_MS: u64 = 33;

#[derive(Debug, Clone)]
pub struct AppConfig {
    pub pcd_path: Option<String>,
    pub listen_addr: Option<String>,
}

impl AppConfig {
    pub fn parse(args: Vec<String>) -> Self {
        let mut pcd_path = None;
        let mut listen_addr = None;
        let mut idx = 0;
        while idx < args.len() {
            match args[idx].as_str() {
                "--listen" => {
                    listen_addr = args.get(idx + 1).cloned();
                    idx += 2;
                }
                path if !path.starts_with("--") && pcd_path.is_none() => {
                    pcd_path = Some(path.to_string());
                    idx += 1;
                }
                _ => {
                    idx += 1;
                }
            }
        }
        Self {
            pcd_path,
            listen_addr,
        }
    }
}

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub enum Message {
    CameraRotate {
        yaw: f32,
        pitch: f32,
    },
    CameraZoomed(f32),
    ResetView,
    SensorTransform {
        rotation: f32,
        translation: [f32; 3],
    },
    NewPointCloudReceived(Vec<PointXYZI>),
    MousePressed(DragMode),
    MouseReleased,
    MouseMoved(Point),
    MouseWheel(f32),
    StreamTick(Instant),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DragMode {
    Orbit,
    Pan,
}

pub struct AppState {
    drag_mode: Option<DragMode>,
    last_mouse_position: Option<Point>,
    orbit_camera: OrbitCamera,
    camera: CameraView,
    coords: CoordsObservation,
    gpu_points: Arc<[GpuPoint]>,
    point_scale: f32,
    source: String,
    stream_receiver: Option<StreamReceiver>,
    stream_status: Option<String>,
}

impl AppState {
    pub fn new(config: AppConfig) -> Self {
        let orbit_camera = OrbitCamera::new(INITIAL_CAMERA_RADIUS);
        let camera = CameraView::new(&orbit_camera, Vector3::ZERO);
        let (points, source) = load_initial_point_cloud(config.pcd_path.as_deref());
        let gpu_points = build_gpu_points(&points);
        let coords = CoordsObservation::new(Vector3::Z, points);
        let stream_receiver = config.listen_addr.map(spawn_point_stream_listener);

        Self {
            orbit_camera,
            camera,
            coords,
            gpu_points,
            point_scale: INITIAL_POINT_SCALE,
            source,
            stream_receiver,
            stream_status: None,
            drag_mode: None,
            last_mouse_position: None,
        }
    }

    pub fn subscription(&self) -> Subscription<Message> {
        let input = event::listen_with(|event, _status, _window| match event {
            Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                Some(Message::MousePressed(DragMode::Orbit))
            }
            Mouse(mouse::Event::ButtonPressed(mouse::Button::Middle)) => {
                Some(Message::MousePressed(DragMode::Pan))
            }
            Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => {
                Some(Message::MouseReleased)
            }
            Mouse(mouse::Event::ButtonReleased(mouse::Button::Middle)) => {
                Some(Message::MouseReleased)
            }
            Mouse(mouse::Event::CursorMoved { position }) => Some(Message::MouseMoved(position)),
            Mouse(mouse::Event::WheelScrolled { delta }) => {
                Some(Message::MouseWheel(scroll_y(delta)))
            }
            _ => None,
        });

        let stream = if self.stream_receiver.is_some() {
            iced::time::every(Duration::from_millis(STREAM_TICK_MS)).map(Message::StreamTick)
        } else {
            Subscription::none()
        };

        Subscription::batch([input, stream])
    }

    pub fn update(&mut self, message: Message) {
        match message {
            Message::CameraRotate { yaw, pitch } => {
                self.rotate_camera(MOUSE_ROTATION_SPEED * yaw, MOUSE_ROTATION_SPEED * pitch);
            }
            Message::CameraZoomed(radius) => {
                self.camera.update_radius(&mut self.orbit_camera, radius);
            }
            Message::ResetView => {
                self.orbit_camera = OrbitCamera::new(self.orbit_camera.radius);
                self.camera.update_radius(&mut self.orbit_camera, 0.0);
            }
            Message::SensorTransform {
                rotation,
                translation,
            } => {
                self.coords.transform(rotation, translation);
                self.coords.transform_to_new_axes();
                self.gpu_points = build_gpu_points(&self.coords.points);
            }
            Message::NewPointCloudReceived(points) => {
                self.coords.points.extend_from_slice(&points);
                self.gpu_points = build_gpu_points(&self.coords.points);
            }
            Message::MousePressed(mode) => {
                self.drag_mode = Some(mode);
                self.last_mouse_position = None;
            }
            Message::MouseReleased => {
                self.drag_mode = None;
                self.last_mouse_position = None;
            }
            Message::MouseMoved(position) => {
                self.handle_mouse_move(position);
            }
            Message::MouseWheel(delta) => {
                self.scale_points(delta);
            }
            Message::StreamTick(_) => {
                self.drain_stream();
            }
        }
    }

    pub fn view(&self) -> Element<'_, Message> {
        column![
            text(format!(
                "source: {} | points: {} | scale: {:.2} | stream: {}",
                self.source,
                self.coords.points.len(),
                self.point_scale,
                self.stream_status.as_deref().unwrap_or("off")
            )),
            point_cloud_scene(self.gpu_points.clone(), &self.camera, self.point_scale),
        ]
        .into()
    }

    fn handle_mouse_move(&mut self, position: Point) {
        if let (Some(mode), Some(last)) = (self.drag_mode, self.last_mouse_position) {
            let dx = position.x - last.x;
            let dy = position.y - last.y;

            match mode {
                DragMode::Orbit => {
                    self.rotate_camera(MOUSE_ROTATION_SPEED * dx, -MOUSE_ROTATION_SPEED * dy);
                }
                DragMode::Pan => {
                    self.camera.pan_screen_delta(dx, dy, self.point_scale);
                }
            }
        }

        self.last_mouse_position = Some(position);
    }

    fn rotate_camera(&mut self, yaw: f32, pitch: f32) {
        self.camera
            .update_orientation(&mut self.orbit_camera, yaw, pitch);
    }

    fn scale_points(&mut self, delta: f32) {
        let factor = POINT_SCALE_SCROLL_FACTOR.powf(delta);
        self.point_scale = (self.point_scale * factor).clamp(MIN_POINT_SCALE, MAX_POINT_SCALE);
    }

    fn drain_stream(&mut self) {
        let Some(receiver) = &self.stream_receiver else {
            return;
        };
        let messages = receiver.drain();
        if messages.is_empty() {
            return;
        }

        let mut changed = false;
        for message in messages {
            match message {
                StreamMessage::Clear => {
                    self.coords.points.clear();
                    changed = true;
                }
                StreamMessage::Points(points) => {
                    self.coords.points.extend(points);
                    changed = true;
                }
                StreamMessage::Status(status) => {
                    self.stream_status = Some(status);
                }
            }
        }
        if changed {
            self.gpu_points = build_gpu_points(&self.coords.points);
            self.source = "live stream".to_string();
        }
    }
}

impl Default for AppState {
    fn default() -> Self {
        Self::new(AppConfig {
            pcd_path: None,
            listen_addr: None,
        })
    }
}

fn scroll_y(delta: mouse::ScrollDelta) -> f32 {
    match delta {
        mouse::ScrollDelta::Lines { y, .. } => y,
        mouse::ScrollDelta::Pixels { y, .. } => y / 30.0,
    }
}

fn load_initial_point_cloud(path: Option<&str>) -> (Vec<PointXYZI>, String) {
    if let Some(path) = path {
        match read_pcd(path) {
            Ok(points) => return (points, path.to_string()),
            Err(err) => eprintln!("failed to read PCD file {path}: {err}"),
        }
    }

    (
        random_point_cloud(INITIAL_POINT_COUNT),
        "random point cloud".to_string(),
    )
}

fn build_gpu_points(points: &[PointXYZI]) -> Arc<[GpuPoint]> {
    let (min_z, max_z) = points.iter().fold(
        (f32::INFINITY, f32::NEG_INFINITY),
        |(min_z, max_z), point| (min_z.min(point.z), max_z.max(point.z)),
    );
    let z_span = (max_z - min_z).max(f32::EPSILON);

    points
        .iter()
        .map(|point| GpuPoint::from_point(point, (point.z - min_z) / z_span))
        .collect::<Vec<_>>()
        .into()
}
