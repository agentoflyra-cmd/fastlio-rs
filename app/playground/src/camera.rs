use crate::math::Vector3;

const MIN_CAMERA_RADIUS: f32 = 0.1;
const MAX_CAMERA_RADIUS: f32 = 100.0;

pub struct CameraView {
    pub eye: Vector3,
    pub target: Vector3,
    pub up: Vector3,
}

impl CameraView {
    pub fn new(orbit_camera: &OrbitCamera, target: Vector3) -> Self {
        Self {
            eye: orbit_camera.calculate(target),
            target,
            up: Vector3::Z,
        }
    }

    pub fn update_orientation(&mut self, orbit_camera: &mut OrbitCamera, yaw: f32, pitch: f32) {
        orbit_camera.rotate(yaw, pitch);
        self.eye = orbit_camera.calculate(self.target);
    }

    pub fn update_radius(&mut self, orbit_camera: &mut OrbitCamera, radius: f32) {
        orbit_camera.zoom(radius);
        self.eye = orbit_camera.calculate(self.target);
    }

    pub fn pan_screen_delta(&mut self, dx: f32, dy: f32, scale: f32) {
        let forward = (self.eye - self.target).normalize();
        let right = forward.cross(self.up).normalize();
        let true_up = right.cross(forward);
        let scale = scale.max(1.0);
        let translation = Vector3 {
            x: -right.x * dx / scale + true_up.x * dy / scale,
            y: -right.y * dx / scale + true_up.y * dy / scale,
            z: -right.z * dx / scale + true_up.z * dy / scale,
        };

        self.eye += translation;
        self.target += translation;
    }

    pub fn get_view_matrix(&self) -> [[f32; 4]; 4] {
        let forward = (self.eye - self.target).normalize();
        let right = forward.cross(self.up).normalize();
        let true_up = right.cross(forward);

        [
            [right.x, right.y, right.z, -right.dot(self.eye)],
            [true_up.x, true_up.y, true_up.z, -true_up.dot(self.eye)],
            [forward.x, forward.y, forward.z, -forward.dot(self.eye)],
            [0.0, 0.0, 0.0, 1.0],
        ]
    }
}

pub struct OrbitCamera {
    pub radius: f32,
    pub yaw: f32,
    pub pitch: f32,
}

impl OrbitCamera {
    pub fn new(radius: f32) -> Self {
        Self {
            radius,
            yaw: 0.0,
            pitch: 0.0,
        }
    }

    pub fn rotate(&mut self, delta_yaw: f32, delta_pitch: f32) {
        self.yaw += delta_yaw;
        let max_pitch = 85.0f32.to_radians();
        self.pitch = (self.pitch + delta_pitch).clamp(-max_pitch, max_pitch);
    }

    pub fn zoom(&mut self, delta_radius: f32) {
        self.radius = (self.radius + delta_radius).clamp(MIN_CAMERA_RADIUS, MAX_CAMERA_RADIUS);
    }

    pub fn calculate(&self, target: Vector3) -> Vector3 {
        let cos_pitch = self.pitch.cos();
        let sin_pitch = self.pitch.sin();
        let cos_yaw = self.yaw.cos();
        let sin_yaw = self.yaw.sin();

        target
            + Vector3 {
                x: self.radius * cos_pitch * sin_yaw,
                y: self.radius * cos_pitch * cos_yaw,
                z: self.radius * sin_pitch,
            }
    }
}
