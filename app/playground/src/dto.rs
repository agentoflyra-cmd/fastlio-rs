use std::ops::{Add, AddAssign, Sub, SubAssign};

const MIN_CAMERA_RADIUS: f32 = 0.0;
const MAX_CAMERA_RADIUS: f32 = 100.0;

#[derive(Debug, Clone, Copy)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Sub for Vector3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Vector3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}
impl Add for Vector3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Vector3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl SubAssign for Vector3 {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl AddAssign for Vector3 {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl Vector3 {
    pub fn cross(&self, rhs: Self) -> Self {
        // X = ya * zb - za * yb
        // Y = za * xb - xa * zb
        // Z = xa * yb - ya * xb
        Self {
            x: self.y * rhs.z - self.z * rhs.y,
            y: self.z * rhs.x - self.x * rhs.z,
            z: self.x * rhs.y - self.y * rhs.x,
        }
    }

    pub fn dot(&self, rhs: Self) -> f32 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    pub fn normalize(self) -> Self {
        let norm = (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt();
        if norm > 0.0 {
            Vector3 {
                x: self.x / norm,
                y: self.y / norm,
                z: self.z / norm,
            }
        } else {
            self
        }
    }
}

pub struct Transform4x4 {
    pub matrix: [f32; 16],
}

impl Transform4x4 {
    pub fn new(axis: Vector3, rotate: f32, translation: [f32; 3]) -> Self {
        let [tx, ty, tz] = translation;
        let Vector3 {
            x: kx,
            y: ky,
            z: kz,
        } = axis.normalize();
        let c = rotate.cos();
        let s = rotate.sin();
        let v = 1.0 - c;
        let matrix = [
            kx.powi(2) * v + c,
            kx * ky * v - kz * s,
            kx * kz * v + ky * s,
            tx,
            kx * ky * v + kz * s,
            ky.powi(2) * v + c,
            ky * kz * v - kx * s,
            ty,
            kx * kz * v - ky * s,
            ky * kz * v + kx * s,
            kz.powi(2) * v + c,
            tz,
            0.0,
            0.0,
            0.0,
            1.0,
        ];

        Self { matrix }
    }

    pub fn transform_vector(&self, v: Vector3) -> Vector3 {
        Vector3 {
            x: self.matrix[0] * v.x + self.matrix[1] * v.y + self.matrix[2] * v.z,
            y: self.matrix[4] * v.x + self.matrix[5] * v.y + self.matrix[6] * v.z,
            z: self.matrix[8] * v.x + self.matrix[9] * v.y + self.matrix[10] * v.z,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PointXYZI {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
}

pub struct CoordsObservation {
    pub local_body: Vector3,
    pub extrinsic: Transform4x4,
    pub points: Vec<PointXYZI>,
}

impl CoordsObservation {
    pub fn new(axis: Vector3, points: Vec<PointXYZI>) -> Self {
        let extrinsic = Transform4x4 {
            matrix: [
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
        };
        let Vector3 {
            x: kx,
            y: ky,
            z: kz,
        } = axis.normalize();
        Self {
            local_body: Vector3 {
                x: kx,
                y: ky,
                z: kz,
            },
            extrinsic,
            points,
        }
    }

    pub fn transform(&mut self, rotate: f32, translation: [f32; 3]) {
        self.extrinsic = Transform4x4::new(self.local_body, rotate, translation);
    }

    pub fn transform_to_new_axes(&mut self) {
        let transformed_body = self.extrinsic.transform_vector(self.local_body).normalize();

        let transformed_points = self
            .points
            .iter()
            .map(|p| {
                let mat = &self.extrinsic.matrix;
                PointXYZI {
                    x: mat[0] * p.x + mat[1] * p.y + mat[2] * p.z + mat[3],
                    y: mat[4] * p.x + mat[5] * p.y + mat[6] * p.z + mat[7],
                    z: mat[8] * p.x + mat[9] * p.y + mat[10] * p.z + mat[11],
                    intensity: p.intensity,
                }
            })
            .collect();

        // CoordsObservation {
        //     local_body: transformed_body,
        //     extrinsic:
        //     points: transformed_points,
        // }
        self.local_body = transformed_body;
        self.points = transformed_points;
    }
    pub fn rotate(&self, rotate: f32, translation: [f32; 3]) -> Vec<PointXYZI> {
        let Vector3 {
            x: kx,
            y: ky,
            z: kz,
        } = self.local_body;
        let c = rotate.cos();
        let s = rotate.sin();
        let v = 1.0 - c;
        let [tx, ty, tz] = translation;

        let r00 = kx.powi(2) * v + c;
        let r01 = kx * ky * v - kz * s;
        let r02 = kx * kz * v + ky * s;
        let r10 = kx * ky * v + kz * s;
        let r11 = ky.powi(2) * v + c;
        let r12 = ky * kz * v - kx * s;
        let r20 = kx * kz * v - ky * s;
        let r21 = ky * kz * v + kx * s;
        let r22 = kz.powi(2) * v + c;

        self.points
            .iter()
            .map(|p| PointXYZI {
                x: r00 * p.x + r01 * p.y + r02 * p.z + tx,
                y: r10 * p.x + r11 * p.y + r12 * p.z + ty,
                z: r20 * p.x + r21 * p.y + r22 * p.z + tz,
                intensity: p.intensity,
            })
            .collect()
    }
}

pub struct CameraView {
    // where camera
    pub eye: Vector3,
    // look at
    pub target: Vector3,
    // head to
    pub up: Vector3,
}

impl CameraView {
    pub fn update_orientation(&mut self, orbit_camera: &mut OrbitCamera, yaw: f32, pitch: f32) {
        orbit_camera.rotate(yaw, pitch);
        self.eye = orbit_camera.caculate(self.target);
    }

    pub fn update_radius(&mut self, orbit_camera: &mut OrbitCamera, radius: f32) {
        orbit_camera.zoom(radius);
        self.eye = orbit_camera.caculate(self.target);
    }

    pub fn get_view_matrix(&self) -> [[f32; 4]; 4] {
        let forward = self.eye - self.target;
        let forward = forward.normalize();

        let left = self.up.cross(forward);
        let left = left.normalize();

        let true_up = forward.cross(left);

        let first_line = [left.x, left.y, left.z, -left.dot(self.eye)];
        let second_line = [true_up.x, true_up.y, true_up.z, -true_up.dot(self.eye)];
        let third_line = [forward.x, forward.y, forward.z, -forward.dot(self.eye)];
        let fourth_line = [0.0, 0.0, 0.0, 1.0f32];

        [first_line, second_line, third_line, fourth_line]
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

    pub fn caculate(&self, target: Vector3) -> Vector3 {
        let cos_pitch = self.pitch.cos();
        let sin_pitch = self.pitch.sin();
        let cos_yaw = self.yaw.cos();
        let sin_yaw = self.yaw.sin();

        let offset_x = self.radius * cos_pitch * sin_yaw;
        let offset_y = self.radius * sin_pitch;
        let offset_z = self.radius * cos_pitch * cos_yaw;

        let vector3 = Vector3 {
            x: offset_x,
            y: offset_y,
            z: offset_z,
        };
        target + vector3
    }
}
