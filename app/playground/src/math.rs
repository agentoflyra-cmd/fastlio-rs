use std::ops::{Add, AddAssign, Sub, SubAssign};

#[derive(Debug, Clone, Copy)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vector3 {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub const Z: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    };

    pub fn cross(&self, rhs: Self) -> Self {
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
            Self {
                x: self.x / norm,
                y: self.y / norm,
                z: self.z / norm,
            }
        } else {
            self
        }
    }
}

impl Sub for Vector3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl Add for Vector3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
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

pub struct Transform4x4 {
    pub matrix: [f32; 16],
}

impl Transform4x4 {
    pub fn identity() -> Self {
        Self {
            matrix: [
                1.0, 0.0, 0.0, 0.0, //
                0.0, 1.0, 0.0, 0.0, //
                0.0, 0.0, 1.0, 0.0, //
                0.0, 0.0, 0.0, 1.0,
            ],
        }
    }

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
