use pcd_rs::{DynReader, Field};
use rand::RngExt;
use std::error::Error;
use std::fmt;
use std::path::Path;

use crate::math::{Transform4x4, Vector3};

#[derive(Debug, Clone)]
pub struct PointXYZI {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
    pub normal: Option<[f32; 3]>,
}

pub struct CoordsObservation {
    pub local_body: Vector3,
    pub extrinsic: Transform4x4,
    pub points: Vec<PointXYZI>,
}

impl CoordsObservation {
    pub fn new(axis: Vector3, points: Vec<PointXYZI>) -> Self {
        Self {
            local_body: axis.normalize(),
            extrinsic: Transform4x4::identity(),
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
            .map(|p| self.extrinsic.transform_point(p))
            .collect();

        self.local_body = transformed_body;
        self.points = transformed_points;
    }
}

impl Transform4x4 {
    pub fn transform_point(&self, point: &PointXYZI) -> PointXYZI {
        let mat = &self.matrix;

        PointXYZI {
            x: mat[0] * point.x + mat[1] * point.y + mat[2] * point.z + mat[3],
            y: mat[4] * point.x + mat[5] * point.y + mat[6] * point.z + mat[7],
            z: mat[8] * point.x + mat[9] * point.y + mat[10] * point.z + mat[11],
            intensity: point.intensity,
            normal: point.normal.map(|[x, y, z]| {
                let normal = self.transform_vector(Vector3 { x, y, z }).normalize();

                [normal.x, normal.y, normal.z]
            }),
        }
    }
}

pub fn random_point_cloud(count: usize) -> Vec<PointXYZI> {
    let mut rng = rand::rng();

    (0..count)
        .map(|_| PointXYZI {
            x: rng.random_range(-5.0..5.0),
            y: rng.random_range(-5.0..5.0),
            z: rng.random_range(-5.0..5.0),
            intensity: rng.random_range(0.0..1.0),
            normal: None,
        })
        .collect()
}

#[derive(Debug)]
pub enum PcdError {
    Read(pcd_rs::Error),
    MissingField(&'static str),
    MissingValue(&'static str),
}

impl fmt::Display for PcdError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Read(err) => write!(f, "{err}"),
            Self::MissingField(field) => write!(f, "PCD FIELDS is missing '{field}'"),
            Self::MissingValue(field) => write!(f, "PCD field '{field}' has no scalar value"),
        }
    }
}

impl Error for PcdError {}

impl From<pcd_rs::Error> for PcdError {
    fn from(err: pcd_rs::Error) -> Self {
        Self::Read(err)
    }
}

pub fn read_pcd(path: impl AsRef<Path>) -> Result<Vec<PointXYZI>, PcdError> {
    let reader = DynReader::open(path)?;
    let fields = reader
        .meta()
        .field_defs
        .iter()
        .filter(|field| !field.is_padding())
        .enumerate()
        .map(|(record_index, field)| (record_index, field.name.clone()))
        .collect::<Vec<_>>();

    let x_index = field_index(&fields, "x")?;
    let y_index = field_index(&fields, "y")?;
    let z_index = field_index(&fields, "z")?;
    let intensity_index = fields
        .iter()
        .find(|(_, field)| field.eq_ignore_ascii_case("intensity"))
        .map(|(index, _)| *index);
    let normal_indices = match (
        optional_field_index(&fields, "normal_x"),
        optional_field_index(&fields, "normal_y"),
        optional_field_index(&fields, "normal_z"),
    ) {
        (Some(x), Some(y), Some(z)) => Some([x, y, z]),
        _ => None,
    };

    reader
        .map(|record| {
            let record = record?;
            let normal = match normal_indices {
                Some([x_index, y_index, z_index]) => Some([
                    field_as_f32(&record.0[x_index], "normal_x")?,
                    field_as_f32(&record.0[y_index], "normal_y")?,
                    field_as_f32(&record.0[z_index], "normal_z")?,
                ]),
                None => None,
            };

            Ok(PointXYZI {
                x: field_as_f32(&record.0[x_index], "x")?,
                y: field_as_f32(&record.0[y_index], "y")?,
                z: field_as_f32(&record.0[z_index], "z")?,
                intensity: match intensity_index {
                    Some(index) => field_as_f32(&record.0[index], "intensity")?,
                    None => 1.0,
                },
                normal,
            })
        })
        .collect()
}

fn field_index(fields: &[(usize, String)], name: &'static str) -> Result<usize, PcdError> {
    fields
        .iter()
        .find(|(_, field)| field.eq_ignore_ascii_case(name))
        .map(|(index, _)| *index)
        .ok_or(PcdError::MissingField(name))
}

fn optional_field_index(fields: &[(usize, String)], name: &'static str) -> Option<usize> {
    fields
        .iter()
        .find(|(_, field)| field.eq_ignore_ascii_case(name))
        .map(|(index, _)| *index)
}

fn field_as_f32(field: &Field, name: &'static str) -> Result<f32, PcdError> {
    let value = match field {
        Field::I8(values) => values.first().copied().map(f32::from),
        Field::I16(values) => values.first().copied().map(f32::from),
        Field::I32(values) => values.first().copied().map(|value| value as f32),
        Field::I64(values) => values.first().copied().map(|value| value as f32),
        Field::U8(values) => values.first().copied().map(f32::from),
        Field::U16(values) => values.first().copied().map(f32::from),
        Field::U32(values) => values.first().copied().map(|value| value as f32),
        Field::U64(values) => values.first().copied().map(|value| value as f32),
        Field::F32(values) => values.first().copied(),
        Field::F64(values) => values.first().copied().map(|value| value as f32),
    };

    value.ok_or(PcdError::MissingValue(name))
}

#[cfg(test)]
mod tests {
    use super::read_pcd;

    #[test]
    fn reads_local_binary_pcd_when_available() {
        let path = "../fast-lio-sam/test.pcd";

        if !std::path::Path::new(path).exists() {
            return;
        }

        let points = read_pcd(path).unwrap();
        assert_eq!(points.len(), 409648);
        assert!(points.iter().any(|point| point.normal.is_some()));
    }
}
