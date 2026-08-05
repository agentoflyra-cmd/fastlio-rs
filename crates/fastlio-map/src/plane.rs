use std::cmp::Ordering;

use anyhow::{Result, anyhow};
use fastlio_types::{Mat3, PointXYZI, Vec3};

#[derive(Debug, Clone, PartialEq)]
pub struct PlaneFit {
    pub centroid_w: Vec3<f64>,
    pub normal_w: Vec3<f64>,
    pub offset: f64,
    pub eigenvalues: Vec3<f64>,
    pub planarity_ratio: f64,
}

pub struct PlaneConfig {
    pub required: usize,
    pub min_spread_eigenvalues: f64,
    pub max_planarity_ratio: f64,
}

pub fn plane_fit(points: &[&PointXYZI], config: &PlaneConfig) -> Result<PlaneFit> {
    let required = config.required;
    let nums = points.len();
    if nums < required {
        return Err(anyhow!(
            "PlaneFitError: not enough points: {} vs {}",
            nums,
            required
        ));
    }

    let mut nums = 0;
    let mut centroid_w = Vec3::<f64>::zeros();
    for point in points {
        if !point.is_valid() {
            continue;
        }
        nums += 1;
        centroid_w += point.to_vec3_f64();
    }
    centroid_w /= nums as f64;

    let mut convariance = Mat3::<f64>::zeros();
    for point in points {
        if !point.is_valid() {
            continue;
        }
        let delta = point.to_vec3_f64() - centroid_w;
        convariance += delta * delta.transpose();
    }
    convariance /= nums as f64;

    let eigen = convariance.symmetric_eigen();
    let eigen_values = eigen.eigenvalues;
    let eigen_vectors = eigen.eigenvectors;
    let mut eigen_pairs = [
        (eigen_values[0], eigen_vectors.column(0).into_owned()),
        (eigen_values[1], eigen_vectors.column(1).into_owned()),
        (eigen_values[2], eigen_vectors.column(2).into_owned()),
    ];
    eigen_pairs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal));
    let mut smallest = eigen_pairs[0];
    let mut middle = eigen_pairs[1];
    let mut largest = eigen_pairs[2];

    smallest.0 = smallest.0.max(0.0);
    middle.0 = middle.0.max(0.0);
    largest.0 = largest.0.max(0.0);

    if middle.0 <= config.min_spread_eigenvalues || largest.0 <= config.min_spread_eigenvalues {
        return Err(anyhow!("PlaneFitError: neighbour degenerated."));
    }

    let planarity_ratio = smallest.0 / middle.0;
    if planarity_ratio >= config.max_planarity_ratio {
        return Err(anyhow!(
            "PlaneFitError: planarity_ratio exceed threshold.Not Planner."
        ));
    }

    let normal_w = smallest.1.normalize();
    let plane_offset = -normal_w.dot(&centroid_w);

    Ok(PlaneFit {
        centroid_w,
        normal_w,
        offset: plane_offset,
        eigenvalues: Vec3::new(smallest.0, middle.0, largest.0),
        planarity_ratio,
    })
}
