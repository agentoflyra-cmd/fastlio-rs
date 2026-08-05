use fastlio_map::PlaneFit;
use fastlio_types::{Pose3, Vec3};
use nalgebra::{SMatrix, SVector, UnitQuaternion};

type Vec6 = SVector<f64, 6>;
type Mat6 = SMatrix<f64, 6, 6>;

/// One point-to-plane optimization factor.
///
/// `point_s` is expressed in the current scan/local frame `S`. `plane_w` is
/// expressed in the world/map frame `W`.
#[derive(Debug, Clone)]
pub struct PointToPlaneFactor {
    pub point_s: Vec3<f64>,
    pub plane_w: PlaneFit,
    pub weight: f64,
}

/// Pose-only Gauss-Newton configuration.
#[derive(Debug, Clone, Copy)]
pub struct PoseOptimizerConfig {
    pub max_iterations: usize,
    pub min_delta_norm: f64,
    pub damping: f64,
    pub huber_delta: Option<f64>,
}

impl Default for PoseOptimizerConfig {
    fn default() -> Self {
        Self {
            max_iterations: 10,
            min_delta_norm: 1.0e-6,
            damping: 1.0e-9,
            huber_delta: Some(0.1),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PoseOptimizationReport {
    pub initial_cost: f64,
    pub final_cost: f64,
    pub iterations: usize,
    pub converged: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum PoseOptimizationError {
    NotEnoughFactors { actual: usize, required: usize },
    NonFiniteInput,
    SingularSystem,
}

/// Optimize `T_WS`, which maps scan-frame points into the world/map frame:
/// `p_W = R_WS * p_S + t_WS`.
///
/// Perturbation convention:
///
/// - rotation uses a left perturbation, `R' = Exp(dtheta) * R`;
/// - translation is updated additively in the world frame, `t' = t + dt`;
/// - update vector order is `[dtheta_x, dtheta_y, dtheta_z, dt_x, dt_y, dt_z]`.
///
/// This stage intentionally implements only pose-only Gauss-Newton. It does not
/// build scan-to-map associations, Jacobians for ESEKF, covariance updates, or
/// degeneracy-aware state filtering.
pub fn optimize_pose_point_to_plane(
    initial_pose_ws: Pose3,
    factors: &[PointToPlaneFactor],
    config: PoseOptimizerConfig,
) -> Result<(Pose3, PoseOptimizationReport), PoseOptimizationError> {
    validate_inputs(&initial_pose_ws, factors)?;

    let mut pose_ws = initial_pose_ws;
    let initial_cost = total_cost(&pose_ws, factors, config.huber_delta);
    let mut final_cost = initial_cost;
    let mut iterations = 0;
    let mut converged = false;

    for _ in 0..config.max_iterations {
        let (normal_matrix, rhs) = build_normal_equations(&pose_ws, factors, config)?;
        let Some(delta) = normal_matrix.lu().solve(&rhs) else {
            return Err(PoseOptimizationError::SingularSystem);
        };

        if !delta.iter().all(|value| value.is_finite()) {
            return Err(PoseOptimizationError::NonFiniteInput);
        }

        pose_ws = apply_delta(&pose_ws, &delta);
        iterations += 1;
        final_cost = total_cost(&pose_ws, factors, config.huber_delta);

        if delta.norm() < config.min_delta_norm {
            converged = true;
            break;
        }
    }

    Ok((
        pose_ws,
        PoseOptimizationReport {
            initial_cost,
            final_cost,
            iterations,
            converged,
        },
    ))
}

pub fn point_to_plane_residual(pose_ws: &Pose3, factor: &PointToPlaneFactor) -> f64 {
    let point_w = pose_ws.transform_point(&factor.point_s);
    factor.plane_w.normal_w.dot(&point_w) + factor.plane_w.offset
}

pub fn point_to_plane_jacobian(pose_ws: &Pose3, factor: &PointToPlaneFactor) -> Vec6 {
    let point_w_without_translation = pose_ws.rotation * factor.point_s;
    let normal_w = factor.plane_w.normal_w;
    let rotation_jacobian = point_w_without_translation.cross(&normal_w);

    Vec6::new(
        rotation_jacobian.x,
        rotation_jacobian.y,
        rotation_jacobian.z,
        normal_w.x,
        normal_w.y,
        normal_w.z,
    )
}

fn validate_inputs(
    initial_pose_ws: &Pose3,
    factors: &[PointToPlaneFactor],
) -> Result<(), PoseOptimizationError> {
    if factors.len() < 3 {
        return Err(PoseOptimizationError::NotEnoughFactors {
            actual: factors.len(),
            required: 3,
        });
    }

    if !initial_pose_ws
        .translation
        .iter()
        .all(|value| value.is_finite())
    {
        return Err(PoseOptimizationError::NonFiniteInput);
    }

    for factor in factors {
        if factor.weight < 0.0
            || !factor.weight.is_finite()
            || !factor.point_s.iter().all(|value| value.is_finite())
            || !factor
                .plane_w
                .normal_w
                .iter()
                .all(|value| value.is_finite())
            || !factor.plane_w.offset.is_finite()
        {
            return Err(PoseOptimizationError::NonFiniteInput);
        }
    }

    Ok(())
}

fn build_normal_equations(
    pose_ws: &Pose3,
    factors: &[PointToPlaneFactor],
    config: PoseOptimizerConfig,
) -> Result<(Mat6, Vec6), PoseOptimizationError> {
    let mut normal_matrix = Mat6::identity() * config.damping.max(0.0);
    let mut gradient = Vec6::zeros();

    for factor in factors {
        if factor.weight == 0.0 {
            continue;
        }

        let residual = point_to_plane_residual(pose_ws, factor);
        let jacobian = point_to_plane_jacobian(pose_ws, factor);
        let robust_weight = huber_weight(residual, config.huber_delta);
        let weight = factor.weight * robust_weight;

        normal_matrix += weight * (jacobian * jacobian.transpose());
        gradient += weight * jacobian * residual;
    }

    Ok((normal_matrix, -gradient))
}

fn total_cost(pose_ws: &Pose3, factors: &[PointToPlaneFactor], huber_delta: Option<f64>) -> f64 {
    factors
        .iter()
        .map(|factor| {
            let residual = point_to_plane_residual(pose_ws, factor);
            factor.weight * huber_cost(residual, huber_delta)
        })
        .sum()
}

fn apply_delta(pose_ws: &Pose3, delta: &Vec6) -> Pose3 {
    let dtheta = Vec3::new(delta[0], delta[1], delta[2]);
    let dt = Vec3::new(delta[3], delta[4], delta[5]);
    let delta_rotation = UnitQuaternion::from_scaled_axis(dtheta);

    Pose3::new(delta_rotation * pose_ws.rotation, pose_ws.translation + dt)
}

fn huber_weight(residual: f64, delta: Option<f64>) -> f64 {
    let Some(delta) = delta else {
        return 1.0;
    };
    if delta <= 0.0 {
        return 1.0;
    }

    let abs_residual = residual.abs();
    if abs_residual <= delta {
        1.0
    } else {
        delta / abs_residual
    }
}

fn huber_cost(residual: f64, delta: Option<f64>) -> f64 {
    let Some(delta) = delta else {
        return 0.5 * residual * residual;
    };
    if delta <= 0.0 {
        return 0.5 * residual * residual;
    }

    let abs_residual = residual.abs();
    if abs_residual <= delta {
        0.5 * residual * residual
    } else {
        delta * (abs_residual - 0.5 * delta)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plane(normal_w: Vec3<f64>, offset: f64) -> PlaneFit {
        PlaneFit {
            centroid_w: Vec3::zeros(),
            normal_w: normal_w.normalize(),
            offset,
            eigenvalues: Vec3::new(0.0, 1.0, 1.0),
            planarity_ratio: 0.0,
        }
    }

    fn pose(rotation: UnitQuaternion<f64>, translation: Vec3<f64>) -> Pose3 {
        Pose3::new(rotation, translation)
    }

    fn horizontal_factors(z_s: f64, count: usize) -> Vec<PointToPlaneFactor> {
        (0..count)
            .map(|idx| PointToPlaneFactor {
                point_s: Vec3::new(idx as f64, (idx % 2) as f64, z_s),
                plane_w: plane(Vec3::z_axis().into_inner(), 0.0),
                weight: 1.0,
            })
            .collect()
    }

    #[test]
    fn point_to_plane_jacobian_matches_finite_difference() {
        let factor = PointToPlaneFactor {
            point_s: Vec3::new(1.2, -0.7, 0.4),
            plane_w: plane(Vec3::new(0.3, -0.4, 0.866), -0.2),
            weight: 1.0,
        };
        let pose_ws = pose(
            UnitQuaternion::from_scaled_axis(Vec3::new(0.1, -0.2, 0.05)),
            Vec3::new(0.3, 0.4, -0.1),
        );
        let analytical = point_to_plane_jacobian(&pose_ws, &factor);
        let eps = 1.0e-6;

        for idx in 0..6 {
            let mut plus = Vec6::zeros();
            plus[idx] = eps;
            let mut minus = Vec6::zeros();
            minus[idx] = -eps;

            let residual_plus = point_to_plane_residual(&apply_delta(&pose_ws, &plus), &factor);
            let residual_minus = point_to_plane_residual(&apply_delta(&pose_ws, &minus), &factor);
            let numerical = (residual_plus - residual_minus) / (2.0 * eps);

            assert!(
                (analytical[idx] - numerical).abs() < 1.0e-6,
                "idx={idx}, analytical={}, numerical={}",
                analytical[idx],
                numerical
            );
        }
    }

    #[test]
    fn optimizer_reduces_translation_residual() {
        let factors = horizontal_factors(0.0, 6);
        let initial_pose = pose(UnitQuaternion::identity(), Vec3::new(0.0, 0.0, 0.5));
        let config = PoseOptimizerConfig {
            huber_delta: None,
            ..PoseOptimizerConfig::default()
        };

        let (result_pose, report) =
            optimize_pose_point_to_plane(initial_pose, &factors, config).unwrap();

        assert!(report.final_cost < report.initial_cost);
        assert!(result_pose.translation.z.abs() < 1.0e-6);
    }

    #[test]
    fn optimizer_corrects_small_roll_against_horizontal_plane() {
        let factors = vec![
            PointToPlaneFactor {
                point_s: Vec3::new(-1.0, -1.0, 0.0),
                plane_w: plane(Vec3::z_axis().into_inner(), 0.0),
                weight: 1.0,
            },
            PointToPlaneFactor {
                point_s: Vec3::new(1.0, -1.0, 0.0),
                plane_w: plane(Vec3::z_axis().into_inner(), 0.0),
                weight: 1.0,
            },
            PointToPlaneFactor {
                point_s: Vec3::new(-1.0, 1.0, 0.0),
                plane_w: plane(Vec3::z_axis().into_inner(), 0.0),
                weight: 1.0,
            },
            PointToPlaneFactor {
                point_s: Vec3::new(1.0, 1.0, 0.0),
                plane_w: plane(Vec3::z_axis().into_inner(), 0.0),
                weight: 1.0,
            },
        ];
        let initial_pose = pose(
            UnitQuaternion::from_scaled_axis(Vec3::new(0.1, 0.0, 0.0)),
            Vec3::zeros(),
        );

        let (result_pose, report) =
            optimize_pose_point_to_plane(initial_pose, &factors, PoseOptimizerConfig::default())
                .unwrap();

        assert!(report.final_cost < report.initial_cost);
        assert!(result_pose.rotation.scaled_axis().x.abs() < 1.0e-4);
    }

    #[test]
    fn optimizer_rejects_not_enough_factors() {
        let factors = horizontal_factors(0.0, 2);

        let result = optimize_pose_point_to_plane(
            pose(UnitQuaternion::identity(), Vec3::zeros()),
            &factors,
            PoseOptimizerConfig::default(),
        );
        let Err(err) = result else {
            panic!("expected not enough factors error");
        };

        assert_eq!(
            err,
            PoseOptimizationError::NotEnoughFactors {
                actual: 2,
                required: 3
            }
        );
    }

    #[test]
    fn huber_weight_downweights_outlier() {
        assert_eq!(huber_weight(0.05, Some(0.1)), 1.0);
        assert!((huber_weight(1.0, Some(0.1)) - 0.1).abs() < 1.0e-12);
    }
}
