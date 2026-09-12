use crate::{
    iekf::{box_minus, gravity_box_plus},
    linearize_point_to_line_observation, linearize_point_to_plane_observation, skew,
};
use fastlio_map::surfel::{SurfelLineObservation, SurfelMap, SurfelObservation};
use fastlio_types::{LidarImuExtrinsic, Mat2, Mat3, Mat32, NavState, PointXYZI, Vec2, Vec3};
use nalgebra::{SMatrix, SVector};

#[derive(Debug, Clone, PartialEq)]
pub enum IekfUpdateError {
    NotSpd,
    SolveFailed,
    InvalidObservation,
    InvalidInput,
    NotEnoughObservations { actual: usize, required: usize },
    MapQueryFailed { context: String },
}

// Line observations carry two scalar residual rows and therefore two 23D
// Jacobians. Keep the enum inline to avoid per-observation heap allocation in
// the IEKF hot path.
#[allow(clippy::large_enum_variant)]
pub enum LinearizedObservation {
    Plane(PlaneLinearizedObservation),
    Line(LineLinearizedObservation),
}

#[derive(Debug, Clone)]
pub struct LineLinearizedObservation {
    pub residual0: f64,
    pub residual1: f64,
    pub jacobian0: SVector<f64, 23>,
    pub jacobian1: SVector<f64, 23>,
    pub variance: f64,
    pub distance: f64,
}

#[derive(Debug, Clone)]
pub struct PlaneLinearizedObservation {
    pub residual: f64,
    pub jacobian: SVector<f64, 23>,
    pub variance: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ObservationDiagnostics {
    pub input_points: usize,
    pub accepted_observations: usize,
    pub plane_accepted: usize,
    pub line_accepted: usize,
    pub no_association: usize,
    pub residual_abs_mean: f64,
    pub residual_abs_p50: f64,
    pub residual_abs_p90: f64,
    pub residual_abs_p95: f64,
    pub residual_abs_p99: f64,
    pub residual_abs_max: f64,
    pub line_residual_abs_p95: f64,
    pub line_distance_p95: f64,
}

impl ObservationDiagnostics {
    pub fn from_observations(input_points: usize, observations: &[LinearizedObservation]) -> Self {
        let accepted_observations = observations.len();
        if accepted_observations == 0 {
            return Self {
                input_points,
                no_association: input_points,
                ..Self::default()
            };
        }

        let residuals = sorted_abs_residual_rows(observations);
        let line_residuals = sorted_abs_line_residual_rows(observations);
        let line_distances = sorted_line_distances(observations);
        let plane_accepted = observations
            .iter()
            .filter(|observation| matches!(observation, LinearizedObservation::Plane(_)))
            .count();
        let line_accepted = observations
            .iter()
            .filter(|observation| matches!(observation, LinearizedObservation::Line(_)))
            .count();
        Self {
            input_points,
            accepted_observations,
            plane_accepted,
            line_accepted,
            no_association: input_points.saturating_sub(accepted_observations),
            residual_abs_mean: mean(&residuals),
            residual_abs_p50: percentile(&residuals, 0.50),
            residual_abs_p90: percentile(&residuals, 0.90),
            residual_abs_p95: percentile(&residuals, 0.95),
            residual_abs_p99: percentile(&residuals, 0.99),
            residual_abs_max: *residuals.last().unwrap_or(&0.0),
            line_residual_abs_p95: percentile(&line_residuals, 0.95),
            line_distance_p95: percentile(&line_distances, 0.95),
        }
    }
}

fn sorted_abs_residual_rows(observations: &[LinearizedObservation]) -> Vec<f64> {
    let mut values = Vec::new();
    for observation in observations {
        match observation {
            LinearizedObservation::Plane(observation) => {
                values.push(observation.residual.abs());
            }
            LinearizedObservation::Line(observation) => {
                values.push(observation.residual0.abs());
                values.push(observation.residual1.abs());
            }
        }
    }
    values.retain(|value| value.is_finite());
    values.sort_by(f64::total_cmp);
    values
}

fn sorted_abs_line_residual_rows(observations: &[LinearizedObservation]) -> Vec<f64> {
    let mut values = Vec::new();
    for observation in observations {
        if let LinearizedObservation::Line(observation) = observation {
            values.push(observation.residual0.abs());
            values.push(observation.residual1.abs());
        }
    }
    values.retain(|value| value.is_finite());
    values.sort_by(f64::total_cmp);
    values
}

fn sorted_line_distances(observations: &[LinearizedObservation]) -> Vec<f64> {
    let mut values = observations
        .iter()
        .filter_map(|observation| match observation {
            LinearizedObservation::Plane(_) => None,
            LinearizedObservation::Line(observation) => Some(observation.distance),
        })
        .filter(|value| value.is_finite())
        .collect::<Vec<_>>();
    values.sort_by(f64::total_cmp);
    values
}

fn mean(values: &[f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    values.iter().sum::<f64>() / values.len() as f64
}

fn percentile(values: &[f64], p: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let index = ((values.len() - 1) as f64 * p).round() as usize;
    values[index]
}

/// Configuration for the current 23D error-state IEKF update.
#[derive(Debug, Clone, Copy)]
pub struct IekfConfig {
    pub max_iterations: usize,
    pub min_delta_norm: f64,
    pub damping: f64,
    pub measurement_variance_floor: f64,
    // TODO(iekf): either wire a real robust kernel into the information build
    // or remove this from the public config before pipeline integration.
    pub huber_delta: Option<f64>,
    pub min_observations: usize,
}

impl Default for IekfConfig {
    fn default() -> Self {
        Self {
            max_iterations: 10,
            min_delta_norm: 1.0e-6,
            damping: 1.0e-6,
            measurement_variance_floor: 1.0e-6,
            huber_delta: Some(0.1),
            min_observations: 400,
        }
    }
}

pub(crate) fn build_observations(
    state: &NavState,
    points: &[PointXYZI],
    map: &SurfelMap,
    extrinsic: &LidarImuExtrinsic,
    config: &IekfConfig,
    out: &mut Vec<LinearizedObservation>,
) -> Result<(), IekfUpdateError> {
    out.clear();
    for point in points {
        let point_i = extrinsic.transform_point(&point.to_vec3_f64());
        let point_w_vec = transform_point(state, &point_i);
        let point_w = PointXYZI {
            x: point_w_vec.x as f32,
            y: point_w_vec.y as f32,
            z: point_w_vec.z as f32,
            intensity: point.intensity,
        };
        let point_i = PointXYZI {
            x: point_i.x as f32,
            y: point_i.y as f32,
            z: point_i.z as f32,
            intensity: point.intensity,
        };

        let plane_observation =
            map.query_plane(&point_w)
                .map_err(|e| IekfUpdateError::MapQueryFailed {
                    context: e.to_string(),
                })?;
        if let Some(plane_observation) = plane_observation {
            out.push(LinearizedObservation::Plane(
                build_plane_linearized_observation(state, &point_i, &plane_observation, config),
            ));
        } else if let Some(line_observation) =
            map.query_line(&point_w)
                .map_err(|e| IekfUpdateError::MapQueryFailed {
                    context: e.to_string(),
                })?
        {
            out.push(LinearizedObservation::Line(
                build_line_linearized_observation(state, &point_i, &line_observation, config),
            ));
        }
    }
    Ok(())
}

fn build_plane_linearized_observation(
    state: &NavState,
    point_i: &PointXYZI,
    surfel_observation: &SurfelObservation,
    config: &IekfConfig,
) -> PlaneLinearizedObservation {
    let jacobian = linearize_point_to_plane_observation(state, point_i, surfel_observation);
    let residual = surfel_observation.signed_residual;
    let variance = build_variance(surfel_observation, config);
    PlaneLinearizedObservation {
        jacobian,
        residual,
        variance,
    }
}

fn build_line_linearized_observation(
    state: &NavState,
    point_i: &PointXYZI,
    surfel_line_observation: &SurfelLineObservation,
    config: &IekfConfig,
) -> LineLinearizedObservation {
    let (jacobian0, jacobian1) =
        linearize_point_to_line_observation(state, point_i, surfel_line_observation);
    LineLinearizedObservation {
        residual0: surfel_line_observation.residual0,
        residual1: surfel_line_observation.residual1,
        jacobian0,
        jacobian1,
        variance: build_line_variance(surfel_line_observation, config),
        distance: surfel_line_observation.distance,
    }
}

fn build_line_variance(obs: &SurfelLineObservation, config: &IekfConfig) -> f64 {
    let min_eigenvalue = obs.eigenvalues[1].max(1.0e-3);
    min_eigenvalue.max(config.measurement_variance_floor)
}

pub(crate) fn build_variance(obs: &SurfelObservation, config: &IekfConfig) -> f64 {
    let min_eigenvalue = obs.eigenvalues[0].max(1.0e-3);
    min_eigenvalue.max(config.measurement_variance_floor)
}

pub(crate) fn transform_point(state: &NavState, point: &Vec3<f64>) -> Vec3<f64> {
    let t = state.position;
    let r = state.orientation.to_rotation_matrix();
    let r = r.matrix();

    r * point + t
}

#[inline]
fn s2_prior_jacobian_inverse(eta: &Vec2<f64>) -> Mat2<f64> {
    let r2 = eta.norm_squared();

    if r2 < 1e-10 {
        return Mat2::identity();
    }

    let r = r2.sqrt();
    let k = r / r.sin();
    k * Mat2::identity() + (1.0 - k) * (eta * eta.transpose()) / r2
}

#[inline]
fn so3_right_jacobian_inverse(phi: &Vec3<f64>) -> Mat3<f64> {
    let theta = phi.norm();
    let theta2 = phi.norm_squared();

    let phi_hat = skew(phi);
    let phi_hat2 = phi_hat * phi_hat;

    if theta >= 1e-6 {
        let a = 1.0 / theta2 - (1.0 + theta.cos()) / (2.0 * theta * theta.sin());

        Mat3::identity() + 0.5 * phi_hat + a * phi_hat2
    } else {
        Mat3::identity() + 0.5 * phi_hat + (1.0 / 12.0) * phi_hat2
    }
}

#[inline]
fn prior_error_jacobian(
    state_iter: &NavState,
    state: &NavState,
    gravity_basis_prior: &Mat32,
    gravity_basis_iter: &Mat32,
) -> SMatrix<f64, 23, 23> {
    let prior_error = box_minus(state_iter, state, gravity_basis_prior);
    let mut jacobian = SMatrix::<f64, 23, 23>::identity();
    let phi = prior_error.fixed_rows::<3>(0).into_owned();
    let eta = prior_error.fixed_rows::<2>(15).into_owned();
    let (_, gravity_basis_direct) = gravity_box_plus(&state.gravity, gravity_basis_prior, &eta);
    let q = gravity_basis_direct.transpose() * gravity_basis_iter;
    let j_gravity = s2_prior_jacobian_inverse(&eta) * q;
    jacobian
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&so3_right_jacobian_inverse(&phi));
    jacobian
        .fixed_view_mut::<2, 2>(15, 15)
        .copy_from(&j_gravity);
    jacobian
}

fn accumulate_row(
    information: &mut SMatrix<f64, 23, 23>,
    rhs: &mut SVector<f64, 23>,
    h: SVector<f64, 23>,
    residual: f64,
    variance: f64,
) -> Result<(), IekfUpdateError> {
    if !residual.is_finite() || !variance.is_finite() || variance <= 0.0 {
        return Err(IekfUpdateError::InvalidObservation);
    }
    let w = 1.0 / variance;
    *information += h * w * h.transpose();
    *rhs -= h * w * residual;
    Ok(())
}

pub(crate) fn linear_update(
    state: &NavState,
    gravity_basis_prior: &Mat32,
    gravity_basis_iter: &Mat32,
    state_iter: &NavState,
    covariance: &SMatrix<f64, 23, 23>,
    observations: &[LinearizedObservation],
    _config: &IekfConfig,
) -> Result<(SVector<f64, 23>, SMatrix<f64, 23, 23>), IekfUpdateError> {
    let p_chol = covariance.cholesky().ok_or(IekfUpdateError::NotSpd)?;
    let l = p_chol.l();

    // Current IEKF linear solve is written in whitened information form.
    let prior_error = box_minus(state_iter, state, gravity_basis_prior);
    let prior_error_jacobian =
        prior_error_jacobian(state_iter, state, gravity_basis_prior, gravity_basis_iter);

    let b_prior = l
        .solve_lower_triangular(&prior_error)
        .ok_or(IekfUpdateError::SolveFailed)?;

    let a_prior = l
        .solve_lower_triangular(&prior_error_jacobian)
        .ok_or(IekfUpdateError::SolveFailed)?;

    let mut information = a_prior.transpose() * a_prior;
    let mut rhs = -a_prior.transpose() * b_prior;

    for obs in observations {
        match obs {
            LinearizedObservation::Plane(o) => {
                accumulate_row(
                    &mut information,
                    &mut rhs,
                    o.jacobian,
                    o.residual,
                    o.variance,
                )?;
            }
            LinearizedObservation::Line(o) => {
                accumulate_row(
                    &mut information,
                    &mut rhs,
                    o.jacobian0,
                    o.residual0,
                    o.variance,
                )?;
                accumulate_row(
                    &mut information,
                    &mut rhs,
                    o.jacobian1,
                    o.residual1,
                    o.variance,
                )?;
            }
        }
    }

    let chol = symmetric(&information)
        .cholesky()
        .ok_or(IekfUpdateError::NotSpd)?;
    let dx = chol.solve(&rhs);
    let post_covariance = chol.inverse();
    Ok((dx, post_covariance))
}

pub(crate) fn symmetric(covariance: &SMatrix<f64, 23, 23>) -> SMatrix<f64, 23, 23> {
    (covariance.transpose() + covariance) / 2.0
}

// reserved
// fn huber_weight(residual: f64, huber_delta: Option<f64>) -> f64 {
//     let Some(huber_delta) = huber_delta else {
//         return 1.0;
//     };

//     if huber_delta <= 0.0 {
//         return 1.0;
//     }

//     let abs_residual = residual.abs();
//     if abs_residual <= huber_delta {
//         1.0
//     } else {
//         huber_delta / abs_residual
//     }
// }

#[cfg(test)]
mod tests {
    use super::*;
    use crate::iekf::{IekfState, box_plus};
    use fastlio_map::surfel::SurfelMap;
    use fastlio_types::{SurfelConfig, SurfelMapConfig, Vec3};
    use nalgebra::UnitQuaternion;

    const TOL: f64 = 1e-9;

    fn make_state() -> NavState {
        NavState {
            position: Vec3::new(1.0, 2.0, 3.0),
            orientation: UnitQuaternion::from_euler_angles(0.3, 0.2, 0.1),
            velocity: Vec3::new(4.0, 5.0, 6.0),
            gyro_bias: Vec3::new(0.1, 0.2, 0.3),
            accel_bias: Vec3::new(0.4, 0.5, 0.6),
            gravity: Vec3::new(0.0, 0.0, -9.81),
        }
    }

    fn config_with_zero_damping() -> IekfConfig {
        IekfConfig {
            damping: 0.0,
            measurement_variance_floor: 1e-12,
            min_observations: 1,
            ..IekfConfig::default()
        }
    }

    /// SPD prior covariance: `base` on the diagonal plus overrides on the
    /// specified entries. Callers must keep the touched 2x2 blocks positive
    /// definite.
    fn diagonal_covariance(base: f64, overrides: &[(usize, f64)]) -> SMatrix<f64, 23, 23> {
        let mut c = SMatrix::<f64, 23, 23>::identity() * base;
        for &(i, v) in overrides {
            c[(i, i)] = v;
        }
        c
    }

    fn position_z_observation(residual: f64, variance: f64) -> LinearizedObservation {
        // Jacobian selects only the position-z error state column (index 5).
        let mut h = SVector::<f64, 23>::zeros();
        h[5] = 1.0;
        LinearizedObservation::Plane(PlaneLinearizedObservation {
            residual,
            jacobian: h,
            variance,
        })
    }

    // ---------------------------------------------------------------
    // 3. Zero residual and zero prior error -> the linear solve must return a
    //    zero correction.
    // ---------------------------------------------------------------
    #[test]
    fn linear_update_zero_residual_and_zero_prior_error_returns_zero_dx() {
        let state = make_state();
        let state_iter = state.clone();
        let covariance = diagonal_covariance(0.01, &[(5, 1.0)]);
        let config = config_with_zero_damping();

        let mut observations = Vec::new();
        for _ in 0..3 {
            observations.push(position_z_observation(0.0, 1.0));
        }

        let gravity_basis = fastlio_types::gravity_tangent_basis(&state.gravity);
        let (dx, post) = linear_update(
            &state,
            &gravity_basis,
            &gravity_basis,
            &state_iter,
            &covariance,
            &observations,
            &config,
        )
        .expect("linear solve must succeed");

        assert!(
            dx.norm() < 1e-9,
            "zero residual + zero prior error must give dx=0, got norm={}",
            dx.norm()
        );
        assert!(
            post.cholesky().is_some(),
            "posterior covariance must remain SPD"
        );
    }

    // ---------------------------------------------------------------
    // 4. A single position observation against a diagonal prior reduces to the
    //    1D closed form dx_5 = -p_z * r / (p_z + var).
    // ---------------------------------------------------------------
    #[test]
    fn linear_update_position_observation_matches_1d_closed_form() {
        let state = make_state();
        let state_iter = state.clone();
        let p_z = 2.0;
        let var = 1.0;
        let r = 0.5;
        let covariance = diagonal_covariance(1e-4, &[(5, p_z)]);
        let config = config_with_zero_damping();

        let observations = vec![position_z_observation(r, var)];
        let gravity_basis = fastlio_types::gravity_tangent_basis(&state.gravity);
        let (dx, _post) = linear_update(
            &state,
            &gravity_basis,
            &gravity_basis,
            &state_iter,
            &covariance,
            &observations,
            &config,
        )
        .expect("linear solve must succeed");

        let expected = -p_z * r / (p_z + var);
        assert!(
            (dx[5] - expected).abs() < TOL,
            "position-z correction: got={:.12}, expected={expected:.12}",
            dx[5]
        );
        for i in 0..23 {
            if i != 5 {
                assert!(
                    dx[i].abs() < TOL,
                    "inactive error-state [{i}] must be zero, got {}",
                    dx[i]
                );
            }
        }
    }

    // ---------------------------------------------------------------
    // 5. With no observations the prior error alone pulls the iterate back to
    //    the prior state (a pure "prior projection").
    // ---------------------------------------------------------------
    #[test]
    fn linear_update_prior_error_pulls_state_back() {
        let state = make_state();
        let covariance = diagonal_covariance(1.0, &[]);
        let config = config_with_zero_damping();

        let mut e = SVector::<f64, 23>::zeros();
        e.fixed_rows_mut::<3>(0)
            .copy_from(&Vec3::new(0.05, 0.0, 0.0));
        e.fixed_rows_mut::<3>(3)
            .copy_from(&Vec3::new(0.2, -0.1, 0.3));
        e.fixed_rows_mut::<3>(6)
            .copy_from(&Vec3::new(0.4, 0.0, -0.2));
        let gravity_basis = fastlio_types::gravity_tangent_basis(&state.gravity);
        let (state_iter, gravity_basis_iter) = box_plus(&state, &gravity_basis, &e);

        let observations = Vec::new();
        let (dx, post) = linear_update(
            &state,
            &gravity_basis,
            &gravity_basis_iter,
            &state_iter,
            &covariance,
            &observations,
            &config,
        )
        .expect("linear solve must succeed");

        // dx must be exactly the negative of the prior error (empty update).
        let neg_prior_error = -box_minus(&state_iter, &state, &gravity_basis);
        assert!(
            (dx - neg_prior_error).norm() < TOL,
            "dx should equal -prior_error, diff norm={}",
            (dx - neg_prior_error).norm()
        );

        // Re-composing must return to the prior state.
        let (back, _) = box_plus(&state_iter, &gravity_basis_iter, &dx);
        assert!(
            (back.position - state.position).norm() < TOL,
            "position not pulled back to prior"
        );
        assert!(
            back.orientation.angle_to(&state.orientation) < TOL,
            "orientation not pulled back to prior"
        );
        assert!(
            (back.velocity - state.velocity).norm() < TOL,
            "velocity not pulled back to prior"
        );
        // The returned covariance is expressed in the current iterate's tangent
        // space, so the prior covariance is transported by J_prior.
        let j_prior =
            prior_error_jacobian(&state_iter, &state, &gravity_basis, &gravity_basis_iter);
        let p_inv = covariance
            .try_inverse()
            .expect("test covariance must be invertible");
        let expected_post = (j_prior.transpose() * p_inv * j_prior)
            .try_inverse()
            .expect("transported information matrix must be invertible");
        assert!(
            (post - expected_post).norm() < 1e-9,
            "covariance must match the prior expressed in the iterate tangent space"
        );
    }

    #[test]
    fn prior_error_jacobian_matches_finite_difference() {
        const FD_EPS: f64 = 1e-7;
        const FD_TOL: f64 = 1e-6;

        let mut state = make_state();
        state.gravity = Vec3::new(2.4, -1.7, -9.2).normalize() * 9.81;
        let mut offset = SVector::<f64, 23>::zeros();
        offset
            .fixed_rows_mut::<3>(0)
            .copy_from(&Vec3::new(0.4, -0.3, 0.6));
        offset
            .fixed_rows_mut::<3>(3)
            .copy_from(&Vec3::new(0.2, -0.1, 0.3));
        offset
            .fixed_rows_mut::<3>(6)
            .copy_from(&Vec3::new(-0.4, 0.1, 0.2));
        offset
            .fixed_rows_mut::<2>(15)
            .copy_from(&Vec2::new(0.2, -0.15));
        let gravity_basis = fastlio_types::gravity_tangent_basis(&state.gravity);
        let (state_iter, gravity_basis_iter) = box_plus(&state, &gravity_basis, &offset);
        let analytic =
            prior_error_jacobian(&state_iter, &state, &gravity_basis, &gravity_basis_iter);

        // NavState owns the first 17 dimensions. The reserved extrinsic blocks
        // (17..23) remain fixed and are not implemented by box_plus.
        for column in 0..17 {
            let mut delta_plus = SVector::<f64, 23>::zeros();
            let mut delta_minus = SVector::<f64, 23>::zeros();
            delta_plus[column] = FD_EPS;
            delta_minus[column] = -FD_EPS;

            let (state_plus, _) = box_plus(&state_iter, &gravity_basis_iter, &delta_plus);
            let (state_minus, _) = box_plus(&state_iter, &gravity_basis_iter, &delta_minus);
            let error_plus = box_minus(&state_plus, &state, &gravity_basis);
            let error_minus = box_minus(&state_minus, &state, &gravity_basis);
            let finite_difference = (error_plus - error_minus) / (2.0 * FD_EPS);
            let analytic_column = analytic.column(column);

            assert!(
                (finite_difference - analytic_column).norm() < FD_TOL,
                "J_prior column {column} does not match finite difference: analytic={}, fd={}, diff_norm={}",
                analytic_column.transpose(),
                finite_difference.transpose(),
                (finite_difference - analytic_column).norm()
            );
        }
    }

    #[test]
    fn prior_error_jacobian_matches_finite_difference_after_two_non_collinear_s2_updates() {
        const FD_EPS: f64 = 1e-7;
        const FD_TOL: f64 = 1e-6;

        let mut state_prior = make_state();
        state_prior.gravity = Vec3::new(2.4, -1.7, -9.2).normalize() * 9.81;
        let gravity_basis_prior = fastlio_types::gravity_tangent_basis(&state_prior.gravity);

        let mut first_delta = SVector::<f64, 23>::zeros();
        first_delta
            .fixed_rows_mut::<2>(15)
            .copy_from(&Vec2::new(0.20, -0.12));
        let (state_after_first, gravity_basis_after_first) =
            box_plus(&state_prior, &gravity_basis_prior, &first_delta);

        let mut second_delta = SVector::<f64, 23>::zeros();
        second_delta
            .fixed_rows_mut::<2>(15)
            .copy_from(&Vec2::new(-0.08, 0.17));
        let (state_iter, gravity_basis_iter) = box_plus(
            &state_after_first,
            &gravity_basis_after_first,
            &second_delta,
        );

        let analytic = prior_error_jacobian(
            &state_iter,
            &state_prior,
            &gravity_basis_prior,
            &gravity_basis_iter,
        );

        for column in 15..17 {
            let mut delta_plus = SVector::<f64, 23>::zeros();
            let mut delta_minus = SVector::<f64, 23>::zeros();
            delta_plus[column] = FD_EPS;
            delta_minus[column] = -FD_EPS;

            let (state_plus, _) = box_plus(&state_iter, &gravity_basis_iter, &delta_plus);
            let (state_minus, _) = box_plus(&state_iter, &gravity_basis_iter, &delta_minus);
            let error_plus = box_minus(&state_plus, &state_prior, &gravity_basis_prior);
            let error_minus = box_minus(&state_minus, &state_prior, &gravity_basis_prior);
            let finite_difference = (error_plus - error_minus) / (2.0 * FD_EPS);
            let analytic_column = analytic.column(column);

            assert!(
                (finite_difference - analytic_column).norm() < FD_TOL,
                "J_prior column {column} after two non-collinear S2 updates does not match finite difference: analytic={}, fd={}, diff_norm={}",
                analytic_column.transpose(),
                finite_difference.transpose(),
                (finite_difference - analytic_column).norm()
            );
        }
    }

    #[test]
    fn prior_error_jacobian_uses_transported_gravity_basis_not_rebuilt_basis() {
        const FD_EPS: f64 = 1e-7;
        const FD_TOL: f64 = 1e-6;

        let mut state_prior = make_state();
        state_prior.gravity = Vec3::new(2.4, -1.7, -9.2).normalize() * 9.81;
        let gravity_basis_prior = fastlio_types::gravity_tangent_basis(&state_prior.gravity);

        let mut first_delta = SVector::<f64, 23>::zeros();
        first_delta
            .fixed_rows_mut::<2>(15)
            .copy_from(&Vec2::new(0.31, -0.19));
        let (state_after_first, gravity_basis_after_first) =
            box_plus(&state_prior, &gravity_basis_prior, &first_delta);

        let mut second_delta = SVector::<f64, 23>::zeros();
        second_delta
            .fixed_rows_mut::<2>(15)
            .copy_from(&Vec2::new(-0.16, 0.27));
        let (state_iter, gravity_basis_iter) = box_plus(
            &state_after_first,
            &gravity_basis_after_first,
            &second_delta,
        );

        let analytic = prior_error_jacobian(
            &state_iter,
            &state_prior,
            &gravity_basis_prior,
            &gravity_basis_iter,
        );
        let rebuilt_basis_iter = fastlio_types::gravity_tangent_basis(&state_iter.gravity);

        let mut transported_diff_norm = 0.0;
        let mut rebuilt_diff_norm = 0.0;
        for column in 15..17 {
            let mut delta_plus = SVector::<f64, 23>::zeros();
            let mut delta_minus = SVector::<f64, 23>::zeros();
            delta_plus[column] = FD_EPS;
            delta_minus[column] = -FD_EPS;

            let (state_plus, _) = box_plus(&state_iter, &gravity_basis_iter, &delta_plus);
            let (state_minus, _) = box_plus(&state_iter, &gravity_basis_iter, &delta_minus);
            let transported_fd = (box_minus(&state_plus, &state_prior, &gravity_basis_prior)
                - box_minus(&state_minus, &state_prior, &gravity_basis_prior))
                / (2.0 * FD_EPS);

            let (state_plus_rebuilt, _) = box_plus(&state_iter, &rebuilt_basis_iter, &delta_plus);
            let (state_minus_rebuilt, _) = box_plus(&state_iter, &rebuilt_basis_iter, &delta_minus);
            let rebuilt_fd = (box_minus(&state_plus_rebuilt, &state_prior, &gravity_basis_prior)
                - box_minus(&state_minus_rebuilt, &state_prior, &gravity_basis_prior))
                / (2.0 * FD_EPS);

            let analytic_column = analytic.column(column);
            transported_diff_norm += (transported_fd - analytic_column).norm();
            rebuilt_diff_norm += (rebuilt_fd - analytic_column).norm();
        }

        assert!(
            transported_diff_norm < FD_TOL,
            "analytic J_prior must match finite difference in the transported runtime basis, diff={transported_diff_norm}"
        );
        assert!(
            rebuilt_diff_norm > 1e-3,
            "rebuilt gravity_tangent_basis accidentally matched transported basis; this test must catch Symbolica derivations that assume runtime basis rebuilding, diff={rebuilt_diff_norm}"
        );
    }

    // ---------------------------------------------------------------
    // 6. A pose observation must move the velocity block through the prior
    //    cross covariance: dx_6 = -c * r / (p_z + var).
    // ---------------------------------------------------------------
    #[test]
    fn linear_update_cross_covariance_updates_velocity_block() {
        let state = make_state();
        let state_iter = state.clone();

        let p_z = 1.0; // position-z prior variance (index 5)
        let p_v = 1.0; // velocity-x prior variance (index 6)
        let c = 0.5; // position-z <-> velocity-x cross covariance
        let var = 1.0;
        let r = 0.3;

        let mut covariance = diagonal_covariance(1e-4, &[(5, p_z), (6, p_v)]);
        covariance[(5, 6)] = c;
        covariance[(6, 5)] = c;

        let config = config_with_zero_damping();
        let observations = vec![position_z_observation(r, var)];

        let gravity_basis = fastlio_types::gravity_tangent_basis(&state.gravity);
        let (dx, _post) = linear_update(
            &state,
            &gravity_basis,
            &gravity_basis,
            &state_iter,
            &covariance,
            &observations,
            &config,
        )
        .expect("linear solve must succeed");

        // Closed form: dx = -C[:,5] * r / (p_z + var), so velocity-x gets
        // dx_6 = -c * r / (p_z + var) through the cross term.
        let expected_vel = -c * r / (p_z + var);
        assert!(
            (dx[6] - expected_vel).abs() < TOL,
            "velocity-x correction through cross covariance: got={:.12}, expected={expected_vel:.12}",
            dx[6]
        );
        assert!(
            dx[6].abs() > 1e-6,
            "velocity block must actually move when cross covariance is present"
        );
    }

    // ---------------------------------------------------------------
    // 7. With zero cross covariance between pose and velocity the velocity
    //    block must stay zero even though the pose block is corrected.
    // ---------------------------------------------------------------
    #[test]
    fn linear_update_zero_cross_covariance_keeps_velocity_block_zero() {
        let state = make_state();
        let state_iter = state.clone();

        let p_z = 1.0;
        let p_v = 1.0;
        let var = 1.0;
        let r = 0.3;

        // Diagonal prior: position and velocity are independent (c = 0).
        let covariance = diagonal_covariance(1e-4, &[(5, p_z), (6, p_v)]);
        let config = config_with_zero_damping();
        let observations = vec![position_z_observation(r, var)];

        let gravity_basis = fastlio_types::gravity_tangent_basis(&state.gravity);
        let (dx, _post) = linear_update(
            &state,
            &gravity_basis,
            &gravity_basis,
            &state_iter,
            &covariance,
            &observations,
            &config,
        )
        .expect("linear solve must succeed");

        let expected_pos = -p_z * r / (p_z + var);
        assert!(
            (dx[5] - expected_pos).abs() < TOL,
            "position-z should still be corrected, got={:.12}, expected={expected_pos:.12}",
            dx[5]
        );
        for i in 6..9 {
            assert!(
                dx[i].abs() < TOL,
                "velocity block [{i}] must stay zero without cross covariance, got {}",
                dx[i]
            );
        }
    }

    // ---------------------------------------------------------------
    // 8. The observed (position-z) covariance must shrink after the update.
    // ---------------------------------------------------------------
    #[test]
    fn linear_update_reduces_observed_covariance() {
        let state = make_state();
        let state_iter = state.clone();

        let p_z = 2.0;
        let var = 1.0;
        let covariance = diagonal_covariance(1e-4, &[(5, p_z)]);
        let config = config_with_zero_damping();
        let observations = vec![position_z_observation(0.5, var)];

        let gravity_basis = fastlio_types::gravity_tangent_basis(&state.gravity);
        let (_, post) = linear_update(
            &state,
            &gravity_basis,
            &gravity_basis,
            &state_iter,
            &covariance,
            &observations,
            &config,
        )
        .expect("linear solve must succeed");

        let expected = p_z * var / (p_z + var);
        assert!(
            post[(5, 5)] < p_z,
            "posterior variance {} must be smaller than prior {}",
            post[(5, 5)],
            p_z
        );
        assert!(
            (post[(5, 5)] - expected).abs() < TOL,
            "posterior variance: got={:.12}, expected={expected:.12}",
            post[(5, 5)]
        );
        assert!(
            post.cholesky().is_some(),
            "posterior covariance must remain SPD"
        );
    }

    // ---------------------------------------------------------------
    // 9. Non-positive measurement variance must be rejected (not silently
    //    produce a non-sensical update).
    // ---------------------------------------------------------------
    #[test]
    fn linear_update_rejects_zero_or_negative_variance() {
        let state = make_state();
        let state_iter = state.clone();
        // Large prior variance so that a negative weight drives the information
        // matrix indefinite.
        let covariance = diagonal_covariance(1e-4, &[(5, 4.0)]);
        let config = config_with_zero_damping();

        let zero_var = vec![position_z_observation(0.5, 0.0)];
        let negative_var = vec![position_z_observation(0.5, -1.0)];
        let gravity_basis = fastlio_types::gravity_tangent_basis(&state.gravity);

        assert!(
            linear_update(
                &state,
                &gravity_basis,
                &gravity_basis,
                &state_iter,
                &covariance,
                &zero_var,
                &config
            )
            .is_err(),
            "zero measurement variance must be rejected"
        );
        assert!(
            linear_update(
                &state,
                &gravity_basis,
                &gravity_basis,
                &state_iter,
                &covariance,
                &negative_var,
                &config
            )
            .is_err(),
            "negative measurement variance must be rejected"
        );
    }

    // ---------------------------------------------------------------
    // 10. IEKF end-to-end: a pose error that is too large for a single
    //     linearized step must still converge because observations (residual
    //     and Jacobian) are rebuilt from the iterated state.
    // ---------------------------------------------------------------
    #[test]
    fn iekf_update_rebuilds_observations_from_state_iter() {
        // Build a map containing a single horizontal plane at world z = 1.
        let map_config = SurfelMapConfig {
            voxel_size: 1.0,
            search_radius: 4,
        };
        let surfel_config = SurfelConfig {
            growing_radius: 3.5,
            max_plane_distance: 0.5,
            ..SurfelConfig::default()
        };
        let mut map = SurfelMap::new(map_config, surfel_config);

        let mut world_points = Vec::new();
        for x in [-1.0f32, -0.5, 0.0, 0.5, 1.0] {
            for y in [-1.0f32, -0.5, 0.0, 0.5, 1.0] {
                world_points.push(PointXYZI {
                    x,
                    y,
                    z: 1.0,
                    intensity: 0.0,
                });
            }
        }
        map.insert(world_points.into_iter()).unwrap();

        // The LiDAR/body-frame points lie on a plane through the body origin.
        // Under the true pose (R=I, t=(0,0,1)) they land on the map plane.
        let mut body_points = Vec::new();
        for x in [-0.75f32, -0.25, 0.25, 0.75] {
            for y in [-0.75f32, -0.25, 0.25, 0.75] {
                body_points.push(PointXYZI {
                    x,
                    y,
                    z: 0.0,
                    intensity: 0.0,
                });
            }
        }

        // Prior pose: offset in z plus a roll that is large enough that a
        // single linearized update from this point is not the final answer.
        let prior = NavState {
            position: Vec3::new(0.0, 0.0, 0.9),
            orientation: UnitQuaternion::from_scaled_axis(Vec3::new(0.25, 0.0, 0.0)),
            velocity: Vec3::zeros(),
            gyro_bias: Vec3::zeros(),
            accel_bias: Vec3::zeros(),
            gravity: Vec3::new(0.0, 0.0, -9.81),
        };
        let covariance = diagonal_covariance(0.1, &[]);

        let config = IekfConfig {
            max_iterations: 1,
            damping: 1e-6,
            measurement_variance_floor: 1e-4,
            // Test plane supplies only 16 body points; the pipeline default of
            // 400 observations can never be reached here.
            min_observations: 1,
            ..IekfConfig::default()
        };

        let mut single = IekfState {
            state: prior.clone(),
            gravity_basis: fastlio_types::gravity_tangent_basis(&prior.gravity),
            covariance,
        };
        let extrinsic = LidarImuExtrinsic::new(UnitQuaternion::identity(), Vec3::zeros());
        single
            .update(&body_points, &extrinsic, &map, &config)
            .unwrap();

        let config_iter = IekfConfig {
            max_iterations: 15,
            ..config
        };
        let mut iterated = IekfState {
            state: prior.clone(),
            gravity_basis: fastlio_types::gravity_tangent_basis(&prior.gravity),
            covariance,
        };
        iterated
            .update(&body_points, &extrinsic, &map, &config_iter)
            .unwrap();

        // Metric: mean absolute plane distance of the transformed points to
        // the map plane (via the relinearized query).
        let single_residual = mean_plane_residual(&single.state, &body_points, &map);
        let iter_residual = mean_plane_residual(&iterated.state, &body_points, &map);

        // Truth: orientation identity, position (0, 0, 1).
        let true_pose = NavState {
            position: Vec3::new(0.0, 0.0, 1.0),
            orientation: UnitQuaternion::identity(),
            ..prior.clone()
        };
        assert!(
            iter_residual < 1e-3,
            "iterated update must converge to the plane, mean residual={iter_residual:.6}"
        );
        assert!(
            iter_residual < single_residual,
            "iterating with rebuilt observations must beat a single linearized step (single={single_residual:.6}, iter={iter_residual:.6})"
        );
        assert!(
            (iterated.state.position - true_pose.position).norm() < 1e-2,
            "converged position must match truth, got {}",
            iterated.state.position
        );
        assert!(
            iterated.state.orientation.angle_to(&true_pose.orientation) < 1e-2,
            "converged orientation must match truth"
        );
    }

    fn mean_plane_residual(state: &NavState, points: &[PointXYZI], map: &SurfelMap) -> f64 {
        let mut total = 0.0;
        let mut count = 0usize;
        for p in points {
            let world = transform_point(state, &p.to_vec3_f64());
            let world = PointXYZI {
                x: world.x as f32,
                y: world.y as f32,
                z: world.z as f32,
                intensity: p.intensity,
            };
            if let Some(obs) = map.query_plane(&world).expect("query must not error") {
                total += obs.signed_residual.abs();
                count += 1;
            }
        }
        if count == 0 {
            f64::INFINITY
        } else {
            total / count as f64
        }
    }
}
