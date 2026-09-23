use crate::{
    optimizer::{build_surfel_observation, symmetric},
    skew,
};
use fastlio_map::surfel::SurfelMap;
use fastlio_types::{
    LidarImuExtrinsic, Mat2, Mat3, Mat32, NavState, PointXYZI, Vec2, Vec3, gravity_tangent_basis,
};
use nalgebra::{SMatrix, SVector, UnitQuaternion};

use crate::optimizer::{IekfConfig, IekfUpdateError};

pub(crate) fn gravity_box_plus(
    gravity: &Vec3<f64>,
    gravity_basis: &Mat32,
    error_gravity: &SVector<f64, 2>,
) -> (Vec3<f64>, Mat32) {
    let g_norm = gravity.norm();
    let u = gravity / g_norm;
    let v = gravity_basis * error_gravity;

    if v.norm_squared() < 1e-20 {
        return (*gravity, *gravity_basis);
    }

    let rotvec = u.cross(&v);
    let rotation = UnitQuaternion::from_scaled_axis(rotvec);
    let gravity_next = rotation * gravity;
    let basis_next = rotation.to_rotation_matrix().matrix() * gravity_basis;
    (gravity_next, basis_next)
}

pub(crate) fn gravity_box_minus(
    gravity_iter: &Vec3<f64>,
    gravity: &Vec3<f64>,
    gravity_basis: &Mat32,
) -> Vec2<f64> {
    let u0 = gravity / gravity.norm(); // base / prior
    let u1 = gravity_iter / gravity_iter.norm(); // target / iter

    let cross = u0.cross(&u1);
    let sin_theta = cross.norm();
    let cos_theta = u0.dot(&u1).clamp(-1.0, 1.0);

    if sin_theta < 1e-10 {
        // same direction
        if cos_theta > 0.0 {
            return Vec2::zeros();
        }

        // antipodal: log map is not unique
        panic!("S2 box_minus undefined near antipodal gravity");
    }

    let theta = sin_theta.atan2(cos_theta);

    // unit tangent direction at u0 toward u1
    let tangent_dir = (u1 - cos_theta * u0) / sin_theta;

    // angular tangent vector, units = rad
    let tangent = tangent_dir * theta;

    gravity_basis.transpose() * tangent
}

/// ```text
/// [delta_theta_i, delta_P_wi, delta_v, delta_bg, delta_ba, delta_g, delta_theta_li, delta_P_li]
/// ```
pub(crate) fn box_plus(
    state: &NavState,
    gravity_basis: &Mat32,
    error_state: &SVector<f64, 23>,
) -> (NavState, Mat32) {
    let delta_theta = error_state.fixed_rows::<3>(0).into_owned();
    let delta_rotation = UnitQuaternion::from_scaled_axis(delta_theta);

    let (gravity, gravity_basis) = gravity_box_plus(
        &state.gravity,
        gravity_basis,
        &error_state.fixed_rows::<2>(15).into_owned(),
    );
    let state = NavState {
        position: state.position + error_state.fixed_rows::<3>(3).into_owned(),
        orientation: state.orientation * delta_rotation,
        velocity: state.velocity + error_state.fixed_rows::<3>(6).into_owned(),
        gyro_bias: state.gyro_bias + error_state.fixed_rows::<3>(9).into_owned(),
        accel_bias: state.accel_bias + error_state.fixed_rows::<3>(12).into_owned(),
        gravity,
    };
    (state, gravity_basis)
}

pub(crate) fn box_minus(
    state_iter: &NavState,
    state: &NavState,
    gravity_basis: &Mat32,
) -> SVector<f64, 23> {
    let theta_iter = state_iter.orientation;
    let theta = state.orientation;
    let dtheta = (theta.inverse() * theta_iter).scaled_axis();

    let grav_iter = state_iter.gravity;
    let grav = state.gravity;
    let dg = gravity_box_minus(&grav_iter, &grav, gravity_basis);

    let mut dx = SVector::<f64, 23>::zeros();
    dx.fixed_rows_mut::<3>(0).copy_from(&dtheta);
    dx.fixed_rows_mut::<3>(3)
        .copy_from(&(state_iter.position - state.position));
    dx.fixed_rows_mut::<3>(6)
        .copy_from(&(state_iter.velocity - state.velocity));
    dx.fixed_rows_mut::<3>(9)
        .copy_from(&(state_iter.gyro_bias - state.gyro_bias));
    dx.fixed_rows_mut::<3>(12)
        .copy_from(&(state_iter.accel_bias - state.accel_bias));
    dx.fixed_rows_mut::<2>(15).copy_from(&dg);
    dx
}

pub struct IekfState {
    pub state: NavState,
    pub gravity_basis: Mat32,
    pub covariance: SMatrix<f64, 23, 23>,
}

impl Default for IekfState {
    fn default() -> Self {
        let state = NavState::default();
        Self {
            gravity_basis: gravity_tangent_basis(&state.gravity),
            state,
            covariance: SMatrix::<f64, 23, 23>::identity() * 0.1,
        }
    }
}

fn navstate_is_finite(state: &NavState) -> bool {
    state.position.iter().all(|value| value.is_finite())
        && state.velocity.iter().all(|value| value.is_finite())
        && state.gyro_bias.iter().all(|value| value.is_finite())
        && state.accel_bias.iter().all(|value| value.is_finite())
        && state.gravity.iter().all(|value| value.is_finite())
}

fn matrix_is_finite(matrix: &SMatrix<f64, 23, 23>) -> bool {
    matrix.iter().all(|value| value.is_finite())
}

#[inline]
fn so3_right_jacobian(phi: &Vec3<f64>) -> Mat3<f64> {
    let mut jacobian = Mat3::<f64>::identity();
    let theta = phi.norm();
    let theta2 = phi.norm_squared();
    let theta3 = theta * theta2;
    let theta_hat = skew(phi);
    let theta_hat2 = theta_hat * theta_hat;

    if theta >= 1e-6 {
        jacobian -= (1.0 - theta.cos()) / theta2 * theta_hat;
        jacobian += (theta - theta.sin()) / theta3 * theta_hat2;
    } else {
        jacobian -= 0.5 * theta_hat;
        jacobian += 1.0 / 6.0 * theta_hat2;
    }
    jacobian
}

#[inline]
fn s2_prior_jacobian(eta: &Vec2<f64>) -> Mat2<f64> {
    let r2 = eta.norm_squared();

    if r2 < 1e-10 {
        return Mat2::identity();
    }

    let r = r2.sqrt();
    let h = r.sin() / r;
    h * Mat2::identity() + (1.0 - h) * (eta * eta.transpose()) / r2
}

#[inline]
fn reset_covariance(
    covariance: &SMatrix<f64, 23, 23>,
    injected_error: &SVector<f64, 23>,
) -> SMatrix<f64, 23, 23> {
    let dtheta = injected_error.fixed_rows::<3>(0).into_owned();
    let dg = injected_error.fixed_rows::<2>(15).into_owned();
    let jr = so3_right_jacobian(&dtheta);
    let jg = s2_prior_jacobian(&dg);

    let mut g = SMatrix::<f64, 23, 23>::identity();
    g.fixed_view_mut::<3, 3>(0, 0).copy_from(&jr);
    g.fixed_view_mut::<2, 2>(15, 15).copy_from(&jg);

    symmetric(&(g * covariance * g.transpose()))
}

impl IekfState {
    pub fn new(state: NavState, covariance: SMatrix<f64, 23, 23>) -> Result<Self, IekfUpdateError> {
        if !navstate_is_finite(&state) || !matrix_is_finite(&covariance) {
            return Err(IekfUpdateError::InvalidInput);
        }

        let gravity_basis = gravity_tangent_basis(&state.gravity);
        Ok(Self {
            state,
            gravity_basis,
            covariance,
        })
    }
    pub fn update(
        &mut self,
        points: &[PointXYZI],
        extrinsic: &LidarImuExtrinsic,
        map: &SurfelMap,
        config: &IekfConfig,
    ) -> Result<(), IekfUpdateError> {
        let state_prior = self.state.clone();
        let gravity_basis_prior = self.gravity_basis;
        let p_prior = self.covariance;

        let mut state_iter = state_prior.clone();
        let mut gravity_basis_iter = gravity_basis_prior;
        let mut p_final = p_prior;
        let mut observations = Vec::new();

        let mut final_error = SVector::zeros();

        for _ in 0..config.max_iterations {
            build_surfel_observation(
                &state_iter,
                points,
                map,
                extrinsic,
                config,
                &mut observations,
            )?;
            let observations_len = observations.len();

            if observations_len < config.min_observations {
                return Err(IekfUpdateError::NotEnoughObservations {
                    actual: observations_len,
                    required: config.min_observations,
                });
            }

            let (error_state, p_work) = crate::optimizer::linear_update(
                &state_prior,
                &gravity_basis_prior,
                &gravity_basis_iter,
                &state_iter,
                &p_prior,
                &observations,
                config,
            )?;

            (state_iter, gravity_basis_iter) =
                box_plus(&state_iter, &gravity_basis_iter, &error_state);
            p_final = p_work;
            final_error = error_state;
            if final_error.norm() < config.min_delta_norm {
                break;
            }
        }

        self.state = state_iter;
        self.gravity_basis = gravity_basis_iter;
        self.covariance = reset_covariance(&p_final, &final_error);
        Ok(())
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use fastlio_types::Vec3;
    use nalgebra::{SVector, UnitQuaternion};

    const ANGLE_TOL: f64 = 1e-12;

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

    #[test]
    fn box_plus_zero_delta_preserves_state() {
        let state = make_state();
        let zero = SVector::<f64, 23>::zeros();
        let basis = gravity_tangent_basis(&state.gravity);
        let (out, out_basis) = box_plus(&state, &basis, &zero);

        assert_eq!(out.position, state.position);
        assert!(
            out.orientation.angle_to(&state.orientation) < ANGLE_TOL,
            "orientation changed under zero delta"
        );
        assert_eq!(out.velocity, state.velocity);
        assert_eq!(out.gyro_bias, state.gyro_bias);
        assert_eq!(out.accel_bias, state.accel_bias);
        assert_eq!(out.gravity, state.gravity);
        assert_eq!(out_basis, basis);
    }

    #[test]
    fn box_plus_adds_euclidean_blocks_and_retracts_gravity_on_s2() {
        let state = make_state();
        let dp = Vec3::new(0.1, -0.2, 0.3);
        let dv = Vec3::new(0.5, 0.5, 0.5);
        let dbg = Vec3::new(0.01, 0.01, 0.01);
        let dba = Vec3::new(0.02, 0.02, 0.02);
        let dg = Vec2::new(0.1, -0.05);

        let mut delta = SVector::<f64, 23>::zeros();
        delta.fixed_rows_mut::<3>(3).copy_from(&dp); // position
        delta.fixed_rows_mut::<3>(6).copy_from(&dv); // velocity
        delta.fixed_rows_mut::<3>(9).copy_from(&dbg); // gyro_bias
        delta.fixed_rows_mut::<3>(12).copy_from(&dba); // accel_bias
        delta.fixed_rows_mut::<2>(15).copy_from(&dg); // gravity

        let basis = gravity_tangent_basis(&state.gravity);
        let (out, out_basis) = box_plus(&state, &basis, &delta);

        assert_eq!(out.position, state.position + dp);
        assert!(
            out.orientation.angle_to(&state.orientation) < ANGLE_TOL,
            "zero delta_theta must leave orientation unchanged"
        );
        assert_eq!(out.velocity, state.velocity + dv);
        assert_eq!(out.gyro_bias, state.gyro_bias + dbg);
        assert_eq!(out.accel_bias, state.accel_bias + dba);
        assert!((out.gravity.norm() - state.gravity.norm()).abs() < 1e-12);
        assert!((gravity_box_minus(&out.gravity, &state.gravity, &basis) - dg).norm() < 1e-12);
        assert!((out_basis.transpose() * out.gravity).norm() < 1e-12);
        assert!((out_basis.transpose() * out_basis - Mat2::identity()).norm() < 1e-12);
    }

    #[test]
    fn box_plus_right_multiplies_orientation() {
        let state = make_state();
        // A pure rotation about the z-axis in the IMU tangent space.
        let delta_theta = Vec3::new(0.0, 0.0, 0.3);

        let mut delta = SVector::<f64, 23>::zeros();
        delta.fixed_rows_mut::<3>(0).copy_from(&delta_theta);

        let basis = gravity_tangent_basis(&state.gravity);
        let (out, _) = box_plus(&state, &basis, &delta);
        // Right-perturbation: R_out = R_wi * Exp(delta_theta)
        let expected = state.orientation * UnitQuaternion::from_scaled_axis(delta_theta);

        assert!(
            out.orientation.angle_to(&expected) < ANGLE_TOL,
            "right perturbation mismatch"
        );
        // Other blocks must be unaffected by the rotation-only error state.
        assert_eq!(out.position, state.position);
        assert_eq!(out.velocity, state.velocity);
        assert_eq!(out.gyro_bias, state.gyro_bias);
        assert_eq!(out.accel_bias, state.accel_bias);
        assert_eq!(out.gravity, state.gravity);
    }

    #[test]
    fn box_minus_zero_between_same_state() {
        let state = make_state();
        let basis = gravity_tangent_basis(&state.gravity);
        let dx = box_minus(&state, &state, &basis);
        assert!(
            dx.norm() < 1e-12,
            "box_minus(same, same) must be the zero error state, got norm={}",
            dx.norm()
        );
    }

    #[test]
    fn box_plus_box_minus_local_round_trip() {
        let state = make_state();

        // A local perturbation in every error-state block that box_plus knows
        // about (rotation, position, velocity, gyro/accel bias, gravity).
        let mut dx = SVector::<f64, 23>::zeros();
        dx.fixed_rows_mut::<3>(0)
            .copy_from(&Vec3::new(0.02, -0.03, 0.04));
        dx.fixed_rows_mut::<3>(3)
            .copy_from(&Vec3::new(0.1, -0.2, 0.3));
        dx.fixed_rows_mut::<3>(6)
            .copy_from(&Vec3::new(0.4, 0.5, -0.6));
        dx.fixed_rows_mut::<3>(9)
            .copy_from(&Vec3::new(0.01, 0.02, 0.03));
        dx.fixed_rows_mut::<3>(12)
            .copy_from(&Vec3::new(0.03, 0.02, 0.01));
        dx.fixed_rows_mut::<2>(15)
            .copy_from(&Vec2::new(0.05, -0.04));

        let basis = gravity_tangent_basis(&state.gravity);
        let (state_perturbed, _) = box_plus(&state, &basis, &dx);
        let dx_round = box_minus(&state_perturbed, &state, &basis);

        // Rotation is exact for right-perturbation: the recovered angle axis
        // must reproduce the injected delta.
        for i in 0..3 {
            assert!(
                (dx_round[i] - dx[i]).abs() < 1e-9,
                "theta block [{i}]: injected={:.12}, round-trip={:.12}",
                dx[i],
                dx_round[i]
            );
        }
        for i in 3..17 {
            assert!(
                (dx_round[i] - dx[i]).abs() < 1e-9,
                "vector block [{i}]: injected={:.12}, round-trip={:.12}",
                dx[i],
                dx_round[i]
            );
        }
    }

    // ---------------------------------------------------------------
    // SO(3) right Jacobian
    // ---------------------------------------------------------------

    #[test]
    fn so3_right_jacobian_zero_is_identity() {
        let jr = so3_right_jacobian(&Vec3::zeros());
        let diff = (jr - Mat3::<f64>::identity()).norm();
        assert!(
            diff < 1e-12,
            "J_r(0) must be exactly the identity, got diff={diff}"
        );
    }

    #[test]
    fn so3_right_jacobian_tiny_angle_is_finite() {
        // Below the `theta >= 1e-6` closed-form threshold the series branch
        // runs; it must stay finite and reduce to identity as theta -> 0.
        let phi = Vec3::new(1e-9, -2e-9, 3e-9);
        let jr = so3_right_jacobian(&phi);
        assert!(jr.iter().all(|v| v.is_finite()), "J_r must stay finite");
        let diff = (jr - Mat3::<f64>::identity()).norm();
        assert!(
            diff < 1e-5,
            "tiny-angle J_r must be close to identity, got diff={diff}"
        );
    }

    #[test]
    fn so3_right_jacobian_matches_finite_difference() {
        // Defining property: for small delta,
        //   log(Exp(phi)^{-1} * Exp(phi + delta)) ~= J_r(phi) * delta.
        // Column j of J_r is the derivative of that log-map w.r.t. delta_j.
        const FD_EPS: f64 = 1e-7;
        const FD_TOL: f64 = 1e-5;

        let phi = Vec3::new(0.4, -0.3, 0.6);
        let r_inv = UnitQuaternion::from_scaled_axis(phi).inverse();
        let jr = so3_right_jacobian(&phi);

        for j in 0..3 {
            let mut dp = Vec3::zeros();
            dp[j] = FD_EPS;
            let mut dm = Vec3::zeros();
            dm[j] = -FD_EPS;

            let v_plus = (r_inv * UnitQuaternion::from_scaled_axis(phi + dp)).scaled_axis();
            let v_minus = (r_inv * UnitQuaternion::from_scaled_axis(phi + dm)).scaled_axis();
            let fd_col = (v_plus - v_minus) / (2.0 * FD_EPS);

            for i in 0..3 {
                assert!(
                    (jr[(i, j)] - fd_col[i]).abs() < FD_TOL,
                    "J_r[{i},{j}]: analytic={:.8}, fd={:.8}, diff={}",
                    jr[(i, j)],
                    fd_col[i],
                    (jr[(i, j)] - fd_col[i]).abs()
                );
            }
        }
    }

    #[test]
    fn s2_reset_jacobian_matches_finite_difference() {
        const FD_EPS: f64 = 1e-7;
        const FD_TOL: f64 = 1e-6;

        let gravity = Vec3::new(0.0, 0.0, -9.81);
        let eta = Vec2::new(0.2, -0.15);
        let gravity_basis = gravity_tangent_basis(&gravity);
        let (gravity_after_injection, basis_after_injection) =
            gravity_box_plus(&gravity, &gravity_basis, &eta);
        let analytic = s2_prior_jacobian(&eta);

        // Reset map:
        // q(epsilon) = (g boxplus (eta + epsilon))
        //              boxminus (g boxplus eta).
        for column in 0..2 {
            let mut epsilon = Vec2::zeros();
            epsilon[column] = FD_EPS;

            let (gravity_plus, _) = gravity_box_plus(&gravity, &gravity_basis, &(eta + epsilon));
            let (gravity_minus, _) = gravity_box_plus(&gravity, &gravity_basis, &(eta - epsilon));
            let q_plus = gravity_box_minus(
                &gravity_plus,
                &gravity_after_injection,
                &basis_after_injection,
            );
            let q_minus = gravity_box_minus(
                &gravity_minus,
                &gravity_after_injection,
                &basis_after_injection,
            );
            let finite_difference = (q_plus - q_minus) / (2.0 * FD_EPS);
            let analytic_column = analytic.column(column);

            assert!(
                (finite_difference - analytic_column).norm() < FD_TOL,
                "J_q column {column} does not match finite difference: analytic={}, fd={}, diff_norm={}",
                analytic_column.transpose(),
                finite_difference.transpose(),
                (finite_difference - analytic_column).norm()
            );
        }
    }

    #[test]
    fn s2_reset_jacobian_matches_after_two_non_collinear_basis_transports() {
        const FD_EPS: f64 = 1e-7;
        const FD_TOL: f64 = 1e-6;

        let gravity_prior = Vec3::new(2.4, -1.7, -9.2).normalize() * 9.81;
        let gravity_basis_prior = gravity_tangent_basis(&gravity_prior);
        let (gravity_after_first, gravity_basis_after_first) = gravity_box_plus(
            &gravity_prior,
            &gravity_basis_prior,
            &Vec2::new(0.20, -0.12),
        );
        let (gravity_iter, gravity_basis_iter) = gravity_box_plus(
            &gravity_after_first,
            &gravity_basis_after_first,
            &Vec2::new(-0.08, 0.17),
        );

        let injected_error = Vec2::new(0.07, -0.11);
        let (gravity_after_injection, gravity_basis_after_injection) =
            gravity_box_plus(&gravity_iter, &gravity_basis_iter, &injected_error);
        let analytic = s2_prior_jacobian(&injected_error);

        // The posterior covariance before reset describes the additive IEKF
        // correction around `injected_error` in the accumulated iter basis.
        for column in 0..2 {
            let mut epsilon = Vec2::zeros();
            epsilon[column] = FD_EPS;

            let (gravity_plus, _) = gravity_box_plus(
                &gravity_iter,
                &gravity_basis_iter,
                &(injected_error + epsilon),
            );
            let (gravity_minus, _) = gravity_box_plus(
                &gravity_iter,
                &gravity_basis_iter,
                &(injected_error - epsilon),
            );
            let reset_error_plus = gravity_box_minus(
                &gravity_plus,
                &gravity_after_injection,
                &gravity_basis_after_injection,
            );
            let reset_error_minus = gravity_box_minus(
                &gravity_minus,
                &gravity_after_injection,
                &gravity_basis_after_injection,
            );
            let finite_difference = (reset_error_plus - reset_error_minus) / (2.0 * FD_EPS);
            let analytic_column = analytic.column(column);

            assert!(
                (finite_difference - analytic_column).norm() < FD_TOL,
                "S2 reset column {column} after accumulated basis transport does not match finite difference: analytic={}, fd={}, diff_norm={}",
                analytic_column.transpose(),
                finite_difference.transpose(),
                (finite_difference - analytic_column).norm()
            );
        }
    }

    // ---------------------------------------------------------------
    // Reset covariance (orientation error injection)
    // ---------------------------------------------------------------

    fn diagonal_covariance_with_cross(base: f64) -> SMatrix<f64, 23, 23> {
        let mut p = SMatrix::<f64, 23, 23>::identity() * base;
        // orientation <-> position cross terms, symmetric by construction.
        let cross = [(0usize, 3usize, 0.02), (1, 4, -0.01), (2, 5, 0.03)];
        for (i, j, v) in cross {
            p[(i, j)] = v;
            p[(j, i)] = v;
        }
        p
    }

    #[test]
    fn reset_covariance_zero_injection_is_unchanged() {
        let p = diagonal_covariance_with_cross(0.1);
        let zero = SVector::<f64, 23>::zeros();
        let p_reset = reset_covariance(&p, &zero);
        let diff = (p_reset - p).norm();
        assert!(
            diff < 1e-12,
            "zero injection must be an identity reset, got diff={diff}"
        );
    }

    #[test]
    fn reset_covariance_transforms_orientation_cross_blocks() {
        let p = diagonal_covariance_with_cross(0.1);
        let dtheta = Vec3::new(0.05, -0.1, 0.2);
        let mut injected = SVector::<f64, 23>::zeros();
        injected.fixed_rows_mut::<3>(0).copy_from(&dtheta);

        let p_reset = reset_covariance(&p, &injected);
        let jr = so3_right_jacobian(&dtheta);

        // Orientation block (0..3) is conjugated by J_r.
        let got_tt = p_reset.fixed_view::<3, 3>(0, 0);
        let exp_tt = jr * p.fixed_view::<3, 3>(0, 0) * jr.transpose();
        assert!(
            (got_tt - exp_tt).norm() < 1e-9,
            "orientation-orientation block must transform as J_r P J_r^T"
        );

        // Cross rows 0..3 vs cols 3..23 scale by J_r on the left.
        let got_tr = p_reset.fixed_view::<3, 20>(0, 3);
        let exp_tr = jr * p.fixed_view::<3, 20>(0, 3);
        assert!(
            (got_tr - exp_tr).norm() < 1e-9,
            "orientation-to-other block must transform as J_r P"
        );

        // Cross cols 0..3 vs rows 3..23 scale by J_r^T on the right.
        let got_bl = p_reset.fixed_view::<20, 3>(3, 0);
        let exp_bl = p.fixed_view::<20, 3>(3, 0) * jr.transpose();
        assert!(
            (got_bl - exp_bl).norm() < 1e-9,
            "other-to-orientation block must transform as P J_r^T"
        );

        // Blocks with no orientation component are untouched.
        let got_br = p_reset.fixed_view::<20, 20>(3, 3);
        let exp_br = p.fixed_view::<20, 20>(3, 3);
        assert!(
            (got_br - exp_br).norm() < 1e-12,
            "orientation-free blocks must be unchanged"
        );
    }

    #[test]
    fn reset_covariance_remains_symmetric_and_spd() {
        let p = diagonal_covariance_with_cross(0.1);
        let mut injected = SVector::<f64, 23>::zeros();
        injected
            .fixed_rows_mut::<3>(0)
            .copy_from(&Vec3::new(0.08, -0.05, 0.12));

        let p_reset = reset_covariance(&p, &injected);

        let asym = (p_reset - p_reset.transpose()).norm();
        assert!(
            asym < 1e-12,
            "reset covariance must stay symmetric, asym={asym}"
        );

        assert!(
            p_reset.cholesky().is_some(),
            "reset covariance must remain SPD"
        );
        assert!(
            p_reset.iter().all(|v| v.is_finite()),
            "reset covariance must remain finite"
        );
    }
}
