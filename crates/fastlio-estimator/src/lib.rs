//! Point-to-plane observation linearization for the FAST-LIO style estimator.
//!
//! This crate follows the FAST-LIO pose notation `T_wi = (R_wi, P_wi)`,
//! transforming points from the IMU frame `I` to the world/map frame `W`:
//!
//! ```text
//! p_w = R_wi * p_i + P_wi
//! r   = n_w^T * (p_w - mean_w)
//! ```
//!
//! `P_wi` is the IMU origin expressed in `W`. Rotation perturbations are applied
//! on the right in the IMU tangent space:
//!
//! ```text
//! R_wi_new = R_wi * Exp(delta_theta_i)
//! P_wi_new = P_wi + delta_P_wi
//! ```
//!
//! The 23D error-state order is:
//!
//! ```text
//! [delta_theta_i, delta_P_wi, delta_v, delta_bg, delta_ba, delta_g, delta_theta_li, delta_P_li]
//! ```
//!
//! Preconditions for the math kernels in this crate:
//! - point coordinates are finite and already expressed in the frame named by
//!   the argument (`point_i` in `I`, `point_l` in `L`);
//! - `SurfelPlaneObservation::norm_w` is finite, unit length, and expressed in `W`;
//! - the observation was produced at the same linearization point as the
//!   `NavState` used to build the Jacobian.

use fastlio_map::surfel::{SurfelLineObservation, SurfelObservation, SurfelPlaneObservation};
use fastlio_types::{LidarImuExtrinsic, Mat3, NavState, PointXYZI, Vec3};
use nalgebra::{SMatrix, SVector};
pub mod iekf;
pub mod optimizer;

#[inline]
pub(crate) fn skew(vec3: &Vec3<f64>) -> Mat3<f64> {
    let (x, y, z) = (vec3.x, vec3.y, vec3.z);
    Mat3::new(0.0, -z, y, z, 0.0, -x, -y, x, 0.0)
}

/// Linearize a point-to-plane residual with respect to the 23D error state.
///
/// `point_i` is a deskewed scan point expressed in the IMU frame `I`. `obs`
/// contains the associated plane in the world frame `W`; its `signed_residual`
/// remains the residual value, while this function returns only the Jacobian
/// row `dr / d(delta_x)`.
///
/// Only the pose blocks are active:
///
/// ```text
/// H[0, 0..3] = -n_w^T * R_wi * [p_i]x
/// H[0, 3..6] =  n_w^T
/// H[0, 6..23] = 0
/// ```
pub fn linearize_point_to_plane_observation(
    state: &NavState,
    point_i: &PointXYZI,
    obs: &SurfelPlaneObservation,
) -> SVector<f64, 23> {
    let r_wi = state.orientation.to_rotation_matrix();
    let r_wi = r_wi.matrix();

    helper(point_i, obs, r_wi)
}

pub fn linearize_point_to_line_observation(
    state: &NavState,
    point_i: &PointXYZI,
    surfel_line_observation: &SurfelLineObservation,
) -> (SVector<f64, 23>, SVector<f64, 23>) {
    let r_wi = state.orientation.to_rotation_matrix();
    let r_wi = r_wi.matrix();
    line_helper(point_i, surfel_line_observation, r_wi)
}

pub fn linearized_point_to_surfel_observation(
    state: &NavState,
    point_i: &PointXYZI,
    _obs: &SurfelObservation,
) -> SMatrix<f64, 3, 23> {
    let r_wi = state.orientation.to_rotation_matrix();
    let r_wi = r_wi.matrix();

    let skew_p = skew_helper(point_i);

    let mut h = SMatrix::<f64, 3, 23>::zeros();

    let j_theta = -r_wi * skew_p;
    let j_position = Mat3::identity();

    h.fixed_view_mut::<3, 3>(0, 0).copy_from(&j_theta);

    h.fixed_view_mut::<3, 3>(0, 3).copy_from(&j_position);

    h
}

#[inline]
fn line_helper(
    point_i: &PointXYZI,
    obs: &SurfelLineObservation,
    r_wi: &Mat3<f64>,
) -> (SVector<f64, 23>, SVector<f64, 23>) {
    let skew_p = skew_helper(point_i);
    let mut h0 = SVector::<f64, 23>::zeros();
    let mut h1 = SVector::<f64, 23>::zeros();
    let j_theta0 = -(obs.normal0_w.transpose() * r_wi * skew_p);
    let j_theta1 = -(obs.normal1_w.transpose() * r_wi * skew_p);
    let j_position0 = obs.normal0_w.transpose();
    let j_position1 = obs.normal1_w.transpose();
    h0.fixed_rows_mut::<3>(0).copy_from(&j_theta0.transpose());
    h0.fixed_rows_mut::<3>(3)
        .copy_from(&j_position0.transpose());
    h1.fixed_rows_mut::<3>(0).copy_from(&j_theta1.transpose());
    h1.fixed_rows_mut::<3>(3)
        .copy_from(&j_position1.transpose());
    (h0, h1)
}

#[inline]
fn helper(point_i: &PointXYZI, obs: &SurfelPlaneObservation, r_wi: &Mat3<f64>) -> SVector<f64, 23> {
    let skew_p = skew_helper(point_i);
    let mut h = SVector::<f64, 23>::zeros();
    let j_theta = -(obs.norm_w.transpose() * r_wi * skew_p);
    let j_position = obs.norm_w.transpose();
    h.fixed_rows_mut::<3>(0).copy_from(&j_theta.transpose());
    h.fixed_rows_mut::<3>(3).copy_from(&j_position.transpose());
    h
}

#[inline]
fn skew_helper(point: &PointXYZI) -> Mat3<f64> {
    let (x, y, z) = (point.x as f64, point.y as f64, point.z as f64);
    Mat3::new(0.0, -z, y, z, 0.0, -x, -y, x, 0.0)
}

/// Linearize a point-to-plane residual including LiDAR-IMU extrinsic blocks.
///
/// `point_l` is the original point in the LiDAR frame `L`, and `point_i` is the
/// same point transformed to `I` using the current `T_li`. The first six
/// columns match [`linearize_point_to_plane_observation`]. Columns `18..24`
/// contain the current extrinsic blocks:
///
/// ```text
/// H[0, 18..21] = -n_w^T * R_wi * R_li * [p_l]x
/// H[0, 21..24] =  n_w^T * R_wi
/// ```
///
/// This function assumes the same right-perturbation convention for `R_li`.
pub fn linearize_point_to_plane_observation_with_extrinsic(
    state: &NavState,
    extrinsic: &LidarImuExtrinsic,
    point_i: &PointXYZI,
    point_l: &PointXYZI,
    obs: &SurfelPlaneObservation,
) -> SVector<f64, 23> {
    let r_wi = state.orientation.to_rotation_matrix();
    let r_wi = r_wi.matrix();
    let r_il = extrinsic.rotation.to_rotation_matrix();
    let r_il = r_il.matrix();

    let mut h = helper(point_i, obs, r_wi);

    let skew_p_l = skew_helper(point_l);
    let j_theta_il = -(obs.norm_w.transpose() * r_wi * r_il * skew_p_l);
    let j_position_il = obs.norm_w.transpose() * r_wi;
    h.fixed_rows_mut::<3>(17).copy_from(&j_theta_il.transpose());
    h.fixed_rows_mut::<3>(20)
        .copy_from(&j_position_il.transpose());
    h
}

pub fn linearize_point_to_line_observation_with_extrinsic(
    state: &NavState,
    extrinsic: &LidarImuExtrinsic,
    point_i: &PointXYZI,
    point_l: &PointXYZI,
    surfel_line_observation: SurfelLineObservation,
) -> (SVector<f64, 23>, SVector<f64, 23>) {
    let r_wi = state.orientation.to_rotation_matrix();
    let r_wi = r_wi.matrix();
    let r_il = extrinsic.rotation.to_rotation_matrix();
    let r_il = r_il.matrix();

    let skew_p_l = skew_helper(point_l);
    let j_theta_il0 = -(surfel_line_observation.normal0_w.transpose() * r_wi * r_il * skew_p_l);
    let j_position_il0 = surfel_line_observation.normal0_w.transpose() * r_wi;
    let j_theta_il1 = -(surfel_line_observation.normal1_w.transpose() * r_wi * r_il * skew_p_l);
    let j_position_il1 = surfel_line_observation.normal1_w.transpose() * r_wi;

    let (mut h0, mut h1) = line_helper(point_i, &surfel_line_observation, r_wi);
    h0.fixed_rows_mut::<3>(17)
        .copy_from(&j_theta_il0.transpose());
    h0.fixed_rows_mut::<3>(20)
        .copy_from(&j_position_il0.transpose());
    h1.fixed_rows_mut::<3>(17)
        .copy_from(&j_theta_il1.transpose());
    h1.fixed_rows_mut::<3>(20)
        .copy_from(&j_position_il1.transpose());
    (h0, h1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use fastlio_map::surfel::SurfelID;
    use fastlio_types::{NavState, Vec3};
    use nalgebra::{Unit, UnitQuaternion};

    const FD_EPS: f64 = 1e-7;
    const FD_TOL: f64 = 1e-4;
    const TAYLOR_EPS: f64 = 1e-5;

    fn dummy_surfel_id() -> SurfelID {
        let mut sm: slotmap::SlotMap<SurfelID, ()> = slotmap::SlotMap::with_key();
        sm.insert(())
    }

    fn make_obs(norm_w: Vec3<f64>, mean_w: Vec3<f64>) -> SurfelPlaneObservation {
        SurfelPlaneObservation::new(
            dummy_surfel_id(),
            mean_w,
            norm_w,
            Vec3::new(0.01, 1.0, 1.0),
            0.0,
            0.01,
            0.0,
        )
    }

    fn make_state() -> NavState {
        NavState {
            position: Vec3::new(1.0, 2.0, 3.0),
            orientation: UnitQuaternion::from_euler_angles(0.3, 0.2, 0.1),
            velocity: Vec3::zeros(),
            gyro_bias: Vec3::zeros(),
            accel_bias: Vec3::zeros(),
            gravity: Vec3::new(0.0, 0.0, -9.81),
        }
    }

    /// Signed residual: r = norm_w^T * (p_W - mean_w)
    /// where p_W = R_WI * p_I + t_WI.
    fn signed_residual(state: &NavState, point_i: &PointXYZI, obs: &SurfelPlaneObservation) -> f64 {
        let r_wi = state.orientation.to_rotation_matrix();
        let p_w = r_wi * point_i.to_vec3_f64() + state.position;
        obs.norm_w.dot(&(p_w - obs.mean_w))
    }

    /// Signed residual for the extrinsic chain:
    ///   p_I = R_LI * p_L + t_LI
    ///   p_W = R_WI * p_I + t_WI
    ///   r   = norm_w^T * (p_W - mean_w)
    ///
    /// Used only to evaluate the residual after perturbing `extrinsic`; the
    /// unperturbed residual `r_0` is taken directly from `obs.signed_residual`.
    fn signed_residual_with_extrinsic(
        state: &NavState,
        extrinsic: &LidarImuExtrinsic,
        point_l: &PointXYZI,
        obs: &SurfelPlaneObservation,
    ) -> f64 {
        let p_l = point_l.to_vec3_f64();
        let p_i = extrinsic.rotation * p_l + extrinsic.translation;
        let p_w = state.orientation.to_rotation_matrix() * p_i + state.position;
        obs.norm_w.dot(&(p_w - obs.mean_w))
    }

    // ---------------------------------------------------------------
    // 1. Verify first-order Taylor expansion holds for the analytical
    //    Jacobian — confirms the linearization is consistent with the
    //    signed residual r = norm_w^T (p_W - mean_w).
    // ---------------------------------------------------------------
    #[test]
    fn jacobian_matches_first_order_residual_changes() {
        let state = make_state();
        let point_i = PointXYZI {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            intensity: 0.0,
        };
        let obs = make_obs(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.5, 0.3, 0.0));
        let h = linearize_point_to_plane_observation(&state, &point_i, &obs);
        let r0 = signed_residual(&state, &point_i, &obs);

        // delta_theta_i is applied on the right:
        // R_wi_new = R_wi * Exp(delta_theta_i)
        for i in 0..3 {
            let mut axis_vec = Vec3::zeros();
            axis_vec[i] = 1.0;
            let axis = Unit::new_unchecked(axis_vec);

            let dq = UnitQuaternion::from_axis_angle(&axis, 1e-4);
            let state_p = NavState {
                orientation: state.orientation * dq,
                ..state.clone()
            };
            let r_p = signed_residual(&state_p, &point_i, &obs);
            let predicted = r0 + h[i] * 1e-4;
            assert!(
                (r_p - predicted).abs() < TAYLOR_EPS,
                "theta[{i}]: r_pert={r_p:.8}, predicted={predicted:.8}, diff={}",
                (r_p - predicted).abs()
            );
        }

        // delta_position
        for i in 0..3 {
            let mut delta = Vec3::zeros();
            delta[i] = 1e-4;
            let state_p = NavState {
                position: state.position + delta,
                ..state.clone()
            };
            let r_p = signed_residual(&state_p, &point_i, &obs);
            let predicted = r0 + h[3 + i] * 1e-4;
            assert!(
                (r_p - predicted).abs() < TAYLOR_EPS,
                "pos[{i}]: r_pert={r_p:.8}, predicted={predicted:.8}, diff={}",
                (r_p - predicted).abs()
            );
        }
    }

    // ---------------------------------------------------------------
    // 2. Central-difference check for position Jacobian block (cols 3..6).
    // ---------------------------------------------------------------
    #[test]
    fn position_jacobian_matches_finite_difference() {
        let state = make_state();
        let point_i = PointXYZI {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            intensity: 0.0,
        };
        let obs = make_obs(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.5, 0.3, 0.0));
        let h = linearize_point_to_plane_observation(&state, &point_i, &obs);

        for i in 0..3 {
            let mut sp = state.clone();
            let mut sm = state.clone();
            sp.position[i] += FD_EPS;
            sm.position[i] -= FD_EPS;
            let fd = (signed_residual(&sp, &point_i, &obs) - signed_residual(&sm, &point_i, &obs))
                / (2.0 * FD_EPS);
            assert!(
                (h[3 + i] - fd).abs() < FD_TOL,
                "J_pos[{i}]: analytical={:.8}, fd={:.8}, diff={}",
                h[3 + i],
                fd,
                (h[3 + i] - fd).abs()
            );
        }
    }

    // ---------------------------------------------------------------
    // 3. Central-difference check for orientation Jacobian block (cols 0..3).
    //    Right perturbation: R_pert = R_wi * Exp([dtheta_i]x).
    // ---------------------------------------------------------------
    #[test]
    fn orientation_jacobian_matches_finite_difference() {
        let state = make_state();
        let point_i = PointXYZI {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            intensity: 0.0,
        };
        let obs = make_obs(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.5, 0.3, 0.0));
        let h = linearize_point_to_plane_observation(&state, &point_i, &obs);

        for i in 0..3 {
            let mut axis_vec = Vec3::zeros();
            axis_vec[i] = 1.0;
            let axis = Unit::new_unchecked(axis_vec);

            let dq_p = UnitQuaternion::from_axis_angle(&axis, FD_EPS);
            let sp = NavState {
                orientation: state.orientation * dq_p,
                ..state.clone()
            };

            let dq_m = UnitQuaternion::from_axis_angle(&axis, -FD_EPS);
            let sm = NavState {
                orientation: state.orientation * dq_m,
                ..state.clone()
            };

            let fd = (signed_residual(&sp, &point_i, &obs) - signed_residual(&sm, &point_i, &obs))
                / (2.0 * FD_EPS);
            assert!(
                (h[i] - fd).abs() < FD_TOL,
                "J_theta[{i}]: analytical={:.8}, fd={:.8}, diff={}",
                h[i],
                fd,
                (h[i] - fd).abs()
            );
        }
    }

    // ---------------------------------------------------------------
    // 4. Inactive state blocks (cols 6..24) must be exactly zero.
    // ---------------------------------------------------------------
    #[test]
    fn inactive_state_blocks_are_zero() {
        let state = make_state();
        let point_i = PointXYZI {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            intensity: 0.0,
        };
        let obs = make_obs(Vec3::new(0.0, 0.0, 1.0), Vec3::zeros());
        let h = linearize_point_to_plane_observation(&state, &point_i, &obs);

        for col in 6..23 {
            assert!(
                h[col].abs() < 1e-15,
                "inactive col {col} should be zero, got {}",
                h[col]
            );
        }
    }

    // ---------------------------------------------------------------
    // 5. With identity pose the Jacobian blocks reduce to closed-form:
    //      J_theta    = -norm_w^T * [p_I]×
    //      J_position =  norm_w^T
    //    Verify each block element-by-element.
    // ---------------------------------------------------------------
    #[test]
    fn jacobian_has_expected_pose_blocks_for_identity_pose() {
        let state = NavState {
            position: Vec3::new(4.0, 5.0, 6.0),
            orientation: UnitQuaternion::identity(),
            velocity: Vec3::zeros(),
            gyro_bias: Vec3::zeros(),
            accel_bias: Vec3::zeros(),
            gravity: Vec3::new(0.0, 0.0, -9.81),
        };
        let point_i = PointXYZI {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            intensity: 0.0,
        };
        let n = Vec3::new(0.0, 0.0, 1.0);
        let obs = make_obs(n, Vec3::new(0.5, 0.3, 0.0));
        let h = linearize_point_to_plane_observation(&state, &point_i, &obs);

        // With R_WI = I:  J_theta = -norm_w^T * [p_I]×
        //                   J_position = norm_w^T
        //
        // [p_I]× = |  0  -3   2 |
        //          |  3   0  -1 |
        //          | -2   1   0 |
        //
        // -norm_w^T * [p_I]× = -[0,0,1] * [p_I]× = -[-2, 1, 0] = [2, -1, 0]
        let expected_j_theta = Vec3::new(2.0, -1.0, 0.0);
        let expected_j_position = n;

        for i in 0..3 {
            assert!(
                (h[i] - expected_j_theta[i]).abs() < 1e-12,
                "J_theta[{i}]: expected={}, got={}",
                expected_j_theta[i],
                h[i]
            );
            assert!(
                (h[3 + i] - expected_j_position[i]).abs() < 1e-12,
                "J_position[{i}]: expected={}, got={}",
                expected_j_position[i],
                h[3 + i]
            );
        }

        // Remaining blocks must be zero.
        for col in 6..23 {
            assert!(
                h[col].abs() < 1e-15,
                "inactive col {col} should be zero, got {}",
                h[col]
            );
        }
    }

    // ---------------------------------------------------------------
    // Extrinsic-chain (with_extrinsic) finite difference checks.
    //
    // Residual chain:
    //   p_I = R_LI * p_L + t_LI
    //   p_W = R_WI * p_I + t_WI
    //   r   = norm_w^T * (p_W - mean_w)
    //
    // Extrinsic blocks (cols 18..24):
    //   J_theta_li = -n_w^T * R_wi * R_li * [p_l]×   (cols 18..21)
    //   J_pos_li   =  n_w^T * R_wi                   (cols 21..24)
    // Both rotations use right-perturbation.
    // ---------------------------------------------------------------
    fn make_extrinsic() -> LidarImuExtrinsic {
        LidarImuExtrinsic::new(
            UnitQuaternion::from_euler_angles(0.4, -0.3, 0.2),
            Vec3::new(0.1, -0.2, 0.3),
        )
    }

    fn make_point_l() -> PointXYZI {
        PointXYZI {
            x: -1.5,
            y: 2.0,
            z: 0.75,
            intensity: 10.0,
        }
    }

    #[test]
    fn extrinsic_orientation_jacobian_matches_finite_difference() {
        let state = make_state();
        let extrinsic = make_extrinsic();
        let point_l = make_point_l();
        let obs = make_obs(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.5, 0.3, 0.0));
        let h = linearize_point_to_plane_observation_with_extrinsic(
            &state, &extrinsic, &point_l, &point_l, &obs,
        );

        // cols 18..21: perturbation of R_LI on the right: R_LI * Exp(δθ_li)
        for i in 0..3 {
            let mut axis_vec = Vec3::zeros();
            axis_vec[i] = 1.0;
            let axis = Unit::new_unchecked(axis_vec);

            let sp = LidarImuExtrinsic::new(
                extrinsic.rotation * UnitQuaternion::from_axis_angle(&axis, FD_EPS),
                extrinsic.translation,
            );
            let sm = LidarImuExtrinsic::new(
                extrinsic.rotation * UnitQuaternion::from_axis_angle(&axis, -FD_EPS),
                extrinsic.translation,
            );
            let rp = signed_residual_with_extrinsic(&state, &sp, &point_l, &obs);
            let rm = signed_residual_with_extrinsic(&state, &sm, &point_l, &obs);
            let fd = (rp - rm) / (2.0 * FD_EPS);
            assert!(
                (h[17 + i] - fd).abs() < FD_TOL,
                "J_theta_li[{i}]: analytical={:.8}, fd={:.8}, diff={}",
                h[17 + i],
                fd,
                (h[17 + i] - fd).abs()
            );
        }
    }

    #[test]
    fn extrinsic_position_jacobian_matches_finite_difference() {
        let state = make_state();
        let extrinsic = make_extrinsic();
        let point_l = make_point_l();
        let obs = make_obs(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.5, 0.3, 0.0));
        let h = linearize_point_to_plane_observation_with_extrinsic(
            &state, &extrinsic, &point_l, &point_l, &obs,
        );

        // cols 21..24: perturbation of t_LI
        for i in 0..3 {
            let mut tp = extrinsic.translation;
            let mut tm = extrinsic.translation;
            tp[i] += FD_EPS;
            tm[i] -= FD_EPS;
            let sp = LidarImuExtrinsic::new(extrinsic.rotation, tp);
            let sm = LidarImuExtrinsic::new(extrinsic.rotation, tm);
            let rp = signed_residual_with_extrinsic(&state, &sp, &point_l, &obs);
            let rm = signed_residual_with_extrinsic(&state, &sm, &point_l, &obs);
            let fd = (rp - rm) / (2.0 * FD_EPS);
            assert!(
                (h[20 + i] - fd).abs() < FD_TOL,
                "J_pos_li[{i}]: analytical={:.8}, fd={:.8}, diff={}",
                h[20 + i],
                fd,
                (h[20 + i] - fd).abs()
            );
        }
    }

    #[test]
    fn extrinsic_pose_blocks_match_without_extrinsic_jacobian() {
        // The pose blocks (cols 0..6) must be identical regardless of whether
        // the extrinsic chain is used, since pose blocks depend only on R_WI.
        let state = make_state();
        let extrinsic = make_extrinsic();
        let point_l = make_point_l();
        let point_i = {
            let p_l = point_l.to_vec3_f64();
            let p_i = extrinsic.rotation * p_l + extrinsic.translation;
            PointXYZI {
                x: p_i.x as f32,
                y: p_i.y as f32,
                z: p_i.z as f32,
                intensity: point_l.intensity,
            }
        };
        let obs = make_obs(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.5, 0.3, 0.0));

        let h_pose_only = linearize_point_to_plane_observation(&state, &point_i, &obs);
        let h_with_ext = linearize_point_to_plane_observation_with_extrinsic(
            &state, &extrinsic, &point_i, &point_l, &obs,
        );

        for col in 0..6 {
            assert!(
                (h_pose_only[col] - h_with_ext[col]).abs() < 1e-12,
                "pose col {col} differs: pose-only={:.12}, with-ext={:.12}",
                h_pose_only[col],
                h_with_ext[col]
            );
        }
    }

    #[test]
    fn extrinsic_inactive_blocks_are_zero() {
        // cols 6..18 (v, bg, ba, g) and the gap must be exactly zero.
        let state = make_state();
        let extrinsic = make_extrinsic();
        let point_l = make_point_l();
        let obs = make_obs(Vec3::new(0.0, 0.0, 1.0), Vec3::zeros());
        let h = linearize_point_to_plane_observation_with_extrinsic(
            &state, &extrinsic, &point_l, &point_l, &obs,
        );

        for col in 6..17 {
            assert!(
                h[col].abs() < 1e-15,
                "inactive col {col} should be zero, got {}",
                h[col]
            );
        }
    }

    #[test]
    fn extrinsic_jacobian_matches_first_order_residual_changes() {
        // Verify the analytical Jacobian's first-order prediction matches the
        // actual residual change for both extrinsic blocks simultaneously.
        let state = make_state();
        let extrinsic = make_extrinsic();
        let point_l = make_point_l();
        let obs = make_obs(Vec3::new(0.0, 0.0, 1.0), Vec3::new(0.5, 0.3, 0.0));
        let h = linearize_point_to_plane_observation_with_extrinsic(
            &state, &extrinsic, &point_l, &point_l, &obs,
        );
        let r0 = signed_residual_with_extrinsic(&state, &extrinsic, &point_l, &obs);

        // delta_theta_li (cols 18..21)
        for i in 0..3 {
            let mut axis_vec = Vec3::zeros();
            axis_vec[i] = 1.0;
            let axis = Unit::new_unchecked(axis_vec);
            let dq = UnitQuaternion::from_axis_angle(&axis, 1e-4);
            let state_p = LidarImuExtrinsic::new(extrinsic.rotation * dq, extrinsic.translation);
            let rp = signed_residual_with_extrinsic(&state, &state_p, &point_l, &obs);
            let predicted = r0 + h[17 + i] * 1e-4;
            assert!(
                (rp - predicted).abs() < TAYLOR_EPS,
                "theta_li[{i}]: r_pert={rp:.8}, predicted={predicted:.8}, diff={}",
                (rp - predicted).abs()
            );
        }

        // delta_t_li (cols 21..24)
        for i in 0..3 {
            let mut dt = Vec3::zeros();
            dt[i] = 1e-4;
            let state_p = LidarImuExtrinsic::new(extrinsic.rotation, extrinsic.translation + dt);
            let rp = signed_residual_with_extrinsic(&state, &state_p, &point_l, &obs);
            let predicted = r0 + h[20 + i] * 1e-4;
            assert!(
                (rp - predicted).abs() < TAYLOR_EPS,
                "pos_li[{i}]: r_pert={rp:.8}, predicted={predicted:.8}, diff={}",
                (rp - predicted).abs()
            );
        }
    }
}
