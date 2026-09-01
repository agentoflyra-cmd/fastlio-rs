use anyhow::{Result, anyhow};
use fastlio_map::surfel::SurfelMap;
use fastlio_types::{NavState, PointXYZI};
use nalgebra::{SMatrix, SVector, UnitQuaternion};

use crate::optimizer::{IekfConfig, build_observations, converged, linear_update};
/// ```text
/// [delta_theta_i, delta_P_wi, delta_v, delta_bg, delta_ba, delta_g, delta_theta_li, delta_P_li]
/// ```
// Not yet wired into the pipeline; only exercised from tests for now.
pub(crate) fn box_plus(state: &NavState, error_state: &SVector<f64, 24>) -> NavState {
    let delta_theta = error_state.fixed_rows::<3>(0).into_owned();
    let delta_rotation = UnitQuaternion::from_scaled_axis(delta_theta);

    NavState {
        position: state.position + error_state.fixed_rows::<3>(3).into_owned(),
        orientation: state.orientation * delta_rotation,
        velocity: state.velocity + error_state.fixed_rows::<3>(6).into_owned(),
        gyro_bias: state.gyro_bias + error_state.fixed_rows::<3>(9).into_owned(),
        accel_bias: state.accel_bias + error_state.fixed_rows::<3>(12).into_owned(),
        gravity: state.gravity + error_state.fixed_rows::<3>(15).into_owned(),
    }
}

pub(crate) fn box_minus(state_iter: &NavState, state: &NavState) -> SVector<f64, 24> {
    let theta_iter = state_iter.orientation;
    let theta = state.orientation;
    let dtheta = (theta.inverse() * theta_iter).scaled_axis();

    let mut dx = SVector::<f64, 24>::zeros();
    dx.fixed_rows_mut::<3>(0).copy_from(&dtheta);
    dx.fixed_rows_mut::<3>(3)
        .copy_from(&(state_iter.position - state.position));
    dx.fixed_rows_mut::<3>(6)
        .copy_from(&(state_iter.velocity - state.velocity));
    dx.fixed_rows_mut::<3>(9)
        .copy_from(&(state_iter.gyro_bias - state.gyro_bias));
    dx.fixed_rows_mut::<3>(12)
        .copy_from(&(state_iter.accel_bias - state.accel_bias));
    dx.fixed_rows_mut::<3>(15)
        .copy_from(&(state_iter.gravity - state.gravity));
    dx
}

pub struct IekfState {
    pub state: NavState,
    pub covariance: SMatrix<f64, 24, 24>,
}

impl IekfState {
    pub fn update(
        &mut self,
        points: &[PointXYZI],
        map: &SurfelMap,
        config: &IekfConfig,
    ) -> Result<()> {
        let state_prior = self.state.clone();
        let p_prior = self.covariance;

        let mut state_iter = state_prior.clone();
        let mut p_final = p_prior;
        let mut observations = Vec::new();

        for _ in 0..config.max_iterations {
            build_observations(&state_iter, points, map, config, &mut observations)?;

            if observations.len() < config.min_observations {
                return Err(anyhow!("IEKF update failed: not enough observations"));
            }

            let Some((error_state, p_work)) =
                linear_update(&state_prior, &state_iter, &p_prior, &observations, config)
            else {
                // TODO(iekf): replace anyhow with a typed update status before
                // this is wired into the main pipeline diagnostics.
                return Err(anyhow!(
                    "IEKF update failed: linear solve or SPD check failed."
                ));
            };

            state_iter = box_plus(&state_iter, &error_state);
            p_final = p_work;

            if converged(&error_state, config) {
                break;
            }
        }

        self.state = state_iter;
        self.covariance = p_final;
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
        let zero = SVector::<f64, 24>::zeros();
        let out = box_plus(&state, &zero);

        assert_eq!(out.position, state.position);
        assert!(
            out.orientation.angle_to(&state.orientation) < ANGLE_TOL,
            "orientation changed under zero delta"
        );
        assert_eq!(out.velocity, state.velocity);
        assert_eq!(out.gyro_bias, state.gyro_bias);
        assert_eq!(out.accel_bias, state.accel_bias);
        assert_eq!(out.gravity, state.gravity);
    }

    #[test]
    fn box_plus_adds_translation_velocity_bias_gravity() {
        let state = make_state();
        let dp = Vec3::new(0.1, -0.2, 0.3);
        let dv = Vec3::new(0.5, 0.5, 0.5);
        let dbg = Vec3::new(0.01, 0.01, 0.01);
        let dba = Vec3::new(0.02, 0.02, 0.02);
        let dg = Vec3::new(1.0, 0.0, 0.5);

        let mut delta = SVector::<f64, 24>::zeros();
        delta.fixed_rows_mut::<3>(3).copy_from(&dp); // position
        delta.fixed_rows_mut::<3>(6).copy_from(&dv); // velocity
        delta.fixed_rows_mut::<3>(9).copy_from(&dbg); // gyro_bias
        delta.fixed_rows_mut::<3>(12).copy_from(&dba); // accel_bias
        delta.fixed_rows_mut::<3>(15).copy_from(&dg); // gravity

        let out = box_plus(&state, &delta);

        assert_eq!(out.position, state.position + dp);
        assert!(
            out.orientation.angle_to(&state.orientation) < ANGLE_TOL,
            "zero delta_theta must leave orientation unchanged"
        );
        assert_eq!(out.velocity, state.velocity + dv);
        assert_eq!(out.gyro_bias, state.gyro_bias + dbg);
        assert_eq!(out.accel_bias, state.accel_bias + dba);
        assert_eq!(out.gravity, state.gravity + dg);
    }

    #[test]
    fn box_plus_right_multiplies_orientation() {
        let state = make_state();
        // A pure rotation about the z-axis in the IMU tangent space.
        let delta_theta = Vec3::new(0.0, 0.0, 0.3);

        let mut delta = SVector::<f64, 24>::zeros();
        delta.fixed_rows_mut::<3>(0).copy_from(&delta_theta);

        let out = box_plus(&state, &delta);
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
        let dx = box_minus(&state, &state);
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
        let mut dx = SVector::<f64, 24>::zeros();
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
        dx.fixed_rows_mut::<3>(15)
            .copy_from(&Vec3::new(0.5, 0.0, -0.5));

        let state_perturbed = box_plus(&state, &dx);
        let dx_round = box_minus(&state_perturbed, &state);

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
        for i in 3..18 {
            assert!(
                (dx_round[i] - dx[i]).abs() < 1e-9,
                "vector block [{i}]: injected={:.12}, round-trip={:.12}",
                dx[i],
                dx_round[i]
            );
        }
    }
}
