use anyhow::Result;
use fastlio_map::surfel::{SurfelMap, SurfelObservation};
use fastlio_types::{NavState, PointXYZI};
use nalgebra::{SMatrix, SVector};

use crate::{iekf::box_minus, linearize_point_to_plane_observation};

pub struct LinearizedObservation {
    pub residual: f64,
    pub jacobian: SMatrix<f64, 1, 24>,
    pub variance: f64,
}

/// Configuration for the current 24D error-state IEKF update.
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

pub(crate) fn converged(error_state: &SVector<f64, 24>, config: &IekfConfig) -> bool {
    error_state.norm() <= config.min_delta_norm
}

pub(crate) fn build_observations(
    state: &NavState,
    points: &[PointXYZI],
    map: &SurfelMap,
    config: &IekfConfig,
    out: &mut Vec<LinearizedObservation>,
) -> Result<()> {
    out.clear();

    for point in points {
        let point_w = transform_point(state, point);

        let Some(obs) = map.query(&point_w)? else {
            continue;
        };

        let jacobian = linearize_point_to_plane_observation(state, point, &obs);

        let residual = obs.signed_residual;

        let variance = build_variance(&obs, config);

        out.push(LinearizedObservation {
            jacobian,
            residual,
            variance,
        });
    }
    Ok(())
}

pub(crate) fn build_variance(obs: &SurfelObservation, config: &IekfConfig) -> f64 {
    let min_eigenvalue = obs.eigenvalues[0];
    min_eigenvalue.max(config.measurement_variance_floor)
}

pub(crate) fn transform_point(state: &NavState, point: &PointXYZI) -> PointXYZI {
    let mut point_vec = point.to_vec3_f64();
    let t = state.position;
    let r = state.orientation.to_rotation_matrix();
    let r = r.matrix();

    point_vec = r * point_vec + t;

    PointXYZI {
        x: point_vec[0] as f32,
        y: point_vec[1] as f32,
        z: point_vec[2] as f32,
        intensity: point.intensity,
    }
}

pub(crate) fn linear_update(
    state: &NavState,
    state_iter: &NavState,
    covariance: &SMatrix<f64, 24, 24>,
    observations: &[LinearizedObservation],
    config: &IekfConfig,
) -> Option<(SVector<f64, 24>, SMatrix<f64, 24, 24>)> {
    let p_chol = covariance.cholesky()?;
    let l = p_chol.l();

    // Current IEKF linear solve is written in whitened information form.
    //
    // P_prior = L L^T, dx = L y.
    // `prior_error` is the local error from the prior state to the current
    // iteration state. The solved `dx` is then interpreted as a correction on
    // the current iteration state.
    let prior_error = box_minus(state_iter, state);

    // L * e_white = prior_error.
    let e_white = l.solve_lower_triangular(&prior_error)?;

    let mut information = SMatrix::<f64, 24, 24>::identity();
    let mut rhs = -e_white;

    for obs in observations {
        if !obs.residual.is_finite() || !obs.variance.is_finite() || obs.variance <= 0.0 {
            return None;
        }

        let w = 1.0 / obs.variance;

        // 1x24
        let j = obs.jacobian * l;

        information += j.transpose() * w * j;
        rhs -= j.transpose() * w * obs.residual;
    }

    information += SMatrix::<f64, 24, 24>::identity() * config.damping;

    let information = symmetric(&information);
    let chol = information.cholesky()?;

    let y = chol.solve(&rhs);
    let dx = l * y;
    // TODO(iekf): consider storing the posterior covariance factor directly:
    // P_post = L A^-1 L^T = (L C^-T)(L C^-T)^T, where A = C C^T.
    // The current covariance form is simpler for API/tests, but forms A^-1.
    let a_inv = chol.inverse();
    let post_covariance = l * a_inv * l.transpose();
    Some((dx, post_covariance))
}

fn symmetric(covariance: &SMatrix<f64, 24, 24>) -> SMatrix<f64, 24, 24> {
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
    fn diagonal_covariance(base: f64, overrides: &[(usize, f64)]) -> SMatrix<f64, 24, 24> {
        let mut c = SMatrix::<f64, 24, 24>::identity() * base;
        for &(i, v) in overrides {
            c[(i, i)] = v;
        }
        c
    }

    fn position_z_observation(residual: f64, variance: f64) -> LinearizedObservation {
        // Jacobian selects only the position-z error state column (index 5).
        let mut h = SMatrix::<f64, 1, 24>::zeros();
        h[(0, 5)] = 1.0;
        LinearizedObservation {
            residual,
            jacobian: h,
            variance,
        }
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

        let (dx, post) = linear_update(&state, &state_iter, &covariance, &observations, &config)
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
        let (dx, _post) = linear_update(&state, &state_iter, &covariance, &observations, &config)
            .expect("linear solve must succeed");

        let expected = -p_z * r / (p_z + var);
        assert!(
            (dx[5] - expected).abs() < TOL,
            "position-z correction: got={:.12}, expected={expected:.12}",
            dx[5]
        );
        for i in 0..24 {
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

        let mut e = SVector::<f64, 24>::zeros();
        e.fixed_rows_mut::<3>(0)
            .copy_from(&Vec3::new(0.05, 0.0, 0.0));
        e.fixed_rows_mut::<3>(3)
            .copy_from(&Vec3::new(0.2, -0.1, 0.3));
        e.fixed_rows_mut::<3>(6)
            .copy_from(&Vec3::new(0.4, 0.0, -0.2));
        let state_iter = box_plus(&state, &e);

        let observations = Vec::new();
        let (dx, post) = linear_update(&state, &state_iter, &covariance, &observations, &config)
            .expect("linear solve must succeed");

        // dx must be exactly the negative of the prior error (empty update).
        let neg_prior_error = -box_minus(&state_iter, &state);
        assert!(
            (dx - neg_prior_error).norm() < TOL,
            "dx should equal -prior_error, diff norm={}",
            (dx - neg_prior_error).norm()
        );

        // Re-composing must return to the prior state.
        let back = box_plus(&state_iter, &dx);
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
        // With no observations the covariance is unchanged.
        assert!(
            (post - covariance).norm() < 1e-6,
            "covariance must be unchanged with no observations"
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

        let (dx, _post) = linear_update(&state, &state_iter, &covariance, &observations, &config)
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

        let (dx, _post) = linear_update(&state, &state_iter, &covariance, &observations, &config)
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

        let (_, post) = linear_update(&state, &state_iter, &covariance, &observations, &config)
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

        assert!(
            linear_update(&state, &state_iter, &covariance, &zero_var, &config).is_none(),
            "zero measurement variance must be rejected"
        );
        assert!(
            linear_update(&state, &state_iter, &covariance, &negative_var, &config).is_none(),
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
            covariance,
        };
        single.update(&body_points, &map, &config).unwrap();

        let config_iter = IekfConfig {
            max_iterations: 15,
            ..config
        };
        let mut iterated = IekfState {
            state: prior.clone(),
            covariance,
        };
        iterated.update(&body_points, &map, &config_iter).unwrap();

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
            let world = transform_point(state, p);
            if let Some(obs) = map.query(&world).expect("query must not error") {
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
