use fastlio_map::PlaneFit;
use fastlio_types::{NavState, Vec3};
use nalgebra::{SMatrix, SVector, UnitQuaternion};

pub type ErrorStateVector = SVector<f64, 24>;
pub type ErrorStateCovariance = SMatrix<f64, 24, 24>;

/// One IESEKF point-to-plane measurement.
///
/// `point_i` is expressed in IMU/body frame `I`. The fitted plane is expressed
/// in world/map frame `W`. The residual is:
/// `r = normal_w.dot(R_WI * point_i + position_wi) + offset`.
#[derive(Debug, Clone)]
pub struct IesekfPointToPlaneFactor {
    pub point_i: Vec3<f64>,
    pub plane_w: PlaneFit,
    pub weight: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct IesekfConfig {
    pub max_iterations: usize,
    pub min_delta_norm: f64,
    pub measurement_noise_variance: f64,
    pub covariance_epsilon: f64,
}

impl Default for IesekfConfig {
    fn default() -> Self {
        Self {
            max_iterations: 5,
            min_delta_norm: 1.0e-6,
            measurement_noise_variance: 1.0e-3,
            covariance_epsilon: 1.0e-12,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct IesekfUpdateReport {
    pub initial_cost: f64,
    pub final_cost: f64,
    pub iterations: usize,
    pub converged: bool,
    pub error_state: ErrorStateVector,
}

#[derive(Debug, Clone, PartialEq)]
pub enum IesekfError {
    EmptyMeasurements,
    InvalidConfig,
    NonFiniteInput,
    SingularCovariance,
    SingularInnovation,
}

/// Minimal 24-dimensional IESEKF front-end state.
///
/// Error-state order:
/// `[δθ_I, δp_I, δv_I, δb_g, δb_a, δg, δθ_LI, δp_LI]`.
///
/// Orientation error uses right perturbation during injection:
/// `R_WI' = R_WI * Exp(δθ_I)`. Position, velocity, biases and gravity are
/// updated additively in their stored frames. The current nominal state does not
/// yet carry LiDAR-IMU extrinsic parameters, so the last six error components
/// remain covariance-only until the nominal extrinsic state is introduced.
#[derive(Clone)]
pub struct Iesekf {
    pub state: NavState,
    pub covariance: ErrorStateCovariance,
}

impl Iesekf {
    pub fn new(state: NavState, covariance: ErrorStateCovariance) -> Result<Self, IesekfError> {
        if !navstate_is_finite(&state) || !matrix_is_finite(&covariance) {
            return Err(IesekfError::NonFiniteInput);
        }

        Ok(Self {
            state,
            covariance: symmetrize(covariance),
        })
    }

    pub fn set_predicted(
        &mut self,
        state: NavState,
        covariance: ErrorStateCovariance,
    ) -> Result<(), IesekfError> {
        if !navstate_is_finite(&state) || !matrix_is_finite(&covariance) {
            return Err(IesekfError::NonFiniteInput);
        }

        self.state = state;
        self.covariance = symmetrize(covariance);
        Ok(())
    }

    pub fn update_point_to_plane_iterated(
        &mut self,
        factors: &[IesekfPointToPlaneFactor],
        config: IesekfConfig,
    ) -> Result<IesekfUpdateReport, IesekfError> {
        validate_update_inputs(factors, config)?;

        let prior_state = self.state.clone();
        let prior_covariance = symmetrize(self.covariance);
        let prior_precision = invert_spd(&regularize(prior_covariance, config.covariance_epsilon))
            .ok_or(IesekfError::SingularCovariance)?;

        let initial_cost = total_cost(&prior_state, factors);
        let mut error_state = ErrorStateVector::zeros();
        let mut iter_state = prior_state.clone();
        let mut final_cost = initial_cost;
        let mut converged = false;
        let mut iterations = 0;

        let mut posterior_covariance = prior_covariance;

        for _ in 0..config.max_iterations {
            let (information, rhs) = build_information_system(
                &iter_state,
                &error_state,
                factors,
                &prior_precision,
                config,
            );
            let Some(cholesky) = information.cholesky() else {
                return Err(IesekfError::SingularInnovation);
            };

            let next_error_state = cholesky.solve(&rhs);
            if !next_error_state.iter().all(|value| value.is_finite()) {
                return Err(IesekfError::NonFiniteInput);
            }

            iter_state = inject_error_state(&prior_state, &next_error_state);
            posterior_covariance = symmetrize(cholesky.inverse());
            final_cost = total_cost(&iter_state, factors);
            iterations += 1;

            let delta_norm = (next_error_state - error_state).norm();
            error_state = next_error_state;
            if delta_norm < config.min_delta_norm {
                converged = true;
                break;
            }
        }

        self.state = iter_state;
        self.covariance = enforce_covariance_floor(posterior_covariance, config.covariance_epsilon);

        Ok(IesekfUpdateReport {
            initial_cost,
            final_cost,
            iterations,
            converged,
            error_state,
        })
    }
}

pub fn point_to_plane_residual(state: &NavState, factor: &IesekfPointToPlaneFactor) -> f64 {
    let point_w = state.orientation * factor.point_i + state.position;
    factor.plane_w.normal_w.dot(&point_w) + factor.plane_w.offset
}

pub fn point_to_plane_jacobian(
    state: &NavState,
    factor: &IesekfPointToPlaneFactor,
) -> ErrorStateVector {
    let normal_i = state.orientation.inverse() * factor.plane_w.normal_w;
    let rotation_jacobian = factor.point_i.cross(&normal_i);

    let mut jacobian = ErrorStateVector::zeros();
    jacobian
        .fixed_rows_mut::<3>(0)
        .copy_from(&rotation_jacobian);
    jacobian
        .fixed_rows_mut::<3>(3)
        .copy_from(&factor.plane_w.normal_w);
    jacobian
}

pub fn inject_error_state(prior: &NavState, error_state: &ErrorStateVector) -> NavState {
    let dtheta = error_state.fixed_rows::<3>(0).into_owned();
    let delta_rotation = UnitQuaternion::from_scaled_axis(dtheta);

    NavState {
        orientation: prior.orientation * delta_rotation,
        position: prior.position + error_state.fixed_rows::<3>(3).into_owned(),
        velocity: prior.velocity + error_state.fixed_rows::<3>(6).into_owned(),
        gyro_bias: prior.gyro_bias + error_state.fixed_rows::<3>(9).into_owned(),
        accel_bias: prior.accel_bias + error_state.fixed_rows::<3>(12).into_owned(),
        gravity: prior.gravity + error_state.fixed_rows::<3>(15).into_owned(),
    }
}

fn validate_update_inputs(
    factors: &[IesekfPointToPlaneFactor],
    config: IesekfConfig,
) -> Result<(), IesekfError> {
    if factors.is_empty() {
        return Err(IesekfError::EmptyMeasurements);
    }
    if config.max_iterations == 0
        || config.measurement_noise_variance <= 0.0
        || !config.measurement_noise_variance.is_finite()
        || config.covariance_epsilon < 0.0
        || !config.covariance_epsilon.is_finite()
    {
        return Err(IesekfError::InvalidConfig);
    }

    for factor in factors {
        if factor.weight < 0.0
            || !factor.weight.is_finite()
            || !factor.point_i.iter().all(|value| value.is_finite())
            || !factor
                .plane_w
                .normal_w
                .iter()
                .all(|value| value.is_finite())
            || !factor.plane_w.offset.is_finite()
        {
            return Err(IesekfError::NonFiniteInput);
        }
    }

    Ok(())
}

fn build_information_system(
    state: &NavState,
    error_state: &ErrorStateVector,
    factors: &[IesekfPointToPlaneFactor],
    prior_precision: &ErrorStateCovariance,
    config: IesekfConfig,
) -> (ErrorStateCovariance, ErrorStateVector) {
    let mut information = *prior_precision;
    let mut rhs = ErrorStateVector::zeros();

    for factor in factors {
        if factor.weight == 0.0 {
            continue;
        }

        let residual = point_to_plane_residual(state, factor);
        let jacobian = point_to_plane_jacobian(state, factor);
        let precision = factor.weight / config.measurement_noise_variance;
        let linearized_target = jacobian.dot(error_state) - residual;

        information += precision * (jacobian * jacobian.transpose());
        rhs += precision * jacobian * linearized_target;
    }

    (symmetrize(information), rhs)
}

fn total_cost(state: &NavState, factors: &[IesekfPointToPlaneFactor]) -> f64 {
    factors
        .iter()
        .map(|factor| {
            let residual = point_to_plane_residual(state, factor);
            0.5 * factor.weight * residual * residual
        })
        .sum()
}

fn invert_spd(matrix: &ErrorStateCovariance) -> Option<ErrorStateCovariance> {
    matrix.cholesky().map(|cholesky| cholesky.inverse())
}

fn regularize(
    mut covariance: ErrorStateCovariance,
    covariance_epsilon: f64,
) -> ErrorStateCovariance {
    if covariance_epsilon > 0.0 {
        covariance += ErrorStateCovariance::identity() * covariance_epsilon;
    }
    symmetrize(covariance)
}

fn enforce_covariance_floor(
    mut covariance: ErrorStateCovariance,
    covariance_epsilon: f64,
) -> ErrorStateCovariance {
    covariance = symmetrize(covariance);
    for idx in 0..24 {
        if covariance[(idx, idx)] < covariance_epsilon {
            covariance[(idx, idx)] = covariance_epsilon;
        }
    }
    covariance
}

fn symmetrize(matrix: ErrorStateCovariance) -> ErrorStateCovariance {
    (matrix + matrix.transpose()) * 0.5
}

fn matrix_is_finite(matrix: &ErrorStateCovariance) -> bool {
    matrix.iter().all(|value| value.is_finite())
}

fn navstate_is_finite(state: &NavState) -> bool {
    state.position.iter().all(|value| value.is_finite())
        && state.velocity.iter().all(|value| value.is_finite())
        && state.gyro_bias.iter().all(|value| value.is_finite())
        && state.accel_bias.iter().all(|value| value.is_finite())
        && state.gravity.iter().all(|value| value.is_finite())
}

#[cfg(test)]
mod tests {
    use super::*;
    use fastlio_map::PlaneFit;

    fn state(position: Vec3<f64>, orientation: UnitQuaternion<f64>) -> NavState {
        NavState {
            position,
            orientation,
            velocity: Vec3::zeros(),
            gyro_bias: Vec3::zeros(),
            accel_bias: Vec3::zeros(),
            gravity: Vec3::new(0.0, 0.0, -9.81),
        }
    }

    fn plane(normal_w: Vec3<f64>, offset: f64) -> PlaneFit {
        PlaneFit {
            centroid_w: Vec3::zeros(),
            normal_w: normal_w.normalize(),
            offset,
            eigenvalues: Vec3::new(0.0, 1.0, 1.0),
            planarity_ratio: 0.0,
        }
    }

    fn factor(point_i: Vec3<f64>, normal_w: Vec3<f64>, offset: f64) -> IesekfPointToPlaneFactor {
        IesekfPointToPlaneFactor {
            point_i,
            plane_w: plane(normal_w, offset),
            weight: 1.0,
        }
    }

    #[test]
    fn jacobian_matches_right_perturbation_finite_difference() {
        let state = state(
            Vec3::new(0.3, -0.4, 0.2),
            UnitQuaternion::from_scaled_axis(Vec3::new(0.1, -0.2, 0.05)),
        );
        let factor = factor(Vec3::new(1.0, -0.5, 0.25), Vec3::new(0.2, -0.3, 0.9), -0.1);
        let analytical = point_to_plane_jacobian(&state, &factor);
        let eps = 1.0e-6;

        for idx in 0..6 {
            let mut plus = ErrorStateVector::zeros();
            plus[idx] = eps;
            let mut minus = ErrorStateVector::zeros();
            minus[idx] = -eps;

            let residual_plus =
                point_to_plane_residual(&inject_error_state(&state, &plus), &factor);
            let residual_minus =
                point_to_plane_residual(&inject_error_state(&state, &minus), &factor);
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
    fn update_reduces_height_residual_and_covariance() {
        let initial_state = state(Vec3::new(0.0, 0.0, 0.5), UnitQuaternion::identity());
        let covariance = ErrorStateCovariance::identity() * 0.1;
        let mut filter = Iesekf::new(initial_state, covariance).unwrap();
        let factors = vec![
            factor(Vec3::new(-1.0, -1.0, 0.0), Vec3::z_axis().into_inner(), 0.0),
            factor(Vec3::new(1.0, -1.0, 0.0), Vec3::z_axis().into_inner(), 0.0),
            factor(Vec3::new(-1.0, 1.0, 0.0), Vec3::z_axis().into_inner(), 0.0),
            factor(Vec3::new(1.0, 1.0, 0.0), Vec3::z_axis().into_inner(), 0.0),
        ];

        let report = filter
            .update_point_to_plane_iterated(
                &factors,
                IesekfConfig {
                    measurement_noise_variance: 1.0e-4,
                    ..IesekfConfig::default()
                },
            )
            .unwrap();

        assert!(report.final_cost < report.initial_cost);
        assert!(filter.state.position.z.abs() < 1.0e-3);
        assert!(filter.covariance[(5, 5)] < 0.1);
        assert_matrix_symmetric(&filter.covariance);
    }

    #[test]
    fn update_corrects_small_roll() {
        let initial_state = state(
            Vec3::zeros(),
            UnitQuaternion::from_scaled_axis(Vec3::new(0.05, 0.0, 0.0)),
        );
        let covariance = ErrorStateCovariance::identity() * 0.1;
        let mut filter = Iesekf::new(initial_state, covariance).unwrap();
        let factors = vec![
            factor(Vec3::new(-1.0, -1.0, 0.0), Vec3::z_axis().into_inner(), 0.0),
            factor(Vec3::new(1.0, -1.0, 0.0), Vec3::z_axis().into_inner(), 0.0),
            factor(Vec3::new(-1.0, 1.0, 0.0), Vec3::z_axis().into_inner(), 0.0),
            factor(Vec3::new(1.0, 1.0, 0.0), Vec3::z_axis().into_inner(), 0.0),
        ];

        let report = filter
            .update_point_to_plane_iterated(
                &factors,
                IesekfConfig {
                    measurement_noise_variance: 1.0e-4,
                    ..IesekfConfig::default()
                },
            )
            .unwrap();

        assert!(report.final_cost < report.initial_cost);
        assert!(filter.state.orientation.scaled_axis().x.abs() < 1.0e-3);
        assert_matrix_symmetric(&filter.covariance);
    }

    #[test]
    fn zero_weight_measurement_does_not_change_state() {
        let initial_state = state(Vec3::new(0.0, 0.0, 0.5), UnitQuaternion::identity());
        let covariance = ErrorStateCovariance::identity() * 0.1;
        let mut filter = Iesekf::new(initial_state.clone(), covariance).unwrap();
        let mut measurement = factor(Vec3::zeros(), Vec3::z_axis().into_inner(), 0.0);
        measurement.weight = 0.0;

        let report = filter
            .update_point_to_plane_iterated(&[measurement], IesekfConfig::default())
            .unwrap();

        assert!(report.final_cost == report.initial_cost);
        assert!((filter.state.position - initial_state.position).norm() < 1.0e-12);
    }

    #[test]
    fn empty_measurements_are_rejected() {
        let initial_state = state(Vec3::zeros(), UnitQuaternion::identity());
        let covariance = ErrorStateCovariance::identity() * 0.1;
        let mut filter = Iesekf::new(initial_state, covariance).unwrap();

        let err = filter
            .update_point_to_plane_iterated(&[], IesekfConfig::default())
            .unwrap_err();

        assert_eq!(err, IesekfError::EmptyMeasurements);
    }

    fn assert_matrix_symmetric(matrix: &ErrorStateCovariance) {
        let asymmetry = matrix - matrix.transpose();
        assert!(asymmetry.amax() < 1.0e-10);
    }
}
