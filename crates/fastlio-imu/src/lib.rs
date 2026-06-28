use fastlio_types::{ImuSample, NavState};
use nalgebra::SMatrix;

pub type SMatrix15<T> = SMatrix<T, 15, 15>;
pub type SMatrix15x12<T> = SMatrix<T, 15, 12>;

pub struct ImuIntegrator {
    pub gyro_noise: f64,
    pub accel_noise: f64,
    pub gyro_bias_noise: f64,
    pub accel_bias_noise: f64,
}

impl ImuIntegrator {
    /// use for init by config
    pub fn init(
        gyro_noise: f64,
        accel_noise: f64,
        gyro_bias_noise: f64,
        accel_bias_noise: f64,
    ) -> Self {
        Self {
            gyro_noise,
            accel_noise,
            gyro_bias_noise,
            accel_bias_noise,
        }
    }

    pub fn propagate_convariance(
        &self,
        _state: &NavState,
        _cov: SMatrix15<f64>,
        _imu_prev: &ImuSample,
        _imu_curr: &ImuSample,
    ) -> SMatrix15<f64> {
        todo!()
    }

    pub fn error_state_transition(
        &self,
        _state: &NavState,
        _imu_prev: &ImuSample,
        _imu_curr: &ImuSample,
    ) -> (SMatrix15<f64>, SMatrix15x12<f64>) {
        todo!()
    }

    pub fn predict_nomial(
        &self,
        _state: &NavState,
        _imu_prev: &ImuSample,
        _imu_curr: &ImuSample,
    ) -> NavState {
        todo!()
    }
}
