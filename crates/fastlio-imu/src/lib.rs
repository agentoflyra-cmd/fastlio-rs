use anyhow::Result;
use fastlio_types::{ImuSample, NavState, Vec3};
use nalgebra::UnitQuaternion;

pub type SMat3 = nalgebra::SMatrix<f64, 3, 3>;
pub type SMat12 = nalgebra::SMatrix<f64, 12, 12>;
pub type SMat24 = nalgebra::SMatrix<f64, 24, 24>;
pub type SMat24x12 = nalgebra::SMatrix<f64, 24, 12>;

/// δx = [δθ_I, δp_I, δv_I, δbω, δba, δg, δθ_LI, δp_LI]
pub struct ImuIntegrator {
    /// discrete per-step standard deviation
    pub accel_bias_noise: f64,
    /// discrete per-step standard deviation
    pub accel_noise: f64,
    /// discrete per-step standard deviation
    pub gyro_bias_noise: f64,
    /// discrete per-step standard deviation
    pub gyro_noise: f64,
}

fn checked_dt(imu_prev: &ImuSample, imu_curr: &ImuSample) -> Result<f64> {
    let dt = imu_curr.time_stamp_sec - imu_prev.time_stamp_sec;
    if !dt.is_finite() {
        anyhow::bail!("non finite IMU at dt: {dt}");
    }
    if dt < 0.0 {
        anyhow::bail!(
            "IMU timestamp regressed: prev={}, curr={}",
            imu_prev.time_stamp_sec,
            imu_curr.time_stamp_sec
        );
    }

    Ok(dt)
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

    pub fn propagate_nominal_state_mut(
        &self,
        state: &mut NavState,
        imu_prev: &ImuSample,
        imu_curr: &ImuSample,
    ) -> Result<()> {
        let dt = checked_dt(imu_prev, imu_curr)?;
        if dt <= 1e-7 {
            return Ok(());
        }
        let omega_mid = 0.5 * (imu_prev.gyro + imu_curr.gyro) - state.gyro_bias;
        let acc_mid = 0.5 * (imu_prev.accel + imu_curr.accel) - state.accel_bias;

        let delta_theta = omega_mid * dt;
        let delta_rotation = UnitQuaternion::from_scaled_axis(delta_theta);

        let half_delta_rotation = UnitQuaternion::from_scaled_axis(delta_theta * 0.5);

        let r_mid = state.orientation * half_delta_rotation;
        let r_next = state.orientation * delta_rotation;

        let r_mid_mat = r_mid.to_rotation_matrix();
        let r_mid_matrix = r_mid_mat.matrix();

        let rotated_acc = r_mid_matrix * acc_mid;
        let a_w = rotated_acc + state.gravity;

        let p_next = state.position + state.velocity * dt + a_w * (0.5 * dt.powi(2));
        let v_next = state.velocity + a_w * dt;

        state.position = p_next;
        state.velocity = v_next;
        state.orientation = r_next;
        Ok(())
    }

    pub fn propagate_covariance(
        &self,
        state_at_k: &NavState,
        cov: SMat24,
        imu_prev: &ImuSample,
        imu_curr: &ImuSample,
    ) -> Result<SMat24> {
        let (fx, fw) = self.error_state_transition(state_at_k, imu_prev, imu_curr)?;
        let mut q = SMat12::zeros();

        q.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(SMat3::identity() * self.gyro_noise.powi(2)));

        q.fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(SMat3::identity() * self.accel_noise.powi(2)));

        q.fixed_view_mut::<3, 3>(6, 6)
            .copy_from(&(SMat3::identity() * self.gyro_bias_noise.powi(2)));

        q.fixed_view_mut::<3, 3>(9, 9)
            .copy_from(&(SMat3::identity() * self.accel_bias_noise.powi(2)));

        let cov_next = fx * cov * fx.transpose() + fw * q * fw.transpose();

        Ok(cov_next)
    }

    pub fn error_state_transition(
        &self,
        state_at_k: &NavState,
        imu_prev: &ImuSample,
        imu_curr: &ImuSample,
    ) -> Result<(SMat24, SMat24x12)> {
        let dt = checked_dt(imu_prev, imu_curr)?;
        if dt <= 1e-7 {
            return Ok((SMat24::identity(), SMat24x12::zeros()));
        }
        let dt2 = dt * dt;
        let omega_mid = 0.5 * (imu_prev.gyro + imu_curr.gyro) - state_at_k.gyro_bias;
        let acc_mid = 0.5 * (imu_prev.accel + imu_curr.accel) - state_at_k.accel_bias;

        let delta_theta = omega_mid * dt;
        let delta_rotation = UnitQuaternion::from_scaled_axis(delta_theta);
        let half_delta_rotation = UnitQuaternion::from_scaled_axis(delta_theta * 0.5);

        let r_mid = state_at_k.orientation * half_delta_rotation;
        let r_mid_mat = r_mid.to_rotation_matrix();
        let r_mid_matrix = r_mid_mat.matrix();

        let acc_skew = skew(&acc_mid);
        let accel_orientation_jac = -(r_mid_matrix * acc_skew);

        let jr = so3_right_jacobian(&delta_theta);
        let delta_rotation_inv = delta_rotation.inverse().to_rotation_matrix();
        let ar = delta_rotation_inv.matrix();

        let mut fx = SMat24::identity();
        // R
        fx.fixed_view_mut::<3, 3>(0, 0).copy_from(ar);
        fx.fixed_view_mut::<3, 3>(0, 9).copy_from(&(-jr * dt));

        // p
        fx.fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(accel_orientation_jac * 0.5 * dt2));
        fx.fixed_view_mut::<3, 3>(3, 6)
            .copy_from(&(SMat3::identity() * dt));
        fx.fixed_view_mut::<3, 3>(3, 12)
            .copy_from(&(-(r_mid_matrix * 0.5 * dt2)));
        fx.fixed_view_mut::<3, 3>(3, 15)
            .copy_from(&(&SMat3::identity() * 0.5 * dt2));

        // v
        fx.fixed_view_mut::<3, 3>(6, 0)
            .copy_from(&(accel_orientation_jac * dt));
        fx.fixed_view_mut::<3, 3>(6, 12)
            .copy_from(&(-(r_mid_matrix * dt)));
        fx.fixed_view_mut::<3, 3>(6, 15)
            .copy_from(&(SMat3::identity() * dt));

        let mut fw = SMat24x12::zeros();
        // gyro_bias -> rotation
        fw.fixed_view_mut::<3, 3>(0, 0).copy_from(&(-(jr * dt)));
        // accel_bias -> position
        fw.fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(-(r_mid_matrix * 0.5 * dt2)));
        // accel_bias -> velocity
        fw.fixed_view_mut::<3, 3>(6, 3)
            .copy_from(&(-(r_mid_matrix * dt)));
        // gyro bias random walk
        fw.fixed_view_mut::<3, 3>(9, 6)
            .copy_from(&(SMat3::identity() * dt));
        // accel bias random walk
        fw.fixed_view_mut::<3, 3>(12, 9)
            .copy_from(&(SMat3::identity() * dt));

        Ok((fx, fw))
    }

    // pub fn predict_nomial(
    //     &self,
    //     _state: &mut NavState,
    //     _cov: SMat24,
    //     _imu_prev: &ImuSample,
    //     _imu_curr: &ImuSample,
    // ) -> Result<()> {
    //     todo!()
    // }
}

#[inline]
fn so3_right_jacobian(phi: &Vec3<f64>) -> SMat3 {
    let theta_sq = phi.norm_squared();
    let phi_hat = skew(phi);
    let phi_hat_sq = phi_hat * phi_hat;

    if theta_sq < 1e-8 {
        // Jr(phi) =
        // I - 1/2 phi^ + 1/6 phi^2
        //   + 1/24 phi^3 + ...
        //
        SMat3::identity() - phi_hat * 0.5 + phi_hat_sq * (1.0 / 6.0)
    } else {
        let theta = theta_sq.sqrt();

        let a = (1.0 - theta.cos()) / theta_sq;
        let b = (theta - theta.sin()) / (theta_sq * theta);

        SMat3::identity() - phi_hat * a + phi_hat_sq * b
    }
}

#[inline]
fn skew(v: &Vec3<f64>) -> SMat3 {
    SMat3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

#[cfg(test)]
mod test {
    use crate::ImuIntegrator;
    use crate::{SMat3, SMat24};
    use approx::assert_relative_eq;
    use fastlio_types::{ImuSample, NavState, Vec3};
    use nalgebra::{SVector, UnitQuaternion};

    struct ImuTest {
        imu_prev: ImuSample,
        imu_curr: ImuSample,
        state: NavState,
    }

    impl ImuTest {
        fn init() -> Self {
            let imu_prev = ImuSample {
                time_stamp_sec: 1.0,
                gyro: Vec3::new(0.0, 0.0, -9.81),
                accel: Vec3::new(1.0, 2.0, 1.0),
            };
            let imu_curr = ImuSample {
                time_stamp_sec: 2.0,
                gyro: Vec3::new(0.0, 0.0, 9.81),
                accel: Vec3::new(1.0, 2.0, 1.0),
            };
            let state = NavState {
                position: Vec3::new(0.0, 0.0, 0.0),
                orientation: UnitQuaternion::identity(),
                velocity: Vec3::new(0.0, 0.0, 0.0),
                gyro_bias: Vec3::new(0.0, 0.0, 0.0),
                accel_bias: Vec3::new(0.0, 0.0, 0.0),
                gravity: Vec3::new(0.0, 0.0, 0.0),
            };

            Self {
                imu_prev,
                imu_curr,
                state,
            }
        }
    }

    #[test]
    fn propagate_rejects_negative_dt() {
        let mut test = ImuTest::init();
        test.imu_curr.time_stamp_sec -= 1.1;
        let imu_inte = ImuIntegrator::init(0.0, 0.0, 0.0, 0.0);
        assert!(
            imu_inte
                .propagate_nominal_state_mut(&mut test.state, &test.imu_prev, &test.imu_curr)
                .is_err()
        );
    }

    #[test]
    fn propagate_zero_dt_returns_same_state() {
        let mut test = ImuTest::init();
        let expected = test.state.clone();
        test.imu_curr.time_stamp_sec = 1.0;
        let imu_inte = ImuIntegrator::init(0.0, 0.0, 0.0, 0.0);
        imu_inte
            .propagate_nominal_state_mut(&mut test.state, &test.imu_prev, &test.imu_curr)
            .unwrap();
        assert!(test.state == expected);
    }

    #[test]
    fn stationary_imu_keeps_pose_with_gravity_cancelled() {
        let mut test = ImuTest::init();
        test.imu_prev.accel = Vec3::new(0.0, 0.0, 9.81);
        test.imu_curr.accel = Vec3::new(0.0, 0.0, 9.81);
        let imu_inte = ImuIntegrator::init(0.0, 0.0, 0.0, 0.0);
        let expected = test.state.clone();
        imu_inte
            .propagate_nominal_state_mut(&mut test.state, &test.imu_prev, &test.imu_curr)
            .unwrap();
        assert!(test.state.orientation.angle_to(&expected.orientation) < 1e-10);
    }

    #[test]
    fn constant_angular_velocity_rotates_orientation() {
        let mut test = ImuTest::init();
        test.imu_prev.gyro = Vec3::new(0.0, 0.0, 1.0);
        test.imu_prev.accel = -test.state.gravity;

        let matrix = Vec3::new(0.0, 0.0, 1.0);
        test.imu_curr.gyro = matrix;
        test.imu_curr.accel = -test.state.gravity;
        let imu_inte = ImuIntegrator::init(0.0, 0.0, 0.0, 0.0);
        imu_inte
            .propagate_nominal_state_mut(&mut test.state, &test.imu_prev, &test.imu_curr)
            .unwrap();
        let expected = UnitQuaternion::from_scaled_axis(
            matrix * (test.imu_curr.time_stamp_sec - test.imu_prev.time_stamp_sec),
        );
        assert!(test.state.orientation.rotation_to(&expected).angle() < 1e-10)
    }

    #[test]
    fn constand_world_acceleration_updates_velocity_and_position() {
        let mut test = ImuTest::init();
        let expected = test.state.velocity
            + Vec3::new(0.0, 1.0, 1.0)
                * (test.imu_curr.time_stamp_sec - test.imu_prev.time_stamp_sec);
        test.imu_prev.accel = Vec3::new(0.0, 1.0, 1.0);
        test.imu_curr.accel = Vec3::new(0.0, 1.0, 1.0);

        let imu_inte = ImuIntegrator::init(0.0, 0.0, 0.0, 0.0);
        imu_inte
            .propagate_nominal_state_mut(&mut test.state, &test.imu_prev, &test.imu_curr)
            .unwrap();
        assert_relative_eq!(test.state.velocity, expected, epsilon = 1e-10);
    }

    // --- error_state_transition block tests ---

    #[test]
    fn error_state_zero_dt_returns_identity_fx_zero_fw() {
        let test = ImuTest::init();
        let integ = ImuIntegrator::init(0.01, 0.01, 0.001, 0.001);
        let (fx, fw) = integ
            .error_state_transition(&test.state, &test.imu_prev, &test.imu_prev)
            .unwrap();
        for r in 0..24 {
            for c in 0..24 {
                assert_relative_eq!(fx[(r, c)], if r == c { 1.0 } else { 0.0 }, epsilon = 1e-14);
            }
        }
        for r in 0..24 {
            for c in 0..12 {
                assert_relative_eq!(fw[(r, c)], 0.0, epsilon = 1e-14);
            }
        }
    }

    #[test]
    fn error_state_fx_rotation_block_is_inverse_of_delta_rotation() {
        let mut test = ImuTest::init();
        test.imu_prev.gyro = Vec3::new(0.1, 0.2, 0.3);
        test.imu_curr.gyro = Vec3::new(0.1, 0.2, 0.3);
        let integ = ImuIntegrator::init(0.0, 0.0, 0.0, 0.0);
        let (fx, _) = integ
            .error_state_transition(&test.state, &test.imu_prev, &test.imu_curr)
            .unwrap();

        let dt = test.imu_curr.time_stamp_sec - test.imu_prev.time_stamp_sec;
        let omega_mid = 0.5 * (test.imu_prev.gyro + test.imu_curr.gyro) - test.state.gyro_bias;
        let delta_rot = UnitQuaternion::from_scaled_axis(omega_mid * dt);
        let expected_ar = delta_rot.inverse().to_rotation_matrix();

        let fx_rot = fx.fixed_view::<3, 3>(0, 0);
        for r in 0..3 {
            for c in 0..3 {
                assert_relative_eq!(
                    fx_rot[(r, c)],
                    expected_ar.matrix()[(r, c)],
                    epsilon = 1e-10
                );
            }
        }
    }

    #[test]
    fn error_state_fx_rotation_to_gyro_bias_block_is_neg_jr_dt() {
        let mut test = ImuTest::init();
        test.imu_prev.gyro = Vec3::new(0.1, 0.2, 0.3);
        test.imu_curr.gyro = Vec3::new(0.1, 0.2, 0.3);
        let integ = ImuIntegrator::init(0.0, 0.0, 0.0, 0.0);
        let (fx, _) = integ
            .error_state_transition(&test.state, &test.imu_prev, &test.imu_curr)
            .unwrap();

        let dt = test.imu_curr.time_stamp_sec - test.imu_prev.time_stamp_sec;
        let omega_mid = 0.5 * (test.imu_prev.gyro + test.imu_curr.gyro) - test.state.gyro_bias;
        let delta_theta = omega_mid * dt;
        let jr = crate::so3_right_jacobian(&delta_theta);
        let expected = -jr * dt;

        let fx_r_bg = fx.fixed_view::<3, 3>(0, 9);
        for r in 0..3 {
            for c in 0..3 {
                assert_relative_eq!(fx_r_bg[(r, c)], expected[(r, c)], epsilon = 1e-10);
            }
        }
    }

    #[test]
    fn error_state_fx_position_to_velocity_block_is_i_dt() {
        let test = ImuTest::init();
        let dt = test.imu_curr.time_stamp_sec - test.imu_prev.time_stamp_sec;
        let integ = ImuIntegrator::init(0.0, 0.0, 0.0, 0.0);
        let (fx, _) = integ
            .error_state_transition(&test.state, &test.imu_prev, &test.imu_curr)
            .unwrap();

        let fx_p_v = fx.fixed_view::<3, 3>(3, 6);
        let expected = SMat3::identity() * dt;
        for r in 0..3 {
            for c in 0..3 {
                assert_relative_eq!(fx_p_v[(r, c)], expected[(r, c)], epsilon = 1e-10);
            }
        }
    }

    #[test]
    fn error_state_fx_velocity_to_gravity_block_is_i_dt() {
        let test = ImuTest::init();
        let dt = test.imu_curr.time_stamp_sec - test.imu_prev.time_stamp_sec;
        let integ = ImuIntegrator::init(0.0, 0.0, 0.0, 0.0);
        let (fx, _) = integ
            .error_state_transition(&test.state, &test.imu_prev, &test.imu_curr)
            .unwrap();

        let fx_v_g = fx.fixed_view::<3, 3>(6, 15);
        let expected = SMat3::identity() * dt;
        for r in 0..3 {
            for c in 0..3 {
                assert_relative_eq!(fx_v_g[(r, c)], expected[(r, c)], epsilon = 1e-10);
            }
        }
    }

    #[test]
    fn error_state_fx_unmodified_blocks_remain_identity() {
        let mut test = ImuTest::init();
        test.imu_prev.gyro = Vec3::new(0.1, 0.2, 0.3);
        test.imu_curr.gyro = Vec3::new(0.1, 0.2, 0.3);
        test.imu_prev.accel = Vec3::new(1.0, -2.0, 3.0);
        test.imu_curr.accel = Vec3::new(1.0, -2.0, 3.0);
        let integ = ImuIntegrator::init(0.0, 0.0, 0.0, 0.0);
        let (fx, _) = integ
            .error_state_transition(&test.state, &test.imu_prev, &test.imu_curr)
            .unwrap();

        // Blocks that should have been left as identity (never overwritten):
        // bg -> bg (9..12, 9..12), ba -> ba (12..15, 12..15),
        // g -> g (15..18, 15..18), unused diagonal (18..24, 18..24)
        let identity_blocks: &[(usize, usize)] = &[
            (9, 9),   // gyro_bias -> gyro_bias
            (12, 12), // accel_bias -> accel_bias
            (15, 15), // gravity -> gravity
            (18, 18), // unused
        ];
        for &(row, col) in identity_blocks {
            let block = fx.fixed_view::<3, 3>(row, col);
            for r in 0..3 {
                for c in 0..3 {
                    assert_relative_eq!(
                        block[(r, c)],
                        if r == c { 1.0 } else { 0.0 },
                        epsilon = 1e-10,
                    );
                }
            }
        }
    }

    // --- error_state_transition F_w block tests ---

    #[test]
    fn error_state_fw_rotation_to_gyro_noise_block_is_neg_jr_dt() {
        let mut test = ImuTest::init();
        test.imu_prev.gyro = Vec3::new(0.1, 0.2, 0.3);
        test.imu_curr.gyro = Vec3::new(0.1, 0.2, 0.3);
        let integ = ImuIntegrator::init(0.0, 0.0, 0.0, 0.0);
        let (_, fw) = integ
            .error_state_transition(&test.state, &test.imu_prev, &test.imu_curr)
            .unwrap();

        let dt = test.imu_curr.time_stamp_sec - test.imu_prev.time_stamp_sec;
        let omega_mid = 0.5 * (test.imu_prev.gyro + test.imu_curr.gyro) - test.state.gyro_bias;
        let delta_theta = omega_mid * dt;
        let jr = crate::so3_right_jacobian(&delta_theta);
        let expected = -jr * dt;

        let fw_r_ng = fw.fixed_view::<3, 3>(0, 0);
        for r in 0..3 {
            for c in 0..3 {
                assert_relative_eq!(fw_r_ng[(r, c)], expected[(r, c)], epsilon = 1e-10);
            }
        }
    }

    #[test]
    fn error_state_fw_bias_noise_blocks_are_i_dt() {
        let test = ImuTest::init();
        let dt = test.imu_curr.time_stamp_sec - test.imu_prev.time_stamp_sec;
        let integ = ImuIntegrator::init(0.0, 0.0, 0.0, 0.0);
        let (_, fw) = integ
            .error_state_transition(&test.state, &test.imu_prev, &test.imu_curr)
            .unwrap();

        let expected = SMat3::identity() * dt;

        let fw_bg_noise = fw.fixed_view::<3, 3>(9, 6);
        for r in 0..3 {
            for c in 0..3 {
                assert_relative_eq!(fw_bg_noise[(r, c)], expected[(r, c)], epsilon = 1e-10);
            }
        }

        let fw_ba_noise = fw.fixed_view::<3, 3>(12, 9);
        for r in 0..3 {
            for c in 0..3 {
                assert_relative_eq!(fw_ba_noise[(r, c)], expected[(r, c)], epsilon = 1e-10);
            }
        }
    }

    // --- propagate_covariance tests ---

    #[test]
    fn propagate_covariance_result_is_symmetric() {
        let mut test = ImuTest::init();
        test.imu_prev.gyro = Vec3::new(0.05, -0.03, 0.07);
        test.imu_curr.gyro = Vec3::new(0.05, -0.03, 0.07);
        test.imu_prev.accel = Vec3::new(1.5, 0.3, -2.0);
        test.imu_curr.accel = Vec3::new(1.5, 0.3, -2.0);
        let integ = ImuIntegrator::init(0.01, 0.05, 0.001, 0.002);

        let mut cov = SMat24::zeros();
        for i in 0..18 {
            cov[(i, i)] = (i as f64 + 1.0) * 0.001;
        }
        cov[(0, 3)] = 0.0001;
        cov[(3, 0)] = 0.0001;
        cov[(6, 12)] = -0.00005;
        cov[(12, 6)] = -0.00005;

        let result = integ
            .propagate_covariance(&test.state, cov, &test.imu_prev, &test.imu_curr)
            .unwrap();

        for r in 0..24 {
            for c in 0..24 {
                assert_relative_eq!(result[(r, c)], result[(c, r)], epsilon = 1e-12);
            }
        }
    }

    #[test]
    fn propagate_covariance_zero_noise_matches_fx_cov_fxt() {
        let mut test = ImuTest::init();
        test.imu_prev.gyro = Vec3::new(0.05, -0.03, 0.07);
        test.imu_curr.gyro = Vec3::new(0.05, -0.03, 0.07);
        test.imu_prev.accel = Vec3::new(1.5, 0.3, -2.0);
        test.imu_curr.accel = Vec3::new(1.5, 0.3, -2.0);
        test.state.orientation = UnitQuaternion::from_scaled_axis(Vec3::new(0.1, -0.2, 0.15));
        let integ = ImuIntegrator::init(0.0, 0.0, 0.0, 0.0);

        let mut cov = SMat24::zeros();
        for i in 0..18 {
            cov[(i, i)] = (i as f64 + 1.0) * 0.001;
        }
        cov[(1, 7)] = 0.0003;
        cov[(7, 1)] = 0.0003;

        let (fx, _) = integ
            .error_state_transition(&test.state, &test.imu_prev, &test.imu_curr)
            .unwrap();
        let expected = fx * cov * fx.transpose();
        let actual = integ
            .propagate_covariance(&test.state, cov, &test.imu_prev, &test.imu_curr)
            .unwrap();

        for r in 0..24 {
            for c in 0..24 {
                assert_relative_eq!(actual[(r, c)], expected[(r, c)], epsilon = 1e-12);
            }
        }
    }

    #[test]
    fn propagate_covariance_zero_dt_preserves_covariance() {
        let test = ImuTest::init();
        let integ = ImuIntegrator::init(0.01, 0.05, 0.001, 0.002);

        let mut cov = SMat24::zeros();
        for i in 0..18 {
            cov[(i, i)] = (i as f64 + 1.0) * 0.001;
        }
        cov[(1, 7)] = 0.0003;
        cov[(7, 1)] = 0.0003;

        let result = integ
            .propagate_covariance(&test.state, cov, &test.imu_prev, &test.imu_prev)
            .unwrap();

        for r in 0..24 {
            for c in 0..24 {
                assert_relative_eq!(result[(r, c)], cov[(r, c)], epsilon = 1e-14);
            }
        }
    }

    // --- finite-difference Jacobian verification ---

    type S18 = SVector<f64, 18>;

    fn skew3(v: &Vec3<f64>) -> SMat3 {
        crate::skew(v)
    }

    fn inject_error_state(nominal: &NavState, dx: &S18) -> NavState {
        let delta_theta: Vec3<f64> = dx.fixed_rows::<3>(0).into_owned();
        let delta_rot = UnitQuaternion::from_scaled_axis(delta_theta);
        NavState {
            orientation: nominal.orientation * delta_rot,
            position: nominal.position + dx.fixed_rows::<3>(3).into_owned(),
            velocity: nominal.velocity + dx.fixed_rows::<3>(6).into_owned(),
            gyro_bias: nominal.gyro_bias + dx.fixed_rows::<3>(9).into_owned(),
            accel_bias: nominal.accel_bias + dx.fixed_rows::<3>(12).into_owned(),
            gravity: nominal.gravity + dx.fixed_rows::<3>(15).into_owned(),
        }
    }

    fn extract_error_state(true_state: &NavState, nominal: &NavState) -> S18 {
        let delta_rot = nominal.orientation.inverse() * true_state.orientation;
        let delta_theta = delta_rot.scaled_axis();

        let mut err = S18::zeros();
        err.fixed_rows_mut::<3>(0).copy_from(&delta_theta);
        err.fixed_rows_mut::<3>(3)
            .copy_from(&(true_state.position - nominal.position));
        err.fixed_rows_mut::<3>(6)
            .copy_from(&(true_state.velocity - nominal.velocity));
        err.fixed_rows_mut::<3>(9)
            .copy_from(&(true_state.gyro_bias - nominal.gyro_bias));
        err.fixed_rows_mut::<3>(12)
            .copy_from(&(true_state.accel_bias - nominal.accel_bias));
        err.fixed_rows_mut::<3>(15)
            .copy_from(&(true_state.gravity - nominal.gravity));
        err
    }

    #[test]
    fn error_state_fx_blocks_match_finite_difference() {
        let mut test = ImuTest::init();
        test.state.orientation = UnitQuaternion::from_scaled_axis(Vec3::new(0.15, -0.25, 0.18));
        test.state.gravity = Vec3::new(0.0, 0.0, -9.81);
        test.state.gyro_bias = Vec3::new(0.01, -0.02, 0.005);
        test.state.accel_bias = Vec3::new(0.05, 0.03, -0.02);
        test.imu_prev.time_stamp_sec = 1.0;
        test.imu_curr.time_stamp_sec = 1.05;
        test.imu_prev.gyro = Vec3::new(0.05, -0.03, 0.07);
        test.imu_curr.gyro = Vec3::new(0.06, -0.04, 0.08);
        test.imu_prev.accel = Vec3::new(1.5, 0.3, -2.0);
        test.imu_curr.accel = Vec3::new(1.3, 0.2, 1.0);

        let dt = test.imu_curr.time_stamp_sec - test.imu_prev.time_stamp_sec;
        let integ = ImuIntegrator::init(0.0, 0.0, 0.0, 0.0);
        let (fx, _) = integ
            .error_state_transition(&test.state, &test.imu_prev, &test.imu_curr)
            .unwrap();

        let mut nominal_forward = test.state.clone();
        integ
            .propagate_nominal_state_mut(&mut nominal_forward, &test.imu_prev, &test.imu_curr)
            .unwrap();

        let h = 1e-5;
        let cmp_eps = 1e-4;

        // Test velocity-to-position block (3..6, 6..9) = I * dt
        {
            let mut dx = S18::zeros();
            dx[6] = h; // perturb v_x
            let perturbed = inject_error_state(&test.state, &dx);
            let mut fwd = perturbed.clone();
            integ
                .propagate_nominal_state_mut(&mut fwd, &test.imu_prev, &test.imu_curr)
                .unwrap();
            let err = extract_error_state(&fwd, &nominal_forward);
            let block = fx.fixed_view::<3, 3>(3, 6);
            assert_relative_eq!(err[3], block[(0, 0)] * h, epsilon = cmp_eps);
            assert_relative_eq!(err[4], block[(1, 0)] * h, epsilon = cmp_eps);
            assert_relative_eq!(err[5], block[(2, 0)] * h, epsilon = cmp_eps);
        }

        // Test accel_bias-to-velocity block (6..9, 12..15) = -R_mid * dt
        {
            let mut dx = S18::zeros();
            dx[12] = h; // perturb ba_x
            let perturbed = inject_error_state(&test.state, &dx);
            let mut fwd = perturbed.clone();
            integ
                .propagate_nominal_state_mut(&mut fwd, &test.imu_prev, &test.imu_curr)
                .unwrap();
            let err = extract_error_state(&fwd, &nominal_forward);
            let block = fx.fixed_view::<3, 3>(6, 12);

            let omega_mid = 0.5 * (test.imu_prev.gyro + test.imu_curr.gyro) - test.state.gyro_bias;
            let half_delta = UnitQuaternion::from_scaled_axis(omega_mid * dt * 0.5);
            let r_mid_rot = (test.state.orientation * half_delta).to_rotation_matrix();
            let r_mid_mat = r_mid_rot.matrix();
            let expected_3x3 = -(r_mid_mat * dt);

            assert_relative_eq!(err[6], expected_3x3[(0, 0)] * h, epsilon = cmp_eps);
            assert_relative_eq!(err[7], expected_3x3[(1, 0)] * h, epsilon = cmp_eps);
            assert_relative_eq!(err[8], expected_3x3[(2, 0)] * h, epsilon = cmp_eps);
            assert_relative_eq!(block[(0, 0)], expected_3x3[(0, 0)], epsilon = 1e-10);
            assert_relative_eq!(block[(1, 0)], expected_3x3[(1, 0)], epsilon = 1e-10);
            assert_relative_eq!(block[(2, 0)], expected_3x3[(2, 0)], epsilon = 1e-10);
        }

        // Test gyro_bias-to-rotation block (0..3, 9..12) = -Jr * dt
        {
            let mut dx = S18::zeros();
            dx[9] = h; // perturb bg_x
            let perturbed = inject_error_state(&test.state, &dx);
            let mut fwd = perturbed.clone();
            integ
                .propagate_nominal_state_mut(&mut fwd, &test.imu_prev, &test.imu_curr)
                .unwrap();
            let err = extract_error_state(&fwd, &nominal_forward);
            let block = fx.fixed_view::<3, 3>(0, 9);

            let omega_mid = 0.5 * (test.imu_prev.gyro + test.imu_curr.gyro) - test.state.gyro_bias;
            let delta_theta = omega_mid * dt;
            let jr = crate::so3_right_jacobian(&delta_theta);
            let expected_3x3 = -(jr * dt);

            assert_relative_eq!(err[0], expected_3x3[(0, 0)] * h, epsilon = cmp_eps);
            assert_relative_eq!(err[1], expected_3x3[(1, 0)] * h, epsilon = cmp_eps);
            assert_relative_eq!(err[2], expected_3x3[(2, 0)] * h, epsilon = cmp_eps);
            assert_relative_eq!(block[(0, 0)], expected_3x3[(0, 0)], epsilon = 1e-10);
            assert_relative_eq!(block[(1, 0)], expected_3x3[(1, 0)], epsilon = 1e-10);
            assert_relative_eq!(block[(2, 0)], expected_3x3[(2, 0)], epsilon = 1e-10);
        }

        // Test rotation-to-position block (3..6, 0..3):
        // accel_orientation_jac * 0.5 * dt^2 where accel_orientation_jac = -R_mid * skew(acc_mid)
        {
            let mut dx = S18::zeros();
            dx[1] = h; // perturb delta_theta_y
            let perturbed = inject_error_state(&test.state, &dx);
            let mut fwd = perturbed.clone();
            integ
                .propagate_nominal_state_mut(&mut fwd, &test.imu_prev, &test.imu_curr)
                .unwrap();
            let _err = extract_error_state(&fwd, &nominal_forward);
            let block = fx.fixed_view::<3, 3>(3, 0);

            let acc_mid = 0.5 * (test.imu_prev.accel + test.imu_curr.accel) - test.state.accel_bias;
            let omega_mid = 0.5 * (test.imu_prev.gyro + test.imu_curr.gyro) - test.state.gyro_bias;
            let half_delta = UnitQuaternion::from_scaled_axis(omega_mid * dt * 0.5);
            let r_mid_rot = (test.state.orientation * half_delta).to_rotation_matrix();
            let r_mid_mat = r_mid_rot.matrix();
            let accel_orientation_jac = -(r_mid_mat * skew3(&acc_mid));
            let expected_3x3 = accel_orientation_jac * 0.5 * dt * dt;

            // Verify analytical block
            assert_relative_eq!(block[(0, 1)], expected_3x3[(0, 1)], epsilon = 1e-10);
            assert_relative_eq!(block[(1, 1)], expected_3x3[(1, 1)], epsilon = 1e-10);
            assert_relative_eq!(block[(2, 1)], expected_3x3[(2, 1)], epsilon = 1e-10);
        }
    }

    #[test]
    fn error_state_fw_measurement_noise_matches_finite_difference() {
        let mut test = ImuTest::init();
        test.state.orientation = UnitQuaternion::from_scaled_axis(Vec3::new(0.15, -0.25, 0.18));
        test.state.gravity = Vec3::new(0.0, 0.0, -9.81);
        test.state.gyro_bias = Vec3::new(0.01, -0.02, 0.005);
        test.state.accel_bias = Vec3::new(0.05, 0.03, -0.02);
        test.imu_prev.time_stamp_sec = 1.0;
        test.imu_curr.time_stamp_sec = 1.05;
        test.imu_prev.gyro = Vec3::new(0.05, -0.03, 0.07);
        test.imu_curr.gyro = Vec3::new(0.06, -0.04, 0.08);
        test.imu_prev.accel = Vec3::new(1.5, 0.3, -2.0);
        test.imu_curr.accel = Vec3::new(1.3, 0.2, 1.0);

        let dt = test.imu_curr.time_stamp_sec - test.imu_prev.time_stamp_sec;
        let integ = ImuIntegrator::init(0.0, 0.0, 0.0, 0.0);

        // F_w maps noise to error state at k+1 given dx_k = 0.
        // The nominal overshoots: w \ne 0 makes omega_nom > omega_true,
        // producing a negative error delta_theta.
        // Test: clean IMU = "true" physics, noisy IMU = "measured" = clean + n_g
        // Nominal propagation uses measured (noisy) IMU → overshoots
        // True propagation uses clean IMU → correct
        // Error = extract(nominal_forward, true_forward)? No:
        // Error state = true - nominal ≈ F_w * w

        let h = 1e-4;
        let cmp_eps = 5e-5;

        // --- gyro noise (n_g) ---
        {
            let n_g = Vec3::new(h, 0.0, 0.0);
            let imu_meas_prev = ImuSample {
                gyro: test.imu_prev.gyro + n_g,
                accel: test.imu_prev.accel,
                time_stamp_sec: test.imu_prev.time_stamp_sec,
            };
            let imu_meas_curr = ImuSample {
                gyro: test.imu_curr.gyro + n_g,
                accel: test.imu_curr.accel,
                time_stamp_sec: test.imu_curr.time_stamp_sec,
            };

            let (_, fw) = integ
                .error_state_transition(&test.state, &imu_meas_prev, &imu_meas_curr)
                .unwrap();

            // nominal = propagated with noisy IMU
            let mut nom_fwd = test.state.clone();
            integ
                .propagate_nominal_state_mut(&mut nom_fwd, &imu_meas_prev, &imu_meas_curr)
                .unwrap();

            // true = propagated with clean IMU
            let mut true_fwd = test.state.clone();
            integ
                .propagate_nominal_state_mut(&mut true_fwd, &test.imu_prev, &test.imu_curr)
                .unwrap();

            let err = extract_error_state(&true_fwd, &nom_fwd);

            let omega_mid_nom =
                0.5 * (imu_meas_prev.gyro + imu_meas_curr.gyro) - test.state.gyro_bias;
            let dtheta_nom = omega_mid_nom * dt;
            let jr_nom = crate::so3_right_jacobian(&dtheta_nom);
            let expected_fw_r_ng = -(jr_nom * dt);

            for r in 0..3 {
                let analytical = expected_fw_r_ng[(r, 0)];
                assert_relative_eq!(
                    err[r],
                    analytical * h,
                    epsilon = cmp_eps,
                    max_relative = 1e-4,
                );
                assert_relative_eq!(fw[(r, 0)], expected_fw_r_ng[(r, 0)], epsilon = 1e-10,);
            }
        }

        // --- accel noise (n_a): position and velocity error ---
        {
            let n_a = Vec3::new(0.0, h, 0.0);
            let imu_meas_prev = ImuSample {
                gyro: test.imu_prev.gyro,
                accel: test.imu_prev.accel + n_a,
                time_stamp_sec: test.imu_prev.time_stamp_sec,
            };
            let imu_meas_curr = ImuSample {
                gyro: test.imu_curr.gyro,
                accel: test.imu_curr.accel + n_a,
                time_stamp_sec: test.imu_curr.time_stamp_sec,
            };

            let (_, fw) = integ
                .error_state_transition(&test.state, &imu_meas_prev, &imu_meas_curr)
                .unwrap();

            let mut nom_fwd = test.state.clone();
            integ
                .propagate_nominal_state_mut(&mut nom_fwd, &imu_meas_prev, &imu_meas_curr)
                .unwrap();

            let mut true_fwd = test.state.clone();
            integ
                .propagate_nominal_state_mut(&mut true_fwd, &test.imu_prev, &test.imu_curr)
                .unwrap();

            let err = extract_error_state(&true_fwd, &nom_fwd);

            let omega_mid_nom =
                0.5 * (imu_meas_prev.gyro + imu_meas_curr.gyro) - test.state.gyro_bias;
            let half_delta = UnitQuaternion::from_scaled_axis(omega_mid_nom * dt * 0.5);
            let r_mid_rot = (test.state.orientation * half_delta).to_rotation_matrix();
            let r_mid_mat = r_mid_rot.matrix();

            let expected_fw_p_na = -(r_mid_mat * 0.5 * dt * dt);
            let expected_fw_v_na = -(r_mid_mat * dt);

            let fw_p_na = fw.fixed_view::<3, 3>(3, 3); // position-to-accel_noise
            let fw_v_na = fw.fixed_view::<3, 3>(6, 3); // velocity-to-accel_noise

            for r in 0..3 {
                assert_relative_eq!(err[3 + r], expected_fw_p_na[(r, 1)] * h, epsilon = cmp_eps);
                assert_relative_eq!(err[6 + r], expected_fw_v_na[(r, 1)] * h, epsilon = cmp_eps);
                assert_relative_eq!(fw_p_na[(r, 1)], expected_fw_p_na[(r, 1)], epsilon = 1e-10,);
                assert_relative_eq!(fw_v_na[(r, 1)], expected_fw_v_na[(r, 1)], epsilon = 1e-10,);
            }
        }
    }

    #[test]
    fn error_state_transition_negative_dt_returns_err() {
        let mut test = ImuTest::init();
        test.imu_curr.time_stamp_sec = 0.5;
        let integ = ImuIntegrator::init(0.01, 0.01, 0.001, 0.001);
        assert!(
            integ
                .error_state_transition(&test.state, &test.imu_prev, &test.imu_curr)
                .is_err()
        );
    }

    #[test]
    fn propagate_covariance_negative_dt_returns_err() {
        let mut test = ImuTest::init();
        test.imu_curr.time_stamp_sec = 0.5;
        let integ = ImuIntegrator::init(0.01, 0.01, 0.001, 0.001);
        assert!(
            integ
                .propagate_covariance(
                    &test.state,
                    SMat24::identity(),
                    &test.imu_prev,
                    &test.imu_curr
                )
                .is_err()
        );
    }

    #[test]
    fn propagate_covariance_positive_noise_increases_diagonal() {
        let mut test = ImuTest::init();
        test.imu_prev.gyro = Vec3::new(0.05, -0.03, 0.07);
        test.imu_curr.gyro = Vec3::new(0.06, -0.04, 0.08);
        test.imu_prev.accel = Vec3::new(1.5, 0.3, -2.0);
        test.imu_curr.accel = Vec3::new(1.3, 0.2, 1.0);

        let cov_in = SMat24::identity() * 0.01;
        let integ = ImuIntegrator::init(0.1, 0.2, 0.03, 0.04);
        let cov_out = integ
            .propagate_covariance(&test.state, cov_in, &test.imu_prev, &test.imu_curr)
            .unwrap();

        let diag_sum_in: f64 = (0..24).map(|i| cov_in[(i, i)]).sum();
        let diag_sum_out: f64 = (0..24).map(|i| cov_out[(i, i)]).sum();
        assert!(diag_sum_out > diag_sum_in);
    }

    #[test]
    fn extrinsic_error_blocks_remain_identity_under_imu_prediction() {
        let mut test = ImuTest::init();
        test.imu_prev.gyro = Vec3::new(0.05, -0.03, 0.07);
        test.imu_curr.gyro = Vec3::new(0.06, -0.04, 0.08);
        test.imu_prev.accel = Vec3::new(1.5, 0.3, -2.0);
        test.imu_curr.accel = Vec3::new(1.3, 0.2, 1.0);

        let integ = ImuIntegrator::init(0.01, 0.01, 0.001, 0.001);
        let (fx, fw) = integ
            .error_state_transition(&test.state, &test.imu_prev, &test.imu_curr)
            .unwrap();

        // F_x extrinsic block (18..24, 18..24) should be identity
        let fx_ext = fx.fixed_view::<6, 6>(18, 18);
        for r in 0..6 {
            for c in 0..6 {
                assert_relative_eq!(
                    fx_ext[(r, c)],
                    if r == c { 1.0 } else { 0.0 },
                    epsilon = 1e-14,
                );
            }
        }

        // F_x cross-terms with extrinsic (18..24, 0..18) should be zero
        for row in 18..24 {
            for col in 0..18 {
                assert_relative_eq!(fx[(row, col)], 0.0, epsilon = 1e-14);
            }
        }

        // F_x cross-terms with extrinsic (0..18, 18..24) should be zero
        for row in 0..18 {
            for col in 18..24 {
                assert_relative_eq!(fx[(row, col)], 0.0, epsilon = 1e-14);
            }
        }

        // F_w should have no noise injection into extrinsic blocks
        for row in 18..24 {
            for col in 0..12 {
                assert_relative_eq!(fw[(row, col)], 0.0, epsilon = 1e-14);
            }
        }

        // Covariance propagation: extrinsic block should be unchanged
        // when there is no cross-covariance with the core state
        let mut cov_in = SMat24::zeros();
        for i in 0..18 {
            cov_in[(i, i)] = 0.01;
        }
        for i in 18..24 {
            cov_in[(i, i)] = 0.05;
        }
        let cov_out = integ
            .propagate_covariance(&test.state, cov_in, &test.imu_prev, &test.imu_curr)
            .unwrap();

        let cov_ext_out = cov_out.fixed_view::<6, 6>(18, 18);
        let cov_ext_in = cov_in.fixed_view::<6, 6>(18, 18);
        for r in 0..6 {
            for c in 0..6 {
                assert_relative_eq!(cov_ext_out[(r, c)], cov_ext_in[(r, c)], epsilon = 1e-14);
            }
        }
    }
}
