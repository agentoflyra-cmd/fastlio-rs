use anyhow::{Result, anyhow};
use fastlio_estimator::{iekf::IekfState, optimizer::IekfConfig};
use fastlio_imu::ImuIntegrator;
use fastlio_map::surfel::SurfelMap;
use fastlio_pointcloud::preprocess::preprocess;
use fastlio_types::{Config, ImuSample, LidarImuExtrinsic, MeasureGroup, PointXYZI, Vec3};
use nalgebra::{Rotation3, UnitQuaternion};

use crate::deskew::{build_motion_segments, deskew};

pub mod deskew;
pub mod synchronizer;
pub mod trajectory;

#[derive(Default)]
pub struct ImuInitializer {
    pub mean_gravity: Vec3<f64>,
    pub mean_accel: Vec3<f64>,
    pub group_count: usize,
    pub sample_count: usize,
    pub cov_accel: Vec3<f64>,
}

#[derive(Debug)]
struct ImuInitialization {
    gravity: Vec3<f64>,
    /// `R_WI` aligning the world +Z (anti-gravity) axis with the mean measured
    /// acceleration direction in the IMU frame. Without this the initial
    /// orientation is identity and gravity is misaligned with the body, so the
    /// prediction accumulates a residual vertical acceleration and `z` drifts
    /// quadratically once LiDAR constraints weaken.
    orientation: UnitQuaternion<f64>,
    /// True when gravity alignment ran (static platform). When false the mean
    /// acceleration is contaminated by linear motion and the identity
    /// orientation / default gravity are kept, exactly as FAST-LIO skips
    /// alignment under the same condition.
    gravity_aligned: bool,
    gyro_bias: Vec3<f64>,
    accel_scale: f64,
}

impl ImuInitializer {
    pub fn accumulate(&mut self, imu_samples: &[ImuSample]) {
        self.group_count += 1;
        imu_samples.iter().for_each(|imu_sample| {
            self.sample_count += 1;
            let n = self.sample_count as f64;
            let delta = imu_sample.accel - self.mean_accel;
            self.mean_accel += delta / n;
            let delta2 = imu_sample.accel - self.mean_accel;
            self.cov_accel = self.cov_accel * (n - 1.0) / n
                + delta2.component_mul(&delta2) * (n - 1.0) / (n * n);
            self.mean_gravity += (imu_sample.gyro - self.mean_gravity) / n;
        });
    }

    pub(crate) fn finish(&mut self) -> Result<ImuInitialization> {
        if self.sample_count == 0 {
            anyhow::bail!("sample initialzed failed: sample count is none.");
        }
        let acc_norm = self.mean_accel.norm();
        if !acc_norm.is_finite() || acc_norm <= 1e-6 {
            anyhow::bail!("invalid acc norm: {acc_norm}");
        }

        let acc_unit = self.mean_accel / acc_norm;
        let acc_cov_max = self
            .cov_accel
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        if acc_cov_max <= 0.02 {
            let rotation_between =
                Rotation3::rotation_between(&acc_unit, &Vec3::new(0.0, 0.0, 1.0))
                    .expect("mean_acc is non-zero by construction.");

            let align_yaw = rotation_between[(1, 0)].atan2(rotation_between[(0, 0)]);
            let yaw_rot = Rotation3::from_axis_angle(&Vec3::z_axis(), -align_yaw);

            let orientation = UnitQuaternion::from_rotation_matrix(&(yaw_rot * rotation_between));
            Ok(ImuInitialization {
                gravity: Vec3::new(0.0, 0.0, -9.81),
                orientation,
                gravity_aligned: true,
                gyro_bias: self.mean_gravity,
                accel_scale: 9.81 / acc_norm,
            })
        } else {
            Ok(ImuInitialization {
                gravity: Vec3::new(0.0, 0.0, -9.81),
                orientation: UnitQuaternion::identity(),
                gravity_aligned: false,
                gyro_bias: self.mean_gravity,
                accel_scale: 9.81 / acc_norm,
            })
        }
    }
}

// TODO(pipeline): report initializing/bootstrap/tracking/tracking-lost mode,
// effective observations, IEKF iterations, map size, and per-stage timings.
pub struct PipelineFrameSummary {}

/// Patch `group_imu` so that its first and last samples coincide with the
/// LiDAR scan begin/end times, and return the IMU sample to carry into the
/// next frame as the deskew start boundary (`last_imu_for_deskew`).
///
/// Three roles are unified onto the current LiDAR frame's time axis:
/// 1. **Previous frame tail** (`last_imu_for_deskew`): leading real samples
///    that lie strictly before it belong to the previous scan interval and are
///    dropped **from the front** (never from the tail); the saved boundary is
///    then placed as the start sample so deskew starts at this frame's begin.
/// 2. **Synthetic begin IMU**: when the first IMU is already later than the
///    scan begin, a copy of it is inserted at `lidar_base_sec` so integration
///    does not start mid-air.
/// 3. **End IMU**: the group's tail is forced to exactly `lidar_end_sec` --
///    either by appending a copy (tail < end) or by linearly interpolating
///    between `previous_imu` and the tail down to `lidar_end_sec` (tail > end).
///
/// The returned sample is the **unified** tail (timed at `lidar_end_sec`), not
/// the raw last IMU. It becomes the next frame's starting boundary.
pub(crate) fn unify_imu_for_deskew(
    group_imu: &mut Vec<ImuSample>,
    lidar_base_sec: f64,
    lidar_end_sec: f64,
    last_imu_for_deskew: Option<ImuSample>,
) -> Result<Option<ImuSample>> {
    if let Some(last_imu) = last_imu_for_deskew {
        let stale_count =
            group_imu.partition_point(|imu| imu.time_stamp_sec < last_imu.time_stamp_sec);
        group_imu.drain(..stale_count);
        if group_imu
            .first()
            .is_none_or(|first| first.time_stamp_sec > last_imu.time_stamp_sec)
        {
            group_imu.insert(0, last_imu);
        }
    }

    if let Some(first_imu) = group_imu.first().cloned()
        && first_imu.time_stamp_sec > lidar_base_sec
    {
        let mut begin_imu = first_imu;
        begin_imu.time_stamp_sec = lidar_base_sec;
        group_imu.insert(0, begin_imu);
    }

    if let Some(last_imu) = group_imu.last().cloned()
        && last_imu.time_stamp_sec < lidar_end_sec
    {
        let mut end_imu = last_imu;
        end_imu.time_stamp_sec = lidar_end_sec;
        group_imu.push(end_imu);
    } else if let Some(last_imu) = group_imu.last().cloned()
        && last_imu.time_stamp_sec > lidar_end_sec
    {
        if group_imu.len() < 2 {
            anyhow::bail!("imu len < 2");
        }
        let len = group_imu.len();
        let previous_imu = group_imu[len - 2].clone();
        let alpha = (lidar_end_sec - previous_imu.time_stamp_sec)
            / (last_imu.time_stamp_sec - previous_imu.time_stamp_sec);
        let boundary = &mut group_imu[len - 1];
        boundary.time_stamp_sec = lidar_end_sec;
        boundary.gyro = previous_imu.gyro + alpha * (last_imu.gyro - previous_imu.gyro);
        boundary.accel = previous_imu.accel + alpha * (last_imu.accel - previous_imu.accel);
    }

    Ok(group_imu.last().cloned())
}

pub struct MainPipeline {
    pub map: SurfelMap,
    pub filter: IekfState,
    pub imu_integrator: ImuIntegrator,
    pub config: Config,
    pub extrinsic: LidarImuExtrinsic,
    pub initialized: bool,
    pub inital_group_count: usize,
    pub last_imu_for_deskew: Option<ImuSample>,
    pub initializer: ImuInitializer,
}

impl MainPipeline {
    pub fn new(config: Config) -> Self {
        Self {
            map: SurfelMap::new(
                config.surfel_map_config.clone().unwrap_or_default(),
                config.surfel_config.clone().unwrap_or_default(),
            ),
            imu_integrator: ImuIntegrator::init(
                config.mapping.gyr_cov,
                config.mapping.acc_cov,
                config.mapping.b_gyr_cov,
                config.mapping.b_acc_cov,
            ),
            filter: IekfState::default(),
            extrinsic: build_extrinsic_from_config(&config),
            config,
            initialized: false,
            inital_group_count: 10,
            last_imu_for_deskew: None,
            initializer: ImuInitializer::default(),
        }
    }

    pub fn process_measure_group(
        &mut self,
        mut group: MeasureGroup,
    ) -> Result<PipelineFrameSummary> {
        let imu_for_initialize = group.imu.clone();

        self.last_imu_for_deskew = unify_imu_for_deskew(
            &mut group.imu,
            group.lidar.base_timestamp_sec,
            group.lidar.end_timestamp_sec(),
            self.last_imu_for_deskew.clone(),
        )?;

        if !self.initialized {
            self.initializer.accumulate(&imu_for_initialize);
            if self.initializer.group_count < self.inital_group_count {
                return Ok(PipelineFrameSummary {});
            }

            let initialization = self.initializer.finish()?;
            if initialization.gravity_aligned {
                eprintln!(
                    "Gravity aligned success: gravity={}, orientation={}, gyro_bias={}, accel_scale={}",
                    initialization.gravity,
                    initialization.orientation,
                    initialization.gyro_bias,
                    initialization.accel_scale
                );
            } else {
                eprintln!(
                    "Gravity aligned skipped: gravity={}, orientation={}, gyro_bias={}, accel_scale={}",
                    initialization.gravity,
                    initialization.orientation,
                    initialization.gyro_bias,
                    initialization.accel_scale
                );
            }
            self.initialized = true;
            self.filter.state.gravity = initialization.gravity;
            self.filter.state.orientation = initialization.orientation;
            self.filter.state.gyro_bias = initialization.gyro_bias;
            self.imu_integrator
                .set_accel_scale(initialization.accel_scale)?;
            return Ok(PipelineFrameSummary {});
        }

        let previous_state = self.filter.state.clone();
        let mut predict_state = previous_state.clone();
        let segments = build_motion_segments(&group, previous_state, &self.imu_integrator)?;
        let mut predict_covariance = self.filter.covariance;
        for imu_pair in group.imu.windows(2) {
            let imu_prev = &imu_pair[0];
            let imu_curr = &imu_pair[1];
            predict_covariance = self.imu_integrator.propagate_covariance(
                &predict_state,
                predict_covariance,
                imu_prev,
                imu_curr,
            )?;
            self.imu_integrator.propagate_nominal_state_mut(
                &mut predict_state,
                imu_prev,
                imu_curr,
            )?;
        }
        self.filter.state = predict_state;
        self.filter.covariance = predict_covariance;

        deskew(&mut group.lidar, &segments, &self.extrinsic)?;
        let pointcloud = preprocess(&self.config.preprocess, group.lidar)?;

        if !self.map.is_empty() {
            let iekf_summary = self
                .filter
                .update(
                    &pointcloud.point_cloud,
                    &self.extrinsic,
                    &self.map,
                    &IekfConfig::default(),
                )
                .map_err(|e| anyhow!("IekfUpdateError: {:?}", e))?;

            if self.config.common.debug_mode {
                eprintln!("{:?}", iekf_summary);
            }
        }

        let map_points = pointcloud.point_cloud.iter().map(|p| {
            let point_i = self.extrinsic.transform_point(&p.to_vec3_f64());
            let point_w = self.filter.state.orientation * point_i + self.filter.state.position;
            PointXYZI {
                x: point_w.x as f32,
                y: point_w.y as f32,
                z: point_w.z as f32,
                intensity: p.intensity,
            }
        });

        self.map.insert(map_points)?;

        Ok(PipelineFrameSummary {})
    }
}

fn build_extrinsic_from_config(config: &Config) -> LidarImuExtrinsic {
    let t = config.mapping.extrinsic_t;
    let r = config.mapping.extrinsic_r;
    let rotation = Rotation3::from_matrix(&r);
    let quat = UnitQuaternion::from_rotation_matrix(&rotation);
    LidarImuExtrinsic {
        rotation: quat,
        translation: t,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fastlio_types::ImuSample;
    use nalgebra::Vector3;

    fn imu(t: f64, g: (f64, f64, f64), a: (f64, f64, f64)) -> ImuSample {
        ImuSample {
            time_stamp_sec: t,
            gyro: Vector3::new(g.0, g.1, g.2),
            accel: Vector3::new(a.0, a.1, a.2),
        }
    }

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    /// Two consecutive frames. In each frame the group's last IMU arrives
    /// *after* the LiDAR scan end (`tail > lidar_end`), so the pipeline must
    /// linearly interpolate the tail (using `previous_imu`) down to exactly
    /// `lidar_end`. The previous frame's tail is carried over (`last_imu_for
    /// _deskew`) and becomes the next frame's starting boundary. All three
    /// IMU roles must sit on the current LiDAR end-frame time axis.
    ///
    /// Constant-velocity motion makes the linear-in-time interpolation exact,
    /// so the boundary's timestamps/gyro/accel are verifiable in closed form:
    /// `boundary = prev + alpha * (tail - prev)`, `alpha = (end - prev_t)/(tail_t - prev_t)`.
    #[test]
    fn two_frames_imu_tail_after_scan_end_unifies_to_lidar_end() {
        // ---- Frame N: lidar [1.0, 1.5], raw IMU tail at t=2.0 > end=1.5 ----
        let mut frame_n = vec![
            imu(1.0, (0.0, 0.0, 1.0), (0.0, 0.0, 0.0)), // begin
            imu(1.2, (0.0, 0.0, 2.0), (1.0, 0.0, 0.0)), // previous_imu
            imu(2.0, (0.0, 0.0, 3.0), (2.0, 0.0, 0.0)), // tail (raw last, after end)
        ];
        let base_n = 1.0;
        let end_n = 1.5;

        let saved_n = unify_imu_for_deskew(&mut frame_n, base_n, end_n, None).unwrap();

        // Alpha for interpolation over the (1.2, 2.0) interval -> end 1.5.
        let alpha_n = (end_n - 1.2) / (2.0 - 1.2); // 0.3 / 0.8 = 0.375
        let boundary_n = &frame_n[frame_n.len() - 1];

        // The unified tail must be at exactly the LiDAR end time.
        assert!(
            approx_eq(boundary_n.time_stamp_sec, end_n, 1e-12),
            "frame N tail time {} != lidar end {}",
            boundary_n.time_stamp_sec,
            end_n
        );
        // Interpolated gyro/accel verifiable in closed form.
        assert!(
            approx_eq(boundary_n.gyro.z, 2.0 + alpha_n * (3.0 - 2.0), 1e-12),
            "frame N boundary gyro.z {} != {}",
            boundary_n.gyro.z,
            2.0 + alpha_n * (3.0 - 2.0)
        );
        assert!(
            approx_eq(boundary_n.accel.x, 1.0 + alpha_n * (2.0 - 1.0), 1e-12),
            "frame N boundary accel.x {} != {}",
            boundary_n.accel.x,
            1.0 + alpha_n * (2.0 - 1.0)
        );

        // The original real samples in the group are preserved (only the tail
        // sample was re-programmed in place).
        assert!(approx_eq(frame_n[0].time_stamp_sec, 1.0, 1e-12));
        assert!(approx_eq(frame_n[1].time_stamp_sec, 1.2, 1e-12));
        assert_eq!(frame_n[1].accel.x, 1.0);

        // The value carried into the next frame is the *unified* end sample,
        // not the raw tail.
        let saved_n = saved_n.unwrap();
        assert!(approx_eq(saved_n.time_stamp_sec, end_n, 1e-12));
        assert!(approx_eq(saved_n.gyro.z, boundary_n.gyro.z, 1e-12));

        // ---- Frame N+1: lidar [1.5, 2.5], saved end boundary must be the
        //      starting IMU, and its own tail (t=3.0) interpolated to end 2.5.
        let mut frame_n1 = vec![
            imu(2.0, (0.0, 0.0, 4.0), (0.0, 0.0, 0.0)),
            imu(2.4, (0.0, 0.0, 5.0), (1.5, 0.0, 0.0)), // previous_imu
            imu(3.0, (0.0, 0.0, 6.0), (2.5, 0.0, 0.0)), // tail, after end
        ];
        let base_n1 = 1.5;
        let end_n1 = 2.5;

        let saved_n1 = unify_imu_for_deskew(&mut frame_n1, base_n1, end_n1, Some(saved_n)).unwrap();

        // Previous frame's unified end boundary is prepended as the start of
        // this frame's IMU sequence (deskew start boundary).
        assert!(
            approx_eq(frame_n1[0].time_stamp_sec, end_n, 1e-12),
            "frame N+1 first IMU {} != previous unified end {}",
            frame_n1[0].time_stamp_sec,
            end_n
        );
        assert!(
            approx_eq(frame_n1[0].gyro.z, boundary_n.gyro.z, 1e-12),
            "frame N+1 first IMU must carry the previous frame's unified boundary"
        );

        let alpha_n1 = (end_n1 - 2.4) / (3.0 - 2.4); // 0.1 / 0.6
        let boundary_n1 = &frame_n1[frame_n1.len() - 1];
        assert!(
            approx_eq(boundary_n1.time_stamp_sec, end_n1, 1e-12),
            "frame N+1 tail time {} != lidar end {}",
            boundary_n1.time_stamp_sec,
            end_n1
        );
        assert!(approx_eq(
            boundary_n1.gyro.z,
            5.0 + alpha_n1 * (6.0 - 5.0),
            1e-12
        ));
        assert!(approx_eq(
            boundary_n1.accel.x,
            1.5 + alpha_n1 * (2.5 - 1.5),
            1e-12
        ));

        let saved_n1 = saved_n1.unwrap();
        assert!(approx_eq(saved_n1.time_stamp_sec, end_n1, 1e-12));
    }

    /// The group's first real IMU arrives just *before* the carried boundary
    /// (1.099999 < 1.100000). Those leading samples belong to the previous
    /// scan interval and must be dropped **from the front**, never from the
    /// tail. After the drop the saved boundary becomes the start sample, while
    /// the tail's real IMU (1.108) is preserved.
    #[test]
    fn front_imu_before_saved_boundary_is_trimmed_not_tail() {
        let saved_boundary = imu(1.100000, (0.0, 0.0, 9.0), (3.0, 0.0, 0.0));

        // First real sample precedes the boundary by 1e-6.
        let mut group = vec![
            imu(1.099999, (0.0, 0.0, 1.0), (0.0, 0.0, 0.0)),
            imu(1.103000, (0.0, 0.0, 2.0), (1.0, 0.0, 0.0)),
            imu(1.108000, (0.0, 0.0, 3.0), (2.0, 0.0, 0.0)),
        ];

        let saved =
            unify_imu_for_deskew(&mut group, 1.100000, 1.108000, Some(saved_boundary.clone()))
                .unwrap();

        // The offending leading sample is gone, replaced by the saved boundary.
        assert_eq!(group.len(), 3, "expected trimmed+replaced sequence length");
        assert!(
            approx_eq(group[0].time_stamp_sec, 1.100000, 1e-12),
            "front must start at the saved boundary, got {}",
            group[0].time_stamp_sec
        );
        assert_eq!(group[0].gyro.z, 9.0, "front must carry the saved boundary");
        assert_eq!(group[0].accel.x, 3.0);

        // The tail's real IMU is preserved, not deleted.
        assert!(
            approx_eq(group[2].time_stamp_sec, 1.108000, 1e-12),
            "tail real IMU must be kept, got {}",
            group[2].time_stamp_sec
        );
        assert_eq!(group[2].gyro.z, 3.0);
        assert_eq!(group[2].accel.x, 2.0);

        // Middle real sample untouched.
        assert!(approx_eq(group[1].time_stamp_sec, 1.103000, 1e-12));

        // The sample saved for the next frame is still the unified tail.
        assert!(approx_eq(saved.unwrap().time_stamp_sec, 1.108000, 1e-12));
    }
}
