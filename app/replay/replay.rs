use std::env;
use std::fs::{self, File};
use std::io::{BufWriter, Write};

use anyhow::{Context, Result, bail};
use camino::{Utf8Path, Utf8PathBuf};
use fastlio_dataset::{SensorEvent, read_mcap_events};
use fastlio_estimator::iesekf::{ErrorStateCovariance, Iesekf, IesekfConfig};
use fastlio_imu::ImuIntegrator;
use fastlio_map::{LocalMap, PointToPlaneConfig};
use fastlio_pipeline::main_pipeline::{FastLioPipeline, PipelineConfig, PipelineMode};
use fastlio_pipeline::synchronizer::MeasurementSynchronizer;
use fastlio_types::{
    Config, LidarImuExtrinsic, NavState, PointXYZI, Pose3, Vec3, read_from_config_path,
};
use nalgebra::UnitQuaternion;

fn main() -> Result<()> {
    let args = ReplayArgs::parse(env::args().collect())?;
    fs::create_dir_all(&args.output_dir)
        .with_context(|| format!("failed to create output dir `{}`", args.output_dir))?;

    let config = read_from_config_path(&args.config_path)
        .with_context(|| format!("failed to read config `{}`", args.config_path))?;
    let mut replay = OfflineReplay::new(config)?;
    let stats = read_mcap_events(&args.bag_path, |event| replay.on_event(event))
        .with_context(|| format!("failed to replay MCAP `{}`", args.bag_path))?;

    let trajectory_path = args.output_dir.join("trajectory.csv");
    let map_path = args.output_dir.join("map.pcd");
    let summary_path = args.output_dir.join("summary.txt");

    write_trajectory_csv(&trajectory_path, &replay.trajectory)?;
    write_ascii_pcd(&map_path, replay.pipeline.local_map.points())?;
    write_summary(&summary_path, stats, &replay)?;

    println!("replay complete");
    println!("  processed frames: {}", replay.processed_frames);
    println!("  tracking frames: {}", replay.tracking_frames);
    println!("  bootstrap frames: {}", replay.bootstrap_frames);
    println!("  failed groups: {}", replay.failed_groups);
    println!("  map points: {}", replay.pipeline.local_map.len());
    println!("  trajectory: {trajectory_path}");
    println!("  map: {map_path}");
    println!("  summary: {summary_path}");

    Ok(())
}

struct ReplayArgs {
    bag_path: Utf8PathBuf,
    config_path: Utf8PathBuf,
    output_dir: Utf8PathBuf,
}

impl ReplayArgs {
    fn parse(args: Vec<String>) -> Result<Self> {
        if args.len() != 4 {
            bail!(
                "usage: {} <bag.mcap> <config.yaml> <output_dir>",
                args.first().map(String::as_str).unwrap_or("fastlio-replay")
            );
        }

        Ok(Self {
            bag_path: Utf8PathBuf::from(&args[1]),
            config_path: Utf8PathBuf::from(&args[2]),
            output_dir: Utf8PathBuf::from(&args[3]),
        })
    }
}

#[derive(Debug, Clone)]
struct TrajectoryRow {
    frame_index: usize,
    timestamp_sec: f64,
    position: Vec3<f64>,
    orientation_wxyz: [f64; 4],
    mode: PipelineMode,
    effective_observations: usize,
    map_points: usize,
}

struct OfflineReplay {
    synchronizer: MeasurementSynchronizer,
    pipeline: FastLioPipeline,
    trajectory: Vec<TrajectoryRow>,
    processed_frames: usize,
    tracking_frames: usize,
    bootstrap_frames: usize,
    failed_groups: usize,
}

impl OfflineReplay {
    fn new(config: Config) -> Result<Self> {
        let imu_integrator = ImuIntegrator::init(
            config.mapping.gyr_cov,
            config.mapping.acc_cov,
            config.mapping.b_gyr_cov,
            config.mapping.b_acc_cov,
        );
        let initial_state = NavState {
            position: Vec3::zeros(),
            orientation: UnitQuaternion::identity(),
            velocity: Vec3::zeros(),
            gyro_bias: Vec3::zeros(),
            accel_bias: Vec3::zeros(),
            gravity: Vec3::new(0.0, 0.0, -9.81),
        };
        let initial_covariance = ErrorStateCovariance::identity() * 0.1;
        let filter = Iesekf::new(initial_state, initial_covariance)
            .map_err(|err| anyhow::anyhow!("failed to initialize IESEKF: {err:?}"))?;
        let extrinsic = lidar_imu_extrinsic_from_config(&config)?;
        let pipeline_config = PipelineConfig {
            preprocess: config.preprocess,
            point_to_plane: PointToPlaneConfig {
                nearest_count: 5,
                max_neighbour_distance: 1.5,
                max_absolute_residual: 0.5,
                ..PointToPlaneConfig::default()
            },
            iesekf: IesekfConfig {
                measurement_noise_variance: 1.0e-3,
                ..IesekfConfig::default()
            },
            min_effective_observations: 10,
            map_crop_radius: Some(100.0),
            insert_scan_points: true,
        };

        Ok(Self {
            synchronizer: MeasurementSynchronizer::new(),
            pipeline: FastLioPipeline::new(
                filter,
                LocalMap::new(),
                imu_integrator,
                extrinsic,
                pipeline_config,
            ),
            trajectory: Vec::new(),
            processed_frames: 0,
            tracking_frames: 0,
            bootstrap_frames: 0,
            failed_groups: 0,
        })
    }

    fn on_event(&mut self, event: SensorEvent) -> Result<()> {
        let group = match event {
            SensorEvent::Imu(imu) => self.synchronizer.pend_imu(imu)?,
            SensorEvent::Lidar(lidar) => self.synchronizer.pend_lidar(lidar)?,
        };

        if let Some(group) = group {
            let timestamp_sec = group.lidar.end_timestamp_sec();
            match self.pipeline.process_measurement_group(group) {
                Ok(report) => {
                    self.processed_frames += 1;
                    match report.mode {
                        PipelineMode::BootstrapMap => self.bootstrap_frames += 1,
                        PipelineMode::Tracking => self.tracking_frames += 1,
                    }
                    self.push_trajectory_row(
                        timestamp_sec,
                        report.mode,
                        report.effective_observations,
                    );
                }
                Err(err) => {
                    self.failed_groups += 1;
                    eprintln!("frame processing failed at {timestamp_sec:.6}: {err:#}");
                }
            }
        }

        Ok(())
    }

    fn push_trajectory_row(
        &mut self,
        timestamp_sec: f64,
        mode: PipelineMode,
        effective_observations: usize,
    ) {
        let state = &self.pipeline.filter.state;
        let quat = state.orientation.quaternion();
        self.trajectory.push(TrajectoryRow {
            frame_index: self.processed_frames,
            timestamp_sec,
            position: state.position,
            orientation_wxyz: [quat.w, quat.i, quat.j, quat.k],
            mode,
            effective_observations,
            map_points: self.pipeline.local_map.len(),
        });
    }
}

fn lidar_imu_extrinsic_from_config(config: &Config) -> Result<LidarImuExtrinsic> {
    let rotation = UnitQuaternion::from_matrix(&config.mapping.extrinsic_r);
    if !rotation.angle().is_finite()
        || !config
            .mapping
            .extrinsic_t
            .iter()
            .all(|value| value.is_finite())
    {
        bail!("invalid LiDAR-IMU extrinsic in config");
    }
    Ok(Pose3::new(rotation, config.mapping.extrinsic_t))
}

fn write_trajectory_csv(path: &Utf8Path, trajectory: &[TrajectoryRow]) -> Result<()> {
    let mut writer = BufWriter::new(
        File::create(path).with_context(|| format!("failed to create trajectory `{path}`"))?,
    );
    writeln!(
        writer,
        "frame_index,timestamp_sec,x,y,z,qw,qx,qy,qz,mode,effective_observations,map_points"
    )?;
    for row in trajectory {
        writeln!(
            writer,
            "{},{:.9},{:.9},{:.9},{:.9},{:.12},{:.12},{:.12},{:.12},{:?},{},{}",
            row.frame_index,
            row.timestamp_sec,
            row.position.x,
            row.position.y,
            row.position.z,
            row.orientation_wxyz[0],
            row.orientation_wxyz[1],
            row.orientation_wxyz[2],
            row.orientation_wxyz[3],
            row.mode,
            row.effective_observations,
            row.map_points
        )?;
    }
    Ok(())
}

fn write_ascii_pcd(path: &Utf8Path, points: &[PointXYZI]) -> Result<()> {
    let mut writer = BufWriter::new(
        File::create(path).with_context(|| format!("failed to create map `{path}`"))?,
    );
    writeln!(writer, "# .PCD v0.7 - Point Cloud Data file format")?;
    writeln!(writer, "VERSION 0.7")?;
    writeln!(writer, "FIELDS x y z intensity")?;
    writeln!(writer, "SIZE 4 4 4 4")?;
    writeln!(writer, "TYPE F F F F")?;
    writeln!(writer, "COUNT 1 1 1 1")?;
    writeln!(writer, "WIDTH {}", points.len())?;
    writeln!(writer, "HEIGHT 1")?;
    writeln!(writer, "VIEWPOINT 0 0 0 1 0 0 0")?;
    writeln!(writer, "POINTS {}", points.len())?;
    writeln!(writer, "DATA ascii")?;
    for point in points {
        writeln!(
            writer,
            "{:.6} {:.6} {:.6} {:.6}",
            point.x, point.y, point.z, point.intensity
        )?;
    }
    Ok(())
}

fn write_summary(
    path: &Utf8Path,
    read_stats: fastlio_dataset::ReadStats,
    replay: &OfflineReplay,
) -> Result<()> {
    let mut writer = BufWriter::new(
        File::create(path).with_context(|| format!("failed to create summary `{path}`"))?,
    );
    writeln!(writer, "total_messages={}", read_stats.total_messages)?;
    writeln!(writer, "emitted_events={}", read_stats.emitted_events)?;
    writeln!(
        writer,
        "skipped_missing_schema={}",
        read_stats.skipped_missing_schema
    )?;
    writeln!(
        writer,
        "skipped_unsupported_schema={}",
        read_stats.skipped_unsupported_schema
    )?;
    writeln!(
        writer,
        "skipped_unsupported_topic={}",
        read_stats.skipped_unsupported_topic
    )?;
    writeln!(writer, "processed_frames={}", replay.processed_frames)?;
    writeln!(writer, "tracking_frames={}", replay.tracking_frames)?;
    writeln!(writer, "bootstrap_frames={}", replay.bootstrap_frames)?;
    writeln!(writer, "failed_groups={}", replay.failed_groups)?;
    writeln!(writer, "map_points={}", replay.pipeline.local_map.len())?;
    writeln!(writer)?;
    writeln!(writer, "minimal_implementation_notes=")?;
    writeln!(
        writer,
        "- callback replay only; no SPSC queue, playback rate control, sleep, pause, or backpressure yet"
    )?;
    writeln!(
        writer,
        "- dataset reader still hardcodes /livox/lidar and /livox/imu instead of using config topics"
    )?;
    writeln!(
        writer,
        "- initial state is fixed identity pose with gravity [0,0,-9.81]; no IMU initialization window yet"
    )?;
    writeln!(
        writer,
        "- map output is a single ASCII PCD; no incremental map shards or rerun visualization yet"
    )?;
    writeln!(
        writer,
        "- failed synchronized frames are counted and skipped; no retry or queue-based recovery yet"
    )?;
    Ok(())
}
