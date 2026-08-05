use std::env;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::net::TcpStream;
use std::path::Path;
use std::process::Command;
use std::sync::mpsc::{self, Receiver, SyncSender};
use std::thread;
use std::time::{Duration, Instant};

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

    let mut config = read_from_config_path(&args.config_path)
        .with_context(|| format!("failed to read config `{}`", args.config_path))?;
    if let Some(offset_sec) = args.time_offset_lidar_to_imu_override {
        config.common.time_offset_lidar_to_imu = Some(offset_sec);
    }
    let mut replay = OfflineReplay::new(config)?;
    replay.live_stream = start_live_viewer(&args.viewer)?;
    let stats = run_spsc_replay(&args, &mut replay)
        .with_context(|| format!("failed to replay MCAP `{}`", args.bag_path))?;

    let trajectory_path = args.output_dir.join("trajectory.csv");
    let map_path = args.output_dir.join("map.pcd");
    let summary_path = args.output_dir.join("summary.txt");

    write_trajectory_csv(&trajectory_path, &replay.trajectory)?;
    write_ascii_pcd(&map_path, replay.pipeline.local_map.points())?;
    write_summary(&summary_path, stats, &replay)?;
    open_viewer(&args.viewer, &map_path)?;

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
    playback_rate: f64,
    channel_capacity: usize,
    time_offset_lidar_to_imu_override: Option<f64>,
    viewer: ReplayViewer,
}

impl ReplayArgs {
    fn parse(args: Vec<String>) -> Result<Self> {
        if args.len() < 4 {
            bail!(
                "usage: {} <bag.mcap> <config.yaml> <output_dir> [playback_rate] [channel_capacity] [--time-offset-lidar-to-imu <sec>] [--open-playground|--live-playground|--playground <path>|--live-addr <addr>]",
                args.first().map(String::as_str).unwrap_or("fastlio-replay")
            );
        }
        let mut next_arg = 4;
        let playback_rate =
            if let Some(rate) = args.get(next_arg).filter(|arg| !arg.starts_with("--")) {
                next_arg += 1;
                rate.parse::<f64>()
                    .with_context(|| format!("invalid playback_rate `{rate}`"))?
            } else {
                0.0
            };
        if playback_rate < 0.0 || !playback_rate.is_finite() {
            bail!("playback_rate must be finite and non-negative");
        }

        let channel_capacity =
            if let Some(capacity) = args.get(next_arg).filter(|arg| !arg.starts_with("--")) {
                next_arg += 1;
                capacity
                    .parse::<usize>()
                    .with_context(|| format!("invalid channel_capacity `{capacity}`"))?
            } else {
                1024
            };
        if channel_capacity == 0 {
            bail!("channel_capacity must be positive");
        }
        let (viewer_args, time_offset_lidar_to_imu_override) =
            parse_replay_options(&args[next_arg..])?;
        let viewer = ReplayViewer::parse(&viewer_args)?;

        Ok(Self {
            bag_path: Utf8PathBuf::from(&args[1]),
            config_path: Utf8PathBuf::from(&args[2]),
            output_dir: Utf8PathBuf::from(&args[3]),
            playback_rate,
            channel_capacity,
            time_offset_lidar_to_imu_override,
            viewer,
        })
    }
}

fn parse_replay_options(args: &[String]) -> Result<(Vec<String>, Option<f64>)> {
    let mut viewer_args = Vec::new();
    let mut time_offset_lidar_to_imu_override = None;
    let mut idx = 0;
    while idx < args.len() {
        match args[idx].as_str() {
            "--time-offset-lidar-to-imu" => {
                let Some(value) = args.get(idx + 1) else {
                    bail!("--time-offset-lidar-to-imu requires a seconds value");
                };
                let offset_sec = value
                    .parse::<f64>()
                    .with_context(|| format!("invalid --time-offset-lidar-to-imu `{value}`"))?;
                if !offset_sec.is_finite() {
                    bail!("--time-offset-lidar-to-imu must be finite");
                }
                time_offset_lidar_to_imu_override = Some(offset_sec);
                idx += 2;
            }
            _ => {
                viewer_args.push(args[idx].clone());
                idx += 1;
            }
        }
    }

    Ok((viewer_args, time_offset_lidar_to_imu_override))
}

enum ReplayViewer {
    None,
    OpenAfter {
        executable: Utf8PathBuf,
    },
    LivePlayground {
        executable: Utf8PathBuf,
        listen_addr: String,
    },
}

impl ReplayViewer {
    fn parse(args: &[String]) -> Result<Self> {
        let mut viewer = ReplayViewer::None;
        let mut idx = 0;
        while idx < args.len() {
            match args[idx].as_str() {
                "--no-viewer" => {
                    viewer = ReplayViewer::None;
                    idx += 1;
                }
                "--open-playground" => {
                    viewer = ReplayViewer::OpenAfter {
                        executable: default_playground_executable(),
                    };
                    idx += 1;
                }
                "--live-playground" => {
                    viewer = ReplayViewer::LivePlayground {
                        executable: default_playground_executable(),
                        listen_addr: default_live_addr(),
                    };
                    idx += 1;
                }
                "--live-addr" => {
                    let Some(addr) = args.get(idx + 1) else {
                        bail!("--live-addr requires an address");
                    };
                    match &mut viewer {
                        ReplayViewer::LivePlayground { listen_addr, .. } => {
                            *listen_addr = addr.clone();
                        }
                        _ => {
                            viewer = ReplayViewer::LivePlayground {
                                executable: default_playground_executable(),
                                listen_addr: addr.clone(),
                            };
                        }
                    }
                    idx += 2;
                }
                "--playground" => {
                    let Some(path) = args.get(idx + 1) else {
                        bail!("--playground requires an executable path");
                    };
                    let executable = Utf8PathBuf::from(path);
                    match &mut viewer {
                        ReplayViewer::LivePlayground {
                            executable: current,
                            ..
                        }
                        | ReplayViewer::OpenAfter {
                            executable: current,
                        } => {
                            *current = executable;
                        }
                        ReplayViewer::None => {
                            viewer = ReplayViewer::OpenAfter { executable };
                        }
                    }
                    idx += 2;
                }
                other => bail!("unknown replay option `{other}`"),
            }
        }
        Ok(viewer)
    }
}

fn default_playground_executable() -> Utf8PathBuf {
    let release = Utf8PathBuf::from("target/release/fastlio-playground");
    if release.exists() {
        release
    } else {
        Utf8PathBuf::from("target/debug/fastlio-playground")
    }
}

fn default_live_addr() -> String {
    "127.0.0.1:9876".to_string()
}

fn open_viewer(viewer: &ReplayViewer, map_path: &Utf8Path) -> Result<()> {
    match viewer {
        ReplayViewer::None => Ok(()),
        ReplayViewer::OpenAfter { executable } => {
            if !Path::new(executable.as_str()).exists() {
                bail!(
                    "playground executable `{}` does not exist; build it first or pass --playground <path>",
                    executable
                );
            }
            Command::new(executable.as_str())
                .arg(map_path.as_str())
                .spawn()
                .with_context(|| format!("failed to launch playground viewer `{executable}`"))?;
            Ok(())
        }
        ReplayViewer::LivePlayground { .. } => Ok(()),
    }
}

fn start_live_viewer(viewer: &ReplayViewer) -> Result<Option<LivePointStream>> {
    let ReplayViewer::LivePlayground {
        executable,
        listen_addr,
    } = viewer
    else {
        return Ok(None);
    };
    if !Path::new(executable.as_str()).exists() {
        bail!(
            "playground executable `{}` does not exist; build it first or pass --playground <path>",
            executable
        );
    }
    Command::new(executable.as_str())
        .arg("--listen")
        .arg(listen_addr)
        .spawn()
        .with_context(|| format!("failed to launch live playground viewer `{executable}`"))?;

    let mut last_err = None;
    for _ in 0..100 {
        match TcpStream::connect(listen_addr) {
            Ok(stream) => {
                stream
                    .set_nodelay(true)
                    .context("failed to set TCP_NODELAY on live stream")?;
                return Ok(Some(LivePointStream::new(stream)));
            }
            Err(err) => {
                last_err = Some(err);
                thread::sleep(Duration::from_millis(50));
            }
        }
    }

    Err(anyhow::anyhow!(
        "failed to connect to live playground at `{}`: {}",
        listen_addr,
        last_err
            .map(|err| err.to_string())
            .unwrap_or_else(|| "unknown connection error".to_string())
    ))
}

struct LivePointStream {
    writer: BufWriter<TcpStream>,
    sent_map_points: usize,
}

impl LivePointStream {
    fn new(stream: TcpStream) -> Self {
        Self {
            writer: BufWriter::new(stream),
            sent_map_points: 0,
        }
    }

    fn send_map_delta(&mut self, points: &[PointXYZI]) -> Result<()> {
        if points.len() < self.sent_map_points {
            writeln!(self.writer, "clear")?;
            self.sent_map_points = 0;
        }

        for point in &points[self.sent_map_points..] {
            writeln!(
                self.writer,
                "point {:.6} {:.6} {:.6} {:.6}",
                point.x, point.y, point.z, point.intensity
            )?;
        }
        writeln!(self.writer, "flush")?;
        self.writer.flush()?;
        self.sent_map_points = points.len();
        Ok(())
    }
}

enum ReplayMessage {
    Event(SensorEvent),
    Finished(Result<fastlio_dataset::ReadStats, String>),
}

fn run_spsc_replay(
    args: &ReplayArgs,
    replay: &mut OfflineReplay,
) -> Result<fastlio_dataset::ReadStats> {
    let (sender, receiver) = mpsc::sync_channel(args.channel_capacity);
    let bag_path = args.bag_path.clone();
    let producer = thread::spawn(move || produce_events(bag_path, sender));

    let stats = consume_events(receiver, replay, args.playback_rate)?;
    producer
        .join()
        .map_err(|_| anyhow::anyhow!("replay producer thread panicked"))?;
    Ok(stats)
}

fn produce_events(bag_path: Utf8PathBuf, sender: SyncSender<ReplayMessage>) {
    let event_sender = sender.clone();
    let result = read_mcap_events(&bag_path, |event| {
        event_sender
            .send(ReplayMessage::Event(event))
            .map_err(|_| anyhow::anyhow!("replay consumer stopped"))?;
        Ok(())
    })
    .map_err(|err| format!("{err:#}"));

    let _ = sender.send(ReplayMessage::Finished(result));
}

fn consume_events(
    receiver: Receiver<ReplayMessage>,
    replay: &mut OfflineReplay,
    playback_rate: f64,
) -> Result<fastlio_dataset::ReadStats> {
    let mut clock = PlaybackClock::new(playback_rate);
    while let Ok(message) = receiver.recv() {
        match message {
            ReplayMessage::Event(event) => {
                clock.sleep_until_event_time(event.timestamp_sec());
                replay.on_event(event)?;
            }
            ReplayMessage::Finished(result) => return result.map_err(|err| anyhow::anyhow!(err)),
        }
    }

    bail!("replay producer stopped before sending completion stats")
}

struct PlaybackClock {
    playback_rate: f64,
    first_sensor_time: Option<f64>,
    wall_start: Instant,
}

impl PlaybackClock {
    fn new(playback_rate: f64) -> Self {
        Self {
            playback_rate,
            first_sensor_time: None,
            wall_start: Instant::now(),
        }
    }

    fn sleep_until_event_time(&mut self, sensor_time_sec: f64) {
        if self.playback_rate <= 0.0 || !sensor_time_sec.is_finite() {
            return;
        }

        let first_sensor_time = *self.first_sensor_time.get_or_insert(sensor_time_sec);
        let sensor_elapsed = sensor_time_sec - first_sensor_time;
        if sensor_elapsed <= 0.0 {
            return;
        }

        let target_wall_elapsed = Duration::from_secs_f64(sensor_elapsed / self.playback_rate);
        let elapsed = self.wall_start.elapsed();
        if target_wall_elapsed > elapsed {
            thread::sleep(target_wall_elapsed - elapsed);
        }
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
    /// IMU absolute timestamps are shifted into the LiDAR clock domain as:
    /// `t_imu_for_sync = t_imu_raw - time_offset_lidar_to_imu_sec`.
    time_offset_lidar_to_imu_sec: f64,
    trajectory: Vec<TrajectoryRow>,
    processed_frames: usize,
    tracking_frames: usize,
    bootstrap_frames: usize,
    failed_groups: usize,
    max_pending_lidar: usize,
    live_stream: Option<LivePointStream>,
    first_imu_raw_time_sec: Option<f64>,
    last_imu_raw_time_sec: Option<f64>,
    first_imu_time_sec: Option<f64>,
    last_imu_time_sec: Option<f64>,
    first_lidar_raw_time_sec: Option<f64>,
    last_lidar_raw_time_sec: Option<f64>,
    first_lidar_time_sec: Option<f64>,
    last_lidar_time_sec: Option<f64>,
}

impl OfflineReplay {
    fn new(config: Config) -> Result<Self> {
        let time_offset_lidar_to_imu_sec = config.common.time_offset_lidar_to_imu.unwrap_or(0.0);
        if !time_offset_lidar_to_imu_sec.is_finite() {
            bail!("common.time_offset_lidar_to_imu must be finite when present");
        }
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
            max_factor_points: Some(2_000),
            max_map_insert_points: Some(5_000),
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
            time_offset_lidar_to_imu_sec,
            trajectory: Vec::new(),
            processed_frames: 0,
            tracking_frames: 0,
            bootstrap_frames: 0,
            failed_groups: 0,
            max_pending_lidar: 0,
            live_stream: None,
            first_imu_raw_time_sec: None,
            last_imu_raw_time_sec: None,
            first_imu_time_sec: None,
            last_imu_time_sec: None,
            first_lidar_raw_time_sec: None,
            last_lidar_raw_time_sec: None,
            first_lidar_time_sec: None,
            last_lidar_time_sec: None,
        })
    }

    fn on_event(&mut self, event: SensorEvent) -> Result<()> {
        let group = match event {
            SensorEvent::Imu(mut imu) => {
                self.first_imu_raw_time_sec
                    .get_or_insert(imu.time_stamp_sec);
                self.last_imu_raw_time_sec = Some(imu.time_stamp_sec);
                imu.time_stamp_sec -= self.time_offset_lidar_to_imu_sec;
                self.first_imu_time_sec.get_or_insert(imu.time_stamp_sec);
                self.last_imu_time_sec = Some(imu.time_stamp_sec);
                self.synchronizer.pend_imu(imu)?
            }
            SensorEvent::Lidar(lidar) => {
                self.first_lidar_raw_time_sec
                    .get_or_insert(lidar.base_timestamp_sec);
                self.last_lidar_raw_time_sec = Some(lidar.base_timestamp_sec);
                self.first_lidar_time_sec
                    .get_or_insert(lidar.base_timestamp_sec);
                self.last_lidar_time_sec = Some(lidar.base_timestamp_sec);
                self.synchronizer.pend_lidar(lidar)?
            }
        };

        if let Some(group) = group {
            self.process_group(group);
        }
        for group in self.synchronizer.drain_ready()? {
            self.process_group(group);
        }
        self.max_pending_lidar = self
            .max_pending_lidar
            .max(self.synchronizer.pending_lidar.len());

        Ok(())
    }

    fn process_group(&mut self, group: fastlio_types::MeasureGroup) {
        let timestamp_sec = group.lidar.end_timestamp_sec();
        match self.pipeline.process_measurement_group(group) {
            Ok(report) => {
                self.processed_frames += 1;
                match report.mode {
                    PipelineMode::BootstrapMap => self.bootstrap_frames += 1,
                    PipelineMode::Tracking => self.tracking_frames += 1,
                }
                self.push_trajectory_row(timestamp_sec, report.mode, report.effective_observations);
                if let Some(live_stream) = &mut self.live_stream
                    && let Err(err) = live_stream.send_map_delta(self.pipeline.local_map.points())
                {
                    eprintln!("live viewer stream failed at {timestamp_sec:.6}: {err:#}");
                    self.live_stream = None;
                }
            }
            Err(err) => {
                self.failed_groups += 1;
                eprintln!("frame processing failed at {timestamp_sec:.6}: {err:#}");
            }
        }
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
    writeln!(writer, "max_pending_lidar={}", replay.max_pending_lidar)?;
    writeln!(
        writer,
        "time_offset_lidar_to_imu_sec={:.9}",
        replay.time_offset_lidar_to_imu_sec
    )?;
    writeln!(
        writer,
        "time_offset_convention=t_imu_for_sync=t_imu_raw-time_offset_lidar_to_imu_sec"
    )?;
    writeln!(
        writer,
        "first_imu_raw_time_sec={}",
        format_optional_f64(replay.first_imu_raw_time_sec)
    )?;
    writeln!(
        writer,
        "last_imu_raw_time_sec={}",
        format_optional_f64(replay.last_imu_raw_time_sec)
    )?;
    writeln!(
        writer,
        "first_imu_time_sec={}",
        format_optional_f64(replay.first_imu_time_sec)
    )?;
    writeln!(
        writer,
        "last_imu_time_sec={}",
        format_optional_f64(replay.last_imu_time_sec)
    )?;
    writeln!(
        writer,
        "first_lidar_raw_time_sec={}",
        format_optional_f64(replay.first_lidar_raw_time_sec)
    )?;
    writeln!(
        writer,
        "last_lidar_raw_time_sec={}",
        format_optional_f64(replay.last_lidar_raw_time_sec)
    )?;
    writeln!(
        writer,
        "first_lidar_time_sec={}",
        format_optional_f64(replay.first_lidar_time_sec)
    )?;
    writeln!(
        writer,
        "last_lidar_time_sec={}",
        format_optional_f64(replay.last_lidar_time_sec)
    )?;
    writeln!(
        writer,
        "dropped_lidar_before_first_imu={}",
        replay.synchronizer.dropped_lidar_before_first_imu
    )?;
    if let Some(drop) = replay.synchronizer.first_lidar_drop_before_first_imu {
        writeln!(
            writer,
            "first_lidar_drop_before_first_imu={:.9},{:.9},{:.9}",
            drop.lidar_base_time_sec, drop.lidar_end_time_sec, drop.first_imu_time_sec
        )?;
    } else {
        writeln!(writer, "first_lidar_drop_before_first_imu=none")?;
    }
    writeln!(writer, "map_points={}", replay.pipeline.local_map.len())?;
    writeln!(writer)?;
    writeln!(writer, "minimal_implementation_notes=")?;
    writeln!(
        writer,
        "- SPSC replay is implemented with a bounded std::sync::mpsc::sync_channel and optional sensor-time playback_rate"
    )?;
    writeln!(
        writer,
        "- replay limits scan-to-map association to 2000 deterministic scan samples and map insertion to 5000 deterministic scan samples per frame"
    )?;
    writeln!(
        writer,
        "- MeasurementSynchronizer follows FAST-LIO-style packaging: wait for IMU coverage at LiDAR end time and let the pipeline prepend the previous frame tail IMU"
    )?;
    writeln!(
        writer,
        "- dataset reader still hardcodes /livox/lidar and /livox/imu instead of using config topics"
    )?;
    writeln!(
        writer,
        "- LiDAR-IMU clock offset follows FAST-LIO semantics by shifting IMU timestamps; no automatic offset calibration yet"
    )?;
    writeln!(
        writer,
        "- initial state is fixed identity pose with gravity [0,0,-9.81]; no IMU initialization window yet"
    )?;
    writeln!(
        writer,
        "- map output is a single ASCII PCD; pass --open-playground to launch /home/lyra/playground on that PCD after replay"
    )?;
    writeln!(
        writer,
        "- failed synchronized frames are counted and skipped; no retry or queue-based recovery yet"
    )?;
    Ok(())
}

fn format_optional_f64(value: Option<f64>) -> String {
    value.map_or_else(|| "none".to_string(), |value| format!("{value:.9}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(extra: &[&str]) -> Vec<String> {
        let mut args = vec![
            "fastlio-replay".to_string(),
            "bag.mcap".to_string(),
            "config.yaml".to_string(),
            "output".to_string(),
        ];
        args.extend(extra.iter().map(|arg| arg.to_string()));
        args
    }

    #[test]
    fn parse_replay_args_defaults_to_unlimited_no_viewer() {
        let parsed = ReplayArgs::parse(args(&[])).unwrap();

        assert_eq!(parsed.playback_rate, 0.0);
        assert_eq!(parsed.channel_capacity, 1024);
        assert_eq!(parsed.time_offset_lidar_to_imu_override, None);
        assert!(matches!(parsed.viewer, ReplayViewer::None));
    }

    #[test]
    fn parse_replay_args_keeps_positional_rate_and_capacity() {
        let parsed = ReplayArgs::parse(args(&["2.5", "64"])).unwrap();

        assert_eq!(parsed.playback_rate, 2.5);
        assert_eq!(parsed.channel_capacity, 64);
    }

    #[test]
    fn parse_replay_args_accepts_time_offset_override() {
        let parsed = ReplayArgs::parse(args(&["--time-offset-lidar-to-imu", "0.0095"])).unwrap();

        assert_eq!(parsed.time_offset_lidar_to_imu_override, Some(0.0095));
        assert!(matches!(parsed.viewer, ReplayViewer::None));
    }

    #[test]
    fn parse_replay_args_accepts_time_offset_with_viewer_options() {
        let parsed = ReplayArgs::parse(args(&[
            "--live-playground",
            "--time-offset-lidar-to-imu",
            "-0.002",
            "--playground",
            "/tmp/live-viewer",
        ]))
        .unwrap();

        assert_eq!(parsed.time_offset_lidar_to_imu_override, Some(-0.002));
        match parsed.viewer {
            ReplayViewer::LivePlayground { executable, .. } => {
                assert_eq!(executable, Utf8PathBuf::from("/tmp/live-viewer"));
            }
            _ => panic!("expected live playground viewer"),
        }
    }

    #[test]
    fn parse_replay_args_rejects_non_finite_time_offset() {
        assert!(ReplayArgs::parse(args(&["--time-offset-lidar-to-imu", "NaN"])).is_err());
    }

    #[test]
    fn parse_replay_args_accepts_playground_viewer() {
        let parsed = ReplayArgs::parse(args(&["--playground", "/tmp/viewer"])).unwrap();

        match parsed.viewer {
            ReplayViewer::OpenAfter { executable } => {
                assert_eq!(executable, Utf8PathBuf::from("/tmp/viewer"));
            }
            _ => panic!("expected playground viewer"),
        }
    }

    #[test]
    fn parse_replay_args_accepts_live_playground_viewer() {
        let parsed = ReplayArgs::parse(args(&[
            "--live-playground",
            "--live-addr",
            "127.0.0.1:9999",
            "--playground",
            "/tmp/live-viewer",
        ]))
        .unwrap();

        match parsed.viewer {
            ReplayViewer::LivePlayground {
                executable,
                listen_addr,
            } => {
                assert_eq!(executable, Utf8PathBuf::from("/tmp/live-viewer"));
                assert_eq!(listen_addr, "127.0.0.1:9999");
            }
            _ => panic!("expected live playground viewer"),
        }
    }

    #[test]
    fn parse_replay_args_rejects_unknown_option() {
        assert!(ReplayArgs::parse(args(&["--rerun"])).is_err());
    }
}
