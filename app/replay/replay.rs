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
use fastlio_map::{LocalMap, PointToPlaneConfig, surfel::SurfelMap};
use fastlio_pipeline::main_pipeline::{
    FastLioPipeline, PipelineAssociationStats, PipelineConfig, PipelineMode, PipelineStageTimings,
};
use fastlio_pipeline::synchronizer::MeasurementSynchronizer;
use fastlio_types::{
    Config, LidarImuExtrinsic, NavState, PointXYZI, Pose3, SurfelConfig, Vec3,
    read_from_config_path,
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
    args.surfel_options.apply_to_config(&mut config)?;
    let mut replay = OfflineReplay::new(config, args.map_backend)?;
    replay.live_stream = start_live_viewer(&args.viewer)?;
    let stats = run_spsc_replay(&args, &mut replay)
        .with_context(|| format!("failed to replay MCAP `{}`", args.bag_path))?;

    let trajectory_path = args.output_dir.join("trajectory.csv");
    let latency_path = args.output_dir.join("latency.csv");
    let map_path = args.output_dir.join("map.pcd");
    let summary_path = args.output_dir.join("summary.txt");

    write_trajectory_csv(&trajectory_path, &replay.trajectory)?;
    write_latency_csv(&latency_path, &replay.latency)?;
    let map_output_points = replay.pipeline.local_map.output_points();
    write_ascii_pcd(&map_path, &map_output_points)?;
    write_summary(&summary_path, stats, &replay)?;
    open_viewer(&args.viewer, &map_path)?;

    println!("replay complete");
    println!("  processed frames: {}", replay.processed_frames);
    println!("  initializing frames: {}", replay.initializing_frames);
    println!("  tracking frames: {}", replay.tracking_frames);
    println!("  tracking lost frames: {}", replay.tracking_lost_frames);
    println!("  bootstrap frames: {}", replay.bootstrap_frames);
    println!("  failed groups: {}", replay.failed_groups);
    println!("  map points: {}", replay.pipeline.local_map.len());
    println!("  trajectory: {trajectory_path}");
    println!("  latency: {latency_path}");
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
    map_backend: ReplayMapBackend,
    surfel_options: ReplaySurfelOptions,
}

impl ReplayArgs {
    fn parse(args: Vec<String>) -> Result<Self> {
        if args.len() < 4 {
            bail!(
                "usage: {} <bag.mcap> <config.yaml> <output_dir> [playback_rate] [channel_capacity] [--time-offset-lidar-to-imu <sec>] [--map-backend kiddo|surfel] [--surfel-allow-growing-constraints] [--surfel-growing-constraint-weight <weight>] [--open-playground|--live-playground|--playground <path>|--live-addr <addr>]",
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
        let (viewer_args, time_offset_lidar_to_imu_override, map_backend, surfel_options) =
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
            map_backend,
            surfel_options,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ReplayMapBackend {
    Kiddo,
    Surfel,
}

impl ReplayMapBackend {
    fn parse(value: &str) -> Result<Self> {
        match value {
            "kiddo" | "local" | "local-map" => Ok(Self::Kiddo),
            "surfel" | "surfel-map" => Ok(Self::Surfel),
            _ => bail!("unknown --map-backend `{value}`; expected kiddo or surfel"),
        }
    }

    fn as_str(self) -> &'static str {
        match self {
            Self::Kiddo => "kiddo",
            Self::Surfel => "surfel",
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct ReplaySurfelOptions {
    allow_growing_constraints: bool,
    growing_constraint_weight: Option<f32>,
}

impl ReplaySurfelOptions {
    fn apply_to_config(self, config: &mut Config) -> Result<()> {
        if !self.allow_growing_constraints && self.growing_constraint_weight.is_none() {
            return Ok(());
        }
        let surfel_config = config
            .surfel_config
            .get_or_insert_with(SurfelConfig::default);
        if self.allow_growing_constraints {
            surfel_config.allow_growing_constraints = true;
        }
        if let Some(weight) = self.growing_constraint_weight {
            if !weight.is_finite() || weight < 0.0 {
                bail!("--surfel-growing-constraint-weight must be finite and non-negative");
            }
            surfel_config.growing_constraint_weight = weight;
        }
        Ok(())
    }
}

fn parse_replay_options(
    args: &[String],
) -> Result<(
    Vec<String>,
    Option<f64>,
    ReplayMapBackend,
    ReplaySurfelOptions,
)> {
    let mut viewer_args = Vec::new();
    let mut time_offset_lidar_to_imu_override = None;
    let mut map_backend = ReplayMapBackend::Kiddo;
    let mut surfel_options = ReplaySurfelOptions::default();
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
            "--map-backend" => {
                let Some(value) = args.get(idx + 1) else {
                    bail!("--map-backend requires `kiddo` or `surfel`");
                };
                map_backend = ReplayMapBackend::parse(value)?;
                idx += 2;
            }
            "--surfel-allow-growing-constraints" => {
                surfel_options.allow_growing_constraints = true;
                idx += 1;
            }
            "--surfel-growing-constraint-weight" => {
                let Some(value) = args.get(idx + 1) else {
                    bail!("--surfel-growing-constraint-weight requires a weight value");
                };
                let weight = value.parse::<f32>().with_context(|| {
                    format!("invalid --surfel-growing-constraint-weight `{value}`")
                })?;
                surfel_options.growing_constraint_weight = Some(weight);
                idx += 2;
            }
            _ => {
                viewer_args.push(args[idx].clone());
                idx += 1;
            }
        }
    }

    Ok((
        viewer_args,
        time_offset_lidar_to_imu_override,
        map_backend,
        surfel_options,
    ))
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

#[derive(Debug, Clone)]
struct LatencyRow {
    frame_index: usize,
    timestamp_sec: f64,
    mode: PipelineMode,
    effective_observations: usize,
    map_points: usize,
    end_to_end: Duration,
    live_stream: Duration,
    pipeline: PipelineStageTimings,
    association_stats: PipelineAssociationStats,
}

struct OfflineReplay {
    synchronizer: MeasurementSynchronizer,
    pipeline: FastLioPipeline,
    /// IMU absolute timestamps are shifted into the LiDAR clock domain as:
    /// `t_imu_for_sync = t_imu_raw - time_offset_lidar_to_imu_sec`.
    time_offset_lidar_to_imu_sec: f64,
    trajectory: Vec<TrajectoryRow>,
    latency: Vec<LatencyRow>,
    processed_frames: usize,
    initializing_frames: usize,
    tracking_frames: usize,
    tracking_lost_frames: usize,
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
    fn new(config: Config, map_backend: ReplayMapBackend) -> Result<Self> {
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
            // Frame-to-frame displacement is physical motion (speed * dt), so a
            // displacement gate is always overtaken by real vehicle speed
            // (1.0 m at 10 m/s, 3.0 m at 30 m/s on aneng). Update sanity is
            // enforced solely by the correction gates below, which constrain
            // predicted->updated change (error), not previous->updated motion.
            max_tracking_translation_step: None,
            max_tracking_rotation_step_rad: Some(20.0_f64.to_radians()),
            max_update_translation_correction: Some(0.3),
            max_update_rotation_correction_rad: Some(10.0_f64.to_radians()),
            map_crop_radius: Some(100.0),
            insert_scan_points: true,
            max_factor_points: Some(2_000),
            max_map_insert_points: Some(5_000),
            map_insert_min_distance: Some(0.10),
            initialization_groups: 10,
            // Reassociate once at the corrected pose: recovers points missed
            // at the predicted pose (pose error above the strict tolerance)
            // without globally relaxing the match tolerance.
            max_reassociation_passes: 1,
        };

        let pipeline = match map_backend {
            ReplayMapBackend::Kiddo => FastLioPipeline::new(
                filter,
                LocalMap::new(),
                imu_integrator,
                extrinsic,
                pipeline_config,
            ),
            ReplayMapBackend::Surfel => FastLioPipeline::new_with_surfel_map(
                filter,
                SurfelMap::new(
                    config.surfel_map_config.unwrap_or_default(),
                    config.surfel_config.unwrap_or_default(),
                ),
                imu_integrator,
                extrinsic,
                pipeline_config,
            ),
        };

        Ok(Self {
            synchronizer: MeasurementSynchronizer::new(),
            pipeline,
            time_offset_lidar_to_imu_sec,
            trajectory: Vec::new(),
            latency: Vec::new(),
            processed_frames: 0,
            initializing_frames: 0,
            tracking_frames: 0,
            tracking_lost_frames: 0,
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
        let end_to_end_start = Instant::now();
        match self.pipeline.process_measurement_group(group) {
            Ok(report) => {
                self.processed_frames += 1;
                match report.mode {
                    PipelineMode::Initializing => self.initializing_frames += 1,
                    PipelineMode::BootstrapMap => self.bootstrap_frames += 1,
                    PipelineMode::Tracking => self.tracking_frames += 1,
                    PipelineMode::TrackingLost => self.tracking_lost_frames += 1,
                }
                self.push_trajectory_row(timestamp_sec, report.mode, report.effective_observations);
                let live_stream_start = Instant::now();
                if let Some(live_stream) = &mut self.live_stream
                    && let Err(err) =
                        live_stream.send_map_delta(&self.pipeline.local_map.output_points())
                {
                    eprintln!("live viewer stream failed at {timestamp_sec:.6}: {err:#}");
                    self.live_stream = None;
                }
                let live_stream_duration = live_stream_start.elapsed();
                self.latency.push(LatencyRow {
                    frame_index: self.processed_frames,
                    timestamp_sec,
                    mode: report.mode,
                    effective_observations: report.effective_observations,
                    map_points: self.pipeline.local_map.len(),
                    end_to_end: end_to_end_start.elapsed(),
                    live_stream: live_stream_duration,
                    pipeline: report.timings,
                    association_stats: report.association_stats,
                });
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

fn write_latency_csv(path: &Utf8Path, latency: &[LatencyRow]) -> Result<()> {
    let mut writer = BufWriter::new(
        File::create(path).with_context(|| format!("failed to create latency `{path}`"))?,
    );
    writeln!(
        writer,
        "frame_index,timestamp_sec,mode,effective_observations,map_points,end_to_end_ms,live_stream_ms,pipeline_total_ms,imu_boundary_ms,initialization_ms,motion_segments_ms,predict_ms,deskew_ms,preprocess_ms,association_ms,association_nearest_ms,association_plane_fit_ms,association_residual_ms,association_factor_build_ms,update_ms,map_insert_ms,map_crop_ms,association_sampled_points,association_accepted_observations,association_non_finite_scan_points,association_invalid_configs,association_no_planar_surfel,association_neighbour_too_far,association_plane_fit_errors,association_residual_too_large,surfel_primary_raw_candidates,surfel_primary_unique_candidates,surfel_fallback_raw_candidates,surfel_fallback_unique_candidates,surfel_fallback_queries,surfel_fallback_hits,surfel_planar_candidates,surfel_growing_candidates,surfel_accepted_growing_weak,association_normal_mean_x,association_normal_mean_y,association_normal_mean_z"
    )?;
    for row in latency {
        let accepted = row.association_stats.accepted_observations as f64;
        let (normal_mean_x, normal_mean_y, normal_mean_z) = if accepted > 0.0 {
            (
                row.association_stats.normal_sum_x / accepted,
                row.association_stats.normal_sum_y / accepted,
                row.association_stats.normal_sum_z / accepted,
            )
        } else {
            (0.0, 0.0, 0.0)
        };
        writeln!(
            writer,
            "{},{:.9},{:?},{},{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{:.9},{:.9},{:.9}",
            row.frame_index,
            row.timestamp_sec,
            row.mode,
            row.effective_observations,
            row.map_points,
            duration_ms(row.end_to_end),
            duration_ms(row.live_stream),
            duration_ms(row.pipeline.total),
            duration_ms(row.pipeline.imu_boundary),
            duration_ms(row.pipeline.initialization),
            duration_ms(row.pipeline.motion_segments),
            duration_ms(row.pipeline.predict),
            duration_ms(row.pipeline.deskew),
            duration_ms(row.pipeline.preprocess),
            duration_ms(row.pipeline.association),
            duration_ms(row.pipeline.association_nearest),
            duration_ms(row.pipeline.association_plane_fit),
            duration_ms(row.pipeline.association_residual),
            duration_ms(row.pipeline.association_factor_build),
            duration_ms(row.pipeline.update),
            duration_ms(row.pipeline.map_insert),
            duration_ms(row.pipeline.map_crop),
            row.association_stats.sampled_points,
            row.association_stats.accepted_observations,
            row.association_stats.non_finite_scan_points,
            row.association_stats.invalid_configs,
            row.association_stats.no_planar_surfel,
            row.association_stats.neighbour_too_far,
            row.association_stats.plane_fit_errors,
            row.association_stats.residual_too_large,
            row.association_stats.surfel_primary_raw_candidates,
            row.association_stats.surfel_primary_unique_candidates,
            row.association_stats.surfel_fallback_raw_candidates,
            row.association_stats.surfel_fallback_unique_candidates,
            row.association_stats.surfel_fallback_queries,
            row.association_stats.surfel_fallback_hits,
            row.association_stats.surfel_planar_candidates,
            row.association_stats.surfel_growing_candidates,
            row.association_stats.surfel_accepted_growing_weak,
            normal_mean_x,
            normal_mean_y,
            normal_mean_z,
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
    writeln!(writer, "initializing_frames={}", replay.initializing_frames)?;
    writeln!(writer, "tracking_frames={}", replay.tracking_frames)?;
    writeln!(
        writer,
        "tracking_lost_frames={}",
        replay.tracking_lost_frames
    )?;
    writeln!(writer, "bootstrap_frames={}", replay.bootstrap_frames)?;
    writeln!(writer, "failed_groups={}", replay.failed_groups)?;
    writeln!(writer, "max_pending_lidar={}", replay.max_pending_lidar)?;
    writeln!(
        writer,
        "map_backend={}",
        replay.pipeline.local_map.backend_name()
    )?;
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
    if replay.pipeline.local_map.backend_name() == ReplayMapBackend::Surfel.as_str() {
        writeln!(
            writer,
            "map_output_points={}",
            replay.pipeline.local_map.output_points().len()
        )?;
        writeln!(
            writer,
            "map_output_semantics=surfel_centroids_with_intensity_as_surfel_point_count"
        )?;
    }
    write_latency_summary(&mut writer, replay)?;
    writeln!(writer)?;
    writeln!(writer, "minimal_implementation_notes=")?;
    writeln!(
        writer,
        "- SPSC replay is implemented with a bounded std::sync::mpsc::sync_channel and optional sensor-time playback_rate"
    )?;
    writeln!(
        writer,
        "- replay limits scan-to-map association to 2000 deterministic scan samples and map insertion to 5000 deterministic scan samples per frame, then rejects map points within 0.10m of existing map points"
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
        "- the first 10 synchronized groups initialize IMU gravity, gyro bias, and acceleration scale before map insertion"
    )?;
    writeln!(
        writer,
        "- map backend defaults to kiddo; pass --map-backend surfel to test the surfel compression backend on the same replay pipeline"
    )?;
    writeln!(
        writer,
        "- map output is a single ASCII PCD; surfel backend writes surfel centroids instead of the dense inserted point pool"
    )?;
    writeln!(
        writer,
        "- failed synchronized frames are counted and skipped; no retry or queue-based recovery yet"
    )?;
    writeln!(
        writer,
        "- after tracking is established, low-observation or gate-rejected frames enter TrackingLost and skip map insertion; with sufficient observations and a passing acceptance gate the next frame recovers tracking and resumes map insertion"
    )?;
    Ok(())
}

fn format_optional_f64(value: Option<f64>) -> String {
    value.map_or_else(|| "none".to_string(), |value| format!("{value:.9}"))
}

fn write_latency_summary(writer: &mut impl Write, replay: &OfflineReplay) -> Result<()> {
    writeln!(writer, "latency_rows={}", replay.latency.len())?;
    write_latency_metric(
        writer,
        "latency_end_to_end_ms",
        replay.latency.iter().map(|row| duration_ms(row.end_to_end)),
    )?;
    write_latency_metric(
        writer,
        "latency_live_stream_ms",
        replay
            .latency
            .iter()
            .map(|row| duration_ms(row.live_stream)),
    )?;
    write_latency_metric(
        writer,
        "latency_pipeline_total_ms",
        replay
            .latency
            .iter()
            .map(|row| duration_ms(row.pipeline.total)),
    )?;
    write_latency_metric(
        writer,
        "latency_imu_boundary_ms",
        replay
            .latency
            .iter()
            .map(|row| duration_ms(row.pipeline.imu_boundary)),
    )?;
    write_latency_metric(
        writer,
        "latency_initialization_ms",
        replay
            .latency
            .iter()
            .map(|row| duration_ms(row.pipeline.initialization)),
    )?;
    write_latency_metric(
        writer,
        "latency_motion_segments_ms",
        replay
            .latency
            .iter()
            .map(|row| duration_ms(row.pipeline.motion_segments)),
    )?;
    write_latency_metric(
        writer,
        "latency_predict_ms",
        replay
            .latency
            .iter()
            .map(|row| duration_ms(row.pipeline.predict)),
    )?;
    write_latency_metric(
        writer,
        "latency_deskew_ms",
        replay
            .latency
            .iter()
            .map(|row| duration_ms(row.pipeline.deskew)),
    )?;
    write_latency_metric(
        writer,
        "latency_preprocess_ms",
        replay
            .latency
            .iter()
            .map(|row| duration_ms(row.pipeline.preprocess)),
    )?;
    write_latency_metric(
        writer,
        "latency_association_ms",
        replay
            .latency
            .iter()
            .map(|row| duration_ms(row.pipeline.association)),
    )?;
    write_latency_metric(
        writer,
        "latency_association_nearest_ms",
        replay
            .latency
            .iter()
            .map(|row| duration_ms(row.pipeline.association_nearest)),
    )?;
    write_latency_metric(
        writer,
        "latency_association_plane_fit_ms",
        replay
            .latency
            .iter()
            .map(|row| duration_ms(row.pipeline.association_plane_fit)),
    )?;
    write_latency_metric(
        writer,
        "latency_association_residual_ms",
        replay
            .latency
            .iter()
            .map(|row| duration_ms(row.pipeline.association_residual)),
    )?;
    write_latency_metric(
        writer,
        "latency_association_factor_build_ms",
        replay
            .latency
            .iter()
            .map(|row| duration_ms(row.pipeline.association_factor_build)),
    )?;
    write_latency_metric(
        writer,
        "latency_update_ms",
        replay
            .latency
            .iter()
            .map(|row| duration_ms(row.pipeline.update)),
    )?;
    write_latency_metric(
        writer,
        "latency_map_insert_ms",
        replay
            .latency
            .iter()
            .map(|row| duration_ms(row.pipeline.map_insert)),
    )?;
    write_latency_metric(
        writer,
        "latency_map_crop_ms",
        replay
            .latency
            .iter()
            .map(|row| duration_ms(row.pipeline.map_crop)),
    )?;
    Ok(())
}

fn write_latency_metric(
    writer: &mut impl Write,
    name: &str,
    values: impl Iterator<Item = f64>,
) -> Result<()> {
    let stats = LatencyStats::from_values(values);
    if let Some(stats) = stats {
        writeln!(
            writer,
            "{name}=count:{},min:{:.6},mean:{:.6},p50:{:.6},p95:{:.6},max:{:.6}",
            stats.count, stats.min, stats.mean, stats.p50, stats.p95, stats.max
        )?;
    } else {
        writeln!(writer, "{name}=none")?;
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct LatencyStats {
    count: usize,
    min: f64,
    mean: f64,
    p50: f64,
    p95: f64,
    max: f64,
}

impl LatencyStats {
    fn from_values(values: impl Iterator<Item = f64>) -> Option<Self> {
        let mut values: Vec<_> = values.filter(|value| value.is_finite()).collect();
        if values.is_empty() {
            return None;
        }
        values.sort_by(f64::total_cmp);
        let count = values.len();
        let sum: f64 = values.iter().sum();
        Some(Self {
            count,
            min: values[0],
            mean: sum / count as f64,
            p50: percentile_sorted(&values, 0.50),
            p95: percentile_sorted(&values, 0.95),
            max: values[count - 1],
        })
    }
}

fn percentile_sorted(values: &[f64], percentile: f64) -> f64 {
    debug_assert!(!values.is_empty());
    let max_index = values.len() - 1;
    let index = ((max_index as f64) * percentile).ceil() as usize;
    values[index.min(max_index)]
}

fn duration_ms(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1_000.0
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
        assert_eq!(parsed.map_backend, ReplayMapBackend::Kiddo);
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
    fn parse_replay_args_accepts_surfel_map_backend() {
        let parsed = ReplayArgs::parse(args(&["--map-backend", "surfel"])).unwrap();

        assert_eq!(parsed.map_backend, ReplayMapBackend::Surfel);
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

    #[test]
    fn latency_stats_sort_and_ignore_non_finite_values() {
        let stats = LatencyStats::from_values([5.0, f64::NAN, 1.0, 3.0, 2.0].into_iter())
            .expect("finite latency stats");

        assert_eq!(stats.count, 4);
        assert_eq!(stats.min, 1.0);
        assert_eq!(stats.mean, 2.75);
        assert_eq!(stats.p50, 3.0);
        assert_eq!(stats.p95, 5.0);
        assert_eq!(stats.max, 5.0);
    }
}
