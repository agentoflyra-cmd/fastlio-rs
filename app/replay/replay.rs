use anyhow::{Context, Result, anyhow, bail};
use camino::Utf8PathBuf;
use fastlio_dataset::{ReadStats, SensorEvent, read_mcap_events};
use fastlio_map::{surfel::SurfelMap, types::GeometryClass};
use fastlio_pipeline::{MainPipeline, synchronizer::MeasurementSynchronizer};
use fastlio_types::{NavState, read_from_config_path};
use pcd_rs::{DataKind, PcdSerialize, WriterInit};
use ringbuffer_spsc::{RingBufferReader, RingBufferWriter, ringbuffer};
use std::env;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::thread;
use std::time::{Duration, Instant};

enum ReplayMessage {
    Event(SensorEvent),
    Finished(Result<ReadStats, String>),
}

#[derive(Debug, Clone)]
struct ReplayConfig {
    playback_rate: f64,
    channel_capacity: usize,
    time_offset_lidar_to_imu_sec: f64,
    storm_mode: bool,
}

#[derive(Debug)]
struct ReplayArgs {
    bag_path: Utf8PathBuf,
    config_path: Utf8PathBuf,
    trajectory_path: Option<Utf8PathBuf>,
    surfel_map_path: Option<Utf8PathBuf>,
    playback_rate: f64,
    channel_capacity: usize,
    storm_mode: bool,
}

impl ReplayArgs {
    fn parse(args: &[String]) -> Result<Self> {
        if args.len() < 3 {
            bail!(
                "usage: {} <bag.mcap> <config.yaml> [playback_rate] [channel_capacity] [trajectory.csv] [surfel-map.pcd] [--storm]",
                args.first().map(String::as_str).unwrap_or("fastlio-replay")
            );
        }
        let storm_mode = args[3..].iter().any(|arg| arg == "--storm");
        if let Some(option) = args[3..]
            .iter()
            .find(|arg| arg.starts_with("--") && arg.as_str() != "--storm")
        {
            bail!("unknown replay option `{option}`");
        }
        let positional: Vec<_> = args[3..]
            .iter()
            .filter(|arg| arg.as_str() != "--storm")
            .collect();
        if positional.len() > 4 {
            bail!("too many replay arguments");
        }
        let playback_rate = positional
            .first()
            .map(|v| v.parse::<f64>())
            .transpose()
            .context("invalid playback_rate")?
            .unwrap_or(0.0);
        if !playback_rate.is_finite() || playback_rate < 0.0 {
            bail!("playback_rate must be finite and non-negative");
        }
        let channel_capacity = positional
            .get(1)
            .map(|v| v.parse::<usize>())
            .transpose()
            .context("invalid channel_capacity")?
            .unwrap_or(1024);
        if channel_capacity == 0 {
            bail!("channel_capacity must be positive");
        }
        Ok(Self {
            bag_path: Utf8PathBuf::from(&args[1]),
            config_path: Utf8PathBuf::from(&args[2]),
            trajectory_path: positional
                .get(2)
                .map(|path| Utf8PathBuf::from(path.as_str())),
            surfel_map_path: positional
                .get(3)
                .map(|path| Utf8PathBuf::from(path.as_str())),
            playback_rate,
            channel_capacity,
            storm_mode,
        })
    }
}

#[derive(PcdSerialize)]
struct SurfelPcdPoint {
    x: f32,
    y: f32,
    z: f32,
    intensity: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    class_id: f32,
}

#[derive(Clone)]
struct TrajectoryRow {
    timestamp_sec: f64,
    state: NavState,
}

#[derive(Default)]
struct ReplayStats {
    read: ReadStats,
    synchronized_groups: usize,
    processed_frames: usize,
    failed_groups: usize,
    max_pending_lidar: usize,
    dropped_lidar_before_first_imu: usize,
    pending_lidar_at_eof: usize,
    first_imu_raw_time_sec: Option<f64>,
    last_imu_raw_time_sec: Option<f64>,
    first_imu_time_sec: Option<f64>,
    last_imu_time_sec: Option<f64>,
    first_lidar_time_sec: Option<f64>,
    last_lidar_time_sec: Option<f64>,
    first_pipeline_error: Option<String>,
    trajectory: Vec<TrajectoryRow>,
}

struct ProducerAlive(Arc<AtomicBool>);

impl Drop for ProducerAlive {
    fn drop(&mut self) {
        self.0.store(false, Ordering::Release);
    }
}

fn push_blocking<T>(
    producer: &mut RingBufferWriter<T>,
    mut item: T,
    consumer_alive: &AtomicBool,
) -> Result<()> {
    loop {
        match producer.push(item) {
            None => return Ok(()),
            Some(returned) if consumer_alive.load(Ordering::Acquire) => {
                item = returned;
                thread::yield_now();
            }
            Some(_) => bail!("replay consumer stopped"),
        }
    }
}

fn produce_events(
    path: Utf8PathBuf,
    producer: &mut RingBufferWriter<ReplayMessage>,
    consumer_alive: Arc<AtomicBool>,
) -> Result<()> {
    let result = read_mcap_events(path, |event| {
        push_blocking(producer, ReplayMessage::Event(event), &consumer_alive)
    })
    .map_err(|error| format!("{error:#}"));
    push_blocking(producer, ReplayMessage::Finished(result), &consumer_alive)
}

struct PlaybackClock {
    rate: f64,
    first_sensor_time_sec: Option<f64>,
    wall_start: Instant,
}

impl PlaybackClock {
    fn new(rate: f64) -> Self {
        Self {
            rate,
            first_sensor_time_sec: None,
            wall_start: Instant::now(),
        }
    }

    fn wait_for(&mut self, sensor_time_sec: f64) {
        if self.rate == 0.0 || !sensor_time_sec.is_finite() {
            return;
        }
        let first = *self.first_sensor_time_sec.get_or_insert_with(|| {
            self.wall_start = Instant::now();
            sensor_time_sec
        });
        let target = Duration::from_secs_f64((sensor_time_sec - first).max(0.0) / self.rate);
        if let Some(remaining) = target.checked_sub(self.wall_start.elapsed()) {
            thread::sleep(remaining);
        }
    }
}

fn apply_time_offset(event: SensorEvent, offset_sec: f64, stats: &mut ReplayStats) -> SensorEvent {
    match event {
        SensorEvent::Imu(mut imu) => {
            let raw = imu.time_stamp_sec;
            stats.first_imu_raw_time_sec.get_or_insert(raw);
            stats.last_imu_raw_time_sec = Some(raw);
            imu.time_stamp_sec = raw - offset_sec;
            stats.first_imu_time_sec.get_or_insert(imu.time_stamp_sec);
            stats.last_imu_time_sec = Some(imu.time_stamp_sec);
            SensorEvent::Imu(imu)
        }
        SensorEvent::Lidar(lidar) => {
            stats
                .first_lidar_time_sec
                .get_or_insert(lidar.base_timestamp_sec);
            stats.last_lidar_time_sec = Some(lidar.base_timestamp_sec);
            SensorEvent::Lidar(lidar)
        }
    }
}

fn process_ready_groups(
    synchronizer: &mut MeasurementSynchronizer,
    pipeline: &mut MainPipeline,
    stats: &mut ReplayStats,
) {
    for group in synchronizer.drain_ready() {
        stats.synchronized_groups += 1;
        let scan_begin_sec = group.lidar.base_timestamp_sec;
        let timestamp_sec = group.lidar.end_timestamp_sec();
        let first_imu_sec = group.imu.first().map(|imu| imu.time_stamp_sec);
        let last_imu_sec = group.imu.last().map(|imu| imu.time_stamp_sec);
        let min_offset_sec = group
            .lidar
            .points
            .iter()
            .map(|point| point.offset_time_sec)
            .reduce(f64::min);
        let max_offset_sec = group
            .lidar
            .points
            .iter()
            .map(|point| point.offset_time_sec)
            .reduce(f64::max);
        let offset_regressions = group
            .lidar
            .points
            .windows(2)
            .filter(|pair| pair[1].offset_time_sec < pair[0].offset_time_sec)
            .count();
        match pipeline.process_measure_group(group) {
            Ok(_) => {
                stats.processed_frames += 1;
                stats.trajectory.push(TrajectoryRow {
                    timestamp_sec,
                    state: pipeline.filter.state.clone(),
                });
            }
            Err(error) => {
                stats.failed_groups += 1;
                stats
                    .first_pipeline_error
                    .get_or_insert_with(|| {
                        format!(
                            "scan=[{scan_begin_sec:.9},{timestamp_sec:.9}], imu=[{first_imu_sec:?},{last_imu_sec:?}], offset=[{min_offset_sec:?},{max_offset_sec:?}], offset_regressions={offset_regressions}: {error:#}"
                        )
                    });
            }
        }
    }
}

fn receive_events(
    consumer: &mut RingBufferReader<ReplayMessage>,
    producer_alive: &AtomicBool,
    synchronizer: &mut MeasurementSynchronizer,
    pipeline: &mut MainPipeline,
    config: &ReplayConfig,
) -> Result<ReplayStats> {
    let mut stats = ReplayStats::default();
    let mut clock = PlaybackClock::new(if config.storm_mode {
        0.0
    } else {
        config.playback_rate
    });
    loop {
        match consumer.pull() {
            Some(ReplayMessage::Event(event)) => {
                clock.wait_for(event.timestamp_sec());
                match apply_time_offset(event, config.time_offset_lidar_to_imu_sec, &mut stats) {
                    SensorEvent::Imu(imu) => synchronizer.pend_imu(imu)?,
                    SensorEvent::Lidar(lidar) => synchronizer.pend_lidar(lidar)?,
                }
                stats.max_pending_lidar =
                    stats.max_pending_lidar.max(synchronizer.lidar_buffer.len());
                process_ready_groups(synchronizer, pipeline, &mut stats);
            }
            Some(ReplayMessage::Finished(result)) => {
                process_ready_groups(synchronizer, pipeline, &mut stats);
                stats.read = result.map_err(anyhow::Error::msg)?;
                stats.dropped_lidar_before_first_imu = synchronizer.dropped_lidar_before_first_imu;
                stats.pending_lidar_at_eof = synchronizer.lidar_buffer.len();
                return Ok(stats);
            }
            None if !producer_alive.load(Ordering::Acquire) => {
                bail!("replay producer stopped without a completion message")
            }
            None => thread::yield_now(),
        }
    }
}

fn run_spsc(
    path: Utf8PathBuf,
    pipeline: &mut MainPipeline,
    config: ReplayConfig,
) -> Result<ReplayStats> {
    let (mut producer, mut consumer) = ringbuffer(config.channel_capacity);
    let producer_alive = Arc::new(AtomicBool::new(true));
    let consumer_alive = Arc::new(AtomicBool::new(true));
    let producer_guard = ProducerAlive(producer_alive.clone());
    let producer_consumer_alive = consumer_alive.clone();
    let handle = thread::spawn(move || {
        let _guard = producer_guard;
        produce_events(path, &mut producer, producer_consumer_alive)
    });
    let mut synchronizer = MeasurementSynchronizer::new();
    let result = receive_events(
        &mut consumer,
        &producer_alive,
        &mut synchronizer,
        pipeline,
        &config,
    );
    consumer_alive.store(false, Ordering::Release);
    let producer_result = handle
        .join()
        .map_err(|_| anyhow!("replay producer thread panicked"))?;
    match result {
        Ok(stats) => {
            producer_result?;
            Ok(stats)
        }
        Err(error) => Err(error),
    }
}

fn write_trajectory(path: &Utf8PathBuf, rows: &[TrajectoryRow]) -> Result<()> {
    let mut writer = BufWriter::new(
        File::create(path).with_context(|| format!("failed to create trajectory `{path}`"))?,
    );
    writeln!(writer, "timestamp_sec,px,py,pz,qx,qy,qz,qw,vx,vy,vz")?;
    for row in rows {
        let p = row.state.position;
        let v = row.state.velocity;
        let q = row.state.orientation.quaternion();
        writeln!(
            writer,
            "{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9}",
            row.timestamp_sec, p.x, p.y, p.z, q.i, q.j, q.k, q.w, v.x, v.y, v.z
        )?;
    }
    Ok(())
}

fn write_surfel_map(path: &Utf8PathBuf, map: &SurfelMap) -> Result<()> {
    let width = map.surfels().count();
    let config = map.surfel_config();
    let mut writer = WriterInit {
        width: width as u64,
        height: 1,
        viewpoint: Default::default(),
        data_kind: DataKind::Binary,
        schema: None,
        version: None,
    }
    .create::<SurfelPcdPoint, _>(path)
    .with_context(|| format!("failed to create surfel map PCD `{path}`"))?;

    for surfel in map.surfels() {
        let class_id = match surfel.geometry_class(config) {
            GeometryClass::Plane => 0.0,
            GeometryClass::Line => 1.0,
            GeometryClass::Scatter => 2.0,
            GeometryClass::Degenerate => 3.0,
            GeometryClass::Growing => 4.0,
        };
        writer.push(&SurfelPcdPoint {
            x: surfel.mean_w.x as f32,
            y: surfel.mean_w.y as f32,
            z: surfel.mean_w.z as f32,
            intensity: surfel.count as f32,
            normal_x: surfel.eigenvectors[(0, 0)] as f32,
            normal_y: surfel.eigenvectors[(1, 0)] as f32,
            normal_z: surfel.eigenvectors[(2, 0)] as f32,
            class_id,
        })?;
    }
    writer.finish()?;
    Ok(())
}

fn print_summary(stats: &ReplayStats, config: &ReplayConfig) {
    println!("replay complete");
    println!("  total messages: {}", stats.read.total_messages);
    println!("  emitted events: {}", stats.read.emitted_events);
    println!("  synchronized groups: {}", stats.synchronized_groups);
    println!("  processed frames: {}", stats.processed_frames);
    println!("  failed groups: {}", stats.failed_groups);
    println!("  max pending lidar: {}", stats.max_pending_lidar);
    println!(
        "  dropped lidar before first imu: {}",
        stats.dropped_lidar_before_first_imu
    );
    println!("  pending lidar at eof: {}", stats.pending_lidar_at_eof);
    println!("  storm mode: {}", config.storm_mode);
    println!(
        "  time offset lidar-to-imu: {:.9} s (t_imu_sync = t_imu_raw - offset)",
        config.time_offset_lidar_to_imu_sec
    );
    println!(
        "  imu time raw/shifted first: {:?} / {:?}",
        stats.first_imu_raw_time_sec, stats.first_imu_time_sec
    );
    println!(
        "  imu time raw/shifted last: {:?} / {:?}",
        stats.last_imu_raw_time_sec, stats.last_imu_time_sec
    );
    println!(
        "  lidar time first/last: {:?} / {:?}",
        stats.first_lidar_time_sec, stats.last_lidar_time_sec
    );
    if let Some(error) = &stats.first_pipeline_error {
        println!("  first pipeline error: {error}");
    }
}

fn main() -> Result<()> {
    let args = ReplayArgs::parse(&env::args().collect::<Vec<_>>())?;
    let pipeline_config = read_from_config_path(&args.config_path)
        .with_context(|| format!("failed to read config `{}`", args.config_path))?;
    let offset = pipeline_config
        .common
        .time_offset_lidar_to_imu
        .unwrap_or(0.0);
    if !offset.is_finite() {
        bail!("time_offset_lidar_to_imu must be finite");
    }
    let replay_config = ReplayConfig {
        playback_rate: args.playback_rate,
        channel_capacity: args.channel_capacity,
        time_offset_lidar_to_imu_sec: offset,
        storm_mode: args.storm_mode,
    };
    let mut pipeline = MainPipeline::new(pipeline_config);
    let stats = run_spsc(args.bag_path, &mut pipeline, replay_config.clone())?;
    if let Some(path) = &args.trajectory_path {
        write_trajectory(path, &stats.trajectory)?;
    }
    if let Some(path) = &args.surfel_map_path {
        write_surfel_map(path, &pipeline.map)?;
    }
    print_summary(&stats, &replay_config);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use fastlio_types::{ImuSample, LidarFrame, Vec3};

    fn imu_event(t: f64) -> SensorEvent {
        SensorEvent::Imu(ImuSample {
            time_stamp_sec: t,
            gyro: Vec3::zeros(),
            accel: Vec3::zeros(),
        })
    }

    #[test]
    fn parse_args_defaults_to_unlimited_playback() {
        let args = vec!["replay".into(), "bag.mcap".into(), "config.yaml".into()];
        let parsed = ReplayArgs::parse(&args).unwrap();
        assert_eq!(parsed.playback_rate, 0.0);
        assert_eq!(parsed.channel_capacity, 1024);
        assert!(!parsed.storm_mode);
        assert!(parsed.trajectory_path.is_none());
        assert!(parsed.surfel_map_path.is_none());
    }

    #[test]
    fn parse_args_accepts_storm_mode_with_positional_arguments() {
        let args = vec![
            "replay".into(),
            "bag.mcap".into(),
            "config.yaml".into(),
            "2.0".into(),
            "64".into(),
            "trajectory.csv".into(),
            "surfel-map.pcd".into(),
            "--storm".into(),
        ];
        let parsed = ReplayArgs::parse(&args).unwrap();
        assert!(parsed.storm_mode);
        assert_eq!(parsed.playback_rate, 2.0);
        assert_eq!(parsed.channel_capacity, 64);
        assert_eq!(
            parsed.surfel_map_path.as_deref(),
            Some("surfel-map.pcd".into())
        );
    }

    #[test]
    fn parse_args_rejects_invalid_rate_and_capacity() {
        let bad_rate = vec!["replay".into(), "bag".into(), "config".into(), "NaN".into()];
        assert!(ReplayArgs::parse(&bad_rate).is_err());
        let bad_capacity = vec![
            "replay".into(),
            "bag".into(),
            "config".into(),
            "0".into(),
            "0".into(),
        ];
        assert!(ReplayArgs::parse(&bad_capacity).is_err());
    }

    #[test]
    fn imu_offset_shifts_only_imu_timestamp() {
        let mut stats = ReplayStats::default();
        match apply_time_offset(imu_event(10.0), 0.25, &mut stats) {
            SensorEvent::Imu(imu) => assert_eq!(imu.time_stamp_sec, 9.75),
            SensorEvent::Lidar(_) => panic!("expected IMU event"),
        }
        assert_eq!(stats.first_imu_raw_time_sec, Some(10.0));
        assert_eq!(stats.first_imu_time_sec, Some(9.75));
        match apply_time_offset(
            SensorEvent::Lidar(LidarFrame::new(20.0, 20.1, vec![])),
            0.25,
            &mut stats,
        ) {
            SensorEvent::Lidar(frame) => assert_eq!(frame.base_timestamp_sec, 20.0),
            SensorEvent::Imu(_) => panic!("expected LiDAR event"),
        }
    }

    #[test]
    fn blocking_push_preserves_items() {
        let (mut producer, mut consumer) = ringbuffer(1);
        let alive = AtomicBool::new(true);
        push_blocking(&mut producer, 1, &alive).unwrap();
        assert_eq!(consumer.pull(), Some(1));
        push_blocking(&mut producer, 2, &alive).unwrap();
        assert_eq!(consumer.pull(), Some(2));
    }

    #[test]
    fn blocking_push_stops_after_consumer_shutdown() {
        let (mut producer, _consumer) = ringbuffer(1);
        let dead = AtomicBool::new(false);
        assert!(producer.push(1).is_none());
        assert!(push_blocking(&mut producer, 2, &dead).is_err());
    }
}
