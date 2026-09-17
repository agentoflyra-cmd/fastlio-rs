use anyhow::{Context, Result, anyhow, bail};
use camino::Utf8PathBuf;
use fastlio_dataset::{ReadStats, SensorEvent, read_mcap_events};
use fastlio_estimator::optimizer::SurfelMeasurementSpectrumMode;
use fastlio_map::{
    surfel::{SurfelMap, SurfelRankMode},
    types::GeometryClass,
};
use fastlio_pipeline::{MainPipeline, PipelineFrameSummary, synchronizer::MeasurementSynchronizer};
use fastlio_types::{NavState, read_from_config_path};
use pcd_rs::{DataKind, PcdSerialize, WriterInit};
use ringbuffer_spsc::{RingBufferReader, RingBufferWriter, ringbuffer};
use std::collections::{HashSet, VecDeque};
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
    surfel_rank_mode: Option<SurfelRankMode>,
    surfel_measurement_spectrum_mode: Option<SurfelMeasurementSpectrumMode>,
    rhs_visualization_window: Option<(f64, f64)>,
}

impl ReplayArgs {
    fn parse(args: &[String]) -> Result<Self> {
        if args.len() < 3 {
            bail!(
                "usage: {} <bag.mcap> <config.yaml> [playback_rate] [channel_capacity] [trajectory.csv] [surfel-map.pcd] [--storm] [--surfel-rank=mahalanobis|euclidean|combined:<beta>] [--surfel-update=full|hard:<tau>] [--rhs-visualization-window=<start_sec>:<end_sec>]",
                args.first().map(String::as_str).unwrap_or("fastlio-replay")
            );
        }
        let mut storm_mode = false;
        let mut surfel_rank_mode = None;
        let mut surfel_measurement_spectrum_mode = None;
        let mut rhs_visualization_window = None;
        let mut positional = Vec::new();
        for arg in &args[3..] {
            if arg == "--storm" {
                storm_mode = true;
            } else if let Some(value) = arg.strip_prefix("--surfel-rank=") {
                if surfel_rank_mode.is_some() {
                    bail!("--surfel-rank may only be specified once");
                }
                surfel_rank_mode = Some(parse_surfel_rank_mode(value)?);
            } else if let Some(value) = arg.strip_prefix("--surfel-update=") {
                if surfel_measurement_spectrum_mode.is_some() {
                    bail!("--surfel-update may only be specified once");
                }
                surfel_measurement_spectrum_mode =
                    Some(parse_surfel_measurement_spectrum_mode(value)?);
            } else if let Some(value) = arg.strip_prefix("--rhs-visualization-window=") {
                if rhs_visualization_window.is_some() {
                    bail!("--rhs-visualization-window may only be specified once");
                }
                rhs_visualization_window = Some(parse_rhs_visualization_window(value)?);
            } else if arg.starts_with("--") {
                bail!("unknown replay option `{arg}`");
            } else {
                positional.push(arg);
            }
        }
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
            surfel_rank_mode,
            surfel_measurement_spectrum_mode,
            rhs_visualization_window,
        })
    }
}

fn parse_rhs_visualization_window(value: &str) -> Result<(f64, f64)> {
    let Some((start, end)) = value.split_once(':') else {
        bail!("--rhs-visualization-window must be <start_sec>:<end_sec>");
    };
    let start = start
        .parse::<f64>()
        .context("invalid RHS visualization start time")?;
    let end = end
        .parse::<f64>()
        .context("invalid RHS visualization end time")?;
    if !start.is_finite() || !end.is_finite() || end <= start {
        bail!("RHS visualization window requires finite end_sec > start_sec");
    }
    Ok((start, end))
}

fn parse_surfel_measurement_spectrum_mode(value: &str) -> Result<SurfelMeasurementSpectrumMode> {
    if value == "full" {
        return Ok(SurfelMeasurementSpectrumMode::FullRank);
    }
    let Some(tau) = value.strip_prefix("hard:") else {
        bail!("invalid --surfel-update `{value}`");
    };
    let max_variance_ratio = tau
        .parse::<f64>()
        .context("hard update tau must be a number")?;
    if !max_variance_ratio.is_finite() || max_variance_ratio <= 1.0 {
        bail!("hard update tau must be finite and greater than 1");
    }
    Ok(SurfelMeasurementSpectrumMode::HardTruncation { max_variance_ratio })
}

fn parse_surfel_rank_mode(value: &str) -> Result<SurfelRankMode> {
    match value {
        "mahalanobis" => Ok(SurfelRankMode::Mahalanobis),
        "euclidean" => Ok(SurfelRankMode::EuclideanSquared),
        _ => {
            let Some(weight) = value.strip_prefix("combined:") else {
                bail!("invalid --surfel-rank `{value}`");
            };
            let centroid_distance_weight = weight
                .parse::<f64>()
                .context("combined rank beta must be a number")?;
            if !centroid_distance_weight.is_finite() || centroid_distance_weight < 0.0 {
                bail!("combined rank beta must be finite and non-negative");
            }
            Ok(SurfelRankMode::Combined {
                centroid_distance_weight,
            })
        }
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

#[derive(PcdSerialize)]
struct ColoredSurfelPcdPoint {
    x: f32,
    y: f32,
    z: f32,
    rgb: u32,
}

#[derive(Clone)]
struct TrajectoryRow {
    timestamp_sec: f64,
    state: NavState,
    frame_summary: PipelineFrameSummary,
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
            Ok(frame_summary) => {
                stats.processed_frames += 1;
                stats.trajectory.push(TrajectoryRow {
                    timestamp_sec,
                    state: pipeline.filter.state.clone(),
                    frame_summary,
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
    write!(
        writer,
        "timestamp_sec,px,py,pz,qx,qy,qz,qw,vx,vy,vz,tracking,iekf_iterations,iekf_observations_first,iekf_observations_last,iekf_mean_abs_residual,iekf_max_abs_residual,iekf_final_rotation_delta_norm,iekf_final_position_delta_norm,iekf_final_velocity_delta_norm,iekf_final_accel_bias_delta_norm,iekf_final_gravity_delta_norm,obs_input_points,obs_accepted,obs_no_association,obs_residual_abs_mean,obs_residual_abs_p50,obs_residual_abs_p90,obs_residual_abs_p95,obs_residual_abs_p99,obs_residual_abs_max,surfel_query_best_score_p50,surfel_query_best_score_p95,surfel_query_second_best_score_p50,surfel_query_second_best_score_p95,surfel_query_second_best_count,surfel_query_score_margin_p05,surfel_query_score_margin_p50,surfel_query_best_over_second_p95,surfel_query_ambiguous_fraction,obs_rotation_info_eigenvalue_0,obs_rotation_info_eigenvalue_1,obs_rotation_info_eigenvalue_2,obs_rotation_info_eigenvector_r0_c0,obs_rotation_info_eigenvector_r0_c1,obs_rotation_info_eigenvector_r0_c2,obs_rotation_info_eigenvector_r1_c0,obs_rotation_info_eigenvector_r1_c1,obs_rotation_info_eigenvector_r1_c2,obs_rotation_info_eigenvector_r2_c0,obs_rotation_info_eigenvector_r2_c1,obs_rotation_info_eigenvector_r2_c2,obs_rotation_info_condition_number,obs_rotation_info_trace,obs_rotation_position_info_r0_c0,obs_rotation_position_info_r0_c1,obs_rotation_position_info_r0_c2,obs_rotation_position_info_r1_c0,obs_rotation_position_info_r1_c1,obs_rotation_position_info_r1_c2,obs_rotation_position_info_r2_c0,obs_rotation_position_info_r2_c1,obs_rotation_position_info_r2_c2,obs_measurement_spectrum_rotation_trace_0,obs_measurement_spectrum_rotation_trace_1,obs_measurement_spectrum_rotation_trace_2,obs_measurement_spectrum_rotation_trace_fraction_0,obs_measurement_spectrum_rotation_trace_fraction_1,obs_measurement_spectrum_rotation_trace_fraction_2,obs_measurement_spectrum_eigenvalue_mean_0,obs_measurement_spectrum_eigenvalue_mean_1,obs_measurement_spectrum_eigenvalue_mean_2,obs_measurement_spectrum_surfel_axis_alignment_0,obs_measurement_spectrum_surfel_axis_alignment_1,obs_measurement_spectrum_surfel_axis_alignment_2,obs_measurement_spectrum_k0_axis_bin_fraction_x,obs_measurement_spectrum_k0_axis_bin_fraction_y,obs_measurement_spectrum_k0_axis_bin_fraction_z,obs_measurement_spectrum_k0_axis_concentration,obs_measurement_spectrum_k0_axis_dominant_world_0,obs_measurement_spectrum_k0_axis_dominant_world_1,obs_measurement_spectrum_k0_axis_dominant_world_2,obs_first_rotation_measurement_rhs_imu_0,obs_first_rotation_measurement_rhs_imu_1,obs_first_rotation_measurement_rhs_imu_2,obs_rotation_measurement_rhs_imu_0,obs_rotation_measurement_rhs_imu_1,obs_rotation_measurement_rhs_imu_2,obs_first_rotation_measurement_rhs_world_0,obs_first_rotation_measurement_rhs_world_1,obs_first_rotation_measurement_rhs_world_2,obs_rotation_measurement_rhs_world_0,obs_rotation_measurement_rhs_world_1,obs_rotation_measurement_rhs_world_2,obs_first_rotation_rhs_world_window_1s_0,obs_first_rotation_rhs_world_window_1s_1,obs_first_rotation_rhs_world_window_1s_2,obs_first_rotation_rhs_world_window_1s_count,obs_first_rotation_rhs_info_eigenbasis_0,obs_first_rotation_rhs_info_eigenbasis_1,obs_first_rotation_rhs_info_eigenbasis_2,obs_rotation_rhs_info_eigenbasis_0,obs_rotation_rhs_info_eigenbasis_1,obs_rotation_rhs_info_eigenbasis_2,obs_first_spectral_rotation_rhs_r0_c0,obs_first_spectral_rotation_rhs_r0_c1,obs_first_spectral_rotation_rhs_r0_c2,obs_first_spectral_rotation_rhs_r1_c0,obs_first_spectral_rotation_rhs_r1_c1,obs_first_spectral_rotation_rhs_r1_c2,obs_first_spectral_rotation_rhs_r2_c0,obs_first_spectral_rotation_rhs_r2_c1,obs_first_spectral_rotation_rhs_r2_c2,obs_spectral_rotation_rhs_r0_c0,obs_spectral_rotation_rhs_r0_c1,obs_spectral_rotation_rhs_r0_c2,obs_spectral_rotation_rhs_r1_c0,obs_spectral_rotation_rhs_r1_c1,obs_spectral_rotation_rhs_r1_c2,obs_spectral_rotation_rhs_r2_c0,obs_spectral_rotation_rhs_r2_c1,obs_spectral_rotation_rhs_r2_c2"
    )?;
    for rank in 0..fastlio_estimator::optimizer::TOP_ROTATION_RHS_SURFELS {
        write!(
            writer,
            ",obs_first_top_rotation_rhs_{rank}_sample_count,obs_first_top_rotation_rhs_{rank}_mean_w_0,obs_first_top_rotation_rhs_{rank}_mean_w_1,obs_first_top_rotation_rhs_{rank}_mean_w_2,obs_first_top_rotation_rhs_{rank}_k0_axis_w_0,obs_first_top_rotation_rhs_{rank}_k0_axis_w_1,obs_first_top_rotation_rhs_{rank}_k0_axis_w_2,obs_first_top_rotation_rhs_{rank}_world_0,obs_first_top_rotation_rhs_{rank}_world_1,obs_first_top_rotation_rhs_{rank}_world_2,obs_first_top_rotation_rhs_{rank}_world_norm,obs_first_top_rotation_rhs_{rank}_residual_norm_mean,obs_first_top_rotation_rhs_{rank}_best_score_mean,obs_first_top_rotation_rhs_{rank}_second_best_score_mean,obs_first_top_rotation_rhs_{rank}_second_best_score_count"
        )?;
    }
    for rank in 0..fastlio_estimator::optimizer::TOP_ROTATION_RHS_VOXELS {
        write!(
            writer,
            ",obs_first_top_rotation_rhs_voxel_{rank}_sample_count,obs_first_top_rotation_rhs_voxel_{rank}_min_w_0,obs_first_top_rotation_rhs_voxel_{rank}_min_w_1,obs_first_top_rotation_rhs_voxel_{rank}_min_w_2,obs_first_top_rotation_rhs_voxel_{rank}_world_0,obs_first_top_rotation_rhs_voxel_{rank}_world_1,obs_first_top_rotation_rhs_voxel_{rank}_world_2,obs_first_top_rotation_rhs_voxel_{rank}_world_norm,obs_first_top_rotation_rhs_voxel_{rank}_residual_norm_mean,obs_first_top_rotation_rhs_voxel_{rank}_best_score_mean,obs_first_top_rotation_rhs_voxel_{rank}_second_best_score_mean,obs_first_top_rotation_rhs_voxel_{rank}_second_best_score_count"
        )?;
    }
    write!(
        writer,
        ",iekf_total_rotation_correction_imu_0,iekf_total_rotation_correction_imu_1,iekf_total_rotation_correction_imu_2,iekf_total_rotation_correction_world_0,iekf_total_rotation_correction_world_1,iekf_total_rotation_correction_world_2,iekf_total_rotation_correction_norm"
    )?;
    writeln!(writer)?;
    let mut first_rotation_rhs_world_window = VecDeque::new();
    let mut first_rotation_rhs_world_window_sum = fastlio_types::Vec3::zeros();
    for row in rows {
        let p = row.state.position;
        let v = row.state.velocity;
        let q = row.state.orientation.quaternion();
        let obs = row.frame_summary.observation_diagnostics;
        while first_rotation_rhs_world_window
            .front()
            .is_some_and(|(timestamp_sec, _)| *timestamp_sec < row.timestamp_sec - 1.0)
        {
            let (_, rhs) = first_rotation_rhs_world_window.pop_front().unwrap();
            first_rotation_rhs_world_window_sum -= rhs;
        }
        if row.frame_summary.tracking {
            let rhs = row
                .frame_summary
                .first_observation_diagnostics
                .rotation_measurement_rhs_world;
            first_rotation_rhs_world_window_sum += rhs;
            first_rotation_rhs_world_window.push_back((row.timestamp_sec, rhs));
        }
        write!(
            writer,
            "{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{},{},{},{},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{},{},{},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{},{:.12},{:.12},{:.12},{:.12}",
            row.timestamp_sec,
            p.x,
            p.y,
            p.z,
            q.i,
            q.j,
            q.k,
            q.w,
            v.x,
            v.y,
            v.z,
            row.frame_summary.tracking,
            row.frame_summary.iekf_iterations,
            row.frame_summary.iekf_observations_first,
            row.frame_summary.iekf_observations_last,
            row.frame_summary.iekf_mean_abs_residual,
            row.frame_summary.iekf_max_abs_residual,
            row.frame_summary.iekf_final_rotation_delta_norm,
            row.frame_summary.iekf_final_position_delta_norm,
            row.frame_summary.iekf_final_velocity_delta_norm,
            row.frame_summary.iekf_final_accel_bias_delta_norm,
            row.frame_summary.iekf_final_gravity_delta_norm,
            obs.input_points,
            obs.accepted_observations,
            obs.no_association,
            obs.residual_abs_mean,
            obs.residual_abs_p50,
            obs.residual_abs_p90,
            obs.residual_abs_p95,
            obs.residual_abs_p99,
            obs.residual_abs_max,
            obs.surfel_query_best_score_p50,
            obs.surfel_query_best_score_p95,
            obs.surfel_query_second_best_score_p50,
            obs.surfel_query_second_best_score_p95,
            obs.surfel_query_second_best_count,
            obs.surfel_query_score_margin_p05,
            obs.surfel_query_score_margin_p50,
            obs.surfel_query_best_over_second_p95,
            obs.surfel_query_ambiguous_fraction,
        )?;
        write!(
            writer,
            ",{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12}",
            obs.rotation_information_eigenvalues[0],
            obs.rotation_information_eigenvalues[1],
            obs.rotation_information_eigenvalues[2],
            obs.rotation_information_eigenvectors[(0, 0)],
            obs.rotation_information_eigenvectors[(0, 1)],
            obs.rotation_information_eigenvectors[(0, 2)],
            obs.rotation_information_eigenvectors[(1, 0)],
            obs.rotation_information_eigenvectors[(1, 1)],
            obs.rotation_information_eigenvectors[(1, 2)],
            obs.rotation_information_eigenvectors[(2, 0)],
            obs.rotation_information_eigenvectors[(2, 1)],
            obs.rotation_information_eigenvectors[(2, 2)],
            obs.rotation_information_condition_number,
            obs.rotation_information_trace,
            obs.rotation_position_information[(0, 0)],
            obs.rotation_position_information[(0, 1)],
            obs.rotation_position_information[(0, 2)],
            obs.rotation_position_information[(1, 0)],
            obs.rotation_position_information[(1, 1)],
            obs.rotation_position_information[(1, 2)],
            obs.rotation_position_information[(2, 0)],
            obs.rotation_position_information[(2, 1)],
            obs.rotation_position_information[(2, 2)],
        )?;
        write!(
            writer,
            ",{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12}",
            obs.measurement_spectrum_rotation_trace[0],
            obs.measurement_spectrum_rotation_trace[1],
            obs.measurement_spectrum_rotation_trace[2],
            obs.measurement_spectrum_rotation_trace_fraction[0],
            obs.measurement_spectrum_rotation_trace_fraction[1],
            obs.measurement_spectrum_rotation_trace_fraction[2],
            obs.measurement_spectrum_eigenvalue_mean[0],
            obs.measurement_spectrum_eigenvalue_mean[1],
            obs.measurement_spectrum_eigenvalue_mean[2],
            obs.measurement_spectrum_surfel_axis_alignment[0],
            obs.measurement_spectrum_surfel_axis_alignment[1],
            obs.measurement_spectrum_surfel_axis_alignment[2],
        )?;
        let first = row.frame_summary.first_observation_diagnostics;
        write!(
            writer,
            ",{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12}",
            obs.measurement_spectrum_k0_axis_bin_fraction[0],
            obs.measurement_spectrum_k0_axis_bin_fraction[1],
            obs.measurement_spectrum_k0_axis_bin_fraction[2],
            obs.measurement_spectrum_k0_axis_concentration,
            obs.measurement_spectrum_k0_axis_dominant_world[0],
            obs.measurement_spectrum_k0_axis_dominant_world[1],
            obs.measurement_spectrum_k0_axis_dominant_world[2],
        )?;
        write!(
            writer,
            ",{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{},",
            first.rotation_measurement_rhs[0],
            first.rotation_measurement_rhs[1],
            first.rotation_measurement_rhs[2],
            obs.rotation_measurement_rhs[0],
            obs.rotation_measurement_rhs[1],
            obs.rotation_measurement_rhs[2],
            first.rotation_measurement_rhs_world[0],
            first.rotation_measurement_rhs_world[1],
            first.rotation_measurement_rhs_world[2],
            obs.rotation_measurement_rhs_world[0],
            obs.rotation_measurement_rhs_world[1],
            obs.rotation_measurement_rhs_world[2],
            first_rotation_rhs_world_window_sum[0],
            first_rotation_rhs_world_window_sum[1],
            first_rotation_rhs_world_window_sum[2],
            first_rotation_rhs_world_window.len(),
        )?;
        write!(
            writer,
            "{:.12},{:.12},{:.12},{:.12},{:.12},{:.12}",
            first.rotation_measurement_rhs_in_information_eigenbasis[0],
            first.rotation_measurement_rhs_in_information_eigenbasis[1],
            first.rotation_measurement_rhs_in_information_eigenbasis[2],
            obs.rotation_measurement_rhs_in_information_eigenbasis[0],
            obs.rotation_measurement_rhs_in_information_eigenbasis[1],
            obs.rotation_measurement_rhs_in_information_eigenbasis[2],
        )?;
        write!(
            writer,
            ",{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12}",
            first.measurement_spectrum_rotation_rhs[(0, 0)],
            first.measurement_spectrum_rotation_rhs[(0, 1)],
            first.measurement_spectrum_rotation_rhs[(0, 2)],
            first.measurement_spectrum_rotation_rhs[(1, 0)],
            first.measurement_spectrum_rotation_rhs[(1, 1)],
            first.measurement_spectrum_rotation_rhs[(1, 2)],
            first.measurement_spectrum_rotation_rhs[(2, 0)],
            first.measurement_spectrum_rotation_rhs[(2, 1)],
            first.measurement_spectrum_rotation_rhs[(2, 2)],
            obs.measurement_spectrum_rotation_rhs[(0, 0)],
            obs.measurement_spectrum_rotation_rhs[(0, 1)],
            obs.measurement_spectrum_rotation_rhs[(0, 2)],
            obs.measurement_spectrum_rotation_rhs[(1, 0)],
            obs.measurement_spectrum_rotation_rhs[(1, 1)],
            obs.measurement_spectrum_rotation_rhs[(1, 2)],
            obs.measurement_spectrum_rotation_rhs[(2, 0)],
            obs.measurement_spectrum_rotation_rhs[(2, 1)],
            obs.measurement_spectrum_rotation_rhs[(2, 2)],
        )?;
        for contributor in first.top_rotation_rhs_surfels {
            write!(
                writer,
                ",{},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{}",
                contributor.sample_count,
                contributor.mean_w[0],
                contributor.mean_w[1],
                contributor.mean_w[2],
                contributor.k0_axis_w[0],
                contributor.k0_axis_w[1],
                contributor.k0_axis_w[2],
                contributor.rhs_world[0],
                contributor.rhs_world[1],
                contributor.rhs_world[2],
                contributor.rhs_world.norm(),
                contributor.residual_norm_mean,
                contributor.best_score_mean,
                contributor.second_best_score_mean,
                contributor.second_best_score_count,
            )?;
        }
        for contributor in first.top_rotation_rhs_voxels {
            write!(
                writer,
                ",{},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{}",
                contributor.sample_count,
                contributor.voxel_min_w[0],
                contributor.voxel_min_w[1],
                contributor.voxel_min_w[2],
                contributor.rhs_world[0],
                contributor.rhs_world[1],
                contributor.rhs_world[2],
                contributor.rhs_world.norm(),
                contributor.residual_norm_mean,
                contributor.best_score_mean,
                contributor.second_best_score_mean,
                contributor.second_best_score_count,
            )?;
        }
        let total_rotation_correction_world =
            row.frame_summary.iekf_total_rotation_correction_world;
        write!(
            writer,
            ",{:.12},{:.12},{:.12},{:.12},{:.12},{:.12},{:.12}",
            row.frame_summary.iekf_total_rotation_correction_imu[0],
            row.frame_summary.iekf_total_rotation_correction_imu[1],
            row.frame_summary.iekf_total_rotation_correction_imu[2],
            total_rotation_correction_world[0],
            total_rotation_correction_world[1],
            total_rotation_correction_world[2],
            total_rotation_correction_world.norm(),
        )?;
        writeln!(writer)?;
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

fn write_rotation_rhs_peak_voxel_visualization(
    map_path: &Utf8PathBuf,
    map: &SurfelMap,
    rows: &[TrajectoryRow],
    time_window: (f64, f64),
) -> Result<()> {
    const PEAK_FRAME_COUNT: usize = 3;
    const VOXEL_SIZE_M: f64 = 1.0;
    const DEFAULT_RGB: u32 = 0x8a_8a_8a;
    const PEAK_RGB: u32 = 0xf0_3b_3b;

    let mut window = VecDeque::new();
    let mut window_sum = fastlio_types::Vec3::zeros();
    let mut peaks = Vec::new();
    for (index, row) in rows.iter().enumerate() {
        while window
            .front()
            .is_some_and(|(timestamp_sec, _)| *timestamp_sec < row.timestamp_sec - 1.0)
        {
            let (_, rhs) = window.pop_front().unwrap();
            window_sum -= rhs;
        }
        if row.frame_summary.tracking
            && row.timestamp_sec >= time_window.0
            && row.timestamp_sec <= time_window.1
        {
            let rhs = row
                .frame_summary
                .first_observation_diagnostics
                .rotation_measurement_rhs_world;
            window_sum += rhs;
            window.push_back((row.timestamp_sec, rhs));
            peaks.push((window_sum.norm(), index));
        }
    }
    peaks.sort_by(|left, right| right.0.total_cmp(&left.0));

    let mut peak_voxels = HashSet::new();
    for (_, index) in peaks.into_iter().take(PEAK_FRAME_COUNT) {
        for voxel in rows[index]
            .frame_summary
            .first_observation_diagnostics
            .top_rotation_rhs_voxels
        {
            if voxel.sample_count > 0 {
                peak_voxels.insert((
                    voxel.voxel_min_w[0] as i32,
                    voxel.voxel_min_w[1] as i32,
                    voxel.voxel_min_w[2] as i32,
                ));
            }
        }
    }

    let visualization_path = map_path.with_extension("rhs-peak-voxels.pcd");
    let mut writer = WriterInit {
        width: map.surfels().count() as u64,
        height: 1,
        viewpoint: Default::default(),
        data_kind: DataKind::Binary,
        schema: None,
        version: None,
    }
    .create::<ColoredSurfelPcdPoint, _>(&visualization_path)
    .with_context(|| {
        format!("failed to create RHS voxel visualization PCD `{visualization_path}`")
    })?;

    for surfel in map.surfels() {
        let voxel = (
            (surfel.mean_w[0] / VOXEL_SIZE_M).floor() as i32,
            (surfel.mean_w[1] / VOXEL_SIZE_M).floor() as i32,
            (surfel.mean_w[2] / VOXEL_SIZE_M).floor() as i32,
        );
        let rgb = if peak_voxels.contains(&voxel) {
            PEAK_RGB
        } else {
            DEFAULT_RGB
        };
        writer.push(&ColoredSurfelPcdPoint {
            x: surfel.mean_w[0] as f32,
            y: surfel.mean_w[1] as f32,
            z: surfel.mean_w[2] as f32,
            rgb,
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
    if let Some(rank_mode) = args.surfel_rank_mode {
        pipeline.set_surfel_rank_mode(rank_mode);
    }
    if let Some(spectrum_mode) = args.surfel_measurement_spectrum_mode {
        pipeline.set_surfel_measurement_spectrum_mode(spectrum_mode);
    }
    let stats = run_spsc(args.bag_path, &mut pipeline, replay_config.clone())?;
    if let Some(path) = &args.trajectory_path {
        write_trajectory(path, &stats.trajectory)?;
    }
    if let Some(path) = &args.surfel_map_path {
        write_surfel_map(path, &pipeline.map)?;
        if let Some(time_window) = args.rhs_visualization_window {
            write_rotation_rhs_peak_voxel_visualization(
                path,
                &pipeline.map,
                &stats.trajectory,
                time_window,
            )?;
        }
    }
    print_summary(&stats, &replay_config);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use fastlio_types::{ImuSample, LidarFrame, Vec3};
    use std::fs;

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
            "--surfel-rank=combined:0.5".into(),
            "--surfel-update=hard:5".into(),
        ];
        let parsed = ReplayArgs::parse(&args).unwrap();
        assert!(parsed.storm_mode);
        assert_eq!(parsed.playback_rate, 2.0);
        assert_eq!(parsed.channel_capacity, 64);
        assert_eq!(
            parsed.surfel_rank_mode,
            Some(SurfelRankMode::Combined {
                centroid_distance_weight: 0.5,
            })
        );
        assert_eq!(
            parsed.surfel_measurement_spectrum_mode,
            Some(SurfelMeasurementSpectrumMode::HardTruncation {
                max_variance_ratio: 5.0,
            })
        );
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
    fn trajectory_csv_has_compact_observation_diagnostics() {
        let path = Utf8PathBuf::from("/tmp/fastlio-rs-trajectory-diagnostics-test.csv");
        let rows = vec![TrajectoryRow {
            timestamp_sec: 1.0,
            state: NavState::default(),
            frame_summary: PipelineFrameSummary::default(),
        }];

        write_trajectory(&path, &rows).unwrap();

        let csv = fs::read_to_string(&path).unwrap();
        let mut lines = csv.lines();
        let header = lines.next().unwrap();
        let row = lines.next().unwrap();
        assert_eq!(header.split(',').count(), row.split(',').count());
        assert!(header.contains("obs_no_association"));
        assert!(header.contains("obs_residual_abs_p95"));
        assert!(header.contains("surfel_query_best_score_p95"));
        assert!(header.contains("surfel_query_second_best_score_p95"));
        assert!(header.contains("surfel_query_score_margin_p05"));
        assert!(header.contains("surfel_query_best_over_second_p95"));
        assert!(header.contains("surfel_query_ambiguous_fraction"));
        assert!(header.contains("obs_rotation_info_eigenvalue_0"));
        assert!(header.contains("obs_rotation_position_info_r2_c2"));
        assert!(!header.contains("obs_extra_line_added"));
        assert!(!header.contains("obs_plane_score_mean"));
        assert!(!header.contains("gravity_variance"));
        assert!(!header.contains("cross_covariance"));

        let _ = fs::remove_file(path);
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
