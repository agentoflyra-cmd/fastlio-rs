use anyhow::{Result, anyhow};
use camino::Utf8PathBuf;
use fastlio_dataset::{ReadStats, SensorEvent, read_mcap_events};
use fastlio_pipeline::synchronizer::MeasurementSynchronizer;
use ringbuffer_spsc::{RingBufferReader, RingBufferWriter, ringbuffer};
use std::str::FromStr;
use std::sync::{Arc, atomic::AtomicBool};
use std::thread;
use std::time::Duration;

enum EventType {
    ProcessingEvent(SensorEvent),
    EndSignal(Result<ReadStats>),
}

pub struct ReplayConfig {
    /// The speed rate is a floating-point
    /// number used to represent the playback rate,
    /// adjusted with keyboard events up and down,
    /// with each increment of 0.1.
    pub speed_rate: f64,
    /// default channel size.
    /// In this implementation does not consider
    /// athe case of multiple mcaps,
    /// so there is no need to replicate
    /// a locked priority queue.
    /// This parameter indicates the
    /// size of the bounded SPSC(ringbuf)
    /// channel, typically a multiple of 2.
    pub default_channel_bound: usize,
    /// Storm mode is an update introduced
    /// later with reference to rosbag.
    /// This mode assumes an unbounded playback
    /// rate and is typically used to check
    /// the system's robustness when
    /// handling large amounts of data.
    pub storm_mode: bool,
}

impl Default for ReplayConfig {
    fn default() -> Self {
        Self {
            speed_rate: 1.0,
            default_channel_bound: 1024,
            storm_mode: false,
        }
    }
}

struct ProducerAlive {
    alive: Arc<AtomicBool>,
}

impl Drop for ProducerAlive {
    fn drop(&mut self) {
        self.alive
            .store(false, std::sync::atomic::Ordering::Release);
    }
}

// used for real use
fn push_blocking<T>(prod: &mut RingBufferWriter<T>, mut item: T) {
    loop {
        match prod.push(item) {
            None => return,
            Some(returned) => {
                item = returned;
                thread::sleep(Duration::from_millis(1));
            }
        }
    }
}

fn run_spsc(p: Utf8PathBuf, config: Arc<ReplayConfig>) -> Result<ReplayStats> {
    // let rb = HeapRb::<EventType>::new(config.default_channel_bound);
    let (mut prod, mut cons) = ringbuffer::<EventType>(config.default_channel_bound);
    let mut synchronizer = MeasurementSynchronizer::new();
    let alive = Arc::new(AtomicBool::new(true));
    let producer_alive = ProducerAlive {
        alive: alive.clone(),
    };
    let producer = thread::spawn(move || {
        let _guard = producer_alive;
        produce_events(p, &mut prod, config)
    });
    let stats = recieve_evnets(&mut cons, &mut synchronizer, alive)?;
    let _ = producer
        .join()
        .map_err(|_| anyhow!("replay producer thread panic!"))?;
    Ok(stats)
}

fn produce_events(
    p: Utf8PathBuf,
    prod: &mut RingBufferWriter<EventType>,
    config: Arc<ReplayConfig>,
) -> Result<()> {
    let result = read_mcap_events(p, |sensor_event| {
        if !config.storm_mode {
            // sleep for 50ms, if speed = 1.0x
            let sleep_time = 0.05 / config.speed_rate.max(0.1);
            thread::sleep(Duration::from_secs_f64(sleep_time));
        }
        // push_blocking(prod, EventType::ProcessingEvent(sensor_event));
        // prod.try_push(EventType::ProcessingEvent(sensor_event))
        //     .map_err(|_| anyhow!("prod buffer has been full!"))?;
        match prod.push(EventType::ProcessingEvent(sensor_event)) {
            Some(_) => Err(anyhow!("prod buffer has been full!")),
            None => Ok(()),
        }
    })
    .map_err(|e| anyhow!("{:#}", e));
    push_blocking(prod, EventType::EndSignal(result));
    Ok(())
}

fn recieve_evnets(
    cons: &mut RingBufferReader<EventType>,
    synchronizer: &mut MeasurementSynchronizer,
    alive: Arc<AtomicBool>,
) -> Result<ReplayStats> {
    let mut replay_stats = ReplayStats::default();
    loop {
        match cons.pull() {
            Some(EventType::ProcessingEvent(sensor_event)) => match sensor_event {
                SensorEvent::Imu(imu_sample) => synchronizer.pend_imu(imu_sample)?,
                SensorEvent::Lidar(lidar_frame) => synchronizer.pend_lidar(lidar_frame)?,
            },
            Some(EventType::EndSignal(read_stats)) => {
                replay_stats.dropped_lidar_before_first_imu =
                    synchronizer.dropped_lidar_before_first_imu;
                replay_stats.pending_lidar_at_eof = synchronizer.lidar_buffer.len();
                replay_stats.read = read_stats?;
                return Ok(replay_stats);
            }
            None if !alive.load(std::sync::atomic::Ordering::Acquire) => {
                return Err(anyhow!("producer dropped without end signal"));
            }
            None => {
                thread::yield_now();
            }
        }
        let groups = synchronizer.drain_ready();
        replay_stats.synchronized_groups += groups.len();
    }
}

#[allow(unused)]
#[derive(Default, Debug)]
struct ReplayStats {
    pub read: ReadStats,
    pub synchronized_groups: usize,
    pub dropped_lidar_before_first_imu: usize,
    pub pending_lidar_at_eof: usize,
}

fn main() -> Result<()> {
    let bag_path = Utf8PathBuf::from_str(
        "/home/lyra/Projects/fastlio-rs/rosbags/rosbag2_upstairs/rosbag2_2026_06_23-15_13_52_0.mcap",
    )?;
    let config = ReplayConfig {
        storm_mode: true,
        ..Default::default()
    };
    let config = Arc::new(config);
    let result = run_spsc(bag_path, config)?;
    println!("{:?}", result);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use fastlio_types::{ImuSample, Vec3};
    use std::sync::mpsc;
    use std::time::Duration;

    const MCAP: &str = "/home/lyra/Projects/fastlio-rs/rosbags/rosbag2_upstairs/rosbag2_2026_06_23-15_13_52_0.mcap";

    fn imu_event(t: f64) -> SensorEvent {
        SensorEvent::Imu(ImuSample {
            time_stamp_sec: t,
            gyro: Vec3::zeros(),
            accel: Vec3::zeros(),
        })
    }

    fn ok_stats(emitted: usize) -> ReadStats {
        ReadStats {
            total_messages: emitted,
            emitted_events: emitted,
            ..Default::default()
        }
    }

    #[test]
    fn normal_replay_completion_is_returned() {
        let (mut prod, mut cons) = ringbuffer::<EventType>(16);
        assert!(prod.push(EventType::EndSignal(Ok(ok_stats(3)))).is_none());
        let mut sync = MeasurementSynchronizer::new();
        let alive = Arc::new(AtomicBool::new(true));
        let stats = recieve_evnets(&mut cons, &mut sync, alive).unwrap();
        assert_eq!(stats.read.emitted_events, 3);
    }

    #[test]
    fn full_ringbuf_returns_channel_full_error() {
        let (mut prod, _cons) = ringbuffer::<EventType>(1);
        assert!(
            prod.push(EventType::ProcessingEvent(imu_event(1.0)))
                .is_none()
        );
        assert!(
            prod.push(EventType::ProcessingEvent(imu_event(2.0)))
                .is_some()
        );
    }

    #[test]
    fn producer_read_error_is_propagated() {
        let (mut prod, mut cons) = ringbuffer::<EventType>(16);
        let config = Arc::new(ReplayConfig {
            storm_mode: true,
            ..Default::default()
        });
        produce_events(
            Utf8PathBuf::from("/nonexistent/path.mcap"),
            &mut prod,
            config,
        )
        .unwrap();
        match cons.pull() {
            Some(EventType::EndSignal(Err(_))) => {}
            _ => panic!("expected EndSignal(Err) for bad path"),
        }
    }

    #[test]
    fn consumer_does_not_wait_forever_after_producer_failure() {
        let (_prod, mut cons) = ringbuffer::<EventType>(16);
        let alive = Arc::new(AtomicBool::new(false));

        let (tx, rx) = mpsc::channel();
        let handle = thread::spawn(move || {
            let mut sync = MeasurementSynchronizer::new();
            let result = recieve_evnets(&mut cons, &mut sync, alive);
            let _ = tx.send(result);
        });

        match rx.recv_timeout(Duration::from_millis(200)) {
            Ok(Err(_)) => {
                let _ = handle.join();
            }
            Ok(Ok(_)) => panic!("expected error after producer drop, got Ok"),
            Err(_) => panic!("consumer waited forever after producer failure"),
        }
    }

    #[test]
    fn buffered_events_are_drained_before_normal_completion() {
        let (mut prod, mut cons) = ringbuffer::<EventType>(16);
        assert!(
            prod.push(EventType::ProcessingEvent(imu_event(1.0)))
                .is_none()
        );
        assert!(
            prod.push(EventType::ProcessingEvent(imu_event(2.0)))
                .is_none()
        );
        assert!(prod.push(EventType::EndSignal(Ok(ok_stats(2)))).is_none());
        let mut sync = MeasurementSynchronizer::new();
        let alive = Arc::new(AtomicBool::new(true));
        let stats = recieve_evnets(&mut cons, &mut sync, alive).unwrap();
        assert_eq!(stats.read.emitted_events, 2);
        assert_eq!(sync.imu_buffer.len(), 2);
    }

    #[test]
    fn channel_capacity_one_producer_failure_exits_with_error() {
        let config = Arc::new(ReplayConfig {
            storm_mode: true,
            default_channel_bound: 1,
            ..Default::default()
        });
        let (tx, rx) = mpsc::channel();
        let handle = thread::spawn(move || {
            let result = run_spsc(Utf8PathBuf::from(MCAP), config);
            let _ = tx.send(result);
        });
        match rx.recv_timeout(Duration::from_secs(5)) {
            Ok(Err(_)) => {
                let _ = handle.join();
            }
            Ok(Ok(_)) => panic!("expected producer failure error, got Ok"),
            Err(_) => panic!("program hung after producer failure"),
        }
    }
}
