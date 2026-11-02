use std::{cmp::max, fs};

use anyhow::{Context, Result};
use camino::Utf8Path;
use fastlio_types::{ImuSample, LidarFrame, PointXYZI, TimedPoint, Vec3, transfer_from_header};
use memmap2::Mmap;
use ros2_dispatch::borrow_decode_message_by_schema;

#[derive(Debug)]
pub enum SensorEvent {
    Imu(ImuSample),
    Lidar(LidarFrame),
}

impl SensorEvent {
    pub fn timestamp_sec(&self) -> f64 {
        match self {
            SensorEvent::Imu(imu_sample) => imu_sample.time_stamp_sec,
            SensorEvent::Lidar(lidar_frame) => lidar_frame.base_timestamp_sec,
        }
    }

    pub fn dispatch(data: &[u8], schema_name: &str) -> Result<Self> {
        let result = borrow_decode_message_by_schema(schema_name, data)?;
        match result {
            ros2_dispatch::DecodedMessageBorrowed::LivoxRosDriver2CustomMsg(custom_msg) => {
                let mut max_offset = 0;
                let mut points: Vec<_> = custom_msg
                    .points
                    .iter()
                    .map(|e| {
                        max_offset = max(max_offset, e.offset_time);
                        TimedPoint {
                            offset_time_sec: e.offset_time as f64 / 1e9,
                            point: PointXYZI {
                                x: e.x,
                                y: e.y,
                                z: e.z,
                                intensity: f32::from(e.reflectivity),
                            },
                            tag: e.tag,
                            line: e.line,
                        }
                    })
                    .collect();
                sort_timepoint_by_offset(&mut points);
                let msg = LidarFrame::new(
                    custom_msg.timebase as f64 / 1e9,
                    custom_msg.timebase as f64 / 1e9 + max_offset as f64 / 1e9,
                    points,
                );
                Ok(Self::Lidar(msg))
            }
            ros2_dispatch::DecodedMessageBorrowed::SensorMsgsImu(imu) => {
                let msg = ImuSample {
                    time_stamp_sec: transfer_from_header(
                        imu.header.stamp.sec,
                        imu.header.stamp.nanosec,
                    ),
                    gyro: Vec3::<f64>::new(
                        imu.angular_velocity.x,
                        imu.angular_velocity.y,
                        imu.angular_velocity.z,
                    ),
                    accel: Vec3::<f64>::new(
                        imu.linear_acceleration.x,
                        imu.linear_acceleration.y,
                        imu.linear_acceleration.z,
                    ),
                };
                Ok(Self::Imu(msg))
            }
            ros2_dispatch::DecodedMessageBorrowed::SensorMsgsPointCloud2(_) => todo!(),
        }
    }
}

fn sort_timepoint_by_offset(points: &mut [TimedPoint]) {
    points.sort_by(|a, b| a.offset_time_sec.total_cmp(&b.offset_time_sec));
}

#[derive(Debug, Default, Clone, Copy)]
pub struct ReadStats {
    pub total_messages: usize,
    pub lidar_messages: usize,
    pub emitted_events: usize,
    pub emitted_lidar_events: usize,
    pub reordered_imu_events: usize,
    pub max_imu_arrival_regression_sec: f64,
    pub skipped_missing_schema: usize,
    pub skipped_unsupported_schema: usize,
    pub skipped_unsupported_topic: usize,
    pub last_timestamp_sec: Option<u64>,
}

// Xsens samples in KA-Urban-East arrive up to 8 ms out of header-time order.
// Delay only IMU emission; LiDAR remains in bag order and the synchronizer
// still rejects any timestamp regression that exceeds this bounded window.
const IMU_REORDER_WINDOW_SEC: f64 = 0.02;

#[derive(Default)]
struct ImuReorderBuffer {
    pending: Vec<ImuSample>,
    max_seen_time_sec: Option<f64>,
    last_emitted_time_sec: Option<f64>,
}

impl ImuReorderBuffer {
    fn push<F>(&mut self, imu: ImuSample, stats: &mut ReadStats, on_event: &mut F) -> Result<()>
    where
        F: FnMut(SensorEvent) -> Result<()>,
    {
        let time = imu.time_stamp_sec;
        if !time.is_finite() {
            anyhow::bail!("non-finite IMU header timestamp: {time}");
        }
        if let Some(last_emitted) = self.last_emitted_time_sec
            && time < last_emitted
        {
            anyhow::bail!(
                "IMU header timestamp {time:.9} arrived after {last_emitted:.9} was emitted; disorder exceeds {IMU_REORDER_WINDOW_SEC:.3} s window"
            );
        }
        if let Some(max_seen) = self.max_seen_time_sec
            && time < max_seen
        {
            stats.reordered_imu_events += 1;
            stats.max_imu_arrival_regression_sec =
                stats.max_imu_arrival_regression_sec.max(max_seen - time);
        }
        let max_seen = self.max_seen_time_sec.map_or(time, |seen| seen.max(time));
        self.max_seen_time_sec = Some(max_seen);
        let index = self
            .pending
            .partition_point(|sample| sample.time_stamp_sec <= time);
        self.pending.insert(index, imu);
        self.flush_until(max_seen - IMU_REORDER_WINDOW_SEC, stats, on_event)
    }

    fn flush_until<F>(
        &mut self,
        cutoff_sec: f64,
        stats: &mut ReadStats,
        on_event: &mut F,
    ) -> Result<()>
    where
        F: FnMut(SensorEvent) -> Result<()>,
    {
        while self
            .pending
            .first()
            .is_some_and(|sample| sample.time_stamp_sec <= cutoff_sec)
        {
            let imu = self.pending.remove(0);
            self.last_emitted_time_sec = Some(imu.time_stamp_sec);
            on_event(SensorEvent::Imu(imu))?;
            stats.emitted_events += 1;
        }
        Ok(())
    }
}

fn map_mcap<P: AsRef<Utf8Path>>(p: P) -> Result<Mmap> {
    let fd = fs::File::open(p.as_ref()).context("Couldn't open MCAP file")?;
    unsafe { Mmap::map(&fd) }.context("Couldn't map MCAP file")
}

fn is_supported_sensor_schema(schema_name: &str) -> bool {
    matches!(
        schema_name,
        "livox_ros_driver2/msg/CustomMsg"
            | "livox_ros_driver/msg/CustomMsg"
            | "sensor_msgs/msg/Imu"
    )
}

pub fn read_mcap_events<P, F>(p: P, mut on_event: F) -> Result<ReadStats>
where
    P: AsRef<Utf8Path>,
    F: FnMut(SensorEvent) -> Result<()>,
{
    read_mcap_events_on_topics(p, "/livox/lidar", "/livox/imu", &mut on_event)
}

/// Decode only the configured LiDAR and IMU topics from an MCAP bag.
/// Sensor timestamps come from each message; MCAP log time is used only to
/// check that the input stream does not regress.
pub fn read_mcap_events_on_topics<P, F>(
    p: P,
    lidar_topic: &str,
    imu_topic: &str,
    mut on_event: F,
) -> Result<ReadStats>
where
    P: AsRef<Utf8Path>,
    F: FnMut(SensorEvent) -> Result<()>,
{
    if lidar_topic.is_empty() || imu_topic.is_empty() || lidar_topic == imu_topic {
        anyhow::bail!("LiDAR and IMU topics must be nonempty and distinct");
    }
    let mapped = map_mcap(p)?;
    let mut stats = ReadStats::default();
    let mut imu_reorder = ImuReorderBuffer::default();

    for message in mcap::MessageStream::new(&mapped)? {
        let message = message?;
        let current = message.log_time;
        stats.total_messages += 1;
        if message.channel.topic == lidar_topic {
            stats.lidar_messages += 1;
        }

        if message.channel.topic != lidar_topic && message.channel.topic != imu_topic {
            stats.skipped_unsupported_topic += 1;
            continue;
        }

        let Some(schema) = message.channel.schema.as_ref() else {
            stats.skipped_missing_schema += 1;
            continue;
        };

        if !is_supported_sensor_schema(&schema.name) {
            stats.skipped_unsupported_schema += 1;
            continue;
        }

        let sensor_source =
            SensorEvent::dispatch(&message.data, &schema.name).with_context(|| {
                format!(
                    "failed to decode MCAP message on topic `{}` with schema `{}`",
                    message.channel.topic, schema.name
                )
            })?;
        if let Some(last_publish_timestamp) = stats.last_timestamp_sec
            && last_publish_timestamp > current
        {
            anyhow::bail!(
                "event timestamp regressed: current={current}, last={last_publish_timestamp}"
            )
        }
        stats.last_timestamp_sec = Some(current);
        match sensor_source {
            SensorEvent::Imu(imu) => imu_reorder.push(imu, &mut stats, &mut on_event)?,
            SensorEvent::Lidar(lidar) => {
                on_event(SensorEvent::Lidar(lidar))?;
                stats.emitted_lidar_events += 1;
                stats.emitted_events += 1;
            }
        }
    }

    imu_reorder.flush_until(f64::INFINITY, &mut stats, &mut on_event)?;

    if stats.emitted_lidar_events == 0 || stats.emitted_events == stats.emitted_lidar_events {
        anyhow::bail!(
            "no decodable LiDAR/IMU pair for topics lidar=`{lidar_topic}`, imu=`{imu_topic}` (emitted lidar={}, emitted imu={})",
            stats.emitted_lidar_events,
            stats.emitted_events - stats.emitted_lidar_events
        );
    }

    Ok(stats)
}

#[cfg(test)]
mod test {
    use crate::{
        ImuReorderBuffer, ReadStats, SensorEvent, read_mcap_events, read_mcap_events_on_topics,
    };
    use fastlio_types::Vec3;

    fn imu(time_stamp_sec: f64) -> fastlio_types::ImuSample {
        fastlio_types::ImuSample {
            time_stamp_sec,
            gyro: Vec3::zeros(),
            accel: Vec3::zeros(),
        }
    }

    #[test]
    fn reorders_bounded_imu_arrival_regression_without_losing_samples() {
        let mut buffer = ImuReorderBuffer::default();
        let mut stats = ReadStats::default();
        let mut emitted = Vec::new();
        let mut collect = |event| {
            if let SensorEvent::Imu(sample) = event {
                emitted.push(sample.time_stamp_sec);
            }
            Ok(())
        };
        for time in [1.0, 1.01, 1.03, 1.022, 1.04, 1.06] {
            buffer.push(imu(time), &mut stats, &mut collect).unwrap();
        }
        buffer
            .flush_until(f64::INFINITY, &mut stats, &mut collect)
            .unwrap();
        assert_eq!(emitted, [1.0, 1.01, 1.022, 1.03, 1.04, 1.06]);
        assert_eq!(stats.emitted_events, emitted.len());
        assert_eq!(stats.reordered_imu_events, 1);
        assert!((stats.max_imu_arrival_regression_sec - 0.008).abs() < 1e-12);
    }

    #[test]
    fn rejects_imu_arriving_after_reorder_window() {
        let mut buffer = ImuReorderBuffer::default();
        let mut stats = ReadStats::default();
        let mut collect = |_| Ok(());
        buffer.push(imu(1.0), &mut stats, &mut collect).unwrap();
        buffer.push(imu(1.03), &mut stats, &mut collect).unwrap();
        let error = buffer
            .push(imu(0.99), &mut stats, &mut collect)
            .unwrap_err();
        assert!(error.to_string().contains("disorder exceeds"));
    }

    #[test]
    fn rejects_ambiguous_topic_selection() {
        let result =
            read_mcap_events_on_topics("missing.mcap", "/imu/data", "/imu/data", |_| Ok(()));
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("distinct"));
    }

    #[test]
    #[ignore]
    fn smoke_read_it() {
        let stats = read_mcap_events(
            "/home/lyra/Projects/fastlio-rs/rosbags/rosbag2_upstairs/rosbag2_2026_06_23-15_13_52_0.mcap",
            |_e| {
                // println!("{:?}", e);
                Ok(())
            },
        )
        .unwrap();
        assert!(stats.emitted_events > 0);
        assert!(stats.skipped_missing_schema == 0);
        assert!(stats.skipped_unsupported_topic == 0);
        // assert!(stats.skipped_unmonotonic_timestamp == 0);
    }
}
