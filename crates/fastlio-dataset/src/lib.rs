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
                let points = custom_msg
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
        }
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct ReadStats {
    pub total_messages: usize,
    pub emitted_events: usize,
    pub skipped_missing_schema: usize,
    pub skipped_unsupported_schema: usize,
    pub skipped_unsupported_topic: usize,
    pub last_timestamp_sec: Option<u64>,
}

fn map_mcap<P: AsRef<Utf8Path>>(p: P) -> Result<Mmap> {
    let fd = fs::File::open(p.as_ref()).context("Couldn't open MCAP file")?;
    unsafe { Mmap::map(&fd) }.context("Couldn't map MCAP file")
}

fn is_supported_sensor_schema(schema_name: &str) -> bool {
    matches!(
        schema_name,
        "livox_ros_driver2/msg/CustomMsg" | "sensor_msgs/msg/Imu"
    )
}

fn is_supported_topic(topic_name: &str) -> bool {
    matches!(topic_name, "/livox/lidar" | "/livox/imu")
}

pub fn read_mcap_events<P, F>(p: P, mut on_event: F) -> Result<ReadStats>
where
    P: AsRef<Utf8Path>,
    F: FnMut(SensorEvent) -> Result<()>,
{
    let mapped = map_mcap(p)?;
    let mut stats = ReadStats::default();

    for message in mcap::MessageStream::new(&mapped)? {
        let message = message?;
        let current = message.log_time;
        stats.total_messages += 1;

        if !is_supported_topic(&message.channel.topic) {
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
        on_event(sensor_source)?;
        stats.emitted_events += 1;
    }

    Ok(stats)
}

#[cfg(test)]
mod test {
    use crate::read_mcap_events;

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
