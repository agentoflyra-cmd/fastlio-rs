use std::{cmp::max, fs};

use anyhow::{Context, Result, bail};
use camino::Utf8Path;
use fastlio_types::{ImuSample, LidarFrame, PointXYZI, TimedPoint, Vec3, transfer_from_header};
use memmap2::Mmap;
use ros2_dispatch::borrow_decode_message_by_schema;
use sensor_msgs::borrowed::{PointCloud2, PointField};

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
                sort_points_by_offset_time(&mut points);
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
            ros2_dispatch::DecodedMessageBorrowed::SensorMsgsPointCloud2(point_cloud) => {
                Ok(Self::Lidar(pointcloud2_to_lidar_frame(&point_cloud)?))
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct ReadOptions {
    pub lidar_topic: String,
    pub imu_topic: String,
}

impl ReadOptions {
    pub fn new(lidar_topic: impl Into<String>, imu_topic: impl Into<String>) -> Self {
        Self {
            lidar_topic: lidar_topic.into(),
            imu_topic: imu_topic.into(),
        }
    }
}

impl Default for ReadOptions {
    fn default() -> Self {
        Self::new("/livox/lidar", "/livox/imu")
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
        "livox_ros_driver2/msg/CustomMsg" | "sensor_msgs/msg/Imu" | "sensor_msgs/msg/PointCloud2"
    )
}

fn is_supported_topic(topic_name: &str, options: &ReadOptions) -> bool {
    topic_name == options.lidar_topic || topic_name == options.imu_topic
}

pub fn read_mcap_events<P, F>(p: P, on_event: F) -> Result<ReadStats>
where
    P: AsRef<Utf8Path>,
    F: FnMut(SensorEvent) -> Result<()>,
{
    read_mcap_events_with_options(p, &ReadOptions::default(), on_event)
}

pub fn read_mcap_events_with_options<P, F>(
    p: P,
    options: &ReadOptions,
    mut on_event: F,
) -> Result<ReadStats>
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

        if !is_supported_topic(&message.channel.topic, options) {
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

fn pointcloud2_to_lidar_frame(point_cloud: &PointCloud2<'_>) -> Result<LidarFrame> {
    if point_cloud.is_bigendian {
        bail!("big-endian PointCloud2 is not supported");
    }
    let base_timestamp_sec = transfer_from_header(
        point_cloud.header.stamp.sec,
        point_cloud.header.stamp.nanosec,
    );
    let field_x = required_field(point_cloud, "x")?;
    let field_y = required_field(point_cloud, "y")?;
    let field_z = required_field(point_cloud, "z")?;
    let field_intensity = optional_field(point_cloud, "intensity");
    let field_ring = optional_field(point_cloud, "ring");
    let field_time = optional_field(point_cloud, "time")
        .or_else(|| optional_field(point_cloud, "timestamp"))
        .or_else(|| optional_field(point_cloud, "t"));
    let point_step = point_cloud.point_step as usize;
    if point_step == 0 {
        bail!("PointCloud2 point_step must be positive");
    }

    let point_count = point_cloud.width as usize * point_cloud.height as usize;
    let available_points = point_cloud.data.len() / point_step;
    let point_count = point_count.min(available_points);
    let mut max_offset_time_sec = 0.0_f64;
    let mut points = Vec::with_capacity(point_count);
    for point_index in 0..point_count {
        let base = point_index * point_step;
        let x = read_f32_field(point_cloud.data, base, field_x)?;
        let y = read_f32_field(point_cloud.data, base, field_y)?;
        let z = read_f32_field(point_cloud.data, base, field_z)?;
        let intensity = field_intensity
            .map(|field| read_numeric_field_as_f32(point_cloud.data, base, field))
            .transpose()?
            .unwrap_or(0.0);
        let offset_time_sec = field_time
            .map(|field| read_time_offset_sec(point_cloud.data, base, field, base_timestamp_sec))
            .transpose()?
            .unwrap_or(0.0);
        if offset_time_sec.is_finite() && offset_time_sec >= max_offset_time_sec {
            max_offset_time_sec = offset_time_sec;
        }
        let line = field_ring
            .map(|field| read_numeric_field_as_u8(point_cloud.data, base, field))
            .transpose()?
            .unwrap_or(0);

        points.push(TimedPoint {
            offset_time_sec,
            point: PointXYZI { x, y, z, intensity },
            tag: 0,
            line,
        });
    }
    sort_points_by_offset_time(&mut points);

    Ok(LidarFrame::new(
        base_timestamp_sec,
        base_timestamp_sec + max_offset_time_sec,
        points,
    ))
}

fn required_field<'a>(point_cloud: &'a PointCloud2<'_>, name: &str) -> Result<&'a PointField<'a>> {
    optional_field(point_cloud, name).with_context(|| format!("PointCloud2 missing `{name}` field"))
}

fn optional_field<'a>(point_cloud: &'a PointCloud2<'_>, name: &str) -> Option<&'a PointField<'a>> {
    point_cloud.fields.iter().find(|field| field.name == name)
}

fn read_f32_field(data: &[u8], point_base: usize, field: &PointField<'_>) -> Result<f32> {
    if field.datatype != PointField::FLOAT32 {
        bail!("PointCloud2 field `{}` must be FLOAT32", field.name);
    }
    read_array::<4>(data, point_base, field).map(f32::from_le_bytes)
}

fn read_time_offset_sec(
    data: &[u8],
    point_base: usize,
    field: &PointField<'_>,
    base_timestamp_sec: f64,
) -> Result<f64> {
    let value = read_numeric_field_as_f64(data, point_base, field)?;
    if value.abs() > 1.0e6 {
        Ok(value - base_timestamp_sec)
    } else {
        Ok(value)
    }
}

fn read_numeric_field_as_f32(
    data: &[u8],
    point_base: usize,
    field: &PointField<'_>,
) -> Result<f32> {
    Ok(match field.datatype {
        PointField::UINT8 => read_array::<1>(data, point_base, field)?[0] as f32,
        PointField::UINT16 => u16::from_le_bytes(read_array::<2>(data, point_base, field)?) as f32,
        PointField::UINT32 => u32::from_le_bytes(read_array::<4>(data, point_base, field)?) as f32,
        PointField::FLOAT32 => f32::from_le_bytes(read_array::<4>(data, point_base, field)?),
        PointField::FLOAT64 => f64::from_le_bytes(read_array::<8>(data, point_base, field)?) as f32,
        _ => bail!(
            "unsupported PointCloud2 numeric field `{}` datatype {}",
            field.name,
            field.datatype
        ),
    })
}

fn read_numeric_field_as_f64(
    data: &[u8],
    point_base: usize,
    field: &PointField<'_>,
) -> Result<f64> {
    Ok(match field.datatype {
        PointField::UINT8 => read_array::<1>(data, point_base, field)?[0] as f64,
        PointField::UINT16 => u16::from_le_bytes(read_array::<2>(data, point_base, field)?) as f64,
        PointField::UINT32 => u32::from_le_bytes(read_array::<4>(data, point_base, field)?) as f64,
        PointField::FLOAT32 => f32::from_le_bytes(read_array::<4>(data, point_base, field)?) as f64,
        PointField::FLOAT64 => f64::from_le_bytes(read_array::<8>(data, point_base, field)?),
        _ => bail!(
            "unsupported PointCloud2 time field `{}` datatype {}",
            field.name,
            field.datatype
        ),
    })
}

fn read_numeric_field_as_u8(data: &[u8], point_base: usize, field: &PointField<'_>) -> Result<u8> {
    let value = read_numeric_field_as_f64(data, point_base, field)?;
    if !(0.0..=u8::MAX as f64).contains(&value) {
        bail!("PointCloud2 field `{}` is out of u8 range", field.name);
    }
    Ok(value as u8)
}

fn read_array<const N: usize>(
    data: &[u8],
    point_base: usize,
    field: &PointField<'_>,
) -> Result<[u8; N]> {
    if field.count != 1 {
        bail!(
            "PointCloud2 field `{}` count={} is not supported",
            field.name,
            field.count
        );
    }
    let start = point_base + field.offset as usize;
    let end = start + N;
    let Some(bytes) = data.get(start..end) else {
        bail!("PointCloud2 field `{}` reads past data buffer", field.name);
    };
    Ok(bytes.try_into().expect("slice length is checked"))
}

fn sort_points_by_offset_time(points: &mut [TimedPoint]) {
    points.sort_by(|a, b| a.offset_time_sec.total_cmp(&b.offset_time_sec));
}

#[cfg(test)]
mod test {
    use fastlio_types::{PointXYZI, TimedPoint};

    use crate::{read_mcap_events, sort_points_by_offset_time};

    fn point(offset_time_sec: f64) -> TimedPoint {
        TimedPoint {
            offset_time_sec,
            point: PointXYZI {
                x: offset_time_sec as f32,
                y: 0.0,
                z: 0.0,
                intensity: 1.0,
            },
            tag: 0,
            line: 0,
        }
    }

    #[test]
    fn livox_points_are_normalized_to_offset_time_order() {
        let mut points = vec![point(0.3), point(0.1), point(0.2), point(0.2)];
        sort_points_by_offset_time(&mut points);
        let offsets: Vec<_> = points.iter().map(|point| point.offset_time_sec).collect();

        assert_eq!(offsets, vec![0.1, 0.2, 0.2, 0.3]);
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
