// TODO:we can use allow clippy here, because acctually i will not use this path.
#[derive(Clone, Debug)]
#[allow(clippy::large_enum_variant)]
pub enum DecodedMessage {
    LivoxRosDriver2CustomMsg(livox_ros_driver2::msg::CustomMsg),
    SensorMsgsImu(sensor_msgs::msg::Imu),
    SensorMsgsPointCloud2(sensor_msgs::msg::PointCloud2),
}

impl DecodedMessage {
    pub fn schema_name(&self) -> &'static str {
        match self {
            Self::LivoxRosDriver2CustomMsg(_) => "livox_ros_driver2/msg/CustomMsg",
            Self::SensorMsgsImu(_) => "sensor_msgs/msg/Imu",
            Self::SensorMsgsPointCloud2(_) => "sensor_msgs/msg/PointCloud2",
        }
    }

    pub fn encode_to_vec(&self) -> cdr_runtime::CdrResult<Vec<u8>> {
        match self {
            Self::LivoxRosDriver2CustomMsg(msg) => livox_ros_driver2::encode::encode_to_vec(msg),
            Self::SensorMsgsImu(msg) => sensor_msgs::encode::encode_to_vec(msg),
            Self::SensorMsgsPointCloud2(msg) => sensor_msgs::encode::encode_to_vec(msg),
        }
    }
}

pub fn decode_message_by_schema(
    schema_name: &str,
    data: &[u8],
) -> cdr_runtime::CdrResult<DecodedMessage> {
    match schema_name {
        "livox_ros_driver2/msg/CustomMsg" => Ok(DecodedMessage::LivoxRosDriver2CustomMsg(
            livox_ros_driver2::decode::decode_from_bytes::<livox_ros_driver2::msg::CustomMsg>(
                data,
            )?,
        )),
        "sensor_msgs/msg/Imu" => Ok(DecodedMessage::SensorMsgsImu(
            sensor_msgs::decode::decode_from_bytes::<sensor_msgs::msg::Imu>(data)?,
        )),
        "sensor_msgs/msg/PointCloud2" => Ok(DecodedMessage::SensorMsgsPointCloud2(
            sensor_msgs::decode::decode_from_bytes::<sensor_msgs::msg::PointCloud2>(data)?,
        )),
        _ => Err(cdr_runtime::CdrError::UnknownSchema(
            schema_name.to_string(),
        )),
    }
}

#[derive(Clone, Debug)]
pub enum DecodedMessageBorrowed<'a> {
    LivoxRosDriver2CustomMsg(livox_ros_driver2::borrowed::CustomMsg<'a>),
    SensorMsgsImu(sensor_msgs::borrowed::Imu<'a>),
    SensorMsgsPointCloud2(sensor_msgs::borrowed::PointCloud2<'a>),
}

pub fn borrow_decode_message_by_schema<'a>(
    schema_name: &str,
    data: &'a [u8],
) -> cdr_runtime::CdrResult<DecodedMessageBorrowed<'a>> {
    match schema_name {
        "livox_ros_driver2/msg/CustomMsg" => Ok(DecodedMessageBorrowed::LivoxRosDriver2CustomMsg(
            livox_ros_driver2::borrow_decode::borrow_decode_from_bytes(data)?,
        )),
        "sensor_msgs/msg/Imu" => Ok(DecodedMessageBorrowed::SensorMsgsImu(
            sensor_msgs::borrow_decode::borrow_decode_from_bytes(data)?,
        )),
        "sensor_msgs/msg/PointCloud2" => Ok(DecodedMessageBorrowed::SensorMsgsPointCloud2(
            sensor_msgs::borrow_decode::borrow_decode_from_bytes(data)?,
        )),
        _ => Err(cdr_runtime::CdrError::UnknownSchema(
            schema_name.to_string(),
        )),
    }
}
