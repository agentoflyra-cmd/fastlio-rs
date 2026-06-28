/// transfer from std_msgs::msg::Header to f64 sec.
pub fn transfer_from_header(sec: i32, nano_sec: u32) -> f64 {
    let nano_sec = f64::from(nano_sec) / 1e9;
    let sec = f64::from(sec);
    nano_sec + sec
}
