use std::io::{BufRead, BufReader};
use std::net::{TcpListener, TcpStream};
use std::sync::mpsc::{self, Receiver};
use std::thread;
use std::time::Duration;

use crate::point_cloud::PointXYZI;

#[derive(Debug, Clone)]
pub enum StreamMessage {
    Clear,
    Points(Vec<PointXYZI>),
    Status(String),
}

pub struct StreamReceiver {
    receiver: Receiver<StreamMessage>,
}

impl StreamReceiver {
    pub fn new(receiver: Receiver<StreamMessage>) -> Self {
        Self { receiver }
    }

    pub fn drain(&self) -> Vec<StreamMessage> {
        let mut messages = Vec::new();
        while let Ok(message) = self.receiver.try_recv() {
            messages.push(message);
        }
        messages
    }
}

pub fn spawn_point_stream_listener(addr: String) -> StreamReceiver {
    let (sender, receiver) = mpsc::channel();
    thread::spawn(move || {
        let listener = match TcpListener::bind(&addr) {
            Ok(listener) => listener,
            Err(err) => {
                let _ = sender.send(StreamMessage::Status(format!(
                    "failed to bind {addr}: {err}"
                )));
                return;
            }
        };
        let _ = sender.send(StreamMessage::Status(format!("listening on {addr}")));

        for stream in listener.incoming() {
            match stream {
                Ok(stream) => handle_stream(stream, &sender),
                Err(err) => {
                    let _ = sender.send(StreamMessage::Status(format!(
                        "stream accept failed: {err}"
                    )));
                    thread::sleep(Duration::from_millis(50));
                }
            }
        }
    });

    StreamReceiver::new(receiver)
}

fn handle_stream(stream: TcpStream, sender: &mpsc::Sender<StreamMessage>) {
    let peer = stream.peer_addr().ok();
    let _ = sender.send(StreamMessage::Status(match peer {
        Some(peer) => format!("connected: {peer}"),
        None => "connected".to_string(),
    }));

    let reader = BufReader::new(stream);
    let mut points = Vec::with_capacity(4096);
    for line in reader.lines() {
        let Ok(line) = line else {
            break;
        };
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        if line == "clear" {
            flush_points(sender, &mut points);
            let _ = sender.send(StreamMessage::Clear);
            continue;
        }
        if line == "flush" {
            flush_points(sender, &mut points);
            continue;
        }
        if let Some(point) = parse_point_line(line) {
            points.push(point);
            if points.len() >= 4096 {
                flush_points(sender, &mut points);
            }
        }
    }
    flush_points(sender, &mut points);
    let _ = sender.send(StreamMessage::Status("disconnected".to_string()));
}

fn flush_points(sender: &mpsc::Sender<StreamMessage>, points: &mut Vec<PointXYZI>) {
    if points.is_empty() {
        return;
    }
    let mut batch = Vec::new();
    std::mem::swap(points, &mut batch);
    let _ = sender.send(StreamMessage::Points(batch));
}

fn parse_point_line(line: &str) -> Option<PointXYZI> {
    let mut fields = line.split_whitespace();
    if fields.next()? != "point" {
        return None;
    }
    let x = fields.next()?.parse::<f32>().ok()?;
    let y = fields.next()?.parse::<f32>().ok()?;
    let z = fields.next()?.parse::<f32>().ok()?;
    let intensity = fields.next()?.parse::<f32>().ok()?;
    Some(PointXYZI {
        x,
        y,
        z,
        intensity,
        normal: None,
    })
}

#[cfg(test)]
mod tests {
    use super::parse_point_line;

    #[test]
    fn parses_point_stream_line() {
        let point = parse_point_line("point 1.0 -2.0 3.5 42.0").unwrap();

        assert_eq!(point.x, 1.0);
        assert_eq!(point.y, -2.0);
        assert_eq!(point.z, 3.5);
        assert_eq!(point.intensity, 42.0);
        assert!(point.normal.is_none());
    }

    #[test]
    fn rejects_non_point_stream_line() {
        assert!(parse_point_line("flush").is_none());
        assert!(parse_point_line("point 1 2").is_none());
    }
}
