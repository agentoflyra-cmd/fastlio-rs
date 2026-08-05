mod app;
mod camera;
mod math;
mod point_cloud;
mod scene;
mod stream;
mod wgpu_point_cloud;

use app::{AppConfig, AppState};

fn main() -> iced::Result {
    let config = AppConfig::parse(std::env::args().skip(1).collect());
    iced::application(
        move || AppState::new(config.clone()),
        AppState::update,
        AppState::view,
    )
    .title("PointCloud Viewer")
    .subscription(AppState::subscription)
    .run()
}
