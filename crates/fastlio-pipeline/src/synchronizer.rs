use anyhow::Result;
use fastlio_types::{ImuSample, LidarFrame, MeasureGroup};
use std::collections::VecDeque;

#[derive(Default)]
pub struct MeasurementSynchronizer {
    pub imu_queue: VecDeque<ImuSample>,
    pub pending_lidar: Option<LidarFrame>,
    pub last_imu_time_sec: Option<f64>,
}

impl MeasurementSynchronizer {
    pub fn new() -> Self {
        Self {
            imu_queue: VecDeque::new(),
            pending_lidar: None,
            last_imu_time_sec: None,
        }
    }

    fn imus_covers_lidar(&self, lidar: &LidarFrame) -> bool {
        let Some(first) = self.imu_queue.front() else {
            return false;
        };
        let Some(last) = self.imu_queue.back() else {
            return false;
        };
        first.time_stamp_sec <= lidar.base_timestamp_sec
            && last.time_stamp_sec >= lidar.end_timestamp_sec
    }

    fn try_build_group(&mut self) -> Result<Option<MeasureGroup>> {
        let Some(lidar) = self.pending_lidar.as_ref() else {
            return Ok(None);
        };
        if !self.imus_covers_lidar(lidar) {
            return Ok(None);
        }
        // lidar frame must exist here, so unwrap is safe here.
        let lidar = self.pending_lidar.take().unwrap();

        let mut start_idx = 0;
        while start_idx + 1 < self.imu_queue.len()
            && self.imu_queue[start_idx + 1].time_stamp_sec < lidar.base_timestamp_sec
        {
            start_idx += 1;
        }
        let mut end_idx = 0;
        while end_idx < self.imu_queue.len()
            && self.imu_queue[end_idx + 1].time_stamp_sec < lidar.end_timestamp_sec
        {
            end_idx += 1;
        }
        // let imu: Vec<_> = self.imu_queue.range(start_idx..=end_idx).cloned().collect();
        let imu = self.imu_queue.drain(start_idx..=end_idx).collect();

        let measure_group = MeasureGroup {
            lidar_beg_time: lidar.base_timestamp_sec,
            lidar_end_time: lidar.end_timestamp_sec,
            imu,
            lidar,
        };
        Ok(Some(measure_group))
    }

    pub fn pend_imu(&mut self, imu: ImuSample) -> Result<Option<MeasureGroup>> {
        if let Some(last_time_sec) = self.last_imu_time_sec
            && imu.time_stamp_sec < last_time_sec
        {
            anyhow::bail!(
                "imu time loop back: last time sec {} > imu_stamp_sec {}",
                last_time_sec,
                imu.time_stamp_sec
            );
        }
        self.last_imu_time_sec = Some(imu.time_stamp_sec);
        self.imu_queue.push_back(imu);
        self.try_build_group()
    }

    pub fn pend_lidar(&mut self, lidar: LidarFrame) -> Result<Option<MeasureGroup>> {
        if self.pending_lidar.is_some() {
            anyhow::bail!("last lidar frame not ready to be processed.");
        }
        if lidar.end_timestamp_sec < lidar.base_timestamp_sec {
            anyhow::bail!("lidar time begin > end");
        }
        self.pending_lidar = Some(lidar);
        self.try_build_group()
    }
}
