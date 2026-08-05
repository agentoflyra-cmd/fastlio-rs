use anyhow::Result;
use fastlio_types::{ImuSample, LidarFrame, MeasureGroup};
use std::collections::VecDeque;

const TIME_EPS_SEC: f64 = 5.0e-6;

#[derive(Default)]
pub struct MeasurementSynchronizer {
    pub imu_buffer: VecDeque<ImuSample>,
    pub lidar_buffer: VecDeque<LidarFrame>,
    pub ready_groups: VecDeque<MeasureGroup>,
    pub last_imu_timestamp_sec: Option<f64>,
    pub last_lidar_base_timestamp_sec: Option<f64>,
    pub dropped_lidar_before_first_imu: usize,
    pub first_lidar_drop_before_first_imu: Option<LidarDropBeforeFirstImu>,
}

#[derive(Debug, Clone, Copy)]
pub struct LidarDropBeforeFirstImu {
    pub lidar_base_time_sec: f64,
    pub lidar_end_time_sec: f64,
    pub first_imu_time_sec: f64,
}

impl MeasurementSynchronizer {
    pub fn new() -> Self {
        Self {
            imu_buffer: VecDeque::new(),
            lidar_buffer: VecDeque::new(),
            ready_groups: VecDeque::new(),
            last_imu_timestamp_sec: None,
            last_lidar_base_timestamp_sec: None,
            dropped_lidar_before_first_imu: 0,
            first_lidar_drop_before_first_imu: None,
        }
    }

    fn imus_covers_lidar(&self, lidar: &LidarFrame) -> bool {
        let Some(last) = self.imu_buffer.back() else {
            return false;
        };
        last.time_stamp_sec + TIME_EPS_SEC >= lidar.end_timestamp_sec()
    }

    pub fn pend_imu(&mut self, imu: ImuSample) -> Result<()> {
        if let Some(last_imu_timestamp_sec) = self.last_imu_timestamp_sec
            && imu.time_stamp_sec < last_imu_timestamp_sec
        {
            anyhow::bail!(
                "imu time loop back: last time sec {} > imu_stamp_sec {}",
                last_imu_timestamp_sec,
                imu.time_stamp_sec
            );
        }
        self.last_imu_timestamp_sec = Some(imu.time_stamp_sec);
        self.imu_buffer.push_back(imu);
        self.push_ready_group()
    }

    pub fn pend_lidar(&mut self, lidar: LidarFrame) -> Result<()> {
        if lidar.end_timestamp_sec() < lidar.base_timestamp_sec {
            anyhow::bail!("lidar time begin > end");
        }
        if let Some(last_lidar_base_timestamp_sec) = self.last_lidar_base_timestamp_sec
            && lidar.base_timestamp_sec < last_lidar_base_timestamp_sec
        {
            anyhow::bail!(
                "lidar time loop back: last base time {} > lidar base time {}",
                last_lidar_base_timestamp_sec,
                lidar.base_timestamp_sec
            );
        }
        self.last_lidar_base_timestamp_sec = Some(lidar.base_timestamp_sec);
        self.lidar_buffer.push_back(lidar);
        self.push_ready_group()
    }

    fn push_ready_group(&mut self) -> Result<()> {
        while let Some(lidar) = self.lidar_buffer.front() {
            if let Some(first_imu) = self.imu_buffer.front()
                && first_imu.time_stamp_sec > lidar.end_timestamp_sec() + TIME_EPS_SEC
            {
                self.first_lidar_drop_before_first_imu
                    .get_or_insert(LidarDropBeforeFirstImu {
                        lidar_base_time_sec: lidar.base_timestamp_sec,
                        lidar_end_time_sec: lidar.end_timestamp_sec(),
                        first_imu_time_sec: first_imu.time_stamp_sec,
                    });
                let Some(_) = self.lidar_buffer.pop_front() else {
                    break;
                };
                self.dropped_lidar_before_first_imu += 1;
                continue;
            }

            if !self.imus_covers_lidar(lidar) {
                break;
            }

            self.try_build_group();
        }
        Ok(())
    }

    fn try_build_group(&mut self) {
        // lidar frame must exist here, so unwrap is safe here.
        let lidar = self.lidar_buffer.pop_front().unwrap();
        let mut start_idx = 0;
        while start_idx + 1 < self.imu_buffer.len()
            && self.imu_buffer[start_idx + 1].time_stamp_sec < lidar.base_timestamp_sec
        {
            start_idx += 1;
        }
        let mut end_idx = 0;
        while end_idx < self.imu_buffer.len()
            && self.imu_buffer[end_idx].time_stamp_sec + TIME_EPS_SEC < lidar.end_timestamp_sec()
        {
            end_idx += 1;
        }

        self.imu_buffer.drain(..start_idx);

        let end_idx = end_idx - start_idx;
        let end_imu = self.imu_buffer[end_idx].clone();
        let mut imu: Vec<ImuSample> = self.imu_buffer.drain(..end_idx).collect();

        imu.push(end_imu);
        self.ready_groups.push_back(MeasureGroup { imu, lidar });
    }

    pub fn pop_ready_group(&mut self) -> Option<MeasureGroup> {
        self.ready_groups.pop_front()
    }

    pub fn drain_ready(&mut self) -> Vec<MeasureGroup> {
        let mut groups = Vec::new();
        while let Some(ready) = self.pop_ready_group() {
            groups.push(ready);
        }
        groups
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use fastlio_types::Vec3;

    fn imu(sec: f64) -> ImuSample {
        ImuSample {
            time_stamp_sec: sec,
            gyro: Vec3::zeros(),
            accel: Vec3::zeros(),
        }
    }

    fn lidar(begin: f64, end: f64) -> LidarFrame {
        LidarFrame::new(begin, end, vec![])
    }

    // --- Existing tests ---

    #[test]
    fn imu_time_must_be_monotonic() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(1.0)).unwrap();
        sync.pend_imu(imu(2.0)).unwrap();
        assert!(sync.pend_imu(imu(1.5)).is_err());
    }

    #[test]
    fn imu_time_equal_is_allowed() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(1.0)).unwrap();
        assert!(sync.pend_imu(imu(1.0)).is_ok());
    }

    #[test]
    fn lidar_begin_must_be_before_end() {
        let mut sync = MeasurementSynchronizer::new();
        assert!(sync.pend_lidar(lidar(2.0, 1.0)).is_err());
    }

    #[test]
    fn lidar_waits_when_imu_queue_empty() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_lidar(lidar(0.0, 1.0)).unwrap();
        assert!(sync.drain_ready().is_empty());
    }

    #[test]
    fn lidar_waits_when_imu_does_not_cover_end() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(0.0)).unwrap();
        sync.pend_imu(imu(0.3)).unwrap();
        sync.pend_lidar(lidar(0.0, 1.0)).unwrap();
        assert!(sync.drain_ready().is_empty());
    }

    #[test]
    fn lidar_begin_before_first_imu_still_syncs() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(1.0)).unwrap();
        sync.pend_imu(imu(2.0)).unwrap();
        sync.pend_lidar(lidar(0.0, 1.0)).unwrap();
        let group = sync.pop_ready_group().unwrap();
        assert_eq!(group.imu.first().unwrap().time_stamp_sec, 1.0);
    }

    #[test]
    fn stale_lidar_before_first_imu_is_dropped_without_blocking_later_lidar() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_lidar(lidar(0.0, 0.5)).unwrap();
        sync.pend_lidar(lidar(1.0, 2.0)).unwrap();
        assert_eq!(sync.lidar_buffer.len(), 2);
        sync.pend_imu(imu(1.0)).unwrap();
        sync.pend_imu(imu(2.0)).unwrap();
        let group = sync.pop_ready_group().unwrap();
        assert_eq!(group.lidar.base_timestamp_sec, 1.0);
        assert_eq!(sync.dropped_lidar_before_first_imu, 1);
        let drop = sync.first_lidar_drop_before_first_imu.unwrap();
        assert_eq!(drop.lidar_base_time_sec, 0.0);
        assert_eq!(drop.lidar_end_time_sec, 0.5);
        assert_eq!(drop.first_imu_time_sec, 1.0);
    }

    #[test]
    fn imu_end_within_tolerance_is_accepted() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(0.0)).unwrap();
        sync.pend_imu(imu(1.0 - 1e-6)).unwrap();
        sync.pend_lidar(lidar(0.0, 1.0)).unwrap();
        let group = sync.pop_ready_group().unwrap();
        assert_eq!(group.imu.last().unwrap().time_stamp_sec, 1.0 - 1e-6);
    }

    #[test]
    fn lidar_succeeds_after_imu_arrives_to_cover() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_lidar(lidar(0.0, 1.0)).unwrap();
        assert!(sync.drain_ready().is_empty());
        sync.pend_imu(imu(0.0)).unwrap();
        sync.pend_imu(imu(1.0)).unwrap();
        let group = sync.pop_ready_group();
        assert!(group.is_some());
    }

    #[test]
    fn multiple_lidars_can_queue_and_process_in_order() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_lidar(lidar(0.0, 1.0)).unwrap();
        sync.pend_lidar(lidar(1.0, 2.0)).unwrap();
        assert_eq!(sync.lidar_buffer.len(), 2);
        sync.pend_imu(imu(0.0)).unwrap();
        sync.pend_imu(imu(1.0)).unwrap();
        let g1 = sync.pop_ready_group().unwrap();
        assert_eq!(g1.lidar.base_timestamp_sec, 0.0);
        sync.pend_imu(imu(2.0)).unwrap();
        let g2 = sync.pop_ready_group().unwrap();
        assert_eq!(g2.lidar.base_timestamp_sec, 1.0);
    }

    #[test]
    fn imu_covers_lidar_at_exact_boundaries_forms_group() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(0.0)).unwrap();
        sync.pend_imu(imu(1.0)).unwrap();
        sync.pend_lidar(lidar(0.0, 1.0)).unwrap();
        assert!(sync.pop_ready_group().is_some());
    }

    #[test]
    fn imu_before_lidar_immediately_forms_group() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(0.0)).unwrap();
        sync.pend_imu(imu(0.5)).unwrap();
        sync.pend_imu(imu(1.0)).unwrap();
        sync.pend_lidar(lidar(0.2, 0.8)).unwrap();
        let group = sync.pop_ready_group().unwrap();
        assert_eq!(group.lidar.base_timestamp_sec, 0.2);
        assert_eq!(group.lidar.end_timestamp_sec(), 0.8);
    }

    // --- New tests per AGENTS.md sync requirements ---

    /// normal IMU + LiDAR time serial
    #[test]
    fn normal_imu_lidar_sequence_forms_group() {
        let mut sync = MeasurementSynchronizer::new();
        for i in 0..=100 {
            sync.pend_imu(imu(i as f64 * 0.01)).unwrap();
        }
        sync.pend_lidar(lidar(0.35, 0.45)).unwrap();
        let group = sync.pop_ready_group().unwrap();
        assert!(group.imu.first().unwrap().time_stamp_sec <= 0.35);
        assert!(group.imu.last().unwrap().time_stamp_sec >= 0.45);
        assert!(group.imu.len() >= 3);
        assert_eq!(group.lidar.base_timestamp_sec, 0.35);
        assert_eq!(group.lidar.end_timestamp_sec(), 0.45);
    }

    /// LiDAR come first
    #[test]
    fn lidar_first_waits_for_sufficient_imu() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_lidar(lidar(0.5, 1.5)).unwrap();
        assert!(sync.drain_ready().is_empty());
        sync.pend_imu(imu(0.0)).unwrap();
        sync.pend_imu(imu(0.3)).unwrap();
        sync.pend_imu(imu(0.8)).unwrap();
        sync.pend_imu(imu(1.2)).unwrap();
        assert!(sync.drain_ready().is_empty());
        sync.pend_imu(imu(2.0)).unwrap();
        let group = sync.pop_ready_group().unwrap();
        assert_eq!(group.imu.len(), 4);
        assert!(group.imu.last().unwrap().time_stamp_sec >= 1.5);
    }

    /// IMU come first and retain imu
    #[test]
    fn imu_first_accumulates_and_drains_correctly() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(0.0)).unwrap();
        sync.pend_imu(imu(0.1)).unwrap();
        sync.pend_imu(imu(0.2)).unwrap();
        sync.pend_imu(imu(0.3)).unwrap();
        sync.pend_imu(imu(0.4)).unwrap();
        sync.pend_imu(imu(0.5)).unwrap();
        sync.pend_imu(imu(0.6)).unwrap();
        sync.pend_lidar(lidar(0.15, 0.45)).unwrap();
        let group = sync.pop_ready_group().unwrap();
        assert_eq!(group.lidar.base_timestamp_sec, 0.15);
        assert_eq!(group.lidar.end_timestamp_sec(), 0.45);
        assert!(!group.imu.is_empty());
        assert!(
            !sync.imu_buffer.is_empty(),
            "expected remaining IMU samples, got {}",
            sync.imu_buffer.len()
        );
    }

    /// scan spans many imu intervals
    #[test]
    fn scan_spans_many_imu_intervals() {
        let mut sync = MeasurementSynchronizer::new();
        for i in 0..=200 {
            sync.pend_imu(imu(i as f64 * 0.005)).unwrap();
        }
        sync.pend_lidar(lidar(0.1, 0.6)).unwrap();
        let group = sync.pop_ready_group().unwrap();
        assert!(
            group.imu.len() >= 50,
            "expected >=50 IMU samples spanning 0.5s at 200Hz, got {}",
            group.imu.len()
        );
        assert!(group.imu.first().unwrap().time_stamp_sec <= 0.1);
        assert!(group.imu.last().unwrap().time_stamp_sec >= 0.6);
    }

    #[test]
    fn boundary_imu_after_scan_end_included_in_group() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(0.9)).unwrap();
        sync.pend_imu(imu(1.0)).unwrap();
        sync.pend_imu(imu(1.5)).unwrap();
        sync.pend_imu(imu(2.0)).unwrap(); // boundary-after
        sync.pend_imu(imu(2.1)).unwrap(); // stays in queue
        sync.pend_lidar(lidar(1.0, 2.0)).unwrap();
        let group = sync.pop_ready_group().unwrap();
        assert!(
            group.imu.iter().any(|s| s.time_stamp_sec >= 2.0),
            "boundary IMU at scan end not found in group"
        );
        assert_eq!(group.imu.last().unwrap().time_stamp_sec, 2.0);
        assert_eq!(sync.imu_buffer.len(), 2);
        assert_eq!(sync.imu_buffer[0].time_stamp_sec, 2.0);
        assert_eq!(sync.imu_buffer[1].time_stamp_sec, 2.1);
    }

    #[test]
    fn lidar_timestamp_regression_across_calls_is_rejected() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(1.0)).unwrap();
        sync.pend_imu(imu(2.0)).unwrap();
        sync.pend_lidar(lidar(1.0, 2.0)).unwrap();
        assert!(sync.pop_ready_group().is_some());
        assert!(sync.pend_lidar(lidar(0.5, 1.5)).is_err());
    }

    #[test]
    fn missing_imu_boundary_returns_none() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(0.5)).unwrap();
        sync.pend_imu(imu(0.7)).unwrap();
        sync.pend_lidar(lidar(0.5, 1.0)).unwrap();
        assert!(
            sync.drain_ready().is_empty(),
            "expected no group when IMU does not cover scan end"
        );
        assert!(
            !sync.lidar_buffer.is_empty(),
            "pending lidar should remain when sync is not yet possible"
        );
    }

    #[test]
    fn empty_lidar_frame_still_syncs() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(0.0)).unwrap();
        sync.pend_imu(imu(1.0)).unwrap();
        sync.pend_lidar(lidar(0.2, 0.8)).unwrap();
        let group = sync.pop_ready_group().unwrap();
        assert!(group.lidar.points.is_empty());
        assert!(!group.imu.is_empty());
        assert_eq!(group.lidar.base_timestamp_sec, 0.2);
        assert_eq!(group.lidar.end_timestamp_sec(), 0.8);
    }
}
