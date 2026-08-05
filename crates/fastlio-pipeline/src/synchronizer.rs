use anyhow::Result;
use fastlio_types::{ImuSample, LidarFrame, MeasureGroup};
use std::collections::VecDeque;

const TIME_EPS_SEC: f64 = 5.0e-6;

#[derive(Default)]
pub struct MeasurementSynchronizer {
    pub imu_queue: VecDeque<ImuSample>,
    pub pending_lidar: VecDeque<LidarFrame>,
    pub last_imu_time_sec: Option<f64>,
    pub last_lidar_time_sec: Option<f64>,
    pub dropped_lidar_without_begin_imu: usize,
    pub first_lidar_drop_without_begin_imu: Option<LidarDropWithoutBeginImu>,
}

#[derive(Debug, Clone, Copy)]
pub struct LidarDropWithoutBeginImu {
    pub lidar_base_time_sec: f64,
    pub lidar_end_time_sec: f64,
    pub first_imu_time_sec: f64,
}

impl MeasurementSynchronizer {
    pub fn new() -> Self {
        Self {
            imu_queue: VecDeque::new(),
            pending_lidar: VecDeque::new(),
            last_imu_time_sec: None,
            last_lidar_time_sec: None,
            dropped_lidar_without_begin_imu: 0,
            first_lidar_drop_without_begin_imu: None,
        }
    }

    fn imus_covers_lidar(&self, lidar: &LidarFrame) -> bool {
        let Some(first) = self.imu_queue.front() else {
            return false;
        };
        let Some(last) = self.imu_queue.back() else {
            return false;
        };
        first.time_stamp_sec <= lidar.base_timestamp_sec + TIME_EPS_SEC
            && last.time_stamp_sec + TIME_EPS_SEC >= lidar.end_timestamp_sec()
    }

    fn try_build_group(&mut self) -> Result<Option<MeasureGroup>> {
        loop {
            let Some(lidar) = self.pending_lidar.front() else {
                return Ok(None);
            };

            if let Some(first_imu) = self.imu_queue.front()
                && first_imu.time_stamp_sec > lidar.base_timestamp_sec + TIME_EPS_SEC
            {
                self.first_lidar_drop_without_begin_imu
                    .get_or_insert(LidarDropWithoutBeginImu {
                        lidar_base_time_sec: lidar.base_timestamp_sec,
                        lidar_end_time_sec: lidar.end_timestamp_sec(),
                        first_imu_time_sec: first_imu.time_stamp_sec,
                    });
                self.pending_lidar.pop_front();
                self.dropped_lidar_without_begin_imu += 1;
                continue;
            }

            if !self.imus_covers_lidar(lidar) {
                return Ok(None);
            }

            break;
        }

        // Front LiDAR frame exists and is covered, so pop_front() is safe here.
        let lidar = self.pending_lidar.pop_front().unwrap();

        let mut start_idx = 0;
        while start_idx + 1 < self.imu_queue.len()
            && self.imu_queue[start_idx + 1].time_stamp_sec
                < lidar.base_timestamp_sec - TIME_EPS_SEC
        {
            start_idx += 1;
        }
        let mut end_idx = 0;
        while end_idx < self.imu_queue.len()
            && self.imu_queue[end_idx].time_stamp_sec < lidar.end_timestamp_sec() - TIME_EPS_SEC
        {
            end_idx += 1;
        }

        if end_idx >= self.imu_queue.len() {
            return Ok(None);
        }

        self.imu_queue.drain(..start_idx);

        let end_idx = end_idx - start_idx;
        let imu: Vec<ImuSample> = self.imu_queue.range(..=end_idx).cloned().collect();
        let retain_idx = if end_idx > 0
            && self.imu_queue[end_idx].time_stamp_sec > lidar.end_timestamp_sec() + TIME_EPS_SEC
        {
            end_idx - 1
        } else {
            end_idx
        };
        self.imu_queue.drain(..retain_idx);
        let measure_group = MeasureGroup { imu, lidar };
        Ok(Some(measure_group))
    }

    pub fn drain_ready(&mut self) -> Result<Vec<MeasureGroup>> {
        let mut groups = Vec::new();
        while let Some(group) = self.try_build_group()? {
            groups.push(group);
        }
        Ok(groups)
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
        if lidar.end_timestamp_sec() < lidar.base_timestamp_sec {
            anyhow::bail!("lidar time begin > end");
        }
        if let Some(last_time_sec) = self.last_lidar_time_sec
            && lidar.base_timestamp_sec < last_time_sec
        {
            anyhow::bail!(
                "lidar time loop back: last base time {} > lidar base time {}",
                last_time_sec,
                lidar.base_timestamp_sec
            );
        }
        self.last_lidar_time_sec = Some(lidar.base_timestamp_sec);
        self.pending_lidar.push_back(lidar);
        self.try_build_group()
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
        let result = sync.pend_lidar(lidar(0.0, 1.0)).unwrap();
        assert!(result.is_none());
    }

    #[test]
    fn lidar_waits_when_imu_does_not_cover_end() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(0.0)).unwrap();
        sync.pend_imu(imu(0.3)).unwrap();
        assert!(sync.pend_lidar(lidar(0.0, 1.0)).unwrap().is_none());
    }

    #[test]
    fn lidar_waits_when_imu_does_not_cover_begin() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(1.0)).unwrap();
        sync.pend_imu(imu(2.0)).unwrap();
        assert!(sync.pend_lidar(lidar(0.0, 1.0)).unwrap().is_none());
    }

    #[test]
    fn lidar_succeeds_after_imu_arrives_to_cover() {
        let mut sync = MeasurementSynchronizer::new();
        assert!(sync.pend_lidar(lidar(0.0, 1.0)).unwrap().is_none());
        sync.pend_imu(imu(0.0)).unwrap();
        let group = sync.pend_imu(imu(1.0)).unwrap();
        assert!(group.is_some());
    }

    #[test]
    fn multiple_lidar_frames_can_wait_for_future_imu() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_lidar(lidar(0.0, 1.0)).unwrap();
        assert!(sync.pend_lidar(lidar(1.0, 2.0)).unwrap().is_none());
        assert_eq!(sync.pending_lidar.len(), 2);
    }

    #[test]
    fn imu_covers_lidar_at_exact_boundaries_forms_group() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(0.0)).unwrap();
        sync.pend_imu(imu(1.0)).unwrap();
        assert!(sync.pend_lidar(lidar(0.0, 1.0)).unwrap().is_some());
    }

    #[test]
    fn imu_before_lidar_immediately_forms_group() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(0.0)).unwrap();
        sync.pend_imu(imu(0.5)).unwrap();
        sync.pend_imu(imu(1.0)).unwrap();
        let group = sync.pend_lidar(lidar(0.2, 0.8)).unwrap().unwrap();
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
        let group = sync.pend_lidar(lidar(0.35, 0.45)).unwrap().unwrap();
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
        assert!(sync.pend_lidar(lidar(0.5, 1.5)).unwrap().is_none());
        sync.pend_imu(imu(0.0)).unwrap();
        assert!(sync.pend_imu(imu(0.3)).unwrap().is_none());
        sync.pend_imu(imu(0.8)).unwrap();
        assert!(sync.pend_imu(imu(1.2)).unwrap().is_none());
        let group = sync.pend_imu(imu(2.0)).unwrap().unwrap();
        // start_idx discards imu[0]=0.0 (too far before begin=0.5),
        // group contains imu[1]=0.3, imu[2]=0.8, imu[3]=1.2, imu[4]=2.0
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
        let group = sync.pend_lidar(lidar(0.15, 0.45)).unwrap().unwrap();
        assert_eq!(group.lidar.base_timestamp_sec, 0.15);
        assert_eq!(group.lidar.end_timestamp_sec(), 0.45);
        assert!(!group.imu.is_empty());
        // Remaining IMU in queue for future sync
        assert!(
            !sync.imu_queue.is_empty(),
            "expected remaining IMU samples, got {}",
            sync.imu_queue.len()
        );
    }

    /// scan spans many imu intervals
    #[test]
    fn scan_spans_many_imu_intervals() {
        let mut sync = MeasurementSynchronizer::new();
        for i in 0..=200 {
            sync.pend_imu(imu(i as f64 * 0.005)).unwrap();
        }
        let group = sync.pend_lidar(lidar(0.1, 0.6)).unwrap().unwrap();
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
        let group = sync.pend_lidar(lidar(1.0, 2.0)).unwrap().unwrap();
        // Boundary IMU at 2.0 must be included
        assert!(
            group.imu.iter().any(|s| s.time_stamp_sec >= 2.0),
            "boundary IMU at scan end not found in group"
        );
        assert_eq!(group.imu.last().unwrap().time_stamp_sec, 2.0);
        // IMU at 2.0, 2.1 must remain in queue
        assert_eq!(sync.imu_queue.len(), 2);
        assert_eq!(sync.imu_queue[0].time_stamp_sec, 2.0);
        assert_eq!(sync.imu_queue[1].time_stamp_sec, 2.1);
    }

    #[test]
    fn lidar_timestamp_regression_across_calls_is_rejected() {
        let mut sync = MeasurementSynchronizer::new();
        // First lidar at t=1.0..2.0, then processed (with IMU coverage)
        sync.pend_imu(imu(1.0)).unwrap();
        sync.pend_imu(imu(2.0)).unwrap();
        let group = sync.pend_lidar(lidar(1.0, 2.0)).unwrap();
        assert!(group.is_some());
        // Second lidar with earlier base timestamp should be rejected
        assert!(sync.pend_lidar(lidar(0.5, 1.5)).is_err());
    }

    #[test]
    fn missing_imu_boundary_returns_none() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(0.5)).unwrap();
        sync.pend_imu(imu(0.7)).unwrap();
        let result = sync.pend_lidar(lidar(0.5, 1.0)).unwrap();
        assert!(
            result.is_none(),
            "expected no group when IMU does not cover scan end"
        );
        assert!(
            !sync.pending_lidar.is_empty(),
            "pending lidar should remain when sync is not yet possible"
        );
    }

    #[test]
    fn drain_ready_returns_all_covered_lidar_frames_in_order() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_lidar(lidar(0.0, 1.0)).unwrap();
        sync.pend_lidar(lidar(1.0, 2.0)).unwrap();
        let mut groups = Vec::new();
        if let Some(group) = sync.pend_imu(imu(0.0)).unwrap() {
            groups.push(group);
        }
        if let Some(group) = sync.pend_imu(imu(1.0)).unwrap() {
            groups.push(group);
        }
        if let Some(group) = sync.pend_imu(imu(2.0)).unwrap() {
            groups.push(group);
        }

        groups.extend(sync.drain_ready().unwrap());

        assert_eq!(groups.len(), 2);
        assert_eq!(groups[0].lidar.base_timestamp_sec, 0.0);
        assert_eq!(groups[1].lidar.base_timestamp_sec, 1.0);
    }

    #[test]
    fn adjacent_lidar_frames_keep_left_imu_boundary() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(0.00)).unwrap();
        sync.pend_imu(imu(0.05)).unwrap();
        sync.pend_imu(imu(0.105)).unwrap();
        sync.pend_lidar(lidar(0.00, 0.10)).unwrap().unwrap();

        sync.pend_imu(imu(0.15)).unwrap();
        sync.pend_imu(imu(0.205)).unwrap();
        let group = sync.pend_lidar(lidar(0.10, 0.20)).unwrap().unwrap();

        assert_eq!(sync.dropped_lidar_without_begin_imu, 0);
        assert!(group.imu.first().unwrap().time_stamp_sec <= 0.10);
        assert!(group.imu.last().unwrap().time_stamp_sec >= 0.20);
    }

    #[test]
    fn microsecond_begin_gap_is_tolerated() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_lidar(lidar(10.0, 10.1)).unwrap();
        sync.pend_imu(imu(10.0 + 2.0e-6)).unwrap();
        let group = sync.pend_imu(imu(10.1)).unwrap().unwrap();

        assert_eq!(sync.dropped_lidar_without_begin_imu, 0);
        assert_eq!(group.lidar.base_timestamp_sec, 10.0);
        assert!(group.imu.first().unwrap().time_stamp_sec > group.lidar.base_timestamp_sec);
    }

    #[test]
    fn stale_lidar_before_first_imu_is_dropped_without_blocking_later_lidar() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_lidar(lidar(0.0, 0.5)).unwrap();
        sync.pend_lidar(lidar(1.0, 2.0)).unwrap();
        assert!(sync.pend_imu(imu(1.0)).unwrap().is_none());
        let group = sync.pend_imu(imu(2.0)).unwrap().unwrap();

        assert_eq!(sync.dropped_lidar_without_begin_imu, 1);
        assert_eq!(group.lidar.base_timestamp_sec, 1.0);
    }

    #[test]
    fn empty_lidar_frame_still_syncs() {
        let mut sync = MeasurementSynchronizer::new();
        sync.pend_imu(imu(0.0)).unwrap();
        sync.pend_imu(imu(1.0)).unwrap();
        let group = sync.pend_lidar(lidar(0.2, 0.8)).unwrap().unwrap();
        assert!(group.lidar.points.is_empty());
        assert!(!group.imu.is_empty());
        assert_eq!(group.lidar.base_timestamp_sec, 0.2);
        assert_eq!(group.lidar.end_timestamp_sec(), 0.8);
    }
}
