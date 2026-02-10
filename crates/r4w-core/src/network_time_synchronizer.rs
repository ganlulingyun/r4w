//! High-precision time synchronization for distributed SDR coherence (NTP/PTP-style).
//!
//! This module implements a Kalman-filter-based clock synchronization engine
//! that estimates offset and drift between a local clock and a remote reference.
//! It supports locked, holdover, and free-running modes, and can tag IQ samples
//! with absolute timestamps for coherent distributed processing.
//!
//! # Example
//!
//! ```
//! use r4w_core::network_time_synchronizer::{
//!     NetworkTimeSync, TimeSyncConfig, TimestampPair,
//! };
//!
//! let config = TimeSyncConfig {
//!     filter_bandwidth: 0.1,
//!     max_offset_s: 1.0,
//!     drift_rate_ppb_limit: 1000.0,
//!     sync_interval_s: 1.0,
//! };
//! let mut sync = NetworkTimeSync::new(config);
//!
//! // Feed timestamp pairs (local, remote) from NTP/PTP exchanges
//! sync.process_timestamp_pair(TimestampPair {
//!     local_time_s: 1000.0,
//!     remote_time_s: 1000.0001,
//! });
//! sync.process_timestamp_pair(TimestampPair {
//!     local_time_s: 1001.0,
//!     remote_time_s: 1001.0001,
//! });
//!
//! // Query estimated offset and corrected time
//! let offset = sync.estimate_offset();
//! let corrected = sync.compensate_timestamp(1002.0);
//! assert!((corrected - 1002.0).abs() < 0.01);
//!
//! // Tag an IQ sample with an absolute timestamp
//! let abs_time = sync.tag_sample(48000, 48000.0);
//! assert!(abs_time > 0.0);
//! ```

/// Quality of the synchronization lock.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SyncQuality {
    /// Actively receiving and tracking sync messages.
    Locked,
    /// No recent sync messages; extrapolating with estimated drift.
    Holdover,
    /// No synchronization information available; free-running local clock.
    Freerunning,
}

/// Configuration for the time synchronization engine.
#[derive(Debug, Clone)]
pub struct TimeSyncConfig {
    /// Kalman filter bandwidth (process noise scaling). Smaller = smoother.
    pub filter_bandwidth: f64,
    /// Maximum allowable offset in seconds before rejecting a measurement.
    pub max_offset_s: f64,
    /// Maximum allowable drift rate in parts-per-billion.
    pub drift_rate_ppb_limit: f64,
    /// Expected interval between sync exchanges in seconds.
    pub sync_interval_s: f64,
}

impl Default for TimeSyncConfig {
    fn default() -> Self {
        Self {
            filter_bandwidth: 0.1,
            max_offset_s: 1.0,
            drift_rate_ppb_limit: 1000.0,
            sync_interval_s: 1.0,
        }
    }
}

/// A pair of timestamps: one from the local clock, one from the remote reference.
#[derive(Debug, Clone, Copy)]
pub struct TimestampPair {
    /// Local clock reading at the moment of exchange (seconds).
    pub local_time_s: f64,
    /// Remote (reference) clock reading at the moment of exchange (seconds).
    pub remote_time_s: f64,
}

/// Current state of the synchronized clock.
#[derive(Debug, Clone)]
pub struct ClockState {
    /// Estimated offset: local - remote (seconds). Positive means local is ahead.
    pub offset_s: f64,
    /// Estimated drift rate in parts-per-billion.
    pub drift_ppb: f64,
    /// RMS jitter of recent residuals (seconds).
    pub jitter_rms_s: f64,
    /// Local time of the last successful sync update (seconds).
    pub last_sync_time_s: f64,
    /// Current synchronization quality.
    pub quality: SyncQuality,
}

/// Main synchronization engine using a 2-state Kalman filter (offset, drift).
///
/// State vector: x = [offset_s, drift_rate_s_per_s]
/// Measurement: z = local_time - remote_time (observed offset)
pub struct NetworkTimeSync {
    config: TimeSyncConfig,
    // Kalman state: [offset, drift_rate (s/s)]
    x: [f64; 2],
    // Kalman covariance (2x2 stored as [p00, p01, p10, p11])
    p: [f64; 4],
    // Reference time for drift extrapolation
    t_ref: f64,
    // Clock state cache
    state: ClockState,
    // Number of measurements processed
    measurement_count: u64,
    // Recent residuals for jitter estimation (circular buffer)
    residuals: Vec<f64>,
    residual_idx: usize,
    // Whether the filter has been initialized
    initialized: bool,
}

impl NetworkTimeSync {
    /// Create a new synchronization engine with the given configuration.
    pub fn new(config: TimeSyncConfig) -> Self {
        Self {
            state: ClockState {
                offset_s: 0.0,
                drift_ppb: 0.0,
                jitter_rms_s: 0.0,
                last_sync_time_s: 0.0,
                quality: SyncQuality::Freerunning,
            },
            x: [0.0; 2],
            // Initial covariance: large uncertainty
            p: [1.0, 0.0, 0.0, 1e-6],
            t_ref: 0.0,
            measurement_count: 0,
            residuals: vec![0.0; 32],
            residual_idx: 0,
            initialized: false,
            config,
        }
    }

    /// Process a new timestamp pair from a sync exchange.
    ///
    /// Updates the Kalman filter estimate of offset and drift. Measurements
    /// that exceed `max_offset_s` (after initial lock) are rejected as outliers.
    pub fn process_timestamp_pair(&mut self, pair: TimestampPair) {
        let measured_offset = pair.local_time_s - pair.remote_time_s;

        if !self.initialized {
            // First measurement: initialize state directly
            self.x[0] = measured_offset;
            self.x[1] = 0.0;
            self.t_ref = pair.local_time_s;
            self.p = [1.0, 0.0, 0.0, 1e-6];
            self.initialized = true;
            self.measurement_count = 1;
            self.state.offset_s = measured_offset;
            self.state.drift_ppb = 0.0;
            self.state.last_sync_time_s = pair.local_time_s;
            self.state.quality = SyncQuality::Locked;
            self.state.jitter_rms_s = 0.0;
            return;
        }

        let dt = pair.local_time_s - self.state.last_sync_time_s;
        if dt <= 0.0 {
            return; // Ignore out-of-order or duplicate timestamps
        }

        // --- Kalman Predict ---
        // State transition: offset += drift * dt
        // F = [[1, dt], [0, 1]]
        let x_pred_0 = self.x[0] + self.x[1] * dt;
        let x_pred_1 = self.x[1];

        // Process noise: scaled by filter_bandwidth and dt
        let q_scale = self.config.filter_bandwidth;
        let q00 = q_scale * dt * dt * dt / 3.0;
        let q01 = q_scale * dt * dt / 2.0;
        let q11 = q_scale * dt;

        // P_pred = F * P * F^T + Q
        let p00 = self.p[0] + dt * self.p[2] + dt * self.p[1] + dt * dt * self.p[3] + q00;
        let p01 = self.p[1] + dt * self.p[3] + q01;
        let p10 = self.p[2] + dt * self.p[3] + q01;
        let p11 = self.p[3] + q11;

        // --- Outlier rejection ---
        let innovation = measured_offset - x_pred_0;
        // H = [1, 0], so S = P_pred[0][0] + R
        let r = 1e-6_f64.max(self.state.jitter_rms_s * self.state.jitter_rms_s);
        let s = p00 + r;

        if self.measurement_count > 2 && innovation.abs() > self.config.max_offset_s {
            // Reject this measurement but still update time tracking
            return;
        }

        // --- Kalman Update ---
        // K = P_pred * H^T / S  =>  K = [p00/s, p10/s]
        let k0 = p00 / s;
        let k1 = p10 / s;

        self.x[0] = x_pred_0 + k0 * innovation;
        self.x[1] = x_pred_1 + k1 * innovation;

        // Clamp drift to limit
        let drift_limit = self.config.drift_rate_ppb_limit * 1e-9;
        self.x[1] = self.x[1].clamp(-drift_limit, drift_limit);

        // P = (I - K*H) * P_pred
        self.p[0] = (1.0 - k0) * p00;
        self.p[1] = (1.0 - k0) * p01;
        self.p[2] = -k1 * p00 + p10;
        self.p[3] = -k1 * p01 + p11;

        // Update reference time
        self.t_ref = pair.local_time_s;

        // Track residuals for jitter estimation
        let residual = measured_offset - self.x[0];
        let idx = self.residual_idx % self.residuals.len();
        self.residuals[idx] = residual;
        self.residual_idx += 1;

        self.measurement_count += 1;

        // Update clock state
        self.state.offset_s = self.x[0];
        self.state.drift_ppb = self.x[1] * 1e9;
        self.state.last_sync_time_s = pair.local_time_s;
        self.state.jitter_rms_s = self.compute_jitter_rms();
        self.state.quality = self.compute_quality();
    }

    /// Return the estimated offset in seconds (local - remote).
    pub fn estimate_offset(&self) -> f64 {
        self.x[0]
    }

    /// Return the estimated drift rate in parts-per-billion.
    pub fn estimate_drift(&self) -> f64 {
        self.x[1] * 1e9
    }

    /// Compensate a local timestamp to produce a corrected (remote-aligned) time.
    ///
    /// `t_corrected = t_local - offset - drift * (t_local - t_ref)`
    pub fn compensate_timestamp(&self, local_time: f64) -> f64 {
        let dt = local_time - self.t_ref;
        local_time - self.x[0] - self.x[1] * dt
    }

    /// Return a reference to the current clock state.
    pub fn clock_state(&self) -> &ClockState {
        &self.state
    }

    /// Tag an IQ sample with an absolute (corrected) timestamp.
    ///
    /// Given a sample index and sample rate, computes the local time and
    /// applies the synchronization correction.
    pub fn tag_sample(&self, sample_index: u64, sample_rate: f64) -> f64 {
        let local_time = sample_index as f64 / sample_rate;
        self.compensate_timestamp(local_time)
    }

    /// Returns `true` if the synchronizer is in the `Locked` state.
    pub fn is_locked(&self) -> bool {
        self.state.quality == SyncQuality::Locked
    }

    /// Estimate the remaining holdover time (seconds) before quality degrades
    /// to free-running. Returns 0.0 if not in holdover, or `f64::INFINITY` if locked.
    ///
    /// Holdover budget is based on drift uncertainty: the time until the
    /// extrapolated offset error exceeds `max_offset_s`.
    pub fn holdover_time_remaining(&self) -> f64 {
        match self.state.quality {
            SyncQuality::Locked => f64::INFINITY,
            SyncQuality::Freerunning => 0.0,
            SyncQuality::Holdover => {
                // Drift uncertainty grows as sqrt(P[1][1]) * dt
                let drift_uncertainty = self.p[3].abs().sqrt();
                if drift_uncertainty <= 0.0 {
                    return f64::INFINITY;
                }
                // Time until offset error exceeds max_offset_s
                let budget = self.config.max_offset_s / drift_uncertainty;
                let elapsed_since_sync = if self.t_ref > self.state.last_sync_time_s {
                    self.t_ref - self.state.last_sync_time_s
                } else {
                    0.0
                };
                (budget - elapsed_since_sync).max(0.0)
            }
        }
    }

    /// Notify the engine that time has advanced without a sync exchange.
    /// This may transition the quality from Locked to Holdover or Freerunning.
    pub fn advance_time(&mut self, current_local_time: f64) {
        if !self.initialized {
            return;
        }
        let elapsed = current_local_time - self.state.last_sync_time_s;
        self.state.quality = self.quality_for_elapsed(elapsed);
    }

    // --- Private helpers ---

    fn compute_jitter_rms(&self) -> f64 {
        let n = self.residual_idx.min(self.residuals.len());
        if n == 0 {
            return 0.0;
        }
        let sum_sq: f64 = self.residuals[..n].iter().map(|r| r * r).sum();
        (sum_sq / n as f64).sqrt()
    }

    fn compute_quality(&self) -> SyncQuality {
        if self.measurement_count < 2 {
            return SyncQuality::Freerunning;
        }
        // If jitter is reasonable relative to our offset uncertainty, we are locked
        let offset_std = self.p[0].abs().sqrt();
        if offset_std < self.config.max_offset_s * 0.1 {
            SyncQuality::Locked
        } else if offset_std < self.config.max_offset_s {
            SyncQuality::Holdover
        } else {
            SyncQuality::Freerunning
        }
    }

    fn quality_for_elapsed(&self, elapsed: f64) -> SyncQuality {
        let holdover_limit = 10.0 * self.config.sync_interval_s;
        let freerun_limit = 100.0 * self.config.sync_interval_s;
        if elapsed < holdover_limit {
            SyncQuality::Locked
        } else if elapsed < freerun_limit {
            SyncQuality::Holdover
        } else {
            SyncQuality::Freerunning
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> TimeSyncConfig {
        TimeSyncConfig::default()
    }

    #[test]
    fn test_new_starts_freerunning() {
        let sync = NetworkTimeSync::new(default_config());
        assert_eq!(sync.clock_state().quality, SyncQuality::Freerunning);
        assert!(!sync.is_locked());
    }

    #[test]
    fn test_first_measurement_initializes() {
        let mut sync = NetworkTimeSync::new(default_config());
        sync.process_timestamp_pair(TimestampPair {
            local_time_s: 100.0,
            remote_time_s: 100.0001,
        });
        // After first measurement, offset should be roughly -0.0001
        let offset = sync.estimate_offset();
        assert!((offset - (-0.0001)).abs() < 1e-6, "offset={}", offset);
    }

    #[test]
    fn test_zero_offset_converges() {
        let mut sync = NetworkTimeSync::new(default_config());
        for i in 0..20 {
            sync.process_timestamp_pair(TimestampPair {
                local_time_s: 100.0 + i as f64,
                remote_time_s: 100.0 + i as f64,
            });
        }
        assert!(sync.estimate_offset().abs() < 1e-6);
        assert!(sync.estimate_drift().abs() < 1.0); // < 1 ppb
    }

    #[test]
    fn test_constant_offset_converges() {
        let mut sync = NetworkTimeSync::new(default_config());
        let true_offset = 0.005; // 5 ms
        for i in 0..50 {
            let t = 1000.0 + i as f64;
            sync.process_timestamp_pair(TimestampPair {
                local_time_s: t,
                remote_time_s: t - true_offset,
            });
        }
        let est = sync.estimate_offset();
        assert!(
            (est - true_offset).abs() < 0.001,
            "expected ~{}, got {}",
            true_offset,
            est
        );
    }

    #[test]
    fn test_drift_estimation() {
        let mut sync = NetworkTimeSync::new(TimeSyncConfig {
            filter_bandwidth: 0.01,
            drift_rate_ppb_limit: 10000.0,
            ..default_config()
        });
        let drift_s_per_s = 100e-9; // 100 ppb
        for i in 0..200 {
            let t = 1000.0 + i as f64;
            let true_offset = drift_s_per_s * (t - 1000.0);
            sync.process_timestamp_pair(TimestampPair {
                local_time_s: t,
                remote_time_s: t - true_offset,
            });
        }
        let est_drift = sync.estimate_drift();
        assert!(
            (est_drift - 100.0).abs() < 50.0,
            "expected ~100 ppb, got {} ppb",
            est_drift
        );
    }

    #[test]
    fn test_compensate_timestamp_no_offset() {
        let mut sync = NetworkTimeSync::new(default_config());
        for i in 0..10 {
            let t = i as f64;
            sync.process_timestamp_pair(TimestampPair {
                local_time_s: t,
                remote_time_s: t,
            });
        }
        let corrected = sync.compensate_timestamp(10.0);
        assert!((corrected - 10.0).abs() < 0.001);
    }

    #[test]
    fn test_compensate_timestamp_with_offset() {
        let mut sync = NetworkTimeSync::new(default_config());
        let offset = 0.01; // 10 ms local ahead
        for i in 0..30 {
            let t = i as f64;
            sync.process_timestamp_pair(TimestampPair {
                local_time_s: t,
                remote_time_s: t - offset,
            });
        }
        let corrected = sync.compensate_timestamp(50.0);
        // corrected should be approximately 50.0 - 0.01 = 49.99
        assert!(
            (corrected - (50.0 - offset)).abs() < 0.005,
            "corrected={}",
            corrected
        );
    }

    #[test]
    fn test_tag_sample() {
        let mut sync = NetworkTimeSync::new(default_config());
        sync.process_timestamp_pair(TimestampPair {
            local_time_s: 0.0,
            remote_time_s: 0.0,
        });
        sync.process_timestamp_pair(TimestampPair {
            local_time_s: 1.0,
            remote_time_s: 1.0,
        });
        let sample_rate = 48000.0;
        let abs_time = sync.tag_sample(48000, sample_rate);
        // sample 48000 at 48kHz = 1.0 s local time
        assert!((abs_time - 1.0).abs() < 0.01, "abs_time={}", abs_time);
    }

    #[test]
    fn test_is_locked_after_convergence() {
        let mut sync = NetworkTimeSync::new(default_config());
        for i in 0..20 {
            sync.process_timestamp_pair(TimestampPair {
                local_time_s: i as f64,
                remote_time_s: i as f64,
            });
        }
        assert!(sync.is_locked());
    }

    #[test]
    fn test_holdover_time_locked() {
        let mut sync = NetworkTimeSync::new(default_config());
        for i in 0..20 {
            sync.process_timestamp_pair(TimestampPair {
                local_time_s: i as f64,
                remote_time_s: i as f64,
            });
        }
        assert!(sync.is_locked());
        assert_eq!(sync.holdover_time_remaining(), f64::INFINITY);
    }

    #[test]
    fn test_holdover_time_freerunning() {
        let sync = NetworkTimeSync::new(default_config());
        assert_eq!(sync.holdover_time_remaining(), 0.0);
    }

    #[test]
    fn test_advance_time_transitions_quality() {
        let mut sync = NetworkTimeSync::new(TimeSyncConfig {
            sync_interval_s: 1.0,
            ..default_config()
        });
        for i in 0..5 {
            sync.process_timestamp_pair(TimestampPair {
                local_time_s: i as f64,
                remote_time_s: i as f64,
            });
        }
        // Advance well beyond holdover limit (10 * sync_interval_s = 10 s)
        sync.advance_time(4.0 + 15.0);
        assert_eq!(sync.clock_state().quality, SyncQuality::Holdover);

        // Advance beyond freerunning limit (100 * sync_interval_s = 100 s)
        sync.advance_time(4.0 + 200.0);
        assert_eq!(sync.clock_state().quality, SyncQuality::Freerunning);
    }

    #[test]
    fn test_outlier_rejection() {
        let mut sync = NetworkTimeSync::new(TimeSyncConfig {
            max_offset_s: 0.01,
            ..default_config()
        });
        // Build up a stable estimate at zero offset
        for i in 0..10 {
            sync.process_timestamp_pair(TimestampPair {
                local_time_s: i as f64,
                remote_time_s: i as f64,
            });
        }
        let offset_before = sync.estimate_offset();

        // Inject a huge outlier
        sync.process_timestamp_pair(TimestampPair {
            local_time_s: 10.0,
            remote_time_s: 10.0 + 5.0, // 5 second jump >> max_offset_s
        });

        // Offset should not have changed significantly
        let offset_after = sync.estimate_offset();
        assert!(
            (offset_after - offset_before).abs() < 0.001,
            "outlier not rejected: before={}, after={}",
            offset_before,
            offset_after
        );
    }

    #[test]
    fn test_jitter_rms_nonzero_with_noise() {
        let mut sync = NetworkTimeSync::new(default_config());
        // Alternate slight offsets to create some jitter
        for i in 0..30 {
            let noise = if i % 2 == 0 { 0.0001 } else { -0.0001 };
            sync.process_timestamp_pair(TimestampPair {
                local_time_s: i as f64,
                remote_time_s: i as f64 + noise,
            });
        }
        assert!(sync.clock_state().jitter_rms_s > 0.0);
    }

    #[test]
    fn test_config_default() {
        let cfg = TimeSyncConfig::default();
        assert_eq!(cfg.filter_bandwidth, 0.1);
        assert_eq!(cfg.max_offset_s, 1.0);
        assert_eq!(cfg.drift_rate_ppb_limit, 1000.0);
        assert_eq!(cfg.sync_interval_s, 1.0);
    }

    #[test]
    fn test_clock_state_fields() {
        let mut sync = NetworkTimeSync::new(default_config());
        sync.process_timestamp_pair(TimestampPair {
            local_time_s: 42.0,
            remote_time_s: 42.001,
        });
        let state = sync.clock_state();
        assert!((state.last_sync_time_s - 42.0).abs() < 1e-9);
        assert!(state.offset_s < 0.0); // local is behind remote
    }

    #[test]
    fn test_sync_quality_enum_equality() {
        assert_eq!(SyncQuality::Locked, SyncQuality::Locked);
        assert_ne!(SyncQuality::Locked, SyncQuality::Holdover);
        assert_ne!(SyncQuality::Holdover, SyncQuality::Freerunning);
    }

    #[test]
    fn test_negative_dt_ignored() {
        let mut sync = NetworkTimeSync::new(default_config());
        sync.process_timestamp_pair(TimestampPair {
            local_time_s: 10.0,
            remote_time_s: 10.0,
        });
        let offset_before = sync.estimate_offset();

        // Send an older timestamp (should be ignored)
        sync.process_timestamp_pair(TimestampPair {
            local_time_s: 5.0,
            remote_time_s: 5.0,
        });
        assert_eq!(sync.estimate_offset(), offset_before);
    }
}
