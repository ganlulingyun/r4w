//! Throttle — Rate-limiting block for simulation pipelines
//!
//! Limits the throughput of a data stream to a specified sample rate.
//! Essential for simulating real-time behavior in non-real-time
//! processing pipelines. Also provides rate measurement (throughput monitor).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::throttle::{Throttle, ThroughputMonitor};
//!
//! let mut throttle = Throttle::new(48000.0);
//! assert_eq!(throttle.sample_rate(), 48000.0);
//! // Compute how long to sleep for a block of N samples
//! let sleep_us = throttle.compute_delay_us(480);
//! assert!((sleep_us - 10000.0).abs() < 100.0); // 480/48000 = 10ms
//! ```

use std::time::{Duration, Instant};

/// Rate-limiting throttle for simulation pipelines.
///
/// Tracks how many samples have been processed and computes the
/// appropriate delay to maintain the target sample rate.
#[derive(Debug, Clone)]
pub struct Throttle {
    /// Target sample rate (samples/second).
    sample_rate: f64,
    /// Total samples processed.
    total_samples: u64,
    /// Start time.
    start_time: Option<Instant>,
    /// Whether throttling is enabled.
    enabled: bool,
}

impl Throttle {
    /// Create a throttle at the given sample rate.
    pub fn new(sample_rate: f64) -> Self {
        Self {
            sample_rate: sample_rate.max(1.0),
            total_samples: 0,
            start_time: None,
            enabled: true,
        }
    }

    /// Compute the delay in microseconds for processing `n_samples`.
    ///
    /// Returns how long the caller should sleep to maintain rate.
    pub fn compute_delay_us(&mut self, n_samples: usize) -> f64 {
        if !self.enabled {
            self.total_samples += n_samples as u64;
            return 0.0;
        }

        let now = Instant::now();
        if self.start_time.is_none() {
            self.start_time = Some(now);
        }

        self.total_samples += n_samples as u64;
        let target_elapsed = self.total_samples as f64 / self.sample_rate;
        let actual_elapsed = now.duration_since(self.start_time.unwrap()).as_secs_f64();
        let delay = target_elapsed - actual_elapsed;

        if delay > 0.0 {
            delay * 1_000_000.0
        } else {
            0.0
        }
    }

    /// Compute delay as a Duration.
    pub fn compute_delay(&mut self, n_samples: usize) -> Duration {
        let us = self.compute_delay_us(n_samples);
        Duration::from_micros(us as u64)
    }

    /// Mark samples as processed without computing delay.
    pub fn advance(&mut self, n_samples: usize) {
        self.total_samples += n_samples as u64;
    }

    /// Get the target sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Set the target sample rate.
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate.max(1.0);
    }

    /// Enable or disable throttling.
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Get total samples processed.
    pub fn total_samples(&self) -> u64 {
        self.total_samples
    }

    /// Get elapsed time since start.
    pub fn elapsed(&self) -> Duration {
        self.start_time.map_or(Duration::ZERO, |t| t.elapsed())
    }

    /// Get actual throughput (samples/second).
    pub fn actual_rate(&self) -> f64 {
        let elapsed = self.elapsed().as_secs_f64();
        if elapsed > 0.0 {
            self.total_samples as f64 / elapsed
        } else {
            0.0
        }
    }

    /// Reset the throttle state.
    pub fn reset(&mut self) {
        self.total_samples = 0;
        self.start_time = None;
    }
}

/// Throughput monitor — measures actual data rate.
#[derive(Debug, Clone)]
pub struct ThroughputMonitor {
    /// Window size for rate measurement.
    window_samples: u64,
    /// Samples in current window.
    current_samples: u64,
    /// Time at start of current window.
    window_start: Option<Instant>,
    /// Last measured rate.
    last_rate: f64,
    /// Total samples ever processed.
    total_samples: u64,
    /// Total time.
    total_start: Option<Instant>,
}

impl ThroughputMonitor {
    /// Create a throughput monitor.
    ///
    /// `window_samples`: Number of samples per measurement window.
    pub fn new(window_samples: u64) -> Self {
        Self {
            window_samples: window_samples.max(1),
            current_samples: 0,
            window_start: None,
            last_rate: 0.0,
            total_samples: 0,
            total_start: None,
        }
    }

    /// Record that `n` samples were processed. Returns rate if window completed.
    pub fn record(&mut self, n: usize) -> Option<f64> {
        let now = Instant::now();
        if self.window_start.is_none() {
            self.window_start = Some(now);
            self.total_start = Some(now);
        }

        self.current_samples += n as u64;
        self.total_samples += n as u64;

        if self.current_samples >= self.window_samples {
            let elapsed = now.duration_since(self.window_start.unwrap()).as_secs_f64();
            let rate = if elapsed > 0.0 {
                self.current_samples as f64 / elapsed
            } else {
                0.0
            };
            self.last_rate = rate;
            self.current_samples = 0;
            self.window_start = Some(now);
            Some(rate)
        } else {
            None
        }
    }

    /// Get the last measured rate (samples/second).
    pub fn last_rate(&self) -> f64 {
        self.last_rate
    }

    /// Get the average rate since start.
    pub fn average_rate(&self) -> f64 {
        if let Some(start) = self.total_start {
            let elapsed = start.elapsed().as_secs_f64();
            if elapsed > 0.0 {
                return self.total_samples as f64 / elapsed;
            }
        }
        0.0
    }

    /// Get total samples processed.
    pub fn total_samples(&self) -> u64 {
        self.total_samples
    }

    /// Reset the monitor.
    pub fn reset(&mut self) {
        self.current_samples = 0;
        self.window_start = None;
        self.last_rate = 0.0;
        self.total_samples = 0;
        self.total_start = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_throttle_creation() {
        let throttle = Throttle::new(48000.0);
        assert_eq!(throttle.sample_rate(), 48000.0);
        assert_eq!(throttle.total_samples(), 0);
    }

    #[test]
    fn test_compute_delay_first_call() {
        let mut throttle = Throttle::new(48000.0);
        let delay = throttle.compute_delay_us(480);
        // 480 samples at 48000 Hz = 10ms = 10000 us
        // First call starts the timer, so delay should be close to target
        assert!(delay >= 0.0);
        assert!(delay < 15000.0); // Should be around 10ms
    }

    #[test]
    fn test_throttle_disabled() {
        let mut throttle = Throttle::new(48000.0);
        throttle.set_enabled(false);
        let delay = throttle.compute_delay_us(480);
        assert_eq!(delay, 0.0);
        assert_eq!(throttle.total_samples(), 480);
    }

    #[test]
    fn test_throttle_advance() {
        let mut throttle = Throttle::new(48000.0);
        throttle.advance(1000);
        assert_eq!(throttle.total_samples(), 1000);
    }

    #[test]
    fn test_throttle_reset() {
        let mut throttle = Throttle::new(48000.0);
        throttle.advance(1000);
        throttle.reset();
        assert_eq!(throttle.total_samples(), 0);
    }

    #[test]
    fn test_set_sample_rate() {
        let mut throttle = Throttle::new(48000.0);
        throttle.set_sample_rate(96000.0);
        assert_eq!(throttle.sample_rate(), 96000.0);
    }

    #[test]
    fn test_min_sample_rate() {
        let throttle = Throttle::new(-100.0);
        assert_eq!(throttle.sample_rate(), 1.0);
    }

    #[test]
    fn test_throughput_monitor_creation() {
        let monitor = ThroughputMonitor::new(1000);
        assert_eq!(monitor.total_samples(), 0);
        assert_eq!(monitor.last_rate(), 0.0);
    }

    #[test]
    fn test_throughput_monitor_window() {
        let mut monitor = ThroughputMonitor::new(100);
        // First 50 samples: no rate yet
        let rate = monitor.record(50);
        assert!(rate.is_none());
        // Next 60 samples: window of 100 reached
        let rate = monitor.record(60);
        assert!(rate.is_some());
        assert!(monitor.last_rate() > 0.0);
    }

    #[test]
    fn test_throughput_monitor_total() {
        let mut monitor = ThroughputMonitor::new(1000);
        monitor.record(100);
        monitor.record(200);
        monitor.record(300);
        assert_eq!(monitor.total_samples(), 600);
    }

    #[test]
    fn test_throughput_monitor_reset() {
        let mut monitor = ThroughputMonitor::new(100);
        monitor.record(500);
        monitor.reset();
        assert_eq!(monitor.total_samples(), 0);
        assert_eq!(monitor.last_rate(), 0.0);
    }
}
