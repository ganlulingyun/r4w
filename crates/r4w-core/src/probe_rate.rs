//! # Probe Rate
//!
//! Measures sample throughput rate in a streaming pipeline.
//! Reports instantaneous and smoothed sample rates, useful for
//! monitoring pipeline performance and detecting bottlenecks.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::probe_rate::ProbeRate;
//!
//! let mut probe = ProbeRate::new(1.0); // 1-second update interval
//! probe.record_samples(48000);
//! probe.record_samples(48000);
//! // After enough time has passed, rate() returns the measured rate.
//! ```

/// Sample rate measurement probe.
#[derive(Debug, Clone)]
pub struct ProbeRate {
    /// Update interval in seconds.
    update_interval: f64,
    /// Smoothing factor for EWMA (0..1, higher = more smoothing).
    alpha: f64,
    /// Total samples in current window.
    window_samples: u64,
    /// Timestamp (seconds) when current window started.
    window_start: f64,
    /// Current time (seconds), advanced by caller.
    current_time: f64,
    /// Smoothed rate estimate (samples/second).
    smoothed_rate: f64,
    /// Last measured instantaneous rate.
    instant_rate: f64,
    /// Total samples ever recorded.
    total_samples: u64,
    /// Number of rate updates.
    update_count: u64,
    /// Has been initialized.
    initialized: bool,
}

impl ProbeRate {
    /// Create a new probe with given update interval (seconds).
    pub fn new(update_interval: f64) -> Self {
        Self {
            update_interval: update_interval.max(0.001),
            alpha: 0.1,
            window_samples: 0,
            window_start: 0.0,
            current_time: 0.0,
            smoothed_rate: 0.0,
            instant_rate: 0.0,
            total_samples: 0,
            update_count: 0,
            initialized: false,
        }
    }

    /// Create with custom smoothing alpha.
    pub fn with_alpha(update_interval: f64, alpha: f64) -> Self {
        let mut p = Self::new(update_interval);
        p.alpha = alpha.clamp(0.001, 1.0);
        p
    }

    /// Record that N samples have been processed.
    pub fn record_samples(&mut self, count: u64) {
        self.total_samples += count;
        self.window_samples += count;
    }

    /// Advance the clock and potentially update the rate estimate.
    ///
    /// Returns `Some(rate)` if a new measurement was taken.
    pub fn tick(&mut self, elapsed_seconds: f64) -> Option<f64> {
        self.current_time += elapsed_seconds;

        if !self.initialized {
            self.window_start = self.current_time;
            self.initialized = true;
            return None;
        }

        let window_dur = self.current_time - self.window_start;
        if window_dur >= self.update_interval {
            self.instant_rate = self.window_samples as f64 / window_dur;
            if self.update_count == 0 {
                self.smoothed_rate = self.instant_rate;
            } else {
                self.smoothed_rate =
                    self.alpha * self.instant_rate + (1.0 - self.alpha) * self.smoothed_rate;
            }
            self.update_count += 1;
            self.window_samples = 0;
            self.window_start = self.current_time;
            Some(self.smoothed_rate)
        } else {
            None
        }
    }

    /// Combined record + tick for known sample rate pipelines.
    ///
    /// If you know the nominal sample rate, use this to auto-advance time.
    pub fn record_with_rate(&mut self, count: u64, nominal_rate: f64) -> Option<f64> {
        self.record_samples(count);
        let elapsed = count as f64 / nominal_rate;
        self.tick(elapsed)
    }

    /// Get smoothed rate estimate (samples/second).
    pub fn rate(&self) -> f64 {
        self.smoothed_rate
    }

    /// Get last instantaneous rate.
    pub fn instant_rate(&self) -> f64 {
        self.instant_rate
    }

    /// Get total samples recorded.
    pub fn total_samples(&self) -> u64 {
        self.total_samples
    }

    /// Get number of rate updates.
    pub fn update_count(&self) -> u64 {
        self.update_count
    }

    /// Get current time.
    pub fn current_time(&self) -> f64 {
        self.current_time
    }

    /// Get overall average rate since start.
    pub fn overall_rate(&self) -> f64 {
        if self.current_time > 0.0 {
            self.total_samples as f64 / self.current_time
        } else {
            0.0
        }
    }

    /// Reset all state.
    pub fn reset(&mut self) {
        self.window_samples = 0;
        self.window_start = 0.0;
        self.current_time = 0.0;
        self.smoothed_rate = 0.0;
        self.instant_rate = 0.0;
        self.total_samples = 0;
        self.update_count = 0;
        self.initialized = false;
    }
}

/// Throughput monitor for multiple named channels.
#[derive(Debug, Clone)]
pub struct ThroughputMonitor {
    channels: Vec<(String, ProbeRate)>,
}

impl ThroughputMonitor {
    /// Create a new throughput monitor.
    pub fn new() -> Self {
        Self {
            channels: Vec::new(),
        }
    }

    /// Add a named channel to monitor.
    pub fn add_channel(&mut self, name: &str, update_interval: f64) {
        self.channels
            .push((name.to_string(), ProbeRate::new(update_interval)));
    }

    /// Record samples for a named channel.
    pub fn record(&mut self, name: &str, count: u64) {
        if let Some((_, probe)) = self.channels.iter_mut().find(|(n, _)| n == name) {
            probe.record_samples(count);
        }
    }

    /// Tick all channels with elapsed time.
    pub fn tick_all(&mut self, elapsed_seconds: f64) {
        for (_, probe) in &mut self.channels {
            probe.tick(elapsed_seconds);
        }
    }

    /// Get rates for all channels.
    pub fn rates(&self) -> Vec<(&str, f64)> {
        self.channels
            .iter()
            .map(|(name, probe)| (name.as_str(), probe.rate()))
            .collect()
    }

    /// Get number of channels.
    pub fn num_channels(&self) -> usize {
        self.channels.len()
    }

    /// Reset all channels.
    pub fn reset(&mut self) {
        for (_, probe) in &mut self.channels {
            probe.reset();
        }
    }
}

impl Default for ThroughputMonitor {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_rate() {
        let mut probe = ProbeRate::new(1.0);
        // First tick initializes window_start.
        assert!(probe.tick(0.0).is_none());
        probe.record_samples(48000);
        // Tick after 1s — should trigger update.
        let rate = probe.tick(1.0);
        assert!(rate.is_some());
        let r = rate.unwrap();
        assert!((r - 48000.0).abs() < 1000.0); // ~48k samples in 1s
    }

    #[test]
    fn test_smoothed_rate() {
        let mut probe = ProbeRate::with_alpha(0.1, 0.5);
        // First window: 10000 samples in 0.1s = 100k/s.
        probe.record_samples(10000);
        probe.tick(0.05);
        probe.tick(0.05);
        // Second window: 20000 samples in 0.1s = 200k/s.
        probe.record_samples(20000);
        let rate = probe.tick(0.1);
        assert!(rate.is_some());
        // Smoothed should be between 100k and 200k.
        let r = rate.unwrap();
        assert!(r > 50000.0 && r < 250000.0);
    }

    #[test]
    fn test_record_with_rate() {
        let mut probe = ProbeRate::new(0.01);
        // Record 48000 samples at 48000 Hz = 1 second of data.
        for _ in 0..100 {
            probe.record_with_rate(480, 48000.0);
        }
        assert!(probe.rate() > 0.0);
        assert_eq!(probe.total_samples(), 48000);
    }

    #[test]
    fn test_overall_rate() {
        let mut probe = ProbeRate::new(1.0);
        probe.record_samples(100000);
        probe.tick(0.0); // initialize
        probe.tick(2.0);
        assert!((probe.overall_rate() - 50000.0).abs() < 1000.0);
    }

    #[test]
    fn test_reset() {
        let mut probe = ProbeRate::new(0.1);
        probe.record_samples(1000);
        probe.tick(0.0);
        probe.tick(0.2);
        assert!(probe.total_samples() > 0);
        probe.reset();
        assert_eq!(probe.total_samples(), 0);
        assert_eq!(probe.rate(), 0.0);
        assert_eq!(probe.update_count(), 0);
    }

    #[test]
    fn test_throughput_monitor() {
        let mut monitor = ThroughputMonitor::new();
        monitor.add_channel("tx", 0.1);
        monitor.add_channel("rx", 0.1);
        assert_eq!(monitor.num_channels(), 2);

        monitor.record("tx", 5000);
        monitor.record("rx", 3000);
        monitor.tick_all(0.0); // init
        monitor.tick_all(0.1);

        let rates = monitor.rates();
        assert_eq!(rates.len(), 2);
    }

    #[test]
    fn test_throughput_monitor_reset() {
        let mut monitor = ThroughputMonitor::new();
        monitor.add_channel("ch1", 0.1);
        monitor.record("ch1", 1000);
        monitor.tick_all(0.0);
        monitor.tick_all(0.2);
        monitor.reset();
        let rates = monitor.rates();
        assert_eq!(rates[0].1, 0.0);
    }

    #[test]
    fn test_no_update_before_interval() {
        let mut probe = ProbeRate::new(1.0);
        probe.record_samples(1000);
        probe.tick(0.0); // init
        // Tick at 0.5s — not enough for 1s interval.
        assert!(probe.tick(0.5).is_none());
    }

    #[test]
    fn test_multiple_updates() {
        let mut probe = ProbeRate::new(0.1);
        probe.tick(0.0); // init
        let mut updates = 0;
        for _ in 0..20 {
            probe.record_samples(1000);
            if probe.tick(0.1).is_some() {
                updates += 1;
            }
        }
        assert!(updates >= 15); // Should get many updates.
    }

    #[test]
    fn test_zero_samples() {
        let mut probe = ProbeRate::new(0.1);
        // Don't record any samples.
        probe.tick(0.0); // init
        let rate = probe.tick(0.2);
        assert!(rate.is_some());
        assert_eq!(rate.unwrap(), 0.0);
    }
}
