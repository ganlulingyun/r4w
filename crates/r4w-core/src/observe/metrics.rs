//! # Metrics Collection
//!
//! Provides Prometheus-compatible metrics for R4W SDR operations:
//!
//! - **Counters**: Total samples processed, packets, errors
//! - **Gauges**: Buffer levels, signal strength, frequency
//! - **Histograms**: Processing latency, packet sizes
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::observe::Metrics;
//!
//! let metrics = Metrics::new();
//!
//! // Record samples processed
//! metrics.rx_samples.inc_by(1024);
//! metrics.tx_samples.inc_by(512);
//!
//! // Update buffer levels
//! metrics.rx_buffer_level.set(4096);
//!
//! // Record latency
//! metrics.processing_latency_us.observe(150.0);
//!
//! // Get snapshot for export
//! let snapshot = metrics.snapshot();
//! println!("Total RX: {}", snapshot.rx_samples);
//! ```

use std::sync::atomic::{AtomicI64, AtomicU64, Ordering};
use std::sync::RwLock;

/// A simple atomic counter.
#[derive(Debug, Default)]
pub struct Counter {
    value: AtomicU64,
}

impl Counter {
    /// Create a new counter.
    pub fn new() -> Self {
        Self {
            value: AtomicU64::new(0),
        }
    }

    /// Increment by 1.
    #[inline]
    pub fn inc(&self) {
        self.value.fetch_add(1, Ordering::Relaxed);
    }

    /// Increment by a specific amount.
    #[inline]
    pub fn inc_by(&self, n: u64) {
        self.value.fetch_add(n, Ordering::Relaxed);
    }

    /// Get the current value.
    #[inline]
    pub fn get(&self) -> u64 {
        self.value.load(Ordering::Relaxed)
    }

    /// Reset to zero.
    #[inline]
    pub fn reset(&self) {
        self.value.store(0, Ordering::Relaxed);
    }
}

/// A simple atomic gauge (can go up or down).
#[derive(Debug, Default)]
pub struct Gauge {
    value: AtomicI64,
}

impl Gauge {
    /// Create a new gauge.
    pub fn new() -> Self {
        Self {
            value: AtomicI64::new(0),
        }
    }

    /// Set the value.
    #[inline]
    pub fn set(&self, v: i64) {
        self.value.store(v, Ordering::Relaxed);
    }

    /// Increment by 1.
    #[inline]
    pub fn inc(&self) {
        self.value.fetch_add(1, Ordering::Relaxed);
    }

    /// Decrement by 1.
    #[inline]
    pub fn dec(&self) {
        self.value.fetch_sub(1, Ordering::Relaxed);
    }

    /// Add a value.
    #[inline]
    pub fn add(&self, v: i64) {
        self.value.fetch_add(v, Ordering::Relaxed);
    }

    /// Get the current value.
    #[inline]
    pub fn get(&self) -> i64 {
        self.value.load(Ordering::Relaxed)
    }
}

/// A simple histogram with fixed buckets.
#[derive(Debug)]
pub struct Histogram {
    /// Bucket boundaries
    boundaries: Vec<f64>,
    /// Bucket counts (len = boundaries.len() + 1 for overflow bucket)
    buckets: Vec<AtomicU64>,
    /// Sum of all observed values
    sum: AtomicU64,
    /// Count of observations
    count: AtomicU64,
}

impl Default for Histogram {
    fn default() -> Self {
        Self::new(vec![10.0, 50.0, 100.0, 250.0, 500.0, 1000.0, 2500.0, 5000.0])
    }
}

impl Histogram {
    /// Create a new histogram with custom bucket boundaries.
    pub fn new(boundaries: Vec<f64>) -> Self {
        let num_buckets = boundaries.len() + 1;
        Self {
            boundaries,
            buckets: (0..num_buckets).map(|_| AtomicU64::new(0)).collect(),
            sum: AtomicU64::new(0),
            count: AtomicU64::new(0),
        }
    }

    /// Create a histogram for microsecond latencies.
    pub fn latency_us() -> Self {
        Self::new(vec![
            1.0, 5.0, 10.0, 25.0, 50.0, 100.0, 250.0, 500.0, 1000.0, 2500.0, 5000.0, 10000.0,
        ])
    }

    /// Observe a value.
    pub fn observe(&self, value: f64) {
        let bucket_idx = self
            .boundaries
            .iter()
            .position(|&b| value < b)
            .unwrap_or(self.boundaries.len());

        self.buckets[bucket_idx].fetch_add(1, Ordering::Relaxed);
        self.sum
            .fetch_add((value * 1000.0) as u64, Ordering::Relaxed); // Store as micro
        self.count.fetch_add(1, Ordering::Relaxed);
    }

    /// Get the count of observations.
    #[inline]
    pub fn count(&self) -> u64 {
        self.count.load(Ordering::Relaxed)
    }

    /// Get the sum of all observations.
    #[inline]
    pub fn sum(&self) -> f64 {
        self.sum.load(Ordering::Relaxed) as f64 / 1000.0
    }

    /// Get the bucket counts.
    pub fn bucket_counts(&self) -> Vec<u64> {
        self.buckets
            .iter()
            .map(|b| b.load(Ordering::Relaxed))
            .collect()
    }

    /// Get the bucket boundaries.
    pub fn boundaries(&self) -> &[f64] {
        &self.boundaries
    }
}

/// R4W metrics collection.
///
/// Provides all metrics for SDR operations.
#[derive(Debug, Default)]
pub struct Metrics {
    // Sample counters
    /// Total RX samples processed
    pub rx_samples: Counter,
    /// Total TX samples processed
    pub tx_samples: Counter,

    // Buffer gauges
    /// Current RX buffer level (samples)
    pub rx_buffer_level: Gauge,
    /// Current TX buffer level (samples)
    pub tx_buffer_level: Gauge,

    // Error counters
    /// RX overflow events
    pub rx_overflows: Counter,
    /// TX underflow events
    pub tx_underflows: Counter,
    /// Packet CRC errors
    pub crc_errors: Counter,
    /// Device errors
    pub device_errors: Counter,

    // Packet counters
    /// Packets received
    pub packets_rx: Counter,
    /// Packets transmitted
    pub packets_tx: Counter,
    /// Packets decoded successfully
    pub packets_decoded: Counter,
    /// Packets failed to decode
    pub packets_failed: Counter,

    // Signal metrics
    /// Current RSSI (dBm * 10)
    pub rssi_dbm_x10: Gauge,
    /// Current SNR estimate (dB * 10)
    pub snr_db_x10: Gauge,
    /// Current frequency offset estimate (Hz)
    pub freq_offset_hz: Gauge,

    // Timing histograms
    /// Processing latency in microseconds
    pub processing_latency_us: Histogram,
    /// Time between packets in milliseconds
    pub packet_interval_ms: Histogram,

    // Waveform info
    /// Currently active waveform name
    pub active_waveform: RwLock<String>,
}

impl Metrics {
    /// Create a new metrics instance.
    pub fn new() -> Self {
        Self {
            processing_latency_us: Histogram::latency_us(),
            packet_interval_ms: Histogram::new(vec![
                1.0, 5.0, 10.0, 25.0, 50.0, 100.0, 250.0, 500.0, 1000.0,
            ]),
            ..Default::default()
        }
    }

    /// Get a snapshot of all metrics.
    pub fn snapshot(&self) -> MetricsSnapshot {
        MetricsSnapshot {
            rx_samples: self.rx_samples.get(),
            tx_samples: self.tx_samples.get(),
            rx_buffer_level: self.rx_buffer_level.get(),
            tx_buffer_level: self.tx_buffer_level.get(),
            rx_overflows: self.rx_overflows.get(),
            tx_underflows: self.tx_underflows.get(),
            crc_errors: self.crc_errors.get(),
            device_errors: self.device_errors.get(),
            packets_rx: self.packets_rx.get(),
            packets_tx: self.packets_tx.get(),
            packets_decoded: self.packets_decoded.get(),
            packets_failed: self.packets_failed.get(),
            rssi_dbm: self.rssi_dbm_x10.get() as f64 / 10.0,
            snr_db: self.snr_db_x10.get() as f64 / 10.0,
            freq_offset_hz: self.freq_offset_hz.get(),
            processing_latency_count: self.processing_latency_us.count(),
            processing_latency_sum_us: self.processing_latency_us.sum(),
            active_waveform: self.active_waveform.read().unwrap().clone(),
        }
    }

    /// Reset all metrics to zero.
    pub fn reset(&self) {
        self.rx_samples.reset();
        self.tx_samples.reset();
        self.rx_overflows.reset();
        self.tx_underflows.reset();
        self.crc_errors.reset();
        self.device_errors.reset();
        self.packets_rx.reset();
        self.packets_tx.reset();
        self.packets_decoded.reset();
        self.packets_failed.reset();
    }

    /// Set the active waveform name.
    pub fn set_waveform(&self, name: &str) {
        if let Ok(mut w) = self.active_waveform.write() {
            *w = name.to_string();
        }
    }

    /// Record RSSI in dBm.
    pub fn record_rssi(&self, rssi_dbm: f64) {
        self.rssi_dbm_x10.set((rssi_dbm * 10.0) as i64);
    }

    /// Record SNR in dB.
    pub fn record_snr(&self, snr_db: f64) {
        self.snr_db_x10.set((snr_db * 10.0) as i64);
    }

    /// Export metrics in Prometheus text format.
    pub fn to_prometheus(&self) -> String {
        let s = self.snapshot();
        let mut output = String::new();

        // Sample counters
        output.push_str("# HELP r4w_rx_samples_total Total RX samples processed\n");
        output.push_str("# TYPE r4w_rx_samples_total counter\n");
        output.push_str(&format!("r4w_rx_samples_total {}\n", s.rx_samples));

        output.push_str("# HELP r4w_tx_samples_total Total TX samples processed\n");
        output.push_str("# TYPE r4w_tx_samples_total counter\n");
        output.push_str(&format!("r4w_tx_samples_total {}\n", s.tx_samples));

        // Buffer gauges
        output.push_str("# HELP r4w_rx_buffer_level Current RX buffer level\n");
        output.push_str("# TYPE r4w_rx_buffer_level gauge\n");
        output.push_str(&format!("r4w_rx_buffer_level {}\n", s.rx_buffer_level));

        output.push_str("# HELP r4w_tx_buffer_level Current TX buffer level\n");
        output.push_str("# TYPE r4w_tx_buffer_level gauge\n");
        output.push_str(&format!("r4w_tx_buffer_level {}\n", s.tx_buffer_level));

        // Error counters
        output.push_str("# HELP r4w_rx_overflows_total RX overflow events\n");
        output.push_str("# TYPE r4w_rx_overflows_total counter\n");
        output.push_str(&format!("r4w_rx_overflows_total {}\n", s.rx_overflows));

        output.push_str("# HELP r4w_tx_underflows_total TX underflow events\n");
        output.push_str("# TYPE r4w_tx_underflows_total counter\n");
        output.push_str(&format!("r4w_tx_underflows_total {}\n", s.tx_underflows));

        // Signal metrics
        output.push_str("# HELP r4w_rssi_dbm Current RSSI in dBm\n");
        output.push_str("# TYPE r4w_rssi_dbm gauge\n");
        output.push_str(&format!("r4w_rssi_dbm {}\n", s.rssi_dbm));

        output.push_str("# HELP r4w_snr_db Current SNR in dB\n");
        output.push_str("# TYPE r4w_snr_db gauge\n");
        output.push_str(&format!("r4w_snr_db {}\n", s.snr_db));

        // Packet counters
        output.push_str("# HELP r4w_packets_decoded_total Successfully decoded packets\n");
        output.push_str("# TYPE r4w_packets_decoded_total counter\n");
        output.push_str(&format!("r4w_packets_decoded_total {}\n", s.packets_decoded));

        output
    }
}

/// A snapshot of metrics at a point in time.
#[derive(Debug, Clone)]
pub struct MetricsSnapshot {
    pub rx_samples: u64,
    pub tx_samples: u64,
    pub rx_buffer_level: i64,
    pub tx_buffer_level: i64,
    pub rx_overflows: u64,
    pub tx_underflows: u64,
    pub crc_errors: u64,
    pub device_errors: u64,
    pub packets_rx: u64,
    pub packets_tx: u64,
    pub packets_decoded: u64,
    pub packets_failed: u64,
    pub rssi_dbm: f64,
    pub snr_db: f64,
    pub freq_offset_hz: i64,
    pub processing_latency_count: u64,
    pub processing_latency_sum_us: f64,
    pub active_waveform: String,
}

impl MetricsSnapshot {
    /// Calculate average processing latency in microseconds.
    pub fn avg_latency_us(&self) -> f64 {
        if self.processing_latency_count == 0 {
            0.0
        } else {
            self.processing_latency_sum_us / self.processing_latency_count as f64
        }
    }

    /// Calculate decode success rate.
    pub fn decode_success_rate(&self) -> f64 {
        let total = self.packets_decoded + self.packets_failed;
        if total == 0 {
            1.0
        } else {
            self.packets_decoded as f64 / total as f64
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_counter() {
        let counter = Counter::new();
        assert_eq!(counter.get(), 0);

        counter.inc();
        assert_eq!(counter.get(), 1);

        counter.inc_by(99);
        assert_eq!(counter.get(), 100);

        counter.reset();
        assert_eq!(counter.get(), 0);
    }

    #[test]
    fn test_gauge() {
        let gauge = Gauge::new();
        assert_eq!(gauge.get(), 0);

        gauge.set(100);
        assert_eq!(gauge.get(), 100);

        gauge.inc();
        assert_eq!(gauge.get(), 101);

        gauge.dec();
        assert_eq!(gauge.get(), 100);

        gauge.add(-50);
        assert_eq!(gauge.get(), 50);
    }

    #[test]
    fn test_histogram() {
        let hist = Histogram::new(vec![10.0, 100.0, 1000.0]);

        hist.observe(5.0); // bucket 0
        hist.observe(50.0); // bucket 1
        hist.observe(500.0); // bucket 2
        hist.observe(5000.0); // bucket 3 (overflow)

        assert_eq!(hist.count(), 4);
        assert!((hist.sum() - 5555.0).abs() < 0.01);

        let counts = hist.bucket_counts();
        assert_eq!(counts[0], 1);
        assert_eq!(counts[1], 1);
        assert_eq!(counts[2], 1);
        assert_eq!(counts[3], 1);
    }

    #[test]
    fn test_metrics() {
        let metrics = Metrics::new();

        metrics.rx_samples.inc_by(1024);
        metrics.tx_samples.inc_by(512);
        metrics.rx_buffer_level.set(4096);
        metrics.record_rssi(-80.5);
        metrics.record_snr(12.3);
        metrics.set_waveform("lora");
        metrics.packets_decoded.inc_by(10);
        metrics.packets_failed.inc_by(2);

        let snapshot = metrics.snapshot();
        assert_eq!(snapshot.rx_samples, 1024);
        assert_eq!(snapshot.tx_samples, 512);
        assert_eq!(snapshot.rx_buffer_level, 4096);
        assert!((snapshot.rssi_dbm - (-80.5)).abs() < 0.2);
        assert!((snapshot.snr_db - 12.3).abs() < 0.2);
        assert_eq!(snapshot.active_waveform, "lora");

        let rate = snapshot.decode_success_rate();
        assert!((rate - 10.0 / 12.0).abs() < 0.01);
    }

    #[test]
    fn test_prometheus_export() {
        let metrics = Metrics::new();
        metrics.rx_samples.inc_by(1000);

        let output = metrics.to_prometheus();
        assert!(output.contains("r4w_rx_samples_total 1000"));
        assert!(output.contains("# TYPE r4w_rx_samples_total counter"));
    }
}
