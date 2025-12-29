//! Benchmark Metrics Collection
//!
//! Rolling statistics for throughput, latency, and quality metrics.

use super::runner::ProcessResult;
use std::collections::VecDeque;
use std::time::{Duration, Instant};

/// Maximum number of latency samples to keep for percentile calculation
const MAX_LATENCY_SAMPLES: usize = 1000;

/// Comprehensive benchmark metrics
#[derive(Debug, Clone)]
pub struct BenchmarkMetrics {
    // Timing
    start_time: Option<Instant>,
    last_update: Option<Instant>,

    // Throughput
    pub samples_received: u64,
    pub samples_processed: u64,
    pub packets_received: u64,
    pub bytes_received: u64,

    // Latency (rolling window)
    latency_samples: VecDeque<Duration>,
    pub total_processing_time: Duration,

    // Quality
    pub bits_demodulated: u64,
    pub symbols_detected: u64,

    // Errors
    pub receive_errors: u64,
    pub processing_errors: u64,
}

impl Default for BenchmarkMetrics {
    fn default() -> Self {
        Self::new()
    }
}

impl BenchmarkMetrics {
    /// Create new metrics tracker
    pub fn new() -> Self {
        Self {
            start_time: None,
            last_update: None,
            samples_received: 0,
            samples_processed: 0,
            packets_received: 0,
            bytes_received: 0,
            latency_samples: VecDeque::with_capacity(MAX_LATENCY_SAMPLES),
            total_processing_time: Duration::ZERO,
            bits_demodulated: 0,
            symbols_detected: 0,
            receive_errors: 0,
            processing_errors: 0,
        }
    }

    /// Start timing (call at benchmark start)
    pub fn start(&mut self) {
        self.start_time = Some(Instant::now());
        self.last_update = Some(Instant::now());
    }

    /// Update metrics with a process result
    pub fn update(&mut self, result: &ProcessResult) {
        if self.start_time.is_none() {
            self.start();
        }
        self.last_update = Some(Instant::now());

        // Throughput
        self.samples_processed += result.samples_processed as u64;

        // Latency
        self.total_processing_time += result.processing_time;
        if self.latency_samples.len() >= MAX_LATENCY_SAMPLES {
            self.latency_samples.pop_front();
        }
        self.latency_samples.push_back(result.processing_time);

        // Quality
        self.bits_demodulated += result.demod_result.bits.len() as u64;
        self.symbols_detected += result.demod_result.symbols.len() as u64;
    }

    /// Record received samples (before processing)
    pub fn record_receive(&mut self, samples: usize, bytes: usize) {
        if self.start_time.is_none() {
            self.start();
        }
        self.samples_received += samples as u64;
        self.bytes_received += bytes as u64;
        self.packets_received += 1;
    }

    /// Record a receive error
    pub fn record_receive_error(&mut self) {
        self.receive_errors += 1;
    }

    /// Record a processing error
    pub fn record_processing_error(&mut self) {
        self.processing_errors += 1;
    }

    /// Get elapsed time since start
    pub fn elapsed(&self) -> Duration {
        self.start_time
            .map(|t| t.elapsed())
            .unwrap_or(Duration::ZERO)
    }

    /// Calculate throughput in samples/sec
    pub fn throughput_samples_per_sec(&self) -> f64 {
        let elapsed = self.elapsed().as_secs_f64();
        if elapsed > 0.0 {
            self.samples_processed as f64 / elapsed
        } else {
            0.0
        }
    }

    /// Calculate throughput in Mbps (assuming complex f64 samples)
    pub fn throughput_mbps(&self) -> f64 {
        // Each sample is 16 bytes (2 * f64)
        let bytes = self.samples_processed * 16;
        let elapsed = self.elapsed().as_secs_f64();
        if elapsed > 0.0 {
            (bytes as f64 * 8.0) / (elapsed * 1_000_000.0)
        } else {
            0.0
        }
    }

    /// Calculate data rate in bits/sec (demodulated bits)
    pub fn data_rate_bps(&self) -> f64 {
        let elapsed = self.elapsed().as_secs_f64();
        if elapsed > 0.0 {
            self.bits_demodulated as f64 / elapsed
        } else {
            0.0
        }
    }

    /// Get minimum latency
    pub fn min_latency(&self) -> Duration {
        self.latency_samples
            .iter()
            .min()
            .copied()
            .unwrap_or(Duration::ZERO)
    }

    /// Get maximum latency
    pub fn max_latency(&self) -> Duration {
        self.latency_samples
            .iter()
            .max()
            .copied()
            .unwrap_or(Duration::ZERO)
    }

    /// Get average latency
    pub fn avg_latency(&self) -> Duration {
        if self.latency_samples.is_empty() {
            return Duration::ZERO;
        }
        let total: Duration = self.latency_samples.iter().sum();
        total / self.latency_samples.len() as u32
    }

    /// Get P99 latency (99th percentile)
    pub fn p99_latency(&self) -> Duration {
        if self.latency_samples.is_empty() {
            return Duration::ZERO;
        }
        let mut sorted: Vec<_> = self.latency_samples.iter().copied().collect();
        sorted.sort();
        let idx = (sorted.len() as f64 * 0.99) as usize;
        sorted.get(idx.min(sorted.len() - 1)).copied().unwrap_or(Duration::ZERO)
    }

    /// Generate summary
    pub fn summary(&self) -> MetricsSummary {
        MetricsSummary {
            elapsed_sec: self.elapsed().as_secs_f64(),
            samples_received: self.samples_received,
            samples_processed: self.samples_processed,
            packets_received: self.packets_received,
            bytes_received: self.bytes_received,
            throughput_samples_per_sec: self.throughput_samples_per_sec(),
            throughput_mbps: self.throughput_mbps(),
            data_rate_bps: self.data_rate_bps(),
            min_latency_us: self.min_latency().as_micros() as f64,
            max_latency_us: self.max_latency().as_micros() as f64,
            avg_latency_us: self.avg_latency().as_micros() as f64,
            p99_latency_us: self.p99_latency().as_micros() as f64,
            bits_demodulated: self.bits_demodulated,
            symbols_detected: self.symbols_detected,
            receive_errors: self.receive_errors,
            processing_errors: self.processing_errors,
        }
    }

    /// Reset all metrics
    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

/// Summary of benchmark metrics for reporting
#[derive(Debug, Clone)]
pub struct MetricsSummary {
    pub elapsed_sec: f64,
    pub samples_received: u64,
    pub samples_processed: u64,
    pub packets_received: u64,
    pub bytes_received: u64,
    pub throughput_samples_per_sec: f64,
    pub throughput_mbps: f64,
    pub data_rate_bps: f64,
    pub min_latency_us: f64,
    pub max_latency_us: f64,
    pub avg_latency_us: f64,
    pub p99_latency_us: f64,
    pub bits_demodulated: u64,
    pub symbols_detected: u64,
    pub receive_errors: u64,
    pub processing_errors: u64,
}

impl MetricsSummary {
    /// Format elapsed time as HH:MM:SS
    pub fn elapsed_formatted(&self) -> String {
        let total_secs = self.elapsed_sec as u64;
        let hours = total_secs / 3600;
        let minutes = (total_secs % 3600) / 60;
        let seconds = total_secs % 60;
        format!("{:02}:{:02}:{:02}", hours, minutes, seconds)
    }

    /// Format throughput with units
    pub fn throughput_formatted(&self) -> String {
        if self.throughput_samples_per_sec >= 1_000_000.0 {
            format!("{:.2} MSps", self.throughput_samples_per_sec / 1_000_000.0)
        } else if self.throughput_samples_per_sec >= 1_000.0 {
            format!("{:.2} kSps", self.throughput_samples_per_sec / 1_000.0)
        } else {
            format!("{:.2} Sps", self.throughput_samples_per_sec)
        }
    }

    /// Format latency with units
    pub fn latency_formatted(&self) -> String {
        if self.avg_latency_us >= 1000.0 {
            format!("{:.2} ms", self.avg_latency_us / 1000.0)
        } else {
            format!("{:.2} us", self.avg_latency_us)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::waveform::DemodResult;

    fn make_result(samples: usize, latency_us: u64) -> ProcessResult {
        ProcessResult {
            demod_result: DemodResult {
                bits: vec![0; 8],
                symbols: vec![],
                ber_estimate: None,
                snr_estimate: None,
                metadata: std::collections::HashMap::new(),
            },
            processing_time: Duration::from_micros(latency_us),
            samples_processed: samples,
        }
    }

    #[test]
    fn test_metrics_new() {
        let metrics = BenchmarkMetrics::new();
        assert_eq!(metrics.samples_processed, 0);
        assert_eq!(metrics.throughput_samples_per_sec(), 0.0);
    }

    #[test]
    fn test_metrics_update() {
        let mut metrics = BenchmarkMetrics::new();
        metrics.start();

        let result = make_result(1000, 100);
        metrics.update(&result);

        assert_eq!(metrics.samples_processed, 1000);
        assert_eq!(metrics.bits_demodulated, 8);
    }

    #[test]
    fn test_latency_percentiles() {
        let mut metrics = BenchmarkMetrics::new();
        metrics.start();

        // Add varying latencies
        for i in 1..=100 {
            let result = make_result(100, i * 10);
            metrics.update(&result);
        }

        let summary = metrics.summary();
        assert!(summary.min_latency_us >= 10.0);
        assert!(summary.max_latency_us >= 990.0);
        assert!(summary.p99_latency_us >= 900.0);
    }

    #[test]
    fn test_elapsed_formatted() {
        let summary = MetricsSummary {
            elapsed_sec: 3661.0, // 1 hour, 1 minute, 1 second
            samples_received: 0,
            samples_processed: 0,
            packets_received: 0,
            bytes_received: 0,
            throughput_samples_per_sec: 0.0,
            throughput_mbps: 0.0,
            data_rate_bps: 0.0,
            min_latency_us: 0.0,
            max_latency_us: 0.0,
            avg_latency_us: 0.0,
            p99_latency_us: 0.0,
            bits_demodulated: 0,
            symbols_detected: 0,
            receive_errors: 0,
            processing_errors: 0,
        };

        assert_eq!(summary.elapsed_formatted(), "01:01:01");
    }
}
