//! Benchmark Report Generation
//!
//! Output formats: JSON, text, CSV

use super::metrics::MetricsSummary;
use serde::{Deserialize, Serialize};

/// System information for benchmark context
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemInfo {
    pub hostname: String,
    pub os: String,
    pub arch: String,
    pub cpu_cores: usize,
    pub rust_version: String,
}

impl Default for SystemInfo {
    fn default() -> Self {
        Self::collect()
    }
}

impl SystemInfo {
    /// Collect system information
    pub fn collect() -> Self {
        Self {
            hostname: hostname::get()
                .map(|h| h.to_string_lossy().to_string())
                .unwrap_or_else(|_| "unknown".to_string()),
            os: std::env::consts::OS.to_string(),
            arch: std::env::consts::ARCH.to_string(),
            cpu_cores: std::thread::available_parallelism()
                .map(|p| p.get())
                .unwrap_or(1),
            rust_version: env!("CARGO_PKG_RUST_VERSION").to_string(),
        }
    }
}

/// Complete benchmark report
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BenchmarkReport {
    pub waveform: String,
    pub sample_rate: f64,
    pub batch_size: usize,
    pub udp_port: u16,
    pub sample_format: String,
    pub metrics: ReportMetrics,
    pub system: SystemInfo,
    pub timestamp: String,
}

/// Metrics subset for reporting (serializable version of MetricsSummary)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReportMetrics {
    pub elapsed_sec: f64,
    pub samples_received: u64,
    pub samples_processed: u64,
    pub packets_received: u64,
    pub bytes_received: u64,
    pub throughput: ThroughputMetrics,
    pub latency: LatencyMetrics,
    pub quality: QualityMetrics,
    pub errors: ErrorMetrics,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThroughputMetrics {
    pub samples_per_sec: f64,
    pub mbps: f64,
    pub data_rate_bps: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LatencyMetrics {
    pub min_us: f64,
    pub max_us: f64,
    pub avg_us: f64,
    pub p99_us: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualityMetrics {
    pub bits_demodulated: u64,
    pub symbols_detected: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ErrorMetrics {
    pub receive: u64,
    pub processing: u64,
}

impl BenchmarkReport {
    /// Create a new report from metrics
    pub fn new(
        waveform: &str,
        sample_rate: f64,
        batch_size: usize,
        udp_port: u16,
        sample_format: &str,
        summary: &MetricsSummary,
    ) -> Self {
        Self {
            waveform: waveform.to_string(),
            sample_rate,
            batch_size,
            udp_port,
            sample_format: sample_format.to_string(),
            metrics: ReportMetrics::from(summary),
            system: SystemInfo::collect(),
            timestamp: chrono::Utc::now().to_rfc3339(),
        }
    }

    /// Output as JSON
    pub fn to_json(&self) -> String {
        serde_json::to_string_pretty(self).unwrap_or_else(|_| "{}".to_string())
    }

    /// Output as compact JSON (one line)
    pub fn to_json_compact(&self) -> String {
        serde_json::to_string(self).unwrap_or_else(|_| "{}".to_string())
    }

    /// Output as human-readable text
    pub fn to_text(&self) -> String {
        let mut s = String::new();

        s.push_str("SDR Waveform Benchmark Report\n");
        s.push_str("=============================\n\n");

        s.push_str(&format!("Waveform:      {}\n", self.waveform));
        s.push_str(&format!("Sample Rate:   {} Hz\n", self.sample_rate));
        s.push_str(&format!("Batch Size:    {} samples\n", self.batch_size));
        s.push_str(&format!("UDP Port:      {} ({})\n", self.udp_port, self.sample_format));
        s.push_str(&format!("Duration:      {}\n", format_duration(self.metrics.elapsed_sec)));
        s.push_str(&format!("Timestamp:     {}\n\n", self.timestamp));

        s.push_str("Throughput\n");
        s.push_str("----------\n");
        s.push_str(&format!("  Samples/sec:    {:>12.0}\n", self.metrics.throughput.samples_per_sec));
        s.push_str(&format!("  Data rate:      {:>12.2} Mbps\n", self.metrics.throughput.mbps));
        s.push_str(&format!("  Demod rate:     {:>12.0} bps\n\n", self.metrics.throughput.data_rate_bps));

        s.push_str("Latency\n");
        s.push_str("-------\n");
        s.push_str(&format!("  Average:        {:>12.1} us\n", self.metrics.latency.avg_us));
        s.push_str(&format!("  P99:            {:>12.1} us\n", self.metrics.latency.p99_us));
        s.push_str(&format!("  Min/Max:        {:>12.1} / {:.1} us\n\n",
            self.metrics.latency.min_us, self.metrics.latency.max_us));

        s.push_str("Quality\n");
        s.push_str("-------\n");
        s.push_str(&format!("  Bits decoded:   {:>12}\n", self.metrics.quality.bits_demodulated));
        s.push_str(&format!("  Symbols:        {:>12}\n\n", self.metrics.quality.symbols_detected));

        s.push_str("Errors\n");
        s.push_str("------\n");
        s.push_str(&format!("  Receive:        {:>12}\n", self.metrics.errors.receive));
        s.push_str(&format!("  Processing:     {:>12}\n\n", self.metrics.errors.processing));

        s.push_str("System\n");
        s.push_str("------\n");
        s.push_str(&format!("  Hostname:       {}\n", self.system.hostname));
        s.push_str(&format!("  OS/Arch:        {}/{}\n", self.system.os, self.system.arch));
        s.push_str(&format!("  CPU Cores:      {}\n", self.system.cpu_cores));

        s
    }

    /// Output as CSV header
    pub fn csv_header() -> &'static str {
        "timestamp,waveform,sample_rate,batch_size,elapsed_sec,samples_processed,throughput_sps,throughput_mbps,avg_latency_us,p99_latency_us,bits_demodulated,symbols_detected,receive_errors,processing_errors"
    }

    /// Output as CSV row
    pub fn to_csv_row(&self) -> String {
        format!(
            "{},{},{},{},{:.2},{},{:.0},{:.2},{:.1},{:.1},{},{},{},{}",
            self.timestamp,
            self.waveform,
            self.sample_rate,
            self.batch_size,
            self.metrics.elapsed_sec,
            self.metrics.samples_processed,
            self.metrics.throughput.samples_per_sec,
            self.metrics.throughput.mbps,
            self.metrics.latency.avg_us,
            self.metrics.latency.p99_us,
            self.metrics.quality.bits_demodulated,
            self.metrics.quality.symbols_detected,
            self.metrics.errors.receive,
            self.metrics.errors.processing,
        )
    }
}

impl From<&MetricsSummary> for ReportMetrics {
    fn from(s: &MetricsSummary) -> Self {
        Self {
            elapsed_sec: s.elapsed_sec,
            samples_received: s.samples_received,
            samples_processed: s.samples_processed,
            packets_received: s.packets_received,
            bytes_received: s.bytes_received,
            throughput: ThroughputMetrics {
                samples_per_sec: s.throughput_samples_per_sec,
                mbps: s.throughput_mbps,
                data_rate_bps: s.data_rate_bps,
            },
            latency: LatencyMetrics {
                min_us: s.min_latency_us,
                max_us: s.max_latency_us,
                avg_us: s.avg_latency_us,
                p99_us: s.p99_latency_us,
            },
            quality: QualityMetrics {
                bits_demodulated: s.bits_demodulated,
                symbols_detected: s.symbols_detected,
            },
            errors: ErrorMetrics {
                receive: s.receive_errors,
                processing: s.processing_errors,
            },
        }
    }
}

/// Format duration as HH:MM:SS
fn format_duration(secs: f64) -> String {
    let total_secs = secs as u64;
    let hours = total_secs / 3600;
    let minutes = (total_secs % 3600) / 60;
    let seconds = total_secs % 60;
    format!("{:02}:{:02}:{:02}", hours, minutes, seconds)
}

/// Live stats display for CLI
#[allow(dead_code)]
pub struct LiveStats {
    last_summary: Option<MetricsSummary>,
}

#[allow(dead_code)]
impl LiveStats {
    pub fn new() -> Self {
        Self { last_summary: None }
    }

    /// Update and return formatted live stats string
    pub fn update(&mut self, summary: &MetricsSummary) -> String {
        self.last_summary = Some(summary.clone());

        let mut s = String::new();

        // Clear screen and move cursor to top (ANSI escape codes)
        s.push_str("\x1B[2J\x1B[H");

        s.push_str("SDR Waveform Benchmark - Live Stats\n");
        s.push_str("====================================\n\n");

        s.push_str("Throughput:\n");
        s.push_str(&format!("  Samples/sec:    {:>12.0}\n", summary.throughput_samples_per_sec));
        s.push_str(&format!("  Data rate:      {:>12.2} Mbps\n\n", summary.throughput_mbps));

        s.push_str("Latency:\n");
        s.push_str(&format!("  Average:        {:>12.1} us\n", summary.avg_latency_us));
        s.push_str(&format!("  P99:            {:>12.1} us\n", summary.p99_latency_us));
        s.push_str(&format!("  Min/Max:        {:>12.1} / {:.1} us\n\n",
            summary.min_latency_us, summary.max_latency_us));

        s.push_str("Quality:\n");
        s.push_str(&format!("  Symbols:        {:>12}\n", summary.symbols_detected));
        s.push_str(&format!("  Bits:           {:>12}\n\n", summary.bits_demodulated));

        s.push_str("Errors:\n");
        s.push_str(&format!("  Receive:        {:>12}\n", summary.receive_errors));
        s.push_str(&format!("  Processing:     {:>12}\n\n", summary.processing_errors));

        s.push_str(&format!("Duration: {}\n", summary.elapsed_formatted()));
        s.push_str("\nPress Ctrl+C to stop...\n");

        s
    }
}

impl Default for LiveStats {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_summary() -> MetricsSummary {
        MetricsSummary {
            elapsed_sec: 60.0,
            samples_received: 1_000_000,
            samples_processed: 999_000,
            packets_received: 1000,
            bytes_received: 8_000_000,
            throughput_samples_per_sec: 16_650.0,
            throughput_mbps: 2.13,
            data_rate_bps: 9600.0,
            min_latency_us: 50.0,
            max_latency_us: 500.0,
            avg_latency_us: 100.0,
            p99_latency_us: 450.0,
            bits_demodulated: 576_000,
            symbols_detected: 72_000,
            receive_errors: 0,
            processing_errors: 1,
        }
    }

    #[test]
    fn test_report_json() {
        let summary = make_summary();
        let report = BenchmarkReport::new("BPSK", 48000.0, 1024, 5000, "f32", &summary);

        let json = report.to_json();
        assert!(json.contains("BPSK"));
        assert!(json.contains("48000"));
    }

    #[test]
    fn test_report_text() {
        let summary = make_summary();
        let report = BenchmarkReport::new("QPSK", 48000.0, 1024, 5000, "f32", &summary);

        let text = report.to_text();
        assert!(text.contains("QPSK"));
        assert!(text.contains("Throughput"));
        assert!(text.contains("Latency"));
    }

    #[test]
    fn test_report_csv() {
        let summary = make_summary();
        let report = BenchmarkReport::new("8-PSK", 48000.0, 1024, 5000, "i16", &summary);

        let csv = report.to_csv_row();
        assert!(csv.contains("8-PSK"));
        assert!(csv.contains("48000"));
    }

    #[test]
    fn test_format_duration() {
        assert_eq!(format_duration(0.0), "00:00:00");
        assert_eq!(format_duration(61.0), "00:01:01");
        assert_eq!(format_duration(3661.0), "01:01:01");
    }
}
