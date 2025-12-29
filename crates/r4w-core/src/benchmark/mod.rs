//! Waveform Benchmarking Module
//!
//! Provides UDP-based benchmarking infrastructure for measuring waveform
//! processing performance. Supports:
//!
//! - UDP I/Q sample reception (f32 and i16 formats)
//! - Waveform processing with timing
//! - Comprehensive metrics (throughput, latency, quality)
//! - Multiple output formats (JSON, text, CSV)

mod metrics;
mod receiver;
mod report;
mod runner;

pub use metrics::{BenchmarkMetrics, MetricsSummary};
pub use receiver::{BenchmarkReceiver, BenchmarkSender, SampleFormat};
pub use report::{BenchmarkReport, SystemInfo};
pub use runner::{ProcessResult, WaveformRunner};
