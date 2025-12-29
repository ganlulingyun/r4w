//! # Observability Stack
//!
//! This module provides three-pillar observability for R4W:
//!
//! - **Logging**: Structured JSON logs via `tracing`
//! - **Metrics**: Prometheus-compatible counters, gauges, and histograms
//! - **Capture**: Real-time I/Q sample capture in SigMF format
//!
//! ## Quick Start
//!
//! ```rust,ignore
//! use r4w_core::observe::{init_logging, Metrics, LogConfig, CaptureManager, CaptureConfig};
//!
//! // Initialize logging
//! let config = LogConfig::default();
//! init_logging(&config);
//!
//! // Record metrics
//! let metrics = Metrics::new();
//! metrics.samples_processed.inc_by(1024);
//! metrics.buffer_level.set(500);
//!
//! // Capture samples
//! let mut capture = CaptureManager::new(CaptureConfig::default().enabled(true));
//! capture.arm();
//! capture.push_samples(&samples, timestamp);
//! capture.trigger("event_detected");
//! ```
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                    Application Code                         │
//! │   tracing::info!(), metrics.foo.inc(), capture.push()       │
//! └───────────────────────┬─────────────────────────────────────┘
//!                         │
//!     ┌───────────────────┼───────────────────┐
//!     │                   │                   │
//!     ▼                   ▼                   ▼
//! ┌─────────┐       ┌─────────┐        ┌──────────┐
//! │ Logging │       │ Metrics │        │ Capture  │
//! │ (JSON)  │       │ (Prom)  │        │ (SigMF)  │
//! └─────────┘       └─────────┘        └──────────┘
//!     │                   │                   │
//!     ▼                   ▼                   ▼
//!   File/            HTTP             File/Network
//!   Stdout           :9090            Recording
//! ```

pub mod capture;
pub mod logging;
pub mod metrics;

pub use capture::{CaptureConfig, CaptureInfo, CaptureManager, CaptureState, CaptureStats, TriggerMode};
pub use logging::{init_logging, LogConfig, LogFormat, LogLevel};
pub use metrics::{Metrics, MetricsSnapshot};

/// Initialize all observability subsystems.
///
/// This is a convenience function that sets up logging and creates a metrics instance.
pub fn init(log_config: &LogConfig) -> Metrics {
    init_logging(log_config);
    Metrics::new()
}
