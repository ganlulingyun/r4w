//! # Structured Logging
//!
//! Provides structured logging via the `tracing` ecosystem with support for:
//!
//! - Multiple output formats (JSON, Pretty, Compact)
//! - Log level filtering
//! - File and stdout output
//! - Environment variable configuration
//!
//! ## Example
//!
//! ```rust,ignore
//! use r4w_core::observe::{init_logging, LogConfig, LogLevel};
//!
//! let config = LogConfig {
//!     level: LogLevel::Debug,
//!     format: LogFormat::Json,
//!     ..Default::default()
//! };
//!
//! init_logging(&config);
//!
//! tracing::info!(samples = 1024, "Processing complete");
//! ```

use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use tracing_subscriber::{fmt, prelude::*, EnvFilter};

/// Log level configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum LogLevel {
    /// Trace level (most verbose)
    Trace,
    /// Debug level
    Debug,
    /// Info level (default)
    Info,
    /// Warning level
    Warn,
    /// Error level (least verbose)
    Error,
}

impl Default for LogLevel {
    fn default() -> Self {
        LogLevel::Info
    }
}

impl std::fmt::Display for LogLevel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LogLevel::Trace => write!(f, "trace"),
            LogLevel::Debug => write!(f, "debug"),
            LogLevel::Info => write!(f, "info"),
            LogLevel::Warn => write!(f, "warn"),
            LogLevel::Error => write!(f, "error"),
        }
    }
}

/// Log output format.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum LogFormat {
    /// JSON format (machine-readable)
    Json,
    /// Pretty format (human-readable, colored)
    Pretty,
    /// Compact format (minimal, one line per event)
    Compact,
}

impl Default for LogFormat {
    fn default() -> Self {
        LogFormat::Pretty
    }
}

/// Logging configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LogConfig {
    /// Log level
    pub level: LogLevel,
    /// Output format
    pub format: LogFormat,
    /// Log file path (None for stdout only)
    pub file: Option<PathBuf>,
    /// Include timestamps
    pub timestamps: bool,
    /// Include source location (file:line)
    pub source_location: bool,
    /// Include thread IDs
    pub thread_ids: bool,
    /// Include thread names
    pub thread_names: bool,
    /// Include span events (enter/exit)
    pub span_events: bool,
    /// Module filter (e.g., "r4w_core=debug,r4w_sim=trace")
    pub filter: Option<String>,
}

impl Default for LogConfig {
    fn default() -> Self {
        Self {
            level: LogLevel::Info,
            format: LogFormat::Pretty,
            file: None,
            timestamps: true,
            source_location: false,
            thread_ids: false,
            thread_names: false,
            span_events: false,
            filter: None,
        }
    }
}

impl LogConfig {
    /// Create a development configuration (verbose, pretty).
    pub fn development() -> Self {
        Self {
            level: LogLevel::Debug,
            format: LogFormat::Pretty,
            source_location: true,
            thread_names: true,
            span_events: true,
            ..Default::default()
        }
    }

    /// Create a production configuration (JSON, minimal).
    pub fn production() -> Self {
        Self {
            level: LogLevel::Info,
            format: LogFormat::Json,
            timestamps: true,
            ..Default::default()
        }
    }

    /// Create a quiet configuration (errors only).
    pub fn quiet() -> Self {
        Self {
            level: LogLevel::Error,
            format: LogFormat::Compact,
            timestamps: false,
            ..Default::default()
        }
    }
}

/// Initialize the global logging subscriber.
///
/// This should be called once at application startup.
/// Subsequent calls will be silently ignored.
///
/// # Example
///
/// ```rust,ignore
/// use r4w_core::observe::{init_logging, LogConfig};
///
/// init_logging(&LogConfig::default());
///
/// tracing::info!("Application started");
/// ```
pub fn init_logging(config: &LogConfig) {
    // Build the filter
    let filter = if let Some(ref custom) = config.filter {
        EnvFilter::try_new(custom).unwrap_or_else(|_| {
            EnvFilter::new(format!("{}", config.level))
        })
    } else {
        // Try RUST_LOG env var first, then use config level
        EnvFilter::try_from_default_env()
            .unwrap_or_else(|_| EnvFilter::new(format!("{}", config.level)))
    };

    // Determine span events
    let span_events = if config.span_events {
        fmt::format::FmtSpan::FULL
    } else {
        fmt::format::FmtSpan::NONE
    };

    // Build the subscriber based on format
    let result = match config.format {
        LogFormat::Json => {
            let subscriber = tracing_subscriber::registry()
                .with(filter)
                .with(
                    fmt::layer()
                        .json()
                        .with_file(config.source_location)
                        .with_line_number(config.source_location)
                        .with_thread_ids(config.thread_ids)
                        .with_thread_names(config.thread_names)
                        .with_span_events(span_events),
                );
            tracing::subscriber::set_global_default(subscriber)
        }
        LogFormat::Pretty => {
            let subscriber = tracing_subscriber::registry()
                .with(filter)
                .with(
                    fmt::layer()
                        .pretty()
                        .with_file(config.source_location)
                        .with_line_number(config.source_location)
                        .with_thread_ids(config.thread_ids)
                        .with_thread_names(config.thread_names)
                        .with_span_events(span_events),
                );
            tracing::subscriber::set_global_default(subscriber)
        }
        LogFormat::Compact => {
            let subscriber = tracing_subscriber::registry()
                .with(filter)
                .with(
                    fmt::layer()
                        .compact()
                        .with_file(config.source_location)
                        .with_line_number(config.source_location)
                        .with_thread_ids(config.thread_ids)
                        .with_thread_names(config.thread_names)
                        .with_span_events(span_events),
                );
            tracing::subscriber::set_global_default(subscriber)
        }
    };

    // Ignore error if subscriber was already set
    let _ = result;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_log_level_display() {
        assert_eq!(format!("{}", LogLevel::Debug), "debug");
        assert_eq!(format!("{}", LogLevel::Info), "info");
        assert_eq!(format!("{}", LogLevel::Error), "error");
    }

    #[test]
    fn test_config_presets() {
        let dev = LogConfig::development();
        assert_eq!(dev.level, LogLevel::Debug);
        assert_eq!(dev.format, LogFormat::Pretty);
        assert!(dev.source_location);

        let prod = LogConfig::production();
        assert_eq!(prod.level, LogLevel::Info);
        assert_eq!(prod.format, LogFormat::Json);

        let quiet = LogConfig::quiet();
        assert_eq!(quiet.level, LogLevel::Error);
        assert!(!quiet.timestamps);
    }
}
