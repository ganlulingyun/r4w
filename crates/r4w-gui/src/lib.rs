//! Waveform Explorer - Educational SDR Visualization Library
//!
//! An interactive application for learning about Software Defined Radio
//! and digital modulation techniques including AM, FM, PSK, QAM, and LoRa.
//!
//! This library provides the core GUI components that can be used by both
//! native and web entry points.

pub mod app;
pub mod platform;
pub mod streaming;
pub mod views;

// Re-export main application type for convenience
pub use app::WaveformExplorer;
