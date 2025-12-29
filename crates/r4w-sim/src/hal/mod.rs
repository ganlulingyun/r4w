//! # Hardware Abstraction Layer (HAL)
//!
//! This module provides a layered abstraction for SDR hardware, supporting:
//!
//! - **StreamHandle**: Streaming I/Q samples with timestamps
//! - **TunerControl**: Frequency, sample rate, gain, bandwidth
//! - **ClockControl**: Clock source, time source, PPS synchronization
//! - **SdrDevice**: High-level device interface combining all capabilities
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                    Waveform Layer                           │
//! ├─────────────────────────────────────────────────────────────┤
//! │                  HAL Interface (Rust traits)                │
//! │   SdrDevice, StreamHandle, TunerControl, ClockControl       │
//! ├───────────────┬───────────────┬─────────────────────────────┤
//! │   Simulator   │   File I/O    │  Hardware Drivers           │
//! │               │   (SigMF)     │  (UHD, SoapySDR, RTL-SDR)   │
//! ├───────────────┴───────────────┴─────────────────────────────┤
//! │                  OS Abstraction (libc, winapi)              │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Example
//!
//! ```rust,ignore
//! use r4w_sim::hal::{StreamHandle, TunerControl};
//!
//! // Configure tuner
//! tuner.set_frequency(915_000_000)?;
//! tuner.set_sample_rate(1_000_000.0)?;
//! tuner.set_rx_gain(30.0)?;
//!
//! // Start streaming
//! stream.start()?;
//! let samples = stream.read(&mut buffer, Duration::from_millis(100))?;
//! stream.stop()?;
//! ```

use r4w_core::timing::{Timestamp, TimeSource, HardwareClock};
use r4w_core::types::IQSample;
use std::time::Duration;

pub mod attenuator;
pub mod rtlsdr;
#[cfg(feature = "rtlsdr")]
pub mod rtlsdr_ffi;
pub mod sigmf;
pub mod soapysdr;
pub mod uhd;

pub use crate::device::{DeviceCapabilities, DeviceInfo, SdrConfig, SdrError, SdrResult};
pub use attenuator::{Attenuator, AttenuatorCapabilities, AttenuatorTestHarness, create_attenuator};
pub use rtlsdr::RtlSdrDriver;
pub use sigmf::{FileDriver, SigMfDevice, SigMfMeta, SigMfReader, SigMfWriter};
pub use soapysdr::SoapySdrDriver;
pub use uhd::UhdDriver;

/// Clock source for hardware clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ClockSource {
    /// Internal oscillator
    #[default]
    Internal,
    /// External reference input
    External,
    /// GPS disciplined oscillator
    Gpsdo,
    /// MIMO cable (for multi-device sync)
    Mimo,
}

/// Stream direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamDirection {
    /// Receive stream
    Rx,
    /// Transmit stream
    Tx,
}

/// Stream configuration.
#[derive(Debug, Clone)]
pub struct StreamConfig {
    /// Stream direction
    pub direction: StreamDirection,
    /// Number of channels
    pub channels: Vec<usize>,
    /// Buffer size in samples
    pub buffer_size: usize,
    /// Number of buffers (for DMA)
    pub num_buffers: usize,
    /// Sample format (not yet used - always f32 IQ)
    pub format: SampleFormat,
}

impl Default for StreamConfig {
    fn default() -> Self {
        Self {
            direction: StreamDirection::Rx,
            channels: vec![0],
            buffer_size: 8192,
            num_buffers: 4,
            format: SampleFormat::ComplexFloat32,
        }
    }
}

/// Sample format.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SampleFormat {
    /// 32-bit float I/Q (our native format)
    #[default]
    ComplexFloat32,
    /// 16-bit signed integer I/Q
    ComplexInt16,
    /// 8-bit signed integer I/Q
    ComplexInt8,
}

/// Stream status information.
#[derive(Debug, Clone, Default)]
pub struct StreamStatus {
    /// Number of overflow events (RX buffer full, samples lost)
    pub overflow_count: u64,
    /// Number of underflow events (TX buffer empty)
    pub underflow_count: u64,
    /// Number of late packet events (TX samples arrived too late)
    pub late_count: u64,
    /// Total samples processed
    pub samples_processed: u64,
    /// Current buffer fill level (samples)
    pub buffer_level: usize,
}

/// Streaming interface for I/Q samples.
///
/// This trait handles the actual data streaming with timestamps.
pub trait StreamHandle: Send {
    /// Get the stream direction.
    fn direction(&self) -> StreamDirection;

    /// Start the stream.
    fn start(&mut self) -> SdrResult<()>;

    /// Stop the stream.
    fn stop(&mut self) -> SdrResult<()>;

    /// Check if the stream is running.
    fn is_running(&self) -> bool;

    /// Read samples from an RX stream.
    ///
    /// Returns the number of samples actually read and the timestamp
    /// of the first sample.
    ///
    /// # Arguments
    /// * `buffer` - Buffer to fill with samples
    /// * `timeout` - Maximum time to wait for samples
    fn read(&mut self, buffer: &mut [IQSample], timeout: Duration) -> SdrResult<(usize, Timestamp)>;

    /// Write samples to a TX stream.
    ///
    /// Returns the number of samples actually written.
    ///
    /// # Arguments
    /// * `buffer` - Samples to transmit
    /// * `timestamp` - When to transmit (None = immediate)
    /// * `timeout` - Maximum time to wait for buffer space
    fn write(
        &mut self,
        buffer: &[IQSample],
        timestamp: Option<Timestamp>,
        timeout: Duration,
    ) -> SdrResult<usize>;

    /// Get current stream status.
    fn status(&self) -> StreamStatus;

    /// Get current timestamp.
    fn timestamp(&self) -> Timestamp;

    /// Get the number of samples available to read.
    fn available(&self) -> usize;

    /// Get the number of samples that can be written.
    fn free_space(&self) -> usize;
}

/// Tuner control interface for frequency, gain, and sample rate.
pub trait TunerControl: Send {
    /// Set center frequency.
    ///
    /// Returns the actual frequency set (may differ from requested).
    fn set_frequency(&mut self, freq_hz: u64) -> SdrResult<u64>;

    /// Get current center frequency.
    fn frequency(&self) -> u64;

    /// Set sample rate.
    ///
    /// Returns the actual sample rate set.
    fn set_sample_rate(&mut self, rate: f64) -> SdrResult<f64>;

    /// Get current sample rate.
    fn sample_rate(&self) -> f64;

    /// Set analog bandwidth.
    ///
    /// Returns the actual bandwidth set.
    fn set_bandwidth(&mut self, bw_hz: f64) -> SdrResult<f64>;

    /// Get current bandwidth.
    fn bandwidth(&self) -> f64;

    /// Set RX gain.
    ///
    /// Returns the actual gain set.
    fn set_rx_gain(&mut self, gain_db: f64) -> SdrResult<f64>;

    /// Get current RX gain.
    fn rx_gain(&self) -> f64;

    /// Set TX gain.
    ///
    /// Returns the actual gain set.
    fn set_tx_gain(&mut self, gain_db: f64) -> SdrResult<f64>;

    /// Get current TX gain.
    fn tx_gain(&self) -> f64;

    /// Set antenna port.
    fn set_antenna(&mut self, antenna: &str) -> SdrResult<()>;

    /// Get current antenna port.
    fn antenna(&self) -> &str;

    /// List available antenna ports.
    fn available_antennas(&self) -> Vec<String>;

    /// Get valid frequency range.
    fn frequency_range(&self) -> (u64, u64);

    /// Get valid sample rate range.
    fn sample_rate_range(&self) -> (f64, f64);

    /// Get valid gain range.
    fn gain_range(&self) -> (f64, f64);
}

/// Clock control interface for timing synchronization.
pub trait ClockControl: Send {
    /// Set clock source.
    fn set_clock_source(&mut self, source: ClockSource) -> SdrResult<()>;

    /// Get current clock source.
    fn clock_source(&self) -> ClockSource;

    /// Set time source.
    fn set_time_source(&mut self, source: TimeSource) -> SdrResult<()>;

    /// Get current time source.
    fn time_source(&self) -> TimeSource;

    /// Get current device time.
    fn time(&self) -> Timestamp;

    /// Set device time.
    fn set_time(&mut self, time: Timestamp) -> SdrResult<()>;

    /// Set time at next PPS edge.
    fn set_time_at_pps(&mut self, time: Timestamp) -> SdrResult<()>;

    /// Wait for PPS edge.
    fn wait_for_pps(&mut self, timeout: Duration) -> SdrResult<()>;

    /// Check if locked to external reference.
    fn is_locked(&self) -> bool;

    /// Get hardware clock info (if available).
    fn hardware_clock(&self) -> Option<HardwareClock>;
}

/// High-level SDR device interface.
///
/// Combines streaming, tuning, and clock control into a single interface.
/// This is the main interface for interacting with SDR hardware.
pub trait SdrDeviceExt: Send {
    /// Get device name/description.
    fn name(&self) -> &str;

    /// Get device capabilities.
    fn capabilities(&self) -> DeviceCapabilities;

    /// Get current configuration.
    fn config(&self) -> &SdrConfig;

    /// Apply configuration.
    fn configure(&mut self, config: &SdrConfig) -> SdrResult<()>;

    /// Create an RX stream.
    fn create_rx_stream(&mut self, config: StreamConfig) -> SdrResult<Box<dyn StreamHandle>>;

    /// Create a TX stream.
    fn create_tx_stream(&mut self, config: StreamConfig) -> SdrResult<Box<dyn StreamHandle>>;

    /// Get tuner control interface.
    fn tuner(&mut self) -> &mut dyn TunerControl;

    /// Get clock control interface (if supported).
    fn clock(&mut self) -> Option<&mut dyn ClockControl>;

    /// Check if device supports GPS synchronization.
    fn supports_gps(&self) -> bool {
        false
    }

    /// Check if device supports PPS input.
    fn supports_pps(&self) -> bool {
        false
    }

    /// Check if device supports external clock reference.
    fn supports_external_clock(&self) -> bool {
        false
    }
}

/// Driver factory for creating devices.
pub trait DeviceDriver: Send + Sync {
    /// Driver name (e.g., "uhd", "soapysdr", "rtlsdr", "simulator").
    fn name(&self) -> &str;

    /// Discover available devices.
    fn discover(&self) -> Vec<DeviceInfo>;

    /// Create a device instance.
    fn create(&self, info: &DeviceInfo) -> SdrResult<Box<dyn SdrDeviceExt>>;

    /// Create a device from connection string.
    fn create_from_string(&self, args: &str) -> SdrResult<Box<dyn SdrDeviceExt>>;
}

/// Registry of available device drivers.
pub struct DriverRegistry {
    drivers: Vec<Box<dyn DeviceDriver>>,
}

impl DriverRegistry {
    /// Create a new registry.
    pub fn new() -> Self {
        Self {
            drivers: Vec::new(),
        }
    }

    /// Register a device driver.
    pub fn register(&mut self, driver: Box<dyn DeviceDriver>) {
        self.drivers.push(driver);
    }

    /// Get a driver by name.
    pub fn get(&self, name: &str) -> Option<&dyn DeviceDriver> {
        self.drivers.iter()
            .find(|d| d.name() == name)
            .map(|d| d.as_ref())
    }

    /// List all available drivers.
    pub fn list(&self) -> Vec<&str> {
        self.drivers.iter().map(|d| d.name()).collect()
    }

    /// Discover all devices across all drivers.
    pub fn discover_all(&self) -> Vec<(String, DeviceInfo)> {
        let mut devices = Vec::new();
        for driver in &self.drivers {
            for info in driver.discover() {
                devices.push((driver.name().to_string(), info));
            }
        }
        devices
    }

    /// Create a device from a connection string.
    ///
    /// Format: "driver://args" (e.g., "uhd://type=b200", "simulator://")
    pub fn create(&self, uri: &str) -> SdrResult<Box<dyn SdrDeviceExt>> {
        let (driver_name, args) = if let Some(pos) = uri.find("://") {
            (&uri[..pos], &uri[pos + 3..])
        } else {
            return Err(SdrError::ConfigError(
                "Invalid URI format. Use 'driver://args'".to_string(),
            ));
        };

        let driver = self.get(driver_name).ok_or_else(|| {
            SdrError::DeviceNotFound(format!("Unknown driver: {}", driver_name))
        })?;

        driver.create_from_string(args)
    }
}

impl Default for DriverRegistry {
    fn default() -> Self {
        Self::new()
    }
}

/// Create a driver registry with all built-in drivers.
///
/// This includes:
/// - UHD driver (USRP devices: B200, N210, X310, etc.)
/// - RTL-SDR driver (cheap USB receivers)
/// - SoapySDR driver (generic SDR wrapper)
///
/// Note: Actual hardware access requires the real driver libraries to be installed.
pub fn create_default_registry() -> DriverRegistry {
    let mut registry = DriverRegistry::new();
    registry.register(Box::new(uhd::UhdDriver::new()));
    registry.register(Box::new(rtlsdr::RtlSdrDriver::new()));
    registry.register(Box::new(soapysdr::SoapySdrDriver::new()));
    registry.register(Box::new(sigmf::FileDriver::new()));
    registry
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stream_config_default() {
        let config = StreamConfig::default();
        assert_eq!(config.direction, StreamDirection::Rx);
        assert_eq!(config.channels, vec![0]);
        assert_eq!(config.buffer_size, 8192);
    }

    #[test]
    fn test_clock_source_default() {
        assert_eq!(ClockSource::default(), ClockSource::Internal);
    }

    #[test]
    fn test_sample_format_default() {
        assert_eq!(SampleFormat::default(), SampleFormat::ComplexFloat32);
    }

    #[test]
    fn test_driver_registry() {
        let registry = DriverRegistry::new();
        assert!(registry.list().is_empty());
        assert!(registry.get("simulator").is_none());
    }
}
