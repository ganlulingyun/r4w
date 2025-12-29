//! SDR Device Abstraction
//!
//! This module defines the common interface for all SDR devices.

use r4w_core::types::IQSample;
use serde::{Deserialize, Serialize};

/// SDR configuration parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SdrConfig {
    /// Center frequency in Hz
    pub frequency: f64,
    /// Sample rate in Hz
    pub sample_rate: f64,
    /// Bandwidth in Hz
    pub bandwidth: f64,
    /// Receive gain in dB
    pub rx_gain: f64,
    /// Transmit gain in dB
    pub tx_gain: f64,
    /// Antenna selection
    pub antenna: String,
    /// Buffer size in samples
    pub buffer_size: usize,
}

impl Default for SdrConfig {
    fn default() -> Self {
        Self {
            frequency: 915.0e6, // US ISM band
            sample_rate: 125_000.0, // Match LoRa BW
            bandwidth: 125_000.0,
            rx_gain: 30.0,
            tx_gain: 30.0,
            antenna: "TX/RX".to_string(),
            buffer_size: 8192,
        }
    }
}

impl SdrConfig {
    /// Create config for EU868 band
    pub fn eu868() -> Self {
        Self {
            frequency: 868.0e6,
            ..Default::default()
        }
    }

    /// Create config for US915 band
    pub fn us915() -> Self {
        Self {
            frequency: 915.0e6,
            ..Default::default()
        }
    }

    /// Set sample rate with oversampling
    pub fn with_oversample(mut self, factor: usize) -> Self {
        self.sample_rate = self.bandwidth * factor as f64;
        self
    }
}

/// Result type for SDR operations
pub type SdrResult<T> = Result<T, SdrError>;

/// Errors that can occur during SDR operations
#[derive(Debug, Clone, thiserror::Error)]
pub enum SdrError {
    #[error("Device not found: {0}")]
    DeviceNotFound(String),

    #[error("Configuration error: {0}")]
    ConfigError(String),

    #[error("Hardware error: {0}")]
    HardwareError(String),

    #[error("Buffer overflow")]
    BufferOverflow,

    #[error("Buffer underflow")]
    BufferUnderflow,

    #[error("Timeout waiting for {0}")]
    Timeout(String),

    #[error("Device not started")]
    NotStarted,

    #[error("Device already running")]
    AlreadyRunning,

    #[error("Unsupported operation: {0}")]
    Unsupported(String),
}

/// Device capabilities
#[derive(Debug, Clone, Default)]
pub struct DeviceCapabilities {
    /// Can transmit
    pub can_tx: bool,
    /// Can receive
    pub can_rx: bool,
    /// Supports full duplex
    pub full_duplex: bool,
    /// Minimum frequency in Hz
    pub min_frequency: f64,
    /// Maximum frequency in Hz
    pub max_frequency: f64,
    /// Maximum sample rate
    pub max_sample_rate: f64,
    /// Number of TX channels
    pub tx_channels: usize,
    /// Number of RX channels
    pub rx_channels: usize,
}

/// Common interface for SDR devices
pub trait SdrDevice: Send {
    /// Get device name/description
    fn name(&self) -> &str;

    /// Get device capabilities
    fn capabilities(&self) -> DeviceCapabilities;

    /// Configure the device
    fn configure(&mut self, config: &SdrConfig) -> SdrResult<()>;

    /// Get current configuration
    fn config(&self) -> &SdrConfig;

    /// Start receiving
    fn start_rx(&mut self) -> SdrResult<()>;

    /// Stop receiving
    fn stop_rx(&mut self) -> SdrResult<()>;

    /// Start transmitting
    fn start_tx(&mut self) -> SdrResult<()>;

    /// Stop transmitting
    fn stop_tx(&mut self) -> SdrResult<()>;

    /// Read samples from receive buffer
    fn read_samples(&mut self, num_samples: usize) -> SdrResult<Vec<IQSample>>;

    /// Write samples to transmit buffer
    fn write_samples(&mut self, samples: &[IQSample]) -> SdrResult<usize>;

    /// Get number of samples available in RX buffer
    fn rx_available(&self) -> usize;

    /// Get free space in TX buffer
    fn tx_available(&self) -> usize;

    /// Is device currently receiving?
    fn is_receiving(&self) -> bool;

    /// Is device currently transmitting?
    fn is_transmitting(&self) -> bool;

    /// Get current timestamp (samples since start)
    fn timestamp(&self) -> u64;

    /// Set frequency (can be called while running)
    fn set_frequency(&mut self, freq: f64) -> SdrResult<()>;

    /// Set RX gain
    fn set_rx_gain(&mut self, gain: f64) -> SdrResult<()>;

    /// Set TX gain
    fn set_tx_gain(&mut self, gain: f64) -> SdrResult<()>;
}

/// Device information for discovery
#[derive(Debug, Clone)]
pub struct DeviceInfo {
    /// Device type/driver
    pub driver: String,
    /// Device serial number
    pub serial: String,
    /// Device label/name
    pub label: String,
    /// Connection string
    pub address: String,
}

/// Discover available SDR devices
pub fn discover_devices() -> Vec<DeviceInfo> {
    let mut devices = Vec::new();

    // Always add simulator
    devices.push(DeviceInfo {
        driver: "simulator".to_string(),
        serial: "SIM001".to_string(),
        label: "Software Simulator".to_string(),
        address: "simulator://default".to_string(),
    });

    // TODO: Add UHD device discovery
    // TODO: Add SoapySDR device discovery

    devices
}
