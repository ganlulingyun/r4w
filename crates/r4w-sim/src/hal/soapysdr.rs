//! # SoapySDR Driver Stub
//!
//! This module provides a stub implementation for SoapySDR-compatible devices.
//! SoapySDR is a vendor-neutral SDR abstraction layer that supports many
//! different hardware platforms through a unified API.
//!
//! ## Supported Hardware
//!
//! SoapySDR supports many devices through plugins:
//! - LimeSDR
//! - HackRF
//! - PlutoSDR
//! - BladeRF
//! - Airspy
//! - RTL-SDR (through soapy-rtlsdr)
//! - USRP (through soapy-uhd)
//! - And many more...
//!
//! ## Usage
//!
//! ```rust,ignore
//! use r4w_sim::hal::soapysdr::SoapySdrDriver;
//! use r4w_sim::hal::DriverRegistry;
//!
//! let mut registry = DriverRegistry::new();
//! registry.register(Box::new(SoapySdrDriver::new()));
//!
//! // Discover all SoapySDR devices
//! let devices = registry.get("soapysdr").unwrap().discover();
//! for dev in devices {
//!     println!("Found: {} ({})", dev.name, dev.driver);
//! }
//!
//! // Create device with args
//! let device = registry.create("soapysdr://driver=lime")?;
//! ```
//!
//! ## Real Implementation
//!
//! A real implementation would use the `soapysdr` crate. The stub here
//! provides the interface structure for when hardware is unavailable.

use super::{
    ClockControl, ClockSource, DeviceDriver, SdrDeviceExt,
    SdrResult, StreamConfig, StreamDirection, StreamHandle, StreamStatus,
    TunerControl,
};
use crate::device::{DeviceCapabilities, DeviceInfo, SdrConfig, SdrError};
use r4w_core::timing::{HardwareClock, TimeSource, Timestamp};
use r4w_core::types::IQSample;
use std::collections::HashMap;
use std::time::Duration;

/// SoapySDR device driver.
///
/// This is a stub implementation that provides the interface structure
/// for SoapySDR devices. A real implementation would use the `soapysdr` crate.
pub struct SoapySdrDriver;

impl SoapySdrDriver {
    /// Create a new SoapySDR driver.
    pub fn new() -> Self {
        Self
    }

    /// Check if SoapySDR is available.
    ///
    /// Returns true if the library is loaded and functional.
    pub fn is_available(&self) -> bool {
        // In a real implementation, try to load libSoapySDR
        // For now, always return false (stub)
        false
    }

    /// List available SoapySDR modules.
    pub fn list_modules(&self) -> Vec<String> {
        // Would call SoapySDR::listModules()
        Vec::new()
    }
}

impl Default for SoapySdrDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl DeviceDriver for SoapySdrDriver {
    fn name(&self) -> &str {
        "soapysdr"
    }

    fn discover(&self) -> Vec<DeviceInfo> {
        // Stub: In real implementation, call SoapySDR::enumerate()
        tracing::debug!("SoapySDR discover: stub implementation, no devices");
        Vec::new()
    }

    fn create(&self, info: &DeviceInfo) -> SdrResult<Box<dyn SdrDeviceExt>> {
        if !self.is_available() {
            return Err(SdrError::DeviceNotFound(
                "SoapySDR library not available. Install libSoapySDR.".to_string(),
            ));
        }

        Ok(Box::new(SoapySdrDevice::new(info.clone())?))
    }

    fn create_from_string(&self, args: &str) -> SdrResult<Box<dyn SdrDeviceExt>> {
        // Parse args as key=value pairs
        let mut params = HashMap::new();
        for pair in args.split(',') {
            if let Some(pos) = pair.find('=') {
                let key = pair[..pos].trim();
                let value = pair[pos + 1..].trim();
                params.insert(key.to_string(), value.to_string());
            }
        }

        let driver = params.get("driver").cloned().unwrap_or_default();
        let serial = params.get("serial").cloned().unwrap_or_default();

        let info = DeviceInfo {
            driver: format!("soapysdr:{}", driver),
            serial,
            label: format!("SoapySDR ({})", if driver.is_empty() { "auto" } else { &driver }),
            address: args.to_string(),
        };

        self.create(&info)
    }
}

/// SoapySDR device instance.
pub struct SoapySdrDevice {
    info: DeviceInfo,
    config: SdrConfig,
    clock_source: ClockSource,
    time_source: TimeSource,
    antenna_rx: String,
    #[allow(dead_code)]
    antenna_tx: String,
    // In real implementation: SoapySDR::Device handle
}

impl SoapySdrDevice {
    /// Create a new SoapySDR device.
    fn new(info: DeviceInfo) -> SdrResult<Self> {
        Ok(Self {
            info,
            config: SdrConfig {
                frequency: 915_000_000.0,
                sample_rate: 1_000_000.0,
                bandwidth: 1_000_000.0,
                rx_gain: 30.0,
                tx_gain: 0.0,
                antenna: "RX".to_string(),
                buffer_size: 8192,
            },
            clock_source: ClockSource::Internal,
            time_source: TimeSource::Freerun,
            antenna_rx: "RX".to_string(),
            antenna_tx: "TX".to_string(),
        })
    }

    /// Get the underlying driver type from the driver string.
    pub fn driver_type(&self) -> &str {
        // Extract driver type from "soapysdr:lime" format
        self.info.driver.strip_prefix("soapysdr:").unwrap_or(&self.info.driver)
    }
}

impl SdrDeviceExt for SoapySdrDevice {
    fn name(&self) -> &str {
        &self.info.label
    }

    fn capabilities(&self) -> DeviceCapabilities {
        // Generic SoapySDR capabilities - real device would query actual hardware
        DeviceCapabilities {
            can_rx: true,
            can_tx: true,
            full_duplex: true,
            rx_channels: 1,
            tx_channels: 1,
            min_frequency: 1_000_000.0,      // 1 MHz
            max_frequency: 6_000_000_000.0,  // 6 GHz
            max_sample_rate: 61_440_000.0, // 61.44 MS/s
        }
    }

    fn config(&self) -> &SdrConfig {
        &self.config
    }

    fn configure(&mut self, config: &SdrConfig) -> SdrResult<()> {
        self.config = config.clone();
        Ok(())
    }

    fn create_rx_stream(&mut self, _config: StreamConfig) -> SdrResult<Box<dyn StreamHandle>> {
        Ok(Box::new(SoapySdrStream::new(
            StreamDirection::Rx,
            self.config.clone(),
        )))
    }

    fn create_tx_stream(&mut self, _config: StreamConfig) -> SdrResult<Box<dyn StreamHandle>> {
        Ok(Box::new(SoapySdrStream::new(
            StreamDirection::Tx,
            self.config.clone(),
        )))
    }

    fn tuner(&mut self) -> &mut dyn TunerControl {
        self
    }

    fn clock(&mut self) -> Option<&mut dyn ClockControl> {
        Some(self)
    }

    fn supports_gps(&self) -> bool {
        // Would query device.hasGainMode(SOAPY_SDR_RX, 0, "GPS")
        false
    }

    fn supports_pps(&self) -> bool {
        // Some SoapySDR devices support PPS
        false
    }

    fn supports_external_clock(&self) -> bool {
        // Would query device.listClockSources()
        true
    }
}

impl TunerControl for SoapySdrDevice {
    fn set_frequency(&mut self, freq_hz: u64) -> SdrResult<u64> {
        self.config.frequency = freq_hz as f64;
        Ok(freq_hz)
    }

    fn frequency(&self) -> u64 {
        self.config.frequency as u64
    }

    fn set_sample_rate(&mut self, rate: f64) -> SdrResult<f64> {
        self.config.sample_rate = rate;
        Ok(rate)
    }

    fn sample_rate(&self) -> f64 {
        self.config.sample_rate
    }

    fn set_bandwidth(&mut self, bw_hz: f64) -> SdrResult<f64> {
        self.config.bandwidth = bw_hz;
        Ok(bw_hz)
    }

    fn bandwidth(&self) -> f64 {
        self.config.bandwidth
    }

    fn set_rx_gain(&mut self, gain_db: f64) -> SdrResult<f64> {
        self.config.rx_gain = gain_db;
        Ok(gain_db)
    }

    fn rx_gain(&self) -> f64 {
        self.config.rx_gain
    }

    fn set_tx_gain(&mut self, gain_db: f64) -> SdrResult<f64> {
        self.config.tx_gain = gain_db;
        Ok(gain_db)
    }

    fn tx_gain(&self) -> f64 {
        self.config.tx_gain
    }

    fn set_antenna(&mut self, antenna: &str) -> SdrResult<()> {
        self.antenna_rx = antenna.to_string();
        Ok(())
    }

    fn antenna(&self) -> &str {
        &self.antenna_rx
    }

    fn available_antennas(&self) -> Vec<String> {
        // Would call device.listAntennas()
        vec!["RX".to_string(), "TX".to_string(), "LNAW".to_string(), "LNAL".to_string()]
    }

    fn frequency_range(&self) -> (u64, u64) {
        // Would call device.getFrequencyRange()
        (1_000_000, 6_000_000_000)
    }

    fn sample_rate_range(&self) -> (f64, f64) {
        // Would call device.getSampleRateRange()
        (100_000.0, 61_440_000.0)
    }

    fn gain_range(&self) -> (f64, f64) {
        // Would call device.getGainRange()
        (0.0, 70.0)
    }
}

impl ClockControl for SoapySdrDevice {
    fn set_clock_source(&mut self, source: ClockSource) -> SdrResult<()> {
        self.clock_source = source;
        Ok(())
    }

    fn clock_source(&self) -> ClockSource {
        self.clock_source
    }

    fn set_time_source(&mut self, source: TimeSource) -> SdrResult<()> {
        self.time_source = source;
        Ok(())
    }

    fn time_source(&self) -> TimeSource {
        self.time_source
    }

    fn time(&self) -> Timestamp {
        Timestamp::default()
    }

    fn set_time(&mut self, _time: Timestamp) -> SdrResult<()> {
        Ok(())
    }

    fn set_time_at_pps(&mut self, _time: Timestamp) -> SdrResult<()> {
        Err(SdrError::ConfigError(
            "SoapySDR stub: PPS not available".to_string(),
        ))
    }

    fn wait_for_pps(&mut self, _timeout: Duration) -> SdrResult<()> {
        Err(SdrError::ConfigError(
            "SoapySDR stub: PPS not available".to_string(),
        ))
    }

    fn is_locked(&self) -> bool {
        matches!(self.clock_source, ClockSource::Internal)
    }

    fn hardware_clock(&self) -> Option<HardwareClock> {
        None
    }
}

/// SoapySDR stream.
struct SoapySdrStream {
    direction: StreamDirection,
    running: bool,
    #[allow(dead_code)]
    config: SdrConfig,
    status: StreamStatus,
}

impl SoapySdrStream {
    fn new(direction: StreamDirection, config: SdrConfig) -> Self {
        Self {
            direction,
            running: false,
            config,
            status: StreamStatus::default(),
        }
    }
}

impl StreamHandle for SoapySdrStream {
    fn direction(&self) -> StreamDirection {
        self.direction
    }

    fn start(&mut self) -> SdrResult<()> {
        self.running = true;
        Ok(())
    }

    fn stop(&mut self) -> SdrResult<()> {
        self.running = false;
        Ok(())
    }

    fn is_running(&self) -> bool {
        self.running
    }

    fn read(&mut self, _buffer: &mut [IQSample], _timeout: Duration) -> SdrResult<(usize, Timestamp)> {
        if !self.running {
            return Err(SdrError::NotStarted);
        }

        if self.direction != StreamDirection::Rx {
            return Err(SdrError::ConfigError("Cannot read from TX stream".to_string()));
        }

        // Stub: real implementation would call stream.read()
        Err(SdrError::HardwareError(
            "SoapySDR stub: no real hardware connected".to_string(),
        ))
    }

    fn write(
        &mut self,
        _buffer: &[IQSample],
        _timestamp: Option<Timestamp>,
        _timeout: Duration,
    ) -> SdrResult<usize> {
        if !self.running {
            return Err(SdrError::NotStarted);
        }

        if self.direction != StreamDirection::Tx {
            return Err(SdrError::ConfigError("Cannot write to RX stream".to_string()));
        }

        // Stub: real implementation would call stream.write()
        Err(SdrError::HardwareError(
            "SoapySDR stub: no real hardware connected".to_string(),
        ))
    }

    fn status(&self) -> StreamStatus {
        self.status.clone()
    }

    fn timestamp(&self) -> Timestamp {
        Timestamp::default()
    }

    fn available(&self) -> usize {
        0
    }

    fn free_space(&self) -> usize {
        0
    }
}

/// Helper to parse SoapySDR-style argument strings.
///
/// Format: "key1=value1,key2=value2,..."
pub fn parse_soapy_args(args: &str) -> HashMap<String, String> {
    let mut result = HashMap::new();
    for pair in args.split(',') {
        if let Some(pos) = pair.find('=') {
            let key = pair[..pos].trim().to_string();
            let value = pair[pos + 1..].trim().to_string();
            result.insert(key, value);
        }
    }
    result
}

/// Format a HashMap as SoapySDR-style argument string.
pub fn format_soapy_args(args: &HashMap<String, String>) -> String {
    args.iter()
        .map(|(k, v)| format!("{}={}", k, v))
        .collect::<Vec<_>>()
        .join(",")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_driver_name() {
        let driver = SoapySdrDriver::new();
        assert_eq!(driver.name(), "soapysdr");
    }

    #[test]
    fn test_driver_discover() {
        let driver = SoapySdrDriver::new();
        let devices = driver.discover();
        // Stub returns empty list
        assert!(devices.is_empty());
    }

    #[test]
    fn test_parse_soapy_args() {
        let args = parse_soapy_args("driver=lime,serial=12345");
        assert_eq!(args.get("driver"), Some(&"lime".to_string()));
        assert_eq!(args.get("serial"), Some(&"12345".to_string()));
    }

    #[test]
    fn test_format_soapy_args() {
        let mut args = HashMap::new();
        args.insert("driver".to_string(), "hackrf".to_string());
        let formatted = format_soapy_args(&args);
        assert_eq!(formatted, "driver=hackrf");
    }

    #[test]
    fn test_device_capabilities() {
        let info = DeviceInfo {
            driver: "soapysdr:lime".to_string(),
            serial: String::new(),
            label: "LimeSDR".to_string(),
            address: "driver=lime".to_string(),
        };

        let device = SoapySdrDevice::new(info).unwrap();
        let caps = device.capabilities();

        assert!(caps.can_rx);
        assert!(caps.can_tx);
        assert!(caps.full_duplex);
    }

    #[test]
    fn test_clock_control() {
        let info = DeviceInfo {
            driver: "soapysdr".to_string(),
            serial: String::new(),
            label: "Test".to_string(),
            address: String::new(),
        };

        let mut device = SoapySdrDevice::new(info).unwrap();

        // Default clock source
        assert_eq!(device.clock_source(), ClockSource::Internal);

        // Change clock source
        device.set_clock_source(ClockSource::External).unwrap();
        assert_eq!(device.clock_source(), ClockSource::External);
    }

    #[test]
    fn test_available_antennas() {
        let info = DeviceInfo {
            driver: "soapysdr".to_string(),
            serial: String::new(),
            label: "Test".to_string(),
            address: String::new(),
        };

        let device = SoapySdrDevice::new(info).unwrap();
        let antennas = device.available_antennas();

        assert!(!antennas.is_empty());
        assert!(antennas.contains(&"RX".to_string()));
    }
}
