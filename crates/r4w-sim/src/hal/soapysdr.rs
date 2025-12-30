//! # SoapySDR Driver
//!
//! Vendor-neutral SDR driver using the SoapySDR abstraction layer.
//!
//! ## Supported Hardware
//!
//! SoapySDR supports many devices through plugins:
//! - **LimeSDR**: Full-duplex, 100 kHz - 3.8 GHz, 61.44 MS/s
//! - **HackRF**: Half-duplex, 1 MHz - 6 GHz, 20 MS/s
//! - **PlutoSDR**: Full-duplex, 325 MHz - 3.8 GHz, 61.44 MS/s
//! - **BladeRF**: Full-duplex, 300 MHz - 3.8 GHz, 40 MS/s
//! - **Airspy**: RX only, 24 - 1800 MHz, 10 MS/s
//! - **RTL-SDR**: via soapy-rtlsdr plugin
//! - **USRP**: via soapy-uhd plugin
//!
//! ## Requirements
//!
//! Install libSoapySDR:
//! - **Linux**: `sudo apt install libsoapysdr-dev soapysdr-tools`
//! - **macOS**: `brew install soapysdr`
//! - **Windows**: Download from https://github.com/pothosware/SoapySDR
//!
//! Install device-specific plugins (e.g., `soapysdr-module-lms7` for LimeSDR).
//!
//! ## Usage
//!
//! ```rust,ignore
//! use r4w_sim::hal::{SoapySdrDriver, DeviceDriver, DriverRegistry};
//!
//! let mut registry = DriverRegistry::new();
//! registry.register(Box::new(SoapySdrDriver::new()));
//!
//! // Discover all SoapySDR devices
//! let devices = registry.get("soapysdr").unwrap().discover();
//! for dev in devices {
//!     println!("Found: {} ({})", dev.label, dev.driver);
//! }
//!
//! // Create device with args
//! let device = registry.create("soapysdr://driver=lime")?;
//! ```

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

#[cfg(feature = "soapysdr")]
use super::soapysdr_ffi::{self, SoapyDevice, SoapyStream, SOAPY_SDR_RX, SOAPY_SDR_TX};

#[cfg(feature = "soapysdr")]
use std::sync::{Arc, Mutex};

#[cfg(feature = "soapysdr")]
use std::time::Instant;

/// SoapySDR device driver.
///
/// When the `soapysdr` feature is enabled and libSoapySDR is available,
/// this driver provides real hardware access. Otherwise, it operates
/// as a stub.
pub struct SoapySdrDriver {
    #[cfg(feature = "soapysdr")]
    available: bool,
    #[cfg(not(feature = "soapysdr"))]
    _phantom: std::marker::PhantomData<()>,
}

impl SoapySdrDriver {
    /// Create a new SoapySDR driver.
    pub fn new() -> Self {
        #[cfg(feature = "soapysdr")]
        let available = soapysdr_ffi::is_available();

        #[cfg(feature = "soapysdr")]
        {
            if available {
                tracing::info!("SoapySDR driver initialized with hardware support");
            } else {
                tracing::debug!("SoapySDR driver initialized (stub mode - no libSoapySDR)");
            }
            Self { available }
        }

        #[cfg(not(feature = "soapysdr"))]
        {
            tracing::debug!("SoapySDR driver initialized (stub mode - feature not enabled)");
            Self {
                _phantom: std::marker::PhantomData,
            }
        }
    }

    /// Check if SoapySDR is available.
    ///
    /// Returns true if the library is loaded and functional.
    pub fn is_available(&self) -> bool {
        #[cfg(feature = "soapysdr")]
        {
            self.available
        }
        #[cfg(not(feature = "soapysdr"))]
        {
            false
        }
    }

    /// List available SoapySDR modules.
    pub fn list_modules(&self) -> Vec<String> {
        #[cfg(feature = "soapysdr")]
        {
            if !self.available {
                return Vec::new();
            }
            // Enumerate and extract unique driver names
            let devices = soapysdr_ffi::enumerate_devices(None);
            let mut drivers: Vec<String> = devices.iter().map(|d| d.driver.clone()).collect();
            drivers.sort();
            drivers.dedup();
            drivers
        }
        #[cfg(not(feature = "soapysdr"))]
        {
            Vec::new()
        }
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
        #[cfg(feature = "soapysdr")]
        {
            if !self.available {
                tracing::debug!("SoapySDR discover: library not available");
                return Vec::new();
            }

            let ffi_devices = soapysdr_ffi::enumerate_devices(None);
            let devices: Vec<DeviceInfo> = ffi_devices
                .iter()
                .map(|d| {
                    // Build address string from args
                    let address = d
                        .args
                        .iter()
                        .map(|(k, v)| format!("{}={}", k, v))
                        .collect::<Vec<_>>()
                        .join(",");

                    DeviceInfo {
                        driver: format!("soapysdr:{}", d.driver),
                        serial: d.serial.clone(),
                        label: d.label.clone(),
                        address,
                    }
                })
                .collect();

            tracing::debug!("SoapySDR found {} devices", devices.len());
            for dev in &devices {
                tracing::debug!("  - {} ({})", dev.label, dev.driver);
            }

            devices
        }

        #[cfg(not(feature = "soapysdr"))]
        {
            tracing::debug!("SoapySDR discover: stub implementation, no devices");
            Vec::new()
        }
    }

    fn create(&self, info: &DeviceInfo) -> SdrResult<Box<dyn SdrDeviceExt>> {
        if !self.is_available() {
            return Err(SdrError::DeviceNotFound(
                "SoapySDR library not available. Install libSoapySDR and enable 'soapysdr' feature.".to_string(),
            ));
        }

        #[cfg(feature = "soapysdr")]
        {
            // Parse address string back to args
            let mut args = HashMap::new();
            for pair in info.address.split(',') {
                if let Some(pos) = pair.find('=') {
                    let key = pair[..pos].trim().to_string();
                    let value = pair[pos + 1..].trim().to_string();
                    args.insert(key, value);
                }
            }

            let ffi_info = soapysdr_ffi::SoapyDeviceInfo {
                driver: info.driver.strip_prefix("soapysdr:").unwrap_or(&info.driver).to_string(),
                label: info.label.clone(),
                serial: info.serial.clone(),
                args,
            };

            let device = SoapyDevice::make(&ffi_info)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;

            Ok(Box::new(SoapySdrDevice::new(info.clone(), device)?))
        }

        #[cfg(not(feature = "soapysdr"))]
        {
            let _ = info;
            Err(SdrError::DeviceNotFound(
                "SoapySDR support not compiled in. Enable 'soapysdr' feature.".to_string(),
            ))
        }
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
#[cfg(feature = "soapysdr")]
pub struct SoapySdrDevice {
    info: DeviceInfo,
    config: SdrConfig,
    device: Arc<Mutex<SoapyDevice>>,
    clock_source: ClockSource,
    time_source: TimeSource,
    antenna_rx: String,
    antenna_tx: String,
    rx_channels: usize,
    tx_channels: usize,
    freq_range: (f64, f64),
    sample_rate_range: (f64, f64),
    gain_range: (f64, f64),
}

#[cfg(feature = "soapysdr")]
impl SoapySdrDevice {
    /// Create a new SoapySDR device.
    fn new(info: DeviceInfo, device: SoapyDevice) -> SdrResult<Self> {
        // Query device capabilities
        let rx_channels = device.num_rx_channels();
        let tx_channels = device.num_tx_channels();

        // Get frequency range (from RX channel 0 if available)
        let freq_range = if rx_channels > 0 {
            device.get_frequency_range(SOAPY_SDR_RX, 0)
        } else if tx_channels > 0 {
            device.get_frequency_range(SOAPY_SDR_TX, 0)
        } else {
            (1_000_000.0, 6_000_000_000.0) // Default
        };

        // Get sample rate range
        let sample_rate_range = if rx_channels > 0 {
            device.get_sample_rate_range(SOAPY_SDR_RX, 0)
        } else if tx_channels > 0 {
            device.get_sample_rate_range(SOAPY_SDR_TX, 0)
        } else {
            (100_000.0, 61_440_000.0) // Default
        };

        // Get gain range
        let gain_range = if rx_channels > 0 {
            device.get_gain_range(SOAPY_SDR_RX, 0)
        } else {
            (0.0, 70.0) // Default
        };

        // Get available antennas
        let rx_antennas = if rx_channels > 0 {
            device.list_antennas(SOAPY_SDR_RX, 0)
        } else {
            Vec::new()
        };
        let tx_antennas = if tx_channels > 0 {
            device.list_antennas(SOAPY_SDR_TX, 0)
        } else {
            Vec::new()
        };

        let antenna_rx = rx_antennas.first().cloned().unwrap_or_else(|| "RX".to_string());
        let antenna_tx = tx_antennas.first().cloned().unwrap_or_else(|| "TX".to_string());

        let mut dev = Self {
            info,
            config: SdrConfig {
                frequency: 915_000_000.0,
                sample_rate: 1_000_000.0,
                bandwidth: 1_000_000.0,
                rx_gain: 30.0,
                tx_gain: 0.0,
                antenna: antenna_rx.clone(),
                buffer_size: 8192,
            },
            device: Arc::new(Mutex::new(device)),
            clock_source: ClockSource::Internal,
            time_source: TimeSource::Freerun,
            antenna_rx,
            antenna_tx,
            rx_channels,
            tx_channels,
            freq_range,
            sample_rate_range,
            gain_range,
        };

        // Apply initial configuration
        dev.apply_config()?;

        Ok(dev)
    }

    /// Apply current configuration to hardware.
    fn apply_config(&mut self) -> SdrResult<()> {
        let device = self.device.lock().unwrap();

        // Set RX parameters if available
        if self.rx_channels > 0 {
            device
                .set_frequency(SOAPY_SDR_RX, 0, self.config.frequency)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;
            device
                .set_sample_rate(SOAPY_SDR_RX, 0, self.config.sample_rate)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;
            if self.config.bandwidth > 0.0 {
                device
                    .set_bandwidth(SOAPY_SDR_RX, 0, self.config.bandwidth)
                    .map_err(|e| SdrError::HardwareError(e.to_string()))?;
            }
            device
                .set_gain(SOAPY_SDR_RX, 0, self.config.rx_gain)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;
        }

        // Set TX parameters if available
        if self.tx_channels > 0 {
            device
                .set_frequency(SOAPY_SDR_TX, 0, self.config.frequency)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;
            device
                .set_sample_rate(SOAPY_SDR_TX, 0, self.config.sample_rate)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;
            device
                .set_gain(SOAPY_SDR_TX, 0, self.config.tx_gain)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;
        }

        Ok(())
    }
}

#[cfg(feature = "soapysdr")]
impl SdrDeviceExt for SoapySdrDevice {
    fn name(&self) -> &str {
        &self.info.label
    }

    fn capabilities(&self) -> DeviceCapabilities {
        DeviceCapabilities {
            can_rx: self.rx_channels > 0,
            can_tx: self.tx_channels > 0,
            full_duplex: self.rx_channels > 0 && self.tx_channels > 0,
            rx_channels: self.rx_channels,
            tx_channels: self.tx_channels,
            min_frequency: self.freq_range.0,
            max_frequency: self.freq_range.1,
            max_sample_rate: self.sample_rate_range.1,
        }
    }

    fn config(&self) -> &SdrConfig {
        &self.config
    }

    fn configure(&mut self, config: &SdrConfig) -> SdrResult<()> {
        self.config = config.clone();
        self.apply_config()
    }

    fn create_rx_stream(&mut self, config: StreamConfig) -> SdrResult<Box<dyn StreamHandle>> {
        if self.rx_channels == 0 {
            return Err(SdrError::ConfigError("Device does not support RX".to_string()));
        }

        let device = self.device.lock().unwrap();
        let channels: Vec<usize> = config.channels.clone();
        let stream = device
            .setup_stream(SOAPY_SDR_RX, &channels)
            .map_err(|e| SdrError::HardwareError(e.to_string()))?;

        Ok(Box::new(SoapySdrStream::new(
            StreamDirection::Rx,
            self.config.clone(),
            config,
            stream,
        )))
    }

    fn create_tx_stream(&mut self, config: StreamConfig) -> SdrResult<Box<dyn StreamHandle>> {
        if self.tx_channels == 0 {
            return Err(SdrError::ConfigError("Device does not support TX".to_string()));
        }

        let device = self.device.lock().unwrap();
        let channels: Vec<usize> = config.channels.clone();
        let stream = device
            .setup_stream(SOAPY_SDR_TX, &channels)
            .map_err(|e| SdrError::HardwareError(e.to_string()))?;

        Ok(Box::new(SoapySdrStream::new(
            StreamDirection::Tx,
            self.config.clone(),
            config,
            stream,
        )))
    }

    fn tuner(&mut self) -> &mut dyn TunerControl {
        self
    }

    fn clock(&mut self) -> Option<&mut dyn ClockControl> {
        Some(self)
    }

    fn supports_gps(&self) -> bool {
        // Would need to query device-specific capability
        false
    }

    fn supports_pps(&self) -> bool {
        // Some SoapySDR devices support PPS
        false
    }

    fn supports_external_clock(&self) -> bool {
        true // Most SoapySDR devices support external clock
    }
}

#[cfg(feature = "soapysdr")]
impl TunerControl for SoapySdrDevice {
    fn set_frequency(&mut self, freq_hz: u64) -> SdrResult<u64> {
        self.config.frequency = freq_hz as f64;

        let device = self.device.lock().unwrap();
        if self.rx_channels > 0 {
            device
                .set_frequency(SOAPY_SDR_RX, 0, self.config.frequency)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;
        }
        if self.tx_channels > 0 {
            device
                .set_frequency(SOAPY_SDR_TX, 0, self.config.frequency)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;
        }

        // Return actual frequency
        let actual = if self.rx_channels > 0 {
            device.get_frequency(SOAPY_SDR_RX, 0)
        } else {
            device.get_frequency(SOAPY_SDR_TX, 0)
        };

        Ok(actual as u64)
    }

    fn frequency(&self) -> u64 {
        let device = self.device.lock().unwrap();
        if self.rx_channels > 0 {
            device.get_frequency(SOAPY_SDR_RX, 0) as u64
        } else if self.tx_channels > 0 {
            device.get_frequency(SOAPY_SDR_TX, 0) as u64
        } else {
            self.config.frequency as u64
        }
    }

    fn set_sample_rate(&mut self, rate: f64) -> SdrResult<f64> {
        self.config.sample_rate = rate;

        let device = self.device.lock().unwrap();
        if self.rx_channels > 0 {
            device
                .set_sample_rate(SOAPY_SDR_RX, 0, rate)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;
        }
        if self.tx_channels > 0 {
            device
                .set_sample_rate(SOAPY_SDR_TX, 0, rate)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;
        }

        // Return actual sample rate
        let actual = if self.rx_channels > 0 {
            device.get_sample_rate(SOAPY_SDR_RX, 0)
        } else {
            device.get_sample_rate(SOAPY_SDR_TX, 0)
        };

        Ok(actual)
    }

    fn sample_rate(&self) -> f64 {
        let device = self.device.lock().unwrap();
        if self.rx_channels > 0 {
            device.get_sample_rate(SOAPY_SDR_RX, 0)
        } else if self.tx_channels > 0 {
            device.get_sample_rate(SOAPY_SDR_TX, 0)
        } else {
            self.config.sample_rate
        }
    }

    fn set_bandwidth(&mut self, bw_hz: f64) -> SdrResult<f64> {
        self.config.bandwidth = bw_hz;

        let device = self.device.lock().unwrap();
        if self.rx_channels > 0 {
            device
                .set_bandwidth(SOAPY_SDR_RX, 0, bw_hz)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;
        }
        if self.tx_channels > 0 {
            device
                .set_bandwidth(SOAPY_SDR_TX, 0, bw_hz)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;
        }

        let actual = if self.rx_channels > 0 {
            device.get_bandwidth(SOAPY_SDR_RX, 0)
        } else {
            device.get_bandwidth(SOAPY_SDR_TX, 0)
        };

        Ok(actual)
    }

    fn bandwidth(&self) -> f64 {
        let device = self.device.lock().unwrap();
        if self.rx_channels > 0 {
            device.get_bandwidth(SOAPY_SDR_RX, 0)
        } else if self.tx_channels > 0 {
            device.get_bandwidth(SOAPY_SDR_TX, 0)
        } else {
            self.config.bandwidth
        }
    }

    fn set_rx_gain(&mut self, gain_db: f64) -> SdrResult<f64> {
        self.config.rx_gain = gain_db;

        if self.rx_channels > 0 {
            let device = self.device.lock().unwrap();
            device
                .set_gain(SOAPY_SDR_RX, 0, gain_db)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;
            Ok(device.get_gain(SOAPY_SDR_RX, 0))
        } else {
            Ok(gain_db)
        }
    }

    fn rx_gain(&self) -> f64 {
        if self.rx_channels > 0 {
            let device = self.device.lock().unwrap();
            device.get_gain(SOAPY_SDR_RX, 0)
        } else {
            self.config.rx_gain
        }
    }

    fn set_tx_gain(&mut self, gain_db: f64) -> SdrResult<f64> {
        self.config.tx_gain = gain_db;

        if self.tx_channels > 0 {
            let device = self.device.lock().unwrap();
            device
                .set_gain(SOAPY_SDR_TX, 0, gain_db)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;
            Ok(device.get_gain(SOAPY_SDR_TX, 0))
        } else {
            Ok(gain_db)
        }
    }

    fn tx_gain(&self) -> f64 {
        if self.tx_channels > 0 {
            let device = self.device.lock().unwrap();
            device.get_gain(SOAPY_SDR_TX, 0)
        } else {
            self.config.tx_gain
        }
    }

    fn set_antenna(&mut self, antenna: &str) -> SdrResult<()> {
        if self.rx_channels > 0 {
            let device = self.device.lock().unwrap();
            device
                .set_antenna(SOAPY_SDR_RX, 0, antenna)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;
            self.antenna_rx = antenna.to_string();
        }
        Ok(())
    }

    fn antenna(&self) -> &str {
        &self.antenna_rx
    }

    fn available_antennas(&self) -> Vec<String> {
        if self.rx_channels > 0 {
            let device = self.device.lock().unwrap();
            device.list_antennas(SOAPY_SDR_RX, 0)
        } else {
            vec!["RX".to_string()]
        }
    }

    fn frequency_range(&self) -> (u64, u64) {
        (self.freq_range.0 as u64, self.freq_range.1 as u64)
    }

    fn sample_rate_range(&self) -> (f64, f64) {
        self.sample_rate_range
    }

    fn gain_range(&self) -> (f64, f64) {
        self.gain_range
    }
}

#[cfg(feature = "soapysdr")]
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
        Err(SdrError::ConfigError("PPS not supported".to_string()))
    }

    fn wait_for_pps(&mut self, _timeout: Duration) -> SdrResult<()> {
        Err(SdrError::ConfigError("PPS not supported".to_string()))
    }

    fn is_locked(&self) -> bool {
        matches!(self.clock_source, ClockSource::Internal)
    }

    fn hardware_clock(&self) -> Option<HardwareClock> {
        None
    }
}

/// SoapySDR stream implementation.
#[cfg(feature = "soapysdr")]
struct SoapySdrStream {
    direction: StreamDirection,
    running: bool,
    sdr_config: SdrConfig,
    stream_config: StreamConfig,
    stream: SoapyStream,
    status: StreamStatus,
    start_time: Option<Instant>,
    samples_processed: u64,
    buffer: Vec<[f32; 2]>,
}

#[cfg(feature = "soapysdr")]
impl SoapySdrStream {
    fn new(
        direction: StreamDirection,
        sdr_config: SdrConfig,
        stream_config: StreamConfig,
        stream: SoapyStream,
    ) -> Self {
        let buffer_size = stream_config.buffer_size;
        Self {
            direction,
            running: false,
            sdr_config,
            stream_config,
            stream,
            status: StreamStatus::default(),
            start_time: None,
            samples_processed: 0,
            buffer: vec![[0.0, 0.0]; buffer_size],
        }
    }
}

#[cfg(feature = "soapysdr")]
impl StreamHandle for SoapySdrStream {
    fn direction(&self) -> StreamDirection {
        self.direction
    }

    fn start(&mut self) -> SdrResult<()> {
        if self.running {
            return Ok(());
        }

        self.stream
            .activate()
            .map_err(|e| SdrError::HardwareError(e.to_string()))?;

        self.running = true;
        self.start_time = Some(Instant::now());
        self.samples_processed = 0;
        self.status = StreamStatus::default();

        tracing::info!(
            "SoapySDR {:?} stream started at {} MHz, {} MS/s",
            self.direction,
            self.sdr_config.frequency / 1_000_000.0,
            self.sdr_config.sample_rate / 1_000_000.0
        );

        Ok(())
    }

    fn stop(&mut self) -> SdrResult<()> {
        if !self.running {
            return Ok(());
        }

        self.stream
            .deactivate()
            .map_err(|e| SdrError::HardwareError(e.to_string()))?;

        self.running = false;
        tracing::info!(
            "SoapySDR stream stopped, {} samples processed",
            self.samples_processed
        );

        Ok(())
    }

    fn is_running(&self) -> bool {
        self.running
    }

    fn read(&mut self, buffer: &mut [IQSample], timeout: Duration) -> SdrResult<(usize, Timestamp)> {
        if !self.running {
            return Err(SdrError::NotStarted);
        }

        if self.direction != StreamDirection::Rx {
            return Err(SdrError::ConfigError("Cannot read from TX stream".to_string()));
        }

        let timeout_us = timeout.as_micros() as i64;
        let samples_to_read = buffer.len().min(self.buffer.len());

        // Read into our f32 buffer
        let (n_samples, _flags, _time_ns) = self
            .stream
            .read(&mut self.buffer[..samples_to_read], timeout_us)
            .map_err(|e| SdrError::HardwareError(e.to_string()))?;

        // Convert to IQSample
        for i in 0..n_samples {
            buffer[i] = IQSample::new(self.buffer[i][0] as f64, self.buffer[i][1] as f64);
        }

        self.samples_processed += n_samples as u64;
        self.status.samples_processed = self.samples_processed;

        let timestamp = Timestamp::at_sample(self.samples_processed, self.sdr_config.sample_rate);

        Ok((n_samples, timestamp))
    }

    fn write(
        &mut self,
        buffer: &[IQSample],
        timestamp: Option<Timestamp>,
        timeout: Duration,
    ) -> SdrResult<usize> {
        if !self.running {
            return Err(SdrError::NotStarted);
        }

        if self.direction != StreamDirection::Tx {
            return Err(SdrError::ConfigError("Cannot write to RX stream".to_string()));
        }

        let timeout_us = timeout.as_micros() as i64;
        let samples_to_write = buffer.len().min(self.buffer.len());

        // Convert from IQSample to f32 buffer
        for i in 0..samples_to_write {
            self.buffer[i] = [buffer[i].re as f32, buffer[i].im as f32];
        }

        let time_ns = timestamp.as_ref().map(|ts| ts.wall.as_nanos() as i64).unwrap_or(0);
        let flags = if timestamp.is_some() { 1 } else { 0 }; // HAS_TIME flag

        let n_written = self
            .stream
            .write(&self.buffer[..samples_to_write], flags, time_ns, timeout_us)
            .map_err(|e| SdrError::HardwareError(e.to_string()))?;

        self.samples_processed += n_written as u64;
        self.status.samples_processed = self.samples_processed;

        Ok(n_written)
    }

    fn status(&self) -> StreamStatus {
        self.status.clone()
    }

    fn timestamp(&self) -> Timestamp {
        Timestamp::at_sample(self.samples_processed, self.sdr_config.sample_rate)
    }

    fn available(&self) -> usize {
        0 // SoapySDR doesn't expose this directly
    }

    fn free_space(&self) -> usize {
        self.stream.mtu()
    }
}

// ============================================================================
// Stub implementation when soapysdr feature is not enabled
// ============================================================================

#[cfg(not(feature = "soapysdr"))]
pub struct SoapySdrDevice {
    info: DeviceInfo,
    config: SdrConfig,
    clock_source: ClockSource,
    time_source: TimeSource,
    antenna_rx: String,
    antenna_tx: String,
}

#[cfg(not(feature = "soapysdr"))]
impl SoapySdrDevice {
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
        self.info.driver.strip_prefix("soapysdr:").unwrap_or(&self.info.driver)
    }
}

#[cfg(not(feature = "soapysdr"))]
impl SdrDeviceExt for SoapySdrDevice {
    fn name(&self) -> &str {
        &self.info.label
    }

    fn capabilities(&self) -> DeviceCapabilities {
        DeviceCapabilities {
            can_rx: true,
            can_tx: true,
            full_duplex: true,
            rx_channels: 1,
            tx_channels: 1,
            min_frequency: 1_000_000.0,
            max_frequency: 6_000_000_000.0,
            max_sample_rate: 61_440_000.0,
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
}

#[cfg(not(feature = "soapysdr"))]
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
        vec!["RX".to_string(), "TX".to_string(), "LNAW".to_string(), "LNAL".to_string()]
    }

    fn frequency_range(&self) -> (u64, u64) {
        (1_000_000, 6_000_000_000)
    }

    fn sample_rate_range(&self) -> (f64, f64) {
        (100_000.0, 61_440_000.0)
    }

    fn gain_range(&self) -> (f64, f64) {
        (0.0, 70.0)
    }
}

#[cfg(not(feature = "soapysdr"))]
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
        Err(SdrError::ConfigError("SoapySDR stub: PPS not available".to_string()))
    }

    fn wait_for_pps(&mut self, _timeout: Duration) -> SdrResult<()> {
        Err(SdrError::ConfigError("SoapySDR stub: PPS not available".to_string()))
    }

    fn is_locked(&self) -> bool {
        matches!(self.clock_source, ClockSource::Internal)
    }

    fn hardware_clock(&self) -> Option<HardwareClock> {
        None
    }
}

/// Stub stream for when soapysdr feature is disabled.
#[cfg(not(feature = "soapysdr"))]
struct SoapySdrStream {
    direction: StreamDirection,
    running: bool,
    #[allow(dead_code)]
    config: SdrConfig,
    status: StreamStatus,
}

#[cfg(not(feature = "soapysdr"))]
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

#[cfg(not(feature = "soapysdr"))]
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
        // Number depends on hardware connected
        println!("Found {} SoapySDR devices", devices.len());
        for dev in &devices {
            println!("  - {} (serial: {})", dev.label, dev.serial);
        }
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

    #[cfg(feature = "soapysdr")]
    #[test]
    fn test_device_capabilities() {
        let driver = SoapySdrDriver::new();
        if !driver.is_available() {
            println!("Skipping test - libSoapySDR not available");
            return;
        }

        let devices = driver.discover();
        if devices.is_empty() {
            println!("Skipping test - no SoapySDR devices found");
            return;
        }

        let device = driver.create(&devices[0]).unwrap();
        let caps = device.capabilities();

        println!("Device capabilities:");
        println!("  RX: {}, TX: {}", caps.can_rx, caps.can_tx);
        println!("  Freq: {} - {} MHz", caps.min_frequency / 1e6, caps.max_frequency / 1e6);
        println!("  Max rate: {} MS/s", caps.max_sample_rate / 1e6);
    }
}
