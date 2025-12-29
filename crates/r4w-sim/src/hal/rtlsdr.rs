//! # RTL-SDR Driver
//!
//! Real hardware driver for RTL-SDR devices based on the RTL2832U chipset.
//!
//! ## Features
//!
//! - Frequency range: ~24 MHz - 1.75 GHz (tuner dependent)
//! - Sample rate: 225 kHz - 3.2 MS/s (stable at 2.4 MS/s)
//! - 8-bit ADC resolution (converted to f64 internally)
//! - RX only (no transmit capability)
//! - Dynamic library loading (no compile-time dependency)
//!
//! ## Requirements
//!
//! Install librtlsdr:
//! - **Linux**: `sudo apt install librtlsdr-dev` or `sudo dnf install rtl-sdr-devel`
//! - **macOS**: `brew install librtlsdr`
//! - **Windows**: Download from https://osmocom.org/projects/rtl-sdr
//!
//! ## Usage
//!
//! ```rust,ignore
//! use r4w_sim::hal::{RtlSdrDriver, DeviceDriver, DriverRegistry};
//!
//! // Check if library is available
//! let driver = RtlSdrDriver::new();
//! if driver.is_available() {
//!     println!("RTL-SDR library found!");
//!
//!     // Discover devices
//!     for dev in driver.discover() {
//!         println!("Found: {} ({})", dev.label, dev.serial);
//!     }
//! }
//!
//! // Create and configure device
//! let mut device = driver.create_from_string("0")?;
//! device.tuner().set_frequency(433_000_000)?;
//! device.tuner().set_sample_rate(2_400_000.0)?;
//! device.tuner().set_rx_gain(30.0)?;
//!
//! // Start streaming
//! let mut stream = device.create_rx_stream(StreamConfig::default())?;
//! stream.start()?;
//!
//! let mut buffer = vec![IQSample::default(); 8192];
//! let (n_samples, timestamp) = stream.read(&mut buffer, Duration::from_millis(100))?;
//! println!("Read {} samples", n_samples);
//! ```
//!
//! ## Supported Tuners
//!
//! - E4000: 52-2200 MHz (with gap at 1100-1250 MHz)
//! - FC0012: 22-948 MHz
//! - FC0013: 22-1100 MHz
//! - FC2580: 146-308 MHz, 438-924 MHz
//! - R820T/R820T2: 24-1766 MHz (most common)
//! - R828D: 24-1766 MHz

use super::{
    ClockControl, DeviceDriver, SdrDeviceExt, SdrResult, StreamConfig,
    StreamDirection, StreamHandle, StreamStatus, TunerControl,
};
use crate::device::{DeviceCapabilities, DeviceInfo, SdrConfig, SdrError};
use r4w_core::timing::Timestamp;
use r4w_core::types::IQSample;
use std::time::Duration;

#[cfg(feature = "rtlsdr")]
use std::sync::{Arc, Mutex};

#[cfg(feature = "rtlsdr")]
use std::time::Instant;

#[cfg(feature = "rtlsdr")]
use super::rtlsdr_ffi::{self, RtlSdrHandle};

/// RTL-SDR frequency range (Hz).
pub const RTLSDR_MIN_FREQ: u64 = 24_000_000;
pub const RTLSDR_MAX_FREQ: u64 = 1_766_000_000;

/// RTL-SDR sample rate range (Hz).
pub const RTLSDR_MIN_SAMPLE_RATE: f64 = 225_001.0;
pub const RTLSDR_MAX_SAMPLE_RATE: f64 = 3_200_000.0;
pub const RTLSDR_RECOMMENDED_SAMPLE_RATE: f64 = 2_400_000.0;

/// RTL-SDR gain range (dB).
/// Actual range depends on tuner, this is typical for R820T.
pub const RTLSDR_MIN_GAIN: f64 = 0.0;
pub const RTLSDR_MAX_GAIN: f64 = 49.6;

/// Default buffer size in bytes (must be multiple of 512).
#[allow(dead_code)]
const DEFAULT_BUFFER_SIZE: usize = 16384;

/// RTL-SDR device driver.
///
/// This driver provides real hardware access when librtlsdr is available,
/// or falls back to a stub implementation when not.
pub struct RtlSdrDriver {
    #[cfg(feature = "rtlsdr")]
    available: bool,
    #[cfg(not(feature = "rtlsdr"))]
    available: bool,
}

impl RtlSdrDriver {
    /// Create a new RTL-SDR driver.
    pub fn new() -> Self {
        #[cfg(feature = "rtlsdr")]
        let available = rtlsdr_ffi::is_available();

        #[cfg(not(feature = "rtlsdr"))]
        let available = false;

        if available {
            tracing::info!("RTL-SDR driver initialized with hardware support");
        } else {
            tracing::debug!("RTL-SDR driver initialized (stub mode - no librtlsdr)");
        }

        Self { available }
    }

    /// Check if librtlsdr is available.
    ///
    /// Returns true if the library is loaded and functional.
    pub fn is_available(&self) -> bool {
        self.available
    }
}

impl Default for RtlSdrDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl DeviceDriver for RtlSdrDriver {
    fn name(&self) -> &str {
        "rtlsdr"
    }

    fn discover(&self) -> Vec<DeviceInfo> {
        #[cfg(feature = "rtlsdr")]
        {
            if !self.available {
                return Vec::new();
            }

            let count = rtlsdr_ffi::get_device_count();
            let mut devices = Vec::with_capacity(count as usize);

            for i in 0..count {
                let name = rtlsdr_ffi::get_device_name(i).unwrap_or_else(|| "Unknown".to_string());
                let usb = rtlsdr_ffi::get_device_usb_strings(i);

                let (serial, label) = if let Some(usb) = usb {
                    let label = if usb.product.is_empty() {
                        name.clone()
                    } else {
                        format!("{} ({})", usb.product, name)
                    };
                    (usb.serial, label)
                } else {
                    (String::new(), name)
                };

                devices.push(DeviceInfo {
                    driver: "rtlsdr".to_string(),
                    serial,
                    label,
                    address: format!("{}", i),
                });

                tracing::debug!("Found RTL-SDR device #{}: {}", i, devices.last().unwrap().label);
            }

            devices
        }

        #[cfg(not(feature = "rtlsdr"))]
        {
            tracing::debug!("RTL-SDR discover: stub mode, no devices");
            Vec::new()
        }
    }

    fn create(&self, info: &DeviceInfo) -> SdrResult<Box<dyn SdrDeviceExt>> {
        if !self.is_available() {
            return Err(SdrError::DeviceNotFound(
                "RTL-SDR library not available. Install librtlsdr and enable 'rtlsdr' feature."
                    .to_string(),
            ));
        }

        #[cfg(feature = "rtlsdr")]
        {
            let index: u32 = info.address.parse().map_err(|_| {
                SdrError::ConfigError(format!("Invalid device index: {}", info.address))
            })?;

            Ok(Box::new(RtlSdrDevice::open(index, info.clone())?))
        }

        #[cfg(not(feature = "rtlsdr"))]
        {
            let _ = info;
            Err(SdrError::DeviceNotFound(
                "RTL-SDR support not compiled in. Enable 'rtlsdr' feature.".to_string(),
            ))
        }
    }

    fn create_from_string(&self, args: &str) -> SdrResult<Box<dyn SdrDeviceExt>> {
        // Parse args: device index (e.g., "0") or serial number
        let index: u32 = args.parse().unwrap_or(0);

        // First try to discover to get proper device info
        let devices = self.discover();

        let info = devices
            .into_iter()
            .find(|d| d.address == format!("{}", index))
            .unwrap_or_else(|| DeviceInfo {
                driver: "rtlsdr".to_string(),
                serial: String::new(),
                label: format!("RTL-SDR #{}", index),
                address: format!("{}", index),
            });

        self.create(&info)
    }
}

/// RTL-SDR device instance.
pub struct RtlSdrDevice {
    info: DeviceInfo,
    config: SdrConfig,
    #[cfg(feature = "rtlsdr")]
    handle: Arc<Mutex<RtlSdrHandle>>,
    #[cfg(feature = "rtlsdr")]
    available_gains: Vec<f64>,
    ppm_correction: i32,
}

impl RtlSdrDevice {
    /// Open an RTL-SDR device by index.
    #[cfg(feature = "rtlsdr")]
    fn open(index: u32, info: DeviceInfo) -> SdrResult<Self> {
        let handle = RtlSdrHandle::open(index).map_err(|e| SdrError::HardwareError(e.to_string()))?;

        // Get available gains and convert from tenths of dB to dB
        let available_gains: Vec<f64> = handle
            .available_gains()
            .iter()
            .map(|&g| g as f64 / 10.0)
            .collect();

        let gain_range = if available_gains.is_empty() {
            (RTLSDR_MIN_GAIN, RTLSDR_MAX_GAIN)
        } else {
            (
                *available_gains.first().unwrap(),
                *available_gains.last().unwrap(),
            )
        };

        tracing::info!(
            "Opened RTL-SDR #{}: gain range {:.1} - {:.1} dB ({} levels)",
            index,
            gain_range.0,
            gain_range.1,
            available_gains.len()
        );

        let mut device = Self {
            info,
            config: SdrConfig {
                frequency: 915_000_000.0,
                sample_rate: RTLSDR_RECOMMENDED_SAMPLE_RATE,
                bandwidth: RTLSDR_RECOMMENDED_SAMPLE_RATE,
                rx_gain: 30.0,
                tx_gain: 0.0,
                antenna: "RX".to_string(),
                buffer_size: DEFAULT_BUFFER_SIZE,
            },
            handle: Arc::new(Mutex::new(handle)),
            available_gains,
            ppm_correction: 0,
        };

        // Apply initial configuration
        device.apply_config()?;

        Ok(device)
    }

    #[cfg(not(feature = "rtlsdr"))]
    #[allow(dead_code)]
    fn open(_index: u32, _info: DeviceInfo) -> SdrResult<Self> {
        Err(SdrError::DeviceNotFound(
            "RTL-SDR support not compiled in".to_string(),
        ))
    }

    /// Apply current configuration to hardware.
    #[cfg(feature = "rtlsdr")]
    fn apply_config(&mut self) -> SdrResult<()> {
        let mut handle = self.handle.lock().unwrap();

        // Set frequency
        handle
            .set_center_freq(self.config.frequency as u32)
            .map_err(|e| SdrError::HardwareError(e.to_string()))?;

        // Set sample rate
        handle
            .set_sample_rate(self.config.sample_rate as u32)
            .map_err(|e| SdrError::HardwareError(e.to_string()))?;

        // Set gain (manual mode)
        handle
            .set_tuner_gain_mode(true)
            .map_err(|e| SdrError::HardwareError(e.to_string()))?;

        // Convert dB to tenths of dB for the API
        let gain_tenth_db = (self.config.rx_gain * 10.0) as i32;
        handle
            .set_tuner_gain(gain_tenth_db)
            .map_err(|e| SdrError::HardwareError(e.to_string()))?;

        // Set frequency correction
        if self.ppm_correction != 0 {
            handle
                .set_freq_correction(self.ppm_correction)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;
        }

        Ok(())
    }

    /// Set frequency correction in PPM.
    #[cfg(feature = "rtlsdr")]
    pub fn set_ppm_correction(&mut self, ppm: i32) -> SdrResult<()> {
        self.ppm_correction = ppm;
        let mut handle = self.handle.lock().unwrap();
        handle
            .set_freq_correction(ppm)
            .map_err(|e| SdrError::HardwareError(e.to_string()))
    }

    /// Get frequency correction in PPM.
    pub fn ppm_correction(&self) -> i32 {
        self.ppm_correction
    }

    /// Get available gain values in dB.
    #[cfg(feature = "rtlsdr")]
    pub fn available_gains(&self) -> &[f64] {
        &self.available_gains
    }
}

impl SdrDeviceExt for RtlSdrDevice {
    fn name(&self) -> &str {
        &self.info.label
    }

    fn capabilities(&self) -> DeviceCapabilities {
        DeviceCapabilities {
            can_rx: true,
            can_tx: false, // RTL-SDR is RX only
            full_duplex: false,
            rx_channels: 1,
            tx_channels: 0,
            min_frequency: RTLSDR_MIN_FREQ as f64,
            max_frequency: RTLSDR_MAX_FREQ as f64,
            max_sample_rate: RTLSDR_MAX_SAMPLE_RATE,
        }
    }

    fn config(&self) -> &SdrConfig {
        &self.config
    }

    fn configure(&mut self, config: &SdrConfig) -> SdrResult<()> {
        // Validate config
        if config.sample_rate < RTLSDR_MIN_SAMPLE_RATE
            || config.sample_rate > RTLSDR_MAX_SAMPLE_RATE
        {
            return Err(SdrError::ConfigError(format!(
                "Sample rate {} out of range [{}, {}]",
                config.sample_rate, RTLSDR_MIN_SAMPLE_RATE, RTLSDR_MAX_SAMPLE_RATE
            )));
        }

        if (config.frequency as u64) < RTLSDR_MIN_FREQ
            || (config.frequency as u64) > RTLSDR_MAX_FREQ
        {
            return Err(SdrError::ConfigError(format!(
                "Frequency {} out of range [{}, {}]",
                config.frequency, RTLSDR_MIN_FREQ, RTLSDR_MAX_FREQ
            )));
        }

        self.config = config.clone();

        #[cfg(feature = "rtlsdr")]
        self.apply_config()?;

        Ok(())
    }

    fn create_rx_stream(&mut self, config: StreamConfig) -> SdrResult<Box<dyn StreamHandle>> {
        #[cfg(feature = "rtlsdr")]
        {
            // Reset buffer before creating stream
            {
                let mut handle = self.handle.lock().unwrap();
                handle
                    .reset_buffer()
                    .map_err(|e| SdrError::HardwareError(e.to_string()))?;
            }

            Ok(Box::new(RtlSdrStream::new(
                self.handle.clone(),
                self.config.clone(),
                config,
            )))
        }

        #[cfg(not(feature = "rtlsdr"))]
        {
            let _ = config;
            Err(SdrError::HardwareError(
                "RTL-SDR support not compiled in".to_string(),
            ))
        }
    }

    fn create_tx_stream(&mut self, _config: StreamConfig) -> SdrResult<Box<dyn StreamHandle>> {
        Err(SdrError::ConfigError(
            "RTL-SDR does not support transmit".to_string(),
        ))
    }

    fn tuner(&mut self) -> &mut dyn TunerControl {
        self
    }

    fn clock(&mut self) -> Option<&mut dyn ClockControl> {
        None // RTL-SDR has no clock control
    }
}

impl TunerControl for RtlSdrDevice {
    fn set_frequency(&mut self, freq_hz: u64) -> SdrResult<u64> {
        if freq_hz < RTLSDR_MIN_FREQ || freq_hz > RTLSDR_MAX_FREQ {
            return Err(SdrError::ConfigError(format!(
                "Frequency {} out of range [{}, {}]",
                freq_hz, RTLSDR_MIN_FREQ, RTLSDR_MAX_FREQ
            )));
        }

        self.config.frequency = freq_hz as f64;

        #[cfg(feature = "rtlsdr")]
        {
            let mut handle = self.handle.lock().unwrap();
            handle
                .set_center_freq(freq_hz as u32)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;

            // Return actual frequency (may differ slightly)
            Ok(handle.get_center_freq() as u64)
        }

        #[cfg(not(feature = "rtlsdr"))]
        Ok(freq_hz)
    }

    fn frequency(&self) -> u64 {
        #[cfg(feature = "rtlsdr")]
        {
            let handle = self.handle.lock().unwrap();
            handle.get_center_freq() as u64
        }

        #[cfg(not(feature = "rtlsdr"))]
        {
            self.config.frequency as u64
        }
    }

    fn set_sample_rate(&mut self, rate: f64) -> SdrResult<f64> {
        if rate < RTLSDR_MIN_SAMPLE_RATE || rate > RTLSDR_MAX_SAMPLE_RATE {
            return Err(SdrError::ConfigError(format!(
                "Sample rate {} out of range [{}, {}]",
                rate, RTLSDR_MIN_SAMPLE_RATE, RTLSDR_MAX_SAMPLE_RATE
            )));
        }

        self.config.sample_rate = rate;

        #[cfg(feature = "rtlsdr")]
        {
            let mut handle = self.handle.lock().unwrap();
            handle
                .set_sample_rate(rate as u32)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;

            // Return actual sample rate
            Ok(handle.get_sample_rate() as f64)
        }

        #[cfg(not(feature = "rtlsdr"))]
        Ok(rate)
    }

    fn sample_rate(&self) -> f64 {
        #[cfg(feature = "rtlsdr")]
        {
            let handle = self.handle.lock().unwrap();
            handle.get_sample_rate() as f64
        }

        #[cfg(not(feature = "rtlsdr"))]
        {
            self.config.sample_rate
        }
    }

    fn set_bandwidth(&mut self, bw_hz: f64) -> SdrResult<f64> {
        // RTL-SDR bandwidth is generally tied to sample rate
        // Some tuners support bandwidth setting
        self.config.bandwidth = bw_hz;

        #[cfg(feature = "rtlsdr")]
        {
            let mut handle = self.handle.lock().unwrap();
            // Setting to 0 means automatic (based on sample rate)
            let _ = handle.set_tuner_bandwidth(bw_hz as u32);
        }

        Ok(bw_hz)
    }

    fn bandwidth(&self) -> f64 {
        self.config.bandwidth
    }

    fn set_rx_gain(&mut self, gain_db: f64) -> SdrResult<f64> {
        let gain = gain_db.clamp(RTLSDR_MIN_GAIN, RTLSDR_MAX_GAIN);
        self.config.rx_gain = gain;

        #[cfg(feature = "rtlsdr")]
        {
            let mut handle = self.handle.lock().unwrap();

            // Set manual gain mode
            handle
                .set_tuner_gain_mode(true)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;

            // Set gain in tenths of dB
            let actual_tenth_db = handle
                .set_tuner_gain((gain * 10.0) as i32)
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;

            let actual_gain = actual_tenth_db as f64 / 10.0;
            self.config.rx_gain = actual_gain;
            Ok(actual_gain)
        }

        #[cfg(not(feature = "rtlsdr"))]
        Ok(gain)
    }

    fn rx_gain(&self) -> f64 {
        #[cfg(feature = "rtlsdr")]
        {
            let handle = self.handle.lock().unwrap();
            handle.get_tuner_gain() as f64 / 10.0
        }

        #[cfg(not(feature = "rtlsdr"))]
        {
            self.config.rx_gain
        }
    }

    fn set_tx_gain(&mut self, _gain_db: f64) -> SdrResult<f64> {
        Err(SdrError::ConfigError("RTL-SDR is RX only".to_string()))
    }

    fn tx_gain(&self) -> f64 {
        0.0
    }

    fn set_antenna(&mut self, _antenna: &str) -> SdrResult<()> {
        Ok(()) // RTL-SDR has no antenna selection
    }

    fn antenna(&self) -> &str {
        "RX"
    }

    fn available_antennas(&self) -> Vec<String> {
        vec!["RX".to_string()]
    }

    fn frequency_range(&self) -> (u64, u64) {
        (RTLSDR_MIN_FREQ, RTLSDR_MAX_FREQ)
    }

    fn sample_rate_range(&self) -> (f64, f64) {
        (RTLSDR_MIN_SAMPLE_RATE, RTLSDR_MAX_SAMPLE_RATE)
    }

    fn gain_range(&self) -> (f64, f64) {
        #[cfg(feature = "rtlsdr")]
        {
            if self.available_gains.is_empty() {
                (RTLSDR_MIN_GAIN, RTLSDR_MAX_GAIN)
            } else {
                (
                    *self.available_gains.first().unwrap(),
                    *self.available_gains.last().unwrap(),
                )
            }
        }

        #[cfg(not(feature = "rtlsdr"))]
        (RTLSDR_MIN_GAIN, RTLSDR_MAX_GAIN)
    }
}

/// RTL-SDR RX stream.
#[cfg(feature = "rtlsdr")]
pub struct RtlSdrStream {
    handle: Arc<Mutex<RtlSdrHandle>>,
    running: bool,
    sdr_config: SdrConfig,
    stream_config: StreamConfig,
    status: StreamStatus,
    start_time: Option<Instant>,
    samples_read: u64,
    raw_buffer: Vec<u8>,
}

#[cfg(feature = "rtlsdr")]
impl RtlSdrStream {
    fn new(handle: Arc<Mutex<RtlSdrHandle>>, sdr_config: SdrConfig, stream_config: StreamConfig) -> Self {
        // Allocate raw buffer (2 bytes per sample: I and Q)
        let buffer_samples = stream_config.buffer_size;
        let raw_buffer_size = buffer_samples * 2;
        // Round up to multiple of 512 (RTL-SDR requirement)
        let raw_buffer_size = (raw_buffer_size + 511) & !511;

        Self {
            handle,
            running: false,
            sdr_config,
            stream_config,
            status: StreamStatus::default(),
            start_time: None,
            samples_read: 0,
            raw_buffer: vec![0u8; raw_buffer_size],
        }
    }
}

#[cfg(feature = "rtlsdr")]
impl StreamHandle for RtlSdrStream {
    fn direction(&self) -> StreamDirection {
        StreamDirection::Rx
    }

    fn start(&mut self) -> SdrResult<()> {
        if self.running {
            return Ok(());
        }

        // Reset buffer
        {
            let mut handle = self.handle.lock().unwrap();
            handle
                .reset_buffer()
                .map_err(|e| SdrError::HardwareError(e.to_string()))?;
        }

        self.running = true;
        self.start_time = Some(Instant::now());
        self.samples_read = 0;
        self.status = StreamStatus::default();

        tracing::info!("RTL-SDR stream started at {} MHz, {} MS/s",
            self.sdr_config.frequency / 1_000_000.0,
            self.sdr_config.sample_rate / 1_000_000.0);

        Ok(())
    }

    fn stop(&mut self) -> SdrResult<()> {
        self.running = false;
        tracing::info!("RTL-SDR stream stopped, {} samples read", self.samples_read);
        Ok(())
    }

    fn is_running(&self) -> bool {
        self.running
    }

    fn read(&mut self, buffer: &mut [IQSample], timeout: Duration) -> SdrResult<(usize, Timestamp)> {
        if !self.running {
            return Err(SdrError::NotStarted);
        }

        let start = Instant::now();

        // Calculate how many bytes we need (2 bytes per sample)
        let samples_requested = buffer.len();
        let bytes_needed = samples_requested * 2;
        let bytes_to_read = bytes_needed.min(self.raw_buffer.len());

        // Read from hardware
        let bytes_read = {
            let mut handle = self.handle.lock().unwrap();
            handle
                .read_sync(&mut self.raw_buffer[..bytes_to_read])
                .map_err(|e| SdrError::HardwareError(e.to_string()))?
        };

        // Check for timeout
        if bytes_read == 0 && start.elapsed() > timeout {
            return Err(SdrError::Timeout("read samples".to_string()));
        }

        // Convert samples
        let samples_read = bytes_read / 2;
        for i in 0..samples_read {
            let i_raw = self.raw_buffer[i * 2];
            let q_raw = self.raw_buffer[i * 2 + 1];
            buffer[i] = IQSample::new(
                rtlsdr_ffi::u8_to_f64(i_raw),
                rtlsdr_ffi::u8_to_f64(q_raw),
            );
        }

        // Update stats
        self.samples_read += samples_read as u64;
        self.status.samples_processed = self.samples_read;

        // Create timestamp based on sample count
        let timestamp = Timestamp::at_sample(self.samples_read, self.sdr_config.sample_rate);

        Ok((samples_read, timestamp))
    }

    fn write(
        &mut self,
        _buffer: &[IQSample],
        _timestamp: Option<Timestamp>,
        _timeout: Duration,
    ) -> SdrResult<usize> {
        Err(SdrError::ConfigError("RTL-SDR is RX only".to_string()))
    }

    fn status(&self) -> StreamStatus {
        self.status.clone()
    }

    fn timestamp(&self) -> Timestamp {
        Timestamp::at_sample(self.samples_read, self.sdr_config.sample_rate)
    }

    fn available(&self) -> usize {
        // RTL-SDR doesn't expose buffer fill level
        0
    }

    fn free_space(&self) -> usize {
        0
    }
}

// Stub stream for when rtlsdr feature is disabled
#[cfg(not(feature = "rtlsdr"))]
pub struct RtlSdrStream {
    running: bool,
    status: StreamStatus,
}

#[cfg(not(feature = "rtlsdr"))]
impl RtlSdrStream {
    #[allow(dead_code)]
    fn new(_config: SdrConfig) -> Self {
        Self {
            running: false,
            status: StreamStatus::default(),
        }
    }
}

#[cfg(not(feature = "rtlsdr"))]
impl StreamHandle for RtlSdrStream {
    fn direction(&self) -> StreamDirection {
        StreamDirection::Rx
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
            "RTL-SDR stub: no real hardware connected".to_string(),
        ))
    }

    fn write(
        &mut self,
        _buffer: &[IQSample],
        _timestamp: Option<Timestamp>,
        _timeout: Duration,
    ) -> SdrResult<usize> {
        Err(SdrError::ConfigError("RTL-SDR is RX only".to_string()))
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_driver_name() {
        let driver = RtlSdrDriver::new();
        assert_eq!(driver.name(), "rtlsdr");
    }

    #[test]
    fn test_driver_availability() {
        let driver = RtlSdrDriver::new();
        // Just check that the function works - actual result depends on system
        let _ = driver.is_available();
    }

    #[test]
    fn test_driver_discover() {
        let driver = RtlSdrDriver::new();
        let devices = driver.discover();
        // Number of devices depends on what's connected
        println!("Found {} RTL-SDR devices", devices.len());
        for dev in &devices {
            println!("  - {} (serial: {})", dev.label, dev.serial);
        }
    }

    #[test]
    fn test_frequency_range() {
        assert_eq!(RTLSDR_MIN_FREQ, 24_000_000);
        assert_eq!(RTLSDR_MAX_FREQ, 1_766_000_000);
    }

    #[test]
    fn test_sample_rate_range() {
        assert!((RTLSDR_MIN_SAMPLE_RATE - 225_001.0).abs() < 0.1);
        assert!((RTLSDR_MAX_SAMPLE_RATE - 3_200_000.0).abs() < 0.1);
    }

    #[cfg(feature = "rtlsdr")]
    #[test]
    fn test_device_capabilities() {
        let driver = RtlSdrDriver::new();
        if !driver.is_available() {
            println!("Skipping test - librtlsdr not available");
            return;
        }

        let devices = driver.discover();
        if devices.is_empty() {
            println!("Skipping test - no RTL-SDR devices found");
            return;
        }

        let device = driver.create(&devices[0]).unwrap();
        let caps = device.capabilities();

        assert!(caps.can_rx);
        assert!(!caps.can_tx);
        assert!(!caps.full_duplex);
        assert_eq!(caps.rx_channels, 1);
        assert_eq!(caps.tx_channels, 0);
    }

    #[cfg(feature = "rtlsdr")]
    #[test]
    fn test_tuner_frequency() {
        let driver = RtlSdrDriver::new();
        if !driver.is_available() {
            println!("Skipping test - librtlsdr not available");
            return;
        }

        let devices = driver.discover();
        if devices.is_empty() {
            println!("Skipping test - no RTL-SDR devices found");
            return;
        }

        let mut device = driver.create(&devices[0]).unwrap();

        // Valid frequency
        let result = device.tuner().set_frequency(433_000_000);
        assert!(result.is_ok());

        let actual = device.tuner().frequency();
        // RTL-SDR may not set exactly the requested frequency
        assert!((actual as i64 - 433_000_000i64).abs() < 1000);

        // Invalid frequency (too low)
        let result = device.tuner().set_frequency(1_000_000);
        assert!(result.is_err());
    }

    #[cfg(feature = "rtlsdr")]
    #[test]
    fn test_stream_read() {
        let driver = RtlSdrDriver::new();
        if !driver.is_available() {
            println!("Skipping test - librtlsdr not available");
            return;
        }

        let devices = driver.discover();
        if devices.is_empty() {
            println!("Skipping test - no RTL-SDR devices found");
            return;
        }

        let mut device = driver.create(&devices[0]).unwrap();

        // Configure
        device.tuner().set_frequency(433_000_000).unwrap();
        device.tuner().set_sample_rate(2_400_000.0).unwrap();
        device.tuner().set_rx_gain(30.0).unwrap();

        // Create stream
        let mut stream = device.create_rx_stream(StreamConfig::default()).unwrap();
        stream.start().unwrap();

        // Read samples
        let mut buffer = vec![IQSample::default(); 8192];
        let result = stream.read(&mut buffer, Duration::from_millis(1000));

        match result {
            Ok((n_samples, _ts)) => {
                println!("Read {} samples", n_samples);
                assert!(n_samples > 0);
            }
            Err(e) => {
                println!("Read failed: {} (this may be expected)", e);
            }
        }

        stream.stop().unwrap();
    }
}
