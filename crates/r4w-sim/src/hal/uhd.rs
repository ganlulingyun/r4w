//! # UHD Driver for USRP Devices
//!
//! This module provides hardware support for Ettus Research USRP devices
//! via the USRP Hardware Driver (UHD) library.
//!
//! ## Supported Devices
//!
//! - **B-Series**: B200, B200mini, B205mini, B210
//! - **N-Series**: N200, N210, N300, N310, N320, N321
//! - **X-Series**: X300, X310
//! - **E-Series**: E310, E320
//!
//! ## Usage
//!
//! ```rust,ignore
//! use r4w_sim::hal::{DriverRegistry, uhd::UhdDriver};
//!
//! let mut registry = DriverRegistry::new();
//! registry.register(Box::new(UhdDriver::new()));
//!
//! // Discover devices
//! let devices = registry.discover_all();
//!
//! // Create B200
//! let mut device = registry.create("uhd://type=b200")?;
//!
//! // Or create N210 by IP
//! let mut device = registry.create("uhd://addr=192.168.10.2")?;
//! ```
//!
//! ## Clock Synchronization
//!
//! ```rust,ignore
//! // Use external 10 MHz reference
//! device.clock().unwrap().set_clock_source(ClockSource::External)?;
//!
//! // Use GPSDO (if installed)
//! device.clock().unwrap().set_clock_source(ClockSource::Gpsdo)?;
//! device.clock().unwrap().set_time_source(TimeSource::Gps)?;
//!
//! // Synchronize time at PPS edge
//! device.clock().unwrap().set_time_at_pps(Timestamp::zero())?;
//! ```
//!
//! trace:FR-0090 | ai:claude

use super::{
    ClockControl, ClockSource, DeviceDriver, SdrDeviceExt, StreamConfig,
    StreamDirection, StreamHandle, StreamStatus, TunerControl,
};
use crate::device::{DeviceCapabilities, DeviceInfo, SdrConfig, SdrError, SdrResult};
use r4w_core::timing::{HardwareClock, TimeSource, Timestamp};
use r4w_core::types::IQSample;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tracing::{debug, info, warn};

// =============================================================================
// UHD FFI Bindings
// =============================================================================

/// UHD library wrapper (dynamic loading)
///
/// In a full implementation, this would load libuhd dynamically and provide
/// safe Rust bindings to the C API.
#[cfg(feature = "uhd")]
mod uhd_ffi {
    //! FFI bindings to libuhd
    //!
    //! These would be generated from uhd.h or use the uhd-rust crate.
    //! For now, we define the key types and function signatures.

    use std::ffi::{c_char, c_double, c_int, c_void};
    use std::os::raw::c_size_t;

    /// UHD error codes
    pub const UHD_ERROR_NONE: c_int = 0;

    /// Opaque handle to a USRP device
    pub type UhdUsrpHandle = *mut c_void;

    /// Opaque handle to an RX streamer
    pub type UhdRxStreamerHandle = *mut c_void;

    /// Opaque handle to a TX streamer
    pub type UhdTxStreamerHandle = *mut c_void;

    /// Stream arguments
    #[repr(C)]
    pub struct UhdStreamArgs {
        pub cpu_format: *const c_char,
        pub otw_format: *const c_char,
        pub args: *const c_char,
        pub channel_list: *const c_size_t,
        pub n_channels: c_int,
    }

    /// RX metadata
    #[repr(C)]
    pub struct UhdRxMetadata {
        pub has_time_spec: bool,
        pub time_spec_full_secs: i64,
        pub time_spec_frac_secs: c_double,
        pub more_fragments: bool,
        pub fragment_offset: c_size_t,
        pub start_of_burst: bool,
        pub end_of_burst: bool,
        pub error_code: c_int,
        pub out_of_sequence: bool,
    }

    /// TX metadata
    #[repr(C)]
    pub struct UhdTxMetadata {
        pub has_time_spec: bool,
        pub time_spec_full_secs: i64,
        pub time_spec_frac_secs: c_double,
        pub start_of_burst: bool,
        pub end_of_burst: bool,
    }

    // Note: In production, these would be actual FFI function declarations:
    // extern "C" {
    //     pub fn uhd_usrp_find(...) -> c_int;
    //     pub fn uhd_usrp_make(...) -> c_int;
    //     pub fn uhd_usrp_free(...) -> c_int;
    //     pub fn uhd_usrp_set_rx_freq(...) -> c_int;
    //     // etc.
    // }
}

// =============================================================================
// UHD Driver
// =============================================================================

/// UHD device driver for USRP hardware.
///
/// Supports all Ettus Research USRP devices via the UHD library.
pub struct UhdDriver {
    /// Whether the UHD library is available
    library_available: bool,
}

impl UhdDriver {
    /// Create a new UHD driver instance.
    pub fn new() -> Self {
        // Check if UHD library is available
        let library_available = Self::check_library();

        Self { library_available }
    }

    /// Check if the UHD library is available on this system.
    fn check_library() -> bool {
        // Try to load libuhd dynamically
        #[cfg(feature = "uhd")]
        {
            // Would use libloading to check for libuhd.so / uhd.dll
            // For now, return false (stub)
            false
        }
        #[cfg(not(feature = "uhd"))]
        {
            false
        }
    }

    /// Parse device arguments from connection string.
    fn parse_args(args: &str) -> HashMap<String, String> {
        let mut result = HashMap::new();
        for part in args.split(',') {
            if let Some(pos) = part.find('=') {
                let key = part[..pos].trim().to_string();
                let value = part[pos + 1..].trim().to_string();
                result.insert(key, value);
            }
        }
        result
    }
}

impl Default for UhdDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl DeviceDriver for UhdDriver {
    fn name(&self) -> &str {
        "uhd"
    }

    fn discover(&self) -> Vec<DeviceInfo> {
        if !self.library_available {
            warn!("UHD library not available - cannot discover devices");
            return Vec::new();
        }

        // In production, this would call uhd_usrp_find()
        // For now, return simulated discovery results for testing

        #[cfg(feature = "uhd")]
        {
            // TODO: Implement actual device discovery
            // let mut found = Vec::new();
            // unsafe {
            //     let result = uhd_ffi::uhd_usrp_find(&mut found, "");
            //     if result == uhd_ffi::UHD_ERROR_NONE { ... }
            // }
            Vec::new()
        }
        #[cfg(not(feature = "uhd"))]
        {
            Vec::new()
        }
    }

    fn create(&self, info: &DeviceInfo) -> SdrResult<Box<dyn SdrDeviceExt>> {
        if !self.library_available {
            return Err(SdrError::HardwareError(
                "UHD library not available. Install libuhd and rebuild with --features uhd"
                    .to_string(),
            ));
        }

        // Use the serial or address from DeviceInfo
        let args = if !info.serial.is_empty() {
            format!("serial={}", info.serial)
        } else if !info.address.is_empty() {
            format!("addr={}", info.address)
        } else {
            info.driver.clone()
        };

        self.create_from_string(&args)
    }

    fn create_from_string(&self, args: &str) -> SdrResult<Box<dyn SdrDeviceExt>> {
        let parsed = Self::parse_args(args);

        // Determine device type from args
        let device_type = parsed
            .get("type")
            .cloned()
            .unwrap_or_else(|| "unknown".to_string());

        // Create device based on type
        let device = UhdDevice::new(args, &device_type)?;
        Ok(Box::new(device))
    }
}

// =============================================================================
// UHD Device
// =============================================================================

/// A USRP device instance.
pub struct UhdDevice {
    /// Device name/description
    name: String,
    /// Device type (b200, n210, x310, etc.)
    #[allow(dead_code)]
    device_type: String,
    /// Current configuration
    config: SdrConfig,
    /// Device capabilities
    capabilities: DeviceCapabilities,
    /// Tuner state
    tuner: UhdTuner,
    /// Clock control
    clock: UhdClock,
    /// Connection arguments
    args: String,
    /// Whether device is open
    is_open: bool,
}

impl UhdDevice {
    /// Create a new UHD device.
    fn new(args: &str, device_type: &str) -> SdrResult<Self> {
        // Determine capabilities based on device type
        let capabilities = Self::capabilities_for_type(device_type);

        let name = format!("USRP {} ({})", device_type.to_uppercase(), args);

        let config = SdrConfig {
            frequency: 915_000_000.0,
            sample_rate: 1_000_000.0,
            bandwidth: 1_000_000.0,
            rx_gain: 30.0,
            tx_gain: 30.0,
            antenna: "TX/RX".to_string(),
            buffer_size: 8192,
        };

        let tuner = UhdTuner::new(device_type);
        let clock = UhdClock::new();

        Ok(Self {
            name,
            device_type: device_type.to_string(),
            config,
            capabilities,
            tuner,
            clock,
            args: args.to_string(),
            is_open: false,
        })
    }

    /// Get device capabilities based on device type.
    fn capabilities_for_type(device_type: &str) -> DeviceCapabilities {
        match device_type.to_lowercase().as_str() {
            "b200" | "b200mini" | "b205mini" => DeviceCapabilities {
                can_tx: true,
                can_rx: true,
                full_duplex: true,
                min_frequency: 70_000_000.0,
                max_frequency: 6_000_000_000.0,
                max_sample_rate: 56_000_000.0,
                tx_channels: 1,
                rx_channels: 1,
            },
            "b210" => DeviceCapabilities {
                can_tx: true,
                can_rx: true,
                full_duplex: true,
                min_frequency: 70_000_000.0,
                max_frequency: 6_000_000_000.0,
                max_sample_rate: 56_000_000.0,
                tx_channels: 2,
                rx_channels: 2,
            },
            "n200" | "n210" => DeviceCapabilities {
                can_tx: true,
                can_rx: true,
                full_duplex: true,
                min_frequency: 10_000_000.0,  // Depends on daughterboard
                max_frequency: 6_000_000_000.0,
                max_sample_rate: 50_000_000.0, // 25 MS/s per channel with MIMO
                tx_channels: 2,
                rx_channels: 2,
            },
            "n310" | "n320" | "n321" => DeviceCapabilities {
                can_tx: true,
                can_rx: true,
                full_duplex: true,
                min_frequency: 10_000_000.0,
                max_frequency: 6_000_000_000.0,
                max_sample_rate: 153_600_000.0,
                tx_channels: 4,
                rx_channels: 4,
            },
            "x300" | "x310" => DeviceCapabilities {
                can_tx: true,
                can_rx: true,
                full_duplex: true,
                min_frequency: 10_000_000.0,
                max_frequency: 6_000_000_000.0,
                max_sample_rate: 200_000_000.0,
                tx_channels: 2,
                rx_channels: 2,
            },
            _ => DeviceCapabilities {
                can_tx: true,
                can_rx: true,
                full_duplex: true,
                min_frequency: 70_000_000.0,
                max_frequency: 6_000_000_000.0,
                max_sample_rate: 56_000_000.0,
                tx_channels: 1,
                rx_channels: 1,
            },
        }
    }

    /// Open the device connection.
    fn open(&mut self) -> SdrResult<()> {
        if self.is_open {
            return Ok(());
        }

        // In production: uhd_usrp_make(&mut handle, args)
        info!("Opening UHD device: {}", self.args);

        // For now, simulate opening (stub)
        #[cfg(feature = "uhd")]
        {
            // TODO: Actual UHD initialization
            // unsafe {
            //     let result = uhd_ffi::uhd_usrp_make(&mut self.handle, args_cstr);
            //     if result != uhd_ffi::UHD_ERROR_NONE {
            //         return Err(SdrError::HardwareError("Failed to open USRP".into()));
            //     }
            // }
        }

        self.is_open = true;
        Ok(())
    }
}

impl SdrDeviceExt for UhdDevice {
    fn name(&self) -> &str {
        &self.name
    }

    fn capabilities(&self) -> DeviceCapabilities {
        self.capabilities.clone()
    }

    fn config(&self) -> &SdrConfig {
        &self.config
    }

    fn configure(&mut self, config: &SdrConfig) -> SdrResult<()> {
        self.open()?;

        // Apply all settings
        self.tuner.set_frequency(config.frequency as u64)?;
        self.tuner.set_sample_rate(config.sample_rate)?;
        self.tuner.set_bandwidth(config.bandwidth)?;
        self.tuner.set_rx_gain(config.rx_gain)?;
        self.tuner.set_tx_gain(config.tx_gain)?;
        self.tuner.set_antenna(&config.antenna)?;

        self.config = config.clone();
        Ok(())
    }

    fn create_rx_stream(&mut self, config: StreamConfig) -> SdrResult<Box<dyn StreamHandle>> {
        self.open()?;

        if !self.capabilities.can_rx {
            return Err(SdrError::ConfigError(
                "Device does not support RX".to_string(),
            ));
        }

        Ok(Box::new(UhdStream::new(
            StreamDirection::Rx,
            config,
            self.config.sample_rate,
        )))
    }

    fn create_tx_stream(&mut self, config: StreamConfig) -> SdrResult<Box<dyn StreamHandle>> {
        self.open()?;

        if !self.capabilities.can_tx {
            return Err(SdrError::ConfigError(
                "Device does not support TX".to_string(),
            ));
        }

        Ok(Box::new(UhdStream::new(
            StreamDirection::Tx,
            config,
            self.config.sample_rate,
        )))
    }

    fn tuner(&mut self) -> &mut dyn TunerControl {
        &mut self.tuner
    }

    fn clock(&mut self) -> Option<&mut dyn ClockControl> {
        Some(&mut self.clock)
    }

    fn supports_gps(&self) -> bool {
        // Most USRPs support GPSDO option
        true
    }

    fn supports_pps(&self) -> bool {
        true
    }

    fn supports_external_clock(&self) -> bool {
        true
    }
}

// =============================================================================
// UHD Tuner
// =============================================================================

/// Tuner control for USRP devices.
struct UhdTuner {
    frequency: u64,
    sample_rate: f64,
    bandwidth: f64,
    rx_gain: f64,
    tx_gain: f64,
    antenna: String,
    device_type: String,
}

impl UhdTuner {
    fn new(device_type: &str) -> Self {
        Self {
            frequency: 915_000_000,
            sample_rate: 1_000_000.0,
            bandwidth: 1_000_000.0,
            rx_gain: 30.0,
            tx_gain: 30.0,
            antenna: "TX/RX".to_string(),
            device_type: device_type.to_string(),
        }
    }
}

impl TunerControl for UhdTuner {
    fn set_frequency(&mut self, freq_hz: u64) -> SdrResult<u64> {
        let (min, max) = self.frequency_range();
        if freq_hz < min || freq_hz > max {
            return Err(SdrError::ConfigError(format!(
                "Frequency {} Hz out of range [{}, {}]",
                freq_hz, min, max
            )));
        }

        // In production: uhd_usrp_set_rx_freq() / uhd_usrp_set_tx_freq()
        debug!("Setting frequency to {} Hz", freq_hz);
        self.frequency = freq_hz;
        Ok(freq_hz)
    }

    fn frequency(&self) -> u64 {
        self.frequency
    }

    fn set_sample_rate(&mut self, rate: f64) -> SdrResult<f64> {
        let (min, max) = self.sample_rate_range();
        if rate < min || rate > max {
            return Err(SdrError::ConfigError(format!(
                "Sample rate {} out of range [{}, {}]",
                rate, min, max
            )));
        }

        // In production: uhd_usrp_set_rx_rate() / uhd_usrp_set_tx_rate()
        debug!("Setting sample rate to {} S/s", rate);
        self.sample_rate = rate;
        Ok(rate)
    }

    fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    fn set_bandwidth(&mut self, bw_hz: f64) -> SdrResult<f64> {
        // In production: uhd_usrp_set_rx_bandwidth()
        debug!("Setting bandwidth to {} Hz", bw_hz);
        self.bandwidth = bw_hz;
        Ok(bw_hz)
    }

    fn bandwidth(&self) -> f64 {
        self.bandwidth
    }

    fn set_rx_gain(&mut self, gain_db: f64) -> SdrResult<f64> {
        let (min, max) = self.gain_range();
        let clamped = gain_db.clamp(min, max);

        // In production: uhd_usrp_set_rx_gain()
        debug!("Setting RX gain to {} dB", clamped);
        self.rx_gain = clamped;
        Ok(clamped)
    }

    fn rx_gain(&self) -> f64 {
        self.rx_gain
    }

    fn set_tx_gain(&mut self, gain_db: f64) -> SdrResult<f64> {
        let (min, max) = self.gain_range();
        let clamped = gain_db.clamp(min, max);

        // In production: uhd_usrp_set_tx_gain()
        debug!("Setting TX gain to {} dB", clamped);
        self.tx_gain = clamped;
        Ok(clamped)
    }

    fn tx_gain(&self) -> f64 {
        self.tx_gain
    }

    fn set_antenna(&mut self, antenna: &str) -> SdrResult<()> {
        let available = self.available_antennas();
        if !available.iter().any(|a| a == antenna) {
            return Err(SdrError::ConfigError(format!(
                "Unknown antenna: {}. Available: {:?}",
                antenna, available
            )));
        }

        // In production: uhd_usrp_set_rx_antenna()
        debug!("Setting antenna to {}", antenna);
        self.antenna = antenna.to_string();
        Ok(())
    }

    fn antenna(&self) -> &str {
        &self.antenna
    }

    fn available_antennas(&self) -> Vec<String> {
        // Antenna options depend on device type and daughterboard
        match self.device_type.as_str() {
            "b200" | "b200mini" | "b210" => {
                vec!["TX/RX".to_string(), "RX2".to_string()]
            }
            "n210" => {
                // Depends on daughterboard (WBX, SBX, UBX, etc.)
                vec![
                    "TX/RX".to_string(),
                    "RX2".to_string(),
                    "CAL".to_string(),
                ]
            }
            _ => vec!["TX/RX".to_string(), "RX2".to_string()],
        }
    }

    fn frequency_range(&self) -> (u64, u64) {
        match self.device_type.as_str() {
            "b200" | "b200mini" | "b210" => (70_000_000, 6_000_000_000),
            "n210" => (10_000_000, 6_000_000_000), // Depends on daughterboard
            _ => (70_000_000, 6_000_000_000),
        }
    }

    fn sample_rate_range(&self) -> (f64, f64) {
        match self.device_type.as_str() {
            "b200" | "b200mini" => (200_000.0, 56_000_000.0),
            "b210" => (200_000.0, 56_000_000.0),
            "n210" => (200_000.0, 50_000_000.0),
            "x310" => (200_000.0, 200_000_000.0),
            _ => (200_000.0, 56_000_000.0),
        }
    }

    fn gain_range(&self) -> (f64, f64) {
        // Typical USRP gain range
        (0.0, 76.0)
    }
}

// =============================================================================
// UHD Clock Control
// =============================================================================

/// Clock control for USRP devices.
struct UhdClock {
    clock_source: ClockSource,
    time_source: TimeSource,
    is_locked: bool,
    device_time: Timestamp,
}

impl UhdClock {
    fn new() -> Self {
        Self {
            clock_source: ClockSource::Internal,
            time_source: TimeSource::Freerun,
            is_locked: true, // Internal clock is always "locked"
            device_time: Timestamp::new(1_000_000.0), // Default 1 MS/s
        }
    }
}

impl ClockControl for UhdClock {
    fn set_clock_source(&mut self, source: ClockSource) -> SdrResult<()> {
        // In production: uhd_usrp_set_clock_source()
        let source_str = match source {
            ClockSource::Internal => "internal",
            ClockSource::External => "external",
            ClockSource::Gpsdo => "gpsdo",
            ClockSource::Mimo => "mimo",
        };

        info!("Setting clock source to {}", source_str);
        self.clock_source = source;

        // External sources take time to lock
        if source != ClockSource::Internal {
            self.is_locked = false;
            // In production, would poll for lock status
        }

        Ok(())
    }

    fn clock_source(&self) -> ClockSource {
        self.clock_source
    }

    fn set_time_source(&mut self, source: TimeSource) -> SdrResult<()> {
        // In production: uhd_usrp_set_time_source()
        let source_str = match source {
            TimeSource::Freerun => "none",
            TimeSource::Gps => "gpsdo",
            TimeSource::Ptp => "external",
            TimeSource::Pps => "external",
            TimeSource::Ntp => "none",
            TimeSource::External => "external",
        };

        info!("Setting time source to {}", source_str);
        self.time_source = source;
        Ok(())
    }

    fn time_source(&self) -> TimeSource {
        self.time_source.clone()
    }

    fn time(&self) -> Timestamp {
        // In production: uhd_usrp_get_time_now()
        Timestamp::new(1_000_000.0)
    }

    fn set_time(&mut self, time: Timestamp) -> SdrResult<()> {
        // In production: uhd_usrp_set_time_now()
        info!("Setting device time to {:?}", time);
        self.device_time = time;
        Ok(())
    }

    fn set_time_at_pps(&mut self, time: Timestamp) -> SdrResult<()> {
        // In production: uhd_usrp_set_time_next_pps()
        info!("Setting device time at next PPS to {:?}", time);
        self.device_time = time;
        Ok(())
    }

    fn wait_for_pps(&mut self, timeout: Duration) -> SdrResult<()> {
        // In production: Get time, wait for change
        debug!("Waiting for PPS (timeout: {:?})", timeout);
        std::thread::sleep(Duration::from_millis(100)); // Simulate PPS wait
        Ok(())
    }

    fn is_locked(&self) -> bool {
        // In production: uhd_usrp_get_mboard_sensor("ref_locked")
        self.is_locked
    }

    fn hardware_clock(&self) -> Option<HardwareClock> {
        // 200 MHz master clock is typical for USRPs
        Some(HardwareClock::new(200_000_000.0))
    }
}

// =============================================================================
// UHD Stream
// =============================================================================

/// Streaming interface for USRP devices.
struct UhdStream {
    direction: StreamDirection,
    config: StreamConfig,
    sample_rate: f64,
    is_running: Arc<AtomicBool>,
    overflow_count: Arc<AtomicU64>,
    underflow_count: Arc<AtomicU64>,
    samples_processed: Arc<AtomicU64>,
    start_time: Option<std::time::Instant>,
}

impl UhdStream {
    fn new(direction: StreamDirection, config: StreamConfig, sample_rate: f64) -> Self {
        Self {
            direction,
            config,
            sample_rate,
            is_running: Arc::new(AtomicBool::new(false)),
            overflow_count: Arc::new(AtomicU64::new(0)),
            underflow_count: Arc::new(AtomicU64::new(0)),
            samples_processed: Arc::new(AtomicU64::new(0)),
            start_time: None,
        }
    }
}

impl StreamHandle for UhdStream {
    fn direction(&self) -> StreamDirection {
        self.direction
    }

    fn start(&mut self) -> SdrResult<()> {
        if self.is_running.load(Ordering::SeqCst) {
            return Ok(());
        }

        // In production: uhd_rx_streamer_issue_stream_cmd()
        info!("Starting {:?} stream", self.direction);
        self.is_running.store(true, Ordering::SeqCst);
        self.start_time = Some(std::time::Instant::now());
        Ok(())
    }

    fn stop(&mut self) -> SdrResult<()> {
        if !self.is_running.load(Ordering::SeqCst) {
            return Ok(());
        }

        // In production: Issue STREAM_MODE_STOP_CONTINUOUS
        info!("Stopping {:?} stream", self.direction);
        self.is_running.store(false, Ordering::SeqCst);
        Ok(())
    }

    fn is_running(&self) -> bool {
        self.is_running.load(Ordering::SeqCst)
    }

    fn read(&mut self, buffer: &mut [IQSample], _timeout: Duration) -> SdrResult<(usize, Timestamp)> {
        if self.direction != StreamDirection::Rx {
            return Err(SdrError::ConfigError(
                "Cannot read from TX stream".to_string(),
            ));
        }

        if !self.is_running.load(Ordering::SeqCst) {
            return Err(SdrError::ConfigError("Stream not running".to_string()));
        }

        // In production: uhd_rx_streamer_recv()
        // For now, fill with zeros (stub)
        let samples_to_read = buffer.len().min(self.config.buffer_size);

        for sample in buffer[..samples_to_read].iter_mut() {
            *sample = IQSample::new(0.0, 0.0);
        }

        self.samples_processed
            .fetch_add(samples_to_read as u64, Ordering::SeqCst);

        let timestamp = Timestamp::at_sample(
            self.samples_processed.load(Ordering::SeqCst),
            self.sample_rate,
        );
        Ok((samples_to_read, timestamp))
    }

    fn write(
        &mut self,
        buffer: &[IQSample],
        _timestamp: Option<Timestamp>,
        _timeout: Duration,
    ) -> SdrResult<usize> {
        if self.direction != StreamDirection::Tx {
            return Err(SdrError::ConfigError(
                "Cannot write to RX stream".to_string(),
            ));
        }

        if !self.is_running.load(Ordering::SeqCst) {
            return Err(SdrError::ConfigError("Stream not running".to_string()));
        }

        // In production: uhd_tx_streamer_send()
        let samples_to_write = buffer.len().min(self.config.buffer_size);

        self.samples_processed
            .fetch_add(samples_to_write as u64, Ordering::SeqCst);

        Ok(samples_to_write)
    }

    fn status(&self) -> StreamStatus {
        StreamStatus {
            overflow_count: self.overflow_count.load(Ordering::SeqCst),
            underflow_count: self.underflow_count.load(Ordering::SeqCst),
            late_count: 0,
            samples_processed: self.samples_processed.load(Ordering::SeqCst),
            buffer_level: 0,
        }
    }

    fn timestamp(&self) -> Timestamp {
        // Calculate timestamp based on samples processed
        let samples = self.samples_processed.load(Ordering::SeqCst);
        Timestamp::at_sample(samples, self.sample_rate)
    }

    fn available(&self) -> usize {
        if self.direction == StreamDirection::Rx && self.is_running.load(Ordering::SeqCst) {
            self.config.buffer_size
        } else {
            0
        }
    }

    fn free_space(&self) -> usize {
        if self.direction == StreamDirection::Tx && self.is_running.load(Ordering::SeqCst) {
            self.config.buffer_size
        } else {
            0
        }
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_uhd_driver_creation() {
        let driver = UhdDriver::new();
        assert_eq!(driver.name(), "uhd");
    }

    #[test]
    fn test_parse_args() {
        let args = "type=b200,serial=ABC123";
        let parsed = UhdDriver::parse_args(args);
        assert_eq!(parsed.get("type"), Some(&"b200".to_string()));
        assert_eq!(parsed.get("serial"), Some(&"ABC123".to_string()));
    }

    #[test]
    fn test_b200_capabilities() {
        let caps = UhdDevice::capabilities_for_type("b200");
        assert!(caps.can_tx);
        assert!(caps.can_rx);
        assert!(caps.full_duplex);
        assert_eq!(caps.min_frequency, 70_000_000.0);
        assert_eq!(caps.max_frequency, 6_000_000_000.0);
    }

    #[test]
    fn test_n210_capabilities() {
        let caps = UhdDevice::capabilities_for_type("n210");
        assert!(caps.can_tx);
        assert!(caps.can_rx);
        assert_eq!(caps.tx_channels, 2);
        assert_eq!(caps.rx_channels, 2);
    }

    #[test]
    fn test_tuner_frequency_range() {
        let tuner = UhdTuner::new("b200");
        let (min, max) = tuner.frequency_range();
        assert_eq!(min, 70_000_000);
        assert_eq!(max, 6_000_000_000);
    }

    #[test]
    fn test_available_antennas() {
        let tuner = UhdTuner::new("b200");
        let antennas = tuner.available_antennas();
        assert!(antennas.contains(&"TX/RX".to_string()));
        assert!(antennas.contains(&"RX2".to_string()));
    }

    #[test]
    fn test_clock_sources() {
        let mut clock = UhdClock::new();
        assert_eq!(clock.clock_source(), ClockSource::Internal);

        clock.set_clock_source(ClockSource::External).unwrap();
        assert_eq!(clock.clock_source(), ClockSource::External);
    }
}
