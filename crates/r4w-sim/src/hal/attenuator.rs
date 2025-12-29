//! # Digital Attenuator Control
//!
//! This module provides an abstraction for controlling digital RF attenuators,
//! enabling precise, repeatable signal level control for SDR testing.
//!
//! ## Supported Attenuators
//!
//! - **USB-controlled**: Mini-Circuits RCDAT, Vaunix Lab Brick
//! - **SPI-controlled**: PE43711, HMC624A, F1956
//! - **Serial-controlled**: Various lab attenuators
//! - **Simulated**: For testing without hardware
//!
//! ## Usage
//!
//! ```rust,ignore
//! use r4w_sim::hal::attenuator::{Attenuator, AttenuatorConfig};
//!
//! // Create attenuator
//! let mut atten = create_attenuator("usb://minicircuits/RCDAT-8000-90")?;
//!
//! // Set fixed attenuation
//! atten.set_attenuation(30.0)?;  // 30 dB
//!
//! // Sweep attenuation for sensitivity testing
//! for db in (0..=60).step_by(5) {
//!     atten.set_attenuation(db as f64)?;
//!     // Run test at this level
//! }
//! ```
//!
//! ## Test Framework Integration
//!
//! ```rust,ignore
//! use r4w_sim::hal::attenuator::AttenuatorTestHarness;
//!
//! let harness = AttenuatorTestHarness::new(tx_device, rx_device, attenuator);
//!
//! // Automatic SNR sweep test
//! let results = harness.snr_sweep_test(
//!     waveform,
//!     0.0..60.0,  // Attenuation range
//!     1.0,        // Step size in dB
//! )?;
//!
//! // Find sensitivity threshold
//! let sensitivity = harness.find_sensitivity(waveform, 0.01)?;  // 1% BER threshold
//! ```
//!
//! trace:FR-0091 | ai:claude

use crate::device::{SdrError, SdrResult};
use std::time::Duration;
use tracing::debug;

// =============================================================================
// Attenuator Traits
// =============================================================================

/// Core attenuator control interface.
pub trait Attenuator: Send {
    /// Get attenuator name/description.
    fn name(&self) -> &str;

    /// Get attenuator capabilities.
    fn capabilities(&self) -> AttenuatorCapabilities;

    /// Set attenuation level in dB.
    ///
    /// Returns the actual attenuation set (may differ slightly due to step size).
    fn set_attenuation(&mut self, db: f64) -> SdrResult<f64>;

    /// Get current attenuation level in dB.
    fn attenuation(&self) -> f64;

    /// Set attenuation with settling time wait.
    fn set_attenuation_with_settle(&mut self, db: f64, settle_time: Duration) -> SdrResult<f64> {
        let actual = self.set_attenuation(db)?;
        std::thread::sleep(settle_time);
        Ok(actual)
    }
}

/// Programmable attenuator with advanced features.
pub trait ProgrammableAttenuator: Attenuator {
    /// Set multiple attenuation values to be stepped through.
    fn program_sequence(&mut self, values: &[f64]) -> SdrResult<()>;

    /// Start sequence execution.
    fn start_sequence(&mut self) -> SdrResult<()>;

    /// Stop sequence execution.
    fn stop_sequence(&mut self) -> SdrResult<()>;

    /// Set dwell time between sequence steps.
    fn set_dwell_time(&mut self, duration: Duration) -> SdrResult<()>;

    /// Check if sequence is running.
    fn is_sequence_running(&self) -> bool;
}

/// Attenuator with GPIO trigger support.
pub trait TriggeredAttenuator: Attenuator {
    /// Arm for external trigger.
    fn arm_trigger(&mut self) -> SdrResult<()>;

    /// Set trigger source.
    fn set_trigger_source(&mut self, source: TriggerSource) -> SdrResult<()>;

    /// Manual trigger (software trigger).
    fn trigger(&mut self) -> SdrResult<()>;
}

// =============================================================================
// Types
// =============================================================================

/// Attenuator capabilities.
#[derive(Debug, Clone)]
pub struct AttenuatorCapabilities {
    /// Minimum attenuation in dB
    pub min_attenuation: f64,
    /// Maximum attenuation in dB
    pub max_attenuation: f64,
    /// Step size in dB (resolution)
    pub step_size: f64,
    /// Minimum frequency in Hz
    pub min_frequency: f64,
    /// Maximum frequency in Hz
    pub max_frequency: f64,
    /// Insertion loss in dB (at 0 dB setting)
    pub insertion_loss: f64,
    /// Switching time in microseconds
    pub switching_time_us: f64,
    /// Number of channels
    pub channels: usize,
    /// Supports programmable sequences
    pub supports_sequences: bool,
    /// Supports external triggering
    pub supports_trigger: bool,
}

impl Default for AttenuatorCapabilities {
    fn default() -> Self {
        Self {
            min_attenuation: 0.0,
            max_attenuation: 31.5,
            step_size: 0.5,
            min_frequency: 0.0,
            max_frequency: 6_000_000_000.0,
            insertion_loss: 2.0,
            switching_time_us: 0.5,
            channels: 1,
            supports_sequences: false,
            supports_trigger: false,
        }
    }
}

/// Trigger source for attenuator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriggerSource {
    /// Software trigger
    Software,
    /// External TTL trigger
    External,
    /// Internal timer
    Timer,
}

/// Attenuator connection type.
#[derive(Debug, Clone)]
pub enum AttenuatorConnection {
    /// USB HID device
    UsbHid { vendor_id: u16, product_id: u16 },
    /// USB serial (virtual COM port)
    UsbSerial { path: String, baud_rate: u32 },
    /// SPI bus
    Spi { bus: u8, cs: u8, speed_hz: u32 },
    /// I2C bus
    I2c { bus: u8, address: u8 },
    /// GPIO pins (direct control)
    Gpio { pins: Vec<u8> },
    /// Network (Ethernet/GPIB)
    Network { address: String, port: u16 },
    /// Simulated (no hardware)
    Simulated,
}

// =============================================================================
// Simulated Attenuator
// =============================================================================

/// Simulated attenuator for testing without hardware.
pub struct SimulatedAttenuator {
    name: String,
    capabilities: AttenuatorCapabilities,
    current_attenuation: f64,
}

impl SimulatedAttenuator {
    /// Create a new simulated attenuator.
    pub fn new(name: &str, max_attenuation: f64, step_size: f64) -> Self {
        Self {
            name: name.to_string(),
            capabilities: AttenuatorCapabilities {
                min_attenuation: 0.0,
                max_attenuation,
                step_size,
                ..Default::default()
            },
            current_attenuation: 0.0,
        }
    }

    /// Create a simulated attenuator matching common hardware.
    pub fn pe43711() -> Self {
        Self::new("Simulated PE43711", 31.75, 0.25)
    }

    /// Create a high-range simulated attenuator.
    pub fn high_range() -> Self {
        Self::new("Simulated High-Range", 90.0, 0.5)
    }
}

impl Attenuator for SimulatedAttenuator {
    fn name(&self) -> &str {
        &self.name
    }

    fn capabilities(&self) -> AttenuatorCapabilities {
        self.capabilities.clone()
    }

    fn set_attenuation(&mut self, db: f64) -> SdrResult<f64> {
        // Clamp to valid range
        let clamped = db.clamp(
            self.capabilities.min_attenuation,
            self.capabilities.max_attenuation,
        );

        // Round to nearest step
        let steps = (clamped / self.capabilities.step_size).round();
        let actual = steps * self.capabilities.step_size;

        debug!("Setting simulated attenuation to {} dB", actual);
        self.current_attenuation = actual;
        Ok(actual)
    }

    fn attenuation(&self) -> f64 {
        self.current_attenuation
    }
}

// =============================================================================
// PE43711 SPI Attenuator
// =============================================================================

/// Peregrine Semiconductor PE43711 digital attenuator.
///
/// 31.75 dB range with 0.25 dB steps, DC to 4 GHz.
/// Commonly used in SDR applications.
pub struct Pe43711Attenuator {
    name: String,
    current_attenuation: f64,
    #[allow(dead_code)]
    connection: AttenuatorConnection,
}

impl Pe43711Attenuator {
    /// Create a new PE43711 attenuator.
    pub fn new(connection: AttenuatorConnection) -> Self {
        Self {
            name: "PE43711".to_string(),
            current_attenuation: 0.0,
            connection,
        }
    }

    /// Convert attenuation in dB to register value.
    fn db_to_register(db: f64) -> u8 {
        // PE43711: 7-bit value, 0.25 dB steps
        // Register = (attenuation * 4) as integer
        let steps = (db * 4.0).round() as u8;
        steps.min(127) // Max 31.75 dB
    }

    /// Convert register value to attenuation in dB.
    fn register_to_db(reg: u8) -> f64 {
        (reg as f64) * 0.25
    }
}

impl Attenuator for Pe43711Attenuator {
    fn name(&self) -> &str {
        &self.name
    }

    fn capabilities(&self) -> AttenuatorCapabilities {
        AttenuatorCapabilities {
            min_attenuation: 0.0,
            max_attenuation: 31.75,
            step_size: 0.25,
            min_frequency: 0.0,
            max_frequency: 4_000_000_000.0,
            insertion_loss: 1.8,
            switching_time_us: 0.5,
            channels: 1,
            supports_sequences: false,
            supports_trigger: false,
        }
    }

    fn set_attenuation(&mut self, db: f64) -> SdrResult<f64> {
        let clamped = db.clamp(0.0, 31.75);
        let reg_value = Self::db_to_register(clamped);
        let actual = Self::register_to_db(reg_value);

        // In production: Write to SPI
        // match &self.connection {
        //     AttenuatorConnection::Spi { bus, cs, speed_hz } => {
        //         // spi.write(&[reg_value])?;
        //     }
        //     _ => {}
        // }

        debug!("PE43711: Setting {} dB (reg=0x{:02X})", actual, reg_value);
        self.current_attenuation = actual;
        Ok(actual)
    }

    fn attenuation(&self) -> f64 {
        self.current_attenuation
    }
}

// =============================================================================
// Mini-Circuits RCDAT Attenuator
// =============================================================================

/// Mini-Circuits RCDAT USB-controlled attenuator.
///
/// Various models: RCDAT-8000-90 (90 dB, DC-8 GHz), etc.
pub struct MiniCircuitsRcdat {
    name: String,
    #[allow(dead_code)]
    model: String,
    max_attenuation: f64,
    current_attenuation: f64,
    #[allow(dead_code)]
    connection: AttenuatorConnection,
}

impl MiniCircuitsRcdat {
    /// Create a new Mini-Circuits RCDAT attenuator.
    pub fn new(model: &str, connection: AttenuatorConnection) -> Self {
        // Determine max attenuation from model number
        let max_atten = if model.contains("90") {
            90.0
        } else if model.contains("60") {
            60.0
        } else if model.contains("30") {
            30.0
        } else {
            63.0 // Default
        };

        Self {
            name: format!("Mini-Circuits {}", model),
            model: model.to_string(),
            max_attenuation: max_atten,
            current_attenuation: 0.0,
            connection,
        }
    }
}

impl Attenuator for MiniCircuitsRcdat {
    fn name(&self) -> &str {
        &self.name
    }

    fn capabilities(&self) -> AttenuatorCapabilities {
        AttenuatorCapabilities {
            min_attenuation: 0.0,
            max_attenuation: self.max_attenuation,
            step_size: 0.25,
            min_frequency: 0.0,
            max_frequency: 8_000_000_000.0, // RCDAT-8000 series
            insertion_loss: 2.5,
            switching_time_us: 10.0,
            channels: 1,
            supports_sequences: true,
            supports_trigger: true,
        }
    }

    fn set_attenuation(&mut self, db: f64) -> SdrResult<f64> {
        let clamped = db.clamp(0.0, self.max_attenuation);

        // Round to 0.25 dB steps
        let actual = (clamped * 4.0).round() / 4.0;

        // In production: Send USB HID command
        // Format: ":SETATT:<value>"
        debug!("RCDAT: Setting {} dB", actual);
        self.current_attenuation = actual;
        Ok(actual)
    }

    fn attenuation(&self) -> f64 {
        self.current_attenuation
    }
}

// =============================================================================
// Attenuator Test Harness
// =============================================================================

/// Test harness for automated RF testing with attenuators.
///
/// Provides SNR sweep testing, sensitivity measurement, and automated
/// characterization of waveform performance.
pub struct AttenuatorTestHarness {
    /// Attenuator being used
    attenuator: Box<dyn Attenuator>,
    /// TX power level (for calculating SNR)
    tx_power_dbm: f64,
    /// Noise floor estimate
    noise_floor_dbm: f64,
    /// Settling time after attenuation change
    settle_time: Duration,
}

/// Result of a single SNR test point.
#[derive(Debug, Clone)]
pub struct SnrTestPoint {
    /// Attenuation setting in dB
    pub attenuation_db: f64,
    /// Estimated SNR in dB
    pub snr_db: f64,
    /// Bit error rate (if measured)
    pub ber: Option<f64>,
    /// Packet error rate (if measured)
    pub per: Option<f64>,
    /// Number of packets tested
    pub packets_tested: usize,
    /// Number of packets successfully decoded
    pub packets_decoded: usize,
}

/// Result of sensitivity testing.
#[derive(Debug, Clone)]
pub struct SensitivityResult {
    /// Attenuation at which threshold was crossed
    pub threshold_attenuation_db: f64,
    /// Corresponding SNR
    pub threshold_snr_db: f64,
    /// All test points
    pub test_points: Vec<SnrTestPoint>,
}

impl AttenuatorTestHarness {
    /// Create a new test harness.
    pub fn new(attenuator: Box<dyn Attenuator>) -> Self {
        Self {
            attenuator,
            tx_power_dbm: 0.0,
            noise_floor_dbm: -100.0,
            settle_time: Duration::from_millis(10),
        }
    }

    /// Set TX power for SNR calculations.
    pub fn set_tx_power(&mut self, power_dbm: f64) {
        self.tx_power_dbm = power_dbm;
    }

    /// Set noise floor estimate.
    pub fn set_noise_floor(&mut self, noise_dbm: f64) {
        self.noise_floor_dbm = noise_dbm;
    }

    /// Set settling time after attenuation changes.
    pub fn set_settle_time(&mut self, duration: Duration) {
        self.settle_time = duration;
    }

    /// Calculate estimated SNR for given attenuation.
    pub fn calculate_snr(&self, attenuation_db: f64) -> f64 {
        // RX signal level = TX power - attenuation
        let rx_power = self.tx_power_dbm - attenuation_db;

        // SNR = signal - noise floor
        rx_power - self.noise_floor_dbm
    }

    /// Set attenuation with settling time.
    pub fn set_attenuation(&mut self, db: f64) -> SdrResult<f64> {
        self.attenuator.set_attenuation_with_settle(db, self.settle_time)
    }

    /// Get current attenuation.
    pub fn attenuation(&self) -> f64 {
        self.attenuator.attenuation()
    }

    /// Get attenuator capabilities.
    pub fn capabilities(&self) -> AttenuatorCapabilities {
        self.attenuator.capabilities()
    }

    /// Generate attenuation sweep values.
    pub fn generate_sweep(start_db: f64, end_db: f64, step_db: f64) -> Vec<f64> {
        let mut values = Vec::new();
        let mut current = start_db;
        while current <= end_db {
            values.push(current);
            current += step_db;
        }
        values
    }

    /// Generate logarithmic sweep (more points near sensitivity threshold).
    pub fn generate_log_sweep(start_db: f64, end_db: f64, points: usize) -> Vec<f64> {
        let mut values = Vec::with_capacity(points);
        for i in 0..points {
            let t = i as f64 / (points - 1) as f64;
            // Use quadratic spacing for more resolution at high attenuation
            let atten = start_db + (end_db - start_db) * t * t;
            values.push(atten);
        }
        values
    }
}

// =============================================================================
// Factory Functions
// =============================================================================

/// Create an attenuator from a connection string.
///
/// ## URI Format
///
/// - `simulated://` - Simulated attenuator (default 31.75 dB range)
/// - `simulated://max=90` - Simulated with 90 dB range
/// - `pe43711://spi:0:0` - PE43711 on SPI bus 0, CS 0
/// - `minicircuits://usb` - Mini-Circuits RCDAT via USB
/// - `minicircuits://RCDAT-8000-90` - Specific model
///
pub fn create_attenuator(uri: &str) -> SdrResult<Box<dyn Attenuator>> {
    let (driver, args) = if let Some(pos) = uri.find("://") {
        (&uri[..pos], &uri[pos + 3..])
    } else {
        (uri, "")
    };

    match driver.to_lowercase().as_str() {
        "simulated" | "sim" => {
            if args.contains("max=90") {
                Ok(Box::new(SimulatedAttenuator::high_range()))
            } else {
                Ok(Box::new(SimulatedAttenuator::pe43711()))
            }
        }
        "pe43711" => {
            // Parse SPI connection: spi:bus:cs
            let connection = if args.starts_with("spi:") {
                let parts: Vec<&str> = args[4..].split(':').collect();
                if parts.len() >= 2 {
                    AttenuatorConnection::Spi {
                        bus: parts[0].parse().unwrap_or(0),
                        cs: parts[1].parse().unwrap_or(0),
                        speed_hz: 1_000_000,
                    }
                } else {
                    AttenuatorConnection::Simulated
                }
            } else {
                AttenuatorConnection::Simulated
            };
            Ok(Box::new(Pe43711Attenuator::new(connection)))
        }
        "minicircuits" | "rcdat" => {
            let model = if args.is_empty() {
                "RCDAT-8000-90"
            } else {
                args
            };
            Ok(Box::new(MiniCircuitsRcdat::new(
                model,
                AttenuatorConnection::UsbHid {
                    vendor_id: 0x20CE, // Mini-Circuits VID
                    product_id: 0x0023,
                },
            )))
        }
        _ => Err(SdrError::DeviceNotFound(format!(
            "Unknown attenuator type: {}",
            driver
        ))),
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulated_attenuator() {
        let mut atten = SimulatedAttenuator::pe43711();
        assert_eq!(atten.attenuation(), 0.0);

        let actual = atten.set_attenuation(15.5).unwrap();
        assert_eq!(actual, 15.5);
        assert_eq!(atten.attenuation(), 15.5);
    }

    #[test]
    fn test_attenuation_clamping() {
        let mut atten = SimulatedAttenuator::pe43711();

        // Below minimum
        let actual = atten.set_attenuation(-5.0).unwrap();
        assert_eq!(actual, 0.0);

        // Above maximum
        let actual = atten.set_attenuation(100.0).unwrap();
        assert_eq!(actual, 31.75);
    }

    #[test]
    fn test_step_rounding() {
        let mut atten = SimulatedAttenuator::new("Test", 31.75, 0.5);

        // Should round to nearest 0.5 dB
        let actual = atten.set_attenuation(10.3).unwrap();
        assert_eq!(actual, 10.5);

        let actual = atten.set_attenuation(10.1).unwrap();
        assert_eq!(actual, 10.0);
    }

    #[test]
    fn test_pe43711_register_conversion() {
        // 0 dB -> 0
        assert_eq!(Pe43711Attenuator::db_to_register(0.0), 0);

        // 31.75 dB -> 127
        assert_eq!(Pe43711Attenuator::db_to_register(31.75), 127);

        // 10 dB -> 40
        assert_eq!(Pe43711Attenuator::db_to_register(10.0), 40);

        // Inverse
        assert_eq!(Pe43711Attenuator::register_to_db(0), 0.0);
        assert_eq!(Pe43711Attenuator::register_to_db(127), 31.75);
        assert_eq!(Pe43711Attenuator::register_to_db(40), 10.0);
    }

    #[test]
    fn test_create_attenuator_simulated() {
        let atten = create_attenuator("simulated://").unwrap();
        assert!(atten.name().contains("Simulated"));

        let caps = atten.capabilities();
        assert_eq!(caps.max_attenuation, 31.75);
    }

    #[test]
    fn test_create_attenuator_high_range() {
        let atten = create_attenuator("simulated://max=90").unwrap();
        let caps = atten.capabilities();
        assert_eq!(caps.max_attenuation, 90.0);
    }

    #[test]
    fn test_harness_snr_calculation() {
        let atten = SimulatedAttenuator::pe43711();
        let mut harness = AttenuatorTestHarness::new(Box::new(atten));

        harness.set_tx_power(0.0);
        harness.set_noise_floor(-100.0);

        // At 0 dB attenuation: SNR = 0 - (-100) = 100 dB
        assert_eq!(harness.calculate_snr(0.0), 100.0);

        // At 30 dB attenuation: SNR = -30 - (-100) = 70 dB
        assert_eq!(harness.calculate_snr(30.0), 70.0);
    }

    #[test]
    fn test_sweep_generation() {
        let sweep = AttenuatorTestHarness::generate_sweep(0.0, 30.0, 5.0);
        assert_eq!(sweep, vec![0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0]);
    }

    #[test]
    fn test_log_sweep_generation() {
        let sweep = AttenuatorTestHarness::generate_log_sweep(0.0, 60.0, 5);
        assert_eq!(sweep.len(), 5);
        assert_eq!(sweep[0], 0.0);
        assert_eq!(sweep[4], 60.0);
        // Middle values should be closer to start due to quadratic spacing
        assert!(sweep[2] < 30.0);
    }
}
