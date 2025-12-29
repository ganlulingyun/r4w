//! Core types for LoRa signal processing
//!
//! This module defines the fundamental types used throughout the LoRa DSP library,
//! particularly for representing complex I/Q (In-phase/Quadrature) samples.
//!
//! ## Understanding I/Q Samples
//!
//! In Software Defined Radio (SDR), signals are represented as complex numbers
//! where:
//! - **I (In-phase)**: The real component, representing the signal aligned with
//!   a reference carrier
//! - **Q (Quadrature)**: The imaginary component, representing the signal 90°
//!   out of phase with the carrier
//!
//! This representation allows us to capture both amplitude AND phase information,
//! which is essential for modern digital modulation schemes like LoRa's CSS.
//!
//! ```text
//!            Q (Imaginary)
//!            ^
//!            |     * (I=0.7, Q=0.7)
//!            |    /
//!            |   / magnitude = 1.0
//!            |  /  phase = 45°
//!            | /
//!   ---------+---------> I (Real)
//!            |
//! ```

use num_complex::Complex64;
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Type alias for complex numbers using f64 precision
pub type Complex = Complex64;

/// A single I/Q sample point
pub type IQSample = Complex64;

/// A floating point sample (for real-valued signals)
pub type Sample = f64;

/// A buffer of I/Q samples
pub type IQBuffer = Vec<IQSample>;

/// Represents a symbol in the LoRa modulation scheme
///
/// Symbols are integers from 0 to 2^SF - 1, where SF is the spreading factor.
/// Each symbol encodes SF bits of data.
pub type Symbol = u16;

/// Represents raw bits as a vector of bytes
pub type BitStream = Vec<u8>;

/// Result type for DSP operations
pub type DspResult<T> = Result<T, DspError>;

/// Errors that can occur during DSP operations
#[derive(Debug, Clone, thiserror::Error)]
pub enum DspError {
    #[error("Invalid spreading factor: {0}. Must be between 5 and 12")]
    InvalidSpreadingFactor(u8),

    #[error("Invalid bandwidth: {0} Hz. Must be 125000, 250000, or 500000")]
    InvalidBandwidth(f64),

    #[error("Invalid coding rate: {0}. Must be between 1 and 4")]
    InvalidCodingRate(u8),

    #[error("Buffer too short: expected {expected}, got {actual}")]
    BufferTooShort { expected: usize, actual: usize },

    #[error("No preamble detected in signal")]
    NoPreambleDetected,

    #[error("CRC check failed")]
    CrcError,

    #[error("Header decode failed: {0}")]
    HeaderDecodeError(String),

    #[error("Synchronization failed: {0}")]
    SyncError(String),
}

/// Represents a point in the signal processing pipeline
///
/// This is useful for educational visualization to show how the signal
/// transforms at each stage.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PipelineStage {
    /// Name of this processing stage
    pub name: String,
    /// Description of what happens at this stage
    pub description: String,
    /// Time-domain samples (if applicable)
    pub time_domain: Option<Vec<IQSample>>,
    /// Frequency-domain samples (if applicable)
    pub freq_domain: Option<Vec<IQSample>>,
    /// Symbol values (if applicable)
    pub symbols: Option<Vec<Symbol>>,
    /// Raw bits (if applicable)
    pub bits: Option<BitStream>,
}

impl PipelineStage {
    pub fn new(name: impl Into<String>, description: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            description: description.into(),
            time_domain: None,
            freq_domain: None,
            symbols: None,
            bits: None,
        }
    }

    pub fn with_time_domain(mut self, samples: Vec<IQSample>) -> Self {
        self.time_domain = Some(samples);
        self
    }

    pub fn with_freq_domain(mut self, samples: Vec<IQSample>) -> Self {
        self.freq_domain = Some(samples);
        self
    }

    pub fn with_symbols(mut self, symbols: Vec<Symbol>) -> Self {
        self.symbols = Some(symbols);
        self
    }

    pub fn with_bits(mut self, bits: BitStream) -> Self {
        self.bits = Some(bits);
        self
    }
}

/// Helper functions for working with complex samples
pub mod complex_ops {
    use super::*;

    /// Create a complex number from magnitude and phase
    #[inline]
    pub fn from_polar(magnitude: f64, phase: f64) -> Complex {
        Complex::new(magnitude * phase.cos(), magnitude * phase.sin())
    }

    /// Get the magnitude of a complex number
    #[inline]
    pub fn magnitude(c: Complex) -> f64 {
        c.norm()
    }

    /// Get the phase (argument) of a complex number in radians
    #[inline]
    pub fn phase(c: Complex) -> f64 {
        c.arg()
    }

    /// Convert phase from radians to degrees
    #[inline]
    pub fn rad_to_deg(radians: f64) -> f64 {
        radians * 180.0 / PI
    }

    /// Convert phase from degrees to radians
    #[inline]
    pub fn deg_to_rad(degrees: f64) -> f64 {
        degrees * PI / 180.0
    }

    /// Compute the power (magnitude squared) of a complex number
    #[inline]
    pub fn power(c: Complex) -> f64 {
        c.norm_sqr()
    }

    /// Compute the average power of a signal
    pub fn average_power(samples: &[IQSample]) -> f64 {
        if samples.is_empty() {
            return 0.0;
        }
        samples.iter().map(|s| power(*s)).sum::<f64>() / samples.len() as f64
    }

    /// Normalize samples to unit power
    pub fn normalize(samples: &mut [IQSample]) {
        let avg_power = average_power(samples);
        if avg_power > 0.0 {
            let scale = 1.0 / avg_power.sqrt();
            for s in samples.iter_mut() {
                *s *= scale;
            }
        }
    }

    /// Generate a complex exponential (cisoid) at given frequency
    ///
    /// This is the fundamental building block for digital signal generation.
    /// Returns e^(j*2*π*f*t) where t = sample_idx / sample_rate
    #[inline]
    pub fn cis(frequency: f64, sample_idx: usize, sample_rate: f64) -> Complex {
        let t = sample_idx as f64 / sample_rate;
        let phase = 2.0 * PI * frequency * t;
        Complex::new(phase.cos(), phase.sin())
    }
}

/// Signal statistics for analysis and debugging
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignalStats {
    pub num_samples: usize,
    pub average_power: f64,
    pub peak_power: f64,
    pub dc_offset: Complex,
    pub snr_estimate: Option<f64>,
}

impl SignalStats {
    pub fn compute(samples: &[IQSample]) -> Self {
        let num_samples = samples.len();
        if num_samples == 0 {
            return Self {
                num_samples: 0,
                average_power: 0.0,
                peak_power: 0.0,
                dc_offset: Complex::new(0.0, 0.0),
                snr_estimate: None,
            };
        }

        let average_power = complex_ops::average_power(samples);
        let peak_power = samples
            .iter()
            .map(|s| complex_ops::power(*s))
            .fold(0.0_f64, f64::max);

        let dc_offset = samples.iter().copied().sum::<Complex>() / num_samples as f64;

        Self {
            num_samples,
            average_power,
            peak_power,
            dc_offset,
            snr_estimate: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_complex_from_polar() {
        let c = complex_ops::from_polar(1.0, PI / 4.0);
        assert_relative_eq!(c.re, 0.7071067811865476, epsilon = 1e-10);
        assert_relative_eq!(c.im, 0.7071067811865476, epsilon = 1e-10);
    }

    #[test]
    fn test_complex_magnitude_phase() {
        let c = Complex::new(3.0, 4.0);
        assert_relative_eq!(complex_ops::magnitude(c), 5.0, epsilon = 1e-10);
    }

    #[test]
    fn test_average_power() {
        let samples = vec![
            Complex::new(1.0, 0.0),
            Complex::new(0.0, 1.0),
            Complex::new(-1.0, 0.0),
            Complex::new(0.0, -1.0),
        ];
        assert_relative_eq!(complex_ops::average_power(&samples), 1.0, epsilon = 1e-10);
    }
}
