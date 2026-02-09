//! Quadrature Demodulator (FM Discriminator)
//!
//! Computes the instantaneous frequency of a complex signal by measuring
//! the phase difference between consecutive samples. This is the fundamental
//! building block for receiving FM, NBFM, WBFM, FSK, GFSK, and GMSK signals.
//!
//! ## Algorithm
//!
//! ```text
//! y[n] = gain * arg( x[n] * conj(x[n-1]) )
//! ```
//!
//! For FM: `gain = sample_rate / (2π * max_deviation)`
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::quadrature_demod::QuadratureDemod;
//! use num_complex::Complex64;
//! use std::f64::consts::PI;
//!
//! // Create FM demodulator for NBFM (5 kHz deviation at 48 kHz sample rate)
//! let mut demod = QuadratureDemod::for_fm(48000.0, 5000.0);
//!
//! // A constant-frequency signal produces constant output
//! let freq = 1000.0; // 1 kHz tone
//! let samples: Vec<Complex64> = (0..100)
//!     .map(|i| {
//!         let phase = 2.0 * PI * freq * i as f64 / 48000.0;
//!         Complex64::new(phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! let output = demod.process_block(&samples);
//! // Output should be proportional to input frequency
//! assert!(output.len() == 100);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Quadrature demodulator (FM discriminator).
///
/// Extracts instantaneous frequency from a complex baseband signal.
#[derive(Debug, Clone)]
pub struct QuadratureDemod {
    gain: f64,
    prev: Complex64,
}

impl QuadratureDemod {
    /// Create with an explicit gain factor.
    ///
    /// Output = gain * arg(x[n] * conj(x[n-1]))
    pub fn new(gain: f64) -> Self {
        Self {
            gain,
            prev: Complex64::new(1.0, 0.0),
        }
    }

    /// Create for FM demodulation.
    ///
    /// gain = sample_rate / (2π * max_deviation_hz)
    ///
    /// Output is normalized so that ±max_deviation maps to ±1.0.
    pub fn for_fm(sample_rate: f64, max_deviation_hz: f64) -> Self {
        let gain = sample_rate / (2.0 * PI * max_deviation_hz);
        Self::new(gain)
    }

    /// Create for FSK demodulation.
    ///
    /// gain = sample_rate / (2π * deviation_hz)
    ///
    /// Output is normalized so ±deviation maps to ±1.0.
    pub fn for_fsk(sample_rate: f64, deviation_hz: f64) -> Self {
        Self::for_fm(sample_rate, deviation_hz)
    }

    /// Demodulate a single complex sample to a real output.
    pub fn process(&mut self, input: Complex64) -> f64 {
        let product = input * self.prev.conj();
        self.prev = input;
        self.gain * product.arg()
    }

    /// Demodulate a block of complex samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<f64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    /// Get the current gain.
    pub fn gain(&self) -> f64 {
        self.gain
    }

    /// Set a new gain.
    pub fn set_gain(&mut self, gain: f64) {
        self.gain = gain;
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.prev = Complex64::new(1.0, 0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dc_input_zero_output() {
        // A constant (DC) signal should produce ~0 output (no frequency change)
        let mut demod = QuadratureDemod::new(1.0);
        let dc = vec![Complex64::new(1.0, 0.0); 100];
        let output = demod.process_block(&dc);
        for &v in &output {
            assert!(v.abs() < 1e-10, "DC should give 0 output, got {}", v);
        }
    }

    #[test]
    fn test_positive_frequency() {
        let mut demod = QuadratureDemod::new(1.0);
        let n = 1000;
        let fs = 48000.0;
        let freq = 1000.0; // 1 kHz

        let samples: Vec<Complex64> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * freq * i as f64 / fs;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        let output = demod.process_block(&samples);
        // Expected output (after first sample): gain * 2π * freq / fs
        let expected = 2.0 * PI * freq / fs;
        // Skip first sample (startup transient)
        for &v in &output[1..] {
            assert!(
                (v - expected).abs() < 1e-6,
                "Expected {:.6}, got {:.6}",
                expected,
                v
            );
        }
    }

    #[test]
    fn test_negative_frequency() {
        let mut demod = QuadratureDemod::new(1.0);
        let n = 1000;
        let fs = 48000.0;
        let freq = -2000.0; // -2 kHz (negative rotation)

        let samples: Vec<Complex64> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * freq * i as f64 / fs;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        let output = demod.process_block(&samples);
        let expected = 2.0 * PI * freq / fs;
        for &v in &output[1..] {
            assert!(
                (v - expected).abs() < 1e-6,
                "Expected {:.6}, got {:.6}",
                expected,
                v
            );
        }
    }

    #[test]
    fn test_fm_gain_normalization() {
        let fs = 48000.0;
        let deviation = 5000.0;
        let mut demod = QuadratureDemod::for_fm(fs, deviation);

        // A tone at exactly max_deviation should give output ≈ ±1.0
        let samples: Vec<Complex64> = (0..1000)
            .map(|i| {
                let phase = 2.0 * PI * deviation * i as f64 / fs;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        let output = demod.process_block(&samples);
        for &v in &output[1..] {
            assert!(
                (v - 1.0).abs() < 1e-4,
                "At max deviation, output should be ~1.0, got {}",
                v
            );
        }
    }

    #[test]
    fn test_fsk_binary() {
        let fs = 48000.0;
        let deviation = 1200.0; // 1200 Hz FSK deviation
        let mut demod = QuadratureDemod::for_fsk(fs, deviation);

        // Mark frequency: +deviation
        let mark: Vec<Complex64> = (0..100)
            .map(|i| {
                let phase = 2.0 * PI * deviation * i as f64 / fs;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        // Space frequency: -deviation
        let space: Vec<Complex64> = (0..100)
            .map(|i| {
                let phase = -2.0 * PI * deviation * i as f64 / fs;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        let mark_out = demod.process_block(&mark);
        demod.reset();
        let space_out = demod.process_block(&space);

        // Mark should be positive (~1.0), space should be negative (~-1.0)
        let mark_avg: f64 = mark_out[5..].iter().sum::<f64>() / (mark_out.len() - 5) as f64;
        let space_avg: f64 = space_out[5..].iter().sum::<f64>() / (space_out.len() - 5) as f64;

        assert!(mark_avg > 0.9, "Mark should be positive: {}", mark_avg);
        assert!(space_avg < -0.9, "Space should be negative: {}", space_avg);
    }

    #[test]
    fn test_gain_setter() {
        let mut demod = QuadratureDemod::new(1.0);
        assert!((demod.gain() - 1.0).abs() < 1e-10);
        demod.set_gain(2.5);
        assert!((demod.gain() - 2.5).abs() < 1e-10);
    }

    #[test]
    fn test_reset() {
        let mut demod = QuadratureDemod::new(1.0);
        // Process some samples
        demod.process(Complex64::new(0.0, 1.0));
        demod.process(Complex64::new(-1.0, 0.0));
        demod.reset();
        // After reset, first sample should give 0 (prev = 1+0j)
        let out = demod.process(Complex64::new(1.0, 0.0));
        assert!(out.abs() < 1e-10, "After reset, first sample should be 0: {}", out);
    }

    #[test]
    fn test_zero_input() {
        let mut demod = QuadratureDemod::new(1.0);
        // Zero-magnitude samples: arg is undefined but shouldn't panic
        let zeros = vec![Complex64::new(0.0, 0.0); 10];
        let output = demod.process_block(&zeros);
        assert_eq!(output.len(), 10);
        // All outputs should be finite (atan2(0,0) = 0 in Rust)
        for &v in &output {
            assert!(v.is_finite(), "Output should be finite for zero input");
        }
    }
}
