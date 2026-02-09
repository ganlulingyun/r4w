//! Carrier Recovery (Costas Loop)
//!
//! Standalone carrier frequency and phase recovery for PSK signals.
//! Extracts from GNSS-specific tracking into a general-purpose block.
//!
//! ## Variants
//!
//! - [`CostasLoop`] - 2nd/4th order Costas loop for BPSK/QPSK
//! - Configurable loop bandwidth, damping factor, and modulation order
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::carrier_recovery::{CostasLoop, CostasConfig};
//! use num_complex::Complex64;
//! use std::f64::consts::PI;
//!
//! let config = CostasConfig::bpsk(0.01); // BPSK, bandwidth 0.01 rad/sample
//! let mut costas = CostasLoop::new(config);
//!
//! // Process signal with 0.1 rad/sample frequency offset
//! let samples: Vec<Complex64> = (0..1000).map(|i| {
//!     let phase = 0.1 * i as f64; // frequency offset
//!     Complex64::new(phase.cos(), phase.sin())
//! }).collect();
//!
//! let corrected = costas.process_block(&samples);
//! // Output should be phase-corrected (real axis only for BPSK)
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Configuration for the Costas Loop.
#[derive(Debug, Clone)]
pub struct CostasConfig {
    /// Modulation order: 2=BPSK, 4=QPSK, 8=8PSK
    pub order: u32,
    /// Loop bandwidth in radians per sample (typical: 0.005 to 0.1)
    pub loop_bw: f64,
    /// Damping factor (typical: 0.707 = critically damped)
    pub damping: f64,
    /// Maximum frequency offset the loop can track (rad/sample)
    pub max_freq: f64,
}

impl CostasConfig {
    /// Create a BPSK Costas loop configuration.
    pub fn bpsk(loop_bw: f64) -> Self {
        Self {
            order: 2,
            loop_bw,
            damping: 0.707,
            max_freq: 1.0,
        }
    }

    /// Create a QPSK Costas loop configuration.
    pub fn qpsk(loop_bw: f64) -> Self {
        Self {
            order: 4,
            loop_bw,
            damping: 0.707,
            max_freq: 1.0,
        }
    }

    /// Create an 8PSK Costas loop configuration.
    pub fn _8psk(loop_bw: f64) -> Self {
        Self {
            order: 8,
            loop_bw,
            damping: 0.707,
            max_freq: 1.0,
        }
    }
}

/// Costas Loop for carrier frequency and phase recovery.
///
/// The Costas loop is a PLL variant that works for suppressed-carrier
/// modulations (PSK). It uses a modulation-order-aware phase detector
/// to generate an error signal that drives the loop filter.
///
/// ## Loop Filter
///
/// Uses a 2nd-order proportional-integral loop filter:
/// ```text
/// freq += beta * error
/// phase += freq + alpha * error
/// ```
///
/// Where alpha and beta are computed from loop bandwidth and damping factor.
#[derive(Debug, Clone)]
pub struct CostasLoop {
    config: CostasConfig,
    /// Current phase estimate (radians)
    phase: f64,
    /// Current frequency estimate (radians/sample)
    freq: f64,
    /// Proportional gain
    alpha: f64,
    /// Integral gain
    beta: f64,
}

impl CostasLoop {
    /// Create a new Costas loop.
    pub fn new(config: CostasConfig) -> Self {
        // Compute loop filter gains from bandwidth and damping
        // Using the standard formulas for a 2nd-order PLL:
        //   denom = 1 + 2*damping*bw + bw^2
        //   alpha = (4*damping*bw) / denom
        //   beta = (4*bw^2) / denom
        let bw = config.loop_bw;
        let damp = config.damping;
        let denom = 1.0 + 2.0 * damp * bw + bw * bw;
        let alpha = (4.0 * damp * bw) / denom;
        let beta = (4.0 * bw * bw) / denom;

        Self {
            config,
            phase: 0.0,
            freq: 0.0,
            alpha,
            beta,
        }
    }

    /// Get the current phase estimate.
    pub fn phase(&self) -> f64 {
        self.phase
    }

    /// Get the current frequency estimate (radians/sample).
    pub fn frequency(&self) -> f64 {
        self.freq
    }

    /// Get the current frequency estimate in Hz.
    pub fn frequency_hz(&self, sample_rate: f64) -> f64 {
        self.freq * sample_rate / (2.0 * PI)
    }

    /// Compute the phase error for the given output sample.
    fn phase_error(&self, sample: Complex64) -> f64 {
        match self.config.order {
            2 => {
                // BPSK Costas: error = I * Q
                // When locked, I contains data, Q should be ~0
                sample.re * sample.im
            }
            4 => {
                // QPSK Costas: error = sign(I) * Q - sign(Q) * I
                let sign_i = if sample.re >= 0.0 { 1.0 } else { -1.0 };
                let sign_q = if sample.im >= 0.0 { 1.0 } else { -1.0 };
                sign_i * sample.im - sign_q * sample.re
            }
            8 => {
                // 8PSK: error based on 8th power phase detector
                let angle = sample.im.atan2(sample.re);
                let sector = (angle * 4.0 / PI).round() * PI / 4.0;
                let mut err = angle - sector;
                if err > PI / 8.0 {
                    err -= PI / 4.0;
                }
                if err < -PI / 8.0 {
                    err += PI / 4.0;
                }
                err
            }
            _ => sample.re * sample.im, // Default to BPSK-like
        }
    }

    /// Process a single complex sample.
    ///
    /// Returns the phase-corrected sample.
    pub fn process(&mut self, input: Complex64) -> Complex64 {
        // Mix input with NCO (remove estimated carrier)
        let nco = Complex64::new((-self.phase).cos(), (-self.phase).sin());
        let output = input * nco;

        // Compute phase error
        let error = self.phase_error(output);

        // Update loop filter (proportional-integral)
        self.freq += self.beta * error;

        // Clamp frequency
        if self.freq > self.config.max_freq {
            self.freq = self.config.max_freq;
        }
        if self.freq < -self.config.max_freq {
            self.freq = -self.config.max_freq;
        }

        // Update phase
        self.phase += self.freq + self.alpha * error;

        // Wrap phase to [-pi, pi]
        while self.phase > PI {
            self.phase -= 2.0 * PI;
        }
        while self.phase < -PI {
            self.phase += 2.0 * PI;
        }

        output
    }

    /// Process a block of complex samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    /// Process samples in place.
    pub fn process_inplace(&mut self, samples: &mut [Complex64]) {
        for s in samples.iter_mut() {
            *s = self.process(*s);
        }
    }

    /// Reset the loop to initial state.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.freq = 0.0;
    }
}

// Implement Filter trait for interoperability with the filter pipeline
impl crate::filters::Filter for CostasLoop {
    fn process(&mut self, input: Complex64) -> Complex64 {
        CostasLoop::process(self, input)
    }

    fn reset(&mut self) {
        CostasLoop::reset(self);
    }

    fn group_delay(&self) -> f64 {
        0.0 // Costas loop has no deterministic group delay
    }

    fn order(&self) -> usize {
        2 // 2nd-order loop
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bpsk_static_phase_correction() {
        let config = CostasConfig::bpsk(0.05);
        let mut costas = CostasLoop::new(config);

        // BPSK signal with static phase offset of 0.5 radians
        let phase_offset: f64 = 0.5;
        let samples: Vec<Complex64> = (0..2000)
            .map(|_| {
                // BPSK: all +1 symbols
                Complex64::new(phase_offset.cos(), phase_offset.sin())
            })
            .collect();

        let output = costas.process_block(&samples);

        // After convergence, output should be on the real axis
        let last = output.last().unwrap();
        assert!(
            last.im.abs() < 0.3,
            "Costas should correct static phase: Q={:.3}",
            last.im
        );
    }

    #[test]
    fn test_bpsk_frequency_offset_tracking() {
        let config = CostasConfig::bpsk(0.05);
        let mut costas = CostasLoop::new(config);

        // BPSK signal with small frequency offset
        let freq_offset = 0.01; // rad/sample
        let samples: Vec<Complex64> = (0..5000)
            .map(|i| {
                let phase = freq_offset * i as f64;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        let output = costas.process_block(&samples);

        // After convergence, frequency estimate should be close to offset
        assert!(
            (costas.frequency() - freq_offset).abs() < 0.01,
            "Frequency should track: est={:.4}, actual={:.4}",
            costas.frequency(),
            freq_offset
        );

        // Output should be roughly on the real axis
        let last = output.last().unwrap();
        assert!(
            last.im.abs() < 0.5,
            "Output should be corrected: Q={:.3}",
            last.im
        );
    }

    #[test]
    fn test_qpsk_phase_correction() {
        let config = CostasConfig::qpsk(0.03);
        let mut costas = CostasLoop::new(config);

        // QPSK signal: symbol at 45 degrees with phase offset
        let phase_offset = 0.3;
        let symbol_phase = PI / 4.0; // QPSK constellation point
        let samples: Vec<Complex64> = (0..3000)
            .map(|_| {
                let total_phase = symbol_phase + phase_offset;
                Complex64::new(total_phase.cos(), total_phase.sin())
            })
            .collect();

        let output = costas.process_block(&samples);

        // After convergence, output angle should be near pi/4 (45 degrees)
        let last = output.last().unwrap();
        let angle = last.im.atan2(last.re);
        let target = PI / 4.0;
        let error = ((angle - target + PI) % (2.0 * PI) - PI).abs();
        // Allow for pi/2 ambiguity in QPSK
        let min_error = error.min((error - PI / 2.0).abs());
        assert!(
            min_error < 0.5,
            "QPSK Costas should correct phase: angle={:.3}, target={:.3}",
            angle,
            target
        );
    }

    #[test]
    fn test_costas_reset() {
        let config = CostasConfig::bpsk(0.05);
        let mut costas = CostasLoop::new(config);

        let input = vec![Complex64::new(1.0, 0.5); 100];
        let _ = costas.process_block(&input);

        assert!(costas.phase().abs() > 0.0 || costas.frequency().abs() > 0.0);

        costas.reset();
        assert_eq!(costas.phase(), 0.0);
        assert_eq!(costas.frequency(), 0.0);
    }

    #[test]
    fn test_costas_filter_trait() {
        use crate::filters::Filter;
        let config = CostasConfig::bpsk(0.05);
        let mut costas = CostasLoop::new(config);

        let input = Complex64::new(1.0, 0.0);
        let output = Filter::process(&mut costas, input);
        assert!(output.norm() > 0.0);
        assert_eq!(costas.group_delay(), 0.0);
    }

    #[test]
    fn test_frequency_hz_conversion() {
        let config = CostasConfig::bpsk(0.05);
        let mut costas = CostasLoop::new(config);

        // Manually set frequency for testing
        let samples: Vec<Complex64> = (0..5000)
            .map(|i| {
                let phase = 0.02 * i as f64;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        let _ = costas.process_block(&samples);

        // At 48000 Hz sample rate, 0.02 rad/sample ≈ 152.8 Hz
        let freq_hz = costas.frequency_hz(48000.0);
        let expected_hz = 0.02 * 48000.0 / (2.0 * PI);
        assert!(
            (freq_hz - expected_hz).abs() < 50.0,
            "Frequency Hz: got {:.1}, expected ~{:.1}",
            freq_hz,
            expected_hz
        );
    }
}
