//! Costas Loop — Decision-Directed Carrier Recovery
//!
//! Recovers carrier phase and frequency for PSK-modulated signals.
//! Unlike a generic PLL, the Costas loop uses a decision-directed phase
//! detector that works with modulated (data-bearing) signals.
//!
//! ## Supported Modulation Orders
//!
//! | Order | Modulation | Phase detector                    |
//! |-------|-----------|-----------------------------------|
//! | 2     | BPSK      | `error = re(y) * im(y)`          |
//! | 4     | QPSK      | `error = re(y)*sign(im(y)) - im(y)*sign(re(y))` |
//! | 8     | 8PSK      | 8th-power phase detector          |
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::costas_loop::{CostasLoop, CostasConfig};
//! use num_complex::Complex64;
//! use std::f64::consts::PI;
//!
//! // Create a QPSK Costas loop
//! let config = CostasConfig {
//!     order: 4,
//!     loop_bw: 0.05,
//!     ..Default::default()
//! };
//! let mut costas = CostasLoop::new(config);
//!
//! // Process QPSK signal with unknown carrier offset
//! let carrier_offset = 0.1; // radians/sample
//! for i in 0..200 {
//!     // QPSK symbol at 45° with carrier offset
//!     let phase = PI / 4.0 + carrier_offset * i as f64;
//!     let sample = Complex64::new(phase.cos(), phase.sin());
//!     let corrected = costas.process_sample(sample);
//! }
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Costas loop configuration.
#[derive(Debug, Clone)]
pub struct CostasConfig {
    /// Modulation order: 2 (BPSK), 4 (QPSK), or 8 (8PSK).
    pub order: u8,
    /// Loop bandwidth (normalized, 0.0 to 1.0). Controls lock speed vs noise.
    /// Typical: 0.01-0.1
    pub loop_bw: f64,
    /// Damping factor. 0.707 = critically damped.
    pub damping: f64,
    /// Maximum frequency offset (normalized, radians/sample).
    pub max_freq: f64,
}

impl Default for CostasConfig {
    fn default() -> Self {
        Self {
            order: 4,
            loop_bw: 0.05,
            damping: 0.707,
            max_freq: 1.0,
        }
    }
}

/// Costas loop for carrier recovery.
///
/// The Costas loop de-rotates the input signal to remove carrier offset,
/// then uses a decision-directed phase detector to track residual phase error.
///
/// ```text
/// input → [×exp(-jθ)] → corrected_output
///              ↑                ↓
///         phase_accum ← loop_filter ← phase_detector
/// ```
#[derive(Debug, Clone)]
pub struct CostasLoop {
    /// Modulation order
    order: u8,
    /// Phase accumulator (radians)
    phase: f64,
    /// Frequency (radians/sample)
    freq: f64,
    /// Proportional gain
    alpha: f64,
    /// Integral gain
    beta: f64,
    /// Max frequency (radians/sample)
    max_freq: f64,
    /// Last phase error (for monitoring)
    last_error: f64,
}

impl CostasLoop {
    /// Create a new Costas loop.
    pub fn new(config: CostasConfig) -> Self {
        // Compute loop filter gains from bandwidth and damping
        let omega_n = config.loop_bw
            / (config.damping + 1.0 / (4.0 * config.damping));
        let alpha = 2.0 * config.damping * omega_n;
        let beta = omega_n * omega_n;

        Self {
            order: config.order.max(2),
            phase: 0.0,
            freq: 0.0,
            alpha,
            beta,
            max_freq: config.max_freq,
            last_error: 0.0,
        }
    }

    /// Process a single input sample.
    ///
    /// Returns the carrier-corrected output sample.
    pub fn process_sample(&mut self, input: Complex64) -> Complex64 {
        // De-rotate by current phase estimate
        let correction = Complex64::new((-self.phase).cos(), (-self.phase).sin());
        let corrected = input * correction;

        // Phase detector (depends on modulation order)
        let error = self.phase_detector(corrected);
        self.last_error = error;

        // Loop filter (PI controller)
        self.freq += self.beta * error;
        self.freq = self.freq.clamp(-self.max_freq, self.max_freq);
        self.phase += self.freq + self.alpha * error;

        // Wrap phase
        while self.phase > PI {
            self.phase -= 2.0 * PI;
        }
        while self.phase < -PI {
            self.phase += 2.0 * PI;
        }

        corrected
    }

    /// Process a block of samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process_sample(s)).collect()
    }

    /// Phase detector for different modulation orders.
    fn phase_detector(&self, symbol: Complex64) -> f64 {
        match self.order {
            2 => {
                // BPSK: error = re(y) * im(y)
                symbol.re * symbol.im
            }
            4 => {
                // QPSK: error = re(y)*sgn(im(y)) - im(y)*sgn(re(y))
                let sgn_re = if symbol.re >= 0.0 { 1.0 } else { -1.0 };
                let sgn_im = if symbol.im >= 0.0 { 1.0 } else { -1.0 };
                symbol.re * sgn_im - symbol.im * sgn_re
            }
            8 => {
                // 8PSK: use 8th-power phase detector
                // Raise to 8th power to remove data modulation, then extract phase
                let y8 = symbol
                    * symbol
                    * symbol
                    * symbol
                    * symbol
                    * symbol
                    * symbol
                    * symbol;
                y8.arg() / 8.0
            }
            _ => {
                // Generic: raise to Nth power
                let n = self.order as i32;
                let mut y_n = Complex64::new(1.0, 0.0);
                for _ in 0..n {
                    y_n *= symbol;
                }
                y_n.arg() / n as f64
            }
        }
    }

    /// Get current phase estimate.
    pub fn phase(&self) -> f64 {
        self.phase
    }

    /// Get current frequency estimate (radians/sample).
    pub fn frequency(&self) -> f64 {
        self.freq
    }

    /// Get current frequency estimate in Hz given sample rate.
    pub fn frequency_hz(&self, sample_rate: f64) -> f64 {
        self.freq * sample_rate / (2.0 * PI)
    }

    /// Get last phase error.
    pub fn phase_error(&self) -> f64 {
        self.last_error
    }

    /// Set initial frequency estimate.
    pub fn set_frequency(&mut self, freq_rad_per_sample: f64) {
        self.freq = freq_rad_per_sample;
    }

    /// Reset the loop.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.freq = 0.0;
        self.last_error = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bpsk_costas_tracks_offset() {
        let config = CostasConfig {
            order: 2,
            loop_bw: 0.1,
            damping: 1.0,
            max_freq: 0.5,
        };
        let mut costas = CostasLoop::new(config);

        // BPSK signal with constant carrier offset
        let offset = 0.05; // radians/sample
        let symbols = [1.0f64, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0];
        let mut phase: f64 = 0.0;

        for _ in 0..5 {
            // Multiple passes for convergence
            for &sym in &symbols {
                phase += offset;
                let sample = Complex64::new(sym * phase.cos(), sym * phase.sin());
                costas.process_sample(sample);
            }
        }

        // After convergence, frequency estimate should be near the offset
        assert!(
            (costas.frequency() - offset).abs() < 0.02,
            "Costas loop should track frequency offset: got {:.4}, expected {:.4}",
            costas.frequency(),
            offset
        );
    }

    #[test]
    fn test_qpsk_costas_reduces_error() {
        let config = CostasConfig {
            order: 4,
            loop_bw: 0.08,
            damping: 1.0,
            ..Default::default()
        };
        let mut costas = CostasLoop::new(config);

        // QPSK symbols with constant carrier offset
        let carrier_offset = 0.05; // radians/sample
        let qpsk_angles = [PI / 4.0, 3.0 * PI / 4.0, 5.0 * PI / 4.0, 7.0 * PI / 4.0];

        // Run many symbols for convergence
        for i in 0..200 {
            let angle = qpsk_angles[i % 4] + carrier_offset * i as f64;
            let sample = Complex64::new(angle.cos(), angle.sin());
            costas.process_sample(sample);
        }

        // After convergence, frequency estimate should be near the offset
        assert!(
            (costas.frequency() - carrier_offset).abs() < 0.02,
            "QPSK Costas should track frequency: got {:.4}, expected {:.4}",
            costas.frequency(),
            carrier_offset
        );

        // Phase error should be small after convergence
        assert!(
            costas.phase_error().abs() < 1.0,
            "Phase error should be small after convergence: {:.3}",
            costas.phase_error()
        );
    }

    #[test]
    fn test_costas_zero_offset() {
        let config = CostasConfig {
            order: 2,
            loop_bw: 0.05,
            ..Default::default()
        };
        let mut costas = CostasLoop::new(config);

        // BPSK with no offset — loop should stay near zero
        for _ in 0..100 {
            let sample = Complex64::new(1.0, 0.0);
            costas.process_sample(sample);
        }

        assert!(
            costas.frequency().abs() < 0.01,
            "No offset should result in ~zero frequency: got {:.4}",
            costas.frequency()
        );
    }

    #[test]
    fn test_costas_reset() {
        let mut costas = CostasLoop::new(CostasConfig::default());
        costas.set_frequency(0.1);
        assert!((costas.frequency() - 0.1).abs() < 1e-10);

        costas.reset();
        assert!(costas.frequency().abs() < 1e-10);
        assert!(costas.phase().abs() < 1e-10);
    }

    #[test]
    fn test_costas_frequency_hz() {
        let mut costas = CostasLoop::new(CostasConfig::default());
        costas.set_frequency(2.0 * PI * 1000.0 / 48000.0);
        let hz = costas.frequency_hz(48000.0);
        assert!(
            (hz - 1000.0).abs() < 1.0,
            "Frequency should be ~1000 Hz: got {hz:.1}"
        );
    }

    #[test]
    fn test_8psk_costas() {
        let config = CostasConfig {
            order: 8,
            loop_bw: 0.05,
            damping: 1.0,
            ..Default::default()
        };
        let mut costas = CostasLoop::new(config);

        // 8PSK with small frequency offset
        let offset = 0.02;
        let mut phase = 0.0;
        for i in 0..500 {
            let sym_angle = 2.0 * PI * (i % 8) as f64 / 8.0;
            phase += offset;
            let sample = Complex64::new(
                (sym_angle + phase).cos(),
                (sym_angle + phase).sin(),
            );
            costas.process_sample(sample);
        }

        assert!(
            (costas.frequency() - offset).abs() < 0.02,
            "8PSK Costas should track: got {:.4}, expected {:.4}",
            costas.frequency(),
            offset
        );
    }

    #[test]
    fn test_costas_block_processing() {
        let config = CostasConfig {
            order: 2,
            loop_bw: 0.05,
            ..Default::default()
        };
        let mut costas = CostasLoop::new(config);

        let input: Vec<Complex64> = (0..100)
            .map(|_| Complex64::new(1.0, 0.0))
            .collect();
        let output = costas.process_block(&input);
        assert_eq!(output.len(), 100);
    }
}
