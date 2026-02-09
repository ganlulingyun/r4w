//! FM Receiver — Narrowband and Wideband
//!
//! Composite FM receiver blocks that combine filtering, quadrature
//! demodulation, and de-emphasis into a single convenient block.
//!
//! - `NbfmReceiver`: Narrowband FM (voice, packet radio, CTCSS)
//! - `WbfmReceiver`: Wideband FM (broadcast FM)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::fm_receiver::NbfmReceiver;
//! use num_complex::Complex64;
//!
//! let mut rx = NbfmReceiver::new(48000.0, 5000.0);
//! let samples = vec![Complex64::new(1.0, 0.0); 100];
//! let audio = rx.process(&samples);
//! assert_eq!(audio.len(), 100);
//! ```

use num_complex::Complex64;

/// Narrowband FM receiver.
///
/// Processes I/Q samples through:
/// 1. Quadrature demodulation (FM discriminator)
/// 2. De-emphasis filter (optional)
/// 3. Output gain normalization
#[derive(Debug, Clone)]
pub struct NbfmReceiver {
    /// Sample rate
    sample_rate: f64,
    /// Maximum FM deviation
    max_deviation: f64,
    /// FM discriminator gain
    gain: f64,
    /// Previous sample for discriminator
    prev: Complex64,
    /// De-emphasis alpha (single-pole IIR)
    deemph_alpha: f64,
    /// De-emphasis state
    deemph_state: f64,
    /// Whether de-emphasis is enabled
    deemph_enabled: bool,
}

impl NbfmReceiver {
    /// Create a narrowband FM receiver.
    ///
    /// - `sample_rate`: Input sample rate in Hz
    /// - `max_deviation`: Maximum FM deviation in Hz (typ. 5000 for NBFM)
    pub fn new(sample_rate: f64, max_deviation: f64) -> Self {
        let gain = sample_rate / (2.0 * std::f64::consts::PI * max_deviation);
        // De-emphasis: 750us time constant for narrowband
        let tau = 750e-6;
        let alpha = 1.0 / (1.0 + sample_rate * tau);
        Self {
            sample_rate,
            max_deviation,
            gain,
            prev: Complex64::new(1.0, 0.0),
            deemph_alpha: alpha,
            deemph_state: 0.0,
            deemph_enabled: true,
        }
    }

    /// Process a block of I/Q samples and return demodulated audio.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());

        for &sample in input {
            // Quadrature demodulation
            let product = sample * self.prev.conj();
            let demod = self.gain * product.arg();
            self.prev = sample;

            // De-emphasis
            let out = if self.deemph_enabled {
                self.deemph_state = self.deemph_alpha * demod
                    + (1.0 - self.deemph_alpha) * self.deemph_state;
                self.deemph_state
            } else {
                demod
            };

            output.push(out);
        }

        output
    }

    /// Enable or disable de-emphasis.
    pub fn set_deemph_enabled(&mut self, enabled: bool) {
        self.deemph_enabled = enabled;
    }

    /// Get the sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.prev = Complex64::new(1.0, 0.0);
        self.deemph_state = 0.0;
    }
}

/// Wideband FM receiver.
///
/// Processes I/Q samples through:
/// 1. Quadrature demodulation
/// 2. De-emphasis filter (75us US or 50us EU)
/// 3. Output gain normalization
#[derive(Debug, Clone)]
pub struct WbfmReceiver {
    /// Sample rate
    sample_rate: f64,
    /// Maximum FM deviation
    max_deviation: f64,
    /// FM discriminator gain
    gain: f64,
    /// Previous sample for discriminator
    prev: Complex64,
    /// De-emphasis alpha
    deemph_alpha: f64,
    /// De-emphasis state
    deemph_state: f64,
}

impl WbfmReceiver {
    /// Create a wideband FM receiver.
    ///
    /// - `sample_rate`: Input sample rate in Hz
    /// - `standard`: "US" (75us) or "EU" (50us) de-emphasis
    pub fn new(sample_rate: f64, standard: &str) -> Self {
        let max_deviation = 75_000.0; // WBFM deviation
        let gain = sample_rate / (2.0 * std::f64::consts::PI * max_deviation);
        let tau = match standard {
            "EU" | "50us" => 50e-6,
            _ => 75e-6, // US default
        };
        let alpha = 1.0 / (1.0 + sample_rate * tau);
        Self {
            sample_rate,
            max_deviation,
            gain,
            prev: Complex64::new(1.0, 0.0),
            deemph_alpha: alpha,
            deemph_state: 0.0,
        }
    }

    /// Process a block of I/Q samples and return demodulated audio.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());

        for &sample in input {
            let product = sample * self.prev.conj();
            let demod = self.gain * product.arg();
            self.prev = sample;

            // De-emphasis
            self.deemph_state = self.deemph_alpha * demod
                + (1.0 - self.deemph_alpha) * self.deemph_state;

            output.push(self.deemph_state);
        }

        output
    }

    /// Get the sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.prev = Complex64::new(1.0, 0.0);
        self.deemph_state = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_nbfm_dc_input() {
        let mut rx = NbfmReceiver::new(48000.0, 5000.0);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = rx.process(&input);
        assert_eq!(output.len(), 100);
        // DC input → zero FM output (no frequency change)
        for &s in &output[5..] {
            assert!(s.abs() < 0.1, "Expected ~0, got {}", s);
        }
    }

    #[test]
    fn test_nbfm_tone() {
        let mut rx = NbfmReceiver::new(48000.0, 5000.0);
        rx.set_deemph_enabled(false);
        let f_mod = 1000.0;
        let n = 1000;
        // Generate FM-modulated signal: exp(j * integral(2*pi*f_mod*sin(2*pi*f_audio*t)))
        let f_audio = 500.0;
        let input: Vec<Complex64> = (0..n).map(|i| {
            let t = i as f64 / 48000.0;
            let phase = 2.0 * PI * f_mod * t;
            Complex64::new(phase.cos(), phase.sin())
        }).collect();
        let output = rx.process(&input);
        assert_eq!(output.len(), n);
    }

    #[test]
    fn test_nbfm_deemph_toggle() {
        let mut rx = NbfmReceiver::new(48000.0, 5000.0);
        assert!(rx.deemph_enabled);
        rx.set_deemph_enabled(false);
        assert!(!rx.deemph_enabled);
    }

    #[test]
    fn test_wbfm_construction() {
        let rx = WbfmReceiver::new(256000.0, "US");
        assert!((rx.sample_rate() - 256000.0).abs() < 1e-10);
    }

    #[test]
    fn test_wbfm_dc_input() {
        let mut rx = WbfmReceiver::new(256000.0, "US");
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = rx.process(&input);
        assert_eq!(output.len(), 100);
        // DC → zero FM output
        for &s in &output[5..] {
            assert!(s.abs() < 0.1, "Expected ~0, got {}", s);
        }
    }

    #[test]
    fn test_wbfm_eu_standard() {
        let rx = WbfmReceiver::new(256000.0, "EU");
        // EU standard should have different de-emphasis alpha
        assert!(rx.deemph_alpha > 0.0 && rx.deemph_alpha < 1.0);
    }

    #[test]
    fn test_nbfm_reset() {
        let mut rx = NbfmReceiver::new(48000.0, 5000.0);
        let tone: Vec<Complex64> = (0..100).map(|i| {
            let phase = 2.0 * PI * 1000.0 * i as f64 / 48000.0;
            Complex64::new(phase.cos(), phase.sin())
        }).collect();
        rx.process(&tone);
        rx.reset();
        assert!((rx.deemph_state - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_output_length_matches() {
        let mut rx = NbfmReceiver::new(48000.0, 5000.0);
        let input = vec![Complex64::new(1.0, 0.0); 500];
        let output = rx.process(&input);
        assert_eq!(output.len(), 500);
    }
}
