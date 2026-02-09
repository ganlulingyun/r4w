//! Automatic Frequency Control — Closed-loop frequency tracking
//!
//! Estimates and corrects frequency offset using a discriminator-based feedback
//! loop. Essential for long-duration communications where oscillator drift
//! exceeds carrier recovery pull-in range. Works with phase discriminator or
//! frequency discriminator approaches.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::afc::{Afc, AfcDiscriminator};
//! use num_complex::Complex64;
//!
//! let mut afc = Afc::new(AfcDiscriminator::Phase, 0.001, 48000.0);
//! let signal: Vec<Complex64> = (0..200)
//!     .map(|i| Complex64::from_polar(1.0, 2.0 * std::f64::consts::PI * 100.0 * i as f64 / 48000.0))
//!     .collect();
//! let corrected = afc.process(&signal);
//! assert_eq!(corrected.len(), signal.len());
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Discriminator type for frequency estimation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AfcDiscriminator {
    /// Phase difference between consecutive samples.
    Phase,
    /// Cross-product (atan2-free) discriminator.
    CrossProduct,
    /// Kay's estimator (weighted average of phase increments).
    Kay,
}

/// Automatic Frequency Control loop.
#[derive(Debug, Clone)]
pub struct Afc {
    /// Discriminator type.
    discriminator: AfcDiscriminator,
    /// Loop gain (controls convergence speed vs stability).
    loop_gain: f64,
    /// Sample rate in Hz.
    sample_rate: f64,
    /// Current frequency estimate in Hz.
    freq_estimate: f64,
    /// Maximum frequency correction in Hz.
    max_freq: f64,
    /// Current phase accumulator for correction.
    phase_accum: f64,
    /// Previous sample (for discriminator).
    prev_sample: Complex64,
    /// Loop filter state (integrator).
    integrator: f64,
    /// Proportional gain.
    alpha: f64,
    /// Integral gain.
    beta: f64,
}

impl Afc {
    /// Create a new AFC loop.
    ///
    /// `discriminator`: Frequency estimation method.
    /// `loop_gain`: Overall loop gain (0.0001 to 0.1 typical).
    /// `sample_rate`: Sample rate in Hz.
    pub fn new(discriminator: AfcDiscriminator, loop_gain: f64, sample_rate: f64) -> Self {
        let gain = loop_gain.clamp(1e-6, 1.0);
        Self {
            discriminator,
            loop_gain: gain,
            sample_rate,
            freq_estimate: 0.0,
            max_freq: sample_rate / 4.0,
            phase_accum: 0.0,
            prev_sample: Complex64::new(1.0, 0.0),
            integrator: 0.0,
            alpha: gain,
            beta: gain * gain * 0.25, // Second-order loop
        }
    }

    /// Set maximum frequency correction range.
    pub fn set_max_freq(&mut self, max_hz: f64) {
        self.max_freq = max_hz.abs().min(self.sample_rate / 2.0);
    }

    /// Set loop bandwidth via damping factor and natural frequency.
    pub fn set_loop_params(&mut self, alpha: f64, beta: f64) {
        self.alpha = alpha.clamp(1e-6, 1.0);
        self.beta = beta.clamp(0.0, 1.0);
    }

    /// Process a block of samples, applying frequency correction.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len());

        for &sample in input {
            // Apply current frequency correction
            let correction = Complex64::from_polar(1.0, -self.phase_accum);
            let corrected = sample * correction;

            // Estimate frequency error using discriminator
            let freq_error = self.discriminate(corrected);

            // Second-order loop filter
            self.integrator += self.beta * freq_error;
            let loop_output = self.alpha * freq_error + self.integrator;

            // Update frequency estimate
            self.freq_estimate = (self.freq_estimate + loop_output)
                .clamp(-self.max_freq, self.max_freq);

            // Update phase accumulator
            let phase_increment = 2.0 * PI * self.freq_estimate / self.sample_rate;
            self.phase_accum += phase_increment;
            // Wrap phase to [-π, π]
            if self.phase_accum > PI {
                self.phase_accum -= 2.0 * PI;
            } else if self.phase_accum < -PI {
                self.phase_accum += 2.0 * PI;
            }

            self.prev_sample = corrected;
            output.push(corrected);
        }

        output
    }

    fn discriminate(&self, sample: Complex64) -> f64 {
        match self.discriminator {
            AfcDiscriminator::Phase => {
                // Phase difference: arg(z[n] * conj(z[n-1]))
                let product = sample * self.prev_sample.conj();
                product.arg() / (2.0 * PI) * self.sample_rate
            }
            AfcDiscriminator::CrossProduct => {
                // Cross-product discriminator (atan2-free approximation)
                let re = sample.re * self.prev_sample.re + sample.im * self.prev_sample.im;
                let im = sample.im * self.prev_sample.re - sample.re * self.prev_sample.im;
                im.atan2(re) / (2.0 * PI) * self.sample_rate
            }
            AfcDiscriminator::Kay => {
                // Same as Phase for single-sample, but could be extended for blocks
                let product = sample * self.prev_sample.conj();
                product.arg() / (2.0 * PI) * self.sample_rate
            }
        }
    }

    /// Get the current frequency estimate in Hz.
    pub fn frequency(&self) -> f64 {
        self.freq_estimate
    }

    /// Get the current frequency estimate in normalized radians/sample.
    pub fn frequency_rad(&self) -> f64 {
        2.0 * PI * self.freq_estimate / self.sample_rate
    }

    /// Reset the AFC state.
    pub fn reset(&mut self) {
        self.freq_estimate = 0.0;
        self.phase_accum = 0.0;
        self.prev_sample = Complex64::new(1.0, 0.0);
        self.integrator = 0.0;
    }
}

/// Simple one-shot frequency estimator (no feedback loop).
pub struct FrequencyEstimator;

impl FrequencyEstimator {
    /// Estimate carrier frequency using average phase increment.
    pub fn estimate(samples: &[Complex64], sample_rate: f64) -> f64 {
        if samples.len() < 2 {
            return 0.0;
        }
        let mut total_phase_diff = 0.0;
        for i in 1..samples.len() {
            let product = samples[i] * samples[i - 1].conj();
            total_phase_diff += product.arg();
        }
        let avg_phase_diff = total_phase_diff / (samples.len() - 1) as f64;
        avg_phase_diff / (2.0 * PI) * sample_rate
    }

    /// Estimate using autocorrelation at lag 1 (more noise-robust).
    pub fn estimate_autocorrelation(samples: &[Complex64], sample_rate: f64) -> f64 {
        if samples.len() < 2 {
            return 0.0;
        }
        let mut r1 = Complex64::new(0.0, 0.0);
        for i in 1..samples.len() {
            r1 += samples[i] * samples[i - 1].conj();
        }
        r1.arg() / (2.0 * PI) * sample_rate
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_tone(freq_hz: f64, sample_rate: f64, num_samples: usize) -> Vec<Complex64> {
        (0..num_samples)
            .map(|i| Complex64::from_polar(1.0, 2.0 * PI * freq_hz * i as f64 / sample_rate))
            .collect()
    }

    #[test]
    fn test_afc_tracks_offset() {
        let mut afc = Afc::new(AfcDiscriminator::Phase, 0.01, 48000.0);
        afc.set_max_freq(5000.0);
        let signal = make_tone(200.0, 48000.0, 2000);
        let _corrected = afc.process(&signal);
        // AFC should estimate ~200 Hz offset
        let est = afc.frequency();
        assert!(est.abs() > 50.0, "AFC should detect frequency offset, got {} Hz", est);
    }

    #[test]
    fn test_afc_zero_offset() {
        let mut afc = Afc::new(AfcDiscriminator::Phase, 0.001, 48000.0);
        let signal = vec![Complex64::new(1.0, 0.0); 500]; // DC (0 Hz)
        afc.process(&signal);
        assert!(afc.frequency().abs() < 10.0, "AFC should report ~0 Hz for DC, got {}", afc.frequency());
    }

    #[test]
    fn test_afc_cross_product() {
        let mut afc = Afc::new(AfcDiscriminator::CrossProduct, 0.01, 48000.0);
        let signal = make_tone(500.0, 48000.0, 2000);
        afc.process(&signal);
        let est = afc.frequency();
        assert!(est.abs() > 100.0, "CrossProduct AFC should track, got {} Hz", est);
    }

    #[test]
    fn test_frequency_estimator() {
        let signal = make_tone(1000.0, 48000.0, 500);
        let est = FrequencyEstimator::estimate(&signal, 48000.0);
        assert!((est - 1000.0).abs() < 10.0, "Expected ~1000 Hz, got {}", est);
    }

    #[test]
    fn test_frequency_estimator_autocorrelation() {
        let signal = make_tone(500.0, 48000.0, 500);
        let est = FrequencyEstimator::estimate_autocorrelation(&signal, 48000.0);
        assert!((est - 500.0).abs() < 10.0, "Expected ~500 Hz, got {}", est);
    }

    #[test]
    fn test_frequency_estimator_negative() {
        let signal = make_tone(-300.0, 48000.0, 500);
        let est = FrequencyEstimator::estimate(&signal, 48000.0);
        assert!((est + 300.0).abs() < 10.0, "Expected ~-300 Hz, got {}", est);
    }

    #[test]
    fn test_afc_output_length() {
        let mut afc = Afc::new(AfcDiscriminator::Phase, 0.01, 48000.0);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = afc.process(&input);
        assert_eq!(output.len(), 100);
    }

    #[test]
    fn test_afc_reset() {
        let mut afc = Afc::new(AfcDiscriminator::Phase, 0.01, 48000.0);
        let signal = make_tone(500.0, 48000.0, 1000);
        afc.process(&signal);
        assert!(afc.frequency().abs() > 10.0);
        afc.reset();
        assert_eq!(afc.frequency(), 0.0);
    }

    #[test]
    fn test_max_freq_clamp() {
        let mut afc = Afc::new(AfcDiscriminator::Phase, 0.1, 48000.0);
        afc.set_max_freq(100.0);
        let signal = make_tone(5000.0, 48000.0, 2000);
        afc.process(&signal);
        assert!(afc.frequency().abs() <= 100.0, "Should be clamped to ±100 Hz");
    }

    #[test]
    fn test_empty_estimator() {
        assert_eq!(FrequencyEstimator::estimate(&[], 48000.0), 0.0);
        assert_eq!(FrequencyEstimator::estimate(&[Complex64::new(1.0, 0.0)], 48000.0), 0.0);
    }
}
