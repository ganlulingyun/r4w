//! Frequency Rotator
//!
//! NCO-based complex multiplier that shifts a signal by a programmable
//! frequency offset. Essential for carrier frequency correction, channel
//! selection, and baseband conversion.
//!
//! ## Algorithm
//!
//! ```text
//! y[n] = x[n] * exp(j * (2π * f_shift/f_s * n + φ))
//! ```
//!
//! Uses a numerically-controlled oscillator (NCO) with phase accumulator
//! for phase-continuous frequency shifting.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::rotator::Rotator;
//! use num_complex::Complex64;
//!
//! // Shift signal down by 1 kHz at 48 kHz sample rate
//! let mut rot = Rotator::new(-1000.0, 48000.0);
//! let input = vec![Complex64::new(1.0, 0.0); 100];
//! let output = rot.process(&input);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// NCO-based frequency rotator.
#[derive(Debug, Clone)]
pub struct Rotator {
    /// Phase increment per sample (radians)
    phase_inc: f64,
    /// Current phase (radians)
    phase: f64,
    /// Sample rate (Hz)
    sample_rate: f64,
    /// Frequency shift (Hz)
    frequency: f64,
}

impl Rotator {
    /// Create a new rotator.
    ///
    /// - `frequency_hz`: Frequency shift in Hz (positive = up, negative = down)
    /// - `sample_rate_hz`: Sample rate in Hz
    pub fn new(frequency_hz: f64, sample_rate_hz: f64) -> Self {
        let phase_inc = 2.0 * PI * frequency_hz / sample_rate_hz;
        Self {
            phase_inc,
            phase: 0.0,
            sample_rate: sample_rate_hz,
            frequency: frequency_hz,
        }
    }

    /// Process a block of complex samples.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len());
        for &sample in input {
            let rotator = Complex64::new(self.phase.cos(), self.phase.sin());
            output.push(sample * rotator);
            self.phase += self.phase_inc;
            // Wrap phase to prevent loss of precision
            if self.phase > PI {
                self.phase -= 2.0 * PI;
            } else if self.phase < -PI {
                self.phase += 2.0 * PI;
            }
        }
        output
    }

    /// Process a single sample.
    pub fn process_one(&mut self, sample: Complex64) -> Complex64 {
        let rotator = Complex64::new(self.phase.cos(), self.phase.sin());
        let result = sample * rotator;
        self.phase += self.phase_inc;
        if self.phase > PI {
            self.phase -= 2.0 * PI;
        } else if self.phase < -PI {
            self.phase += 2.0 * PI;
        }
        result
    }

    /// Set frequency shift (Hz).
    pub fn set_frequency(&mut self, frequency_hz: f64) {
        self.frequency = frequency_hz;
        self.phase_inc = 2.0 * PI * frequency_hz / self.sample_rate;
    }

    /// Get current frequency shift (Hz).
    pub fn frequency(&self) -> f64 {
        self.frequency
    }

    /// Get current phase (radians).
    pub fn phase(&self) -> f64 {
        self.phase
    }

    /// Set phase directly (radians).
    pub fn set_phase(&mut self, phase: f64) {
        self.phase = phase;
    }

    /// Reset phase to zero.
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zero_shift() {
        let mut rot = Rotator::new(0.0, 48000.0);
        let input = vec![Complex64::new(1.0, 0.0); 10];
        let output = rot.process(&input);
        for (i, &s) in output.iter().enumerate() {
            assert!(
                (s.re - 1.0).abs() < 1e-10 && s.im.abs() < 1e-10,
                "Sample {} should be 1+0j, got {:?}",
                i, s
            );
        }
    }

    #[test]
    fn test_quarter_sample_rate_shift() {
        // Shift by fs/4 → 90° per sample
        let mut rot = Rotator::new(12000.0, 48000.0);
        let input = vec![Complex64::new(1.0, 0.0); 4];
        let output = rot.process(&input);

        // Sample 0: phase=0 → 1+0j
        assert!((output[0].re - 1.0).abs() < 1e-10);
        // Sample 1: phase=π/2 → 0+1j
        assert!(output[1].re.abs() < 1e-10);
        assert!((output[1].im - 1.0).abs() < 1e-10);
        // Sample 2: phase=π → -1+0j
        assert!((output[2].re + 1.0).abs() < 1e-10);
        // Sample 3: phase=3π/2 → 0-1j
        assert!(output[3].re.abs() < 1e-10);
        assert!((output[3].im + 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_negative_shift() {
        let mut rot = Rotator::new(-12000.0, 48000.0);
        let input = vec![Complex64::new(1.0, 0.0); 4];
        let output = rot.process(&input);

        // Sample 1: phase=-π/2 → 0-1j
        assert!(output[1].re.abs() < 1e-10);
        assert!((output[1].im + 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_magnitude_preservation() {
        let mut rot = Rotator::new(1000.0, 48000.0);
        let input: Vec<Complex64> = (0..100)
            .map(|i| Complex64::new((i as f64 * 0.1).cos(), (i as f64 * 0.1).sin()))
            .collect();
        let output = rot.process(&input);

        for (i, (&inp, &out)) in input.iter().zip(output.iter()).enumerate() {
            let mag_diff = (inp.norm() - out.norm()).abs();
            assert!(
                mag_diff < 1e-10,
                "Sample {} magnitude changed: {} → {}",
                i, inp.norm(), out.norm()
            );
        }
    }

    #[test]
    fn test_phase_wrapping() {
        let mut rot = Rotator::new(1000.0, 48000.0);
        let input = vec![Complex64::new(1.0, 0.0); 100000];
        rot.process(&input);
        // Phase should stay bounded
        assert!(rot.phase().abs() <= PI + 0.01);
    }

    #[test]
    fn test_set_frequency() {
        let mut rot = Rotator::new(1000.0, 48000.0);
        assert_eq!(rot.frequency(), 1000.0);
        rot.set_frequency(2000.0);
        assert_eq!(rot.frequency(), 2000.0);
    }

    #[test]
    fn test_process_one() {
        let mut rot1 = Rotator::new(1000.0, 48000.0);
        let mut rot2 = Rotator::new(1000.0, 48000.0);
        let input = Complex64::new(1.0, 0.5);

        let single = rot1.process_one(input);
        let block = rot2.process(&[input]);

        assert!((single.re - block[0].re).abs() < 1e-10);
        assert!((single.im - block[0].im).abs() < 1e-10);
    }

    #[test]
    fn test_reset() {
        let mut rot = Rotator::new(1000.0, 48000.0);
        rot.process(&vec![Complex64::new(1.0, 0.0); 50]);
        assert!(rot.phase().abs() > 0.01);
        rot.reset();
        assert_eq!(rot.phase(), 0.0);
    }
}
