//! Hardware Impairment Simulator
//!
//! Models real-world SDR hardware imperfections: phase noise, IQ imbalance,
//! and DC offset. Useful for realistic receiver testing and education.
//!
//! ## Blocks
//!
//! - **PhaseNoiseGenerator**: Adds random phase jitter (colored noise)
//! - **IqImbalanceGenerator**: Models gain/phase mismatch between I and Q paths
//! - **DcOffset**: Adds constant DC bias to I and/or Q
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::hw_impairments::{PhaseNoiseGenerator, IqImbalanceGenerator, DcOffset};
//! use num_complex::Complex64;
//!
//! let mut pn = PhaseNoiseGenerator::new(-30.0, 0.9);
//! let signal = vec![Complex64::new(1.0, 0.0); 100];
//! let noisy = pn.process(&signal);
//! // Phase noise preserves envelope approximately
//! assert!((noisy[0].norm() - 1.0).abs() < 0.01);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Phase noise generator — adds filtered random phase jitter.
///
/// Models oscillator phase noise as a first-order IIR-filtered random walk.
/// `alpha` controls spectral shape: 0 = white phase noise, near 1 = 1/f-like.
#[derive(Debug, Clone)]
pub struct PhaseNoiseGenerator {
    /// Noise magnitude in dB (relative to carrier)
    noise_magnitude_db: f64,
    /// Linear noise magnitude
    noise_linear: f64,
    /// IIR filter coefficient (0..1), higher = more low-frequency content
    alpha: f64,
    /// IIR filter state
    iir_state: f64,
    /// Simple PRNG state (xorshift64)
    rng_state: u64,
}

impl PhaseNoiseGenerator {
    /// Create a new phase noise generator.
    ///
    /// - `noise_magnitude_db`: Phase noise level in dBc (typically -20 to -60)
    /// - `alpha`: IIR coefficient for spectral shaping (0..1)
    pub fn new(noise_magnitude_db: f64, alpha: f64) -> Self {
        Self {
            noise_magnitude_db,
            noise_linear: 10.0_f64.powf(noise_magnitude_db / 20.0),
            alpha: alpha.clamp(0.0, 0.9999),
            iir_state: 0.0,
            rng_state: 0xDEADBEEF_CAFEBABE,
        }
    }

    /// Apply phase noise to a block of samples.
    pub fn process(&mut self, samples: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(samples.len());
        for &s in samples {
            // Generate white noise sample
            let white = self.next_gaussian() * self.noise_linear;
            // Filter through single-pole IIR
            self.iir_state = self.alpha * self.iir_state + (1.0 - self.alpha) * white;
            // Apply phase rotation
            let phase = self.iir_state;
            let rot = Complex64::new(phase.cos(), phase.sin());
            output.push(s * rot);
        }
        output
    }

    /// Set noise magnitude in dB.
    pub fn set_magnitude(&mut self, mag_db: f64) {
        self.noise_magnitude_db = mag_db;
        self.noise_linear = 10.0_f64.powf(mag_db / 20.0);
    }

    /// Get noise magnitude in dB.
    pub fn magnitude_db(&self) -> f64 {
        self.noise_magnitude_db
    }

    fn next_gaussian(&mut self) -> f64 {
        // Box-Muller using xorshift64
        let u1 = self.next_uniform();
        let u2 = self.next_uniform();
        (-2.0 * u1.max(1e-30).ln()).sqrt() * (2.0 * PI * u2).cos()
    }

    fn next_uniform(&mut self) -> f64 {
        self.rng_state ^= self.rng_state << 13;
        self.rng_state ^= self.rng_state >> 7;
        self.rng_state ^= self.rng_state << 17;
        (self.rng_state as f64) / (u64::MAX as f64)
    }

    pub fn reset(&mut self) {
        self.iir_state = 0.0;
        self.rng_state = 0xDEADBEEF_CAFEBABE;
    }
}

/// IQ imbalance generator — models gain and phase mismatch.
///
/// In a real receiver, the I and Q analog paths may have slightly different
/// gains and imperfect 90° phase splitting. This creates an image of the
/// desired signal at the mirror frequency.
#[derive(Debug, Clone)]
pub struct IqImbalanceGenerator {
    /// Gain mismatch in dB
    magnitude_db: f64,
    /// Phase error in degrees
    phase_deg: f64,
    /// Pre-computed gain factor
    gain: f64,
    /// Pre-computed phase error in radians
    phase_rad: f64,
}

impl IqImbalanceGenerator {
    /// Create a new IQ imbalance generator.
    ///
    /// - `magnitude_db`: Gain mismatch (I vs Q) in dB. 0 = no gain error.
    /// - `phase_deg`: Phase error in degrees. 0 = perfect quadrature.
    pub fn new(magnitude_db: f64, phase_deg: f64) -> Self {
        Self {
            magnitude_db,
            phase_deg,
            gain: 10.0_f64.powf(magnitude_db / 20.0),
            phase_rad: phase_deg * PI / 180.0,
        }
    }

    /// Apply IQ imbalance to samples.
    ///
    /// Model: I' = I, Q' = gain * (Q * cos(φ) + I * sin(φ))
    pub fn process(&self, samples: &[Complex64]) -> Vec<Complex64> {
        let cos_p = self.phase_rad.cos();
        let sin_p = self.phase_rad.sin();
        samples
            .iter()
            .map(|s| {
                let i = s.re;
                let q = self.gain * (s.im * cos_p + s.re * sin_p);
                Complex64::new(i, q)
            })
            .collect()
    }

    pub fn magnitude_db(&self) -> f64 {
        self.magnitude_db
    }

    pub fn phase_deg(&self) -> f64 {
        self.phase_deg
    }
}

/// DC offset — adds constant bias to I and/or Q channels.
#[derive(Debug, Clone)]
pub struct DcOffset {
    /// DC offset on I channel
    pub i_offset: f64,
    /// DC offset on Q channel
    pub q_offset: f64,
}

impl DcOffset {
    pub fn new(i_offset: f64, q_offset: f64) -> Self {
        Self { i_offset, q_offset }
    }

    /// Apply DC offset to samples.
    pub fn process(&self, samples: &[Complex64]) -> Vec<Complex64> {
        samples
            .iter()
            .map(|s| Complex64::new(s.re + self.i_offset, s.im + self.q_offset))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_phase_noise_preserves_envelope() {
        let mut pn = PhaseNoiseGenerator::new(-30.0, 0.9);
        let signal = vec![Complex64::new(1.0, 0.0); 1000];
        let noisy = pn.process(&signal);
        for z in &noisy {
            assert!((z.norm() - 1.0).abs() < 0.01, "Phase noise should preserve envelope");
        }
    }

    #[test]
    fn test_phase_noise_zero_magnitude() {
        let mut pn = PhaseNoiseGenerator::new(-200.0, 0.0); // Effectively zero noise
        let signal = vec![Complex64::new(1.0, 0.0); 100];
        let noisy = pn.process(&signal);
        for z in &noisy {
            assert!((z.re - 1.0).abs() < 0.01);
        }
    }

    #[test]
    fn test_phase_noise_reset() {
        let mut pn = PhaseNoiseGenerator::new(-20.0, 0.5);
        let signal = vec![Complex64::new(1.0, 0.0); 10];
        let out1 = pn.process(&signal);
        pn.reset();
        let out2 = pn.process(&signal);
        assert_eq!(out1, out2, "Reset should produce identical output");
    }

    #[test]
    fn test_iq_imbalance_identity() {
        let iq = IqImbalanceGenerator::new(0.0, 0.0); // No imbalance
        let signal = vec![Complex64::new(0.5, 0.5), Complex64::new(1.0, -1.0)];
        let output = iq.process(&signal);
        for (i, o) in signal.iter().zip(output.iter()) {
            assert!((i.re - o.re).abs() < 1e-10);
            assert!((i.im - o.im).abs() < 1e-10);
        }
    }

    #[test]
    fn test_iq_imbalance_gain_only() {
        let iq = IqImbalanceGenerator::new(6.0, 0.0); // ~2x gain on Q
        let signal = vec![Complex64::new(0.0, 1.0)]; // Pure Q
        let output = iq.process(&signal);
        let expected_gain = 10.0_f64.powf(6.0 / 20.0); // ~1.995
        assert!((output[0].im - expected_gain).abs() < 0.01);
        assert!(output[0].re.abs() < 1e-10); // I unchanged
    }

    #[test]
    fn test_iq_imbalance_phase_only() {
        let iq = IqImbalanceGenerator::new(0.0, 5.0); // 5° phase error
        let signal = vec![Complex64::new(1.0, 0.0)];
        let output = iq.process(&signal);
        // I should be unchanged, Q should have some leakage from I
        assert!((output[0].re - 1.0).abs() < 1e-10);
        let expected_q = (5.0_f64 * PI / 180.0).sin(); // sin(5°) ≈ 0.087
        assert!((output[0].im - expected_q).abs() < 0.001);
    }

    #[test]
    fn test_dc_offset_basic() {
        let dc = DcOffset::new(0.1, -0.2);
        let signal = vec![Complex64::new(1.0, 1.0), Complex64::new(0.0, 0.0)];
        let output = dc.process(&signal);
        assert!((output[0].re - 1.1).abs() < 1e-10);
        assert!((output[0].im - 0.8).abs() < 1e-10);
        assert!((output[1].re - 0.1).abs() < 1e-10);
        assert!((output[1].im + 0.2).abs() < 1e-10);
    }

    #[test]
    fn test_dc_offset_zero() {
        let dc = DcOffset::new(0.0, 0.0);
        let signal = vec![Complex64::new(3.0, -4.0)];
        let output = dc.process(&signal);
        assert_eq!(output[0], signal[0]);
    }

    #[test]
    fn test_phase_noise_set_magnitude() {
        let mut pn = PhaseNoiseGenerator::new(-30.0, 0.5);
        assert!((pn.magnitude_db() - (-30.0)).abs() < 1e-10);
        pn.set_magnitude(-60.0);
        assert!((pn.magnitude_db() - (-60.0)).abs() < 1e-10);
    }

    #[test]
    fn test_iq_imbalance_large_error() {
        let iq = IqImbalanceGenerator::new(3.0, 10.0); // Significant imbalance
        let signal = vec![Complex64::new(1.0, 1.0); 100];
        let output = iq.process(&signal);
        // All outputs should be finite and non-zero
        for z in &output {
            assert!(z.re.is_finite());
            assert!(z.im.is_finite());
        }
    }

    #[test]
    fn test_phase_noise_empty_input() {
        let mut pn = PhaseNoiseGenerator::new(-30.0, 0.9);
        assert!(pn.process(&[]).is_empty());
    }
}
