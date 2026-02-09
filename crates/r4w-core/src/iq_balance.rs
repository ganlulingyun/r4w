//! IQ Balance — IQ imbalance estimation and correction
//!
//! Corrects gain and phase imbalance between I and Q channels in
//! a quadrature receiver. Imperfect mixers and ADCs introduce
//! amplitude mismatch and phase skew that degrade image rejection.
//! GNU Radio equivalent: `iq_balance_cc`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::iq_balance::{IqBalanceCorrector, estimate_iq_imbalance};
//! use num_complex::Complex64;
//!
//! // Measure imbalance on a test signal
//! let samples: Vec<Complex64> = (0..1000)
//!     .map(|i| {
//!         let t = i as f64 / 1000.0;
//!         Complex64::new(1.1 * (6.28 * t).cos(), 0.9 * (6.28 * t).sin())
//!     })
//!     .collect();
//! let (gain_err, phase_err) = estimate_iq_imbalance(&samples);
//! // Apply correction
//! let mut corrector = IqBalanceCorrector::new(gain_err, phase_err);
//! let corrected = corrector.process(&samples);
//! ```

use num_complex::Complex64;

/// IQ imbalance corrector.
///
/// Applies gain and phase correction to compensate for analog
/// front-end imperfections.
#[derive(Debug, Clone)]
pub struct IqBalanceCorrector {
    /// Gain correction factor for Q channel.
    gain: f64,
    /// Phase correction (sin of phase error).
    phase_sin: f64,
}

impl IqBalanceCorrector {
    /// Create a corrector from estimated imbalance.
    ///
    /// - `gain_imbalance`: ratio of I gain to Q gain (1.0 = balanced)
    /// - `phase_imbalance`: phase error in radians (0.0 = balanced)
    pub fn new(gain_imbalance: f64, phase_imbalance: f64) -> Self {
        Self {
            gain: if gain_imbalance.abs() > 1e-12 {
                1.0 / gain_imbalance
            } else {
                1.0
            },
            phase_sin: phase_imbalance.sin(),
        }
    }

    /// Correct a single sample.
    #[inline]
    pub fn correct(&self, sample: Complex64) -> Complex64 {
        let i = sample.re;
        let q = sample.im * self.gain - i * self.phase_sin;
        Complex64::new(i, q)
    }

    /// Correct a block of samples.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.correct(s)).collect()
    }

    /// Correct in-place.
    pub fn process_inplace(&self, data: &mut [Complex64]) {
        for s in data.iter_mut() {
            *s = self.correct(*s);
        }
    }

    /// Update correction parameters.
    pub fn set_imbalance(&mut self, gain_imbalance: f64, phase_imbalance: f64) {
        self.gain = if gain_imbalance.abs() > 1e-12 {
            1.0 / gain_imbalance
        } else {
            1.0
        };
        self.phase_sin = phase_imbalance.sin();
    }
}

/// Estimate IQ gain and phase imbalance from a complex signal.
///
/// Returns `(gain_ratio, phase_error_rad)`.
/// - gain_ratio: I_rms / Q_rms (1.0 = balanced)
/// - phase_error: estimated phase skew in radians
pub fn estimate_iq_imbalance(samples: &[Complex64]) -> (f64, f64) {
    if samples.is_empty() {
        return (1.0, 0.0);
    }
    let n = samples.len() as f64;

    // RMS of I and Q channels
    let i_power: f64 = samples.iter().map(|s| s.re * s.re).sum::<f64>() / n;
    let q_power: f64 = samples.iter().map(|s| s.im * s.im).sum::<f64>() / n;

    let gain_ratio = if q_power > 1e-30 {
        (i_power / q_power).sqrt()
    } else {
        1.0
    };

    // Estimate phase error from cross-correlation of I and Q
    let cross: f64 = samples.iter().map(|s| s.re * s.im).sum::<f64>() / n;
    let phase_error = if i_power > 1e-30 && q_power > 1e-30 {
        (cross / (i_power * q_power).sqrt()).asin()
    } else {
        0.0
    };

    (gain_ratio, phase_error)
}

/// Adaptive IQ balance corrector using LMS.
#[derive(Debug, Clone)]
pub struct AdaptiveIqBalance {
    /// Current gain correction.
    gain: f64,
    /// Current phase correction.
    phase: f64,
    /// Learning rate.
    mu: f64,
}

impl AdaptiveIqBalance {
    /// Create an adaptive IQ balancer.
    ///
    /// `mu`: learning rate (0.0001 to 0.01 typical)
    pub fn new(mu: f64) -> Self {
        Self {
            gain: 1.0,
            phase: 0.0,
            mu: mu.clamp(0.0, 1.0),
        }
    }

    /// Process and correct a single sample, adapting the correction.
    #[inline]
    pub fn process_sample(&mut self, sample: Complex64) -> Complex64 {
        // Apply current correction
        let i = sample.re;
        let q = sample.im * self.gain - i * self.phase;

        let corrected = Complex64::new(i, q);

        // Update: minimize E[I² - Q²] and E[I*Q]
        let error_gain = i * i - q * q;
        let error_phase = i * q;
        self.gain -= self.mu * error_gain * sample.im;
        self.phase -= self.mu * error_phase;

        corrected
    }

    /// Process a block.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process_sample(s)).collect()
    }

    /// Get current estimated gain correction.
    pub fn gain(&self) -> f64 {
        self.gain
    }

    /// Get current estimated phase correction.
    pub fn phase(&self) -> f64 {
        self.phase
    }

    /// Reset.
    pub fn reset(&mut self) {
        self.gain = 1.0;
        self.phase = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn make_test_signal(n: usize, gain_err: f64, phase_err: f64) -> Vec<Complex64> {
        (0..n)
            .map(|i| {
                let t = i as f64 / n as f64;
                let phase = 2.0 * PI * 3.0 * t;
                Complex64::new(
                    gain_err * phase.cos(),
                    (phase + phase_err).sin(),
                )
            })
            .collect()
    }

    #[test]
    fn test_no_correction_needed() {
        let samples = make_test_signal(1000, 1.0, 0.0);
        let (gain, phase) = estimate_iq_imbalance(&samples);
        assert!((gain - 1.0).abs() < 0.1);
        assert!(phase.abs() < 0.1);
    }

    #[test]
    fn test_gain_imbalance_detection() {
        let samples = make_test_signal(10000, 1.2, 0.0);
        let (gain, _) = estimate_iq_imbalance(&samples);
        assert!((gain - 1.2).abs() < 0.15);
    }

    #[test]
    fn test_corrector_identity() {
        let mut corrector = IqBalanceCorrector::new(1.0, 0.0);
        let input = vec![Complex64::new(1.0, 1.0), Complex64::new(-1.0, -1.0)];
        let output = corrector.process(&input);
        assert!((output[0].re - 1.0).abs() < 1e-10);
        assert!((output[0].im - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_corrector_gain_only() {
        // Q channel is 2x too loud → gain_imbalance = 0.5
        let corrector = IqBalanceCorrector::new(0.5, 0.0);
        let sample = Complex64::new(1.0, 2.0);
        let corrected = corrector.correct(sample);
        assert!((corrected.re - 1.0).abs() < 1e-10);
        assert!((corrected.im - 4.0).abs() < 1e-10); // 2.0 * (1/0.5)
    }

    #[test]
    fn test_corrector_inplace() {
        let corrector = IqBalanceCorrector::new(1.0, 0.0);
        let mut data = vec![Complex64::new(1.0, 2.0)];
        corrector.process_inplace(&mut data);
        assert!((data[0].re - 1.0).abs() < 1e-10);
        assert!((data[0].im - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_set_imbalance() {
        let mut corrector = IqBalanceCorrector::new(1.0, 0.0);
        corrector.set_imbalance(2.0, 0.1);
        let s = corrector.correct(Complex64::new(1.0, 1.0));
        assert!(s.im != 1.0); // Should be corrected
    }

    #[test]
    fn test_estimate_empty() {
        let (gain, phase) = estimate_iq_imbalance(&[]);
        assert_eq!(gain, 1.0);
        assert_eq!(phase, 0.0);
    }

    #[test]
    fn test_adaptive_basic() {
        let mut iq = AdaptiveIqBalance::new(0.001);
        // Feed some samples — just verify it doesn't crash
        for i in 0..100 {
            let t = i as f64 / 100.0;
            let s = Complex64::new((2.0 * PI * t).cos(), 1.2 * (2.0 * PI * t).sin());
            iq.process_sample(s);
        }
        // Gain should have moved from 1.0
        assert!(iq.gain() != 1.0 || iq.phase() != 0.0);
    }

    #[test]
    fn test_adaptive_reset() {
        let mut iq = AdaptiveIqBalance::new(0.01);
        iq.process_sample(Complex64::new(1.0, 2.0));
        iq.reset();
        assert_eq!(iq.gain(), 1.0);
        assert_eq!(iq.phase(), 0.0);
    }

    #[test]
    fn test_adaptive_block() {
        let mut iq = AdaptiveIqBalance::new(0.001);
        let input: Vec<Complex64> = (0..50)
            .map(|i| Complex64::new(i as f64 * 0.1, i as f64 * 0.15))
            .collect();
        let output = iq.process(&input);
        assert_eq!(output.len(), 50);
    }

    #[test]
    fn test_balanced_signal() {
        // A perfectly balanced signal should need minimal correction
        let samples: Vec<Complex64> = (0..1000)
            .map(|i| {
                let t = i as f64 / 1000.0;
                Complex64::new((2.0 * PI * 5.0 * t).cos(), (2.0 * PI * 5.0 * t).sin())
            })
            .collect();
        let (gain, phase) = estimate_iq_imbalance(&samples);
        assert!((gain - 1.0).abs() < 0.05, "gain={gain}");
        assert!(phase.abs() < 0.1, "phase={phase}");
    }
}
