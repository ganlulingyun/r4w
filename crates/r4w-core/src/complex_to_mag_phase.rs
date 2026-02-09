//! ComplexToMagPhase — Simultaneous magnitude and phase extraction
//!
//! Converts complex samples to paired magnitude and phase streams in a
//! single pass. More efficient than running separate mag/phase extractors.
//! Also provides MagPhaseToComplex for the inverse operation.
//! GNU Radio equivalent: `complex_to_mag_phase`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::complex_to_mag_phase::{ComplexToMagPhase, MagPhaseToComplex};
//! use num_complex::Complex64;
//!
//! let c2mp = ComplexToMagPhase;
//! let input = vec![Complex64::new(3.0, 4.0)]; // mag=5, phase=atan2(4,3)
//! let (mags, phases) = c2mp.process(&input);
//! assert!((mags[0] - 5.0).abs() < 1e-10);
//!
//! // Inverse: convert back
//! let mp2c = MagPhaseToComplex;
//! let reconstructed = mp2c.process(&mags, &phases);
//! assert!((reconstructed[0].re - 3.0).abs() < 1e-10);
//! assert!((reconstructed[0].im - 4.0).abs() < 1e-10);
//! ```

use num_complex::Complex64;

/// Convert complex samples to (magnitude, phase) pairs.
#[derive(Debug, Clone, Copy)]
pub struct ComplexToMagPhase;

impl ComplexToMagPhase {
    /// Process a block of complex samples.
    ///
    /// Returns `(magnitudes, phases)` where phase is in radians [-π, π].
    pub fn process(&self, input: &[Complex64]) -> (Vec<f64>, Vec<f64>) {
        let mut mags = Vec::with_capacity(input.len());
        let mut phases = Vec::with_capacity(input.len());
        for &c in input {
            mags.push(c.norm());
            phases.push(c.arg());
        }
        (mags, phases)
    }

    /// Process a single sample.
    #[inline]
    pub fn process_sample(&self, c: Complex64) -> (f64, f64) {
        (c.norm(), c.arg())
    }
}

/// Convert complex samples to magnitude only.
#[derive(Debug, Clone, Copy)]
pub struct ComplexToMag;

impl ComplexToMag {
    /// Extract magnitudes.
    pub fn process(&self, input: &[Complex64]) -> Vec<f64> {
        input.iter().map(|c| c.norm()).collect()
    }

    /// Squared magnitude (avoids sqrt).
    pub fn process_sq(&self, input: &[Complex64]) -> Vec<f64> {
        input.iter().map(|c| c.norm_sqr()).collect()
    }
}

/// Convert complex samples to phase (argument) only.
#[derive(Debug, Clone, Copy)]
pub struct ComplexToArg;

impl ComplexToArg {
    /// Extract phases in radians [-π, π].
    pub fn process(&self, input: &[Complex64]) -> Vec<f64> {
        input.iter().map(|c| c.arg()).collect()
    }
}

/// Convert (magnitude, phase) pairs back to complex samples.
#[derive(Debug, Clone, Copy)]
pub struct MagPhaseToComplex;

impl MagPhaseToComplex {
    /// Process paired magnitude and phase arrays.
    ///
    /// Arrays must be the same length.
    pub fn process(&self, mags: &[f64], phases: &[f64]) -> Vec<Complex64> {
        mags.iter()
            .zip(phases.iter())
            .map(|(&m, &p)| Complex64::from_polar(m, p))
            .collect()
    }

    /// Process a single sample.
    #[inline]
    pub fn process_sample(&self, mag: f64, phase: f64) -> Complex64 {
        Complex64::from_polar(mag, phase)
    }
}

/// Convert complex to separate real and imaginary streams.
#[derive(Debug, Clone, Copy)]
pub struct ComplexToFloat;

impl ComplexToFloat {
    /// Split into (real, imag) streams.
    pub fn process(&self, input: &[Complex64]) -> (Vec<f64>, Vec<f64>) {
        let mut re = Vec::with_capacity(input.len());
        let mut im = Vec::with_capacity(input.len());
        for &c in input {
            re.push(c.re);
            im.push(c.im);
        }
        (re, im)
    }
}

/// Combine separate real and imaginary streams into complex.
#[derive(Debug, Clone, Copy)]
pub struct FloatToComplex;

impl FloatToComplex {
    /// Combine real and imaginary into complex.
    pub fn process(&self, re: &[f64], im: &[f64]) -> Vec<Complex64> {
        re.iter()
            .zip(im.iter())
            .map(|(&r, &i)| Complex64::new(r, i))
            .collect()
    }

    /// Create complex from real only (imaginary = 0).
    pub fn from_real(&self, re: &[f64]) -> Vec<Complex64> {
        re.iter().map(|&r| Complex64::new(r, 0.0)).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_complex_to_mag_phase() {
        let c2mp = ComplexToMagPhase;
        let input = vec![
            Complex64::new(3.0, 4.0),
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 1.0),
        ];
        let (mags, phases) = c2mp.process(&input);
        assert!((mags[0] - 5.0).abs() < 1e-10);
        assert!((mags[1] - 1.0).abs() < 1e-10);
        assert!((mags[2] - 1.0).abs() < 1e-10);
        assert!((phases[1] - 0.0).abs() < 1e-10);
        assert!((phases[2] - PI / 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_mag_phase_roundtrip() {
        let c2mp = ComplexToMagPhase;
        let mp2c = MagPhaseToComplex;
        let input = vec![
            Complex64::new(3.0, 4.0),
            Complex64::new(-1.0, -2.0),
            Complex64::new(0.0, 0.0),
        ];
        let (mags, phases) = c2mp.process(&input);
        let output = mp2c.process(&mags, &phases);
        for (a, b) in input.iter().zip(output.iter()) {
            assert!((a.re - b.re).abs() < 1e-10);
            assert!((a.im - b.im).abs() < 1e-10);
        }
    }

    #[test]
    fn test_complex_to_mag() {
        let c2m = ComplexToMag;
        let input = vec![Complex64::new(3.0, 4.0)];
        let mags = c2m.process(&input);
        assert!((mags[0] - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_to_mag_sq() {
        let c2m = ComplexToMag;
        let input = vec![Complex64::new(3.0, 4.0)];
        let mags_sq = c2m.process_sq(&input);
        assert!((mags_sq[0] - 25.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_to_arg() {
        let c2a = ComplexToArg;
        let input = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 1.0),
            Complex64::new(-1.0, 0.0),
        ];
        let phases = c2a.process(&input);
        assert!((phases[0] - 0.0).abs() < 1e-10);
        assert!((phases[1] - PI / 2.0).abs() < 1e-10);
        assert!((phases[2] - PI).abs() < 1e-10);
    }

    #[test]
    fn test_complex_to_float() {
        let c2f = ComplexToFloat;
        let input = vec![Complex64::new(1.0, 2.0), Complex64::new(3.0, 4.0)];
        let (re, im) = c2f.process(&input);
        assert_eq!(re, vec![1.0, 3.0]);
        assert_eq!(im, vec![2.0, 4.0]);
    }

    #[test]
    fn test_float_to_complex() {
        let f2c = FloatToComplex;
        let re = vec![1.0, 3.0];
        let im = vec![2.0, 4.0];
        let output = f2c.process(&re, &im);
        assert_eq!(output[0], Complex64::new(1.0, 2.0));
        assert_eq!(output[1], Complex64::new(3.0, 4.0));
    }

    #[test]
    fn test_float_to_complex_real_only() {
        let f2c = FloatToComplex;
        let re = vec![1.0, 2.0, 3.0];
        let output = f2c.from_real(&re);
        assert_eq!(output[0], Complex64::new(1.0, 0.0));
        assert_eq!(output[2], Complex64::new(3.0, 0.0));
    }

    #[test]
    fn test_empty_input() {
        let c2mp = ComplexToMagPhase;
        let (m, p) = c2mp.process(&[]);
        assert!(m.is_empty());
        assert!(p.is_empty());
    }

    #[test]
    fn test_single_sample() {
        let c2mp = ComplexToMagPhase;
        let (m, p) = c2mp.process_sample(Complex64::new(0.0, 1.0));
        assert!((m - 1.0).abs() < 1e-10);
        assert!((p - PI / 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_mag_phase_to_complex_single() {
        let mp2c = MagPhaseToComplex;
        let c = mp2c.process_sample(2.0, PI / 4.0);
        assert!((c.re - 2.0_f64.sqrt()).abs() < 1e-10);
        assert!((c.im - 2.0_f64.sqrt()).abs() < 1e-10);
    }
}
