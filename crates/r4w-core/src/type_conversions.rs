//! Type Conversion Blocks
//!
//! Lightweight blocks for converting between complex and real signal
//! representations. These are essential pipeline glue blocks for
//! connecting complex-domain processing to real-domain displays,
//! measurements, and audio.
//!
//! ## Available Converters
//!
//! | Block | Operation | Input | Output |
//! |-------|-----------|-------|--------|
//! | `ComplexToMag` | `\|z\| = sqrt(re² + im²)` | Complex | Real |
//! | `ComplexToMagSq` | `\|z\|² = re² + im²` | Complex | Real |
//! | `ComplexToArg` | `atan2(im, re)` | Complex | Real |
//! | `ComplexToReal` | `re(z)` | Complex | Real |
//! | `ComplexToImag` | `im(z)` | Complex | Real |
//! | `RealToComplex` | `re + j·0` | Real | Complex |
//! | `MagPhaseToComplex` | `mag·exp(j·phase)` | (Real, Real) | Complex |
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::type_conversions::*;
//! use num_complex::Complex64;
//!
//! let samples = vec![Complex64::new(3.0, 4.0), Complex64::new(1.0, 0.0)];
//!
//! let mags = ComplexToMag.convert_block(&samples);
//! assert!((mags[0] - 5.0).abs() < 1e-10);
//!
//! let phases = ComplexToArg.convert_block(&samples);
//! assert!((phases[1]).abs() < 1e-10); // arg(1+0j) = 0
//! ```

use num_complex::Complex64;

/// Extract magnitude: |z| = sqrt(re² + im²)
pub struct ComplexToMag;

impl ComplexToMag {
    pub fn convert(&self, input: Complex64) -> f64 {
        input.norm()
    }

    pub fn convert_block(&self, input: &[Complex64]) -> Vec<f64> {
        input.iter().map(|z| z.norm()).collect()
    }
}

/// Extract magnitude squared: |z|² = re² + im² (avoids sqrt, efficient for power)
pub struct ComplexToMagSq;

impl ComplexToMagSq {
    pub fn convert(&self, input: Complex64) -> f64 {
        input.norm_sqr()
    }

    pub fn convert_block(&self, input: &[Complex64]) -> Vec<f64> {
        input.iter().map(|z| z.norm_sqr()).collect()
    }
}

/// Extract phase angle: arg(z) = atan2(im, re), range (-π, π]
pub struct ComplexToArg;

impl ComplexToArg {
    pub fn convert(&self, input: Complex64) -> f64 {
        input.arg()
    }

    pub fn convert_block(&self, input: &[Complex64]) -> Vec<f64> {
        input.iter().map(|z| z.arg()).collect()
    }
}

/// Extract real part: re(z)
pub struct ComplexToReal;

impl ComplexToReal {
    pub fn convert(&self, input: Complex64) -> f64 {
        input.re
    }

    pub fn convert_block(&self, input: &[Complex64]) -> Vec<f64> {
        input.iter().map(|z| z.re).collect()
    }
}

/// Extract imaginary part: im(z)
pub struct ComplexToImag;

impl ComplexToImag {
    pub fn convert(&self, input: Complex64) -> f64 {
        input.im
    }

    pub fn convert_block(&self, input: &[Complex64]) -> Vec<f64> {
        input.iter().map(|z| z.im).collect()
    }
}

/// Convert real to complex: re → re + j·0
pub struct RealToComplex;

impl RealToComplex {
    pub fn convert(&self, input: f64) -> Complex64 {
        Complex64::new(input, 0.0)
    }

    pub fn convert_block(&self, input: &[f64]) -> Vec<Complex64> {
        input.iter().map(|&r| Complex64::new(r, 0.0)).collect()
    }
}

/// Convert magnitude + phase to complex: mag·exp(j·phase)
pub struct MagPhaseToComplex;

impl MagPhaseToComplex {
    pub fn convert(&self, mag: f64, phase: f64) -> Complex64 {
        Complex64::new(mag * phase.cos(), mag * phase.sin())
    }

    pub fn convert_block(&self, mags: &[f64], phases: &[f64]) -> Vec<Complex64> {
        mags.iter()
            .zip(phases.iter())
            .map(|(&m, &p)| Complex64::new(m * p.cos(), m * p.sin()))
            .collect()
    }
}

/// Split complex into separate I and Q real streams, or merge them back.
pub struct ComplexToFloat;

impl ComplexToFloat {
    /// Split complex stream into (I, Q) real streams.
    pub fn split(input: &[Complex64]) -> (Vec<f64>, Vec<f64>) {
        let i = input.iter().map(|z| z.re).collect();
        let q = input.iter().map(|z| z.im).collect();
        (i, q)
    }

    /// Merge separate I and Q streams into complex.
    /// If lengths differ, output is truncated to the shorter.
    pub fn merge(i: &[f64], q: &[f64]) -> Vec<Complex64> {
        i.iter()
            .zip(q.iter())
            .map(|(&re, &im)| Complex64::new(re, im))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_complex_to_mag() {
        let c = ComplexToMag;
        assert!((c.convert(Complex64::new(3.0, 4.0)) - 5.0).abs() < 1e-10);
        assert!((c.convert(Complex64::new(0.0, 0.0))).abs() < 1e-10);
        assert!((c.convert(Complex64::new(1.0, 0.0)) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_to_mag_block() {
        let c = ComplexToMag;
        let input = vec![
            Complex64::new(3.0, 4.0),
            Complex64::new(0.0, 1.0),
            Complex64::new(-1.0, 0.0),
        ];
        let out = c.convert_block(&input);
        assert!((out[0] - 5.0).abs() < 1e-10);
        assert!((out[1] - 1.0).abs() < 1e-10);
        assert!((out[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_to_mag_sq() {
        let c = ComplexToMagSq;
        assert!((c.convert(Complex64::new(3.0, 4.0)) - 25.0).abs() < 1e-10);
        assert!((c.convert(Complex64::new(1.0, 1.0)) - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_to_arg() {
        let c = ComplexToArg;
        assert!((c.convert(Complex64::new(1.0, 0.0))).abs() < 1e-10);
        assert!((c.convert(Complex64::new(0.0, 1.0)) - PI / 2.0).abs() < 1e-10);
        assert!((c.convert(Complex64::new(-1.0, 0.0)) - PI).abs() < 1e-10);
        assert!((c.convert(Complex64::new(0.0, -1.0)) + PI / 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_to_real() {
        let c = ComplexToReal;
        assert!((c.convert(Complex64::new(3.14, 2.71)) - 3.14).abs() < 1e-10);
    }

    #[test]
    fn test_complex_to_imag() {
        let c = ComplexToImag;
        assert!((c.convert(Complex64::new(3.14, 2.71)) - 2.71).abs() < 1e-10);
    }

    #[test]
    fn test_real_to_complex() {
        let c = RealToComplex;
        let z = c.convert(42.0);
        assert!((z.re - 42.0).abs() < 1e-10);
        assert!(z.im.abs() < 1e-10);
    }

    #[test]
    fn test_real_to_complex_block() {
        let c = RealToComplex;
        let out = c.convert_block(&[1.0, 2.0, 3.0]);
        assert_eq!(out.len(), 3);
        assert!((out[1].re - 2.0).abs() < 1e-10);
        assert!(out[1].im.abs() < 1e-10);
    }

    #[test]
    fn test_mag_phase_to_complex() {
        let c = MagPhaseToComplex;
        // Unit magnitude, 0 phase → (1, 0)
        let z = c.convert(1.0, 0.0);
        assert!((z.re - 1.0).abs() < 1e-10);
        assert!(z.im.abs() < 1e-10);
        // Magnitude 2, phase π/2 → (0, 2)
        let z = c.convert(2.0, PI / 2.0);
        assert!(z.re.abs() < 1e-10);
        assert!((z.im - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_mag_phase_roundtrip() {
        let input = vec![
            Complex64::new(3.0, 4.0),
            Complex64::new(-1.0, 2.0),
            Complex64::new(0.5, -0.5),
        ];
        let mags = ComplexToMag.convert_block(&input);
        let phases = ComplexToArg.convert_block(&input);
        let reconstructed = MagPhaseToComplex.convert_block(&mags, &phases);
        for (orig, recon) in input.iter().zip(reconstructed.iter()) {
            assert!((orig.re - recon.re).abs() < 1e-10, "re mismatch: {} vs {}", orig.re, recon.re);
            assert!((orig.im - recon.im).abs() < 1e-10, "im mismatch: {} vs {}", orig.im, recon.im);
        }
    }

    #[test]
    fn test_complex_to_float_split_merge() {
        let input = vec![
            Complex64::new(1.0, 2.0),
            Complex64::new(3.0, 4.0),
            Complex64::new(5.0, 6.0),
        ];
        let (i, q) = ComplexToFloat::split(&input);
        assert_eq!(i, vec![1.0, 3.0, 5.0]);
        assert_eq!(q, vec![2.0, 4.0, 6.0]);

        let merged = ComplexToFloat::merge(&i, &q);
        assert_eq!(merged.len(), 3);
        assert!((merged[0].re - 1.0).abs() < 1e-10);
        assert!((merged[2].im - 6.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_to_float_merge_unequal() {
        let merged = ComplexToFloat::merge(&[1.0, 2.0, 3.0], &[10.0, 20.0]);
        assert_eq!(merged.len(), 2); // truncated to shorter
    }

    #[test]
    fn test_real_imag_roundtrip() {
        let input = vec![Complex64::new(1.5, -2.5), Complex64::new(0.0, 3.0)];
        let reals = ComplexToReal.convert_block(&input);
        let imags = ComplexToImag.convert_block(&input);
        let merged = ComplexToFloat::merge(&reals, &imags);
        for (orig, recon) in input.iter().zip(merged.iter()) {
            assert!((orig.re - recon.re).abs() < 1e-10);
            assert!((orig.im - recon.im).abs() < 1e-10);
        }
    }
}
