//! Rail — Signal value clamping / clipping
//!
//! Clips signal values to a specified range. Essential for preventing
//! overflow in fixed-point hardware, AGC saturation recovery, and
//! general signal conditioning. GNU Radio equivalent: `rail_ff`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::rail::{Rail, ComplexRail};
//! use num_complex::Complex64;
//!
//! // Clamp real values to [-1.0, 1.0]
//! let rail = Rail::new(-1.0, 1.0);
//! let input = vec![-2.0, -0.5, 0.0, 0.5, 2.0];
//! let output = rail.process(&input);
//! assert_eq!(output, vec![-1.0, -0.5, 0.0, 0.5, 1.0]);
//!
//! // Clamp complex I/Q components independently
//! let crail = ComplexRail::component(-1.0, 1.0);
//! let iq = vec![Complex64::new(1.5, -2.0)];
//! let out = crail.process(&iq);
//! assert_eq!(out[0], Complex64::new(1.0, -1.0));
//! ```

use num_complex::Complex64;

/// Real-valued rail (clamp) block.
///
/// Clips each sample to `[lo, hi]`.
#[derive(Debug, Clone, Copy)]
pub struct Rail {
    /// Lower bound.
    lo: f64,
    /// Upper bound.
    hi: f64,
}

impl Rail {
    /// Create a rail with the given bounds.
    ///
    /// If `lo > hi`, they are swapped.
    pub fn new(lo: f64, hi: f64) -> Self {
        let (lo, hi) = if lo <= hi { (lo, hi) } else { (hi, lo) };
        Self { lo, hi }
    }

    /// Create a symmetric rail: `[-limit, +limit]`.
    pub fn symmetric(limit: f64) -> Self {
        let limit = limit.abs();
        Self { lo: -limit, hi: limit }
    }

    /// Clamp a single sample.
    #[inline]
    pub fn clamp(&self, x: f64) -> f64 {
        x.clamp(self.lo, self.hi)
    }

    /// Process a block of samples.
    pub fn process(&self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&x| self.clamp(x)).collect()
    }

    /// Process in-place.
    pub fn process_inplace(&self, data: &mut [f64]) {
        for x in data.iter_mut() {
            *x = x.clamp(self.lo, self.hi);
        }
    }

    /// Get the lower bound.
    pub fn lo(&self) -> f64 {
        self.lo
    }

    /// Get the upper bound.
    pub fn hi(&self) -> f64 {
        self.hi
    }

    /// Set bounds.
    pub fn set_bounds(&mut self, lo: f64, hi: f64) {
        if lo <= hi {
            self.lo = lo;
            self.hi = hi;
        } else {
            self.lo = hi;
            self.hi = lo;
        }
    }
}

/// Clipping mode for complex signals.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ComplexClipMode {
    /// Clip I and Q components independently.
    Component,
    /// Clip by magnitude (preserves phase).
    Magnitude,
}

/// Complex-valued rail (clamp) block.
#[derive(Debug, Clone, Copy)]
pub struct ComplexRail {
    /// Lower bound.
    lo: f64,
    /// Upper bound.
    hi: f64,
    /// Clipping mode.
    mode: ComplexClipMode,
}

impl ComplexRail {
    /// Create a complex rail with component-wise clipping.
    pub fn component(lo: f64, hi: f64) -> Self {
        let (lo, hi) = if lo <= hi { (lo, hi) } else { (hi, lo) };
        Self { lo, hi, mode: ComplexClipMode::Component }
    }

    /// Create a complex rail with magnitude clipping.
    ///
    /// Clips magnitude to `[lo, hi]` while preserving phase.
    pub fn magnitude(lo: f64, hi: f64) -> Self {
        let (lo, hi) = if lo <= hi { (lo, hi) } else { (hi, lo) };
        Self { lo, hi, mode: ComplexClipMode::Magnitude }
    }

    /// Clamp a single complex sample.
    #[inline]
    pub fn clamp(&self, x: Complex64) -> Complex64 {
        match self.mode {
            ComplexClipMode::Component => {
                Complex64::new(
                    x.re.clamp(self.lo, self.hi),
                    x.im.clamp(self.lo, self.hi),
                )
            }
            ComplexClipMode::Magnitude => {
                let mag = x.norm();
                if mag < 1e-30 {
                    return x;
                }
                let clamped_mag = mag.clamp(self.lo, self.hi);
                if (clamped_mag - mag).abs() < 1e-30 {
                    x
                } else {
                    x * (clamped_mag / mag)
                }
            }
        }
    }

    /// Process a block of complex samples.
    pub fn process(&self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&x| self.clamp(x)).collect()
    }

    /// Process in-place.
    pub fn process_inplace(&self, data: &mut [Complex64]) {
        for x in data.iter_mut() {
            *x = self.clamp(*x);
        }
    }

    /// Get the clipping mode.
    pub fn mode(&self) -> ComplexClipMode {
        self.mode
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rail_basic() {
        let rail = Rail::new(-1.0, 1.0);
        assert_eq!(rail.clamp(0.5), 0.5);
        assert_eq!(rail.clamp(2.0), 1.0);
        assert_eq!(rail.clamp(-3.0), -1.0);
    }

    #[test]
    fn test_rail_process() {
        let rail = Rail::new(-1.0, 1.0);
        let input = vec![-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0];
        let output = rail.process(&input);
        assert_eq!(output, vec![-1.0, -1.0, -0.5, 0.0, 0.5, 1.0, 1.0]);
    }

    #[test]
    fn test_rail_symmetric() {
        let rail = Rail::symmetric(0.5);
        assert_eq!(rail.lo(), -0.5);
        assert_eq!(rail.hi(), 0.5);
        assert_eq!(rail.clamp(1.0), 0.5);
        assert_eq!(rail.clamp(-1.0), -0.5);
    }

    #[test]
    fn test_rail_swapped_bounds() {
        let rail = Rail::new(1.0, -1.0);
        assert_eq!(rail.lo(), -1.0);
        assert_eq!(rail.hi(), 1.0);
    }

    #[test]
    fn test_rail_inplace() {
        let rail = Rail::new(0.0, 1.0);
        let mut data = vec![-0.5, 0.5, 1.5];
        rail.process_inplace(&mut data);
        assert_eq!(data, vec![0.0, 0.5, 1.0]);
    }

    #[test]
    fn test_set_bounds() {
        let mut rail = Rail::new(-1.0, 1.0);
        rail.set_bounds(-0.5, 0.5);
        assert_eq!(rail.lo(), -0.5);
        assert_eq!(rail.hi(), 0.5);
    }

    #[test]
    fn test_complex_component() {
        let crail = ComplexRail::component(-1.0, 1.0);
        let x = Complex64::new(1.5, -2.0);
        let y = crail.clamp(x);
        assert_eq!(y.re, 1.0);
        assert_eq!(y.im, -1.0);
    }

    #[test]
    fn test_complex_magnitude() {
        let crail = ComplexRail::magnitude(0.0, 1.0);
        let x = Complex64::new(2.0, 0.0);
        let y = crail.clamp(x);
        assert!((y.re - 1.0).abs() < 1e-10);
        assert!(y.im.abs() < 1e-10);
    }

    #[test]
    fn test_complex_magnitude_preserves_phase() {
        let crail = ComplexRail::magnitude(0.0, 0.5);
        let x = Complex64::new(3.0, 4.0); // magnitude 5, angle atan2(4,3)
        let y = crail.clamp(x);
        assert!((y.norm() - 0.5).abs() < 1e-10);
        // Phase should be preserved
        assert!((y.arg() - x.arg()).abs() < 1e-10);
    }

    #[test]
    fn test_complex_within_range() {
        let crail = ComplexRail::magnitude(0.0, 10.0);
        let x = Complex64::new(1.0, 1.0);
        let y = crail.clamp(x);
        assert!((y.re - x.re).abs() < 1e-10);
        assert!((y.im - x.im).abs() < 1e-10);
    }

    #[test]
    fn test_complex_process() {
        let crail = ComplexRail::component(-1.0, 1.0);
        let input = vec![
            Complex64::new(0.5, 0.5),
            Complex64::new(2.0, -2.0),
        ];
        let output = crail.process(&input);
        assert_eq!(output[0], Complex64::new(0.5, 0.5));
        assert_eq!(output[1], Complex64::new(1.0, -1.0));
    }
}
