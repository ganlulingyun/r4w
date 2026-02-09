//! Phase Shift — Apply constant phase rotation to complex samples
//!
//! Multiplies each sample by exp(jθ) to apply a fixed phase offset.
//! Used for phase calibration, I/Q correction, constellation rotation,
//! and differential encoding/decoding.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::phase_shift::PhaseShift;
//! use num_complex::Complex64;
//! use std::f64::consts::PI;
//!
//! // 90° rotation: (1+0j) → (0+1j)
//! let mut ps = PhaseShift::new(PI / 2.0);
//! let input = vec![Complex64::new(1.0, 0.0)];
//! let output = ps.process(&input);
//! assert!(output[0].re.abs() < 1e-10);
//! assert!((output[0].im - 1.0).abs() < 1e-10);
//! ```

use num_complex::Complex64;

/// Constant phase rotation block.
#[derive(Debug, Clone)]
pub struct PhaseShift {
    /// Phase rotation in radians.
    phase_rad: f64,
    /// Precomputed rotation phasor: exp(jθ).
    phasor: Complex64,
}

impl PhaseShift {
    /// Create a phase shift block.
    ///
    /// `phase_rad`: rotation angle in radians.
    pub fn new(phase_rad: f64) -> Self {
        Self {
            phase_rad,
            phasor: Complex64::from_polar(1.0, phase_rad),
        }
    }

    /// Create from degrees.
    pub fn from_degrees(degrees: f64) -> Self {
        Self::new(degrees * std::f64::consts::PI / 180.0)
    }

    /// Process a block of complex samples.
    pub fn process(&self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&x| x * self.phasor).collect()
    }

    /// Process in-place.
    pub fn process_inplace(&self, data: &mut [Complex64]) {
        for x in data.iter_mut() {
            *x *= self.phasor;
        }
    }

    /// Process a single sample.
    #[inline]
    pub fn process_sample(&self, x: Complex64) -> Complex64 {
        x * self.phasor
    }

    /// Get phase in radians.
    pub fn phase_rad(&self) -> f64 {
        self.phase_rad
    }

    /// Get phase in degrees.
    pub fn phase_deg(&self) -> f64 {
        self.phase_rad * 180.0 / std::f64::consts::PI
    }

    /// Set phase in radians.
    pub fn set_phase(&mut self, phase_rad: f64) {
        self.phase_rad = phase_rad;
        self.phasor = Complex64::from_polar(1.0, phase_rad);
    }

    /// Set phase in degrees.
    pub fn set_phase_degrees(&mut self, degrees: f64) {
        self.set_phase(degrees * std::f64::consts::PI / 180.0);
    }

    /// Get the rotation phasor.
    pub fn phasor(&self) -> Complex64 {
        self.phasor
    }
}

/// Apply a constant phase shift to a slice (convenience function).
pub fn phase_rotate(input: &[Complex64], phase_rad: f64) -> Vec<Complex64> {
    let phasor = Complex64::from_polar(1.0, phase_rad);
    input.iter().map(|&x| x * phasor).collect()
}

/// Compute the phase difference between consecutive samples.
pub fn phase_delta(input: &[Complex64]) -> Vec<f64> {
    if input.len() < 2 {
        return Vec::new();
    }
    input
        .windows(2)
        .map(|w| (w[1] * w[0].conj()).arg())
        .collect()
}

/// Compute mean phase offset between two signal sequences.
pub fn mean_phase_offset(a: &[Complex64], b: &[Complex64]) -> f64 {
    if a.is_empty() || b.is_empty() {
        return 0.0;
    }
    let sum: Complex64 = a
        .iter()
        .zip(b.iter())
        .map(|(&ai, &bi)| bi * ai.conj())
        .sum();
    sum.arg()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_zero_phase() {
        let ps = PhaseShift::new(0.0);
        let input = vec![Complex64::new(1.0, 0.0), Complex64::new(0.0, 1.0)];
        let output = ps.process(&input);
        assert!((output[0].re - 1.0).abs() < 1e-10);
        assert!((output[1].im - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_90_degrees() {
        let ps = PhaseShift::new(PI / 2.0);
        let output = ps.process_sample(Complex64::new(1.0, 0.0));
        assert!(output.re.abs() < 1e-10);
        assert!((output.im - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_180_degrees() {
        let ps = PhaseShift::new(PI);
        let output = ps.process_sample(Complex64::new(1.0, 0.0));
        assert!((output.re - (-1.0)).abs() < 1e-10);
        assert!(output.im.abs() < 1e-10);
    }

    #[test]
    fn test_from_degrees() {
        let ps = PhaseShift::from_degrees(90.0);
        assert!((ps.phase_deg() - 90.0).abs() < 1e-10);
        let output = ps.process_sample(Complex64::new(1.0, 0.0));
        assert!(output.re.abs() < 1e-10);
        assert!((output.im - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_preserves_magnitude() {
        let ps = PhaseShift::new(1.234);
        let input = Complex64::new(3.0, 4.0); // |z| = 5
        let output = ps.process_sample(input);
        assert!((output.norm() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_inplace() {
        let ps = PhaseShift::new(PI);
        let mut data = vec![Complex64::new(1.0, 0.0)];
        ps.process_inplace(&mut data);
        assert!((data[0].re - (-1.0)).abs() < 1e-10);
    }

    #[test]
    fn test_set_phase() {
        let mut ps = PhaseShift::new(0.0);
        ps.set_phase(PI / 2.0);
        assert!((ps.phase_rad() - PI / 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_set_phase_degrees() {
        let mut ps = PhaseShift::new(0.0);
        ps.set_phase_degrees(45.0);
        assert!((ps.phase_deg() - 45.0).abs() < 1e-10);
    }

    #[test]
    fn test_phase_rotate_fn() {
        let input = vec![Complex64::new(1.0, 0.0)];
        let output = phase_rotate(&input, PI / 2.0);
        assert!(output[0].re.abs() < 1e-10);
        assert!((output[0].im - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_phase_delta() {
        let input = vec![
            Complex64::from_polar(1.0, 0.0),
            Complex64::from_polar(1.0, 0.1),
            Complex64::from_polar(1.0, 0.2),
        ];
        let deltas = phase_delta(&input);
        assert_eq!(deltas.len(), 2);
        assert!((deltas[0] - 0.1).abs() < 1e-10);
        assert!((deltas[1] - 0.1).abs() < 1e-10);
    }

    #[test]
    fn test_mean_phase_offset() {
        let a: Vec<Complex64> = (0..10)
            .map(|i| Complex64::from_polar(1.0, i as f64 * 0.1))
            .collect();
        let offset = PI / 4.0;
        let b: Vec<Complex64> = a.iter().map(|&x| x * Complex64::from_polar(1.0, offset)).collect();
        let measured = mean_phase_offset(&a, &b);
        assert!((measured - offset).abs() < 1e-10);
    }

    #[test]
    fn test_empty() {
        let ps = PhaseShift::new(PI);
        assert!(ps.process(&[]).is_empty());
        assert!(phase_delta(&[]).is_empty());
    }

    #[test]
    fn test_phase_delta_single() {
        assert!(phase_delta(&[Complex64::new(1.0, 0.0)]).is_empty());
    }
}
