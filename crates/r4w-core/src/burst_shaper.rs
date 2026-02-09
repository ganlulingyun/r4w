//! Burst Shaper — Apply windowed ramp up/down to burst edges
//!
//! Shapes burst boundaries with smooth ramp-up and ramp-down windows
//! to reduce spectral splatter from abrupt on/off transitions.
//! Supports raised-cosine, linear, and Hann window shapes.
//! GNU Radio equivalent: `burst_shaper`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::burst_shaper::{BurstShaper, WindowShape};
//!
//! let shaper = BurstShaper::new(WindowShape::RaisedCosine, 16, 16);
//! let burst = vec![1.0; 100];
//! let shaped = shaper.shape(&burst);
//! // First sample is ramped up from ~0, last is ramped down
//! assert!(shaped[0].abs() < 0.1);
//! assert!((shaped[50] - 1.0).abs() < 1e-10);
//! ```

use num_complex::Complex64;

/// Window shape for burst ramp-up/down.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WindowShape {
    /// Linear ramp (triangle).
    Linear,
    /// Raised cosine (smooth).
    RaisedCosine,
    /// Hann window ramp (cos²).
    Hann,
}

/// Burst shaper that applies ramp-up and ramp-down windows.
#[derive(Debug, Clone)]
pub struct BurstShaper {
    /// Window shape.
    shape: WindowShape,
    /// Ramp-up length in samples.
    ramp_up_len: usize,
    /// Ramp-down length in samples.
    ramp_down_len: usize,
    /// Pre-computed ramp-up window.
    ramp_up: Vec<f64>,
    /// Pre-computed ramp-down window.
    ramp_down: Vec<f64>,
}

impl BurstShaper {
    /// Create a burst shaper.
    ///
    /// - `shape`: window shape for ramps
    /// - `ramp_up_len`: number of samples for ramp-up
    /// - `ramp_down_len`: number of samples for ramp-down
    pub fn new(shape: WindowShape, ramp_up_len: usize, ramp_down_len: usize) -> Self {
        let ramp_up = make_ramp(shape, ramp_up_len, true);
        let ramp_down = make_ramp(shape, ramp_down_len, false);
        Self {
            shape,
            ramp_up_len,
            ramp_down_len,
            ramp_up,
            ramp_down,
        }
    }

    /// Create with symmetric ramp lengths.
    pub fn symmetric(shape: WindowShape, ramp_len: usize) -> Self {
        Self::new(shape, ramp_len, ramp_len)
    }

    /// Shape a real-valued burst.
    pub fn shape(&self, burst: &[f64]) -> Vec<f64> {
        let mut output = burst.to_vec();
        self.apply_ramps(&mut output);
        output
    }

    /// Shape a complex burst.
    pub fn shape_complex(&self, burst: &[Complex64]) -> Vec<Complex64> {
        let mut output = burst.to_vec();
        self.apply_ramps_complex(&mut output);
        output
    }

    /// Apply ramps in-place to a real signal.
    pub fn apply_ramps(&self, data: &mut [f64]) {
        let n = data.len();
        // Ramp up
        let up_len = self.ramp_up_len.min(n);
        for i in 0..up_len {
            data[i] *= self.ramp_up[i];
        }
        // Ramp down
        let down_len = self.ramp_down_len.min(n);
        let down_start = n.saturating_sub(down_len);
        for i in 0..down_len {
            if down_start + i < n {
                data[down_start + i] *= self.ramp_down[i];
            }
        }
    }

    /// Apply ramps in-place to a complex signal.
    pub fn apply_ramps_complex(&self, data: &mut [Complex64]) {
        let n = data.len();
        let up_len = self.ramp_up_len.min(n);
        for i in 0..up_len {
            data[i] *= self.ramp_up[i];
        }
        let down_len = self.ramp_down_len.min(n);
        let down_start = n.saturating_sub(down_len);
        for i in 0..down_len {
            if down_start + i < n {
                data[down_start + i] *= self.ramp_down[i];
            }
        }
    }

    /// Get the ramp-up window coefficients.
    pub fn ramp_up_window(&self) -> &[f64] {
        &self.ramp_up
    }

    /// Get the ramp-down window coefficients.
    pub fn ramp_down_window(&self) -> &[f64] {
        &self.ramp_down
    }

    /// Get the window shape.
    pub fn shape_type(&self) -> WindowShape {
        self.shape
    }

    /// Total overhead samples (ramp_up + ramp_down).
    pub fn overhead(&self) -> usize {
        self.ramp_up_len + self.ramp_down_len
    }
}

/// Generate a ramp window.
fn make_ramp(shape: WindowShape, len: usize, rising: bool) -> Vec<f64> {
    if len == 0 {
        return Vec::new();
    }
    (0..len)
        .map(|i| {
            let t = if rising {
                i as f64 / len as f64
            } else {
                1.0 - i as f64 / len as f64
            };
            match shape {
                WindowShape::Linear => t,
                WindowShape::RaisedCosine => 0.5 * (1.0 - (std::f64::consts::PI * t).cos()),
                WindowShape::Hann => {
                    let w = 0.5 * (1.0 - (std::f64::consts::PI * t).cos());
                    w
                }
            }
        })
        .collect()
}

/// One-shot burst shaping with raised-cosine ramps.
pub fn shape_burst(burst: &[f64], ramp_len: usize) -> Vec<f64> {
    let shaper = BurstShaper::symmetric(WindowShape::RaisedCosine, ramp_len);
    shaper.shape(burst)
}

/// Add zero-padding before and after a burst.
pub fn pad_burst<T: Clone + Default>(burst: &[T], pre: usize, post: usize) -> Vec<T> {
    let mut output = vec![T::default(); pre];
    output.extend_from_slice(burst);
    output.resize(output.len() + post, T::default());
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_linear_ramp_up() {
        let ramp = make_ramp(WindowShape::Linear, 4, true);
        assert!((ramp[0] - 0.0).abs() < 1e-10);
        assert!((ramp[1] - 0.25).abs() < 1e-10);
        assert!((ramp[2] - 0.5).abs() < 1e-10);
        assert!((ramp[3] - 0.75).abs() < 1e-10);
    }

    #[test]
    fn test_linear_ramp_down() {
        let ramp = make_ramp(WindowShape::Linear, 4, false);
        assert!((ramp[0] - 1.0).abs() < 1e-10);
        assert!((ramp[3] - 0.25).abs() < 1e-10);
    }

    #[test]
    fn test_raised_cosine_ramp() {
        let ramp = make_ramp(WindowShape::RaisedCosine, 10, true);
        assert!(ramp[0].abs() < 1e-10); // Start at ~0
        assert!(ramp[9] > 0.9); // End near 1
    }

    #[test]
    fn test_shape_burst_real() {
        let shaper = BurstShaper::symmetric(WindowShape::Linear, 4);
        let burst = vec![1.0; 10];
        let shaped = shaper.shape(&burst);
        // First sample = 0 (ramp up)
        assert!((shaped[0] - 0.0).abs() < 1e-10);
        // Middle untouched
        assert!((shaped[5] - 1.0).abs() < 1e-10);
        // Last sample ramped down
        assert!(shaped[9] < 0.5);
    }

    #[test]
    fn test_shape_complex() {
        let shaper = BurstShaper::symmetric(WindowShape::RaisedCosine, 8);
        let burst = vec![Complex64::new(1.0, 1.0); 32];
        let shaped = shaper.shape_complex(&burst);
        assert_eq!(shaped.len(), 32);
        assert!(shaped[0].norm() < 0.5); // Ramped
        assert!((shaped[16].re - 1.0).abs() < 1e-10); // Middle untouched
    }

    #[test]
    fn test_asymmetric_ramps() {
        let shaper = BurstShaper::new(WindowShape::Linear, 2, 4);
        assert_eq!(shaper.overhead(), 6);
        let burst = vec![1.0; 20];
        let shaped = shaper.shape(&burst);
        // First sample ramped (2-sample ramp up)
        assert!(shaped[0] < 0.5);
        // Last 4 samples ramped down
        assert!(shaped[19] < 0.5);
    }

    #[test]
    fn test_zero_ramp() {
        let shaper = BurstShaper::new(WindowShape::Linear, 0, 0);
        let burst = vec![1.0; 10];
        let shaped = shaper.shape(&burst);
        assert_eq!(shaped, burst); // No ramps applied
    }

    #[test]
    fn test_ramp_longer_than_burst() {
        let shaper = BurstShaper::symmetric(WindowShape::Linear, 20);
        let burst = vec![1.0; 5];
        let shaped = shaper.shape(&burst);
        assert_eq!(shaped.len(), 5);
        // All samples are ramped
        for &v in &shaped {
            assert!(v <= 1.0);
        }
    }

    #[test]
    fn test_shape_burst_fn() {
        let burst = vec![1.0; 50];
        let shaped = shape_burst(&burst, 10);
        assert!(shaped[0].abs() < 0.1);
        assert!((shaped[25] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_pad_burst() {
        let burst = vec![1.0, 2.0, 3.0];
        let padded = pad_burst(&burst, 2, 3);
        assert_eq!(padded, vec![0.0, 0.0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_window_accessors() {
        let shaper = BurstShaper::symmetric(WindowShape::Hann, 8);
        assert_eq!(shaper.ramp_up_window().len(), 8);
        assert_eq!(shaper.ramp_down_window().len(), 8);
        assert_eq!(shaper.shape_type(), WindowShape::Hann);
    }

    #[test]
    fn test_empty_burst() {
        let shaper = BurstShaper::symmetric(WindowShape::Linear, 4);
        assert!(shaper.shape(&[]).is_empty());
    }
}
