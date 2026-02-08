//! Window Functions for Filter Design
//!
//! Provides various window functions used in FIR filter design and spectral analysis.
//!
//! ## Window Selection Guide
//!
//! | Window     | Main Lobe Width | Sidelobe Level | Use Case                    |
//! |------------|-----------------|----------------|------------------------------|
//! | Rectangular| Narrowest       | -13 dB         | High frequency resolution    |
//! | Hamming    | Medium          | -43 dB         | General purpose              |
//! | Hann       | Medium          | -32 dB         | Spectral analysis            |
//! | Blackman   | Wide            | -58 dB         | High dynamic range           |
//! | Kaiser(β)  | Adjustable      | Adjustable     | Optimal trade-off            |
//!
//! ## Example
//!
//! ```rust,ignore
//! use r4w_core::filters::windows::{Window, kaiser_window, hamming_window};
//!
//! // Generate a Kaiser window with β=8 (good sidelobe suppression)
//! let kaiser = kaiser_window(64, 8.0);
//!
//! // Generate a Hamming window
//! let hamming = hamming_window(64);
//! ```

use std::f64::consts::PI;

/// Window function type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Window {
    /// Rectangular window (no windowing)
    Rectangular,
    /// Hamming window: 0.54 - 0.46*cos(2πn/N)
    Hamming,
    /// Hann (Hanning) window: 0.5*(1 - cos(2πn/N))
    Hann,
    /// Blackman window: 0.42 - 0.5*cos(2πn/N) + 0.08*cos(4πn/N)
    Blackman,
    /// Blackman-Harris window (4-term, -92 dB sidelobes)
    BlackmanHarris,
    /// Kaiser window with shape parameter β
    Kaiser(f64),
}

impl Default for Window {
    fn default() -> Self {
        Window::Blackman
    }
}

impl Window {
    /// Generate window coefficients for the given length.
    pub fn generate(&self, length: usize) -> Vec<f64> {
        match self {
            Window::Rectangular => rectangular_window(length),
            Window::Hamming => hamming_window(length),
            Window::Hann => hann_window(length),
            Window::Blackman => blackman_window(length),
            Window::BlackmanHarris => blackman_harris_window(length),
            Window::Kaiser(beta) => kaiser_window(length, *beta),
        }
    }

    /// Get the coherent gain of this window type.
    /// This is the sum of window coefficients normalized by length.
    pub fn coherent_gain(&self, length: usize) -> f64 {
        let window = self.generate(length);
        window.iter().sum::<f64>() / length as f64
    }

    /// Estimate the equivalent noise bandwidth (ENBW) in bins.
    /// Higher ENBW means more noise power in spectral estimates.
    pub fn enbw(&self) -> f64 {
        match self {
            Window::Rectangular => 1.0,
            Window::Hamming => 1.36,
            Window::Hann => 1.50,
            Window::Blackman => 1.73,
            Window::BlackmanHarris => 2.00,
            Window::Kaiser(beta) => 1.0 + 0.1 * beta, // Approximate
        }
    }
}

/// Generate a rectangular (boxcar) window.
pub fn rectangular_window(length: usize) -> Vec<f64> {
    vec![1.0; length]
}

/// Generate a Hamming window.
///
/// The Hamming window is defined as:
/// w[n] = 0.54 - 0.46 * cos(2πn/(N-1))
///
/// Provides -43 dB sidelobe level with moderate main lobe width.
pub fn hamming_window(length: usize) -> Vec<f64> {
    if length == 0 {
        return vec![];
    }
    if length == 1 {
        return vec![1.0];
    }

    let n_minus_1 = (length - 1) as f64;
    (0..length)
        .map(|n| 0.54 - 0.46 * (2.0 * PI * n as f64 / n_minus_1).cos())
        .collect()
}

/// Generate a Hann (Hanning) window.
///
/// The Hann window is defined as:
/// w[n] = 0.5 * (1 - cos(2πn/(N-1)))
///
/// Provides -32 dB sidelobe level, commonly used for spectral analysis.
pub fn hann_window(length: usize) -> Vec<f64> {
    if length == 0 {
        return vec![];
    }
    if length == 1 {
        return vec![1.0];
    }

    let n_minus_1 = (length - 1) as f64;
    (0..length)
        .map(|n| 0.5 * (1.0 - (2.0 * PI * n as f64 / n_minus_1).cos()))
        .collect()
}

/// Generate a Blackman window.
///
/// The Blackman window is defined as:
/// w[n] = 0.42 - 0.5*cos(2πn/(N-1)) + 0.08*cos(4πn/(N-1))
///
/// Provides -58 dB sidelobe level with wider main lobe.
pub fn blackman_window(length: usize) -> Vec<f64> {
    if length == 0 {
        return vec![];
    }
    if length == 1 {
        return vec![1.0];
    }

    let n_minus_1 = (length - 1) as f64;
    (0..length)
        .map(|n| {
            let x = 2.0 * PI * n as f64 / n_minus_1;
            0.42 - 0.5 * x.cos() + 0.08 * (2.0 * x).cos()
        })
        .collect()
}

/// Generate a Blackman-Harris window (4-term).
///
/// Provides -92 dB sidelobe level, excellent for high dynamic range applications.
pub fn blackman_harris_window(length: usize) -> Vec<f64> {
    if length == 0 {
        return vec![];
    }
    if length == 1 {
        return vec![1.0];
    }

    let n_minus_1 = (length - 1) as f64;
    let a0 = 0.35875;
    let a1 = 0.48829;
    let a2 = 0.14128;
    let a3 = 0.01168;

    (0..length)
        .map(|n| {
            let x = 2.0 * PI * n as f64 / n_minus_1;
            a0 - a1 * x.cos() + a2 * (2.0 * x).cos() - a3 * (3.0 * x).cos()
        })
        .collect()
}

/// Generate a Kaiser window with shape parameter β.
///
/// The Kaiser window provides optimal trade-off between main lobe width
/// and sidelobe level, controlled by β:
///
/// - β = 0: Rectangular window
/// - β = 5: Similar to Hamming
/// - β = 6: Similar to Hann
/// - β = 8.6: Similar to Blackman
/// - β > 10: Very low sidelobes
///
/// # Arguments
/// * `length` - Number of window samples
/// * `beta` - Shape parameter (typically 0 to 12)
pub fn kaiser_window(length: usize, beta: f64) -> Vec<f64> {
    if length == 0 {
        return vec![];
    }
    if length == 1 {
        return vec![1.0];
    }

    let n_minus_1 = (length - 1) as f64;
    let half = n_minus_1 / 2.0;
    let i0_beta = bessel_i0(beta);

    (0..length)
        .map(|n| {
            let x = (n as f64 - half) / half;
            let arg = beta * (1.0 - x * x).sqrt();
            bessel_i0(arg) / i0_beta
        })
        .collect()
}

/// Calculate Kaiser β parameter from desired attenuation.
///
/// Given a desired stopband attenuation in dB, computes the
/// optimal Kaiser β parameter.
///
/// # Arguments
/// * `attenuation_db` - Desired stopband attenuation in dB (positive value)
///
/// # Returns
/// Kaiser β parameter
///
/// # Example
/// ```rust,ignore
/// let beta = kaiser_beta_from_attenuation(60.0);  // β ≈ 5.65
/// ```
pub fn kaiser_beta_from_attenuation(attenuation_db: f64) -> f64 {
    if attenuation_db > 50.0 {
        0.1102 * (attenuation_db - 8.7)
    } else if attenuation_db >= 21.0 {
        0.5842 * (attenuation_db - 21.0).powf(0.4) + 0.07886 * (attenuation_db - 21.0)
    } else {
        0.0
    }
}

/// Calculate required filter order for Kaiser window.
///
/// Given transition bandwidth and desired attenuation, computes
/// the minimum filter order needed.
///
/// # Arguments
/// * `transition_width` - Normalized transition bandwidth (0 to 0.5)
/// * `attenuation_db` - Desired stopband attenuation in dB
///
/// # Returns
/// Minimum filter order (number of taps - 1)
pub fn kaiser_order(transition_width: f64, attenuation_db: f64) -> usize {
    let num = attenuation_db - 7.95;
    let denom = 14.36 * transition_width;
    ((num / denom).ceil() as usize).max(1)
}

/// Modified Bessel function of the first kind, order 0.
///
/// Uses series expansion for accurate computation.
fn bessel_i0(x: f64) -> f64 {
    if x.abs() < 1e-10 {
        return 1.0;
    }

    let ax = x.abs();

    if ax < 3.75 {
        // Polynomial approximation for small x
        let t = (x / 3.75).powi(2);
        1.0 + t
            * (3.5156229
                + t * (3.0899424
                    + t * (1.2067492 + t * (0.2659732 + t * (0.0360768 + t * 0.0045813)))))
    } else {
        // Asymptotic approximation for large x
        let t = 3.75 / ax;
        (ax.exp() / ax.sqrt())
            * (0.39894228
                + t * (0.01328592
                    + t * (0.00225319
                        + t * (-0.00157565
                            + t * (0.00916281
                                + t * (-0.02057706
                                    + t * (0.02635537 + t * (-0.01647633 + t * 0.00392377))))))))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rectangular_window() {
        let w = rectangular_window(8);
        assert_eq!(w.len(), 8);
        assert!(w.iter().all(|&x| (x - 1.0).abs() < 1e-10));
    }

    #[test]
    fn test_hamming_window() {
        let w = hamming_window(8);
        assert_eq!(w.len(), 8);

        // Hamming window starts and ends near 0.08
        assert!((w[0] - 0.08).abs() < 0.01);
        assert!((w[7] - 0.08).abs() < 0.01);

        // Peak in the middle - for length 8, samples 3 and 4 are near peak
        let max_val = w.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(max_val > 0.9, "Hamming window should peak above 0.9");

        // Use odd length to get exact 1.0 peak
        let w_odd = hamming_window(9);
        let center_val = w_odd[4]; // Center sample
        assert!((center_val - 1.0).abs() < 0.01, "Odd-length Hamming center should be ~1.0");
    }

    #[test]
    fn test_hann_window() {
        let w = hann_window(8);
        assert_eq!(w.len(), 8);

        // Hann window is exactly 0 at endpoints
        assert!(w[0].abs() < 1e-10);
        assert!(w[7].abs() < 1e-10);
    }

    #[test]
    fn test_blackman_window() {
        let w = blackman_window(8);
        assert_eq!(w.len(), 8);

        // Blackman window is very small at endpoints
        assert!(w[0].abs() < 0.01);
        assert!(w[7].abs() < 0.01);
    }

    #[test]
    fn test_kaiser_window_beta_zero() {
        // β=0 should approximate rectangular
        let w = kaiser_window(8, 0.0);
        assert_eq!(w.len(), 8);

        // All values should be close to 1
        for &v in &w {
            assert!((v - 1.0).abs() < 0.1);
        }
    }

    #[test]
    fn test_kaiser_window_symmetry() {
        let w = kaiser_window(9, 5.0);
        assert_eq!(w.len(), 9);

        // Kaiser window should be symmetric
        for i in 0..4 {
            assert!(
                (w[i] - w[8 - i]).abs() < 1e-10,
                "Kaiser window should be symmetric"
            );
        }
    }

    #[test]
    fn test_kaiser_beta_from_attenuation() {
        // Low attenuation: β should be 0
        assert!((kaiser_beta_from_attenuation(20.0) - 0.0).abs() < 0.1);

        // 60 dB attenuation: β ≈ 5.65
        let beta_60 = kaiser_beta_from_attenuation(60.0);
        assert!(beta_60 > 5.0 && beta_60 < 6.0);

        // Higher attenuation should give higher β
        let beta_80 = kaiser_beta_from_attenuation(80.0);
        assert!(beta_80 > beta_60);
    }

    #[test]
    fn test_kaiser_order() {
        // Narrower transition = more taps
        let order_wide = kaiser_order(0.2, 60.0);
        let order_narrow = kaiser_order(0.1, 60.0);
        assert!(order_narrow > order_wide);

        // Higher attenuation = more taps
        let order_40db = kaiser_order(0.1, 40.0);
        let order_80db = kaiser_order(0.1, 80.0);
        assert!(order_80db > order_40db);
    }

    #[test]
    fn test_bessel_i0() {
        // I₀(0) = 1
        assert!((bessel_i0(0.0) - 1.0).abs() < 1e-10);

        // I₀(x) is always positive and increasing
        assert!(bessel_i0(1.0) > bessel_i0(0.0));
        assert!(bessel_i0(5.0) > bessel_i0(1.0));
    }

    #[test]
    fn test_window_enum() {
        let w = Window::Hamming;
        let coeffs = w.generate(64);
        assert_eq!(coeffs.len(), 64);

        let kaiser = Window::Kaiser(8.0);
        let kaiser_coeffs = kaiser.generate(64);
        assert_eq!(kaiser_coeffs.len(), 64);
    }

    #[test]
    fn test_window_coherent_gain() {
        let rect = Window::Rectangular.coherent_gain(64);
        assert!((rect - 1.0).abs() < 0.01);

        // Other windows have lower coherent gain
        let hamming = Window::Hamming.coherent_gain(64);
        assert!(hamming < rect);
    }

    #[test]
    fn test_blackman_harris_window() {
        let w = blackman_harris_window(64);
        assert_eq!(w.len(), 64);

        // Should be symmetric
        for i in 0..32 {
            assert!(
                (w[i] - w[63 - i]).abs() < 1e-10,
                "Blackman-Harris should be symmetric"
            );
        }
    }

    #[test]
    fn test_empty_windows() {
        assert!(rectangular_window(0).is_empty());
        assert!(hamming_window(0).is_empty());
        assert!(hann_window(0).is_empty());
        assert!(blackman_window(0).is_empty());
        assert!(kaiser_window(0, 5.0).is_empty());
    }

    #[test]
    fn test_single_sample_windows() {
        assert_eq!(rectangular_window(1), vec![1.0]);
        assert_eq!(hamming_window(1), vec![1.0]);
        assert_eq!(hann_window(1), vec![1.0]);
        assert_eq!(blackman_window(1), vec![1.0]);
        assert_eq!(kaiser_window(1, 5.0), vec![1.0]);
    }
}
