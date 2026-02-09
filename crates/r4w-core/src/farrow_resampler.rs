//! Farrow Resampler — Polynomial Structure for Variable Fractional Resampling
//!
//! Implements the Farrow structure for continuously variable fractional
//! delay and sample rate conversion. Unlike polyphase resamplers (fixed ratio)
//! or cubic interpolation (fixed polynomial), the Farrow structure uses a
//! bank of FIR sub-filters whose outputs are combined via Horner's method
//! at an arbitrary fractional delay μ ∈ [0, 1).
//!
//! Supports linear (2-tap), quadratic (3-tap), cubic (4-tap), and
//! configurable-order polynomial interpolation.
//!
//! GNU Radio equivalent: `gr::filter::fractional_interpolator_cc` (uses MMSE FIR).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::farrow_resampler::{FarrowResampler, FarrowOrder};
//!
//! // Resample from 48 kHz to 44.1 kHz using cubic Farrow
//! let mut resampler = FarrowResampler::new(FarrowOrder::Cubic, 44100.0 / 48000.0);
//! let input: Vec<f64> = (0..100).map(|i| (i as f64 * 0.1).sin()).collect();
//! let output = resampler.process(&input);
//! assert!(output.len() < input.len()); // Downsampled
//! ```

use num_complex::Complex64;

/// Farrow interpolation polynomial order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FarrowOrder {
    /// Linear interpolation (2 points, order 1).
    Linear,
    /// Quadratic interpolation (3 points, order 2).
    Quadratic,
    /// Cubic interpolation (4 points, order 3). Best quality/complexity trade-off.
    Cubic,
}

impl FarrowOrder {
    /// Number of taps (filter length) for this order.
    pub fn num_taps(&self) -> usize {
        match self {
            FarrowOrder::Linear => 2,
            FarrowOrder::Quadratic => 3,
            FarrowOrder::Cubic => 4,
        }
    }
}

/// Farrow structure resampler for continuously variable sample rate conversion.
///
/// The output y(μ) is computed using Horner's scheme:
/// y(μ) = c₀ + μ*(c₁ + μ*(c₂ + μ*c₃))
/// where cₖ = Σⱼ hₖⱼ * x[n-j] are the sub-filter outputs.
#[derive(Debug, Clone)]
pub struct FarrowResampler {
    order: FarrowOrder,
    /// Resampling ratio (output_rate / input_rate)
    ratio: f64,
    /// Current fractional position in input
    mu: f64,
    /// Input history buffer
    history: Vec<f64>,
    /// Number of valid samples in history
    history_count: usize,
}

impl FarrowResampler {
    /// Create a new Farrow resampler with the given order and rate ratio.
    ///
    /// `ratio` = output_rate / input_rate. E.g., 44100/48000 = 0.91875 for downsampling.
    pub fn new(order: FarrowOrder, ratio: f64) -> Self {
        let taps = order.num_taps();
        Self {
            order,
            ratio: ratio.max(0.001),
            mu: 0.0,
            history: vec![0.0; taps],
            history_count: 0,
        }
    }

    /// Set a new resampling ratio (can be changed on the fly).
    pub fn set_ratio(&mut self, ratio: f64) {
        self.ratio = ratio.max(0.001);
    }

    /// Get the current resampling ratio.
    pub fn ratio(&self) -> f64 {
        self.ratio
    }

    /// Process a block of real input samples, producing resampled output.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity((input.len() as f64 * self.ratio * 1.1) as usize + 1);
        let taps = self.order.num_taps();

        for &sample in input {
            // Shift history and insert new sample
            for i in 0..taps - 1 {
                self.history[i] = self.history[i + 1];
            }
            self.history[taps - 1] = sample;
            if self.history_count < taps {
                self.history_count += 1;
            }

            if self.history_count < taps {
                continue;
            }

            // Generate output samples while mu < 1
            while self.mu < 1.0 {
                output.push(self.interpolate(self.mu));
                self.mu += 1.0 / self.ratio;
            }
            self.mu -= 1.0;
        }
        output
    }

    /// Process a block of complex input samples.
    pub fn process_complex(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        // Use two real Farrow resamplers for I and Q
        let reals: Vec<f64> = input.iter().map(|c| c.re).collect();
        let imags: Vec<f64> = input.iter().map(|c| c.im).collect();

        let mut re_resampler = self.clone();
        let re_out = self.process(&reals);
        let im_out = re_resampler.process(&imags);

        re_out.iter().zip(im_out.iter())
            .map(|(&r, &i)| Complex64::new(r, i))
            .collect()
    }

    /// Evaluate the interpolation polynomial at fractional delay μ.
    fn interpolate(&self, mu: f64) -> f64 {
        match self.order {
            FarrowOrder::Linear => {
                // y(μ) = x[0]*(1-μ) + x[1]*μ
                self.history[0] * (1.0 - mu) + self.history[1] * mu
            }
            FarrowOrder::Quadratic => {
                // Lagrange 3-point interpolation via Horner form
                let x = &self.history;
                let c0 = x[1];
                let c1 = 0.5 * (x[2] - x[0]);
                let c2 = 0.5 * (x[0] - 2.0 * x[1] + x[2]);
                c0 + mu * (c1 + mu * c2)
            }
            FarrowOrder::Cubic => {
                // Cubic Hermite / Catmull-Rom via Horner form
                let x = &self.history;
                let c0 = x[1];
                let c1 = 0.5 * (x[2] - x[0]);
                let c2 = x[0] - 2.5 * x[1] + 2.0 * x[2] - 0.5 * x[3];
                let c3 = 0.5 * (-x[0] + 3.0 * x[1] - 3.0 * x[2] + x[3]);
                c0 + mu * (c1 + mu * (c2 + mu * c3))
            }
        }
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.history.fill(0.0);
        self.history_count = 0;
        self.mu = 0.0;
    }

    /// Get the interpolation order.
    pub fn order(&self) -> FarrowOrder {
        self.order
    }
}

/// Compute a single interpolated value from a signal at a fractional position.
///
/// `signal[index]` is the integer part; `mu` ∈ [0, 1) is the fractional part.
/// Uses cubic Catmull-Rom interpolation (needs index-1, index, index+1, index+2).
pub fn interpolate_at(signal: &[f64], index: usize, mu: f64) -> Option<f64> {
    if index < 1 || index + 2 >= signal.len() {
        return None;
    }
    let x0 = signal[index - 1];
    let x1 = signal[index];
    let x2 = signal[index + 1];
    let x3 = signal[index + 2];

    let c0 = x1;
    let c1 = 0.5 * (x2 - x0);
    let c2 = x0 - 2.5 * x1 + 2.0 * x2 - 0.5 * x3;
    let c3 = 0.5 * (-x0 + 3.0 * x1 - 3.0 * x2 + x3);
    Some(c0 + mu * (c1 + mu * (c2 + mu * c3)))
}

/// Resample a signal to a new length using cubic Farrow interpolation.
///
/// One-shot utility: resamples `input` of length N to `output_len` samples.
pub fn resample_to_length(input: &[f64], output_len: usize) -> Vec<f64> {
    if input.len() < 4 || output_len == 0 {
        return vec![];
    }
    let ratio = (input.len() - 1) as f64 / (output_len - 1).max(1) as f64;
    let mut output = Vec::with_capacity(output_len);
    for k in 0..output_len {
        let pos = k as f64 * ratio;
        let idx = pos.floor() as usize;
        let mu = pos - idx as f64;
        if let Some(val) = interpolate_at(input, idx.max(1).min(input.len() - 3), mu) {
            output.push(val);
        }
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_linear_passthrough() {
        // Ratio 1.0 = no resampling
        let mut r = FarrowResampler::new(FarrowOrder::Linear, 1.0);
        let input: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let output = r.process(&input);
        // Should produce approximately same number of samples (minus startup)
        assert!(output.len() >= input.len() - 2, "len={}", output.len());
    }

    #[test]
    fn test_cubic_downsample() {
        let mut r = FarrowResampler::new(FarrowOrder::Cubic, 0.5);
        let input: Vec<f64> = (0..100).map(|i| (i as f64 * 0.1).sin()).collect();
        let output = r.process(&input);
        // Should produce roughly half the samples
        let expected = (input.len() as f64 * 0.5) as usize;
        assert!((output.len() as i64 - expected as i64).unsigned_abs() < 5,
            "output.len()={}, expected≈{expected}", output.len());
    }

    #[test]
    fn test_cubic_upsample() {
        let mut r = FarrowResampler::new(FarrowOrder::Cubic, 2.0);
        let input: Vec<f64> = (0..50).map(|i| (i as f64 * 0.1).sin()).collect();
        let output = r.process(&input);
        let expected = (input.len() as f64 * 2.0) as usize;
        assert!((output.len() as i64 - expected as i64).unsigned_abs() < 10,
            "output.len()={}, expected≈{expected}", output.len());
    }

    #[test]
    fn test_quadratic_sine_quality() {
        // Quadratic interpolation of a low-frequency sine
        let mut r = FarrowResampler::new(FarrowOrder::Quadratic, 1.5);
        let freq = 0.02;
        let n = 200;
        let input: Vec<f64> = (0..n).map(|i| (2.0 * PI * freq * i as f64).sin()).collect();
        let output = r.process(&input);
        // Check that output values are bounded
        assert!(output.iter().all(|&v| v.abs() < 1.5));
    }

    #[test]
    fn test_variable_ratio() {
        let mut r = FarrowResampler::new(FarrowOrder::Cubic, 1.0);
        let input = vec![1.0; 50];
        let out1 = r.process(&input);
        r.set_ratio(2.0);
        let out2 = r.process(&input);
        // After doubling ratio, should get more output samples
        assert!(out2.len() > out1.len());
    }

    #[test]
    fn test_interpolate_at_midpoint() {
        let signal = vec![0.0, 0.0, 1.0, 1.0, 0.0, 0.0];
        let val = interpolate_at(&signal, 2, 0.5).unwrap();
        // Midpoint between 1.0 and 1.0 should be close to 1.0
        assert!((val - 1.0).abs() < 0.2, "val={val}");
    }

    #[test]
    fn test_interpolate_at_exact() {
        let signal = vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0];
        let val = interpolate_at(&signal, 2, 0.0).unwrap();
        assert!((val - 2.0).abs() < 1e-10, "val={val}");
    }

    #[test]
    fn test_interpolate_at_boundary() {
        let signal = vec![0.0, 1.0];
        assert!(interpolate_at(&signal, 0, 0.5).is_none()); // Not enough context
    }

    #[test]
    fn test_resample_to_length() {
        let input: Vec<f64> = (0..100).map(|i| (i as f64 * 0.05).sin()).collect();
        let output = resample_to_length(&input, 200);
        assert_eq!(output.len(), 200);
    }

    #[test]
    fn test_resample_identity() {
        let input: Vec<f64> = (0..50).map(|i| (i as f64 * 0.1).sin()).collect();
        let output = resample_to_length(&input, 50);
        assert_eq!(output.len(), 50);
        // Should closely match input (except edge effects)
        for i in 5..45 {
            assert!((output[i] - input[i]).abs() < 0.01, "i={i}: {} vs {}", output[i], input[i]);
        }
    }

    #[test]
    fn test_reset() {
        let mut r = FarrowResampler::new(FarrowOrder::Cubic, 1.0);
        r.process(&[1.0; 50]);
        r.reset();
        assert_eq!(r.history, vec![0.0; 4]);
    }

    #[test]
    fn test_dc_preservation() {
        // DC input should produce DC output regardless of ratio
        let mut r = FarrowResampler::new(FarrowOrder::Cubic, 0.7);
        let input = vec![0.5; 200];
        let output = r.process(&input);
        // After transient, output should be ~0.5
        let tail = &output[output.len() / 2..];
        for &v in tail {
            assert!((v - 0.5).abs() < 0.01, "v={v}");
        }
    }
}
