//! Noise Shaping for Quantization
//!
//! Pushes quantization noise out of the signal band using feedback filters.
//! This technique is fundamental to sigma-delta modulators and high-quality
//! audio DACs, where oversampling combined with noise shaping yields effective
//! resolution far beyond the raw quantizer bit depth.
//!
//! The noise transfer function (NTF) determines how quantization error is
//! spectrally redistributed. Higher-order shapers move more noise energy to
//! high frequencies at the cost of potential instability for large inputs.
//!
//! ## Shapers
//!
//! | Struct                     | Description                                |
//! |----------------------------|--------------------------------------------|
//! | `NoiseShaper`              | Standard error-feedback noise shaping       |
//! | `DitherNoiseShaper`        | Adds TPDF dither to decorrelate noise       |
//! | `SpectrumWeightedShaper`   | Custom noise transfer function coefficients |
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::noise_shaper::NoiseShaper;
//!
//! let mut shaper = NoiseShaper::new(2, 8);
//! let input = vec![0.3, 0.7, -0.2, 0.5];
//! let output = shaper.process(&input);
//! assert_eq!(output.len(), input.len());
//! // Output values are quantized to 2^8 = 256 levels
//! ```

/// Standard error-feedback noise shaper.
///
/// Applies an FIR error-feedback filter so that quantization noise is shaped
/// according to the noise transfer function `H(z) = 1 - sum(c_k * z^{-k})`.
/// Orders 1 through 3 use classical difference-based NTF coefficients.
#[derive(Debug, Clone)]
pub struct NoiseShaper {
    /// Filter order (number of feedback taps).
    pub order: usize,
    /// Error-feedback coefficients `[c_1, c_2, ..., c_order]`.
    pub coefficients: Vec<f64>,
    /// Circular buffer of past quantization errors.
    pub error_history: Vec<f64>,
    /// Quantizer bit depth.
    pub quantizer_bits: u32,
}

impl NoiseShaper {
    /// Create a new noise shaper with standard NTF coefficients.
    ///
    /// # Panics
    ///
    /// Panics if `order` is 0 or greater than 3 (use `SpectrumWeightedShaper`
    /// for custom NTF coefficients of arbitrary order).
    pub fn new(order: usize, quantizer_bits: u32) -> Self {
        let coefficients = noise_transfer_function(order);
        Self {
            order,
            coefficients,
            error_history: vec![0.0; order],
            quantizer_bits,
        }
    }

    /// Process a block of input samples through noise shaping and quantization.
    ///
    /// For each sample: add shaped error feedback, quantize, compute new
    /// quantization error, and shift it into the error history.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            // Add error feedback: x_shaped = x + sum(c_k * e[n-k])
            let mut shaped = x;
            for k in 0..self.order {
                shaped += self.coefficients[k] * self.error_history[k];
            }

            // Quantize
            let y = quantize(shaped, self.quantizer_bits);

            // Quantization error = quantized - shaped (what the quantizer added)
            let error = y - shaped;

            // Shift error history (most recent at index 0)
            for k in (1..self.order).rev() {
                self.error_history[k] = self.error_history[k - 1];
            }
            if self.order > 0 {
                self.error_history[0] = error;
            }

            output.push(y);
        }
        output
    }

    /// Clear the error history, resetting all feedback state to zero.
    pub fn reset(&mut self) {
        for e in &mut self.error_history {
            *e = 0.0;
        }
    }
}

/// Uniform mid-tread quantizer.
///
/// Maps a value in `[-1.0, 1.0]` to one of `2^bits` uniformly spaced levels
/// symmetric about zero. Values outside `[-1.0, 1.0]` are clamped.
pub fn quantize(value: f64, bits: u32) -> f64 {
    let levels = (1u64 << bits) as f64;
    let half = levels / 2.0;
    // Scale to [-half, half], round to nearest integer, scale back
    let clamped = value.clamp(-1.0, 1.0);
    let scaled = clamped * half;
    let rounded = scaled.round();
    // Clamp to valid range after rounding
    let clamped_rounded = rounded.clamp(-half, half - 1.0);
    clamped_rounded / half
}

// ---------------------------------------------------------------------------
// Simple LCG PRNG for dither generation (no external crate dependency)
// ---------------------------------------------------------------------------

/// Linear congruential generator (Numerical Recipes parameters).
#[derive(Debug, Clone)]
struct Lcg {
    state: u64,
}

impl Lcg {
    fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(1), // avoid zero state
        }
    }

    /// Return a uniform random value in `[0, 1)`.
    fn next_uniform(&mut self) -> f64 {
        // LCG with Numerical Recipes constants
        self.state = self
            .state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        // Use upper bits for better quality
        let upper = (self.state >> 11) as f64;
        upper / ((1u64 << 53) as f64)
    }

    /// Return a TPDF (triangular probability density function) dither sample
    /// in `[-amplitude, +amplitude]`.
    ///
    /// Formed by summing two independent uniform samples.
    fn next_tpdf(&mut self, amplitude: f64) -> f64 {
        let u1 = self.next_uniform();
        let u2 = self.next_uniform();
        // Sum of two uniform [-0.5, 0.5] gives triangular [-1, 1]
        let tri = (u1 - 0.5) + (u2 - 0.5);
        tri * amplitude
    }
}

// ---------------------------------------------------------------------------
// DitherNoiseShaper
// ---------------------------------------------------------------------------

/// Noise shaper with TPDF dither added before quantization.
///
/// Dithering decorrelates quantization noise from the signal, eliminating
/// distortion harmonics at the cost of a slight increase in noise floor.
/// Combined with noise shaping, the added noise is pushed out of band.
#[derive(Debug, Clone)]
pub struct DitherNoiseShaper {
    /// Underlying noise shaper state.
    order: usize,
    coefficients: Vec<f64>,
    error_history: Vec<f64>,
    quantizer_bits: u32,
    /// TPDF dither amplitude in LSBs (typically 1.0 for optimal results).
    dither_amplitude: f64,
    /// Internal PRNG for dither generation.
    rng: Lcg,
}

impl DitherNoiseShaper {
    /// Create a dithered noise shaper.
    ///
    /// `dither_amplitude` is specified relative to the quantizer step size.
    /// A value of 1.0 produces standard TPDF dither of +/-1 LSB.
    pub fn new(order: usize, quantizer_bits: u32, dither_amplitude: f64) -> Self {
        let coefficients = noise_transfer_function(order);
        Self {
            order,
            coefficients,
            error_history: vec![0.0; order],
            quantizer_bits,
            dither_amplitude,
            rng: Lcg::new(0xDEAD_BEEF_CAFE_1234),
        }
    }

    /// Process a block of input samples with dither and noise shaping.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let levels = (1u64 << self.quantizer_bits) as f64;
        let lsb = 2.0 / levels; // quantizer step size for [-1, 1] range

        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            // Error feedback
            let mut shaped = x;
            for k in 0..self.order {
                shaped += self.coefficients[k] * self.error_history[k];
            }

            // Add TPDF dither scaled to LSB
            let dither = self.rng.next_tpdf(self.dither_amplitude * lsb);
            let dithered = shaped + dither;

            // Quantize
            let y = quantize(dithered, self.quantizer_bits);

            // Error feedback: error relative to the un-dithered shaped signal
            let error = y - shaped;

            // Shift error history
            for k in (1..self.order).rev() {
                self.error_history[k] = self.error_history[k - 1];
            }
            if self.order > 0 {
                self.error_history[0] = error;
            }

            output.push(y);
        }
        output
    }
}

// ---------------------------------------------------------------------------
// SpectrumWeightedShaper
// ---------------------------------------------------------------------------

/// Noise shaper with a user-defined noise transfer function.
///
/// Allows custom NTF coefficients for application-specific noise spectral
/// shaping, such as psychoacoustic weighting curves for audio or band-limited
/// shaping for narrowband SDR signals.
#[derive(Debug, Clone)]
pub struct SpectrumWeightedShaper {
    /// NTF coefficients (arbitrary order).
    ntf_coefficients: Vec<f64>,
    /// Error feedback history.
    error_history: Vec<f64>,
    /// Quantizer bit depth.
    quantizer_bits: u32,
}

impl SpectrumWeightedShaper {
    /// Create a spectrum-weighted noise shaper with custom NTF coefficients.
    ///
    /// The coefficients define the error-feedback filter taps. The noise
    /// transfer function is `H(z) = 1 - sum(c_k * z^{-k})`.
    pub fn new(ntf_coefficients: Vec<f64>, quantizer_bits: u32) -> Self {
        let order = ntf_coefficients.len();
        Self {
            ntf_coefficients,
            error_history: vec![0.0; order],
            quantizer_bits,
        }
    }

    /// Process a block of input samples through custom-weighted noise shaping.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let order = self.ntf_coefficients.len();
        let mut output = Vec::with_capacity(input.len());

        for &x in input {
            let mut shaped = x;
            for k in 0..order {
                shaped += self.ntf_coefficients[k] * self.error_history[k];
            }

            let y = quantize(shaped, self.quantizer_bits);
            let error = y - shaped;

            for k in (1..order).rev() {
                self.error_history[k] = self.error_history[k - 1];
            }
            if order > 0 {
                self.error_history[0] = error;
            }

            output.push(y);
        }
        output
    }
}

/// Return the default noise transfer function coefficients for a given order.
///
/// These are the classical difference-based NTF coefficients derived from
/// `(1 - z^{-1})^N` expansion (binomial coefficients with alternating signs):
///
/// | Order | Coefficients      | NTF                    |
/// |-------|-------------------|------------------------|
/// | 1     | `[1.0]`           | `(1 - z^{-1})`        |
/// | 2     | `[2.0, -1.0]`     | `(1 - z^{-1})^2`      |
/// | 3     | `[3.0, -3.0, 1.0]`| `(1 - z^{-1})^3`      |
///
/// # Panics
///
/// Panics if `order` is 0 or greater than 3.
pub fn noise_transfer_function(order: usize) -> Vec<f64> {
    match order {
        1 => vec![1.0],
        2 => vec![2.0, -1.0],
        3 => vec![3.0, -3.0, 1.0],
        _ => panic!(
            "Unsupported noise shaping order {}; use SpectrumWeightedShaper for custom NTF",
            order
        ),
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_first_order() {
        let mut shaper = NoiseShaper::new(1, 8);
        let input: Vec<f64> = (0..100).map(|i| (i as f64 / 100.0) * 2.0 - 1.0).collect();
        let output = shaper.process(&input);

        assert_eq!(output.len(), input.len());
        // All outputs must be valid quantized levels within [-1, 1]
        for &y in &output {
            assert!(y >= -1.0 && y <= 1.0, "output {} out of range", y);
        }
    }

    #[test]
    fn test_second_order() {
        let mut shaper = NoiseShaper::new(2, 8);
        let input: Vec<f64> = (0..100).map(|i| (i as f64 / 100.0) * 2.0 - 1.0).collect();
        let output = shaper.process(&input);

        assert_eq!(output.len(), input.len());
        for &y in &output {
            assert!(y >= -1.0 && y <= 1.0, "output {} out of range", y);
        }
        // Verify second-order coefficients
        assert_eq!(shaper.coefficients, vec![2.0, -1.0]);
    }

    #[test]
    fn test_third_order() {
        let mut shaper = NoiseShaper::new(3, 8);
        let input: Vec<f64> = (0..100).map(|i| (i as f64 / 100.0) * 2.0 - 1.0).collect();
        let output = shaper.process(&input);

        assert_eq!(output.len(), input.len());
        assert_eq!(shaper.coefficients, vec![3.0, -3.0, 1.0]);
        assert_eq!(shaper.error_history.len(), 3);
    }

    #[test]
    fn test_quantize() {
        // 1-bit quantizer: 2 levels -> output is -1.0 or 0.0
        assert_eq!(quantize(0.6, 1), 1.0_f64.min(0.0)); // rounds to 1/1 = 1.0 but clamped

        // 2-bit: 4 levels, step = 0.5. Levels: -1.0, -0.5, 0.0, 0.5
        let q = quantize(0.0, 2);
        assert!((q - 0.0).abs() < 1e-10, "expected 0.0, got {}", q);

        let q = quantize(0.3, 2);
        assert!((q - 0.25).abs() < 1e-10 || (q - 0.5).abs() < 1e-10,
            "expected near 0.25 or 0.5, got {}", q);

        // 8-bit: 256 levels, step = 1/128
        let q = quantize(0.5, 8);
        let step = 1.0 / 128.0;
        let error = (q - 0.5).abs();
        assert!(error <= step, "8-bit quantize error {} > step {}", error, step);

        // Clamping: values beyond [-1, 1] are clamped
        let q_high = quantize(2.0, 8);
        assert!(q_high <= 1.0, "should clamp high values");
        let q_low = quantize(-2.0, 8);
        assert!(q_low >= -1.0, "should clamp low values");

        // Zero should quantize to zero
        assert!((quantize(0.0, 8) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_dc_signal() {
        // A constant DC input should produce output that tracks the input
        // closely; noise shaping preserves DC accuracy.
        let mut shaper = NoiseShaper::new(2, 8);
        let dc_level = 0.3;
        let input = vec![dc_level; 200];
        let output = shaper.process(&input);

        // Average of output should be close to the DC level
        let mean: f64 = output.iter().sum::<f64>() / output.len() as f64;
        let step = 2.0 / 256.0; // 8-bit step size
        assert!(
            (mean - dc_level).abs() < 2.0 * step,
            "DC tracking: mean {} vs expected {}, step {}",
            mean,
            dc_level,
            step
        );
    }

    #[test]
    fn test_reset() {
        let mut shaper = NoiseShaper::new(2, 8);
        // Use a value that does NOT land on an exact quantizer level so the
        // error history accumulates non-zero entries. 0.37 is between 8-bit
        // levels (step = 1/128 ~ 0.0078, nearest level = 0.3671875).
        let input = vec![0.37; 50];
        let _ = shaper.process(&input);

        // Error history should be non-zero after processing
        let has_nonzero = shaper.error_history.iter().any(|&e| e.abs() > 1e-15);
        assert!(has_nonzero, "error history should have non-zero values");

        // After reset, all errors should be zero
        shaper.reset();
        for &e in &shaper.error_history {
            assert_eq!(e, 0.0, "error history should be zero after reset");
        }

        // Processing the same input again after reset should give the
        // same result as a fresh shaper
        let mut fresh = NoiseShaper::new(2, 8);
        let out_reset = shaper.process(&input);
        let out_fresh = fresh.process(&input);
        assert_eq!(out_reset, out_fresh, "reset shaper should match fresh shaper");
    }

    #[test]
    fn test_dither() {
        let mut dithered = DitherNoiseShaper::new(2, 8, 1.0);
        let mut plain = NoiseShaper::new(2, 8);

        let input = vec![0.25; 500];
        let out_dither = dithered.process(&input);
        let out_plain = plain.process(&input);

        assert_eq!(out_dither.len(), input.len());

        // Dithered output should differ from plain (stochastic dither)
        let diffs: usize = out_dither
            .iter()
            .zip(out_plain.iter())
            .filter(|(&a, &b)| (a - b).abs() > 1e-15)
            .count();
        assert!(
            diffs > 0,
            "dithered output should differ from plain output"
        );

        // Both should have similar mean (DC tracking)
        let mean_d: f64 = out_dither.iter().sum::<f64>() / out_dither.len() as f64;
        let mean_p: f64 = out_plain.iter().sum::<f64>() / out_plain.len() as f64;
        assert!(
            (mean_d - 0.25).abs() < 0.05,
            "dithered mean {} too far from 0.25",
            mean_d
        );
        assert!(
            (mean_p - 0.25).abs() < 0.05,
            "plain mean {} too far from 0.25",
            mean_p
        );
    }

    #[test]
    fn test_custom_ntf() {
        // Use SpectrumWeightedShaper with first-order coefficients
        let mut custom = SpectrumWeightedShaper::new(vec![1.0], 8);
        let mut standard = NoiseShaper::new(1, 8);

        let input: Vec<f64> = (0..100).map(|i| (i as f64 / 100.0) * 2.0 - 1.0).collect();
        let out_custom = custom.process(&input);
        let out_standard = standard.process(&input);

        // With identical coefficients, outputs must match exactly
        assert_eq!(
            out_custom, out_standard,
            "custom NTF [1.0] should match first-order standard"
        );

        // Try a non-standard NTF (e.g., half-strength first-order)
        let mut half = SpectrumWeightedShaper::new(vec![0.5], 8);
        let out_half = half.process(&input);
        assert_eq!(out_half.len(), input.len());
    }

    #[test]
    fn test_noise_reduction_in_band() {
        // Noise shaping should concentrate quantization error at high
        // frequencies. We verify by low-pass filtering the error signal
        // (simple moving average) and checking that the shaped error has
        // lower in-band energy than plain (unshaped) quantization.
        let n = 4096;
        let input: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / n as f64;
                // Low-frequency sine (well within the baseband)
                0.5 * (2.0 * std::f64::consts::PI * 3.0 * t).sin()
            })
            .collect();

        // Plain quantization (no noise shaping) -- equivalent to order-0
        let plain_output: Vec<f64> = input.iter().map(|&x| quantize(x, 4)).collect();

        // Noise-shaped quantization (second order, 4-bit = coarse)
        let mut shaper = NoiseShaper::new(2, 4);
        let shaped_output = shaper.process(&input);

        // Compute error signals
        let plain_error: Vec<f64> = plain_output
            .iter()
            .zip(input.iter())
            .map(|(&o, &i)| o - i)
            .collect();
        let shaped_error: Vec<f64> = shaped_output
            .iter()
            .zip(input.iter())
            .map(|(&o, &i)| o - i)
            .collect();

        // Low-pass filter the error with a moving average (window = 64)
        // to extract in-band error energy
        let window = 64;
        let lpf_error = |error: &[f64]| -> f64 {
            let mut energy = 0.0;
            for start in 0..error.len().saturating_sub(window) {
                let block_mean: f64 =
                    error[start..start + window].iter().sum::<f64>() / window as f64;
                energy += block_mean * block_mean;
            }
            energy
        };

        let plain_inband = lpf_error(&plain_error);
        let shaped_inband = lpf_error(&shaped_error);

        // Noise shaping should reduce in-band error energy
        assert!(
            shaped_inband < plain_inband,
            "shaped in-band error {} should be less than plain {}",
            shaped_inband,
            plain_inband
        );
    }

    #[test]
    fn test_empty_input() {
        let mut shaper = NoiseShaper::new(1, 8);
        let output = shaper.process(&[]);
        assert!(output.is_empty(), "empty input should produce empty output");

        let mut dithered = DitherNoiseShaper::new(2, 8, 1.0);
        let output = dithered.process(&[]);
        assert!(output.is_empty());

        let mut custom = SpectrumWeightedShaper::new(vec![1.0, -0.5], 8);
        let output = custom.process(&[]);
        assert!(output.is_empty());
    }
}
