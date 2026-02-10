//! Programmable uniform scalar quantization for ADC modeling and quantization noise analysis.
//!
//! This module provides a configurable [`UniformScalarQuantizer`] that supports mid-tread
//! and mid-riser quantization modes, multiple overflow handling strategies, complex (I/Q)
//! signal quantization, dithering, and quantization performance metrics (SQNR, ENOB).
//!
//! # Example
//!
//! ```
//! use r4w_core::uniform_scalar_quantizer::{UniformScalarQuantizer, QuantizerMode, OverflowMode};
//!
//! // Create an 8-bit mid-tread quantizer with full-scale range [-1.0, 1.0]
//! let q = UniformScalarQuantizer::new(8, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
//!
//! // Quantize a single sample
//! let quantized = q.quantize(0.3);
//! assert!((quantized - 0.3).abs() < q.step_size());
//!
//! // Quantize an entire signal and measure SQNR
//! let signal: Vec<f64> = (0..1024).map(|i| 0.5 * (2.0 * std::f64::consts::PI * i as f64 / 64.0).sin()).collect();
//! let quantized_signal = q.quantize_slice(&signal);
//! let sqnr = q.measure_sqnr(&signal, &quantized_signal);
//! assert!(sqnr > 40.0); // 8-bit quantizer should achieve ~49 dB SQNR for sinusoid
//! ```

use std::f64::consts::PI;

/// Quantizer topology: mid-tread places zero as a reconstruction level;
/// mid-riser places zero as a decision boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QuantizerMode {
    /// Zero is a reconstruction level (odd number of symmetric levels around zero).
    MidTread,
    /// Zero is a decision boundary (even number of symmetric levels around zero).
    MidRiser,
}

/// Strategy for handling samples that exceed the full-scale range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverflowMode {
    /// Clamp to the nearest extremal reconstruction level.
    Clip,
    /// Wrap around using modular arithmetic.
    Wrap,
    /// Identical to Clip (alias for ADC-style saturation).
    Saturate,
}

/// Statistics collected from a quantization error sequence.
#[derive(Debug, Clone)]
pub struct QuantizationErrorStats {
    /// Mean of the quantization error.
    pub mean: f64,
    /// Variance of the quantization error.
    pub variance: f64,
    /// Minimum error value.
    pub min: f64,
    /// Maximum error value.
    pub max: f64,
    /// Histogram bin edges (length = `num_bins + 1`).
    pub histogram_edges: Vec<f64>,
    /// Histogram counts (length = `num_bins`).
    pub histogram_counts: Vec<usize>,
}

/// A programmable uniform scalar quantizer.
///
/// Models an ideal ADC with configurable bit depth, full-scale range,
/// quantization mode, and overflow handling.
#[derive(Debug, Clone)]
pub struct UniformScalarQuantizer {
    bits: u32,
    full_scale: f64,
    mode: QuantizerMode,
    overflow: OverflowMode,
    dither_enabled: bool,
    /// Simple LCG state for deterministic dither generation.
    dither_state: u64,
    num_levels: u64,
    step: f64,
}

impl UniformScalarQuantizer {
    /// Create a new quantizer.
    ///
    /// * `bits` – bit depth (1..=32).
    /// * `full_scale` – the positive full-scale amplitude. The input range is `[-full_scale, full_scale)`.
    /// * `mode` – mid-tread or mid-riser topology.
    /// * `overflow` – how to handle out-of-range samples.
    pub fn new(bits: u32, full_scale: f64, mode: QuantizerMode, overflow: OverflowMode) -> Self {
        assert!(bits >= 1 && bits <= 32, "bits must be in 1..=32");
        assert!(full_scale > 0.0, "full_scale must be positive");
        let num_levels = 1u64 << bits;
        let step = 2.0 * full_scale / num_levels as f64;
        Self {
            bits,
            full_scale,
            mode,
            overflow,
            dither_enabled: false,
            dither_state: 0x1234_5678_9ABC_DEF0,
            num_levels,
            step,
        }
    }

    // ------------------------------------------------------------------ accessors

    /// Quantization step size (Δ).
    pub fn step_size(&self) -> f64 {
        self.step
    }

    /// Number of quantization levels (2^bits).
    pub fn num_levels(&self) -> u64 {
        self.num_levels
    }

    /// Bit depth.
    pub fn bits(&self) -> u32 {
        self.bits
    }

    /// Full-scale amplitude.
    pub fn full_scale(&self) -> f64 {
        self.full_scale
    }

    /// Return all decision boundaries (the thresholds between adjacent levels).
    pub fn decision_levels(&self) -> Vec<f64> {
        let n = self.num_levels as usize;
        match self.mode {
            QuantizerMode::MidTread => {
                // Reconstruction levels are k*step for k in [-(n/2), n/2-1]
                // Decision boundaries sit at (k + 0.5)*step
                (0..n - 1)
                    .map(|i| {
                        let k = i as f64 - (n as f64 / 2.0);
                        (k + 0.5) * self.step
                    })
                    .collect()
            }
            QuantizerMode::MidRiser => {
                // Reconstruction levels at (k + 0.5)*step
                // Decision boundaries at (k + 1)*step, i.e. k*step for integer k
                (0..n - 1)
                    .map(|i| {
                        let k = i as f64 - (n as f64 / 2.0) + 1.0;
                        k * self.step
                    })
                    .collect()
            }
        }
    }

    /// Return all reconstruction levels.
    pub fn reconstruction_levels(&self) -> Vec<f64> {
        let n = self.num_levels as usize;
        match self.mode {
            QuantizerMode::MidTread => {
                (0..n)
                    .map(|i| (i as f64 - n as f64 / 2.0) * self.step)
                    .collect()
            }
            QuantizerMode::MidRiser => {
                (0..n)
                    .map(|i| (i as f64 - n as f64 / 2.0 + 0.5) * self.step)
                    .collect()
            }
        }
    }

    // ------------------------------------------------------------------ dither

    /// Enable or disable triangular-PDF dither.
    pub fn set_dither(&mut self, enabled: bool) {
        self.dither_enabled = enabled;
    }

    /// Seed the internal dither PRNG for reproducibility.
    pub fn seed_dither(&mut self, seed: u64) {
        self.dither_state = seed | 1; // ensure odd for LCG
    }

    /// Generate one triangular-PDF dither sample in [-step, step] via sum of two uniform.
    fn next_dither(&mut self) -> f64 {
        let u1 = self.next_uniform();
        let u2 = self.next_uniform();
        // TPDF: sum of two independent Uniform[-Δ/2, Δ/2]
        let d1 = (u1 - 0.5) * self.step;
        let d2 = (u2 - 0.5) * self.step;
        d1 + d2
    }

    /// Simple LCG returning a value in [0, 1).
    fn next_uniform(&mut self) -> f64 {
        // Numerical Recipes LCG
        self.dither_state = self
            .dither_state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.dither_state >> 33) as f64 / (1u64 << 31) as f64
    }

    // ------------------------------------------------------------------ core quantize

    /// Quantize a single real-valued sample.
    pub fn quantize(&self, x: f64) -> f64 {
        self.quantize_inner(x, 0.0)
    }

    /// Quantize with a caller-supplied dither value (useful for external dither).
    fn quantize_inner(&self, x: f64, dither: f64) -> f64 {
        let xd = x + dither;
        let index = self.to_index(xd);
        self.index_to_level(index)
    }

    /// Map a continuous value to a quantization index in `[0, num_levels)`.
    fn to_index(&self, x: f64) -> i64 {
        let n = self.num_levels as i64;

        let raw = match self.mode {
            QuantizerMode::MidTread => {
                // Round to nearest integer multiple of step
                ((x / self.step).round()) as i64 + n / 2
            }
            QuantizerMode::MidRiser => {
                // Floor-based: index = floor(x / step + n/2)
                ((x / self.step) + (n as f64 / 2.0)).floor() as i64
            }
        };

        match self.overflow {
            OverflowMode::Clip | OverflowMode::Saturate => raw.max(0).min(n - 1),
            OverflowMode::Wrap => ((raw % n) + n) % n,
        }
    }

    /// Convert a quantization index back to a reconstruction level.
    fn index_to_level(&self, idx: i64) -> f64 {
        let n = self.num_levels as f64;
        match self.mode {
            QuantizerMode::MidTread => (idx as f64 - n / 2.0) * self.step,
            QuantizerMode::MidRiser => (idx as f64 - n / 2.0 + 0.5) * self.step,
        }
    }

    // ------------------------------------------------------------------ batch

    /// Quantize a slice of real-valued samples.
    pub fn quantize_slice(&self, signal: &[f64]) -> Vec<f64> {
        if self.dither_enabled {
            // Need mutable clone for dither state
            let mut q = self.clone();
            signal
                .iter()
                .map(|&x| {
                    let d = q.next_dither();
                    q.quantize_inner(x, d)
                })
                .collect()
        } else {
            signal.iter().map(|&x| self.quantize(x)).collect()
        }
    }

    /// Quantize a slice of real-valued samples, advancing internal dither state.
    pub fn quantize_slice_mut(&mut self, signal: &[f64]) -> Vec<f64> {
        signal
            .iter()
            .map(|&x| {
                let d = if self.dither_enabled {
                    self.next_dither()
                } else {
                    0.0
                };
                self.quantize_inner(x, d)
            })
            .collect()
    }

    /// Quantize a complex (I/Q) sample, applying quantization independently to I and Q.
    pub fn quantize_complex(&self, iq: (f64, f64)) -> (f64, f64) {
        (self.quantize(iq.0), self.quantize(iq.1))
    }

    /// Quantize a slice of complex (I/Q) samples.
    pub fn quantize_complex_slice(&self, signal: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if self.dither_enabled {
            let mut q = self.clone();
            signal
                .iter()
                .map(|&(i, qv)| {
                    let di = q.next_dither();
                    let dq = q.next_dither();
                    (q.quantize_inner(i, di), q.quantize_inner(qv, dq))
                })
                .collect()
        } else {
            signal
                .iter()
                .map(|&iq| self.quantize_complex(iq))
                .collect()
        }
    }

    // ------------------------------------------------------------------ metrics

    /// Compute SQNR (Signal-to-Quantization-Noise Ratio) in dB.
    ///
    /// `original` and `quantized` must have the same length.
    pub fn measure_sqnr(&self, original: &[f64], quantized: &[f64]) -> f64 {
        assert_eq!(original.len(), quantized.len());
        let signal_power: f64 =
            original.iter().map(|&x| x * x).sum::<f64>() / original.len() as f64;
        let noise_power: f64 = original
            .iter()
            .zip(quantized.iter())
            .map(|(&o, &q)| {
                let e = o - q;
                e * e
            })
            .sum::<f64>()
            / original.len() as f64;
        if noise_power == 0.0 {
            return f64::INFINITY;
        }
        10.0 * (signal_power / noise_power).log10()
    }

    /// Compute SQNR for complex signals.
    pub fn measure_sqnr_complex(
        &self,
        original: &[(f64, f64)],
        quantized: &[(f64, f64)],
    ) -> f64 {
        assert_eq!(original.len(), quantized.len());
        let signal_power: f64 = original
            .iter()
            .map(|&(i, q)| i * i + q * q)
            .sum::<f64>()
            / original.len() as f64;
        let noise_power: f64 = original
            .iter()
            .zip(quantized.iter())
            .map(|(&(oi, oq), &(qi, qq))| {
                let ei = oi - qi;
                let eq_ = oq - qq;
                ei * ei + eq_ * eq_
            })
            .sum::<f64>()
            / original.len() as f64;
        if noise_power == 0.0 {
            return f64::INFINITY;
        }
        10.0 * (signal_power / noise_power).log10()
    }

    /// Compute ENOB (Effective Number of Bits) from a measured SQNR (in dB).
    ///
    /// Uses the standard formula: ENOB = (SQNR - 1.76) / 6.02
    pub fn enob_from_sqnr(sqnr_db: f64) -> f64 {
        (sqnr_db - 1.76) / 6.02
    }

    /// Compute ENOB by quantizing a full-scale sinusoid internally.
    pub fn measure_enob(&self) -> f64 {
        let n = 8192;
        // Use a non-integer number of cycles to avoid spectral leakage alignment
        let cycles = 101.0;
        let signal: Vec<f64> = (0..n)
            .map(|i| self.full_scale * 0.99 * (2.0 * PI * cycles * i as f64 / n as f64).sin())
            .collect();
        let quantized = self.quantize_slice(&signal);
        let sqnr = self.measure_sqnr(&signal, &quantized);
        Self::enob_from_sqnr(sqnr)
    }

    // ------------------------------------------------------------------ error stats

    /// Compute statistics over the quantization error sequence.
    ///
    /// * `num_bins` – number of histogram bins.
    pub fn error_statistics(
        &self,
        original: &[f64],
        quantized: &[f64],
        num_bins: usize,
    ) -> QuantizationErrorStats {
        assert_eq!(original.len(), quantized.len());
        assert!(num_bins > 0);

        let errors: Vec<f64> = original
            .iter()
            .zip(quantized.iter())
            .map(|(&o, &q)| o - q)
            .collect();

        let n = errors.len() as f64;
        let mean = errors.iter().sum::<f64>() / n;
        let variance = errors.iter().map(|&e| (e - mean) * (e - mean)).sum::<f64>() / n;
        let min = errors.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = errors.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        // Build histogram
        let range = if (max - min).abs() < f64::EPSILON {
            1.0
        } else {
            max - min
        };
        let bin_width = range / num_bins as f64;
        let mut histogram_edges = Vec::with_capacity(num_bins + 1);
        for i in 0..=num_bins {
            histogram_edges.push(min + i as f64 * bin_width);
        }
        let mut histogram_counts = vec![0usize; num_bins];
        for &e in &errors {
            let mut bin = ((e - min) / bin_width).floor() as usize;
            if bin >= num_bins {
                bin = num_bins - 1;
            }
            histogram_counts[bin] += 1;
        }

        QuantizationErrorStats {
            mean,
            variance,
            min,
            max,
            histogram_edges,
            histogram_counts,
        }
    }

    /// Theoretical SQNR for a full-scale sinusoidal input: 6.02*N + 1.76 dB.
    pub fn theoretical_sqnr_sinusoid(&self) -> f64 {
        6.02 * self.bits as f64 + 1.76
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // -------------------------------------------------------------- construction

    #[test]
    fn test_new_basic() {
        let q = UniformScalarQuantizer::new(8, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
        assert_eq!(q.bits(), 8);
        assert_eq!(q.num_levels(), 256);
        assert!(approx_eq(q.full_scale(), 1.0, 1e-15));
    }

    #[test]
    fn test_step_size() {
        let q = UniformScalarQuantizer::new(4, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
        // step = 2*1.0 / 16 = 0.125
        assert!(approx_eq(q.step_size(), 0.125, 1e-15));
    }

    #[test]
    #[should_panic(expected = "bits must be in 1..=32")]
    fn test_zero_bits_panics() {
        UniformScalarQuantizer::new(0, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
    }

    #[test]
    #[should_panic(expected = "full_scale must be positive")]
    fn test_negative_full_scale_panics() {
        UniformScalarQuantizer::new(8, -1.0, QuantizerMode::MidTread, OverflowMode::Clip);
    }

    // -------------------------------------------------------------- mid-tread

    #[test]
    fn test_mid_tread_zero_is_reconstruction_level() {
        let q = UniformScalarQuantizer::new(4, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
        let levels = q.reconstruction_levels();
        assert!(
            levels.iter().any(|&l| approx_eq(l, 0.0, 1e-15)),
            "Mid-tread must have 0 as a reconstruction level, got: {:?}",
            levels
        );
    }

    #[test]
    fn test_mid_tread_quantize_near_zero() {
        let q = UniformScalarQuantizer::new(8, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
        // Small positive value near zero should quantize close to zero
        let result = q.quantize(0.001);
        assert!(result.abs() < q.step_size());
    }

    #[test]
    fn test_mid_tread_symmetry() {
        let q = UniformScalarQuantizer::new(4, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
        // Quantizing symmetric values should give symmetric reconstruction levels
        let pos = q.quantize(0.3);
        let neg = q.quantize(-0.3);
        assert!(
            approx_eq(pos, -neg, 1e-15),
            "Mid-tread should be symmetric: q(0.3)={}, q(-0.3)={}",
            pos,
            neg
        );
    }

    // -------------------------------------------------------------- mid-riser

    #[test]
    fn test_mid_riser_zero_is_decision_boundary() {
        let q = UniformScalarQuantizer::new(4, 1.0, QuantizerMode::MidRiser, OverflowMode::Clip);
        let levels = q.reconstruction_levels();
        // Zero should NOT be a reconstruction level for mid-riser
        assert!(
            !levels.iter().any(|&l| approx_eq(l, 0.0, 1e-15)),
            "Mid-riser must NOT have 0 as a reconstruction level, got: {:?}",
            levels
        );
    }

    #[test]
    fn test_mid_riser_small_positive_maps_above_zero() {
        let q = UniformScalarQuantizer::new(4, 1.0, QuantizerMode::MidRiser, OverflowMode::Clip);
        let result = q.quantize(0.01);
        assert!(
            result > 0.0,
            "Mid-riser: small positive input should map to positive level, got {}",
            result
        );
    }

    // -------------------------------------------------------------- overflow modes

    #[test]
    fn test_clip_overflow() {
        let q = UniformScalarQuantizer::new(4, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
        let levels = q.reconstruction_levels();
        let max_level = levels.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min_level = levels.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!(approx_eq(q.quantize(100.0), max_level, 1e-15));
        assert!(approx_eq(q.quantize(-100.0), min_level, 1e-15));
    }

    #[test]
    fn test_saturate_same_as_clip() {
        let q_clip =
            UniformScalarQuantizer::new(4, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
        let q_sat =
            UniformScalarQuantizer::new(4, 1.0, QuantizerMode::MidTread, OverflowMode::Saturate);
        for &x in &[-5.0, -1.5, 0.0, 1.5, 5.0] {
            assert!(approx_eq(q_clip.quantize(x), q_sat.quantize(x), 1e-15));
        }
    }

    #[test]
    fn test_wrap_overflow() {
        let q = UniformScalarQuantizer::new(4, 1.0, QuantizerMode::MidTread, OverflowMode::Wrap);
        let levels = q.reconstruction_levels();
        // The wrapped output must be one of the valid reconstruction levels
        let result = q.quantize(1.5);
        assert!(
            levels.iter().any(|&l| approx_eq(l, result, 1e-12)),
            "Wrap result {} is not a valid reconstruction level",
            result
        );
    }

    // -------------------------------------------------------------- complex

    #[test]
    fn test_complex_quantization() {
        let q = UniformScalarQuantizer::new(8, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
        let iq = (0.3, -0.7);
        let (qi, qq) = q.quantize_complex(iq);
        assert!((qi - 0.3).abs() < q.step_size());
        assert!((qq - (-0.7)).abs() < q.step_size());
    }

    #[test]
    fn test_complex_slice_quantization() {
        let q = UniformScalarQuantizer::new(8, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
        let signal: Vec<(f64, f64)> = (0..100)
            .map(|i| {
                let t = i as f64 / 100.0;
                (
                    (2.0 * PI * t).sin() * 0.5,
                    (2.0 * PI * t).cos() * 0.5,
                )
            })
            .collect();
        let quantized = q.quantize_complex_slice(&signal);
        assert_eq!(quantized.len(), signal.len());
        for (&(oi, oq), &(qi, qq)) in signal.iter().zip(quantized.iter()) {
            assert!((oi - qi).abs() < q.step_size());
            assert!((oq - qq).abs() < q.step_size());
        }
    }

    // -------------------------------------------------------------- SQNR & ENOB

    #[test]
    fn test_sqnr_8bit_sinusoid() {
        let q = UniformScalarQuantizer::new(8, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
        let n = 8192;
        let signal: Vec<f64> = (0..n)
            .map(|i| 0.99 * (2.0 * PI * 101.0 * i as f64 / n as f64).sin())
            .collect();
        let quantized = q.quantize_slice(&signal);
        let sqnr = q.measure_sqnr(&signal, &quantized);
        // Theoretical: 6.02*8 + 1.76 = 49.92 dB. Allow some margin.
        assert!(
            sqnr > 44.0 && sqnr < 55.0,
            "8-bit SQNR should be ~49.9 dB, got {} dB",
            sqnr
        );
    }

    #[test]
    fn test_sqnr_complex() {
        let q =
            UniformScalarQuantizer::new(10, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
        let n = 4096;
        let signal: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / n as f64;
                (
                    0.5 * (2.0 * PI * 73.0 * t).sin(),
                    0.5 * (2.0 * PI * 73.0 * t).cos(),
                )
            })
            .collect();
        let quantized = q.quantize_complex_slice(&signal);
        let sqnr = q.measure_sqnr_complex(&signal, &quantized);
        assert!(
            sqnr > 50.0,
            "10-bit complex SQNR should be high, got {} dB",
            sqnr
        );
    }

    #[test]
    fn test_enob_from_sqnr() {
        // 49.92 dB -> (49.92 - 1.76)/6.02 = 8.0
        let enob = UniformScalarQuantizer::enob_from_sqnr(49.92);
        assert!(approx_eq(enob, 8.0, 0.01));
    }

    #[test]
    fn test_measure_enob() {
        let q =
            UniformScalarQuantizer::new(12, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
        let enob = q.measure_enob();
        // ENOB should be close to 12 for an ideal quantizer
        assert!(
            enob > 11.0 && enob < 12.5,
            "12-bit ENOB should be ~12, got {}",
            enob
        );
    }

    #[test]
    fn test_theoretical_sqnr() {
        let q =
            UniformScalarQuantizer::new(16, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
        let sqnr = q.theoretical_sqnr_sinusoid();
        assert!(approx_eq(sqnr, 98.08, 0.01));
    }

    // -------------------------------------------------------------- dither

    #[test]
    fn test_dither_changes_output() {
        let mut q =
            UniformScalarQuantizer::new(4, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
        let signal: Vec<f64> = (0..256)
            .map(|i| 0.5 * (2.0 * PI * 7.0 * i as f64 / 256.0).sin())
            .collect();

        let no_dither = q.quantize_slice(&signal);
        q.set_dither(true);
        q.seed_dither(42);
        let with_dither = q.quantize_slice(&signal);

        // Dithered output should differ from non-dithered for at least some samples
        let diff_count = no_dither
            .iter()
            .zip(with_dither.iter())
            .filter(|(&a, &b)| (a - b).abs() > 1e-15)
            .count();
        assert!(
            diff_count > 0,
            "Dither should change at least some output samples"
        );
    }

    // -------------------------------------------------------------- error statistics

    #[test]
    fn test_error_statistics() {
        let q = UniformScalarQuantizer::new(8, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
        let n = 4096;
        let signal: Vec<f64> = (0..n)
            .map(|i| 0.9 * (2.0 * PI * 53.0 * i as f64 / n as f64).sin())
            .collect();
        let quantized = q.quantize_slice(&signal);
        let stats = q.error_statistics(&signal, &quantized, 20);

        // Mean error should be close to zero for a symmetric quantizer
        assert!(
            stats.mean.abs() < q.step_size(),
            "Mean error should be near zero, got {}",
            stats.mean
        );
        // Variance should be less than step^2/12 * some margin
        let max_var = q.step_size() * q.step_size() / 12.0 * 2.0;
        assert!(
            stats.variance < max_var,
            "Variance {} too large (max expected {})",
            stats.variance,
            max_var
        );
        // Histogram should account for all samples
        let total: usize = stats.histogram_counts.iter().sum();
        assert_eq!(total, n);
        assert_eq!(stats.histogram_edges.len(), 21);
    }

    // -------------------------------------------------------------- decision levels

    #[test]
    fn test_decision_levels_count() {
        let q = UniformScalarQuantizer::new(3, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
        let dl = q.decision_levels();
        // 8 levels -> 7 decision boundaries
        assert_eq!(dl.len(), 7);

        let rl = q.reconstruction_levels();
        assert_eq!(rl.len(), 8);
    }

    #[test]
    fn test_reconstruction_levels_mid_riser_count() {
        let q = UniformScalarQuantizer::new(3, 1.0, QuantizerMode::MidRiser, OverflowMode::Clip);
        let rl = q.reconstruction_levels();
        assert_eq!(rl.len(), 8);
        let dl = q.decision_levels();
        assert_eq!(dl.len(), 7);
    }

    // -------------------------------------------------------------- quantize_slice_mut

    #[test]
    fn test_quantize_slice_mut_no_dither() {
        let mut q =
            UniformScalarQuantizer::new(8, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
        let signal = vec![0.1, 0.5, -0.3, 0.99, -0.99];
        let result = q.quantize_slice_mut(&signal);
        assert_eq!(result.len(), signal.len());
        for (&orig, &quant) in signal.iter().zip(result.iter()) {
            assert!((orig - quant).abs() < q.step_size());
        }
    }

    // -------------------------------------------------------------- 1-bit quantizer

    #[test]
    fn test_1bit_quantizer() {
        let q = UniformScalarQuantizer::new(1, 1.0, QuantizerMode::MidTread, OverflowMode::Clip);
        assert_eq!(q.num_levels(), 2);
        let levels = q.reconstruction_levels();
        assert_eq!(levels.len(), 2);
        // Should produce two distinct levels
        assert!((levels[0] - levels[1]).abs() > 1e-15);
    }
}
