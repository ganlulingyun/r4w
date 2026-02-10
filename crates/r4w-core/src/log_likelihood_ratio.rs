//! Soft-decision Log-Likelihood Ratio (LLR) computation for iterative decoding.
//!
//! This module computes LLRs from received symbols for use with turbo codes, LDPC,
//! and other soft-decision decoders. The sign of each LLR gives the hard decision
//! (positive → bit 0, negative → bit 1), while the magnitude indicates confidence.
//!
//! Supported modulations: BPSK, QPSK, 16-QAM, and 64-QAM. Both exact MAP and
//! max-log-MAP approximation methods are provided.
//!
//! # Example
//!
//! ```
//! use r4w_core::log_likelihood_ratio::{LlrCalculator, Modulation};
//!
//! // Create a calculator with noise variance σ² = 0.5
//! let calc = LlrCalculator::new(0.5);
//!
//! // BPSK: received symbol y = +0.8 → positive LLR → hard decision is bit 0
//! let llr = calc.bpsk_llr(0.8);
//! assert!(llr > 0.0, "positive received symbol → positive LLR → bit 0");
//!
//! // Batch computation for QPSK
//! let symbols = vec![(0.7, 0.9), (-0.6, 0.3)];
//! let llrs = calc.compute_batch(&symbols, Modulation::Qpsk);
//! assert_eq!(llrs.len(), 4); // 2 bits per symbol × 2 symbols
//! ```

use std::f64;

/// Modulation scheme selector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Modulation {
    /// BPSK: 1 bit per symbol, mapping {0 → +1, 1 → −1}.
    Bpsk,
    /// QPSK: 2 bits per symbol (I and Q each carry one bit).
    Qpsk,
    /// 16-QAM: 4 bits per symbol on a 4×4 grid.
    Qam16,
    /// 64-QAM: 6 bits per symbol on an 8×8 grid.
    Qam64,
}

/// LLR computation method.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Method {
    /// Exact MAP: sum over constellation points using log-sum-exp.
    ExactMap,
    /// Max-log-MAP approximation: replace log-sum-exp with max.
    MaxLogMap,
}

/// Soft-decision LLR calculator for various modulation schemes.
///
/// The calculator is parameterized by noise variance σ² and an optional
/// clipping threshold for numerical stability.
#[derive(Debug, Clone)]
pub struct LlrCalculator {
    /// Noise variance σ².
    noise_variance: f64,
    /// LLR clipping magnitude (values are clamped to ±clip_value).
    clip_value: f64,
    /// Computation method (exact MAP or max-log-MAP).
    method: Method,
}

impl LlrCalculator {
    /// Create a new LLR calculator with the given noise variance.
    ///
    /// Defaults to max-log-MAP method with a clip value of 100.0.
    ///
    /// # Panics
    ///
    /// Panics if `noise_variance` is not positive.
    pub fn new(noise_variance: f64) -> Self {
        assert!(noise_variance > 0.0, "noise_variance must be positive");
        Self {
            noise_variance,
            clip_value: 100.0,
            method: Method::MaxLogMap,
        }
    }

    /// Set the computation method.
    pub fn with_method(mut self, method: Method) -> Self {
        self.method = method;
        self
    }

    /// Set the LLR clipping value for numerical stability.
    ///
    /// LLR outputs will be clamped to the range `[-clip_value, +clip_value]`.
    ///
    /// # Panics
    ///
    /// Panics if `clip_value` is not positive.
    pub fn with_clip(mut self, clip_value: f64) -> Self {
        assert!(clip_value > 0.0, "clip_value must be positive");
        self.clip_value = clip_value;
        self
    }

    /// Update the noise variance (e.g., from a new SNR estimate).
    ///
    /// # Panics
    ///
    /// Panics if `noise_variance` is not positive.
    pub fn set_noise_variance(&mut self, noise_variance: f64) {
        assert!(noise_variance > 0.0, "noise_variance must be positive");
        self.noise_variance = noise_variance;
    }

    /// Return the current noise variance.
    pub fn noise_variance(&self) -> f64 {
        self.noise_variance
    }

    /// Clamp an LLR to the configured clip range.
    fn clip(&self, llr: f64) -> f64 {
        llr.clamp(-self.clip_value, self.clip_value)
    }

    // ── BPSK ──────────────────────────────────────────────────────────

    /// Compute BPSK LLR for a single real-valued received sample.
    ///
    /// With mapping `{bit 0 → +1, bit 1 → −1}`:
    ///
    /// ```text
    /// L(b) = 2·y / σ²
    /// ```
    pub fn bpsk_llr(&self, y: f64) -> f64 {
        self.clip(2.0 * y / self.noise_variance)
    }

    /// Compute BPSK LLRs from complex symbols (uses only the real part).
    pub fn bpsk_llr_complex(&self, symbol: (f64, f64)) -> f64 {
        self.bpsk_llr(symbol.0)
    }

    // ── QPSK ──────────────────────────────────────────────────────────

    /// Compute QPSK LLRs for a single symbol, returning `[L(b0), L(b1)]`.
    ///
    /// With Gray-coded mapping at ±1/√2:
    /// - bit 0 from I component: `L(b0) = 2·√2·yI / σ²`
    /// - bit 1 from Q component: `L(b1) = 2·√2·yQ / σ²`
    pub fn qpsk_llr(&self, symbol: (f64, f64)) -> [f64; 2] {
        let scale = 2.0 * std::f64::consts::SQRT_2 / self.noise_variance;
        [
            self.clip(scale * symbol.0),
            self.clip(scale * symbol.1),
        ]
    }

    // ── 16-QAM ────────────────────────────────────────────────────────

    /// Standard 16-QAM constellation points (Gray-coded, unit average energy).
    ///
    /// Returns the 16 constellation points with their Gray-coded bit labels.
    fn qam16_constellation() -> &'static [(f64, f64, [u8; 4])] {
        // Gray-coded 16-QAM constellation.
        // Bit labeling: [b0, b1, b2, b3] where b0,b1 select I, b2,b3 select Q.
        // I mapping (Gray): 00→+3, 01→+1, 11→-1, 10→-3
        // Q mapping: same.
        const K: f64 = 0.316_227_766_016_837_94; // 1/√10
        static POINTS: [(f64, f64, [u8; 4]); 16] = [
            ( 3.0 * K,  3.0 * K, [0, 0, 0, 0]),
            ( 3.0 * K,  1.0 * K, [0, 0, 0, 1]),
            ( 3.0 * K, -1.0 * K, [0, 0, 1, 1]),
            ( 3.0 * K, -3.0 * K, [0, 0, 1, 0]),
            ( 1.0 * K,  3.0 * K, [0, 1, 0, 0]),
            ( 1.0 * K,  1.0 * K, [0, 1, 0, 1]),
            ( 1.0 * K, -1.0 * K, [0, 1, 1, 1]),
            ( 1.0 * K, -3.0 * K, [0, 1, 1, 0]),
            (-1.0 * K,  3.0 * K, [1, 1, 0, 0]),
            (-1.0 * K,  1.0 * K, [1, 1, 0, 1]),
            (-1.0 * K, -1.0 * K, [1, 1, 1, 1]),
            (-1.0 * K, -3.0 * K, [1, 1, 1, 0]),
            (-3.0 * K,  3.0 * K, [1, 0, 0, 0]),
            (-3.0 * K,  1.0 * K, [1, 0, 0, 1]),
            (-3.0 * K, -1.0 * K, [1, 0, 1, 1]),
            (-3.0 * K, -3.0 * K, [1, 0, 1, 0]),
        ];
        &POINTS
    }

    /// Compute 16-QAM LLRs for a single symbol, returning 4 LLR values.
    pub fn qam16_llr(&self, symbol: (f64, f64)) -> [f64; 4] {
        let constellation = Self::qam16_constellation();
        let mut llrs = [0.0f64; 4];

        for bit_index in 0..4 {
            llrs[bit_index] = self.generic_llr(symbol, constellation, bit_index);
        }
        llrs
    }

    // ── 64-QAM ────────────────────────────────────────────────────────

    /// Standard 64-QAM constellation points (Gray-coded, unit average energy).
    fn qam64_constellation() -> Vec<(f64, f64, [u8; 6])> {
        // 64-QAM: levels {-7, -5, -3, -1, +1, +3, +5, +7} scaled by 1/√42
        let k: f64 = 1.0 / 42.0_f64.sqrt();

        // Gray code mapping for 3 bits: 000→+7, 001→+5, 011→+3, 010→+1,
        //                                110→-1, 111→-3, 101→-5, 100→-7
        let gray_map: [(u8, u8, u8, f64); 8] = [
            (0, 0, 0,  7.0),
            (0, 0, 1,  5.0),
            (0, 1, 1,  3.0),
            (0, 1, 0,  1.0),
            (1, 1, 0, -1.0),
            (1, 1, 1, -3.0),
            (1, 0, 1, -5.0),
            (1, 0, 0, -7.0),
        ];

        let mut points = Vec::with_capacity(64);
        for &(bi0, bi1, bi2, i_level) in &gray_map {
            for &(bq0, bq1, bq2, q_level) in &gray_map {
                points.push((
                    i_level * k,
                    q_level * k,
                    [bi0, bi1, bi2, bq0, bq1, bq2],
                ));
            }
        }
        points
    }

    /// Compute 64-QAM LLRs for a single symbol, returning 6 LLR values.
    pub fn qam64_llr(&self, symbol: (f64, f64)) -> [f64; 6] {
        let constellation = Self::qam64_constellation();
        let mut llrs = [0.0f64; 6];

        for bit_index in 0..6 {
            llrs[bit_index] = self.generic_llr_vec(symbol, &constellation, bit_index);
        }
        llrs
    }

    // ── Generic helpers ───────────────────────────────────────────────

    /// Generic LLR computation for one bit position over a 4-bit constellation.
    fn generic_llr(
        &self,
        symbol: (f64, f64),
        constellation: &[(f64, f64, [u8; 4])],
        bit_index: usize,
    ) -> f64 {
        match self.method {
            Method::MaxLogMap => {
                let mut max_metric_0 = f64::NEG_INFINITY;
                let mut max_metric_1 = f64::NEG_INFINITY;

                for &(ci, cq, ref bits) in constellation {
                    let di = symbol.0 - ci;
                    let dq = symbol.1 - cq;
                    let metric = -(di * di + dq * dq) / self.noise_variance;

                    if bits[bit_index] == 0 {
                        if metric > max_metric_0 {
                            max_metric_0 = metric;
                        }
                    } else if metric > max_metric_1 {
                        max_metric_1 = metric;
                    }
                }
                self.clip(max_metric_0 - max_metric_1)
            }
            Method::ExactMap => {
                let mut metrics_0 = Vec::new();
                let mut metrics_1 = Vec::new();

                for &(ci, cq, ref bits) in constellation {
                    let di = symbol.0 - ci;
                    let dq = symbol.1 - cq;
                    let metric = -(di * di + dq * dq) / self.noise_variance;

                    if bits[bit_index] == 0 {
                        metrics_0.push(metric);
                    } else {
                        metrics_1.push(metric);
                    }
                }
                self.clip(log_sum_exp(&metrics_0) - log_sum_exp(&metrics_1))
            }
        }
    }

    /// Generic LLR computation for one bit position over a 6-bit constellation.
    fn generic_llr_vec(
        &self,
        symbol: (f64, f64),
        constellation: &[(f64, f64, [u8; 6])],
        bit_index: usize,
    ) -> f64 {
        match self.method {
            Method::MaxLogMap => {
                let mut max_metric_0 = f64::NEG_INFINITY;
                let mut max_metric_1 = f64::NEG_INFINITY;

                for &(ci, cq, ref bits) in constellation {
                    let di = symbol.0 - ci;
                    let dq = symbol.1 - cq;
                    let metric = -(di * di + dq * dq) / self.noise_variance;

                    if bits[bit_index] == 0 {
                        if metric > max_metric_0 {
                            max_metric_0 = metric;
                        }
                    } else if metric > max_metric_1 {
                        max_metric_1 = metric;
                    }
                }
                self.clip(max_metric_0 - max_metric_1)
            }
            Method::ExactMap => {
                let mut metrics_0 = Vec::new();
                let mut metrics_1 = Vec::new();

                for &(ci, cq, ref bits) in constellation {
                    let di = symbol.0 - ci;
                    let dq = symbol.1 - cq;
                    let metric = -(di * di + dq * dq) / self.noise_variance;

                    if bits[bit_index] == 0 {
                        metrics_0.push(metric);
                    } else {
                        metrics_1.push(metric);
                    }
                }
                self.clip(log_sum_exp(&metrics_0) - log_sum_exp(&metrics_1))
            }
        }
    }

    // ── Batch computation ─────────────────────────────────────────────

    /// Compute LLRs for a batch of symbols, returning a flat vector of LLR values.
    ///
    /// The number of LLRs per symbol depends on the modulation:
    /// - BPSK: 1 LLR per symbol
    /// - QPSK: 2 LLRs per symbol
    /// - 16-QAM: 4 LLRs per symbol
    /// - 64-QAM: 6 LLRs per symbol
    pub fn compute_batch(&self, symbols: &[(f64, f64)], modulation: Modulation) -> Vec<f64> {
        let bits_per_symbol = match modulation {
            Modulation::Bpsk => 1,
            Modulation::Qpsk => 2,
            Modulation::Qam16 => 4,
            Modulation::Qam64 => 6,
        };
        let mut llrs = Vec::with_capacity(symbols.len() * bits_per_symbol);

        for &sym in symbols {
            match modulation {
                Modulation::Bpsk => {
                    llrs.push(self.bpsk_llr_complex(sym));
                }
                Modulation::Qpsk => {
                    let l = self.qpsk_llr(sym);
                    llrs.extend_from_slice(&l);
                }
                Modulation::Qam16 => {
                    let l = self.qam16_llr(sym);
                    llrs.extend_from_slice(&l);
                }
                Modulation::Qam64 => {
                    let l = self.qam64_llr(sym);
                    llrs.extend_from_slice(&l);
                }
            }
        }
        llrs
    }

    /// Convert LLRs to hard decisions: positive LLR → 0, negative → 1.
    pub fn hard_decide(llrs: &[f64]) -> Vec<u8> {
        llrs.iter().map(|&l| if l >= 0.0 { 0 } else { 1 }).collect()
    }
}

/// Numerically stable log-sum-exp: `log(sum(exp(x_i)))`.
fn log_sum_exp(values: &[f64]) -> f64 {
    if values.is_empty() {
        return f64::NEG_INFINITY;
    }
    let max_val = values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    if max_val == f64::NEG_INFINITY {
        return f64::NEG_INFINITY;
    }
    let sum: f64 = values.iter().map(|&v| (v - max_val).exp()).sum();
    max_val + sum.ln()
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-10;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn test_bpsk_llr_positive_symbol() {
        let calc = LlrCalculator::new(1.0);
        let llr = calc.bpsk_llr(1.0);
        // L(b) = 2*1.0/1.0 = 2.0
        assert!(approx_eq(llr, 2.0, EPSILON), "expected 2.0, got {llr}");
    }

    #[test]
    fn test_bpsk_llr_negative_symbol() {
        let calc = LlrCalculator::new(1.0);
        let llr = calc.bpsk_llr(-1.0);
        // L(b) = 2*(-1.0)/1.0 = -2.0
        assert!(approx_eq(llr, -2.0, EPSILON), "expected -2.0, got {llr}");
    }

    #[test]
    fn test_bpsk_llr_noise_variance_scaling() {
        let calc = LlrCalculator::new(0.5);
        let llr = calc.bpsk_llr(1.0);
        // L(b) = 2*1.0/0.5 = 4.0
        assert!(approx_eq(llr, 4.0, EPSILON), "expected 4.0, got {llr}");
    }

    #[test]
    fn test_bpsk_hard_decision_from_sign() {
        let calc = LlrCalculator::new(1.0);
        // Positive received symbol → positive LLR → bit 0
        assert!(calc.bpsk_llr(0.5) > 0.0);
        // Negative received symbol → negative LLR → bit 1
        assert!(calc.bpsk_llr(-0.5) < 0.0);
    }

    #[test]
    fn test_qpsk_llr_both_positive() {
        let calc = LlrCalculator::new(1.0);
        let s2 = std::f64::consts::FRAC_1_SQRT_2; // 1/√2
        let llrs = calc.qpsk_llr((s2, s2));
        // L(b0) = 2*√2 * (1/√2) / 1.0 = 2.0
        // L(b1) = 2*√2 * (1/√2) / 1.0 = 2.0
        assert!(approx_eq(llrs[0], 2.0, EPSILON), "expected 2.0, got {}", llrs[0]);
        assert!(approx_eq(llrs[1], 2.0, EPSILON), "expected 2.0, got {}", llrs[1]);
    }

    #[test]
    fn test_qpsk_llr_mixed_signs() {
        let calc = LlrCalculator::new(1.0);
        let s2 = std::f64::consts::FRAC_1_SQRT_2;
        let llrs = calc.qpsk_llr((s2, -s2));
        // I positive → bit 0 is 0, Q negative → bit 1 is 1
        assert!(llrs[0] > 0.0, "I > 0 should give positive LLR");
        assert!(llrs[1] < 0.0, "Q < 0 should give negative LLR");
    }

    #[test]
    fn test_clipping() {
        let calc = LlrCalculator::new(0.001).with_clip(10.0);
        let llr = calc.bpsk_llr(100.0);
        // Without clipping: 2*100/0.001 = 200000, but clipped to 10
        assert!(approx_eq(llr, 10.0, EPSILON), "expected 10.0 (clipped), got {llr}");
    }

    #[test]
    fn test_clipping_negative() {
        let calc = LlrCalculator::new(0.001).with_clip(5.0);
        let llr = calc.bpsk_llr(-100.0);
        assert!(approx_eq(llr, -5.0, EPSILON), "expected -5.0 (clipped), got {llr}");
    }

    #[test]
    fn test_qam16_correct_number_of_llrs() {
        let calc = LlrCalculator::new(1.0);
        let llrs = calc.qam16_llr((0.3, -0.5));
        assert_eq!(llrs.len(), 4);
    }

    #[test]
    fn test_qam16_noiseless_corner() {
        // Transmit the (+3,+3)/√10 corner point (all bits 0) with no noise.
        let calc = LlrCalculator::new(0.1); // low noise
        let k = 1.0 / 10.0_f64.sqrt();
        let llrs = calc.qam16_llr((3.0 * k, 3.0 * k));
        // All bits are 0, so all LLRs should be positive (and large).
        for (i, &l) in llrs.iter().enumerate() {
            assert!(l > 0.0, "bit {i}: expected positive LLR for all-zero label, got {l}");
        }
    }

    #[test]
    fn test_qam64_correct_number_of_llrs() {
        let calc = LlrCalculator::new(1.0);
        let llrs = calc.qam64_llr((0.1, -0.2));
        assert_eq!(llrs.len(), 6);
    }

    #[test]
    fn test_batch_bpsk() {
        let calc = LlrCalculator::new(1.0);
        let symbols = vec![(1.0, 0.0), (-1.0, 0.0), (0.5, 0.0)];
        let llrs = calc.compute_batch(&symbols, Modulation::Bpsk);
        assert_eq!(llrs.len(), 3);
        assert!(llrs[0] > 0.0);
        assert!(llrs[1] < 0.0);
        assert!(llrs[2] > 0.0);
    }

    #[test]
    fn test_batch_qpsk_length() {
        let calc = LlrCalculator::new(1.0);
        let symbols = vec![(0.5, 0.5), (-0.5, -0.5), (0.3, -0.7)];
        let llrs = calc.compute_batch(&symbols, Modulation::Qpsk);
        assert_eq!(llrs.len(), 6); // 2 bits/sym * 3 symbols
    }

    #[test]
    fn test_batch_qam16_length() {
        let calc = LlrCalculator::new(1.0);
        let symbols = vec![(0.3, 0.3), (-0.9, 0.1)];
        let llrs = calc.compute_batch(&symbols, Modulation::Qam16);
        assert_eq!(llrs.len(), 8); // 4 bits/sym * 2 symbols
    }

    #[test]
    fn test_hard_decide() {
        let decisions = LlrCalculator::hard_decide(&[2.5, -1.3, 0.0, -0.01, 7.0]);
        assert_eq!(decisions, vec![0, 1, 0, 1, 0]);
    }

    #[test]
    fn test_exact_map_vs_max_log_map_bpsk() {
        // For BPSK, the closed-form formula is used regardless of method setting,
        // so results should be identical.
        let calc_max = LlrCalculator::new(1.0).with_method(Method::MaxLogMap);
        let calc_exact = LlrCalculator::new(1.0).with_method(Method::ExactMap);
        let llr_max = calc_max.bpsk_llr(0.7);
        let llr_exact = calc_exact.bpsk_llr(0.7);
        assert!(approx_eq(llr_max, llr_exact, EPSILON));
    }

    #[test]
    fn test_exact_map_qam16_sign_agreement() {
        // Exact MAP and max-log-MAP should agree on hard decisions.
        let calc_max = LlrCalculator::new(1.0).with_method(Method::MaxLogMap);
        let calc_exact = LlrCalculator::new(1.0).with_method(Method::ExactMap);

        let sym = (0.5, -0.3);
        let llr_max = calc_max.qam16_llr(sym);
        let llr_exact = calc_exact.qam16_llr(sym);

        for i in 0..4 {
            assert_eq!(
                llr_max[i] >= 0.0,
                llr_exact[i] >= 0.0,
                "bit {i}: sign mismatch between methods"
            );
        }
    }

    #[test]
    fn test_log_sum_exp_basic() {
        // log(exp(0) + exp(0)) = log(2) ≈ 0.693
        let result = log_sum_exp(&[0.0, 0.0]);
        assert!(approx_eq(result, 2.0_f64.ln(), 1e-12));
    }

    #[test]
    fn test_log_sum_exp_empty() {
        let result = log_sum_exp(&[]);
        assert!(result == f64::NEG_INFINITY);
    }

    #[test]
    #[should_panic(expected = "noise_variance must be positive")]
    fn test_zero_noise_variance_panics() {
        LlrCalculator::new(0.0);
    }

    #[test]
    #[should_panic(expected = "noise_variance must be positive")]
    fn test_negative_noise_variance_panics() {
        LlrCalculator::new(-1.0);
    }

    #[test]
    fn test_set_noise_variance() {
        let mut calc = LlrCalculator::new(1.0);
        assert!(approx_eq(calc.noise_variance(), 1.0, EPSILON));
        calc.set_noise_variance(2.0);
        assert!(approx_eq(calc.noise_variance(), 2.0, EPSILON));

        // LLR should reflect new variance.
        let llr = calc.bpsk_llr(1.0);
        assert!(approx_eq(llr, 1.0, EPSILON)); // 2*1/2 = 1
    }

    #[test]
    fn test_confidence_scales_with_snr() {
        // Higher SNR (lower noise variance) → larger LLR magnitude → more confidence.
        let high_snr = LlrCalculator::new(0.1);
        let low_snr = LlrCalculator::new(10.0);
        let llr_high = high_snr.bpsk_llr(1.0).abs();
        let llr_low = low_snr.bpsk_llr(1.0).abs();
        assert!(llr_high > llr_low, "higher SNR should give larger |LLR|");
    }

    #[test]
    fn test_qam64_noiseless_corner() {
        // Transmit corner point with all bits 0: (+7,+7)/√42
        let calc = LlrCalculator::new(0.1);
        let k = 1.0 / 42.0_f64.sqrt();
        let llrs = calc.qam64_llr((7.0 * k, 7.0 * k));
        // All bits 0, so all LLRs should be positive.
        for (i, &l) in llrs.iter().enumerate() {
            assert!(l > 0.0, "bit {i}: expected positive LLR for all-zero label, got {l}");
        }
    }
}
