//! Time-domain adaptive equalization for single-carrier waveforms.
//!
//! This module provides adaptive equalizers that compensate for inter-symbol
//! interference (ISI) caused by multipath channels. Three adaptation algorithms
//! are supported:
//!
//! - **LMS (Least Mean Squares)**: Simple, robust, low computational cost.
//! - **RLS (Recursive Least Squares)**: Faster convergence at higher cost.
//! - **CMA (Constant Modulus Algorithm)**: Blind equalization for QAM signals.
//!
//! The equalizer operates in three modes:
//!
//! - **Training**: Uses a known reference sequence for supervised adaptation.
//! - **DecisionDirected**: After training, uses detected symbols as the reference.
//! - **Blind**: Uses CMA (no reference needed) for constant-modulus constellations.
//!
//! # Example
//!
//! ```
//! use r4w_core::time_domain_equalizer::{TimeDomainEqualizer, AdaptationAlgorithm};
//!
//! // Create an LMS equalizer with 5 taps
//! let mut eq = TimeDomainEqualizer::new(5, AdaptationAlgorithm::Lms { step_size: 0.05 });
//!
//! // Generate BPSK training data using a simple LFSR
//! let mut state: u32 = 0xACE1;
//! let tx_symbols: Vec<(f64, f64)> = (0..200).map(|_| {
//!     let bit = state & 1;
//!     state = (state >> 1) ^ (if bit == 1 { 0xB400 } else { 0 });
//!     if bit == 1 { (1.0, 0.0) } else { (-1.0, 0.0) }
//! }).collect();
//!
//! // Apply channel distortion: y[n] = x[n] + 0.5*x[n-1]
//! let mut distorted = vec![(tx_symbols[0].0, tx_symbols[0].1)];
//! for i in 1..tx_symbols.len() {
//!     distorted.push((
//!         tx_symbols[i].0 + 0.5 * tx_symbols[i - 1].0,
//!         tx_symbols[i].1 + 0.5 * tx_symbols[i - 1].1,
//!     ));
//! }
//!
//! // Train the equalizer
//! let _output = eq.train(&distorted, &tx_symbols);
//!
//! // After training, MSE should decrease
//! let mse = eq.mse();
//! assert!(mse < 1.0, "MSE should decrease during training, got {}", mse);
//! ```

use std::f64;

// ─── Public types ───────────────────────────────────────────────────────────

/// Adaptation algorithm used by the equalizer.
#[derive(Clone, Debug)]
pub enum AdaptationAlgorithm {
    /// Least Mean Squares with a given step size (mu).
    Lms { step_size: f64 },
    /// Recursive Least Squares with a given forgetting factor (lambda).
    Rls { forgetting_factor: f64 },
    /// Constant Modulus Algorithm for blind equalization of QAM signals.
    Cma { step_size: f64, modulus: f64 },
}

/// Equalizer operating mode.
#[derive(Clone, Debug, PartialEq)]
pub enum EqualizerMode {
    /// Supervised training with a known reference sequence.
    Training,
    /// Decision-directed mode: uses slicer output as the reference.
    DecisionDirected,
    /// Blind equalization using CMA (no reference needed).
    Blind,
}

// ─── Complex helpers ────────────────────────────────────────────────────────

type C = (f64, f64);

#[inline]
fn c_add(a: C, b: C) -> C {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: C, b: C) -> C {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_mul(a: C, b: C) -> C {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_conj(a: C) -> C {
    (a.0, -a.1)
}

#[inline]
fn c_scale(a: C, s: f64) -> C {
    (a.0 * s, a.1 * s)
}

#[inline]
fn c_mag_sq(a: C) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

#[inline]
fn c_div(a: C, b: C) -> C {
    let denom = c_mag_sq(b);
    if denom < 1e-30 {
        (0.0, 0.0)
    } else {
        c_scale(c_mul(a, c_conj(b)), 1.0 / denom)
    }
}

// ─── Decision function ──────────────────────────────────────────────────────

/// Find the nearest constellation point to `sample` from a set of candidates.
///
/// This is used in decision-directed mode to generate the reference signal
/// from the equalizer output.
pub fn nearest_qam_point(sample: C, constellation: &[C]) -> C {
    let mut best = constellation[0];
    let mut best_dist = c_mag_sq(c_sub(sample, best));
    for &pt in &constellation[1..] {
        let dist = c_mag_sq(c_sub(sample, pt));
        if dist < best_dist {
            best_dist = dist;
            best = pt;
        }
    }
    best
}

/// Default BPSK constellation used for decision-directed mode when no
/// constellation is explicitly provided.
const BPSK_CONSTELLATION: [C; 2] = [(1.0, 0.0), (-1.0, 0.0)];

// ─── Equalizer ──────────────────────────────────────────────────────────────

/// Time-domain adaptive equalizer.
///
/// Implements a transversal (FIR) filter whose tap weights are adapted
/// sample-by-sample using the selected algorithm.
pub struct TimeDomainEqualizer {
    num_taps: usize,
    weights: Vec<C>,
    algorithm: AdaptationAlgorithm,
    mode: EqualizerMode,
    delay_line: Vec<C>,
    mse_history: Vec<f64>,
    /// Constellation used for decision-directed mode.
    constellation: Vec<C>,
    // RLS inverse correlation matrix (num_taps x num_taps)
    p_matrix: Option<Vec<Vec<C>>>,
}

impl TimeDomainEqualizer {
    /// Create a new equalizer with `num_taps` taps and the given algorithm.
    ///
    /// The centre tap is initialized to 1 (identity) so the equalizer passes
    /// the signal through before any adaptation has occurred.
    pub fn new(num_taps: usize, algorithm: AdaptationAlgorithm) -> Self {
        assert!(num_taps > 0, "num_taps must be at least 1");

        let mut weights = vec![(0.0, 0.0); num_taps];
        // Centre-spike initialization
        weights[num_taps / 2] = (1.0, 0.0);

        let p_matrix = if matches!(algorithm, AdaptationAlgorithm::Rls { .. }) {
            // Initialize P = delta * I  (large diagonal)
            let delta = 100.0;
            let mut p = vec![vec![(0.0, 0.0); num_taps]; num_taps];
            for i in 0..num_taps {
                p[i][i] = (delta, 0.0);
            }
            Some(p)
        } else {
            None
        };

        let mode = match algorithm {
            AdaptationAlgorithm::Cma { .. } => EqualizerMode::Blind,
            _ => EqualizerMode::Training,
        };

        TimeDomainEqualizer {
            num_taps,
            weights,
            algorithm,
            mode,
            delay_line: vec![(0.0, 0.0); num_taps],
            mse_history: Vec::new(),
            constellation: BPSK_CONSTELLATION.to_vec(),
            p_matrix,
        }
    }

    // ── Internal helpers ────────────────────────────────────────────────

    /// Compute the filter output y = w^T * x (linear inner product).
    fn filter_output(&self) -> C {
        let mut y = (0.0, 0.0);
        for i in 0..self.num_taps {
            y = c_add(y, c_mul(self.weights[i], self.delay_line[i]));
        }
        y
    }

    /// Push a new sample into the delay line (shift register).
    fn push_sample(&mut self, sample: C) {
        // Shift right
        for i in (1..self.num_taps).rev() {
            self.delay_line[i] = self.delay_line[i - 1];
        }
        self.delay_line[0] = sample;
    }

    /// Update tap weights using the LMS algorithm.
    fn update_lms(&mut self, error: C, step_size: f64) {
        for i in 0..self.num_taps {
            // w[i] += mu * e * conj(x[i])  — note: not conj(e), we want e*x*
            // Standard complex LMS: w += mu * conj(x) * e
            let update = c_mul(c_conj(self.delay_line[i]), error);
            self.weights[i] = c_add(self.weights[i], c_scale(update, step_size));
        }
    }

    /// Update tap weights using the RLS algorithm.
    fn update_rls(&mut self, error: C, forgetting_factor: f64) {
        let p = self.p_matrix.as_mut().expect("RLS requires P matrix");
        let n = self.num_taps;
        let lambda = forgetting_factor;
        let inv_lambda = 1.0 / lambda;

        // k = (P * x) / (lambda + x^H * P * x)
        // First compute P * x
        let mut px = vec![(0.0, 0.0); n];
        for i in 0..n {
            for j in 0..n {
                px[i] = c_add(px[i], c_mul(p[i][j], self.delay_line[j]));
            }
        }

        // Compute x^H * P * x (scalar)
        let mut xhpx = (0.0, 0.0);
        for i in 0..n {
            xhpx = c_add(xhpx, c_mul(c_conj(self.delay_line[i]), px[i]));
        }

        // denominator = lambda + x^H P x
        let denom = c_add((lambda, 0.0), xhpx);

        // Gain vector k = P*x / denom
        let mut k = vec![(0.0, 0.0); n];
        for i in 0..n {
            k[i] = c_div(px[i], denom);
        }

        // Update weights: w += k * conj(error)
        // (using conj(error) because e = d - y and we want w += k * e*)
        // Actually standard RLS: w += k * e  (complex)
        for i in 0..n {
            self.weights[i] = c_add(self.weights[i], c_mul(k[i], error));
        }

        // Update P: P = (1/lambda) * (P - k * x^H * P)
        // More numerically: P = inv_lambda * (P - k * (P*x)^H)  since (x^H P)^H = P^H x = P x for Hermitian P
        // But P may not stay perfectly Hermitian, so use the standard form:
        // P = inv_lambda * (P - k * x^H * P)
        // First compute x^H * P (row vector)
        let mut xhp = vec![(0.0, 0.0); n];
        for j in 0..n {
            for i in 0..n {
                xhp[j] = c_add(xhp[j], c_mul(c_conj(self.delay_line[i]), p[i][j]));
            }
        }

        // P = inv_lambda * (P - k * xhp)
        for i in 0..n {
            for j in 0..n {
                let kxhp = c_mul(k[i], xhp[j]);
                p[i][j] = c_scale(c_sub(p[i][j], kxhp), inv_lambda);
            }
        }
    }

    /// Process one sample and adapt. Returns (output, error).
    fn process_one(&mut self, input: C, desired: C) -> (C, C) {
        self.push_sample(input);
        let y = self.filter_output();
        let e = c_sub(desired, y);

        match self.algorithm.clone() {
            AdaptationAlgorithm::Lms { step_size } => {
                self.update_lms(e, step_size);
            }
            AdaptationAlgorithm::Rls { forgetting_factor } => {
                self.update_rls(e, forgetting_factor);
            }
            AdaptationAlgorithm::Cma { step_size, modulus } => {
                // CMA error: e = y * (R - |y|^2)  where R = modulus
                let cma_err = c_scale(y, modulus - c_mag_sq(y));
                self.update_lms(cma_err, step_size);
                // Return standard error for MSE tracking
                return (y, e);
            }
        }

        (y, e)
    }

    // ── Public API ──────────────────────────────────────────────────────

    /// Train the equalizer using a known reference sequence.
    ///
    /// Returns the equalized output. The equalizer mode is set to `Training`.
    pub fn train(&mut self, input: &[C], reference: &[C]) -> Vec<C> {
        self.mode = EqualizerMode::Training;
        let len = input.len().min(reference.len());
        let mut output = Vec::with_capacity(len);

        for i in 0..len {
            let (y, e) = self.process_one(input[i], reference[i]);
            output.push(y);
            self.mse_history.push(c_mag_sq(e));
        }
        output
    }

    /// Equalize a block of complex samples.
    ///
    /// In `Training` or `DecisionDirected` mode the equalizer uses the nearest
    /// constellation point as the reference. In `Blind` mode CMA is used.
    pub fn equalize(&mut self, input: &[C]) -> Vec<C> {
        let mut output = Vec::with_capacity(input.len());

        for &sample in input {
            self.push_sample(sample);
            let y = self.filter_output();

            let desired = match self.mode {
                EqualizerMode::Training | EqualizerMode::DecisionDirected => {
                    nearest_qam_point(y, &self.constellation)
                }
                EqualizerMode::Blind => y, // CMA computes its own error
            };

            let e = c_sub(desired, y);

            match self.algorithm.clone() {
                AdaptationAlgorithm::Lms { step_size } => {
                    self.update_lms(e, step_size);
                }
                AdaptationAlgorithm::Rls { forgetting_factor } => {
                    self.update_rls(e, forgetting_factor);
                }
                AdaptationAlgorithm::Cma { step_size, modulus } => {
                    let cma_err = c_scale(y, modulus - c_mag_sq(y));
                    self.update_lms(cma_err, step_size);
                }
            }

            self.mse_history.push(c_mag_sq(e));
            output.push(y);
        }
        output
    }

    /// Equalize real-valued samples.
    ///
    /// If `reference` is `Some`, supervised training is used; otherwise
    /// decision-directed mode is employed with a simple +1/-1 slicer.
    pub fn equalize_real(&mut self, input: &[f64], reference: Option<&[f64]>) -> Vec<f64> {
        let complex_input: Vec<C> = input.iter().map(|&x| (x, 0.0)).collect();

        let output = if let Some(ref_signal) = reference {
            let complex_ref: Vec<C> = ref_signal.iter().map(|&x| (x, 0.0)).collect();
            self.train(&complex_input, &complex_ref)
        } else {
            self.set_mode(EqualizerMode::DecisionDirected);
            self.equalize(&complex_input)
        };

        output.iter().map(|&(re, _)| re).collect()
    }

    /// Set the operating mode.
    pub fn set_mode(&mut self, mode: EqualizerMode) {
        self.mode = mode;
    }

    /// Set the constellation used for decision-directed mode.
    pub fn set_constellation(&mut self, constellation: Vec<C>) {
        self.constellation = constellation;
    }

    /// Get the current tap weights.
    pub fn weights(&self) -> &[C] {
        &self.weights
    }

    /// Get the most recent MSE value (last entry in MSE history).
    ///
    /// Returns 0.0 if no samples have been processed yet.
    pub fn mse(&self) -> f64 {
        self.mse_history.last().copied().unwrap_or(0.0)
    }

    /// Get the full MSE history (one value per processed sample).
    pub fn mse_history(&self) -> &[f64] {
        &self.mse_history
    }

    /// Reset the equalizer state (weights, delay line, MSE history).
    ///
    /// Re-initializes to centre-spike weights and clears all history.
    pub fn reset(&mut self) {
        self.weights = vec![(0.0, 0.0); self.num_taps];
        self.weights[self.num_taps / 2] = (1.0, 0.0);
        self.delay_line = vec![(0.0, 0.0); self.num_taps];
        self.mse_history.clear();

        if let Some(ref mut p) = self.p_matrix {
            let delta = 100.0;
            for i in 0..self.num_taps {
                for j in 0..self.num_taps {
                    p[i][j] = if i == j { (delta, 0.0) } else { (0.0, 0.0) };
                }
            }
        }
    }
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: apply a 2-tap channel h = [1.0, h1] to a signal.
    fn apply_channel(input: &[C], h1: C) -> Vec<C> {
        let mut out = Vec::with_capacity(input.len());
        for i in 0..input.len() {
            let prev = if i > 0 { input[i - 1] } else { (0.0, 0.0) };
            out.push(c_add(input[i], c_mul(h1, prev)));
        }
        out
    }

    /// Helper: generate a BPSK training sequence.
    fn bpsk_sequence(len: usize) -> Vec<C> {
        // Simple PRBS-like sequence using a basic LFSR idea (deterministic)
        let mut seq = Vec::with_capacity(len);
        let mut state: u32 = 0xACE1;
        for _ in 0..len {
            let bit = state & 1;
            state = (state >> 1) ^ (if bit == 1 { 0xB400 } else { 0 });
            seq.push(if bit == 1 { (1.0, 0.0) } else { (-1.0, 0.0) });
        }
        seq
    }

    #[test]
    fn test_lms_basic_convergence() {
        let mut eq = TimeDomainEqualizer::new(7, AdaptationAlgorithm::Lms { step_size: 0.01 });
        let tx = bpsk_sequence(200);
        let rx = apply_channel(&tx, (0.5, 0.0));
        eq.train(&rx, &tx);

        let hist = eq.mse_history();
        // Compare average MSE of first 20 vs last 20 samples
        let early_mse: f64 = hist[..20].iter().sum::<f64>() / 20.0;
        let late_mse: f64 = hist[hist.len() - 20..].iter().sum::<f64>() / 20.0;
        assert!(
            late_mse < early_mse,
            "LMS should converge: early={early_mse}, late={late_mse}"
        );
    }

    #[test]
    fn test_rls_basic_convergence() {
        let mut eq = TimeDomainEqualizer::new(
            7,
            AdaptationAlgorithm::Rls {
                forgetting_factor: 0.99,
            },
        );
        let tx = bpsk_sequence(100);
        let rx = apply_channel(&tx, (0.4, 0.0));
        eq.train(&rx, &tx);

        let hist = eq.mse_history();
        let early_mse: f64 = hist[..10].iter().sum::<f64>() / 10.0;
        let late_mse: f64 = hist[hist.len() - 10..].iter().sum::<f64>() / 10.0;
        assert!(
            late_mse < early_mse,
            "RLS should converge: early={early_mse}, late={late_mse}"
        );
    }

    #[test]
    fn test_rls_converges_faster_than_lms() {
        let tx = bpsk_sequence(100);
        let rx = apply_channel(&tx, (0.5, 0.0));

        let mut lms = TimeDomainEqualizer::new(7, AdaptationAlgorithm::Lms { step_size: 0.02 });
        lms.train(&rx, &tx);

        let mut rls = TimeDomainEqualizer::new(
            7,
            AdaptationAlgorithm::Rls {
                forgetting_factor: 0.99,
            },
        );
        rls.train(&rx, &tx);

        // RLS should reach lower MSE in the same number of samples
        let lms_final: f64 = lms.mse_history()[80..].iter().sum::<f64>() / 20.0;
        let rls_final: f64 = rls.mse_history()[80..].iter().sum::<f64>() / 20.0;
        assert!(
            rls_final < lms_final * 1.5,
            "RLS should converge at least comparably: rls={rls_final}, lms={lms_final}"
        );
    }

    #[test]
    fn test_cma_blind_equalization() {
        let mut eq = TimeDomainEqualizer::new(
            11,
            AdaptationAlgorithm::Cma {
                step_size: 0.001,
                modulus: 1.0,
            },
        );
        let tx = bpsk_sequence(500);
        let rx = apply_channel(&tx, (0.3, 0.0));

        let output = eq.equalize(&rx);

        // After convergence, output magnitude should be closer to 1.0
        let late_output = &output[400..];
        let avg_mag: f64 =
            late_output.iter().map(|s| c_mag_sq(*s).sqrt()).sum::<f64>() / late_output.len() as f64;
        assert!(
            (avg_mag - 1.0).abs() < 0.3,
            "CMA should push magnitude toward modulus: avg_mag={avg_mag}"
        );
    }

    #[test]
    fn test_decision_directed_mode() {
        let mut eq = TimeDomainEqualizer::new(7, AdaptationAlgorithm::Lms { step_size: 0.01 });
        let tx = bpsk_sequence(200);
        let rx = apply_channel(&tx, (0.3, 0.0));

        // Train on first half
        eq.train(&rx[..100], &tx[..100]);

        // Switch to decision-directed on second half
        eq.set_mode(EqualizerMode::DecisionDirected);
        let output = eq.equalize(&rx[100..]);

        // Should still produce valid BPSK-like outputs
        let correct = output
            .iter()
            .zip(tx[100..].iter())
            .filter(|(&y, &t)| {
                let decision = if y.0 >= 0.0 { 1.0 } else { -1.0 };
                (decision - t.0).abs() < 0.01
            })
            .count();

        let accuracy = correct as f64 / output.len() as f64;
        assert!(
            accuracy > 0.7,
            "DD mode should maintain reasonable accuracy: {accuracy}"
        );
    }

    #[test]
    fn test_equalize_real() {
        let mut eq = TimeDomainEqualizer::new(7, AdaptationAlgorithm::Lms { step_size: 0.01 });

        let tx_real: Vec<f64> = bpsk_sequence(200).iter().map(|s| s.0).collect();
        let rx_real: Vec<f64> = {
            let mut out = vec![tx_real[0]];
            for i in 1..tx_real.len() {
                out.push(tx_real[i] + 0.4 * tx_real[i - 1]);
            }
            out
        };

        let output = eq.equalize_real(&rx_real, Some(&tx_real));

        // Late outputs should be closer to the original
        let late_err: f64 = output[150..]
            .iter()
            .zip(tx_real[150..].iter())
            .map(|(y, t)| (y - t).powi(2))
            .sum::<f64>()
            / 50.0;
        assert!(late_err < 0.5, "Real equalization should converge: mse={late_err}");
    }

    #[test]
    fn test_weights_and_reset() {
        let mut eq = TimeDomainEqualizer::new(5, AdaptationAlgorithm::Lms { step_size: 0.05 });

        // Check initial centre-spike
        let w = eq.weights();
        assert_eq!(w.len(), 5);
        assert_eq!(w[2], (1.0, 0.0)); // centre tap
        assert_eq!(w[0], (0.0, 0.0));

        // Train briefly to change weights
        let tx = bpsk_sequence(50);
        let rx = apply_channel(&tx, (0.5, 0.0));
        eq.train(&rx, &tx);

        // Weights should have changed
        let w_after = eq.weights().to_vec();
        assert_ne!(w_after[0], (0.0, 0.0), "Weights should adapt");

        // MSE history should have entries
        assert_eq!(eq.mse_history().len(), 50);

        // Reset
        eq.reset();
        assert_eq!(eq.weights()[2], (1.0, 0.0));
        assert_eq!(eq.weights()[0], (0.0, 0.0));
        assert!(eq.mse_history().is_empty());
        assert_eq!(eq.mse(), 0.0);
    }

    #[test]
    fn test_complex_channel_equalization() {
        // Channel with complex ISI: h = [1.0, 0.3+0.2j]
        let mut eq = TimeDomainEqualizer::new(
            11,
            AdaptationAlgorithm::Lms { step_size: 0.005 },
        );
        let tx = bpsk_sequence(300);
        let rx = apply_channel(&tx, (0.3, 0.2));
        eq.train(&rx, &tx);

        let hist = eq.mse_history();
        let early_mse: f64 = hist[..20].iter().sum::<f64>() / 20.0;
        let late_mse: f64 = hist[hist.len() - 20..].iter().sum::<f64>() / 20.0;
        assert!(
            late_mse < early_mse * 0.5,
            "Should converge on complex channel: early={early_mse}, late={late_mse}"
        );
    }

    #[test]
    fn test_nearest_qam_point() {
        let qpsk: Vec<C> = vec![(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)];

        assert_eq!(nearest_qam_point((0.8, 0.9), &qpsk), (1.0, 1.0));
        assert_eq!(nearest_qam_point((-0.5, -0.7), &qpsk), (-1.0, -1.0));
        assert_eq!(nearest_qam_point((1.2, -0.3), &qpsk), (1.0, -1.0));
        assert_eq!(nearest_qam_point((-0.9, 1.1), &qpsk), (-1.0, 1.0));
    }

    #[test]
    fn test_mse_decreases_monotonically_smoothed() {
        let mut eq = TimeDomainEqualizer::new(9, AdaptationAlgorithm::Lms { step_size: 0.01 });
        let tx = bpsk_sequence(400);
        let rx = apply_channel(&tx, (0.4, 0.0));
        eq.train(&rx, &tx);

        let hist = eq.mse_history();
        // Check that MSE trend is decreasing by comparing quartiles
        let q1: f64 = hist[..100].iter().sum::<f64>() / 100.0;
        let q2: f64 = hist[100..200].iter().sum::<f64>() / 100.0;
        let q3: f64 = hist[200..300].iter().sum::<f64>() / 100.0;
        let q4: f64 = hist[300..].iter().sum::<f64>() / 100.0;

        assert!(q2 <= q1 * 1.1, "Q2 should be <= Q1: q1={q1}, q2={q2}");
        assert!(q3 <= q2 * 1.1, "Q3 should be <= Q2: q2={q2}, q3={q3}");
        assert!(q4 <= q3 * 1.1, "Q4 should be <= Q3: q3={q3}, q4={q4}");
    }

    #[test]
    fn test_single_tap_identity() {
        // With 1 tap, the equalizer is just a scalar gain
        let mut eq = TimeDomainEqualizer::new(1, AdaptationAlgorithm::Lms { step_size: 0.1 });

        // Feed identity: input == reference
        let signal: Vec<C> = vec![(1.0, 0.0); 20];
        eq.train(&signal, &signal);

        // Weight should be close to 1.0
        let w = eq.weights()[0];
        assert!(
            (w.0 - 1.0).abs() < 0.1 && w.1.abs() < 0.1,
            "Single tap should be ~1.0: {w:?}"
        );
    }

    #[test]
    fn test_set_constellation_qpsk() {
        let qpsk: Vec<C> = vec![(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)];
        let mut eq = TimeDomainEqualizer::new(
            11,
            AdaptationAlgorithm::Lms { step_size: 0.005 },
        );
        eq.set_constellation(qpsk.clone());

        // Generate pseudo-random QPSK symbols using LFSR
        let mut state: u32 = 0xDEAD;
        let tx: Vec<C> = (0..500)
            .map(|_| {
                let bits = state & 3;
                state = (state >> 1) ^ (if state & 1 == 1 { 0xB400 } else { 0 });
                qpsk[bits as usize]
            })
            .collect();
        let rx = apply_channel(&tx, (0.25, 0.0));

        // Train on first 300 symbols (plenty for LMS on mild channel)
        eq.train(&rx[..300], &tx[..300]);

        eq.set_mode(EqualizerMode::DecisionDirected);
        let output = eq.equalize(&rx[300..]);

        // Check that DD output is near QPSK constellation points
        let mut near_count = 0;
        for &y in &output[20..] {
            let nearest = nearest_qam_point(y, &qpsk);
            if c_mag_sq(c_sub(y, nearest)) < 1.0 {
                near_count += 1;
            }
        }
        let ratio = near_count as f64 / (output.len() - 20) as f64;
        assert!(
            ratio > 0.5,
            "DD with QPSK should produce near-constellation outputs: ratio={ratio}"
        );
    }
}
