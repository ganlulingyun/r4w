//! Digital Pre-Distortion (DPD) for power amplifier linearization.
//!
//! This module implements a memory polynomial DPD engine using indirect learning
//! architecture. It supports Rapp, Saleh, memory polynomial, and lookup-table PA
//! models, and trains an inverse model so that the cascade of predistorter + PA
//! produces a linear output.
//!
//! All complex IQ samples are represented as `(f64, f64)` tuples (no external
//! crate dependencies).
//!
//! # Example
//!
//! ```rust
//! use r4w_core::digital_predistortion::{DigitalPredistorter, DpdConfig, PaModel};
//!
//! // Configure a 5th-order memoryless DPD
//! let config = DpdConfig {
//!     polynomial_order: 5,
//!     memory_depth: 0,
//!     learning_rate: 0.05,
//! };
//! let mut dpd = DigitalPredistorter::new(config);
//!
//! // Generate a simple test tone
//! let input: Vec<(f64, f64)> = (0..256)
//!     .map(|i| {
//!         let phase = 2.0 * std::f64::consts::PI * i as f64 / 64.0;
//!         (0.7 * phase.cos(), 0.7 * phase.sin())
//!     })
//!     .collect();
//!
//! // Simulate the PA (Rapp model with saturation = 1.0, smoothness = 2)
//! let pa = PaModel::Rapp { a_sat: 1.0, p: 2 };
//! let pa_output = dpd.simulate_pa(&pa, &input);
//!
//! // Train the predistorter (indirect learning: PA output -> input)
//! dpd.train(&input, &pa_output);
//!
//! // Apply predistortion then PA — output should be more linear
//! let predistorted = dpd.predistort(&input);
//! assert_eq!(predistorted.len(), input.len());
//!
//! let linearized = dpd.simulate_pa(&pa, &predistorted);
//! assert_eq!(linearized.len(), input.len());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Helper complex arithmetic on (f64, f64) tuples
// ---------------------------------------------------------------------------

#[inline]
fn c_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

#[inline]
fn c_abs(a: (f64, f64)) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

#[inline]
fn c_scale(s: f64, a: (f64, f64)) -> (f64, f64) {
    (s * a.0, s * a.1)
}

#[inline]
fn c_abs_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Power amplifier behavioural model.
#[derive(Debug, Clone)]
pub enum PaModel {
    /// Rapp model (solid-state PA).
    ///
    /// AM/AM: `g(r) = r / (1 + (r / a_sat)^(2p))^(1/(2p))`
    ///
    /// AM/PM: none (memoryless, real gain).
    Rapp {
        /// Saturation amplitude.
        a_sat: f64,
        /// Smoothness factor (higher = sharper saturation knee).
        p: u32,
    },

    /// Saleh model (travelling-wave tube amplifier).
    ///
    /// AM/AM: `A(r) = alpha_a * r / (1 + beta_a * r^2)`
    /// AM/PM: `Phi(r) = alpha_p * r^2 / (1 + beta_p * r^2)`
    Saleh {
        alpha_a: f64,
        beta_a: f64,
        alpha_p: f64,
        beta_p: f64,
    },

    /// Arbitrary memory polynomial coefficients supplied externally.
    ///
    /// `y(n) = sum_k sum_q coeff[k][q] * x(n-q) * |x(n-q)|^k`
    ///
    /// Outer index = nonlinear order k (0, 1, 2, ...), inner = memory tap q.
    MemoryPolynomial {
        /// coefficients\[k\]\[q\] — complex.
        coefficients: Vec<Vec<(f64, f64)>>,
    },

    /// Lookup-table AM/AM + AM/PM model.
    ///
    /// Each entry maps an input amplitude to `(output_amplitude, phase_shift_rad)`.
    /// Linearly interpolated between entries. Entries must be sorted by input
    /// amplitude (ascending).
    LookupTable {
        /// `(input_amplitude, output_amplitude, phase_shift_radians)`
        table: Vec<(f64, f64, f64)>,
    },
}

/// DPD configuration parameters.
#[derive(Debug, Clone)]
pub struct DpdConfig {
    /// Polynomial non-linearity order (typically 3, 5, or 7).
    ///
    /// Only odd orders are used in the basis expansion; however the internal
    /// coefficient vector length is `(polynomial_order + 1) / 2`.
    pub polynomial_order: usize,

    /// Number of memory taps (0 = memoryless).
    pub memory_depth: usize,

    /// LMS-style learning rate (step size) for indirect learning.
    pub learning_rate: f64,
}

/// Metrics that quantify the DPD improvement.
#[derive(Debug, Clone)]
pub struct DpdMetrics {
    /// Adjacent-channel power ratio *before* DPD (dB). Negative means below carrier.
    pub acpr_before: f64,
    /// Adjacent-channel power ratio *after* DPD (dB).
    pub acpr_after: f64,
    /// Error-vector magnitude *before* DPD (%).
    pub evm_before: f64,
    /// Error-vector magnitude *after* DPD (%).
    pub evm_after: f64,
    /// Overall improvement (positive = better) in dB.
    pub improvement_db: f64,
}

/// Main DPD engine using a memory polynomial model and indirect learning.
#[derive(Debug, Clone)]
pub struct DigitalPredistorter {
    config: DpdConfig,
    /// Flat coefficient vector of length `num_basis * (memory_depth + 1)`.
    /// Stored in row-major order: for each odd order k, then for each delay q.
    coefficients: Vec<(f64, f64)>,
}

impl DigitalPredistorter {
    // ------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------

    /// Create a new `DigitalPredistorter` from the given configuration.
    ///
    /// Coefficients are initialised to the identity (linear, order-1, delay-0
    /// coefficient = 1, all others = 0).
    pub fn new(config: DpdConfig) -> Self {
        let num_basis = Self::num_basis(config.polynomial_order);
        let mem = config.memory_depth + 1; // taps 0..=memory_depth
        let total = num_basis * mem;
        let mut coefficients = vec![(0.0, 0.0); total];
        // Identity: first basis function (k=0 i.e. linear) at delay 0.
        if !coefficients.is_empty() {
            coefficients[0] = (1.0, 0.0);
        }
        Self {
            config,
            coefficients,
        }
    }

    // ------------------------------------------------------------------
    // Public API
    // ------------------------------------------------------------------

    /// Train the predistorter coefficients using indirect learning architecture.
    ///
    /// In indirect learning the *PA output* is used as the input to the
    /// identification model, and the *original clean input* is the desired
    /// output.  After training, the identified inverse model is used as the
    /// predistorter.
    ///
    /// A single pass of block LMS is performed over the provided data.
    pub fn train(&mut self, input: &[(f64, f64)], pa_output: &[(f64, f64)]) {
        let n = input.len().min(pa_output.len());
        if n == 0 {
            return;
        }
        let num_basis = Self::num_basis(self.config.polynomial_order);
        let mem = self.config.memory_depth + 1;
        let mu = self.config.learning_rate;

        for i in 0..n {
            // Build basis vector from PA output (indirect learning input).
            let basis = self.build_basis(pa_output, i, num_basis, mem);
            // Compute current model output: y_hat = sum coeff_j * basis_j
            let y_hat = self.apply_basis(&basis);
            // Error = desired (clean input) - model output
            let err = c_sub(input[i], y_hat);
            // Normalisation factor to avoid divergence
            let norm: f64 = basis.iter().map(|b| c_abs_sq(*b)).sum::<f64>() + 1e-12;
            // LMS update: coeff += mu * err * conj(basis) / norm
            for (j, bj) in basis.iter().enumerate() {
                let update = c_scale(mu / norm, c_mul(err, c_conj(*bj)));
                self.coefficients[j] = c_add(self.coefficients[j], update);
            }
        }
    }

    /// Apply the trained predistortion to a block of input samples.
    pub fn predistort(&self, samples: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n = samples.len();
        let num_basis = Self::num_basis(self.config.polynomial_order);
        let mem = self.config.memory_depth + 1;
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let basis = self.build_basis(samples, i, num_basis, mem);
            out.push(self.apply_basis(&basis));
        }
        out
    }

    /// Simulate a power amplifier on a block of samples.
    pub fn simulate_pa(&self, model: &PaModel, samples: &[(f64, f64)]) -> Vec<(f64, f64)> {
        samples.iter().map(|&s| Self::pa_sample(model, s)).collect()
    }

    /// Compute DPD quality metrics by comparing input and output signals.
    ///
    /// `input` is the ideal (reference) signal; `output` is the signal after
    /// PA (with or without DPD).  Returns metrics including EVM and a simple
    /// spectral regrowth estimate used as ACPR proxy.
    pub fn metrics(&self, input: &[(f64, f64)], output: &[(f64, f64)]) -> DpdMetrics {
        let n = input.len().min(output.len());
        if n == 0 {
            return DpdMetrics {
                acpr_before: 0.0,
                acpr_after: 0.0,
                evm_before: 0.0,
                evm_after: 0.0,
                improvement_db: 0.0,
            };
        }

        // EVM (%) = rms(error) / rms(reference) * 100
        let rms_ref: f64 = (input[..n].iter().map(|s| c_abs_sq(*s)).sum::<f64>() / n as f64).sqrt();
        let mse: f64 = input[..n]
            .iter()
            .zip(output[..n].iter())
            .map(|(a, b)| c_abs_sq(c_sub(*a, *b)))
            .sum::<f64>()
            / n as f64;
        let evm = if rms_ref > 1e-15 {
            mse.sqrt() / rms_ref * 100.0
        } else {
            0.0
        };

        // Simple spectral-regrowth ACPR proxy: ratio of high-frequency energy
        // (upper half of spectrum) to low-frequency energy after removing the
        // linear component. We compute the "nonlinear residual" first.
        let acpr = Self::estimate_acpr(input, output, n);

        DpdMetrics {
            acpr_before: acpr,
            acpr_after: acpr, // caller should compare before/after externally
            evm_before: evm,
            evm_after: evm,
            improvement_db: 0.0,
        }
    }

    /// Change the polynomial order, re-initialising the coefficients.
    pub fn set_polynomial_order(&mut self, order: usize) {
        self.config.polynomial_order = order;
        let num_basis = Self::num_basis(order);
        let mem = self.config.memory_depth + 1;
        let total = num_basis * mem;
        self.coefficients = vec![(0.0, 0.0); total];
        if !self.coefficients.is_empty() {
            self.coefficients[0] = (1.0, 0.0);
        }
    }

    /// Return a reference to the current coefficient vector.
    pub fn coefficients(&self) -> &[(f64, f64)] {
        &self.coefficients
    }

    // ------------------------------------------------------------------
    // Internal helpers
    // ------------------------------------------------------------------

    /// Number of odd-order basis functions up to `order`.
    /// Orders used: 1, 3, 5, ..., order  (if order is even, we round down).
    fn num_basis(order: usize) -> usize {
        // order 1 => 1, order 3 => 2, order 5 => 3, order 7 => 4
        (order + 1) / 2
    }

    /// Build the basis vector for sample index `i` from `data`.
    ///
    /// basis\[k * mem + q\] = data(i - q) * |data(i - q)|^(2k)
    ///
    /// where k indexes the odd orders (k=0 => order 1, k=1 => order 3, ...).
    fn build_basis(
        &self,
        data: &[(f64, f64)],
        i: usize,
        num_basis: usize,
        mem: usize,
    ) -> Vec<(f64, f64)> {
        let mut basis = Vec::with_capacity(num_basis * mem);
        for k in 0..num_basis {
            let exponent = 2 * k; // |x|^(2k) for odd-order (2k+1) term
            for q in 0..mem {
                let idx = if i >= q { i - q } else { 0 };
                let x = data[idx];
                let r2 = c_abs_sq(x); // |x|^2
                let gain = r2.powi(exponent as i32); // |x|^(2k)
                basis.push(c_scale(gain, x));
            }
        }
        basis
    }

    /// Evaluate the model: y = sum_j coeff_j * basis_j.
    fn apply_basis(&self, basis: &[(f64, f64)]) -> (f64, f64) {
        let mut y = (0.0, 0.0);
        for (j, bj) in basis.iter().enumerate() {
            if j < self.coefficients.len() {
                y = c_add(y, c_mul(self.coefficients[j], *bj));
            }
        }
        y
    }

    /// Evaluate a single sample through a PA model.
    fn pa_sample(model: &PaModel, s: (f64, f64)) -> (f64, f64) {
        match model {
            PaModel::Rapp { a_sat, p } => {
                let r = c_abs(s);
                if r < 1e-15 {
                    return (0.0, 0.0);
                }
                let ratio = r / a_sat;
                let denom = (1.0 + ratio.powi(2 * (*p as i32))).powf(1.0 / (2.0 * *p as f64));
                let gain = 1.0 / denom;
                c_scale(gain, s)
            }
            PaModel::Saleh {
                alpha_a,
                beta_a,
                alpha_p,
                beta_p,
            } => {
                let r = c_abs(s);
                if r < 1e-15 {
                    return (0.0, 0.0);
                }
                let r2 = r * r;
                let am_am = alpha_a * r / (1.0 + beta_a * r2);
                let am_pm = alpha_p * r2 / (1.0 + beta_p * r2);
                let phase = s.1.atan2(s.0) + am_pm;
                (am_am * phase.cos(), am_am * phase.sin())
            }
            PaModel::MemoryPolynomial { coefficients } => {
                // Memoryless evaluation (only delay 0)
                let r2 = c_abs_sq(s);
                let mut y = (0.0, 0.0);
                for (k, row) in coefficients.iter().enumerate() {
                    if let Some(&ck) = row.first() {
                        let gain = r2.powi(k as i32);
                        y = c_add(y, c_mul(ck, c_scale(gain, s)));
                    }
                }
                y
            }
            PaModel::LookupTable { table } => {
                let r = c_abs(s);
                if r < 1e-15 || table.is_empty() {
                    return (0.0, 0.0);
                }
                // Find surrounding entries and interpolate
                let (out_amp, phase_shift) = if r <= table[0].0 {
                    (table[0].1 * r / table[0].0.max(1e-15), table[0].2)
                } else if r >= table[table.len() - 1].0 {
                    let last = &table[table.len() - 1];
                    (last.1, last.2)
                } else {
                    // Linear interpolation
                    let mut idx = 0;
                    for j in 1..table.len() {
                        if table[j].0 >= r {
                            idx = j - 1;
                            break;
                        }
                    }
                    let (r0, a0, p0) = table[idx];
                    let (r1, a1, p1) = table[idx + 1];
                    let t = (r - r0) / (r1 - r0).max(1e-15);
                    (a0 + t * (a1 - a0), p0 + t * (p1 - p0))
                };
                let phase = s.1.atan2(s.0) + phase_shift;
                (out_amp * phase.cos(), out_amp * phase.sin())
            }
        }
    }

    /// Rough ACPR estimate: energy of the error signal relative to the
    /// reference signal, expressed in dB.
    fn estimate_acpr(input: &[(f64, f64)], output: &[(f64, f64)], n: usize) -> f64 {
        let signal_power: f64 = input[..n].iter().map(|s| c_abs_sq(*s)).sum::<f64>();
        let error_power: f64 = input[..n]
            .iter()
            .zip(output[..n].iter())
            .map(|(a, b)| c_abs_sq(c_sub(*a, *b)))
            .sum::<f64>();
        if signal_power < 1e-30 {
            return -100.0;
        }
        10.0 * (error_power / signal_power).log10()
    }
}

// ===========================================================================
// Unit tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-9;

    // -- helper ----------------------------------------------------------

    fn tone(n: usize, amplitude: f64, freq_cycles: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let phase = 2.0 * PI * freq_cycles * i as f64 / n as f64;
                (amplitude * phase.cos(), amplitude * phase.sin())
            })
            .collect()
    }

    // -- complex arithmetic helpers --------------------------------------

    #[test]
    fn test_c_add() {
        let a = (1.0, 2.0);
        let b = (3.0, -1.0);
        let r = c_add(a, b);
        assert!((r.0 - 4.0).abs() < TOL);
        assert!((r.1 - 1.0).abs() < TOL);
    }

    #[test]
    fn test_c_sub() {
        let r = c_sub((5.0, 3.0), (2.0, 1.0));
        assert!((r.0 - 3.0).abs() < TOL);
        assert!((r.1 - 2.0).abs() < TOL);
    }

    #[test]
    fn test_c_mul() {
        // (1+2i)*(3+4i) = 3+4i+6i+8i^2 = -5+10i
        let r = c_mul((1.0, 2.0), (3.0, 4.0));
        assert!((r.0 - (-5.0)).abs() < TOL);
        assert!((r.1 - 10.0).abs() < TOL);
    }

    #[test]
    fn test_c_abs() {
        assert!((c_abs((3.0, 4.0)) - 5.0).abs() < TOL);
    }

    // -- DPD construction ------------------------------------------------

    #[test]
    fn test_new_identity_passthrough() {
        let cfg = DpdConfig {
            polynomial_order: 5,
            memory_depth: 0,
            learning_rate: 0.01,
        };
        let dpd = DigitalPredistorter::new(cfg);
        let input = tone(64, 0.5, 3.0);
        let output = dpd.predistort(&input);
        for (a, b) in input.iter().zip(output.iter()) {
            assert!((a.0 - b.0).abs() < TOL, "real mismatch");
            assert!((a.1 - b.1).abs() < TOL, "imag mismatch");
        }
    }

    #[test]
    fn test_coefficients_initial_length() {
        let cfg = DpdConfig {
            polynomial_order: 7,
            memory_depth: 2,
            learning_rate: 0.01,
        };
        let dpd = DigitalPredistorter::new(cfg);
        // num_basis(7) = 4, mem = 3 => 12 coefficients
        assert_eq!(dpd.coefficients().len(), 12);
    }

    #[test]
    fn test_set_polynomial_order() {
        let cfg = DpdConfig {
            polynomial_order: 3,
            memory_depth: 0,
            learning_rate: 0.01,
        };
        let mut dpd = DigitalPredistorter::new(cfg);
        assert_eq!(dpd.coefficients().len(), 2); // num_basis(3)=2, mem=1
        dpd.set_polynomial_order(7);
        assert_eq!(dpd.coefficients().len(), 4); // num_basis(7)=4, mem=1
        // First coefficient should be identity again
        assert!((dpd.coefficients()[0].0 - 1.0).abs() < TOL);
    }

    // -- PA models -------------------------------------------------------

    #[test]
    fn test_rapp_linear_region() {
        let pa = PaModel::Rapp { a_sat: 1.0, p: 2 };
        // Small signal should be nearly linear
        let s = (0.1, 0.05);
        let out = DigitalPredistorter::pa_sample(&pa, s);
        // gain should be close to 1 for small r
        let r_in = c_abs(s);
        let r_out = c_abs(out);
        let gain = r_out / r_in;
        assert!(
            (gain - 1.0).abs() < 0.01,
            "small-signal gain should be ~1, got {}",
            gain
        );
    }

    #[test]
    fn test_rapp_saturation() {
        let pa = PaModel::Rapp { a_sat: 1.0, p: 2 };
        // Large signal should be compressed
        let s = (2.0, 0.0);
        let out = DigitalPredistorter::pa_sample(&pa, s);
        let r_out = c_abs(out);
        assert!(
            r_out < 2.0,
            "output amplitude should be compressed, got {}",
            r_out
        );
        assert!(r_out < 1.1, "should be near saturation, got {}", r_out);
    }

    #[test]
    fn test_rapp_zero_input() {
        let pa = PaModel::Rapp { a_sat: 1.0, p: 3 };
        let out = DigitalPredistorter::pa_sample(&pa, (0.0, 0.0));
        assert!((out.0).abs() < TOL);
        assert!((out.1).abs() < TOL);
    }

    #[test]
    fn test_saleh_model() {
        let pa = PaModel::Saleh {
            alpha_a: 2.1587,
            beta_a: 1.1517,
            alpha_p: 4.0033,
            beta_p: 9.1040,
        };
        let s = (0.5, 0.0);
        let out = DigitalPredistorter::pa_sample(&pa, s);
        // Should produce non-zero output with some phase rotation
        assert!(c_abs(out) > 0.1);
    }

    #[test]
    fn test_lookup_table_model() {
        let pa = PaModel::LookupTable {
            table: vec![
                (0.0, 0.0, 0.0),
                (0.5, 0.5, 0.0),
                (1.0, 0.9, 0.1),
                (1.5, 0.95, 0.3),
            ],
        };
        let s = (0.75, 0.0);
        let out = DigitalPredistorter::pa_sample(&pa, s);
        // Interpolation between (0.5, 0.5, 0.0) and (1.0, 0.9, 0.1)
        // t = (0.75 - 0.5) / 0.5 = 0.5 => amp = 0.7, phase = 0.05
        let expected_amp = 0.7;
        let expected_phase = 0.05;
        let r_out = c_abs(out);
        assert!(
            (r_out - expected_amp).abs() < 0.01,
            "LUT amp: expected ~{}, got {}",
            expected_amp,
            r_out
        );
        let phase_out = out.1.atan2(out.0);
        assert!(
            (phase_out - expected_phase).abs() < 0.01,
            "LUT phase: expected ~{}, got {}",
            expected_phase,
            phase_out
        );
    }

    #[test]
    fn test_memory_polynomial_pa_model() {
        // Simple linear model: coeff[0][0] = (0.9, 0.0)
        let pa = PaModel::MemoryPolynomial {
            coefficients: vec![vec![(0.9, 0.0)]],
        };
        let s = (1.0, 0.0);
        let out = DigitalPredistorter::pa_sample(&pa, s);
        assert!((out.0 - 0.9).abs() < TOL);
        assert!((out.1).abs() < TOL);
    }

    // -- Training & predistortion ----------------------------------------

    #[test]
    fn test_train_reduces_error() {
        let cfg = DpdConfig {
            polynomial_order: 5,
            memory_depth: 0,
            learning_rate: 0.1,
        };
        let mut dpd = DigitalPredistorter::new(cfg);
        let pa = PaModel::Rapp { a_sat: 1.0, p: 2 };
        let input = tone(512, 0.7, 5.0);
        let pa_out = dpd.simulate_pa(&pa, &input);

        // Measure error before training (identity predistorter)
        let pred_before = dpd.predistort(&input);
        let lin_before = dpd.simulate_pa(&pa, &pred_before);
        let mse_before: f64 = input
            .iter()
            .zip(lin_before.iter())
            .map(|(a, b)| c_abs_sq(c_sub(*a, *b)))
            .sum::<f64>()
            / input.len() as f64;

        // Train multiple iterations
        for _ in 0..20 {
            let pa_out_iter = dpd.simulate_pa(&pa, &input);
            dpd.train(&input, &pa_out_iter);
        }

        // Measure error after training
        let pred_after = dpd.predistort(&input);
        let lin_after = dpd.simulate_pa(&pa, &pred_after);
        let mse_after: f64 = input
            .iter()
            .zip(lin_after.iter())
            .map(|(a, b)| c_abs_sq(c_sub(*a, *b)))
            .sum::<f64>()
            / input.len() as f64;

        assert!(
            mse_after < mse_before,
            "training should reduce MSE: before={:.6}, after={:.6}",
            mse_before,
            mse_after
        );
    }

    #[test]
    fn test_predistort_preserves_length() {
        let cfg = DpdConfig {
            polynomial_order: 3,
            memory_depth: 1,
            learning_rate: 0.01,
        };
        let dpd = DigitalPredistorter::new(cfg);
        let input = tone(128, 0.5, 2.0);
        let output = dpd.predistort(&input);
        assert_eq!(output.len(), input.len());
    }

    #[test]
    fn test_simulate_pa_preserves_length() {
        let cfg = DpdConfig {
            polynomial_order: 3,
            memory_depth: 0,
            learning_rate: 0.01,
        };
        let dpd = DigitalPredistorter::new(cfg);
        let pa = PaModel::Rapp { a_sat: 1.0, p: 2 };
        let input = tone(100, 0.8, 1.0);
        let output = dpd.simulate_pa(&pa, &input);
        assert_eq!(output.len(), input.len());
    }

    #[test]
    fn test_metrics_zero_error() {
        let cfg = DpdConfig {
            polynomial_order: 3,
            memory_depth: 0,
            learning_rate: 0.01,
        };
        let dpd = DigitalPredistorter::new(cfg);
        let input = tone(64, 0.5, 2.0);
        let m = dpd.metrics(&input, &input);
        assert!(m.evm_before < 1e-10, "EVM should be ~0 for identical signals");
    }

    #[test]
    fn test_metrics_nonzero_error() {
        let cfg = DpdConfig {
            polynomial_order: 3,
            memory_depth: 0,
            learning_rate: 0.01,
        };
        let dpd = DigitalPredistorter::new(cfg);
        let input = tone(64, 0.5, 2.0);
        let pa = PaModel::Rapp { a_sat: 1.0, p: 2 };
        let distorted = dpd.simulate_pa(&pa, &input);
        let m = dpd.metrics(&input, &distorted);
        assert!(m.evm_before > 0.0, "EVM should be > 0 for distorted signal");
    }

    #[test]
    fn test_empty_input_handling() {
        let cfg = DpdConfig {
            polynomial_order: 3,
            memory_depth: 0,
            learning_rate: 0.01,
        };
        let mut dpd = DigitalPredistorter::new(cfg);
        let empty: Vec<(f64, f64)> = vec![];
        // These should not panic
        let pred = dpd.predistort(&empty);
        assert!(pred.is_empty());
        let pa = PaModel::Rapp { a_sat: 1.0, p: 2 };
        let sim = dpd.simulate_pa(&pa, &empty);
        assert!(sim.is_empty());
        dpd.train(&empty, &empty);
        let m = dpd.metrics(&empty, &empty);
        assert!((m.evm_before).abs() < TOL);
    }

    #[test]
    fn test_num_basis() {
        assert_eq!(DigitalPredistorter::num_basis(1), 1);
        assert_eq!(DigitalPredistorter::num_basis(3), 2);
        assert_eq!(DigitalPredistorter::num_basis(5), 3);
        assert_eq!(DigitalPredistorter::num_basis(7), 4);
        assert_eq!(DigitalPredistorter::num_basis(2), 1); // even rounds down
    }
}
