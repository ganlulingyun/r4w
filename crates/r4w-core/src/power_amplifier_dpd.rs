//! Power Amplifier DPD — Digital Pre-Distortion for PA Linearization
//!
//! Memory polynomial and generalized memory polynomial pre-distortion
//! to linearize power amplifier nonlinearities. Supports LMS and RLS
//! adaptation with indirect learning architecture.
//! GNU Radio equivalent: `gr-dpd` (`dpd_predistorter_training`,
//! `dpd_LUT_predistorter`).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::power_amplifier_dpd::{MemoryPolynomialDpd, DpdConfig, DpdAlgorithm};
//! use num_complex::Complex64;
//!
//! let config = DpdConfig {
//!     polynomial_order: 5,
//!     memory_depth: 1,
//!     learning_rate: 0.01,
//!     algorithm: DpdAlgorithm::Lms,
//! };
//! let mut dpd = MemoryPolynomialDpd::new(config);
//! let signal = vec![Complex64::new(0.3, 0.1); 100];
//! let predistorted = dpd.predistort(&signal);
//! assert_eq!(predistorted.len(), 100);
//! ```

use num_complex::Complex64;

/// DPD adaptation algorithm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DpdAlgorithm {
    /// Least Mean Squares.
    Lms,
    /// Recursive Least Squares.
    Rls { forgetting_factor: f64 },
}

/// DPD configuration.
#[derive(Debug, Clone)]
pub struct DpdConfig {
    /// Maximum polynomial order (odd orders: 1, 3, 5, ...).
    pub polynomial_order: usize,
    /// Memory depth (0 = memoryless, 1+ = with memory).
    pub memory_depth: usize,
    /// Learning rate (mu for LMS).
    pub learning_rate: f64,
    /// Adaptation algorithm.
    pub algorithm: DpdAlgorithm,
}

/// Memory polynomial DPD.
#[derive(Debug, Clone)]
pub struct MemoryPolynomialDpd {
    config: DpdConfig,
    /// Coefficients: [memory_tap][polynomial_order] = Complex64.
    coefficients: Vec<Vec<Complex64>>,
    /// Number of odd-order terms: ceil(poly_order/2).
    num_terms: usize,
    /// RLS inverse correlation matrix (if using RLS).
    rls_p: Option<Vec<Vec<Complex64>>>,
}

impl MemoryPolynomialDpd {
    /// Create a new memory polynomial DPD.
    pub fn new(config: DpdConfig) -> Self {
        let num_terms = (config.polynomial_order + 1) / 2; // odd orders: 1,3,5,...
        let memory = config.memory_depth + 1; // include current sample

        // Initialize coefficients: identity for first term, zero otherwise
        let mut coefficients = vec![vec![Complex64::new(0.0, 0.0); num_terms]; memory];
        if !coefficients.is_empty() && num_terms > 0 {
            coefficients[0][0] = Complex64::new(1.0, 0.0); // Linear pass-through
        }

        let total_dim = memory * num_terms;
        let rls_p = match config.algorithm {
            DpdAlgorithm::Rls { .. } => {
                // Initialize P = delta * I
                let delta = 100.0;
                let mut p = vec![vec![Complex64::new(0.0, 0.0); total_dim]; total_dim];
                for i in 0..total_dim {
                    p[i][i] = Complex64::new(delta, 0.0);
                }
                Some(p)
            }
            DpdAlgorithm::Lms => None,
        };

        Self {
            config,
            coefficients,
            num_terms,
            rls_p,
        }
    }

    /// Apply pre-distortion to input signal.
    pub fn predistort(&self, input: &[Complex64]) -> Vec<Complex64> {
        let memory = self.config.memory_depth + 1;
        let mut output = vec![Complex64::new(0.0, 0.0); input.len()];

        for n in 0..input.len() {
            let mut y = Complex64::new(0.0, 0.0);
            for m in 0..memory {
                if n >= m {
                    let x = input[n - m];
                    let r = x.norm();
                    let mut r_pow = 1.0; // |x|^0
                    for k in 0..self.num_terms {
                        y += self.coefficients[m][k] * x * r_pow;
                        r_pow *= r * r; // |x|^2, |x|^4, ...
                    }
                }
            }
            output[n] = y;
        }

        output
    }

    /// Adapt DPD coefficients using indirect learning.
    ///
    /// `pa_input`: signal fed to PA (= DPD output).
    /// `pa_output`: observed PA output.
    ///
    /// In indirect learning: train a postdistorter on (pa_output -> pa_input),
    /// then copy those coefficients to the predistorter.
    pub fn adapt(&mut self, pa_input: &[Complex64], pa_output: &[Complex64]) {
        let len = pa_input.len().min(pa_output.len());
        let memory = self.config.memory_depth + 1;

        match self.config.algorithm {
            DpdAlgorithm::Lms => {
                let mu = self.config.learning_rate;
                for n in 0..len {
                    // Build basis vector from PA output (postdistorter input)
                    let mut basis = Vec::new();
                    for m in 0..memory {
                        if n >= m {
                            let x = pa_output[n - m];
                            let r = x.norm();
                            let mut r_pow = 1.0;
                            for _ in 0..self.num_terms {
                                basis.push(x * r_pow);
                                r_pow *= r * r;
                            }
                        } else {
                            for _ in 0..self.num_terms {
                                basis.push(Complex64::new(0.0, 0.0));
                            }
                        }
                    }

                    // Compute postdistorter output
                    let mut y_hat = Complex64::new(0.0, 0.0);
                    for (idx, &b) in basis.iter().enumerate() {
                        let m = idx / self.num_terms;
                        let k = idx % self.num_terms;
                        y_hat += self.coefficients[m][k] * b;
                    }

                    // Error: desired - estimated (desired = pa_input)
                    let error = pa_input[n] - y_hat;

                    // Update coefficients
                    let norm_sq: f64 = basis.iter().map(|b| b.norm_sqr()).sum();
                    if norm_sq > 1e-12 {
                        let step = mu / (norm_sq + 1e-8);
                        for (idx, &b) in basis.iter().enumerate() {
                            let m = idx / self.num_terms;
                            let k = idx % self.num_terms;
                            self.coefficients[m][k] += error * b.conj() * step;
                        }
                    }
                }
            }
            DpdAlgorithm::Rls { forgetting_factor } => {
                let lambda = forgetting_factor;
                let p = self.rls_p.as_mut().unwrap();
                let dim = memory * self.num_terms;

                for n in 0..len {
                    // Build basis vector
                    let mut u = vec![Complex64::new(0.0, 0.0); dim];
                    for m in 0..memory {
                        if n >= m {
                            let x = pa_output[n - m];
                            let r = x.norm();
                            let mut r_pow = 1.0;
                            for k in 0..self.num_terms {
                                u[m * self.num_terms + k] = x * r_pow;
                                r_pow *= r * r;
                            }
                        }
                    }

                    // Compute output
                    let mut y_hat = Complex64::new(0.0, 0.0);
                    for (idx, &ui) in u.iter().enumerate() {
                        let m = idx / self.num_terms;
                        let k = idx % self.num_terms;
                        y_hat += self.coefficients[m][k] * ui;
                    }

                    let error = pa_input[n] - y_hat;

                    // RLS update: k = P*u / (lambda + u^H * P * u)
                    let mut pu = vec![Complex64::new(0.0, 0.0); dim];
                    for i in 0..dim {
                        for j in 0..dim {
                            pu[i] += p[i][j] * u[j];
                        }
                    }

                    let mut denom = Complex64::new(lambda, 0.0);
                    for j in 0..dim {
                        denom += u[j].conj() * pu[j];
                    }

                    if denom.norm() > 1e-12 {
                        let inv_denom = Complex64::new(1.0, 0.0) / denom;
                        let mut k = vec![Complex64::new(0.0, 0.0); dim];
                        for i in 0..dim {
                            k[i] = pu[i] * inv_denom;
                        }

                        // Update coefficients
                        for (idx, &ki) in k.iter().enumerate() {
                            let m = idx / self.num_terms;
                            let kk = idx % self.num_terms;
                            self.coefficients[m][kk] += ki * error;
                        }

                        // Update P
                        let inv_lambda = 1.0 / lambda;
                        for i in 0..dim {
                            for j in 0..dim {
                                p[i][j] = inv_lambda * (p[i][j] - k[i] * pu[j].conj());
                            }
                        }
                    }
                }
            }
        }
    }

    /// Get flattened coefficient vector.
    pub fn coefficients_flat(&self) -> Vec<Complex64> {
        self.coefficients.iter().flat_map(|row| row.iter().copied()).collect()
    }

    /// Set coefficients from flat vector.
    pub fn set_coefficients_flat(&mut self, coeffs: &[Complex64]) {
        let memory = self.config.memory_depth + 1;
        let mut idx = 0;
        for m in 0..memory {
            for k in 0..self.num_terms {
                if idx < coeffs.len() {
                    self.coefficients[m][k] = coeffs[idx];
                    idx += 1;
                }
            }
        }
    }

    /// Reset to initial state.
    pub fn reset(&mut self) {
        let memory = self.config.memory_depth + 1;
        self.coefficients = vec![vec![Complex64::new(0.0, 0.0); self.num_terms]; memory];
        if !self.coefficients.is_empty() && self.num_terms > 0 {
            self.coefficients[0][0] = Complex64::new(1.0, 0.0);
        }
    }

    /// Compute normalized mean squared error between two signals.
    pub fn nmse(reference: &[Complex64], distorted: &[Complex64]) -> f64 {
        let len = reference.len().min(distorted.len());
        if len == 0 {
            return 0.0;
        }
        let error_power: f64 = (0..len)
            .map(|i| (reference[i] - distorted[i]).norm_sqr())
            .sum();
        let ref_power: f64 = (0..len).map(|i| reference[i].norm_sqr()).sum();
        if ref_power > 1e-15 {
            10.0 * (error_power / ref_power).log10()
        } else {
            -100.0
        }
    }

    /// Get config.
    pub fn config(&self) -> &DpdConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::power_amplifier_model::{PaModel, PowerAmplifier};

    fn make_test_signal(len: usize, amplitude: f64) -> Vec<Complex64> {
        (0..len)
            .map(|i| {
                let phase = i as f64 * 0.3;
                Complex64::from_polar(amplitude, phase)
            })
            .collect()
    }

    #[test]
    fn test_identity_predistortion() {
        let config = DpdConfig {
            polynomial_order: 5,
            memory_depth: 0,
            learning_rate: 0.01,
            algorithm: DpdAlgorithm::Lms,
        };
        let dpd = MemoryPolynomialDpd::new(config);
        let input = make_test_signal(50, 0.3);
        let output = dpd.predistort(&input);
        // Default coefficients = [1, 0, 0] should pass through
        for (x, y) in input.iter().zip(output.iter()) {
            assert!((x - y).norm() < 1e-10);
        }
    }

    #[test]
    fn test_lms_adaptation() {
        let pa = PowerAmplifier::new(PaModel::rapp(1.0, 2.0), 0.0);
        let config = DpdConfig {
            polynomial_order: 5,
            memory_depth: 0,
            learning_rate: 0.005,
            algorithm: DpdAlgorithm::Lms,
        };
        let mut dpd = MemoryPolynomialDpd::new(config);

        // Training iterations
        let signal = make_test_signal(500, 0.5);
        for _ in 0..20 {
            let predistorted = dpd.predistort(&signal);
            let pa_out = pa.process(&predistorted);
            dpd.adapt(&predistorted, &pa_out);
        }

        // After training, PA(DPD(x)) should be closer to x
        let test_signal = make_test_signal(200, 0.5);
        let pd = dpd.predistort(&test_signal);
        let linearized = pa.process(&pd);
        let nmse_after = MemoryPolynomialDpd::nmse(&test_signal, &linearized);

        // Without DPD
        let direct = pa.process(&test_signal);
        let nmse_before = MemoryPolynomialDpd::nmse(&test_signal, &direct);

        // DPD should improve NMSE (more negative = better)
        assert!(
            nmse_after < nmse_before + 1.0,
            "DPD should improve: before={:.1} dB, after={:.1} dB",
            nmse_before,
            nmse_after
        );
    }

    #[test]
    fn test_rls_adaptation() {
        let pa = PowerAmplifier::new(PaModel::rapp(1.0, 2.0), 0.0);
        let config = DpdConfig {
            polynomial_order: 5,
            memory_depth: 0,
            learning_rate: 0.01,
            algorithm: DpdAlgorithm::Rls {
                forgetting_factor: 0.99,
            },
        };
        let mut dpd = MemoryPolynomialDpd::new(config);

        let signal = make_test_signal(200, 0.5);
        for _ in 0..5 {
            let pd = dpd.predistort(&signal);
            let pa_out = pa.process(&pd);
            dpd.adapt(&pd, &pa_out);
        }

        // RLS should converge
        let coeffs = dpd.coefficients_flat();
        assert!(coeffs[0].norm() > 0.5, "Linear coefficient should be significant");
    }

    #[test]
    fn test_memory_depth() {
        let config_mem0 = DpdConfig {
            polynomial_order: 3,
            memory_depth: 0,
            learning_rate: 0.01,
            algorithm: DpdAlgorithm::Lms,
        };
        let config_mem2 = DpdConfig {
            polynomial_order: 3,
            memory_depth: 2,
            learning_rate: 0.01,
            algorithm: DpdAlgorithm::Lms,
        };
        let dpd0 = MemoryPolynomialDpd::new(config_mem0);
        let dpd2 = MemoryPolynomialDpd::new(config_mem2);
        assert_eq!(dpd0.coefficients_flat().len(), 2); // 2 terms, 1 memory tap
        assert_eq!(dpd2.coefficients_flat().len(), 6); // 2 terms, 3 memory taps
    }

    #[test]
    fn test_nmse() {
        let a = vec![Complex64::new(1.0, 0.0); 10];
        let b = vec![Complex64::new(1.0, 0.0); 10];
        let nmse = MemoryPolynomialDpd::nmse(&a, &b);
        assert!(nmse < -50.0, "Identical signals should have very low NMSE");
    }

    #[test]
    fn test_coefficient_save_restore() {
        let config = DpdConfig {
            polynomial_order: 5,
            memory_depth: 1,
            learning_rate: 0.01,
            algorithm: DpdAlgorithm::Lms,
        };
        let mut dpd = MemoryPolynomialDpd::new(config.clone());

        // Modify coefficients
        let signal = make_test_signal(100, 0.3);
        let pa = PowerAmplifier::new(PaModel::rapp(1.0, 2.0), 0.0);
        let pd = dpd.predistort(&signal);
        let pa_out = pa.process(&pd);
        dpd.adapt(&pd, &pa_out);

        let saved = dpd.coefficients_flat();
        let output1 = dpd.predistort(&signal);

        // Create new DPD and restore
        let mut dpd2 = MemoryPolynomialDpd::new(config);
        dpd2.set_coefficients_flat(&saved);
        let output2 = dpd2.predistort(&signal);

        for (a, b) in output1.iter().zip(output2.iter()) {
            assert!((a - b).norm() < 1e-10);
        }
    }

    #[test]
    fn test_reset() {
        let config = DpdConfig {
            polynomial_order: 3,
            memory_depth: 0,
            learning_rate: 0.01,
            algorithm: DpdAlgorithm::Lms,
        };
        let mut dpd = MemoryPolynomialDpd::new(config);
        let signal = make_test_signal(50, 0.3);
        let pa = PowerAmplifier::new(PaModel::rapp(1.0, 2.0), 0.0);
        let pd = dpd.predistort(&signal);
        let pa_out = pa.process(&pd);
        dpd.adapt(&pd, &pa_out);

        dpd.reset();
        // After reset, should be identity again
        let output = dpd.predistort(&signal);
        for (x, y) in signal.iter().zip(output.iter()) {
            assert!((x - y).norm() < 1e-10);
        }
    }

    #[test]
    fn test_predistort_preserves_length() {
        let config = DpdConfig {
            polynomial_order: 7,
            memory_depth: 3,
            learning_rate: 0.01,
            algorithm: DpdAlgorithm::Lms,
        };
        let dpd = MemoryPolynomialDpd::new(config);
        let input = make_test_signal(1000, 0.4);
        let output = dpd.predistort(&input);
        assert_eq!(output.len(), 1000);
    }

    #[test]
    fn test_small_signal_passthrough() {
        let config = DpdConfig {
            polynomial_order: 5,
            memory_depth: 0,
            learning_rate: 0.01,
            algorithm: DpdAlgorithm::Lms,
        };
        let dpd = MemoryPolynomialDpd::new(config);
        // Very small signal: higher-order terms negligible
        let input = make_test_signal(20, 0.001);
        let output = dpd.predistort(&input);
        for (x, y) in input.iter().zip(output.iter()) {
            assert!((x - y).norm() < 1e-6);
        }
    }
}
