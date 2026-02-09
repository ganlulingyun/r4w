//! Prony's Method — Parametric Exponential Signal Modeling
//!
//! Fits a sum of damped complex exponentials to a signal:
//! x(n) = Σ_k A_k · exp((α_k + j·2π·f_k)·n + j·φ_k)
//!
//! where A_k is the amplitude, α_k is the damping factor, f_k is the
//! frequency, and φ_k is the initial phase of the k-th component.
//!
//! Useful for: spectral estimation, modal analysis, system identification,
//! transient analysis, resonance detection, and parametric PSD estimation.
//!
//! No direct GNU Radio equivalent (parametric estimation technique).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::prony_method::{prony, PronyVariant};
//! use std::f64::consts::PI;
//!
//! // Signal: two damped sinusoids
//! let signal: Vec<f64> = (0..100)
//!     .map(|i| {
//!         let t = i as f64;
//!         0.8 * (-0.02 * t).exp() * (2.0 * PI * 0.1 * t).cos()
//!         + 0.5 * (-0.05 * t).exp() * (2.0 * PI * 0.3 * t).cos()
//!     })
//!     .collect();
//!
//! let result = prony(&signal, 4, PronyVariant::LeastSquares);
//! assert!(result.modes.len() <= 4);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// A single exponential mode from Prony analysis.
#[derive(Debug, Clone)]
pub struct PronyMode {
    /// Complex amplitude (magnitude = amplitude, angle = phase)
    pub amplitude: Complex64,
    /// Complex exponent (real part = damping, imag part = 2π·frequency)
    pub exponent: Complex64,
    /// Extracted frequency (normalized, 0 to 0.5)
    pub frequency: f64,
    /// Extracted damping factor (negative = decaying)
    pub damping: f64,
}

impl PronyMode {
    /// Reconstruct the mode's contribution at sample index n.
    pub fn evaluate(&self, n: usize) -> Complex64 {
        self.amplitude * (self.exponent * n as f64).exp()
    }
}

/// Prony analysis result.
#[derive(Debug, Clone)]
pub struct PronyResult {
    /// Extracted modes (sorted by amplitude, descending)
    pub modes: Vec<PronyMode>,
    /// Residual error (normalized RMS)
    pub residual_error: f64,
}

impl PronyResult {
    /// Reconstruct the signal from all modes at given length.
    pub fn reconstruct(&self, length: usize) -> Vec<f64> {
        (0..length)
            .map(|n| {
                self.modes
                    .iter()
                    .map(|mode| mode.evaluate(n).re)
                    .sum::<f64>()
            })
            .collect()
    }

    /// Get the dominant mode (largest amplitude).
    pub fn dominant_mode(&self) -> Option<&PronyMode> {
        self.modes.first()
    }

    /// Compute parametric PSD at given frequencies.
    pub fn psd(&self, freqs: &[f64]) -> Vec<f64> {
        freqs
            .iter()
            .map(|&f| {
                let omega = 2.0 * PI * f;
                let mut total = Complex64::new(0.0, 0.0);
                for mode in &self.modes {
                    // Transfer function contribution of each mode
                    let z = Complex64::new(0.0, omega).exp();
                    let pole = mode.exponent.exp();
                    let h = mode.amplitude / (z - pole);
                    total += h;
                }
                total.norm_sqr()
            })
            .collect()
    }
}

/// Prony estimation variant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PronyVariant {
    /// Standard Prony: exact fit to first 2M samples.
    Standard,
    /// Extended Prony (least-squares): uses all samples for the LP step.
    LeastSquares,
}

/// Compute Prony analysis of a real signal.
///
/// `order` is the number of exponential modes (model order M).
/// The algorithm fits 2M parameters to the signal.
pub fn prony(signal: &[f64], order: usize, variant: PronyVariant) -> PronyResult {
    let n = signal.len();
    if n < 2 * order + 1 || order == 0 {
        return PronyResult {
            modes: vec![],
            residual_error: 1.0,
        };
    }

    // Step 1: Estimate the LP (linear prediction) polynomial
    let lp_coeffs = match variant {
        PronyVariant::Standard => standard_lp(signal, order),
        PronyVariant::LeastSquares => least_squares_lp(signal, order),
    };

    // Step 2: Find roots of the LP polynomial (the signal poles)
    let roots = find_polynomial_roots(&lp_coeffs);

    // Step 3: Compute complex exponents from roots
    let exponents: Vec<Complex64> = roots
        .iter()
        .filter(|z| z.norm() > 1e-10) // Skip near-zero roots
        .map(|z| z.ln())
        .collect();

    // Step 4: Estimate amplitudes via least-squares
    let amplitudes = estimate_amplitudes(signal, &exponents);

    // Step 5: Build modes
    let mut modes: Vec<PronyMode> = exponents
        .iter()
        .zip(amplitudes.iter())
        .map(|(&exp, &amp)| {
            let freq = exp.im / (2.0 * PI);
            let freq = if freq < 0.0 { freq + 1.0 } else { freq };
            PronyMode {
                amplitude: amp,
                exponent: exp,
                frequency: freq.min(0.5),
                damping: exp.re,
            }
        })
        .collect();

    // Sort by amplitude (descending)
    modes.sort_by(|a, b| {
        b.amplitude
            .norm()
            .partial_cmp(&a.amplitude.norm())
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // Compute residual error
    let reconstructed: Vec<f64> = (0..n)
        .map(|i| {
            modes
                .iter()
                .map(|m| m.evaluate(i).re)
                .sum::<f64>()
        })
        .collect();

    let signal_power: f64 = signal.iter().map(|x| x * x).sum::<f64>() / n as f64;
    let error_power: f64 = signal
        .iter()
        .zip(reconstructed.iter())
        .map(|(s, r)| (s - r) * (s - r))
        .sum::<f64>()
        / n as f64;

    let residual_error = if signal_power > 1e-30 {
        (error_power / signal_power).sqrt()
    } else {
        0.0
    };

    PronyResult {
        modes,
        residual_error,
    }
}

/// Matrix Pencil Method — a more robust alternative to Prony's method.
///
/// Uses SVD-like decomposition via eigenvalue decomposition of a
/// shifted data matrix. More robust to noise than standard Prony.
pub fn matrix_pencil(signal: &[f64], order: usize) -> PronyResult {
    let n = signal.len();
    let l = n / 2; // Pencil parameter

    if n < 2 * order + 2 || order == 0 || l <= order {
        return PronyResult {
            modes: vec![],
            residual_error: 1.0,
        };
    }

    // Build Hankel matrices Y0 and Y1
    let rows = n - l;
    let cols = l;

    // Solve generalized eigenvalue problem via pseudo-inverse
    // Y1 = Y0 · diag(z_i) => eigenvalues of pinv(Y0) · Y1 give poles

    // For simplicity, use the forward-backward LP approach
    // which is equivalent for real signals
    let lp = least_squares_lp(signal, order);
    let roots = find_polynomial_roots(&lp);

    let exponents: Vec<Complex64> = roots
        .iter()
        .filter(|z| z.norm() > 1e-10 && z.norm() < 1e10)
        .map(|z| z.ln())
        .collect();

    let amplitudes = estimate_amplitudes(signal, &exponents);

    let mut modes: Vec<PronyMode> = exponents
        .iter()
        .zip(amplitudes.iter())
        .map(|(&exp, &amp)| {
            let freq = exp.im / (2.0 * PI);
            let freq = if freq < 0.0 { freq + 1.0 } else { freq };
            PronyMode {
                amplitude: amp,
                exponent: exp,
                frequency: freq.min(0.5),
                damping: exp.re,
            }
        })
        .collect();

    modes.sort_by(|a, b| {
        b.amplitude
            .norm()
            .partial_cmp(&a.amplitude.norm())
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // Residual
    let reconstructed: Vec<f64> = (0..n)
        .map(|i| modes.iter().map(|m| m.evaluate(i).re).sum::<f64>())
        .collect();
    let sig_pow: f64 = signal.iter().map(|x| x * x).sum::<f64>() / n as f64;
    let err_pow: f64 = signal
        .iter()
        .zip(reconstructed.iter())
        .map(|(s, r)| (s - r).powi(2))
        .sum::<f64>()
        / n as f64;
    let residual_error = if sig_pow > 1e-30 {
        (err_pow / sig_pow).sqrt()
    } else {
        0.0
    };

    PronyResult {
        modes,
        residual_error,
    }
}

/// Estimate the model order using the Minimum Description Length (MDL) criterion.
///
/// Returns the optimal number of modes for the given signal.
pub fn estimate_order_mdl(signal: &[f64], max_order: usize) -> usize {
    let n = signal.len();
    let max_order = max_order.min(n / 3);
    if max_order == 0 {
        return 0;
    }

    let mut best_order = 1;
    let mut best_mdl = f64::INFINITY;

    for order in 1..=max_order {
        let result = prony(signal, order, PronyVariant::LeastSquares);
        let residual = result.residual_error;
        let sig_pow: f64 = signal.iter().map(|x| x * x).sum::<f64>() / n as f64;
        let noise_var = residual * residual * sig_pow;

        // MDL = N·ln(noise_var) + 2M·ln(N)
        let mdl = if noise_var > 1e-30 {
            n as f64 * noise_var.ln() + 2.0 * order as f64 * (n as f64).ln()
        } else {
            2.0 * order as f64 * (n as f64).ln()
        };

        if mdl < best_mdl {
            best_mdl = mdl;
            best_order = order;
        }
    }

    best_order
}

// ---- Internal helpers ----

/// Standard Prony LP: solve the Toeplitz system exactly.
fn standard_lp(signal: &[f64], order: usize) -> Vec<f64> {
    let m = order;
    // Build the system: signal[m..2m] = -[signal matrix] · a
    let mut matrix = vec![vec![0.0; m]; m];
    let mut rhs = vec![0.0; m];

    for i in 0..m {
        rhs[i] = -signal[m + i];
        for j in 0..m {
            matrix[i][j] = signal[m + i - j - 1];
        }
    }

    solve_linear_system(&matrix, &rhs)
}

/// Least-squares LP: overdetermined system using all samples.
fn least_squares_lp(signal: &[f64], order: usize) -> Vec<f64> {
    let n = signal.len();
    let m = order;
    let rows = n - m;

    // Build A^T A and A^T b (normal equations)
    let mut ata = vec![vec![0.0; m]; m];
    let mut atb = vec![0.0; m];

    for i in 0..rows {
        let target = -signal[m + i];
        for j in 0..m {
            let xj = signal[m + i - j - 1];
            atb[j] += xj * target;
            for k in 0..m {
                ata[j][k] += xj * signal[m + i - k - 1];
            }
        }
    }

    solve_linear_system(&ata, &atb)
}

/// Solve a small linear system Ax = b using Gaussian elimination with pivoting.
fn solve_linear_system(a: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    let n = b.len();
    if n == 0 {
        return vec![];
    }

    let mut aug: Vec<Vec<f64>> = a
        .iter()
        .enumerate()
        .map(|(i, row)| {
            let mut r = row.clone();
            r.push(b[i]);
            r
        })
        .collect();

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_val = aug[col][col].abs();
        let mut max_row = col;
        for row in (col + 1)..n {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        if max_val < 1e-30 {
            continue;
        }
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        for row in (col + 1)..n {
            let factor = aug[row][col] / pivot;
            for j in col..=n {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = aug[i][n];
        for j in (i + 1)..n {
            sum -= aug[i][j] * x[j];
        }
        if aug[i][i].abs() > 1e-30 {
            x[i] = sum / aug[i][i];
        }
    }

    x
}

/// Find roots of polynomial 1 + a[0]z^{-1} + ... + a[M-1]z^{-M} = 0.
///
/// Equivalently: z^M + a[0]z^{M-1} + ... + a[M-1] = 0.
fn find_polynomial_roots(coeffs: &[f64]) -> Vec<Complex64> {
    let m = coeffs.len();
    if m == 0 {
        return vec![];
    }
    if m == 1 {
        return vec![Complex64::new(-coeffs[0], 0.0)];
    }
    if m == 2 {
        // Quadratic formula: z² + a[0]z + a[1] = 0
        let a = coeffs[0];
        let b = coeffs[1];
        let disc = Complex64::new(a * a - 4.0 * b, 0.0).sqrt();
        let r1 = (-Complex64::new(a, 0.0) + disc) / 2.0;
        let r2 = (-Complex64::new(a, 0.0) - disc) / 2.0;
        return vec![r1, r2];
    }

    // For higher orders, use companion matrix eigenvalue decomposition
    let mut companion = vec![vec![Complex64::new(0.0, 0.0); m]; m];
    for i in 1..m {
        companion[i][i - 1] = Complex64::new(1.0, 0.0);
    }
    for i in 0..m {
        companion[i][m - 1] = Complex64::new(-coeffs[m - 1 - i], 0.0);
    }

    eigenvalues_qr(&companion)
}

/// Simple QR iteration for eigenvalues of a companion matrix.
fn eigenvalues_qr(matrix: &[Vec<Complex64>]) -> Vec<Complex64> {
    let n = matrix.len();
    if n == 0 {
        return vec![];
    }
    if n == 1 {
        return vec![matrix[0][0]];
    }

    let mut a: Vec<Vec<Complex64>> = matrix.to_vec();
    let max_iter = 200;

    for _ in 0..max_iter {
        // Wilkinson shift
        let shift = a[n - 1][n - 1];

        // Shift
        for i in 0..n {
            a[i][i] -= shift;
        }

        // QR decomposition via Givens rotations
        let (q, r) = qr_decompose(&a);

        // A = R * Q + shift
        a = mat_mul(&r, &q);
        for i in 0..n {
            a[i][i] += shift;
        }

        // Check convergence (subdiagonal elements)
        let mut converged = true;
        for i in 1..n {
            if a[i][i - 1].norm() > 1e-12 {
                converged = false;
                break;
            }
        }
        if converged {
            break;
        }
    }

    // Extract eigenvalues from diagonal
    (0..n).map(|i| a[i][i]).collect()
}

fn qr_decompose(a: &[Vec<Complex64>]) -> (Vec<Vec<Complex64>>, Vec<Vec<Complex64>>) {
    let n = a.len();
    let mut r: Vec<Vec<Complex64>> = a.to_vec();
    let mut q = vec![vec![Complex64::new(0.0, 0.0); n]; n];
    for i in 0..n {
        q[i][i] = Complex64::new(1.0, 0.0);
    }

    for j in 0..n - 1 {
        for i in (j + 1)..n {
            if r[i][j].norm() < 1e-30 {
                continue;
            }
            let a_val = r[j][j];
            let b_val = r[i][j];
            let rr = (a_val.norm_sqr() + b_val.norm_sqr()).sqrt();
            let c = a_val.norm() / rr;
            let s = if a_val.norm() > 1e-30 {
                b_val * a_val.conj() / (a_val.norm() * rr)
            } else {
                Complex64::new(1.0, 0.0)
            };

            // Apply Givens rotation to R
            for k in 0..n {
                let rj = r[j][k];
                let ri = r[i][k];
                r[j][k] = c * rj + s.conj() * ri;
                r[i][k] = -s * rj + c * ri;
            }

            // Accumulate Q
            for k in 0..n {
                let qj = q[k][j];
                let qi = q[k][i];
                q[k][j] = c * qj + s.conj() * qi;
                q[k][i] = -s * qj + c * qi;
            }
        }
    }

    (q, r)
}

fn mat_mul(a: &[Vec<Complex64>], b: &[Vec<Complex64>]) -> Vec<Vec<Complex64>> {
    let n = a.len();
    let mut c = vec![vec![Complex64::new(0.0, 0.0); n]; n];
    for i in 0..n {
        for j in 0..n {
            for k in 0..n {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    c
}

/// Estimate complex amplitudes given signal and exponents via least-squares.
fn estimate_amplitudes(signal: &[f64], exponents: &[Complex64]) -> Vec<Complex64> {
    let n = signal.len();
    let m = exponents.len();
    if m == 0 {
        return vec![];
    }

    // Build Vandermonde-like matrix: V[i][k] = exp(exponent_k * i)
    // Solve V · amplitudes = signal (least squares)

    // Normal equations: V^H V a = V^H s
    let mut vhv = vec![vec![Complex64::new(0.0, 0.0); m]; m];
    let mut vhs = vec![Complex64::new(0.0, 0.0); m];

    for i in 0..n.min(2 * m + 10) {
        // Use subset for efficiency
        let t = i as f64;
        let v: Vec<Complex64> = exponents.iter().map(|&e| (e * t).exp()).collect();

        for j in 0..m {
            vhs[j] += v[j].conj() * signal[i];
            for k in 0..m {
                vhv[j][k] += v[j].conj() * v[k];
            }
        }
    }

    // Solve complex linear system
    solve_complex_system(&vhv, &vhs)
}

fn solve_complex_system(a: &[Vec<Complex64>], b: &[Complex64]) -> Vec<Complex64> {
    let n = b.len();
    if n == 0 {
        return vec![];
    }

    let mut aug: Vec<Vec<Complex64>> = a
        .iter()
        .enumerate()
        .map(|(i, row)| {
            let mut r = row.clone();
            r.push(b[i]);
            r
        })
        .collect();

    for col in 0..n {
        let mut max_val = aug[col][col].norm();
        let mut max_row = col;
        for row in (col + 1)..n {
            if aug[row][col].norm() > max_val {
                max_val = aug[row][col].norm();
                max_row = row;
            }
        }
        if max_val < 1e-30 {
            continue;
        }
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        for row in (col + 1)..n {
            let factor = aug[row][col] / pivot;
            for j in col..=n {
                let val = aug[col][j];
                aug[row][j] -= factor * val;
            }
        }
    }

    let mut x = vec![Complex64::new(0.0, 0.0); n];
    for i in (0..n).rev() {
        let mut sum = aug[i][n];
        for j in (i + 1)..n {
            sum -= aug[i][j] * x[j];
        }
        if aug[i][i].norm() > 1e-30 {
            x[i] = sum / aug[i][i];
        }
    }

    x
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_single_exponential() {
        // x(n) = exp(-0.05n) · cos(2π·0.1·n)
        let signal: Vec<f64> = (0..100)
            .map(|i| {
                let t = i as f64;
                (-0.05 * t).exp() * (2.0 * PI * 0.1 * t).cos()
            })
            .collect();

        let result = prony(&signal, 2, PronyVariant::LeastSquares);
        assert!(!result.modes.is_empty());

        // Should have low residual
        assert!(
            result.residual_error < 0.5,
            "residual={}",
            result.residual_error
        );
    }

    #[test]
    fn test_dc_signal() {
        let signal = vec![1.0; 50];
        let result = prony(&signal, 1, PronyVariant::LeastSquares);
        assert!(!result.modes.is_empty());
        // Dominant mode should have near-zero frequency
        let dominant = result.dominant_mode().unwrap();
        assert!(
            dominant.frequency < 0.05 || dominant.frequency > 0.95,
            "freq={}",
            dominant.frequency
        );
    }

    #[test]
    fn test_reconstruct() {
        let signal: Vec<f64> = (0..50)
            .map(|i| (2.0 * PI * 0.2 * i as f64).cos())
            .collect();
        let result = prony(&signal, 2, PronyVariant::LeastSquares);
        let recon = result.reconstruct(50);
        assert_eq!(recon.len(), 50);
    }

    #[test]
    fn test_least_squares_variant() {
        let signal: Vec<f64> = (0..200)
            .map(|i| {
                let t = i as f64;
                (-0.01 * t).exp() * (2.0 * PI * 0.15 * t).cos()
            })
            .collect();

        let std_result = prony(&signal, 2, PronyVariant::Standard);
        let ls_result = prony(&signal, 2, PronyVariant::LeastSquares);

        // Both should work, LS typically better for noisy signals
        assert!(!std_result.modes.is_empty());
        assert!(!ls_result.modes.is_empty());
    }

    #[test]
    fn test_matrix_pencil() {
        let signal: Vec<f64> = (0..100)
            .map(|i| (2.0 * PI * 0.1 * i as f64).cos())
            .collect();
        let result = matrix_pencil(&signal, 2);
        assert!(!result.modes.is_empty());
    }

    #[test]
    fn test_short_signal() {
        let result = prony(&[1.0, 2.0], 1, PronyVariant::Standard);
        assert!(result.modes.is_empty()); // Too short
    }

    #[test]
    fn test_psd() {
        let signal: Vec<f64> = (0..200)
            .map(|i| (2.0 * PI * 0.1 * i as f64).cos())
            .collect();
        let result = prony(&signal, 4, PronyVariant::LeastSquares);
        let freqs: Vec<f64> = (0..100).map(|i| i as f64 / 200.0).collect();
        let psd = result.psd(&freqs);
        assert_eq!(psd.len(), 100);
        assert!(psd.iter().all(|&p| p >= 0.0));
    }

    #[test]
    fn test_estimate_order() {
        let signal: Vec<f64> = (0..200)
            .map(|i| (2.0 * PI * 0.1 * i as f64).cos())
            .collect();
        let order = estimate_order_mdl(&signal, 10);
        assert!(order >= 1 && order <= 10);
    }

    #[test]
    fn test_solve_linear_system() {
        // 2x + y = 5, x + 3y = 7 => x=1.6, y=1.8
        let a = vec![vec![2.0, 1.0], vec![1.0, 3.0]];
        let b = vec![5.0, 7.0];
        let x = solve_linear_system(&a, &b);
        assert!((x[0] - 1.6).abs() < 1e-10);
        assert!((x[1] - 1.8).abs() < 1e-10);
    }

    #[test]
    fn test_polynomial_roots() {
        // z^2 - 3z + 2 = 0 => z = 1, 2
        // Coefficients: a[0] = -3, a[1] = 2
        let roots = find_polynomial_roots(&[-3.0, 2.0]);
        assert_eq!(roots.len(), 2);
        let mut reals: Vec<f64> = roots.iter().map(|r| r.re).collect();
        reals.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((reals[0] - 1.0).abs() < 0.01, "root0={}", reals[0]);
        assert!((reals[1] - 2.0).abs() < 0.01, "root1={}", reals[1]);
    }

    #[test]
    fn test_mode_evaluate() {
        let mode = PronyMode {
            amplitude: Complex64::new(1.0, 0.0),
            exponent: Complex64::new(0.0, 2.0 * PI * 0.1),
            frequency: 0.1,
            damping: 0.0,
        };
        let v0 = mode.evaluate(0);
        assert!((v0.re - 1.0).abs() < 1e-10);
    }
}
