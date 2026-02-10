//! Extended Prony method for modal decomposition of signals into damped sinusoids.
//!
//! The Prony method fits a signal as a sum of complex exponentials:
//!
//! ```text
//! x[n] = sum_{k=1}^{p} A_k * exp(j * phi_k) * exp((alpha_k + j * 2*pi*f_k) * n * dt)
//! ```
//!
//! where each mode has amplitude `A_k`, phase `phi_k`, damping factor `alpha_k`,
//! and frequency `f_k`.
//!
//! # Example
//!
//! ```
//! use r4w_core::modal_analysis_prony_extended::{PronyAnalyzer, ModeSortKey};
//!
//! // Create a simple test signal: two damped sinusoids
//! let fs = 1000.0;
//! let dt = 1.0 / fs;
//! let n = 128;
//! let signal: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 * dt;
//!         let s1 = (-5.0 * t).exp() * (2.0 * std::f64::consts::PI * 50.0 * t).cos();
//!         let s2 = 0.5 * (-10.0 * t).exp() * (2.0 * std::f64::consts::PI * 120.0 * t).cos();
//!         (s1 + s2, 0.0)
//!     })
//!     .collect();
//!
//! let analyzer = PronyAnalyzer::new(4, fs);
//! let result = analyzer.analyze(&signal);
//! assert!(result.is_ok());
//! let modes = result.unwrap();
//! assert!(!modes.modes.is_empty());
//! ```

use std::f64::consts::PI;

// ── Complex arithmetic helpers using (f64, f64) tuples ──────────────────────

/// Complex number type alias.
pub type Complex = (f64, f64);

fn c_add(a: Complex, b: Complex) -> Complex {
    (a.0 + b.0, a.1 + b.1)
}

fn c_sub(a: Complex, b: Complex) -> Complex {
    (a.0 - b.0, a.1 - b.1)
}

fn c_mul(a: Complex, b: Complex) -> Complex {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

fn c_div(a: Complex, b: Complex) -> Complex {
    let denom = b.0 * b.0 + b.1 * b.1;
    if denom < 1e-300 {
        return (0.0, 0.0);
    }
    ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
}

fn c_abs(a: Complex) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

fn c_arg(a: Complex) -> f64 {
    a.1.atan2(a.0)
}

fn c_conj(a: Complex) -> Complex {
    (a.0, -a.1)
}

fn c_scale(s: f64, a: Complex) -> Complex {
    (s * a.0, s * a.1)
}

fn c_from_polar(r: f64, theta: f64) -> Complex {
    (r * theta.cos(), r * theta.sin())
}

/// Complex square root.
fn c_sqrt(z: Complex) -> Complex {
    let r = c_abs(z);
    if r < 1e-300 {
        return (0.0, 0.0);
    }
    let theta = c_arg(z);
    c_from_polar(r.sqrt(), theta / 2.0)
}

// ── Public types ────────────────────────────────────────────────────────────

/// A single extracted mode (damped sinusoidal component).
#[derive(Debug, Clone)]
pub struct Mode {
    /// Frequency in Hz.
    pub frequency: f64,
    /// Damping factor (negative means decaying, Neper/s).
    pub damping: f64,
    /// Amplitude (linear scale).
    pub amplitude: f64,
    /// Phase in radians.
    pub phase: f64,
    /// Complex pole (z-domain).
    pub pole: Complex,
    /// Complex residue.
    pub residue: Complex,
}

/// Result of Prony analysis containing all extracted modes.
#[derive(Debug, Clone)]
pub struct PronyResult {
    /// Extracted modes sorted by default (frequency ascending).
    pub modes: Vec<Mode>,
    /// Residual error energy (sum of |error|^2).
    pub residual_energy: f64,
    /// Normalized residual (residual_energy / signal_energy).
    pub normalized_residual: f64,
}

/// Sort key for modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeSortKey {
    /// Sort by frequency (ascending absolute value).
    Frequency,
    /// Sort by amplitude (descending).
    Amplitude,
    /// Sort by damping (least damped first, i.e., closest to zero).
    Damping,
}

/// Singular value info for model order estimation.
#[derive(Debug, Clone)]
pub struct ModelOrderEstimate {
    /// Singular values of the data matrix (descending).
    pub singular_values: Vec<f64>,
    /// Ratios between consecutive singular values.
    pub ratios: Vec<f64>,
    /// Estimated model order.
    pub estimated_order: usize,
}

/// Mode tracking entry linking modes across time windows.
#[derive(Debug, Clone)]
pub struct TrackedMode {
    /// Frequency trajectory across windows.
    pub frequencies: Vec<f64>,
    /// Damping trajectory across windows.
    pub dampings: Vec<f64>,
    /// Amplitude trajectory across windows.
    pub amplitudes: Vec<f64>,
    /// Phase trajectory across windows.
    pub phases: Vec<f64>,
    /// Window indices where this mode was present.
    pub window_indices: Vec<usize>,
}

/// Configuration for the Prony analyzer.
#[derive(Debug, Clone)]
pub struct PronyAnalyzer {
    /// Model order (number of exponential terms).
    model_order: usize,
    /// Sample rate in Hz.
    sample_rate: f64,
}

impl PronyAnalyzer {
    /// Create a new PronyAnalyzer.
    ///
    /// # Arguments
    /// * `model_order` - Number of complex exponentials to fit (p).
    /// * `sample_rate` - Sample rate in Hz.
    pub fn new(model_order: usize, sample_rate: f64) -> Self {
        Self {
            model_order,
            sample_rate,
        }
    }

    /// Get the model order.
    pub fn model_order(&self) -> usize {
        self.model_order
    }

    /// Get the sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Set the model order.
    pub fn set_model_order(&mut self, order: usize) {
        self.model_order = order;
    }

    /// Perform Prony analysis on a complex signal.
    ///
    /// Returns the extracted modes or an error message.
    pub fn analyze(&self, signal: &[Complex]) -> Result<PronyResult, String> {
        let p = self.model_order;
        let n = signal.len();

        if n < 2 * p + 1 {
            return Err(format!(
                "Signal length {} too short for model order {}; need at least {}",
                n,
                p,
                2 * p + 1
            ));
        }
        if p == 0 {
            return Err("Model order must be at least 1".to_string());
        }

        // Step 1: Compute linear prediction coefficients
        let lp_coeffs = self.linear_prediction(signal, p)?;

        // Step 2: Find roots of the characteristic polynomial
        let roots = self.find_polynomial_roots(&lp_coeffs)?;

        // Step 3: Estimate residues (amplitudes and phases)
        let modes = self.estimate_modes(signal, &roots)?;

        // Step 4: Compute residual
        let reconstructed = self.reconstruct(signal.len(), &modes);
        let mut residual_energy = 0.0;
        let mut signal_energy = 0.0;
        for i in 0..n {
            let err = c_sub(signal[i], reconstructed[i]);
            residual_energy += err.0 * err.0 + err.1 * err.1;
            signal_energy += signal[i].0 * signal[i].0 + signal[i].1 * signal[i].1;
        }
        let normalized_residual = if signal_energy > 1e-300 {
            residual_energy / signal_energy
        } else {
            0.0
        };

        Ok(PronyResult {
            modes,
            residual_energy,
            normalized_residual,
        })
    }

    /// Compute linear prediction coefficients using the extended Prony method.
    ///
    /// Solves the Toeplitz-like system via normal equations.
    pub fn linear_prediction(
        &self,
        signal: &[Complex],
        order: usize,
    ) -> Result<Vec<Complex>, String> {
        let n = signal.len();
        if n < 2 * order + 1 {
            return Err("Insufficient signal length for linear prediction".to_string());
        }

        // Build the overdetermined system using forward linear prediction.
        // x[n] + a1*x[n-1] + ... + ap*x[n-p] = 0  for n = p..N-1
        // Solve via normal equations: A^H A coeffs = A^H b
        let num_eqs = n - order;

        // Compute A^H * A  (order x order) and A^H * b  (order)
        let mut aha = vec![(0.0, 0.0); order * order];
        let mut ahb = vec![(0.0, 0.0); order];

        for row in 0..num_eqs {
            let n_idx = row + order;
            let b_val = c_scale(-1.0, signal[n_idx]);

            for col_i in 0..order {
                let a_i = signal[n_idx - 1 - col_i];
                ahb[col_i] = c_add(ahb[col_i], c_mul(c_conj(a_i), b_val));

                for col_j in 0..order {
                    let a_j = signal[n_idx - 1 - col_j];
                    aha[col_i * order + col_j] =
                        c_add(aha[col_i * order + col_j], c_mul(c_conj(a_i), a_j));
                }
            }
        }

        self.solve_complex_system(&mut aha, &mut ahb, order)
    }

    /// Solve a complex linear system Ax = b via Gaussian elimination with partial pivoting.
    fn solve_complex_system(
        &self,
        a: &mut [Complex],
        b: &mut [Complex],
        n: usize,
    ) -> Result<Vec<Complex>, String> {
        for col in 0..n {
            // Find pivot
            let mut max_val = 0.0;
            let mut max_row = col;
            for row in col..n {
                let mag = c_abs(a[row * n + col]);
                if mag > max_val {
                    max_val = mag;
                    max_row = row;
                }
            }
            if max_val < 1e-15 {
                return Err("Singular matrix in linear prediction".to_string());
            }

            // Swap rows
            if max_row != col {
                for j in 0..n {
                    a.swap(col * n + j, max_row * n + j);
                }
                b.swap(col, max_row);
            }

            // Eliminate
            let pivot = a[col * n + col];
            for row in (col + 1)..n {
                let factor = c_div(a[row * n + col], pivot);
                for j in col..n {
                    let tmp = c_mul(factor, a[col * n + j]);
                    a[row * n + j] = c_sub(a[row * n + j], tmp);
                }
                let tmp = c_mul(factor, b[col]);
                b[row] = c_sub(b[row], tmp);
            }
        }

        // Back substitution
        let mut x = vec![(0.0, 0.0); n];
        for i in (0..n).rev() {
            let mut sum = b[i];
            for j in (i + 1)..n {
                sum = c_sub(sum, c_mul(a[i * n + j], x[j]));
            }
            x[i] = c_div(sum, a[i * n + i]);
        }

        Ok(x)
    }

    /// Find roots of the characteristic polynomial using companion matrix eigenvalues.
    ///
    /// The polynomial is: z^p + a1*z^(p-1) + ... + ap = 0
    pub fn find_polynomial_roots(&self, coeffs: &[Complex]) -> Result<Vec<Complex>, String> {
        let p = coeffs.len();
        if p == 0 {
            return Err("Empty coefficient vector".to_string());
        }
        if p == 1 {
            return Ok(vec![c_scale(-1.0, coeffs[0])]);
        }

        // Build companion matrix (p x p)
        let mut mat = vec![(0.0, 0.0); p * p];
        for j in 0..p {
            mat[j] = c_scale(-1.0, coeffs[j]);
        }
        for i in 1..p {
            mat[i * p + (i - 1)] = (1.0, 0.0);
        }

        self.qr_eigenvalues(&mut mat, p)
    }

    /// QR iteration for eigenvalues of a complex matrix.
    fn qr_eigenvalues(&self, mat: &mut [Complex], n: usize) -> Result<Vec<Complex>, String> {
        if n == 0 {
            return Ok(vec![]);
        }
        if n == 1 {
            return Ok(vec![mat[0]]);
        }

        self.to_hessenberg(mat, n);

        let max_iter = 200 * n;
        let mut eigenvalues = Vec::with_capacity(n);
        let mut size = n;
        let mut h = mat.to_vec();

        let mut iter_count = 0;
        while size > 1 && iter_count < max_iter {
            iter_count += 1;

            let sub = c_abs(h[(size - 1) * n + (size - 2)]);
            let diag_sum =
                c_abs(h[(size - 2) * n + (size - 2)]) + c_abs(h[(size - 1) * n + (size - 1)]);
            let threshold = 1e-14 * diag_sum.max(1e-300);

            if sub <= threshold {
                eigenvalues.push(h[(size - 1) * n + (size - 1)]);
                size -= 1;
                continue;
            }

            // Wilkinson shift
            let shift = self.wilkinson_shift(
                h[(size - 2) * n + (size - 2)],
                h[(size - 2) * n + (size - 1)],
                h[(size - 1) * n + (size - 2)],
                h[(size - 1) * n + (size - 1)],
            );

            for i in 0..size {
                h[i * n + i] = c_sub(h[i * n + i], shift);
            }

            // QR step via Givens rotations
            let mut cs = vec![0.0_f64; size - 1];
            let mut sn = vec![(0.0, 0.0); size - 1];

            for i in 0..(size - 1) {
                let a_val = h[i * n + i];
                let b_val = h[(i + 1) * n + i];
                let (c, s, _r) = self.givens_rotation(a_val, b_val);
                cs[i] = c;
                sn[i] = s;

                for j in i..size {
                    let t1 = h[i * n + j];
                    let t2 = h[(i + 1) * n + j];
                    h[i * n + j] = c_add(c_scale(c, t1), c_mul(c_conj(s), t2));
                    h[(i + 1) * n + j] = c_sub(c_scale(c, t2), c_mul(s, t1));
                }
            }

            for i in 0..(size - 1) {
                let c = cs[i];
                let s = sn[i];
                let limit = (i + 2).min(size);
                for j in 0..limit {
                    let t1 = h[j * n + i];
                    let t2 = h[j * n + (i + 1)];
                    h[j * n + i] = c_add(c_scale(c, t1), c_mul(s, t2));
                    h[j * n + (i + 1)] = c_sub(c_scale(c, t2), c_mul(c_conj(s), t1));
                }
            }

            for i in 0..size {
                h[i * n + i] = c_add(h[i * n + i], shift);
            }
        }

        if size == 1 {
            eigenvalues.push(h[0]);
        } else if size > 1 {
            for i in 0..size {
                eigenvalues.push(h[i * n + i]);
            }
        }

        Ok(eigenvalues)
    }

    /// Reduce matrix to upper Hessenberg form using Householder reflections.
    fn to_hessenberg(&self, mat: &mut [Complex], n: usize) {
        for col in 0..(n.saturating_sub(2)) {
            let m = n - col - 1;
            let mut v = vec![(0.0, 0.0); m];
            for i in 0..m {
                v[i] = mat[(col + 1 + i) * n + col];
            }

            let alpha = {
                let norm: f64 = v.iter().map(|x| x.0 * x.0 + x.1 * x.1).sum::<f64>().sqrt();
                if norm < 1e-300 {
                    continue;
                }
                let phase = c_arg(v[0]);
                c_from_polar(-norm, phase)
            };

            v[0] = c_sub(v[0], alpha);
            let v_norm: f64 = v.iter().map(|x| x.0 * x.0 + x.1 * x.1).sum::<f64>().sqrt();
            if v_norm < 1e-300 {
                continue;
            }
            for vi in v.iter_mut() {
                *vi = c_scale(1.0 / v_norm, *vi);
            }

            // Apply H = I - 2*v*v^H from left
            for j in 0..n {
                let mut dot = (0.0, 0.0);
                for i in 0..m {
                    dot = c_add(dot, c_mul(c_conj(v[i]), mat[(col + 1 + i) * n + j]));
                }
                for i in 0..m {
                    let tmp = c_scale(2.0, c_mul(v[i], dot));
                    mat[(col + 1 + i) * n + j] = c_sub(mat[(col + 1 + i) * n + j], tmp);
                }
            }

            // Apply from right
            for j in 0..n {
                let mut dot = (0.0, 0.0);
                for i in 0..m {
                    dot = c_add(dot, c_mul(mat[j * n + (col + 1 + i)], v[i]));
                }
                for i in 0..m {
                    let tmp = c_scale(2.0, c_mul(dot, c_conj(v[i])));
                    mat[j * n + (col + 1 + i)] = c_sub(mat[j * n + (col + 1 + i)], tmp);
                }
            }
        }
    }

    /// Compute Wilkinson shift from 2x2 submatrix.
    fn wilkinson_shift(&self, a: Complex, b: Complex, c: Complex, d: Complex) -> Complex {
        let trace = c_add(a, d);
        let det = c_sub(c_mul(a, d), c_mul(b, c));
        let disc = c_sub(c_mul(trace, trace), c_scale(4.0, det));
        let sqrt_disc = c_sqrt(disc);
        let e1 = c_scale(0.5, c_add(trace, sqrt_disc));
        let e2 = c_scale(0.5, c_sub(trace, sqrt_disc));

        if c_abs(c_sub(e1, d)) < c_abs(c_sub(e2, d)) {
            e1
        } else {
            e2
        }
    }

    /// Givens rotation to zero out b given (a, b).
    fn givens_rotation(&self, a: Complex, b: Complex) -> (f64, Complex, Complex) {
        let abs_a = c_abs(a);
        let abs_b = c_abs(b);

        if abs_b < 1e-300 {
            return (1.0, (0.0, 0.0), a);
        }
        if abs_a < 1e-300 {
            return (0.0, c_from_polar(1.0, c_arg(b)), (abs_b, 0.0));
        }

        let norm = (abs_a * abs_a + abs_b * abs_b).sqrt();
        let c = abs_a / norm;
        let phase_a = c_from_polar(1.0, c_arg(a));
        let s = c_mul(phase_a, c_conj(b));
        let s = c_scale(1.0 / norm, s);
        let r = c_scale(norm, phase_a);
        (c, s, r)
    }

    /// Estimate mode parameters from poles and signal data.
    fn estimate_modes(&self, signal: &[Complex], poles: &[Complex]) -> Result<Vec<Mode>, String> {
        let p = poles.len();
        let n = signal.len();
        let dt = 1.0 / self.sample_rate;

        // Least-squares via Vandermonde: V * residues = signal
        // Normal equations: V^H V residues = V^H signal
        let use_n = n.min(4 * p + p);

        let mut vhv = vec![(0.0, 0.0); p * p];
        let mut vhs = vec![(0.0, 0.0); p];

        for k in 0..use_n {
            // Compute pole_powers via repeated multiplication
            let mut pole_powers = vec![(1.0, 0.0); p];
            for j in 0..p {
                let mut pk = (1.0, 0.0);
                for _ in 0..k {
                    pk = c_mul(pk, poles[j]);
                }
                pole_powers[j] = pk;
            }

            for i in 0..p {
                vhs[i] = c_add(vhs[i], c_mul(c_conj(pole_powers[i]), signal[k]));
                for j in 0..p {
                    vhv[i * p + j] = c_add(
                        vhv[i * p + j],
                        c_mul(c_conj(pole_powers[i]), pole_powers[j]),
                    );
                }
            }
        }

        let residues = self.solve_complex_system(&mut vhv, &mut vhs, p)?;

        let mut modes = Vec::with_capacity(p);
        for i in 0..p {
            let z = poles[i];
            let abs_z = c_abs(z);
            let angle_z = c_arg(z);

            let damping = if abs_z > 1e-300 {
                abs_z.ln() / dt
            } else {
                -1e10
            };
            let frequency = angle_z / (2.0 * PI * dt);
            let amplitude = c_abs(residues[i]);
            let phase = c_arg(residues[i]);

            modes.push(Mode {
                frequency,
                damping,
                amplitude,
                phase,
                pole: z,
                residue: residues[i],
            });
        }

        Ok(modes)
    }

    /// Reconstruct a signal from modes.
    pub fn reconstruct(&self, num_samples: usize, modes: &[Mode]) -> Vec<Complex> {
        let mut signal = vec![(0.0, 0.0); num_samples];
        for mode in modes {
            let mut pk = (1.0, 0.0); // pole^0
            for k in 0..num_samples {
                signal[k] = c_add(signal[k], c_mul(mode.residue, pk));
                pk = c_mul(pk, mode.pole);
            }
        }
        signal
    }

    /// Compute residual error between original signal and reconstruction.
    pub fn residual_error(&self, signal: &[Complex], modes: &[Mode]) -> f64 {
        let recon = self.reconstruct(signal.len(), modes);
        let mut err = 0.0;
        for i in 0..signal.len() {
            let d = c_sub(signal[i], recon[i]);
            err += d.0 * d.0 + d.1 * d.1;
        }
        err
    }

    /// Sort modes by the given key.
    pub fn sort_modes(modes: &mut [Mode], key: ModeSortKey) {
        match key {
            ModeSortKey::Frequency => {
                modes.sort_by(|a, b| {
                    a.frequency
                        .abs()
                        .partial_cmp(&b.frequency.abs())
                        .unwrap_or(std::cmp::Ordering::Equal)
                });
            }
            ModeSortKey::Amplitude => {
                modes.sort_by(|a, b| {
                    b.amplitude
                        .partial_cmp(&a.amplitude)
                        .unwrap_or(std::cmp::Ordering::Equal)
                });
            }
            ModeSortKey::Damping => {
                modes.sort_by(|a, b| {
                    a.damping
                        .abs()
                        .partial_cmp(&b.damping.abs())
                        .unwrap_or(std::cmp::Ordering::Equal)
                });
            }
        }
    }

    /// Estimate model order from singular value decay of the data matrix.
    ///
    /// Builds a Hankel matrix from the signal and computes approximate singular values
    /// using the power method. The estimated order is where the ratio between consecutive
    /// singular values exceeds a threshold.
    pub fn estimate_model_order(
        signal: &[Complex],
        max_order: usize,
        threshold: f64,
    ) -> ModelOrderEstimate {
        let n = signal.len();
        let m = max_order.min(n / 2);
        let rows = n - m;

        // Build H^H * H (m x m) from Hankel matrix
        let mut hhh = vec![(0.0, 0.0); m * m];
        for i in 0..m {
            for j in 0..m {
                let mut sum = (0.0, 0.0);
                for k in 0..rows {
                    sum = c_add(sum, c_mul(c_conj(signal[k + i]), signal[k + j]));
                }
                hhh[i * m + j] = sum;
            }
        }

        let svs = Self::approximate_singular_values(&hhh, m);

        let mut ratios = Vec::with_capacity(svs.len().saturating_sub(1));
        for i in 0..svs.len().saturating_sub(1) {
            if svs[i + 1] > 1e-300 {
                ratios.push(svs[i] / svs[i + 1]);
            } else {
                ratios.push(f64::INFINITY);
            }
        }

        let mut estimated_order = svs.len();
        for (i, &r) in ratios.iter().enumerate() {
            if r > threshold {
                estimated_order = i + 1;
                break;
            }
        }

        ModelOrderEstimate {
            singular_values: svs,
            ratios,
            estimated_order,
        }
    }

    /// Approximate singular values using deflated power iteration on the Gram matrix.
    fn approximate_singular_values(gram: &[Complex], n: usize) -> Vec<f64> {
        let mut svs = Vec::with_capacity(n);
        let mut work = gram.to_vec();

        for _ in 0..n {
            let mut v = vec![(0.0, 0.0); n];
            for (j, vj) in v.iter_mut().enumerate() {
                *vj = (((j * 7 + 3) % 11) as f64 - 5.0, ((j * 13 + 7) % 11) as f64 - 5.0);
            }

            let mut eigenvalue = 0.0;
            for _ in 0..50 {
                let mut w = vec![(0.0, 0.0); n];
                for ii in 0..n {
                    for jj in 0..n {
                        w[ii] = c_add(w[ii], c_mul(work[ii * n + jj], v[jj]));
                    }
                }

                let norm: f64 = w.iter().map(|x| x.0 * x.0 + x.1 * x.1).sum::<f64>().sqrt();
                if norm < 1e-300 {
                    break;
                }
                eigenvalue = norm;
                for wi in w.iter_mut() {
                    *wi = c_scale(1.0 / norm, *wi);
                }
                v = w;
            }

            svs.push(eigenvalue.sqrt());

            // Deflate
            for ii in 0..n {
                for jj in 0..n {
                    let outer = c_mul(v[ii], c_conj(v[jj]));
                    work[ii * n + jj] = c_sub(work[ii * n + jj], c_scale(eigenvalue, outer));
                }
            }
        }

        svs.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));
        svs
    }

    /// Track modes across time windows for time-varying analysis.
    ///
    /// Divides the signal into overlapping windows, performs Prony analysis on each,
    /// and links modes across windows by frequency proximity.
    pub fn track_modes(
        &self,
        signal: &[Complex],
        window_size: usize,
        hop_size: usize,
        freq_tolerance: f64,
    ) -> Result<Vec<TrackedMode>, String> {
        let n = signal.len();
        if window_size < 2 * self.model_order + 1 {
            return Err("Window size too small for model order".to_string());
        }
        if hop_size == 0 {
            return Err("Hop size must be positive".to_string());
        }

        let mut all_window_modes: Vec<Vec<Mode>> = Vec::new();
        let mut start = 0;
        while start + window_size <= n {
            let window = &signal[start..start + window_size];
            match self.analyze(window) {
                Ok(result) => all_window_modes.push(result.modes),
                Err(_) => all_window_modes.push(Vec::new()),
            }
            start += hop_size;
        }

        let mut tracks: Vec<TrackedMode> = Vec::new();

        for (win_idx, window_modes) in all_window_modes.iter().enumerate() {
            let mut used = vec![false; window_modes.len()];

            // Try to extend existing tracks
            for track in tracks.iter_mut() {
                if track.window_indices.last() == Some(&(win_idx.wrapping_sub(1))) {
                    let last_freq = *track.frequencies.last().unwrap();

                    let mut best_idx = None;
                    let mut best_dist = f64::INFINITY;
                    for (m_idx, mode) in window_modes.iter().enumerate() {
                        if !used[m_idx] {
                            let dist = (mode.frequency - last_freq).abs();
                            if dist < best_dist && dist < freq_tolerance {
                                best_dist = dist;
                                best_idx = Some(m_idx);
                            }
                        }
                    }

                    if let Some(idx) = best_idx {
                        track.frequencies.push(window_modes[idx].frequency);
                        track.dampings.push(window_modes[idx].damping);
                        track.amplitudes.push(window_modes[idx].amplitude);
                        track.phases.push(window_modes[idx].phase);
                        track.window_indices.push(win_idx);
                        used[idx] = true;
                    }
                }
            }

            // Start new tracks for unmatched modes
            for (m_idx, mode) in window_modes.iter().enumerate() {
                if !used[m_idx] {
                    tracks.push(TrackedMode {
                        frequencies: vec![mode.frequency],
                        dampings: vec![mode.damping],
                        amplitudes: vec![mode.amplitude],
                        phases: vec![mode.phase],
                        window_indices: vec![win_idx],
                    });
                }
            }
        }

        Ok(tracks)
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a damped sinusoid.
    fn damped_sinusoid(
        freq: f64,
        damping: f64,
        amplitude: f64,
        phase: f64,
        fs: f64,
        n: usize,
    ) -> Vec<Complex> {
        let dt = 1.0 / fs;
        (0..n)
            .map(|k| {
                let t = k as f64 * dt;
                let val =
                    amplitude * (damping * t).exp() * (2.0 * PI * freq * t + phase).cos();
                (val, 0.0)
            })
            .collect()
    }

    #[test]
    fn test_new_analyzer() {
        let analyzer = PronyAnalyzer::new(4, 1000.0);
        assert_eq!(analyzer.model_order(), 4);
        assert_eq!(analyzer.sample_rate(), 1000.0);
    }

    #[test]
    fn test_set_model_order() {
        let mut analyzer = PronyAnalyzer::new(4, 1000.0);
        analyzer.set_model_order(8);
        assert_eq!(analyzer.model_order(), 8);
    }

    #[test]
    fn test_insufficient_signal_length() {
        let analyzer = PronyAnalyzer::new(4, 1000.0);
        let signal = vec![(1.0, 0.0); 5];
        let result = analyzer.analyze(&signal);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("too short"));
    }

    #[test]
    fn test_zero_model_order() {
        let analyzer = PronyAnalyzer::new(0, 1000.0);
        let signal = vec![(1.0, 0.0); 100];
        let result = analyzer.analyze(&signal);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("at least 1"));
    }

    #[test]
    fn test_single_sinusoid() {
        let fs = 1000.0;
        let freq = 50.0;
        let n = 128;
        let signal = damped_sinusoid(freq, -2.0, 1.0, 0.0, fs, n);

        let analyzer = PronyAnalyzer::new(2, fs);
        let result = analyzer.analyze(&signal).unwrap();

        assert!(!result.modes.is_empty());
        let has_target = result
            .modes
            .iter()
            .any(|m| (m.frequency.abs() - freq).abs() < 10.0);
        assert!(
            has_target,
            "Expected mode near {} Hz, got: {:?}",
            freq,
            result.modes.iter().map(|m| m.frequency).collect::<Vec<_>>()
        );
    }

    #[test]
    fn test_two_sinusoids() {
        let fs = 1000.0;
        let n = 256;
        let dt = 1.0 / fs;
        let signal: Vec<Complex> = (0..n)
            .map(|k| {
                let t = k as f64 * dt;
                let s1 = (-3.0 * t).exp() * (2.0 * PI * 50.0 * t).cos();
                let s2 = 0.5 * (-8.0 * t).exp() * (2.0 * PI * 150.0 * t).cos();
                (s1 + s2, 0.0)
            })
            .collect();

        let analyzer = PronyAnalyzer::new(4, fs);
        let result = analyzer.analyze(&signal).unwrap();

        assert!(
            result.modes.len() >= 2,
            "Expected at least 2 modes, got {}",
            result.modes.len()
        );
    }

    #[test]
    fn test_reconstruction_accuracy() {
        let fs = 500.0;
        let n = 64;
        let signal = damped_sinusoid(25.0, -5.0, 1.0, 0.0, fs, n);

        let analyzer = PronyAnalyzer::new(2, fs);
        let result = analyzer.analyze(&signal).unwrap();
        let recon = analyzer.reconstruct(n, &result.modes);

        let mut max_err = 0.0_f64;
        for i in 0..n {
            let err = c_abs(c_sub(signal[i], recon[i]));
            max_err = max_err.max(err);
        }
        assert!(
            max_err < 1.0,
            "Max reconstruction error too large: {}",
            max_err
        );
    }

    #[test]
    fn test_residual_error() {
        let fs = 500.0;
        let n = 64;
        let signal = damped_sinusoid(30.0, -3.0, 1.0, 0.0, fs, n);

        let analyzer = PronyAnalyzer::new(2, fs);
        let result = analyzer.analyze(&signal).unwrap();
        let err = analyzer.residual_error(&signal, &result.modes);

        assert!(err >= 0.0);
        assert!(err.is_finite());
    }

    #[test]
    fn test_normalized_residual() {
        let fs = 500.0;
        let n = 64;
        let signal = damped_sinusoid(30.0, -3.0, 1.0, 0.0, fs, n);

        let analyzer = PronyAnalyzer::new(2, fs);
        let result = analyzer.analyze(&signal).unwrap();

        assert!(result.normalized_residual >= 0.0);
        assert!(result.normalized_residual.is_finite());
    }

    #[test]
    fn test_sort_by_frequency() {
        let mut modes = vec![
            Mode {
                frequency: 100.0,
                damping: -1.0,
                amplitude: 0.5,
                phase: 0.0,
                pole: (0.9, 0.3),
                residue: (0.5, 0.0),
            },
            Mode {
                frequency: 20.0,
                damping: -2.0,
                amplitude: 1.0,
                phase: 0.0,
                pole: (0.9, 0.1),
                residue: (1.0, 0.0),
            },
            Mode {
                frequency: 60.0,
                damping: -0.5,
                amplitude: 0.3,
                phase: 0.0,
                pole: (0.95, 0.2),
                residue: (0.3, 0.0),
            },
        ];
        PronyAnalyzer::sort_modes(&mut modes, ModeSortKey::Frequency);
        assert!(modes[0].frequency.abs() <= modes[1].frequency.abs());
        assert!(modes[1].frequency.abs() <= modes[2].frequency.abs());
    }

    #[test]
    fn test_sort_by_amplitude() {
        let mut modes = vec![
            Mode {
                frequency: 100.0,
                damping: -1.0,
                amplitude: 0.5,
                phase: 0.0,
                pole: (0.9, 0.3),
                residue: (0.5, 0.0),
            },
            Mode {
                frequency: 20.0,
                damping: -2.0,
                amplitude: 1.0,
                phase: 0.0,
                pole: (0.9, 0.1),
                residue: (1.0, 0.0),
            },
            Mode {
                frequency: 60.0,
                damping: -0.5,
                amplitude: 0.3,
                phase: 0.0,
                pole: (0.95, 0.2),
                residue: (0.3, 0.0),
            },
        ];
        PronyAnalyzer::sort_modes(&mut modes, ModeSortKey::Amplitude);
        assert!(modes[0].amplitude >= modes[1].amplitude);
        assert!(modes[1].amplitude >= modes[2].amplitude);
    }

    #[test]
    fn test_sort_by_damping() {
        let mut modes = vec![
            Mode {
                frequency: 100.0,
                damping: -5.0,
                amplitude: 0.5,
                phase: 0.0,
                pole: (0.9, 0.3),
                residue: (0.5, 0.0),
            },
            Mode {
                frequency: 20.0,
                damping: -1.0,
                amplitude: 1.0,
                phase: 0.0,
                pole: (0.9, 0.1),
                residue: (1.0, 0.0),
            },
            Mode {
                frequency: 60.0,
                damping: -10.0,
                amplitude: 0.3,
                phase: 0.0,
                pole: (0.95, 0.2),
                residue: (0.3, 0.0),
            },
        ];
        PronyAnalyzer::sort_modes(&mut modes, ModeSortKey::Damping);
        assert!(modes[0].damping.abs() <= modes[1].damping.abs());
        assert!(modes[1].damping.abs() <= modes[2].damping.abs());
    }

    #[test]
    fn test_model_order_estimation() {
        let fs = 1000.0;
        let dt = 1.0 / fs;
        let n = 256;
        let signal: Vec<Complex> = (0..n)
            .map(|k| {
                let t = k as f64 * dt;
                let s = (2.0 * PI * 50.0 * t).cos() + 0.5 * (2.0 * PI * 120.0 * t).cos();
                (s, 0.0)
            })
            .collect();

        let estimate = PronyAnalyzer::estimate_model_order(&signal, 20, 10.0);
        assert!(!estimate.singular_values.is_empty());
        assert!(estimate.estimated_order >= 1);
        assert!(estimate.estimated_order <= 20);
    }

    #[test]
    fn test_singular_values_descending() {
        let fs = 1000.0;
        let dt = 1.0 / fs;
        let n = 128;
        let signal: Vec<Complex> = (0..n)
            .map(|k| {
                let t = k as f64 * dt;
                ((2.0 * PI * 50.0 * t).cos(), 0.0)
            })
            .collect();

        let estimate = PronyAnalyzer::estimate_model_order(&signal, 10, 10.0);
        for i in 0..estimate.singular_values.len().saturating_sub(1) {
            assert!(
                estimate.singular_values[i] >= estimate.singular_values[i + 1] - 1e-10,
                "Singular values not descending at index {}: {} < {}",
                i,
                estimate.singular_values[i],
                estimate.singular_values[i + 1]
            );
        }
    }

    #[test]
    fn test_complex_signal() {
        let fs = 1000.0;
        let dt = 1.0 / fs;
        let n = 128;
        let freq = 75.0;
        // A single complex exponential needs model order 1
        let signal: Vec<Complex> = (0..n)
            .map(|k| {
                let t = k as f64 * dt;
                let phase = 2.0 * PI * freq * t;
                let decay = (-2.0 * t).exp();
                (decay * phase.cos(), decay * phase.sin())
            })
            .collect();

        let analyzer = PronyAnalyzer::new(1, fs);
        let result = analyzer.analyze(&signal).unwrap();
        assert!(!result.modes.is_empty());

        let has_target = result
            .modes
            .iter()
            .any(|m| (m.frequency.abs() - freq).abs() < 15.0);
        assert!(
            has_target,
            "Expected mode near {} Hz, got: {:?}",
            freq,
            result.modes.iter().map(|m| m.frequency).collect::<Vec<_>>()
        );
    }

    #[test]
    fn test_mode_tracking_basic() {
        let fs = 1000.0;
        let dt = 1.0 / fs;
        let n = 512;
        let signal: Vec<Complex> = (0..n)
            .map(|k| {
                let t = k as f64 * dt;
                let s = (-1.0 * t).exp() * (2.0 * PI * 50.0 * t).cos();
                (s, 0.0)
            })
            .collect();

        let analyzer = PronyAnalyzer::new(2, fs);
        let tracks = analyzer.track_modes(&signal, 128, 64, 20.0).unwrap();

        assert!(
            !tracks.is_empty(),
            "Should produce at least one tracked mode"
        );
    }

    #[test]
    fn test_mode_tracking_window_too_small() {
        let analyzer = PronyAnalyzer::new(4, 1000.0);
        let signal = vec![(1.0, 0.0); 256];
        let result = analyzer.track_modes(&signal, 5, 2, 10.0);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Window size too small"));
    }

    #[test]
    fn test_mode_tracking_zero_hop() {
        let analyzer = PronyAnalyzer::new(2, 1000.0);
        let signal = vec![(1.0, 0.0); 256];
        let result = analyzer.track_modes(&signal, 64, 0, 10.0);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Hop size must be positive"));
    }

    #[test]
    fn test_linear_prediction_coefficients() {
        let fs = 1000.0;
        let n = 64;
        let signal = damped_sinusoid(50.0, -2.0, 1.0, 0.0, fs, n);

        let analyzer = PronyAnalyzer::new(2, fs);
        let coeffs = analyzer.linear_prediction(&signal, 2).unwrap();

        assert_eq!(coeffs.len(), 2);
        for c in &coeffs {
            assert!(c.0.is_finite() && c.1.is_finite());
        }
    }

    #[test]
    fn test_polynomial_root_finding_linear() {
        let analyzer = PronyAnalyzer::new(1, 1000.0);
        let coeffs = vec![(-0.5, 0.0)];
        let roots = analyzer.find_polynomial_roots(&coeffs).unwrap();
        assert_eq!(roots.len(), 1);
        assert!(
            (roots[0].0 - 0.5).abs() < 1e-10,
            "Expected root ~0.5, got {:?}",
            roots[0]
        );
        assert!(roots[0].1.abs() < 1e-10);
    }

    #[test]
    fn test_polynomial_root_finding_quadratic() {
        let analyzer = PronyAnalyzer::new(2, 1000.0);
        let coeffs = vec![(0.0, 0.0), (-1.0, 0.0)];
        let roots = analyzer.find_polynomial_roots(&coeffs).unwrap();
        assert_eq!(roots.len(), 2);

        let mut root_reals: Vec<f64> = roots.iter().map(|r| r.0).collect();
        root_reals.sort_by(|a, b| a.partial_cmp(b).unwrap());

        assert!(
            (root_reals[0] - (-1.0)).abs() < 0.1,
            "Expected root ~-1, got {}",
            root_reals[0]
        );
        assert!(
            (root_reals[1] - 1.0).abs() < 0.1,
            "Expected root ~1, got {}",
            root_reals[1]
        );
    }

    #[test]
    fn test_reconstruct_empty_modes() {
        let analyzer = PronyAnalyzer::new(2, 1000.0);
        let recon = analyzer.reconstruct(10, &[]);
        assert_eq!(recon.len(), 10);
        for s in &recon {
            assert_eq!(s.0, 0.0);
            assert_eq!(s.1, 0.0);
        }
    }

    #[test]
    fn test_mode_struct_fields() {
        let mode = Mode {
            frequency: 100.0,
            damping: -5.0,
            amplitude: 1.5,
            phase: 0.3,
            pole: (0.9, 0.4),
            residue: (1.2, -0.3),
        };
        assert_eq!(mode.frequency, 100.0);
        assert_eq!(mode.damping, -5.0);
        assert_eq!(mode.amplitude, 1.5);
        assert!((mode.phase - 0.3).abs() < 1e-15);
    }

    #[test]
    fn test_prony_result_fields() {
        let fs = 500.0;
        let n = 64;
        let signal = damped_sinusoid(30.0, -3.0, 1.0, 0.0, fs, n);

        let analyzer = PronyAnalyzer::new(2, fs);
        let result = analyzer.analyze(&signal).unwrap();

        assert!(result.residual_energy >= 0.0);
        assert!(result.normalized_residual >= 0.0);
        assert!(result.residual_energy.is_finite());
        assert!(result.normalized_residual.is_finite());
    }

    #[test]
    fn test_complex_arithmetic_helpers() {
        assert_eq!(c_add((1.0, 2.0), (3.0, 4.0)), (4.0, 6.0));
        assert_eq!(c_sub((5.0, 3.0), (2.0, 1.0)), (3.0, 2.0));
        assert_eq!(c_mul((1.0, 0.0), (0.0, 1.0)), (0.0, 1.0));
        assert!((c_abs((3.0, 4.0)) - 5.0).abs() < 1e-12);
        assert_eq!(c_conj((1.0, 2.0)), (1.0, -2.0));

        let d = c_div((1.0, 0.0), (1.0, 0.0));
        assert!((d.0 - 1.0).abs() < 1e-12);
        assert!(d.1.abs() < 1e-12);
    }

    #[test]
    fn test_c_sqrt() {
        let r = c_sqrt((4.0, 0.0));
        assert!((r.0 - 2.0).abs() < 1e-12);
        assert!(r.1.abs() < 1e-12);

        let r = c_sqrt((-1.0, 0.0));
        assert!(r.0.abs() < 1e-12);
        assert!((r.1.abs() - 1.0).abs() < 1e-12);

        let r = c_sqrt((0.0, 0.0));
        assert!(r.0.abs() < 1e-12);
        assert!(r.1.abs() < 1e-12);
    }

    #[test]
    fn test_pure_dc_signal() {
        let fs = 1000.0;
        let n = 64;
        let signal: Vec<Complex> = vec![(1.0, 0.0); n];

        let analyzer = PronyAnalyzer::new(1, fs);
        let result = analyzer.analyze(&signal).unwrap();

        assert!(!result.modes.is_empty());
        let min_freq = result
            .modes
            .iter()
            .map(|m| m.frequency.abs())
            .fold(f64::INFINITY, f64::min);
        assert!(
            min_freq < 50.0,
            "Expected near-DC mode, min freq = {}",
            min_freq
        );
    }

    #[test]
    fn test_model_order_estimate_ratios() {
        let fs = 1000.0;
        let dt = 1.0 / fs;
        let n = 128;
        let signal: Vec<Complex> = (0..n)
            .map(|k| {
                let t = k as f64 * dt;
                ((2.0 * PI * 50.0 * t).cos(), 0.0)
            })
            .collect();

        let estimate = PronyAnalyzer::estimate_model_order(&signal, 10, 5.0);
        assert_eq!(
            estimate.ratios.len(),
            estimate.singular_values.len() - 1
        );
        for r in &estimate.ratios {
            assert!(*r >= 0.0 || r.is_infinite());
        }
    }
}
