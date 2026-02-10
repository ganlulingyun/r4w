//! Nonlinear system modeling using Volterra series expansion.
//!
//! This module provides a Volterra series filter for modeling nonlinear systems
//! such as power amplifiers (PAs) and RF components. The Volterra series
//! generalizes the linear convolution (FIR filter) to include nonlinear terms
//! with memory effects.
//!
//! The output of a Volterra filter up to 3rd order is:
//!
//! ```text
//! y(n) = Σ h1(m1) * x(n-m1)
//!      + Σ h2(m1,m2) * x(n-m1) * x(n-m2)
//!      + Σ h3(m1,m2,m3) * x(n-m1) * x(n-m2) * x(n-m3)
//! ```
//!
//! For practical PA modeling, a pruned Volterra (memory polynomial) form is
//! often preferred, which retains only diagonal terms:
//!
//! ```text
//! y(n) = Σ_p Σ_m a(p,m) * x(n-m) * |x(n-m)|^(p-1)
//! ```
//!
//! # Example
//!
//! ```
//! use r4w_core::volterra_filter::{VolterraFilter, VolterraOrder};
//!
//! // Create a 3rd-order Volterra filter with memory depth 3
//! let mut filter = VolterraFilter::new(VolterraOrder::Third, 3);
//!
//! // Set 1st-order (linear) kernel coefficients
//! filter.set_kernel1(&[1.0, 0.1, -0.05]);
//!
//! // Set 3rd-order diagonal kernel for cubic nonlinearity
//! filter.set_kernel3_diagonal(&[-0.1, 0.02, 0.0]);
//!
//! // Process a real-valued signal
//! let input = vec![0.5, 0.8, 1.0, 0.7, 0.3];
//! let output = filter.process_real(&input);
//! assert_eq!(output.len(), input.len());
//! ```

use std::f64::consts::PI;

/// Complex number represented as (real, imaginary) tuple.
pub type Cplx = (f64, f64);

/// Maximum supported Volterra order.
const MAX_ORDER: usize = 5;

/// Volterra series order configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VolterraOrder {
    /// Linear only (1st order) - equivalent to FIR filter.
    First,
    /// Up to 2nd order (linear + quadratic).
    Second,
    /// Up to 3rd order (linear + quadratic + cubic).
    Third,
    /// Up to 5th order.
    Fifth,
}

impl VolterraOrder {
    /// Returns the numeric order value.
    pub fn value(&self) -> usize {
        match self {
            VolterraOrder::First => 1,
            VolterraOrder::Second => 2,
            VolterraOrder::Third => 3,
            VolterraOrder::Fifth => 5,
        }
    }
}

// ---------------------------------------------------------------------------
// Complex arithmetic helpers (using (f64, f64) tuples)
// ---------------------------------------------------------------------------

fn cx_add(a: Cplx, b: Cplx) -> Cplx {
    (a.0 + b.0, a.1 + b.1)
}

fn cx_sub(a: Cplx, b: Cplx) -> Cplx {
    (a.0 - b.0, a.1 - b.1)
}

fn cx_mul(a: Cplx, b: Cplx) -> Cplx {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

fn cx_conj(a: Cplx) -> Cplx {
    (a.0, -a.1)
}

fn cx_abs_sq(a: Cplx) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

fn cx_abs(a: Cplx) -> f64 {
    cx_abs_sq(a).sqrt()
}

fn cx_scale(a: Cplx, s: f64) -> Cplx {
    (a.0 * s, a.1 * s)
}

fn cx_phase(a: Cplx) -> f64 {
    a.1.atan2(a.0)
}

/// Complex multiplicative inverse: 1/z.
fn cx_inv(z: Cplx) -> Cplx {
    let denom = cx_abs_sq(z);
    if denom < 1e-60 {
        return (0.0, 0.0);
    }
    (z.0 / denom, -z.1 / denom)
}

/// Volterra series filter for nonlinear system modeling.
///
/// Supports 1st through 5th order kernels with configurable memory depth.
/// Also provides a pruned "memory polynomial" mode for efficient PA modeling.
#[derive(Debug, Clone)]
pub struct VolterraFilter {
    order: VolterraOrder,
    memory_depth: usize,
    /// 1st-order kernel h1[m], length = memory_depth
    kernel1: Vec<f64>,
    /// 2nd-order kernel h2[m1][m2], size = memory_depth x memory_depth
    kernel2: Vec<Vec<f64>>,
    /// 3rd-order diagonal kernel h3_diag[m], length = memory_depth
    kernel3_diag: Vec<f64>,
    /// 3rd-order full kernel h3[m1][m2][m3] (optional, only if set)
    kernel3_full: Option<Vec<Vec<Vec<f64>>>>,
    /// 5th-order diagonal kernel h5_diag[m], length = memory_depth
    kernel5_diag: Vec<f64>,
}

impl VolterraFilter {
    /// Create a new Volterra filter with given order and memory depth.
    ///
    /// All kernels are initialized to zero.
    pub fn new(order: VolterraOrder, memory_depth: usize) -> Self {
        assert!(memory_depth > 0, "Memory depth must be at least 1");
        let m = memory_depth;
        Self {
            order,
            memory_depth: m,
            kernel1: vec![0.0; m],
            kernel2: vec![vec![0.0; m]; m],
            kernel3_diag: vec![0.0; m],
            kernel3_full: None,
            kernel5_diag: vec![0.0; m],
        }
    }

    /// Returns the filter order.
    pub fn order(&self) -> VolterraOrder {
        self.order
    }

    /// Returns the memory depth.
    pub fn memory_depth(&self) -> usize {
        self.memory_depth
    }

    /// Set the 1st-order (linear FIR) kernel coefficients.
    ///
    /// The slice length must equal the memory depth.
    pub fn set_kernel1(&mut self, coeffs: &[f64]) {
        assert_eq!(
            coeffs.len(),
            self.memory_depth,
            "Kernel1 length must equal memory depth"
        );
        self.kernel1 = coeffs.to_vec();
    }

    /// Set the 2nd-order kernel as a full matrix.
    ///
    /// `coeffs` is a memory_depth x memory_depth matrix stored row-major.
    pub fn set_kernel2(&mut self, coeffs: &[Vec<f64>]) {
        assert_eq!(coeffs.len(), self.memory_depth);
        for row in coeffs {
            assert_eq!(row.len(), self.memory_depth);
        }
        self.kernel2 = coeffs.to_vec();
    }

    /// Set the 2nd-order kernel diagonal only (h2[m,m]).
    pub fn set_kernel2_diagonal(&mut self, coeffs: &[f64]) {
        assert_eq!(coeffs.len(), self.memory_depth);
        for i in 0..self.memory_depth {
            self.kernel2[i][i] = coeffs[i];
        }
    }

    /// Set the 3rd-order diagonal kernel h3[m,m,m].
    pub fn set_kernel3_diagonal(&mut self, coeffs: &[f64]) {
        assert_eq!(
            coeffs.len(),
            self.memory_depth,
            "Kernel3 diagonal length must equal memory depth"
        );
        self.kernel3_diag = coeffs.to_vec();
    }

    /// Set the full 3rd-order kernel (expensive, use diagonal for most cases).
    pub fn set_kernel3_full(&mut self, coeffs: Vec<Vec<Vec<f64>>>) {
        let m = self.memory_depth;
        assert_eq!(coeffs.len(), m);
        for plane in &coeffs {
            assert_eq!(plane.len(), m);
            for row in plane {
                assert_eq!(row.len(), m);
            }
        }
        self.kernel3_full = Some(coeffs);
    }

    /// Set the 5th-order diagonal kernel h5[m,m,m,m,m].
    pub fn set_kernel5_diagonal(&mut self, coeffs: &[f64]) {
        assert_eq!(coeffs.len(), self.memory_depth);
        self.kernel5_diag = coeffs.to_vec();
    }

    /// Process a real-valued input signal through the Volterra filter.
    ///
    /// Returns the output signal of the same length as the input.
    pub fn process_real(&self, input: &[f64]) -> Vec<f64> {
        let n = input.len();
        let m = self.memory_depth;
        let mut output = vec![0.0; n];

        for i in 0..n {
            let mut y = 0.0;

            // 1st-order (linear) term
            for k in 0..m {
                if i >= k {
                    y += self.kernel1[k] * input[i - k];
                }
            }

            // 2nd-order term
            if self.order.value() >= 2 {
                for m1 in 0..m {
                    if i < m1 {
                        continue;
                    }
                    for m2 in 0..m {
                        if i < m2 {
                            continue;
                        }
                        y += self.kernel2[m1][m2] * input[i - m1] * input[i - m2];
                    }
                }
            }

            // 3rd-order term
            if self.order.value() >= 3 {
                if let Some(ref k3) = self.kernel3_full {
                    // Full 3rd-order kernel
                    for m1 in 0..m {
                        if i < m1 {
                            continue;
                        }
                        for m2 in 0..m {
                            if i < m2 {
                                continue;
                            }
                            for m3 in 0..m {
                                if i < m3 {
                                    continue;
                                }
                                y += k3[m1][m2][m3]
                                    * input[i - m1]
                                    * input[i - m2]
                                    * input[i - m3];
                            }
                        }
                    }
                } else {
                    // Diagonal kernel only
                    for k in 0..m {
                        if i >= k {
                            let x = input[i - k];
                            y += self.kernel3_diag[k] * x * x * x;
                        }
                    }
                }
            }

            // 5th-order diagonal term
            if self.order.value() >= 5 {
                for k in 0..m {
                    if i >= k {
                        let x = input[i - k];
                        y += self.kernel5_diag[k] * x.powi(5);
                    }
                }
            }

            output[i] = y;
        }

        output
    }

    /// Process a complex-valued input signal through the Volterra filter.
    ///
    /// Uses the baseband Volterra model appropriate for PA modeling:
    /// - 1st order: linear FIR (kernel1 as real gains)
    /// - 3rd order diagonal: x(n-m) * |x(n-m)|^2 (AM/AM and AM/PM)
    /// - 5th order diagonal: x(n-m) * |x(n-m)|^4
    pub fn process_complex(&self, input: &[Cplx]) -> Vec<Cplx> {
        let n = input.len();
        let m = self.memory_depth;
        let mut output = vec![(0.0, 0.0); n];

        for i in 0..n {
            let mut y: Cplx = (0.0, 0.0);

            // 1st-order
            for k in 0..m {
                if i >= k {
                    y = cx_add(y, cx_scale(input[i - k], self.kernel1[k]));
                }
            }

            // 3rd-order baseband: h3[k] * x(n-k) * |x(n-k)|^2
            if self.order.value() >= 3 {
                for k in 0..m {
                    if i >= k {
                        let x = input[i - k];
                        let mag_sq = cx_abs_sq(x);
                        y = cx_add(y, cx_scale(x, self.kernel3_diag[k] * mag_sq));
                    }
                }
            }

            // 5th-order baseband: h5[k] * x(n-k) * |x(n-k)|^4
            if self.order.value() >= 5 {
                for k in 0..m {
                    if i >= k {
                        let x = input[i - k];
                        let mag_sq = cx_abs_sq(x);
                        y = cx_add(y, cx_scale(x, self.kernel5_diag[k] * mag_sq * mag_sq));
                    }
                }
            }

            output[i] = y;
        }

        output
    }

    /// Total number of parameters in the filter.
    pub fn num_parameters(&self) -> usize {
        let m = self.memory_depth;
        let mut count = m; // kernel1
        if self.order.value() >= 2 {
            count += m * m; // kernel2
        }
        if self.order.value() >= 3 {
            if self.kernel3_full.is_some() {
                count += m * m * m;
            } else {
                count += m; // diagonal only
            }
        }
        if self.order.value() >= 5 {
            count += m; // 5th-order diagonal
        }
        count
    }
}

// ---------------------------------------------------------------------------
// Memory Polynomial (Pruned Volterra)
// ---------------------------------------------------------------------------

/// Memory polynomial model for efficient PA modeling.
///
/// Uses the pruned Volterra form:
/// `y(n) = Σ_p Σ_m a(p,m) * x(n-m) * |x(n-m)|^(p-1)`
///
/// where p is the nonlinearity order (odd only: 1, 3, 5, ...) and
/// m is the memory tap index.
#[derive(Debug, Clone)]
pub struct MemoryPolynomial {
    /// Maximum nonlinearity order (odd, e.g. 5 means orders 1,3,5).
    max_order: usize,
    /// Memory depth (number of taps per order).
    memory_depth: usize,
    /// Coefficients indexed as [order_index][memory_index].
    /// order_index 0 -> order 1, 1 -> order 3, 2 -> order 5, etc.
    coeffs: Vec<Vec<Cplx>>,
}

impl MemoryPolynomial {
    /// Create a new memory polynomial with given max order and memory depth.
    ///
    /// `max_order` must be odd and >= 1.
    pub fn new(max_order: usize, memory_depth: usize) -> Self {
        assert!(max_order >= 1, "Max order must be >= 1");
        assert!(max_order % 2 == 1, "Max order must be odd");
        assert!(memory_depth >= 1, "Memory depth must be >= 1");
        let num_orders = (max_order + 1) / 2;
        let coeffs = vec![vec![(0.0, 0.0); memory_depth]; num_orders];
        Self {
            max_order,
            memory_depth,
            coeffs,
        }
    }

    /// Number of nonlinearity orders (e.g. 3 for orders 1,3,5).
    pub fn num_orders(&self) -> usize {
        (self.max_order + 1) / 2
    }

    /// Total number of complex coefficients.
    pub fn num_coefficients(&self) -> usize {
        self.num_orders() * self.memory_depth
    }

    /// Set coefficient for a given nonlinearity order and memory tap.
    ///
    /// `order` must be odd (1, 3, 5, ...) and <= max_order.
    pub fn set_coeff(&mut self, order: usize, memory_tap: usize, value: Cplx) {
        assert!(order >= 1 && order % 2 == 1 && order <= self.max_order);
        assert!(memory_tap < self.memory_depth);
        let idx = (order - 1) / 2;
        self.coeffs[idx][memory_tap] = value;
    }

    /// Get coefficient for a given nonlinearity order and memory tap.
    pub fn get_coeff(&self, order: usize, memory_tap: usize) -> Cplx {
        assert!(order >= 1 && order % 2 == 1 && order <= self.max_order);
        assert!(memory_tap < self.memory_depth);
        let idx = (order - 1) / 2;
        self.coeffs[idx][memory_tap]
    }

    /// Set all coefficients from a flat vector of complex values.
    ///
    /// Layout: [order1_tap0, order1_tap1, ..., order3_tap0, order3_tap1, ...]
    pub fn set_coefficients_flat(&mut self, flat: &[Cplx]) {
        assert_eq!(flat.len(), self.num_coefficients());
        let mut idx = 0;
        for oi in 0..self.num_orders() {
            for mi in 0..self.memory_depth {
                self.coeffs[oi][mi] = flat[idx];
                idx += 1;
            }
        }
    }

    /// Process a complex-valued input signal.
    pub fn process(&self, input: &[Cplx]) -> Vec<Cplx> {
        let n = input.len();
        let mut output = vec![(0.0, 0.0); n];

        for i in 0..n {
            let mut y: Cplx = (0.0, 0.0);

            for oi in 0..self.num_orders() {
                let _p = 2 * oi + 1; // nonlinearity order: 1, 3, 5, ...
                for mi in 0..self.memory_depth {
                    if i >= mi {
                        let x = input[i - mi];
                        let mag_sq = cx_abs_sq(x);
                        // x * |x|^(p-1) = x * (|x|^2)^((p-1)/2)
                        let mag_pow = mag_sq.powi(oi as i32); // (|x|^2)^((p-1)/2)
                        let basis = cx_scale(x, mag_pow);
                        y = cx_add(y, cx_mul(self.coeffs[oi][mi], basis));
                    }
                }
            }

            output[i] = y;
        }

        output
    }

    /// Process a real-valued input signal (treats input as real-only complex).
    pub fn process_real(&self, input: &[f64]) -> Vec<f64> {
        let cx_input: Vec<Cplx> = input.iter().map(|&x| (x, 0.0)).collect();
        let cx_output = self.process(&cx_input);
        cx_output.iter().map(|c| c.0).collect()
    }
}

// ---------------------------------------------------------------------------
// Kernel Identification (Least Squares)
// ---------------------------------------------------------------------------

/// Identifies memory polynomial coefficients from input-output data using
/// least squares estimation.
///
/// Builds the basis matrix Phi where each column corresponds to
/// `x(n-m) * |x(n-m)|^(p-1)` and solves `y = Phi * a` for coefficients `a`.
pub fn identify_memory_polynomial(
    input: &[Cplx],
    output: &[Cplx],
    max_order: usize,
    memory_depth: usize,
) -> MemoryPolynomial {
    assert_eq!(input.len(), output.len());
    assert!(max_order >= 1 && max_order % 2 == 1);
    let n = input.len();
    let num_orders = (max_order + 1) / 2;
    let num_coeffs = num_orders * memory_depth;

    // Build basis matrix Phi (n x num_coeffs) and output vector
    // We solve using normal equations: a = (Phi^H Phi)^{-1} Phi^H y

    // Phi^H Phi (num_coeffs x num_coeffs)
    let mut phi_h_phi = vec![vec![(0.0, 0.0); num_coeffs]; num_coeffs];
    // Phi^H y (num_coeffs x 1)
    let mut phi_h_y = vec![(0.0, 0.0); num_coeffs];

    // Compute basis vectors for each sample
    for i in 0..n {
        let mut basis = vec![(0.0, 0.0); num_coeffs];
        let mut col = 0;
        for oi in 0..num_orders {
            for mi in 0..memory_depth {
                if i >= mi {
                    let x = input[i - mi];
                    let mag_sq = cx_abs_sq(x);
                    let mag_pow = mag_sq.powi(oi as i32);
                    basis[col] = cx_scale(x, mag_pow);
                }
                col += 1;
            }
        }

        // Accumulate Phi^H Phi
        for r in 0..num_coeffs {
            for c in 0..num_coeffs {
                phi_h_phi[r][c] = cx_add(phi_h_phi[r][c], cx_mul(cx_conj(basis[r]), basis[c]));
            }
            // Accumulate Phi^H y
            phi_h_y[r] = cx_add(phi_h_y[r], cx_mul(cx_conj(basis[r]), output[i]));
        }
    }

    // Solve via Gaussian elimination with partial pivoting (complex)
    let coeffs_flat = solve_complex_linear_system(&phi_h_phi, &phi_h_y);

    let mut mp = MemoryPolynomial::new(max_order, memory_depth);
    mp.set_coefficients_flat(&coeffs_flat);
    mp
}

/// Solve a complex linear system Ax = b using Gaussian elimination with partial pivoting.
fn solve_complex_linear_system(a: &[Vec<Cplx>], b: &[Cplx]) -> Vec<Cplx> {
    let n = b.len();
    if n == 0 {
        return vec![];
    }
    // Build augmented matrix
    let mut aug: Vec<Vec<Cplx>> = Vec::with_capacity(n);
    for i in 0..n {
        let mut row = a[i].clone();
        row.push(b[i]);
        aug.push(row);
    }

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_mag = cx_abs_sq(aug[col][col]);
        let mut max_row = col;
        for row in (col + 1)..n {
            let mag = cx_abs_sq(aug[row][col]);
            if mag > max_mag {
                max_mag = mag;
                max_row = row;
            }
        }

        if max_mag < 1e-30 {
            // Singular or near-singular: return zeros
            return vec![(0.0, 0.0); n];
        }

        // Swap rows
        if max_row != col {
            aug.swap(col, max_row);
        }

        // Eliminate below
        let pivot = aug[col][col];
        for row in (col + 1)..n {
            let factor = cx_mul(aug[row][col], cx_inv(pivot));
            aug[row][col] = (0.0, 0.0);
            for j in (col + 1)..=n {
                let sub = cx_mul(factor, aug[col][j]);
                aug[row][j] = cx_sub(aug[row][j], sub);
            }
        }
    }

    // Back substitution
    let mut x = vec![(0.0, 0.0); n];
    for i in (0..n).rev() {
        let mut sum = aug[i][n];
        for j in (i + 1)..n {
            sum = cx_sub(sum, cx_mul(aug[i][j], x[j]));
        }
        x[i] = cx_mul(sum, cx_inv(aug[i][i]));
    }

    x
}

// ---------------------------------------------------------------------------
// Metrics: NMSE, AM/AM, AM/PM
// ---------------------------------------------------------------------------

/// Compute the Normalized Mean Square Error (NMSE) in dB between
/// a reference signal and a test signal.
///
/// NMSE = 10 * log10( sum|ref - test|^2 / sum|ref|^2 )
///
/// Lower is better. A perfect match returns negative infinity (clamped to -100 dB).
pub fn nmse_db(reference: &[Cplx], test: &[Cplx]) -> f64 {
    assert_eq!(reference.len(), test.len());
    let mut err_pow = 0.0;
    let mut ref_pow = 0.0;
    for i in 0..reference.len() {
        let diff = cx_sub(reference[i], test[i]);
        err_pow += cx_abs_sq(diff);
        ref_pow += cx_abs_sq(reference[i]);
    }
    if ref_pow < 1e-60 {
        return 0.0;
    }
    let ratio = err_pow / ref_pow;
    if ratio < 1e-10 {
        -100.0
    } else {
        10.0 * ratio.log10()
    }
}

/// Compute NMSE for real-valued signals.
pub fn nmse_db_real(reference: &[f64], test: &[f64]) -> f64 {
    assert_eq!(reference.len(), test.len());
    let ref_cx: Vec<Cplx> = reference.iter().map(|&x| (x, 0.0)).collect();
    let test_cx: Vec<Cplx> = test.iter().map(|&x| (x, 0.0)).collect();
    nmse_db(&ref_cx, &test_cx)
}

/// AM/AM and AM/PM distortion extraction.
///
/// Given input and output complex samples, computes:
/// - AM/AM: output amplitude vs input amplitude
/// - AM/PM: output phase shift vs input amplitude
///
/// Returns a vector of `(input_amplitude, output_amplitude, phase_shift_degrees)`.
pub fn extract_am_am_pm(input: &[Cplx], output: &[Cplx]) -> Vec<(f64, f64, f64)> {
    assert_eq!(input.len(), output.len());
    let mut result = Vec::with_capacity(input.len());

    for i in 0..input.len() {
        let in_amp = cx_abs(input[i]);
        let out_amp = cx_abs(output[i]);

        // Phase shift = phase(output) - phase(input)
        let phase_shift = if in_amp > 1e-12 {
            let diff = cx_mul(output[i], cx_inv(input[i]));
            cx_phase(diff) * 180.0 / PI
        } else {
            0.0
        };

        result.push((in_amp, out_amp, phase_shift));
    }

    result
}

/// Compute AM/AM characteristic binned by input amplitude.
///
/// Returns `(bin_centers, avg_output_amplitude)` for `num_bins` amplitude bins.
pub fn am_am_curve(
    input: &[Cplx],
    output: &[Cplx],
    num_bins: usize,
) -> (Vec<f64>, Vec<f64>) {
    let raw = extract_am_am_pm(input, output);
    let max_amp = raw.iter().map(|r| r.0).fold(0.0_f64, f64::max);
    if max_amp < 1e-12 || num_bins == 0 {
        return (vec![], vec![]);
    }

    let bin_width = max_amp / num_bins as f64;
    let mut bin_sum = vec![0.0; num_bins];
    let mut bin_count = vec![0usize; num_bins];

    for (in_amp, out_amp, _) in &raw {
        let bin = ((*in_amp / bin_width) as usize).min(num_bins - 1);
        bin_sum[bin] += out_amp;
        bin_count[bin] += 1;
    }

    let centers: Vec<f64> = (0..num_bins)
        .map(|i| (i as f64 + 0.5) * bin_width)
        .collect();
    let averages: Vec<f64> = (0..num_bins)
        .map(|i| {
            if bin_count[i] > 0 {
                bin_sum[i] / bin_count[i] as f64
            } else {
                0.0
            }
        })
        .collect();

    (centers, averages)
}

/// Compute AM/PM characteristic binned by input amplitude.
///
/// Returns `(bin_centers, avg_phase_shift_degrees)` for `num_bins` amplitude bins.
pub fn am_pm_curve(
    input: &[Cplx],
    output: &[Cplx],
    num_bins: usize,
) -> (Vec<f64>, Vec<f64>) {
    let raw = extract_am_am_pm(input, output);
    let max_amp = raw.iter().map(|r| r.0).fold(0.0_f64, f64::max);
    if max_amp < 1e-12 || num_bins == 0 {
        return (vec![], vec![]);
    }

    let bin_width = max_amp / num_bins as f64;
    let mut bin_sum = vec![0.0; num_bins];
    let mut bin_count = vec![0usize; num_bins];

    for (in_amp, _, phase) in &raw {
        let bin = ((*in_amp / bin_width) as usize).min(num_bins - 1);
        bin_sum[bin] += phase;
        bin_count[bin] += 1;
    }

    let centers: Vec<f64> = (0..num_bins)
        .map(|i| (i as f64 + 0.5) * bin_width)
        .collect();
    let averages: Vec<f64> = (0..num_bins)
        .map(|i| {
            if bin_count[i] > 0 {
                bin_sum[i] / bin_count[i] as f64
            } else {
                0.0
            }
        })
        .collect();

    (centers, averages)
}

// ---------------------------------------------------------------------------
// Utility: simple PA model for testing
// ---------------------------------------------------------------------------

/// Rapp solid-state PA model (memoryless nonlinearity for testing).
///
/// `g(x) = G * x / (1 + |G*x|^(2p) / A_sat^(2p))^(1/(2p))`
///
/// - `gain`: small-signal gain G
/// - `saturation`: output saturation amplitude A_sat
/// - `smoothness`: Rapp smoothness parameter p (higher = sharper compression)
pub fn rapp_pa_model(
    input: &[Cplx],
    gain: f64,
    saturation: f64,
    smoothness: f64,
) -> Vec<Cplx> {
    let p2 = 2.0 * smoothness;
    let a_sat_2p = saturation.powf(p2);
    input
        .iter()
        .map(|&x| {
            let gx = cx_scale(x, gain);
            let gx_amp = cx_abs(gx);
            let denom = (1.0 + (gx_amp.powf(p2) / a_sat_2p)).powf(1.0 / p2);
            cx_scale(gx, 1.0 / denom)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-10;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn cx_approx_eq(a: Cplx, b: Cplx, tol: f64) -> bool {
        approx_eq(a.0, b.0, tol) && approx_eq(a.1, b.1, tol)
    }

    // --- VolterraOrder tests ---

    #[test]
    fn test_volterra_order_values() {
        assert_eq!(VolterraOrder::First.value(), 1);
        assert_eq!(VolterraOrder::Second.value(), 2);
        assert_eq!(VolterraOrder::Third.value(), 3);
        assert_eq!(VolterraOrder::Fifth.value(), 5);
    }

    // --- VolterraFilter construction ---

    #[test]
    fn test_filter_construction() {
        let f = VolterraFilter::new(VolterraOrder::Third, 4);
        assert_eq!(f.order(), VolterraOrder::Third);
        assert_eq!(f.memory_depth(), 4);
        // All kernels should be zero
        assert!(f.kernel1.iter().all(|&x| x == 0.0));
        assert!(f.kernel3_diag.iter().all(|&x| x == 0.0));
    }

    #[test]
    #[should_panic(expected = "Memory depth must be at least 1")]
    fn test_filter_zero_memory_panics() {
        let _ = VolterraFilter::new(VolterraOrder::First, 0);
    }

    // --- Linear (1st order) processing ---

    #[test]
    fn test_linear_fir_processing() {
        // A 1st-order Volterra filter is just a FIR filter
        let mut f = VolterraFilter::new(VolterraOrder::First, 3);
        f.set_kernel1(&[1.0, 0.5, 0.25]);

        let input = vec![1.0, 0.0, 0.0, 0.0, 0.0];
        let output = f.process_real(&input);

        // Impulse response should match kernel
        assert!(approx_eq(output[0], 1.0, TOL));
        assert!(approx_eq(output[1], 0.5, TOL));
        assert!(approx_eq(output[2], 0.25, TOL));
        assert!(approx_eq(output[3], 0.0, TOL));
        assert!(approx_eq(output[4], 0.0, TOL));
    }

    #[test]
    fn test_linear_convolution() {
        let mut f = VolterraFilter::new(VolterraOrder::First, 2);
        f.set_kernel1(&[0.6, 0.4]);

        let input = vec![1.0, 2.0, 3.0, 4.0];
        let output = f.process_real(&input);

        // y[0] = 0.6*1.0 = 0.6
        // y[1] = 0.6*2.0 + 0.4*1.0 = 1.6
        // y[2] = 0.6*3.0 + 0.4*2.0 = 2.6
        // y[3] = 0.6*4.0 + 0.4*3.0 = 3.6
        assert!(approx_eq(output[0], 0.6, TOL));
        assert!(approx_eq(output[1], 1.6, TOL));
        assert!(approx_eq(output[2], 2.6, TOL));
        assert!(approx_eq(output[3], 3.6, TOL));
    }

    // --- 2nd order processing ---

    #[test]
    fn test_second_order_diagonal() {
        let mut f = VolterraFilter::new(VolterraOrder::Second, 2);
        f.set_kernel1(&[1.0, 0.0]);
        f.set_kernel2_diagonal(&[0.5, 0.0]);

        let input = vec![2.0, 3.0];
        let output = f.process_real(&input);

        // y[0] = 1.0*2.0 + 0.5*2.0*2.0 = 2.0 + 2.0 = 4.0
        assert!(approx_eq(output[0], 4.0, TOL));
        // y[1] = 1.0*3.0 + 0.5*3.0*3.0 = 3.0 + 4.5 = 7.5
        assert!(approx_eq(output[1], 7.5, TOL));
    }

    #[test]
    fn test_second_order_cross_products() {
        // Test 2nd-order kernel with cross-terms
        let mut f = VolterraFilter::new(VolterraOrder::Second, 2);
        f.set_kernel1(&[0.0, 0.0]); // no linear term
        let mut k2 = vec![vec![0.0; 2]; 2];
        k2[0][1] = 1.0; // h2(0,1) = 1.0, cross product x(n)*x(n-1)
        f.set_kernel2(&k2);

        let input = vec![2.0, 3.0, 4.0];
        let output = f.process_real(&input);

        // y[0] = 0 (no x(n-1) available with full depth)
        assert!(approx_eq(output[0], 0.0, TOL));
        // y[1] = h2[0][1]*x(1)*x(0) = 1.0*3.0*2.0 = 6.0
        assert!(approx_eq(output[1], 6.0, TOL));
        // y[2] = h2[0][1]*x(2)*x(1) = 1.0*4.0*3.0 = 12.0
        assert!(approx_eq(output[2], 12.0, TOL));
    }

    // --- 3rd order processing ---

    #[test]
    fn test_third_order_diagonal() {
        let mut f = VolterraFilter::new(VolterraOrder::Third, 2);
        f.set_kernel1(&[1.0, 0.0]);
        f.set_kernel3_diagonal(&[-0.1, 0.0]);

        let input = vec![2.0];
        let output = f.process_real(&input);

        // y[0] = 1.0*2.0 + (-0.1)*2.0^3 = 2.0 - 0.8 = 1.2
        assert!(approx_eq(output[0], 1.2, TOL));
    }

    #[test]
    fn test_third_order_full_kernel() {
        let mut f = VolterraFilter::new(VolterraOrder::Third, 1);
        f.set_kernel1(&[1.0]);
        let k3 = vec![vec![vec![0.5]]]; // h3[0][0][0] = 0.5
        f.set_kernel3_full(k3);

        let input = vec![3.0];
        let output = f.process_real(&input);

        // y[0] = 1.0*3.0 + 0.5*3.0^3 = 3.0 + 13.5 = 16.5
        assert!(approx_eq(output[0], 16.5, TOL));
    }

    // --- 5th order processing ---

    #[test]
    fn test_fifth_order_diagonal() {
        let mut f = VolterraFilter::new(VolterraOrder::Fifth, 1);
        f.set_kernel1(&[1.0]);
        f.set_kernel3_diagonal(&[0.0]);
        f.set_kernel5_diagonal(&[-0.01]);

        let input = vec![2.0];
        let output = f.process_real(&input);

        // y[0] = 1.0*2.0 + (-0.01)*2.0^5 = 2.0 - 0.32 = 1.68
        assert!(approx_eq(output[0], 1.68, TOL));
    }

    // --- Complex processing ---

    #[test]
    fn test_complex_linear_processing() {
        let mut f = VolterraFilter::new(VolterraOrder::First, 2);
        f.set_kernel1(&[1.0, 0.5]);

        let input: Vec<Cplx> = vec![(1.0, 1.0), (0.0, 0.0), (0.0, 0.0)];
        let output = f.process_complex(&input);

        // y[0] = 1.0*(1+j) = (1, 1)
        assert!(cx_approx_eq(output[0], (1.0, 1.0), TOL));
        // y[1] = 0.5*(1+j) = (0.5, 0.5)
        assert!(cx_approx_eq(output[1], (0.5, 0.5), TOL));
        // y[2] = 0
        assert!(cx_approx_eq(output[2], (0.0, 0.0), TOL));
    }

    #[test]
    fn test_complex_third_order() {
        // Test baseband PA model: h3 * x * |x|^2
        let mut f = VolterraFilter::new(VolterraOrder::Third, 1);
        f.set_kernel1(&[1.0]);
        f.set_kernel3_diagonal(&[-0.1]);

        let x: Cplx = (1.0, 1.0); // |x|^2 = 2
        let input = vec![x];
        let output = f.process_complex(&input);

        // y = 1.0*x + (-0.1)*x*|x|^2 = x - 0.1*x*2 = x*(1 - 0.2) = 0.8*x
        let expected = cx_scale(x, 0.8);
        assert!(cx_approx_eq(output[0], expected, TOL));
    }

    // --- Memory Polynomial ---

    #[test]
    fn test_memory_polynomial_construction() {
        let mp = MemoryPolynomial::new(5, 3);
        assert_eq!(mp.num_orders(), 3);
        assert_eq!(mp.num_coefficients(), 9);
    }

    #[test]
    #[should_panic(expected = "Max order must be odd")]
    fn test_memory_polynomial_even_order_panics() {
        let _ = MemoryPolynomial::new(4, 2);
    }

    #[test]
    fn test_memory_polynomial_linear() {
        let mut mp = MemoryPolynomial::new(1, 2);
        mp.set_coeff(1, 0, (1.0, 0.0));
        mp.set_coeff(1, 1, (0.5, 0.0));

        let input: Vec<Cplx> = vec![(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)];
        let output = mp.process(&input);

        // y[0] = 1.0 * x(0) = 1.0
        assert!(approx_eq(output[0].0, 1.0, TOL));
        // y[1] = 1.0 * x(1) + 0.5 * x(0) = 2.0 + 0.5 = 2.5
        assert!(approx_eq(output[1].0, 2.5, TOL));
        // y[2] = 1.0 * x(2) + 0.5 * x(1) = 3.0 + 1.0 = 4.0
        assert!(approx_eq(output[2].0, 4.0, TOL));
    }

    #[test]
    fn test_memory_polynomial_nonlinear() {
        let mut mp = MemoryPolynomial::new(3, 1);
        mp.set_coeff(1, 0, (1.0, 0.0));
        mp.set_coeff(3, 0, (-0.1, 0.0));

        // Real input x = 2.0
        let input: Vec<Cplx> = vec![(2.0, 0.0)];
        let output = mp.process(&input);

        // y = 1.0*2.0 + (-0.1)*2.0*|2.0|^2 = 2.0 - 0.8 = 1.2
        assert!(approx_eq(output[0].0, 1.2, TOL));
    }

    #[test]
    fn test_memory_polynomial_process_real() {
        let mut mp = MemoryPolynomial::new(3, 1);
        mp.set_coeff(1, 0, (1.0, 0.0));
        mp.set_coeff(3, 0, (-0.05, 0.0));

        let input = vec![1.0, 2.0, 3.0];
        let output = mp.process_real(&input);

        // y[0] = 1.0*1.0 - 0.05*1.0*1.0 = 0.95
        assert!(approx_eq(output[0], 0.95, TOL));
        // y[1] = 1.0*2.0 - 0.05*2.0*4.0 = 2.0 - 0.4 = 1.6
        assert!(approx_eq(output[1], 1.6, TOL));
        // y[2] = 1.0*3.0 - 0.05*3.0*9.0 = 3.0 - 1.35 = 1.65
        assert!(approx_eq(output[2], 1.65, TOL));
    }

    // --- Kernel Identification ---

    #[test]
    fn test_identify_linear_system() {
        // Create a simple linear system and identify it
        let mut mp = MemoryPolynomial::new(1, 2);
        mp.set_coeff(1, 0, (0.8, 0.0));
        mp.set_coeff(1, 1, (0.3, 0.0));

        // Generate test signal
        let n = 200;
        let input: Vec<Cplx> = (0..n)
            .map(|i| {
                let t = i as f64 / n as f64;
                ((2.0 * PI * 3.0 * t).cos(), (2.0 * PI * 3.0 * t).sin())
            })
            .collect();

        let output = mp.process(&input);

        // Identify
        let identified = identify_memory_polynomial(&input, &output, 1, 2);

        // Check that identified coefficients are close to original
        let c0 = identified.get_coeff(1, 0);
        let c1 = identified.get_coeff(1, 1);
        assert!(approx_eq(c0.0, 0.8, 1e-4), "c0.re = {}, expected 0.8", c0.0);
        assert!(approx_eq(c0.1, 0.0, 1e-4));
        assert!(approx_eq(c1.0, 0.3, 1e-4), "c1.re = {}, expected 0.3", c1.0);
        assert!(approx_eq(c1.1, 0.0, 1e-4));
    }

    #[test]
    fn test_identify_nonlinear_system() {
        // Create a nonlinear system and identify it
        let mut mp = MemoryPolynomial::new(3, 1);
        mp.set_coeff(1, 0, (1.0, 0.0));
        mp.set_coeff(3, 0, (-0.1, 0.0));

        // Generate multi-amplitude test signal for good conditioning
        let n = 500;
        let input: Vec<Cplx> = (0..n)
            .map(|i| {
                let t = i as f64 / n as f64;
                let amp = 0.5 + 1.5 * t; // varying amplitude
                let phase = 2.0 * PI * 5.0 * t;
                (amp * phase.cos(), amp * phase.sin())
            })
            .collect();

        let output = mp.process(&input);
        let identified = identify_memory_polynomial(&input, &output, 3, 1);

        let c1 = identified.get_coeff(1, 0);
        let c3 = identified.get_coeff(3, 0);
        assert!(
            approx_eq(c1.0, 1.0, 1e-3),
            "c1.re = {}, expected 1.0",
            c1.0
        );
        assert!(
            approx_eq(c3.0, -0.1, 1e-3),
            "c3.re = {}, expected -0.1",
            c3.0
        );
    }

    // --- NMSE ---

    #[test]
    fn test_nmse_perfect_match() {
        let sig: Vec<Cplx> = vec![(1.0, 0.5), (0.3, -0.7), (-0.5, 0.2)];
        let nmse = nmse_db(&sig, &sig);
        assert!(nmse <= -100.0, "Perfect match NMSE should be <= -100 dB, got {}", nmse);
    }

    #[test]
    fn test_nmse_real_signals() {
        let reference = vec![1.0, 2.0, 3.0, 4.0];
        let test = vec![1.01, 2.02, 3.03, 4.04];
        let nmse = nmse_db_real(&reference, &test);
        // Error is about 0.01^2 per sample relative to signal power
        assert!(nmse < -30.0, "NMSE should be < -30 dB for 1% error, got {}", nmse);
        assert!(nmse > -60.0, "NMSE should be > -60 dB, got {}", nmse);
    }

    // --- AM/AM, AM/PM ---

    #[test]
    fn test_amam_ampm_linear() {
        // Linear system: output = 2 * input (pure gain, no distortion)
        let input: Vec<Cplx> = vec![(1.0, 0.0), (0.5, 0.5), (0.0, 1.0)];
        let output: Vec<Cplx> = input.iter().map(|&x| cx_scale(x, 2.0)).collect();

        let amam_pm = extract_am_am_pm(&input, &output);

        for (in_amp, out_amp, phase) in &amam_pm {
            // AM/AM: output amp should be 2x input amp
            assert!(approx_eq(*out_amp, 2.0 * in_amp, 1e-10));
            // AM/PM: no phase shift for linear gain
            assert!(approx_eq(*phase, 0.0, 1e-10));
        }
    }

    #[test]
    fn test_amam_curve_binning() {
        // Create a simple compressing PA characteristic
        let n = 1000;
        let input: Vec<Cplx> = (0..n)
            .map(|i| {
                let amp = (i as f64) / (n as f64) * 2.0;
                (amp, 0.0)
            })
            .collect();
        let output = rapp_pa_model(&input, 1.0, 1.0, 2.0);

        let (centers, avgs) = am_am_curve(&input, &output, 10);
        assert_eq!(centers.len(), 10);
        assert_eq!(avgs.len(), 10);

        // At low amplitude, gain should be close to 1
        // First non-empty bin should have out ~= in
        if avgs[0] > 0.0 && centers[0] > 0.0 {
            let gain = avgs[0] / centers[0];
            assert!(gain > 0.8, "Small-signal gain should be near 1.0, got {}", gain);
        }
    }

    #[test]
    fn test_ampm_curve_binning() {
        let input: Vec<Cplx> = (0..100)
            .map(|i| {
                let amp = (i as f64 + 1.0) / 100.0;
                (amp, 0.0)
            })
            .collect();
        // Linear system: no phase shift
        let output = input.clone();
        let (centers, phases) = am_pm_curve(&input, &output, 5);
        assert_eq!(centers.len(), 5);
        for &ph in &phases {
            assert!(ph.abs() < 1.0, "Phase should be ~0 for linear system, got {}", ph);
        }
    }

    // --- Rapp PA model ---

    #[test]
    fn test_rapp_pa_small_signal() {
        // At small amplitude, Rapp model should be nearly linear
        let input: Vec<Cplx> = vec![(0.01, 0.0)];
        let output = rapp_pa_model(&input, 10.0, 1.0, 2.0);
        // Expected: gain ~= 10.0 at small signal
        let expected_amp = 10.0 * 0.01;
        let actual_amp = cx_abs(output[0]);
        assert!(
            approx_eq(actual_amp, expected_amp, 1e-4),
            "Small signal: expected {}, got {}",
            expected_amp,
            actual_amp
        );
    }

    #[test]
    fn test_rapp_pa_saturation() {
        // At large amplitude, output should be bounded near saturation
        let input: Vec<Cplx> = vec![(100.0, 0.0)];
        let output = rapp_pa_model(&input, 1.0, 1.0, 2.0);
        let out_amp = cx_abs(output[0]);
        // Output should be close to but below saturation (1.0)
        assert!(
            out_amp < 1.01,
            "Saturated output should be near 1.0, got {}",
            out_amp
        );
        assert!(
            out_amp > 0.9,
            "Saturated output should be near 1.0, got {}",
            out_amp
        );
    }

    // --- Parameter count ---

    #[test]
    fn test_num_parameters() {
        let f1 = VolterraFilter::new(VolterraOrder::First, 4);
        assert_eq!(f1.num_parameters(), 4); // just kernel1

        let f2 = VolterraFilter::new(VolterraOrder::Second, 3);
        // kernel1(3) + kernel2(3x3) = 3 + 9 = 12
        assert_eq!(f2.num_parameters(), 12);

        let f3 = VolterraFilter::new(VolterraOrder::Third, 2);
        // kernel1(2) + kernel2(2x2) + kernel3_diag(2) = 2 + 4 + 2 = 8
        assert_eq!(f3.num_parameters(), 8);

        let f5 = VolterraFilter::new(VolterraOrder::Fifth, 2);
        // kernel1(2) + kernel2(4) + kernel3_diag(2) + kernel5_diag(2) = 10
        assert_eq!(f5.num_parameters(), 10);
    }

    // --- Edge cases ---

    #[test]
    fn test_empty_input() {
        let f = VolterraFilter::new(VolterraOrder::Third, 3);
        let output = f.process_real(&[]);
        assert!(output.is_empty());

        let output_cx = f.process_complex(&[]);
        assert!(output_cx.is_empty());
    }

    #[test]
    fn test_memory_polynomial_set_get_coeff() {
        let mut mp = MemoryPolynomial::new(5, 2);
        mp.set_coeff(1, 0, (1.0, 2.0));
        mp.set_coeff(3, 1, (3.0, 4.0));
        mp.set_coeff(5, 0, (5.0, 6.0));

        assert!(cx_approx_eq(mp.get_coeff(1, 0), (1.0, 2.0), TOL));
        assert!(cx_approx_eq(mp.get_coeff(3, 1), (3.0, 4.0), TOL));
        assert!(cx_approx_eq(mp.get_coeff(5, 0), (5.0, 6.0), TOL));
        // Unset coefficients should be zero
        assert!(cx_approx_eq(mp.get_coeff(1, 1), (0.0, 0.0), TOL));
    }

    #[test]
    fn test_flat_coefficients_roundtrip() {
        let mut mp = MemoryPolynomial::new(3, 2);
        let flat: Vec<Cplx> = vec![(1.0, 0.1), (2.0, 0.2), (3.0, 0.3), (4.0, 0.4)];
        mp.set_coefficients_flat(&flat);

        assert!(cx_approx_eq(mp.get_coeff(1, 0), (1.0, 0.1), TOL));
        assert!(cx_approx_eq(mp.get_coeff(1, 1), (2.0, 0.2), TOL));
        assert!(cx_approx_eq(mp.get_coeff(3, 0), (3.0, 0.3), TOL));
        assert!(cx_approx_eq(mp.get_coeff(3, 1), (4.0, 0.4), TOL));
    }
}
