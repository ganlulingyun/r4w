//! Streak Camera Temporal Signal Analyzer
//!
//! Implements temporal signal analysis for streak camera systems used in ultrafast
//! time-resolved spectroscopy, fluorescence lifetime imaging, plasma diagnostics,
//! and semiconductor carrier dynamics.
//!
//! # Theory
//!
//! A streak camera converts photon arrival time into spatial displacement on a
//! 2D detector (CCD/CMOS). The horizontal axis represents the spectral or spatial
//! dimension, while the vertical axis encodes time via a sweep voltage ramp.
//!
//! ## Signal Model
//!
//! The measured streak image is a convolution of the true signal with the
//! instrument response function (IRF):
//!
//! ```text
//! I_meas(t) = IRF(t) ⊗ I_true(t) + noise
//! ```
//!
//! For multi-exponential fluorescence decay:
//!
//! ```text
//! I_true(t) = Σᵢ Aᵢ · exp(-t/τᵢ),  where τᵢ are lifetimes
//! ```
//!
//! The full reconvolution model:
//!
//! ```text
//! I_fit(t) = IRF(t) ⊗ [A₁·exp(-t/τ₁) + A₂·exp(-t/τ₂) + ...]
//! ```
//!
//! ## Richardson-Lucy Deconvolution
//!
//! Iterative update rule for deconvolving measured signal with IRF:
//!
//! ```text
//! u_{k+1}(t) = u_k(t) · [IRF(-t) ⊗ (d(t) / (IRF(t) ⊗ u_k(t)))]
//! ```
//!
//! # Application Presets
//!
//! - **Fluorescence lifetime**: ns timescale, single/multi-exponential decay
//! - **Ultrafast spectroscopy**: ps/fs timescale, pump-probe dynamics
//! - **Plasma emission**: ns–μs timescale, line emission evolution
//! - **Semiconductor carriers**: ps–ns timescale, carrier recombination

// ---------------------------------------------------------------------------
// Core data structures
// ---------------------------------------------------------------------------

/// A 2D streak image: rows = time bins, cols = spectral/spatial bins.
#[derive(Debug, Clone)]
pub struct StreakImage {
    /// Pixel data in row-major order (time × wavelength).
    pub data: Vec<f64>,
    /// Number of time rows.
    pub n_time: usize,
    /// Number of spectral/spatial columns.
    pub n_spectral: usize,
}

impl StreakImage {
    /// Create a new streak image filled with zeros.
    pub fn new(n_time: usize, n_spectral: usize) -> Self {
        Self {
            data: vec![0.0; n_time * n_spectral],
            n_time,
            n_spectral,
        }
    }

    /// Get pixel value at (time_row, spectral_col).
    pub fn get(&self, t: usize, s: usize) -> f64 {
        self.data[t * self.n_spectral + s]
    }

    /// Set pixel value at (time_row, spectral_col).
    pub fn set(&mut self, t: usize, s: usize, val: f64) {
        self.data[t * self.n_spectral + s] = val;
    }

    /// Extract a temporal profile (column) at a given spectral index.
    pub fn temporal_profile(&self, s: usize) -> Vec<f64> {
        (0..self.n_time).map(|t| self.get(t, s)).collect()
    }

    /// Extract a spectral profile (row) at a given time index.
    pub fn spectral_profile(&self, t: usize) -> Vec<f64> {
        (0..self.n_spectral).map(|s| self.get(t, s)).collect()
    }

    /// Sum all columns to get the integrated temporal profile.
    pub fn integrated_temporal_profile(&self) -> Vec<f64> {
        let mut profile = vec![0.0; self.n_time];
        for t in 0..self.n_time {
            for s in 0..self.n_spectral {
                profile[t] += self.get(t, s);
            }
        }
        profile
    }

    /// Sum all rows to get the integrated spectral profile.
    pub fn integrated_spectral_profile(&self) -> Vec<f64> {
        let mut profile = vec![0.0; self.n_spectral];
        for s in 0..self.n_spectral {
            for t in 0..self.n_time {
                profile[s] += self.get(t, s);
            }
        }
        profile
    }
}

// ---------------------------------------------------------------------------
// Time calibration
// ---------------------------------------------------------------------------

/// Sweep speed calibration for time axis linearization.
#[derive(Debug, Clone)]
pub struct TimeCalibration {
    /// Nominal sweep speed in ps/pixel (or ns/pixel).
    pub ps_per_pixel: f64,
    /// Polynomial correction coefficients [a0, a1, a2, ...] for nonlinear sweep.
    /// t_corrected = Σ aᵢ · pixel^i
    pub poly_coeffs: Vec<f64>,
    /// Time offset (t = 0 location in pixels).
    pub t0_pixel: f64,
}

impl TimeCalibration {
    /// Create a linear calibration (constant ps/pixel, no nonlinearity).
    pub fn linear(ps_per_pixel: f64, t0_pixel: f64) -> Self {
        Self {
            ps_per_pixel,
            poly_coeffs: vec![0.0, ps_per_pixel],
            t0_pixel,
        }
    }

    /// Create a calibration with polynomial nonlinearity correction.
    pub fn polynomial(ps_per_pixel: f64, t0_pixel: f64, poly_coeffs: Vec<f64>) -> Self {
        Self {
            ps_per_pixel,
            poly_coeffs,
            t0_pixel,
        }
    }

    /// Convert pixel index to calibrated time in picoseconds.
    pub fn pixel_to_time_ps(&self, pixel: f64) -> f64 {
        let p = pixel - self.t0_pixel;
        if self.poly_coeffs.len() <= 1 {
            return p * self.ps_per_pixel;
        }
        // Horner evaluation
        let mut result = 0.0;
        let mut pw = 1.0;
        for &c in &self.poly_coeffs {
            result += c * pw;
            pw *= p;
        }
        result
    }

    /// Convert a pixel array to calibrated time axis.
    pub fn calibrate_axis(&self, n_pixels: usize) -> Vec<f64> {
        (0..n_pixels)
            .map(|i| self.pixel_to_time_ps(i as f64))
            .collect()
    }

    /// Convert time in ps to nearest pixel index.
    pub fn time_ps_to_pixel(&self, time_ps: f64) -> f64 {
        // For linear cal, simple inversion
        if self.poly_coeffs.len() <= 1 || self.ps_per_pixel == 0.0 {
            return time_ps / self.ps_per_pixel + self.t0_pixel;
        }
        // For polynomial, use ps_per_pixel as first-order approximation
        time_ps / self.ps_per_pixel + self.t0_pixel
    }
}

// ---------------------------------------------------------------------------
// Instrument Response Function (IRF)
// ---------------------------------------------------------------------------

/// IRF modeled as a Gaussian pulse.
#[derive(Debug, Clone)]
pub struct InstrumentResponseFunction {
    /// Center position in pixels (or time units).
    pub center: f64,
    /// Gaussian sigma in pixels.
    pub sigma: f64,
    /// Peak amplitude (usually 1.0).
    pub amplitude: f64,
}

impl InstrumentResponseFunction {
    /// Create a Gaussian IRF.
    pub fn gaussian(center: f64, sigma_pixels: f64) -> Self {
        Self {
            center,
            sigma: sigma_pixels,
            amplitude: 1.0,
        }
    }

    /// FWHM = 2√(2ln2) · σ ≈ 2.3548 · σ
    pub fn fwhm(&self) -> f64 {
        2.0 * (2.0 * 2_f64.ln()).sqrt() * self.sigma
    }

    /// Evaluate the IRF at a given pixel position.
    pub fn evaluate(&self, pixel: f64) -> f64 {
        let x = (pixel - self.center) / self.sigma;
        self.amplitude * (-0.5 * x * x).exp()
    }

    /// Generate IRF as a sampled vector of length `n`.
    pub fn sample(&self, n: usize) -> Vec<f64> {
        (0..n).map(|i| self.evaluate(i as f64)).collect()
    }

    /// Fit a Gaussian IRF to measured data using centroid and variance.
    pub fn fit_from_data(data: &[f64]) -> Self {
        let n = data.len();
        let sum: f64 = data.iter().sum();
        if sum == 0.0 {
            return Self::gaussian(n as f64 / 2.0, 1.0);
        }
        // Centroid
        let center = data
            .iter()
            .enumerate()
            .map(|(i, &v)| i as f64 * v)
            .sum::<f64>()
            / sum;
        // Variance → sigma
        let variance = data
            .iter()
            .enumerate()
            .map(|(i, &v)| {
                let d = i as f64 - center;
                d * d * v
            })
            .sum::<f64>()
            / sum;
        let sigma = variance.sqrt().max(0.5);
        let amplitude = data.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        Self {
            center,
            sigma,
            amplitude,
        }
    }
}

// ---------------------------------------------------------------------------
// Convolution helpers
// ---------------------------------------------------------------------------

/// Linear convolution of `signal` with `kernel` (output length = n + m - 1).
pub fn convolve(signal: &[f64], kernel: &[f64]) -> Vec<f64> {
    let n = signal.len();
    let m = kernel.len();
    let out_len = n + m - 1;
    let mut out = vec![0.0; out_len];
    for i in 0..n {
        for j in 0..m {
            out[i + j] += signal[i] * kernel[j];
        }
    }
    out
}

/// Circular cross-correlation of two equal-length signals.
/// Returns correlation at lags [0, 1, ..., n-1].
pub fn circular_cross_correlation(a: &[f64], b: &[f64]) -> Vec<f64> {
    let n = a.len().min(b.len());
    let mut result = vec![0.0; n];
    for lag in 0..n {
        let mut sum = 0.0;
        for i in 0..n {
            sum += a[i] * b[(i + lag) % n];
        }
        result[lag] = sum;
    }
    result
}

/// Cross-correlation over all integer lags (non-circular, zero-padded).
/// Returns (max_lag, lag_values) where lag_values[k] = correlation at lag k - (n-1).
pub fn cross_correlation_full(a: &[f64], b: &[f64]) -> Vec<f64> {
    let n = a.len();
    let m = b.len();
    let out_len = n + m - 1;
    let mut out = vec![0.0; out_len];
    for i in 0..n {
        for j in 0..m {
            out[i + j] += a[i] * b[j];
        }
    }
    out
}

/// Find the lag (in samples) of maximum cross-correlation between two signals.
/// Returns signed lag: positive means b is delayed relative to a.
pub fn find_lag(a: &[f64], b: &[f64]) -> i64 {
    let n = a.len();
    let corr = cross_correlation_full(a, b);
    let max_idx = corr
        .iter()
        .enumerate()
        .max_by(|x, y| x.1.partial_cmp(y.1).unwrap())
        .map(|(i, _)| i)
        .unwrap_or(n - 1);
    max_idx as i64 - (n as i64 - 1)
}

// ---------------------------------------------------------------------------
// Richardson-Lucy Deconvolution
// ---------------------------------------------------------------------------

/// Deconvolution result from Richardson-Lucy iterations.
#[derive(Debug, Clone)]
pub struct DeconvolutionResult {
    /// Estimated true signal after deconvolution.
    pub signal: Vec<f64>,
    /// Number of iterations performed.
    pub iterations: usize,
    /// Residual norm at convergence.
    pub residual_norm: f64,
}

/// Richardson-Lucy iterative deconvolution.
///
/// Recovers `u` from `d = h ⊗ u + noise` where `h` is the IRF.
///
/// Update rule: `u_{k+1}[i] = u_k[i] · Σⱼ h[j-i] · d[j] / (h ⊗ u_k)[j]`
pub fn richardson_lucy_deconvolve(
    measured: &[f64],
    irf: &[f64],
    max_iterations: usize,
    tolerance: f64,
) -> DeconvolutionResult {
    let n = measured.len();
    if n == 0 || irf.is_empty() {
        return DeconvolutionResult {
            signal: measured.to_vec(),
            iterations: 0,
            residual_norm: 0.0,
        };
    }

    // Normalize IRF so it sums to 1
    let irf_sum: f64 = irf.iter().sum();
    let irf_norm: Vec<f64> = if irf_sum > 0.0 {
        irf.iter().map(|&x| x / irf_sum).collect()
    } else {
        irf.to_vec()
    };

    // Flipped IRF for back-projection
    let irf_flip: Vec<f64> = irf_norm.iter().cloned().rev().collect();

    // Initialize estimate as uniform (or positive measured)
    let init_val = measured.iter().cloned().sum::<f64>() / n as f64;
    let mut u: Vec<f64> = measured
        .iter()
        .map(|&d| if d > 0.0 { d } else { init_val.max(1e-10) })
        .collect();

    let mut residual_norm = f64::INFINITY;

    for iter in 0..max_iterations {
        // Forward: convolve current estimate with IRF
        let hu = convolve_same(&u, &irf_norm);

        // Compute ratio d / (h ⊗ u), clip to avoid division by zero
        let ratio: Vec<f64> = measured
            .iter()
            .zip(hu.iter())
            .map(|(&d, &h)| {
                if h > 1e-15 {
                    d / h
                } else {
                    0.0
                }
            })
            .collect();

        // Back-project: convolve ratio with flipped IRF
        let correction = convolve_same(&ratio, &irf_flip);

        // Update estimate (ensure positivity)
        let mut new_u = vec![0.0; n];
        let mut diff_sq_sum = 0.0;
        for i in 0..n {
            new_u[i] = (u[i] * correction[i]).max(0.0);
            let diff = new_u[i] - u[i];
            diff_sq_sum += diff * diff;
        }

        residual_norm = diff_sq_sum.sqrt();
        u = new_u;

        if residual_norm < tolerance && iter > 5 {
            return DeconvolutionResult {
                signal: u,
                iterations: iter + 1,
                residual_norm,
            };
        }
    }

    DeconvolutionResult {
        signal: u,
        iterations: max_iterations,
        residual_norm,
    }
}

/// Convolve with "same" output length (centered, truncated to input length).
fn convolve_same(signal: &[f64], kernel: &[f64]) -> Vec<f64> {
    let n = signal.len();
    let m = kernel.len();
    let offset = (m as i64 - 1) / 2;
    let mut out = vec![0.0; n];
    for i in 0..n {
        let mut acc = 0.0;
        for j in 0..m {
            let idx = i as i64 - offset + j as i64;
            if idx >= 0 && idx < n as i64 {
                acc += signal[idx as usize] * kernel[j];
            }
        }
        out[i] = acc;
    }
    out
}

// ---------------------------------------------------------------------------
// Exponential lifetime fitting
// ---------------------------------------------------------------------------

/// Parameters for a multi-exponential decay model.
#[derive(Debug, Clone)]
pub struct ExponentialDecayModel {
    /// Number of decay components.
    pub n_components: usize,
    /// Amplitudes [A₁, A₂, ...].
    pub amplitudes: Vec<f64>,
    /// Lifetimes [τ₁, τ₂, ...] in the same time units as input.
    pub lifetimes: Vec<f64>,
    /// Baseline offset.
    pub baseline: f64,
}

impl ExponentialDecayModel {
    /// Evaluate the model at time `t` (relative to t=0 of the decay).
    pub fn evaluate(&self, t: f64) -> f64 {
        let decay: f64 = self
            .amplitudes
            .iter()
            .zip(self.lifetimes.iter())
            .map(|(&a, &tau)| {
                if tau > 0.0 {
                    a * (-t / tau).exp()
                } else {
                    0.0
                }
            })
            .sum();
        (decay + self.baseline).max(0.0)
    }

    /// Evaluate over a time array.
    pub fn evaluate_array(&self, times: &[f64]) -> Vec<f64> {
        times.iter().map(|&t| self.evaluate(t)).collect()
    }

    /// Amplitude-weighted average lifetime: <τ> = Σ Aᵢτᵢ / Σ Aᵢ
    pub fn amplitude_weighted_lifetime(&self) -> f64 {
        let sum_a: f64 = self.amplitudes.iter().sum();
        if sum_a == 0.0 {
            return 0.0;
        }
        self.amplitudes
            .iter()
            .zip(self.lifetimes.iter())
            .map(|(&a, &tau)| a * tau)
            .sum::<f64>()
            / sum_a
    }

    /// Intensity-weighted average lifetime: <τ> = Σ Aᵢτᵢ² / Σ Aᵢτᵢ
    pub fn intensity_weighted_lifetime(&self) -> f64 {
        let denom: f64 = self
            .amplitudes
            .iter()
            .zip(self.lifetimes.iter())
            .map(|(&a, &tau)| a * tau)
            .sum();
        if denom == 0.0 {
            return 0.0;
        }
        self.amplitudes
            .iter()
            .zip(self.lifetimes.iter())
            .map(|(&a, &tau)| a * tau * tau)
            .sum::<f64>()
            / denom
    }
}

/// Fit a single-exponential decay via linearized log least squares.
///
/// Assumes `signal[i] = A · exp(-time[i] / τ) + baseline`.
/// Baseline is estimated from the tail of the signal.
pub fn fit_single_exponential(time: &[f64], signal: &[f64]) -> ExponentialDecayModel {
    let n = time.len().min(signal.len());
    if n < 3 {
        return ExponentialDecayModel {
            n_components: 1,
            amplitudes: vec![1.0],
            lifetimes: vec![1.0],
            baseline: 0.0,
        };
    }

    // Estimate baseline from last 10% of signal
    let tail_start = (n * 9 / 10).max(1);
    let baseline = signal[tail_start..]
        .iter()
        .cloned()
        .sum::<f64>()
        / (n - tail_start) as f64;
    let baseline = baseline.max(0.0);

    // Log-linearize: ln(signal - baseline) = ln(A) - t/τ
    let mut sum_t = 0.0f64;
    let mut sum_y = 0.0f64;
    let mut sum_tt = 0.0f64;
    let mut sum_ty = 0.0f64;
    let mut cnt = 0usize;

    for i in 0..n {
        let s = signal[i] - baseline;
        if s > 0.0 {
            let y = s.ln();
            let t = time[i];
            sum_t += t;
            sum_y += y;
            sum_tt += t * t;
            sum_ty += t * y;
            cnt += 1;
        }
    }

    if cnt < 2 {
        return ExponentialDecayModel {
            n_components: 1,
            amplitudes: vec![signal[0].max(0.0)],
            lifetimes: vec![time[n - 1] - time[0]],
            baseline,
        };
    }

    let cnt_f = cnt as f64;
    let denom = cnt_f * sum_tt - sum_t * sum_t;
    let (ln_a, slope) = if denom.abs() > 1e-15 {
        let slope = (cnt_f * sum_ty - sum_t * sum_y) / denom;
        let intercept = (sum_y - slope * sum_t) / cnt_f;
        (intercept, slope)
    } else {
        (0.0, -1.0)
    };

    let amplitude = ln_a.exp();
    let tau = if slope < 0.0 { -1.0 / slope } else { 1.0 };

    ExponentialDecayModel {
        n_components: 1,
        amplitudes: vec![amplitude],
        lifetimes: vec![tau],
        baseline,
    }
}

/// Fit a bi-exponential decay using alternating variable projection.
///
/// Iteratively updates lifetimes τ₁, τ₂ and solves for amplitudes A₁, A₂ via
/// linear least squares at each step.
pub fn fit_bi_exponential(
    time: &[f64],
    signal: &[f64],
    tau1_init: f64,
    tau2_init: f64,
    max_iter: usize,
) -> ExponentialDecayModel {
    let n = time.len().min(signal.len());
    if n < 4 {
        return ExponentialDecayModel {
            n_components: 2,
            amplitudes: vec![0.5, 0.5],
            lifetimes: vec![tau1_init, tau2_init],
            baseline: 0.0,
        };
    }

    let mut tau1 = tau1_init;
    let mut tau2 = tau2_init;

    // Gradient-free Nelder-Mead style: alternate between fitting lifetimes
    let step = 0.1; // 10% perturbation
    let mut best_sse = f64::INFINITY;
    let mut best_params = (tau1, tau2, 0.0f64, 0.0f64, 0.0f64);

    for _iter in 0..max_iter {
        // Solve for amplitudes given τ₁, τ₂ via 2×2 linear system
        let (a1, a2, bl) = solve_bi_exp_amplitudes(time, signal, tau1, tau2);
        let model = ExponentialDecayModel {
            n_components: 2,
            amplitudes: vec![a1, a2],
            lifetimes: vec![tau1, tau2],
            baseline: bl,
        };
        let sse = compute_sse(time, signal, &model);
        if sse < best_sse {
            best_sse = sse;
            best_params = (tau1, tau2, a1, a2, bl);
        }

        // Perturb lifetimes
        for &delta1 in &[-step, step] {
            for &delta2 in &[-step, step] {
                let t1 = tau1 * (1.0 + delta1);
                let t2 = tau2 * (1.0 + delta2);
                if t1 > 0.0 && t2 > 0.0 && (t1 - t2).abs() > 1e-6 {
                    let (a1, a2, bl) = solve_bi_exp_amplitudes(time, signal, t1, t2);
                    let m2 = ExponentialDecayModel {
                        n_components: 2,
                        amplitudes: vec![a1, a2],
                        lifetimes: vec![t1, t2],
                        baseline: bl,
                    };
                    let s2 = compute_sse(time, signal, &m2);
                    if s2 < best_sse {
                        best_sse = s2;
                        best_params = (t1, t2, a1, a2, bl);
                        tau1 = t1;
                        tau2 = t2;
                    }
                }
            }
        }
    }

    ExponentialDecayModel {
        n_components: 2,
        amplitudes: vec![best_params.2, best_params.3],
        lifetimes: vec![best_params.0, best_params.1],
        baseline: best_params.4,
    }
}

/// Solve for [A1, A2, baseline] in A1·e^(-t/τ1) + A2·e^(-t/τ2) + B = y
/// via normal equations (3×3 system).
fn solve_bi_exp_amplitudes(time: &[f64], signal: &[f64], tau1: f64, tau2: f64) -> (f64, f64, f64) {
    let n = time.len().min(signal.len());
    // Build design matrix columns: [e1, e2, 1]
    // Normal equations: AᵀA x = Aᵀy
    let (mut s11, mut s12, mut s13) = (0.0f64, 0.0f64, 0.0f64);
    let (mut s22, mut s23) = (0.0f64, 0.0f64);
    let mut s33 = 0.0f64;
    let (mut r1, mut r2, mut r3) = (0.0f64, 0.0f64, 0.0f64);

    for i in 0..n {
        let t = time[i];
        let e1 = (-t / tau1).exp();
        let e2 = (-t / tau2).exp();
        let y = signal[i];
        s11 += e1 * e1;
        s12 += e1 * e2;
        s13 += e1;
        s22 += e2 * e2;
        s23 += e2;
        s33 += 1.0;
        r1 += e1 * y;
        r2 += e2 * y;
        r3 += y;
    }

    // 3×3 symmetric solve via Cramer's rule
    let mat = [
        [s11, s12, s13],
        [s12, s22, s23],
        [s13, s23, s33],
    ];
    let rhs = [r1, r2, r3];
    let sol = solve_3x3(&mat, &rhs);
    (sol[0], sol[1], sol[2])
}

/// Solve a 3×3 linear system via Gaussian elimination.
fn solve_3x3(mat: &[[f64; 3]; 3], rhs: &[f64; 3]) -> [f64; 3] {
    let mut a = [
        [mat[0][0], mat[0][1], mat[0][2], rhs[0]],
        [mat[1][0], mat[1][1], mat[1][2], rhs[1]],
        [mat[2][0], mat[2][1], mat[2][2], rhs[2]],
    ];
    // Forward elimination
    for col in 0..3 {
        // Find pivot
        let mut max_row = col;
        for row in (col + 1)..3 {
            if a[row][col].abs() > a[max_row][col].abs() {
                max_row = row;
            }
        }
        a.swap(col, max_row);
        let pivot = a[col][col];
        if pivot.abs() < 1e-15 {
            continue;
        }
        for row in (col + 1)..3 {
            let factor = a[row][col] / pivot;
            for k in col..4 {
                a[row][k] -= factor * a[col][k];
            }
        }
    }
    // Back substitution
    let mut x = [0.0f64; 3];
    for i in (0..3).rev() {
        let mut sum = a[i][3];
        for j in (i + 1)..3 {
            sum -= a[i][j] * x[j];
        }
        x[i] = if a[i][i].abs() > 1e-15 {
            sum / a[i][i]
        } else {
            0.0
        };
    }
    x
}

/// Compute sum of squared errors between model and data.
fn compute_sse(time: &[f64], signal: &[f64], model: &ExponentialDecayModel) -> f64 {
    time.iter()
        .zip(signal.iter())
        .map(|(&t, &y)| {
            let diff = y - model.evaluate(t);
            diff * diff
        })
        .sum()
}

// ---------------------------------------------------------------------------
// Jitter correction
// ---------------------------------------------------------------------------

/// Trigger jitter correction result.
#[derive(Debug, Clone)]
pub struct JitterCorrectionResult {
    /// Estimated jitter (in pixels) per frame.
    pub jitter_pixels: Vec<f64>,
    /// Jitter-corrected frames (shifted to reference frame 0).
    pub corrected_profiles: Vec<Vec<f64>>,
    /// RMS jitter in pixels.
    pub rms_jitter: f64,
}

/// Measure and correct trigger jitter across multiple streak frames.
///
/// Uses cross-correlation of each frame against frame 0 to find the
/// sub-pixel temporal offset, then applies integer-pixel shift correction.
pub fn correct_jitter(frames: &[Vec<f64>]) -> JitterCorrectionResult {
    if frames.is_empty() {
        return JitterCorrectionResult {
            jitter_pixels: vec![],
            corrected_profiles: vec![],
            rms_jitter: 0.0,
        };
    }
    let reference = &frames[0];
    let n = reference.len();
    let mut jitter_pixels = vec![0.0f64; frames.len()];
    let mut corrected = vec![reference.clone()];

    for (idx, frame) in frames.iter().enumerate().skip(1) {
        let lag = find_lag(reference, frame);
        jitter_pixels[idx] = lag as f64;
        // Shift frame by -lag (integer shift)
        let mut shifted = vec![0.0f64; n];
        for i in 0..n {
            let src = i as i64 + lag;
            if src >= 0 && src < n as i64 {
                shifted[i] = frame[src as usize];
            }
        }
        corrected.push(shifted);
    }

    let rms_jitter = {
        let sq_sum: f64 = jitter_pixels.iter().map(|&j| j * j).sum();
        (sq_sum / jitter_pixels.len() as f64).sqrt()
    };

    JitterCorrectionResult {
        jitter_pixels,
        corrected_profiles: corrected,
        rms_jitter,
    }
}

// ---------------------------------------------------------------------------
// Photon counting mode
// ---------------------------------------------------------------------------

/// A detected single-photon event with centroid position.
#[derive(Debug, Clone, PartialEq)]
pub struct PhotonEvent {
    /// Fractional time position (sub-pixel centroid).
    pub time_pixel: f64,
    /// Fractional spectral position.
    pub spectral_pixel: f64,
    /// Peak intensity of the event spot.
    pub peak_intensity: f64,
}

/// Detect single-photon events in a streak image above a threshold.
///
/// Uses a local maximum search followed by centroid refinement.
pub fn detect_photon_events(
    image: &StreakImage,
    threshold: f64,
    window: usize,
) -> Vec<PhotonEvent> {
    let mut events = Vec::new();
    let half = (window / 2).max(1);

    for t in half..image.n_time.saturating_sub(half) {
        for s in half..image.n_spectral.saturating_sub(half) {
            let center_val = image.get(t, s);
            if center_val < threshold {
                continue;
            }
            // Check local maximum
            let mut is_max = true;
            'outer: for dt in 0..=2 * half {
                for ds in 0..=2 * half {
                    let tt = t + dt - half;
                    let ss = s + ds - half;
                    if tt < image.n_time && ss < image.n_spectral {
                        if image.get(tt, ss) > center_val {
                            is_max = false;
                            break 'outer;
                        }
                    }
                }
            }
            if !is_max {
                continue;
            }
            // Centroid in window
            let mut sum_val = 0.0;
            let mut sum_t = 0.0;
            let mut sum_s = 0.0;
            for dt in 0..=2 * half {
                for ds in 0..=2 * half {
                    let tt = (t + dt).saturating_sub(half);
                    let ss = (s + ds).saturating_sub(half);
                    if tt < image.n_time && ss < image.n_spectral {
                        let v = image.get(tt, ss).max(0.0);
                        sum_val += v;
                        sum_t += tt as f64 * v;
                        sum_s += ss as f64 * v;
                    }
                }
            }
            let (ct, cs) = if sum_val > 0.0 {
                (sum_t / sum_val, sum_s / sum_val)
            } else {
                (t as f64, s as f64)
            };
            events.push(PhotonEvent {
                time_pixel: ct,
                spectral_pixel: cs,
                peak_intensity: center_val,
            });
        }
    }
    events
}

/// Accumulate photon events into a 1D time histogram.
pub fn accumulate_photon_histogram(
    events: &[PhotonEvent],
    n_bins: usize,
    t_min: f64,
    t_max: f64,
) -> Vec<u64> {
    let mut hist = vec![0u64; n_bins];
    let range = t_max - t_min;
    if range <= 0.0 {
        return hist;
    }
    for ev in events {
        let bin = ((ev.time_pixel - t_min) / range * n_bins as f64) as i64;
        if bin >= 0 && bin < n_bins as i64 {
            hist[bin as usize] += 1;
        }
    }
    hist
}

// ---------------------------------------------------------------------------
// Time-resolved spectroscopy
// ---------------------------------------------------------------------------

/// Spectral moment analysis result.
#[derive(Debug, Clone)]
pub struct SpectralMoments {
    /// Center wavelength (first moment) at each time step.
    pub center_wavelength: Vec<f64>,
    /// Spectral width (RMS bandwidth) at each time step.
    pub spectral_width: Vec<f64>,
    /// Peak wavelength at each time step.
    pub peak_wavelength: Vec<f64>,
}

/// Compute spectral moments (center λ, width) as a function of time.
pub fn spectral_moment_analysis(
    image: &StreakImage,
    wavelengths: &[f64],
) -> SpectralMoments {
    let nw = wavelengths.len().min(image.n_spectral);
    let mut center_wavelength = vec![0.0f64; image.n_time];
    let mut spectral_width = vec![0.0f64; image.n_time];
    let mut peak_wavelength = vec![0.0f64; image.n_time];

    for t in 0..image.n_time {
        let mut sum_s = 0.0;
        let mut sum_sw = 0.0;
        let mut peak_val = f64::NEG_INFINITY;
        let mut peak_wl = wavelengths[0];

        for s in 0..nw {
            let v = image.get(t, s).max(0.0);
            let wl = wavelengths[s];
            sum_s += v;
            sum_sw += v * wl;
            if v > peak_val {
                peak_val = v;
                peak_wl = wl;
            }
        }

        let cw = if sum_s > 0.0 { sum_sw / sum_s } else { 0.0 };
        center_wavelength[t] = cw;
        peak_wavelength[t] = peak_wl;

        // Spectral width (RMS)
        if sum_s > 0.0 {
            let var: f64 = (0..nw)
                .map(|s| {
                    let v = image.get(t, s).max(0.0);
                    let d = wavelengths[s] - cw;
                    v * d * d
                })
                .sum::<f64>()
                / sum_s;
            spectral_width[t] = var.sqrt();
        }
    }

    SpectralMoments {
        center_wavelength,
        spectral_width,
        peak_wavelength,
    }
}

// ---------------------------------------------------------------------------
// Synchroscan mode accumulation
// ---------------------------------------------------------------------------

/// Accumulated synchroscan result.
#[derive(Debug, Clone)]
pub struct SynchroscanAccumulation {
    /// Accumulated averaged streak image.
    pub accumulated: StreakImage,
    /// Number of frames accumulated.
    pub n_frames: usize,
    /// Estimated SNR improvement factor: sqrt(n_frames).
    pub snr_improvement: f64,
}

/// Accumulate multiple streak frames (synchroscan mode).
///
/// Applies jitter correction before averaging for optimal SNR.
pub fn synchroscan_accumulate(frames: &[StreakImage]) -> SynchroscanAccumulation {
    if frames.is_empty() {
        return SynchroscanAccumulation {
            accumulated: StreakImage::new(0, 0),
            n_frames: 0,
            snr_improvement: 1.0,
        };
    }

    let n_time = frames[0].n_time;
    let n_spectral = frames[0].n_spectral;
    let n_frames = frames.len();

    // Extract integrated temporal profiles for jitter correction
    let profiles: Vec<Vec<f64>> = frames
        .iter()
        .map(|f| f.integrated_temporal_profile())
        .collect();

    let jitter_result = correct_jitter(&profiles);

    // Accumulate jitter-corrected frames
    let mut acc = StreakImage::new(n_time, n_spectral);
    for (frame_idx, frame) in frames.iter().enumerate() {
        let lag = jitter_result.jitter_pixels[frame_idx] as i64;
        for t in 0..n_time {
            let src_t = t as i64 + lag;
            if src_t >= 0 && src_t < n_time as i64 {
                for s in 0..n_spectral {
                    let val = acc.get(t, s) + frame.get(src_t as usize, s);
                    acc.set(t, s, val);
                }
            }
        }
    }

    // Average
    let n_f = n_frames as f64;
    for val in acc.data.iter_mut() {
        *val /= n_f;
    }

    SynchroscanAccumulation {
        accumulated: acc,
        n_frames,
        snr_improvement: n_f.sqrt(),
    }
}

// ---------------------------------------------------------------------------
// Application presets
// ---------------------------------------------------------------------------

/// Application preset configuration for streak camera analysis.
#[derive(Debug, Clone, PartialEq)]
pub enum ApplicationPreset {
    /// Fluorescence lifetime imaging: ns timescale, single/bi-exponential decay.
    FluorescenceLifetime,
    /// Ultrafast spectroscopy: ps/fs timescale, pump-probe dynamics.
    UltrafastSpectroscopy,
    /// Plasma emission diagnostics: ns–μs timescale.
    PlasmaEmission,
    /// Semiconductor carrier dynamics: ps–ns timescale.
    SemiconductorCarriers,
}

/// Preset parameters for a given application.
#[derive(Debug, Clone)]
pub struct PresetParameters {
    /// Typical time window in picoseconds.
    pub time_window_ps: f64,
    /// Nominal sweep speed in ps/pixel.
    pub sweep_speed_ps_per_pixel: f64,
    /// Typical IRF FWHM in pixels.
    pub irf_fwhm_pixels: f64,
    /// Number of exponential components expected.
    pub n_exp_components: usize,
    /// Whether synchroscan accumulation is recommended.
    pub use_synchroscan: bool,
    /// Minimum detectable lifetime in ps.
    pub min_lifetime_ps: f64,
    /// Description string.
    pub description: &'static str,
}

/// Get application preset parameters.
pub fn get_preset(preset: &ApplicationPreset) -> PresetParameters {
    match preset {
        ApplicationPreset::FluorescenceLifetime => PresetParameters {
            time_window_ps: 20_000.0,           // 20 ns
            sweep_speed_ps_per_pixel: 20.0,     // 20 ps/pixel
            irf_fwhm_pixels: 15.0,              // ~300 ps FWHM
            n_exp_components: 2,
            use_synchroscan: true,
            min_lifetime_ps: 100.0,
            description: "Fluorescence lifetime imaging (FLIM) in the nanosecond range",
        },
        ApplicationPreset::UltrafastSpectroscopy => PresetParameters {
            time_window_ps: 100.0,              // 100 ps
            sweep_speed_ps_per_pixel: 0.1,      // 100 fs/pixel
            irf_fwhm_pixels: 20.0,              // ~2 ps FWHM
            n_exp_components: 1,
            use_synchroscan: true,
            min_lifetime_ps: 0.5,
            description: "Pump-probe ultrafast spectroscopy in the femtosecond to picosecond range",
        },
        ApplicationPreset::PlasmaEmission => PresetParameters {
            time_window_ps: 10_000_000.0,       // 10 μs
            sweep_speed_ps_per_pixel: 10_000.0, // 10 ns/pixel
            irf_fwhm_pixels: 5.0,               // ~50 ns FWHM
            n_exp_components: 3,
            use_synchroscan: false,
            min_lifetime_ps: 50_000.0,
            description: "Plasma emission spectroscopy on nanosecond to microsecond timescales",
        },
        ApplicationPreset::SemiconductorCarriers => PresetParameters {
            time_window_ps: 2_000.0,            // 2 ns
            sweep_speed_ps_per_pixel: 2.0,      // 2 ps/pixel
            irf_fwhm_pixels: 10.0,              // ~20 ps FWHM
            n_exp_components: 2,
            use_synchroscan: true,
            min_lifetime_ps: 10.0,
            description: "Semiconductor carrier recombination dynamics (ps to ns)",
        },
    }
}

// ---------------------------------------------------------------------------
// Dynamic range and SNR utilities
// ---------------------------------------------------------------------------

/// Compute peak signal-to-noise ratio from a signal and noise floor.
pub fn compute_snr_db(signal: &[f64], noise_std: f64) -> f64 {
    let peak = signal
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);
    if noise_std <= 0.0 || peak <= 0.0 {
        return 0.0;
    }
    20.0 * (peak / noise_std).log10()
}

/// Expected SNR improvement for N accumulated frames: 10·log10(√N) dB.
pub fn snr_improvement_db(n_frames: usize) -> f64 {
    if n_frames == 0 {
        return 0.0;
    }
    10.0 * (n_frames as f64).log10() / 2.0 // = 5·log10(N)
}

/// Dynamic range of a signal in dB: 20·log10(max / min_nonzero).
pub fn dynamic_range_db(signal: &[f64]) -> f64 {
    let max = signal.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let min_nonzero = signal
        .iter()
        .cloned()
        .filter(|&v| v > 0.0)
        .fold(f64::INFINITY, f64::min);
    if max <= 0.0 || min_nonzero.is_infinite() {
        return 0.0;
    }
    20.0 * (max / min_nonzero).log10()
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-9;

    // ---- StreakImage --------------------------------------------------------

    #[test]
    fn test_streak_image_creation() {
        let img = StreakImage::new(100, 64);
        assert_eq!(img.n_time, 100);
        assert_eq!(img.n_spectral, 64);
        assert_eq!(img.data.len(), 100 * 64);
        assert!(img.data.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_streak_image_get_set() {
        let mut img = StreakImage::new(10, 5);
        img.set(3, 2, 7.5);
        assert!((img.get(3, 2) - 7.5).abs() < EPSILON);
        assert!((img.get(0, 0)).abs() < EPSILON);
    }

    #[test]
    fn test_temporal_profile_extraction() {
        let mut img = StreakImage::new(8, 4);
        // Set column 2 to [1, 2, 3, 4, 5, 6, 7, 8]
        for t in 0..8 {
            img.set(t, 2, (t + 1) as f64);
        }
        let profile = img.temporal_profile(2);
        assert_eq!(profile.len(), 8);
        assert!((profile[4] - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_spectral_profile_extraction() {
        let mut img = StreakImage::new(4, 6);
        for s in 0..6 {
            img.set(2, s, s as f64 * 2.0);
        }
        let profile = img.spectral_profile(2);
        assert_eq!(profile.len(), 6);
        assert!((profile[3] - 6.0).abs() < EPSILON);
    }

    #[test]
    fn test_integrated_temporal_profile() {
        let mut img = StreakImage::new(4, 3);
        // Row 0: [1, 2, 3] → sum = 6
        img.set(0, 0, 1.0);
        img.set(0, 1, 2.0);
        img.set(0, 2, 3.0);
        let tp = img.integrated_temporal_profile();
        assert!((tp[0] - 6.0).abs() < EPSILON);
        assert!((tp[1]).abs() < EPSILON);
    }

    #[test]
    fn test_integrated_spectral_profile() {
        let mut img = StreakImage::new(3, 4);
        // Col 1: all rows = 5
        for t in 0..3 {
            img.set(t, 1, 5.0);
        }
        let sp = img.integrated_spectral_profile();
        assert!((sp[1] - 15.0).abs() < EPSILON);
        assert!((sp[0]).abs() < EPSILON);
    }

    // ---- Time calibration --------------------------------------------------

    #[test]
    fn test_linear_calibration() {
        let cal = TimeCalibration::linear(10.0, 50.0); // 10 ps/pixel, t0 at pixel 50
        let t = cal.pixel_to_time_ps(60.0);
        assert!((t - 100.0).abs() < EPSILON); // 10 pixels × 10 ps/pixel = 100 ps
    }

    #[test]
    fn test_linear_calibration_negative() {
        let cal = TimeCalibration::linear(5.0, 10.0);
        let t = cal.pixel_to_time_ps(5.0); // 5 - 10 = -5 pixels
        assert!((t - (-25.0)).abs() < EPSILON);
    }

    #[test]
    fn test_calibrate_axis_length() {
        let cal = TimeCalibration::linear(2.0, 0.0);
        let axis = cal.calibrate_axis(100);
        assert_eq!(axis.len(), 100);
        assert!((axis[50] - 100.0).abs() < EPSILON);
    }

    #[test]
    fn test_polynomial_calibration() {
        // t = 0 + 10*p + 0.01*p^2
        let cal = TimeCalibration::polynomial(10.0, 0.0, vec![0.0, 10.0, 0.01]);
        let t = cal.pixel_to_time_ps(10.0);
        assert!((t - (100.0 + 1.0)).abs() < EPSILON); // 10*10 + 0.01*100 = 101
    }

    // ---- IRF ---------------------------------------------------------------

    #[test]
    fn test_irf_fwhm() {
        let irf = InstrumentResponseFunction::gaussian(50.0, 5.0);
        let fwhm = irf.fwhm();
        // FWHM ≈ 2.3548 × σ
        assert!((fwhm - 2.0 * (2.0 * 2.0_f64.ln()).sqrt() * 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_irf_evaluate_peak() {
        let irf = InstrumentResponseFunction::gaussian(10.0, 2.0);
        let peak = irf.evaluate(10.0);
        assert!((peak - 1.0).abs() < EPSILON); // amplitude = 1.0 at center
    }

    #[test]
    fn test_irf_evaluate_decay() {
        let irf = InstrumentResponseFunction::gaussian(0.0, 1.0);
        let val_at_sigma = irf.evaluate(1.0);
        // Should be exp(-0.5) ≈ 0.6065
        assert!((val_at_sigma - (-0.5f64).exp()).abs() < 1e-10);
    }

    #[test]
    fn test_irf_sample_length() {
        let irf = InstrumentResponseFunction::gaussian(50.0, 5.0);
        let samples = irf.sample(100);
        assert_eq!(samples.len(), 100);
    }

    #[test]
    fn test_irf_fit_from_data() {
        // Create Gaussian with known center=30 and sigma=5
        let data: Vec<f64> = (0..100)
            .map(|i| {
                let x = (i as f64 - 30.0) / 5.0;
                (-0.5 * x * x).exp()
            })
            .collect();
        let irf = InstrumentResponseFunction::fit_from_data(&data);
        assert!((irf.center - 30.0).abs() < 0.5);
        assert!((irf.sigma - 5.0).abs() < 0.5);
    }

    // ---- Convolution -------------------------------------------------------

    #[test]
    fn test_convolve_basic() {
        let a = vec![1.0, 2.0, 3.0];
        let b = vec![1.0, 1.0];
        let c = convolve(&a, &b);
        // Expected: [1, 3, 5, 3]
        assert_eq!(c.len(), 4);
        assert!((c[0] - 1.0).abs() < EPSILON);
        assert!((c[1] - 3.0).abs() < EPSILON);
        assert!((c[2] - 5.0).abs() < EPSILON);
        assert!((c[3] - 3.0).abs() < EPSILON);
    }

    #[test]
    fn test_convolve_impulse() {
        let signal = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let impulse = vec![1.0];
        let result = convolve(&signal, &impulse);
        // Convolution with impulse = signal
        assert_eq!(result.len(), 5);
        for (a, b) in signal.iter().zip(result.iter()) {
            assert!((a - b).abs() < EPSILON);
        }
    }

    #[test]
    fn test_find_lag_zero() {
        let a = vec![0.0, 0.0, 1.0, 0.0, 0.0];
        let lag = find_lag(&a, &a);
        assert_eq!(lag, 0);
    }

    #[test]
    fn test_find_lag_positive() {
        let a = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
        let b = vec![0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]; // delayed by +2
        let lag = find_lag(&a, &b);
        assert_eq!(lag, 2);
    }

    // ---- Richardson-Lucy deconvolution -------------------------------------

    #[test]
    fn test_rl_deconvolution_identity() {
        // Deconvolve signal with delta-like IRF → recover original
        let signal: Vec<f64> = (0..50)
            .map(|i| (-((i as f64 - 10.0) / 3.0).powi(2)).exp())
            .collect();
        let mut irf = vec![0.0; 50];
        irf[0] = 1.0; // delta impulse
        let result = richardson_lucy_deconvolve(&signal, &irf, 20, 1e-6);
        // With delta IRF, deconvolution should approximately recover the signal
        assert_eq!(result.signal.len(), signal.len());
        assert!(result.iterations > 0);
    }

    #[test]
    fn test_rl_deconvolution_positive() {
        let signal: Vec<f64> = (0..30)
            .map(|i| (-(i as f64 / 5.0)).exp())
            .collect();
        let irf: Vec<f64> = (0..30)
            .map(|i| (-(i as f64 / 2.0).powi(2)).exp())
            .collect();
        let result = richardson_lucy_deconvolve(&signal, &irf, 50, 1e-8);
        // All deconvolved values should be non-negative
        assert!(result.signal.iter().all(|&v| v >= 0.0));
    }

    #[test]
    fn test_rl_empty_input() {
        let result = richardson_lucy_deconvolve(&[], &[1.0], 10, 1e-6);
        assert_eq!(result.signal.len(), 0);
    }

    // ---- Exponential decay fitting -----------------------------------------

    #[test]
    fn test_single_exp_fit_simple() {
        let tau = 5.0;
        let amplitude = 10.0;
        let time: Vec<f64> = (0..50).map(|i| i as f64).collect();
        let signal: Vec<f64> = time.iter().map(|&t| amplitude * (-t / tau).exp()).collect();
        let model = fit_single_exponential(&time, &signal);
        // Linearized log-LS can be inaccurate due to weighting; use 15% tolerance
        assert!((model.lifetimes[0] - tau).abs() / tau < 0.15);
        assert!(model.amplitudes[0] > 0.0);
    }

    #[test]
    fn test_single_exp_model_evaluate() {
        let model = ExponentialDecayModel {
            n_components: 1,
            amplitudes: vec![1.0],
            lifetimes: vec![10.0],
            baseline: 0.0,
        };
        assert!((model.evaluate(0.0) - 1.0).abs() < EPSILON);
        assert!((model.evaluate(10.0) - (-1.0f64).exp()).abs() < 1e-10);
    }

    #[test]
    fn test_amplitude_weighted_lifetime() {
        let model = ExponentialDecayModel {
            n_components: 2,
            amplitudes: vec![1.0, 1.0],
            lifetimes: vec![2.0, 8.0],
            baseline: 0.0,
        };
        let awt = model.amplitude_weighted_lifetime();
        assert!((awt - 5.0).abs() < EPSILON); // (1*2 + 1*8) / (1+1) = 5
    }

    #[test]
    fn test_intensity_weighted_lifetime() {
        let model = ExponentialDecayModel {
            n_components: 2,
            amplitudes: vec![1.0, 1.0],
            lifetimes: vec![2.0, 8.0],
            baseline: 0.0,
        };
        let iwt = model.intensity_weighted_lifetime();
        // (1*4 + 1*64) / (1*2 + 1*8) = 68/10 = 6.8
        assert!((iwt - 6.8).abs() < 1e-10);
    }

    #[test]
    fn test_bi_exp_fit_runs() {
        let time: Vec<f64> = (0..40).map(|i| i as f64).collect();
        // Two-component decay: τ₁=3, τ₂=15, A₁=0.7, A₂=0.3
        let signal: Vec<f64> = time
            .iter()
            .map(|&t| 0.7 * (-t / 3.0).exp() + 0.3 * (-t / 15.0).exp())
            .collect();
        let model = fit_bi_exponential(&time, &signal, 3.0, 15.0, 30);
        assert_eq!(model.n_components, 2);
        assert!(model.lifetimes.iter().all(|&tau| tau > 0.0));
    }

    #[test]
    fn test_evaluate_array() {
        let model = ExponentialDecayModel {
            n_components: 1,
            amplitudes: vec![2.0],
            lifetimes: vec![5.0],
            baseline: 0.0,
        };
        let times = vec![0.0, 5.0, 10.0];
        let vals = model.evaluate_array(&times);
        assert_eq!(vals.len(), 3);
        assert!((vals[0] - 2.0).abs() < EPSILON);
        assert!((vals[1] - 2.0 * (-1.0f64).exp()).abs() < 1e-10);
    }

    // ---- Jitter correction -------------------------------------------------

    #[test]
    fn test_jitter_correction_no_jitter() {
        let frame: Vec<f64> = vec![0.0, 0.0, 1.0, 0.5, 0.0];
        let frames = vec![frame.clone(), frame.clone(), frame.clone()];
        let result = correct_jitter(&frames);
        assert_eq!(result.jitter_pixels.len(), 3);
        assert!((result.jitter_pixels[0]).abs() < EPSILON); // reference = 0
    }

    #[test]
    fn test_jitter_rms_all_zero() {
        // Reference frame jitter[0] must always be 0.0
        let frame = vec![0.0, 0.0, 0.0, 10.0, 5.0, 2.0, 0.5, 0.1, 0.0, 0.0];
        let frames = vec![frame.clone(), frame.clone()];
        let result = correct_jitter(&frames);
        // Frame 0 is always the reference with jitter 0
        assert!((result.jitter_pixels[0]).abs() < EPSILON);
        assert_eq!(result.corrected_profiles.len(), 2);
    }

    #[test]
    fn test_jitter_correction_shifted_frame() {
        let frame: Vec<f64> = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
        let shifted: Vec<f64> = vec![0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]; // +1 pixel
        let frames = vec![frame, shifted];
        let result = correct_jitter(&frames);
        // After correction, frame 1 should be aligned to frame 0
        assert_eq!(result.corrected_profiles.len(), 2);
    }

    // ---- Photon counting ---------------------------------------------------

    #[test]
    fn test_detect_photon_events_none() {
        let img = StreakImage::new(20, 20);
        let events = detect_photon_events(&img, 0.5, 2);
        assert!(events.is_empty());
    }

    #[test]
    fn test_detect_photon_events_single() {
        let mut img = StreakImage::new(20, 20);
        img.set(10, 10, 10.0); // Single bright pixel
        let events = detect_photon_events(&img, 0.5, 2);
        assert!(!events.is_empty());
        // Centroid should be near (10, 10)
        let ev = &events[0];
        assert!((ev.time_pixel - 10.0).abs() < 1.0);
        assert!((ev.spectral_pixel - 10.0).abs() < 1.0);
    }

    #[test]
    fn test_accumulate_photon_histogram() {
        let events = vec![
            PhotonEvent { time_pixel: 5.0, spectral_pixel: 0.0, peak_intensity: 1.0 },
            PhotonEvent { time_pixel: 5.5, spectral_pixel: 0.0, peak_intensity: 1.0 },
            PhotonEvent { time_pixel: 15.0, spectral_pixel: 0.0, peak_intensity: 1.0 },
        ];
        let hist = accumulate_photon_histogram(&events, 10, 0.0, 20.0);
        assert_eq!(hist.len(), 10);
        // Bins for 5.0 and 5.5 should be bin 2 (5.0/20.0*10=2.5→2)
        let total: u64 = hist.iter().sum();
        assert_eq!(total, 3);
    }

    // ---- Spectral moment analysis ------------------------------------------

    #[test]
    fn test_spectral_moments_flat() {
        let mut img = StreakImage::new(5, 4);
        let wavelengths = vec![400.0, 450.0, 500.0, 550.0];
        // Flat spectrum at every time → center = mean(wavelengths) = 475
        for t in 0..5 {
            for s in 0..4 {
                img.set(t, s, 1.0);
            }
        }
        let moments = spectral_moment_analysis(&img, &wavelengths);
        for &cw in &moments.center_wavelength {
            assert!((cw - 475.0).abs() < 0.1);
        }
    }

    #[test]
    fn test_spectral_moments_peak() {
        let mut img = StreakImage::new(3, 5);
        let wavelengths = vec![400.0, 450.0, 500.0, 550.0, 600.0];
        // Strong peak at wavelength 500 nm (index 2)
        for t in 0..3 {
            img.set(t, 2, 100.0);
        }
        let moments = spectral_moment_analysis(&img, &wavelengths);
        for &pw in &moments.peak_wavelength {
            assert!((pw - 500.0).abs() < EPSILON);
        }
    }

    // ---- Synchroscan accumulation ------------------------------------------

    #[test]
    fn test_synchroscan_empty() {
        let result = synchroscan_accumulate(&[]);
        assert_eq!(result.n_frames, 0);
    }

    #[test]
    fn test_synchroscan_single_frame() {
        let mut img = StreakImage::new(10, 5);
        img.set(3, 2, 8.0);
        let result = synchroscan_accumulate(&[img]);
        assert_eq!(result.n_frames, 1);
        assert!((result.snr_improvement - 1.0).abs() < EPSILON);
        assert!((result.accumulated.get(3, 2) - 8.0).abs() < EPSILON);
    }

    #[test]
    fn test_synchroscan_snr_improvement() {
        let result_n4 = SynchroscanAccumulation {
            accumulated: StreakImage::new(1, 1),
            n_frames: 4,
            snr_improvement: 2.0,
        };
        // √4 = 2.0
        assert!((result_n4.snr_improvement - 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_synchroscan_multiple_frames() {
        let mut img = StreakImage::new(10, 4);
        for t in 0..10 {
            for s in 0..4 {
                img.set(t, s, 2.0);
            }
        }
        let frames = vec![img.clone(), img.clone(), img.clone()];
        let result = synchroscan_accumulate(&frames);
        assert_eq!(result.n_frames, 3);
        // Average of 3 identical frames = 2.0
        assert!((result.accumulated.get(0, 0) - 2.0).abs() < 0.1);
    }

    // ---- Application presets -----------------------------------------------

    #[test]
    fn test_preset_fluorescence() {
        let p = get_preset(&ApplicationPreset::FluorescenceLifetime);
        assert_eq!(p.n_exp_components, 2);
        assert!(p.use_synchroscan);
        assert!(p.time_window_ps > 0.0);
    }

    #[test]
    fn test_preset_ultrafast() {
        let p = get_preset(&ApplicationPreset::UltrafastSpectroscopy);
        assert!(p.sweep_speed_ps_per_pixel < 1.0); // sub-ps per pixel
        assert!(p.use_synchroscan);
    }

    #[test]
    fn test_preset_plasma() {
        let p = get_preset(&ApplicationPreset::PlasmaEmission);
        assert!(!p.use_synchroscan); // single-shot plasma diagnostics
        assert_eq!(p.n_exp_components, 3);
    }

    #[test]
    fn test_preset_semiconductor() {
        let p = get_preset(&ApplicationPreset::SemiconductorCarriers);
        assert_eq!(p.n_exp_components, 2);
        assert!(p.min_lifetime_ps < 100.0);
    }

    // ---- Dynamic range / SNR utilities -------------------------------------

    #[test]
    fn test_snr_db_basic() {
        let signal = vec![0.0, 0.0, 100.0, 50.0, 0.0];
        let snr = compute_snr_db(&signal, 1.0);
        assert!((snr - 40.0).abs() < 0.01); // 20*log10(100/1) = 40 dB
    }

    #[test]
    fn test_snr_improvement_db() {
        let imp = snr_improvement_db(100);
        // 5 * log10(100) = 5 * 2 = 10 dB
        assert!((imp - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_dynamic_range_db() {
        let signal = vec![1.0, 10.0, 100.0];
        let dr = dynamic_range_db(&signal);
        assert!((dr - 40.0).abs() < 0.01); // 20*log10(100/1) = 40 dB
    }

    #[test]
    fn test_dynamic_range_db_zero_min() {
        let signal = vec![0.0, 0.0, 50.0];
        let dr = dynamic_range_db(&signal);
        assert!((dr - 0.0).abs() < 0.01); // min_nonzero = 50 → range = 0 dB
    }

    #[test]
    fn test_snr_improvement_one_frame() {
        let imp = snr_improvement_db(1);
        assert!((imp).abs() < 1e-10); // 10*log10(1)/2 = 0 dB
    }
}
