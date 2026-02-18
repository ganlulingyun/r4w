//! Spin Echo NMR Signal Processor
//!
//! Implements spin echo NMR pulse sequence signal processing for measuring
//! T1 (spin-lattice) and T2 (spin-spin) relaxation times, as well as
//! diffusion coefficients via Stejskal-Tanner sequences.
//!
//! ## Key Sequences
//!
//! - **Hahn Echo**: 90°–τ–180°–τ–echo. Echo amplitude ∝ exp(−2τ/T2).
//! - **CPMG**: 90°–(τ–180°–τ)_N train. Measures T2 from N echo amplitudes.
//! - **Inversion Recovery**: 180°–τ–90°. Measures T1 via null crossing.
//! - **Stejskal-Tanner**: Pulsed field gradient SE for diffusion coefficient D.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::spin_echo_nmr_processor::{CpmgProcessor, CpmgConfig};
//!
//! let config = CpmgConfig {
//!     n_echoes: 32,
//!     echo_spacing: 2e-3,
//!     larmor_freq: 2.0e6,
//!     sample_rate: 10e3,
//!     samples_per_echo: 64,
//! };
//! let proc = CpmgProcessor::new(config);
//! let train = proc.synthesize_echo_train(1.0, 0.1);
//! assert_eq!(train.len(), 32);
//! assert!(train[0] > train[31]);
//! ```
//!
//! ## Physical Constants
//!
//! - γ_H = 267.522 × 10⁶ rad/(s·T)  (¹H proton gyromagnetic ratio)

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Proton (¹H) gyromagnetic ratio in rad/(s·T).
pub const GAMMA_H: f64 = 267.522e6;

// ---------------------------------------------------------------------------
// Helper math utilities (no external crates)
// ---------------------------------------------------------------------------

/// Compute exp of a value (re-exported from std).
#[inline]
fn exp(x: f64) -> f64 {
    x.exp()
}

/// Natural logarithm.
#[inline]
fn ln(x: f64) -> f64 {
    x.ln()
}

/// Solve Ax = b for an (n×n) system via Gaussian elimination with partial
/// pivoting.  Returns None if the system is singular.
fn gauss_eliminate(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for col in 0..n {
        // Find pivot
        let (pivot_row, _) = (col..n)
            .map(|r| (r, a[r][col].abs()))
            .max_by(|x, y| x.1.partial_cmp(&y.1).unwrap())?;
        a.swap(col, pivot_row);
        b.swap(col, pivot_row);

        let pivot = a[col][col];
        if pivot.abs() < 1e-14 {
            return None;
        }
        for j in col..n {
            a[col][j] /= pivot;
        }
        b[col] /= pivot;

        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = a[row][col];
            for j in col..n {
                let v = a[col][j];
                a[row][j] -= factor * v;
            }
            let bv = b[col];
            b[row] -= factor * bv;
        }
    }
    Some(b)
}

// ---------------------------------------------------------------------------
// Hahn Echo
// ---------------------------------------------------------------------------

/// Parameters for the Hahn (single) spin echo sequence: 90°–τ–180°–τ–echo.
#[derive(Debug, Clone)]
pub struct HahnEchoConfig {
    /// Larmor / centre frequency (Hz).
    pub larmor_freq: f64,
    /// Sampling rate for the echo signal (Hz).
    pub sample_rate: f64,
    /// Half the inter-echo delay τ (s).  Full echo time = 2τ.
    pub tau: f64,
    /// Number of complex samples to generate around the echo peak.
    pub samples_per_echo: usize,
}

/// Synthetic Hahn echo signal (magnitude envelope only, ignoring RF carrier).
///
/// Returns a vector of `samples_per_echo` real amplitudes centred on the echo
/// peak.  The peak amplitude is scaled by `exp(−2τ / T2)` in accordance with
/// the Hahn echo decay law.
///
/// # Arguments
/// * `config` – Hahn echo parameters.
/// * `m0`    – Equilibrium magnetisation (arbitrary units).
/// * `t2`    – Spin-spin relaxation time (s).
/// * `t2_star` – Effective relaxation time including field inhomogeneity (s).
///              Controls the width of the echo envelope.
pub fn hahn_echo_signal(
    config: &HahnEchoConfig,
    m0: f64,
    t2: f64,
    t2_star: f64,
) -> Vec<f64> {
    let peak_amp = m0 * exp(-2.0 * config.tau / t2);
    let dt = 1.0 / config.sample_rate;
    let half = config.samples_per_echo / 2;
    (0..config.samples_per_echo)
        .map(|i| {
            let t = (i as f64 - half as f64) * dt; // time relative to echo peak
            peak_amp * exp(-(t.abs()) / t2_star)
        })
        .collect()
}

/// Compute the peak amplitude of a Hahn echo at inter-pulse delay τ.
///
/// `A(τ) = M₀ · exp(−2τ / T₂)`
pub fn hahn_echo_amplitude(m0: f64, tau: f64, t2: f64) -> f64 {
    m0 * exp(-2.0 * tau / t2)
}

// ---------------------------------------------------------------------------
// CPMG Multi-Echo
// ---------------------------------------------------------------------------

/// Configuration for a CPMG (Carr-Purcell-Meiboom-Gill) echo train.
#[derive(Debug, Clone)]
pub struct CpmgConfig {
    /// Number of 180° refocusing pulses (= number of echoes).
    pub n_echoes: usize,
    /// Echo spacing 2τ (s) — time between consecutive echoes.
    pub echo_spacing: f64,
    /// Larmor frequency (Hz).
    pub larmor_freq: f64,
    /// Sampling rate (Hz) — for synthesising echo shapes.
    pub sample_rate: f64,
    /// Number of samples per echo acquisition window.
    pub samples_per_echo: usize,
}

/// CPMG echo-train processor.
pub struct CpmgProcessor {
    /// Configuration.
    pub config: CpmgConfig,
}

impl CpmgProcessor {
    /// Create a new CPMG processor.
    pub fn new(config: CpmgConfig) -> Self {
        Self { config }
    }

    /// Synthesise an echo-train amplitude vector.
    ///
    /// Echo amplitudes follow `A_n = M₀ · exp(−t_n / T₂)` where `t_n = n · TE`
    /// (`TE` = echo spacing, `n = 1…N`).
    ///
    /// # Arguments
    /// * `m0` – Initial magnetisation.
    /// * `t2` – Spin-spin relaxation time (s).
    pub fn synthesize_echo_train(&self, m0: f64, t2: f64) -> Vec<f64> {
        (1..=self.config.n_echoes)
            .map(|n| {
                let t = n as f64 * self.config.echo_spacing;
                m0 * exp(-t / t2)
            })
            .collect()
    }

    /// Fit T2 from a measured echo-train vector using single-exponential
    /// linear regression on ln(S) vs t.
    ///
    /// Returns `(T2, M0)` or `None` if fitting fails.
    pub fn fit_t2(&self, echo_amplitudes: &[f64]) -> Option<(f64, f64)> {
        fit_single_exponential_t2(echo_amplitudes, self.config.echo_spacing)
    }

    /// Compute echo times vector (1-indexed, in seconds).
    pub fn echo_times(&self) -> Vec<f64> {
        (1..=self.config.n_echoes)
            .map(|n| n as f64 * self.config.echo_spacing)
            .collect()
    }
}

// ---------------------------------------------------------------------------
// T2 Relaxation fitting
// ---------------------------------------------------------------------------

/// Fit a single-exponential T2 decay `S(t) = S₀·exp(−t/T₂)` to a set of
/// amplitude samples using linear regression on `ln(S)` vs `t`.
///
/// # Arguments
/// * `amplitudes` – Echo amplitudes (must be positive).
/// * `echo_spacing` – Time between samples (s).
///
/// # Returns
/// `Some((T2, S0))` — relaxation time and initial amplitude, or `None` on error.
pub fn fit_single_exponential_t2(
    amplitudes: &[f64],
    echo_spacing: f64,
) -> Option<(f64, f64)> {
    let n = amplitudes.len();
    if n < 2 {
        return None;
    }
    // Keep only positive amplitudes
    let valid: Vec<(f64, f64)> = amplitudes
        .iter()
        .enumerate()
        .filter(|(_, &a)| a > 0.0)
        .map(|(i, &a)| ((i as f64 + 1.0) * echo_spacing, ln(a)))
        .collect();
    if valid.len() < 2 {
        return None;
    }
    // Linear regression: y = slope*t + intercept, slope = -1/T2
    let m = valid.len() as f64;
    let sum_t: f64 = valid.iter().map(|(t, _)| t).sum();
    let sum_y: f64 = valid.iter().map(|(_, y)| y).sum();
    let sum_tt: f64 = valid.iter().map(|(t, _)| t * t).sum();
    let sum_ty: f64 = valid.iter().map(|(t, y)| t * y).sum();
    let denom = m * sum_tt - sum_t * sum_t;
    if denom.abs() < 1e-20 {
        return None;
    }
    let slope = (m * sum_ty - sum_t * sum_y) / denom;
    let intercept = (sum_y - slope * sum_t) / m;
    if slope >= 0.0 {
        return None; // non-decaying — cannot fit
    }
    let t2 = -1.0 / slope;
    let s0 = intercept.exp();
    Some((t2, s0))
}

/// Result of a multi-exponential T2 fit.
#[derive(Debug, Clone)]
pub struct MultiExpT2Result {
    /// Amplitudes for each T2 component (arbitrary units).
    pub amplitudes: Vec<f64>,
    /// T2 values for each component (s).
    pub t2_values: Vec<f64>,
    /// Residual sum of squares.
    pub rss: f64,
}

/// Fit a bi-exponential T2 decay to echo data.
///
/// `S(t) = A₁·exp(−t/T₂₁) + A₂·exp(−t/T₂₂)`
///
/// Performs a grid search over the two T2 values then refines using the
/// known-T2 linear least-squares for the amplitudes.
///
/// # Arguments
/// * `amplitudes`   – Measured echo amplitudes.
/// * `echo_spacing` – Time between echoes (s).
/// * `t2_grid`      – Candidate T2 values to search over (s).
pub fn fit_bi_exponential_t2(
    amplitudes: &[f64],
    echo_spacing: f64,
    t2_grid: &[f64],
) -> Option<MultiExpT2Result> {
    let n = amplitudes.len();
    if n < 4 || t2_grid.len() < 2 {
        return None;
    }
    let times: Vec<f64> = (1..=n)
        .map(|i| i as f64 * echo_spacing)
        .collect();

    let mut best_rss = f64::INFINITY;
    let mut best = None;

    for (i, &t2a) in t2_grid.iter().enumerate() {
        for &t2b in t2_grid.iter().skip(i + 1) {
            // Build design matrix columns
            let col_a: Vec<f64> = times.iter().map(|&t| exp(-t / t2a)).collect();
            let col_b: Vec<f64> = times.iter().map(|&t| exp(-t / t2b)).collect();
            // Normal equations for [A1, A2]
            let aaa: f64 = col_a.iter().map(|&v| v * v).sum();
            let aab: f64 = col_a.iter().zip(col_b.iter()).map(|(&a, &b)| a * b).sum();
            let abb: f64 = col_b.iter().map(|&v| v * v).sum();
            let rhs_a: f64 = col_a.iter().zip(amplitudes.iter()).map(|(&a, &s)| a * s).sum();
            let rhs_b: f64 = col_b.iter().zip(amplitudes.iter()).map(|(&b, &s)| b * s).sum();
            let mat = vec![vec![aaa, aab], vec![aab, abb]];
            let rhs = vec![rhs_a, rhs_b];
            if let Some(coeffs) = gauss_eliminate(mat, rhs) {
                let a1 = coeffs[0].max(0.0);
                let a2 = coeffs[1].max(0.0);
                let rss: f64 = amplitudes
                    .iter()
                    .zip(times.iter())
                    .map(|(&s, &t)| {
                        let pred = a1 * exp(-t / t2a) + a2 * exp(-t / t2b);
                        (s - pred).powi(2)
                    })
                    .sum();
                if rss < best_rss {
                    best_rss = rss;
                    best = Some(MultiExpT2Result {
                        amplitudes: vec![a1, a2],
                        t2_values: vec![t2a, t2b],
                        rss,
                    });
                }
            }
        }
    }
    best
}

// ---------------------------------------------------------------------------
// T2 Distribution via regularised NNLS (Tikhonov)
// ---------------------------------------------------------------------------

/// Result of a T2 distribution inversion.
#[derive(Debug, Clone)]
pub struct T2Distribution {
    /// T2 axis values (s).
    pub t2_axis: Vec<f64>,
    /// Amplitude at each T2 (arbitrary units, ≥ 0).
    pub amplitudes: Vec<f64>,
    /// Regularisation parameter used.
    pub lambda: f64,
}

/// Compute the T2 distribution from a CPMG echo train using Tikhonov-regularised
/// non-negative least squares (projected gradient NNLS).
///
/// Builds a basis matrix `K[i,j] = exp(−t_i / T2_j)` and solves
/// `min ||K·x − s||² + λ||x||²` subject to `x ≥ 0`.
///
/// # Arguments
/// * `echo_amplitudes` – Measured CPMG echo amplitudes.
/// * `echo_spacing`    – Inter-echo time (s).
/// * `t2_axis`         – Candidate T2 values for the distribution grid.
/// * `lambda`          – Tikhonov regularisation parameter (try 0.01–1.0).
pub fn compute_t2_distribution(
    echo_amplitudes: &[f64],
    echo_spacing: f64,
    t2_axis: &[f64],
    lambda: f64,
) -> T2Distribution {
    let n_echoes = echo_amplitudes.len();
    let n_t2 = t2_axis.len();
    let times: Vec<f64> = (1..=n_echoes).map(|i| i as f64 * echo_spacing).collect();

    // Build K matrix (n_echoes × n_t2)
    let mut k = vec![vec![0.0f64; n_t2]; n_echoes];
    for (i, &t) in times.iter().enumerate() {
        for (j, &t2) in t2_axis.iter().enumerate() {
            k[i][j] = exp(-t / t2);
        }
    }

    // Form Gram matrix G = Kᵀ K + λ I  (n_t2 × n_t2)
    let mut g = vec![vec![0.0f64; n_t2]; n_t2];
    for a in 0..n_t2 {
        for b in 0..n_t2 {
            let mut v = 0.0;
            for i in 0..n_echoes {
                v += k[i][a] * k[i][b];
            }
            g[a][b] = v;
        }
        g[a][a] += lambda;
    }

    // Form RHS = Kᵀ s
    let mut rhs = vec![0.0f64; n_t2];
    for b in 0..n_t2 {
        let mut v = 0.0;
        for i in 0..n_echoes {
            v += k[i][b] * echo_amplitudes[i];
        }
        rhs[b] = v;
    }

    // Projected gradient NNLS: iterate x ← max(0, x − step*(G x − rhs))
    // Step size = 1 / L where L is the largest eigenvalue of G (estimated by
    // the Frobenius norm as a safe upper bound).
    let mut x = vec![0.0f64; n_t2];
    let g_frob_sq: f64 = g.iter().flat_map(|row| row.iter()).map(|&v| v * v).sum();
    let step = if g_frob_sq > 0.0 { 0.5 / g_frob_sq.sqrt() } else { 1e-3 };
    for _iter in 0..10000 {
        // gradient = G x − rhs
        let mut grad = vec![0.0f64; n_t2];
        for a in 0..n_t2 {
            let mut gx = 0.0;
            for b in 0..n_t2 {
                gx += g[a][b] * x[b];
            }
            grad[a] = gx - rhs[a];
        }
        let mut changed = false;
        for a in 0..n_t2 {
            let new_x = (x[a] - step * grad[a]).max(0.0);
            if (new_x - x[a]).abs() > 1e-14 {
                changed = true;
            }
            x[a] = new_x;
        }
        if !changed {
            break;
        }
    }

    T2Distribution {
        t2_axis: t2_axis.to_vec(),
        amplitudes: x,
        lambda,
    }
}

// ---------------------------------------------------------------------------
// T1 Inversion Recovery
// ---------------------------------------------------------------------------

/// Configuration for inversion-recovery T1 measurement.
#[derive(Debug, Clone)]
pub struct InversionRecoveryConfig {
    /// Inversion times τ to sample (s).
    pub inversion_times: Vec<f64>,
}

/// Synthesise inversion-recovery signal.
///
/// `S(τ) = M₀ · (1 − 2·exp(−τ/T₁))`
///
/// # Arguments
/// * `config` – Inversion recovery configuration.
/// * `m0`     – Equilibrium magnetisation.
/// * `t1`     – Spin-lattice relaxation time (s).
pub fn inversion_recovery_signal(
    config: &InversionRecoveryConfig,
    m0: f64,
    t1: f64,
) -> Vec<f64> {
    config
        .inversion_times
        .iter()
        .map(|&tau| m0 * (1.0 - 2.0 * exp(-tau / t1)))
        .collect()
}

/// Fit T1 from inversion-recovery magnitude data using nonlinear least squares
/// via a golden-section search on a logarithmic T1 grid.
///
/// Fits `|S(τ)| = M₀ · |1 − 2·exp(−τ/T₁)|` (magnitude mode).
///
/// The search is performed on `log10(T1)` to handle the wide dynamic range of
/// T1 (milliseconds to seconds) without bias.
///
/// # Arguments
/// * `config`     – Inversion recovery configuration.
/// * `magnitudes` – Measured |S(τ)| values (must match `config.inversion_times`).
///
/// # Returns
/// `Some((T1, M0))` or `None` if fitting fails.
pub fn fit_t1_inversion_recovery(
    config: &InversionRecoveryConfig,
    magnitudes: &[f64],
) -> Option<(f64, f64)> {
    let taus = &config.inversion_times;
    if taus.len() != magnitudes.len() || taus.len() < 3 {
        return None;
    }

    // Estimate a reasonable T1 search range from the inversion times.
    // The null crossing occurs at τ_null = T1 * ln(2), so T1 = τ_null / ln(2).
    // We search from 0.1 * min_tau to 10 * max_tau.
    let tau_min = taus.iter().cloned().fold(f64::INFINITY, f64::min);
    let tau_max = taus.iter().cloned().fold(0.0_f64, f64::max);
    let log_lo = (tau_min * 0.1_f64).max(1e-8).log10();
    let log_hi = (tau_max * 10.0_f64).log10();

    let cost = |log_t1: f64| -> f64 {
        let t1 = 10.0_f64.powf(log_t1);
        // Estimate M0 analytically given T1 via linear least-squares
        let basis: Vec<f64> = taus
            .iter()
            .map(|&tau| (1.0 - 2.0 * exp(-tau / t1)).abs())
            .collect();
        let dot_mb: f64 = magnitudes.iter().zip(basis.iter()).map(|(m, b)| m * b).sum();
        let dot_bb: f64 = basis.iter().map(|b| b * b).sum();
        if dot_bb < 1e-30 {
            return f64::INFINITY;
        }
        let m0 = dot_mb / dot_bb;
        magnitudes
            .iter()
            .zip(taus.iter())
            .map(|(&meas, &tau)| {
                let pred = m0 * (1.0 - 2.0 * exp(-tau / t1)).abs();
                (meas - pred).powi(2)
            })
            .sum()
    };

    // Golden-section search on log10(T1)
    let phi = (5.0_f64.sqrt() - 1.0) / 2.0;
    let mut lo = log_lo;
    let mut hi = log_hi;
    for _ in 0..300 {
        let x1 = hi - phi * (hi - lo);
        let x2 = lo + phi * (hi - lo);
        if cost(x1) < cost(x2) {
            hi = x2;
        } else {
            lo = x1;
        }
    }
    let log_t1_best = (lo + hi) / 2.0;
    let t1_best = 10.0_f64.powf(log_t1_best);

    // Recompute M0
    let basis: Vec<f64> = taus
        .iter()
        .map(|&tau| (1.0 - 2.0 * exp(-tau / t1_best)).abs())
        .collect();
    let dot_mb: f64 = magnitudes.iter().zip(basis.iter()).map(|(m, b)| m * b).sum();
    let dot_bb: f64 = basis.iter().map(|b| b * b).sum();
    let m0_best = if dot_bb > 1e-30 { dot_mb / dot_bb } else { return None; };
    Some((t1_best, m0_best))
}

// ---------------------------------------------------------------------------
// Diffusion — Stejskal-Tanner
// ---------------------------------------------------------------------------

/// Configuration for a Stejskal-Tanner pulsed field-gradient spin echo (PGSE).
#[derive(Debug, Clone)]
pub struct StejalTannerConfig {
    /// Gradient magnitudes G (T/m) at which to acquire the signal.
    pub gradient_strengths: Vec<f64>,
    /// Gradient pulse duration δ (s).
    pub delta: f64,
    /// Diffusion time Δ (s) (interval between leading edges of gradient pulses).
    pub big_delta: f64,
    /// Gyromagnetic ratio γ (rad/s/T). Defaults to GAMMA_H if zero.
    pub gamma: f64,
}

/// Compute the Stejskal-Tanner b-value for each gradient strength.
///
/// `b = γ² G² δ² (Δ − δ/3)`
///
/// # Arguments
/// * `config` – PGSE configuration.
pub fn stejskal_tanner_b_values(config: &StejalTannerConfig) -> Vec<f64> {
    let gamma = if config.gamma > 0.0 { config.gamma } else { GAMMA_H };
    let factor = config.delta.powi(2) * (config.big_delta - config.delta / 3.0);
    config
        .gradient_strengths
        .iter()
        .map(|&g| gamma.powi(2) * g.powi(2) * factor)
        .collect()
}

/// Synthesise Stejskal-Tanner signal attenuation.
///
/// `S(b) = S₀ · exp(−b · D)`
///
/// # Arguments
/// * `config` – PGSE configuration.
/// * `s0`     – Signal without diffusion weighting.
/// * `d`      – Apparent diffusion coefficient (m²/s).
pub fn stejskal_tanner_signal(config: &StejalTannerConfig, s0: f64, d: f64) -> Vec<f64> {
    let b_values = stejskal_tanner_b_values(config);
    b_values.iter().map(|&b| s0 * exp(-b * d)).collect()
}

/// Extract the apparent diffusion coefficient (ADC) from Stejskal-Tanner data.
///
/// Performs linear regression on `ln(S/S₀)` vs `b`.
///
/// # Arguments
/// * `b_values`  – b-values in s/m².
/// * `signals`   – Measured signal at each b-value.
///
/// # Returns
/// `Some((D, S0))` — ADC in m²/s and zero-b signal, or `None` on failure.
pub fn fit_adc(b_values: &[f64], signals: &[f64]) -> Option<(f64, f64)> {
    if b_values.len() != signals.len() || signals.len() < 2 {
        return None;
    }
    let valid: Vec<(f64, f64)> = b_values
        .iter()
        .zip(signals.iter())
        .filter(|(_, &s)| s > 0.0)
        .map(|(&b, &s)| (b, ln(s)))
        .collect();
    if valid.len() < 2 {
        return None;
    }
    let m = valid.len() as f64;
    let sum_b: f64 = valid.iter().map(|(b, _)| b).sum();
    let sum_y: f64 = valid.iter().map(|(_, y)| y).sum();
    let sum_bb: f64 = valid.iter().map(|(b, _)| b * b).sum();
    let sum_by: f64 = valid.iter().map(|(b, y)| b * y).sum();
    let denom = m * sum_bb - sum_b * sum_b;
    if denom.abs() < 1e-30 {
        return None;
    }
    let slope = (m * sum_by - sum_b * sum_y) / denom;
    let intercept = (sum_y - slope * sum_b) / m;
    if slope > 0.0 {
        return None;
    }
    let d = -slope;
    let s0 = intercept.exp();
    Some((d, s0))
}

// ---------------------------------------------------------------------------
// Phase correction
// ---------------------------------------------------------------------------

/// Apply zero-order phase correction to complex NMR data.
///
/// Multiplies each sample by `exp(i·φ₀)`, rotating the entire spectrum.
///
/// # Arguments
/// * `real`    – Real (I) component vector.
/// * `imag`    – Imaginary (Q) component vector.
/// * `phi0`    – Zero-order phase correction angle (radians).
///
/// # Returns
/// `(real_corrected, imag_corrected)`
pub fn phase_correct_zero_order(
    real: &[f64],
    imag: &[f64],
    phi0: f64,
) -> (Vec<f64>, Vec<f64>) {
    let cos_phi = phi0.cos();
    let sin_phi = phi0.sin();
    let r: Vec<f64> = real
        .iter()
        .zip(imag.iter())
        .map(|(&re, &im)| re * cos_phi - im * sin_phi)
        .collect();
    let i: Vec<f64> = real
        .iter()
        .zip(imag.iter())
        .map(|(&re, &im)| re * sin_phi + im * cos_phi)
        .collect();
    (r, i)
}

/// Apply first-order phase correction.
///
/// The phase ramp `φ(k) = φ₀ + φ₁ · k / N` is applied across N samples.
///
/// # Arguments
/// * `real`  – Real component.
/// * `imag`  – Imaginary component.
/// * `phi0`  – Zero-order phase (rad).
/// * `phi1`  – First-order phase slope (rad, total across spectrum).
///
/// # Returns
/// `(real_corrected, imag_corrected)`
pub fn phase_correct_first_order(
    real: &[f64],
    imag: &[f64],
    phi0: f64,
    phi1: f64,
) -> (Vec<f64>, Vec<f64>) {
    let n = real.len() as f64;
    let r: Vec<f64> = real
        .iter()
        .zip(imag.iter())
        .enumerate()
        .map(|(k, (&re, &im))| {
            let phi = phi0 + phi1 * k as f64 / n;
            re * phi.cos() - im * phi.sin()
        })
        .collect();
    let i: Vec<f64> = real
        .iter()
        .zip(imag.iter())
        .enumerate()
        .map(|(k, (&re, &im))| {
            let phi = phi0 + phi1 * k as f64 / n;
            re * phi.sin() + im * phi.cos()
        })
        .collect();
    (r, i)
}

/// Estimate zero-order phase correction from the first echo by maximising
/// the real component integral.
///
/// Returns the phase angle (radians) that rotates the peak into the real
/// channel.
pub fn estimate_zero_order_phase(real: &[f64], imag: &[f64]) -> f64 {
    // Simple approach: use atan2 of the sum of I and Q over the peak region
    let sum_re: f64 = real.iter().sum();
    let sum_im: f64 = imag.iter().sum();
    -sum_im.atan2(sum_re)
}

// ---------------------------------------------------------------------------
// Echo integration / SNR
// ---------------------------------------------------------------------------

/// Integrate an echo by summing magnitude samples in a central window.
///
/// # Arguments
/// * `echo` – Magnitude echo samples.
/// * `half_width` – Number of samples each side of the centre to include.
pub fn echo_peak_integral(echo: &[f64], half_width: usize) -> f64 {
    let centre = echo.len() / 2;
    let start = centre.saturating_sub(half_width);
    let end = (centre + half_width + 1).min(echo.len());
    echo[start..end].iter().sum()
}

/// Estimate noise standard deviation from the tail of an echo signal
/// (last 20 % of samples).
pub fn estimate_echo_noise(echo: &[f64]) -> f64 {
    let tail_start = echo.len() * 4 / 5;
    let tail = &echo[tail_start..];
    if tail.len() < 2 {
        return 0.0;
    }
    let mean: f64 = tail.iter().sum::<f64>() / tail.len() as f64;
    let var: f64 = tail.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / (tail.len() - 1) as f64;
    var.sqrt()
}

/// Compute SNR of an echo as `peak / noise_std`.
pub fn echo_snr(echo: &[f64]) -> f64 {
    let peak = echo.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let noise = estimate_echo_noise(echo);
    if noise < 1e-15 {
        f64::INFINITY
    } else {
        peak / noise
    }
}

// ---------------------------------------------------------------------------
// T2* (FID decay)
// ---------------------------------------------------------------------------

/// Generate a Free Induction Decay (FID) envelope decaying at rate 1/T2*.
///
/// `FID(t) = M₀ · exp(−t / T₂*) · cos(2π·Δf·t + φ₀)`
///
/// # Arguments
/// * `m0`          – Initial magnetisation.
/// * `t2_star`     – Effective relaxation time (s).
/// * `delta_f`     – Off-resonance frequency (Hz).
/// * `phi0`        – Initial phase (rad).
/// * `sample_rate` – Sample rate (Hz).
/// * `n_samples`   – Number of samples.
pub fn fid_signal(
    m0: f64,
    t2_star: f64,
    delta_f: f64,
    phi0: f64,
    sample_rate: f64,
    n_samples: usize,
) -> Vec<f64> {
    let dt = 1.0 / sample_rate;
    (0..n_samples)
        .map(|k| {
            let t = k as f64 * dt;
            m0 * exp(-t / t2_star) * (2.0 * PI * delta_f * t + phi0).cos()
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Material database
// ---------------------------------------------------------------------------

/// NMR relaxation parameters for a known material.
#[derive(Debug, Clone)]
pub struct NmrMaterial {
    /// Material name.
    pub name: &'static str,
    /// Spin-lattice relaxation time T1 (s).
    pub t1: f64,
    /// Spin-spin relaxation time T2 (s).
    pub t2: f64,
    /// Self-diffusion coefficient D (m²/s).
    pub diffusion: f64,
    /// Proton density (relative, water = 1.0).
    pub proton_density: f64,
}

/// Built-in NMR material database.
pub const NMR_MATERIALS: &[NmrMaterial] = &[
    NmrMaterial {
        name: "Water (25°C)",
        t1: 3.0,
        t2: 3.0,
        diffusion: 2.3e-9,
        proton_density: 1.0,
    },
    NmrMaterial {
        name: "Mineral Oil",
        t1: 0.3,
        t2: 0.1,
        diffusion: 1.0e-11,
        proton_density: 0.85,
    },
    NmrMaterial {
        name: "Rubber",
        t1: 0.5,
        t2: 0.01,
        diffusion: 1.0e-13,
        proton_density: 0.7,
    },
    NmrMaterial {
        name: "Brain White Matter",
        t1: 0.8,
        t2: 0.08,
        diffusion: 0.7e-9,
        proton_density: 0.7,
    },
    NmrMaterial {
        name: "Brain Gray Matter",
        t1: 1.3,
        t2: 0.1,
        diffusion: 1.0e-9,
        proton_density: 0.85,
    },
    NmrMaterial {
        name: "Cerebrospinal Fluid",
        t1: 4.0,
        t2: 2.0,
        diffusion: 3.0e-9,
        proton_density: 1.0,
    },
    NmrMaterial {
        name: "Liver",
        t1: 0.8,
        t2: 0.04,
        diffusion: 1.0e-9,
        proton_density: 0.72,
    },
    NmrMaterial {
        name: "Muscle",
        t1: 1.0,
        t2: 0.03,
        diffusion: 1.3e-9,
        proton_density: 0.79,
    },
];

/// Look up an NMR material by name (case-insensitive substring match).
pub fn lookup_material(query: &str) -> Option<&'static NmrMaterial> {
    let q = query.to_lowercase();
    NMR_MATERIALS.iter().find(|m| m.name.to_lowercase().contains(&q))
}

// ---------------------------------------------------------------------------
// Well logging — NMR porosity and permeability
// ---------------------------------------------------------------------------

/// NMR well-logging porosity from total signal amplitude.
///
/// `φ = S_total / S_water`
///
/// where `S_water` is the signal from a 100 % water-saturated reference.
///
/// # Arguments
/// * `total_signal`  – Integrated echo-train signal from the formation.
/// * `water_signal`  – Reference signal for 100 % porosity calibration.
pub fn nmr_porosity(total_signal: f64, water_signal: f64) -> f64 {
    if water_signal <= 0.0 {
        return 0.0;
    }
    (total_signal / water_signal).min(1.0).max(0.0)
}

/// Partition T2 distribution into Free Fluid Index (FFI) and Bulk Volume
/// Irreducible (BVI) using the standard T2 cut-off (33 ms for carbonates,
/// 3 ms for sandstones typically).
///
/// # Arguments
/// * `dist`      – T2 distribution.
/// * `t2_cutoff` – T2 cut-off time (s) separating mobile from bound fluid.
///
/// # Returns
/// `(FFI, BVI)` as fractions of total signal.
pub fn partition_ffi_bvi(dist: &T2Distribution, t2_cutoff: f64) -> (f64, f64) {
    let total: f64 = dist.amplitudes.iter().sum();
    if total <= 0.0 {
        return (0.0, 0.0);
    }
    let ffi: f64 = dist
        .t2_axis
        .iter()
        .zip(dist.amplitudes.iter())
        .filter(|(&t2, _)| t2 > t2_cutoff)
        .map(|(_, &a)| a)
        .sum();
    let bvi = total - ffi;
    (ffi / total, bvi / total)
}

/// Estimate permeability using the Timur-Coates model.
///
/// `k = (φ / C)⁴ · (FFI / BVI)²`  (units: mD if φ in p.u.)
///
/// Typical C ≈ 10 for sandstones.
///
/// # Arguments
/// * `porosity` – NMR porosity (0–1 fraction).
/// * `ffi`      – Free fluid index fraction.
/// * `bvi`      – Bound volume irreducible fraction.
/// * `c`        – Timur-Coates constant.
pub fn timur_coates_permeability(porosity: f64, ffi: f64, bvi: f64, c: f64) -> f64 {
    if bvi <= 0.0 || c <= 0.0 {
        return 0.0;
    }
    (porosity / c).powi(4) * (ffi / bvi).powi(2)
}

/// T2 cut-off thresholds for fluid typing in NMR well logging.
///
/// # Returns
/// `(gas_cutoff, oil_cutoff, water_cutoff)` in seconds.
pub fn fluid_typing_cutoffs() -> (f64, f64, f64) {
    (0.003, 0.033, 0.1) // gas < 3 ms, oil 3–33 ms, water > 33 ms
}

// ---------------------------------------------------------------------------
// T2/T1 ratio
// ---------------------------------------------------------------------------

/// Compute the T2/T1 ratio, which indicates molecular mobility.
///
/// Values near 1 suggest fast molecular motion (liquid); values << 1 indicate
/// restricted motion (solid-like or macromolecular systems).
pub fn t2_t1_ratio(t2: f64, t1: f64) -> f64 {
    t2 / t1
}

// ---------------------------------------------------------------------------
// Relaxation time utilities
// ---------------------------------------------------------------------------

/// Convert CPMG echo-train SNR to minimum detectable T2.
///
/// Rule of thumb: need at least ~5 echoes above noise floor to fit T2.
/// Returns the minimum T2 resolvable given the echo spacing.
///
/// # Arguments
/// * `echo_spacing` – CPMG echo spacing TE (s).
/// * `n_echoes`     – Total number of echoes acquired.
pub fn minimum_resolvable_t2(echo_spacing: f64, n_echoes: usize) -> f64 {
    // Need at least 5 echoes; T2 ~ 5 * TE / ln(SNR_needed)
    let t_total = echo_spacing * n_echoes as f64;
    t_total / 5.0 // rough lower bound
}

/// Estimate maximum resolvable T2 given total acquisition time.
pub fn maximum_resolvable_t2(echo_spacing: f64, n_echoes: usize) -> f64 {
    echo_spacing * n_echoes as f64 * 2.0
}

// ---------------------------------------------------------------------------
// Full CPMG pipeline helper
// ---------------------------------------------------------------------------

/// High-level result from processing a complete CPMG experiment.
#[derive(Debug, Clone)]
pub struct CpmgResult {
    /// Fitted T2 (s).
    pub t2: f64,
    /// Fitted initial magnetisation M0.
    pub m0: f64,
    /// Echo train SNR (ratio, not dB).
    pub snr: f64,
    /// T2 distribution (if requested).
    pub t2_distribution: Option<T2Distribution>,
}

/// Process a CPMG echo-train amplitude vector end-to-end.
///
/// Performs single-exponential T2 fitting and optionally a T2 distribution
/// inversion.
///
/// # Arguments
/// * `config`        – CPMG configuration.
/// * `echo_amps`     – Measured echo amplitudes (length = `config.n_echoes`).
/// * `t2_axis`       – If `Some`, compute T2 distribution on this grid.
/// * `lambda`        – Regularisation parameter for T2 distribution.
pub fn process_cpmg(
    config: &CpmgConfig,
    echo_amps: &[f64],
    t2_axis: Option<&[f64]>,
    lambda: f64,
) -> Option<CpmgResult> {
    let (t2, m0) = fit_single_exponential_t2(echo_amps, config.echo_spacing)?;
    // SNR: first echo amplitude vs noise in last 10 % of echoes
    let tail_start = echo_amps.len() * 9 / 10;
    let tail = &echo_amps[tail_start..];
    let noise = if tail.len() > 1 {
        let mean: f64 = tail.iter().sum::<f64>() / tail.len() as f64;
        let var: f64 = tail.iter().map(|&x| (x - mean).powi(2)).sum::<f64>()
            / (tail.len() - 1) as f64;
        var.sqrt()
    } else {
        1e-15
    };
    let snr = if noise < 1e-15 {
        f64::INFINITY
    } else {
        echo_amps[0] / noise
    };
    let t2_distribution = t2_axis.map(|ax| {
        compute_t2_distribution(echo_amps, config.echo_spacing, ax, lambda)
    });
    Some(CpmgResult {
        t2,
        m0,
        snr,
        t2_distribution,
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Tolerance helpers
    fn rel_err(measured: f64, expected: f64) -> f64 {
        ((measured - expected) / expected).abs()
    }

    // --- Hahn Echo ---

    #[test]
    fn test_hahn_echo_amplitude_decay() {
        let t2 = 0.1; // 100 ms
        let m0 = 1.0;
        let tau1 = 0.01;
        let tau2 = 0.05;
        let a1 = hahn_echo_amplitude(m0, tau1, t2);
        let a2 = hahn_echo_amplitude(m0, tau2, t2);
        assert!(a1 > a2, "Later echo should be smaller");
        assert!(rel_err(a1, m0 * (-2.0 * tau1 / t2).exp()) < 1e-10);
        assert!(rel_err(a2, m0 * (-2.0 * tau2 / t2).exp()) < 1e-10);
    }

    #[test]
    fn test_hahn_echo_amplitude_zero_tau() {
        let a = hahn_echo_amplitude(2.5, 0.0, 0.1);
        assert!((a - 2.5).abs() < 1e-12, "At τ=0 amplitude should equal M0");
    }

    #[test]
    fn test_hahn_echo_signal_length() {
        let config = HahnEchoConfig {
            larmor_freq: 1.0e6,
            sample_rate: 1.0e6,
            tau: 0.01,
            samples_per_echo: 128,
        };
        let sig = hahn_echo_signal(&config, 1.0, 0.1, 0.005);
        assert_eq!(sig.len(), 128);
    }

    #[test]
    fn test_hahn_echo_signal_peak_at_centre() {
        let config = HahnEchoConfig {
            larmor_freq: 1.0e6,
            sample_rate: 1.0e6,
            tau: 0.05,
            samples_per_echo: 64,
        };
        let sig = hahn_echo_signal(&config, 1.0, 0.5, 0.001);
        let peak_idx = sig
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        // Peak should be near centre ± 2 samples
        assert!((peak_idx as i64 - 32).abs() <= 2);
    }

    #[test]
    fn test_hahn_echo_amplitude_with_known_t2() {
        // T2 = 50 ms, tau = 10 ms → amplitude = exp(-0.4) ≈ 0.6703
        let a = hahn_echo_amplitude(1.0, 0.010, 0.050);
        let expected = (-0.4_f64).exp();
        assert!(rel_err(a, expected) < 1e-12);
    }

    // --- CPMG ---

    #[test]
    fn test_cpmg_echo_train_length() {
        let config = CpmgConfig {
            n_echoes: 32,
            echo_spacing: 2e-3,
            larmor_freq: 2.0e6,
            sample_rate: 10e3,
            samples_per_echo: 64,
        };
        let proc = CpmgProcessor::new(config);
        let train = proc.synthesize_echo_train(1.0, 0.1);
        assert_eq!(train.len(), 32);
    }

    #[test]
    fn test_cpmg_echo_train_monotonic_decay() {
        let config = CpmgConfig {
            n_echoes: 16,
            echo_spacing: 5e-3,
            larmor_freq: 2.0e6,
            sample_rate: 10e3,
            samples_per_echo: 64,
        };
        let proc = CpmgProcessor::new(config);
        let train = proc.synthesize_echo_train(1.0, 0.05);
        for i in 1..train.len() {
            assert!(
                train[i] <= train[i - 1] + 1e-12,
                "Echo train must be monotonically non-increasing"
            );
        }
    }

    #[test]
    fn test_cpmg_echo_times() {
        let config = CpmgConfig {
            n_echoes: 4,
            echo_spacing: 10e-3,
            larmor_freq: 2.0e6,
            sample_rate: 10e3,
            samples_per_echo: 64,
        };
        let proc = CpmgProcessor::new(config.clone());
        let times = proc.echo_times();
        assert_eq!(times.len(), 4);
        for (i, &t) in times.iter().enumerate() {
            let expected = (i + 1) as f64 * config.echo_spacing;
            assert!((t - expected).abs() < 1e-12);
        }
    }

    #[test]
    fn test_cpmg_first_echo_amplitude() {
        // First echo at t = TE: A = M0 * exp(-TE/T2)
        let te = 2e-3;
        let t2 = 0.1;
        let m0 = 5.0;
        let config = CpmgConfig {
            n_echoes: 8,
            echo_spacing: te,
            larmor_freq: 2.0e6,
            sample_rate: 10e3,
            samples_per_echo: 64,
        };
        let proc = CpmgProcessor::new(config);
        let train = proc.synthesize_echo_train(m0, t2);
        let expected = m0 * (-te / t2).exp();
        assert!(rel_err(train[0], expected) < 1e-12);
    }

    // --- T2 Fitting ---

    #[test]
    fn test_fit_t2_perfect_data() {
        let t2_true = 0.08;
        let m0_true = 1.0;
        let te = 2e-3;
        let n = 64;
        let amps: Vec<f64> = (1..=n).map(|i| m0_true * (-(i as f64) * te / t2_true).exp()).collect();
        let (t2_fit, m0_fit) = fit_single_exponential_t2(&amps, te).unwrap();
        assert!(rel_err(t2_fit, t2_true) < 0.01, "T2 fit error: {}", rel_err(t2_fit, t2_true));
        assert!(rel_err(m0_fit, m0_true) < 0.01);
    }

    #[test]
    fn test_fit_t2_long_relaxation() {
        let t2_true = 2.0; // water-like
        let te = 0.05;
        let n = 128;
        let amps: Vec<f64> = (1..=n).map(|i| (-(i as f64) * te / t2_true).exp()).collect();
        let (t2_fit, _) = fit_single_exponential_t2(&amps, te).unwrap();
        assert!(rel_err(t2_fit, t2_true) < 0.05);
    }

    #[test]
    fn test_fit_t2_too_few_points() {
        let result = fit_single_exponential_t2(&[1.0], 0.01);
        assert!(result.is_none());
    }

    #[test]
    fn test_fit_t2_negative_amplitudes_filtered() {
        // Mix of positive and some near-zero values
        let amps = vec![1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3];
        let result = fit_single_exponential_t2(&amps, 0.01);
        assert!(result.is_some());
    }

    #[test]
    fn test_cpmg_fit_t2_method() {
        let t2 = 0.05;
        let te = 2e-3;
        let config = CpmgConfig {
            n_echoes: 64,
            echo_spacing: te,
            larmor_freq: 2.0e6,
            sample_rate: 10e3,
            samples_per_echo: 64,
        };
        let proc = CpmgProcessor::new(config);
        let train = proc.synthesize_echo_train(1.0, t2);
        let (t2_fit, _) = proc.fit_t2(&train).unwrap();
        assert!(rel_err(t2_fit, t2) < 0.01);
    }

    // --- Multi-exponential T2 fitting ---

    #[test]
    fn test_bi_exponential_t2_fit() {
        let t2a = 0.01;
        let t2b = 0.1;
        let a1 = 0.6;
        let a2 = 0.4;
        let te = 1e-3;
        let n = 128;
        let amps: Vec<f64> = (1..=n)
            .map(|i| {
                let t = i as f64 * te;
                a1 * (-t / t2a).exp() + a2 * (-t / t2b).exp()
            })
            .collect();
        let grid: Vec<f64> = (1..=20).map(|i| i as f64 * 0.01).collect();
        let result = fit_bi_exponential_t2(&amps, te, &grid);
        assert!(result.is_some());
        let r = result.unwrap();
        assert_eq!(r.t2_values.len(), 2);
        assert!(r.rss < 0.1);
    }

    #[test]
    fn test_bi_exponential_insufficient_data() {
        let result = fit_bi_exponential_t2(&[1.0, 0.9], 0.01, &[0.1, 0.5]);
        assert!(result.is_none());
    }

    // --- T1 Inversion Recovery ---

    #[test]
    fn test_inversion_recovery_signal_shape() {
        let t1 = 1.0;
        let m0 = 1.0;
        let config = InversionRecoveryConfig {
            inversion_times: vec![0.1, 0.5, 0.693, 1.0, 2.0, 5.0],
        };
        let sig = inversion_recovery_signal(&config, m0, t1);
        // At τ = T1 * ln(2) ≈ 0.693 the signal crosses zero
        assert!(sig[0] < 0.0, "Short τ signal should be negative (inverted)");
        assert!(sig[5] > 0.9, "Long τ signal should approach M0");
    }

    #[test]
    fn test_inversion_recovery_null_at_ln2_t1() {
        let t1 = 0.5;
        let tau_null = t1 * 2.0_f64.ln();
        let config = InversionRecoveryConfig {
            inversion_times: vec![tau_null],
        };
        let sig = inversion_recovery_signal(&config, 1.0, t1);
        assert!(sig[0].abs() < 0.01, "Signal should be near zero at T1*ln2");
    }

    #[test]
    fn test_fit_t1_inversion_recovery() {
        let t1_true = 0.5;
        let m0_true = 1.0;
        let taus = vec![0.01, 0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0, 2.0, 3.0];
        let config = InversionRecoveryConfig {
            inversion_times: taus.clone(),
        };
        let sigs = inversion_recovery_signal(&config, m0_true, t1_true);
        let mags: Vec<f64> = sigs.iter().map(|s| s.abs()).collect();
        let (t1_fit, _m0_fit) = fit_t1_inversion_recovery(&config, &mags).unwrap();
        assert!(
            rel_err(t1_fit, t1_true) < 0.05,
            "T1 fit error too large: measured={t1_fit:.4} expected={t1_true}"
        );
    }

    #[test]
    fn test_fit_t1_too_few_points() {
        let config = InversionRecoveryConfig {
            inversion_times: vec![0.1, 0.5],
        };
        let result = fit_t1_inversion_recovery(&config, &[0.8, 0.9]);
        assert!(result.is_none());
    }

    // --- Stejskal-Tanner / Diffusion ---

    #[test]
    fn test_b_value_calculation() {
        // For G = 0.01 T/m, δ = 5 ms, Δ = 20 ms
        let config = StejalTannerConfig {
            gradient_strengths: vec![0.0, 0.01, 0.05],
            delta: 5e-3,
            big_delta: 20e-3,
            gamma: GAMMA_H,
        };
        let b = stejskal_tanner_b_values(&config);
        assert_eq!(b.len(), 3);
        assert!((b[0]).abs() < 1e-6, "b=0 for G=0");
        assert!(b[1] > 0.0 && b[2] > b[1]);
    }

    #[test]
    fn test_stejskal_tanner_signal_decay() {
        let d = 2.3e-9; // water
        let config = StejalTannerConfig {
            gradient_strengths: vec![0.0, 0.01, 0.02, 0.05],
            delta: 5e-3,
            big_delta: 20e-3,
            gamma: GAMMA_H,
        };
        let sigs = stejskal_tanner_signal(&config, 1.0, d);
        assert_eq!(sigs[0], 1.0);
        for i in 1..sigs.len() {
            assert!(sigs[i] < sigs[i - 1], "Signal must decay with b");
        }
    }

    #[test]
    fn test_adc_extraction_water() {
        let d_true = 2.3e-9;
        let config = StejalTannerConfig {
            gradient_strengths: vec![0.0, 0.005, 0.01, 0.02, 0.04, 0.08],
            delta: 5e-3,
            big_delta: 25e-3,
            gamma: GAMMA_H,
        };
        let b = stejskal_tanner_b_values(&config);
        let sigs = stejskal_tanner_signal(&config, 1.0, d_true);
        let (d_fit, s0_fit) = fit_adc(&b, &sigs).unwrap();
        assert!(rel_err(d_fit, d_true) < 0.01, "ADC fit error: {}", rel_err(d_fit, d_true));
        assert!((s0_fit - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_adc_insufficient_data() {
        let result = fit_adc(&[0.0], &[1.0]);
        assert!(result.is_none());
    }

    #[test]
    fn test_b_value_zero_gradient() {
        let config = StejalTannerConfig {
            gradient_strengths: vec![0.0],
            delta: 5e-3,
            big_delta: 20e-3,
            gamma: GAMMA_H,
        };
        let b = stejskal_tanner_b_values(&config);
        assert!(b[0].abs() < 1e-30);
    }

    // --- Phase Correction ---

    #[test]
    fn test_zero_order_phase_correction_identity() {
        let re = vec![1.0, 2.0, 3.0];
        let im = vec![0.0, 0.0, 0.0];
        let (r, i) = phase_correct_zero_order(&re, &im, 0.0);
        for (a, b) in r.iter().zip(re.iter()) {
            assert!((a - b).abs() < 1e-12);
        }
        for &v in &i {
            assert!(v.abs() < 1e-12);
        }
    }

    #[test]
    fn test_zero_order_phase_correction_90deg() {
        // Rotating a real signal by +90° should move energy into Q channel
        let re = vec![1.0, 0.0];
        let im = vec![0.0, 0.0];
        let (r, i) = phase_correct_zero_order(&re, &im, PI / 2.0);
        assert!(r[0].abs() < 1e-12);
        assert!((i[0] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_first_order_phase_correction() {
        let re = vec![1.0; 8];
        let im = vec![0.0; 8];
        let (r, i) = phase_correct_first_order(&re, &im, 0.0, 0.0);
        for (a, b) in r.iter().zip(re.iter()) {
            assert!((a - b).abs() < 1e-12);
        }
        for &v in &i {
            assert!(v.abs() < 1e-12);
        }
    }

    #[test]
    fn test_estimate_zero_order_phase() {
        // A purely imaginary signal should return -π/2
        let re = vec![0.0; 8];
        let im = vec![1.0; 8];
        let phi = estimate_zero_order_phase(&re, &im);
        // After correction, signal should be real
        let (r, _) = phase_correct_zero_order(&re, &im, phi);
        let sum_r: f64 = r.iter().sum();
        assert!(sum_r > 0.0);
    }

    // --- Echo integration / SNR ---

    #[test]
    fn test_echo_peak_integral() {
        let echo = vec![0.0, 0.1, 0.5, 1.0, 0.5, 0.1, 0.0];
        let integral = echo_peak_integral(&echo, 1);
        // Centre is index 3, half_width=1 → indices 2,3,4
        assert!((integral - 2.0).abs() < 1e-12);
    }

    #[test]
    fn test_estimate_echo_noise() {
        let mut echo = vec![1.0; 10];
        // Put noise only in the tail (indices 8,9)
        echo[8] = 0.1;
        echo[9] = -0.1;
        let noise = estimate_echo_noise(&echo);
        assert!(noise >= 0.0);
    }

    #[test]
    fn test_echo_snr_clean_signal() {
        // Clean echo: high peak, zero tail → infinite SNR
        let mut echo = vec![0.0; 20];
        echo[10] = 10.0;
        let snr = echo_snr(&echo);
        // Noise in tail is 0, snr should be infinite or very large
        assert!(snr > 100.0 || snr.is_infinite());
    }

    #[test]
    fn test_echo_snr_noisy_signal() {
        let echo: Vec<f64> = (0..100).map(|i| if i == 50 { 10.0 } else { 0.01 }).collect();
        let snr = echo_snr(&echo);
        assert!(snr > 1.0);
    }

    // --- FID ---

    #[test]
    fn test_fid_length() {
        let fid = fid_signal(1.0, 0.1, 0.0, 0.0, 1e6, 1000);
        assert_eq!(fid.len(), 1000);
    }

    #[test]
    fn test_fid_decays() {
        let fid = fid_signal(1.0, 0.01, 0.0, 0.0, 1e6, 1000);
        // Envelope at t=0 should be largest
        assert!(fid[0].abs() >= fid[999].abs());
    }

    #[test]
    fn test_fid_initial_amplitude() {
        let m0 = 3.7;
        let fid = fid_signal(m0, 10.0, 0.0, 0.0, 1e6, 1);
        assert!((fid[0] - m0).abs() < 1e-12);
    }

    // --- Material database ---

    #[test]
    fn test_lookup_water() {
        let mat = lookup_material("Water").expect("Water should be in DB");
        assert!((mat.t1 - 3.0).abs() < 0.01);
        assert!((mat.t2 - 3.0).abs() < 0.01);
        assert!((mat.diffusion - 2.3e-9).abs() < 1e-11);
    }

    #[test]
    fn test_lookup_brain_white_matter() {
        let mat = lookup_material("White Matter").expect("Brain White Matter should be found");
        assert!((mat.t1 - 0.8).abs() < 0.01);
    }

    #[test]
    fn test_lookup_unknown_material() {
        let result = lookup_material("Unobtainium");
        assert!(result.is_none());
    }

    #[test]
    fn test_material_database_size() {
        assert!(NMR_MATERIALS.len() >= 5);
    }

    #[test]
    fn test_lookup_mineral_oil() {
        let mat = lookup_material("Mineral Oil").unwrap();
        assert!(mat.t2 < mat.t1, "For oils, T2 < T1");
    }

    #[test]
    fn test_material_t2_t1_relationship() {
        for mat in NMR_MATERIALS {
            assert!(mat.t2 <= mat.t1 + 1e-9, "T2 ≤ T1 for {}", mat.name);
        }
    }

    // --- Well logging ---

    #[test]
    fn test_nmr_porosity_full_saturation() {
        let phi = nmr_porosity(1.0, 1.0);
        assert!((phi - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_nmr_porosity_partial() {
        let phi = nmr_porosity(0.3, 1.0);
        assert!((phi - 0.3).abs() < 1e-12);
    }

    #[test]
    fn test_nmr_porosity_clamps_to_one() {
        let phi = nmr_porosity(2.0, 1.0);
        assert!((phi - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_nmr_porosity_zero_reference() {
        let phi = nmr_porosity(1.0, 0.0);
        assert_eq!(phi, 0.0);
    }

    #[test]
    fn test_timur_coates_permeability() {
        let k = timur_coates_permeability(0.2, 0.8, 0.2, 10.0);
        // k = (0.2/10)^4 * (0.8/0.2)^2 = (0.02)^4 * 16 = 1.6e-7 * 16
        let expected = (0.2_f64 / 10.0).powi(4) * (0.8_f64 / 0.2).powi(2);
        assert!(rel_err(k, expected) < 1e-10);
    }

    #[test]
    fn test_timur_coates_zero_bvi() {
        let k = timur_coates_permeability(0.2, 1.0, 0.0, 10.0);
        assert_eq!(k, 0.0);
    }

    #[test]
    fn test_partition_ffi_bvi() {
        let dist = T2Distribution {
            t2_axis: vec![0.001, 0.01, 0.033, 0.1, 0.5],
            amplitudes: vec![0.1, 0.2, 0.1, 0.4, 0.2],
            lambda: 0.1,
        };
        let cutoff = 0.033;
        let (ffi, bvi) = partition_ffi_bvi(&dist, cutoff);
        assert!((ffi + bvi - 1.0).abs() < 1e-10);
        assert!(ffi >= 0.0 && bvi >= 0.0);
    }

    #[test]
    fn test_fluid_typing_cutoffs() {
        let (gas, oil, water) = fluid_typing_cutoffs();
        assert!(gas < oil);
        assert!(oil < water);
    }

    // --- T2 Distribution (NNLS) ---

    #[test]
    fn test_t2_distribution_single_component() {
        // Simulate a single-T2 echo train
        let t2_true = 0.1;
        let te = 5e-3;
        let n = 64;
        let amps: Vec<f64> = (1..=n).map(|i| (-(i as f64) * te / t2_true).exp()).collect();
        let t2_axis: Vec<f64> = (1..=30).map(|i| i as f64 * 0.01).collect();
        let dist = compute_t2_distribution(&amps, te, &t2_axis, 0.01);
        assert_eq!(dist.amplitudes.len(), t2_axis.len());
        // All amplitudes should be non-negative
        for &a in &dist.amplitudes {
            assert!(a >= -1e-10, "NNLS should produce non-negative amplitudes");
        }
    }

    #[test]
    fn test_t2_distribution_total_amplitude_positive() {
        let amps: Vec<f64> = (1..=32).map(|i| (-(i as f64) * 0.005 / 0.1).exp()).collect();
        let t2_axis: Vec<f64> = (1..=20).map(|i| i as f64 * 0.02).collect();
        let dist = compute_t2_distribution(&amps, 0.005, &t2_axis, 0.1);
        let total: f64 = dist.amplitudes.iter().sum();
        assert!(total > 0.0);
    }

    // --- T2/T1 ratio ---

    #[test]
    fn test_t2_t1_ratio_water() {
        let mat = lookup_material("Water").unwrap();
        let ratio = t2_t1_ratio(mat.t2, mat.t1);
        assert!((ratio - 1.0).abs() < 0.01, "Water should have T2/T1 ≈ 1");
    }

    #[test]
    fn test_t2_t1_ratio_solid_like() {
        let ratio = t2_t1_ratio(0.001, 1.0);
        assert!(ratio < 0.01, "Solid-like material: T2 << T1");
    }

    // --- CPMG process helper ---

    #[test]
    fn test_process_cpmg_end_to_end() {
        let t2 = 0.08;
        let te = 2e-3;
        let config = CpmgConfig {
            n_echoes: 64,
            echo_spacing: te,
            larmor_freq: 2.0e6,
            sample_rate: 10e3,
            samples_per_echo: 64,
        };
        let proc = CpmgProcessor::new(config.clone());
        let amps = proc.synthesize_echo_train(1.0, t2);
        let result = process_cpmg(&config, &amps, None, 0.1).unwrap();
        assert!(rel_err(result.t2, t2) < 0.01);
        assert!(result.snr > 1.0);
        assert!(result.t2_distribution.is_none());
    }

    #[test]
    fn test_process_cpmg_with_t2_distribution() {
        let t2 = 0.05;
        let te = 2e-3;
        let config = CpmgConfig {
            n_echoes: 64,
            echo_spacing: te,
            larmor_freq: 2.0e6,
            sample_rate: 10e3,
            samples_per_echo: 64,
        };
        let proc = CpmgProcessor::new(config.clone());
        let amps = proc.synthesize_echo_train(1.0, t2);
        let t2_axis: Vec<f64> = (1..=20).map(|i| i as f64 * 0.01).collect();
        let result = process_cpmg(&config, &amps, Some(&t2_axis), 0.1).unwrap();
        assert!(result.t2_distribution.is_some());
    }

    // --- Resolvable T2 bounds ---

    #[test]
    fn test_minimum_resolvable_t2() {
        let t2_min = minimum_resolvable_t2(2e-3, 64);
        assert!(t2_min > 0.0);
    }

    #[test]
    fn test_maximum_resolvable_t2() {
        let t2_max = maximum_resolvable_t2(2e-3, 64);
        assert!(t2_max > minimum_resolvable_t2(2e-3, 64));
    }

    // --- GAMMA_H constant ---

    #[test]
    fn test_gamma_h_value() {
        assert!((GAMMA_H - 267.522e6).abs() < 1.0);
    }
}
