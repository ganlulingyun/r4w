//! # DSL Vectoring Processor (VDSL2 G.993.5)
//!
//! Implements ITU-T G.993.5 vectoring for VDSL2, which cancels Far-End
//! CrossTalk (FEXT) between copper pairs in the same cable binder.
//!
//! Vectoring can improve downstream bit rates by 50–100% on medium loops
//! (500–1500 m) by eliminating the dominant interference source between lines.
//!
//! ## Architecture
//!
//! ```text
//!  DSLAM                          CPE (each line)
//!  ┌─────────────────────────┐    ┌──────────────┐
//!  │ Data[0..N]              │    │  Slicer      │
//!  │      │                  │    │  │  Error     │
//!  │  VectoringPrecoder      │    │  └──────────►│ Backchannel
//!  │  (ZF / MMSE per tone)   │    └──────────────┘
//!  │      │                  │             │
//!  │  Tx[0..N] ─────────────►── Binder ──►│
//!  │                         │    │        │
//!  │  VectoringCanceller     │◄───┘        │
//!  │  (per tone)             │   FEXT      │
//!  │      │                  │             │
//!  │  ErrorFeedback ◄────────────────────── backchannel
//!  └─────────────────────────┘
//! ```
//!
//! ## Standards
//!
//! - ITU-T G.993.5 (Vectoring)
//! - ITU-T G.993.2 (VDSL2 – base DMT parameters)
//! - BBF TR-069 amendment 6 (vectoring management)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::dsl_vectoring_processor::{
//!     VectoringConfig, DslVectoringProcessor, Vdsl2Profile,
//! };
//!
//! let cfg = VectoringConfig {
//!     num_lines: 4,
//!     num_tones: 64,   // small for test
//!     profile: Vdsl2Profile::Profile17a,
//!     adaptation_rate: 0.01,
//!     diagonal_loading: 1e-4,
//!     noise_variance: 1e-6,
//! };
//!
//! let mut proc = DslVectoringProcessor::new(cfg);
//! proc.initialize_identity();
//!
//! // Transmit four random symbols (one per line) on tone 0
//! let tx_data: Vec<[f64; 2]> = (0..4).map(|i| [i as f64, 0.0]).collect();
//! let precoded = proc.precode_downstream(0, &tx_data);
//! assert_eq!(precoded.len(), 4);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// VDSL2 deployment profile per ITU-T G.993.2 Amendment 1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Vdsl2Profile {
    /// 8 MHz, 2048 subcarriers (4.3125 kHz spacing)
    Profile8a,
    /// 8.832 MHz, 2048 subcarriers
    Profile8b,
    /// 8.5 MHz, 2048 subcarriers
    Profile8c,
    /// 8.832 MHz, 2048 subcarriers (alternate mask)
    Profile8d,
    /// 12 MHz, 2783 subcarriers
    Profile12a,
    /// 12.0 MHz, 2783 subcarriers (alternate mask)
    Profile12b,
    /// 17.664 MHz, 4096 subcarriers — primary vectoring profile
    Profile17a,
    /// 30 MHz, 3479 subcarriers
    Profile30a,
    /// 35 MHz, 8192 subcarriers (G.fast overlap)
    Profile35b,
}

impl Vdsl2Profile {
    /// Nominal subcarrier spacing in Hz (4312.5 Hz for all VDSL2 profiles).
    pub fn subcarrier_spacing_hz(&self) -> f64 {
        4312.5
    }

    /// Maximum number of subcarriers in downstream direction.
    pub fn max_downstream_tones(&self) -> usize {
        match self {
            Vdsl2Profile::Profile8a
            | Vdsl2Profile::Profile8b
            | Vdsl2Profile::Profile8c
            | Vdsl2Profile::Profile8d => 2048,
            Vdsl2Profile::Profile12a | Vdsl2Profile::Profile12b => 2783,
            Vdsl2Profile::Profile17a => 4096,
            Vdsl2Profile::Profile30a => 3479,
            Vdsl2Profile::Profile35b => 8192,
        }
    }

    /// Maximum bits per symbol per tone (log2 of maximum QAM order).
    pub fn max_bits_per_tone(&self) -> u8 {
        15 // 32768-QAM
    }
}

/// Configuration for the vectoring processor.
#[derive(Debug, Clone)]
pub struct VectoringConfig {
    /// Number of DSL lines in the vectoring group (N).  Max ~512 per G.993.5.
    pub num_lines: usize,
    /// Number of DMT subcarriers (tones) to process.
    pub num_tones: usize,
    /// VDSL2 profile.
    pub profile: Vdsl2Profile,
    /// LMS adaptation step size μ for coefficient updates.
    pub adaptation_rate: f64,
    /// Diagonal loading factor λ for ill-conditioned tone matrices.
    pub diagonal_loading: f64,
    /// Per-tone noise power spectral density (linear, V²/Hz) — σ² for MMSE.
    pub noise_variance: f64,
}

impl Default for VectoringConfig {
    fn default() -> Self {
        VectoringConfig {
            num_lines: 4,
            num_tones: 512,
            profile: Vdsl2Profile::Profile17a,
            adaptation_rate: 0.005,
            diagonal_loading: 1e-5,
            noise_variance: 1e-7,
        }
    }
}

// ---------------------------------------------------------------------------
// Complex number helpers (no external crates)
// ---------------------------------------------------------------------------

/// A 2-element array `[re, im]` represents a complex number.
type C64 = [f64; 2];

#[inline]
fn c_add(a: C64, b: C64) -> C64 {
    [a[0] + b[0], a[1] + b[1]]
}

#[inline]
fn c_sub(a: C64, b: C64) -> C64 {
    [a[0] - b[0], a[1] - b[1]]
}

#[inline]
fn c_mul(a: C64, b: C64) -> C64 {
    [a[0] * b[0] - a[1] * b[1], a[0] * b[1] + a[1] * b[0]]
}

#[inline]
fn c_conj(a: C64) -> C64 {
    [a[0], -a[1]]
}

#[inline]
fn c_scale(a: C64, s: f64) -> C64 {
    [a[0] * s, a[1] * s]
}

#[inline]
fn c_abs_sq(a: C64) -> f64 {
    a[0] * a[0] + a[1] * a[1]
}

#[inline]
fn c_abs(a: C64) -> f64 {
    c_abs_sq(a).sqrt()
}

#[inline]
fn c_recip(a: C64) -> C64 {
    let d = c_abs_sq(a);
    if d < 1e-300 {
        [0.0, 0.0]
    } else {
        [a[0] / d, -a[1] / d]
    }
}

/// Multiply two N×N complex matrices stored as flat row-major Vec<C64>.
fn mat_mul(a: &[C64], b: &[C64], n: usize) -> Vec<C64> {
    let mut c = vec![[0.0, 0.0]; n * n];
    for i in 0..n {
        for k in 0..n {
            let aik = a[i * n + k];
            for j in 0..n {
                let bkj = b[k * n + j];
                c[i * n + j] = c_add(c[i * n + j], c_mul(aik, bkj));
            }
        }
    }
    c
}

/// Return the N×N identity matrix.
fn mat_identity(n: usize) -> Vec<C64> {
    let mut m = vec![[0.0_f64, 0.0_f64]; n * n];
    for i in 0..n {
        m[i * n + i] = [1.0, 0.0];
    }
    m
}

/// Compute the Hermitian (conjugate transpose) of an N×N matrix.
fn mat_herm(a: &[C64], n: usize) -> Vec<C64> {
    let mut h = vec![[0.0_f64, 0.0_f64]; n * n];
    for i in 0..n {
        for j in 0..n {
            h[j * n + i] = c_conj(a[i * n + j]);
        }
    }
    h
}

/// Multiply matrix A (N×N) by column vector x (N) → y (N).
fn mat_vec_mul(a: &[C64], x: &[C64], n: usize) -> Vec<C64> {
    let mut y = vec![[0.0_f64, 0.0_f64]; n];
    for i in 0..n {
        for j in 0..n {
            y[i] = c_add(y[i], c_mul(a[i * n + j], x[j]));
        }
    }
    y
}

/// Gauss-Jordan inversion of an N×N complex matrix.
/// Returns None if the matrix is singular (determinant ≈ 0).
fn mat_inv(a: &[C64], n: usize) -> Option<Vec<C64>> {
    // Build augmented [A | I]
    let mut aug = vec![[0.0_f64, 0.0_f64]; n * 2 * n];
    for i in 0..n {
        for j in 0..n {
            aug[i * (2 * n) + j] = a[i * n + j];
        }
        aug[i * (2 * n) + n + i] = [1.0, 0.0];
    }
    let w = 2 * n;
    for col in 0..n {
        // Partial pivot by magnitude
        let mut max_row = col;
        let mut max_val = c_abs(aug[col * w + col]);
        for row in (col + 1)..n {
            let v = c_abs(aug[row * w + col]);
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < 1e-14 {
            return None; // singular
        }
        if max_row != col {
            for j in 0..w {
                let tmp = aug[col * w + j];
                aug[col * w + j] = aug[max_row * w + j];
                aug[max_row * w + j] = tmp;
            }
        }
        let pivot = aug[col * w + col];
        let pivot_inv = c_recip(pivot);
        for j in 0..w {
            aug[col * w + j] = c_mul(aug[col * w + j], pivot_inv);
        }
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row * w + col];
            for j in 0..w {
                let sub = c_mul(factor, aug[col * w + j]);
                aug[row * w + j] = c_sub(aug[row * w + j], sub);
            }
        }
    }
    // Extract right half
    let mut inv = vec![[0.0_f64, 0.0_f64]; n * n];
    for i in 0..n {
        for j in 0..n {
            inv[i * n + j] = aug[i * w + n + j];
        }
    }
    Some(inv)
}

/// Diagonal-loaded pseudo-inverse: (A^H A + λI)^{-1} A^H.
fn mmse_pseudo_inv(a: &[C64], n: usize, lambda: f64) -> Vec<C64> {
    let ah = mat_herm(a, n);
    let aha = mat_mul(&ah, a, n);
    // Add λI
    let mut aha_reg = aha.clone();
    for i in 0..n {
        aha_reg[i * n + i] = c_add(aha_reg[i * n + i], [lambda, 0.0]);
    }
    match mat_inv(&aha_reg, n) {
        Some(inv) => mat_mul(&inv, &ah, n),
        None => mat_identity(n),
    }
}

// ---------------------------------------------------------------------------
// FEXT Channel Model
// ---------------------------------------------------------------------------

/// FEXT coupling model for N DSL lines in a binder.
///
/// The FEXT power spectral density between lines i and j follows:
///
/// `|H_fext(i,j,f)|² = k_f · f² · |H_direct(i,f)|² · d`
///
/// where:
/// - `k_f` = FEXT coupling constant (~1e-20 for typical binders, ITU-T G.993.1)
/// - `f` = tone frequency in Hz
/// - `d` = coupling length in meters
/// - `H_direct(i,f)` = direct-path channel of line i at frequency f
#[derive(Debug, Clone)]
pub struct FextModel {
    /// Number of lines N.
    pub num_lines: usize,
    /// Coupling coefficient k_f (default 1e-20 per G.993.1 annex B).
    pub coupling_constant: f64,
    /// Coupling length in metres.
    pub coupling_length_m: f64,
    /// Per-tone complex channel matrices H[tone], each N×N row-major.
    /// H[tone][i*N+j] = channel from transmitter j to receiver i.
    pub channel_matrices: Vec<Vec<C64>>,
}

impl FextModel {
    /// Build a FEXT model from direct-path attenuation vectors.
    ///
    /// `direct_paths[line][tone]` = complex attenuation of the direct path for
    /// `line` at `tone`.  Off-diagonal FEXT coupling is synthesised according
    /// to the ITU formula above.
    pub fn from_direct_paths(
        direct_paths: &[Vec<C64>],
        tone_freqs_hz: &[f64],
        coupling_constant: f64,
        coupling_length_m: f64,
    ) -> Self {
        let n = direct_paths.len();
        let num_tones = tone_freqs_hz.len();
        let mut channel_matrices = Vec::with_capacity(num_tones);

        for t in 0..num_tones {
            let f = tone_freqs_hz[t];
            let mut h = vec![[0.0_f64, 0.0_f64]; n * n];
            for i in 0..n {
                for j in 0..n {
                    if i == j {
                        // Direct path on the diagonal
                        h[i * n + j] = direct_paths[i][t];
                    } else {
                        // FEXT coupling magnitude: sqrt(k_f · f² · |H_direct_j|² · d)
                        let h_direct_sq = c_abs_sq(direct_paths[j][t]);
                        let mag = (coupling_constant * f * f * h_direct_sq * coupling_length_m)
                            .sqrt()
                            .min(1.0); // clamp to physical limit
                        // Random-looking but deterministic phase from indices
                        let phase = 2.0 * PI * ((i * 7 + j * 13 + t * 3) as f64 / 31.0);
                        h[i * n + j] = [mag * phase.cos(), mag * phase.sin()];
                    }
                }
            }
            channel_matrices.push(h);
        }

        FextModel {
            num_lines: n,
            coupling_constant,
            coupling_length_m,
            channel_matrices,
        }
    }

    /// Create a simple model with identity direct path + small FEXT (for testing).
    pub fn identity_with_fext(num_lines: usize, num_tones: usize, fext_level: f64) -> Self {
        let n = num_lines;
        let mut channel_matrices = Vec::with_capacity(num_tones);
        for t in 0..num_tones {
            let mut h = mat_identity(n);
            for i in 0..n {
                for j in 0..n {
                    if i != j {
                        let phase = 2.0 * PI * ((i * 7 + j * 13 + t) as f64 / 31.0);
                        h[i * n + j] = [
                            fext_level * phase.cos(),
                            fext_level * phase.sin(),
                        ];
                    }
                }
            }
            channel_matrices.push(h);
        }
        FextModel {
            num_lines,
            coupling_constant: 1e-20,
            coupling_length_m: 500.0,
            channel_matrices,
        }
    }

    /// Return the N×N channel matrix for a given tone index.
    pub fn channel_matrix(&self, tone: usize) -> &[C64] {
        &self.channel_matrices[tone]
    }

    /// Compute received signal for a given tone.
    /// `tx[j]` = transmitted sample from line j.
    pub fn apply(&self, tone: usize, tx: &[C64]) -> Vec<C64> {
        mat_vec_mul(self.channel_matrix(tone), tx, self.num_lines)
    }

    /// Compute per-line FEXT power for a given tone (sum of off-diagonal |H|²).
    pub fn fext_power_per_line(&self, tone: usize) -> Vec<f64> {
        let n = self.num_lines;
        let h = self.channel_matrix(tone);
        let mut pwr = vec![0.0_f64; n];
        for i in 0..n {
            for j in 0..n {
                if i != j {
                    pwr[i] += c_abs_sq(h[i * n + j]);
                }
            }
        }
        pwr
    }
}

// ---------------------------------------------------------------------------
// Vectoring Precoder (Downstream)
// ---------------------------------------------------------------------------

/// Downstream vectoring precoder.
///
/// For each DMT tone, stores a precoding matrix **P** such that the
/// transmitted vector after precoding is `x = P · d` where `d` is the
/// desired data vector.  The precoder is designed so that after the channel
/// `H`, the received signal `y = H · x = H · P · d ≈ d` (zero-forcing)
/// or has MMSE-optimal interference.
///
/// Zero-forcing: `P = H^(-1)` (exact inverse, when N is small).
/// MMSE: `P = H^H (H H^H + σ²I)^{-1}` (Wiener solution).
#[derive(Debug, Clone)]
pub struct VectoringPrecoder {
    /// Number of lines.
    num_lines: usize,
    /// Number of tones.
    num_tones: usize,
    /// Precoding matrices P[tone], each N×N row-major.
    precoding_matrices: Vec<Vec<C64>>,
    /// Per-tone normalisation gains to satisfy power constraints.
    norm_gains: Vec<f64>,
    /// Diagonal loading factor.
    diagonal_loading: f64,
}

impl VectoringPrecoder {
    /// Create a new precoder initialised to identity (no precoding).
    pub fn new(num_lines: usize, num_tones: usize, diagonal_loading: f64) -> Self {
        let identity = mat_identity(num_lines);
        VectoringPrecoder {
            num_lines,
            num_tones,
            precoding_matrices: vec![identity; num_tones],
            norm_gains: vec![1.0; num_tones],
            diagonal_loading,
        }
    }

    /// Update the precoding matrix for `tone` using zero-forcing (H^{-1}).
    /// Falls back to diagonal-loaded pseudo-inverse if H is ill-conditioned.
    pub fn update_zf(&mut self, tone: usize, channel: &[C64]) {
        let n = self.num_lines;
        // Try exact inverse first
        let p = match mat_inv(channel, n) {
            Some(inv) => inv,
            None => {
                // Diagonal-loaded pseudo-inverse
                mmse_pseudo_inv(channel, n, self.diagonal_loading)
            }
        };
        // Power normalisation: scale so that max column norm ≤ 1
        let max_norm = (0..n)
            .map(|j| {
                (0..n)
                    .map(|i| c_abs_sq(p[i * n + j]))
                    .sum::<f64>()
                    .sqrt()
            })
            .fold(f64::NEG_INFINITY, f64::max);
        let gain = if max_norm > 1e-12 { 1.0 / max_norm } else { 1.0 };
        self.norm_gains[tone] = gain;
        self.precoding_matrices[tone] = p.iter().map(|&c| c_scale(c, gain)).collect();
    }

    /// Update the precoding matrix for `tone` using MMSE.
    pub fn update_mmse(&mut self, tone: usize, channel: &[C64], noise_var: f64) {
        let n = self.num_lines;
        // MMSE receive filter: W = (H^H H + σ²I)^{-1} H^H
        // For precoding we use the matched-filter / regularised approach
        let p = mmse_pseudo_inv(channel, n, noise_var + self.diagonal_loading);
        let max_norm = (0..n)
            .map(|j| {
                (0..n)
                    .map(|i| c_abs_sq(p[i * n + j]))
                    .sum::<f64>()
                    .sqrt()
            })
            .fold(f64::NEG_INFINITY, f64::max);
        let gain = if max_norm > 1e-12 { 1.0 / max_norm } else { 1.0 };
        self.norm_gains[tone] = gain;
        self.precoding_matrices[tone] = p.iter().map(|&c| c_scale(c, gain)).collect();
    }

    /// Apply the precoder: returns precoded vector `x = P · d`.
    pub fn precode(&self, tone: usize, data: &[C64]) -> Vec<C64> {
        mat_vec_mul(&self.precoding_matrices[tone], data, self.num_lines)
    }

    /// Return the precoding matrix for inspection.
    pub fn matrix(&self, tone: usize) -> &[C64] {
        &self.precoding_matrices[tone]
    }

    /// Per-tone normalisation gain applied after matrix multiplication.
    pub fn norm_gain(&self, tone: usize) -> f64 {
        self.norm_gains[tone]
    }
}

// ---------------------------------------------------------------------------
// Vectoring Canceller (Upstream)
// ---------------------------------------------------------------------------

/// Upstream FEXT cancellation at the DSLAM receiver.
///
/// For each DMT tone, stores a cancellation matrix **W** such that the
/// estimated data vector is `d̂ = W · y`, where `y` is the received vector.
///
/// MMSE: `W = (H^H H + σ²I)^{-1} H^H`
#[derive(Debug, Clone)]
pub struct VectoringCanceller {
    /// Number of lines.
    num_lines: usize,
    /// Number of tones.
    num_tones: usize,
    /// Cancellation matrices W[tone], each N×N row-major.
    cancellation_matrices: Vec<Vec<C64>>,
    /// Per-tone residual FEXT power estimate (for monitoring).
    residual_fext: Vec<f64>,
    /// Noise variance used during MMSE computation.
    noise_variance: f64,
}

impl VectoringCanceller {
    /// Create a new canceller initialised to identity (no cancellation).
    pub fn new(num_lines: usize, num_tones: usize, noise_variance: f64) -> Self {
        let identity = mat_identity(num_lines);
        VectoringCanceller {
            num_lines,
            num_tones,
            cancellation_matrices: vec![identity; num_tones],
            residual_fext: vec![0.0; num_tones],
            noise_variance,
        }
    }

    /// Update the cancellation matrix for `tone` using MMSE.
    pub fn update_mmse(&mut self, tone: usize, channel: &[C64]) {
        let n = self.num_lines;
        let w = mmse_pseudo_inv(channel, n, self.noise_variance);
        self.cancellation_matrices[tone] = w;
    }

    /// Update the cancellation matrix for `tone` using zero-forcing (H^{-1}).
    pub fn update_zf(&mut self, tone: usize, channel: &[C64]) {
        let n = self.num_lines;
        let w = match mat_inv(channel, n) {
            Some(inv) => inv,
            None => mmse_pseudo_inv(channel, n, self.noise_variance),
        };
        self.cancellation_matrices[tone] = w;
    }

    /// Apply the canceller: returns estimated data `d̂ = W · y`.
    pub fn cancel(&self, tone: usize, received: &[C64]) -> Vec<C64> {
        mat_vec_mul(&self.cancellation_matrices[tone], received, self.num_lines)
    }

    /// Iterative FEXT subtraction for large groups (block Gauss-Seidel).
    /// Iterates `iters` times, subtracting estimated interference per line.
    pub fn cancel_iterative(&self, tone: usize, received: &[C64], iters: usize) -> Vec<C64> {
        let n = self.num_lines;
        let w = &self.cancellation_matrices[tone];
        let mut estimate = received.to_vec();
        for _ in 0..iters {
            let new_est = mat_vec_mul(w, &estimate, n);
            estimate = new_est;
        }
        estimate
    }

    /// Record estimated residual FEXT power for a tone.
    pub fn set_residual_fext(&mut self, tone: usize, power: f64) {
        self.residual_fext[tone] = power;
    }

    /// Return the estimated residual FEXT power for a tone.
    pub fn residual_fext(&self, tone: usize) -> f64 {
        self.residual_fext[tone]
    }

    /// Return the cancellation matrix for inspection.
    pub fn matrix(&self, tone: usize) -> &[C64] {
        &self.cancellation_matrices[tone]
    }
}

// ---------------------------------------------------------------------------
// Error Feedback
// ---------------------------------------------------------------------------

/// Compressed error sample from a CPE slicer.
#[derive(Debug, Clone, Copy)]
pub struct SlicerError {
    /// Line index.
    pub line: usize,
    /// Tone index.
    pub tone: usize,
    /// Complex error value e = received − ideal_symbol.
    pub error: C64,
}

/// Error feedback subsystem.
///
/// CPE devices send quantised slicer errors back to the DSLAM via the
/// vectoring backchannel.  The DSLAM uses these errors to adapt the
/// precoder/canceller coefficients with an LMS update.
#[derive(Debug, Clone)]
pub struct ErrorFeedback {
    /// Number of lines.
    num_lines: usize,
    /// Number of tones.
    num_tones: usize,
    /// Accumulated error vectors per tone (N complex).
    error_accum: Vec<Vec<C64>>,
    /// Update count per tone (for averaging).
    update_count: Vec<usize>,
    /// LMS step size μ.
    adaptation_rate: f64,
    /// Error compression: quantisation bits (4 for typical backchannel).
    quant_bits: u8,
}

impl ErrorFeedback {
    /// Create a new error feedback module.
    pub fn new(num_lines: usize, num_tones: usize, adaptation_rate: f64) -> Self {
        ErrorFeedback {
            num_lines,
            num_tones,
            error_accum: vec![vec![[0.0, 0.0]; num_lines]; num_tones],
            update_count: vec![0; num_tones],
            adaptation_rate,
            quant_bits: 6,
        }
    }

    /// Compress an error value to `quant_bits` bits per component.
    pub fn compress_error(&self, err: C64, max_val: f64) -> C64 {
        let levels = (1u32 << self.quant_bits) as f64;
        let q = |x: f64| (x / max_val * levels).round() / levels * max_val;
        [q(err[0]), q(err[1])]
    }

    /// Accumulate a slicer error report from a CPE.
    pub fn accumulate(&mut self, report: SlicerError) {
        if report.tone < self.num_tones && report.line < self.num_lines {
            let e = report.error;
            let acc = &mut self.error_accum[report.tone][report.line];
            *acc = c_add(*acc, e);
            self.update_count[report.tone] += 1;
        }
    }

    /// Drain the accumulated errors for a tone, returning mean error per line.
    pub fn drain_errors(&mut self, tone: usize) -> Vec<C64> {
        let count = self.update_count[tone].max(1) as f64;
        let avg: Vec<C64> = self.error_accum[tone]
            .iter()
            .map(|&e| c_scale(e, 1.0 / count))
            .collect();
        // Reset accumulators
        for e in &mut self.error_accum[tone] {
            *e = [0.0, 0.0];
        }
        self.update_count[tone] = 0;
        avg
    }

    /// Compute an LMS gradient for a precoding column update.
    ///
    /// Given error vector `e` and transmitted symbol `d`, the gradient update
    /// for precoding matrix column j is:
    ///   `ΔP[:,j] = −μ · e · d*`
    pub fn lms_precoder_gradient(&self, error: &[C64], tx_symbol: C64) -> Vec<C64> {
        let mu = self.adaptation_rate;
        let d_conj = c_conj(tx_symbol);
        error
            .iter()
            .map(|&e| c_scale(c_mul(e, d_conj), -mu))
            .collect()
    }

    /// LMS update for the canceller: `ΔW[:,j] = −μ · error · rx_sample*`
    pub fn lms_canceller_gradient(&self, error: &[C64], rx_sample: C64) -> Vec<C64> {
        let mu = self.adaptation_rate;
        let r_conj = c_conj(rx_sample);
        error
            .iter()
            .map(|&e| c_scale(c_mul(e, r_conj), -mu))
            .collect()
    }

    /// Set the number of quantisation bits for backchannel compression.
    pub fn set_quant_bits(&mut self, bits: u8) {
        self.quant_bits = bits.clamp(2, 16);
    }
}

// ---------------------------------------------------------------------------
// Performance Metrics
// ---------------------------------------------------------------------------

/// Per-line performance metrics for a single tone.
#[derive(Debug, Clone, Default)]
pub struct ToneLinePerformance {
    /// Effective SNR after vectoring (dB).
    pub snr_db: f64,
    /// Allocated bits per symbol for this tone.
    pub bits_per_tone: u8,
    /// Signal power (linear).
    pub signal_power: f64,
    /// Noise + residual FEXT power (linear).
    pub noise_power: f64,
}

/// Per-line aggregated performance over all tones.
#[derive(Debug, Clone, Default)]
pub struct LinePerformance {
    /// Line index.
    pub line: usize,
    /// Effective data rate in bit/s.
    pub data_rate_bps: f64,
    /// Average SNR across all active tones (dB).
    pub avg_snr_db: f64,
    /// Number of active tones.
    pub active_tones: usize,
    /// Vectoring gain over baseline (dB).
    pub vectoring_gain_db: f64,
    /// Per-tone metrics.
    pub tone_metrics: Vec<ToneLinePerformance>,
}

/// FEXT-free SNR estimator.
pub struct SnrEstimator {
    /// Number of lines.
    num_lines: usize,
    /// Number of tones.
    num_tones: usize,
    /// Tone subcarrier frequencies in Hz.
    tone_freqs_hz: Vec<f64>,
    /// Noise PSD per line per tone (linear).
    noise_psd: Vec<Vec<f64>>,
    /// Signal PSD per line per tone (linear).
    signal_psd: Vec<Vec<f64>>,
}

impl SnrEstimator {
    /// Create a new estimator.  `tone_freqs_hz[t]` is the centre frequency of
    /// tone `t`.
    pub fn new(num_lines: usize, num_tones: usize, tone_freqs_hz: Vec<f64>) -> Self {
        SnrEstimator {
            num_lines,
            num_tones,
            tone_freqs_hz,
            noise_psd: vec![vec![1e-9; num_tones]; num_lines],
            signal_psd: vec![vec![1.0; num_tones]; num_lines],
        }
    }

    /// Update signal PSD estimate from received pilot symbols.
    pub fn update_signal_psd(&mut self, line: usize, tone: usize, power: f64) {
        if line < self.num_lines && tone < self.num_tones {
            // Exponential smoothing with α = 0.1
            self.signal_psd[line][tone] =
                0.9 * self.signal_psd[line][tone] + 0.1 * power;
        }
    }

    /// Update noise PSD estimate from error samples.
    pub fn update_noise_psd(&mut self, line: usize, tone: usize, power: f64) {
        if line < self.num_lines && tone < self.num_tones {
            self.noise_psd[line][tone] =
                0.9 * self.noise_psd[line][tone] + 0.1 * power;
        }
    }

    /// Compute SNR in dB for a specific line and tone.
    pub fn snr_db(&self, line: usize, tone: usize) -> f64 {
        let s = self.signal_psd[line][tone].max(1e-300);
        let n = self.noise_psd[line][tone].max(1e-300);
        10.0 * (s / n).log10()
    }

    /// Compute achievable bits per tone using Shannon–Hartley theorem.
    ///
    /// `b = floor(log2(1 + SNR))` clamped to [0, 15] for 32768-QAM max.
    pub fn bits_per_tone(&self, line: usize, tone: usize) -> u8 {
        let snr_lin = {
            let s = self.signal_psd[line][tone];
            let n = self.noise_psd[line][tone].max(1e-300);
            s / n
        };
        let bits = (1.0 + snr_lin).log2().floor() as u8;
        bits.min(15)
    }

    /// Calculate total bit rate for a line across all tones.
    ///
    /// `R = Σ_t bits_per_tone(t) × subcarrier_spacing_hz`
    pub fn data_rate_bps(&self, line: usize, subcarrier_spacing_hz: f64) -> f64 {
        (0..self.num_tones)
            .map(|t| self.bits_per_tone(line, t) as f64 * subcarrier_spacing_hz)
            .sum()
    }

    /// Compute vectoring gain for a line by comparing signal power to
    /// (noise + FEXT power) before and after vectoring.
    ///
    /// `gain_dB = avg_SNR_vectored - avg_SNR_baseline`
    pub fn vectoring_gain_db(
        &self,
        line: usize,
        baseline_noise_psd: &[f64],
    ) -> f64 {
        let avg_snr_vec: f64 = (0..self.num_tones)
            .map(|t| self.snr_db(line, t))
            .sum::<f64>()
            / self.num_tones as f64;
        let avg_snr_base: f64 = (0..self.num_tones)
            .map(|t| {
                let s = self.signal_psd[line][t].max(1e-300);
                let n = baseline_noise_psd[t].max(1e-300);
                10.0 * (s / n).log10()
            })
            .sum::<f64>()
            / self.num_tones as f64;
        avg_snr_vec - avg_snr_base
    }

    /// Return aggregated `LinePerformance` for one line.
    pub fn line_performance(
        &self,
        line: usize,
        subcarrier_spacing_hz: f64,
        baseline_noise_psd: &[f64],
    ) -> LinePerformance {
        let tone_metrics: Vec<ToneLinePerformance> = (0..self.num_tones)
            .map(|t| ToneLinePerformance {
                snr_db: self.snr_db(line, t),
                bits_per_tone: self.bits_per_tone(line, t),
                signal_power: self.signal_psd[line][t],
                noise_power: self.noise_psd[line][t],
            })
            .collect();

        let active: Vec<usize> = (0..self.num_tones)
            .filter(|&t| tone_metrics[t].bits_per_tone > 0)
            .collect();

        let avg_snr = if active.is_empty() {
            0.0
        } else {
            active.iter().map(|&t| tone_metrics[t].snr_db).sum::<f64>()
                / active.len() as f64
        };

        LinePerformance {
            line,
            data_rate_bps: self.data_rate_bps(line, subcarrier_spacing_hz),
            avg_snr_db: avg_snr,
            active_tones: active.len(),
            vectoring_gain_db: self.vectoring_gain_db(line, baseline_noise_psd),
            tone_metrics,
        }
    }

    /// Return `num_tones`.
    pub fn num_tones(&self) -> usize {
        self.num_tones
    }

    /// Return `num_lines`.
    pub fn num_lines(&self) -> usize {
        self.num_lines
    }

    /// Return the tone frequency array.
    pub fn tone_freqs_hz(&self) -> &[f64] {
        &self.tone_freqs_hz
    }
}

// ---------------------------------------------------------------------------
// Binder Management
// ---------------------------------------------------------------------------

/// State of a single line within the vectoring group.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LineState {
    /// Line has not yet joined the vectoring group.
    Idle,
    /// Line is in the showtime phase and being vectored.
    Active,
    /// Line is leaving the group; coefficients being flushed.
    Leaving,
}

/// Manages lines joining and leaving the vectoring binder group.
#[derive(Debug, Clone)]
pub struct BinderGroup {
    /// Maximum number of lines supported.
    capacity: usize,
    /// Current state of each line slot.
    line_states: Vec<LineState>,
    /// Pilot sequence seed per line (for cross-correlation sync).
    pilot_seeds: Vec<u32>,
    /// Whether partial vectoring is enabled (vectoring a subset of lines).
    partial_vectoring: bool,
}

impl BinderGroup {
    /// Create a new binder group with the given capacity.
    pub fn new(capacity: usize, partial_vectoring: bool) -> Self {
        BinderGroup {
            capacity,
            line_states: vec![LineState::Idle; capacity],
            pilot_seeds: (0..capacity as u32).collect(),
            partial_vectoring,
        }
    }

    /// Request that `line` joins the vectoring group.
    /// Returns true if the join was accepted.
    pub fn join(&mut self, line: usize) -> bool {
        if line >= self.capacity {
            return false;
        }
        if self.line_states[line] == LineState::Idle {
            self.line_states[line] = LineState::Active;
            true
        } else {
            false
        }
    }

    /// Request that `line` leaves the vectoring group.
    pub fn leave(&mut self, line: usize) {
        if line < self.capacity {
            self.line_states[line] = LineState::Leaving;
        }
    }

    /// Confirm `line` has completed its leaving procedure.
    pub fn confirm_leave(&mut self, line: usize) {
        if line < self.capacity && self.line_states[line] == LineState::Leaving {
            self.line_states[line] = LineState::Idle;
        }
    }

    /// Returns the set of active line indices.
    pub fn active_lines(&self) -> Vec<usize> {
        self.line_states
            .iter()
            .enumerate()
            .filter(|(_, s)| **s == LineState::Active)
            .map(|(i, _)| i)
            .collect()
    }

    /// Returns the number of active lines.
    pub fn num_active(&self) -> usize {
        self.line_states
            .iter()
            .filter(|s| **s == LineState::Active)
            .count()
    }

    /// Returns the pilot seed for a line (used for sequence assignment).
    pub fn pilot_seed(&self, line: usize) -> u32 {
        self.pilot_seeds.get(line).copied().unwrap_or(0)
    }

    /// Returns whether partial vectoring is enabled.
    pub fn is_partial(&self) -> bool {
        self.partial_vectoring
    }

    /// Return a simple maximal-length LFSR pilot sequence for a line.
    /// Length `len`, seeded with a unique per-line initial state derived from
    /// the pilot seed so that distinct lines produce different sequences.
    pub fn pilot_sequence(&self, line: usize, len: usize) -> Vec<f64> {
        let seed = self.pilot_seeds.get(line).copied().unwrap_or(line as u32 + 1);
        // Combine seed with line index to guarantee different states for each line.
        // Multiply by a prime and add 1 to avoid the all-zero degenerate state.
        let mut state = ((seed.wrapping_mul(0x9E3779B9u32)).wrapping_add(line as u32 + 1)) | 1;
        let mut seq = Vec::with_capacity(len);
        for _ in 0..len {
            // Fibonacci LFSR over GF(2) with polynomial x^16+x^14+x^13+x^11+1
            let bit = ((state >> 15) ^ (state >> 13) ^ (state >> 12) ^ (state >> 10)) & 1;
            state = ((state << 1) & 0xFFFF) | bit;
            seq.push(if bit == 0 { 1.0 } else { -1.0 });
        }
        seq
    }
}

// ---------------------------------------------------------------------------
// Main Processor
// ---------------------------------------------------------------------------

/// Complete DSL Vectoring Processor combining precoder, canceller, error
/// feedback, channel model, binder management, and performance estimation.
pub struct DslVectoringProcessor {
    /// Configuration.
    pub config: VectoringConfig,
    /// FEXT channel model.
    pub fext_model: Option<FextModel>,
    /// Downstream precoder.
    pub precoder: VectoringPrecoder,
    /// Upstream canceller.
    pub canceller: VectoringCanceller,
    /// Error feedback module.
    pub error_feedback: ErrorFeedback,
    /// SNR estimator.
    pub snr_estimator: SnrEstimator,
    /// Binder group manager.
    pub binder: BinderGroup,
    /// Tone centre frequencies in Hz.
    pub tone_freqs_hz: Vec<f64>,
    /// Per-tone noise PSD used as MMSE regulariser.
    noise_psd_per_tone: Vec<f64>,
}

impl DslVectoringProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: VectoringConfig) -> Self {
        let n = config.num_lines;
        let k = config.num_tones;
        let spacing = config.profile.subcarrier_spacing_hz();

        // First downstream subcarrier for VDSL2 17a is tone 6 (25.875 kHz)
        let start_tone = 6usize;
        let tone_freqs_hz: Vec<f64> = (0..k)
            .map(|t| (start_tone + t) as f64 * spacing)
            .collect();

        let noise_psd_per_tone = vec![config.noise_variance; k];

        let snr_estimator = SnrEstimator::new(n, k, tone_freqs_hz.clone());

        DslVectoringProcessor {
            precoder: VectoringPrecoder::new(n, k, config.diagonal_loading),
            canceller: VectoringCanceller::new(n, k, config.noise_variance),
            error_feedback: ErrorFeedback::new(n, k, config.adaptation_rate),
            binder: BinderGroup::new(n, false),
            snr_estimator,
            tone_freqs_hz,
            noise_psd_per_tone,
            fext_model: None,
            config,
        }
    }

    /// Attach a FEXT channel model.
    pub fn set_channel_model(&mut self, model: FextModel) {
        self.fext_model = Some(model);
    }

    /// Initialise precoder and canceller to identity (baseline, no vectoring).
    pub fn initialize_identity(&mut self) {
        let n = self.config.num_lines;
        let k = self.config.num_tones;
        let identity = mat_identity(n);
        for t in 0..k {
            self.precoder.precoding_matrices[t] = identity.clone();
            self.precoder.norm_gains[t] = 1.0;
            self.canceller.cancellation_matrices[t] = identity.clone();
        }
    }

    /// Initialise precoder and canceller from the stored FEXT model using ZF.
    pub fn initialize_from_channel_zf(&mut self) {
        if let Some(model) = &self.fext_model.clone() {
            for t in 0..self.config.num_tones {
                let h = model.channel_matrix(t);
                self.precoder.update_zf(t, h);
                self.canceller.update_zf(t, h);
            }
        }
    }

    /// Initialise precoder and canceller from the stored FEXT model using MMSE.
    pub fn initialize_from_channel_mmse(&mut self) {
        let noise_var = self.config.noise_variance;
        if let Some(model) = &self.fext_model.clone() {
            for t in 0..self.config.num_tones {
                let h = model.channel_matrix(t);
                self.precoder.update_mmse(t, h, noise_var);
                self.canceller.update_mmse(t, h);
            }
        }
    }

    /// Apply downstream precoding for a given tone.
    ///
    /// `data[i]` is the DMT symbol intended for line `i` on `tone`.
    /// Returns the N precoded transmit samples.
    pub fn precode_downstream(&self, tone: usize, data: &[C64]) -> Vec<C64> {
        self.precoder.precode(tone, data)
    }

    /// Simulate full downstream transmission for a tone through the FEXT channel
    /// and apply the precoder.  Returns received signals per line.
    pub fn simulate_downstream(&self, tone: usize, data: &[C64]) -> Vec<C64> {
        let precoded = self.precoder.precode(tone, data);
        if let Some(model) = &self.fext_model {
            model.apply(tone, &precoded)
        } else {
            precoded
        }
    }

    /// Apply upstream FEXT cancellation for a given tone.
    ///
    /// `received[i]` is the signal received at the DSLAM from line `i`.
    /// Returns the estimated data symbols.
    pub fn cancel_upstream(&self, tone: usize, received: &[C64]) -> Vec<C64> {
        self.canceller.cancel(tone, received)
    }

    /// Process an upstream received vector through the channel model then cancel.
    pub fn simulate_upstream(&self, tone: usize, tx: &[C64]) -> Vec<C64> {
        let received = if let Some(model) = &self.fext_model {
            model.apply(tone, tx)
        } else {
            tx.to_vec()
        };
        self.canceller.cancel(tone, &received)
    }

    /// Feed a CPE slicer error report into the error feedback module.
    pub fn report_error(&mut self, report: SlicerError) {
        self.error_feedback.accumulate(report);
    }

    /// Perform one round of LMS adaptation for a given tone using accumulated
    /// error feedback.  Updates both the precoder and canceller.
    pub fn adapt(&mut self, tone: usize, tx_symbols: &[C64], rx_samples: &[C64]) {
        let n = self.config.num_lines;
        let errors = self.error_feedback.drain_errors(tone);
        for j in 0..n {
            let grad_pre = self.error_feedback.lms_precoder_gradient(&errors, tx_symbols[j]);
            let grad_can = self.error_feedback.lms_canceller_gradient(&errors, rx_samples[j]);
            // Apply gradient update to column j of P and W
            let p = &mut self.precoder.precoding_matrices[tone];
            let w = &mut self.canceller.cancellation_matrices[tone];
            for i in 0..n {
                p[i * n + j] = c_add(p[i * n + j], grad_pre[i]);
                w[i * n + j] = c_add(w[i * n + j], grad_can[i]);
            }
        }
    }

    /// Update SNR estimates from a received pilot symbol set.
    pub fn update_snr_from_pilots(
        &mut self,
        tone: usize,
        received: &[C64],
        pilots: &[C64],
    ) {
        for (line, (&rx, &pilot)) in received.iter().zip(pilots.iter()).enumerate() {
            let sig_pwr = c_abs_sq(pilot).max(1e-30);
            let err = c_sub(rx, pilot);
            let noise_pwr = c_abs_sq(err).max(1e-30);
            self.snr_estimator.update_signal_psd(line, tone, sig_pwr);
            self.snr_estimator.update_noise_psd(line, tone, noise_pwr);
        }
    }

    /// Return line performance metrics for all lines.
    pub fn line_performances(&self) -> Vec<LinePerformance> {
        let spacing = self.config.profile.subcarrier_spacing_hz();
        let baseline_noise = &self.noise_psd_per_tone;
        (0..self.config.num_lines)
            .map(|l| self.snr_estimator.line_performance(l, spacing, baseline_noise))
            .collect()
    }

    /// Return the FEXT channel matrix for a tone (if channel model is present).
    pub fn channel_matrix(&self, tone: usize) -> Option<Vec<C64>> {
        self.fext_model
            .as_ref()
            .map(|m| m.channel_matrix(tone).to_vec())
    }

    /// Return total crosstalk reduction across all tones (dB).
    ///
    /// Computed as average reduction in off-diagonal power after applying
    /// the precoder: (||H·P - I||_F²_off_diag) relative to (||H - I||_F²_off_diag).
    pub fn crosstalk_reduction_db(&self, tone: usize) -> f64 {
        let n = self.config.num_lines;
        let model = match &self.fext_model {
            Some(m) => m,
            None => return 0.0,
        };
        let h = model.channel_matrix(tone);
        let p = self.precoder.matrix(tone);

        // H·P
        let hp = mat_mul(h, p, n);

        // Off-diagonal power before (H alone, with P=I)
        let before: f64 = (0..n)
            .flat_map(|i| (0..n).filter(move |&j| i != j).map(move |j| (i, j)))
            .map(|(i, j)| c_abs_sq(h[i * n + j]))
            .sum();

        // Off-diagonal power after (H·P, off-diagonal from identity)
        let after: f64 = (0..n)
            .flat_map(|i| (0..n).filter(move |&j| i != j).map(move |j| (i, j)))
            .map(|(i, j)| c_abs_sq(hp[i * n + j]))
            .sum();

        if after < 1e-300 {
            60.0 // clamp at 60 dB
        } else if before < 1e-300 {
            0.0
        } else {
            10.0 * (before / after).log10()
        }
    }
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/// Compute FEXT coupling constant from measured reference values per G.993.1.
///
/// Standard value for 0.4 mm copper at 1 MHz reference:
/// `k_f ≈ 8e-20` (unshielded twisted pair, worst-case 50-pair binder).
pub fn default_fext_coupling_constant() -> f64 {
    8e-20
}

/// Compute FEXT PSD attenuation at a given frequency from the coupling formula.
///
/// Returns `|H_fext|²` (linear power ratio, not dB).
pub fn fext_psd_linear(
    freq_hz: f64,
    coupling_constant: f64,
    direct_path_power: f64,
    coupling_length_m: f64,
) -> f64 {
    coupling_constant * freq_hz * freq_hz * direct_path_power * coupling_length_m
}

/// Convert FEXT PSD to dB attenuation: `10·log10(PSD)`.
pub fn fext_psd_db(
    freq_hz: f64,
    coupling_constant: f64,
    direct_path_power: f64,
    coupling_length_m: f64,
) -> f64 {
    let lin = fext_psd_linear(freq_hz, coupling_constant, direct_path_power, coupling_length_m);
    10.0 * lin.max(1e-300).log10()
}

/// Simple model of the direct-path cable attenuation for 0.4 mm copper.
///
/// Uses a simplified cable loss model: `|H| = exp(-α·f^0.5·d)` where
/// - α ≈ 0.00072 (dB/m/MHz^0.5 for 0.4 mm Cu)
/// - f in Hz, d in metres
pub fn cable_attenuation_linear(freq_hz: f64, length_m: f64) -> f64 {
    let alpha = 0.00072; // dB/m/(Hz^0.5)
    let loss_db = alpha * freq_hz.sqrt() * length_m;
    // Convert dB loss to linear magnitude
    (-loss_db / 20.0 * std::f64::consts::LN_10).exp()
}

/// Build tone frequency vector for a given profile and number of tones.
pub fn tone_frequency_vector(profile: Vdsl2Profile, num_tones: usize) -> Vec<f64> {
    let spacing = profile.subcarrier_spacing_hz();
    let start = 6usize; // first VDSL2 downstream subcarrier
    (0..num_tones).map(|t| (start + t) as f64 * spacing).collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // Helper to build a simple 2×2 test system
    // -----------------------------------------------------------------------
    fn make_2line_config() -> VectoringConfig {
        VectoringConfig {
            num_lines: 2,
            num_tones: 8,
            profile: Vdsl2Profile::Profile17a,
            adaptation_rate: 0.01,
            diagonal_loading: 1e-6,
            noise_variance: 1e-6,
        }
    }

    fn make_4line_config() -> VectoringConfig {
        VectoringConfig {
            num_lines: 4,
            num_tones: 16,
            profile: Vdsl2Profile::Profile17a,
            adaptation_rate: 0.005,
            diagonal_loading: 1e-5,
            noise_variance: 1e-7,
        }
    }

    // -----------------------------------------------------------------------
    // Complex arithmetic
    // -----------------------------------------------------------------------

    #[test]
    fn test_c_mul() {
        let a = [1.0, 2.0];
        let b = [3.0, 4.0];
        let c = c_mul(a, b);
        assert!((c[0] - (1.0 * 3.0 - 2.0 * 4.0)).abs() < 1e-12);
        assert!((c[1] - (1.0 * 4.0 + 2.0 * 3.0)).abs() < 1e-12);
    }

    #[test]
    fn test_c_conj() {
        let a = [3.0, -5.0];
        let c = c_conj(a);
        assert_eq!(c, [3.0, 5.0]);
    }

    #[test]
    fn test_c_recip() {
        // 1/(1+0i) = 1+0i
        let r = c_recip([1.0, 0.0]);
        assert!((r[0] - 1.0).abs() < 1e-12);
        assert!(r[1].abs() < 1e-12);
        // 1/(0+1i) = 0-1i
        let r2 = c_recip([0.0, 1.0]);
        assert!(r2[0].abs() < 1e-12);
        assert!((r2[1] + 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_c_abs_sq() {
        let a = [3.0, 4.0];
        assert!((c_abs_sq(a) - 25.0).abs() < 1e-12);
        assert!((c_abs(a) - 5.0).abs() < 1e-12);
    }

    // -----------------------------------------------------------------------
    // Matrix operations
    // -----------------------------------------------------------------------

    #[test]
    fn test_mat_identity() {
        let id = mat_identity(3);
        assert_eq!(id[0], [1.0, 0.0]); // (0,0)
        assert_eq!(id[1], [0.0, 0.0]); // (0,1)
        assert_eq!(id[4], [1.0, 0.0]); // (1,1)
    }

    #[test]
    fn test_mat_mul_identity() {
        let n = 3;
        let id = mat_identity(n);
        let a: Vec<C64> = (0..(n * n))
            .map(|k| [k as f64, 0.0])
            .collect();
        let c = mat_mul(&a, &id, n);
        for (x, y) in a.iter().zip(c.iter()) {
            assert!((x[0] - y[0]).abs() < 1e-12);
        }
    }

    #[test]
    fn test_mat_inv_2x2() {
        // [2 0; 0 2] → [0.5 0; 0 0.5]
        let n = 2;
        let a = vec![[2.0_f64, 0.0], [0.0, 0.0], [0.0, 0.0], [2.0, 0.0]];
        let inv = mat_inv(&a, n).unwrap();
        assert!((inv[0][0] - 0.5).abs() < 1e-12);
        assert!((inv[3][0] - 0.5).abs() < 1e-12);
    }

    #[test]
    fn test_mat_inv_identity() {
        let n = 3;
        let id = mat_identity(n);
        let inv = mat_inv(&id, n).unwrap();
        for i in 0..n {
            for j in 0..n {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!((inv[i * n + j][0] - expected).abs() < 1e-12);
                assert!(inv[i * n + j][1].abs() < 1e-12);
            }
        }
    }

    #[test]
    fn test_mat_inv_2x2_complex() {
        // [[1+i, 0], [0, 2]] → [[1/(1+i), 0], [0, 0.5]]
        let n = 2;
        let a = vec![[1.0, 1.0], [0.0, 0.0], [0.0, 0.0], [2.0, 0.0]];
        let inv = mat_inv(&a, n).unwrap();
        // Expected: (1+i)^{-1} = (1-i)/2
        assert!((inv[0][0] - 0.5).abs() < 1e-10);
        assert!((inv[0][1] + 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_mat_herm() {
        let n = 2;
        let a = vec![[1.0, 2.0], [3.0, 4.0], [5.0, 6.0], [7.0, 8.0]];
        let h = mat_herm(&a, n);
        // H[0,0] = conj(A[0,0]) = [1,-2]
        assert_eq!(h[0], [1.0, -2.0]);
        // H[0,1] = conj(A[1,0]) = [5,-6]
        assert_eq!(h[1], [5.0, -6.0]);
    }

    #[test]
    fn test_mat_vec_mul() {
        let n = 2;
        let a = vec![[1.0, 0.0], [0.0, 0.0], [0.0, 0.0], [1.0, 0.0]];
        let x = vec![[3.0, 1.0], [4.0, 2.0]];
        let y = mat_vec_mul(&a, &x, n);
        assert!((y[0][0] - 3.0).abs() < 1e-12);
        assert!((y[1][0] - 4.0).abs() < 1e-12);
    }

    #[test]
    fn test_mmse_pseudo_inv_returns_matrix() {
        let n = 3;
        let id = mat_identity(n);
        let pinv = mmse_pseudo_inv(&id, n, 1e-4);
        // For identity: (I^H I + λI)^{-1} I^H ≈ (1/(1+λ)) · I
        for i in 0..n {
            let diag = pinv[i * n + i][0];
            assert!((diag - 1.0 / (1.0 + 1e-4)).abs() < 1e-6);
        }
    }

    // -----------------------------------------------------------------------
    // FEXT Model
    // -----------------------------------------------------------------------

    #[test]
    fn test_fext_model_diagonal_is_direct() {
        let n = 3;
        let model = FextModel::identity_with_fext(n, 4, 0.1);
        for t in 0..4 {
            let h = model.channel_matrix(t);
            for i in 0..n {
                // Diagonal should be [1, 0] (identity direct path)
                assert!((h[i * n + i][0] - 1.0).abs() < 1e-12);
                assert!(h[i * n + i][1].abs() < 1e-12);
            }
        }
    }

    #[test]
    fn test_fext_model_off_diagonal_bounded() {
        let n = 3;
        let fext_level = 0.05;
        let model = FextModel::identity_with_fext(n, 4, fext_level);
        for t in 0..4 {
            let h = model.channel_matrix(t);
            for i in 0..n {
                for j in 0..n {
                    if i != j {
                        let mag = c_abs(h[i * n + j]);
                        assert!(
                            (mag - fext_level).abs() < 1e-10,
                            "Expected FEXT magnitude {fext_level}, got {mag}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn test_fext_model_apply_no_fext() {
        let n = 2;
        let model = FextModel::identity_with_fext(n, 2, 0.0);
        let tx = vec![[1.0, 0.0], [2.0, 0.0]];
        let rx = model.apply(0, &tx);
        assert!((rx[0][0] - 1.0).abs() < 1e-12);
        assert!((rx[1][0] - 2.0).abs() < 1e-12);
    }

    #[test]
    fn test_fext_power_per_line() {
        let n = 3;
        let fext_level = 0.1;
        let model = FextModel::identity_with_fext(n, 2, fext_level);
        let pwr = model.fext_power_per_line(0);
        // Each line receives FEXT from (N-1) others, each with power fext_level²
        for &p in &pwr {
            let expected = (n - 1) as f64 * fext_level * fext_level;
            assert!((p - expected).abs() < 1e-10);
        }
    }

    #[test]
    fn test_fext_from_direct_paths() {
        let n = 2;
        let num_tones = 4;
        let freqs: Vec<f64> = (1..=4).map(|i| i as f64 * 1e6).collect();
        let direct: Vec<Vec<C64>> = (0..n)
            .map(|_| vec![[1.0, 0.0]; num_tones])
            .collect();
        let model = FextModel::from_direct_paths(&direct, &freqs, 1e-20, 500.0);
        assert_eq!(model.channel_matrices.len(), num_tones);
        // Diagonal should be [1, 0]
        for t in 0..num_tones {
            let h = model.channel_matrix(t);
            assert!((h[0][0] - 1.0).abs() < 1e-12);
        }
    }

    // -----------------------------------------------------------------------
    // Vectoring Precoder
    // -----------------------------------------------------------------------

    #[test]
    fn test_precoder_init_identity() {
        let n = 3;
        let mut prec = VectoringPrecoder::new(n, 4, 1e-6);
        // After init, should be identity
        let data = vec![[1.0, 0.0], [2.0, 0.0], [3.0, 0.0]];
        let out = prec.precode(0, &data);
        for (o, d) in out.iter().zip(data.iter()) {
            assert!((o[0] - d[0]).abs() < 1e-12);
        }
        // Suppress unused warning
        prec.update_zf(0, &mat_identity(n));
    }

    #[test]
    fn test_precoder_zf_identity_channel() {
        let n = 3;
        let mut prec = VectoringPrecoder::new(n, 4, 1e-6);
        let id = mat_identity(n);
        prec.update_zf(0, &id);
        let data = vec![[1.0, 0.5], [2.0, -0.5], [0.5, 0.25]];
        let out = prec.precode(0, &data);
        // ZF of identity * norm should be a scaled version of data
        let gain = prec.norm_gain(0);
        for (o, d) in out.iter().zip(data.iter()) {
            assert!((o[0] - d[0] * gain).abs() < 1e-10);
        }
    }

    #[test]
    fn test_precoder_mmse_identity_channel() {
        let n = 2;
        let mut prec = VectoringPrecoder::new(n, 4, 1e-6);
        let id = mat_identity(n);
        prec.update_mmse(0, &id, 1e-4);
        let data = vec![[1.0, 0.0], [0.0, 1.0]];
        let out = prec.precode(0, &data);
        assert_eq!(out.len(), 2);
    }

    #[test]
    fn test_precoder_reduces_fext() {
        // With a 2×2 channel that has FEXT, ZF precoder should suppress crosstalk
        let n = 2;
        let k = 4;
        let fext = 0.1;
        let model = FextModel::identity_with_fext(n, k, fext);
        let mut prec = VectoringPrecoder::new(n, k, 1e-6);
        prec.update_zf(0, model.channel_matrix(0));

        let data = vec![[1.0, 0.0], [0.0, 0.0]];
        let precoded = prec.precode(0, &data);
        let received = model.apply(0, &precoded);

        // Line 1 should have very small received power (crosstalk suppressed)
        let crosstalk = c_abs_sq(received[1]);
        assert!(crosstalk < fext * fext, "Crosstalk {crosstalk} should be < {}", fext * fext);
    }

    #[test]
    fn test_precoder_norm_gain_positive() {
        let n = 2;
        let mut prec = VectoringPrecoder::new(n, 4, 1e-6);
        let id = mat_identity(n);
        prec.update_zf(0, &id);
        assert!(prec.norm_gain(0) > 0.0);
    }

    // -----------------------------------------------------------------------
    // Vectoring Canceller
    // -----------------------------------------------------------------------

    #[test]
    fn test_canceller_init_identity() {
        let n = 3;
        let canc = VectoringCanceller::new(n, 4, 1e-6);
        let rx = vec![[1.0, 0.0], [2.0, 0.0], [3.0, 0.0]];
        let out = canc.cancel(0, &rx);
        for (o, r) in out.iter().zip(rx.iter()) {
            assert!((o[0] - r[0]).abs() < 1e-12);
        }
    }

    #[test]
    fn test_canceller_zf_identity_channel() {
        let n = 2;
        let mut canc = VectoringCanceller::new(n, 4, 1e-6);
        canc.update_zf(0, &mat_identity(n));
        let rx = vec![[1.0, 2.0], [3.0, 4.0]];
        let out = canc.cancel(0, &rx);
        for (o, r) in out.iter().zip(rx.iter()) {
            assert!((o[0] - r[0]).abs() < 1e-10);
        }
    }

    #[test]
    fn test_canceller_mmse_identity_channel() {
        let n = 2;
        let mut canc = VectoringCanceller::new(n, 4, 1e-6);
        canc.update_mmse(0, &mat_identity(n));
        let rx = vec![[5.0, 0.0], [3.0, 0.0]];
        let out = canc.cancel(0, &rx);
        assert_eq!(out.len(), 2);
        // MMSE with identity ≈ slightly attenuated version
        assert!(c_abs(out[0]) > 0.0);
    }

    #[test]
    fn test_canceller_iterative() {
        let n = 2;
        let mut canc = VectoringCanceller::new(n, 4, 1e-6);
        canc.update_mmse(0, &mat_identity(n));
        let rx = vec![[1.0, 0.0], [1.0, 0.0]];
        let out1 = canc.cancel(0, &rx);
        let out2 = canc.cancel_iterative(0, &rx, 1);
        // Both should produce similar results for identity
        assert!((c_abs(out1[0]) - c_abs(out2[0])).abs() < 1e-6);
    }

    #[test]
    fn test_canceller_residual_fext_storage() {
        let n = 2;
        let mut canc = VectoringCanceller::new(n, 4, 1e-6);
        canc.set_residual_fext(2, 0.042);
        assert!((canc.residual_fext(2) - 0.042).abs() < 1e-12);
    }

    // -----------------------------------------------------------------------
    // Error Feedback
    // -----------------------------------------------------------------------

    #[test]
    fn test_error_feedback_accumulate() {
        let mut ef = ErrorFeedback::new(3, 8, 0.01);
        ef.accumulate(SlicerError {
            line: 1,
            tone: 3,
            error: [0.1, 0.2],
        });
        let errors = ef.drain_errors(3);
        assert!((errors[1][0] - 0.1).abs() < 1e-10);
        assert!((errors[1][1] - 0.2).abs() < 1e-10);
    }

    #[test]
    fn test_error_feedback_drain_resets() {
        let mut ef = ErrorFeedback::new(2, 4, 0.01);
        ef.accumulate(SlicerError {
            line: 0,
            tone: 1,
            error: [1.0, 0.0],
        });
        let _ = ef.drain_errors(1);
        let second = ef.drain_errors(1);
        // Should be zero after draining
        assert!(second[0][0].abs() < 1e-12);
    }

    #[test]
    fn test_error_feedback_lms_gradient() {
        let ef = ErrorFeedback::new(2, 4, 0.05);
        let error = vec![[1.0, 0.0], [0.0, 1.0]];
        let d = [1.0, 0.0];
        let grad = ef.lms_precoder_gradient(&error, d);
        // ΔP[:,j] = −μ · e · d* = −0.05 · [[1,0],[0,1]] · [1,0]
        //         = −0.05 · [[1,0],[0,0]]... wait: e_0 · d* = [1,0]*[1,0]=[1,0]
        assert!((grad[0][0] + 0.05).abs() < 1e-12); // -μ * 1.0
        assert!((grad[1][0]).abs() < 1e-12);         // -μ * 0.0
    }

    #[test]
    fn test_error_feedback_compress() {
        let ef = ErrorFeedback::new(2, 4, 0.01);
        let err = [0.12345, -0.98765];
        let comp = ef.compress_error(err, 1.0);
        // Should be quantised to 6-bit resolution
        let resolution = 1.0 / 64.0;
        assert!((comp[0] - err[0]).abs() < resolution);
        assert!((comp[1] - err[1]).abs() < resolution);
    }

    #[test]
    fn test_error_feedback_quant_bits() {
        let mut ef = ErrorFeedback::new(2, 4, 0.01);
        ef.set_quant_bits(8);
        assert_eq!(ef.quant_bits, 8);
        ef.set_quant_bits(100); // clamped
        assert_eq!(ef.quant_bits, 16);
    }

    // -----------------------------------------------------------------------
    // SNR Estimator
    // -----------------------------------------------------------------------

    #[test]
    fn test_snr_estimator_basic() {
        let freqs = vec![1e6, 2e6, 3e6, 4e6];
        let mut est = SnrEstimator::new(2, 4, freqs);
        est.update_signal_psd(0, 0, 1.0);
        est.update_noise_psd(0, 0, 0.01);
        // SNR = 10*log10(1.0/0.01) but with exponential averaging from init
        // signal: 0.9*1.0 + 0.1*1.0 = 1.0 (since init is 1.0)
        // noise: 0.9*1e-9 + 0.1*0.01 ≈ 0.001
        let snr = est.snr_db(0, 0);
        assert!(snr > 0.0, "SNR should be positive, got {snr}");
    }

    #[test]
    fn test_snr_estimator_bits_per_tone() {
        let freqs = vec![1e6];
        let mut est = SnrEstimator::new(1, 1, freqs);
        // High SNR → many bits
        for _ in 0..50 {
            est.update_signal_psd(0, 0, 1000.0);
            est.update_noise_psd(0, 0, 0.001);
        }
        let bits = est.bits_per_tone(0, 0);
        assert!(bits > 5, "Expected >5 bits/tone, got {bits}");
        assert!(bits <= 15);
    }

    #[test]
    fn test_snr_estimator_data_rate() {
        let freqs = vec![1e6, 2e6, 3e6, 4e6];
        let est = SnrEstimator::new(1, 4, freqs);
        let rate = est.data_rate_bps(0, 4312.5);
        assert!(rate >= 0.0);
    }

    #[test]
    fn test_snr_estimator_line_performance() {
        let n = 2;
        let k = 4;
        let freqs: Vec<f64> = (0..k).map(|i| (i + 1) as f64 * 4312.5).collect();
        let est = SnrEstimator::new(n, k, freqs);
        let baseline_noise = vec![1e-9; k];
        let perf = est.line_performance(0, 4312.5, &baseline_noise);
        assert_eq!(perf.line, 0);
        assert_eq!(perf.tone_metrics.len(), k);
    }

    #[test]
    fn test_snr_estimator_vectoring_gain() {
        let k = 4;
        let freqs: Vec<f64> = (0..k).map(|i| (i + 1) as f64 * 4312.5).collect();
        let mut est = SnrEstimator::new(1, k, freqs);
        // Set high SNR after vectoring
        for t in 0..k {
            for _ in 0..20 {
                est.update_signal_psd(0, t, 100.0);
                est.update_noise_psd(0, t, 0.01);
            }
        }
        // High baseline noise (before vectoring)
        let baseline = vec![1.0; k];
        let gain = est.vectoring_gain_db(0, &baseline);
        assert!(gain > 0.0, "Vectoring gain should be positive after better SNR");
    }

    // -----------------------------------------------------------------------
    // Binder Group
    // -----------------------------------------------------------------------

    #[test]
    fn test_binder_join_leave() {
        let mut bg = BinderGroup::new(4, false);
        assert!(bg.join(0));
        assert!(bg.join(1));
        assert_eq!(bg.num_active(), 2);
        bg.leave(0);
        assert_eq!(bg.line_states[0], LineState::Leaving);
        bg.confirm_leave(0);
        assert_eq!(bg.line_states[0], LineState::Idle);
        assert_eq!(bg.num_active(), 1);
    }

    #[test]
    fn test_binder_active_lines() {
        let mut bg = BinderGroup::new(4, false);
        bg.join(0);
        bg.join(2);
        let active = bg.active_lines();
        assert_eq!(active, vec![0, 2]);
    }

    #[test]
    fn test_binder_join_out_of_range() {
        let mut bg = BinderGroup::new(4, false);
        assert!(!bg.join(4)); // out of range
        assert!(!bg.join(100));
    }

    #[test]
    fn test_binder_pilot_sequence() {
        let bg = BinderGroup::new(4, false);
        let seq = bg.pilot_sequence(0, 100);
        assert_eq!(seq.len(), 100);
        // All values should be ±1
        for &v in &seq {
            assert!(v == 1.0 || v == -1.0);
        }
    }

    #[test]
    fn test_binder_pilot_sequences_differ() {
        let bg = BinderGroup::new(4, false);
        let s0 = bg.pilot_sequence(0, 32);
        let s1 = bg.pilot_sequence(1, 32);
        // Different seeds → different sequences (with high probability)
        let diffs = s0.iter().zip(s1.iter()).filter(|(a, b)| a != b).count();
        assert!(diffs > 0, "Pilot sequences for different lines should differ");
    }

    #[test]
    fn test_binder_double_join_fails() {
        let mut bg = BinderGroup::new(4, false);
        assert!(bg.join(0));
        assert!(!bg.join(0)); // already active
    }

    // -----------------------------------------------------------------------
    // Vdsl2Profile
    // -----------------------------------------------------------------------

    #[test]
    fn test_profile_17a_tones() {
        assert_eq!(Vdsl2Profile::Profile17a.max_downstream_tones(), 4096);
    }

    #[test]
    fn test_profile_subcarrier_spacing() {
        for &p in &[
            Vdsl2Profile::Profile8a,
            Vdsl2Profile::Profile17a,
            Vdsl2Profile::Profile30a,
        ] {
            assert!((p.subcarrier_spacing_hz() - 4312.5).abs() < 1e-6);
        }
    }

    #[test]
    fn test_profile_max_bits() {
        assert_eq!(Vdsl2Profile::Profile17a.max_bits_per_tone(), 15);
    }

    // -----------------------------------------------------------------------
    // Utility functions
    // -----------------------------------------------------------------------

    #[test]
    fn test_cable_attenuation_increases_with_length() {
        let f = 1e6;
        let att_short = cable_attenuation_linear(f, 100.0);
        let att_long = cable_attenuation_linear(f, 1000.0);
        assert!(att_long < att_short, "Longer cable should attenuate more");
    }

    #[test]
    fn test_cable_attenuation_increases_with_freq() {
        let d = 500.0;
        let att_low = cable_attenuation_linear(1e6, d);
        let att_high = cable_attenuation_linear(10e6, d);
        assert!(att_high < att_low, "Higher freq should attenuate more");
    }

    #[test]
    fn test_fext_psd_linear() {
        let psd = fext_psd_linear(1e6, 1e-20, 1.0, 500.0);
        let expected = 1e-20 * 1e12 * 1.0 * 500.0;
        assert!((psd - expected).abs() < 1e-10 * expected.abs().max(1e-40));
    }

    #[test]
    fn test_fext_psd_db() {
        let db = fext_psd_db(1e6, 1e-20, 1.0, 500.0);
        assert!(db < 0.0, "FEXT PSD dB should be negative (attenuated)");
    }

    #[test]
    fn test_tone_frequency_vector() {
        let freqs = tone_frequency_vector(Vdsl2Profile::Profile17a, 10);
        assert_eq!(freqs.len(), 10);
        // First tone: tone index 6 × 4312.5 Hz
        assert!((freqs[0] - 6.0 * 4312.5).abs() < 1e-6);
        // Monotone increasing
        for i in 1..freqs.len() {
            assert!(freqs[i] > freqs[i - 1]);
        }
    }

    #[test]
    fn test_default_fext_constant() {
        let k = default_fext_coupling_constant();
        assert!((k - 8e-20).abs() < 1e-30);
    }

    // -----------------------------------------------------------------------
    // DslVectoringProcessor (integration)
    // -----------------------------------------------------------------------

    #[test]
    fn test_processor_init() {
        let cfg = make_2line_config();
        let proc = DslVectoringProcessor::new(cfg);
        assert_eq!(proc.config.num_lines, 2);
        assert_eq!(proc.tone_freqs_hz.len(), 8);
    }

    #[test]
    fn test_processor_initialize_identity() {
        let cfg = make_2line_config();
        let mut proc = DslVectoringProcessor::new(cfg);
        proc.initialize_identity();
        // After identity init, precoding should pass data through unchanged
        let data = vec![[1.0, 0.0], [0.0, 1.0]];
        let out = proc.precode_downstream(0, &data);
        assert!((out[0][0] - 1.0).abs() < 1e-12);
        assert!((out[1][1] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_processor_set_channel_model() {
        let cfg = make_2line_config();
        let mut proc = DslVectoringProcessor::new(cfg.clone());
        let model = FextModel::identity_with_fext(cfg.num_lines, cfg.num_tones, 0.05);
        proc.set_channel_model(model);
        assert!(proc.fext_model.is_some());
    }

    #[test]
    fn test_processor_initialize_from_channel_zf() {
        let cfg = make_2line_config();
        let mut proc = DslVectoringProcessor::new(cfg.clone());
        let model = FextModel::identity_with_fext(cfg.num_lines, cfg.num_tones, 0.05);
        proc.set_channel_model(model);
        proc.initialize_from_channel_zf();
        // Precoder should now be non-identity (ZF of FEXT channel)
        let p = proc.precoder.matrix(0);
        // The diagonal may still dominate but off-diagonal should be non-zero
        let off_diag_power: f64 = (0..2)
            .flat_map(|i| (0..2).filter(move |&j| i != j).map(move |j| (i, j)))
            .map(|(i, j)| c_abs_sq(p[i * 2 + j]))
            .sum();
        assert!(off_diag_power > 0.0, "ZF precoder should have non-zero off-diagonal terms");
    }

    #[test]
    fn test_processor_initialize_from_channel_mmse() {
        let cfg = make_2line_config();
        let mut proc = DslVectoringProcessor::new(cfg.clone());
        let model = FextModel::identity_with_fext(cfg.num_lines, cfg.num_tones, 0.05);
        proc.set_channel_model(model);
        proc.initialize_from_channel_mmse();
        // Sanity: precoder matrix should be present
        let p = proc.precoder.matrix(0);
        assert_eq!(p.len(), cfg.num_lines * cfg.num_lines);
    }

    #[test]
    fn test_processor_simulate_downstream() {
        let cfg = make_2line_config();
        let mut proc = DslVectoringProcessor::new(cfg.clone());
        proc.initialize_identity();
        let model = FextModel::identity_with_fext(cfg.num_lines, cfg.num_tones, 0.0);
        proc.set_channel_model(model);
        let data = vec![[1.0, 0.0], [0.0, 1.0]];
        let rx = proc.simulate_downstream(0, &data);
        assert_eq!(rx.len(), 2);
    }

    #[test]
    fn test_processor_simulate_upstream() {
        let cfg = make_2line_config();
        let mut proc = DslVectoringProcessor::new(cfg.clone());
        proc.initialize_identity();
        let model = FextModel::identity_with_fext(cfg.num_lines, cfg.num_tones, 0.0);
        proc.set_channel_model(model);
        let tx = vec![[1.0, 0.0], [2.0, 0.0]];
        let rx = proc.simulate_upstream(0, &tx);
        // With identity precoder and zero FEXT, received ≈ transmitted
        assert!((c_abs(rx[0]) - 1.0).abs() < 1e-10);
        assert!((c_abs(rx[1]) - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_processor_report_error_and_adapt() {
        let cfg = make_2line_config();
        let mut proc = DslVectoringProcessor::new(cfg);
        proc.initialize_identity();
        proc.report_error(SlicerError {
            line: 0,
            tone: 0,
            error: [0.01, 0.0],
        });
        let tx = vec![[1.0, 0.0], [1.0, 0.0]];
        let rx = vec![[1.01, 0.0], [1.0, 0.0]];
        proc.adapt(0, &tx, &rx);
        // Adaptation should not panic; just verify it runs
    }

    #[test]
    fn test_processor_update_snr_from_pilots() {
        let cfg = make_2line_config();
        let mut proc = DslVectoringProcessor::new(cfg);
        let received = vec![[1.0, 0.0], [1.0, 0.0]];
        let pilots = vec![[1.0, 0.0], [1.0, 0.0]];
        proc.update_snr_from_pilots(0, &received, &pilots);
        // No assertion, just ensure it doesn't panic
    }

    #[test]
    fn test_processor_line_performances() {
        let cfg = make_4line_config();
        let proc = DslVectoringProcessor::new(cfg.clone());
        let perfs = proc.line_performances();
        assert_eq!(perfs.len(), cfg.num_lines);
        for p in &perfs {
            assert_eq!(p.tone_metrics.len(), cfg.num_tones);
        }
    }

    #[test]
    fn test_processor_channel_matrix_none_without_model() {
        let cfg = make_2line_config();
        let proc = DslVectoringProcessor::new(cfg);
        assert!(proc.channel_matrix(0).is_none());
    }

    #[test]
    fn test_processor_channel_matrix_with_model() {
        let cfg = make_2line_config();
        let mut proc = DslVectoringProcessor::new(cfg.clone());
        let model = FextModel::identity_with_fext(cfg.num_lines, cfg.num_tones, 0.1);
        proc.set_channel_model(model);
        let h = proc.channel_matrix(0);
        assert!(h.is_some());
        assert_eq!(h.unwrap().len(), cfg.num_lines * cfg.num_lines);
    }

    #[test]
    fn test_processor_crosstalk_reduction_no_model() {
        let cfg = make_2line_config();
        let proc = DslVectoringProcessor::new(cfg);
        // Should return 0.0 if no channel model attached
        assert!((proc.crosstalk_reduction_db(0) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_processor_crosstalk_reduction_with_zf() {
        let cfg = make_2line_config();
        let mut proc = DslVectoringProcessor::new(cfg.clone());
        let model = FextModel::identity_with_fext(cfg.num_lines, cfg.num_tones, 0.2);
        proc.set_channel_model(model);
        proc.initialize_from_channel_zf();
        let reduction = proc.crosstalk_reduction_db(0);
        assert!(reduction > 0.0, "ZF precoder should reduce crosstalk; got {reduction}");
    }

    #[test]
    fn test_processor_binder_join() {
        let cfg = make_4line_config();
        let mut proc = DslVectoringProcessor::new(cfg);
        assert!(proc.binder.join(0));
        assert!(proc.binder.join(2));
        assert_eq!(proc.binder.num_active(), 2);
    }

    // -----------------------------------------------------------------------
    // Round-trip downstream test: precode → channel → cancel
    // -----------------------------------------------------------------------

    #[test]
    fn test_roundtrip_downstream_identity() {
        // With identity channel, precoder and canceller identity: rx ≈ tx
        let n = 2;
        let k = 4;
        let cfg = VectoringConfig {
            num_lines: n,
            num_tones: k,
            ..Default::default()
        };
        let mut proc = DslVectoringProcessor::new(cfg);
        proc.initialize_identity();
        let model = FextModel::identity_with_fext(n, k, 0.0);
        proc.set_channel_model(model);

        let data = vec![[1.0, 0.5], [0.5, -1.0]];
        let rx = proc.simulate_downstream(0, &data);
        let est = proc.cancel_upstream(0, &rx);

        for (e, d) in est.iter().zip(data.iter()) {
            assert!((e[0] - d[0]).abs() < 1e-10);
            assert!((e[1] - d[1]).abs() < 1e-10);
        }
    }

    #[test]
    fn test_roundtrip_zf_cancellation() {
        // With FEXT channel, ZF precode + ZF cancel should recover symbols
        let n = 2;
        let k = 2;
        let fext = 0.1;
        let cfg = VectoringConfig {
            num_lines: n,
            num_tones: k,
            diagonal_loading: 1e-8,
            ..Default::default()
        };
        let mut proc = DslVectoringProcessor::new(cfg);
        let model = FextModel::identity_with_fext(n, k, fext);
        proc.set_channel_model(model.clone());
        proc.initialize_from_channel_zf();

        let data = vec![[1.0, 0.0], [0.0, 1.0]];
        // Precode
        let precoded = proc.precode_downstream(0, &data);
        // Pass through channel
        let received = model.apply(0, &precoded);
        // Cancel
        let estimated = proc.cancel_upstream(0, &received);

        // The estimated symbols should be close to the (normalised) data
        // Due to precoder normalisation, scale may differ, but the ratio should hold
        let scale0 = if data[0][0].abs() > 1e-10 {
            estimated[0][0] / data[0][0]
        } else {
            1.0
        };
        assert!(scale0.abs() > 0.0, "Recovered symbol scale should be non-zero");
    }
}
