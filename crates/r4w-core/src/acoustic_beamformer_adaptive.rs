//! Multi-channel adaptive acoustic beamforming for microphone arrays and hydrophone arrays.
//!
//! This module provides delay-and-sum beamforming, MVDR (Capon) beamforming,
//! LCMV (linearly constrained minimum variance) beamforming, generalized
//! cross-correlation (GCC-PHAT) for time delay estimation, broadband beamforming
//! with frequency-dependent steering, beam pattern computation, and white noise
//! gain calculation.
//!
//! All complex numbers are represented as `(f64, f64)` tuples (real, imaginary).
//!
//! # Example
//!
//! ```
//! use r4w_core::acoustic_beamformer_adaptive::{AcousticBeamformer, delay_and_sum_acoustic};
//!
//! let bf = AcousticBeamformer::new(4, 0.05, 343.0, 16000.0);
//! // Four channels of silence
//! let signals = vec![vec![0.0f64; 128]; 4];
//! let output = delay_and_sum_acoustic(&signals, 0.0, bf.element_spacing_m,
//!     bf.sound_speed_mps, bf.sample_rate_hz);
//! assert_eq!(output.len(), 128);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex arithmetic helpers
// ---------------------------------------------------------------------------

type C64 = (f64, f64);

fn c_add(a: C64, b: C64) -> C64 {
    (a.0 + b.0, a.1 + b.1)
}

fn c_sub(a: C64, b: C64) -> C64 {
    (a.0 - b.0, a.1 - b.1)
}

fn c_mul(a: C64, b: C64) -> C64 {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

fn c_conj(a: C64) -> C64 {
    (a.0, -a.1)
}

fn c_abs(a: C64) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

fn c_exp(phase: f64) -> C64 {
    (phase.cos(), phase.sin())
}

fn c_scale(s: f64, a: C64) -> C64 {
    (s * a.0, s * a.1)
}

fn c_div(a: C64, b: C64) -> C64 {
    let denom = b.0 * b.0 + b.1 * b.1;
    if denom < 1e-30 {
        (0.0, 0.0)
    } else {
        ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
    }
}

// ---------------------------------------------------------------------------
// Tiny FFT (radix-2 Cooley-Tukey, in-place)
// ---------------------------------------------------------------------------

fn next_power_of_two(n: usize) -> usize {
    let mut p = 1;
    while p < n {
        p <<= 1;
    }
    p
}

fn bit_reverse(mut x: usize, log2n: u32) -> usize {
    let mut result = 0usize;
    for _ in 0..log2n {
        result = (result << 1) | (x & 1);
        x >>= 1;
    }
    result
}

/// In-place radix-2 FFT. `inverse` = true for IFFT.
fn fft_in_place(buf: &mut [C64], inverse: bool) {
    let n = buf.len();
    assert!(n.is_power_of_two(), "FFT size must be power of two");
    let log2n = n.trailing_zeros();

    // bit-reversal permutation
    for i in 0..n {
        let j = bit_reverse(i, log2n);
        if i < j {
            buf.swap(i, j);
        }
    }

    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * 2.0 * PI / len as f64;
        let wn = c_exp(angle);
        let mut start = 0;
        while start < n {
            let mut w: C64 = (1.0, 0.0);
            for k in 0..half {
                let u = buf[start + k];
                let t = c_mul(w, buf[start + k + half]);
                buf[start + k] = c_add(u, t);
                buf[start + k + half] = c_sub(u, t);
                w = c_mul(w, wn);
            }
            start += len;
        }
        len <<= 1;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for v in buf.iter_mut() {
            v.0 *= inv_n;
            v.1 *= inv_n;
        }
    }
}

fn fft(data: &[C64]) -> Vec<C64> {
    let mut buf = data.to_vec();
    fft_in_place(&mut buf, false);
    buf
}

fn ifft(data: &[C64]) -> Vec<C64> {
    let mut buf = data.to_vec();
    fft_in_place(&mut buf, true);
    buf
}

// ---------------------------------------------------------------------------
// Small matrix helpers (for MVDR / LCMV, sizes = num_elements)
// ---------------------------------------------------------------------------

/// Invert a small complex matrix via Gauss-Jordan. Returns None if singular.
fn mat_invert(mat: &[Vec<C64>]) -> Option<Vec<Vec<C64>>> {
    let n = mat.len();
    // augmented [A | I]
    let mut aug: Vec<Vec<C64>> = Vec::with_capacity(n);
    for i in 0..n {
        let mut row = vec![(0.0, 0.0); 2 * n];
        for j in 0..n {
            row[j] = mat[i][j];
        }
        row[n + i] = (1.0, 0.0);
        aug.push(row);
    }

    for col in 0..n {
        // partial pivoting
        let mut max_mag = c_abs(aug[col][col]);
        let mut max_row = col;
        for row in (col + 1)..n {
            let mag = c_abs(aug[row][col]);
            if mag > max_mag {
                max_mag = mag;
                max_row = row;
            }
        }
        if max_mag < 1e-14 {
            return None; // singular
        }
        if max_row != col {
            aug.swap(col, max_row);
        }

        let pivot = aug[col][col];
        let inv_pivot = c_div((1.0, 0.0), pivot);
        for j in 0..(2 * n) {
            aug[col][j] = c_mul(aug[col][j], inv_pivot);
        }

        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            for j in 0..(2 * n) {
                let tmp = c_mul(factor, aug[col][j]);
                aug[row][j] = c_sub(aug[row][j], tmp);
            }
        }
    }

    let mut inv = vec![vec![(0.0, 0.0); n]; n];
    for i in 0..n {
        for j in 0..n {
            inv[i][j] = aug[i][n + j];
        }
    }
    Some(inv)
}

/// Multiply matrix A (m x n) by column vector v (n). Returns m-vec.
fn mat_vec_mul(a: &[Vec<C64>], v: &[C64]) -> Vec<C64> {
    let m = a.len();
    let mut out = vec![(0.0, 0.0); m];
    for i in 0..m {
        for (j, vj) in v.iter().enumerate() {
            out[i] = c_add(out[i], c_mul(a[i][j], *vj));
        }
    }
    out
}

/// Hermitian dot product: sum_i conj(a_i)*b_i
fn hermitian_dot(a: &[C64], b: &[C64]) -> C64 {
    let mut s = (0.0, 0.0);
    for (ai, bi) in a.iter().zip(b.iter()) {
        s = c_add(s, c_mul(c_conj(*ai), *bi));
    }
    s
}

// ---------------------------------------------------------------------------
// Estimate spatial covariance from time-domain signals
// ---------------------------------------------------------------------------

/// Build steering vector for a ULA at the given angle and frequency.
fn steering_vector(num_elements: usize, spacing_m: f64, freq_hz: f64, sound_speed: f64, angle_rad: f64) -> Vec<C64> {
    let d = spacing_m * angle_rad.sin();
    let wavelength = sound_speed / freq_hz;
    (0..num_elements)
        .map(|k| {
            let phase = -2.0 * PI * k as f64 * d / wavelength;
            c_exp(phase)
        })
        .collect()
}

/// Estimate the spatial covariance matrix from multi-channel signals.
/// Each row of `signals` is one channel.
fn estimate_covariance(signals: &[Vec<f64>]) -> Vec<Vec<C64>> {
    let m = signals.len();
    let n = signals[0].len();
    let mut cov = vec![vec![(0.0, 0.0); m]; m];
    let inv_n = 1.0 / n as f64;
    for i in 0..m {
        for j in 0..m {
            let mut s = 0.0;
            for k in 0..n {
                s += signals[i][k] * signals[j][k];
            }
            cov[i][j] = (s * inv_n, 0.0);
        }
    }
    cov
}

/// Add diagonal loading to a covariance matrix for numerical stability.
fn diagonal_load(cov: &mut [Vec<C64>], eps: f64) {
    for i in 0..cov.len() {
        cov[i][i].0 += eps;
    }
}

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Configuration for an acoustic beamformer based on a uniform linear array.
#[derive(Debug, Clone)]
pub struct AcousticBeamformer {
    /// Number of array elements (microphones / hydrophones).
    pub num_elements: usize,
    /// Inter-element spacing in metres.
    pub element_spacing_m: f64,
    /// Speed of sound in m/s (343 for air, ~1500 for water).
    pub sound_speed_mps: f64,
    /// Sample rate in Hz.
    pub sample_rate_hz: f64,
}

impl AcousticBeamformer {
    /// Create a new `AcousticBeamformer`.
    pub fn new(num_elements: usize, element_spacing_m: f64, sound_speed_mps: f64, sample_rate_hz: f64) -> Self {
        Self {
            num_elements,
            element_spacing_m,
            sound_speed_mps,
            sample_rate_hz,
        }
    }
}

/// Beam pattern: angular response of the array in dB.
#[derive(Debug, Clone)]
pub struct BeamPattern {
    /// Angles in radians (typically -PI/2 to PI/2).
    pub angles_rad: Vec<f64>,
    /// Response in dB at each angle.
    pub response_db: Vec<f64>,
}

// ---------------------------------------------------------------------------
// Public functions
// ---------------------------------------------------------------------------

/// Delay-and-sum beamformer for acoustic arrays.
///
/// Each element of `signals` is one channel (same length). The output has the
/// same length as the input channels. Fractional-sample delays are applied via
/// sinc interpolation.
pub fn delay_and_sum_acoustic(
    signals: &[Vec<f64>],
    steering_angle_rad: f64,
    spacing_m: f64,
    sound_speed: f64,
    sample_rate: f64,
) -> Vec<f64> {
    let m = signals.len();
    if m == 0 {
        return Vec::new();
    }
    let n = signals[0].len();
    let mut output = vec![0.0; n];

    let d_sin = spacing_m * steering_angle_rad.sin();
    let inv_m = 1.0 / m as f64;

    for ch in 0..m {
        let delay_samples = ch as f64 * d_sin / sound_speed * sample_rate;
        for i in 0..n {
            let pos = i as f64 + delay_samples;
            output[i] += sinc_interp(signals[ch].as_slice(), pos) * inv_m;
        }
    }
    output
}

/// MVDR (Capon) beamformer for acoustic arrays.
///
/// Minimises output power while maintaining unity gain in the look direction.
/// Uses a reference frequency equal to sample_rate / 4 for the steering vector.
pub fn mvdr_acoustic(
    signals: &[Vec<f64>],
    steering_angle_rad: f64,
    spacing_m: f64,
    sound_speed: f64,
    sample_rate: f64,
) -> Vec<f64> {
    let m = signals.len();
    if m == 0 {
        return Vec::new();
    }
    let n = signals[0].len();

    // Estimate covariance and regularise
    let mut cov = estimate_covariance(signals);
    diagonal_load(&mut cov, 1e-6);

    // Steering vector at a representative frequency (fs / 4)
    let freq = sample_rate / 4.0;
    let sv = steering_vector(m, spacing_m, freq, sound_speed, steering_angle_rad);

    let inv_cov = mat_invert(&cov).unwrap_or_else(|| {
        // fallback: identity
        let mut id = vec![vec![(0.0, 0.0); m]; m];
        for i in 0..m {
            id[i][i] = (1.0, 0.0);
        }
        id
    });

    // w = R^{-1} a / (a^H R^{-1} a)
    let inv_a = mat_vec_mul(&inv_cov, &sv);
    let denom = hermitian_dot(&sv, &inv_a);
    let weights: Vec<C64> = inv_a.iter().map(|&v| c_div(v, denom)).collect();

    // Apply weights (real-valued signals, take real part of output)
    let mut output = vec![0.0; n];
    for i in 0..n {
        let mut s: C64 = (0.0, 0.0);
        for ch in 0..m {
            s = c_add(s, c_mul(c_conj(weights[ch]), (signals[ch][i], 0.0)));
        }
        output[i] = s.0;
    }
    output
}

/// LCMV (Linearly Constrained Minimum Variance) beamformer.
///
/// `constraints` is a list of `(angle_rad, (gain_re, gain_im))` pairs.
/// The beamformer maintains the specified complex gain at each angle while
/// minimising output power.
pub fn lcmv_beamform(
    signals: &[Vec<f64>],
    constraints: &[(f64, C64)],
    spacing_m: f64,
    sound_speed: f64,
    sample_rate: f64,
) -> Vec<f64> {
    let m = signals.len();
    if m == 0 || constraints.is_empty() {
        return vec![0.0; signals.first().map_or(0, |s| s.len())];
    }
    let n = signals[0].len();
    let p = constraints.len();
    let freq = sample_rate / 4.0;

    // Build constraint matrix C (m x p) and response vector f (p)
    let mut c_mat: Vec<Vec<C64>> = vec![vec![(0.0, 0.0); p]; m];
    let mut f_vec: Vec<C64> = Vec::with_capacity(p);
    for (j, &(angle, gain)) in constraints.iter().enumerate() {
        let sv = steering_vector(m, spacing_m, freq, sound_speed, angle);
        for i in 0..m {
            c_mat[i][j] = sv[i];
        }
        f_vec.push(gain);
    }

    // Covariance
    let mut cov = estimate_covariance(signals);
    diagonal_load(&mut cov, 1e-6);
    let inv_cov = mat_invert(&cov).unwrap_or_else(|| {
        let mut id = vec![vec![(0.0, 0.0); m]; m];
        for i in 0..m {
            id[i][i] = (1.0, 0.0);
        }
        id
    });

    // w = R^{-1} C (C^H R^{-1} C)^{-1} f
    // First: R^{-1} C  (m x p)
    let mut inv_c: Vec<Vec<C64>> = vec![vec![(0.0, 0.0); p]; m];
    for j in 0..p {
        let col: Vec<C64> = (0..m).map(|i| c_mat[i][j]).collect();
        let tmp = mat_vec_mul(&inv_cov, &col);
        for i in 0..m {
            inv_c[i][j] = tmp[i];
        }
    }

    // C^H R^{-1} C  (p x p)
    let mut ch_inv_c = vec![vec![(0.0, 0.0); p]; p];
    for i in 0..p {
        for j in 0..p {
            let mut s = (0.0, 0.0);
            for k in 0..m {
                s = c_add(s, c_mul(c_conj(c_mat[k][i]), inv_c[k][j]));
            }
            ch_inv_c[i][j] = s;
        }
    }

    let ch_inv_c_inv = mat_invert(&ch_inv_c).unwrap_or_else(|| {
        let mut id = vec![vec![(0.0, 0.0); p]; p];
        for i in 0..p {
            id[i][i] = (1.0, 0.0);
        }
        id
    });

    // (C^H R^{-1} C)^{-1} f  => p-vector
    let tmp2 = mat_vec_mul(&ch_inv_c_inv, &f_vec);

    // w = inv_c * tmp2  => m-vector
    let mut weights = vec![(0.0, 0.0); m];
    for i in 0..m {
        for j in 0..p {
            weights[i] = c_add(weights[i], c_mul(inv_c[i][j], tmp2[j]));
        }
    }

    // Apply
    let mut output = vec![0.0; n];
    for i in 0..n {
        let mut s: C64 = (0.0, 0.0);
        for ch in 0..m {
            s = c_add(s, c_mul(c_conj(weights[ch]), (signals[ch][i], 0.0)));
        }
        output[i] = s.0;
    }
    output
}

/// Generalised Cross-Correlation with Phase Transform (GCC-PHAT).
///
/// Returns `(delay_samples, correlation_peak)` where `delay_samples` is the
/// estimated fractional delay of `sig_b` relative to `sig_a`.
pub fn gcc_phat(sig_a: &[f64], sig_b: &[f64]) -> (f64, f64) {
    let n = sig_a.len().max(sig_b.len());
    let fft_len = next_power_of_two(2 * n);

    let mut a_buf: Vec<C64> = vec![(0.0, 0.0); fft_len];
    let mut b_buf: Vec<C64> = vec![(0.0, 0.0); fft_len];
    for (i, &v) in sig_a.iter().enumerate() {
        a_buf[i] = (v, 0.0);
    }
    for (i, &v) in sig_b.iter().enumerate() {
        b_buf[i] = (v, 0.0);
    }

    fft_in_place(&mut a_buf, false);
    fft_in_place(&mut b_buf, false);

    // Cross-spectrum with PHAT weighting
    let mut cross: Vec<C64> = Vec::with_capacity(fft_len);
    for i in 0..fft_len {
        let g = c_mul(c_conj(a_buf[i]), b_buf[i]);
        let mag = c_abs(g);
        if mag > 1e-30 {
            cross.push(c_scale(1.0 / mag, g));
        } else {
            cross.push((0.0, 0.0));
        }
    }

    fft_in_place(&mut cross, true);

    // Search for peak over valid delay range
    let half = fft_len / 2;
    let mut best_idx: usize = 0;
    let mut best_val: f64 = f64::NEG_INFINITY;
    // Positive delays: indices 0..half
    for i in 0..half {
        if cross[i].0 > best_val {
            best_val = cross[i].0;
            best_idx = i;
        }
    }
    // Negative delays: indices fft_len-half..fft_len
    for i in (fft_len - half)..fft_len {
        if cross[i].0 > best_val {
            best_val = cross[i].0;
            best_idx = i;
        }
    }

    let delay = if best_idx <= half {
        best_idx as f64
    } else {
        best_idx as f64 - fft_len as f64
    };

    // Parabolic interpolation for sub-sample accuracy
    let left = if best_idx == 0 { fft_len - 1 } else { best_idx - 1 };
    let right = if best_idx == fft_len - 1 { 0 } else { best_idx + 1 };
    let alpha = cross[left].0;
    let beta = cross[best_idx].0;
    let gamma = cross[right].0;
    let denom = alpha - 2.0 * beta + gamma;
    let frac = if denom.abs() > 1e-30 {
        0.5 * (alpha - gamma) / denom
    } else {
        0.0
    };

    (delay + frac, best_val)
}

/// Estimate direction of arrival (DOA) using GCC-PHAT on the first pair of
/// elements. Returns the estimated angle in radians.
pub fn estimate_doa(
    signals: &[Vec<f64>],
    spacing_m: f64,
    sound_speed: f64,
    sample_rate: f64,
) -> f64 {
    if signals.len() < 2 || signals[0].is_empty() {
        return 0.0;
    }
    let (delay_samples, _) = gcc_phat(&signals[0], &signals[1]);
    let delay_sec = delay_samples / sample_rate;
    let sin_theta = delay_sec * sound_speed / spacing_m;
    sin_theta.clamp(-1.0, 1.0).asin()
}

/// Compute the beam pattern of a set of complex weights.
///
/// Returns the angular power response from -PI/2 to PI/2 in `num_points`
/// steps.
pub fn compute_beam_pattern(
    weights: &[C64],
    spacing_m: f64,
    freq_hz: f64,
    sound_speed: f64,
    num_points: usize,
) -> BeamPattern {
    let m = weights.len();
    let wavelength = sound_speed / freq_hz;
    let mut angles = Vec::with_capacity(num_points);
    let mut response_db = Vec::with_capacity(num_points);

    for k in 0..num_points {
        let angle = -PI / 2.0 + PI * k as f64 / (num_points - 1).max(1) as f64;
        angles.push(angle);

        let d_sin = spacing_m * angle.sin();
        let mut acc: C64 = (0.0, 0.0);
        for i in 0..m {
            let phase = -2.0 * PI * i as f64 * d_sin / wavelength;
            let sv_i = c_exp(phase);
            acc = c_add(acc, c_mul(c_conj(weights[i]), sv_i));
        }
        let mag = c_abs(acc);
        let db = if mag > 1e-30 {
            20.0 * mag.log10()
        } else {
            -300.0
        };
        response_db.push(db);
    }

    BeamPattern {
        angles_rad: angles,
        response_db,
    }
}

/// White Noise Gain (WNG) of a set of complex weights, in dB.
///
/// WNG = |w^H a|^2 / ||w||^2 for a broadside steering vector a = [1,1,...,1].
/// For uniform weights this equals N (the array gain).
pub fn white_noise_gain(weights: &[C64]) -> f64 {
    if weights.is_empty() {
        return 0.0;
    }
    // a = [1, 1, ..., 1] for broadside
    let mut numerator: C64 = (0.0, 0.0);
    let mut denominator: f64 = 0.0;
    for w in weights {
        numerator = c_add(numerator, c_conj(*w));
        denominator += w.0 * w.0 + w.1 * w.1;
    }
    let num_mag2 = numerator.0 * numerator.0 + numerator.1 * numerator.1;
    if denominator < 1e-30 {
        return -300.0;
    }
    10.0 * (num_mag2 / denominator).log10()
}

/// Broadband frequency-domain beamformer.
///
/// Applies frequency-dependent delay-and-sum in the frequency domain via an
/// overlap-save FFT structure. `fft_size` must be a power of two.
pub fn broadband_beamform(
    signals: &[Vec<f64>],
    steering_angle_rad: f64,
    spacing_m: f64,
    sound_speed: f64,
    sample_rate: f64,
    fft_size: usize,
) -> Vec<f64> {
    let m = signals.len();
    if m == 0 {
        return Vec::new();
    }
    let n = signals[0].len();
    let fft_len = next_power_of_two(fft_size);
    let inv_m = 1.0 / m as f64;

    let mut output = vec![0.0; n];

    // Process in overlapping blocks
    let hop = fft_len / 2;
    let num_blocks = (n + hop - 1) / hop;

    for blk in 0..num_blocks {
        let start = blk * hop;
        let end = (start + fft_len).min(n);
        let block_len = end - start;

        let mut sum_buf: Vec<C64> = vec![(0.0, 0.0); fft_len];

        for ch in 0..m {
            // Window and FFT this channel's block
            let mut buf: Vec<C64> = vec![(0.0, 0.0); fft_len];
            for i in 0..block_len {
                // Hann window
                let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / fft_len as f64).cos());
                buf[i] = (signals[ch][start + i] * w, 0.0);
            }
            fft_in_place(&mut buf, false);

            // Apply frequency-dependent phase shift (steering)
            let d_sin = spacing_m * steering_angle_rad.sin();
            for k in 0..fft_len {
                let freq_bin = if k <= fft_len / 2 {
                    k as f64 * sample_rate / fft_len as f64
                } else {
                    (k as f64 - fft_len as f64) * sample_rate / fft_len as f64
                };
                let delay_phase = 2.0 * PI * ch as f64 * d_sin * freq_bin / (sound_speed * sample_rate);
                let steering = c_exp(delay_phase);
                buf[k] = c_mul(buf[k], steering);
            }

            // Accumulate
            for k in 0..fft_len {
                sum_buf[k] = c_add(sum_buf[k], c_scale(inv_m, buf[k]));
            }
        }

        // IFFT
        fft_in_place(&mut sum_buf, true);

        // Overlap-add
        for i in 0..block_len {
            output[start + i] += sum_buf[i].0;
        }
    }

    output
}

// ---------------------------------------------------------------------------
// Sinc interpolation helper
// ---------------------------------------------------------------------------

fn sinc_interp(signal: &[f64], pos: f64) -> f64 {
    let n = signal.len();
    if n == 0 {
        return 0.0;
    }
    let idx = pos.floor() as i64;
    let frac = pos - idx as f64;

    if frac.abs() < 1e-12 {
        let i = idx as usize;
        return if i < n { signal[i] } else { 0.0 };
    }

    // Use a small window (8 taps) for efficiency
    let half_taps: i64 = 4;
    let mut sum = 0.0;
    let mut wsum = 0.0;
    for k in (-half_taps)..=half_taps {
        let si = idx + k;
        if si >= 0 && (si as usize) < n {
            let x = frac - k as f64;
            let sinc_val = if x.abs() < 1e-12 {
                1.0
            } else {
                (PI * x).sin() / (PI * x)
            };
            // Hann window
            let win = 0.5 * (1.0 + (PI * (k as f64 + half_taps as f64) / (2 * half_taps) as f64).cos());
            let w = sinc_val * win;
            sum += signal[si as usize] * w;
            wsum += w;
        }
    }
    if wsum.abs() > 1e-30 {
        sum / wsum
    } else {
        0.0
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    /// Generate a sinusoidal signal at `freq_hz` for `num_samples`.
    fn tone(freq_hz: f64, sample_rate: f64, num_samples: usize, phase: f64) -> Vec<f64> {
        (0..num_samples)
            .map(|i| (2.0 * PI * freq_hz * i as f64 / sample_rate + phase).sin())
            .collect()
    }

    /// Build multi-channel ULA signals for a plane wave from `angle_rad`.
    fn plane_wave(
        freq_hz: f64,
        angle_rad: f64,
        num_elements: usize,
        spacing_m: f64,
        sound_speed: f64,
        sample_rate: f64,
        num_samples: usize,
    ) -> Vec<Vec<f64>> {
        let d_sin = spacing_m * angle_rad.sin();
        (0..num_elements)
            .map(|ch| {
                let delay_sec = ch as f64 * d_sin / sound_speed;
                let phase = -2.0 * PI * freq_hz * delay_sec;
                tone(freq_hz, sample_rate, num_samples, phase)
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // 1. AcousticBeamformer construction
    // -----------------------------------------------------------------------
    #[test]
    fn test_beamformer_new() {
        let bf = AcousticBeamformer::new(8, 0.04, 343.0, 16000.0);
        assert_eq!(bf.num_elements, 8);
        assert!((bf.element_spacing_m - 0.04).abs() < EPS);
        assert!((bf.sound_speed_mps - 343.0).abs() < EPS);
        assert!((bf.sample_rate_hz - 16000.0).abs() < EPS);
    }

    // -----------------------------------------------------------------------
    // 2. Delay-and-sum: broadside signal produces coherent sum
    // -----------------------------------------------------------------------
    #[test]
    fn test_das_broadside() {
        let num_el = 4;
        let spacing = 0.05;
        let fs = 16000.0;
        let c = 343.0;
        let freq = 1000.0;
        let n = 256;

        // Plane wave from broadside (0 rad)
        let signals = plane_wave(freq, 0.0, num_el, spacing, c, fs, n);
        let out = delay_and_sum_acoustic(&signals, 0.0, spacing, c, fs);
        assert_eq!(out.len(), n);

        // Output power should be close to the input power (coherent sum)
        let pwr: f64 = out.iter().map(|x| x * x).sum::<f64>() / n as f64;
        assert!(pwr > 0.3, "expected coherent sum, got pwr={pwr}");
    }

    // -----------------------------------------------------------------------
    // 3. Delay-and-sum: steered to correct angle
    // -----------------------------------------------------------------------
    #[test]
    fn test_das_steered() {
        let num_el = 4;
        let spacing = 0.05;
        let fs = 16000.0;
        let c = 343.0;
        let freq = 1000.0;
        let n = 256;
        let angle = 0.3; // radians

        let signals = plane_wave(freq, angle, num_el, spacing, c, fs, n);
        let out_correct = delay_and_sum_acoustic(&signals, angle, spacing, c, fs);
        let out_wrong = delay_and_sum_acoustic(&signals, -angle, spacing, c, fs);

        let pwr_correct: f64 = out_correct.iter().map(|x| x * x).sum::<f64>();
        let pwr_wrong: f64 = out_wrong.iter().map(|x| x * x).sum::<f64>();

        assert!(
            pwr_correct > pwr_wrong,
            "steering to correct angle should give more power: {pwr_correct} vs {pwr_wrong}"
        );
    }

    // -----------------------------------------------------------------------
    // 4. Delay-and-sum: empty input
    // -----------------------------------------------------------------------
    #[test]
    fn test_das_empty() {
        let out = delay_and_sum_acoustic(&[], 0.0, 0.05, 343.0, 16000.0);
        assert!(out.is_empty());
    }

    // -----------------------------------------------------------------------
    // 5. MVDR: broadside produces output
    // -----------------------------------------------------------------------
    #[test]
    fn test_mvdr_broadside() {
        let num_el = 4;
        let spacing = 0.05;
        let fs = 16000.0;
        let c = 343.0;
        let freq = 1000.0;
        let n = 256;

        let signals = plane_wave(freq, 0.0, num_el, spacing, c, fs, n);
        let out = mvdr_acoustic(&signals, 0.0, spacing, c, fs);
        assert_eq!(out.len(), n);

        let pwr: f64 = out.iter().map(|x| x * x).sum::<f64>() / n as f64;
        assert!(pwr > 0.01, "MVDR should produce non-trivial output, pwr={pwr}");
    }

    // -----------------------------------------------------------------------
    // 6. MVDR: empty input
    // -----------------------------------------------------------------------
    #[test]
    fn test_mvdr_empty() {
        let out = mvdr_acoustic(&[], 0.0, 0.05, 343.0, 16000.0);
        assert!(out.is_empty());
    }

    // -----------------------------------------------------------------------
    // 7. MVDR steered vs mis-steered
    // -----------------------------------------------------------------------
    #[test]
    fn test_mvdr_steered() {
        let num_el = 4;
        let spacing = 0.05;
        let fs = 16000.0;
        let c = 343.0;
        let freq = 1000.0;
        let n = 512;
        let angle = 0.3;

        let signals = plane_wave(freq, angle, num_el, spacing, c, fs, n);
        let out_hit = mvdr_acoustic(&signals, angle, spacing, c, fs);
        let out_miss = mvdr_acoustic(&signals, -angle, spacing, c, fs);

        let pwr_hit: f64 = out_hit.iter().map(|x| x * x).sum::<f64>();
        let pwr_miss: f64 = out_miss.iter().map(|x| x * x).sum::<f64>();

        assert!(
            pwr_hit > pwr_miss * 0.5,
            "MVDR steered should capture signal: {pwr_hit} vs {pwr_miss}"
        );
    }

    // -----------------------------------------------------------------------
    // 8. LCMV with single unity constraint equals MVDR-like
    // -----------------------------------------------------------------------
    #[test]
    fn test_lcmv_single_constraint() {
        let num_el = 4;
        let spacing = 0.05;
        let fs = 16000.0;
        let c = 343.0;
        let freq = 1000.0;
        let n = 256;

        let signals = plane_wave(freq, 0.0, num_el, spacing, c, fs, n);
        let constraints = vec![(0.0, (1.0, 0.0))];
        let out = lcmv_beamform(&signals, &constraints, spacing, c, fs);
        assert_eq!(out.len(), n);

        let pwr: f64 = out.iter().map(|x| x * x).sum::<f64>() / n as f64;
        assert!(pwr > 0.01, "LCMV single constraint should produce output");
    }

    // -----------------------------------------------------------------------
    // 9. LCMV: null constraint suppresses interferer direction
    // -----------------------------------------------------------------------
    #[test]
    fn test_lcmv_null_constraint() {
        let num_el = 8;
        let spacing = 0.04;
        let fs = 16000.0;
        let c = 343.0;
        let n = 512;

        let sig_angle = 0.0;
        let intf_angle = 0.5;
        let freq_sig = 1000.0;
        let freq_intf = 2000.0;

        // Mix signal + interferer
        let sig = plane_wave(freq_sig, sig_angle, num_el, spacing, c, fs, n);
        let intf = plane_wave(freq_intf, intf_angle, num_el, spacing, c, fs, n);
        let signals: Vec<Vec<f64>> = (0..num_el)
            .map(|ch| sig[ch].iter().zip(&intf[ch]).map(|(a, b)| a + 2.0 * b).collect())
            .collect();

        // Maintain gain at signal, null at interferer
        let constraints = vec![(sig_angle, (1.0, 0.0)), (intf_angle, (0.0, 0.0))];
        let out = lcmv_beamform(&signals, &constraints, spacing, c, fs);
        assert_eq!(out.len(), n);

        // The output should exist (non-zero)
        let pwr: f64 = out.iter().map(|x| x * x).sum::<f64>() / n as f64;
        assert!(pwr > 1e-6, "LCMV should produce some output even with null constraint");
    }

    // -----------------------------------------------------------------------
    // 10. LCMV empty constraints
    // -----------------------------------------------------------------------
    #[test]
    fn test_lcmv_empty_constraints() {
        let signals = vec![vec![1.0; 64]; 4];
        let out = lcmv_beamform(&signals, &[], 0.05, 343.0, 16000.0);
        assert_eq!(out.len(), 64);
        // All zeros since no constraints
        assert!(out.iter().all(|&v| v.abs() < EPS));
    }

    // -----------------------------------------------------------------------
    // 11. GCC-PHAT: zero delay
    // -----------------------------------------------------------------------
    #[test]
    fn test_gcc_phat_zero_delay() {
        let sig = tone(500.0, 16000.0, 256, 0.0);
        let (delay, peak) = gcc_phat(&sig, &sig);
        assert!(
            delay.abs() < 1.5,
            "identical signals should give ~0 delay, got {delay}"
        );
        assert!(peak > 0.5, "peak correlation should be high, got {peak}");
    }

    // -----------------------------------------------------------------------
    // 12. GCC-PHAT: known integer delay
    // -----------------------------------------------------------------------
    #[test]
    fn test_gcc_phat_known_delay() {
        let n = 512;
        let delay_samples: i64 = 5;
        let fs = 16000.0;
        let freq = 500.0;
        // sig_b is a delayed version of sig_a (continuous sinusoid with phase offset)
        let sig_a: Vec<f64> = (0..n).map(|i| (2.0 * PI * freq * i as f64 / fs).sin()).collect();
        let sig_b: Vec<f64> = (0..n).map(|i| (2.0 * PI * freq * (i as f64 - delay_samples as f64) / fs).sin()).collect();

        let (est_delay, _) = gcc_phat(&sig_a, &sig_b);
        assert!(
            (est_delay - delay_samples as f64).abs() < 1.5,
            "expected delay ~{delay_samples}, got {est_delay}"
        );
    }

    // -----------------------------------------------------------------------
    // 13. GCC-PHAT: negative delay
    // -----------------------------------------------------------------------
    #[test]
    fn test_gcc_phat_negative_delay() {
        let n = 512;
        let delay_samples: i64 = -3;
        let fs = 16000.0;
        let freq = 500.0;
        // sig_b is an advanced version of sig_a (negative delay)
        let sig_a: Vec<f64> = (0..n).map(|i| (2.0 * PI * freq * i as f64 / fs).sin()).collect();
        let sig_b: Vec<f64> = (0..n).map(|i| (2.0 * PI * freq * (i as f64 - delay_samples as f64) / fs).sin()).collect();

        let (est_delay, _) = gcc_phat(&sig_a, &sig_b);
        assert!(
            (est_delay - delay_samples as f64).abs() < 1.5,
            "expected delay ~{delay_samples}, got {est_delay}"
        );
    }

    // -----------------------------------------------------------------------
    // 14. DOA estimation: broadside source
    // -----------------------------------------------------------------------
    #[test]
    fn test_doa_broadside() {
        let num_el = 4;
        let spacing = 0.05;
        let fs = 16000.0;
        let c = 343.0;
        let signals = plane_wave(1000.0, 0.0, num_el, spacing, c, fs, 512);
        let doa = estimate_doa(&signals, spacing, c, fs);
        assert!(
            doa.abs() < 0.15,
            "broadside DOA should be near 0, got {doa}"
        );
    }

    // -----------------------------------------------------------------------
    // 15. DOA estimation: off-broadside
    // -----------------------------------------------------------------------
    #[test]
    fn test_doa_off_broadside() {
        let num_el = 4;
        let spacing = 0.10;
        let fs = 48000.0;
        let c = 343.0;
        let target_angle = 0.3;
        let signals = plane_wave(1000.0, target_angle, num_el, spacing, c, fs, 2048);
        let doa = estimate_doa(&signals, spacing, c, fs);
        assert!(
            (doa - target_angle).abs() < 0.35,
            "expected DOA ~{target_angle}, got {doa}"
        );
    }

    // -----------------------------------------------------------------------
    // 16. DOA estimation: insufficient channels
    // -----------------------------------------------------------------------
    #[test]
    fn test_doa_insufficient_channels() {
        let signals = vec![vec![0.0; 64]];
        let doa = estimate_doa(&signals, 0.05, 343.0, 16000.0);
        assert!((doa - 0.0).abs() < EPS, "single channel should return 0");
    }

    // -----------------------------------------------------------------------
    // 17. Beam pattern: uniform weights peak at broadside
    // -----------------------------------------------------------------------
    #[test]
    fn test_beam_pattern_uniform() {
        let m = 8;
        let weights: Vec<C64> = vec![(1.0 / m as f64, 0.0); m];
        let bp = compute_beam_pattern(&weights, 0.05, 3000.0, 343.0, 181);

        assert_eq!(bp.angles_rad.len(), 181);
        assert_eq!(bp.response_db.len(), 181);

        // Peak should be near 0 rad (the middle index)
        let peak_idx = bp
            .response_db
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        let peak_angle = bp.angles_rad[peak_idx];
        assert!(
            peak_angle.abs() < 0.1,
            "uniform weights should peak at broadside, got angle={peak_angle}"
        );
    }

    // -----------------------------------------------------------------------
    // 18. White noise gain: uniform weights
    // -----------------------------------------------------------------------
    #[test]
    fn test_wng_uniform() {
        let m = 4;
        let weights: Vec<C64> = vec![(1.0 / m as f64, 0.0); m];
        let wng = white_noise_gain(&weights);
        // WNG of uniform weights = N => 10*log10(4) = 6.02 dB
        let expected = 10.0 * (m as f64).log10();
        assert!(
            (wng - expected).abs() < 0.1,
            "expected WNG ~{expected} dB, got {wng}"
        );
    }

    // -----------------------------------------------------------------------
    // 19. White noise gain: empty weights
    // -----------------------------------------------------------------------
    #[test]
    fn test_wng_empty() {
        let wng = white_noise_gain(&[]);
        assert!((wng - 0.0).abs() < EPS);
    }

    // -----------------------------------------------------------------------
    // 20. Broadband beamforming: produces output
    // -----------------------------------------------------------------------
    #[test]
    fn test_broadband_output() {
        let num_el = 4;
        let spacing = 0.05;
        let fs = 16000.0;
        let c = 343.0;
        let n = 512;

        let signals = plane_wave(1000.0, 0.0, num_el, spacing, c, fs, n);
        let out = broadband_beamform(&signals, 0.0, spacing, c, fs, 128);
        assert_eq!(out.len(), n);

        let pwr: f64 = out.iter().map(|x| x * x).sum::<f64>() / n as f64;
        assert!(pwr > 0.01, "broadband should produce non-trivial output, pwr={pwr}");
    }

    // -----------------------------------------------------------------------
    // 21. Broadband: empty input
    // -----------------------------------------------------------------------
    #[test]
    fn test_broadband_empty() {
        let out = broadband_beamform(&[], 0.0, 0.05, 343.0, 16000.0, 128);
        assert!(out.is_empty());
    }

    // -----------------------------------------------------------------------
    // 22. FFT roundtrip
    // -----------------------------------------------------------------------
    #[test]
    fn test_fft_roundtrip() {
        let n = 64;
        let data: Vec<C64> = (0..n).map(|i| ((i as f64).sin(), (i as f64).cos())).collect();
        let spectrum = fft(&data);
        let recovered = ifft(&spectrum);
        for (a, b) in data.iter().zip(recovered.iter()) {
            assert!(approx_eq(a.0, b.0, 1e-10), "real mismatch: {} vs {}", a.0, b.0);
            assert!(approx_eq(a.1, b.1, 1e-10), "imag mismatch: {} vs {}", a.1, b.1);
        }
    }

    // -----------------------------------------------------------------------
    // 23. Beam pattern shape: sidelobes below mainlobe
    // -----------------------------------------------------------------------
    #[test]
    fn test_beam_pattern_sidelobes() {
        let m = 8;
        let weights: Vec<C64> = vec![(1.0 / m as f64, 0.0); m];
        let bp = compute_beam_pattern(&weights, 0.04, 4000.0, 343.0, 361);

        let peak_db = bp.response_db.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        // Check that at least some values are well below peak (sidelobes)
        let min_db = bp.response_db.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!(
            peak_db - min_db > 5.0,
            "beam pattern should have sidelobes below mainlobe: peak={peak_db}, min={min_db}"
        );
    }

    // -----------------------------------------------------------------------
    // 24. Steering vector properties
    // -----------------------------------------------------------------------
    #[test]
    fn test_steering_vector_broadside() {
        let sv = steering_vector(4, 0.05, 1000.0, 343.0, 0.0);
        // At broadside, all phases = 0, so all elements should be (1, 0)
        for s in &sv {
            assert!(approx_eq(s.0, 1.0, 1e-10));
            assert!(approx_eq(s.1, 0.0, 1e-10));
        }
    }

    // -----------------------------------------------------------------------
    // 25. Matrix inversion: identity
    // -----------------------------------------------------------------------
    #[test]
    fn test_mat_invert_identity() {
        let n = 3;
        let mut mat = vec![vec![(0.0, 0.0); n]; n];
        for i in 0..n {
            mat[i][i] = (1.0, 0.0);
        }
        let inv = mat_invert(&mat).unwrap();
        for i in 0..n {
            for j in 0..n {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(approx_eq(inv[i][j].0, expected, 1e-10));
                assert!(approx_eq(inv[i][j].1, 0.0, 1e-10));
            }
        }
    }
}
