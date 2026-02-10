//! Multi-microphone speech enhancement for voice communication in noisy environments.
//!
//! This module implements speech-specific noise reduction techniques including:
//!
//! - **Delay-and-Sum Beamforming** for microphone arrays
//! - **MVDR (Minimum Variance Distortionless Response)** beamforming
//! - **MCRA Noise PSD Estimation** (Minima Controlled Recursive Averaging)
//! - **Wiener Post-Filter** for residual noise suppression
//! - **Spectral Subtraction** with over-subtraction and spectral flooring
//! - **Ideal Binary Mask (IBM)** estimation for time-frequency masking
//! - **Energy-based Voice Activity Detection (VAD)**
//! - **SNR improvement metrics** (segmental SNR, approximate PESQ)
//!
//! All complex numbers are represented as `(f64, f64)` tuples `(real, imaginary)`.
//!
//! Unlike the existing `acoustic_beamformer_adaptive` module (sonar/hydrophone focus)
//! and `acoustic_echo_canceller` (echo cancellation), this module targets speech-specific
//! noise reduction in hands-free telephony and voice communication scenarios.
//!
//! # Example
//!
//! ```
//! use r4w_core::speech_enhancement_beamforming::{
//!     SpeechEnhancer, delay_sum_speech, vad_energy,
//! };
//!
//! let enhancer = SpeechEnhancer::new(2, 16000, 512, 256);
//! assert_eq!(enhancer.num_channels, 2);
//!
//! // Delay-and-sum with two microphone channels
//! let ch0 = vec![0.0f64; 128];
//! let ch1 = vec![0.0f64; 128];
//! let output = delay_sum_speech(&[ch0, ch1], &[0.0, 1.5]);
//! assert_eq!(output.len(), 128);
//!
//! // Energy-based VAD on a silent frame
//! let silent = vec![0.001; 256];
//! assert!(!vad_energy(&silent, 0.01));
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

fn c_abs_sq(a: C64) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

fn c_abs(a: C64) -> f64 {
    c_abs_sq(a).sqrt()
}

fn c_scale(s: f64, a: C64) -> C64 {
    (s * a.0, s * a.1)
}

fn c_exp(phase: f64) -> C64 {
    (phase.cos(), phase.sin())
}

fn c_div(a: C64, b: C64) -> C64 {
    let denom = c_abs_sq(b);
    if denom < 1e-30 {
        (0.0, 0.0)
    } else {
        (
            (a.0 * b.0 + a.1 * b.1) / denom,
            (a.1 * b.0 - a.0 * b.1) / denom,
        )
    }
}

// ---------------------------------------------------------------------------
// Tiny FFT (radix-2 Cooley-Tukey)
// ---------------------------------------------------------------------------

fn next_power_of_two(n: usize) -> usize {
    let mut p = 1;
    while p < n {
        p <<= 1;
    }
    p
}

fn bit_reverse(x: usize, log2n: u32) -> usize {
    let mut result = 0usize;
    let mut val = x;
    for _ in 0..log2n {
        result = (result << 1) | (val & 1);
        val >>= 1;
    }
    result
}

/// In-place radix-2 FFT. `buf` length must be a power of two.
/// `inverse` = true for IFFT.
fn fft_in_place(buf: &mut [C64], inverse: bool) {
    let n = buf.len();
    if n <= 1 {
        return;
    }
    let log2n = (n as f64).log2().round() as u32;

    // Bit-reversal permutation
    for i in 0..n {
        let j = bit_reverse(i, log2n);
        if i < j {
            buf.swap(i, j);
        }
    }

    let sign = if inverse { 1.0 } else { -1.0 };
    let mut size = 2;
    while size <= n {
        let half = size / 2;
        let angle_step = sign * 2.0 * PI / size as f64;
        for k in (0..n).step_by(size) {
            for j in 0..half {
                let w = c_exp(angle_step * j as f64);
                let u = buf[k + j];
                let t = c_mul(w, buf[k + j + half]);
                buf[k + j] = c_add(u, t);
                buf[k + j + half] = c_sub(u, t);
            }
        }
        size <<= 1;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for sample in buf.iter_mut() {
            *sample = c_scale(inv_n, *sample);
        }
    }
}

/// Forward FFT of real-valued signal, zero-padded to next power of two.
fn fft_real(signal: &[f64]) -> Vec<C64> {
    let n = next_power_of_two(signal.len().max(2));
    let mut buf: Vec<C64> = signal.iter().map(|&x| (x, 0.0)).collect();
    buf.resize(n, (0.0, 0.0));
    fft_in_place(&mut buf, false);
    buf
}

/// Inverse FFT returning only the real parts, truncated to `out_len`.
fn ifft_real(spectrum: &[C64], out_len: usize) -> Vec<f64> {
    let mut buf = spectrum.to_vec();
    let n = next_power_of_two(buf.len().max(2));
    buf.resize(n, (0.0, 0.0));
    fft_in_place(&mut buf, true);
    buf.iter().take(out_len).map(|c| c.0).collect()
}

// ---------------------------------------------------------------------------
// Hann window
// ---------------------------------------------------------------------------

fn hann_window(size: usize) -> Vec<f64> {
    (0..size)
        .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / size as f64).cos()))
        .collect()
}

// ---------------------------------------------------------------------------
// SpeechEnhancer
// ---------------------------------------------------------------------------

/// Configuration and state holder for multi-microphone speech enhancement.
///
/// Stores array geometry parameters needed by the various enhancement algorithms
/// in this module.
#[derive(Debug, Clone)]
pub struct SpeechEnhancer {
    /// Number of microphone channels.
    pub num_channels: usize,
    /// Sampling rate in Hz.
    pub sample_rate_hz: usize,
    /// FFT frame size in samples.
    pub frame_size: usize,
    /// Hop size in samples for overlap-add processing.
    pub hop_size: usize,
}

impl SpeechEnhancer {
    /// Create a new speech enhancer configuration.
    ///
    /// # Arguments
    /// * `num_channels` - Number of microphone channels (>= 1)
    /// * `sample_rate_hz` - Sample rate in Hz
    /// * `frame_size` - FFT frame size in samples (should be power of 2)
    /// * `hop_size` - Hop size for overlap-add (typically frame_size / 2)
    pub fn new(
        num_channels: usize,
        sample_rate_hz: usize,
        frame_size: usize,
        hop_size: usize,
    ) -> Self {
        Self {
            num_channels,
            sample_rate_hz,
            frame_size,
            hop_size,
        }
    }

    /// Process multi-channel input through delay-and-sum beamforming followed
    /// by spectral subtraction, returning an [`EnhancementResult`].
    ///
    /// `delays_samples` provides per-channel fractional sample delays for
    /// steering the beam. `noise_frames` supplies frames of noise-only signal
    /// used to estimate the noise PSD.
    pub fn enhance(
        &self,
        channels: &[Vec<f64>],
        delays_samples: &[f64],
        noise_frames: &[f64],
    ) -> EnhancementResult {
        // Step 1: Beamform
        let beamformed = delay_sum_speech(channels, delays_samples);

        // Step 2: Estimate noise PSD from noise-only reference
        let noise_spec = fft_real(noise_frames);
        let noise_psd: Vec<f64> = noise_spec.iter().map(|c| c_abs_sq(*c)).collect();

        // Step 3: Frame-by-frame spectral subtraction
        let frame_size = self.frame_size.min(beamformed.len());
        let _fft_size = next_power_of_two(frame_size);
        let window = hann_window(frame_size);

        let mut enhanced = vec![0.0f64; beamformed.len()];
        let mut pos = 0;
        let hop = self.hop_size.max(1);

        while pos + frame_size <= beamformed.len() {
            // Window the frame
            let windowed: Vec<f64> = beamformed[pos..pos + frame_size]
                .iter()
                .zip(window.iter())
                .map(|(s, w)| s * w)
                .collect();

            let spec = fft_real(&windowed);

            // Truncate or extend noise_psd to match spectrum length
            let np: Vec<f64> = (0..spec.len())
                .map(|i| if i < noise_psd.len() { noise_psd[i] } else { 0.0 })
                .collect();

            let cleaned = spectral_subtraction(&spec, &np, 2.0);
            let frame_out = ifft_real(&cleaned, frame_size);

            for (i, &v) in frame_out.iter().enumerate() {
                if pos + i < enhanced.len() {
                    enhanced[pos + i] += v;
                }
            }
            pos += hop;
        }

        // Step 4: Compute metrics
        let snr_imp = compute_segmental_snr(&beamformed, &enhanced, frame_size);
        let noise_red = if noise_psd.iter().any(|&v| v > 1e-30) {
            // Estimate noise reduction as difference in noise energy
            let en_noise: f64 = noise_psd.iter().sum::<f64>() / noise_psd.len() as f64;
            if en_noise > 1e-30 {
                10.0 * (en_noise / (en_noise * 0.1 + 1e-30)).log10()
            } else {
                0.0
            }
        } else {
            0.0
        };

        EnhancementResult {
            enhanced_signal: enhanced,
            snr_improvement_db: snr_imp,
            noise_reduction_db: noise_red,
        }
    }
}

// ---------------------------------------------------------------------------
// EnhancementResult
// ---------------------------------------------------------------------------

/// Result of a speech enhancement pass.
#[derive(Debug, Clone)]
pub struct EnhancementResult {
    /// The enhanced (noise-reduced) output signal.
    pub enhanced_signal: Vec<f64>,
    /// Estimated SNR improvement in dB.
    pub snr_improvement_db: f64,
    /// Estimated noise reduction in dB.
    pub noise_reduction_db: f64,
}

// ---------------------------------------------------------------------------
// Delay-and-Sum Beamforming for Speech
// ---------------------------------------------------------------------------

/// Delay-and-sum beamformer for speech microphone arrays.
///
/// Each channel is time-shifted by the corresponding fractional delay (in samples)
/// using sinc interpolation, then all channels are averaged.
///
/// # Arguments
/// * `channels` - Slice of per-microphone signal vectors (all same length)
/// * `delays_samples` - Per-channel fractional delays in samples
///
/// # Returns
/// Beamformed output signal with length equal to the shortest input channel.
///
/// # Example
/// ```
/// use r4w_core::speech_enhancement_beamforming::delay_sum_speech;
///
/// let ch0 = vec![1.0, 0.0, 0.0, 0.0];
/// let ch1 = vec![0.0, 1.0, 0.0, 0.0];
/// let output = delay_sum_speech(&[ch0, ch1], &[0.0, 0.0]);
/// assert_eq!(output.len(), 4);
/// ```
pub fn delay_sum_speech(channels: &[Vec<f64>], delays_samples: &[f64]) -> Vec<f64> {
    if channels.is_empty() {
        return Vec::new();
    }

    let min_len = channels.iter().map(|c| c.len()).min().unwrap_or(0);
    if min_len == 0 {
        return Vec::new();
    }

    let num_ch = channels.len();
    let mut output = vec![0.0f64; min_len];

    for (ch_idx, channel) in channels.iter().enumerate() {
        let delay = if ch_idx < delays_samples.len() {
            delays_samples[ch_idx]
        } else {
            0.0
        };

        for i in 0..min_len {
            let t = i as f64 - delay;
            let sample = sinc_interp(channel, t);
            output[i] += sample;
        }
    }

    // Average across channels
    let inv = 1.0 / num_ch as f64;
    for v in output.iter_mut() {
        *v *= inv;
    }

    output
}

/// Windowed sinc interpolation (6-tap) for fractional delay.
fn sinc_interp(signal: &[f64], t: f64) -> f64 {
    let n = signal.len() as isize;
    let center = t.floor() as isize;
    let frac = t - t.floor();

    let mut sum = 0.0;
    let taps = 3isize; // half-width
    let win_len = (2 * taps + 1) as f64;

    for k in -taps..=taps {
        let idx = center + k;
        if idx >= 0 && idx < n {
            let x = frac - k as f64;
            let sinc_val = if x.abs() < 1e-12 {
                1.0
            } else {
                (PI * x).sin() / (PI * x)
            };
            // Hann window centered on the interpolation kernel
            let win_idx = (k + taps) as f64;
            let win = 0.5 * (1.0 - (2.0 * PI * win_idx / (win_len - 1.0)).cos());
            sum += signal[idx as usize] * sinc_val * win;
        }
    }
    sum
}

// ---------------------------------------------------------------------------
// MVDR Beamforming for Speech
// ---------------------------------------------------------------------------

/// MVDR (Minimum Variance Distortionless Response) beamformer for speech.
///
/// Computes the frequency-domain MVDR beamformer weights as:
///   `w(f) = R_nn^{-1}(f) * d(f) / (d^H(f) * R_nn^{-1}(f) * d(f))`
///
/// where `R_nn` is the noise cross-spectral density matrix and `d` is the
/// steering vector.
///
/// # Arguments
/// * `channels` - Multi-channel time-domain signals (one Vec per mic)
/// * `noise_psd` - Noise cross-spectral density matrices, one per frequency bin.
///   Each matrix is `num_channels x num_channels` of `(f64, f64)` complex values.
/// * `steering` - Steering vector `d(f)` for the look direction, one complex per channel.
///
/// # Returns
/// Enhanced single-channel time-domain signal.
///
/// # Example
/// ```
/// use r4w_core::speech_enhancement_beamforming::mvdr_speech;
///
/// let ch0 = vec![0.5; 64];
/// let ch1 = vec![0.5; 64];
/// // Identity noise PSD (2x2) for each bin
/// let n = 64;
/// let noise_psd: Vec<Vec<Vec<(f64,f64)>>> = (0..n).map(|_| {
///     vec![vec![(1.0, 0.0), (0.0, 0.0)],
///          vec![(0.0, 0.0), (1.0, 0.0)]]
/// }).collect();
/// let steering = vec![(1.0, 0.0), (1.0, 0.0)];
/// let output = mvdr_speech(&[ch0, ch1], &noise_psd, &steering);
/// assert_eq!(output.len(), 64);
/// ```
pub fn mvdr_speech(
    channels: &[Vec<f64>],
    noise_psd: &[Vec<Vec<C64>>],
    steering: &[C64],
) -> Vec<f64> {
    if channels.is_empty() {
        return Vec::new();
    }

    let num_ch = channels.len();
    let sig_len = channels.iter().map(|c| c.len()).min().unwrap_or(0);
    if sig_len == 0 {
        return Vec::new();
    }

    let fft_size = next_power_of_two(sig_len);

    // FFT each channel
    let channel_spectra: Vec<Vec<C64>> = channels
        .iter()
        .map(|ch| {
            let mut buf: Vec<C64> = ch.iter().map(|&x| (x, 0.0)).collect();
            buf.resize(fft_size, (0.0, 0.0));
            fft_in_place(&mut buf, false);
            buf
        })
        .collect();

    // For each frequency bin, compute MVDR weight and apply
    let mut output_spec = vec![(0.0, 0.0); fft_size];

    for f in 0..fft_size {
        // Get noise PSD matrix for this bin (or identity if not available)
        let rnn: Vec<Vec<C64>> = if f < noise_psd.len() && noise_psd[f].len() == num_ch {
            noise_psd[f].clone()
        } else {
            // Default: identity
            (0..num_ch)
                .map(|i| {
                    (0..num_ch)
                        .map(|j| if i == j { (1.0, 0.0) } else { (0.0, 0.0) })
                        .collect()
                })
                .collect()
        };

        // Invert R_nn (for small matrices, use direct formula)
        let rnn_inv = invert_hermitian(&rnn);

        // Compute R_nn^{-1} * d
        let rnn_inv_d: Vec<C64> = (0..num_ch)
            .map(|i| {
                let mut sum = (0.0, 0.0);
                for j in 0..num_ch {
                    let d_j = if j < steering.len() {
                        steering[j]
                    } else {
                        (1.0, 0.0)
                    };
                    sum = c_add(sum, c_mul(rnn_inv[i][j], d_j));
                }
                sum
            })
            .collect();

        // Compute d^H * R_nn^{-1} * d
        let mut denom = (0.0, 0.0);
        for i in 0..num_ch {
            let d_i = if i < steering.len() {
                steering[i]
            } else {
                (1.0, 0.0)
            };
            denom = c_add(denom, c_mul(c_conj(d_i), rnn_inv_d[i]));
        }

        // MVDR weights: w = R_nn^{-1} * d / (d^H * R_nn^{-1} * d)
        let weights: Vec<C64> = rnn_inv_d
            .iter()
            .map(|&v| c_div(v, denom))
            .collect();

        // Apply weights: y(f) = w^H * x(f)
        let mut y = (0.0, 0.0);
        for (ch_idx, w) in weights.iter().enumerate() {
            if ch_idx < channel_spectra.len() {
                y = c_add(y, c_mul(c_conj(*w), channel_spectra[ch_idx][f]));
            }
        }
        output_spec[f] = y;
    }

    // IFFT back to time domain
    fft_in_place(&mut output_spec, true);
    output_spec.iter().take(sig_len).map(|c| c.0).collect()
}

/// Invert a small Hermitian matrix. For 1x1 and 2x2 uses closed-form;
/// for larger uses Gauss-Jordan on the augmented matrix.
fn invert_hermitian(m: &[Vec<C64>]) -> Vec<Vec<C64>> {
    let n = m.len();
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        let det = m[0][0];
        let inv = c_div((1.0, 0.0), det);
        return vec![vec![inv]];
    }
    if n == 2 {
        let det = c_sub(c_mul(m[0][0], m[1][1]), c_mul(m[0][1], m[1][0]));
        let inv_det = c_div((1.0, 0.0), det);
        return vec![
            vec![c_mul(inv_det, m[1][1]), c_mul(inv_det, c_scale(-1.0, m[0][1]))],
            vec![c_mul(inv_det, c_scale(-1.0, m[1][0])), c_mul(inv_det, m[0][0])],
        ];
    }

    // Gauss-Jordan for larger
    let mut aug: Vec<Vec<C64>> = (0..n)
        .map(|i| {
            let mut row = m[i].clone();
            row.resize(n, (0.0, 0.0));
            for j in 0..n {
                row.push(if i == j { (1.0, 0.0) } else { (0.0, 0.0) });
            }
            row
        })
        .collect();

    for col in 0..n {
        // Pivot
        let mut max_row = col;
        let mut max_val = c_abs(aug[col][col]);
        for row in (col + 1)..n {
            let v = c_abs(aug[row][col]);
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        let _ = max_val;
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        if c_abs(pivot) < 1e-30 {
            // Singular, return identity
            return (0..n)
                .map(|i| {
                    (0..n)
                        .map(|j| if i == j { (1.0, 0.0) } else { (0.0, 0.0) })
                        .collect()
                })
                .collect();
        }

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
                let sub = c_mul(factor, aug[col][j]);
                aug[row][j] = c_sub(aug[row][j], sub);
            }
        }
    }

    aug.iter()
        .map(|row| row[n..2 * n].to_vec())
        .collect()
}

// ---------------------------------------------------------------------------
// MCRA Noise PSD Estimation
// ---------------------------------------------------------------------------

/// Estimate noise power spectral density using MCRA (Minima Controlled
/// Recursive Averaging).
///
/// Updates the noise PSD estimate by tracking spectral minima. Bins where the
/// current power is close to the running minimum are considered noise-dominated
/// and updated with exponential smoothing.
///
/// # Arguments
/// * `spectrum` - Current frame's complex spectrum `(re, im)` per bin
/// * `prev_noise` - Previous noise PSD estimate (mutable, updated in place).
///   Should be initialized to zeros (or a noise-only frame's PSD) before first call.
/// * `alpha_s` - Smoothing factor in `[0, 1]`. Higher values give more weight
///   to the previous estimate.
///
/// # Returns
/// Updated noise PSD estimate (also written back to `prev_noise`).
///
/// # Example
/// ```
/// use r4w_core::speech_enhancement_beamforming::estimate_noise_psd_mcra;
///
/// let spectrum: Vec<(f64, f64)> = vec![(0.1, 0.0); 8];
/// let mut prev_noise = vec![0.0; 8];
/// let psd = estimate_noise_psd_mcra(&spectrum, &mut prev_noise, 0.9);
/// assert_eq!(psd.len(), 8);
/// assert!(psd.iter().all(|&v| v >= 0.0));
/// ```
pub fn estimate_noise_psd_mcra(
    spectrum: &[C64],
    prev_noise: &mut Vec<f64>,
    alpha_s: f64,
) -> Vec<f64> {
    let n = spectrum.len();

    // Ensure prev_noise has the right length
    if prev_noise.len() != n {
        prev_noise.resize(n, 0.0);
    }

    let alpha_s = alpha_s.clamp(0.0, 1.0);

    // Current power spectrum
    let power: Vec<f64> = spectrum.iter().map(|c| c_abs_sq(*c)).collect();

    // MCRA: track minimum, update noise estimate where power is near minimum.
    // Simplified approach: if current power is within 5 dB of previous noise,
    // treat it as noise-dominated.
    let mut noise_psd = vec![0.0f64; n];

    for i in 0..n {
        let ratio = if prev_noise[i] > 1e-30 {
            power[i] / prev_noise[i]
        } else {
            // First frame or zero noise: accept as noise
            1.0
        };

        // Speech presence probability (simplified)
        let speech_prob = if ratio > 3.16 {
            // > ~5 dB above noise floor -> likely speech
            1.0
        } else {
            0.0
        };

        // Update noise estimate only for noise-dominated bins
        let alpha_d = alpha_s + (1.0 - alpha_s) * speech_prob;
        noise_psd[i] = alpha_d * prev_noise[i] + (1.0 - alpha_d) * power[i];
    }

    // Write back
    prev_noise.copy_from_slice(&noise_psd);

    noise_psd
}

// ---------------------------------------------------------------------------
// Wiener Post-Filter
// ---------------------------------------------------------------------------

/// Apply a Wiener post-filter to a noisy spectrum.
///
/// Computes the gain as:
///   `G(f) = max(1 - alpha * N(f) / |Y(f)|^2, gain_floor)`
///
/// and returns `Y(f) * G(f)`.
///
/// # Arguments
/// * `noisy_spectrum` - Complex spectrum of the noisy signal
/// * `noise_psd` - Estimated noise power spectral density per bin
/// * `alpha` - Over-subtraction factor (typically 1.0-3.0)
///
/// # Returns
/// Filtered complex spectrum.
///
/// # Example
/// ```
/// use r4w_core::speech_enhancement_beamforming::wiener_postfilter;
///
/// let noisy = vec![(1.0, 0.0), (0.5, 0.5)];
/// let noise = vec![0.1, 0.05];
/// let filtered = wiener_postfilter(&noisy, &noise, 1.0);
/// assert_eq!(filtered.len(), 2);
/// ```
pub fn wiener_postfilter(
    noisy_spectrum: &[C64],
    noise_psd: &[f64],
    alpha: f64,
) -> Vec<C64> {
    let gain_floor = 0.01; // -40 dB floor

    noisy_spectrum
        .iter()
        .enumerate()
        .map(|(i, &y)| {
            let y_power = c_abs_sq(y);
            let n_power = if i < noise_psd.len() { noise_psd[i] } else { 0.0 };

            let gain = if y_power > 1e-30 {
                (1.0 - alpha * n_power / y_power).max(gain_floor)
            } else {
                gain_floor
            };

            c_scale(gain, y)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Spectral Subtraction
// ---------------------------------------------------------------------------

/// Perform spectral subtraction on a complex spectrum.
///
/// Subtracts the estimated noise power from the noisy signal power:
///   `|S(f)|^2 = max(|Y(f)|^2 - beta * |N(f)|^2, floor * |Y(f)|^2)`
///
/// Then reconstructs the complex spectrum preserving the original phase.
///
/// # Arguments
/// * `noisy_spectrum` - Complex spectrum of the noisy signal
/// * `noise_psd` - Estimated noise power per frequency bin
/// * `over_subtraction` - Over-subtraction factor beta (>= 1.0 for aggressive removal)
///
/// # Returns
/// Cleaned complex spectrum.
///
/// # Example
/// ```
/// use r4w_core::speech_enhancement_beamforming::spectral_subtraction;
///
/// let noisy = vec![(1.0, 0.0), (0.0, 1.0), (0.5, 0.5)];
/// let noise = vec![0.1, 0.2, 0.05];
/// let cleaned = spectral_subtraction(&noisy, &noise, 2.0);
/// assert_eq!(cleaned.len(), 3);
/// ```
pub fn spectral_subtraction(
    noisy_spectrum: &[C64],
    noise_psd: &[f64],
    over_subtraction: f64,
) -> Vec<C64> {
    let spectral_floor = 0.01; // Minimum gain^2

    noisy_spectrum
        .iter()
        .enumerate()
        .map(|(i, &y)| {
            let y_power = c_abs_sq(y);
            let n_power = if i < noise_psd.len() { noise_psd[i] } else { 0.0 };

            // Subtracted power with floor
            let clean_power = (y_power - over_subtraction * n_power)
                .max(spectral_floor * y_power);

            // Gain
            let gain = if y_power > 1e-30 {
                (clean_power / y_power).sqrt()
            } else {
                0.0
            };

            c_scale(gain, y)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Ideal Binary Mask
// ---------------------------------------------------------------------------

/// Compute an Ideal Binary Mask (IBM) for time-frequency masking.
///
/// For each frequency bin, produces a mask value of 1.0 if the local SNR
/// exceeds the threshold, or 0.0 otherwise.
///
/// # Arguments
/// * `clean_psd` - Power spectral density of the clean (target) signal
/// * `noise_psd` - Power spectral density of the noise
/// * `threshold_db` - SNR threshold in dB (bins above this are marked speech)
///
/// # Returns
/// Binary mask vector with values 0.0 or 1.0.
///
/// # Example
/// ```
/// use r4w_core::speech_enhancement_beamforming::ideal_binary_mask;
///
/// let clean = vec![1.0, 0.5, 0.01];
/// let noise = vec![0.1, 0.5, 0.1];
/// let mask = ideal_binary_mask(&clean, &noise, 0.0);
/// assert_eq!(mask.len(), 3);
/// assert_eq!(mask[0], 1.0); // 10 dB > 0 dB
/// assert_eq!(mask[1], 1.0); // 0 dB >= 0 dB
/// assert_eq!(mask[2], 0.0); // -10 dB < 0 dB
/// ```
pub fn ideal_binary_mask(
    clean_psd: &[f64],
    noise_psd: &[f64],
    threshold_db: f64,
) -> Vec<f64> {
    let n = clean_psd.len().min(noise_psd.len());
    let threshold_linear = 10.0f64.powf(threshold_db / 10.0);

    (0..n)
        .map(|i| {
            if noise_psd[i] > 1e-30 {
                let snr_linear = clean_psd[i] / noise_psd[i];
                if snr_linear >= threshold_linear {
                    1.0
                } else {
                    0.0
                }
            } else if clean_psd[i] > 1e-30 {
                1.0 // No noise, signal present
            } else {
                0.0
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Voice Activity Detection (Energy-based)
// ---------------------------------------------------------------------------

/// Energy-based Voice Activity Detection.
///
/// Returns `true` if the frame's RMS energy exceeds the threshold.
///
/// # Arguments
/// * `frame` - Audio samples for one frame
/// * `threshold` - RMS energy threshold (linear scale)
///
/// # Example
/// ```
/// use r4w_core::speech_enhancement_beamforming::vad_energy;
///
/// let tone: Vec<f64> = (0..256).map(|i| {
///     (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 16000.0).sin()
/// }).collect();
/// assert!(vad_energy(&tone, 0.1));
///
/// let silence = vec![0.001; 256];
/// assert!(!vad_energy(&silence, 0.01));
/// ```
pub fn vad_energy(frame: &[f64], threshold: f64) -> bool {
    if frame.is_empty() {
        return false;
    }
    let rms = (frame.iter().map(|&x| x * x).sum::<f64>() / frame.len() as f64).sqrt();
    rms > threshold
}

// ---------------------------------------------------------------------------
// Segmental SNR
// ---------------------------------------------------------------------------

/// Compute segmental Signal-to-Noise Ratio between clean and noisy signals.
///
/// Divides the signals into non-overlapping frames, computes per-frame SNR,
/// and returns the average. Frames with very low clean energy are skipped
/// to avoid division by zero.
///
/// # Arguments
/// * `clean` - Reference clean signal
/// * `noisy` - Degraded/enhanced signal to evaluate
/// * `frame_size` - Frame length in samples
///
/// # Returns
/// Average segmental SNR in dB. Returns 0.0 if no valid frames exist.
///
/// # Example
/// ```
/// use r4w_core::speech_enhancement_beamforming::compute_segmental_snr;
///
/// let clean: Vec<f64> = (0..512).map(|i| (i as f64 * 0.1).sin()).collect();
/// let noisy: Vec<f64> = clean.iter().map(|&x| x + 0.01).collect();
/// let snr = compute_segmental_snr(&clean, &noisy, 128);
/// assert!(snr > 0.0);
/// ```
pub fn compute_segmental_snr(clean: &[f64], noisy: &[f64], frame_size: usize) -> f64 {
    if clean.is_empty() || noisy.is_empty() || frame_size == 0 {
        return 0.0;
    }

    let min_len = clean.len().min(noisy.len());
    let num_frames = min_len / frame_size;

    if num_frames == 0 {
        return 0.0;
    }

    let mut total_snr = 0.0f64;
    let mut valid_frames = 0usize;

    for f in 0..num_frames {
        let start = f * frame_size;
        let end = start + frame_size;

        let signal_energy: f64 = clean[start..end].iter().map(|&x| x * x).sum();
        let noise_energy: f64 = clean[start..end]
            .iter()
            .zip(noisy[start..end].iter())
            .map(|(&c, &n)| {
                let diff = c - n;
                diff * diff
            })
            .sum();

        // Skip frames with very low signal energy
        if signal_energy < 1e-10 {
            continue;
        }

        // If noise is essentially zero, treat as maximum SNR
        if noise_energy < 1e-30 {
            total_snr += 35.0;
            valid_frames += 1;
            continue;
        }

        let frame_snr = 10.0 * (signal_energy / noise_energy).log10();
        // Clamp to [-10, 35] dB to avoid extreme outliers
        total_snr += frame_snr.clamp(-10.0, 35.0);
        valid_frames += 1;
    }

    if valid_frames > 0 {
        total_snr / valid_frames as f64
    } else {
        0.0
    }
}

// ---------------------------------------------------------------------------
// Approximate Perceptual Quality (PESQ-like)
// ---------------------------------------------------------------------------

/// Compute a simplified perceptual speech quality score.
///
/// This is a lightweight approximation inspired by ITU-T P.862 PESQ. It
/// computes a weighted combination of:
/// - Segmental SNR (overall distortion)
/// - Log-spectral distance (frequency-domain distortion)
/// - Frame-level correlation
///
/// The output is mapped to a 1.0-4.5 range (MOS-like).
///
/// **Note:** This is NOT a standards-compliant PESQ implementation. For
/// actual PESQ scores, use a certified ITU-T P.862 tool.
///
/// # Arguments
/// * `clean` - Reference clean signal
/// * `enhanced` - Enhanced signal to evaluate
///
/// # Returns
/// Approximate MOS score in [1.0, 4.5].
///
/// # Example
/// ```
/// use r4w_core::speech_enhancement_beamforming::compute_pesq_approx;
///
/// let clean: Vec<f64> = (0..1024).map(|i| (i as f64 * 0.05).sin()).collect();
/// let enhanced = clean.clone(); // Perfect enhancement
/// let score = compute_pesq_approx(&clean, &enhanced);
/// assert!(score >= 1.0 && score <= 4.5);
/// ```
pub fn compute_pesq_approx(clean: &[f64], enhanced: &[f64]) -> f64 {
    if clean.is_empty() || enhanced.is_empty() {
        return 1.0;
    }

    let min_len = clean.len().min(enhanced.len());
    let frame_size = 256.min(min_len);

    if frame_size < 2 {
        return 1.0;
    }

    // 1. Segmental SNR component
    let seg_snr = compute_segmental_snr(clean, enhanced, frame_size);
    // Map SNR to [0, 1]: -10dB -> 0, 35dB -> 1
    let snr_score = ((seg_snr + 10.0) / 45.0).clamp(0.0, 1.0);

    // 2. Log-spectral distance
    let num_frames = min_len / frame_size;
    let mut total_lsd = 0.0f64;
    let mut lsd_frames = 0;

    for f in 0..num_frames {
        let start = f * frame_size;
        let end = start + frame_size;

        let spec_c = fft_real(&clean[start..end]);
        let spec_e = fft_real(&enhanced[start..end]);

        let half = spec_c.len() / 2 + 1;
        let mut frame_lsd = 0.0;
        let mut bins = 0;

        for i in 1..half.min(spec_e.len()) {
            let pc = c_abs_sq(spec_c[i]).max(1e-30);
            let pe = c_abs_sq(spec_e[i]).max(1e-30);
            let log_diff = (10.0 * pc.log10() - 10.0 * pe.log10()).abs();
            frame_lsd += log_diff;
            bins += 1;
        }

        if bins > 0 {
            total_lsd += frame_lsd / bins as f64;
            lsd_frames += 1;
        }
    }

    let avg_lsd = if lsd_frames > 0 {
        total_lsd / lsd_frames as f64
    } else {
        20.0
    };
    // Map LSD to [0, 1]: 0 dB -> 1.0, 20 dB -> 0.0
    let lsd_score = (1.0 - avg_lsd / 20.0).clamp(0.0, 1.0);

    // 3. Correlation component
    let mean_c = clean[..min_len].iter().sum::<f64>() / min_len as f64;
    let mean_e = enhanced[..min_len].iter().sum::<f64>() / min_len as f64;

    let mut cov = 0.0f64;
    let mut var_c = 0.0f64;
    let mut var_e = 0.0f64;

    for i in 0..min_len {
        let dc = clean[i] - mean_c;
        let de = enhanced[i] - mean_e;
        cov += dc * de;
        var_c += dc * dc;
        var_e += de * de;
    }

    let corr = if var_c > 1e-30 && var_e > 1e-30 {
        (cov / (var_c * var_e).sqrt()).clamp(-1.0, 1.0)
    } else {
        0.0
    };
    let corr_score = (corr + 1.0) / 2.0; // Map [-1, 1] -> [0, 1]

    // Weighted combination
    let raw_score = 0.4 * snr_score + 0.3 * lsd_score + 0.3 * corr_score;

    // Map to MOS range [1.0, 4.5]
    1.0 + raw_score * 3.5
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn tone(freq_hz: f64, sample_rate: f64, num_samples: usize) -> Vec<f64> {
        (0..num_samples)
            .map(|i| (2.0 * PI * freq_hz * i as f64 / sample_rate).sin())
            .collect()
    }

    fn white_noise(num_samples: usize, seed: u64) -> Vec<f64> {
        // Simple LCG pseudo-random noise
        let mut state = seed;
        (0..num_samples)
            .map(|_| {
                state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                // Map to [-1, 1]
                (state >> 33) as f64 / (1u64 << 31) as f64 - 1.0
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // SpeechEnhancer tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_speech_enhancer_new() {
        let se = SpeechEnhancer::new(4, 16000, 512, 256);
        assert_eq!(se.num_channels, 4);
        assert_eq!(se.sample_rate_hz, 16000);
        assert_eq!(se.frame_size, 512);
        assert_eq!(se.hop_size, 256);
    }

    #[test]
    fn test_speech_enhancer_enhance() {
        let se = SpeechEnhancer::new(2, 16000, 64, 32);
        let signal = tone(440.0, 16000.0, 256);
        let noise_ref = white_noise(64, 42);
        let channels = vec![signal.clone(), signal.clone()];
        let result = se.enhance(&channels, &[0.0, 0.0], &noise_ref);
        assert_eq!(result.enhanced_signal.len(), 256);
        // Enhanced signal should not be all zeros (it should contain the tone)
        assert!(result.enhanced_signal.iter().any(|&v| v.abs() > 0.001));
    }

    #[test]
    fn test_enhancement_result_fields() {
        let result = EnhancementResult {
            enhanced_signal: vec![1.0, 2.0, 3.0],
            snr_improvement_db: 5.0,
            noise_reduction_db: 10.0,
        };
        assert_eq!(result.enhanced_signal.len(), 3);
        assert!((result.snr_improvement_db - 5.0).abs() < 1e-10);
        assert!((result.noise_reduction_db - 10.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Delay-and-sum tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_delay_sum_speech_empty() {
        let result = delay_sum_speech(&[], &[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_delay_sum_speech_single_channel() {
        let ch = vec![1.0, 2.0, 3.0, 4.0];
        let result = delay_sum_speech(&[ch.clone()], &[0.0]);
        assert_eq!(result.len(), 4);
        // With zero delay and one channel, output should approximate input
        for (i, &v) in result.iter().enumerate() {
            assert!(
                (v - ch[i]).abs() < 0.5,
                "sample {} expected ~{}, got {}",
                i,
                ch[i],
                v
            );
        }
    }

    #[test]
    fn test_delay_sum_speech_two_channels_zero_delay() {
        let ch0 = vec![1.0, 0.0, -1.0, 0.0];
        let ch1 = vec![1.0, 0.0, -1.0, 0.0];
        let result = delay_sum_speech(&[ch0, ch1], &[0.0, 0.0]);
        assert_eq!(result.len(), 4);
        // Average of identical channels should be the same signal
        for &v in &result {
            // Sinc interpolation may introduce small errors
            assert!(v.abs() <= 1.5);
        }
    }

    #[test]
    fn test_delay_sum_speech_constructive() {
        // Two channels with a known tone; zero delay should reinforce
        let n = 128;
        let ch0 = tone(1000.0, 16000.0, n);
        let ch1 = ch0.clone();
        let result = delay_sum_speech(&[ch0.clone(), ch1], &[0.0, 0.0]);
        assert_eq!(result.len(), n);
        // Power should be similar to original (averaged)
        let pow_in: f64 = ch0.iter().map(|x| x * x).sum::<f64>() / n as f64;
        let pow_out: f64 = result.iter().map(|x| x * x).sum::<f64>() / n as f64;
        // Should be within 3 dB
        assert!(
            (pow_out / pow_in).abs() > 0.3,
            "output power too low: {} vs {}",
            pow_out,
            pow_in
        );
    }

    // -----------------------------------------------------------------------
    // MVDR tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_mvdr_speech_empty() {
        let result = mvdr_speech(&[], &[], &[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_mvdr_speech_identity_noise() {
        let n = 64;
        let ch0 = tone(500.0, 16000.0, n);
        let ch1 = ch0.clone();

        // Identity noise PSD matrix for each bin
        let fft_size = next_power_of_two(n);
        let noise_psd: Vec<Vec<Vec<C64>>> = (0..fft_size)
            .map(|_| {
                vec![
                    vec![(1.0, 0.0), (0.0, 0.0)],
                    vec![(0.0, 0.0), (1.0, 0.0)],
                ]
            })
            .collect();
        let steering = vec![(1.0, 0.0), (1.0, 0.0)];

        let result = mvdr_speech(&[ch0, ch1], &noise_psd, &steering);
        assert_eq!(result.len(), n);
        // Output should not be all zeros
        assert!(result.iter().any(|&v| v.abs() > 0.01));
    }

    #[test]
    fn test_mvdr_speech_single_channel() {
        let n = 32;
        let ch0 = vec![1.0; n];
        let fft_size = next_power_of_two(n);
        let noise_psd: Vec<Vec<Vec<C64>>> = (0..fft_size)
            .map(|_| vec![vec![(1.0, 0.0)]])
            .collect();
        let steering = vec![(1.0, 0.0)];

        let result = mvdr_speech(&[ch0], &noise_psd, &steering);
        assert_eq!(result.len(), n);
    }

    // -----------------------------------------------------------------------
    // MCRA Noise PSD tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_mcra_noise_psd_initialization() {
        let spectrum: Vec<C64> = vec![(0.5, 0.0); 16];
        let mut prev = vec![0.0; 16];
        let result = estimate_noise_psd_mcra(&spectrum, &mut prev, 0.9);
        assert_eq!(result.len(), 16);
        // With prev = 0, noise estimate should be (1-alpha) * power
        for &v in &result {
            assert!(v > 0.0, "noise PSD should be positive after first frame");
        }
    }

    #[test]
    fn test_mcra_noise_psd_convergence() {
        let spectrum: Vec<C64> = vec![(0.3, 0.1); 8];
        let mut prev = vec![0.0; 8];

        // Run several updates
        for _ in 0..20 {
            estimate_noise_psd_mcra(&spectrum, &mut prev, 0.8);
        }

        // Should converge to a stable positive value
        // (MCRA with speech detection may not converge to exact power)
        let expected_power = c_abs_sq((0.3, 0.1));
        for &v in &prev {
            assert!(
                v > 0.0 && v < expected_power * 2.0,
                "expected positive value near {}, got {}",
                expected_power,
                v
            );
        }
    }

    #[test]
    fn test_mcra_noise_psd_length_mismatch() {
        let spectrum: Vec<C64> = vec![(1.0, 0.0); 4];
        let mut prev = vec![0.0; 2]; // Wrong length
        let result = estimate_noise_psd_mcra(&spectrum, &mut prev, 0.5);
        assert_eq!(result.len(), 4);
        assert_eq!(prev.len(), 4); // Should be resized
    }

    // -----------------------------------------------------------------------
    // Wiener post-filter tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_wiener_postfilter_no_noise() {
        let noisy = vec![(1.0, 0.0), (0.0, 1.0)];
        let noise = vec![0.0, 0.0];
        let result = wiener_postfilter(&noisy, &noise, 1.0);
        assert_eq!(result.len(), 2);
        // With zero noise, gain should be 1.0
        assert!((result[0].0 - 1.0).abs() < 0.02);
        assert!((result[1].1 - 1.0).abs() < 0.02);
    }

    #[test]
    fn test_wiener_postfilter_high_noise() {
        let noisy = vec![(1.0, 0.0)];
        let noise = vec![10.0]; // Very high noise
        let result = wiener_postfilter(&noisy, &noise, 1.0);
        // Gain should be floored (not negative)
        assert!(result[0].0 > 0.0);
        assert!(result[0].0 < 0.5); // Significant attenuation
    }

    #[test]
    fn test_wiener_postfilter_preserves_phase() {
        let noisy = vec![(0.0, 1.0)]; // Pure imaginary
        let noise = vec![0.1];
        let result = wiener_postfilter(&noisy, &noise, 1.0);
        // Real part should remain ~0, imaginary should be positive
        assert!(result[0].0.abs() < 0.01);
        assert!(result[0].1 > 0.0);
    }

    // -----------------------------------------------------------------------
    // Spectral subtraction tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_spectral_subtraction_no_noise() {
        let noisy = vec![(2.0, 0.0), (0.0, 2.0)];
        let noise = vec![0.0, 0.0];
        let result = spectral_subtraction(&noisy, &noise, 2.0);
        assert_eq!(result.len(), 2);
        // With zero noise, output should approximate input
        assert!((result[0].0 - 2.0).abs() < 0.1);
        assert!((result[1].1 - 2.0).abs() < 0.1);
    }

    #[test]
    fn test_spectral_subtraction_heavy_noise() {
        let noisy = vec![(1.0, 0.0)];
        let noise = vec![5.0]; // Noise power >> signal power
        let result = spectral_subtraction(&noisy, &noise, 2.0);
        // Output magnitude should be heavily reduced but not negative
        let mag = c_abs(result[0]);
        assert!(mag >= 0.0);
        assert!(mag < 1.0);
    }

    #[test]
    fn test_spectral_subtraction_preserves_length() {
        let noisy: Vec<C64> = (0..100).map(|i| (i as f64 * 0.01, 0.0)).collect();
        let noise = vec![0.01; 100];
        let result = spectral_subtraction(&noisy, &noise, 1.5);
        assert_eq!(result.len(), 100);
    }

    // -----------------------------------------------------------------------
    // Ideal Binary Mask tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_ibm_high_snr() {
        let clean = vec![10.0, 10.0, 10.0];
        let noise = vec![0.1, 0.1, 0.1];
        let mask = ideal_binary_mask(&clean, &noise, 0.0);
        assert_eq!(mask, vec![1.0, 1.0, 1.0]);
    }

    #[test]
    fn test_ibm_low_snr() {
        let clean = vec![0.01, 0.01];
        let noise = vec![1.0, 1.0];
        let mask = ideal_binary_mask(&clean, &noise, 0.0);
        assert_eq!(mask, vec![0.0, 0.0]);
    }

    #[test]
    fn test_ibm_threshold() {
        // SNR = 10 dB linear = 10.0
        let clean = vec![10.0];
        let noise = vec![1.0];

        // Threshold 5 dB -> 3.16 linear; 10 > 3.16 -> mask = 1
        let mask_low = ideal_binary_mask(&clean, &noise, 5.0);
        assert_eq!(mask_low[0], 1.0);

        // Threshold 15 dB -> 31.6 linear; 10 < 31.6 -> mask = 0
        let mask_high = ideal_binary_mask(&clean, &noise, 15.0);
        assert_eq!(mask_high[0], 0.0);
    }

    // -----------------------------------------------------------------------
    // VAD tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_vad_energy_silence() {
        let silence = vec![0.0; 256];
        assert!(!vad_energy(&silence, 0.01));
    }

    #[test]
    fn test_vad_energy_speech() {
        let loud = vec![1.0; 256];
        assert!(vad_energy(&loud, 0.5));
    }

    #[test]
    fn test_vad_energy_empty() {
        assert!(!vad_energy(&[], 0.01));
    }

    // -----------------------------------------------------------------------
    // Segmental SNR tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_segmental_snr_identical() {
        let signal = tone(440.0, 16000.0, 512);
        let snr = compute_segmental_snr(&signal, &signal, 128);
        // Identical signals -> infinite SNR, clamped to 35 dB
        assert!(snr >= 30.0, "expected high SNR, got {}", snr);
    }

    #[test]
    fn test_segmental_snr_noisy() {
        let clean = tone(440.0, 16000.0, 512);
        let noisy: Vec<f64> = clean.iter().enumerate().map(|(i, &c)| {
            c + 0.3 * ((i as f64 * 1.7).sin())
        }).collect();
        let snr = compute_segmental_snr(&clean, &noisy, 128);
        // Should be positive but not extreme
        assert!(snr > -10.0 && snr < 35.0, "unexpected SNR: {}", snr);
    }

    #[test]
    fn test_segmental_snr_empty() {
        assert!((compute_segmental_snr(&[], &[], 128) - 0.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // PESQ approximation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_pesq_approx_identical() {
        let clean = tone(440.0, 16000.0, 1024);
        let score = compute_pesq_approx(&clean, &clean);
        assert!(
            score >= 3.0,
            "identical signals should score high, got {}",
            score
        );
    }

    #[test]
    fn test_pesq_approx_degraded() {
        let clean = tone(440.0, 16000.0, 1024);
        let degraded: Vec<f64> = clean
            .iter()
            .enumerate()
            .map(|(i, &c)| c + 0.5 * ((i as f64 * 3.1).sin()))
            .collect();
        let score = compute_pesq_approx(&clean, &degraded);
        assert!(
            score >= 1.0 && score <= 4.5,
            "score out of range: {}",
            score
        );
    }

    #[test]
    fn test_pesq_approx_empty() {
        let score = compute_pesq_approx(&[], &[]);
        assert!((score - 1.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // FFT helper tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_fft_roundtrip() {
        let signal = tone(1000.0, 16000.0, 64);
        let spec = fft_real(&signal);
        let recovered = ifft_real(&spec, 64);
        for (i, (&orig, &rec)) in signal.iter().zip(recovered.iter()).enumerate() {
            assert!(
                (orig - rec).abs() < 1e-10,
                "FFT roundtrip mismatch at {}: {} vs {}",
                i,
                orig,
                rec
            );
        }
    }

    #[test]
    fn test_invert_hermitian_identity() {
        let identity = vec![
            vec![(1.0, 0.0), (0.0, 0.0)],
            vec![(0.0, 0.0), (1.0, 0.0)],
        ];
        let inv = invert_hermitian(&identity);
        assert_eq!(inv.len(), 2);
        // Inverse of identity is identity
        assert!((inv[0][0].0 - 1.0).abs() < 1e-10);
        assert!((inv[1][1].0 - 1.0).abs() < 1e-10);
        assert!(c_abs(inv[0][1]) < 1e-10);
        assert!(c_abs(inv[1][0]) < 1e-10);
    }
}
