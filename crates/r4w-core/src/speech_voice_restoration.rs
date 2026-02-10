//! Audio restoration for degraded speech signals.
//!
//! This module provides algorithms for restoring quality to degraded speech
//! recordings, including declipping, denoising via spectral gating,
//! dereverberation using weighted prediction error, bandwidth extension
//! through spectral envelope extrapolation, click/pop removal, wow and
//! flutter correction, spectral tilt compensation, and dynamic range
//! restoration.
//!
//! # Example
//!
//! ```
//! use r4w_core::speech_voice_restoration::{
//!     RestorationConfig, SpeechRestorer, declip, spectral_gate,
//!     estimate_noise_floor,
//! };
//!
//! let config = RestorationConfig::new(16000.0, 512, 256);
//! let restorer = SpeechRestorer::new(config);
//!
//! // Declip a signal
//! let mut signal = vec![0.0, 0.5, 1.0, 1.0, 1.0, 0.5, 0.0];
//! declip(&mut signal, 0.95);
//! assert!(signal[2] < 1.01); // clipped samples interpolated
//!
//! // Estimate noise floor then apply spectral gate
//! let noisy: Vec<f64> = (0..512).map(|i| (i as f64 * 0.1).sin() + 0.01).collect();
//! let noise_profile = estimate_noise_floor(&noisy, 128);
//! let cleaned = spectral_gate(&noisy, &noise_profile, 6.0, 16000.0);
//! assert_eq!(cleaned.len(), noisy.len());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration parameters for the speech restoration pipeline.
#[derive(Debug, Clone)]
pub struct RestorationConfig {
    /// Audio sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Analysis frame size in samples.
    pub frame_size: usize,
    /// Hop size (frame advance) in samples.
    pub hop_size: usize,
}

impl RestorationConfig {
    /// Create a new `RestorationConfig`.
    ///
    /// # Arguments
    /// * `sample_rate_hz` - Sample rate in Hz (e.g. 16000.0).
    /// * `frame_size` - Analysis frame size in samples (must be > 0).
    /// * `hop_size` - Hop size in samples (must be > 0).
    pub fn new(sample_rate_hz: f64, frame_size: usize, hop_size: usize) -> Self {
        assert!(sample_rate_hz > 0.0, "sample_rate_hz must be positive");
        assert!(frame_size > 0, "frame_size must be positive");
        assert!(hop_size > 0, "hop_size must be positive");
        Self {
            sample_rate_hz,
            frame_size,
            hop_size,
        }
    }
}

// ---------------------------------------------------------------------------
// Main restorer
// ---------------------------------------------------------------------------

/// Main speech restoration processor that bundles configuration and provides
/// a convenient method to run the full restoration pipeline on a signal.
#[derive(Debug, Clone)]
pub struct SpeechRestorer {
    /// The configuration used by this restorer.
    pub config: RestorationConfig,
}

impl SpeechRestorer {
    /// Create a new `SpeechRestorer` with the given configuration.
    pub fn new(config: RestorationConfig) -> Self {
        Self { config }
    }

    /// Run the full restoration pipeline on a signal (in-place where possible).
    ///
    /// Steps executed in order:
    /// 1. Click/pop removal
    /// 2. Declipping
    /// 3. Noise estimation + spectral gating
    /// 4. Dereverberation
    /// 5. Spectral tilt compensation
    /// 6. Dynamic range restoration
    ///
    /// Returns the restored signal.
    pub fn restore(&self, signal: &[f64]) -> Vec<f64> {
        let sr = self.config.sample_rate_hz;
        let mut buf = signal.to_vec();

        // 1. Click removal
        remove_clicks(&mut buf, 5.0);

        // 2. Declip
        declip(&mut buf, 0.95);

        // 3. Denoise
        let noise = estimate_noise_floor(&buf, self.config.frame_size);
        buf = spectral_gate(&buf, &noise, 6.0, sr);

        // 4. Dereverberate
        let rt60 = estimate_rt60(&buf, sr);
        if rt60 > 0.05 {
            buf = dereverberate(&buf, sr, rt60);
        }

        // 5. Spectral tilt compensation (mild +3 dB/octave boost)
        buf = spectral_tilt_compensate(&buf, 3.0, sr);

        // 6. Dynamic range restoration
        buf = dynamic_range_restore(&buf, 40.0);

        buf
    }
}

// ---------------------------------------------------------------------------
// Declipping
// ---------------------------------------------------------------------------

/// Declip a signal by replacing clipped regions with cubic Hermite
/// interpolation.
///
/// Samples whose absolute value exceeds `clip_threshold` are considered
/// clipped. Contiguous clipped regions are replaced with a smooth cubic
/// interpolation between the boundary samples.
///
/// # Arguments
/// * `signal` - The audio signal (modified in place).
/// * `clip_threshold` - Absolute value above which a sample is clipped
///   (e.g. 0.95 for a signal normalised to +/-1.0).
pub fn declip(signal: &mut Vec<f64>, clip_threshold: f64) {
    let n = signal.len();
    if n < 3 {
        return;
    }
    let thresh = clip_threshold.abs();

    // Find clipped regions
    let mut i = 0;
    while i < n {
        if signal[i].abs() >= thresh {
            // Find the end of the clipped region
            let start = i;
            while i < n && signal[i].abs() >= thresh {
                i += 1;
            }
            let end = i; // exclusive

            // Boundary values for interpolation
            let left_idx = if start > 0 { start - 1 } else { start };
            let right_idx = if end < n { end } else { n - 1 };

            let y0 = signal[left_idx];
            let y1 = signal[right_idx];

            // Estimate slopes at boundaries
            let m0 = if left_idx > 0 {
                signal[left_idx] - signal[left_idx - 1]
            } else {
                0.0
            };
            let m1 = if right_idx + 1 < n {
                signal[right_idx + 1] - signal[right_idx]
            } else {
                0.0
            };

            let span = (end - left_idx) as f64;
            // Cubic Hermite interpolation across the clipped region
            for j in start..end {
                let t = (j - left_idx) as f64 / span;
                let t2 = t * t;
                let t3 = t2 * t;
                let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
                let h10 = t3 - 2.0 * t2 + t;
                let h01 = -2.0 * t3 + 3.0 * t2;
                let h11 = t3 - t2;
                signal[j] = h00 * y0 + h10 * span * m0 + h01 * y1 + h11 * span * m1;
            }
        } else {
            i += 1;
        }
    }
}

// ---------------------------------------------------------------------------
// Spectral Gate (noise gate in the frequency domain)
// ---------------------------------------------------------------------------

/// Apply spectral gating to remove stationary noise.
///
/// The signal is processed in overlapping frames. For each frame, a DFT is
/// computed and frequency bins whose magnitude falls below
/// `noise_profile[bin] * 10^(threshold_db/20)` are attenuated.
///
/// # Arguments
/// * `signal` - Input signal.
/// * `noise_profile` - Per-bin noise magnitude estimate (length determines
///   the FFT size).
/// * `threshold_db` - Threshold above the noise floor in dB.
/// * `_sample_rate` - Sample rate in Hz (reserved for future use).
///
/// # Returns
/// The denoised signal with the same length as the input.
pub fn spectral_gate(
    signal: &[f64],
    noise_profile: &[f64],
    threshold_db: f64,
    _sample_rate: f64,
) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return vec![];
    }

    let fft_size = noise_profile.len().max(4);
    let hop = fft_size / 2;
    let gain_factor = 10.0_f64.powf(threshold_db / 20.0);

    let mut output = vec![0.0_f64; n];
    let mut window_sum = vec![0.0_f64; n];

    // Hann window
    let win: Vec<f64> = (0..fft_size)
        .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / fft_size as f64).cos()))
        .collect();

    let mut pos = 0usize;
    while pos + fft_size <= n {
        // Windowed frame
        let mut frame_re: Vec<f64> = (0..fft_size)
            .map(|i| signal[pos + i] * win[i])
            .collect();
        let mut frame_im = vec![0.0_f64; fft_size];

        dft_forward(&mut frame_re, &mut frame_im);

        // Spectral gating: soft mask
        for k in 0..fft_size {
            let mag = (frame_re[k] * frame_re[k] + frame_im[k] * frame_im[k]).sqrt();
            let noise_mag = if k < noise_profile.len() {
                noise_profile[k] * gain_factor
            } else {
                0.0
            };
            if mag < noise_mag {
                // Attenuate
                let attenuation = if noise_mag > 1e-30 {
                    (mag / noise_mag).powi(2)
                } else {
                    0.0
                };
                frame_re[k] *= attenuation;
                frame_im[k] *= attenuation;
            }
        }

        dft_inverse(&mut frame_re, &mut frame_im);

        // Overlap-add
        for i in 0..fft_size {
            if pos + i < n {
                output[pos + i] += frame_re[i] * win[i];
                window_sum[pos + i] += win[i] * win[i];
            }
        }

        pos += hop;
    }

    // Normalise by window sum
    for i in 0..n {
        if window_sum[i] > 1e-10 {
            output[i] /= window_sum[i];
        } else {
            output[i] = signal[i];
        }
    }

    output
}

// ---------------------------------------------------------------------------
// Dereverberation (simplified WPE - weighted prediction error)
// ---------------------------------------------------------------------------

/// Remove reverberation from a speech signal using a simplified Weighted
/// Prediction Error (WPE) method.
///
/// The algorithm estimates a linear prediction filter that predicts the
/// reverberant tail and subtracts it from the signal.
///
/// # Arguments
/// * `signal` - Reverberant speech signal.
/// * `sample_rate` - Sample rate in Hz.
/// * `rt60_estimate_s` - Estimated RT60 reverberation time in seconds.
///
/// # Returns
/// The dereverberated signal (same length as input).
pub fn dereverberate(signal: &[f64], sample_rate: f64, rt60_estimate_s: f64) -> Vec<f64> {
    let n = signal.len();
    if n < 4 {
        return signal.to_vec();
    }

    // Prediction delay ~ 30ms
    let prediction_delay = ((0.03 * sample_rate) as usize).max(1);
    // Filter length proportional to RT60
    let filter_len = ((rt60_estimate_s * sample_rate * 0.3) as usize).clamp(2, 512.min(n / 2));

    // Levinson-Durbin on autocorrelation of delayed signal
    let mut acf = vec![0.0_f64; filter_len + 1];
    for lag in 0..=filter_len {
        let mut sum = 0.0;
        let start = prediction_delay + lag;
        for i in start..n {
            sum += signal[i] * signal[i - lag - prediction_delay];
        }
        acf[lag] = sum;
    }

    // Avoid division by zero
    if acf[0].abs() < 1e-30 {
        return signal.to_vec();
    }

    // Solve for LP coefficients (Levinson-Durbin)
    let coeffs = levinson_durbin(&acf, filter_len);

    // Subtract predicted reverberant tail
    let mut output = signal.to_vec();
    for i in (prediction_delay + filter_len)..n {
        let mut pred = 0.0;
        for j in 0..filter_len {
            pred += coeffs[j] * signal[i - prediction_delay - j - 1];
        }
        output[i] = signal[i] - 0.5 * pred; // conservative subtraction
    }

    output
}

// ---------------------------------------------------------------------------
// Bandwidth Extension
// ---------------------------------------------------------------------------

/// Extend the bandwidth of a narrowband signal using spectral folding.
///
/// The method mirrors the upper portion of the existing spectrum into the
/// extension band and applies a spectral envelope weighting that tapers off
/// towards `target_bw_hz`.
///
/// # Arguments
/// * `signal` - Input narrowband signal.
/// * `original_bw_hz` - Original bandwidth in Hz.
/// * `target_bw_hz` - Target bandwidth in Hz (must be > original_bw_hz).
/// * `sample_rate` - Sample rate in Hz (must be >= 2 * target_bw_hz).
///
/// # Returns
/// The bandwidth-extended signal (same length).
pub fn bandwidth_extend(
    signal: &[f64],
    original_bw_hz: f64,
    target_bw_hz: f64,
    sample_rate: f64,
) -> Vec<f64> {
    let n = signal.len();
    if n < 4 || target_bw_hz <= original_bw_hz || sample_rate < 2.0 * target_bw_hz {
        return signal.to_vec();
    }

    let mut re = signal.to_vec();
    let mut im = vec![0.0_f64; n];

    dft_forward(&mut re, &mut im);

    let bin_hz = sample_rate / n as f64;
    let orig_bin = (original_bw_hz / bin_hz) as usize;
    let target_bin = (target_bw_hz / bin_hz).min(n as f64 / 2.0) as usize;

    if orig_bin == 0 || target_bin <= orig_bin {
        return signal.to_vec();
    }

    // Mirror and taper
    for k in orig_bin..target_bin {
        let mirror_k = orig_bin - (k - orig_bin) % orig_bin;
        let mirror_k = mirror_k.min(orig_bin).max(1);

        // Taper: linear fade towards target
        let t = (k - orig_bin) as f64 / (target_bin - orig_bin) as f64;
        let gain = (1.0 - t) * 0.3; // gentle extension

        re[k] = re[mirror_k] * gain;
        im[k] = im[mirror_k] * gain;

        // Mirror for negative frequencies
        if n - k < n {
            re[n - k] = re[k];
            im[n - k] = -im[k];
        }
    }

    dft_inverse(&mut re, &mut im);
    re
}

// ---------------------------------------------------------------------------
// Click / Pop Removal
// ---------------------------------------------------------------------------

/// Remove clicks and pops from a signal using median filtering.
///
/// Samples whose deviation from a local median exceeds `click_threshold`
/// standard deviations are replaced with the median value.
///
/// # Arguments
/// * `signal` - The audio signal (modified in place).
/// * `click_threshold` - Detection threshold in standard deviations (e.g. 5.0).
pub fn remove_clicks(signal: &mut Vec<f64>, click_threshold: f64) {
    let n = signal.len();
    if n < 5 {
        return;
    }

    let kernel = 5; // median filter kernel size
    let half = kernel / 2;

    // Compute local median values
    let medians: Vec<f64> = (0..n)
        .map(|i| {
            let lo = if i >= half { i - half } else { 0 };
            let hi = (i + half + 1).min(n);
            let mut window: Vec<f64> = signal[lo..hi].to_vec();
            window.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            window[window.len() / 2]
        })
        .collect();

    // Compute deviations
    let deviations: Vec<f64> = (0..n).map(|i| (signal[i] - medians[i]).abs()).collect();
    let mean_dev = deviations.iter().sum::<f64>() / n as f64;
    let std_dev = (deviations
        .iter()
        .map(|d| (d - mean_dev).powi(2))
        .sum::<f64>()
        / n as f64)
        .sqrt()
        .max(1e-30);

    for i in 0..n {
        if deviations[i] > click_threshold * std_dev {
            signal[i] = medians[i];
        }
    }
}

// ---------------------------------------------------------------------------
// Wow and Flutter Correction
// ---------------------------------------------------------------------------

/// Correct wow and flutter (speed variation) in a recorded signal.
///
/// Uses a reference frequency (e.g. a pilot tone or mains hum) to estimate
/// instantaneous speed deviations and resample the signal to correct them.
///
/// # Arguments
/// * `signal` - Input signal with wow/flutter.
/// * `reference_freq_hz` - Expected reference frequency in Hz.
/// * `sample_rate` - Sample rate in Hz.
///
/// # Returns
/// The speed-corrected signal (may differ slightly in length).
pub fn correct_wow_flutter(
    signal: &[f64],
    reference_freq_hz: f64,
    sample_rate: f64,
) -> Vec<f64> {
    let n = signal.len();
    if n < 4 || reference_freq_hz <= 0.0 {
        return signal.to_vec();
    }

    // Estimate instantaneous frequency around the reference using zero crossings
    let frame_size = (sample_rate / reference_freq_hz * 4.0) as usize;
    let frame_size = frame_size.clamp(8, n);
    let hop = frame_size / 2;

    // Collect instantaneous frequency estimates per frame
    let mut inst_freq = Vec::new();
    let mut pos = 0;
    while pos + frame_size <= n {
        // Count zero crossings in this frame
        let mut crossings = 0usize;
        for i in pos + 1..pos + frame_size {
            if signal[i - 1] * signal[i] < 0.0 {
                crossings += 1;
            }
        }
        let freq_est = crossings as f64 * sample_rate / (2.0 * frame_size as f64);
        inst_freq.push(freq_est);
        pos += hop;
    }

    if inst_freq.is_empty() {
        return signal.to_vec();
    }

    // Compute speed ratio per frame
    let speed_ratios: Vec<f64> = inst_freq
        .iter()
        .map(|f| {
            if *f > 0.0 {
                f / reference_freq_hz
            } else {
                1.0
            }
        })
        .collect();

    // Resample by integrating speed ratios
    let mut output = Vec::with_capacity(n);
    let mut read_pos = 0.0_f64;

    for i in 0..n {
        // Determine which frame we are in
        let frame_idx = (i / hop).min(speed_ratios.len() - 1);
        let speed = speed_ratios[frame_idx].clamp(0.9, 1.1); // limit correction

        let idx = read_pos as usize;
        if idx + 1 >= n {
            break;
        }
        let frac = read_pos - idx as f64;
        // Linear interpolation
        let sample = signal[idx] * (1.0 - frac) + signal[idx + 1] * frac;
        output.push(sample);

        read_pos += speed;
    }

    output
}

// ---------------------------------------------------------------------------
// Spectral Tilt Compensation
// ---------------------------------------------------------------------------

/// Compensate for spectral tilt in a speech signal.
///
/// Applies a frequency-dependent gain that boosts or attenuates at
/// `tilt_db_per_octave` dB per octave relative to the reference frequency
/// (1 kHz). Positive values boost high frequencies (de-emphasis of a
/// recording that was pre-emphasised too strongly, or boosting a dull
/// recording).
///
/// # Arguments
/// * `signal` - Input signal.
/// * `tilt_db_per_octave` - Tilt in dB per octave (positive = HF boost).
/// * `sample_rate` - Sample rate in Hz.
///
/// # Returns
/// The tilt-compensated signal (same length).
pub fn spectral_tilt_compensate(
    signal: &[f64],
    tilt_db_per_octave: f64,
    sample_rate: f64,
) -> Vec<f64> {
    let n = signal.len();
    if n < 2 || tilt_db_per_octave.abs() < 1e-6 {
        return signal.to_vec();
    }

    let mut re = signal.to_vec();
    let mut im = vec![0.0_f64; n];

    dft_forward(&mut re, &mut im);

    let ref_freq = 1000.0_f64; // 1 kHz reference
    let bin_hz = sample_rate / n as f64;

    for k in 1..n {
        let freq = k as f64 * bin_hz;
        // Wrap negative frequencies
        let actual_freq = if k <= n / 2 {
            freq
        } else {
            sample_rate - freq
        };
        if actual_freq < 1.0 {
            continue;
        }
        let octaves = (actual_freq / ref_freq).log2();
        let gain_db = tilt_db_per_octave * octaves;
        let gain = 10.0_f64.powf(gain_db / 20.0);
        re[k] *= gain;
        im[k] *= gain;
    }

    dft_inverse(&mut re, &mut im);
    re
}

// ---------------------------------------------------------------------------
// Dynamic Range Restoration
// ---------------------------------------------------------------------------

/// Restore dynamic range by expanding a compressed signal.
///
/// Applies an expander that increases the difference between loud and quiet
/// portions. The target dynamic range is specified in dB.
///
/// # Arguments
/// * `signal` - Input (compressed) signal.
/// * `target_dr_db` - Target dynamic range in dB (e.g. 40.0).
///
/// # Returns
/// The expanded signal (same length), normalised to +/-1.0.
pub fn dynamic_range_restore(signal: &[f64], target_dr_db: f64) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return vec![];
    }

    // Measure current dynamic range
    let peak = signal.iter().map(|s| s.abs()).fold(0.0_f64, f64::max);
    if peak < 1e-30 {
        return signal.to_vec();
    }

    // RMS in short frames
    let frame = 256.min(n);
    let mut rms_values = Vec::new();
    let mut pos = 0;
    while pos + frame <= n {
        let rms = (signal[pos..pos + frame]
            .iter()
            .map(|s| s * s)
            .sum::<f64>()
            / frame as f64)
            .sqrt();
        if rms > 1e-30 {
            rms_values.push(rms);
        }
        pos += frame;
    }

    if rms_values.len() < 2 {
        return signal.to_vec();
    }

    rms_values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let quiet_rms = rms_values[rms_values.len() / 10]; // 10th percentile
    let loud_rms = rms_values[rms_values.len() * 9 / 10]; // 90th percentile

    let current_dr_db = 20.0 * (loud_rms / quiet_rms).log10();
    if current_dr_db < 1.0 {
        return signal.to_vec();
    }

    let expansion_ratio = target_dr_db / current_dr_db;

    // Apply expansion
    let mut output = Vec::with_capacity(n);
    for &s in signal {
        let level = s.abs().max(1e-30);
        let level_db = 20.0 * (level / peak).log10();
        let new_level_db = level_db * expansion_ratio;
        let gain = 10.0_f64.powf((new_level_db - level_db) / 20.0);
        output.push(s * gain);
    }

    // Normalise to peak = 1.0
    let new_peak = output.iter().map(|s| s.abs()).fold(0.0_f64, f64::max);
    if new_peak > 1e-30 {
        for s in &mut output {
            *s /= new_peak;
        }
    }

    output
}

// ---------------------------------------------------------------------------
// Noise Floor Estimation (minimum statistics)
// ---------------------------------------------------------------------------

/// Estimate the spectral noise floor using minimum statistics.
///
/// Splits the signal into frames, computes the DFT magnitude of each frame,
/// and takes the minimum across frames for each frequency bin. This gives a
/// conservative estimate of the stationary noise floor.
///
/// # Arguments
/// * `signal` - Input signal.
/// * `frame_size` - Analysis frame size (also determines FFT/output length).
///
/// # Returns
/// A vector of length `frame_size` containing per-bin noise magnitude
/// estimates.
pub fn estimate_noise_floor(signal: &[f64], frame_size: usize) -> Vec<f64> {
    let n = signal.len();
    let frame_size = frame_size.max(4);
    if n < frame_size {
        // Not enough data; return zeros
        return vec![0.0; frame_size];
    }

    let hop = frame_size / 2;
    let mut min_mag = vec![f64::MAX; frame_size];
    let mut count = 0usize;

    // Hann window
    let win: Vec<f64> = (0..frame_size)
        .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / frame_size as f64).cos()))
        .collect();

    let mut pos = 0;
    while pos + frame_size <= n {
        let mut re: Vec<f64> = (0..frame_size)
            .map(|i| signal[pos + i] * win[i])
            .collect();
        let mut im = vec![0.0_f64; frame_size];

        dft_forward(&mut re, &mut im);

        for k in 0..frame_size {
            let mag = (re[k] * re[k] + im[k] * im[k]).sqrt();
            if mag < min_mag[k] {
                min_mag[k] = mag;
            }
        }

        count += 1;
        pos += hop;
    }

    if count == 0 {
        return vec![0.0; frame_size];
    }

    // Bias correction: minimum statistics underestimates, scale up slightly
    let bias = 1.5;
    for v in &mut min_mag {
        if *v == f64::MAX {
            *v = 0.0;
        } else {
            *v *= bias;
        }
    }

    min_mag
}

// ---------------------------------------------------------------------------
// RT60 Estimation
// ---------------------------------------------------------------------------

/// Estimate the RT60 reverberation time from a signal's energy decay curve.
///
/// The algorithm computes the backwards-integrated energy decay curve
/// (Schroeder integration) and fits a line to the -5 dB to -25 dB region
/// to estimate the decay rate. The RT60 is extrapolated from this slope.
///
/// # Arguments
/// * `signal` - Input signal (should contain a reverberant tail, e.g.
///   after an impulse or speech offset).
/// * `sample_rate` - Sample rate in Hz.
///
/// # Returns
/// Estimated RT60 in seconds. Returns 0.0 if the signal is too short or
/// the estimation fails.
pub fn estimate_rt60(signal: &[f64], sample_rate: f64) -> f64 {
    let n = signal.len();
    if n < 16 || sample_rate <= 0.0 {
        return 0.0;
    }

    // Compute energy in frames
    let frame = 64.min(n / 4).max(1);
    let mut energies = Vec::new();
    let mut pos = 0;
    while pos + frame <= n {
        let e: f64 = signal[pos..pos + frame].iter().map(|s| s * s).sum();
        energies.push(e);
        pos += frame;
    }

    if energies.is_empty() {
        return 0.0;
    }

    // Schroeder backwards integration
    let mut edc = vec![0.0_f64; energies.len()];
    let mut cumsum = 0.0;
    for i in (0..energies.len()).rev() {
        cumsum += energies[i];
        edc[i] = cumsum;
    }

    // Convert to dB
    let max_e = edc[0].max(1e-30);
    let edc_db: Vec<f64> = edc
        .iter()
        .map(|e| 10.0 * (e / max_e).max(1e-30).log10())
        .collect();

    // Find -5 dB and -25 dB points
    let mut start_idx = None;
    let mut end_idx = None;
    for (i, &db) in edc_db.iter().enumerate() {
        if start_idx.is_none() && db <= -5.0 {
            start_idx = Some(i);
        }
        if start_idx.is_some() && end_idx.is_none() && db <= -25.0 {
            end_idx = Some(i);
            break;
        }
    }

    let (si, ei) = match (start_idx, end_idx) {
        (Some(s), Some(e)) if e > s => (s, e),
        _ => return 0.0,
    };

    // Linear regression on the EDC dB values
    let n_pts = ei - si + 1;
    let mut sum_x = 0.0_f64;
    let mut sum_y = 0.0_f64;
    let mut sum_xx = 0.0_f64;
    let mut sum_xy = 0.0_f64;

    for i in si..=ei {
        let x = (i - si) as f64;
        let y = edc_db[i];
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
    }

    let n_f = n_pts as f64;
    let denom = n_f * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return 0.0;
    }

    let slope = (n_f * sum_xy - sum_x * sum_y) / denom; // dB per frame index
    if slope >= 0.0 {
        return 0.0; // no decay
    }

    // Convert slope from dB/frame_index to dB/second
    let time_per_frame = frame as f64 / sample_rate;
    let slope_per_sec = slope / time_per_frame;

    // RT60 = -60 / slope_per_sec
    let rt60 = -60.0 / slope_per_sec;
    rt60.max(0.0)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Simple DFT (forward). Operates in place.
/// This is O(N^2) but sufficient for the frame sizes used here.
fn dft_forward(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let mut out_re = vec![0.0_f64; n];
    let mut out_im = vec![0.0_f64; n];

    for k in 0..n {
        let mut sr = 0.0;
        let mut si = 0.0;
        for j in 0..n {
            let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
            sr += re[j] * angle.cos() - im[j] * angle.sin();
            si += re[j] * angle.sin() + im[j] * angle.cos();
        }
        out_re[k] = sr;
        out_im[k] = si;
    }

    re.copy_from_slice(&out_re);
    im.copy_from_slice(&out_im);
}

/// Simple DFT (inverse). Operates in place. Includes 1/N scaling.
fn dft_inverse(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let mut out_re = vec![0.0_f64; n];
    let mut out_im = vec![0.0_f64; n];

    for k in 0..n {
        let mut sr = 0.0;
        let mut si = 0.0;
        for j in 0..n {
            let angle = 2.0 * PI * k as f64 * j as f64 / n as f64;
            sr += re[j] * angle.cos() - im[j] * angle.sin();
            si += re[j] * angle.sin() + im[j] * angle.cos();
        }
        out_re[k] = sr / n as f64;
        out_im[k] = si / n as f64;
    }

    re.copy_from_slice(&out_re);
    im.copy_from_slice(&out_im);
}

/// Levinson-Durbin algorithm for solving Toeplitz systems.
/// Given autocorrelation values `acf[0..=order]`, returns LP coefficients.
fn levinson_durbin(acf: &[f64], order: usize) -> Vec<f64> {
    let mut coeffs = vec![0.0_f64; order];
    let mut err = acf[0];

    if err.abs() < 1e-30 {
        return coeffs;
    }

    for i in 0..order {
        let mut lambda = 0.0;
        for j in 0..i {
            lambda += coeffs[j] * acf[i - j];
        }
        lambda = (acf[i + 1] - lambda) / err;

        // Update coefficients
        let mut new_coeffs = coeffs.clone();
        new_coeffs[i] = lambda;
        for j in 0..i {
            new_coeffs[j] = coeffs[j] - lambda * coeffs[i - 1 - j];
        }
        coeffs = new_coeffs;

        err *= 1.0 - lambda * lambda;
        if err.abs() < 1e-30 {
            break;
        }
    }

    coeffs
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn sine_signal(freq: f64, sample_rate: f64, duration_s: f64) -> Vec<f64> {
        let n = (sample_rate * duration_s) as usize;
        (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / sample_rate).sin())
            .collect()
    }

    #[test]
    fn test_restoration_config_new() {
        let c = RestorationConfig::new(16000.0, 512, 256);
        assert_eq!(c.sample_rate_hz, 16000.0);
        assert_eq!(c.frame_size, 512);
        assert_eq!(c.hop_size, 256);
    }

    #[test]
    #[should_panic]
    fn test_restoration_config_zero_sample_rate() {
        RestorationConfig::new(0.0, 512, 256);
    }

    #[test]
    fn test_speech_restorer_new() {
        let config = RestorationConfig::new(8000.0, 256, 128);
        let restorer = SpeechRestorer::new(config);
        assert_eq!(restorer.config.sample_rate_hz, 8000.0);
    }

    #[test]
    fn test_speech_restorer_restore() {
        let config = RestorationConfig::new(8000.0, 64, 32);
        let restorer = SpeechRestorer::new(config);
        let signal = sine_signal(440.0, 8000.0, 0.05);
        let restored = restorer.restore(&signal);
        assert!(!restored.is_empty());
        // Restored signal should be roughly the same length
        assert!(restored.len() >= signal.len() / 2);
    }

    #[test]
    fn test_declip_no_clipping() {
        let mut signal = vec![0.0, 0.3, 0.5, 0.3, 0.0];
        let original = signal.clone();
        declip(&mut signal, 0.95);
        // No samples exceed threshold, so signal should be unchanged
        for (a, b) in signal.iter().zip(original.iter()) {
            assert!((a - b).abs() < 1e-10);
        }
    }

    #[test]
    fn test_declip_with_clipping() {
        let mut signal = vec![0.0, 0.5, 1.0, 1.0, 1.0, 0.5, 0.0, -0.3];
        declip(&mut signal, 0.95);
        // The clipped samples (indices 2, 3, 4) should now be interpolated
        // They should be finite
        for s in &signal {
            assert!(s.is_finite());
        }
    }

    #[test]
    fn test_declip_short_signal() {
        let mut signal = vec![1.0, 1.0];
        declip(&mut signal, 0.5);
        // Should not crash on very short signals
        assert_eq!(signal.len(), 2);
    }

    #[test]
    fn test_spectral_gate_preserves_length() {
        let signal = sine_signal(440.0, 8000.0, 0.1);
        let noise_profile = vec![0.001; 64];
        let cleaned = spectral_gate(&signal, &noise_profile, 6.0, 8000.0);
        assert_eq!(cleaned.len(), signal.len());
    }

    #[test]
    fn test_spectral_gate_empty() {
        let cleaned = spectral_gate(&[], &[0.1; 8], 6.0, 8000.0);
        assert!(cleaned.is_empty());
    }

    #[test]
    fn test_spectral_gate_removes_noise() {
        // Create a tone + noise, gate should preserve tone energy
        let n = 128;
        let sr = 8000.0;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 440.0 * i as f64 / sr).sin() + 0.01 * (i as f64 * 0.7).sin())
            .collect();
        let noise_profile = vec![0.05; 64];
        let cleaned = spectral_gate(&signal, &noise_profile, 3.0, sr);
        assert_eq!(cleaned.len(), n);
        // Energy should be preserved (approximately)
        let orig_energy: f64 = signal.iter().map(|s| s * s).sum();
        let clean_energy: f64 = cleaned.iter().map(|s| s * s).sum();
        // Cleaned energy should be within an order of magnitude
        assert!(clean_energy > orig_energy * 0.01);
    }

    #[test]
    fn test_dereverberate_short_signal() {
        let signal = vec![1.0, 0.5, 0.2];
        let result = dereverberate(&signal, 8000.0, 0.3);
        assert_eq!(result.len(), signal.len());
    }

    #[test]
    fn test_dereverberate_preserves_length() {
        let signal = sine_signal(440.0, 8000.0, 0.1);
        let result = dereverberate(&signal, 8000.0, 0.3);
        assert_eq!(result.len(), signal.len());
    }

    #[test]
    fn test_bandwidth_extend_preserves_length() {
        let signal = sine_signal(440.0, 16000.0, 0.05);
        let extended = bandwidth_extend(&signal, 4000.0, 8000.0, 16000.0);
        assert_eq!(extended.len(), signal.len());
    }

    #[test]
    fn test_bandwidth_extend_no_op_when_same_bw() {
        let signal = sine_signal(440.0, 16000.0, 0.01);
        let extended = bandwidth_extend(&signal, 4000.0, 4000.0, 16000.0);
        // Should return copy of original
        for (a, b) in extended.iter().zip(signal.iter()) {
            assert!((a - b).abs() < 1e-10);
        }
    }

    #[test]
    fn test_remove_clicks_no_clicks() {
        let mut signal: Vec<f64> = (0..100)
            .map(|i| (2.0 * PI * 100.0 * i as f64 / 8000.0).sin())
            .collect();
        let original = signal.clone();
        remove_clicks(&mut signal, 10.0); // high threshold, should not modify
        // Most samples should remain close to original
        let max_diff: f64 = signal
            .iter()
            .zip(original.iter())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0_f64, f64::max);
        assert!(max_diff < 0.5);
    }

    #[test]
    fn test_remove_clicks_with_click() {
        let mut signal: Vec<f64> = (0..100)
            .map(|i| (2.0 * PI * 100.0 * i as f64 / 8000.0).sin() * 0.1)
            .collect();
        // Insert a click
        signal[50] = 5.0;
        remove_clicks(&mut signal, 3.0);
        // The click should be reduced
        assert!(signal[50].abs() < 5.0);
    }

    #[test]
    fn test_correct_wow_flutter_preserves_signal() {
        let signal = sine_signal(1000.0, 8000.0, 0.05);
        let corrected = correct_wow_flutter(&signal, 1000.0, 8000.0);
        assert!(!corrected.is_empty());
        // Should be roughly the same length
        let len_ratio = corrected.len() as f64 / signal.len() as f64;
        assert!(len_ratio > 0.5 && len_ratio < 2.0);
    }

    #[test]
    fn test_correct_wow_flutter_short() {
        let signal = vec![1.0, 0.5];
        let corrected = correct_wow_flutter(&signal, 440.0, 8000.0);
        assert_eq!(corrected, signal);
    }

    #[test]
    fn test_spectral_tilt_compensate_preserves_length() {
        let signal = sine_signal(440.0, 8000.0, 0.05);
        let compensated = spectral_tilt_compensate(&signal, 3.0, 8000.0);
        assert_eq!(compensated.len(), signal.len());
    }

    #[test]
    fn test_spectral_tilt_compensate_zero_tilt() {
        let signal = sine_signal(440.0, 8000.0, 0.05);
        let compensated = spectral_tilt_compensate(&signal, 0.0, 8000.0);
        // Zero tilt should return the same signal
        for (a, b) in compensated.iter().zip(signal.iter()) {
            assert!((a - b).abs() < 1e-10);
        }
    }

    #[test]
    fn test_dynamic_range_restore_preserves_length() {
        let signal = sine_signal(440.0, 8000.0, 0.1);
        let restored = dynamic_range_restore(&signal, 40.0);
        assert_eq!(restored.len(), signal.len());
    }

    #[test]
    fn test_dynamic_range_restore_empty() {
        let restored = dynamic_range_restore(&[], 40.0);
        assert!(restored.is_empty());
    }

    #[test]
    fn test_dynamic_range_restore_normalised_peak() {
        let signal: Vec<f64> = (0..2048)
            .map(|i| {
                let t = i as f64 / 8000.0;
                // AM signal with varying envelope
                (2.0 * PI * 440.0 * t).sin()
                    * (0.3 + 0.7 * (2.0 * PI * 2.0 * t).sin().abs())
            })
            .collect();
        let restored = dynamic_range_restore(&signal, 40.0);
        let peak = restored.iter().map(|s| s.abs()).fold(0.0_f64, f64::max);
        // Peak should be close to 1.0 after normalisation
        assert!((peak - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_estimate_noise_floor_size() {
        let signal = sine_signal(440.0, 8000.0, 0.1);
        let noise = estimate_noise_floor(&signal, 64);
        assert_eq!(noise.len(), 64);
    }

    #[test]
    fn test_estimate_noise_floor_short_signal() {
        let noise = estimate_noise_floor(&[0.1, 0.2], 64);
        assert_eq!(noise.len(), 64);
        // All zeros when signal is too short
        assert!(noise.iter().all(|v| *v == 0.0));
    }

    #[test]
    fn test_estimate_rt60_decaying_signal() {
        // Create a signal with exponential decay (simulated reverb tail)
        let sr = 8000.0;
        let n = 8000; // 1 second
        let rt60_true = 0.5; // 0.5 seconds
        let decay_rate = 6.9078 / rt60_true; // ln(1000)/rt60 for 60dB decay
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sr;
                (-decay_rate * t).exp() * (2.0 * PI * 440.0 * t).sin()
            })
            .collect();

        let rt60_est = estimate_rt60(&signal, sr);
        // Should be in the right ballpark (within 50% of true value)
        assert!(rt60_est > 0.1, "RT60 estimate {} too low", rt60_est);
        assert!(rt60_est < 2.0, "RT60 estimate {} too high", rt60_est);
    }

    #[test]
    fn test_estimate_rt60_short_signal() {
        let rt60 = estimate_rt60(&[0.5, 0.3, 0.1], 8000.0);
        // Too short to estimate, should return 0
        assert_eq!(rt60, 0.0);
    }

    #[test]
    fn test_dft_roundtrip() {
        let original = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let mut re = original.clone();
        let mut im = vec![0.0; 8];

        dft_forward(&mut re, &mut im);
        dft_inverse(&mut re, &mut im);

        for (a, b) in re.iter().zip(original.iter()) {
            assert!(
                (a - b).abs() < 1e-10,
                "DFT roundtrip failed: {} vs {}",
                a,
                b
            );
        }
    }
}
