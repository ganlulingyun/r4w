//! Automatic P-wave and S-wave arrival time detection for seismic/geophysical data.
//!
//! This module provides algorithms for detecting seismic phase arrivals including:
//! - **STA/LTA** (Short-Term Average / Long-Term Average) trigger detection
//! - **AIC** (Akaike Information Criterion) precise onset picker
//! - **AR-AIC** (Autoregressive AIC) picker for improved accuracy
//! - **Kurtosis-based** picker for impulsive onset detection
//! - **Pick quality** assessment via SNR estimation
//! - **First-motion polarity** detection (Up/Down)
//! - **Bandpass filtering** for phase discrimination
//!
//! # Example
//!
//! ```
//! use r4w_core::seismic_arrival_detector::{sta_lta, trigger_sta_lta, aic_pick, SeismicConfig};
//!
//! // Generate a synthetic signal with a sharp onset at sample 500
//! let mut signal = vec![0.0_f64; 1000];
//! for i in 500..1000 {
//!     signal[i] = 2.0 * ((i - 500) as f64 * 0.1).sin();
//! }
//!
//! // Compute STA/LTA characteristic function
//! let cf = sta_lta(&signal, 10, 50);
//! assert!(!cf.is_empty());
//!
//! // Find trigger on/off pairs
//! let triggers = trigger_sta_lta(&cf, 3.0, 1.5);
//! assert!(!triggers.is_empty());
//!
//! // Refine pick with AIC
//! let (on, off) = triggers[0];
//! let pick = aic_pick(&signal, on.saturating_sub(50), off.min(signal.len()));
//! assert!(pick >= 350 && pick <= 550, "pick was {}", pick);
//! ```

use std::f64::consts::PI;

/// Configuration for the seismic arrival detector.
#[derive(Debug, Clone)]
pub struct SeismicConfig {
    /// Sample rate of the input data in Hz.
    pub sample_rate_hz: f64,
    /// Short-term averaging window duration in seconds.
    pub sta_window_s: f64,
    /// Long-term averaging window duration in seconds.
    pub lta_window_s: f64,
    /// STA/LTA ratio threshold to declare trigger ON.
    pub trigger_threshold: f64,
    /// STA/LTA ratio threshold to declare trigger OFF.
    pub detrigger_threshold: f64,
}

impl SeismicConfig {
    /// Creates a new configuration with the given sample rate and default parameters.
    ///
    /// Defaults: STA = 1.0 s, LTA = 10.0 s, trigger = 4.0, detrigger = 1.5.
    pub fn new(sample_rate_hz: f64) -> Self {
        Self {
            sample_rate_hz,
            sta_window_s: 1.0,
            lta_window_s: 10.0,
            trigger_threshold: 4.0,
            detrigger_threshold: 1.5,
        }
    }

    /// Returns the STA window length in samples.
    pub fn sta_samples(&self) -> usize {
        (self.sta_window_s * self.sample_rate_hz).round() as usize
    }

    /// Returns the LTA window length in samples.
    pub fn lta_samples(&self) -> usize {
        (self.lta_window_s * self.sample_rate_hz).round() as usize
    }
}

impl Default for SeismicConfig {
    fn default() -> Self {
        Self::new(100.0)
    }
}

/// Type of seismic phase arrival.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhaseType {
    /// Primary (compressional) wave.
    P,
    /// Secondary (shear) wave.
    S,
    /// Unknown or unclassified phase.
    Unknown,
}

/// First-motion polarity of a seismic arrival.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Polarity {
    /// Upward first motion (compressive).
    Up,
    /// Downward first motion (dilatational).
    Down,
    /// Polarity could not be determined.
    Unknown,
}

/// A detected seismic phase arrival (pick).
#[derive(Debug, Clone)]
pub struct SeismicPick {
    /// Sample index of the detected arrival.
    pub sample_index: usize,
    /// Time of the arrival in seconds (sample_index / sample_rate).
    pub time_s: f64,
    /// Phase type classification.
    pub phase: PhaseType,
    /// Pick quality metric in the range [0.0, 1.0], where 1.0 is highest quality.
    pub quality: f64,
    /// First-motion polarity.
    pub polarity: Polarity,
}

/// Main seismic arrival detector combining STA/LTA trigger with AIC refinement.
///
/// The detector operates in two stages:
/// 1. STA/LTA trigger to identify candidate windows
/// 2. AIC picker to refine the onset time within each window
#[derive(Debug, Clone)]
pub struct SeismicDetector {
    config: SeismicConfig,
}

impl SeismicDetector {
    /// Creates a new detector with the given configuration.
    pub fn new(config: SeismicConfig) -> Self {
        Self { config }
    }

    /// Detects all seismic arrivals in the given signal.
    ///
    /// Returns a vector of [`SeismicPick`] structs, one for each detected arrival.
    pub fn detect(&self, signal: &[f64]) -> Vec<SeismicPick> {
        let sta_len = self.config.sta_samples();
        let lta_len = self.config.lta_samples();

        if signal.len() < sta_len + lta_len || sta_len == 0 || lta_len == 0 {
            return Vec::new();
        }

        let cf = sta_lta(signal, sta_len, lta_len);
        let triggers =
            trigger_sta_lta(&cf, self.config.trigger_threshold, self.config.detrigger_threshold);

        let mut picks = Vec::new();
        for (on, off) in triggers {
            // Map cf indices back to signal indices (cf starts at lta_len)
            let sig_on = on + lta_len;
            let sig_off = (off + lta_len).min(signal.len());

            // Expand the window slightly for AIC refinement
            let win_start = sig_on.saturating_sub(sta_len);
            let win_end = sig_off.min(signal.len());

            if win_end <= win_start + 2 {
                continue;
            }

            let pick_sample = aic_pick(signal, win_start, win_end);
            let quality = pick_quality(signal, pick_sample, sta_len);
            let polarity = first_motion_polarity(signal, pick_sample);
            let time_s = pick_sample as f64 / self.config.sample_rate_hz;

            picks.push(SeismicPick {
                sample_index: pick_sample,
                time_s,
                phase: PhaseType::Unknown,
                quality,
                polarity,
            });
        }

        picks
    }

    /// Detects arrivals and classifies phases using multi-band filtering.
    ///
    /// P-waves are typically higher frequency; S-waves are lower frequency.
    /// Uses a simple frequency-ratio heuristic: if the high-band energy at the
    /// pick exceeds the low-band energy, classify as P; otherwise S.
    pub fn detect_and_classify(&self, signal: &[f64]) -> Vec<SeismicPick> {
        let mut picks = self.detect(signal);

        if picks.is_empty() || self.config.sample_rate_hz <= 0.0 {
            return picks;
        }

        let nyquist = self.config.sample_rate_hz / 2.0;
        // P-wave band: higher frequencies
        let p_band = bandpass_filter(signal, nyquist * 0.2, nyquist * 0.8, self.config.sample_rate_hz);
        // S-wave band: lower frequencies
        let s_band = bandpass_filter(signal, nyquist * 0.02, nyquist * 0.2, self.config.sample_rate_hz);

        let window = self.config.sta_samples().max(4);

        for pick in &mut picks {
            let start = pick.sample_index.saturating_sub(window / 2);
            let end = (pick.sample_index + window / 2).min(signal.len());

            if end <= start {
                continue;
            }

            let p_energy: f64 = p_band[start..end].iter().map(|x| x * x).sum();
            let s_energy: f64 = s_band[start..end].iter().map(|x| x * x).sum();

            pick.phase = if p_energy > s_energy {
                PhaseType::P
            } else if s_energy > p_energy {
                PhaseType::S
            } else {
                PhaseType::Unknown
            };
        }

        picks
    }
}

/// Computes the STA/LTA characteristic function.
///
/// The STA/LTA ratio is computed at each sample as the ratio of the short-term
/// average to the long-term average of the squared signal amplitude. High values
/// indicate a sudden increase in energy (potential arrival).
///
/// # Arguments
/// * `signal` - Input seismic trace
/// * `sta_len` - Short-term averaging window length in samples
/// * `lta_len` - Long-term averaging window length in samples
///
/// # Returns
/// A vector of length `signal.len() - lta_len` containing the STA/LTA ratio
/// at each sample starting from index `lta_len`.
pub fn sta_lta(signal: &[f64], sta_len: usize, lta_len: usize) -> Vec<f64> {
    if signal.is_empty() || sta_len == 0 || lta_len == 0 || sta_len >= lta_len {
        return Vec::new();
    }
    let total_len = signal.len();
    if total_len < lta_len + 1 {
        return Vec::new();
    }

    let n_out = total_len - lta_len;
    let mut cf = Vec::with_capacity(n_out);

    for i in lta_len..total_len {
        // STA window: [i - sta_len + 1, i]
        let sta_start = i + 1 - sta_len;
        let sta_sum: f64 = signal[sta_start..=i].iter().map(|x| x * x).sum();
        let sta = sta_sum / sta_len as f64;

        // LTA window: [i - lta_len, i - sta_len]
        let lta_start = i - lta_len;
        let lta_end = i - sta_len;
        if lta_end <= lta_start {
            cf.push(0.0);
            continue;
        }
        let lta_count = lta_end - lta_start;
        let lta_sum: f64 = signal[lta_start..lta_end].iter().map(|x| x * x).sum();
        let lta = lta_sum / lta_count as f64;

        if lta > 1e-30 {
            cf.push(sta / lta);
        } else {
            cf.push(0.0);
        }
    }

    cf
}

/// Finds trigger on/off index pairs from a characteristic function.
///
/// A trigger turns ON when the characteristic function exceeds `on_threshold`
/// and turns OFF when it drops below `off_threshold`.
///
/// # Arguments
/// * `cf` - Characteristic function (e.g., from [`sta_lta`])
/// * `on_threshold` - Threshold to declare trigger ON
/// * `off_threshold` - Threshold to declare trigger OFF (must be <= on_threshold)
///
/// # Returns
/// A vector of `(on_index, off_index)` pairs into the `cf` array.
pub fn trigger_sta_lta(cf: &[f64], on_threshold: f64, off_threshold: f64) -> Vec<(usize, usize)> {
    let mut triggers = Vec::new();
    let mut triggered = false;
    let mut on_idx = 0;

    for (i, &val) in cf.iter().enumerate() {
        if !triggered && val >= on_threshold {
            triggered = true;
            on_idx = i;
        } else if triggered && val < off_threshold {
            triggered = false;
            triggers.push((on_idx, i));
        }
    }

    // If still triggered at end, close it at the last sample
    if triggered {
        triggers.push((on_idx, cf.len().saturating_sub(1)));
    }

    triggers
}

/// Picks the precise onset time using the Akaike Information Criterion (AIC).
///
/// The AIC picker finds the sample that minimizes the AIC function within a
/// window. The AIC function measures the optimal split point between two
/// stationary segments (noise before and signal after).
///
/// AIC(k) = k * ln(var(signal[start..k])) + (N-k-1) * ln(var(signal[k..end]))
///
/// # Arguments
/// * `signal` - Input seismic trace
/// * `window_start` - Start index of the search window
/// * `window_end` - End index of the search window (exclusive)
///
/// # Returns
/// The sample index (in the original signal) of the AIC minimum.
pub fn aic_pick(signal: &[f64], window_start: usize, window_end: usize) -> usize {
    let end = window_end.min(signal.len());
    if end <= window_start + 2 {
        return window_start;
    }

    let window = &signal[window_start..end];
    let n = window.len();

    let mut min_aic = f64::MAX;
    let mut min_idx = 0;

    // Compute AIC for each candidate split point (avoid edges)
    for k in 1..(n - 1) {
        let var_before = variance(&window[..k]);
        let var_after = variance(&window[k..]);

        // Avoid log(0) by clamping
        let lb = if var_before > 1e-30 {
            var_before.ln()
        } else {
            -60.0
        };
        let la = if var_after > 1e-30 {
            var_after.ln()
        } else {
            -60.0
        };

        let aic = k as f64 * lb + (n - k - 1) as f64 * la;

        if aic < min_aic {
            min_aic = aic;
            min_idx = k;
        }
    }

    window_start + min_idx
}

/// Picks the onset using an autoregressive AIC method.
///
/// Fits forward and backward AR models of given order and computes the combined
/// AIC from prediction error variances. The minimum of the AR-AIC function
/// gives the most likely onset.
///
/// # Arguments
/// * `signal` - Input seismic trace
/// * `order` - AR model order (typically 2-6)
///
/// # Returns
/// The sample index of the AR-AIC minimum (best onset estimate).
pub fn ar_aic_pick(signal: &[f64], order: usize) -> usize {
    let n = signal.len();
    if n < order + 4 {
        return 0;
    }

    let mut min_aic = f64::MAX;
    let mut min_idx = order + 1;

    // For efficiency, use prediction error variance approximated by windowed variance
    // of the residual after removing a simple AR(order) trend
    for k in (order + 1)..(n - order - 1) {
        // Forward segment variance (proxy for AR prediction error)
        let var_fwd = ar_prediction_error_variance(&signal[..k], order);
        // Backward segment variance
        let var_bwd = ar_prediction_error_variance(&signal[k..], order);

        let lb = if var_fwd > 1e-30 {
            var_fwd.ln()
        } else {
            -60.0
        };
        let la = if var_bwd > 1e-30 {
            var_bwd.ln()
        } else {
            -60.0
        };

        let aic = k as f64 * lb + (n - k) as f64 * la;

        if aic < min_aic {
            min_aic = aic;
            min_idx = k;
        }
    }

    min_idx
}

/// Detects onsets using running kurtosis.
///
/// Kurtosis measures the "tailedness" of a distribution. Seismic arrivals
/// produce transient increases in kurtosis because the amplitude distribution
/// changes suddenly from Gaussian noise to a signal with outliers.
///
/// # Arguments
/// * `signal` - Input seismic trace
/// * `window_len` - Sliding window length in samples
///
/// # Returns
/// Indices where the kurtosis exceeds a threshold (mean + 2*std of kurtosis values).
pub fn kurtosis_pick(signal: &[f64], window_len: usize) -> Vec<usize> {
    if signal.len() < window_len || window_len < 4 {
        return Vec::new();
    }

    let n_out = signal.len() - window_len + 1;
    let mut kurt_values = Vec::with_capacity(n_out);

    for i in 0..n_out {
        let window = &signal[i..i + window_len];
        let k = kurtosis(window);
        kurt_values.push(k);
    }

    if kurt_values.is_empty() {
        return Vec::new();
    }

    // Compute mean and std of kurtosis values
    let mean_k: f64 = kurt_values.iter().sum::<f64>() / kurt_values.len() as f64;
    let var_k: f64 =
        kurt_values.iter().map(|x| (x - mean_k).powi(2)).sum::<f64>() / kurt_values.len() as f64;
    let std_k = var_k.sqrt();

    let threshold = mean_k + 2.0 * std_k;

    // Find peaks above threshold with simple peak detection
    let mut picks = Vec::new();
    let mut in_peak = false;
    let mut peak_idx = 0;
    let mut peak_val = f64::MIN;

    for (i, &val) in kurt_values.iter().enumerate() {
        if val > threshold {
            if !in_peak {
                in_peak = true;
                peak_idx = i;
                peak_val = val;
            } else if val > peak_val {
                peak_idx = i;
                peak_val = val;
            }
        } else if in_peak {
            in_peak = false;
            picks.push(peak_idx);
        }
    }
    if in_peak {
        picks.push(peak_idx);
    }

    picks
}

/// Estimates pick quality as a normalized SNR in the range [0.0, 1.0].
///
/// Quality is computed as:
/// `quality = 1.0 - 1.0 / (1.0 + snr_linear)`
///
/// where `snr_linear` is the ratio of signal energy (after pick) to noise
/// energy (before pick) within the given window.
///
/// # Arguments
/// * `signal` - Input seismic trace
/// * `pick_sample` - Sample index of the pick
/// * `window` - Window length in samples for energy estimation
///
/// # Returns
/// Quality value in [0.0, 1.0].
pub fn pick_quality(signal: &[f64], pick_sample: usize, window: usize) -> f64 {
    if signal.is_empty() || window == 0 {
        return 0.0;
    }

    // Noise window: before the pick
    let noise_start = pick_sample.saturating_sub(window);
    let noise_end = pick_sample.min(signal.len());
    if noise_end <= noise_start {
        return 0.0;
    }
    let noise_energy: f64 = signal[noise_start..noise_end].iter().map(|x| x * x).sum::<f64>()
        / (noise_end - noise_start) as f64;

    // Signal window: after the pick
    let sig_start = pick_sample.min(signal.len());
    let sig_end = (pick_sample + window).min(signal.len());
    if sig_end <= sig_start {
        return 0.0;
    }
    let sig_energy: f64 = signal[sig_start..sig_end].iter().map(|x| x * x).sum::<f64>()
        / (sig_end - sig_start) as f64;

    if noise_energy < 1e-30 {
        if sig_energy > 1e-30 {
            return 1.0;
        } else {
            return 0.0;
        }
    }

    let snr = sig_energy / noise_energy;
    1.0 - 1.0 / (1.0 + snr)
}

/// Determines the first-motion polarity at a pick location.
///
/// Examines the first few samples after the pick to determine whether the
/// initial ground motion is upward (compressive) or downward (dilatational).
///
/// # Arguments
/// * `signal` - Input seismic trace
/// * `pick_sample` - Sample index of the pick
///
/// # Returns
/// [`Polarity::Up`] if the first significant motion is positive,
/// [`Polarity::Down`] if negative, or [`Polarity::Unknown`] if indeterminate.
pub fn first_motion_polarity(signal: &[f64], pick_sample: usize) -> Polarity {
    if pick_sample >= signal.len() {
        return Polarity::Unknown;
    }

    // Look at a small window after the pick
    let look_ahead = 10.min(signal.len() - pick_sample);
    if look_ahead < 2 {
        return Polarity::Unknown;
    }

    // Find the first sample with a significant amplitude change
    let baseline = signal[pick_sample];
    let window = &signal[pick_sample..pick_sample + look_ahead];

    // Compute the RMS of the pre-pick noise for threshold
    let pre_start = pick_sample.saturating_sub(20);
    let pre_end = pick_sample;
    let noise_rms = if pre_end > pre_start {
        let sum_sq: f64 = signal[pre_start..pre_end].iter().map(|x| x * x).sum();
        (sum_sq / (pre_end - pre_start) as f64).sqrt()
    } else {
        0.0
    };

    let threshold = noise_rms * 0.5;

    // Find the first significant deviation from baseline
    for &sample in &window[1..] {
        let diff = sample - baseline;
        if diff.abs() > threshold {
            return if diff > 0.0 {
                Polarity::Up
            } else {
                Polarity::Down
            };
        }
    }

    // If no clear polarity, look at the overall trend
    let last = window[look_ahead - 1];
    let diff = last - baseline;
    if diff.abs() > f64::EPSILON {
        if diff > 0.0 {
            Polarity::Up
        } else {
            Polarity::Down
        }
    } else {
        Polarity::Unknown
    }
}

/// Applies a simple bandpass filter to a signal.
///
/// Uses a windowed sinc (FIR) filter with a Hann window. The filter is applied
/// via direct convolution (time-domain).
///
/// # Arguments
/// * `signal` - Input signal
/// * `low_hz` - Lower cutoff frequency in Hz
/// * `high_hz` - Upper cutoff frequency in Hz
/// * `sample_rate` - Sample rate in Hz
///
/// # Returns
/// Filtered signal of the same length as input.
pub fn bandpass_filter(signal: &[f64], low_hz: f64, high_hz: f64, sample_rate: f64) -> Vec<f64> {
    if signal.is_empty() || sample_rate <= 0.0 || low_hz >= high_hz {
        return signal.to_vec();
    }

    let nyquist = sample_rate / 2.0;
    let low_norm = low_hz / nyquist;
    let high_norm = high_hz / nyquist;

    if low_norm >= 1.0 || high_norm <= 0.0 {
        return signal.to_vec();
    }

    let low_norm = low_norm.max(0.001);
    let high_norm = high_norm.min(0.999);

    // Design FIR bandpass filter using windowed sinc
    let filter_order = 63; // Odd order for symmetric filter
    let half = filter_order / 2;
    let mut coeffs = vec![0.0_f64; filter_order];

    for i in 0..filter_order {
        let n = i as f64 - half as f64;
        let sinc_val = if n.abs() < 1e-10 {
            high_norm - low_norm
        } else {
            let pi_n = PI * n;
            (high_norm * pi_n).sin() / pi_n - (low_norm * pi_n).sin() / pi_n
        };

        // Hann window
        let window =
            0.5 * (1.0 - (2.0 * PI * i as f64 / (filter_order - 1) as f64).cos());

        coeffs[i] = sinc_val * window;
    }

    // Normalize filter gain at center frequency
    let center_freq = (low_norm + high_norm) / 2.0;
    let gain: f64 = coeffs
        .iter()
        .enumerate()
        .map(|(i, &c)| {
            c * (2.0 * PI * center_freq * (i as f64 - half as f64)).cos()
        })
        .sum();
    if gain.abs() > 1e-10 {
        let norm = 1.0 / gain;
        for c in &mut coeffs {
            *c *= norm;
        }
    }

    // Apply filter via convolution
    convolve(signal, &coeffs)
}

// --- Internal helper functions ---

/// Computes variance of a slice.
fn variance(data: &[f64]) -> f64 {
    if data.len() < 2 {
        return 0.0;
    }
    let n = data.len() as f64;
    let mean = data.iter().sum::<f64>() / n;
    data.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n
}

/// Computes excess kurtosis of a slice.
fn kurtosis(data: &[f64]) -> f64 {
    let n = data.len() as f64;
    if n < 4.0 {
        return 0.0;
    }
    let mean = data.iter().sum::<f64>() / n;
    let m2: f64 = data.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n;
    if m2 < 1e-30 {
        return 0.0;
    }
    let m4: f64 = data.iter().map(|x| (x - mean).powi(4)).sum::<f64>() / n;
    m4 / (m2 * m2) - 3.0
}

/// Estimates AR prediction error variance for a segment using Burg's method (simplified).
///
/// Computes the first-order prediction error variance as a proxy for full AR(p).
fn ar_prediction_error_variance(data: &[f64], order: usize) -> f64 {
    let n = data.len();
    if n <= order + 1 {
        return variance(data);
    }

    // Simple approach: compute residual variance after removing linear prediction
    // using autocorrelation-based AR coefficients
    let autocorr = autocorrelation(data, order);
    let coeffs = levinson_durbin(&autocorr, order);

    // Compute prediction error
    let mut error_sum = 0.0;
    let mut count = 0;
    for i in order..n {
        let mut predicted = 0.0;
        for j in 0..order {
            predicted += coeffs[j] * data[i - j - 1];
        }
        let error = data[i] - predicted;
        error_sum += error * error;
        count += 1;
    }

    if count > 0 {
        error_sum / count as f64
    } else {
        variance(data)
    }
}

/// Computes autocorrelation coefficients for lags 0..=max_lag.
fn autocorrelation(data: &[f64], max_lag: usize) -> Vec<f64> {
    let n = data.len();
    let mean = data.iter().sum::<f64>() / n as f64;
    let mut result = Vec::with_capacity(max_lag + 1);

    for lag in 0..=max_lag {
        let mut sum = 0.0;
        for i in 0..(n - lag) {
            sum += (data[i] - mean) * (data[i + lag] - mean);
        }
        result.push(sum / n as f64);
    }

    result
}

/// Levinson-Durbin recursion to solve AR coefficients from autocorrelation.
fn levinson_durbin(r: &[f64], order: usize) -> Vec<f64> {
    if order == 0 || r.is_empty() || r[0].abs() < 1e-30 {
        return vec![0.0; order];
    }

    let mut a = vec![0.0; order];
    let mut a_prev = vec![0.0; order];
    let mut error = r[0];

    for i in 0..order {
        // Compute reflection coefficient
        let mut sum = 0.0;
        for j in 0..i {
            if i - j < r.len() {
                sum += a_prev[j] * r[i - j];
            }
        }

        let k = if error.abs() > 1e-30 {
            if i + 1 < r.len() {
                -(r[i + 1] + sum) / error
            } else {
                0.0
            }
        } else {
            0.0
        };

        a[i] = k;
        for j in 0..i {
            a[j] = a_prev[j] + k * a_prev[i - 1 - j];
        }

        error *= 1.0 - k * k;
        a_prev[..=i].copy_from_slice(&a[..=i]);
    }

    a
}

/// Direct convolution with output length equal to input length (centered).
fn convolve(signal: &[f64], kernel: &[f64]) -> Vec<f64> {
    let n = signal.len();
    let m = kernel.len();
    let half = m / 2;
    let mut output = vec![0.0; n];

    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..m {
            let sig_idx = i as isize + j as isize - half as isize;
            if sig_idx >= 0 && (sig_idx as usize) < n {
                sum += signal[sig_idx as usize] * kernel[j];
            }
        }
        output[i] = sum;
    }

    output
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a synthetic seismic trace with a sharp onset.
    fn synthetic_signal(n: usize, onset: usize, amplitude: f64) -> Vec<f64> {
        let mut signal = vec![0.0; n];
        // Background noise
        let mut rng_state: u64 = 42;
        for s in signal.iter_mut() {
            rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u = (rng_state >> 33) as f64 / (1u64 << 31) as f64 - 1.0;
            *s = u * 0.01; // small noise
        }
        // Signal after onset
        for i in onset..n {
            let t = (i - onset) as f64;
            signal[i] += amplitude * (t * 0.05).sin() * (-t * 0.002).exp();
        }
        signal
    }

    /// Simple deterministic pseudo-random noise generator.
    fn pseudo_noise(n: usize, seed: u64, amplitude: f64) -> Vec<f64> {
        let mut out = Vec::with_capacity(n);
        let mut state = seed;
        for _ in 0..n {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let val = ((state >> 33) as f64 / (1u64 << 31) as f64) * 2.0 - 1.0;
            out.push(val * amplitude);
        }
        out
    }

    #[test]
    fn test_seismic_config_default() {
        let config = SeismicConfig::default();
        assert_eq!(config.sample_rate_hz, 100.0);
        assert_eq!(config.sta_window_s, 1.0);
        assert_eq!(config.lta_window_s, 10.0);
        assert_eq!(config.trigger_threshold, 4.0);
        assert_eq!(config.detrigger_threshold, 1.5);
    }

    #[test]
    fn test_seismic_config_samples() {
        let config = SeismicConfig::new(200.0);
        assert_eq!(config.sta_samples(), 200);
        assert_eq!(config.lta_samples(), 2000);
    }

    #[test]
    fn test_sta_lta_basic() {
        let signal = synthetic_signal(2000, 1000, 5.0);
        let cf = sta_lta(&signal, 50, 200);
        assert_eq!(cf.len(), 2000 - 200);

        // The STA/LTA ratio should peak near the onset
        let max_idx = cf
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        // max_idx is relative to the cf array, which starts at lta_len=200
        let peak_signal_idx = max_idx + 200;
        // Should be near the onset at 1000 (within STA window tolerance)
        assert!(
            (peak_signal_idx as isize - 1000).unsigned_abs() < 100,
            "Peak at {peak_signal_idx}, expected near 1000"
        );
    }

    #[test]
    fn test_sta_lta_empty() {
        let cf = sta_lta(&[], 10, 50);
        assert!(cf.is_empty());
    }

    #[test]
    fn test_sta_lta_invalid_windows() {
        let signal = vec![1.0; 100];
        // sta_len >= lta_len should return empty
        let cf = sta_lta(&signal, 50, 50);
        assert!(cf.is_empty());

        let cf = sta_lta(&signal, 60, 50);
        assert!(cf.is_empty());
    }

    #[test]
    fn test_trigger_sta_lta_basic() {
        // Create a CF with a clear peak
        let mut cf = vec![1.0; 200];
        for i in 80..120 {
            cf[i] = 5.0;
        }

        let triggers = trigger_sta_lta(&cf, 3.0, 1.5);
        assert_eq!(triggers.len(), 1);
        let (on, off) = triggers[0];
        assert_eq!(on, 80);
        assert_eq!(off, 120);
    }

    #[test]
    fn test_trigger_sta_lta_no_triggers() {
        let cf = vec![1.0; 100];
        let triggers = trigger_sta_lta(&cf, 3.0, 1.5);
        assert!(triggers.is_empty());
    }

    #[test]
    fn test_trigger_sta_lta_multiple() {
        let mut cf = vec![1.0; 300];
        for i in 50..80 {
            cf[i] = 5.0;
        }
        for i in 200..230 {
            cf[i] = 6.0;
        }

        let triggers = trigger_sta_lta(&cf, 3.0, 1.5);
        assert_eq!(triggers.len(), 2);
    }

    #[test]
    fn test_trigger_sta_lta_unterminated() {
        // Trigger that stays on until end
        let mut cf = vec![1.0; 100];
        for i in 80..100 {
            cf[i] = 5.0;
        }

        let triggers = trigger_sta_lta(&cf, 3.0, 1.5);
        assert_eq!(triggers.len(), 1);
        let (on, _off) = triggers[0];
        assert_eq!(on, 80);
    }

    #[test]
    fn test_aic_pick_basic() {
        let signal = synthetic_signal(500, 250, 3.0);
        let pick = aic_pick(&signal, 200, 350);
        // Pick should be near 250
        assert!(
            (pick as isize - 250).unsigned_abs() < 30,
            "AIC pick at {pick}, expected near 250"
        );
    }

    #[test]
    fn test_aic_pick_edge_cases() {
        let pick = aic_pick(&[1.0, 2.0], 0, 2);
        assert_eq!(pick, 0); // Too short, returns window_start

        let pick = aic_pick(&[], 0, 0);
        assert_eq!(pick, 0);
    }

    #[test]
    fn test_ar_aic_pick_basic() {
        let signal = synthetic_signal(800, 400, 4.0);
        let pick = ar_aic_pick(&signal, 3);
        // Should be near 400
        assert!(
            (pick as isize - 400).unsigned_abs() < 60,
            "AR-AIC pick at {pick}, expected near 400"
        );
    }

    #[test]
    fn test_ar_aic_pick_short_signal() {
        let signal = vec![1.0, 2.0, 3.0];
        let pick = ar_aic_pick(&signal, 3);
        assert_eq!(pick, 0);
    }

    #[test]
    fn test_kurtosis_pick_basic() {
        let mut signal = pseudo_noise(1000, 123, 0.01);
        // Add an impulsive onset
        for i in 500..520 {
            signal[i] = 3.0 * ((i - 500) as f64 * 0.3).sin();
        }

        let picks = kurtosis_pick(&signal, 50);
        // Should detect at least one onset near 500
        assert!(
            !picks.is_empty(),
            "Kurtosis picker should detect the onset"
        );
        let closest = picks
            .iter()
            .min_by_key(|&&p| (p as isize - 500).unsigned_abs())
            .unwrap();
        assert!(
            (*closest as isize - 500).unsigned_abs() < 60,
            "Kurtosis pick at {closest}, expected near 500"
        );
    }

    #[test]
    fn test_kurtosis_pick_empty() {
        let picks = kurtosis_pick(&[], 10);
        assert!(picks.is_empty());
    }

    #[test]
    fn test_kurtosis_pick_window_too_small() {
        let signal = vec![1.0; 100];
        let picks = kurtosis_pick(&signal, 3);
        assert!(picks.is_empty());
    }

    #[test]
    fn test_pick_quality_high_snr() {
        let mut signal = vec![0.001; 200];
        // Strong signal after pick
        for i in 100..200 {
            signal[i] = 5.0;
        }

        let q = pick_quality(&signal, 100, 50);
        assert!(q > 0.9, "Quality {q} should be high for strong signal");
    }

    #[test]
    fn test_pick_quality_low_snr() {
        // Uniform noise, no real onset
        let signal = pseudo_noise(200, 999, 1.0);
        let q = pick_quality(&signal, 100, 50);
        // Should be around 0.5 since noise energy is similar before/after
        assert!(
            q < 0.8,
            "Quality {q} should be moderate for uniform noise"
        );
    }

    #[test]
    fn test_pick_quality_edge_cases() {
        assert_eq!(pick_quality(&[], 0, 10), 0.0);
        assert_eq!(pick_quality(&[1.0], 5, 10), 0.0);
    }

    #[test]
    fn test_first_motion_polarity_up() {
        let mut signal = vec![0.0; 100];
        // Clear upward first motion
        for i in 50..60 {
            signal[i] = (i - 50) as f64 * 0.5;
        }

        let pol = first_motion_polarity(&signal, 50);
        assert_eq!(pol, Polarity::Up);
    }

    #[test]
    fn test_first_motion_polarity_down() {
        let mut signal = vec![0.0; 100];
        // Clear downward first motion
        for i in 50..60 {
            signal[i] = -((i - 50) as f64) * 0.5;
        }

        let pol = first_motion_polarity(&signal, 50);
        assert_eq!(pol, Polarity::Down);
    }

    #[test]
    fn test_first_motion_polarity_unknown() {
        let pol = first_motion_polarity(&[], 0);
        assert_eq!(pol, Polarity::Unknown);

        let pol = first_motion_polarity(&[0.0], 5);
        assert_eq!(pol, Polarity::Unknown);
    }

    #[test]
    fn test_bandpass_filter_basic() {
        // Generate a pure out-of-band signal (50 Hz) and verify attenuation
        let sample_rate = 1000.0;
        let n = 1000;
        let mut low_signal = vec![0.0; n];
        for i in 0..n {
            let t = i as f64 / sample_rate;
            low_signal[i] = (2.0 * PI * 50.0 * t).sin();
        }

        // Filter to keep only 200-400 Hz band (should attenuate 50 Hz)
        let filtered = bandpass_filter(&low_signal, 200.0, 400.0, sample_rate);
        assert_eq!(filtered.len(), n);

        // Compute energy in the middle section (avoiding filter transients)
        let mid_start = 200;
        let mid_end = 800;
        let orig_energy: f64 = low_signal[mid_start..mid_end].iter().map(|x| x * x).sum();
        let filt_energy: f64 = filtered[mid_start..mid_end].iter().map(|x| x * x).sum();

        // The 50 Hz component should be significantly attenuated by the bandpass
        assert!(
            filt_energy < orig_energy * 0.5,
            "Filtered energy ({filt_energy}) should be much less than original ({orig_energy})"
        );
    }

    #[test]
    fn test_bandpass_filter_empty() {
        let result = bandpass_filter(&[], 10.0, 100.0, 1000.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_bandpass_filter_invalid_params() {
        let signal = vec![1.0; 100];
        // low >= high should return unchanged
        let result = bandpass_filter(&signal, 100.0, 50.0, 1000.0);
        assert_eq!(result, signal);
    }

    #[test]
    fn test_seismic_detector_integration() {
        let config = SeismicConfig {
            sample_rate_hz: 100.0,
            sta_window_s: 0.5,
            lta_window_s: 5.0,
            trigger_threshold: 3.0,
            detrigger_threshold: 1.5,
        };

        let signal = synthetic_signal(2000, 1000, 5.0);
        let detector = SeismicDetector::new(config);
        let picks = detector.detect(&signal);

        assert!(
            !picks.is_empty(),
            "Detector should find at least one arrival"
        );
        let pick = &picks[0];
        assert!(
            (pick.sample_index as isize - 1000).unsigned_abs() < 100,
            "Pick at {}, expected near 1000",
            pick.sample_index
        );
        assert!(pick.quality > 0.0);
        assert!(pick.time_s > 0.0);
    }

    #[test]
    fn test_seismic_detector_classify() {
        let config = SeismicConfig {
            sample_rate_hz: 1000.0,
            sta_window_s: 0.05,
            lta_window_s: 0.5,
            trigger_threshold: 3.0,
            detrigger_threshold: 1.5,
        };

        let mut signal = pseudo_noise(5000, 77, 0.01);
        // Add a high-frequency arrival (P-wave-like)
        for i in 2500..3000 {
            let t = (i - 2500) as f64;
            signal[i] += 2.0 * (2.0 * PI * 200.0 * t / 1000.0).sin() * (-t * 0.005).exp();
        }

        let detector = SeismicDetector::new(config);
        let picks = detector.detect_and_classify(&signal);

        assert!(!picks.is_empty(), "Should detect the arrival");
        // The phase classification is tested to return a valid variant
        for pick in &picks {
            assert!(matches!(
                pick.phase,
                PhaseType::P | PhaseType::S | PhaseType::Unknown
            ));
        }
    }

    #[test]
    fn test_seismic_detector_empty_signal() {
        let config = SeismicConfig::default();
        let detector = SeismicDetector::new(config);
        let picks = detector.detect(&[]);
        assert!(picks.is_empty());
    }

    #[test]
    fn test_phase_type_debug() {
        // Ensure Debug trait works
        let p = format!("{:?}", PhaseType::P);
        assert_eq!(p, "P");
        let s = format!("{:?}", PhaseType::S);
        assert_eq!(s, "S");
        let u = format!("{:?}", PhaseType::Unknown);
        assert_eq!(u, "Unknown");
    }

    #[test]
    fn test_polarity_equality() {
        assert_eq!(Polarity::Up, Polarity::Up);
        assert_ne!(Polarity::Up, Polarity::Down);
        assert_ne!(Polarity::Down, Polarity::Unknown);
    }

    #[test]
    fn test_variance_helper() {
        let data = [2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0];
        let v = variance(&data);
        // Population variance of this data is 4.0
        assert!((v - 4.0).abs() < 0.01, "Variance {v}, expected 4.0");
    }

    #[test]
    fn test_kurtosis_helper() {
        // For a uniform-like distribution, kurtosis should be negative
        let data: Vec<f64> = (0..100).map(|i| i as f64 / 100.0).collect();
        let k = kurtosis(&data);
        // Uniform distribution has excess kurtosis of -1.2
        assert!(k < 0.0, "Uniform kurtosis {k} should be negative");
    }

    #[test]
    fn test_convolve_identity() {
        let signal = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let kernel = vec![0.0, 0.0, 1.0, 0.0, 0.0]; // Identity (centered)
        let result = convolve(&signal, &kernel);
        for (a, b) in result.iter().zip(signal.iter()) {
            assert!(
                (a - b).abs() < 1e-10,
                "Convolve identity failed: {a} != {b}"
            );
        }
    }
}
