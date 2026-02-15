// trace:FR-EEG-PROC | ai:claude
//! # Electroencephalogram (EEG) Signal Processor
//!
//! Low-level EEG signal processing primitives for brain-computer interfaces,
//! clinical neuroscience, and cognitive research.
//!
//! This module provides:
//! - **Artifact removal**: Eye blink, muscle artifact, flatline detection
//! - **Spectral band power extraction**: Delta, Theta, Alpha, Beta, Gamma
//! - **Event-Related Potential (ERP) analysis**: Epoch extraction, averaging, peak detection
//! - **Inter-channel coherence**: Magnitude squared coherence, phase locking value
//!
//! ## Overview
//!
//! EEG records electrical activity from the scalp using multiple electrodes.
//! Raw signals require extensive preprocessing before analysis:
//!
//! 1. Re-referencing (common average, linked ears, bipolar, Laplacian)
//! 2. Artifact rejection (eye blinks, muscle activity, flatlines)
//! 3. Notch filtering (50/60 Hz powerline removal)
//! 4. Bandpass filtering for frequency bands of interest
//! 5. Spectral analysis (PSD, band powers)
//! 6. ERP extraction and averaging
//! 7. Connectivity analysis (coherence, phase locking)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::electroencephalogram_processor::*;
//!
//! let config = EegConfig {
//!     sample_rate_hz: 256.0,
//!     num_channels: 16,
//!     channel_names: (0..16).map(|i| format!("Ch{}", i)).collect(),
//!     reference: ReferenceScheme::CommonAverage,
//!     notch_frequency_hz: 60.0,
//! };
//!
//! let processor = EegProcessor::new(config);
//!
//! // Compute band powers for a signal
//! let signal: Vec<f64> = (0..512)
//!     .map(|i| (2.0 * std::f64::consts::PI * 10.0 * i as f64 / 256.0).sin())
//!     .collect();
//! let powers = processor.all_band_powers(&signal, 256.0);
//! assert!(powers.alpha > powers.delta);
//! ```

use std::f64::consts::PI;

// ─── Enums ───────────────────────────────────────────────────────────────────

/// Re-referencing scheme for multi-channel EEG.
///
/// Re-referencing removes common-mode noise and changes the spatial
/// sensitivity of each electrode.
#[derive(Debug, Clone, PartialEq)]
pub enum ReferenceScheme {
    /// Subtract the mean of all channels from each channel.
    /// Widely used in clinical and research settings.
    CommonAverage,
    /// Average of two ear/mastoid electrodes used as reference.
    /// Indices refer to the left and right ear channels.
    LinkedEars { left_idx: usize, right_idx: usize },
    /// Difference between adjacent electrode pairs.
    /// Each output channel = channel[i] - channel[i+1].
    Bipolar,
    /// Surface Laplacian: each channel minus mean of its neighbors.
    /// `neighbors[i]` lists neighbor channel indices for channel i.
    Laplacian { neighbors: Vec<Vec<usize>> },
}

/// Standard EEG frequency bands with approximate ranges.
///
/// Each band correlates with different cognitive states:
/// - Delta: deep sleep
/// - Theta: drowsiness, meditation
/// - Alpha: relaxed wakefulness, eyes closed
/// - Beta: active thinking, focus
/// - Gamma: higher cognitive functions, perception
#[derive(Debug, Clone, PartialEq)]
pub enum FrequencyBand {
    /// 0.5 - 4 Hz. Associated with deep sleep.
    Delta,
    /// 4 - 8 Hz. Associated with drowsiness and meditation.
    Theta,
    /// 8 - 13 Hz. Associated with relaxed wakefulness.
    Alpha,
    /// 13 - 30 Hz. Associated with active thinking and focus.
    Beta,
    /// 30 - 100 Hz. Associated with higher cognitive functions.
    Gamma,
    /// User-defined frequency range.
    Custom { low_hz: f64, high_hz: f64 },
}

impl FrequencyBand {
    /// Returns the (low, high) frequency range in Hz for this band.
    pub fn range_hz(&self) -> (f64, f64) {
        match self {
            FrequencyBand::Delta => (0.5, 4.0),
            FrequencyBand::Theta => (4.0, 8.0),
            FrequencyBand::Alpha => (8.0, 13.0),
            FrequencyBand::Beta => (13.0, 30.0),
            FrequencyBand::Gamma => (30.0, 100.0),
            FrequencyBand::Custom { low_hz, high_hz } => (*low_hz, *high_hz),
        }
    }
}

/// Polarity for ERP peak detection.
#[derive(Debug, Clone, PartialEq)]
pub enum Polarity {
    /// Search for positive (upward) peaks.
    Positive,
    /// Search for negative (downward) peaks.
    Negative,
}

// ─── Configuration ───────────────────────────────────────────────────────────

/// Configuration for EEG signal processing.
///
/// Typical setups:
/// - Clinical: 256 Hz, 19 channels (10-20 system), 60 Hz notch (US)
/// - Research: 512 Hz, 64 channels (10-10 system), 50 Hz notch (EU)
/// - BCI: 250 Hz, 8-16 channels, application-specific bands
#[derive(Debug, Clone)]
pub struct EegConfig {
    /// Sampling rate in Hz. Typical values: 250, 256, 512, 1000.
    pub sample_rate_hz: f64,
    /// Number of EEG channels.
    pub num_channels: usize,
    /// Human-readable channel names (e.g., "Fp1", "Fp2", "C3", "C4").
    pub channel_names: Vec<String>,
    /// Re-referencing scheme to apply.
    pub reference: ReferenceScheme,
    /// Powerline interference frequency (50 Hz in EU, 60 Hz in US).
    pub notch_frequency_hz: f64,
}

// ─── Band Powers ─────────────────────────────────────────────────────────────

/// Spectral power in each standard EEG frequency band.
///
/// Power values are in arbitrary units (signal amplitude squared).
/// Use `relative_power()` for normalized comparisons.
#[derive(Debug, Clone)]
pub struct BandPowers {
    /// Delta band power (0.5-4 Hz).
    pub delta: f64,
    /// Theta band power (4-8 Hz).
    pub theta: f64,
    /// Alpha band power (8-13 Hz).
    pub alpha: f64,
    /// Beta band power (13-30 Hz).
    pub beta: f64,
    /// Gamma band power (30-100 Hz).
    pub gamma: f64,
    /// Total power across all bands.
    pub total: f64,
}

impl BandPowers {
    /// Compute relative power for a given band (band / total).
    ///
    /// Returns a value between 0.0 and 1.0 representing the fraction
    /// of total power contained in the specified band.
    pub fn relative_power(&self, band: &FrequencyBand) -> f64 {
        if self.total <= 0.0 {
            return 0.0;
        }
        let power = match band {
            FrequencyBand::Delta => self.delta,
            FrequencyBand::Theta => self.theta,
            FrequencyBand::Alpha => self.alpha,
            FrequencyBand::Beta => self.beta,
            FrequencyBand::Gamma => self.gamma,
            FrequencyBand::Custom { .. } => 0.0,
        };
        power / self.total
    }

    /// Alpha / Theta ratio. Higher values indicate greater alertness.
    ///
    /// Used in neurofeedback for attention/relaxation assessment.
    pub fn alpha_theta_ratio(&self) -> f64 {
        if self.theta <= 0.0 {
            return 0.0;
        }
        self.alpha / self.theta
    }

    /// Theta / Beta ratio. Higher values may indicate reduced attention.
    ///
    /// Commonly used in ADHD research and neurofeedback.
    pub fn theta_beta_ratio(&self) -> f64 {
        if self.beta <= 0.0 {
            return 0.0;
        }
        self.theta / self.beta
    }
}

// ─── EEG Processor ───────────────────────────────────────────────────────────

/// Core EEG signal processor.
///
/// Provides filtering, spectral analysis, and re-referencing operations.
pub struct EegProcessor {
    /// Configuration parameters.
    pub config: EegConfig,
}

impl EegProcessor {
    /// Create a new EEG processor with the given configuration.
    pub fn new(config: EegConfig) -> Self {
        Self { config }
    }

    /// Apply a re-referencing scheme to multi-channel EEG data.
    ///
    /// Each inner `Vec<f64>` represents one channel's time series.
    ///
    /// # Re-referencing schemes
    ///
    /// - **CommonAverage**: Subtract the mean across all channels at each time point.
    /// - **LinkedEars**: Subtract the average of left and right ear electrodes.
    /// - **Bipolar**: Each channel becomes the difference with the next channel.
    /// - **Laplacian**: Each channel minus mean of its specified neighbors.
    pub fn re_reference(channels: &mut Vec<Vec<f64>>, scheme: &ReferenceScheme) {
        if channels.is_empty() {
            return;
        }
        let num_channels = channels.len();
        let num_samples = channels[0].len();

        match scheme {
            ReferenceScheme::CommonAverage => {
                for s in 0..num_samples {
                    let mean: f64 =
                        channels.iter().map(|ch| ch[s]).sum::<f64>() / num_channels as f64;
                    for ch in channels.iter_mut() {
                        ch[s] -= mean;
                    }
                }
            }
            ReferenceScheme::LinkedEars {
                left_idx,
                right_idx,
            } => {
                for s in 0..num_samples {
                    let ear_avg = (channels[*left_idx][s] + channels[*right_idx][s]) / 2.0;
                    for ch in channels.iter_mut() {
                        ch[s] -= ear_avg;
                    }
                }
            }
            ReferenceScheme::Bipolar => {
                // Bipolar: ch[i] = ch[i] - ch[i+1]. Last channel is zeroed.
                for s in 0..num_samples {
                    let vals: Vec<f64> = channels.iter().map(|ch| ch[s]).collect();
                    for i in 0..num_channels - 1 {
                        channels[i][s] = vals[i] - vals[i + 1];
                    }
                    channels[num_channels - 1][s] = 0.0;
                }
            }
            ReferenceScheme::Laplacian { neighbors } => {
                for s in 0..num_samples {
                    let vals: Vec<f64> = channels.iter().map(|ch| ch[s]).collect();
                    for (i, nbrs) in neighbors.iter().enumerate() {
                        if i < num_channels && !nbrs.is_empty() {
                            let neighbor_mean: f64 =
                                nbrs.iter().map(|&n| vals[n]).sum::<f64>() / nbrs.len() as f64;
                            channels[i][s] = vals[i] - neighbor_mean;
                        }
                    }
                }
            }
        }
    }

    /// Apply an IIR notch filter to remove powerline interference.
    ///
    /// Uses a second-order IIR notch (band-reject) filter designed via
    /// pole-zero placement. The Q factor controls the notch bandwidth:
    /// - Higher Q = narrower notch (more selective)
    /// - Typical Q for powerline: 30-50
    ///
    /// # Arguments
    /// - `signal`: Mutable signal to filter in-place
    /// - `freq_hz`: Center frequency to remove (e.g., 50.0 or 60.0)
    /// - `q_factor`: Quality factor (bandwidth = freq_hz / q_factor)
    /// - `sample_rate`: Sampling rate in Hz
    pub fn notch_filter(signal: &mut Vec<f64>, freq_hz: f64, q_factor: f64, sample_rate: f64) {
        if signal.len() < 3 || sample_rate <= 0.0 || q_factor <= 0.0 {
            return;
        }

        // Design second-order IIR notch filter via pole-zero placement
        let w0 = 2.0 * PI * freq_hz / sample_rate;
        let bw = w0 / q_factor;
        let r = 1.0 - bw / 2.0; // pole radius
        let r = r.max(0.0).min(0.999); // clamp for stability

        // Transfer function: H(z) = (1 - 2cos(w0)z^-1 + z^-2) / (1 - 2r*cos(w0)z^-1 + r^2*z^-2)
        let cos_w0 = w0.cos();

        // Numerator coefficients (zeros on unit circle at w0)
        let b0 = 1.0;
        let b1 = -2.0 * cos_w0;
        let b2 = 1.0;

        // Denominator coefficients (poles inside unit circle)
        let a1 = -2.0 * r * cos_w0;
        let a2 = r * r;

        // Normalize gain at DC
        let gain_dc = (b0 + b1 + b2) / (1.0 + a1 + a2);
        let norm = if gain_dc.abs() > 1e-10 {
            1.0 / gain_dc
        } else {
            1.0
        };

        let b0 = b0 * norm;
        let b1 = b1 * norm;
        let b2 = b2 * norm;

        // Apply Direct Form II transposed
        let mut w1 = 0.0_f64;
        let mut w2 = 0.0_f64;

        for sample in signal.iter_mut() {
            let x = *sample;
            let y = b0 * x + w1;
            w1 = b1 * x - a1 * y + w2;
            w2 = b2 * x - a2 * y;
            *sample = y;
        }
    }

    /// Apply a FIR bandpass filter using windowed-sinc design.
    ///
    /// Constructs a bandpass filter by subtracting a lowpass at `low_hz`
    /// from a lowpass at `high_hz`, using a Hamming window.
    ///
    /// # Arguments
    /// - `signal`: Input time series
    /// - `low_hz`: Lower cutoff frequency
    /// - `high_hz`: Upper cutoff frequency
    /// - `sample_rate`: Sampling rate in Hz
    /// - `order`: Filter order (higher = sharper cutoff, more delay)
    ///
    /// # Returns
    /// Filtered signal (same length as input, zero-padded at edges).
    pub fn bandpass_filter(
        signal: &[f64],
        low_hz: f64,
        high_hz: f64,
        sample_rate: f64,
        order: usize,
    ) -> Vec<f64> {
        if signal.is_empty() || sample_rate <= 0.0 || order == 0 {
            return signal.to_vec();
        }

        let n = order | 1; // ensure odd for symmetric FIR
        let m = n / 2;

        let fc_low = low_hz / sample_rate;
        let fc_high = high_hz / sample_rate;

        // Design bandpass kernel = highpass(low) convolved as lowpass(high) - lowpass(low)
        let mut kernel = vec![0.0; n];
        for i in 0..n {
            let k = i as f64 - m as f64;
            // Hamming window
            let w = 0.54 - 0.46 * (2.0 * PI * i as f64 / (n - 1) as f64).cos();

            let sinc_high = if k.abs() < 1e-10 {
                2.0 * fc_high
            } else {
                (2.0 * PI * fc_high * k).sin() / (PI * k)
            };

            let sinc_low = if k.abs() < 1e-10 {
                2.0 * fc_low
            } else {
                (2.0 * PI * fc_low * k).sin() / (PI * k)
            };

            kernel[i] = (sinc_high - sinc_low) * w;
        }

        // Normalize kernel for unity passband gain
        let sum: f64 = kernel.iter().sum();
        if sum.abs() > 1e-10 {
            // For bandpass, normalize by the magnitude response at center frequency
            let fc_center = (fc_low + fc_high) / 2.0;
            let mut mag_re = 0.0;
            let mut mag_im = 0.0;
            for (i, &h) in kernel.iter().enumerate() {
                let phase = 2.0 * PI * fc_center * i as f64;
                mag_re += h * phase.cos();
                mag_im += h * phase.sin();
            }
            let mag = (mag_re * mag_re + mag_im * mag_im).sqrt();
            if mag > 1e-10 {
                for h in kernel.iter_mut() {
                    *h /= mag;
                }
            }
        }

        // Convolve (linear convolution, output same length as input)
        convolve_same(signal, &kernel)
    }

    /// Compute average power in a specific frequency band using Welch's method.
    ///
    /// Uses overlapping windowed segments to estimate the PSD, then integrates
    /// over the specified frequency range.
    ///
    /// # Arguments
    /// - `signal`: Input time series
    /// - `band`: Frequency band to measure
    /// - `sample_rate`: Sampling rate in Hz
    ///
    /// # Returns
    /// Average power in the specified band (arbitrary units).
    pub fn band_power(signal: &[f64], band: &FrequencyBand, sample_rate: f64) -> f64 {
        let (low, high) = band.range_hz();
        let window_size = (sample_rate * 2.0) as usize; // 2-second windows
        let window_size = window_size.min(signal.len()).max(4);

        let psd = Self::power_spectral_density(signal, window_size, sample_rate);

        let mut power = 0.0;
        let mut count = 0;
        for &(freq, pwr) in &psd {
            if freq >= low && freq <= high {
                power += pwr;
                count += 1;
            }
        }

        if count > 0 {
            power
        } else {
            0.0
        }
    }

    /// Compute power in all five standard EEG bands simultaneously.
    ///
    /// More efficient than calling `band_power` five times since the PSD
    /// is computed only once.
    ///
    /// # Returns
    /// `BandPowers` struct with delta, theta, alpha, beta, gamma, and total power.
    pub fn all_band_powers(signal: &[f64], sample_rate: f64) -> BandPowers {
        let window_size = (sample_rate * 2.0) as usize;
        let window_size = window_size.min(signal.len()).max(4);

        let psd = Self::power_spectral_density(signal, window_size, sample_rate);

        let mut delta = 0.0;
        let mut theta = 0.0;
        let mut alpha = 0.0;
        let mut beta = 0.0;
        let mut gamma = 0.0;

        for &(freq, pwr) in &psd {
            if freq >= 0.5 && freq < 4.0 {
                delta += pwr;
            }
            if freq >= 4.0 && freq < 8.0 {
                theta += pwr;
            }
            if freq >= 8.0 && freq < 13.0 {
                alpha += pwr;
            }
            if freq >= 13.0 && freq < 30.0 {
                beta += pwr;
            }
            if freq >= 30.0 && freq <= 100.0 {
                gamma += pwr;
            }
        }

        let total = delta + theta + alpha + beta + gamma;

        BandPowers {
            delta,
            theta,
            alpha,
            beta,
            gamma,
            total,
        }
    }

    /// Estimate power spectral density using Welch's method.
    ///
    /// Divides the signal into overlapping Hann-windowed segments, computes
    /// the periodogram for each, and averages them. This reduces PSD variance
    /// compared to a single FFT.
    ///
    /// # Arguments
    /// - `signal`: Input time series
    /// - `window_size`: Length of each segment (determines frequency resolution)
    /// - `sample_rate`: Sampling rate in Hz
    ///
    /// # Returns
    /// Vector of (frequency_hz, power) pairs, from 0 Hz to Nyquist.
    pub fn power_spectral_density(
        signal: &[f64],
        window_size: usize,
        sample_rate: f64,
    ) -> Vec<(f64, f64)> {
        if signal.is_empty() || window_size < 2 {
            return vec![];
        }

        let win_size = window_size.min(signal.len());
        let overlap = win_size / 2;
        let step = win_size - overlap;

        // Hann window
        let window: Vec<f64> = (0..win_size)
            .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / (win_size - 1) as f64).cos()))
            .collect();

        // Window power normalization
        let win_power: f64 = window.iter().map(|w| w * w).sum::<f64>() / win_size as f64;

        let n_fft = win_size;
        let n_freqs = n_fft / 2 + 1;
        let mut psd_accum = vec![0.0; n_freqs];
        let mut n_segments = 0;

        let mut start = 0;
        while start + win_size <= signal.len() {
            // Apply window
            let windowed: Vec<f64> = (0..win_size)
                .map(|i| signal[start + i] * window[i])
                .collect();

            // Compute DFT magnitude squared
            for k in 0..n_freqs {
                let mut re = 0.0;
                let mut im = 0.0;
                for (n, &x) in windowed.iter().enumerate() {
                    let angle = 2.0 * PI * k as f64 * n as f64 / n_fft as f64;
                    re += x * angle.cos();
                    im -= x * angle.sin();
                }
                psd_accum[k] += (re * re + im * im) / (sample_rate * win_power * n_fft as f64);
            }

            n_segments += 1;
            start += step;
        }

        // If signal is shorter than window, use single segment
        if n_segments == 0 {
            let actual_len = signal.len();
            let win: Vec<f64> = (0..actual_len)
                .map(|i| {
                    0.5 * (1.0 - (2.0 * PI * i as f64 / (actual_len - 1).max(1) as f64).cos())
                })
                .collect();
            let wp: f64 = win.iter().map(|w| w * w).sum::<f64>() / actual_len as f64;
            let nf = actual_len / 2 + 1;
            psd_accum = vec![0.0; nf];

            let windowed: Vec<f64> = (0..actual_len).map(|i| signal[i] * win[i]).collect();
            for k in 0..nf {
                let mut re = 0.0;
                let mut im = 0.0;
                for (n, &x) in windowed.iter().enumerate() {
                    let angle = 2.0 * PI * k as f64 * n as f64 / actual_len as f64;
                    re += x * angle.cos();
                    im -= x * angle.sin();
                }
                let norm = if wp > 0.0 {
                    sample_rate * wp * actual_len as f64
                } else {
                    1.0
                };
                psd_accum[k] = (re * re + im * im) / norm;
            }
            n_segments = 1;

            return psd_accum
                .iter()
                .enumerate()
                .map(|(k, &p)| {
                    let freq = k as f64 * sample_rate / actual_len as f64;
                    (freq, p / n_segments as f64)
                })
                .collect();
        }

        // Average and build frequency axis
        psd_accum
            .iter()
            .enumerate()
            .map(|(k, &p)| {
                let freq = k as f64 * sample_rate / n_fft as f64;
                (freq, p / n_segments as f64)
            })
            .collect()
    }

    /// Compute spectral edge frequency (SEF).
    ///
    /// The spectral edge frequency is the frequency below which a given
    /// percentage of the total spectral power resides.
    ///
    /// Common clinical uses:
    /// - SEF95: frequency below which 95% of power resides (depth of anesthesia)
    /// - SEF50: median frequency
    ///
    /// # Arguments
    /// - `psd`: Power spectral density as (frequency, power) pairs
    /// - `percentile`: Target percentile (0.0 to 1.0, e.g., 0.95 for SEF95)
    ///
    /// # Returns
    /// Frequency in Hz at which the cumulative power reaches the percentile.
    pub fn spectral_edge_frequency(psd: &[(f64, f64)], percentile: f64) -> f64 {
        if psd.is_empty() {
            return 0.0;
        }

        let total_power: f64 = psd.iter().map(|&(_, p)| p).sum();
        if total_power <= 0.0 {
            return 0.0;
        }

        let target = total_power * percentile;
        let mut cumulative = 0.0;

        for &(freq, power) in psd {
            cumulative += power;
            if cumulative >= target {
                return freq;
            }
        }

        psd.last().map(|&(f, _)| f).unwrap_or(0.0)
    }
}

// ─── Artifact Detector ───────────────────────────────────────────────────────

/// Artifact detection for EEG signals.
///
/// EEG signals are contaminated by non-cerebral sources:
/// - Eye blinks: Large amplitude deflections in frontal channels (Fp1, Fp2)
/// - Muscle artifacts: High-frequency bursts from EMG contamination
/// - Flatlines: Technical issues (electrode disconnection, amplifier saturation)
/// - Amplitude exceedances: Movement artifacts, electrode pops
pub struct ArtifactDetector;

impl ArtifactDetector {
    /// Detect eye blink artifacts in frontal EEG channels.
    ///
    /// Eye blinks produce large-amplitude (~100-200 uV) deflections
    /// primarily in frontal electrodes (Fp1, Fp2, F3, F4).
    ///
    /// # Arguments
    /// - `frontal_channels`: Time series from frontal electrodes
    /// - `threshold_uv`: Amplitude threshold in microvolts (typical: 100-150)
    ///
    /// # Returns
    /// Vector of (start, end) sample index pairs for detected blinks.
    pub fn detect_eye_blink(
        frontal_channels: &[Vec<f64>],
        threshold_uv: f64,
    ) -> Vec<(usize, usize)> {
        if frontal_channels.is_empty() {
            return vec![];
        }

        // Average frontal channels
        let len = frontal_channels[0].len();
        let mut avg = vec![0.0; len];
        for ch in frontal_channels {
            for (i, &v) in ch.iter().enumerate() {
                if i < len {
                    avg[i] += v;
                }
            }
        }
        let n_ch = frontal_channels.len() as f64;
        for v in avg.iter_mut() {
            *v /= n_ch;
        }

        // Find segments exceeding threshold
        Self::find_segments_above_threshold(&avg, threshold_uv)
    }

    /// Detect muscle (EMG) artifact in EEG.
    ///
    /// Muscle artifacts manifest as high-frequency (>30 Hz) broadband
    /// bursts. Detection uses the high-frequency power ratio.
    ///
    /// # Arguments
    /// - `signal`: Single-channel EEG time series
    /// - `sample_rate`: Sampling rate in Hz
    /// - `threshold`: High-frequency power ratio threshold (typical: 0.5)
    ///
    /// # Returns
    /// Vector of (start, end) sample index pairs for detected muscle artifacts.
    pub fn detect_muscle_artifact(
        signal: &[f64],
        sample_rate: f64,
        threshold: f64,
    ) -> Vec<(usize, usize)> {
        if signal.len() < 4 {
            return vec![];
        }

        // Use a sliding window to compute high-frequency energy ratio
        let window_samples = (sample_rate * 0.25) as usize; // 250ms windows
        let window_samples = window_samples.max(4).min(signal.len());

        let mut artifacts = vec![];
        let mut in_artifact = false;
        let mut start = 0;

        let mut i = 0;
        while i + window_samples <= signal.len() {
            let segment = &signal[i..i + window_samples];

            // Compute total power and high-frequency power
            // Simple approach: compute variance of first-difference (emphasizes high freq)
            let total_var = variance(segment);
            let diffs: Vec<f64> = segment.windows(2).map(|w| w[1] - w[0]).collect();
            let diff_var = variance(&diffs);

            // High-freq ratio: derivative variance / total variance
            let ratio = if total_var > 1e-15 {
                diff_var / total_var
            } else {
                0.0
            };

            if ratio > threshold {
                if !in_artifact {
                    start = i;
                    in_artifact = true;
                }
            } else if in_artifact {
                artifacts.push((start, i + window_samples - 1));
                in_artifact = false;
            }

            i += window_samples / 2; // 50% hop
        }

        if in_artifact {
            artifacts.push((start, signal.len() - 1));
        }

        artifacts
    }

    /// Detect flatline segments (electrode disconnection or amplifier rail).
    ///
    /// A flatline is a region where signal amplitude remains below a threshold
    /// for an extended duration.
    ///
    /// # Arguments
    /// - `signal`: Single-channel EEG time series
    /// - `min_duration_samples`: Minimum flatline length to report
    /// - `threshold`: Maximum absolute amplitude variation to consider flat
    ///
    /// # Returns
    /// Vector of (start, end) sample index pairs for detected flatlines.
    pub fn detect_flatline(
        signal: &[f64],
        min_duration_samples: usize,
        threshold: f64,
    ) -> Vec<(usize, usize)> {
        if signal.len() < 2 {
            return vec![];
        }

        let mut flatlines = vec![];
        let mut start = 0;
        let mut in_flat = true;
        let ref_val = signal[0];

        for i in 1..signal.len() {
            let diff = (signal[i] - ref_val).abs();
            if diff <= threshold {
                if !in_flat {
                    start = i;
                    in_flat = true;
                }
            } else {
                if in_flat && (i - start) >= min_duration_samples {
                    flatlines.push((start, i - 1));
                }
                in_flat = false;
            }
        }

        // Check trailing flatline
        if in_flat && (signal.len() - start) >= min_duration_samples {
            flatlines.push((start, signal.len() - 1));
        }

        flatlines
    }

    /// Zero-out artifact regions in a signal.
    ///
    /// Replaces samples within artifact intervals with zeros. This is a simple
    /// rejection strategy; more sophisticated approaches (e.g., ICA-based
    /// correction) exist but are beyond this module's scope.
    ///
    /// # Arguments
    /// - `signal`: Input time series
    /// - `artifacts`: Artifact region intervals as (start, end) pairs
    ///
    /// # Returns
    /// Copy of signal with artifact regions zeroed.
    pub fn reject_epochs(signal: &[f64], artifacts: &[(usize, usize)]) -> Vec<f64> {
        let mut output = signal.to_vec();
        for &(start, end) in artifacts {
            let s = start.min(output.len());
            let e = (end + 1).min(output.len());
            for sample in output[s..e].iter_mut() {
                *sample = 0.0;
            }
        }
        output
    }

    /// Find segments where absolute amplitude exceeds a threshold.
    ///
    /// Useful for detecting movement artifacts, electrode pops, or
    /// any high-amplitude transients.
    ///
    /// # Arguments
    /// - `signal`: Input time series
    /// - `max_uv`: Maximum allowed absolute amplitude
    ///
    /// # Returns
    /// Vector of (start, end) sample index pairs for exceeding segments.
    pub fn amplitude_threshold(signal: &[f64], max_uv: f64) -> Vec<(usize, usize)> {
        Self::find_segments_above_threshold(signal, max_uv)
    }

    /// Internal: find contiguous segments where |signal| > threshold.
    fn find_segments_above_threshold(signal: &[f64], threshold: f64) -> Vec<(usize, usize)> {
        let mut segments = vec![];
        let mut in_segment = false;
        let mut start = 0;

        for (i, &v) in signal.iter().enumerate() {
            if v.abs() > threshold {
                if !in_segment {
                    start = i;
                    in_segment = true;
                }
            } else if in_segment {
                segments.push((start, i - 1));
                in_segment = false;
            }
        }

        if in_segment {
            segments.push((start, signal.len() - 1));
        }

        segments
    }
}

// ─── ERP Analyzer ────────────────────────────────────────────────────────────

/// Event-Related Potential (ERP) analyzer.
///
/// ERPs are time-locked brain responses to specific events (stimuli, actions).
/// They are extracted by averaging multiple epochs aligned to event onsets,
/// which cancels random noise while preserving the consistent neural response.
///
/// Common ERP components:
/// - N100: Negative peak ~100ms (auditory attention)
/// - P200: Positive peak ~200ms (stimulus classification)
/// - N200: Negative peak ~200ms (mismatch detection)
/// - P300: Positive peak ~300ms (target detection, BCI paradigm)
/// - N400: Negative peak ~400ms (semantic processing)
pub struct ErpAnalyzer {
    /// Sampling rate in Hz.
    pub sample_rate_hz: f64,
}

impl ErpAnalyzer {
    /// Create a new ERP analyzer.
    ///
    /// # Arguments
    /// - `sample_rate_hz`: Sampling rate of the EEG data
    pub fn new(sample_rate_hz: f64) -> Self {
        Self { sample_rate_hz }
    }

    /// Extract time-locked epochs around event markers.
    ///
    /// Cuts segments from the continuous signal centered on each event.
    /// Each epoch spans [-pre_samples, +post_samples] around the event.
    ///
    /// # Arguments
    /// - `signal`: Continuous EEG time series
    /// - `event_indices`: Sample indices where events occurred
    /// - `pre_samples`: Number of samples before event (baseline period)
    /// - `post_samples`: Number of samples after event (response period)
    ///
    /// # Returns
    /// Vector of epochs. Each epoch has length (pre_samples + post_samples).
    /// Events too close to signal boundaries are skipped.
    pub fn epoch_extraction(
        &self,
        signal: &[f64],
        event_indices: &[usize],
        pre_samples: usize,
        post_samples: usize,
    ) -> Vec<Vec<f64>> {
        let epoch_len = pre_samples + post_samples;
        let mut epochs = vec![];

        for &event in event_indices {
            if event >= pre_samples && event + post_samples <= signal.len() {
                let start = event - pre_samples;
                let end = event + post_samples;
                epochs.push(signal[start..end].to_vec());
            }
        }

        // Ensure consistent epoch length
        epochs.retain(|e| e.len() == epoch_len);
        epochs
    }

    /// Apply baseline correction to epochs.
    ///
    /// Subtracts the mean of the pre-stimulus baseline period from each epoch.
    /// This removes slow voltage drifts and ensures the pre-stimulus period
    /// has approximately zero mean.
    ///
    /// # Arguments
    /// - `epochs`: Mutable vector of epochs to correct
    /// - `baseline_samples`: Number of initial samples to use as baseline
    pub fn baseline_correction(epochs: &mut Vec<Vec<f64>>, baseline_samples: usize) {
        for epoch in epochs.iter_mut() {
            if baseline_samples > 0 && baseline_samples <= epoch.len() {
                let baseline_mean: f64 = epoch[..baseline_samples].iter().sum::<f64>()
                    / baseline_samples as f64;
                for sample in epoch.iter_mut() {
                    *sample -= baseline_mean;
                }
            }
        }
    }

    /// Compute the grand average ERP across all epochs.
    ///
    /// Point-by-point averaging cancels random noise while preserving
    /// consistent event-related components. The SNR improves as sqrt(N)
    /// where N is the number of epochs.
    ///
    /// # Arguments
    /// - `epochs`: Slice of equal-length epochs
    ///
    /// # Returns
    /// Averaged ERP waveform.
    pub fn average_erp(epochs: &[Vec<f64>]) -> Vec<f64> {
        if epochs.is_empty() {
            return vec![];
        }

        let len = epochs[0].len();
        let mut avg = vec![0.0; len];

        for epoch in epochs {
            for (i, &v) in epoch.iter().enumerate() {
                if i < len {
                    avg[i] += v;
                }
            }
        }

        let n = epochs.len() as f64;
        for v in avg.iter_mut() {
            *v /= n;
        }

        avg
    }

    /// Find peak latency and amplitude within a time window.
    ///
    /// Searches for the maximum (positive polarity) or minimum (negative polarity)
    /// value within the specified sample window.
    ///
    /// # Arguments
    /// - `erp`: ERP waveform
    /// - `window`: (start, end) sample indices to search within
    /// - `polarity`: Whether to search for positive or negative peak
    ///
    /// # Returns
    /// (sample_index, amplitude) of the detected peak.
    pub fn peak_latency(&self, erp: &[f64], window: (usize, usize), polarity: Polarity) -> (usize, f64) {
        let start = window.0.min(erp.len());
        let end = window.1.min(erp.len());

        if start >= end {
            return (0, 0.0);
        }

        let mut best_idx = start;
        let mut best_val = erp[start];

        for i in start..end {
            match polarity {
                Polarity::Positive => {
                    if erp[i] > best_val {
                        best_val = erp[i];
                        best_idx = i;
                    }
                }
                Polarity::Negative => {
                    if erp[i] < best_val {
                        best_val = erp[i];
                        best_idx = i;
                    }
                }
            }
        }

        (best_idx, best_val)
    }

    /// Estimate signal-to-noise ratio of an ERP.
    ///
    /// SNR = power(average_erp) / mean(power(noise_epochs))
    /// where noise epochs are from a condition with no expected response.
    ///
    /// # Arguments
    /// - `erp`: Averaged ERP waveform
    /// - `noise_epochs`: Individual epochs from a noise/control condition
    ///
    /// # Returns
    /// SNR in linear scale (>1 means signal dominates).
    pub fn snr_erp(erp: &[f64], noise_epochs: &[Vec<f64>]) -> f64 {
        if erp.is_empty() || noise_epochs.is_empty() {
            return 0.0;
        }

        let signal_power: f64 = erp.iter().map(|x| x * x).sum::<f64>() / erp.len() as f64;

        let mut noise_power = 0.0;
        for epoch in noise_epochs {
            let p: f64 = epoch.iter().map(|x| x * x).sum::<f64>() / epoch.len().max(1) as f64;
            noise_power += p;
        }
        noise_power /= noise_epochs.len() as f64;

        if noise_power > 1e-15 {
            signal_power / noise_power
        } else {
            0.0
        }
    }
}

// ─── Coherence Analyzer ──────────────────────────────────────────────────────

/// Inter-channel coherence analysis.
///
/// Coherence measures the degree of linear relationship between two signals
/// as a function of frequency. It is the frequency-domain analog of correlation.
///
/// Applications:
/// - Functional connectivity between brain regions
/// - Synchronization analysis in epilepsy
/// - Brain-computer interface feature extraction
pub struct CoherenceAnalyzer;

impl CoherenceAnalyzer {
    /// Compute magnitude squared coherence (MSC) between two signals.
    ///
    /// MSC gamma^2(f) = |Sxy(f)|^2 / (Sxx(f) * Syy(f))
    ///
    /// Values range from 0 (no linear relationship) to 1 (perfect correlation).
    /// Uses Welch's method with Hann windowing for spectral estimation.
    ///
    /// # Arguments
    /// - `signal1`: First channel time series
    /// - `signal2`: Second channel time series (same length)
    /// - `window_size`: Segment length for spectral estimation
    /// - `sample_rate`: Sampling rate in Hz
    ///
    /// # Returns
    /// Vector of (frequency_hz, coherence) pairs.
    pub fn magnitude_squared_coherence(
        signal1: &[f64],
        signal2: &[f64],
        window_size: usize,
        sample_rate: f64,
    ) -> Vec<(f64, f64)> {
        let len = signal1.len().min(signal2.len());
        if len < 2 || window_size < 2 {
            return vec![];
        }

        let win_size = window_size.min(len);
        let overlap = win_size / 2;
        let step = win_size - overlap;
        let n_freqs = win_size / 2 + 1;

        // Hann window
        let window: Vec<f64> = (0..win_size)
            .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / (win_size - 1) as f64).cos()))
            .collect();

        let mut sxx = vec![0.0; n_freqs];
        let mut syy = vec![0.0; n_freqs];
        let mut sxy_re = vec![0.0; n_freqs];
        let mut sxy_im = vec![0.0; n_freqs];
        let mut n_segments = 0u32;

        let mut start = 0;
        while start + win_size <= len {
            // Windowed segments
            let x: Vec<f64> = (0..win_size)
                .map(|i| signal1[start + i] * window[i])
                .collect();
            let y: Vec<f64> = (0..win_size)
                .map(|i| signal2[start + i] * window[i])
                .collect();

            // DFT of both
            for k in 0..n_freqs {
                let mut xre = 0.0;
                let mut xim = 0.0;
                let mut yre = 0.0;
                let mut yim = 0.0;
                for n in 0..win_size {
                    let angle = 2.0 * PI * k as f64 * n as f64 / win_size as f64;
                    let c = angle.cos();
                    let s = angle.sin();
                    xre += x[n] * c;
                    xim -= x[n] * s;
                    yre += y[n] * c;
                    yim -= y[n] * s;
                }

                sxx[k] += xre * xre + xim * xim;
                syy[k] += yre * yre + yim * yim;
                // Cross-spectrum: X(f) * conj(Y(f))
                sxy_re[k] += xre * yre + xim * yim;
                sxy_im[k] += xim * yre - xre * yim;
            }

            n_segments += 1;
            start += step;
        }

        if n_segments == 0 {
            return vec![];
        }

        // Compute MSC
        (0..n_freqs)
            .map(|k| {
                let freq = k as f64 * sample_rate / win_size as f64;
                let denom = sxx[k] * syy[k];
                let coh = if denom > 1e-30 {
                    let cross_mag_sq = sxy_re[k] * sxy_re[k] + sxy_im[k] * sxy_im[k];
                    (cross_mag_sq / denom).min(1.0)
                } else {
                    0.0
                };
                (freq, coh)
            })
            .collect()
    }

    /// Compute Phase Locking Value (PLV) between two phase series.
    ///
    /// PLV = |mean(exp(j * (phi1 - phi2)))|
    ///
    /// Measures the consistency of the phase difference across time.
    /// Values range from 0 (random phase relationship) to 1 (perfectly locked).
    ///
    /// # Arguments
    /// - `phases1`: Instantaneous phases of signal 1 (in radians)
    /// - `phases2`: Instantaneous phases of signal 2 (in radians)
    ///
    /// # Returns
    /// PLV value between 0 and 1.
    pub fn phase_locking_value(phases1: &[f64], phases2: &[f64]) -> f64 {
        let len = phases1.len().min(phases2.len());
        if len == 0 {
            return 0.0;
        }

        let mut sum_cos = 0.0;
        let mut sum_sin = 0.0;

        for i in 0..len {
            let delta = phases1[i] - phases2[i];
            sum_cos += delta.cos();
            sum_sin += delta.sin();
        }

        let mean_cos = sum_cos / len as f64;
        let mean_sin = sum_sin / len as f64;

        (mean_cos * mean_cos + mean_sin * mean_sin).sqrt()
    }

    /// Estimate instantaneous phase via finite-difference approximation.
    ///
    /// Uses a simple analytic signal approximation based on the discrete
    /// Hilbert-like transform (finite-difference method). The phase is
    /// computed as atan2(hilbert(x), x).
    ///
    /// For more accurate results, a proper Hilbert transform (FIR-based)
    /// should be used, but this provides a reasonable approximation
    /// without requiring FFT.
    ///
    /// # Arguments
    /// - `signal`: Real-valued input signal
    ///
    /// # Returns
    /// Instantaneous phase at each sample (in radians, range -pi to pi).
    pub fn instantaneous_phase(signal: &[f64]) -> Vec<f64> {
        let n = signal.len();
        if n < 2 {
            return vec![0.0; n];
        }

        // Approximate Hilbert transform using FIR kernel
        // h[k] = (2/(pi*k)) for odd k, 0 for even k
        let half_len = 15.min(n / 2);
        let mut hilbert = vec![0.0; n];

        for i in 0..n {
            let mut sum = 0.0;
            for k in 1..=half_len {
                let odd_k = 2 * k - 1;
                let coeff = 2.0 / (PI * odd_k as f64);
                if i + odd_k < n {
                    sum += coeff * signal[i + odd_k];
                }
                if i >= odd_k {
                    sum -= coeff * signal[i - odd_k];
                }
            }
            hilbert[i] = sum;
        }

        // Phase = atan2(hilbert, original)
        signal
            .iter()
            .zip(hilbert.iter())
            .map(|(&x, &h)| h.atan2(x))
            .collect()
    }
}

// ─── Utility Functions ───────────────────────────────────────────────────────

/// Compute variance of a slice.
fn variance(data: &[f64]) -> f64 {
    if data.len() < 2 {
        return 0.0;
    }
    let mean = data.iter().sum::<f64>() / data.len() as f64;
    data.iter().map(|x| (x - mean) * (x - mean)).sum::<f64>() / data.len() as f64
}

/// Linear convolution producing output with same length as input.
fn convolve_same(signal: &[f64], kernel: &[f64]) -> Vec<f64> {
    let sig_len = signal.len();
    let ker_len = kernel.len();
    if sig_len == 0 || ker_len == 0 {
        return signal.to_vec();
    }

    let offset = ker_len / 2;
    let mut output = vec![0.0; sig_len];

    for i in 0..sig_len {
        let mut sum = 0.0;
        for j in 0..ker_len {
            let idx = i as isize + j as isize - offset as isize;
            if idx >= 0 && (idx as usize) < sig_len {
                sum += signal[idx as usize] * kernel[j];
            }
        }
        output[i] = sum;
    }

    output
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn make_sine(freq_hz: f64, sample_rate: f64, duration_s: f64) -> Vec<f64> {
        let n = (sample_rate * duration_s) as usize;
        (0..n)
            .map(|i| (2.0 * PI * freq_hz * i as f64 / sample_rate).sin())
            .collect()
    }

    fn make_sine_amplitude(freq_hz: f64, sample_rate: f64, duration_s: f64, amp: f64) -> Vec<f64> {
        let n = (sample_rate * duration_s) as usize;
        (0..n)
            .map(|i| amp * (2.0 * PI * freq_hz * i as f64 / sample_rate).sin())
            .collect()
    }

    // ─── ReferenceScheme Tests ───────────────────────────────────────────

    #[test]
    fn test_common_average_zeroes_mean() {
        let mut channels = vec![
            vec![1.0, 2.0, 3.0, 4.0],
            vec![5.0, 6.0, 7.0, 8.0],
            vec![3.0, 4.0, 5.0, 6.0],
        ];
        EegProcessor::re_reference(&mut channels, &ReferenceScheme::CommonAverage);

        // After common average, the mean across channels at each sample should be ~0
        for s in 0..4 {
            let mean: f64 = channels.iter().map(|ch| ch[s]).sum::<f64>() / 3.0;
            assert!(
                mean.abs() < 1e-10,
                "Mean at sample {} = {} (expected ~0)",
                s,
                mean
            );
        }
    }

    #[test]
    fn test_linked_ears_reference() {
        let mut channels = vec![
            vec![10.0, 20.0],
            vec![2.0, 4.0],  // left ear
            vec![6.0, 8.0],  // right ear
            vec![15.0, 25.0],
        ];
        EegProcessor::re_reference(
            &mut channels,
            &ReferenceScheme::LinkedEars {
                left_idx: 1,
                right_idx: 2,
            },
        );

        // Ear average at sample 0: (2+6)/2 = 4
        assert!((channels[0][0] - 6.0).abs() < 1e-10); // 10 - 4 = 6
        assert!((channels[3][0] - 11.0).abs() < 1e-10); // 15 - 4 = 11
    }

    #[test]
    fn test_bipolar_reference() {
        let mut channels = vec![vec![10.0, 20.0], vec![7.0, 15.0], vec![3.0, 8.0]];
        EegProcessor::re_reference(&mut channels, &ReferenceScheme::Bipolar);

        assert!((channels[0][0] - 3.0).abs() < 1e-10); // 10 - 7
        assert!((channels[1][0] - 4.0).abs() < 1e-10); // 7 - 3
        assert!((channels[2][0]).abs() < 1e-10); // last channel zeroed
    }

    #[test]
    fn test_laplacian_reference() {
        let mut channels = vec![
            vec![10.0],
            vec![4.0],
            vec![6.0],
        ];
        let neighbors = vec![
            vec![1, 2],  // ch0 neighbors: ch1, ch2
            vec![0, 2],  // ch1 neighbors: ch0, ch2
            vec![0, 1],  // ch2 neighbors: ch0, ch1
        ];
        EegProcessor::re_reference(&mut channels, &ReferenceScheme::Laplacian { neighbors });

        // ch0: 10 - (4+6)/2 = 10 - 5 = 5
        assert!((channels[0][0] - 5.0).abs() < 1e-10);
    }

    // ─── Notch Filter Tests ─────────────────────────────────────────────

    #[test]
    fn test_notch_filter_attenuates_target_frequency() {
        let sample_rate = 256.0;
        let duration = 2.0;
        let notch_freq = 60.0;

        // Signal: 10 Hz + 60 Hz
        let n = (sample_rate * duration) as usize;
        let mut signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                (2.0 * PI * 10.0 * t).sin() + (2.0 * PI * notch_freq * t).sin()
            })
            .collect();

        // Measure power at 60 Hz before
        let power_before = goertzel_power(&signal, notch_freq, sample_rate);

        EegProcessor::notch_filter(&mut signal, notch_freq, 30.0, sample_rate);

        // Measure power at 60 Hz after
        let power_after = goertzel_power(&signal, notch_freq, sample_rate);

        // 60 Hz should be substantially reduced
        assert!(
            power_after < power_before * 0.1,
            "Notch filter should attenuate 60 Hz: before={}, after={}",
            power_before,
            power_after
        );
    }

    #[test]
    fn test_notch_filter_preserves_other_frequencies() {
        let sample_rate = 256.0;
        let duration = 2.0;

        let n = (sample_rate * duration) as usize;
        let mut signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                (2.0 * PI * 10.0 * t).sin() + (2.0 * PI * 60.0 * t).sin()
            })
            .collect();

        let power_10hz_before = goertzel_power(&signal, 10.0, sample_rate);
        EegProcessor::notch_filter(&mut signal, 60.0, 30.0, sample_rate);
        let power_10hz_after = goertzel_power(&signal, 10.0, sample_rate);

        // 10 Hz should be preserved (within 20%)
        let ratio = power_10hz_after / power_10hz_before;
        assert!(
            ratio > 0.8 && ratio < 1.2,
            "10 Hz should be preserved: ratio={}",
            ratio
        );
    }

    // ─── Bandpass Filter Tests ──────────────────────────────────────────

    #[test]
    fn test_bandpass_filter_passes_in_band() {
        let sample_rate = 256.0;
        let signal = make_sine(10.0, sample_rate, 2.0);
        let filtered = EegProcessor::bandpass_filter(&signal, 8.0, 13.0, sample_rate, 65);

        let power_in = signal.iter().map(|x| x * x).sum::<f64>() / signal.len() as f64;
        let power_out = filtered.iter().map(|x| x * x).sum::<f64>() / filtered.len() as f64;

        // In-band signal should pass through with reasonable power
        assert!(
            power_out > power_in * 0.3,
            "In-band signal should pass: in={}, out={}",
            power_in,
            power_out
        );
    }

    #[test]
    fn test_bandpass_filter_rejects_out_of_band() {
        let sample_rate = 256.0;
        let signal = make_sine(50.0, sample_rate, 2.0);
        let filtered = EegProcessor::bandpass_filter(&signal, 8.0, 13.0, sample_rate, 101);

        let power_in = signal.iter().map(|x| x * x).sum::<f64>() / signal.len() as f64;
        let power_out = filtered.iter().map(|x| x * x).sum::<f64>() / filtered.len() as f64;

        // Out-of-band signal should be attenuated
        assert!(
            power_out < power_in * 0.1,
            "Out-of-band signal should be rejected: in={}, out={}",
            power_in,
            power_out
        );
    }

    // ─── Band Power Tests ────────────────────────────────────────────────

    #[test]
    fn test_alpha_power_for_10hz_signal() {
        let sample_rate = 256.0;
        let signal = make_sine(10.0, sample_rate, 4.0);
        let power = EegProcessor::band_power(&signal, &FrequencyBand::Alpha, sample_rate);

        assert!(
            power > 0.0,
            "Alpha power should be positive for 10 Hz signal"
        );
    }

    #[test]
    fn test_delta_band_range() {
        let (low, high) = FrequencyBand::Delta.range_hz();
        assert!((low - 0.5).abs() < 1e-10);
        assert!((high - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_all_band_powers_10hz_dominant_alpha() {
        let sample_rate = 256.0;
        let signal = make_sine(10.0, sample_rate, 4.0);
        let powers = EegProcessor::all_band_powers(&signal, sample_rate);

        assert!(
            powers.alpha > powers.delta,
            "10 Hz signal should have more alpha than delta"
        );
        assert!(
            powers.alpha > powers.theta,
            "10 Hz signal should have more alpha than theta"
        );
    }

    #[test]
    fn test_band_power_pure_tone_in_correct_band() {
        let sample_rate = 256.0;

        // 2 Hz tone -> should be in delta
        let sig_delta = make_sine(2.0, sample_rate, 4.0);
        let delta_power =
            EegProcessor::band_power(&sig_delta, &FrequencyBand::Delta, sample_rate);
        let alpha_power =
            EegProcessor::band_power(&sig_delta, &FrequencyBand::Alpha, sample_rate);
        assert!(
            delta_power > alpha_power,
            "2 Hz tone should have more delta than alpha power"
        );

        // 6 Hz tone -> should be in theta
        let sig_theta = make_sine(6.0, sample_rate, 4.0);
        let theta_power =
            EegProcessor::band_power(&sig_theta, &FrequencyBand::Theta, sample_rate);
        let beta_power = EegProcessor::band_power(&sig_theta, &FrequencyBand::Beta, sample_rate);
        assert!(
            theta_power > beta_power,
            "6 Hz tone should have more theta than beta power"
        );
    }

    #[test]
    fn test_relative_power_sums_to_one() {
        let sample_rate = 256.0;
        // White-ish noise
        let signal: Vec<f64> = (0..1024)
            .map(|i| {
                let t = i as f64 / sample_rate;
                (2.0 * PI * 2.0 * t).sin()
                    + (2.0 * PI * 6.0 * t).sin()
                    + (2.0 * PI * 10.0 * t).sin()
                    + (2.0 * PI * 20.0 * t).sin()
                    + (2.0 * PI * 40.0 * t).sin()
            })
            .collect();
        let powers = EegProcessor::all_band_powers(&signal, sample_rate);

        let sum = powers.relative_power(&FrequencyBand::Delta)
            + powers.relative_power(&FrequencyBand::Theta)
            + powers.relative_power(&FrequencyBand::Alpha)
            + powers.relative_power(&FrequencyBand::Beta)
            + powers.relative_power(&FrequencyBand::Gamma);

        assert!(
            (sum - 1.0).abs() < 0.01,
            "Relative powers should sum to ~1.0, got {}",
            sum
        );
    }

    #[test]
    fn test_alpha_theta_ratio() {
        let powers = BandPowers {
            delta: 1.0,
            theta: 2.0,
            alpha: 6.0,
            beta: 3.0,
            gamma: 1.0,
            total: 13.0,
        };
        assert!((powers.alpha_theta_ratio() - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_theta_beta_ratio() {
        let powers = BandPowers {
            delta: 1.0,
            theta: 4.0,
            alpha: 6.0,
            beta: 2.0,
            gamma: 1.0,
            total: 14.0,
        };
        assert!((powers.theta_beta_ratio() - 2.0).abs() < 1e-10);
    }

    // ─── PSD Tests ──────────────────────────────────────────────────────

    #[test]
    fn test_psd_has_peak_at_signal_frequency() {
        let sample_rate = 256.0;
        let signal = make_sine(10.0, sample_rate, 4.0);
        let psd = EegProcessor::power_spectral_density(&signal, 256, sample_rate);

        // Find the frequency bin with max power
        let (peak_freq, _) = psd
            .iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();

        assert!(
            (*peak_freq - 10.0).abs() < 2.0,
            "PSD peak should be near 10 Hz, got {} Hz",
            peak_freq
        );
    }

    #[test]
    fn test_spectral_edge_frequency() {
        // Create a simple PSD: uniform power up to 20 Hz, nothing above
        let psd: Vec<(f64, f64)> = (0..100)
            .map(|k| {
                let f = k as f64;
                let p = if f <= 20.0 { 1.0 } else { 0.0 };
                (f, p)
            })
            .collect();

        let sef95 = EegProcessor::spectral_edge_frequency(&psd, 0.95);
        assert!(
            sef95 <= 20.0,
            "SEF95 should be <= 20 Hz for uniform PSD up to 20 Hz, got {}",
            sef95
        );
        assert!(sef95 >= 18.0, "SEF95 should be near 20 Hz, got {}", sef95);
    }

    // ─── Artifact Detector Tests ────────────────────────────────────────

    #[test]
    fn test_detect_eye_blink_finds_high_amplitude() {
        // Simulated frontal channel with two blinks
        let mut ch = vec![0.0; 500];
        // Blink 1 at samples 100-120
        for i in 100..120 {
            ch[i] = 200.0;
        }
        // Blink 2 at samples 300-310
        for i in 300..310 {
            ch[i] = 150.0;
        }

        let blinks = ArtifactDetector::detect_eye_blink(&[ch], 100.0);
        assert!(
            blinks.len() >= 2,
            "Should detect at least 2 blinks, found {}",
            blinks.len()
        );
    }

    #[test]
    fn test_detect_flatline() {
        let mut signal = vec![0.0; 200];
        // Add some variation
        for i in 0..50 {
            signal[i] = (i as f64 * 0.1).sin() * 10.0;
        }
        // Flat region from 50 to 150
        // Already zeros from initialization
        // Add variation after
        for i in 150..200 {
            signal[i] = (i as f64 * 0.1).sin() * 10.0;
        }

        let flatlines = ArtifactDetector::detect_flatline(&signal, 20, 0.5);
        assert!(
            !flatlines.is_empty(),
            "Should detect the flatline region"
        );
    }

    #[test]
    fn test_amplitude_threshold() {
        let mut signal = vec![0.0; 100];
        signal[30] = 200.0;
        signal[31] = 200.0;
        signal[32] = 200.0;
        signal[70] = -150.0;

        let artifacts = ArtifactDetector::amplitude_threshold(&signal, 100.0);
        assert!(
            artifacts.len() >= 2,
            "Should find at least 2 segments above threshold"
        );
    }

    #[test]
    fn test_reject_epochs_zeros_artifacts() {
        let signal = vec![1.0; 100];
        let artifacts = vec![(10, 20), (50, 60)];
        let cleaned = ArtifactDetector::reject_epochs(&signal, &artifacts);

        // Artifact regions should be zero
        for i in 10..=20 {
            assert!(
                cleaned[i].abs() < 1e-10,
                "Sample {} should be zeroed",
                i
            );
        }
        // Non-artifact regions should be preserved
        assert!((cleaned[0] - 1.0).abs() < 1e-10);
        assert!((cleaned[30] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_muscle_artifact_detection() {
        let sample_rate = 256.0;
        let n = 1024;
        let mut signal = vec![0.0; n];

        // Normal low-frequency signal
        for i in 0..n {
            signal[i] = (2.0 * PI * 5.0 * i as f64 / sample_rate).sin();
        }

        // Inject high-frequency burst (muscle artifact) at samples 400-500
        for i in 400..500 {
            signal[i] += 10.0 * (2.0 * PI * 80.0 * i as f64 / sample_rate).sin();
        }

        let artifacts = ArtifactDetector::detect_muscle_artifact(&signal, sample_rate, 0.5);
        // Should detect the high-frequency burst region
        let has_artifact_in_range = artifacts
            .iter()
            .any(|&(s, e)| s <= 500 && e >= 400);
        assert!(
            has_artifact_in_range,
            "Should detect muscle artifact near samples 400-500, found {:?}",
            artifacts
        );
    }

    // ─── ERP Tests ──────────────────────────────────────────────────────

    #[test]
    fn test_epoch_extraction_correct_length() {
        let signal: Vec<f64> = (0..1000).map(|i| i as f64).collect();
        let events = vec![100usize, 300, 500, 700];
        let pre: usize = 50;
        let post: usize = 100;

        let analyzer = ErpAnalyzer::new(256.0);
        let epochs = analyzer.epoch_extraction(&signal, &events, pre, post);

        assert_eq!(epochs.len(), 4);
        for epoch in &epochs {
            assert_eq!(epoch.len(), pre + post);
        }
    }

    #[test]
    fn test_epoch_extraction_skips_boundary_events() {
        let signal: Vec<f64> = (0..200).map(|i| i as f64).collect();
        // Event at 10 with pre=50 would go before signal start
        let events = vec![10usize, 100, 195];
        let pre: usize = 50;
        let post: usize = 50;

        let analyzer = ErpAnalyzer::new(256.0);
        let epochs = analyzer.epoch_extraction(&signal, &events, pre, post);

        // Only event at 100 should be valid
        assert_eq!(epochs.len(), 1, "Only event at 100 should produce an epoch");
    }

    #[test]
    fn test_baseline_correction_makes_prestimulus_zero() {
        let analyzer = ErpAnalyzer::new(256.0);
        let mut epochs = vec![
            vec![5.0, 5.0, 5.0, 10.0, 15.0, 20.0],
            vec![3.0, 3.0, 3.0, 8.0, 12.0, 18.0],
        ];
        let baseline_samples = 3;

        ErpAnalyzer::baseline_correction(&mut epochs, baseline_samples);

        // Pre-stimulus mean should be ~0
        for epoch in &epochs {
            let baseline_mean: f64 =
                epoch[..baseline_samples].iter().sum::<f64>() / baseline_samples as f64;
            assert!(
                baseline_mean.abs() < 1e-10,
                "Baseline mean should be ~0, got {}",
                baseline_mean
            );
        }
    }

    #[test]
    fn test_average_erp_reduces_noise() {
        let analyzer = ErpAnalyzer::new(256.0);
        let n = 100; // epoch length
        let n_epochs = 50;

        // Create epochs: consistent signal + random-ish noise
        let mut epochs: Vec<Vec<f64>> = Vec::new();
        for ep in 0..n_epochs {
            let epoch: Vec<f64> = (0..n)
                .map(|i| {
                    // Signal component: positive bump at samples 30-50
                    let signal = if i >= 30 && i < 50 { 10.0 } else { 0.0 };
                    // "Noise": varies across epochs using simple hash
                    let noise = ((ep * 137 + i * 31) % 100) as f64 / 50.0 - 1.0;
                    signal + noise
                })
                .collect();
            epochs.push(epoch);
        }

        let avg = ErpAnalyzer::average_erp(&epochs);

        // The signal region should have higher amplitude than noise region
        let signal_mean: f64 = avg[30..50].iter().sum::<f64>() / 20.0;
        let noise_mean: f64 = avg[0..20].iter().map(|x| x.abs()).sum::<f64>() / 20.0;

        assert!(
            signal_mean > noise_mean * 2.0,
            "Averaged ERP signal should dominate noise: signal={}, noise={}",
            signal_mean,
            noise_mean
        );
    }

    #[test]
    fn test_peak_latency_finds_maximum() {
        let erp = vec![0.0, 1.0, 3.0, 5.0, 2.0, 0.0, -1.0];
        let analyzer = ErpAnalyzer::new(256.0);
        let (idx, amp) = analyzer.peak_latency(&erp, (0, 7), Polarity::Positive);
        assert_eq!(idx, 3);
        assert!((amp - 5.0_f64).abs() < 1e-10);
    }

    #[test]
    fn test_peak_latency_negative_polarity() {
        let erp = vec![0.0, 1.0, -3.0, -7.0, -2.0, 0.0];
        let analyzer = ErpAnalyzer::new(256.0);
        let (idx, amp) = analyzer.peak_latency(&erp, (0, 6), Polarity::Negative);
        assert_eq!(idx, 3);
        assert!((amp - (-7.0_f64)).abs() < 1e-10);
    }

    #[test]
    fn test_snr_erp() {
        let analyzer = ErpAnalyzer::new(256.0);

        // Strong ERP signal
        let erp: Vec<f64> = (0..100).map(|i| if i >= 30 && i < 50 { 10.0 } else { 0.0 }).collect();

        // Noise epochs with small amplitude
        let noise_epochs: Vec<Vec<f64>> = (0..5)
            .map(|ep| {
                (0..100)
                    .map(|i| ((ep * 31 + i * 7) % 20) as f64 / 100.0)
                    .collect()
            })
            .collect();

        let snr = ErpAnalyzer::snr_erp(&erp, &noise_epochs);
        assert!(snr > 1.0, "SNR should be > 1 for strong signal, got {}", snr);
    }

    // ─── Coherence Tests ────────────────────────────────────────────────

    #[test]
    fn test_coherence_identical_signals_equals_one() {
        let sample_rate = 256.0;
        let signal = make_sine(10.0, sample_rate, 2.0);

        let coh =
            CoherenceAnalyzer::magnitude_squared_coherence(&signal, &signal, 128, sample_rate);

        // At 10 Hz, coherence should be ~1
        let near_10hz: Vec<_> = coh.iter().filter(|&&(f, _)| (f - 10.0).abs() < 3.0).collect();
        assert!(!near_10hz.is_empty(), "Should have bins near 10 Hz");

        let max_coh = near_10hz.iter().map(|&&(_, c)| c).fold(0.0f64, f64::max);
        assert!(
            max_coh > 0.9,
            "Coherence of identical signals should be ~1, got {}",
            max_coh
        );
    }

    #[test]
    fn test_coherence_uncorrelated_signals_near_zero() {
        let sample_rate = 256.0;
        let n = (sample_rate * 8.0) as usize;

        // Generate two independent pseudo-random noise signals using simple LCG
        let mut signal1 = vec![0.0; n];
        let mut signal2 = vec![0.0; n];
        let mut seed1: u64 = 12345;
        let mut seed2: u64 = 67890;
        for i in 0..n {
            seed1 = seed1.wrapping_mul(6364136223846793005).wrapping_add(1);
            signal1[i] = (seed1 >> 33) as f64 / (1u64 << 31) as f64 - 0.5;
            seed2 = seed2.wrapping_mul(6364136223846793005).wrapping_add(3);
            signal2[i] = (seed2 >> 33) as f64 / (1u64 << 31) as f64 - 0.5;
        }

        let coh = CoherenceAnalyzer::magnitude_squared_coherence(
            &signal1, &signal2, 256, sample_rate,
        );

        // Average coherence across all frequencies should be low for uncorrelated noise
        let avg_coh: f64 = coh.iter().map(|&(_, c)| c).sum::<f64>() / coh.len() as f64;
        assert!(
            avg_coh < 0.5,
            "Average coherence of independent noise should be low, got {}",
            avg_coh
        );
    }

    #[test]
    fn test_phase_locking_value_synchronized() {
        // Two identical phase series -> PLV = 1
        let n = 500;
        let phases: Vec<f64> = (0..n).map(|i| 2.0 * PI * i as f64 / 100.0).collect();

        let plv = CoherenceAnalyzer::phase_locking_value(&phases, &phases);
        assert!(
            (plv - 1.0).abs() < 0.01,
            "PLV for identical phases should be ~1, got {}",
            plv
        );
    }

    #[test]
    fn test_phase_locking_value_constant_offset() {
        // Constant phase offset -> PLV = 1
        let n = 500;
        let phases1: Vec<f64> = (0..n).map(|i| 2.0 * PI * i as f64 / 100.0).collect();
        let phases2: Vec<f64> = phases1.iter().map(|&p| p + 0.5).collect();

        let plv = CoherenceAnalyzer::phase_locking_value(&phases1, &phases2);
        assert!(
            (plv - 1.0).abs() < 0.01,
            "PLV for constant offset phases should be ~1, got {}",
            plv
        );
    }

    #[test]
    fn test_instantaneous_phase_returns_correct_length() {
        let signal = make_sine(10.0, 256.0, 1.0);
        let phases = CoherenceAnalyzer::instantaneous_phase(&signal);
        assert_eq!(phases.len(), signal.len());
    }

    // ─── Frequency Band Tests ───────────────────────────────────────────

    #[test]
    fn test_frequency_band_ranges() {
        assert_eq!(FrequencyBand::Delta.range_hz(), (0.5, 4.0));
        assert_eq!(FrequencyBand::Theta.range_hz(), (4.0, 8.0));
        assert_eq!(FrequencyBand::Alpha.range_hz(), (8.0, 13.0));
        assert_eq!(FrequencyBand::Beta.range_hz(), (13.0, 30.0));
        assert_eq!(FrequencyBand::Gamma.range_hz(), (30.0, 100.0));

        let custom = FrequencyBand::Custom {
            low_hz: 15.0,
            high_hz: 25.0,
        };
        assert_eq!(custom.range_hz(), (15.0, 25.0));
    }

    #[test]
    fn test_band_powers_zero_total() {
        let powers = BandPowers {
            delta: 0.0,
            theta: 0.0,
            alpha: 0.0,
            beta: 0.0,
            gamma: 0.0,
            total: 0.0,
        };
        assert!((powers.relative_power(&FrequencyBand::Alpha)).abs() < 1e-10);
        assert!((powers.alpha_theta_ratio()).abs() < 1e-10);
        assert!((powers.theta_beta_ratio()).abs() < 1e-10);
    }

    // ─── Edge Case Tests ────────────────────────────────────────────────

    #[test]
    fn test_empty_signal_handling() {
        let empty: Vec<f64> = vec![];
        let psd = EegProcessor::power_spectral_density(&empty, 256, 256.0);
        assert!(psd.is_empty());

        let filtered = EegProcessor::bandpass_filter(&empty, 8.0, 13.0, 256.0, 65);
        assert!(filtered.is_empty());
    }

    #[test]
    fn test_erp_empty_epochs() {
        let avg = ErpAnalyzer::average_erp(&[]);
        assert!(avg.is_empty());
    }

    #[test]
    fn test_coherence_empty_signals() {
        let coh = CoherenceAnalyzer::magnitude_squared_coherence(&[], &[], 128, 256.0);
        assert!(coh.is_empty());
    }

    #[test]
    fn test_custom_band_power() {
        let sample_rate = 256.0;
        let signal = make_sine(15.0, sample_rate, 4.0);
        let custom_band = FrequencyBand::Custom {
            low_hz: 12.0,
            high_hz: 18.0,
        };
        let power = EegProcessor::band_power(&signal, &custom_band, sample_rate);
        assert!(power > 0.0, "Custom band should capture 15 Hz tone power");
    }

    // ─── Helper ─────────────────────────────────────────────────────────

    /// Goertzel algorithm for measuring power at a single frequency.
    fn goertzel_power(signal: &[f64], freq_hz: f64, sample_rate: f64) -> f64 {
        let n = signal.len();
        let k = (freq_hz * n as f64 / sample_rate).round() as usize;
        let w = 2.0 * PI * k as f64 / n as f64;
        let coeff = 2.0 * w.cos();

        let mut s0 = 0.0;
        let mut s1 = 0.0;
        let mut s2 = 0.0;

        for &x in signal {
            s0 = x + coeff * s1 - s2;
            s2 = s1;
            s1 = s0;
        }

        let power = s1 * s1 + s2 * s2 - coeff * s1 * s2;
        power / (n as f64 * n as f64)
    }
}
