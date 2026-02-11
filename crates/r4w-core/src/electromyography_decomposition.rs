//! EMG signal decomposition for extracting individual motor unit action potential trains.
//!
//! This module implements decomposition of intramuscular or high-density surface EMG
//! recordings into constituent Motor Unit Action Potential Trains (MUAPTs). The approach
//! uses iterative spike detection, spike-triggered averaging for template extraction,
//! cross-correlation-based template matching, and template subtraction (peel-off) to
//! progressively isolate individual motor units from a composite EMG signal.
//!
//! # Algorithm Overview
//!
//! 1. **Whitening**: AR-based spectral whitening to flatten the noise floor
//! 2. **Spike Detection**: Threshold crossing with refractory period enforcement
//! 3. **Template Extraction**: Spike-triggered averaging across detected events
//! 4. **Template Matching**: Sliding cross-correlation to find all occurrences
//! 5. **Template Subtraction**: Remove classified MUAPs from residual
//! 6. **Iterate**: Repeat on the residual to find additional motor units
//!
//! # Example
//!
//! ```
//! use r4w_core::electromyography_decomposition::{
//!     DecompositionConfig, EmgDecomposer, generate_synthetic_emg,
//! };
//!
//! let config = DecompositionConfig {
//!     sample_rate_hz: 10000.0,
//!     num_channels: 1,
//!     template_duration_ms: 10.0,
//!     min_firing_rate_hz: 6.0,
//!     max_firing_rate_hz: 35.0,
//! };
//!
//! // Generate a synthetic EMG signal with two motor units
//! let template1: Vec<f64> = (0..100).map(|i| {
//!     let t = i as f64 / 10000.0;
//!     1.0 * (-(t - 0.005).powi(2) / (2.0 * 0.001f64.powi(2))).exp()
//! }).collect();
//! let template2: Vec<f64> = (0..100).map(|i| {
//!     let t = i as f64 / 10000.0;
//!     0.6 * (-(t - 0.005).powi(2) / (2.0 * 0.0008f64.powi(2))).exp()
//! }).collect();
//!
//! let emg = generate_synthetic_emg(
//!     &[(template1, 12.0), (template2, 18.0)],
//!     0.5,
//!     10000.0,
//! );
//!
//! let decomposer = EmgDecomposer::new(config);
//! let result = decomposer.decompose(&[emg]);
//! assert!(result.motor_units.len() >= 1);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for the EMG decomposition engine.
#[derive(Debug, Clone)]
pub struct DecompositionConfig {
    /// Sampling rate of the EMG recording in Hz (e.g. 10 000).
    pub sample_rate_hz: f64,
    /// Number of EMG channels (1 for single-channel intramuscular).
    pub num_channels: usize,
    /// Duration of the MUAP template window in milliseconds.
    pub template_duration_ms: f64,
    /// Minimum physiological firing rate in Hz (default 6).
    pub min_firing_rate_hz: f64,
    /// Maximum physiological firing rate in Hz (default 35).
    pub max_firing_rate_hz: f64,
}

impl Default for DecompositionConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 10_000.0,
            num_channels: 1,
            template_duration_ms: 10.0,
            min_firing_rate_hz: 6.0,
            max_firing_rate_hz: 35.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Result types
// ---------------------------------------------------------------------------

/// A single identified motor unit with its template and firing pattern.
#[derive(Debug, Clone)]
pub struct MotorUnit {
    /// Representative MUAP template (spike-triggered average).
    pub template: Vec<f64>,
    /// Sample indices at which this motor unit fired.
    pub firing_times: Vec<usize>,
    /// Mean instantaneous firing rate in Hz.
    pub mean_firing_rate_hz: f64,
    /// Coefficient of variation of inter-spike intervals (std / mean).
    pub cv_isi: f64,
}

/// Aggregate result of EMG decomposition.
#[derive(Debug, Clone)]
pub struct DecompositionResult {
    /// Extracted motor units, ordered by descending template amplitude.
    pub motor_units: Vec<MotorUnit>,
    /// Fraction of original signal energy remaining in the residual.
    pub residual_energy: f64,
    /// Reconstruction quality = 1 - (residual_energy / original_energy).
    pub reconstruction_quality: f64,
}

/// Inter-spike interval statistics.
#[derive(Debug, Clone)]
pub struct IsiStats {
    /// Mean ISI in milliseconds.
    pub mean_ms: f64,
    /// Standard deviation of ISI in milliseconds.
    pub std_ms: f64,
    /// Coefficient of variation (std / mean).
    pub cv: f64,
    /// Minimum ISI in milliseconds.
    pub min_ms: f64,
    /// Maximum ISI in milliseconds.
    pub max_ms: f64,
}

// ---------------------------------------------------------------------------
// Spike detection
// ---------------------------------------------------------------------------

/// Detect spikes by threshold crossing with a refractory period.
///
/// Returns sample indices where `|signal[i]| > threshold` with at least
/// `refractory` samples between consecutive detections. A default refractory
/// period of 20 samples (~2 ms at 10 kHz) is used when the signal first
/// crosses the threshold.
pub fn detect_spikes(signal: &[f64], threshold: f64) -> Vec<usize> {
    if signal.is_empty() {
        return Vec::new();
    }
    // Default refractory period: 20 samples (~2 ms at 10 kHz).
    let refractory: usize = 20;
    let mut spikes = Vec::new();
    let mut last_spike: Option<usize> = None;

    for (i, &s) in signal.iter().enumerate() {
        if s.abs() > threshold {
            let ok = match last_spike {
                Some(prev) => i >= prev + refractory,
                None => true,
            };
            if ok {
                // Refine to local maximum within a small window.
                let start = if i >= 5 { i - 5 } else { 0 };
                let end = (i + 6).min(signal.len());
                let mut best_idx = i;
                let mut best_val = signal[i].abs();
                for j in start..end {
                    if signal[j].abs() > best_val {
                        best_val = signal[j].abs();
                        best_idx = j;
                    }
                }
                // Avoid duplicates.
                if let Some(prev) = last_spike {
                    if best_idx < prev + refractory {
                        continue;
                    }
                }
                spikes.push(best_idx);
                last_spike = Some(best_idx);
            }
        }
    }
    spikes
}

// ---------------------------------------------------------------------------
// Template extraction (spike-triggered average)
// ---------------------------------------------------------------------------

/// Extract a MUAP template by spike-triggered averaging.
///
/// For each spike time, a window of `window_samples` centred on the spike is
/// extracted and the pointwise average across all windows is returned.
/// Spikes too close to the signal edges are skipped.
pub fn extract_template(
    signal: &[f64],
    spike_times: &[usize],
    window_samples: usize,
) -> Vec<f64> {
    if spike_times.is_empty() || window_samples == 0 || signal.is_empty() {
        return vec![0.0; window_samples];
    }
    let half = window_samples / 2;
    let mut template = vec![0.0; window_samples];
    let mut count = 0usize;

    for &t in spike_times {
        if t < half || t + half >= signal.len() {
            continue;
        }
        let start = t - half;
        for j in 0..window_samples {
            template[j] += signal[start + j];
        }
        count += 1;
    }
    if count > 0 {
        for v in &mut template {
            *v /= count as f64;
        }
    }
    template
}

// ---------------------------------------------------------------------------
// Template subtraction (peel-off)
// ---------------------------------------------------------------------------

/// Subtract a classified MUAP template from the signal at each firing time.
///
/// This "peel-off" step removes the contribution of one motor unit so the
/// next iteration can detect weaker units.
pub fn template_subtraction(
    signal: &mut [f64],
    template: &[f64],
    firing_times: &[usize],
) {
    let half = template.len() / 2;
    for &t in firing_times {
        if t < half {
            continue;
        }
        let start = t - half;
        for j in 0..template.len() {
            if start + j < signal.len() {
                signal[start + j] -= template[j];
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Firing-rate and ISI statistics
// ---------------------------------------------------------------------------

/// Compute the mean instantaneous firing rate in Hz.
///
/// If fewer than two spikes are present, returns 0.
pub fn compute_firing_rate(firing_times: &[usize], fs: f64) -> f64 {
    if firing_times.len() < 2 || fs <= 0.0 {
        return 0.0;
    }
    let n = firing_times.len();
    let mut sum_rate = 0.0;
    for i in 1..n {
        let isi_samples = firing_times[i] as f64 - firing_times[i - 1] as f64;
        if isi_samples > 0.0 {
            sum_rate += fs / isi_samples;
        }
    }
    sum_rate / (n - 1) as f64
}

/// Compute inter-spike interval statistics.
pub fn compute_isi_statistics(firing_times: &[usize], fs: f64) -> IsiStats {
    let empty = IsiStats {
        mean_ms: 0.0,
        std_ms: 0.0,
        cv: 0.0,
        min_ms: 0.0,
        max_ms: 0.0,
    };
    if firing_times.len() < 2 || fs <= 0.0 {
        return empty;
    }

    let isis: Vec<f64> = firing_times
        .windows(2)
        .map(|w| (w[1] as f64 - w[0] as f64) / fs * 1000.0)
        .collect();

    let n = isis.len() as f64;
    let mean = isis.iter().sum::<f64>() / n;
    let var = isis.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n;
    let std = var.sqrt();
    let min = isis.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = isis.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let cv = if mean > 0.0 { std / mean } else { 0.0 };

    IsiStats {
        mean_ms: mean,
        std_ms: std,
        cv,
        min_ms: min,
        max_ms: max,
    }
}

// ---------------------------------------------------------------------------
// Cross-correlation template matching
// ---------------------------------------------------------------------------

/// Sliding normalised cross-correlation between a template and a signal.
///
/// Returns `(position, correlation_coefficient)` pairs for every lag where
/// the local signal energy is nonzero. The correlation values are normalised
/// to the range \[-1, 1\].
pub fn cross_correlation_peak(template: &[f64], signal: &[f64]) -> Vec<(usize, f64)> {
    if template.is_empty() || signal.len() < template.len() {
        return Vec::new();
    }
    let m = template.len();
    let n = signal.len();

    // Template energy (constant).
    let template_energy: f64 = template.iter().map(|x| x * x).sum();
    if template_energy == 0.0 {
        return Vec::new();
    }
    let template_norm = template_energy.sqrt();

    let mut results = Vec::with_capacity(n - m + 1);
    for lag in 0..=(n - m) {
        let mut dot = 0.0;
        let mut sig_energy = 0.0;
        for j in 0..m {
            dot += template[j] * signal[lag + j];
            sig_energy += signal[lag + j] * signal[lag + j];
        }
        let sig_norm = sig_energy.sqrt();
        let ncc = if sig_norm > 1e-15 {
            dot / (template_norm * sig_norm)
        } else {
            0.0
        };
        results.push((lag, ncc));
    }
    results
}

// ---------------------------------------------------------------------------
// Whitening filter (AR-based)
// ---------------------------------------------------------------------------

/// AR-based spectral whitening via the Levinson-Durbin algorithm.
///
/// Fits an autoregressive model of the specified `order` to the signal, then
/// filters it with the prediction-error (whitening) coefficients.
pub fn whitening_filter(signal: &[f64], order: usize) -> Vec<f64> {
    if signal.is_empty() || order == 0 {
        return signal.to_vec();
    }
    let n = signal.len();
    let order = order.min(n - 1);

    // Compute autocorrelation r[0..=order].
    let mut r = vec![0.0; order + 1];
    for k in 0..=order {
        for i in k..n {
            r[k] += signal[i] * signal[i - k];
        }
        r[k] /= n as f64;
    }

    if r[0] <= 0.0 {
        return signal.to_vec();
    }

    // Levinson-Durbin recursion.
    let mut a = vec![0.0; order + 1]; // AR coefficients, a[0] = 1
    a[0] = 1.0;
    let mut e = r[0]; // prediction error

    for m in 1..=order {
        // Compute reflection coefficient k_m.
        let mut lambda = 0.0;
        for j in 1..m {
            lambda += a[j] * r[m - j];
        }
        lambda = (r[m] - lambda) / e;

        // Update coefficients.
        let mut a_new = a.clone();
        a_new[m] = lambda;
        for j in 1..m {
            a_new[j] = a[j] - lambda * a[m - j];
        }
        a = a_new;

        e *= 1.0 - lambda * lambda;
        if e <= 0.0 {
            break;
        }
    }

    // Apply the whitening (FIR) filter: y[n] = x[n] - sum(a[k]*x[n-k], k=1..order)
    let mut output = vec![0.0; n];
    for i in 0..n {
        let mut val = signal[i];
        for k in 1..=order {
            if i >= k {
                val -= a[k] * signal[i - k];
            }
        }
        output[i] = val;
    }
    output
}

// ---------------------------------------------------------------------------
// MUAP amplitude and duration
// ---------------------------------------------------------------------------

/// Peak-to-peak amplitude of a MUAP template.
pub fn compute_muap_amplitude(template: &[f64]) -> f64 {
    if template.is_empty() {
        return 0.0;
    }
    let max = template.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let min = template.iter().cloned().fold(f64::INFINITY, f64::min);
    max - min
}

/// Duration (in samples) of the portion of the template above `threshold_fraction`
/// of the peak absolute value.
pub fn compute_muap_duration(template: &[f64], threshold_fraction: f64) -> usize {
    if template.is_empty() {
        return 0;
    }
    let peak = template.iter().map(|x| x.abs()).fold(0.0f64, f64::max);
    let threshold = peak * threshold_fraction;
    let first = template.iter().position(|x| x.abs() >= threshold);
    let last = template.iter().rposition(|x| x.abs() >= threshold);
    match (first, last) {
        (Some(f), Some(l)) => l - f + 1,
        _ => 0,
    }
}

// ---------------------------------------------------------------------------
// Synthetic EMG generation
// ---------------------------------------------------------------------------

/// Generate a synthetic composite EMG signal from motor unit templates and firing rates.
///
/// Each element of `motor_units` is `(template, mean_firing_rate_hz)`. The function
/// places the template at pseudo-random intervals drawn from a Gaussian-like ISI
/// distribution centred on `1/rate` and sums all contributions.
pub fn generate_synthetic_emg(
    motor_units: &[(Vec<f64>, f64)],
    duration_s: f64,
    fs: f64,
) -> Vec<f64> {
    let n = (duration_s * fs) as usize;
    let mut signal = vec![0.0; n];

    // Simple deterministic PRNG (xorshift64).
    let mut rng_state: u64 = 0xDEAD_BEEF_CAFE_1234;
    let mut next_u64 = move || -> u64 {
        rng_state ^= rng_state << 13;
        rng_state ^= rng_state >> 7;
        rng_state ^= rng_state << 17;
        rng_state
    };

    for (template, rate) in motor_units {
        if *rate <= 0.0 || template.is_empty() {
            continue;
        }
        let mean_isi_samples = fs / rate;
        let half = template.len() / 2;

        // Generate firing times with jitter.
        let mut t = half;
        while t + half < n {
            // Place template centered at t.
            let start = if t >= half { t - half } else { 0 };
            for j in 0..template.len() {
                if start + j < n {
                    signal[start + j] += template[j];
                }
            }
            // Next firing: mean_isi + Gaussian-ish jitter (~5% CV).
            let u1 = (next_u64() as f64) / (u64::MAX as f64);
            let u2 = (next_u64() as f64) / (u64::MAX as f64);
            // Box-Muller transform for approximate Gaussian.
            let z = (-2.0 * u1.max(1e-15).ln()).sqrt() * (2.0 * PI * u2).cos();
            let jitter = 0.05 * mean_isi_samples * z;
            let isi = (mean_isi_samples + jitter).max(20.0) as usize;
            t += isi;
        }
    }
    signal
}

// ---------------------------------------------------------------------------
// Decomposer
// ---------------------------------------------------------------------------

/// EMG decomposition engine.
///
/// Iteratively extracts motor units from a composite EMG signal using a
/// detect-average-match-subtract loop.
#[derive(Debug, Clone)]
pub struct EmgDecomposer {
    config: DecompositionConfig,
}

impl EmgDecomposer {
    /// Create a new decomposer with the given configuration.
    pub fn new(config: DecompositionConfig) -> Self {
        Self { config }
    }

    /// Decompose one or more channels of EMG into individual motor units.
    ///
    /// For multi-channel data, channel 0 is used for decomposition (single-channel
    /// algorithm). Future revisions may implement spatial filtering.
    pub fn decompose(&self, emg_signals: &[Vec<f64>]) -> DecompositionResult {
        if emg_signals.is_empty() || emg_signals[0].is_empty() {
            return DecompositionResult {
                motor_units: Vec::new(),
                residual_energy: 0.0,
                reconstruction_quality: 0.0,
            };
        }

        let fs = self.config.sample_rate_hz;
        let window_samples =
            (self.config.template_duration_ms / 1000.0 * fs).round() as usize;
        let window_samples = window_samples.max(4);

        // Work on a copy (residual signal).
        let mut residual = emg_signals[0].clone();

        let original_energy: f64 = residual.iter().map(|x| x * x).sum();
        if original_energy == 0.0 {
            return DecompositionResult {
                motor_units: Vec::new(),
                residual_energy: 0.0,
                reconstruction_quality: 1.0,
            };
        }

        // Whitened copy for spike detection.
        let whitened = whitening_filter(&residual, 12);

        let mut motor_units: Vec<MotorUnit> = Vec::new();
        let max_iterations = 10;

        // Adaptive threshold: start at 3x RMS and decrease.
        let rms = (residual.iter().map(|x| x * x).sum::<f64>() / residual.len() as f64).sqrt();
        let mut threshold = 3.0 * rms;

        for _iter in 0..max_iterations {
            // Detect spikes on current residual.
            let spikes = detect_spikes(&residual, threshold);
            if spikes.len() < 3 {
                // Try lowering threshold once before giving up.
                threshold *= 0.7;
                let spikes2 = detect_spikes(&residual, threshold);
                if spikes2.len() < 3 {
                    break;
                }
                // Continue with lowered threshold.
                let template = extract_template(&residual, &spikes2, window_samples);
                let amp = compute_muap_amplitude(&template);
                if amp < 1e-12 {
                    break;
                }

                // Template matching on residual.
                let match_threshold = 0.7;
                let matched_times =
                    self.match_template(&residual, &template, match_threshold, window_samples);
                if matched_times.is_empty() {
                    break;
                }

                let fr = compute_firing_rate(&matched_times, fs);
                let isi = compute_isi_statistics(&matched_times, fs);

                // Physiological plausibility check.
                if fr >= self.config.min_firing_rate_hz
                    && fr <= self.config.max_firing_rate_hz
                {
                    template_subtraction(&mut residual, &template, &matched_times);
                    motor_units.push(MotorUnit {
                        template,
                        firing_times: matched_times,
                        mean_firing_rate_hz: fr,
                        cv_isi: isi.cv,
                    });
                }
                continue;
            }

            let template = extract_template(&residual, &spikes, window_samples);
            let amp = compute_muap_amplitude(&template);
            if amp < 1e-12 {
                threshold *= 0.7;
                continue;
            }

            // Template matching on residual.
            let match_threshold = 0.7;
            let matched_times =
                self.match_template(&residual, &template, match_threshold, window_samples);
            if matched_times.is_empty() {
                threshold *= 0.7;
                continue;
            }

            let fr = compute_firing_rate(&matched_times, fs);
            let isi = compute_isi_statistics(&matched_times, fs);

            // Physiological plausibility check.
            if fr >= self.config.min_firing_rate_hz && fr <= self.config.max_firing_rate_hz {
                template_subtraction(&mut residual, &template, &matched_times);
                motor_units.push(MotorUnit {
                    template,
                    firing_times: matched_times,
                    mean_firing_rate_hz: fr,
                    cv_isi: isi.cv,
                });
            }

            // Lower threshold for next, weaker motor unit.
            threshold *= 0.7;

            // Stop if residual is small.
            let res_energy: f64 = residual.iter().map(|x| x * x).sum();
            if res_energy < 0.05 * original_energy {
                break;
            }
        }

        // Sort by descending amplitude.
        motor_units.sort_by(|a, b| {
            let amp_a = compute_muap_amplitude(&a.template);
            let amp_b = compute_muap_amplitude(&b.template);
            amp_b.partial_cmp(&amp_a).unwrap_or(std::cmp::Ordering::Equal)
        });

        let residual_energy: f64 = residual.iter().map(|x| x * x).sum();
        let reconstruction_quality = 1.0 - residual_energy / original_energy;

        DecompositionResult {
            motor_units,
            residual_energy,
            reconstruction_quality: reconstruction_quality.clamp(0.0, 1.0),
        }
    }

    /// Find all positions where the template matches the signal above a
    /// correlation threshold, enforcing a minimum spacing (refractory period).
    fn match_template(
        &self,
        signal: &[f64],
        template: &[f64],
        corr_threshold: f64,
        _window_samples: usize,
    ) -> Vec<usize> {
        let cc = cross_correlation_peak(template, signal);
        let half = template.len() / 2;
        let min_spacing = (self.config.sample_rate_hz / self.config.max_firing_rate_hz) as usize;

        // Find peaks above threshold.
        let mut candidates: Vec<(usize, f64)> = cc
            .into_iter()
            .filter(|&(_, ncc)| ncc >= corr_threshold)
            .map(|(pos, ncc)| (pos + half, ncc)) // Convert to centre position.
            .collect();

        // Sort by descending correlation.
        candidates.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        // Greedy selection with minimum spacing.
        let mut selected: Vec<usize> = Vec::new();
        for (pos, _ncc) in candidates {
            let ok = selected.iter().all(|&s| {
                (pos as isize - s as isize).unsigned_abs() as usize >= min_spacing
            });
            if ok {
                selected.push(pos);
            }
        }
        selected.sort();
        selected
    }
}

// ---------------------------------------------------------------------------
// Helper: signal energy
// ---------------------------------------------------------------------------

fn signal_energy(signal: &[f64]) -> f64 {
    signal.iter().map(|x| x * x).sum()
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // Utility: Gaussian pulse centred at `centre` with given sigma.
    fn gaussian_pulse(len: usize, centre: f64, sigma: f64, amplitude: f64) -> Vec<f64> {
        (0..len)
            .map(|i| {
                let t = i as f64;
                amplitude * (-(t - centre).powi(2) / (2.0 * sigma * sigma)).exp()
            })
            .collect()
    }

    // Utility: place a template at given positions in a signal of length n.
    fn place_template(n: usize, template: &[f64], positions: &[usize]) -> Vec<f64> {
        let mut sig = vec![0.0; n];
        let half = template.len() / 2;
        for &p in positions {
            if p < half {
                continue;
            }
            let start = p - half;
            for j in 0..template.len() {
                if start + j < n {
                    sig[start + j] += template[j];
                }
            }
        }
        sig
    }

    // -----------------------------------------------------------------------
    // 1. Spike detection
    // -----------------------------------------------------------------------

    #[test]
    fn test_detect_spikes_basic() {
        let mut signal = vec![0.0; 1000];
        signal[100] = 5.0;
        signal[300] = 4.0;
        signal[600] = 6.0;
        let spikes = detect_spikes(&signal, 3.0);
        assert_eq!(spikes.len(), 3);
        assert!(spikes.contains(&100));
        assert!(spikes.contains(&300));
        assert!(spikes.contains(&600));
    }

    #[test]
    fn test_detect_spikes_refractory() {
        // Two peaks closer than refractory period: only one should be detected.
        let mut signal = vec![0.0; 200];
        signal[50] = 5.0;
        signal[55] = 4.5; // Within 20-sample refractory.
        let spikes = detect_spikes(&signal, 3.0);
        assert_eq!(spikes.len(), 1);
    }

    #[test]
    fn test_detect_spikes_negative() {
        // Should detect negative spikes too (absolute value).
        let mut signal = vec![0.0; 500];
        signal[100] = -5.0;
        signal[250] = -4.0;
        let spikes = detect_spikes(&signal, 3.0);
        assert_eq!(spikes.len(), 2);
    }

    #[test]
    fn test_detect_spikes_empty() {
        let spikes = detect_spikes(&[], 1.0);
        assert!(spikes.is_empty());
    }

    #[test]
    fn test_detect_spikes_no_above_threshold() {
        let signal = vec![0.1; 1000];
        let spikes = detect_spikes(&signal, 1.0);
        assert!(spikes.is_empty());
    }

    // -----------------------------------------------------------------------
    // 2. Template extraction
    // -----------------------------------------------------------------------

    #[test]
    fn test_extract_template_single_spike() {
        let template_ref = gaussian_pulse(50, 25.0, 5.0, 2.0);
        let signal = place_template(500, &template_ref, &[100]);
        let extracted = extract_template(&signal, &[100], 50);
        assert_eq!(extracted.len(), 50);
        // Peak should be near the centre.
        let peak_idx = extracted
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert!((peak_idx as isize - 25).unsigned_abs() <= 2);
    }

    #[test]
    fn test_extract_template_averaging() {
        // Multiple occurrences should average out noise.
        let template_ref = gaussian_pulse(40, 20.0, 4.0, 3.0);
        let positions = vec![100, 250, 400];
        let signal = place_template(600, &template_ref, &positions);
        let extracted = extract_template(&signal, &positions, 40);
        // The extracted template should closely match the original.
        let error: f64 = extracted
            .iter()
            .zip(&template_ref)
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            / template_ref.len() as f64;
        assert!(error < 0.01, "Template extraction error too large: {}", error);
    }

    #[test]
    fn test_extract_template_empty() {
        let t = extract_template(&[1.0, 2.0, 3.0], &[], 10);
        assert_eq!(t.len(), 10);
        assert!(t.iter().all(|&x| x == 0.0));
    }

    // -----------------------------------------------------------------------
    // 3. Template subtraction
    // -----------------------------------------------------------------------

    #[test]
    fn test_template_subtraction_removes_signal() {
        let template = gaussian_pulse(40, 20.0, 4.0, 5.0);
        let positions = vec![100, 300, 500];
        let mut signal = place_template(700, &template, &positions);
        let energy_before = signal_energy(&signal);
        template_subtraction(&mut signal, &template, &positions);
        let energy_after = signal_energy(&signal);
        assert!(
            energy_after < 0.01 * energy_before,
            "Subtraction did not remove signal: before={}, after={}",
            energy_before,
            energy_after
        );
    }

    #[test]
    fn test_template_subtraction_near_edges() {
        let template = vec![1.0; 10];
        let mut signal = vec![2.0; 20];
        // Firing time at 3 (half=5 so start would underflow).
        template_subtraction(&mut signal, &template, &[3]);
        // Should not panic; the firing at 3 is skipped since 3 < half(5).
        assert_eq!(signal.len(), 20);
    }

    // -----------------------------------------------------------------------
    // 4. Firing rate
    // -----------------------------------------------------------------------

    #[test]
    fn test_compute_firing_rate_known() {
        // Spikes every 1000 samples at 10 kHz -> 10 Hz firing rate.
        let ft: Vec<usize> = (0..10).map(|i| i * 1000).collect();
        let rate = compute_firing_rate(&ft, 10000.0);
        assert!((rate - 10.0).abs() < 0.01, "rate={}", rate);
    }

    #[test]
    fn test_compute_firing_rate_single_spike() {
        let rate = compute_firing_rate(&[42], 10000.0);
        assert_eq!(rate, 0.0);
    }

    #[test]
    fn test_compute_firing_rate_empty() {
        let rate = compute_firing_rate(&[], 10000.0);
        assert_eq!(rate, 0.0);
    }

    // -----------------------------------------------------------------------
    // 5. ISI statistics
    // -----------------------------------------------------------------------

    #[test]
    fn test_isi_statistics_uniform() {
        // Uniform ISI = 500 samples at 10 kHz -> 50 ms intervals.
        let ft: Vec<usize> = (0..20).map(|i| i * 500).collect();
        let stats = compute_isi_statistics(&ft, 10000.0);
        assert!((stats.mean_ms - 50.0).abs() < 0.01, "mean={}", stats.mean_ms);
        assert!(stats.std_ms < 0.01, "std={}", stats.std_ms);
        assert!(stats.cv < 0.001, "cv={}", stats.cv);
        assert!((stats.min_ms - 50.0).abs() < 0.01);
        assert!((stats.max_ms - 50.0).abs() < 0.01);
    }

    #[test]
    fn test_isi_statistics_variable() {
        let ft = vec![0, 400, 900, 1500, 2200];
        let stats = compute_isi_statistics(&ft, 10000.0);
        // ISIs in ms: 40, 50, 60, 70 -> mean=55, std~11.18
        assert!((stats.mean_ms - 55.0).abs() < 0.1);
        assert!(stats.std_ms > 10.0 && stats.std_ms < 13.0);
        assert!((stats.min_ms - 40.0).abs() < 0.1);
        assert!((stats.max_ms - 70.0).abs() < 0.1);
        assert!(stats.cv > 0.0);
    }

    #[test]
    fn test_isi_statistics_single() {
        let stats = compute_isi_statistics(&[100], 10000.0);
        assert_eq!(stats.mean_ms, 0.0);
    }

    // -----------------------------------------------------------------------
    // 6. Cross-correlation
    // -----------------------------------------------------------------------

    #[test]
    fn test_cross_correlation_perfect_match() {
        let template = gaussian_pulse(30, 15.0, 3.0, 1.0);
        // Place template at position 100 in a signal.
        let signal = place_template(300, &template, &[115]);
        let cc = cross_correlation_peak(&template, &signal);
        // Find the peak correlation.
        let (best_pos, best_val) = cc
            .iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();
        // Should be at lag=100 (template starts there) with correlation ~1.0.
        assert!(
            (*best_pos as isize - 100).unsigned_abs() <= 2,
            "best_pos={}, expected ~100",
            best_pos
        );
        assert!(*best_val > 0.95, "best_val={}", best_val);
    }

    #[test]
    fn test_cross_correlation_no_match() {
        let template = vec![1.0, -1.0, 1.0, -1.0];
        let signal = vec![0.0; 100];
        let cc = cross_correlation_peak(&template, &signal);
        // All correlations should be zero.
        for &(_, ncc) in &cc {
            assert!(ncc.abs() < 1e-10);
        }
    }

    #[test]
    fn test_cross_correlation_empty_template() {
        let cc = cross_correlation_peak(&[], &[1.0, 2.0, 3.0]);
        assert!(cc.is_empty());
    }

    #[test]
    fn test_cross_correlation_short_signal() {
        let cc = cross_correlation_peak(&[1.0, 2.0, 3.0], &[1.0]);
        assert!(cc.is_empty());
    }

    // -----------------------------------------------------------------------
    // 7. Whitening filter
    // -----------------------------------------------------------------------

    #[test]
    fn test_whitening_preserves_length() {
        let signal: Vec<f64> = (0..200).map(|i| (i as f64 * 0.1).sin()).collect();
        let whitened = whitening_filter(&signal, 10);
        assert_eq!(whitened.len(), signal.len());
    }

    #[test]
    fn test_whitening_reduces_autocorrelation() {
        // Coloured signal: low-pass filtered noise.
        let n = 2000;
        let mut signal = vec![0.0; n];
        let mut state: u64 = 42;
        for i in 0..n {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            let white = (state as f64 / u64::MAX as f64) * 2.0 - 1.0;
            signal[i] = if i > 0 {
                0.9 * signal[i - 1] + 0.1 * white
            } else {
                white
            };
        }

        let whitened = whitening_filter(&signal, 4);

        // Autocorrelation at lag 1 should be much smaller after whitening.
        let ac_orig = autocorrelation_lag1(&signal);
        let ac_white = autocorrelation_lag1(&whitened);
        assert!(
            ac_white.abs() < ac_orig.abs(),
            "Whitening should reduce autocorrelation: orig={}, whitened={}",
            ac_orig,
            ac_white
        );
    }

    #[test]
    fn test_whitening_empty() {
        let w = whitening_filter(&[], 5);
        assert!(w.is_empty());
    }

    #[test]
    fn test_whitening_order_zero() {
        let signal = vec![1.0, 2.0, 3.0];
        let w = whitening_filter(&signal, 0);
        assert_eq!(w, signal);
    }

    fn autocorrelation_lag1(signal: &[f64]) -> f64 {
        let n = signal.len();
        if n < 2 {
            return 0.0;
        }
        let mean: f64 = signal.iter().sum::<f64>() / n as f64;
        let mut num = 0.0;
        let mut den = 0.0;
        for i in 0..n {
            den += (signal[i] - mean).powi(2);
            if i > 0 {
                num += (signal[i] - mean) * (signal[i - 1] - mean);
            }
        }
        if den > 0.0 {
            num / den
        } else {
            0.0
        }
    }

    // -----------------------------------------------------------------------
    // 8. MUAP amplitude
    // -----------------------------------------------------------------------

    #[test]
    fn test_muap_amplitude_symmetric() {
        let t = gaussian_pulse(50, 25.0, 5.0, 3.0);
        let amp = compute_muap_amplitude(&t);
        // Gaussian only goes positive; min ~ 0, max ~ 3.0 -> amp ~ 3.0.
        assert!((amp - 3.0).abs() < 0.1, "amp={}", amp);
    }

    #[test]
    fn test_muap_amplitude_biphasic() {
        // Template that goes +2 then -1 -> peak-to-peak = 3.
        let mut template = vec![0.0; 40];
        template[10] = 2.0;
        template[20] = -1.0;
        let amp = compute_muap_amplitude(&template);
        assert!((amp - 3.0).abs() < 0.01);
    }

    #[test]
    fn test_muap_amplitude_empty() {
        assert_eq!(compute_muap_amplitude(&[]), 0.0);
    }

    // -----------------------------------------------------------------------
    // 9. MUAP duration
    // -----------------------------------------------------------------------

    #[test]
    fn test_muap_duration_gaussian() {
        let t = gaussian_pulse(100, 50.0, 5.0, 1.0);
        // At 10% threshold, the Gaussian should span roughly 30 samples.
        let dur = compute_muap_duration(&t, 0.1);
        assert!(dur > 15 && dur < 60, "dur={}", dur);
    }

    #[test]
    fn test_muap_duration_narrow() {
        let mut t = vec![0.0; 100];
        t[50] = 1.0;
        let dur = compute_muap_duration(&t, 0.5);
        assert_eq!(dur, 1);
    }

    #[test]
    fn test_muap_duration_empty() {
        assert_eq!(compute_muap_duration(&[], 0.1), 0);
    }

    // -----------------------------------------------------------------------
    // 10. Synthetic EMG generation
    // -----------------------------------------------------------------------

    #[test]
    fn test_generate_synthetic_emg_length() {
        let template = gaussian_pulse(50, 25.0, 5.0, 1.0);
        let emg = generate_synthetic_emg(&[(template, 15.0)], 1.0, 10000.0);
        assert_eq!(emg.len(), 10000);
    }

    #[test]
    fn test_generate_synthetic_emg_nonzero() {
        let template = gaussian_pulse(50, 25.0, 5.0, 1.0);
        let emg = generate_synthetic_emg(&[(template, 15.0)], 0.5, 10000.0);
        let energy = signal_energy(&emg);
        assert!(energy > 0.0, "Synthetic EMG should have nonzero energy");
    }

    #[test]
    fn test_generate_synthetic_emg_multiple_units() {
        let t1 = gaussian_pulse(50, 25.0, 5.0, 2.0);
        let t2 = gaussian_pulse(50, 25.0, 3.0, 1.0);
        let emg = generate_synthetic_emg(&[(t1, 10.0), (t2, 20.0)], 0.5, 10000.0);
        let energy = signal_energy(&emg);
        // Energy should be greater than a single unit.
        let t1b = gaussian_pulse(50, 25.0, 5.0, 2.0);
        let emg_single = generate_synthetic_emg(&[(t1b, 10.0)], 0.5, 10000.0);
        let energy_single = signal_energy(&emg_single);
        assert!(energy > energy_single);
    }

    #[test]
    fn test_generate_synthetic_emg_zero_rate() {
        let template = gaussian_pulse(50, 25.0, 5.0, 1.0);
        let emg = generate_synthetic_emg(&[(template, 0.0)], 0.5, 10000.0);
        assert!(signal_energy(&emg) == 0.0);
    }

    // -----------------------------------------------------------------------
    // 11. Decomposition pipeline
    // -----------------------------------------------------------------------

    #[test]
    fn test_decompose_single_unit() {
        let config = DecompositionConfig {
            sample_rate_hz: 10000.0,
            num_channels: 1,
            template_duration_ms: 10.0,
            min_firing_rate_hz: 5.0,
            max_firing_rate_hz: 40.0,
        };

        let template = gaussian_pulse(100, 50.0, 8.0, 5.0);
        let emg = generate_synthetic_emg(&[(template, 12.0)], 1.0, 10000.0);

        let decomposer = EmgDecomposer::new(config);
        let result = decomposer.decompose(&[emg]);

        assert!(
            !result.motor_units.is_empty(),
            "Should find at least one motor unit"
        );
        // Reconstruction quality should be positive.
        assert!(
            result.reconstruction_quality > 0.0,
            "rq={}",
            result.reconstruction_quality
        );
    }

    #[test]
    fn test_decompose_empty_signal() {
        let config = DecompositionConfig::default();
        let decomposer = EmgDecomposer::new(config);
        let result = decomposer.decompose(&[vec![]]);
        assert!(result.motor_units.is_empty());
    }

    #[test]
    fn test_decompose_silent_signal() {
        let config = DecompositionConfig::default();
        let decomposer = EmgDecomposer::new(config);
        let result = decomposer.decompose(&[vec![0.0; 5000]]);
        assert!(result.motor_units.is_empty());
        assert_eq!(result.reconstruction_quality, 1.0);
    }

    #[test]
    fn test_decompose_no_channels() {
        let config = DecompositionConfig::default();
        let decomposer = EmgDecomposer::new(config);
        let result = decomposer.decompose(&[]);
        assert!(result.motor_units.is_empty());
    }

    // -----------------------------------------------------------------------
    // 12. Config default values
    // -----------------------------------------------------------------------

    #[test]
    fn test_config_defaults() {
        let config = DecompositionConfig::default();
        assert_eq!(config.sample_rate_hz, 10000.0);
        assert_eq!(config.num_channels, 1);
        assert_eq!(config.template_duration_ms, 10.0);
        assert_eq!(config.min_firing_rate_hz, 6.0);
        assert_eq!(config.max_firing_rate_hz, 35.0);
    }

    // -----------------------------------------------------------------------
    // 13. Signal energy helper
    // -----------------------------------------------------------------------

    #[test]
    fn test_signal_energy() {
        assert_eq!(signal_energy(&[]), 0.0);
        assert!((signal_energy(&[3.0, 4.0]) - 25.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // 14. Cross-correlation multiple peaks
    // -----------------------------------------------------------------------

    #[test]
    fn test_cross_correlation_multiple_peaks() {
        let template = gaussian_pulse(20, 10.0, 2.0, 1.0);
        let signal = place_template(500, &template, &[50, 200, 350]);
        let cc = cross_correlation_peak(&template, &signal);
        // Collect peaks above 0.9.
        let peaks: Vec<usize> = cc
            .iter()
            .filter(|&&(_, ncc)| ncc > 0.9)
            .map(|&(pos, _)| pos)
            .collect();
        // Should find peaks near lags 40, 190, 340 (offset by half template).
        assert!(peaks.len() >= 3, "Found {} peaks", peaks.len());
    }

    // -----------------------------------------------------------------------
    // 15. Template subtraction completeness
    // -----------------------------------------------------------------------

    #[test]
    fn test_template_subtraction_complete() {
        let template = vec![1.0, 2.0, 3.0, 2.0, 1.0];
        let positions = vec![50, 100, 150];
        let mut signal = place_template(200, &template, &positions);
        template_subtraction(&mut signal, &template, &positions);
        // Residual should be essentially zero.
        let max_abs = signal.iter().map(|x| x.abs()).fold(0.0f64, f64::max);
        assert!(max_abs < 1e-10, "max residual={}", max_abs);
    }

    // -----------------------------------------------------------------------
    // 16. Firing rate edge cases
    // -----------------------------------------------------------------------

    #[test]
    fn test_firing_rate_zero_fs() {
        assert_eq!(compute_firing_rate(&[0, 100], 0.0), 0.0);
    }

    #[test]
    fn test_firing_rate_negative_fs() {
        assert_eq!(compute_firing_rate(&[0, 100], -1.0), 0.0);
    }

    // -----------------------------------------------------------------------
    // 17. ISI edge cases
    // -----------------------------------------------------------------------

    #[test]
    fn test_isi_zero_fs() {
        let stats = compute_isi_statistics(&[0, 100], 0.0);
        assert_eq!(stats.mean_ms, 0.0);
    }

    // -----------------------------------------------------------------------
    // 18. MUAP duration at various thresholds
    // -----------------------------------------------------------------------

    #[test]
    fn test_muap_duration_high_threshold() {
        let template = gaussian_pulse(100, 50.0, 5.0, 1.0);
        let dur_high = compute_muap_duration(&template, 0.9);
        let dur_low = compute_muap_duration(&template, 0.1);
        assert!(dur_high < dur_low, "Higher threshold should give shorter duration");
    }

    // -----------------------------------------------------------------------
    // 19. Whitening on constant signal
    // -----------------------------------------------------------------------

    #[test]
    fn test_whitening_constant_signal() {
        let signal = vec![5.0; 100];
        let w = whitening_filter(&signal, 4);
        assert_eq!(w.len(), 100);
        // After whitening a constant signal, the predictor should remove most
        // of the energy. Allow for numerical imprecision in Levinson-Durbin.
        let orig_energy: f64 = signal.iter().map(|x| x * x).sum();
        let white_energy: f64 = w[4..].iter().map(|x| x * x).sum();
        assert!(
            white_energy < 0.01 * orig_energy,
            "Whitened energy should be much smaller: white={}, orig={}",
            white_energy,
            orig_energy,
        );
    }

    // -----------------------------------------------------------------------
    // 20. Decompose reconstruction quality range
    // -----------------------------------------------------------------------

    #[test]
    fn test_decompose_quality_range() {
        let config = DecompositionConfig {
            sample_rate_hz: 10000.0,
            num_channels: 1,
            template_duration_ms: 10.0,
            min_firing_rate_hz: 5.0,
            max_firing_rate_hz: 40.0,
        };
        let template = gaussian_pulse(100, 50.0, 8.0, 4.0);
        let emg = generate_synthetic_emg(&[(template, 15.0)], 0.5, 10000.0);
        let decomposer = EmgDecomposer::new(config);
        let result = decomposer.decompose(&[emg]);
        assert!(result.reconstruction_quality >= 0.0);
        assert!(result.reconstruction_quality <= 1.0);
    }

    // -----------------------------------------------------------------------
    // 21. Motor unit cv_isi non-negative
    // -----------------------------------------------------------------------

    #[test]
    fn test_motor_unit_cv_isi_nonnegative() {
        let config = DecompositionConfig {
            sample_rate_hz: 10000.0,
            num_channels: 1,
            template_duration_ms: 10.0,
            min_firing_rate_hz: 5.0,
            max_firing_rate_hz: 40.0,
        };
        let template = gaussian_pulse(100, 50.0, 8.0, 5.0);
        let emg = generate_synthetic_emg(&[(template, 12.0)], 1.0, 10000.0);
        let decomposer = EmgDecomposer::new(config);
        let result = decomposer.decompose(&[emg]);
        for mu in &result.motor_units {
            assert!(mu.cv_isi >= 0.0, "cv_isi should be non-negative");
        }
    }

    // -----------------------------------------------------------------------
    // 22. Cross-correlation symmetry
    // -----------------------------------------------------------------------

    #[test]
    fn test_cross_correlation_self_match() {
        let template = gaussian_pulse(30, 15.0, 3.0, 1.0);
        // Template as the signal: perfect match at lag 0.
        let cc = cross_correlation_peak(&template, &template);
        assert!(!cc.is_empty());
        let (pos, val) = cc[0];
        assert_eq!(pos, 0);
        assert!((val - 1.0).abs() < 1e-10, "Self-correlation should be 1.0, got {}", val);
    }

    // -----------------------------------------------------------------------
    // 23. Generate synthetic EMG reproducibility
    // -----------------------------------------------------------------------

    #[test]
    fn test_generate_synthetic_emg_deterministic() {
        let template = gaussian_pulse(50, 25.0, 5.0, 1.0);
        let emg1 = generate_synthetic_emg(&[(template.clone(), 15.0)], 0.5, 10000.0);
        let emg2 = generate_synthetic_emg(&[(template, 15.0)], 0.5, 10000.0);
        // Same seed -> same output.
        assert_eq!(emg1.len(), emg2.len());
        for (a, b) in emg1.iter().zip(&emg2) {
            assert!((a - b).abs() < 1e-15);
        }
    }

    // -----------------------------------------------------------------------
    // 24. Detect spikes peak refinement
    // -----------------------------------------------------------------------

    #[test]
    fn test_detect_spikes_peak_refinement() {
        let mut signal = vec![0.0; 500];
        // Ramp up to a peak at 102, but threshold crossed at 100.
        signal[99] = 2.0;
        signal[100] = 3.5;
        signal[101] = 4.0;
        signal[102] = 4.5; // True peak.
        signal[103] = 3.8;
        signal[104] = 2.5;
        let spikes = detect_spikes(&signal, 3.0);
        assert_eq!(spikes.len(), 1);
        assert_eq!(spikes[0], 102, "Should refine to the local maximum");
    }

    // -----------------------------------------------------------------------
    // 25. ISI statistics CV validation
    // -----------------------------------------------------------------------

    #[test]
    fn test_isi_cv_is_std_over_mean() {
        let ft = vec![0, 300, 700, 1200, 1800, 2500];
        let stats = compute_isi_statistics(&ft, 10000.0);
        let expected_cv = stats.std_ms / stats.mean_ms;
        assert!(
            (stats.cv - expected_cv).abs() < 1e-10,
            "CV should be std/mean: {} vs {}",
            stats.cv,
            expected_cv
        );
    }

    // -----------------------------------------------------------------------
    // 26. Whitening high order
    // -----------------------------------------------------------------------

    #[test]
    fn test_whitening_high_order_no_panic() {
        let signal = vec![1.0, 0.5, -0.3, 0.2, -0.1];
        // Order larger than signal: should clamp gracefully.
        let w = whitening_filter(&signal, 100);
        assert_eq!(w.len(), 5);
    }

    // -----------------------------------------------------------------------
    // 27. MUAP amplitude all zeros
    // -----------------------------------------------------------------------

    #[test]
    fn test_muap_amplitude_all_zeros() {
        let t = vec![0.0; 50];
        assert_eq!(compute_muap_amplitude(&t), 0.0);
    }

    // -----------------------------------------------------------------------
    // 28. Template extraction edge spikes skipped
    // -----------------------------------------------------------------------

    #[test]
    fn test_extract_template_edge_spikes_skipped() {
        let signal = vec![1.0; 100];
        // Spike at index 2 with window 20 (half=10): index 2 < 10, should be skipped.
        // Spike at index 98 with window 20 (half=10): 98+10 >= 100, should be skipped.
        let t = extract_template(&signal, &[2, 98], 20);
        // Both are skipped: result should be zeros (no averages).
        assert!(t.iter().all(|&x| x == 0.0));
    }
}
