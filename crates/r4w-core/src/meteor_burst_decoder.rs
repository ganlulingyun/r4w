//! # Meteor Burst (Meteor Scatter) Decoder
//!
//! Detection, synchronization, and demodulation of meteor scatter reflections
//! for VHF/UHF communications. Meteor burst communication exploits the
//! ionized trails left by meteoroids entering Earth's atmosphere (typically at
//! 80–120 km altitude) to reflect VHF/UHF signals far beyond line-of-sight.
//!
//! This module provides:
//!
//! - **Burst detection** via energy-envelope thresholding
//! - **Burst duration estimation** (typical 0.1–10 s)
//! - **Doppler shift estimation** during the trail reflection
//! - **Simple FSK / BPSK demodulation** of burst data
//! - **Trail decay modelling** (exponential amplitude decay)
//! - **Link availability estimation** from burst statistics
//! - **Burst quality metrics** (SNR, duration, Doppler stability)
//! - **Underdense vs. overdense trail classification**
//!
//! Complex samples are represented as `(f64, f64)` tuples `(re, im)`.
//!
//! # Example
//!
//! ```
//! use r4w_core::meteor_burst_decoder::{MeteorBurstDecoder, MeteorBurstConfig};
//!
//! let config = MeteorBurstConfig {
//!     carrier_freq_hz: 50.0e6,
//!     sample_rate_hz: 48_000.0,
//!     detection_threshold_db: 6.0,
//!     min_burst_duration_s: 0.05,
//!     max_burst_duration_s: 15.0,
//! };
//! let mut decoder = MeteorBurstDecoder::new(config);
//!
//! // Build signal: silence + tone burst + silence
//! let mut samples: Vec<(f64, f64)> = vec![(0.001, 0.001); 4800]; // 0.1s noise floor
//! for i in 0..4800 { // 0.1s burst
//!     let t = i as f64 / 48_000.0;
//!     let phase = 2.0 * std::f64::consts::PI * 1000.0 * t;
//!     samples.push((phase.cos() * 5.0, phase.sin() * 5.0));
//! }
//! samples.extend(vec![(0.001, 0.001); 4800]); // 0.1s noise floor
//!
//! let bursts = decoder.detect_bursts(&samples);
//! assert!(!bursts.is_empty(), "should detect the tone burst");
//! ```

use std::f64::consts::PI;

// ─── configuration ───────────────────────────────────────────────────────────

/// Configuration parameters for the meteor-burst decoder.
#[derive(Debug, Clone)]
pub struct MeteorBurstConfig {
    /// Nominal carrier frequency in Hz (e.g. 50 MHz).
    pub carrier_freq_hz: f64,
    /// Sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Detection threshold in dB above the noise floor.
    pub detection_threshold_db: f64,
    /// Minimum burst duration in seconds to accept.
    pub min_burst_duration_s: f64,
    /// Maximum burst duration in seconds to accept.
    pub max_burst_duration_s: f64,
}

impl Default for MeteorBurstConfig {
    fn default() -> Self {
        Self {
            carrier_freq_hz: 50.0e6,
            sample_rate_hz: 48_000.0,
            detection_threshold_db: 6.0,
            min_burst_duration_s: 0.05,
            max_burst_duration_s: 15.0,
        }
    }
}

// ─── trail classification ────────────────────────────────────────────────────

/// Classification of a meteor trail.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrailType {
    /// Underdense trail – electron line density < ~2×10¹² e⁻/m.
    /// Short duration, smooth exponential decay.
    Underdense,
    /// Overdense trail – electron line density > ~2×10¹² e⁻/m.
    /// Longer duration, plateau then irregular decay.
    Overdense,
}

// ─── burst descriptor ────────────────────────────────────────────────────────

/// Describes a single detected meteor-scatter burst.
#[derive(Debug, Clone)]
pub struct BurstDescriptor {
    /// Index of the first sample in the burst.
    pub start_sample: usize,
    /// Index one past the last sample in the burst.
    pub end_sample: usize,
    /// Burst duration in seconds.
    pub duration_s: f64,
    /// Peak signal-to-noise ratio in dB.
    pub peak_snr_db: f64,
    /// Estimated Doppler shift in Hz (positive = closing).
    pub doppler_shift_hz: f64,
    /// Standard deviation of instantaneous Doppler over the burst, in Hz.
    pub doppler_stability_hz: f64,
    /// Estimated exponential decay time-constant (seconds). Positive.
    pub decay_tau_s: f64,
    /// Trail classification.
    pub trail_type: TrailType,
}

// ─── burst quality ───────────────────────────────────────────────────────────

/// Aggregate quality metric for a burst.
#[derive(Debug, Clone)]
pub struct BurstQuality {
    /// SNR in dB.
    pub snr_db: f64,
    /// Duration in seconds.
    pub duration_s: f64,
    /// Doppler stability (lower = better) in Hz.
    pub doppler_stability_hz: f64,
    /// Overall quality score in [0, 1].
    pub quality_score: f64,
}

// ─── link availability ──────────────────────────────────────────────────────

/// Link-availability statistics derived from observed bursts.
#[derive(Debug, Clone)]
pub struct LinkAvailability {
    /// Total observation time in seconds.
    pub observation_time_s: f64,
    /// Number of usable bursts detected.
    pub burst_count: usize,
    /// Fraction of time the link was available (0–1).
    pub duty_cycle: f64,
    /// Mean burst duration in seconds.
    pub mean_burst_duration_s: f64,
    /// Mean gap between bursts in seconds.
    pub mean_gap_s: f64,
}

// ─── decoder ─────────────────────────────────────────────────────────────────

/// Meteor-burst detector, synchroniser, and demodulator.
pub struct MeteorBurstDecoder {
    config: MeteorBurstConfig,
    /// History of detected bursts (for link-availability estimation).
    burst_history: Vec<BurstDescriptor>,
}

impl MeteorBurstDecoder {
    /// Create a new decoder with the given configuration.
    pub fn new(config: MeteorBurstConfig) -> Self {
        Self {
            config,
            burst_history: Vec::new(),
        }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &MeteorBurstConfig {
        &self.config
    }

    // ── energy envelope ──────────────────────────────────────────────────

    /// Compute the energy envelope (magnitude squared) of complex samples.
    pub fn energy_envelope(samples: &[(f64, f64)]) -> Vec<f64> {
        samples.iter().map(|&(re, im)| re * re + im * im).collect()
    }

    /// Smooth an envelope with a simple moving-average of `window` samples.
    pub fn smooth_envelope(envelope: &[f64], window: usize) -> Vec<f64> {
        if window == 0 || envelope.is_empty() {
            return envelope.to_vec();
        }
        let w = window.max(1);
        let n = envelope.len();
        let mut out = vec![0.0; n];
        let mut sum = 0.0;
        for i in 0..n {
            sum += envelope[i];
            if i >= w {
                sum -= envelope[i - w];
            }
            let count = (i + 1).min(w) as f64;
            out[i] = sum / count;
        }
        out
    }

    /// Estimate the noise floor as the median of the envelope.
    pub fn estimate_noise_floor(envelope: &[f64]) -> f64 {
        if envelope.is_empty() {
            return 0.0;
        }
        let mut sorted = envelope.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mid = sorted.len() / 2;
        if sorted.len() % 2 == 0 && sorted.len() >= 2 {
            (sorted[mid - 1] + sorted[mid]) / 2.0
        } else {
            sorted[mid]
        }
    }

    // ── burst detection ──────────────────────────────────────────────────

    /// Detect meteor-scatter bursts in the sample buffer.
    ///
    /// Returns a list of [`BurstDescriptor`]s for every burst that exceeds the
    /// detection threshold and satisfies the duration constraints.
    pub fn detect_bursts(&mut self, samples: &[(f64, f64)]) -> Vec<BurstDescriptor> {
        if samples.is_empty() {
            return Vec::new();
        }

        let sr = self.config.sample_rate_hz;
        // Smoothing window: ~1 ms
        let smooth_window = (sr * 0.001).max(1.0) as usize;
        let envelope = Self::energy_envelope(samples);
        let smoothed = Self::smooth_envelope(&envelope, smooth_window);
        let noise_floor = Self::estimate_noise_floor(&smoothed);

        // Threshold in linear power
        let threshold_linear = if noise_floor > 0.0 {
            noise_floor * 10.0_f64.powf(self.config.detection_threshold_db / 10.0)
        } else {
            // Fallback: use absolute threshold from sorted values
            let mut sorted = smoothed.clone();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            sorted[sorted.len() * 3 / 4] // 75th percentile
        };

        let min_samples = (self.config.min_burst_duration_s * sr) as usize;
        let max_samples = (self.config.max_burst_duration_s * sr) as usize;

        let mut bursts = Vec::new();
        let mut i = 0;
        let n = smoothed.len();
        while i < n {
            if smoothed[i] > threshold_linear {
                let start = i;
                while i < n && smoothed[i] > threshold_linear {
                    i += 1;
                }
                let end = i;
                let len = end - start;
                if len >= min_samples && len <= max_samples {
                    let burst_samples = &samples[start..end];
                    let burst_envelope = &smoothed[start..end];

                    let peak_power = burst_envelope
                        .iter()
                        .cloned()
                        .fold(0.0_f64, f64::max);
                    let peak_snr_db = if noise_floor > 0.0 {
                        10.0 * (peak_power / noise_floor).log10()
                    } else {
                        30.0 // assume high SNR
                    };

                    let doppler_shift_hz = self.estimate_doppler(burst_samples);
                    let doppler_stability_hz = self.estimate_doppler_stability(burst_samples);
                    let decay_tau_s = self.estimate_decay_tau(burst_envelope, sr);
                    let duration_s = len as f64 / sr;
                    let trail_type = Self::classify_trail(duration_s, burst_envelope);

                    let desc = BurstDescriptor {
                        start_sample: start,
                        end_sample: end,
                        duration_s,
                        peak_snr_db,
                        doppler_shift_hz,
                        doppler_stability_hz,
                        decay_tau_s,
                        trail_type,
                    };
                    bursts.push(desc);
                }
            } else {
                i += 1;
            }
        }

        self.burst_history.extend(bursts.iter().cloned());
        bursts
    }

    // ── Doppler estimation ───────────────────────────────────────────────

    /// Estimate the mean Doppler shift (in Hz) of a burst using the
    /// average instantaneous-frequency estimator:
    ///   f_inst[n] = (1 / 2π) × arg{ x[n] × conj(x[n-1]) } × sample_rate
    pub fn estimate_doppler(&self, burst: &[(f64, f64)]) -> f64 {
        if burst.len() < 2 {
            return 0.0;
        }
        let sr = self.config.sample_rate_hz;
        let mut sum = 0.0;
        let count = burst.len() - 1;
        for i in 1..burst.len() {
            let (re0, im0) = burst[i - 1];
            let (re1, im1) = burst[i];
            // x[n] * conj(x[n-1])
            let prod_re = re1 * re0 + im1 * im0;
            let prod_im = im1 * re0 - re1 * im0;
            sum += prod_im.atan2(prod_re);
        }
        let mean_phase_diff = sum / count as f64;
        mean_phase_diff * sr / (2.0 * PI)
    }

    /// Estimate Doppler stability (std-dev of instantaneous frequency) in Hz.
    pub fn estimate_doppler_stability(&self, burst: &[(f64, f64)]) -> f64 {
        if burst.len() < 3 {
            return 0.0;
        }
        let sr = self.config.sample_rate_hz;
        let n = burst.len() - 1;
        let mut freqs = Vec::with_capacity(n);
        for i in 1..burst.len() {
            let (re0, im0) = burst[i - 1];
            let (re1, im1) = burst[i];
            let prod_re = re1 * re0 + im1 * im0;
            let prod_im = im1 * re0 - re1 * im0;
            let f = prod_im.atan2(prod_re) * sr / (2.0 * PI);
            freqs.push(f);
        }
        let mean = freqs.iter().sum::<f64>() / freqs.len() as f64;
        let var = freqs.iter().map(|f| (f - mean).powi(2)).sum::<f64>() / freqs.len() as f64;
        var.sqrt()
    }

    // ── decay estimation ─────────────────────────────────────────────────

    /// Fit an exponential decay time-constant τ to a power envelope.
    ///
    /// Models the envelope as  A·exp(−t/τ) from the peak onward.
    /// Uses a simple log-linear least-squares fit.
    pub fn estimate_decay_tau(&self, envelope: &[f64], sample_rate: f64) -> f64 {
        if envelope.len() < 2 {
            return 1.0;
        }
        // Find peak index
        let peak_idx = envelope
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(i, _)| i)
            .unwrap_or(0);

        let tail = &envelope[peak_idx..];
        if tail.len() < 2 {
            return 1.0;
        }
        let peak_val = tail[0];
        if peak_val <= 0.0 {
            return 1.0;
        }

        // Log-linear fit: ln(env) = ln(A) - t/τ  ⟹  slope = −1/τ
        let mut sum_t = 0.0;
        let mut sum_y = 0.0;
        let mut sum_ty = 0.0;
        let mut sum_tt = 0.0;
        let mut count = 0usize;
        for (j, &val) in tail.iter().enumerate() {
            if val > peak_val * 0.01 {
                // only fit where signal is meaningful
                let t = j as f64 / sample_rate;
                let y = (val / peak_val).ln();
                sum_t += t;
                sum_y += y;
                sum_ty += t * y;
                sum_tt += t * t;
                count += 1;
            }
        }
        if count < 2 {
            return 1.0;
        }
        let n = count as f64;
        let denom = n * sum_tt - sum_t * sum_t;
        if denom.abs() < 1e-30 {
            return 1.0;
        }
        let slope = (n * sum_ty - sum_t * sum_y) / denom;
        if slope >= 0.0 {
            // non-decaying or rising – return large tau
            return 10.0;
        }
        let tau = -1.0 / slope;
        tau.clamp(0.001, 100.0)
    }

    // ── trail classification ─────────────────────────────────────────────

    /// Classify a trail as underdense or overdense.
    ///
    /// Heuristic: overdense trails have a plateau (the first half of the
    /// envelope stays within ~3 dB of the peak), while underdense trails
    /// decay immediately from the peak.
    pub fn classify_trail(duration_s: f64, envelope: &[f64]) -> TrailType {
        // Short bursts (< 0.5 s) are almost always underdense.
        if duration_s < 0.5 {
            return TrailType::Underdense;
        }
        if envelope.is_empty() {
            return TrailType::Underdense;
        }
        let peak = envelope
            .iter()
            .cloned()
            .fold(0.0_f64, f64::max);
        if peak <= 0.0 {
            return TrailType::Underdense;
        }
        // Check if the first half stays within 3 dB of peak
        let half = envelope.len() / 2;
        let threshold = peak * 0.5; // −3 dB in power
        let plateau_frac = envelope[..half]
            .iter()
            .filter(|&&v| v >= threshold)
            .count() as f64
            / half.max(1) as f64;
        if plateau_frac > 0.7 {
            TrailType::Overdense
        } else {
            TrailType::Underdense
        }
    }

    // ── burst quality ────────────────────────────────────────────────────

    /// Compute a quality metric for a detected burst.
    pub fn burst_quality(burst: &BurstDescriptor) -> BurstQuality {
        // Quality score: weighted combination of SNR, duration, and Doppler stability.
        // Each factor is normalised to [0,1] with soft saturation.
        let snr_factor = (burst.peak_snr_db / 20.0).clamp(0.0, 1.0); // 20 dB → 1.0
        let dur_factor = (burst.duration_s / 2.0).clamp(0.0, 1.0); // 2 s → 1.0
        let doppler_factor = 1.0 - (burst.doppler_stability_hz / 200.0).clamp(0.0, 1.0);
        let score = 0.5 * snr_factor + 0.3 * dur_factor + 0.2 * doppler_factor;
        BurstQuality {
            snr_db: burst.peak_snr_db,
            duration_s: burst.duration_s,
            doppler_stability_hz: burst.doppler_stability_hz,
            quality_score: score.clamp(0.0, 1.0),
        }
    }

    // ── link availability ────────────────────────────────────────────────

    /// Estimate link availability from accumulated burst history.
    ///
    /// `total_observation_s` is the total wall-clock time spanned by
    /// all calls to [`detect_bursts`](Self::detect_bursts).
    pub fn link_availability(&self, total_observation_s: f64) -> LinkAvailability {
        let burst_count = self.burst_history.len();
        if burst_count == 0 {
            return LinkAvailability {
                observation_time_s: total_observation_s,
                burst_count: 0,
                duty_cycle: 0.0,
                mean_burst_duration_s: 0.0,
                mean_gap_s: total_observation_s,
            };
        }
        let total_burst_time: f64 = self.burst_history.iter().map(|b| b.duration_s).sum();
        let duty = if total_observation_s > 0.0 {
            (total_burst_time / total_observation_s).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let mean_dur = total_burst_time / burst_count as f64;
        let gap_time = (total_observation_s - total_burst_time).max(0.0);
        let mean_gap = if burst_count > 1 {
            gap_time / (burst_count - 1) as f64
        } else {
            gap_time
        };
        LinkAvailability {
            observation_time_s: total_observation_s,
            burst_count,
            duty_cycle: duty,
            mean_burst_duration_s: mean_dur,
            mean_gap_s: mean_gap,
        }
    }

    /// Clear the internal burst history.
    pub fn reset_history(&mut self) {
        self.burst_history.clear();
    }

    /// Return the burst history accumulated so far.
    pub fn burst_history(&self) -> &[BurstDescriptor] {
        &self.burst_history
    }

    // ── exponential trail model ──────────────────────────────────────────

    /// Generate an exponential-decay trail amplitude envelope.
    ///
    /// `peak_amplitude` – amplitude at the start of the trail.
    /// `tau_s`          – decay time-constant in seconds.
    /// `duration_s`     – total duration to model.
    /// `sample_rate`    – samples per second.
    pub fn trail_decay_model(
        peak_amplitude: f64,
        tau_s: f64,
        duration_s: f64,
        sample_rate: f64,
    ) -> Vec<f64> {
        let n = (duration_s * sample_rate).ceil() as usize;
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                peak_amplitude * (-t / tau_s).exp()
            })
            .collect()
    }

    // ── FSK demodulation ─────────────────────────────────────────────────

    /// Demodulate a burst assuming binary FSK.
    ///
    /// `burst`          – complex samples of the burst
    /// `symbol_rate_hz` – baud rate
    /// `freq_deviation` – Hz deviation from carrier for mark/space
    ///
    /// Returns demodulated bits.
    pub fn demodulate_fsk(
        &self,
        burst: &[(f64, f64)],
        symbol_rate_hz: f64,
        freq_deviation: f64,
    ) -> Vec<bool> {
        if burst.is_empty() || symbol_rate_hz <= 0.0 {
            return Vec::new();
        }
        let sr = self.config.sample_rate_hz;
        let samples_per_symbol = (sr / symbol_rate_hz).round() as usize;
        if samples_per_symbol < 2 {
            return Vec::new();
        }

        let num_symbols = burst.len() / samples_per_symbol;
        let mut bits = Vec::with_capacity(num_symbols);

        for sym_idx in 0..num_symbols {
            let start = sym_idx * samples_per_symbol;
            let end = start + samples_per_symbol;
            let symbol_samples = &burst[start..end];

            // Correlate with mark (+deviation) and space (−deviation)
            let mark_energy = self.correlate_tone(symbol_samples, freq_deviation);
            let space_energy = self.correlate_tone(symbol_samples, -freq_deviation);

            bits.push(mark_energy > space_energy);
        }
        bits
    }

    /// Correlate a segment with a tone at the given frequency offset.
    fn correlate_tone(&self, samples: &[(f64, f64)], freq_hz: f64) -> f64 {
        let sr = self.config.sample_rate_hz;
        let mut acc_re = 0.0;
        let mut acc_im = 0.0;
        for (i, &(re, im)) in samples.iter().enumerate() {
            let t = i as f64 / sr;
            let phase = -2.0 * PI * freq_hz * t;
            let (sin_p, cos_p) = phase.sin_cos();
            acc_re += re * cos_p - im * sin_p;
            acc_im += re * sin_p + im * cos_p;
        }
        acc_re * acc_re + acc_im * acc_im
    }

    // ── BPSK demodulation ────────────────────────────────────────────────

    /// Demodulate a burst assuming BPSK (carrier-coherent).
    ///
    /// Assumes the carrier has already been removed (baseband).
    /// `symbol_rate_hz` – baud rate.
    ///
    /// Returns demodulated bits.
    pub fn demodulate_bpsk(
        &self,
        burst: &[(f64, f64)],
        symbol_rate_hz: f64,
    ) -> Vec<bool> {
        if burst.is_empty() || symbol_rate_hz <= 0.0 {
            return Vec::new();
        }
        let sr = self.config.sample_rate_hz;
        let samples_per_symbol = (sr / symbol_rate_hz).round() as usize;
        if samples_per_symbol < 1 {
            return Vec::new();
        }

        let num_symbols = burst.len() / samples_per_symbol;
        let mut bits = Vec::with_capacity(num_symbols);

        for sym_idx in 0..num_symbols {
            let start = sym_idx * samples_per_symbol;
            let end = start + samples_per_symbol;
            // Integrate the real component over the symbol period
            let sum_re: f64 = burst[start..end].iter().map(|&(re, _)| re).sum();
            bits.push(sum_re >= 0.0);
        }
        bits
    }

    // ── helper: generate test burst ──────────────────────────────────────

    /// Synthesise a complex sinusoidal burst at `freq_hz` with exponential
    /// decay and additive noise.  Useful for testing.
    ///
    /// Returns a vector of `(re, im)` samples surrounded by `gap_samples`
    /// of noise on each side.
    pub fn generate_test_burst(
        sample_rate: f64,
        freq_hz: f64,
        amplitude: f64,
        decay_tau_s: f64,
        duration_s: f64,
        noise_amplitude: f64,
        gap_samples: usize,
    ) -> Vec<(f64, f64)> {
        let burst_len = (duration_s * sample_rate).ceil() as usize;
        let total_len = gap_samples + burst_len + gap_samples;
        let mut samples = Vec::with_capacity(total_len);

        // Simple LCG PRNG (deterministic, no external crate)
        let mut rng_state: u64 = 0xDEAD_BEEF_CAFE_1234;
        let mut next_noise = || -> f64 {
            rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            // Map to approximately [-1, 1]
            ((rng_state >> 33) as f64 / (1u64 << 31) as f64) * 2.0 - 1.0
        };

        // Leading gap (noise only)
        for _ in 0..gap_samples {
            samples.push((noise_amplitude * next_noise(), noise_amplitude * next_noise()));
        }
        // Burst
        for i in 0..burst_len {
            let t = i as f64 / sample_rate;
            let decay = (-t / decay_tau_s).exp();
            let phase = 2.0 * PI * freq_hz * t;
            let re = amplitude * decay * phase.cos() + noise_amplitude * next_noise();
            let im = amplitude * decay * phase.sin() + noise_amplitude * next_noise();
            samples.push((re, im));
        }
        // Trailing gap (noise only)
        for _ in 0..gap_samples {
            samples.push((noise_amplitude * next_noise(), noise_amplitude * next_noise()));
        }
        samples
    }
}

// ─── tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_decoder() -> MeteorBurstDecoder {
        MeteorBurstDecoder::new(MeteorBurstConfig::default())
    }

    // ── energy envelope ──────────────────────────────────────────────────

    #[test]
    fn test_energy_envelope_basic() {
        let samples = vec![(3.0, 4.0), (1.0, 0.0), (0.0, 0.0)];
        let env = MeteorBurstDecoder::energy_envelope(&samples);
        assert!((env[0] - 25.0).abs() < 1e-9);
        assert!((env[1] - 1.0).abs() < 1e-9);
        assert!((env[2] - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_energy_envelope_empty() {
        let env = MeteorBurstDecoder::energy_envelope(&[]);
        assert!(env.is_empty());
    }

    // ── smooth envelope ──────────────────────────────────────────────────

    #[test]
    fn test_smooth_envelope_identity() {
        let envelope = vec![1.0, 2.0, 3.0, 4.0];
        let smoothed = MeteorBurstDecoder::smooth_envelope(&envelope, 1);
        for (a, b) in smoothed.iter().zip(envelope.iter()) {
            assert!((a - b).abs() < 1e-9);
        }
    }

    #[test]
    fn test_smooth_envelope_averaging() {
        let envelope = vec![0.0, 0.0, 10.0, 0.0, 0.0];
        let smoothed = MeteorBurstDecoder::smooth_envelope(&envelope, 3);
        // At index 2: avg of [0, 0, 10] = 10/3
        assert!((smoothed[2] - 10.0 / 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_smooth_envelope_empty() {
        let smoothed = MeteorBurstDecoder::smooth_envelope(&[], 5);
        assert!(smoothed.is_empty());
    }

    // ── noise floor ──────────────────────────────────────────────────────

    #[test]
    fn test_noise_floor_estimate() {
        let envelope = vec![1.0, 2.0, 3.0, 100.0, 4.0];
        let nf = MeteorBurstDecoder::estimate_noise_floor(&envelope);
        // Sorted: [1, 2, 3, 4, 100] → median = 3
        assert!((nf - 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_noise_floor_empty() {
        assert!((MeteorBurstDecoder::estimate_noise_floor(&[]) - 0.0).abs() < 1e-9);
    }

    // ── burst detection ──────────────────────────────────────────────────

    #[test]
    fn test_detect_single_burst() {
        let sr: f64 = 48_000.0;
        let samples = MeteorBurstDecoder::generate_test_burst(
            sr,
            1000.0, // freq
            10.0,   // amplitude
            2.0,    // tau
            0.2,    // duration (200 ms)
            0.01,   // noise
            24000,  // gap = 500 ms
        );
        let mut dec = default_decoder();
        let bursts = dec.detect_bursts(&samples);
        assert_eq!(bursts.len(), 1, "should detect exactly one burst");
        let b = &bursts[0];
        assert!(b.duration_s > 0.05, "burst duration should be meaningful");
        assert!(b.peak_snr_db > 10.0, "burst should have high SNR");
    }

    #[test]
    fn test_detect_no_burst_in_noise() {
        let n = 48_000; // 1 second of noise
        let mut rng: u64 = 42;
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|_| {
                rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
                let re = ((rng >> 33) as f64 / (1u64 << 31) as f64) * 2.0 - 1.0;
                rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
                let im = ((rng >> 33) as f64 / (1u64 << 31) as f64) * 2.0 - 1.0;
                (re * 0.01, im * 0.01)
            })
            .collect();
        let mut dec = default_decoder();
        let bursts = dec.detect_bursts(&samples);
        assert!(bursts.is_empty(), "should not detect bursts in pure noise");
    }

    #[test]
    fn test_detect_burst_too_short_rejected() {
        let sr: f64 = 48_000.0;
        let config = MeteorBurstConfig {
            min_burst_duration_s: 0.5, // require at least 500 ms
            ..Default::default()
        };
        // Generate a 100 ms burst – should be rejected
        let samples = MeteorBurstDecoder::generate_test_burst(
            sr, 1000.0, 10.0, 2.0, 0.1, 0.01, 4800,
        );
        let mut dec = MeteorBurstDecoder::new(config);
        let bursts = dec.detect_bursts(&samples);
        assert!(bursts.is_empty(), "short burst should be rejected");
    }

    // ── Doppler estimation ───────────────────────────────────────────────

    #[test]
    fn test_doppler_estimate_accuracy() {
        let dec = default_decoder();
        let sr: f64 = 48_000.0;
        let expected_freq = 500.0;
        let n = 4800; // 100 ms
        let burst: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / sr;
                let phase = 2.0 * PI * expected_freq * t;
                (phase.cos(), phase.sin())
            })
            .collect();
        let doppler = dec.estimate_doppler(&burst);
        assert!(
            (doppler - expected_freq).abs() < 5.0,
            "Doppler estimate {doppler:.1} should be close to {expected_freq}"
        );
    }

    #[test]
    fn test_doppler_zero_for_dc() {
        let dec = default_decoder();
        let burst: Vec<(f64, f64)> = vec![(1.0, 0.0); 1000];
        let doppler = dec.estimate_doppler(&burst);
        assert!(
            doppler.abs() < 1.0,
            "DC signal should have ~0 Hz Doppler, got {doppler}"
        );
    }

    #[test]
    fn test_doppler_stability_pure_tone() {
        let dec = default_decoder();
        let sr: f64 = 48_000.0;
        let n = 4800;
        let burst: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / sr;
                let phase = 2.0 * PI * 200.0 * t;
                (phase.cos(), phase.sin())
            })
            .collect();
        let stability = dec.estimate_doppler_stability(&burst);
        assert!(
            stability < 5.0,
            "pure tone should have low Doppler instability, got {stability:.1} Hz"
        );
    }

    // ── trail classification ─────────────────────────────────────────────

    #[test]
    fn test_classify_underdense_short() {
        let env = vec![10.0, 8.0, 5.0, 3.0, 1.0];
        let tt = MeteorBurstDecoder::classify_trail(0.2, &env);
        assert_eq!(tt, TrailType::Underdense);
    }

    #[test]
    fn test_classify_overdense_plateau() {
        // Long burst with flat first half
        let mut env = vec![10.0; 200];
        // Decay in second half
        for i in 200..400 {
            env.push(10.0 * (-(i as f64 - 200.0) / 50.0).exp());
        }
        let duration_s = 2.0; // > 0.5 s trigger
        let tt = MeteorBurstDecoder::classify_trail(duration_s, &env);
        assert_eq!(tt, TrailType::Overdense);
    }

    // ── decay model ──────────────────────────────────────────────────────

    #[test]
    fn test_trail_decay_model_shape() {
        let env = MeteorBurstDecoder::trail_decay_model(1.0, 0.5, 1.0, 1000.0);
        assert_eq!(env.len(), 1000);
        assert!((env[0] - 1.0).abs() < 1e-9, "starts at peak_amplitude");
        // At t = τ the amplitude should be 1/e ≈ 0.368
        let idx_tau = 500; // 0.5 s at 1000 Hz
        assert!(
            (env[idx_tau] - (-1.0_f64).exp()).abs() < 0.01,
            "amplitude at t=τ should be ~1/e"
        );
    }

    #[test]
    fn test_decay_tau_estimation() {
        let sr = 10_000.0;
        let tau = 0.3;
        let env = MeteorBurstDecoder::trail_decay_model(5.0, tau, 1.0, sr);
        let power_env: Vec<f64> = env.iter().map(|a| a * a).collect();
        let dec = default_decoder();
        let estimated_tau = dec.estimate_decay_tau(&power_env, sr);
        // Power decays as exp(-2t/tau), so the power tau is tau/2.
        // Our estimator fits the power envelope directly, so it should return tau/2.
        let power_tau = tau / 2.0;
        let error = (estimated_tau - power_tau).abs();
        assert!(
            error < 0.15,
            "estimated τ={estimated_tau:.4} should be close to power τ={power_tau:.3} (error={error:.4})"
        );
    }

    // ── burst quality ────────────────────────────────────────────────────

    #[test]
    fn test_burst_quality_high_snr() {
        let burst = BurstDescriptor {
            start_sample: 0,
            end_sample: 48000,
            duration_s: 1.0,
            peak_snr_db: 20.0,
            doppler_shift_hz: 50.0,
            doppler_stability_hz: 10.0,
            decay_tau_s: 0.5,
            trail_type: TrailType::Underdense,
        };
        let q = MeteorBurstDecoder::burst_quality(&burst);
        assert!(q.quality_score > 0.5, "high-SNR burst should have good quality");
        assert!((q.snr_db - 20.0).abs() < 1e-9);
    }

    #[test]
    fn test_burst_quality_low_snr() {
        let burst = BurstDescriptor {
            start_sample: 0,
            end_sample: 480,
            duration_s: 0.01,
            peak_snr_db: 3.0,
            doppler_shift_hz: 50.0,
            doppler_stability_hz: 300.0,
            decay_tau_s: 0.05,
            trail_type: TrailType::Underdense,
        };
        let q = MeteorBurstDecoder::burst_quality(&burst);
        assert!(q.quality_score < 0.3, "low quality burst should score low");
    }

    // ── link availability ────────────────────────────────────────────────

    #[test]
    fn test_link_availability_no_bursts() {
        let dec = default_decoder();
        let la = dec.link_availability(60.0);
        assert_eq!(la.burst_count, 0);
        assert!((la.duty_cycle - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_link_availability_with_bursts() {
        let sr: f64 = 48_000.0;
        let samples = MeteorBurstDecoder::generate_test_burst(
            sr, 1000.0, 20.0, 2.0, 0.3, 0.01, 9600,
        );
        let mut dec = default_decoder();
        let _ = dec.detect_bursts(&samples);
        let total_time = samples.len() as f64 / sr;
        let la = dec.link_availability(total_time);
        assert!(la.burst_count >= 1, "should have at least one burst");
        assert!(la.duty_cycle > 0.0, "duty cycle should be positive");
        assert!(la.duty_cycle <= 1.0, "duty cycle should be <= 1");
    }

    // ── FSK demodulation ─────────────────────────────────────────────────

    #[test]
    fn test_fsk_demodulation() {
        let sr: f64 = 48_000.0;
        let symbol_rate: f64 = 300.0;
        let deviation = 500.0;
        let sps = (sr / symbol_rate).round() as usize;

        let bits = vec![true, false, true, true, false];
        let mut samples = Vec::new();
        for &bit in &bits {
            let freq = if bit { deviation } else { -deviation };
            for i in 0..sps {
                let t = i as f64 / sr;
                let phase = 2.0 * PI * freq * t;
                samples.push((phase.cos(), phase.sin()));
            }
        }

        let dec = default_decoder();
        let decoded = dec.demodulate_fsk(&samples, symbol_rate, deviation);
        assert_eq!(decoded.len(), bits.len());
        assert_eq!(decoded, bits);
    }

    // ── BPSK demodulation ────────────────────────────────────────────────

    #[test]
    fn test_bpsk_demodulation() {
        let sr: f64 = 48_000.0;
        let symbol_rate: f64 = 1200.0;
        let sps = (sr / symbol_rate).round() as usize;

        let bits = vec![true, false, false, true, true, false];
        let mut samples = Vec::new();
        for &bit in &bits {
            let amplitude = if bit { 1.0 } else { -1.0 };
            for _ in 0..sps {
                samples.push((amplitude, 0.0));
            }
        }

        let dec = default_decoder();
        let decoded = dec.demodulate_bpsk(&samples, symbol_rate);
        assert_eq!(decoded.len(), bits.len());
        assert_eq!(decoded, bits);
    }

    // ── config & reset ───────────────────────────────────────────────────

    #[test]
    fn test_config_default() {
        let cfg = MeteorBurstConfig::default();
        assert!((cfg.carrier_freq_hz - 50.0e6).abs() < 1.0);
        assert!((cfg.sample_rate_hz - 48_000.0).abs() < 1.0);
    }

    #[test]
    fn test_reset_history() {
        let sr: f64 = 48_000.0;
        let samples = MeteorBurstDecoder::generate_test_burst(
            sr, 1000.0, 10.0, 2.0, 0.2, 0.01, 24000,
        );
        let mut dec = default_decoder();
        let _ = dec.detect_bursts(&samples);
        assert!(!dec.burst_history().is_empty());
        dec.reset_history();
        assert!(dec.burst_history().is_empty());
    }

    // ── generate_test_burst ──────────────────────────────────────────────

    #[test]
    fn test_generate_test_burst_length() {
        let sr: f64 = 48_000.0;
        let dur = 0.5;
        let gap = 1000;
        let samples = MeteorBurstDecoder::generate_test_burst(
            sr, 500.0, 1.0, 1.0, dur, 0.01, gap,
        );
        let expected_burst = (dur * sr).ceil() as usize;
        assert_eq!(samples.len(), gap + expected_burst + gap);
    }
}
