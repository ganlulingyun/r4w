//! Real-time pulse parameter estimation for radar ESM and ELINT applications.
//!
//! This module provides a [`PulseDescriptorExtractor`] that detects radar pulses in IQ sample
//! streams and produces [`PulseDescriptorWord`] (PDW) records describing each pulse's
//! time-of-arrival, pulse width, amplitude, frequency, and modulation-on-pulse (MOP) type.
//!
//! # Example
//!
//! ```
//! use r4w_core::pulse_descriptor_extractor::{PulseDescriptorExtractor, PdwConfig, MopType};
//!
//! // Configure for a 10 MHz sample rate, -20 dB threshold
//! let config = PdwConfig {
//!     sample_rate: 10e6,
//!     threshold_db: -20.0,
//!     min_pulse_width_s: 0.5e-6,
//!     max_pulse_width_s: 100e-6,
//!     min_gap_s: 0.5e-6,
//! };
//!
//! let extractor = PulseDescriptorExtractor::new(config);
//!
//! // Generate a simple test pulse: 50 samples of tone in noise
//! let num_samples = 500;
//! let mut samples: Vec<(f64, f64)> = vec![(0.001, 0.001); num_samples];
//! // Insert a pulse from sample 100 to 150
//! for i in 100..150 {
//!     let phase = 2.0 * std::f64::consts::PI * 1e6 * (i as f64) / 10e6;
//!     samples[i] = (phase.cos(), phase.sin());
//! }
//!
//! let pdws = extractor.detect_pulses(&samples);
//! assert!(!pdws.is_empty(), "should detect at least one pulse");
//! assert!(pdws[0].pw_s > 0.0, "pulse width must be positive");
//! ```

use std::f64::consts::PI;

// ─── Configuration ───────────────────────────────────────────────────────────

/// Configuration for the pulse descriptor extractor.
#[derive(Debug, Clone)]
pub struct PdwConfig {
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Detection threshold in dB relative to full-scale (negative means below full-scale).
    pub threshold_db: f64,
    /// Minimum accepted pulse width in seconds.
    pub min_pulse_width_s: f64,
    /// Maximum accepted pulse width in seconds.
    pub max_pulse_width_s: f64,
    /// Minimum gap between pulses in seconds.
    pub min_gap_s: f64,
}

impl Default for PdwConfig {
    fn default() -> Self {
        Self {
            sample_rate: 10e6,
            threshold_db: -20.0,
            min_pulse_width_s: 0.5e-6,
            max_pulse_width_s: 100e-6,
            min_gap_s: 0.5e-6,
        }
    }
}

// ─── Modulation-on-Pulse Type ────────────────────────────────────────────────

/// Classification of the modulation present within a single radar pulse.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MopType {
    /// Continuous-wave pulse with no intra-pulse modulation.
    Unmodulated,
    /// Linear frequency modulation (chirp).
    LinearFM,
    /// Barker-coded phase modulation.
    BarkerCoded,
    /// Polyphase-coded (e.g., Frank, P1-P4).
    PolyphaseCode,
    /// Frequency-stepped pulse.
    FreqStepped,
    /// Could not determine the MOP.
    Unknown,
}

// ─── Pulse Descriptor Word ──────────────────────────────────────────────────

/// A single Pulse Descriptor Word produced by the extractor.
#[derive(Debug, Clone)]
pub struct PulseDescriptorWord {
    /// Time of arrival of the pulse leading edge (seconds from start of buffer).
    pub toa_s: f64,
    /// Pulse width in seconds (leading edge to trailing edge).
    pub pw_s: f64,
    /// Pulse repetition interval in seconds (zero if not yet computed).
    pub pri_s: f64,
    /// Peak envelope amplitude in dB (20 * log10(magnitude)).
    pub amplitude_db: f64,
    /// Estimated carrier frequency offset within the pulse in Hz.
    pub freq_hz: f64,
    /// Estimated instantaneous bandwidth in Hz.
    pub bandwidth_hz: f64,
    /// Modulation-on-pulse classification.
    pub mop_type: MopType,
}

// ─── Extractor ──────────────────────────────────────────────────────────────

/// Main pulse measurement engine.
///
/// Accepts a stream of complex IQ samples and produces a vector of
/// [`PulseDescriptorWord`] records.
pub struct PulseDescriptorExtractor {
    config: PdwConfig,
    /// Linear threshold derived from `threshold_db`.
    threshold_linear: f64,
}

impl PulseDescriptorExtractor {
    /// Create a new extractor with the given configuration.
    pub fn new(config: PdwConfig) -> Self {
        // Convert dB threshold to linear magnitude.
        let threshold_linear = 10.0_f64.powf(config.threshold_db / 20.0);
        Self {
            config,
            threshold_linear,
        }
    }

    // ── public API ──────────────────────────────────────────────────────────

    /// Detect all pulses in the given IQ sample buffer and return their PDWs.
    ///
    /// Envelope detection is performed via `|I + jQ|` (magnitude).  Leading and
    /// trailing edges are found by threshold crossing.  Pulses that violate the
    /// configured width or gap constraints are discarded.
    pub fn detect_pulses(&self, samples: &[(f64, f64)]) -> Vec<PulseDescriptorWord> {
        let envelope = self.compute_envelope(samples);
        let pulse_regions = self.find_pulse_regions(&envelope);

        let mut pdws: Vec<PulseDescriptorWord> = Vec::new();
        for (start, end) in &pulse_regions {
            let pulse_samples = &samples[*start..*end];
            let toa = *start as f64 / self.config.sample_rate;
            let pdw = self.extract_pdw(pulse_samples, toa);
            pdws.push(pdw);
        }

        // Fill in PRI from successive TOAs.
        if pdws.len() >= 2 {
            let pri = self.estimate_pri(&pdws);
            for pdw in &mut pdws {
                pdw.pri_s = pri;
            }
        }

        pdws
    }

    /// Build a full [`PulseDescriptorWord`] from the IQ samples of a single pulse.
    pub fn extract_pdw(&self, pulse_samples: &[(f64, f64)], toa: f64) -> PulseDescriptorWord {
        let pw_s = pulse_samples.len() as f64 / self.config.sample_rate;
        let amplitude_db = self.peak_amplitude_db(pulse_samples);
        let freq_hz = self.estimate_frequency(pulse_samples);
        let bandwidth_hz = self.estimate_bandwidth(pulse_samples);
        let mop_type = self.classify_mop(pulse_samples);

        PulseDescriptorWord {
            toa_s: toa,
            pw_s,
            pri_s: 0.0,
            amplitude_db,
            freq_hz,
            bandwidth_hz,
            mop_type,
        }
    }

    /// Estimate the pulse repetition interval (PRI) from a sequence of PDWs.
    ///
    /// Uses the median of successive TOA differences.
    pub fn estimate_pri(&self, pdws: &[PulseDescriptorWord]) -> f64 {
        if pdws.len() < 2 {
            return 0.0;
        }
        let mut deltas: Vec<f64> = pdws
            .windows(2)
            .map(|w| w[1].toa_s - w[0].toa_s)
            .collect();
        deltas.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        // Return the median.
        deltas[deltas.len() / 2]
    }

    /// Estimate the dominant frequency offset in a pulse using phase-difference
    /// estimation across consecutive samples.
    pub fn estimate_frequency(&self, samples: &[(f64, f64)]) -> f64 {
        if samples.len() < 2 {
            return 0.0;
        }

        // Phase-difference frequency estimator:
        // f_est = (1 / 2pi) * mean(delta_phi) * fs
        let mut phase_diffs = Vec::with_capacity(samples.len() - 1);
        for i in 1..samples.len() {
            let (i1, q1) = samples[i - 1];
            let (i2, q2) = samples[i];
            // Complex conjugate multiply: s[n] * conj(s[n-1])
            let re = i2 * i1 + q2 * q1;
            let im = q2 * i1 - i2 * q1;
            phase_diffs.push(im.atan2(re));
        }

        let mean_dphi = phase_diffs.iter().sum::<f64>() / phase_diffs.len() as f64;
        mean_dphi * self.config.sample_rate / (2.0 * PI)
    }

    /// Classify the modulation-on-pulse (MOP) of a set of pulse samples.
    ///
    /// Uses several heuristics applied in priority order:
    /// 1. **LinearFM**: instantaneous frequency (from conjugate-product phase diffs)
    ///    shows a strong linear trend (R-squared > 0.85).
    /// 2. **BarkerCoded / PolyphaseCode**: abrupt phase jumps between consecutive
    ///    samples indicate phase coding. Binary jumps near pi => Barker.
    /// 3. **FreqStepped**: instantaneous frequency occupies a small number of
    ///    discrete levels.
    /// 4. **Unmodulated**: low inst-freq variance.
    pub fn classify_mop(&self, samples: &[(f64, f64)]) -> MopType {
        if samples.len() < 4 {
            return MopType::Unknown;
        }

        // Compute instantaneous frequency via conjugate-product phase differences.
        // This avoids the wrapping artefacts of raw atan2 differencing and gives
        // a smooth frequency estimate for continuous-phase signals like LFM.
        let inst_freq: Vec<f64> = (1..samples.len())
            .map(|i| {
                let (i1, q1) = samples[i - 1];
                let (i2, q2) = samples[i];
                let re = i2 * i1 + q2 * q1;
                let im = q2 * i1 - i2 * q1;
                im.atan2(re)
            })
            .collect();

        if inst_freq.is_empty() {
            return MopType::Unknown;
        }

        let n = inst_freq.len() as f64;
        let x_mean = (n - 1.0) / 2.0;
        let y_mean = inst_freq.iter().sum::<f64>() / n;

        let mut ss_xy = 0.0;
        let mut ss_xx = 0.0;
        let mut ss_yy = 0.0;
        for (i, &y) in inst_freq.iter().enumerate() {
            let x = i as f64 - x_mean;
            let dy = y - y_mean;
            ss_xy += x * dy;
            ss_xx += x * x;
            ss_yy += dy * dy;
        }

        // ── 1. Check for LinearFM (chirp) first ────────────────────────────
        // A chirp has a smoothly varying inst_freq with high R-squared to a line.
        if ss_xx > 0.0 && ss_yy > 1e-20 {
            let r_squared = (ss_xy * ss_xy) / (ss_xx * ss_yy);
            if r_squared > 0.85 {
                return MopType::LinearFM;
            }
        }

        // ── 2. Check for phase coding (Barker / Polyphase) ────────────────
        // Phase-coded signals have abrupt inst_freq spikes at chip boundaries
        // but the majority of samples have a near-constant inst_freq.
        // We detect this by looking at outliers in inst_freq relative to the
        // median value.
        let mut sorted_freq = inst_freq.clone();
        sorted_freq.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median_freq = sorted_freq[sorted_freq.len() / 2];

        // Compute inter-quartile range for robust outlier detection.
        let q1_idx = sorted_freq.len() / 4;
        let q3_idx = 3 * sorted_freq.len() / 4;
        let iqr = sorted_freq[q3_idx] - sorted_freq[q1_idx];

        // Count samples whose inst_freq deviates significantly from the median.
        // Phase transitions cause large transient frequency spikes.
        let outlier_threshold = if iqr > 1e-10 { 3.0 * iqr } else { 0.3 };
        let outlier_count = inst_freq
            .iter()
            .filter(|&&f| (f - median_freq).abs() > outlier_threshold)
            .count();
        let outlier_ratio = outlier_count as f64 / inst_freq.len() as f64;

        // Phase-coded signals: small fraction (1-15%) of samples are outliers
        // (the chip transitions), while the rest are near-constant.
        if outlier_ratio > 0.005 && outlier_ratio < 0.20 {
            // Check if the phase jumps at transitions are predominantly near pi
            // (binary phase coding = Barker) vs other angles (polyphase).
            let inst_phase: Vec<f64> = samples.iter().map(|(i, q)| q.atan2(*i)).collect();
            let mut pi_jump_count = 0usize;
            let mut total_jumps = 0usize;
            for w in inst_phase.windows(2) {
                let mut d = (w[1] - w[0]).abs();
                if d > PI {
                    d = 2.0 * PI - d;
                }
                if d > PI / 4.0 {
                    total_jumps += 1;
                    if d > 0.6 * PI && d < 1.4 * PI {
                        pi_jump_count += 1;
                    }
                }
            }
            if total_jumps > 0 {
                return if pi_jump_count as f64 / total_jumps as f64 > 0.5 {
                    MopType::BarkerCoded
                } else {
                    MopType::PolyphaseCode
                };
            }
        }

        // ── 3. Check for frequency stepping ────────────────────────────────
        if self.detect_freq_stepping(&inst_freq) {
            return MopType::FreqStepped;
        }

        // ── 4. Unmodulated if inst_freq variance is low ────────────────────
        let variance = ss_yy / n;
        if variance < 0.05 {
            return MopType::Unmodulated;
        }

        MopType::Unknown
    }

    /// Compute the duty cycle from a sequence of PDWs: mean(pw) / mean(pri).
    pub fn duty_cycle(&self, pdws: &[PulseDescriptorWord]) -> f64 {
        if pdws.is_empty() {
            return 0.0;
        }
        let mean_pw = pdws.iter().map(|p| p.pw_s).sum::<f64>() / pdws.len() as f64;
        let pri = self.estimate_pri(pdws);
        if pri <= 0.0 {
            return 0.0;
        }
        (mean_pw / pri).clamp(0.0, 1.0)
    }

    // ── private helpers ─────────────────────────────────────────────────────

    /// Compute the magnitude envelope of the IQ samples.
    fn compute_envelope(&self, samples: &[(f64, f64)]) -> Vec<f64> {
        samples
            .iter()
            .map(|(i, q)| (i * i + q * q).sqrt())
            .collect()
    }

    /// Find contiguous regions where the envelope exceeds the threshold.
    ///
    /// Returns a vector of `(start_index, end_index)` pairs (end is exclusive).
    fn find_pulse_regions(&self, envelope: &[f64]) -> Vec<(usize, usize)> {
        let min_samples =
            (self.config.min_pulse_width_s * self.config.sample_rate).round() as usize;
        let max_samples =
            (self.config.max_pulse_width_s * self.config.sample_rate).round() as usize;
        let min_gap_samples =
            (self.config.min_gap_s * self.config.sample_rate).round() as usize;

        let mut regions = Vec::new();
        let mut in_pulse = false;
        let mut start = 0usize;
        let mut last_end = 0usize;

        for (i, &val) in envelope.iter().enumerate() {
            if !in_pulse && val >= self.threshold_linear {
                // Leading edge.
                // Enforce minimum gap from previous pulse.
                if !regions.is_empty() && (i - last_end) < min_gap_samples {
                    continue;
                }
                in_pulse = true;
                start = i;
            } else if in_pulse && val < self.threshold_linear {
                // Trailing edge.
                let end = i;
                let width = end - start;
                if width >= min_samples && width <= max_samples {
                    regions.push((start, end));
                    last_end = end;
                }
                in_pulse = false;
            }
        }

        // Handle pulse that extends to end of buffer.
        if in_pulse {
            let end = envelope.len();
            let width = end - start;
            if width >= min_samples && width <= max_samples {
                regions.push((start, end));
            }
        }

        regions
    }

    /// Peak amplitude in dB (20*log10 of max magnitude).
    fn peak_amplitude_db(&self, samples: &[(f64, f64)]) -> f64 {
        let max_mag = samples
            .iter()
            .map(|(i, q)| (i * i + q * q).sqrt())
            .fold(0.0_f64, f64::max);
        if max_mag > 0.0 {
            20.0 * max_mag.log10()
        } else {
            f64::NEG_INFINITY
        }
    }

    /// Estimate bandwidth from the spread of instantaneous frequency.
    fn estimate_bandwidth(&self, samples: &[(f64, f64)]) -> f64 {
        if samples.len() < 3 {
            return 0.0;
        }

        // Compute instantaneous frequency via phase differences.
        let mut inst_freq_hz = Vec::with_capacity(samples.len() - 1);
        for i in 1..samples.len() {
            let (i1, q1) = samples[i - 1];
            let (i2, q2) = samples[i];
            let re = i2 * i1 + q2 * q1;
            let im = q2 * i1 - i2 * q1;
            let dphi = im.atan2(re);
            inst_freq_hz.push(dphi * self.config.sample_rate / (2.0 * PI));
        }

        let min_f = inst_freq_hz.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_f = inst_freq_hz
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        (max_f - min_f).abs()
    }

    /// Detect frequency stepping by quantising instantaneous frequency and
    /// checking for a small number of distinct levels with low intra-level
    /// variance.
    fn detect_freq_stepping(&self, inst_freq: &[f64]) -> bool {
        if inst_freq.len() < 16 {
            return false;
        }

        // Compute the range.
        let min_f = inst_freq.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_f = inst_freq
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let range = max_f - min_f;
        if range < 0.1 {
            return false; // Need meaningful frequency variation.
        }

        // Quantise into bins and count occupied bins.
        let num_bins = 20usize;
        let bin_width = range / num_bins as f64;
        let mut bin_counts = vec![0usize; num_bins];
        for &f in inst_freq {
            let bin = ((f - min_f) / bin_width).floor() as usize;
            let bin = bin.min(num_bins - 1);
            bin_counts[bin] += 1;
        }
        let occupied_count = bin_counts.iter().filter(|&&c| c > 0).count();

        // Frequency stepping: a few discrete levels (2-6 occupied bins) and
        // each occupied bin should hold a significant fraction of samples.
        if !(2..=6).contains(&occupied_count) {
            return false;
        }

        // Check that the occupied bins each hold a reasonable share of samples.
        // Each step should have at least 5% of total samples.
        let min_bin_share = inst_freq.len() / 20;
        let well_populated = bin_counts
            .iter()
            .filter(|&&c| c >= min_bin_share.max(2))
            .count();
        well_populated >= 2 && well_populated == occupied_count
    }
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> PdwConfig {
        PdwConfig {
            sample_rate: 10e6,
            threshold_db: -20.0,
            min_pulse_width_s: 0.5e-6,
            max_pulse_width_s: 100e-6,
            min_gap_s: 0.5e-6,
        }
    }

    /// Helper: create a tone burst at a given frequency, embedded in silence.
    fn make_tone_pulse(
        total_samples: usize,
        pulse_start: usize,
        pulse_end: usize,
        freq_hz: f64,
        sample_rate: f64,
        amplitude: f64,
    ) -> Vec<(f64, f64)> {
        let mut out = vec![(0.0, 0.0); total_samples];
        for i in pulse_start..pulse_end.min(total_samples) {
            let t = i as f64 / sample_rate;
            let phase = 2.0 * PI * freq_hz * t;
            out[i] = (amplitude * phase.cos(), amplitude * phase.sin());
        }
        out
    }

    #[test]
    fn test_new_extractor_sets_threshold() {
        let config = default_config();
        let ext = PulseDescriptorExtractor::new(config.clone());
        let expected = 10.0_f64.powf(-20.0 / 20.0);
        assert!((ext.threshold_linear - expected).abs() < 1e-12);
    }

    #[test]
    fn test_detect_single_pulse() {
        let config = default_config();
        let ext = PulseDescriptorExtractor::new(config.clone());
        let samples = make_tone_pulse(1000, 200, 300, 1e6, config.sample_rate, 1.0);
        let pdws = ext.detect_pulses(&samples);
        assert_eq!(pdws.len(), 1, "should detect exactly one pulse");
        // Pulse width should be close to 100 samples / 10 MHz = 10 us.
        assert!((pdws[0].pw_s - 10e-6).abs() < 1e-6);
    }

    #[test]
    fn test_detect_multiple_pulses() {
        let config = default_config();
        let ext = PulseDescriptorExtractor::new(config.clone());
        // Two pulses separated by a gap.
        let mut samples = vec![(0.0, 0.0); 2000];
        for i in 100..200 {
            let t = i as f64 / config.sample_rate;
            let phase = 2.0 * PI * 1e6 * t;
            samples[i] = (phase.cos(), phase.sin());
        }
        for i in 500..600 {
            let t = i as f64 / config.sample_rate;
            let phase = 2.0 * PI * 1e6 * t;
            samples[i] = (phase.cos(), phase.sin());
        }
        let pdws = ext.detect_pulses(&samples);
        assert_eq!(pdws.len(), 2, "should detect two pulses");
    }

    #[test]
    fn test_toa_accuracy() {
        let config = default_config();
        let ext = PulseDescriptorExtractor::new(config.clone());
        let pulse_start = 400;
        let samples = make_tone_pulse(1000, pulse_start, 500, 1e6, config.sample_rate, 1.0);
        let pdws = ext.detect_pulses(&samples);
        assert_eq!(pdws.len(), 1);
        let expected_toa = pulse_start as f64 / config.sample_rate;
        assert!(
            (pdws[0].toa_s - expected_toa).abs() < 2.0 / config.sample_rate,
            "TOA {:.9} should be close to {:.9}",
            pdws[0].toa_s,
            expected_toa
        );
    }

    #[test]
    fn test_amplitude_db() {
        let config = default_config();
        let ext = PulseDescriptorExtractor::new(config);
        // Unit-amplitude pulse -> ~0 dB.
        let samples: Vec<(f64, f64)> = (0..100)
            .map(|i| {
                let phase = 2.0 * PI * 1e6 * i as f64 / 10e6;
                (phase.cos(), phase.sin())
            })
            .collect();
        let pdw = ext.extract_pdw(&samples, 0.0);
        assert!(
            (pdw.amplitude_db - 0.0).abs() < 0.5,
            "amplitude_db {:.2} should be near 0 dB for unit amplitude",
            pdw.amplitude_db
        );
    }

    #[test]
    fn test_frequency_estimation_positive() {
        let config = default_config();
        let ext = PulseDescriptorExtractor::new(config.clone());
        let freq = 2.5e6;
        let samples: Vec<(f64, f64)> = (0..200)
            .map(|i| {
                let t = i as f64 / config.sample_rate;
                let phase = 2.0 * PI * freq * t;
                (phase.cos(), phase.sin())
            })
            .collect();
        let est = ext.estimate_frequency(&samples);
        assert!(
            (est - freq).abs() < 0.01 * freq,
            "estimated freq {:.0} should be close to {:.0}",
            est,
            freq
        );
    }

    #[test]
    fn test_frequency_estimation_negative() {
        let config = default_config();
        let ext = PulseDescriptorExtractor::new(config.clone());
        let freq = -1.5e6;
        let samples: Vec<(f64, f64)> = (0..200)
            .map(|i| {
                let t = i as f64 / config.sample_rate;
                let phase = 2.0 * PI * freq * t;
                (phase.cos(), phase.sin())
            })
            .collect();
        let est = ext.estimate_frequency(&samples);
        assert!(
            (est - freq).abs() < 0.01 * freq.abs(),
            "estimated freq {:.0} should be close to {:.0}",
            est,
            freq
        );
    }

    #[test]
    fn test_frequency_estimation_empty() {
        let config = default_config();
        let ext = PulseDescriptorExtractor::new(config);
        assert_eq!(ext.estimate_frequency(&[]), 0.0);
        assert_eq!(ext.estimate_frequency(&[(1.0, 0.0)]), 0.0);
    }

    #[test]
    fn test_classify_mop_unmodulated() {
        let config = default_config();
        let ext = PulseDescriptorExtractor::new(config.clone());
        let freq = 1e6;
        let samples: Vec<(f64, f64)> = (0..500)
            .map(|i| {
                let t = i as f64 / config.sample_rate;
                let phase = 2.0 * PI * freq * t;
                (phase.cos(), phase.sin())
            })
            .collect();
        assert_eq!(ext.classify_mop(&samples), MopType::Unmodulated);
    }

    #[test]
    fn test_classify_mop_linear_fm() {
        let config = default_config();
        let ext = PulseDescriptorExtractor::new(config.clone());
        // Linear chirp: frequency sweeps from f0 to f1.
        let f0 = 0.5e6;
        let f1 = 3.0e6;
        let n = 500;
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / config.sample_rate;
                let duration = n as f64 / config.sample_rate;
                let phase = 2.0 * PI * (f0 * t + 0.5 * (f1 - f0) * t * t / duration);
                (phase.cos(), phase.sin())
            })
            .collect();
        assert_eq!(ext.classify_mop(&samples), MopType::LinearFM);
    }

    #[test]
    fn test_classify_mop_barker_coded() {
        let config = PdwConfig {
            sample_rate: 10e6,
            ..default_config()
        };
        let ext = PulseDescriptorExtractor::new(config.clone());
        // Barker-13 code: [+1,+1,+1,+1,+1,-1,-1,+1,+1,-1,+1,-1,+1]
        let barker13 = [1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1];
        let chips_per_bit = 40; // samples per chip
        let mut samples = Vec::new();
        for &chip in &barker13 {
            for _j in 0..chips_per_bit {
                let t = (samples.len()) as f64 / config.sample_rate;
                let phase = 2.0 * PI * 1e6 * t;
                let amp = chip as f64;
                samples.push((amp * phase.cos(), amp * phase.sin()));
            }
        }
        let mop = ext.classify_mop(&samples);
        assert_eq!(
            mop,
            MopType::BarkerCoded,
            "Barker-13 should be classified as BarkerCoded"
        );
    }

    #[test]
    fn test_classify_mop_short_samples() {
        let config = default_config();
        let ext = PulseDescriptorExtractor::new(config);
        assert_eq!(ext.classify_mop(&[]), MopType::Unknown);
        assert_eq!(ext.classify_mop(&[(1.0, 0.0)]), MopType::Unknown);
        assert_eq!(
            ext.classify_mop(&[(1.0, 0.0), (0.0, 1.0)]),
            MopType::Unknown
        );
    }

    #[test]
    fn test_estimate_pri_two_pulses() {
        let config = default_config();
        let ext = PulseDescriptorExtractor::new(config.clone());
        // Place two pulses 1000 samples apart (100 us at 10 MHz).
        let mut samples = vec![(0.0, 0.0); 3000];
        for i in 100..200 {
            let t = i as f64 / config.sample_rate;
            let phase = 2.0 * PI * 1e6 * t;
            samples[i] = (phase.cos(), phase.sin());
        }
        for i in 1100..1200 {
            let t = i as f64 / config.sample_rate;
            let phase = 2.0 * PI * 1e6 * t;
            samples[i] = (phase.cos(), phase.sin());
        }
        let pdws = ext.detect_pulses(&samples);
        assert_eq!(pdws.len(), 2);
        let pri = ext.estimate_pri(&pdws);
        // Expected PRI: 1000 samples / 10 MHz = 100 us.
        assert!(
            (pri - 100e-6).abs() < 2e-6,
            "PRI {:.9} should be near 100e-6",
            pri
        );
    }

    #[test]
    fn test_estimate_pri_single_pdw() {
        let config = default_config();
        let ext = PulseDescriptorExtractor::new(config);
        let pdws = vec![PulseDescriptorWord {
            toa_s: 0.001,
            pw_s: 10e-6,
            pri_s: 0.0,
            amplitude_db: 0.0,
            freq_hz: 1e6,
            bandwidth_hz: 0.0,
            mop_type: MopType::Unmodulated,
        }];
        assert_eq!(ext.estimate_pri(&pdws), 0.0);
    }

    #[test]
    fn test_duty_cycle() {
        let config = default_config();
        let ext = PulseDescriptorExtractor::new(config);
        // pw = 10 us, PRI = 100 us -> duty cycle = 0.1
        let pdws = vec![
            PulseDescriptorWord {
                toa_s: 0.0,
                pw_s: 10e-6,
                pri_s: 0.0,
                amplitude_db: 0.0,
                freq_hz: 0.0,
                bandwidth_hz: 0.0,
                mop_type: MopType::Unmodulated,
            },
            PulseDescriptorWord {
                toa_s: 100e-6,
                pw_s: 10e-6,
                pri_s: 0.0,
                amplitude_db: 0.0,
                freq_hz: 0.0,
                bandwidth_hz: 0.0,
                mop_type: MopType::Unmodulated,
            },
        ];
        let dc = ext.duty_cycle(&pdws);
        assert!(
            (dc - 0.1).abs() < 0.02,
            "duty cycle {:.4} should be near 0.1",
            dc
        );
    }

    #[test]
    fn test_duty_cycle_empty() {
        let config = default_config();
        let ext = PulseDescriptorExtractor::new(config);
        assert_eq!(ext.duty_cycle(&[]), 0.0);
    }

    #[test]
    fn test_no_pulses_below_threshold() {
        let config = PdwConfig {
            threshold_db: -6.0, // -6 dB ~ 0.5 linear
            ..default_config()
        };
        let ext = PulseDescriptorExtractor::new(config.clone());
        // All samples have amplitude 0.1 (well below 0.5 threshold).
        let samples: Vec<(f64, f64)> = (0..1000)
            .map(|i| {
                let t = i as f64 / config.sample_rate;
                let phase = 2.0 * PI * 1e6 * t;
                (0.1 * phase.cos(), 0.1 * phase.sin())
            })
            .collect();
        let pdws = ext.detect_pulses(&samples);
        assert!(pdws.is_empty(), "should detect no pulses below threshold");
    }

    #[test]
    fn test_pulse_too_short_rejected() {
        let config = PdwConfig {
            min_pulse_width_s: 20e-6, // 200 samples at 10 MHz
            ..default_config()
        };
        let ext = PulseDescriptorExtractor::new(config.clone());
        // 50 samples = 5 us, below the 20 us minimum.
        let samples = make_tone_pulse(1000, 200, 250, 1e6, config.sample_rate, 1.0);
        let pdws = ext.detect_pulses(&samples);
        assert!(
            pdws.is_empty(),
            "pulse shorter than min_pulse_width should be rejected"
        );
    }

    #[test]
    fn test_pulse_too_long_rejected() {
        let config = PdwConfig {
            max_pulse_width_s: 5e-6, // 50 samples at 10 MHz
            ..default_config()
        };
        let ext = PulseDescriptorExtractor::new(config.clone());
        // 200 samples = 20 us, above the 5 us maximum.
        let samples = make_tone_pulse(1000, 200, 400, 1e6, config.sample_rate, 1.0);
        let pdws = ext.detect_pulses(&samples);
        assert!(
            pdws.is_empty(),
            "pulse longer than max_pulse_width should be rejected"
        );
    }

    #[test]
    fn test_bandwidth_estimate_chirp() {
        let config = default_config();
        let ext = PulseDescriptorExtractor::new(config.clone());
        // Chirp from 1 MHz to 3 MHz -> bandwidth ~ 2 MHz.
        let f0 = 1.0e6;
        let f1 = 3.0e6;
        let n = 500;
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / config.sample_rate;
                let duration = n as f64 / config.sample_rate;
                let phase = 2.0 * PI * (f0 * t + 0.5 * (f1 - f0) * t * t / duration);
                (phase.cos(), phase.sin())
            })
            .collect();
        let bw = ext.estimate_bandwidth(&samples);
        // Allow 20% tolerance.
        let expected_bw = 2.0e6;
        assert!(
            (bw - expected_bw).abs() < 0.20 * expected_bw,
            "bandwidth {:.0} should be near {:.0} Hz",
            bw,
            expected_bw
        );
    }

    #[test]
    fn test_pdw_config_default() {
        let config = PdwConfig::default();
        assert_eq!(config.sample_rate, 10e6);
        assert_eq!(config.threshold_db, -20.0);
        assert!(config.min_pulse_width_s > 0.0);
        assert!(config.max_pulse_width_s > config.min_pulse_width_s);
    }
}
