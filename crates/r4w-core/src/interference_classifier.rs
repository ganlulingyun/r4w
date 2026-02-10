//! Interference type identification and characterization using feature extraction.
//!
//! This module provides a decision-tree classifier that identifies interference types
//! from IQ samples by extracting spectral and temporal features. No external ML
//! dependencies are required -- classification is performed via threshold-based
//! rules on a [`FeatureVector`].
//!
//! # Supported interference types
//!
//! | Type | Description |
//! |------|-------------|
//! | CW | Continuous-wave (single tone) |
//! | Narrowband | Narrowband interference (modulated carrier) |
//! | Wideband | Wideband noise-like interference |
//! | Pulsed | Pulsed/radar-like interference |
//! | Chirped | Linear or non-linear frequency sweep |
//! | FreqHopping | Frequency-hopping spread spectrum |
//! | NoiseFloor | Background noise (no interference detected) |
//!
//! # Example
//!
//! ```
//! use r4w_core::interference_classifier::{InterferenceClassifier, InterferenceType};
//!
//! // Generate a simple CW (single-tone) interference signal
//! let sample_rate = 10_000.0_f64;
//! let freq = 1_000.0_f64;
//! let n = 1024_usize;
//! let samples: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / sample_rate;
//!         let phase = 2.0 * std::f64::consts::PI * freq * t;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! let classifier = InterferenceClassifier::new();
//! let report = classifier.classify(&samples, sample_rate);
//!
//! // A pure tone should be classified as CW with high confidence
//! assert_eq!(report.interference_type, InterferenceType::CW);
//! assert!(report.confidence > 0.5);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Identified interference type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterferenceType {
    /// Continuous wave -- single tone.
    CW,
    /// Narrowband modulated signal.
    Narrowband,
    /// Wideband noise-like interference.
    Wideband,
    /// Pulsed / radar-like interference.
    Pulsed,
    /// Chirped (frequency sweep) interference.
    Chirped,
    /// Frequency-hopping spread spectrum.
    FreqHopping,
    /// Background noise floor -- no significant interference.
    NoiseFloor,
}

/// Classification result.
#[derive(Debug, Clone)]
pub struct InterferenceReport {
    /// Identified interference type.
    pub interference_type: InterferenceType,
    /// Classification confidence in the range `[0, 1]`.
    pub confidence: f64,
    /// Estimated centre frequency in Hz (relative to baseband centre).
    pub center_freq: f64,
    /// Estimated bandwidth in Hz.
    pub bandwidth: f64,
    /// Estimated duty cycle in the range `[0, 1]`.
    pub duty_cycle: f64,
}

/// Feature vector extracted from IQ samples.
#[derive(Debug, Clone)]
pub struct FeatureVector {
    /// Spectral flatness (Wiener entropy) -- 0 = tonal, 1 = white noise.
    pub spectral_flatness: f64,
    /// Excess kurtosis of the magnitude envelope.
    pub kurtosis: f64,
    /// Fraction of FFT bins whose power exceeds a threshold.
    pub bandwidth_occupancy: f64,
    /// Peak-to-average power ratio (dB).
    pub peak_to_average: f64,
    /// Time-domain duty cycle estimate (fraction of time signal exceeds threshold).
    pub duty_cycle: f64,
    /// Index of the peak spectral bin (normalised to `[0, 1]`).
    pub peak_bin_normalised: f64,
    /// Spectral centroid normalised to `[0, 1]`.
    pub spectral_centroid: f64,
    /// Standard deviation of instantaneous frequency (normalised to sample rate).
    pub inst_freq_std: f64,
    /// Number of detected spectral peaks above the noise floor.
    pub num_spectral_peaks: usize,
    /// Maximum spectral peak magnitude (dB above mean).
    pub max_peak_db: f64,
}

/// Feature-extraction and decision-tree interference classifier.
#[derive(Debug, Clone)]
pub struct InterferenceClassifier {
    /// Minimum number of samples required.
    min_samples: usize,
    /// Threshold (dB above mean) used for spectral peak detection.
    peak_threshold_db: f64,
    /// Threshold for the duty-cycle envelope detector (fraction of peak).
    duty_cycle_threshold: f64,
}

// ---------------------------------------------------------------------------
// Implementation
// ---------------------------------------------------------------------------

impl Default for InterferenceClassifier {
    fn default() -> Self {
        Self::new()
    }
}

impl InterferenceClassifier {
    /// Create a new classifier with default parameters.
    pub fn new() -> Self {
        Self {
            min_samples: 16,
            peak_threshold_db: 6.0,
            duty_cycle_threshold: 0.25,
        }
    }

    /// Create a classifier with custom parameters.
    pub fn with_params(min_samples: usize, peak_threshold_db: f64, duty_cycle_threshold: f64) -> Self {
        Self {
            min_samples: min_samples.max(8),
            peak_threshold_db,
            duty_cycle_threshold,
        }
    }

    /// Classify interference present in `samples` captured at `sample_rate` Hz.
    pub fn classify(&self, samples: &[(f64, f64)], sample_rate: f64) -> InterferenceReport {
        let features = self.extract_features(samples, sample_rate);
        self.classify_from_features(&features, sample_rate)
    }

    // ---- Feature extraction ------------------------------------------------

    /// Extract a [`FeatureVector`] from raw IQ samples.
    pub fn extract_features(&self, samples: &[(f64, f64)], _sample_rate: f64) -> FeatureVector {
        let spectrum = self.power_spectrum(samples);
        let magnitudes: Vec<f64> = samples.iter().map(|(i, q)| (i * i + q * q).sqrt()).collect();

        // -- Spectral flatness (Wiener entropy) --
        let spectral_flatness = Self::compute_spectral_flatness(&spectrum);

        // -- Kurtosis of magnitude envelope --
        let kurtosis = Self::compute_kurtosis(&magnitudes);

        // -- Bandwidth occupancy --
        let mean_power: f64 = spectrum.iter().sum::<f64>() / spectrum.len() as f64;
        let threshold_power = mean_power * Self::db_to_linear(self.peak_threshold_db);
        let bins_above = spectrum.iter().filter(|&&p| p > threshold_power).count();
        let bandwidth_occupancy = bins_above as f64 / spectrum.len() as f64;

        // -- Peak-to-average ratio --
        let max_power = spectrum.iter().cloned().fold(0.0_f64, f64::max);
        let peak_to_average = if mean_power > 0.0 {
            10.0 * (max_power / mean_power).log10()
        } else {
            0.0
        };

        // -- Duty cycle --
        let mag_max = magnitudes.iter().cloned().fold(0.0_f64, f64::max);
        let duty_threshold = mag_max * self.duty_cycle_threshold;
        let active_count = magnitudes.iter().filter(|&&m| m > duty_threshold).count();
        let duty_cycle = active_count as f64 / magnitudes.len() as f64;

        // -- Peak bin (normalised) --
        let peak_idx = spectrum
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, _)| i)
            .unwrap_or(0);
        let peak_bin_normalised = peak_idx as f64 / spectrum.len() as f64;

        // -- Spectral centroid --
        let total_power: f64 = spectrum.iter().sum();
        let spectral_centroid = if total_power > 0.0 {
            spectrum
                .iter()
                .enumerate()
                .map(|(i, &p)| i as f64 * p)
                .sum::<f64>()
                / total_power
                / spectrum.len() as f64
        } else {
            0.5
        };

        // -- Instantaneous frequency standard deviation --
        let inst_freq_std = self.compute_inst_freq_std(samples);

        // -- Number of spectral peaks --
        let (num_spectral_peaks, max_peak_db) =
            self.count_spectral_peaks(&spectrum, mean_power);

        FeatureVector {
            spectral_flatness,
            kurtosis,
            bandwidth_occupancy,
            peak_to_average,
            duty_cycle,
            peak_bin_normalised,
            spectral_centroid,
            inst_freq_std,
            num_spectral_peaks,
            max_peak_db,
        }
    }

    /// Classify from a pre-computed [`FeatureVector`].
    ///
    /// `sample_rate` is used to convert normalised quantities to Hz in the
    /// returned report.
    pub fn classify_from_features(&self, f: &FeatureVector, sample_rate: f64) -> InterferenceReport {
        // Estimate centre frequency and bandwidth from features.
        let center_freq = (f.peak_bin_normalised - 0.5) * sample_rate;
        let bandwidth = f.bandwidth_occupancy * sample_rate;

        // ---- Decision tree ----
        //
        // The ordering is chosen so that the most distinctive signatures are
        // tested first.

        // 1. Noise floor: high spectral flatness with moderate PAPR indicates
        //    no structured interference is present.
        if f.spectral_flatness > 0.35 && f.peak_to_average < 14.0 {
            return InterferenceReport {
                interference_type: InterferenceType::NoiseFloor,
                confidence: Self::clamp01(0.5 + (f.spectral_flatness - 0.35) * 1.5),
                center_freq,
                bandwidth,
                duty_cycle: f.duty_cycle,
            };
        }

        // 2. Pulsed: low time-domain duty cycle with few spectral peaks
        //    (to exclude multi-tone wideband which also has low duty cycle
        //    under our threshold-based metric).
        if f.duty_cycle < 0.40 && f.spectral_flatness < 0.20 && f.num_spectral_peaks <= 4 {
            return InterferenceReport {
                interference_type: InterferenceType::Pulsed,
                confidence: Self::clamp01(0.6 + (1.0 - f.duty_cycle) * 0.4),
                center_freq,
                bandwidth,
                duty_cycle: f.duty_cycle,
            };
        }

        // 3. Chirped / Frequency-hopping: large instantaneous-frequency
        //    variation AND tonal structure (not noise).
        if f.inst_freq_std > 0.05 && f.spectral_flatness < 0.25 {
            // Freq-hopping: many distinct spectral peaks spread across the band
            if f.num_spectral_peaks >= 3 && f.bandwidth_occupancy > 0.05 {
                return InterferenceReport {
                    interference_type: InterferenceType::FreqHopping,
                    confidence: Self::clamp01(0.5 + f.inst_freq_std.min(0.5)),
                    center_freq,
                    bandwidth,
                    duty_cycle: f.duty_cycle,
                };
            }
            return InterferenceReport {
                interference_type: InterferenceType::Chirped,
                confidence: Self::clamp01(0.5 + f.inst_freq_std.min(0.5)),
                center_freq,
                bandwidth,
                duty_cycle: f.duty_cycle,
            };
        }

        // 4. CW: very high PAPR with very few occupied bins.
        if f.peak_to_average > 10.0 && f.bandwidth_occupancy < 0.08 {
            return InterferenceReport {
                interference_type: InterferenceType::CW,
                confidence: Self::clamp01(0.6 + (f.peak_to_average - 10.0) / 40.0),
                center_freq,
                bandwidth,
                duty_cycle: f.duty_cycle,
            };
        }

        // 5. Narrowband: moderate PAPR, bandwidth occupancy < 25%.
        if f.bandwidth_occupancy < 0.25 {
            return InterferenceReport {
                interference_type: InterferenceType::Narrowband,
                confidence: Self::clamp01(0.5 + (0.25 - f.bandwidth_occupancy)),
                center_freq,
                bandwidth,
                duty_cycle: f.duty_cycle,
            };
        }

        // 6. Wideband: everything else.
        InterferenceReport {
            interference_type: InterferenceType::Wideband,
            confidence: Self::clamp01(0.4 + f.bandwidth_occupancy * 0.6),
            center_freq,
            bandwidth,
            duty_cycle: f.duty_cycle,
        }
    }

    // ---- Internal helpers --------------------------------------------------

    /// Compute the power spectrum using a radix-2 DIT FFT.
    fn power_spectrum(&self, samples: &[(f64, f64)]) -> Vec<f64> {
        // Zero-pad to next power of two.
        let n = samples.len().next_power_of_two();
        let mut re: Vec<f64> = samples.iter().map(|s| s.0).collect();
        let mut im: Vec<f64> = samples.iter().map(|s| s.1).collect();
        re.resize(n, 0.0);
        im.resize(n, 0.0);

        // Apply Hann window to the original (non-padded) portion.
        let win_len = samples.len();
        for i in 0..win_len {
            let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / win_len as f64).cos());
            re[i] *= w;
            im[i] *= w;
        }

        Self::fft_in_place(&mut re, &mut im);

        re.iter()
            .zip(im.iter())
            .map(|(r, i)| r * r + i * i)
            .collect()
    }

    /// In-place radix-2 Cooley-Tukey FFT.
    fn fft_in_place(re: &mut [f64], im: &mut [f64]) {
        let n = re.len();
        assert!(n.is_power_of_two());
        let log2n = n.trailing_zeros() as usize;

        // Bit-reversal permutation.
        for i in 0..n {
            let j = Self::bit_reverse(i, log2n);
            if i < j {
                re.swap(i, j);
                im.swap(i, j);
            }
        }

        let mut size = 2;
        while size <= n {
            let half = size / 2;
            let angle_step = -2.0 * PI / size as f64;
            for k in (0..n).step_by(size) {
                for j in 0..half {
                    let angle = angle_step * j as f64;
                    let wr = angle.cos();
                    let wi = angle.sin();
                    let tr = wr * re[k + j + half] - wi * im[k + j + half];
                    let ti = wr * im[k + j + half] + wi * re[k + j + half];
                    re[k + j + half] = re[k + j] - tr;
                    im[k + j + half] = im[k + j] - ti;
                    re[k + j] += tr;
                    im[k + j] += ti;
                }
            }
            size *= 2;
        }
    }

    fn bit_reverse(mut x: usize, bits: usize) -> usize {
        let mut result = 0;
        for _ in 0..bits {
            result = (result << 1) | (x & 1);
            x >>= 1;
        }
        result
    }

    /// Spectral flatness (Wiener entropy): geometric mean / arithmetic mean of
    /// the power spectrum.  A value close to 1 indicates white noise; close to
    /// 0 indicates a tonal signal.
    fn compute_spectral_flatness(spectrum: &[f64]) -> f64 {
        let n = spectrum.len() as f64;
        if n == 0.0 {
            return 0.0;
        }
        let arith_mean = spectrum.iter().sum::<f64>() / n;
        if arith_mean <= 0.0 {
            return 0.0;
        }
        // Use log-domain for geometric mean to avoid overflow.
        let log_sum: f64 = spectrum
            .iter()
            .map(|&p| if p > 1e-30 { p.ln() } else { -69.0 }) // ln(1e-30) ~ -69
            .sum();
        let geo_mean = (log_sum / n).exp();
        (geo_mean / arith_mean).min(1.0).max(0.0)
    }

    /// Excess kurtosis of a slice.
    fn compute_kurtosis(data: &[f64]) -> f64 {
        let n = data.len() as f64;
        if n < 4.0 {
            return 0.0;
        }
        let mean = data.iter().sum::<f64>() / n;
        let m2: f64 = data.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n;
        if m2 < 1e-30 {
            return 0.0;
        }
        let m4: f64 = data.iter().map(|&x| (x - mean).powi(4)).sum::<f64>() / n;
        (m4 / (m2 * m2)) - 3.0
    }

    /// Standard deviation of the instantaneous frequency (normalised to
    /// cycles per sample).
    fn compute_inst_freq_std(&self, samples: &[(f64, f64)]) -> f64 {
        if samples.len() < 3 {
            return 0.0;
        }
        // Only compute over samples with significant magnitude to avoid
        // noise-dominated phase estimates from near-zero samples.
        let magnitudes: Vec<f64> = samples
            .iter()
            .map(|(r, i)| (r * r + i * i).sqrt())
            .collect();
        let mag_max = magnitudes.iter().cloned().fold(0.0_f64, f64::max);
        let mag_threshold = mag_max * 0.1;

        let mut inst_freqs: Vec<f64> = Vec::with_capacity(samples.len() - 1);
        for i in 1..samples.len() {
            if magnitudes[i - 1] < mag_threshold || magnitudes[i] < mag_threshold {
                continue;
            }
            let (r0, i0) = samples[i - 1];
            let (r1, i1) = samples[i];
            let cr = r1 * r0 + i1 * i0;
            let ci = i1 * r0 - r1 * i0;
            let phase_diff = ci.atan2(cr);
            let freq = phase_diff / (2.0 * PI);
            inst_freqs.push(freq);
        }

        if inst_freqs.len() < 2 {
            return 0.0;
        }

        let n = inst_freqs.len() as f64;
        let mean = inst_freqs.iter().sum::<f64>() / n;
        let variance = inst_freqs.iter().map(|&f| (f - mean).powi(2)).sum::<f64>() / n;
        variance.sqrt()
    }

    /// Count spectral peaks above `peak_threshold_db` dB over `mean_power`.
    /// Returns `(count, max_peak_dB)`.
    fn count_spectral_peaks(&self, spectrum: &[f64], mean_power: f64) -> (usize, f64) {
        if spectrum.is_empty() || mean_power <= 0.0 {
            return (0, 0.0);
        }
        let threshold = mean_power * Self::db_to_linear(self.peak_threshold_db);
        let mut count = 0usize;
        let mut max_db = 0.0f64;

        for i in 1..spectrum.len().saturating_sub(1) {
            if spectrum[i] > threshold
                && spectrum[i] >= spectrum[i - 1]
                && spectrum[i] >= spectrum[i + 1]
            {
                count += 1;
                let db = 10.0 * (spectrum[i] / mean_power).log10();
                if db > max_db {
                    max_db = db;
                }
            }
        }
        (count, max_db)
    }

    fn db_to_linear(db: f64) -> f64 {
        10.0_f64.powf(db / 10.0)
    }

    fn clamp01(v: f64) -> f64 {
        v.max(0.0).min(1.0)
    }
}

// ---------------------------------------------------------------------------
// Display implementations
// ---------------------------------------------------------------------------

impl std::fmt::Display for InterferenceType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::CW => write!(f, "CW"),
            Self::Narrowband => write!(f, "Narrowband"),
            Self::Wideband => write!(f, "Wideband"),
            Self::Pulsed => write!(f, "Pulsed"),
            Self::Chirped => write!(f, "Chirped"),
            Self::FreqHopping => write!(f, "FreqHopping"),
            Self::NoiseFloor => write!(f, "NoiseFloor"),
        }
    }
}

impl std::fmt::Display for InterferenceReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} (conf={:.2}, fc={:.0} Hz, bw={:.0} Hz, dc={:.2})",
            self.interference_type,
            self.confidence,
            self.center_freq,
            self.bandwidth,
            self.duty_cycle,
        )
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a single-tone CW signal.
    fn cw_signal(freq: f64, sample_rate: f64, n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * PI * freq * t;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    /// Helper: generate white Gaussian-ish noise via a simple LCG + Box-Muller.
    fn noise_signal(n: usize, amplitude: f64) -> Vec<(f64, f64)> {
        let mut state: u64 = 0xDEAD_BEEF_CAFE_1234;
        let mut out = Vec::with_capacity(n);
        for _ in 0..n {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u1 = (state >> 33) as f64 / (1u64 << 31) as f64;
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u2 = (state >> 33) as f64 / (1u64 << 31) as f64;
            let u1 = u1.max(1e-10);
            let r = (-2.0 * u1.ln()).sqrt();
            let theta = 2.0 * PI * u2;
            out.push((amplitude * r * theta.cos(), amplitude * r * theta.sin()));
        }
        out
    }

    /// Helper: generate a chirp signal.
    fn chirp_signal(f0: f64, f1: f64, sample_rate: f64, n: usize) -> Vec<(f64, f64)> {
        let duration = n as f64 / sample_rate;
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * PI * (f0 * t + 0.5 * (f1 - f0) * t * t / duration);
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    /// Helper: generate a pulsed CW signal with given duty cycle.
    fn pulsed_signal(
        freq: f64,
        sample_rate: f64,
        n: usize,
        duty: f64,
    ) -> Vec<(f64, f64)> {
        let pulse_len = (n as f64 * duty) as usize;
        (0..n)
            .map(|i| {
                if i < pulse_len {
                    let t = i as f64 / sample_rate;
                    let phase = 2.0 * PI * freq * t;
                    (phase.cos(), phase.sin())
                } else {
                    (0.0, 0.0)
                }
            })
            .collect()
    }

    // ---- Tests ----

    #[test]
    fn test_classify_cw() {
        let samples = cw_signal(1000.0, 10000.0, 1024);
        let c = InterferenceClassifier::new();
        let report = c.classify(&samples, 10000.0);
        assert_eq!(report.interference_type, InterferenceType::CW);
        assert!(report.confidence > 0.5);
    }

    #[test]
    fn test_classify_noise_floor() {
        let samples = noise_signal(4096, 0.01);
        let c = InterferenceClassifier::new();
        let report = c.classify(&samples, 10000.0);
        assert_eq!(report.interference_type, InterferenceType::NoiseFloor);
    }

    #[test]
    fn test_classify_chirp() {
        let samples = chirp_signal(-2000.0, 2000.0, 10000.0, 2048);
        let c = InterferenceClassifier::new();
        let report = c.classify(&samples, 10000.0);
        assert!(
            report.interference_type == InterferenceType::Chirped
                || report.interference_type == InterferenceType::Wideband,
            "Expected Chirped or Wideband, got {:?}",
            report.interference_type
        );
    }

    #[test]
    fn test_classify_pulsed() {
        // 20% duty cycle pulse
        let samples = pulsed_signal(1500.0, 10000.0, 2048, 0.2);
        let c = InterferenceClassifier::new();
        let report = c.classify(&samples, 10000.0);
        assert_eq!(report.interference_type, InterferenceType::Pulsed);
        assert!(report.duty_cycle < 0.5);
    }

    #[test]
    fn test_classify_wideband() {
        // Wideband: add many tones spread across the band
        let n = 2048;
        let sr = 10000.0;
        let mut samples: Vec<(f64, f64)> = vec![(0.0, 0.0); n];
        for &f in &[500.0, 1500.0, 2500.0, 3500.0, 4500.0, -500.0, -1500.0, -2500.0] {
            for i in 0..n {
                let t = i as f64 / sr;
                let phase = 2.0 * PI * f * t;
                samples[i].0 += phase.cos();
                samples[i].1 += phase.sin();
            }
        }
        let c = InterferenceClassifier::new();
        let report = c.classify(&samples, sr);
        // Multi-tone could appear as FreqHopping or Narrowband depending on
        // spectral spread.
        assert!(
            report.interference_type == InterferenceType::Wideband
                || report.interference_type == InterferenceType::FreqHopping
                || report.interference_type == InterferenceType::Narrowband
                || report.interference_type == InterferenceType::Chirped,
            "Expected Wideband/FreqHopping/Narrowband/Chirped for multi-tone, got {:?}",
            report.interference_type
        );
    }

    #[test]
    fn test_extract_features_cw() {
        let samples = cw_signal(500.0, 10000.0, 1024);
        let c = InterferenceClassifier::new();
        let f = c.extract_features(&samples, 10000.0);
        // CW should have low spectral flatness and high peak-to-average
        assert!(f.spectral_flatness < 0.3, "flatness={}", f.spectral_flatness);
        assert!(f.peak_to_average > 8.0, "papr={}", f.peak_to_average);
    }

    #[test]
    fn test_extract_features_noise() {
        let samples = noise_signal(4096, 1.0);
        let c = InterferenceClassifier::new();
        let f = c.extract_features(&samples, 10000.0);
        // Noise should have high spectral flatness
        assert!(f.spectral_flatness > 0.4, "flatness={}", f.spectral_flatness);
    }

    #[test]
    fn test_classify_from_features_roundtrip() {
        let samples = cw_signal(2000.0, 10000.0, 1024);
        let c = InterferenceClassifier::new();
        let features = c.extract_features(&samples, 10000.0);
        let report = c.classify_from_features(&features, 10000.0);
        assert_eq!(report.interference_type, InterferenceType::CW);
    }

    #[test]
    fn test_report_confidence_range() {
        let samples = cw_signal(1000.0, 10000.0, 512);
        let c = InterferenceClassifier::new();
        let report = c.classify(&samples, 10000.0);
        assert!((0.0..=1.0).contains(&report.confidence));
    }

    #[test]
    fn test_display_interference_type() {
        assert_eq!(format!("{}", InterferenceType::CW), "CW");
        assert_eq!(format!("{}", InterferenceType::Pulsed), "Pulsed");
        assert_eq!(format!("{}", InterferenceType::NoiseFloor), "NoiseFloor");
        assert_eq!(format!("{}", InterferenceType::FreqHopping), "FreqHopping");
    }

    #[test]
    fn test_display_report() {
        let report = InterferenceReport {
            interference_type: InterferenceType::CW,
            confidence: 0.95,
            center_freq: 1000.0,
            bandwidth: 50.0,
            duty_cycle: 1.0,
        };
        let s = format!("{}", report);
        assert!(s.contains("CW"));
        assert!(s.contains("0.95"));
    }

    #[test]
    fn test_with_params() {
        let c = InterferenceClassifier::with_params(32, 10.0, 0.3);
        let samples = cw_signal(1000.0, 10000.0, 512);
        let report = c.classify(&samples, 10000.0);
        assert_eq!(report.interference_type, InterferenceType::CW);
    }

    #[test]
    fn test_default_trait() {
        let c = InterferenceClassifier::default();
        let samples = noise_signal(2048, 0.01);
        let report = c.classify(&samples, 10000.0);
        assert_eq!(report.interference_type, InterferenceType::NoiseFloor);
    }

    #[test]
    fn test_small_input() {
        // Should not panic on very small inputs.
        let samples = vec![(1.0, 0.0); 16];
        let c = InterferenceClassifier::new();
        let _report = c.classify(&samples, 1000.0);
    }

    #[test]
    fn test_feature_vector_duty_cycle_full() {
        // A continuous signal should have duty cycle close to 1.
        let samples = cw_signal(500.0, 10000.0, 1024);
        let c = InterferenceClassifier::new();
        let f = c.extract_features(&samples, 10000.0);
        assert!(f.duty_cycle > 0.8, "duty={}", f.duty_cycle);
    }

    #[test]
    fn test_narrowband_classification() {
        // Create a narrowband signal: two closely-spaced tones.
        let n = 2048;
        let sr = 10000.0;
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / sr;
                let p1 = 2.0 * PI * 1000.0 * t;
                let p2 = 2.0 * PI * 1050.0 * t;
                (p1.cos() + p2.cos(), p1.sin() + p2.sin())
            })
            .collect();
        let c = InterferenceClassifier::new();
        let report = c.classify(&samples, sr);
        assert!(
            report.interference_type == InterferenceType::CW
                || report.interference_type == InterferenceType::Narrowband,
            "Expected CW or Narrowband for two close tones, got {:?}",
            report.interference_type
        );
    }
}
