//! Wideband cooperative spectrum sensing for cognitive radio using energy detection fusion.
//!
//! This module provides blind spectrum sensing based on FFT energy detection with
//! Neyman-Pearson thresholding and cooperative fusion across multiple sensor nodes.
//!
//! # Example
//!
//! ```
//! use r4w_core::blind_spectrum_sensing::{BlindSpectrumSensor, SensingConfig, FusionRule, CooperativeFusion};
//!
//! let config = SensingConfig {
//!     fft_size: 64,
//!     sample_rate: 1_000_000.0,
//!     num_bands: 4,
//!     false_alarm_prob: 0.01,
//! };
//!
//! let sensor = BlindSpectrumSensor::new(config);
//!
//! // Generate some test samples (tone at ~250 kHz in a 1 MHz band)
//! let num_samples = 64;
//! let samples: Vec<(f64, f64)> = (0..num_samples)
//!     .map(|i| {
//!         let t = i as f64 / 1_000_000.0;
//!         let freq = 250_000.0;
//!         let phase = 2.0 * std::f64::consts::PI * freq * t;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! let results = sensor.sense(&samples);
//! assert_eq!(results.len(), 4);
//!
//! // Cooperative fusion from two sensors
//! let fusion = CooperativeFusion::new(FusionRule::Or);
//! let sensor2_results = sensor.sense(&samples);
//! let fused = fusion.fuse(&[results, sensor2_results]);
//! assert_eq!(fused.len(), 4);
//! ```

use std::f64::consts::PI;

/// Configuration for the blind spectrum sensor.
#[derive(Debug, Clone)]
pub struct SensingConfig {
    /// FFT size used for spectral analysis (must be a power of 2).
    pub fft_size: usize,
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Number of frequency bands to divide the spectrum into.
    pub num_bands: usize,
    /// Target probability of false alarm (Pfa) for Neyman-Pearson threshold.
    pub false_alarm_prob: f64,
}

/// Result for a single frequency band after sensing.
#[derive(Debug, Clone)]
pub struct BandResult {
    /// Index of the band (0-based).
    pub band_index: usize,
    /// Center frequency of this band in Hz.
    pub center_freq_hz: f64,
    /// Bandwidth of this band in Hz.
    pub bandwidth_hz: f64,
    /// Measured energy in dB.
    pub energy_db: f64,
    /// Whether the band is detected as occupied.
    pub is_occupied: bool,
    /// Detection confidence in [0, 1].
    pub confidence: f64,
}

/// Fusion rules for cooperative spectrum sensing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FusionRule {
    /// OR rule: band is occupied if ANY sensor detects it (conservative, high Pd).
    Or,
    /// AND rule: band is occupied only if ALL sensors detect it (aggressive, low Pfa).
    And,
    /// Majority vote: band is occupied if more than half the sensors detect it.
    Majority,
    /// Maximal ratio combining: weight by each sensor's confidence.
    MaximalRatio,
}

/// Blind spectrum sensor using FFT-based energy detection.
pub struct BlindSpectrumSensor {
    config: SensingConfig,
}

impl BlindSpectrumSensor {
    /// Create a new blind spectrum sensor with the given configuration.
    pub fn new(config: SensingConfig) -> Self {
        Self { config }
    }

    /// Perform spectrum sensing on the provided IQ samples.
    ///
    /// Returns a `BandResult` for each of the configured frequency bands.
    /// The samples are analyzed using an FFT-based energy detector with
    /// Neyman-Pearson thresholding derived from the target false alarm probability.
    pub fn sense(&self, samples: &[(f64, f64)]) -> Vec<BandResult> {
        let fft_size = self.config.fft_size;
        let num_bands = self.config.num_bands;
        let sample_rate = self.config.sample_rate;

        // Compute power spectrum via DFT
        let spectrum = self.compute_power_spectrum(samples);

        // Estimate noise floor from the full spectrum (median-based for robustness)
        let noise_floor_db = self.estimate_noise_floor(&spectrum);

        // Compute the adaptive threshold
        let threshold_db = self.adaptive_threshold(noise_floor_db);

        // Number of FFT bins per band
        let bins_per_band = fft_size / num_bands;
        let band_bw = sample_rate / num_bands as f64;

        let mut results = Vec::with_capacity(num_bands);

        for band_idx in 0..num_bands {
            let start_bin = band_idx * bins_per_band;
            let end_bin = start_bin + bins_per_band;

            // Average energy in this band (in linear power)
            let mut band_energy_linear = 0.0;
            let count = (end_bin.min(spectrum.len()) - start_bin.min(spectrum.len())).max(1);
            for bin in start_bin..end_bin.min(spectrum.len()) {
                band_energy_linear += spectrum[bin];
            }
            band_energy_linear /= count as f64;

            let energy_db = 10.0 * band_energy_linear.max(1e-30).log10();

            let is_occupied = energy_db > threshold_db;

            // Confidence: how far above/below threshold, mapped through a sigmoid
            let margin = energy_db - threshold_db;
            let confidence = 1.0 / (1.0 + (-margin * 0.5).exp());

            // Center frequency: bands span [0, fs)
            let center_freq_hz = (band_idx as f64 + 0.5) * band_bw;

            results.push(BandResult {
                band_index: band_idx,
                center_freq_hz,
                bandwidth_hz: band_bw,
                energy_db,
                is_occupied,
                confidence,
            });
        }

        results
    }

    /// Compute the adaptive detection threshold from the noise floor
    /// using the Neyman-Pearson criterion for the target false alarm probability.
    ///
    /// For energy detection with M samples, under H0 (noise only), the test
    /// statistic follows a chi-squared distribution. We approximate the threshold
    /// using the inverse complementary error function relationship:
    ///
    ///   threshold = noise_floor + margin_db
    ///
    /// where the margin is derived from Pfa.
    pub fn adaptive_threshold(&self, noise_floor_db: f64) -> f64 {
        let pfa = self.config.false_alarm_prob.clamp(1e-12, 1.0 - 1e-12);

        // For energy detection, threshold above noise floor is related to Pfa by:
        //   Pfa = Q(sqrt(2*M) * (threshold/noise - 1))
        // Practical approximation:
        //   margin_db = -10 * log10(Pfa) / sqrt(bins_per_band)
        let bins_per_band = (self.config.fft_size / self.config.num_bands).max(1);
        let margin_db = -10.0 * pfa.log10() / (bins_per_band as f64).sqrt();

        noise_floor_db + margin_db
    }

    /// Compute the power spectrum (magnitude squared) of the input samples
    /// using a radix-2 DIT FFT. Returns linear power per bin.
    fn compute_power_spectrum(&self, samples: &[(f64, f64)]) -> Vec<f64> {
        let fft_size = self.config.fft_size;

        // Prepare input buffer, zero-pad or truncate as needed
        let mut re = vec![0.0; fft_size];
        let mut im = vec![0.0; fft_size];

        let n = samples.len().min(fft_size);
        for i in 0..n {
            re[i] = samples[i].0;
            im[i] = samples[i].1;
        }

        // Apply Hann window for spectral leakage reduction
        for i in 0..fft_size {
            let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / fft_size as f64).cos());
            re[i] *= w;
            im[i] *= w;
        }

        // In-place radix-2 FFT
        fft_in_place(&mut re, &mut im);

        // Power spectrum (magnitude squared), normalized by FFT size
        let norm = 1.0 / (fft_size as f64 * fft_size as f64);
        re.iter()
            .zip(im.iter())
            .map(|(&r, &i)| (r * r + i * i) * norm)
            .collect()
    }

    /// Estimate the noise floor from the power spectrum using sorted-median approach.
    fn estimate_noise_floor(&self, spectrum: &[f64]) -> f64 {
        if spectrum.is_empty() {
            return -100.0;
        }

        let mut sorted: Vec<f64> = spectrum.iter().copied().collect();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        // Use the 25th percentile as a robust noise floor estimate
        let idx = sorted.len() / 4;
        let noise_linear = sorted[idx].max(1e-30);
        10.0 * noise_linear.log10()
    }
}

/// Cooperative fusion engine for combining sensing results from multiple sensors.
pub struct CooperativeFusion {
    rule: FusionRule,
    history: std::cell::RefCell<Vec<Vec<bool>>>,
}

impl CooperativeFusion {
    /// Create a new cooperative fusion engine with the given fusion rule.
    pub fn new(rule: FusionRule) -> Self {
        Self {
            rule,
            history: std::cell::RefCell::new(Vec::new()),
        }
    }

    /// Fuse results from multiple sensor nodes into a single set of band results.
    ///
    /// All input result vectors must have the same number of bands.
    /// The fused result uses the geometry (center_freq, bandwidth) from the first sensor.
    pub fn fuse(&self, results: &[Vec<BandResult>]) -> Vec<BandResult> {
        if results.is_empty() {
            return Vec::new();
        }

        let num_bands = results[0].len();
        let num_sensors = results.len();

        // Validate all sensors report the same number of bands
        for r in results {
            assert_eq!(
                r.len(),
                num_bands,
                "All sensors must report the same number of bands"
            );
        }

        let mut fused = Vec::with_capacity(num_bands);
        let mut occupancy_snapshot = Vec::with_capacity(num_bands);

        for band_idx in 0..num_bands {
            let ref_band = &results[0][band_idx];

            let occupied_count = results
                .iter()
                .filter(|r| r[band_idx].is_occupied)
                .count();

            let confidences: Vec<f64> = results.iter().map(|r| r[band_idx].confidence).collect();
            let energies: Vec<f64> = results.iter().map(|r| r[band_idx].energy_db).collect();

            let (is_occupied, confidence) = match self.rule {
                FusionRule::Or => {
                    let occ = occupied_count > 0;
                    let conf = confidences
                        .iter()
                        .copied()
                        .fold(0.0_f64, f64::max);
                    (occ, conf)
                }
                FusionRule::And => {
                    let occ = occupied_count == num_sensors;
                    let conf = confidences
                        .iter()
                        .copied()
                        .fold(1.0_f64, f64::min);
                    (occ, conf)
                }
                FusionRule::Majority => {
                    let occ = occupied_count * 2 > num_sensors;
                    let conf = confidences.iter().sum::<f64>() / num_sensors as f64;
                    (occ, conf)
                }
                FusionRule::MaximalRatio => {
                    // Weighted combination: sum(energy_i * confidence_i) / sum(confidence_i)
                    let weight_sum: f64 = confidences.iter().sum();
                    let weighted_energy: f64 = energies
                        .iter()
                        .zip(confidences.iter())
                        .map(|(e, c)| e * c)
                        .sum();
                    let avg_energy = if weight_sum > 0.0 {
                        weighted_energy / weight_sum
                    } else {
                        energies.iter().sum::<f64>() / num_sensors as f64
                    };

                    // Re-derive occupancy from the combined confidence
                    let avg_conf = if weight_sum > 0.0 {
                        confidences
                            .iter()
                            .map(|c| c * c)
                            .sum::<f64>()
                            / weight_sum
                    } else {
                        0.0
                    };
                    let energy_avg = energies.iter().sum::<f64>() / num_sensors as f64;
                    let occ = avg_conf > 0.5 && avg_energy > energy_avg - 3.0;
                    (occ, avg_conf)
                }
            };

            // Average energy across sensors for fused result
            let avg_energy_db = energies.iter().sum::<f64>() / num_sensors as f64;

            occupancy_snapshot.push(is_occupied);

            fused.push(BandResult {
                band_index: ref_band.band_index,
                center_freq_hz: ref_band.center_freq_hz,
                bandwidth_hz: ref_band.bandwidth_hz,
                energy_db: avg_energy_db,
                is_occupied,
                confidence,
            });
        }

        // Record occupancy history
        self.history.borrow_mut().push(occupancy_snapshot);

        fused
    }

    /// Return the occupancy history: each entry is a snapshot of band occupancy
    /// (one bool per band) from each call to `fuse`.
    pub fn occupancy_history(&self) -> Vec<Vec<bool>> {
        self.history.borrow().clone()
    }

    /// Predict the probability that a given band will be occupied in the future,
    /// based on the historical occupancy rate (exponentially weighted moving average).
    ///
    /// Returns a value in [0.0, 1.0]. If no history is available, returns 0.5 (unknown).
    pub fn prediction(&self, band: usize) -> f64 {
        let history = self.history.borrow();
        if history.is_empty() {
            return 0.5;
        }

        // Exponentially weighted moving average with decay factor alpha
        let alpha = 0.3;
        let mut ewma = 0.5; // prior

        for snapshot in history.iter() {
            if band < snapshot.len() {
                let obs = if snapshot[band] { 1.0 } else { 0.0 };
                ewma = alpha * obs + (1.0 - alpha) * ewma;
            }
        }

        ewma
    }
}

/// In-place radix-2 decimation-in-time FFT.
///
/// `re` and `im` must have the same length, which must be a power of 2.
fn fft_in_place(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    assert_eq!(n, im.len());
    if n <= 1 {
        return;
    }
    // n must be a power of 2
    assert!(n.is_power_of_two(), "FFT size must be a power of 2");

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 0..n {
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
        let mut m = n >> 1;
        while m >= 1 && j >= m {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // Cooley-Tukey butterfly
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle_step = -2.0 * PI / len as f64;
        for start in (0..n).step_by(len) {
            for k in 0..half {
                let angle = angle_step * k as f64;
                let wr = angle.cos();
                let wi = angle.sin();

                let a = start + k;
                let b = start + k + half;

                let tr = wr * re[b] - wi * im[b];
                let ti = wr * im[b] + wi * re[b];

                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
            }
        }
        len <<= 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> SensingConfig {
        SensingConfig {
            fft_size: 64,
            sample_rate: 1_000_000.0,
            num_bands: 4,
            false_alarm_prob: 0.01,
        }
    }

    fn make_tone(freq: f64, sample_rate: f64, num_samples: usize, amplitude: f64) -> Vec<(f64, f64)> {
        (0..num_samples)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * PI * freq * t;
                (amplitude * phase.cos(), amplitude * phase.sin())
            })
            .collect()
    }

    fn make_noise(num_samples: usize, power: f64) -> Vec<(f64, f64)> {
        // Simple deterministic pseudo-noise using a linear congruential generator
        let mut seed: u64 = 12345;
        (0..num_samples)
            .map(|_| {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                let r1 = ((seed >> 33) as f64) / (u32::MAX as f64) - 0.5;
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                let r2 = ((seed >> 33) as f64) / (u32::MAX as f64) - 0.5;
                (r1 * power.sqrt(), r2 * power.sqrt())
            })
            .collect()
    }

    #[test]
    fn test_sensor_creation() {
        let config = default_config();
        let _sensor = BlindSpectrumSensor::new(config);
    }

    #[test]
    fn test_sense_returns_correct_num_bands() {
        let config = default_config();
        let sensor = BlindSpectrumSensor::new(config);
        let samples = make_noise(64, 0.001);
        let results = sensor.sense(&samples);
        assert_eq!(results.len(), 4);
    }

    #[test]
    fn test_band_frequencies() {
        let config = SensingConfig {
            fft_size: 128,
            sample_rate: 2_000_000.0,
            num_bands: 4,
            false_alarm_prob: 0.01,
        };
        let sensor = BlindSpectrumSensor::new(config);
        let samples = make_noise(128, 0.001);
        let results = sensor.sense(&samples);

        // Band BW should be 2 MHz / 4 = 500 kHz
        for r in &results {
            assert!((r.bandwidth_hz - 500_000.0).abs() < 1e-6);
        }
        // Center frequencies: 250, 750, 1250, 1750 kHz
        assert!((results[0].center_freq_hz - 250_000.0).abs() < 1e-6);
        assert!((results[1].center_freq_hz - 750_000.0).abs() < 1e-6);
        assert!((results[2].center_freq_hz - 1_250_000.0).abs() < 1e-6);
        assert!((results[3].center_freq_hz - 1_750_000.0).abs() < 1e-6);
    }

    #[test]
    fn test_tone_detection() {
        // Place a strong tone in the second band (250-500 kHz for 1 MHz / 4 bands)
        let config = SensingConfig {
            fft_size: 256,
            sample_rate: 1_000_000.0,
            num_bands: 4,
            false_alarm_prob: 0.01,
        };
        let sensor = BlindSpectrumSensor::new(config);

        // Tone at 375 kHz (center of band index 1: 250-500 kHz) plus low noise
        let mut samples = make_noise(256, 0.0001);
        let tone = make_tone(375_000.0, 1_000_000.0, 256, 1.0);
        for i in 0..256 {
            samples[i].0 += tone[i].0;
            samples[i].1 += tone[i].1;
        }

        let results = sensor.sense(&samples);

        // The band containing the tone should have much higher energy
        let tone_band_energy = results[1].energy_db;
        let noise_band_energy = results[0].energy_db;
        assert!(
            tone_band_energy > noise_band_energy + 10.0,
            "Tone band energy ({}) should be well above noise band ({})",
            tone_band_energy,
            noise_band_energy
        );
    }

    #[test]
    fn test_adaptive_threshold_increases_with_stricter_pfa() {
        let config1 = SensingConfig {
            fft_size: 64,
            sample_rate: 1_000_000.0,
            num_bands: 4,
            false_alarm_prob: 0.1,
        };
        let config2 = SensingConfig {
            fft_size: 64,
            sample_rate: 1_000_000.0,
            num_bands: 4,
            false_alarm_prob: 0.001,
        };
        let sensor1 = BlindSpectrumSensor::new(config1);
        let sensor2 = BlindSpectrumSensor::new(config2);

        let nf = -80.0;
        let t1 = sensor1.adaptive_threshold(nf);
        let t2 = sensor2.adaptive_threshold(nf);

        // Lower Pfa means higher threshold
        assert!(
            t2 > t1,
            "Stricter Pfa should produce higher threshold: t2={} vs t1={}",
            t2,
            t1
        );
    }

    #[test]
    fn test_adaptive_threshold_above_noise_floor() {
        let config = default_config();
        let sensor = BlindSpectrumSensor::new(config);
        let nf = -90.0;
        let thresh = sensor.adaptive_threshold(nf);
        assert!(thresh > nf, "Threshold {} must be above noise floor {}", thresh, nf);
    }

    #[test]
    fn test_confidence_range() {
        let config = default_config();
        let sensor = BlindSpectrumSensor::new(config);
        let samples = make_tone(100_000.0, 1_000_000.0, 64, 1.0);
        let results = sensor.sense(&samples);

        for r in &results {
            assert!(
                r.confidence >= 0.0 && r.confidence <= 1.0,
                "Confidence {} out of [0,1] range",
                r.confidence
            );
        }
    }

    #[test]
    fn test_fusion_or_rule() {
        let fusion = CooperativeFusion::new(FusionRule::Or);

        let band_occupied = BandResult {
            band_index: 0, center_freq_hz: 250_000.0, bandwidth_hz: 500_000.0,
            energy_db: -50.0, is_occupied: true, confidence: 0.9,
        };
        let band_empty = BandResult {
            band_index: 0, center_freq_hz: 250_000.0, bandwidth_hz: 500_000.0,
            energy_db: -90.0, is_occupied: false, confidence: 0.2,
        };

        // OR: if any sensor says occupied, fused says occupied
        let fused = fusion.fuse(&[vec![band_occupied.clone()], vec![band_empty.clone()]]);
        assert!(fused[0].is_occupied, "OR rule should detect occupation if any sensor does");
    }

    #[test]
    fn test_fusion_and_rule() {
        let fusion = CooperativeFusion::new(FusionRule::And);

        let band_occupied = BandResult {
            band_index: 0, center_freq_hz: 250_000.0, bandwidth_hz: 500_000.0,
            energy_db: -50.0, is_occupied: true, confidence: 0.9,
        };
        let band_empty = BandResult {
            band_index: 0, center_freq_hz: 250_000.0, bandwidth_hz: 500_000.0,
            energy_db: -90.0, is_occupied: false, confidence: 0.2,
        };

        // AND: all sensors must agree
        let fused = fusion.fuse(&[vec![band_occupied.clone()], vec![band_empty.clone()]]);
        assert!(!fused[0].is_occupied, "AND rule should not detect if not all sensors agree");

        // Both occupied
        let fused2 = fusion.fuse(&[vec![band_occupied.clone()], vec![band_occupied.clone()]]);
        assert!(fused2[0].is_occupied, "AND rule should detect if all sensors agree");
    }

    #[test]
    fn test_fusion_majority_rule() {
        let fusion = CooperativeFusion::new(FusionRule::Majority);

        let occ = BandResult {
            band_index: 0, center_freq_hz: 250_000.0, bandwidth_hz: 500_000.0,
            energy_db: -50.0, is_occupied: true, confidence: 0.8,
        };
        let empty = BandResult {
            band_index: 0, center_freq_hz: 250_000.0, bandwidth_hz: 500_000.0,
            energy_db: -90.0, is_occupied: false, confidence: 0.2,
        };

        // 2 out of 3 say occupied -> majority
        let fused = fusion.fuse(&[vec![occ.clone()], vec![occ.clone()], vec![empty.clone()]]);
        assert!(fused[0].is_occupied, "Majority (2/3) should detect occupation");

        // 1 out of 3 -> not majority
        let fused2 = fusion.fuse(&[vec![occ.clone()], vec![empty.clone()], vec![empty.clone()]]);
        assert!(!fused2[0].is_occupied, "Minority (1/3) should not detect occupation");
    }

    #[test]
    fn test_fusion_maximal_ratio() {
        let fusion = CooperativeFusion::new(FusionRule::MaximalRatio);

        let high_conf = BandResult {
            band_index: 0, center_freq_hz: 250_000.0, bandwidth_hz: 500_000.0,
            energy_db: -40.0, is_occupied: true, confidence: 0.95,
        };
        let low_conf = BandResult {
            band_index: 0, center_freq_hz: 250_000.0, bandwidth_hz: 500_000.0,
            energy_db: -85.0, is_occupied: false, confidence: 0.1,
        };

        let fused = fusion.fuse(&[vec![high_conf], vec![low_conf]]);
        // High-confidence sensor should dominate
        assert_eq!(fused.len(), 1);
        // Confidence should be finite and in reasonable range
        assert!(fused[0].confidence >= 0.0 && fused[0].confidence <= 1.0);
    }

    #[test]
    fn test_occupancy_history_tracking() {
        let fusion = CooperativeFusion::new(FusionRule::Or);

        let occ = BandResult {
            band_index: 0, center_freq_hz: 250_000.0, bandwidth_hz: 500_000.0,
            energy_db: -50.0, is_occupied: true, confidence: 0.9,
        };
        let empty = BandResult {
            band_index: 0, center_freq_hz: 250_000.0, bandwidth_hz: 500_000.0,
            energy_db: -90.0, is_occupied: false, confidence: 0.1,
        };

        fusion.fuse(&[vec![occ.clone()]]);
        fusion.fuse(&[vec![empty.clone()]]);
        fusion.fuse(&[vec![occ.clone()]]);

        let history = fusion.occupancy_history();
        assert_eq!(history.len(), 3);
        assert_eq!(history[0], vec![true]);
        assert_eq!(history[1], vec![false]);
        assert_eq!(history[2], vec![true]);
    }

    #[test]
    fn test_prediction_no_history() {
        let fusion = CooperativeFusion::new(FusionRule::Or);
        let pred = fusion.prediction(0);
        assert!((pred - 0.5).abs() < 1e-10, "No history should return 0.5, got {}", pred);
    }

    #[test]
    fn test_prediction_all_occupied() {
        let fusion = CooperativeFusion::new(FusionRule::Or);

        let occ = BandResult {
            band_index: 0, center_freq_hz: 250_000.0, bandwidth_hz: 500_000.0,
            energy_db: -50.0, is_occupied: true, confidence: 0.9,
        };

        // Feed 20 occupied observations to let EWMA converge
        for _ in 0..20 {
            fusion.fuse(&[vec![occ.clone()]]);
        }

        let pred = fusion.prediction(0);
        assert!(
            pred > 0.9,
            "Prediction after many occupied observations should be high, got {}",
            pred
        );
    }

    #[test]
    fn test_prediction_all_empty() {
        let fusion = CooperativeFusion::new(FusionRule::Or);

        let empty = BandResult {
            band_index: 0, center_freq_hz: 250_000.0, bandwidth_hz: 500_000.0,
            energy_db: -95.0, is_occupied: false, confidence: 0.1,
        };

        for _ in 0..20 {
            fusion.fuse(&[vec![empty.clone()]]);
        }

        let pred = fusion.prediction(0);
        assert!(
            pred < 0.1,
            "Prediction after many empty observations should be low, got {}",
            pred
        );
    }

    #[test]
    fn test_fft_dc_component() {
        // A constant signal should produce energy only at DC (bin 0 / band 0)
        let config = SensingConfig {
            fft_size: 64,
            sample_rate: 1_000_000.0,
            num_bands: 4,
            false_alarm_prob: 0.01,
        };
        let sensor = BlindSpectrumSensor::new(config);

        let dc_signal: Vec<(f64, f64)> = vec![(1.0, 0.0); 64];
        let results = sensor.sense(&dc_signal);

        // Band 0 (DC) should have higher energy than other bands
        assert!(
            results[0].energy_db > results[2].energy_db + 10.0,
            "DC band energy ({}) should dominate over other bands ({})",
            results[0].energy_db,
            results[2].energy_db
        );
    }

    #[test]
    fn test_empty_samples() {
        let config = default_config();
        let sensor = BlindSpectrumSensor::new(config);
        let samples: Vec<(f64, f64)> = vec![];
        let results = sensor.sense(&samples);
        // Should still return band results (all zero-padded)
        assert_eq!(results.len(), 4);
    }

    #[test]
    fn test_multi_band_sensing() {
        // Two tones in different bands
        let config = SensingConfig {
            fft_size: 256,
            sample_rate: 1_000_000.0,
            num_bands: 8,
            false_alarm_prob: 0.01,
        };
        let sensor = BlindSpectrumSensor::new(config);

        // Tone at ~62.5 kHz (band 0: 0-125 kHz) and ~562.5 kHz (band 4: 500-625 kHz)
        let mut samples = make_noise(256, 0.0001);
        let tone1 = make_tone(62_500.0, 1_000_000.0, 256, 1.0);
        let tone2 = make_tone(562_500.0, 1_000_000.0, 256, 1.0);
        for i in 0..256 {
            samples[i].0 += tone1[i].0 + tone2[i].0;
            samples[i].1 += tone1[i].1 + tone2[i].1;
        }

        let results = sensor.sense(&samples);
        assert_eq!(results.len(), 8);

        // Bands 0 and 4 should have noticeably higher energy than quiet bands
        let quiet_energy = results[2].energy_db;
        assert!(
            results[0].energy_db > quiet_energy + 5.0,
            "Band 0 with tone ({}) should be above quiet band ({})",
            results[0].energy_db,
            quiet_energy
        );
        assert!(
            results[4].energy_db > quiet_energy + 5.0,
            "Band 4 with tone ({}) should be above quiet band ({})",
            results[4].energy_db,
            quiet_energy
        );
    }
}
