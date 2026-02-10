//! # Spectrum Hole Detector
//!
//! Cognitive radio spectrum sensing for dynamic spectrum access. This module
//! detects unused frequency bands ("spectrum holes") in a given bandwidth by
//! performing FFT-based energy detection, estimating the noise floor, and
//! identifying contiguous regions below a configurable threshold.
//!
//! ## Algorithm Overview
//!
//! 1. Collect IQ samples and compute power spectral density via FFT.
//! 2. Estimate noise floor using the sorted-median method (median of sorted
//!    bin powers).
//! 3. Mark bins whose power falls below `noise_floor + threshold_db` as empty.
//! 4. Merge adjacent empty bins into contiguous spectrum holes.
//! 5. Score each hole by `bandwidth * snr_margin` for quality ranking.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::spectrum_hole_detector::{SpectrumHoleDetector, DetectorConfig};
//!
//! let config = DetectorConfig {
//!     fft_size: 64,
//!     threshold_db: 6.0,
//!     min_hole_bandwidth_hz: 1000.0,
//!     sample_rate: 1_000_000.0,
//! };
//!
//! let mut detector = SpectrumHoleDetector::new(config);
//!
//! // Generate noise-only samples (simulating empty spectrum)
//! let samples: Vec<(f64, f64)> = (0..256)
//!     .map(|i| {
//!         let phase = i as f64 * 0.01;
//!         (phase.cos() * 0.001, phase.sin() * 0.001)
//!     })
//!     .collect();
//!
//! detector.push_samples(&samples);
//!
//! let holes = detector.detect_holes();
//! // With low-level noise and no strong signals, most of the band is a hole
//! assert!(!holes.is_empty(), "should detect at least one spectrum hole");
//!
//! let ranked = detector.rank_holes();
//! // Ranked holes are sorted by quality (best first)
//! if ranked.len() >= 2 {
//!     assert!(ranked[0].quality_score >= ranked[1].quality_score);
//! }
//!
//! let utilization = detector.spectrum_utilization();
//! assert!(utilization >= 0.0 && utilization <= 1.0);
//! ```

use std::f64::consts::PI;

/// Configuration for the spectrum hole detector.
#[derive(Debug, Clone)]
pub struct DetectorConfig {
    /// Number of FFT bins (must be a power of 2).
    pub fft_size: usize,
    /// Threshold above noise floor in dB to consider a bin occupied.
    pub threshold_db: f64,
    /// Minimum bandwidth in Hz for a hole to be reported.
    pub min_hole_bandwidth_hz: f64,
    /// Sample rate in Hz.
    pub sample_rate: f64,
}

/// A detected spectrum hole (unused frequency band).
#[derive(Debug, Clone, PartialEq)]
pub struct SpectrumHole {
    /// Center frequency of the hole in Hz (relative to DC / baseband center).
    pub center_freq_hz: f64,
    /// Bandwidth of the hole in Hz.
    pub bandwidth_hz: f64,
    /// SNR margin in dB below the detection threshold.
    pub snr_margin_db: f64,
    /// Quality score: higher is better (bandwidth_hz * snr_margin_db).
    pub quality_score: f64,
}

/// FFT-based spectrum hole detector for cognitive radio spectrum sensing.
///
/// Accumulates IQ samples, computes power spectral density, estimates the
/// noise floor, and identifies contiguous frequency regions that are unused.
pub struct SpectrumHoleDetector {
    config: DetectorConfig,
    /// Accumulated power per bin (linear scale), averaged over segments.
    accumulated_power: Vec<f64>,
    /// Number of FFT segments averaged.
    segment_count: usize,
    /// Leftover samples from previous push that didn't fill a full FFT.
    sample_buffer: Vec<(f64, f64)>,
}

impl SpectrumHoleDetector {
    /// Create a new detector with the given configuration.
    ///
    /// # Panics
    ///
    /// Panics if `fft_size` is zero or not a power of 2, or if `sample_rate`
    /// is not positive.
    pub fn new(config: DetectorConfig) -> Self {
        assert!(config.fft_size > 0, "fft_size must be positive");
        assert!(
            config.fft_size.is_power_of_two(),
            "fft_size must be a power of 2"
        );
        assert!(config.sample_rate > 0.0, "sample_rate must be positive");

        Self {
            accumulated_power: vec![0.0; config.fft_size],
            segment_count: 0,
            sample_buffer: Vec::new(),
            config,
        }
    }

    /// Push IQ samples into the detector. Samples are segmented into FFT-sized
    /// blocks and their power spectra are accumulated (averaged). Leftover
    /// samples that don't fill a complete FFT block are buffered for the next
    /// call.
    pub fn push_samples(&mut self, samples: &[(f64, f64)]) {
        self.sample_buffer.extend_from_slice(samples);

        let n = self.config.fft_size;

        while self.sample_buffer.len() >= n {
            let segment: Vec<(f64, f64)> = self.sample_buffer.drain(..n).collect();
            self.process_segment(&segment);
        }
    }

    /// Process a single FFT-sized segment: apply Hann window, compute FFT,
    /// accumulate power.
    fn process_segment(&mut self, segment: &[(f64, f64)]) {
        let n = self.config.fft_size;
        debug_assert_eq!(segment.len(), n);

        // Apply Hann window
        let windowed: Vec<(f64, f64)> = segment
            .iter()
            .enumerate()
            .map(|(i, &(re, im))| {
                let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos());
                (re * w, im * w)
            })
            .collect();

        // Compute FFT
        let spectrum = Self::fft(&windowed);

        // Accumulate power (magnitude squared), FFT-shifted so DC is in the center
        for i in 0..n {
            // FFT shift: move bin i to shifted position
            let shifted = (i + n / 2) % n;
            let (re, im) = spectrum[i];
            let power = (re * re + im * im) / (n as f64 * n as f64);
            self.accumulated_power[shifted] += power;
        }
        self.segment_count += 1;
    }

    /// Detect spectrum holes: contiguous frequency regions below the threshold.
    ///
    /// Returns an empty vector if no samples have been processed yet.
    pub fn detect_holes(&self) -> Vec<SpectrumHole> {
        if self.segment_count == 0 {
            return Vec::new();
        }

        let n = self.config.fft_size;
        let avg_power = self.average_power();
        let noise_floor_linear = self.noise_floor_linear(&avg_power);
        let threshold_linear =
            noise_floor_linear * Self::db_to_linear(self.config.threshold_db);

        let bin_bw = self.config.sample_rate / n as f64;

        // Identify empty bins (below threshold)
        let empty: Vec<bool> = avg_power.iter().map(|&p| p < threshold_linear).collect();

        // Merge contiguous empty bins into holes
        let mut holes = Vec::new();
        let mut start: Option<usize> = None;

        for i in 0..n {
            if empty[i] {
                if start.is_none() {
                    start = Some(i);
                }
            } else if let Some(s) = start {
                self.try_add_hole(&mut holes, s, i, &avg_power, noise_floor_linear, bin_bw);
                start = None;
            }
        }
        // Handle hole that extends to the last bin
        if let Some(s) = start {
            self.try_add_hole(&mut holes, s, n, &avg_power, noise_floor_linear, bin_bw);
        }

        holes
    }

    /// Try to create a hole from bins [start, end) and add it if it meets
    /// the minimum bandwidth requirement.
    fn try_add_hole(
        &self,
        holes: &mut Vec<SpectrumHole>,
        start: usize,
        end: usize,
        avg_power: &[f64],
        noise_floor_linear: f64,
        bin_bw: f64,
    ) {
        let n = self.config.fft_size;
        let num_bins = end - start;
        let bandwidth = num_bins as f64 * bin_bw;

        if bandwidth < self.config.min_hole_bandwidth_hz {
            return;
        }

        // Center frequency: bins are FFT-shifted, so bin 0 = -fs/2, bin N-1 = +fs/2 - bin_bw
        let center_bin = start as f64 + num_bins as f64 / 2.0;
        let center_freq = (center_bin - n as f64 / 2.0) * bin_bw;

        // SNR margin: how far below the threshold the average hole power is
        let avg_hole_power: f64 =
            avg_power[start..end].iter().sum::<f64>() / num_bins as f64;
        let snr_margin = if avg_hole_power > 0.0 {
            Self::linear_to_db(noise_floor_linear / avg_hole_power)
                + self.config.threshold_db
        } else {
            self.config.threshold_db + 60.0 // very clean hole
        };
        let snr_margin = snr_margin.max(0.0);

        let quality_score = bandwidth * snr_margin;

        holes.push(SpectrumHole {
            center_freq_hz: center_freq,
            bandwidth_hz: bandwidth,
            snr_margin_db: snr_margin,
            quality_score,
        });
    }

    /// Return detected holes sorted by quality score (best first).
    pub fn rank_holes(&self) -> Vec<SpectrumHole> {
        let mut holes = self.detect_holes();
        holes.sort_by(|a, b| {
            b.quality_score
                .partial_cmp(&a.quality_score)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        holes
    }

    /// Estimated noise floor in dB (10*log10 of linear noise floor).
    ///
    /// Uses the sorted-median method: sort all bin powers and take the median
    /// as the noise floor estimate, which is robust to narrowband signals.
    ///
    /// Returns `-200.0` if no samples have been processed.
    pub fn noise_floor_db(&self) -> f64 {
        if self.segment_count == 0 {
            return -200.0;
        }
        let avg_power = self.average_power();
        let nf = self.noise_floor_linear(&avg_power);
        Self::linear_to_db(nf)
    }

    /// Fraction of the band that is occupied (above threshold).
    ///
    /// Returns `0.0` if no samples have been processed.
    pub fn spectrum_utilization(&self) -> f64 {
        if self.segment_count == 0 {
            return 0.0;
        }

        let n = self.config.fft_size;
        let avg_power = self.average_power();
        let noise_floor_linear = self.noise_floor_linear(&avg_power);
        let threshold_linear =
            noise_floor_linear * Self::db_to_linear(self.config.threshold_db);

        let occupied_bins = avg_power.iter().filter(|&&p| p >= threshold_linear).count();
        occupied_bins as f64 / n as f64
    }

    /// Power spectral density as (frequency_hz, power_db) pairs.
    ///
    /// Frequencies are relative to baseband center (DC = 0 Hz), spanning
    /// from `-sample_rate/2` to `+sample_rate/2`.
    ///
    /// Returns an empty vector if no samples have been processed.
    pub fn power_spectral_density(&self) -> Vec<(f64, f64)> {
        if self.segment_count == 0 {
            return Vec::new();
        }

        let n = self.config.fft_size;
        let avg_power = self.average_power();
        let bin_bw = self.config.sample_rate / n as f64;

        (0..n)
            .map(|i| {
                let freq = (i as f64 - n as f64 / 2.0) * bin_bw;
                let power_db = Self::linear_to_db(avg_power[i]);
                (freq, power_db)
            })
            .collect()
    }

    // ---- Internal helpers ----

    /// Return average power per bin (linear).
    fn average_power(&self) -> Vec<f64> {
        let count = self.segment_count as f64;
        self.accumulated_power.iter().map(|&p| p / count).collect()
    }

    /// Noise floor estimate in linear power using the sorted-median method.
    fn noise_floor_linear(&self, avg_power: &[f64]) -> f64 {
        let mut sorted: Vec<f64> = avg_power.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mid = sorted.len() / 2;
        if sorted.len() % 2 == 0 && sorted.len() >= 2 {
            (sorted[mid - 1] + sorted[mid]) / 2.0
        } else {
            sorted[mid]
        }
    }

    /// Convert dB to linear power ratio.
    fn db_to_linear(db: f64) -> f64 {
        10.0_f64.powf(db / 10.0)
    }

    /// Convert linear power to dB. Returns -200 for zero or negative input.
    fn linear_to_db(linear: f64) -> f64 {
        if linear <= 0.0 {
            -200.0
        } else {
            10.0 * linear.log10()
        }
    }

    /// Radix-2 decimation-in-time FFT (Cooley-Tukey).
    ///
    /// Input length must be a power of 2.
    fn fft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n = input.len();
        if n <= 1 {
            return input.to_vec();
        }

        // Bit-reversal permutation
        let mut output = input.to_vec();
        let mut j = 0usize;
        for i in 0..n {
            if i < j {
                output.swap(i, j);
            }
            let mut m = n >> 1;
            while m >= 1 && j >= m {
                j -= m;
                m >>= 1;
            }
            j += m;
        }

        // Butterfly stages
        let mut len = 2;
        while len <= n {
            let half = len / 2;
            let angle_step = -2.0 * PI / len as f64;
            for start in (0..n).step_by(len) {
                for k in 0..half {
                    let angle = angle_step * k as f64;
                    let twiddle_re = angle.cos();
                    let twiddle_im = angle.sin();

                    let u = output[start + k];
                    let v = output[start + k + half];

                    // Complex multiply: twiddle * v
                    let t_re = twiddle_re * v.0 - twiddle_im * v.1;
                    let t_im = twiddle_re * v.1 + twiddle_im * v.0;

                    output[start + k] = (u.0 + t_re, u.1 + t_im);
                    output[start + k + half] = (u.0 - t_re, u.1 - t_im);
                }
            }
            len <<= 1;
        }

        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn default_config() -> DetectorConfig {
        DetectorConfig {
            fft_size: 64,
            threshold_db: 6.0,
            min_hole_bandwidth_hz: 1000.0,
            sample_rate: 1_000_000.0,
        }
    }

    /// Generate a tone at a given frequency (Hz) relative to baseband center.
    fn generate_tone(freq: f64, sample_rate: f64, num_samples: usize, amplitude: f64) -> Vec<(f64, f64)> {
        (0..num_samples)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * PI * freq * t;
                (amplitude * phase.cos(), amplitude * phase.sin())
            })
            .collect()
    }

    /// Generate white-ish noise using a simple LCG PRNG (deterministic).
    fn generate_noise(num_samples: usize, amplitude: f64) -> Vec<(f64, f64)> {
        let mut state: u64 = 0xDEAD_BEEF_CAFE_1234;
        (0..num_samples)
            .map(|_| {
                // Simple LCG
                state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
                let r1 = ((state >> 33) as f64 / (1u64 << 31) as f64) - 1.0;
                state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
                let r2 = ((state >> 33) as f64 / (1u64 << 31) as f64) - 1.0;
                (r1 * amplitude, r2 * amplitude)
            })
            .collect()
    }

    #[test]
    fn test_new_creates_detector() {
        let config = default_config();
        let det = SpectrumHoleDetector::new(config.clone());
        assert_eq!(det.accumulated_power.len(), 64);
        assert_eq!(det.segment_count, 0);
    }

    #[test]
    #[should_panic(expected = "fft_size must be a power of 2")]
    fn test_new_rejects_non_power_of_two() {
        let config = DetectorConfig {
            fft_size: 100,
            ..default_config()
        };
        SpectrumHoleDetector::new(config);
    }

    #[test]
    #[should_panic(expected = "fft_size must be positive")]
    fn test_new_rejects_zero_fft_size() {
        let config = DetectorConfig {
            fft_size: 0,
            ..default_config()
        };
        SpectrumHoleDetector::new(config);
    }

    #[test]
    fn test_no_samples_returns_empty() {
        let det = SpectrumHoleDetector::new(default_config());
        assert!(det.detect_holes().is_empty());
        assert!(det.rank_holes().is_empty());
        assert_eq!(det.noise_floor_db(), -200.0);
        assert_eq!(det.spectrum_utilization(), 0.0);
        assert!(det.power_spectral_density().is_empty());
    }

    #[test]
    fn test_noise_only_most_band_is_hole() {
        let config = default_config();
        let mut det = SpectrumHoleDetector::new(config.clone());

        let noise = generate_noise(config.fft_size * 8, 0.001);
        det.push_samples(&noise);

        let holes = det.detect_holes();
        // With pure noise and threshold_db > 0, most bins should be below threshold
        // so we expect at least one large hole
        assert!(!holes.is_empty(), "should find holes in noise-only signal");

        let total_hole_bw: f64 = holes.iter().map(|h| h.bandwidth_hz).sum();
        // At least 30% of the band should be holes
        assert!(total_hole_bw > config.sample_rate * 0.3);
    }

    #[test]
    fn test_strong_tone_creates_occupied_region() {
        let config = DetectorConfig {
            fft_size: 256,
            threshold_db: 10.0,
            min_hole_bandwidth_hz: 1000.0,
            sample_rate: 1_000_000.0,
        };
        let mut det = SpectrumHoleDetector::new(config.clone());

        // Strong tone at +100 kHz plus low noise
        let num = config.fft_size * 16;
        let tone = generate_tone(100_000.0, config.sample_rate, num, 1.0);
        let noise = generate_noise(num, 0.001);
        let combined: Vec<(f64, f64)> = tone
            .iter()
            .zip(noise.iter())
            .map(|(&(tr, ti), &(nr, ni))| (tr + nr, ti + ni))
            .collect();

        det.push_samples(&combined);

        let utilization = det.spectrum_utilization();
        // Only a small fraction of the band should be occupied by the tone
        assert!(utilization > 0.0, "tone should occupy some spectrum");
        assert!(utilization < 0.5, "tone should not occupy most of the spectrum");
    }

    #[test]
    fn test_rank_holes_sorted_by_quality() {
        let config = default_config();
        let mut det = SpectrumHoleDetector::new(config.clone());

        let noise = generate_noise(config.fft_size * 4, 0.001);
        det.push_samples(&noise);

        let ranked = det.rank_holes();
        for i in 1..ranked.len() {
            assert!(
                ranked[i - 1].quality_score >= ranked[i].quality_score,
                "holes should be sorted by quality descending"
            );
        }
    }

    #[test]
    fn test_noise_floor_db_reasonable() {
        let config = default_config();
        let mut det = SpectrumHoleDetector::new(config.clone());

        let noise = generate_noise(config.fft_size * 4, 0.01);
        det.push_samples(&noise);

        let nf = det.noise_floor_db();
        // Noise floor should be a finite negative number for low-level noise
        assert!(nf.is_finite());
        assert!(nf < 0.0, "noise floor of low-level noise should be negative dB");
    }

    #[test]
    fn test_spectrum_utilization_range() {
        let config = default_config();
        let mut det = SpectrumHoleDetector::new(config.clone());

        let noise = generate_noise(config.fft_size * 4, 0.01);
        det.push_samples(&noise);

        let util = det.spectrum_utilization();
        assert!(util >= 0.0 && util <= 1.0, "utilization must be in [0, 1]");
    }

    #[test]
    fn test_power_spectral_density_length_and_freq_range() {
        let config = default_config();
        let mut det = SpectrumHoleDetector::new(config.clone());

        let noise = generate_noise(config.fft_size * 2, 0.01);
        det.push_samples(&noise);

        let psd = det.power_spectral_density();
        assert_eq!(psd.len(), config.fft_size);

        // Check that frequencies span -fs/2 to +fs/2
        let (first_freq, _) = psd[0];
        let (last_freq, _) = psd[psd.len() - 1];
        let half_fs = config.sample_rate / 2.0;
        assert!(
            first_freq < -half_fs + config.sample_rate / config.fft_size as f64 + 1.0,
            "first bin should be near -fs/2"
        );
        assert!(
            last_freq > half_fs - config.sample_rate / config.fft_size as f64 - 1.0,
            "last bin should be near +fs/2"
        );
    }

    #[test]
    fn test_fft_dc_signal() {
        // A constant (DC) signal should produce energy only in bin 0
        let n = 16;
        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        let result = SpectrumHoleDetector::fft(&input);

        // Bin 0 should have magnitude N
        let mag0 = (result[0].0 * result[0].0 + result[0].1 * result[0].1).sqrt();
        assert!((mag0 - n as f64).abs() < 1e-10);

        // Other bins should be ~0
        for i in 1..n {
            let mag = (result[i].0 * result[i].0 + result[i].1 * result[i].1).sqrt();
            assert!(mag < 1e-10, "bin {} should be zero for DC signal", i);
        }
    }

    #[test]
    fn test_fft_single_tone() {
        let n = 64;
        // Tone at bin k=4
        let k = 4;
        let input: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * k as f64 * i as f64 / n as f64;
                (phase.cos(), phase.sin())
            })
            .collect();
        let result = SpectrumHoleDetector::fft(&input);

        // Bin k should have magnitude ~N
        let mag_k = (result[k].0 * result[k].0 + result[k].1 * result[k].1).sqrt();
        assert!(
            (mag_k - n as f64).abs() < 1e-8,
            "bin {} magnitude should be {}, got {}",
            k,
            n,
            mag_k
        );

        // Other bins should be ~0
        for i in 0..n {
            if i == k {
                continue;
            }
            let mag = (result[i].0 * result[i].0 + result[i].1 * result[i].1).sqrt();
            assert!(mag < 1e-8, "bin {} should be ~0, got {}", i, mag);
        }
    }

    #[test]
    fn test_push_samples_buffering() {
        let config = DetectorConfig {
            fft_size: 32,
            ..default_config()
        };
        let mut det = SpectrumHoleDetector::new(config.clone());

        // Push less than one FFT block
        let partial = generate_noise(20, 0.01);
        det.push_samples(&partial);
        assert_eq!(det.segment_count, 0, "not enough samples for a segment");
        assert_eq!(det.sample_buffer.len(), 20);

        // Push enough to complete one block and start another
        let more = generate_noise(30, 0.01);
        det.push_samples(&more);
        assert_eq!(det.segment_count, 1, "should have processed one segment");
        assert_eq!(det.sample_buffer.len(), 18, "leftover = 50 - 32 = 18");
    }

    #[test]
    fn test_hole_bandwidth_filter() {
        let config = DetectorConfig {
            fft_size: 64,
            threshold_db: 6.0,
            min_hole_bandwidth_hz: 500_000.0, // Very large minimum
            sample_rate: 1_000_000.0,
        };
        let mut det = SpectrumHoleDetector::new(config.clone());

        // Add a signal that occupies roughly half the band, so no single hole is 500kHz
        let num = config.fft_size * 8;
        let mut samples = generate_noise(num, 0.001);
        // Add many tones spread across the upper half to break up holes
        for freq_offset in [100_000.0, 200_000.0, 300_000.0, 400_000.0] {
            let tone = generate_tone(freq_offset, config.sample_rate, num, 1.0);
            for (i, &(tr, ti)) in tone.iter().enumerate() {
                samples[i].0 += tr;
                samples[i].1 += ti;
            }
        }
        det.push_samples(&samples);

        let holes = det.detect_holes();
        // All returned holes must meet the minimum bandwidth
        for hole in &holes {
            assert!(
                hole.bandwidth_hz >= config.min_hole_bandwidth_hz,
                "hole bandwidth {} < min {}",
                hole.bandwidth_hz,
                config.min_hole_bandwidth_hz
            );
        }
    }

    #[test]
    fn test_spectrum_hole_fields_positive() {
        let config = default_config();
        let mut det = SpectrumHoleDetector::new(config.clone());

        let noise = generate_noise(config.fft_size * 4, 0.001);
        det.push_samples(&noise);

        let holes = det.detect_holes();
        for hole in &holes {
            assert!(hole.bandwidth_hz > 0.0, "bandwidth must be positive");
            assert!(hole.snr_margin_db >= 0.0, "snr_margin must be non-negative");
            assert!(hole.quality_score >= 0.0, "quality_score must be non-negative");
        }
    }

    #[test]
    fn test_db_conversions_roundtrip() {
        let db_values = [-30.0, -10.0, 0.0, 3.0, 10.0, 20.0];
        for &db in &db_values {
            let linear = SpectrumHoleDetector::db_to_linear(db);
            let back = SpectrumHoleDetector::linear_to_db(linear);
            assert!(
                (back - db).abs() < 1e-10,
                "dB roundtrip failed for {}: got {}",
                db,
                back
            );
        }
    }

    #[test]
    fn test_multiple_push_accumulates() {
        let config = default_config();
        let mut det = SpectrumHoleDetector::new(config.clone());

        // Push in small batches
        for _ in 0..8 {
            let chunk = generate_noise(config.fft_size, 0.01);
            det.push_samples(&chunk);
        }

        assert_eq!(det.segment_count, 8);
        // Should be able to detect holes after accumulation
        let holes = det.detect_holes();
        assert!(!holes.is_empty());
    }

    #[test]
    fn test_full_band_occupied() {
        // With a strong tone on top of noise, the tone bins should be occupied
        // and utilization should be nonzero. We test that adding a strong signal
        // increases utilization compared to pure noise.
        let config = DetectorConfig {
            fft_size: 64,
            threshold_db: 6.0,
            min_hole_bandwidth_hz: 1000.0,
            sample_rate: 1_000_000.0,
        };

        // Baseline: noise only
        let mut det_noise = SpectrumHoleDetector::new(config.clone());
        let noise = generate_noise(config.fft_size * 8, 0.001);
        det_noise.push_samples(&noise);
        let util_noise = det_noise.spectrum_utilization();

        // With signal: add multiple strong tones spread across the band
        let mut det_sig = SpectrumHoleDetector::new(config.clone());
        let num = config.fft_size * 8;
        let mut samples = generate_noise(num, 0.001);
        // Add 8 tones spread across the band
        for k in 0..8 {
            let freq = -400_000.0 + k as f64 * 100_000.0;
            let tone = generate_tone(freq, config.sample_rate, num, 1.0);
            for (i, &(tr, ti)) in tone.iter().enumerate() {
                samples[i].0 += tr;
                samples[i].1 += ti;
            }
        }
        det_sig.push_samples(&samples);
        let util_sig = det_sig.spectrum_utilization();

        // Adding strong tones should increase utilization
        assert!(
            util_sig > util_noise,
            "adding tones should increase utilization: {} vs {}",
            util_sig,
            util_noise
        );
        // And utilization should be positive
        assert!(util_sig > 0.0, "utilization with tones should be > 0");
    }
}
