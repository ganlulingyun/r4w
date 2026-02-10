//! # Spurs Mitigation
//!
//! Detect and suppress spurious emissions, clock harmonics, and LO leakage in RF chains.
//!
//! This module provides [`SpursMitigator`] for automatic detection and cancellation of
//! narrowband spurs using spectral analysis and IIR notch filtering.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::spurs_mitigation::{SpursMitigator, MitigationConfig};
//!
//! let config = MitigationConfig {
//!     sample_rate: 1_000_000.0,
//!     fft_size: 1024,
//!     detection_threshold_db: 20.0,
//!     max_notch_width_hz: 5000.0,
//! };
//! let mitigator = SpursMitigator::new(config);
//!
//! // Generate a signal with a known spur (DC offset = LO leakage)
//! let n = 1024;
//! let samples: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / 1_000_000.0;
//!         // Desired signal at 100 kHz plus DC offset (LO leakage)
//!         let sig_i = (2.0 * std::f64::consts::PI * 100_000.0 * t).cos();
//!         let sig_q = (2.0 * std::f64::consts::PI * 100_000.0 * t).sin();
//!         (sig_i + 0.5, sig_q + 0.3)
//!     })
//!     .collect();
//!
//! // Remove DC offset (LO leakage)
//! let cleaned = mitigator.remove_dc_offset(&samples);
//!
//! // Verify DC is suppressed: mean should be near zero
//! let mean_i: f64 = cleaned.iter().map(|s| s.0).sum::<f64>() / cleaned.len() as f64;
//! let mean_q: f64 = cleaned.iter().map(|s| s.1).sum::<f64>() / cleaned.len() as f64;
//! assert!(mean_i.abs() < 0.01, "DC offset on I should be removed");
//! assert!(mean_q.abs() < 0.01, "DC offset on Q should be removed");
//!
//! // Auto-mitigate detects and suppresses spurs automatically
//! let mitigated = mitigator.auto_mitigate(&samples);
//! assert_eq!(mitigated.len(), samples.len());
//! ```

use std::f64::consts::PI;

/// Classification of a detected spur by its likely origin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpurClassification {
    /// Harmonic of a reference clock (integer multiple of clock frequency).
    ClockHarmonic,
    /// Local oscillator leakage (DC or near-DC component).
    LoLeakage,
    /// DAC image frequency (mirror around Nyquist).
    DacImage,
    /// PLL reference spur (offset from carrier by PLL reference frequency).
    PllSpur,
    /// Unknown or unclassified spur.
    Unknown,
}

/// A detected spurious emission with its characteristics.
#[derive(Debug, Clone)]
pub struct Spur {
    /// Frequency of the spur in Hz (can be negative for below-DC spurs).
    pub freq_hz: f64,
    /// Power level of the spur in dBm.
    pub power_dbm: f64,
    /// Estimated bandwidth of the spur in Hz.
    pub bandwidth_hz: f64,
    /// Classification of the spur source.
    pub classification: SpurClassification,
}

/// Configuration for the spurs mitigation engine.
#[derive(Debug, Clone)]
pub struct MitigationConfig {
    /// Sample rate of the input signal in Hz.
    pub sample_rate: f64,
    /// FFT size for spectral analysis (must be a power of 2).
    pub fft_size: usize,
    /// Detection threshold in dB above the noise floor.
    pub detection_threshold_db: f64,
    /// Maximum notch filter width in Hz.
    pub max_notch_width_hz: f64,
}

impl Default for MitigationConfig {
    fn default() -> Self {
        Self {
            sample_rate: 1_000_000.0,
            fft_size: 1024,
            detection_threshold_db: 20.0,
            max_notch_width_hz: 5000.0,
        }
    }
}

/// Main spur detection and cancellation engine.
///
/// Performs spectral analysis to find narrowband spurs, classifies them,
/// and applies IIR notch filters for cancellation.
#[derive(Debug, Clone)]
pub struct SpursMitigator {
    config: MitigationConfig,
}

impl SpursMitigator {
    /// Create a new spurs mitigator with the given configuration.
    pub fn new(config: MitigationConfig) -> Self {
        Self { config }
    }

    /// Detect spurious emissions in the given IQ samples.
    ///
    /// Uses FFT-based spectral analysis with peak detection above the noise floor.
    /// Peaks exceeding `detection_threshold_db` above the median power are reported.
    pub fn detect_spurs(&self, samples: &[(f64, f64)]) -> Vec<Spur> {
        let n = self.config.fft_size.min(samples.len());
        if n == 0 {
            return Vec::new();
        }

        // Compute power spectrum via DFT
        let spectrum = self.compute_power_spectrum(samples, n);

        // Estimate noise floor as the median power
        let noise_floor = self.estimate_noise_floor(&spectrum);
        let threshold = noise_floor + self.config.detection_threshold_db;

        // Find peaks above threshold
        let mut spurs = Vec::new();
        let freq_resolution = self.config.sample_rate / n as f64;

        for bin in 0..n {
            let power_db = spectrum[bin];
            if power_db > threshold && self.is_local_peak(&spectrum, bin) {
                // Convert bin to frequency (centered around DC)
                let freq_hz = if bin <= n / 2 {
                    bin as f64 * freq_resolution
                } else {
                    (bin as f64 - n as f64) * freq_resolution
                };

                // Estimate bandwidth from 3dB drop
                let bandwidth_hz = self.estimate_spur_bandwidth(&spectrum, bin, freq_resolution);

                // Classify the spur
                let classification = self.classify_spur(freq_hz, freq_resolution);

                spurs.push(Spur {
                    freq_hz,
                    power_dbm: power_db,
                    bandwidth_hz,
                    classification,
                });
            }
        }

        // Sort by power (strongest first)
        spurs.sort_by(|a, b| b.power_dbm.partial_cmp(&a.power_dbm).unwrap_or(std::cmp::Ordering::Equal));
        spurs
    }

    /// Suppress the specified spurs using IIR notch filters.
    ///
    /// Applies a cascade of second-order IIR notch filters, one per spur.
    pub fn suppress(&self, samples: &[(f64, f64)], spurs: &[Spur]) -> Vec<(f64, f64)> {
        if spurs.is_empty() || samples.is_empty() {
            return samples.to_vec();
        }

        let mut result = samples.to_vec();

        for spur in spurs {
            let notch_bw = spur.bandwidth_hz.min(self.config.max_notch_width_hz);
            result = self.apply_notch_filter(&result, spur.freq_hz, notch_bw);
        }

        result
    }

    /// Automatically detect and suppress all spurs in the signal.
    ///
    /// Convenience method that calls [`detect_spurs`](Self::detect_spurs) followed
    /// by [`suppress`](Self::suppress).
    pub fn auto_mitigate(&self, samples: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let spurs = self.detect_spurs(samples);
        self.suppress(samples, &spurs)
    }

    /// Remove DC offset (LO leakage) from the signal.
    ///
    /// Subtracts the mean I and Q values, then applies a single-pole highpass
    /// filter with a cutoff near DC for residual tracking.
    pub fn remove_dc_offset(&self, samples: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if samples.is_empty() {
            return Vec::new();
        }

        let n = samples.len() as f64;
        let mean_i: f64 = samples.iter().map(|s| s.0).sum::<f64>() / n;
        let mean_q: f64 = samples.iter().map(|s| s.1).sum::<f64>() / n;

        // Mean subtraction
        let mut result: Vec<(f64, f64)> = samples
            .iter()
            .map(|s| (s.0 - mean_i, s.1 - mean_q))
            .collect();

        // Single-pole highpass IIR for tracking residual DC drift
        // H(z) = (1 - z^-1) / (1 - alpha * z^-1)
        // alpha close to 1.0 gives a very narrow notch at DC
        let alpha = 1.0 - (10.0 / self.config.sample_rate); // ~10 Hz cutoff
        let alpha = alpha.max(0.9).min(0.9999);

        let mut prev_in_i = 0.0_f64;
        let mut prev_in_q = 0.0_f64;
        let mut prev_out_i = 0.0_f64;
        let mut prev_out_q = 0.0_f64;

        for sample in result.iter_mut() {
            let out_i = sample.0 - prev_in_i + alpha * prev_out_i;
            let out_q = sample.1 - prev_in_q + alpha * prev_out_q;
            prev_in_i = sample.0;
            prev_in_q = sample.1;
            prev_out_i = out_i;
            prev_out_q = out_q;
            *sample = (out_i, out_q);
        }

        result
    }

    /// Apply a cascade of notch filters at the specified frequencies.
    ///
    /// Each notch has the default bandwidth from the configuration.
    pub fn notch_cascade(&self, samples: &[(f64, f64)], freqs: &[f64]) -> Vec<(f64, f64)> {
        if freqs.is_empty() || samples.is_empty() {
            return samples.to_vec();
        }

        let mut result = samples.to_vec();
        let default_bw = self.config.max_notch_width_hz;

        for &freq in freqs {
            result = self.apply_notch_filter(&result, freq, default_bw);
        }

        result
    }

    // --- Internal methods ---

    /// Compute the power spectrum in dB using a simple DFT.
    fn compute_power_spectrum(&self, samples: &[(f64, f64)], n: usize) -> Vec<f64> {
        let mut spectrum = vec![0.0_f64; n];

        // Apply Hann window and compute DFT
        for k in 0..n {
            let mut sum_re = 0.0_f64;
            let mut sum_im = 0.0_f64;

            for (idx, sample) in samples.iter().take(n).enumerate() {
                // Hann window
                let window = 0.5 * (1.0 - (2.0 * PI * idx as f64 / n as f64).cos());
                let angle = -2.0 * PI * k as f64 * idx as f64 / n as f64;
                let cos_a = angle.cos();
                let sin_a = angle.sin();

                // Complex multiply: (sample.0 + j*sample.1) * (cos_a + j*sin_a)
                let windowed_i = sample.0 * window;
                let windowed_q = sample.1 * window;
                sum_re += windowed_i * cos_a - windowed_q * sin_a;
                sum_im += windowed_i * sin_a + windowed_q * cos_a;
            }

            let power = sum_re * sum_re + sum_im * sum_im;
            // Convert to dB (use a small floor to avoid -inf)
            spectrum[k] = 10.0 * (power / (n as f64 * n as f64) + 1e-20).log10();
        }

        spectrum
    }

    /// Estimate the noise floor as the median of the power spectrum.
    fn estimate_noise_floor(&self, spectrum: &[f64]) -> f64 {
        let mut sorted: Vec<f64> = spectrum.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mid = sorted.len() / 2;
        if sorted.is_empty() {
            return -200.0;
        }
        sorted[mid]
    }

    /// Check if a bin is a local spectral peak (higher than its neighbors).
    fn is_local_peak(&self, spectrum: &[f64], bin: usize) -> bool {
        let n = spectrum.len();
        if n < 3 {
            return true;
        }
        let val = spectrum[bin];
        let left = spectrum[(bin + n - 1) % n];
        let right = spectrum[(bin + 1) % n];
        val >= left && val >= right
    }

    /// Estimate the 3dB bandwidth of a spectral peak.
    fn estimate_spur_bandwidth(&self, spectrum: &[f64], peak_bin: usize, freq_res: f64) -> f64 {
        let n = spectrum.len();
        let peak_power = spectrum[peak_bin];
        let threshold = peak_power - 3.0; // 3 dB below peak

        // Count bins above threshold on each side
        let mut width = 1usize;
        for offset in 1..n / 2 {
            let left = (peak_bin + n - offset) % n;
            let right = (peak_bin + offset) % n;
            if spectrum[left] > threshold || spectrum[right] > threshold {
                width += 1;
            } else {
                break;
            }
        }

        width as f64 * freq_res
    }

    /// Classify a spur based on its frequency.
    fn classify_spur(&self, freq_hz: f64, freq_resolution: f64) -> SpurClassification {
        let abs_freq = freq_hz.abs();

        // DC or near-DC: LO leakage
        if abs_freq < freq_resolution * 1.5 {
            return SpurClassification::LoLeakage;
        }

        // Near Nyquist/2: DAC image
        let nyquist = self.config.sample_rate / 2.0;
        if (abs_freq - nyquist).abs() < freq_resolution * 3.0 {
            return SpurClassification::DacImage;
        }

        // Check if it's a harmonic of a common clock frequency
        // Common clocks: 10 MHz, 26 MHz, 38.4 MHz, etc.
        let common_clocks = [10e6, 26e6, 38.4e6, 19.2e6, 32.768e3, 48e3];
        for &clock in &common_clocks {
            if clock > self.config.sample_rate {
                continue;
            }
            // Check if freq is an integer multiple of clock
            let ratio = abs_freq / clock;
            if ratio > 0.5 && (ratio - ratio.round()).abs() < 0.05 {
                return SpurClassification::ClockHarmonic;
            }
        }

        SpurClassification::Unknown
    }

    /// Apply a second-order IIR notch filter at the specified frequency.
    ///
    /// Uses a constrained poles/zeros design:
    ///   - Zeros on the unit circle at the notch frequency
    ///   - Poles at the same angle but radius r < 1 (controls bandwidth)
    fn apply_notch_filter(&self, samples: &[(f64, f64)], freq_hz: f64, bandwidth_hz: f64) -> Vec<(f64, f64)> {
        let fs = self.config.sample_rate;

        // Normalized notch frequency
        let w0 = 2.0 * PI * freq_hz / fs;

        // Pole radius: controls the notch bandwidth
        // r = 1 - pi * BW / fs  (approximate for narrow notches)
        let r = (1.0 - PI * bandwidth_hz / fs).max(0.5).min(0.9999);

        // Transfer function:
        // H(z) = (1 - 2*cos(w0)*z^-1 + z^-2) / (1 - 2*r*cos(w0)*z^-1 + r^2*z^-2)
        let cos_w0 = w0.cos();
        let b0 = 1.0;
        let b1 = -2.0 * cos_w0;
        let b2 = 1.0;
        let a1 = -2.0 * r * cos_w0;
        let a2 = r * r;

        // Normalize so passband gain = 1 at DC (for non-DC notches)
        // Gain at DC: H(1) = (1 + b1 + b2) / (1 + a1 + a2)
        let gain_dc = (b0 + b1 + b2) / (1.0 + a1 + a2);
        let norm = if gain_dc.abs() > 1e-10 { 1.0 / gain_dc } else { 1.0 };
        let b0 = b0 * norm;
        let b1 = b1 * norm;
        let b2 = b2 * norm;

        // Apply the filter to both I and Q channels
        let mut output = Vec::with_capacity(samples.len());
        let mut x1_i = 0.0_f64;
        let mut x2_i = 0.0_f64;
        let mut y1_i = 0.0_f64;
        let mut y2_i = 0.0_f64;
        let mut x1_q = 0.0_f64;
        let mut x2_q = 0.0_f64;
        let mut y1_q = 0.0_f64;
        let mut y2_q = 0.0_f64;

        for &(xi, xq) in samples {
            let yi = b0 * xi + b1 * x1_i + b2 * x2_i - a1 * y1_i - a2 * y2_i;
            let yq = b0 * xq + b1 * x1_q + b2 * x2_q - a1 * y1_q - a2 * y2_q;

            x2_i = x1_i;
            x1_i = xi;
            y2_i = y1_i;
            y1_i = yi;

            x2_q = x1_q;
            x1_q = xq;
            y2_q = y1_q;
            y1_q = yq;

            output.push((yi, yq));
        }

        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> MitigationConfig {
        MitigationConfig {
            sample_rate: 1_000_000.0,
            fft_size: 1024,
            detection_threshold_db: 20.0,
            max_notch_width_hz: 5000.0,
        }
    }

    fn generate_tone(freq: f64, fs: f64, n: usize, amplitude: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let phase = 2.0 * PI * freq * t;
                (amplitude * phase.cos(), amplitude * phase.sin())
            })
            .collect()
    }

    fn signal_power(samples: &[(f64, f64)]) -> f64 {
        let n = samples.len() as f64;
        samples.iter().map(|s| s.0 * s.0 + s.1 * s.1).sum::<f64>() / n
    }

    #[test]
    fn test_new_mitigator() {
        let config = default_config();
        let m = SpursMitigator::new(config);
        assert_eq!(m.config.sample_rate, 1_000_000.0);
        assert_eq!(m.config.fft_size, 1024);
        assert_eq!(m.config.detection_threshold_db, 20.0);
        assert_eq!(m.config.max_notch_width_hz, 5000.0);
    }

    #[test]
    fn test_default_config() {
        let config = MitigationConfig::default();
        assert_eq!(config.sample_rate, 1_000_000.0);
        assert_eq!(config.fft_size, 1024);
        assert_eq!(config.detection_threshold_db, 20.0);
        assert_eq!(config.max_notch_width_hz, 5000.0);
    }

    #[test]
    fn test_detect_spurs_empty_input() {
        let m = SpursMitigator::new(default_config());
        let spurs = m.detect_spurs(&[]);
        assert!(spurs.is_empty());
    }

    #[test]
    fn test_detect_dc_spur() {
        let config = MitigationConfig {
            sample_rate: 100_000.0,
            fft_size: 256,
            detection_threshold_db: 10.0,
            max_notch_width_hz: 1000.0,
        };
        let m = SpursMitigator::new(config);

        // Strong DC offset in noise
        let n = 256;
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let noise = ((i * 7 + 3) % 100) as f64 / 10000.0 - 0.005;
                (5.0 + noise, 3.0 + noise)
            })
            .collect();

        let spurs = m.detect_spurs(&samples);
        // Should detect the DC component
        assert!(!spurs.is_empty(), "Should detect DC spur");

        // The strongest spur should be near DC
        let dc_spur = &spurs[0];
        assert!(
            dc_spur.freq_hz.abs() < 1000.0,
            "DC spur should be near 0 Hz, got {} Hz",
            dc_spur.freq_hz
        );
    }

    #[test]
    fn test_detect_tone_spur() {
        let config = MitigationConfig {
            sample_rate: 100_000.0,
            fft_size: 512,
            detection_threshold_db: 15.0,
            max_notch_width_hz: 2000.0,
        };
        let m = SpursMitigator::new(config);

        // Strong tone at 25 kHz in weak noise
        let n = 512;
        let fs = 100_000.0;
        let mut samples = generate_tone(25_000.0, fs, n, 10.0);
        for (i, s) in samples.iter_mut().enumerate() {
            let noise = ((i * 13 + 7) % 200) as f64 / 200000.0 - 0.0005;
            s.0 += noise;
            s.1 += noise;
        }

        let spurs = m.detect_spurs(&samples);
        assert!(!spurs.is_empty(), "Should detect the tone spur");

        // Check that at least one spur is near 25 kHz
        let near_25k = spurs.iter().any(|s| (s.freq_hz - 25_000.0).abs() < 500.0);
        assert!(near_25k, "Should find a spur near 25 kHz");
    }

    #[test]
    fn test_suppress_empty_spurs() {
        let m = SpursMitigator::new(default_config());
        let samples = generate_tone(10_000.0, 1_000_000.0, 256, 1.0);
        let result = m.suppress(&samples, &[]);
        assert_eq!(result.len(), samples.len());
        // Should be unchanged
        for (a, b) in result.iter().zip(samples.iter()) {
            assert!((a.0 - b.0).abs() < 1e-12);
            assert!((a.1 - b.1).abs() < 1e-12);
        }
    }

    #[test]
    fn test_suppress_reduces_spur_power() {
        let fs = 100_000.0;
        let n = 2048;
        let spur_freq = 20_000.0;

        // Signal: desired tone at 5 kHz + spur at 20 kHz
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let desired_i = (2.0 * PI * 5_000.0 * t).cos();
                let desired_q = (2.0 * PI * 5_000.0 * t).sin();
                let spur_i = 0.5 * (2.0 * PI * spur_freq * t).cos();
                let spur_q = 0.5 * (2.0 * PI * spur_freq * t).sin();
                (desired_i + spur_i, desired_q + spur_q)
            })
            .collect();

        let spur = Spur {
            freq_hz: spur_freq,
            power_dbm: -20.0,
            bandwidth_hz: 500.0,
            classification: SpurClassification::Unknown,
        };

        let config = MitigationConfig {
            sample_rate: fs,
            fft_size: 1024,
            detection_threshold_db: 15.0,
            max_notch_width_hz: 2000.0,
        };
        let m = SpursMitigator::new(config);

        let result = m.suppress(&samples, &[spur]);
        assert_eq!(result.len(), n);

        // Measure power at the spur frequency before and after (via correlation)
        let spur_power_before = measure_tone_power(&samples, spur_freq, fs);
        let spur_power_after = measure_tone_power(&result, spur_freq, fs);

        assert!(
            spur_power_after < spur_power_before * 0.1,
            "Spur power should be reduced by at least 10x, before={}, after={}",
            spur_power_before,
            spur_power_after
        );
    }

    #[test]
    fn test_remove_dc_offset_zeros_mean() {
        let m = SpursMitigator::new(default_config());
        let n = 4096;
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / 1_000_000.0;
                (
                    0.8 + (2.0 * PI * 50_000.0 * t).cos(),
                    -0.3 + (2.0 * PI * 50_000.0 * t).sin(),
                )
            })
            .collect();

        let cleaned = m.remove_dc_offset(&samples);
        assert_eq!(cleaned.len(), n);

        // After DC removal the mean should be near zero
        let mean_i: f64 = cleaned.iter().map(|s| s.0).sum::<f64>() / n as f64;
        let mean_q: f64 = cleaned.iter().map(|s| s.1).sum::<f64>() / n as f64;
        assert!(mean_i.abs() < 0.05, "Mean I should be near zero, got {}", mean_i);
        assert!(mean_q.abs() < 0.05, "Mean Q should be near zero, got {}", mean_q);
    }

    #[test]
    fn test_remove_dc_offset_empty() {
        let m = SpursMitigator::new(default_config());
        let result = m.remove_dc_offset(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_notch_cascade_single_freq() {
        let fs = 100_000.0;
        let n = 2048;
        let notch_freq = 15_000.0;

        let samples = generate_tone(notch_freq, fs, n, 1.0);
        let config = MitigationConfig {
            sample_rate: fs,
            fft_size: 1024,
            detection_threshold_db: 20.0,
            max_notch_width_hz: 2000.0,
        };
        let m = SpursMitigator::new(config);

        let result = m.notch_cascade(&samples, &[notch_freq]);
        assert_eq!(result.len(), n);

        let power_before = signal_power(&samples);
        let power_after = signal_power(&result);
        assert!(
            power_after < power_before * 0.1,
            "Notch should attenuate the tone significantly"
        );
    }

    #[test]
    fn test_notch_cascade_multiple_freqs() {
        let fs = 100_000.0;
        let n = 4096;

        // Two tones
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let t1_i = (2.0 * PI * 10_000.0 * t).cos();
                let t1_q = (2.0 * PI * 10_000.0 * t).sin();
                let t2_i = (2.0 * PI * 30_000.0 * t).cos();
                let t2_q = (2.0 * PI * 30_000.0 * t).sin();
                (t1_i + t2_i, t1_q + t2_q)
            })
            .collect();

        let config = MitigationConfig {
            sample_rate: fs,
            fft_size: 1024,
            detection_threshold_db: 20.0,
            max_notch_width_hz: 2000.0,
        };
        let m = SpursMitigator::new(config);

        let result = m.notch_cascade(&samples, &[10_000.0, 30_000.0]);
        let power_before = signal_power(&samples);
        let power_after = signal_power(&result);

        assert!(
            power_after < power_before * 0.15,
            "Both tones should be attenuated, ratio={}",
            power_after / power_before
        );
    }

    #[test]
    fn test_notch_cascade_empty_freqs() {
        let m = SpursMitigator::new(default_config());
        let samples = generate_tone(10_000.0, 1_000_000.0, 128, 1.0);
        let result = m.notch_cascade(&samples, &[]);
        assert_eq!(result.len(), samples.len());
        for (a, b) in result.iter().zip(samples.iter()) {
            assert!((a.0 - b.0).abs() < 1e-12);
        }
    }

    #[test]
    fn test_auto_mitigate_preserves_length() {
        let m = SpursMitigator::new(default_config());
        let samples = generate_tone(50_000.0, 1_000_000.0, 2048, 1.0);
        let result = m.auto_mitigate(&samples);
        assert_eq!(result.len(), samples.len());
    }

    #[test]
    fn test_spur_classification_lo_leakage() {
        let config = default_config();
        let m = SpursMitigator::new(config);
        let freq_res = 1_000_000.0 / 1024.0; // ~976 Hz
        let class = m.classify_spur(0.0, freq_res);
        assert_eq!(class, SpurClassification::LoLeakage);
    }

    #[test]
    fn test_spur_classification_dac_image() {
        let config = default_config();
        let m = SpursMitigator::new(config);
        let freq_res = 1_000_000.0 / 1024.0;
        let nyquist = 500_000.0;
        let class = m.classify_spur(nyquist, freq_res);
        assert_eq!(class, SpurClassification::DacImage);
    }

    #[test]
    fn test_spur_classification_unknown() {
        let config = default_config();
        let m = SpursMitigator::new(config);
        let freq_res = 1_000_000.0 / 1024.0;
        // 123456 Hz is not near DC, Nyquist, or a common clock harmonic
        let class = m.classify_spur(123_456.0, freq_res);
        assert_eq!(class, SpurClassification::Unknown);
    }

    #[test]
    fn test_noise_floor_estimation() {
        let m = SpursMitigator::new(default_config());
        let spectrum = vec![-80.0, -80.0, -80.0, -80.0, -20.0, -80.0, -80.0];
        let floor = m.estimate_noise_floor(&spectrum);
        assert!(
            (floor - (-80.0)).abs() < 1.0,
            "Noise floor should be near -80 dB, got {}",
            floor
        );
    }

    #[test]
    fn test_spur_struct_fields() {
        let spur = Spur {
            freq_hz: 10_000.0,
            power_dbm: -30.0,
            bandwidth_hz: 100.0,
            classification: SpurClassification::ClockHarmonic,
        };
        assert_eq!(spur.freq_hz, 10_000.0);
        assert_eq!(spur.power_dbm, -30.0);
        assert_eq!(spur.bandwidth_hz, 100.0);
        assert_eq!(spur.classification, SpurClassification::ClockHarmonic);
    }

    #[test]
    fn test_pll_spur_variant_exists() {
        // Ensure PllSpur variant is usable
        let class = SpurClassification::PllSpur;
        assert_ne!(class, SpurClassification::Unknown);
        assert_ne!(class, SpurClassification::ClockHarmonic);
    }

    // --- Helpers ---

    /// Measure the power of a specific tone frequency via correlation.
    fn measure_tone_power(samples: &[(f64, f64)], freq: f64, fs: f64) -> f64 {
        let n = samples.len() as f64;
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;
        for (i, s) in samples.iter().enumerate() {
            let t = i as f64 / fs;
            let phase = 2.0 * PI * freq * t;
            // Correlate complex signal with complex tone
            sum_re += s.0 * phase.cos() + s.1 * phase.sin();
            sum_im += s.1 * phase.cos() - s.0 * phase.sin();
        }
        (sum_re * sum_re + sum_im * sum_im) / (n * n)
    }
}
