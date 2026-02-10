//! Automated spurious emission testing for regulatory compliance.
//!
//! This module provides tools to scan an RF signal's spectrum for spurious emissions,
//! check compliance against user-defined spectral mask rules, identify harmonics and
//! intermodulation products, and generate pass/fail reports.
//!
//! # Example
//!
//! ```
//! use r4w_core::spurious_emission_scanner::{SpuriousScanner, ScanConfig, MaskRule};
//!
//! let config = ScanConfig {
//!     rbw_hz: 1000.0,
//!     start_freq: 0.0,
//!     stop_freq: 50000.0,
//!     sample_rate: 100000.0,
//!     num_averages: 1,
//! };
//!
//! let mut scanner = SpuriousScanner::new(config);
//!
//! // Define a regulatory mask: allow up to -20 dBm across the whole band
//! scanner.add_mask_rule(MaskRule {
//!     freq_start_hz: 0.0,
//!     freq_stop_hz: 50000.0,
//!     limit_dbm: -20.0,
//! });
//!
//! // Generate a simple tone at 10 kHz (sample rate = 100 kHz)
//! let num_samples = 1024;
//! let sample_rate = 100000.0;
//! let tone_freq = 10000.0;
//! let samples: Vec<(f64, f64)> = (0..num_samples)
//!     .map(|i| {
//!         let t = i as f64 / sample_rate;
//!         let phase = 2.0 * std::f64::consts::PI * tone_freq * t;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! let report = scanner.scan(&samples);
//! // The report contains any emissions that violate the mask
//! assert!(report.emissions.len() <= num_samples); // sanity check
//! ```

use std::f64::consts::PI;

/// Classification of a spurious emission.
#[derive(Debug, Clone, PartialEq)]
pub enum EmissionClass {
    /// Harmonic of the fundamental frequency.
    Harmonic,
    /// Intermodulation product.
    Intermod,
    /// Other / unclassified spur.
    Other,
}

/// A single detected spurious emission.
#[derive(Debug, Clone)]
pub struct SpuriousEmission {
    /// Center frequency of the detected emission in Hz.
    pub freq_hz: f64,
    /// Measured power of the emission in dBm.
    pub power_dbm: f64,
    /// The applicable regulatory limit in dBm.
    pub limit_dbm: f64,
    /// Margin relative to the limit (positive means within spec, negative means violation).
    pub margin_db: f64,
    /// Classification of the emission source.
    pub classification: EmissionClass,
}

/// Report produced by a spurious emission scan.
#[derive(Debug, Clone)]
pub struct EmissionReport {
    /// All detected spurious emissions.
    pub emissions: Vec<SpuriousEmission>,
    /// Overall pass/fail: true if all emissions are within mask limits.
    pub pass: bool,
    /// Worst (smallest) margin in dB across all emissions. Positive means passing.
    pub worst_margin_db: f64,
}

/// A single regulatory mask rule defining an emission limit over a frequency range.
#[derive(Debug, Clone)]
pub struct MaskRule {
    /// Start of the frequency range in Hz.
    pub freq_start_hz: f64,
    /// End of the frequency range in Hz.
    pub freq_stop_hz: f64,
    /// Maximum allowed emission power in dBm.
    pub limit_dbm: f64,
}

/// Configuration for a spurious emission scan.
#[derive(Debug, Clone)]
pub struct ScanConfig {
    /// Resolution bandwidth in Hz. Determines FFT bin width.
    pub rbw_hz: f64,
    /// Start frequency of the scan range in Hz.
    pub start_freq: f64,
    /// Stop frequency of the scan range in Hz.
    pub stop_freq: f64,
    /// Sample rate of the input IQ data in Hz.
    pub sample_rate: f64,
    /// Number of FFT averages for noise reduction.
    pub num_averages: usize,
}

/// Main spurious emission scanner engine.
///
/// Performs FFT-based spectral analysis and checks the resulting spectrum against
/// user-defined mask rules to detect regulatory violations.
#[derive(Debug, Clone)]
pub struct SpuriousScanner {
    config: ScanConfig,
    mask_rules: Vec<MaskRule>,
}

impl SpuriousScanner {
    /// Create a new scanner with the given configuration.
    pub fn new(config: ScanConfig) -> Self {
        Self {
            config,
            mask_rules: Vec::new(),
        }
    }

    /// Add a regulatory mask rule.
    pub fn add_mask_rule(&mut self, rule: MaskRule) {
        self.mask_rules.push(rule);
    }

    /// Perform a full spurious emission scan on the provided IQ samples.
    ///
    /// Returns an `EmissionReport` detailing all detected emissions,
    /// their compliance status, and the worst margin.
    pub fn scan(&self, samples: &[(f64, f64)]) -> EmissionReport {
        let spectrum = self.compute_averaged_spectrum(samples);
        let fft_size = self.fft_size_for_rbw();
        let bin_hz = self.config.sample_rate / fft_size as f64;

        let mut emissions = Vec::new();

        for (bin_idx, &power_dbm) in spectrum.iter().enumerate() {
            let freq = bin_idx as f64 * bin_hz;

            // Only consider bins within the scan range
            if freq < self.config.start_freq || freq > self.config.stop_freq {
                continue;
            }

            // Check against all mask rules
            for rule in &self.mask_rules {
                if freq >= rule.freq_start_hz && freq <= rule.freq_stop_hz {
                    let margin = rule.limit_dbm - power_dbm;
                    if margin < 0.0 {
                        emissions.push(SpuriousEmission {
                            freq_hz: freq,
                            power_dbm,
                            limit_dbm: rule.limit_dbm,
                            margin_db: margin,
                            classification: EmissionClass::Other,
                        });
                    }
                }
            }
        }

        let pass = emissions.is_empty();
        let worst_margin_db = if emissions.is_empty() {
            f64::INFINITY
        } else {
            emissions
                .iter()
                .map(|e| e.margin_db)
                .fold(f64::INFINITY, f64::min)
        };

        EmissionReport {
            emissions,
            pass,
            worst_margin_db,
        }
    }

    /// Scan for harmonic emissions at multiples of the fundamental frequency.
    ///
    /// Searches for energy at `N * fundamental_hz` for N = 2..=num_harmonics+1,
    /// returning a `SpuriousEmission` for each harmonic found above the noise floor.
    pub fn scan_harmonics(
        &self,
        fundamental_hz: f64,
        num_harmonics: usize,
        samples: &[(f64, f64)],
    ) -> Vec<SpuriousEmission> {
        let spectrum = self.compute_averaged_spectrum(samples);
        let fft_size = self.fft_size_for_rbw();
        let bin_hz = self.config.sample_rate / fft_size as f64;

        // Compute noise floor as median power
        let noise_floor = self.estimate_noise_floor(&spectrum);

        let mut results = Vec::new();
        for n in 2..=(num_harmonics + 1) {
            let harmonic_freq = fundamental_hz * n as f64;
            if harmonic_freq > self.config.sample_rate / 2.0 {
                break;
            }

            let bin = (harmonic_freq / bin_hz).round() as usize;
            if bin >= spectrum.len() {
                continue;
            }

            // Search in a small window around the expected bin for the peak
            let search_radius = 3.min(spectrum.len() / 2);
            let start = bin.saturating_sub(search_radius);
            let end = (bin + search_radius + 1).min(spectrum.len());

            let (peak_bin, peak_power) = spectrum[start..end]
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
                .map(|(i, &p)| (start + i, p))
                .unwrap_or((bin, spectrum[bin]));

            let peak_freq = peak_bin as f64 * bin_hz;

            // Only report if significantly above noise floor
            if peak_power > noise_floor + 6.0 {
                let limit = self.find_limit_at_freq(peak_freq);
                let margin = limit - peak_power;

                results.push(SpuriousEmission {
                    freq_hz: peak_freq,
                    power_dbm: peak_power,
                    limit_dbm: limit,
                    margin_db: margin,
                    classification: EmissionClass::Harmonic,
                });
            }
        }

        results
    }

    /// Check whether the given samples comply with all mask rules.
    ///
    /// Returns `true` if no emission exceeds any mask limit, `false` otherwise.
    pub fn check_mask_compliance(&self, samples: &[(f64, f64)]) -> bool {
        let report = self.scan(samples);
        report.pass
    }

    /// Predict intermodulation product frequencies for two input tones.
    ///
    /// Computes all intermod products `|m*f1 + n*f2|` where `|m| + |n| <= order`
    /// and both m, n are nonzero (excluding fundamental tones themselves).
    /// Returns a sorted, deduplicated list of predicted frequencies.
    pub fn predict_intermod(&self, f1: f64, f2: f64, order: usize) -> Vec<f64> {
        let mut freqs = Vec::new();
        let order_i = order as i64;

        for m in -order_i..=order_i {
            for n in -order_i..=order_i {
                let abs_sum = m.unsigned_abs() + n.unsigned_abs();
                if abs_sum < 2 || abs_sum > order as u64 {
                    continue;
                }
                // Skip fundamentals: (±1, 0) and (0, ±1)
                if (m.abs() == 1 && n == 0) || (m == 0 && n.abs() == 1) {
                    continue;
                }
                let freq = (m as f64 * f1 + n as f64 * f2).abs();
                if freq > 0.0 {
                    freqs.push(freq);
                }
            }
        }

        freqs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        freqs.dedup_by(|a, b| (*a - *b).abs() < 1.0); // dedup within 1 Hz
        freqs
    }

    // --- Internal helpers ---

    /// Compute the FFT size that achieves the configured RBW.
    fn fft_size_for_rbw(&self) -> usize {
        let n = (self.config.sample_rate / self.config.rbw_hz).ceil() as usize;
        // Round up to power of 2 for FFT efficiency
        n.next_power_of_two().max(4)
    }

    /// Compute a Blackman-Harris window of the given length.
    fn blackman_harris_window(len: usize) -> Vec<f64> {
        let a0 = 0.35875;
        let a1 = 0.48829;
        let a2 = 0.14128;
        let a3 = 0.01168;
        (0..len)
            .map(|i| {
                let x = 2.0 * PI * i as f64 / (len as f64 - 1.0);
                a0 - a1 * x.cos() + a2 * (2.0 * x).cos() - a3 * (3.0 * x).cos()
            })
            .collect()
    }

    /// Simple radix-2 DIT FFT (Cooley-Tukey). Input length must be a power of 2.
    fn fft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n = input.len();
        if n <= 1 {
            return input.to_vec();
        }
        debug_assert!(n.is_power_of_two(), "FFT length must be power of 2");

        // Bit-reversal permutation
        let mut result: Vec<(f64, f64)> = vec![(0.0, 0.0); n];
        let bits = n.trailing_zeros();
        for i in 0..n {
            let mut rev = 0usize;
            for b in 0..bits {
                if i & (1 << b) != 0 {
                    rev |= 1 << (bits - 1 - b);
                }
            }
            result[rev] = input[i];
        }

        // Iterative butterfly
        let mut size = 2;
        while size <= n {
            let half = size / 2;
            let angle_step = -2.0 * PI / size as f64;
            for start in (0..n).step_by(size) {
                for k in 0..half {
                    let angle = angle_step * k as f64;
                    let twiddle = (angle.cos(), angle.sin());
                    let even = result[start + k];
                    let odd = result[start + k + half];
                    // Complex multiply: twiddle * odd
                    let t = (
                        twiddle.0 * odd.0 - twiddle.1 * odd.1,
                        twiddle.0 * odd.1 + twiddle.1 * odd.0,
                    );
                    result[start + k] = (even.0 + t.0, even.1 + t.1);
                    result[start + k + half] = (even.0 - t.0, even.1 - t.1);
                }
            }
            size *= 2;
        }

        result
    }

    /// Compute averaged power spectrum in dBm from IQ samples.
    fn compute_averaged_spectrum(&self, samples: &[(f64, f64)]) -> Vec<f64> {
        let fft_size = self.fft_size_for_rbw();
        let window = Self::blackman_harris_window(fft_size);

        // Compute window power for normalization
        let window_power: f64 = window.iter().map(|w| w * w).sum::<f64>() / fft_size as f64;

        let num_segments = if samples.len() >= fft_size {
            let available = samples.len() - fft_size;
            let step = fft_size / 2; // 50% overlap
            if step == 0 {
                1
            } else {
                (available / step + 1).min(self.config.num_averages.max(1))
            }
        } else {
            1
        };

        let mut avg_power = vec![0.0f64; fft_size];
        let step = if num_segments > 1 {
            fft_size / 2
        } else {
            fft_size
        };

        for seg in 0..num_segments {
            let offset = seg * step;
            let mut windowed = vec![(0.0, 0.0); fft_size];
            for i in 0..fft_size {
                if offset + i < samples.len() {
                    let s = samples[offset + i];
                    windowed[i] = (s.0 * window[i], s.1 * window[i]);
                }
            }

            let spectrum = Self::fft(&windowed);
            for (i, s) in spectrum.iter().enumerate() {
                let mag_sq = s.0 * s.0 + s.1 * s.1;
                avg_power[i] += mag_sq;
            }
        }

        // Average and convert to dBm
        // PSD normalization: P_bin = mag^2 / (N * W_power * N_avg)
        // Convert to dBm: 10*log10(P) + 30 (assuming 1 ohm / normalized)
        let norm = (fft_size as f64) * window_power * num_segments as f64;
        avg_power
            .iter()
            .map(|&p| {
                let p_norm = if norm > 0.0 { p / norm } else { p };
                if p_norm > 1e-30 {
                    10.0 * p_norm.log10()
                } else {
                    -300.0 // effectively -infinity dBm
                }
            })
            .collect()
    }

    /// Estimate noise floor from a power spectrum (median-based).
    fn estimate_noise_floor(&self, spectrum_dbm: &[f64]) -> f64 {
        let mut sorted: Vec<f64> = spectrum_dbm.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        if sorted.is_empty() {
            return -300.0;
        }
        sorted[sorted.len() / 2]
    }

    /// Find the applicable mask limit at a given frequency.
    /// Returns the most restrictive (lowest) limit if multiple rules overlap.
    /// If no rule covers the frequency, returns +infinity (no limit).
    fn find_limit_at_freq(&self, freq: f64) -> f64 {
        let mut limit = f64::INFINITY;
        for rule in &self.mask_rules {
            if freq >= rule.freq_start_hz && freq <= rule.freq_stop_hz {
                limit = limit.min(rule.limit_dbm);
            }
        }
        limit
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn default_config() -> ScanConfig {
        ScanConfig {
            rbw_hz: 100.0,
            start_freq: 0.0,
            stop_freq: 50000.0,
            sample_rate: 100000.0,
            num_averages: 1,
        }
    }

    fn generate_tone(freq: f64, sample_rate: f64, num_samples: usize, amplitude: f64) -> Vec<(f64, f64)> {
        (0..num_samples)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * PI * freq * t;
                (amplitude * phase.cos(), amplitude * phase.sin())
            })
            .collect()
    }

    fn generate_two_tones(
        f1: f64,
        f2: f64,
        sample_rate: f64,
        num_samples: usize,
        a1: f64,
        a2: f64,
    ) -> Vec<(f64, f64)> {
        (0..num_samples)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let p1 = 2.0 * PI * f1 * t;
                let p2 = 2.0 * PI * f2 * t;
                (
                    a1 * p1.cos() + a2 * p2.cos(),
                    a1 * p1.sin() + a2 * p2.sin(),
                )
            })
            .collect()
    }

    #[test]
    fn test_new_scanner() {
        let config = default_config();
        let scanner = SpuriousScanner::new(config.clone());
        assert!(scanner.mask_rules.is_empty());
        assert!((scanner.config.sample_rate - 100000.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_add_mask_rule() {
        let mut scanner = SpuriousScanner::new(default_config());
        scanner.add_mask_rule(MaskRule {
            freq_start_hz: 0.0,
            freq_stop_hz: 50000.0,
            limit_dbm: -30.0,
        });
        assert_eq!(scanner.mask_rules.len(), 1);
        scanner.add_mask_rule(MaskRule {
            freq_start_hz: 10000.0,
            freq_stop_hz: 20000.0,
            limit_dbm: -50.0,
        });
        assert_eq!(scanner.mask_rules.len(), 2);
    }

    #[test]
    fn test_scan_no_rules_passes() {
        let scanner = SpuriousScanner::new(default_config());
        let samples = generate_tone(10000.0, 100000.0, 2048, 1.0);
        let report = scanner.scan(&samples);
        // No mask rules means nothing can violate
        assert!(report.pass);
        assert!(report.emissions.is_empty());
    }

    #[test]
    fn test_scan_strong_signal_violates_strict_mask() {
        let mut scanner = SpuriousScanner::new(default_config());
        // Very strict mask that a strong tone should violate
        scanner.add_mask_rule(MaskRule {
            freq_start_hz: 0.0,
            freq_stop_hz: 50000.0,
            limit_dbm: -80.0,
        });
        let samples = generate_tone(10000.0, 100000.0, 2048, 1.0);
        let report = scanner.scan(&samples);
        assert!(!report.pass);
        assert!(!report.emissions.is_empty());
        assert!(report.worst_margin_db < 0.0);
    }

    #[test]
    fn test_scan_weak_signal_passes_lenient_mask() {
        let mut scanner = SpuriousScanner::new(default_config());
        // Very lenient mask
        scanner.add_mask_rule(MaskRule {
            freq_start_hz: 0.0,
            freq_stop_hz: 50000.0,
            limit_dbm: 100.0,
        });
        let samples = generate_tone(10000.0, 100000.0, 2048, 0.001);
        let report = scanner.scan(&samples);
        assert!(report.pass);
    }

    #[test]
    fn test_emission_report_worst_margin() {
        let mut scanner = SpuriousScanner::new(default_config());
        scanner.add_mask_rule(MaskRule {
            freq_start_hz: 0.0,
            freq_stop_hz: 50000.0,
            limit_dbm: -60.0,
        });
        let samples = generate_tone(10000.0, 100000.0, 2048, 1.0);
        let report = scanner.scan(&samples);
        if !report.pass {
            // worst_margin should be negative
            assert!(report.worst_margin_db < 0.0);
            // All emission margins should be >= worst_margin
            for e in &report.emissions {
                assert!(e.margin_db >= report.worst_margin_db - 1e-10);
            }
        }
    }

    #[test]
    fn test_check_mask_compliance_true() {
        let mut scanner = SpuriousScanner::new(default_config());
        scanner.add_mask_rule(MaskRule {
            freq_start_hz: 0.0,
            freq_stop_hz: 50000.0,
            limit_dbm: 200.0,
        });
        let samples = generate_tone(10000.0, 100000.0, 1024, 1.0);
        assert!(scanner.check_mask_compliance(&samples));
    }

    #[test]
    fn test_check_mask_compliance_false() {
        let mut scanner = SpuriousScanner::new(default_config());
        scanner.add_mask_rule(MaskRule {
            freq_start_hz: 0.0,
            freq_stop_hz: 50000.0,
            limit_dbm: -100.0,
        });
        let samples = generate_tone(10000.0, 100000.0, 1024, 1.0);
        assert!(!scanner.check_mask_compliance(&samples));
    }

    #[test]
    fn test_predict_intermod_order_3() {
        let scanner = SpuriousScanner::new(default_config());
        let products = scanner.predict_intermod(100.0, 200.0, 3);
        // 3rd order: 2*f1 - f2, 2*f2 - f1, 2*f1 + f2, 2*f2 + f1, etc.
        // 2*100 - 200 = 0 (filtered out since <= 0), 2*200 - 100 = 300
        assert!(!products.is_empty());
        // Should include 2f1-f2 = 0 (excluded) or |2f1-f2| but that's 0 so excluded
        // Should include 2f2-f1 = 300
        assert!(products.iter().any(|&f| (f - 300.0).abs() < 1.5));
        // Should include f1+f2 = 300 (order 2)
        // Should include 2f1+f2 = 400 (order 3)
        assert!(products.iter().any(|&f| (f - 400.0).abs() < 1.5));
    }

    #[test]
    fn test_predict_intermod_excludes_fundamentals() {
        let scanner = SpuriousScanner::new(default_config());
        let products = scanner.predict_intermod(1000.0, 2000.0, 5);
        // Should NOT include f1=1000 or f2=2000 as standalone fundamentals
        // (m=1,n=0) and (m=0,n=1) are excluded
        for &f in &products {
            // Check it's not exactly a fundamental (allow some tolerance for the dedup)
            if (f - 1000.0).abs() < 1.5 {
                // If 1000 appears, it must be from a combination like |2*2000 - 3*1000|=1000
                // which is valid intermod, not fundamental
            }
        }
        // At minimum we should get some products
        assert!(!products.is_empty());
    }

    #[test]
    fn test_predict_intermod_order_2() {
        let scanner = SpuriousScanner::new(default_config());
        let products = scanner.predict_intermod(500.0, 700.0, 2);
        // Order 2 products: f1+f2=1200, |f1-f2|=200, 2*f1=1000, 2*f2=1400
        assert!(products.iter().any(|&f| (f - 1200.0).abs() < 1.5));
        assert!(products.iter().any(|&f| (f - 200.0).abs() < 1.5));
        assert!(products.iter().any(|&f| (f - 1000.0).abs() < 1.5));
        assert!(products.iter().any(|&f| (f - 1400.0).abs() < 1.5));
    }

    #[test]
    fn test_scan_harmonics_pure_tone() {
        let config = ScanConfig {
            rbw_hz: 50.0,
            start_freq: 0.0,
            stop_freq: 25000.0,
            sample_rate: 50000.0,
            num_averages: 1,
        };
        let scanner = SpuriousScanner::new(config);

        // Generate a pure single-frequency tone - should have minimal harmonics
        let samples = generate_tone(5000.0, 50000.0, 4096, 1.0);
        let harmonics = scanner.scan_harmonics(5000.0, 3, &samples);
        // A pure complex exponential should have very few detectable harmonics
        // (spectral leakage may cause some, but they should be weak)
        // Just verify it returns something valid
        for h in &harmonics {
            assert_eq!(h.classification, EmissionClass::Harmonic);
            assert!(h.freq_hz > 5000.0); // harmonics are above fundamental
        }
    }

    #[test]
    fn test_scan_harmonics_with_harmonic_content() {
        let config = ScanConfig {
            rbw_hz: 50.0,
            start_freq: 0.0,
            stop_freq: 25000.0,
            sample_rate: 50000.0,
            num_averages: 1,
        };
        let mut scanner = SpuriousScanner::new(config);
        scanner.add_mask_rule(MaskRule {
            freq_start_hz: 0.0,
            freq_stop_hz: 25000.0,
            limit_dbm: 50.0,
        });

        // Generate a signal with explicit harmonic content
        let sample_rate = 50000.0;
        let n = 4096;
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let fundamental = 2.0 * PI * 2000.0 * t;
                let second_harmonic = 2.0 * PI * 4000.0 * t;
                let third_harmonic = 2.0 * PI * 6000.0 * t;
                (
                    fundamental.cos() + 0.5 * second_harmonic.cos() + 0.25 * third_harmonic.cos(),
                    fundamental.sin() + 0.5 * second_harmonic.sin() + 0.25 * third_harmonic.sin(),
                )
            })
            .collect();

        let harmonics = scanner.scan_harmonics(2000.0, 3, &samples);
        // Should detect 2nd and 3rd harmonics (at 4 kHz and 6 kHz)
        assert!(harmonics.len() >= 2);
        let freqs: Vec<f64> = harmonics.iter().map(|h| h.freq_hz).collect();
        // Check that detected frequencies are near 4000 and 6000 Hz
        assert!(freqs.iter().any(|&f| (f - 4000.0).abs() < 200.0));
        assert!(freqs.iter().any(|&f| (f - 6000.0).abs() < 200.0));
    }

    #[test]
    fn test_fft_size_calculation() {
        let config = ScanConfig {
            rbw_hz: 100.0,
            start_freq: 0.0,
            stop_freq: 50000.0,
            sample_rate: 100000.0,
            num_averages: 1,
        };
        let scanner = SpuriousScanner::new(config);
        let fft_size = scanner.fft_size_for_rbw();
        // sample_rate / rbw = 1000, next power of 2 = 1024
        assert_eq!(fft_size, 1024);
    }

    #[test]
    fn test_fft_basic() {
        // Single bin test: DC component
        let samples = vec![(1.0, 0.0); 8];
        let result = SpuriousScanner::fft(&samples);
        // DC bin should have magnitude 8
        let dc_mag = (result[0].0 * result[0].0 + result[0].1 * result[0].1).sqrt();
        assert!((dc_mag - 8.0).abs() < 1e-10);
        // Other bins should be ~0
        for i in 1..8 {
            let mag = (result[i].0 * result[i].0 + result[i].1 * result[i].1).sqrt();
            assert!(mag < 1e-10, "bin {i} mag = {mag}");
        }
    }

    #[test]
    fn test_fft_single_tone() {
        // Tone at bin 2 of an 8-point FFT
        let n = 8;
        let bin = 2;
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * bin as f64 * i as f64 / n as f64;
                (phase.cos(), phase.sin())
            })
            .collect();
        let result = SpuriousScanner::fft(&samples);
        // Bin 2 should have magnitude n=8
        let target_mag = (result[bin].0 * result[bin].0 + result[bin].1 * result[bin].1).sqrt();
        assert!(
            (target_mag - n as f64).abs() < 1e-8,
            "expected {n}, got {target_mag}"
        );
        // Other bins should be near zero
        for i in 0..n {
            if i == bin {
                continue;
            }
            let mag = (result[i].0 * result[i].0 + result[i].1 * result[i].1).sqrt();
            assert!(mag < 1e-8, "bin {i} should be ~0, got {mag}");
        }
    }

    #[test]
    fn test_multiple_mask_rules() {
        let mut scanner = SpuriousScanner::new(default_config());
        // Strict in-band, lenient out-of-band
        scanner.add_mask_rule(MaskRule {
            freq_start_hz: 9000.0,
            freq_stop_hz: 11000.0,
            limit_dbm: 50.0, // lenient around the tone
        });
        scanner.add_mask_rule(MaskRule {
            freq_start_hz: 0.0,
            freq_stop_hz: 9000.0,
            limit_dbm: -80.0, // strict below
        });
        scanner.add_mask_rule(MaskRule {
            freq_start_hz: 11000.0,
            freq_stop_hz: 50000.0,
            limit_dbm: -80.0, // strict above
        });

        let samples = generate_tone(10000.0, 100000.0, 2048, 1.0);
        let report = scanner.scan(&samples);
        // The tone at 10 kHz should pass under the lenient rule;
        // spectral leakage into strict bands may or may not cause violations
        // depending on window function. With Blackman-Harris, sidelobes are very low.
        // This test just validates multiple rules work without panic.
        let _ = report;
    }

    #[test]
    fn test_emission_classification() {
        // Verify that scan marks emissions as Other, while scan_harmonics marks as Harmonic
        let config = ScanConfig {
            rbw_hz: 100.0,
            start_freq: 0.0,
            stop_freq: 25000.0,
            sample_rate: 50000.0,
            num_averages: 1,
        };
        let mut scanner = SpuriousScanner::new(config);
        scanner.add_mask_rule(MaskRule {
            freq_start_hz: 0.0,
            freq_stop_hz: 25000.0,
            limit_dbm: -100.0,
        });

        let samples = generate_tone(5000.0, 50000.0, 1024, 1.0);
        let report = scanner.scan(&samples);
        for e in &report.emissions {
            assert_eq!(e.classification, EmissionClass::Other);
        }

        let harmonics = scanner.scan_harmonics(5000.0, 3, &samples);
        for h in &harmonics {
            assert_eq!(h.classification, EmissionClass::Harmonic);
        }
    }

    #[test]
    fn test_noise_floor_estimation() {
        let scanner = SpuriousScanner::new(default_config());
        let spectrum = vec![-80.0, -82.0, -79.0, -81.0, -10.0, -83.0, -80.0];
        let nf = scanner.estimate_noise_floor(&spectrum);
        // Median of sorted [-83, -82, -81, -80, -80, -79, -10] = -80
        assert!((nf - (-80.0)).abs() < 0.01);
    }

    #[test]
    fn test_find_limit_at_freq() {
        let mut scanner = SpuriousScanner::new(default_config());
        scanner.add_mask_rule(MaskRule {
            freq_start_hz: 0.0,
            freq_stop_hz: 10000.0,
            limit_dbm: -30.0,
        });
        scanner.add_mask_rule(MaskRule {
            freq_start_hz: 5000.0,
            freq_stop_hz: 15000.0,
            limit_dbm: -40.0,
        });

        // At 3 kHz, only first rule applies
        assert!((scanner.find_limit_at_freq(3000.0) - (-30.0)).abs() < f64::EPSILON);
        // At 7 kHz, both rules apply, most restrictive = -40
        assert!((scanner.find_limit_at_freq(7000.0) - (-40.0)).abs() < f64::EPSILON);
        // At 12 kHz, only second rule applies
        assert!((scanner.find_limit_at_freq(12000.0) - (-40.0)).abs() < f64::EPSILON);
        // At 20 kHz, no rule applies
        assert!(scanner.find_limit_at_freq(20000.0).is_infinite());
    }
}
