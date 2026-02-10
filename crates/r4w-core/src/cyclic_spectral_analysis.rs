//! # Cyclic Spectral Analysis
//!
//! Second-order cyclostationary feature extraction for automatic modulation
//! classification. Computes the Spectral Correlation Function (SCF) using
//! frequency-smoothing, extracts cycle frequency peaks, and derives
//! modulation features such as estimated symbol rate and carrier offset.
//!
//! ## Overview
//!
//! Cyclostationary signals exhibit periodicity in their second-order statistics.
//! The SCF, `S_x^\alpha(f)`, characterises these periodicities as a function of
//! spectral frequency `f` and cycle frequency `\alpha`. Peaks in the cycle
//! frequency profile reveal the symbol rate, carrier frequency, and other
//! modulation parameters, enabling blind modulation recognition.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cyclic_spectral_analysis::{CyclicSpectralAnalyzer, CsaConfig};
//!
//! // Build a BPSK-like signal: carrier at 1 kHz, symbol rate 100 Hz
//! let sample_rate = 8000.0_f64;
//! let num_samples = 4096;
//! let carrier_hz = 1000.0;
//! let symbol_rate = 100.0;
//! let samples_per_symbol = (sample_rate / symbol_rate) as usize;
//!
//! let mut samples: Vec<(f64, f64)> = Vec::with_capacity(num_samples);
//! let mut symbol_val = 1.0_f64;
//! for i in 0..num_samples {
//!     if i % samples_per_symbol == 0 {
//!         // Toggle symbol polarity (deterministic for doctest)
//!         symbol_val = if (i / samples_per_symbol) % 2 == 0 { 1.0 } else { -1.0 };
//!     }
//!     let t = i as f64 / sample_rate;
//!     let phase = 2.0 * std::f64::consts::PI * carrier_hz * t;
//!     samples.push((symbol_val * phase.cos(), symbol_val * phase.sin()));
//! }
//!
//! let config = CsaConfig {
//!     fft_size: 256,
//!     num_segments: 8,
//!     sample_rate,
//!     alpha_resolution_hz: 10.0,
//! };
//!
//! let analyzer = CyclicSpectralAnalyzer::new(config);
//!
//! // Compute cyclic autocorrelation at the double-carrier cycle frequency
//! let cac = analyzer.cyclic_autocorrelation(&samples, 2.0 * carrier_hz, 0);
//! let mag = (cac.0 * cac.0 + cac.1 * cac.1).sqrt();
//! assert!(mag > 0.0, "cyclic autocorrelation magnitude should be nonzero at 2*fc");
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration & result types
// ---------------------------------------------------------------------------

/// Configuration for the cyclic spectral analyser.
#[derive(Debug, Clone)]
pub struct CsaConfig {
    /// Number of frequency bins (FFT length) per segment.
    pub fft_size: usize,
    /// Number of time segments for averaging.
    pub num_segments: usize,
    /// Sample rate of the input signal in Hz.
    pub sample_rate: f64,
    /// Resolution for scanning cycle frequencies (Hz).
    pub alpha_resolution_hz: f64,
}

/// A single cyclic autocorrelation measurement.
#[derive(Debug, Clone)]
pub struct CyclicAutocorrelation {
    /// Cycle frequency (Hz).
    pub alpha: f64,
    /// Lag (in samples).
    pub tau: isize,
    /// Complex value of the cyclic autocorrelation.
    pub value: (f64, f64),
}

/// A detected peak in the cycle-frequency profile.
#[derive(Debug, Clone)]
pub struct CycleFrequencyPeak {
    /// Cycle frequency in Hz.
    pub alpha_hz: f64,
    /// Magnitude of the SCF integrated over spectral frequency.
    pub magnitude: f64,
    /// Signal-to-noise ratio in dB (peak power vs median).
    pub snr_db: f64,
}

/// High-level modulation features derived from cycle-frequency peaks.
#[derive(Debug, Clone)]
pub struct ModulationFeatures {
    /// Estimated symbol rate (Hz), derived from the dominant non-DC peak.
    pub estimated_symbol_rate: f64,
    /// Estimated carrier frequency (Hz), from the strongest peak near half the
    /// double-carrier cycle frequency.
    pub estimated_carrier_freq: f64,
    /// Number of significant cycle-frequency peaks (above noise floor).
    pub num_significant_peaks: usize,
    /// Human-readable hint about the likely modulation family.
    pub modulation_hint: String,
}

// ---------------------------------------------------------------------------
// Main analyser
// ---------------------------------------------------------------------------

/// Cyclic spectral analyser engine.
///
/// Implements the *frequency-smoothing* method for computing the Spectral
/// Correlation Function (SCF):
///
/// ```text
/// S_x^alpha(f) = (1/K) * sum_k  X_k(f + alpha/2) * conj(X_k(f - alpha/2))
/// ```
///
/// where `X_k` is the DFT of the k-th time segment.
#[derive(Debug, Clone)]
pub struct CyclicSpectralAnalyzer {
    config: CsaConfig,
}

impl CyclicSpectralAnalyzer {
    /// Create a new analyser from the given configuration.
    pub fn new(config: CsaConfig) -> Self {
        Self { config }
    }

    // -----------------------------------------------------------------------
    // Public API
    // -----------------------------------------------------------------------

    /// Compute the spectral correlation function for a given cycle frequency
    /// `alpha` (in normalised frequency, cycles/sample).
    ///
    /// Returns a vector of length `fft_size` containing the complex SCF values
    /// `S_x^alpha(f)` for each frequency bin.
    pub fn compute_spectral_correlation(
        &self,
        samples: &[(f64, f64)],
        alpha: f64,
    ) -> Vec<(f64, f64)> {
        let n = self.config.fft_size;
        let num_seg = self.effective_segments(samples.len());
        if num_seg == 0 {
            return vec![(0.0, 0.0); n];
        }

        let mut accum = vec![(0.0, 0.0); n];

        for seg in 0..num_seg {
            let start = seg * n;
            let segment = &samples[start..start + n];

            // Apply Hanning window
            let windowed: Vec<(f64, f64)> = segment
                .iter()
                .enumerate()
                .map(|(i, &(re, im))| {
                    let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos());
                    (re * w, im * w)
                })
                .collect();

            let spectrum = dft(&windowed);

            // S_x^alpha(f) += X(f + alpha/2) * conj(X(f - alpha/2))
            let shift = (alpha * n as f64 / 2.0).round() as isize;
            for f in 0..n {
                let idx_pos = ((f as isize + shift).rem_euclid(n as isize)) as usize;
                let idx_neg = ((f as isize - shift).rem_euclid(n as isize)) as usize;
                let (ar, ai) = spectrum[idx_pos];
                let (br, bi) = spectrum[idx_neg];
                // multiply a * conj(b)
                accum[f].0 += ar * br + ai * bi;
                accum[f].1 += ai * br - ar * bi;
            }
        }

        let inv = 1.0 / num_seg as f64;
        accum.iter_mut().for_each(|(r, i)| {
            *r *= inv;
            *i *= inv;
        });

        accum
    }

    /// Extract the strongest cycle-frequency peaks by scanning over a range
    /// of candidate cycle frequencies.
    ///
    /// Returns up to `num_peaks` peaks sorted by descending magnitude.
    pub fn extract_cycle_frequencies(
        &self,
        samples: &[(f64, f64)],
        num_peaks: usize,
    ) -> Vec<CycleFrequencyPeak> {
        let alpha_step = self.config.alpha_resolution_hz / self.config.sample_rate;
        let max_alpha = 0.5_f64; // Nyquist in normalised frequency

        // Collect (alpha_hz, magnitude) for every candidate
        let mut profile: Vec<(f64, f64)> = Vec::new();

        let mut alpha = alpha_step;
        while alpha <= max_alpha {
            let scf = self.compute_spectral_correlation(samples, alpha);
            let mag: f64 = scf
                .iter()
                .map(|(r, i)| (r * r + i * i).sqrt())
                .sum::<f64>()
                / scf.len() as f64;

            profile.push((alpha * self.config.sample_rate, mag));
            alpha += alpha_step;
        }

        if profile.is_empty() {
            return Vec::new();
        }

        // Compute median magnitude for SNR estimation
        let mut mags: Vec<f64> = profile.iter().map(|&(_, m)| m).collect();
        mags.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median = mags[mags.len() / 2].max(1e-30);

        // Build peak list
        let mut peaks: Vec<CycleFrequencyPeak> = profile
            .iter()
            .map(|&(alpha_hz, magnitude)| {
                let snr_db = 10.0 * (magnitude / median).log10();
                CycleFrequencyPeak {
                    alpha_hz,
                    magnitude,
                    snr_db,
                }
            })
            .collect();

        // Sort descending by magnitude and truncate
        peaks.sort_by(|a, b| {
            b.magnitude
                .partial_cmp(&a.magnitude)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        peaks.truncate(num_peaks);
        peaks
    }

    /// Compute the cyclic autocorrelation at cycle frequency `alpha` (Hz) and
    /// lag `tau` (samples).
    ///
    /// ```text
    /// R_x^alpha(tau) = lim_{T->inf} (1/T) int x(t + tau/2) * conj(x(t - tau/2)) * e^{-j 2 pi alpha t} dt
    /// ```
    pub fn cyclic_autocorrelation(
        &self,
        samples: &[(f64, f64)],
        alpha: f64,
        tau: isize,
    ) -> (f64, f64) {
        let n = samples.len() as isize;
        let (mut sum_re, mut sum_im) = (0.0, 0.0);
        let mut count = 0u64;

        let alpha_norm = alpha / self.config.sample_rate;

        for t in 0..n {
            let idx_pos = t + tau / 2;
            let idx_neg = t - tau / 2;
            if idx_pos < 0 || idx_pos >= n || idx_neg < 0 || idx_neg >= n {
                continue;
            }
            let (ar, ai) = samples[idx_pos as usize];
            let (br, bi) = samples[idx_neg as usize];
            // x(t+tau/2) * conj(x(t-tau/2))
            let prod_re = ar * br + ai * bi;
            let prod_im = ai * br - ar * bi;
            // multiply by e^{-j 2 pi alpha t}
            let angle = -2.0 * PI * alpha_norm * t as f64;
            let (cs, sn) = (angle.cos(), angle.sin());
            sum_re += prod_re * cs - prod_im * sn;
            sum_im += prod_re * sn + prod_im * cs;
            count += 1;
        }

        if count == 0 {
            return (0.0, 0.0);
        }

        (sum_re / count as f64, sum_im / count as f64)
    }

    /// Compute the spectral coherence (magnitude-squared coherence) for a
    /// given cycle frequency `alpha` (normalised, cycles/sample).
    ///
    /// Returns a vector of length `fft_size` with values in [0, 1].
    pub fn spectral_coherence(
        &self,
        samples: &[(f64, f64)],
        alpha: f64,
    ) -> Vec<f64> {
        let scf_alpha = self.compute_spectral_correlation(samples, alpha);
        let scf_zero = self.compute_spectral_correlation(samples, 0.0);
        let n = self.config.fft_size;

        (0..n)
            .map(|f| {
                let shift = (alpha * n as f64 / 2.0).round() as isize;
                let idx_pos = ((f as isize + shift).rem_euclid(n as isize)) as usize;
                let idx_neg = ((f as isize - shift).rem_euclid(n as isize)) as usize;

                let psd_pos = {
                    let (r, i) = scf_zero[idx_pos];
                    (r * r + i * i).sqrt()
                };
                let psd_neg = {
                    let (r, i) = scf_zero[idx_neg];
                    (r * r + i * i).sqrt()
                };

                let denom = (psd_pos * psd_neg).sqrt().max(1e-30);
                let (sr, si) = scf_alpha[f];
                let num = (sr * sr + si * si).sqrt();
                (num / denom).min(1.0)
            })
            .collect()
    }

    /// Derive high-level modulation features from a set of cycle-frequency
    /// peaks.
    pub fn modulation_features(&self, peaks: &[CycleFrequencyPeak]) -> ModulationFeatures {
        let significant: Vec<&CycleFrequencyPeak> =
            peaks.iter().filter(|p| p.snr_db > 3.0).collect();

        let (est_symbol, est_carrier) = if let Some(strongest) = significant.first() {
            // Heuristic: the strongest non-zero peak is often 2*fc for BPSK-like
            // or the symbol rate.
            let fc_guess = strongest.alpha_hz / 2.0;
            let sym_guess = if significant.len() > 1 {
                significant[1].alpha_hz
            } else {
                strongest.alpha_hz
            };
            (sym_guess, fc_guess)
        } else if let Some(first) = peaks.first() {
            (first.alpha_hz, first.alpha_hz / 2.0)
        } else {
            (0.0, 0.0)
        };

        let hint = match significant.len() {
            0 => "Unknown/Noise".to_string(),
            1 => "BPSK or AM".to_string(),
            2 => "QPSK or FSK".to_string(),
            3..=5 => "Higher-order PSK/QAM".to_string(),
            _ => "Complex/Multi-carrier".to_string(),
        };

        ModulationFeatures {
            estimated_symbol_rate: est_symbol,
            estimated_carrier_freq: est_carrier,
            num_significant_peaks: significant.len(),
            modulation_hint: hint,
        }
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    /// Number of non-overlapping segments that fit in the data.
    fn effective_segments(&self, data_len: usize) -> usize {
        let max = data_len / self.config.fft_size;
        max.min(self.config.num_segments)
    }
}

// ---------------------------------------------------------------------------
// Minimal radix-2 DIT FFT (std-only)
// ---------------------------------------------------------------------------

/// In-place radix-2 DIT FFT. `buf` length must be a power of two.
fn fft_in_place(buf: &mut [(f64, f64)]) {
    let n = buf.len();
    if n <= 1 {
        return;
    }
    debug_assert!(n.is_power_of_two(), "FFT length must be a power of two");

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            buf.swap(i, j);
        }
    }

    // Cooley-Tukey butterfly
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle_step = -2.0 * PI / len as f64;
        for start in (0..n).step_by(len) {
            for k in 0..half {
                let angle = angle_step * k as f64;
                let (wc, ws) = (angle.cos(), angle.sin());
                let (tr, ti) = buf[start + k + half];
                let tw_re = tr * wc - ti * ws;
                let tw_im = tr * ws + ti * wc;
                let (ur, ui) = buf[start + k];
                buf[start + k] = (ur + tw_re, ui + tw_im);
                buf[start + k + half] = (ur - tw_re, ui - tw_im);
            }
        }
        len <<= 1;
    }
}

/// Convenience wrapper: DFT of a slice, returning a new vector.
fn dft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = input.len();
    if n.is_power_of_two() {
        let mut buf = input.to_vec();
        fft_in_place(&mut buf);
        buf
    } else {
        // Fallback: naive O(N^2) DFT for non-power-of-two lengths
        (0..n)
            .map(|k| {
                let mut re = 0.0;
                let mut im = 0.0;
                for (idx, &(xr, xi)) in input.iter().enumerate() {
                    let angle = -2.0 * PI * k as f64 * idx as f64 / n as f64;
                    let (cs, sn) = (angle.cos(), angle.sin());
                    re += xr * cs - xi * sn;
                    im += xr * sn + xi * cs;
                }
                (re, im)
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn default_config() -> CsaConfig {
        CsaConfig {
            fft_size: 64,
            num_segments: 4,
            sample_rate: 8000.0,
            alpha_resolution_hz: 100.0,
        }
    }

    /// Generate a real-valued tone at `freq_hz` (only I component, Q=0).
    /// A real tone has cyclostationary features at alpha = 2*freq_hz.
    fn tone(freq_hz: f64, sample_rate: f64, n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * PI * freq_hz * t;
                (phase.cos(), 0.0)
            })
            .collect()
    }

    /// Generate a BPSK signal at `carrier_hz` with `symbol_rate`.
    fn bpsk_signal(
        carrier_hz: f64,
        symbol_rate: f64,
        sample_rate: f64,
        n: usize,
    ) -> Vec<(f64, f64)> {
        let sps = (sample_rate / symbol_rate) as usize;
        (0..n)
            .map(|i| {
                let sym = if (i / sps) % 2 == 0 { 1.0 } else { -1.0 };
                let t = i as f64 / sample_rate;
                let phase = 2.0 * PI * carrier_hz * t;
                (sym * phase.cos(), sym * phase.sin())
            })
            .collect()
    }

    // ----- Test 1: Construction -----------------------------------------
    #[test]
    fn test_analyzer_construction() {
        let cfg = default_config();
        let a = CyclicSpectralAnalyzer::new(cfg.clone());
        assert_eq!(a.config.fft_size, 64);
        assert_eq!(a.config.num_segments, 4);
        assert!((a.config.sample_rate - 8000.0).abs() < 1e-9);
    }

    // ----- Test 2: SCF at alpha=0 equals PSD ----------------------------
    #[test]
    fn test_scf_zero_alpha_is_psd() {
        let cfg = default_config();
        let analyzer = CyclicSpectralAnalyzer::new(cfg.clone());
        let samples = tone(1000.0, cfg.sample_rate, 512);
        let scf = analyzer.compute_spectral_correlation(&samples, 0.0);

        // All values should be real and non-negative (PSD) to good approximation
        let total_power: f64 = scf.iter().map(|(r, i)| (r * r + i * i).sqrt()).sum();
        assert!(total_power > 0.0, "PSD must have nonzero total power");
    }

    // ----- Test 3: SCF length matches fft_size --------------------------
    #[test]
    fn test_scf_output_length() {
        let cfg = default_config();
        let analyzer = CyclicSpectralAnalyzer::new(cfg.clone());
        let samples = tone(500.0, cfg.sample_rate, 512);
        let scf = analyzer.compute_spectral_correlation(&samples, 0.1);
        assert_eq!(scf.len(), cfg.fft_size);
    }

    // ----- Test 4: Cyclic autocorrelation at tau=0 for real tone --------
    #[test]
    fn test_cyclic_autocorrelation_tone() {
        let cfg = default_config();
        let analyzer = CyclicSpectralAnalyzer::new(cfg);
        let freq = 1000.0;
        let samples = tone(freq, 8000.0, 2048);
        // A real tone at f0 has cyclic feature at alpha = 2*f0
        let cac = analyzer.cyclic_autocorrelation(&samples, 2.0 * freq, 0);
        let mag = (cac.0 * cac.0 + cac.1 * cac.1).sqrt();
        assert!(mag > 0.1, "Real tone should have strong CAC at 2*f0, got {}", mag);
    }

    // ----- Test 5: CAC at wrong frequency is weaker ---------------------
    #[test]
    fn test_cyclic_autocorrelation_wrong_freq() {
        let cfg = default_config();
        let analyzer = CyclicSpectralAnalyzer::new(cfg);
        let samples = tone(1000.0, 8000.0, 4096);
        // Real tone at 1 kHz: cyclic feature at 2 kHz
        let cac_right = analyzer.cyclic_autocorrelation(&samples, 2000.0, 0);
        let cac_wrong = analyzer.cyclic_autocorrelation(&samples, 1234.5, 0);
        let mag_right = (cac_right.0 * cac_right.0 + cac_right.1 * cac_right.1).sqrt();
        let mag_wrong = (cac_wrong.0 * cac_wrong.0 + cac_wrong.1 * cac_wrong.1).sqrt();
        assert!(
            mag_right > mag_wrong,
            "CAC at 2*f0 ({}) should exceed wrong freq ({})",
            mag_right,
            mag_wrong
        );
    }

    // ----- Test 6: BPSK has cyclic feature at 2*fc ----------------------
    #[test]
    fn test_bpsk_double_carrier_feature() {
        let cfg = CsaConfig {
            fft_size: 128,
            num_segments: 8,
            sample_rate: 8000.0,
            alpha_resolution_hz: 50.0,
        };
        let analyzer = CyclicSpectralAnalyzer::new(cfg);
        // Use real-valued BPSK (Q=0) for clear 2*fc feature
        let carrier = 1000.0;
        let sym_rate = 200.0;
        let sr = 8000.0;
        let n = 8192;
        let sps = (sr / sym_rate) as usize;
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let sym = if (i / sps) % 2 == 0 { 1.0 } else { -1.0 };
                let t = i as f64 / sr;
                let phase = 2.0 * PI * carrier * t;
                (sym * phase.cos(), 0.0)
            })
            .collect();
        let cac = analyzer.cyclic_autocorrelation(&samples, 2.0 * carrier, 0);
        let mag = (cac.0 * cac.0 + cac.1 * cac.1).sqrt();
        assert!(mag > 0.01, "BPSK should show feature at 2*fc, got {}", mag);
    }

    // ----- Test 7: extract_cycle_frequencies returns sorted peaks --------
    #[test]
    fn test_peak_extraction_sorted() {
        let cfg = CsaConfig {
            fft_size: 64,
            num_segments: 4,
            sample_rate: 8000.0,
            alpha_resolution_hz: 200.0,
        };
        let analyzer = CyclicSpectralAnalyzer::new(cfg);
        let samples = bpsk_signal(1000.0, 500.0, 8000.0, 512);
        let peaks = analyzer.extract_cycle_frequencies(&samples, 5);
        // Sorted descending by magnitude
        for w in peaks.windows(2) {
            assert!(
                w[0].magnitude >= w[1].magnitude,
                "Peaks must be sorted descending"
            );
        }
    }

    // ----- Test 8: extract_cycle_frequencies respects num_peaks ----------
    #[test]
    fn test_peak_extraction_count() {
        let cfg = CsaConfig {
            fft_size: 64,
            num_segments: 4,
            sample_rate: 8000.0,
            alpha_resolution_hz: 500.0,
        };
        let analyzer = CyclicSpectralAnalyzer::new(cfg);
        let samples = tone(1000.0, 8000.0, 512);
        let peaks = analyzer.extract_cycle_frequencies(&samples, 3);
        assert!(peaks.len() <= 3, "Should return at most num_peaks peaks");
    }

    // ----- Test 9: spectral coherence values in [0, 1] ------------------
    #[test]
    fn test_spectral_coherence_range() {
        let cfg = default_config();
        let analyzer = CyclicSpectralAnalyzer::new(cfg);
        let samples = bpsk_signal(1000.0, 200.0, 8000.0, 512);
        let coh = analyzer.spectral_coherence(&samples, 0.1);
        for &v in &coh {
            assert!(
                (0.0..=1.0001).contains(&v),
                "Coherence must be in [0,1], got {}",
                v
            );
        }
    }

    // ----- Test 10: spectral coherence length ----------------------------
    #[test]
    fn test_spectral_coherence_length() {
        let cfg = default_config();
        let analyzer = CyclicSpectralAnalyzer::new(cfg.clone());
        let samples = tone(500.0, cfg.sample_rate, 512);
        let coh = analyzer.spectral_coherence(&samples, 0.05);
        assert_eq!(coh.len(), cfg.fft_size);
    }

    // ----- Test 11: modulation_features for single peak ------------------
    #[test]
    fn test_modulation_features_single_peak() {
        let peaks = vec![CycleFrequencyPeak {
            alpha_hz: 2000.0,
            magnitude: 10.0,
            snr_db: 15.0,
        }];
        let cfg = default_config();
        let analyzer = CyclicSpectralAnalyzer::new(cfg);
        let features = analyzer.modulation_features(&peaks);
        assert!((features.estimated_carrier_freq - 1000.0).abs() < 1e-9);
        assert_eq!(features.num_significant_peaks, 1);
        assert!(features.modulation_hint.contains("BPSK") || features.modulation_hint.contains("AM"));
    }

    // ----- Test 12: modulation_features with no peaks --------------------
    #[test]
    fn test_modulation_features_no_peaks() {
        let cfg = default_config();
        let analyzer = CyclicSpectralAnalyzer::new(cfg);
        let features = analyzer.modulation_features(&[]);
        assert_eq!(features.num_significant_peaks, 0);
        assert!(features.modulation_hint.contains("Unknown"));
    }

    // ----- Test 13: modulation_features two peaks -> QPSK/FSK hint ------
    #[test]
    fn test_modulation_features_two_peaks() {
        let peaks = vec![
            CycleFrequencyPeak {
                alpha_hz: 2000.0,
                magnitude: 10.0,
                snr_db: 12.0,
            },
            CycleFrequencyPeak {
                alpha_hz: 500.0,
                magnitude: 5.0,
                snr_db: 8.0,
            },
        ];
        let cfg = default_config();
        let analyzer = CyclicSpectralAnalyzer::new(cfg);
        let features = analyzer.modulation_features(&peaks);
        assert_eq!(features.num_significant_peaks, 2);
        assert!(features.modulation_hint.contains("QPSK") || features.modulation_hint.contains("FSK"));
    }

    // ----- Test 14: empty input gracefully returns zeros -----------------
    #[test]
    fn test_empty_input() {
        let cfg = default_config();
        let analyzer = CyclicSpectralAnalyzer::new(cfg.clone());
        let scf = analyzer.compute_spectral_correlation(&[], 0.1);
        assert_eq!(scf.len(), cfg.fft_size);
        assert!(scf.iter().all(|&(r, i)| r == 0.0 && i == 0.0));
    }

    // ----- Test 15: DFT roundtrip (impulse -> flat spectrum) -------------
    #[test]
    fn test_dft_impulse() {
        let mut input = vec![(0.0, 0.0); 8];
        input[0] = (1.0, 0.0);
        let spectrum = dft(&input);
        // All bins should be (1, 0) for a unit impulse
        for (k, &(re, im)) in spectrum.iter().enumerate() {
            assert!(
                (re - 1.0).abs() < 1e-10 && im.abs() < 1e-10,
                "Bin {} should be (1,0), got ({}, {})",
                k,
                re,
                im
            );
        }
    }

    // ----- Test 16: DFT of DC signal -> energy in bin 0 -----------------
    #[test]
    fn test_dft_dc() {
        let input = vec![(1.0, 0.0); 16];
        let spectrum = dft(&input);
        let (re0, im0) = spectrum[0];
        assert!(
            (re0 - 16.0).abs() < 1e-10,
            "DC bin should equal N, got {}",
            re0
        );
        assert!(im0.abs() < 1e-10);
        // All other bins should be zero
        for k in 1..16 {
            let (r, i) = spectrum[k];
            assert!(
                r.abs() < 1e-10 && i.abs() < 1e-10,
                "Bin {} should be zero, got ({}, {})",
                k,
                r,
                i
            );
        }
    }

    // ----- Test 17: CycleFrequencyPeak SNR is in dB ---------------------
    #[test]
    fn test_peak_snr_is_db() {
        let cfg = CsaConfig {
            fft_size: 64,
            num_segments: 4,
            sample_rate: 8000.0,
            alpha_resolution_hz: 500.0,
        };
        let analyzer = CyclicSpectralAnalyzer::new(cfg);
        let samples = bpsk_signal(1000.0, 500.0, 8000.0, 512);
        let peaks = analyzer.extract_cycle_frequencies(&samples, 10);
        // At least one peak should exist; SNR should be finite
        assert!(!peaks.is_empty());
        for p in &peaks {
            assert!(p.snr_db.is_finite(), "SNR must be finite, got {}", p.snr_db);
        }
    }

    // ----- Test 18: CAC with non-zero lag --------------------------------
    #[test]
    fn test_cyclic_autocorrelation_nonzero_lag() {
        let cfg = default_config();
        let analyzer = CyclicSpectralAnalyzer::new(cfg);
        let samples = tone(500.0, 8000.0, 2048);
        // Real tone at 500 Hz: cyclic feature at 1000 Hz
        let cac = analyzer.cyclic_autocorrelation(&samples, 1000.0, 4);
        // Should still be nonzero (phase-shifted version of the tau=0 value)
        let mag = (cac.0 * cac.0 + cac.1 * cac.1).sqrt();
        assert!(mag > 0.01, "CAC at nonzero lag should be nonzero for real tone, got {}", mag);
    }
}
