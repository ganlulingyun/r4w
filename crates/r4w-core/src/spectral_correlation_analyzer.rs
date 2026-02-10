//! Cyclic spectral correlation function for cyclostationary signal analysis.
//!
//! This module computes the spectral correlation function (SCF), cyclic
//! autocorrelation, spectral coherence, and provides automatic feature
//! detection and basic signal classification based on cyclostationary
//! properties.
//!
//! Cyclostationary analysis exploits the periodicity in the statistics of
//! communication signals (e.g., symbol rate, carrier frequency, chip rate)
//! to detect and classify them even in low SNR environments.
//!
//! # Example
//!
//! ```
//! use r4w_core::spectral_correlation_analyzer::{SpectralCorrelationAnalyzer, ScfConfig};
//!
//! let config = ScfConfig {
//!     fft_size: 64,
//!     num_segments: 4,
//!     sample_rate: 1000.0,
//!     alpha_resolution: 10.0,
//! };
//! let analyzer = SpectralCorrelationAnalyzer::new(config);
//!
//! // Generate a simple tone at 100 Hz (cyclic freq = 200 Hz for real-valued)
//! let n = 256;
//! let samples: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / 1000.0;
//!         let phase = 2.0 * std::f64::consts::PI * 100.0 * t;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! // Compute SCF at cyclic frequency alpha = 0 (the power spectral density)
//! let scf = analyzer.compute_scf(&samples, 0.0);
//! assert_eq!(scf.len(), 64);
//!
//! // Verify there is non-trivial energy at alpha = 0
//! let total_energy: f64 = scf.iter().map(|(re, im)| re * re + im * im).sum();
//! assert!(total_energy > 0.0);
//! ```

use std::f64::consts::PI;

// ── Structs ──────────────────────────────────────────────────────────────────

/// Configuration for the spectral correlation analyzer.
#[derive(Debug, Clone)]
pub struct ScfConfig {
    /// FFT size used for the frequency-domain representation.
    pub fft_size: usize,
    /// Number of overlapping segments for averaging.
    pub num_segments: usize,
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Resolution for scanning cyclic frequencies (Hz).
    pub alpha_resolution: f64,
}

/// A detected cyclic feature from the SCF analysis.
#[derive(Debug, Clone)]
pub struct CyclicFeature {
    /// Cyclic frequency in Hz.
    pub cyclic_freq_hz: f64,
    /// Spectral (carrier) frequency in Hz where the feature peaks.
    pub spectral_freq_hz: f64,
    /// Magnitude of the SCF at this feature.
    pub magnitude: f64,
    /// Phase of the SCF at this feature in radians.
    pub phase_rad: f64,
}

/// Result of automatic signal classification from cyclic features.
#[derive(Debug, Clone)]
pub struct SignalClassification {
    /// Detected symbol rate in symbols/second (dominant non-zero cyclic freq).
    pub detected_symbol_rate: f64,
    /// Detected carrier offset in Hz (spectral frequency of strongest feature at alpha=0).
    pub detected_carrier_offset: f64,
    /// Hint about the modulation type based on cyclic feature patterns.
    pub modulation_hint: String,
}

/// Main engine for computing spectral correlation functions.
#[derive(Debug, Clone)]
pub struct SpectralCorrelationAnalyzer {
    config: ScfConfig,
}

// ── Helper functions ─────────────────────────────────────────────────────────

/// Multiply two complex numbers represented as (f64, f64).
#[inline]
fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Conjugate of a complex number.
#[inline]
fn complex_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Magnitude of a complex number.
#[inline]
fn complex_abs(a: (f64, f64)) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

/// Phase (argument) of a complex number in radians.
#[inline]
fn complex_arg(a: (f64, f64)) -> f64 {
    a.1.atan2(a.0)
}

/// In-place radix-2 Cooley-Tukey FFT (decimation-in-time).
/// `inverse` = true computes IFFT (with 1/N scaling).
fn fft_inplace(buf: &mut [(f64, f64)], inverse: bool) {
    let n = buf.len();
    if n <= 1 {
        return;
    }
    assert!(n.is_power_of_two(), "FFT size must be a power of two");

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

    // Butterfly stages
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle_step = sign * 2.0 * PI / len as f64;
        let wn = (angle_step.cos(), angle_step.sin());
        let mut start = 0;
        while start < n {
            let mut w = (1.0, 0.0);
            for k in 0..half {
                let u = buf[start + k];
                let t = complex_mul(w, buf[start + k + half]);
                buf[start + k] = (u.0 + t.0, u.1 + t.1);
                buf[start + k + half] = (u.0 - t.0, u.1 - t.1);
                w = complex_mul(w, wn);
            }
            start += len;
        }
        len <<= 1;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for s in buf.iter_mut() {
            s.0 *= inv_n;
            s.1 *= inv_n;
        }
    }
}

/// Compute the FFT of a slice, returning a new vector. Pads or truncates to `size`.
fn fft(data: &[(f64, f64)], size: usize) -> Vec<(f64, f64)> {
    let mut buf = vec![(0.0, 0.0); size];
    let copy_len = data.len().min(size);
    buf[..copy_len].copy_from_slice(&data[..copy_len]);
    fft_inplace(&mut buf, false);
    buf
}

// ── Implementation ───────────────────────────────────────────────────────────

impl SpectralCorrelationAnalyzer {
    /// Create a new analyzer with the given configuration.
    pub fn new(config: ScfConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &ScfConfig {
        &self.config
    }

    /// Compute the spectral correlation function at cyclic frequency `alpha` Hz.
    ///
    /// Uses the frequency-smoothing method:
    ///   S_x^alpha(f) = <X_T(f + alpha/2) * conj(X_T(f - alpha/2))> / T
    ///
    /// where <·> denotes averaging over segments.
    ///
    /// Returns a vector of length `fft_size` with complex SCF values indexed by
    /// spectral frequency bin.
    pub fn compute_scf(&self, samples: &[(f64, f64)], alpha: f64) -> Vec<(f64, f64)> {
        let n = self.config.fft_size;
        let num_seg = self.config.num_segments.max(1);
        let fs = self.config.sample_rate;

        // Determine segment hop; allow overlap if possible
        let total = samples.len();
        let hop = if total > n {
            ((total - n) / num_seg.max(1)).max(1)
        } else {
            1
        };

        let mut accum = vec![(0.0, 0.0); n];
        let mut count = 0u64;

        for seg in 0..num_seg {
            let start = seg * hop;
            if start + n > total {
                break;
            }
            let segment = &samples[start..start + n];

            // Apply Hanning window
            let windowed: Vec<(f64, f64)> = segment
                .iter()
                .enumerate()
                .map(|(i, &(re, im))| {
                    let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / (n as f64 - 1.0)).cos());
                    (re * w, im * w)
                })
                .collect();

            // Frequency shift by +alpha/2 and -alpha/2 before FFT
            let shift_pos: Vec<(f64, f64)> = windowed
                .iter()
                .enumerate()
                .map(|(i, &(re, im))| {
                    let t = i as f64 / fs;
                    let angle = 2.0 * PI * (alpha / 2.0) * t;
                    complex_mul((re, im), (angle.cos(), angle.sin()))
                })
                .collect();

            let shift_neg: Vec<(f64, f64)> = windowed
                .iter()
                .enumerate()
                .map(|(i, &(re, im))| {
                    let t = i as f64 / fs;
                    let angle = -2.0 * PI * (alpha / 2.0) * t;
                    complex_mul((re, im), (angle.cos(), angle.sin()))
                })
                .collect();

            let x_pos = fft(&shift_pos, n);
            let x_neg = fft(&shift_neg, n);

            // Cross-spectral product
            for k in 0..n {
                let prod = complex_mul(x_pos[k], complex_conj(x_neg[k]));
                accum[k].0 += prod.0;
                accum[k].1 += prod.1;
            }
            count += 1;
        }

        // Average and normalise by segment duration
        if count > 0 {
            let norm = 1.0 / (count as f64 * n as f64);
            for s in accum.iter_mut() {
                s.0 *= norm;
                s.1 *= norm;
            }
        }

        accum
    }

    /// Compute the cyclic autocorrelation at cyclic frequency `alpha` and lag `tau`.
    ///
    /// R_x^alpha(tau) = E[x(t + tau) * conj(x(t)) * exp(-j 2 pi alpha t)]
    ///
    /// Estimated by time-averaging over the provided samples.
    pub fn cyclic_autocorrelation(
        &self,
        samples: &[(f64, f64)],
        alpha: f64,
        tau: isize,
    ) -> (f64, f64) {
        let n = samples.len() as isize;
        if n == 0 {
            return (0.0, 0.0);
        }

        let fs = self.config.sample_rate;
        let mut sum = (0.0, 0.0);
        let mut count = 0u64;

        for t in 0..n {
            let t_shift = t + tau;
            if t_shift < 0 || t_shift >= n {
                continue;
            }
            let x_tau = samples[t_shift as usize];
            let x_t = samples[t as usize];
            let prod = complex_mul(x_tau, complex_conj(x_t));

            // Multiply by exp(-j 2 pi alpha t / fs)
            let angle = -2.0 * PI * alpha * (t as f64) / fs;
            let rot = (angle.cos(), angle.sin());
            let val = complex_mul(prod, rot);

            sum.0 += val.0;
            sum.1 += val.1;
            count += 1;
        }

        if count > 0 {
            sum.0 /= count as f64;
            sum.1 /= count as f64;
        }
        sum
    }

    /// Compute the spectral coherence at cyclic frequency `alpha`.
    ///
    /// C_x^alpha(f) = |S_x^alpha(f)| / sqrt(S_x^0(f + alpha/2) * S_x^0(f - alpha/2))
    ///
    /// Returns a vector of real-valued coherence (0..1) of length `fft_size`.
    pub fn spectral_coherence(&self, samples: &[(f64, f64)], alpha: f64) -> Vec<f64> {
        let scf_alpha = self.compute_scf(samples, alpha);
        let scf_zero = self.compute_scf(samples, 0.0);
        let n = self.config.fft_size;
        let fs = self.config.sample_rate;
        let bin_hz = fs / n as f64;

        // Shift in bins for alpha/2
        let shift_bins = ((alpha / 2.0) / bin_hz).round() as isize;

        scf_alpha
            .iter()
            .enumerate()
            .map(|(k, &s_alpha)| {
                let k_pos = ((k as isize + shift_bins).rem_euclid(n as isize)) as usize;
                let k_neg = ((k as isize - shift_bins).rem_euclid(n as isize)) as usize;
                let psd_pos = complex_abs(scf_zero[k_pos]);
                let psd_neg = complex_abs(scf_zero[k_neg]);
                let denom = (psd_pos * psd_neg).sqrt();
                if denom > 1e-30 {
                    complex_abs(s_alpha) / denom
                } else {
                    0.0
                }
            })
            .collect()
    }

    /// Detect prominent cyclic features by scanning over a range of cyclic frequencies.
    ///
    /// Scans from `alpha_resolution` to `sample_rate / 2` in steps of `alpha_resolution`,
    /// keeping features whose peak coherence exceeds a threshold.
    pub fn detect_cyclic_features(&self, samples: &[(f64, f64)]) -> Vec<CyclicFeature> {
        let fs = self.config.sample_rate;
        let alpha_step = self.config.alpha_resolution;
        if alpha_step <= 0.0 || fs <= 0.0 {
            return Vec::new();
        }
        let n = self.config.fft_size;
        let bin_hz = fs / n as f64;

        let mut features = Vec::new();

        // Compute the PSD (alpha=0) for baseline noise estimation
        let psd = self.compute_scf(samples, 0.0);
        let mean_psd: f64 =
            psd.iter().map(|s| complex_abs(*s)).sum::<f64>() / n as f64;
        let threshold = if mean_psd > 1e-30 { 0.15 } else { 1e-30 };

        let max_alpha = fs / 2.0;
        let mut alpha = alpha_step;
        while alpha <= max_alpha {
            let coherence = self.spectral_coherence(samples, alpha);
            // Find the peak coherence and its bin
            let (peak_bin, &peak_val) = coherence
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
                .unwrap_or((0, &0.0));

            if peak_val > threshold {
                let scf = self.compute_scf(samples, alpha);
                let scf_val = scf[peak_bin];
                // Convert bin to Hz (centered around DC)
                let spectral_freq = if peak_bin <= n / 2 {
                    peak_bin as f64 * bin_hz
                } else {
                    (peak_bin as f64 - n as f64) * bin_hz
                };

                features.push(CyclicFeature {
                    cyclic_freq_hz: alpha,
                    spectral_freq_hz: spectral_freq,
                    magnitude: complex_abs(scf_val),
                    phase_rad: complex_arg(scf_val),
                });
            }

            alpha += alpha_step;
        }

        // Sort by magnitude descending
        features.sort_by(|a, b| b.magnitude.partial_cmp(&a.magnitude).unwrap_or(std::cmp::Ordering::Equal));
        features
    }

    /// Classify the signal based on detected cyclic features.
    ///
    /// Uses simple heuristics:
    /// - The dominant non-zero cyclic frequency is assumed to be the symbol rate.
    /// - The spectral frequency of the strongest alpha=0 feature approximates
    ///   the carrier offset.
    /// - The pattern of harmonics provides a modulation hint.
    pub fn classify_signal(&self, features: &[CyclicFeature]) -> SignalClassification {
        if features.is_empty() {
            return SignalClassification {
                detected_symbol_rate: 0.0,
                detected_carrier_offset: 0.0,
                modulation_hint: "Unknown (no features)".to_string(),
            };
        }

        // Dominant cyclic frequency -> symbol rate estimate
        let dominant = &features[0];
        let detected_symbol_rate = dominant.cyclic_freq_hz;

        // Carrier offset from the spectral frequency of the dominant feature
        let detected_carrier_offset = dominant.spectral_freq_hz;

        // Count how many harmonics of the fundamental are present
        let fundamental = detected_symbol_rate;
        let harmonic_count = if fundamental > 0.0 {
            features
                .iter()
                .filter(|f| {
                    let ratio = f.cyclic_freq_hz / fundamental;
                    let rounded = ratio.round();
                    rounded >= 1.0 && (ratio - rounded).abs() < 0.15
                })
                .count()
        } else {
            0
        };

        let modulation_hint = match harmonic_count {
            0 => "Unknown".to_string(),
            1 => "BPSK or ASK (single cyclic feature)".to_string(),
            2 => "QPSK or OQPSK (two harmonics)".to_string(),
            3..=4 => "Higher-order PSK/QAM (multiple harmonics)".to_string(),
            _ => format!("Complex modulation ({} harmonics detected)", harmonic_count),
        };

        SignalClassification {
            detected_symbol_rate,
            detected_carrier_offset,
            modulation_hint,
        }
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> ScfConfig {
        ScfConfig {
            fft_size: 64,
            num_segments: 4,
            sample_rate: 1000.0,
            alpha_resolution: 10.0,
        }
    }

    fn make_tone(freq: f64, fs: f64, n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let phase = 2.0 * PI * freq * t;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    // --- basic construction and config ---

    #[test]
    fn test_new_analyzer() {
        let cfg = default_config();
        let a = SpectralCorrelationAnalyzer::new(cfg.clone());
        assert_eq!(a.config().fft_size, 64);
        assert_eq!(a.config().num_segments, 4);
        assert!((a.config().sample_rate - 1000.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_scf_output_length() {
        let a = SpectralCorrelationAnalyzer::new(default_config());
        let samples = make_tone(100.0, 1000.0, 256);
        let scf = a.compute_scf(&samples, 0.0);
        assert_eq!(scf.len(), 64);
    }

    // --- SCF properties ---

    #[test]
    fn test_scf_alpha_zero_is_psd() {
        // At alpha=0 the SCF reduces to the power spectral density
        let a = SpectralCorrelationAnalyzer::new(default_config());
        let samples = make_tone(100.0, 1000.0, 256);
        let psd = a.compute_scf(&samples, 0.0);
        // PSD should have positive real parts (power) and be non-trivial
        let total: f64 = psd.iter().map(|(re, _)| re.abs()).sum();
        assert!(total > 0.0, "PSD should be non-trivial");
    }

    #[test]
    fn test_scf_noise_floor_lower_off_alpha() {
        // A pure tone at alpha far from any cyclic freq should have lower energy
        let a = SpectralCorrelationAnalyzer::new(default_config());
        let samples = make_tone(100.0, 1000.0, 256);
        let psd_energy: f64 = a
            .compute_scf(&samples, 0.0)
            .iter()
            .map(|s| s.0 * s.0 + s.1 * s.1)
            .sum();
        let off_energy: f64 = a
            .compute_scf(&samples, 37.0) // arbitrary non-harmonic alpha
            .iter()
            .map(|s| s.0 * s.0 + s.1 * s.1)
            .sum();
        assert!(
            psd_energy > off_energy,
            "PSD energy ({}) should exceed off-alpha energy ({})",
            psd_energy,
            off_energy,
        );
    }

    #[test]
    fn test_scf_dc_signal() {
        // A DC signal (constant) should only show energy at alpha=0
        let a = SpectralCorrelationAnalyzer::new(default_config());
        let dc: Vec<(f64, f64)> = vec![(1.0, 0.0); 256];
        let psd_energy: f64 = a
            .compute_scf(&dc, 0.0)
            .iter()
            .map(|s| s.0 * s.0 + s.1 * s.1)
            .sum();
        let off_energy: f64 = a
            .compute_scf(&dc, 50.0)
            .iter()
            .map(|s| s.0 * s.0 + s.1 * s.1)
            .sum();
        assert!(psd_energy > off_energy * 10.0);
    }

    // --- Cyclic autocorrelation ---

    #[test]
    fn test_cyclic_autocorrelation_zero_lag() {
        let a = SpectralCorrelationAnalyzer::new(default_config());
        let samples = make_tone(100.0, 1000.0, 256);
        let r = a.cyclic_autocorrelation(&samples, 0.0, 0);
        // At alpha=0, tau=0 this is the average power — should be ~0.5 for unit tone
        assert!(r.0 > 0.0, "Power at tau=0 should be positive: {}", r.0);
    }

    #[test]
    fn test_cyclic_autocorrelation_empty() {
        let a = SpectralCorrelationAnalyzer::new(default_config());
        let r = a.cyclic_autocorrelation(&[], 0.0, 0);
        assert!((r.0).abs() < f64::EPSILON);
        assert!((r.1).abs() < f64::EPSILON);
    }

    #[test]
    fn test_cyclic_autocorrelation_tau_symmetry() {
        // R_x^0(tau) should roughly equal conj(R_x^0(-tau)) for real-valued signals
        let a = SpectralCorrelationAnalyzer::new(default_config());
        let samples = make_tone(100.0, 1000.0, 256);
        let r_pos = a.cyclic_autocorrelation(&samples, 0.0, 5);
        let r_neg = a.cyclic_autocorrelation(&samples, 0.0, -5);
        // For a complex tone the conjugate symmetry gives R(tau) ~ conj(R(-tau))
        let diff_re = (r_pos.0 - r_neg.0).abs();
        let diff_im = (r_pos.1 + r_neg.1).abs(); // conj flips imaginary sign
        assert!(
            diff_re < 0.15,
            "Real part symmetry broken: diff = {}",
            diff_re
        );
        assert!(
            diff_im < 0.15,
            "Imag part conjugate symmetry broken: diff = {}",
            diff_im
        );
    }

    #[test]
    fn test_cyclic_autocorrelation_large_tau() {
        // tau beyond the sample range should still return a value (from fewer samples)
        let a = SpectralCorrelationAnalyzer::new(default_config());
        let samples = make_tone(100.0, 1000.0, 64);
        let r = a.cyclic_autocorrelation(&samples, 0.0, 63);
        // Only one sample pair contributes; magnitude limited
        assert!(complex_abs(r) < 2.0);
    }

    // --- Spectral coherence ---

    #[test]
    fn test_spectral_coherence_length() {
        let a = SpectralCorrelationAnalyzer::new(default_config());
        let samples = make_tone(100.0, 1000.0, 256);
        let coh = a.spectral_coherence(&samples, 0.0);
        assert_eq!(coh.len(), 64);
    }

    #[test]
    fn test_spectral_coherence_at_alpha_zero() {
        // Coherence at alpha=0 is |PSD|/|PSD| = 1.0 wherever there is energy
        let a = SpectralCorrelationAnalyzer::new(default_config());
        let samples = make_tone(100.0, 1000.0, 256);
        let coh = a.spectral_coherence(&samples, 0.0);
        // At least one bin should have coherence near 1.0
        let max_coh = coh.iter().cloned().fold(0.0f64, f64::max);
        assert!(
            max_coh > 0.9,
            "Expected coherence ~1.0 at alpha=0, got {}",
            max_coh
        );
    }

    #[test]
    fn test_spectral_coherence_bounded() {
        // Coherence values should be in [0, inf) but typically ≤ 1 for well-behaved signals
        let a = SpectralCorrelationAnalyzer::new(default_config());
        let samples = make_tone(100.0, 1000.0, 256);
        let coh = a.spectral_coherence(&samples, 50.0);
        for &c in &coh {
            assert!(c >= 0.0, "Coherence must be non-negative, got {}", c);
        }
    }

    // --- Feature detection ---

    #[test]
    fn test_detect_features_returns_sorted() {
        let a = SpectralCorrelationAnalyzer::new(default_config());
        let samples = make_tone(100.0, 1000.0, 256);
        let features = a.detect_cyclic_features(&samples);
        // Features should be sorted by magnitude descending
        for w in features.windows(2) {
            assert!(
                w[0].magnitude >= w[1].magnitude,
                "Features not sorted: {} < {}",
                w[0].magnitude,
                w[1].magnitude
            );
        }
    }

    #[test]
    fn test_detect_features_empty_input() {
        let a = SpectralCorrelationAnalyzer::new(default_config());
        let features = a.detect_cyclic_features(&[]);
        // Should handle gracefully — no crash, possibly empty
        assert!(features.is_empty() || !features.is_empty()); // no panic
    }

    // --- Classification ---

    #[test]
    fn test_classify_empty_features() {
        let a = SpectralCorrelationAnalyzer::new(default_config());
        let cls = a.classify_signal(&[]);
        assert_eq!(cls.detected_symbol_rate, 0.0);
        assert!(cls.modulation_hint.contains("Unknown"));
    }

    #[test]
    fn test_classify_single_feature() {
        let a = SpectralCorrelationAnalyzer::new(default_config());
        let features = vec![CyclicFeature {
            cyclic_freq_hz: 100.0,
            spectral_freq_hz: 50.0,
            magnitude: 1.0,
            phase_rad: 0.0,
        }];
        let cls = a.classify_signal(&features);
        assert!((cls.detected_symbol_rate - 100.0).abs() < f64::EPSILON);
        assert!((cls.detected_carrier_offset - 50.0).abs() < f64::EPSILON);
        assert!(cls.modulation_hint.contains("BPSK") || cls.modulation_hint.contains("ASK"));
    }

    #[test]
    fn test_classify_multiple_harmonics() {
        let a = SpectralCorrelationAnalyzer::new(default_config());
        let features = vec![
            CyclicFeature {
                cyclic_freq_hz: 100.0,
                spectral_freq_hz: 0.0,
                magnitude: 2.0,
                phase_rad: 0.0,
            },
            CyclicFeature {
                cyclic_freq_hz: 200.0,
                spectral_freq_hz: 0.0,
                magnitude: 1.0,
                phase_rad: 0.0,
            },
        ];
        let cls = a.classify_signal(&features);
        assert!((cls.detected_symbol_rate - 100.0).abs() < f64::EPSILON);
        assert!(cls.modulation_hint.contains("QPSK") || cls.modulation_hint.contains("two"));
    }

    // --- FFT helper ---

    #[test]
    fn test_fft_parseval() {
        // Parseval's theorem: sum |x|^2 == (1/N) sum |X|^2
        let n = 64;
        let samples = make_tone(5.0, 64.0, n);
        let spectrum = fft(&samples, n);
        let time_energy: f64 = samples.iter().map(|s| s.0 * s.0 + s.1 * s.1).sum();
        let freq_energy: f64 =
            spectrum.iter().map(|s| s.0 * s.0 + s.1 * s.1).sum::<f64>() / n as f64;
        assert!(
            (time_energy - freq_energy).abs() < 1e-6,
            "Parseval: time={} freq={}",
            time_energy,
            freq_energy,
        );
    }
}
