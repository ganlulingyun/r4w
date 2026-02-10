//! Phase coherence analysis for MIMO and phased array calibration.
//!
//! This module measures and monitors phase relationships between signals,
//! providing metrics essential for MIMO system calibration, phased array
//! beamforming, and oscillator stability assessment.
//!
//! # Key Capabilities
//!
//! - **Phase Locking Value (PLV)**: Quantifies synchronization between two signals (0 = uncorrelated, 1 = perfectly locked).
//! - **Coherence Metrics**: Comprehensive phase relationship analysis including mean phase difference and variance.
//! - **Phase Noise Profiling**: SSB phase noise L(f) estimation from baseband signals.
//! - **Allan Variance**: Oscillator stability characterization over averaging intervals.
//! - **Cross-Spectral Density**: Frequency-domain coherence estimation between signal pairs.
//!
//! # Example
//!
//! ```
//! use r4w_core::phase_coherence_analyzer::{PhaseCoherenceAnalyzer, CoherenceMetrics};
//!
//! let analyzer = PhaseCoherenceAnalyzer::new(1000.0);
//!
//! // Two perfectly phase-locked signals (same frequency, constant phase offset)
//! let n = 256;
//! let sig_a: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / 1000.0;
//!         let phase = 2.0 * std::f64::consts::PI * 50.0 * t;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//! let sig_b: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / 1000.0;
//!         let phase = 2.0 * std::f64::consts::PI * 50.0 * t + 0.3;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! let plv = analyzer.phase_locking_value(&sig_a, &sig_b);
//! assert!(plv > 0.95, "Phase-locked signals should have PLV near 1.0, got {}", plv);
//!
//! let metrics = analyzer.coherence(&sig_a, &sig_b);
//! assert!(metrics.phase_locking_value > 0.95);
//! assert!(metrics.phase_variance < 0.05);
//! ```

use std::f64::consts::PI;

/// Result of coherence analysis between two signals.
#[derive(Debug, Clone)]
pub struct CoherenceMetrics {
    /// Phase Locking Value in [0, 1]. 1 = perfectly phase-locked.
    pub phase_locking_value: f64,
    /// Mean phase difference in radians, wrapped to [-pi, pi].
    pub mean_phase_diff_rad: f64,
    /// Variance of the instantaneous phase difference (radians^2).
    pub phase_variance: f64,
    /// Magnitude of the mean complex coherence vector.
    pub coherence_magnitude: f64,
}

/// SSB phase noise profile of a signal relative to a carrier.
#[derive(Debug, Clone)]
pub struct PhaseNoiseProfile {
    /// Pairs of (offset_frequency_hz, ssb_phase_noise_dBc_Hz).
    pub offset_freq_hz: Vec<f64>,
    /// SSB phase noise L(f) values in dBc/Hz corresponding to each offset frequency.
    pub ssb_phase_noise_dbc_hz: Vec<f64>,
    /// Integrated RMS phase jitter in radians over the measured offset range.
    pub integrated_jitter_rms_rad: f64,
}

/// Phase coherence analysis engine for MIMO / phased-array calibration.
///
/// Operates on IQ sample pairs represented as `(f64, f64)` tuples where the
/// first element is the in-phase (I) component and the second is the
/// quadrature (Q) component.
#[derive(Debug, Clone)]
pub struct PhaseCoherenceAnalyzer {
    sample_rate: f64,
}

impl PhaseCoherenceAnalyzer {
    /// Create a new analyzer with the given sample rate in Hz.
    ///
    /// # Panics
    /// Panics if `sample_rate` is not positive and finite.
    pub fn new(sample_rate: f64) -> Self {
        assert!(
            sample_rate > 0.0 && sample_rate.is_finite(),
            "sample_rate must be positive and finite"
        );
        Self { sample_rate }
    }

    /// Return the configured sample rate in Hz.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Compute the Phase Locking Value (PLV) between two signals.
    ///
    /// PLV = |mean(exp(j * (phase_a - phase_b)))|
    ///
    /// Returns a value in [0, 1]:
    /// - 1.0 means the phase difference is constant (perfectly locked).
    /// - 0.0 means the phase difference is uniformly distributed (no locking).
    ///
    /// # Panics
    /// Panics if the signals are empty or have different lengths.
    pub fn phase_locking_value(&self, sig_a: &[(f64, f64)], sig_b: &[(f64, f64)]) -> f64 {
        assert!(!sig_a.is_empty(), "sig_a must not be empty");
        assert_eq!(sig_a.len(), sig_b.len(), "signals must have the same length");

        let n = sig_a.len() as f64;
        let (sum_re, sum_im) = sig_a.iter().zip(sig_b.iter()).fold(
            (0.0, 0.0),
            |(acc_re, acc_im), (&(ai, aq), &(bi, bq))| {
                // Phase difference via conjugate multiplication: a * conj(b)
                let re = ai * bi + aq * bq;
                let im = aq * bi - ai * bq;
                // Normalize to unit magnitude to extract pure phase
                let mag = (re * re + im * im).sqrt();
                if mag < 1e-30 {
                    (acc_re, acc_im)
                } else {
                    (acc_re + re / mag, acc_im + im / mag)
                }
            },
        );

        let mean_re = sum_re / n;
        let mean_im = sum_im / n;
        (mean_re * mean_re + mean_im * mean_im).sqrt()
    }

    /// Compute comprehensive coherence metrics between two signals.
    ///
    /// Returns [`CoherenceMetrics`] with PLV, mean phase difference,
    /// phase variance, and coherence magnitude.
    ///
    /// # Panics
    /// Panics if the signals are empty or have different lengths.
    pub fn coherence(
        &self,
        sig_a: &[(f64, f64)],
        sig_b: &[(f64, f64)],
    ) -> CoherenceMetrics {
        assert!(!sig_a.is_empty(), "sig_a must not be empty");
        assert_eq!(sig_a.len(), sig_b.len(), "signals must have the same length");

        let n = sig_a.len() as f64;

        // Collect instantaneous phase differences
        let mut phase_diffs: Vec<f64> = Vec::with_capacity(sig_a.len());
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;

        for (&(ai, aq), &(bi, bq)) in sig_a.iter().zip(sig_b.iter()) {
            // a * conj(b)
            let re = ai * bi + aq * bq;
            let im = aq * bi - ai * bq;
            let phase_diff = im.atan2(re);
            phase_diffs.push(phase_diff);

            // Unit-magnitude accumulator for PLV
            let mag = (re * re + im * im).sqrt();
            if mag > 1e-30 {
                sum_re += re / mag;
                sum_im += im / mag;
            }
        }

        // PLV
        let mean_re = sum_re / n;
        let mean_im = sum_im / n;
        let plv = (mean_re * mean_re + mean_im * mean_im).sqrt();

        // Mean phase difference (circular mean)
        let circ_sum_re: f64 = phase_diffs.iter().map(|&p| p.cos()).sum();
        let circ_sum_im: f64 = phase_diffs.iter().map(|&p| p.sin()).sum();
        let mean_phase = circ_sum_im.atan2(circ_sum_re);

        // Circular variance: 1 - R, where R = |mean(exp(j*theta))|
        let r = ((circ_sum_re / n).powi(2) + (circ_sum_im / n).powi(2)).sqrt();
        let phase_variance = 1.0 - r;

        // Coherence magnitude from the raw (non-normalized) cross product
        let raw_sum_re: f64 = sig_a
            .iter()
            .zip(sig_b.iter())
            .map(|(&(ai, aq), &(bi, bq))| ai * bi + aq * bq)
            .sum();
        let raw_sum_im: f64 = sig_a
            .iter()
            .zip(sig_b.iter())
            .map(|(&(ai, aq), &(bi, bq))| aq * bi - ai * bq)
            .sum();
        let power_a: f64 = sig_a.iter().map(|&(i, q)| i * i + q * q).sum();
        let power_b: f64 = sig_b.iter().map(|&(i, q)| i * i + q * q).sum();
        let denom = (power_a * power_b).sqrt();
        let coherence_magnitude = if denom > 1e-30 {
            ((raw_sum_re / n).powi(2) + (raw_sum_im / n).powi(2)).sqrt()
                / (denom / n)
        } else {
            0.0
        };

        CoherenceMetrics {
            phase_locking_value: plv,
            mean_phase_diff_rad: mean_phase,
            phase_variance,
            coherence_magnitude,
        }
    }

    /// Estimate the SSB phase noise profile of a signal at the given carrier frequency.
    ///
    /// The signal is mixed down to baseband by multiplying with a complex
    /// exponential at `-carrier_freq`, then the power spectral density of the
    /// instantaneous phase is computed via FFT.
    ///
    /// # Panics
    /// Panics if `signal` has fewer than 4 samples or `carrier_freq` is negative.
    pub fn phase_noise(
        &self,
        signal: &[(f64, f64)],
        carrier_freq: f64,
    ) -> PhaseNoiseProfile {
        assert!(signal.len() >= 4, "need at least 4 samples for phase noise estimation");
        assert!(carrier_freq >= 0.0, "carrier_freq must be non-negative");

        let n = signal.len();

        // Mix down to baseband
        let baseband: Vec<(f64, f64)> = signal
            .iter()
            .enumerate()
            .map(|(k, &(si, sq))| {
                let t = k as f64 / self.sample_rate;
                let angle = -2.0 * PI * carrier_freq * t;
                let (sin_a, cos_a) = angle.sin_cos();
                // (s) * exp(j*angle) = (si + j*sq) * (cos_a + j*sin_a)
                (si * cos_a - sq * sin_a, si * sin_a + sq * cos_a)
            })
            .collect();

        // Extract instantaneous phase
        let phases: Vec<f64> = baseband.iter().map(|&(i, q)| q.atan2(i)).collect();

        // Remove mean phase (DC offset)
        let mean_phase: f64 = phases.iter().sum::<f64>() / n as f64;
        let centered: Vec<f64> = phases.iter().map(|&p| p - mean_phase).collect();

        // Compute PSD via FFT (power of DFT of phase deviation)
        let fft_len = n;
        let half = fft_len / 2;

        // DFT of centered phase
        let mut psd_re = vec![0.0; fft_len];
        let mut psd_im = vec![0.0; fft_len];
        for k in 0..fft_len {
            for (j, &c) in centered.iter().enumerate() {
                let angle = -2.0 * PI * k as f64 * j as f64 / fft_len as f64;
                psd_re[k] += c * angle.cos();
                psd_im[k] += c * angle.sin();
            }
        }

        // Power spectral density (single-sided, in rad^2/Hz)
        let freq_res = self.sample_rate / fft_len as f64;
        let norm = 1.0 / (fft_len as f64 * self.sample_rate);

        let mut offset_freqs = Vec::with_capacity(half);
        let mut ssb_noise = Vec::with_capacity(half);

        for k in 1..=half {
            let freq = k as f64 * freq_res;
            let power = (psd_re[k] * psd_re[k] + psd_im[k] * psd_im[k]) * norm * 2.0;
            let l_f = if power > 1e-30 {
                10.0 * power.log10()
            } else {
                -300.0 // floor
            };
            offset_freqs.push(freq);
            ssb_noise.push(l_f);
        }

        // Integrated jitter: sqrt(2 * integral of L(f) df) where L(f) is in linear
        let mut integrated_power = 0.0;
        for k in 1..=half {
            let power = (psd_re[k] * psd_re[k] + psd_im[k] * psd_im[k]) * norm * 2.0;
            integrated_power += power * freq_res;
        }
        let integrated_jitter_rms_rad = (2.0 * integrated_power).sqrt();

        PhaseNoiseProfile {
            offset_freq_hz: offset_freqs,
            ssb_phase_noise_dbc_hz: ssb_noise,
            integrated_jitter_rms_rad,
        }
    }

    /// Compute Allan variance for a sequence of phase samples at the given
    /// averaging interval `tau` (in seconds).
    ///
    /// Allan variance: sigma^2(tau) = 0.5 * mean((x_{n+1} - x_n)^2)
    /// where x_n are phase samples spaced by `tau`.
    ///
    /// The phase samples should represent accumulated phase (not wrapped).
    ///
    /// # Panics
    /// Panics if `tau` is not positive, or if there are fewer than 2 usable samples
    /// at the given tau spacing.
    pub fn allan_variance(&self, phase_samples: &[f64], tau: f64) -> f64 {
        assert!(tau > 0.0, "tau must be positive");

        let dt = 1.0 / self.sample_rate;
        let m = (tau / dt).round() as usize;
        let m = if m < 1 { 1 } else { m };

        assert!(
            phase_samples.len() > 2 * m,
            "not enough samples for Allan variance at tau={}: need > {}, have {}",
            tau,
            2 * m,
            phase_samples.len()
        );

        // Fractional frequency: y_k = (x_{k+m} - x_k) / tau
        let n_y = phase_samples.len() - m;
        let y: Vec<f64> = (0..n_y)
            .map(|k| (phase_samples[k + m] - phase_samples[k]) / (m as f64 * dt))
            .collect();

        // Allan variance: 0.5 * mean((y_{k+1} - y_k)^2)
        let n_pairs = y.len() - 1;
        assert!(n_pairs > 0, "not enough fractional frequency samples");

        let sum_sq: f64 = y.windows(2).map(|w| (w[1] - w[0]).powi(2)).sum();

        0.5 * sum_sq / n_pairs as f64
    }

    /// Compute the cross-spectral density of two signals via DFT.
    ///
    /// Returns a vector of complex-valued cross-spectrum samples as `(re, im)` pairs.
    /// The length equals the length of the shorter input signal.
    ///
    /// S_ab(f) = conj(DFT(a)) * DFT(b)
    ///
    /// # Panics
    /// Panics if either signal is empty.
    pub fn cross_spectrum(
        &self,
        sig_a: &[(f64, f64)],
        sig_b: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        assert!(!sig_a.is_empty(), "sig_a must not be empty");
        assert!(!sig_b.is_empty(), "sig_b must not be empty");

        let n = sig_a.len().min(sig_b.len());

        // DFT of sig_a
        let mut a_re = vec![0.0; n];
        let mut a_im = vec![0.0; n];
        for k in 0..n {
            for j in 0..n {
                let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
                let (sin_a, cos_a) = angle.sin_cos();
                a_re[k] += sig_a[j].0 * cos_a - sig_a[j].1 * sin_a;
                a_im[k] += sig_a[j].0 * sin_a + sig_a[j].1 * cos_a;
            }
        }

        // DFT of sig_b
        let mut b_re = vec![0.0; n];
        let mut b_im = vec![0.0; n];
        for k in 0..n {
            for j in 0..n {
                let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
                let (sin_a, cos_a) = angle.sin_cos();
                b_re[k] += sig_b[j].0 * cos_a - sig_b[j].1 * sin_a;
                b_im[k] += sig_b[j].0 * sin_a + sig_b[j].1 * cos_a;
            }
        }

        // Cross-spectrum: conj(A) * B
        (0..n)
            .map(|k| {
                // conj(A) = (a_re, -a_im)
                // conj(A) * B = (a_re * b_re + a_im * b_im, a_re * b_im - a_im * b_re)
                let re = a_re[k] * b_re[k] + a_im[k] * b_im[k];
                let im = a_re[k] * b_im[k] - a_im[k] * b_re[k];
                (re, im)
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const SAMPLE_RATE: f64 = 1000.0;

    fn make_tone(n: usize, freq: f64, phase: f64, fs: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let angle = 2.0 * PI * freq * t + phase;
                (angle.cos(), angle.sin())
            })
            .collect()
    }

    #[test]
    fn test_new_valid() {
        let a = PhaseCoherenceAnalyzer::new(48000.0);
        assert_eq!(a.sample_rate(), 48000.0);
    }

    #[test]
    #[should_panic]
    fn test_new_zero_sample_rate() {
        PhaseCoherenceAnalyzer::new(0.0);
    }

    #[test]
    #[should_panic]
    fn test_new_negative_sample_rate() {
        PhaseCoherenceAnalyzer::new(-100.0);
    }

    #[test]
    fn test_plv_identical_signals() {
        let analyzer = PhaseCoherenceAnalyzer::new(SAMPLE_RATE);
        let sig = make_tone(256, 50.0, 0.0, SAMPLE_RATE);
        let plv = analyzer.phase_locking_value(&sig, &sig);
        assert!(
            (plv - 1.0).abs() < 1e-10,
            "PLV of identical signals should be 1.0, got {}",
            plv
        );
    }

    #[test]
    fn test_plv_constant_phase_offset() {
        let analyzer = PhaseCoherenceAnalyzer::new(SAMPLE_RATE);
        let sig_a = make_tone(512, 50.0, 0.0, SAMPLE_RATE);
        let sig_b = make_tone(512, 50.0, 1.2, SAMPLE_RATE);
        let plv = analyzer.phase_locking_value(&sig_a, &sig_b);
        assert!(
            plv > 0.99,
            "Signals with constant phase offset should have PLV ~1.0, got {}",
            plv
        );
    }

    #[test]
    fn test_plv_uncorrelated_signals() {
        // Two signals at different, non-harmonically-related frequencies
        // over many cycles should have low PLV
        let analyzer = PhaseCoherenceAnalyzer::new(SAMPLE_RATE);
        let sig_a = make_tone(1024, 50.0, 0.0, SAMPLE_RATE);
        let sig_b = make_tone(1024, 73.0, 0.0, SAMPLE_RATE);
        let plv = analyzer.phase_locking_value(&sig_a, &sig_b);
        assert!(
            plv < 0.3,
            "Uncorrelated signals should have low PLV, got {}",
            plv
        );
    }

    #[test]
    #[should_panic]
    fn test_plv_empty_signals() {
        let analyzer = PhaseCoherenceAnalyzer::new(SAMPLE_RATE);
        analyzer.phase_locking_value(&[], &[]);
    }

    #[test]
    #[should_panic]
    fn test_plv_mismatched_lengths() {
        let analyzer = PhaseCoherenceAnalyzer::new(SAMPLE_RATE);
        let sig_a = make_tone(100, 50.0, 0.0, SAMPLE_RATE);
        let sig_b = make_tone(200, 50.0, 0.0, SAMPLE_RATE);
        analyzer.phase_locking_value(&sig_a, &sig_b);
    }

    #[test]
    fn test_coherence_locked_signals() {
        let analyzer = PhaseCoherenceAnalyzer::new(SAMPLE_RATE);
        let sig_a = make_tone(256, 50.0, 0.0, SAMPLE_RATE);
        let sig_b = make_tone(256, 50.0, 0.5, SAMPLE_RATE);
        let metrics = analyzer.coherence(&sig_a, &sig_b);

        assert!(
            metrics.phase_locking_value > 0.99,
            "PLV should be ~1.0, got {}",
            metrics.phase_locking_value
        );
        assert!(
            (metrics.mean_phase_diff_rad - (-0.5)).abs() < 0.05,
            "Mean phase diff should be ~-0.5 rad, got {}",
            metrics.mean_phase_diff_rad
        );
        assert!(
            metrics.phase_variance < 0.01,
            "Phase variance should be near 0, got {}",
            metrics.phase_variance
        );
        assert!(
            metrics.coherence_magnitude > 0.95,
            "Coherence magnitude should be near 1.0, got {}",
            metrics.coherence_magnitude
        );
    }

    #[test]
    fn test_coherence_zero_phase_diff() {
        let analyzer = PhaseCoherenceAnalyzer::new(SAMPLE_RATE);
        let sig = make_tone(256, 50.0, 0.0, SAMPLE_RATE);
        let metrics = analyzer.coherence(&sig, &sig);
        assert!(
            metrics.mean_phase_diff_rad.abs() < 0.01,
            "Mean phase diff of identical signals should be ~0, got {}",
            metrics.mean_phase_diff_rad
        );
    }

    #[test]
    fn test_phase_noise_pure_tone() {
        let analyzer = PhaseCoherenceAnalyzer::new(SAMPLE_RATE);
        let sig = make_tone(128, 100.0, 0.0, SAMPLE_RATE);
        let profile = analyzer.phase_noise(&sig, 100.0);

        assert!(!profile.offset_freq_hz.is_empty());
        assert_eq!(profile.offset_freq_hz.len(), profile.ssb_phase_noise_dbc_hz.len());
        // Pure tone should have very low integrated jitter
        assert!(
            profile.integrated_jitter_rms_rad < 1.0,
            "Pure tone jitter should be small, got {}",
            profile.integrated_jitter_rms_rad
        );
    }

    #[test]
    fn test_phase_noise_offsets_increasing() {
        let analyzer = PhaseCoherenceAnalyzer::new(SAMPLE_RATE);
        let sig = make_tone(64, 50.0, 0.0, SAMPLE_RATE);
        let profile = analyzer.phase_noise(&sig, 50.0);

        // Offset frequencies should be monotonically increasing
        for w in profile.offset_freq_hz.windows(2) {
            assert!(w[1] > w[0], "Offset frequencies must increase");
        }
    }

    #[test]
    #[should_panic]
    fn test_phase_noise_too_few_samples() {
        let analyzer = PhaseCoherenceAnalyzer::new(SAMPLE_RATE);
        let sig = make_tone(2, 50.0, 0.0, SAMPLE_RATE);
        analyzer.phase_noise(&sig, 50.0);
    }

    #[test]
    fn test_allan_variance_constant_phase() {
        // Constant accumulated phase => zero Allan variance
        let analyzer = PhaseCoherenceAnalyzer::new(SAMPLE_RATE);
        let phase_samples: Vec<f64> = (0..200).map(|i| i as f64 * 0.001).collect();
        // Linear ramp = constant frequency => AVAR should be near zero
        let avar = analyzer.allan_variance(&phase_samples, 0.01);
        assert!(
            avar < 1e-20,
            "Allan variance of linear ramp should be ~0, got {}",
            avar
        );
    }

    #[test]
    fn test_allan_variance_noisy_phase() {
        // Add some deterministic 'noise' to the phase
        let analyzer = PhaseCoherenceAnalyzer::new(SAMPLE_RATE);
        let phase_samples: Vec<f64> = (0..500)
            .map(|i| {
                let t = i as f64 * 0.001;
                t + 0.01 * (100.0 * t).sin()
            })
            .collect();
        let avar = analyzer.allan_variance(&phase_samples, 0.01);
        assert!(avar > 0.0, "Allan variance with noise should be > 0, got {}", avar);
    }

    #[test]
    #[should_panic]
    fn test_allan_variance_insufficient_samples() {
        let analyzer = PhaseCoherenceAnalyzer::new(SAMPLE_RATE);
        let phase_samples = vec![1.0, 2.0, 3.0];
        // tau = 0.005 => m = 5 => need > 10 samples, but only have 3
        analyzer.allan_variance(&phase_samples, 0.005);
    }

    #[test]
    fn test_cross_spectrum_identical() {
        let analyzer = PhaseCoherenceAnalyzer::new(SAMPLE_RATE);
        let sig = make_tone(32, 50.0, 0.0, SAMPLE_RATE);
        let cs = analyzer.cross_spectrum(&sig, &sig);

        assert_eq!(cs.len(), 32);
        // Cross-spectrum of a signal with itself = power spectrum (real, non-negative)
        // The imaginary parts should be ~0
        for &(re, im) in &cs {
            assert!(
                im.abs() < 1e-6,
                "Auto-spectrum imaginary part should be ~0, got {}",
                im
            );
            assert!(
                re >= -1e-6,
                "Auto-spectrum should be non-negative, got {}",
                re
            );
        }
    }

    #[test]
    fn test_cross_spectrum_length() {
        let analyzer = PhaseCoherenceAnalyzer::new(SAMPLE_RATE);
        let sig_a = make_tone(64, 50.0, 0.0, SAMPLE_RATE);
        let sig_b = make_tone(128, 50.0, 0.0, SAMPLE_RATE);
        let cs = analyzer.cross_spectrum(&sig_a, &sig_b);
        assert_eq!(cs.len(), 64, "Cross-spectrum length should equal shorter signal");
    }

    #[test]
    #[should_panic]
    fn test_cross_spectrum_empty() {
        let analyzer = PhaseCoherenceAnalyzer::new(SAMPLE_RATE);
        analyzer.cross_spectrum(&[], &[(1.0, 0.0)]);
    }

    #[test]
    fn test_plv_range() {
        // PLV should always be in [0, 1]
        let analyzer = PhaseCoherenceAnalyzer::new(SAMPLE_RATE);
        let sig_a = make_tone(256, 30.0, 0.0, SAMPLE_RATE);
        let sig_b = make_tone(256, 77.0, 1.0, SAMPLE_RATE);
        let plv = analyzer.phase_locking_value(&sig_a, &sig_b);
        assert!(
            (0.0..=1.0).contains(&plv),
            "PLV must be in [0,1], got {}",
            plv
        );
    }

    #[test]
    fn test_coherence_metrics_debug_clone() {
        let m = CoherenceMetrics {
            phase_locking_value: 0.95,
            mean_phase_diff_rad: 0.1,
            phase_variance: 0.02,
            coherence_magnitude: 0.9,
        };
        let m2 = m.clone();
        assert_eq!(
            format!("{:?}", m),
            format!("{:?}", m2),
            "Clone and Debug should work"
        );
    }

    #[test]
    fn test_phase_noise_profile_debug_clone() {
        let p = PhaseNoiseProfile {
            offset_freq_hz: vec![100.0],
            ssb_phase_noise_dbc_hz: vec![-100.0],
            integrated_jitter_rms_rad: 0.001,
        };
        let p2 = p.clone();
        assert_eq!(p2.offset_freq_hz.len(), 1);
        let _ = format!("{:?}", p);
    }
}
