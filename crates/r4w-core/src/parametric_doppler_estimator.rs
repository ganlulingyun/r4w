//! Maximum-likelihood and parametric Doppler frequency estimation.
//!
//! This module provides high-accuracy frequency estimation algorithms for
//! Doppler measurement, velocity determination, and frequency tracking. It
//! includes coarse FFT-based estimation, fine parabolic interpolation, Kay's
//! phase-based estimator, Pisarenko's autocorrelation method, multi-interval
//! coherent estimation, frequency rate (chirp) estimation, and ambiguity
//! resolution for aliased Doppler measurements.
//!
//! All complex numbers are represented as `(f64, f64)` tuples of (real, imag).
//!
//! # Example
//!
//! ```
//! use r4w_core::parametric_doppler_estimator::ParametricDopplerEstimator;
//!
//! let estimator = ParametricDopplerEstimator::new(1000.0, 256);
//!
//! // Generate a 50 Hz tone sampled at 1000 Hz
//! let n = 256;
//! let freq = 50.0_f64;
//! let samples: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / 1000.0;
//!         let phase = 2.0 * std::f64::consts::PI * freq * t;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! let est = estimator.estimate_frequency(&samples);
//! assert!((est - 50.0).abs() < 1.0);
//! ```

use std::f64::consts::PI;

/// Speed of light in m/s.
const C: f64 = 299_792_458.0;

// ── Complex helpers ──────────────────────────────────────────────────

type Complex = (f64, f64);

#[inline]
fn cx_mul(a: Complex, b: Complex) -> Complex {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn cx_conj(a: Complex) -> Complex {
    (a.0, -a.1)
}

#[inline]
fn cx_add(a: Complex, b: Complex) -> Complex {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn cx_mag_sq(a: Complex) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

#[inline]
fn cx_exp(phase: f64) -> Complex {
    (phase.cos(), phase.sin())
}

// ── In-place radix-2 DIT FFT (no external crates) ───────────────────

fn bit_reverse(x: usize, log2n: u32) -> usize {
    let mut result = 0usize;
    let mut val = x;
    for _ in 0..log2n {
        result = (result << 1) | (val & 1);
        val >>= 1;
    }
    result
}

/// In-place radix-2 DIT FFT. `buf.len()` **must** be a power of two.
fn fft_in_place(buf: &mut [Complex]) {
    let n = buf.len();
    assert!(n.is_power_of_two(), "FFT size must be a power of two");
    let log2n = n.trailing_zeros();

    // Bit-reverse permutation
    for i in 0..n {
        let j = bit_reverse(i, log2n);
        if i < j {
            buf.swap(i, j);
        }
    }

    // Cooley-Tukey butterfly
    let mut size = 2usize;
    while size <= n {
        let half = size / 2;
        let angle_step = -2.0 * PI / size as f64;
        for k in (0..n).step_by(size) {
            for j in 0..half {
                let w = cx_exp(angle_step * j as f64);
                let u = buf[k + j];
                let t = cx_mul(w, buf[k + j + half]);
                buf[k + j] = cx_add(u, t);
                buf[k + j + half] = (u.0 - t.0, u.1 - t.1);
            }
        }
        size <<= 1;
    }
}

// ── ParametricDopplerEstimator ──────────────────────────────────────

/// High-accuracy parametric Doppler / frequency estimator.
///
/// Combines coarse FFT peak detection with several fine-estimation
/// algorithms (parabolic interpolation, Kay's phase estimator,
/// Pisarenko autocorrelation, multi-interval coherent) and provides
/// utilities for Cramér-Rao bounds, chirp-rate estimation, velocity
/// conversion, and aliased-Doppler ambiguity resolution.
#[derive(Debug, Clone)]
pub struct ParametricDopplerEstimator {
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// FFT size (must be a power of two).
    pub fft_size: usize,
}

impl ParametricDopplerEstimator {
    /// Create a new estimator.
    ///
    /// # Panics
    /// Panics if `fft_size` is not a power of two or is zero.
    pub fn new(sample_rate: f64, fft_size: usize) -> Self {
        assert!(fft_size > 0 && fft_size.is_power_of_two(), "fft_size must be a power of two");
        Self { sample_rate, fft_size }
    }

    // ── Internal helpers ─────────────────────────────────────────────

    /// Zero-pad or truncate `samples` to `self.fft_size`, compute FFT,
    /// return the magnitude-squared spectrum.
    fn compute_spectrum(&self, samples: &[Complex]) -> Vec<f64> {
        let n = self.fft_size;
        let mut buf = vec![(0.0, 0.0); n];
        let copy_len = samples.len().min(n);
        buf[..copy_len].copy_from_slice(&samples[..copy_len]);
        fft_in_place(&mut buf);
        buf.iter().map(|&s| cx_mag_sq(s)).collect()
    }

    /// Index of the maximum value in a slice.
    fn argmax(slice: &[f64]) -> usize {
        let mut best = 0;
        let mut best_val = f64::NEG_INFINITY;
        for (i, &v) in slice.iter().enumerate() {
            if v > best_val {
                best_val = v;
                best = i;
            }
        }
        best
    }

    /// Convert an FFT bin index to a frequency in Hz, mapping the upper
    /// half of the spectrum to negative frequencies.
    fn bin_to_freq(&self, bin: f64) -> f64 {
        let n = self.fft_size as f64;
        let half = n / 2.0;
        let b = if bin >= half { bin - n } else { bin };
        b * self.sample_rate / n
    }

    // ── Public estimation methods ────────────────────────────────────

    /// Coarse frequency estimate via FFT peak detection.
    ///
    /// Resolution is `sample_rate / fft_size` Hz.
    pub fn coarse_estimate(&self, samples: &[Complex]) -> f64 {
        let spec = self.compute_spectrum(samples);
        let peak = Self::argmax(&spec);
        self.bin_to_freq(peak as f64)
    }

    /// Fine frequency estimate using parabolic (quadratic) interpolation
    /// around the FFT peak.  Achieves sub-bin accuracy.
    pub fn fine_estimate_parabolic(&self, samples: &[Complex]) -> f64 {
        let spec = self.compute_spectrum(samples);
        let n = self.fft_size;
        let peak = Self::argmax(&spec);

        // Neighbours (wrap-around)
        let left = if peak == 0 { n - 1 } else { peak - 1 };
        let right = if peak == n - 1 { 0 } else { peak + 1 };

        let alpha = spec[left].sqrt();
        let beta = spec[peak].sqrt();
        let gamma = spec[right].sqrt();

        let denom = alpha - 2.0 * beta + gamma;
        let delta = if denom.abs() < 1e-30 {
            0.0
        } else {
            0.5 * (alpha - gamma) / denom
        };

        self.bin_to_freq(peak as f64 + delta)
    }

    /// Combined coarse + fine estimation (convenience wrapper).
    pub fn estimate_frequency(&self, samples: &[Complex]) -> f64 {
        self.fine_estimate_parabolic(samples)
    }

    /// Kay's weighted-phase-difference frequency estimator.
    ///
    /// Operates directly on time-domain samples without FFT. Very low
    /// computational cost, good for moderate-to-high SNR.
    pub fn kay_estimate(&self, samples: &[Complex]) -> f64 {
        let n = samples.len();
        if n < 2 {
            return 0.0;
        }

        // Phase differences between consecutive samples
        let mut weighted_sum = 0.0;
        let mut weight_sum = 0.0;
        let nm1 = (n - 1) as f64;
        for i in 0..n - 1 {
            let prod = cx_mul(samples[i + 1], cx_conj(samples[i]));
            let phase_diff = prod.1.atan2(prod.0);
            // Kay's optimal weights for white noise
            let k = i as f64;
            let w = 6.0 * k * (nm1 - k) / (nm1 * (4.0 * nm1 * nm1 - 1.0));
            weighted_sum += w * phase_diff;
            weight_sum += w;
        }

        if weight_sum.abs() < 1e-30 {
            return 0.0;
        }

        let omega = weighted_sum / weight_sum;
        omega * self.sample_rate / (2.0 * PI)
    }

    /// Pisarenko's autocorrelation-based single-frequency estimator.
    ///
    /// Estimates a single sinusoidal frequency from the lag-1
    /// autocorrelation of the signal.
    pub fn pisarenko_estimate(&self, samples: &[Complex]) -> f64 {
        let n = samples.len();
        if n < 2 {
            return 0.0;
        }

        // Compute lag-0 and lag-1 autocorrelation
        let mut r0: Complex = (0.0, 0.0);
        let mut r1: Complex = (0.0, 0.0);
        for i in 0..n {
            r0 = cx_add(r0, cx_mul(samples[i], cx_conj(samples[i])));
        }
        for i in 0..n - 1 {
            r1 = cx_add(r1, cx_mul(samples[i + 1], cx_conj(samples[i])));
        }

        // Normalize
        let r0_mag = r0.0; // r0 is always real and positive
        if r0_mag.abs() < 1e-30 {
            return 0.0;
        }
        let r1_norm = (r1.0 / r0_mag, r1.1 / r0_mag);

        let omega = r1_norm.1.atan2(r1_norm.0);
        omega * self.sample_rate / (2.0 * PI)
    }

    /// Multi-interval coherent frequency estimation.
    ///
    /// Splits the signal into `num_intervals` segments, estimates the
    /// phase progression between them, and refines the coarse FFT
    /// estimate.  Improves accuracy when the signal spans many samples.
    pub fn coherent_estimate(&self, samples: &[Complex], num_intervals: usize) -> f64 {
        if num_intervals < 2 || samples.len() < 2 {
            return self.estimate_frequency(samples);
        }

        let seg_len = samples.len() / num_intervals;
        if seg_len == 0 {
            return self.estimate_frequency(samples);
        }

        // Get coarse estimate from full signal
        let coarse = self.coarse_estimate(samples);

        // De-rotate each segment by coarse estimate and accumulate
        // phase difference between consecutive segments.
        let mut prev_phase = 0.0f64;
        let mut total_delta = 0.0f64;
        let mut count = 0u64;

        for seg_idx in 0..num_intervals {
            let start = seg_idx * seg_len;
            let end = (start + seg_len).min(samples.len());
            let seg = &samples[start..end];

            // Correlate segment against coarse-frequency reference
            let mut acc: Complex = (0.0, 0.0);
            for (j, &s) in seg.iter().enumerate() {
                let t = (start + j) as f64 / self.sample_rate;
                let ref_sig = cx_exp(-2.0 * PI * coarse * t);
                acc = cx_add(acc, cx_mul(s, ref_sig));
            }

            let phase = acc.1.atan2(acc.0);
            if seg_idx > 0 {
                let mut dp = phase - prev_phase;
                // Unwrap
                while dp > PI {
                    dp -= 2.0 * PI;
                }
                while dp < -PI {
                    dp += 2.0 * PI;
                }
                total_delta += dp;
                count += 1;
            }
            prev_phase = phase;
        }

        if count == 0 {
            return coarse;
        }

        let avg_delta = total_delta / count as f64;
        let dt = seg_len as f64 / self.sample_rate;
        let fine_correction = avg_delta / (2.0 * PI * dt);

        coarse + fine_correction
    }

    /// Cramér-Rao lower bound (CRLB) for frequency estimation variance.
    ///
    /// Returns the minimum achievable variance (Hz²) for an unbiased
    /// frequency estimator given `n` samples and linear `snr` (not dB).
    ///
    /// Formula: `CRLB = 12 / (4π² · N · (N²−1) · SNR · Ts²)`
    /// simplified to `CRLB = 3·fs² / (π² · N · (N²−1) · SNR)`.
    pub fn cramer_rao_bound(&self, n: usize, snr_linear: f64) -> f64 {
        if n < 2 || snr_linear <= 0.0 {
            return f64::INFINITY;
        }
        let nf = n as f64;
        let fs = self.sample_rate;
        3.0 * fs * fs / (PI * PI * nf * (nf * nf - 1.0) * snr_linear)
    }

    /// Convert a Doppler frequency shift to radial velocity.
    ///
    /// `carrier_freq` is the carrier frequency in Hz.
    /// Returns velocity in m/s (positive = approaching).
    pub fn doppler_to_velocity(&self, doppler_hz: f64, carrier_freq: f64) -> f64 {
        if carrier_freq.abs() < 1e-30 {
            return 0.0;
        }
        // v = -c * f_d / f_c  (approaching → positive Doppler → negative velocity by convention,
        // but commonly reported as positive for approaching)
        -C * doppler_hz / carrier_freq
    }

    /// Convert a radial velocity to the expected Doppler shift.
    ///
    /// `velocity` in m/s (positive = approaching), `carrier_freq` in Hz.
    /// Returns Doppler shift in Hz.
    pub fn velocity_to_doppler(&self, velocity: f64, carrier_freq: f64) -> f64 {
        -velocity * carrier_freq / C
    }

    /// Estimate frequency rate (chirp rate) in Hz/s.
    ///
    /// Splits the signal in half, estimates frequency in each half,
    /// and computes the rate of change.
    pub fn estimate_frequency_rate(&self, samples: &[Complex]) -> f64 {
        let n = samples.len();
        if n < 4 {
            return 0.0;
        }
        let mid = n / 2;
        let f1 = self.estimate_frequency(&samples[..mid]);
        let f2 = self.estimate_frequency(&samples[mid..]);

        let t1 = (mid as f64 / 2.0) / self.sample_rate; // centre of first half
        let t2 = (mid as f64 + (n - mid) as f64 / 2.0) / self.sample_rate; // centre of second half
        let dt = t2 - t1;
        if dt.abs() < 1e-30 {
            return 0.0;
        }
        (f2 - f1) / dt
    }

    /// Resolve aliased Doppler by selecting the candidate closest to
    /// `expected_doppler` from the set of aliases spaced by `sample_rate`.
    ///
    /// Returns the un-aliased Doppler frequency in Hz.
    pub fn resolve_ambiguity(&self, measured_doppler: f64, expected_doppler: f64) -> f64 {
        let fs = self.sample_rate;
        // Bring measured into [-fs/2, fs/2)
        let mut m = measured_doppler % fs;
        if m > fs / 2.0 {
            m -= fs;
        }
        if m < -fs / 2.0 {
            m += fs;
        }

        // Search nearby aliases: m + k*fs for k in a reasonable range
        let k_center = ((expected_doppler - m) / fs).round() as i64;
        let mut best = m;
        let mut best_err = f64::INFINITY;
        for k in (k_center - 2)..=(k_center + 2) {
            let candidate = m + k as f64 * fs;
            let err = (candidate - expected_doppler).abs();
            if err < best_err {
                best_err = err;
                best = candidate;
            }
        }
        best
    }
}

// ── Tests ────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Helper: generate a complex tone at `freq` Hz sampled at `fs` Hz.
    fn tone(freq: f64, fs: f64, n: usize) -> Vec<Complex> {
        (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let phase = 2.0 * PI * freq * t;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    /// Helper: add white Gaussian-ish noise (Box-Muller, deterministic seed).
    fn add_noise(samples: &mut [Complex], noise_std: f64, seed: u64) {
        // Simple LCG for reproducibility (no external crate)
        let mut state = seed;
        let next = |s: &mut u64| -> f64 {
            *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            // Map to (0,1)
            (*s >> 11) as f64 / (1u64 << 53) as f64
        };
        for s in samples.iter_mut() {
            let u1 = next(&mut state).max(1e-15);
            let u2 = next(&mut state);
            let r = (-2.0 * u1.ln()).sqrt();
            let theta = 2.0 * PI * u2;
            s.0 += noise_std * r * theta.cos();
            s.1 += noise_std * r * theta.sin();
        }
    }

    // ── Construction ─────────────────────────────────────────────────

    #[test]
    fn test_new_valid() {
        let e = ParametricDopplerEstimator::new(1000.0, 256);
        assert_eq!(e.sample_rate, 1000.0);
        assert_eq!(e.fft_size, 256);
    }

    #[test]
    #[should_panic(expected = "fft_size must be a power of two")]
    fn test_new_non_power_of_two() {
        ParametricDopplerEstimator::new(1000.0, 100);
    }

    // ── Coarse estimation ────────────────────────────────────────────

    #[test]
    fn test_coarse_estimate_exact_bin() {
        let fs = 1024.0;
        let fft = 1024;
        let e = ParametricDopplerEstimator::new(fs, fft);
        // 50 Hz is bin 50 exactly
        let s = tone(50.0, fs, fft);
        let est = e.coarse_estimate(&s);
        assert!((est - 50.0).abs() < 1.5, "coarse={est}");
    }

    #[test]
    fn test_coarse_estimate_negative_freq() {
        let fs = 1024.0;
        let fft = 1024;
        let e = ParametricDopplerEstimator::new(fs, fft);
        let s = tone(-100.0, fs, fft);
        let est = e.coarse_estimate(&s);
        assert!((est - (-100.0)).abs() < 1.5, "coarse={est}");
    }

    // ── Fine parabolic estimation ────────────────────────────────────

    #[test]
    fn test_fine_parabolic_on_bin() {
        let fs = 1024.0;
        let fft = 1024;
        let e = ParametricDopplerEstimator::new(fs, fft);
        let s = tone(50.0, fs, fft);
        let est = e.fine_estimate_parabolic(&s);
        assert!((est - 50.0).abs() < 0.5, "fine={est}");
    }

    #[test]
    fn test_fine_parabolic_off_bin() {
        let fs = 1024.0;
        let fft = 1024;
        let e = ParametricDopplerEstimator::new(fs, fft);
        // 50.3 Hz is between bins
        let s = tone(50.3, fs, fft);
        let est = e.fine_estimate_parabolic(&s);
        assert!((est - 50.3).abs() < 0.5, "fine={est}");
    }

    // ── Kay's estimator ──────────────────────────────────────────────

    #[test]
    fn test_kay_estimate_clean() {
        let fs = 1000.0;
        let s = tone(123.0, fs, 256);
        let e = ParametricDopplerEstimator::new(fs, 256);
        let est = e.kay_estimate(&s);
        assert!((est - 123.0).abs() < 1.0, "kay={est}");
    }

    #[test]
    fn test_kay_estimate_negative() {
        let fs = 1000.0;
        let s = tone(-75.0, fs, 256);
        let e = ParametricDopplerEstimator::new(fs, 256);
        let est = e.kay_estimate(&s);
        assert!((est - (-75.0)).abs() < 1.0, "kay={est}");
    }

    #[test]
    fn test_kay_single_sample() {
        let e = ParametricDopplerEstimator::new(1000.0, 64);
        assert_eq!(e.kay_estimate(&[(1.0, 0.0)]), 0.0);
    }

    // ── Pisarenko estimator ──────────────────────────────────────────

    #[test]
    fn test_pisarenko_estimate_clean() {
        let fs = 1000.0;
        let s = tone(80.0, fs, 256);
        let e = ParametricDopplerEstimator::new(fs, 256);
        let est = e.pisarenko_estimate(&s);
        assert!((est - 80.0).abs() < 1.0, "pisarenko={est}");
    }

    #[test]
    fn test_pisarenko_negative_freq() {
        let fs = 1000.0;
        let s = tone(-60.0, fs, 256);
        let e = ParametricDopplerEstimator::new(fs, 256);
        let est = e.pisarenko_estimate(&s);
        assert!((est - (-60.0)).abs() < 1.0, "pisarenko={est}");
    }

    // ── Coherent estimation ──────────────────────────────────────────

    #[test]
    fn test_coherent_estimate() {
        let fs = 4096.0;
        let fft = 1024;
        let e = ParametricDopplerEstimator::new(fs, fft);
        // 200 Hz = bin 50 at fs=4096/fft=1024 (resolution=4 Hz), exactly on bin
        let s = tone(200.0, fs, 4096);
        let est = e.coherent_estimate(&s, 4);
        assert!((est - 200.0).abs() < 5.0, "coherent={est}");
    }

    #[test]
    fn test_coherent_single_interval_fallback() {
        let fs = 1024.0;
        let e = ParametricDopplerEstimator::new(fs, 64);
        // 48 Hz = bin 3 at fs=1024/fft=64 (resolution=16 Hz), exactly on bin
        let s = tone(48.0, fs, 64);
        // With num_intervals=1 falls back to estimate_frequency
        let est = e.coherent_estimate(&s, 1);
        assert!((est - 48.0).abs() < 2.0, "coherent_fallback={est}");
    }

    // ── Cramér-Rao bound ─────────────────────────────────────────────

    #[test]
    fn test_crlb_decreases_with_snr() {
        let e = ParametricDopplerEstimator::new(1000.0, 256);
        let crlb_low = e.cramer_rao_bound(256, 10.0);
        let crlb_high = e.cramer_rao_bound(256, 100.0);
        assert!(crlb_high < crlb_low);
    }

    #[test]
    fn test_crlb_decreases_with_n() {
        let e = ParametricDopplerEstimator::new(1000.0, 256);
        let crlb_short = e.cramer_rao_bound(64, 10.0);
        let crlb_long = e.cramer_rao_bound(256, 10.0);
        assert!(crlb_long < crlb_short);
    }

    #[test]
    fn test_crlb_edge_cases() {
        let e = ParametricDopplerEstimator::new(1000.0, 256);
        assert_eq!(e.cramer_rao_bound(1, 10.0), f64::INFINITY);
        assert_eq!(e.cramer_rao_bound(256, 0.0), f64::INFINITY);
        assert_eq!(e.cramer_rao_bound(256, -1.0), f64::INFINITY);
    }

    // ── Doppler-velocity conversion ──────────────────────────────────

    #[test]
    fn test_doppler_to_velocity_roundtrip() {
        let e = ParametricDopplerEstimator::new(1000.0, 256);
        let fc = 1.575e9; // GPS L1
        let v = 30.0; // m/s
        let d = e.velocity_to_doppler(v, fc);
        let v2 = e.doppler_to_velocity(d, fc);
        assert!((v2 - v).abs() < 1e-6, "roundtrip v={v2}");
    }

    #[test]
    fn test_doppler_to_velocity_zero_carrier() {
        let e = ParametricDopplerEstimator::new(1000.0, 256);
        assert_eq!(e.doppler_to_velocity(100.0, 0.0), 0.0);
    }

    // ── Frequency rate (chirp) estimation ────────────────────────────

    #[test]
    fn test_frequency_rate_chirp() {
        let fs = 4096.0;
        let n = 2048;
        let f0 = 100.0;
        let chirp_rate = 500.0; // Hz/s
        let samples: Vec<Complex> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let phase = 2.0 * PI * (f0 * t + 0.5 * chirp_rate * t * t);
                (phase.cos(), phase.sin())
            })
            .collect();
        let e = ParametricDopplerEstimator::new(fs, 512);
        let rate = e.estimate_frequency_rate(&samples);
        // Allow generous tolerance — split-half method is approximate
        assert!((rate - chirp_rate).abs() < 100.0, "chirp_rate={rate}");
    }

    #[test]
    fn test_frequency_rate_constant() {
        let fs = 1024.0;
        let n = 1024;
        let s = tone(100.0, fs, n);
        let e = ParametricDopplerEstimator::new(fs, 256);
        let rate = e.estimate_frequency_rate(&s);
        assert!(rate.abs() < 5.0, "constant_rate={rate}");
    }

    // ── Ambiguity resolution ─────────────────────────────────────────

    #[test]
    fn test_resolve_ambiguity_no_alias() {
        let e = ParametricDopplerEstimator::new(1000.0, 256);
        let resolved = e.resolve_ambiguity(100.0, 100.0);
        assert!((resolved - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_resolve_ambiguity_aliased() {
        let e = ParametricDopplerEstimator::new(1000.0, 256);
        // True Doppler 1300 Hz aliases to 300 Hz at fs=1000
        let resolved = e.resolve_ambiguity(300.0, 1300.0);
        assert!((resolved - 1300.0).abs() < 1e-6, "resolved={resolved}");
    }

    // ── Noisy signal estimation ──────────────────────────────────────

    #[test]
    fn test_estimation_with_noise() {
        let fs = 4096.0;
        let fft = 512;
        let freq = 150.0;
        let mut s = tone(freq, fs, 1024);
        add_noise(&mut s, 0.1, 42);
        let e = ParametricDopplerEstimator::new(fs, fft);
        let est = e.estimate_frequency(&s);
        assert!((est - freq).abs() < 3.0, "noisy_est={est}");
    }
}
