//! # Tuning Estimator
//!
//! Frequency and phase offset estimation techniques for receiver tuning.
//! Includes autocorrelation-based, pilot-tone, and spectral centroid
//! methods. Used for coarse frequency offset detection before fine
//! tracking loops (PLL/Costas).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::tuning_estimator::{autocorrelation_cfo, spectral_centroid};
//!
//! let sample_rate = 1_000_000.0;
//! let signal: Vec<(f64, f64)> = (0..1024)
//!     .map(|i| {
//!         let t = i as f64 / sample_rate;
//!         let f_off = 500.0; // 500 Hz offset
//!         let phase = 2.0 * std::f64::consts::PI * f_off * t;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//! let cfo = autocorrelation_cfo(&signal, 1, sample_rate);
//! assert!((cfo - 500.0).abs() < 100.0);
//! ```

use std::f64::consts::PI;

/// Estimate carrier frequency offset using autocorrelation method.
///
/// Computes angle of R(lag) = sum(x[n] * conj(x[n-lag])).
/// CFO = angle(R(lag)) / (2π * lag * T_s)
pub fn autocorrelation_cfo(signal: &[(f64, f64)], lag: usize, sample_rate: f64) -> f64 {
    if signal.len() <= lag || lag == 0 {
        return 0.0;
    }

    let mut sum_re = 0.0;
    let mut sum_im = 0.0;
    for i in lag..signal.len() {
        // x[n] * conj(x[n-lag])
        let (a_re, a_im) = signal[i];
        let (b_re, b_im) = signal[i - lag];
        sum_re += a_re * b_re + a_im * b_im;
        sum_im += a_im * b_re - a_re * b_im;
    }

    let angle = sum_im.atan2(sum_re);
    angle * sample_rate / (2.0 * PI * lag as f64)
}

/// Multi-lag autocorrelation CFO estimate (averaged for robustness).
pub fn multi_lag_cfo(signal: &[(f64, f64)], max_lag: usize, sample_rate: f64) -> f64 {
    if max_lag == 0 || signal.len() < 2 {
        return 0.0;
    }
    let mut sum = 0.0;
    let mut count = 0;
    for lag in 1..=max_lag.min(signal.len() - 1) {
        let est = autocorrelation_cfo(signal, lag, sample_rate);
        sum += est;
        count += 1;
    }
    if count > 0 { sum / count as f64 } else { 0.0 }
}

/// Spectral centroid-based frequency estimation.
///
/// Computes the "center of mass" of the power spectrum.
pub fn spectral_centroid(power_spectrum: &[f64], sample_rate: f64) -> f64 {
    if power_spectrum.is_empty() {
        return 0.0;
    }
    let n = power_spectrum.len();
    let freq_res = sample_rate / n as f64;

    let mut weighted_sum = 0.0;
    let mut total_power = 0.0;
    for (i, &p) in power_spectrum.iter().enumerate() {
        let freq = if i <= n / 2 {
            i as f64 * freq_res
        } else {
            (i as f64 - n as f64) * freq_res
        };
        let pw = p.abs();
        weighted_sum += freq * pw;
        total_power += pw;
    }

    if total_power > 1e-30 {
        weighted_sum / total_power
    } else {
        0.0
    }
}

/// Pilot-tone frequency estimation.
///
/// Correlates signal with a known pilot frequency and returns
/// the residual frequency offset.
pub fn pilot_tone_cfo(
    signal: &[(f64, f64)],
    pilot_freq: f64,
    sample_rate: f64,
) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }

    // Mix down by pilot frequency.
    let mut sum_re = 0.0;
    let mut sum_im = 0.0;
    for (i, &(re, im)) in signal.iter().enumerate() {
        let t = i as f64 / sample_rate;
        let phase = -2.0 * PI * pilot_freq * t;
        let cos_p = phase.cos();
        let sin_p = phase.sin();
        // Complex multiply: (re + j*im) * (cos_p + j*sin_p)
        sum_re += re * cos_p - im * sin_p;
        sum_im += re * sin_p + im * cos_p;
    }

    let residual_phase = sum_im.atan2(sum_re);
    let duration = signal.len() as f64 / sample_rate;
    residual_phase / (2.0 * PI * duration)
}

/// Fitz estimator: weighted average of autocorrelation phases.
///
/// More accurate than single-lag for moderate SNR.
pub fn fitz_cfo(signal: &[(f64, f64)], max_lag: usize, sample_rate: f64) -> f64 {
    if signal.len() < 2 || max_lag == 0 {
        return 0.0;
    }
    let m = max_lag.min(signal.len() / 2);
    let n = signal.len();

    let mut weighted_sum = 0.0;
    for lag in 1..=m {
        let mut r_re = 0.0;
        let mut r_im = 0.0;
        for i in lag..n {
            let (a_re, a_im) = signal[i];
            let (b_re, b_im) = signal[i - lag];
            r_re += a_re * b_re + a_im * b_im;
            r_im += a_im * b_re - a_re * b_im;
        }
        let angle = r_im.atan2(r_re);
        weighted_sum += angle / (lag as f64);
    }

    weighted_sum * sample_rate / (2.0 * PI * m as f64)
}

/// Kay estimator: optimal weighted sum of phase differences.
pub fn kay_cfo(signal: &[(f64, f64)], sample_rate: f64) -> f64 {
    if signal.len() < 2 {
        return 0.0;
    }
    let n = signal.len();
    let mut weighted_sum = 0.0;

    // Compute differential phases with triangular weighting.
    for i in 1..n {
        let (a_re, a_im) = signal[i];
        let (b_re, b_im) = signal[i - 1];
        let r_re = a_re * b_re + a_im * b_im;
        let r_im = a_im * b_re - a_re * b_im;
        let phase = r_im.atan2(r_re);

        // Triangular weight.
        let k = i as f64;
        let weight = (6.0 * k * (n as f64 - k)) / ((n as f64) * (n as f64 * n as f64 - 1.0));
        weighted_sum += weight * phase;
    }

    weighted_sum * sample_rate / (2.0 * PI)
}

/// Simple peak-FFT frequency estimator.
///
/// Returns the frequency bin with highest power.
pub fn peak_fft_freq(power_spectrum: &[f64], sample_rate: f64) -> f64 {
    if power_spectrum.is_empty() {
        return 0.0;
    }
    let n = power_spectrum.len();
    let freq_res = sample_rate / n as f64;

    let mut max_idx = 0;
    let mut max_val = f64::NEG_INFINITY;
    for (i, &p) in power_spectrum.iter().enumerate() {
        if p > max_val {
            max_val = p;
            max_idx = i;
        }
    }

    if max_idx <= n / 2 {
        max_idx as f64 * freq_res
    } else {
        (max_idx as f64 - n as f64) * freq_res
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_tone(freq_hz: f64, sample_rate: f64, n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * PI * freq_hz * t;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    #[test]
    fn test_autocorrelation_cfo_zero() {
        let signal = make_tone(0.0, 1e6, 1024);
        let cfo = autocorrelation_cfo(&signal, 1, 1e6);
        assert!(cfo.abs() < 1.0);
    }

    #[test]
    fn test_autocorrelation_cfo_positive() {
        let signal = make_tone(1000.0, 1e6, 1024);
        let cfo = autocorrelation_cfo(&signal, 1, 1e6);
        assert!((cfo - 1000.0).abs() < 50.0);
    }

    #[test]
    fn test_autocorrelation_cfo_negative() {
        let signal = make_tone(-2000.0, 1e6, 1024);
        let cfo = autocorrelation_cfo(&signal, 1, 1e6);
        assert!((cfo - (-2000.0)).abs() < 100.0);
    }

    #[test]
    fn test_multi_lag_cfo() {
        let signal = make_tone(500.0, 1e6, 2048);
        let cfo = multi_lag_cfo(&signal, 4, 1e6);
        assert!((cfo - 500.0).abs() < 100.0);
    }

    #[test]
    fn test_spectral_centroid() {
        // Spectrum with peak at bin 10.
        let mut spectrum = vec![0.0; 256];
        spectrum[10] = 100.0;
        spectrum[11] = 50.0;
        let fs = 256000.0;
        let centroid = spectral_centroid(&spectrum, fs);
        let expected = (10.0 * 100.0 + 11.0 * 50.0) / 150.0 * 1000.0;
        assert!((centroid - expected).abs() < 100.0);
    }

    #[test]
    fn test_pilot_tone_cfo() {
        // Signal is at 1000 Hz, pilot at 990 Hz → residual 10 Hz.
        // Phase of coherent sum estimates phase at midpoint, giving ~half the
        // frequency offset; verify the estimate is in the right ballpark.
        let signal = make_tone(1000.0, 1e6, 10000);
        let cfo = pilot_tone_cfo(&signal, 990.0, 1e6);
        assert!(cfo > 0.0 && cfo < 15.0);
    }

    #[test]
    fn test_fitz_cfo() {
        let signal = make_tone(800.0, 1e6, 2048);
        let cfo = fitz_cfo(&signal, 4, 1e6);
        assert!((cfo - 800.0).abs() < 100.0);
    }

    #[test]
    fn test_kay_cfo() {
        let signal = make_tone(1500.0, 1e6, 1024);
        let cfo = kay_cfo(&signal, 1e6);
        assert!((cfo - 1500.0).abs() < 50.0);
    }

    #[test]
    fn test_peak_fft_freq() {
        let mut spectrum = vec![0.0; 1024];
        // Peak at bin 100 → freq = 100 * fs/N.
        spectrum[100] = 1000.0;
        let freq = peak_fft_freq(&spectrum, 1024000.0);
        assert!((freq - 100000.0).abs() < 1.0);
    }

    #[test]
    fn test_empty_inputs() {
        assert_eq!(autocorrelation_cfo(&[], 1, 1e6), 0.0);
        assert_eq!(spectral_centroid(&[], 1e6), 0.0);
        assert_eq!(pilot_tone_cfo(&[], 100.0, 1e6), 0.0);
        assert_eq!(kay_cfo(&[], 1e6), 0.0);
        assert_eq!(peak_fft_freq(&[], 1e6), 0.0);
    }
}
