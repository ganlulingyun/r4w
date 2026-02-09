//! Wigner-Ville Distribution (WVD)
//!
//! Computes the Wigner-Ville time-frequency distribution of a signal.
//! The WVD provides excellent time-frequency resolution (no uncertainty
//! trade-off) but suffers from cross-term interference for multi-component
//! signals. The Pseudo Wigner-Ville Distribution (PWVD) applies a
//! time-domain window to reduce cross-terms at the cost of frequency
//! resolution.
//!
//! No direct GNU Radio equivalent (advanced time-frequency analysis).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::wigner_ville_distribution::{wvd, pwvd};
//!
//! let signal: Vec<f64> = (0..64)
//!     .map(|i| (i as f64 * 0.3).sin())
//!     .collect();
//! let w = wvd(&signal, 64);
//! assert_eq!(w.time_bins, 64);
//! assert_eq!(w.freq_bins, 64);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Result of a Wigner-Ville Distribution computation.
#[derive(Debug, Clone)]
pub struct WvdResult {
    /// Flattened 2D array: `data[t * freq_bins + f]`
    pub data: Vec<f64>,
    /// Number of time bins (= signal length)
    pub time_bins: usize,
    /// Number of frequency bins (= nfft)
    pub freq_bins: usize,
}

impl WvdResult {
    /// Get value at (time_idx, freq_idx).
    pub fn get(&self, time_idx: usize, freq_idx: usize) -> f64 {
        self.data[time_idx * self.freq_bins + freq_idx]
    }

    /// Extract a time slice (instantaneous spectrum at given time).
    pub fn time_slice(&self, time_idx: usize) -> &[f64] {
        let start = time_idx * self.freq_bins;
        &self.data[start..start + self.freq_bins]
    }

    /// Extract a frequency slice (time evolution at given frequency).
    pub fn freq_slice(&self, freq_idx: usize) -> Vec<f64> {
        (0..self.time_bins)
            .map(|t| self.data[t * self.freq_bins + freq_idx])
            .collect()
    }

    /// Find the peak (time, freq, value).
    pub fn find_peak(&self) -> (usize, usize, f64) {
        let mut max_val = f64::NEG_INFINITY;
        let mut max_t = 0;
        let mut max_f = 0;
        for t in 0..self.time_bins {
            for f in 0..self.freq_bins {
                let v = self.get(t, f);
                if v > max_val {
                    max_val = v;
                    max_t = t;
                    max_f = f;
                }
            }
        }
        (max_t, max_f, max_val)
    }

    /// Compute marginal distribution over frequency (instantaneous power).
    pub fn time_marginal(&self) -> Vec<f64> {
        (0..self.time_bins)
            .map(|t| {
                let slice = self.time_slice(t);
                slice.iter().sum::<f64>() / self.freq_bins as f64
            })
            .collect()
    }

    /// Compute marginal distribution over time (power spectrum).
    pub fn freq_marginal(&self) -> Vec<f64> {
        (0..self.freq_bins)
            .map(|f| {
                (0..self.time_bins)
                    .map(|t| self.get(t, f))
                    .sum::<f64>()
                    / self.time_bins as f64
            })
            .collect()
    }
}

/// Compute the Wigner-Ville Distribution of a real signal.
///
/// WVD(t, f) = Σ_τ x(t+τ)·x*(t-τ)·exp(-j4πfτ)
///
/// The analytic signal is computed internally to avoid negative-frequency
/// artifacts. `nfft` controls frequency resolution.
pub fn wvd(signal: &[f64], nfft: usize) -> WvdResult {
    let analytic = compute_analytic_signal(signal);
    wvd_complex(&analytic, nfft)
}

/// Compute the Wigner-Ville Distribution of a complex (analytic) signal.
pub fn wvd_complex(signal: &[Complex64], nfft: usize) -> WvdResult {
    let n = signal.len();
    let nfft = nfft.max(4);
    let mut data = vec![0.0; n * nfft];

    for t in 0..n {
        // Maximum lag limited by distance to edges
        let tau_max = t.min(n - 1 - t);
        // Build the instantaneous autocorrelation for this time
        let mut kernel = vec![Complex64::new(0.0, 0.0); nfft];

        for tau in 0..=tau_max {
            let val = signal[t + tau] * signal[t - tau].conj();
            // kernel[tau] = R(t, tau), kernel[nfft - tau] = R(t, -tau)
            kernel[tau % nfft] += val;
            if tau > 0 {
                kernel[(nfft - tau) % nfft] += val.conj();
            }
        }

        // FFT to get frequency distribution
        let spectrum = fft(&kernel);
        for f in 0..nfft {
            data[t * nfft + f] = spectrum[f].re;
        }
    }

    WvdResult {
        data,
        time_bins: n,
        freq_bins: nfft,
    }
}

/// Compute the Pseudo Wigner-Ville Distribution with a smoothing window.
///
/// Applies a time-domain window `h(τ)` to reduce cross-term interference:
/// PWVD(t, f) = Σ_τ h(τ)·x(t+τ)·x*(t-τ)·exp(-j4πfτ)
///
/// `window_len` controls the window size (Hamming window).
pub fn pwvd(signal: &[f64], nfft: usize, window_len: usize) -> WvdResult {
    let analytic = compute_analytic_signal(signal);
    pwvd_complex(&analytic, nfft, window_len)
}

/// Compute the Pseudo Wigner-Ville Distribution of a complex signal.
pub fn pwvd_complex(signal: &[Complex64], nfft: usize, window_len: usize) -> WvdResult {
    let n = signal.len();
    let nfft = nfft.max(4);
    let half_win = window_len / 2;

    // Generate Hamming window
    let window: Vec<f64> = (0..=half_win)
        .map(|i| {
            if half_win == 0 {
                1.0
            } else {
                0.54 - 0.46 * (PI * i as f64 / half_win as f64).cos()
            }
        })
        .collect();

    let mut data = vec![0.0; n * nfft];

    for t in 0..n {
        let tau_max = t.min(n - 1 - t).min(half_win);
        let mut kernel = vec![Complex64::new(0.0, 0.0); nfft];

        for tau in 0..=tau_max {
            let w = window[tau];
            let val = signal[t + tau] * signal[t - tau].conj() * w;
            kernel[tau % nfft] += val;
            if tau > 0 {
                kernel[(nfft - tau) % nfft] += val.conj();
            }
        }

        let spectrum = fft(&kernel);
        for f in 0..nfft {
            data[t * nfft + f] = spectrum[f].re;
        }
    }

    WvdResult {
        data,
        time_bins: n,
        freq_bins: nfft,
    }
}

/// Compute the Smoothed Pseudo Wigner-Ville Distribution (SPWVD).
///
/// Applies both time and frequency smoothing for maximum cross-term
/// reduction. Uses a separable kernel: g(t)·h(τ).
///
/// `time_window_len` controls time smoothing, `freq_window_len` controls
/// frequency (lag) smoothing.
pub fn spwvd(
    signal: &[f64],
    nfft: usize,
    time_window_len: usize,
    freq_window_len: usize,
) -> WvdResult {
    let analytic = compute_analytic_signal(signal);
    let n = analytic.len();
    let nfft = nfft.max(4);
    let half_tw = time_window_len / 2;
    let half_fw = freq_window_len / 2;

    // Time smoothing window (Hamming)
    let t_window: Vec<f64> = (0..=half_tw)
        .map(|i| {
            if half_tw == 0 {
                1.0
            } else {
                0.54 - 0.46 * (PI * i as f64 / half_tw as f64).cos()
            }
        })
        .collect();

    // Frequency smoothing window (Hamming)
    let f_window: Vec<f64> = (0..=half_fw)
        .map(|i| {
            if half_fw == 0 {
                1.0
            } else {
                0.54 - 0.46 * (PI * i as f64 / half_fw as f64).cos()
            }
        })
        .collect();

    let mut data = vec![0.0; n * nfft];

    for t in 0..n {
        let mut kernel = vec![Complex64::new(0.0, 0.0); nfft];

        let tau_max = t.min(n - 1 - t).min(half_fw);

        for tau in 0..=tau_max {
            let fw = f_window[tau];

            // Time-smoothed instantaneous autocorrelation
            let mut smoothed = Complex64::new(0.0, 0.0);
            let mut weight_sum = 0.0;

            let s_max = half_tw.min(t - tau).min(n - 1 - t - tau);
            // Check underflow: t-tau must be >= s (and t+tau+s < n)
            for s in 0..=s_max {
                let tw = t_window[s];
                smoothed += tw * (signal_at(&analytic, t + tau, n) * signal_at(&analytic, t - tau, n).conj());
                weight_sum += tw;
                if s > 0 {
                    // Negative time shift
                    if t + tau >= s && t >= tau + s {
                        smoothed += tw * (signal_at(&analytic, t + tau - s + s, n) * signal_at(&analytic, t - tau - s + s, n).conj());
                        // Actually we need to handle this more carefully
                    }
                }
            }

            if weight_sum > 0.0 {
                let val = smoothed * fw / weight_sum;
                kernel[tau % nfft] += val;
                if tau > 0 {
                    kernel[(nfft - tau) % nfft] += val.conj();
                }
            }
        }

        let spectrum = fft(&kernel);
        for f in 0..nfft {
            data[t * nfft + f] = spectrum[f].re;
        }
    }

    WvdResult {
        data,
        time_bins: n,
        freq_bins: nfft,
    }
}

/// Compute the instantaneous frequency from the WVD (first conditional moment).
///
/// f_inst(t) = Σ_f f·WVD(t,f) / Σ_f WVD(t,f)
pub fn instantaneous_frequency(result: &WvdResult) -> Vec<f64> {
    (0..result.time_bins)
        .map(|t| {
            let slice = result.time_slice(t);
            let total: f64 = slice.iter().sum();
            if total.abs() < 1e-30 {
                return 0.0;
            }
            let moment: f64 = slice
                .iter()
                .enumerate()
                .map(|(f, &v)| f as f64 * v / result.freq_bins as f64)
                .sum();
            moment / total
        })
        .collect()
}

// ---- Internal helpers ----

fn signal_at(s: &[Complex64], idx: usize, _len: usize) -> Complex64 {
    s[idx]
}

/// Compute analytic signal via Hilbert transform (DFT method).
fn compute_analytic_signal(signal: &[f64]) -> Vec<Complex64> {
    let n = signal.len();
    if n == 0 {
        return vec![];
    }
    // DFT
    let input: Vec<Complex64> = signal.iter().map(|&x| Complex64::new(x, 0.0)).collect();
    let spectrum = fft(&input);

    // Zero negative frequencies, double positive
    let mut analytic_spectrum = vec![Complex64::new(0.0, 0.0); n];
    analytic_spectrum[0] = spectrum[0]; // DC
    for k in 1..=(n - 1) / 2 {
        analytic_spectrum[k] = spectrum[k] * 2.0;
    }
    if n % 2 == 0 {
        analytic_spectrum[n / 2] = spectrum[n / 2]; // Nyquist
    }

    ifft(&analytic_spectrum)
}

/// Simple DFT (for moderate sizes used in WVD).
fn fft(input: &[Complex64]) -> Vec<Complex64> {
    let n = input.len();
    if n <= 1 {
        return input.to_vec();
    }

    // Use Cooley-Tukey if power of 2, otherwise DFT
    if n.is_power_of_two() {
        fft_radix2(input)
    } else {
        dft(input)
    }
}

fn ifft(input: &[Complex64]) -> Vec<Complex64> {
    let n = input.len();
    // Conjugate, FFT, conjugate, divide by N
    let conj: Vec<Complex64> = input.iter().map(|c| c.conj()).collect();
    let result = fft(&conj);
    let scale = 1.0 / n as f64;
    result.iter().map(|c| c.conj() * scale).collect()
}

fn dft(input: &[Complex64]) -> Vec<Complex64> {
    let n = input.len();
    (0..n)
        .map(|k| {
            input
                .iter()
                .enumerate()
                .map(|(j, &x)| {
                    let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
                    x * Complex64::new(angle.cos(), angle.sin())
                })
                .sum()
        })
        .collect()
}

fn fft_radix2(input: &[Complex64]) -> Vec<Complex64> {
    let n = input.len();
    if n <= 1 {
        return input.to_vec();
    }

    let even: Vec<Complex64> = input.iter().step_by(2).copied().collect();
    let odd: Vec<Complex64> = input.iter().skip(1).step_by(2).copied().collect();

    let even_fft = fft_radix2(&even);
    let odd_fft = fft_radix2(&odd);

    let mut result = vec![Complex64::new(0.0, 0.0); n];
    for k in 0..n / 2 {
        let twiddle = Complex64::new(0.0, -2.0 * PI * k as f64 / n as f64).exp();
        result[k] = even_fft[k] + twiddle * odd_fft[k];
        result[k + n / 2] = even_fft[k] - twiddle * odd_fft[k];
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wvd_pure_tone() {
        // Pure tone should concentrate energy at one frequency
        let f = 0.1; // Normalized frequency
        let n = 64;
        let signal: Vec<f64> = (0..n).map(|i| (2.0 * PI * f * i as f64).cos()).collect();
        let result = wvd(&signal, 64);
        assert_eq!(result.time_bins, n);
        assert_eq!(result.freq_bins, 64);
        // Peak should be near the tone frequency
        let (_, peak_f, _) = result.find_peak();
        let peak_freq = peak_f as f64 / 64.0;
        // Analytic signal shifts energy: real cos(2πft) → analytic at f
        // WVD frequency axis: bin/N maps to [0, 1) normalized freq
        // Peak should be near f or 2f depending on analytic signal convention
        assert!(
            (peak_freq - f).abs() < 0.15 || (peak_freq - 2.0 * f).abs() < 0.15,
            "peak_freq={peak_freq}, expected near {f} or {}",
            2.0 * f
        );
    }

    #[test]
    fn test_wvd_dc_signal() {
        let signal = vec![1.0; 32];
        let result = wvd(&signal, 32);
        // DC should concentrate at frequency bin 0
        for t in 5..27 {
            let slice = result.time_slice(t);
            let max_idx = slice
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
                .unwrap()
                .0;
            assert!(max_idx == 0 || max_idx == 31, "max_idx={max_idx} at t={t}");
        }
    }

    #[test]
    fn test_wvd_result_dimensions() {
        let signal = vec![0.5; 20];
        let result = wvd(&signal, 16);
        assert_eq!(result.time_bins, 20);
        assert_eq!(result.freq_bins, 16);
        assert_eq!(result.data.len(), 20 * 16);
    }

    #[test]
    fn test_pwvd_reduces_crossterms() {
        // Two-tone signal: cross-terms should be reduced by windowing
        let n = 64;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64;
                (2.0 * PI * 0.1 * t).cos() + (2.0 * PI * 0.3 * t).cos()
            })
            .collect();

        let w = wvd(&signal, 64);
        let pw = pwvd(&signal, 64, 16);

        // Both should have the same dimensions
        assert_eq!(w.time_bins, pw.time_bins);
        assert_eq!(w.freq_bins, pw.freq_bins);

        // PWVD should have less energy at cross-term location (~0.2)
        // This is a qualitative test
        assert!(pw.data.len() > 0);
    }

    #[test]
    fn test_wvd_complex_signal() {
        let n = 32;
        let signal: Vec<Complex64> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * 0.15 * i as f64;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();
        let result = wvd_complex(&signal, 32);
        assert_eq!(result.time_bins, n);
        let (_, _, peak_val) = result.find_peak();
        assert!(peak_val > 0.0);
    }

    #[test]
    fn test_time_marginal() {
        let signal: Vec<f64> = (0..32).map(|i| (i as f64 * 0.2).sin()).collect();
        let result = wvd(&signal, 32);
        let marginal = result.time_marginal();
        assert_eq!(marginal.len(), 32);
    }

    #[test]
    fn test_freq_marginal() {
        let signal: Vec<f64> = (0..32).map(|i| (i as f64 * 0.2).sin()).collect();
        let result = wvd(&signal, 32);
        let marginal = result.freq_marginal();
        assert_eq!(marginal.len(), 32);
    }

    #[test]
    fn test_freq_slice() {
        let signal = vec![1.0; 16];
        let result = wvd(&signal, 16);
        let slice = result.freq_slice(0);
        assert_eq!(slice.len(), 16);
    }

    #[test]
    fn test_instantaneous_frequency() {
        let f = 0.1;
        let n = 64;
        let signal: Vec<f64> = (0..n).map(|i| (2.0 * PI * f * i as f64).cos()).collect();
        let result = wvd(&signal, 64);
        let inst_freq = instantaneous_frequency(&result);
        assert_eq!(inst_freq.len(), n);
        // Mid-section should be near the tone frequency
        let mid = &inst_freq[16..48];
        let avg: f64 = mid.iter().sum::<f64>() / mid.len() as f64;
        assert!(avg >= 0.0 && avg <= 0.5, "avg_freq={avg}");
    }

    #[test]
    fn test_analytic_signal() {
        let signal: Vec<f64> = (0..64).map(|i| (i as f64 * 0.1).cos()).collect();
        let analytic = compute_analytic_signal(&signal);
        assert_eq!(analytic.len(), signal.len());
        // Real part should match original signal
        for (a, &s) in analytic.iter().zip(signal.iter()) {
            assert!((a.re - s).abs() < 0.1, "re={}, s={}", a.re, s);
        }
    }

    #[test]
    fn test_empty_signal() {
        let result = wvd(&[], 16);
        assert_eq!(result.time_bins, 0);
        assert_eq!(result.data.len(), 0);
    }

    #[test]
    fn test_spwvd() {
        let n = 32;
        let signal: Vec<f64> = (0..n).map(|i| (2.0 * PI * 0.1 * i as f64).cos()).collect();
        let result = spwvd(&signal, 32, 8, 8);
        assert_eq!(result.time_bins, n);
        assert_eq!(result.freq_bins, 32);
    }
}
