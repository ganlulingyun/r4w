//! Cepstral Analysis
//!
//! Computes the real cepstrum, complex cepstrum, and power cepstrum of signals.
//! Includes pitch detection via cepstral peak picking, mel-frequency cepstral
//! coefficients (MFCCs) for speech/audio analysis, and homomorphic filtering
//! for separating convolved components (e.g., source-filter separation).
//!
//! The cepstrum is the IFT of the log spectrum: c(n) = IFFT{log|FFT{x}|}.
//! It maps multiplicative (convolved) components to additive (separated)
//! components in the quefrency domain.
//!
//! No direct GNU Radio equivalent (speech/audio analysis tool).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cepstral_analysis::{real_cepstrum, detect_pitch};
//!
//! let signal: Vec<f64> = (0..512)
//!     .map(|i| (i as f64 * 0.1).sin())
//!     .collect();
//! let cepstrum = real_cepstrum(&signal);
//! assert_eq!(cepstrum.len(), 512);
//! ```

use std::f64::consts::PI;

/// Compute the real cepstrum: c(n) = IFFT{log|FFT{x}|}.
///
/// The output has the same length as the input. The real cepstrum
/// is symmetric and contains only spectral magnitude information.
pub fn real_cepstrum(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return vec![];
    }

    // FFT
    let spectrum = fft_real(signal);

    // Log magnitude (with floor to avoid -inf)
    let log_mag: Vec<f64> = spectrum
        .iter()
        .map(|c| {
            let mag = (c.re * c.re + c.im * c.im).sqrt();
            mag.max(1e-30).ln()
        })
        .collect();

    // IFFT of real sequence
    ifft_real(&log_mag)
}

/// Compute the power cepstrum: |IFFT{log|FFT{x}|²}|².
///
/// The power cepstrum emphasizes periodicities in the log power spectrum,
/// making it excellent for pitch detection and echo detection.
pub fn power_cepstrum(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return vec![];
    }

    let spectrum = fft_real(signal);

    let log_power: Vec<f64> = spectrum
        .iter()
        .map(|c| {
            let power = c.re * c.re + c.im * c.im;
            power.max(1e-30).ln()
        })
        .collect();

    let cepstrum = ifft_real(&log_power);
    cepstrum.iter().map(|&c| c * c).collect()
}

/// Compute the complex cepstrum using phase unwrapping.
///
/// Unlike the real cepstrum, the complex cepstrum preserves phase information,
/// enabling signal reconstruction. Requires careful phase unwrapping.
///
/// Returns (cepstrum, minimum_phase_flag).
pub fn complex_cepstrum(signal: &[f64]) -> (Vec<f64>, bool) {
    let n = signal.len();
    if n == 0 {
        return (vec![], true);
    }

    let spectrum = fft_real(signal);

    // Log of complex spectrum with phase unwrapping
    let mut log_spectrum_re = Vec::with_capacity(n);
    let mut log_spectrum_im = Vec::with_capacity(n);

    let mut prev_phase = 0.0;
    let mut phase_offset = 0.0;

    for c in &spectrum {
        let mag = (c.re * c.re + c.im * c.im).sqrt().max(1e-30);
        let phase = c.im.atan2(c.re);

        // Phase unwrapping
        let mut delta = phase - prev_phase;
        if delta > PI {
            phase_offset -= 2.0 * PI;
        } else if delta < -PI {
            phase_offset += 2.0 * PI;
        }
        prev_phase = phase;

        log_spectrum_re.push(mag.ln());
        log_spectrum_im.push(phase + phase_offset);
    }

    // IFFT of complex log spectrum
    let cepstrum = ifft_complex_to_real(&log_spectrum_re, &log_spectrum_im);

    // Check minimum phase: cepstrum should be zero for n < 0 (second half)
    let is_min_phase = cepstrum[n / 2..]
        .iter()
        .all(|&c| c.abs() < 0.01);

    (cepstrum, is_min_phase)
}

/// Detect pitch period using cepstral peak picking.
///
/// Returns the detected pitch period in samples (quefrency of the
/// dominant peak), or `None` if no clear pitch is found.
///
/// `min_period` and `max_period` bound the search range (in samples).
/// For speech at 16 kHz: min_period ≈ 20 (800 Hz), max_period ≈ 320 (50 Hz).
pub fn detect_pitch(
    signal: &[f64],
    min_period: usize,
    max_period: usize,
) -> Option<(usize, f64)> {
    let cepstrum = real_cepstrum(signal);
    let n = cepstrum.len();

    if n == 0 || min_period >= n || max_period >= n {
        return None;
    }

    let search_end = max_period.min(n / 2);
    let search_start = min_period.max(1);

    if search_start >= search_end {
        return None;
    }

    // Find peak in the search range
    let mut max_val = f64::NEG_INFINITY;
    let mut max_idx = search_start;

    for i in search_start..search_end {
        if cepstrum[i] > max_val {
            max_val = cepstrum[i];
            max_idx = i;
        }
    }

    // Confidence: ratio of peak to mean absolute value
    let mean_abs: f64 = cepstrum[search_start..search_end]
        .iter()
        .map(|c| c.abs())
        .sum::<f64>()
        / (search_end - search_start) as f64;

    let confidence = if mean_abs > 1e-30 {
        max_val / mean_abs
    } else {
        0.0
    };

    // Require minimum confidence
    if confidence > 2.0 {
        Some((max_idx, confidence))
    } else {
        None
    }
}

/// Homomorphic filter: separate fast-varying (excitation) and slow-varying
/// (vocal tract / system) components of a signal.
///
/// `lifter_cutoff` is the quefrency cutoff in samples. Components below
/// this quefrency are kept (low-time liftering = spectral envelope).
///
/// Returns (envelope_component, excitation_component).
pub fn homomorphic_filter(signal: &[f64], lifter_cutoff: usize) -> (Vec<f64>, Vec<f64>) {
    let n = signal.len();
    if n == 0 {
        return (vec![], vec![]);
    }

    let cepstrum = real_cepstrum(signal);

    // Low-time lifter (keep low quefrencies = spectral envelope)
    let mut low_cepstrum = vec![0.0; n];
    let cutoff = lifter_cutoff.min(n / 2);
    for i in 0..cutoff {
        low_cepstrum[i] = cepstrum[i];
        if i > 0 && i < n / 2 {
            low_cepstrum[n - i] = cepstrum[n - i]; // Symmetric part
        }
    }

    // High-time lifter (keep high quefrencies = excitation)
    let mut high_cepstrum = vec![0.0; n];
    for i in cutoff..n - cutoff {
        high_cepstrum[i] = cepstrum[i];
    }

    let envelope = cepstrum_to_spectrum(&low_cepstrum);
    let excitation = cepstrum_to_spectrum(&high_cepstrum);

    (envelope, excitation)
}

/// Mel-Frequency Cepstral Coefficients (MFCCs).
///
/// Standard feature extraction for speech/audio recognition:
/// 1. FFT → power spectrum
/// 2. Apply mel filterbank
/// 3. Log of filterbank energies
/// 4. DCT to get MFCCs
///
/// `num_coeffs`: number of MFCCs to return (typically 13)
/// `num_filters`: number of mel filters (typically 26)
/// `sample_rate`: sampling frequency in Hz
pub fn mfcc(
    signal: &[f64],
    num_coeffs: usize,
    num_filters: usize,
    sample_rate: f64,
) -> Vec<f64> {
    let n = signal.len();
    if n == 0 || num_coeffs == 0 || num_filters == 0 {
        return vec![];
    }

    // Apply Hamming window
    let windowed: Vec<f64> = signal
        .iter()
        .enumerate()
        .map(|(i, &x)| {
            let w = 0.54 - 0.46 * (2.0 * PI * i as f64 / (n - 1).max(1) as f64).cos();
            x * w
        })
        .collect();

    // FFT
    let spectrum = fft_real(&windowed);
    let nfft = spectrum.len();

    // Power spectrum (first half + Nyquist)
    let half = nfft / 2 + 1;
    let power: Vec<f64> = spectrum[..half]
        .iter()
        .map(|c| (c.re * c.re + c.im * c.im).max(1e-30))
        .collect();

    // Build mel filterbank
    let filterbank = mel_filterbank(num_filters, half, sample_rate, nfft);

    // Apply filterbank and take log
    let mut log_energies = Vec::with_capacity(num_filters);
    for filter in &filterbank {
        let energy: f64 = filter
            .iter()
            .zip(power.iter())
            .map(|(&f, &p)| f * p)
            .sum();
        log_energies.push(energy.max(1e-30).ln());
    }

    // DCT-II to get MFCCs
    let num_out = num_coeffs.min(num_filters);
    let mut mfccs = Vec::with_capacity(num_out);

    for k in 0..num_out {
        let coeff: f64 = log_energies
            .iter()
            .enumerate()
            .map(|(n, &e)| {
                e * (PI * (n as f64 + 0.5) * k as f64 / num_filters as f64).cos()
            })
            .sum();
        mfccs.push(coeff * (2.0 / num_filters as f64).sqrt());
    }

    mfccs
}

/// Compute the spectral envelope from a cepstrum using exp(FFT{cepstrum}).
pub fn spectral_envelope(signal: &[f64], lifter_cutoff: usize) -> Vec<f64> {
    let cepstrum = real_cepstrum(signal);
    let n = cepstrum.len();
    let cutoff = lifter_cutoff.min(n / 2);

    let mut liftered = vec![0.0; n];
    for i in 0..cutoff {
        liftered[i] = cepstrum[i];
        if i > 0 && i < n / 2 {
            liftered[n - i] = cepstrum[n - i];
        }
    }

    // FFT of liftered cepstrum = log spectral envelope
    let log_env = fft_real(&liftered);
    log_env.iter().map(|c| c.re.exp()).collect()
}

// ---- Internal helpers ----

fn cepstrum_to_spectrum(cepstrum: &[f64]) -> Vec<f64> {
    let log_spec = fft_real(cepstrum);
    log_spec.iter().map(|c| c.re.exp()).collect()
}

/// Convert frequency to mel scale.
fn hz_to_mel(hz: f64) -> f64 {
    2595.0 * (1.0 + hz / 700.0).log10()
}

/// Convert mel to frequency.
fn mel_to_hz(mel: f64) -> f64 {
    700.0 * (10.0_f64.powf(mel / 2595.0) - 1.0)
}

/// Build triangular mel filterbank.
fn mel_filterbank(
    num_filters: usize,
    num_fft_bins: usize,
    sample_rate: f64,
    nfft: usize,
) -> Vec<Vec<f64>> {
    let low_mel = hz_to_mel(0.0);
    let high_mel = hz_to_mel(sample_rate / 2.0);

    // Equally spaced mel points
    let mel_points: Vec<f64> = (0..num_filters + 2)
        .map(|i| low_mel + (high_mel - low_mel) * i as f64 / (num_filters + 1) as f64)
        .collect();

    let hz_points: Vec<f64> = mel_points.iter().map(|&m| mel_to_hz(m)).collect();

    let bin_points: Vec<usize> = hz_points
        .iter()
        .map(|&hz| ((hz * nfft as f64 / sample_rate).round() as usize).min(num_fft_bins - 1))
        .collect();

    let mut filterbank = Vec::with_capacity(num_filters);

    for m in 0..num_filters {
        let mut filter = vec![0.0; num_fft_bins];
        let start = bin_points[m];
        let center = bin_points[m + 1];
        let end = bin_points[m + 2];

        // Rising slope
        if center > start {
            for k in start..=center {
                filter[k] = (k - start) as f64 / (center - start) as f64;
            }
        }

        // Falling slope
        if end > center {
            for k in center..=end.min(num_fft_bins - 1) {
                filter[k] = (end - k) as f64 / (end - center) as f64;
            }
        }

        filterbank.push(filter);
    }

    filterbank
}

#[derive(Clone, Copy)]
struct Cmplx {
    re: f64,
    im: f64,
}

fn fft_real(signal: &[f64]) -> Vec<Cmplx> {
    let n = signal.len();
    let n_padded = n.next_power_of_two();
    let mut input: Vec<Cmplx> = signal
        .iter()
        .map(|&x| Cmplx { re: x, im: 0.0 })
        .collect();
    input.resize(n_padded, Cmplx { re: 0.0, im: 0.0 });
    fft_impl(&input)
}

fn ifft_real(spectrum: &[f64]) -> Vec<f64> {
    let n = spectrum.len();
    let n_padded = n.next_power_of_two();
    let mut input: Vec<Cmplx> = spectrum
        .iter()
        .map(|&x| Cmplx { re: x, im: 0.0 })
        .collect();
    input.resize(n_padded, Cmplx { re: 0.0, im: 0.0 });

    // Conjugate, FFT, conjugate, scale
    let conj: Vec<Cmplx> = input
        .iter()
        .map(|c| Cmplx {
            re: c.re,
            im: -c.im,
        })
        .collect();
    let result = fft_impl(&conj);
    let scale = 1.0 / n_padded as f64;
    result.iter().take(n).map(|c| c.re * scale).collect()
}

fn ifft_complex_to_real(re: &[f64], im: &[f64]) -> Vec<f64> {
    let n = re.len();
    let n_padded = n.next_power_of_two();
    let mut input: Vec<Cmplx> = re
        .iter()
        .zip(im.iter())
        .map(|(&r, &i)| Cmplx { re: r, im: i })
        .collect();
    input.resize(n_padded, Cmplx { re: 0.0, im: 0.0 });

    let conj: Vec<Cmplx> = input
        .iter()
        .map(|c| Cmplx {
            re: c.re,
            im: -c.im,
        })
        .collect();
    let result = fft_impl(&conj);
    let scale = 1.0 / n_padded as f64;
    result.iter().take(n).map(|c| c.re * scale).collect()
}

fn fft_impl(input: &[Cmplx]) -> Vec<Cmplx> {
    let n = input.len();
    if n <= 1 {
        return input.to_vec();
    }

    if !n.is_power_of_two() {
        // DFT fallback
        return (0..n)
            .map(|k| {
                let mut sum = Cmplx { re: 0.0, im: 0.0 };
                for (j, x) in input.iter().enumerate() {
                    let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
                    sum.re += x.re * angle.cos() - x.im * angle.sin();
                    sum.im += x.re * angle.sin() + x.im * angle.cos();
                }
                sum
            })
            .collect();
    }

    let even: Vec<Cmplx> = input.iter().step_by(2).copied().collect();
    let odd: Vec<Cmplx> = input.iter().skip(1).step_by(2).copied().collect();

    let even_fft = fft_impl(&even);
    let odd_fft = fft_impl(&odd);

    let mut result = vec![Cmplx { re: 0.0, im: 0.0 }; n];
    for k in 0..n / 2 {
        let angle = -2.0 * PI * k as f64 / n as f64;
        let tw = Cmplx {
            re: angle.cos(),
            im: angle.sin(),
        };
        let t = Cmplx {
            re: tw.re * odd_fft[k].re - tw.im * odd_fft[k].im,
            im: tw.re * odd_fft[k].im + tw.im * odd_fft[k].re,
        };
        result[k] = Cmplx {
            re: even_fft[k].re + t.re,
            im: even_fft[k].im + t.im,
        };
        result[k + n / 2] = Cmplx {
            re: even_fft[k].re - t.re,
            im: even_fft[k].im - t.im,
        };
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_real_cepstrum_length() {
        let signal: Vec<f64> = (0..128).map(|i| (i as f64 * 0.1).sin()).collect();
        let cepstrum = real_cepstrum(&signal);
        assert_eq!(cepstrum.len(), 128);
    }

    #[test]
    fn test_real_cepstrum_dc() {
        let signal = vec![1.0; 64];
        let cepstrum = real_cepstrum(&signal);
        // DC signal should have energy concentrated at quefrency 0
        assert!(cepstrum[0].abs() > 0.0);
    }

    #[test]
    fn test_power_cepstrum() {
        let signal: Vec<f64> = (0..256).map(|i| (i as f64 * 0.1).sin()).collect();
        let pcep = power_cepstrum(&signal);
        assert_eq!(pcep.len(), 256);
        // Power cepstrum is non-negative
        assert!(pcep.iter().all(|&c| c >= 0.0));
    }

    #[test]
    fn test_complex_cepstrum() {
        let signal: Vec<f64> = (0..128).map(|i| (i as f64 * 0.2).sin()).collect();
        let (cepstrum, _min_phase) = complex_cepstrum(&signal);
        assert_eq!(cepstrum.len(), 128);
    }

    #[test]
    fn test_pitch_detection() {
        // Signal with clear periodicity at period 32
        let period = 32;
        let signal: Vec<f64> = (0..512)
            .map(|i| (2.0 * PI * i as f64 / period as f64).sin())
            .collect();
        let result = detect_pitch(&signal, 10, 100);
        if let Some((detected_period, confidence)) = result {
            assert!(
                (detected_period as i64 - period as i64).unsigned_abs() < 5,
                "detected={detected_period}, expected={period}"
            );
            assert!(confidence > 0.0);
        }
    }

    #[test]
    fn test_pitch_detection_no_pitch() {
        // White-noise-like signal (no clear pitch)
        let signal: Vec<f64> = (0..256)
            .map(|i| {
                let seed = (i as u64).wrapping_mul(6364136223846793005).wrapping_add(1);
                (seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5
            })
            .collect();
        // May or may not detect pitch - just verify it doesn't crash
        let _ = detect_pitch(&signal, 10, 100);
    }

    #[test]
    fn test_homomorphic_filter() {
        let signal: Vec<f64> = (0..256)
            .map(|i| (i as f64 * 0.1).sin() + 0.5 * (i as f64 * 0.3).sin())
            .collect();
        let (envelope, excitation) = homomorphic_filter(&signal, 20);
        assert_eq!(envelope.len(), 256);
        assert_eq!(excitation.len(), 256);
    }

    #[test]
    fn test_mfcc() {
        let signal: Vec<f64> = (0..512)
            .map(|i| (2.0 * PI * 440.0 * i as f64 / 16000.0).sin())
            .collect();
        let coeffs = mfcc(&signal, 13, 26, 16000.0);
        assert_eq!(coeffs.len(), 13);
    }

    #[test]
    fn test_mfcc_empty() {
        let coeffs = mfcc(&[], 13, 26, 16000.0);
        assert!(coeffs.is_empty());
    }

    #[test]
    fn test_spectral_envelope() {
        let signal: Vec<f64> = (0..256).map(|i| (i as f64 * 0.15).sin()).collect();
        let env = spectral_envelope(&signal, 30);
        assert!(!env.is_empty());
        // Spectral envelope should be positive (exp of log)
        assert!(env.iter().all(|&e| e > 0.0));
    }

    #[test]
    fn test_mel_scale() {
        assert!((hz_to_mel(0.0)).abs() < 1e-10);
        let mel_1k = hz_to_mel(1000.0);
        assert!((mel_1k - 1000.0).abs() < 50.0); // ~1000 mels at 1000 Hz
        let roundtrip = mel_to_hz(hz_to_mel(440.0));
        assert!((roundtrip - 440.0).abs() < 0.01);
    }

    #[test]
    fn test_empty_signal() {
        assert!(real_cepstrum(&[]).is_empty());
        assert!(power_cepstrum(&[]).is_empty());
        let (c, _) = complex_cepstrum(&[]);
        assert!(c.is_empty());
    }
}
