//! Power Spectral Density estimation using the Welch method and periodogram.
//!
//! This module provides PSD estimation via averaged, windowed periodograms
//! (Welch's method). It supports configurable FFT sizes, window functions,
//! overlap fractions, and averaging modes. All computations use only the
//! standard library -- no external crates are required.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::periodogram_psd::{Periodogram, PeriodogramConfig, WindowType, welch_psd};
//!
//! // Create a 1 kHz tone sampled at 8 kHz
//! let sample_rate = 8000.0;
//! let n = 2048;
//! let signal: Vec<f64> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / sample_rate;
//!         (2.0 * std::f64::consts::PI * 1000.0 * t).sin()
//!     })
//!     .collect();
//!
//! // Compute PSD via convenience function
//! let (freqs, psd_db) = welch_psd(&signal, 256, 0.5, sample_rate);
//! assert_eq!(freqs.len(), 129); // fft_size/2 + 1 for real signals
//! assert_eq!(psd_db.len(), 129);
//!
//! // Or use the struct API with more control
//! let config = PeriodogramConfig {
//!     fft_size: 256,
//!     window: WindowType::Hann,
//!     overlap_fraction: 0.5,
//!     average_count: 0, // auto
//! };
//! let mut psd = Periodogram::new(config);
//! let result = psd.estimate(&signal);
//! assert_eq!(result.len(), 129);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Window function types for spectral analysis.
#[derive(Debug, Clone, PartialEq)]
pub enum WindowType {
    /// No windowing (all coefficients = 1.0).
    Rectangular,
    /// Hann (raised-cosine) window -- good general-purpose choice.
    Hann,
    /// Hamming window -- slightly less sidelobe suppression than Hann but
    /// narrower main lobe.
    Hamming,
    /// Blackman window -- excellent sidelobe suppression at the cost of a
    /// wider main lobe.
    Blackman,
    /// Kaiser window with shape parameter beta.
    /// Higher beta gives more sidelobe suppression.
    Kaiser(f64),
}

/// Averaging mode for multiple periodogram segments.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AveragingMode {
    /// Linear (arithmetic) average of all segments.
    Linear,
    /// Exponential moving average with the given smoothing factor alpha
    /// in (0, 1]. alpha = 1.0 means no smoothing (latest segment only).
    Exponential(f64),
}

/// Configuration for the [`Periodogram`] PSD estimator.
#[derive(Debug, Clone)]
pub struct PeriodogramConfig {
    /// FFT size (number of points per segment). Must be a power of two.
    /// If not a power of two it will be rounded up.
    pub fft_size: usize,
    /// Window function applied to each segment.
    pub window: WindowType,
    /// Overlap fraction between consecutive segments, in `[0.0, 1.0)`.
    /// Common choices: 0.0 (no overlap) or 0.5 (50 %).
    pub overlap_fraction: f64,
    /// Maximum number of segments to average. Set to 0 for "use all
    /// available segments" (the default).
    pub average_count: usize,
}

impl Default for PeriodogramConfig {
    fn default() -> Self {
        Self {
            fft_size: 256,
            window: WindowType::Hann,
            overlap_fraction: 0.5,
            average_count: 0,
        }
    }
}

// ---------------------------------------------------------------------------
// Periodogram estimator
// ---------------------------------------------------------------------------

/// Welch periodogram-based PSD estimator.
///
/// Computes power spectral density in dB/Hz by dividing the input signal into
/// overlapping segments, applying a window, computing the FFT, and averaging
/// the squared magnitudes.
pub struct Periodogram {
    fft_size: usize,
    window_coeffs: Vec<f64>,
    window_power: f64,
    overlap_fraction: f64,
    average_count: usize,
    averaging_mode: AveragingMode,
}

impl Periodogram {
    /// Create a new periodogram estimator from the given configuration.
    pub fn new(config: PeriodogramConfig) -> Self {
        let fft_size = config.fft_size.max(4).next_power_of_two();
        let window_coeffs = generate_window(&config.window, fft_size);
        let window_power: f64 = window_coeffs.iter().map(|w| w * w).sum();

        Self {
            fft_size,
            window_coeffs,
            window_power,
            overlap_fraction: config.overlap_fraction.clamp(0.0, 0.99),
            average_count: config.average_count,
            averaging_mode: AveragingMode::Linear,
        }
    }

    /// Set the averaging mode (default is [`AveragingMode::Linear`]).
    pub fn set_averaging_mode(&mut self, mode: AveragingMode) {
        self.averaging_mode = mode;
    }

    /// Estimate the one-sided PSD of real-valued samples in dB/Hz.
    ///
    /// Returns `fft_size / 2 + 1` frequency bins covering `[0, fs/2]`.
    /// If `samples` is shorter than `fft_size` it is zero-padded.
    pub fn estimate(&mut self, samples: &[f64]) -> Vec<f64> {
        let n = self.fft_size;
        let overlap = (n as f64 * self.overlap_fraction) as usize;
        let step = (n - overlap).max(1);

        // Determine how many segments we can form
        let max_segs = if samples.len() >= n {
            (samples.len() - n) / step + 1
        } else {
            1 // zero-pad single segment
        };
        let num_segs = if self.average_count > 0 {
            max_segs.min(self.average_count)
        } else {
            max_segs
        };

        let out_len = n / 2 + 1;
        let mut psd_accum = vec![0.0f64; out_len];

        match self.averaging_mode {
            AveragingMode::Linear => {
                for seg_idx in 0..num_segs {
                    let offset = seg_idx * step;
                    let segment = self.extract_windowed_segment_real(samples, offset);
                    let spectrum = fft_forward(&segment);
                    for i in 0..out_len {
                        let mag_sq = spectrum[i].0 * spectrum[i].0
                            + spectrum[i].1 * spectrum[i].1;
                        psd_accum[i] += mag_sq;
                    }
                }
                // Normalise: divide by num_segs and window power
                let scale = 1.0 / (self.window_power * num_segs as f64);
                for i in 0..out_len {
                    let mut val = psd_accum[i] * scale;
                    // Double non-DC, non-Nyquist bins for one-sided spectrum
                    if i > 0 && i < n / 2 {
                        val *= 2.0;
                    }
                    psd_accum[i] = if val > 1e-30 {
                        10.0 * val.log10()
                    } else {
                        -300.0
                    };
                }
            }
            AveragingMode::Exponential(alpha) => {
                let alpha = alpha.clamp(0.01, 1.0);
                let mut first = true;
                for seg_idx in 0..num_segs {
                    let offset = seg_idx * step;
                    let segment = self.extract_windowed_segment_real(samples, offset);
                    let spectrum = fft_forward(&segment);
                    let scale = 1.0 / self.window_power;
                    for i in 0..out_len {
                        let mag_sq = spectrum[i].0 * spectrum[i].0
                            + spectrum[i].1 * spectrum[i].1;
                        let mut val = mag_sq * scale;
                        if i > 0 && i < n / 2 {
                            val *= 2.0;
                        }
                        if first {
                            psd_accum[i] = val;
                        } else {
                            psd_accum[i] = alpha * val + (1.0 - alpha) * psd_accum[i];
                        }
                    }
                    first = false;
                }
                // Convert to dB
                for v in psd_accum.iter_mut() {
                    *v = if *v > 1e-30 {
                        10.0 * v.log10()
                    } else {
                        -300.0
                    };
                }
            }
        }

        psd_accum
    }

    /// Estimate the two-sided PSD of complex IQ samples in dB/Hz.
    ///
    /// Returns `fft_size` frequency bins covering `[-fs/2, fs/2)` (not shifted;
    /// bin 0 = DC, bins wrap at Nyquist).
    pub fn estimate_complex(&mut self, samples: &[(f64, f64)]) -> Vec<f64> {
        let n = self.fft_size;
        let overlap = (n as f64 * self.overlap_fraction) as usize;
        let step = (n - overlap).max(1);

        let max_segs = if samples.len() >= n {
            (samples.len() - n) / step + 1
        } else {
            1
        };
        let num_segs = if self.average_count > 0 {
            max_segs.min(self.average_count)
        } else {
            max_segs
        };

        let mut psd_accum = vec![0.0f64; n];

        for seg_idx in 0..num_segs {
            let offset = seg_idx * step;
            let segment = self.extract_windowed_segment_complex(samples, offset);
            let spectrum = fft_forward(&segment);
            for i in 0..n {
                let mag_sq =
                    spectrum[i].0 * spectrum[i].0 + spectrum[i].1 * spectrum[i].1;
                psd_accum[i] += mag_sq;
            }
        }

        let scale = 1.0 / (self.window_power * num_segs as f64);
        for v in psd_accum.iter_mut() {
            let val = *v * scale;
            *v = if val > 1e-30 {
                10.0 * val.log10()
            } else {
                -300.0
            };
        }

        psd_accum
    }

    /// Return the frequency axis for the one-sided real PSD estimate.
    ///
    /// `sample_rate` is in Hz. Returns `fft_size / 2 + 1` values from 0 to
    /// `sample_rate / 2`.
    pub fn frequency_axis(&self, sample_rate: f64) -> Vec<f64> {
        let out_len = self.fft_size / 2 + 1;
        (0..out_len)
            .map(|i| i as f64 * sample_rate / self.fft_size as f64)
            .collect()
    }

    /// Return the frequency axis for the two-sided complex PSD estimate.
    ///
    /// Returns `fft_size` values. Bin 0 = DC.
    pub fn frequency_axis_complex(&self, sample_rate: f64) -> Vec<f64> {
        let n = self.fft_size;
        (0..n)
            .map(|i| {
                if i <= n / 2 {
                    i as f64 * sample_rate / n as f64
                } else {
                    (i as f64 - n as f64) * sample_rate / n as f64
                }
            })
            .collect()
    }

    /// Get the FFT size (may differ from the requested size if it was rounded
    /// up to a power of two).
    pub fn fft_size(&self) -> usize {
        self.fft_size
    }

    // -- internal helpers ---------------------------------------------------

    fn extract_windowed_segment_real(
        &self,
        samples: &[f64],
        offset: usize,
    ) -> Vec<(f64, f64)> {
        let n = self.fft_size;
        let mut buf = vec![(0.0, 0.0); n];
        for i in 0..n {
            let idx = offset + i;
            let sample = if idx < samples.len() {
                samples[idx]
            } else {
                0.0
            };
            buf[i] = (sample * self.window_coeffs[i], 0.0);
        }
        buf
    }

    fn extract_windowed_segment_complex(
        &self,
        samples: &[(f64, f64)],
        offset: usize,
    ) -> Vec<(f64, f64)> {
        let n = self.fft_size;
        let mut buf = vec![(0.0, 0.0); n];
        for i in 0..n {
            let idx = offset + i;
            let (re, im) = if idx < samples.len() {
                samples[idx]
            } else {
                (0.0, 0.0)
            };
            buf[i] = (re * self.window_coeffs[i], im * self.window_coeffs[i]);
        }
        buf
    }
}

// ---------------------------------------------------------------------------
// Convenience function
// ---------------------------------------------------------------------------

/// Compute a one-sided Welch PSD of real-valued samples.
///
/// Returns `(frequencies, psd_db)` where `frequencies` has `fft_size / 2 + 1`
/// entries in Hz and `psd_db` is the corresponding power spectral density
/// in dB/Hz.
///
/// This is a thin wrapper around [`Periodogram`] with a Hann window.
pub fn welch_psd(
    samples: &[f64],
    fft_size: usize,
    overlap: f64,
    sample_rate: f64,
) -> (Vec<f64>, Vec<f64>) {
    let config = PeriodogramConfig {
        fft_size,
        window: WindowType::Hann,
        overlap_fraction: overlap,
        average_count: 0,
    };
    let mut p = Periodogram::new(config);
    let psd_db = p.estimate(samples);
    let freqs = p.frequency_axis(sample_rate);
    (freqs, psd_db)
}

// ---------------------------------------------------------------------------
// Window functions
// ---------------------------------------------------------------------------

/// Apply a window function **in place** to a slice of real-valued samples.
pub fn apply_window(samples: &mut [f64], window: &WindowType) {
    let coeffs = generate_window(window, samples.len());
    for (s, w) in samples.iter_mut().zip(coeffs.iter()) {
        *s *= w;
    }
}

/// Compute the coherent gain (sum of window coefficients divided by the
/// window length) for the given window type and size.
///
/// The coherent gain indicates the amplitude scaling introduced by the window
/// at DC and is used for signal power normalisation.
pub fn coherent_gain(window: &WindowType, size: usize) -> f64 {
    if size == 0 {
        return 0.0;
    }
    let coeffs = generate_window(window, size);
    coeffs.iter().sum::<f64>() / size as f64
}

/// Generate window coefficients for the given type and size.
fn generate_window(window: &WindowType, size: usize) -> Vec<f64> {
    if size == 0 {
        return Vec::new();
    }
    (0..size)
        .map(|i| {
            let x = i as f64 / size as f64;
            match window {
                WindowType::Rectangular => 1.0,
                WindowType::Hann => 0.5 * (1.0 - (2.0 * PI * x).cos()),
                WindowType::Hamming => 0.54 - 0.46 * (2.0 * PI * x).cos(),
                WindowType::Blackman => {
                    0.42 - 0.5 * (2.0 * PI * x).cos() + 0.08 * (4.0 * PI * x).cos()
                }
                WindowType::Kaiser(beta) => {
                    // Kaiser window: w(n) = I0(beta * sqrt(1 - ((2n/N)-1)^2)) / I0(beta)
                    let m = size as f64;
                    let t = 2.0 * i as f64 / m - 1.0;
                    let arg = beta * (1.0 - t * t).max(0.0).sqrt();
                    bessel_i0(arg) / bessel_i0(*beta)
                }
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------
// FFT -- Cooley-Tukey radix-2 decimation-in-time
// ---------------------------------------------------------------------------

/// In-place Cooley-Tukey radix-2 DIT FFT.
///
/// `data` must have power-of-two length. Each element is `(re, im)`.
fn fft_forward(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = input.len();
    assert!(n.is_power_of_two(), "FFT size must be a power of two");

    let mut data: Vec<(f64, f64)> = input.to_vec();

    if n <= 1 {
        return data;
    }

    // Bit-reversal permutation
    let bits = n.trailing_zeros();
    for i in 0..n {
        let j = i.reverse_bits() >> (usize::BITS - bits);
        if i < j {
            data.swap(i, j);
        }
    }

    // Butterfly stages
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let w_step = -2.0 * PI / len as f64;
        for start in (0..n).step_by(len) {
            for k in 0..half {
                let angle = w_step * k as f64;
                let (tw_re, tw_im) = (angle.cos(), angle.sin());
                let (u_re, u_im) = data[start + k];
                let (v_re, v_im) = data[start + k + half];
                // twiddle * v
                let tv_re = tw_re * v_re - tw_im * v_im;
                let tv_im = tw_re * v_im + tw_im * v_re;
                data[start + k] = (u_re + tv_re, u_im + tv_im);
                data[start + k + half] = (u_re - tv_re, u_im - tv_im);
            }
        }
        len *= 2;
    }

    data
}

// ---------------------------------------------------------------------------
// Bessel I0 approximation (for Kaiser window)
// ---------------------------------------------------------------------------

/// Modified Bessel function of the first kind, order zero.
///
/// Uses the polynomial series expansion which converges rapidly for moderate
/// arguments.
fn bessel_i0(x: f64) -> f64 {
    let mut sum = 1.0;
    let mut term = 1.0;
    let x_half = x / 2.0;
    for k in 1..30 {
        term *= (x_half / k as f64) * (x_half / k as f64);
        sum += term;
        if term < 1e-15 * sum {
            break;
        }
    }
    sum
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a real sine tone at a given normalised frequency.
    fn make_tone(n: usize, freq_hz: f64, sample_rate: f64) -> Vec<f64> {
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                (2.0 * PI * freq_hz * t).sin()
            })
            .collect()
    }

    /// Helper: generate complex IQ tone.
    fn make_complex_tone(n: usize, freq_hz: f64, sample_rate: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * PI * freq_hz * t;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    /// Deterministic pseudo-noise for testing variance.
    fn make_pseudo_noise(n: usize) -> Vec<f64> {
        let mut out = Vec::with_capacity(n);
        let mut state: u64 = 0xDEAD_BEEF_CAFE_BABE;
        for _ in 0..n {
            // xorshift64
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            // Map to [-1, 1)
            let val = (state as f64) / (u64::MAX as f64) * 2.0 - 1.0;
            out.push(val);
        }
        out
    }

    // -- Test 1: Construction with defaults ---------------------------------

    #[test]
    fn test_construction_defaults() {
        let config = PeriodogramConfig::default();
        assert_eq!(config.fft_size, 256);
        assert_eq!(config.window, WindowType::Hann);
        assert!((config.overlap_fraction - 0.5).abs() < 1e-10);
        assert_eq!(config.average_count, 0);

        let p = Periodogram::new(config);
        assert_eq!(p.fft_size(), 256);
        assert_eq!(p.window_coeffs.len(), 256);
        assert!(p.window_power > 0.0);
    }

    // -- Test 2: Rectangular window periodogram of pure tone ----------------

    #[test]
    fn test_rectangular_tone_peak_at_correct_bin() {
        let sample_rate = 1024.0;
        let fft_size = 256;
        let tone_freq = 100.0; // Bin = 100 * 256 / 1024 = 25

        let signal = make_tone(2048, tone_freq, sample_rate);

        let config = PeriodogramConfig {
            fft_size,
            window: WindowType::Rectangular,
            overlap_fraction: 0.0,
            average_count: 0,
        };
        let mut p = Periodogram::new(config);
        let psd = p.estimate(&signal);

        // Find peak bin
        let peak_bin = psd
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        let expected_bin = (tone_freq * fft_size as f64 / sample_rate).round() as usize;
        assert_eq!(
            peak_bin, expected_bin,
            "Peak at bin {} but expected {}",
            peak_bin, expected_bin
        );
    }

    // -- Test 3: Hann window reduces spectral leakage -----------------------

    #[test]
    fn test_hann_reduces_leakage() {
        let sample_rate = 256.0;
        let fft_size = 256;
        // Frequency that falls between bins (worst case for leakage)
        let tone_freq = 30.5;
        let signal = make_tone(2048, tone_freq, sample_rate);

        // Rectangular window
        let config_rect = PeriodogramConfig {
            fft_size,
            window: WindowType::Rectangular,
            overlap_fraction: 0.5,
            average_count: 0,
        };
        let mut p_rect = Periodogram::new(config_rect);
        let psd_rect = p_rect.estimate(&signal);

        // Hann window
        let config_hann = PeriodogramConfig {
            fft_size,
            window: WindowType::Hann,
            overlap_fraction: 0.5,
            average_count: 0,
        };
        let mut p_hann = Periodogram::new(config_hann);
        let psd_hann = p_hann.estimate(&signal);

        // Measure leakage: sum of power far from the peak
        let peak_bin = 31; // closest bin to 30.5
        let leakage_rect: f64 = psd_rect
            .iter()
            .enumerate()
            .filter(|(i, _)| (*i as i64 - peak_bin as i64).unsigned_abs() > 5)
            .map(|(_, &v)| {
                // Convert from dB to linear for fair summation
                10.0f64.powf(v / 10.0)
            })
            .sum();

        let leakage_hann: f64 = psd_hann
            .iter()
            .enumerate()
            .filter(|(i, _)| (*i as i64 - peak_bin as i64).unsigned_abs() > 5)
            .map(|(_, &v)| 10.0f64.powf(v / 10.0))
            .sum();

        assert!(
            leakage_hann < leakage_rect,
            "Hann leakage ({:.2e}) should be less than Rectangular ({:.2e})",
            leakage_hann,
            leakage_rect
        );
    }

    // -- Test 4: Welch method reduces variance vs single periodogram --------

    #[test]
    fn test_welch_reduces_variance() {
        let signal = make_pseudo_noise(8192);

        // Single periodogram (no overlap, 1 segment of full length)
        let config_single = PeriodogramConfig {
            fft_size: 512,
            window: WindowType::Hann,
            overlap_fraction: 0.0,
            average_count: 1,
        };
        let mut p_single = Periodogram::new(config_single);
        let psd_single = p_single.estimate(&signal);

        // Welch with many averaged segments
        let config_welch = PeriodogramConfig {
            fft_size: 512,
            window: WindowType::Hann,
            overlap_fraction: 0.5,
            average_count: 0,
        };
        let mut p_welch = Periodogram::new(config_welch);
        let psd_welch = p_welch.estimate(&signal);

        // Compute variance of each PSD estimate
        let mean_single = psd_single.iter().sum::<f64>() / psd_single.len() as f64;
        let var_single: f64 = psd_single
            .iter()
            .map(|&v| (v - mean_single).powi(2))
            .sum::<f64>()
            / psd_single.len() as f64;

        let mean_welch = psd_welch.iter().sum::<f64>() / psd_welch.len() as f64;
        let var_welch: f64 = psd_welch
            .iter()
            .map(|&v| (v - mean_welch).powi(2))
            .sum::<f64>()
            / psd_welch.len() as f64;

        assert!(
            var_welch < var_single,
            "Welch variance ({:.4}) should be less than single ({:.4})",
            var_welch,
            var_single
        );
    }

    // -- Test 5: Frequency axis correctness ---------------------------------

    #[test]
    fn test_frequency_axis_correctness() {
        let fft_size = 128;
        let sample_rate = 48000.0;

        let config = PeriodogramConfig {
            fft_size,
            ..Default::default()
        };
        let p = Periodogram::new(config);
        let freqs = p.frequency_axis(sample_rate);

        // One-sided: fft_size/2 + 1 bins
        assert_eq!(freqs.len(), fft_size / 2 + 1);

        // First bin is 0 Hz (DC)
        assert!((freqs[0] - 0.0).abs() < 1e-10, "DC bin should be 0 Hz");

        // Last bin is Nyquist (fs/2)
        let nyquist = sample_rate / 2.0;
        assert!(
            (freqs[freqs.len() - 1] - nyquist).abs() < 1e-6,
            "Last bin should be Nyquist ({}), got {}",
            nyquist,
            freqs[freqs.len() - 1]
        );

        // Frequency resolution
        let expected_resolution = sample_rate / fft_size as f64;
        let actual_resolution = freqs[1] - freqs[0];
        assert!(
            (actual_resolution - expected_resolution).abs() < 1e-6,
            "Resolution should be {}, got {}",
            expected_resolution,
            actual_resolution
        );
    }

    // -- Test 6: Complex signal PSD -----------------------------------------

    #[test]
    fn test_complex_signal_psd() {
        let sample_rate = 1024.0;
        let fft_size = 256;
        let tone_freq = 200.0; // Bin = 200 * 256 / 1024 = 50

        let signal = make_complex_tone(2048, tone_freq, sample_rate);

        let config = PeriodogramConfig {
            fft_size,
            window: WindowType::Rectangular,
            overlap_fraction: 0.0,
            average_count: 0,
        };
        let mut p = Periodogram::new(config);
        let psd = p.estimate_complex(&signal);

        // Complex PSD returns fft_size bins
        assert_eq!(psd.len(), fft_size);

        // Peak should be at bin 50
        let peak_bin = psd
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        let expected_bin = (tone_freq * fft_size as f64 / sample_rate).round() as usize;
        assert_eq!(
            peak_bin, expected_bin,
            "Complex PSD peak at bin {} but expected {}",
            peak_bin, expected_bin
        );
    }

    // -- Test 7: Overlap fraction affects output ----------------------------

    #[test]
    fn test_overlap_fraction_affects_output() {
        let signal = make_pseudo_noise(4096);
        let fft_size = 256;

        // No overlap
        let config_no_overlap = PeriodogramConfig {
            fft_size,
            window: WindowType::Hann,
            overlap_fraction: 0.0,
            average_count: 0,
        };
        let psd_no_overlap = Periodogram::new(config_no_overlap).estimate(&signal);

        // 75 % overlap (more segments -> smoother estimate)
        let config_75 = PeriodogramConfig {
            fft_size,
            window: WindowType::Hann,
            overlap_fraction: 0.75,
            average_count: 0,
        };
        let psd_75 = Periodogram::new(config_75).estimate(&signal);

        // Both should be the same length
        assert_eq!(psd_no_overlap.len(), psd_75.len());

        // But the values should differ (different number of averaged segments)
        let differ = psd_no_overlap
            .iter()
            .zip(psd_75.iter())
            .any(|(a, b)| (a - b).abs() > 0.01);
        assert!(
            differ,
            "Different overlap fractions should produce different PSD values"
        );
    }

    // -- Test 8: Window coherent gain values --------------------------------

    #[test]
    fn test_window_coherent_gain() {
        let size = 1024;

        // Rectangular: all 1s => coherent gain = 1.0
        let cg_rect = coherent_gain(&WindowType::Rectangular, size);
        assert!(
            (cg_rect - 1.0).abs() < 1e-10,
            "Rectangular CG = {}, expected 1.0",
            cg_rect
        );

        // Hann: mean of 0.5*(1-cos) ~ 0.5
        let cg_hann = coherent_gain(&WindowType::Hann, size);
        assert!(
            (cg_hann - 0.5).abs() < 0.01,
            "Hann CG = {}, expected ~0.5",
            cg_hann
        );

        // Hamming: mean ~ 0.54
        let cg_hamming = coherent_gain(&WindowType::Hamming, size);
        assert!(
            (cg_hamming - 0.54).abs() < 0.01,
            "Hamming CG = {}, expected ~0.54",
            cg_hamming
        );

        // Blackman: mean ~ 0.42
        let cg_blackman = coherent_gain(&WindowType::Blackman, size);
        assert!(
            (cg_blackman - 0.42).abs() < 0.02,
            "Blackman CG = {}, expected ~0.42",
            cg_blackman
        );

        // Kaiser(0) should equal Rectangular
        let cg_kaiser0 = coherent_gain(&WindowType::Kaiser(0.0), size);
        assert!(
            (cg_kaiser0 - 1.0).abs() < 0.01,
            "Kaiser(0) CG = {}, expected ~1.0",
            cg_kaiser0
        );

        // Kaiser with large beta should have smaller gain
        let cg_kaiser10 = coherent_gain(&WindowType::Kaiser(10.0), size);
        assert!(
            cg_kaiser10 < cg_kaiser0,
            "Kaiser(10) CG ({}) should be less than Kaiser(0) ({})",
            cg_kaiser10,
            cg_kaiser0
        );
    }

    // -- Test 9: Zero-padding behavior (fft_size > signal length) -----------

    #[test]
    fn test_zero_padding_behavior() {
        let sample_rate = 256.0;
        let fft_size = 256;
        let tone_freq = 32.0; // Bin 32

        // Signal shorter than FFT size
        let signal = make_tone(64, tone_freq, sample_rate);

        let config = PeriodogramConfig {
            fft_size,
            window: WindowType::Rectangular,
            overlap_fraction: 0.0,
            average_count: 0,
        };
        let mut p = Periodogram::new(config);
        let psd = p.estimate(&signal);

        // Output length is always fft_size/2 + 1
        assert_eq!(psd.len(), fft_size / 2 + 1);

        // Should still detect the tone (zero-padded, so peak is broader but
        // still around the correct bin)
        let peak_bin = psd
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        let expected_bin = (tone_freq * fft_size as f64 / sample_rate).round() as usize;
        assert!(
            (peak_bin as i64 - expected_bin as i64).unsigned_abs() <= 1,
            "Zero-padded peak at bin {} but expected near {}",
            peak_bin,
            expected_bin
        );
    }

    // -- Test 10: Convenience welch_psd function ----------------------------

    #[test]
    fn test_welch_psd_convenience() {
        let sample_rate = 8000.0;
        let fft_size = 256;
        let tone_freq = 1000.0; // Bin = 1000 * 256 / 8000 = 32

        let signal = make_tone(4096, tone_freq, sample_rate);
        let (freqs, psd_db) = welch_psd(&signal, fft_size, 0.5, sample_rate);

        // Check dimensions
        assert_eq!(freqs.len(), fft_size / 2 + 1);
        assert_eq!(psd_db.len(), fft_size / 2 + 1);

        // Check frequency range
        assert!((freqs[0] - 0.0).abs() < 1e-10);
        assert!((freqs[freqs.len() - 1] - sample_rate / 2.0).abs() < 1e-6);

        // Peak should be at the tone frequency
        let peak_idx = psd_db
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        let peak_freq = freqs[peak_idx];
        assert!(
            (peak_freq - tone_freq).abs() < sample_rate / fft_size as f64 * 1.5,
            "Peak frequency {:.1} Hz should be near {:.1} Hz",
            peak_freq,
            tone_freq
        );
    }
}
