//! Time-Frequency Reassignment
//!
//! Advanced time-frequency analysis using the reassignment method for improved
//! localization of signal energy in the time-frequency plane. Unlike a standard
//! spectrogram, which smears energy across the analysis window, reassignment
//! moves each spectrogram coefficient to its center of gravity, yielding a
//! sharper representation.
//!
//! Uses only the standard library with `(f64, f64)` for complex IQ samples.
//!
//! ## Background
//!
//! Given a standard STFT computed with window `h(t)`:
//! - **Time reassignment**: `t_hat = t - Re{ STFT_th(t,f) / STFT_h(t,f) }`
//!   where `th` is the time-ramped window `t * h(t)`.
//! - **Frequency reassignment**: `f_hat = f + Im{ STFT_dh(t,f) / STFT_h(t,f) } / (2*pi)`
//!   where `dh` is the derivative window `h'(t)`.
//!
//! A magnitude threshold avoids reassigning low-energy (noisy) bins.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::time_frequency_reassignment::{ReassignedSpectrogram, TfConfig};
//!
//! let config = TfConfig {
//!     window_size: 64,
//!     hop_size: 32,
//!     sample_rate: 1000.0,
//!     threshold_db: -40.0,
//! };
//! let rs = ReassignedSpectrogram::new(config);
//!
//! // Generate a 200 Hz tone at 1000 Hz sample rate
//! let n = 256;
//! let samples: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / 1000.0;
//!         let phase = 2.0 * std::f64::consts::PI * 200.0 * t;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! let result = rs.compute(&samples);
//! assert!(!result.is_empty());
//! // Each frame should have reassigned points
//! let total_points: usize = result.iter().map(|f| f.len()).sum();
//! assert!(total_points > 0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Configuration for time-frequency reassignment analysis.
#[derive(Debug, Clone)]
pub struct TfConfig {
    /// Analysis window size in samples (will be rounded up to next power of 2).
    pub window_size: usize,
    /// Hop size between successive frames in samples.
    pub hop_size: usize,
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Magnitude threshold in dB below peak; bins below this are not reassigned.
    pub threshold_db: f64,
}

/// A single reassigned time-frequency point.
#[derive(Debug, Clone, Copy)]
pub struct TfPoint {
    /// Reassigned time coordinate in seconds.
    pub time_s: f64,
    /// Reassigned frequency coordinate in Hz.
    pub freq_hz: f64,
    /// Magnitude in dB.
    pub magnitude_db: f64,
}

/// Instantaneous signal parameters at a single sample.
#[derive(Debug, Clone, Copy)]
pub struct InstantaneousParams {
    /// Instantaneous frequency in Hz.
    pub inst_freq_hz: f64,
    /// Instantaneous time (group delay) in seconds.
    pub inst_time_s: f64,
    /// Instantaneous amplitude (magnitude).
    pub inst_amplitude: f64,
}

/// Reassigned spectrogram engine.
///
/// Computes three STFTs in parallel (standard window, time-ramped window,
/// derivative window) and uses the ratios to reassign each bin.
#[derive(Debug, Clone)]
pub struct ReassignedSpectrogram {
    /// Effective FFT / window size (power of 2).
    fft_size: usize,
    /// Hop size.
    hop_size: usize,
    /// Sample rate in Hz.
    sample_rate: f64,
    /// Threshold in dB (negative value below peak).
    threshold_db: f64,
    /// Analysis window h(t) — Hann.
    window_h: Vec<f64>,
    /// Time-ramped window th(t) = t * h(t) (in sample indices centered at 0).
    window_th: Vec<f64>,
    /// Derivative window dh(t) = h'(t).
    window_dh: Vec<f64>,
}

impl ReassignedSpectrogram {
    /// Create a new reassigned spectrogram from the given configuration.
    pub fn new(config: TfConfig) -> Self {
        let fft_size = config.window_size.next_power_of_two().max(4);
        let hop_size = config.hop_size.max(1).min(fft_size);
        let n = fft_size;

        // Hann window
        let window_h: Vec<f64> = (0..n)
            .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos()))
            .collect();

        // Time-ramped window: (i - N/2) * h[i]  (centered time index)
        let window_th: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 - (n as f64 / 2.0);
                t * window_h[i]
            })
            .collect();

        // Derivative of Hann window: h'(t) via analytic derivative
        // h(t) = 0.5 * (1 - cos(2*pi*t/N))
        // h'(t) = 0.5 * (2*pi/N) * sin(2*pi*t/N)  = (pi/N)*sin(2*pi*t/N)
        let window_dh: Vec<f64> = (0..n)
            .map(|i| (PI / n as f64) * (2.0 * PI * i as f64 / n as f64).sin())
            .collect();

        Self {
            fft_size,
            hop_size,
            sample_rate: config.sample_rate,
            threshold_db: config.threshold_db,
            window_h,
            window_th,
            window_dh,
        }
    }

    /// Compute the reassigned spectrogram.
    ///
    /// Returns a `Vec` of frames, where each frame is a `Vec<TfPoint>` of
    /// reassigned time-frequency points that exceeded the energy threshold.
    pub fn compute(&self, samples: &[(f64, f64)]) -> Vec<Vec<TfPoint>> {
        let n = self.fft_size;
        let num_frames = self.num_frames(samples.len());
        let mut result = Vec::with_capacity(num_frames);

        for frame_idx in 0..num_frames {
            let offset = frame_idx * self.hop_size;
            let frame_time = offset as f64 / self.sample_rate;

            // Extract windowed segments for the three windows
            let stft_h = self.windowed_fft(samples, offset, &self.window_h);
            let stft_th = self.windowed_fft(samples, offset, &self.window_th);
            let stft_dh = self.windowed_fft(samples, offset, &self.window_dh);

            // Find peak magnitude in this frame for thresholding
            let peak_mag_sq = stft_h
                .iter()
                .map(|&(re, im)| re * re + im * im)
                .fold(0.0_f64, f64::max);
            let peak_db = if peak_mag_sq > 0.0 {
                10.0 * peak_mag_sq.log10()
            } else {
                -200.0
            };

            let mut frame_points = Vec::new();

            for k in 0..n {
                let (re_h, im_h) = stft_h[k];
                let mag_sq = re_h * re_h + im_h * im_h;

                if mag_sq < 1e-30 {
                    continue;
                }

                let mag_db = 10.0 * mag_sq.log10();
                if mag_db < peak_db + self.threshold_db {
                    continue;
                }

                // STFT_th / STFT_h  (complex division)
                let (re_th, im_th) = stft_th[k];
                let ratio_th = complex_div((re_th, im_th), (re_h, im_h));

                // STFT_dh / STFT_h
                let (re_dh, im_dh) = stft_dh[k];
                let ratio_dh = complex_div((re_dh, im_dh), (re_h, im_h));

                // Time reassignment: t_hat = t - Re{STFT_th / STFT_h} / sample_rate
                let time_correction = ratio_th.0 / self.sample_rate;
                let t_hat = frame_time - time_correction;

                // Frequency reassignment: f_hat = f + Im{STFT_dh / STFT_h} / (2*pi) * sample_rate
                let bin_freq = k as f64 * self.sample_rate / n as f64;
                let freq_correction = ratio_dh.1 * self.sample_rate / (2.0 * PI);
                let f_hat = bin_freq + freq_correction;

                frame_points.push(TfPoint {
                    time_s: t_hat,
                    freq_hz: f_hat,
                    magnitude_db: mag_db,
                });
            }

            result.push(frame_points);
        }

        result
    }

    /// Compute instantaneous frequency for each sample using the analytic
    /// signal approach (finite-difference of unwrapped phase).
    ///
    /// Returns a vector of instantaneous frequencies in Hz, one per sample
    /// (the last sample reuses the previous value).
    pub fn instantaneous_frequency(&self, samples: &[(f64, f64)]) -> Vec<f64> {
        if samples.is_empty() {
            return Vec::new();
        }
        let mut result = Vec::with_capacity(samples.len());
        for i in 0..samples.len() {
            if i == 0 {
                // Use forward difference for first sample
                if samples.len() > 1 {
                    let phase0 = samples[0].1.atan2(samples[0].0);
                    let phase1 = samples[1].1.atan2(samples[1].0);
                    let dp = wrap_phase(phase1 - phase0);
                    result.push(dp * self.sample_rate / (2.0 * PI));
                } else {
                    result.push(0.0);
                }
            } else {
                let phase_prev = samples[i - 1].1.atan2(samples[i - 1].0);
                let phase_curr = samples[i].1.atan2(samples[i].0);
                let dp = wrap_phase(phase_curr - phase_prev);
                result.push(dp * self.sample_rate / (2.0 * PI));
            }
        }
        result
    }

    /// Compute the reassignment vectors (time corrections and frequency
    /// corrections) for each STFT frame and bin.
    ///
    /// Returns `(time_corrections, freq_corrections)` where each is a
    /// `Vec<Vec<f64>>` indexed by `[frame][bin]`.
    /// - `time_corrections[f][k]`: seconds to subtract from nominal frame time.
    /// - `freq_corrections[f][k]`: Hz to add to nominal bin frequency.
    pub fn reassignment_vectors(
        &self,
        samples: &[(f64, f64)],
    ) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
        let n = self.fft_size;
        let num_frames = self.num_frames(samples.len());
        let mut time_corr = Vec::with_capacity(num_frames);
        let mut freq_corr = Vec::with_capacity(num_frames);

        for frame_idx in 0..num_frames {
            let offset = frame_idx * self.hop_size;

            let stft_h = self.windowed_fft(samples, offset, &self.window_h);
            let stft_th = self.windowed_fft(samples, offset, &self.window_th);
            let stft_dh = self.windowed_fft(samples, offset, &self.window_dh);

            let mut t_vec = vec![0.0; n];
            let mut f_vec = vec![0.0; n];

            for k in 0..n {
                let (re_h, im_h) = stft_h[k];
                let mag_sq = re_h * re_h + im_h * im_h;

                if mag_sq < 1e-30 {
                    continue;
                }

                let ratio_th = complex_div(stft_th[k], stft_h[k]);
                let ratio_dh = complex_div(stft_dh[k], stft_h[k]);

                t_vec[k] = ratio_th.0 / self.sample_rate;
                f_vec[k] = ratio_dh.1 * self.sample_rate / (2.0 * PI);
            }

            time_corr.push(t_vec);
            freq_corr.push(f_vec);
        }

        (time_corr, freq_corr)
    }

    /// Compute a standard STFT magnitude spectrogram (no reassignment).
    ///
    /// Returns `Vec<Vec<f64>>` where each inner vec has `fft_size` magnitude
    /// values in dB.
    pub fn spectrogram(&self, samples: &[(f64, f64)]) -> Vec<Vec<f64>> {
        let n = self.fft_size;
        let num_frames = self.num_frames(samples.len());
        let mut result = Vec::with_capacity(num_frames);

        for frame_idx in 0..num_frames {
            let offset = frame_idx * self.hop_size;
            let stft_h = self.windowed_fft(samples, offset, &self.window_h);

            let mag_db: Vec<f64> = stft_h
                .iter()
                .map(|&(re, im)| {
                    let mag_sq = re * re + im * im;
                    if mag_sq > 1e-30 {
                        10.0 * mag_sq.log10()
                    } else {
                        -300.0
                    }
                })
                .collect();

            result.push(mag_db);
        }

        result
    }

    /// Return the FFT size used.
    pub fn fft_size(&self) -> usize {
        self.fft_size
    }

    /// Return the hop size used.
    pub fn hop_size(&self) -> usize {
        self.hop_size
    }

    /// Return the sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /// Number of STFT frames for a given signal length.
    fn num_frames(&self, signal_len: usize) -> usize {
        if signal_len < self.fft_size {
            return 0;
        }
        (signal_len - self.fft_size) / self.hop_size + 1
    }

    /// Apply a window to the signal starting at `offset` and compute the FFT.
    fn windowed_fft(
        &self,
        samples: &[(f64, f64)],
        offset: usize,
        window: &[f64],
    ) -> Vec<(f64, f64)> {
        let n = self.fft_size;
        let mut buf: Vec<(f64, f64)> = Vec::with_capacity(n);

        for i in 0..n {
            let idx = offset + i;
            if idx < samples.len() {
                let (re, im) = samples[idx];
                buf.push((re * window[i], im * window[i]));
            } else {
                buf.push((0.0, 0.0));
            }
        }

        fft_radix2(&mut buf);
        buf
    }
}

// ---------------------------------------------------------------------------
// Minimal in-place radix-2 Cooley-Tukey FFT (std-only)
// ---------------------------------------------------------------------------

/// In-place radix-2 FFT on a buffer of `(re, im)` tuples.
/// The buffer length **must** be a power of two.
fn fft_radix2(buf: &mut [(f64, f64)]) {
    let n = buf.len();
    debug_assert!(n.is_power_of_two(), "FFT size must be power of 2");

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
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle_step = -2.0 * PI / len as f64;
        for start in (0..n).step_by(len) {
            for k in 0..half {
                let angle = angle_step * k as f64;
                let (tw_re, tw_im) = (angle.cos(), angle.sin());

                let (a_re, a_im) = buf[start + k];
                let (b_re, b_im) = buf[start + k + half];

                // Twiddle multiply
                let t_re = b_re * tw_re - b_im * tw_im;
                let t_im = b_re * tw_im + b_im * tw_re;

                buf[start + k] = (a_re + t_re, a_im + t_im);
                buf[start + k + half] = (a_re - t_re, a_im - t_im);
            }
        }
        len <<= 1;
    }
}

/// Complex division: (a + bi) / (c + di).
#[inline]
fn complex_div(num: (f64, f64), den: (f64, f64)) -> (f64, f64) {
    let (a, b) = num;
    let (c, d) = den;
    let denom = c * c + d * d;
    if denom < 1e-30 {
        return (0.0, 0.0);
    }
    ((a * c + b * d) / denom, (b * c - a * d) / denom)
}

/// Wrap a phase difference into (-pi, pi].
#[inline]
fn wrap_phase(mut dp: f64) -> f64 {
    while dp > PI {
        dp -= 2.0 * PI;
    }
    while dp <= -PI {
        dp += 2.0 * PI;
    }
    dp
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> TfConfig {
        TfConfig {
            window_size: 64,
            hop_size: 32,
            sample_rate: 1000.0,
            threshold_db: -60.0,
        }
    }

    /// Helper: generate a complex tone at `freq_hz` sampled at `fs`.
    fn tone(freq_hz: f64, fs: f64, n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let phase = 2.0 * PI * freq_hz * t;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    // -- TfConfig / construction -------------------------------------------

    #[test]
    fn test_new_rounds_to_power_of_two() {
        let config = TfConfig {
            window_size: 50,
            hop_size: 20,
            sample_rate: 8000.0,
            threshold_db: -40.0,
        };
        let rs = ReassignedSpectrogram::new(config);
        assert_eq!(rs.fft_size(), 64);
    }

    #[test]
    fn test_new_minimum_fft_size() {
        let config = TfConfig {
            window_size: 1,
            hop_size: 1,
            sample_rate: 100.0,
            threshold_db: -40.0,
        };
        let rs = ReassignedSpectrogram::new(config);
        assert!(rs.fft_size() >= 4);
    }

    #[test]
    fn test_hop_size_clamped() {
        let config = TfConfig {
            window_size: 16,
            hop_size: 999,
            sample_rate: 100.0,
            threshold_db: -40.0,
        };
        let rs = ReassignedSpectrogram::new(config);
        assert!(rs.hop_size() <= rs.fft_size());
    }

    // -- FFT ---------------------------------------------------------------

    #[test]
    fn test_fft_dc() {
        // All-ones input -> energy in bin 0
        let mut buf: Vec<(f64, f64)> = vec![(1.0, 0.0); 8];
        fft_radix2(&mut buf);
        let mag0 = (buf[0].0 * buf[0].0 + buf[0].1 * buf[0].1).sqrt();
        assert!((mag0 - 8.0).abs() < 1e-10);
        // Other bins should be ~0
        for k in 1..8 {
            let mag = (buf[k].0 * buf[k].0 + buf[k].1 * buf[k].1).sqrt();
            assert!(mag < 1e-10, "bin {} magnitude = {}", k, mag);
        }
    }

    #[test]
    fn test_fft_single_tone() {
        // exp(j * 2*pi * 2 * n / 16)  -> peak at bin 2
        let n = 16;
        let mut buf: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * 2.0 * i as f64 / n as f64;
                (phase.cos(), phase.sin())
            })
            .collect();
        fft_radix2(&mut buf);
        // Find peak bin
        let peak_bin = buf
            .iter()
            .enumerate()
            .max_by(|a, b| {
                let ma = a.1 .0 * a.1 .0 + a.1 .1 * a.1 .1;
                let mb = b.1 .0 * b.1 .0 + b.1 .1 * b.1 .1;
                ma.partial_cmp(&mb).unwrap()
            })
            .unwrap()
            .0;
        assert_eq!(peak_bin, 2);
    }

    // -- complex_div -------------------------------------------------------

    #[test]
    fn test_complex_div_basic() {
        // (3 + 4i) / (1 + 0i) = (3 + 4i)
        let r = complex_div((3.0, 4.0), (1.0, 0.0));
        assert!((r.0 - 3.0).abs() < 1e-12);
        assert!((r.1 - 4.0).abs() < 1e-12);
    }

    #[test]
    fn test_complex_div_zero_denominator() {
        let r = complex_div((1.0, 2.0), (0.0, 0.0));
        assert_eq!(r, (0.0, 0.0));
    }

    // -- spectrogram -------------------------------------------------------

    #[test]
    fn test_spectrogram_dimensions() {
        let cfg = default_config();
        let rs = ReassignedSpectrogram::new(cfg);
        let samples = tone(100.0, 1000.0, 256);
        let spec = rs.spectrogram(&samples);

        // Expected frames: (256 - 64) / 32 + 1 = 7
        assert_eq!(spec.len(), 7);
        // Each frame has fft_size bins
        for frame in &spec {
            assert_eq!(frame.len(), 64);
        }
    }

    #[test]
    fn test_spectrogram_empty_input() {
        let cfg = default_config();
        let rs = ReassignedSpectrogram::new(cfg);
        let spec = rs.spectrogram(&[]);
        assert!(spec.is_empty());
    }

    #[test]
    fn test_spectrogram_too_short_input() {
        let cfg = default_config();
        let rs = ReassignedSpectrogram::new(cfg);
        let samples = tone(100.0, 1000.0, 32); // shorter than window
        let spec = rs.spectrogram(&samples);
        assert!(spec.is_empty());
    }

    #[test]
    fn test_spectrogram_peak_at_tone_frequency() {
        // 250 Hz tone at 1000 Hz sample rate -> bin 64 * 250/1000 = 16
        let cfg = TfConfig {
            window_size: 64,
            hop_size: 32,
            sample_rate: 1000.0,
            threshold_db: -60.0,
        };
        let rs = ReassignedSpectrogram::new(cfg);
        let samples = tone(250.0, 1000.0, 256);
        let spec = rs.spectrogram(&samples);

        // Check middle frame for peak near bin 16
        let mid = spec.len() / 2;
        let frame = &spec[mid];
        let peak_bin = frame
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert_eq!(peak_bin, 16, "Expected peak at bin 16 for 250 Hz tone");
    }

    // -- compute (reassigned spectrogram) ----------------------------------

    #[test]
    fn test_compute_returns_points() {
        let cfg = default_config();
        let rs = ReassignedSpectrogram::new(cfg);
        let samples = tone(200.0, 1000.0, 256);
        let result = rs.compute(&samples);
        assert!(!result.is_empty());
        let total: usize = result.iter().map(|f| f.len()).sum();
        assert!(total > 0, "Should have reassigned points");
    }

    #[test]
    fn test_compute_reassigned_freq_near_true_freq() {
        // A 200 Hz pure tone should reassign near 200 Hz
        let cfg = TfConfig {
            window_size: 128,
            hop_size: 64,
            sample_rate: 1000.0,
            threshold_db: -30.0,
        };
        let rs = ReassignedSpectrogram::new(cfg);
        let samples = tone(200.0, 1000.0, 512);
        let result = rs.compute(&samples);

        // Collect the strongest point per frame and check frequency
        for frame in &result {
            if frame.is_empty() {
                continue;
            }
            let best = frame
                .iter()
                .max_by(|a, b| a.magnitude_db.partial_cmp(&b.magnitude_db).unwrap())
                .unwrap();
            // Reassigned frequency should be within 20 Hz of 200 Hz
            assert!(
                (best.freq_hz - 200.0).abs() < 20.0,
                "Reassigned freq {} Hz too far from 200 Hz",
                best.freq_hz
            );
        }
    }

    #[test]
    fn test_compute_threshold_filters_noise() {
        // Very high threshold should keep only the strongest bins
        let cfg = TfConfig {
            window_size: 64,
            hop_size: 32,
            sample_rate: 1000.0,
            threshold_db: -3.0, // very strict
        };
        let rs = ReassignedSpectrogram::new(cfg);
        let samples = tone(200.0, 1000.0, 256);
        let result = rs.compute(&samples);

        // With a strict threshold, fewer points should survive
        let total_strict: usize = result.iter().map(|f| f.len()).sum();

        let cfg_loose = TfConfig {
            window_size: 64,
            hop_size: 32,
            sample_rate: 1000.0,
            threshold_db: -80.0, // very loose
        };
        let rs_loose = ReassignedSpectrogram::new(cfg_loose);
        let result_loose = rs_loose.compute(&samples);
        let total_loose: usize = result_loose.iter().map(|f| f.len()).sum();

        assert!(
            total_strict <= total_loose,
            "Strict threshold ({}) should yield <= points than loose ({})",
            total_strict,
            total_loose,
        );
    }

    // -- instantaneous_frequency -------------------------------------------

    #[test]
    fn test_instantaneous_frequency_pure_tone() {
        let freq = 100.0;
        let fs = 1000.0;
        let samples = tone(freq, fs, 128);
        let cfg = default_config();
        let rs = ReassignedSpectrogram::new(cfg);
        let inst_freq = rs.instantaneous_frequency(&samples);

        assert_eq!(inst_freq.len(), 128);
        // Skip first sample (forward difference), check the rest
        for &f in &inst_freq[1..] {
            assert!(
                (f - freq).abs() < 1.0,
                "Instantaneous freq {} too far from {} Hz",
                f,
                freq
            );
        }
    }

    #[test]
    fn test_instantaneous_frequency_empty() {
        let cfg = default_config();
        let rs = ReassignedSpectrogram::new(cfg);
        let result = rs.instantaneous_frequency(&[]);
        assert!(result.is_empty());
    }

    // -- reassignment_vectors ----------------------------------------------

    #[test]
    fn test_reassignment_vectors_dimensions() {
        let cfg = default_config();
        let rs = ReassignedSpectrogram::new(cfg);
        let samples = tone(200.0, 1000.0, 256);
        let (time_corr, freq_corr) = rs.reassignment_vectors(&samples);

        let expected_frames = 7; // (256 - 64) / 32 + 1
        assert_eq!(time_corr.len(), expected_frames);
        assert_eq!(freq_corr.len(), expected_frames);
        for f in 0..expected_frames {
            assert_eq!(time_corr[f].len(), 64);
            assert_eq!(freq_corr[f].len(), 64);
        }
    }

    // -- wrap_phase --------------------------------------------------------

    #[test]
    fn test_wrap_phase() {
        assert!((wrap_phase(0.0)).abs() < 1e-12);
        assert!((wrap_phase(PI) - PI).abs() < 1e-12);
        assert!((wrap_phase(3.0 * PI) - PI).abs() < 1e-12);
        assert!((wrap_phase(-3.0 * PI) - PI).abs() < 1e-12);
    }
}
