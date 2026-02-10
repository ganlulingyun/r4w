//! Spectral subtraction denoiser for speech and audio enhancement.
//!
//! This module implements power spectral subtraction, a classic frequency-domain
//! technique for removing stationary broadband noise from audio signals. The
//! algorithm estimates the noise power spectrum from known-silent segments and
//! subtracts it from each frame's power spectrum, with configurable
//! oversubtraction and spectral flooring to control artefacts.
//!
//! # Algorithm
//!
//! For each frame of input samples:
//!
//! 1. Apply a Hann analysis window
//! 2. Compute the DFT to obtain `Y(f)`
//! 3. Estimate the clean power spectrum:
//!    `|S(f)|² = max(|Y(f)|² − α·|N(f)|², β·|N(f)|²)`
//! 4. Compute a spectral gain: `G(f) = sqrt(|S(f)|² / |Y(f)|²)`
//! 5. (Optional) Apply a Wiener post-filter for musical-noise reduction
//! 6. Multiply `Y(f)` by `G(f)` and inverse-DFT back to time domain
//! 7. Apply a Hann synthesis window and overlap-add into the output buffer
//!
//! # Example
//!
//! ```
//! use r4w_core::spectral_subtraction_denoiser::{SpectralSubtractionDenoiser, DenoiserConfig};
//!
//! // Create a denoiser with default settings (FFT size 512)
//! let config = DenoiserConfig::default();
//! let mut denoiser = SpectralSubtractionDenoiser::new(config);
//!
//! // Feed a silent frame so the denoiser learns the noise floor
//! let silence: Vec<f64> = vec![0.01; 512];
//! denoiser.update_noise_estimate(&silence);
//!
//! // Process a noisy frame
//! let noisy_frame: Vec<f64> = (0..512)
//!     .map(|i| (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 8000.0).sin() + 0.05)
//!     .collect();
//! let clean = denoiser.process_frame(&noisy_frame);
//! assert_eq!(clean.len(), 512);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for the spectral subtraction denoiser.
#[derive(Debug, Clone)]
pub struct DenoiserConfig {
    /// FFT size (number of samples per analysis frame). Must be >= 2.
    pub fft_size: usize,
    /// Hop size in samples for overlap-add streaming. Defaults to `fft_size / 2`.
    pub hop_size: usize,
    /// Oversubtraction factor α (>= 1.0 for aggressive removal).
    pub oversubtraction_factor: f64,
    /// Spectral floor β (0..1). Prevents gain from going below `β·|N|²`.
    pub spectral_floor: f64,
    /// Exponential smoothing factor for noise estimate updates (0..1).
    /// Higher values give more weight to new observations.
    pub smoothing_factor: f64,
    /// Enable Wiener post-filter to reduce musical noise.
    pub wiener_post_filter: bool,
}

impl Default for DenoiserConfig {
    fn default() -> Self {
        Self {
            fft_size: 512,
            hop_size: 256,
            oversubtraction_factor: 2.0,
            spectral_floor: 0.01,
            smoothing_factor: 0.9,
            wiener_post_filter: false,
        }
    }
}

// ---------------------------------------------------------------------------
// Internal complex type
// ---------------------------------------------------------------------------

/// Minimal complex number for internal FFT use.
#[derive(Debug, Clone, Copy)]
struct Cpx {
    re: f64,
    im: f64,
}

impl Cpx {
    fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    fn mag_sq(self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    fn scale(self, s: f64) -> Self {
        Self {
            re: self.re * s,
            im: self.im * s,
        }
    }

    fn mul(self, rhs: Self) -> Self {
        Self {
            re: self.re * rhs.re - self.im * rhs.im,
            im: self.re * rhs.im + self.im * rhs.re,
        }
    }

    fn add(self, rhs: Self) -> Self {
        Self {
            re: self.re + rhs.re,
            im: self.im + rhs.im,
        }
    }

    fn sub(self, rhs: Self) -> Self {
        Self {
            re: self.re - rhs.re,
            im: self.im - rhs.im,
        }
    }
}

// ---------------------------------------------------------------------------
// FFT (radix-2 Cooley-Tukey + Bluestein for non-power-of-2)
// ---------------------------------------------------------------------------

/// Returns true if `n` is a power of two.
fn is_power_of_two(n: usize) -> bool {
    n > 0 && (n & (n - 1)) == 0
}

/// Next power of two >= n.
fn next_pow2(n: usize) -> usize {
    let mut v = 1;
    while v < n {
        v <<= 1;
    }
    v
}

/// In-place radix-2 Cooley-Tukey FFT. `inverse` selects IFFT.
/// `buf.len()` **must** be a power of two.
fn radix2_fft(buf: &mut [Cpx], inverse: bool) {
    let n = buf.len();
    debug_assert!(is_power_of_two(n));

    // Bit-reversal permutation
    let mut j: usize = 0;
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
    let sign: f64 = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * 2.0 * PI / len as f64;
        let wn = Cpx::new(angle.cos(), angle.sin());
        let mut start = 0;
        while start < n {
            let mut w = Cpx::new(1.0, 0.0);
            for k in 0..half {
                let u = buf[start + k];
                let t = w.mul(buf[start + k + half]);
                buf[start + k] = u.add(t);
                buf[start + k + half] = u.sub(t);
                w = w.mul(wn);
            }
            start += len;
        }
        len <<= 1;
    }

    if inverse {
        let inv = 1.0 / n as f64;
        for x in buf.iter_mut() {
            *x = x.scale(inv);
        }
    }
}

/// General-purpose DFT / IDFT via Bluestein's algorithm (uses radix-2 FFT
/// internally on a zero-padded convolution of length `next_pow2(2*n - 1)`).
fn bluestein_fft(input: &[Cpx], inverse: bool) -> Vec<Cpx> {
    let n = input.len();
    if n <= 1 {
        return input.to_vec();
    }
    let m = next_pow2(2 * n - 1);
    let sign: f64 = if inverse { 1.0 } else { -1.0 };

    // Chirp sequence
    let chirp: Vec<Cpx> = (0..n)
        .map(|k| {
            let angle = sign * PI * (k as f64 * k as f64) / n as f64;
            Cpx::new(angle.cos(), angle.sin())
        })
        .collect();

    // a[k] = x[k] * conj(chirp[k])
    let mut a = vec![Cpx::new(0.0, 0.0); m];
    for k in 0..n {
        a[k] = input[k].mul(Cpx::new(chirp[k].re, -chirp[k].im));
    }

    // b[k] = chirp[k] (with wrap-around)
    let mut b = vec![Cpx::new(0.0, 0.0); m];
    b[0] = chirp[0];
    for k in 1..n {
        b[k] = chirp[k];
        b[m - k] = chirp[k];
    }

    radix2_fft(&mut a, false);
    radix2_fft(&mut b, false);

    for i in 0..m {
        a[i] = a[i].mul(b[i]);
    }
    radix2_fft(&mut a, true);

    let mut out = Vec::with_capacity(n);
    for k in 0..n {
        let val = a[k].mul(Cpx::new(chirp[k].re, -chirp[k].im));
        if inverse {
            out.push(val.scale(1.0 / n as f64));
        } else {
            out.push(val);
        }
    }
    out
}

/// Compute forward FFT of `input`, returning complex spectrum.
fn fft(input: &[Cpx]) -> Vec<Cpx> {
    let n = input.len();
    if is_power_of_two(n) {
        let mut buf = input.to_vec();
        radix2_fft(&mut buf, false);
        buf
    } else {
        bluestein_fft(input, false)
    }
}

/// Compute inverse FFT of `spectrum`, returning time-domain samples.
fn ifft(spectrum: &[Cpx]) -> Vec<Cpx> {
    let n = spectrum.len();
    if is_power_of_two(n) {
        let mut buf = spectrum.to_vec();
        radix2_fft(&mut buf, true);
        buf
    } else {
        bluestein_fft(spectrum, true)
    }
}

// ---------------------------------------------------------------------------
// Hann window
// ---------------------------------------------------------------------------

fn hann_window(n: usize) -> Vec<f64> {
    (0..n)
        .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos()))
        .collect()
}

// ---------------------------------------------------------------------------
// Denoiser
// ---------------------------------------------------------------------------

/// Spectral subtraction denoiser.
///
/// Removes stationary broadband noise from audio signals using power spectral
/// subtraction in the frequency domain.
pub struct SpectralSubtractionDenoiser {
    config: DenoiserConfig,
    /// Estimated noise power spectrum, one value per FFT bin.
    noise_spectrum: Vec<f64>,
    /// Analysis window (Hann).
    window: Vec<f64>,
    /// Number of noise frames accumulated (for initial averaging).
    noise_frames: usize,
    /// Internal overlap-add output buffer for streaming.
    overlap_buffer: Vec<f64>,
    /// Accumulator for pending input samples in streaming mode.
    input_buffer: Vec<f64>,
}

impl SpectralSubtractionDenoiser {
    /// Create a new denoiser with the given configuration.
    pub fn new(config: DenoiserConfig) -> Self {
        let fft_size = config.fft_size.max(2);
        let hop_size = if config.hop_size == 0 {
            fft_size / 2
        } else {
            config.hop_size.min(fft_size)
        };
        let window = hann_window(fft_size);
        Self {
            noise_spectrum: vec![0.0; fft_size],
            window,
            noise_frames: 0,
            overlap_buffer: vec![0.0; fft_size],
            input_buffer: Vec::new(),
            config: DenoiserConfig {
                fft_size,
                hop_size,
                ..config
            },
        }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &DenoiserConfig {
        &self.config
    }

    /// Return the current estimated noise power spectrum.
    pub fn noise_spectrum(&self) -> &[f64] {
        &self.noise_spectrum
    }

    /// Reset all internal state (noise estimate, buffers).
    pub fn reset(&mut self) {
        self.noise_spectrum.fill(0.0);
        self.noise_frames = 0;
        self.overlap_buffer.fill(0.0);
        self.input_buffer.clear();
    }

    // -- Noise estimation ---------------------------------------------------

    /// Update the noise estimate from a segment of known silence / noise-only
    /// audio.  Multiple calls are averaged together (initial calls) or
    /// exponentially smoothed (after the first estimate).
    pub fn update_noise_estimate(&mut self, noise_samples: &[f64]) {
        let n = self.config.fft_size;
        // Process as many full frames as we can from the provided samples.
        let mut offset = 0;
        while offset + n <= noise_samples.len() {
            let frame = &noise_samples[offset..offset + n];
            self.accumulate_noise_frame(frame);
            offset += self.config.hop_size;
        }
        // If the caller gave fewer samples than one frame, still use what we have
        // (zero-pad).
        if offset == 0 {
            let mut padded = vec![0.0; n];
            let copy_len = noise_samples.len().min(n);
            padded[..copy_len].copy_from_slice(&noise_samples[..copy_len]);
            self.accumulate_noise_frame(&padded);
        }
    }

    fn accumulate_noise_frame(&mut self, frame: &[f64]) {
        let n = self.config.fft_size;
        let spectrum = self.windowed_fft(frame);
        let power: Vec<f64> = spectrum.iter().map(|c| c.mag_sq()).collect();

        if self.noise_frames == 0 {
            self.noise_spectrum = power;
        } else {
            let alpha = self.config.smoothing_factor;
            for i in 0..n {
                self.noise_spectrum[i] = alpha * power[i] + (1.0 - alpha) * self.noise_spectrum[i];
            }
        }
        self.noise_frames += 1;
    }

    // -- Single-frame processing -------------------------------------------

    /// Process a single frame of `fft_size` samples and return the denoised
    /// frame (same length).  If the frame length differs from `fft_size` it is
    /// zero-padded or truncated.
    pub fn process_frame(&self, frame: &[f64]) -> Vec<f64> {
        let n = self.config.fft_size;
        let mut padded;
        let input = if frame.len() == n {
            frame
        } else {
            padded = vec![0.0; n];
            let copy_len = frame.len().min(n);
            padded[..copy_len].copy_from_slice(&frame[..copy_len]);
            &padded
        };

        let spectrum = self.windowed_fft(input);
        let enhanced = self.apply_subtraction(&spectrum);
        let time_domain = ifft(&enhanced);

        time_domain.iter().map(|c| c.re).collect()
    }

    // -- Streaming interface ------------------------------------------------

    /// Push samples into the streaming denoiser and return any denoised output
    /// produced so far via overlap-add synthesis.  Output may be shorter than
    /// input (buffering latency) or longer (flushing previous data).
    pub fn process_stream(&mut self, samples: &[f64]) -> Vec<f64> {
        self.input_buffer.extend_from_slice(samples);
        let n = self.config.fft_size;
        let hop = self.config.hop_size;
        let mut output = Vec::new();

        while self.input_buffer.len() >= n {
            let frame: Vec<f64> = self.input_buffer[..n].to_vec();

            // Analysis
            let spectrum = self.windowed_fft(&frame);
            let enhanced = self.apply_subtraction(&spectrum);
            let time_domain = ifft(&enhanced);

            // Synthesis windowing + overlap-add
            for i in 0..n {
                self.overlap_buffer[i] += time_domain[i].re * self.window[i];
            }

            // Output the first `hop` samples
            output.extend_from_slice(&self.overlap_buffer[..hop]);

            // Shift overlap buffer
            let mut new_buf = vec![0.0; n];
            if hop < n {
                new_buf[..n - hop].copy_from_slice(&self.overlap_buffer[hop..n]);
            }
            self.overlap_buffer = new_buf;

            // Advance input
            self.input_buffer.drain(..hop);
        }

        output
    }

    // -- SNR estimation -----------------------------------------------------

    /// Estimate the signal-to-noise ratio (in dB) of a frame, using the
    /// current noise estimate.
    pub fn estimate_snr(&self, frame: &[f64]) -> f64 {
        let n = self.config.fft_size;
        let mut padded;
        let input = if frame.len() == n {
            frame
        } else {
            padded = vec![0.0; n];
            let copy_len = frame.len().min(n);
            padded[..copy_len].copy_from_slice(&frame[..copy_len]);
            &padded
        };

        let spectrum = self.windowed_fft(input);
        let total_power: f64 = spectrum.iter().map(|c| c.mag_sq()).sum();
        let noise_power: f64 = self.noise_spectrum.iter().sum();

        if noise_power < 1e-30 {
            return f64::INFINITY;
        }

        let signal_power = (total_power - noise_power).max(1e-30);
        10.0 * (signal_power / noise_power).log10()
    }

    // -- Internal helpers ---------------------------------------------------

    fn windowed_fft(&self, frame: &[f64]) -> Vec<Cpx> {
        let windowed: Vec<Cpx> = frame
            .iter()
            .zip(self.window.iter())
            .map(|(&s, &w)| Cpx::new(s * w, 0.0))
            .collect();
        fft(&windowed)
    }

    fn apply_subtraction(&self, spectrum: &[Cpx]) -> Vec<Cpx> {
        let alpha = self.config.oversubtraction_factor;
        let beta = self.config.spectral_floor;
        let wiener = self.config.wiener_post_filter;

        spectrum
            .iter()
            .zip(self.noise_spectrum.iter())
            .map(|(&y, &n_pwr)| {
                let y_pwr = y.mag_sq();
                // Power spectral subtraction with floor
                let s_pwr = {
                    let sub = y_pwr - alpha * n_pwr;
                    let floor = beta * n_pwr;
                    sub.max(floor)
                };

                // Gain
                let gain = if y_pwr > 1e-30 {
                    let g = (s_pwr / y_pwr).sqrt();
                    if wiener {
                        // Wiener post-filter: G_w = s_pwr / (s_pwr + n_pwr)
                        let w = s_pwr / (s_pwr + n_pwr + 1e-30);
                        g * w
                    } else {
                        g
                    }
                } else {
                    0.0
                };

                y.scale(gain)
            })
            .collect()
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a sine wave at `freq` Hz sampled at `fs` Hz.
    fn sine_wave(freq: f64, fs: f64, n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / fs).sin())
            .collect()
    }

    /// Helper: add white noise (simple LCG) scaled by `amplitude`.
    fn add_noise(signal: &[f64], amplitude: f64, seed: u64) -> Vec<f64> {
        let mut rng = seed;
        signal
            .iter()
            .map(|&s| {
                // Simple LCG for deterministic noise
                rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
                let u = (rng >> 33) as f64 / (1u64 << 31) as f64 - 1.0; // -1..1
                s + amplitude * u
            })
            .collect()
    }

    #[test]
    fn test_default_config() {
        let cfg = DenoiserConfig::default();
        assert_eq!(cfg.fft_size, 512);
        assert_eq!(cfg.hop_size, 256);
        assert!((cfg.oversubtraction_factor - 2.0).abs() < 1e-9);
        assert!((cfg.spectral_floor - 0.01).abs() < 1e-9);
    }

    #[test]
    fn test_new_denoiser_initialises_zeroed_noise() {
        let d = SpectralSubtractionDenoiser::new(DenoiserConfig::default());
        assert_eq!(d.noise_spectrum().len(), 512);
        assert!(d.noise_spectrum().iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_process_frame_returns_correct_length() {
        let d = SpectralSubtractionDenoiser::new(DenoiserConfig {
            fft_size: 256,
            hop_size: 128,
            ..DenoiserConfig::default()
        });
        let frame = vec![0.0; 256];
        let out = d.process_frame(&frame);
        assert_eq!(out.len(), 256);
    }

    #[test]
    fn test_process_frame_short_input_pads() {
        let d = SpectralSubtractionDenoiser::new(DenoiserConfig {
            fft_size: 128,
            hop_size: 64,
            ..DenoiserConfig::default()
        });
        let frame = vec![1.0; 50]; // shorter than fft_size
        let out = d.process_frame(&frame);
        assert_eq!(out.len(), 128);
    }

    #[test]
    fn test_noise_estimate_updates() {
        let mut d = SpectralSubtractionDenoiser::new(DenoiserConfig {
            fft_size: 64,
            hop_size: 32,
            ..DenoiserConfig::default()
        });
        let noise = add_noise(&vec![0.0; 64], 0.5, 42);
        d.update_noise_estimate(&noise);
        // After one update the noise spectrum should be non-zero
        let total: f64 = d.noise_spectrum().iter().sum();
        assert!(total > 0.0, "noise spectrum should be non-zero after update");
    }

    #[test]
    fn test_denoiser_reduces_noise_power() {
        let fft_size = 256;
        let mut d = SpectralSubtractionDenoiser::new(DenoiserConfig {
            fft_size,
            hop_size: 128,
            oversubtraction_factor: 2.0,
            spectral_floor: 0.01,
            smoothing_factor: 0.9,
            wiener_post_filter: false,
        });

        // Learn noise from a noise-only segment
        let pure_noise = add_noise(&vec![0.0; fft_size * 4], 0.3, 123);
        d.update_noise_estimate(&pure_noise);

        // Create a noisy signal (tone + noise)
        let tone = sine_wave(1000.0, 8000.0, fft_size);
        let noisy = add_noise(&tone, 0.3, 456);
        let denoised = d.process_frame(&noisy);

        // Compute power of noisy vs denoised
        let noisy_power: f64 = noisy.iter().map(|x| x * x).sum::<f64>() / fft_size as f64;
        let denoised_power: f64 =
            denoised.iter().map(|x| x * x).sum::<f64>() / fft_size as f64;

        // Denoised should have less power (noise removed)
        assert!(
            denoised_power < noisy_power,
            "denoised power ({denoised_power}) should be less than noisy ({noisy_power})"
        );
    }

    #[test]
    fn test_wiener_post_filter_further_reduces() {
        let fft_size = 256;

        let make_denoiser = |wiener: bool| {
            let mut d = SpectralSubtractionDenoiser::new(DenoiserConfig {
                fft_size,
                hop_size: 128,
                oversubtraction_factor: 2.0,
                spectral_floor: 0.01,
                smoothing_factor: 0.9,
                wiener_post_filter: wiener,
            });
            let pure_noise = add_noise(&vec![0.0; fft_size * 4], 0.5, 111);
            d.update_noise_estimate(&pure_noise);
            d
        };

        let noisy = add_noise(&vec![0.0; fft_size], 0.5, 222);

        let d_no_wiener = make_denoiser(false);
        let d_wiener = make_denoiser(true);

        let out_no = d_no_wiener.process_frame(&noisy);
        let out_wi = d_wiener.process_frame(&noisy);

        let pwr_no: f64 = out_no.iter().map(|x| x * x).sum();
        let pwr_wi: f64 = out_wi.iter().map(|x| x * x).sum();

        // Wiener should suppress more on a pure-noise frame
        assert!(
            pwr_wi <= pwr_no + 1e-9,
            "wiener ({pwr_wi}) should suppress as much or more than plain ({pwr_no})"
        );
    }

    #[test]
    fn test_reset_clears_state() {
        let mut d = SpectralSubtractionDenoiser::new(DenoiserConfig {
            fft_size: 64,
            hop_size: 32,
            ..DenoiserConfig::default()
        });
        let noise = add_noise(&vec![0.0; 128], 1.0, 99);
        d.update_noise_estimate(&noise);
        assert!(d.noise_spectrum().iter().any(|&v| v > 0.0));

        d.reset();
        assert!(d.noise_spectrum().iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_snr_estimation_high_snr() {
        let fft_size = 256;
        let mut d = SpectralSubtractionDenoiser::new(DenoiserConfig {
            fft_size,
            hop_size: 128,
            ..DenoiserConfig::default()
        });

        let noise = add_noise(&vec![0.0; fft_size * 4], 0.01, 10);
        d.update_noise_estimate(&noise);

        // Loud tone => high SNR
        let tone = sine_wave(1000.0, 8000.0, fft_size);
        let snr = d.estimate_snr(&tone);
        assert!(snr > 10.0, "SNR should be high for loud tone, got {snr}");
    }

    #[test]
    fn test_snr_estimation_low_snr() {
        let fft_size = 256;
        let mut d = SpectralSubtractionDenoiser::new(DenoiserConfig {
            fft_size,
            hop_size: 128,
            ..DenoiserConfig::default()
        });

        let noise = add_noise(&vec![0.0; fft_size * 4], 1.0, 20);
        d.update_noise_estimate(&noise);

        // Weak tone buried in noise => low SNR
        let tone: Vec<f64> = sine_wave(500.0, 8000.0, fft_size)
            .iter()
            .map(|&s| s * 0.01)
            .collect();
        let noisy = add_noise(&tone, 1.0, 30);
        let snr = d.estimate_snr(&noisy);
        assert!(snr < 10.0, "SNR should be low for weak tone in noise, got {snr}");
    }

    #[test]
    fn test_process_stream_overlap_add() {
        let fft_size = 128;
        let hop = 64;
        let mut d = SpectralSubtractionDenoiser::new(DenoiserConfig {
            fft_size,
            hop_size: hop,
            oversubtraction_factor: 1.0,
            spectral_floor: 0.0,
            smoothing_factor: 0.9,
            wiener_post_filter: false,
        });

        // Feed 3 frames worth of data
        let samples = vec![0.1; fft_size + 2 * hop];
        let out = d.process_stream(&samples);

        // With hop=64 and fft=128, after 128+128 = 256 samples we should have
        // produced at least some output hops.
        assert!(
            !out.is_empty(),
            "stream should produce output after enough samples"
        );
        // Output length should be a multiple of hop
        assert_eq!(out.len() % hop, 0);
    }

    #[test]
    fn test_hann_window_properties() {
        let w = hann_window(256);
        assert_eq!(w.len(), 256);
        // First and last should be near zero
        assert!(w[0].abs() < 1e-10, "hann[0] should be ~0");
        // Middle should be near 1
        assert!((w[128] - 1.0).abs() < 1e-10, "hann[N/2] should be ~1");
    }

    #[test]
    fn test_fft_roundtrip() {
        let n = 64;
        let original: Vec<Cpx> = (0..n)
            .map(|i| Cpx::new((2.0 * PI * 3.0 * i as f64 / n as f64).sin(), 0.0))
            .collect();
        let freq = fft(&original);
        let recovered = ifft(&freq);

        for i in 0..n {
            assert!(
                (recovered[i].re - original[i].re).abs() < 1e-10,
                "FFT roundtrip mismatch at bin {i}"
            );
            assert!(
                recovered[i].im.abs() < 1e-10,
                "imaginary part should be ~0 at bin {i}"
            );
        }
    }

    #[test]
    fn test_non_power_of_two_fft() {
        // Use a non-power-of-two size to exercise the Bluestein path
        let n = 100;
        let original: Vec<Cpx> = (0..n)
            .map(|i| Cpx::new((2.0 * PI * 7.0 * i as f64 / n as f64).cos(), 0.0))
            .collect();
        let freq = fft(&original);
        let recovered = ifft(&freq);

        for i in 0..n {
            assert!(
                (recovered[i].re - original[i].re).abs() < 1e-8,
                "Bluestein roundtrip mismatch at bin {i}: {} vs {}",
                recovered[i].re,
                original[i].re
            );
        }
    }
}
