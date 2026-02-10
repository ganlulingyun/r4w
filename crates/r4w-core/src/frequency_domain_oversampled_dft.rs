//! Non-uniform DFT bank for arbitrary sub-band extraction without aliasing.
//!
//! This module provides an oversampled DFT-based channelizer that splits a wideband
//! input into multiple sub-bands with configurable oversampling to prevent aliasing
//! at channel edges. It also supports arbitrary single sub-band extraction via
//! frequency shifting and the Goertzel algorithm for efficient single-bin DFT
//! computation.
//!
//! # Example
//!
//! ```
//! use r4w_core::frequency_domain_oversampled_dft::{
//!     OversampledDftBank, DftBankConfig, SubbandExtractor, WindowType,
//! };
//!
//! // Create a 4-channel oversampled DFT bank
//! let config = DftBankConfig {
//!     num_channels: 4,
//!     oversampling_factor: 2.0,
//!     sample_rate: 1000.0,
//!     window_type: WindowType::Hann,
//! };
//! let bank = OversampledDftBank::new(config);
//!
//! // Generate a test signal: tone at 100 Hz
//! let n = 64;
//! let fs = 1000.0;
//! let input: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / fs;
//!         let phase = 2.0 * std::f64::consts::PI * 100.0 * t;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! // Channelize into sub-bands
//! let channels = bank.channelize(&input);
//! assert_eq!(channels.len(), 4);
//!
//! // Extract a single sub-band at 100 Hz
//! let mut extractor = SubbandExtractor::new(100.0, 250.0, fs);
//! let result = extractor.extract(&input);
//! assert!(result.samples.len() > 0);
//! assert!((result.center_freq_hz - 100.0).abs() < 1e-10);
//! ```

use std::f64::consts::PI;

// ─── Types ───────────────────────────────────────────────────────────────────

/// Window function type for the DFT bank prototype filter.
#[derive(Debug, Clone, PartialEq)]
pub enum WindowType {
    /// Rectangular (no windowing).
    Rectangular,
    /// Hann window.
    Hann,
    /// Hamming window.
    Hamming,
    /// Blackman window.
    Blackman,
    /// Kaiser window with parameter beta.
    Kaiser(f64),
}

/// Configuration for an [`OversampledDftBank`].
#[derive(Debug, Clone)]
pub struct DftBankConfig {
    /// Number of channels to split the input into.
    pub num_channels: usize,
    /// Oversampling factor (1.0–4.0). Values > 1 widen each channel to prevent
    /// aliasing at edges.
    pub oversampling_factor: f64,
    /// Input sample rate in Hz.
    pub sample_rate: f64,
    /// Window applied to the prototype lowpass filter.
    pub window_type: WindowType,
}

/// Result of sub-band extraction.
#[derive(Debug, Clone)]
pub struct SubbandResult {
    /// Complex IQ samples of the extracted sub-band.
    pub samples: Vec<(f64, f64)>,
    /// Center frequency of the sub-band in Hz.
    pub center_freq_hz: f64,
    /// Bandwidth of the sub-band in Hz.
    pub bandwidth_hz: f64,
    /// Output sample rate of the sub-band in Hz.
    pub output_sample_rate: f64,
}

// ─── Window helpers ──────────────────────────────────────────────────────────

/// Compute the zeroth-order modified Bessel function of the first kind, I₀(x).
fn bessel_i0(x: f64) -> f64 {
    let mut sum = 1.0_f64;
    let mut term = 1.0_f64;
    for k in 1..=30 {
        term *= (x / (2.0 * k as f64)) * (x / (2.0 * k as f64));
        sum += term;
        if term < 1e-15 * sum {
            break;
        }
    }
    sum
}

/// Generate a window of length `n` according to `wtype`.
fn generate_window(wtype: &WindowType, n: usize) -> Vec<f64> {
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return vec![1.0];
    }
    let m = (n - 1) as f64;
    (0..n)
        .map(|i| {
            let x = i as f64;
            match wtype {
                WindowType::Rectangular => 1.0,
                WindowType::Hann => 0.5 * (1.0 - (2.0 * PI * x / m).cos()),
                WindowType::Hamming => 0.54 - 0.46 * (2.0 * PI * x / m).cos(),
                WindowType::Blackman => {
                    0.42 - 0.5 * (2.0 * PI * x / m).cos()
                        + 0.08 * (4.0 * PI * x / m).cos()
                }
                WindowType::Kaiser(beta) => {
                    let arg = 2.0 * x / m - 1.0;
                    bessel_i0(beta * (1.0 - arg * arg).max(0.0).sqrt()) / bessel_i0(*beta)
                }
            }
        })
        .collect()
}

// ─── Complex arithmetic helpers ──────────────────────────────────────────────

#[inline]
fn cx_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn cx_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn cx_mag_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

// ─── OversampledDftBank ──────────────────────────────────────────────────────

/// Oversampled polyphase DFT channelizer.
///
/// Splits a wideband input into `num_channels` sub-bands. When
/// `oversampling_factor > 1.0`, each channel is wider than the critically-sampled
/// spacing, which eliminates aliasing at channel edges.
pub struct OversampledDftBank {
    config: DftBankConfig,
    /// Prototype lowpass filter taps (windowed sinc), length = num_channels * taps_per_phase.
    prototype: Vec<f64>,
    /// Number of taps per polyphase phase.
    taps_per_phase: usize,
}

impl OversampledDftBank {
    /// Create a new oversampled DFT bank from the given configuration.
    ///
    /// The oversampling factor is clamped to \[1.0, 4.0\].
    pub fn new(config: DftBankConfig) -> Self {
        let osf = config.oversampling_factor.clamp(1.0, 4.0);
        let num_ch = config.num_channels.max(1);
        // Design prototype lowpass: cutoff = osf / (2 * num_channels) normalised to fs.
        let taps_per_phase = 4; // 4 taps per polyphase arm is a reasonable default
        let filter_len = num_ch * taps_per_phase;
        let cutoff = osf / (2.0 * num_ch as f64); // normalised to fs
        let window = generate_window(&config.window_type, filter_len);

        let half = (filter_len - 1) as f64 / 2.0;
        let prototype: Vec<f64> = (0..filter_len)
            .map(|i| {
                let n = i as f64 - half;
                let sinc = if n.abs() < 1e-12 {
                    1.0
                } else {
                    (2.0 * PI * cutoff * n).sin() / (PI * n)
                };
                sinc * window[i] * 2.0 * cutoff
            })
            .collect();

        // Normalise so that DC gain = 1.
        let dc_gain: f64 = prototype.iter().sum();
        let prototype = if dc_gain.abs() > 1e-15 {
            prototype.iter().map(|&v| v / dc_gain).collect()
        } else {
            prototype
        };

        Self {
            config: DftBankConfig {
                oversampling_factor: osf,
                num_channels: num_ch,
                ..config
            },
            prototype,
            taps_per_phase,
        }
    }

    /// Channelize the input into `num_channels` sub-band streams.
    ///
    /// Returns a vector of length `num_channels`, each containing the decimated
    /// complex samples for that channel.
    pub fn channelize(&self, input: &[(f64, f64)]) -> Vec<Vec<(f64, f64)>> {
        let m = self.config.num_channels;
        if m == 0 || input.is_empty() {
            return vec![Vec::new(); m];
        }

        // Determine decimation factor based on oversampling.
        // Critically sampled: decimate by M.  Oversampled: decimate by M / osf (rounded).
        let decim = (m as f64 / self.config.oversampling_factor).round().max(1.0) as usize;
        let num_blocks = input.len() / decim;
        if num_blocks == 0 {
            return vec![Vec::new(); m];
        }

        let mut channels: Vec<Vec<(f64, f64)>> = vec![Vec::with_capacity(num_blocks); m];

        for blk in 0..num_blocks {
            let start = blk * decim;

            // Polyphase filtering: partition the input into M phases and apply
            // the corresponding prototype sub-filter.
            let mut phase_out: Vec<(f64, f64)> = vec![(0.0, 0.0); m];
            for p in 0..m {
                let mut acc = (0.0, 0.0);
                for t in 0..self.taps_per_phase {
                    let tap_idx = p + t * m;
                    if tap_idx >= self.prototype.len() {
                        break;
                    }
                    let h = self.prototype[tap_idx];
                    // Input index goes backwards from the current position for
                    // the polyphase decomposition.
                    let in_idx_signed = start as isize + p as isize - (t as isize * m as isize);
                    if in_idx_signed >= 0 && (in_idx_signed as usize) < input.len() {
                        let s = input[in_idx_signed as usize];
                        acc = cx_add(acc, (s.0 * h, s.1 * h));
                    }
                }
                phase_out[p] = acc;
            }

            // Apply M-point DFT across the polyphase outputs.
            for k in 0..m {
                let mut sum = (0.0, 0.0);
                for n in 0..m {
                    let angle = -2.0 * PI * (k as f64) * (n as f64) / (m as f64);
                    let twiddle = (angle.cos(), angle.sin());
                    sum = cx_add(sum, cx_mul(phase_out[n], twiddle));
                }
                channels[k].push(sum);
            }
        }

        channels
    }

    /// Return the channel spacing in Hz.
    pub fn channel_spacing_hz(&self) -> f64 {
        self.config.sample_rate / self.config.num_channels as f64
    }

    /// Return the per-channel bandwidth in Hz (accounting for oversampling).
    pub fn channel_bandwidth_hz(&self) -> f64 {
        self.config.sample_rate * self.config.oversampling_factor
            / self.config.num_channels as f64
    }

    /// Return a reference to the bank configuration.
    pub fn config(&self) -> &DftBankConfig {
        &self.config
    }
}

// ─── SubbandExtractor ────────────────────────────────────────────────────────

/// Extract a single sub-band at an arbitrary center frequency.
///
/// Internally this frequency-shifts the input so the desired band is at DC,
/// applies a lowpass filter, and decimates.
pub struct SubbandExtractor {
    center_freq_hz: f64,
    bandwidth_hz: f64,
    sample_rate: f64,
}

impl SubbandExtractor {
    /// Create a new sub-band extractor.
    ///
    /// * `center_freq_hz` – desired center frequency in Hz
    /// * `bandwidth_hz` – width of the sub-band in Hz
    /// * `sample_rate` – input sample rate in Hz
    pub fn new(center_freq_hz: f64, bandwidth_hz: f64, sample_rate: f64) -> Self {
        Self {
            center_freq_hz,
            bandwidth_hz,
            sample_rate,
        }
    }

    /// Extract the sub-band from the input signal.
    pub fn extract(&self, input: &[(f64, f64)]) -> SubbandResult {
        if input.is_empty() {
            return SubbandResult {
                samples: Vec::new(),
                center_freq_hz: self.center_freq_hz,
                bandwidth_hz: self.bandwidth_hz,
                output_sample_rate: self.output_sample_rate(),
            };
        }

        // 1) Frequency-shift the input to move the desired center to DC.
        let shifted: Vec<(f64, f64)> = input
            .iter()
            .enumerate()
            .map(|(i, &s)| {
                let t = i as f64 / self.sample_rate;
                let angle = -2.0 * PI * self.center_freq_hz * t;
                cx_mul(s, (angle.cos(), angle.sin()))
            })
            .collect();

        // 2) Lowpass filter at bandwidth / 2.
        let cutoff_norm = (self.bandwidth_hz / self.sample_rate).min(0.5);
        let filter_len = 31; // fixed moderate length
        let window = generate_window(&WindowType::Hamming, filter_len);
        let half = (filter_len - 1) as f64 / 2.0;
        let mut lpf: Vec<f64> = (0..filter_len)
            .map(|i| {
                let n = i as f64 - half;
                let sinc = if n.abs() < 1e-12 {
                    1.0
                } else {
                    (2.0 * PI * cutoff_norm * n).sin() / (PI * n)
                };
                sinc * window[i] * 2.0 * cutoff_norm
            })
            .collect();

        let dc: f64 = lpf.iter().sum();
        if dc.abs() > 1e-15 {
            for v in lpf.iter_mut() {
                *v /= dc;
            }
        }

        // Apply FIR filter.
        let filtered: Vec<(f64, f64)> = (0..shifted.len())
            .map(|i| {
                let mut acc = (0.0, 0.0);
                for (j, &h) in lpf.iter().enumerate() {
                    if i >= j {
                        let s = shifted[i - j];
                        acc = cx_add(acc, (s.0 * h, s.1 * h));
                    }
                }
                acc
            })
            .collect();

        // 3) Decimate.
        let decim = (self.sample_rate / self.bandwidth_hz).round().max(1.0) as usize;
        let out: Vec<(f64, f64)> = filtered.iter().step_by(decim).copied().collect();

        SubbandResult {
            samples: out,
            center_freq_hz: self.center_freq_hz,
            bandwidth_hz: self.bandwidth_hz,
            output_sample_rate: self.sample_rate / decim as f64,
        }
    }

    /// Compute a single-bin DFT at the given frequency using the Goertzel algorithm.
    ///
    /// Returns the complex DFT coefficient for `freq_hz`.
    pub fn goertzel(&self, input: &[(f64, f64)], freq_hz: f64) -> (f64, f64) {
        goertzel(input, freq_hz, self.sample_rate)
    }

    /// Change the center frequency for subsequent extractions.
    pub fn set_center_freq(&mut self, freq_hz: f64) {
        self.center_freq_hz = freq_hz;
    }

    /// The output sample rate after decimation.
    pub fn output_sample_rate(&self) -> f64 {
        let decim = (self.sample_rate / self.bandwidth_hz).round().max(1.0) as usize;
        self.sample_rate / decim as f64
    }
}

// ─── Goertzel (free function) ────────────────────────────────────────────────

/// Goertzel algorithm: compute a single DFT bin for complex input.
///
/// This is O(N) and much cheaper than a full FFT when only one or a few bins are
/// needed.
pub fn goertzel(input: &[(f64, f64)], freq_hz: f64, sample_rate: f64) -> (f64, f64) {
    let n = input.len();
    if n == 0 {
        return (0.0, 0.0);
    }
    // Normalised frequency index (can be non-integer for arbitrary freq).
    let k = freq_hz * n as f64 / sample_rate;
    let w = 2.0 * PI * k / n as f64;
    let coeff = 2.0 * w.cos();

    // Goertzel on the real part of the input.
    let (re_s1, re_s2) = goertzel_recurse(input.iter().map(|s| s.0), coeff);
    // Goertzel on the imaginary part of the input.
    let (im_s1, im_s2) = goertzel_recurse(input.iter().map(|s| s.1), coeff);

    // Final combination: S = s1 - s2 * exp(-j w)
    let ew = (w.cos(), -(w.sin()));
    let re_part = re_s1 - (re_s2 * ew.0);
    let re_part_i = -(re_s2 * ew.1);
    let im_part = im_s1 - (im_s2 * ew.0);
    let im_part_i = -(im_s2 * ew.1);

    // X[k] = (Re-channel result) + j * (Im-channel result)
    //       = (re_part + j re_part_i) + j (im_part + j im_part_i)
    //       = (re_part - im_part_i) + j (re_part_i + im_part)
    (re_part - im_part_i, re_part_i + im_part)
}

/// Run the second-order Goertzel recursion on a real sequence, returning (s1, s2).
fn goertzel_recurse(samples: impl Iterator<Item = f64>, coeff: f64) -> (f64, f64) {
    let mut s0;
    let mut s1 = 0.0_f64;
    let mut s2 = 0.0_f64;
    for x in samples {
        s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    (s1, s2)
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    // Helper: generate a complex tone at `freq` Hz, sampled at `fs` Hz, `n` samples.
    fn tone(freq: f64, fs: f64, n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let phase = 2.0 * PI * freq * t;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    // ── Window tests ─────────────────────────────────────────────────────

    #[test]
    fn test_rectangular_window() {
        let w = generate_window(&WindowType::Rectangular, 8);
        assert_eq!(w.len(), 8);
        for &v in &w {
            assert!((v - 1.0).abs() < TOL);
        }
    }

    #[test]
    fn test_hann_window_endpoints() {
        let w = generate_window(&WindowType::Hann, 64);
        // Hann window is zero at endpoints.
        assert!(w[0].abs() < TOL);
        assert!(w[63].abs() < TOL);
        // Peak near centre.
        assert!(w[32] > 0.9);
    }

    #[test]
    fn test_hamming_window_endpoints() {
        let w = generate_window(&WindowType::Hamming, 64);
        // Hamming window is ~0.08 at endpoints (not zero).
        assert!((w[0] - 0.08).abs() < 0.01);
    }

    #[test]
    fn test_blackman_window_symmetry() {
        let w = generate_window(&WindowType::Blackman, 32);
        for i in 0..16 {
            assert!((w[i] - w[31 - i]).abs() < TOL, "symmetry at {}", i);
        }
    }

    #[test]
    fn test_kaiser_window() {
        let w = generate_window(&WindowType::Kaiser(5.0), 32);
        assert_eq!(w.len(), 32);
        // Should be symmetric.
        for i in 0..16 {
            assert!((w[i] - w[31 - i]).abs() < TOL);
        }
        // Peak at center.
        assert!(w[15] > 0.5);
    }

    // ── Goertzel tests ───────────────────────────────────────────────────

    #[test]
    fn test_goertzel_single_tone() {
        let fs = 1000.0;
        let n = 256;
        let freq = 100.0;
        let sig = tone(freq, fs, n);
        let result = goertzel(&sig, freq, fs);
        let mag = cx_mag_sq(result).sqrt();
        // For a unit-amplitude complex exponential of length N, the DFT peak
        // magnitude should be N.
        assert!(
            (mag - n as f64).abs() < 1.0,
            "expected ~{}, got {}",
            n,
            mag
        );
    }

    #[test]
    fn test_goertzel_off_bin() {
        // At a frequency where there is no tone, magnitude should be small.
        let fs = 1000.0;
        let n = 256;
        let sig = tone(100.0, fs, n);
        let result = goertzel(&sig, 300.0, fs);
        let mag = cx_mag_sq(result).sqrt();
        assert!(mag < 5.0, "off-bin magnitude too large: {}", mag);
    }

    #[test]
    fn test_goertzel_dc() {
        let n = 64;
        let sig: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        let result = goertzel(&sig, 0.0, 1000.0);
        assert!((result.0 - n as f64).abs() < TOL);
        assert!(result.1.abs() < TOL);
    }

    #[test]
    fn test_goertzel_empty_input() {
        let result = goertzel(&[], 100.0, 1000.0);
        assert!((result.0).abs() < TOL);
        assert!((result.1).abs() < TOL);
    }

    // ── OversampledDftBank tests ─────────────────────────────────────────

    #[test]
    fn test_bank_channel_count() {
        let config = DftBankConfig {
            num_channels: 8,
            oversampling_factor: 2.0,
            sample_rate: 8000.0,
            window_type: WindowType::Hann,
        };
        let bank = OversampledDftBank::new(config);
        let sig = tone(500.0, 8000.0, 256);
        let channels = bank.channelize(&sig);
        assert_eq!(channels.len(), 8);
    }

    #[test]
    fn test_bank_oversampling_clamp() {
        let config = DftBankConfig {
            num_channels: 4,
            oversampling_factor: 10.0, // should be clamped to 4.0
            sample_rate: 1000.0,
            window_type: WindowType::Rectangular,
        };
        let bank = OversampledDftBank::new(config);
        assert!((bank.config.oversampling_factor - 4.0).abs() < TOL);
    }

    #[test]
    fn test_bank_channel_spacing() {
        let config = DftBankConfig {
            num_channels: 4,
            oversampling_factor: 1.0,
            sample_rate: 4000.0,
            window_type: WindowType::Rectangular,
        };
        let bank = OversampledDftBank::new(config);
        assert!((bank.channel_spacing_hz() - 1000.0).abs() < TOL);
    }

    #[test]
    fn test_bank_channel_bandwidth_oversampled() {
        let config = DftBankConfig {
            num_channels: 4,
            oversampling_factor: 2.0,
            sample_rate: 4000.0,
            window_type: WindowType::Hann,
        };
        let bank = OversampledDftBank::new(config);
        // BW = fs * osf / M = 4000 * 2 / 4 = 2000
        assert!((bank.channel_bandwidth_hz() - 2000.0).abs() < TOL);
    }

    #[test]
    fn test_bank_empty_input() {
        let config = DftBankConfig {
            num_channels: 4,
            oversampling_factor: 1.0,
            sample_rate: 1000.0,
            window_type: WindowType::Hann,
        };
        let bank = OversampledDftBank::new(config);
        let channels = bank.channelize(&[]);
        assert_eq!(channels.len(), 4);
        for ch in &channels {
            assert!(ch.is_empty());
        }
    }

    #[test]
    fn test_bank_tone_energy_concentration() {
        // A tone at DC (0 Hz) should have most energy in channel 0.
        let fs = 1000.0;
        let n = 256;
        let sig: Vec<(f64, f64)> = vec![(1.0, 0.0); n]; // DC signal
        let config = DftBankConfig {
            num_channels: 4,
            oversampling_factor: 1.0,
            sample_rate: fs,
            window_type: WindowType::Hamming,
        };
        let bank = OversampledDftBank::new(config);
        let channels = bank.channelize(&sig);

        let energy: Vec<f64> = channels
            .iter()
            .map(|ch| ch.iter().map(|s| cx_mag_sq(*s)).sum::<f64>())
            .collect();

        // Channel 0 (DC) should have the most energy.
        let max_ch = energy
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert_eq!(max_ch, 0, "DC tone should be in channel 0, energies: {:?}", energy);
    }

    // ── SubbandExtractor tests ───────────────────────────────────────────

    #[test]
    fn test_subband_extractor_basic() {
        let fs = 1000.0;
        let n = 256;
        let bw = 250.0;
        let sig = tone(100.0, fs, n);
        let ext = SubbandExtractor::new(100.0, bw, fs);
        let result = ext.extract(&sig);
        assert!(!result.samples.is_empty());
        assert!((result.center_freq_hz - 100.0).abs() < TOL);
        assert!((result.bandwidth_hz - bw).abs() < TOL);
    }

    #[test]
    fn test_subband_extractor_output_rate() {
        let ext = SubbandExtractor::new(500.0, 100.0, 1000.0);
        // decim = round(1000/100) = 10, output rate = 1000/10 = 100
        assert!((ext.output_sample_rate() - 100.0).abs() < TOL);
    }

    #[test]
    fn test_subband_extractor_set_center_freq() {
        let mut ext = SubbandExtractor::new(100.0, 200.0, 1000.0);
        ext.set_center_freq(400.0);
        let sig = tone(400.0, 1000.0, 128);
        let result = ext.extract(&sig);
        assert!((result.center_freq_hz - 400.0).abs() < TOL);
    }

    #[test]
    fn test_subband_extractor_empty_input() {
        let ext = SubbandExtractor::new(100.0, 200.0, 1000.0);
        let result = ext.extract(&[]);
        assert!(result.samples.is_empty());
    }

    #[test]
    fn test_subband_goertzel_via_extractor() {
        let fs = 1000.0;
        let n = 256;
        let freq = 200.0;
        let sig = tone(freq, fs, n);
        let ext = SubbandExtractor::new(0.0, 500.0, fs);
        let coeff = ext.goertzel(&sig, freq);
        let mag = cx_mag_sq(coeff).sqrt();
        assert!(
            (mag - n as f64).abs() < 1.0,
            "expected ~{}, got {}",
            n,
            mag
        );
    }
}
