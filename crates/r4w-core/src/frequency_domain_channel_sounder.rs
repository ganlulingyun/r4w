//! Wideband channel characterization using frequency sweeps or multitone stimuli.
//!
//! This module provides a frequency-domain channel sounder that estimates the
//! channel frequency response H(f) = Y(f) / X(f), extracts multipath profiles
//! via IFFT, and computes channel quality metrics such as coherence bandwidth
//! and RMS delay spread.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::frequency_domain_channel_sounder::{
//!     FrequencyDomainChannelSounder, StimulusType,
//! };
//!
//! // Create a sounder spanning 1 MHz to 10 MHz with 64 frequency points
//! let mut sounder = FrequencyDomainChannelSounder::new(1e6, 10e6, 64);
//! sounder.set_stimulus_type(StimulusType::Multitone);
//!
//! // Generate the transmit stimulus
//! let tx = sounder.generate_stimulus();
//! assert_eq!(tx.len(), 64);
//!
//! // In a real system the signal would pass through a channel.
//! // Here we simulate a trivial pass-through (identity channel).
//! let rx = tx.clone();
//!
//! // Measure the channel response
//! let response = sounder.measure(&tx, &rx);
//! assert_eq!(response.h_f.len(), 64);
//!
//! // Extract multipath profile from the response
//! let profile = sounder.extract_multipath_profile(&response);
//! assert!(!profile.taps.is_empty());
//!
//! // Compute RMS delay spread
//! let rms_ds = sounder.rms_delay_spread(&profile);
//! assert!(rms_ds >= 0.0);
//! ```

use std::f64::consts::PI;

// ── Complex helpers (using (f64, f64) tuples) ─────────────────────────────

fn cx_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

fn cx_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = b.0 * b.0 + b.1 * b.1;
    if denom < 1e-30 {
        return (0.0, 0.0);
    }
    ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
}

fn cx_abs(a: (f64, f64)) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

fn cx_phase(a: (f64, f64)) -> f64 {
    a.1.atan2(a.0)
}

fn cx_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

fn cx_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

fn cx_exp(phase: f64) -> (f64, f64) {
    (phase.cos(), phase.sin())
}

// ── FFT (Cooley-Tukey radix-2 DIT) ────────────────────────────────────────

/// Bit-reversal permutation for radix-2 FFT.
fn bit_reverse(x: usize, log2n: u32) -> usize {
    let mut result = 0usize;
    let mut val = x;
    for _ in 0..log2n {
        result = (result << 1) | (val & 1);
        val >>= 1;
    }
    result
}

/// In-place radix-2 DIT FFT. `inverse` selects IFFT (with 1/N scaling).
fn fft_in_place(buf: &mut [(f64, f64)], inverse: bool) {
    let n = buf.len();
    assert!(n.is_power_of_two(), "FFT size must be a power of two");
    let log2n = (n as f64).log2() as u32;

    // Bit-reversal permutation
    for i in 0..n {
        let j = bit_reverse(i, log2n);
        if i < j {
            buf.swap(i, j);
        }
    }

    // Butterfly stages
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut size = 2;
    while size <= n {
        let half = size / 2;
        let angle = sign * 2.0 * PI / size as f64;
        let w_step = cx_exp(angle);
        for start in (0..n).step_by(size) {
            let mut w: (f64, f64) = (1.0, 0.0);
            for k in 0..half {
                let even = buf[start + k];
                let odd = cx_mul(w, buf[start + k + half]);
                buf[start + k] = cx_add(even, odd);
                buf[start + k + half] = (even.0 - odd.0, even.1 - odd.1);
                w = cx_mul(w, w_step);
            }
        }
        size *= 2;
    }

    if inverse {
        let scale = 1.0 / n as f64;
        for sample in buf.iter_mut() {
            sample.0 *= scale;
            sample.1 *= scale;
        }
    }
}

/// Forward FFT, returns new Vec.
fn fft(data: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = data.len().next_power_of_two();
    let mut buf = vec![(0.0, 0.0); n];
    buf[..data.len()].copy_from_slice(data);
    fft_in_place(&mut buf, false);
    buf
}

/// Inverse FFT, returns new Vec.
fn ifft(data: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = data.len().next_power_of_two();
    let mut buf = vec![(0.0, 0.0); n];
    buf[..data.len()].copy_from_slice(data);
    fft_in_place(&mut buf, true);
    buf
}

// ── Public types ───────────────────────────────────────────────────────────

/// Stimulus waveform type used by the channel sounder.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StimulusType {
    /// Single tone swept linearly across the band.
    SweptTone,
    /// All tones transmitted simultaneously with random phase.
    Multitone,
    /// OFDM-style pilot tones at evenly spaced subcarriers.
    OfdmPilots,
    /// Maximal-length pseudo-noise sequence (PRBS-like).
    PseudoNoise,
}

/// Measured frequency-domain channel response.
#[derive(Debug, Clone)]
pub struct ChannelResponse {
    /// Centre frequency of each measurement bin (Hz).
    pub frequency_bins: Vec<f64>,
    /// Complex channel transfer function H(f) at each bin.
    pub h_f: Vec<(f64, f64)>,
    /// Group delay at each bin (seconds). One fewer element than `h_f` when
    /// computed from finite differences; padded to same length with last value.
    pub group_delay: Vec<f64>,
    /// Coherence bandwidth at the 0.9 correlation threshold (Hz).
    pub coherence_bandwidth: f64,
}

/// A single tap in the multipath profile (time-domain CIR).
#[derive(Debug, Clone)]
pub struct MultipathTap {
    /// Delay of this tap relative to the first arrival (seconds).
    pub delay: f64,
    /// Complex amplitude of the tap.
    pub amplitude: (f64, f64),
}

/// Time-domain channel impulse response extracted from the frequency response.
#[derive(Debug, Clone)]
pub struct MultipathProfile {
    /// Multipath taps sorted by delay.
    pub taps: Vec<MultipathTap>,
}

// ── Main engine ────────────────────────────────────────────────────────────

/// Frequency-domain channel sounder.
///
/// Generates stimulus waveforms spanning a configurable frequency band,
/// measures the complex transfer function H(f) = Y(f)/X(f), and derives
/// channel quality metrics (coherence bandwidth, RMS delay spread, multipath
/// profile).
#[derive(Debug, Clone)]
pub struct FrequencyDomainChannelSounder {
    start_freq: f64,
    stop_freq: f64,
    num_points: usize,
    stimulus_type: StimulusType,
    /// Deterministic seed for reproducible pseudo-random phases / PN sequences.
    seed: u64,
}

impl FrequencyDomainChannelSounder {
    /// Create a new channel sounder.
    ///
    /// * `start_freq` – lower edge of the measurement band (Hz)
    /// * `stop_freq`  – upper edge of the measurement band (Hz)
    /// * `num_points` – number of frequency measurement points (will be rounded
    ///   up to the next power of two internally for FFT operations)
    pub fn new(start_freq: f64, stop_freq: f64, num_points: usize) -> Self {
        assert!(stop_freq > start_freq, "stop_freq must exceed start_freq");
        assert!(num_points >= 2, "need at least 2 points");
        Self {
            start_freq,
            stop_freq,
            num_points,
            stimulus_type: StimulusType::SweptTone,
            seed: 42,
        }
    }

    /// Set the stimulus waveform type.
    pub fn set_stimulus_type(&mut self, st: StimulusType) {
        self.stimulus_type = st;
    }

    /// Set the seed for reproducible random phases / PN sequences.
    pub fn set_seed(&mut self, seed: u64) {
        self.seed = seed;
    }

    /// Return the frequency of measurement bin `i`.
    fn bin_freq(&self, i: usize) -> f64 {
        self.start_freq
            + (self.stop_freq - self.start_freq) * i as f64 / (self.num_points - 1).max(1) as f64
    }

    // Simple LCG PRNG for deterministic sequences.
    fn lcg(state: &mut u64) -> f64 {
        *state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        // Map to [0, 1)
        (*state >> 33) as f64 / (1u64 << 31) as f64
    }

    /// Generate the transmit stimulus waveform (frequency-domain representation).
    ///
    /// Returns `num_points` complex samples, one per frequency bin.
    pub fn generate_stimulus(&self) -> Vec<(f64, f64)> {
        let n = self.num_points;
        let mut rng = self.seed;
        match self.stimulus_type {
            StimulusType::SweptTone => {
                // Each bin is excited one at a time with unit amplitude – the
                // concatenated single-tone measurements form the stimulus vector.
                (0..n).map(|_| (1.0, 0.0)).collect()
            }
            StimulusType::Multitone => {
                // All tones at once with random phase (low PAPR variant).
                (0..n)
                    .map(|_| {
                        let phase = Self::lcg(&mut rng) * 2.0 * PI;
                        cx_exp(phase)
                    })
                    .collect()
            }
            StimulusType::OfdmPilots => {
                // Pilot tones at every 4th subcarrier; others zero.
                (0..n)
                    .map(|i| if i % 4 == 0 { (1.0, 0.0) } else { (0.0, 0.0) })
                    .collect()
            }
            StimulusType::PseudoNoise => {
                // BPSK PN sequence: amplitude ±1 with deterministic pattern.
                (0..n)
                    .map(|_| {
                        let bit = (Self::lcg(&mut rng) * 2.0) as i32; // 0 or 1
                        if bit == 0 {
                            (1.0, 0.0)
                        } else {
                            (-1.0, 0.0)
                        }
                    })
                    .collect()
            }
        }
    }

    /// Measure the channel frequency response from transmitted and received
    /// frequency-domain vectors.
    ///
    /// Both `tx` and `rx` must have the same length (`num_points`).
    /// Returns a [`ChannelResponse`] with H(f), frequency bins, group delay,
    /// and coherence bandwidth.
    pub fn measure(&self, tx: &[(f64, f64)], rx: &[(f64, f64)]) -> ChannelResponse {
        assert_eq!(tx.len(), rx.len(), "tx and rx must have equal length");
        let n = tx.len();

        // H(f) = Y(f) / X(f)
        let h_f: Vec<(f64, f64)> = tx
            .iter()
            .zip(rx.iter())
            .map(|(&x, &y)| cx_div(y, x))
            .collect();

        // Frequency bins
        let frequency_bins: Vec<f64> = (0..n).map(|i| self.bin_freq(i)).collect();

        // Group delay: -d(phase)/d(omega), finite difference
        let group_delay = self.compute_group_delay(&h_f, &frequency_bins);

        // Coherence bandwidth at 0.9 threshold
        let coherence_bandwidth = self.coherence_bandwidth_inner(&h_f, &frequency_bins, 0.9);

        ChannelResponse {
            frequency_bins,
            h_f,
            group_delay,
            coherence_bandwidth,
        }
    }

    /// Compute group delay from H(f) using finite differences.
    fn compute_group_delay(&self, h_f: &[(f64, f64)], freqs: &[f64]) -> Vec<f64> {
        let n = h_f.len();
        if n < 2 {
            return vec![0.0; n];
        }
        let mut gd = Vec::with_capacity(n);
        for i in 0..n - 1 {
            let phase_i = cx_phase(h_f[i]);
            let phase_next = cx_phase(h_f[i + 1]);
            let mut dp = phase_next - phase_i;
            // Unwrap
            while dp > PI {
                dp -= 2.0 * PI;
            }
            while dp < -PI {
                dp += 2.0 * PI;
            }
            let df = freqs[i + 1] - freqs[i];
            let omega_diff = 2.0 * PI * df;
            if omega_diff.abs() > 1e-30 {
                gd.push(-dp / omega_diff);
            } else {
                gd.push(0.0);
            }
        }
        // Pad last element
        let last = *gd.last().unwrap_or(&0.0);
        gd.push(last);
        gd
    }

    /// Extract the multipath profile (time-domain CIR) from a channel response
    /// using IFFT.
    pub fn extract_multipath_profile(&self, response: &ChannelResponse) -> MultipathProfile {
        let cir = ifft(&response.h_f);
        let bandwidth = self.stop_freq - self.start_freq;
        let n = cir.len();
        // Time resolution = 1 / bandwidth
        let dt = if bandwidth > 0.0 {
            1.0 / bandwidth
        } else {
            1.0
        };

        // Find peak to set noise threshold
        let peak_mag = cir.iter().map(|&c| cx_abs(c)).fold(0.0f64, f64::max);
        let threshold = peak_mag * 0.01; // -40 dB relative to peak

        let taps: Vec<MultipathTap> = cir
            .iter()
            .enumerate()
            .filter(|(_, &c)| cx_abs(c) > threshold)
            .map(|(i, &c)| {
                // Wrap index so that second half maps to negative delays
                // (we only keep non-negative delays for the profile).
                let delay_idx = if i <= n / 2 { i } else { 0 };
                MultipathTap {
                    delay: delay_idx as f64 * dt,
                    amplitude: c,
                }
            })
            .collect();

        MultipathProfile { taps }
    }

    /// Compute the coherence bandwidth of the channel at a given correlation
    /// threshold (typically 0.5 or 0.9).
    ///
    /// The coherence bandwidth is estimated as the largest frequency separation
    /// Δf for which the normalised autocorrelation of H(f) stays above
    /// `threshold`.
    pub fn coherence_bandwidth(&self, response: &ChannelResponse, threshold: f64) -> f64 {
        self.coherence_bandwidth_inner(&response.h_f, &response.frequency_bins, threshold)
    }

    fn coherence_bandwidth_inner(
        &self,
        h_f: &[(f64, f64)],
        freqs: &[f64],
        threshold: f64,
    ) -> f64 {
        let n = h_f.len();
        if n < 2 {
            return 0.0;
        }

        // Frequency spacing (assumed uniform)
        let df = (freqs[n - 1] - freqs[0]) / (n - 1) as f64;

        // Compute autocorrelation of H(f) via FFT
        let h_fft = fft(h_f);
        let power_spectrum: Vec<(f64, f64)> = h_fft
            .iter()
            .map(|&c| {
                let p = cx_mul(c, cx_conj(c));
                p
            })
            .collect();
        let autocorr = ifft(&power_spectrum);

        // Normalise by R(0)
        let r0 = cx_abs(autocorr[0]);
        if r0 < 1e-30 {
            return 0.0;
        }

        // Walk increasing lag until correlation drops below threshold
        for k in 1..n {
            let rk = cx_abs(autocorr[k]) / r0;
            if rk < threshold {
                // Interpolate between k-1 and k
                let rk_prev = cx_abs(autocorr[k - 1]) / r0;
                let frac = if (rk_prev - rk).abs() > 1e-30 {
                    (rk_prev - threshold) / (rk_prev - rk)
                } else {
                    0.0
                };
                return ((k - 1) as f64 + frac) * df;
            }
        }

        // Correlation never dropped below threshold → entire bandwidth is coherent
        (n - 1) as f64 * df
    }

    /// Compute the RMS delay spread of a multipath profile.
    ///
    /// τ_rms = sqrt( E[τ²] − (E[τ])² )  weighted by tap power.
    pub fn rms_delay_spread(&self, profile: &MultipathProfile) -> f64 {
        if profile.taps.is_empty() {
            return 0.0;
        }

        let total_power: f64 = profile
            .taps
            .iter()
            .map(|t| {
                let m = cx_abs(t.amplitude);
                m * m
            })
            .sum();

        if total_power < 1e-30 {
            return 0.0;
        }

        let mean_delay: f64 = profile
            .taps
            .iter()
            .map(|t| {
                let p = cx_abs(t.amplitude);
                t.delay * p * p
            })
            .sum::<f64>()
            / total_power;

        let mean_delay_sq: f64 = profile
            .taps
            .iter()
            .map(|t| {
                let p = cx_abs(t.amplitude);
                t.delay * t.delay * p * p
            })
            .sum::<f64>()
            / total_power;

        let variance = mean_delay_sq - mean_delay * mean_delay;
        if variance < 0.0 {
            0.0
        } else {
            variance.sqrt()
        }
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    // Helper: assert two floats are close.
    fn assert_close(a: f64, b: f64, tol: f64, msg: &str) {
        assert!(
            (a - b).abs() < tol,
            "{msg}: expected {b}, got {a} (diff={})",
            (a - b).abs()
        );
    }

    #[test]
    fn test_new_valid() {
        let s = FrequencyDomainChannelSounder::new(1e6, 10e6, 64);
        assert_eq!(s.num_points, 64);
        assert_eq!(s.start_freq, 1e6);
        assert_eq!(s.stop_freq, 10e6);
    }

    #[test]
    #[should_panic(expected = "stop_freq must exceed start_freq")]
    fn test_new_invalid_freqs() {
        FrequencyDomainChannelSounder::new(10e6, 1e6, 64);
    }

    #[test]
    #[should_panic(expected = "need at least 2 points")]
    fn test_new_too_few_points() {
        FrequencyDomainChannelSounder::new(1e6, 10e6, 1);
    }

    #[test]
    fn test_stimulus_swept_tone() {
        let s = FrequencyDomainChannelSounder::new(1e6, 10e6, 16);
        // Default is SweptTone
        let stim = s.generate_stimulus();
        assert_eq!(stim.len(), 16);
        for &(re, im) in &stim {
            assert_close(re, 1.0, TOL, "SweptTone real");
            assert_close(im, 0.0, TOL, "SweptTone imag");
        }
    }

    #[test]
    fn test_stimulus_multitone() {
        let mut s = FrequencyDomainChannelSounder::new(1e6, 10e6, 32);
        s.set_stimulus_type(StimulusType::Multitone);
        let stim = s.generate_stimulus();
        assert_eq!(stim.len(), 32);
        // Each tone has unit magnitude
        for &sample in &stim {
            let mag = cx_abs(sample);
            assert_close(mag, 1.0, TOL, "Multitone magnitude");
        }
    }

    #[test]
    fn test_stimulus_ofdm_pilots() {
        let mut s = FrequencyDomainChannelSounder::new(1e6, 10e6, 16);
        s.set_stimulus_type(StimulusType::OfdmPilots);
        let stim = s.generate_stimulus();
        for (i, &sample) in stim.iter().enumerate() {
            if i % 4 == 0 {
                assert_close(cx_abs(sample), 1.0, TOL, "Pilot tone mag");
            } else {
                assert_close(cx_abs(sample), 0.0, TOL, "Null subcarrier");
            }
        }
    }

    #[test]
    fn test_stimulus_pseudo_noise() {
        let mut s = FrequencyDomainChannelSounder::new(1e6, 10e6, 32);
        s.set_stimulus_type(StimulusType::PseudoNoise);
        let stim = s.generate_stimulus();
        for &sample in &stim {
            // BPSK: real is ±1, imag is 0
            assert_close(sample.1, 0.0, TOL, "PN imag");
            assert_close(sample.0.abs(), 1.0, TOL, "PN magnitude");
        }
    }

    #[test]
    fn test_identity_channel() {
        let mut s = FrequencyDomainChannelSounder::new(1e6, 10e6, 64);
        s.set_stimulus_type(StimulusType::Multitone);
        let tx = s.generate_stimulus();
        let rx = tx.clone(); // identity channel
        let resp = s.measure(&tx, &rx);
        // H(f) should be (1,0) everywhere
        for &h in &resp.h_f {
            assert_close(h.0, 1.0, TOL, "H real");
            assert_close(h.1, 0.0, TOL, "H imag");
        }
    }

    #[test]
    fn test_constant_attenuation_channel() {
        let mut s = FrequencyDomainChannelSounder::new(1e6, 10e6, 64);
        s.set_stimulus_type(StimulusType::Multitone);
        let tx = s.generate_stimulus();
        let rx: Vec<(f64, f64)> = tx.iter().map(|&(r, i)| (r * 0.5, i * 0.5)).collect();
        let resp = s.measure(&tx, &rx);
        for &h in &resp.h_f {
            assert_close(cx_abs(h), 0.5, TOL, "6dB attenuation");
        }
    }

    #[test]
    fn test_measure_length_mismatch() {
        let s = FrequencyDomainChannelSounder::new(1e6, 10e6, 8);
        let tx = vec![(1.0, 0.0); 8];
        let rx = vec![(1.0, 0.0); 4];
        let result = std::panic::catch_unwind(|| s.measure(&tx, &rx));
        assert!(result.is_err(), "Should panic on length mismatch");
    }

    #[test]
    fn test_frequency_bins() {
        let s = FrequencyDomainChannelSounder::new(1e6, 5e6, 5);
        let tx = s.generate_stimulus();
        let rx = tx.clone();
        let resp = s.measure(&tx, &rx);
        let expected = vec![1e6, 2e6, 3e6, 4e6, 5e6];
        for (i, (&actual, &exp)) in resp.frequency_bins.iter().zip(expected.iter()).enumerate() {
            assert_close(actual, exp, 1.0, &format!("bin {i} frequency"));
        }
    }

    #[test]
    fn test_group_delay_flat_channel() {
        let s = FrequencyDomainChannelSounder::new(1e6, 10e6, 64);
        let tx = s.generate_stimulus();
        let rx = tx.clone();
        let resp = s.measure(&tx, &rx);
        // Flat channel → zero phase → zero group delay
        for &gd in &resp.group_delay {
            assert_close(gd, 0.0, 1e-9, "flat channel group delay");
        }
    }

    #[test]
    fn test_extract_multipath_identity() {
        let s = FrequencyDomainChannelSounder::new(1e6, 10e6, 64);
        let tx = s.generate_stimulus();
        let rx = tx.clone();
        let resp = s.measure(&tx, &rx);
        let profile = s.extract_multipath_profile(&resp);
        // Identity channel should have one dominant tap near zero delay
        assert!(!profile.taps.is_empty(), "should have at least one tap");
        // The strongest tap should be at delay 0
        let strongest = profile
            .taps
            .iter()
            .max_by(|a, b| {
                cx_abs(a.amplitude)
                    .partial_cmp(&cx_abs(b.amplitude))
                    .unwrap()
            })
            .unwrap();
        assert_close(strongest.delay, 0.0, 1e-6, "strongest tap at zero delay");
    }

    #[test]
    fn test_rms_delay_spread_single_tap() {
        let s = FrequencyDomainChannelSounder::new(1e6, 10e6, 64);
        let profile = MultipathProfile {
            taps: vec![MultipathTap {
                delay: 1e-6,
                amplitude: (1.0, 0.0),
            }],
        };
        let rms = s.rms_delay_spread(&profile);
        // Single tap → zero spread
        assert_close(rms, 0.0, 1e-15, "single tap delay spread");
    }

    #[test]
    fn test_rms_delay_spread_two_taps() {
        let s = FrequencyDomainChannelSounder::new(1e6, 10e6, 64);
        let profile = MultipathProfile {
            taps: vec![
                MultipathTap {
                    delay: 0.0,
                    amplitude: (1.0, 0.0),
                },
                MultipathTap {
                    delay: 1e-6,
                    amplitude: (1.0, 0.0),
                },
            ],
        };
        let rms = s.rms_delay_spread(&profile);
        // Two equal-power taps at 0 and 1µs → mean = 0.5µs → rms = 0.5µs
        assert_close(rms, 0.5e-6, 1e-12, "two-tap delay spread");
    }

    #[test]
    fn test_rms_delay_spread_empty() {
        let s = FrequencyDomainChannelSounder::new(1e6, 10e6, 64);
        let profile = MultipathProfile { taps: vec![] };
        let rms = s.rms_delay_spread(&profile);
        assert_close(rms, 0.0, TOL, "empty profile delay spread");
    }

    #[test]
    fn test_coherence_bandwidth_flat() {
        let s = FrequencyDomainChannelSounder::new(1e6, 10e6, 64);
        let tx = s.generate_stimulus();
        let rx = tx.clone();
        let resp = s.measure(&tx, &rx);
        let cb = s.coherence_bandwidth(&resp, 0.9);
        // Flat channel → coherence bandwidth equals full measurement band
        let full_bw = 9e6; // 10 MHz - 1 MHz
        assert!(
            cb >= full_bw * 0.9,
            "flat channel coherence BW should be ~full band, got {cb}"
        );
    }

    #[test]
    fn test_coherence_bandwidth_custom_threshold() {
        let s = FrequencyDomainChannelSounder::new(1e6, 10e6, 64);
        let tx = s.generate_stimulus();
        let rx = tx.clone();
        let resp = s.measure(&tx, &rx);
        let cb_50 = s.coherence_bandwidth(&resp, 0.5);
        let cb_90 = s.coherence_bandwidth(&resp, 0.9);
        // Lower threshold → wider coherence bandwidth (or equal for flat)
        assert!(
            cb_50 >= cb_90,
            "CB at 0.5 ({cb_50}) should be >= CB at 0.9 ({cb_90})"
        );
    }

    #[test]
    fn test_seed_reproducibility() {
        let mut s1 = FrequencyDomainChannelSounder::new(1e6, 10e6, 32);
        s1.set_stimulus_type(StimulusType::Multitone);
        s1.set_seed(12345);
        let stim1 = s1.generate_stimulus();

        let mut s2 = FrequencyDomainChannelSounder::new(1e6, 10e6, 32);
        s2.set_stimulus_type(StimulusType::Multitone);
        s2.set_seed(12345);
        let stim2 = s2.generate_stimulus();

        for (a, b) in stim1.iter().zip(stim2.iter()) {
            assert_close(a.0, b.0, 1e-12, "seed reproducibility re");
            assert_close(a.1, b.1, 1e-12, "seed reproducibility im");
        }
    }

    #[test]
    fn test_fft_roundtrip() {
        let signal: Vec<(f64, f64)> = (0..16)
            .map(|i| {
                let t = i as f64 / 16.0;
                cx_exp(2.0 * PI * t)
            })
            .collect();
        let spectrum = fft(&signal);
        let recovered = ifft(&spectrum);
        for (i, (&orig, &rec)) in signal.iter().zip(recovered.iter()).enumerate() {
            assert_close(orig.0, rec.0, 1e-10, &format!("FFT roundtrip re[{i}]"));
            assert_close(orig.1, rec.1, 1e-10, &format!("FFT roundtrip im[{i}]"));
        }
    }

    #[test]
    fn test_phase_shift_channel() {
        // Channel that introduces a constant 90-degree phase shift
        let mut s = FrequencyDomainChannelSounder::new(1e6, 10e6, 64);
        s.set_stimulus_type(StimulusType::Multitone);
        let tx = s.generate_stimulus();
        // Multiply each sample by j = (0, 1) → 90° rotation
        let rx: Vec<(f64, f64)> = tx.iter().map(|&t| cx_mul(t, (0.0, 1.0))).collect();
        let resp = s.measure(&tx, &rx);
        for &h in &resp.h_f {
            let mag = cx_abs(h);
            let phase = cx_phase(h);
            assert_close(mag, 1.0, 1e-6, "phase channel magnitude");
            assert_close(phase, PI / 2.0, 1e-6, "phase channel angle");
        }
    }
}
