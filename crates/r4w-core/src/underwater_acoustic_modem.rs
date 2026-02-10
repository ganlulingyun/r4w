//! Underwater acoustic modem for the 20–100 kHz band.
//!
//! This module implements underwater acoustic communication (UWA) waveforms
//! including frequency-hopped chirp modulation, OFDM adapted for underwater
//! channels, Doppler scaling estimation and compensation, decision feedback
//! equalization, guard interval design, preamble-based synchronization,
//! Thorp absorption loss, spreading loss, and BER analysis.
//!
//! Different from the existing `sonar_processor` (active sonar detection) and
//! `synthetic_aperture_sonar` (SAS imaging), this module focuses on underwater
//! data communication.
//!
//! # Example
//!
//! ```
//! use r4w_core::underwater_acoustic_modem::{UwaConfig, modulate_chirp, demodulate_chirp};
//!
//! let config = UwaConfig {
//!     center_freq_hz: 25_000.0,
//!     bandwidth_hz: 4_000.0,
//!     symbol_rate_hz: 100.0,
//!     sample_rate_hz: 100_000.0,
//!     num_subcarriers: 64,
//! };
//!
//! let bits = vec![true, false, true, true, false];
//! let signal = modulate_chirp(&bits, &config);
//! assert!(!signal.is_empty());
//! let recovered = demodulate_chirp(&signal, &config);
//! assert_eq!(recovered.len(), bits.len());
//! assert_eq!(recovered, bits);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Complex sample represented as an (I, Q) tuple.
pub type Complex = (f64, f64);

/// Configuration for the underwater acoustic modem.
///
/// All frequencies are in Hz. `num_subcarriers` is used only by the OFDM path.
#[derive(Debug, Clone)]
pub struct UwaConfig {
    /// Center frequency in Hz (typically 20 000 – 100 000).
    pub center_freq_hz: f64,
    /// Usable bandwidth in Hz.
    pub bandwidth_hz: f64,
    /// Symbol rate in symbols per second.
    pub symbol_rate_hz: f64,
    /// Sampling rate in samples per second.
    pub sample_rate_hz: f64,
    /// Number of OFDM subcarriers (must be a power of two).
    pub num_subcarriers: usize,
}

/// Main modem struct that wraps a configuration and provides stateful helpers.
///
/// For most use-cases the free-standing functions are sufficient, but `UwaModem`
/// bundles them together for convenience.
#[derive(Debug, Clone)]
pub struct UwaModem {
    /// Modem configuration.
    pub config: UwaConfig,
}

/// Result of a BER simulation run.
#[derive(Debug, Clone)]
pub struct BerResult {
    /// Signal-to-noise ratio in dB.
    pub snr_db: f64,
    /// Measured bit error rate (0.0 – 1.0).
    pub ber: f64,
    /// Number of bits tested.
    pub num_bits: usize,
    /// Number of bit errors observed.
    pub num_errors: usize,
}

/// Decision-feedback equalizer state.
#[derive(Debug, Clone)]
pub struct DfeEqualizer {
    /// Feed-forward filter taps.
    pub ff_taps: Vec<f64>,
    /// Feedback filter taps.
    pub fb_taps: Vec<f64>,
    /// Step size (mu) for LMS adaptation.
    pub step_size: f64,
    /// Previous decisions buffer (for feedback).
    decisions: Vec<f64>,
}

// ---------------------------------------------------------------------------
// Complex arithmetic helpers
// ---------------------------------------------------------------------------

#[inline]
fn c_add(a: Complex, b: Complex) -> Complex {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: Complex, b: Complex) -> Complex {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_mul(a: Complex, b: Complex) -> Complex {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_conj(a: Complex) -> Complex {
    (a.0, -a.1)
}

#[inline]
fn c_abs_sq(a: Complex) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

#[inline]
fn c_abs(a: Complex) -> f64 {
    c_abs_sq(a).sqrt()
}

#[inline]
fn c_scale(a: Complex, s: f64) -> Complex {
    (a.0 * s, a.1 * s)
}

#[inline]
fn c_exp_j(theta: f64) -> Complex {
    (theta.cos(), theta.sin())
}

// ---------------------------------------------------------------------------
// Tiny in-place radix-2 DIT FFT (no external crate)
// ---------------------------------------------------------------------------

fn bit_reverse(x: usize, log2n: u32) -> usize {
    let mut result = 0usize;
    let mut val = x;
    for _ in 0..log2n {
        result = (result << 1) | (val & 1);
        val >>= 1;
    }
    result
}

fn fft_in_place(buf: &mut [Complex], inverse: bool) {
    let n = buf.len();
    assert!(n.is_power_of_two(), "FFT length must be a power of two");
    let log2n = n.trailing_zeros();

    // Bit-reversal permutation
    for i in 0..n {
        let j = bit_reverse(i, log2n);
        if i < j {
            buf.swap(i, j);
        }
    }

    let sign = if inverse { 1.0 } else { -1.0 };

    let mut size = 2;
    while size <= n {
        let half = size / 2;
        let angle_step = sign * 2.0 * PI / size as f64;
        for start in (0..n).step_by(size) {
            for k in 0..half {
                let w = c_exp_j(angle_step * k as f64);
                let a = buf[start + k];
                let b = c_mul(w, buf[start + k + half]);
                buf[start + k] = c_add(a, b);
                buf[start + k + half] = c_sub(a, b);
            }
        }
        size <<= 1;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for sample in buf.iter_mut() {
            *sample = c_scale(*sample, inv_n);
        }
    }
}

fn next_power_of_two(n: usize) -> usize {
    if n.is_power_of_two() {
        n
    } else {
        1 << (usize::BITS - (n - 1).leading_zeros())
    }
}

// ---------------------------------------------------------------------------
// Propagation loss models
// ---------------------------------------------------------------------------

/// Compute Thorp absorption loss in dB for a given frequency and range.
///
/// Uses Thorp's empirical formula (valid roughly for 0.1–100 kHz).
///
/// # Arguments
/// * `freq_khz` – Frequency in kHz.
/// * `range_km` – Range in km.
///
/// # Returns
/// Absorption loss in dB (positive value).
pub fn thorp_absorption_db(freq_khz: f64, range_km: f64) -> f64 {
    // Thorp's formula gives absorption coefficient in dB/km
    let f2 = freq_khz * freq_khz;
    let alpha = 0.11 * f2 / (1.0 + f2)
        + 44.0 * f2 / (4100.0 + f2)
        + 2.75e-4 * f2
        + 0.003;
    alpha * range_km
}

/// Compute geometric spreading loss in dB.
///
/// `spreading_factor` is typically 1.0 (cylindrical), 1.5 (practical), or 2.0 (spherical).
///
/// # Arguments
/// * `range_m` – Range in metres.
/// * `spreading_factor` – Spreading exponent (k).
///
/// # Returns
/// Spreading loss in dB: `10 * k * log10(range_m)`.
pub fn spreading_loss_db(range_m: f64, spreading_factor: f64) -> f64 {
    if range_m <= 0.0 {
        return 0.0;
    }
    10.0 * spreading_factor * range_m.log10()
}

/// Total transmission loss in dB (spreading + absorption).
///
/// # Arguments
/// * `range_m` – Range in metres.
/// * `freq_hz` – Frequency in Hz.
/// * `spreading_factor` – Geometric spreading exponent.
pub fn total_transmission_loss(range_m: f64, freq_hz: f64, spreading_factor: f64) -> f64 {
    let freq_khz = freq_hz / 1000.0;
    let range_km = range_m / 1000.0;
    spreading_loss_db(range_m, spreading_factor) + thorp_absorption_db(freq_khz, range_km)
}

// ---------------------------------------------------------------------------
// LFM Preamble
// ---------------------------------------------------------------------------

/// Generate a linear frequency modulated (LFM) chirp preamble for synchronization.
///
/// The chirp sweeps from `center_freq - bandwidth/2` to `center_freq + bandwidth/2`
/// over the given `duration_s`.
///
/// # Arguments
/// * `config` – Modem configuration (uses center_freq, bandwidth, sample_rate).
/// * `duration_s` – Duration of the preamble in seconds.
///
/// # Returns
/// Vector of complex IQ samples.
pub fn generate_lfm_preamble(config: &UwaConfig, duration_s: f64) -> Vec<Complex> {
    let n_samples = (config.sample_rate_hz * duration_s).round() as usize;
    let f0 = config.center_freq_hz - config.bandwidth_hz / 2.0;
    let chirp_rate = config.bandwidth_hz / duration_s;
    let dt = 1.0 / config.sample_rate_hz;

    (0..n_samples)
        .map(|i| {
            let t = i as f64 * dt;
            let phase = 2.0 * PI * (f0 * t + 0.5 * chirp_rate * t * t);
            c_exp_j(phase)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Doppler estimation & compensation
// ---------------------------------------------------------------------------

/// Estimate the Doppler scale factor by cross-correlating a known preamble with
/// a received copy.
///
/// Returns a scale factor `a` such that the received signal is time-stretched by
/// `(1 + a)`. A positive `a` indicates expansion (moving away), negative indicates
/// compression (moving towards).
///
/// The estimation searches over a grid of candidate scale factors in the range
/// `[-0.01, +0.01]` (i.e., up to 1 % Doppler scaling, typical for underwater).
pub fn estimate_doppler_scale(
    preamble_tx: &[Complex],
    preamble_rx: &[Complex],
) -> f64 {
    let n_candidates = 201; // search grid
    let a_min = -0.01_f64;
    let a_max = 0.01_f64;
    let step = (a_max - a_min) / (n_candidates as f64 - 1.0);

    let mut best_a = 0.0_f64;
    let mut best_corr = 0.0_f64;

    for ci in 0..n_candidates {
        let a = a_min + ci as f64 * step;
        // Resample preamble_tx by factor (1+a) and correlate with preamble_rx
        let resampled = resample_signal(preamble_tx, 1.0 + a);
        let corr = cross_correlate_peak(&resampled, preamble_rx);
        if corr > best_corr {
            best_corr = corr;
            best_a = a;
        }
    }

    best_a
}

/// Compensate Doppler scaling by resampling the signal.
///
/// Resamples `signal` by the factor `1 / (1 + scale_factor)` to undo time
/// stretching/compression.
pub fn compensate_doppler(signal: &mut Vec<Complex>, scale_factor: f64) {
    if scale_factor.abs() < 1e-12 {
        return;
    }
    let resampled = resample_signal(signal, 1.0 / (1.0 + scale_factor));
    *signal = resampled;
}

/// Linear-interpolation resampler: produce output at rate `factor` times the
/// input rate (factor > 1 means more output samples).
fn resample_signal(input: &[Complex], factor: f64) -> Vec<Complex> {
    if input.is_empty() {
        return Vec::new();
    }
    let out_len = ((input.len() as f64) / factor).round().max(1.0) as usize;
    let mut output = Vec::with_capacity(out_len);
    for i in 0..out_len {
        let pos = i as f64 * factor;
        let idx = pos.floor() as usize;
        let frac = pos - idx as f64;
        if idx + 1 < input.len() {
            let a = input[idx];
            let b = input[idx + 1];
            output.push((
                a.0 * (1.0 - frac) + b.0 * frac,
                a.1 * (1.0 - frac) + b.1 * frac,
            ));
        } else if idx < input.len() {
            output.push(input[idx]);
        }
    }
    output
}

/// Peak absolute value of cross-correlation between two signals.
fn cross_correlate_peak(a: &[Complex], b: &[Complex]) -> f64 {
    let len = a.len().min(b.len());
    let mut sum = (0.0, 0.0);
    for i in 0..len {
        sum = c_add(sum, c_mul(a[i], c_conj(b[i])));
    }
    c_abs(sum)
}

// ---------------------------------------------------------------------------
// Chirp-FSK modulation / demodulation
// ---------------------------------------------------------------------------

/// Number of samples per chirp symbol.
fn samples_per_symbol(config: &UwaConfig) -> usize {
    (config.sample_rate_hz / config.symbol_rate_hz).round() as usize
}

/// Generate an up-chirp or down-chirp for one bit.
///
/// `bit == true` -> up-chirp (freq increases), `bit == false` -> down-chirp.
fn generate_chirp_symbol(bit: bool, config: &UwaConfig) -> Vec<Complex> {
    let n = samples_per_symbol(config);
    let dt = 1.0 / config.sample_rate_hz;
    let f_low = config.center_freq_hz - config.bandwidth_hz / 2.0;
    let f_high = config.center_freq_hz + config.bandwidth_hz / 2.0;
    let symbol_duration = n as f64 * dt;
    let (f_start, f_end) = if bit {
        (f_low, f_high)
    } else {
        (f_high, f_low)
    };
    let chirp_rate = (f_end - f_start) / symbol_duration;

    (0..n)
        .map(|i| {
            let t = i as f64 * dt;
            let phase = 2.0 * PI * (f_start * t + 0.5 * chirp_rate * t * t);
            c_exp_j(phase)
        })
        .collect()
}

/// Modulate a bit sequence using Chirp-FSK.
///
/// Each bit is mapped to an up-chirp (1) or down-chirp (0). The resulting
/// complex baseband signal is the concatenation of all chirp symbols.
pub fn modulate_chirp(bits: &[bool], config: &UwaConfig) -> Vec<Complex> {
    let mut signal = Vec::with_capacity(bits.len() * samples_per_symbol(config));
    for &bit in bits {
        signal.extend(generate_chirp_symbol(bit, config));
    }
    signal
}

/// Demodulate a Chirp-FSK signal back to bits.
///
/// Uses matched-filter (correlation) detection: each symbol-length segment is
/// correlated with both the up-chirp and down-chirp references. The reference
/// yielding higher correlation magnitude wins.
pub fn demodulate_chirp(signal: &[Complex], config: &UwaConfig) -> Vec<bool> {
    let sps = samples_per_symbol(config);
    let ref_up = generate_chirp_symbol(true, config);
    let ref_down = generate_chirp_symbol(false, config);
    let num_symbols = signal.len() / sps;

    let mut bits = Vec::with_capacity(num_symbols);
    for sym_idx in 0..num_symbols {
        let start = sym_idx * sps;
        let end = start + sps;
        let chunk = &signal[start..end];

        let corr_up = correlate_chunk(chunk, &ref_up);
        let corr_down = correlate_chunk(chunk, &ref_down);

        bits.push(corr_up > corr_down);
    }
    bits
}

fn correlate_chunk(chunk: &[Complex], reference: &[Complex]) -> f64 {
    let mut sum = (0.0, 0.0);
    let len = chunk.len().min(reference.len());
    for i in 0..len {
        sum = c_add(sum, c_mul(chunk[i], c_conj(reference[i])));
    }
    c_abs(sum)
}

// ---------------------------------------------------------------------------
// OFDM for underwater channels
// ---------------------------------------------------------------------------

/// Guard interval length in samples (cyclic prefix): 25% of symbol.
fn guard_interval_samples(config: &UwaConfig) -> usize {
    config.num_subcarriers / 4
}

/// Modulate bits using OFDM adapted for underwater acoustic channels.
///
/// Uses BPSK mapping on each subcarrier, a cyclic prefix guard interval, and
/// the configured number of subcarriers. Null subcarriers are inserted at DC
/// and Nyquist.
pub fn modulate_ofdm_uwa(bits: &[bool], config: &UwaConfig) -> Vec<Complex> {
    let n = config.num_subcarriers;
    let gi = guard_interval_samples(config);
    // Usable subcarriers: skip DC (index 0) and Nyquist (index N/2).
    let usable = n - 2;
    let num_ofdm_symbols = (bits.len() + usable - 1) / usable;

    let mut output = Vec::new();
    let mut bit_idx = 0;

    for _ in 0..num_ofdm_symbols {
        // Frequency-domain symbol
        let mut freq = vec![(0.0, 0.0); n];
        for sc in 0..usable {
            let carrier_idx = if sc < n / 2 - 1 { sc + 1 } else { sc + 2 };
            let bpsk = if bit_idx < bits.len() && bits[bit_idx] {
                (1.0, 0.0)
            } else {
                (-1.0, 0.0)
            };
            freq[carrier_idx] = bpsk;
            bit_idx += 1;
        }

        // IFFT to get time-domain OFDM symbol
        fft_in_place(&mut freq, true); // inverse FFT

        // Add cyclic prefix (copy last `gi` samples to front)
        let cp: Vec<Complex> = freq[n - gi..].to_vec();
        output.extend_from_slice(&cp);
        output.extend_from_slice(&freq);
    }

    output
}

/// Demodulate an OFDM underwater acoustic signal back to bits.
///
/// Strips the cyclic prefix, performs FFT, and BPSK-demaps the usable
/// subcarriers.
pub fn demodulate_ofdm_uwa(signal: &[Complex], config: &UwaConfig) -> Vec<bool> {
    let n = config.num_subcarriers;
    let gi = guard_interval_samples(config);
    let symbol_len = n + gi;
    let usable = n - 2;
    let num_ofdm_symbols = signal.len() / symbol_len;

    let mut bits = Vec::new();

    for sym_idx in 0..num_ofdm_symbols {
        let start = sym_idx * symbol_len + gi; // skip CP
        if start + n > signal.len() {
            break;
        }
        let mut buf: Vec<Complex> = signal[start..start + n].to_vec();

        // FFT (forward)
        fft_in_place(&mut buf, false);

        for sc in 0..usable {
            let carrier_idx = if sc < n / 2 - 1 { sc + 1 } else { sc + 2 };
            bits.push(buf[carrier_idx].0 > 0.0);
        }
    }

    bits
}

// ---------------------------------------------------------------------------
// Decision Feedback Equalizer (DFE)
// ---------------------------------------------------------------------------

impl DfeEqualizer {
    /// Create a new DFE with the given number of feed-forward and feedback taps.
    ///
    /// Feed-forward taps are initialised to a unit impulse at the centre;
    /// feedback taps are initialised to zero.
    pub fn new(num_ff: usize, num_fb: usize, step_size: f64) -> Self {
        let mut ff_taps = vec![0.0; num_ff];
        if num_ff > 0 {
            ff_taps[num_ff / 2] = 1.0; // centre-spike initialisation
        }
        Self {
            ff_taps,
            fb_taps: vec![0.0; num_fb],
            step_size,
            decisions: vec![0.0; num_fb],
        }
    }

    /// Equalize a real-valued symbol sequence using LMS-adapted DFE.
    ///
    /// Returns the sequence of hard decisions (BPSK: +1 or -1).
    pub fn equalize(&mut self, input: &[f64]) -> Vec<f64> {
        let nff = self.ff_taps.len();
        let nfb = self.fb_taps.len();
        let mut output = Vec::with_capacity(input.len());

        // sliding window over input
        for i in 0..input.len() {
            // Feed-forward part
            let mut y = 0.0;
            for k in 0..nff {
                let idx = i as isize - (nff as isize / 2) + k as isize;
                if idx >= 0 && (idx as usize) < input.len() {
                    y += self.ff_taps[k] * input[idx as usize];
                }
            }
            // Feedback part (subtract ISI from past decisions)
            for k in 0..nfb {
                y -= self.fb_taps[k] * self.decisions[k];
            }

            // Hard decision (BPSK)
            let decision = if y >= 0.0 { 1.0 } else { -1.0 };
            let error = decision - y;

            // LMS update: feed-forward
            for k in 0..nff {
                let idx = i as isize - (nff as isize / 2) + k as isize;
                if idx >= 0 && (idx as usize) < input.len() {
                    self.ff_taps[k] += self.step_size * error * input[idx as usize];
                }
            }
            // LMS update: feedback
            for k in 0..nfb {
                self.fb_taps[k] -= self.step_size * error * self.decisions[k];
            }

            // Shift decision history
            for k in (1..nfb).rev() {
                self.decisions[k] = self.decisions[k - 1];
            }
            if nfb > 0 {
                self.decisions[0] = decision;
            }

            output.push(decision);
        }

        output
    }
}

// ---------------------------------------------------------------------------
// BER analysis
// ---------------------------------------------------------------------------

/// Run a simple BER simulation for chirp-FSK over an AWGN channel.
///
/// Generates `num_bits` random bits (deterministic PRBS), modulates, adds
/// Gaussian noise at the given `snr_db`, and demodulates.
pub fn ber_analysis_chirp(config: &UwaConfig, snr_db: f64, num_bits: usize) -> BerResult {
    // Generate deterministic pseudo-random bits using a simple LFSR
    let bits = prbs_bits(num_bits);

    let signal = modulate_chirp(&bits, config);

    // Add AWGN
    let noisy = add_awgn(&signal, snr_db);

    let decoded = demodulate_chirp(&noisy, config);

    let num_errors = bits
        .iter()
        .zip(decoded.iter())
        .filter(|(a, b)| a != b)
        .count();

    BerResult {
        snr_db,
        ber: num_errors as f64 / num_bits as f64,
        num_bits,
        num_errors,
    }
}

/// Simple PRBS generator (xorshift-based, deterministic).
fn prbs_bits(n: usize) -> Vec<bool> {
    let mut state: u32 = 0xACE1_u32;
    (0..n)
        .map(|_| {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            (state & 1) == 1
        })
        .collect()
}

/// Add AWGN to a complex signal at a given SNR (dB).
///
/// Uses a simple Box-Muller transform with a deterministic seed for
/// reproducibility.
fn add_awgn(signal: &[Complex], snr_db: f64) -> Vec<Complex> {
    if signal.is_empty() {
        return Vec::new();
    }
    // Compute signal power
    let sig_power: f64 =
        signal.iter().map(|s| c_abs_sq(*s)).sum::<f64>() / signal.len() as f64;
    let noise_power = sig_power / (10.0_f64).powf(snr_db / 10.0);
    let sigma = (noise_power / 2.0).sqrt();

    // Deterministic pseudo-random noise (LCG + Box-Muller)
    let mut rng_state: u64 = 123456789;
    let mut next_uniform = || -> f64 {
        rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        // Map to (0, 1)
        ((rng_state >> 33) as f64) / (u32::MAX as f64)
    };

    signal
        .iter()
        .map(|&s| {
            let u1 = next_uniform().max(1e-15);
            let u2 = next_uniform();
            let g0 = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos();
            let g1 = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).sin();
            (s.0 + sigma * g0, s.1 + sigma * g1)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// UwaModem convenience methods
// ---------------------------------------------------------------------------

impl UwaModem {
    /// Create a new `UwaModem` with the given configuration.
    pub fn new(config: UwaConfig) -> Self {
        Self { config }
    }

    /// Modulate bits using chirp-FSK.
    pub fn modulate_chirp(&self, bits: &[bool]) -> Vec<Complex> {
        modulate_chirp(bits, &self.config)
    }

    /// Demodulate a chirp-FSK signal.
    pub fn demodulate_chirp(&self, signal: &[Complex]) -> Vec<bool> {
        demodulate_chirp(signal, &self.config)
    }

    /// Modulate bits using underwater OFDM.
    pub fn modulate_ofdm(&self, bits: &[bool]) -> Vec<Complex> {
        modulate_ofdm_uwa(bits, &self.config)
    }

    /// Demodulate an underwater OFDM signal.
    pub fn demodulate_ofdm(&self, signal: &[Complex]) -> Vec<bool> {
        demodulate_ofdm_uwa(signal, &self.config)
    }

    /// Generate an LFM preamble.
    pub fn generate_preamble(&self, duration_s: f64) -> Vec<Complex> {
        generate_lfm_preamble(&self.config, duration_s)
    }

    /// Compute total transmission loss for this modem's center frequency.
    pub fn transmission_loss(&self, range_m: f64, spreading_factor: f64) -> f64 {
        total_transmission_loss(range_m, self.config.center_freq_hz, spreading_factor)
    }

    /// Run BER analysis for chirp-FSK at the given SNR.
    pub fn ber_analysis(&self, snr_db: f64, num_bits: usize) -> BerResult {
        ber_analysis_chirp(&self.config, snr_db, num_bits)
    }
}

// ---------------------------------------------------------------------------
// Guard interval helpers (public for direct use)
// ---------------------------------------------------------------------------

/// Compute the recommended guard interval duration in seconds for a given
/// maximum multipath spread.
///
/// The guard interval should be at least as long as the expected maximum
/// multipath delay to avoid inter-symbol interference.
///
/// # Arguments
/// * `max_delay_s` – Expected maximum multipath delay in seconds.
/// * `margin_factor` – Safety margin (e.g. 1.25 for 25% extra).
pub fn recommended_guard_interval(max_delay_s: f64, margin_factor: f64) -> f64 {
    max_delay_s * margin_factor
}

/// Number of guard samples at the given sample rate.
pub fn guard_samples(max_delay_s: f64, margin_factor: f64, sample_rate_hz: f64) -> usize {
    (recommended_guard_interval(max_delay_s, margin_factor) * sample_rate_hz).ceil() as usize
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> UwaConfig {
        UwaConfig {
            center_freq_hz: 25_000.0,
            bandwidth_hz: 4_000.0,
            symbol_rate_hz: 100.0,
            sample_rate_hz: 100_000.0,
            num_subcarriers: 64,
        }
    }

    // ------------------------------------------------------------------
    // 1. Thorp absorption
    // ------------------------------------------------------------------
    #[test]
    fn test_thorp_absorption_positive() {
        let loss = thorp_absorption_db(25.0, 1.0);
        assert!(loss > 0.0, "Absorption loss must be positive");
    }

    #[test]
    fn test_thorp_absorption_increases_with_range() {
        let loss1 = thorp_absorption_db(25.0, 1.0);
        let loss2 = thorp_absorption_db(25.0, 5.0);
        assert!(loss2 > loss1, "Absorption should increase with range");
    }

    #[test]
    fn test_thorp_absorption_increases_with_frequency() {
        let loss_low = thorp_absorption_db(10.0, 1.0);
        let loss_high = thorp_absorption_db(50.0, 1.0);
        assert!(loss_high > loss_low, "Absorption should increase with frequency");
    }

    // ------------------------------------------------------------------
    // 2. Spreading loss
    // ------------------------------------------------------------------
    #[test]
    fn test_spreading_loss_spherical() {
        let loss = spreading_loss_db(1000.0, 2.0);
        let expected = 20.0 * 1000.0_f64.log10(); // 60 dB
        assert!((loss - expected).abs() < 1e-6);
    }

    #[test]
    fn test_spreading_loss_zero_range() {
        assert_eq!(spreading_loss_db(0.0, 2.0), 0.0);
    }

    // ------------------------------------------------------------------
    // 3. Total transmission loss
    // ------------------------------------------------------------------
    #[test]
    fn test_total_transmission_loss() {
        let tl = total_transmission_loss(1000.0, 25000.0, 1.5);
        assert!(tl > 0.0, "Total TL should be positive for non-zero range");
        // Should be sum of spreading + absorption
        let spread = spreading_loss_db(1000.0, 1.5);
        let absorb = thorp_absorption_db(25.0, 1.0);
        assert!((tl - spread - absorb).abs() < 1e-6);
    }

    // ------------------------------------------------------------------
    // 4. LFM preamble
    // ------------------------------------------------------------------
    #[test]
    fn test_lfm_preamble_length() {
        let config = default_config();
        let preamble = generate_lfm_preamble(&config, 0.01);
        let expected = (config.sample_rate_hz * 0.01).round() as usize;
        assert_eq!(preamble.len(), expected);
    }

    #[test]
    fn test_lfm_preamble_unit_amplitude() {
        let config = default_config();
        let preamble = generate_lfm_preamble(&config, 0.01);
        for &s in &preamble {
            let mag = c_abs(s);
            assert!((mag - 1.0).abs() < 1e-10, "LFM preamble should have unit amplitude");
        }
    }

    // ------------------------------------------------------------------
    // 5. Chirp modulation round-trip
    // ------------------------------------------------------------------
    #[test]
    fn test_chirp_modulate_demodulate_roundtrip() {
        let config = default_config();
        let bits = vec![true, false, true, true, false, false, true, false];
        let signal = modulate_chirp(&bits, &config);
        let recovered = demodulate_chirp(&signal, &config);
        assert_eq!(recovered, bits);
    }

    #[test]
    fn test_chirp_modulation_length() {
        let config = default_config();
        let bits = vec![true, false, true];
        let signal = modulate_chirp(&bits, &config);
        let sps = samples_per_symbol(&config);
        assert_eq!(signal.len(), bits.len() * sps);
    }

    #[test]
    fn test_chirp_empty_bits() {
        let config = default_config();
        let signal = modulate_chirp(&[], &config);
        assert!(signal.is_empty());
        let bits = demodulate_chirp(&[], &config);
        assert!(bits.is_empty());
    }

    // ------------------------------------------------------------------
    // 6. OFDM round-trip
    // ------------------------------------------------------------------
    #[test]
    fn test_ofdm_modulate_demodulate_roundtrip() {
        let config = default_config();
        let usable = config.num_subcarriers - 2;
        // Use exactly one OFDM symbol worth of bits
        let bits: Vec<bool> = (0..usable).map(|i| i % 3 == 0).collect();
        let signal = modulate_ofdm_uwa(&bits, &config);
        let recovered = demodulate_ofdm_uwa(&signal, &config);
        assert_eq!(&recovered[..bits.len()], &bits[..]);
    }

    #[test]
    fn test_ofdm_multiple_symbols() {
        let config = default_config();
        let usable = config.num_subcarriers - 2;
        let num_symbols = 3;
        let bits: Vec<bool> = (0..usable * num_symbols).map(|i| i % 2 == 0).collect();
        let signal = modulate_ofdm_uwa(&bits, &config);
        let recovered = demodulate_ofdm_uwa(&signal, &config);
        assert_eq!(&recovered[..bits.len()], &bits[..]);
    }

    // ------------------------------------------------------------------
    // 7. Doppler estimation
    // ------------------------------------------------------------------
    #[test]
    fn test_doppler_estimate_zero() {
        let config = default_config();
        let preamble = generate_lfm_preamble(&config, 0.01);
        let est = estimate_doppler_scale(&preamble, &preamble);
        assert!(est.abs() < 0.001, "Zero Doppler should be estimated near 0: got {est}");
    }

    #[test]
    fn test_doppler_compensation_identity() {
        let config = default_config();
        let original = generate_lfm_preamble(&config, 0.01);
        let mut signal = original.clone();
        compensate_doppler(&mut signal, 0.0);
        assert_eq!(signal.len(), original.len());
    }

    // ------------------------------------------------------------------
    // 8. DFE
    // ------------------------------------------------------------------
    #[test]
    fn test_dfe_clean_signal() {
        let mut dfe = DfeEqualizer::new(5, 3, 0.01);
        // Clean BPSK signal: +1, -1, +1, -1, ...
        let input: Vec<f64> = (0..50).map(|i| if i % 2 == 0 { 1.0 } else { -1.0 }).collect();
        let output = dfe.equalize(&input);
        assert_eq!(output.len(), input.len());
        // After convergence, decisions should match the clean input
        let correct = output
            .iter()
            .zip(input.iter())
            .skip(5) // skip transient
            .filter(|(a, b)| (*a - *b).abs() < 1e-6)
            .count();
        let total = input.len() - 5;
        assert!(
            correct as f64 / total as f64 > 0.9,
            "DFE should converge on clean signal"
        );
    }

    #[test]
    fn test_dfe_creation() {
        let dfe = DfeEqualizer::new(7, 3, 0.05);
        assert_eq!(dfe.ff_taps.len(), 7);
        assert_eq!(dfe.fb_taps.len(), 3);
        assert!((dfe.ff_taps[3] - 1.0).abs() < 1e-12, "Centre tap should be 1.0");
    }

    // ------------------------------------------------------------------
    // 9. BER analysis
    // ------------------------------------------------------------------
    #[test]
    fn test_ber_analysis_high_snr() {
        let config = default_config();
        let result = ber_analysis_chirp(&config, 20.0, 50);
        assert!(result.ber < 0.1, "BER should be low at 20 dB SNR: got {}", result.ber);
        assert_eq!(result.num_bits, 50);
    }

    #[test]
    fn test_ber_result_fields() {
        let config = default_config();
        let result = ber_analysis_chirp(&config, 10.0, 30);
        assert_eq!(result.snr_db, 10.0);
        assert_eq!(result.num_bits, 30);
        assert!(result.ber >= 0.0 && result.ber <= 1.0);
        assert!(result.num_errors <= result.num_bits);
    }

    // ------------------------------------------------------------------
    // 10. Guard interval helpers
    // ------------------------------------------------------------------
    #[test]
    fn test_recommended_guard_interval() {
        let gi = recommended_guard_interval(0.010, 1.25);
        assert!((gi - 0.0125).abs() < 1e-9);
    }

    #[test]
    fn test_guard_samples() {
        let gs = guard_samples(0.010, 1.0, 100_000.0);
        assert_eq!(gs, 1000);
    }

    // ------------------------------------------------------------------
    // 11. UwaModem convenience
    // ------------------------------------------------------------------
    #[test]
    fn test_uwa_modem_chirp_roundtrip() {
        let modem = UwaModem::new(default_config());
        let bits = vec![true, true, false, true, false];
        let signal = modem.modulate_chirp(&bits);
        let recovered = modem.demodulate_chirp(&signal);
        assert_eq!(recovered, bits);
    }

    #[test]
    fn test_uwa_modem_ofdm_roundtrip() {
        let modem = UwaModem::new(default_config());
        let usable = modem.config.num_subcarriers - 2;
        let bits: Vec<bool> = (0..usable).map(|i| i % 5 == 0).collect();
        let signal = modem.modulate_ofdm(&bits);
        let recovered = modem.demodulate_ofdm(&signal);
        assert_eq!(&recovered[..bits.len()], &bits[..]);
    }

    #[test]
    fn test_uwa_modem_transmission_loss() {
        let modem = UwaModem::new(default_config());
        let tl = modem.transmission_loss(500.0, 1.5);
        assert!(tl > 0.0);
    }

    // ------------------------------------------------------------------
    // 12. FFT internal
    // ------------------------------------------------------------------
    #[test]
    fn test_fft_roundtrip() {
        let mut data: Vec<Complex> = (0..16)
            .map(|i| ((i as f64).sin(), (i as f64).cos()))
            .collect();
        let original = data.clone();
        fft_in_place(&mut data, false);
        fft_in_place(&mut data, true);
        for (a, b) in data.iter().zip(original.iter()) {
            assert!((a.0 - b.0).abs() < 1e-10);
            assert!((a.1 - b.1).abs() < 1e-10);
        }
    }

    // ------------------------------------------------------------------
    // 13. Resampler
    // ------------------------------------------------------------------
    #[test]
    fn test_resample_identity() {
        let input: Vec<Complex> = (0..100).map(|i| (i as f64, 0.0)).collect();
        let output = resample_signal(&input, 1.0);
        assert_eq!(output.len(), input.len());
    }
}
