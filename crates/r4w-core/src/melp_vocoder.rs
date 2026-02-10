//! Simplified MELP (Mixed Excitation Linear Prediction) vocoder for ultra-narrowband speech coding.
//!
//! This module implements a 2.4 kbps vocoder based on the MELP algorithm (MIL-STD-3005).
//! MELP achieves high-quality speech at very low bit rates by using mixed excitation
//! (blending voiced pulse trains and unvoiced noise per frequency band) combined with
//! Linear Predictive Coding (LPC) for spectral envelope modeling.
//!
//! # Overview
//!
//! - **Bit rate**: 2400 bps (54 bits per 22.5 ms frame)
//! - **Sample rate**: 8000 Hz
//! - **Frame size**: 180 samples (22.5 ms)
//! - **LPC order**: 10
//! - **Pitch range**: 50–500 Hz (16–160 samples)
//! - **Bandpass voicing**: 5 sub-bands with independent voiced/unvoiced decisions
//!
//! # Example
//!
//! ```
//! use r4w_core::melp_vocoder::{MelpEncoder, MelpDecoder, MelpConfig};
//!
//! let config = MelpConfig::default();
//! let mut encoder = MelpEncoder::new(config);
//! let mut decoder = MelpDecoder::new(config);
//!
//! // Generate a simple test signal (sine wave at 200 Hz)
//! let frame: Vec<f64> = (0..config.frame_size)
//!     .map(|i| 0.5 * (2.0 * std::f64::consts::PI * 200.0 * i as f64 / config.sample_rate as f64).sin())
//!     .collect();
//!
//! // Encode speech frame to compressed parameters
//! let melp_frame = encoder.analyze_frame(&frame);
//! assert_eq!(melp_frame.lpc_coefficients.len(), config.lpc_order);
//!
//! // Quantize to bitstream (54 bits = 2400 bps)
//! let bits = encoder.quantize_frame(&melp_frame);
//! assert_eq!(bits.len(), 54);
//!
//! // Dequantize and synthesize
//! let reconstructed_frame_params = decoder.dequantize_frame(&bits);
//! let output = decoder.synthesize_frame(&reconstructed_frame_params);
//! assert_eq!(output.len(), config.frame_size);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration parameters for the MELP vocoder.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MelpConfig {
    /// Sampling rate in Hz (typically 8000).
    pub sample_rate: usize,
    /// Number of samples per analysis/synthesis frame.
    pub frame_size: usize,
    /// Order of the LPC analysis.
    pub lpc_order: usize,
    /// Number of bandpass voicing sub-bands.
    pub num_bands: usize,
    /// Minimum pitch period in samples (corresponds to 500 Hz at 8 kHz).
    pub pitch_min: usize,
    /// Maximum pitch period in samples (corresponds to 50 Hz at 8 kHz).
    pub pitch_max: usize,
}

impl Default for MelpConfig {
    fn default() -> Self {
        Self {
            sample_rate: 8000,
            frame_size: 180, // 22.5 ms at 8 kHz
            lpc_order: 10,
            num_bands: 5,
            pitch_min: 16,  // 500 Hz
            pitch_max: 160, // 50 Hz
        }
    }
}

// ---------------------------------------------------------------------------
// MELP Frame
// ---------------------------------------------------------------------------

/// Parameters extracted from (or used to synthesize) one speech frame.
#[derive(Debug, Clone)]
pub struct MelpFrame {
    /// Pitch period in samples (0 means unvoiced).
    pub pitch_period: usize,
    /// Overall voiced/unvoiced flag.
    pub voiced_flag: bool,
    /// LPC coefficients \[a_1 .. a_p\].
    pub lpc_coefficients: Vec<f64>,
    /// Frame energy / gain (linear scale).
    pub gain: f64,
    /// Per-band voicing strengths \[0.0 .. 1.0\] for mixed excitation (5 bands).
    pub bandpass_voicing: Vec<f64>,
}

impl MelpFrame {
    /// Create an empty (silent) frame for the given config.
    pub fn silent(config: &MelpConfig) -> Self {
        Self {
            pitch_period: 0,
            voiced_flag: false,
            lpc_coefficients: vec![0.0; config.lpc_order],
            gain: 0.0,
            bandpass_voicing: vec![0.0; config.num_bands],
        }
    }
}

// ---------------------------------------------------------------------------
// Encoder
// ---------------------------------------------------------------------------

/// MELP speech encoder – analyses raw PCM frames into [`MelpFrame`] parameters.
#[derive(Debug)]
pub struct MelpEncoder {
    config: MelpConfig,
}

impl MelpEncoder {
    /// Create a new encoder with the given configuration.
    pub fn new(config: MelpConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the active configuration.
    pub fn config(&self) -> &MelpConfig {
        &self.config
    }

    /// Analyse one frame of PCM samples and return the MELP parameters.
    ///
    /// `samples` must contain exactly `config.frame_size` elements.
    pub fn analyze_frame(&self, samples: &[f64]) -> MelpFrame {
        assert_eq!(
            samples.len(),
            self.config.frame_size,
            "Frame must be exactly {} samples",
            self.config.frame_size
        );

        let gain = compute_gain(samples);
        if gain < 1e-10 {
            return MelpFrame::silent(&self.config);
        }

        let lpc = lpc_analysis(samples, self.config.lpc_order);
        let pitch = pitch_detection(samples, self.config.pitch_min, self.config.pitch_max);
        let voiced = pitch > 0;
        let bandpass_voicing = compute_bandpass_voicing(
            samples,
            self.config.sample_rate,
            self.config.num_bands,
            voiced,
        );

        MelpFrame {
            pitch_period: pitch,
            voiced_flag: voiced,
            lpc_coefficients: lpc,
            gain,
            bandpass_voicing,
        }
    }

    /// Quantize a [`MelpFrame`] into a bitstream.
    ///
    /// Returns a `Vec<u8>` of length 54 (each element is 0 or 1), yielding
    /// 54 bits / 22.5 ms = 2400 bps.
    ///
    /// Bit allocation:
    /// - Pitch period: 8 bits (log-uniform over 16..160)
    /// - Voiced flag: 1 bit
    /// - Gain: 5 bits (log-uniform)
    /// - LPC (as LSFs): 30 bits (3 bits each for 10 coefficients)
    /// - Bandpass voicing: 10 bits (2 bits x 5 bands)
    /// - Total: 54 bits
    pub fn quantize_frame(&self, frame: &MelpFrame) -> Vec<u8> {
        let mut bits = Vec::with_capacity(54);

        // Pitch period – 8 bits
        let pitch_q = if frame.pitch_period == 0 {
            0u8
        } else {
            let clamped = frame.pitch_period.clamp(self.config.pitch_min, self.config.pitch_max);
            let range = self.config.pitch_max - self.config.pitch_min;
            (((clamped - self.config.pitch_min) as f64 / range as f64) * 255.0).round() as u8
        };
        push_bits(&mut bits, pitch_q as u32, 8);

        // Voiced flag – 1 bit
        bits.push(if frame.voiced_flag { 1 } else { 0 });

        // Gain – 5 bits (log scale, 0..31)
        let gain_db = if frame.gain > 1e-10 {
            20.0 * frame.gain.log10()
        } else {
            -90.0
        };
        let gain_q = ((gain_db + 90.0) / 90.0 * 31.0).round().clamp(0.0, 31.0) as u8;
        push_bits(&mut bits, gain_q as u32, 5);

        // LPC as LSFs – 30 bits (3 bits x 10)
        let lsfs = lpc_to_lsf(&frame.lpc_coefficients);
        for &lsf in &lsfs {
            let q = ((lsf / PI) * 7.0).round().clamp(0.0, 7.0) as u8;
            push_bits(&mut bits, q as u32, 3);
        }

        // Bandpass voicing – 10 bits (2 bits x 5)
        for &bv in &frame.bandpass_voicing {
            let q = (bv * 3.0).round().clamp(0.0, 3.0) as u8;
            push_bits(&mut bits, q as u32, 2);
        }

        debug_assert_eq!(bits.len(), 54);
        bits
    }
}

// ---------------------------------------------------------------------------
// Decoder
// ---------------------------------------------------------------------------

/// MELP speech decoder – synthesizes PCM audio from [`MelpFrame`] parameters.
#[derive(Debug)]
pub struct MelpDecoder {
    config: MelpConfig,
    /// Phase accumulator for voiced excitation.
    pulse_phase: f64,
    /// Simple PRNG state for noise excitation.
    noise_state: u32,
}

impl MelpDecoder {
    /// Create a new decoder with the given configuration.
    pub fn new(config: MelpConfig) -> Self {
        Self {
            config,
            pulse_phase: 0.0,
            noise_state: 0x1234_5678,
        }
    }

    /// Return a reference to the active configuration.
    pub fn config(&self) -> &MelpConfig {
        &self.config
    }

    /// Dequantize a 54-bit bitstream back into a [`MelpFrame`].
    pub fn dequantize_frame(&self, bits: &[u8]) -> MelpFrame {
        assert_eq!(bits.len(), 54, "Bitstream must be 54 bits");

        let mut pos = 0;

        // Pitch – 8 bits
        let pitch_q = read_bits(bits, &mut pos, 8) as usize;
        let range = self.config.pitch_max - self.config.pitch_min;
        let pitch_period = if pitch_q == 0 {
            0
        } else {
            self.config.pitch_min + (pitch_q as f64 / 255.0 * range as f64).round() as usize
        };

        // Voiced flag – 1 bit
        let voiced_flag = bits[pos] != 0;
        pos += 1;

        // Gain – 5 bits
        let gain_q = read_bits(bits, &mut pos, 5);
        let gain_db = (gain_q as f64 / 31.0) * 90.0 - 90.0;
        let gain = 10.0_f64.powf(gain_db / 20.0);

        // LSFs – 30 bits (3 x 10)
        let mut lsfs = Vec::with_capacity(self.config.lpc_order);
        for _ in 0..self.config.lpc_order {
            let q = read_bits(bits, &mut pos, 3);
            lsfs.push(q as f64 / 7.0 * PI);
        }
        let lpc_coefficients = lsf_to_lpc(&lsfs);

        // Bandpass voicing – 10 bits (2 x 5)
        let mut bandpass_voicing = Vec::with_capacity(self.config.num_bands);
        for _ in 0..self.config.num_bands {
            let q = read_bits(bits, &mut pos, 2);
            bandpass_voicing.push(q as f64 / 3.0);
        }

        MelpFrame {
            pitch_period,
            voiced_flag,
            lpc_coefficients,
            gain,
            bandpass_voicing,
        }
    }

    /// Synthesize one frame of PCM audio from the given [`MelpFrame`] parameters.
    ///
    /// Returns `config.frame_size` samples.
    pub fn synthesize_frame(&mut self, frame: &MelpFrame) -> Vec<f64> {
        let n = self.config.frame_size;

        // Generate mixed excitation
        let excitation = mixed_excitation(
            n,
            frame.pitch_period,
            &frame.bandpass_voicing,
            self.config.sample_rate,
            &mut self.pulse_phase,
            &mut self.noise_state,
        );

        // LPC synthesis filter: s(n) = excitation(n) + sum_{k=1}^{p} a_k * s(n-k)
        let p = frame.lpc_coefficients.len();
        let mut output = vec![0.0; n];
        for i in 0..n {
            let mut sample = excitation[i] * frame.gain;
            for k in 0..p {
                if i > k {
                    sample += frame.lpc_coefficients[k] * output[i - 1 - k];
                }
            }
            output[i] = sample;
        }

        output
    }
}

// ---------------------------------------------------------------------------
// DSP helpers – public for reuse / testing
// ---------------------------------------------------------------------------

/// Autocorrelation-based pitch detection.
///
/// Returns the estimated pitch period in samples, or 0 if the frame is unvoiced.
/// Search range is `[pitch_min, pitch_max]` corresponding to approximately 500–50 Hz.
pub fn pitch_detection(samples: &[f64], pitch_min: usize, pitch_max: usize) -> usize {
    let n = samples.len();
    if n == 0 || pitch_max >= n {
        return 0;
    }

    // Compute autocorrelation for lags in the pitch range
    let r0 = autocorrelation(samples, 0);
    if r0 < 1e-12 {
        return 0;
    }

    let mut best_lag = 0;
    let mut best_val = 0.0f64;

    for lag in pitch_min..=pitch_max.min(n - 1) {
        let r = autocorrelation(samples, lag) / r0;
        if r > best_val {
            best_val = r;
            best_lag = lag;
        }
    }

    // Voiced decision: require normalised autocorrelation peak > 0.3
    if best_val > 0.3 {
        best_lag
    } else {
        0
    }
}

/// Levinson-Durbin recursion for LPC coefficient estimation.
///
/// Returns `order` LPC coefficients \[a_1 .. a_p\].
pub fn lpc_analysis(samples: &[f64], order: usize) -> Vec<f64> {
    let n = samples.len();
    if n == 0 || order == 0 {
        return vec![0.0; order];
    }

    // Autocorrelation method
    let mut r = vec![0.0; order + 1];
    for k in 0..=order {
        r[k] = autocorrelation(samples, k);
    }

    if r[0].abs() < 1e-12 {
        return vec![0.0; order];
    }

    // Levinson-Durbin
    let mut a = vec![0.0; order];
    let mut a_prev = vec![0.0; order];
    let mut error = r[0];

    for i in 0..order {
        // Compute reflection coefficient
        let mut lambda = 0.0;
        for j in 0..i {
            lambda += a[j] * r[i - j];
        }
        lambda = (r[i + 1] - lambda) / error;

        // Update coefficients
        a_prev[..order].copy_from_slice(&a[..order]);
        a[i] = lambda;
        for j in 0..i {
            a[j] = a_prev[j] - lambda * a_prev[i - 1 - j];
        }

        error *= 1.0 - lambda * lambda;
        if error < 1e-12 {
            break;
        }
    }

    a
}

/// Blend voiced (pulse train) and unvoiced (white noise) excitation per band.
///
/// This is the core of MELP: each of the 5 frequency bands can have an independent
/// mix of periodic and aperiodic excitation, producing more natural speech than a
/// simple binary voiced/unvoiced decision.
pub fn mixed_excitation(
    length: usize,
    pitch_period: usize,
    bandpass_voicing: &[f64],
    sample_rate: usize,
    pulse_phase: &mut f64,
    noise_state: &mut u32,
) -> Vec<f64> {
    let num_bands = bandpass_voicing.len();
    let mut output = vec![0.0; length];

    // Generate full-band voiced and unvoiced excitation
    let voiced_exc = generate_pulse_train(length, pitch_period, pulse_phase);
    let unvoiced_exc = generate_white_noise(length, noise_state);

    // Band edges (normalised to Nyquist = sample_rate / 2)
    let nyquist = sample_rate as f64 / 2.0;
    let band_edges: Vec<(f64, f64)> = (0..num_bands)
        .map(|b| {
            let lo = b as f64 / num_bands as f64 * nyquist;
            let hi = (b + 1) as f64 / num_bands as f64 * nyquist;
            (lo, hi)
        })
        .collect();

    // For each band, mix voiced and unvoiced then bandpass-filter (simplified)
    for (band_idx, &(lo, hi)) in band_edges.iter().enumerate() {
        let v = bandpass_voicing[band_idx].clamp(0.0, 1.0);
        let centre = (lo + hi) / 2.0;
        let omega = 2.0 * PI * centre / sample_rate as f64;

        // Simple mixing with band-centre modulation (cosine weighting approximation)
        for i in 0..length {
            let mixed = v * voiced_exc[i] + (1.0 - v) * unvoiced_exc[i];
            // Bandpass shaping via cosine modulation (crude but functional)
            let bp = (omega * i as f64).cos();
            output[i] += mixed * bp / num_bands as f64;
        }
    }

    output
}

/// Quantize and pack a [`MelpFrame`] into a compact bitstream — delegates to
/// [`MelpEncoder::quantize_frame`].
pub fn quantize_frame(encoder: &MelpEncoder, frame: &MelpFrame) -> Vec<u8> {
    encoder.quantize_frame(frame)
}

/// Dequantize a bitstream back into a [`MelpFrame`] — delegates to
/// [`MelpDecoder::dequantize_frame`].
pub fn dequantize_frame(decoder: &MelpDecoder, bits: &[u8]) -> MelpFrame {
    decoder.dequantize_frame(bits)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Autocorrelation at a given lag.
fn autocorrelation(samples: &[f64], lag: usize) -> f64 {
    let n = samples.len();
    let mut sum = 0.0;
    for i in 0..n - lag {
        sum += samples[i] * samples[i + lag];
    }
    sum
}

/// Compute RMS gain of a frame.
fn compute_gain(samples: &[f64]) -> f64 {
    let n = samples.len();
    if n == 0 {
        return 0.0;
    }
    let energy: f64 = samples.iter().map(|&s| s * s).sum();
    (energy / n as f64).sqrt()
}

/// Estimate per-band voicing by comparing band energy to overall energy.
fn compute_bandpass_voicing(
    samples: &[f64],
    sample_rate: usize,
    num_bands: usize,
    voiced: bool,
) -> Vec<f64> {
    if !voiced {
        return vec![0.0; num_bands];
    }

    // Simple DFT-based band energy estimation
    let n = samples.len();
    let nyquist = sample_rate as f64 / 2.0;
    let fft_size = n;

    // Compute magnitude spectrum (simplified DFT for band energies)
    let mut band_energy = vec![0.0; num_bands];
    let total_energy: f64 = samples.iter().map(|&s| s * s).sum();

    if total_energy < 1e-12 {
        return vec![0.0; num_bands];
    }

    for k in 0..fft_size / 2 {
        let freq = k as f64 * sample_rate as f64 / fft_size as f64;
        let band = ((freq / nyquist) * num_bands as f64).floor() as usize;
        let band = band.min(num_bands - 1);

        // Goertzel-style single-bin energy
        let omega = 2.0 * PI * k as f64 / fft_size as f64;
        let mut re = 0.0;
        let mut im = 0.0;
        for (i, &s) in samples.iter().enumerate() {
            re += s * (omega * i as f64).cos();
            im -= s * (omega * i as f64).sin();
        }
        band_energy[band] += re * re + im * im;
    }

    // Normalise to [0, 1]
    let max_e = band_energy.iter().cloned().fold(0.0f64, f64::max);
    if max_e < 1e-12 {
        return vec![0.0; num_bands];
    }
    band_energy.iter().map(|&e| (e / max_e).clamp(0.0, 1.0)).collect()
}

/// Generate a pulse train (voiced excitation).
fn generate_pulse_train(length: usize, pitch_period: usize, phase: &mut f64) -> Vec<f64> {
    let mut out = vec![0.0; length];
    if pitch_period == 0 {
        return out;
    }
    let period = pitch_period as f64;
    for i in 0..length {
        *phase += 1.0;
        if *phase >= period {
            *phase -= period;
            out[i] = 1.0;
        }
    }
    out
}

/// Generate white noise (unvoiced excitation) using a simple xorshift PRNG.
fn generate_white_noise(length: usize, state: &mut u32) -> Vec<f64> {
    let mut out = Vec::with_capacity(length);
    for _ in 0..length {
        *state ^= *state << 13;
        *state ^= *state >> 17;
        *state ^= *state << 5;
        // Map u32 to [-1, 1)
        let f = (*state as f64) / (u32::MAX as f64) * 2.0 - 1.0;
        out.push(f);
    }
    out
}

/// Convert LPC coefficients to Line Spectral Frequencies (LSFs).
///
/// LSFs are the angular positions of the roots of the symmetric and
/// antisymmetric polynomials derived from the LPC polynomial.
fn lpc_to_lsf(lpc: &[f64]) -> Vec<f64> {
    let p = lpc.len();
    if p == 0 {
        return vec![];
    }

    // Build A(z) polynomial: 1, a1, a2, ..., ap
    let mut a = Vec::with_capacity(p + 1);
    a.push(1.0);
    a.extend_from_slice(lpc);

    // Symmetric polynomial P(z) = A(z) + z^{-(p+1)} A(z^{-1})
    // Antisymmetric polynomial Q(z) = A(z) - z^{-(p+1)} A(z^{-1})
    let mut p_poly = vec![0.0; p + 2];
    let mut q_poly = vec![0.0; p + 2];

    for i in 0..=p {
        p_poly[i] = a[i] + a[p - i];
        q_poly[i] = a[i] - a[p - i];
    }

    // Find roots by evaluating on the unit circle and detecting sign changes
    let mut lsfs = Vec::with_capacity(p);
    let num_points = 1024;

    for poly in &[&p_poly, &q_poly] {
        let mut prev_val = eval_poly(poly, 0.0);
        for k in 1..=num_points {
            let omega = PI * k as f64 / num_points as f64;
            let val = eval_poly(poly, omega);
            if prev_val * val < 0.0 {
                // Linear interpolation for root
                let omega_prev = PI * (k - 1) as f64 / num_points as f64;
                let root = omega_prev + (omega - omega_prev) * prev_val.abs() / (prev_val.abs() + val.abs());
                lsfs.push(root);
            }
            prev_val = val;
        }
    }

    lsfs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    lsfs.truncate(p);

    // Ensure we always return exactly p values
    while lsfs.len() < p {
        let idx = lsfs.len();
        lsfs.push((idx + 1) as f64 * PI / (p + 1) as f64);
    }

    lsfs
}

/// Evaluate a polynomial at a point on the unit circle: sum_k c_k cos(k*omega).
fn eval_poly(coeffs: &[f64], omega: f64) -> f64 {
    let mut val = 0.0;
    for (k, &c) in coeffs.iter().enumerate() {
        val += c * (k as f64 * omega).cos();
    }
    val
}

/// Convert Line Spectral Frequencies back to LPC coefficients.
///
/// Reconstructs the LPC polynomial from the roots of the symmetric and
/// antisymmetric polynomials.
fn lsf_to_lpc(lsfs: &[f64]) -> Vec<f64> {
    let p = lsfs.len();
    if p == 0 {
        return vec![];
    }

    // Separate LSFs into P and Q groups (alternating)
    let mut p_roots = Vec::new();
    let mut q_roots = Vec::new();
    for (i, &lsf) in lsfs.iter().enumerate() {
        if i % 2 == 0 {
            p_roots.push(lsf);
        } else {
            q_roots.push(lsf);
        }
    }

    // Build P(z) and Q(z) from their roots
    let p_poly = roots_to_poly(&p_roots);
    let q_poly = roots_to_poly(&q_roots);

    // A(z) = 0.5 * (P(z) + Q(z))
    let n = p + 1;
    let mut a = vec![0.0; n];
    for i in 0..n.min(p_poly.len()).min(q_poly.len()) {
        a[i] = 0.5 * (p_poly[i] + q_poly[i]);
    }

    // Return a_1 .. a_p (skip a_0 which should be ~1.0)
    if a.len() > 1 {
        a[1..].to_vec()
    } else {
        vec![0.0; p]
    }
}

/// Build a polynomial from angular roots on the unit circle.
///
/// Each root at angle omega contributes factor (1 - 2cos(omega)z^{-1} + z^{-2}).
fn roots_to_poly(roots: &[f64]) -> Vec<f64> {
    let mut poly = vec![1.0];
    for &root in roots {
        let b = [-2.0 * root.cos(), 1.0];
        let mut new_poly = vec![0.0; poly.len() + 2];
        for (i, &p) in poly.iter().enumerate() {
            new_poly[i] += p;
            new_poly[i + 1] += p * b[0];
            new_poly[i + 2] += p * b[1];
        }
        poly = new_poly;
    }
    poly
}

/// Push `num_bits` bits (MSB first) from `value` into a bit vector.
fn push_bits(bits: &mut Vec<u8>, value: u32, num_bits: usize) {
    for i in (0..num_bits).rev() {
        bits.push(((value >> i) & 1) as u8);
    }
}

/// Read `num_bits` bits (MSB first) from a bit slice, advancing `pos`.
fn read_bits(bits: &[u8], pos: &mut usize, num_bits: usize) -> u32 {
    let mut value = 0u32;
    for _ in 0..num_bits {
        value = (value << 1) | (bits[*pos] as u32 & 1);
        *pos += 1;
    }
    value
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn sine_frame(freq: f64, config: &MelpConfig) -> Vec<f64> {
        (0..config.frame_size)
            .map(|i| {
                0.8 * (2.0 * PI * freq * i as f64 / config.sample_rate as f64).sin()
            })
            .collect()
    }

    fn silent_frame(config: &MelpConfig) -> Vec<f64> {
        vec![0.0; config.frame_size]
    }

    #[test]
    fn test_default_config() {
        let cfg = MelpConfig::default();
        assert_eq!(cfg.sample_rate, 8000);
        assert_eq!(cfg.frame_size, 180);
        assert_eq!(cfg.lpc_order, 10);
        assert_eq!(cfg.num_bands, 5);
        assert_eq!(cfg.pitch_min, 16);
        assert_eq!(cfg.pitch_max, 160);
        // Verify 2400 bps: 54 bits per 22.5 ms frame
        let frame_duration = cfg.frame_size as f64 / cfg.sample_rate as f64;
        let bps = 54.0 / frame_duration;
        assert!((bps - 2400.0).abs() < 1.0);
    }

    #[test]
    fn test_encoder_decoder_roundtrip_length() {
        let config = MelpConfig::default();
        let encoder = MelpEncoder::new(config);
        let mut decoder = MelpDecoder::new(config);

        let frame = sine_frame(200.0, &config);
        let melp = encoder.analyze_frame(&frame);
        let bits = encoder.quantize_frame(&melp);
        assert_eq!(bits.len(), 54);

        let recon_params = decoder.dequantize_frame(&bits);
        let output = decoder.synthesize_frame(&recon_params);
        assert_eq!(output.len(), config.frame_size);
    }

    #[test]
    fn test_silent_frame_analysis() {
        let config = MelpConfig::default();
        let encoder = MelpEncoder::new(config);

        let frame = silent_frame(&config);
        let melp = encoder.analyze_frame(&frame);
        assert!(!melp.voiced_flag);
        assert_eq!(melp.pitch_period, 0);
        assert!(melp.gain < 1e-6);
    }

    #[test]
    fn test_voiced_detection_for_periodic_signal() {
        let config = MelpConfig::default();
        let encoder = MelpEncoder::new(config);

        // 200 Hz tone -> expected pitch period = 8000/200 = 40 samples
        let frame = sine_frame(200.0, &config);
        let melp = encoder.analyze_frame(&frame);
        assert!(melp.voiced_flag, "200 Hz sine should be detected as voiced");
        // Pitch should be close to 40
        let expected = (config.sample_rate as f64 / 200.0).round() as usize;
        let diff = (melp.pitch_period as i64 - expected as i64).unsigned_abs() as usize;
        assert!(diff <= 2, "Pitch {} should be close to {}", melp.pitch_period, expected);
    }

    #[test]
    fn test_unvoiced_detection_for_noise() {
        let config = MelpConfig::default();
        let encoder = MelpEncoder::new(config);

        // White noise should be unvoiced
        let mut state = 0xDEAD_BEEFu32;
        let frame: Vec<f64> = (0..config.frame_size)
            .map(|_| {
                state ^= state << 13;
                state ^= state >> 17;
                state ^= state << 5;
                state as f64 / u32::MAX as f64 * 2.0 - 1.0
            })
            .collect();

        let melp = encoder.analyze_frame(&frame);
        assert!(!melp.voiced_flag, "White noise should be unvoiced");
    }

    #[test]
    fn test_lpc_analysis_order() {
        let config = MelpConfig::default();
        let frame = sine_frame(300.0, &config);
        let lpc = lpc_analysis(&frame, config.lpc_order);
        assert_eq!(lpc.len(), config.lpc_order);

        // LPC coefficients should be finite
        for &c in &lpc {
            assert!(c.is_finite(), "LPC coefficient must be finite, got {}", c);
        }
    }

    #[test]
    fn test_pitch_detection_range() {
        let config = MelpConfig::default();

        // Test several frequencies within the pitch range
        for &freq in &[100.0, 150.0, 200.0, 300.0, 400.0] {
            let frame = sine_frame(freq, &config);
            let pitch = pitch_detection(&frame, config.pitch_min, config.pitch_max);
            if pitch > 0 {
                let detected_freq = config.sample_rate as f64 / pitch as f64;
                let ratio = detected_freq / freq;
                assert!(
                    (0.8..=1.25).contains(&ratio),
                    "Detected freq {:.1} Hz for input {:.1} Hz (ratio {:.2})",
                    detected_freq,
                    freq,
                    ratio,
                );
            }
        }
    }

    #[test]
    fn test_quantize_dequantize_voiced_flag() {
        let config = MelpConfig::default();
        let encoder = MelpEncoder::new(config);
        let decoder = MelpDecoder::new(config);

        // Voiced frame
        let mut frame = MelpFrame::silent(&config);
        frame.voiced_flag = true;
        frame.pitch_period = 80;
        frame.gain = 0.5;
        let bits = encoder.quantize_frame(&frame);
        let recon = decoder.dequantize_frame(&bits);
        assert!(recon.voiced_flag);

        // Unvoiced frame
        frame.voiced_flag = false;
        let bits = encoder.quantize_frame(&frame);
        let recon = decoder.dequantize_frame(&bits);
        assert!(!recon.voiced_flag);
    }

    #[test]
    fn test_quantize_dequantize_pitch_preservation() {
        let config = MelpConfig::default();
        let encoder = MelpEncoder::new(config);
        let decoder = MelpDecoder::new(config);

        for &pitch in &[20, 40, 80, 120, 160] {
            let mut frame = MelpFrame::silent(&config);
            frame.pitch_period = pitch;
            frame.voiced_flag = true;
            frame.gain = 0.1;

            let bits = encoder.quantize_frame(&frame);
            let recon = decoder.dequantize_frame(&bits);

            let diff = (recon.pitch_period as i64 - pitch as i64).unsigned_abs() as usize;
            assert!(
                diff <= 2,
                "Pitch {} should survive quantisation (got {})",
                pitch,
                recon.pitch_period
            );
        }
    }

    #[test]
    fn test_mixed_excitation_length() {
        let mut phase = 0.0;
        let mut noise_state = 42u32;
        let voicing = vec![1.0, 0.5, 0.0, 0.5, 1.0];

        let exc = mixed_excitation(180, 40, &voicing, 8000, &mut phase, &mut noise_state);
        assert_eq!(exc.len(), 180);

        // All values should be finite
        for &s in &exc {
            assert!(s.is_finite());
        }
    }

    #[test]
    fn test_bandpass_voicing_bands() {
        let config = MelpConfig::default();
        let frame = sine_frame(200.0, &config);
        let bv = compute_bandpass_voicing(&frame, config.sample_rate, config.num_bands, true);
        assert_eq!(bv.len(), config.num_bands);

        for &v in &bv {
            assert!((0.0..=1.0).contains(&v), "Voicing must be in [0,1], got {}", v);
        }
    }

    #[test]
    fn test_bitstream_all_zeros_and_ones() {
        // Verify push_bits / read_bits round-trip for edge cases
        let mut bits = Vec::new();
        push_bits(&mut bits, 0, 8);
        push_bits(&mut bits, 255, 8);
        assert_eq!(bits.len(), 16);

        let mut pos = 0;
        assert_eq!(read_bits(&bits, &mut pos, 8), 0);
        assert_eq!(read_bits(&bits, &mut pos, 8), 255);
    }

    #[test]
    fn test_synthesize_non_zero_output_for_voiced() {
        let config = MelpConfig::default();
        let mut decoder = MelpDecoder::new(config);

        let mut frame = MelpFrame::silent(&config);
        frame.voiced_flag = true;
        frame.pitch_period = 40;
        frame.gain = 0.5;
        frame.bandpass_voicing = vec![1.0; config.num_bands];

        let output = decoder.synthesize_frame(&frame);
        let energy: f64 = output.iter().map(|s| s * s).sum();
        assert!(energy > 0.0, "Voiced synthesis should produce non-zero output");
    }

    #[test]
    fn test_full_pipeline_encode_decode() {
        let config = MelpConfig::default();
        let encoder = MelpEncoder::new(config);
        let mut decoder = MelpDecoder::new(config);

        // Multi-frame encode/decode pipeline
        for &freq in &[150.0, 250.0, 350.0] {
            let input = sine_frame(freq, &config);
            let melp = encoder.analyze_frame(&input);
            let bits = encoder.quantize_frame(&melp);
            let recon_params = decoder.dequantize_frame(&bits);
            let output = decoder.synthesize_frame(&recon_params);

            assert_eq!(output.len(), config.frame_size);
            // Output should have non-trivial energy
            let energy: f64 = output.iter().map(|s| s * s).sum();
            assert!(energy > 0.0, "Output should be non-silent for {} Hz input", freq);
        }
    }
}
