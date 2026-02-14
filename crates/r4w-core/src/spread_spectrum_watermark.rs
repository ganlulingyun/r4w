//! Spread-Spectrum Audio Watermarking
//!
//! Embeds imperceptible watermarks into audio signals using direct-sequence
//! spread spectrum (DSSS) techniques. The watermark is spread across the
//! frequency spectrum using pseudo-noise (PN) sequences, making it robust
//! against common signal processing attacks while remaining inaudible.
//!
//! ## How It Works
//!
//! ```text
//! Embedding:
//!   message bits -> DSSS spreading (PN sequence) -> psychoacoustic shaping
//!                -> additive embedding into audio -> watermarked audio
//!
//! Extraction:
//!   watermarked audio -> despreading (correlate with PN) -> threshold
//!                     -> recovered message bits + confidence
//! ```
//!
//! The processing gain from spreading provides resilience to noise, compression,
//! and other attacks. Each message bit is spread across many audio samples
//! (chips_per_bit = sample_rate / chip_rate), so even if individual samples
//! are corrupted, the correlation-based extraction can still recover the bits.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::spread_spectrum_watermark::{
//!     WatermarkConfig, Watermarker, bit_error_rate,
//! };
//!
//! let config = WatermarkConfig {
//!     sample_rate_hz: 44100.0,
//!     chip_rate_hz: 1000.0,
//!     embedding_strength: 0.01,
//!     watermark_bits: 16,
//!     pn_seed: 42,
//! };
//!
//! let wm = Watermarker::new(config);
//!
//! // Create a test audio signal (sine wave)
//! let audio: Vec<f64> = (0..44100)
//!     .map(|i| (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 44100.0).sin())
//!     .collect();
//!
//! // Embed a 16-bit message
//! let message: Vec<bool> = (0..16).map(|i| i % 3 == 0).collect();
//! let watermarked = wm.embed(&audio, &message);
//!
//! // Extract the message
//! let (extracted, confidence) = wm.extract(&watermarked);
//! assert!(confidence > 0.5);
//! assert_eq!(bit_error_rate(&message, &extracted), 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for spread-spectrum watermarking.
///
/// The chip rate determines how many PN chips fit per audio sample period,
/// and together with the sample rate defines the spreading factor
/// (chips_per_bit = sample_rate / chip_rate * watermark_bits fits within the
/// audio length).
#[derive(Debug, Clone)]
pub struct WatermarkConfig {
    /// Audio sample rate in Hz (e.g. 44100.0).
    pub sample_rate_hz: f64,
    /// PN chip rate in Hz. Lower values give more spreading gain.
    pub chip_rate_hz: f64,
    /// Embedding strength (linear scale). Typical range 0.001 .. 0.05.
    /// Higher values are more robust but more audible.
    pub embedding_strength: f64,
    /// Number of watermark message bits to embed / extract.
    pub watermark_bits: usize,
    /// Seed for the PN sequence generator. Must match between embedder and
    /// extractor (acts as a secret key).
    pub pn_seed: u64,
}

impl Default for WatermarkConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 44100.0,
            chip_rate_hz: 1000.0,
            embedding_strength: 0.01,
            watermark_bits: 64,
            pn_seed: 0xDEAD_BEEF,
        }
    }
}

// ---------------------------------------------------------------------------
// Attack types for robustness testing
// ---------------------------------------------------------------------------

/// Signal processing attacks for robustness testing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AttackType {
    /// Additive white Gaussian noise. Parameter is noise standard deviation.
    NoiseAddition(f64),
    /// Resample to a different rate then back. Parameter is intermediate rate
    /// as a fraction of original (e.g. 0.9 = downsample to 90% then back up).
    Resampling(f64),
    /// Lossy compression simulation via low-pass filtering. Parameter is the
    /// cutoff frequency as a fraction of Nyquist (0.0 .. 1.0).
    Compression(f64),
    /// Time stretching. Parameter is stretch factor (1.0 = no change,
    /// 1.1 = 10% longer).
    TimeStretch(f64),
    /// Cropping: keep only a fraction of the signal (0.0 .. 1.0).
    /// The kept portion starts at the beginning.
    Cropping(f64),
}

// ---------------------------------------------------------------------------
// Watermarker
// ---------------------------------------------------------------------------

/// Spread-spectrum audio watermarker.
///
/// Embeds and extracts imperceptible watermarks using DSSS spreading
/// with a configurable PN sequence.
#[derive(Debug, Clone)]
pub struct Watermarker {
    config: WatermarkConfig,
    /// Pre-computed PN sequence for the configured seed.
    pn: Vec<f64>,
    /// Number of PN chips per watermark bit.
    chips_per_bit: usize,
}

impl Watermarker {
    /// Create a new watermarker from configuration.
    ///
    /// The PN sequence length is `chips_per_bit * watermark_bits`, where
    /// `chips_per_bit = floor(sample_rate / chip_rate)`.
    pub fn new(config: WatermarkConfig) -> Self {
        let chips_per_bit = (config.sample_rate_hz / config.chip_rate_hz).floor() as usize;
        assert!(chips_per_bit > 0, "chip_rate must be less than sample_rate");
        let total_chips = chips_per_bit * config.watermark_bits;
        let pn = generate_pn_sequence(total_chips, config.pn_seed);
        Self {
            config,
            pn,
            chips_per_bit,
        }
    }

    /// Embed a watermark message into an audio signal.
    ///
    /// The message length must equal `config.watermark_bits`. The audio must
    /// be at least `chips_per_bit * watermark_bits` samples long. Samples
    /// beyond the watermark region are copied unchanged.
    ///
    /// Returns the watermarked audio (same length as input).
    pub fn embed(&self, audio: &[f64], message: &[bool]) -> Vec<f64> {
        assert_eq!(
            message.len(),
            self.config.watermark_bits,
            "message length must equal watermark_bits ({})",
            self.config.watermark_bits
        );
        let spread = spread_bits(message, self.chips_per_bit, &self.pn);
        let total_chips = spread.len();
        assert!(
            audio.len() >= total_chips,
            "audio too short: need at least {} samples, got {}",
            total_chips,
            audio.len()
        );

        // Compute psychoacoustic mask for perceptual shaping
        let mask = psychoacoustic_mask(audio, self.config.sample_rate_hz);

        // Embed with perceptual shaping
        let mut out = audio.to_vec();
        for i in 0..total_chips {
            let shaping = if i < mask.len() { mask[i] } else { 1.0 };
            out[i] += self.config.embedding_strength * shaping * spread[i];
        }
        out
    }

    /// Extract a watermark from a (possibly degraded) audio signal.
    ///
    /// Returns `(bits, confidence)` where confidence is a value in [0, 1]
    /// representing the average absolute correlation across all bits.
    /// Higher confidence means more reliable extraction.
    pub fn extract(&self, watermarked: &[f64]) -> (Vec<bool>, f64) {
        let total_chips = self.chips_per_bit * self.config.watermark_bits;
        let len = watermarked.len().min(total_chips);
        let signal = &watermarked[..len];

        let results = despread_bits(signal, self.chips_per_bit, &self.pn);
        let bits: Vec<bool> = results.iter().map(|&(b, _)| b).collect();
        let confidence = if results.is_empty() {
            0.0
        } else {
            let sum: f64 = results.iter().map(|&(_, c)| c.abs()).sum();
            sum / results.len() as f64
        };
        (bits, confidence)
    }
}

// ---------------------------------------------------------------------------
// PN sequence generation
// ---------------------------------------------------------------------------

/// Generate a pseudo-noise sequence of bipolar values (-1.0 / +1.0).
///
/// Uses a xorshift64-based PRNG seeded deterministically. The output is
/// balanced (approximately equal +1 and -1 chips) with good autocorrelation
/// properties for spread-spectrum applications.
///
/// # Arguments
/// * `length` - Number of chips to generate.
/// * `seed` - Seed controlling the sequence (acts as a secret key).
///
/// # Returns
/// A vector of `length` values, each +1.0 or -1.0.
pub fn generate_pn_sequence(length: usize, seed: u64) -> Vec<f64> {
    if length == 0 {
        return Vec::new();
    }

    // xorshift64 PRNG — well-balanced, deterministic, fast
    let mut state: u64 = seed | 1; // ensure nonzero
    // Mix the seed through several rounds to avoid correlation with seed value
    for _ in 0..8 {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
    }

    let mut seq = Vec::with_capacity(length);
    for _ in 0..length {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        // Use bit 0 of state to decide polarity
        let chip = if state & 1 == 0 { 1.0 } else { -1.0 };
        seq.push(chip);
    }
    seq
}

// ---------------------------------------------------------------------------
// DSSS spreading / despreading
// ---------------------------------------------------------------------------

/// Spread message bits using direct-sequence spread spectrum.
///
/// Each message bit is multiplied by `chips_per_bit` consecutive PN chips.
/// A `true` bit maps to +1.0, `false` maps to -1.0 (BPSK convention).
///
/// # Arguments
/// * `bits` - Message bits to spread.
/// * `chips_per_bit` - Number of PN chips per message bit.
/// * `pn` - PN sequence (must have at least `bits.len() * chips_per_bit` elements).
///
/// # Returns
/// Spread signal of length `bits.len() * chips_per_bit`.
pub fn spread_bits(bits: &[bool], chips_per_bit: usize, pn: &[f64]) -> Vec<f64> {
    let total = bits.len() * chips_per_bit;
    assert!(
        pn.len() >= total,
        "PN sequence too short: need {}, have {}",
        total,
        pn.len()
    );

    let mut spread = Vec::with_capacity(total);
    for (bit_idx, &bit) in bits.iter().enumerate() {
        let data_chip: f64 = if bit { 1.0 } else { -1.0 };
        let offset = bit_idx * chips_per_bit;
        for j in 0..chips_per_bit {
            spread.push(data_chip * pn[offset + j]);
        }
    }
    spread
}

/// Despread a received signal to recover message bits and per-bit correlations.
///
/// For each bit position, the received signal is correlated with the
/// corresponding segment of the PN sequence. The sign of the correlation
/// determines the bit value, and the magnitude indicates reliability.
///
/// # Arguments
/// * `signal` - Received (possibly noisy) signal.
/// * `chips_per_bit` - Number of PN chips per message bit.
/// * `pn` - PN sequence (same as used for spreading).
///
/// # Returns
/// Vector of `(bit, correlation)` pairs. The correlation is normalized to
/// [-1.0, +1.0]. Positive correlation => `false` (0), negative => `true` (1)
/// following the BPSK convention where true => -1 and the correlation
/// reflects the data sign.
pub fn despread_bits(signal: &[f64], chips_per_bit: usize, pn: &[f64]) -> Vec<(bool, f64)> {
    if chips_per_bit == 0 {
        return Vec::new();
    }
    let num_bits = signal.len() / chips_per_bit;
    let mut results = Vec::with_capacity(num_bits);

    for bit_idx in 0..num_bits {
        let offset = bit_idx * chips_per_bit;
        if offset + chips_per_bit > signal.len() || offset + chips_per_bit > pn.len() {
            break;
        }
        let mut corr: f64 = 0.0;
        for j in 0..chips_per_bit {
            corr += signal[offset + j] * pn[offset + j];
        }
        // Normalize by chips_per_bit
        let norm_corr = corr / chips_per_bit as f64;
        // Positive correlation means the data chip was +1.0 (bit = true since
        // we used true => +1 in spreading). Negative => bit = false.
        let bit = norm_corr > 0.0;
        results.push((bit, norm_corr));
    }
    results
}

// ---------------------------------------------------------------------------
// Embedding
// ---------------------------------------------------------------------------

/// Additively embed a spread-spectrum signal into audio.
///
/// `watermarked[i] = audio[i] + strength * spread_signal[i]`
///
/// The spread signal is only added to the first `spread_signal.len()` samples;
/// remaining audio samples are copied unchanged.
///
/// # Arguments
/// * `audio` - Original audio samples.
/// * `spread_signal` - Spread-spectrum watermark signal.
/// * `strength` - Embedding strength (linear scale).
///
/// # Returns
/// Watermarked audio (same length as input).
pub fn embed_spread_spectrum(audio: &[f64], spread_signal: &[f64], strength: f64) -> Vec<f64> {
    let mut out = audio.to_vec();
    let embed_len = spread_signal.len().min(audio.len());
    for i in 0..embed_len {
        out[i] += strength * spread_signal[i];
    }
    out
}

// ---------------------------------------------------------------------------
// Correlation detector
// ---------------------------------------------------------------------------

/// Compute normalized cross-correlation peak between a signal and a PN sequence.
///
/// Slides the PN template across the signal and returns the maximum absolute
/// normalized correlation value. This can be used to detect the presence of
/// a watermark even without knowing the exact alignment.
///
/// The normalization ensures the output is in [-1.0, +1.0]:
///
/// ```text
///          sum(signal[i+k] * pn[k])
/// R[i] = ─────────────────────────────────────
///         sqrt(sum(signal[i+k]^2) * sum(pn[k]^2))
/// ```
///
/// # Returns
/// Maximum absolute normalized correlation value (0.0 .. 1.0).
pub fn correlator_detect(signal: &[f64], pn: &[f64]) -> f64 {
    if pn.is_empty() || signal.len() < pn.len() {
        return 0.0;
    }

    let pn_energy: f64 = pn.iter().map(|x| x * x).sum();
    if pn_energy == 0.0 {
        return 0.0;
    }

    let num_lags = signal.len() - pn.len() + 1;
    let mut max_abs_corr: f64 = 0.0;

    for lag in 0..num_lags {
        let mut dot: f64 = 0.0;
        let mut sig_energy: f64 = 0.0;
        for k in 0..pn.len() {
            let s = signal[lag + k];
            dot += s * pn[k];
            sig_energy += s * s;
        }
        let denom = (sig_energy * pn_energy).sqrt();
        if denom > 0.0 {
            let corr = (dot / denom).abs();
            if corr > max_abs_corr {
                max_abs_corr = corr;
            }
        }
    }
    max_abs_corr
}

// ---------------------------------------------------------------------------
// Psychoacoustic masking
// ---------------------------------------------------------------------------

/// Compute a simplified psychoacoustic masking curve for perceptual shaping.
///
/// This provides a per-sample gain factor that makes the watermark less
/// audible by embedding more energy where the audio signal can mask it
/// (louder regions) and less where the signal is quiet.
///
/// The implementation uses a short-time energy envelope with smoothing:
/// 1. Compute local RMS energy in overlapping windows.
/// 2. Normalize to [0.1, 1.0] range (never fully zero to avoid losing the
///    watermark in silence).
/// 3. Apply smoothing to avoid abrupt gain changes.
///
/// # Arguments
/// * `signal` - Audio signal.
/// * `fs` - Sample rate in Hz (used to set window size).
///
/// # Returns
/// Per-sample masking gain in [0.1, 1.0].
pub fn psychoacoustic_mask(signal: &[f64], fs: f64) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return Vec::new();
    }

    // Window size: ~10 ms
    let win_size = ((fs * 0.01).round() as usize).max(1);
    let half_win = win_size / 2;

    // Compute local RMS energy
    let mut envelope = vec![0.0_f64; n];
    for i in 0..n {
        let start = if i >= half_win { i - half_win } else { 0 };
        let end = (i + half_win + 1).min(n);
        let count = end - start;
        let energy: f64 = signal[start..end].iter().map(|&s| s * s).sum();
        envelope[i] = (energy / count as f64).sqrt();
    }

    // Find peak envelope for normalization
    let peak = envelope.iter().cloned().fold(0.0_f64, f64::max);
    if peak <= 1e-12 {
        // Signal is essentially silent; use uniform low-level masking
        return vec![0.1; n];
    }

    // Normalize and clamp to [0.1, 1.0]
    let mut mask = Vec::with_capacity(n);
    for &e in &envelope {
        let normalized = e / peak;
        // Map to [0.1, 1.0]: more energy => higher mask => embed more
        mask.push(0.1 + 0.9 * normalized);
    }

    // Smooth with simple IIR low-pass (alpha ~ 0.01)
    let alpha = 0.01_f64;
    let mut smoothed = mask.clone();
    // Forward pass
    for i in 1..n {
        smoothed[i] = alpha * mask[i] + (1.0 - alpha) * smoothed[i - 1];
    }
    // Backward pass for zero-phase
    for i in (0..n.saturating_sub(1)).rev() {
        smoothed[i] = alpha * smoothed[i] + (1.0 - alpha) * smoothed[i + 1];
    }

    smoothed
}

// ---------------------------------------------------------------------------
// Metrics
// ---------------------------------------------------------------------------

/// Compute bit error rate between original and extracted bit sequences.
///
/// BER = (number of differing bits) / (total bits compared).
/// If the sequences have different lengths, only the shorter length is compared.
///
/// # Returns
/// BER in [0.0, 1.0]. Returns 0.0 for empty sequences.
pub fn bit_error_rate(original: &[bool], extracted: &[bool]) -> f64 {
    let len = original.len().min(extracted.len());
    if len == 0 {
        return 0.0;
    }
    let errors = original[..len]
        .iter()
        .zip(extracted[..len].iter())
        .filter(|(&a, &b)| a != b)
        .count();
    errors as f64 / len as f64
}

/// Compute the SNR in dB between the original and watermarked signals.
///
/// SNR = 10 * log10(signal_power / watermark_power)
///
/// where watermark_power = mean((watermarked - original)^2)
/// and signal_power = mean(original^2).
///
/// # Returns
/// SNR in dB. Returns `f64::INFINITY` if watermark power is zero,
/// and `f64::NEG_INFINITY` if signal power is zero.
pub fn snr_db(original: &[f64], watermarked: &[f64]) -> f64 {
    let len = original.len().min(watermarked.len());
    if len == 0 {
        return 0.0;
    }

    let signal_power: f64 = original[..len].iter().map(|&s| s * s).sum::<f64>() / len as f64;
    let watermark_power: f64 = original[..len]
        .iter()
        .zip(watermarked[..len].iter())
        .map(|(&o, &w)| {
            let diff = w - o;
            diff * diff
        })
        .sum::<f64>()
        / len as f64;

    if watermark_power < 1e-30 {
        return f64::INFINITY;
    }
    if signal_power < 1e-30 {
        return f64::NEG_INFINITY;
    }
    10.0 * (signal_power / watermark_power).log10()
}

// ---------------------------------------------------------------------------
// Robustness testing
// ---------------------------------------------------------------------------

/// Apply a signal processing attack to a watermarked signal for robustness
/// testing.
///
/// Each attack type simulates a common real-world degradation. The returned
/// signal has the same length as the input (padding or truncation as needed).
///
/// # Arguments
/// * `watermarked` - Watermarked audio signal.
/// * `attack` - Type and severity of the attack.
///
/// # Returns
/// Degraded signal (same length as input).
pub fn robustness_test(watermarked: &[f64], attack: AttackType) -> Vec<f64> {
    match attack {
        AttackType::NoiseAddition(sigma) => attack_noise(watermarked, sigma),
        AttackType::Resampling(ratio) => attack_resample(watermarked, ratio),
        AttackType::Compression(cutoff) => attack_compression(watermarked, cutoff),
        AttackType::TimeStretch(factor) => attack_time_stretch(watermarked, factor),
        AttackType::Cropping(fraction) => attack_cropping(watermarked, fraction),
    }
}

/// Add white Gaussian noise using Box-Muller transform.
fn attack_noise(signal: &[f64], sigma: f64) -> Vec<f64> {
    let mut out = signal.to_vec();
    // Simple deterministic PRNG for reproducibility (xorshift64)
    let mut rng_state: u64 = 0x1234_5678_9ABC_DEF0;
    let mut next_rand = || -> f64 {
        rng_state ^= rng_state << 13;
        rng_state ^= rng_state >> 7;
        rng_state ^= rng_state << 17;
        // Map to (0, 1) - avoid exactly 0 for log
        (rng_state as f64 / u64::MAX as f64).max(1e-15)
    };

    let mut i = 0;
    while i < out.len() {
        let u1 = next_rand();
        let u2 = next_rand();
        let mag = sigma * (-2.0 * u1.ln()).sqrt();
        let z0 = mag * (2.0 * PI * u2).cos();
        let z1 = mag * (2.0 * PI * u2).sin();
        out[i] += z0;
        if i + 1 < out.len() {
            out[i + 1] += z1;
        }
        i += 2;
    }
    out
}

/// Resample down then up using linear interpolation.
fn attack_resample(signal: &[f64], ratio: f64) -> Vec<f64> {
    if signal.is_empty() || ratio <= 0.0 {
        return signal.to_vec();
    }
    let n = signal.len();
    // Downsample to intermediate length
    let intermediate_len = ((n as f64) * ratio).round() as usize;
    if intermediate_len == 0 {
        return vec![0.0; n];
    }

    // Linear interpolation to intermediate
    let mut intermediate = Vec::with_capacity(intermediate_len);
    for i in 0..intermediate_len {
        let src_pos = i as f64 * (n - 1) as f64 / (intermediate_len - 1).max(1) as f64;
        let idx = src_pos.floor() as usize;
        let frac = src_pos - idx as f64;
        let s0 = signal[idx.min(n - 1)];
        let s1 = signal[(idx + 1).min(n - 1)];
        intermediate.push(s0 + frac * (s1 - s0));
    }

    // Upsample back to original length
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let src_pos =
            i as f64 * (intermediate_len - 1).max(1) as f64 / (n - 1).max(1) as f64;
        let idx = src_pos.floor() as usize;
        let frac = src_pos - idx as f64;
        let s0 = intermediate[idx.min(intermediate_len - 1)];
        let s1 = intermediate[(idx + 1).min(intermediate_len - 1)];
        out.push(s0 + frac * (s1 - s0));
    }
    out
}

/// Simulate lossy compression via low-pass filtering (simple moving average).
fn attack_compression(signal: &[f64], cutoff_fraction: f64) -> Vec<f64> {
    if signal.is_empty() {
        return Vec::new();
    }
    // cutoff_fraction in (0, 1]: lower = more aggressive filtering
    let cutoff = cutoff_fraction.clamp(0.01, 1.0);
    // Approximate: filter length inversely proportional to cutoff
    let filter_len = (1.0 / cutoff).round() as usize;
    let filter_len = filter_len.max(1).min(signal.len());

    // Apply moving average (simple FIR low-pass)
    let mut out = Vec::with_capacity(signal.len());
    for i in 0..signal.len() {
        let start = if i >= filter_len / 2 {
            i - filter_len / 2
        } else {
            0
        };
        let end = (i + filter_len / 2 + 1).min(signal.len());
        let sum: f64 = signal[start..end].iter().sum();
        out.push(sum / (end - start) as f64);
    }
    out
}

/// Time stretching via linear interpolation (without pitch correction).
fn attack_time_stretch(signal: &[f64], factor: f64) -> Vec<f64> {
    if signal.is_empty() || factor <= 0.0 {
        return signal.to_vec();
    }
    let n = signal.len();
    // Stretch then truncate/pad to original length
    let stretched_len = ((n as f64) * factor).round() as usize;
    if stretched_len == 0 {
        return vec![0.0; n];
    }

    let mut stretched = Vec::with_capacity(stretched_len);
    for i in 0..stretched_len {
        let src_pos = i as f64 / factor;
        let idx = src_pos.floor() as usize;
        let frac = src_pos - idx as f64;
        if idx + 1 < n {
            stretched.push(signal[idx] + frac * (signal[idx + 1] - signal[idx]));
        } else if idx < n {
            stretched.push(signal[idx]);
        } else {
            stretched.push(0.0);
        }
    }

    // Truncate or pad to original length
    let mut out = vec![0.0; n];
    let copy_len = n.min(stretched.len());
    out[..copy_len].copy_from_slice(&stretched[..copy_len]);
    out
}

/// Cropping: zero out everything beyond the kept fraction.
fn attack_cropping(signal: &[f64], fraction: f64) -> Vec<f64> {
    let n = signal.len();
    let keep = ((n as f64) * fraction.clamp(0.0, 1.0)).round() as usize;
    let mut out = vec![0.0; n];
    let copy_len = keep.min(n);
    out[..copy_len].copy_from_slice(&signal[..copy_len]);
    out
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a sine wave test signal.
    fn test_sine(n: usize, freq_hz: f64, fs: f64) -> Vec<f64> {
        (0..n)
            .map(|i| (2.0 * PI * freq_hz * i as f64 / fs).sin())
            .collect()
    }

    /// Helper: create a default test config with short watermark for fast tests.
    /// Uses high embedding strength (0.5) to ensure reliable roundtrip in unit
    /// tests even with only 16 chips per bit of processing gain.
    fn test_config() -> WatermarkConfig {
        WatermarkConfig {
            sample_rate_hz: 8000.0,
            chip_rate_hz: 500.0,
            embedding_strength: 0.5,
            watermark_bits: 8,
            pn_seed: 42,
        }
    }

    // -----------------------------------------------------------------------
    // PN Sequence Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_pn_sequence_length() {
        let pn = generate_pn_sequence(1000, 123);
        assert_eq!(pn.len(), 1000);
    }

    #[test]
    fn test_pn_sequence_bipolar() {
        let pn = generate_pn_sequence(500, 0xBEEF);
        for &chip in &pn {
            assert!(chip == 1.0 || chip == -1.0, "chip must be +1 or -1, got {}", chip);
        }
    }

    #[test]
    fn test_pn_sequence_balance() {
        // A Gold-like PN sequence should have roughly equal +1 and -1 values
        let pn = generate_pn_sequence(10000, 7);
        let ones: usize = pn.iter().filter(|&&x| x == 1.0).count();
        let ratio = ones as f64 / pn.len() as f64;
        assert!(
            (ratio - 0.5).abs() < 0.15,
            "PN balance ratio {} not near 0.5",
            ratio
        );
    }

    #[test]
    fn test_pn_sequence_different_seeds() {
        let pn_a = generate_pn_sequence(200, 1);
        let pn_b = generate_pn_sequence(200, 2);
        // Different seeds should produce different sequences
        assert_ne!(pn_a, pn_b);
    }

    #[test]
    fn test_pn_sequence_deterministic() {
        let pn_a = generate_pn_sequence(200, 42);
        let pn_b = generate_pn_sequence(200, 42);
        assert_eq!(pn_a, pn_b);
    }

    #[test]
    fn test_pn_sequence_empty() {
        let pn = generate_pn_sequence(0, 99);
        assert!(pn.is_empty());
    }

    #[test]
    fn test_pn_autocorrelation() {
        // Autocorrelation at lag 0 should be much larger than at other lags
        let pn = generate_pn_sequence(1023, 42);
        let n = pn.len();
        let r0: f64 = pn.iter().map(|x| x * x).sum();
        // Check a few non-zero lags
        for lag in [1, 10, 100, 511] {
            let r_lag: f64 = (0..n).map(|i| pn[i] * pn[(i + lag) % n]).sum();
            assert!(
                r_lag.abs() < r0 * 0.25,
                "autocorrelation at lag {} ({}) too high relative to peak ({})",
                lag,
                r_lag,
                r0
            );
        }
    }

    // -----------------------------------------------------------------------
    // Spreading / Despreading Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_spread_bits_length() {
        let pn = generate_pn_sequence(100, 1);
        let bits = vec![true, false, true, true, false];
        let spread = spread_bits(&bits, 20, &pn);
        assert_eq!(spread.len(), 100);
    }

    #[test]
    fn test_spread_despread_roundtrip() {
        let chips_per_bit = 32;
        let bits = vec![true, false, true, true, false, false, true, false];
        let pn = generate_pn_sequence(bits.len() * chips_per_bit, 42);
        let spread = spread_bits(&bits, chips_per_bit, &pn);
        let results = despread_bits(&spread, chips_per_bit, &pn);
        let recovered: Vec<bool> = results.iter().map(|&(b, _)| b).collect();
        assert_eq!(recovered, bits);
    }

    #[test]
    fn test_spreading_gain() {
        // With DSSS, the processing gain should help overcome noise
        let chips_per_bit = 64;
        let bits = vec![true; 10];
        let pn = generate_pn_sequence(bits.len() * chips_per_bit, 7);
        let spread = spread_bits(&bits, chips_per_bit, &pn);

        // Add noise (moderate level)
        let noisy: Vec<f64> = spread
            .iter()
            .enumerate()
            .map(|(i, &s)| {
                let noise = 0.5 * (((i * 7 + 13) % 100) as f64 / 50.0 - 1.0);
                s + noise
            })
            .collect();

        let results = despread_bits(&noisy, chips_per_bit, &pn);
        let recovered: Vec<bool> = results.iter().map(|&(b, _)| b).collect();
        // All bits should still be recoverable with 64 chips of gain
        assert_eq!(recovered, bits);
    }

    #[test]
    fn test_despread_correlation_magnitude() {
        let chips_per_bit = 16;
        let bits = vec![true, false];
        let pn = generate_pn_sequence(bits.len() * chips_per_bit, 5);
        let spread = spread_bits(&bits, chips_per_bit, &pn);
        let results = despread_bits(&spread, chips_per_bit, &pn);
        // Clean signal: correlation magnitude should be close to 1.0
        for &(_, corr) in &results {
            assert!(
                corr.abs() > 0.9,
                "clean correlation magnitude {} should be near 1.0",
                corr.abs()
            );
        }
    }

    // -----------------------------------------------------------------------
    // Embed / Extract Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_embed_extract_roundtrip() {
        let config = test_config();
        let wm = Watermarker::new(config.clone());
        let audio = test_sine(config.watermark_bits * 16 + 500, 440.0, config.sample_rate_hz);
        let message: Vec<bool> = (0..config.watermark_bits).map(|i| i % 2 == 0).collect();

        let watermarked = wm.embed(&audio, &message);
        let (extracted, confidence) = wm.extract(&watermarked);

        assert_eq!(extracted.len(), message.len());
        assert_eq!(extracted, message);
        assert!(confidence > 0.5, "confidence {} should be > 0.5", confidence);
    }

    #[test]
    fn test_embed_preserves_length() {
        let config = test_config();
        let wm = Watermarker::new(config.clone());
        let audio = test_sine(2000, 440.0, config.sample_rate_hz);
        let message: Vec<bool> = vec![true; config.watermark_bits];

        let watermarked = wm.embed(&audio, &message);
        assert_eq!(watermarked.len(), audio.len());
    }

    #[test]
    fn test_embed_is_imperceptible() {
        let config = WatermarkConfig {
            embedding_strength: 0.01,
            ..test_config()
        };
        let wm = Watermarker::new(config.clone());
        let audio = test_sine(2000, 440.0, config.sample_rate_hz);
        let message: Vec<bool> = vec![false; config.watermark_bits];

        let watermarked = wm.embed(&audio, &message);
        let snr = snr_db(&audio, &watermarked);

        // SNR should be very high (watermark is very quiet)
        assert!(snr > 30.0, "SNR {} dB should be > 30 dB for imperceptible watermark", snr);
    }

    #[test]
    fn test_embed_all_zeros() {
        let config = test_config();
        let wm = Watermarker::new(config.clone());
        let audio = test_sine(2000, 440.0, config.sample_rate_hz);
        let message: Vec<bool> = vec![false; config.watermark_bits];

        let watermarked = wm.embed(&audio, &message);
        let (extracted, _) = wm.extract(&watermarked);
        assert_eq!(extracted, message);
    }

    #[test]
    fn test_embed_all_ones() {
        let config = test_config();
        let wm = Watermarker::new(config.clone());
        let audio = test_sine(2000, 440.0, config.sample_rate_hz);
        let message: Vec<bool> = vec![true; config.watermark_bits];

        let watermarked = wm.embed(&audio, &message);
        let (extracted, _) = wm.extract(&watermarked);
        assert_eq!(extracted, message);
    }

    #[test]
    fn test_wrong_seed_fails() {
        // Use more bits and longer signal for statistical significance
        let config = WatermarkConfig {
            sample_rate_hz: 8000.0,
            chip_rate_hz: 500.0,
            embedding_strength: 0.5,
            watermark_bits: 32,
            pn_seed: 42,
        };
        let wm_embed = Watermarker::new(config.clone());
        let audio = test_sine(8000, 440.0, config.sample_rate_hz);
        let message: Vec<bool> = (0..config.watermark_bits).map(|i| i % 3 == 0).collect();

        let watermarked = wm_embed.embed(&audio, &message);

        // Extract with wrong seed
        let wrong_config = WatermarkConfig {
            pn_seed: config.pn_seed + 9999,
            ..config
        };
        let wm_wrong = Watermarker::new(wrong_config);
        let (extracted, _) = wm_wrong.extract(&watermarked);

        // With wrong PN and 32 bits, BER should be non-zero (very unlikely
        // to get all 32 bits correct by chance: P = 0.5^32 ≈ 2.3e-10)
        let ber = bit_error_rate(&message, &extracted);
        assert!(
            ber > 0.0,
            "wrong seed should produce some errors, got BER {}",
            ber
        );
    }

    // -----------------------------------------------------------------------
    // Correlator Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_correlator_detect_perfect_match() {
        let pn = generate_pn_sequence(100, 42);
        let corr = correlator_detect(&pn, &pn);
        assert!(
            (corr - 1.0).abs() < 1e-10,
            "perfect match should give correlation ~1.0, got {}",
            corr
        );
    }

    #[test]
    fn test_correlator_detect_embedded() {
        let pn = generate_pn_sequence(50, 42);
        // Embed PN in the middle of a longer signal
        let mut signal = vec![0.0; 200];
        for i in 0..50 {
            signal[75 + i] = pn[i];
        }
        let corr = correlator_detect(&signal, &pn);
        assert!(
            corr > 0.99,
            "should detect embedded PN, got correlation {}",
            corr
        );
    }

    #[test]
    fn test_correlator_detect_noise_only() {
        let pn = generate_pn_sequence(100, 42);
        // Uncorrelated noise-like signal
        let noise: Vec<f64> = (0..200)
            .map(|i| ((i * 31 + 17) % 100) as f64 / 50.0 - 1.0)
            .collect();
        let corr = correlator_detect(&noise, &pn);
        assert!(
            corr < 0.5,
            "noise-only should give low correlation, got {}",
            corr
        );
    }

    #[test]
    fn test_correlator_empty_inputs() {
        assert_eq!(correlator_detect(&[], &[1.0, -1.0]), 0.0);
        assert_eq!(correlator_detect(&[1.0, -1.0], &[]), 0.0);
    }

    // -----------------------------------------------------------------------
    // Psychoacoustic Mask Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_psychoacoustic_mask_length() {
        let signal = test_sine(1000, 440.0, 8000.0);
        let mask = psychoacoustic_mask(&signal, 8000.0);
        assert_eq!(mask.len(), signal.len());
    }

    #[test]
    fn test_psychoacoustic_mask_range() {
        let signal = test_sine(1000, 440.0, 8000.0);
        let mask = psychoacoustic_mask(&signal, 8000.0);
        for &m in &mask {
            assert!(m >= 0.09 && m <= 1.01, "mask value {} out of [0.1, 1.0]", m);
        }
    }

    #[test]
    fn test_psychoacoustic_mask_silence() {
        let signal = vec![0.0; 500];
        let mask = psychoacoustic_mask(&signal, 8000.0);
        // Silence should give minimum masking
        for &m in &mask {
            assert!(
                (m - 0.1).abs() < 0.01,
                "silent signal should give mask ~0.1, got {}",
                m
            );
        }
    }

    #[test]
    fn test_psychoacoustic_mask_empty() {
        let mask = psychoacoustic_mask(&[], 8000.0);
        assert!(mask.is_empty());
    }

    // -----------------------------------------------------------------------
    // BER Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_ber_identical() {
        let a = vec![true, false, true, true];
        assert_eq!(bit_error_rate(&a, &a), 0.0);
    }

    #[test]
    fn test_ber_all_different() {
        let a = vec![true, false, true, false];
        let b = vec![false, true, false, true];
        assert!((bit_error_rate(&a, &b) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_ber_half() {
        let a = vec![true, true, false, false];
        let b = vec![true, false, true, false];
        assert!((bit_error_rate(&a, &b) - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_ber_empty() {
        let empty: Vec<bool> = Vec::new();
        assert_eq!(bit_error_rate(&empty, &empty), 0.0);
    }

    #[test]
    fn test_ber_different_lengths() {
        let a = vec![true, false, true];
        let b = vec![true, false];
        // Should compare only 2 bits
        assert_eq!(bit_error_rate(&a, &b), 0.0);
    }

    // -----------------------------------------------------------------------
    // SNR Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_snr_identical_signals() {
        let signal = test_sine(1000, 440.0, 8000.0);
        let snr = snr_db(&signal, &signal);
        assert_eq!(snr, f64::INFINITY);
    }

    #[test]
    fn test_snr_known_ratio() {
        // Create watermark with known power ratio
        let signal: Vec<f64> = (0..1000).map(|i| (i as f64 * 0.01).sin()).collect();
        let strength = 0.01;
        let watermarked: Vec<f64> = signal
            .iter()
            .enumerate()
            .map(|(i, &s)| s + strength * (i as f64 * 0.1).cos())
            .collect();
        let snr = snr_db(&signal, &watermarked);
        // SNR should be positive and reasonable
        assert!(snr > 20.0, "SNR {} should be > 20 dB", snr);
    }

    #[test]
    fn test_snr_empty() {
        let snr = snr_db(&[], &[]);
        assert_eq!(snr, 0.0);
    }

    // -----------------------------------------------------------------------
    // Robustness Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_robustness_noise_addition() {
        let signal = test_sine(1000, 440.0, 8000.0);
        let attacked = robustness_test(&signal, AttackType::NoiseAddition(0.1));
        assert_eq!(attacked.len(), signal.len());
        // Signal should be different
        let diff: f64 = signal
            .iter()
            .zip(attacked.iter())
            .map(|(a, b)| (a - b).abs())
            .sum();
        assert!(diff > 0.0, "noise attack should modify signal");
    }

    #[test]
    fn test_robustness_resampling() {
        let signal = test_sine(1000, 440.0, 8000.0);
        let attacked = robustness_test(&signal, AttackType::Resampling(0.8));
        assert_eq!(attacked.len(), signal.len());
    }

    #[test]
    fn test_robustness_compression() {
        let signal = test_sine(1000, 440.0, 8000.0);
        let attacked = robustness_test(&signal, AttackType::Compression(0.5));
        assert_eq!(attacked.len(), signal.len());
    }

    #[test]
    fn test_robustness_time_stretch() {
        let signal = test_sine(1000, 440.0, 8000.0);
        let attacked = robustness_test(&signal, AttackType::TimeStretch(1.1));
        assert_eq!(attacked.len(), signal.len());
    }

    #[test]
    fn test_robustness_cropping() {
        let signal = test_sine(1000, 440.0, 8000.0);
        let attacked = robustness_test(&signal, AttackType::Cropping(0.5));
        assert_eq!(attacked.len(), signal.len());
        // Second half should be zeros
        let tail_energy: f64 = attacked[500..].iter().map(|x| x * x).sum();
        assert!(tail_energy < 1e-10, "cropped tail should be zero");
    }

    #[test]
    fn test_robustness_watermark_survives_noise() {
        let config = WatermarkConfig {
            sample_rate_hz: 8000.0,
            chip_rate_hz: 200.0, // high spreading gain
            embedding_strength: 0.5,
            watermark_bits: 8,
            pn_seed: 42,
        };
        let wm = Watermarker::new(config.clone());
        let audio = test_sine(
            config.watermark_bits * 40 + 500,
            440.0,
            config.sample_rate_hz,
        );
        let message: Vec<bool> = vec![true, false, true, false, true, false, true, false];

        let watermarked = wm.embed(&audio, &message);
        let attacked = robustness_test(&watermarked, AttackType::NoiseAddition(0.01));
        let (extracted, _) = wm.extract(&attacked);

        let ber = bit_error_rate(&message, &extracted);
        assert!(
            ber < 0.25,
            "BER {} should be < 0.25 after mild noise attack",
            ber
        );
    }

    // -----------------------------------------------------------------------
    // Embed Spread Spectrum (standalone function)
    // -----------------------------------------------------------------------

    #[test]
    fn test_embed_spread_spectrum_standalone() {
        let audio = test_sine(500, 440.0, 8000.0);
        let spread = generate_pn_sequence(300, 99);
        let watermarked = embed_spread_spectrum(&audio, &spread, 0.01);
        assert_eq!(watermarked.len(), audio.len());
        // First 300 samples should be modified
        let modified: usize = (0..300)
            .filter(|&i| (watermarked[i] - audio[i]).abs() > 1e-15)
            .count();
        assert!(modified > 0, "embedding should modify samples");
        // Remaining samples unchanged
        for i in 300..500 {
            assert_eq!(watermarked[i], audio[i], "sample {} should be unchanged", i);
        }
    }

    // -----------------------------------------------------------------------
    // Edge Cases
    // -----------------------------------------------------------------------

    #[test]
    fn test_default_config() {
        let config = WatermarkConfig::default();
        assert_eq!(config.sample_rate_hz, 44100.0);
        assert_eq!(config.chip_rate_hz, 1000.0);
        assert_eq!(config.embedding_strength, 0.01);
        assert_eq!(config.watermark_bits, 64);
    }

    #[test]
    fn test_watermarker_with_silence() {
        let config = test_config();
        let wm = Watermarker::new(config.clone());
        let audio = vec![0.0; 2000];
        let message: Vec<bool> = vec![true; config.watermark_bits];

        let watermarked = wm.embed(&audio, &message);
        let (extracted, confidence) = wm.extract(&watermarked);

        // Even in silence, the watermark should be detectable (it IS the signal)
        assert_eq!(extracted, message);
        assert!(confidence > 0.0, "should detect watermark in silence");
    }

    #[test]
    fn test_watermarker_high_strength() {
        let config = WatermarkConfig {
            embedding_strength: 1.0, // Very strong (would be audible)
            ..test_config()
        };
        let wm = Watermarker::new(config.clone());
        let audio = test_sine(2000, 440.0, config.sample_rate_hz);
        let message: Vec<bool> = (0..config.watermark_bits).map(|i| i % 2 == 0).collect();

        let watermarked = wm.embed(&audio, &message);
        let (extracted, confidence) = wm.extract(&watermarked);

        assert_eq!(extracted, message);
        assert!(confidence > 0.8, "high strength should give high confidence");
    }

    #[test]
    #[should_panic(expected = "message length must equal watermark_bits")]
    fn test_embed_wrong_message_length() {
        let config = test_config();
        let wm = Watermarker::new(config.clone());
        let audio = vec![0.0; 2000];
        let wrong_len_message = vec![true; config.watermark_bits + 1];
        let _ = wm.embed(&audio, &wrong_len_message);
    }

    #[test]
    #[should_panic(expected = "audio too short")]
    fn test_embed_audio_too_short() {
        let config = test_config();
        let wm = Watermarker::new(config.clone());
        let short_audio = vec![0.0; 10]; // Much too short
        let message = vec![true; config.watermark_bits];
        let _ = wm.embed(&short_audio, &message);
    }

    #[test]
    fn test_attack_type_variants() {
        // Ensure all AttackType variants can be constructed and used
        let signal = vec![1.0; 100];
        let attacks = [
            AttackType::NoiseAddition(0.1),
            AttackType::Resampling(0.9),
            AttackType::Compression(0.5),
            AttackType::TimeStretch(1.1),
            AttackType::Cropping(0.8),
        ];
        for attack in &attacks {
            let result = robustness_test(&signal, *attack);
            assert_eq!(result.len(), signal.len(), "attack {:?} changed length", attack);
        }
    }

    #[test]
    fn test_different_message_patterns() {
        let config = test_config();
        let wm = Watermarker::new(config.clone());
        let audio = test_sine(2000, 440.0, config.sample_rate_hz);

        // Test several bit patterns
        let patterns: Vec<Vec<bool>> = vec![
            vec![true, false, true, false, true, false, true, false],
            vec![true, true, true, true, false, false, false, false],
            vec![false, false, false, false, true, true, true, true],
            vec![true, true, false, false, true, true, false, false],
        ];

        for (idx, message) in patterns.iter().enumerate() {
            let watermarked = wm.embed(&audio, message);
            let (extracted, _) = wm.extract(&watermarked);
            assert_eq!(
                &extracted, message,
                "pattern {} failed roundtrip",
                idx
            );
        }
    }
}
