//! # JT65 Weak-Signal Digital Mode Modulator/Demodulator
//!
//! Implements the JT65 amateur radio weak-signal digital protocol using 65-FSK
//! modulation. JT65 is designed for extremely weak signal communication (down to
//! -28 dB SNR) and is widely used for EME (Earth-Moon-Earth), tropospheric scatter,
//! and HF DX contacts.
//!
//! ## Protocol Overview
//!
//! - **Modulation**: 65-FSK (65 tones)
//! - **Symbol rate**: ~2.69 baud (4096 samples at 11025 Hz per symbol)
//! - **Message length**: 126 symbols (63 data + 63 sync, interleaved)
//! - **Character set**: 0-9, A-Z, space, +, -, ., /, ? (42 chars)
//! - **Submodes**: A (2.69 Hz spacing), B (5.38 Hz), C (10.77 Hz)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::jt65_modulator::{Jt65Modulator, Jt65Submode};
//!
//! let modulator = Jt65Modulator::new(11025.0, Jt65Submode::A);
//!
//! // Encode a message into symbols
//! let symbols = modulator.encode_message("CQ DX K1ABC");
//! assert_eq!(symbols.len(), 126);
//!
//! // Modulate symbols into IQ samples
//! let iq_samples = modulator.modulate(&symbols);
//! assert!(!iq_samples.is_empty());
//!
//! // Demodulate back to symbols
//! let (decoded_symbols, _sync_offset) = modulator.demodulate(&iq_samples);
//! assert_eq!(decoded_symbols.len(), 126);
//! ```

use std::f64::consts::PI;

/// The standard JT65 sample rate in Hz.
pub const STANDARD_SAMPLE_RATE: f64 = 11025.0;

/// Samples per symbol at the standard sample rate (4096).
pub const SAMPLES_PER_SYMBOL: usize = 4096;

/// Total symbols in a JT65 transmission.
pub const TOTAL_SYMBOLS: usize = 126;

/// Number of data symbols in a JT65 transmission.
pub const DATA_SYMBOLS: usize = 63;

/// Number of sync symbols in a JT65 transmission.
pub const SYNC_SYMBOLS: usize = 63;

/// Number of possible tones (0-64, where 0 is sync tone).
pub const NUM_TONES: usize = 65;

/// Base tone frequency offset from carrier (Hz).
pub const BASE_TONE_OFFSET: f64 = 1270.5;

/// Number of characters in the JT65 character set.
pub const CHARSET_SIZE: usize = 42;

/// Maximum message length in characters.
pub const MAX_MESSAGE_LEN: usize = 13;

/// The JT65 character set: 0-9, A-Z, space, +, -, ., /, ?
const CHARSET: &[u8; 42] = b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ +-./?";

/// JT65 sync vector (126 bits). 1 = sync symbol position, 0 = data symbol position.
/// This is the standard JT65 pseudo-random sync pattern.
const SYNC_VECTOR: [u8; 126] = [
    0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1,
    0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0,
    1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1,
    0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0,
    0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1,
    0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1,
    0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1,
    1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 0,
];

/// JT65 submode determining tone spacing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Jt65Submode {
    /// Submode A: 2.6917 Hz tone spacing (narrowest, best for EME)
    A,
    /// Submode B: 5.3833 Hz tone spacing (2x A)
    B,
    /// Submode C: 10.7666 Hz tone spacing (4x A, widest)
    C,
}

impl Jt65Submode {
    /// Returns the tone spacing in Hz for this submode.
    pub fn tone_spacing(&self) -> f64 {
        let base = STANDARD_SAMPLE_RATE / SAMPLES_PER_SYMBOL as f64;
        match self {
            Jt65Submode::A => base,
            Jt65Submode::B => 2.0 * base,
            Jt65Submode::C => 4.0 * base,
        }
    }

    /// Returns the bandwidth occupied by the 65-FSK signal in Hz.
    pub fn bandwidth(&self) -> f64 {
        self.tone_spacing() * (NUM_TONES as f64 - 1.0)
    }

    /// Returns the submode multiplier (1, 2, or 4).
    pub fn multiplier(&self) -> usize {
        match self {
            Jt65Submode::A => 1,
            Jt65Submode::B => 2,
            Jt65Submode::C => 4,
        }
    }
}

/// JT65 modulator and demodulator.
///
/// Handles encoding messages into 65-FSK symbols, modulating symbols into IQ
/// samples, demodulating IQ samples back to symbols, and decoding symbols into
/// messages.
#[derive(Debug, Clone)]
pub struct Jt65Modulator {
    /// Sample rate in Hz.
    sample_rate: f64,
    /// JT65 submode (A, B, or C).
    submode: Jt65Submode,
    /// Samples per symbol, scaled from the standard rate.
    samples_per_symbol: usize,
}

impl Jt65Modulator {
    /// Creates a new JT65 modulator with the given sample rate and submode.
    ///
    /// # Arguments
    /// * `sample_rate` - Sample rate in Hz (standard is 11025.0)
    /// * `submode` - JT65 submode (A, B, or C)
    pub fn new(sample_rate: f64, submode: Jt65Submode) -> Self {
        let samples_per_symbol =
            (SAMPLES_PER_SYMBOL as f64 * sample_rate / STANDARD_SAMPLE_RATE).round() as usize;
        Self {
            sample_rate,
            submode,
            samples_per_symbol,
        }
    }

    /// Returns the sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Returns the submode.
    pub fn submode(&self) -> Jt65Submode {
        self.submode
    }

    /// Returns the number of samples per symbol.
    pub fn samples_per_symbol(&self) -> usize {
        self.samples_per_symbol
    }

    /// Returns the tone spacing in Hz for the current submode.
    pub fn tone_spacing(&self) -> f64 {
        self.submode.tone_spacing()
    }

    /// Returns the symbol duration in seconds.
    pub fn symbol_duration(&self) -> f64 {
        self.samples_per_symbol as f64 / self.sample_rate
    }

    /// Returns the total transmission duration in seconds.
    pub fn transmission_duration(&self) -> f64 {
        self.symbol_duration() * TOTAL_SYMBOLS as f64
    }

    /// Returns the frequency of a given tone index (0-64).
    ///
    /// Tone 0 is the sync tone. Data tones are 1-64.
    pub fn tone_frequency(&self, tone: u8) -> f64 {
        BASE_TONE_OFFSET + tone as f64 * self.submode.tone_spacing()
    }

    /// Encodes a character to its JT65 character set index.
    ///
    /// Returns `None` for characters not in the JT65 character set.
    pub fn encode_char(c: char) -> Option<u8> {
        let upper = c.to_ascii_uppercase();
        CHARSET.iter().position(|&ch| ch == upper as u8).map(|i| i as u8)
    }

    /// Decodes a JT65 character set index to its character.
    ///
    /// Returns `None` for indices >= 38.
    pub fn decode_char(index: u8) -> Option<char> {
        if (index as usize) < CHARSET_SIZE {
            Some(CHARSET[index as usize] as char)
        } else {
            None
        }
    }

    /// Packs a message string (up to 13 characters) into a 72-bit value.
    ///
    /// Characters are encoded using base-38 packing. The message is padded
    /// with spaces if shorter than 13 characters.
    pub fn pack_message(message: &str) -> u128 {
        let mut chars = [36u8; MAX_MESSAGE_LEN]; // 36 = space in charset
        for (i, c) in message.chars().take(MAX_MESSAGE_LEN).enumerate() {
            chars[i] = Self::encode_char(c).unwrap_or(36); // default to space
        }

        let mut packed: u128 = 0;
        for &ch in &chars {
            packed = packed * CHARSET_SIZE as u128 + ch as u128;
        }
        packed
    }

    /// Unpacks a 72-bit value into a message string.
    pub fn unpack_message(mut packed: u128) -> String {
        let mut chars = vec![0u8; MAX_MESSAGE_LEN];
        for i in (0..MAX_MESSAGE_LEN).rev() {
            chars[i] = (packed % CHARSET_SIZE as u128) as u8;
            packed /= CHARSET_SIZE as u128;
        }

        chars
            .iter()
            .map(|&c| Self::decode_char(c).unwrap_or(' '))
            .collect::<String>()
            .trim_end()
            .to_string()
    }

    /// Encodes a message string into 126 JT65 symbols.
    ///
    /// The message is packed, then spread across 63 data symbol positions
    /// interleaved with 63 sync symbols according to the sync vector.
    /// Data symbols have values 1-64, sync symbols have value 0.
    pub fn encode_message(&self, message: &str) -> Vec<u8> {
        let packed = Self::pack_message(message);

        // Generate 63 data symbols from packed message (values 0-63, then +1 for tone offset)
        let mut data_symbols = Vec::with_capacity(DATA_SYMBOLS);
        let mut remaining = packed;
        for _ in 0..DATA_SYMBOLS {
            data_symbols.push((remaining % 64) as u8);
            remaining /= 64;
        }
        data_symbols.reverse();

        // Interleave data and sync symbols according to sync vector
        let mut symbols = Vec::with_capacity(TOTAL_SYMBOLS);
        let mut data_idx = 0;
        for i in 0..TOTAL_SYMBOLS {
            if SYNC_VECTOR[i] == 1 {
                symbols.push(0u8); // sync tone
            } else {
                // Data tone: value + 1 (tones 1-64 are data, 0 is sync)
                let val = if data_idx < data_symbols.len() {
                    data_symbols[data_idx] + 1
                } else {
                    1
                };
                symbols.push(val);
                data_idx += 1;
            }
        }

        symbols
    }

    /// Decodes 126 symbols back into a message string.
    ///
    /// Extracts data symbols from non-sync positions, reconstructs the packed
    /// message value, and unpacks it into a string.
    pub fn decode_symbols(&self, symbols: &[u8]) -> String {
        if symbols.len() != TOTAL_SYMBOLS {
            return String::new();
        }

        // Extract data symbols (subtract 1 to undo tone offset)
        let mut data_symbols = Vec::with_capacity(DATA_SYMBOLS);
        for i in 0..TOTAL_SYMBOLS {
            if SYNC_VECTOR[i] == 0 {
                let val = if symbols[i] > 0 {
                    symbols[i] - 1
                } else {
                    0
                };
                data_symbols.push(val);
            }
        }

        // Reconstruct packed value
        let mut packed: u128 = 0;
        for &sym in &data_symbols {
            packed = packed * 64 + sym as u128;
        }

        Self::unpack_message(packed)
    }

    /// Modulates a sequence of symbols into IQ samples with phase continuity.
    ///
    /// Each symbol generates `samples_per_symbol` complex samples at the
    /// appropriate tone frequency. Phase is continuous across symbol boundaries.
    pub fn modulate(&self, symbols: &[u8]) -> Vec<(f64, f64)> {
        let total_samples = symbols.len() * self.samples_per_symbol;
        let mut samples = Vec::with_capacity(total_samples);
        let mut phase: f64 = 0.0;

        for &symbol in symbols {
            let freq = self.tone_frequency(symbol);
            let omega = 2.0 * PI * freq / self.sample_rate;

            for _ in 0..self.samples_per_symbol {
                let re = phase.cos();
                let im = phase.sin();
                samples.push((re, im));
                phase += omega;
                // Keep phase bounded to avoid precision loss
                if phase > 2.0 * PI {
                    phase -= 2.0 * PI;
                }
            }
        }

        samples
    }

    /// Generates IQ samples for a single tone at the given frequency.
    ///
    /// Useful for testing and calibration.
    pub fn generate_tone(&self, freq: f64, num_samples: usize) -> Vec<(f64, f64)> {
        let omega = 2.0 * PI * freq / self.sample_rate;
        let mut samples = Vec::with_capacity(num_samples);
        for n in 0..num_samples {
            let phase = omega * n as f64;
            samples.push((phase.cos(), phase.sin()));
        }
        samples
    }

    /// Computes the DFT magnitude at a specific frequency for a block of IQ samples.
    ///
    /// This is a single-bin DFT (Goertzel-like), more efficient than a full FFT
    /// when only a few frequencies need to be evaluated.
    pub fn tone_correlation(
        &self,
        samples: &[(f64, f64)],
        freq: f64,
    ) -> f64 {
        let omega = 2.0 * PI * freq / self.sample_rate;
        let n = samples.len() as f64;
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;

        for (k, &(re, im)) in samples.iter().enumerate() {
            let angle = omega * k as f64;
            let cos_a = angle.cos();
            let sin_a = angle.sin();
            // Multiply sample by conjugate of reference: (re + j*im) * (cos - j*sin)
            sum_re += re * cos_a + im * sin_a;
            sum_im += im * cos_a - re * sin_a;
        }

        (sum_re * sum_re + sum_im * sum_im).sqrt() / n
    }

    /// Demodulates IQ samples into symbols by correlating with each possible tone.
    ///
    /// Returns the detected symbols and the estimated sync offset (in samples).
    /// Uses sync pattern correlation to find the start of the frame, then
    /// applies tone correlation at each symbol position.
    pub fn demodulate(&self, samples: &[(f64, f64)]) -> (Vec<u8>, usize) {
        // Find sync offset
        let sync_offset = self.find_sync_offset(samples);

        // Demodulate each symbol
        let mut symbols = Vec::with_capacity(TOTAL_SYMBOLS);
        for sym_idx in 0..TOTAL_SYMBOLS {
            let start = sync_offset + sym_idx * self.samples_per_symbol;
            let end = start + self.samples_per_symbol;

            if end > samples.len() {
                symbols.push(0);
                continue;
            }

            let symbol_samples = &samples[start..end];

            // Find the tone with the highest correlation
            let mut best_tone = 0u8;
            let mut best_power = 0.0f64;

            for tone in 0..NUM_TONES {
                let freq = self.tone_frequency(tone as u8);
                let power = self.tone_correlation(symbol_samples, freq);

                if power > best_power {
                    best_power = power;
                    best_tone = tone as u8;
                }
            }

            symbols.push(best_tone);
        }

        (symbols, sync_offset)
    }

    /// Finds the sync offset by correlating the sync vector with detected sync tones.
    ///
    /// Searches over a range of possible offsets and returns the one with the
    /// highest correlation to the known sync pattern.
    pub fn find_sync_offset(&self, samples: &[(f64, f64)]) -> usize {
        let expected_len = TOTAL_SYMBOLS * self.samples_per_symbol;
        if samples.len() < expected_len {
            return 0;
        }

        let max_offset = samples.len().saturating_sub(expected_len);
        let search_range = max_offset.min(self.samples_per_symbol);

        let mut best_offset = 0usize;
        let mut best_corr = f64::NEG_INFINITY;

        // Search with coarse steps first, then refine
        let step = if search_range > 100 { search_range / 50 } else { 1 };
        let step = step.max(1);

        for offset in (0..=search_range).step_by(step) {
            let corr = self.sync_correlation_at_offset(samples, offset);
            if corr > best_corr {
                best_corr = corr;
                best_offset = offset;
            }
        }

        // Refine around the best offset
        let refine_start = best_offset.saturating_sub(step);
        let refine_end = (best_offset + step).min(search_range);
        for offset in refine_start..=refine_end {
            let corr = self.sync_correlation_at_offset(samples, offset);
            if corr > best_corr {
                best_corr = corr;
                best_offset = offset;
            }
        }

        best_offset
    }

    /// Computes sync correlation at a given sample offset.
    ///
    /// For each symbol position, measures the power at the sync tone frequency.
    /// Positions where the sync vector is 1 should have high sync tone power,
    /// positions where it is 0 should have low sync tone power.
    fn sync_correlation_at_offset(&self, samples: &[(f64, f64)], offset: usize) -> f64 {
        let sync_freq = self.tone_frequency(0);
        let mut correlation = 0.0;

        // Check a subset of symbol positions for speed
        let check_step = 3; // check every 3rd symbol
        for sym_idx in (0..TOTAL_SYMBOLS).step_by(check_step) {
            let start = offset + sym_idx * self.samples_per_symbol;
            let end = start + self.samples_per_symbol;

            if end > samples.len() {
                break;
            }

            let power = self.tone_correlation(&samples[start..end], sync_freq);
            let expected = if SYNC_VECTOR[sym_idx] == 1 {
                1.0
            } else {
                -1.0
            };
            correlation += power * expected;
        }

        correlation
    }

    /// Estimates the SNR of a demodulated signal in dB.
    ///
    /// Computes the ratio of signal power (strongest tone) to noise power
    /// (average of non-peak tones) for each symbol, then averages.
    pub fn estimate_snr(&self, samples: &[(f64, f64)]) -> f64 {
        let (_, sync_offset) = self.demodulate(samples);
        let mut signal_powers = Vec::new();
        let mut noise_powers = Vec::new();

        for sym_idx in 0..TOTAL_SYMBOLS.min(
            (samples.len().saturating_sub(sync_offset)) / self.samples_per_symbol,
        ) {
            let start = sync_offset + sym_idx * self.samples_per_symbol;
            let end = start + self.samples_per_symbol;
            if end > samples.len() {
                break;
            }

            let symbol_samples = &samples[start..end];

            let powers: Vec<f64> = (0..NUM_TONES)
                .map(|tone| {
                    let freq = self.tone_frequency(tone as u8);
                    let p = self.tone_correlation(symbol_samples, freq);
                    p * p
                })
                .collect();

            // Find the peak
            let max_power = powers.iter().cloned().fold(0.0f64, f64::max);
            signal_powers.push(max_power);

            // Average the rest as noise
            let noise_sum: f64 = powers
                .iter()
                .cloned()
                .filter(|&p| (p - max_power).abs() > 1e-15)
                .sum();
            let noise_avg = noise_sum / (NUM_TONES as f64 - 1.0);
            noise_powers.push(noise_avg);
        }

        if signal_powers.is_empty() || noise_powers.is_empty() {
            return 0.0;
        }

        let avg_signal: f64 = signal_powers.iter().sum::<f64>() / signal_powers.len() as f64;
        let avg_noise: f64 = noise_powers.iter().sum::<f64>() / noise_powers.len() as f64;

        if avg_noise < 1e-20 {
            return 60.0; // effectively infinite SNR
        }

        10.0 * (avg_signal / avg_noise).log10()
    }

    /// Returns the sync vector as a slice.
    pub fn sync_vector() -> &'static [u8; 126] {
        &SYNC_VECTOR
    }

    /// Counts the number of sync symbols correctly detected in a symbol sequence.
    pub fn count_sync_matches(&self, symbols: &[u8]) -> usize {
        if symbols.len() != TOTAL_SYMBOLS {
            return 0;
        }
        let mut matches = 0;
        for i in 0..TOTAL_SYMBOLS {
            if SYNC_VECTOR[i] == 1 && symbols[i] == 0 {
                matches += 1;
            }
        }
        matches
    }

    /// Adds white Gaussian noise to IQ samples at the specified SNR (in dB).
    ///
    /// Uses a simple Box-Muller transform for Gaussian noise generation.
    pub fn add_noise(samples: &[(f64, f64)], snr_db: f64) -> Vec<(f64, f64)> {
        // Compute signal power
        let signal_power: f64 = samples
            .iter()
            .map(|&(re, im)| re * re + im * im)
            .sum::<f64>()
            / samples.len() as f64;

        let noise_power = signal_power / 10.0f64.powf(snr_db / 10.0);
        let noise_std = (noise_power / 2.0).sqrt(); // /2 for each of I and Q

        // Simple deterministic PRNG (linear congruential) for reproducibility
        let mut seed: u64 = 42;
        let lcg_next = |s: &mut u64| -> f64 {
            *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            // Convert to uniform [0, 1)
            (*s >> 11) as f64 / (1u64 << 53) as f64
        };

        let mut noisy = Vec::with_capacity(samples.len());
        for &(re, im) in samples {
            // Box-Muller transform
            let u1 = lcg_next(&mut seed).max(1e-15);
            let u2 = lcg_next(&mut seed);
            let r = (-2.0 * u1.ln()).sqrt();
            let theta = 2.0 * PI * u2;
            let n_re = r * theta.cos() * noise_std;
            let n_im = r * theta.sin() * noise_std;
            noisy.push((re + n_re, im + n_im));
        }

        noisy
    }

    /// Returns information about the JT65 mode as a formatted string.
    pub fn info(&self) -> String {
        format!(
            "JT65{} Mode:\n\
             Sample rate: {:.0} Hz\n\
             Tone spacing: {:.4} Hz\n\
             Bandwidth: {:.1} Hz\n\
             Symbol duration: {:.4} s\n\
             Samples per symbol: {}\n\
             Total symbols: {}\n\
             Transmission duration: {:.2} s\n\
             Character set size: {}\n\
             Max message length: {} chars",
            match self.submode {
                Jt65Submode::A => "A",
                Jt65Submode::B => "B",
                Jt65Submode::C => "C",
            },
            self.sample_rate,
            self.tone_spacing(),
            self.submode.bandwidth(),
            self.symbol_duration(),
            self.samples_per_symbol,
            TOTAL_SYMBOLS,
            self.transmission_duration(),
            CHARSET_SIZE,
            MAX_MESSAGE_LEN,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_submode_tone_spacing() {
        let base = STANDARD_SAMPLE_RATE / SAMPLES_PER_SYMBOL as f64;
        assert!((Jt65Submode::A.tone_spacing() - base).abs() < 1e-6);
        assert!((Jt65Submode::B.tone_spacing() - 2.0 * base).abs() < 1e-6);
        assert!((Jt65Submode::C.tone_spacing() - 4.0 * base).abs() < 1e-6);
    }

    #[test]
    fn test_submode_bandwidth() {
        let spacing_a = Jt65Submode::A.tone_spacing();
        let expected_bw = spacing_a * 64.0;
        assert!((Jt65Submode::A.bandwidth() - expected_bw).abs() < 1e-6);
    }

    #[test]
    fn test_submode_multiplier() {
        assert_eq!(Jt65Submode::A.multiplier(), 1);
        assert_eq!(Jt65Submode::B.multiplier(), 2);
        assert_eq!(Jt65Submode::C.multiplier(), 4);
    }

    #[test]
    fn test_new_standard_rate() {
        let m = Jt65Modulator::new(STANDARD_SAMPLE_RATE, Jt65Submode::A);
        assert_eq!(m.samples_per_symbol(), SAMPLES_PER_SYMBOL);
        assert!((m.sample_rate() - STANDARD_SAMPLE_RATE).abs() < 1e-6);
        assert_eq!(m.submode(), Jt65Submode::A);
    }

    #[test]
    fn test_new_custom_rate() {
        let m = Jt65Modulator::new(22050.0, Jt65Submode::B);
        assert_eq!(m.samples_per_symbol(), 8192); // double the standard
        assert_eq!(m.submode(), Jt65Submode::B);
    }

    #[test]
    fn test_symbol_duration() {
        let m = Jt65Modulator::new(STANDARD_SAMPLE_RATE, Jt65Submode::A);
        let expected = SAMPLES_PER_SYMBOL as f64 / STANDARD_SAMPLE_RATE;
        assert!((m.symbol_duration() - expected).abs() < 1e-6);
        // Approximately 0.372 seconds
        assert!((m.symbol_duration() - 0.3715).abs() < 0.001);
    }

    #[test]
    fn test_transmission_duration() {
        let m = Jt65Modulator::new(STANDARD_SAMPLE_RATE, Jt65Submode::A);
        let expected = 126.0 * SAMPLES_PER_SYMBOL as f64 / STANDARD_SAMPLE_RATE;
        assert!((m.transmission_duration() - expected).abs() < 0.01);
        // Should be about 46.8 seconds
        assert!(m.transmission_duration() > 46.0 && m.transmission_duration() < 48.0);
    }

    #[test]
    fn test_tone_frequency() {
        let m = Jt65Modulator::new(STANDARD_SAMPLE_RATE, Jt65Submode::A);
        let f0 = m.tone_frequency(0);
        assert!((f0 - BASE_TONE_OFFSET).abs() < 1e-6);

        let f1 = m.tone_frequency(1);
        assert!((f1 - f0 - m.tone_spacing()).abs() < 1e-6);

        let f64_tone = m.tone_frequency(64);
        assert!((f64_tone - (BASE_TONE_OFFSET + 64.0 * m.tone_spacing())).abs() < 1e-6);
    }

    #[test]
    fn test_encode_decode_char() {
        // Test digits
        assert_eq!(Jt65Modulator::encode_char('0'), Some(0));
        assert_eq!(Jt65Modulator::encode_char('9'), Some(9));

        // Test letters
        assert_eq!(Jt65Modulator::encode_char('A'), Some(10));
        assert_eq!(Jt65Modulator::encode_char('Z'), Some(35));

        // Test special characters
        assert_eq!(Jt65Modulator::encode_char(' '), Some(36));
        assert_eq!(Jt65Modulator::encode_char('+'), Some(37));

        // Test lowercase maps to uppercase
        assert_eq!(Jt65Modulator::encode_char('a'), Some(10));

        // Test invalid character
        assert_eq!(Jt65Modulator::encode_char('!'), None);

        // Round-trip
        for i in 0..CHARSET_SIZE as u8 {
            let c = Jt65Modulator::decode_char(i).unwrap();
            assert_eq!(Jt65Modulator::encode_char(c), Some(i));
        }

        // Invalid index
        assert_eq!(Jt65Modulator::decode_char(42), None);
    }

    #[test]
    fn test_pack_unpack_message() {
        let msg = "CQ DX K1ABC";
        let packed = Jt65Modulator::pack_message(msg);
        let unpacked = Jt65Modulator::unpack_message(packed);
        assert_eq!(unpacked, msg);
    }

    #[test]
    fn test_pack_unpack_short_message() {
        let msg = "HI";
        let packed = Jt65Modulator::pack_message(msg);
        let unpacked = Jt65Modulator::unpack_message(packed);
        assert_eq!(unpacked, msg);
    }

    #[test]
    fn test_pack_unpack_special_chars() {
        let msg = "73 DE W1AW/2";
        let packed = Jt65Modulator::pack_message(msg);
        let unpacked = Jt65Modulator::unpack_message(packed);
        assert_eq!(unpacked, msg);
    }

    #[test]
    fn test_encode_message_length() {
        let m = Jt65Modulator::new(STANDARD_SAMPLE_RATE, Jt65Submode::A);
        let symbols = m.encode_message("CQ DX K1ABC");
        assert_eq!(symbols.len(), TOTAL_SYMBOLS);
    }

    #[test]
    fn test_encode_message_sync_positions() {
        let m = Jt65Modulator::new(STANDARD_SAMPLE_RATE, Jt65Submode::A);
        let symbols = m.encode_message("CQ DX K1ABC");

        // Sync positions should have tone 0
        for i in 0..TOTAL_SYMBOLS {
            if SYNC_VECTOR[i] == 1 {
                assert_eq!(symbols[i], 0, "Sync symbol at position {} should be 0", i);
            } else {
                // Data symbols should be 1-64
                assert!(
                    symbols[i] >= 1 && symbols[i] <= 64,
                    "Data symbol at position {} should be 1-64, got {}",
                    i,
                    symbols[i]
                );
            }
        }
    }

    #[test]
    fn test_encode_decode_roundtrip() {
        let m = Jt65Modulator::new(STANDARD_SAMPLE_RATE, Jt65Submode::A);
        let msg = "CQ DX K1ABC";
        let symbols = m.encode_message(msg);
        let decoded = m.decode_symbols(&symbols);
        assert_eq!(decoded, msg);
    }

    #[test]
    fn test_modulate_sample_count() {
        let m = Jt65Modulator::new(STANDARD_SAMPLE_RATE, Jt65Submode::A);
        let symbols = vec![0u8; 10];
        let iq = m.modulate(&symbols);
        assert_eq!(iq.len(), 10 * SAMPLES_PER_SYMBOL);
    }

    #[test]
    fn test_modulate_amplitude() {
        let m = Jt65Modulator::new(STANDARD_SAMPLE_RATE, Jt65Submode::A);
        let symbols = vec![32u8; 5];
        let iq = m.modulate(&symbols);

        // All samples should have unit amplitude (cos^2 + sin^2 = 1)
        for &(re, im) in &iq {
            let mag = (re * re + im * im).sqrt();
            assert!(
                (mag - 1.0).abs() < 1e-10,
                "Magnitude should be 1.0, got {}",
                mag
            );
        }
    }

    #[test]
    fn test_tone_correlation_correct_frequency() {
        let m = Jt65Modulator::new(STANDARD_SAMPLE_RATE, Jt65Submode::A);
        let freq = m.tone_frequency(10);
        let tone = m.generate_tone(freq, SAMPLES_PER_SYMBOL);

        // Correlation at the correct frequency should be high
        let corr = m.tone_correlation(&tone, freq);
        assert!(corr > 0.9, "Correlation at correct freq should be high, got {}", corr);

        // Correlation at a wrong frequency should be low
        let wrong_freq = m.tone_frequency(20);
        let wrong_corr = m.tone_correlation(&tone, wrong_freq);
        assert!(
            wrong_corr < 0.1,
            "Correlation at wrong freq should be low, got {}",
            wrong_corr
        );
    }

    #[test]
    fn test_modulate_demodulate_roundtrip() {
        let m = Jt65Modulator::new(STANDARD_SAMPLE_RATE, Jt65Submode::A);
        let symbols = m.encode_message("TEST");
        let iq = m.modulate(&symbols);
        let (demod_symbols, offset) = m.demodulate(&iq);

        assert_eq!(offset, 0);
        assert_eq!(demod_symbols.len(), TOTAL_SYMBOLS);

        // All symbols should match in noiseless conditions
        for i in 0..TOTAL_SYMBOLS {
            assert_eq!(
                demod_symbols[i], symbols[i],
                "Symbol mismatch at position {}: expected {}, got {}",
                i, symbols[i], demod_symbols[i]
            );
        }
    }

    #[test]
    fn test_full_roundtrip_message() {
        let m = Jt65Modulator::new(STANDARD_SAMPLE_RATE, Jt65Submode::A);
        let msg = "CQ K1ABC FN42";
        let symbols = m.encode_message(msg);
        let iq = m.modulate(&symbols);
        let (demod_symbols, _) = m.demodulate(&iq);
        let decoded = m.decode_symbols(&demod_symbols);
        assert_eq!(decoded, msg);
    }

    #[test]
    fn test_sync_vector_counts() {
        let sync_count = SYNC_VECTOR.iter().filter(|&&s| s == 1).count();
        let data_count = SYNC_VECTOR.iter().filter(|&&s| s == 0).count();
        assert_eq!(sync_count, SYNC_SYMBOLS);
        assert_eq!(data_count, DATA_SYMBOLS);
        assert_eq!(sync_count + data_count, TOTAL_SYMBOLS);
    }

    #[test]
    fn test_count_sync_matches() {
        let m = Jt65Modulator::new(STANDARD_SAMPLE_RATE, Jt65Submode::A);
        let symbols = m.encode_message("HELLO");
        let matches = m.count_sync_matches(&symbols);
        assert_eq!(matches, SYNC_SYMBOLS, "All sync symbols should match");
    }

    #[test]
    fn test_add_noise() {
        let m = Jt65Modulator::new(STANDARD_SAMPLE_RATE, Jt65Submode::A);
        let tone = m.generate_tone(1500.0, 4096);
        let noisy = Jt65Modulator::add_noise(&tone, 20.0);
        assert_eq!(noisy.len(), tone.len());

        // Noisy samples should differ from original
        let mut any_different = false;
        for i in 0..tone.len() {
            if (noisy[i].0 - tone[i].0).abs() > 1e-10 {
                any_different = true;
                break;
            }
        }
        assert!(any_different, "Noisy signal should differ from original");
    }

    #[test]
    fn test_snr_estimation_clean() {
        let m = Jt65Modulator::new(STANDARD_SAMPLE_RATE, Jt65Submode::A);
        let full_symbols = m.encode_message("TEST");
        let full_iq = m.modulate(&full_symbols);
        let snr = m.estimate_snr(&full_iq);
        assert!(snr > 20.0, "Clean signal SNR should be high, got {} dB", snr);
    }

    #[test]
    fn test_info_string() {
        let m = Jt65Modulator::new(STANDARD_SAMPLE_RATE, Jt65Submode::A);
        let info = m.info();
        assert!(info.contains("JT65A"));
        assert!(info.contains("11025"));
        assert!(info.contains("126"));
    }

    #[test]
    fn test_generate_tone() {
        let m = Jt65Modulator::new(STANDARD_SAMPLE_RATE, Jt65Submode::A);
        let tone = m.generate_tone(1000.0, 100);
        assert_eq!(tone.len(), 100);

        // First sample should be (1.0, 0.0) since phase starts at 0
        assert!((tone[0].0 - 1.0).abs() < 1e-10);
        assert!(tone[0].1.abs() < 1e-10);

        // All samples should have unit magnitude
        for &(re, im) in &tone {
            let mag = (re * re + im * im).sqrt();
            assert!((mag - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_submode_b_and_c_modulation() {
        for submode in [Jt65Submode::B, Jt65Submode::C] {
            let m = Jt65Modulator::new(STANDARD_SAMPLE_RATE, submode);
            let symbols = m.encode_message("AB");
            let iq = m.modulate(&symbols);
            assert_eq!(iq.len(), TOTAL_SYMBOLS * m.samples_per_symbol());

            // Verify demodulation works for each submode
            let (demod, _) = m.demodulate(&iq);
            assert_eq!(demod.len(), TOTAL_SYMBOLS);

            // Sync symbols should be detected correctly
            for i in 0..TOTAL_SYMBOLS {
                if SYNC_VECTOR[i] == 1 {
                    assert_eq!(demod[i], 0, "Sync mismatch at {} for {:?}", i, submode);
                }
            }
        }
    }
}
