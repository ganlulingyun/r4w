//! WSPR (Weak Signal Propagation Reporter) 4-FSK modulator and demodulator.
//!
//! Implements the WSPR digital mode used by amateur radio operators for
//! low-power propagation testing. WSPR transmissions encode callsign, Maidenhead
//! grid locator, and transmit power into 162 symbols using 4-FSK modulation
//! with 1.4648 Hz tone spacing (12000/8192 Hz).
//!
//! # Protocol Summary
//!
//! - **Message**: callsign (28 bits) + grid (15 bits) + power (7 bits) = 50 bits
//! - **FEC**: Rate-1/2 convolutional code, K=32 → 162 bits
//! - **Interleaving**: Bit-reversal permutation
//! - **Modulation**: 4-FSK with sync vector, tone spacing 1.4648 Hz
//! - **Symbol rate**: 1.4648 baud (symbol period = 8192/12000 s ≈ 0.683 s)
//! - **Transmission duration**: ~110.6 seconds (162 symbols)
//!
//! # Example
//!
//! ```rust
//! use r4w_core::wspr_modulator::{WsprModulator, WsprMessage};
//!
//! let msg = WsprMessage::new("K1ABC", "FN42", 37).unwrap();
//! let modulator = WsprModulator::new(12000.0);
//! let symbols = modulator.encode(&msg);
//! assert_eq!(symbols.len(), 162);
//!
//! // Generate IQ samples
//! let iq = modulator.modulate(&symbols, 1500.0);
//! assert!(iq.len() > 0);
//! ```

use std::f64::consts::PI;

// ─── Constants ───────────────────────────────────────────────────────────────

/// Number of symbols in a WSPR transmission.
pub const WSPR_SYMBOL_COUNT: usize = 162;

/// Tone spacing in Hz: 12000 / 8192 ≈ 1.46484375 Hz.
pub const TONE_SPACING: f64 = 12000.0 / 8192.0;

/// Symbol period in seconds: 8192 / 12000 ≈ 0.68266667 s.
pub const SYMBOL_PERIOD: f64 = 8192.0 / 12000.0;

/// Total transmission duration in seconds: 162 × symbol_period ≈ 110.6 s.
pub const TX_DURATION: f64 = WSPR_SYMBOL_COUNT as f64 * SYMBOL_PERIOD;

/// Convolutional encoder polynomial G1 (octal 0xF2D05351).
const POLY_G1: u32 = 0xF2D0_5351;

/// Convolutional encoder polynomial G2 (octal 0xE4613C47).
const POLY_G2: u32 = 0xE461_3C47;

/// 162-element sync vector (pseudo-random sequence for frame synchronization).
/// Each element is 0 or 1. The transmitted tone index = sync[i] + 2 * data[i].
const SYNC_VECTOR: [u8; WSPR_SYMBOL_COUNT] = [
    1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0,
    0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1,
    0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1,
    1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1,
    0, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1,
    0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1,
    0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0,
    0, 0,
];

// ─── Valid power levels ──────────────────────────────────────────────────────

/// Valid WSPR power levels in dBm. Only these values can be encoded.
const VALID_POWERS: [u8; 19] = [
    0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60,
];

// ─── Error type ──────────────────────────────────────────────────────────────

/// Errors from WSPR encoding/decoding.
#[derive(Debug, Clone, PartialEq)]
pub enum WsprError {
    /// Callsign is invalid (must be 1-6 alphanumeric, 3rd char must be digit).
    InvalidCallsign(String),
    /// Grid locator is invalid (must be 4 chars like "FN42").
    InvalidGrid(String),
    /// Power level is not a valid WSPR power value.
    InvalidPower(u8),
    /// Wrong number of symbols for demodulation.
    WrongSymbolCount(usize),
}

impl std::fmt::Display for WsprError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            WsprError::InvalidCallsign(c) => write!(f, "Invalid WSPR callsign: '{}'", c),
            WsprError::InvalidGrid(g) => write!(f, "Invalid WSPR grid locator: '{}'", g),
            WsprError::InvalidPower(p) => write!(f, "Invalid WSPR power level: {} dBm", p),
            WsprError::WrongSymbolCount(n) => {
                write!(f, "Expected {} symbols, got {}", WSPR_SYMBOL_COUNT, n)
            }
        }
    }
}

impl std::error::Error for WsprError {}

// ─── WsprMessage ─────────────────────────────────────────────────────────────

/// A WSPR message containing callsign, grid locator, and power level.
#[derive(Debug, Clone, PartialEq)]
pub struct WsprMessage {
    /// Callsign (e.g., "K1ABC"). Stored uppercase, zero-padded to 6 chars.
    pub callsign: String,
    /// 4-character Maidenhead grid locator (e.g., "FN42").
    pub grid: String,
    /// Transmit power in dBm (must be a valid WSPR power level).
    pub power: u8,
}

impl WsprMessage {
    /// Create a new WSPR message.
    ///
    /// The callsign is normalized to uppercase and left-padded with spaces to
    /// 6 characters. The third character (index 2) must be a digit.
    ///
    /// # Errors
    ///
    /// Returns `WsprError` if the callsign, grid, or power is invalid.
    pub fn new(callsign: &str, grid: &str, power: u8) -> Result<Self, WsprError> {
        let call = normalize_callsign(callsign)?;
        validate_grid(grid)?;
        validate_power(power)?;
        Ok(WsprMessage {
            callsign: call,
            grid: grid.to_uppercase(),
            power,
        })
    }

    /// Pack the callsign into a 28-bit integer.
    pub fn pack_callsign(&self) -> u32 {
        pack_callsign(&self.callsign)
    }

    /// Pack the grid locator into a 15-bit integer.
    pub fn pack_grid(&self) -> u16 {
        pack_grid(&self.grid)
    }

    /// Pack the power level into a 7-bit integer.
    /// The power is encoded along with the grid as: grid * 128 + power + 64.
    pub fn pack_power(&self) -> u8 {
        self.power + 64
    }

    /// Pack the entire message into 50 source bits.
    pub fn pack(&self) -> Vec<bool> {
        let c = self.pack_callsign() as u64;
        let g = self.pack_grid() as u64;
        let p = (self.power as u64) + 64;

        // Pack as bitfields: 28-bit callsign | 15-bit grid | 7-bit power
        let n: u64 = (c << 22) | (g << 7) | p;

        let mut bits = Vec::with_capacity(50);
        for i in (0..50).rev() {
            bits.push((n >> i) & 1 == 1);
        }
        bits
    }

    /// Unpack a WSPR message from 50 source bits.
    pub fn unpack(bits: &[bool]) -> Result<Self, WsprError> {
        if bits.len() < 50 {
            return Err(WsprError::WrongSymbolCount(bits.len()));
        }

        let mut n: u64 = 0;
        for &b in &bits[..50] {
            n = (n << 1) | (b as u64);
        }

        let p_enc = (n & 0x7F) as u8;
        let g = ((n >> 7) & 0x7FFF) as u16;
        let c = ((n >> 22) & 0x0FFF_FFFF) as u32;

        let power = if p_enc >= 64 { p_enc - 64 } else { 0 };
        let callsign = unpack_callsign(c);
        let grid = unpack_grid(g);

        // Validate the round-tripped power
        if !VALID_POWERS.contains(&power) {
            return Err(WsprError::InvalidPower(power));
        }

        Ok(WsprMessage {
            callsign: callsign.trim().to_string(),
            grid,
            power,
        })
    }
}

// ─── Callsign packing ───────────────────────────────────────────────────────

/// Normalize a callsign: uppercase, pad to 6 chars, ensure third
/// character (index 2) is a digit per the WSPR encoding spec.
///
/// Padding rules:
/// - If the callsign already has 6 chars and index 2 is a digit, use as-is.
/// - If the callsign has a digit at position 2 (1-indexed), left-pad with
///   one space and right-pad with spaces to 6 chars (e.g., "W1AW" -> " W1AW ").
/// - Otherwise, right-pad with spaces to 6 chars (e.g., "K1ABC" -> " K1ABC" with
///   leading space fill to put digit at index 2).
fn normalize_callsign(call: &str) -> Result<String, WsprError> {
    let call = call.trim().to_uppercase();
    if call.is_empty() || call.len() > 6 {
        return Err(WsprError::InvalidCallsign(call));
    }

    // All chars must be space, letter, or digit
    for ch in call.chars() {
        if !ch.is_ascii_alphanumeric() && ch != ' ' {
            return Err(WsprError::InvalidCallsign(call));
        }
    }

    let chars: Vec<char> = call.chars().collect();

    // Try to find the right padding so index 2 is a digit.
    // Strategy: try left-padding with 0, 1, or 2 spaces.
    for left_pad in 0..=2 {
        let right_pad_needed = 6usize.saturating_sub(call.len() + left_pad);
        if call.len() + left_pad + right_pad_needed != 6 {
            continue;
        }
        let padded = format!(
            "{}{}{}",
            " ".repeat(left_pad),
            call,
            " ".repeat(right_pad_needed)
        );
        let pchars: Vec<char> = padded.chars().collect();
        if pchars[2].is_ascii_digit() {
            return Ok(padded);
        }
    }

    Err(WsprError::InvalidCallsign(call))
}

/// Encode a single character for callsign packing.
/// Space=36, A-Z=10-35, 0-9=0-9.
fn char_code(c: char) -> u32 {
    match c {
        ' ' => 36,
        '0'..='9' => (c as u32) - ('0' as u32),
        'A'..='Z' => (c as u32) - ('A' as u32) + 10,
        _ => 36, // treat invalid as space
    }
}

/// Decode a character code back to a character.
fn code_char(code: u32) -> char {
    match code {
        0..=9 => (b'0' + code as u8) as char,
        10..=35 => (b'A' + (code - 10) as u8) as char,
        _ => ' ',
    }
}

/// Pack a 6-character callsign into a 28-bit integer.
///
/// Where c1,c2 use full 37-char alphabet, c3 is digit (0-9),
/// c4,c5,c6 use letter-or-space (27 values: A-Z + space).
fn pack_callsign(call: &str) -> u32 {
    let chars: Vec<char> = call.chars().collect();
    assert!(chars.len() == 6);

    let c1 = char_code(chars[0]);
    let c2 = char_code(chars[1]);
    let c3 = chars[2].to_digit(10).unwrap_or(0);

    // c4, c5, c6: letter or space -> 0-26 (space=0, A=1..Z=26)
    let letter_code = |ch: char| -> u32 {
        match ch {
            'A'..='Z' => (ch as u32) - ('A' as u32) + 1,
            _ => 0, // space
        }
    };

    let c4 = letter_code(chars[3]);
    let c5 = letter_code(chars[4]);
    let c6 = letter_code(chars[5]);

    ((((c1 * 36 + c2) * 10 + c3) * 27 + c4) * 27 + c5) * 27 + c6
}

/// Unpack a 28-bit integer back to a 6-character callsign.
fn unpack_callsign(mut n: u32) -> String {
    let c6 = n % 27;
    n /= 27;
    let c5 = n % 27;
    n /= 27;
    let c4 = n % 27;
    n /= 27;
    let c3 = n % 10;
    n /= 10;
    let c2 = n % 36;
    n /= 36;
    let c1 = n;

    let letter_char = |code: u32| -> char {
        if code == 0 {
            ' '
        } else {
            (b'A' + (code - 1) as u8) as char
        }
    };

    let mut s = String::with_capacity(6);
    s.push(code_char(c1));
    s.push(code_char(c2));
    s.push(std::char::from_digit(c3, 10).unwrap_or('0'));
    s.push(letter_char(c4));
    s.push(letter_char(c5));
    s.push(letter_char(c6));
    s
}

// ─── Grid locator encoding ──────────────────────────────────────────────────

/// Validate a 4-character Maidenhead grid locator (e.g., "FN42").
fn validate_grid(grid: &str) -> Result<(), WsprError> {
    let g = grid.to_uppercase();
    let chars: Vec<char> = g.chars().collect();
    if chars.len() != 4 {
        return Err(WsprError::InvalidGrid(grid.to_string()));
    }
    if !('A'..='R').contains(&chars[0]) || !('A'..='R').contains(&chars[1]) {
        return Err(WsprError::InvalidGrid(grid.to_string()));
    }
    if !chars[2].is_ascii_digit() || !chars[3].is_ascii_digit() {
        return Err(WsprError::InvalidGrid(grid.to_string()));
    }
    Ok(())
}

/// Pack a 4-character grid locator into a 15-bit integer.
fn pack_grid(grid: &str) -> u16 {
    let g = grid.to_uppercase();
    let chars: Vec<char> = g.chars().collect();
    let lon_field = (chars[0] as u16) - ('A' as u16);
    let lat_field = (chars[1] as u16) - ('A' as u16);
    let lon_sq = chars[2].to_digit(10).unwrap_or(0) as u16;
    let lat_sq = chars[3].to_digit(10).unwrap_or(0) as u16;
    (lon_field * 18 + lat_field) * 100 + lon_sq * 10 + lat_sq
}

/// Unpack a 15-bit integer back to a 4-character grid locator.
fn unpack_grid(mut n: u16) -> String {
    let lat_sq = n % 10;
    n /= 10;
    let lon_sq = n % 10;
    n /= 10;
    let lat_field = n % 18;
    n /= 18;
    let lon_field = n;

    let mut s = String::with_capacity(4);
    s.push((b'A' + lon_field as u8) as char);
    s.push((b'A' + lat_field as u8) as char);
    s.push(std::char::from_digit(lon_sq as u32, 10).unwrap_or('0'));
    s.push(std::char::from_digit(lat_sq as u32, 10).unwrap_or('0'));
    s
}

/// Validate a WSPR power level.
fn validate_power(power: u8) -> Result<(), WsprError> {
    if VALID_POWERS.contains(&power) {
        Ok(())
    } else {
        Err(WsprError::InvalidPower(power))
    }
}

// ─── Convolutional encoder ──────────────────────────────────────────────────

/// Apply rate-1/2 convolutional encoding with K=32.
///
/// For each input bit, two output bits are produced by computing parity of
/// the shift register ANDed with each generator polynomial.
fn convolutional_encode(source_bits: &[bool]) -> Vec<bool> {
    let mut reg: u32 = 0;
    let mut output = Vec::with_capacity(source_bits.len() * 2);

    for &bit in source_bits {
        reg = (reg << 1) | (bit as u32);

        // Parity = popcount(reg & poly) mod 2
        let p1 = (reg & POLY_G1).count_ones() % 2;
        let p2 = (reg & POLY_G2).count_ones() % 2;

        output.push(p1 == 1);
        output.push(p2 == 1);
    }

    output
}

// ─── Interleaver ─────────────────────────────────────────────────────────────

/// Bit-reversal interleaving on 256 positions, keeping only first 162.
///
/// The WSPR interleaver works by computing the bit-reversal of each index
/// in an 8-bit field (0..255), then assigning input bits to output positions
/// in the order determined by sorted bit-reversed indices.
fn interleave(bits: &[bool]) -> Vec<bool> {
    assert!(bits.len() >= WSPR_SYMBOL_COUNT);

    // Generate (bit_reversed_index, original_index) pairs for 0..255
    let mut perm: Vec<(u8, u8)> = (0u8..=255)
        .map(|i| (bit_reverse_8(i), i))
        .collect();

    // Sort by bit-reversed value to get the interleaving permutation
    perm.sort();

    // Take first 162 entries and apply the permutation
    let mut output = vec![false; WSPR_SYMBOL_COUNT];
    let mut dst = 0;
    for &(_, src_idx) in &perm {
        if (src_idx as usize) < WSPR_SYMBOL_COUNT {
            output[dst] = bits[src_idx as usize];
            dst += 1;
            if dst >= WSPR_SYMBOL_COUNT {
                break;
            }
        }
    }

    output
}

/// De-interleave 162 symbols back to the original bit order.
fn deinterleave(bits: &[bool]) -> Vec<bool> {
    assert!(bits.len() >= WSPR_SYMBOL_COUNT);

    let mut perm: Vec<(u8, u8)> = (0u8..=255)
        .map(|i| (bit_reverse_8(i), i))
        .collect();
    perm.sort();

    // Build forward mapping: dst_index -> src_index
    let mut forward_map = Vec::new();
    for &(_, src_idx) in &perm {
        if (src_idx as usize) < WSPR_SYMBOL_COUNT {
            forward_map.push(src_idx as usize);
            if forward_map.len() >= WSPR_SYMBOL_COUNT {
                break;
            }
        }
    }

    // Invert: output[forward_map[i]] = bits[i]
    let mut output = vec![false; WSPR_SYMBOL_COUNT];
    for (dst, &src) in forward_map.iter().enumerate() {
        output[src] = bits[dst];
    }

    output
}

/// Reverse the bits of an 8-bit value.
fn bit_reverse_8(mut x: u8) -> u8 {
    let mut result: u8 = 0;
    for _ in 0..8 {
        result = (result << 1) | (x & 1);
        x >>= 1;
    }
    result
}

// ─── WsprModulator ───────────────────────────────────────────────────────────

/// WSPR 4-FSK modulator and demodulator.
///
/// Generates phase-continuous 4-FSK IQ samples for WSPR transmissions
/// and can demodulate them back to symbols via tone detection.
#[derive(Debug, Clone)]
pub struct WsprModulator {
    /// Sample rate in Hz.
    pub sample_rate: f64,
}

impl WsprModulator {
    /// Create a new WSPR modulator with the given sample rate.
    ///
    /// Common sample rates: 12000.0 Hz (standard audio), 48000.0 Hz.
    pub fn new(sample_rate: f64) -> Self {
        WsprModulator { sample_rate }
    }

    /// Number of samples per symbol at the configured sample rate.
    pub fn samples_per_symbol(&self) -> usize {
        (self.sample_rate * SYMBOL_PERIOD).round() as usize
    }

    /// Encode a WSPR message into 162 channel symbols (each 0-3).
    ///
    /// Steps:
    /// 1. Pack message into 50 source bits
    /// 2. Convolutional encode (rate 1/2, K=32) -> 162 coded bits
    ///    (padded to 81 source bits with 31 tail zeros)
    /// 3. Interleave the 162 coded bits
    /// 4. Merge with sync vector: symbol = sync[i] + 2 * data[i]
    pub fn encode(&self, msg: &WsprMessage) -> Vec<u8> {
        let source_bits = msg.pack();

        // Pad source bits to 81 with trailing zeros (K-1 = 31 tail bits)
        let mut padded = source_bits;
        padded.resize(81, false);

        // Convolutional encode -> 162 coded bits
        let coded = convolutional_encode(&padded);
        assert_eq!(coded.len(), 162);

        // Interleave
        let interleaved = interleave(&coded);

        // Merge with sync vector to produce 4-FSK symbols (0-3)
        let mut symbols = Vec::with_capacity(WSPR_SYMBOL_COUNT);
        for i in 0..WSPR_SYMBOL_COUNT {
            let data_bit = interleaved[i] as u8;
            let sync_bit = SYNC_VECTOR[i];
            symbols.push(sync_bit + 2 * data_bit);
        }

        symbols
    }

    /// Generate IQ samples for a sequence of WSPR symbols.
    ///
    /// `symbols`: slice of symbol values, each 0-3.
    /// `audio_freq`: Center frequency in Hz (e.g., 1500.0 for audio baseband).
    ///
    /// Returns a vector of `(re, im)` tuples representing complex IQ samples.
    /// The FSK modulation is phase-continuous across symbol boundaries.
    pub fn modulate(&self, symbols: &[u8], audio_freq: f64) -> Vec<(f64, f64)> {
        let sps = self.samples_per_symbol();
        let total_samples = sps * symbols.len();
        let mut iq = Vec::with_capacity(total_samples);

        let mut phase: f64 = 0.0;

        for &sym in symbols {
            // Tone frequency: center + (sym - 1.5) * tone_spacing
            let tone_offset = (sym as f64 - 1.5) * TONE_SPACING;
            let freq = audio_freq + tone_offset;
            let phase_inc = 2.0 * PI * freq / self.sample_rate;

            for _ in 0..sps {
                let re = phase.cos();
                let im = phase.sin();
                iq.push((re, im));
                phase += phase_inc;
                // Keep phase in [0, 2pi) to avoid precision loss
                if phase >= 2.0 * PI {
                    phase -= 2.0 * PI;
                }
            }
        }

        iq
    }

    /// Demodulate IQ samples back to WSPR symbols via tone detection.
    ///
    /// Uses a matched filter approach: for each symbol period, compute the
    /// magnitude at each of the 4 tone frequencies and pick the strongest.
    ///
    /// `samples`: IQ sample slice.
    /// `audio_freq`: Center frequency used during modulation.
    ///
    /// Returns a vector of detected symbols (each 0-3).
    pub fn demodulate(&self, samples: &[(f64, f64)], audio_freq: f64) -> Vec<u8> {
        let sps = self.samples_per_symbol();
        let num_symbols = samples.len() / sps;
        let mut symbols = Vec::with_capacity(num_symbols);

        for sym_idx in 0..num_symbols {
            let start = sym_idx * sps;
            let end = start + sps;
            let segment = &samples[start..end];

            let mut best_tone = 0u8;
            let mut best_mag = -1.0f64;

            for tone in 0u8..4 {
                let tone_offset = (tone as f64 - 1.5) * TONE_SPACING;
                let freq = audio_freq + tone_offset;
                let phase_inc = 2.0 * PI * freq / self.sample_rate;

                // Correlate segment with reference tone
                let mut sum_re = 0.0;
                let mut sum_im = 0.0;

                for (k, &(re, im)) in segment.iter().enumerate() {
                    let ref_phase = phase_inc * k as f64;
                    let ref_re = ref_phase.cos();
                    let ref_im = ref_phase.sin();

                    // Complex multiply: sample * conj(reference)
                    sum_re += re * ref_re + im * ref_im;
                    sum_im += im * ref_re - re * ref_im;
                }

                let mag = sum_re * sum_re + sum_im * sum_im;
                if mag > best_mag {
                    best_mag = mag;
                    best_tone = tone;
                }
            }

            symbols.push(best_tone);
        }

        symbols
    }

    /// Decode 162 channel symbols back to a WSPR message.
    ///
    /// Steps:
    /// 1. Extract data bits from symbols using sync vector
    /// 2. De-interleave
    /// 3. Hard-decision sequential decode
    ///
    /// Note: This is a simplified decoder suitable for clean (noiseless or
    /// low-noise) signals. A production decoder would use soft-decision Viterbi.
    pub fn decode_symbols(&self, symbols: &[u8]) -> Result<WsprMessage, WsprError> {
        if symbols.len() != WSPR_SYMBOL_COUNT {
            return Err(WsprError::WrongSymbolCount(symbols.len()));
        }

        // Extract data bits: data = (symbol - sync) / 2
        let mut data_bits = Vec::with_capacity(WSPR_SYMBOL_COUNT);
        for i in 0..WSPR_SYMBOL_COUNT {
            let data = (symbols[i].saturating_sub(SYNC_VECTOR[i])) / 2;
            data_bits.push(data != 0);
        }

        // De-interleave
        let deinterleaved = deinterleave(&data_bits);

        // Sequential decode of the convolutional code
        let source_bits = viterbi_decode_hard(&deinterleaved);

        // Unpack the first 50 bits
        WsprMessage::unpack(&source_bits)
    }

    /// Get the 4 tone frequencies for a given center frequency.
    pub fn tone_frequencies(&self, audio_freq: f64) -> [f64; 4] {
        let mut freqs = [0.0; 4];
        for tone in 0..4 {
            freqs[tone] = audio_freq + (tone as f64 - 1.5) * TONE_SPACING;
        }
        freqs
    }

    /// Compute the bandwidth of the WSPR signal (4 tones spanning ~6 Hz).
    pub fn bandwidth(&self) -> f64 {
        3.0 * TONE_SPACING // from tone 0 to tone 3
    }
}

/// Hard-decision sequential decoder for the K=32 convolutional code.
///
/// For each source bit position, try both 0 and 1, compute the expected
/// parity outputs, and pick whichever matches the received coded bits better.
fn viterbi_decode_hard(coded_bits: &[bool]) -> Vec<bool> {
    let num_source = coded_bits.len() / 2;
    let mut reg: u32 = 0;
    let mut source = Vec::with_capacity(num_source);

    for i in 0..num_source {
        let expected_p1_0 = ((reg << 1) & POLY_G1).count_ones() % 2 == 1;
        let expected_p2_0 = ((reg << 1) & POLY_G2).count_ones() % 2 == 1;
        let expected_p1_1 = (((reg << 1) | 1) & POLY_G1).count_ones() % 2 == 1;
        let expected_p2_1 = (((reg << 1) | 1) & POLY_G2).count_ones() % 2 == 1;

        let received_p1 = coded_bits[2 * i];
        let received_p2 = coded_bits[2 * i + 1];

        let err_0 = (expected_p1_0 != received_p1) as u32 + (expected_p2_0 != received_p2) as u32;
        let err_1 = (expected_p1_1 != received_p1) as u32 + (expected_p2_1 != received_p2) as u32;

        let bit = err_1 < err_0;
        source.push(bit);
        reg = (reg << 1) | (bit as u32);
    }

    source
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_constants() {
        // Tone spacing should be 12000/8192
        assert!((TONE_SPACING - 1.46484375).abs() < 1e-10);
        // Symbol period
        assert!((SYMBOL_PERIOD - 8192.0 / 12000.0).abs() < 1e-10);
        // Transmission duration approximately 110.6 seconds
        assert!((TX_DURATION - 110.592).abs() < 0.01);
    }

    #[test]
    fn test_sync_vector_length() {
        assert_eq!(SYNC_VECTOR.len(), WSPR_SYMBOL_COUNT);
        // All values should be 0 or 1
        for &v in &SYNC_VECTOR {
            assert!(v == 0 || v == 1);
        }
    }

    #[test]
    fn test_valid_message_creation() {
        let msg = WsprMessage::new("K1ABC", "FN42", 37);
        assert!(msg.is_ok());
        let msg = msg.unwrap();
        assert_eq!(msg.callsign, " K1ABC");
        assert_eq!(msg.grid, "FN42");
        assert_eq!(msg.power, 37);
    }

    #[test]
    fn test_invalid_callsign() {
        // Too long
        assert!(WsprMessage::new("K1ABCDEF", "FN42", 37).is_err());
        // Third char not a digit (after padding)
        assert!(WsprMessage::new("AABBCC", "FN42", 37).is_err());
        // Empty
        assert!(WsprMessage::new("", "FN42", 37).is_err());
    }

    #[test]
    fn test_invalid_grid() {
        // Wrong length
        assert!(WsprMessage::new("K1ABC", "FN4", 37).is_err());
        // Invalid field characters
        assert!(WsprMessage::new("K1ABC", "ZZ42", 37).is_err());
        // Non-digit subsquare
        assert!(WsprMessage::new("K1ABC", "FNAB", 37).is_err());
    }

    #[test]
    fn test_invalid_power() {
        // 5 dBm is not a valid WSPR power level
        assert!(WsprMessage::new("K1ABC", "FN42", 5).is_err());
        assert!(WsprMessage::new("K1ABC", "FN42", 1).is_err());
    }

    #[test]
    fn test_callsign_roundtrip() {
        let call = " K1ABC";
        let packed = pack_callsign(call);
        let unpacked = unpack_callsign(packed);
        assert_eq!(unpacked, call);
    }

    #[test]
    fn test_callsign_various() {
        // Test different callsign formats
        for call in &["W1AW", "K1ABC", "VE3XYZ", "AA1AA"] {
            let msg = WsprMessage::new(call, "FN42", 37).unwrap();
            let packed = msg.pack_callsign();
            let unpacked = unpack_callsign(packed);
            assert_eq!(unpacked.trim(), msg.callsign.trim());
        }
    }

    #[test]
    fn test_grid_roundtrip() {
        let grid = "FN42";
        let packed = pack_grid(grid);
        let unpacked = unpack_grid(packed);
        assert_eq!(unpacked, grid);
    }

    #[test]
    fn test_grid_various() {
        for grid in &["AA00", "FN42", "JO62", "RR99", "IO91"] {
            let packed = pack_grid(grid);
            let unpacked = unpack_grid(packed);
            assert_eq!(&unpacked, grid);
        }
    }

    #[test]
    fn test_message_pack_unpack_roundtrip() {
        let msg = WsprMessage::new("K1ABC", "FN42", 37).unwrap();
        let bits = msg.pack();
        assert_eq!(bits.len(), 50);

        let decoded = WsprMessage::unpack(&bits).unwrap();
        assert_eq!(decoded.callsign, msg.callsign.trim());
        assert_eq!(decoded.grid, msg.grid);
        assert_eq!(decoded.power, msg.power);
    }

    #[test]
    fn test_bit_reverse() {
        assert_eq!(bit_reverse_8(0b00000000), 0b00000000);
        assert_eq!(bit_reverse_8(0b10000000), 0b00000001);
        assert_eq!(bit_reverse_8(0b11110000), 0b00001111);
        assert_eq!(bit_reverse_8(0b10101010), 0b01010101);
        assert_eq!(bit_reverse_8(0xFF), 0xFF);
    }

    #[test]
    fn test_convolutional_encoder_output_length() {
        let input = vec![false; 81];
        let output = convolutional_encode(&input);
        assert_eq!(output.len(), 162);
    }

    #[test]
    fn test_convolutional_encoder_deterministic() {
        let input: Vec<bool> = (0..81).map(|i| i % 3 == 0).collect();
        let out1 = convolutional_encode(&input);
        let out2 = convolutional_encode(&input);
        assert_eq!(out1, out2);
    }

    #[test]
    fn test_interleave_deinterleave_roundtrip() {
        let original: Vec<bool> = (0..162).map(|i| i % 5 < 2).collect();
        let interleaved = interleave(&original);
        let deinterleaved = deinterleave(&interleaved);
        assert_eq!(original, deinterleaved);
    }

    #[test]
    fn test_encode_produces_162_symbols() {
        let msg = WsprMessage::new("K1ABC", "FN42", 37).unwrap();
        let modulator = WsprModulator::new(12000.0);
        let symbols = modulator.encode(&msg);
        assert_eq!(symbols.len(), WSPR_SYMBOL_COUNT);
        // All symbols should be 0-3
        for &s in &symbols {
            assert!(s <= 3, "Symbol {} out of range", s);
        }
    }

    #[test]
    fn test_modulate_sample_count() {
        let msg = WsprMessage::new("K1ABC", "FN42", 37).unwrap();
        let modulator = WsprModulator::new(12000.0);
        let symbols = modulator.encode(&msg);
        let iq = modulator.modulate(&symbols, 1500.0);

        let expected_sps = (12000.0 * SYMBOL_PERIOD).round() as usize;
        assert_eq!(iq.len(), expected_sps * WSPR_SYMBOL_COUNT);
    }

    #[test]
    fn test_modulate_amplitude() {
        let modulator = WsprModulator::new(12000.0);
        let symbols = vec![0u8; 10]; // Use fewer symbols for speed
        let iq = modulator.modulate(&symbols, 1500.0);

        // Each sample should have unit magnitude (pure tone)
        for &(re, im) in &iq {
            let mag = (re * re + im * im).sqrt();
            assert!(
                (mag - 1.0).abs() < 1e-10,
                "Magnitude {} != 1.0",
                mag
            );
        }
    }

    #[test]
    fn test_demodulate_clean_signal() {
        let modulator = WsprModulator::new(12000.0);
        let symbols: Vec<u8> = (0..20).map(|i| (i % 4) as u8).collect();
        let iq = modulator.modulate(&symbols, 1500.0);
        let detected = modulator.demodulate(&iq, 1500.0);

        assert_eq!(detected.len(), symbols.len());
        assert_eq!(detected, symbols);
    }

    #[test]
    fn test_full_encode_decode_roundtrip() {
        let msg = WsprMessage::new("K1ABC", "FN42", 37).unwrap();
        let modulator = WsprModulator::new(12000.0);

        let symbols = modulator.encode(&msg);
        let decoded = modulator.decode_symbols(&symbols).unwrap();

        assert_eq!(decoded.callsign, msg.callsign.trim());
        assert_eq!(decoded.grid, msg.grid);
        assert_eq!(decoded.power, msg.power);
    }

    #[test]
    fn test_full_modulate_demodulate_decode() {
        let msg = WsprMessage::new("W1AW", "FN31", 30).unwrap();
        let modulator = WsprModulator::new(12000.0);

        let symbols = modulator.encode(&msg);
        let iq = modulator.modulate(&symbols, 1500.0);
        let detected = modulator.demodulate(&iq, 1500.0);

        // Clean signal should demodulate perfectly
        assert_eq!(detected, symbols);

        let decoded = modulator.decode_symbols(&detected).unwrap();
        assert_eq!(decoded.callsign, msg.callsign.trim());
        assert_eq!(decoded.grid, msg.grid);
        assert_eq!(decoded.power, msg.power);
    }

    #[test]
    fn test_tone_frequencies() {
        let modulator = WsprModulator::new(12000.0);
        let freqs = modulator.tone_frequencies(1500.0);

        // Tones should be spaced by TONE_SPACING
        for i in 1..4 {
            let diff = freqs[i] - freqs[i - 1];
            assert!((diff - TONE_SPACING).abs() < 1e-10);
        }

        // Center should be at audio_freq
        let center = (freqs[0] + freqs[3]) / 2.0;
        assert!((center - 1500.0).abs() < 1e-10);
    }

    #[test]
    fn test_bandwidth() {
        let modulator = WsprModulator::new(12000.0);
        let bw = modulator.bandwidth();
        // Bandwidth = 3 * tone spacing ~ 4.395 Hz
        assert!((bw - 3.0 * TONE_SPACING).abs() < 1e-10);
    }

    #[test]
    fn test_samples_per_symbol() {
        let modulator = WsprModulator::new(12000.0);
        // 12000 * 8192/12000 = 8192
        assert_eq!(modulator.samples_per_symbol(), 8192);

        let modulator48k = WsprModulator::new(48000.0);
        // 48000 * 8192/12000 = 32768
        assert_eq!(modulator48k.samples_per_symbol(), 32768);
    }

    #[test]
    fn test_decode_wrong_symbol_count() {
        let modulator = WsprModulator::new(12000.0);
        let short_symbols = vec![0u8; 100];
        let result = modulator.decode_symbols(&short_symbols);
        assert!(result.is_err());
        match result {
            Err(WsprError::WrongSymbolCount(100)) => {}
            _ => panic!("Expected WrongSymbolCount(100)"),
        }
    }

    #[test]
    fn test_error_display() {
        let e = WsprError::InvalidCallsign("BAD!".to_string());
        assert!(format!("{}", e).contains("BAD!"));

        let e = WsprError::InvalidGrid("XX".to_string());
        assert!(format!("{}", e).contains("XX"));

        let e = WsprError::InvalidPower(5);
        assert!(format!("{}", e).contains("5"));

        let e = WsprError::WrongSymbolCount(100);
        assert!(format!("{}", e).contains("100"));
    }

    #[test]
    fn test_char_code_roundtrip() {
        // Digits
        for d in '0'..='9' {
            let code = char_code(d);
            assert_eq!(code_char(code), d);
        }
        // Letters
        for c in 'A'..='Z' {
            let code = char_code(c);
            assert_eq!(code_char(code), c);
        }
        // Space
        assert_eq!(code_char(char_code(' ')), ' ');
    }
}
