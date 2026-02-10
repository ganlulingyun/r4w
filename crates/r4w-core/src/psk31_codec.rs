//! PSK31 amateur radio digital mode encoding and decoding.
//!
//! PSK31 is a popular amateur radio digital mode that uses phase-shift keying at
//! 31.25 baud. It was designed by Peter Martinez (G3PLX) for keyboard-to-keyboard
//! QSOs on HF bands. Characters are encoded using Varicode, a variable-length
//! binary code where common characters have shorter representations.
//!
//! # Features
//!
//! - **Varicode encoding/decoding**: ASCII to/from variable-length bit patterns
//! - **BPSK modulation**: 31.25 baud with raised-cosine shaped phase transitions
//! - **QPSK31 variant**: Doubled throughput at 62.5 baud effective rate
//! - **IQ sample generation**: Smooth phase reversals for clean spectrum
//! - **Demodulation**: Phase detection with bit synchronization
//! - **Idle pattern**: `00` bits inserted between characters
//!
//! # Example
//!
//! ```
//! use r4w_core::psk31_codec::{Psk31Codec, Varicode};
//!
//! // Encode text to varicode bits
//! let bits = Varicode::encode_text("CQ");
//! assert!(!bits.is_empty());
//!
//! // Decode varicode bits back to text
//! let text = Varicode::decode_bits(&bits);
//! assert_eq!(text, "CQ");
//!
//! // Full modulation/demodulation round-trip
//! let codec = Psk31Codec::new(8000.0);
//! let samples = codec.modulate_bpsk(&bits);
//! assert!(!samples.is_empty());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Varicode table (ASCII 0..=127)
// ---------------------------------------------------------------------------

/// Varicode representations for ASCII characters 0..=127.
///
/// Each entry is `(pattern, bit_length)`. The pattern is stored as a `u16` with
/// the MSB transmitted first. Two consecutive `0` bits never appear *inside* a
/// codeword, so they serve as the inter-character delimiter.
const VARICODE_TABLE: [(u16, u8); 128] = [
    // ASCII 0-15  (control chars - long codes)
    (0b1010101011, 10),  // NUL  0
    (0b1011011011, 10),  // SOH  1
    (0b1011101101, 10),  // STX  2
    (0b1101110111, 10),  // ETX  3
    (0b1011101011, 10),  // EOT  4
    (0b1101011111, 10),  // ENQ  5
    (0b1011101111, 10),  // ACK  6
    (0b1011111101, 10),  // BEL  7
    (0b1011111111, 10),  // BS   8
    (0b11101111, 8),     // HT   9
    (0b11101, 5),        // LF  10
    (0b1101101111, 10),  // VT  11
    (0b1011011101, 10),  // FF  12
    (0b11111, 5),        // CR  13
    (0b1101110101, 10),  // SO  14
    (0b1110101011, 10),  // SI  15
    // ASCII 16-31
    (0b1011110111, 10),  // DLE 16
    (0b1011110101, 10),  // DC1 17
    (0b1110101101, 10),  // DC2 18
    (0b1110101111, 10),  // DC3 19
    (0b1101011011, 10),  // DC4 20
    (0b1101101011, 10),  // NAK 21
    (0b1101101101, 10),  // SYN 22
    (0b1101010111, 10),  // ETB 23
    (0b1101111011, 10),  // CAN 24
    (0b1101111101, 10),  // EM  25
    (0b1110110111, 10),  // SUB 26
    (0b1101010101, 10),  // ESC 27
    (0b1101011101, 10),  // FS  28
    (0b1110111011, 10),  // GS  29
    (0b1011111011, 10),  // RS  30
    (0b1101111111, 10),  // US  31
    // ASCII 32-47
    (0b1, 1),            // SP  32
    (0b111111111, 9),    // !   33
    (0b101011111, 9),    // "   34
    (0b111110101, 9),    // #   35
    (0b111011011, 9),    // $   36
    (0b1011010101, 10),  // %   37
    (0b1010111011, 10),  // &   38
    (0b101111111, 9),    // '   39
    (0b11111011, 8),     // (   40
    (0b11110111, 8),     // )   41
    (0b101101111, 9),    // *   42
    (0b111011111, 9),    // +   43
    (0b1110101, 7),      // ,   44
    (0b110101, 6),       // -   45
    (0b1010111, 7),      // .   46
    (0b110101111, 9),    // /   47
    // ASCII 48-57 (digits)
    (0b10110111, 8),     // 0   48
    (0b10111101, 8),     // 1   49
    (0b11101101, 8),     // 2   50
    (0b11111111, 8),     // 3   51
    (0b101110111, 9),    // 4   52
    (0b101011011, 9),    // 5   53
    (0b101101011, 9),    // 6   54
    (0b110101101, 9),    // 7   55
    (0b110101011, 9),    // 8   56
    (0b110110111, 9),    // 9   57
    // ASCII 58-64
    (0b11110101, 8),     // :   58
    (0b110111101, 9),    // ;   59
    (0b111101101, 9),    // <   60
    (0b1010101, 7),      // =   61
    (0b111010111, 9),    // >   62
    (0b1010101111, 10),  // ?   63
    (0b1010111101, 10),  // @   64
    // ASCII 65-90 (uppercase)
    (0b1111101, 7),      // A   65
    (0b11101011, 8),     // B   66
    (0b10101101, 8),     // C   67
    (0b10110101, 8),     // D   68
    (0b1110111, 7),      // E   69
    (0b11011011, 8),     // F   70
    (0b11111101, 8),     // G   71
    (0b101010101, 9),    // H   72
    (0b1111111, 7),      // I   73
    (0b111111101, 9),    // J   74
    (0b101111101, 9),    // K   75
    (0b11010111, 8),     // L   76
    (0b10111011, 8),     // M   77
    (0b11011101, 8),     // N   78
    (0b10101011, 8),     // O   79
    (0b11010101, 8),     // P   80
    (0b111011101, 9),    // Q   81
    (0b10101111, 8),     // R   82
    (0b1101111, 7),      // S   83
    (0b1101101, 7),      // T   84
    (0b101010111, 9),    // U   85
    (0b110110101, 9),    // V   86
    (0b101011101, 9),    // W   87
    (0b101110101, 9),    // X   88
    (0b101111011, 9),    // Y   89
    (0b1010101101, 10),  // Z   90
    // ASCII 91-96
    (0b111101111, 9),    // [   91
    (0b111101011, 9),    // \   92
    (0b111110111, 9),    // ]   93
    (0b101101101, 9),    // ^   94
    (0b10111111, 8),     // _   95
    (0b1010111111, 10),  // `   96
    // ASCII 97-122 (lowercase)
    (0b1011, 4),         // a   97
    (0b1011111, 7),      // b   98
    (0b101111, 6),       // c   99
    (0b101101, 6),       // d  100
    (0b11, 2),           // e  101
    (0b111101, 6),       // f  102
    (0b1011011, 7),      // g  103
    (0b101011, 6),       // h  104
    (0b1101, 4),         // i  105
    (0b111111011, 9),    // j  106
    (0b1011011111, 10),  // k  107
    (0b11011, 5),        // l  108
    (0b111011, 6),       // m  109
    (0b1111, 4),         // n  110
    (0b111, 3),          // o  111
    (0b111111, 6),       // p  112
    (0b110111111, 9),    // q  113
    (0b10101, 5),        // r  114
    (0b10111, 5),        // s  115
    (0b101, 3),          // t  116
    (0b110111, 6),       // u  117
    (0b1111011, 7),      // v  118
    (0b1101011, 7),      // w  119
    (0b11011111, 8),     // x  120
    (0b1011101, 7),      // y  121
    (0b111010101, 9),    // z  122
    // ASCII 123-127
    (0b1010110111, 10),  // {  123
    (0b110111011, 9),    // |  124
    (0b1010110101, 10),  // }  125
    (0b1011010111, 10),  // ~  126
    (0b1110110101, 10),  // DEL 127
];

// ---------------------------------------------------------------------------
// Varicode encoder / decoder
// ---------------------------------------------------------------------------

/// Varicode encoding and decoding for PSK31.
///
/// Varicode is a variable-length code designed so that no codeword contains
/// two consecutive zero bits. This property allows `00` to serve as an
/// unambiguous character boundary.
pub struct Varicode;

impl Varicode {
    /// Encode a single ASCII character into its varicode bit pattern.
    ///
    /// Returns `None` for characters outside the 0..=127 range.
    pub fn encode_char(ch: char) -> Option<Vec<bool>> {
        let idx = ch as u32;
        if idx > 127 {
            return None;
        }
        let (pattern, len) = VARICODE_TABLE[idx as usize];
        let mut bits = Vec::with_capacity(len as usize);
        for i in (0..len).rev() {
            bits.push((pattern >> i) & 1 == 1);
        }
        Some(bits)
    }

    /// Encode a text string into varicode bits with `00` delimiters between characters.
    pub fn encode_text(text: &str) -> Vec<bool> {
        let mut bits = Vec::new();
        for (i, ch) in text.chars().enumerate() {
            if i > 0 {
                // Inter-character separator: two zero bits
                bits.push(false);
                bits.push(false);
            }
            if let Some(char_bits) = Self::encode_char(ch) {
                bits.extend(char_bits);
            }
        }
        bits
    }

    /// Decode a varicode bit stream into a `String`.
    ///
    /// Looks for `00` boundaries to split characters. Unrecognised codewords
    /// are silently skipped.
    pub fn decode_bits(bits: &[bool]) -> String {
        let mut result = String::new();
        let mut current: u16 = 0;
        let mut bit_count: u8 = 0;
        let mut zero_count: u8 = 0;

        for &bit in bits {
            if bit {
                // If we had accumulated exactly one zero, it is part of the codeword
                if zero_count == 1 {
                    current = (current << 1) | 0;
                    bit_count += 1;
                }
                zero_count = 0;
                current = (current << 1) | 1;
                bit_count += 1;
            } else {
                zero_count += 1;
                if zero_count >= 2 {
                    // End of character
                    if bit_count > 0 {
                        if let Some(ch) = Self::lookup_varicode(current, bit_count) {
                            result.push(ch);
                        }
                    }
                    current = 0;
                    bit_count = 0;
                    zero_count = 0;
                }
            }
        }
        // Flush remaining (character at end without trailing 00)
        if bit_count > 0 {
            if let Some(ch) = Self::lookup_varicode(current, bit_count) {
                result.push(ch);
            }
        }
        result
    }

    /// Look up a varicode pattern and return the corresponding ASCII character.
    fn lookup_varicode(pattern: u16, len: u8) -> Option<char> {
        for (i, &(p, l)) in VARICODE_TABLE.iter().enumerate() {
            if l == len && p == pattern {
                return Some(i as u8 as char);
            }
        }
        None
    }
}

// ---------------------------------------------------------------------------
// PSK31 Codec
// ---------------------------------------------------------------------------

/// PSK31 modulation mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Psk31Mode {
    /// Binary PSK at 31.25 baud.
    Bpsk,
    /// Quaternary PSK at 31.25 symbols/s (62.5 bits/s effective).
    Qpsk,
}

/// PSK31 codec for modulation and demodulation.
///
/// Provides BPSK31 and QPSK31 modulation/demodulation with raised-cosine
/// shaped phase transitions.
pub struct Psk31Codec {
    /// Audio sample rate in Hz.
    sample_rate: f64,
    /// Baud rate (31.25 for PSK31).
    baud_rate: f64,
    /// Carrier frequency for modulation (default 1000 Hz).
    carrier_freq: f64,
}

impl Psk31Codec {
    /// Standard PSK31 baud rate.
    pub const BAUD_RATE: f64 = 31.25;

    /// Create a new codec with the given sample rate and default carrier of 1000 Hz.
    pub fn new(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            baud_rate: Self::BAUD_RATE,
            carrier_freq: 1000.0,
        }
    }

    /// Create a codec with a custom carrier frequency.
    pub fn with_carrier(sample_rate: f64, carrier_freq: f64) -> Self {
        Self {
            sample_rate,
            baud_rate: Self::BAUD_RATE,
            carrier_freq,
        }
    }

    /// Return the number of samples per baud interval.
    pub fn samples_per_symbol(&self) -> usize {
        (self.sample_rate / self.baud_rate).round() as usize
    }

    /// Return the configured sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Return the baud rate.
    pub fn baud_rate(&self) -> f64 {
        self.baud_rate
    }

    // -----------------------------------------------------------------------
    // BPSK modulation
    // -----------------------------------------------------------------------

    /// Modulate varicode bits into BPSK IQ samples.
    ///
    /// A `0` bit causes a 180-degree phase reversal; a `1` bit keeps the
    /// current phase. Phase transitions are shaped with a raised-cosine
    /// envelope to keep the signal bandwidth narrow.
    pub fn modulate_bpsk(&self, bits: &[bool]) -> Vec<(f64, f64)> {
        let sps = self.samples_per_symbol();
        let mut samples = Vec::with_capacity(bits.len() * sps);
        let mut phase = 0.0_f64; // current carrier phase offset (0 or PI)

        for (bit_idx, &bit) in bits.iter().enumerate() {
            let prev_phase = phase;
            if !bit {
                // Phase reversal for '0' bit
                phase += PI;
            }
            // '1' bit: no phase change

            let phase_delta = phase - prev_phase;

            for k in 0..sps {
                let t = k as f64 / sps as f64; // 0..1 within symbol
                let carrier_phase =
                    2.0 * PI * self.carrier_freq * (bit_idx * sps + k) as f64 / self.sample_rate;

                // Raised-cosine shaping during transition
                let current_phase = if phase_delta.abs() > 0.01 {
                    // Shape the transition: use raised-cosine window for smooth reversal
                    let rc = 0.5 * (1.0 - (PI * t).cos()); // 0->1 over the symbol
                    prev_phase + phase_delta * rc
                } else {
                    phase
                };

                let re = (carrier_phase + current_phase).cos();
                let im = (carrier_phase + current_phase).sin();
                samples.push((re, im));
            }
        }

        samples
    }

    // -----------------------------------------------------------------------
    // BPSK demodulation
    // -----------------------------------------------------------------------

    /// Demodulate BPSK IQ samples back to bits.
    ///
    /// Uses differential correlation: compute the dot product of adjacent
    /// symbols' baseband representations. Positive means same phase (bit 1),
    /// negative means phase reversal (bit 0).
    pub fn demodulate_bpsk(&self, samples: &[(f64, f64)]) -> Vec<bool> {
        let sps = self.samples_per_symbol();
        if samples.len() < 2 * sps {
            return Vec::new();
        }

        let num_symbols = samples.len() / sps;
        let mut bits = Vec::with_capacity(num_symbols);

        // Mix each symbol down to baseband and store I/Q sums.
        // Use only the latter portion of each symbol (after raised-cosine
        // transition has settled) for accurate phase estimation.
        let start_frac = sps * 3 / 4; // last quarter of symbol
        let mut baseband: Vec<(f64, f64)> = Vec::with_capacity(num_symbols);
        for sym in 0..num_symbols {
            let mut sum_i = 0.0;
            let mut sum_q = 0.0;
            for k in start_frac..sps {
                let idx = sym * sps + k;
                if idx >= samples.len() {
                    break;
                }
                let carrier_phase =
                    2.0 * PI * self.carrier_freq * idx as f64 / self.sample_rate;
                let (re, im) = samples[idx];
                sum_i += re * carrier_phase.cos() + im * carrier_phase.sin();
                sum_q += -re * carrier_phase.sin() + im * carrier_phase.cos();
            }
            baseband.push((sum_i, sum_q));
        }

        // Differential decode: dot product of adjacent baseband vectors
        for i in 1..baseband.len() {
            let (i0, q0) = baseband[i - 1];
            let (i1, q1) = baseband[i];
            let dot = i0 * i1 + q0 * q1;
            // Positive dot product -> same phase -> bit 1
            // Negative dot product -> phase reversal -> bit 0
            bits.push(dot > 0.0);
        }

        bits
    }

    // -----------------------------------------------------------------------
    // QPSK modulation
    // -----------------------------------------------------------------------

    /// Modulate bits using QPSK31 (two bits per symbol at 31.25 symbols/s).
    ///
    /// Bits are taken in pairs (MSB first). Each dibit selects a phase
    /// increment: 00->0deg, 01->90deg, 11->180deg, 10->270deg.
    pub fn modulate_qpsk(&self, bits: &[bool]) -> Vec<(f64, f64)> {
        let sps = self.samples_per_symbol();
        // Pad to even length
        let mut padded = bits.to_vec();
        if padded.len() % 2 != 0 {
            padded.push(false);
        }
        let num_symbols = padded.len() / 2;
        let mut samples = Vec::with_capacity(num_symbols * sps);
        let mut phase = 0.0_f64;

        for sym_idx in 0..num_symbols {
            let b0 = padded[sym_idx * 2];
            let b1 = padded[sym_idx * 2 + 1];
            let prev_phase = phase;

            // Dibit to phase increment
            let delta = match (b0, b1) {
                (false, false) => 0.0,
                (false, true) => PI / 2.0,
                (true, true) => PI,
                (true, false) => 3.0 * PI / 2.0,
            };
            phase += delta;

            for k in 0..sps {
                let t = k as f64 / sps as f64;
                let carrier_phase =
                    2.0 * PI * self.carrier_freq * (sym_idx * sps + k) as f64 / self.sample_rate;

                // Raised-cosine shaping
                let current_phase = if delta.abs() > 0.01 {
                    let rc = 0.5 * (1.0 - (PI * t).cos());
                    prev_phase + delta * rc
                } else {
                    phase
                };

                let re = (carrier_phase + current_phase).cos();
                let im = (carrier_phase + current_phase).sin();
                samples.push((re, im));
            }
        }

        samples
    }

    /// Demodulate QPSK31 IQ samples back to bits.
    pub fn demodulate_qpsk(&self, samples: &[(f64, f64)]) -> Vec<bool> {
        let sps = self.samples_per_symbol();
        if samples.len() < 2 * sps {
            return Vec::new();
        }

        let num_symbols = samples.len() / sps;
        let mut bits = Vec::new();

        // Mix each symbol down to baseband (last quarter for settled phase)
        let start_frac = sps * 3 / 4;
        let mut baseband: Vec<(f64, f64)> = Vec::with_capacity(num_symbols);
        for sym in 0..num_symbols {
            let mut sum_i = 0.0;
            let mut sum_q = 0.0;
            for k in start_frac..sps {
                let idx = sym * sps + k;
                if idx >= samples.len() {
                    break;
                }
                let carrier_phase =
                    2.0 * PI * self.carrier_freq * idx as f64 / self.sample_rate;
                let (re, im) = samples[idx];
                sum_i += re * carrier_phase.cos() + im * carrier_phase.sin();
                sum_q += -re * carrier_phase.sin() + im * carrier_phase.cos();
            }
            baseband.push((sum_i, sum_q));
        }

        // Differential decode via complex conjugate multiply:
        // result = conj(prev) * curr -> angle gives phase increment
        for i in 1..baseband.len() {
            let (i0, q0) = baseband[i - 1];
            let (i1, q1) = baseband[i];
            // conj(prev) * curr = (i0 - j*q0) * (i1 + j*q1)
            //                   = (i0*i1 + q0*q1) + j*(i0*q1 - q0*i1)
            let diff_i = i0 * i1 + q0 * q1;
            let diff_q = i0 * q1 - q0 * i1;
            let delta = diff_q.atan2(diff_i);

            // Quantise to nearest quadrant: 0, PI/2, PI, -PI/2
            let quadrant = if delta.abs() < PI / 4.0 {
                0 // ~0: no rotation
            } else if delta > 0.0 && delta < 3.0 * PI / 4.0 {
                1 // ~+PI/2
            } else if delta.abs() > 3.0 * PI / 4.0 {
                2 // ~PI (or ~-PI)
            } else {
                3 // ~-PI/2 (which is +3PI/2 unwrapped)
            };
            let (b0, b1) = match quadrant {
                0 => (false, false),
                1 => (false, true),
                2 => (true, true),
                3 => (true, false),
                _ => (false, false),
            };
            bits.push(b0);
            bits.push(b1);
        }

        bits
    }

    // -----------------------------------------------------------------------
    // Idle / convenience
    // -----------------------------------------------------------------------

    /// Generate idle tone (continuous `1` bits - no phase change).
    ///
    /// `duration_secs` specifies how many seconds of idle to produce.
    pub fn generate_idle(&self, duration_secs: f64) -> Vec<(f64, f64)> {
        let num_bits = (duration_secs * self.baud_rate).round() as usize;
        let idle_bits = vec![true; num_bits];
        self.modulate_bpsk(&idle_bits)
    }

    /// Generate the `00` inter-character idle pattern as raw bits.
    pub fn idle_bits() -> Vec<bool> {
        vec![false, false]
    }

    /// Encode text, modulate with BPSK, and return IQ samples.
    ///
    /// Convenience method that chains varicode encoding and modulation.
    pub fn encode_and_modulate(&self, text: &str) -> Vec<(f64, f64)> {
        let bits = Varicode::encode_text(text);
        self.modulate_bpsk(&bits)
    }

    /// Demodulate BPSK IQ samples and decode the varicode back to text.
    pub fn demodulate_and_decode(&self, samples: &[(f64, f64)]) -> String {
        let bits = self.demodulate_bpsk(samples);
        Varicode::decode_bits(&bits)
    }
}

// ---------------------------------------------------------------------------
// Helper: normalise phase difference to (-pi, pi]
// ---------------------------------------------------------------------------

fn phase_diff(a: f64, b: f64) -> f64 {
    let mut d = a - b;
    while d > PI {
        d -= 2.0 * PI;
    }
    while d <= -PI {
        d += 2.0 * PI;
    }
    d
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -- Varicode encoding --------------------------------------------------

    #[test]
    fn varicode_encode_space() {
        // Space (ASCII 32) is the shortest: a single '1' bit
        let bits = Varicode::encode_char(' ').unwrap();
        assert_eq!(bits, vec![true]);
    }

    #[test]
    fn varicode_encode_e() {
        // 'e' (ASCII 101) = 0b11, two bits
        let bits = Varicode::encode_char('e').unwrap();
        assert_eq!(bits, vec![true, true]);
    }

    #[test]
    fn varicode_encode_t() {
        // 't' (ASCII 116) = 0b101, three bits
        let bits = Varicode::encode_char('t').unwrap();
        assert_eq!(bits, vec![true, false, true]);
    }

    #[test]
    fn varicode_non_ascii_returns_none() {
        assert!(Varicode::encode_char('\u{00E9}').is_none()); // e with accent
        assert!(Varicode::encode_char('\u{1F600}').is_none()); // emoji
    }

    #[test]
    fn varicode_encode_text_cq() {
        let bits = Varicode::encode_text("CQ");
        // C = 10101101 (8 bits), separator 00, Q = 111011101 (9 bits)
        assert!(!bits.is_empty());
        // Decode should round-trip
        let decoded = Varicode::decode_bits(&bits);
        assert_eq!(decoded, "CQ");
    }

    #[test]
    fn varicode_round_trip_alphabet() {
        let text = "abcdefghijklmnopqrstuvwxyz";
        let bits = Varicode::encode_text(text);
        let decoded = Varicode::decode_bits(&bits);
        assert_eq!(decoded, text);
    }

    #[test]
    fn varicode_round_trip_digits() {
        let text = "0123456789";
        let bits = Varicode::encode_text(text);
        let decoded = Varicode::decode_bits(&bits);
        assert_eq!(decoded, text);
    }

    #[test]
    fn varicode_round_trip_sentence() {
        let text = "CQ CQ de W1AW";
        let bits = Varicode::encode_text(text);
        let decoded = Varicode::decode_bits(&bits);
        assert_eq!(decoded, text);
    }

    #[test]
    fn varicode_empty_string() {
        let bits = Varicode::encode_text("");
        assert!(bits.is_empty());
        let decoded = Varicode::decode_bits(&bits);
        assert_eq!(decoded, "");
    }

    #[test]
    fn varicode_no_internal_double_zero() {
        // Every codeword must not contain two consecutive zero bits
        for i in 0..128u8 {
            let (pattern, len) = VARICODE_TABLE[i as usize];
            let mut prev_zero = false;
            for bit_pos in (0..len).rev() {
                let bit = (pattern >> bit_pos) & 1 == 0;
                if bit && prev_zero {
                    panic!(
                        "Character {} (ASCII {}) has internal 00 in varicode",
                        i as char, i
                    );
                }
                prev_zero = bit;
            }
        }
    }

    // -- BPSK modulation / demodulation -------------------------------------

    #[test]
    fn bpsk_samples_per_symbol() {
        let codec = Psk31Codec::new(8000.0);
        // 8000 / 31.25 = 256
        assert_eq!(codec.samples_per_symbol(), 256);
    }

    #[test]
    fn bpsk_modulate_produces_correct_length() {
        let codec = Psk31Codec::new(8000.0);
        let bits = vec![true, false, true, true, false];
        let samples = codec.modulate_bpsk(&bits);
        assert_eq!(samples.len(), 5 * 256);
    }

    #[test]
    fn bpsk_modulate_amplitude_bounded() {
        let codec = Psk31Codec::new(8000.0);
        let bits = vec![true, false, true, false, true, false, false, true];
        let samples = codec.modulate_bpsk(&bits);
        for (re, im) in &samples {
            let mag = (re * re + im * im).sqrt();
            assert!(mag <= 1.01, "Magnitude {} exceeds 1.0", mag);
        }
    }

    #[test]
    fn bpsk_round_trip_simple() {
        let codec = Psk31Codec::new(8000.0);
        // Prefix with a known-phase reference symbol (1 bit) then the payload
        let mut bits = vec![true]; // reference
        bits.extend_from_slice(&[true, false, true, true, false, true, false, false]);
        let samples = codec.modulate_bpsk(&bits);
        let demod = codec.demodulate_bpsk(&samples);
        // demod has one fewer bit (differential) so the reference bit is consumed
        assert_eq!(demod.len(), bits.len() - 1);
        assert_eq!(&demod, &bits[1..]);
    }

    #[test]
    fn bpsk_round_trip_all_ones() {
        let codec = Psk31Codec::new(8000.0);
        let bits = vec![true; 20];
        let samples = codec.modulate_bpsk(&bits);
        let demod = codec.demodulate_bpsk(&samples);
        // All ones = no phase change, differential gives all ones
        for &b in &demod {
            assert!(b, "Expected all 1-bits for constant phase");
        }
    }

    #[test]
    fn bpsk_round_trip_all_zeros() {
        let codec = Psk31Codec::new(8000.0);
        let bits = vec![true, false, false, false, false, false]; // ref + zeros
        let samples = codec.modulate_bpsk(&bits);
        let demod = codec.demodulate_bpsk(&samples);
        assert_eq!(demod.len(), 5);
        for &b in &demod {
            assert!(!b, "Expected all 0-bits for alternating phase");
        }
    }

    // -- QPSK modulation / demodulation -------------------------------------

    #[test]
    fn qpsk_modulate_produces_samples() {
        let codec = Psk31Codec::new(8000.0);
        let bits = vec![false, true, true, false, true, true, false, false];
        let samples = codec.modulate_qpsk(&bits);
        // 8 bits -> 4 symbols -> 4 * 256 samples
        assert_eq!(samples.len(), 4 * 256);
    }

    #[test]
    fn qpsk_round_trip() {
        let codec = Psk31Codec::new(8000.0);
        // Reference symbol (00 = no change) then payload
        let bits = vec![
            false, false, // ref symbol (no rotation)
            false, true,  // +90 deg
            true, true,   // +180 deg
            true, false,  // +270 deg
            false, false,  // 0 deg
        ];
        let samples = codec.modulate_qpsk(&bits);
        let demod = codec.demodulate_qpsk(&samples);
        // Differential: first symbol consumed as reference
        assert_eq!(demod.len(), 8); // 4 decoded symbols x 2 bits
        assert_eq!(&demod[0..2], &[false, true]);
        assert_eq!(&demod[2..4], &[true, true]);
        assert_eq!(&demod[4..6], &[true, false]);
        assert_eq!(&demod[6..8], &[false, false]);
    }

    // -- Idle ---------------------------------------------------------------

    #[test]
    fn idle_bits_are_double_zero() {
        assert_eq!(Psk31Codec::idle_bits(), vec![false, false]);
    }

    #[test]
    fn generate_idle_length() {
        let codec = Psk31Codec::new(8000.0);
        let idle = codec.generate_idle(1.0);
        // ~31.25 symbols x 256 samples = 8000 samples (approximately)
        let expected = (1.0_f64 * 31.25).round() as usize * 256;
        assert_eq!(idle.len(), expected);
    }

    // -- Integration --------------------------------------------------------

    #[test]
    fn full_encode_modulate_pipeline() {
        let codec = Psk31Codec::new(8000.0);
        let samples = codec.encode_and_modulate("TEST");
        assert!(!samples.is_empty());
        // Verify amplitude stays bounded
        for (re, im) in &samples {
            let mag = (re * re + im * im).sqrt();
            assert!(mag <= 1.01);
        }
    }

    #[test]
    fn codec_with_carrier() {
        let codec = Psk31Codec::with_carrier(8000.0, 1500.0);
        assert_eq!(codec.sample_rate(), 8000.0);
        assert_eq!(codec.baud_rate(), 31.25);
        let samples = codec.modulate_bpsk(&[true, false, true]);
        assert_eq!(samples.len(), 3 * 256);
    }

    // -- Phase helper -------------------------------------------------------

    #[test]
    fn phase_diff_wraps_correctly() {
        let d1 = phase_diff(3.0, -3.0);
        assert!(d1.abs() < PI + 0.01);

        let d2 = phase_diff(0.1, -0.1);
        assert!((d2 - 0.2).abs() < 1e-10);

        let d3 = phase_diff(0.0, 0.0);
        assert!(d3.abs() < 1e-10);
    }
}
