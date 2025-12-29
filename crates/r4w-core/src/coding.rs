//! Coding and Error Correction for LoRa
//!
//! This module implements the various coding stages in the LoRa pipeline:
//!
//! 1. **Gray Coding**: Maps symbols to minimize bit errors from small frequency errors
//! 2. **Hamming FEC**: Forward Error Correction using Hamming codes
//! 3. **Interleaving**: Spreads errors across multiple codewords
//!
//! ## Processing Pipeline
//!
//! ```text
//! TX: Data → Hamming → Interleave → Gray
//! RX: Gray⁻¹ → De-interleave → Hamming Decode → Data
//! ```
//!
//! ## Gray Coding
//!
//! Gray code ensures that adjacent symbols differ by only one bit.
//! This is important because small frequency errors might cause the
//! receiver to decode an adjacent symbol, resulting in only 1 bit error
//! instead of potentially many.
//!
//! ```text
//! Binary  Gray
//! 000     000
//! 001     001
//! 010     011  ← Only 1 bit different from neighbors
//! 011     010
//! 100     110
//! ...
//! ```
//!
//! ## Hamming Codes
//!
//! LoRa uses Hamming codes for FEC. These can detect 2-bit errors and
//! correct 1-bit errors per codeword.
//!
//! Coding rates:
//! - 4/5: 1 parity bit per 4 data bits (can detect single errors)
//! - 4/6: 2 parity bits per 4 data bits
//! - 4/7: 3 parity bits per 4 data bits
//! - 4/8: 4 parity bits per 4 data bits (full Hamming)

use crate::params::CodingRate;
use crate::types::Symbol;

/// Gray code encoder/decoder
///
/// Gray code has the property that adjacent values differ by only one bit,
/// making it robust against small errors in symbol detection.
#[derive(Debug, Clone)]
pub struct GrayCode {
    /// Number of bits (SF value)
    bits: u8,
    /// Lookup table: binary → gray
    encode_lut: Vec<u16>,
    /// Lookup table: gray → binary
    decode_lut: Vec<u16>,
}

impl GrayCode {
    /// Create a Gray code converter for the given number of bits
    pub fn new(bits: u8) -> Self {
        let size = 1usize << bits;
        let mut encode_lut = vec![0u16; size];
        let mut decode_lut = vec![0u16; size];

        // Generate Gray code: g = n ^ (n >> 1)
        for n in 0..size {
            let gray = (n ^ (n >> 1)) as u16;
            encode_lut[n] = gray;
            decode_lut[gray as usize] = n as u16;
        }

        Self {
            bits,
            encode_lut,
            decode_lut,
        }
    }

    /// Encode a binary value to Gray code
    #[inline]
    pub fn encode(&self, value: Symbol) -> Symbol {
        self.encode_lut[value as usize]
    }

    /// Decode a Gray code value to binary
    #[inline]
    pub fn decode(&self, gray: Symbol) -> Symbol {
        self.decode_lut[gray as usize]
    }

    /// Encode multiple symbols
    pub fn encode_all(&self, symbols: &[Symbol]) -> Vec<Symbol> {
        symbols.iter().map(|&s| self.encode(s)).collect()
    }

    /// Decode multiple symbols
    pub fn decode_all(&self, symbols: &[Symbol]) -> Vec<Symbol> {
        symbols.iter().map(|&s| self.decode(s)).collect()
    }

    /// Show the Gray code table (for educational purposes)
    pub fn table(&self) -> Vec<(u16, u16, String, String)> {
        let size = 1usize << self.bits;
        let mut table = Vec::with_capacity(size);

        for n in 0..size {
            let binary = n as u16;
            let gray = self.encode_lut[n];
            let binary_str = format!("{:0width$b}", binary, width = self.bits as usize);
            let gray_str = format!("{:0width$b}", gray, width = self.bits as usize);
            table.push((binary, gray, binary_str, gray_str));
        }

        table
    }
}

/// Hamming code encoder/decoder for LoRa FEC
///
/// LoRa uses systematic Hamming codes where data bits are passed through
/// unchanged and parity bits are appended.
#[derive(Debug, Clone)]
pub struct HammingCode {
    /// Coding rate
    rate: CodingRate,
    /// Parity matrix (generator)
    parity_matrix: Vec<Vec<u8>>,
}

impl HammingCode {
    /// Create a Hamming coder for the given coding rate
    pub fn new(rate: CodingRate) -> Self {
        let parity_matrix = Self::get_parity_matrix(rate);
        Self {
            rate,
            parity_matrix,
        }
    }

    /// Get the parity generation matrix for a coding rate
    ///
    /// These matrices are from the LoRa specification/reverse engineering
    fn get_parity_matrix(rate: CodingRate) -> Vec<Vec<u8>> {
        match rate {
            CodingRate::CR4_5 => {
                // 4/5: 1 parity bit (simple parity)
                vec![vec![1, 1, 1, 1]]
            }
            CodingRate::CR4_6 => {
                // 4/6: 2 parity bits
                vec![vec![1, 0, 1, 1], vec![0, 1, 1, 1]]
            }
            CodingRate::CR4_7 => {
                // 4/7: 3 parity bits
                vec![vec![1, 0, 1, 1], vec![1, 1, 1, 0], vec![0, 1, 1, 1]]
            }
            CodingRate::CR4_8 => {
                // 4/8: 4 parity bits (full Hamming 8,4)
                vec![
                    vec![1, 0, 1, 1],
                    vec![1, 1, 1, 0],
                    vec![1, 1, 0, 1],
                    vec![0, 1, 1, 1],
                ]
            }
        }
    }

    /// Encode 4 data bits into 4+CR bits
    ///
    /// Returns the codeword with data bits followed by parity bits
    pub fn encode(&self, data: u8) -> u8 {
        let data_bits: Vec<u8> = (0..4).map(|i| (data >> (3 - i)) & 1).collect();

        let mut codeword = data & 0x0F; // Keep lower 4 bits

        // Calculate parity bits
        for (i, row) in self.parity_matrix.iter().enumerate() {
            let parity: u8 = row.iter().zip(data_bits.iter()).map(|(&p, &d)| p & d).sum::<u8>() & 1;

            codeword |= parity << (4 + i);
        }

        codeword
    }

    /// Decode a codeword and correct single-bit errors if possible
    ///
    /// Returns (decoded_data, error_detected, error_corrected)
    pub fn decode(&self, codeword: u8) -> (u8, bool, bool) {
        let n_parity = self.rate.value() as usize;
        let data_bits: Vec<u8> = (0..4).map(|i| (codeword >> (3 - i)) & 1).collect();
        let parity_bits: Vec<u8> = (0..n_parity).map(|i| (codeword >> (4 + i)) & 1).collect();

        // Calculate syndrome (expected parity XOR received parity)
        let mut syndrome = 0u8;
        for (i, row) in self.parity_matrix.iter().enumerate() {
            let expected: u8 = row.iter().zip(data_bits.iter()).map(|(&p, &d)| p & d).sum::<u8>() & 1;
            let actual = parity_bits[i];
            if expected != actual {
                syndrome |= 1 << i;
            }
        }

        let error_detected = syndrome != 0;
        let mut data = codeword & 0x0F;
        let mut error_corrected = false;

        // Attempt error correction for CR 4/8 (full Hamming)
        if error_detected && self.rate == CodingRate::CR4_8 {
            // Syndrome indicates error position in Hamming code
            // This is a simplified correction - full implementation would
            // use proper syndrome decoding
            if syndrome > 0 && syndrome <= 8 {
                if syndrome <= 4 {
                    // Error in data bits
                    data ^= 1 << (4 - syndrome);
                    error_corrected = true;
                }
                // Error in parity bits can be ignored for data recovery
            }
        }

        (data, error_detected, error_corrected)
    }

    /// Get the coding rate
    pub fn rate(&self) -> CodingRate {
        self.rate
    }
}

/// Interleaver for LoRa
///
/// Interleaving spreads consecutive bits/symbols across the transmission,
/// so that burst errors affect multiple codewords slightly rather than
/// destroying one codeword completely.
///
/// LoRa uses a diagonal interleaver pattern.
#[derive(Debug, Clone)]
pub struct Interleaver {
    /// Spreading factor (determines symbol size)
    sf: u8,
    /// Coding rate (determines codeword size)
    cr: CodingRate,
}

impl Interleaver {
    /// Create a new interleaver
    pub fn new(sf: u8, cr: CodingRate) -> Self {
        Self { sf, cr }
    }

    /// Interleave a block of codewords
    ///
    /// Takes SF codewords of (4+CR) bits each and produces (4+CR) symbols
    /// of SF bits each, with diagonal interleaving.
    pub fn interleave(&self, codewords: &[u8]) -> Vec<Symbol> {
        let sf = self.sf as usize;
        let n_bits = self.cr.output_bits() as usize;

        // We need exactly SF codewords
        assert!(codewords.len() >= sf);

        let mut symbols = vec![0u16; n_bits];

        // Diagonal interleaving
        for (i, &cw) in codewords.iter().take(sf).enumerate() {
            for j in 0..n_bits {
                let bit = (cw >> j) & 1;
                let sym_idx = (i + j) % n_bits;
                let bit_pos = i;
                symbols[sym_idx] |= (bit as u16) << bit_pos;
            }
        }

        symbols
    }

    /// De-interleave a block of symbols
    ///
    /// Reverses the interleaving process.
    pub fn deinterleave(&self, symbols: &[Symbol]) -> Vec<u8> {
        let sf = self.sf as usize;
        let n_bits = self.cr.output_bits() as usize;

        assert!(symbols.len() >= n_bits);

        let mut codewords = vec![0u8; sf];

        // Reverse diagonal interleaving
        for i in 0..sf {
            for j in 0..n_bits {
                let sym_idx = (i + j) % n_bits;
                let bit = (symbols[sym_idx] >> i) & 1;
                codewords[i] |= (bit as u8) << j;
            }
        }

        codewords
    }
}

/// Complete encoder combining all stages
pub struct LoRaEncoder {
    gray: GrayCode,
    hamming: HammingCode,
    interleaver: Interleaver,
}

impl LoRaEncoder {
    /// Create a new encoder
    pub fn new(sf: u8, cr: CodingRate) -> Self {
        Self {
            gray: GrayCode::new(sf),
            hamming: HammingCode::new(cr),
            interleaver: Interleaver::new(sf, cr),
        }
    }

    /// Encode a block of nibbles (4-bit values) to symbols
    ///
    /// Process: Hamming encode → Interleave → Gray encode
    pub fn encode_block(&self, nibbles: &[u8]) -> Vec<Symbol> {
        // Hamming encode each nibble
        let codewords: Vec<u8> = nibbles.iter().map(|&n| self.hamming.encode(n)).collect();

        // Interleave
        let symbols = self.interleaver.interleave(&codewords);

        // Gray encode
        self.gray.encode_all(&symbols)
    }
}

/// Complete decoder combining all stages
pub struct LoRaDecoder {
    gray: GrayCode,
    hamming: HammingCode,
    interleaver: Interleaver,
}

impl LoRaDecoder {
    /// Create a new decoder
    pub fn new(sf: u8, cr: CodingRate) -> Self {
        Self {
            gray: GrayCode::new(sf),
            hamming: HammingCode::new(cr),
            interleaver: Interleaver::new(sf, cr),
        }
    }

    /// Decode a block of symbols to nibbles
    ///
    /// Process: Gray decode → De-interleave → Hamming decode
    pub fn decode_block(&self, symbols: &[Symbol]) -> Vec<u8> {
        // Gray decode
        let gray_decoded = self.gray.decode_all(symbols);

        // De-interleave
        let codewords = self.interleaver.deinterleave(&gray_decoded);

        // Hamming decode
        codewords.iter().map(|&cw| self.hamming.decode(cw).0).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gray_code_adjacent() {
        let gray = GrayCode::new(4);

        // Check that adjacent codes differ by exactly 1 bit
        for i in 0..15u16 {
            let g1 = gray.encode(i);
            let g2 = gray.encode(i + 1);
            let diff = g1 ^ g2;
            assert_eq!(diff.count_ones(), 1, "Gray codes {} and {} differ by {} bits", i, i + 1, diff.count_ones());
        }
    }

    #[test]
    fn test_gray_code_roundtrip() {
        for bits in 4..=8 {
            let gray = GrayCode::new(bits);
            for i in 0..(1u16 << bits) {
                let encoded = gray.encode(i);
                let decoded = gray.decode(encoded);
                assert_eq!(i, decoded);
            }
        }
    }

    #[test]
    fn test_hamming_encode_decode() {
        for cr in [CodingRate::CR4_5, CodingRate::CR4_6, CodingRate::CR4_7, CodingRate::CR4_8] {
            let hamming = HammingCode::new(cr);

            for data in 0..16u8 {
                let encoded = hamming.encode(data);
                let (decoded, _, _) = hamming.decode(encoded);
                assert_eq!(data, decoded, "Hamming {:?} failed for data {}", cr, data);
            }
        }
    }

    #[test]
    fn test_interleaver_roundtrip() {
        let interleaver = Interleaver::new(7, CodingRate::CR4_5);

        let original: Vec<u8> = (0..7).map(|i| i * 2).collect();
        let interleaved = interleaver.interleave(&original);
        let deinterleaved = interleaver.deinterleave(&interleaved);

        assert_eq!(original, deinterleaved);
    }
}
