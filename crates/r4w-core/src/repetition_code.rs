//! # Repetition Code — Simplest Forward Error Correction
//!
//! This module implements **repetition coding**, the most elementary form of FEC.
//! Each input bit is transmitted N times (where N is odd), and the decoder uses
//! majority-logic voting to recover the original bit. Soft-decision decoding via
//! LLR (log-likelihood ratio) summation is also supported for improved performance
//! when channel reliability information is available.
//!
//! ## Properties
//!
//! | Parameter | Value |
//! |---|---|
//! | Code rate | 1/N |
//! | Error correction capability | floor((N-1)/2) bit errors per codeword |
//! | Minimum distance | N |
//!
//! ## Example
//!
//! ```
//! use r4w_core::repetition_code::RepetitionCode;
//!
//! // Create a rate-1/3 repetition code
//! let code = RepetitionCode::new(3).unwrap();
//! assert_eq!(code.code_rate(), 1.0 / 3.0);
//!
//! // Encode a bit sequence
//! let data = vec![true, false, true];
//! let encoded = code.encode(&data);
//! assert_eq!(encoded, vec![true, true, true, false, false, false, true, true, true]);
//!
//! // Introduce one error per codeword (correctable for N=3)
//! let mut received = encoded.clone();
//! received[0] = false; // flip one bit in first codeword
//! received[4] = true;  // flip one bit in second codeword
//!
//! let (decoded, stats) = code.decode_hard(&received);
//! assert_eq!(decoded, data);
//! assert_eq!(stats.corrected_errors, 2);
//! assert_eq!(stats.uncorrectable_blocks, 0);
//! ```

use std::fmt;

/// Statistics returned by the decoder.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecodingStats {
    /// Total number of bit errors that were corrected by majority-logic voting.
    pub corrected_errors: usize,
    /// Number of codeword blocks where majority voting still yielded the wrong
    /// result (more than floor((N-1)/2) errors in a single block). This is only
    /// detectable via soft decoding or external CRC; for hard decoding this
    /// field counts blocks where the vote was exactly tied (N even — which we
    /// disallow — so this is always 0 for valid codes) or where the caller can
    /// verify via other means.
    pub uncorrectable_blocks: usize,
    /// Total number of codeword blocks processed.
    pub total_blocks: usize,
}

impl DecodingStats {
    fn new() -> Self {
        Self {
            corrected_errors: 0,
            uncorrectable_blocks: 0,
            total_blocks: 0,
        }
    }
}

/// A repetition code with configurable odd repeat factor N.
///
/// The encoder repeats each input bit N times. The decoder uses majority-logic
/// voting (hard decision) or LLR summation (soft decision) to recover the
/// original data.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RepetitionCode {
    n: usize,
}

/// Errors that can occur when constructing or using a [`RepetitionCode`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RepetitionCodeError {
    /// The repeat factor must be an odd integer >= 1.
    InvalidRepeatFactor(usize),
    /// The encoded data length is not a multiple of N.
    InvalidEncodedLength { length: usize, n: usize },
}

impl fmt::Display for RepetitionCodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidRepeatFactor(n) => {
                write!(f, "repeat factor must be odd and >= 1, got {n}")
            }
            Self::InvalidEncodedLength { length, n } => {
                write!(f, "encoded length {length} is not a multiple of N={n}")
            }
        }
    }
}

impl std::error::Error for RepetitionCodeError {}

impl RepetitionCode {
    /// Create a new repetition code with repeat factor `n`.
    ///
    /// `n` must be an odd positive integer. Returns an error otherwise.
    pub fn new(n: usize) -> Result<Self, RepetitionCodeError> {
        if n == 0 || n % 2 == 0 {
            return Err(RepetitionCodeError::InvalidRepeatFactor(n));
        }
        Ok(Self { n })
    }

    /// The repeat factor N.
    pub fn repeat_factor(&self) -> usize {
        self.n
    }

    /// The code rate = 1/N.
    pub fn code_rate(&self) -> f64 {
        1.0 / self.n as f64
    }

    /// Maximum number of bit errors correctable per codeword: floor((N-1)/2).
    pub fn error_correction_capability(&self) -> usize {
        (self.n - 1) / 2
    }

    /// Minimum Hamming distance of the code, which equals N.
    pub fn minimum_distance(&self) -> usize {
        self.n
    }

    // -- Bit-level interface --------------------------------------------------

    /// Encode a sequence of bits by repeating each bit N times.
    pub fn encode(&self, bits: &[bool]) -> Vec<bool> {
        let mut out = Vec::with_capacity(bits.len() * self.n);
        for &bit in bits {
            for _ in 0..self.n {
                out.push(bit);
            }
        }
        out
    }

    /// Decode hard-decision bits using majority-logic voting.
    ///
    /// The input length must be a multiple of N. Returns the decoded bits and
    /// decoding statistics.
    pub fn decode_hard(&self, bits: &[bool]) -> (Vec<bool>, DecodingStats) {
        let num_blocks = bits.len() / self.n;
        let mut decoded = Vec::with_capacity(num_blocks);
        let mut stats = DecodingStats::new();
        stats.total_blocks = num_blocks;
        let threshold = self.n / 2 + 1; // majority threshold

        for block_idx in 0..num_blocks {
            let block = &bits[block_idx * self.n..(block_idx + 1) * self.n];
            let ones: usize = block.iter().filter(|&&b| b).count();
            let decided_bit = ones >= threshold;

            // Count how many bits in the block disagreed with the decision.
            let disagreements = if decided_bit {
                self.n - ones
            } else {
                ones
            };
            stats.corrected_errors += disagreements;
            decoded.push(decided_bit);
        }

        (decoded, stats)
    }

    /// Decode soft-decision samples using LLR summation.
    ///
    /// Each input value is a log-likelihood ratio where positive values favour
    /// bit = 1 and negative values favour bit = 0. The decoder sums the N LLRs
    /// in each codeword block and decides based on the sign of the sum.
    ///
    /// The input length must be a multiple of N.
    ///
    /// Returns the decoded bits and statistics. The `corrected_errors` field
    /// counts the number of individual LLR samples whose sign disagreed with the
    /// block decision.
    pub fn decode_soft(&self, llrs: &[f64]) -> (Vec<bool>, DecodingStats) {
        let num_blocks = llrs.len() / self.n;
        let mut decoded = Vec::with_capacity(num_blocks);
        let mut stats = DecodingStats::new();
        stats.total_blocks = num_blocks;

        for block_idx in 0..num_blocks {
            let block = &llrs[block_idx * self.n..(block_idx + 1) * self.n];
            let sum: f64 = block.iter().sum();
            let decided_bit = sum >= 0.0;

            // Count samples that disagreed with the decision.
            let disagreements = block
                .iter()
                .filter(|&&v| (v >= 0.0) != decided_bit)
                .count();
            stats.corrected_errors += disagreements;
            decoded.push(decided_bit);
        }

        (decoded, stats)
    }

    // -- Byte-level interface -------------------------------------------------

    /// Encode a byte slice. Each byte is unpacked into 8 bits (MSB first),
    /// each bit is repeated N times, and the result is returned as an encoded
    /// bit vector.
    pub fn encode_bytes(&self, data: &[u8]) -> Vec<bool> {
        let bits = bytes_to_bits(data);
        self.encode(&bits)
    }

    /// Decode hard-decision bits back into bytes.
    ///
    /// The decoded bit count must be a multiple of 8. Any trailing bits that
    /// do not form a complete byte are discarded.
    pub fn decode_bytes_hard(&self, encoded: &[bool]) -> (Vec<u8>, DecodingStats) {
        let (bits, stats) = self.decode_hard(encoded);
        (bits_to_bytes(&bits), stats)
    }

    /// Decode soft-decision LLRs back into bytes.
    pub fn decode_bytes_soft(&self, llrs: &[f64]) -> (Vec<u8>, DecodingStats) {
        let (bits, stats) = self.decode_soft(llrs);
        (bits_to_bytes(&bits), stats)
    }

    /// Validate that an encoded length is compatible with this code.
    pub fn validate_encoded_length(&self, len: usize) -> Result<(), RepetitionCodeError> {
        if len % self.n != 0 {
            Err(RepetitionCodeError::InvalidEncodedLength {
                length: len,
                n: self.n,
            })
        } else {
            Ok(())
        }
    }
}

// -- Bit / byte helpers -------------------------------------------------------

/// Unpack bytes into a bit vector (MSB first).
pub fn bytes_to_bits(data: &[u8]) -> Vec<bool> {
    let mut bits = Vec::with_capacity(data.len() * 8);
    for &byte in data {
        for i in (0..8).rev() {
            bits.push((byte >> i) & 1 == 1);
        }
    }
    bits
}

/// Pack a bit vector into bytes (MSB first). Trailing bits that don't fill a
/// complete byte are discarded.
pub fn bits_to_bytes(bits: &[bool]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(bits.len() / 8);
    for chunk in bits.chunks(8) {
        if chunk.len() < 8 {
            break;
        }
        let mut byte = 0u8;
        for (i, &bit) in chunk.iter().enumerate() {
            if bit {
                byte |= 1 << (7 - i);
            }
        }
        bytes.push(byte);
    }
    bytes
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_valid_odd_factors() {
        for n in [1, 3, 5, 7, 9, 11] {
            let code = RepetitionCode::new(n);
            assert!(code.is_ok(), "n={n} should be valid");
            assert_eq!(code.unwrap().repeat_factor(), n);
        }
    }

    #[test]
    fn test_new_rejects_even_and_zero() {
        for n in [0, 2, 4, 6, 8] {
            let code = RepetitionCode::new(n);
            assert!(
                matches!(code, Err(RepetitionCodeError::InvalidRepeatFactor(_))),
                "n={n} should be rejected"
            );
        }
    }

    #[test]
    fn test_code_rate() {
        let code = RepetitionCode::new(5).unwrap();
        assert!((code.code_rate() - 0.2).abs() < 1e-12);
        let code3 = RepetitionCode::new(3).unwrap();
        assert!((code3.code_rate() - 1.0 / 3.0).abs() < 1e-12);
    }

    #[test]
    fn test_error_correction_capability() {
        assert_eq!(RepetitionCode::new(1).unwrap().error_correction_capability(), 0);
        assert_eq!(RepetitionCode::new(3).unwrap().error_correction_capability(), 1);
        assert_eq!(RepetitionCode::new(5).unwrap().error_correction_capability(), 2);
        assert_eq!(RepetitionCode::new(7).unwrap().error_correction_capability(), 3);
    }

    #[test]
    fn test_minimum_distance() {
        let code = RepetitionCode::new(7).unwrap();
        assert_eq!(code.minimum_distance(), 7);
    }

    #[test]
    fn test_encode_basic() {
        let code = RepetitionCode::new(3).unwrap();
        let encoded = code.encode(&[true, false]);
        assert_eq!(encoded, vec![true, true, true, false, false, false]);
    }

    #[test]
    fn test_encode_n1_is_identity() {
        let code = RepetitionCode::new(1).unwrap();
        let data = vec![true, false, true, true, false];
        let encoded = code.encode(&data);
        assert_eq!(encoded, data);
    }

    #[test]
    fn test_decode_hard_no_errors() {
        let code = RepetitionCode::new(5).unwrap();
        let data = vec![true, false, true, false];
        let encoded = code.encode(&data);
        let (decoded, stats) = code.decode_hard(&encoded);
        assert_eq!(decoded, data);
        assert_eq!(stats.corrected_errors, 0);
        assert_eq!(stats.total_blocks, 4);
    }

    #[test]
    fn test_decode_hard_corrects_within_capability() {
        let code = RepetitionCode::new(5).unwrap();
        // Can correct up to 2 errors per codeword.
        let data = vec![true, false];
        let mut encoded = code.encode(&data);
        // Introduce 2 errors in first codeword (flip positions 0 and 1)
        encoded[0] = false;
        encoded[1] = false;
        // Introduce 1 error in second codeword
        encoded[5] = true;

        let (decoded, stats) = code.decode_hard(&encoded);
        assert_eq!(decoded, data);
        assert_eq!(stats.corrected_errors, 3);
    }

    #[test]
    fn test_decode_hard_fails_beyond_capability() {
        let code = RepetitionCode::new(3).unwrap();
        // Can correct 1 error. With 2 errors the majority flips.
        let data = vec![true];
        let mut encoded = code.encode(&data); // [true, true, true]
        encoded[0] = false;
        encoded[1] = false;
        // Now [false, false, true] - majority is false, decoded wrong.
        let (decoded, _stats) = code.decode_hard(&encoded);
        assert_eq!(decoded, vec![false]); // Incorrect decode - expected.
    }

    #[test]
    fn test_decode_soft_basic() {
        let code = RepetitionCode::new(3).unwrap();
        // Positive LLR = bit 1, negative = bit 0.
        // Encode bits [1, 0]: LLRs [+, +, +, -, -, -]
        let llrs = vec![1.0, 0.5, 1.2, -0.8, -1.5, -0.3];
        let (decoded, stats) = code.decode_soft(&llrs);
        assert_eq!(decoded, vec![true, false]);
        assert_eq!(stats.corrected_errors, 0);
    }

    #[test]
    fn test_decode_soft_corrects_one_flip() {
        let code = RepetitionCode::new(3).unwrap();
        // Original bit = 1, LLRs should be positive. One is flipped negative
        // but the sum is still positive.
        let llrs = vec![2.0, -0.5, 1.0]; // sum = 2.5 > 0, decides bit 1
        let (decoded, stats) = code.decode_soft(&llrs);
        assert_eq!(decoded, vec![true]);
        assert_eq!(stats.corrected_errors, 1); // one LLR disagreed
    }

    #[test]
    fn test_encode_decode_bytes_roundtrip() {
        let code = RepetitionCode::new(3).unwrap();
        let data = b"Hi";
        let encoded = code.encode_bytes(data);
        assert_eq!(encoded.len(), 2 * 8 * 3); // 2 bytes x 8 bits x 3

        let (decoded_bytes, stats) = code.decode_bytes_hard(&encoded);
        assert_eq!(decoded_bytes, data.to_vec());
        assert_eq!(stats.corrected_errors, 0);
    }

    #[test]
    fn test_encode_decode_bytes_with_errors() {
        let code = RepetitionCode::new(5).unwrap();
        let data = b"\xAB";
        let mut encoded = code.encode_bytes(data);

        // Flip up to 2 bits in every codeword (there are 8 codewords for 1 byte).
        // Flip the first two bits of every block.
        for block in 0..8 {
            encoded[block * 5] = !encoded[block * 5];
            encoded[block * 5 + 1] = !encoded[block * 5 + 1];
        }

        let (decoded_bytes, stats) = code.decode_bytes_hard(&encoded);
        assert_eq!(decoded_bytes, b"\xAB".to_vec());
        assert_eq!(stats.corrected_errors, 16); // 2 per block x 8 blocks
    }

    #[test]
    fn test_validate_encoded_length() {
        let code = RepetitionCode::new(5).unwrap();
        assert!(code.validate_encoded_length(0).is_ok());
        assert!(code.validate_encoded_length(10).is_ok());
        assert!(code.validate_encoded_length(7).is_err());
    }

    #[test]
    fn test_bytes_to_bits_roundtrip() {
        let data = vec![0xA5, 0x3C];
        let bits = bytes_to_bits(&data);
        assert_eq!(bits.len(), 16);
        let back = bits_to_bytes(&bits);
        assert_eq!(back, data);
    }

    #[test]
    fn test_error_display() {
        let e = RepetitionCodeError::InvalidRepeatFactor(4);
        assert!(e.to_string().contains("odd"));
        let e2 = RepetitionCodeError::InvalidEncodedLength { length: 7, n: 3 };
        assert!(e2.to_string().contains("7"));
    }
}
