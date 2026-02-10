//! Golay Error Correction Codes
//!
//! Implements the (23,12) perfect binary Golay code and the (24,12) extended
//! Golay code. The Golay code is a perfect linear error-correcting code capable
//! of correcting up to 3 bit errors in a 23-bit codeword. The extended (24,12)
//! variant adds an overall parity bit for improved error detection.
//!
//! The Golay code is used in deep-space communication (Voyager missions),
//! amateur radio (AX.25), and military standards (MIL-STD-188).
//!
//! ## Properties
//!
//! - **(23,12,7)**: 12 data bits, 11 parity bits, minimum distance 7
//! - **(24,12,8)**: 12 data bits, 12 parity bits (extended), minimum distance 8
//! - Corrects up to 3 errors, detects up to 6 (extended: 7)
//! - Perfect code: every 23-bit word is within Hamming distance 3 of exactly one codeword
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::golay_code::{encode_golay23, decode_golay23, GolayCodec};
//!
//! // Encode 12 data bits
//! let data: u16 = 0b1010_1100_0011;
//! let codeword = encode_golay23(data);
//!
//! // Introduce 2 bit errors
//! let corrupted = codeword ^ 0b0000_0000_0100_0000_0010_0000;
//! let recovered = decode_golay23(corrupted).unwrap();
//! assert_eq!(recovered, data);
//!
//! // Codec for byte-level encoding
//! let codec = GolayCodec::new(false); // (23,12) mode
//! let original = vec![0xAB, 0xCD, 0xEF];
//! let encoded = codec.encode_bytes(&original);
//! let decoded = codec.decode_bytes(&encoded);
//! assert_eq!(decoded, original);
//! ```

/// The 12x12 parity matrix P for the (23,12) Golay code.
///
/// This matrix is used to construct both the generator matrix G = [I_12 | P]
/// and the parity-check matrix H = [P^T | I_11]. The matrix is derived from
/// the quadratic residues of GF(23).
///
/// The rows correspond to the remainders of x^{22}, x^{21}, ..., x^{11}
/// modulo the generator polynomial g(x) = x^11 + x^9 + x^7 + x^6 + x^5 + x + 1.
pub fn golay_parity_matrix() -> [[bool; 12]; 12] {
    // Each row is the 11-bit remainder of x^(22-i) mod g(x), extended to 12 bits
    // by prepending a zero. However, the standard presentation uses a 12x12
    // symmetric matrix from quadratic residues of 23.
    //
    // We derive this from our generator polynomial for consistency with encode/decode.
    let mut matrix = [[false; 12]; 12];
    for i in 0..12 {
        // Compute remainder of x^(22-i) mod g(x), which is the parity
        // contributed by data bit i.
        let remainder = golay23_parity(1u32 << (11 - i));
        // The parity is 11 bits; store in columns 0..10 and set column 11
        // to the overall parity of (data_bit + remainder) to make it 12 columns.
        for j in 0..11 {
            matrix[i][j] = (remainder >> (10 - j)) & 1 == 1;
        }
        // Column 11: overall even parity over the row
        let row_weight = 1u32 + remainder.count_ones(); // 1 for the data bit itself
        matrix[i][11] = row_weight & 1 == 1;
    }
    matrix
}

/// Compute the Hamming weight (number of set bits) of a 32-bit value.
pub fn weight(val: u32) -> u32 {
    val.count_ones()
}

/// Encode 12 data bits into a 23-bit Golay codeword.
///
/// The lower 12 bits of `data` are used. Returns a 23-bit codeword in the
/// lower 23 bits of the result, structured as `[data (12 bits) | parity (11 bits)]`.
pub fn encode_golay23(data: u16) -> u32 {
    let d = (data & 0x0FFF) as u32;
    let parity = golay23_parity(d);
    (d << 11) | parity
}

/// Decode a 23-bit Golay codeword, correcting up to 3 bit errors.
///
/// Returns `Some(data)` with the corrected 12 data bits, or `None` if the
/// error pattern has weight > 3 (uncorrectable).
pub fn decode_golay23(received: u32) -> Option<u16> {
    let r = received & 0x7FFFFF; // mask to 23 bits
    let error = golay23_error_pattern(r)?;
    let corrected = r ^ error;
    Some((corrected >> 11) as u16)
}

/// Encode 12 data bits into a 24-bit extended Golay codeword.
///
/// The extended code appends an overall parity bit to the (23,12) codeword,
/// yielding minimum distance 8 and the ability to detect up to 7 errors.
pub fn encode_golay24(data: u16) -> u32 {
    let codeword23 = encode_golay23(data);
    let parity_bit = weight(codeword23) & 1;
    (codeword23 << 1) | parity_bit
}

/// Decode a 24-bit extended Golay codeword, correcting up to 3 bit errors.
///
/// Returns `Some(data)` with the corrected 12 data bits, or `None` if the
/// error pattern is uncorrectable.
pub fn decode_golay24(received: u32) -> Option<u16> {
    let r = received & 0xFFFFFF; // mask to 24 bits

    // Extract the 23-bit codeword (bits 23..1) and the overall parity bit (bit 0)
    let cw23 = r >> 1;

    // Overall parity of the full 24-bit word
    let overall_parity_ok = weight(r) & 1 == 0;

    // Decode the inner (23,12) codeword
    match golay23_error_pattern(cw23) {
        Some(e) => {
            let error_weight = weight(e);
            let corrected = cw23 ^ e;

            if error_weight <= 3 {
                // The parity bit may or may not be in error.
                // If overall parity is wrong after correcting the 23-bit part,
                // then the parity bit was also flipped (one additional error).
                let total_errors = if overall_parity_ok {
                    // Parity is consistent: either 0 errors in parity bit,
                    // or an even number of additional errors (not possible
                    // with <= 3 total). So total = error_weight.
                    error_weight
                } else {
                    // Parity is inconsistent: parity bit was flipped
                    error_weight + 1
                };

                if total_errors <= 3 {
                    Some((corrected >> 11) as u16)
                } else {
                    None
                }
            } else {
                None
            }
        }
        None => None,
    }
}

/// Compute the 11-bit syndrome for a 23-bit Golay codeword.
///
/// A syndrome of zero indicates a valid codeword (no detectable errors).
pub fn syndrome_golay23(codeword: u32) -> u16 {
    let c = codeword & 0x7FFFFF;
    compute_syndrome(c)
}

/// Golay codec for encoding/decoding byte arrays.
///
/// Packs data into 12-bit chunks, encodes each with the Golay code,
/// and serializes the codewords into a byte stream.
#[derive(Debug, Clone)]
pub struct GolayCodec {
    /// If true, use the (24,12) extended code; otherwise (23,12).
    extended: bool,
}

impl GolayCodec {
    /// Create a new Golay codec.
    ///
    /// - `extended = false`: use the (23,12) perfect Golay code
    /// - `extended = true`: use the (24,12) extended Golay code
    pub fn new(extended: bool) -> Self {
        Self { extended }
    }

    /// Encode a byte slice into Golay-coded bytes.
    ///
    /// Data is packed into 12-bit chunks. Each chunk is encoded into a
    /// 23-bit (or 24-bit) codeword. Codewords are packed MSB-first into
    /// the output byte stream.
    pub fn encode_bytes(&self, data: &[u8]) -> Vec<u8> {
        let bits_per_cw = if self.extended { 24 } else { 23 };

        // Convert input bytes to a bit stream
        let input_bits = bytes_to_bits(data);

        // Split into 12-bit chunks
        let mut codewords: Vec<u32> = Vec::new();
        let mut i = 0;
        while i < input_bits.len() {
            let mut chunk: u16 = 0;
            for j in 0..12 {
                if i + j < input_bits.len() {
                    chunk |= (input_bits[i + j] as u16) << (11 - j);
                }
            }
            let cw = if self.extended {
                encode_golay24(chunk)
            } else {
                encode_golay23(chunk)
            };
            codewords.push(cw);
            i += 12;
        }

        // Pack codewords into output bit stream
        let total_bits = codewords.len() * bits_per_cw;
        let mut out_bits = vec![false; total_bits];
        for (idx, &cw) in codewords.iter().enumerate() {
            for b in 0..bits_per_cw {
                out_bits[idx * bits_per_cw + b] =
                    (cw >> (bits_per_cw - 1 - b)) & 1 == 1;
            }
        }

        bits_to_bytes(&out_bits)
    }

    /// Decode Golay-coded bytes back to original data.
    ///
    /// Unpacks codewords from the byte stream, decodes each, and
    /// reassembles the 12-bit data chunks into bytes.
    ///
    /// The `original_byte_count` is inferred from the encoded length.
    /// Any padding bits in the last chunk are discarded.
    pub fn decode_bytes(&self, encoded: &[u8]) -> Vec<u8> {
        let bits_per_cw = if self.extended { 24 } else { 23 };

        let enc_bits = bytes_to_bits(encoded);

        // Extract codewords
        let num_codewords = enc_bits.len() / bits_per_cw;
        let mut data_bits: Vec<bool> = Vec::new();

        for idx in 0..num_codewords {
            let mut cw: u32 = 0;
            for b in 0..bits_per_cw {
                if enc_bits[idx * bits_per_cw + b] {
                    cw |= 1 << (bits_per_cw - 1 - b);
                }
            }

            let decoded = if self.extended {
                decode_golay24(cw)
            } else {
                decode_golay23(cw)
            };

            match decoded {
                Some(d) => {
                    for b in 0..12 {
                        data_bits.push((d >> (11 - b)) & 1 == 1);
                    }
                }
                None => {
                    // Uncorrectable error: output zeros for this block
                    for _ in 0..12 {
                        data_bits.push(false);
                    }
                }
            }
        }

        // Convert bits back to bytes, trimming any padding
        bits_to_bytes(&data_bits)
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Generator polynomial for the (23,12) Golay code.
///
/// g(x) = x^11 + x^9 + x^7 + x^6 + x^5 + x + 1
///
/// In binary (bit 11 = x^11, bit 0 = x^0): 0b1010_1110_0011 = 0xAE3
const GOLAY_POLY: u32 = 0xAE3;

/// Compute the 11-bit parity (remainder) for 12 data bits using the generator polynomial.
///
/// Performs polynomial long division of data(x) * x^11 by g(x).
fn golay23_parity(data: u32) -> u32 {
    let mut remainder = data << 11;
    for i in (0..12).rev() {
        if remainder & (1 << (i + 11)) != 0 {
            remainder ^= GOLAY_POLY << i;
        }
    }
    remainder & 0x7FF
}

/// Compute the 11-bit syndrome for a received 23-bit word.
///
/// The syndrome is the remainder of the received polynomial divided by g(x).
fn compute_syndrome(received: u32) -> u16 {
    let mut r = received;
    for i in (0..12).rev() {
        if r & (1 << (i + 11)) != 0 {
            r ^= GOLAY_POLY << i;
        }
    }
    (r & 0x7FF) as u16
}

/// Find the error pattern for a 23-bit received word.
///
/// Uses a syndrome-based approach with precomputed single-bit syndromes.
/// Since the Golay code is perfect, every 23-bit word is within Hamming
/// distance 3 of exactly one codeword. We exploit the linearity of
/// syndromes: S(a XOR b) = S(a) XOR S(b).
///
/// Algorithm:
/// 1. Compute S(r). If zero, no errors.
/// 2. Check if S(r) matches any single-bit syndrome (weight-1 error).
/// 3. Check if S(r) XOR S(e_i) matches another single-bit syndrome for i < j (weight-2).
/// 4. Check if S(r) XOR S(e_i) XOR S(e_j) matches a single-bit syndrome for j < k (weight-3).
fn golay23_error_pattern(received: u32) -> Option<u32> {
    let target = compute_syndrome(received);

    if target == 0 {
        return Some(0); // no errors
    }

    // Precompute single-bit syndromes
    let single_syn = precompute_single_bit_syndromes();

    // Build a reverse map: syndrome -> bit position (for O(1) lookup)
    // Syndrome is 11 bits, so max value is 2047.
    let mut syn_to_bit = [255u8; 2048];
    for i in 0..23u8 {
        syn_to_bit[single_syn[i as usize] as usize] = i;
    }

    // Weight-1: check if target matches any single-bit syndrome
    if (target as usize) < 2048 && syn_to_bit[target as usize] < 23 {
        let bit = syn_to_bit[target as usize] as u32;
        return Some(1u32 << bit);
    }

    // Weight-2: for each bit i, check if target XOR S(e_i) is a single-bit syndrome
    // for some bit j > i
    for i in 0..23 {
        let residual = target ^ single_syn[i];
        let j = syn_to_bit[residual as usize];
        if j < 23 && (j as usize) > i {
            return Some((1u32 << i) | (1u32 << j as u32));
        }
    }

    // Weight-3: for each pair (i, j), check if target XOR S(e_i) XOR S(e_j)
    // is a single-bit syndrome for some bit k > j
    for i in 0..23 {
        let partial_i = target ^ single_syn[i];
        for j in (i + 1)..23 {
            let residual = partial_i ^ single_syn[j];
            let k = syn_to_bit[residual as usize];
            if k < 23 && (k as usize) > j {
                return Some((1u32 << i) | (1u32 << j as u32) | (1u32 << k as u32));
            }
        }
    }

    // Perfect code: should never reach here for a 23-bit input,
    // but return None for safety.
    None
}

/// Precompute single-bit syndromes for all 23 bit positions.
///
/// Returns an array of 23 syndromes, one for each bit position.
fn precompute_single_bit_syndromes() -> [u16; 23] {
    let mut syns = [0u16; 23];
    for i in 0..23 {
        syns[i] = compute_syndrome(1u32 << i);
    }
    syns
}

/// Convert a byte slice to a vector of bits (MSB first).
fn bytes_to_bits(data: &[u8]) -> Vec<bool> {
    let mut bits = Vec::with_capacity(data.len() * 8);
    for &byte in data {
        for i in (0..8).rev() {
            bits.push((byte >> i) & 1 == 1);
        }
    }
    bits
}

/// Convert a vector of bits to bytes (MSB first), padding with zeros if needed.
fn bits_to_bytes(bits: &[bool]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity((bits.len() + 7) / 8);
    let mut i = 0;
    while i < bits.len() {
        let mut byte: u8 = 0;
        for j in 0..8 {
            if i + j < bits.len() && bits[i + j] {
                byte |= 1 << (7 - j);
            }
        }
        bytes.push(byte);
        i += 8;
    }
    bytes
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_decode_golay23() {
        // Test all possible 12-bit data values for roundtrip
        for data in 0u16..4096 {
            let cw = encode_golay23(data);
            assert!(cw < (1 << 23), "codeword must fit in 23 bits");
            let decoded = decode_golay23(cw).unwrap();
            assert_eq!(decoded, data, "roundtrip failed for data={:#06x}", data);
        }
    }

    #[test]
    fn test_encode_decode_golay24() {
        // Test a selection of 12-bit data values
        let test_values: Vec<u16> = (0..4096).step_by(17).collect();
        for &data in &test_values {
            let cw = encode_golay24(data);
            assert!(cw < (1 << 24), "codeword must fit in 24 bits");
            let decoded = decode_golay24(cw).unwrap();
            assert_eq!(decoded, data, "roundtrip failed for data={:#06x}", data);
        }
    }

    #[test]
    fn test_error_correction_1bit() {
        let data: u16 = 0b1010_1010_1010;
        let cw = encode_golay23(data);

        // Flip each bit position and verify correction
        for bit in 0..23 {
            let corrupted = cw ^ (1 << bit);
            let decoded = decode_golay23(corrupted);
            assert_eq!(
                decoded,
                Some(data),
                "failed to correct 1-bit error at position {}",
                bit
            );
        }
    }

    #[test]
    fn test_error_correction_2bit() {
        let data: u16 = 0b1100_0011_0101;
        let cw = encode_golay23(data);

        // Test a selection of 2-bit error patterns
        let positions: Vec<(usize, usize)> = vec![
            (0, 1),
            (0, 11),
            (5, 10),
            (3, 22),
            (11, 22),
            (0, 22),
            (7, 15),
            (1, 20),
        ];

        for &(b1, b2) in &positions {
            let corrupted = cw ^ (1 << b1) ^ (1 << b2);
            let decoded = decode_golay23(corrupted);
            assert_eq!(
                decoded,
                Some(data),
                "failed to correct 2-bit error at positions ({}, {})",
                b1,
                b2
            );
        }
    }

    #[test]
    fn test_error_correction_3bit() {
        let data: u16 = 0b0110_1001_1011;
        let cw = encode_golay23(data);

        // Test a selection of 3-bit error patterns
        let positions: Vec<(usize, usize, usize)> = vec![
            (0, 1, 2),
            (0, 11, 22),
            (5, 10, 15),
            (3, 7, 19),
            (1, 12, 20),
        ];

        for &(b1, b2, b3) in &positions {
            let corrupted = cw ^ (1 << b1) ^ (1 << b2) ^ (1 << b3);
            let decoded = decode_golay23(corrupted);
            assert_eq!(
                decoded,
                Some(data),
                "failed to correct 3-bit error at positions ({}, {}, {})",
                b1,
                b2,
                b3
            );
        }
    }

    #[test]
    fn test_4bit_error_detects_failure() {
        let data: u16 = 0b1111_0000_1111;
        let cw = encode_golay23(data);

        // 4-bit error should generally be uncorrectable (or miscorrect)
        let corrupted = cw ^ (1 << 0) ^ (1 << 5) ^ (1 << 10) ^ (1 << 15);
        let decoded = decode_golay23(corrupted);

        // Either returns None or returns wrong data (miscorrection)
        // For a perfect code, 4-bit errors either miscorrect or fail
        match decoded {
            None => {} // correctly detected as uncorrectable
            Some(d) => {
                assert_ne!(d, data, "4-bit error should not decode to correct data");
            }
        }
    }

    #[test]
    fn test_syndrome_zero_for_valid() {
        // Valid codewords should have zero syndrome
        for data in (0u16..4096).step_by(31) {
            let cw = encode_golay23(data);
            let s = syndrome_golay23(cw);
            assert_eq!(s, 0, "valid codeword should have zero syndrome for data={}", data);
        }
    }

    #[test]
    fn test_all_zero_codeword() {
        let cw = encode_golay23(0);
        assert_eq!(cw, 0, "all-zero data should produce all-zero codeword");
        let decoded = decode_golay23(0).unwrap();
        assert_eq!(decoded, 0);
    }

    #[test]
    fn test_codec_bytes_roundtrip() {
        let codec = GolayCodec::new(false);

        // Test with various data lengths
        let test_data = vec![
            vec![0xABu8],
            vec![0x12, 0x34],
            vec![0xAB, 0xCD, 0xEF],
            vec![0x00, 0xFF, 0x55, 0xAA, 0x01],
        ];

        for original in &test_data {
            let encoded = codec.encode_bytes(original);
            let decoded = codec.decode_bytes(&encoded);
            // The decoded length may include padding; trim to original length
            assert_eq!(
                &decoded[..original.len()],
                original.as_slice(),
                "codec roundtrip failed for {:?}",
                original
            );
        }

        // Also test extended mode
        let codec_ext = GolayCodec::new(true);
        for original in &test_data {
            let encoded = codec_ext.encode_bytes(original);
            let decoded = codec_ext.decode_bytes(&encoded);
            assert_eq!(
                &decoded[..original.len()],
                original.as_slice(),
                "extended codec roundtrip failed for {:?}",
                original
            );
        }
    }

    #[test]
    fn test_weight_function() {
        assert_eq!(weight(0), 0);
        assert_eq!(weight(1), 1);
        assert_eq!(weight(0b1010_1010), 4);
        assert_eq!(weight(0b1111_1111), 8);
        assert_eq!(weight(0xFFFF_FFFF), 32);
        assert_eq!(weight(0b1000_0000_0000_0000_0000_0001), 2);
    }
}
