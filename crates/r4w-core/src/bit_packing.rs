//! Bit Packing and Unpacking
//!
//! Convert between different bit-width representations:
//!
//! - `PackKBits`: Pack K single-bit bytes (LSB) into bytes with K bits each
//! - `UnpackKBits`: Unpack bytes with K bits each into K single-bit bytes
//! - `RepackBits`: Convert between different bits-per-byte representations
//!
//! These are essential "glue" blocks between FEC encoders/decoders and
//! modulators/demodulators that expect different bit packing.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::bit_packing::{PackKBits, UnpackKBits};
//!
//! // Pack 8 individual bits into one byte
//! let bits = vec![1u8, 0, 1, 1, 0, 0, 1, 0]; // LSB first
//! let packed = PackKBits::new(8).pack(&bits);
//! assert_eq!(packed, vec![0b01001101]); // MSB-first packing
//!
//! // Unpack back
//! let unpacked = UnpackKBits::new(8).unpack(&packed);
//! assert_eq!(unpacked, bits);
//! ```

/// Pack K individual bits (one bit per byte, in LSB) into packed bytes.
///
/// Input: stream of bytes where only bit 0 matters (0 or 1).
/// Output: stream of bytes with K bits packed per byte (MSB first).
#[derive(Debug, Clone)]
pub struct PackKBits {
    k: usize,
}

impl PackKBits {
    /// Create a packer that groups K bits into each output byte.
    pub fn new(k: usize) -> Self {
        assert!(k > 0 && k <= 8, "K must be 1..=8");
        Self { k }
    }

    /// Pack input bits into bytes.
    /// Input length should be a multiple of K; any remainder is zero-padded.
    pub fn pack(&self, input: &[u8]) -> Vec<u8> {
        let mut output = Vec::with_capacity((input.len() + self.k - 1) / self.k);
        for chunk in input.chunks(self.k) {
            let mut byte = 0u8;
            for (i, &bit) in chunk.iter().enumerate() {
                // MSB-first packing
                byte |= (bit & 1) << (self.k - 1 - i);
            }
            output.push(byte);
        }
        output
    }

    /// Get K.
    pub fn k(&self) -> usize {
        self.k
    }
}

/// Unpack bytes with K bits each into individual bits (one per byte).
///
/// Input: stream of bytes with K valid bits each.
/// Output: stream of bytes where only bit 0 matters (0 or 1), MSB first.
#[derive(Debug, Clone)]
pub struct UnpackKBits {
    k: usize,
}

impl UnpackKBits {
    /// Create an unpacker that extracts K bits from each input byte.
    pub fn new(k: usize) -> Self {
        assert!(k > 0 && k <= 8, "K must be 1..=8");
        Self { k }
    }

    /// Unpack bytes into individual bits.
    pub fn unpack(&self, input: &[u8]) -> Vec<u8> {
        let mut output = Vec::with_capacity(input.len() * self.k);
        for &byte in input {
            for i in 0..self.k {
                // MSB-first unpacking
                output.push((byte >> (self.k - 1 - i)) & 1);
            }
        }
        output
    }

    /// Get K.
    pub fn k(&self) -> usize {
        self.k
    }
}

/// Repack bits from one bits-per-byte representation to another.
///
/// Converts between different bit-width representations (e.g., 1 bit/byte
/// to 4 bits/byte for QAM mapping, or 8 bits/byte to 2 bits/byte for QPSK).
#[derive(Debug, Clone)]
pub struct RepackBits {
    bits_in: usize,
    bits_out: usize,
}

impl RepackBits {
    /// Create a repacker from `bits_in` per input byte to `bits_out` per output byte.
    pub fn new(bits_in: usize, bits_out: usize) -> Self {
        assert!(bits_in > 0 && bits_in <= 8, "bits_in must be 1..=8");
        assert!(bits_out > 0 && bits_out <= 8, "bits_out must be 1..=8");
        Self { bits_in, bits_out }
    }

    /// Repack a stream of bytes.
    pub fn repack(&self, input: &[u8]) -> Vec<u8> {
        // First unpack to individual bits
        let mut bits = Vec::with_capacity(input.len() * self.bits_in);
        for &byte in input {
            for i in 0..self.bits_in {
                bits.push((byte >> (self.bits_in - 1 - i)) & 1);
            }
        }
        // Then pack into output representation
        let mut output = Vec::with_capacity((bits.len() + self.bits_out - 1) / self.bits_out);
        for chunk in bits.chunks(self.bits_out) {
            let mut byte = 0u8;
            for (i, &bit) in chunk.iter().enumerate() {
                byte |= bit << (self.bits_out - 1 - i);
            }
            output.push(byte);
        }
        output
    }

    /// Get bits per input byte.
    pub fn bits_in(&self) -> usize {
        self.bits_in
    }

    /// Get bits per output byte.
    pub fn bits_out(&self) -> usize {
        self.bits_out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pack_8_bits() {
        let bits = vec![1u8, 0, 1, 1, 0, 0, 1, 0];
        let packed = PackKBits::new(8).pack(&bits);
        assert_eq!(packed.len(), 1);
        assert_eq!(packed[0], 0b10110010);
    }

    #[test]
    fn test_unpack_8_bits() {
        let packed = vec![0b10110010u8];
        let bits = UnpackKBits::new(8).unpack(&packed);
        assert_eq!(bits, vec![1u8, 0, 1, 1, 0, 0, 1, 0]);
    }

    #[test]
    fn test_pack_unpack_roundtrip() {
        let original = vec![1u8, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 1];
        let packed = PackKBits::new(8).pack(&original);
        let unpacked = UnpackKBits::new(8).unpack(&packed);
        assert_eq!(unpacked, original);
    }

    #[test]
    fn test_pack_2_bits() {
        // Pack pairs of bits
        let bits = vec![1u8, 0, 1, 1, 0, 0, 0, 1];
        let packed = PackKBits::new(2).pack(&bits);
        assert_eq!(packed.len(), 4);
        assert_eq!(packed[0], 0b10); // 1, 0
        assert_eq!(packed[1], 0b11); // 1, 1
        assert_eq!(packed[2], 0b00); // 0, 0
        assert_eq!(packed[3], 0b01); // 0, 1
    }

    #[test]
    fn test_unpack_2_bits() {
        let packed = vec![0b10u8, 0b11, 0b00, 0b01];
        let bits = UnpackKBits::new(2).unpack(&packed);
        assert_eq!(bits, vec![1u8, 0, 1, 1, 0, 0, 0, 1]);
    }

    #[test]
    fn test_pack_4_bits() {
        // Pack nibbles
        let bits = vec![1u8, 0, 1, 0, 1, 1, 0, 0];
        let packed = PackKBits::new(4).pack(&bits);
        assert_eq!(packed.len(), 2);
        assert_eq!(packed[0], 0b1010);
        assert_eq!(packed[1], 0b1100);
    }

    #[test]
    fn test_pack_1_bit() {
        let bits = vec![1u8, 0, 1];
        let packed = PackKBits::new(1).pack(&bits);
        assert_eq!(packed, vec![1, 0, 1]); // Same as input
    }

    #[test]
    fn test_repack_1_to_4() {
        // 8 individual bits → 2 nibbles
        let input = vec![1u8, 0, 1, 0, 1, 1, 0, 0];
        let output = RepackBits::new(1, 4).repack(&input);
        assert_eq!(output.len(), 2);
        assert_eq!(output[0], 0b1010);
        assert_eq!(output[1], 0b1100);
    }

    #[test]
    fn test_repack_4_to_1() {
        let input = vec![0b1010u8, 0b1100];
        let output = RepackBits::new(4, 1).repack(&input);
        assert_eq!(output, vec![1u8, 0, 1, 0, 1, 1, 0, 0]);
    }

    #[test]
    fn test_repack_2_to_4() {
        // 4 pairs → 2 nibbles
        let input = vec![0b10u8, 0b11, 0b00, 0b01];
        let output = RepackBits::new(2, 4).repack(&input);
        assert_eq!(output.len(), 2);
        assert_eq!(output[0], 0b1011);
        assert_eq!(output[1], 0b0001);
    }

    #[test]
    fn test_repack_identity() {
        let input = vec![0b1010u8, 0b0101];
        let output = RepackBits::new(4, 4).repack(&input);
        assert_eq!(output, input);
    }

    #[test]
    fn test_pack_padding() {
        // 5 bits with k=4 → 2 bytes (second is zero-padded)
        let bits = vec![1u8, 0, 1, 1, 1];
        let packed = PackKBits::new(4).pack(&bits);
        assert_eq!(packed.len(), 2);
        assert_eq!(packed[0], 0b1011);
        assert_eq!(packed[1], 0b1000); // zero-padded
    }
}
