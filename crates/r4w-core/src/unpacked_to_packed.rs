//! # Unpacked to Packed / Packed to Unpacked
//!
//! Converts between unpacked format (one bit per byte, right-justified)
//! and packed format (8 bits per byte). Supports configurable bits per
//! chunk and MSB/LSB first ordering. Standard GNU Radio digital comms format.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::unpacked_to_packed::{pack_bits, unpack_bits, BitOrder};
//!
//! // Pack: [0,0,1,0,1,0,1,0] → [0x2A]  (MSB first)
//! let unpacked = vec![0u8, 0, 1, 0, 1, 0, 1, 0];
//! let packed = pack_bits(&unpacked, 1, BitOrder::MsbFirst);
//! assert_eq!(packed, vec![0x2A]);
//!
//! // Unpack back
//! let restored = unpack_bits(&packed, 1, BitOrder::MsbFirst);
//! assert_eq!(restored, unpacked);
//! ```

/// Bit ordering for packing/unpacking.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BitOrder {
    /// Most significant bit first.
    MsbFirst,
    /// Least significant bit first.
    LsbFirst,
}

/// Pack unpacked bytes (bits_per_chunk bits per byte) into packed bytes.
///
/// Each input byte contains `bits_per_chunk` bits (right-justified).
/// Output bytes are fully packed (8 bits).
pub fn pack_bits(input: &[u8], bits_per_chunk: usize, order: BitOrder) -> Vec<u8> {
    if bits_per_chunk == 0 || input.is_empty() {
        return Vec::new();
    }

    let mask = (1u8 << bits_per_chunk) - 1;
    let total_bits = input.len() * bits_per_chunk;
    let num_bytes = (total_bits + 7) / 8;
    let mut output = vec![0u8; num_bytes];

    let mut bit_pos = 0;
    for &chunk in input {
        let val = chunk & mask;
        for b in 0..bits_per_chunk {
            let bit = match order {
                BitOrder::MsbFirst => (val >> (bits_per_chunk - 1 - b)) & 1,
                BitOrder::LsbFirst => (val >> b) & 1,
            };
            if bit == 1 {
                let byte_idx = bit_pos / 8;
                let bit_idx = match order {
                    BitOrder::MsbFirst => 7 - (bit_pos % 8),
                    BitOrder::LsbFirst => bit_pos % 8,
                };
                output[byte_idx] |= 1 << bit_idx;
            }
            bit_pos += 1;
        }
    }

    output
}

/// Unpack packed bytes into unpacked bytes (bits_per_chunk bits per byte).
///
/// Each output byte contains `bits_per_chunk` bits (right-justified).
pub fn unpack_bits(input: &[u8], bits_per_chunk: usize, order: BitOrder) -> Vec<u8> {
    if bits_per_chunk == 0 || input.is_empty() {
        return Vec::new();
    }

    let total_bits = input.len() * 8;
    let num_chunks = total_bits / bits_per_chunk;
    let mut output = Vec::with_capacity(num_chunks);

    let mut bit_pos = 0;
    for _ in 0..num_chunks {
        let mut val = 0u8;
        for b in 0..bits_per_chunk {
            let byte_idx = bit_pos / 8;
            let bit_idx = match order {
                BitOrder::MsbFirst => 7 - (bit_pos % 8),
                BitOrder::LsbFirst => bit_pos % 8,
            };
            let bit = (input[byte_idx] >> bit_idx) & 1;
            match order {
                BitOrder::MsbFirst => {
                    val |= bit << (bits_per_chunk - 1 - b);
                }
                BitOrder::LsbFirst => {
                    val |= bit << b;
                }
            }
            bit_pos += 1;
        }
        output.push(val);
    }

    output
}

/// Stateful packer that handles partial bytes across calls.
#[derive(Debug, Clone)]
pub struct PackedToUnpacked {
    bits_per_chunk: usize,
    order: BitOrder,
    /// Leftover bits from previous call.
    remainder: Vec<u8>,
}

impl PackedToUnpacked {
    /// Create a new unpacker.
    pub fn new(bits_per_chunk: usize, order: BitOrder) -> Self {
        Self {
            bits_per_chunk,
            order,
            remainder: Vec::new(),
        }
    }

    /// Unpack a block of packed bytes.
    pub fn process(&mut self, input: &[u8]) -> Vec<u8> {
        let mut all_bytes = self.remainder.clone();
        all_bytes.extend_from_slice(input);
        self.remainder.clear();

        let total_bits = all_bytes.len() * 8;
        let num_chunks = total_bits / self.bits_per_chunk;
        let used_bits = num_chunks * self.bits_per_chunk;
        let leftover_bits = total_bits - used_bits;

        let result = unpack_bits(&all_bytes, self.bits_per_chunk, self.order);

        // Save leftover bytes (those containing unused bits).
        if leftover_bits > 0 {
            let leftover_bytes = (leftover_bits + 7) / 8;
            let start = all_bytes.len() - leftover_bytes;
            self.remainder = all_bytes[start..].to_vec();
        }

        result
    }

    /// Get bits per chunk.
    pub fn bits_per_chunk(&self) -> usize {
        self.bits_per_chunk
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.remainder.clear();
    }
}

/// Stateful unpacked-to-packed converter.
#[derive(Debug, Clone)]
pub struct UnpackedToPacked {
    bits_per_chunk: usize,
    order: BitOrder,
    /// Leftover unpacked chunks.
    remainder: Vec<u8>,
}

impl UnpackedToPacked {
    /// Create a new packer.
    pub fn new(bits_per_chunk: usize, order: BitOrder) -> Self {
        Self {
            bits_per_chunk,
            order,
            remainder: Vec::new(),
        }
    }

    /// Pack a block of unpacked bytes.
    pub fn process(&mut self, input: &[u8]) -> Vec<u8> {
        let mut all_chunks = self.remainder.clone();
        all_chunks.extend_from_slice(input);
        self.remainder.clear();

        let total_bits = all_chunks.len() * self.bits_per_chunk;
        let num_bytes = total_bits / 8;
        let used_chunks = (num_bytes * 8) / self.bits_per_chunk;

        let result = pack_bits(&all_chunks[..used_chunks], self.bits_per_chunk, self.order);

        if used_chunks < all_chunks.len() {
            self.remainder = all_chunks[used_chunks..].to_vec();
        }

        result
    }

    /// Get bits per chunk.
    pub fn bits_per_chunk(&self) -> usize {
        self.bits_per_chunk
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.remainder.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pack_1bit_msb() {
        let input = vec![0u8, 0, 1, 0, 1, 0, 1, 0]; // 0x2A = 42
        let packed = pack_bits(&input, 1, BitOrder::MsbFirst);
        assert_eq!(packed, vec![0x2A]);
    }

    #[test]
    fn test_unpack_1bit_msb() {
        let packed = vec![0xABu8]; // 10101011
        let unpacked = unpack_bits(&packed, 1, BitOrder::MsbFirst);
        assert_eq!(unpacked, vec![1, 0, 1, 0, 1, 0, 1, 1]);
    }

    #[test]
    fn test_roundtrip_1bit() {
        let original = vec![1u8, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1];
        let packed = pack_bits(&original, 1, BitOrder::MsbFirst);
        let unpacked = unpack_bits(&packed, 1, BitOrder::MsbFirst);
        assert_eq!(unpacked, original);
    }

    #[test]
    fn test_pack_2bit_msb() {
        // 4 chunks of 2 bits: 01 10 11 00 → 0x6C
        let input = vec![1u8, 2, 3, 0];
        let packed = pack_bits(&input, 2, BitOrder::MsbFirst);
        assert_eq!(packed, vec![0x6C]); // 01_10_11_00
    }

    #[test]
    fn test_roundtrip_2bit() {
        let original = vec![0u8, 1, 2, 3, 3, 2, 1, 0];
        let packed = pack_bits(&original, 2, BitOrder::MsbFirst);
        let unpacked = unpack_bits(&packed, 2, BitOrder::MsbFirst);
        assert_eq!(unpacked, original);
    }

    #[test]
    fn test_lsb_first() {
        let input = vec![1u8, 0, 1, 0, 1, 0, 1, 1]; // LSB first
        let packed = pack_bits(&input, 1, BitOrder::LsbFirst);
        let unpacked = unpack_bits(&packed, 1, BitOrder::LsbFirst);
        assert_eq!(unpacked, input);
    }

    #[test]
    fn test_empty() {
        assert!(pack_bits(&[], 1, BitOrder::MsbFirst).is_empty());
        assert!(unpack_bits(&[], 1, BitOrder::MsbFirst).is_empty());
    }

    #[test]
    fn test_stateful_packer() {
        let mut packer = UnpackedToPacked::new(1, BitOrder::MsbFirst);
        // Feed 4 bits at a time (need 8 for a full byte).
        let out1 = packer.process(&[1, 0, 1, 0]);
        assert!(out1.is_empty()); // Not enough for a full byte.
        let out2 = packer.process(&[1, 0, 1, 1]);
        assert_eq!(out2, vec![0xAB]); // 10101011
    }

    #[test]
    fn test_stateful_unpacker() {
        let mut unpacker = PackedToUnpacked::new(1, BitOrder::MsbFirst);
        let out = unpacker.process(&[0xFF]);
        assert_eq!(out, vec![1, 1, 1, 1, 1, 1, 1, 1]);
    }

    #[test]
    fn test_roundtrip_4bit() {
        let original = vec![0x0Au8, 0x05]; // 4-bit chunks: 10, 5
        let packed = pack_bits(&original, 4, BitOrder::MsbFirst);
        let unpacked = unpack_bits(&packed, 4, BitOrder::MsbFirst);
        assert_eq!(unpacked, original);
    }
}
