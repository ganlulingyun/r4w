//! # Stream Byte Converter
//!
//! Bit-level packing, unpacking, and stream conversion utilities for SDR data paths.
//!
//! This module provides [`BitPacker`] and [`BitUnpacker`] for converting between
//! boolean bit sequences and packed byte representations, a [`StreamConverter`]
//! for repacking streams between different symbol widths, and convenience functions
//! for Gray coding, bit reversal, and nibble swapping.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::stream_byte_converter::{BitPacker, BitUnpacker, BitOrder};
//!
//! let packer = BitPacker::new(BitOrder::MsbFirst);
//! let bits = vec![true, false, true, false, false, false, false, true];
//! let packed = packer.pack(&bits);
//! assert_eq!(packed, vec![0xA1]);
//!
//! let unpacker = BitUnpacker::new(BitOrder::MsbFirst);
//! let recovered = unpacker.unpack(&packed);
//! assert_eq!(recovered, bits);
//! ```

/// Bit ordering within a byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BitOrder {
    /// Most significant bit first (bit 7 is the first bit).
    MsbFirst,
    /// Least significant bit first (bit 0 is the first bit).
    LsbFirst,
}

/// Packs boolean bit sequences into bytes or N-bit symbols.
#[derive(Debug, Clone)]
pub struct BitPacker {
    order: BitOrder,
}

impl BitPacker {
    /// Create a new `BitPacker` with the specified bit ordering.
    pub fn new(order: BitOrder) -> Self {
        Self { order }
    }

    /// Pack a slice of bools into bytes.
    ///
    /// If the number of bits is not a multiple of 8, the final byte is padded
    /// with `false` (zero) bits in the trailing positions.
    pub fn pack(&self, bits: &[bool]) -> Vec<u8> {
        self.pack_n(bits, 8)
    }

    /// Pack a slice of bools into N-bit symbols (each stored in the low bits of a `u8`).
    ///
    /// `bits_per_symbol` must be in the range 1..=8. If the total number of bits
    /// is not a multiple of `bits_per_symbol`, the final symbol is padded with
    /// `false` (zero) bits.
    pub fn pack_n(&self, bits: &[bool], bits_per_symbol: usize) -> Vec<u8> {
        assert!(
            bits_per_symbol >= 1 && bits_per_symbol <= 8,
            "bits_per_symbol must be 1..=8"
        );

        let num_symbols = (bits.len() + bits_per_symbol - 1) / bits_per_symbol;
        let mut result = Vec::with_capacity(num_symbols);

        for chunk in bits.chunks(bits_per_symbol) {
            let mut byte: u8 = 0;
            for (i, &bit) in chunk.iter().enumerate() {
                if bit {
                    match self.order {
                        BitOrder::MsbFirst => {
                            byte |= 1 << (bits_per_symbol - 1 - i);
                        }
                        BitOrder::LsbFirst => {
                            byte |= 1 << i;
                        }
                    }
                }
            }
            result.push(byte);
        }

        result
    }
}

/// Unpacks bytes or N-bit symbols into boolean bit sequences.
#[derive(Debug, Clone)]
pub struct BitUnpacker {
    order: BitOrder,
}

impl BitUnpacker {
    /// Create a new `BitUnpacker` with the specified bit ordering.
    pub fn new(order: BitOrder) -> Self {
        Self { order }
    }

    /// Unpack bytes into individual bits.
    pub fn unpack(&self, bytes: &[u8]) -> Vec<bool> {
        self.unpack_n(bytes, 8)
    }

    /// Unpack N-bit symbols into individual bits.
    ///
    /// Each input byte is treated as containing `bits_per_symbol` valid bits
    /// in its low-order positions. `bits_per_symbol` must be 1..=8.
    pub fn unpack_n(&self, bytes: &[u8], bits_per_symbol: usize) -> Vec<bool> {
        assert!(
            bits_per_symbol >= 1 && bits_per_symbol <= 8,
            "bits_per_symbol must be 1..=8"
        );

        let mut result = Vec::with_capacity(bytes.len() * bits_per_symbol);

        for &byte in bytes {
            for i in 0..bits_per_symbol {
                let bit = match self.order {
                    BitOrder::MsbFirst => (byte >> (bits_per_symbol - 1 - i)) & 1 == 1,
                    BitOrder::LsbFirst => (byte >> i) & 1 == 1,
                };
                result.push(bit);
            }
        }

        result
    }
}

/// Streaming bit-width converter.
///
/// Repacks a stream of N-bit input symbols into M-bit output symbols. An internal
/// bit buffer accumulates input bits and emits complete output symbols as they
/// become available. Call [`flush`](StreamConverter::flush) at end-of-stream to
/// retrieve any remaining partial symbol (zero-padded).
#[derive(Debug, Clone)]
pub struct StreamConverter {
    bits_per_input: usize,
    bits_per_output: usize,
    buffer: u64,
    buffered_bits: usize,
}

impl StreamConverter {
    /// Create a new `StreamConverter`.
    ///
    /// * `bits_per_input` - number of valid bits in each input byte (1..=8)
    /// * `bits_per_output` - number of bits per output symbol (1..=8)
    pub fn new(bits_per_input: usize, bits_per_output: usize) -> Self {
        assert!(
            bits_per_input >= 1 && bits_per_input <= 8,
            "bits_per_input must be 1..=8"
        );
        assert!(
            bits_per_output >= 1 && bits_per_output <= 8,
            "bits_per_output must be 1..=8"
        );
        Self {
            bits_per_input,
            bits_per_output,
            buffer: 0,
            buffered_bits: 0,
        }
    }

    /// Feed input symbols and produce as many complete output symbols as possible.
    ///
    /// Input bytes are treated as MSB-first with `bits_per_input` valid bits each.
    /// Output symbols are emitted MSB-first with `bits_per_output` bits each.
    pub fn convert(&mut self, input: &[u8]) -> Vec<u8> {
        let mut output = Vec::new();

        for &byte in input {
            // Shift in the valid input bits (MSB-first)
            let masked = (byte as u64) & ((1u64 << self.bits_per_input) - 1);
            self.buffer = (self.buffer << self.bits_per_input) | masked;
            self.buffered_bits += self.bits_per_input;

            // Extract as many complete output symbols as possible
            while self.buffered_bits >= self.bits_per_output {
                self.buffered_bits -= self.bits_per_output;
                let symbol =
                    ((self.buffer >> self.buffered_bits) & ((1u64 << self.bits_per_output) - 1))
                        as u8;
                output.push(symbol);
            }
        }

        output
    }

    /// Flush any remaining buffered bits as a zero-padded output symbol.
    ///
    /// Returns an empty `Vec` if no bits are buffered.
    pub fn flush(&mut self) -> Vec<u8> {
        if self.buffered_bits == 0 {
            return Vec::new();
        }

        // Left-align remaining bits within the output symbol width
        let shift = self.bits_per_output - self.buffered_bits;
        let symbol = ((self.buffer << shift) & ((1u64 << self.bits_per_output) - 1)) as u8;
        self.buffered_bits = 0;
        self.buffer = 0;
        vec![symbol]
    }
}

// ---------------------------------------------------------------------------
// Convenience functions
// ---------------------------------------------------------------------------

/// Convert bytes to a boolean bit vector (MSB-first).
pub fn bytes_to_bits(bytes: &[u8]) -> Vec<bool> {
    BitUnpacker::new(BitOrder::MsbFirst).unpack(bytes)
}

/// Convert a boolean bit vector to bytes (MSB-first, zero-padded).
pub fn bits_to_bytes(bits: &[bool]) -> Vec<u8> {
    BitPacker::new(BitOrder::MsbFirst).pack(bits)
}

/// Gray code encoding: `val XOR (val >> 1)`.
pub fn gray_encode(val: u8) -> u8 {
    val ^ (val >> 1)
}

/// Gray code decoding (inverse of [`gray_encode`]).
pub fn gray_decode(val: u8) -> u8 {
    let mut n = val;
    let mut mask = n >> 1;
    while mask != 0 {
        n ^= mask;
        mask >>= 1;
    }
    n
}

/// Reverse the bit order within a byte.
///
/// Bit 0 becomes bit 7, bit 1 becomes bit 6, etc.
pub fn reverse_bits(byte: u8) -> u8 {
    let mut result: u8 = 0;
    let mut val = byte;
    for _ in 0..8 {
        result = (result << 1) | (val & 1);
        val >>= 1;
    }
    result
}

/// Swap the high and low nibbles of a byte.
///
/// `0xAB` becomes `0xBA`.
pub fn swap_nibbles(byte: u8) -> u8 {
    (byte >> 4) | (byte << 4)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_msb_first_packing() {
        let packer = BitPacker::new(BitOrder::MsbFirst);
        let bits = vec![true, false, true, false, false, false, false, true];
        let packed = packer.pack(&bits);
        assert_eq!(packed, vec![0xA1]);
    }

    #[test]
    fn test_lsb_first_packing() {
        let packer = BitPacker::new(BitOrder::LsbFirst);
        let bits = vec![true, false, true, false, false, false, false, true];
        let packed = packer.pack(&bits);
        // LSB-first: bit0=true(1), bit1=false(0), bit2=true(4), bit3=false,
        //            bit4=false, bit5=false, bit6=false, bit7=true(128)
        assert_eq!(packed, vec![0x85]); // 1 + 4 + 128 = 133 = 0x85
    }

    #[test]
    fn test_unpack_round_trip() {
        let packer = BitPacker::new(BitOrder::MsbFirst);
        let unpacker = BitUnpacker::new(BitOrder::MsbFirst);
        let original = vec![
            true, false, true, true, false, true, false, false, // 0xB4
            true, true, false, false, true, false, true, true, // 0xCB
        ];
        let packed = packer.pack(&original);
        let recovered = unpacker.unpack(&packed);
        assert_eq!(recovered, original);
    }

    #[test]
    fn test_pack_with_padding() {
        let packer = BitPacker::new(BitOrder::MsbFirst);
        // 5 bits: 10110 -> padded to 10110_000 = 0xB0
        let bits = vec![true, false, true, true, false];
        let packed = packer.pack(&bits);
        assert_eq!(packed, vec![0xB0]);
    }

    #[test]
    fn test_n_bit_packing() {
        let packer = BitPacker::new(BitOrder::MsbFirst);
        // 2 bits per symbol, MSB-first
        // [true, false] -> 0b10 = 2
        // [true, true]  -> 0b11 = 3
        // [false, true] -> 0b01 = 1
        // [false, false] -> 0b00 = 0
        let bits = vec![
            true, false, true, true, false, true, false, false,
        ];
        let packed = packer.pack_n(&bits, 2);
        assert_eq!(packed, vec![2, 3, 1, 0]);
    }

    #[test]
    fn test_stream_converter_1_to_8() {
        // 1-bit input -> 8-bit output
        let mut converter = StreamConverter::new(1, 8);
        // Feed 8 one-bit symbols representing 0xA1 = 10100001
        let input: Vec<u8> = vec![1, 0, 1, 0, 0, 0, 0, 1];
        let output = converter.convert(&input);
        assert_eq!(output, vec![0xA1]);
    }

    #[test]
    fn test_gray_encode_decode_round_trip() {
        for val in 0..=255u8 {
            let encoded = gray_encode(val);
            let decoded = gray_decode(encoded);
            assert_eq!(decoded, val, "Gray round-trip failed for {val}");
        }
    }

    #[test]
    fn test_reverse_bits() {
        assert_eq!(reverse_bits(0b10000000), 0b00000001);
        assert_eq!(reverse_bits(0b11010010), 0b01001011);
        assert_eq!(reverse_bits(0xFF), 0xFF);
        assert_eq!(reverse_bits(0x00), 0x00);
    }

    #[test]
    fn test_swap_nibbles() {
        assert_eq!(swap_nibbles(0xAB), 0xBA);
        assert_eq!(swap_nibbles(0x12), 0x21);
        assert_eq!(swap_nibbles(0x00), 0x00);
        assert_eq!(swap_nibbles(0xFF), 0xFF);
    }

    #[test]
    fn test_stream_converter_flush() {
        // 4-bit input -> 8-bit output; feed one 4-bit symbol, need 2 for a full output
        let mut converter = StreamConverter::new(4, 8);
        let output = converter.convert(&[0x0A]); // 4 bits in: 1010
        assert!(output.is_empty(), "should not produce output yet");

        let flushed = converter.flush();
        // 4 buffered bits (1010) left-shifted into 8-bit symbol -> 10100000 = 0xA0
        assert_eq!(flushed, vec![0xA0]);
    }
}
