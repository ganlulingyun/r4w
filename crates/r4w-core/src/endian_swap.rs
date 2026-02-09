//! Endian Swap — Byte order conversion for multi-byte samples
//!
//! Swaps byte ordering (little-endian ↔ big-endian) for 16-bit,
//! 32-bit, and 64-bit values in byte streams. Essential for
//! interoperability between different hardware platforms, file
//! formats, and network protocols in SDR systems.
//! GNU Radio equivalent: `endian_swap`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::endian_swap::{swap_bytes_16, swap_bytes_32};
//!
//! let data = vec![0x01, 0x02, 0x03, 0x04];
//! let swapped16 = swap_bytes_16(&data);
//! assert_eq!(swapped16, vec![0x02, 0x01, 0x04, 0x03]);
//!
//! let swapped32 = swap_bytes_32(&data);
//! assert_eq!(swapped32, vec![0x04, 0x03, 0x02, 0x01]);
//! ```

/// Swap byte pairs (16-bit endian swap).
///
/// Each pair of bytes [a, b] becomes [b, a].
/// Input length should be even; trailing byte is dropped.
pub fn swap_bytes_16(input: &[u8]) -> Vec<u8> {
    let mut output = Vec::with_capacity(input.len());
    for chunk in input.chunks_exact(2) {
        output.push(chunk[1]);
        output.push(chunk[0]);
    }
    output
}

/// Swap 4-byte groups (32-bit endian swap).
///
/// Each group [a, b, c, d] becomes [d, c, b, a].
/// Trailing bytes that don't fill a group are dropped.
pub fn swap_bytes_32(input: &[u8]) -> Vec<u8> {
    let mut output = Vec::with_capacity(input.len());
    for chunk in input.chunks_exact(4) {
        output.push(chunk[3]);
        output.push(chunk[2]);
        output.push(chunk[1]);
        output.push(chunk[0]);
    }
    output
}

/// Swap 8-byte groups (64-bit endian swap).
pub fn swap_bytes_64(input: &[u8]) -> Vec<u8> {
    let mut output = Vec::with_capacity(input.len());
    for chunk in input.chunks_exact(8) {
        for i in (0..8).rev() {
            output.push(chunk[i]);
        }
    }
    output
}

/// In-place 16-bit endian swap.
pub fn swap_bytes_16_inplace(data: &mut [u8]) {
    for chunk in data.chunks_exact_mut(2) {
        chunk.swap(0, 1);
    }
}

/// In-place 32-bit endian swap.
pub fn swap_bytes_32_inplace(data: &mut [u8]) {
    for chunk in data.chunks_exact_mut(4) {
        chunk.swap(0, 3);
        chunk.swap(1, 2);
    }
}

/// In-place 64-bit endian swap.
pub fn swap_bytes_64_inplace(data: &mut [u8]) {
    for chunk in data.chunks_exact_mut(8) {
        chunk.swap(0, 7);
        chunk.swap(1, 6);
        chunk.swap(2, 5);
        chunk.swap(3, 4);
    }
}

/// Swap endianness of a slice of i16 values.
pub fn swap_i16(input: &[i16]) -> Vec<i16> {
    input.iter().map(|&x| x.swap_bytes()).collect()
}

/// Swap endianness of a slice of u16 values.
pub fn swap_u16(input: &[u16]) -> Vec<u16> {
    input.iter().map(|&x| x.swap_bytes()).collect()
}

/// Swap endianness of a slice of i32 values.
pub fn swap_i32(input: &[i32]) -> Vec<i32> {
    input.iter().map(|&x| x.swap_bytes()).collect()
}

/// Swap endianness of a slice of u32 values.
pub fn swap_u32(input: &[u32]) -> Vec<u32> {
    input.iter().map(|&x| x.swap_bytes()).collect()
}

/// Swap endianness of a slice of f32 values.
pub fn swap_f32(input: &[f32]) -> Vec<f32> {
    input
        .iter()
        .map(|&x| f32::from_bits(x.to_bits().swap_bytes()))
        .collect()
}

/// Swap endianness of a slice of f64 values.
pub fn swap_f64(input: &[f64]) -> Vec<f64> {
    input
        .iter()
        .map(|&x| f64::from_bits(x.to_bits().swap_bytes()))
        .collect()
}

/// Bit reversal within each byte.
pub fn reverse_bits(input: &[u8]) -> Vec<u8> {
    input.iter().map(|&x| x.reverse_bits()).collect()
}

/// In-place bit reversal within each byte.
pub fn reverse_bits_inplace(data: &mut [u8]) {
    for x in data.iter_mut() {
        *x = x.reverse_bits();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_swap_16() {
        let data = vec![0x01, 0x02, 0x03, 0x04];
        assert_eq!(swap_bytes_16(&data), vec![0x02, 0x01, 0x04, 0x03]);
    }

    #[test]
    fn test_swap_32() {
        let data = vec![0x01, 0x02, 0x03, 0x04];
        assert_eq!(swap_bytes_32(&data), vec![0x04, 0x03, 0x02, 0x01]);
    }

    #[test]
    fn test_swap_64() {
        let data: Vec<u8> = (1..=8).collect();
        assert_eq!(
            swap_bytes_64(&data),
            vec![8, 7, 6, 5, 4, 3, 2, 1]
        );
    }

    #[test]
    fn test_swap_16_inplace() {
        let mut data = vec![0xAA, 0xBB, 0xCC, 0xDD];
        swap_bytes_16_inplace(&mut data);
        assert_eq!(data, vec![0xBB, 0xAA, 0xDD, 0xCC]);
    }

    #[test]
    fn test_swap_32_inplace() {
        let mut data = vec![1, 2, 3, 4, 5, 6, 7, 8];
        swap_bytes_32_inplace(&mut data);
        assert_eq!(data, vec![4, 3, 2, 1, 8, 7, 6, 5]);
    }

    #[test]
    fn test_swap_64_inplace() {
        let mut data: Vec<u8> = (1..=8).collect();
        swap_bytes_64_inplace(&mut data);
        assert_eq!(data, vec![8, 7, 6, 5, 4, 3, 2, 1]);
    }

    #[test]
    fn test_roundtrip_16() {
        let original = vec![0x12, 0x34, 0x56, 0x78];
        let swapped = swap_bytes_16(&original);
        let restored = swap_bytes_16(&swapped);
        assert_eq!(restored, original);
    }

    #[test]
    fn test_roundtrip_32() {
        let original = vec![0x12, 0x34, 0x56, 0x78];
        let swapped = swap_bytes_32(&original);
        let restored = swap_bytes_32(&swapped);
        assert_eq!(restored, original);
    }

    #[test]
    fn test_swap_i16() {
        let data = vec![0x0102i16, 0x0304i16];
        let swapped = swap_i16(&data);
        assert_eq!(swapped, vec![0x0201i16, 0x0403i16]);
    }

    #[test]
    fn test_swap_f32_roundtrip() {
        let original = vec![1.0f32, -2.5, 3.14];
        let swapped = swap_f32(&original);
        let restored = swap_f32(&swapped);
        assert_eq!(restored, original);
    }

    #[test]
    fn test_swap_f64_roundtrip() {
        let original = vec![1.0f64, -2.5, 3.14159];
        let swapped = swap_f64(&original);
        let restored = swap_f64(&swapped);
        assert_eq!(restored, original);
    }

    #[test]
    fn test_reverse_bits() {
        assert_eq!(reverse_bits(&[0b10000000]), vec![0b00000001]);
        assert_eq!(reverse_bits(&[0b11110000]), vec![0b00001111]);
        assert_eq!(reverse_bits(&[0xFF]), vec![0xFF]);
        assert_eq!(reverse_bits(&[0x00]), vec![0x00]);
    }

    #[test]
    fn test_reverse_bits_roundtrip() {
        let data = vec![0xA5, 0x3C, 0x81];
        let reversed = reverse_bits(&data);
        let restored = reverse_bits(&reversed);
        assert_eq!(restored, data);
    }

    #[test]
    fn test_trailing_bytes_dropped() {
        // Odd-length input: trailing byte is dropped for 16-bit swap
        let data = vec![1, 2, 3];
        assert_eq!(swap_bytes_16(&data), vec![2, 1]); // Only first pair
    }

    #[test]
    fn test_empty() {
        assert!(swap_bytes_16(&[]).is_empty());
        assert!(swap_bytes_32(&[]).is_empty());
        assert!(swap_bytes_64(&[]).is_empty());
        assert!(reverse_bits(&[]).is_empty());
    }
}
