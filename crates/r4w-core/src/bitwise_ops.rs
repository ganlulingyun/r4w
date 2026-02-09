//! Bitwise Operations — XOR, AND, OR, NOT for byte/bit streams
//!
//! Element-wise logical operations on byte streams. Essential for
//! scrambling (XOR with PN sequence), masking, syndrome computation,
//! and general bit manipulation in coding/framing blocks.
//! GNU Radio equivalents: `xor_bb`, `and_bb`, `or_bb`, `not_bb`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::bitwise_ops::*;
//!
//! let a = vec![0xFFu8, 0x0F, 0xAA];
//! let b = vec![0x0Fu8, 0xF0, 0x55];
//! assert_eq!(xor_bb(&a, &b), vec![0xF0, 0xFF, 0xFF]);
//! assert_eq!(and_bb(&a, &b), vec![0x0F, 0x00, 0x00]);
//! assert_eq!(or_bb(&a, &b),  vec![0xFF, 0xFF, 0xFF]);
//! assert_eq!(not_bb(&a),     vec![0x00, 0xF0, 0x55]);
//! ```

/// Element-wise XOR of two byte streams.
pub fn xor_bb(a: &[u8], b: &[u8]) -> Vec<u8> {
    a.iter().zip(b.iter()).map(|(&x, &y)| x ^ y).collect()
}

/// Element-wise AND of two byte streams.
pub fn and_bb(a: &[u8], b: &[u8]) -> Vec<u8> {
    a.iter().zip(b.iter()).map(|(&x, &y)| x & y).collect()
}

/// Element-wise OR of two byte streams.
pub fn or_bb(a: &[u8], b: &[u8]) -> Vec<u8> {
    a.iter().zip(b.iter()).map(|(&x, &y)| x | y).collect()
}

/// Bitwise NOT (complement) of a byte stream.
pub fn not_bb(a: &[u8]) -> Vec<u8> {
    a.iter().map(|&x| !x).collect()
}

/// XOR with a constant byte (all elements).
pub fn xor_const(a: &[u8], val: u8) -> Vec<u8> {
    a.iter().map(|&x| x ^ val).collect()
}

/// AND with a constant byte (mask).
pub fn and_const(a: &[u8], mask: u8) -> Vec<u8> {
    a.iter().map(|&x| x & mask).collect()
}

/// OR with a constant byte.
pub fn or_const(a: &[u8], val: u8) -> Vec<u8> {
    a.iter().map(|&x| x | val).collect()
}

/// In-place XOR of two byte slices (a ^= b).
pub fn xor_inplace(a: &mut [u8], b: &[u8]) {
    for (x, &y) in a.iter_mut().zip(b.iter()) {
        *x ^= y;
    }
}

/// In-place XOR with constant.
pub fn xor_const_inplace(a: &mut [u8], val: u8) {
    for x in a.iter_mut() {
        *x ^= val;
    }
}

/// In-place NOT.
pub fn not_inplace(a: &mut [u8]) {
    for x in a.iter_mut() {
        *x = !*x;
    }
}

/// Bit-level XOR on boolean streams.
pub fn xor_bits(a: &[bool], b: &[bool]) -> Vec<bool> {
    a.iter().zip(b.iter()).map(|(&x, &y)| x ^ y).collect()
}

/// Bit-level AND on boolean streams.
pub fn and_bits(a: &[bool], b: &[bool]) -> Vec<bool> {
    a.iter().zip(b.iter()).map(|(&x, &y)| x & y).collect()
}

/// Bit-level OR on boolean streams.
pub fn or_bits(a: &[bool], b: &[bool]) -> Vec<bool> {
    a.iter().zip(b.iter()).map(|(&x, &y)| x | y).collect()
}

/// Bit-level NOT on boolean stream.
pub fn not_bits(a: &[bool]) -> Vec<bool> {
    a.iter().map(|&x| !x).collect()
}

/// Count number of set bits (popcount) in a byte.
#[inline]
pub fn popcount(x: u8) -> u32 {
    x.count_ones()
}

/// Hamming distance between two byte slices (total differing bits).
pub fn hamming_distance(a: &[u8], b: &[u8]) -> u32 {
    a.iter()
        .zip(b.iter())
        .map(|(&x, &y)| (x ^ y).count_ones())
        .sum()
}

/// Hamming distance between two bit sequences.
pub fn hamming_distance_bits(a: &[bool], b: &[bool]) -> usize {
    a.iter().zip(b.iter()).filter(|(&x, &y)| x != y).count()
}

/// Compute parity of a byte (1 if odd number of set bits).
#[inline]
pub fn parity(x: u8) -> bool {
    x.count_ones() % 2 == 1
}

/// Compute parity of a byte slice (XOR all bytes, then check parity).
pub fn parity_block(data: &[u8]) -> bool {
    let xored = data.iter().fold(0u8, |acc, &x| acc ^ x);
    parity(xored)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_xor() {
        assert_eq!(xor_bb(&[0xFF, 0x0F], &[0x0F, 0xF0]), vec![0xF0, 0xFF]);
    }

    #[test]
    fn test_and() {
        assert_eq!(and_bb(&[0xFF, 0x0F], &[0x0F, 0xF0]), vec![0x0F, 0x00]);
    }

    #[test]
    fn test_or() {
        assert_eq!(or_bb(&[0xF0, 0x0F], &[0x0F, 0xF0]), vec![0xFF, 0xFF]);
    }

    #[test]
    fn test_not() {
        assert_eq!(not_bb(&[0xFF, 0x00, 0xAA]), vec![0x00, 0xFF, 0x55]);
    }

    #[test]
    fn test_xor_const() {
        assert_eq!(xor_const(&[0x00, 0xFF], 0xFF), vec![0xFF, 0x00]);
    }

    #[test]
    fn test_and_const() {
        assert_eq!(and_const(&[0xFF, 0xAB], 0x0F), vec![0x0F, 0x0B]);
    }

    #[test]
    fn test_or_const() {
        assert_eq!(or_const(&[0x00, 0xF0], 0x0F), vec![0x0F, 0xFF]);
    }

    #[test]
    fn test_xor_inplace() {
        let mut a = vec![0xFF, 0x00];
        xor_inplace(&mut a, &[0x0F, 0xF0]);
        assert_eq!(a, vec![0xF0, 0xF0]);
    }

    #[test]
    fn test_not_inplace() {
        let mut a = vec![0xAA];
        not_inplace(&mut a);
        assert_eq!(a, vec![0x55]);
    }

    #[test]
    fn test_xor_bits() {
        let a = vec![true, false, true, false];
        let b = vec![true, true, false, false];
        assert_eq!(xor_bits(&a, &b), vec![false, true, true, false]);
    }

    #[test]
    fn test_and_bits() {
        let a = vec![true, false, true];
        let b = vec![true, true, false];
        assert_eq!(and_bits(&a, &b), vec![true, false, false]);
    }

    #[test]
    fn test_or_bits() {
        let a = vec![true, false, false];
        let b = vec![false, false, true];
        assert_eq!(or_bits(&a, &b), vec![true, false, true]);
    }

    #[test]
    fn test_not_bits() {
        assert_eq!(not_bits(&[true, false]), vec![false, true]);
    }

    #[test]
    fn test_hamming_distance() {
        assert_eq!(hamming_distance(&[0xFF], &[0x00]), 8);
        assert_eq!(hamming_distance(&[0xFF], &[0xFF]), 0);
        assert_eq!(hamming_distance(&[0xFF], &[0xFE]), 1);
    }

    #[test]
    fn test_hamming_distance_bits() {
        let a = vec![true, true, false];
        let b = vec![true, false, false];
        assert_eq!(hamming_distance_bits(&a, &b), 1);
    }

    #[test]
    fn test_popcount() {
        assert_eq!(popcount(0x00), 0);
        assert_eq!(popcount(0xFF), 8);
        assert_eq!(popcount(0xAA), 4);
    }

    #[test]
    fn test_parity() {
        assert!(!parity(0x00));  // 0 bits set → even
        assert!(parity(0x01));   // 1 bit set → odd
        assert!(!parity(0x03));  // 2 bits set → even
        assert!(parity(0x07));   // 3 bits set → odd
    }

    #[test]
    fn test_parity_block() {
        assert!(!parity_block(&[0x00, 0x00]));
        assert!(parity_block(&[0x01]));
        assert!(!parity_block(&[0x03, 0x00])); // 2 bits → even
    }

    #[test]
    fn test_empty() {
        assert!(xor_bb(&[], &[]).is_empty());
        assert!(not_bb(&[]).is_empty());
        assert_eq!(hamming_distance(&[], &[]), 0);
    }

    #[test]
    fn test_unequal_lengths() {
        // zip truncates to shorter
        assert_eq!(xor_bb(&[0xFF, 0xFF, 0xFF], &[0x00, 0x00]), vec![0xFF, 0xFF]);
    }
}
