//! Fletcher checksum algorithms (Fletcher-16, Fletcher-32, Fletcher-64) and related
//! error-detection checksums.
//!
//! This module provides streaming (incremental) and one-shot computation of Fletcher
//! checksums, Adler-32 (as used by zlib), BSD checksums, and the Internet checksum
//! (RFC 1071). It also includes check-byte generation, verification helpers, and
//! bit-error detection probability estimation.
//!
//! # Example
//!
//! ```
//! use r4w_core::fletcher_checksum::{FletcherChecksum, FletcherVariant};
//!
//! // One-shot Fletcher-16
//! let data = b"Hello, world!";
//! let cksum = FletcherChecksum::compute(FletcherVariant::Fletcher16, data);
//! assert_ne!(cksum, 0);
//!
//! // Streaming / incremental
//! let mut fc = FletcherChecksum::new(FletcherVariant::Fletcher16);
//! fc.update(&data[..5]);
//! fc.update(&data[5..]);
//! assert_eq!(fc.finish(), cksum);
//! ```

/// Which Fletcher variant (or width) to use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FletcherVariant {
    /// Fletcher-16: operates on 8-bit data words, produces a 16-bit checksum.
    Fletcher16,
    /// Fletcher-32: operates on 16-bit data words, produces a 32-bit checksum.
    Fletcher32,
    /// Fletcher-64: operates on 32-bit data words, produces a 64-bit checksum.
    Fletcher64,
}

/// A streaming Fletcher checksum calculator.
///
/// Supports Fletcher-16, Fletcher-32, and Fletcher-64. Accumulate data with
/// [`update`](Self::update) and retrieve the final checksum with
/// [`finish`](Self::finish).
#[derive(Debug, Clone)]
pub struct FletcherChecksum {
    variant: FletcherVariant,
    sum1: u64,
    sum2: u64,
    /// Leftover bytes when the variant requires multi-byte words and the input
    /// was not aligned (Fletcher-32 needs 2-byte words, Fletcher-64 needs 4-byte words).
    remainder: Vec<u8>,
}

impl FletcherChecksum {
    /// Create a new streaming checksum with the given variant.
    pub fn new(variant: FletcherVariant) -> Self {
        Self {
            variant,
            sum1: 0,
            sum2: 0,
            remainder: Vec::new(),
        }
    }

    /// One-shot computation over a complete buffer.
    pub fn compute(variant: FletcherVariant, data: &[u8]) -> u64 {
        let mut fc = Self::new(variant);
        fc.update(data);
        fc.finish()
    }

    /// Feed more data into the checksum.
    pub fn update(&mut self, data: &[u8]) {
        match self.variant {
            FletcherVariant::Fletcher16 => self.update_16(data),
            FletcherVariant::Fletcher32 => self.update_wide(data, 2),
            FletcherVariant::Fletcher64 => self.update_wide(data, 4),
        }
    }

    /// Return the final checksum value.
    ///
    /// For Fletcher-16 the result fits in `u16`, for Fletcher-32 in `u32`, and for
    /// Fletcher-64 in `u64`. The value is returned as `u64` in all cases for a
    /// uniform API.
    pub fn finish(&self) -> u64 {
        match self.variant {
            FletcherVariant::Fletcher16 => {
                let s1 = (self.sum1 % 255) as u64;
                let s2 = (self.sum2 % 255) as u64;
                (s2 << 8) | s1
            }
            FletcherVariant::Fletcher32 => {
                let s1 = (self.sum1 % 65535) as u64;
                let s2 = (self.sum2 % 65535) as u64;
                (s2 << 16) | s1
            }
            FletcherVariant::Fletcher64 => {
                let s1 = self.sum1 % 0xFFFF_FFFF;
                let s2 = self.sum2 % 0xFFFF_FFFF;
                (s2 << 32) | s1
            }
        }
    }

    /// Reset the checksum to its initial state.
    pub fn reset(&mut self) {
        self.sum1 = 0;
        self.sum2 = 0;
        self.remainder.clear();
    }

    // ---- internals ----

    fn update_16(&mut self, data: &[u8]) {
        // Process in blocks to avoid frequent modular reduction.
        // 255 * 5802 < 2^32, so we can safely accumulate 5802 bytes before reducing.
        const BLOCK: usize = 5802;
        let mut s1 = self.sum1;
        let mut s2 = self.sum2;
        for chunk in data.chunks(BLOCK) {
            for &b in chunk {
                s1 += b as u64;
                s2 += s1;
            }
            s1 %= 255;
            s2 %= 255;
        }
        self.sum1 = s1;
        self.sum2 = s2;
    }

    fn update_wide(&mut self, data: &[u8], word_size: usize) {
        // Prepend any remainder bytes from a previous call.
        let combined: Vec<u8>;
        let input = if !self.remainder.is_empty() {
            combined = [self.remainder.as_slice(), data].concat();
            self.remainder.clear();
            combined.as_slice()
        } else {
            data
        };

        let aligned_len = (input.len() / word_size) * word_size;
        if aligned_len < input.len() {
            self.remainder.extend_from_slice(&input[aligned_len..]);
        }

        let modulus: u64 = match self.variant {
            FletcherVariant::Fletcher32 => 65535,
            FletcherVariant::Fletcher64 => 0xFFFF_FFFF,
            _ => 255,
        };

        let mut s1 = self.sum1;
        let mut s2 = self.sum2;

        for chunk in input[..aligned_len].chunks(word_size) {
            let word: u64 = match word_size {
                2 => u16::from_le_bytes([chunk[0], chunk[1]]) as u64,
                4 => u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]) as u64,
                _ => unreachable!(),
            };
            s1 = (s1 + word) % modulus;
            s2 = (s2 + s1) % modulus;
        }

        self.sum1 = s1;
        self.sum2 = s2;
    }
}

// ---------------------------------------------------------------------------
// Check-byte generation and verification
// ---------------------------------------------------------------------------

/// Compute two check bytes for Fletcher-16 that, when appended to `data`, cause
/// the checksum over the extended message to be zero.
///
/// Returns `(c0, c1)` to append.
pub fn fletcher16_check_bytes(data: &[u8]) -> (u8, u8) {
    let mut s1: u64 = 0;
    let mut s2: u64 = 0;
    for &b in data {
        s1 = (s1 + b as u64) % 255;
        s2 = (s2 + s1) % 255;
    }
    let c0 = (255 - ((s1 + s2) % 255)) as u8;
    let c1 = (255 - ((s1 + c0 as u64) % 255)) as u8;
    (c0, c1)
}

/// Verify that `data` (which should include the two appended check bytes)
/// produces a Fletcher-16 checksum of zero.
pub fn fletcher16_verify(data_with_check: &[u8]) -> bool {
    let cksum = FletcherChecksum::compute(FletcherVariant::Fletcher16, data_with_check);
    let s1 = cksum & 0xFF;
    let s2 = (cksum >> 8) & 0xFF;
    s1 == 0 && s2 == 0
}

// ---------------------------------------------------------------------------
// Adler-32 (zlib)
// ---------------------------------------------------------------------------

/// Adler-32 checksum as used by zlib (RFC 1950).
///
/// This is closely related to Fletcher-16 but uses modulus 65521 (the largest
/// prime below 2^16) and an initial value of 1.
#[derive(Debug, Clone)]
pub struct Adler32 {
    a: u32,
    b: u32,
}

const ADLER_MOD: u32 = 65521;

impl Adler32 {
    /// Create a new Adler-32 accumulator.
    pub fn new() -> Self {
        Self { a: 1, b: 0 }
    }

    /// Feed more data.
    pub fn update(&mut self, data: &[u8]) {
        // Process in blocks. 65521 * 5552 < 2^32 so we can safely accumulate
        // 5552 bytes before reducing.
        const BLOCK: usize = 5552;
        for chunk in data.chunks(BLOCK) {
            for &byte in chunk {
                self.a += byte as u32;
                self.b += self.a;
            }
            self.a %= ADLER_MOD;
            self.b %= ADLER_MOD;
        }
    }

    /// Return the Adler-32 checksum.
    pub fn finish(&self) -> u32 {
        (self.b << 16) | self.a
    }

    /// One-shot computation.
    pub fn compute(data: &[u8]) -> u32 {
        let mut a = Self::new();
        a.update(data);
        a.finish()
    }

    /// Reset to initial state.
    pub fn reset(&mut self) {
        self.a = 1;
        self.b = 0;
    }
}

impl Default for Adler32 {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// BSD Checksum
// ---------------------------------------------------------------------------

/// Compute the classic BSD checksum (16-bit rotating checksum).
///
/// This is the algorithm used by the `sum` command with the `-r` flag on BSD
/// systems.
pub fn bsd_checksum(data: &[u8]) -> u16 {
    let mut cksum: u16 = 0;
    for &b in data {
        // Rotate right by 1 bit (within 16 bits)
        cksum = (cksum >> 1) | (cksum << 15);
        cksum = cksum.wrapping_add(b as u16);
    }
    cksum
}

// ---------------------------------------------------------------------------
// Internet Checksum (RFC 1071)
// ---------------------------------------------------------------------------

/// Compute the Internet checksum as defined in RFC 1071.
///
/// This is a one's-complement sum of 16-bit words, used in IP, TCP, and UDP
/// headers. If the data length is odd, it is padded with a zero byte.
pub fn internet_checksum(data: &[u8]) -> u16 {
    let mut sum: u32 = 0;
    let mut i = 0;
    while i + 1 < data.len() {
        let word = u16::from_be_bytes([data[i], data[i + 1]]);
        sum += word as u32;
        i += 2;
    }
    // If odd number of bytes, pad with zero.
    if i < data.len() {
        sum += (data[i] as u32) << 8;
    }
    // Fold 32-bit sum to 16 bits.
    while sum >> 16 != 0 {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    !(sum as u16)
}

/// Verify an internet checksum. Compute over the data that includes the
/// checksum field; the result should be `0xFFFF` (all ones) if valid.
pub fn internet_checksum_verify(data: &[u8]) -> bool {
    let mut sum: u32 = 0;
    let mut i = 0;
    while i + 1 < data.len() {
        let word = u16::from_be_bytes([data[i], data[i + 1]]);
        sum += word as u32;
        i += 2;
    }
    if i < data.len() {
        sum += (data[i] as u32) << 8;
    }
    while sum >> 16 != 0 {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    sum as u16 == 0xFFFF
}

// ---------------------------------------------------------------------------
// Simple checksum (byte sum mod 256) - for comparison
// ---------------------------------------------------------------------------

/// Trivial 8-bit checksum: sum of all bytes modulo 256.
pub fn simple_checksum(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, &b| acc.wrapping_add(b))
}

// ---------------------------------------------------------------------------
// Detection probability estimation
// ---------------------------------------------------------------------------

/// Estimate the probability that a random `n_bit`-bit error pattern goes
/// **undetected** by a checksum with the given number of check bits.
///
/// For a simple sum checksum the undetected error probability is approximately
/// `2^{-c}` where `c` is the number of check bits. Fletcher and Adler have
/// slightly better performance; CRC is significantly better for burst errors.
///
/// Returns a tuple of `(f64, f64)` representing `(simple_sum_prob, fletcher_prob)`.
/// These are rough analytical estimates, not exact values.
pub fn undetected_error_probability(check_bits: u32, _n_bit_errors: u32) -> (f64, f64) {
    // Simple checksum: roughly 1 / 2^c
    let simple = 2.0_f64.powi(-(check_bits as i32));
    // Fletcher-style has two independent sums so the second sum provides
    // ordering information, making it slightly better.
    // A conservative estimate is about half the simple rate.
    let fletcher = 2.0_f64.powi(-(check_bits as i32) - 1);
    (simple, fletcher)
}

/// Compare detection capabilities of various checksum algorithms for a given
/// message. Returns a summary struct.
pub fn compare_checksums(data: &[u8]) -> ChecksumComparison {
    ChecksumComparison {
        simple: simple_checksum(data) as u64,
        fletcher16: FletcherChecksum::compute(FletcherVariant::Fletcher16, data),
        fletcher32: FletcherChecksum::compute(FletcherVariant::Fletcher32, data),
        adler32: Adler32::compute(data) as u64,
        bsd: bsd_checksum(data) as u64,
        internet: internet_checksum(data) as u64,
    }
}

/// Result of [`compare_checksums`].
#[derive(Debug, Clone)]
pub struct ChecksumComparison {
    /// Simple 8-bit sum.
    pub simple: u64,
    /// Fletcher-16 checksum.
    pub fletcher16: u64,
    /// Fletcher-32 checksum.
    pub fletcher32: u64,
    /// Adler-32 checksum.
    pub adler32: u64,
    /// BSD rotating checksum.
    pub bsd: u64,
    /// Internet checksum (RFC 1071).
    pub internet: u64,
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ---- Fletcher-16 ----

    #[test]
    fn test_fletcher16_known_vector() {
        // Verify against manual computation.
        let data = b"abcde";
        let cksum = FletcherChecksum::compute(FletcherVariant::Fletcher16, data);
        let mut s1: u64 = 0;
        let mut s2: u64 = 0;
        for &b in data.iter() {
            s1 = (s1 + b as u64) % 255;
            s2 = (s2 + s1) % 255;
        }
        let expected = (s2 << 8) | s1;
        assert_eq!(cksum, expected);
    }

    #[test]
    fn test_fletcher16_empty() {
        let cksum = FletcherChecksum::compute(FletcherVariant::Fletcher16, b"");
        assert_eq!(cksum, 0);
    }

    #[test]
    fn test_fletcher16_streaming() {
        let data = b"The quick brown fox jumps over the lazy dog";
        let one_shot = FletcherChecksum::compute(FletcherVariant::Fletcher16, data);

        let mut fc = FletcherChecksum::new(FletcherVariant::Fletcher16);
        fc.update(&data[..10]);
        fc.update(&data[10..25]);
        fc.update(&data[25..]);
        assert_eq!(fc.finish(), one_shot);
    }

    #[test]
    fn test_fletcher16_single_byte() {
        let cksum = FletcherChecksum::compute(FletcherVariant::Fletcher16, &[0x42]);
        // s1 = 0x42, s2 = 0x42
        assert_eq!(cksum, (0x42u64 << 8) | 0x42);
    }

    // ---- Fletcher-32 ----

    #[test]
    fn test_fletcher32_known() {
        let data: Vec<u8> = (0u8..=7).collect();
        let cksum = FletcherChecksum::compute(FletcherVariant::Fletcher32, &data);
        assert_ne!(cksum, 0);

        // Verify streaming matches one-shot.
        let mut fc = FletcherChecksum::new(FletcherVariant::Fletcher32);
        fc.update(&data[..4]);
        fc.update(&data[4..]);
        assert_eq!(fc.finish(), cksum);
    }

    #[test]
    fn test_fletcher32_streaming_odd_split() {
        // Split data at non-word boundary to exercise remainder logic.
        let data: Vec<u8> = (0u8..20).collect();
        let one_shot = FletcherChecksum::compute(FletcherVariant::Fletcher32, &data);

        let mut fc = FletcherChecksum::new(FletcherVariant::Fletcher32);
        fc.update(&data[..3]);
        fc.update(&data[3..11]);
        fc.update(&data[11..]);
        assert_eq!(fc.finish(), one_shot);
    }

    #[test]
    fn test_fletcher32_empty() {
        let cksum = FletcherChecksum::compute(FletcherVariant::Fletcher32, b"");
        assert_eq!(cksum, 0);
    }

    // ---- Fletcher-64 ----

    #[test]
    fn test_fletcher64_known() {
        let data: Vec<u8> = (0u8..16).collect();
        let cksum = FletcherChecksum::compute(FletcherVariant::Fletcher64, &data);
        assert_ne!(cksum, 0);

        let mut fc = FletcherChecksum::new(FletcherVariant::Fletcher64);
        fc.update(&data);
        assert_eq!(fc.finish(), cksum);
    }

    #[test]
    fn test_fletcher64_streaming_odd_split() {
        let data: Vec<u8> = (0u8..32).collect();
        let one_shot = FletcherChecksum::compute(FletcherVariant::Fletcher64, &data);

        let mut fc = FletcherChecksum::new(FletcherVariant::Fletcher64);
        fc.update(&data[..5]);
        fc.update(&data[5..17]);
        fc.update(&data[17..]);
        assert_eq!(fc.finish(), one_shot);
    }

    // ---- Check bytes / verification ----

    #[test]
    fn test_fletcher16_check_bytes() {
        let data = b"Hello, world!";
        let (c0, c1) = fletcher16_check_bytes(data);

        let mut extended = data.to_vec();
        extended.push(c0);
        extended.push(c1);

        assert!(fletcher16_verify(&extended));
    }

    #[test]
    fn test_fletcher16_check_bytes_empty() {
        let data = b"";
        let (c0, c1) = fletcher16_check_bytes(data);
        let extended = vec![c0, c1];
        assert!(fletcher16_verify(&extended));
    }

    #[test]
    fn test_fletcher16_verify_detects_corruption() {
        let data = b"Test data";
        let (c0, c1) = fletcher16_check_bytes(data);
        let mut extended = data.to_vec();
        extended.push(c0);
        extended.push(c1);

        // Corrupt one byte.
        extended[3] ^= 0x01;
        assert!(!fletcher16_verify(&extended));
    }

    // ---- Adler-32 ----

    #[test]
    fn test_adler32_known_vector() {
        // Well-known: Adler32("Wikipedia") = 0x11E60398
        let cksum = Adler32::compute(b"Wikipedia");
        assert_eq!(cksum, 0x11E60398);
    }

    #[test]
    fn test_adler32_empty() {
        let cksum = Adler32::compute(b"");
        // a=1, b=0 -> 0x00000001
        assert_eq!(cksum, 1);
    }

    #[test]
    fn test_adler32_streaming() {
        let data = b"Hello, Adler-32 world!";
        let one_shot = Adler32::compute(data);

        let mut a = Adler32::new();
        a.update(&data[..5]);
        a.update(&data[5..]);
        assert_eq!(a.finish(), one_shot);
    }

    // ---- BSD checksum ----

    #[test]
    fn test_bsd_checksum_nonempty() {
        let cksum = bsd_checksum(b"abc");
        assert_ne!(cksum, 0);
        // Deterministic.
        assert_eq!(cksum, bsd_checksum(b"abc"));
    }

    #[test]
    fn test_bsd_checksum_empty() {
        assert_eq!(bsd_checksum(b""), 0);
    }

    // ---- Internet checksum ----

    #[test]
    fn test_internet_checksum_self_verify() {
        // Checksum of (data || checksum) should verify when data is 16-bit aligned.
        let data = b"Hell";  // even-length so checksum is word-aligned
        let cksum = internet_checksum(data);

        let mut msg = data.to_vec();
        msg.push((cksum >> 8) as u8);
        msg.push(cksum as u8);

        assert!(internet_checksum_verify(&msg));
    }

    #[test]
    fn test_internet_checksum_even_length() {
        let data = [0x00, 0x01, 0xf2, 0x03];
        let cksum = internet_checksum(&data);
        let mut msg = data.to_vec();
        msg.push((cksum >> 8) as u8);
        msg.push(cksum as u8);
        assert!(internet_checksum_verify(&msg));
    }

    // ---- Simple checksum ----

    #[test]
    fn test_simple_checksum() {
        assert_eq!(simple_checksum(&[1, 2, 3, 4]), 10);
        assert_eq!(simple_checksum(&[255, 1]), 0); // wrapping
    }

    // ---- Detection probability ----

    #[test]
    fn test_undetected_error_probability() {
        let (simple, fletcher) = undetected_error_probability(16, 1);
        // Simple: ~1/65536, Fletcher: ~1/131072
        assert!(simple > 0.0 && simple < 1.0);
        assert!(fletcher < simple);
    }

    // ---- Compare ----

    #[test]
    fn test_compare_checksums() {
        let data = b"comparison test";
        let cmp = compare_checksums(data);
        assert_ne!(cmp.fletcher16, 0);
        assert_ne!(cmp.fletcher32, 0);
        assert_ne!(cmp.adler32, 0);
        assert_ne!(cmp.bsd, 0);
    }

    // ---- Reset ----

    #[test]
    fn test_reset() {
        let data = b"reset test";
        let mut fc = FletcherChecksum::new(FletcherVariant::Fletcher16);
        fc.update(data);
        let first = fc.finish();
        fc.reset();
        fc.update(data);
        assert_eq!(fc.finish(), first);
    }

    // ---- Adler-32 reset ----

    #[test]
    fn test_adler32_reset() {
        let mut a = Adler32::new();
        a.update(b"data");
        let first = a.finish();
        a.reset();
        a.update(b"data");
        assert_eq!(a.finish(), first);
    }
}
