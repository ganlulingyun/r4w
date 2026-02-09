//! Check LFSR — Verify LFSR sequences in received data
//!
//! Detects and verifies known LFSR (pseudo-random) sequences in
//! received bit streams. Used for BER testing, synchronization
//! verification, and data integrity checking in comms test setups.
//! GNU Radio equivalent: `check_lfsr`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::check_lfsr::{LfsrChecker, generate_reference};
//!
//! // Generate a reference 7-bit LFSR sequence
//! let reference = generate_reference(7, 0x60, 1, 127);
//! assert_eq!(reference.len(), 127);
//!
//! // Check received bits against reference
//! let mut checker = LfsrChecker::new(7, 0x60, 1);
//! let (matched, errors) = checker.check(&reference);
//! assert_eq!(matched, 127);
//! assert_eq!(errors, 0);
//! ```

/// Generate a reference LFSR sequence.
///
/// - `nbits`: register length
/// - `taps`: feedback polynomial
/// - `seed`: initial state
/// - `length`: number of bits to generate
pub fn generate_reference(nbits: u32, taps: u64, seed: u64, length: usize) -> Vec<u8> {
    let nbits = nbits.clamp(1, 63);
    let mask = (1u64 << nbits) - 1;
    let mut state = (seed & mask).max(1);
    let taps = taps & mask;

    let mut output = Vec::with_capacity(length);
    for _ in 0..length {
        output.push((state & 1) as u8);
        let feedback = state & 1;
        state >>= 1;
        if feedback != 0 {
            state ^= taps;
        }
        state &= mask;
    }
    output
}

/// LFSR sequence checker for BER testing.
#[derive(Debug, Clone)]
pub struct LfsrChecker {
    /// Register length.
    nbits: u32,
    /// Feedback taps.
    taps: u64,
    /// Initial seed.
    seed: u64,
    /// Mask.
    mask: u64,
    /// Current LFSR state (for tracking).
    state: u64,
    /// Total bits checked.
    total_bits: usize,
    /// Total bit errors found.
    total_errors: usize,
    /// Synchronized flag.
    synchronized: bool,
}

impl LfsrChecker {
    /// Create an LFSR checker.
    pub fn new(nbits: u32, taps: u64, seed: u64) -> Self {
        let nbits = nbits.clamp(1, 63);
        let mask = (1u64 << nbits) - 1;
        Self {
            nbits,
            taps: taps & mask,
            seed: (seed & mask).max(1),
            mask,
            state: (seed & mask).max(1),
            total_bits: 0,
            total_errors: 0,
            synchronized: false,
        }
    }

    /// Generate next expected bit and advance state.
    #[inline]
    fn next_expected(&mut self) -> u8 {
        let output = (self.state & 1) as u8;
        let feedback = self.state & 1;
        self.state >>= 1;
        if feedback != 0 {
            self.state ^= self.taps;
        }
        self.state &= self.mask;
        output
    }

    /// Check a block of received bits against expected LFSR sequence.
    ///
    /// Returns (bits_checked, bit_errors).
    pub fn check(&mut self, received: &[u8]) -> (usize, usize) {
        let mut errors = 0;
        for &bit in received {
            let expected = self.next_expected();
            if (bit & 1) != expected {
                errors += 1;
            }
        }
        self.total_bits += received.len();
        self.total_errors += errors;
        (received.len(), errors)
    }

    /// Check boolean bits.
    pub fn check_bool(&mut self, received: &[bool]) -> (usize, usize) {
        let mut errors = 0;
        for &bit in received {
            let expected = self.next_expected() != 0;
            if bit != expected {
                errors += 1;
            }
        }
        self.total_bits += received.len();
        self.total_errors += errors;
        (received.len(), errors)
    }

    /// Try to synchronize by searching for the LFSR sequence in data.
    ///
    /// Tries all possible starting states and returns the number of
    /// matching bits for the best alignment.
    pub fn synchronize(&mut self, data: &[u8]) -> usize {
        let period = ((1u64 << self.nbits) - 1) as usize;
        let search_len = data.len().min(period);

        let mut best_offset = 0;
        let mut best_matches = 0;

        // Generate one full period
        let reference = generate_reference(self.nbits, self.taps, self.seed, period);

        for offset in 0..reference.len() {
            let mut matches = 0;
            for i in 0..search_len {
                let ref_idx = (offset + i) % reference.len();
                if (data[i] & 1) == reference[ref_idx] {
                    matches += 1;
                }
            }
            if matches > best_matches {
                best_matches = matches;
                best_offset = offset;
            }
        }

        if best_matches > search_len * 3 / 4 {
            // Good enough sync — reset state to match offset
            self.state = self.seed;
            for _ in 0..best_offset {
                self.next_expected();
            }
            self.synchronized = true;
        }

        best_matches
    }

    /// Get total bits checked.
    pub fn total_bits(&self) -> usize {
        self.total_bits
    }

    /// Get total bit errors.
    pub fn total_errors(&self) -> usize {
        self.total_errors
    }

    /// Get BER (bit error rate).
    pub fn ber(&self) -> f64 {
        if self.total_bits == 0 {
            return 0.0;
        }
        self.total_errors as f64 / self.total_bits as f64
    }

    /// Is synchronized?
    pub fn is_synchronized(&self) -> bool {
        self.synchronized
    }

    /// Reset checker state.
    pub fn reset(&mut self) {
        self.state = self.seed;
        self.total_bits = 0;
        self.total_errors = 0;
        self.synchronized = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_reference() {
        let seq = generate_reference(3, 0x6, 1, 7);
        assert_eq!(seq.len(), 7);
        for &b in &seq {
            assert!(b <= 1);
        }
    }

    #[test]
    fn test_reference_period() {
        // 7-bit LFSR should have period 127
        let seq = generate_reference(7, 0x60, 1, 254);
        assert_eq!(&seq[..127], &seq[127..]);
    }

    #[test]
    fn test_perfect_check() {
        let seq = generate_reference(7, 0x60, 1, 100);
        let mut checker = LfsrChecker::new(7, 0x60, 1);
        let (bits, errors) = checker.check(&seq);
        assert_eq!(bits, 100);
        assert_eq!(errors, 0);
        assert_eq!(checker.ber(), 0.0);
    }

    #[test]
    fn test_check_with_errors() {
        let mut seq = generate_reference(7, 0x60, 1, 100);
        // Flip some bits
        seq[0] ^= 1;
        seq[10] ^= 1;
        seq[50] ^= 1;
        let mut checker = LfsrChecker::new(7, 0x60, 1);
        let (bits, errors) = checker.check(&seq);
        assert_eq!(bits, 100);
        assert_eq!(errors, 3);
        assert!((checker.ber() - 0.03).abs() < 1e-10);
    }

    #[test]
    fn test_check_bool() {
        let seq: Vec<bool> = generate_reference(5, 0x14, 1, 31)
            .into_iter()
            .map(|b| b != 0)
            .collect();
        let mut checker = LfsrChecker::new(5, 0x14, 1);
        let (bits, errors) = checker.check_bool(&seq);
        assert_eq!(bits, 31);
        assert_eq!(errors, 0);
    }

    #[test]
    fn test_synchronize_aligned() {
        let seq = generate_reference(7, 0x60, 1, 50);
        let mut checker = LfsrChecker::new(7, 0x60, 1);
        let matches = checker.synchronize(&seq);
        assert_eq!(matches, 50); // Perfect alignment
        assert!(checker.is_synchronized());
    }

    #[test]
    fn test_synchronize_offset() {
        // Skip first 10 bits of the sequence
        let full = generate_reference(7, 0x60, 1, 60);
        let offset_data = &full[10..];
        let mut checker = LfsrChecker::new(7, 0x60, 1);
        let matches = checker.synchronize(offset_data);
        assert!(matches >= 45, "should find good alignment, got {matches} matches");
    }

    #[test]
    fn test_cumulative_stats() {
        let seq = generate_reference(5, 0x14, 1, 62);
        let mut checker = LfsrChecker::new(5, 0x14, 1);
        checker.check(&seq[..31]);
        checker.check(&seq[31..]);
        assert_eq!(checker.total_bits(), 62);
        assert_eq!(checker.total_errors(), 0);
    }

    #[test]
    fn test_reset() {
        let seq = generate_reference(5, 0x14, 1, 10);
        let mut checker = LfsrChecker::new(5, 0x14, 1);
        checker.check(&seq);
        checker.reset();
        assert_eq!(checker.total_bits(), 0);
        assert_eq!(checker.total_errors(), 0);
        assert!(!checker.is_synchronized());
    }

    #[test]
    fn test_ber_all_errors() {
        let mut seq = generate_reference(3, 0x6, 1, 7);
        // Flip all bits
        for b in &mut seq {
            *b ^= 1;
        }
        let mut checker = LfsrChecker::new(3, 0x6, 1);
        let (_, errors) = checker.check(&seq);
        assert_eq!(errors, 7);
        assert!((checker.ber() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_empty_check() {
        let mut checker = LfsrChecker::new(5, 0x14, 1);
        let (bits, errors) = checker.check(&[]);
        assert_eq!(bits, 0);
        assert_eq!(errors, 0);
    }

    #[test]
    fn test_generate_balance() {
        // M-sequence of length 2^7-1 should have 64 ones and 63 zeros
        let seq = generate_reference(7, 0x60, 1, 127);
        let ones: usize = seq.iter().map(|&b| b as usize).sum();
        assert_eq!(ones, 64);
    }
}
