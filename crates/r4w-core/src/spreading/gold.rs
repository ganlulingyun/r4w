//! Gold Code Generator
//!
//! Gold codes are a family of spreading codes formed by XORing two
//! m-sequences with preferred polynomial pairs. They are widely used
//! in CDMA systems (GPS, IS-95, WCDMA) due to their favorable
//! cross-correlation properties.
//!
//! ## Properties
//!
//! - **Length**: 2^n - 1 (same as constituent m-sequences)
//! - **Family Size**: 2^n + 1 codes (including the two m-sequences)
//! - **Cross-Correlation**: Bounded to three values: {-1, -t(n), t(n)-2}
//!   where t(n) = 2^((n+1)/2) + 1 for odd n, 2^((n+2)/2) + 1 for even n
//!
//! ## Preferred Pairs
//!
//! Not all m-sequence pairs produce good Gold codes. Preferred pairs
//! must satisfy specific mathematical properties for bounded cross-correlation.
//!
//! ## Example
//!
//! ```rust,ignore
//! use r4w_core::spreading::{GoldCodeGenerator, PnSequence};
//!
//! // Create Gold code generator (degree 7, length 127)
//! let mut gold = GoldCodeGenerator::new(7);
//!
//! // Select code index 5 (0 to 2^n codes available)
//! gold.set_code_index(5);
//!
//! // Generate the sequence
//! let code = gold.generate_sequence();
//! assert_eq!(code.len(), 127);
//! ```

use super::lfsr::{Lfsr, GOLD_PREFERRED_PAIRS};
use super::PnSequence;

/// Gold Code Generator
///
/// Generates Gold codes by XORing two m-sequences from preferred pairs.
#[derive(Debug, Clone)]
pub struct GoldCodeGenerator {
    /// First LFSR (m-sequence A)
    lfsr_a: Lfsr,
    /// Second LFSR (m-sequence B)
    lfsr_b: Lfsr,
    /// Relative phase offset between the two LFSRs (determines which code)
    phase_offset: usize,
    /// Degree (determines sequence length)
    degree: u8,
    /// Initial states for reset
    initial_state_a: u32,
    initial_state_b: u32,
    /// Current position in sequence
    position: usize,
}

impl GoldCodeGenerator {
    /// Create a new Gold code generator with default preferred pair
    ///
    /// # Arguments
    /// - `degree`: LFSR degree (5-10 supported with default pairs)
    pub fn new(degree: u8) -> Self {
        let (_, poly_a, poly_b) = GOLD_PREFERRED_PAIRS
            .iter()
            .find(|(d, _, _)| *d == degree)
            .unwrap_or_else(|| panic!("No preferred pair for degree {}", degree));

        Self::with_polynomials(degree, *poly_a, *poly_b)
    }

    /// Create with custom polynomials
    pub fn with_polynomials(degree: u8, poly_a: u32, poly_b: u32) -> Self {
        Self {
            lfsr_a: Lfsr::new(degree, poly_a, 0x01),
            lfsr_b: Lfsr::new(degree, poly_b, 0x01),
            phase_offset: 0,
            degree,
            initial_state_a: 0x01,
            initial_state_b: 0x01,
            position: 0,
        }
    }

    /// Set the code index (selects which Gold code from the family)
    ///
    /// Index 0 gives m-sequence A, index 1 gives m-sequence B,
    /// indices 2 to 2^n give the XOR combinations.
    pub fn set_code_index(&mut self, index: usize) {
        let seq_len = self.length();
        self.phase_offset = index % (seq_len + 2);
        self.reset();
    }

    /// Get the current code index
    pub fn code_index(&self) -> usize {
        self.phase_offset
    }

    /// Get the number of codes in the family
    pub fn family_size(&self) -> usize {
        (1 << self.degree) + 1
    }

    /// Get the maximum cross-correlation bound t(n)
    pub fn cross_correlation_bound(&self) -> i32 {
        if self.degree % 2 == 1 {
            // Odd n: t(n) = 2^((n+1)/2) + 1
            (1 << ((self.degree + 1) / 2)) + 1
        } else {
            // Even n: t(n) = 2^((n+2)/2) + 1
            (1 << ((self.degree + 2) / 2)) + 1
        }
    }

    /// Generate a specific Gold code by index
    pub fn generate_code(&mut self, index: usize) -> Vec<i8> {
        self.set_code_index(index);
        self.generate_sequence()
    }

    /// Generate all codes in the family
    pub fn generate_family(&mut self) -> Vec<Vec<i8>> {
        (0..self.family_size())
            .map(|i| self.generate_code(i))
            .collect()
    }
}

impl PnSequence for GoldCodeGenerator {
    fn next_chip(&mut self) -> i8 {
        let seq_len = self.length();

        // Special cases: output just one m-sequence
        if self.phase_offset == 0 {
            // M-sequence A only
            let bit_a = self.lfsr_a.clock();
            self.lfsr_b.clock(); // Keep in sync for consistent state
            self.position = (self.position + 1) % seq_len;
            return if bit_a == 0 { 1 } else { -1 };
        } else if self.phase_offset == 1 {
            // M-sequence B only
            self.lfsr_a.clock(); // Keep in sync
            let bit_b = self.lfsr_b.clock();
            self.position = (self.position + 1) % seq_len;
            return if bit_b == 0 { 1 } else { -1 };
        }

        // XOR of two m-sequences with relative phase offset
        let bit_a = self.lfsr_a.clock();
        let bit_b = self.lfsr_b.clock();

        // The phase offset is handled by the initial state of LFSR B
        let xor_bit = bit_a ^ bit_b;

        self.position = (self.position + 1) % seq_len;

        if xor_bit == 0 {
            1
        } else {
            -1
        }
    }

    fn reset(&mut self) {
        // Reset LFSR A to initial state
        self.lfsr_a.set_state(self.initial_state_a);

        // Reset LFSR B and advance by phase offset
        self.lfsr_b.set_state(self.initial_state_b);

        // Advance LFSR B by the phase offset (for codes 2 and above)
        if self.phase_offset >= 2 {
            for _ in 0..(self.phase_offset - 2) {
                self.lfsr_b.clock();
            }
        }

        self.position = 0;
    }

    fn length(&self) -> usize {
        (1 << self.degree) - 1
    }
}

/// Generate balanced Gold codes suitable for a given number of users
pub fn gold_codes_for_users(num_users: usize) -> Option<(u8, GoldCodeGenerator)> {
    // Find smallest degree that provides enough codes
    for (degree, _, _) in GOLD_PREFERRED_PAIRS {
        let family_size = (1 << degree) + 1;
        if family_size >= num_users {
            return Some((*degree, GoldCodeGenerator::new(*degree)));
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spreading::{cross_correlation, max_cross_correlation};

    #[test]
    fn test_gold_code_length() {
        let gold = GoldCodeGenerator::new(7);
        assert_eq!(gold.length(), 127);
        assert_eq!(gold.family_size(), 129); // 2^7 + 1
    }

    #[test]
    fn test_gold_code_generation() {
        let mut gold = GoldCodeGenerator::new(5);
        let code = gold.generate_sequence();

        assert_eq!(code.len(), 31);

        // All chips should be +1 or -1
        assert!(code.iter().all(|&c| c == 1 || c == -1));
    }

    #[test]
    fn test_gold_code_family() {
        let mut gold = GoldCodeGenerator::new(5);
        let family = gold.generate_family();

        // Should have 2^5 + 1 = 33 codes
        assert_eq!(family.len(), 33);

        // All codes should be length 31
        assert!(family.iter().all(|code| code.len() == 31));
    }

    #[test]
    fn test_gold_cross_correlation_bounded() {
        let mut gold = GoldCodeGenerator::new(5);
        let bound = gold.cross_correlation_bound();

        // t(5) = 2^3 + 1 = 9
        assert_eq!(bound, 9);

        // Generate two different codes and check cross-correlation
        let code_a = gold.generate_code(2);
        let code_b = gold.generate_code(3);

        let max_xcorr = max_cross_correlation(&code_a, &code_b);

        // Should be bounded by t(n)
        assert!(max_xcorr <= bound);
    }

    #[test]
    fn test_gold_autocorrelation() {
        let mut gold = GoldCodeGenerator::new(5);
        let code = gold.generate_code(5);

        // Autocorrelation at lag 0 should equal sequence length
        let peak = cross_correlation(&code, &code, 0);
        assert_eq!(peak, 31);
    }

    #[test]
    fn test_different_codes_are_different() {
        let mut gold = GoldCodeGenerator::new(5);
        let code_2 = gold.generate_code(2);
        let code_3 = gold.generate_code(3);

        assert_ne!(code_2, code_3);
    }
}
