//! GLFSR Source — Galois Linear Feedback Shift Register PN generator
//!
//! Generates pseudorandom bit sequences using a Galois LFSR. Produces
//! maximal-length sequences (m-sequences) for known polynomial taps.
//! Used in spread spectrum, scrambling, test patterns, and PN code
//! generation.
//! GNU Radio equivalent: `glfsr_source_b`, `glfsr_source_f`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::glfsr_source::GaloisLfsr;
//!
//! // 7-bit LFSR with polynomial x^7 + x^6 + 1 (taps=0xC0)
//! let mut lfsr = GaloisLfsr::new(7, 0xC0, 1);
//! let bits: Vec<u8> = (0..127).map(|_| lfsr.next_bit()).collect();
//! assert_eq!(bits.len(), 127);
//! // M-sequence has period 2^n - 1 = 127
//! ```

/// Galois Linear Feedback Shift Register.
#[derive(Debug, Clone)]
pub struct GaloisLfsr {
    /// Current register state.
    state: u64,
    /// Feedback taps (polynomial).
    taps: u64,
    /// Register length in bits.
    nbits: u32,
    /// Mask for valid bits.
    mask: u64,
}

impl GaloisLfsr {
    /// Create a GLFSR.
    ///
    /// - `nbits`: register length (1-63)
    /// - `taps`: feedback polynomial (bit positions that are tapped)
    /// - `seed`: initial state (must be non-zero for maximal sequence)
    pub fn new(nbits: u32, taps: u64, seed: u64) -> Self {
        let nbits = nbits.clamp(1, 63);
        let mask = (1u64 << nbits) - 1;
        Self {
            state: (seed & mask).max(1), // Ensure non-zero
            taps: taps & mask,
            nbits,
            mask,
        }
    }

    /// Create with standard maximal-length polynomial.
    ///
    /// Common LFSR polynomials for maximal-length sequences:
    /// - 3: x³+x+1, 5: x⁵+x²+1, 7: x⁷+x⁶+1, 9: x⁹+x⁴+1
    /// - 10: x¹⁰+x³+1, 15: x¹⁵+x+1, 16: x¹⁶+x¹⁴+x¹³+x¹¹+1
    /// - 20: x²⁰+x³+1, 23: x²³+x⁵+1, 31: x³¹+x³+1
    pub fn maximal(nbits: u32) -> Self {
        let taps = match nbits {
            2 => 0x3,
            3 => 0x6,
            4 => 0xC,
            5 => 0x14,
            6 => 0x30,
            7 => 0x60,
            8 => 0xB8,
            9 => 0x110,
            10 => 0x240,
            11 => 0x500,
            12 => 0xE08,
            13 => 0x1C80,
            14 => 0x3802,
            15 => 0x6000,
            16 => 0xD008,
            17 => 0x12000,
            18 => 0x20400,
            19 => 0x72000,
            20 => 0x90000,
            21 => 0x140000,
            22 => 0x300000,
            23 => 0x420000,
            24 => 0xE10000,
            25 => 0x1000004,
            31 => 0x48000000,
            _ => {
                // Fallback: use simple taps
                let mask = (1u64 << nbits) - 1;
                (1u64 << (nbits - 1)) | 1 & mask
            }
        };
        Self::new(nbits, taps, 1)
    }

    /// Generate next output bit (0 or 1).
    #[inline]
    pub fn next_bit(&mut self) -> u8 {
        let output = (self.state & 1) as u8;
        let feedback = self.state & 1;
        self.state >>= 1;
        if feedback != 0 {
            self.state ^= self.taps;
        }
        self.state &= self.mask;
        output
    }

    /// Generate N bits.
    pub fn generate_bits(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| self.next_bit()).collect()
    }

    /// Generate N bits as booleans.
    pub fn generate_bool(&mut self, n: usize) -> Vec<bool> {
        (0..n).map(|_| self.next_bit() != 0).collect()
    }

    /// Generate N bipolar samples (+1.0 / -1.0).
    pub fn generate_bipolar(&mut self, n: usize) -> Vec<f64> {
        (0..n)
            .map(|_| if self.next_bit() != 0 { 1.0 } else { -1.0 })
            .collect()
    }

    /// Get current state.
    pub fn state(&self) -> u64 {
        self.state
    }

    /// Set state.
    pub fn set_state(&mut self, state: u64) {
        self.state = (state & self.mask).max(1);
    }

    /// Get register length.
    pub fn nbits(&self) -> u32 {
        self.nbits
    }

    /// Get feedback taps.
    pub fn taps(&self) -> u64 {
        self.taps
    }

    /// Get maximal sequence length (2^n - 1).
    pub fn period(&self) -> u64 {
        (1u64 << self.nbits) - 1
    }

    /// Reset to initial seed.
    pub fn reset(&mut self, seed: u64) {
        self.state = (seed & self.mask).max(1);
    }
}

/// Fibonacci LFSR (alternative implementation, matches some standards).
#[derive(Debug, Clone)]
pub struct FibonacciLfsr {
    state: u64,
    taps: u64,
    nbits: u32,
    mask: u64,
}

impl FibonacciLfsr {
    /// Create a Fibonacci LFSR.
    pub fn new(nbits: u32, taps: u64, seed: u64) -> Self {
        let nbits = nbits.clamp(1, 63);
        let mask = (1u64 << nbits) - 1;
        Self {
            state: (seed & mask).max(1),
            taps: taps & mask,
            nbits,
            mask,
        }
    }

    /// Generate next bit.
    #[inline]
    pub fn next_bit(&mut self) -> u8 {
        let output = ((self.state >> (self.nbits - 1)) & 1) as u8;
        let feedback = (self.state & self.taps).count_ones() & 1;
        self.state = ((self.state << 1) | feedback as u64) & self.mask;
        output
    }

    /// Generate N bits.
    pub fn generate_bits(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| self.next_bit()).collect()
    }

    /// Get state.
    pub fn state(&self) -> u64 {
        self.state
    }

    /// Reset.
    pub fn reset(&mut self, seed: u64) {
        self.state = (seed & self.mask).max(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_output() {
        let mut lfsr = GaloisLfsr::new(3, 0x6, 1);
        let bits = lfsr.generate_bits(7);
        assert_eq!(bits.len(), 7);
        // All bits should be 0 or 1
        for &b in &bits {
            assert!(b <= 1);
        }
    }

    #[test]
    fn test_maximal_length_7() {
        // 7-bit LFSR should have period 127
        let mut lfsr = GaloisLfsr::maximal(7);
        let initial = lfsr.state();
        let mut period = 0;
        loop {
            lfsr.next_bit();
            period += 1;
            if lfsr.state() == initial || period > 200 {
                break;
            }
        }
        assert_eq!(period, 127, "expected period 127 for 7-bit LFSR");
    }

    #[test]
    fn test_maximal_length_5() {
        let mut lfsr = GaloisLfsr::maximal(5);
        let initial = lfsr.state();
        let mut period = 0;
        loop {
            lfsr.next_bit();
            period += 1;
            if lfsr.state() == initial || period > 50 {
                break;
            }
        }
        assert_eq!(period, 31);
    }

    #[test]
    fn test_bipolar() {
        let mut lfsr = GaloisLfsr::maximal(5);
        let samples = lfsr.generate_bipolar(31);
        for &s in &samples {
            assert!(s == 1.0 || s == -1.0);
        }
    }

    #[test]
    fn test_bool_output() {
        let mut lfsr = GaloisLfsr::maximal(3);
        let bits = lfsr.generate_bool(7);
        assert_eq!(bits.len(), 7);
    }

    #[test]
    fn test_balance() {
        // M-sequence should have one more 1 than 0
        let mut lfsr = GaloisLfsr::maximal(7);
        let bits = lfsr.generate_bits(127);
        let ones: usize = bits.iter().map(|&b| b as usize).sum();
        assert_eq!(ones, 64, "127-bit m-sequence should have 64 ones");
    }

    #[test]
    fn test_reset() {
        let mut lfsr = GaloisLfsr::maximal(5);
        let bits1 = lfsr.generate_bits(10);
        lfsr.reset(1);
        let bits2 = lfsr.generate_bits(10);
        assert_eq!(bits1, bits2);
    }

    #[test]
    fn test_different_seeds() {
        let mut lfsr1 = GaloisLfsr::new(7, 0x60, 1);
        let mut lfsr2 = GaloisLfsr::new(7, 0x60, 42);
        let bits1 = lfsr1.generate_bits(10);
        let bits2 = lfsr2.generate_bits(10);
        assert_ne!(bits1, bits2);
    }

    #[test]
    fn test_accessors() {
        let lfsr = GaloisLfsr::new(10, 0x240, 1);
        assert_eq!(lfsr.nbits(), 10);
        assert_eq!(lfsr.taps(), 0x240);
        assert_eq!(lfsr.period(), 1023);
    }

    #[test]
    fn test_set_state() {
        let mut lfsr = GaloisLfsr::maximal(5);
        lfsr.set_state(15);
        assert_eq!(lfsr.state(), 15);
    }

    #[test]
    fn test_fibonacci_basic() {
        let mut lfsr = FibonacciLfsr::new(3, 0x6, 1);
        let bits = lfsr.generate_bits(7);
        assert_eq!(bits.len(), 7);
    }

    #[test]
    fn test_fibonacci_reset() {
        let mut lfsr = FibonacciLfsr::new(5, 0x14, 1);
        let bits1 = lfsr.generate_bits(10);
        lfsr.reset(1);
        let bits2 = lfsr.generate_bits(10);
        assert_eq!(bits1, bits2);
    }

    #[test]
    fn test_zero_seed_protection() {
        // Zero seed should be bumped to 1
        let lfsr = GaloisLfsr::new(5, 0x14, 0);
        assert_eq!(lfsr.state(), 1);
    }

    #[test]
    fn test_nonzero_output() {
        // Verify the LFSR produces at least some 1s
        let mut lfsr = GaloisLfsr::maximal(10);
        let bits = lfsr.generate_bits(100);
        let ones: usize = bits.iter().map(|&b| b as usize).sum();
        assert!(ones > 0, "LFSR should produce some 1 bits");
        assert!(ones < 100, "LFSR should produce some 0 bits");
    }
}
