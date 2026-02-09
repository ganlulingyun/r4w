//! Additive Scrambler — LFSR-based stream scrambling for DC balance
//!
//! XORs input data with an LFSR-generated PN sequence to achieve
//! spectral whitening and DC balance. Used in DVB, 802.11, Bluetooth,
//! and many other standards. Self-synchronizing: the same scrambler
//! instance also descrambles.
//! GNU Radio equivalent: `additive_scrambler_bb`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::additive_scrambler::AdditiveScrambler;
//!
//! // DVB scrambler: x^15 + x^14 + 1 (taps=0x6000), seed=0x4A80
//! let mut scrambler = AdditiveScrambler::new(15, 0x6000, 0x4A80, 0);
//! let data = vec![0u8, 1, 0, 1, 1, 0, 1, 0];
//! let scrambled = scrambler.process(&data);
//! // Descramble with a fresh scrambler (same params)
//! let mut descrambler = AdditiveScrambler::new(15, 0x6000, 0x4A80, 0);
//! let recovered = descrambler.process(&scrambled);
//! assert_eq!(recovered, data);
//! ```

/// Additive (synchronous) scrambler using Galois LFSR.
#[derive(Debug, Clone)]
pub struct AdditiveScrambler {
    /// Current LFSR state.
    state: u64,
    /// Initial seed for reset.
    seed: u64,
    /// Feedback taps (polynomial).
    taps: u64,
    /// Register length in bits.
    nbits: u32,
    /// Mask for valid bits.
    mask: u64,
    /// Reset period (0 = never reset).
    reset_period: usize,
    /// Counter for auto-reset.
    count: usize,
}

impl AdditiveScrambler {
    /// Create an additive scrambler.
    ///
    /// - `nbits`: LFSR register length (1-63)
    /// - `taps`: feedback polynomial
    /// - `seed`: initial LFSR state (non-zero for operation)
    /// - `reset_period`: auto-reset LFSR every N bits (0 = never)
    pub fn new(nbits: u32, taps: u64, seed: u64, reset_period: usize) -> Self {
        let nbits = nbits.clamp(1, 63);
        let mask = (1u64 << nbits) - 1;
        Self {
            state: (seed & mask).max(1),
            seed: (seed & mask).max(1),
            taps: taps & mask,
            nbits,
            mask,
            reset_period,
            count: 0,
        }
    }

    /// Create a DVB scrambler (x^15 + x^14 + 1).
    pub fn dvb() -> Self {
        Self::new(15, 0x6000, 0x4A80, 0)
    }

    /// Create an 802.11 scrambler (x^7 + x^4 + 1).
    pub fn wifi(seed: u8) -> Self {
        Self::new(7, 0x48, (seed & 0x7F) as u64, 0)
    }

    /// Generate next LFSR bit and advance state.
    #[inline]
    fn next_bit(&mut self) -> u8 {
        let output = (self.state & 1) as u8;
        let feedback = self.state & 1;
        self.state >>= 1;
        if feedback != 0 {
            self.state ^= self.taps;
        }
        self.state &= self.mask;
        output
    }

    /// Scramble/descramble a single bit (XOR with LFSR output).
    #[inline]
    pub fn process_bit(&mut self, bit: u8) -> u8 {
        if self.reset_period > 0 {
            if self.count >= self.reset_period {
                self.state = self.seed;
                self.count = 0;
            }
            self.count += 1;
        }
        bit ^ self.next_bit()
    }

    /// Scramble/descramble a block of bits.
    pub fn process(&mut self, input: &[u8]) -> Vec<u8> {
        input.iter().map(|&b| self.process_bit(b)).collect()
    }

    /// Scramble/descramble boolean bits.
    pub fn process_bool(&mut self, input: &[bool]) -> Vec<bool> {
        input
            .iter()
            .map(|&b| self.process_bit(b as u8) != 0)
            .collect()
    }

    /// In-place scrambling.
    pub fn process_inplace(&mut self, data: &mut [u8]) {
        for b in data.iter_mut() {
            *b = self.process_bit(*b);
        }
    }

    /// Reset LFSR to initial seed.
    pub fn reset(&mut self) {
        self.state = self.seed;
        self.count = 0;
    }

    /// Get current state.
    pub fn state(&self) -> u64 {
        self.state
    }

    /// Get register length.
    pub fn nbits(&self) -> u32 {
        self.nbits
    }

    /// Get reset period.
    pub fn reset_period(&self) -> usize {
        self.reset_period
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_roundtrip() {
        let mut scr = AdditiveScrambler::new(7, 0x48, 1, 0);
        let data: Vec<u8> = vec![0, 1, 0, 1, 1, 0, 1, 0, 0, 1];
        let scrambled = scr.process(&data);
        let mut descr = AdditiveScrambler::new(7, 0x48, 1, 0);
        let recovered = descr.process(&scrambled);
        assert_eq!(recovered, data);
    }

    #[test]
    fn test_dvb_roundtrip() {
        let mut scr = AdditiveScrambler::dvb();
        let data: Vec<u8> = (0..100).map(|i| (i % 2) as u8).collect();
        let scrambled = scr.process(&data);
        assert_ne!(scrambled, data);
        let mut descr = AdditiveScrambler::dvb();
        let recovered = descr.process(&scrambled);
        assert_eq!(recovered, data);
    }

    #[test]
    fn test_wifi_roundtrip() {
        let mut scr = AdditiveScrambler::wifi(0x5B);
        let data: Vec<u8> = vec![1, 0, 1, 1, 0, 0, 1, 0];
        let scrambled = scr.process(&data);
        let mut descr = AdditiveScrambler::wifi(0x5B);
        let recovered = descr.process(&scrambled);
        assert_eq!(recovered, data);
    }

    #[test]
    fn test_scrambles_data() {
        let mut scr = AdditiveScrambler::new(7, 0x48, 1, 0);
        let data: Vec<u8> = vec![0; 20];
        let scrambled = scr.process(&data);
        let ones: usize = scrambled.iter().map(|&b| b as usize).sum();
        assert!(ones > 0, "scrambler should produce some 1s from all-zeros");
    }

    #[test]
    fn test_bool_roundtrip() {
        let mut scr = AdditiveScrambler::new(5, 0x14, 1, 0);
        let data: Vec<bool> = vec![false, true, true, false, true, false, false, true];
        let scrambled = scr.process_bool(&data);
        let mut descr = AdditiveScrambler::new(5, 0x14, 1, 0);
        let recovered = descr.process_bool(&scrambled);
        assert_eq!(recovered, data);
    }

    #[test]
    fn test_inplace() {
        let mut scr1 = AdditiveScrambler::new(7, 0x48, 1, 0);
        let mut scr2 = AdditiveScrambler::new(7, 0x48, 1, 0);
        let data: Vec<u8> = vec![0, 1, 0, 1, 1, 0];
        let expected = scr1.process(&data);
        let mut actual = data.clone();
        scr2.process_inplace(&mut actual);
        assert_eq!(actual, expected);
    }

    #[test]
    fn test_reset() {
        let mut scr = AdditiveScrambler::new(7, 0x48, 1, 0);
        let data: Vec<u8> = vec![0, 1, 0, 1];
        let out1 = scr.process(&data);
        scr.reset();
        let out2 = scr.process(&data);
        assert_eq!(out1, out2);
    }

    #[test]
    fn test_reset_period() {
        let mut scr1 = AdditiveScrambler::new(5, 0x14, 1, 4);
        let mut scr2 = AdditiveScrambler::new(5, 0x14, 1, 4);
        let data: Vec<u8> = vec![0; 12];
        let scrambled = scr1.process(&data);
        let recovered = scr2.process(&scrambled);
        assert_eq!(recovered, data);
    }

    #[test]
    fn test_accessors() {
        let scr = AdditiveScrambler::new(15, 0x6000, 0x4A80, 100);
        assert_eq!(scr.nbits(), 15);
        assert_eq!(scr.reset_period(), 100);
    }

    #[test]
    fn test_empty() {
        let mut scr = AdditiveScrambler::dvb();
        assert!(scr.process(&[]).is_empty());
    }

    #[test]
    fn test_spectral_whitening() {
        // All-ones input should produce roughly balanced output
        let mut scr = AdditiveScrambler::new(10, 0x240, 1, 0);
        let data: Vec<u8> = vec![1; 1023];
        let scrambled = scr.process(&data);
        let ones: usize = scrambled.iter().map(|&b| b as usize).sum();
        // M-sequence has (2^n)/2 ones and (2^n)/2-1 zeros in one period
        assert!(ones > 400 && ones < 623, "expected ~balanced output, got {} ones", ones);
    }

    #[test]
    fn test_different_seeds() {
        let mut scr1 = AdditiveScrambler::new(7, 0x48, 1, 0);
        let mut scr2 = AdditiveScrambler::new(7, 0x48, 42, 0);
        let data: Vec<u8> = vec![0; 10];
        assert_ne!(scr1.process(&data), scr2.process(&data));
    }
}
