//! LFSR Scrambler / Descrambler
//!
//! Additive and multiplicative LFSR-based scrambling for standard
//! communications protocols (DVB-S2, IEEE 802.11, Bluetooth, V.34, etc.).
//!
//! Scrambling randomizes bit patterns to ensure DC balance and sufficient
//! transitions for clock recovery, without expanding the data.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::scrambler::{Scrambler, ScramblerConfig, ScramblerType};
//!
//! // DVB-S2 scrambler: x^14 + x^13 + 1
//! let config = ScramblerConfig::dvb_s2();
//! let mut scrambler = Scrambler::new(config.clone());
//! let mut descrambler = Scrambler::new(config);
//!
//! let data = vec![0u8, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0];
//! let scrambled = scrambler.scramble_bits(&data);
//! let recovered = descrambler.descramble_bits(&scrambled);
//! assert_eq!(data, recovered);
//! ```

/// Scrambler type: additive or multiplicative.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ScramblerType {
    /// Additive (synchronous): output = input XOR LFSR_output.
    /// Same operation for scramble and descramble.
    Additive,
    /// Multiplicative (self-synchronizing): scrambler feeds back output,
    /// descrambler feeds back input.
    Multiplicative,
}

/// Configuration for the LFSR scrambler.
#[derive(Debug, Clone)]
pub struct ScramblerConfig {
    /// Feedback polynomial mask (taps). Bit positions indicate taps.
    /// E.g., x^7 + x^4 + 1 → 0b1001_0001 = 0x91 (bits 7, 4, 0).
    pub mask: u64,
    /// LFSR register length in bits.
    pub length: u32,
    /// Initial seed (must be nonzero for multiplicative).
    pub seed: u64,
    /// Scrambler type.
    pub scrambler_type: ScramblerType,
    /// Optional reset period (reset LFSR every N bits). 0 = never.
    pub reset_period: usize,
}

impl ScramblerConfig {
    /// DVB-S2 scrambler: x^14 + x^13 + 1, seed = 0x1.
    pub fn dvb_s2() -> Self {
        Self {
            mask: (1 << 14) | (1 << 13) | 1, // x^14 + x^13 + 1
            length: 15,
            seed: 0x0001,
            scrambler_type: ScramblerType::Additive,
            reset_period: 0,
        }
    }

    /// IEEE 802.11 scrambler: x^7 + x^4 + 1, all-ones seed.
    pub fn wifi() -> Self {
        Self {
            mask: (1 << 7) | (1 << 4) | 1, // x^7 + x^4 + 1
            length: 7,
            seed: 0x7F,
            scrambler_type: ScramblerType::Additive,
            reset_period: 0,
        }
    }

    /// V.34 modem scrambler: x^23 + x^18 + 1, multiplicative.
    pub fn v34() -> Self {
        Self {
            mask: (1 << 23) | (1 << 18) | 1,
            length: 23,
            seed: 0x000001,
            scrambler_type: ScramblerType::Multiplicative,
            reset_period: 0,
        }
    }

    /// Bluetooth whitening: x^7 + x^4 + 1 (same polynomial as WiFi).
    /// Seed derived from channel index.
    pub fn bluetooth(channel: u8) -> Self {
        Self {
            mask: (1 << 7) | (1 << 4) | 1,
            length: 7,
            seed: (1 << 6) | (channel as u64 & 0x3F), // bit 6 set + channel
            scrambler_type: ScramblerType::Additive,
            reset_period: 0,
        }
    }

    /// Simple LFSR for testing: x^3 + x^2 + 1.
    pub fn simple() -> Self {
        Self {
            mask: (1 << 3) | (1 << 2) | 1,
            length: 3,
            seed: 0x07,
            scrambler_type: ScramblerType::Additive,
            reset_period: 0,
        }
    }
}

/// LFSR-based scrambler/descrambler.
///
/// For additive scramblers, scramble and descramble are the same operation
/// (XOR with LFSR output). For multiplicative scramblers, the feedback
/// differs between scrambler and descrambler.
#[derive(Debug, Clone)]
pub struct Scrambler {
    config: ScramblerConfig,
    /// Current LFSR state.
    state: u64,
    /// Bit counter for reset_period.
    bit_count: usize,
    /// State mask (all valid bits set).
    state_mask: u64,
}

impl Scrambler {
    /// Create a new scrambler.
    pub fn new(config: ScramblerConfig) -> Self {
        let state_mask = (1u64 << config.length) - 1;
        let state = config.seed & state_mask;
        Self {
            config,
            state,
            bit_count: 0,
            state_mask,
        }
    }

    /// Clock the LFSR one step and return the output bit.
    fn clock(&mut self) -> u8 {
        // XOR all tapped bits (parity of state AND mask)
        let feedback_bits = self.state & self.config.mask;
        let feedback = feedback_bits.count_ones() & 1;

        // Output is the MSB
        let output = ((self.state >> (self.config.length - 1)) & 1) as u8;

        // Shift left and insert feedback
        self.state = ((self.state << 1) | feedback as u64) & self.state_mask;

        output
    }

    /// Scramble a single bit (additive mode).
    fn scramble_bit_additive(&mut self, input: u8) -> u8 {
        let lfsr_out = self.clock();
        (input & 1) ^ lfsr_out
    }

    /// Scramble a single bit (multiplicative mode).
    fn scramble_bit_multiplicative(&mut self, input: u8) -> u8 {
        let output = (input & 1) ^ (((self.state >> (self.config.length - 1)) & 1) as u8);

        // Feed output back into LFSR
        let feedback_bits = self.state & self.config.mask;
        let feedback = feedback_bits.count_ones() & 1;
        self.state = ((self.state << 1) | (output as u64)) & self.state_mask;
        let _ = feedback; // multiplicative uses output, not parity feedback

        output
    }

    /// Descramble a single bit (multiplicative mode).
    fn descramble_bit_multiplicative(&mut self, input: u8) -> u8 {
        let output = (input & 1) ^ (((self.state >> (self.config.length - 1)) & 1) as u8);

        // Feed input (not output) back into LFSR
        self.state = ((self.state << 1) | ((input & 1) as u64)) & self.state_mask;

        output
    }

    /// Scramble a sequence of bits.
    pub fn scramble_bits(&mut self, input: &[u8]) -> Vec<u8> {
        let mut output = Vec::with_capacity(input.len());
        for &bit in input {
            if self.config.reset_period > 0 && self.bit_count >= self.config.reset_period {
                self.reset();
            }
            let out = match self.config.scrambler_type {
                ScramblerType::Additive => self.scramble_bit_additive(bit),
                ScramblerType::Multiplicative => self.scramble_bit_multiplicative(bit),
            };
            output.push(out);
            self.bit_count += 1;
        }
        output
    }

    /// Descramble a sequence of bits.
    pub fn descramble_bits(&mut self, input: &[u8]) -> Vec<u8> {
        let mut output = Vec::with_capacity(input.len());
        for &bit in input {
            if self.config.reset_period > 0 && self.bit_count >= self.config.reset_period {
                self.reset();
            }
            let out = match self.config.scrambler_type {
                ScramblerType::Additive => self.scramble_bit_additive(bit),
                ScramblerType::Multiplicative => self.descramble_bit_multiplicative(bit),
            };
            output.push(out);
            self.bit_count += 1;
        }
        output
    }

    /// Scramble a byte array (MSB first per byte).
    pub fn scramble_bytes(&mut self, input: &[u8]) -> Vec<u8> {
        let mut output = Vec::with_capacity(input.len());
        for &byte in input {
            let mut out_byte = 0u8;
            for bit_pos in (0..8).rev() {
                let bit = (byte >> bit_pos) & 1;
                let scrambled = match self.config.scrambler_type {
                    ScramblerType::Additive => self.scramble_bit_additive(bit),
                    ScramblerType::Multiplicative => self.scramble_bit_multiplicative(bit),
                };
                out_byte |= scrambled << bit_pos;
                self.bit_count += 1;
                if self.config.reset_period > 0 && self.bit_count >= self.config.reset_period {
                    self.reset();
                }
            }
            output.push(out_byte);
        }
        output
    }

    /// Descramble a byte array (MSB first per byte).
    pub fn descramble_bytes(&mut self, input: &[u8]) -> Vec<u8> {
        let mut output = Vec::with_capacity(input.len());
        for &byte in input {
            let mut out_byte = 0u8;
            for bit_pos in (0..8).rev() {
                let bit = (byte >> bit_pos) & 1;
                let descrambled = match self.config.scrambler_type {
                    ScramblerType::Additive => self.scramble_bit_additive(bit),
                    ScramblerType::Multiplicative => self.descramble_bit_multiplicative(bit),
                };
                out_byte |= descrambled << bit_pos;
                self.bit_count += 1;
                if self.config.reset_period > 0 && self.bit_count >= self.config.reset_period {
                    self.reset();
                }
            }
            output.push(out_byte);
        }
        output
    }

    /// Generate the LFSR sequence (for analysis/testing).
    pub fn generate_sequence(&mut self, length: usize) -> Vec<u8> {
        let mut seq = Vec::with_capacity(length);
        for _ in 0..length {
            seq.push(self.clock());
        }
        seq
    }

    /// Reset to initial seed.
    pub fn reset(&mut self) {
        self.state = self.config.seed & self.state_mask;
        self.bit_count = 0;
    }

    /// Get the current LFSR state.
    pub fn state(&self) -> u64 {
        self.state
    }

    /// Get the LFSR period (2^length - 1 for maximal-length).
    pub fn max_period(&self) -> u64 {
        (1u64 << self.config.length) - 1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_additive_roundtrip() {
        let config = ScramblerConfig::wifi();
        let mut scrambler = Scrambler::new(config.clone());
        let mut descrambler = Scrambler::new(config);

        let data: Vec<u8> = (0..100).map(|i| (i % 2) as u8).collect();
        let scrambled = scrambler.scramble_bits(&data);
        let recovered = descrambler.descramble_bits(&scrambled);

        assert_eq!(data, recovered, "Additive scramble/descramble roundtrip failed");
    }

    #[test]
    fn test_scrambler_changes_data() {
        let mut scrambler = Scrambler::new(ScramblerConfig::wifi());
        let data: Vec<u8> = vec![0; 50];
        let scrambled = scrambler.scramble_bits(&data);

        // Scrambled data should differ from all-zeros
        assert!(
            scrambled.iter().any(|&b| b != 0),
            "Scrambler should change all-zeros input"
        );
    }

    #[test]
    fn test_dvb_s2_roundtrip() {
        let config = ScramblerConfig::dvb_s2();
        let mut scrambler = Scrambler::new(config.clone());
        let mut descrambler = Scrambler::new(config);

        let data: Vec<u8> = (0..200).map(|i| ((i * 7 + 3) % 2) as u8).collect();
        let scrambled = scrambler.scramble_bits(&data);
        let recovered = descrambler.descramble_bits(&scrambled);

        assert_eq!(data, recovered);
    }

    #[test]
    fn test_byte_roundtrip() {
        let config = ScramblerConfig::wifi();
        let mut scrambler = Scrambler::new(config.clone());
        let mut descrambler = Scrambler::new(config);

        let data = b"Hello World!";
        let scrambled = scrambler.scramble_bytes(data);
        let recovered = descrambler.descramble_bytes(&scrambled);

        assert_eq!(data.as_slice(), &recovered[..]);
    }

    #[test]
    fn test_bluetooth_channel() {
        let config = ScramblerConfig::bluetooth(37); // Advertising channel
        let mut scrambler = Scrambler::new(config.clone());
        let mut descrambler = Scrambler::new(config);

        let data: Vec<u8> = (0..80).map(|i| (i % 2) as u8).collect();
        let scrambled = scrambler.scramble_bits(&data);
        let recovered = descrambler.descramble_bits(&scrambled);

        assert_eq!(data, recovered);
    }

    #[test]
    fn test_lfsr_period() {
        // Simple 3-bit LFSR should have period 7 (2^3 - 1)
        let mut scrambler = Scrambler::new(ScramblerConfig::simple());
        let seq = scrambler.generate_sequence(20);

        // Check that the sequence repeats with period 7
        for i in 0..7 {
            assert_eq!(
                seq[i], seq[i + 7],
                "Maximal-length LFSR should repeat with period 7"
            );
        }
    }

    #[test]
    fn test_reset() {
        let config = ScramblerConfig::wifi();
        let mut scrambler = Scrambler::new(config);

        let data: Vec<u8> = (0..50).map(|i| (i % 2) as u8).collect();
        let run1 = scrambler.scramble_bits(&data);
        scrambler.reset();
        let run2 = scrambler.scramble_bits(&data);

        assert_eq!(run1, run2, "Reset should produce identical output");
    }

    #[test]
    fn test_reset_period() {
        let config = ScramblerConfig {
            reset_period: 10,
            ..ScramblerConfig::simple()
        };
        let mut scrambler = Scrambler::new(config.clone());
        let mut descrambler = Scrambler::new(config);

        let data: Vec<u8> = (0..30).map(|i| (i % 2) as u8).collect();
        let scrambled = scrambler.scramble_bits(&data);
        let recovered = descrambler.descramble_bits(&scrambled);

        assert_eq!(data, recovered, "Reset period roundtrip failed");
    }

    #[test]
    fn test_multiplicative_roundtrip() {
        let config = ScramblerConfig::v34();
        let mut scrambler = Scrambler::new(config.clone());
        let mut descrambler = Scrambler::new(config);

        let data: Vec<u8> = (0..100).map(|i| ((i * 3 + 1) % 2) as u8).collect();
        let scrambled = scrambler.scramble_bits(&data);
        let recovered = descrambler.descramble_bits(&scrambled);

        assert_eq!(data, recovered, "Multiplicative scramble/descramble roundtrip failed");
    }

    #[test]
    fn test_different_seeds_different_output() {
        let mut s1 = Scrambler::new(ScramblerConfig {
            seed: 0x7F,
            ..ScramblerConfig::wifi()
        });
        let mut s2 = Scrambler::new(ScramblerConfig {
            seed: 0x01,
            ..ScramblerConfig::wifi()
        });

        let data: Vec<u8> = vec![0; 50];
        let out1 = s1.scramble_bits(&data);
        let out2 = s2.scramble_bits(&data);

        assert_ne!(out1, out2, "Different seeds should produce different output");
    }
}
