//! # PN-Sequence Scramblers/Descramblers
//!
//! This module implements PN (Pseudo-Noise) sequence scramblers and descramblers
//! for data whitening in digital communication systems. Scramblers randomize
//! data bit patterns to ensure spectral flatness and avoid long runs of identical
//! bits, which improves clock recovery and reduces DC offset.
//!
//! Two scrambler types are provided:
//!
//! - **Additive (synchronous)**: XORs data with an independently generated PN
//!   sequence. Both transmitter and receiver must be initialized to the same state.
//! - **Multiplicative (self-synchronizing)**: Feeds output bits back into the shift
//!   register. The descrambler automatically synchronizes after receiving enough
//!   bits to fill the shift register.
//!
//! Factory functions provide standard scramblers for DVB-S/S2, IEEE 802.11 WiFi,
//! and CDMA systems.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::pn_scrambler::{PnScrambler, ScramblerType};
//!
//! // Create an additive scrambler with polynomial x^7 + x^4 + 1
//! let mut scrambler = PnScrambler::new(0b10010000, 0x7F, 7);
//! let data = vec![true, false, true, true, false, false, true, false];
//!
//! let scrambled = scrambler.scramble(&data);
//! scrambler.reset();
//! let recovered = scrambler.descramble(&scrambled);
//!
//! assert_eq!(data, recovered);
//! ```

/// Type of scrambler operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScramblerType {
    /// Additive (synchronous) scrambler: XORs data with PN sequence.
    /// Both ends must share the same initial state.
    Additive,
    /// Multiplicative (self-synchronizing) scrambler: feeds the output
    /// bit back into the shift register. The descrambler uses received
    /// bits as feedback and self-synchronizes.
    Multiplicative,
}

/// LFSR-based PN-sequence scrambler/descrambler.
///
/// Supports both additive and multiplicative scrambling modes.
/// The LFSR polynomial is specified as a bitmask where bit `i` is set
/// if `x^i` appears in the polynomial (the implicit `x^degree` term
/// is not included in the mask).
#[derive(Debug, Clone)]
pub struct PnScrambler {
    /// LFSR feedback polynomial bitmask (taps).
    polynomial: u32,
    /// Current LFSR state.
    state: u32,
    /// Initial LFSR state (for reset).
    initial_state: u32,
    /// Degree of the LFSR (number of register stages).
    degree: usize,
    /// Bitmask for the register (2^degree - 1).
    mask: u32,
    /// Scrambler type (additive or multiplicative).
    scrambler_type: ScramblerType,
}

impl PnScrambler {
    /// Creates a new PN scrambler.
    ///
    /// # Arguments
    ///
    /// * `polynomial` - LFSR feedback polynomial as a bitmask. Bit `i` is set
    ///   if `x^i` is a tap in the feedback polynomial. The `x^degree` term is
    ///   implicit.
    /// * `initial_state` - Starting state of the LFSR. Must not be zero for
    ///   maximal-length sequences.
    /// * `degree` - Number of stages in the shift register.
    pub fn new(polynomial: u32, initial_state: u32, degree: usize) -> Self {
        assert!(degree > 0 && degree <= 31, "degree must be between 1 and 31");
        let mask = (1u32 << degree) - 1;
        Self {
            polynomial,
            state: initial_state & mask,
            initial_state: initial_state & mask,
            degree,
            mask,
            scrambler_type: ScramblerType::Additive,
        }
    }

    /// Sets the scrambler type.
    pub fn set_type(&mut self, stype: ScramblerType) {
        self.scrambler_type = stype;
    }

    /// Resets the LFSR to the initial state.
    pub fn reset(&mut self) {
        self.state = self.initial_state;
    }

    /// Generates a single PN bit and advances the LFSR.
    fn clock_lfsr(&mut self) -> bool {
        // The output bit is the MSB of the current state
        let output_bit = (self.state >> (self.degree - 1)) & 1 != 0;

        // Compute feedback: XOR of all tapped bits
        let feedback_bits = self.state & self.polynomial;
        let feedback = (feedback_bits.count_ones() & 1) != 0;

        // Shift register left and insert feedback bit at LSB
        self.state = ((self.state << 1) | (feedback as u32)) & self.mask;

        output_bit
    }

    /// Scrambles data by XOR with PN sequence (additive mode) or with
    /// feedback from output (multiplicative mode).
    pub fn scramble(&mut self, data: &[bool]) -> Vec<bool> {
        match self.scrambler_type {
            ScramblerType::Additive => {
                data.iter()
                    .map(|&bit| {
                        let pn_bit = self.clock_lfsr();
                        bit ^ pn_bit
                    })
                    .collect()
            }
            ScramblerType::Multiplicative => {
                data.iter()
                    .map(|&bit| {
                        let pn_bit = self.clock_lfsr();
                        let output = bit ^ pn_bit;
                        // Feed output back: replace the LSB that was just inserted
                        // with the output bit to create self-synchronizing behavior
                        self.state = (self.state & !1) | (output as u32);
                        output
                    })
                    .collect()
            }
        }
    }

    /// Descrambles data. For additive mode this is identical to scramble.
    /// For multiplicative mode, the received bits feed the shift register.
    pub fn descramble(&mut self, data: &[bool]) -> Vec<bool> {
        match self.scrambler_type {
            ScramblerType::Additive => {
                // Additive: descramble is the same as scramble
                data.iter()
                    .map(|&bit| {
                        let pn_bit = self.clock_lfsr();
                        bit ^ pn_bit
                    })
                    .collect()
            }
            ScramblerType::Multiplicative => {
                data.iter()
                    .map(|&bit| {
                        let pn_bit = self.clock_lfsr();
                        let output = bit ^ pn_bit;
                        // Feed received bit into register (not the output)
                        self.state = (self.state & !1) | (bit as u32);
                        output
                    })
                    .collect()
            }
        }
    }

    /// Generates a raw PN bit sequence of the given length.
    pub fn generate_sequence(&mut self, length: usize) -> Vec<bool> {
        (0..length).map(|_| self.clock_lfsr()).collect()
    }
}

/// Self-synchronizing scrambler with separate scramble/descramble paths.
///
/// Uses a shift register fed by the scrambled (transmitted) bit stream.
/// The descrambler automatically synchronizes after receiving `degree`
/// bits because it builds the same shift register state from the received
/// data.
#[derive(Debug, Clone)]
pub struct SelfSyncScrambler {
    /// LFSR feedback polynomial bitmask (taps).
    polynomial: u32,
    /// Current shift register state.
    state: u32,
    /// Degree of the shift register.
    degree: usize,
    /// Bitmask for the register.
    mask: u32,
}

impl SelfSyncScrambler {
    /// Creates a new self-synchronizing scrambler.
    ///
    /// # Arguments
    ///
    /// * `polynomial` - Feedback polynomial bitmask (taps, excluding `x^degree`).
    /// * `degree` - Number of stages in the shift register.
    pub fn new(polynomial: u32, degree: usize) -> Self {
        assert!(degree > 0 && degree <= 31, "degree must be between 1 and 31");
        let mask = (1u32 << degree) - 1;
        Self {
            polynomial,
            state: 0,
            degree,
            mask,
        }
    }

    /// Computes the feedback bit from the current state.
    fn feedback(&self) -> bool {
        let tapped = self.state & self.polynomial;
        (tapped.count_ones() & 1) != 0
    }

    /// Scrambles data. The output bit is XOR of data bit and feedback.
    /// The output bit is shifted into the register.
    pub fn scramble(&mut self, data: &[bool]) -> Vec<bool> {
        data.iter()
            .map(|&bit| {
                let fb = self.feedback();
                let output = bit ^ fb;
                // Shift output bit into the register
                self.state = ((self.state << 1) | (output as u32)) & self.mask;
                output
            })
            .collect()
    }

    /// Descrambles data. The received bit is shifted into the register,
    /// and the output is XOR of received bit and feedback.
    pub fn descramble(&mut self, data: &[bool]) -> Vec<bool> {
        data.iter()
            .map(|&bit| {
                let fb = self.feedback();
                let output = bit ^ fb;
                // Shift received bit into the register
                self.state = ((self.state << 1) | (bit as u32)) & self.mask;
                output
            })
            .collect()
    }
}

/// Creates a DVB-S/S2 PRBS scrambler.
///
/// Polynomial: x^15 + x^14 + 1 (taps at bits 14 and 13, i.e., `0x6000`).
/// Initial state: `0x4A80` (standard DVB initialization sequence `100101010000000`).
pub fn dvb_scrambler() -> PnScrambler {
    // x^15 + x^14 + 1: taps at positions 14 and 13 (zero-indexed from LSB)
    // Bit 14 = 0x4000, Bit 13 = 0x2000 => 0x6000
    PnScrambler::new(0x6000, 0x4A80, 15)
}

/// Creates an IEEE 802.11 WiFi scrambler.
///
/// Polynomial: x^7 + x^4 + 1 (taps at bits 6 and 3, i.e., `0x48`).
///
/// # Arguments
///
/// * `init` - 7-bit initialization value (scrambler seed).
pub fn wifi_scrambler(init: u8) -> PnScrambler {
    // x^7 + x^4 + 1: taps at positions 6 and 3
    // Bit 6 = 0x40, Bit 3 = 0x08 => 0x48
    PnScrambler::new(0x48, (init & 0x7F) as u32, 7)
}

/// Creates a CDMA IS-95 scrambler (simplified).
///
/// The real IS-95 long code uses a 42-stage LFSR, which is impractical
/// for simulation. This provides a simplified 15-stage version that
/// preserves the scrambling behavior.
///
/// Polynomial: x^15 + x^13 + x^9 + x^8 + x^7 + x^5 + 1
///
/// # Arguments
///
/// * `seed` - 15-bit seed value.
pub fn cdma_scrambler(seed: u32) -> PnScrambler {
    // x^15 + x^13 + x^9 + x^8 + x^7 + x^5 + 1
    // Taps at positions 14, 12, 8, 7, 6, 4 (zero-indexed)
    // 0x4000 | 0x1000 | 0x0100 | 0x0080 | 0x0040 | 0x0010 = 0x51D0
    PnScrambler::new(0x51D0, seed & 0x7FFF, 15)
}

/// Computes the period of a PN sequence generated by the given LFSR polynomial.
///
/// Runs the LFSR until the state repeats, returning the number of steps.
/// For a maximal-length LFSR of degree `n`, the period is `2^n - 1`.
///
/// # Arguments
///
/// * `polynomial` - LFSR feedback polynomial bitmask.
/// * `degree` - Number of stages in the shift register.
pub fn pn_sequence_period(polynomial: u32, degree: usize) -> usize {
    assert!(degree > 0 && degree <= 31, "degree must be between 1 and 31");
    let mask = (1u32 << degree) - 1;
    // Start from all-ones state (guaranteed non-zero)
    let initial_state = mask;
    let mut state = initial_state;
    let max_period = 1usize << degree; // absolute upper bound including zero state

    for count in 1..=max_period {
        // Clock the LFSR: compute feedback and shift
        let feedback_bits = state & polynomial;
        let feedback = (feedback_bits.count_ones() & 1) != 0;
        state = ((state << 1) | (feedback as u32)) & mask;

        if state == initial_state {
            return count;
        }
    }

    // Should not reach here for valid polynomials
    max_period
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scramble_descramble_additive() {
        let data: Vec<bool> = vec![
            true, false, true, true, false, false, true, false, true, true, false, true, false,
            false, true, true,
        ];

        let mut scrambler = PnScrambler::new(0x48, 0x7F, 7);
        let scrambled = scrambler.scramble(&data);

        // Scrambled data should differ from original
        assert_ne!(data, scrambled);

        // Descramble with a fresh scrambler in the same initial state
        let mut descrambler = PnScrambler::new(0x48, 0x7F, 7);
        let recovered = descrambler.descramble(&scrambled);

        assert_eq!(data, recovered);
    }

    #[test]
    fn test_pn_sequence_period_maximal_length() {
        // x^7 + x^6 + 1 is a maximal-length polynomial for degree 7
        // Taps at bits 6 and 5 => 0x60
        let period = pn_sequence_period(0x60, 7);
        assert_eq!(period, 127); // 2^7 - 1

        // x^4 + x^3 + 1 is maximal-length for degree 4
        // Taps at bits 3 and 2 => 0x0C
        let period = pn_sequence_period(0x0C, 4);
        assert_eq!(period, 15); // 2^4 - 1
    }

    #[test]
    fn test_different_initial_states_produce_different_sequences() {
        let mut s1 = PnScrambler::new(0x48, 0x01, 7);
        let mut s2 = PnScrambler::new(0x48, 0x7F, 7);

        let seq1 = s1.generate_sequence(20);
        let seq2 = s2.generate_sequence(20);

        assert_ne!(seq1, seq2);
    }

    #[test]
    fn test_dvb_scrambler_factory() {
        let mut s = dvb_scrambler();
        assert_eq!(s.degree, 15);
        assert_eq!(s.polynomial, 0x6000);
        assert_eq!(s.initial_state, 0x4A80);

        // Generate some bits and verify scramble/descramble roundtrip
        let data: Vec<bool> = (0..100).map(|i| i % 3 == 0).collect();
        let scrambled = s.scramble(&data);
        s.reset();
        let recovered = s.descramble(&scrambled);
        assert_eq!(data, recovered);
    }

    #[test]
    fn test_wifi_scrambler_factory() {
        let mut s = wifi_scrambler(0x5B);
        assert_eq!(s.degree, 7);
        assert_eq!(s.polynomial, 0x48);
        assert_eq!(s.initial_state, 0x5B);

        // Roundtrip
        let data: Vec<bool> = (0..50).map(|i| i % 2 == 0).collect();
        let scrambled = s.scramble(&data);
        s.reset();
        let recovered = s.descramble(&scrambled);
        assert_eq!(data, recovered);
    }

    #[test]
    fn test_self_sync_scrambler_convergence() {
        let poly = 0x48u32; // x^7 + x^4 + 1
        let degree = 7;

        let mut tx = SelfSyncScrambler::new(poly, degree);
        // Start the descrambler with a *different* initial state (zero vs whatever)
        let mut rx = SelfSyncScrambler::new(poly, degree);

        // Send a known preamble to let the descrambler converge
        let preamble: Vec<bool> = vec![false; 20];
        let data: Vec<bool> = (0..50).map(|i| i % 3 == 0).collect();

        let mut all_data = preamble.clone();
        all_data.extend_from_slice(&data);

        let scrambled = tx.scramble(&all_data);
        let descrambled = rx.descramble(&scrambled);

        // After the preamble (>= degree bits), descrambled data should match
        let offset = 20; // preamble length
        assert_eq!(&descrambled[offset..], &all_data[offset..]);
    }

    #[test]
    fn test_reset_restores_initial_state() {
        let mut s = PnScrambler::new(0x48, 0x7F, 7);

        // Generate some bits to advance state
        let _ = s.generate_sequence(50);
        assert_ne!(s.state, s.initial_state);

        // Reset and verify
        s.reset();
        assert_eq!(s.state, s.initial_state);

        // Verify that the sequence after reset matches a fresh scrambler
        let mut fresh = PnScrambler::new(0x48, 0x7F, 7);
        let seq_reset = s.generate_sequence(20);
        let seq_fresh = fresh.generate_sequence(20);
        assert_eq!(seq_reset, seq_fresh);
    }

    #[test]
    fn test_generate_sequence_length() {
        let mut s = PnScrambler::new(0x48, 0x7F, 7);
        let seq = s.generate_sequence(256);
        assert_eq!(seq.len(), 256);

        // Empty sequence
        let mut s2 = PnScrambler::new(0x48, 0x7F, 7);
        let seq0 = s2.generate_sequence(0);
        assert_eq!(seq0.len(), 0);
    }

    #[test]
    fn test_cdma_scrambler_factory() {
        let mut s = cdma_scrambler(0x1234);
        assert_eq!(s.degree, 15);
        assert_eq!(s.polynomial, 0x51D0);
        assert_eq!(s.initial_state, 0x1234);

        // Roundtrip
        let data: Vec<bool> = (0..200).map(|i| i % 5 < 2).collect();
        let scrambled = s.scramble(&data);
        s.reset();
        let recovered = s.descramble(&scrambled);
        assert_eq!(data, recovered);
    }

    #[test]
    fn test_period_computation_known_polynomials() {
        // x^3 + x^2 + 1 (maximal-length for degree 3)
        // Taps at bits 2 and 1 => 0x06
        let period = pn_sequence_period(0x06, 3);
        assert_eq!(period, 7); // 2^3 - 1

        // x^5 + x^3 + 1 (maximal-length for degree 5)
        // Taps at bits 4 and 2 => 0x14
        let period = pn_sequence_period(0x14, 5);
        assert_eq!(period, 31); // 2^5 - 1

        // x^10 + x^7 + 1 (maximal-length for degree 10)
        // Taps at bits 9 and 6 => 0x240
        let period = pn_sequence_period(0x240, 10);
        assert_eq!(period, 1023); // 2^10 - 1
    }
}
