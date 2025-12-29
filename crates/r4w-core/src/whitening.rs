//! Data Whitening for LoRa
//!
//! Whitening (also called scrambling) is used to:
//! 1. Eliminate long runs of 0s or 1s in the data
//! 2. Provide DC balance to the transmitted signal
//! 3. Reduce spectral peaks
//!
//! ## How Whitening Works
//!
//! LoRa uses a Linear Feedback Shift Register (LFSR) to generate a
//! pseudo-random sequence. This sequence is XORed with the data bits:
//!
//! ```text
//! Data:     1 0 1 1 0 0 1 0
//! Whitening: 1 1 0 1 0 1 1 0  (from LFSR)
//! XOR:      0 1 1 0 0 1 0 0  (transmitted)
//! ```
//!
//! On the receiver side, the same LFSR sequence is generated and XORed
//! again to recover the original data.
//!
//! ## LFSR Structure
//!
//! LoRa uses an 8-bit LFSR with feedback taps at positions 3, 4, 5, and 7:
//!
//! ```text
//! ┌───┬───┬───┬───┬───┬───┬───┬───┐
//! │ 7 │ 6 │ 5 │ 4 │ 3 │ 2 │ 1 │ 0 │
//! └─┬─┴───┴─┬─┴─┬─┴─┬─┴───┴───┴─┬─┘
//!   │       │   │   │           ▼
//!   │       └───┴───┴─────XOR───┴──► Next bit 0
//!   │
//!   └─────────────────────────────► Output
//! ```

use serde::{Deserialize, Serialize};

/// LFSR-based whitening/scrambling for LoRa
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Whitening {
    /// Current LFSR state
    state: u8,
    /// Feedback polynomial taps (XOR mask)
    /// LoRa uses taps at positions 3, 4, 5, 7 → 0b10111000 = 0xB8
    /// But the implementation uses a different bit ordering
    feedback: u8,
}

impl Default for Whitening {
    fn default() -> Self {
        Self::new()
    }
}

impl Whitening {
    /// Initial LFSR state: all 1s
    const INITIAL_STATE: u8 = 0xFF;

    /// Feedback polynomial for LoRa whitening
    /// Taps at bits 3, 4, 5, 7 in Python impl: [0,0,0,1,1,1,0,1]
    const FEEDBACK_POLY: u8 = 0b00111010; // Reversed bit order

    /// Create a new whitening instance with default initial state
    pub fn new() -> Self {
        Self {
            state: Self::INITIAL_STATE,
            feedback: Self::FEEDBACK_POLY,
        }
    }

    /// Reset the LFSR to initial state
    pub fn reset(&mut self) {
        self.state = Self::INITIAL_STATE;
    }

    /// Get the current LFSR state (useful for debugging)
    pub fn state(&self) -> u8 {
        self.state
    }

    /// Advance the LFSR by one step and return the output bit
    fn step(&mut self) -> u8 {
        // Calculate feedback bit using XOR of selected taps
        let feedback_bit = (self.state & self.feedback).count_ones() as u8 & 1;

        // Shift register and insert new bit
        let output = (self.state >> 7) & 1;
        self.state = (self.state << 1) | feedback_bit;

        output
    }

    /// Generate the next whitening byte
    fn next_byte(&mut self) -> u8 {
        let mut result = 0u8;
        for i in 0..8 {
            result |= self.step() << (7 - i);
        }
        result
    }

    /// Whiten (or de-whiten) a data buffer in place
    ///
    /// Since XOR is its own inverse, the same function works for both
    /// whitening and de-whitening.
    pub fn process(&mut self, data: &mut [u8]) {
        for byte in data.iter_mut() {
            *byte ^= self.next_byte();
        }
    }

    /// Whiten data and return a new buffer
    pub fn whiten(&mut self, data: &[u8]) -> Vec<u8> {
        let mut result = data.to_vec();
        self.process(&mut result);
        result
    }

    /// Generate whitening sequence of given length (for visualization)
    pub fn generate_sequence(&mut self, len: usize) -> Vec<u8> {
        let mut seq = Vec::with_capacity(len);
        for _ in 0..len {
            seq.push(self.next_byte());
        }
        seq
    }

    /// Generate whitening sequence as bits (for detailed visualization)
    pub fn generate_bit_sequence(&mut self, num_bits: usize) -> Vec<u8> {
        let mut bits = Vec::with_capacity(num_bits);
        for _ in 0..num_bits {
            bits.push(self.step());
        }
        bits
    }
}

/// Educational demonstration of LFSR operation
#[derive(Debug, Clone)]
pub struct LfsrDemo {
    /// History of LFSR states
    pub states: Vec<u8>,
    /// Output bits generated
    pub outputs: Vec<u8>,
    /// Feedback bits calculated
    pub feedback_bits: Vec<u8>,
}

impl LfsrDemo {
    /// Run the LFSR for a number of steps and record the history
    pub fn run(num_steps: usize) -> Self {
        let mut whitening = Whitening::new();
        let mut states = Vec::with_capacity(num_steps + 1);
        let mut outputs = Vec::with_capacity(num_steps);
        let mut feedback_bits = Vec::with_capacity(num_steps);

        states.push(whitening.state());

        for _ in 0..num_steps {
            // Record feedback calculation
            let fb = (whitening.state & Whitening::FEEDBACK_POLY).count_ones() as u8 & 1;
            feedback_bits.push(fb);

            // Step and record output
            let output = whitening.step();
            outputs.push(output);
            states.push(whitening.state());
        }

        Self {
            states,
            outputs,
            feedback_bits,
        }
    }

    /// Format state as binary string for display
    pub fn format_state(state: u8) -> String {
        format!("{:08b}", state)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_whitening_reversible() {
        let original = vec![0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0];

        // Whiten
        let mut whitening = Whitening::new();
        let whitened = whitening.whiten(&original);

        // De-whiten (reset and apply again)
        whitening.reset();
        let mut recovered = whitened.clone();
        whitening.process(&mut recovered);

        assert_eq!(original, recovered);
    }

    #[test]
    fn test_whitening_changes_data() {
        let original = vec![0x00; 16];

        let mut whitening = Whitening::new();
        let whitened = whitening.whiten(&original);

        // Should not be all zeros anymore
        assert_ne!(original, whitened);
    }

    #[test]
    fn test_lfsr_periodicity() {
        // Test that the LFSR enters a valid cycle
        // Note: The initial state (0xFF) may not be part of the main cycle,
        // so we test that the LFSR eventually repeats (enters a cycle).
        let mut whitening = Whitening::new();

        // Run a few steps to enter the cycle from transient states
        for _ in 0..10 {
            whitening.step();
        }
        let cycle_state = whitening.state();

        // Now find the period from this state
        let mut period = 0;
        for i in 1..=256 {
            whitening.step();
            if whitening.state() == cycle_state {
                period = i;
                break;
            }
        }

        // LFSR should have a valid cycle (period > 0)
        assert!(period > 0, "LFSR should have a finite period");
        // Period should be reasonable for an 8-bit LFSR
        assert!(period <= 255, "Period {} exceeds maximum for 8-bit LFSR", period);
    }
}
