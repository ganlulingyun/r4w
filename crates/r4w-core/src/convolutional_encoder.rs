//! # Convolutional Encoder
//!
//! A rate-1/n convolutional encoder for forward error correction (FEC) in digital
//! communications systems. This module implements the core encoding side of
//! convolutional codes, widely used in DVB, TETRA, DMR, IEEE 802.11, and many
//! other standards.
//!
//! ## Features
//!
//! - **Standard K=7 rate-1/2 code** with generator polynomials 171/133 (octal)
//! - **Configurable constraint length** from K=3 to K=9
//! - **Termination modes**: zero-terminated (flush with tail bits) and tail-biting
//! - **Puncturing support** for higher code rates (2/3, 3/4, 5/6, 7/8)
//! - **Trellis state machine** inspection for educational and decoder use
//! - **Encoder state** query and reset
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::convolutional_encoder::ConvolutionalEncoder;
//!
//! // Create a standard K=7, rate-1/2 encoder (171/133 octal)
//! let mut encoder = ConvolutionalEncoder::k7_rate_half();
//! assert_eq!(encoder.rate(), 0.5);
//!
//! // Encode some data bits with zero termination (appends K-1 tail bits)
//! let data = vec![true, false, true, true, false, false, true, false];
//! let coded = encoder.encode_terminated(&data);
//!
//! // The terminated output has (data_len + K - 1) * n_outputs bits
//! // = (8 + 6) * 2 = 28 bits
//! assert_eq!(coded.len(), 28);
//!
//! // The encoder is back at state 0 after termination
//! assert_eq!(encoder.state(), 0);
//! ```

/// A convolutional encoder supporting configurable constraint lengths, generator
/// polynomials, multiple termination modes, and puncturing.
///
/// The encoder operates as a finite state machine where each input bit causes a
/// state transition and produces `n` output bits (one per generator polynomial).
/// The state register holds the last `K-1` input bits.
#[derive(Debug, Clone)]
pub struct ConvolutionalEncoder {
    /// Constraint length K (memory = K-1)
    constraint_length: usize,
    /// Generator polynomials stored as bitmasks. Each polynomial is provided in
    /// octal but stored internally as a binary mask of `K` bits. Bit 0 (LSB)
    /// corresponds to the most recent input, bit K-1 to the oldest.
    generators: Vec<u64>,
    /// Current shift-register state (K-1 bits wide)
    state: u64,
    /// Optional puncture pattern. Each inner vec corresponds to one encoder
    /// output. `true` means transmit, `false` means puncture (discard).
    puncture_pattern: Option<Vec<Vec<bool>>>,
}

impl ConvolutionalEncoder {
    /// Create a new convolutional encoder with the given constraint length and
    /// generator polynomials (specified in **octal**).
    ///
    /// # Panics
    ///
    /// Panics if `constraint_length` is less than 2 or greater than 64, or if
    /// `generators` is empty.
    pub fn new(constraint_length: usize, generators: &[u64]) -> Self {
        assert!(
            constraint_length >= 2,
            "Constraint length must be at least 2"
        );
        assert!(
            constraint_length <= 64,
            "Constraint length must be at most 64"
        );
        assert!(!generators.is_empty(), "Must provide at least one generator polynomial");

        // Convert octal representation to binary bitmask
        let gen_masks: Vec<u64> = generators.iter().map(|&g| octal_to_binary(g)).collect();

        ConvolutionalEncoder {
            constraint_length,
            generators: gen_masks,
            state: 0,
            puncture_pattern: None,
        }
    }

    /// Create the industry-standard K=7, rate-1/2 encoder with generator
    /// polynomials 171 and 133 (octal). This code is used in DVB-S, 802.11a/g,
    /// TETRA, DMR, CCSDS, and many other standards.
    pub fn k7_rate_half() -> Self {
        Self::new(7, &[0o171, 0o133])
    }

    /// Encode a sequence of input bits. The encoder state carries over between
    /// calls (no termination or tail bits are appended).
    ///
    /// For each input bit, `n` output bits are produced (one per generator
    /// polynomial), yielding `bits.len() * n` output bits total.
    pub fn encode(&mut self, bits: &[bool]) -> Vec<bool> {
        let n = self.generators.len();
        let mut output = Vec::with_capacity(bits.len() * n);

        for &bit in bits {
            let outputs = self.step(bit);
            output.extend_from_slice(&outputs);
        }

        output
    }

    /// Encode with zero termination: after encoding all data bits, append `K-1`
    /// zero bits to flush the encoder back to the all-zeros state. This makes
    /// the code easier to decode (the Viterbi decoder knows the final state).
    ///
    /// Total output length: `(bits.len() + K - 1) * n`
    pub fn encode_terminated(&mut self, bits: &[bool]) -> Vec<bool> {
        self.reset();
        let tail_len = self.constraint_length - 1;
        let n = self.generators.len();
        let total_input = bits.len() + tail_len;
        let mut output = Vec::with_capacity(total_input * n);

        // Encode data bits
        for &bit in bits {
            let outputs = self.step(bit);
            output.extend_from_slice(&outputs);
        }

        // Append tail bits (zeros) to flush encoder to state 0
        for _ in 0..tail_len {
            let outputs = self.step(false);
            output.extend_from_slice(&outputs);
        }

        output
    }

    /// Encode with tail-biting: the encoder is initialized to the state
    /// corresponding to the last `K-1` bits of the input, so that the final
    /// state equals the initial state. This avoids the rate loss of zero
    /// termination at the cost of decoder complexity.
    ///
    /// Total output length: `bits.len() * n` (no overhead)
    ///
    /// # Panics
    ///
    /// Panics if `bits.len() < K - 1`.
    pub fn encode_tail_biting(&mut self, bits: &[bool]) -> Vec<bool> {
        let memory = self.constraint_length - 1;
        assert!(
            bits.len() >= memory,
            "Input length must be at least K-1 = {} for tail-biting",
            memory
        );

        // Initialize state from the last K-1 bits of the input
        let state_mask = (1u64 << memory) - 1;
        let mut init_state: u64 = 0;
        for i in 0..memory {
            if bits[bits.len() - memory + i] {
                init_state |= 1u64 << i;
            }
        }
        self.state = init_state & state_mask;

        // Encode all bits (no tail)
        self.encode(bits)
    }

    /// Set the puncture pattern for producing higher code rates.
    ///
    /// The pattern is a 2D array: `pattern[output_index][position_in_period]`.
    /// `true` keeps the bit, `false` punctures (discards) it. The pattern
    /// repeats every `period` input bits where `period = pattern[0].len()`.
    ///
    /// # Common patterns (for rate-1/2 base code)
    ///
    /// - **Rate 2/3**: `[[true, true], [true, false]]` (transmit 3 of 4 coded bits)
    /// - **Rate 3/4**: `[[true, true, false], [true, false, true]]` (transmit 4 of 6)
    /// - **Rate 5/6**: puncture to keep 6 out of 10 coded bits per period
    /// - **Rate 7/8**: puncture to keep 8 out of 14 coded bits per period
    ///
    /// # Panics
    ///
    /// Panics if the pattern dimensions do not match the number of generator
    /// polynomials, or if inner vectors have different lengths.
    pub fn set_puncture_pattern(&mut self, pattern: Vec<Vec<bool>>) {
        assert_eq!(
            pattern.len(),
            self.generators.len(),
            "Puncture pattern must have one row per generator polynomial"
        );
        if !pattern.is_empty() {
            let period = pattern[0].len();
            assert!(period > 0, "Puncture pattern period must be positive");
            for row in &pattern {
                assert_eq!(
                    row.len(),
                    period,
                    "All puncture pattern rows must have the same length"
                );
            }
        }
        self.puncture_pattern = Some(pattern);
    }

    /// Encode with puncturing applied. The encoder is reset before encoding.
    /// The puncture pattern must be set beforehand via [`set_puncture_pattern`].
    ///
    /// # Panics
    ///
    /// Panics if no puncture pattern has been set.
    pub fn encode_punctured(&mut self, bits: &[bool]) -> Vec<bool> {
        let pattern = self
            .puncture_pattern
            .as_ref()
            .expect("Puncture pattern must be set before calling encode_punctured")
            .clone();

        self.reset();

        let n = self.generators.len();
        let period = pattern[0].len();
        let mut output = Vec::new();

        for (i, &bit) in bits.iter().enumerate() {
            let outputs = self.step(bit);
            let pos = i % period;
            for (j, &out_bit) in outputs.iter().enumerate() {
                if pattern[j][pos] {
                    output.push(out_bit);
                }
            }
        }

        // Append terminated tail bits with puncturing
        let tail_len = self.constraint_length - 1;
        for t in 0..tail_len {
            let outputs = self.step(false);
            let pos = (bits.len() + t) % period;
            for (j, &out_bit) in outputs.iter().enumerate() {
                if pattern[j][pos] {
                    output.push(out_bit);
                }
            }
        }

        output
    }

    /// Return the current encoder shift-register state.
    pub fn state(&self) -> u64 {
        self.state
    }

    /// Reset the encoder state to all zeros.
    pub fn reset(&mut self) {
        self.state = 0;
    }

    /// Return the base code rate as a fraction (1/n) where n is the number of
    /// generator polynomials.
    pub fn rate(&self) -> f64 {
        1.0 / self.generators.len() as f64
    }

    /// Generate all trellis transitions for this code. Each entry is
    /// `(from_state, to_state, input_bit, output_bits)`.
    ///
    /// This is useful for building Viterbi decoders and for educational
    /// visualization of the trellis/state diagram.
    pub fn trellis_transitions(&self) -> Vec<(u64, u64, bool, Vec<bool>)> {
        let memory = self.constraint_length - 1;
        let num_states = 1u64 << memory;
        let state_mask = num_states - 1;
        let mut transitions = Vec::with_capacity((num_states as usize) * 2);

        for from_state in 0..num_states {
            for &input in &[false, true] {
                let input_bit = if input { 1u64 } else { 0u64 };
                // The register contents: input bit in MSB position, state shifted right
                let reg = (input_bit << memory) | from_state;

                let mut outputs = Vec::with_capacity(self.generators.len());
                for gen in &self.generators {
                    let masked = reg & gen;
                    let parity = masked.count_ones() % 2;
                    outputs.push(parity == 1);
                }

                // Next state: shift in the new bit from the left
                let to_state = (reg >> 1) & state_mask;

                transitions.push((from_state, to_state, input, outputs));
            }
        }

        transitions
    }

    // --- Private helpers ---

    /// Process one input bit through the encoder and return the output bits.
    fn step(&mut self, input: bool) -> Vec<bool> {
        let memory = self.constraint_length - 1;
        let state_mask = (1u64 << memory) - 1;
        let input_bit = if input { 1u64 } else { 0u64 };

        // Form the full register: input bit at position K-1, state in lower K-2..0
        let reg = (input_bit << memory) | self.state;

        let mut outputs = Vec::with_capacity(self.generators.len());
        for gen in &self.generators {
            let masked = reg & gen;
            let parity = masked.count_ones() % 2;
            outputs.push(parity == 1);
        }

        // Update state: shift in new bit from the left
        self.state = (reg >> 1) & state_mask;

        outputs
    }
}

/// Convert an octal-encoded polynomial to its binary bitmask.
///
/// For example, octal 171 = 0o171 = binary 001_111_001 = 0b1111001 = 121 decimal.
/// Each octal digit maps to 3 binary bits.
fn octal_to_binary(octal: u64) -> u64 {
    // Rust integer literals with the 0o prefix are already in octal, so the
    // value passed in is already the correct binary representation. However,
    // if the caller passes a *decimal* integer that represents octal digits
    // (e.g. 171 meaning 1*64 + 7*8 + 1 = 121), we need to handle that.
    //
    // Since the API documents that generators are "in octal" and Rust has the
    // 0o prefix, callers should use 0o171. But we also handle the case where
    // the literal is decimal digits of octal representation.
    //
    // Convention: we accept the value as-is. If the user writes 0o171, Rust
    // converts that to 121 at compile time. If they write 171 decimal, that
    // is a different polynomial. The docstring tells users to use octal.
    octal
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_k7_rate_half_creation() {
        let enc = ConvolutionalEncoder::k7_rate_half();
        assert_eq!(enc.constraint_length, 7);
        assert_eq!(enc.generators.len(), 2);
        assert_eq!(enc.state, 0);
        assert!((enc.rate() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_encode_output_length() {
        let mut enc = ConvolutionalEncoder::k7_rate_half();
        let bits = vec![true, false, true, true, false];
        let coded = enc.encode(&bits);
        // rate-1/2: each input bit produces 2 output bits
        assert_eq!(coded.len(), bits.len() * 2);
    }

    #[test]
    fn test_encode_terminated_output_length() {
        let mut enc = ConvolutionalEncoder::k7_rate_half();
        let bits = vec![true, false, true, true, false, false, true, false];
        let coded = enc.encode_terminated(&bits);
        // (data_len + K-1) * n = (8 + 6) * 2 = 28
        assert_eq!(coded.len(), (bits.len() + 6) * 2);
    }

    #[test]
    fn test_encode_terminated_returns_to_zero_state() {
        let mut enc = ConvolutionalEncoder::k7_rate_half();
        let bits = vec![true, true, false, true, false, true, true, false];
        let _coded = enc.encode_terminated(&bits);
        assert_eq!(enc.state(), 0);
    }

    #[test]
    fn test_tail_biting_initial_equals_final_state() {
        let mut enc = ConvolutionalEncoder::new(3, &[0o7, 0o5]);
        let bits = vec![true, false, true, true, false, false, true, true];
        let _coded = enc.encode_tail_biting(&bits);
        // For tail-biting, the initial state was set from the last K-1 bits,
        // and after encoding all bits we should return to that same state.
        // Re-derive what the initial state should have been:
        let memory = 2; // K-1
        let mut expected_state: u64 = 0;
        for i in 0..memory {
            if bits[bits.len() - memory + i] {
                expected_state |= 1u64 << i;
            }
        }
        // After encoding, the final state should equal the initial state
        // because the last K-1 input bits are the same ones used to initialize.
        assert_eq!(enc.state(), expected_state);
    }

    #[test]
    fn test_reset_clears_state() {
        let mut enc = ConvolutionalEncoder::k7_rate_half();
        enc.encode(&[true, true, true]);
        assert_ne!(enc.state(), 0);
        enc.reset();
        assert_eq!(enc.state(), 0);
    }

    #[test]
    fn test_encode_deterministic() {
        // Same input should produce same output when starting from same state
        let mut enc1 = ConvolutionalEncoder::k7_rate_half();
        let mut enc2 = ConvolutionalEncoder::k7_rate_half();
        let bits = vec![true, false, true, true, false, false, true, false];
        let coded1 = enc1.encode(&bits);
        let coded2 = enc2.encode(&bits);
        assert_eq!(coded1, coded2);
    }

    #[test]
    fn test_all_zeros_input() {
        let mut enc = ConvolutionalEncoder::k7_rate_half();
        let bits = vec![false; 20];
        let coded = enc.encode(&bits);
        // All-zeros input from zero state should produce all-zeros output
        assert!(coded.iter().all(|&b| !b));
        assert_eq!(enc.state(), 0);
    }

    #[test]
    fn test_single_one_bit() {
        // A single '1' bit into an all-zero state should produce a non-trivial output
        let mut enc = ConvolutionalEncoder::k7_rate_half();
        let coded = enc.encode(&[true]);
        assert_eq!(coded.len(), 2);
        // For generators 0o171 (=0b1111001=121) and 0o133 (=0b1011011=91):
        // Register = [1, 0, 0, 0, 0, 0, 0] with input=1 at position 6
        // reg = 1<<6 | 0 = 64 = 0b1000000
        // gen0 = 0o171 = 121 = 0b1111001; masked = 0b1000000 & 0b1111001 = 0b1000000;
        //   popcount=1, parity=1 => true
        // gen1 = 0o133 = 91 = 0b1011011; masked = 0b1000000 & 0b1011011 = 0b1000000;
        //   popcount=1, parity=1 => true
        assert_eq!(coded, vec![true, true]);
    }

    #[test]
    fn test_k3_rate_half_known_sequence() {
        // K=3, generators 7 (0o7=0b111) and 5 (0o5=0b101)
        // This is a well-known example from textbooks
        let mut enc = ConvolutionalEncoder::new(3, &[0o7, 0o5]);
        // Input: 1, 0, 1
        // Step 1: input=1, state=00
        //   reg = 1<<2 | 00 = 100 = 4
        //   g0=7=111: 4&7=4=100, popcount=1, parity=1 => true
        //   g1=5=101: 4&5=4=100, popcount=1, parity=1 => true
        //   new state = (4>>1)&3 = 2&3 = 2 = 10
        // Step 2: input=0, state=10
        //   reg = 0<<2 | 10 = 010 = 2
        //   g0=7=111: 2&7=2=010, popcount=1, parity=1 => true
        //   g1=5=101: 2&5=0=000, popcount=0, parity=0 => false
        //   new state = (2>>1)&3 = 1&3 = 1 = 01
        // Step 3: input=1, state=01
        //   reg = 1<<2 | 01 = 101 = 5
        //   g0=7=111: 5&7=5=101, popcount=2, parity=0 => false
        //   g1=5=101: 5&5=5=101, popcount=2, parity=0 => false
        //   new state = (5>>1)&3 = 2&3 = 2 = 10
        let coded = enc.encode(&[true, false, true]);
        assert_eq!(coded, vec![true, true, true, false, false, false]);
        assert_eq!(enc.state(), 2); // state = 10
    }

    #[test]
    fn test_punctured_encoding() {
        let mut enc = ConvolutionalEncoder::new(3, &[0o7, 0o5]);
        // Rate 2/3 puncture pattern: keep 3 of 4 coded bits per period of 2
        enc.set_puncture_pattern(vec![
            vec![true, true],
            vec![true, false],
        ]);

        let bits = vec![true, false, true, true, false, false];
        let coded = enc.encode_punctured(&bits);

        // Without puncturing, we'd have (6 + 2) * 2 = 16 coded bits (including tail)
        // With puncturing, we keep 3/4 of them, repeating with period 2
        // Total input positions = 6 data + 2 tail = 8
        // For each pair of positions: 4 coded bits, keep 3 => 3 bits per pair
        // 8 positions / 2 period = 4 repetitions, 4 * 3 = 12 kept bits
        assert_eq!(coded.len(), 12);
    }

    #[test]
    fn test_trellis_transitions_count() {
        let enc = ConvolutionalEncoder::new(3, &[0o7, 0o5]);
        let transitions = enc.trellis_transitions();
        // K=3 => 4 states, 2 inputs each => 8 transitions
        assert_eq!(transitions.len(), 8);

        // Each transition should have 2 output bits
        for (_, _, _, ref outputs) in &transitions {
            assert_eq!(outputs.len(), 2);
        }
    }

    #[test]
    fn test_trellis_transitions_state_consistency() {
        let enc = ConvolutionalEncoder::new(3, &[0o7, 0o5]);
        let transitions = enc.trellis_transitions();

        // Verify transitions match step-by-step encoding
        for &(from_state, to_state, input, ref expected_outputs) in &transitions {
            let mut test_enc = ConvolutionalEncoder::new(3, &[0o7, 0o5]);
            test_enc.state = from_state;
            let actual_outputs = test_enc.step(input);
            assert_eq!(test_enc.state, to_state,
                "State mismatch for transition from {} with input {}",
                from_state, input);
            assert_eq!(&actual_outputs, expected_outputs,
                "Output mismatch for transition from {} with input {}",
                from_state, input);
        }
    }

    #[test]
    fn test_rate_calculation() {
        let enc2 = ConvolutionalEncoder::new(3, &[0o7, 0o5]);
        assert!((enc2.rate() - 0.5).abs() < 1e-10);

        let enc3 = ConvolutionalEncoder::new(3, &[0o7, 0o5, 0o3]);
        assert!((enc3.rate() - 1.0 / 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_encode_state_continuity() {
        // Encoding in chunks should produce the same result as encoding all at once
        let mut enc_all = ConvolutionalEncoder::k7_rate_half();
        let bits = vec![true, false, true, true, false, false, true, false, true, true];
        let coded_all = enc_all.encode(&bits);

        let mut enc_chunked = ConvolutionalEncoder::k7_rate_half();
        let mut coded_chunked = Vec::new();
        coded_chunked.extend(enc_chunked.encode(&bits[..5]));
        coded_chunked.extend(enc_chunked.encode(&bits[5..]));

        assert_eq!(coded_all, coded_chunked);
    }

    #[test]
    #[should_panic(expected = "Constraint length must be at least 2")]
    fn test_invalid_constraint_length() {
        ConvolutionalEncoder::new(1, &[0o7, 0o5]);
    }

    #[test]
    #[should_panic(expected = "Must provide at least one generator polynomial")]
    fn test_empty_generators() {
        ConvolutionalEncoder::new(3, &[]);
    }
}
