//! Convolutional Encoder and Viterbi Decoder
//!
//! Implements convolutional encoding with arbitrary constraint length and
//! generator polynomials, plus a hard/soft decision Viterbi decoder.
//!
//! ## Standard Codes
//!
//! - **NASA K=7, Rate 1/2**: Generators [0o171, 0o133] (used in Voyager, CCSDS, 802.11a/g)
//! - **GSM K=5, Rate 1/2**: Generators [0o23, 0o33]
//! - **3GPP K=9, Rate 1/3**: Generators [0o557, 0o663, 0o711]
//!
//! ## Algorithm
//!
//! The Viterbi decoder uses the standard trellis-based approach:
//! 1. Branch metric computation (Hamming distance for hard, Euclidean for soft)
//! 2. Path metric update (add-compare-select)
//! 3. Traceback to recover the most likely input sequence

use std::fmt;

/// Configuration for a convolutional code.
#[derive(Debug, Clone)]
pub struct ConvCodeConfig {
    /// Constraint length K (memory + 1)
    pub constraint_length: usize,
    /// Generator polynomials in octal (one per output)
    pub generators: Vec<u32>,
}

impl ConvCodeConfig {
    /// NASA standard rate 1/2, K=7 convolutional code.
    ///
    /// Generators: G1=171₈ (0x79), G2=133₈ (0x5B)
    /// Used in: CCSDS, IEEE 802.11a/g, DVB-S, Voyager mission
    pub fn nasa_k7_rate_half() -> Self {
        Self {
            constraint_length: 7,
            generators: vec![0o171, 0o133],
        }
    }

    /// GSM rate 1/2, K=5 convolutional code.
    ///
    /// Generators: G1=23₈, G2=33₈
    pub fn gsm_k5_rate_half() -> Self {
        Self {
            constraint_length: 5,
            generators: vec![0o23, 0o33],
        }
    }

    /// Rate 1/3, K=9 convolutional code.
    ///
    /// Generators: G1=557₈, G2=663₈, G3=711₈
    /// Used in: 3GPP (as constituent code for turbo)
    pub fn k9_rate_third() -> Self {
        Self {
            constraint_length: 9,
            generators: vec![0o557, 0o663, 0o711],
        }
    }

    /// Simple rate 1/2, K=3 code (for testing/education).
    ///
    /// Generators: G1=7₈ (111), G2=5₈ (101)
    pub fn simple_k3() -> Self {
        Self {
            constraint_length: 3,
            generators: vec![0o7, 0o5],
        }
    }

    /// Code rate as a fraction (1/n where n = number of generators).
    pub fn rate(&self) -> f64 {
        1.0 / self.generators.len() as f64
    }

    /// Number of states in the trellis (2^(K-1)).
    pub fn num_states(&self) -> usize {
        1 << (self.constraint_length - 1)
    }

    /// Number of output bits per input bit.
    pub fn outputs_per_input(&self) -> usize {
        self.generators.len()
    }
}

impl fmt::Display for ConvCodeConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Conv(K={}, rate=1/{}, generators={:?})",
            self.constraint_length,
            self.generators.len(),
            self.generators
                .iter()
                .map(|g| format!("{:o}", g))
                .collect::<Vec<_>>()
        )
    }
}

/// Convolutional encoder.
///
/// Encodes input bits using a shift register and XOR gates defined by
/// the generator polynomials.
#[derive(Debug, Clone)]
pub struct ConvolutionalEncoder {
    config: ConvCodeConfig,
    state: u32,
}

impl ConvolutionalEncoder {
    /// Create a new encoder.
    pub fn new(config: ConvCodeConfig) -> Self {
        Self { config, state: 0 }
    }

    /// Encode a single input bit, producing n output bits.
    pub fn encode_bit(&mut self, input: bool) -> Vec<bool> {
        // Shift input bit into state register
        self.state = ((self.state << 1) | (input as u32)) & ((1 << self.config.constraint_length) - 1);

        // Compute output for each generator polynomial
        self.config
            .generators
            .iter()
            .map(|&gen| {
                let masked = self.state & gen;
                // XOR all bits (parity)
                (masked.count_ones() % 2) == 1
            })
            .collect()
    }

    /// Encode a block of input bits.
    ///
    /// Appends (K-1) flush bits to terminate the trellis at the zero state.
    pub fn encode(&mut self, input: &[bool]) -> Vec<bool> {
        self.state = 0;
        let mut output = Vec::with_capacity(
            (input.len() + self.config.constraint_length - 1) * self.config.generators.len(),
        );

        // Encode data bits
        for &bit in input {
            output.extend(self.encode_bit(bit));
        }

        // Flush bits (K-1 zeros to terminate trellis)
        for _ in 0..(self.config.constraint_length - 1) {
            output.extend(self.encode_bit(false));
        }

        output
    }

    /// Encode without trellis termination (for streaming).
    pub fn encode_stream(&mut self, input: &[bool]) -> Vec<bool> {
        let mut output = Vec::with_capacity(input.len() * self.config.generators.len());
        for &bit in input {
            output.extend(self.encode_bit(bit));
        }
        output
    }

    /// Reset encoder state.
    pub fn reset(&mut self) {
        self.state = 0;
    }
}

/// Viterbi decoder for convolutional codes.
///
/// Implements the Viterbi algorithm for maximum-likelihood sequence estimation.
#[derive(Debug)]
pub struct ViterbiDecoder {
    config: ConvCodeConfig,
    /// Precomputed output bits for each (state, input) pair
    output_table: Vec<Vec<bool>>,
}

impl ViterbiDecoder {
    /// Create a new Viterbi decoder.
    pub fn new(config: ConvCodeConfig) -> Self {
        let num_states = config.num_states();
        let n = config.outputs_per_input();

        // Precompute the output for every (state, input_bit) combination
        let mut output_table = Vec::with_capacity(num_states * 2);
        for state in 0..num_states {
            for input_bit in 0..2u32 {
                let full_state = ((state as u32) << 1) | input_bit;
                let outputs: Vec<bool> = config
                    .generators
                    .iter()
                    .map(|&gen| {
                        let masked = full_state & gen;
                        (masked.count_ones() % 2) == 1
                    })
                    .collect();
                output_table.push(outputs);
            }
        }

        Self {
            config,
            output_table,
        }
    }

    /// Look up precomputed output for a given state and input bit.
    fn expected_output(&self, state: usize, input_bit: usize) -> &[bool] {
        &self.output_table[state * 2 + input_bit]
    }

    /// Compute Hamming distance between two bit sequences.
    fn hamming_distance(a: &[bool], b: &[bool]) -> u32 {
        a.iter().zip(b.iter()).filter(|(x, y)| x != y).count() as u32
    }

    /// Decode using hard-decision Viterbi algorithm.
    ///
    /// # Arguments
    /// * `received` - Received coded bits (hard decisions)
    /// * `data_len` - Expected number of data bits (before tail bits)
    ///
    /// # Returns
    /// Decoded data bits
    pub fn decode_hard(&self, received: &[bool], data_len: usize) -> Vec<bool> {
        let num_states = self.config.num_states();
        let n = self.config.outputs_per_input();
        let total_steps = received.len() / n;

        if total_steps == 0 {
            return Vec::new();
        }

        // Path metrics: cost to reach each state
        let mut path_metrics = vec![u32::MAX; num_states];
        path_metrics[0] = 0; // Start at state 0

        // Traceback: for each step and state, store the previous state
        let mut traceback = vec![vec![0usize; num_states]; total_steps];

        let state_mask = num_states - 1;

        // Forward pass: compute path metrics
        for step in 0..total_steps {
            let received_bits: Vec<bool> = received[step * n..(step + 1) * n].to_vec();
            let mut new_metrics = vec![u32::MAX; num_states];

            for prev_state in 0..num_states {
                if path_metrics[prev_state] == u32::MAX {
                    continue;
                }

                for input_bit in 0..2usize {
                    let next_state = ((prev_state << 1) | input_bit) & state_mask;
                    let expected = self.expected_output(prev_state, input_bit);
                    let branch_metric = Self::hamming_distance(&received_bits, expected);
                    let total_metric = path_metrics[prev_state] + branch_metric;

                    if total_metric < new_metrics[next_state] {
                        new_metrics[next_state] = total_metric;
                        traceback[step][next_state] = prev_state;
                    }
                }
            }

            path_metrics = new_metrics;
        }

        // Find best ending state (should be 0 for terminated trellis)
        let end_state = if path_metrics[0] != u32::MAX {
            0 // Prefer terminated state
        } else {
            path_metrics
                .iter()
                .enumerate()
                .min_by_key(|(_, &m)| m)
                .map(|(s, _)| s)
                .unwrap_or(0)
        };

        // Traceback to recover input bits
        let mut decoded = vec![false; total_steps];
        let mut current_state = end_state;

        for step in (0..total_steps).rev() {
            let prev_state = traceback[step][current_state];
            // The input bit is the LSB of current_state that wasn't in prev_state
            let input_bit = current_state & 1;
            decoded[step] = input_bit == 1;
            current_state = prev_state;
        }

        // Return only data bits (exclude tail)
        decoded.truncate(data_len);
        decoded
    }

    /// Decode using soft-decision Viterbi algorithm.
    ///
    /// # Arguments
    /// * `received` - Received soft values (positive = likely 1, negative = likely 0)
    /// * `data_len` - Expected number of data bits
    ///
    /// # Returns
    /// Decoded data bits
    pub fn decode_soft(&self, received: &[f64], data_len: usize) -> Vec<bool> {
        let n = self.config.outputs_per_input();
        let num_states = self.config.num_states();
        let total_steps = received.len() / n;

        if total_steps == 0 {
            return Vec::new();
        }

        // Path metrics (floating point for soft decisions)
        let mut path_metrics = vec![f64::MAX; num_states];
        path_metrics[0] = 0.0;

        let mut traceback = vec![vec![0usize; num_states]; total_steps];
        let state_mask = num_states - 1;

        for step in 0..total_steps {
            let received_slice = &received[step * n..(step + 1) * n];
            let mut new_metrics = vec![f64::MAX; num_states];

            for prev_state in 0..num_states {
                if path_metrics[prev_state] == f64::MAX {
                    continue;
                }

                for input_bit in 0..2usize {
                    let next_state = ((prev_state << 1) | input_bit) & state_mask;
                    let expected = self.expected_output(prev_state, input_bit);

                    // Euclidean distance: sum of (received - expected)^2
                    let branch_metric: f64 = expected
                        .iter()
                        .zip(received_slice.iter())
                        .map(|(&exp, &recv)| {
                            let exp_val = if exp { 1.0 } else { -1.0 };
                            (recv - exp_val) * (recv - exp_val)
                        })
                        .sum();

                    let total_metric = path_metrics[prev_state] + branch_metric;

                    if total_metric < new_metrics[next_state] {
                        new_metrics[next_state] = total_metric;
                        traceback[step][next_state] = prev_state;
                    }
                }
            }

            path_metrics = new_metrics;
        }

        // Find best ending state
        let end_state = if path_metrics[0] < f64::MAX {
            0
        } else {
            path_metrics
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
                .map(|(s, _)| s)
                .unwrap_or(0)
        };

        // Traceback
        let mut decoded = vec![false; total_steps];
        let mut current_state = end_state;

        for step in (0..total_steps).rev() {
            let prev_state = traceback[step][current_state];
            let input_bit = current_state & 1;
            decoded[step] = input_bit == 1;
            current_state = prev_state;
        }

        decoded.truncate(data_len);
        decoded
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simple_k3_encode_decode() {
        let config = ConvCodeConfig::simple_k3();
        let mut encoder = ConvolutionalEncoder::new(config.clone());
        let decoder = ViterbiDecoder::new(config);

        let data = vec![true, false, true, true, false, false, true, false];
        let encoded = encoder.encode(&data);

        // Rate 1/2 with K-1=2 tail bits
        assert_eq!(encoded.len(), (data.len() + 2) * 2);

        let decoded = decoder.decode_hard(&encoded, data.len());
        assert_eq!(decoded, data, "K=3 decode should match original");
    }

    #[test]
    fn test_nasa_k7_encode_decode() {
        let config = ConvCodeConfig::nasa_k7_rate_half();
        let mut encoder = ConvolutionalEncoder::new(config.clone());
        let decoder = ViterbiDecoder::new(config);

        let data = vec![
            true, false, true, true, false, false, true, false, true, true, false, true, false,
            false, true, true,
        ];
        let encoded = encoder.encode(&data);

        // Rate 1/2 with K-1=6 tail bits
        assert_eq!(encoded.len(), (data.len() + 6) * 2);

        let decoded = decoder.decode_hard(&encoded, data.len());
        assert_eq!(decoded, data, "K=7 decode should match original");
    }

    #[test]
    fn test_error_correction_k3() {
        let config = ConvCodeConfig::simple_k3();
        let mut encoder = ConvolutionalEncoder::new(config.clone());
        let decoder = ViterbiDecoder::new(config);

        let data = vec![true, false, true, true, false, false, true, false];
        let mut encoded = encoder.encode(&data);

        // Introduce 1 bit error
        encoded[4] = !encoded[4];

        let decoded = decoder.decode_hard(&encoded, data.len());
        assert_eq!(
            decoded, data,
            "Viterbi should correct single bit error in K=3 code"
        );
    }

    #[test]
    fn test_error_correction_k7() {
        let config = ConvCodeConfig::nasa_k7_rate_half();
        let mut encoder = ConvolutionalEncoder::new(config.clone());
        let decoder = ViterbiDecoder::new(config);

        let data: Vec<bool> = (0..32).map(|i| i % 3 == 0).collect();
        let mut encoded = encoder.encode(&data);

        // Introduce 3 scattered bit errors
        encoded[2] = !encoded[2];
        encoded[15] = !encoded[15];
        encoded[40] = !encoded[40];

        let decoded = decoder.decode_hard(&encoded, data.len());
        assert_eq!(
            decoded, data,
            "Viterbi K=7 should correct scattered errors"
        );
    }

    #[test]
    fn test_soft_decision_decoding() {
        let config = ConvCodeConfig::simple_k3();
        let mut encoder = ConvolutionalEncoder::new(config.clone());
        let decoder = ViterbiDecoder::new(config);

        let data = vec![true, false, true, true, false, false, true, false];
        let encoded = encoder.encode(&data);

        // Convert to soft values: true -> +1.0, false -> -1.0
        // Add some noise
        let soft: Vec<f64> = encoded
            .iter()
            .enumerate()
            .map(|(i, &b)| {
                let val = if b { 1.0 } else { -1.0 };
                // Add pseudo-noise
                let noise = ((i as f64 * 1.7).sin()) * 0.3;
                val + noise
            })
            .collect();

        let decoded = decoder.decode_soft(&soft, data.len());
        assert_eq!(decoded, data, "Soft decision should decode correctly");
    }

    #[test]
    fn test_encoder_reset() {
        let config = ConvCodeConfig::simple_k3();
        let mut encoder = ConvolutionalEncoder::new(config);

        let data = vec![true, false, true];
        let encoded1 = encoder.encode(&data);
        let encoded2 = encoder.encode(&data);

        // encode() resets state internally, so results should match
        assert_eq!(encoded1, encoded2);
    }

    #[test]
    fn test_config_display() {
        let config = ConvCodeConfig::nasa_k7_rate_half();
        let s = format!("{}", config);
        assert!(s.contains("K=7"));
        assert!(s.contains("rate=1/2"));
    }

    #[test]
    fn test_config_properties() {
        let config = ConvCodeConfig::nasa_k7_rate_half();
        assert_eq!(config.num_states(), 64); // 2^6
        assert_eq!(config.outputs_per_input(), 2);
        assert!((config.rate() - 0.5).abs() < 1e-10);

        let config3 = ConvCodeConfig::k9_rate_third();
        assert_eq!(config3.num_states(), 256); // 2^8
        assert_eq!(config3.outputs_per_input(), 3);
        assert!((config3.rate() - 1.0 / 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_all_zeros() {
        let config = ConvCodeConfig::simple_k3();
        let mut encoder = ConvolutionalEncoder::new(config.clone());
        let decoder = ViterbiDecoder::new(config);

        let data = vec![false; 16];
        let encoded = encoder.encode(&data);
        let decoded = decoder.decode_hard(&encoded, data.len());
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_all_ones() {
        let config = ConvCodeConfig::simple_k3();
        let mut encoder = ConvolutionalEncoder::new(config.clone());
        let decoder = ViterbiDecoder::new(config);

        let data = vec![true; 16];
        let encoded = encoder.encode(&data);
        let decoded = decoder.decode_hard(&encoded, data.len());
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_gsm_code() {
        let config = ConvCodeConfig::gsm_k5_rate_half();
        let mut encoder = ConvolutionalEncoder::new(config.clone());
        let decoder = ViterbiDecoder::new(config);

        let data: Vec<bool> = (0..20).map(|i| i % 2 == 0).collect();
        let encoded = encoder.encode(&data);
        let decoded = decoder.decode_hard(&encoded, data.len());
        assert_eq!(decoded, data, "GSM K=5 should roundtrip correctly");
    }
}
