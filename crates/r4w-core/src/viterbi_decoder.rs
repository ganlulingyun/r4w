//! Viterbi Decoder — Convolutional Encoder and Maximum-Likelihood Decoder
//!
//! Implements the Viterbi algorithm for decoding convolutional codes, the
//! workhorse FEC scheme in digital communications. Convolutional codes add
//! redundancy by convolving input bits with generator polynomials through a
//! shift register, producing a continuous stream of coded bits. The Viterbi
//! decoder finds the maximum-likelihood path through the code trellis using
//! dynamic programming, achieving near-optimal performance with tractable
//! complexity.
//!
//! Applications include DVB-S/S2, IEEE 802.11a/g/n (WiFi), GSM/EDGE, IS-95
//! (CDMA), deep-space links (Voyager, Mars missions), and aviation (VDL Mode 2).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::viterbi_decoder::{ConvolutionalEncoder, ViterbiDecoder};
//!
//! // NASA standard rate-1/2, K=7 code (Voyager, 802.11a)
//! let (k, gens) = (7, vec![0o171, 0o133]);
//! let mut encoder = ConvolutionalEncoder::new(k, gens.clone());
//! let data = vec![true, false, true, true, false, false, true, false];
//! let encoded = encoder.encode(&data);
//!
//! let decoder = ViterbiDecoder::new(k, gens);
//! let decoded = decoder.decode_hard(&encoded);
//! assert_eq!(&decoded[..data.len()], &data[..]);
//! ```

/// Convolutional encoder with configurable constraint length and generator polynomials.
///
/// Encodes input bits at rate 1/n where n is the number of generator polynomials.
/// The shift register has K-1 memory elements (K = constraint length), giving
/// 2^(K-1) states in the code trellis.
///
/// Generator polynomials are specified in octal notation following standard
/// convention. For example, the NASA K=7 code uses generators 171 and 133
/// (octal), which correspond to the binary tap patterns 1111001 and 1011011.
#[derive(Debug, Clone)]
pub struct ConvolutionalEncoder {
    /// Constraint length K (shift register length including input).
    constraint_length: usize,
    /// Generator polynomials in octal notation.
    generators: Vec<u32>,
    /// Current shift register state (K-1 bits).
    state: u32,
    /// Bitmask for the shift register (2^(K-1) - 1).
    state_mask: u32,
}

impl ConvolutionalEncoder {
    /// Create a new convolutional encoder.
    ///
    /// # Arguments
    /// * `constraint_length` - K, the number of shift register stages including input
    /// * `generators` - Generator polynomials in octal notation
    ///
    /// # Panics
    /// Panics if constraint_length < 2 or generators is empty.
    pub fn new(constraint_length: usize, generators: Vec<u32>) -> Self {
        assert!(constraint_length >= 2, "Constraint length must be >= 2");
        assert!(!generators.is_empty(), "Must have at least one generator");
        let state_mask = (1u32 << (constraint_length - 1)) - 1;
        Self {
            constraint_length,
            generators,
            state: 0,
            state_mask,
        }
    }

    /// Encode input bits, producing rate 1/n output.
    ///
    /// For each input bit, the encoder produces n output bits (one per generator
    /// polynomial). The input bit is shifted into the register and each generator
    /// polynomial is applied via modulo-2 convolution (XOR of tapped positions).
    pub fn encode(&mut self, input: &[bool]) -> Vec<bool> {
        let n = self.generators.len();
        let mut output = Vec::with_capacity(input.len() * n);

        for &bit in input {
            // Form the full register: input bit in MSB position, then state
            let reg = ((bit as u32) << (self.constraint_length - 1)) | self.state;

            for &gen in &self.generators {
                // XOR the register with the generator and count parity
                let masked = reg & gen;
                let parity = masked.count_ones() & 1;
                output.push(parity == 1);
            }

            // Shift state: new state is upper K-2 bits
            self.state = (reg >> 1) & self.state_mask;
        }

        output
    }

    /// Reset the shift register to the all-zero state.
    pub fn reset(&mut self) {
        self.state = 0;
    }

    /// Return the code rate as (1, n).
    pub fn rate(&self) -> (usize, usize) {
        (1, self.generators.len())
    }

    /// Return the number of trellis states: 2^(K-1).
    pub fn num_states(&self) -> usize {
        1 << (self.constraint_length - 1)
    }

}

/// Maximum-likelihood Viterbi decoder for convolutional codes.
///
/// Implements the classic Viterbi algorithm with both hard-decision and
/// soft-decision decoding. Hard decision uses Hamming distance as the branch
/// metric; soft decision uses Euclidean distance on LLR (log-likelihood ratio)
/// values.
///
/// The traceback depth defaults to 5*(K-1), which is sufficient for most
/// codes to ensure the survivor paths have merged. The decoder assumes the
/// encoder starts in the zero state.
#[derive(Debug, Clone)]
pub struct ViterbiDecoder {
    /// Constraint length K.
    constraint_length: usize,
    /// Generator polynomials (must match encoder).
    generators: Vec<u32>,
    /// Number of trellis states: 2^(K-1).
    num_states: usize,
    /// Traceback depth (default: 5*(K-1)).
    traceback_depth: usize,
    /// Precomputed branch outputs: branch_outputs[state][input] = Vec<bool>.
    branch_outputs: Vec<[Vec<bool>; 2]>,
    /// Precomputed next states: next_state[state][input] = next_state.
    next_states: Vec<[usize; 2]>,
}

impl ViterbiDecoder {
    /// Create a Viterbi decoder matching the given encoder parameters.
    ///
    /// # Arguments
    /// * `constraint_length` - K, must match the encoder
    /// * `generators` - Generator polynomials in octal, must match the encoder
    pub fn new(constraint_length: usize, generators: Vec<u32>) -> Self {
        assert!(constraint_length >= 2, "Constraint length must be >= 2");
        assert!(!generators.is_empty(), "Must have at least one generator");

        let num_states = 1 << (constraint_length - 1);
        let traceback_depth = 5 * (constraint_length - 1);

        // Precompute trellis structure
        let mut branch_outputs = Vec::with_capacity(num_states);
        let mut next_states = Vec::with_capacity(num_states);

        for state in 0..num_states {
            let s = state as u32;
            let mut outputs = [vec![], vec![]];
            let mut nexts = [0usize; 2];

            for input in 0..2u32 {
                let reg = (input << (constraint_length - 1)) | s;
                let out: Vec<bool> = generators
                    .iter()
                    .map(|&gen| {
                        let masked = reg & gen;
                        (masked.count_ones() & 1) == 1
                    })
                    .collect();
                outputs[input as usize] = out;
                nexts[input as usize] = ((reg >> 1) & ((num_states as u32) - 1)) as usize;
            }

            branch_outputs.push(outputs);
            next_states.push(nexts);
        }

        Self {
            constraint_length,
            generators,
            num_states,
            traceback_depth,
            branch_outputs,
            next_states,
        }
    }

    /// Decode soft LLR values using the Viterbi algorithm.
    ///
    /// Input LLR convention: positive values indicate bit 0 is more likely,
    /// negative values indicate bit 1 is more likely. The magnitude represents
    /// confidence. Groups of n soft values form one trellis step (n = number of
    /// generator polynomials).
    ///
    /// Uses Euclidean distance as the branch metric, computed as the sum of
    /// squared differences between received soft values and expected values
    /// (+1.0 for bit 0, -1.0 for bit 1).
    pub fn decode(&self, received: &[f64]) -> Vec<bool> {
        let n = self.generators.len();
        assert!(
            received.len() % n == 0,
            "Received length must be a multiple of n={}",
            n
        );
        let num_steps = received.len() / n;
        if num_steps == 0 {
            return vec![];
        }

        // Path metrics: cost to reach each state
        let mut path_metrics = vec![f64::INFINITY; self.num_states];
        path_metrics[0] = 0.0; // Start in state 0

        // Survivor memory: for each step, store the previous state for each current state
        let mut survivors: Vec<Vec<usize>> = Vec::with_capacity(num_steps);
        let mut decisions: Vec<Vec<bool>> = Vec::with_capacity(num_steps);

        for step in 0..num_steps {
            let received_chunk = &received[step * n..(step + 1) * n];
            let mut new_metrics = vec![f64::INFINITY; self.num_states];
            let mut new_survivors = vec![0usize; self.num_states];
            let mut new_decisions = vec![false; self.num_states];

            for state in 0..self.num_states {
                if path_metrics[state] == f64::INFINITY {
                    continue;
                }

                for input in 0..2usize {
                    let next = self.next_states[state][input];
                    let expected = &self.branch_outputs[state][input];

                    // Euclidean branch metric: sum of (received - expected)^2
                    // Expected: +1.0 for false (bit 0), -1.0 for true (bit 1)
                    let mut branch_metric = 0.0;
                    for (i, &exp_bit) in expected.iter().enumerate() {
                        let expected_val = if exp_bit { -1.0 } else { 1.0 };
                        let diff = received_chunk[i] - expected_val;
                        branch_metric += diff * diff;
                    }

                    let candidate = path_metrics[state] + branch_metric;
                    if candidate < new_metrics[next] {
                        new_metrics[next] = candidate;
                        new_survivors[next] = state;
                        new_decisions[next] = input == 1;
                    }
                }
            }

            path_metrics = new_metrics;
            survivors.push(new_survivors);
            decisions.push(new_decisions);
        }

        // Traceback: find the state with minimum metric
        let mut best_state = 0;
        let mut best_metric = f64::INFINITY;
        for (state, &metric) in path_metrics.iter().enumerate() {
            if metric < best_metric {
                best_metric = metric;
                best_state = state;
            }
        }

        // Trace back through survivors
        let mut decoded = vec![false; num_steps];
        let mut current_state = best_state;
        for step in (0..num_steps).rev() {
            decoded[step] = decisions[step][current_state];
            current_state = survivors[step][current_state];
        }

        decoded
    }

    /// Decode hard-decision bits using Hamming distance as the branch metric.
    ///
    /// Converts hard bits to LLR-like values internally (+1.0 for false, -1.0
    /// for true) and delegates to the soft decoder. This produces identical
    /// results to a dedicated hard-decision Viterbi but reuses the same
    /// traceback machinery.
    pub fn decode_hard(&self, received: &[bool]) -> Vec<bool> {
        let n = self.generators.len();
        assert!(
            received.len() % n == 0,
            "Received length must be a multiple of n={}",
            n
        );
        let num_steps = received.len() / n;
        if num_steps == 0 {
            return vec![];
        }

        // Path metrics using Hamming distance
        let mut path_metrics = vec![u32::MAX; self.num_states];
        path_metrics[0] = 0;

        let mut survivors: Vec<Vec<usize>> = Vec::with_capacity(num_steps);
        let mut decisions_store: Vec<Vec<bool>> = Vec::with_capacity(num_steps);

        for step in 0..num_steps {
            let received_chunk = &received[step * n..(step + 1) * n];
            let mut new_metrics = vec![u32::MAX; self.num_states];
            let mut new_survivors = vec![0usize; self.num_states];
            let mut new_decisions = vec![false; self.num_states];

            for state in 0..self.num_states {
                if path_metrics[state] == u32::MAX {
                    continue;
                }

                for input in 0..2usize {
                    let next = self.next_states[state][input];
                    let expected = &self.branch_outputs[state][input];

                    // Hamming distance
                    let mut hamming = 0u32;
                    for (i, &exp_bit) in expected.iter().enumerate() {
                        if received_chunk[i] != exp_bit {
                            hamming += 1;
                        }
                    }

                    let candidate = path_metrics[state].saturating_add(hamming);
                    if candidate < new_metrics[next] {
                        new_metrics[next] = candidate;
                        new_survivors[next] = state;
                        new_decisions[next] = input == 1;
                    }
                }
            }

            path_metrics = new_metrics;
            survivors.push(new_survivors);
            decisions_store.push(new_decisions);
        }

        // Traceback from minimum-metric state
        let mut best_state = 0;
        let mut best_metric = u32::MAX;
        for (state, &metric) in path_metrics.iter().enumerate() {
            if metric < best_metric {
                best_metric = metric;
                best_state = state;
            }
        }

        let mut decoded = vec![false; num_steps];
        let mut current_state = best_state;
        for step in (0..num_steps).rev() {
            decoded[step] = decisions_store[step][current_state];
            current_state = survivors[step][current_state];
        }

        decoded
    }
}

// ---------------------------------------------------------------------------
// Presets
// ---------------------------------------------------------------------------

/// NASA standard rate-1/2, K=7 convolutional code.
///
/// Generator polynomials: (171, 133) octal = (1111001, 1011011) binary.
/// Used in Voyager deep-space missions, IEEE 802.11a/g (WiFi), and DVB-S.
/// Free distance d_free = 10, providing approximately 5 dB coding gain
/// over uncoded BPSK at BER = 10^-5.
pub fn nasa_k7_rate_half() -> (usize, Vec<u32>) {
    (7, vec![0o171, 0o133])
}

/// GSM rate-1/2, K=5 convolutional code.
///
/// Generator polynomials: (23, 33) octal = (10011, 11011) binary.
/// Used in GSM full-rate speech channel coding (class 1a and class 1b bits).
pub fn gsm_k5_rate_half() -> (usize, Vec<u32>) {
    (5, vec![0o23, 0o33])
}

// ---------------------------------------------------------------------------
// Free distance computation
// ---------------------------------------------------------------------------

/// Compute the free distance (d_free) of a convolutional code.
///
/// The free distance is the minimum Hamming distance between any two valid
/// codeword sequences, and determines the error-correcting capability:
/// the code can correct up to floor((d_free - 1) / 2) errors.
///
/// This uses exhaustive weight enumeration: for each nonzero input sequence
/// of weight 1 that starts and returns to the zero state, we count the output
/// weight. The minimum such output weight is d_free.
///
/// Only practical for small constraint lengths (K <= 10 or so) due to
/// exponential search space.
pub fn free_distance(constraint_length: usize, generators: &[u32]) -> usize {
    let num_states = 1u32 << (constraint_length - 1);
    let state_mask = num_states - 1;
    // We search for the minimum-weight output for all paths that diverge
    // from state 0 and re-merge to state 0. The maximum path length is
    // bounded: after at most 2^(K-1) + K - 1 steps a path must re-merge
    // or we skip it.
    let max_path_len = (num_states as usize) + constraint_length;
    let mut min_dist = usize::MAX;

    // BFS/DFS: enumerate all paths from state 0 with input bit 1 first step
    // (to diverge), then any input, tracking output weight until re-merge.
    struct PathState {
        state: u32,
        output_weight: usize,
        depth: usize,
    }

    let mut stack: Vec<PathState> = Vec::new();

    // First step must have input = 1 to diverge from zero state
    {
        let reg = 1u32 << (constraint_length - 1); // input=1, state=0
        let next = (reg >> 1) & state_mask;
        let mut weight = 0;
        for &gen in generators {
            let masked = reg & gen;
            weight += (masked.count_ones() & 1) as usize;
        }
        stack.push(PathState {
            state: next,
            output_weight: weight,
            depth: 1,
        });
    }

    while let Some(ps) = stack.pop() {
        if ps.depth >= max_path_len {
            continue;
        }
        // Prune: if already worse than best, skip
        if ps.output_weight >= min_dist {
            continue;
        }

        for input in 0..2u32 {
            let reg = (input << (constraint_length - 1)) | ps.state;
            let next = (reg >> 1) & state_mask;
            let mut step_weight = 0;
            for &gen in generators {
                let masked = reg & gen;
                step_weight += (masked.count_ones() & 1) as usize;
            }
            let total_weight = ps.output_weight + step_weight;

            if next == 0 {
                // Re-merged to zero state
                if total_weight < min_dist {
                    min_dist = total_weight;
                }
            } else if total_weight < min_dist {
                stack.push(PathState {
                    state: next,
                    output_weight: total_weight,
                    depth: ps.depth + 1,
                });
            }
        }
    }

    if min_dist == usize::MAX {
        // Should not happen for valid codes
        0
    } else {
        min_dist
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Test basic encoding with a simple K=3 rate-1/2 code.
    /// Generators: (7, 5) octal = (111, 101) binary.
    #[test]
    fn test_encode_basic() {
        let mut encoder = ConvolutionalEncoder::new(3, vec![0o7, 0o5]);
        // Input: [1], state starts at 00
        // Register: 1|00 = 100
        // Gen 7 (111): 100 & 111 = 100 -> parity 1
        // Gen 5 (101): 100 & 101 = 100 -> parity 1
        let output = encoder.encode(&[true]);
        assert_eq!(output, vec![true, true]);

        // Reset and encode a longer sequence
        encoder.reset();
        let input = vec![true, false, true];
        let output = encoder.encode(&input);
        // Rate 1/2 -> 6 output bits
        assert_eq!(output.len(), 6);
    }

    /// Test encode then decode roundtrip with K=3, generators (7,5).
    #[test]
    fn test_encode_decode_roundtrip_k3() {
        let k = 3;
        let gens = vec![0o7, 0o5];
        let mut encoder = ConvolutionalEncoder::new(k, gens.clone());

        let data = vec![true, false, true, true, false, false, true, false];
        // Append tail bits (K-1 zeros) to flush encoder
        let mut input = data.clone();
        for _ in 0..(k - 1) {
            input.push(false);
        }
        let encoded = encoder.encode(&input);

        let decoder = ViterbiDecoder::new(k, gens);
        let decoded = decoder.decode_hard(&encoded);

        // Compare original data bits (excluding tail)
        assert_eq!(&decoded[..data.len()], &data[..]);
    }

    /// Test NASA K=7 rate-1/2 preset encode/decode roundtrip.
    #[test]
    fn test_nasa_preset_roundtrip() {
        let (k, gens) = nasa_k7_rate_half();
        let mut encoder = ConvolutionalEncoder::new(k, gens.clone());

        let data: Vec<bool> = vec![
            true, false, true, true, false, false, true, false, true, true, false, true, false,
            false, true, true,
        ];
        let mut input = data.clone();
        for _ in 0..(k - 1) {
            input.push(false);
        }
        let encoded = encoder.encode(&input);

        let decoder = ViterbiDecoder::new(k, gens);
        let decoded = decoder.decode_hard(&encoded);
        assert_eq!(&decoded[..data.len()], &data[..]);
    }

    /// Test GSM K=5 rate-1/2 preset encode/decode roundtrip.
    #[test]
    fn test_gsm_preset_roundtrip() {
        let (k, gens) = gsm_k5_rate_half();
        let mut encoder = ConvolutionalEncoder::new(k, gens.clone());

        let data: Vec<bool> = vec![
            false, true, true, false, true, false, false, true, true, true, false, false,
        ];
        let mut input = data.clone();
        for _ in 0..(k - 1) {
            input.push(false);
        }
        let encoded = encoder.encode(&input);

        let decoder = ViterbiDecoder::new(k, gens);
        let decoded = decoder.decode_hard(&encoded);
        assert_eq!(&decoded[..data.len()], &data[..]);
    }

    /// Test soft decoding with noisy LLR values.
    #[test]
    fn test_soft_decoding_with_noise() {
        let k = 3;
        let gens = vec![0o7, 0o5];
        let mut encoder = ConvolutionalEncoder::new(k, gens.clone());

        let data = vec![true, false, true, true, false, false, true, false];
        let mut input = data.clone();
        for _ in 0..(k - 1) {
            input.push(false);
        }
        let encoded = encoder.encode(&input);

        // Convert to soft LLRs: +1.0 for 0, -1.0 for 1, then add noise
        let soft: Vec<f64> = encoded
            .iter()
            .enumerate()
            .map(|(i, &b)| {
                let clean = if b { -1.0 } else { 1.0 };
                // Add deterministic "noise" pattern
                let noise = ((i as f64) * 0.31415).sin() * 0.3;
                clean + noise
            })
            .collect();

        let decoder = ViterbiDecoder::new(k, gens);
        let decoded = decoder.decode(&soft);
        assert_eq!(&decoded[..data.len()], &data[..]);
    }

    /// Test hard decoding explicitly.
    #[test]
    fn test_hard_decoding() {
        let k = 3;
        let gens = vec![0o7, 0o5];
        let mut encoder = ConvolutionalEncoder::new(k, gens.clone());

        let data = vec![false, true, false, true, true, false];
        let mut input = data.clone();
        for _ in 0..(k - 1) {
            input.push(false);
        }
        let encoded = encoder.encode(&input);

        // Introduce one bit error
        let mut received = encoded.clone();
        received[3] = !received[3]; // Flip one bit

        let decoder = ViterbiDecoder::new(k, gens);
        let decoded = decoder.decode_hard(&received);
        // Should still decode correctly (within error-correcting capability)
        assert_eq!(&decoded[..data.len()], &data[..]);
    }

    /// Test rate computation.
    #[test]
    fn test_rate_computation() {
        let enc2 = ConvolutionalEncoder::new(3, vec![0o7, 0o5]);
        assert_eq!(enc2.rate(), (1, 2));

        let enc3 = ConvolutionalEncoder::new(3, vec![0o7, 0o5, 0o3]);
        assert_eq!(enc3.rate(), (1, 3));
    }

    /// Test num_states computation.
    #[test]
    fn test_num_states() {
        let enc3 = ConvolutionalEncoder::new(3, vec![0o7, 0o5]);
        assert_eq!(enc3.num_states(), 4); // 2^(3-1) = 4

        let enc7 = ConvolutionalEncoder::new(7, vec![0o171, 0o133]);
        assert_eq!(enc7.num_states(), 64); // 2^(7-1) = 64

        let enc5 = ConvolutionalEncoder::new(5, vec![0o23, 0o33]);
        assert_eq!(enc5.num_states(), 16); // 2^(5-1) = 16
    }

    /// Test free distance for a known K=3 rate-1/2 code.
    /// (7,5) octal has d_free = 5.
    #[test]
    fn test_free_distance_k3() {
        let dfree = free_distance(3, &[0o7, 0o5]);
        assert_eq!(dfree, 5);
    }

    /// Test encoder reset returns to zero state.
    #[test]
    fn test_encoder_reset() {
        let mut encoder = ConvolutionalEncoder::new(3, vec![0o7, 0o5]);

        // Encode something to change state
        encoder.encode(&[true, true, false, true]);

        // Reset and encode again - should produce same output as fresh encoder
        encoder.reset();
        let out1 = encoder.encode(&[true, false]);

        let mut fresh = ConvolutionalEncoder::new(3, vec![0o7, 0o5]);
        let out2 = fresh.encode(&[true, false]);

        assert_eq!(out1, out2);
    }
}
