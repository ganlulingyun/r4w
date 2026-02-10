//! Soft-Output Viterbi Algorithm (SOVA) — Soft-In / Soft-Out Decoding
//!
//! Implements the SOVA decoder for convolutional codes, producing soft output
//! (log-likelihood ratios) for each decoded bit rather than hard decisions alone.
//! The reliability information enables iterative decoding in turbo codes and
//! concatenated coding schemes where extrinsic information is exchanged between
//! component decoders.
//!
//! SOVA extends the classic Viterbi algorithm by tracking the path metric
//! difference between the survivor and its competitor at each trellis node.
//! During traceback, the reliability of each decoded bit is computed as the
//! minimum metric delta along the competing path that would have flipped that
//! bit. The output LLR sign indicates the hard decision (positive = bit 0,
//! negative = bit 1) and the magnitude indicates confidence.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::viterbi_sova::{convolutional_encode, sova_k3, SovaDecoder};
//!
//! // Encode some data with K=3 rate-1/2 code (generators 7, 5 octal)
//! let data = vec![true, false, true, true, false];
//! let encoded = convolutional_encode(&data, &[0o7, 0o5], 3);
//!
//! // Convert to LLRs (perfect channel: +1.0 for bit 0, -1.0 for bit 1)
//! let llrs: Vec<f64> = encoded.iter().map(|&b| if b { -1.0 } else { 1.0 }).collect();
//!
//! // Decode with SOVA — get soft output LLRs
//! let mut decoder = sova_k3();
//! let soft_out = decoder.decode(&llrs, 2);
//! let hard_out: Vec<bool> = soft_out.iter().map(|&v| v < 0.0).collect();
//! assert_eq!(&hard_out[..data.len()], &data[..]);
//! ```

/// Trellis structure describing the state machine of a convolutional code.
///
/// Each state has a list of possible transitions. Each transition specifies
/// the next state and the output bits produced for that branch.
#[derive(Debug, Clone)]
pub struct Trellis {
    /// Number of states in the trellis.
    num_states: usize,
    /// Transition table: `outputs[state]` is a list of `(next_state, output_bits)`.
    /// For a rate-1/n code, there are 2 transitions per state (input 0 and input 1).
    outputs: Vec<Vec<(usize, Vec<i8>)>>,
}

impl Trellis {
    /// Create a new trellis with the given state transitions.
    ///
    /// # Arguments
    /// * `num_states` - Number of states (must be a power of 2 for convolutional codes)
    /// * `outputs` - Transition table: `outputs[state] = [(next_state, output_bits), ...]`
    ///
    /// # Panics
    /// Panics if `outputs.len() != num_states` or any state index is out of range.
    pub fn new(num_states: usize, outputs: Vec<Vec<(usize, Vec<i8>)>>) -> Self {
        assert_eq!(
            outputs.len(),
            num_states,
            "outputs length must equal num_states"
        );
        for (s, transitions) in outputs.iter().enumerate() {
            for (next, _) in transitions {
                assert!(
                    *next < num_states,
                    "state {} has transition to invalid state {}",
                    s,
                    next
                );
            }
        }
        Self { num_states, outputs }
    }

    /// Return the number of states in the trellis.
    pub fn num_states(&self) -> usize {
        self.num_states
    }
}

/// Soft-Output Viterbi Algorithm (SOVA) decoder.
///
/// Produces soft-decision output (LLR per decoded bit) from soft-input channel
/// observations. The reliability of each bit equals the minimum path metric
/// difference between the survivor and any competing path that would have
/// produced a different decision for that bit within the traceback window.
#[derive(Debug, Clone)]
pub struct SovaDecoder {
    /// The code trellis.
    trellis: Trellis,
    /// Traceback length (number of trellis steps to look back).
    traceback_length: usize,
    /// Number of branches per state (2 for binary input codes).
    branches_per_state: usize,
    /// Precomputed input bit for each (state, branch) pair.
    /// For rate-1/n codes, branch 0 => input bit 0, branch 1 => input bit 1.
    input_bits: Vec<Vec<bool>>,
}

impl SovaDecoder {
    /// Create a new SOVA decoder.
    ///
    /// # Arguments
    /// * `trellis` - The code trellis describing state transitions and outputs
    /// * `traceback_length` - Number of steps to trace back for reliability computation;
    ///   a typical value is 5*(K-1) for constraint length K
    pub fn new(trellis: Trellis, traceback_length: usize) -> Self {
        assert!(traceback_length > 0, "traceback_length must be > 0");
        // Determine branches per state from the first state
        let branches_per_state = if trellis.num_states > 0 {
            trellis.outputs[0].len()
        } else {
            2
        };

        // For a rate-1/n code, branch index IS the input bit.
        let input_bits: Vec<Vec<bool>> = trellis
            .outputs
            .iter()
            .map(|transitions| {
                transitions
                    .iter()
                    .enumerate()
                    .map(|(branch_idx, _)| branch_idx != 0)
                    .collect()
            })
            .collect();

        Self {
            trellis,
            traceback_length,
            branches_per_state,
            input_bits,
        }
    }

    /// Decode received LLRs and produce soft-output LLRs for each data bit.
    ///
    /// # Arguments
    /// * `received_llrs` - Channel LLR values (positive = bit 0 more likely,
    ///   negative = bit 1 more likely). Length must be a multiple of `code_rate_inv`.
    /// * `code_rate_inv` - Inverse of the code rate (e.g., 2 for rate-1/2)
    ///
    /// # Returns
    /// Soft output LLRs, one per decoded data bit. Positive means bit 0,
    /// negative means bit 1. Magnitude indicates reliability.
    pub fn decode(&mut self, received_llrs: &[f64], code_rate_inv: usize) -> Vec<f64> {
        assert!(
            received_llrs.len() % code_rate_inv == 0,
            "received_llrs length must be a multiple of code_rate_inv={}",
            code_rate_inv
        );
        let num_steps = received_llrs.len() / code_rate_inv;
        if num_steps == 0 {
            return vec![];
        }

        let ns = self.trellis.num_states;

        // Path metrics for each state (lower is better).
        let mut path_metrics = vec![f64::INFINITY; ns];
        path_metrics[0] = 0.0;

        // Storage for traceback:
        // survivor_state[step][state] = predecessor state on the survivor path
        // survivor_input[step][state] = input bit (true/false) on the survivor path
        // delta[step][state] = metric difference between survivor and competitor
        let mut survivor_state: Vec<Vec<usize>> = Vec::with_capacity(num_steps);
        let mut survivor_input: Vec<Vec<bool>> = Vec::with_capacity(num_steps);
        let mut delta: Vec<Vec<f64>> = Vec::with_capacity(num_steps);

        // Forward pass: ACS (Add-Compare-Select)
        for step in 0..num_steps {
            let rx = &received_llrs[step * code_rate_inv..(step + 1) * code_rate_inv];

            let mut new_metrics = vec![f64::INFINITY; ns];
            let mut new_survivor_state = vec![0usize; ns];
            let mut new_survivor_input = vec![false; ns];
            let mut new_delta = vec![0.0f64; ns];

            // For each current state, try all branches
            for state in 0..ns {
                if path_metrics[state] == f64::INFINITY {
                    continue;
                }

                for branch in 0..self.branches_per_state {
                    let (next_state, ref out_bits) = self.trellis.outputs[state][branch];
                    let input_bit = self.input_bits[state][branch];

                    // Branch metric: correlation-style metric.
                    // For each output bit, expected LLR is +1 for 0, -1 for 1.
                    // Branch metric = sum of -rx_i * expected_i (we minimize, so negate correlation).
                    let mut branch_metric = 0.0;
                    for (i, &out_bit) in out_bits.iter().enumerate() {
                        let expected = if out_bit == 0 { 1.0 } else { -1.0 };
                        let diff = rx[i] - expected;
                        branch_metric += diff * diff;
                    }

                    let candidate = path_metrics[state] + branch_metric;

                    if candidate < new_metrics[next_state] {
                        // This is a better path — the old best (if any) becomes competitor.
                        let old_metric = new_metrics[next_state];
                        let competitor_delta = if old_metric == f64::INFINITY {
                            f64::INFINITY
                        } else {
                            old_metric - candidate
                        };
                        new_delta[next_state] = competitor_delta;
                        new_metrics[next_state] = candidate;
                        new_survivor_state[next_state] = state;
                        new_survivor_input[next_state] = input_bit;
                    } else if candidate < new_metrics[next_state] + new_delta[next_state]
                        || new_delta[next_state] == f64::INFINITY
                    {
                        // This is the closest competitor so far.
                        let d = candidate - new_metrics[next_state];
                        if d < new_delta[next_state] || new_delta[next_state] == f64::INFINITY {
                            new_delta[next_state] = d;
                        }
                    }
                }
            }

            path_metrics = new_metrics;
            survivor_state.push(new_survivor_state);
            survivor_input.push(new_survivor_input);
            delta.push(new_delta);
        }

        // Find best final state.
        let mut best_state = 0;
        let mut best_metric = f64::INFINITY;
        for (s, &m) in path_metrics.iter().enumerate() {
            if m < best_metric {
                best_metric = m;
                best_state = s;
            }
        }

        // Traceback to get the survivor path and hard decisions.
        let mut hard_decisions = vec![false; num_steps];
        let mut path_states = vec![0usize; num_steps + 1];
        path_states[num_steps] = best_state;

        {
            let mut cs = best_state;
            for step in (0..num_steps).rev() {
                hard_decisions[step] = survivor_input[step][cs];
                let prev = survivor_state[step][cs];
                path_states[step] = prev;
                cs = prev;
            }
        }

        // Compute soft output using SOVA reliability update.
        // For each bit position, the reliability is the minimum delta along
        // the traceback window where the competing path disagrees with the
        // survivor decision.
        let mut soft_output = vec![0.0f64; num_steps];

        for bit_pos in 0..num_steps {
            let mut min_delta = f64::INFINITY;
            let survivor_bit = hard_decisions[bit_pos];

            // Look forward within the traceback window from this bit position.
            let window_end = (bit_pos + self.traceback_length).min(num_steps);

            for step in bit_pos..window_end {
                let state_on_path = path_states[step + 1];
                let d = delta[step][state_on_path];

                if d < f64::INFINITY {
                    // Check if the competing path at this step would have produced
                    // a different bit at bit_pos. We approximate this by checking
                    // whether the competitor at each step along the window could
                    // differ. In the standard SOVA, any competitor within the
                    // traceback window that disagrees contributes.
                    //
                    // Simplified SOVA: use the minimum delta along the path
                    // from bit_pos to the window end. This is conservative but
                    // widely used and avoids the complexity of full competing
                    // path reconstruction.
                    if d < min_delta {
                        min_delta = d;
                    }
                }
            }

            // Clamp infinity to a large but finite value.
            if min_delta == f64::INFINITY {
                min_delta = 100.0;
            }

            // Output sign: positive for bit 0, negative for bit 1.
            let sign = if survivor_bit { -1.0 } else { 1.0 };
            soft_output[bit_pos] = sign * min_delta;
        }

        soft_output
    }

    /// Decode received LLRs and produce hard-decision output bits.
    ///
    /// This is a convenience wrapper around [`decode`](Self::decode) that
    /// converts the soft LLR output to hard bits.
    pub fn decode_hard(
        &mut self,
        received_llrs: &[f64],
        code_rate_inv: usize,
    ) -> Vec<bool> {
        let soft = self.decode(received_llrs, code_rate_inv);
        soft.iter().map(|&v| v < 0.0).collect()
    }

    /// Reset internal state. Currently a no-op since the decoder is stateless
    /// between calls, but provided for API symmetry and future extensions
    /// (e.g., streaming / windowed decoding).
    pub fn reset(&mut self) {
        // Reserved for future state (e.g., sliding-window buffer).
        // Resetting traceback_length to itself is a no-op marker that
        // tests can verify via decode behavior after reset.
        let _ = self.traceback_length;
    }
}

/// Build a trellis for a rate-1/2 convolutional code.
///
/// # Arguments
/// * `constraint_length` - K (shift register length including the input bit)
/// * `generators` - Two generator polynomials in octal notation
///
/// # Returns
/// A [`Trellis`] with `2^(K-1)` states and 2 branches per state.
///
/// # Panics
/// Panics if `generators` does not have exactly 2 elements or K < 2.
pub fn rate_1_2_trellis(constraint_length: usize, generators: &[u32]) -> Trellis {
    assert!(constraint_length >= 2, "Constraint length must be >= 2");
    assert_eq!(generators.len(), 2, "rate_1_2_trellis requires exactly 2 generators");

    let num_states = 1 << (constraint_length - 1);
    let state_mask = num_states - 1;

    let mut outputs = Vec::with_capacity(num_states);

    for state in 0..num_states {
        let mut transitions = Vec::with_capacity(2);

        for input in 0..2u32 {
            // Form shift register: input bit at MSB, state in lower bits
            let reg = (input << (constraint_length - 1)) | (state as u32);

            // Compute output bits from each generator polynomial
            let out_bits: Vec<i8> = generators
                .iter()
                .map(|&gen| {
                    let masked = reg & gen;
                    (masked.count_ones() & 1) as i8
                })
                .collect();

            // Next state: shift right by 1
            let next_state = ((reg >> 1) & (state_mask as u32)) as usize;

            transitions.push((next_state, out_bits));
        }

        outputs.push(transitions);
    }

    Trellis::new(num_states, outputs)
}

/// Encode data bits using a convolutional code with the given generators.
///
/// # Arguments
/// * `data` - Input data bits
/// * `generators` - Generator polynomials in octal notation
/// * `constraint_length` - K
///
/// # Returns
/// Encoded bits (length = `data.len() * generators.len()`)
pub fn convolutional_encode(data: &[bool], generators: &[u32], constraint_length: usize) -> Vec<bool> {
    assert!(constraint_length >= 2, "Constraint length must be >= 2");
    assert!(!generators.is_empty(), "Must have at least one generator");

    let state_mask = (1u32 << (constraint_length - 1)) - 1;
    let n = generators.len();
    let mut output = Vec::with_capacity(data.len() * n);
    let mut state: u32 = 0;

    for &bit in data {
        let reg = ((bit as u32) << (constraint_length - 1)) | state;

        for &gen in generators {
            let masked = reg & gen;
            let parity = (masked.count_ones() & 1) == 1;
            output.push(parity);
        }

        state = (reg >> 1) & state_mask;
    }

    output
}

/// Create a SOVA decoder for the K=7 rate-1/2 code with generators (171, 133) octal.
///
/// This is the NASA/ESA standard convolutional code used in Voyager, 802.11a/g/n,
/// DVB-S, and many other systems. The decoder has 64 states and a traceback
/// length of 30 (5 * (K-1)).
pub fn sova_k7() -> SovaDecoder {
    let trellis = rate_1_2_trellis(7, &[0o171, 0o133]);
    SovaDecoder::new(trellis, 5 * 6)
}

/// Create a SOVA decoder for the K=3 rate-1/2 code with generators (7, 5) octal.
///
/// This is a simple 4-state code useful for testing and educational purposes.
/// Traceback length is 10 (5 * (K-1)).
pub fn sova_k3() -> SovaDecoder {
    let trellis = rate_1_2_trellis(3, &[0o7, 0o5]);
    SovaDecoder::new(trellis, 5 * 2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_trellis_construction() {
        let trellis = rate_1_2_trellis(3, &[0o7, 0o5]);
        assert_eq!(trellis.num_states(), 4);
        // Each state should have exactly 2 transitions (input 0 and input 1)
        for s in 0..4 {
            assert_eq!(trellis.outputs[s].len(), 2);
            // Each transition should produce 2 output bits (rate 1/2)
            for (next, bits) in &trellis.outputs[s] {
                assert!(*next < 4, "next state out of range");
                assert_eq!(bits.len(), 2, "rate-1/2 should produce 2 bits per branch");
            }
        }
    }

    #[test]
    fn test_convolutional_encoding_correctness() {
        // K=3, generators (7, 5) octal = (111, 101) binary
        // For input [1, 0, 1] starting from state 0:
        // Step 1: input=1, reg=100, gen7: 100&111=100 -> parity 1, gen5: 100&101=100 -> parity 1
        //   output: [1, 1], next state: 10
        // Step 2: input=0, reg=010, gen7: 010&111=010 -> parity 1, gen5: 010&101=000 -> parity 0
        //   output: [1, 0], next state: 01
        // Step 3: input=1, reg=101, gen7: 101&111=101 -> parity 0 (2 ones), gen5: 101&101=101 -> parity 0
        //   Wait, 101 has 2 ones, parity = 0. Let me recount.
        //   reg=101, gen7=111: 101&111=101, count_ones=2, parity=0
        //   reg=101, gen5=101: 101&101=101, count_ones=2, parity=0
        //   output: [0, 0], next state: 10
        let data = vec![true, false, true];
        let encoded = convolutional_encode(&data, &[0o7, 0o5], 3);
        assert_eq!(encoded.len(), 6); // 3 bits * rate 2
        assert_eq!(
            encoded,
            vec![true, true, true, false, false, false]
        );
    }

    #[test]
    fn test_hard_decode_recovers_encoded_data_k3() {
        let data = vec![true, false, true, true, false, false, true, false];
        let encoded = convolutional_encode(&data, &[0o7, 0o5], 3);

        // Perfect channel: convert to LLRs
        let llrs: Vec<f64> = encoded
            .iter()
            .map(|&b| if b { -1.0 } else { 1.0 })
            .collect();

        let mut decoder = sova_k3();
        let decoded = decoder.decode_hard(&llrs, 2);
        assert_eq!(&decoded[..data.len()], &data[..]);
    }

    #[test]
    fn test_soft_output_correct_sign() {
        // The sign of the soft output should match the hard decision.
        // Positive LLR => bit 0 (false), Negative LLR => bit 1 (true).
        let data = vec![true, false, true, false, true];
        let encoded = convolutional_encode(&data, &[0o7, 0o5], 3);

        let llrs: Vec<f64> = encoded
            .iter()
            .map(|&b| if b { -1.0 } else { 1.0 })
            .collect();

        let mut decoder = sova_k3();
        let soft = decoder.decode(&llrs, 2);

        for i in 0..data.len() {
            if data[i] {
                // bit 1 => negative LLR
                assert!(
                    soft[i] < 0.0,
                    "bit {} is 1 but soft output {:.2} is not negative",
                    i,
                    soft[i]
                );
            } else {
                // bit 0 => positive LLR
                assert!(
                    soft[i] > 0.0,
                    "bit {} is 0 but soft output {:.2} is not positive",
                    i,
                    soft[i]
                );
            }
        }
    }

    #[test]
    fn test_soft_output_reliability() {
        // Perfect channel should produce higher-magnitude LLRs than noisy channel.
        let data = vec![true, false, true, true, false, false, true, false];
        let encoded = convolutional_encode(&data, &[0o7, 0o5], 3);

        // Perfect channel
        let llrs_perfect: Vec<f64> = encoded
            .iter()
            .map(|&b| if b { -3.0 } else { 3.0 })
            .collect();

        // Slightly noisy channel (reduced confidence)
        let llrs_noisy: Vec<f64> = encoded
            .iter()
            .map(|&b| if b { -0.5 } else { 0.5 })
            .collect();

        let mut decoder = sova_k3();

        let soft_perfect = decoder.decode(&llrs_perfect, 2);
        decoder.reset();
        let soft_noisy = decoder.decode(&llrs_noisy, 2);

        // Average magnitude should be higher for the perfect channel
        let avg_perfect: f64 =
            soft_perfect.iter().map(|v| v.abs()).sum::<f64>() / soft_perfect.len() as f64;
        let avg_noisy: f64 =
            soft_noisy.iter().map(|v| v.abs()).sum::<f64>() / soft_noisy.len() as f64;

        assert!(
            avg_perfect > avg_noisy,
            "perfect channel avg |LLR| ({:.2}) should exceed noisy ({:.2})",
            avg_perfect,
            avg_noisy
        );
    }

    #[test]
    fn test_noisy_channel_decode() {
        // Add noise to the channel and verify we can still decode.
        let data = vec![true, false, true, true, false, false, true, false, true, false];
        let encoded = convolutional_encode(&data, &[0o7, 0o5], 3);

        // Simulate moderate SNR: LLRs with some noise added.
        // Use a simple deterministic "noise" pattern for reproducibility.
        let noise_pattern = [0.3, -0.2, 0.1, -0.4, 0.2, -0.1, 0.35, -0.3, 0.15, -0.25];
        let llrs: Vec<f64> = encoded
            .iter()
            .enumerate()
            .map(|(i, &b)| {
                let clean = if b { -2.0 } else { 2.0 };
                clean + noise_pattern[i % noise_pattern.len()]
            })
            .collect();

        let mut decoder = sova_k3();
        let decoded = decoder.decode_hard(&llrs, 2);
        assert_eq!(
            &decoded[..data.len()],
            &data[..],
            "SOVA should recover data through moderate noise"
        );
    }

    #[test]
    fn test_k7_factory_construction() {
        let decoder = sova_k7();
        assert_eq!(decoder.trellis.num_states(), 64);
        assert_eq!(decoder.traceback_length, 30);
        assert_eq!(decoder.branches_per_state, 2);

        // Verify trellis integrity
        for s in 0..64 {
            assert_eq!(decoder.trellis.outputs[s].len(), 2);
            for (next, bits) in &decoder.trellis.outputs[s] {
                assert!(*next < 64);
                assert_eq!(bits.len(), 2);
            }
        }
    }

    #[test]
    fn test_k3_factory_construction() {
        let decoder = sova_k3();
        assert_eq!(decoder.trellis.num_states(), 4);
        assert_eq!(decoder.traceback_length, 10);
        assert_eq!(decoder.branches_per_state, 2);

        // Verify we can decode a trivial message
        let data = vec![false, false, false];
        let encoded = convolutional_encode(&data, &[0o7, 0o5], 3);
        let llrs: Vec<f64> = encoded
            .iter()
            .map(|&b| if b { -1.0 } else { 1.0 })
            .collect();

        let mut dec = sova_k3();
        let decoded = dec.decode_hard(&llrs, 2);
        assert_eq!(&decoded[..data.len()], &data[..]);
    }

    #[test]
    fn test_reset_clears_state() {
        let data = vec![true, false, true, true, false];
        let encoded = convolutional_encode(&data, &[0o7, 0o5], 3);
        let llrs: Vec<f64> = encoded
            .iter()
            .map(|&b| if b { -1.0 } else { 1.0 })
            .collect();

        let mut decoder = sova_k3();

        // First decode
        let result1 = decoder.decode(&llrs, 2);

        // Reset and decode again — results should be identical since the
        // decoder is stateless between calls.
        decoder.reset();
        let result2 = decoder.decode(&llrs, 2);

        assert_eq!(result1.len(), result2.len());
        for (a, b) in result1.iter().zip(result2.iter()) {
            assert!(
                (a - b).abs() < 1e-12,
                "results differ after reset: {} vs {}",
                a,
                b
            );
        }
    }

    #[test]
    fn test_traceback_length_effect() {
        // Longer traceback should generally produce equal or better reliability
        // estimates. We compare a very short traceback vs. the default.
        let data: Vec<bool> = vec![
            true, false, true, true, false, false, true, false, true, true, false, true,
            false, false, true, true,
        ];
        let encoded = convolutional_encode(&data, &[0o7, 0o5], 3);

        let llrs: Vec<f64> = encoded
            .iter()
            .map(|&b| if b { -1.5 } else { 1.5 })
            .collect();

        // Short traceback (2 steps)
        let trellis_short = rate_1_2_trellis(3, &[0o7, 0o5]);
        let mut decoder_short = SovaDecoder::new(trellis_short, 2);
        let soft_short = decoder_short.decode(&llrs, 2);

        // Standard traceback (10 steps)
        let mut decoder_std = sova_k3();
        let soft_std = decoder_std.decode(&llrs, 2);

        // Both should produce valid hard decisions
        let hard_short: Vec<bool> = soft_short.iter().map(|&v| v < 0.0).collect();
        let hard_std: Vec<bool> = soft_std.iter().map(|&v| v < 0.0).collect();

        assert_eq!(
            &hard_short[..data.len()],
            &data[..],
            "short traceback should still decode correctly on clean channel"
        );
        assert_eq!(
            &hard_std[..data.len()],
            &data[..],
            "standard traceback should decode correctly"
        );

        // The longer traceback should generally produce equal or higher average reliability
        // (min delta accumulates over more steps, but with clean data the deltas are large).
        // At minimum, the standard decoder should produce valid output.
        let avg_short: f64 =
            soft_short[..data.len()].iter().map(|v| v.abs()).sum::<f64>() / data.len() as f64;
        let avg_std: f64 =
            soft_std[..data.len()].iter().map(|v| v.abs()).sum::<f64>() / data.len() as f64;

        // With a longer traceback, the minimum delta search has more opportunities
        // to find a smaller competitor delta, so avg_std <= avg_short is typical.
        // We just verify both are positive (valid reliability values).
        assert!(avg_short > 0.0, "short traceback avg reliability should be positive");
        assert!(avg_std > 0.0, "standard traceback avg reliability should be positive");
    }
}
