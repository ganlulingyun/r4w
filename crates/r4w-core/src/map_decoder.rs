//! MAP (Maximum A-Posteriori) Decoder — BCJR Forward-Backward Algorithm
//!
//! Implements the BCJR algorithm for soft-in / soft-out decoding of
//! convolutional codes. The MAP decoder computes exact a-posteriori
//! probabilities (or their log-domain equivalents) for each information
//! bit given received channel observations and a-priori information.
//!
//! This is the constituent decoder used inside turbo decoders and also
//! applicable standalone for any trellis-based code. Supports both true
//! MAP (log-sum-exp) and the reduced-complexity max-log-MAP approximation.
//!
//! LLR sign convention: **positive LLR means bit=0 is more likely**.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::map_decoder::{MapDecoder, MapConfig, encode_rsc, map_k3_rate_half};
//!
//! // Encode data with K=3 RSC (generators 7, 5 octal)
//! let data = vec![true, false, true, true, false, false, true, false];
//! let (systematic, parity) = encode_rsc(&data, 7, 5, 3);
//!
//! // Convert to LLRs (positive = bit 0 likely, negative = bit 1 likely)
//! let sys_llrs: Vec<f64> = systematic.iter()
//!     .map(|&b| if b { -4.0 } else { 4.0 })
//!     .collect();
//! let par_llrs: Vec<f64> = parity.iter()
//!     .map(|&b| if b { -4.0 } else { 4.0 })
//!     .collect();
//! let apriori = vec![0.0; data.len()];
//!
//! let mut decoder = map_k3_rate_half();
//! let extrinsic = decoder.decode(&sys_llrs, &par_llrs, &apriori);
//! // Extrinsic LLRs carry additional information beyond channel + apriori
//! assert_eq!(extrinsic.len(), data.len());
//! ```

/// Configuration for MAP decoder behavior.
#[derive(Debug, Clone)]
pub struct MapConfig {
    /// When true, use the max-log-MAP approximation (max instead of log-sum-exp).
    /// Faster but slightly sub-optimal compared to true MAP.
    pub use_max_log: bool,
}

impl Default for MapConfig {
    fn default() -> Self {
        Self { use_max_log: false }
    }
}

/// A single trellis transition: (from_state, to_state, input_bit, output_bits).
///
/// `output_bits` are the parity outputs as +1/-1 values stored as `i8`:
/// +1 represents bit=0, -1 represents bit=1.
#[derive(Debug, Clone)]
pub struct TrellisSection {
    /// Source state index.
    pub from_state: usize,
    /// Destination state index.
    pub to_state: usize,
    /// Input bit (true = 1, false = 0).
    pub input_bit: bool,
    /// Parity output bits for this transition.
    pub output_bits: Vec<i8>,
}

/// MAP decoder using the BCJR forward-backward algorithm.
///
/// Computes extrinsic log-likelihood ratios (LLRs) for each information bit
/// given systematic LLRs, parity LLRs, and a-priori LLRs. The output can
/// be fed to another decoder in an iterative (turbo) scheme.
#[derive(Debug, Clone)]
pub struct MapDecoder {
    /// Number of states in the trellis.
    num_states: usize,
    /// Trellis transitions grouped by input bit.
    /// transitions_for_bit[0] = transitions where input=0,
    /// transitions_for_bit[1] = transitions where input=1.
    transitions_for_bit: [Vec<TrellisSection>; 2],
    /// All transitions (for iteration).
    transitions: Vec<TrellisSection>,
    /// Forward state metrics (alpha), reused across calls.
    alpha: Vec<Vec<f64>>,
    /// Backward state metrics (beta), reused across calls.
    beta: Vec<Vec<f64>>,
    /// Decoder configuration.
    config: MapConfig,
}

impl MapDecoder {
    /// Create a new MAP decoder from trellis transition definitions.
    ///
    /// # Arguments
    /// * `num_states` - Number of states in the code trellis (2^(K-1))
    /// * `transitions` - Trellis edges as (from_state, to_state, input_bit, output_bits).
    ///   `output_bits` uses i8: +1 for bit=0, -1 for bit=1.
    ///
    /// # Panics
    /// Panics if `num_states` is 0.
    pub fn new(num_states: usize, transitions: Vec<(usize, usize, bool, Vec<i8>)>) -> Self {
        assert!(num_states > 0, "num_states must be > 0");

        let mut sections = Vec::with_capacity(transitions.len());
        let mut bit0 = Vec::new();
        let mut bit1 = Vec::new();

        for (from, to, input, outputs) in transitions {
            let section = TrellisSection {
                from_state: from,
                to_state: to,
                input_bit: input,
                output_bits: outputs,
            };
            if input {
                bit1.push(section.clone());
            } else {
                bit0.push(section.clone());
            }
            sections.push(section);
        }

        Self {
            num_states,
            transitions_for_bit: [bit0, bit1],
            transitions: sections,
            alpha: Vec::new(),
            beta: Vec::new(),
            config: MapConfig::default(),
        }
    }

    /// Create a MAP decoder with custom configuration.
    pub fn with_config(
        num_states: usize,
        transitions: Vec<(usize, usize, bool, Vec<i8>)>,
        config: MapConfig,
    ) -> Self {
        let mut decoder = Self::new(num_states, transitions);
        decoder.config = config;
        decoder
    }

    /// Run the BCJR algorithm and return extrinsic LLRs.
    ///
    /// # Arguments
    /// * `systematic_llrs` - Channel LLRs for systematic bits (positive = bit 0 likely)
    /// * `parity_llrs` - Channel LLRs for parity bits (positive = bit 0 likely)
    /// * `apriori_llrs` - A-priori LLRs from external source (e.g., other turbo decoder)
    ///
    /// # Returns
    /// Extrinsic LLRs: total_llr - systematic - apriori, one per information bit.
    ///
    /// All three input slices must have the same length.
    pub fn decode(
        &mut self,
        systematic_llrs: &[f64],
        parity_llrs: &[f64],
        apriori_llrs: &[f64],
    ) -> Vec<f64> {
        let n = systematic_llrs.len();
        if n == 0 {
            return Vec::new();
        }
        assert_eq!(
            parity_llrs.len(),
            n,
            "parity_llrs length must match systematic_llrs"
        );
        assert_eq!(
            apriori_llrs.len(),
            n,
            "apriori_llrs length must match systematic_llrs"
        );

        let combine = if self.config.use_max_log {
            max_log_map
        } else {
            log_sum_exp
        };

        // Allocate / resize alpha and beta
        self.alpha.clear();
        self.alpha
            .resize(n + 1, vec![f64::NEG_INFINITY; self.num_states]);
        self.beta.clear();
        self.beta
            .resize(n + 1, vec![f64::NEG_INFINITY; self.num_states]);

        // Initial conditions: start in state 0
        self.alpha[0][0] = 0.0;
        // Terminal condition: unterminated trellis — all ending states equally likely
        for s in 0..self.num_states {
            self.beta[n][s] = 0.0;
        }

        // --- Forward recursion (alpha) ---
        for k in 0..n {
            for tr in &self.transitions {
                let from = tr.from_state;
                let to = tr.to_state;
                if self.alpha[k][from] == f64::NEG_INFINITY {
                    continue;
                }
                let gamma = self.branch_metric(tr, systematic_llrs[k], parity_llrs[k], apriori_llrs[k]);
                let candidate = self.alpha[k][from] + gamma;
                self.alpha[k + 1][to] = combine(self.alpha[k + 1][to], candidate);
            }
        }

        // --- Backward recursion (beta) ---
        for k in (0..n).rev() {
            for tr in &self.transitions {
                let from = tr.from_state;
                let to = tr.to_state;
                if self.beta[k + 1][to] == f64::NEG_INFINITY {
                    continue;
                }
                let gamma = self.branch_metric(tr, systematic_llrs[k], parity_llrs[k], apriori_llrs[k]);
                let candidate = self.beta[k + 1][to] + gamma;
                self.beta[k][from] = combine(self.beta[k][from], candidate);
            }
        }

        // --- Compute extrinsic LLRs ---
        let mut extrinsic = vec![0.0; n];
        for k in 0..n {
            let mut llr_0 = f64::NEG_INFINITY;
            let mut llr_1 = f64::NEG_INFINITY;

            for tr in &self.transitions {
                let from = tr.from_state;
                let to = tr.to_state;
                if self.alpha[k][from] == f64::NEG_INFINITY {
                    continue;
                }
                if self.beta[k + 1][to] == f64::NEG_INFINITY {
                    continue;
                }
                // Gamma uses only parity contribution for extrinsic computation
                let parity_gamma = self.parity_metric(tr, parity_llrs[k]);
                let metric = self.alpha[k][from] + parity_gamma + self.beta[k + 1][to];

                if !tr.input_bit {
                    llr_0 = combine(llr_0, metric);
                } else {
                    llr_1 = combine(llr_1, metric);
                }
            }

            // Extrinsic = posterior LLR minus channel systematic minus apriori.
            // Since alpha/beta incorporate systematic+apriori from all time steps,
            // but the LLR sum uses only parity gamma, we subtract the current
            // bit's systematic and apriori to isolate the code-structure contribution.
            extrinsic[k] = (llr_0 - llr_1) - systematic_llrs[k] - apriori_llrs[k];
        }

        extrinsic
    }

    /// Reset internal state (clears cached alpha/beta buffers).
    pub fn reset(&mut self) {
        self.alpha.clear();
        self.beta.clear();
    }

    /// Compute branch metric (gamma) for a transition at time step k.
    ///
    /// gamma = (input_sign * (systematic + apriori) + parity_sign * parity) / 2
    fn branch_metric(
        &self,
        tr: &TrellisSection,
        systematic_llr: f64,
        parity_llr: f64,
        apriori_llr: f64,
    ) -> f64 {
        let input_sign: f64 = if tr.input_bit { -1.0 } else { 1.0 };
        let sys_component = input_sign * (systematic_llr + apriori_llr) / 2.0;

        let mut parity_component = 0.0;
        // For rate-1/2 codes, output_bits has one element representing the parity
        // For higher-rate codes, multiple parity bits are summed
        if !tr.output_bits.is_empty() {
            let parity_sign = tr.output_bits[0] as f64; // +1 or -1
            parity_component = parity_sign * parity_llr / 2.0;
        }

        sys_component + parity_component
    }

    /// Compute parity-only branch metric (used for extrinsic LLR calculation).
    fn parity_metric(&self, tr: &TrellisSection, parity_llr: f64) -> f64 {
        if tr.output_bits.is_empty() {
            return 0.0;
        }
        let parity_sign = tr.output_bits[0] as f64;
        parity_sign * parity_llr / 2.0
    }
}

/// Numerically stable log-sum-exp: log(exp(a) + exp(b)).
///
/// Computed as max(a, b) + ln(1 + exp(-|a - b|)) to avoid overflow/underflow.
/// Returns exact result for `NEG_INFINITY` inputs.
pub fn log_sum_exp(a: f64, b: f64) -> f64 {
    if a == f64::NEG_INFINITY {
        return b;
    }
    if b == f64::NEG_INFINITY {
        return a;
    }
    let max_val = a.max(b);
    let diff = (a - b).abs();
    if diff > 30.0 {
        return max_val; // Correction term negligible
    }
    max_val + (1.0 + (-diff).exp()).ln()
}

/// Max-log-MAP approximation: max(a, b).
///
/// Simplified version of log-sum-exp that drops the correction term.
/// Faster but introduces ~0.2 dB performance loss compared to true MAP.
pub fn max_log_map(a: f64, b: f64) -> f64 {
    a.max(b)
}

/// Create a MAP decoder for the standard K=3, rate-1/2 RSC code.
///
/// Generator polynomials: feedback = 7 (octal, 111 binary),
/// feedforward = 5 (octal, 101 binary). This is the constituent code
/// used in 3G UMTS turbo codes and many academic examples.
///
/// Trellis has 4 states (2^(K-1) = 4).
pub fn map_k3_rate_half() -> MapDecoder {
    let num_states = 4; // 2^(3-1)
    let feedback: u32 = 7;   // octal 7 = 0b111
    let feedforward: u32 = 5; // octal 5 = 0b101
    let memory = 2; // K - 1

    let mut transitions = Vec::new();

    for state in 0..num_states {
        for input in 0..2u32 {
            // Feedback: XOR of state bits selected by feedback poly (excluding MSB)
            let fb_bits = (state as u32) & (feedback >> 1);
            let fb = (fb_bits.count_ones() % 2) ^ input;

            // Parity output
            let ff_bits = (state as u32) & (feedforward >> 1);
            let parity = (ff_bits.count_ones() % 2) ^ fb;

            // Next state
            let next_state =
                ((fb << (memory - 1)) | ((state as u32) >> 1)) & ((1u32 << memory) - 1);

            // Output bit as +1/-1: bit=0 -> +1, bit=1 -> -1
            let parity_signed: i8 = if parity == 0 { 1 } else { -1 };

            transitions.push((
                state,
                next_state as usize,
                input == 1,
                vec![parity_signed],
            ));
        }
    }

    MapDecoder::new(num_states, transitions)
}

/// Recursive Systematic Convolutional (RSC) encoder.
///
/// Encodes `data` using a rate-1/2 RSC code defined by `generator_fb` (feedback)
/// and `generator_ff` (feedforward) polynomials in octal notation, with
/// constraint length `k`.
///
/// # Returns
/// `(systematic, parity)` — systematic bits are the input data, parity bits
/// are generated by the RSC encoder.
///
/// # Arguments
/// * `data` - Input data bits
/// * `generator_fb` - Feedback generator polynomial (octal)
/// * `generator_ff` - Feedforward generator polynomial (octal)
/// * `k` - Constraint length
pub fn encode_rsc(
    data: &[bool],
    generator_fb: u32,
    generator_ff: u32,
    k: usize,
) -> (Vec<bool>, Vec<bool>) {
    assert!(k >= 2, "Constraint length must be >= 2");
    let memory = k - 1;
    let state_mask = (1u32 << memory) - 1;
    let mut state: u32 = 0;

    let systematic: Vec<bool> = data.to_vec();
    let mut parity = Vec::with_capacity(data.len());

    for &bit in data {
        let input = bit as u32;

        // Feedback: XOR of state bits selected by feedback poly with input
        let fb_bits = state & (generator_fb >> 1);
        let fb = (fb_bits.count_ones() % 2) ^ input;

        // Parity output
        let ff_bits = state & (generator_ff >> 1);
        let p = (ff_bits.count_ones() % 2) ^ fb;
        parity.push(p == 1);

        // Update state: shift in feedback result
        state = ((fb << (memory - 1)) | (state >> 1)) & state_mask;
    }

    (systematic, parity)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_construction() {
        // K=3 rate-1/2: 4 states, 8 transitions (4 states x 2 inputs)
        let decoder = map_k3_rate_half();
        assert_eq!(decoder.num_states, 4);
        assert_eq!(decoder.transitions.len(), 8);
        // Each input bit (0 and 1) should have 4 transitions
        assert_eq!(decoder.transitions_for_bit[0].len(), 4);
        assert_eq!(decoder.transitions_for_bit[1].len(), 4);
    }

    #[test]
    fn test_rsc_encoding() {
        // Encode known data with K=3, generators (7, 5)
        let data = vec![true, false, true, true, false];
        let (sys, par) = encode_rsc(&data, 7, 5, 3);

        // Systematic output must equal input
        assert_eq!(sys, data);
        // Parity must have same length as input
        assert_eq!(par.len(), data.len());
        // Parity for all-zero input should be all-zero (since state starts at 0)
        let (_, par_zeros) = encode_rsc(&vec![false; 8], 7, 5, 3);
        assert!(par_zeros.iter().all(|&b| !b), "All-zero input should yield all-zero parity");
    }

    #[test]
    fn test_decode_recovers_encoded_data() {
        let data = vec![true, false, true, true, false, false, true, false];
        let (sys, par) = encode_rsc(&data, 7, 5, 3);

        // Convert to strong LLRs
        let sys_llrs: Vec<f64> = sys.iter().map(|&b| if b { -6.0 } else { 6.0 }).collect();
        let par_llrs: Vec<f64> = par.iter().map(|&b| if b { -6.0 } else { 6.0 }).collect();
        let apriori = vec![0.0; data.len()];

        let mut decoder = map_k3_rate_half();
        let extrinsic = decoder.decode(&sys_llrs, &par_llrs, &apriori);

        // Hard-decide using systematic + extrinsic
        let decoded: Vec<bool> = sys_llrs
            .iter()
            .zip(extrinsic.iter())
            .map(|(s, e)| (s + e) < 0.0)
            .collect();
        assert_eq!(
            decoded, data,
            "Clean decode with strong LLRs should perfectly recover data"
        );
    }

    #[test]
    fn test_extrinsic_llr_sign_correctness() {
        // For clean channel, extrinsic LLRs should reinforce the correct bit
        let data = vec![true, false, true, false, true, false, true, false];
        let (sys, par) = encode_rsc(&data, 7, 5, 3);

        let sys_llrs: Vec<f64> = sys.iter().map(|&b| if b { -5.0 } else { 5.0 }).collect();
        let par_llrs: Vec<f64> = par.iter().map(|&b| if b { -5.0 } else { 5.0 }).collect();
        let apriori = vec![0.0; data.len()];

        let mut decoder = map_k3_rate_half();
        let extrinsic = decoder.decode(&sys_llrs, &par_llrs, &apriori);

        // Extrinsic LLR sign should agree with systematic for most bits
        let mut agree_count = 0;
        for (i, &ext) in extrinsic.iter().enumerate() {
            // For bit=1 (true), both sys_llr and ext should be negative
            // For bit=0 (false), both should be positive
            let expected_sign = if data[i] { -1.0 } else { 1.0 };
            if ext * expected_sign > 0.0 {
                agree_count += 1;
            }
        }
        assert!(
            agree_count >= 6,
            "At least 6/8 extrinsic LLRs should have correct sign, got {agree_count}"
        );
    }

    #[test]
    fn test_log_sum_exp_numerical_stability() {
        // Basic: log(exp(0) + exp(0)) = ln(2)
        let result = log_sum_exp(0.0, 0.0);
        assert!(
            (result - 2.0f64.ln()).abs() < 1e-10,
            "log_sum_exp(0, 0) should be ln(2)"
        );

        // Large difference: log(exp(100) + exp(0)) ~ 100
        let result = log_sum_exp(100.0, 0.0);
        assert!(
            (result - 100.0).abs() < 1e-6,
            "log_sum_exp(100, 0) should be ~100"
        );

        // Very large values: should not overflow
        let result = log_sum_exp(700.0, 700.0);
        assert!(
            (result - (700.0 + 2.0f64.ln())).abs() < 1e-6,
            "log_sum_exp(700, 700) should be 700 + ln(2)"
        );

        // NEG_INFINITY handling
        assert_eq!(log_sum_exp(f64::NEG_INFINITY, 5.0), 5.0);
        assert_eq!(log_sum_exp(3.0, f64::NEG_INFINITY), 3.0);
        assert_eq!(
            log_sum_exp(f64::NEG_INFINITY, f64::NEG_INFINITY),
            f64::NEG_INFINITY
        );

        // Symmetry
        assert!(
            (log_sum_exp(2.0, 5.0) - log_sum_exp(5.0, 2.0)).abs() < 1e-15,
            "log_sum_exp should be symmetric"
        );
    }

    #[test]
    fn test_max_log_map_approximation() {
        // max_log_map should return the max
        assert_eq!(max_log_map(3.0, 5.0), 5.0);
        assert_eq!(max_log_map(5.0, 3.0), 5.0);
        assert_eq!(max_log_map(-10.0, -20.0), -10.0);

        // Compare with log_sum_exp: max_log_map <= log_sum_exp (always)
        for &(a, b) in &[(0.0, 0.0), (1.0, 2.0), (-3.0, 4.0), (10.0, 10.0)] {
            let exact = log_sum_exp(a, b);
            let approx = max_log_map(a, b);
            assert!(
                approx <= exact + 1e-10,
                "max_log_map({a}, {b}) = {approx} should be <= log_sum_exp = {exact}"
            );
        }

        // Verify max-log-MAP decoder still produces reasonable output
        let data = vec![true, false, true, true, false, false, true, false];
        let (sys, par) = encode_rsc(&data, 7, 5, 3);
        let sys_llrs: Vec<f64> = sys.iter().map(|&b| if b { -5.0 } else { 5.0 }).collect();
        let par_llrs: Vec<f64> = par.iter().map(|&b| if b { -5.0 } else { 5.0 }).collect();
        let apriori = vec![0.0; data.len()];

        let config = MapConfig { use_max_log: true };
        let transitions = build_k3_transitions();
        let mut decoder = MapDecoder::with_config(4, transitions, config);
        let extrinsic = decoder.decode(&sys_llrs, &par_llrs, &apriori);

        // Should still decode correctly with clean input
        let decoded: Vec<bool> = sys_llrs
            .iter()
            .zip(extrinsic.iter())
            .map(|(s, e)| (s + e) < 0.0)
            .collect();
        assert_eq!(decoded, data, "Max-log-MAP should decode clean signal correctly");
    }

    #[test]
    fn test_zero_apriori() {
        // With zero a-priori LLRs, decoder should still work using only channel info
        let data = vec![false, true, false, true, true, false, false, true];
        let (sys, par) = encode_rsc(&data, 7, 5, 3);

        let sys_llrs: Vec<f64> = sys.iter().map(|&b| if b { -4.0 } else { 4.0 }).collect();
        let par_llrs: Vec<f64> = par.iter().map(|&b| if b { -4.0 } else { 4.0 }).collect();
        let apriori = vec![0.0; data.len()]; // No prior information

        let mut decoder = map_k3_rate_half();
        let extrinsic = decoder.decode(&sys_llrs, &par_llrs, &apriori);

        // Extrinsic should not be all zeros (decoder extracts information from code structure)
        let non_zero = extrinsic.iter().any(|&e| e.abs() > 1e-10);
        assert!(non_zero, "Extrinsic LLRs should be non-zero even with zero apriori");

        // Hard decision should recover data
        let decoded: Vec<bool> = sys_llrs
            .iter()
            .zip(extrinsic.iter())
            .map(|(s, e)| (s + e) < 0.0)
            .collect();
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_apriori_helps_decoding() {
        // When channel observations are weak, a-priori information should
        // influence the decoder output. We use weak LLRs so the trellis
        // is not dominated by a single path.
        let data = vec![true, false, true, false, false, true, false, true];
        let (sys, par) = encode_rsc(&data, 7, 5, 3);

        // Use weak channel LLRs (magnitude 1.0) so the apriori can matter
        let sys_llrs: Vec<f64> = sys.iter().map(|&b| if b { -1.0 } else { 1.0 }).collect();
        let par_llrs: Vec<f64> = par.iter().map(|&b| if b { -1.0 } else { 1.0 }).collect();

        // Decode without apriori
        let apriori_none = vec![0.0; data.len()];
        let mut decoder = map_k3_rate_half();
        let ext_no_apriori = decoder.decode(&sys_llrs, &par_llrs, &apriori_none);

        // Decode with correct apriori bias for all bits
        let apriori_help: Vec<f64> = data.iter().map(|&b| if b { -2.0 } else { 2.0 }).collect();
        decoder.reset();
        let ext_with_apriori = decoder.decode(&sys_llrs, &par_llrs, &apriori_help);

        // The extrinsic LLRs should differ when apriori is provided,
        // because apriori shifts the trellis path probabilities which
        // propagates through alpha/beta to neighboring positions.
        let total_diff: f64 = ext_no_apriori
            .iter()
            .zip(ext_with_apriori.iter())
            .map(|(a, b)| (a - b).abs())
            .sum();
        assert!(
            total_diff > 0.01,
            "Apriori should influence extrinsic LLRs, total diff = {total_diff:.6}"
        );

        // With correct apriori, hard decisions on the posterior should be correct
        let mut correct_with = 0;
        let mut correct_without = 0;
        for i in 0..data.len() {
            let posterior_no = sys_llrs[i] + ext_no_apriori[i];
            let posterior_with = sys_llrs[i] + apriori_help[i] + ext_with_apriori[i];
            if (posterior_no < 0.0) == data[i] {
                correct_without += 1;
            }
            if (posterior_with < 0.0) == data[i] {
                correct_with += 1;
            }
        }
        assert!(
            correct_with >= correct_without,
            "Correct apriori should not degrade decoding: \
             with={correct_with}, without={correct_without}"
        );
    }

    #[test]
    fn test_k3_factory() {
        let decoder = map_k3_rate_half();

        // Verify trellis structure: K=3 means 4 states, 2 transitions per state
        assert_eq!(decoder.num_states, 4);
        assert_eq!(decoder.transitions.len(), 8);

        // Check that all transitions reference valid states
        for tr in &decoder.transitions {
            assert!(tr.from_state < 4, "from_state must be < 4");
            assert!(tr.to_state < 4, "to_state must be < 4");
            assert_eq!(tr.output_bits.len(), 1, "Rate-1/2 should have 1 parity bit");
            assert!(
                tr.output_bits[0] == 1 || tr.output_bits[0] == -1,
                "Parity bit should be +1 or -1"
            );
        }

        // Verify state 0 with input 0 goes to state 0 (all-zero path)
        let s0_i0: Vec<&TrellisSection> = decoder
            .transitions
            .iter()
            .filter(|t| t.from_state == 0 && !t.input_bit)
            .collect();
        assert_eq!(s0_i0.len(), 1);
        assert_eq!(s0_i0[0].to_state, 0, "State 0 + input 0 should stay at state 0");
        assert_eq!(s0_i0[0].output_bits[0], 1, "All-zero path should output parity=0 (+1)");
    }

    #[test]
    fn test_reset_clears_state() {
        let data = vec![true, false, true, false, true, false, true, false];
        let (sys, par) = encode_rsc(&data, 7, 5, 3);
        let sys_llrs: Vec<f64> = sys.iter().map(|&b| if b { -5.0 } else { 5.0 }).collect();
        let par_llrs: Vec<f64> = par.iter().map(|&b| if b { -5.0 } else { 5.0 }).collect();
        let apriori = vec![0.0; data.len()];

        let mut decoder = map_k3_rate_half();

        // First decode populates internal buffers
        let ext1 = decoder.decode(&sys_llrs, &par_llrs, &apriori);
        assert!(!decoder.alpha.is_empty(), "Alpha should be populated after decode");
        assert!(!decoder.beta.is_empty(), "Beta should be populated after decode");

        // Reset clears them
        decoder.reset();
        assert!(decoder.alpha.is_empty(), "Alpha should be empty after reset");
        assert!(decoder.beta.is_empty(), "Beta should be empty after reset");

        // Decode again after reset should give same result
        let ext2 = decoder.decode(&sys_llrs, &par_llrs, &apriori);
        assert_eq!(ext1.len(), ext2.len());
        for (a, b) in ext1.iter().zip(ext2.iter()) {
            assert!(
                (a - b).abs() < 1e-10,
                "Decode after reset should give identical results"
            );
        }
    }

    // Helper: build K=3 rate-1/2 trellis transitions for direct construction tests
    fn build_k3_transitions() -> Vec<(usize, usize, bool, Vec<i8>)> {
        let feedback: u32 = 7;
        let feedforward: u32 = 5;
        let memory = 2;
        let num_states = 4;
        let mut transitions = Vec::new();

        for state in 0..num_states {
            for input in 0..2u32 {
                let fb_bits = (state as u32) & (feedback >> 1);
                let fb = (fb_bits.count_ones() % 2) ^ input;
                let ff_bits = (state as u32) & (feedforward >> 1);
                let parity = (ff_bits.count_ones() % 2) ^ fb;
                let next_state =
                    ((fb << (memory - 1)) | ((state as u32) >> 1)) & ((1u32 << memory) - 1);
                let parity_signed: i8 = if parity == 0 { 1 } else { -1 };
                transitions.push((state, next_state as usize, input == 1, vec![parity_signed]));
            }
        }
        transitions
    }
}
