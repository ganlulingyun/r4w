//! Iterative belief propagation (message passing) for soft-decision decoding on factor graphs.
//!
//! This module implements the sum-product and min-sum algorithms for decoding
//! linear block codes defined by parity-check matrices. Messages are passed in
//! the log-likelihood ratio (LLR) domain for numerical stability.
//!
//! # Overview
//!
//! A factor graph consists of **variable nodes** (one per codeword bit) and
//! **check nodes** (one per parity-check equation). Edges connect variable
//! nodes to the check nodes they participate in. The decoder iteratively
//! refines soft information until a valid codeword is found or a maximum
//! iteration count is reached.
//!
//! Two algorithms are provided:
//!
//! - **Sum-Product** (exact belief propagation): computes exact marginal
//!   posteriors via `tanh` operations. Higher accuracy but more expensive.
//! - **Min-Sum** (approximate): replaces `tanh`/`atanh` with `min` and `sign`
//!   products. Lower complexity, suitable for hardware implementations.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::belief_propagation_decoder::{
//!     BeliefPropagationDecoder, BpAlgorithm, ParityCheckMatrix,
//! };
//!
//! // Simple (7,4) Hamming code parity-check matrix
//! let h = ParityCheckMatrix::from_dense(
//!     3, 7,
//!     &[
//!         1, 0, 0, 1, 1, 0, 1,
//!         0, 1, 0, 1, 0, 1, 1,
//!         0, 0, 1, 0, 1, 1, 1,
//!     ],
//! );
//!
//! let mut decoder = BeliefPropagationDecoder::new(h, BpAlgorithm::SumProduct, 50);
//!
//! // Channel LLRs: positive = likely 0, negative = likely 1
//! // Encode all-zeros codeword, add a bit of noise
//! let channel_llrs = vec![2.0, 3.0, 1.5, 2.5, 1.0, 3.5, 2.0];
//!
//! let result = decoder.decode(&channel_llrs);
//! assert!(result.converged);
//! assert_eq!(result.hard_decision, vec![0u8; 7]);
//! ```

use std::f64;

// ---------------------------------------------------------------------------
// Parity-check matrix
// ---------------------------------------------------------------------------

/// Sparse representation of a binary parity-check matrix **H** (m x n).
///
/// Internally stored as adjacency lists for both rows (check nodes) and
/// columns (variable nodes) to allow efficient message passing in both
/// directions.
#[derive(Debug, Clone)]
pub struct ParityCheckMatrix {
    /// Number of check nodes (rows).
    pub num_checks: usize,
    /// Number of variable nodes (columns / codeword bits).
    pub num_vars: usize,
    /// For each check node, the sorted list of variable-node indices with a 1.
    check_to_var: Vec<Vec<usize>>,
    /// For each variable node, the sorted list of check-node indices with a 1.
    var_to_check: Vec<Vec<usize>>,
}

impl ParityCheckMatrix {
    /// Build from a dense row-major binary matrix (0/1 values).
    ///
    /// `rows` is the number of check equations (m), `cols` the codeword length
    /// (n). `data` must have exactly `rows * cols` elements.
    ///
    /// # Panics
    ///
    /// Panics if `data.len() != rows * cols`.
    pub fn from_dense(rows: usize, cols: usize, data: &[u8]) -> Self {
        assert_eq!(
            data.len(),
            rows * cols,
            "data length must equal rows * cols"
        );
        let mut check_to_var = vec![Vec::new(); rows];
        let mut var_to_check = vec![Vec::new(); cols];

        for r in 0..rows {
            for c in 0..cols {
                if data[r * cols + c] != 0 {
                    check_to_var[r].push(c);
                    var_to_check[c].push(r);
                }
            }
        }

        Self {
            num_checks: rows,
            num_vars: cols,
            check_to_var,
            var_to_check,
        }
    }

    /// Build from sparse coordinate lists.
    ///
    /// Each `(row, col)` pair indicates a 1-entry in **H**.
    pub fn from_sparse(rows: usize, cols: usize, entries: &[(usize, usize)]) -> Self {
        let mut check_to_var = vec![Vec::new(); rows];
        let mut var_to_check = vec![Vec::new(); cols];

        for &(r, c) in entries {
            assert!(r < rows && c < cols, "entry ({r}, {c}) out of bounds");
            check_to_var[r].push(c);
            var_to_check[c].push(r);
        }

        // Ensure sorted and deduplicated.
        for list in check_to_var.iter_mut() {
            list.sort_unstable();
            list.dedup();
        }
        for list in var_to_check.iter_mut() {
            list.sort_unstable();
            list.dedup();
        }

        Self {
            num_checks: rows,
            num_vars: cols,
            check_to_var,
            var_to_check,
        }
    }

    /// Return the variable-node neighbours of check node `c`.
    #[inline]
    pub fn check_neighbors(&self, c: usize) -> &[usize] {
        &self.check_to_var[c]
    }

    /// Return the check-node neighbours of variable node `v`.
    #[inline]
    pub fn var_neighbors(&self, v: usize) -> &[usize] {
        &self.var_to_check[v]
    }

    /// Total number of edges (1-entries) in **H**.
    pub fn num_edges(&self) -> usize {
        self.check_to_var.iter().map(|v| v.len()).sum()
    }
}

// ---------------------------------------------------------------------------
// Algorithm selector
// ---------------------------------------------------------------------------

/// Algorithm used for belief propagation message updates.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BpAlgorithm {
    /// Exact sum-product (log-domain with `tanh` / `atanh`).
    SumProduct,
    /// Approximate min-sum (sign + minimum magnitude).
    MinSum,
    /// Scaled min-sum with a multiplicative correction factor (typically 0.75).
    ScaledMinSum(ScaledMinSumParams),
}

/// Parameters for scaled min-sum.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScaledMinSumParams {
    /// Scaling factor applied to min-sum check-to-variable messages.
    /// A value of ~0.75 is commonly used.
    pub alpha: f64,
}

impl Eq for ScaledMinSumParams {}

// ---------------------------------------------------------------------------
// Decode result
// ---------------------------------------------------------------------------

/// Output of a belief propagation decode attempt.
#[derive(Debug, Clone)]
pub struct BpDecodeResult {
    /// Hard-decision output (0 or 1 per bit).
    pub hard_decision: Vec<u8>,
    /// A-posteriori LLRs for each bit.
    pub llrs: Vec<f64>,
    /// Number of iterations performed.
    pub iterations: usize,
    /// Whether the decoder converged to a valid codeword (all syndrome bits 0).
    pub converged: bool,
}

// ---------------------------------------------------------------------------
// Decoder
// ---------------------------------------------------------------------------

/// Iterative belief propagation decoder for codes defined by a parity-check
/// matrix.
#[derive(Debug, Clone)]
pub struct BeliefPropagationDecoder {
    /// Parity-check matrix defining the code.
    h: ParityCheckMatrix,
    /// Decoding algorithm.
    algorithm: BpAlgorithm,
    /// Maximum number of iterations.
    max_iters: usize,
    /// Messages from variable nodes to check nodes, indexed by edge.
    /// Edge order: for each variable v, for each check c in var_neighbors(v).
    var_to_check_msgs: Vec<f64>,
    /// Messages from check nodes to variable nodes, same edge order.
    check_to_var_msgs: Vec<f64>,
    /// Edge-index lookup: `var_edge_start[v]` is the first edge index for variable v.
    var_edge_start: Vec<usize>,
    /// Edge-index lookup: for check node c, for each var in check_neighbors(c),
    /// the corresponding edge index in the variable-centric ordering.
    check_edge_indices: Vec<Vec<usize>>,
}

impl BeliefPropagationDecoder {
    /// Create a new decoder for the given parity-check matrix and algorithm.
    ///
    /// `max_iters` controls the maximum number of message-passing iterations.
    pub fn new(h: ParityCheckMatrix, algorithm: BpAlgorithm, max_iters: usize) -> Self {
        let num_edges = h.num_edges();

        // Build var_edge_start: cumulative count of edges per variable node.
        let mut var_edge_start = Vec::with_capacity(h.num_vars + 1);
        var_edge_start.push(0);
        for v in 0..h.num_vars {
            let prev = *var_edge_start.last().unwrap();
            var_edge_start.push(prev + h.var_neighbors(v).len());
        }

        // Build check_edge_indices: for each check c, for each var in
        // check_neighbors(c), find the edge index in the var-centric order.
        let mut check_edge_indices = Vec::with_capacity(h.num_checks);
        for c in 0..h.num_checks {
            let mut indices = Vec::with_capacity(h.check_neighbors(c).len());
            for &v in h.check_neighbors(c) {
                // Find position of c in var_neighbors(v).
                let pos = h
                    .var_neighbors(v)
                    .iter()
                    .position(|&cc| cc == c)
                    .expect("inconsistent adjacency");
                indices.push(var_edge_start[v] + pos);
            }
            check_edge_indices.push(indices);
        }

        Self {
            h,
            algorithm,
            max_iters,
            var_to_check_msgs: vec![0.0; num_edges],
            check_to_var_msgs: vec![0.0; num_edges],
            var_edge_start,
            check_edge_indices,
        }
    }

    /// Change the decoding algorithm.
    pub fn set_algorithm(&mut self, algorithm: BpAlgorithm) {
        self.algorithm = algorithm;
    }

    /// Change the maximum iteration count.
    pub fn set_max_iters(&mut self, max_iters: usize) {
        self.max_iters = max_iters;
    }

    /// Return a reference to the parity-check matrix.
    pub fn parity_check_matrix(&self) -> &ParityCheckMatrix {
        &self.h
    }

    /// Decode a received word given channel LLRs.
    ///
    /// `channel_llrs` must have length equal to `h.num_vars`. Positive LLR
    /// means bit 0 is more likely; negative means bit 1 is more likely.
    ///
    /// Returns [`BpDecodeResult`] with hard decisions, posterior LLRs,
    /// iteration count, and convergence flag.
    ///
    /// # Panics
    ///
    /// Panics if `channel_llrs.len() != h.num_vars`.
    pub fn decode(&mut self, channel_llrs: &[f64]) -> BpDecodeResult {
        let n = self.h.num_vars;
        assert_eq!(channel_llrs.len(), n, "LLR length must match codeword length");

        // Initialise variable-to-check messages with channel LLRs.
        for v in 0..n {
            let start = self.var_edge_start[v];
            let end = self.var_edge_start[v + 1];
            for idx in start..end {
                self.var_to_check_msgs[idx] = channel_llrs[v];
            }
        }

        // Clear check-to-variable messages.
        for m in self.check_to_var_msgs.iter_mut() {
            *m = 0.0;
        }

        let mut posterior_llrs = vec![0.0f64; n];
        let mut hard = vec![0u8; n];

        for iter in 0..self.max_iters {
            // --- Check-node update ---
            self.check_node_update();

            // --- Variable-node update & posterior computation ---
            for v in 0..n {
                let start = self.var_edge_start[v];
                let end = self.var_edge_start[v + 1];
                let degree = end - start;

                // Compute sum of all incoming check-to-var messages.
                let mut total: f64 = channel_llrs[v];
                for idx in start..end {
                    total += self.check_to_var_msgs[idx];
                }

                posterior_llrs[v] = total;
                hard[v] = if total < 0.0 { 1 } else { 0 };

                // Extrinsic messages: total minus the incoming from each check.
                for i in 0..degree {
                    let idx = start + i;
                    self.var_to_check_msgs[idx] = total - self.check_to_var_msgs[idx];
                }
            }

            // --- Syndrome check ---
            if self.syndrome_check(&hard) {
                return BpDecodeResult {
                    hard_decision: hard,
                    llrs: posterior_llrs,
                    iterations: iter + 1,
                    converged: true,
                };
            }
        }

        BpDecodeResult {
            hard_decision: hard,
            llrs: posterior_llrs,
            iterations: self.max_iters,
            converged: false,
        }
    }

    /// Perform the check-node update step according to the selected algorithm.
    fn check_node_update(&mut self) {
        match self.algorithm {
            BpAlgorithm::SumProduct => self.check_node_update_sum_product(),
            BpAlgorithm::MinSum => self.check_node_update_min_sum(1.0),
            BpAlgorithm::ScaledMinSum(params) => self.check_node_update_min_sum(params.alpha),
        }
    }

    /// Sum-product check-node update in log domain.
    ///
    /// For each check node c and each connected variable v, the outgoing
    /// message is:
    ///
    /// ```text
    /// L(c->v) = 2 * atanh( prod_{v' in N(c)\v} tanh(L(v'->c) / 2) )
    /// ```
    fn check_node_update_sum_product(&mut self) {
        for c in 0..self.h.num_checks {
            let vars = self.h.check_neighbors(c);
            let edge_indices = &self.check_edge_indices[c];
            let degree = vars.len();

            if degree == 0 {
                continue;
            }

            // Precompute tanh(msg/2) for each incoming var-to-check message.
            let mut tanh_vals: Vec<f64> = Vec::with_capacity(degree);
            for &eidx in edge_indices.iter() {
                let half = self.var_to_check_msgs[eidx] * 0.5;
                tanh_vals.push(half.tanh());
            }

            // For each variable connected to this check, compute the product
            // of all *other* tanh values.
            for i in 0..degree {
                let mut product = 1.0f64;
                for (j, &tv) in tanh_vals.iter().enumerate() {
                    if j != i {
                        product *= tv;
                    }
                }
                // Clamp to avoid atanh(+-1) = +-inf.
                let clamped = product.clamp(-1.0 + 1e-15, 1.0 - 1e-15);
                self.check_to_var_msgs[edge_indices[i]] = 2.0 * clamped.atanh();
            }
        }
    }

    /// Min-sum check-node update (optionally scaled).
    ///
    /// For each check node c and variable v:
    ///
    /// ```text
    /// L(c->v) = alpha * (prod_{v'!=v} sign(L(v'->c))) * min_{v'!=v} |L(v'->c)|
    /// ```
    fn check_node_update_min_sum(&mut self, alpha: f64) {
        for c in 0..self.h.num_checks {
            let edge_indices = &self.check_edge_indices[c];
            let degree = edge_indices.len();

            if degree == 0 {
                continue;
            }

            // Collect magnitudes and sign product.
            let mut magnitudes: Vec<f64> = Vec::with_capacity(degree);
            let mut signs: Vec<f64> = Vec::with_capacity(degree);
            let mut sign_product: f64 = 1.0;
            // Track the two smallest magnitudes for efficient exclusion.
            let mut min1 = f64::INFINITY;
            let mut min1_idx = 0usize;
            let mut min2 = f64::INFINITY;

            for (i, &eidx) in edge_indices.iter().enumerate() {
                let msg = self.var_to_check_msgs[eidx];
                let mag = msg.abs();
                let sgn = if msg >= 0.0 { 1.0 } else { -1.0 };
                magnitudes.push(mag);
                signs.push(sgn);
                sign_product *= sgn;

                if mag < min1 {
                    min2 = min1;
                    min1 = mag;
                    min1_idx = i;
                } else if mag < min2 {
                    min2 = mag;
                }
            }

            for i in 0..degree {
                // Sign: product of all other signs = total product / this sign.
                let excl_sign = sign_product * signs[i]; // sign[i] is +-1 so *sign = /sign
                // Magnitude: min of all other magnitudes.
                let excl_min = if i == min1_idx { min2 } else { min1 };
                self.check_to_var_msgs[edge_indices[i]] = alpha * excl_sign * excl_min;
            }
        }
    }

    /// Check whether a hard-decision vector satisfies all parity checks.
    ///
    /// Returns `true` if **H** * **x** = **0** (mod 2).
    pub fn syndrome_check(&self, hard_bits: &[u8]) -> bool {
        for c in 0..self.h.num_checks {
            let mut sum = 0u8;
            for &v in self.h.check_neighbors(c) {
                sum ^= hard_bits[v];
            }
            if sum != 0 {
                return false;
            }
        }
        true
    }

    /// Compute the syndrome vector **s** = **H** * **x** (mod 2).
    pub fn syndrome(&self, hard_bits: &[u8]) -> Vec<u8> {
        let mut s = vec![0u8; self.h.num_checks];
        for c in 0..self.h.num_checks {
            for &v in self.h.check_neighbors(c) {
                s[c] ^= hard_bits[v];
            }
        }
        s
    }
}

// ---------------------------------------------------------------------------
// BER estimation
// ---------------------------------------------------------------------------

/// Estimate the bit-error rate from soft LLRs given known transmitted bits.
///
/// `tx_bits` are the transmitted bits (0 or 1), `llrs` are the posterior or
/// channel LLRs. A positive LLR corresponds to a hard-decision 0.
pub fn estimate_ber(tx_bits: &[u8], llrs: &[f64]) -> f64 {
    assert_eq!(tx_bits.len(), llrs.len(), "length mismatch");
    if tx_bits.is_empty() {
        return 0.0;
    }
    let errors: usize = tx_bits
        .iter()
        .zip(llrs.iter())
        .filter(|(&b, &l)| {
            let hard = if l < 0.0 { 1u8 } else { 0u8 };
            hard != b
        })
        .count();
    errors as f64 / tx_bits.len() as f64
}

/// Make hard decisions from LLRs.
///
/// Positive LLR -> 0, negative LLR -> 1.
pub fn hard_decide(llrs: &[f64]) -> Vec<u8> {
    llrs.iter().map(|&l| if l < 0.0 { 1 } else { 0 }).collect()
}

// ---------------------------------------------------------------------------
// Helper: construct standard Hamming(7,4) H matrix
// ---------------------------------------------------------------------------

/// Create a (7,4) Hamming code parity-check matrix.
///
/// This is useful for testing and examples.
pub fn hamming_7_4_parity_check() -> ParityCheckMatrix {
    #[rustfmt::skip]
    let h_data: &[u8] = &[
        1, 0, 0, 1, 1, 0, 1,
        0, 1, 0, 1, 0, 1, 1,
        0, 0, 1, 0, 1, 1, 1,
    ];
    ParityCheckMatrix::from_dense(3, 7, h_data)
}

/// Encode a 4-bit message into a 7-bit Hamming(7,4) codeword using the
/// systematic generator matrix.
///
/// With the parity-check matrix H having an identity in columns 0-2,
/// the encoding places parity in positions 0-2 and data in positions 3-6.
pub fn hamming_7_4_encode(msg: &[u8; 4]) -> [u8; 7] {
    // H columns 0,1,2 form I_3.  Columns 3-6 are the P sub-matrix.
    // Systematic: data d[0..4] in positions 3..6, parity in 0..2.
    // From H*c = 0 (mod 2):
    //   row 0: c0 ^ c3 ^ c4 ^ c6 = 0  =>  c0 = d0 ^ d1 ^ d3
    //   row 1: c1 ^ c3 ^ c5 ^ c6 = 0  =>  c1 = d0 ^ d2 ^ d3
    //   row 2: c2 ^ c4 ^ c5 ^ c6 = 0  =>  c2 = d1 ^ d2 ^ d3
    // where d0=msg[0]=c3, d1=msg[1]=c4, d2=msg[2]=c5, d3=msg[3]=c6
    let d = msg;
    let p0 = d[0] ^ d[1] ^ d[3];
    let p1 = d[0] ^ d[2] ^ d[3];
    let p2 = d[1] ^ d[2] ^ d[3];
    [p0, p1, p2, d[0], d[1], d[2], d[3]]
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Helper: build (7,4) Hamming decoder with given algorithm.
    fn hamming_decoder(algo: BpAlgorithm) -> BeliefPropagationDecoder {
        let h = hamming_7_4_parity_check();
        BeliefPropagationDecoder::new(h, algo, 50)
    }

    // 1. Basic construction
    #[test]
    fn test_parity_check_from_dense() {
        let h = hamming_7_4_parity_check();
        assert_eq!(h.num_checks, 3);
        assert_eq!(h.num_vars, 7);
        assert_eq!(h.num_edges(), 12); // 4 ones per row x 3 rows
    }

    // 2. Sparse construction
    #[test]
    fn test_parity_check_from_sparse() {
        let entries = vec![
            (0, 0),
            (0, 3),
            (0, 4),
            (0, 6),
            (1, 1),
            (1, 3),
            (1, 5),
            (1, 6),
            (2, 2),
            (2, 4),
            (2, 5),
            (2, 6),
        ];
        let h = ParityCheckMatrix::from_sparse(3, 7, &entries);
        assert_eq!(h.num_checks, 3);
        assert_eq!(h.num_vars, 7);
        assert_eq!(h.num_edges(), 12);
        assert_eq!(h.check_neighbors(0), &[0, 3, 4, 6]);
    }

    // 3. Adjacency consistency
    #[test]
    fn test_adjacency_consistency() {
        let h = hamming_7_4_parity_check();
        for c in 0..h.num_checks {
            for &v in h.check_neighbors(c) {
                assert!(
                    h.var_neighbors(v).contains(&c),
                    "var {v} should list check {c}"
                );
            }
        }
    }

    // 4. Hamming encode produces valid codeword
    #[test]
    fn test_hamming_encode_valid() {
        let h = hamming_7_4_parity_check();
        // Test all 16 possible 4-bit messages.
        for bits in 0u8..16 {
            let msg = [
                (bits >> 3) & 1,
                (bits >> 2) & 1,
                (bits >> 1) & 1,
                bits & 1,
            ];
            let codeword = hamming_7_4_encode(&msg);
            // Check syndrome is zero.
            let decoder = BeliefPropagationDecoder::new(h.clone(), BpAlgorithm::SumProduct, 1);
            assert!(
                decoder.syndrome_check(&codeword),
                "codeword for msg {:?} failed syndrome check: {:?}",
                msg,
                codeword
            );
        }
    }

    // 5. Sum-product decodes all-zero codeword
    #[test]
    fn test_sp_decode_all_zeros() {
        let mut dec = hamming_decoder(BpAlgorithm::SumProduct);
        // Strong positive LLRs -> all zeros.
        let llrs = vec![5.0; 7];
        let result = dec.decode(&llrs);
        assert!(result.converged);
        assert_eq!(result.hard_decision, vec![0; 7]);
    }

    // 6. Sum-product decodes a non-trivial codeword
    #[test]
    fn test_sp_decode_nontrivial() {
        let mut dec = hamming_decoder(BpAlgorithm::SumProduct);
        // Encode [1,0,1,0]
        let cw = hamming_7_4_encode(&[1, 0, 1, 0]);
        // Convert to LLRs: bit 0 -> +5, bit 1 -> -5.
        let llrs: Vec<f64> = cw.iter().map(|&b| if b == 0 { 5.0 } else { -5.0 }).collect();
        let result = dec.decode(&llrs);
        assert!(result.converged);
        assert_eq!(result.hard_decision, cw.to_vec());
    }

    // 7. Sum-product corrects a single error
    #[test]
    fn test_sp_single_error_correction() {
        let mut dec = hamming_decoder(BpAlgorithm::SumProduct);
        let cw = hamming_7_4_encode(&[1, 1, 0, 1]);
        // Introduce an error at position 2 by flipping the LLR sign.
        let mut llrs: Vec<f64> = cw.iter().map(|&b| if b == 0 { 4.0 } else { -4.0 }).collect();
        // Flip bit 2: make its LLR weak and wrong.
        llrs[2] = if cw[2] == 0 { -1.0 } else { 1.0 };

        let result = dec.decode(&llrs);
        assert!(result.converged, "should converge for single-error correction");
        assert_eq!(result.hard_decision, cw.to_vec());
    }

    // 8. Min-sum decodes all-zero codeword
    #[test]
    fn test_ms_decode_all_zeros() {
        let mut dec = hamming_decoder(BpAlgorithm::MinSum);
        let llrs = vec![5.0; 7];
        let result = dec.decode(&llrs);
        assert!(result.converged);
        assert_eq!(result.hard_decision, vec![0; 7]);
    }

    // 9. Min-sum decodes a nontrivial codeword
    #[test]
    fn test_ms_decode_nontrivial() {
        let mut dec = hamming_decoder(BpAlgorithm::MinSum);
        let cw = hamming_7_4_encode(&[0, 1, 1, 0]);
        let llrs: Vec<f64> = cw.iter().map(|&b| if b == 0 { 5.0 } else { -5.0 }).collect();
        let result = dec.decode(&llrs);
        assert!(result.converged);
        assert_eq!(result.hard_decision, cw.to_vec());
    }

    // 10. Min-sum corrects a single error
    #[test]
    fn test_ms_single_error_correction() {
        let mut dec = hamming_decoder(BpAlgorithm::MinSum);
        let cw = hamming_7_4_encode(&[0, 0, 1, 1]);
        let mut llrs: Vec<f64> = cw.iter().map(|&b| if b == 0 { 4.0 } else { -4.0 }).collect();
        llrs[5] = if cw[5] == 0 { -1.5 } else { 1.5 };
        let result = dec.decode(&llrs);
        assert!(result.converged);
        assert_eq!(result.hard_decision, cw.to_vec());
    }

    // 11. Scaled min-sum
    #[test]
    fn test_scaled_min_sum() {
        let algo = BpAlgorithm::ScaledMinSum(ScaledMinSumParams { alpha: 0.75 });
        let h = hamming_7_4_parity_check();
        let mut dec = BeliefPropagationDecoder::new(h, algo, 50);
        let cw = hamming_7_4_encode(&[1, 1, 1, 1]);
        let llrs: Vec<f64> = cw.iter().map(|&b| if b == 0 { 5.0 } else { -5.0 }).collect();
        let result = dec.decode(&llrs);
        assert!(result.converged);
        assert_eq!(result.hard_decision, cw.to_vec());
    }

    // 12. Syndrome check
    #[test]
    fn test_syndrome_check_valid() {
        let dec = hamming_decoder(BpAlgorithm::SumProduct);
        let cw = hamming_7_4_encode(&[1, 0, 0, 1]);
        assert!(dec.syndrome_check(&cw));
    }

    // 13. Syndrome check detects errors
    #[test]
    fn test_syndrome_check_invalid() {
        let dec = hamming_decoder(BpAlgorithm::SumProduct);
        let mut cw = hamming_7_4_encode(&[1, 0, 0, 1]);
        cw[0] ^= 1; // flip one bit
        assert!(!dec.syndrome_check(&cw));
    }

    // 14. Syndrome vector
    #[test]
    fn test_syndrome_vector() {
        let dec = hamming_decoder(BpAlgorithm::SumProduct);
        let cw = hamming_7_4_encode(&[0, 1, 0, 1]);
        let s = dec.syndrome(&cw);
        assert_eq!(s, vec![0, 0, 0]);
    }

    // 15. Early termination -- converges in fewer than max_iters
    #[test]
    fn test_early_termination() {
        let mut dec = hamming_decoder(BpAlgorithm::SumProduct);
        dec.set_max_iters(100);
        let llrs = vec![10.0; 7]; // very strong all-zeros
        let result = dec.decode(&llrs);
        assert!(result.converged);
        assert!(result.iterations < 100, "should converge early, used {} iters", result.iterations);
    }

    // 16. BER estimation
    #[test]
    fn test_ber_estimation() {
        let tx = vec![0, 0, 1, 1, 0, 1, 0];
        let llrs = vec![3.0, 2.0, -4.0, -3.0, 1.0, -2.0, 5.0];
        let ber = estimate_ber(&tx, &llrs);
        assert!((ber - 0.0).abs() < 1e-10, "no errors expected, got BER={ber}");

        // Introduce one error: flip llrs[2] so it decodes as 0 instead of 1.
        let llrs_err = vec![3.0, 2.0, 4.0, -3.0, 1.0, -2.0, 5.0];
        let ber_err = estimate_ber(&tx, &llrs_err);
        assert!(
            (ber_err - 1.0 / 7.0).abs() < 1e-10,
            "expected 1/7 BER, got {ber_err}"
        );
    }

    // 17. Hard decision helper
    #[test]
    fn test_hard_decide() {
        let llrs = vec![1.0, -2.0, 0.5, -0.1, 0.0];
        let bits = hard_decide(&llrs);
        assert_eq!(bits, vec![0, 1, 0, 1, 0]);
    }

    // 18. Decode with very noisy channel -- may not converge
    #[test]
    fn test_noisy_non_convergence() {
        let mut dec = hamming_decoder(BpAlgorithm::SumProduct);
        dec.set_max_iters(5);
        // Near-zero LLRs: the decoder has almost no information.
        let llrs = vec![0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01];
        let result = dec.decode(&llrs);
        // It may or may not converge, but it should run without panicking
        // and use all iterations if it doesn't converge.
        assert!(result.iterations <= 5);
        assert_eq!(result.hard_decision.len(), 7);
    }

    // 19. Decoder reuse -- decode multiple words
    #[test]
    fn test_decoder_reuse() {
        let mut dec = hamming_decoder(BpAlgorithm::SumProduct);
        for bits in 0u8..16 {
            let msg = [
                (bits >> 3) & 1,
                (bits >> 2) & 1,
                (bits >> 1) & 1,
                bits & 1,
            ];
            let cw = hamming_7_4_encode(&msg);
            let llrs: Vec<f64> = cw.iter().map(|&b| if b == 0 { 5.0 } else { -5.0 }).collect();
            let result = dec.decode(&llrs);
            assert!(result.converged, "failed for msg {:?}", msg);
            assert_eq!(result.hard_decision, cw.to_vec(), "wrong decode for msg {:?}", msg);
        }
    }

    // 20. set_algorithm switches between sum-product and min-sum
    #[test]
    fn test_set_algorithm() {
        let mut dec = hamming_decoder(BpAlgorithm::SumProduct);
        let cw = hamming_7_4_encode(&[1, 0, 1, 1]);
        let llrs: Vec<f64> = cw.iter().map(|&b| if b == 0 { 5.0 } else { -5.0 }).collect();

        let r1 = dec.decode(&llrs);
        assert!(r1.converged);

        dec.set_algorithm(BpAlgorithm::MinSum);
        let r2 = dec.decode(&llrs);
        assert!(r2.converged);
        assert_eq!(r2.hard_decision, cw.to_vec());
    }

    // 21. Posterior LLRs have correct sign
    #[test]
    fn test_posterior_llr_signs() {
        let mut dec = hamming_decoder(BpAlgorithm::SumProduct);
        let cw = hamming_7_4_encode(&[0, 1, 0, 1]);
        let llrs: Vec<f64> = cw.iter().map(|&b| if b == 0 { 3.0 } else { -3.0 }).collect();
        let result = dec.decode(&llrs);
        assert!(result.converged);
        for (i, &b) in cw.iter().enumerate() {
            if b == 0 {
                assert!(
                    result.llrs[i] > 0.0,
                    "bit {i} is 0, expected positive LLR, got {}",
                    result.llrs[i]
                );
            } else {
                assert!(
                    result.llrs[i] < 0.0,
                    "bit {i} is 1, expected negative LLR, got {}",
                    result.llrs[i]
                );
            }
        }
    }

    // 22. Empty BER estimation
    #[test]
    fn test_ber_empty() {
        let ber = estimate_ber(&[], &[]);
        assert!((ber - 0.0).abs() < 1e-10);
    }
}
