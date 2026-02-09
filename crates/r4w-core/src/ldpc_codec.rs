//! LDPC Codec — Low-Density Parity-Check encoder/decoder
//!
//! Implements LDPC forward error correction using sparse parity-check
//! matrices and iterative belief propagation decoding. LDPC is the
//! dominant FEC in modern standards (5G NR, Wi-Fi, DVB-S2, CCSDS).
//! GNU Radio equivalent: `gr-fec/ldpc_encoder`, `ldpc_decoder`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ldpc_codec::{LdpcEncoder, LdpcDecoder, SparseParityCheck, DecodingAlgorithm};
//!
//! // Create a small (7,4) Hamming-like LDPC code
//! let h = SparseParityCheck::hamming_7_4();
//! let encoder = LdpcEncoder::new(h.clone());
//! let decoder = LdpcDecoder::new(h, DecodingAlgorithm::MinSum { scale: 0.8 }, 50);
//!
//! let info = vec![true, false, true, true];
//! let codeword = encoder.encode(&info);
//! let llrs: Vec<f64> = codeword.iter().map(|&b| if b { -5.0 } else { 5.0 }).collect();
//! let result = decoder.decode(&llrs);
//! assert!(result.converged);
//! ```

/// Sparse parity-check matrix in CSR-like format.
#[derive(Debug, Clone)]
pub struct SparseParityCheck {
    /// Number of rows (check nodes).
    pub num_checks: usize,
    /// Number of columns (variable nodes / codeword length).
    pub num_vars: usize,
    /// For each check node, the indices of connected variable nodes.
    pub check_to_var: Vec<Vec<usize>>,
    /// For each variable node, the indices of connected check nodes.
    pub var_to_check: Vec<Vec<usize>>,
}

impl SparseParityCheck {
    /// Create from a dense binary matrix (row-major, `rows x cols`).
    pub fn from_dense(matrix: &[Vec<u8>]) -> Self {
        let num_checks = matrix.len();
        let num_vars = if num_checks > 0 { matrix[0].len() } else { 0 };

        let mut check_to_var = vec![Vec::new(); num_checks];
        let mut var_to_check = vec![Vec::new(); num_vars];

        for (r, row) in matrix.iter().enumerate() {
            for (c, &val) in row.iter().enumerate() {
                if val != 0 {
                    check_to_var[r].push(c);
                    var_to_check[c].push(r);
                }
            }
        }

        Self {
            num_checks,
            num_vars,
            check_to_var,
            var_to_check,
        }
    }

    /// Create a (7,4) Hamming code parity-check matrix.
    pub fn hamming_7_4() -> Self {
        // H = [1 1 1 0 1 0 0]
        //     [1 1 0 1 0 1 0]
        //     [1 0 1 1 0 0 1]
        Self::from_dense(&[
            vec![1, 1, 1, 0, 1, 0, 0],
            vec![1, 1, 0, 1, 0, 1, 0],
            vec![1, 0, 1, 1, 0, 0, 1],
        ])
    }

    /// Create a regular (n, n-m) LDPC code from a proto-matrix.
    ///
    /// Each non-negative entry in proto is a circulant shift; -1 means zero block.
    /// Expansion factor `z` determines the final matrix size.
    pub fn from_proto_matrix(proto: &[Vec<i32>], z: usize) -> Self {
        let m_b = proto.len();
        let n_b = if m_b > 0 { proto[0].len() } else { 0 };
        let num_checks = m_b * z;
        let num_vars = n_b * z;

        let mut check_to_var = vec![Vec::new(); num_checks];
        let mut var_to_check = vec![Vec::new(); num_vars];

        for (br, row) in proto.iter().enumerate() {
            for (bc, &shift) in row.iter().enumerate() {
                if shift < 0 {
                    continue;
                }
                let s = shift as usize % z;
                for k in 0..z {
                    let r = br * z + k;
                    let c = bc * z + (k + s) % z;
                    check_to_var[r].push(c);
                    var_to_check[c].push(r);
                }
            }
        }

        Self {
            num_checks,
            num_vars,
            check_to_var,
            var_to_check,
        }
    }

    /// Code rate k/n.
    pub fn rate(&self) -> f64 {
        if self.num_vars == 0 {
            return 0.0;
        }
        (self.num_vars - self.num_checks) as f64 / self.num_vars as f64
    }

    /// Information word length.
    pub fn info_len(&self) -> usize {
        self.num_vars.saturating_sub(self.num_checks)
    }

    /// Check syndrome: H * codeword = 0 (mod 2).
    pub fn syndrome(&self, codeword: &[bool]) -> Vec<bool> {
        self.check_to_var
            .iter()
            .map(|vars| {
                let sum: usize = vars.iter().filter(|&&v| codeword.get(v).copied().unwrap_or(false)).count();
                sum % 2 != 0
            })
            .collect()
    }

    /// Check if codeword is valid (all-zero syndrome).
    pub fn is_valid(&self, codeword: &[bool]) -> bool {
        self.syndrome(codeword).iter().all(|&s| !s)
    }
}

/// Decoding algorithm selection.
#[derive(Debug, Clone, Copy)]
pub enum DecodingAlgorithm {
    /// Sum-Product (Belief Propagation) — optimal but slower.
    SumProduct,
    /// Min-Sum with scaling factor — faster approximation.
    MinSum { scale: f64 },
}

/// LDPC decoding result.
#[derive(Debug, Clone)]
pub struct LdpcResult {
    /// Decoded information bits.
    pub decoded: Vec<bool>,
    /// Full codeword (including parity).
    pub codeword: Vec<bool>,
    /// Whether the decoder converged (all-zero syndrome).
    pub converged: bool,
    /// Number of iterations used.
    pub iterations: usize,
}

/// LDPC encoder (systematic).
#[derive(Debug, Clone)]
pub struct LdpcEncoder {
    h: SparseParityCheck,
}

impl LdpcEncoder {
    /// Create encoder from parity-check matrix.
    pub fn new(h: SparseParityCheck) -> Self {
        Self { h }
    }

    /// Encode information bits into a systematic codeword.
    ///
    /// Uses iterative parity computation for sparse H.
    pub fn encode(&self, info_bits: &[bool]) -> Vec<bool> {
        let k = self.h.info_len();
        let n = self.h.num_vars;
        let m = self.h.num_checks;

        // Pad or truncate info bits
        let mut info = vec![false; k];
        let copy_len = info_bits.len().min(k);
        info[..copy_len].copy_from_slice(&info_bits[..copy_len]);

        // Systematic: [info | parity]
        let mut codeword = vec![false; n];
        codeword[..k].copy_from_slice(&info);

        // Compute parity bits to satisfy H * c = 0
        // For each check equation, solve for the parity bit
        for ci in 0..m {
            let parity_idx = k + ci;
            if parity_idx >= n {
                break;
            }
            let mut sum = false;
            for &v in &self.h.check_to_var[ci] {
                if v != parity_idx && v < n {
                    sum ^= codeword[v];
                }
            }
            codeword[parity_idx] = sum;
        }

        codeword
    }
}

/// LDPC decoder using iterative belief propagation.
#[derive(Debug, Clone)]
pub struct LdpcDecoder {
    h: SparseParityCheck,
    algorithm: DecodingAlgorithm,
    max_iterations: usize,
}

impl LdpcDecoder {
    /// Create decoder.
    pub fn new(h: SparseParityCheck, algorithm: DecodingAlgorithm, max_iterations: usize) -> Self {
        Self {
            h,
            algorithm,
            max_iterations: max_iterations.max(1),
        }
    }

    /// Decode from channel LLRs (positive = more likely 0, negative = more likely 1).
    pub fn decode(&self, llrs: &[f64]) -> LdpcResult {
        let n = self.h.num_vars;
        let m = self.h.num_checks;

        // Initialize variable-to-check messages with channel LLRs
        let mut v2c: Vec<Vec<f64>> = vec![vec![]; m];
        for (ci, vars) in self.h.check_to_var.iter().enumerate() {
            v2c[ci] = vars.iter().map(|&v| llrs.get(v).copied().unwrap_or(0.0)).collect();
        }

        let mut c2v: Vec<Vec<f64>> = vec![vec![]; m];
        for (ci, vars) in self.h.check_to_var.iter().enumerate() {
            c2v[ci] = vec![0.0; vars.len()];
        }

        let mut total_llr = vec![0.0f64; n];
        let mut iterations = 0;

        for iter in 0..self.max_iterations {
            iterations = iter + 1;

            // Check node update
            for ci in 0..m {
                let vars = &self.h.check_to_var[ci];
                let num_v = vars.len();
                for vi_idx in 0..num_v {
                    match self.algorithm {
                        DecodingAlgorithm::SumProduct => {
                            let mut product = 1.0f64;
                            for vj_idx in 0..num_v {
                                if vj_idx != vi_idx {
                                    let x = v2c[ci][vj_idx];
                                    product *= (x / 2.0).tanh();
                                }
                            }
                            c2v[ci][vi_idx] = 2.0 * product.clamp(-1.0 + 1e-15, 1.0 - 1e-15).atanh();
                        }
                        DecodingAlgorithm::MinSum { scale } => {
                            let mut min_abs = f64::MAX;
                            let mut sign = 1i32;
                            for vj_idx in 0..num_v {
                                if vj_idx != vi_idx {
                                    let x = v2c[ci][vj_idx];
                                    if x < 0.0 {
                                        sign = -sign;
                                    }
                                    let abs_x = x.abs();
                                    if abs_x < min_abs {
                                        min_abs = abs_x;
                                    }
                                }
                            }
                            c2v[ci][vi_idx] = sign as f64 * min_abs * scale;
                        }
                    }
                }
            }

            // Variable node update + total LLR
            total_llr.iter_mut().for_each(|x| *x = 0.0);
            for vi in 0..n {
                total_llr[vi] = llrs.get(vi).copied().unwrap_or(0.0);
                for &ci in &self.h.var_to_check[vi] {
                    let vars = &self.h.check_to_var[ci];
                    if let Some(idx) = vars.iter().position(|&v| v == vi) {
                        total_llr[vi] += c2v[ci][idx];
                    }
                }
            }

            // Update v2c messages (total LLR minus incoming c2v)
            for ci in 0..m {
                let vars = &self.h.check_to_var[ci];
                for (vi_idx, &vi) in vars.iter().enumerate() {
                    v2c[ci][vi_idx] = total_llr[vi] - c2v[ci][vi_idx];
                }
            }

            // Hard decision and syndrome check
            let hard: Vec<bool> = total_llr.iter().map(|&l| l < 0.0).collect();
            if self.h.is_valid(&hard) {
                let k = self.h.info_len();
                return LdpcResult {
                    decoded: hard[..k].to_vec(),
                    codeword: hard,
                    converged: true,
                    iterations,
                };
            }
        }

        let hard: Vec<bool> = total_llr.iter().map(|&l| l < 0.0).collect();
        let k = self.h.info_len();
        LdpcResult {
            decoded: hard[..k].to_vec(),
            codeword: hard,
            converged: false,
            iterations,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hamming_7_4_dimensions() {
        let h = SparseParityCheck::hamming_7_4();
        assert_eq!(h.num_checks, 3);
        assert_eq!(h.num_vars, 7);
        assert_eq!(h.info_len(), 4);
    }

    #[test]
    fn test_encode_decode_roundtrip() {
        let h = SparseParityCheck::hamming_7_4();
        let encoder = LdpcEncoder::new(h.clone());
        let decoder = LdpcDecoder::new(h.clone(), DecodingAlgorithm::MinSum { scale: 0.8 }, 50);

        let info = vec![true, false, true, true];
        let codeword = encoder.encode(&info);
        assert_eq!(codeword.len(), 7);

        // Valid codeword
        assert!(h.is_valid(&codeword));

        // Decode with clean LLRs
        let llrs: Vec<f64> = codeword.iter().map(|&b| if b { -5.0 } else { 5.0 }).collect();
        let result = decoder.decode(&llrs);
        assert!(result.converged);
        assert_eq!(result.decoded, info);
    }

    #[test]
    fn test_syndrome_valid_codeword() {
        let h = SparseParityCheck::hamming_7_4();
        let encoder = LdpcEncoder::new(h.clone());

        for bits in 0u8..16 {
            let info: Vec<bool> = (0..4).map(|i| (bits >> i) & 1 == 1).collect();
            let codeword = encoder.encode(&info);
            let syndrome = h.syndrome(&codeword);
            assert!(
                syndrome.iter().all(|&s| !s),
                "Syndrome non-zero for info={:?}",
                info
            );
        }
    }

    #[test]
    fn test_syndrome_invalid_codeword() {
        let h = SparseParityCheck::hamming_7_4();
        let encoder = LdpcEncoder::new(h.clone());

        let codeword = encoder.encode(&[true, false, true, false]);
        let mut corrupted = codeword.clone();
        corrupted[0] = !corrupted[0]; // Flip one bit
        assert!(!h.is_valid(&corrupted));
    }

    #[test]
    fn test_sum_product_decode() {
        let h = SparseParityCheck::hamming_7_4();
        let encoder = LdpcEncoder::new(h.clone());
        let decoder = LdpcDecoder::new(h, DecodingAlgorithm::SumProduct, 50);

        let info = vec![false, true, false, true];
        let codeword = encoder.encode(&info);
        let llrs: Vec<f64> = codeword.iter().map(|&b| if b { -4.0 } else { 4.0 }).collect();
        let result = decoder.decode(&llrs);
        assert!(result.converged);
        assert_eq!(result.decoded, info);
    }

    #[test]
    fn test_early_termination() {
        let h = SparseParityCheck::hamming_7_4();
        let encoder = LdpcEncoder::new(h.clone());
        let decoder = LdpcDecoder::new(h, DecodingAlgorithm::MinSum { scale: 0.8 }, 100);

        let codeword = encoder.encode(&[true, true, false, false]);
        let llrs: Vec<f64> = codeword.iter().map(|&b| if b { -10.0 } else { 10.0 }).collect();
        let result = decoder.decode(&llrs);
        assert!(result.converged);
        assert!(result.iterations < 100, "Should converge early, used {} iters", result.iterations);
    }

    #[test]
    fn test_max_iterations_respected() {
        let h = SparseParityCheck::hamming_7_4();
        let decoder = LdpcDecoder::new(h, DecodingAlgorithm::MinSum { scale: 0.8 }, 3);

        // Very weak LLRs
        let llrs = vec![0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01];
        let result = decoder.decode(&llrs);
        // Should not exceed max_iterations
        assert!(result.iterations <= 3, "Iterations {} should be <= 3", result.iterations);
    }

    #[test]
    fn test_proto_matrix_expansion() {
        // Simple 2x3 proto-matrix with z=4
        let proto = vec![
            vec![0, 1, -1],
            vec![-1, 0, 2],
        ];
        let h = SparseParityCheck::from_proto_matrix(&proto, 4);
        assert_eq!(h.num_checks, 8);  // 2 * 4
        assert_eq!(h.num_vars, 12);   // 3 * 4
    }

    #[test]
    fn test_code_rate() {
        let h = SparseParityCheck::hamming_7_4();
        let rate = h.rate();
        assert!((rate - 4.0 / 7.0).abs() < 1e-10);
    }

    #[test]
    fn test_decode_with_noise() {
        let h = SparseParityCheck::hamming_7_4();
        let encoder = LdpcEncoder::new(h.clone());
        let decoder = LdpcDecoder::new(h, DecodingAlgorithm::MinSum { scale: 0.8 }, 50);

        let info = vec![true, false, true, false];
        let codeword = encoder.encode(&info);

        // Add some noise to LLRs (weaken one bit)
        let mut llrs: Vec<f64> = codeword.iter().map(|&b| if b { -3.0 } else { 3.0 }).collect();
        llrs[2] *= -0.5; // Weaken/flip bit 2

        let result = decoder.decode(&llrs);
        // Should still recover
        assert_eq!(result.decoded, info);
    }

    #[test]
    fn test_min_sum_vs_sum_product_agree() {
        let h = SparseParityCheck::hamming_7_4();
        let encoder = LdpcEncoder::new(h.clone());

        let dec_sp = LdpcDecoder::new(h.clone(), DecodingAlgorithm::SumProduct, 50);
        let dec_ms = LdpcDecoder::new(h, DecodingAlgorithm::MinSum { scale: 0.8 }, 50);

        let info = vec![false, false, true, true];
        let codeword = encoder.encode(&info);
        let llrs: Vec<f64> = codeword.iter().map(|&b| if b { -5.0 } else { 5.0 }).collect();

        let r_sp = dec_sp.decode(&llrs);
        let r_ms = dec_ms.decode(&llrs);

        assert!(r_sp.converged);
        assert!(r_ms.converged);
        assert_eq!(r_sp.decoded, r_ms.decoded);
    }

    #[test]
    fn test_from_dense() {
        let h = SparseParityCheck::from_dense(&[
            vec![1, 0, 1],
            vec![0, 1, 1],
        ]);
        assert_eq!(h.num_checks, 2);
        assert_eq!(h.num_vars, 3);
        assert_eq!(h.check_to_var[0], vec![0, 2]);
        assert_eq!(h.check_to_var[1], vec![1, 2]);
    }
}
