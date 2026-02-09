//! Polar Code — Arikan's capacity-achieving channel codes for 5G NR
//!
//! Implements polar encoding and successive cancellation (SC) decoding.
//! Polar codes are used in 5G NR for control channels (PDCCH, PUCCH) per
//! 3GPP TS 38.212. They achieve channel capacity with low-complexity decoding.
//!
//! ## Algorithm
//!
//! 1. Select K most reliable bit channels (frozen bits = 0 for unreliable channels)
//! 2. Encode: u × G_N where G_N = B_N × F^⊗n, F = [1 0; 1 1]
//! 3. Decode: Successive Cancellation — traverse binary tree, compute LLRs
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::polar_code::{PolarEncoder, PolarDecoder};
//!
//! let n = 8; // Code length
//! let k = 4; // Info bits
//! let encoder = PolarEncoder::new(n, k);
//! let info_bits = vec![true, false, true, true];
//! let codeword = encoder.encode(&info_bits);
//! assert_eq!(codeword.len(), 8);
//!
//! let decoder = PolarDecoder::new(n, k);
//! // With soft LLR inputs, decode back
//! let llrs: Vec<f64> = codeword.iter().map(|&b| if b { -5.0 } else { 5.0 }).collect();
//! let decoded = decoder.decode(&llrs);
//! assert_eq!(decoded, info_bits);
//! ```

/// Polar encoder.
#[derive(Debug, Clone)]
pub struct PolarEncoder {
    /// Code length N (must be power of 2).
    n: usize,
    /// Number of information bits K.
    k: usize,
    /// log2(N).
    log_n: usize,
    /// Information bit positions (sorted by reliability).
    info_positions: Vec<usize>,
    /// Frozen bit positions.
    frozen_positions: Vec<usize>,
}

/// Polar decoder (Successive Cancellation).
#[derive(Debug, Clone)]
pub struct PolarDecoder {
    /// Code length N.
    n: usize,
    /// Number of information bits K.
    k: usize,
    /// log2(N).
    log_n: usize,
    /// Information bit positions.
    info_positions: Vec<usize>,
    /// Frozen bit mask (true = frozen).
    frozen_mask: Vec<bool>,
}

impl PolarEncoder {
    /// Create a new polar encoder.
    ///
    /// # Arguments
    /// * `n` - Code length (must be power of 2, >= 2)
    /// * `k` - Number of information bits (k <= n)
    pub fn new(n: usize, k: usize) -> Self {
        assert!(n.is_power_of_two() && n >= 2, "N must be power of 2 >= 2");
        assert!(k <= n, "K must be <= N");
        let log_n = n.trailing_zeros() as usize;

        // Compute channel reliabilities using Bhattacharyya bounds
        let reliabilities = compute_bhattacharyya_bounds(n, 0.0);

        // Select K most reliable positions for info bits
        let mut sorted_indices: Vec<usize> = (0..n).collect();
        sorted_indices.sort_by(|&a, &b| {
            reliabilities[a].partial_cmp(&reliabilities[b]).unwrap()
        });

        let info_positions: Vec<usize> = {
            let mut pos: Vec<usize> = sorted_indices[..k].to_vec();
            pos.sort();
            pos
        };

        let frozen_positions: Vec<usize> = {
            let mut pos: Vec<usize> = sorted_indices[k..].to_vec();
            pos.sort();
            pos
        };

        Self {
            n,
            k,
            log_n,
            info_positions,
            frozen_positions,
        }
    }

    /// Create with explicit information bit positions.
    pub fn with_info_positions(n: usize, info_positions: Vec<usize>) -> Self {
        assert!(n.is_power_of_two() && n >= 2);
        let k = info_positions.len();
        let log_n = n.trailing_zeros() as usize;

        let mut frozen_mask = vec![true; n];
        for &pos in &info_positions {
            frozen_mask[pos] = false;
        }
        let frozen_positions: Vec<usize> = (0..n).filter(|&i| frozen_mask[i]).collect();

        Self {
            n,
            k,
            log_n,
            info_positions,
            frozen_positions,
        }
    }

    /// Encode information bits into a polar codeword.
    pub fn encode(&self, info_bits: &[bool]) -> Vec<bool> {
        assert_eq!(info_bits.len(), self.k, "Expected {} info bits", self.k);

        // Place info bits and frozen bits
        let mut u = vec![false; self.n];
        for (idx, &pos) in self.info_positions.iter().enumerate() {
            u[pos] = info_bits[idx];
        }
        // Frozen positions stay false (0)

        // Apply polar transform: u × G_N
        // G_N = F^⊗n where F = [1 0; 1 1]
        // Butterfly structure
        polar_transform(&mut u);

        u
    }

    /// Get information bit positions.
    pub fn info_positions(&self) -> &[usize] {
        &self.info_positions
    }

    /// Get frozen bit positions.
    pub fn frozen_positions(&self) -> &[usize] {
        &self.frozen_positions
    }

    /// Code rate K/N.
    pub fn rate(&self) -> f64 {
        self.k as f64 / self.n as f64
    }
}

impl PolarDecoder {
    /// Create a new SC decoder.
    pub fn new(n: usize, k: usize) -> Self {
        assert!(n.is_power_of_two() && n >= 2);
        assert!(k <= n);
        let log_n = n.trailing_zeros() as usize;

        let reliabilities = compute_bhattacharyya_bounds(n, 0.0);
        let mut sorted_indices: Vec<usize> = (0..n).collect();
        sorted_indices.sort_by(|&a, &b| {
            reliabilities[a].partial_cmp(&reliabilities[b]).unwrap()
        });

        let info_positions: Vec<usize> = {
            let mut pos: Vec<usize> = sorted_indices[..k].to_vec();
            pos.sort();
            pos
        };

        let mut frozen_mask = vec![true; n];
        for &pos in &info_positions {
            frozen_mask[pos] = false;
        }

        Self {
            n,
            k,
            log_n,
            info_positions,
            frozen_mask,
        }
    }

    /// Create with explicit info positions.
    pub fn with_info_positions(n: usize, info_positions: Vec<usize>) -> Self {
        let k = info_positions.len();
        let log_n = n.trailing_zeros() as usize;

        let mut frozen_mask = vec![true; n];
        for &pos in &info_positions {
            frozen_mask[pos] = false;
        }

        Self {
            n,
            k,
            log_n,
            info_positions,
            frozen_mask,
        }
    }

    /// Decode using Successive Cancellation.
    ///
    /// Input: LLR values (positive = more likely 0, negative = more likely 1).
    pub fn decode(&self, llrs: &[f64]) -> Vec<bool> {
        assert_eq!(llrs.len(), self.n, "Expected {} LLRs", self.n);

        let u = sc_decode_recursive(llrs, &self.frozen_mask);

        // Extract info bits
        self.info_positions.iter().map(|&pos| u[pos]).collect()
    }
}

/// f function for SC decoding (min-sum approximation of boxplus).
fn f_function(l1: f64, l2: f64) -> f64 {
    // sign(l1) * sign(l2) * min(|l1|, |l2|)
    let sign = if (l1 < 0.0) != (l2 < 0.0) { -1.0 } else { 1.0 };
    sign * l1.abs().min(l2.abs())
}

/// Recursive SC decoder: decode u-vector from channel LLRs.
fn sc_decode_recursive(channel_llrs: &[f64], frozen_mask: &[bool]) -> Vec<bool> {
    let n = channel_llrs.len();

    if n == 1 {
        return if frozen_mask[0] {
            vec![false]
        } else {
            vec![channel_llrs[0] < 0.0]
        };
    }

    let half = n / 2;

    // f-function LLRs for left child
    let f_llrs: Vec<f64> = (0..half)
        .map(|i| f_function(channel_llrs[i], channel_llrs[i + half]))
        .collect();

    // Decode left child (recovers u[0..N/2])
    let u_left = sc_decode_recursive(&f_llrs, &frozen_mask[..half]);

    // Encode left decisions for g-function partial sums: v_L = T(u_left)
    let mut v_left = u_left.clone();
    polar_transform(&mut v_left);

    // g-function LLRs for right child (using encoded partial sums)
    let g_llrs: Vec<f64> = (0..half)
        .map(|i| {
            let sign = if v_left[i] { -1.0 } else { 1.0 };
            channel_llrs[i + half] + sign * channel_llrs[i]
        })
        .collect();

    // Decode right child (recovers u[N/2..N])
    let u_right = sc_decode_recursive(&g_llrs, &frozen_mask[half..]);

    // Return concatenated u = [u_left, u_right]
    let mut u = u_left;
    u.extend_from_slice(&u_right);

    u
}

/// Apply the polar transform (Arikan's butterfly) in-place.
fn polar_transform(bits: &mut [bool]) {
    let n = bits.len();
    if n <= 1 {
        return;
    }

    let mut step = 1;
    while step < n {
        for i in (0..n).step_by(2 * step) {
            for j in 0..step {
                bits[i + j] ^= bits[i + j + step];
            }
        }
        step *= 2;
    }
}

/// Compute Bhattacharyya bounds for channel reliability ordering.
///
/// Returns reliability scores (lower = more reliable = better for info bits).
fn compute_bhattacharyya_bounds(n: usize, _design_snr_db: f64) -> Vec<f64> {
    // Use bit-reversal-based reliability ordering (simplified)
    // For AWGN, this approximates the Bhattacharyya parameter ordering
    let log_n = n.trailing_zeros() as usize;

    let mut reliabilities = vec![0.0; n];
    for i in 0..n {
        // Compute reliability using bit-reversal weight
        let mut weight = 0.0;
        for bit in 0..log_n {
            if (i >> bit) & 1 == 1 {
                weight += 1.0 / (1 << bit) as f64;
            }
        }
        // Lower weight = more reliable channel
        reliabilities[i] = -(weight + (i as f64) * 1e-10); // tiebreaker
    }

    reliabilities
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_decode_roundtrip() {
        let encoder = PolarEncoder::new(8, 4);
        let decoder = PolarDecoder::new(8, 4);

        let info = vec![true, false, true, true];
        let codeword = encoder.encode(&info);
        assert_eq!(codeword.len(), 8);

        // Perfect channel (large LLRs)
        let llrs: Vec<f64> = codeword.iter().map(|&b| if b { -10.0 } else { 10.0 }).collect();
        let decoded = decoder.decode(&llrs);
        assert_eq!(decoded, info, "Roundtrip failed: {:?} vs {:?}", decoded, info);
    }

    #[test]
    fn test_all_zeros() {
        let encoder = PolarEncoder::new(8, 4);
        let decoder = PolarDecoder::new(8, 4);

        let info = vec![false, false, false, false];
        let codeword = encoder.encode(&info);
        let llrs: Vec<f64> = codeword.iter().map(|&b| if b { -10.0 } else { 10.0 }).collect();
        let decoded = decoder.decode(&llrs);
        assert_eq!(decoded, info);
    }

    #[test]
    fn test_all_ones() {
        let encoder = PolarEncoder::new(8, 4);
        let decoder = PolarDecoder::new(8, 4);

        let info = vec![true, true, true, true];
        let codeword = encoder.encode(&info);
        let llrs: Vec<f64> = codeword.iter().map(|&b| if b { -10.0 } else { 10.0 }).collect();
        let decoded = decoder.decode(&llrs);
        assert_eq!(decoded, info);
    }

    #[test]
    fn test_code_rate() {
        let encoder = PolarEncoder::new(16, 8);
        assert!((encoder.rate() - 0.5).abs() < 1e-10);

        let encoder2 = PolarEncoder::new(32, 16);
        assert!((encoder2.rate() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_frozen_info_partition() {
        let encoder = PolarEncoder::new(8, 3);
        assert_eq!(encoder.info_positions().len(), 3);
        assert_eq!(encoder.frozen_positions().len(), 5);

        // All positions covered
        let mut all_pos: Vec<usize> = encoder.info_positions().to_vec();
        all_pos.extend_from_slice(encoder.frozen_positions());
        all_pos.sort();
        assert_eq!(all_pos, (0..8).collect::<Vec<_>>());
    }

    #[test]
    fn test_polar_transform() {
        // F^⊗1 = [1 0; 1 1]
        // [1, 0] → [1^0, 0] = [1, 0]
        let mut bits = vec![true, false];
        polar_transform(&mut bits);
        assert_eq!(bits, vec![true, false]);

        // [1, 1] → [1^1, 1] = [0, 1]
        let mut bits = vec![true, true];
        polar_transform(&mut bits);
        assert_eq!(bits, vec![false, true]);
    }

    #[test]
    fn test_f_function() {
        // f(+5, +3) = +3 (same sign, min magnitude)
        assert!((f_function(5.0, 3.0) - 3.0).abs() < 1e-10);

        // f(-5, +3) = -3 (opposite sign)
        assert!((f_function(-5.0, 3.0) - (-3.0)).abs() < 1e-10);

        // f(0, 5) = 0
        assert!((f_function(0.0, 5.0)).abs() < 1e-10);
    }

    #[test]
    fn test_explicit_info_positions() {
        let positions = vec![3, 5, 6, 7];
        let encoder = PolarEncoder::with_info_positions(8, positions.clone());
        assert_eq!(encoder.info_positions(), &positions);
        assert_eq!(encoder.k, 4);
    }

    #[test]
    fn test_larger_code() {
        let encoder = PolarEncoder::new(32, 16);
        let decoder = PolarDecoder::new(32, 16);

        let info: Vec<bool> = (0..16).map(|i| i % 3 == 0).collect();
        let codeword = encoder.encode(&info);
        assert_eq!(codeword.len(), 32);

        let llrs: Vec<f64> = codeword.iter().map(|&b| if b { -10.0 } else { 10.0 }).collect();
        let decoded = decoder.decode(&llrs);
        assert_eq!(decoded, info);
    }

    #[test]
    fn test_bhattacharyya_bounds() {
        let reliabilities = compute_bhattacharyya_bounds(8, 0.0);
        assert_eq!(reliabilities.len(), 8);
        // Should produce distinct values for sorting
        let mut sorted = reliabilities.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        for i in 1..8 {
            assert!(sorted[i] > sorted[i - 1], "Reliabilities should be distinct");
        }
    }
}
