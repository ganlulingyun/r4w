//! Argmax — Find index of maximum in vectors
//!
//! Operates on vectors of samples, returning the index of the maximum.
//! Essential for FFT-based demodulation (find peak bin), OFDM subcarrier
//! detection, and pattern matching. GNU Radio equivalent: `argmax`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::argmax::{argmax_f64, argmax_mag_sqrd, ArgmaxBlock};
//! use num_complex::Complex64;
//!
//! // Find peak in real vector
//! let data = vec![1.0, 3.0, 7.0, 2.0, 5.0];
//! let (idx, val) = argmax_f64(&data).unwrap();
//! assert_eq!(idx, 2);
//! assert_eq!(val, 7.0);
//!
//! // Find strongest bin in complex FFT output
//! let fft = vec![
//!     Complex64::new(1.0, 0.0),
//!     Complex64::new(0.0, 5.0), // |z|² = 25
//!     Complex64::new(2.0, 1.0), // |z|² = 5
//! ];
//! let (idx, mag2) = argmax_mag_sqrd(&fft).unwrap();
//! assert_eq!(idx, 1);
//! assert!((mag2 - 25.0).abs() < 1e-10);
//! ```

use num_complex::Complex64;

/// Find index and value of maximum in a real slice.
pub fn argmax_f64(data: &[f64]) -> Option<(usize, f64)> {
    if data.is_empty() {
        return None;
    }
    let mut best_idx = 0;
    let mut best_val = data[0];
    for (i, &v) in data.iter().enumerate().skip(1) {
        if v > best_val {
            best_val = v;
            best_idx = i;
        }
    }
    Some((best_idx, best_val))
}

/// Find index and value of minimum in a real slice.
pub fn argmin_f64(data: &[f64]) -> Option<(usize, f64)> {
    if data.is_empty() {
        return None;
    }
    let mut best_idx = 0;
    let mut best_val = data[0];
    for (i, &v) in data.iter().enumerate().skip(1) {
        if v < best_val {
            best_val = v;
            best_idx = i;
        }
    }
    Some((best_idx, best_val))
}

/// Find index and magnitude squared of the strongest complex sample.
pub fn argmax_mag_sqrd(data: &[Complex64]) -> Option<(usize, f64)> {
    if data.is_empty() {
        return None;
    }
    let mut best_idx = 0;
    let mut best_mag2 = data[0].norm_sqr();
    for (i, z) in data.iter().enumerate().skip(1) {
        let m2 = z.norm_sqr();
        if m2 > best_mag2 {
            best_mag2 = m2;
            best_idx = i;
        }
    }
    Some((best_idx, best_mag2))
}

/// Find index and magnitude squared of the weakest complex sample.
pub fn argmin_mag_sqrd(data: &[Complex64]) -> Option<(usize, f64)> {
    if data.is_empty() {
        return None;
    }
    let mut best_idx = 0;
    let mut best_mag2 = data[0].norm_sqr();
    for (i, z) in data.iter().enumerate().skip(1) {
        let m2 = z.norm_sqr();
        if m2 < best_mag2 {
            best_mag2 = m2;
            best_idx = i;
        }
    }
    Some((best_idx, best_mag2))
}

/// Argmax over vectors of fixed size (block-based).
///
/// Processes a stream of samples in windows of `vlen`, returning
/// the argmax index for each window.
#[derive(Debug, Clone)]
pub struct ArgmaxBlock {
    vlen: usize,
}

impl ArgmaxBlock {
    /// Create an argmax block that operates on vectors of `vlen` samples.
    pub fn new(vlen: usize) -> Self {
        Self { vlen: vlen.max(1) }
    }

    /// Process real samples, returning argmax index per vector.
    pub fn process(&self, input: &[f64]) -> Vec<usize> {
        input
            .chunks_exact(self.vlen)
            .map(|chunk| argmax_f64(chunk).map(|(i, _)| i).unwrap_or(0))
            .collect()
    }

    /// Process real samples, returning (index, value) per vector.
    pub fn process_with_value(&self, input: &[f64]) -> Vec<(usize, f64)> {
        input
            .chunks_exact(self.vlen)
            .map(|chunk| argmax_f64(chunk).unwrap_or((0, 0.0)))
            .collect()
    }

    /// Process complex samples by |z|², returning argmax index per vector.
    pub fn process_complex(&self, input: &[Complex64]) -> Vec<usize> {
        input
            .chunks_exact(self.vlen)
            .map(|chunk| argmax_mag_sqrd(chunk).map(|(i, _)| i).unwrap_or(0))
            .collect()
    }

    /// Process complex samples, returning (index, |z|²) per vector.
    pub fn process_complex_with_value(&self, input: &[Complex64]) -> Vec<(usize, f64)> {
        input
            .chunks_exact(self.vlen)
            .map(|chunk| argmax_mag_sqrd(chunk).unwrap_or((0, 0.0)))
            .collect()
    }

    /// Get vector length.
    pub fn vlen(&self) -> usize {
        self.vlen
    }
}

/// Find the top-K indices and values from a real slice.
pub fn top_k(data: &[f64], k: usize) -> Vec<(usize, f64)> {
    if data.is_empty() || k == 0 {
        return Vec::new();
    }
    let mut indexed: Vec<(usize, f64)> = data.iter().copied().enumerate().collect();
    indexed.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
    indexed.truncate(k);
    indexed
}

/// Find the top-K complex bins by magnitude squared.
pub fn top_k_complex(data: &[Complex64], k: usize) -> Vec<(usize, f64)> {
    if data.is_empty() || k == 0 {
        return Vec::new();
    }
    let mut indexed: Vec<(usize, f64)> = data
        .iter()
        .enumerate()
        .map(|(i, z)| (i, z.norm_sqr()))
        .collect();
    indexed.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
    indexed.truncate(k);
    indexed
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_argmax_basic() {
        let (idx, val) = argmax_f64(&[1.0, 5.0, 3.0, 2.0]).unwrap();
        assert_eq!(idx, 1);
        assert_eq!(val, 5.0);
    }

    #[test]
    fn test_argmax_first() {
        let (idx, _) = argmax_f64(&[10.0, 1.0, 2.0]).unwrap();
        assert_eq!(idx, 0);
    }

    #[test]
    fn test_argmax_last() {
        let (idx, _) = argmax_f64(&[1.0, 2.0, 10.0]).unwrap();
        assert_eq!(idx, 2);
    }

    #[test]
    fn test_argmax_empty() {
        assert!(argmax_f64(&[]).is_none());
    }

    #[test]
    fn test_argmin_basic() {
        let (idx, val) = argmin_f64(&[5.0, 1.0, 3.0, 2.0]).unwrap();
        assert_eq!(idx, 1);
        assert_eq!(val, 1.0);
    }

    #[test]
    fn test_argmax_complex() {
        let data = vec![
            Complex64::new(1.0, 0.0), // |z|² = 1
            Complex64::new(3.0, 4.0), // |z|² = 25
            Complex64::new(2.0, 1.0), // |z|² = 5
        ];
        let (idx, mag2) = argmax_mag_sqrd(&data).unwrap();
        assert_eq!(idx, 1);
        assert!((mag2 - 25.0).abs() < 1e-10);
    }

    #[test]
    fn test_argmin_complex() {
        let data = vec![
            Complex64::new(3.0, 4.0), // |z|² = 25
            Complex64::new(0.1, 0.0), // |z|² = 0.01
            Complex64::new(2.0, 1.0), // |z|² = 5
        ];
        let (idx, mag2) = argmin_mag_sqrd(&data).unwrap();
        assert_eq!(idx, 1);
        assert!((mag2 - 0.01).abs() < 1e-10);
    }

    #[test]
    fn test_block_real() {
        let block = ArgmaxBlock::new(4);
        let data = vec![1.0, 5.0, 3.0, 2.0, 0.0, 0.0, 9.0, 1.0];
        let indices = block.process(&data);
        assert_eq!(indices, vec![1, 2]); // max of [1,5,3,2]=idx1, max of [0,0,9,1]=idx2
    }

    #[test]
    fn test_block_with_value() {
        let block = ArgmaxBlock::new(3);
        let data = vec![1.0, 5.0, 3.0];
        let results = block.process_with_value(&data);
        assert_eq!(results, vec![(1, 5.0)]);
    }

    #[test]
    fn test_block_complex() {
        let block = ArgmaxBlock::new(2);
        let data = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 3.0),
            Complex64::new(5.0, 0.0),
            Complex64::new(2.0, 0.0),
        ];
        let indices = block.process_complex(&data);
        assert_eq!(indices, vec![1, 0]); // |3j|²=9 > |1|²=1, |5|²=25 > |2|²=4
    }

    #[test]
    fn test_block_remainder_ignored() {
        let block = ArgmaxBlock::new(3);
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0]; // 2 leftover
        let indices = block.process(&data);
        assert_eq!(indices.len(), 1); // Only one full vector
        assert_eq!(indices[0], 2);
    }

    #[test]
    fn test_top_k() {
        let data = vec![3.0, 1.0, 5.0, 4.0, 2.0];
        let top3 = top_k(&data, 3);
        assert_eq!(top3.len(), 3);
        assert_eq!(top3[0], (2, 5.0)); // index 2, value 5
        assert_eq!(top3[1], (3, 4.0)); // index 3, value 4
        assert_eq!(top3[2], (0, 3.0)); // index 0, value 3
    }

    #[test]
    fn test_top_k_complex() {
        let data = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(3.0, 4.0),
            Complex64::new(0.0, 2.0),
        ];
        let top2 = top_k_complex(&data, 2);
        assert_eq!(top2.len(), 2);
        assert_eq!(top2[0].0, 1); // index 1, |3+4j|²=25
        assert_eq!(top2[1].0, 2); // index 2, |2j|²=4
    }

    #[test]
    fn test_top_k_empty() {
        assert!(top_k(&[], 5).is_empty());
        assert!(top_k(&[1.0, 2.0], 0).is_empty());
    }

    #[test]
    fn test_single_element() {
        let (idx, val) = argmax_f64(&[42.0]).unwrap();
        assert_eq!(idx, 0);
        assert_eq!(val, 42.0);
    }
}
