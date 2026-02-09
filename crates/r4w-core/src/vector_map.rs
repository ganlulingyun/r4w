//! Vector Map — Remap vector elements by index mapping
//!
//! Reorders, duplicates, or selects elements from input vectors
//! according to a configurable index mapping table. Supports
//! multi-input to single-output mapping for combining streams.
//! GNU Radio equivalent: `vector_map`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::vector_map::{VectorMap, remap};
//!
//! // Reverse a 4-element vector
//! let map = VectorMap::new(4, vec![3, 2, 1, 0]);
//! let input = vec![10.0, 20.0, 30.0, 40.0];
//! let output = map.apply(&input);
//! assert_eq!(output, vec![40.0, 30.0, 20.0, 10.0]);
//! ```

/// A vector element remapper using an index mapping table.
#[derive(Debug, Clone)]
pub struct VectorMap {
    /// Expected input vector length.
    input_len: usize,
    /// Index mapping: output[i] = input[mapping[i]].
    mapping: Vec<usize>,
}

impl VectorMap {
    /// Create a vector map with specified input length and index mapping.
    ///
    /// `mapping[i]` specifies which input index maps to output position `i`.
    /// All indices in `mapping` must be < `input_len`.
    pub fn new(input_len: usize, mapping: Vec<usize>) -> Self {
        Self { input_len, mapping }
    }

    /// Create an identity map (no reordering).
    pub fn identity(len: usize) -> Self {
        Self {
            input_len: len,
            mapping: (0..len).collect(),
        }
    }

    /// Create a reverse map.
    pub fn reverse(len: usize) -> Self {
        Self {
            input_len: len,
            mapping: (0..len).rev().collect(),
        }
    }

    /// Create a map that selects specific indices from the input.
    pub fn select(input_len: usize, indices: &[usize]) -> Self {
        Self {
            input_len,
            mapping: indices.to_vec(),
        }
    }

    /// Create a map that repeats each element `n` times.
    pub fn repeat_each(input_len: usize, n: usize) -> Self {
        let mut mapping = Vec::with_capacity(input_len * n);
        for i in 0..input_len {
            for _ in 0..n {
                mapping.push(i);
            }
        }
        Self { input_len, mapping }
    }

    /// Create a map that interleaves elements from two halves.
    ///
    /// Input: [a0, a1, ..., b0, b1, ...] → [a0, b0, a1, b1, ...]
    pub fn interleave_halves(half_len: usize) -> Self {
        let input_len = half_len * 2;
        let mut mapping = Vec::with_capacity(input_len);
        for i in 0..half_len {
            mapping.push(i);
            mapping.push(i + half_len);
        }
        Self { input_len, mapping }
    }

    /// Apply the mapping to an input vector.
    pub fn apply<T: Clone>(&self, input: &[T]) -> Vec<T> {
        self.mapping
            .iter()
            .filter_map(|&idx| input.get(idx).cloned())
            .collect()
    }

    /// Apply the mapping to a block of vectors (each `input_len` elements).
    pub fn process<T: Clone>(&self, input: &[T]) -> Vec<T> {
        let mut output = Vec::with_capacity(
            (input.len() / self.input_len) * self.mapping.len(),
        );
        for chunk in input.chunks_exact(self.input_len) {
            output.extend(self.apply(chunk));
        }
        output
    }

    /// Get the output vector length.
    pub fn output_len(&self) -> usize {
        self.mapping.len()
    }

    /// Get the expected input vector length.
    pub fn input_len(&self) -> usize {
        self.input_len
    }

    /// Get the mapping table.
    pub fn mapping(&self) -> &[usize] {
        &self.mapping
    }

    /// Check if the mapping is valid (all indices < input_len).
    pub fn is_valid(&self) -> bool {
        self.mapping.iter().all(|&idx| idx < self.input_len)
    }
}

/// One-shot remap: reorder elements of `input` according to `mapping`.
pub fn remap<T: Clone>(input: &[T], mapping: &[usize]) -> Vec<T> {
    mapping
        .iter()
        .filter_map(|&idx| input.get(idx).cloned())
        .collect()
}

/// Create a bit-reversal permutation table for FFT reordering.
pub fn bit_reversal_map(n: usize) -> Vec<usize> {
    if n == 0 {
        return Vec::new();
    }
    let bits = (n as f64).log2() as u32;
    (0..n)
        .map(|i| (i as u32).reverse_bits() >> (32 - bits))
        .map(|i| i as usize)
        .collect()
}

/// Create a circular shift mapping: shift all indices by `shift` positions.
pub fn circular_shift_map(len: usize, shift: isize) -> Vec<usize> {
    (0..len)
        .map(|i| {
            let shifted = (i as isize - shift).rem_euclid(len as isize);
            shifted as usize
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity() {
        let map = VectorMap::identity(4);
        let input = vec![10, 20, 30, 40];
        assert_eq!(map.apply(&input), input);
    }

    #[test]
    fn test_reverse() {
        let map = VectorMap::reverse(4);
        let input = vec![1, 2, 3, 4];
        assert_eq!(map.apply(&input), vec![4, 3, 2, 1]);
    }

    #[test]
    fn test_select() {
        let map = VectorMap::select(5, &[0, 2, 4]);
        let input = vec![10, 20, 30, 40, 50];
        assert_eq!(map.apply(&input), vec![10, 30, 50]);
    }

    #[test]
    fn test_repeat_each() {
        let map = VectorMap::repeat_each(3, 2);
        let input = vec![1, 2, 3];
        assert_eq!(map.apply(&input), vec![1, 1, 2, 2, 3, 3]);
    }

    #[test]
    fn test_interleave_halves() {
        let map = VectorMap::interleave_halves(3);
        let input = vec![1, 2, 3, 4, 5, 6];
        assert_eq!(map.apply(&input), vec![1, 4, 2, 5, 3, 6]);
    }

    #[test]
    fn test_process_multiple_vectors() {
        let map = VectorMap::reverse(3);
        let input = vec![1, 2, 3, 4, 5, 6];
        let output = map.process(&input);
        assert_eq!(output, vec![3, 2, 1, 6, 5, 4]);
    }

    #[test]
    fn test_duplicate_indices() {
        let map = VectorMap::new(3, vec![0, 0, 1, 1, 2, 2]);
        let input = vec![10, 20, 30];
        assert_eq!(map.apply(&input), vec![10, 10, 20, 20, 30, 30]);
    }

    #[test]
    fn test_remap_fn() {
        let input = vec!['a', 'b', 'c', 'd'];
        let result = remap(&input, &[3, 1, 0, 2]);
        assert_eq!(result, vec!['d', 'b', 'a', 'c']);
    }

    #[test]
    fn test_bit_reversal() {
        let map = bit_reversal_map(8);
        // For N=8: 0→0, 1→4, 2→2, 3→6, 4→1, 5→5, 6→3, 7→7
        assert_eq!(map[0], 0);
        assert_eq!(map[1], 4);
        assert_eq!(map[4], 1);
    }

    #[test]
    fn test_circular_shift() {
        let map = circular_shift_map(5, 2);
        let input = vec![1, 2, 3, 4, 5];
        let output = remap(&input, &map);
        // shift=2: output[i] = input[(i-2) mod 5]
        assert_eq!(output, vec![4, 5, 1, 2, 3]);
    }

    #[test]
    fn test_circular_shift_negative() {
        let map = circular_shift_map(5, -1);
        let input = vec![1, 2, 3, 4, 5];
        let output = remap(&input, &map);
        // shift=-1: output[i] = input[(i+1) mod 5]
        assert_eq!(output, vec![2, 3, 4, 5, 1]);
    }

    #[test]
    fn test_is_valid() {
        let valid = VectorMap::new(3, vec![0, 1, 2]);
        assert!(valid.is_valid());
        let invalid = VectorMap::new(3, vec![0, 1, 5]);
        assert!(!invalid.is_valid());
    }

    #[test]
    fn test_accessors() {
        let map = VectorMap::new(4, vec![3, 2, 1, 0]);
        assert_eq!(map.input_len(), 4);
        assert_eq!(map.output_len(), 4);
        assert_eq!(map.mapping(), &[3, 2, 1, 0]);
    }

    #[test]
    fn test_empty() {
        let map = VectorMap::identity(0);
        assert!(map.apply::<i32>(&[]).is_empty());
        assert!(bit_reversal_map(0).is_empty());
    }

    #[test]
    fn test_out_of_bounds_ignored() {
        let map = VectorMap::new(3, vec![0, 10, 2]);
        let input = vec![1, 2, 3];
        let output = map.apply(&input);
        // Index 10 is out of bounds, filtered out
        assert_eq!(output, vec![1, 3]);
    }
}
