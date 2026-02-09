//! Permute — Vector Index Reordering
//!
//! Reorder samples in a vector according to a permutation index table.
//! Used for bit/symbol interleaving, OFDM subcarrier mapping, and
//! general stream manipulation.
//! GNU Radio equivalent: custom permutation implementations in
//! `gr::blocks::vector_source`, `gr::digital::map_bb`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::permute::{Permute, inverse_permutation};
//!
//! let perm = Permute::new(vec![2, 0, 1]).unwrap();
//! let data = vec![10.0, 20.0, 30.0];
//! let result = perm.apply(&data);
//! assert_eq!(result, vec![30.0, 10.0, 20.0]);
//!
//! // Inverse permutation undoes the reordering
//! let inv = inverse_permutation(&[2, 0, 1]).unwrap();
//! assert_eq!(inv, vec![1, 2, 0]);
//! ```

use num_complex::Complex64;

/// Error types for permutation operations.
#[derive(Debug, Clone, PartialEq)]
pub enum PermuteError {
    /// An index in the permutation is out of bounds.
    IndexOutOfBounds { index: usize, len: usize },
    /// The permutation contains duplicate indices.
    DuplicateIndex(usize),
    /// Input length does not match permutation length.
    LengthMismatch { expected: usize, got: usize },
    /// Empty permutation.
    Empty,
}

impl std::fmt::Display for PermuteError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::IndexOutOfBounds { index, len } => {
                write!(f, "permutation index {} out of bounds for length {}", index, len)
            }
            Self::DuplicateIndex(idx) => write!(f, "duplicate index {} in permutation", idx),
            Self::LengthMismatch { expected, got } => {
                write!(f, "expected input length {}, got {}", expected, got)
            }
            Self::Empty => write!(f, "permutation must not be empty"),
        }
    }
}

impl std::error::Error for PermuteError {}

/// Compute the inverse of a permutation.
///
/// If `perm[i] = j`, then `inv[j] = i`.
pub fn inverse_permutation(perm: &[usize]) -> Result<Vec<usize>, PermuteError> {
    let n = perm.len();
    if n == 0 {
        return Err(PermuteError::Empty);
    }
    let mut inv = vec![0usize; n];
    let mut seen = vec![false; n];
    for (i, &j) in perm.iter().enumerate() {
        if j >= n {
            return Err(PermuteError::IndexOutOfBounds { index: j, len: n });
        }
        if seen[j] {
            return Err(PermuteError::DuplicateIndex(j));
        }
        seen[j] = true;
        inv[j] = i;
    }
    Ok(inv)
}

/// Check if a permutation is the identity (no reordering).
pub fn is_identity(perm: &[usize]) -> bool {
    perm.iter().enumerate().all(|(i, &p)| p == i)
}

/// Compose two permutations: result[i] = b[a[i]].
pub fn compose(a: &[usize], b: &[usize]) -> Result<Vec<usize>, PermuteError> {
    if a.len() != b.len() {
        return Err(PermuteError::LengthMismatch {
            expected: a.len(),
            got: b.len(),
        });
    }
    let n = a.len();
    let mut result = vec![0usize; n];
    for i in 0..n {
        if a[i] >= n {
            return Err(PermuteError::IndexOutOfBounds { index: a[i], len: n });
        }
        result[i] = b[a[i]];
    }
    Ok(result)
}

/// A validated permutation that can be applied to vectors.
#[derive(Debug, Clone)]
pub struct Permute {
    indices: Vec<usize>,
}

impl Permute {
    /// Create a new permutation from an index vector.
    ///
    /// `indices[i]` specifies which input element maps to output position `i`.
    /// That is, `output[i] = input[indices[i]]`.
    pub fn new(indices: Vec<usize>) -> Result<Self, PermuteError> {
        let n = indices.len();
        if n == 0 {
            return Err(PermuteError::Empty);
        }
        let mut seen = vec![false; n];
        for &idx in &indices {
            if idx >= n {
                return Err(PermuteError::IndexOutOfBounds { index: idx, len: n });
            }
            if seen[idx] {
                return Err(PermuteError::DuplicateIndex(idx));
            }
            seen[idx] = true;
        }
        Ok(Self { indices })
    }

    /// Create an identity permutation of given size.
    pub fn identity(size: usize) -> Result<Self, PermuteError> {
        if size == 0 {
            return Err(PermuteError::Empty);
        }
        Ok(Self {
            indices: (0..size).collect(),
        })
    }

    /// Create a bit-reversal permutation for FFT reordering.
    ///
    /// `size` must be a power of 2.
    pub fn bit_reversal(size: usize) -> Result<Self, PermuteError> {
        if size == 0 {
            return Err(PermuteError::Empty);
        }
        assert!(size.is_power_of_two(), "Size must be a power of 2");
        let bits = size.trailing_zeros() as usize;
        let indices: Vec<usize> = (0..size)
            .map(|i| {
                let mut rev = 0usize;
                let mut val = i;
                for _ in 0..bits {
                    rev = (rev << 1) | (val & 1);
                    val >>= 1;
                }
                rev
            })
            .collect();
        Self::new(indices)
    }

    /// Compute the inverse permutation.
    pub fn inverse(&self) -> Self {
        let inv = inverse_permutation(&self.indices).unwrap();
        Self { indices: inv }
    }

    /// Apply the permutation to a slice of f64.
    pub fn apply(&self, input: &[f64]) -> Vec<f64> {
        assert_eq!(input.len(), self.indices.len());
        self.indices.iter().map(|&i| input[i]).collect()
    }

    /// Apply the permutation to a slice of Complex64.
    pub fn apply_complex(&self, input: &[Complex64]) -> Vec<Complex64> {
        assert_eq!(input.len(), self.indices.len());
        self.indices.iter().map(|&i| input[i]).collect()
    }

    /// Apply the permutation to a slice of u8.
    pub fn apply_bytes(&self, input: &[u8]) -> Vec<u8> {
        assert_eq!(input.len(), self.indices.len());
        self.indices.iter().map(|&i| input[i]).collect()
    }

    /// Apply to a generic cloneable slice.
    pub fn apply_generic<T: Clone>(&self, input: &[T]) -> Vec<T> {
        assert_eq!(input.len(), self.indices.len());
        self.indices.iter().map(|&i| input[i].clone()).collect()
    }

    /// Apply in-place (uses temporary buffer).
    pub fn apply_inplace(&self, data: &mut [f64]) {
        let result = self.apply(data);
        data.copy_from_slice(&result);
    }

    /// Get permutation size.
    pub fn len(&self) -> usize {
        self.indices.len()
    }

    /// Get the index table.
    pub fn indices(&self) -> &[usize] {
        &self.indices
    }

    /// Check if this is an identity permutation.
    pub fn is_identity(&self) -> bool {
        is_identity(&self.indices)
    }
}

/// Apply a permutation to process a stream in blocks.
#[derive(Debug, Clone)]
pub struct StreamPermuter {
    permute: Permute,
    buffer: Vec<f64>,
    pos: usize,
}

impl StreamPermuter {
    /// Create a new stream permuter.
    pub fn new(permute: Permute) -> Self {
        let n = permute.len();
        Self {
            permute,
            buffer: vec![0.0; n],
            pos: 0,
        }
    }

    /// Process a stream of samples, outputting permuted blocks.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::new();
        for &sample in input {
            self.buffer[self.pos] = sample;
            self.pos += 1;
            if self.pos == self.permute.len() {
                output.extend(self.permute.apply(&self.buffer));
                self.pos = 0;
            }
        }
        output
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.buffer.fill(0.0);
        self.pos = 0;
    }

    /// Number of buffered (unprocessed) samples.
    pub fn buffered(&self) -> usize {
        self.pos
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simple_permutation() {
        let perm = Permute::new(vec![2, 0, 1]).unwrap();
        let data = vec![10.0, 20.0, 30.0];
        assert_eq!(perm.apply(&data), vec![30.0, 10.0, 20.0]);
    }

    #[test]
    fn test_identity_permutation() {
        let perm = Permute::identity(4).unwrap();
        let data = vec![1.0, 2.0, 3.0, 4.0];
        assert_eq!(perm.apply(&data), data);
        assert!(perm.is_identity());
    }

    #[test]
    fn test_inverse_permutation() {
        let perm = Permute::new(vec![2, 0, 1]).unwrap();
        let inv = perm.inverse();
        let data = vec![10.0, 20.0, 30.0];
        let permuted = perm.apply(&data);
        let restored = inv.apply(&permuted);
        assert_eq!(restored, data);
    }

    #[test]
    fn test_bit_reversal() {
        let perm = Permute::bit_reversal(8).unwrap();
        // Bit reversal of 8-element: 0→0, 1→4, 2→2, 3→6, 4→1, 5→5, 6→3, 7→7
        assert_eq!(perm.indices(), &[0, 4, 2, 6, 1, 5, 3, 7]);
    }

    #[test]
    fn test_complex_permutation() {
        let perm = Permute::new(vec![1, 0]).unwrap();
        let data = vec![Complex64::new(1.0, 2.0), Complex64::new(3.0, 4.0)];
        let result = perm.apply_complex(&data);
        assert_eq!(result[0], Complex64::new(3.0, 4.0));
        assert_eq!(result[1], Complex64::new(1.0, 2.0));
    }

    #[test]
    fn test_byte_permutation() {
        let perm = Permute::new(vec![3, 2, 1, 0]).unwrap();
        let data = vec![0xAA, 0xBB, 0xCC, 0xDD];
        assert_eq!(perm.apply_bytes(&data), vec![0xDD, 0xCC, 0xBB, 0xAA]);
    }

    #[test]
    fn test_invalid_permutation_out_of_bounds() {
        let result = Permute::new(vec![0, 5, 2]);
        assert!(matches!(result, Err(PermuteError::IndexOutOfBounds { .. })));
    }

    #[test]
    fn test_invalid_permutation_duplicate() {
        let result = Permute::new(vec![0, 1, 1]);
        assert!(matches!(result, Err(PermuteError::DuplicateIndex(_))));
    }

    #[test]
    fn test_compose_permutations() {
        let a = vec![1, 2, 0]; // rotate left
        let b = vec![1, 2, 0]; // rotate left again
        let c = compose(&a, &b).unwrap();
        assert_eq!(c, vec![2, 0, 1]); // rotated left twice
    }

    #[test]
    fn test_stream_permuter() {
        let perm = Permute::new(vec![2, 0, 1]).unwrap();
        let mut sp = StreamPermuter::new(perm);
        // Feed 6 samples (2 blocks of 3)
        let output = sp.process(&[10.0, 20.0, 30.0, 40.0, 50.0, 60.0]);
        assert_eq!(output, vec![30.0, 10.0, 20.0, 60.0, 40.0, 50.0]);
    }

    #[test]
    fn test_stream_permuter_partial() {
        let perm = Permute::new(vec![1, 0]).unwrap();
        let mut sp = StreamPermuter::new(perm);
        // Feed 3 samples: 1 full block + 1 buffered
        let output = sp.process(&[1.0, 2.0, 3.0]);
        assert_eq!(output, vec![2.0, 1.0]); // Only 1 complete block
        assert_eq!(sp.buffered(), 1);
    }

    #[test]
    fn test_apply_inplace() {
        let perm = Permute::new(vec![2, 0, 1]).unwrap();
        let mut data = vec![10.0, 20.0, 30.0];
        perm.apply_inplace(&mut data);
        assert_eq!(data, vec![30.0, 10.0, 20.0]);
    }

    #[test]
    fn test_inverse_function() {
        let inv = inverse_permutation(&[2, 0, 1]).unwrap();
        assert_eq!(inv, vec![1, 2, 0]);
    }
}
