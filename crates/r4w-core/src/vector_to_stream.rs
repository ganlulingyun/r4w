//! # Vector to Stream
//!
//! Converts fixed-length vectors (blocks) back to a flat sample stream.
//! The inverse of `stream_to_vector`. Essential for interfacing between
//! vector-based processing (FFT, OFDM symbols) and sample-based processing.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::vector_to_stream::VectorToStream;
//!
//! let v2s = VectorToStream::new(4);
//! let vectors = vec![
//!     vec![1.0, 2.0, 3.0, 4.0],
//!     vec![5.0, 6.0, 7.0, 8.0],
//! ];
//! let stream = v2s.process(&vectors);
//! assert_eq!(stream, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]);
//! ```

/// Vector to stream converter.
#[derive(Debug, Clone)]
pub struct VectorToStream {
    /// Expected vector size.
    vector_size: usize,
    /// Total vectors processed.
    vectors_processed: u64,
    /// Total samples output.
    samples_output: u64,
}

impl VectorToStream {
    /// Create a new converter with the expected vector size.
    pub fn new(vector_size: usize) -> Self {
        Self {
            vector_size,
            vectors_processed: 0,
            samples_output: 0,
        }
    }

    /// Convert vectors to a flat stream of real samples.
    pub fn process(&self, vectors: &[Vec<f64>]) -> Vec<f64> {
        let mut stream = Vec::with_capacity(vectors.len() * self.vector_size);
        for v in vectors {
            stream.extend_from_slice(v);
        }
        stream
    }

    /// Convert vectors to a flat stream, tracking counts.
    pub fn process_counted(&mut self, vectors: &[Vec<f64>]) -> Vec<f64> {
        let mut stream = Vec::with_capacity(vectors.len() * self.vector_size);
        for v in vectors {
            stream.extend_from_slice(v);
            self.vectors_processed += 1;
            self.samples_output += v.len() as u64;
        }
        stream
    }

    /// Convert complex vectors to a flat complex stream.
    pub fn process_complex(&self, vectors: &[Vec<(f64, f64)>]) -> Vec<(f64, f64)> {
        let mut stream = Vec::with_capacity(vectors.len() * self.vector_size);
        for v in vectors {
            stream.extend_from_slice(v);
        }
        stream
    }

    /// Convert vectors with overlap-add (for STFT reconstruction).
    pub fn overlap_add(&self, vectors: &[Vec<f64>], hop_size: usize) -> Vec<f64> {
        if vectors.is_empty() {
            return Vec::new();
        }

        let total_len = (vectors.len() - 1) * hop_size + self.vector_size;
        let mut output = vec![0.0; total_len];

        for (i, v) in vectors.iter().enumerate() {
            let start = i * hop_size;
            for (j, &sample) in v.iter().enumerate() {
                if start + j < output.len() {
                    output[start + j] += sample;
                }
            }
        }

        output
    }

    /// Convert vectors with an extraction window (take a subset of each vector).
    pub fn process_windowed(&self, vectors: &[Vec<f64>], start: usize, len: usize) -> Vec<f64> {
        let mut stream = Vec::with_capacity(vectors.len() * len);
        for v in vectors {
            let end = (start + len).min(v.len());
            if start < v.len() {
                stream.extend_from_slice(&v[start..end]);
            }
        }
        stream
    }

    /// Get the expected vector size.
    pub fn vector_size(&self) -> usize {
        self.vector_size
    }

    /// Get total vectors processed (only counted version).
    pub fn vectors_processed(&self) -> u64 {
        self.vectors_processed
    }

    /// Get total samples output (only counted version).
    pub fn samples_output(&self) -> u64 {
        self.samples_output
    }

    /// Reset counters.
    pub fn reset(&mut self) {
        self.vectors_processed = 0;
        self.samples_output = 0;
    }
}

/// Simple flatten: convert a slice of slices to a flat Vec.
pub fn flatten_f64(vectors: &[&[f64]]) -> Vec<f64> {
    let total: usize = vectors.iter().map(|v| v.len()).sum();
    let mut out = Vec::with_capacity(total);
    for v in vectors {
        out.extend_from_slice(v);
    }
    out
}

/// Simple flatten for complex samples.
pub fn flatten_complex(vectors: &[&[(f64, f64)]]) -> Vec<(f64, f64)> {
    let total: usize = vectors.iter().map(|v| v.len()).sum();
    let mut out = Vec::with_capacity(total);
    for v in vectors {
        out.extend_from_slice(v);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_flatten() {
        let v2s = VectorToStream::new(3);
        let vectors = vec![
            vec![1.0, 2.0, 3.0],
            vec![4.0, 5.0, 6.0],
        ];
        let stream = v2s.process(&vectors);
        assert_eq!(stream, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn test_counted() {
        let mut v2s = VectorToStream::new(2);
        let vectors = vec![vec![1.0, 2.0], vec![3.0, 4.0], vec![5.0, 6.0]];
        let stream = v2s.process_counted(&vectors);
        assert_eq!(stream.len(), 6);
        assert_eq!(v2s.vectors_processed(), 3);
        assert_eq!(v2s.samples_output(), 6);
    }

    #[test]
    fn test_complex() {
        let v2s = VectorToStream::new(2);
        let vectors = vec![
            vec![(1.0, 2.0), (3.0, 4.0)],
            vec![(5.0, 6.0), (7.0, 8.0)],
        ];
        let stream = v2s.process_complex(&vectors);
        assert_eq!(stream.len(), 4);
        assert_eq!(stream[2], (5.0, 6.0));
    }

    #[test]
    fn test_overlap_add() {
        let v2s = VectorToStream::new(4);
        // Two windows of size 4 with hop=2.
        let vectors = vec![
            vec![1.0, 1.0, 1.0, 1.0],
            vec![2.0, 2.0, 2.0, 2.0],
        ];
        let output = v2s.overlap_add(&vectors, 2);
        // Length = (2-1)*2 + 4 = 6
        assert_eq!(output.len(), 6);
        // Overlap region [2..4]: 1+2=3
        assert_eq!(output[0], 1.0);
        assert_eq!(output[1], 1.0);
        assert_eq!(output[2], 3.0); // 1+2
        assert_eq!(output[3], 3.0); // 1+2
        assert_eq!(output[4], 2.0);
        assert_eq!(output[5], 2.0);
    }

    #[test]
    fn test_windowed() {
        let v2s = VectorToStream::new(8);
        let vectors = vec![
            vec![0.0, 0.0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0],
            vec![0.0, 0.0, 4.0, 5.0, 6.0, 0.0, 0.0, 0.0],
        ];
        // Extract samples [2..5) from each vector.
        let stream = v2s.process_windowed(&vectors, 2, 3);
        assert_eq!(stream, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn test_empty_vectors() {
        let v2s = VectorToStream::new(4);
        let stream = v2s.process(&[]);
        assert!(stream.is_empty());
    }

    #[test]
    fn test_overlap_add_empty() {
        let v2s = VectorToStream::new(4);
        let output = v2s.overlap_add(&[], 2);
        assert!(output.is_empty());
    }

    #[test]
    fn test_flatten_f64() {
        let a = [1.0, 2.0];
        let b = [3.0, 4.0, 5.0];
        let flat = flatten_f64(&[&a, &b]);
        assert_eq!(flat, vec![1.0, 2.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn test_flatten_complex() {
        let a = [(1.0, 0.0), (2.0, 0.0)];
        let b = [(3.0, 0.0)];
        let flat = flatten_complex(&[&a, &b]);
        assert_eq!(flat.len(), 3);
    }

    #[test]
    fn test_reset() {
        let mut v2s = VectorToStream::new(2);
        v2s.process_counted(&[vec![1.0, 2.0]]);
        assert_eq!(v2s.vectors_processed(), 1);
        v2s.reset();
        assert_eq!(v2s.vectors_processed(), 0);
        assert_eq!(v2s.samples_output(), 0);
    }
}
