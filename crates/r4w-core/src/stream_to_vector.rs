//! Stream/Vector Conversion Blocks
//!
//! Convert between scalar streams and vector (block) formats. Essential for
//! FFT-based processing (OFDM), MIMO, and any block-oriented pipeline.
//!
//! ## Blocks
//!
//! - **StreamToVector**: Collect N scalar samples into vectors of length N
//! - **VectorToStream**: Serialize vectors back to a scalar stream
//! - **Interleave**: Combine N streams by interleaving samples
//! - **Deinterleave**: Split one stream into N by deinterleaving
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::stream_to_vector::{StreamToVector, VectorToStream};
//! use num_complex::Complex64;
//!
//! let mut s2v = StreamToVector::new(4);
//! let input = vec![Complex64::new(1.0, 0.0); 12];
//! let vectors = s2v.process(&input);
//! assert_eq!(vectors.len(), 3); // 12 / 4 = 3 complete vectors
//!
//! let stream = VectorToStream::process(&vectors);
//! assert_eq!(stream.len(), 12);
//! ```

use num_complex::Complex64;

/// Collect scalar samples into fixed-size vectors.
#[derive(Debug, Clone)]
pub struct StreamToVector {
    vector_size: usize,
    buffer: Vec<Complex64>,
}

impl StreamToVector {
    pub fn new(vector_size: usize) -> Self {
        assert!(vector_size > 0);
        Self {
            vector_size,
            buffer: Vec::with_capacity(vector_size),
        }
    }

    /// Process input samples, returning complete vectors.
    /// Partial data is buffered for the next call.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Vec<Complex64>> {
        let mut output = Vec::new();
        for &s in input {
            self.buffer.push(s);
            if self.buffer.len() == self.vector_size {
                output.push(std::mem::replace(
                    &mut self.buffer,
                    Vec::with_capacity(self.vector_size),
                ));
            }
        }
        output
    }

    /// Process real-valued samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<Vec<f64>> {
        // Use a simple non-stateful approach for real values
        input.chunks(self.vector_size)
            .filter(|c| c.len() == self.vector_size)
            .map(|c| c.to_vec())
            .collect()
    }

    pub fn vector_size(&self) -> usize {
        self.vector_size
    }

    pub fn buffered(&self) -> usize {
        self.buffer.len()
    }

    pub fn reset(&mut self) {
        self.buffer.clear();
    }
}

/// Serialize vectors back to a scalar stream.
pub struct VectorToStream;

impl VectorToStream {
    /// Flatten vectors into a single stream.
    pub fn process(vectors: &[Vec<Complex64>]) -> Vec<Complex64> {
        vectors.iter().flat_map(|v| v.iter().copied()).collect()
    }

    /// Flatten a single vector.
    pub fn process_single(vector: &[Complex64]) -> Vec<Complex64> {
        vector.to_vec()
    }

    /// Flatten real-valued vectors.
    pub fn process_real(vectors: &[Vec<f64>]) -> Vec<f64> {
        vectors.iter().flat_map(|v| v.iter().copied()).collect()
    }
}

/// Interleave N streams into one by taking samples round-robin.
#[derive(Debug, Clone)]
pub struct Interleave {
    num_streams: usize,
    blocksize: usize,
}

impl Interleave {
    /// Create an interleaver with blocksize = 1 (sample-by-sample).
    pub fn new(num_streams: usize) -> Self {
        assert!(num_streams > 0);
        Self {
            num_streams,
            blocksize: 1,
        }
    }

    /// Create an interleaver with specified blocksize.
    pub fn with_blocksize(num_streams: usize, blocksize: usize) -> Self {
        assert!(num_streams > 0);
        assert!(blocksize > 0);
        Self {
            num_streams,
            blocksize,
        }
    }

    /// Interleave complex streams.
    pub fn process(&self, streams: &[&[Complex64]]) -> Vec<Complex64> {
        assert_eq!(streams.len(), self.num_streams);
        let min_len = streams.iter().map(|s| s.len()).min().unwrap_or(0);
        let num_blocks = min_len / self.blocksize;
        let mut output = Vec::with_capacity(num_blocks * self.blocksize * self.num_streams);

        for block_idx in 0..num_blocks {
            for stream in streams {
                let start = block_idx * self.blocksize;
                output.extend_from_slice(&stream[start..start + self.blocksize]);
            }
        }
        output
    }

    /// Interleave real-valued streams.
    pub fn process_real(&self, streams: &[&[f64]]) -> Vec<f64> {
        assert_eq!(streams.len(), self.num_streams);
        let min_len = streams.iter().map(|s| s.len()).min().unwrap_or(0);
        let num_blocks = min_len / self.blocksize;
        let mut output = Vec::with_capacity(num_blocks * self.blocksize * self.num_streams);

        for block_idx in 0..num_blocks {
            for stream in streams {
                let start = block_idx * self.blocksize;
                output.extend_from_slice(&stream[start..start + self.blocksize]);
            }
        }
        output
    }
}

/// Deinterleave one stream into N streams.
#[derive(Debug, Clone)]
pub struct Deinterleave {
    num_streams: usize,
    blocksize: usize,
}

impl Deinterleave {
    pub fn new(num_streams: usize) -> Self {
        assert!(num_streams > 0);
        Self {
            num_streams,
            blocksize: 1,
        }
    }

    pub fn with_blocksize(num_streams: usize, blocksize: usize) -> Self {
        assert!(num_streams > 0);
        assert!(blocksize > 0);
        Self {
            num_streams,
            blocksize,
        }
    }

    /// Deinterleave complex samples into N streams.
    pub fn process(&self, input: &[Complex64]) -> Vec<Vec<Complex64>> {
        let chunk_size = self.num_streams * self.blocksize;
        let num_chunks = input.len() / chunk_size;
        let mut streams: Vec<Vec<Complex64>> = (0..self.num_streams)
            .map(|_| Vec::with_capacity(num_chunks * self.blocksize))
            .collect();

        for chunk in input.chunks_exact(chunk_size) {
            for (stream_idx, stream) in streams.iter_mut().enumerate() {
                let start = stream_idx * self.blocksize;
                stream.extend_from_slice(&chunk[start..start + self.blocksize]);
            }
        }
        streams
    }

    /// Deinterleave real-valued samples.
    pub fn process_real(&self, input: &[f64]) -> Vec<Vec<f64>> {
        let chunk_size = self.num_streams * self.blocksize;
        let num_chunks = input.len() / chunk_size;
        let mut streams: Vec<Vec<f64>> = (0..self.num_streams)
            .map(|_| Vec::with_capacity(num_chunks * self.blocksize))
            .collect();

        for chunk in input.chunks_exact(chunk_size) {
            for (stream_idx, stream) in streams.iter_mut().enumerate() {
                let start = stream_idx * self.blocksize;
                stream.extend_from_slice(&chunk[start..start + self.blocksize]);
            }
        }
        streams
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stream_to_vector_basic() {
        let mut s2v = StreamToVector::new(4);
        let input: Vec<Complex64> = (0..12)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let vectors = s2v.process(&input);
        assert_eq!(vectors.len(), 3);
        assert_eq!(vectors[0].len(), 4);
        assert!((vectors[0][0].re - 0.0).abs() < 1e-10);
        assert!((vectors[1][0].re - 4.0).abs() < 1e-10);
        assert!((vectors[2][0].re - 8.0).abs() < 1e-10);
    }

    #[test]
    fn test_stream_to_vector_partial() {
        let mut s2v = StreamToVector::new(4);
        let input: Vec<Complex64> = (0..6)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let v1 = s2v.process(&input);
        assert_eq!(v1.len(), 1); // 4 complete, 2 buffered
        assert_eq!(s2v.buffered(), 2);

        // Feed more to complete the next vector
        let v2 = s2v.process(&[Complex64::new(6.0, 0.0), Complex64::new(7.0, 0.0)]);
        assert_eq!(v2.len(), 1);
        assert!((v2[0][0].re - 4.0).abs() < 1e-10); // Continued from previous buffer
    }

    #[test]
    fn test_vector_to_stream_roundtrip() {
        let mut s2v = StreamToVector::new(4);
        let input: Vec<Complex64> = (0..8)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let vectors = s2v.process(&input);
        let stream = VectorToStream::process(&vectors);
        assert_eq!(stream.len(), 8);
        for (i, s) in stream.iter().enumerate() {
            assert!((s.re - i as f64).abs() < 1e-10);
        }
    }

    #[test]
    fn test_interleave_basic() {
        let il = Interleave::new(2);
        let a = vec![Complex64::new(1.0, 0.0), Complex64::new(2.0, 0.0), Complex64::new(3.0, 0.0)];
        let b = vec![Complex64::new(10.0, 0.0), Complex64::new(20.0, 0.0), Complex64::new(30.0, 0.0)];
        let out = il.process(&[&a, &b]);
        assert_eq!(out.len(), 6);
        assert!((out[0].re - 1.0).abs() < 1e-10);
        assert!((out[1].re - 10.0).abs() < 1e-10);
        assert!((out[2].re - 2.0).abs() < 1e-10);
        assert!((out[3].re - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_interleave_blocksize() {
        let il = Interleave::with_blocksize(2, 2);
        let a = vec![Complex64::new(1.0, 0.0), Complex64::new(2.0, 0.0), Complex64::new(3.0, 0.0), Complex64::new(4.0, 0.0)];
        let b = vec![Complex64::new(10.0, 0.0), Complex64::new(20.0, 0.0), Complex64::new(30.0, 0.0), Complex64::new(40.0, 0.0)];
        let out = il.process(&[&a, &b]);
        // blocksize=2: [1,2,10,20,3,4,30,40]
        assert_eq!(out.len(), 8);
        assert!((out[0].re - 1.0).abs() < 1e-10);
        assert!((out[1].re - 2.0).abs() < 1e-10);
        assert!((out[2].re - 10.0).abs() < 1e-10);
        assert!((out[3].re - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_deinterleave_basic() {
        let di = Deinterleave::new(2);
        let input = vec![
            Complex64::new(1.0, 0.0), Complex64::new(10.0, 0.0),
            Complex64::new(2.0, 0.0), Complex64::new(20.0, 0.0),
        ];
        let streams = di.process(&input);
        assert_eq!(streams.len(), 2);
        assert_eq!(streams[0].len(), 2);
        assert!((streams[0][0].re - 1.0).abs() < 1e-10);
        assert!((streams[0][1].re - 2.0).abs() < 1e-10);
        assert!((streams[1][0].re - 10.0).abs() < 1e-10);
        assert!((streams[1][1].re - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_interleave_deinterleave_roundtrip() {
        let a = vec![Complex64::new(1.0, 0.0), Complex64::new(2.0, 0.0), Complex64::new(3.0, 0.0)];
        let b = vec![Complex64::new(4.0, 0.0), Complex64::new(5.0, 0.0), Complex64::new(6.0, 0.0)];
        let interleaved = Interleave::new(2).process(&[&a, &b]);
        let streams = Deinterleave::new(2).process(&interleaved);
        assert_eq!(streams[0], a);
        assert_eq!(streams[1], b);
    }

    #[test]
    fn test_deinterleave_blocksize() {
        let di = Deinterleave::with_blocksize(2, 2);
        let input = vec![
            Complex64::new(1.0, 0.0), Complex64::new(2.0, 0.0),
            Complex64::new(10.0, 0.0), Complex64::new(20.0, 0.0),
        ];
        let streams = di.process(&input);
        assert_eq!(streams[0], vec![Complex64::new(1.0, 0.0), Complex64::new(2.0, 0.0)]);
        assert_eq!(streams[1], vec![Complex64::new(10.0, 0.0), Complex64::new(20.0, 0.0)]);
    }

    #[test]
    fn test_interleave_real() {
        let il = Interleave::new(3);
        let a: Vec<f64> = vec![1.0, 2.0];
        let b: Vec<f64> = vec![10.0, 20.0];
        let c: Vec<f64> = vec![100.0, 200.0];
        let out = il.process_real(&[&a, &b, &c]);
        assert_eq!(out, vec![1.0, 10.0, 100.0, 2.0, 20.0, 200.0]);
    }

    #[test]
    fn test_stream_to_vector_empty() {
        let mut s2v = StreamToVector::new(4);
        assert!(s2v.process(&[]).is_empty());
    }

    #[test]
    fn test_stream_to_vector_reset() {
        let mut s2v = StreamToVector::new(4);
        s2v.process(&[Complex64::new(1.0, 0.0), Complex64::new(2.0, 0.0)]);
        assert_eq!(s2v.buffered(), 2);
        s2v.reset();
        assert_eq!(s2v.buffered(), 0);
    }

    #[test]
    fn test_stream_to_vector_real() {
        let mut s2v = StreamToVector::new(3);
        let vectors = s2v.process_real(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]);
        assert_eq!(vectors.len(), 2); // 7/3 = 2 complete + remainder
        assert_eq!(vectors[0], vec![1.0, 2.0, 3.0]);
        assert_eq!(vectors[1], vec![4.0, 5.0, 6.0]);
    }
}
