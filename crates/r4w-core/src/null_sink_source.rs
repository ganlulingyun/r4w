//! Null Sink, Null Source, Vector Source, and Tee
//!
//! Pipeline plumbing primitives for terminating, generating, and splitting streams.
//!
//! ## Blocks
//!
//! - **NullSource**: Generates zero-valued samples (placeholder/test source)
//! - **NullSink**: Consumes and discards samples (bit bucket)
//! - **VectorSource**: Outputs data from a predefined vector (test signal generator)
//! - **Tee**: Splits one stream into N identical copies
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::null_sink_source::{NullSource, NullSink, VectorSource};
//!
//! let src = NullSource::new();
//! let zeros = src.generate(10);
//! assert_eq!(zeros.len(), 10);
//! assert!(zeros.iter().all(|z| z.norm() < 1e-10));
//!
//! let mut vs = VectorSource::new(vec![1.0, 2.0, 3.0], true);
//! let data = vs.generate(7);
//! assert_eq!(data, vec![1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0]);
//! ```

use num_complex::Complex64;

/// Null source — generates zero-valued samples.
#[derive(Debug, Clone)]
pub struct NullSource {
    sample_rate: f64,
}

impl NullSource {
    pub fn new() -> Self {
        Self { sample_rate: 0.0 }
    }

    pub fn with_sample_rate(rate: f64) -> Self {
        Self { sample_rate: rate }
    }

    /// Generate N complex zero samples.
    pub fn generate(&self, num_samples: usize) -> Vec<Complex64> {
        vec![Complex64::new(0.0, 0.0); num_samples]
    }

    /// Generate N real zero samples.
    pub fn generate_real(&self, num_samples: usize) -> Vec<f64> {
        vec![0.0; num_samples]
    }

    /// Generate N false bits.
    pub fn generate_bits(&self, num_bits: usize) -> Vec<bool> {
        vec![false; num_bits]
    }

    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }
}

impl Default for NullSource {
    fn default() -> Self {
        Self::new()
    }
}

/// Null sink — consumes and discards samples.
#[derive(Debug, Clone)]
pub struct NullSink {
    samples_consumed: u64,
}

impl NullSink {
    pub fn new() -> Self {
        Self {
            samples_consumed: 0,
        }
    }

    /// Consume complex samples.
    pub fn consume(&mut self, samples: &[Complex64]) {
        self.samples_consumed += samples.len() as u64;
    }

    /// Consume real samples.
    pub fn consume_real(&mut self, samples: &[f64]) {
        self.samples_consumed += samples.len() as u64;
    }

    /// Consume bits.
    pub fn consume_bits(&mut self, bits: &[bool]) {
        self.samples_consumed += bits.len() as u64;
    }

    pub fn samples_consumed(&self) -> u64 {
        self.samples_consumed
    }

    pub fn reset(&mut self) {
        self.samples_consumed = 0;
    }
}

impl Default for NullSink {
    fn default() -> Self {
        Self::new()
    }
}

/// Vector source — outputs data from a predefined vector.
///
/// Useful for test signals. When `repeat` is true, wraps around after
/// the end of the data.
#[derive(Debug, Clone)]
pub struct VectorSource<T: Clone> {
    data: Vec<T>,
    repeat: bool,
    position: usize,
}

impl<T: Clone> VectorSource<T> {
    pub fn new(data: Vec<T>, repeat: bool) -> Self {
        Self {
            data,
            repeat,
            position: 0,
        }
    }

    /// Generate up to `num_samples` from the vector.
    pub fn generate(&mut self, num_samples: usize) -> Vec<T> {
        if self.data.is_empty() {
            return Vec::new();
        }

        let mut output = Vec::with_capacity(num_samples);
        for _ in 0..num_samples {
            if self.position >= self.data.len() {
                if self.repeat {
                    self.position = 0;
                } else {
                    break;
                }
            }
            output.push(self.data[self.position].clone());
            self.position += 1;
        }
        output
    }

    /// Check if all data has been consumed (only meaningful when repeat=false).
    pub fn finished(&self) -> bool {
        !self.repeat && self.position >= self.data.len()
    }

    pub fn reset(&mut self) {
        self.position = 0;
    }

    pub fn len(&self) -> usize {
        self.data.len()
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }
}

/// Tee — splits one stream into N identical copies.
#[derive(Debug, Clone)]
pub struct Tee {
    num_outputs: usize,
}

impl Tee {
    pub fn new(num_outputs: usize) -> Self {
        assert!(num_outputs > 0);
        Self { num_outputs }
    }

    /// Split complex samples into N copies.
    pub fn split(&self, input: &[Complex64]) -> Vec<Vec<Complex64>> {
        (0..self.num_outputs).map(|_| input.to_vec()).collect()
    }

    /// Split real samples into N copies.
    pub fn split_real(&self, input: &[f64]) -> Vec<Vec<f64>> {
        (0..self.num_outputs).map(|_| input.to_vec()).collect()
    }

    pub fn num_outputs(&self) -> usize {
        self.num_outputs
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_null_source_complex() {
        let src = NullSource::new();
        let output = src.generate(100);
        assert_eq!(output.len(), 100);
        for z in &output {
            assert_eq!(*z, Complex64::new(0.0, 0.0));
        }
    }

    #[test]
    fn test_null_source_real() {
        let src = NullSource::new();
        let output = src.generate_real(50);
        assert_eq!(output.len(), 50);
        assert!(output.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn test_null_source_bits() {
        let src = NullSource::new();
        let bits = src.generate_bits(32);
        assert_eq!(bits.len(), 32);
        assert!(bits.iter().all(|&b| !b));
    }

    #[test]
    fn test_null_sink_count() {
        let mut sink = NullSink::new();
        sink.consume(&vec![Complex64::new(1.0, 0.0); 100]);
        sink.consume_real(&vec![1.0; 50]);
        assert_eq!(sink.samples_consumed(), 150);
    }

    #[test]
    fn test_null_sink_reset() {
        let mut sink = NullSink::new();
        sink.consume_real(&vec![1.0; 100]);
        assert_eq!(sink.samples_consumed(), 100);
        sink.reset();
        assert_eq!(sink.samples_consumed(), 0);
    }

    #[test]
    fn test_vector_source_no_repeat() {
        let mut vs = VectorSource::new(vec![1.0, 2.0, 3.0], false);
        let out = vs.generate(5);
        assert_eq!(out, vec![1.0, 2.0, 3.0]); // Only 3 available
        assert!(vs.finished());
    }

    #[test]
    fn test_vector_source_repeat() {
        let mut vs = VectorSource::new(vec![1.0, 2.0, 3.0], true);
        let out = vs.generate(7);
        assert_eq!(out, vec![1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0]);
        assert!(!vs.finished());
    }

    #[test]
    fn test_vector_source_streaming() {
        let mut vs = VectorSource::new(vec![1.0, 2.0, 3.0], false);
        let o1 = vs.generate(2);
        let o2 = vs.generate(2);
        assert_eq!(o1, vec![1.0, 2.0]);
        assert_eq!(o2, vec![3.0]); // Only 1 left
    }

    #[test]
    fn test_vector_source_reset() {
        let mut vs = VectorSource::new(vec![1.0, 2.0], false);
        vs.generate(3);
        assert!(vs.finished());
        vs.reset();
        assert!(!vs.finished());
        let out = vs.generate(2);
        assert_eq!(out, vec![1.0, 2.0]);
    }

    #[test]
    fn test_vector_source_empty() {
        let mut vs: VectorSource<f64> = VectorSource::new(vec![], true);
        assert!(vs.generate(10).is_empty());
        assert!(vs.is_empty());
    }

    #[test]
    fn test_vector_source_complex() {
        let data = vec![Complex64::new(1.0, 2.0), Complex64::new(3.0, 4.0)];
        let mut vs = VectorSource::new(data.clone(), false);
        let out = vs.generate(2);
        assert_eq!(out, data);
    }

    #[test]
    fn test_tee_basic() {
        let tee = Tee::new(3);
        let input = vec![Complex64::new(1.0, 0.0), Complex64::new(2.0, 0.0)];
        let outputs = tee.split(&input);
        assert_eq!(outputs.len(), 3);
        for out in &outputs {
            assert_eq!(out, &input);
        }
    }

    #[test]
    fn test_tee_real() {
        let tee = Tee::new(2);
        let input = vec![1.0, 2.0, 3.0];
        let outputs = tee.split_real(&input);
        assert_eq!(outputs.len(), 2);
        assert_eq!(outputs[0], input);
        assert_eq!(outputs[1], input);
    }

    #[test]
    fn test_tee_single() {
        let tee = Tee::new(1);
        let input = vec![Complex64::new(5.0, 3.0)];
        let outputs = tee.split(&input);
        assert_eq!(outputs.len(), 1);
        assert_eq!(outputs[0], input);
    }

    #[test]
    fn test_null_source_sample_rate() {
        let src = NullSource::with_sample_rate(48000.0);
        assert_eq!(src.sample_rate(), 48000.0);
    }
}
