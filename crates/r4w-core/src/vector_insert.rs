//! Vector Insert — Periodic known-sequence insertion
//!
//! Inserts pilot symbols, sync words, or training sequences into a stream
//! at regular intervals. Essential for OFDM pilot-aided channel estimation,
//! frame synchronization, and TDMA slot markers.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::vector_insert::{VectorInsert, VectorRemove};
//! use num_complex::Complex64;
//!
//! let pilots = vec![Complex64::new(1.0, 0.0), Complex64::new(-1.0, 0.0)];
//! let mut inserter = VectorInsert::new(pilots, 10, 0);
//!
//! let data: Vec<Complex64> = (0..20).map(|i| Complex64::new(i as f64 * 0.1, 0.0)).collect();
//! let with_pilots = inserter.process(&data);
//! assert!(with_pilots.len() > data.len()); // Pilots added
//! ```

use num_complex::Complex64;

/// Insert a known vector periodically into a sample stream.
#[derive(Debug, Clone)]
pub struct VectorInsert {
    /// Vector to insert (e.g., pilot symbols).
    vector: Vec<Complex64>,
    /// Period: insert every `period` output items.
    period: usize,
    /// Offset within the period where insertion starts.
    offset: usize,
    /// Current position in the period.
    counter: usize,
}

impl VectorInsert {
    pub fn new(vector: Vec<Complex64>, period: usize, offset: usize) -> Self {
        Self {
            vector,
            period: period.max(1),
            offset,
            counter: 0,
        }
    }

    /// Process input samples, inserting the vector at periodic intervals.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len() + input.len() / self.period * self.vector.len() + self.vector.len());
        let mut input_idx = 0;

        while input_idx < input.len() {
            if self.counter == self.offset {
                // Insert the vector
                output.extend_from_slice(&self.vector);
            }

            output.push(input[input_idx]);
            input_idx += 1;
            self.counter = (self.counter + 1) % self.period;
        }

        output
    }

    /// Get the overhead ratio (output_len / input_len).
    pub fn overhead_ratio(&self) -> f64 {
        1.0 + self.vector.len() as f64 / self.period as f64
    }

    pub fn reset(&mut self) {
        self.counter = 0;
    }
}

/// Remove periodically inserted vectors from a stream.
#[derive(Debug, Clone)]
pub struct VectorRemove {
    /// Length of inserted vector.
    vector_len: usize,
    /// Period.
    period: usize,
    /// Offset.
    offset: usize,
    /// Counter.
    counter: usize,
    /// Items to skip.
    skip_remaining: usize,
}

impl VectorRemove {
    pub fn new(vector_len: usize, period: usize, offset: usize) -> Self {
        Self {
            vector_len,
            period: period.max(1),
            offset,
            counter: 0,
            skip_remaining: 0,
        }
    }

    /// Remove the periodically inserted vectors.
    ///
    /// The `period` parameter should match the `period` used in `VectorInsert`.
    /// Internally, the output stream cycle is `period + vector_len` items.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let output_period = self.period + self.vector_len;
        let mut output = Vec::with_capacity(input.len());

        for &s in input {
            // Skip samples that fall within the inserted vector region
            if self.counter >= self.offset && self.counter < self.offset + self.vector_len {
                self.counter = (self.counter + 1) % output_period;
                continue;
            }

            output.push(s);
            self.counter = (self.counter + 1) % output_period;
        }

        output
    }

    pub fn reset(&mut self) {
        self.counter = 0;
        self.skip_remaining = 0;
    }
}

/// Insert real-valued training sequence.
#[derive(Debug, Clone)]
pub struct RealVectorInsert {
    vector: Vec<f64>,
    period: usize,
    offset: usize,
    counter: usize,
}

impl RealVectorInsert {
    pub fn new(vector: Vec<f64>, period: usize, offset: usize) -> Self {
        Self {
            vector,
            period: period.max(1),
            offset,
            counter: 0,
        }
    }

    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() + input.len() / self.period * self.vector.len() + self.vector.len());
        let mut input_idx = 0;

        while input_idx < input.len() {
            if self.counter == self.offset {
                output.extend_from_slice(&self.vector);
            }
            output.push(input[input_idx]);
            input_idx += 1;
            self.counter = (self.counter + 1) % self.period;
        }

        output
    }

    pub fn reset(&mut self) {
        self.counter = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_insertion() {
        let pilots = vec![Complex64::new(99.0, 0.0)];
        let mut inserter = VectorInsert::new(pilots, 4, 0);
        let data: Vec<Complex64> = (0..8)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let output = inserter.process(&data);
        // Every 4 data samples, 1 pilot should be inserted at offset 0
        assert!(output.len() > data.len());
        // First item should be the pilot (99.0)
        assert_eq!(output[0].re, 99.0);
    }

    #[test]
    fn test_overhead_ratio() {
        let pilots = vec![Complex64::new(1.0, 0.0), Complex64::new(-1.0, 0.0)];
        let inserter = VectorInsert::new(pilots, 10, 0);
        let ratio = inserter.overhead_ratio();
        assert!((ratio - 1.2).abs() < 0.01); // 2 pilots per 10 data = 20% overhead
    }

    #[test]
    fn test_insert_remove_roundtrip() {
        let pilots = vec![Complex64::new(1.0, 1.0)];
        let mut inserter = VectorInsert::new(pilots, 5, 0);
        let mut remover = VectorRemove::new(1, 5, 0);

        let data: Vec<Complex64> = (0..20)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let with_pilots = inserter.process(&data);
        let recovered = remover.process(&with_pilots);

        assert_eq!(recovered.len(), data.len());
        for (orig, recv) in data.iter().zip(recovered.iter()) {
            assert!(
                (orig - recv).norm() < 1e-10,
                "Roundtrip should preserve data"
            );
        }
    }

    #[test]
    fn test_offset_insertion() {
        let pilots = vec![Complex64::new(99.0, 0.0)];
        let mut inserter = VectorInsert::new(pilots, 5, 3);
        let data: Vec<Complex64> = (0..10)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let output = inserter.process(&data);
        // Pilot should appear at position 3 (before data[3])
        assert_eq!(output[3].re, 99.0);
    }

    #[test]
    fn test_multi_element_vector() {
        let pilots = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(-1.0, 0.0),
            Complex64::new(1.0, 0.0),
        ];
        let mut inserter = VectorInsert::new(pilots, 6, 0);
        let data: Vec<Complex64> = (0..6)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let output = inserter.process(&data);
        // 3 pilots + 6 data + possibly more = at least 9
        assert!(output.len() >= 9);
    }

    #[test]
    fn test_reset() {
        let pilots = vec![Complex64::new(99.0, 0.0)];
        let mut inserter = VectorInsert::new(pilots, 4, 0);
        inserter.process(&vec![Complex64::new(1.0, 0.0); 3]);
        inserter.reset();
        assert_eq!(inserter.counter, 0);
    }

    #[test]
    fn test_empty_input() {
        let pilots = vec![Complex64::new(1.0, 0.0)];
        let mut inserter = VectorInsert::new(pilots, 4, 0);
        let output = inserter.process(&[]);
        assert!(output.is_empty());
    }

    #[test]
    fn test_real_vector_insert() {
        let pilots = vec![99.0];
        let mut inserter = RealVectorInsert::new(pilots, 5, 0);
        let data: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let output = inserter.process(&data);
        assert!(output.len() > data.len());
        assert_eq!(output[0], 99.0);
    }

    #[test]
    fn test_vector_remove_basic() {
        let mut remover = VectorRemove::new(2, 5, 0);
        // Simulate: pilot(2), data(3), pilot(2), data(3), ...
        let input: Vec<Complex64> = vec![
            Complex64::new(99.0, 0.0), Complex64::new(99.0, 0.0), // pilots
            Complex64::new(1.0, 0.0), Complex64::new(2.0, 0.0), Complex64::new(3.0, 0.0), // data
        ];
        let output = remover.process(&input);
        assert_eq!(output.len(), 3);
        assert_eq!(output[0].re, 1.0);
    }
}
