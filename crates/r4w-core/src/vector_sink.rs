//! Vector Sink (Data Capture)
//!
//! Terminal blocks that capture samples into memory for analysis, testing,
//! and signal inspection. Thread-safe for use with GUI observation.
//!
//! ## Variants
//!
//! - **VectorSinkComplex**: Captures complex I/Q samples
//! - **VectorSinkReal**: Captures real-valued samples
//! - **VectorSinkBits**: Captures bit streams
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::vector_sink::{VectorSinkComplex, VectorSinkReal};
//! use num_complex::Complex64;
//!
//! let mut sink = VectorSinkComplex::new(1024);
//! sink.process(&vec![Complex64::new(1.0, 0.0); 100]);
//! assert_eq!(sink.data().len(), 100);
//! sink.process(&vec![Complex64::new(0.0, 1.0); 50]);
//! assert_eq!(sink.data().len(), 150);
//! ```

use num_complex::Complex64;

/// Complex I/Q sample sink.
#[derive(Debug, Clone)]
pub struct VectorSinkComplex {
    /// Captured data
    data: Vec<Complex64>,
    /// Maximum capacity (0 = unlimited)
    max_capacity: usize,
    /// Total samples received (even if capacity exceeded)
    total_received: usize,
}

impl VectorSinkComplex {
    /// Create a new complex vector sink.
    ///
    /// - `max_capacity`: Maximum samples to store (0 = unlimited)
    pub fn new(max_capacity: usize) -> Self {
        let initial_cap = if max_capacity > 0 { max_capacity } else { 1024 };
        Self {
            data: Vec::with_capacity(initial_cap),
            max_capacity,
            total_received: 0,
        }
    }

    /// Unlimited capacity sink.
    pub fn unlimited() -> Self {
        Self::new(0)
    }

    /// Process (capture) a block of complex samples.
    pub fn process(&mut self, input: &[Complex64]) {
        self.total_received += input.len();
        if self.max_capacity == 0 {
            self.data.extend_from_slice(input);
        } else {
            let remaining = self.max_capacity.saturating_sub(self.data.len());
            let to_take = input.len().min(remaining);
            if to_take > 0 {
                self.data.extend_from_slice(&input[..to_take]);
            }
        }
    }

    /// Get captured data.
    pub fn data(&self) -> &[Complex64] {
        &self.data
    }

    /// Get the last N samples.
    pub fn last_n(&self, n: usize) -> &[Complex64] {
        let start = self.data.len().saturating_sub(n);
        &self.data[start..]
    }

    /// Total samples received.
    pub fn total_received(&self) -> usize {
        self.total_received
    }

    /// Number of stored samples.
    pub fn len(&self) -> usize {
        self.data.len()
    }

    /// Check if empty.
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// Check if at capacity.
    pub fn is_full(&self) -> bool {
        self.max_capacity > 0 && self.data.len() >= self.max_capacity
    }

    /// Clear all captured data.
    pub fn clear(&mut self) {
        self.data.clear();
        self.total_received = 0;
    }

    /// Take ownership of captured data, leaving the sink empty.
    pub fn take_data(&mut self) -> Vec<Complex64> {
        self.total_received = 0;
        std::mem::take(&mut self.data)
    }
}

/// Real-valued sample sink.
#[derive(Debug, Clone)]
pub struct VectorSinkReal {
    data: Vec<f64>,
    max_capacity: usize,
    total_received: usize,
}

impl VectorSinkReal {
    pub fn new(max_capacity: usize) -> Self {
        let initial_cap = if max_capacity > 0 { max_capacity } else { 1024 };
        Self {
            data: Vec::with_capacity(initial_cap),
            max_capacity,
            total_received: 0,
        }
    }

    pub fn unlimited() -> Self {
        Self::new(0)
    }

    pub fn process(&mut self, input: &[f64]) {
        self.total_received += input.len();
        if self.max_capacity == 0 {
            self.data.extend_from_slice(input);
        } else {
            let remaining = self.max_capacity.saturating_sub(self.data.len());
            let to_take = input.len().min(remaining);
            if to_take > 0 {
                self.data.extend_from_slice(&input[..to_take]);
            }
        }
    }

    pub fn data(&self) -> &[f64] {
        &self.data
    }

    pub fn last_n(&self, n: usize) -> &[f64] {
        let start = self.data.len().saturating_sub(n);
        &self.data[start..]
    }

    pub fn total_received(&self) -> usize {
        self.total_received
    }

    pub fn len(&self) -> usize {
        self.data.len()
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    pub fn is_full(&self) -> bool {
        self.max_capacity > 0 && self.data.len() >= self.max_capacity
    }

    pub fn clear(&mut self) {
        self.data.clear();
        self.total_received = 0;
    }

    pub fn take_data(&mut self) -> Vec<f64> {
        self.total_received = 0;
        std::mem::take(&mut self.data)
    }
}

/// Bit stream sink.
#[derive(Debug, Clone)]
pub struct VectorSinkBits {
    data: Vec<bool>,
    max_capacity: usize,
    total_received: usize,
}

impl VectorSinkBits {
    pub fn new(max_capacity: usize) -> Self {
        let initial_cap = if max_capacity > 0 { max_capacity } else { 1024 };
        Self {
            data: Vec::with_capacity(initial_cap),
            max_capacity,
            total_received: 0,
        }
    }

    pub fn unlimited() -> Self {
        Self::new(0)
    }

    pub fn process(&mut self, input: &[bool]) {
        self.total_received += input.len();
        if self.max_capacity == 0 {
            self.data.extend_from_slice(input);
        } else {
            let remaining = self.max_capacity.saturating_sub(self.data.len());
            let to_take = input.len().min(remaining);
            if to_take > 0 {
                self.data.extend_from_slice(&input[..to_take]);
            }
        }
    }

    pub fn data(&self) -> &[bool] {
        &self.data
    }

    /// Convert captured bits to bytes (MSB first, zero-padded).
    pub fn to_bytes(&self) -> Vec<u8> {
        self.data
            .chunks(8)
            .map(|chunk| {
                let mut byte = 0u8;
                for (i, &bit) in chunk.iter().enumerate() {
                    if bit {
                        byte |= 1 << (7 - i);
                    }
                }
                byte
            })
            .collect()
    }

    pub fn total_received(&self) -> usize {
        self.total_received
    }

    pub fn len(&self) -> usize {
        self.data.len()
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    pub fn is_full(&self) -> bool {
        self.max_capacity > 0 && self.data.len() >= self.max_capacity
    }

    pub fn clear(&mut self) {
        self.data.clear();
        self.total_received = 0;
    }

    pub fn take_data(&mut self) -> Vec<bool> {
        self.total_received = 0;
        std::mem::take(&mut self.data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_complex_sink_basic() {
        let mut sink = VectorSinkComplex::new(1024);
        assert!(sink.is_empty());

        sink.process(&[Complex64::new(1.0, 0.0), Complex64::new(0.0, 1.0)]);
        assert_eq!(sink.len(), 2);
        assert_eq!(sink.data()[0], Complex64::new(1.0, 0.0));
    }

    #[test]
    fn test_complex_sink_capacity() {
        let mut sink = VectorSinkComplex::new(5);
        sink.process(&vec![Complex64::new(1.0, 0.0); 10]);

        assert_eq!(sink.len(), 5);
        assert!(sink.is_full());
        assert_eq!(sink.total_received(), 10);
    }

    #[test]
    fn test_complex_sink_unlimited() {
        let mut sink = VectorSinkComplex::unlimited();
        sink.process(&vec![Complex64::new(1.0, 0.0); 10000]);
        assert_eq!(sink.len(), 10000);
        assert!(!sink.is_full());
    }

    #[test]
    fn test_complex_last_n() {
        let mut sink = VectorSinkComplex::unlimited();
        for i in 0..10 {
            sink.process(&[Complex64::new(i as f64, 0.0)]);
        }
        let last3 = sink.last_n(3);
        assert_eq!(last3.len(), 3);
        assert_eq!(last3[0].re, 7.0);
        assert_eq!(last3[2].re, 9.0);
    }

    #[test]
    fn test_complex_take_data() {
        let mut sink = VectorSinkComplex::unlimited();
        sink.process(&vec![Complex64::new(1.0, 0.0); 5]);
        let data = sink.take_data();
        assert_eq!(data.len(), 5);
        assert!(sink.is_empty());
    }

    #[test]
    fn test_real_sink() {
        let mut sink = VectorSinkReal::new(100);
        sink.process(&[1.0, 2.0, 3.0]);
        assert_eq!(sink.len(), 3);
        assert_eq!(sink.data(), &[1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_bits_sink() {
        let mut sink = VectorSinkBits::new(100);
        sink.process(&[true, false, true, true]);
        assert_eq!(sink.len(), 4);
    }

    #[test]
    fn test_bits_to_bytes() {
        let mut sink = VectorSinkBits::unlimited();
        // 0xA5 = 10100101
        sink.process(&[true, false, true, false, false, true, false, true]);
        assert_eq!(sink.to_bytes(), vec![0xA5]);
    }

    #[test]
    fn test_clear() {
        let mut sink = VectorSinkReal::unlimited();
        sink.process(&[1.0, 2.0]);
        sink.clear();
        assert!(sink.is_empty());
        assert_eq!(sink.total_received(), 0);
    }

    #[test]
    fn test_incremental_fill() {
        let mut sink = VectorSinkReal::new(10);
        sink.process(&[1.0, 2.0, 3.0]);
        sink.process(&[4.0, 5.0, 6.0]);
        sink.process(&[7.0, 8.0, 9.0, 10.0, 11.0]); // Exceeds capacity
        assert_eq!(sink.len(), 10);
        assert_eq!(sink.total_received(), 11);
    }
}
