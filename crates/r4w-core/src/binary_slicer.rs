//! Binary Slicer — Soft to Hard Bit Decision
//!
//! Converts soft floating-point symbols to hard binary decisions (0 or 1)
//! using a configurable threshold. Essential for transitioning from analog
//! demodulation to digital decoding in packet radio, APRS, AX.25, etc.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::binary_slicer::BinarySlicer;
//!
//! let slicer = BinarySlicer::new(0.0);
//! let soft = vec![-1.2, 0.8, -0.3, 1.5, -0.9, 0.1];
//! let bits = slicer.slice(&soft);
//! assert_eq!(bits, vec![false, true, false, true, false, true]);
//! ```

/// Binary slicer for soft-to-hard bit decision.
#[derive(Debug, Clone)]
pub struct BinarySlicer {
    /// Decision threshold
    threshold: f64,
}

impl BinarySlicer {
    /// Create a binary slicer with the given threshold.
    /// Samples >= threshold → true (1), samples < threshold → false (0).
    pub fn new(threshold: f64) -> Self {
        Self { threshold }
    }

    /// Create with default threshold of 0.0 (standard for bipolar signals).
    pub fn bipolar() -> Self {
        Self::new(0.0)
    }

    /// Create with threshold of 0.5 (standard for unipolar signals).
    pub fn unipolar() -> Self {
        Self::new(0.5)
    }

    /// Slice a single sample.
    pub fn process(&self, sample: f64) -> bool {
        sample >= self.threshold
    }

    /// Slice a block of samples to bits.
    pub fn slice(&self, input: &[f64]) -> Vec<bool> {
        input.iter().map(|&s| s >= self.threshold).collect()
    }

    /// Slice to bytes (0 or 1).
    pub fn slice_to_bytes(&self, input: &[f64]) -> Vec<u8> {
        input.iter().map(|&s| if s >= self.threshold { 1u8 } else { 0u8 }).collect()
    }

    /// Get the threshold.
    pub fn threshold(&self) -> f64 {
        self.threshold
    }

    /// Set a new threshold.
    pub fn set_threshold(&mut self, threshold: f64) {
        self.threshold = threshold;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bipolar_basic() {
        let slicer = BinarySlicer::bipolar();
        assert!(!slicer.process(-1.0));
        assert!(slicer.process(1.0));
        assert!(slicer.process(0.0)); // >= threshold
    }

    #[test]
    fn test_unipolar_basic() {
        let slicer = BinarySlicer::unipolar();
        assert!(!slicer.process(0.3));
        assert!(slicer.process(0.7));
        assert!(slicer.process(0.5)); // >= threshold
    }

    #[test]
    fn test_slice_block() {
        let slicer = BinarySlicer::bipolar();
        let input = vec![-1.0, 1.0, -0.5, 0.5, -0.1, 0.1];
        let bits = slicer.slice(&input);
        assert_eq!(bits, vec![false, true, false, true, false, true]);
    }

    #[test]
    fn test_slice_to_bytes() {
        let slicer = BinarySlicer::bipolar();
        let input = vec![-1.0, 1.0, -0.5, 0.5];
        let bytes = slicer.slice_to_bytes(&input);
        assert_eq!(bytes, vec![0u8, 1, 0, 1]);
    }

    #[test]
    fn test_noisy_signal() {
        let slicer = BinarySlicer::bipolar();
        // Noisy bipolar signal: should still make correct decisions
        let input = vec![-0.8, 0.9, -1.2, 0.7, -0.3, 1.5];
        let bits = slicer.slice(&input);
        assert_eq!(bits, vec![false, true, false, true, false, true]);
    }

    #[test]
    fn test_custom_threshold() {
        let slicer = BinarySlicer::new(0.25);
        assert!(!slicer.process(0.2));
        assert!(slicer.process(0.3));
        assert!(slicer.process(0.25));
    }

    #[test]
    fn test_set_threshold() {
        let mut slicer = BinarySlicer::bipolar();
        assert!((slicer.threshold() - 0.0).abs() < 1e-10);
        slicer.set_threshold(0.5);
        assert!((slicer.threshold() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_empty_input() {
        let slicer = BinarySlicer::bipolar();
        assert!(slicer.slice(&[]).is_empty());
    }
}
