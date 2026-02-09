//! Correlate and Sync
//!
//! Cross-correlation-based frame synchronizer for detecting known preambles
//! or sync words in a complex signal stream. Outputs detected correlation
//! peaks with timing information.
//!
//! ## Algorithm
//!
//! 1. Sliding cross-correlation with reference sequence
//! 2. Normalize by input power (CFAR-like detection)
//! 3. Peak detection with threshold and minimum spacing
//! 4. Output peak timing and correlation value
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::correlate_sync::CorrelateSync;
//! use num_complex::Complex64;
//!
//! // Create a sync detector for a known preamble
//! let preamble = vec![
//!     Complex64::new(1.0, 0.0),
//!     Complex64::new(-1.0, 0.0),
//!     Complex64::new(1.0, 0.0),
//!     Complex64::new(-1.0, 0.0),
//! ];
//! let mut sync = CorrelateSync::new(&preamble, 0.7);
//!
//! // Process input stream
//! let input = vec![Complex64::new(0.0, 0.0); 100];
//! let detections = sync.process(&input);
//! ```

use num_complex::Complex64;

/// A detected synchronization event.
#[derive(Debug, Clone)]
pub struct SyncDetection {
    /// Sample index where peak was detected
    pub index: usize,
    /// Normalized correlation magnitude (0..1)
    pub correlation: f64,
    /// Phase at detection point (radians)
    pub phase: f64,
}

/// Cross-correlation frame synchronizer.
#[derive(Debug, Clone)]
pub struct CorrelateSync {
    /// Reference sequence (conjugated for correlation)
    reference: Vec<Complex64>,
    /// Detection threshold (normalized, 0..1)
    threshold: f64,
    /// Minimum spacing between detections (samples)
    min_spacing: usize,
    /// Sliding window buffer
    buffer: Vec<Complex64>,
    /// Running power estimate for normalization
    power_buffer: Vec<f64>,
    /// Total samples processed
    sample_count: usize,
    /// Last detection index (for spacing enforcement)
    last_detection: Option<usize>,
    /// Total detections
    total_detections: usize,
}

impl CorrelateSync {
    /// Create a new correlator with a reference sequence and threshold.
    ///
    /// - `reference`: Known preamble/sync word (will be conjugated internally)
    /// - `threshold`: Detection threshold for normalized correlation (0..1)
    pub fn new(reference: &[Complex64], threshold: f64) -> Self {
        let ref_len = reference.len();
        assert!(!reference.is_empty(), "Reference sequence cannot be empty");

        // Pre-conjugate reference for cross-correlation
        let reference_conj: Vec<Complex64> = reference.iter().map(|s| s.conj()).collect();

        // Normalize reference energy
        let energy: f64 = reference.iter().map(|s| s.norm_sqr()).sum();
        let scale = if energy > 0.0 {
            1.0 / energy.sqrt()
        } else {
            1.0
        };
        let reference_norm: Vec<Complex64> =
            reference_conj.iter().map(|&s| s * scale).collect();

        Self {
            reference: reference_norm,
            threshold,
            min_spacing: ref_len,
            buffer: Vec::with_capacity(ref_len),
            power_buffer: Vec::with_capacity(ref_len),
            sample_count: 0,
            last_detection: None,
            total_detections: 0,
        }
    }

    /// Set minimum spacing between detections.
    pub fn set_min_spacing(&mut self, spacing: usize) {
        self.min_spacing = spacing;
    }

    /// Process a block of samples, returning detected sync events.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<SyncDetection> {
        let ref_len = self.reference.len();
        let mut detections = Vec::new();

        for &sample in input {
            // Add to sliding buffer
            self.buffer.push(sample);
            self.power_buffer.push(sample.norm_sqr());

            // Keep buffer size at ref_len
            if self.buffer.len() > ref_len {
                self.buffer.remove(0);
                self.power_buffer.remove(0);
            }

            self.sample_count += 1;

            // Need full buffer to correlate
            if self.buffer.len() < ref_len {
                continue;
            }

            // Compute cross-correlation
            let mut corr = Complex64::new(0.0, 0.0);
            for (i, &ref_sample) in self.reference.iter().enumerate() {
                corr += self.buffer[i] * ref_sample;
            }

            // Compute input power for normalization
            let input_power: f64 = self.power_buffer.iter().sum();
            let input_rms = if input_power > 0.0 {
                input_power.sqrt()
            } else {
                1.0
            };

            let normalized_corr = corr.norm() / input_rms;

            // Check threshold
            if normalized_corr >= self.threshold {
                // Check spacing
                let current_idx = self.sample_count;
                let spaced = match self.last_detection {
                    Some(last) => current_idx - last >= self.min_spacing,
                    None => true,
                };

                if spaced {
                    detections.push(SyncDetection {
                        index: current_idx - ref_len,
                        correlation: normalized_corr,
                        phase: corr.arg(),
                    });
                    self.last_detection = Some(current_idx);
                    self.total_detections += 1;
                }
            }
        }

        detections
    }

    /// Process and return correlation values for the entire input (for visualization).
    pub fn correlate_all(&self, input: &[Complex64]) -> Vec<f64> {
        let ref_len = self.reference.len();
        if input.len() < ref_len {
            return vec![0.0; input.len()];
        }

        let mut output = vec![0.0; input.len()];

        for i in 0..=(input.len() - ref_len) {
            let mut corr = Complex64::new(0.0, 0.0);
            let mut power = 0.0;
            for (j, &ref_sample) in self.reference.iter().enumerate() {
                corr += input[i + j] * ref_sample;
                power += input[i + j].norm_sqr();
            }
            let rms = if power > 0.0 { power.sqrt() } else { 1.0 };
            output[i + ref_len - 1] = corr.norm() / rms;
        }

        output
    }

    /// Get the reference length.
    pub fn reference_len(&self) -> usize {
        self.reference.len()
    }

    /// Get total detections.
    pub fn total_detections(&self) -> usize {
        self.total_detections
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.power_buffer.clear();
        self.sample_count = 0;
        self.last_detection = None;
        self.total_detections = 0;
    }
}

/// Bit-level correlator for binary access codes.
///
/// Works on hard-decision bits rather than complex samples.
/// Useful after binary slicer for protocol sync word detection.
#[derive(Debug, Clone)]
pub struct BitCorrelator {
    /// Reference bit pattern
    pattern: Vec<bool>,
    /// Detection threshold (number of matching bits)
    threshold: usize,
    /// Shift register
    shift_reg: Vec<bool>,
    /// Total samples processed
    sample_count: usize,
    /// Total detections
    total_detections: usize,
}

impl BitCorrelator {
    /// Create a new bit correlator.
    ///
    /// - `pattern`: Known sync word as bits
    /// - `max_errors`: Maximum allowed bit errors (Hamming distance)
    pub fn new(pattern: &[bool], max_errors: usize) -> Self {
        let threshold = pattern.len().saturating_sub(max_errors);
        Self {
            pattern: pattern.to_vec(),
            threshold,
            shift_reg: vec![false; pattern.len()],
            sample_count: 0,
            total_detections: 0,
        }
    }

    /// Process a stream of bits, returning indices where pattern is found.
    pub fn process(&mut self, bits: &[bool]) -> Vec<usize> {
        let mut detections = Vec::new();
        let plen = self.pattern.len();

        for &bit in bits {
            // Shift in new bit
            self.shift_reg.push(bit);
            if self.shift_reg.len() > plen {
                self.shift_reg.remove(0);
            }
            self.sample_count += 1;

            if self.shift_reg.len() < plen {
                continue;
            }

            // Count matching bits
            let matches: usize = self
                .shift_reg
                .iter()
                .zip(self.pattern.iter())
                .filter(|(&a, &b)| a == b)
                .count();

            if matches >= self.threshold {
                detections.push(self.sample_count - plen);
                self.total_detections += 1;
            }
        }

        detections
    }

    /// Get total detections.
    pub fn total_detections(&self) -> usize {
        self.total_detections
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.shift_reg.fill(false);
        self.sample_count = 0;
        self.total_detections = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_perfect_detection() {
        let preamble = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(-1.0, 0.0),
            Complex64::new(1.0, 0.0),
            Complex64::new(-1.0, 0.0),
        ];
        let mut sync = CorrelateSync::new(&preamble, 0.8);

        // Build input: noise, then preamble, then noise
        let mut input = vec![Complex64::new(0.1, 0.0); 20];
        input.extend_from_slice(&preamble);
        input.extend(vec![Complex64::new(0.1, 0.0); 20]);

        let detections = sync.process(&input);
        assert!(
            !detections.is_empty(),
            "Should detect the preamble"
        );
        // Detection should be near index 20
        assert!(
            detections[0].index >= 18 && detections[0].index <= 22,
            "Detection at {} (expected ~20)",
            detections[0].index
        );
    }

    #[test]
    fn test_no_false_alarms() {
        let preamble = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(-1.0, 0.0),
            Complex64::new(1.0, 0.0),
            Complex64::new(-1.0, 0.0),
        ];
        let mut sync = CorrelateSync::new(&preamble, 0.95);

        // Pure noise (low amplitude)
        let input = vec![Complex64::new(0.01, 0.01); 200];
        let detections = sync.process(&input);
        assert!(
            detections.is_empty(),
            "Should not detect in noise"
        );
    }

    #[test]
    fn test_multiple_detections() {
        let preamble = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(-1.0, 0.0),
        ];
        let mut sync = CorrelateSync::new(&preamble, 0.7);
        sync.set_min_spacing(20);

        let mut input = vec![Complex64::new(0.01, 0.0); 10];
        input.extend_from_slice(&preamble);
        input.extend(vec![Complex64::new(0.01, 0.0); 30]);
        input.extend_from_slice(&preamble);
        input.extend(vec![Complex64::new(0.01, 0.0); 10]);

        let detections = sync.process(&input);
        assert!(
            detections.len() >= 2,
            "Should detect two preambles, got {}",
            detections.len()
        );
    }

    #[test]
    fn test_correlate_all() {
        let preamble = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(-1.0, 0.0),
            Complex64::new(1.0, 0.0),
        ];
        let sync = CorrelateSync::new(&preamble, 0.8);

        let mut input = vec![Complex64::new(0.0, 0.0); 10];
        input.extend_from_slice(&preamble);
        input.extend(vec![Complex64::new(0.0, 0.0); 10]);

        let corr = sync.correlate_all(&input);
        assert_eq!(corr.len(), input.len());

        // Peak should be at the preamble location
        let max_idx = corr
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap()
            .0;
        assert!(
            max_idx >= 11 && max_idx <= 13,
            "Peak at {} (expected ~12)",
            max_idx
        );
    }

    #[test]
    fn test_bit_correlator_exact() {
        let pattern = vec![true, false, true, false, true, true];
        let mut corr = BitCorrelator::new(&pattern, 0);

        let mut bits = vec![false; 10];
        bits.extend_from_slice(&pattern);
        bits.extend(vec![false; 10]);

        let detections = corr.process(&bits);
        assert_eq!(detections.len(), 1);
        assert_eq!(detections[0], 10);
    }

    #[test]
    fn test_bit_correlator_with_errors() {
        let pattern = vec![true, false, true, false, true, true];
        let mut corr = BitCorrelator::new(&pattern, 1);

        // Pattern with 1 bit error
        let mut bits = vec![false; 10];
        bits.extend_from_slice(&[true, true, true, false, true, true]); // 1 error at bit 1
        bits.extend(vec![false; 10]);

        let detections = corr.process(&bits);
        assert_eq!(detections.len(), 1);
    }

    #[test]
    fn test_bit_correlator_no_match() {
        let pattern = vec![true, false, true, false];
        let mut corr = BitCorrelator::new(&pattern, 0);

        let bits = vec![false; 50]; // All zeros
        let detections = corr.process(&bits);
        assert!(detections.is_empty());
    }

    #[test]
    fn test_reset() {
        let preamble = vec![Complex64::new(1.0, 0.0)];
        let mut sync = CorrelateSync::new(&preamble, 0.5);
        sync.process(&vec![Complex64::new(1.0, 0.0); 10]);
        assert!(sync.total_detections() > 0);
        sync.reset();
        assert_eq!(sync.total_detections(), 0);
        assert_eq!(sync.sample_count, 0);
    }

    #[test]
    fn test_phase_detection() {
        // A 90° rotated preamble should still be detected
        let preamble = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(-1.0, 0.0),
        ];
        let mut sync = CorrelateSync::new(&preamble, 0.7);

        // Rotate preamble by 90°
        let rotated: Vec<Complex64> = preamble
            .iter()
            .map(|s| s * Complex64::new(0.0, 1.0))
            .collect();

        let mut input = vec![Complex64::new(0.01, 0.0); 20];
        input.extend_from_slice(&rotated);
        input.extend(vec![Complex64::new(0.01, 0.0); 20]);

        let detections = sync.process(&input);
        assert!(
            !detections.is_empty(),
            "Should detect rotated preamble"
        );
        // Phase should be near π/2
        let phase = detections[0].phase;
        assert!(
            (phase.abs() - std::f64::consts::FRAC_PI_2).abs() < 0.5,
            "Phase {} not near π/2",
            phase
        );
    }
}
