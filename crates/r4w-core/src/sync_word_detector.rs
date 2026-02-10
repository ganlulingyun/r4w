//! Sync Word / Preamble Detector
//!
//! Provides bit-level and sample-level sync word detection for frame
//! synchronization in digital communication systems. The bit-level
//! [`SyncWordDetector`] uses a shift register and Hamming distance to find
//! known bit patterns in a binary stream, tolerating a configurable number
//! of bit errors. The sample-level [`CorrelationSyncDetector`] uses
//! cross-correlation to locate a known waveform pattern in a real-valued
//! sample stream, detecting peaks that exceed a configurable threshold.
//!
//! [`SyncWordGenerator`] offers factory methods for common sync word patterns
//! used in various communication standards (IEEE 802.15.4, HDLC, Barker
//! codes, etc.).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::sync_word_detector::{SyncWordDetector, SyncWordGenerator};
//!
//! // Build a detector for the HDLC flag pattern (0x7E = 01111110)
//! let pattern = SyncWordGenerator::hdlc_flag();
//! let mut detector = SyncWordDetector::new(pattern.clone(), 1);
//!
//! // Feed some noise, then the pattern with one bit error
//! let mut stream = vec![false; 10];
//! stream.extend_from_slice(&[false, true, true, true, true, true, true, false]); // exact
//! let hits = detector.feed_bits(&stream);
//! assert_eq!(hits.len(), 1);
//! assert_eq!(hits[0], 17); // detection at last bit of pattern
//! ```

/// Compute the Hamming distance (number of differing positions) between two
/// boolean slices. Both slices must have the same length.
///
/// # Panics
///
/// Panics if `a` and `b` have different lengths.
pub fn hamming_distance(a: &[bool], b: &[bool]) -> usize {
    assert_eq!(a.len(), b.len(), "Hamming distance requires equal-length slices");
    a.iter().zip(b.iter()).filter(|(x, y)| x != y).count()
}

// ---------------------------------------------------------------------------
// Bit-level sync word detector
// ---------------------------------------------------------------------------

/// Bit-level sync word detector using a shift register and Hamming distance.
///
/// Bits are shifted in one at a time (or in bulk via [`feed_bits`]). After
/// each shift the register contents are compared against the stored pattern.
/// A detection is signalled when the Hamming distance between the register
/// and the pattern is less than or equal to `threshold`.
///
/// [`feed_bits`]: SyncWordDetector::feed_bits
#[derive(Debug, Clone)]
pub struct SyncWordDetector {
    /// The sync word bit pattern to search for.
    pattern: Vec<bool>,
    /// Maximum allowed bit errors (Hamming distance) for a detection.
    threshold: usize,
    /// Shift register holding the most recent `pattern.len()` bits.
    shift_register: Vec<bool>,
}

impl SyncWordDetector {
    /// Create a new detector.
    ///
    /// - `pattern`: The bit pattern to search for. Must not be empty.
    /// - `threshold`: Maximum Hamming distance to still count as a detection.
    ///   Use 0 for exact-match only.
    ///
    /// # Panics
    ///
    /// Panics if `pattern` is empty.
    pub fn new(pattern: Vec<bool>, threshold: usize) -> Self {
        assert!(!pattern.is_empty(), "Sync word pattern must not be empty");
        let len = pattern.len();
        Self {
            pattern,
            threshold,
            shift_register: vec![false; len],
        }
    }

    /// Shift a single bit into the detector.
    ///
    /// Returns `true` if the current register contents match the pattern
    /// within the allowed Hamming distance.
    pub fn feed_bit(&mut self, bit: bool) -> bool {
        // Shift left: drop oldest bit, push new bit at the end.
        let len = self.shift_register.len();
        for i in 0..len - 1 {
            self.shift_register[i] = self.shift_register[i + 1];
        }
        self.shift_register[len - 1] = bit;

        hamming_distance(&self.shift_register, &self.pattern) <= self.threshold
    }

    /// Feed a slice of bits and return the positions (indices into `bits`)
    /// where a sync word detection occurred.
    ///
    /// A position `p` means the pattern was detected after bit `bits[p]` was
    /// shifted in.
    pub fn feed_bits(&mut self, bits: &[bool]) -> Vec<usize> {
        let mut detections = Vec::new();
        for (i, &bit) in bits.iter().enumerate() {
            if self.feed_bit(bit) {
                detections.push(i);
            }
        }
        detections
    }

    /// Clear the shift register to all `false` values, resetting the
    /// detector state.
    pub fn reset(&mut self) {
        for b in self.shift_register.iter_mut() {
            *b = false;
        }
    }
}

// ---------------------------------------------------------------------------
// Sample-level correlation sync detector
// ---------------------------------------------------------------------------

/// Sample-level sync word detector using cross-correlation.
///
/// Slides the stored reference pattern across the input buffer and computes
/// the normalized correlation at each position. Positions where the
/// correlation exceeds `threshold` are reported as detections.
#[derive(Debug, Clone)]
pub struct CorrelationSyncDetector {
    /// Reference pattern samples.
    pattern: Vec<f64>,
    /// Detection threshold (normalized correlation, typically 0.0..1.0).
    threshold: f64,
    /// Energy of the reference pattern (pre-computed).
    pattern_energy: f64,
}

impl CorrelationSyncDetector {
    /// Create a new correlation-based sync detector.
    ///
    /// - `pattern`: Reference waveform pattern. Must not be empty.
    /// - `threshold`: Minimum normalized correlation magnitude to trigger a
    ///   detection. Values near 1.0 require a near-perfect match; lower
    ///   values allow more noise.
    ///
    /// # Panics
    ///
    /// Panics if `pattern` is empty.
    pub fn new(pattern: Vec<f64>, threshold: f64) -> Self {
        assert!(!pattern.is_empty(), "Correlation pattern must not be empty");
        let pattern_energy: f64 = pattern.iter().map(|x| x * x).sum();
        Self {
            pattern,
            threshold,
            pattern_energy,
        }
    }

    /// Process an input buffer and return sample indices where the normalized
    /// correlation with the reference pattern exceeds the threshold.
    ///
    /// The returned indices correspond to the *start* of the matching
    /// segment in `input`. Only positions where the full pattern fits inside
    /// the input buffer are considered.
    ///
    /// Returns an empty vector if `input` is shorter than the pattern.
    pub fn process(&mut self, input: &[f64]) -> Vec<usize> {
        let pat_len = self.pattern.len();
        if input.len() < pat_len {
            return Vec::new();
        }

        let mut detections = Vec::new();

        for start in 0..=(input.len() - pat_len) {
            let segment = &input[start..start + pat_len];

            // Cross-correlation (dot product).
            let cross: f64 = segment
                .iter()
                .zip(self.pattern.iter())
                .map(|(s, p)| s * p)
                .sum();

            // Energy of the input segment.
            let seg_energy: f64 = segment.iter().map(|x| x * x).sum();

            // Normalized correlation: cross / sqrt(E_pattern * E_segment).
            let denom = (self.pattern_energy * seg_energy).sqrt();
            if denom == 0.0 {
                continue;
            }
            let normalized = cross / denom;

            if normalized >= self.threshold {
                detections.push(start);
            }
        }

        detections
    }
}

// ---------------------------------------------------------------------------
// Sync word generator
// ---------------------------------------------------------------------------

/// Factory for common sync word / preamble bit patterns.
pub struct SyncWordGenerator;

impl SyncWordGenerator {
    /// IEEE 802.15.4 SHR preamble: 32 zero bits.
    ///
    /// The 802.15.4 preamble consists of four repetitions of the zero symbol
    /// (all-zeros byte), totalling 32 bits.
    pub fn ieee_802_15_4() -> Vec<bool> {
        vec![false; 32]
    }

    /// HDLC flag byte `0x7E` (binary `01111110`).
    ///
    /// Used as a frame delimiter in HDLC, AX.25, PPP, and related protocols.
    pub fn hdlc_flag() -> Vec<bool> {
        vec![false, true, true, true, true, true, true, false]
    }

    /// Barker-13 code as a boolean sequence.
    ///
    /// The 13-chip Barker code `[+1, +1, +1, +1, +1, -1, -1, +1, +1, -1, +1,
    /// -1, +1]` is encoded with `true` = +1 and `false` = -1.
    pub fn custom_barker13() -> Vec<bool> {
        vec![
            true, true, true, true, true, false, false, true, true, false, true, false, true,
        ]
    }

    /// Alternating bit pattern `10101010...` of the given `length`.
    ///
    /// The first bit is `true`, the second is `false`, and so on.
    pub fn alternating(length: usize) -> Vec<bool> {
        (0..length).map(|i| i % 2 == 0).collect()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_exact_match() {
        let pattern = vec![true, false, true, true];
        let mut det = SyncWordDetector::new(pattern.clone(), 0);

        // Feed the exact pattern; the last bit should trigger detection.
        for (i, &b) in pattern.iter().enumerate() {
            let result = det.feed_bit(b);
            if i < pattern.len() - 1 {
                assert!(!result, "Should not detect before full pattern is in");
            } else {
                assert!(result, "Should detect on last bit of exact pattern");
            }
        }
    }

    #[test]
    fn test_with_errors() {
        let pattern = vec![true, true, false, false, true, true, false, true];
        let mut det = SyncWordDetector::new(pattern.clone(), 2);

        // Pattern with 2 errors (bits 0 and 7 flipped).
        let noisy = vec![false, true, false, false, true, true, false, false];
        for &b in &noisy {
            det.feed_bit(b);
        }
        // Should still detect because hamming distance is exactly 2.
        assert!(det.feed_bit(noisy[0]) || {
            // Re-check by feeding the full noisy pattern from a fresh detector.
            let mut d2 = SyncWordDetector::new(pattern, 2);
            let last = noisy.last().copied().unwrap();
            for &b in &noisy[..noisy.len() - 1] {
                d2.feed_bit(b);
            }
            d2.feed_bit(last)
        });
    }

    #[test]
    fn test_threshold_reject() {
        let pattern = vec![true, true, true, true];
        let mut det = SyncWordDetector::new(pattern, 0);

        // Feed 3 trues and 1 false -- hamming distance 1, threshold 0.
        let bits = vec![true, true, true, false];
        for &b in &bits {
            det.feed_bit(b);
        }
        // Register should be [true, true, true, false] vs [true, true, true, true] -> dist 1.
        // With threshold 0 this must NOT detect.
        // (The last feed_bit already returned; test via feed_bits on a fresh detector.)
        let mut d2 = SyncWordDetector::new(vec![true, true, true, true], 0);
        let hits = d2.feed_bits(&bits);
        assert!(hits.is_empty(), "Should reject when hamming distance exceeds threshold");
    }

    #[test]
    fn test_feed_bits_positions() {
        // Pattern: 1010
        let pattern = vec![true, false, true, false];
        let mut det = SyncWordDetector::new(pattern, 0);

        // Stream: 8 zeros, then the pattern, then 4 zeros.
        let mut stream = vec![false; 8];
        stream.extend_from_slice(&[true, false, true, false]);
        stream.extend_from_slice(&[false; 4]);

        let hits = det.feed_bits(&stream);
        // Detection at index 11 (0-based), the last bit of the pattern.
        assert_eq!(hits, vec![11]);
    }

    #[test]
    fn test_hamming_distance() {
        assert_eq!(hamming_distance(&[], &[]), 0);
        assert_eq!(
            hamming_distance(&[true, false, true], &[true, false, true]),
            0
        );
        assert_eq!(
            hamming_distance(&[true, false, true], &[false, false, false]),
            2
        );
        assert_eq!(
            hamming_distance(&[true, true, true, true], &[false, false, false, false]),
            4
        );
    }

    #[test]
    fn test_correlation_detect() {
        // Pattern: short bipolar pulse [1, -1, 1, -1].
        let pattern = vec![1.0, -1.0, 1.0, -1.0];
        let mut det = CorrelationSyncDetector::new(pattern.clone(), 0.95);

        // Build input: silence, then the pattern, then silence.
        let mut input = vec![0.0; 10];
        input.extend_from_slice(&pattern);
        input.extend(vec![0.0; 10]);

        let hits = det.process(&input);
        assert!(
            hits.contains(&10),
            "Should detect pattern starting at index 10, got {:?}",
            hits
        );
    }

    #[test]
    fn test_reset() {
        let pattern = vec![true, true, true, true];
        let mut det = SyncWordDetector::new(pattern, 0);

        // Feed 3 trues.
        det.feed_bit(true);
        det.feed_bit(true);
        det.feed_bit(true);

        // Reset should clear progress.
        det.reset();

        // After reset, feeding one more true should NOT detect (register was cleared).
        assert!(!det.feed_bit(true), "After reset, partial pattern should not detect");
    }

    #[test]
    fn test_sync_generators() {
        let ieee = SyncWordGenerator::ieee_802_15_4();
        assert_eq!(ieee.len(), 32);
        assert!(ieee.iter().all(|&b| !b), "802.15.4 preamble should be all zeros");

        let hdlc = SyncWordGenerator::hdlc_flag();
        assert_eq!(hdlc, vec![false, true, true, true, true, true, true, false]);

        let barker = SyncWordGenerator::custom_barker13();
        assert_eq!(barker.len(), 13);
        // Known Barker-13: +++++--++-+-+
        assert_eq!(
            barker,
            vec![true, true, true, true, true, false, false, true, true, false, true, false, true]
        );

        let alt = SyncWordGenerator::alternating(6);
        assert_eq!(alt, vec![true, false, true, false, true, false]);

        let alt0 = SyncWordGenerator::alternating(0);
        assert!(alt0.is_empty());
    }

    #[test]
    fn test_multiple_syncs() {
        // Detect the same pattern appearing twice in a stream.
        let pattern = vec![true, false, true];
        let mut det = SyncWordDetector::new(pattern, 0);

        // Two copies separated by some zeros.
        let stream = vec![
            true, false, true, // match at index 2
            false, false, false, // gap
            true, false, true, // match at index 8
        ];

        let hits = det.feed_bits(&stream);
        assert_eq!(hits.len(), 2, "Should detect exactly two matches");
        assert_eq!(hits[0], 2);
        assert_eq!(hits[1], 8);
    }

    #[test]
    fn test_empty_input() {
        // Bit-level: empty bits.
        let mut det = SyncWordDetector::new(vec![true, false], 0);
        let hits = det.feed_bits(&[]);
        assert!(hits.is_empty());

        // Correlation: empty input.
        let mut cdet = CorrelationSyncDetector::new(vec![1.0, -1.0], 0.9);
        let chits = cdet.process(&[]);
        assert!(chits.is_empty());

        // Correlation: input shorter than pattern.
        let chits2 = cdet.process(&[1.0]);
        assert!(chits2.is_empty());
    }
}
