//! Plateau Detector
//!
//! Detects flat regions (plateaus) in a signal and outputs the midpoint
//! sample index. This is used in OFDM synchronization with the Schmidl-Cox
//! algorithm, which produces a plateau in the timing metric rather than a
//! sharp peak.
//!
//! ## Algorithm
//!
//! 1. Compute running average of the input magnitude
//! 2. When the average exceeds a threshold, start tracking a plateau
//! 3. When it drops below the threshold, emit the midpoint of the plateau
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::plateau_detector::PlateauDetector;
//!
//! let mut pd = PlateauDetector::new(0.8, 4);
//! let input = vec![
//!     0.1, 0.2, 0.9, 0.95, 0.92, 0.93, 0.91, 0.3, 0.1,
//! ];
//! let detections = pd.detect(&input);
//! // Should detect a plateau around samples 2-6
//! assert!(!detections.is_empty());
//! ```

/// Plateau detector for OFDM timing synchronization.
#[derive(Debug, Clone)]
pub struct PlateauDetector {
    /// Detection threshold (0.0 to 1.0)
    threshold: f64,
    /// Minimum plateau width in samples
    min_width: usize,
    /// Internal state
    in_plateau: bool,
    plateau_start: usize,
    sample_count: usize,
}

/// Result of plateau detection.
#[derive(Debug, Clone)]
pub struct PlateauResult {
    /// Midpoint of the detected plateau (sample index)
    pub midpoint: usize,
    /// Start of the plateau
    pub start: usize,
    /// End of the plateau
    pub end: usize,
    /// Width of the plateau in samples
    pub width: usize,
    /// Average value within the plateau
    pub avg_value: f64,
}

impl PlateauDetector {
    /// Create a new plateau detector.
    ///
    /// - `threshold`: Value above which a plateau is detected (0.0..1.0)
    /// - `min_width`: Minimum number of consecutive samples above threshold
    pub fn new(threshold: f64, min_width: usize) -> Self {
        Self {
            threshold,
            min_width: min_width.max(1),
            in_plateau: false,
            plateau_start: 0,
            sample_count: 0,
        }
    }

    /// Detect plateaus in the input signal.
    /// Returns a list of detected plateaus with their midpoints.
    pub fn detect(&mut self, input: &[f64]) -> Vec<PlateauResult> {
        let mut results = Vec::new();

        for (i, &value) in input.iter().enumerate() {
            let idx = self.sample_count + i;

            if value >= self.threshold {
                if !self.in_plateau {
                    self.in_plateau = true;
                    self.plateau_start = idx;
                }
            } else {
                if self.in_plateau {
                    let width = idx - self.plateau_start;
                    if width >= self.min_width {
                        // Compute average value within plateau
                        let start_local = self.plateau_start.saturating_sub(self.sample_count);
                        let end_local = i;
                        let avg = if end_local > start_local {
                            input[start_local..end_local].iter().sum::<f64>()
                                / (end_local - start_local) as f64
                        } else {
                            value
                        };
                        results.push(PlateauResult {
                            midpoint: self.plateau_start + width / 2,
                            start: self.plateau_start,
                            end: idx - 1,
                            width,
                            avg_value: avg,
                        });
                    }
                    self.in_plateau = false;
                }
            }
        }

        self.sample_count += input.len();
        results
    }

    /// Detect plateaus and return only midpoint indices.
    pub fn detect_midpoints(&mut self, input: &[f64]) -> Vec<usize> {
        self.detect(input).iter().map(|r| r.midpoint).collect()
    }

    /// Process input and return a binary output (1.0 at plateau midpoints, 0.0 elsewhere).
    pub fn process_block(&mut self, input: &[f64]) -> Vec<f64> {
        let initial_count = self.sample_count;
        let results = self.detect(input);
        let mut output = vec![0.0; input.len()];
        for r in &results {
            let local_idx = r.midpoint.saturating_sub(initial_count);
            if local_idx < output.len() {
                output[local_idx] = 1.0;
            }
        }
        output
    }

    /// Get the detection threshold.
    pub fn threshold(&self) -> f64 {
        self.threshold
    }

    /// Set a new threshold.
    pub fn set_threshold(&mut self, threshold: f64) {
        self.threshold = threshold;
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.in_plateau = false;
        self.plateau_start = 0;
        self.sample_count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_plateau() {
        let mut pd = PlateauDetector::new(0.8, 3);
        let input = vec![0.1, 0.2, 0.9, 0.95, 0.92, 0.93, 0.91, 0.3, 0.1];
        let results = pd.detect(&input);
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].width, 5);
        assert_eq!(results[0].start, 2);
        assert_eq!(results[0].end, 6);
        assert_eq!(results[0].midpoint, 4); // 2 + 5/2 = 4
    }

    #[test]
    fn test_no_plateau() {
        let mut pd = PlateauDetector::new(0.8, 3);
        let input = vec![0.1, 0.2, 0.3, 0.2, 0.1];
        let results = pd.detect(&input);
        assert!(results.is_empty());
    }

    #[test]
    fn test_too_narrow() {
        let mut pd = PlateauDetector::new(0.8, 5);
        // Only 3 samples above threshold, but need 5
        let input = vec![0.1, 0.9, 0.9, 0.9, 0.1];
        let results = pd.detect(&input);
        assert!(results.is_empty());
    }

    #[test]
    fn test_multiple_plateaus() {
        let mut pd = PlateauDetector::new(0.5, 2);
        let input = vec![
            0.1, 0.8, 0.9, 0.1, // plateau 1: samples 1-2
            0.1, 0.7, 0.8, 0.7, 0.1, // plateau 2: samples 5-7
        ];
        let results = pd.detect(&input);
        assert_eq!(results.len(), 2);
    }

    #[test]
    fn test_detect_midpoints() {
        let mut pd = PlateauDetector::new(0.5, 2);
        let input = vec![0.1, 0.8, 0.9, 0.8, 0.1];
        let midpoints = pd.detect_midpoints(&input);
        assert_eq!(midpoints.len(), 1);
        assert_eq!(midpoints[0], 2); // midpoint of [1..3]
    }

    #[test]
    fn test_process_block_binary() {
        let mut pd = PlateauDetector::new(0.5, 2);
        let input = vec![0.1, 0.8, 0.9, 0.8, 0.1];
        let output = pd.process_block(&input);
        assert_eq!(output.len(), 5);
        // Midpoint at index 2
        assert!((output[2] - 1.0).abs() < 1e-10);
        // Others should be 0
        assert!((output[0]).abs() < 1e-10);
        assert!((output[4]).abs() < 1e-10);
    }

    #[test]
    fn test_avg_value() {
        let mut pd = PlateauDetector::new(0.5, 2);
        let input = vec![0.1, 0.8, 0.9, 0.1];
        let results = pd.detect(&input);
        assert_eq!(results.len(), 1);
        assert!((results[0].avg_value - 0.85).abs() < 1e-10); // avg(0.8, 0.9)
    }

    #[test]
    fn test_threshold_exact() {
        let mut pd = PlateauDetector::new(0.5, 2);
        // Exactly at threshold should be included
        let input = vec![0.1, 0.5, 0.5, 0.1];
        let results = pd.detect(&input);
        assert_eq!(results.len(), 1);
    }

    #[test]
    fn test_reset() {
        let mut pd = PlateauDetector::new(0.5, 2);
        pd.detect(&[0.8, 0.9, 0.1]);
        pd.reset();
        assert!(!pd.in_plateau);
        assert_eq!(pd.sample_count, 0);
    }

    #[test]
    fn test_min_width_one() {
        let mut pd = PlateauDetector::new(0.5, 1);
        let input = vec![0.1, 0.8, 0.1, 0.9, 0.1];
        let results = pd.detect(&input);
        assert_eq!(results.len(), 2); // Each single-sample plateau counts
    }
}
