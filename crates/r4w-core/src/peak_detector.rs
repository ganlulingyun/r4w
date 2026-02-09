//! Peak Detector
//!
//! Detects peaks in real-valued or magnitude signals. Useful for timing recovery,
//! frequency lock, sync word detection, and spectral peak finding.
//!
//! ## Variants
//!
//! - **PeakDetector**: Sliding-window peak detection with threshold and minimum spacing
//! - **IntegrateAndDump**: Accumulate N samples then output and reset (matched filter for rect pulses)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::peak_detector::PeakDetector;
//!
//! let mut pd = PeakDetector::new(0.5, 10); // threshold=0.5, min spacing=10
//! let signal = vec![0.1, 0.2, 0.3, 0.9, 0.3, 0.1, 0.0, 0.0, 0.0, 0.0,
//!                   0.0, 0.1, 0.2, 0.8, 0.2, 0.1];
//! let peaks = pd.process(&signal);
//! assert_eq!(peaks.len(), 1); // Only first peak (second too close if min_spacing)
//! assert_eq!(peaks[0].index, 3);
//! ```

/// Detected peak information.
#[derive(Debug, Clone, PartialEq)]
pub struct PeakInfo {
    /// Sample index of the peak
    pub index: usize,
    /// Peak magnitude
    pub magnitude: f64,
    /// Estimated fractional index (from parabolic interpolation)
    pub fractional_index: f64,
}

/// Peak detector with threshold and minimum spacing.
#[derive(Debug, Clone)]
pub struct PeakDetector {
    /// Minimum threshold for peak detection
    threshold: f64,
    /// Minimum samples between detected peaks
    min_spacing: usize,
    /// Last detected peak index (for spacing enforcement)
    last_peak_index: Option<usize>,
    /// Total samples processed
    total_samples: usize,
}

impl PeakDetector {
    /// Create a new peak detector.
    ///
    /// - `threshold`: Minimum value for a sample to be considered a peak
    /// - `min_spacing`: Minimum distance between consecutive peaks
    pub fn new(threshold: f64, min_spacing: usize) -> Self {
        Self {
            threshold,
            min_spacing,
            last_peak_index: None,
            total_samples: 0,
        }
    }

    /// Process a block of real-valued samples and return detected peaks.
    pub fn process(&mut self, input: &[f64]) -> Vec<PeakInfo> {
        let mut peaks = Vec::new();

        for i in 1..input.len().saturating_sub(1) {
            let global_idx = self.total_samples + i;

            // Check if this is a local maximum above threshold
            if input[i] > self.threshold
                && input[i] >= input[i - 1]
                && input[i] >= input[i + 1]
            {
                // Check minimum spacing
                if let Some(last) = self.last_peak_index {
                    if global_idx - last < self.min_spacing {
                        continue;
                    }
                }

                // Parabolic interpolation for fractional peak
                let frac = parabolic_interpolation(input[i - 1], input[i], input[i + 1]);

                peaks.push(PeakInfo {
                    index: global_idx,
                    magnitude: input[i],
                    fractional_index: global_idx as f64 + frac,
                });
                self.last_peak_index = Some(global_idx);
            }
        }

        self.total_samples += input.len();
        peaks
    }

    /// Get threshold.
    pub fn threshold(&self) -> f64 {
        self.threshold
    }

    /// Set threshold.
    pub fn set_threshold(&mut self, threshold: f64) {
        self.threshold = threshold;
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.last_peak_index = None;
        self.total_samples = 0;
    }
}

/// Parabolic interpolation to find fractional peak offset.
///
/// Given three consecutive samples (y_minus1, y_0, y_plus1) where y_0 is the
/// peak, returns the fractional offset from the center sample.
fn parabolic_interpolation(y_m1: f64, y_0: f64, y_p1: f64) -> f64 {
    let denom = 2.0 * (2.0 * y_0 - y_m1 - y_p1);
    if denom.abs() < 1e-30 {
        0.0
    } else {
        (y_m1 - y_p1) / denom
    }
}

/// Integrate and Dump filter.
///
/// Accumulates `length` samples, outputs the sum (or average), then resets.
/// Acts as a matched filter for rectangular (NRZ) pulses.
#[derive(Debug, Clone)]
pub struct IntegrateAndDump {
    /// Integration length (samples per symbol)
    length: usize,
    /// Current accumulator
    accumulator: f64,
    /// Samples accumulated so far
    count: usize,
    /// Whether to output average (true) or sum (false)
    average: bool,
}

impl IntegrateAndDump {
    /// Create a new integrate-and-dump filter.
    ///
    /// - `length`: Number of samples to integrate
    /// - `average`: If true, output average; if false, output sum
    pub fn new(length: usize, average: bool) -> Self {
        assert!(length > 0);
        Self {
            length,
            accumulator: 0.0,
            count: 0,
            average,
        }
    }

    /// Process a block of samples. Outputs one value per `length` input samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::new();

        for &x in input {
            self.accumulator += x;
            self.count += 1;

            if self.count >= self.length {
                let value = if self.average {
                    self.accumulator / self.length as f64
                } else {
                    self.accumulator
                };
                output.push(value);
                self.accumulator = 0.0;
                self.count = 0;
            }
        }

        output
    }

    /// Process complex samples.
    pub fn process_complex(&mut self, input: &[num_complex::Complex64]) -> Vec<num_complex::Complex64> {
        use num_complex::Complex64;
        let mut output = Vec::new();
        let mut acc = Complex64::new(0.0, 0.0);
        let mut count = 0usize;

        for &x in input {
            acc += x;
            count += 1;

            if count >= self.length {
                let value = if self.average {
                    acc / self.length as f64
                } else {
                    acc
                };
                output.push(value);
                acc = Complex64::new(0.0, 0.0);
                count = 0;
            }
        }

        output
    }

    /// Get integration length.
    pub fn length(&self) -> usize {
        self.length
    }

    /// Reset accumulator.
    pub fn reset(&mut self) {
        self.accumulator = 0.0;
        self.count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use num_complex::Complex64;

    #[test]
    fn test_peak_detector_basic() {
        let mut pd = PeakDetector::new(0.5, 1);
        let signal = vec![0.1, 0.3, 0.8, 0.3, 0.1];
        let peaks = pd.process(&signal);
        assert_eq!(peaks.len(), 1);
        assert_eq!(peaks[0].index, 2);
        assert!((peaks[0].magnitude - 0.8).abs() < 1e-10);
    }

    #[test]
    fn test_peak_detector_threshold() {
        let mut pd = PeakDetector::new(0.9, 1);
        let signal = vec![0.1, 0.3, 0.8, 0.3, 0.1]; // peak=0.8 < threshold=0.9
        let peaks = pd.process(&signal);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_peak_detector_min_spacing() {
        let mut pd = PeakDetector::new(0.3, 5);
        let signal = vec![0.1, 0.5, 0.1, 0.5, 0.1, 0.1, 0.5, 0.1];
        let peaks = pd.process(&signal);
        // Peak at index 1, then index 3 is too close (spacing=2 < 5)
        // Peak at index 6 is 5 samples from index 1 → accepted
        assert_eq!(peaks.len(), 2);
        assert_eq!(peaks[0].index, 1);
        assert_eq!(peaks[1].index, 6);
    }

    #[test]
    fn test_peak_detector_no_peaks() {
        let mut pd = PeakDetector::new(0.5, 1);
        let signal = vec![0.1, 0.2, 0.3, 0.4]; // monotonic, no local max
        let peaks = pd.process(&signal);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_peak_detector_empty() {
        let mut pd = PeakDetector::new(0.5, 1);
        assert!(pd.process(&[]).is_empty());
    }

    #[test]
    fn test_peak_detector_streaming() {
        let mut pd = PeakDetector::new(0.5, 1);
        let peaks1 = pd.process(&[0.1, 0.2, 0.9, 0.2, 0.1]);
        let peaks2 = pd.process(&[0.1, 0.2, 0.8, 0.2, 0.1]);
        assert_eq!(peaks1.len(), 1);
        assert_eq!(peaks1[0].index, 2); // Global index 2
        assert_eq!(peaks2.len(), 1);
        assert_eq!(peaks2[0].index, 7); // Global index 5+2=7
    }

    #[test]
    fn test_parabolic_interpolation_centered() {
        // Symmetric peak → offset = 0
        let frac = parabolic_interpolation(0.5, 1.0, 0.5);
        assert!(frac.abs() < 1e-10);
    }

    #[test]
    fn test_parabolic_interpolation_shifted() {
        // Asymmetric: left higher → peak shifts right
        let frac = parabolic_interpolation(0.8, 1.0, 0.5);
        assert!(frac > 0.0); // Shifts toward higher neighbor
    }

    #[test]
    fn test_integrate_and_dump_sum() {
        let mut iad = IntegrateAndDump::new(4, false);
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let output = iad.process(&input);
        assert_eq!(output.len(), 2);
        assert!((output[0] - 10.0).abs() < 1e-10); // 1+2+3+4
        assert!((output[1] - 26.0).abs() < 1e-10); // 5+6+7+8
    }

    #[test]
    fn test_integrate_and_dump_average() {
        let mut iad = IntegrateAndDump::new(4, true);
        let input = vec![1.0, 2.0, 3.0, 4.0];
        let output = iad.process(&input);
        assert_eq!(output.len(), 1);
        assert!((output[0] - 2.5).abs() < 1e-10);
    }

    #[test]
    fn test_integrate_and_dump_partial() {
        let mut iad = IntegrateAndDump::new(4, false);
        let input = vec![1.0, 2.0]; // Only 2 of 4 needed
        let output = iad.process(&input);
        assert!(output.is_empty()); // Not enough samples yet
        let output2 = iad.process(&[3.0, 4.0]);
        assert_eq!(output2.len(), 1);
        assert!((output2[0] - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_integrate_and_dump_complex() {
        let mut iad = IntegrateAndDump::new(2, false);
        let input = vec![
            Complex64::new(1.0, 2.0),
            Complex64::new(3.0, 4.0),
        ];
        let output = iad.process_complex(&input);
        assert_eq!(output.len(), 1);
        assert!((output[0].re - 4.0).abs() < 1e-10);
        assert!((output[0].im - 6.0).abs() < 1e-10);
    }

    #[test]
    fn test_integrate_and_dump_reset() {
        let mut iad = IntegrateAndDump::new(4, false);
        iad.process(&[1.0, 2.0]);
        iad.reset();
        let output = iad.process(&[10.0, 20.0, 30.0, 40.0]);
        assert_eq!(output.len(), 1);
        assert!((output[0] - 100.0).abs() < 1e-10);
    }
}
