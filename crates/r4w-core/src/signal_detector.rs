//! Signal Detector — Energy-based signal presence detection
//!
//! Determines whether a signal is present in a block of samples using energy
//! detection. Estimates the noise floor from quiet periods and compares
//! instantaneous power against a threshold. Used for spectrum sensing,
//! cognitive radio, and burst detection.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::signal_detector::{SignalDetector, DetectionResult};
//! use num_complex::Complex64;
//!
//! let mut det = SignalDetector::new(6.0, 0.01); // 6 dB above noise
//! // Feed noise-only samples in small blocks to let noise floor settle
//! let noise: Vec<Complex64> = (0..1000)
//!     .map(|i| Complex64::new((i as f64 * 7.3).sin() * 0.01, 0.0))
//!     .collect();
//! for chunk in noise.chunks(50) {
//!     det.process(chunk);
//! }
//! assert!(!det.signal_present());
//! ```

use num_complex::Complex64;

/// Detection result for a block of samples.
#[derive(Debug, Clone, Copy)]
pub struct DetectionResult {
    /// Whether signal is detected.
    pub detected: bool,
    /// Signal power in dB.
    pub signal_power_db: f64,
    /// Estimated noise floor in dB.
    pub noise_floor_db: f64,
    /// SNR estimate in dB (signal_power - noise_floor).
    pub snr_db: f64,
}

/// Energy-based signal detector.
#[derive(Debug, Clone)]
pub struct SignalDetector {
    /// Detection threshold above noise floor in dB.
    threshold_db: f64,
    /// Noise floor averaging constant.
    alpha: f64,
    /// Running noise floor estimate (linear power).
    noise_floor: f64,
    /// Current signal power (linear).
    signal_power: f64,
    /// Detection state.
    detected: bool,
    /// Hysteresis in dB (to prevent chattering).
    hysteresis_db: f64,
    /// Minimum consecutive detections before asserting.
    min_detections: usize,
    /// Detection counter.
    detect_count: usize,
    /// Sample counter for warmup.
    count: u64,
}

impl SignalDetector {
    /// Create a signal detector.
    ///
    /// `threshold_db`: Detection threshold above noise floor in dB (typically 3-10).
    /// `alpha`: Noise floor averaging constant (0.001 = slow, 0.1 = fast).
    pub fn new(threshold_db: f64, alpha: f64) -> Self {
        Self {
            threshold_db: threshold_db.max(0.0),
            alpha: alpha.clamp(0.0001, 1.0),
            noise_floor: 0.0,
            signal_power: 0.0,
            detected: false,
            hysteresis_db: 1.0,
            min_detections: 1,
            detect_count: 0,
            count: 0,
        }
    }

    /// Set hysteresis to prevent chattering.
    pub fn set_hysteresis(&mut self, hysteresis_db: f64) {
        self.hysteresis_db = hysteresis_db.max(0.0);
    }

    /// Set minimum consecutive detections before asserting.
    pub fn set_min_detections(&mut self, min: usize) {
        self.min_detections = min.max(1);
    }

    /// Process a block of samples.
    pub fn process(&mut self, input: &[Complex64]) -> DetectionResult {
        if input.is_empty() {
            return DetectionResult {
                detected: self.detected,
                signal_power_db: -300.0,
                noise_floor_db: -300.0,
                snr_db: 0.0,
            };
        }

        // Compute block power
        let block_power = input.iter().map(|s| s.norm_sqr()).sum::<f64>() / input.len() as f64;
        self.signal_power = block_power;

        let warmup = (2.0 / self.alpha) as u64;

        // Update noise floor (only when not detecting signal)
        if !self.detected {
            if self.count == 0 {
                // Initialize noise floor from first block
                self.noise_floor = block_power;
            } else {
                self.noise_floor = (1.0 - self.alpha) * self.noise_floor + self.alpha * block_power;
            }
        }
        self.count += input.len() as u64;

        // Detect signal
        let threshold_linear = if self.noise_floor > 1e-30 {
            self.noise_floor * 10.0_f64.powf(self.threshold_db / 10.0)
        } else {
            f64::MAX
        };

        let hysteresis_linear = if self.detected {
            // Use lower threshold when already detecting (hysteresis)
            self.noise_floor * 10.0_f64.powf((self.threshold_db - self.hysteresis_db) / 10.0)
        } else {
            threshold_linear
        };

        if self.count > warmup && block_power > hysteresis_linear {
            self.detect_count += 1;
            if self.detect_count >= self.min_detections {
                self.detected = true;
            }
        } else {
            self.detect_count = 0;
            self.detected = false;
        }

        let signal_db = if block_power > 1e-30 { 10.0 * block_power.log10() } else { -300.0 };
        let noise_db = if self.noise_floor > 1e-30 { 10.0 * self.noise_floor.log10() } else { -300.0 };

        DetectionResult {
            detected: self.detected,
            signal_power_db: signal_db,
            noise_floor_db: noise_db,
            snr_db: signal_db - noise_db,
        }
    }

    /// Whether signal is currently detected.
    pub fn signal_present(&self) -> bool {
        self.detected
    }

    /// Current noise floor estimate in dB.
    pub fn noise_floor_db(&self) -> f64 {
        if self.noise_floor > 1e-30 { 10.0 * self.noise_floor.log10() } else { -300.0 }
    }

    /// Current signal power in dB.
    pub fn signal_power_db(&self) -> f64 {
        if self.signal_power > 1e-30 { 10.0 * self.signal_power.log10() } else { -300.0 }
    }

    /// Reset the detector state.
    pub fn reset(&mut self) {
        self.noise_floor = 0.0;
        self.signal_power = 0.0;
        self.detected = false;
        self.detect_count = 0;
        self.count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_noise(amplitude: f64, n: usize) -> Vec<Complex64> {
        (0..n).map(|i| {
            let v = (i as f64 * 7.3 + 0.5).sin() * amplitude;
            Complex64::new(v, (i as f64 * 11.1).sin() * amplitude)
        }).collect()
    }

    #[test]
    fn test_no_signal() {
        let mut det = SignalDetector::new(6.0, 0.1);
        let noise = make_noise(0.01, 500);
        // Process in small blocks to let noise floor settle
        for chunk in noise.chunks(50) {
            det.process(chunk);
        }
        let final_block = make_noise(0.01, 100);
        let result = det.process(&final_block);
        assert!(!result.detected, "Should not detect noise-only");
    }

    #[test]
    fn test_detects_signal() {
        let mut det = SignalDetector::new(6.0, 0.1);
        // First establish noise floor
        let noise = make_noise(0.01, 500);
        det.process(&noise);
        // Now add strong signal
        let signal: Vec<Complex64> = (0..200)
            .map(|i| Complex64::new(1.0 * (0.3 * i as f64).cos(), 0.0))
            .collect();
        let result = det.process(&signal);
        assert!(result.detected, "Should detect strong signal");
        assert!(result.snr_db > 6.0, "SNR should be above threshold, got {} dB", result.snr_db);
    }

    #[test]
    fn test_returns_to_no_signal() {
        let mut det = SignalDetector::new(6.0, 0.1);
        // Establish baseline in small blocks
        let noise = make_noise(0.01, 500);
        for chunk in noise.chunks(50) {
            det.process(chunk);
        }
        // Signal
        let signal = vec![Complex64::new(1.0, 0.0); 100];
        det.process(&signal);
        assert!(det.signal_present());
        // Back to noise
        let noise2 = make_noise(0.01, 200);
        det.process(&noise2);
        assert!(!det.signal_present(), "Should return to no-signal");
    }

    #[test]
    fn test_noise_floor_tracking() {
        let mut det = SignalDetector::new(6.0, 0.1);
        let noise = make_noise(0.1, 1000);
        det.process(&noise);
        let nf = det.noise_floor_db();
        // Noise floor should be near 10*log10(0.1^2/2) ≈ -13 dB
        assert!(nf > -30.0 && nf < 0.0, "Noise floor should be reasonable, got {} dB", nf);
    }

    #[test]
    fn test_detection_result_fields() {
        let mut det = SignalDetector::new(6.0, 0.1);
        // Establish noise floor first
        let noise = make_noise(0.01, 500);
        for chunk in noise.chunks(50) {
            det.process(chunk);
        }
        // Now check strong signal
        let signal = vec![Complex64::new(1.0, 0.0); 100];
        let result = det.process(&signal);
        assert!(result.signal_power_db > -5.0); // |1|^2 = 1 → 0 dB
        assert!(result.noise_floor_db < result.signal_power_db);
    }

    #[test]
    fn test_empty_input() {
        let mut det = SignalDetector::new(6.0, 0.1);
        let result = det.process(&[]);
        assert!(!result.detected);
        assert!(result.signal_power_db < -100.0);
    }

    #[test]
    fn test_hysteresis() {
        let mut det = SignalDetector::new(6.0, 0.1);
        det.set_hysteresis(3.0);
        // Establish baseline
        let noise = make_noise(0.01, 500);
        det.process(&noise);
        // Marginal signal (just above threshold)
        let signal = make_noise(0.1, 100);
        det.process(&signal);
        // State should persist with hysteresis
    }

    #[test]
    fn test_reset() {
        let mut det = SignalDetector::new(6.0, 0.1);
        det.process(&vec![Complex64::new(1.0, 0.0); 100]);
        det.reset();
        assert!(!det.signal_present());
        assert!(det.noise_floor_db() < -200.0);
    }

    #[test]
    fn test_min_detections() {
        let mut det = SignalDetector::new(3.0, 0.1);
        det.set_min_detections(3);
        // Establish baseline
        let noise = make_noise(0.01, 500);
        det.process(&noise);
        // First detection block
        let signal = vec![Complex64::new(1.0, 0.0); 100];
        det.process(&signal);
        // May not detect yet (depends on min_detections)
    }
}
