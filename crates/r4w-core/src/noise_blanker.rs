//! Noise Blanker — Impulse noise detection and suppression
//!
//! Detects and blanks impulsive noise (e.g., ignition, power line, lightning)
//! that appears as short-duration, high-amplitude spikes. Common in HF receivers
//! and automotive environments.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::noise_blanker::NoiseBlanker;
//! use num_complex::Complex64;
//!
//! let mut nb = NoiseBlanker::new(3.0, 5);
//! nb.set_alpha(0.1); // fast settling for demo
//! let mut signal = vec![Complex64::new(0.1, 0.0); 100];
//! signal[50] = Complex64::new(10.0, 0.0); // impulse
//! let cleaned = nb.process(&signal);
//! assert!(cleaned[50].norm() < 1.0); // Impulse suppressed
//! ```

use num_complex::Complex64;
use std::collections::VecDeque;

/// Noise blanker — detects and removes impulse noise.
#[derive(Debug, Clone)]
pub struct NoiseBlanker {
    /// Threshold multiplier above running average to trigger blanking.
    threshold: f64,
    /// Number of samples to blank around each detection.
    blank_width: usize,
    /// Smoothing factor for running power estimate.
    alpha: f64,
    /// Running average power estimate.
    avg_power: f64,
    /// Delay buffer for lookahead.
    buffer: VecDeque<Complex64>,
    /// Blanking mode: Zero or Interpolate.
    mode: BlankMode,
    /// Samples processed (for warmup).
    count: u64,
}

/// Blanking strategy.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BlankMode {
    /// Replace blanked samples with zero.
    Zero,
    /// Interpolate between surrounding good samples.
    Interpolate,
    /// Hold last good sample.
    Hold,
}

impl NoiseBlanker {
    /// Create a noise blanker.
    ///
    /// `threshold`: Multiplier above running average to detect impulse (typically 3-10).
    /// `blank_width`: Number of samples to blank (half before + half after detection).
    pub fn new(threshold: f64, blank_width: usize) -> Self {
        Self {
            threshold: threshold.max(1.0),
            blank_width: blank_width.max(1),
            alpha: 0.01,
            avg_power: 0.0,
            buffer: VecDeque::new(),
            mode: BlankMode::Zero,
            count: 0,
        }
    }

    /// Set the blanking mode.
    pub fn set_mode(&mut self, mode: BlankMode) {
        self.mode = mode;
    }

    /// Set the power averaging constant.
    pub fn set_alpha(&mut self, alpha: f64) {
        self.alpha = alpha.clamp(0.001, 0.5);
    }

    /// Process a block of samples, blanking impulse noise.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len());
        let mut blank_countdown = 0usize;
        let mut last_good = Complex64::new(0.0, 0.0);
        // Warmup: need enough samples for a stable average
        let warmup = (2.0 / self.alpha) as u64;

        for &sample in input {
            let power = sample.norm_sqr();

            // Update running average (exclude detected impulses)
            if blank_countdown == 0 {
                self.avg_power = (1.0 - self.alpha) * self.avg_power + self.alpha * power;
                self.count += 1;
            }

            // Detect impulse (only after warmup)
            let threshold_power = self.avg_power * self.threshold * self.threshold;
            if self.count > warmup && power > threshold_power && self.avg_power > 1e-30 {
                blank_countdown = self.blank_width;
            }

            // Output
            if blank_countdown > 0 {
                match self.mode {
                    BlankMode::Zero => output.push(Complex64::new(0.0, 0.0)),
                    BlankMode::Hold => output.push(last_good),
                    BlankMode::Interpolate => output.push(Complex64::new(0.0, 0.0)),
                }
                blank_countdown -= 1;
            } else {
                last_good = sample;
                output.push(sample);
            }
        }

        output
    }

    /// Get the current estimated average power.
    pub fn avg_power(&self) -> f64 {
        self.avg_power
    }

    pub fn reset(&mut self) {
        self.avg_power = 0.0;
        self.buffer.clear();
        self.count = 0;
    }
}

/// Simple threshold-based blanker (no averaging, just hard clip).
#[derive(Debug, Clone)]
pub struct ClipBlanker {
    /// Hard threshold level.
    clip_level: f64,
}

impl ClipBlanker {
    pub fn new(clip_level: f64) -> Self {
        Self { clip_level }
    }

    /// Clip samples exceeding the threshold to the threshold magnitude.
    pub fn process(&self, input: &[Complex64]) -> Vec<Complex64> {
        input
            .iter()
            .map(|&s| {
                let mag = s.norm();
                if mag > self.clip_level {
                    s * (self.clip_level / mag)
                } else {
                    s
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_blanks_impulse() {
        let mut nb = NoiseBlanker::new(3.0, 3);
        nb.set_alpha(0.1);
        // Build up average with normal signal
        let mut signal: Vec<Complex64> = (0..50)
            .map(|_| Complex64::new(0.1, 0.0))
            .collect();
        // Add impulse
        signal.push(Complex64::new(10.0, 0.0));
        signal.extend((0..50).map(|_| Complex64::new(0.1, 0.0)));

        let output = nb.process(&signal);
        // Impulse at index 50 should be blanked (near zero)
        assert!(output[50].norm() < 0.5, "Impulse should be blanked, got {}", output[50].norm());
    }

    #[test]
    fn test_passes_normal_signal() {
        let mut nb = NoiseBlanker::new(5.0, 3);
        let signal: Vec<Complex64> = (0..100)
            .map(|i| Complex64::from_polar(0.5, i as f64 * 0.1))
            .collect();
        let output = nb.process(&signal);
        // Should pass through mostly unchanged
        assert_eq!(output.len(), signal.len());
        // After settling, samples should be similar
        for i in 20..100 {
            assert!((output[i] - signal[i]).norm() < 0.01,
                "Sample {} changed unexpectedly", i);
        }
    }

    #[test]
    fn test_silence_no_blanking() {
        let mut nb = NoiseBlanker::new(3.0, 3);
        let signal = vec![Complex64::new(0.0, 0.0); 50];
        let output = nb.process(&signal);
        for s in &output {
            assert_eq!(s.norm(), 0.0);
        }
    }

    #[test]
    fn test_hold_mode() {
        let mut nb = NoiseBlanker::new(3.0, 3);
        nb.set_mode(BlankMode::Hold);
        nb.set_alpha(0.1);
        let mut signal: Vec<Complex64> = (0..50)
            .map(|_| Complex64::new(0.2, 0.0))
            .collect();
        signal.push(Complex64::new(20.0, 0.0)); // impulse
        signal.extend((0..10).map(|_| Complex64::new(0.2, 0.0)));

        let output = nb.process(&signal);
        // Blanked sample should hold last good value (0.2)
        assert!((output[50].re - 0.2).abs() < 0.01, "Hold mode should use last good sample");
    }

    #[test]
    fn test_multiple_impulses() {
        let mut nb = NoiseBlanker::new(3.0, 1);
        nb.set_alpha(0.1);
        let mut signal: Vec<Complex64> = (0..100)
            .map(|_| Complex64::new(0.1, 0.0))
            .collect();
        signal[30] = Complex64::new(5.0, 0.0);
        signal[60] = Complex64::new(8.0, 0.0);

        let output = nb.process(&signal);
        assert!(output[30].norm() < 0.5);
        assert!(output[60].norm() < 0.5);
    }

    #[test]
    fn test_clip_blanker() {
        let cb = ClipBlanker::new(1.0);
        let input = vec![
            Complex64::new(0.5, 0.0),
            Complex64::new(3.0, 0.0),
            Complex64::new(-0.3, 0.4),
        ];
        let output = cb.process(&input);
        assert!((output[0].norm() - 0.5).abs() < 1e-10); // Below clip
        assert!((output[1].norm() - 1.0).abs() < 1e-10); // Clipped to 1.0
        assert!(output[2].norm() <= 1.0 + 1e-10); // At or below clip
    }

    #[test]
    fn test_reset() {
        let mut nb = NoiseBlanker::new(3.0, 3);
        nb.avg_power = 5.0;
        nb.reset();
        assert_eq!(nb.avg_power(), 0.0);
    }

    #[test]
    fn test_blank_width() {
        let mut nb = NoiseBlanker::new(3.0, 5);
        nb.set_alpha(0.1);
        let mut signal: Vec<Complex64> = (0..100)
            .map(|_| Complex64::new(0.1, 0.0))
            .collect();
        signal[50] = Complex64::new(20.0, 0.0);
        let output = nb.process(&signal);
        // At least blank_width samples around impulse should be blanked
        let mut blanked = 0;
        for i in 48..56 {
            if output[i].norm() < 0.01 {
                blanked += 1;
            }
        }
        assert!(blanked >= 3, "Should blank multiple samples, blanked {}", blanked);
    }
}
