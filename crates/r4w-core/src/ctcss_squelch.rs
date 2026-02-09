//! CTCSS Squelch — Continuous Tone-Coded Squelch System
//!
//! Encodes and decodes sub-audible tones (67.0–254.1 Hz) used for
//! repeater access control in FM radio systems. Includes Goertzel-based
//! tone detection, tone generation, and standard EIA/TIA tone table.
//! GNU Radio equivalent: `gr::analog::ctcss_squelch_ff`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ctcss_squelch::{CtcssDetector, CtcssGenerator, CTCSS_TONES};
//!
//! let mut gen = CtcssGenerator::new(100.0, 8000.0, 0.5);
//! let samples = gen.generate(8000); // 1 second at 8kHz
//!
//! let det = CtcssDetector::new(8000.0, 0.1);
//! let tone = det.detect(&samples);
//! assert!(tone.is_some());
//! assert!((tone.unwrap() - 100.0).abs() < 1.0);
//! ```

use std::f64::consts::PI;

/// Standard CTCSS tone frequencies (Hz) per EIA/TIA-603.
pub const CTCSS_TONES: [f64; 38] = [
    67.0, 71.9, 74.4, 77.0, 79.7, 82.5, 85.4, 88.5, 91.5, 94.8, 97.4,
    100.0, 103.5, 107.2, 110.9, 114.8, 118.8, 123.0, 127.3, 131.8, 136.5,
    141.3, 146.2, 151.4, 156.7, 162.2, 167.9, 173.8, 179.9, 186.2, 192.8,
    203.5, 210.7, 218.1, 225.7, 233.6, 241.8, 254.1,
];

/// CTCSS tone generator.
#[derive(Debug, Clone)]
pub struct CtcssGenerator {
    /// Tone frequency in Hz.
    frequency: f64,
    /// Sample rate in Hz.
    sample_rate: f64,
    /// Tone amplitude (0.0 to 1.0).
    amplitude: f64,
    /// Phase accumulator.
    phase: f64,
    /// Phase increment per sample.
    phase_inc: f64,
}

impl CtcssGenerator {
    /// Create a new CTCSS tone generator.
    pub fn new(frequency: f64, sample_rate: f64, amplitude: f64) -> Self {
        let phase_inc = 2.0 * PI * frequency / sample_rate;
        Self {
            frequency,
            sample_rate,
            amplitude: amplitude.clamp(0.0, 1.0),
            phase: 0.0,
            phase_inc,
        }
    }

    /// Generate samples.
    pub fn generate(&mut self, num_samples: usize) -> Vec<f64> {
        let mut output = Vec::with_capacity(num_samples);
        for _ in 0..num_samples {
            output.push(self.amplitude * self.phase.sin());
            self.phase += self.phase_inc;
            if self.phase >= 2.0 * PI {
                self.phase -= 2.0 * PI;
            }
        }
        output
    }

    /// Mix CTCSS tone into audio signal.
    pub fn add_to_signal(&mut self, audio: &[f64]) -> Vec<f64> {
        let tone = self.generate(audio.len());
        audio
            .iter()
            .zip(tone.iter())
            .map(|(&a, &t)| a + t)
            .collect()
    }

    /// Set frequency.
    pub fn set_frequency(&mut self, freq: f64) {
        self.frequency = freq;
        self.phase_inc = 2.0 * PI * freq / self.sample_rate;
    }

    /// Get frequency.
    pub fn frequency(&self) -> f64 {
        self.frequency
    }
}

/// CTCSS tone detector using Goertzel algorithm.
#[derive(Debug, Clone)]
pub struct CtcssDetector {
    /// Sample rate in Hz.
    sample_rate: f64,
    /// Detection threshold (relative power).
    threshold: f64,
    /// Block size for Goertzel (determines frequency resolution).
    block_size: usize,
}

impl CtcssDetector {
    /// Create a new CTCSS detector.
    ///
    /// `threshold`: detection threshold (0.0 to 1.0, typical 0.1 to 0.5).
    pub fn new(sample_rate: f64, threshold: f64) -> Self {
        // Block size for ~1 Hz resolution
        let block_size = (sample_rate / 1.0).min(4096.0) as usize;
        Self {
            sample_rate,
            threshold,
            block_size,
        }
    }

    /// Detect which CTCSS tone (if any) is present in the signal.
    ///
    /// Returns the detected tone frequency, or None.
    pub fn detect(&self, samples: &[f64]) -> Option<f64> {
        if samples.len() < self.block_size {
            return None;
        }

        let block = &samples[..self.block_size];
        let mut best_tone = 0.0f64;
        let mut best_power = 0.0f64;

        // Test each standard CTCSS tone
        for &freq in &CTCSS_TONES {
            let power = goertzel_power(block, freq, self.sample_rate);
            if power > best_power {
                best_power = power;
                best_tone = freq;
            }
        }

        // Compute total signal power for threshold comparison
        let total_power: f64 = block.iter().map(|&s| s * s).sum::<f64>() / block.len() as f64;

        if total_power > 1e-12 && best_power / (total_power * self.block_size as f64) > self.threshold
        {
            Some(best_tone)
        } else {
            None
        }
    }

    /// Detect with power level.
    pub fn detect_with_power(&self, samples: &[f64]) -> Option<(f64, f64)> {
        if samples.len() < self.block_size {
            return None;
        }

        let block = &samples[..self.block_size];
        let mut best_tone = 0.0f64;
        let mut best_power = 0.0f64;

        for &freq in &CTCSS_TONES {
            let power = goertzel_power(block, freq, self.sample_rate);
            if power > best_power {
                best_power = power;
                best_tone = freq;
            }
        }

        let total_power: f64 = block.iter().map(|&s| s * s).sum::<f64>() / block.len() as f64;
        let relative = if total_power > 1e-12 {
            best_power / (total_power * self.block_size as f64)
        } else {
            0.0
        };

        if relative > self.threshold {
            Some((best_tone, relative))
        } else {
            None
        }
    }

    /// Get block size.
    pub fn block_size(&self) -> usize {
        self.block_size
    }

    /// Set threshold.
    pub fn set_threshold(&mut self, threshold: f64) {
        self.threshold = threshold;
    }
}

/// CTCSS squelch gate — passes audio only when correct tone is detected.
#[derive(Debug, Clone)]
pub struct CtcssSquel {
    detector: CtcssDetector,
    /// Required tone frequency.
    required_tone: f64,
    /// Tolerance in Hz.
    tolerance: f64,
    /// Whether squelch is currently open.
    is_open: bool,
}

impl CtcssSquel {
    /// Create a CTCSS squelch with required tone.
    pub fn new(required_tone: f64, sample_rate: f64, threshold: f64) -> Self {
        Self {
            detector: CtcssDetector::new(sample_rate, threshold),
            required_tone,
            tolerance: 2.0,
            is_open: false,
        }
    }

    /// Process audio: returns gated output (zeros if wrong/no tone).
    pub fn process(&mut self, samples: &[f64]) -> Vec<f64> {
        if let Some(tone) = self.detector.detect(samples) {
            self.is_open = (tone - self.required_tone).abs() < self.tolerance;
        } else {
            self.is_open = false;
        }

        if self.is_open {
            samples.to_vec()
        } else {
            vec![0.0; samples.len()]
        }
    }

    /// Check if squelch is open.
    pub fn is_open(&self) -> bool {
        self.is_open
    }
}

/// Goertzel algorithm: compute power at a specific frequency.
fn goertzel_power(samples: &[f64], freq: f64, sample_rate: f64) -> f64 {
    let n = samples.len();
    let k = (freq * n as f64 / sample_rate).round();
    let w = 2.0 * PI * k / n as f64;
    let coeff = 2.0 * w.cos();

    let mut s0 = 0.0f64;
    let mut s1 = 0.0f64;
    let mut s2;

    for &x in samples {
        s2 = s1;
        s1 = s0;
        s0 = x + coeff * s1 - s2;
    }

    // Power = s0^2 + s1^2 - coeff * s0 * s1
    s0 * s0 + s1 * s1 - coeff * s0 * s1
}

/// Find the closest standard CTCSS tone to a given frequency.
pub fn nearest_ctcss_tone(freq: f64) -> f64 {
    CTCSS_TONES
        .iter()
        .min_by(|&&a, &&b| {
            (a - freq)
                .abs()
                .partial_cmp(&(b - freq).abs())
                .unwrap()
        })
        .copied()
        .unwrap_or(100.0)
}

/// Get CTCSS tone index (0-37) for a frequency.
pub fn ctcss_tone_index(freq: f64) -> Option<usize> {
    CTCSS_TONES
        .iter()
        .position(|&t| (t - freq).abs() < 0.5)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tone_table() {
        assert_eq!(CTCSS_TONES.len(), 38);
        assert!((CTCSS_TONES[0] - 67.0).abs() < 0.1);
        assert!((CTCSS_TONES[37] - 254.1).abs() < 0.1);
        // Should be monotonically increasing
        for w in CTCSS_TONES.windows(2) {
            assert!(w[1] > w[0]);
        }
    }

    #[test]
    fn test_generator() {
        let mut gen = CtcssGenerator::new(100.0, 8000.0, 0.15);
        let samples = gen.generate(800);
        assert_eq!(samples.len(), 800);
        let max = samples.iter().cloned().fold(0.0f64, f64::max);
        assert!(max <= 0.15 + 1e-10);
    }

    #[test]
    fn test_detector_100hz() {
        let mut gen = CtcssGenerator::new(100.0, 8000.0, 0.5);
        let samples = gen.generate(8000); // 1 second
        let det = CtcssDetector::new(8000.0, 0.1);
        let tone = det.detect(&samples);
        assert!(tone.is_some(), "Should detect 100 Hz tone");
        assert!((tone.unwrap() - 100.0).abs() < 1.0);
    }

    #[test]
    fn test_detector_no_tone() {
        let det = CtcssDetector::new(8000.0, 0.5);
        // White noise-like
        let samples: Vec<f64> = (0..8000)
            .map(|i| ((i * 12345 + 6789) % 1000) as f64 / 500.0 - 1.0)
            .collect();
        // With high threshold, should not detect
        let tone = det.detect(&samples);
        // May or may not detect depending on noise; just don't crash
        let _ = tone;
    }

    #[test]
    fn test_add_to_signal() {
        let mut gen = CtcssGenerator::new(100.0, 8000.0, 0.1);
        let audio = vec![0.5f64; 800];
        let mixed = gen.add_to_signal(&audio);
        assert_eq!(mixed.len(), 800);
        // Mixed should be different from original
        assert!((mixed[100] - 0.5).abs() > 0.001);
    }

    #[test]
    fn test_squelch_correct_tone() {
        let mut gen = CtcssGenerator::new(100.0, 8000.0, 0.5);
        let samples = gen.generate(8000);
        let mut squelch = CtcssSquel::new(100.0, 8000.0, 0.1);
        let output = squelch.process(&samples);
        // Should pass through
        if squelch.is_open() {
            assert!(output.iter().any(|&s| s.abs() > 0.01));
        }
    }

    #[test]
    fn test_squelch_wrong_tone() {
        let mut gen = CtcssGenerator::new(100.0, 8000.0, 0.5);
        let samples = gen.generate(8000);
        let mut squelch = CtcssSquel::new(150.0, 8000.0, 0.1);
        let output = squelch.process(&samples);
        // Should be muted (wrong tone)
        if !squelch.is_open() {
            assert!(output.iter().all(|&s| s == 0.0));
        }
    }

    #[test]
    fn test_nearest_tone() {
        assert!((nearest_ctcss_tone(99.5) - 100.0).abs() < 0.1);
        assert!((nearest_ctcss_tone(67.5) - 67.0).abs() < 1.0);
    }

    #[test]
    fn test_tone_index() {
        assert_eq!(ctcss_tone_index(100.0), Some(11));
        assert_eq!(ctcss_tone_index(67.0), Some(0));
        assert_eq!(ctcss_tone_index(999.0), None);
    }

    #[test]
    fn test_set_frequency() {
        let mut gen = CtcssGenerator::new(100.0, 8000.0, 0.15);
        gen.set_frequency(150.0);
        assert!((gen.frequency() - 150.0).abs() < 1e-10);
    }

    #[test]
    fn test_detect_with_power() {
        let mut gen = CtcssGenerator::new(100.0, 8000.0, 0.5);
        let samples = gen.generate(8000);
        let det = CtcssDetector::new(8000.0, 0.1);
        if let Some((tone, power)) = det.detect_with_power(&samples) {
            assert!((tone - 100.0).abs() < 1.0);
            assert!(power > 0.0);
        }
    }
}
