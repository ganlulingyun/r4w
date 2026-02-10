//! # Complex to Arg (Phase Angle)
//!
//! Extracts the phase angle (argument) from complex IQ samples.
//! Outputs values in radians (-π to π) or degrees (-180 to 180).
//! Essential for FM demodulation, phase analysis, and constellation viewing.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::complex_to_arg::{complex_to_arg, complex_to_arg_deg};
//!
//! let samples = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
//! let phases_rad = complex_to_arg(&samples);
//! let phases_deg = complex_to_arg_deg(&samples);
//! assert!((phases_deg[0] - 0.0).abs() < 1e-10);    // 0°
//! assert!((phases_deg[1] - 90.0).abs() < 1e-10);   // 90°
//! ```

use std::f64::consts::PI;

/// Extract phase angle in radians from complex samples.
pub fn complex_to_arg(samples: &[(f64, f64)]) -> Vec<f64> {
    samples.iter().map(|&(re, im)| im.atan2(re)).collect()
}

/// Extract phase angle in degrees from complex samples.
pub fn complex_to_arg_deg(samples: &[(f64, f64)]) -> Vec<f64> {
    samples
        .iter()
        .map(|&(re, im)| im.atan2(re).to_degrees())
        .collect()
}

/// Extract magnitude from complex samples.
pub fn complex_to_mag(samples: &[(f64, f64)]) -> Vec<f64> {
    samples
        .iter()
        .map(|&(re, im)| (re * re + im * im).sqrt())
        .collect()
}

/// Extract magnitude squared from complex samples (avoids sqrt).
pub fn complex_to_mag_sq(samples: &[(f64, f64)]) -> Vec<f64> {
    samples
        .iter()
        .map(|&(re, im)| re * re + im * im)
        .collect()
}

/// Extract both magnitude and phase from complex samples.
pub fn complex_to_mag_arg(samples: &[(f64, f64)]) -> Vec<(f64, f64)> {
    samples
        .iter()
        .map(|&(re, im)| {
            let mag = (re * re + im * im).sqrt();
            let arg = im.atan2(re);
            (mag, arg)
        })
        .collect()
}

/// Convert magnitude and phase back to complex (I/Q).
pub fn mag_arg_to_complex(mag_phase: &[(f64, f64)]) -> Vec<(f64, f64)> {
    mag_phase
        .iter()
        .map(|&(mag, phase)| (mag * phase.cos(), mag * phase.sin()))
        .collect()
}

/// Compute instantaneous frequency from phase differences.
/// Returns frequency in radians per sample.
pub fn instantaneous_frequency(samples: &[(f64, f64)]) -> Vec<f64> {
    if samples.len() < 2 {
        return Vec::new();
    }

    let phases = complex_to_arg(samples);
    let mut freq = Vec::with_capacity(phases.len() - 1);

    for i in 1..phases.len() {
        let mut diff = phases[i] - phases[i - 1];
        // Unwrap phase.
        while diff > PI {
            diff -= 2.0 * PI;
        }
        while diff < -PI {
            diff += 2.0 * PI;
        }
        freq.push(diff);
    }

    freq
}

/// Stateful phase extractor with unwrapping.
#[derive(Debug, Clone)]
pub struct PhaseExtractor {
    /// Previous phase (for unwrapping).
    prev_phase: f64,
    /// Accumulated phase offset.
    offset: f64,
    /// Whether we've seen the first sample.
    initialized: bool,
    /// Total samples processed.
    count: u64,
}

impl PhaseExtractor {
    /// Create a new phase extractor.
    pub fn new() -> Self {
        Self {
            prev_phase: 0.0,
            offset: 0.0,
            initialized: false,
            count: 0,
        }
    }

    /// Extract unwrapped phase from complex samples.
    pub fn process(&mut self, samples: &[(f64, f64)]) -> Vec<f64> {
        let mut output = Vec::with_capacity(samples.len());

        for &(re, im) in samples {
            let phase = im.atan2(re);

            if !self.initialized {
                self.prev_phase = phase;
                self.initialized = true;
            } else {
                let mut diff = phase - self.prev_phase;
                if diff > PI {
                    self.offset -= 2.0 * PI;
                } else if diff < -PI {
                    self.offset += 2.0 * PI;
                }
                self.prev_phase = phase;
            }

            output.push(phase + self.offset);
            self.count += 1;
        }

        output
    }

    /// Get total samples processed.
    pub fn count(&self) -> u64 {
        self.count
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.prev_phase = 0.0;
        self.offset = 0.0;
        self.initialized = false;
        self.count = 0;
    }
}

impl Default for PhaseExtractor {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cardinal_directions() {
        let samples = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
        let phases = complex_to_arg_deg(&samples);
        assert!((phases[0] - 0.0).abs() < 1e-10);
        assert!((phases[1] - 90.0).abs() < 1e-10);
        assert!((phases[2] - 180.0).abs() < 1e-10);
        assert!((phases[3] + 90.0).abs() < 1e-10);
    }

    #[test]
    fn test_magnitude() {
        let samples = vec![(3.0, 4.0), (0.0, 0.0), (1.0, 0.0)];
        let mags = complex_to_mag(&samples);
        assert!((mags[0] - 5.0).abs() < 1e-10);
        assert!((mags[1] - 0.0).abs() < 1e-10);
        assert!((mags[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_mag_squared() {
        let samples = vec![(3.0, 4.0)];
        let mag_sq = complex_to_mag_sq(&samples);
        assert!((mag_sq[0] - 25.0).abs() < 1e-10);
    }

    #[test]
    fn test_mag_arg_roundtrip() {
        let original = vec![(1.0, 0.0), (0.0, 1.0), (-0.5, 0.5)];
        let mag_arg = complex_to_mag_arg(&original);
        let restored = mag_arg_to_complex(&mag_arg);
        for (o, r) in original.iter().zip(restored.iter()) {
            assert!((o.0 - r.0).abs() < 1e-10);
            assert!((o.1 - r.1).abs() < 1e-10);
        }
    }

    #[test]
    fn test_instantaneous_frequency() {
        // Constant frequency: phase advances by π/4 per sample.
        let n = 16;
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = i as f64 * PI / 4.0;
                (phase.cos(), phase.sin())
            })
            .collect();
        let freq = instantaneous_frequency(&samples);
        assert_eq!(freq.len(), n - 1);
        for &f in &freq {
            assert!((f - PI / 4.0).abs() < 1e-10, "f={}", f);
        }
    }

    #[test]
    fn test_phase_extractor_unwrap() {
        let mut pe = PhaseExtractor::new();
        // Generate samples that wrap around.
        let n = 20;
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = i as f64 * 0.5; // Steadily increasing phase.
                (phase.cos(), phase.sin())
            })
            .collect();
        let unwrapped = pe.process(&samples);
        // Unwrapped phase should be monotonically increasing.
        for i in 1..unwrapped.len() {
            assert!(
                unwrapped[i] >= unwrapped[i - 1] - 0.01,
                "Phase should be monotonic: {}[{}] vs {}[{}]",
                unwrapped[i], i, unwrapped[i - 1], i - 1
            );
        }
    }

    #[test]
    fn test_empty_input() {
        assert!(complex_to_arg(&[]).is_empty());
        assert!(complex_to_arg_deg(&[]).is_empty());
        assert!(complex_to_mag(&[]).is_empty());
        assert!(instantaneous_frequency(&[]).is_empty());
    }

    #[test]
    fn test_phase_extractor_count() {
        let mut pe = PhaseExtractor::new();
        pe.process(&[(1.0, 0.0), (0.0, 1.0)]);
        assert_eq!(pe.count(), 2);
    }

    #[test]
    fn test_phase_extractor_reset() {
        let mut pe = PhaseExtractor::new();
        pe.process(&[(1.0, 0.0)]);
        pe.reset();
        assert_eq!(pe.count(), 0);
    }

    #[test]
    fn test_zero_magnitude() {
        let samples = vec![(0.0, 0.0)];
        let phases = complex_to_arg(&samples);
        // atan2(0,0) = 0 on most platforms.
        assert_eq!(phases.len(), 1);
    }
}
