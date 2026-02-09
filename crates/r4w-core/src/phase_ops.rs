//! Phase Wrap and Unwrap Operations
//!
//! Utilities for processing phase angle streams. Phase wrapping constrains
//! angles to [-π, π], while phase unwrapping reconstructs continuous phase
//! trajectories from wrapped (discontinuous) phase values.
//!
//! ## Blocks
//!
//! - **PhaseWrap**: Wrap angles to [-π, π]
//! - **PhaseUnwrap**: Reconstruct continuous phase from wrapped values
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::phase_ops::{phase_wrap, phase_unwrap};
//! use std::f64::consts::PI;
//!
//! // Wrap an angle beyond π
//! let wrapped = phase_wrap(&[3.5]);
//! assert!((wrapped[0] - (3.5 - 2.0 * PI)).abs() < 1e-10);
//!
//! // Unwrap a phase trajectory with discontinuity
//! let wrapped_phase = vec![0.0, 1.0, 2.0, 3.0, -3.0, -2.0, -1.0]; // jump at index 4
//! let unwrapped = phase_unwrap(&wrapped_phase);
//! // Should be monotonically increasing after unwrap
//! for i in 1..unwrapped.len() {
//!     assert!(unwrapped[i] > unwrapped[i-1] - 0.1);
//! }
//! ```

use std::f64::consts::PI;

const TWO_PI: f64 = 2.0 * PI;

/// Wrap phase angles to [-π, π].
pub fn phase_wrap(input: &[f64]) -> Vec<f64> {
    input.iter().map(|&x| wrap_angle(x)).collect()
}

/// Wrap a single angle to [-π, π].
#[inline]
pub fn wrap_angle(x: f64) -> f64 {
    let mut y = x % TWO_PI;
    if y > PI {
        y -= TWO_PI;
    } else if y < -PI {
        y += TWO_PI;
    }
    y
}

/// Unwrap phase angles to produce a continuous trajectory.
///
/// When the difference between consecutive samples exceeds π, a 2π correction
/// is applied to remove the discontinuity.
pub fn phase_unwrap(input: &[f64]) -> Vec<f64> {
    if input.is_empty() {
        return Vec::new();
    }
    let mut output = Vec::with_capacity(input.len());
    output.push(input[0]);
    let mut cumulative_correction = 0.0;

    for i in 1..input.len() {
        let diff = input[i] - input[i - 1];
        if diff > PI {
            cumulative_correction -= TWO_PI;
        } else if diff < -PI {
            cumulative_correction += TWO_PI;
        }
        output.push(input[i] + cumulative_correction);
    }

    output
}

/// Stateful phase unwrapper for streaming.
#[derive(Debug, Clone)]
pub struct PhaseUnwrapper {
    prev_sample: Option<f64>,
    cumulative_correction: f64,
}

impl PhaseUnwrapper {
    pub fn new() -> Self {
        Self {
            prev_sample: None,
            cumulative_correction: 0.0,
        }
    }

    /// Process a block of wrapped phase values.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());

        for &sample in input {
            if let Some(prev) = self.prev_sample {
                let diff = sample - prev;
                if diff > PI {
                    self.cumulative_correction -= TWO_PI;
                } else if diff < -PI {
                    self.cumulative_correction += TWO_PI;
                }
            }
            output.push(sample + self.cumulative_correction);
            self.prev_sample = Some(sample);
        }

        output
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.prev_sample = None;
        self.cumulative_correction = 0.0;
    }
}

impl Default for PhaseUnwrapper {
    fn default() -> Self {
        Self::new()
    }
}

/// Compute instantaneous frequency from phase (finite difference).
///
/// `freq[n] = (phase[n] - phase[n-1]) / (2π * dt)` where `dt = 1/sample_rate`.
pub fn phase_to_freq(phase: &[f64], sample_rate: f64) -> Vec<f64> {
    if phase.len() < 2 {
        return Vec::new();
    }
    let dt = 1.0 / sample_rate;
    phase
        .windows(2)
        .map(|w| wrap_angle(w[1] - w[0]) / (TWO_PI * dt))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wrap_angle_in_range() {
        assert!((wrap_angle(0.0)).abs() < 1e-10);
        assert!((wrap_angle(1.0) - 1.0).abs() < 1e-10);
        assert!((wrap_angle(-1.0) + 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_wrap_angle_beyond_pi() {
        let w = wrap_angle(4.0);
        assert!(w >= -PI && w <= PI);
        // 4.0 - 2π ≈ -2.283
        assert!((w - (4.0 - TWO_PI)).abs() < 1e-10);
    }

    #[test]
    fn test_wrap_angle_below_neg_pi() {
        let w = wrap_angle(-4.0);
        assert!(w >= -PI && w <= PI);
        // -4.0 + 2π ≈ 2.283
        assert!((w - (-4.0 + TWO_PI)).abs() < 1e-10);
    }

    #[test]
    fn test_phase_wrap_vector() {
        let input = vec![0.0, PI, -PI, 4.0, -4.0];
        let wrapped = phase_wrap(&input);
        for &v in &wrapped {
            assert!(v >= -PI - 1e-10 && v <= PI + 1e-10);
        }
    }

    #[test]
    fn test_phase_unwrap_no_discontinuity() {
        let phase = vec![0.0, 0.5, 1.0, 1.5, 2.0];
        let unwrapped = phase_unwrap(&phase);
        assert_eq!(phase, unwrapped);
    }

    #[test]
    fn test_phase_unwrap_positive_jump() {
        // Phase jumps from ~3.0 to ~-3.0 (a positive 2π discontinuity in wrapped domain)
        let phase = vec![2.5, 3.0, -3.0, -2.5];
        let unwrapped = phase_unwrap(&phase);
        // After unwrap, should be approximately monotonic
        assert!((unwrapped[2] - (TWO_PI - 3.0)).abs() < 1e-10);
        assert!((unwrapped[3] - (TWO_PI - 2.5)).abs() < 1e-10);
    }

    #[test]
    fn test_phase_unwrap_negative_jump() {
        // Phase jumps from ~-3.0 to ~3.0 (a negative 2π discontinuity)
        let phase = vec![-2.5, -3.0, 3.0, 2.5];
        let unwrapped = phase_unwrap(&phase);
        assert!((unwrapped[2] - (3.0 - TWO_PI)).abs() < 1e-10);
    }

    #[test]
    fn test_phase_unwrap_streaming() {
        let phase = vec![2.5, 3.0, -3.0, -2.5];
        let batch_result = phase_unwrap(&phase);

        let mut unwrapper = PhaseUnwrapper::new();
        let stream_result: Vec<f64> = phase
            .iter()
            .flat_map(|&s| unwrapper.process(&[s]))
            .collect();

        for (a, b) in batch_result.iter().zip(stream_result.iter()) {
            assert!((a - b).abs() < 1e-10);
        }
    }

    #[test]
    fn test_phase_unwrap_empty() {
        assert!(phase_unwrap(&[]).is_empty());
    }

    #[test]
    fn test_phase_unwrap_single() {
        let result = phase_unwrap(&[1.5]);
        assert_eq!(result.len(), 1);
        assert!((result[0] - 1.5).abs() < 1e-10);
    }

    #[test]
    fn test_phase_to_freq() {
        // Constant phase increment = constant frequency
        let sample_rate = 1000.0;
        let freq_hz = 100.0;
        let phase_inc = TWO_PI * freq_hz / sample_rate;
        let phase: Vec<f64> = (0..100).map(|i| i as f64 * phase_inc).collect();
        let wrapped: Vec<f64> = phase.iter().map(|&p| wrap_angle(p)).collect();
        let unwrapped = phase_unwrap(&wrapped);
        let freq = phase_to_freq(&unwrapped, sample_rate);
        // All frequency estimates should be ~100 Hz
        for &f in &freq {
            assert!((f - freq_hz).abs() < 1.0);
        }
    }

    #[test]
    fn test_phase_unwrapper_reset() {
        let mut unwrapper = PhaseUnwrapper::new();
        unwrapper.process(&[1.0, 2.0]);
        unwrapper.reset();
        let result = unwrapper.process(&[1.0]);
        assert!((result[0] - 1.0).abs() < 1e-10);
    }
}
