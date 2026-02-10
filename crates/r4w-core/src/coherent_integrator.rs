//! Coherent and Non-Coherent Integration
//!
//! Pulse integration techniques for radar and signal detection applications.
//! Coherent integration preserves phase information and sums complex samples
//! across pulses, yielding an N^2 SNR gain in power (N in amplitude).
//! Non-coherent integration sums magnitudes (squared), discarding phase,
//! and provides approximately N times SNR gain with some integration loss.
//!
//! Also includes binary (M-of-N) integration for detection fusion and
//! sliding-window non-coherent integration for continuous processing.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::coherent_integrator::{coherent_integrate, non_coherent_integrate};
//!
//! // Three identical pulses with a target at bin 0
//! let pulse = vec![(1.0, 0.0), (0.0, 0.0), (0.0, 0.0)];
//! let pulses = vec![pulse.clone(), pulse.clone(), pulse.clone()];
//!
//! // Coherent: complex sum preserves phase, amplitude = 3.0
//! let result = coherent_integrate(&pulses);
//! assert!((result[0].0 - 3.0).abs() < 1e-12);
//!
//! // Non-coherent: sum of magnitudes squared = 3.0
//! let nc_result = non_coherent_integrate(&pulses);
//! assert!((nc_result[0] - 3.0).abs() < 1e-12);
//! ```

/// Coherent integration: complex (vector) sum across pulses.
///
/// Each pulse is a slice of `(re, im)` pairs. All pulses must have the same
/// length. The output preserves phase, so aligned targets add constructively
/// and yield N^2 power gain (N amplitude gain) for N pulses.
///
/// Returns an empty vector if `pulses` is empty or the first pulse is empty.
pub fn coherent_integrate(pulses: &[Vec<(f64, f64)>]) -> Vec<(f64, f64)> {
    if pulses.is_empty() {
        return Vec::new();
    }
    let len = pulses[0].len();
    if len == 0 {
        return Vec::new();
    }
    let mut result = vec![(0.0_f64, 0.0_f64); len];
    for pulse in pulses {
        for (i, &(re, im)) in pulse.iter().enumerate().take(len) {
            result[i].0 += re;
            result[i].1 += im;
        }
    }
    result
}

/// Non-coherent integration: sum of magnitudes squared across pulses.
///
/// For each range/frequency bin, computes the sum of |sample|^2 over all
/// pulses. Phase information is discarded. Provides approximately N times
/// SNR gain (in power) with some integration loss compared to coherent.
///
/// Returns an empty vector if `pulses` is empty or the first pulse is empty.
pub fn non_coherent_integrate(pulses: &[Vec<(f64, f64)>]) -> Vec<f64> {
    if pulses.is_empty() {
        return Vec::new();
    }
    let len = pulses[0].len();
    if len == 0 {
        return Vec::new();
    }
    let mut result = vec![0.0_f64; len];
    for pulse in pulses {
        for (i, &(re, im)) in pulse.iter().enumerate().take(len) {
            result[i] += re * re + im * im;
        }
    }
    result
}

/// Streaming coherent integrator.
///
/// Accumulates complex pulses and returns the coherently integrated result
/// once `num_pulses` have been collected, then resets for the next batch.
#[derive(Debug, Clone)]
pub struct CoherentIntegrator {
    /// Number of pulses to integrate before producing output.
    pub num_pulses: usize,
    /// Internal pulse buffer.
    buffer: Vec<Vec<(f64, f64)>>,
    /// Number of pulses added so far in the current integration period.
    count: usize,
}

impl CoherentIntegrator {
    /// Create a new coherent integrator that accumulates `num_pulses` before
    /// producing a result.
    ///
    /// # Panics
    ///
    /// Panics if `num_pulses` is zero.
    pub fn new(num_pulses: usize) -> Self {
        assert!(num_pulses > 0, "num_pulses must be > 0");
        Self {
            num_pulses,
            buffer: Vec::with_capacity(num_pulses),
            count: 0,
        }
    }

    /// Add a pulse to the integration buffer.
    ///
    /// Returns `Some(integrated_result)` when the buffer is full (i.e. after
    /// `num_pulses` have been added), then resets the internal state.
    /// Returns `None` otherwise.
    pub fn add_pulse(&mut self, pulse: Vec<(f64, f64)>) -> Option<Vec<(f64, f64)>> {
        self.buffer.push(pulse);
        self.count += 1;
        if self.count >= self.num_pulses {
            let result = coherent_integrate(&self.buffer);
            self.buffer.clear();
            self.count = 0;
            Some(result)
        } else {
            None
        }
    }
}

/// Streaming non-coherent integrator.
///
/// Accumulates pulses and returns the non-coherently integrated result
/// (sum of magnitudes squared) once `num_pulses` have been collected,
/// then resets for the next batch.
#[derive(Debug, Clone)]
pub struct NonCoherentIntegrator {
    /// Number of pulses to integrate before producing output.
    pub num_pulses: usize,
    /// Internal pulse buffer.
    buffer: Vec<Vec<(f64, f64)>>,
    /// Number of pulses added so far in the current integration period.
    count: usize,
}

impl NonCoherentIntegrator {
    /// Create a new non-coherent integrator that accumulates `num_pulses`
    /// before producing a result.
    ///
    /// # Panics
    ///
    /// Panics if `num_pulses` is zero.
    pub fn new(num_pulses: usize) -> Self {
        assert!(num_pulses > 0, "num_pulses must be > 0");
        Self {
            num_pulses,
            buffer: Vec::with_capacity(num_pulses),
            count: 0,
        }
    }

    /// Add a pulse to the integration buffer.
    ///
    /// Returns `Some(integrated_magnitudes)` when the buffer is full,
    /// then resets internal state. Returns `None` otherwise.
    pub fn add_pulse(&mut self, pulse: Vec<(f64, f64)>) -> Option<Vec<f64>> {
        self.buffer.push(pulse);
        self.count += 1;
        if self.count >= self.num_pulses {
            let result = non_coherent_integrate(&self.buffer);
            self.buffer.clear();
            self.count = 0;
            Some(result)
        } else {
            None
        }
    }
}

/// Compute coherent integration gain in dB.
///
/// For coherent integration of N pulses, the SNR improvement in power is
/// N^2 (amplitude adds coherently, noise adds incoherently). Expressed in dB:
///
///   gain = 20 * log10(N)  (power gain = N^2 => 10*log10(N^2) = 20*log10(N))
///
/// However, when measuring SNR gain (signal power / noise power), the
/// conventional result is 10*log10(N) because both signal power and noise
/// power scale, but signal adds coherently (N^2) while noise adds as N,
/// giving net SNR gain = N, i.e. 10*log10(N) dB.
///
/// This function returns the SNR gain: `10 * log10(N)` dB.
pub fn integration_gain_coherent(num_pulses: usize) -> f64 {
    if num_pulses == 0 {
        return 0.0;
    }
    10.0 * (num_pulses as f64).log10()
}

/// Compute non-coherent integration gain in dB (approximate).
///
/// Non-coherent integration of N pulses provides less gain than coherent
/// due to integration loss. Albersheim's approximation for the loss factor
/// gives roughly:
///
///   gain ~ 10 * log10(N) - loss
///
/// where loss = 10 * log10(1 + 1/sqrt(N)) accounts for the squaring loss.
/// For large N the loss tends to zero and non-coherent approaches coherent.
pub fn integration_gain_non_coherent(num_pulses: usize) -> f64 {
    if num_pulses == 0 {
        return 0.0;
    }
    let n = num_pulses as f64;
    let ideal = 10.0 * n.log10();
    let loss = 10.0 * (1.0 + 1.0 / n.sqrt()).log10();
    ideal - loss
}

/// Binary integration (M-of-N detector).
///
/// Given a set of per-pulse binary detection vectors, declares a detection
/// at each cell if at least `threshold` out of N pulses reported a detection
/// there. This provides robustness against individual-pulse false alarms.
///
/// All detection vectors must have the same length. Returns an empty vector
/// if `detections` is empty.
pub fn binary_integration(detections: &[Vec<bool>], threshold: usize) -> Vec<bool> {
    if detections.is_empty() {
        return Vec::new();
    }
    let len = detections[0].len();
    let mut counts = vec![0_usize; len];
    for det in detections {
        for (i, &d) in det.iter().enumerate().take(len) {
            if d {
                counts[i] += 1;
            }
        }
    }
    counts.iter().map(|&c| c >= threshold).collect()
}

/// Sliding-window non-coherent integration.
///
/// Applies a sliding window of `window_size` samples over a contiguous
/// complex signal and computes the sum of magnitudes squared within each
/// window position. Useful for continuous energy detection.
///
/// Returns a vector of length `signal.len() - window_size + 1`.
/// Returns an empty vector if `window_size` is zero or exceeds `signal.len()`.
pub fn sliding_window_integrate(signal: &[(f64, f64)], window_size: usize) -> Vec<f64> {
    if window_size == 0 || window_size > signal.len() {
        return Vec::new();
    }

    let n = signal.len() - window_size + 1;
    let mut result = Vec::with_capacity(n);

    // Compute the first window sum
    let mut sum: f64 = signal[..window_size]
        .iter()
        .map(|&(re, im)| re * re + im * im)
        .sum();
    result.push(sum);

    // Slide the window
    for i in 1..n {
        let (re_old, im_old) = signal[i - 1];
        sum -= re_old * re_old + im_old * im_old;
        let (re_new, im_new) = signal[i + window_size - 1];
        sum += re_new * re_new + im_new * im_new;
        result.push(sum);
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_coherent_basic() {
        // Two pulses: (1,0) and (1,0) should sum to (2,0)
        let p1 = vec![(1.0, 0.0), (0.5, 0.5)];
        let p2 = vec![(1.0, 0.0), (-0.5, 0.5)];
        let result = coherent_integrate(&[p1, p2]);
        assert_eq!(result.len(), 2);
        assert!((result[0].0 - 2.0).abs() < 1e-12);
        assert!((result[0].1 - 0.0).abs() < 1e-12);
        assert!((result[1].0 - 0.0).abs() < 1e-12);
        assert!((result[1].1 - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_non_coherent_basic() {
        // Two pulses, sum of magnitudes squared
        let p1 = vec![(1.0, 0.0), (0.0, 1.0)];
        let p2 = vec![(0.0, 1.0), (1.0, 0.0)];
        let result = non_coherent_integrate(&[p1, p2]);
        assert_eq!(result.len(), 2);
        // bin 0: 1^2 + 1^2 = 2.0
        assert!((result[0] - 2.0).abs() < 1e-12);
        // bin 1: 1^2 + 1^2 = 2.0
        assert!((result[1] - 2.0).abs() < 1e-12);
    }

    #[test]
    fn test_coherent_snr_gain() {
        // N identical pulses with amplitude A: coherent sum = N*A
        // Power = (N*A)^2 = N^2 * A^2 => N^2 power gain
        // SNR gain = N (noise adds incoherently)
        let n = 16_usize;
        let amplitude = 0.5_f64;
        let pulse = vec![(amplitude, 0.0)];
        let pulses: Vec<Vec<(f64, f64)>> = (0..n).map(|_| pulse.clone()).collect();
        let result = coherent_integrate(&pulses);
        let expected_amplitude = n as f64 * amplitude;
        assert!(
            (result[0].0 - expected_amplitude).abs() < 1e-12,
            "Coherent sum amplitude should be N * A"
        );

        // Verify gain formula
        let gain = integration_gain_coherent(n);
        let expected_gain = 10.0 * (n as f64).log10();
        assert!((gain - expected_gain).abs() < 1e-12);
    }

    #[test]
    fn test_non_coherent_snr_gain() {
        // N pulses with amplitude A: each |s|^2 = A^2, sum = N * A^2
        let n = 10_usize;
        let amplitude = 2.0_f64;
        let pulse = vec![(amplitude, 0.0)];
        let pulses: Vec<Vec<(f64, f64)>> = (0..n).map(|_| pulse.clone()).collect();
        let result = non_coherent_integrate(&pulses);
        let expected = n as f64 * amplitude * amplitude;
        assert!(
            (result[0] - expected).abs() < 1e-12,
            "Non-coherent sum should be N * A^2"
        );

        // Non-coherent gain should be less than coherent gain
        let nc_gain = integration_gain_non_coherent(n);
        let c_gain = integration_gain_coherent(n);
        assert!(
            nc_gain < c_gain,
            "Non-coherent gain ({nc_gain}) should be less than coherent ({c_gain})"
        );
        assert!(nc_gain > 0.0, "Non-coherent gain should be positive");
    }

    #[test]
    fn test_integrator_struct() {
        let mut integrator = CoherentIntegrator::new(3);

        // First two pulses: no output
        assert!(integrator.add_pulse(vec![(1.0, 0.0)]).is_none());
        assert!(integrator.add_pulse(vec![(1.0, 0.0)]).is_none());

        // Third pulse: output
        let result = integrator.add_pulse(vec![(1.0, 0.0)]);
        assert!(result.is_some());
        let r = result.unwrap();
        assert!((r[0].0 - 3.0).abs() < 1e-12);

        // After output, counter resets — next pulse should not produce output
        assert!(integrator.add_pulse(vec![(2.0, 0.0)]).is_none());
    }

    #[test]
    fn test_non_coherent_struct() {
        let mut integrator = NonCoherentIntegrator::new(2);

        // First pulse: no output
        assert!(integrator.add_pulse(vec![(3.0, 4.0)]).is_none());

        // Second pulse: output
        let result = integrator.add_pulse(vec![(0.0, 5.0)]);
        assert!(result.is_some());
        let r = result.unwrap();
        // bin 0: (3^2+4^2) + (0^2+5^2) = 25 + 25 = 50
        assert!((r[0] - 50.0).abs() < 1e-12);

        // Resets after output
        assert!(integrator.add_pulse(vec![(1.0, 0.0)]).is_none());
    }

    #[test]
    fn test_binary_integration() {
        let d1 = vec![true, false, true, true, false];
        let d2 = vec![true, true, false, true, false];
        let d3 = vec![false, true, true, false, false];

        // M-of-N with threshold=2 (2 of 3)
        let result = binary_integration(&[d1, d2, d3], 2);
        assert_eq!(result, vec![true, true, true, true, false]);

        // Threshold=3 (all must agree)
        let d1 = vec![true, true, false];
        let d2 = vec![true, false, false];
        let d3 = vec![true, true, false];
        let result = binary_integration(&[d1, d2, d3], 3);
        assert_eq!(result, vec![true, false, false]);
    }

    #[test]
    fn test_sliding_window() {
        // Simple signal: magnitude pattern is easy to verify
        let signal = vec![
            (1.0, 0.0),
            (2.0, 0.0),
            (3.0, 0.0),
            (4.0, 0.0),
            (5.0, 0.0),
        ];
        let result = sliding_window_integrate(&signal, 3);
        assert_eq!(result.len(), 3);
        // window[0]: 1^2 + 2^2 + 3^2 = 1 + 4 + 9 = 14
        assert!((result[0] - 14.0).abs() < 1e-12);
        // window[1]: 2^2 + 3^2 + 4^2 = 4 + 9 + 16 = 29
        assert!((result[1] - 29.0).abs() < 1e-12);
        // window[2]: 3^2 + 4^2 + 5^2 = 9 + 16 + 25 = 50
        assert!((result[2] - 50.0).abs() < 1e-12);
    }

    #[test]
    fn test_single_pulse() {
        // Single pulse coherent integration should return the pulse itself
        let pulse = vec![(1.0, 2.0), (3.0, 4.0)];
        let result = coherent_integrate(&[pulse.clone()]);
        assert_eq!(result.len(), 2);
        assert!((result[0].0 - 1.0).abs() < 1e-12);
        assert!((result[0].1 - 2.0).abs() < 1e-12);
        assert!((result[1].0 - 3.0).abs() < 1e-12);
        assert!((result[1].1 - 4.0).abs() < 1e-12);

        // Single pulse non-coherent should return magnitudes squared
        let nc = non_coherent_integrate(&[pulse]);
        assert!((nc[0] - 5.0).abs() < 1e-12); // 1^2 + 2^2 = 5
        assert!((nc[1] - 25.0).abs() < 1e-12); // 3^2 + 4^2 = 25
    }

    #[test]
    fn test_empty_input() {
        // Empty slices should return empty results
        let empty: Vec<Vec<(f64, f64)>> = vec![];
        assert!(coherent_integrate(&empty).is_empty());
        assert!(non_coherent_integrate(&empty).is_empty());

        // Empty inner vectors
        let empty_inner = vec![vec![]];
        assert!(coherent_integrate(&empty_inner).is_empty());
        assert!(non_coherent_integrate(&empty_inner).is_empty());

        // Binary integration with empty
        let empty_det: Vec<Vec<bool>> = vec![];
        assert!(binary_integration(&empty_det, 1).is_empty());

        // Sliding window edge cases
        assert!(sliding_window_integrate(&[], 1).is_empty());
        assert!(sliding_window_integrate(&[(1.0, 0.0)], 0).is_empty());
        assert!(sliding_window_integrate(&[(1.0, 0.0)], 2).is_empty());
    }
}
