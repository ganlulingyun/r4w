//! Adaptive Interference Cancellation
//!
//! This module provides adaptive interference cancellation algorithms for
//! suppressing narrowband and wideband interference in SDR receivers. It
//! includes LMS-based adaptive cancellers, narrowband notch filters, and
//! complex-valued wideband cancellers.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::adaptive_interference_canceller::{AdaptiveInterferenceCanceller, lms_canceller};
//!
//! // Create an LMS canceller with 32 taps
//! let mut canceller = lms_canceller(32);
//!
//! // Primary signal contains desired signal + interference
//! let primary: Vec<f64> = (0..128).map(|i| {
//!     let desired = (2.0 * std::f64::consts::PI * 0.01 * i as f64).sin();
//!     let interference = 5.0 * (2.0 * std::f64::consts::PI * 0.25 * i as f64).sin();
//!     desired + interference
//! }).collect();
//!
//! // Reference signal correlated with interference
//! let reference: Vec<f64> = (0..128).map(|i| {
//!     (2.0 * std::f64::consts::PI * 0.25 * i as f64).sin()
//! }).collect();
//!
//! let output = canceller.cancel(&primary, &reference);
//! assert_eq!(output.len(), 128);
//! ```

use std::f64::consts::PI;

/// Adaptive interference canceller using the Least Mean Squares (LMS) algorithm.
///
/// The LMS canceller estimates the interference component in the primary signal
/// by adaptively filtering a reference signal that is correlated with the
/// interference. The output is the primary signal minus the estimated interference.
pub struct AdaptiveInterferenceCanceller {
    weights: Vec<f64>,
    num_taps: usize,
    step_size: f64,
    buffer: Vec<f64>,
}

impl AdaptiveInterferenceCanceller {
    /// Create a new LMS-based adaptive interference canceller.
    ///
    /// # Arguments
    /// * `num_taps` - Number of filter taps (adaptive filter length)
    /// * `step_size` - LMS step size (mu), controls convergence speed vs stability
    pub fn new(num_taps: usize, step_size: f64) -> Self {
        Self {
            weights: vec![0.0; num_taps],
            num_taps,
            step_size,
            buffer: vec![0.0; num_taps],
        }
    }

    /// Cancel interference from the primary signal using an adaptive LMS filter.
    ///
    /// output\[n\] = primary\[n\] - W^T * reference_buffer\[n\]
    ///
    /// The weights are updated each sample using the LMS rule:
    /// W\[n+1\] = W\[n\] + mu * error\[n\] * reference_buffer\[n\]
    ///
    /// # Arguments
    /// * `primary` - Primary input containing desired signal + interference
    /// * `reference` - Reference input correlated with the interference
    ///
    /// # Returns
    /// Output signal with interference suppressed
    pub fn cancel(&mut self, primary: &[f64], reference: &[f64]) -> Vec<f64> {
        let len = primary.len().min(reference.len());
        let mut output = Vec::with_capacity(len);

        for i in 0..len {
            // Shift buffer and insert new reference sample
            for j in (1..self.num_taps).rev() {
                self.buffer[j] = self.buffer[j - 1];
            }
            self.buffer[0] = reference[i];

            // Compute filter output: y = W^T * x
            let y: f64 = self
                .weights
                .iter()
                .zip(self.buffer.iter())
                .map(|(w, x)| w * x)
                .sum();

            // Error signal = primary - estimated interference
            let error = primary[i] - y;
            output.push(error);

            // LMS weight update: W = W + mu * error * x
            for j in 0..self.num_taps {
                self.weights[j] += self.step_size * error * self.buffer[j];
            }
        }

        output
    }

    /// Return the current adaptive filter weights.
    pub fn weights(&self) -> &[f64] {
        &self.weights
    }

    /// Reset the canceller state (weights and buffer) to zero.
    pub fn reset(&mut self) {
        self.weights.iter_mut().for_each(|w| *w = 0.0);
        self.buffer.iter_mut().for_each(|b| *b = 0.0);
    }
}

/// A single second-order IIR notch filter section.
struct NotchSection {
    freq_hz: f64,
    /// Normalized frequency in radians
    omega: f64,
    /// Notch bandwidth parameter (pole radius)
    r: f64,
    // State variables for Direct Form II
    z1: f64,
    z2: f64,
}

impl NotchSection {
    fn new(freq_hz: f64, sample_rate: f64, bandwidth_hz: f64) -> Self {
        let omega = 2.0 * PI * freq_hz / sample_rate;
        // r controls the notch bandwidth: closer to 1.0 = narrower notch
        let r = 1.0 - PI * bandwidth_hz / sample_rate;
        let r = r.max(0.0).min(0.999);
        Self {
            freq_hz,
            omega,
            r,
            z1: 0.0,
            z2: 0.0,
        }
    }

    /// Process one sample through the notch filter.
    ///
    /// Transfer function: H(z) = (1 - 2*cos(omega)*z^-1 + z^-2) /
    ///                           (1 - 2*r*cos(omega)*z^-1 + r^2*z^-2)
    fn process_sample(&mut self, input: f64) -> f64 {
        let cos_omega = self.omega.cos();

        // Numerator coefficients (zeros on unit circle at notch frequency)
        let b0 = 1.0;
        let b1 = -2.0 * cos_omega;
        let b2 = 1.0;

        // Denominator coefficients (poles inside unit circle)
        let a1 = -2.0 * self.r * cos_omega;
        let a2 = self.r * self.r;

        // Direct Form II transposed
        let output = b0 * input + self.z1;
        self.z1 = b1 * input - a1 * output + self.z2;
        self.z2 = b2 * input - a2 * output;

        // Normalize gain so passband is ~unity
        let gain = (1.0 + self.r * self.r - 2.0 * self.r) / (2.0 - 2.0 * cos_omega);
        if gain.is_finite() && gain > 0.0 {
            output * gain.min(2.0)
        } else {
            output
        }
    }

    fn reset(&mut self) {
        self.z1 = 0.0;
        self.z2 = 0.0;
    }
}

/// Narrowband interference canceller using cascaded notch filters.
///
/// Removes narrowband (CW) interference at specified frequencies by placing
/// IIR notch filters at those frequencies.
pub struct NarrowbandCanceller {
    sections: Vec<NotchSection>,
    sample_rate: f64,
    bandwidth_hz: f64,
}

impl NarrowbandCanceller {
    /// Create a new narrowband canceller.
    ///
    /// # Arguments
    /// * `notch_frequencies` - Frequencies to notch out (Hz)
    /// * `sample_rate` - Signal sample rate (Hz)
    /// * `bandwidth_hz` - 3-dB bandwidth of each notch (Hz)
    pub fn new(notch_frequencies: &[f64], sample_rate: f64, bandwidth_hz: f64) -> Self {
        let sections = notch_frequencies
            .iter()
            .map(|&f| NotchSection::new(f, sample_rate, bandwidth_hz))
            .collect();
        Self {
            sections,
            sample_rate,
            bandwidth_hz,
        }
    }

    /// Process an input signal, removing narrowband interference.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output: Vec<f64> = input.to_vec();
        for section in &mut self.sections {
            section.reset();
            for sample in output.iter_mut() {
                *sample = section.process_sample(*sample);
            }
        }
        output
    }

    /// Add a notch filter at the specified frequency.
    pub fn add_notch(&mut self, freq_hz: f64) {
        self.sections.push(NotchSection::new(
            freq_hz,
            self.sample_rate,
            self.bandwidth_hz,
        ));
    }

    /// Remove the notch filter closest to the specified frequency.
    pub fn remove_notch(&mut self, freq_hz: f64) {
        if let Some(idx) = self
            .sections
            .iter()
            .position(|s| (s.freq_hz - freq_hz).abs() < 1.0)
        {
            self.sections.remove(idx);
        }
    }
}

/// Wideband interference canceller using complex-valued LMS.
///
/// Operates on complex (I/Q) samples represented as `(f64, f64)` tuples
/// where the first element is the real (I) component and the second is the
/// imaginary (Q) component.
pub struct WidebandCanceller {
    weights_re: Vec<f64>,
    weights_im: Vec<f64>,
    num_taps: usize,
    step_size: f64,
    buffer_re: Vec<f64>,
    buffer_im: Vec<f64>,
}

impl WidebandCanceller {
    /// Create a new complex-valued wideband LMS canceller.
    ///
    /// # Arguments
    /// * `num_taps` - Number of complex filter taps
    /// * `step_size` - LMS step size (mu)
    pub fn new(num_taps: usize, step_size: f64) -> Self {
        Self {
            weights_re: vec![0.0; num_taps],
            weights_im: vec![0.0; num_taps],
            num_taps,
            step_size,
            buffer_re: vec![0.0; num_taps],
            buffer_im: vec![0.0; num_taps],
        }
    }

    /// Cancel interference from complex primary signal using complex reference.
    ///
    /// Uses complex Normalized LMS (NLMS): output = primary - W^H * reference
    /// Weight update: W = W + (mu / (||x||^2 + eps)) * x * conj(error)
    ///
    /// The normalization ensures stability regardless of input signal power.
    pub fn cancel(
        &mut self,
        primary: &[(f64, f64)],
        reference: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        let len = primary.len().min(reference.len());
        let mut output = Vec::with_capacity(len);
        // Regularization prevents division by near-zero when the buffer is
        // not yet fully populated.
        let eps = 1.0;

        for i in 0..len {
            // Shift buffer
            for j in (1..self.num_taps).rev() {
                self.buffer_re[j] = self.buffer_re[j - 1];
                self.buffer_im[j] = self.buffer_im[j - 1];
            }
            self.buffer_re[0] = reference[i].0;
            self.buffer_im[0] = reference[i].1;

            // Complex dot product: y = W^H * x = sum(conj(w) * x)
            let mut y_re = 0.0;
            let mut y_im = 0.0;
            for j in 0..self.num_taps {
                // conj(w) * x = (wr - j*wi) * (xr + j*xi)
                //             = (wr*xr + wi*xi) + j*(wr*xi - wi*xr)
                y_re += self.weights_re[j] * self.buffer_re[j]
                    + self.weights_im[j] * self.buffer_im[j];
                y_im += self.weights_re[j] * self.buffer_im[j]
                    - self.weights_im[j] * self.buffer_re[j];
            }

            // Error
            let err_re = primary[i].0 - y_re;
            let err_im = primary[i].1 - y_im;
            output.push((err_re, err_im));

            // Compute buffer power for normalization: ||x||^2
            let buf_power: f64 = self
                .buffer_re
                .iter()
                .zip(self.buffer_im.iter())
                .map(|(r, i)| r * r + i * i)
                .sum();

            let norm_step = self.step_size / (buf_power + eps);

            // Complex NLMS update for y = W^H * x:
            // W = W + norm_step * x * conj(error)
            // x_k * conj(e) = (xr + j*xi) * (er - j*ei)
            //               = (xr*er + xi*ei) + j*(xi*er - xr*ei)
            for j in 0..self.num_taps {
                self.weights_re[j] += norm_step
                    * (self.buffer_re[j] * err_re + self.buffer_im[j] * err_im);
                self.weights_im[j] += norm_step
                    * (self.buffer_im[j] * err_re - self.buffer_re[j] * err_im);
            }
        }

        output
    }
}

/// Measure SINR improvement in dB between input and output signals.
///
/// Computes:
/// - Input SINR = signal_power / (input_noise_power) where input_noise_power = ||input - signal||^2
/// - Output SINR = signal_power / (output_noise_power) where output_noise_power = ||output - signal||^2
/// - Improvement = Output_SINR_dB - Input_SINR_dB
///
/// # Arguments
/// * `input` - Original input signal (signal + interference)
/// * `output` - Canceller output signal
/// * `signal` - Known desired signal (for measurement purposes)
///
/// # Returns
/// SINR improvement in dB (positive means improvement)
pub fn sinr_improvement(input: &[f64], output: &[f64], signal: &[f64]) -> f64 {
    let len = input.len().min(output.len()).min(signal.len());
    if len == 0 {
        return 0.0;
    }

    let signal_power: f64 = signal[..len].iter().map(|s| s * s).sum::<f64>() / len as f64;

    let input_noise_power: f64 = input[..len]
        .iter()
        .zip(signal[..len].iter())
        .map(|(i, s)| {
            let diff = i - s;
            diff * diff
        })
        .sum::<f64>()
        / len as f64;

    let output_noise_power: f64 = output[..len]
        .iter()
        .zip(signal[..len].iter())
        .map(|(o, s)| {
            let diff = o - s;
            diff * diff
        })
        .sum::<f64>()
        / len as f64;

    if input_noise_power <= 1e-30 || output_noise_power <= 1e-30 || signal_power <= 1e-30 {
        return 0.0;
    }

    let input_sinr_db = 10.0 * (signal_power / input_noise_power).log10();
    let output_sinr_db = 10.0 * (signal_power / output_noise_power).log10();

    output_sinr_db - input_sinr_db
}

/// Statistics from interference cancellation.
#[derive(Debug, Clone)]
pub struct CancellationStats {
    /// Residual power of the output signal in dB
    pub residual_power_db: f64,
    /// Cancellation depth: ratio of input power to output power in dB
    pub cancellation_depth_db: f64,
    /// Convergence speed: number of samples until the canceller has settled
    /// (measured as the point where running average error stabilizes)
    pub convergence_speed: usize,
}

/// Measure cancellation performance between input and output signals.
///
/// # Arguments
/// * `input` - Original input signal
/// * `output` - Output signal after cancellation
///
/// # Returns
/// `CancellationStats` with residual power, cancellation depth, and convergence speed
pub fn measure_cancellation(input: &[f64], output: &[f64]) -> CancellationStats {
    let len = input.len().min(output.len());
    if len == 0 {
        return CancellationStats {
            residual_power_db: f64::NEG_INFINITY,
            cancellation_depth_db: 0.0,
            convergence_speed: 0,
        };
    }

    let input_power: f64 = input[..len].iter().map(|x| x * x).sum::<f64>() / len as f64;
    let output_power: f64 = output[..len].iter().map(|x| x * x).sum::<f64>() / len as f64;

    let residual_power_db = if output_power > 1e-30 {
        10.0 * output_power.log10()
    } else {
        f64::NEG_INFINITY
    };

    let cancellation_depth_db = if output_power > 1e-30 && input_power > 1e-30 {
        10.0 * (input_power / output_power).log10()
    } else {
        0.0
    };

    // Estimate convergence speed by looking at the running power of the output.
    // We use a sliding window and detect when the output power stabilizes.
    let window = 32.min(len / 4).max(1);
    let mut converged_at = len;
    if len > window * 2 {
        // Compute final (steady-state) power over the last window
        let final_power: f64 = output[len - window..len]
            .iter()
            .map(|x| x * x)
            .sum::<f64>()
            / window as f64;

        // Walk from the beginning and find where running power is within 3 dB of final
        let threshold = final_power * 2.0; // ~3 dB above steady state
        for i in 0..len.saturating_sub(window) {
            let window_power: f64 = output[i..i + window]
                .iter()
                .map(|x| x * x)
                .sum::<f64>()
                / window as f64;
            if window_power <= threshold || window_power <= final_power * 1.1 {
                converged_at = i + window;
                break;
            }
        }
    }

    CancellationStats {
        residual_power_db,
        cancellation_depth_db,
        convergence_speed: converged_at,
    }
}

/// Factory function: create an LMS canceller with a typical step size.
///
/// Uses step_size = 0.01, which provides a good balance between convergence
/// speed and steady-state error for most interference scenarios.
///
/// # Arguments
/// * `num_taps` - Number of adaptive filter taps
pub fn lms_canceller(num_taps: usize) -> AdaptiveInterferenceCanceller {
    AdaptiveInterferenceCanceller::new(num_taps, 0.01)
}

/// Factory function: create a narrowband canceller for CW interference.
///
/// Uses a 50 Hz notch bandwidth, suitable for removing a single-tone
/// (continuous wave) interferer.
///
/// # Arguments
/// * `freq_hz` - Frequency of the CW interference (Hz)
/// * `sample_rate` - Signal sample rate (Hz)
pub fn cw_interference_canceller(freq_hz: f64, sample_rate: f64) -> NarrowbandCanceller {
    NarrowbandCanceller::new(&[freq_hz], sample_rate, 50.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Test that the LMS canceller reduces correlated interference.
    #[test]
    fn test_lms_reduces_correlated_interference() {
        let n = 2000;
        let mut canceller = AdaptiveInterferenceCanceller::new(16, 0.005);

        // Desired signal: low-frequency sine
        let desired: Vec<f64> = (0..n)
            .map(|i| 0.5 * (2.0 * PI * 0.01 * i as f64).sin())
            .collect();

        // Interference: higher-frequency sine
        let interference: Vec<f64> = (0..n)
            .map(|i| 3.0 * (2.0 * PI * 0.15 * i as f64).sin())
            .collect();

        // Primary = desired + interference
        let primary: Vec<f64> = desired
            .iter()
            .zip(interference.iter())
            .map(|(d, i)| d + i)
            .collect();

        // Reference is correlated with interference (same frequency, slight phase shift)
        let reference: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 0.15 * i as f64 + 0.3).sin())
            .collect();

        let output = canceller.cancel(&primary, &reference);

        // Measure interference power in the second half (after convergence)
        let half = n / 2;
        let input_interference_power: f64 = primary[half..]
            .iter()
            .zip(desired[half..].iter())
            .map(|(p, d)| (p - d).powi(2))
            .sum::<f64>()
            / (n - half) as f64;

        let output_interference_power: f64 = output[half..]
            .iter()
            .zip(desired[half..].iter())
            .map(|(o, d)| (o - d).powi(2))
            .sum::<f64>()
            / (n - half) as f64;

        // The canceller should reduce interference significantly
        assert!(
            output_interference_power < input_interference_power * 0.3,
            "Interference not sufficiently reduced: input={:.4}, output={:.4}",
            input_interference_power,
            output_interference_power
        );
    }

    /// Test that LMS weights converge toward expected values.
    #[test]
    fn test_weights_converge() {
        let n = 5000;
        // Simple case: primary = 2.0 * reference + noise
        // The weight should converge to approximately 2.0
        let mut canceller = AdaptiveInterferenceCanceller::new(1, 0.01);

        let reference: Vec<f64> = (0..n).map(|i| (2.0 * PI * 0.05 * i as f64).sin()).collect();

        let primary: Vec<f64> = reference.iter().map(|r| 2.0 * r).collect();

        let _output = canceller.cancel(&primary, &reference);

        let weights = canceller.weights();
        assert!(
            (weights[0] - 2.0).abs() < 0.2,
            "Weight should converge near 2.0, got {:.4}",
            weights[0]
        );
    }

    /// Test that reset clears all state.
    #[test]
    fn test_reset_clears_state() {
        let mut canceller = AdaptiveInterferenceCanceller::new(8, 0.01);

        // Run some data through
        let primary: Vec<f64> = (0..100).map(|i| (i as f64) * 0.1).collect();
        let reference: Vec<f64> = (0..100).map(|i| (i as f64) * 0.05).collect();
        let _output = canceller.cancel(&primary, &reference);

        // Weights should be non-zero now
        assert!(canceller.weights().iter().any(|w| w.abs() > 1e-10));

        // Reset
        canceller.reset();

        // All weights should be zero
        assert!(canceller.weights().iter().all(|w| *w == 0.0));
    }

    /// Test that narrowband canceller removes CW interference.
    #[test]
    fn test_narrowband_removes_cw() {
        let sample_rate = 8000.0;
        let n = 4000;
        let cw_freq = 1000.0;

        let mut canceller = NarrowbandCanceller::new(&[cw_freq], sample_rate, 100.0);

        // Signal: low-frequency desired + CW interference
        let input: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let desired = (2.0 * PI * 200.0 * t).sin();
                let cw = 5.0 * (2.0 * PI * cw_freq * t).sin();
                desired + cw
            })
            .collect();

        let output = canceller.process(&input);

        // Measure CW power using Goertzel-like approach
        // (correlate with the CW frequency)
        let half = n / 2; // skip transient
        let cw_corr_input: f64 = input[half..]
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let t = (i + half) as f64 / sample_rate;
                s * (2.0 * PI * cw_freq * t).sin()
            })
            .sum::<f64>()
            .abs()
            / (n - half) as f64;

        let cw_corr_output: f64 = output[half..]
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let t = (i + half) as f64 / sample_rate;
                s * (2.0 * PI * cw_freq * t).sin()
            })
            .sum::<f64>()
            .abs()
            / (n - half) as f64;

        assert!(
            cw_corr_output < cw_corr_input * 0.3,
            "CW not sufficiently attenuated: input_corr={:.4}, output_corr={:.4}",
            cw_corr_input,
            cw_corr_output
        );
    }

    /// Test add_notch and remove_notch functionality.
    #[test]
    fn test_add_remove_notch() {
        let mut canceller = NarrowbandCanceller::new(&[1000.0], 8000.0, 50.0);
        assert_eq!(canceller.sections.len(), 1);

        canceller.add_notch(2000.0);
        assert_eq!(canceller.sections.len(), 2);

        canceller.add_notch(3000.0);
        assert_eq!(canceller.sections.len(), 3);

        canceller.remove_notch(2000.0);
        assert_eq!(canceller.sections.len(), 2);

        // Verify remaining frequencies
        let freqs: Vec<f64> = canceller.sections.iter().map(|s| s.freq_hz).collect();
        assert!(freqs.contains(&1000.0));
        assert!(freqs.contains(&3000.0));
        assert!(!freqs.contains(&2000.0));

        // Remove non-existent frequency should do nothing
        canceller.remove_notch(5000.0);
        assert_eq!(canceller.sections.len(), 2);
    }

    /// Test wideband complex canceller.
    #[test]
    fn test_wideband_complex_canceller() {
        let n = 8000;
        // For NLMS, step_size (mu) should be 0 < mu < 2
        let mut canceller = WidebandCanceller::new(4, 0.3);

        // Complex interference at a specific frequency
        let interference: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * 0.1 * i as f64;
                (2.0 * phase.cos(), 2.0 * phase.sin())
            })
            .collect();

        // Desired signal at different frequency
        let desired: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * 0.01 * i as f64;
                (0.3 * phase.cos(), 0.3 * phase.sin())
            })
            .collect();

        // Primary = desired + interference
        let primary: Vec<(f64, f64)> = desired
            .iter()
            .zip(interference.iter())
            .map(|(d, i)| (d.0 + i.0, d.1 + i.1))
            .collect();

        // Reference correlated with interference (same frequency, slight phase offset)
        let reference: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * 0.1 * i as f64 + 0.2;
                (phase.cos(), phase.sin())
            })
            .collect();

        let output = canceller.cancel(&primary, &reference);

        // Check that output power is less than primary power in the last quarter
        // (after convergence). We compare total output power to total input power.
        let quarter = 3 * n / 4;
        let input_total_power: f64 = primary[quarter..]
            .iter()
            .map(|(r, i)| r * r + i * i)
            .sum::<f64>()
            / (n - quarter) as f64;

        let output_total_power: f64 = output[quarter..]
            .iter()
            .map(|(r, i)| r * r + i * i)
            .sum::<f64>()
            / (n - quarter) as f64;

        // The canceller should reduce the total power (since it removes the
        // large interference component)
        assert!(
            output_total_power < input_total_power * 0.8,
            "Complex interference not reduced: input_power={:.4}, output_power={:.4}",
            input_total_power,
            output_total_power
        );
    }

    /// Test SINR improvement measurement.
    #[test]
    fn test_sinr_improvement() {
        let n = 1000;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 0.02 * i as f64).sin())
            .collect();

        // Input has large interference
        let interference: Vec<f64> = (0..n)
            .map(|i| 5.0 * (2.0 * PI * 0.2 * i as f64).sin())
            .collect();

        let input: Vec<f64> = signal
            .iter()
            .zip(interference.iter())
            .map(|(s, i)| s + i)
            .collect();

        // Output has reduced interference
        let reduced_interference: Vec<f64> = interference.iter().map(|i| i * 0.1).collect();
        let output: Vec<f64> = signal
            .iter()
            .zip(reduced_interference.iter())
            .map(|(s, i)| s + i)
            .collect();

        let improvement = sinr_improvement(&input, &output, &signal);

        // We reduced interference by 10x (20 dB power), so improvement should be ~20 dB
        assert!(
            improvement > 15.0,
            "SINR improvement should be >15 dB, got {:.2} dB",
            improvement
        );
        assert!(
            improvement < 25.0,
            "SINR improvement should be <25 dB, got {:.2} dB",
            improvement
        );
    }

    /// Test cancellation stats measurement.
    #[test]
    fn test_cancellation_stats() {
        let n = 2000;

        // Input with interference
        let input: Vec<f64> = (0..n)
            .map(|i| 5.0 * (2.0 * PI * 0.1 * i as f64).sin())
            .collect();

        // Output with reduced interference (simulate convergence)
        let output: Vec<f64> = (0..n)
            .map(|i| {
                let decay = (-(i as f64) / 200.0).exp();
                let residual = 0.2;
                let amplitude = 5.0 * decay + residual;
                amplitude * (2.0 * PI * 0.1 * i as f64).sin()
            })
            .collect();

        let stats = measure_cancellation(&input, &output);

        // Cancellation depth should be positive (input power > output power)
        assert!(
            stats.cancellation_depth_db > 0.0,
            "Cancellation depth should be positive, got {:.2} dB",
            stats.cancellation_depth_db
        );

        // Residual power should be finite
        assert!(
            stats.residual_power_db.is_finite(),
            "Residual power should be finite"
        );

        // Convergence speed should be reasonable (less than full length)
        assert!(
            stats.convergence_speed < n,
            "Convergence speed {} should be less than signal length {}",
            stats.convergence_speed,
            n
        );
    }

    /// Test LMS factory function.
    #[test]
    fn test_lms_factory() {
        let canceller = lms_canceller(32);
        assert_eq!(canceller.weights().len(), 32);
        assert_eq!(canceller.num_taps, 32);
        assert_eq!(canceller.step_size, 0.01);
        assert!(canceller.weights().iter().all(|w| *w == 0.0));
    }

    /// Test CW interference factory function.
    #[test]
    fn test_cw_factory() {
        let canceller = cw_interference_canceller(1500.0, 8000.0);
        assert_eq!(canceller.sections.len(), 1);
        assert_eq!(canceller.sample_rate, 8000.0);
        assert_eq!(canceller.bandwidth_hz, 50.0);
        assert!((canceller.sections[0].freq_hz - 1500.0).abs() < 1e-10);

        // Verify it can process data without panicking
        let input: Vec<f64> = (0..100).map(|i| (i as f64 * 0.1).sin()).collect();
        let output = canceller.clone_and_process(&input);
        assert_eq!(output.len(), 100);
    }
}

// Helper for testing the CW factory (avoids needing Clone on NarrowbandCanceller)
impl NarrowbandCanceller {
    #[cfg(test)]
    fn clone_and_process(&self, input: &[f64]) -> Vec<f64> {
        let mut canceller =
            NarrowbandCanceller::new(&self.freqs_list(), self.sample_rate, self.bandwidth_hz);
        canceller.process(input)
    }

    #[cfg(test)]
    fn freqs_list(&self) -> Vec<f64> {
        self.sections.iter().map(|s| s.freq_hz).collect()
    }
}
