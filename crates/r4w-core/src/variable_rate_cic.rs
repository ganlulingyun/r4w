//! Variable-Rate CIC (Cascaded Integrator-Comb) Filter
//!
//! Provides CIC decimation and interpolation filters whose rate can be changed
//! at runtime without reconstructing the filter. This is useful in applications
//! such as adaptive sample rate conversion, multi-standard receivers, and
//! cognitive radio systems where the decimation/interpolation ratio must be
//! adjusted dynamically based on signal conditions or channel bandwidth.
//!
//! A CIC filter consists of N cascaded integrator-comb pairs. The transfer
//! function is:
//!
//! ```text
//! H(z) = [(1 - z^(-R)) / (1 - z^(-1))]^N
//! ```
//!
//! where R is the rate change factor and N is the filter order (number of
//! stages). The filter gain is R^N.
//!
//! This module also provides a [`CicCompensator`] that applies an inverse-sinc
//! FIR filter to correct the passband droop inherent in CIC filters.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::variable_rate_cic::{VariableRateCicDecimator, VariableRateCicInterpolator};
//!
//! // Create a 3rd-order CIC decimator with rate 4
//! let mut dec = VariableRateCicDecimator::new(3, 4);
//! assert_eq!(dec.rate(), 4);
//! assert_eq!(dec.gain(), 64.0); // 4^3
//!
//! let input = vec![1.0_f64; 40];
//! let output = dec.process(&input);
//! assert_eq!(output.len(), 10); // 40 / 4
//!
//! // Change rate at runtime
//! dec.set_rate(2);
//! assert_eq!(dec.rate(), 2);
//! let output2 = dec.process(&vec![1.0; 20]);
//! assert_eq!(output2.len(), 10); // 20 / 2
//! ```

use std::collections::VecDeque;

/// Variable-rate CIC decimation filter.
///
/// Implements an N-th order CIC decimator whose decimation rate can be changed
/// at runtime via [`set_rate`](VariableRateCicDecimator::set_rate). Internally
/// uses f64 accumulators for the integrator stages and `VecDeque<f64>` delay
/// lines for the comb stages.
///
/// The processing pipeline is:
/// ```text
/// Input → [Integrator 1] → ... → [Integrator N] → ↓R → [Comb 1] → ... → [Comb N] → Output
/// ```
#[derive(Debug, Clone)]
pub struct VariableRateCicDecimator {
    /// Filter order (number of integrator-comb stages)
    order: usize,
    /// Current decimation rate
    rate: usize,
    /// Integrator accumulators, one per stage
    integrators: Vec<f64>,
    /// Comb delay lines, one VecDeque per stage (depth = 1 for differential delay)
    combs: Vec<VecDeque<f64>>,
    /// Running count of input samples for decimation timing
    sample_count: usize,
}

impl VariableRateCicDecimator {
    /// Create a new variable-rate CIC decimator.
    ///
    /// - `order`: number of integrator-comb stages (N). Clamped to at least 1.
    /// - `rate`: initial decimation factor (R). Clamped to at least 1.
    pub fn new(order: usize, rate: usize) -> Self {
        let order = order.max(1);
        let rate = rate.max(1);
        Self {
            order,
            rate,
            integrators: vec![0.0; order],
            combs: (0..order).map(|_| {
                let mut d = VecDeque::with_capacity(1);
                d.push_back(0.0);
                d
            }).collect(),
            sample_count: 0,
        }
    }

    /// Process a block of input samples, producing decimated output.
    ///
    /// For every `rate` input samples consumed, one output sample is produced.
    /// The output length is `input.len() / rate` (integer division). Any
    /// leftover samples that do not complete a full decimation period are
    /// retained in the internal state for the next call.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() / self.rate + 1);

        for &x in input {
            // Integrator stages (run at input rate)
            let mut val = x;
            for i in 0..self.order {
                self.integrators[i] += val;
                val = self.integrators[i];
            }

            self.sample_count += 1;
            if self.sample_count >= self.rate {
                self.sample_count = 0;

                // Comb stages (run at output rate)
                let mut comb_val = val;
                for i in 0..self.order {
                    let delayed = self.combs[i][0];
                    self.combs[i][0] = comb_val;
                    comb_val -= delayed;
                }

                // Normalize by CIC gain R^N
                output.push(comb_val / self.gain());
            }
        }

        output
    }

    /// Change the decimation rate at runtime.
    ///
    /// This resets the comb delay lines and sample counter to avoid artifacts
    /// from the rate transition. The integrator accumulators are preserved so
    /// that DC tracking is maintained. The new rate is clamped to at least 1.
    pub fn set_rate(&mut self, rate: usize) {
        let rate = rate.max(1);
        self.rate = rate;
        // Reset comb delay lines for the new rate
        for comb in &mut self.combs {
            comb[0] = 0.0;
        }
        self.sample_count = 0;
    }

    /// Return the current decimation rate.
    pub fn rate(&self) -> usize {
        self.rate
    }

    /// Return the CIC filter gain, which is R^N.
    ///
    /// This is the DC gain of the unnormalized CIC filter. The `process`
    /// method divides by this value automatically.
    pub fn gain(&self) -> f64 {
        (self.rate as f64).powi(self.order as i32)
    }
}

/// Variable-rate CIC interpolation filter.
///
/// Implements an N-th order CIC interpolator whose interpolation rate can be
/// changed at runtime via [`set_rate`](VariableRateCicInterpolator::set_rate).
///
/// The processing pipeline is the reverse of decimation:
/// ```text
/// Input → [Comb 1] → ... → [Comb N] → ↑R → [Integrator 1] → ... → [Integrator N] → Output
/// ```
///
/// For each input sample, `rate` output samples are produced (one from the
/// input value, followed by `rate - 1` zeros fed through the integrators).
#[derive(Debug, Clone)]
pub struct VariableRateCicInterpolator {
    /// Filter order (number of comb-integrator stages)
    order: usize,
    /// Current interpolation rate
    rate: usize,
    /// Comb delay lines (run at input rate)
    combs: Vec<VecDeque<f64>>,
    /// Integrator accumulators (run at output rate)
    integrators: Vec<f64>,
}

impl VariableRateCicInterpolator {
    /// Create a new variable-rate CIC interpolator.
    ///
    /// - `order`: number of comb-integrator stages (N). Clamped to at least 1.
    /// - `rate`: initial interpolation factor (R). Clamped to at least 1.
    pub fn new(order: usize, rate: usize) -> Self {
        let order = order.max(1);
        let rate = rate.max(1);
        Self {
            order,
            rate,
            combs: (0..order).map(|_| {
                let mut d = VecDeque::with_capacity(1);
                d.push_back(0.0);
                d
            }).collect(),
            integrators: vec![0.0; order],
        }
    }

    /// Process a block of input samples, producing interpolated output.
    ///
    /// For each input sample, `rate` output samples are produced. The total
    /// output length is `input.len() * rate`.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() * self.rate);

        for &x in input {
            // Comb stages (run at input/low rate)
            let mut comb_val = x;
            for i in 0..self.order {
                let delayed = self.combs[i][0];
                self.combs[i][0] = comb_val;
                comb_val -= delayed;
            }

            // Upsample: insert the comb output then R-1 zeros
            for k in 0..self.rate {
                let inp = if k == 0 { comb_val } else { 0.0 };

                // Integrator stages (run at output/high rate)
                let mut val = inp;
                for i in 0..self.order {
                    self.integrators[i] += val;
                    val = self.integrators[i];
                }

                // Normalize by CIC gain R^N
                output.push(val / self.gain());
            }
        }

        output
    }

    /// Change the interpolation rate at runtime.
    ///
    /// Resets comb delay lines and integrator accumulators. The new rate is
    /// clamped to at least 1.
    pub fn set_rate(&mut self, rate: usize) {
        let rate = rate.max(1);
        self.rate = rate;
        for comb in &mut self.combs {
            comb[0] = 0.0;
        }
        self.integrators.fill(0.0);
    }

    /// Return the current interpolation rate.
    pub fn rate(&self) -> usize {
        self.rate
    }

    /// Return the CIC filter gain (R^N).
    pub fn gain(&self) -> f64 {
        (self.rate as f64).powi(self.order as i32)
    }
}

/// CIC droop compensator using an inverse-sinc FIR filter.
///
/// The CIC frequency response follows a sinc^N shape that droops away from DC.
/// This compensator designs a short FIR filter whose response is the inverse of
/// the CIC droop across the passband, restoring flatness.
///
/// The compensation filter taps are computed once at construction and applied
/// via direct-form FIR convolution.
#[derive(Debug, Clone)]
pub struct CicCompensator {
    /// FIR filter taps (inverse-sinc compensating filter)
    taps: Vec<f64>,
}

impl CicCompensator {
    /// Design a CIC droop compensation FIR filter.
    ///
    /// - `order`: CIC filter order (N)
    /// - `rate`: CIC decimation/interpolation rate (R)
    /// - `num_taps`: number of FIR taps (will be forced odd for symmetry)
    ///
    /// The filter is designed by sampling the inverse CIC magnitude response
    /// and applying a Hamming-windowed sinc interpolation kernel.
    pub fn new(order: usize, rate: usize, num_taps: usize) -> Self {
        let n = num_taps.max(3) | 1; // force odd
        let half = n / 2;
        let mut taps = vec![0.0; n];

        for i in 0..n {
            let k = i as f64 - half as f64;
            // Normalized frequency for this tap position
            let f = (k / n as f64).abs().clamp(1e-12, 0.4999);

            // CIC magnitude response: |sin(pi*R*f) / (R * sin(pi*f))|^N
            let sinc_num = (std::f64::consts::PI * rate as f64 * f).sin();
            let sinc_den = rate as f64 * (std::f64::consts::PI * f).sin();
            let cic_mag = (sinc_num / sinc_den).abs().powi(order as i32);

            // Inverse CIC for compensation (clamp to avoid division by near-zero)
            let inv_cic = if cic_mag > 0.01 { 1.0 / cic_mag } else { 1.0 };

            // Lowpass sinc kernel
            let sinc = if k.abs() < 1e-10 {
                1.0
            } else {
                let x = std::f64::consts::PI * k;
                x.sin() / x
            };

            // Hamming window
            let window = 0.54
                - 0.46 * (2.0 * std::f64::consts::PI * i as f64 / (n - 1) as f64).cos();

            taps[i] = sinc * inv_cic * window;
        }

        // Normalize taps to unity DC gain
        let sum: f64 = taps.iter().sum();
        if sum.abs() > 1e-10 {
            for t in &mut taps {
                *t /= sum;
            }
        }

        Self { taps }
    }

    /// Apply the compensation filter to an input signal.
    ///
    /// Performs direct-form FIR filtering. The output has the same length as
    /// the input; the filter is applied with zero-padded edges (causal, with
    /// group delay of `num_taps / 2` samples).
    pub fn process(&self, input: &[f64]) -> Vec<f64> {
        let n = self.taps.len();
        let mut output = Vec::with_capacity(input.len());

        for i in 0..input.len() {
            let mut acc = 0.0;
            for (j, &tap) in self.taps.iter().enumerate() {
                let idx = i as isize - j as isize + (n / 2) as isize;
                if idx >= 0 && (idx as usize) < input.len() {
                    acc += tap * input[idx as usize];
                }
            }
            output.push(acc);
        }

        output
    }

    /// Return a reference to the FIR taps.
    pub fn taps(&self) -> &[f64] {
        &self.taps
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_decimate_by_2() {
        let mut dec = VariableRateCicDecimator::new(2, 2);
        let input = vec![1.0; 20];
        let output = dec.process(&input);
        assert_eq!(output.len(), 10, "Decimation by 2 of 20 samples should produce 10");
    }

    #[test]
    fn test_decimate_by_4() {
        let mut dec = VariableRateCicDecimator::new(2, 4);
        let input = vec![1.0; 40];
        let output = dec.process(&input);
        assert_eq!(output.len(), 10, "Decimation by 4 of 40 samples should produce 10");
    }

    #[test]
    fn test_interpolate_by_2() {
        let mut interp = VariableRateCicInterpolator::new(2, 2);
        let input = vec![1.0; 10];
        let output = interp.process(&input);
        assert_eq!(output.len(), 20, "Interpolation by 2 of 10 samples should produce 20");
    }

    #[test]
    fn test_interpolate_by_4() {
        let mut interp = VariableRateCicInterpolator::new(2, 4);
        let input = vec![1.0; 10];
        let output = interp.process(&input);
        assert_eq!(output.len(), 40, "Interpolation by 4 of 10 samples should produce 40");
    }

    #[test]
    fn test_variable_rate_change() {
        let mut dec = VariableRateCicDecimator::new(2, 4);
        assert_eq!(dec.rate(), 4);

        let input = vec![1.0; 40];
        let out1 = dec.process(&input);
        assert_eq!(out1.len(), 10);

        // Change rate to 2
        dec.set_rate(2);
        assert_eq!(dec.rate(), 2);

        let input2 = vec![1.0; 20];
        let out2 = dec.process(&input2);
        assert_eq!(out2.len(), 10, "After rate change to 2, 20 samples should produce 10");
    }

    #[test]
    fn test_dc_passthrough() {
        // A CIC decimator should pass DC (constant) input through to a constant
        // output after the filter has settled.
        let mut dec = VariableRateCicDecimator::new(2, 4);
        let input = vec![1.0; 200];
        let output = dec.process(&input);

        // Check settled portion (skip initial transient)
        let settled = &output[output.len() / 2..];
        for &s in settled {
            assert!(
                (s - 1.0).abs() < 0.01,
                "DC should pass through CIC decimator: got {:.6}",
                s
            );
        }
    }

    #[test]
    fn test_gain_calculation() {
        let dec2 = VariableRateCicDecimator::new(2, 4);
        assert_eq!(dec2.gain(), 16.0, "4^2 = 16");

        let dec3 = VariableRateCicDecimator::new(3, 3);
        assert_eq!(dec3.gain(), 27.0, "3^3 = 27");

        let dec1 = VariableRateCicDecimator::new(1, 10);
        assert_eq!(dec1.gain(), 10.0, "10^1 = 10");

        let interp = VariableRateCicInterpolator::new(3, 5);
        assert_eq!(interp.gain(), 125.0, "5^3 = 125");
    }

    #[test]
    fn test_compensator() {
        let comp = CicCompensator::new(3, 4, 21);
        let taps = comp.taps();
        assert_eq!(taps.len(), 21);

        // Taps should sum to approximately 1.0 (unity DC gain)
        let sum: f64 = taps.iter().sum();
        assert!(
            (sum - 1.0).abs() < 0.01,
            "Compensator taps should sum to ~1.0, got {:.6}",
            sum
        );

        // DC input through compensator should remain ~1.0
        let dc_input = vec![1.0; 100];
        let dc_output = comp.process(&dc_input);
        // Check middle portion (away from edge effects)
        let mid = dc_output.len() / 2;
        assert!(
            (dc_output[mid] - 1.0).abs() < 0.05,
            "Compensator should pass DC: got {:.6}",
            dc_output[mid]
        );
    }

    #[test]
    fn test_higher_order() {
        // Test with higher order (5th order) to ensure stability
        let mut dec = VariableRateCicDecimator::new(5, 3);
        assert_eq!(dec.gain(), 243.0, "3^5 = 243");

        let input = vec![1.0; 300];
        let output = dec.process(&input);
        assert_eq!(output.len(), 100, "Decimation by 3 of 300 samples should produce 100");

        // Settled DC output should still be ~1.0
        let settled = &output[output.len() / 2..];
        for &s in settled {
            assert!(
                (s - 1.0).abs() < 0.01,
                "5th-order CIC should pass DC: got {:.6}",
                s
            );
        }

        // Higher order interpolator
        let mut interp = VariableRateCicInterpolator::new(5, 3);
        let input = vec![1.0; 10];
        let output = interp.process(&input);
        assert_eq!(output.len(), 30, "Interpolation by 3 of 10 samples should produce 30");
    }

    #[test]
    fn test_empty_input() {
        let mut dec = VariableRateCicDecimator::new(3, 4);
        let output = dec.process(&[]);
        assert!(output.is_empty(), "Empty input should produce empty output");

        let mut interp = VariableRateCicInterpolator::new(3, 4);
        let output = interp.process(&[]);
        assert!(output.is_empty(), "Empty input should produce empty output for interpolator");

        let comp = CicCompensator::new(3, 4, 21);
        let output = comp.process(&[]);
        assert!(output.is_empty(), "Empty input should produce empty output for compensator");
    }
}
