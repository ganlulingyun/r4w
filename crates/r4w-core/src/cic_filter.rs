//! CIC Filter — Cascaded Integrator-Comb for Multirate DSP
//!
//! Efficient sample rate conversion without multiplications. CIC filters
//! are the workhorse of hardware-friendly decimation and interpolation
//! in SDR front-ends (HackRF, USRP, FPGA stages).
//!
//! Implements N-stage CIC decimation, interpolation, and passband
//! compensation FIR filter design.
//!
//! GNU Radio equivalent: `cic_decimator`, `cic_compensator`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cic_filter::CicDecimator;
//!
//! let mut cic = CicDecimator::new(3, 4);
//! let input: Vec<f64> = (0..64).map(|i| (i as f64 * 0.1).sin()).collect();
//! let output = cic.process(&input);
//! assert_eq!(output.len(), 16); // 64 / 4
//! ```

use std::f64::consts::PI;

/// N-stage CIC decimation filter.
///
/// Each stage consists of one integrator (running sum) and one comb
/// (delay-and-subtract). The CIC transfer function is:
///
///   H(z) = ((1 - z^{-R}) / (1 - z^{-1}))^N
///
/// where R is the decimation factor and N is the order (number of stages).
#[derive(Debug, Clone)]
pub struct CicDecimator {
    order: usize,
    decimation: usize,
    integrators: Vec<f64>,
    comb_delays: Vec<f64>,
    sample_count: usize,
}

impl CicDecimator {
    /// Create a new CIC decimation filter.
    ///
    /// `order`: number of cascaded stages (typically 1-5).
    /// `decimation`: decimation factor R.
    pub fn new(order: usize, decimation: usize) -> Self {
        let order = order.max(1);
        let decimation = decimation.max(1);
        Self {
            order,
            decimation,
            integrators: vec![0.0; order],
            comb_delays: vec![0.0; order],
            sample_count: 0,
        }
    }

    /// Process a block of input samples, returning decimated output.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() / self.decimation + 1);

        for &x in input {
            // Integrator stages (run at input rate)
            let mut val = x;
            for i in 0..self.order {
                self.integrators[i] += val;
                val = self.integrators[i];
            }

            self.sample_count += 1;
            if self.sample_count >= self.decimation {
                self.sample_count = 0;

                // Comb stages (run at output rate, delay of 1)
                let mut comb_val = val;
                for i in 0..self.order {
                    let delayed = self.comb_delays[i];
                    self.comb_delays[i] = comb_val;
                    comb_val = comb_val - delayed;
                }

                output.push(comb_val);
            }
        }
        output
    }

    /// Compute the magnitude frequency response at `num_points` equally spaced
    /// frequencies from 0 to π (normalized).
    pub fn frequency_response(&self, num_points: usize) -> Vec<f64> {
        (0..num_points)
            .map(|k| {
                let f = k as f64 * PI / num_points as f64;
                if f.abs() < 1e-12 {
                    (self.decimation as f64).powi(self.order as i32)
                } else {
                    let num = (f * self.decimation as f64 / 2.0).sin();
                    let den = (f / 2.0).sin();
                    (num / den).abs().powi(self.order as i32)
                }
            })
            .collect()
    }

    /// DC gain of the CIC filter = R^N.
    pub fn dc_gain(&self) -> f64 {
        (self.decimation as f64).powi(self.order as i32)
    }

    /// Number of additional bits needed to represent the CIC output
    /// without overflow: N * ceil(log2(R)).
    pub fn bit_growth(&self) -> u32 {
        let log2_r = (self.decimation as f64).log2().ceil() as u32;
        self.order as u32 * log2_r
    }

    /// Reset all internal state to zero.
    pub fn reset(&mut self) {
        self.integrators.fill(0.0);
        self.comb_delays.fill(0.0);
        self.sample_count = 0;
    }
}

/// N-stage CIC interpolation filter.
///
/// Upsamples by factor L with zero-insertion followed by CIC filtering.
/// The comb stages run at the input rate and integrator stages at the output rate.
#[derive(Debug, Clone)]
pub struct CicInterpolator {
    order: usize,
    interpolation: usize,
    integrators: Vec<f64>,
    comb_delays: Vec<f64>,
}

impl CicInterpolator {
    /// Create a new CIC interpolation filter.
    pub fn new(order: usize, interpolation: usize) -> Self {
        let order = order.max(1);
        let interpolation = interpolation.max(1);
        Self {
            order,
            interpolation,
            integrators: vec![0.0; order],
            comb_delays: vec![0.0; order],
        }
    }

    /// Process a block of input samples, returning interpolated output.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() * self.interpolation);

        for &x in input {
            // Comb stages (input rate)
            let mut val = x;
            for i in 0..self.order {
                let delayed = self.comb_delays[i];
                self.comb_delays[i] = val;
                val = val - delayed;
            }

            // Zero-insertion + integrator stages (output rate)
            for j in 0..self.interpolation {
                let insert = if j == 0 { val } else { 0.0 };
                let mut int_val = insert;
                for i in 0..self.order {
                    self.integrators[i] += int_val;
                    int_val = self.integrators[i];
                }
                output.push(int_val);
            }
        }
        output
    }

    /// Reset all internal state.
    pub fn reset(&mut self) {
        self.integrators.fill(0.0);
        self.comb_delays.fill(0.0);
    }
}

/// Design a CIC compensation FIR filter.
///
/// The CIC filter has a sinc^N passband droop. This function designs a
/// short FIR filter whose response approximates the inverse of the CIC
/// droop in the passband, flattening the overall response.
///
/// Returns FIR tap coefficients.
pub fn cic_compensator(order: usize, decimation: usize, num_taps: usize) -> Vec<f64> {
    let num_taps = num_taps | 1; // ensure odd for symmetry
    let half = num_taps / 2;
    let r = decimation as f64;
    let n = order as i32;

    // Design inverse-sinc FIR via windowed frequency sampling
    let mut taps = vec![0.0; num_taps];
    for i in 0..num_taps {
        let k = i as f64 - half as f64;
        if k.abs() < 1e-12 {
            taps[i] = 1.0;
        } else {
            // Sinc interpolation kernel
            let x = PI * k / (num_taps as f64);
            let sinc = x.sin() / x;
            // Inverse CIC droop at this point's equivalent frequency
            let f = k / (num_taps as f64 * r);
            let cic_resp = if f.abs() < 1e-12 {
                1.0
            } else {
                let num = (PI * f * r).sin();
                let den = (PI * f).sin() * r;
                (num / den).abs().powi(n)
            };
            let inv_droop = if cic_resp > 0.01 { 1.0 / cic_resp } else { 1.0 };
            taps[i] = sinc * inv_droop;
        }
    }

    // Apply Blackman window
    for i in 0..num_taps {
        let w = 0.42 - 0.5 * (2.0 * PI * i as f64 / (num_taps - 1) as f64).cos()
            + 0.08 * (4.0 * PI * i as f64 / (num_taps - 1) as f64).cos();
        taps[i] *= w;
    }

    // Normalize to unity DC gain
    let sum: f64 = taps.iter().sum();
    if sum.abs() > 1e-12 {
        for t in &mut taps {
            *t /= sum;
        }
    }

    taps
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_decimation_by_4() {
        let mut cic = CicDecimator::new(3, 4);
        let input: Vec<f64> = (0..64).map(|i| (i as f64 * 0.01).sin()).collect();
        let output = cic.process(&input);
        assert_eq!(output.len(), 16);
    }

    #[test]
    fn test_decimation_by_2() {
        let mut cic = CicDecimator::new(2, 2);
        let input: Vec<f64> = (0..100).map(|i| (i as f64 * 0.05).sin()).collect();
        let output = cic.process(&input);
        assert_eq!(output.len(), 50);
    }

    #[test]
    fn test_decimation_by_1_is_identity_shape() {
        let mut cic = CicDecimator::new(1, 1);
        let input = vec![1.0, 2.0, 3.0, 4.0];
        let output = cic.process(&input);
        assert_eq!(output.len(), 4);
    }

    #[test]
    fn test_dc_gain() {
        let cic = CicDecimator::new(3, 4);
        assert_eq!(cic.dc_gain(), 64.0); // 4^3
    }

    #[test]
    fn test_bit_growth() {
        let cic = CicDecimator::new(3, 4);
        assert_eq!(cic.bit_growth(), 6); // 3 * 2
        let cic2 = CicDecimator::new(4, 8);
        assert_eq!(cic2.bit_growth(), 12); // 4 * 3
    }

    #[test]
    fn test_dc_input_matches_gain() {
        let mut cic = CicDecimator::new(2, 4);
        // Feed DC signal of amplitude 1.0
        let input = vec![1.0; 100];
        let output = cic.process(&input);
        // After settling, output should approach R^N = 16
        let last = *output.last().unwrap();
        assert!((last - 16.0).abs() < 0.1, "last={last}, expected ~16.0");
    }

    #[test]
    fn test_interpolation_length() {
        let mut cic = CicInterpolator::new(2, 4);
        let input = vec![1.0, 2.0, 3.0, 4.0];
        let output = cic.process(&input);
        assert_eq!(output.len(), 16); // 4 * 4
    }

    #[test]
    fn test_frequency_response_dc() {
        let cic = CicDecimator::new(3, 4);
        let resp = cic.frequency_response(256);
        assert!((resp[0] - 64.0).abs() < 0.01); // DC gain = R^N
    }

    #[test]
    fn test_reset() {
        let mut cic = CicDecimator::new(2, 4);
        let input = vec![1.0; 20];
        let _ = cic.process(&input);
        cic.reset();
        assert!(cic.integrators.iter().all(|&x| x == 0.0));
        assert!(cic.comb_delays.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn test_compensator_unity_dc() {
        let taps = cic_compensator(3, 4, 15);
        let sum: f64 = taps.iter().sum();
        assert!((sum - 1.0).abs() < 0.01, "compensator DC gain={sum}");
    }
}
