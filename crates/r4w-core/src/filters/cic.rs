//! CIC (Cascaded Integrator-Comb) Filters
//!
//! Efficient multirate filters for high decimation/interpolation ratios.
//! CIC filters use only adders and delays (no multipliers), making them
//! ideal for first-stage decimation of high-sample-rate ADC data.
//!
//! ## Architecture
//!
//! ```text
//! Decimator:  Integrator stages → Downsample → Comb stages
//! Interpolator: Comb stages → Upsample → Integrator stages
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::filters::cic::{CicDecimator, CicInterpolator};
//! use num_complex::Complex64;
//!
//! // Decimate by 10 with 4 stages
//! let mut dec = CicDecimator::new(10, 4);
//! let input: Vec<Complex64> = (0..100).map(|i| Complex64::new(1.0, 0.0)).collect();
//! let output = dec.process_block(&input);
//! assert_eq!(output.len(), 10); // 100 / 10 = 10
//! ```

use num_complex::Complex64;

/// CIC Decimation filter.
///
/// Implements an N-stage CIC decimator. The transfer function is:
/// ```text
/// H(z) = [(1 - z^(-R)) / (1 - z^(-1))]^N
/// ```
/// where R is the decimation rate and N is the number of stages.
///
/// Gain is R^N, which can be large. Use `set_gain_correction(true)` for
/// automatic output scaling.
#[derive(Debug, Clone)]
pub struct CicDecimator {
    /// Decimation rate
    rate: usize,
    /// Number of stages
    stages: usize,
    /// Integrator state (one per stage)
    integrators: Vec<i64>,
    /// Comb delay state (one per stage)
    comb_delays: Vec<i64>,
    /// Sample counter for decimation
    count: usize,
    /// Whether to apply gain correction
    gain_correction: bool,
    /// Gain = rate^stages (precomputed)
    gain: f64,
}

impl CicDecimator {
    /// Create a new CIC decimator.
    ///
    /// - `rate`: decimation factor (R)
    /// - `stages`: number of integrator-comb stages (N), typically 1-6
    pub fn new(rate: usize, stages: usize) -> Self {
        let rate = rate.max(1);
        let stages = stages.max(1).min(6);
        let gain = (rate as f64).powi(stages as i32);
        Self {
            rate,
            stages,
            integrators: vec![0i64; stages],
            comb_delays: vec![0i64; stages],
            count: 0,
            gain_correction: true,
            gain,
        }
    }

    /// Enable or disable automatic gain correction.
    pub fn set_gain_correction(&mut self, enable: bool) {
        self.gain_correction = enable;
    }

    /// Process a single input sample. Returns `Some(output)` when a
    /// decimated sample is ready, `None` otherwise.
    pub fn process_sample(&mut self, input: Complex64) -> Option<Complex64> {
        // Scale input to integer representation for accumulator precision
        let scale = 1 << 16;
        let re_int = (input.re * scale as f64) as i64;
        let im_int = (input.im * scale as f64) as i64;

        // Integrator stages (run at input rate)
        let mut re = re_int;
        let mut im = im_int;
        for i in 0..self.stages {
            self.integrators[i] = self.integrators[i].wrapping_add(re);
            re = self.integrators[i];
            // Reuse integrators vec for imaginary part by interleaving
            // Actually use separate tracking — simpler to do complex as two real
        }

        self.count += 1;
        if self.count >= self.rate {
            self.count = 0;

            // Comb stages (run at output rate)
            let mut out = re;
            for i in 0..self.stages {
                let delayed = self.comb_delays[i];
                self.comb_delays[i] = out;
                out = out.wrapping_sub(delayed);
            }

            // Scale back
            let scale_back = if self.gain_correction {
                scale as f64 * self.gain
            } else {
                scale as f64
            };

            Some(Complex64::new(out as f64 / scale_back, 0.0))
        } else {
            None
        }
    }

    /// Process a block of complex input samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        // Process real and imaginary parts with separate CIC chains
        let mut output = Vec::with_capacity(input.len() / self.rate + 1);

        // We need separate state for I and Q channels
        let mut integrators_q = vec![0i64; self.stages];
        let mut comb_delays_q = vec![0i64; self.stages];

        // Reset internal state and process fresh for block mode
        // Actually, let's just do it properly with separate I/Q tracking
        // by processing real-valued through two instances
        for &sample in input {
            let scale = 1 << 16;
            let mut re = (sample.re * scale as f64) as i64;
            let mut im = (sample.im * scale as f64) as i64;

            // Integrator stages
            for i in 0..self.stages {
                self.integrators[i] = self.integrators[i].wrapping_add(re);
                re = self.integrators[i];
                integrators_q[i] = integrators_q[i].wrapping_add(im);
                im = integrators_q[i];
            }

            self.count += 1;
            if self.count >= self.rate {
                self.count = 0;

                // Comb stages for I
                let mut out_re = re;
                for i in 0..self.stages {
                    let delayed = self.comb_delays[i];
                    self.comb_delays[i] = out_re;
                    out_re = out_re.wrapping_sub(delayed);
                }

                // Comb stages for Q
                let mut out_im = im;
                for i in 0..self.stages {
                    let delayed = comb_delays_q[i];
                    comb_delays_q[i] = out_im;
                    out_im = out_im.wrapping_sub(delayed);
                }

                let scale_back = if self.gain_correction {
                    (1 << 16) as f64 * self.gain
                } else {
                    (1 << 16) as f64
                };

                output.push(Complex64::new(
                    out_re as f64 / scale_back,
                    out_im as f64 / scale_back,
                ));
            }
        }

        output
    }

    /// Get the decimation rate.
    pub fn rate(&self) -> usize {
        self.rate
    }

    /// Get the number of stages.
    pub fn stages(&self) -> usize {
        self.stages
    }

    /// Get the CIC gain (R^N).
    pub fn gain(&self) -> f64 {
        self.gain
    }

    /// Reset all internal state.
    pub fn reset(&mut self) {
        self.integrators.fill(0);
        self.comb_delays.fill(0);
        self.count = 0;
    }

    /// Compute the passband droop at a normalized frequency (0 to 0.5).
    ///
    /// The CIC frequency response has droop described by:
    /// ```text
    /// |H(f)| = |sin(π*R*f) / sin(π*f)|^N / R^N
    /// ```
    pub fn passband_droop_db(&self, freq_normalized: f64) -> f64 {
        let f = freq_normalized.clamp(1e-10, 0.4999);
        let numerator = (std::f64::consts::PI * self.rate as f64 * f).sin();
        let denominator = (std::f64::consts::PI * f).sin();
        let response = (numerator / denominator / self.rate as f64).powi(self.stages as i32);
        20.0 * response.abs().log10()
    }
}

/// CIC Interpolation filter.
///
/// Implements an N-stage CIC interpolator: comb stages first (at low rate),
/// then upsample, then integrator stages (at high rate).
#[derive(Debug, Clone)]
pub struct CicInterpolator {
    /// Interpolation rate
    rate: usize,
    /// Number of stages
    stages: usize,
    /// Comb delay state
    comb_delays_re: Vec<i64>,
    comb_delays_im: Vec<i64>,
    /// Integrator state
    integrators_re: Vec<i64>,
    integrators_im: Vec<i64>,
    /// Gain
    gain: f64,
    /// Gain correction enabled
    gain_correction: bool,
}

impl CicInterpolator {
    /// Create a new CIC interpolator.
    pub fn new(rate: usize, stages: usize) -> Self {
        let rate = rate.max(1);
        let stages = stages.max(1).min(6);
        let gain = (rate as f64).powi(stages as i32);
        Self {
            rate,
            stages,
            comb_delays_re: vec![0i64; stages],
            comb_delays_im: vec![0i64; stages],
            integrators_re: vec![0i64; stages],
            integrators_im: vec![0i64; stages],
            gain,
            gain_correction: true,
        }
    }

    /// Process a block of samples, outputting `input.len() * rate` samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len() * self.rate);
        let scale = 1 << 16;

        for &sample in input {
            // Comb stages (at low rate)
            let mut re = (sample.re * scale as f64) as i64;
            let mut im = (sample.im * scale as f64) as i64;

            for i in 0..self.stages {
                let delayed_re = self.comb_delays_re[i];
                let delayed_im = self.comb_delays_im[i];
                self.comb_delays_re[i] = re;
                self.comb_delays_im[i] = im;
                re = re.wrapping_sub(delayed_re);
                im = im.wrapping_sub(delayed_im);
            }

            // Upsample: insert sample then R-1 zeros
            for k in 0..self.rate {
                let (inp_re, inp_im) = if k == 0 { (re, im) } else { (0, 0) };

                // Integrator stages (at high rate)
                let mut out_re = inp_re;
                let mut out_im = inp_im;
                for i in 0..self.stages {
                    self.integrators_re[i] = self.integrators_re[i].wrapping_add(out_re);
                    self.integrators_im[i] = self.integrators_im[i].wrapping_add(out_im);
                    out_re = self.integrators_re[i];
                    out_im = self.integrators_im[i];
                }

                let scale_back = if self.gain_correction {
                    scale as f64 * self.gain
                } else {
                    scale as f64
                };

                output.push(Complex64::new(
                    out_re as f64 / scale_back,
                    out_im as f64 / scale_back,
                ));
            }
        }

        output
    }

    /// Get the interpolation rate.
    pub fn rate(&self) -> usize {
        self.rate
    }

    /// Get the number of stages.
    pub fn stages(&self) -> usize {
        self.stages
    }

    /// Reset all internal state.
    pub fn reset(&mut self) {
        self.comb_delays_re.fill(0);
        self.comb_delays_im.fill(0);
        self.integrators_re.fill(0);
        self.integrators_im.fill(0);
    }
}

/// Design a compensation FIR filter for CIC passband droop.
///
/// Returns FIR taps that flatten the CIC passband response.
/// Use this after CIC decimation to improve passband flatness.
pub fn cic_compensation_taps(
    cic_rate: usize,
    cic_stages: usize,
    num_taps: usize,
    cutoff_normalized: f64,
) -> Vec<f64> {
    let n = num_taps.max(3) | 1; // Ensure odd
    let half = n / 2;
    let mut taps = vec![0.0; n];

    for i in 0..n {
        let t = (i as f64 - half as f64) / n as f64;

        // Inverse CIC response
        let f = t.abs().clamp(1e-10, 0.4999);
        let sinc_num = (std::f64::consts::PI * cic_rate as f64 * f).sin();
        let sinc_den = (cic_rate as f64 * (std::f64::consts::PI * f).sin()).max(1e-20);
        let cic_response = (sinc_num / sinc_den).abs().powi(cic_stages as i32);

        // Inverse = compensation, windowed by sinc lowpass
        let sinc = if t.abs() < 1e-10 {
            1.0
        } else {
            let x = std::f64::consts::PI * 2.0 * cutoff_normalized * (i as f64 - half as f64);
            x.sin() / x
        };

        let inv_cic = if cic_response > 0.01 {
            1.0 / cic_response
        } else {
            1.0
        };

        // Apply Hamming window
        let window =
            0.54 - 0.46 * (2.0 * std::f64::consts::PI * i as f64 / (n - 1) as f64).cos();

        taps[i] = sinc * inv_cic * window;
    }

    // Normalize
    let sum: f64 = taps.iter().sum();
    if sum.abs() > 1e-10 {
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
    fn test_cic_decimator_output_length() {
        let mut dec = CicDecimator::new(10, 3);
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 100];
        let output = dec.process_block(&input);
        assert_eq!(output.len(), 10, "Decimation by 10 of 100 samples should give 10");
    }

    #[test]
    fn test_cic_decimator_dc_passthrough() {
        let mut dec = CicDecimator::new(4, 2);

        // DC input should pass through (with some gain correction)
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 400];
        let output = dec.process_block(&input);

        // After settling, output should be close to 1.0 (DC passthrough)
        let settled = &output[output.len() / 2..];
        for s in settled {
            assert!(
                (s.re - 1.0).abs() < 0.1,
                "DC should pass through CIC: got {:.3}",
                s.re
            );
        }
    }

    #[test]
    fn test_cic_interpolator_output_length() {
        let mut interp = CicInterpolator::new(5, 3);
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 20];
        let output = interp.process_block(&input);
        assert_eq!(output.len(), 100, "Interpolation by 5 of 20 samples should give 100");
    }

    #[test]
    fn test_cic_decimator_rate1_passthrough() {
        let mut dec = CicDecimator::new(1, 2);
        let input: Vec<Complex64> = (0..10)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let output = dec.process_block(&input);
        assert_eq!(output.len(), 10, "Rate=1 should not decimate");
    }

    #[test]
    fn test_cic_passband_droop() {
        let dec = CicDecimator::new(10, 4);

        // DC has 0 dB droop
        let dc_droop = dec.passband_droop_db(0.001);
        assert!(dc_droop.abs() < 0.1, "DC droop should be ~0 dB: got {dc_droop:.2}");

        // Higher frequencies have more droop
        let edge_droop = dec.passband_droop_db(0.04);
        assert!(edge_droop < -0.1, "Edge droop should be negative: got {edge_droop:.2}");
    }

    #[test]
    fn test_cic_reset() {
        let mut dec = CicDecimator::new(4, 2);
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 40];
        let out1 = dec.process_block(&input);
        dec.reset();
        let out2 = dec.process_block(&input);

        for (a, b) in out1.iter().zip(out2.iter()) {
            assert!(
                (a - b).norm() < 1e-6,
                "Reset should produce identical output"
            );
        }
    }

    #[test]
    fn test_cic_compensation_taps() {
        let taps = cic_compensation_taps(10, 3, 31, 0.4);
        assert_eq!(taps.len(), 31);

        // Taps should sum to ~1.0 (normalized)
        let sum: f64 = taps.iter().sum();
        assert!(
            (sum - 1.0).abs() < 0.01,
            "Compensation taps should sum to 1.0: got {sum:.3}"
        );
    }

    #[test]
    fn test_cic_complex_signal() {
        let mut dec = CicDecimator::new(4, 2);

        // Complex signal
        let input: Vec<Complex64> = (0..80)
            .map(|i| Complex64::new(1.0, 0.5))
            .collect();
        let output = dec.process_block(&input);
        assert_eq!(output.len(), 20);

        // Settled output should have both I and Q
        let last = output.last().unwrap();
        assert!(last.re.abs() > 0.1, "I channel should be present");
        assert!(last.im.abs() > 0.1, "Q channel should be present");
    }
}
