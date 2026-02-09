//! Decimating FIR Filter — Combined lowpass filtering and decimation
//!
//! Efficiently combines FIR filtering with integer decimation by only computing
//! output samples that won't be discarded. This is the standard approach for
//! downsampling in SDR receivers (e.g., wideband IQ → narrowband channel).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::decimating_fir::DecimatingFir;
//! use num_complex::Complex64;
//!
//! // Decimate by 4 with a simple lowpass
//! let taps = vec![0.25; 4]; // Simple boxcar
//! let mut dfir = DecimatingFir::new(&taps, 4);
//! let input: Vec<Complex64> = (0..100)
//!     .map(|i| Complex64::new((i as f64 * 0.1).sin(), 0.0))
//!     .collect();
//! let output = dfir.process(&input);
//! assert_eq!(output.len(), 25); // 100 / 4 = 25
//! ```

use num_complex::Complex64;

/// Combined FIR filter + decimation.
///
/// Only computes output samples at the decimation rate, saving computation.
/// Maintains state between calls for streaming operation.
#[derive(Debug, Clone)]
pub struct DecimatingFir {
    /// FIR filter taps.
    taps: Vec<f64>,
    /// Decimation factor.
    decim: usize,
    /// Delay line (history buffer).
    history: Vec<Complex64>,
    /// Current sample counter (for tracking decimation phase).
    phase: usize,
}

impl DecimatingFir {
    /// Create a decimating FIR filter.
    ///
    /// `taps`: FIR filter coefficients.
    /// `decimation`: Decimation factor (keep 1 in N samples).
    pub fn new(taps: &[f64], decimation: usize) -> Self {
        let decim = decimation.max(1);
        Self {
            taps: taps.to_vec(),
            decim,
            history: vec![Complex64::new(0.0, 0.0); taps.len()],
            phase: 0,
        }
    }

    /// Create with auto-designed lowpass filter.
    ///
    /// Designs a Kaiser-window FIR lowpass with cutoff at π/decimation
    /// and the specified transition bandwidth.
    pub fn lowpass(decimation: usize, num_taps: usize) -> Self {
        let decim = decimation.max(1);
        let n = if num_taps > 0 { num_taps } else { decim * 8 + 1 };
        let cutoff = 1.0 / decim as f64;
        let taps = design_lowpass(n, cutoff);
        Self::new(&taps, decim)
    }

    /// Process a block of complex samples.
    ///
    /// Output length = ceil((input_len + phase_offset) / decimation) approximately.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len() / self.decim + 1);

        for &sample in input {
            // Shift history
            self.history.rotate_right(1);
            self.history[0] = sample;

            // Only compute output at decimation phase
            self.phase += 1;
            if self.phase >= self.decim {
                self.phase = 0;
                // Compute FIR output
                let mut sum = Complex64::new(0.0, 0.0);
                for (i, &tap) in self.taps.iter().enumerate() {
                    if i < self.history.len() {
                        sum += self.history[i] * tap;
                    }
                }
                output.push(sum);
            }
        }

        output
    }

    /// Process real-valued samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        let complex: Vec<Complex64> = input.iter()
            .map(|&r| Complex64::new(r, 0.0))
            .collect();
        self.process(&complex).iter().map(|c| c.re).collect()
    }

    /// Get the decimation factor.
    pub fn decimation(&self) -> usize {
        self.decim
    }

    /// Get the filter taps.
    pub fn taps(&self) -> &[f64] {
        &self.taps
    }

    /// Get the filter order (number of taps).
    pub fn order(&self) -> usize {
        self.taps.len()
    }

    /// Reset the filter state.
    pub fn reset(&mut self) {
        self.history.fill(Complex64::new(0.0, 0.0));
        self.phase = 0;
    }
}

/// Decimating FIR with real-valued taps applied to real samples.
#[derive(Debug, Clone)]
pub struct RealDecimatingFir {
    taps: Vec<f64>,
    decim: usize,
    history: Vec<f64>,
    phase: usize,
}

impl RealDecimatingFir {
    pub fn new(taps: &[f64], decimation: usize) -> Self {
        let decim = decimation.max(1);
        Self {
            taps: taps.to_vec(),
            decim,
            history: vec![0.0; taps.len()],
            phase: 0,
        }
    }

    pub fn lowpass(decimation: usize, num_taps: usize) -> Self {
        let decim = decimation.max(1);
        let n = if num_taps > 0 { num_taps } else { decim * 8 + 1 };
        let cutoff = 1.0 / decim as f64;
        let taps = design_lowpass(n, cutoff);
        Self::new(&taps, decim)
    }

    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() / self.decim + 1);

        for &sample in input {
            self.history.rotate_right(1);
            self.history[0] = sample;

            self.phase += 1;
            if self.phase >= self.decim {
                self.phase = 0;
                let sum: f64 = self.taps.iter().zip(self.history.iter())
                    .map(|(&t, &h)| t * h)
                    .sum();
                output.push(sum);
            }
        }

        output
    }

    pub fn reset(&mut self) {
        self.history.fill(0.0);
        self.phase = 0;
    }
}

/// Design a simple lowpass FIR using windowed-sinc method.
fn design_lowpass(num_taps: usize, cutoff: f64) -> Vec<f64> {
    let n = num_taps;
    let m = (n - 1) as f64 / 2.0;
    let mut taps = Vec::with_capacity(n);

    for i in 0..n {
        let x = i as f64 - m;
        let sinc = if x.abs() < 1e-10 {
            cutoff
        } else {
            (std::f64::consts::PI * cutoff * x).sin() / (std::f64::consts::PI * x)
        };
        // Hamming window
        let window = 0.54 - 0.46 * (2.0 * std::f64::consts::PI * i as f64 / (n - 1) as f64).cos();
        taps.push(sinc * window);
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
    use std::f64::consts::PI;

    #[test]
    fn test_decimation_factor() {
        let taps = vec![1.0; 4];
        let mut dfir = DecimatingFir::new(&taps, 4);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = dfir.process(&input);
        assert_eq!(output.len(), 25, "Should decimate 100 → 25 (factor 4)");
    }

    #[test]
    fn test_constant_input() {
        let taps = vec![0.25; 4];
        let mut dfir = DecimatingFir::new(&taps, 2);
        let input = vec![Complex64::new(1.0, 0.0); 40];
        let output = dfir.process(&input);
        assert_eq!(output.len(), 20);
        // After settling, output should be ~1.0 (sum of taps = 1.0)
        for &s in &output[4..] {
            assert!((s.re - 1.0).abs() < 0.01, "Expected ~1.0, got {}", s.re);
        }
    }

    #[test]
    fn test_lowpass_design() {
        let dfir = DecimatingFir::lowpass(4, 33);
        assert_eq!(dfir.order(), 33);
        assert_eq!(dfir.decimation(), 4);
        // Taps should be normalized to sum ≈ 1
        let sum: f64 = dfir.taps().iter().sum();
        assert!((sum - 1.0).abs() < 0.01, "Taps should sum to ~1.0, got {}", sum);
    }

    #[test]
    fn test_removes_high_frequency() {
        let mut dfir = DecimatingFir::lowpass(4, 33);
        // Low frequency signal (should pass)
        let low: Vec<Complex64> = (0..200)
            .map(|i| Complex64::new((2.0 * PI * 0.01 * i as f64).sin(), 0.0))
            .collect();
        let out_low = dfir.process(&low);

        dfir.reset();

        // High frequency signal (near Nyquist, should be attenuated)
        let high: Vec<Complex64> = (0..200)
            .map(|i| Complex64::new((2.0 * PI * 0.45 * i as f64).sin(), 0.0))
            .collect();
        let out_high = dfir.process(&high);

        // Compare power in second half (after settling)
        let power_low: f64 = out_low[25..].iter().map(|s| s.norm_sqr()).sum::<f64>() / (out_low.len() - 25) as f64;
        let power_high: f64 = out_high[25..].iter().map(|s| s.norm_sqr()).sum::<f64>() / (out_high.len() - 25) as f64;

        assert!(power_low > power_high * 5.0,
            "Low freq power {} should be much more than high freq power {}", power_low, power_high);
    }

    #[test]
    fn test_streaming() {
        let taps = vec![0.5, 0.5];
        let mut dfir = DecimatingFir::new(&taps, 2);
        let input1 = vec![Complex64::new(1.0, 0.0); 10];
        let input2 = vec![Complex64::new(1.0, 0.0); 10];
        let out1 = dfir.process(&input1);
        let out2 = dfir.process(&input2);
        // Total output should be 10
        assert_eq!(out1.len() + out2.len(), 10);
    }

    #[test]
    fn test_real_decimating_fir() {
        let mut dfir = RealDecimatingFir::lowpass(4, 17);
        let input: Vec<f64> = (0..100).map(|i| (0.02 * PI * i as f64).sin()).collect();
        let output = dfir.process(&input);
        assert_eq!(output.len(), 25);
    }

    #[test]
    fn test_process_real() {
        let taps = vec![0.5, 0.5];
        let mut dfir = DecimatingFir::new(&taps, 2);
        let input: Vec<f64> = vec![1.0; 20];
        let output = dfir.process_real(&input);
        assert_eq!(output.len(), 10);
    }

    #[test]
    fn test_reset() {
        let mut dfir = DecimatingFir::lowpass(4, 17);
        dfir.process(&vec![Complex64::new(5.0, 0.0); 50]);
        dfir.reset();
        // After reset, first output should be near zero (clean state)
        let out = dfir.process(&vec![Complex64::new(0.0, 0.0); 10]);
        for &s in &out {
            assert!(s.norm() < 0.01, "After reset, output should be near zero");
        }
    }

    #[test]
    fn test_decim_1_passthrough() {
        let taps = vec![1.0];
        let mut dfir = DecimatingFir::new(&taps, 1);
        let input: Vec<Complex64> = (0..20)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let output = dfir.process(&input);
        assert_eq!(output.len(), 20);
        for (i, &s) in output.iter().enumerate() {
            assert!((s.re - i as f64).abs() < 0.01);
        }
    }

    #[test]
    fn test_complex_signal() {
        let taps = vec![0.5, 0.5];
        let mut dfir = DecimatingFir::new(&taps, 2);
        let input: Vec<Complex64> = (0..20)
            .map(|i| Complex64::new(1.0, (i as f64 * 0.3).sin()))
            .collect();
        let output = dfir.process(&input);
        assert_eq!(output.len(), 10);
    }
}
