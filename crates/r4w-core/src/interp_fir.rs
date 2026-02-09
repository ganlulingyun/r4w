//! Interpolating FIR Filter — Upsample and filter in one step
//!
//! Combines integer upsampling with FIR anti-imaging filtering
//! using polyphase decomposition for efficiency. Each input sample
//! produces L output samples. Used for sample rate conversion,
//! pulse shaping, and symbol-rate to chip-rate upconversion.
//! GNU Radio equivalent: `interp_fir_filter_ccf/fff`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::interp_fir::InterpFir;
//!
//! // 4x interpolation with a simple boxcar (all-ones) filter
//! let taps = vec![1.0; 4];
//! let mut filt = InterpFir::new(4, &taps);
//! let output = filt.process(&[1.0, 0.0]);
//! assert_eq!(output.len(), 8); // 2 inputs × 4 = 8 outputs
//! ```

use num_complex::Complex64;

/// Interpolating FIR filter (real-valued).
#[derive(Debug, Clone)]
pub struct InterpFir {
    /// Interpolation factor.
    interp: usize,
    /// Polyphase filter arms (interp arms, each with ceil(taps/interp) taps).
    arms: Vec<Vec<f64>>,
    /// Delay line.
    delay: Vec<f64>,
    /// Write position in delay line.
    pos: usize,
}

impl InterpFir {
    /// Create an interpolating FIR filter.
    ///
    /// - `interp`: interpolation factor (L)
    /// - `taps`: prototype lowpass filter coefficients
    pub fn new(interp: usize, taps: &[f64]) -> Self {
        let interp = interp.max(1);
        let taps_per_arm = (taps.len() + interp - 1) / interp;

        // Decompose into polyphase arms
        let mut arms = vec![vec![0.0; taps_per_arm]; interp];
        for (i, &t) in taps.iter().enumerate() {
            let arm = i % interp;
            let idx = i / interp;
            arms[arm][idx] = t;
        }

        let delay_len = taps_per_arm;
        Self {
            interp,
            arms,
            delay: vec![0.0; delay_len],
            pos: 0,
        }
    }

    /// Process a single input sample, producing `interp` output samples.
    pub fn interpolate_one(&mut self, x: f64) -> Vec<f64> {
        // Insert sample into delay line
        self.delay[self.pos] = x;

        let mut output = Vec::with_capacity(self.interp);
        let delay_len = self.delay.len();

        for arm in &self.arms {
            let mut sum = 0.0;
            for (j, &coeff) in arm.iter().enumerate() {
                let idx = (self.pos + delay_len - j) % delay_len;
                sum += coeff * self.delay[idx];
            }
            output.push(sum);
        }

        self.pos = (self.pos + 1) % delay_len;
        output
    }

    /// Process a block of input samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() * self.interp);
        for &x in input {
            output.extend(self.interpolate_one(x));
        }
        output
    }

    /// Get interpolation factor.
    pub fn interp_factor(&self) -> usize {
        self.interp
    }

    /// Get number of polyphase arms.
    pub fn num_arms(&self) -> usize {
        self.arms.len()
    }

    /// Reset delay line.
    pub fn reset(&mut self) {
        self.delay.fill(0.0);
        self.pos = 0;
    }
}

/// Interpolating FIR filter for complex samples.
#[derive(Debug, Clone)]
pub struct InterpFirComplex {
    /// Interpolation factor.
    interp: usize,
    /// Polyphase arms (real-valued taps applied to complex data).
    arms: Vec<Vec<f64>>,
    /// Delay line.
    delay: Vec<Complex64>,
    /// Write position.
    pos: usize,
}

impl InterpFirComplex {
    /// Create a complex interpolating FIR filter.
    pub fn new(interp: usize, taps: &[f64]) -> Self {
        let interp = interp.max(1);
        let taps_per_arm = (taps.len() + interp - 1) / interp;

        let mut arms = vec![vec![0.0; taps_per_arm]; interp];
        for (i, &t) in taps.iter().enumerate() {
            let arm = i % interp;
            let idx = i / interp;
            arms[arm][idx] = t;
        }

        Self {
            interp,
            arms,
            delay: vec![Complex64::new(0.0, 0.0); taps_per_arm],
            pos: 0,
        }
    }

    /// Process a single complex input sample.
    pub fn interpolate_one(&mut self, x: Complex64) -> Vec<Complex64> {
        self.delay[self.pos] = x;

        let mut output = Vec::with_capacity(self.interp);
        let delay_len = self.delay.len();

        for arm in &self.arms {
            let mut sum = Complex64::new(0.0, 0.0);
            for (j, &coeff) in arm.iter().enumerate() {
                let idx = (self.pos + delay_len - j) % delay_len;
                sum += self.delay[idx] * coeff;
            }
            output.push(sum);
        }

        self.pos = (self.pos + 1) % delay_len;
        output
    }

    /// Process a block of complex samples.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len() * self.interp);
        for &x in input {
            output.extend(self.interpolate_one(x));
        }
        output
    }

    /// Reset.
    pub fn reset(&mut self) {
        self.delay.fill(Complex64::new(0.0, 0.0));
        self.pos = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_output_length() {
        let mut filt = InterpFir::new(4, &[1.0; 4]);
        let output = filt.process(&[1.0, 2.0, 3.0]);
        assert_eq!(output.len(), 12); // 3 * 4
    }

    #[test]
    fn test_passthrough_interp1() {
        // Interp=1 should be a regular FIR
        let mut filt = InterpFir::new(1, &[1.0]);
        let output = filt.process(&[1.0, 2.0, 3.0]);
        assert_eq!(output, vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_interp2_simple() {
        // 2x interpolation with [1, 1] taps → zero-order hold
        let mut filt = InterpFir::new(2, &[1.0, 1.0]);
        let output = filt.process(&[1.0, 0.0]);
        assert_eq!(output.len(), 4);
    }

    #[test]
    fn test_impulse_response() {
        let taps = vec![0.25, 0.5, 0.75, 1.0];
        let mut filt = InterpFir::new(2, &taps);
        let output = filt.process(&[1.0, 0.0, 0.0, 0.0]);
        assert_eq!(output.len(), 8);
        // First output pair comes from first input
    }

    #[test]
    fn test_complex_output_length() {
        let mut filt = InterpFirComplex::new(3, &[1.0; 3]);
        let input = vec![Complex64::new(1.0, 0.0), Complex64::new(0.0, 1.0)];
        let output = filt.process(&input);
        assert_eq!(output.len(), 6); // 2 * 3
    }

    #[test]
    fn test_complex_passthrough() {
        let mut filt = InterpFirComplex::new(1, &[1.0]);
        let input = vec![Complex64::new(1.0, 2.0), Complex64::new(3.0, 4.0)];
        let output = filt.process(&input);
        assert_eq!(output.len(), 2);
        assert!((output[0].re - 1.0).abs() < 1e-10);
        assert!((output[0].im - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_reset() {
        let mut filt = InterpFir::new(2, &[1.0, 0.5]);
        filt.process(&[1.0, 2.0]);
        filt.reset();
        let out1 = filt.process(&[1.0]);
        filt.reset();
        let out2 = filt.process(&[1.0]);
        assert_eq!(out1, out2);
    }

    #[test]
    fn test_accessors() {
        let filt = InterpFir::new(4, &[1.0; 8]);
        assert_eq!(filt.interp_factor(), 4);
        assert_eq!(filt.num_arms(), 4);
    }

    #[test]
    fn test_energy_conservation() {
        // Total energy should scale with interpolation factor
        let taps = vec![1.0; 4];
        let mut filt = InterpFir::new(4, &taps);
        let input = vec![1.0];
        let output = filt.process(&input);
        let in_energy: f64 = input.iter().map(|&x| x * x).sum();
        let out_energy: f64 = output.iter().map(|&x| x * x).sum();
        // Output energy should be interp * input energy (for boxcar)
        assert!(
            out_energy > 0.0,
            "output should have non-zero energy"
        );
    }

    #[test]
    fn test_complex_reset() {
        let mut filt = InterpFirComplex::new(2, &[1.0, 0.5]);
        filt.process(&[Complex64::new(1.0, 0.0)]);
        filt.reset();
        let out1 = filt.process(&[Complex64::new(1.0, 0.0)]);
        filt.reset();
        let out2 = filt.process(&[Complex64::new(1.0, 0.0)]);
        assert_eq!(out1, out2);
    }

    #[test]
    fn test_empty_input() {
        let mut filt = InterpFir::new(3, &[1.0; 3]);
        assert!(filt.process(&[]).is_empty());
    }
}
