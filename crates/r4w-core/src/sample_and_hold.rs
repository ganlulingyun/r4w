//! Sample-and-Hold and Quantizer Blocks
//!
//! Control-plane utility blocks for holding values, quantizing signals,
//! and modeling ADC/DAC effects.
//!
//! ## Blocks
//!
//! - **SampleAndHold**: Holds the last accepted sample when control is false
//! - **Quantizer**: Quantizes signal to discrete levels (ADC simulation)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::sample_and_hold::SampleAndHold;
//!
//! let mut sh = SampleAndHold::new(0.0_f64);
//! let data = vec![1.0, 2.0, 3.0, 4.0];
//! let ctrl = vec![true, false, true, false];
//! let output = sh.process(&data, &ctrl);
//! assert_eq!(output, vec![1.0, 1.0, 3.0, 3.0]);
//! ```

use num_complex::Complex64;

/// Sample-and-hold block — holds last accepted value when control is false.
#[derive(Debug, Clone)]
pub struct SampleAndHold<T: Copy + Default> {
    held_value: T,
}

impl<T: Copy + Default> SampleAndHold<T> {
    pub fn new(initial: T) -> Self {
        Self {
            held_value: initial,
        }
    }

    /// Process: when control is true, accept new sample; when false, output held value.
    pub fn process(&mut self, data: &[T], control: &[bool]) -> Vec<T> {
        let len = data.len().min(control.len());
        let mut output = Vec::with_capacity(len);
        for i in 0..len {
            if control[i] {
                self.held_value = data[i];
            }
            output.push(self.held_value);
        }
        output
    }

    /// Process a single sample.
    pub fn process_single(&mut self, data: T, control: bool) -> T {
        if control {
            self.held_value = data;
        }
        self.held_value
    }

    pub fn held_value(&self) -> T {
        self.held_value
    }

    pub fn reset(&mut self) {
        self.held_value = T::default();
    }
}

/// Convenience type alias for real-valued sample-and-hold.
pub type SampleAndHoldF = SampleAndHold<f64>;
/// Convenience type alias for complex sample-and-hold.
pub type SampleAndHoldC = SampleAndHold<Complex64>;

/// Quantizer — maps continuous values to discrete levels.
///
/// Simulates ADC quantization. Divides the range [min_val, max_val]
/// into `levels` equal bins and maps each input to the nearest bin center.
#[derive(Debug, Clone)]
pub struct Quantizer {
    levels: usize,
    min_val: f64,
    max_val: f64,
    step: f64,
}

impl Quantizer {
    pub fn new(levels: usize, min_val: f64, max_val: f64) -> Self {
        assert!(levels >= 2);
        assert!(max_val > min_val);
        let step = (max_val - min_val) / levels as f64;
        Self {
            levels,
            min_val,
            max_val,
            step,
        }
    }

    /// Create a quantizer for N-bit ADC with full-scale range [-1.0, 1.0].
    pub fn n_bit(bits: u32) -> Self {
        Self::new(1 << bits, -1.0, 1.0)
    }

    /// Quantize a single value.
    pub fn quantize(&self, input: f64) -> f64 {
        let clamped = input.clamp(self.min_val, self.max_val - 1e-15);
        let bin = ((clamped - self.min_val) / self.step) as usize;
        let bin = bin.min(self.levels - 1);
        self.min_val + (bin as f64 + 0.5) * self.step
    }

    /// Quantize a block of real samples.
    pub fn quantize_block(&self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&x| self.quantize(x)).collect()
    }

    /// Quantize complex samples (I and Q independently).
    pub fn quantize_complex(&self, input: &[Complex64]) -> Vec<Complex64> {
        input
            .iter()
            .map(|z| Complex64::new(self.quantize(z.re), self.quantize(z.im)))
            .collect()
    }

    pub fn levels(&self) -> usize {
        self.levels
    }

    pub fn step_size(&self) -> f64 {
        self.step
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sample_and_hold_basic() {
        let mut sh = SampleAndHold::new(0.0_f64);
        let data = vec![1.0, 2.0, 3.0, 4.0];
        let ctrl = vec![true, false, true, false];
        let output = sh.process(&data, &ctrl);
        assert_eq!(output, vec![1.0, 1.0, 3.0, 3.0]);
    }

    #[test]
    fn test_sample_and_hold_all_true() {
        let mut sh = SampleAndHold::new(0.0_f64);
        let data = vec![1.0, 2.0, 3.0, 4.0];
        let ctrl = vec![true, true, true, true];
        let output = sh.process(&data, &ctrl);
        assert_eq!(output, vec![1.0, 2.0, 3.0, 4.0]); // Passthrough
    }

    #[test]
    fn test_sample_and_hold_all_false() {
        let mut sh = SampleAndHold::new(99.0_f64);
        let data = vec![1.0, 2.0, 3.0];
        let ctrl = vec![false, false, false];
        let output = sh.process(&data, &ctrl);
        assert_eq!(output, vec![99.0, 99.0, 99.0]); // All held at initial
    }

    #[test]
    fn test_sample_and_hold_alternating() {
        let mut sh = SampleAndHold::new(0.0_f64);
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let ctrl = vec![true, false, true, false, true, false];
        let output = sh.process(&data, &ctrl);
        assert_eq!(output, vec![1.0, 1.0, 3.0, 3.0, 5.0, 5.0]);
    }

    #[test]
    fn test_sample_and_hold_complex() {
        let mut sh = SampleAndHold::new(Complex64::new(0.0, 0.0));
        let data = vec![
            Complex64::new(1.0, 2.0),
            Complex64::new(3.0, 4.0),
        ];
        let ctrl = vec![true, false];
        let output = sh.process(&data, &ctrl);
        assert_eq!(output[0], Complex64::new(1.0, 2.0));
        assert_eq!(output[1], Complex64::new(1.0, 2.0)); // Held
    }

    #[test]
    fn test_sample_and_hold_single() {
        let mut sh = SampleAndHold::new(0.0_f64);
        assert_eq!(sh.process_single(5.0, true), 5.0);
        assert_eq!(sh.process_single(10.0, false), 5.0); // Held
        assert_eq!(sh.process_single(20.0, true), 20.0); // Updated
    }

    #[test]
    fn test_sample_and_hold_reset() {
        let mut sh = SampleAndHold::new(0.0_f64);
        sh.process_single(42.0, true);
        assert_eq!(sh.held_value(), 42.0);
        sh.reset();
        assert_eq!(sh.held_value(), 0.0);
    }

    #[test]
    fn test_quantizer_4_levels() {
        let q = Quantizer::new(4, -1.0, 1.0);
        // Step = 0.5, bins: [-1,-0.5), [-0.5,0), [0,0.5), [0.5,1)
        // Centers: -0.75, -0.25, 0.25, 0.75
        assert!((q.quantize(-0.9) - (-0.75)).abs() < 1e-10);
        assert!((q.quantize(-0.1) - (-0.25)).abs() < 1e-10);
        assert!((q.quantize(0.1) - 0.25).abs() < 1e-10);
        assert!((q.quantize(0.9) - 0.75).abs() < 1e-10);
    }

    #[test]
    fn test_quantizer_clamps() {
        let q = Quantizer::new(4, -1.0, 1.0);
        // Values outside range should clamp
        let below = q.quantize(-5.0);
        let above = q.quantize(5.0);
        assert!((below - (-0.75)).abs() < 1e-10); // First bin center
        assert!((above - 0.75).abs() < 1e-10); // Last bin center
    }

    #[test]
    fn test_quantizer_n_bit() {
        let q = Quantizer::n_bit(3); // 8 levels
        assert_eq!(q.levels(), 8);
        assert!((q.step_size() - 0.25).abs() < 1e-10);
    }

    #[test]
    fn test_quantizer_block() {
        let q = Quantizer::new(2, 0.0, 1.0);
        let input = vec![0.1, 0.3, 0.7, 0.9];
        let output = q.quantize_block(&input);
        // 2 levels: [0, 0.5) → 0.25, [0.5, 1.0) → 0.75
        assert_eq!(output, vec![0.25, 0.25, 0.75, 0.75]);
    }

    #[test]
    fn test_quantizer_complex() {
        let q = Quantizer::new(2, -1.0, 1.0);
        let input = vec![Complex64::new(0.3, -0.8)];
        let output = q.quantize_complex(&input);
        // 2 levels: [-1,0) → -0.5, [0,1) → 0.5
        assert!((output[0].re - 0.5).abs() < 1e-10);
        assert!((output[0].im - (-0.5)).abs() < 1e-10);
    }

    #[test]
    fn test_quantizer_many_levels_approximates_identity() {
        let q = Quantizer::new(10000, -1.0, 1.0);
        let input = 0.3141;
        let output = q.quantize(input);
        assert!((output - input).abs() < 0.001);
    }
}
