//! Polyphase Filter Implementations
//!
//! Provides efficient sample rate conversion using polyphase decomposition.
//!
//! ## Overview
//!
//! Polyphase filters restructure FIR operations to minimize computation:
//! - **Decimation**: Filter at high rate, keep every M-th sample → O(N) per output
//! - **Interpolation**: Insert zeros, filter → O(N) per output
//!
//! By decomposing the filter into polyphase components, we operate at the
//! lower sample rate, reducing computation by a factor of M or L.
//!
//! ## Example
//!
//! ```rust,ignore
//! use r4w_core::filters::{PolyphaseDecimator, PolyphaseInterpolator, Resampler};
//! use num_complex::Complex64;
//!
//! // Decimate by 4 (e.g., 1 MHz → 250 kHz)
//! let mut decimator = PolyphaseDecimator::new(4, 64);
//! let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 1000];
//! let output = decimator.process(&input);
//! assert_eq!(output.len(), 250);
//!
//! // Interpolate by 3 (e.g., 100 kHz → 300 kHz)
//! let mut interpolator = PolyphaseInterpolator::new(3, 48);
//! let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 100];
//! let output = interpolator.process(&input);
//! assert_eq!(output.len(), 300);
//!
//! // Rational resampling: 3/4 (e.g., 48 kHz → 36 kHz)
//! let mut resampler = Resampler::new(3, 4, 96);
//! ```

use super::windows::Window;
use num_complex::Complex64;
use std::f64::consts::PI;

/// Polyphase decimator for efficient downsampling.
///
/// Reduces sample rate by factor M while applying anti-aliasing filter.
/// Uses polyphase decomposition to operate at the output rate.
///
/// ## Structure
///
/// ```text
/// Input (M samples) → [Polyphase Filter Bank] → 1 Output sample
///                     (M branches, N/M taps each)
/// ```
#[derive(Debug, Clone)]
pub struct PolyphaseDecimator {
    /// Decimation factor
    factor: usize,
    /// Polyphase filter branches (factor x taps_per_branch)
    branches: Vec<Vec<f64>>,
    /// Delay lines for each branch (complex)
    delay_lines: Vec<Vec<Complex64>>,
    /// Delay lines for real processing
    delay_lines_real: Vec<Vec<f64>>,
    /// Current input phase (0 to factor-1)
    phase: usize,
    /// Accumulated samples for current output
    accumulator: Complex64,
    /// Real accumulator
    accumulator_real: f64,
    /// Samples accumulated count
    samples_accumulated: usize,
}

impl PolyphaseDecimator {
    /// Create a new polyphase decimator.
    ///
    /// # Arguments
    /// * `factor` - Decimation factor M (output_rate = input_rate / M)
    /// * `num_taps` - Total number of filter taps (will be rounded to multiple of factor)
    ///
    /// # Example
    /// ```rust,ignore
    /// // Decimate by 4 with 64-tap filter
    /// let decimator = PolyphaseDecimator::new(4, 64);
    /// ```
    pub fn new(factor: usize, num_taps: usize) -> Self {
        assert!(factor > 0, "Decimation factor must be positive");
        assert!(num_taps > 0, "Number of taps must be positive");

        // Design lowpass filter with cutoff at 1/(2*factor) of input Nyquist
        let cutoff = 0.5 / factor as f64;
        let prototype = design_lowpass_prototype(num_taps, cutoff);

        Self::from_coefficients(factor, &prototype)
    }

    /// Create a decimator with custom filter coefficients.
    ///
    /// # Arguments
    /// * `factor` - Decimation factor M
    /// * `coefficients` - FIR filter coefficients
    pub fn from_coefficients(factor: usize, coefficients: &[f64]) -> Self {
        assert!(factor > 0, "Decimation factor must be positive");
        assert!(!coefficients.is_empty(), "Coefficients cannot be empty");

        // Pad to multiple of factor
        let padded_len = ((coefficients.len() + factor - 1) / factor) * factor;
        let mut padded = vec![0.0; padded_len];
        padded[..coefficients.len()].copy_from_slice(coefficients);

        let taps_per_branch = padded_len / factor;

        // Decompose into polyphase branches
        // Branch k gets coefficients h[k], h[k+M], h[k+2M], ...
        let mut branches = vec![vec![0.0; taps_per_branch]; factor];
        for (i, &coeff) in padded.iter().enumerate() {
            let branch = i % factor;
            let tap = i / factor;
            branches[branch][tap] = coeff;
        }

        let delay_lines = vec![vec![Complex64::new(0.0, 0.0); taps_per_branch]; factor];
        let delay_lines_real = vec![vec![0.0; taps_per_branch]; factor];

        Self {
            factor,
            branches,
            delay_lines,
            delay_lines_real,
            phase: 0,
            accumulator: Complex64::new(0.0, 0.0),
            accumulator_real: 0.0,
            samples_accumulated: 0,
        }
    }

    /// Get the decimation factor.
    pub fn factor(&self) -> usize {
        self.factor
    }

    /// Get the number of taps per polyphase branch.
    pub fn taps_per_branch(&self) -> usize {
        self.branches.first().map(|b| b.len()).unwrap_or(0)
    }

    /// Get total number of filter taps.
    pub fn num_taps(&self) -> usize {
        self.factor * self.taps_per_branch()
    }

    /// Process a block of complex samples.
    ///
    /// # Returns
    /// Decimated output (length = input.len() / factor, rounded down)
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len() / self.factor + 1);

        for &sample in input {
            if let Some(out) = self.process_sample(sample) {
                output.push(out);
            }
        }

        output
    }

    /// Process a single complex sample.
    ///
    /// Returns Some(output) every M input samples.
    pub fn process_sample(&mut self, input: Complex64) -> Option<Complex64> {
        // Shift sample into the appropriate branch's delay line
        let branch = &self.branches[self.phase];
        let delay = &mut self.delay_lines[self.phase];

        // Shift delay line
        for i in (1..delay.len()).rev() {
            delay[i] = delay[i - 1];
        }
        delay[0] = input;

        // Compute branch output and accumulate
        let mut branch_output = Complex64::new(0.0, 0.0);
        for (i, &coeff) in branch.iter().enumerate() {
            branch_output += delay[i] * coeff;
        }
        self.accumulator += branch_output;
        self.samples_accumulated += 1;

        // Advance phase
        self.phase = (self.phase + 1) % self.factor;

        // Output when we've processed M samples
        if self.samples_accumulated >= self.factor {
            let result = self.accumulator;
            self.accumulator = Complex64::new(0.0, 0.0);
            self.samples_accumulated = 0;
            Some(result)
        } else {
            None
        }
    }

    /// Process a block of real samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() / self.factor + 1);

        for &sample in input {
            if let Some(out) = self.process_sample_real(sample) {
                output.push(out);
            }
        }

        output
    }

    /// Process a single real sample.
    pub fn process_sample_real(&mut self, input: f64) -> Option<f64> {
        let branch = &self.branches[self.phase];
        let delay = &mut self.delay_lines_real[self.phase];

        // Shift delay line
        for i in (1..delay.len()).rev() {
            delay[i] = delay[i - 1];
        }
        delay[0] = input;

        // Compute branch output
        let mut branch_output = 0.0;
        for (i, &coeff) in branch.iter().enumerate() {
            branch_output += delay[i] * coeff;
        }
        self.accumulator_real += branch_output;
        self.samples_accumulated += 1;

        self.phase = (self.phase + 1) % self.factor;

        if self.samples_accumulated >= self.factor {
            let result = self.accumulator_real;
            self.accumulator_real = 0.0;
            self.samples_accumulated = 0;
            Some(result)
        } else {
            None
        }
    }

    /// Reset the filter state.
    pub fn reset(&mut self) {
        for delay in &mut self.delay_lines {
            for s in delay.iter_mut() {
                *s = Complex64::new(0.0, 0.0);
            }
        }
        for delay in &mut self.delay_lines_real {
            for s in delay.iter_mut() {
                *s = 0.0;
            }
        }
        self.phase = 0;
        self.accumulator = Complex64::new(0.0, 0.0);
        self.accumulator_real = 0.0;
        self.samples_accumulated = 0;
    }

    /// Get the group delay in output samples.
    pub fn group_delay(&self) -> f64 {
        (self.num_taps() - 1) as f64 / 2.0 / self.factor as f64
    }
}

/// Polyphase interpolator for efficient upsampling.
///
/// Increases sample rate by factor L while applying anti-imaging filter.
/// Uses polyphase decomposition to avoid computing zero-valued products.
///
/// ## Structure
///
/// ```text
/// 1 Input sample → [Polyphase Filter Bank] → L Output samples
///                  (L branches, N/L taps each)
/// ```
#[derive(Debug, Clone)]
pub struct PolyphaseInterpolator {
    /// Interpolation factor
    factor: usize,
    /// Polyphase filter branches (factor x taps_per_branch)
    branches: Vec<Vec<f64>>,
    /// Delay line (shared across branches)
    delay_line: Vec<Complex64>,
    /// Real delay line
    delay_line_real: Vec<f64>,
    /// Current output phase
    output_phase: usize,
    /// Gain compensation (L for unity passband gain)
    gain: f64,
}

impl PolyphaseInterpolator {
    /// Create a new polyphase interpolator.
    ///
    /// # Arguments
    /// * `factor` - Interpolation factor L (output_rate = input_rate * L)
    /// * `num_taps` - Total number of filter taps (will be rounded to multiple of factor)
    ///
    /// # Example
    /// ```rust,ignore
    /// // Interpolate by 3 with 48-tap filter
    /// let interpolator = PolyphaseInterpolator::new(3, 48);
    /// ```
    pub fn new(factor: usize, num_taps: usize) -> Self {
        assert!(factor > 0, "Interpolation factor must be positive");
        assert!(num_taps > 0, "Number of taps must be positive");

        // Design lowpass filter with cutoff at 1/(2*factor) of output Nyquist
        let cutoff = 0.5 / factor as f64;
        let prototype = design_lowpass_prototype(num_taps, cutoff);

        Self::from_coefficients(factor, &prototype)
    }

    /// Create an interpolator with custom filter coefficients.
    ///
    /// # Arguments
    /// * `factor` - Interpolation factor L
    /// * `coefficients` - FIR filter coefficients
    pub fn from_coefficients(factor: usize, coefficients: &[f64]) -> Self {
        assert!(factor > 0, "Interpolation factor must be positive");
        assert!(!coefficients.is_empty(), "Coefficients cannot be empty");

        // Pad to multiple of factor
        let padded_len = ((coefficients.len() + factor - 1) / factor) * factor;
        let mut padded = vec![0.0; padded_len];
        padded[..coefficients.len()].copy_from_slice(coefficients);

        let taps_per_branch = padded_len / factor;

        // Decompose into polyphase branches
        // For interpolation, branch ordering is reversed compared to decimation
        let mut branches = vec![vec![0.0; taps_per_branch]; factor];
        for (i, &coeff) in padded.iter().enumerate() {
            let branch = i % factor;
            let tap = i / factor;
            // Reverse branch order for interpolation
            branches[factor - 1 - branch][tap] = coeff;
        }

        let delay_line = vec![Complex64::new(0.0, 0.0); taps_per_branch];
        let delay_line_real = vec![0.0; taps_per_branch];

        Self {
            factor,
            branches,
            delay_line,
            delay_line_real,
            output_phase: 0,
            gain: factor as f64, // Compensate for zero-stuffing
        }
    }

    /// Get the interpolation factor.
    pub fn factor(&self) -> usize {
        self.factor
    }

    /// Get the number of taps per polyphase branch.
    pub fn taps_per_branch(&self) -> usize {
        self.branches.first().map(|b| b.len()).unwrap_or(0)
    }

    /// Get total number of filter taps.
    pub fn num_taps(&self) -> usize {
        self.factor * self.taps_per_branch()
    }

    /// Process a block of complex samples.
    ///
    /// # Returns
    /// Interpolated output (length = input.len() * factor)
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len() * self.factor);

        for &sample in input {
            output.extend(self.process_sample(sample));
        }

        output
    }

    /// Process a single complex sample.
    ///
    /// Returns L output samples for each input sample.
    pub fn process_sample(&mut self, input: Complex64) -> Vec<Complex64> {
        // Shift new sample into delay line
        for i in (1..self.delay_line.len()).rev() {
            self.delay_line[i] = self.delay_line[i - 1];
        }
        self.delay_line[0] = input;

        // Generate L output samples from L branches
        let mut outputs = Vec::with_capacity(self.factor);
        for branch in &self.branches {
            let mut sum = Complex64::new(0.0, 0.0);
            for (i, &coeff) in branch.iter().enumerate() {
                sum += self.delay_line[i] * coeff;
            }
            outputs.push(sum * self.gain);
        }

        outputs
    }

    /// Process a block of real samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() * self.factor);

        for &sample in input {
            output.extend(self.process_sample_real(sample));
        }

        output
    }

    /// Process a single real sample.
    pub fn process_sample_real(&mut self, input: f64) -> Vec<f64> {
        // Shift new sample into delay line
        for i in (1..self.delay_line_real.len()).rev() {
            self.delay_line_real[i] = self.delay_line_real[i - 1];
        }
        self.delay_line_real[0] = input;

        // Generate L output samples
        let mut outputs = Vec::with_capacity(self.factor);
        for branch in &self.branches {
            let mut sum = 0.0;
            for (i, &coeff) in branch.iter().enumerate() {
                sum += self.delay_line_real[i] * coeff;
            }
            outputs.push(sum * self.gain);
        }

        outputs
    }

    /// Reset the filter state.
    pub fn reset(&mut self) {
        for s in self.delay_line.iter_mut() {
            *s = Complex64::new(0.0, 0.0);
        }
        for s in self.delay_line_real.iter_mut() {
            *s = 0.0;
        }
        self.output_phase = 0;
    }

    /// Get the group delay in input samples.
    pub fn group_delay(&self) -> f64 {
        (self.num_taps() - 1) as f64 / 2.0 / self.factor as f64
    }
}

/// Rational resampler for L/M rate conversion.
///
/// Combines interpolation by L and decimation by M for efficient
/// arbitrary rate conversion. Uses polyphase decomposition.
///
/// ## Example
///
/// ```rust,ignore
/// // Convert 48 kHz to 44.1 kHz (ratio 147/160)
/// let resampler = Resampler::new(147, 160, 1024);
/// ```
#[derive(Debug, Clone)]
pub struct Resampler {
    /// Interpolation factor
    interp_factor: usize,
    /// Decimation factor
    decim_factor: usize,
    /// Polyphase filter branches
    branches: Vec<Vec<f64>>,
    /// Delay line
    delay_line: Vec<Complex64>,
    /// Real delay line
    delay_line_real: Vec<f64>,
    /// Current phase in the polyphase filter
    phase: usize,
    /// Gain compensation
    gain: f64,
}

impl Resampler {
    /// Create a new rational resampler.
    ///
    /// # Arguments
    /// * `interp_factor` - Interpolation factor L (numerator of rate ratio)
    /// * `decim_factor` - Decimation factor M (denominator of rate ratio)
    /// * `num_taps` - Prototype filter taps (will be rounded to multiple of L)
    ///
    /// Output rate = Input rate * L / M
    ///
    /// # Example
    /// ```rust,ignore
    /// // 3/4 rate conversion (e.g., 48 kHz → 36 kHz)
    /// let resampler = Resampler::new(3, 4, 96);
    /// ```
    pub fn new(interp_factor: usize, decim_factor: usize, num_taps: usize) -> Self {
        assert!(interp_factor > 0, "Interpolation factor must be positive");
        assert!(decim_factor > 0, "Decimation factor must be positive");
        assert!(num_taps > 0, "Number of taps must be positive");

        // Use GCD to simplify ratio
        let gcd = gcd(interp_factor, decim_factor);
        let l = interp_factor / gcd;
        let m = decim_factor / gcd;

        // Design filter with cutoff at min(1/2L, 1/2M) of input Nyquist
        let cutoff = 0.5 / (l.max(m)) as f64;
        let prototype = design_lowpass_prototype(num_taps, cutoff);

        Self::from_coefficients(l, m, &prototype)
    }

    /// Create a resampler with custom filter coefficients.
    pub fn from_coefficients(
        interp_factor: usize,
        decim_factor: usize,
        coefficients: &[f64],
    ) -> Self {
        assert!(interp_factor > 0);
        assert!(decim_factor > 0);
        assert!(!coefficients.is_empty());

        // Pad to multiple of interp_factor
        let padded_len = ((coefficients.len() + interp_factor - 1) / interp_factor) * interp_factor;
        let mut padded = vec![0.0; padded_len];
        padded[..coefficients.len()].copy_from_slice(coefficients);

        let taps_per_branch = padded_len / interp_factor;

        // Decompose into polyphase branches
        let mut branches = vec![vec![0.0; taps_per_branch]; interp_factor];
        for (i, &coeff) in padded.iter().enumerate() {
            let branch = i % interp_factor;
            let tap = i / interp_factor;
            branches[branch][tap] = coeff;
        }

        let delay_line = vec![Complex64::new(0.0, 0.0); taps_per_branch];
        let delay_line_real = vec![0.0; taps_per_branch];

        Self {
            interp_factor,
            decim_factor,
            branches,
            delay_line,
            delay_line_real,
            phase: 0,
            gain: interp_factor as f64,
        }
    }

    /// Get the interpolation factor.
    pub fn interp_factor(&self) -> usize {
        self.interp_factor
    }

    /// Get the decimation factor.
    pub fn decim_factor(&self) -> usize {
        self.decim_factor
    }

    /// Get the rate ratio as a float.
    pub fn rate_ratio(&self) -> f64 {
        self.interp_factor as f64 / self.decim_factor as f64
    }

    /// Process a block of complex samples.
    ///
    /// # Returns
    /// Resampled output (length ≈ input.len() * L / M)
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let expected_len = (input.len() * self.interp_factor) / self.decim_factor + 1;
        let mut output = Vec::with_capacity(expected_len);

        for &sample in input {
            // For each input sample, we conceptually produce L outputs
            // but only keep every M-th one

            // Shift new sample into delay line
            for i in (1..self.delay_line.len()).rev() {
                self.delay_line[i] = self.delay_line[i - 1];
            }
            self.delay_line[0] = sample;

            // Generate outputs at current phase, stepping by decim_factor
            // We produce ceil(L/M) or floor(L/M) outputs per input
            for _ in 0..self.interp_factor {
                if self.phase % self.decim_factor == 0 {
                    // Compute output from this branch
                    let branch_idx = self.phase % self.interp_factor;
                    let branch = &self.branches[branch_idx];
                    let mut sum = Complex64::new(0.0, 0.0);
                    for (i, &coeff) in branch.iter().enumerate() {
                        sum += self.delay_line[i] * coeff;
                    }
                    output.push(sum * self.gain);
                }
                self.phase = (self.phase + 1) % (self.interp_factor * self.decim_factor);
            }
        }

        output
    }

    /// Process a block of real samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        let expected_len = (input.len() * self.interp_factor) / self.decim_factor + 1;
        let mut output = Vec::with_capacity(expected_len);

        for &sample in input {
            for i in (1..self.delay_line_real.len()).rev() {
                self.delay_line_real[i] = self.delay_line_real[i - 1];
            }
            self.delay_line_real[0] = sample;

            for _ in 0..self.interp_factor {
                if self.phase % self.decim_factor == 0 {
                    let branch_idx = self.phase % self.interp_factor;
                    let branch = &self.branches[branch_idx];
                    let mut sum = 0.0;
                    for (i, &coeff) in branch.iter().enumerate() {
                        sum += self.delay_line_real[i] * coeff;
                    }
                    output.push(sum * self.gain);
                }
                self.phase = (self.phase + 1) % (self.interp_factor * self.decim_factor);
            }
        }

        output
    }

    /// Reset the filter state.
    pub fn reset(&mut self) {
        for s in self.delay_line.iter_mut() {
            *s = Complex64::new(0.0, 0.0);
        }
        for s in self.delay_line_real.iter_mut() {
            *s = 0.0;
        }
        self.phase = 0;
    }
}

/// Half-band filter for efficient 2x decimation/interpolation.
///
/// Half-band filters have the special property that every other coefficient
/// is zero (except the center), reducing computation by nearly 50%.
#[derive(Debug, Clone)]
pub struct HalfbandFilter {
    /// Non-zero coefficients (half + center)
    coeffs: Vec<f64>,
    /// Delay line (complex)
    delay_line: Vec<Complex64>,
    /// Real delay line
    delay_line_real: Vec<f64>,
    /// Filter length
    length: usize,
}

impl HalfbandFilter {
    /// Create a new half-band filter.
    ///
    /// # Arguments
    /// * `num_taps` - Number of taps (should be 4k+3 for proper half-band)
    pub fn new(num_taps: usize) -> Self {
        // Ensure proper half-band form: 4k+3 taps
        let num_taps = ((num_taps - 3) / 4) * 4 + 3;

        // Design lowpass at 0.25 (half-band)
        let prototype = design_lowpass_prototype(num_taps, 0.25);

        Self::from_coefficients(&prototype)
    }

    /// Create from filter coefficients.
    pub fn from_coefficients(coefficients: &[f64]) -> Self {
        let length = coefficients.len();

        // Extract non-zero coefficients (odd indices + center)
        // For half-band: h[n] = 0 for even n ≠ center
        let mut coeffs = Vec::new();
        let center = length / 2;

        for (i, &c) in coefficients.iter().enumerate() {
            if i == center || i % 2 == 1 {
                coeffs.push(c);
            }
        }

        let delay_line = vec![Complex64::new(0.0, 0.0); length];
        let delay_line_real = vec![0.0; length];

        Self {
            coeffs,
            delay_line,
            delay_line_real,
            length,
        }
    }

    /// Process for 2x decimation.
    pub fn decimate(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len() / 2 + 1);

        for (i, &sample) in input.iter().enumerate() {
            // Shift delay line
            for j in (1..self.delay_line.len()).rev() {
                self.delay_line[j] = self.delay_line[j - 1];
            }
            self.delay_line[0] = sample;

            // Output every other sample
            if i % 2 == 1 {
                let mut sum = Complex64::new(0.0, 0.0);
                let center = self.length / 2;

                // Center tap
                sum += self.delay_line[center] * 0.5;

                // Odd taps only
                let mut coeff_idx = 0;
                for j in (1..self.length).step_by(2) {
                    if coeff_idx < self.coeffs.len() {
                        sum += self.delay_line[j] * self.coeffs[coeff_idx];
                        coeff_idx += 1;
                    }
                }

                output.push(sum);
            }
        }

        output
    }

    /// Process for 2x interpolation.
    pub fn interpolate(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len() * 2);

        for &sample in input {
            // Shift delay line by 2 (account for zero-stuffing)
            for j in (2..self.delay_line.len()).rev() {
                self.delay_line[j] = self.delay_line[j - 2];
            }
            self.delay_line[1] = Complex64::new(0.0, 0.0);
            self.delay_line[0] = sample;

            // Generate two outputs
            for phase in 0..2 {
                let mut sum = Complex64::new(0.0, 0.0);

                if phase == 0 {
                    // Even phase: only center contributes significantly
                    let center = self.length / 2;
                    sum = self.delay_line[center] * 0.5;
                } else {
                    // Odd phase: sum odd-indexed taps
                    let mut coeff_idx = 0;
                    for j in (1..self.length).step_by(2) {
                        if coeff_idx < self.coeffs.len() {
                            sum += self.delay_line[j] * self.coeffs[coeff_idx];
                            coeff_idx += 1;
                        }
                    }
                }

                output.push(sum * 2.0); // Compensate for zero-stuffing
            }
        }

        output
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        for s in self.delay_line.iter_mut() {
            *s = Complex64::new(0.0, 0.0);
        }
        for s in self.delay_line_real.iter_mut() {
            *s = 0.0;
        }
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Design a lowpass FIR prototype filter using windowed sinc.
fn design_lowpass_prototype(num_taps: usize, cutoff: f64) -> Vec<f64> {
    let num_taps = if num_taps % 2 == 0 { num_taps + 1 } else { num_taps };
    let m = (num_taps - 1) as f64;
    let mid = m / 2.0;

    let window = Window::Kaiser(8.0).generate(num_taps);
    let mut coeffs = Vec::with_capacity(num_taps);

    for i in 0..num_taps {
        let n = i as f64;
        let sinc = if (n - mid).abs() < 1e-10 {
            2.0 * PI * cutoff
        } else {
            (2.0 * PI * cutoff * (n - mid)).sin() / (n - mid)
        };
        coeffs.push(sinc * window[i]);
    }

    // Normalize
    let sum: f64 = coeffs.iter().sum();
    if sum.abs() > 1e-10 {
        for c in coeffs.iter_mut() {
            *c /= sum;
        }
    }

    coeffs
}

/// Compute greatest common divisor using Euclidean algorithm.
fn gcd(mut a: usize, mut b: usize) -> usize {
    while b != 0 {
        let t = b;
        b = a % b;
        a = t;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_decimator_creation() {
        let dec = PolyphaseDecimator::new(4, 64);
        assert_eq!(dec.factor(), 4);
        // num_taps may be slightly larger due to odd-length requirement and padding
        assert!(dec.num_taps() >= 64);
        assert_eq!(dec.num_taps() % 4, 0); // Must be multiple of factor
    }

    #[test]
    fn test_decimator_output_length() {
        let mut dec = PolyphaseDecimator::new(4, 32);
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 100];
        let output = dec.process(&input);
        assert_eq!(output.len(), 25); // 100 / 4
    }

    #[test]
    fn test_decimator_dc_passthrough() {
        let mut dec = PolyphaseDecimator::new(4, 64);

        // Feed DC signal
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 200];
        let output = dec.process(&input);

        // After settling, output should be close to 1.0
        let last_10_avg: f64 = output.iter().rev().take(10).map(|c| c.re).sum::<f64>() / 10.0;
        assert!(
            (last_10_avg - 1.0).abs() < 0.1,
            "DC passthrough failed: {}",
            last_10_avg
        );
    }

    #[test]
    fn test_interpolator_creation() {
        let interp = PolyphaseInterpolator::new(3, 48);
        assert_eq!(interp.factor(), 3);
    }

    #[test]
    fn test_interpolator_output_length() {
        let mut interp = PolyphaseInterpolator::new(3, 24);
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 100];
        let output = interp.process(&input);
        assert_eq!(output.len(), 300); // 100 * 3
    }

    #[test]
    fn test_interpolator_dc_passthrough() {
        let mut interp = PolyphaseInterpolator::new(4, 64);

        // Feed DC signal
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 50];
        let output = interp.process(&input);

        // After settling, output should be close to 1.0
        let last_20_avg: f64 = output.iter().rev().take(20).map(|c| c.re).sum::<f64>() / 20.0;
        assert!(
            (last_20_avg - 1.0).abs() < 0.2,
            "DC passthrough failed: {}",
            last_20_avg
        );
    }

    #[test]
    fn test_resampler_creation() {
        let resampler = Resampler::new(3, 4, 48);
        assert_eq!(resampler.interp_factor(), 3);
        assert_eq!(resampler.decim_factor(), 4);
        assert!((resampler.rate_ratio() - 0.75).abs() < 1e-10);
    }

    #[test]
    fn test_resampler_output_length() {
        let mut resampler = Resampler::new(3, 4, 48);
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 100];
        let output = resampler.process(&input);

        // Expected: 100 * 3 / 4 = 75
        // Allow some variance due to edge effects
        assert!(output.len() >= 70 && output.len() <= 80);
    }

    #[test]
    fn test_resampler_2x_upsample() {
        let mut resampler = Resampler::new(2, 1, 32);
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 50];
        let output = resampler.process(&input);

        // 2x upsample: output should be ~100 samples
        assert!(output.len() >= 95 && output.len() <= 105);
    }

    #[test]
    fn test_resampler_2x_downsample() {
        let mut resampler = Resampler::new(1, 2, 32);
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 100];
        let output = resampler.process(&input);

        // 2x downsample: output should be ~50 samples
        assert!(output.len() >= 45 && output.len() <= 55);
    }

    #[test]
    fn test_decimator_real() {
        let mut dec = PolyphaseDecimator::new(4, 32);
        let input: Vec<f64> = vec![1.0; 100];
        let output = dec.process_real(&input);
        assert_eq!(output.len(), 25);
    }

    #[test]
    fn test_interpolator_real() {
        let mut interp = PolyphaseInterpolator::new(3, 24);
        let input: Vec<f64> = vec![1.0; 100];
        let output = interp.process_real(&input);
        assert_eq!(output.len(), 300);
    }

    #[test]
    fn test_decimator_reset() {
        let mut dec = PolyphaseDecimator::new(4, 32);

        // Process some samples
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 100];
        dec.process(&input);

        // Reset
        dec.reset();

        // First output should be affected by reset
        let output = dec.process(&vec![Complex64::new(1.0, 0.0); 4]);
        assert!(output[0].norm() < 0.5); // Not fully settled
    }

    #[test]
    fn test_halfband_creation() {
        let hb = HalfbandFilter::new(15);
        assert_eq!(hb.length, 15);
    }

    #[test]
    fn test_halfband_decimate() {
        let mut hb = HalfbandFilter::new(15);
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 100];
        let output = hb.decimate(&input);
        assert_eq!(output.len(), 50); // 2x decimation
    }

    #[test]
    fn test_halfband_interpolate() {
        let mut hb = HalfbandFilter::new(15);
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 50];
        let output = hb.interpolate(&input);
        assert_eq!(output.len(), 100); // 2x interpolation
    }

    #[test]
    fn test_gcd() {
        assert_eq!(gcd(12, 8), 4);
        assert_eq!(gcd(17, 13), 1);
        assert_eq!(gcd(100, 25), 25);
        assert_eq!(gcd(48000, 44100), 300);
    }

    #[test]
    fn test_group_delay() {
        let dec = PolyphaseDecimator::new(4, 64);
        let delay = dec.group_delay();
        assert!(delay > 0.0);

        let interp = PolyphaseInterpolator::new(4, 64);
        let delay = interp.group_delay();
        assert!(delay > 0.0);
    }

    #[test]
    fn test_various_factors() {
        for factor in [2, 3, 4, 5, 8, 10] {
            let dec = PolyphaseDecimator::new(factor, 32);
            assert_eq!(dec.factor(), factor);

            let interp = PolyphaseInterpolator::new(factor, 32);
            assert_eq!(interp.factor(), factor);
        }
    }

    #[test]
    fn test_custom_coefficients() {
        let coeffs = vec![0.1, 0.2, 0.4, 0.2, 0.1];

        let dec = PolyphaseDecimator::from_coefficients(2, &coeffs);
        assert_eq!(dec.factor(), 2);

        let interp = PolyphaseInterpolator::from_coefficients(2, &coeffs);
        assert_eq!(interp.factor(), 2);
    }
}
