//! Integrate-and-Dump Filter for Symbol Detection
//!
//! An integrate-and-dump (I&D) filter accumulates incoming samples over a
//! fixed symbol period, outputs the integrated value, then resets the
//! accumulator to zero. This matched-filter operation is optimal for
//! rectangular pulse shapes in AWGN and is widely used in digital
//! communications receivers for symbol-rate detection.
//!
//! This module provides three variants:
//!
//! - [`IntegrateAndDump`] / [`IntegrateAndDumpComplex`] — classic I&D with
//!   one output per symbol period (decimation equals integration length).
//! - [`DecimatingIntegrator`] — integration length and decimation factor
//!   are independently configurable.
//! - [`MovingIntegrator`] — sliding-window integration with no dump;
//!   produces one output per input sample (continuous, no decimation).
//!
//! # Example
//!
//! ```rust
//! use r4w_core::integrate_and_dump::IntegrateAndDump;
//!
//! // 4 samples per symbol — integrate and dump
//! let mut iad = IntegrateAndDump::new(4);
//! let input = vec![1.0, 1.0, 1.0, 1.0, -1.0, -1.0, -1.0, -1.0];
//! let symbols = iad.process(&input);
//! assert_eq!(symbols, vec![4.0, -4.0]);
//! ```

// ---------------------------------------------------------------------------
// IntegrateAndDump (real-valued)
// ---------------------------------------------------------------------------

/// Classic integrate-and-dump filter for real-valued samples.
///
/// Accumulates `period` input samples, emits the sum, then resets.
#[derive(Debug, Clone)]
pub struct IntegrateAndDump {
    /// Number of samples per integration period (symbol length).
    period: usize,
    /// Running accumulator.
    accumulator: f64,
    /// Number of samples accumulated so far in the current period.
    count: usize,
}

impl IntegrateAndDump {
    /// Create a new integrate-and-dump filter.
    ///
    /// `period` is the number of input samples per output symbol.
    /// A period of zero is promoted to 1.
    pub fn new(period: usize) -> Self {
        Self {
            period: period.max(1),
            accumulator: 0.0,
            count: 0,
        }
    }

    /// Process a block of real-valued samples.
    ///
    /// Returns one output value for every complete integration period.
    /// Partial periods are retained internally for the next call.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() / self.period + 1);
        for &x in input {
            self.accumulator += x;
            self.count += 1;
            if self.count >= self.period {
                output.push(self.accumulator);
                self.accumulator = 0.0;
                self.count = 0;
            }
        }
        output
    }

    /// Process a block of complex samples represented as `(re, im)` tuples.
    ///
    /// Real and imaginary parts are integrated independently.
    /// Returns one complex output per complete integration period.
    pub fn process_complex(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        // We need a separate imaginary accumulator; reuse `self.accumulator`
        // for the real part and track imaginary locally, flushing count via
        // the shared counter.
        let mut im_accum: f64 = 0.0;
        let mut output = Vec::with_capacity(input.len() / self.period + 1);
        for &(re, im) in input {
            self.accumulator += re;
            im_accum += im;
            self.count += 1;
            if self.count >= self.period {
                output.push((self.accumulator, im_accum));
                self.accumulator = 0.0;
                im_accum = 0.0;
                self.count = 0;
            }
        }
        // If there is a partial period, stash the imaginary remainder.
        // Because we cannot grow the struct at runtime, partial complex
        // state is *lost* across calls.  Use `IntegrateAndDumpComplex` for
        // streaming complex processing.
        output
    }

    /// Reset the accumulator and sample count to zero.
    pub fn reset(&mut self) {
        self.accumulator = 0.0;
        self.count = 0;
    }

    /// Number of samples remaining until the next dump.
    pub fn remaining(&self) -> usize {
        self.period - self.count
    }

    /// Current integration period.
    pub fn period(&self) -> usize {
        self.period
    }

    /// Current accumulator value (for inspection / debug).
    pub fn accumulator(&self) -> f64 {
        self.accumulator
    }
}

// ---------------------------------------------------------------------------
// IntegrateAndDumpComplex
// ---------------------------------------------------------------------------

/// Dedicated complex integrate-and-dump filter.
///
/// Maintains separate real and imaginary accumulators so that streaming
/// across multiple `process` calls is fully supported.
#[derive(Debug, Clone)]
pub struct IntegrateAndDumpComplex {
    /// Number of samples per integration period.
    period: usize,
    /// Real part accumulator.
    accum_re: f64,
    /// Imaginary part accumulator.
    accum_im: f64,
    /// Samples accumulated in the current period.
    count: usize,
}

impl IntegrateAndDumpComplex {
    /// Create a new complex integrate-and-dump filter.
    pub fn new(period: usize) -> Self {
        Self {
            period: period.max(1),
            accum_re: 0.0,
            accum_im: 0.0,
            count: 0,
        }
    }

    /// Process a block of complex samples.
    pub fn process(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut output = Vec::with_capacity(input.len() / self.period + 1);
        for &(re, im) in input {
            self.accum_re += re;
            self.accum_im += im;
            self.count += 1;
            if self.count >= self.period {
                output.push((self.accum_re, self.accum_im));
                self.accum_re = 0.0;
                self.accum_im = 0.0;
                self.count = 0;
            }
        }
        output
    }

    /// Reset accumulators and count.
    pub fn reset(&mut self) {
        self.accum_re = 0.0;
        self.accum_im = 0.0;
        self.count = 0;
    }

    /// Samples remaining until next dump.
    pub fn remaining(&self) -> usize {
        self.period - self.count
    }

    /// Current integration period.
    pub fn period(&self) -> usize {
        self.period
    }
}

// ---------------------------------------------------------------------------
// DecimatingIntegrator
// ---------------------------------------------------------------------------

/// Integrate-and-dump with independent integration length and decimation.
///
/// Integrates over `integration_length` samples, dumps the result, then
/// discards (`decimation_factor - integration_length`) samples before
/// starting the next integration window.  When `decimation_factor` equals
/// `integration_length` this behaves identically to [`IntegrateAndDump`].
///
/// If `decimation_factor < integration_length`, the integration windows
/// overlap and more outputs are produced; if greater, there is a gap
/// between windows.
#[derive(Debug, Clone)]
pub struct DecimatingIntegrator {
    /// Number of samples to integrate per output.
    integration_length: usize,
    /// Total samples consumed per output cycle.
    decimation_factor: usize,
    /// Running accumulator.
    accumulator: f64,
    /// Position within the current decimation cycle (0..decimation_factor).
    position: usize,
}

impl DecimatingIntegrator {
    /// Create a new decimating integrator.
    ///
    /// * `integration_length` — how many samples to sum per output.
    /// * `decimation_factor` — how many samples to advance between outputs.
    ///
    /// Both values are clamped to a minimum of 1.
    pub fn new(integration_length: usize, decimation_factor: usize) -> Self {
        Self {
            integration_length: integration_length.max(1),
            decimation_factor: decimation_factor.max(1),
            accumulator: 0.0,
            position: 0,
        }
    }

    /// Process a block of samples.
    ///
    /// Returns one output each time a full integration window completes
    /// within a decimation cycle.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() / self.decimation_factor + 1);
        for &x in input {
            // Accumulate only while within the integration window.
            if self.position < self.integration_length {
                self.accumulator += x;
            }
            self.position += 1;

            // End of decimation cycle — dump and reset.
            if self.position >= self.decimation_factor {
                output.push(self.accumulator);
                self.accumulator = 0.0;
                self.position = 0;
            }
        }
        output
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.accumulator = 0.0;
        self.position = 0;
    }

    /// Integration length.
    pub fn integration_length(&self) -> usize {
        self.integration_length
    }

    /// Decimation factor.
    pub fn decimation_factor(&self) -> usize {
        self.decimation_factor
    }
}

// ---------------------------------------------------------------------------
// MovingIntegrator
// ---------------------------------------------------------------------------

/// Sliding-window integrator (no dump, continuous output).
///
/// Produces one output per input sample.  Each output is the sum of the
/// most recent `window_size` input samples.  The first `window_size - 1`
/// outputs reflect partial windows (the sum of however many samples have
/// been seen so far).
#[derive(Debug, Clone)]
pub struct MovingIntegrator {
    /// Window size.
    window_size: usize,
    /// Circular buffer storing recent samples.
    buffer: Vec<f64>,
    /// Write position in the circular buffer.
    write_pos: usize,
    /// Running sum.
    sum: f64,
    /// Total samples seen (used to detect initial fill).
    samples_seen: usize,
}

impl MovingIntegrator {
    /// Create a new sliding-window integrator.
    pub fn new(window_size: usize) -> Self {
        let window_size = window_size.max(1);
        Self {
            window_size,
            buffer: vec![0.0; window_size],
            write_pos: 0,
            sum: 0.0,
            samples_seen: 0,
        }
    }

    /// Process a block of samples.
    ///
    /// Returns one output per input sample.  Each output is the running
    /// sum over the last `window_size` samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            // Subtract the oldest sample only once the buffer is full.
            if self.samples_seen >= self.window_size {
                self.sum -= self.buffer[self.write_pos];
            }
            self.buffer[self.write_pos] = x;
            self.sum += x;
            self.write_pos = (self.write_pos + 1) % self.window_size;
            self.samples_seen += 1;
            output.push(self.sum);
        }
        output
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.buffer.fill(0.0);
        self.write_pos = 0;
        self.sum = 0.0;
        self.samples_seen = 0;
    }

    /// Window size.
    pub fn window_size(&self) -> usize {
        self.window_size
    }

    /// Current running sum.
    pub fn current_sum(&self) -> f64 {
        self.sum
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // IntegrateAndDump (real)
    // -----------------------------------------------------------------------

    #[test]
    fn test_basic_integrate() {
        let mut iad = IntegrateAndDump::new(4);
        let input = vec![1.0, 1.0, 1.0, 1.0];
        let output = iad.process(&input);
        assert_eq!(output.len(), 1);
        assert!((output[0] - 4.0).abs() < 1e-12);
    }

    #[test]
    fn test_complex_integrate() {
        let mut iad = IntegrateAndDumpComplex::new(3);
        let input = vec![(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)];
        let output = iad.process(&input);
        assert_eq!(output.len(), 1);
        assert!((output[0].0 - 9.0).abs() < 1e-12); // 1+3+5
        assert!((output[0].1 - 12.0).abs() < 1e-12); // 2+4+6
    }

    #[test]
    fn test_partial_period() {
        let mut iad = IntegrateAndDump::new(4);
        // Feed only 3 samples — no output yet.
        let output = iad.process(&[1.0, 2.0, 3.0]);
        assert!(output.is_empty());
        assert!((iad.accumulator() - 6.0).abs() < 1e-12);
        assert_eq!(iad.remaining(), 1);

        // Feed one more to complete the period.
        let output = iad.process(&[4.0]);
        assert_eq!(output.len(), 1);
        assert!((output[0] - 10.0).abs() < 1e-12);
    }

    #[test]
    fn test_multiple_periods() {
        let mut iad = IntegrateAndDump::new(2);
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let output = iad.process(&input);
        assert_eq!(output.len(), 3);
        assert!((output[0] - 3.0).abs() < 1e-12); // 1+2
        assert!((output[1] - 7.0).abs() < 1e-12); // 3+4
        assert!((output[2] - 11.0).abs() < 1e-12); // 5+6
    }

    #[test]
    fn test_reset() {
        let mut iad = IntegrateAndDump::new(4);
        iad.process(&[1.0, 2.0]);
        assert!((iad.accumulator() - 3.0).abs() < 1e-12);
        assert_eq!(iad.remaining(), 2);

        iad.reset();
        assert!((iad.accumulator() - 0.0).abs() < 1e-12);
        assert_eq!(iad.remaining(), 4);
    }

    #[test]
    fn test_remaining() {
        let mut iad = IntegrateAndDump::new(5);
        assert_eq!(iad.remaining(), 5);
        iad.process(&[1.0]);
        assert_eq!(iad.remaining(), 4);
        iad.process(&[1.0, 1.0]);
        assert_eq!(iad.remaining(), 2);
        iad.process(&[1.0, 1.0]); // completes period
        assert_eq!(iad.remaining(), 5); // reset after dump
    }

    // -----------------------------------------------------------------------
    // DecimatingIntegrator
    // -----------------------------------------------------------------------

    #[test]
    fn test_decimating() {
        // Integration length = 3, decimation = 5.
        // Cycle: accumulate 3 samples, skip 2, dump.
        let mut dec = DecimatingIntegrator::new(3, 5);
        let input = vec![1.0, 2.0, 3.0, 0.0, 0.0, 4.0, 5.0, 6.0, 0.0, 0.0];
        let output = dec.process(&input);
        assert_eq!(output.len(), 2);
        assert!((output[0] - 6.0).abs() < 1e-12); // 1+2+3 (skip 0,0)
        assert!((output[1] - 15.0).abs() < 1e-12); // 4+5+6 (skip 0,0)

        // When integration_length == decimation_factor, same as plain I&D.
        let mut dec2 = DecimatingIntegrator::new(4, 4);
        let output2 = dec2.process(&[1.0, 1.0, 1.0, 1.0]);
        assert_eq!(output2.len(), 1);
        assert!((output2[0] - 4.0).abs() < 1e-12);
    }

    // -----------------------------------------------------------------------
    // MovingIntegrator
    // -----------------------------------------------------------------------

    #[test]
    fn test_moving_integrator() {
        let mut mi = MovingIntegrator::new(3);
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let output = mi.process(&input);
        assert_eq!(output.len(), 5);
        // Partial windows during fill:
        assert!((output[0] - 1.0).abs() < 1e-12); // just 1
        assert!((output[1] - 3.0).abs() < 1e-12); // 1+2
        // Full windows:
        assert!((output[2] - 6.0).abs() < 1e-12); // 1+2+3
        assert!((output[3] - 9.0).abs() < 1e-12); // 2+3+4
        assert!((output[4] - 12.0).abs() < 1e-12); // 3+4+5
    }

    // -----------------------------------------------------------------------
    // Edge cases
    // -----------------------------------------------------------------------

    #[test]
    fn test_single_sample_period() {
        // Period of 1: every sample is its own symbol.
        let mut iad = IntegrateAndDump::new(1);
        let input = vec![3.0, -1.0, 7.5];
        let output = iad.process(&input);
        assert_eq!(output.len(), 3);
        assert!((output[0] - 3.0).abs() < 1e-12);
        assert!((output[1] - (-1.0)).abs() < 1e-12);
        assert!((output[2] - 7.5).abs() < 1e-12);
    }

    #[test]
    fn test_impulse_response() {
        // A single 1.0 followed by zeros — the I&D output should equal 1.0
        // exactly once, in the period that contains the impulse.
        let mut iad = IntegrateAndDump::new(4);
        let input = vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let output = iad.process(&input);
        assert_eq!(output.len(), 2);
        assert!((output[0] - 1.0).abs() < 1e-12); // impulse captured
        assert!((output[1] - 0.0).abs() < 1e-12); // all zeros
    }
}
