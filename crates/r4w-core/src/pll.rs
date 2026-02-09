//! Phase Locked Loop (PLL)
//!
//! General-purpose PLL for carrier tracking, clock recovery, and frequency
//! synthesis. Uses the NCO as the core oscillator with a configurable
//! loop filter.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::pll::{Pll, PllConfig};
//! use num_complex::Complex64;
//! use std::f64::consts::PI;
//!
//! // Create a PLL to track a carrier at ~1kHz, sample rate 48kHz
//! let config = PllConfig {
//!     loop_bw: 100.0,       // 100 Hz loop bandwidth
//!     damping: 0.707,       // Critically damped
//!     sample_rate: 48000.0,
//!     ..Default::default()
//! };
//! let mut pll = Pll::new(config);
//!
//! // Feed in a complex signal and track the carrier
//! let signal: Vec<Complex64> = (0..480)
//!     .map(|i| {
//!         let phase = 2.0 * PI * 1000.0 * i as f64 / 48000.0;
//!         Complex64::new(phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! for &s in &signal {
//!     let (output, _freq_est) = pll.step(s);
//! }
//!
//! // After convergence, frequency estimate should be near 1000 Hz
//! let freq_est = pll.frequency_estimate();
//! // (Convergence depends on loop BW and signal characteristics)
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// PLL configuration.
#[derive(Debug, Clone)]
pub struct PllConfig {
    /// Loop bandwidth in Hz. Controls lock speed vs noise rejection.
    pub loop_bw: f64,
    /// Damping factor (0.707 = critically damped, typical).
    pub damping: f64,
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Maximum frequency in Hz (limits VCO range).
    pub max_freq: f64,
    /// Minimum frequency in Hz.
    pub min_freq: f64,
}

impl Default for PllConfig {
    fn default() -> Self {
        Self {
            loop_bw: 100.0,
            damping: 0.707,
            sample_rate: 48000.0,
            max_freq: 24000.0,
            min_freq: -24000.0,
        }
    }
}

/// Phase Locked Loop.
///
/// Implements a second-order PLL with proportional + integral loop filter:
///
/// ```text
/// Phase Error: e[n] = angle(input[n] * conj(nco_output))
/// Loop Filter: f[n] = alpha * e[n] + beta * integral(e)
/// NCO Update:  freq += f[n]
/// ```
///
/// The loop filter coefficients are derived from the natural frequency
/// and damping factor:
/// ```text
/// omega_n = 2 * pi * loop_bw / (damping + 1/(4*damping))
/// alpha = 4 * damping * omega_n / (1 + 2*damping*omega_n + omega_n^2)
/// beta = 4 * omega_n^2 / (1 + 2*damping*omega_n + omega_n^2)
/// ```
#[derive(Debug, Clone)]
pub struct Pll {
    config: PllConfig,
    /// NCO phase (radians)
    phase: f64,
    /// NCO frequency (radians/sample)
    freq: f64,
    /// Proportional gain
    alpha: f64,
    /// Integral gain
    beta: f64,
    /// Integrator state
    integrator: f64,
    /// Phase error (for monitoring)
    phase_error: f64,
    /// Lock detector: running average of |phase_error|
    lock_metric: f64,
    /// Lock detector smoothing
    lock_alpha: f64,
}

impl Pll {
    /// Create a new PLL with the given configuration.
    pub fn new(config: PllConfig) -> Self {
        // Compute loop filter coefficients using the standard second-order
        // PLL design (Gardner, "Phaselock Techniques", 3rd ed).
        //
        // omega_n = normalized natural frequency (rad/sample)
        // alpha = proportional gain = 2 * damping * omega_n
        // beta  = integral gain = omega_n^2
        let omega_n = 2.0 * PI * config.loop_bw / config.sample_rate
            / (config.damping + 1.0 / (4.0 * config.damping));

        let alpha = 2.0 * config.damping * omega_n;
        let beta = omega_n * omega_n;

        Self {
            config,
            phase: 0.0,
            freq: 0.0,
            alpha,
            beta,
            integrator: 0.0,
            phase_error: 0.0,
            lock_metric: 1.0,
            lock_alpha: 0.01,
        }
    }

    /// Process one input sample.
    ///
    /// Returns (NCO output, frequency estimate in Hz).
    pub fn step(&mut self, input: Complex64) -> (Complex64, f64) {
        // Generate NCO output
        let nco_out = Complex64::new(self.phase.cos(), self.phase.sin());

        // Phase detector: cross product for phase error
        let error_complex = input * nco_out.conj();
        self.phase_error = error_complex.arg();

        // Update lock detector
        self.lock_metric = (1.0 - self.lock_alpha) * self.lock_metric
            + self.lock_alpha * self.phase_error.abs();

        // Loop filter (proportional + integral)
        // freq = alpha * error + integrator  (PI controller)
        self.integrator += self.beta * self.phase_error;
        self.freq = self.alpha * self.phase_error + self.integrator;

        // Clamp frequency
        let max_freq_rad = 2.0 * PI * self.config.max_freq / self.config.sample_rate;
        let min_freq_rad = 2.0 * PI * self.config.min_freq / self.config.sample_rate;
        self.freq = self.freq.clamp(min_freq_rad, max_freq_rad);

        // Advance phase
        self.phase += self.freq;
        if self.phase > PI {
            self.phase -= 2.0 * PI;
        } else if self.phase < -PI {
            self.phase += 2.0 * PI;
        }

        let freq_hz = self.freq * self.config.sample_rate / (2.0 * PI);
        (nco_out, freq_hz)
    }

    /// Process a block of samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.step(s).0).collect()
    }

    /// Get the current frequency estimate in Hz.
    pub fn frequency_estimate(&self) -> f64 {
        self.freq * self.config.sample_rate / (2.0 * PI)
    }

    /// Get the current phase.
    pub fn phase(&self) -> f64 {
        self.phase
    }

    /// Get the last phase error.
    pub fn phase_error(&self) -> f64 {
        self.phase_error
    }

    /// Check if the PLL is locked (phase error is small).
    pub fn is_locked(&self, threshold: f64) -> bool {
        self.lock_metric < threshold
    }

    /// Get the lock metric (lower = better lock).
    pub fn lock_metric(&self) -> f64 {
        self.lock_metric
    }

    /// Set the initial frequency estimate.
    pub fn set_frequency(&mut self, freq_hz: f64) {
        self.freq = 2.0 * PI * freq_hz / self.config.sample_rate;
    }

    /// Reset the PLL.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.freq = 0.0;
        self.integrator = 0.0;
        self.phase_error = 0.0;
        self.lock_metric = 1.0;
    }
}

/// DC Blocker.
///
/// Removes DC offset from a signal using a single-pole IIR highpass filter:
/// ```text
/// y[n] = x[n] - x[n-1] + alpha * y[n-1]
/// ```
///
/// Alpha close to 1.0 preserves more low-frequency content.
/// Typical value: 0.998 for narrowband DC removal.
#[derive(Debug, Clone)]
pub struct DcBlocker {
    alpha: f64,
    prev_input: Complex64,
    prev_output: Complex64,
}

impl DcBlocker {
    /// Create a new DC blocker.
    pub fn new(alpha: f64) -> Self {
        Self {
            alpha,
            prev_input: Complex64::new(0.0, 0.0),
            prev_output: Complex64::new(0.0, 0.0),
        }
    }

    /// Process a single sample.
    pub fn process(&mut self, input: Complex64) -> Complex64 {
        let output = input - self.prev_input + self.alpha * self.prev_output;
        self.prev_input = input;
        self.prev_output = output;
        output
    }

    /// Process a block of samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    /// Process in place.
    pub fn process_inplace(&mut self, samples: &mut [Complex64]) {
        for s in samples.iter_mut() {
            *s = self.process(*s);
        }
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.prev_input = Complex64::new(0.0, 0.0);
        self.prev_output = Complex64::new(0.0, 0.0);
    }
}

impl crate::filters::Filter for DcBlocker {
    fn process(&mut self, input: Complex64) -> Complex64 {
        DcBlocker::process(self, input)
    }

    fn reset(&mut self) {
        DcBlocker::reset(self);
    }

    fn group_delay(&self) -> f64 {
        0.5 // Approximately half-sample delay
    }

    fn order(&self) -> usize {
        1
    }
}

/// Sample Delay.
///
/// Delays a signal by a fixed number of samples using a circular buffer.
#[derive(Debug, Clone)]
pub struct SampleDelay {
    buffer: Vec<Complex64>,
    write_idx: usize,
    delay: usize,
}

impl SampleDelay {
    /// Create a new delay of the given number of samples.
    pub fn new(delay: usize) -> Self {
        Self {
            buffer: vec![Complex64::new(0.0, 0.0); delay.max(1)],
            write_idx: 0,
            delay: delay.max(1),
        }
    }

    /// Process a single sample.
    pub fn process(&mut self, input: Complex64) -> Complex64 {
        let output = self.buffer[self.write_idx];
        self.buffer[self.write_idx] = input;
        self.write_idx = (self.write_idx + 1) % self.delay;
        output
    }

    /// Process a block of samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    /// Get the delay in samples.
    pub fn delay(&self) -> usize {
        self.delay
    }

    /// Reset (fill buffer with zeros).
    pub fn reset(&mut self) {
        self.buffer.fill(Complex64::new(0.0, 0.0));
        self.write_idx = 0;
    }
}

impl crate::filters::Filter for SampleDelay {
    fn process(&mut self, input: Complex64) -> Complex64 {
        SampleDelay::process(self, input)
    }

    fn reset(&mut self) {
        SampleDelay::reset(self);
    }

    fn group_delay(&self) -> f64 {
        self.delay as f64
    }

    fn order(&self) -> usize {
        self.delay
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pll_tracks_tone() {
        let sr = 48000.0;
        let freq = 1000.0;
        let config = PllConfig {
            loop_bw: 200.0,
            damping: 0.707,
            sample_rate: sr,
            ..Default::default()
        };
        let mut pll = Pll::new(config);

        // Generate test tone
        let samples: Vec<Complex64> = (0..4800)
            .map(|i| {
                let phase = 2.0 * PI * freq * i as f64 / sr;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        for &s in &samples {
            pll.step(s);
        }

        let est = pll.frequency_estimate();
        assert!(
            (est - freq).abs() < 50.0,
            "PLL should track 1 kHz tone: got {est:.1} Hz"
        );
    }

    #[test]
    fn test_pll_lock_indicator() {
        let sr = 48000.0;
        let config = PllConfig {
            loop_bw: 200.0,
            sample_rate: sr,
            ..Default::default()
        };
        let mut pll = Pll::new(config);

        // Feed clean tone for convergence
        for i in 0..9600 {
            let phase = 2.0 * PI * 500.0 * i as f64 / sr;
            pll.step(Complex64::new(phase.cos(), phase.sin()));
        }

        assert!(
            pll.lock_metric() < 0.5,
            "PLL should be locked after convergence: lock_metric = {:.3}",
            pll.lock_metric()
        );
    }

    #[test]
    fn test_pll_set_frequency() {
        let config = PllConfig {
            sample_rate: 48000.0,
            ..Default::default()
        };
        let mut pll = Pll::new(config);
        pll.set_frequency(1000.0);
        assert!(
            (pll.frequency_estimate() - 1000.0).abs() < 0.1,
            "Set frequency should work"
        );
    }

    #[test]
    fn test_pll_reset() {
        let mut pll = Pll::new(PllConfig::default());
        pll.set_frequency(1000.0);
        pll.reset();
        assert!(pll.frequency_estimate().abs() < 0.1);
    }

    #[test]
    fn test_dc_blocker_removes_dc() {
        let mut blocker = DcBlocker::new(0.998);

        // DC signal — need enough samples for IIR to converge
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.5); 5000];
        let output = blocker.process_block(&input);

        // After convergence, output should be near zero
        let last = output.last().unwrap();
        assert!(
            last.norm() < 0.05,
            "DC blocker should remove DC: got {:.3}",
            last.norm()
        );
    }

    #[test]
    fn test_dc_blocker_passes_ac() {
        let mut blocker = DcBlocker::new(0.998);

        // AC signal (500 Hz at 48kHz)
        let sr = 48000.0;
        let input: Vec<Complex64> = (0..1000)
            .map(|i| {
                let phase = 2.0 * PI * 500.0 * i as f64 / sr;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        let output = blocker.process_block(&input);

        // AC should pass through (after initial transient)
        let output_power: f64 = output[100..].iter().map(|s| s.norm_sqr()).sum::<f64>()
            / (output.len() - 100) as f64;
        assert!(
            output_power > 0.5,
            "DC blocker should pass AC signal: power = {output_power:.3}"
        );
    }

    #[test]
    fn test_dc_blocker_filter_trait() {
        use crate::filters::Filter;
        let mut blocker = DcBlocker::new(0.998);
        let output = Filter::process(&mut blocker, Complex64::new(1.0, 0.0));
        // First sample should pass through (approximately)
        assert!(output.norm() > 0.5);
    }

    #[test]
    fn test_sample_delay() {
        let mut delay = SampleDelay::new(3);

        // First 3 outputs should be zero (buffer initialized to zero)
        assert!(delay.process(Complex64::new(1.0, 0.0)).norm() < 1e-10);
        assert!(delay.process(Complex64::new(2.0, 0.0)).norm() < 1e-10);
        assert!(delay.process(Complex64::new(3.0, 0.0)).norm() < 1e-10);

        // Now we should get the delayed values
        let out = delay.process(Complex64::new(4.0, 0.0));
        assert!((out.re - 1.0).abs() < 1e-10, "Should get first input delayed by 3");

        let out = delay.process(Complex64::new(5.0, 0.0));
        assert!((out.re - 2.0).abs() < 1e-10, "Should get second input delayed by 3");
    }

    #[test]
    fn test_sample_delay_block() {
        let mut delay = SampleDelay::new(2);
        let input = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(2.0, 0.0),
            Complex64::new(3.0, 0.0),
            Complex64::new(4.0, 0.0),
        ];
        let output = delay.process_block(&input);

        assert!(output[0].norm() < 1e-10); // zero (delay)
        assert!(output[1].norm() < 1e-10); // zero (delay)
        assert!((output[2].re - 1.0).abs() < 1e-10); // input[0]
        assert!((output[3].re - 2.0).abs() < 1e-10); // input[1]
    }

    #[test]
    fn test_sample_delay_filter_trait() {
        use crate::filters::Filter;
        let mut delay = SampleDelay::new(5);
        assert!((delay.group_delay() - 5.0).abs() < 1e-10);
        let _ = Filter::process(&mut delay, Complex64::new(1.0, 0.0));
    }

    #[test]
    fn test_sample_delay_reset() {
        let mut delay = SampleDelay::new(2);
        let _ = delay.process(Complex64::new(1.0, 0.0));
        let _ = delay.process(Complex64::new(2.0, 0.0));
        delay.reset();
        // After reset, should output zeros again
        assert!(delay.process(Complex64::new(3.0, 0.0)).norm() < 1e-10);
    }
}
