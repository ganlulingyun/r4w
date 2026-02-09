//! DPLL — Digital Phase-Locked Loop (second-order)
//!
//! General-purpose second-order digital PLL for tracking phase and
//! frequency of an input signal. Implements a PI (proportional-integral)
//! loop filter for carrier tracking, clock recovery, and frequency
//! synthesis applications.
//! GNU Radio equivalent: `dpll_bb`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::dpll::Dpll;
//!
//! let mut dpll = Dpll::new(0.01, 0.001); // BW, damping gains
//! // Feed phase errors (e.g., from a phase detector)
//! let error = 0.1;
//! dpll.advance(error);
//! let phase = dpll.phase();
//! let freq = dpll.frequency();
//! ```

use std::f64::consts::TAU;

/// Second-order digital phase-locked loop.
#[derive(Debug, Clone)]
pub struct Dpll {
    /// Current phase estimate (radians).
    phase: f64,
    /// Current frequency estimate (radians/sample).
    frequency: f64,
    /// Proportional gain (alpha).
    alpha: f64,
    /// Integral gain (beta).
    beta: f64,
    /// Maximum frequency magnitude (radians/sample).
    max_freq: f64,
    /// Minimum frequency magnitude (radians/sample).
    min_freq: f64,
}

impl Dpll {
    /// Create a DPLL with proportional and integral gains.
    ///
    /// - `alpha`: proportional gain (phase tracking bandwidth)
    /// - `beta`: integral gain (frequency tracking bandwidth)
    ///
    /// Typical: alpha=4*BW, beta=4*BW² for critically damped loop.
    pub fn new(alpha: f64, beta: f64) -> Self {
        Self {
            phase: 0.0,
            frequency: 0.0,
            alpha,
            beta,
            max_freq: TAU,
            min_freq: -TAU,
        }
    }

    /// Create from loop bandwidth and damping factor.
    ///
    /// - `loop_bw`: normalized loop bandwidth (0.0 to 1.0, typical 0.01-0.1)
    /// - `damping`: damping factor (1.0 = critically damped)
    pub fn from_bandwidth(loop_bw: f64, damping: f64) -> Self {
        let denom = 1.0 + 2.0 * damping * loop_bw + loop_bw * loop_bw;
        let alpha = 4.0 * damping * loop_bw / denom;
        let beta = 4.0 * loop_bw * loop_bw / denom;
        Self::new(alpha, beta)
    }

    /// Set frequency limits (radians/sample).
    pub fn set_freq_limits(&mut self, min_freq: f64, max_freq: f64) {
        self.min_freq = min_freq;
        self.max_freq = max_freq;
    }

    /// Advance the PLL by one sample given a phase error.
    ///
    /// Returns the current phase estimate.
    #[inline]
    pub fn advance(&mut self, error: f64) -> f64 {
        // PI loop filter
        self.frequency += self.beta * error;
        self.frequency = self.frequency.clamp(self.min_freq, self.max_freq);
        self.phase += self.frequency + self.alpha * error;
        // Wrap phase to [-pi, pi]
        self.phase = wrap_phase(self.phase);
        self.phase
    }

    /// Get current phase estimate (radians).
    pub fn phase(&self) -> f64 {
        self.phase
    }

    /// Get current frequency estimate (radians/sample).
    pub fn frequency(&self) -> f64 {
        self.frequency
    }

    /// Set phase.
    pub fn set_phase(&mut self, phase: f64) {
        self.phase = wrap_phase(phase);
    }

    /// Set frequency.
    pub fn set_frequency(&mut self, freq: f64) {
        self.frequency = freq.clamp(self.min_freq, self.max_freq);
    }

    /// Get proportional gain.
    pub fn alpha(&self) -> f64 {
        self.alpha
    }

    /// Get integral gain.
    pub fn beta(&self) -> f64 {
        self.beta
    }

    /// Set gains.
    pub fn set_gains(&mut self, alpha: f64, beta: f64) {
        self.alpha = alpha;
        self.beta = beta;
    }

    /// Reset PLL state.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.frequency = 0.0;
    }

    /// Process a block of phase errors, returning phase estimates.
    pub fn process(&mut self, errors: &[f64]) -> Vec<f64> {
        errors.iter().map(|&e| self.advance(e)).collect()
    }
}

/// Binary DPLL for clock recovery from edge transitions.
///
/// Tracks bit timing from a binary (hard-decision) signal by
/// detecting transitions and adjusting phase accordingly.
#[derive(Debug, Clone)]
pub struct BinaryDpll {
    /// Phase accumulator (0.0 to 1.0 = one bit period).
    phase: f64,
    /// Nominal frequency (1/samples_per_bit).
    nominal_freq: f64,
    /// Current frequency.
    freq: f64,
    /// Loop gain for phase correction.
    gain: f64,
    /// Previous input sample.
    prev_sample: f64,
}

impl BinaryDpll {
    /// Create a binary DPLL for clock recovery.
    ///
    /// - `samples_per_bit`: nominal samples per bit
    /// - `gain`: loop gain (0.0 to 1.0, typical 0.05-0.2)
    pub fn new(samples_per_bit: f64, gain: f64) -> Self {
        let nominal_freq = 1.0 / samples_per_bit;
        Self {
            phase: 0.0,
            nominal_freq,
            freq: nominal_freq,
            gain: gain.clamp(0.0, 1.0),
            prev_sample: 0.0,
        }
    }

    /// Process a sample, returns (output_bit, is_sample_point).
    ///
    /// `is_sample_point` is true when the phase accumulator wraps,
    /// indicating the optimal sampling instant.
    pub fn process_sample(&mut self, x: f64) -> (f64, bool) {
        let prev_phase = self.phase;
        self.phase += self.freq;

        // Detect transition (sign change)
        let transition = (x > 0.0) != (self.prev_sample > 0.0);
        if transition {
            // Phase error: how far from midpoint (0.5)?
            let error = self.phase - 0.5;
            self.freq = self.nominal_freq - self.gain * error;
        }

        self.prev_sample = x;

        // Check for sample point (phase wraps past 1.0)
        if self.phase >= 1.0 {
            self.phase -= 1.0;
            (x, true)
        } else {
            (x, prev_phase > self.phase && self.phase < 0.5)
        }
    }

    /// Process a block, returning only the samples at optimal points.
    pub fn recover_bits(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::new();
        for &x in input {
            let (val, is_sample) = self.process_sample(x);
            if is_sample {
                output.push(val);
            }
        }
        output
    }

    /// Get current phase.
    pub fn phase(&self) -> f64 {
        self.phase
    }

    /// Get current frequency.
    pub fn frequency(&self) -> f64 {
        self.freq
    }

    /// Reset.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.freq = self.nominal_freq;
        self.prev_sample = 0.0;
    }
}

/// Wrap phase to [-pi, pi].
#[inline]
fn wrap_phase(mut phase: f64) -> f64 {
    while phase > std::f64::consts::PI {
        phase -= TAU;
    }
    while phase < -std::f64::consts::PI {
        phase += TAU;
    }
    phase
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dpll_basic() {
        let mut dpll = Dpll::new(0.1, 0.01);
        assert_eq!(dpll.phase(), 0.0);
        assert_eq!(dpll.frequency(), 0.0);
    }

    #[test]
    fn test_dpll_tracks_constant_error() {
        let mut dpll = Dpll::new(0.1, 0.01);
        // Feed constant positive error — phase and freq should increase
        for _ in 0..100 {
            dpll.advance(0.5);
        }
        assert!(dpll.frequency() > 0.0);
    }

    #[test]
    fn test_dpll_zero_error() {
        let mut dpll = Dpll::new(0.1, 0.01);
        for _ in 0..100 {
            dpll.advance(0.0);
        }
        assert_eq!(dpll.phase(), 0.0);
        assert_eq!(dpll.frequency(), 0.0);
    }

    #[test]
    fn test_dpll_from_bandwidth() {
        let dpll = Dpll::from_bandwidth(0.05, 1.0);
        assert!(dpll.alpha() > 0.0);
        assert!(dpll.beta() > 0.0);
        assert!(dpll.alpha() > dpll.beta()); // alpha > beta for typical BWs
    }

    #[test]
    fn test_dpll_freq_limits() {
        let mut dpll = Dpll::new(0.1, 0.5);
        dpll.set_freq_limits(-0.1, 0.1);
        // Large positive errors
        for _ in 0..1000 {
            dpll.advance(1.0);
        }
        assert!(dpll.frequency() <= 0.1);
    }

    #[test]
    fn test_dpll_reset() {
        let mut dpll = Dpll::new(0.1, 0.01);
        dpll.advance(1.0);
        dpll.advance(1.0);
        dpll.reset();
        assert_eq!(dpll.phase(), 0.0);
        assert_eq!(dpll.frequency(), 0.0);
    }

    #[test]
    fn test_dpll_set_gains() {
        let mut dpll = Dpll::new(0.1, 0.01);
        dpll.set_gains(0.2, 0.02);
        assert_eq!(dpll.alpha(), 0.2);
        assert_eq!(dpll.beta(), 0.02);
    }

    #[test]
    fn test_dpll_process_block() {
        let mut dpll = Dpll::new(0.1, 0.01);
        let errors = vec![0.1, -0.1, 0.1, -0.1, 0.0];
        let phases = dpll.process(&errors);
        assert_eq!(phases.len(), 5);
    }

    #[test]
    fn test_dpll_phase_wrapping() {
        let mut dpll = Dpll::new(0.5, 0.0);
        // Large phase jump should be wrapped
        for _ in 0..100 {
            dpll.advance(1.0);
        }
        assert!(dpll.phase() >= -std::f64::consts::PI);
        assert!(dpll.phase() <= std::f64::consts::PI);
    }

    #[test]
    fn test_binary_dpll_basic() {
        let dpll = BinaryDpll::new(4.0, 0.1);
        assert!((dpll.frequency() - 0.25).abs() < 1e-10);
    }

    #[test]
    fn test_binary_dpll_recovery() {
        let mut dpll = BinaryDpll::new(4.0, 0.1);
        // Generate a simple signal: 4 samples per bit, alternating
        let mut signal = Vec::new();
        for bit in &[1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0] {
            for _ in 0..4 {
                signal.push(*bit);
            }
        }
        let recovered = dpll.recover_bits(&signal);
        assert!(!recovered.is_empty(), "should recover some bits");
    }

    #[test]
    fn test_binary_dpll_reset() {
        let mut dpll = BinaryDpll::new(8.0, 0.05);
        dpll.process_sample(1.0);
        dpll.process_sample(-1.0);
        dpll.reset();
        assert_eq!(dpll.phase(), 0.0);
    }

    #[test]
    fn test_wrap_phase() {
        assert!((wrap_phase(0.0) - 0.0).abs() < 1e-10);
        assert!((wrap_phase(TAU) - 0.0).abs() < 1e-10);
        assert!((wrap_phase(-TAU) - 0.0).abs() < 1e-10);
        let p = wrap_phase(std::f64::consts::PI + 0.1);
        assert!(p < 0.0); // Should wrap to negative
    }
}
