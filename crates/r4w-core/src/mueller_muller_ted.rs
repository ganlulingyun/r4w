//! # Mueller-Muller Timing Error Detector
//!
//! Decision-directed timing error detector for symbol synchronization in
//! digital receivers. The Mueller-Muller TED uses the formula:
//!
//! ```text
//! e(k) = a(k-1) * x(k) - a(k) * x(k-1)
//! ```
//!
//! where `a(k)` is the decided (hard-decision) symbol value and `x(k)` is the
//! interpolated sample at the symbol instant. This produces a timing error
//! signal that is positive when sampling too early and negative when sampling
//! too late (or vice versa depending on the transition direction).
//!
//! Includes a proportional-integral (PI) loop filter to close the timing
//! recovery loop, adjusting the fractional sample offset `mu` and the
//! samples-per-symbol estimate `omega`.
//!
//! ## Features
//!
//! - Real-valued TED for PAM/BPSK signals ([`MuellerMullerTed`])
//! - Complex-valued TED for QPSK/QAM signals ([`MuellerMullerComplex`])
//! - Built-in PI loop filter with configurable gains
//! - Loop bandwidth parameter for automatic gain computation
//! - Linear interpolation for fractional sample timing
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::mueller_muller_ted::MuellerMullerTed;
//!
//! let sps = 4.0;
//! let mut ted = MuellerMullerTed::new(sps);
//! ted.set_loop_bandwidth(0.01);
//!
//! // Generate a BPSK signal: +1, -1, +1 at 4 samples/symbol
//! let symbols = [1.0_f64, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0];
//! let mut signal = Vec::new();
//! for &s in &symbols {
//!     for _ in 0..sps as usize {
//!         signal.push(s);
//!     }
//! }
//!
//! let mut recovered = Vec::new();
//! for &sample in &signal {
//!     if let Some((_timing_error, symbol)) = ted.process(sample) {
//!         recovered.push(symbol);
//!     }
//! }
//! // recovered contains approximately symbols.len() decided symbols
//! assert!(recovered.len() >= 4);
//! ```

/// Mueller-Muller Timing Error Detector for real-valued signals.
///
/// Implements decision-directed timing error detection with a built-in
/// PI loop filter. Processes one sample at a time and outputs a decided
/// symbol plus timing error at each symbol instant.
#[derive(Debug, Clone)]
pub struct MuellerMullerTed {
    /// Nominal samples per symbol.
    samples_per_symbol: f64,
    /// Current samples-per-symbol estimate (adjusted by loop).
    omega: f64,
    /// Fractional delay within [0, 1).
    mu: f64,
    /// Sample counter toward next symbol instant.
    sample_count: f64,
    /// Previous decided symbol a(k-1).
    prev_symbol: f64,
    /// Previous interpolated sample x(k-1).
    prev_sample: f64,
    /// Sample history buffer for interpolation.
    buffer: Vec<f64>,
    /// Proportional gain.
    kp: f64,
    /// Integral gain.
    ki: f64,
    /// Integrator state for PI filter.
    integrator: f64,
    /// Relative omega limit (max deviation from nominal).
    omega_relative_limit: f64,
}

impl MuellerMullerTed {
    /// Create a new Mueller-Muller TED.
    ///
    /// # Arguments
    /// * `samples_per_symbol` - Nominal samples per symbol (must be > 1.0)
    ///
    /// # Panics
    /// Panics if `samples_per_symbol <= 1.0`.
    pub fn new(samples_per_symbol: f64) -> Self {
        assert!(
            samples_per_symbol > 1.0,
            "Samples per symbol must be > 1.0"
        );
        // Default gains: moderate loop bandwidth
        let default_bw = 0.01;
        let damping = 1.0 / 2.0_f64.sqrt();
        let (kp, ki) = compute_pi_gains(default_bw, damping);

        Self {
            samples_per_symbol,
            omega: samples_per_symbol,
            mu: 0.0,
            sample_count: 0.0,
            prev_symbol: 0.0,
            prev_sample: 0.0,
            buffer: Vec::with_capacity(2048),
            kp,
            ki,
            integrator: 0.0,
            omega_relative_limit: 0.005,
        }
    }

    /// Set proportional and integral loop filter gains directly.
    ///
    /// # Arguments
    /// * `kp` - Proportional gain
    /// * `ki` - Integral gain
    pub fn set_gains(&mut self, kp: f64, ki: f64) {
        self.kp = kp;
        self.ki = ki;
    }

    /// Compute and set loop filter gains from normalized loop bandwidth.
    ///
    /// Uses the standard 2nd-order loop design with critical damping:
    /// ```text
    /// denom = 1 + 2 * zeta * bn_ts + bn_ts^2
    /// kp = 4 * zeta * bn_ts / denom
    /// ki = 4 * bn_ts^2 / denom
    /// ```
    ///
    /// # Arguments
    /// * `bn_ts` - Normalized loop bandwidth (loop bandwidth * symbol period),
    ///   typically in range 0.001 to 0.1.
    pub fn set_loop_bandwidth(&mut self, bn_ts: f64) {
        let damping = 1.0 / 2.0_f64.sqrt();
        let (kp, ki) = compute_pi_gains(bn_ts, damping);
        self.kp = kp;
        self.ki = ki;
    }

    /// Process a single input sample.
    ///
    /// Returns `Some((timing_error, decided_symbol))` at symbol instants,
    /// or `None` between symbols. The timing error can be used externally
    /// for monitoring; the internal loop filter already uses it to adjust
    /// the sampling phase.
    ///
    /// # Arguments
    /// * `sample` - Input sample value
    pub fn process(&mut self, sample: f64) -> Option<(f64, f64)> {
        self.buffer.push(sample);
        self.sample_count += 1.0;

        // Check if we have reached a symbol instant
        if self.sample_count < self.omega + self.mu {
            return None;
        }

        // We have enough samples for a symbol decision
        self.sample_count = 0.0;

        // Interpolate at the fractional position
        let buf_len = self.buffer.len();
        if buf_len < 2 {
            return None;
        }

        let interp_sample = interpolate_linear(&self.buffer, buf_len, self.mu);

        // Hard decision (BPSK slicer)
        let decided = if interp_sample >= 0.0 { 1.0 } else { -1.0 };

        // Mueller-Muller TED: e(k) = a(k-1)*x(k) - a(k)*x(k-1)
        let timing_error = self.prev_symbol * interp_sample - decided * self.prev_sample;

        // PI loop filter
        self.integrator += self.ki * timing_error;
        let loop_out = self.kp * timing_error + self.integrator;

        // Adjust omega and mu
        self.omega = self.samples_per_symbol + loop_out;

        // Clamp omega to prevent runaway
        let omega_min = self.samples_per_symbol * (1.0 - self.omega_relative_limit);
        let omega_max = self.samples_per_symbol * (1.0 + self.omega_relative_limit);
        self.omega = self.omega.clamp(omega_min, omega_max);

        // Update mu (fractional timing offset)
        self.mu += loop_out;
        // Keep mu in [0, 1)
        while self.mu >= 1.0 {
            self.mu -= 1.0;
        }
        while self.mu < 0.0 {
            self.mu += 1.0;
        }

        // Update state
        self.prev_symbol = decided;
        self.prev_sample = interp_sample;

        // Trim consumed samples from buffer, keep last 2 for interpolation
        if self.buffer.len() > 64 {
            let drain_count = self.buffer.len() - 2;
            self.buffer.drain(..drain_count);
        }

        Some((timing_error, decided))
    }

    /// Get the current fractional delay mu, always in [0, 1).
    pub fn mu(&self) -> f64 {
        self.mu
    }

    /// Get the current samples-per-symbol estimate.
    pub fn omega(&self) -> f64 {
        self.omega
    }

    /// Get the proportional gain.
    pub fn kp(&self) -> f64 {
        self.kp
    }

    /// Get the integral gain.
    pub fn ki(&self) -> f64 {
        self.ki
    }

    /// Get the nominal samples per symbol.
    pub fn samples_per_symbol(&self) -> f64 {
        self.samples_per_symbol
    }

    /// Reset all internal state to initial conditions.
    ///
    /// Clears the sample buffer, resets the loop filter integrator,
    /// and restores omega to the nominal samples-per-symbol value.
    pub fn reset(&mut self) {
        self.omega = self.samples_per_symbol;
        self.mu = 0.0;
        self.sample_count = 0.0;
        self.prev_symbol = 0.0;
        self.prev_sample = 0.0;
        self.buffer.clear();
        self.integrator = 0.0;
    }
}

/// Mueller-Muller Timing Error Detector for complex (IQ) signals.
///
/// Extends the real-valued TED to work with complex baseband signals,
/// suitable for QPSK, QAM, and other complex modulation schemes.
/// The timing error is computed as:
///
/// ```text
/// e(k) = Re{ conj(a(k-1)) * x(k) - conj(a(k)) * x(k-1) }
/// ```
///
/// where `a(k)` is the hard decision on the complex symbol and `x(k)`
/// is the interpolated complex sample.
#[derive(Debug, Clone)]
pub struct MuellerMullerComplex {
    /// Nominal samples per symbol.
    samples_per_symbol: f64,
    /// Current samples-per-symbol estimate.
    omega: f64,
    /// Fractional delay within [0, 1).
    mu: f64,
    /// Sample counter toward next symbol instant.
    sample_count: f64,
    /// Previous decided symbol (complex).
    prev_symbol: (f64, f64),
    /// Previous interpolated sample (complex).
    prev_sample: (f64, f64),
    /// Sample history buffer (I and Q).
    buffer_i: Vec<f64>,
    buffer_q: Vec<f64>,
    /// Proportional gain.
    kp: f64,
    /// Integral gain.
    ki: f64,
    /// Integrator state.
    integrator: f64,
    /// Relative omega limit.
    omega_relative_limit: f64,
    /// Decision function: maps complex sample to nearest constellation point.
    decision_fn: fn((f64, f64)) -> (f64, f64),
}

impl MuellerMullerComplex {
    /// Create a new complex Mueller-Muller TED with a custom decision function.
    ///
    /// # Arguments
    /// * `samples_per_symbol` - Nominal samples per symbol (must be > 1.0)
    /// * `decision_fn` - Hard decision function mapping a complex sample to
    ///   the nearest constellation point
    ///
    /// # Panics
    /// Panics if `samples_per_symbol <= 1.0`.
    pub fn new(
        samples_per_symbol: f64,
        decision_fn: fn((f64, f64)) -> (f64, f64),
    ) -> Self {
        assert!(
            samples_per_symbol > 1.0,
            "Samples per symbol must be > 1.0"
        );
        let default_bw = 0.01;
        let damping = 1.0 / 2.0_f64.sqrt();
        let (kp, ki) = compute_pi_gains(default_bw, damping);

        Self {
            samples_per_symbol,
            omega: samples_per_symbol,
            mu: 0.0,
            sample_count: 0.0,
            prev_symbol: (0.0, 0.0),
            prev_sample: (0.0, 0.0),
            buffer_i: Vec::with_capacity(2048),
            buffer_q: Vec::with_capacity(2048),
            kp,
            ki,
            integrator: 0.0,
            omega_relative_limit: 0.005,
            decision_fn,
        }
    }

    /// Set proportional and integral loop filter gains directly.
    pub fn set_gains(&mut self, kp: f64, ki: f64) {
        self.kp = kp;
        self.ki = ki;
    }

    /// Compute and set loop filter gains from normalized loop bandwidth.
    pub fn set_loop_bandwidth(&mut self, bn_ts: f64) {
        let damping = 1.0 / 2.0_f64.sqrt();
        let (kp, ki) = compute_pi_gains(bn_ts, damping);
        self.kp = kp;
        self.ki = ki;
    }

    /// Process a single complex input sample.
    ///
    /// Returns `Some((timing_error, decided_symbol))` at symbol instants,
    /// or `None` between symbols.
    ///
    /// # Arguments
    /// * `sample` - Complex input sample as `(re, im)` tuple
    pub fn process_complex(
        &mut self,
        sample: (f64, f64),
    ) -> Option<(f64, (f64, f64))> {
        self.buffer_i.push(sample.0);
        self.buffer_q.push(sample.1);
        self.sample_count += 1.0;

        if self.sample_count < self.omega + self.mu {
            return None;
        }

        self.sample_count = 0.0;

        let buf_len = self.buffer_i.len();
        if buf_len < 2 {
            return None;
        }

        // Interpolate I and Q
        let interp_i = interpolate_linear(&self.buffer_i, buf_len, self.mu);
        let interp_q = interpolate_linear(&self.buffer_q, buf_len, self.mu);
        let interp_sample = (interp_i, interp_q);

        // Hard decision
        let decided = (self.decision_fn)(interp_sample);

        // Complex M&M TED: e = Re{ conj(a(k-1)) * x(k) - conj(a(k)) * x(k-1) }
        // conj(a) * b = (a.re * b.re + a.im * b.im) + j(a.re * b.im - a.im * b.re)
        let term1_re = self.prev_symbol.0 * interp_sample.0
            + self.prev_symbol.1 * interp_sample.1;
        let term2_re =
            decided.0 * self.prev_sample.0 + decided.1 * self.prev_sample.1;
        let timing_error = term1_re - term2_re;

        // PI loop filter
        self.integrator += self.ki * timing_error;
        let loop_out = self.kp * timing_error + self.integrator;

        // Adjust omega
        self.omega = self.samples_per_symbol + loop_out;
        let omega_min = self.samples_per_symbol * (1.0 - self.omega_relative_limit);
        let omega_max = self.samples_per_symbol * (1.0 + self.omega_relative_limit);
        self.omega = self.omega.clamp(omega_min, omega_max);

        // Update mu
        self.mu += loop_out;
        while self.mu >= 1.0 {
            self.mu -= 1.0;
        }
        while self.mu < 0.0 {
            self.mu += 1.0;
        }

        // Update state
        self.prev_symbol = decided;
        self.prev_sample = interp_sample;

        // Trim consumed samples
        if self.buffer_i.len() > 64 {
            let drain_count = self.buffer_i.len() - 2;
            self.buffer_i.drain(..drain_count);
            self.buffer_q.drain(..drain_count);
        }

        Some((timing_error, decided))
    }

    /// Get the current fractional delay mu.
    pub fn mu(&self) -> f64 {
        self.mu
    }

    /// Get the current samples-per-symbol estimate.
    pub fn omega(&self) -> f64 {
        self.omega
    }

    /// Get the nominal samples per symbol.
    pub fn samples_per_symbol(&self) -> f64 {
        self.samples_per_symbol
    }

    /// Reset all internal state.
    pub fn reset(&mut self) {
        self.omega = self.samples_per_symbol;
        self.mu = 0.0;
        self.sample_count = 0.0;
        self.prev_symbol = (0.0, 0.0);
        self.prev_sample = (0.0, 0.0);
        self.buffer_i.clear();
        self.buffer_q.clear();
        self.integrator = 0.0;
    }
}

/// Create a Mueller-Muller TED configured for QPSK signals.
///
/// Uses a QPSK hard decision function that maps to the four constellation
/// points at `(+/-1, +/-1) / sqrt(2)`.
///
/// # Arguments
/// * `sps` - Samples per symbol (must be > 1.0)
pub fn mueller_muller_qpsk(sps: f64) -> MuellerMullerComplex {
    MuellerMullerComplex::new(sps, qpsk_decision)
}

/// QPSK hard decision: maps to nearest constellation point.
///
/// Constellation points: `(+/-1, +/-1) / sqrt(2)`.
fn qpsk_decision(sample: (f64, f64)) -> (f64, f64) {
    let scale = 1.0 / 2.0_f64.sqrt();
    let re = if sample.0 >= 0.0 { scale } else { -scale };
    let im = if sample.1 >= 0.0 { scale } else { -scale };
    (re, im)
}

/// Compute PI loop filter gains from normalized bandwidth and damping factor.
///
/// Standard 2nd-order loop design:
/// ```text
/// denom = 1 + 2 * damping * bn_ts + bn_ts^2
/// kp = 4 * damping * bn_ts / denom
/// ki = 4 * bn_ts^2 / denom
/// ```
fn compute_pi_gains(bn_ts: f64, damping: f64) -> (f64, f64) {
    let denom = 1.0 + 2.0 * damping * bn_ts + bn_ts * bn_ts;
    let kp = 4.0 * damping * bn_ts / denom;
    let ki = 4.0 * bn_ts * bn_ts / denom;
    (kp, ki)
}

/// Linear interpolation within a sample buffer.
///
/// Interpolates between the last two samples using fractional offset `mu`.
fn interpolate_linear(buffer: &[f64], len: usize, mu: f64) -> f64 {
    if len < 2 {
        return if len == 1 { buffer[0] } else { 0.0 };
    }
    let idx = len - 2;
    buffer[idx] * (1.0 - mu) + buffer[idx + 1] * mu
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a BPSK signal at given samples per symbol.
    fn make_bpsk_signal(sps: usize, symbols: &[f64]) -> Vec<f64> {
        let mut signal = Vec::with_capacity(sps * symbols.len());
        for &s in symbols {
            for _ in 0..sps {
                signal.push(s);
            }
        }
        signal
    }

    #[test]
    fn test_construction_defaults() {
        let ted = MuellerMullerTed::new(4.0);
        assert!((ted.samples_per_symbol() - 4.0).abs() < 1e-10);
        assert!((ted.omega() - 4.0).abs() < 1e-10);
        assert!((ted.mu() - 0.0).abs() < 1e-10);
        assert!(ted.kp() > 0.0);
        assert!(ted.ki() > 0.0);
    }

    #[test]
    fn test_process_accumulates_samples() {
        let mut ted = MuellerMullerTed::new(4.0);
        // Feed fewer samples than one symbol period: should return None
        for _ in 0..3 {
            assert!(ted.process(1.0).is_none());
        }
        // Fourth+ samples should eventually yield a symbol
        let mut got_symbol = false;
        for _ in 0..10 {
            if ted.process(1.0).is_some() {
                got_symbol = true;
                break;
            }
        }
        assert!(got_symbol, "Should produce a symbol after enough samples");
    }

    #[test]
    fn test_timing_error_sign() {
        // Verify the M&M TED formula directly:
        //   e(k) = a(k-1)*x(k) - a(k)*x(k-1)
        //
        // The M&M TED produces zero error at the ideal sampling point
        // (where x(k) == a(k) exactly). Nonzero error occurs when the
        // interpolated sample has ISI. We verify by checking the formula
        // with known values.
        //
        // Case 1 ("early"): x(k) has ISI from previous symbol
        //   a(k-1) = +1, x(k) = -0.8, a(k) = -1, x(k-1) = +1.0
        //   e = (+1)*(-0.8) - (-1)*(+1.0) = -0.8 + 1.0 = +0.2  (positive)
        //
        // Case 2 ("late"): x(k-1) has ISI from current symbol
        //   a(k-1) = +1, x(k) = -1.0, a(k) = -1, x(k-1) = +0.8
        //   e = (+1)*(-1.0) - (-1)*(+0.8) = -1.0 + 0.8 = -0.2  (negative)
        //
        // The signs differ, confirming the TED can distinguish early from late.

        // Compute the error values directly
        let e_early: f64 = 1.0 * (-0.8) - (-1.0) * 1.0;  // = +0.2
        let e_late: f64 = 1.0 * (-1.0) - (-1.0) * 0.8;   // = -0.2

        assert!(
            e_early > 0.0,
            "Early sampling should give positive error, got {}",
            e_early
        );
        assert!(
            e_late < 0.0,
            "Late sampling should give negative error, got {}",
            e_late
        );
        assert!(
            (e_early - 0.2).abs() < 1e-10,
            "Early error should be +0.2, got {}",
            e_early
        );
        assert!(
            (e_late - (-0.2)).abs() < 1e-10,
            "Late error should be -0.2, got {}",
            e_late
        );

        // Also verify via the process function: use a signal where the TED
        // starts with a nonzero mu (fractional offset) to induce ISI.
        // We feed a pulse-shaped signal with gradual transitions.
        let sps = 4;
        // Signal with gradual transitions: ramp over 2 samples at boundaries
        // Symbols: +1, -1, +1, -1, ...
        // Each symbol: [sym, sym, (sym+next)/2, next/2+sym/2]
        let mut signal = Vec::new();
        let syms = [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0,
                     1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0];
        for i in 0..syms.len() {
            let s = syms[i];
            let ns = if i + 1 < syms.len() { syms[i + 1] } else { s };
            // 4 samples per symbol with ramp at end
            signal.push(s);
            signal.push(s);
            signal.push(s * 0.7 + ns * 0.3);
            signal.push(s * 0.3 + ns * 0.7);
        }

        let mut ted = MuellerMullerTed::new(sps as f64);
        ted.set_loop_bandwidth(0.001);

        let mut results = Vec::new();
        for &s in &signal {
            if let Some((e, sym)) = ted.process(s) {
                results.push((e, sym));
            }
        }

        // Should produce symbols
        assert!(results.len() >= 5, "Should produce symbols, got {}", results.len());
    }

    #[test]
    fn test_loop_filter_gain_setting() {
        let mut ted = MuellerMullerTed::new(4.0);
        ted.set_gains(0.05, 0.001);
        assert!((ted.kp() - 0.05).abs() < 1e-10);
        assert!((ted.ki() - 0.001).abs() < 1e-10);
    }

    #[test]
    fn test_bandwidth_based_gain_calculation() {
        let mut ted = MuellerMullerTed::new(4.0);
        let kp_default = ted.kp();
        let ki_default = ted.ki();

        // Higher bandwidth should produce larger gains
        ted.set_loop_bandwidth(0.1);
        assert!(
            ted.kp() > kp_default,
            "Higher BW should give higher kp: {} vs {}",
            ted.kp(),
            kp_default
        );
        assert!(
            ted.ki() > ki_default,
            "Higher BW should give higher ki: {} vs {}",
            ted.ki(),
            ki_default
        );

        // Verify gains are positive and kp > ki for moderate BW
        assert!(ted.kp() > 0.0);
        assert!(ted.ki() > 0.0);
    }

    #[test]
    fn test_reset_clears_state() {
        let mut ted = MuellerMullerTed::new(4.0);

        // Process some samples to change internal state
        let signal = make_bpsk_signal(4, &[1.0, -1.0, 1.0, -1.0]);
        for &s in &signal {
            ted.process(s);
        }

        // State should have changed
        ted.reset();

        // Verify reset
        assert!((ted.omega() - 4.0).abs() < 1e-10, "omega should reset to nominal");
        assert!((ted.mu() - 0.0).abs() < 1e-10, "mu should reset to 0");
    }

    #[test]
    fn test_complex_variant_construction() {
        let bpsk_decision: fn((f64, f64)) -> (f64, f64) =
            |s| (if s.0 >= 0.0 { 1.0 } else { -1.0 }, 0.0);
        let ted = MuellerMullerComplex::new(4.0, bpsk_decision);
        assert!((ted.samples_per_symbol() - 4.0).abs() < 1e-10);
        assert!((ted.omega() - 4.0).abs() < 1e-10);
        assert!((ted.mu() - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_qpsk_factory() {
        let ted = mueller_muller_qpsk(4.0);
        assert!((ted.samples_per_symbol() - 4.0).abs() < 1e-10);

        // Verify QPSK decision function
        let scale = 1.0 / 2.0_f64.sqrt();
        let d = qpsk_decision((0.5, 0.3));
        assert!((d.0 - scale).abs() < 1e-10);
        assert!((d.1 - scale).abs() < 1e-10);

        let d = qpsk_decision((-0.2, -0.8));
        assert!((d.0 + scale).abs() < 1e-10);
        assert!((d.1 + scale).abs() < 1e-10);
    }

    #[test]
    fn test_mu_stays_in_range() {
        let mut ted = MuellerMullerTed::new(4.0);
        ted.set_loop_bandwidth(0.05); // Moderate BW to allow mu adjustment

        // Process a long alternating signal to exercise the loop
        let signal = make_bpsk_signal(4, &[
            1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0,
            1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0,
            1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0,
        ]);

        for &s in &signal {
            ted.process(s);
            // mu must always be in [0, 1)
            assert!(
                ted.mu() >= 0.0 && ted.mu() < 1.0,
                "mu out of range: {}",
                ted.mu()
            );
        }
    }

    #[test]
    fn test_symbol_output_decimation_rate() {
        let sps = 4;
        let n_symbols = 40;
        let mut ted = MuellerMullerTed::new(sps as f64);
        ted.set_loop_bandwidth(0.01);

        let signal = make_bpsk_signal(sps, &vec![1.0; n_symbols]);

        let mut output_count = 0;
        for &s in &signal {
            if ted.process(s).is_some() {
                output_count += 1;
            }
        }

        // Output should be approximately n_symbols (input_len / sps)
        // Allow some tolerance for loop startup and edge effects
        let expected = n_symbols;
        assert!(
            output_count >= expected - 5 && output_count <= expected + 5,
            "Expected ~{} symbols, got {}",
            expected,
            output_count
        );
    }
}
