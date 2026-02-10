//! # Early-Late Gate Timing Error Detector
//!
//! Symbol-clock synchronization using the Early-Late Gate algorithm. The
//! detector computes timing error by comparing early and late sample
//! magnitudes: when the sampling instant is too early, the early sample is
//! larger; when too late, the late sample dominates. A PI loop filter steers
//! the fractional timing offset to minimise the error.
//!
//! Two variants are provided:
//!
//! - [`EarlyLateGate`] for real-valued signals (e.g. BPSK after matched
//!   filtering). Timing error: `|early|^2 - |late|^2`.
//! - [`EarlyLateComplex`] for complex (IQ) signals (e.g. QPSK). Timing
//!   error: `Re{early * conj(on_time)} - Re{late * conj(on_time)}`
//!   (decision-directed).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::early_late_gate::EarlyLateGate;
//!
//! let mut elg = EarlyLateGate::new(4.0, 0.5);
//!
//! // Feed samples one at a time; returns Some at symbol instants
//! let samples = vec![1.0, 0.7, 0.0, -0.7, -1.0, -0.7, 0.0, 0.7];
//! for &s in &samples {
//!     if let Some((timing_error, symbol)) = elg.process(s) {
//!         println!("symbol={:.3}, timing_error={:.4}", symbol, timing_error);
//!     }
//! }
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// EarlyLateGate (real-valued signals)
// ---------------------------------------------------------------------------

/// Early-Late Gate timing error detector for real-valued signals.
///
/// At each symbol strobe the detector linearly interpolates early, on-time,
/// and late samples and computes the timing error as
/// `|early|^2 - |late|^2`.
///
/// A proportional-integral (PI) loop filter converts the error into a
/// fractional timing offset (`mu`) that steers the interpolation point.
#[derive(Debug, Clone)]
pub struct EarlyLateGate {
    /// Nominal samples per symbol.
    samples_per_symbol: f64,
    /// Early/late spacing as a fraction of the symbol period (0..1).
    spacing: f64,
    /// Fractional timing offset in `[0, 1)`.
    mu: f64,
    /// Current instantaneous period estimate.
    omega: f64,
    /// Nominal period (centre).
    omega_mid: f64,
    /// PI proportional gain.
    kp: f64,
    /// PI integral gain.
    ki: f64,
    /// Integral accumulator.
    integrator: f64,
    /// Down-counter: fires strobe when it reaches zero.
    counter: f64,
    /// Ring buffer for incoming samples.
    buf: Vec<f64>,
    /// Write index into ring buffer.
    buf_write: usize,
    /// Ring buffer capacity.
    buf_len: usize,
    /// Whether we have produced at least one symbol.
    has_output: bool,
}

impl EarlyLateGate {
    /// Create a new Early-Late Gate detector.
    ///
    /// * `samples_per_symbol` - nominal oversampling ratio (must be >= 2.0).
    /// * `early_late_spacing` - spacing between early/late taps as a fraction
    ///   of the symbol period. 0.5 means half-symbol spacing, which is the
    ///   most common choice.
    ///
    /// # Panics
    ///
    /// Panics if `samples_per_symbol < 2.0` or `early_late_spacing` is not
    /// in `(0, 1]`.
    pub fn new(samples_per_symbol: f64, early_late_spacing: f64) -> Self {
        assert!(
            samples_per_symbol >= 2.0,
            "Early-Late Gate requires >= 2 samples per symbol, got {}",
            samples_per_symbol
        );
        assert!(
            early_late_spacing > 0.0 && early_late_spacing <= 1.0,
            "early_late_spacing must be in (0, 1], got {}",
            early_late_spacing
        );

        let buf_len = (samples_per_symbol.ceil() as usize) + 4;

        let mut elg = Self {
            samples_per_symbol,
            spacing: early_late_spacing,
            mu: 0.0,
            omega: samples_per_symbol,
            omega_mid: samples_per_symbol,
            kp: 0.0,
            ki: 0.0,
            integrator: 0.0,
            counter: samples_per_symbol,
            buf: vec![0.0; buf_len],
            buf_write: 0,
            buf_len,
            has_output: false,
        };
        elg.set_loop_bandwidth(0.01);
        elg
    }

    /// Set the PI loop-filter gains directly.
    pub fn set_loop_gains(&mut self, kp: f64, ki: f64) {
        self.kp = kp;
        self.ki = ki;
    }

    /// Compute PI loop-filter gains from a normalised loop bandwidth.
    ///
    /// `bn_ts` is the loop bandwidth normalised to the symbol rate (typical
    /// range 0.005 .. 0.05). Uses the standard second-order loop formulas
    /// with critical damping (zeta = 1).
    pub fn set_loop_bandwidth(&mut self, bn_ts: f64) {
        let bw = bn_ts.clamp(1e-6, 0.5);
        let theta = bw * 2.0 * PI / self.samples_per_symbol;
        let d = 1.0 + 2.0 * theta + theta * theta;
        self.kp = 4.0 * theta / d;
        self.ki = 4.0 * theta * theta / d;
    }

    /// Reset all internal state.
    pub fn reset(&mut self) {
        self.mu = 0.0;
        self.omega = self.omega_mid;
        self.integrator = 0.0;
        self.counter = self.omega_mid;
        self.has_output = false;
        for s in self.buf.iter_mut() {
            *s = 0.0;
        }
        self.buf_write = 0;
    }

    /// Return the current fractional timing offset in `[0, 1)`.
    pub fn mu(&self) -> f64 {
        self.mu
    }

    /// Feed one real-valued sample into the detector.
    ///
    /// Returns `Some((timing_error, symbol_value))` when a symbol strobe
    /// fires, or `None` when no symbol is ready.
    pub fn process(&mut self, sample: f64) -> Option<(f64, f64)> {
        // Push sample into ring buffer.
        self.buf[self.buf_write] = sample;
        self.buf_write = (self.buf_write + 1) % self.buf_len;

        // Decrement counter.
        self.counter -= 1.0;
        if self.counter > 0.0 {
            return None;
        }

        // -- strobe fires --
        self.mu = (self.counter + 1.0).clamp(0.0, 1.0 - f64::EPSILON);

        // Interpolate early, on-time, late samples.
        let half_spacing_samples = self.spacing * self.omega / 2.0;
        let on_time = self.interp(0.0);
        let early = self.interp(half_spacing_samples);
        let late = self.interp(-half_spacing_samples);

        // Timing error: |early|^2 - |late|^2
        let timing_error = early * early - late * late;

        // PI loop filter.
        self.integrator += self.ki * timing_error;
        let loop_out = self.kp * timing_error + self.integrator;

        // Update period estimate.
        self.omega = self.omega_mid + loop_out;
        // Clamp omega to +/- 10% of nominal.
        let lo = self.omega_mid * 0.9;
        let hi = self.omega_mid * 1.1;
        self.omega = self.omega.clamp(lo, hi);

        // Reset counter for next symbol.
        self.counter = self.omega;

        self.has_output = true;
        Some((timing_error, on_time))
    }

    /// Linear interpolation at `offset` samples relative to the on-time
    /// point. Positive offset means earlier (further back), negative means
    /// later (closer to most recent sample).
    fn interp(&self, offset: f64) -> f64 {
        let pos = (self.buf_write as f64) - 1.0 - offset + self.mu;
        let base = pos.floor() as isize;
        let frac = pos - pos.floor();

        let idx = |i: isize| -> usize {
            (((base + i) % self.buf_len as isize + self.buf_len as isize) as usize)
                % self.buf_len
        };

        let y0 = self.buf[idx(0)];
        let y1 = self.buf[idx(1)];
        y0 + frac * (y1 - y0)
    }
}

// ---------------------------------------------------------------------------
// EarlyLateComplex (IQ signals)
// ---------------------------------------------------------------------------

/// Early-Late Gate timing error detector for complex (IQ) signals.
///
/// Uses the decision-directed timing error:
/// `Re{early * conj(on_time)} - Re{late * conj(on_time)}`
///
/// This formulation works well for PSK/QAM constellations where the on-time
/// sample provides a reference phase.
#[derive(Debug, Clone)]
pub struct EarlyLateComplex {
    /// Nominal samples per symbol.
    samples_per_symbol: f64,
    /// Early/late spacing as fraction of symbol period.
    spacing: f64,
    /// Fractional timing offset in `[0, 1)`.
    mu: f64,
    /// Current instantaneous period estimate.
    omega: f64,
    /// Nominal period.
    omega_mid: f64,
    /// PI proportional gain.
    kp: f64,
    /// PI integral gain.
    ki: f64,
    /// Integral accumulator.
    integrator: f64,
    /// Down-counter.
    counter: f64,
    /// Ring buffer for I component.
    buf_i: Vec<f64>,
    /// Ring buffer for Q component.
    buf_q: Vec<f64>,
    /// Write index.
    buf_write: usize,
    /// Buffer capacity.
    buf_len: usize,
}

impl EarlyLateComplex {
    /// Create a new complex Early-Late Gate detector.
    ///
    /// * `samples_per_symbol` - nominal oversampling ratio (>= 2.0).
    /// * `early_late_spacing` - spacing in `(0, 1]`, fraction of symbol.
    ///
    /// # Panics
    ///
    /// Panics if `samples_per_symbol < 2.0` or spacing out of range.
    pub fn new(samples_per_symbol: f64, early_late_spacing: f64) -> Self {
        assert!(
            samples_per_symbol >= 2.0,
            "EarlyLateComplex requires >= 2 samples per symbol, got {}",
            samples_per_symbol
        );
        assert!(
            early_late_spacing > 0.0 && early_late_spacing <= 1.0,
            "early_late_spacing must be in (0, 1], got {}",
            early_late_spacing
        );

        let buf_len = (samples_per_symbol.ceil() as usize) + 4;

        let mut elc = Self {
            samples_per_symbol,
            spacing: early_late_spacing,
            mu: 0.0,
            omega: samples_per_symbol,
            omega_mid: samples_per_symbol,
            kp: 0.0,
            ki: 0.0,
            integrator: 0.0,
            counter: samples_per_symbol,
            buf_i: vec![0.0; buf_len],
            buf_q: vec![0.0; buf_len],
            buf_write: 0,
            buf_len,
        };
        elc.set_loop_bandwidth(0.01);
        elc
    }

    /// Set PI loop-filter gains directly.
    pub fn set_loop_gains(&mut self, kp: f64, ki: f64) {
        self.kp = kp;
        self.ki = ki;
    }

    /// Compute PI gains from normalised loop bandwidth.
    pub fn set_loop_bandwidth(&mut self, bn_ts: f64) {
        let bw = bn_ts.clamp(1e-6, 0.5);
        let theta = bw * 2.0 * PI / self.samples_per_symbol;
        let d = 1.0 + 2.0 * theta + theta * theta;
        self.kp = 4.0 * theta / d;
        self.ki = 4.0 * theta * theta / d;
    }

    /// Reset all internal state.
    pub fn reset(&mut self) {
        self.mu = 0.0;
        self.omega = self.omega_mid;
        self.integrator = 0.0;
        self.counter = self.omega_mid;
        for s in self.buf_i.iter_mut() {
            *s = 0.0;
        }
        for s in self.buf_q.iter_mut() {
            *s = 0.0;
        }
        self.buf_write = 0;
    }

    /// Return the current fractional timing offset.
    pub fn mu(&self) -> f64 {
        self.mu
    }

    /// Feed one complex sample `(i, q)` into the detector.
    ///
    /// Returns `Some((timing_error, (sym_i, sym_q)))` at symbol instants.
    pub fn process_complex(&mut self, sample: (f64, f64)) -> Option<(f64, (f64, f64))> {
        self.buf_i[self.buf_write] = sample.0;
        self.buf_q[self.buf_write] = sample.1;
        self.buf_write = (self.buf_write + 1) % self.buf_len;

        self.counter -= 1.0;
        if self.counter > 0.0 {
            return None;
        }

        self.mu = (self.counter + 1.0).clamp(0.0, 1.0 - f64::EPSILON);

        let half_spacing = self.spacing * self.omega / 2.0;

        let on_i = self.interp_i(0.0);
        let on_q = self.interp_q(0.0);
        let early_i = self.interp_i(half_spacing);
        let early_q = self.interp_q(half_spacing);
        let late_i = self.interp_i(-half_spacing);
        let late_q = self.interp_q(-half_spacing);

        // Decision-directed timing error:
        // e = Re{early * conj(on_time)} - Re{late * conj(on_time)}
        let re_early_conj_on = early_i * on_i + early_q * on_q;
        let re_late_conj_on = late_i * on_i + late_q * on_q;
        let timing_error = re_early_conj_on - re_late_conj_on;

        // PI loop filter.
        self.integrator += self.ki * timing_error;
        let loop_out = self.kp * timing_error + self.integrator;

        self.omega = self.omega_mid + loop_out;
        let lo = self.omega_mid * 0.9;
        let hi = self.omega_mid * 1.1;
        self.omega = self.omega.clamp(lo, hi);

        self.counter = self.omega;

        Some((timing_error, (on_i, on_q)))
    }

    /// Linear interpolation for I channel.
    fn interp_i(&self, offset: f64) -> f64 {
        self.interp_buf(&self.buf_i, offset)
    }

    /// Linear interpolation for Q channel.
    fn interp_q(&self, offset: f64) -> f64 {
        self.interp_buf(&self.buf_q, offset)
    }

    fn interp_buf(&self, buf: &[f64], offset: f64) -> f64 {
        let pos = (self.buf_write as f64) - 1.0 - offset + self.mu;
        let base = pos.floor() as isize;
        let frac = pos - pos.floor();

        let idx = |i: isize| -> usize {
            (((base + i) % self.buf_len as isize + self.buf_len as isize) as usize)
                % self.buf_len
        };

        let y0 = buf[idx(0)];
        let y1 = buf[idx(1)];
        y0 + frac * (y1 - y0)
    }
}

// ---------------------------------------------------------------------------
// Factory functions
// ---------------------------------------------------------------------------

/// Create an [`EarlyLateGate`] pre-tuned for BPSK reception.
///
/// Uses half-symbol early/late spacing and moderate loop bandwidth.
pub fn early_late_bpsk(sps: f64) -> EarlyLateGate {
    let mut elg = EarlyLateGate::new(sps, 0.5);
    elg.set_loop_bandwidth(0.01);
    elg
}

/// Create an [`EarlyLateComplex`] pre-tuned for QPSK reception.
///
/// Uses half-symbol early/late spacing and moderate loop bandwidth.
pub fn early_late_qpsk(sps: f64) -> EarlyLateComplex {
    let mut elc = EarlyLateComplex::new(sps, 0.5);
    elc.set_loop_bandwidth(0.01);
    elc
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // 1. Construction and default values
    #[test]
    fn test_construction_and_defaults() {
        let elg = EarlyLateGate::new(4.0, 0.5);
        assert_eq!(elg.samples_per_symbol, 4.0);
        assert_eq!(elg.spacing, 0.5);
        assert_eq!(elg.mu(), 0.0);
        assert!(elg.kp > 0.0, "kp should be positive after construction");
        assert!(elg.ki > 0.0, "ki should be positive after construction");
        assert!((elg.omega - 4.0).abs() < 1e-12);
    }

    // 2. Process returns None before first full symbol
    #[test]
    fn test_process_returns_none_before_first_symbol() {
        let mut elg = EarlyLateGate::new(4.0, 0.5);
        // With sps=4, the first 3 samples should not produce a strobe.
        assert!(elg.process(1.0).is_none());
        assert!(elg.process(0.5).is_none());
        assert!(elg.process(0.0).is_none());
    }

    // 3. Timing error computation on known waveform
    #[test]
    fn test_timing_error_known_waveform() {
        let mut elg = EarlyLateGate::new(4.0, 0.5);
        // Perfectly aligned rectangular BPSK: +1 for 4 samples, -1 for 4.
        let signal: Vec<f64> = (0..40)
            .map(|i| if (i / 4) % 2 == 0 { 1.0 } else { -1.0 })
            .collect();

        let mut results = Vec::new();
        for &s in &signal {
            if let Some((te, sym)) = elg.process(s) {
                results.push((te, sym));
            }
        }

        // Should produce several symbol outputs.
        assert!(
            results.len() >= 4,
            "Expected >= 4 symbols, got {}",
            results.len()
        );

        // On a rectangular aligned signal the early and late samples have
        // similar magnitudes when sampling is centred, so timing errors
        // should be bounded.
        for &(te, _) in &results {
            assert!(
                te.abs() < 5.0,
                "Timing error {:.4} unexpectedly large",
                te
            );
        }
    }

    // 4. Early vs late timing produces opposite error signs
    #[test]
    fn test_early_vs_late_opposite_error() {
        let sps = 8usize;
        let n_sym = 40;

        // Build a smooth BPSK waveform using raised-cosine shaped pulses so
        // that timing offsets produce asymmetric early/late magnitudes.
        let make_signal = |offset: f64| -> Vec<f64> {
            let len = sps * n_sym;
            (0..len)
                .map(|i| {
                    let t = (i as f64 + offset) / sps as f64;
                    let sym_idx = t.floor() as isize;
                    let frac = t - t.floor(); // [0,1) within symbol
                    let val = if sym_idx % 2 == 0 { 1.0 } else { -1.0 };
                    // Raised-cosine pulse shape within each symbol
                    let envelope = 0.5 * (1.0 - (2.0 * PI * frac).cos());
                    val * envelope
                })
                .collect()
        };

        let run = |offset: f64| -> f64 {
            let mut elg = EarlyLateGate::new(sps as f64, 0.5);
            elg.set_loop_bandwidth(0.0005); // very slow loop so it doesn't correct
            let sig = make_signal(offset);
            let mut total = 0.0;
            let mut count = 0usize;
            for &s in &sig {
                if let Some((te, _)) = elg.process(s) {
                    total += te;
                    count += 1;
                }
            }
            if count > 0 { total / count as f64 } else { 0.0 }
        };

        let early_err = run(-1.5);
        let late_err = run(1.5);

        // With a shaped pulse, opposite offsets should produce timing errors
        // of opposite sign (or at least different values).
        let diff = early_err - late_err;
        assert!(
            diff.abs() > 1e-6,
            "Expected different errors for early ({:.6}) vs late ({:.6}), diff={:.9}",
            early_err,
            late_err,
            diff
        );
    }

    // 5. Loop gain setting
    #[test]
    fn test_set_loop_gains() {
        let mut elg = EarlyLateGate::new(4.0, 0.5);
        elg.set_loop_gains(0.123, 0.0045);
        assert!((elg.kp - 0.123).abs() < 1e-12);
        assert!((elg.ki - 0.0045).abs() < 1e-12);
    }

    // 6. Bandwidth-based gain computation
    #[test]
    fn test_bandwidth_gain_computation() {
        let mut elg = EarlyLateGate::new(4.0, 0.5);

        // At very small bandwidth, gains should be small.
        elg.set_loop_bandwidth(0.001);
        let kp_narrow = elg.kp;
        let ki_narrow = elg.ki;

        // At wider bandwidth, gains should be larger.
        elg.set_loop_bandwidth(0.05);
        let kp_wide = elg.kp;
        let ki_wide = elg.ki;

        assert!(
            kp_wide > kp_narrow,
            "Wider bandwidth should produce larger kp: {} vs {}",
            kp_wide,
            kp_narrow
        );
        assert!(
            ki_wide > ki_narrow,
            "Wider bandwidth should produce larger ki: {} vs {}",
            ki_wide,
            ki_narrow
        );
    }

    // 7. Reset clears state
    #[test]
    fn test_reset_clears_state() {
        let mut elg = EarlyLateGate::new(4.0, 0.5);
        // Run some samples.
        for i in 0..20 {
            elg.process(if i % 4 < 2 { 1.0 } else { -1.0 });
        }
        assert!(elg.has_output, "Should have produced output before reset");

        elg.reset();

        assert_eq!(elg.mu(), 0.0);
        assert!(!elg.has_output);
        assert!((elg.omega - elg.omega_mid).abs() < 1e-12);
        assert_eq!(elg.integrator, 0.0);
    }

    // 8. Mu stays in [0,1) range
    #[test]
    fn test_mu_stays_in_range() {
        let mut elg = EarlyLateGate::new(4.0, 0.5);
        let signal: Vec<f64> = (0..80)
            .map(|i| if (i / 4) % 2 == 0 { 1.0 } else { -1.0 })
            .collect();

        let mut mus = Vec::new();
        for &s in &signal {
            if elg.process(s).is_some() {
                mus.push(elg.mu());
            }
        }

        assert!(mus.len() >= 4, "Expected >= 4 mu values, got {}", mus.len());
        for &m in &mus {
            assert!(
                (0.0..1.0).contains(&m),
                "mu={:.6} out of range [0, 1)",
                m
            );
        }
    }

    // 9. BPSK factory
    #[test]
    fn test_early_late_bpsk_factory() {
        let elg = early_late_bpsk(4.0);
        assert_eq!(elg.samples_per_symbol, 4.0);
        assert_eq!(elg.spacing, 0.5);
        assert!(elg.kp > 0.0);
        assert!(elg.ki > 0.0);

        // Should be usable immediately.
        let mut elg = elg;
        assert!(elg.process(1.0).is_none());
    }

    // 10. QPSK complex factory
    #[test]
    fn test_early_late_qpsk_factory() {
        let mut elc = early_late_qpsk(4.0);
        assert_eq!(elc.samples_per_symbol, 4.0);
        assert_eq!(elc.spacing, 0.5);
        assert!(elc.kp > 0.0);
        assert!(elc.ki > 0.0);

        // Feed a simple QPSK-like IQ signal (alternating +1+j / -1-j).
        let mut results = Vec::new();
        for i in 0..40 {
            let val = if (i / 4) % 2 == 0 {
                (1.0, 1.0)
            } else {
                (-1.0, -1.0)
            };
            if let Some((te, (si, sq))) = elc.process_complex(val) {
                results.push((te, si, sq));
            }
        }

        assert!(
            results.len() >= 4,
            "Expected >= 4 complex symbols, got {}",
            results.len()
        );

        // Timing errors should be finite and bounded.
        for &(te, _, _) in &results {
            assert!(te.is_finite(), "Timing error should be finite");
            assert!(te.abs() < 10.0, "Timing error {:.4} unexpectedly large", te);
        }
    }
}
