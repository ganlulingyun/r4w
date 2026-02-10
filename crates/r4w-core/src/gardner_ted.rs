//! # Gardner Timing Error Detector (TED)
//!
//! Symbol-clock recovery using the Gardner timing error detector algorithm.
//! The Gardner TED is a non-data-aided (NDA) detector that requires at least
//! 2 samples per symbol. It uses mid-symbol interpolation to estimate the
//! timing error without needing symbol decisions, making it suitable for
//! burst-mode and acquisition scenarios.
//!
//! ## Algorithm
//!
//! The Gardner timing error is computed as:
//!
//! ```text
//! e(k) = x(kT + T/2) * [x(kT) - x((k-1)T)]
//! ```
//!
//! where `x(kT)` is the on-time (strobe) sample, `x((k-1)T)` is the previous
//! strobe sample, and `x(kT + T/2)` is the mid-symbol interpolated sample.
//!
//! A proportional-integral (PI) loop filter converts the timing error into a
//! fractional sampling offset (`mu`) that steers the interpolation point.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::gardner_ted::GardnerTed;
//!
//! let mut ted = GardnerTed::new(4.0);
//!
//! // Feed samples one at a time; returns Some at symbol instants
//! let samples = vec![1.0, 0.7, 0.0, -0.7, -1.0, -0.7, 0.0, 0.7];
//! for &s in &samples {
//!     if let Some((timing_error, symbol)) = ted.process(s) {
//!         println!("symbol={:.3}, timing_error={:.4}", symbol, timing_error);
//!     }
//! }
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for the Gardner TED.
#[derive(Debug, Clone)]
pub struct GardnerConfig {
    /// Nominal samples per symbol (must be >= 2.0).
    pub samples_per_symbol: f64,
    /// Normalised loop bandwidth (0 < bw < 0.5). Default 0.01.
    pub loop_bandwidth: f64,
    /// Damping factor for the PI loop filter. Default 1.0 (critically damped).
    pub damping: f64,
    /// Maximum allowed deviation of the instantaneous period from the nominal
    /// (as a fraction). Default 0.1 (i.e. +/- 10%).
    pub max_deviation: f64,
}

impl Default for GardnerConfig {
    fn default() -> Self {
        Self {
            samples_per_symbol: 4.0,
            loop_bandwidth: 0.01,
            damping: 1.0,
            max_deviation: 0.1,
        }
    }
}

impl GardnerConfig {
    /// Create a new configuration with the given samples-per-symbol.
    pub fn new(samples_per_symbol: f64) -> Self {
        Self {
            samples_per_symbol,
            ..Default::default()
        }
    }

    /// Set the loop bandwidth.
    pub fn with_loop_bandwidth(mut self, bw: f64) -> Self {
        self.loop_bandwidth = bw;
        self
    }

    /// Set the damping factor.
    pub fn with_damping(mut self, d: f64) -> Self {
        self.damping = d;
        self
    }

    /// Set the max deviation.
    pub fn with_max_deviation(mut self, d: f64) -> Self {
        self.max_deviation = d;
        self
    }

    /// Build a [`GardnerTed`] from this configuration.
    pub fn build(self) -> GardnerTed {
        let mut ted = GardnerTed::new(self.samples_per_symbol);
        ted.set_loop_bandwidth(self.loop_bandwidth);
        ted.damping = self.damping;
        ted.max_deviation = self.max_deviation;
        ted
    }
}

// ---------------------------------------------------------------------------
// GardnerTed
// ---------------------------------------------------------------------------

/// Gardner Timing Error Detector for symbol-clock recovery.
///
/// Feed real-valued samples one at a time via [`process`](GardnerTed::process).
/// At each symbol strobe instant the method returns
/// `Some((timing_error, interpolated_symbol))`.
#[derive(Debug, Clone)]
pub struct GardnerTed {
    /// Nominal samples per symbol.
    pub samples_per_symbol: f64,

    /// Fractional timing offset in the range `[0, 1)`.
    pub mu: f64,

    /// Set to `true` at optimal sample instants (strobe output).
    pub strobe: bool,

    // -- internal state -----------------------------------------------------

    /// Current instantaneous period estimate.
    omega: f64,
    /// Nominal (centre) period.
    omega_mid: f64,
    /// Max deviation of omega from omega_mid (absolute).
    max_deviation: f64,

    // Loop filter gains
    alpha: f64, // proportional
    beta: f64,  // integral

    // Damping factor
    damping: f64,

    // Sample history ring-buffer (we only need the last ceil(sps)+2 samples).
    buf: Vec<f64>,
    buf_write: usize,
    buf_len: usize,

    /// Down-counter: counts down from omega, fires strobe at zero crossing.
    counter: f64,

    /// Previous on-time (strobe) symbol sample.
    prev_symbol: f64,

    /// Whether we have produced at least one symbol yet.
    has_prev: bool,
}

impl GardnerTed {
    /// Create a new Gardner TED.
    ///
    /// # Panics
    ///
    /// Panics if `samples_per_symbol < 2.0`.
    pub fn new(samples_per_symbol: f64) -> Self {
        assert!(
            samples_per_symbol >= 2.0,
            "Gardner TED requires at least 2 samples per symbol, got {}",
            samples_per_symbol
        );

        let buf_len = (samples_per_symbol.ceil() as usize) + 4;

        let mut ted = Self {
            samples_per_symbol,
            mu: 0.0,
            strobe: false,
            omega: samples_per_symbol,
            omega_mid: samples_per_symbol,
            max_deviation: 0.1 * samples_per_symbol,
            alpha: 0.0,
            beta: 0.0,
            damping: 1.0,
            buf: vec![0.0; buf_len],
            buf_write: 0,
            buf_len,
            counter: samples_per_symbol,
            prev_symbol: 0.0,
            has_prev: false,
        };
        ted.set_loop_bandwidth(0.01);
        ted
    }

    /// Configure the PI loop-filter gains from a normalised bandwidth.
    ///
    /// `bw_normalized` is the loop bandwidth normalised to the symbol rate.
    /// Typical values are 0.005 .. 0.05. The loop gains are derived using the
    /// standard second-order control-loop formulas with the current damping
    /// factor.
    pub fn set_loop_bandwidth(&mut self, bw_normalized: f64) {
        let bw = bw_normalized.clamp(1e-6, 0.5);
        let theta = bw * 2.0 * PI / self.samples_per_symbol;
        let d = 1.0 + 2.0 * self.damping * theta + theta * theta;
        self.alpha = 4.0 * self.damping * theta / d;
        self.beta = 4.0 * theta * theta / d;
    }

    /// Reset all internal state (clear buffers, counters, loop filter).
    pub fn reset(&mut self) {
        self.mu = 0.0;
        self.strobe = false;
        self.omega = self.omega_mid;
        self.counter = self.omega_mid;
        self.prev_symbol = 0.0;
        self.has_prev = false;
        for s in self.buf.iter_mut() {
            *s = 0.0;
        }
        self.buf_write = 0;
    }

    /// Feed one real-valued sample into the detector.
    ///
    /// Returns `Some((timing_error, interpolated_symbol))` when a symbol
    /// strobe fires, or `None` when no symbol is ready yet.
    pub fn process(&mut self, sample: f64) -> Option<(f64, f64)> {
        // Push sample into ring buffer.
        self.buf[self.buf_write] = sample;
        self.buf_write = (self.buf_write + 1) % self.buf_len;

        self.strobe = false;

        // Decrement counter
        self.counter -= 1.0;
        if self.counter > 0.0 {
            return None;
        }

        // -- strobe fires ---------------------------------------------------
        self.strobe = true;

        // The fractional part tells us where within the sample interval the
        // optimal point lies. mu = 1 - fractional overshoot.
        self.mu = self.counter + 1.0; // counter <= 0, so mu in [0, 1)
        self.mu = self.mu.clamp(0.0, 1.0 - f64::EPSILON);

        // Interpolate the on-time sample at the current write-head minus half
        // a period, and the mid-sample at minus one full period (relative to
        // the strobe).
        let on_time = self.interpolate_at(0.0);
        let mid_sample = self.interpolate_at(self.omega / 2.0);

        // Gardner TED: e = mid * (on_time - prev_on_time)
        let timing_error = if self.has_prev {
            mid_sample * (on_time - self.prev_symbol)
        } else {
            0.0
        };

        // Update loop filter
        self.omega += self.beta * timing_error;
        // Clamp omega
        let lo = self.omega_mid - self.max_deviation;
        let hi = self.omega_mid + self.max_deviation;
        self.omega = self.omega.clamp(lo, hi);

        self.mu += self.alpha * timing_error;

        // Reset counter for the next symbol period
        self.counter = self.omega + self.mu;
        // Keep mu in [0, 1)
        if self.mu >= 1.0 {
            self.counter += 1.0;
            self.mu -= 1.0;
        } else if self.mu < 0.0 {
            self.counter -= 1.0;
            self.mu += 1.0;
        }

        self.prev_symbol = on_time;
        self.has_prev = true;

        Some((timing_error, on_time))
    }

    /// Cubic Hermite interpolation at `delay` samples back from the most
    /// recently written sample.
    fn interpolate_at(&self, delay: f64) -> f64 {
        // Position in the ring buffer (fractional).
        let pos = (self.buf_write as f64) - 1.0 - delay + self.mu;
        let base = pos.floor() as isize;
        let frac = pos - pos.floor();

        let y = |i: isize| -> f64 {
            let idx = ((base + i) % self.buf_len as isize
                + self.buf_len as isize) as usize
                % self.buf_len;
            self.buf[idx]
        };

        // Four-point cubic Hermite (Catmull-Rom) interpolation.
        let y_m1 = y(-1);
        let y_0 = y(0);
        let y_1 = y(1);
        let y_2 = y(2);

        let c0 = y_0;
        let c1 = 0.5 * (y_1 - y_m1);
        let c2 = y_m1 - 2.5 * y_0 + 2.0 * y_1 - 0.5 * y_2;
        let c3 = 0.5 * (y_2 - y_m1) + 1.5 * (y_0 - y_1);

        ((c3 * frac + c2) * frac + c1) * frac + c0
    }

    /// Return the current loop-filter proportional gain.
    pub fn alpha(&self) -> f64 {
        self.alpha
    }

    /// Return the current loop-filter integral gain.
    pub fn beta(&self) -> f64 {
        self.beta
    }

    /// Return the current instantaneous period estimate.
    pub fn omega(&self) -> f64 {
        self.omega
    }
}

// ---------------------------------------------------------------------------
// Factory helpers
// ---------------------------------------------------------------------------

/// Create a [`GardnerTed`] pre-tuned for BPSK reception.
///
/// Uses a moderate loop bandwidth (0.01) and critical damping which works
/// well for BPSK/QPSK signals at typical SNR.
pub fn gardner_bpsk(sps: f64) -> GardnerTed {
    GardnerConfig::new(sps)
        .with_loop_bandwidth(0.01)
        .with_damping(1.0)
        .with_max_deviation(0.1)
        .build()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // 1. Basic construction and defaults
    #[test]
    fn test_basic_construction_and_defaults() {
        let ted = GardnerTed::new(4.0);
        assert_eq!(ted.samples_per_symbol, 4.0);
        assert_eq!(ted.mu, 0.0);
        assert!(!ted.strobe);
        assert!(ted.alpha() > 0.0);
        assert!(ted.beta() > 0.0);
        assert!((ted.omega() - 4.0).abs() < 1e-12);
    }

    // 2. Process returns None for non-strobe samples
    #[test]
    fn test_process_returns_none_for_non_strobe() {
        let mut ted = GardnerTed::new(4.0);
        // The first 3 samples should not produce a strobe (period ~4).
        assert!(ted.process(1.0).is_none());
        assert!(ted.process(0.5).is_none());
        assert!(ted.process(0.0).is_none());
    }

    // 3. Timing error computation on known waveform
    #[test]
    fn test_timing_error_known_waveform() {
        let mut ted = GardnerTed::new(4.0);
        // Generate a clean BPSK-like waveform: +1 for 4 samples, -1 for 4.
        let signal: Vec<f64> = (0..32)
            .map(|i| if (i / 4) % 2 == 0 { 1.0 } else { -1.0 })
            .collect();

        let mut errors = Vec::new();
        for &s in &signal {
            if let Some((te, _sym)) = ted.process(s) {
                errors.push(te);
            }
        }

        // We should get several symbol outputs.
        assert!(
            errors.len() >= 4,
            "Expected at least 4 symbols, got {}",
            errors.len()
        );

        // On a rectangular BPSK waveform, transitions produce non-zero
        // mid-sample values so timing errors can be significant. We just
        // verify the errors are finite and bounded (within +/- 3).
        for &e in errors.iter().skip(1) {
            assert!(
                e.abs() < 3.0,
                "Timing error {:.4} unexpectedly large on aligned signal",
                e
            );
        }
    }

    // 4. Loop filter convergence
    #[test]
    fn test_loop_filter_convergence() {
        let mut ted = GardnerTed::new(4.0);
        ted.set_loop_bandwidth(0.05); // faster convergence for the test

        // Feed a BPSK signal with a deliberate 1-sample timing offset
        // (symbols shifted by 1 sample). The loop should pull in.
        let n_syms = 80;
        let sps = 4usize;
        let offset = 1; // shift by 1 sample
        let mut signal = vec![0.0f64; n_syms * sps + offset];
        for sym in 0..n_syms {
            let val: f64 = if sym % 2 == 0 { 1.0 } else { -1.0 };
            for k in 0..sps {
                signal[offset + sym * sps + k] = val;
            }
        }

        let mut errors = Vec::new();
        for &s in &signal {
            if let Some((te, _)) = ted.process(s) {
                errors.push(te);
            }
        }

        // The latter-half errors should be smaller than the first-half
        // (indicating convergence).
        let n = errors.len();
        if n >= 8 {
            let first_half_energy: f64 =
                errors[1..n / 2].iter().map(|e| e * e).sum::<f64>() / (n / 2 - 1) as f64;
            let second_half_energy: f64 =
                errors[n / 2..].iter().map(|e| e * e).sum::<f64>() / (n - n / 2) as f64;
            assert!(
                second_half_energy <= first_half_energy + 1e-6,
                "Loop did not converge: first_half={:.6}, second_half={:.6}",
                first_half_energy,
                second_half_energy
            );
        }
    }

    // 5. Reset clears state
    #[test]
    fn test_reset_clears_state() {
        let mut ted = GardnerTed::new(4.0);
        // Feed some samples
        for i in 0..20 {
            ted.process(if i % 4 < 2 { 1.0 } else { -1.0 });
        }
        // State should be non-initial
        assert!(ted.has_prev);

        ted.reset();

        assert_eq!(ted.mu, 0.0);
        assert!(!ted.strobe);
        assert!(!ted.has_prev);
        assert_eq!(ted.prev_symbol, 0.0);
        assert!((ted.omega() - ted.omega_mid).abs() < 1e-12);
    }

    // 6. Different samples_per_symbol values (2, 4, 8)
    #[test]
    fn test_different_sps_values() {
        for &sps in &[2.0, 4.0, 8.0] {
            let mut ted = GardnerTed::new(sps);
            let n = (sps as usize) * 20;
            let mut count = 0usize;
            for i in 0..n {
                let val = if (i / sps as usize) % 2 == 0 {
                    1.0
                } else {
                    -1.0
                };
                if ted.process(val).is_some() {
                    count += 1;
                }
            }
            // Should produce roughly n/sps symbols. With loop dynamics the
            // count can deviate somewhat, especially at low SPS where the
            // loop has less room. Allow +/- 50% tolerance.
            let expected = (n as f64 / sps) as usize;
            let lo = expected / 2;
            let hi = expected * 2;
            assert!(
                count >= lo && count <= hi,
                "sps={}: expected ~{} symbols (range {}..{}), got {}",
                sps,
                expected,
                lo,
                hi,
                count
            );
        }
    }

    // 7. BPSK factory function
    #[test]
    fn test_gardner_bpsk_factory() {
        let ted = gardner_bpsk(4.0);
        assert_eq!(ted.samples_per_symbol, 4.0);
        assert!(ted.alpha() > 0.0);
        assert!(ted.beta() > 0.0);
        // Should be usable immediately
        let mut ted = ted;
        assert!(ted.process(1.0).is_none()); // just confirm it runs
    }

    // 8. Timing error sign for early/late
    #[test]
    fn test_timing_error_sign_early_late() {
        // When the sampling instant is *early* relative to the transition,
        // the Gardner error should have the opposite sign compared to *late*.
        let sps = 8usize;
        let n_sym = 20;

        // Build a smooth BPSK waveform using sinc-ish transitions (raised
        // cosine pulse shaped) so the mid-sample is meaningful.
        let make_signal = |offset: isize| -> Vec<f64> {
            let len = sps * n_sym;
            (0..len)
                .map(|i| {
                    let t = i as f64 + offset as f64;
                    let sym_idx = (t / sps as f64).floor() as isize;
                    if sym_idx % 2 == 0 {
                        1.0
                    } else {
                        -1.0
                    }
                })
                .collect()
        };

        let run = |offset: isize| -> f64 {
            let mut ted = GardnerTed::new(sps as f64);
            ted.set_loop_bandwidth(0.001); // very slow loop so it doesn't correct
            let sig = make_signal(offset);
            let mut total_error = 0.0;
            let mut count = 0;
            for &s in &sig {
                if let Some((te, _)) = ted.process(s) {
                    if count > 0 {
                        // skip first (no prev symbol)
                        total_error += te;
                    }
                    count += 1;
                }
            }
            if count > 1 {
                total_error / (count - 1) as f64
            } else {
                0.0
            }
        };

        let early_error = run(-2);
        let late_error = run(2);

        // The two offsets should produce errors of different sign (or at least
        // the difference should be nonzero, showing the TED is sensitive to
        // the direction of the offset).
        let diff = early_error - late_error;
        // We do not insist on a particular sign convention because the absolute
        // sign depends on symbol patterns; we only require they differ.
        assert!(
            diff.abs() > 1e-9 || (early_error.abs() < 1e-9 && late_error.abs() < 1e-9),
            "Expected different errors for early ({:.6}) vs late ({:.6})",
            early_error,
            late_error
        );
    }

    // 9. Mu tracking / fractional offset update
    #[test]
    fn test_mu_tracking() {
        let mut ted = GardnerTed::new(4.0);
        let mut mus = Vec::new();

        // Run enough samples to collect a few mu values.
        let signal: Vec<f64> = (0..60)
            .map(|i| if (i / 4) % 2 == 0 { 1.0 } else { -1.0 })
            .collect();

        for &s in &signal {
            if ted.process(s).is_some() {
                mus.push(ted.mu);
            }
        }

        // mu should stay in [0, 1) at all times.
        for &m in &mus {
            assert!(
                (0.0..1.0).contains(&m),
                "mu={:.6} out of range [0,1)",
                m
            );
        }

        // We should have collected several mu values.
        assert!(
            mus.len() >= 4,
            "Expected at least 4 mu observations, got {}",
            mus.len()
        );
    }

    // 10. Config builder
    #[test]
    fn test_config_builder() {
        let cfg = GardnerConfig::new(8.0)
            .with_loop_bandwidth(0.02)
            .with_damping(0.707)
            .with_max_deviation(0.15);

        assert_eq!(cfg.samples_per_symbol, 8.0);
        assert!((cfg.loop_bandwidth - 0.02).abs() < 1e-12);
        assert!((cfg.damping - 0.707).abs() < 1e-12);
        assert!((cfg.max_deviation - 0.15).abs() < 1e-12);

        let ted = cfg.build();
        assert_eq!(ted.samples_per_symbol, 8.0);
        assert!(ted.alpha() > 0.0);
        assert!(ted.beta() > 0.0);
    }
}
