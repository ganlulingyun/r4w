//! Proportional-Integral (PI) Biquad Phase-Locked Loop
//!
//! A second-order PLL for carrier and clock recovery using a biquad-structured
//! loop filter. The loop bandwidth and damping factor are independently tunable,
//! and the design uses `(f64, f64)` tuples for complex numbers (re, im) with no
//! external crate dependencies.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────┐    ┌───────────────┐    ┌─────────────┐    ┌─────┐
//! │  Input   │───>│ Phase Detector │───>│ Loop Filter │───>│ NCO │──┐
//! │ (re,im)  │    │  (conjugate    │    │ (PI biquad) │    │     │  │
//! └─────────┘    │   multiply)    │    └─────────────┘    └─────┘  │
//!                └───────────────┘                                  │
//!                        ^                                          │
//!                        └──────────────────────────────────────────┘
//! ```
//!
//! # Example
//!
//! ```rust
//! use r4w_core::phase_locked_loop_biquad::{PllBiquad, PllBiquadConfig};
//! use std::f64::consts::PI;
//!
//! let config = PllBiquadConfig {
//!     loop_bw: 50.0,
//!     damping: 0.707,
//!     sample_rate: 10000.0,
//! };
//! let mut pll = PllBiquad::new(config);
//!
//! // Generate a 100 Hz tone and track it
//! let freq = 100.0;
//! for i in 0..500 {
//!     let phase = 2.0 * PI * freq * i as f64 / 10000.0;
//!     let sample = (phase.cos(), phase.sin());
//!     let output = pll.step(sample);
//! }
//!
//! // After convergence the frequency estimate should be near 100 Hz
//! let est = pll.frequency_estimate();
//! assert!((est - freq).abs() < 10.0, "freq estimate {est} not near {freq}");
//! ```

use std::f64::consts::PI;

// ── helpers for (f64,f64) complex arithmetic ────────────────────────────────

/// Complex multiply: (a+jb)(c+jd)
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex conjugate
#[inline]
fn conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Magnitude squared
#[inline]
fn mag2(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Phase angle (atan2)
#[inline]
fn angle(a: (f64, f64)) -> f64 {
    a.1.atan2(a.0)
}

/// Unit phasor from angle
#[inline]
fn expj(theta: f64) -> (f64, f64) {
    (theta.cos(), theta.sin())
}

// ── configuration ───────────────────────────────────────────────────────────

/// Configuration for [`PllBiquad`].
#[derive(Debug, Clone)]
pub struct PllBiquadConfig {
    /// Single-sided noise bandwidth of the loop (Hz).
    /// Controls the trade-off between tracking speed and noise rejection.
    pub loop_bw: f64,
    /// Damping factor (ζ). 0.707 is critically damped and most common.
    pub damping: f64,
    /// Input sample rate (Hz).
    pub sample_rate: f64,
}

impl Default for PllBiquadConfig {
    fn default() -> Self {
        Self {
            loop_bw: 50.0,
            damping: 0.707,
            sample_rate: 48000.0,
        }
    }
}

// ── loop filter coefficients ────────────────────────────────────────────────

/// Second-order PI loop-filter coefficients derived from bandwidth and damping.
///
/// The natural frequency ω_n is derived from the loop bandwidth B_L:
///
///   ω_n = B_L / (ζ + 1/(4ζ))          (rad/s)
///
/// Proportional gain:  α = 2 ζ ω_n / ω_s
/// Integral gain:      β = (ω_n / ω_s)²
///
/// where ω_s = sample_rate.
#[derive(Debug, Clone, Copy)]
pub struct LoopCoefficients {
    /// Proportional (phase) gain.
    pub alpha: f64,
    /// Integral (frequency) gain.
    pub beta: f64,
}

impl LoopCoefficients {
    /// Compute coefficients from bandwidth, damping, and sample rate.
    ///
    /// # Panics
    /// Panics if `sample_rate` or `damping` is zero/negative.
    pub fn from_params(loop_bw: f64, damping: f64, sample_rate: f64) -> Self {
        assert!(sample_rate > 0.0, "sample_rate must be positive");
        assert!(damping > 0.0, "damping must be positive");

        // Noise-bandwidth formula for a second-order loop:
        //   B_L = ω_n (ζ + 1/(4ζ)) / 2   (one-sided)
        // Solve for ω_n:
        let denom = damping + 1.0 / (4.0 * damping);
        let omega_n = 2.0 * loop_bw / denom; // rad/s

        // Normalise to sample rate (discrete-time gains)
        let omega_n_norm = omega_n / sample_rate;

        let alpha = 2.0 * damping * omega_n_norm;
        let beta = omega_n_norm * omega_n_norm;

        Self { alpha, beta }
    }
}

// ── PLL core ────────────────────────────────────────────────────────────────

/// Proportional-Integral biquad Phase-Locked Loop.
///
/// Tracks the carrier phase and frequency of an input complex signal.
#[derive(Debug, Clone)]
pub struct PllBiquad {
    /// Loop-filter coefficients.
    coeffs: LoopCoefficients,
    /// Sample rate (Hz).
    sample_rate: f64,
    /// NCO phase accumulator (radians).
    nco_phase: f64,
    /// NCO frequency (radians per sample).
    nco_freq: f64,
    /// Integrator state for the PI filter.
    integrator: f64,
    /// Last phase error (for monitoring).
    last_phase_error: f64,
    /// Running sum of squared phase errors (for lock detection).
    error_sq_sum: f64,
    /// Number of samples processed.
    sample_count: u64,
    /// Exponential moving average of |phase_error|² for lock detection.
    error_ema: f64,
    /// EMA smoothing factor.
    ema_alpha: f64,
}

impl PllBiquad {
    /// Create a new PLL from the given configuration.
    pub fn new(config: PllBiquadConfig) -> Self {
        let coeffs =
            LoopCoefficients::from_params(config.loop_bw, config.damping, config.sample_rate);

        // EMA factor: time constant roughly scales with loop bandwidth
        let ema_alpha = (config.loop_bw / config.sample_rate).clamp(0.001, 0.5);

        Self {
            coeffs,
            sample_rate: config.sample_rate,
            nco_phase: 0.0,
            nco_freq: 0.0,
            integrator: 0.0,
            last_phase_error: 0.0,
            error_sq_sum: 0.0,
            sample_count: 0,
            error_ema: 1.0, // start pessimistic (unlocked)
            ema_alpha,
        }
    }

    /// Process one input sample. Returns the carrier-wiped-off (baseband) output.
    pub fn step(&mut self, input: (f64, f64)) -> (f64, f64) {
        // 1. Generate NCO output (local replica)
        let nco_out = expj(self.nco_phase);

        // 2. Carrier wipeoff: multiply input by conjugate of NCO
        let baseband = cmul(input, conj(nco_out));

        // 3. Phase detector: extract phase error
        let phase_error = angle(baseband);
        self.last_phase_error = phase_error;

        // 4. Update lock detector EMA
        let err_sq = phase_error * phase_error;
        self.error_ema = self.ema_alpha * err_sq + (1.0 - self.ema_alpha) * self.error_ema;
        self.error_sq_sum += err_sq;
        self.sample_count += 1;

        // 5. PI loop filter (biquad structure)
        //    proportional path:  alpha * error
        //    integral path:      integrator += beta * error
        self.integrator += self.coeffs.beta * phase_error;
        let loop_out = self.coeffs.alpha * phase_error + self.integrator;

        // 6. Update NCO frequency and phase
        self.nco_freq = loop_out;
        self.nco_phase += self.nco_freq;

        // Wrap phase to [-π, π)
        self.nco_phase = wrap_phase(self.nco_phase);

        baseband
    }

    /// Process a block of input samples, returning carrier-wiped-off outputs.
    pub fn process(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        input.iter().map(|&s| self.step(s)).collect()
    }

    /// Current frequency estimate in Hz.
    pub fn frequency_estimate(&self) -> f64 {
        // nco_freq is in radians/sample → Hz
        self.nco_freq * self.sample_rate / (2.0 * PI)
    }

    /// Current phase estimate in radians.
    pub fn phase_estimate(&self) -> f64 {
        self.nco_phase
    }

    /// Last phase error in radians.
    pub fn phase_error(&self) -> f64 {
        self.last_phase_error
    }

    /// Frequency error in Hz (same as frequency estimate when no initial offset).
    pub fn frequency_error(&self) -> f64 {
        self.frequency_estimate()
    }

    /// Returns `true` when the loop is locked.
    ///
    /// Lock is declared when the exponential moving average of the squared
    /// phase error drops below a threshold (default: 0.1 rad²  ≈  18°).
    pub fn is_locked(&self) -> bool {
        self.is_locked_with_threshold(0.1)
    }

    /// Lock detection with a custom threshold on the squared-error EMA.
    pub fn is_locked_with_threshold(&self, threshold: f64) -> bool {
        self.sample_count > 10 && self.error_ema < threshold
    }

    /// Exponential moving average of the squared phase error.
    pub fn phase_error_variance(&self) -> f64 {
        self.error_ema
    }

    /// Mean squared phase error over all samples processed so far.
    pub fn mean_squared_error(&self) -> f64 {
        if self.sample_count == 0 {
            return 0.0;
        }
        self.error_sq_sum / self.sample_count as f64
    }

    /// Theoretical pull-in range (Hz).
    ///
    /// For a second-order PLL the pull-in range is approximately:
    ///   Δf_pull ≈ √(2 B_L f_s / π)
    pub fn pull_in_range(&self) -> f64 {
        let bl = self.loop_bandwidth();
        (2.0 * bl * self.sample_rate / PI).sqrt()
    }

    /// Theoretical lock range (Hz).
    ///
    /// The lock range (hold-in range) of a second-order loop is approximately:
    ///   Δf_lock ≈ 2 B_L / (ζ + 1/(4ζ))
    pub fn lock_range(&self) -> f64 {
        let bl = self.loop_bandwidth();
        let damping = self.damping();
        let denom = damping + 1.0 / (4.0 * damping);
        2.0 * bl / denom
    }

    /// Carrier wipeoff: multiply `input` by the conjugate of the current NCO
    /// output *without* advancing the loop state.
    pub fn carrier_wipeoff(&self, input: (f64, f64)) -> (f64, f64) {
        cmul(input, conj(expj(self.nco_phase)))
    }

    /// Current NCO output as a complex phasor.
    pub fn nco_output(&self) -> (f64, f64) {
        expj(self.nco_phase)
    }

    /// Reset the PLL to its initial state.
    pub fn reset(&mut self) {
        self.nco_phase = 0.0;
        self.nco_freq = 0.0;
        self.integrator = 0.0;
        self.last_phase_error = 0.0;
        self.error_sq_sum = 0.0;
        self.sample_count = 0;
        self.error_ema = 1.0;
    }

    /// Set the NCO frequency directly (Hz). Useful for coarse acquisition.
    pub fn set_frequency(&mut self, freq_hz: f64) {
        self.nco_freq = 2.0 * PI * freq_hz / self.sample_rate;
    }

    /// Set the NCO phase directly (radians).
    pub fn set_phase(&mut self, phase_rad: f64) {
        self.nco_phase = wrap_phase(phase_rad);
    }

    /// Return the loop bandwidth (Hz) implied by current coefficients.
    pub fn loop_bandwidth(&self) -> f64 {
        // Reverse-engineer from alpha, beta:
        //   alpha = 2 ζ ω_n / fs
        //   beta  = (ω_n / fs)²
        // ω_n/fs = sqrt(beta)
        // ζ = alpha / (2 sqrt(beta))
        let omega_n_norm = self.coeffs.beta.sqrt();
        let zeta = if omega_n_norm > 0.0 {
            self.coeffs.alpha / (2.0 * omega_n_norm)
        } else {
            0.707
        };
        let omega_n = omega_n_norm * self.sample_rate;
        // B_L = ω_n (ζ + 1/(4ζ)) / 2
        omega_n * (zeta + 1.0 / (4.0 * zeta)) / 2.0
    }

    /// Return the effective damping factor.
    pub fn damping(&self) -> f64 {
        let omega_n_norm = self.coeffs.beta.sqrt();
        if omega_n_norm > 0.0 {
            self.coeffs.alpha / (2.0 * omega_n_norm)
        } else {
            0.707
        }
    }

    /// Return the current loop-filter coefficients.
    pub fn coefficients(&self) -> LoopCoefficients {
        self.coeffs
    }

    /// Update loop bandwidth and damping at run time (e.g., widen during
    /// acquisition, narrow after lock).
    pub fn set_loop_params(&mut self, loop_bw: f64, damping: f64) {
        self.coeffs = LoopCoefficients::from_params(loop_bw, damping, self.sample_rate);
        self.ema_alpha = (loop_bw / self.sample_rate).clamp(0.001, 0.5);
    }

    /// Number of samples processed.
    pub fn samples_processed(&self) -> u64 {
        self.sample_count
    }
}

/// Wrap an angle into (−π, π].
fn wrap_phase(mut phase: f64) -> f64 {
    while phase > PI {
        phase -= 2.0 * PI;
    }
    while phase <= -PI {
        phase += 2.0 * PI;
    }
    phase
}

// ── tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a complex tone at `freq` Hz, `n` samples, sample rate `fs`.
    fn tone(freq: f64, n: usize, fs: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let phase = 2.0 * PI * freq * i as f64 / fs;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    /// Helper: generate a complex tone with a constant phase offset.
    fn tone_with_phase(freq: f64, n: usize, fs: f64, phi: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let phase = 2.0 * PI * freq * i as f64 / fs + phi;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    fn default_config() -> PllBiquadConfig {
        PllBiquadConfig {
            loop_bw: 50.0,
            damping: 0.707,
            sample_rate: 10000.0,
        }
    }

    // ---- construction & defaults ----

    #[test]
    fn test_default_config() {
        let cfg = PllBiquadConfig::default();
        assert_eq!(cfg.loop_bw, 50.0);
        assert!((cfg.damping - 0.707).abs() < 1e-9);
        assert_eq!(cfg.sample_rate, 48000.0);
    }

    #[test]
    fn test_new_pll_initial_state() {
        let pll = PllBiquad::new(default_config());
        assert_eq!(pll.frequency_estimate(), 0.0);
        assert_eq!(pll.phase_estimate(), 0.0);
        assert_eq!(pll.samples_processed(), 0);
        assert!(!pll.is_locked());
    }

    // ---- coefficient computation ----

    #[test]
    fn test_coefficients_positive() {
        let c = LoopCoefficients::from_params(100.0, 0.707, 48000.0);
        assert!(c.alpha > 0.0, "alpha should be positive");
        assert!(c.beta > 0.0, "beta should be positive");
    }

    #[test]
    fn test_higher_bw_larger_coeffs() {
        let c1 = LoopCoefficients::from_params(50.0, 0.707, 48000.0);
        let c2 = LoopCoefficients::from_params(200.0, 0.707, 48000.0);
        assert!(c2.alpha > c1.alpha);
        assert!(c2.beta > c1.beta);
    }

    #[test]
    #[should_panic]
    fn test_zero_sample_rate_panics() {
        let _ = LoopCoefficients::from_params(50.0, 0.707, 0.0);
    }

    #[test]
    #[should_panic]
    fn test_zero_damping_panics() {
        let _ = LoopCoefficients::from_params(50.0, 0.0, 48000.0);
    }

    // ---- frequency tracking ----

    #[test]
    fn test_track_100hz_tone() {
        let fs = 10000.0;
        let freq = 100.0;
        let mut pll = PllBiquad::new(PllBiquadConfig {
            loop_bw: 50.0,
            damping: 0.707,
            sample_rate: fs,
        });

        let signal = tone(freq, 2000, fs);
        pll.process(&signal);

        let est = pll.frequency_estimate();
        assert!(
            (est - freq).abs() < 5.0,
            "Expected ~{freq} Hz, got {est} Hz"
        );
    }

    #[test]
    fn test_track_negative_frequency() {
        let fs = 10000.0;
        let freq = -200.0;
        let mut pll = PllBiquad::new(PllBiquadConfig {
            loop_bw: 80.0,
            damping: 0.707,
            sample_rate: fs,
        });

        let signal = tone(freq, 3000, fs);
        pll.process(&signal);

        let est = pll.frequency_estimate();
        assert!(
            (est - freq).abs() < 10.0,
            "Expected ~{freq} Hz, got {est} Hz"
        );
    }

    #[test]
    fn test_track_zero_frequency() {
        let fs = 10000.0;
        let mut pll = PllBiquad::new(PllBiquadConfig {
            loop_bw: 50.0,
            damping: 0.707,
            sample_rate: fs,
        });

        // DC signal (0 Hz carrier)
        let signal: Vec<(f64, f64)> = vec![(1.0, 0.0); 1000];
        pll.process(&signal);

        let est = pll.frequency_estimate();
        assert!(est.abs() < 2.0, "Expected ~0 Hz, got {est} Hz");
    }

    // ---- phase tracking ----

    #[test]
    fn test_phase_offset_tracking() {
        let fs = 10000.0;
        let phi = 1.0; // 1 radian offset
        let mut pll = PllBiquad::new(PllBiquadConfig {
            loop_bw: 50.0,
            damping: 0.707,
            sample_rate: fs,
        });

        // Feed a DC signal with a phase offset
        let signal = tone_with_phase(0.0, 2000, fs, phi);
        pll.process(&signal);

        // After lock, phase error should be small
        assert!(
            pll.phase_error().abs() < 0.1,
            "Phase error {} should be small after lock",
            pll.phase_error()
        );
    }

    // ---- lock detection ----

    #[test]
    fn test_lock_detection_locked() {
        let fs = 10000.0;
        let mut pll = PllBiquad::new(PllBiquadConfig {
            loop_bw: 80.0,
            damping: 0.707,
            sample_rate: fs,
        });

        let signal = tone(50.0, 5000, fs);
        pll.process(&signal);

        assert!(
            pll.is_locked(),
            "PLL should be locked after tracking a clean tone (error_ema={:.4})",
            pll.phase_error_variance()
        );
    }

    #[test]
    fn test_lock_detection_unlocked_initial() {
        let pll = PllBiquad::new(default_config());
        assert!(!pll.is_locked(), "PLL should not be locked initially");
    }

    #[test]
    fn test_lock_detection_custom_threshold() {
        let fs = 10000.0;
        let mut pll = PllBiquad::new(PllBiquadConfig {
            loop_bw: 80.0,
            damping: 0.707,
            sample_rate: fs,
        });
        let signal = tone(50.0, 5000, fs);
        pll.process(&signal);

        // Very tight threshold should still pass for a clean signal
        assert!(pll.is_locked_with_threshold(0.15));
    }

    // ---- carrier wipeoff ----

    #[test]
    fn test_carrier_wipeoff() {
        let fs = 10000.0;
        let freq = 200.0;
        let mut pll = PllBiquad::new(PllBiquadConfig {
            loop_bw: 80.0,
            damping: 0.707,
            sample_rate: fs,
        });

        let signal = tone(freq, 3000, fs);
        pll.process(&signal);

        // After lock, carrier wipeoff of a new sample should yield ~(1, 0)
        let test_phase = 2.0 * PI * freq * 3000.0 / fs;
        let test_sample = (test_phase.cos(), test_phase.sin());
        let wiped = pll.carrier_wipeoff(test_sample);
        let mag = mag2(wiped).sqrt();
        assert!(
            (mag - 1.0).abs() < 0.1,
            "Wiped carrier magnitude {mag} should be near 1.0"
        );
    }

    // ---- NCO output ----

    #[test]
    fn test_nco_output_unit_magnitude() {
        let mut pll = PllBiquad::new(default_config());
        let signal = tone(100.0, 500, 10000.0);
        pll.process(&signal);

        let nco = pll.nco_output();
        let mag = mag2(nco).sqrt();
        assert!(
            (mag - 1.0).abs() < 1e-10,
            "NCO output must have unit magnitude, got {mag}"
        );
    }

    // ---- pull-in and lock range ----

    #[test]
    fn test_pull_in_range_positive() {
        let pll = PllBiquad::new(default_config());
        let pr = pll.pull_in_range();
        assert!(pr > 0.0, "Pull-in range must be positive, got {pr}");
    }

    #[test]
    fn test_lock_range_positive() {
        let pll = PllBiquad::new(default_config());
        let lr = pll.lock_range();
        assert!(lr > 0.0, "Lock range must be positive, got {lr}");
    }

    #[test]
    fn test_pull_in_greater_than_lock_range() {
        let pll = PllBiquad::new(default_config());
        assert!(
            pll.pull_in_range() > pll.lock_range(),
            "Pull-in range ({}) should exceed lock range ({})",
            pll.pull_in_range(),
            pll.lock_range()
        );
    }

    // ---- reset ----

    #[test]
    fn test_reset() {
        let mut pll = PllBiquad::new(default_config());
        let signal = tone(100.0, 1000, 10000.0);
        pll.process(&signal);

        assert!(pll.samples_processed() > 0);
        pll.reset();

        assert_eq!(pll.frequency_estimate(), 0.0);
        assert_eq!(pll.phase_estimate(), 0.0);
        assert_eq!(pll.samples_processed(), 0);
        assert!(!pll.is_locked());
    }

    // ---- set_frequency / set_phase ----

    #[test]
    fn test_set_frequency() {
        let mut pll = PllBiquad::new(default_config());
        pll.set_frequency(500.0);

        let est = pll.frequency_estimate();
        assert!(
            (est - 500.0).abs() < 1e-6,
            "Expected 500 Hz, got {est} Hz"
        );
    }

    #[test]
    fn test_set_phase() {
        let mut pll = PllBiquad::new(default_config());
        pll.set_phase(1.5);
        assert!((pll.phase_estimate() - 1.5).abs() < 1e-10);

        // Wrapping check
        pll.set_phase(4.0);
        assert!(
            pll.phase_estimate().abs() <= PI + 1e-10,
            "Phase should be wrapped, got {}",
            pll.phase_estimate()
        );
    }

    // ---- set_loop_params (dynamic bandwidth) ----

    #[test]
    fn test_dynamic_bandwidth_change() {
        let fs = 10000.0;
        let mut pll = PllBiquad::new(PllBiquadConfig {
            loop_bw: 20.0,
            damping: 0.707,
            sample_rate: fs,
        });

        // Start with narrow BW
        let bw1 = pll.loop_bandwidth();

        // Widen for acquisition
        pll.set_loop_params(200.0, 0.707);
        let bw2 = pll.loop_bandwidth();
        assert!(
            bw2 > bw1 * 5.0,
            "BW should have increased: {bw1} -> {bw2}"
        );

        // Narrow back down
        pll.set_loop_params(20.0, 0.707);
        let bw3 = pll.loop_bandwidth();
        assert!(
            (bw3 - bw1).abs() < 1.0,
            "BW should return to original: {bw1} vs {bw3}"
        );
    }

    // ---- loop bandwidth / damping round-trip ----

    #[test]
    fn test_loop_bw_roundtrip() {
        let cfg = PllBiquadConfig {
            loop_bw: 120.0,
            damping: 1.0,
            sample_rate: 48000.0,
        };
        let pll = PllBiquad::new(cfg);
        let bw = pll.loop_bandwidth();
        assert!(
            (bw - 120.0).abs() < 0.5,
            "Loop BW round-trip failed: expected 120, got {bw}"
        );
    }

    #[test]
    fn test_damping_roundtrip() {
        let cfg = PllBiquadConfig {
            loop_bw: 100.0,
            damping: 1.5,
            sample_rate: 48000.0,
        };
        let pll = PllBiquad::new(cfg);
        let z = pll.damping();
        assert!(
            (z - 1.5).abs() < 1e-6,
            "Damping round-trip failed: expected 1.5, got {z}"
        );
    }

    // ---- helper functions ----

    #[test]
    fn test_wrap_phase() {
        assert!((wrap_phase(0.0)).abs() < 1e-15);
        assert!((wrap_phase(PI) - PI).abs() < 1e-15);
        assert!((wrap_phase(PI + 0.1) - (-PI + 0.1)).abs() < 1e-10);
        assert!((wrap_phase(-PI - 0.1) - (PI - 0.1)).abs() < 1e-10);
        assert!((wrap_phase(3.0 * PI) - PI).abs() < 1e-10);
    }

    #[test]
    fn test_complex_helpers() {
        // cmul: (1+j0) * (0+j1) = (0+j1)
        let r = cmul((1.0, 0.0), (0.0, 1.0));
        assert!((r.0).abs() < 1e-15 && (r.1 - 1.0).abs() < 1e-15);

        // conj
        let c = conj((3.0, 4.0));
        assert_eq!(c, (3.0, -4.0));

        // mag2
        assert!((mag2((3.0, 4.0)) - 25.0).abs() < 1e-10);

        // angle
        assert!((angle((1.0, 0.0))).abs() < 1e-15);
        assert!((angle((0.0, 1.0)) - PI / 2.0).abs() < 1e-15);

        // expj
        let e = expj(PI / 4.0);
        assert!((e.0 - (PI / 4.0).cos()).abs() < 1e-15);
        assert!((e.1 - (PI / 4.0).sin()).abs() < 1e-15);
    }

    // ---- mean squared error ----

    #[test]
    fn test_mean_squared_error() {
        let mut pll = PllBiquad::new(default_config());
        assert_eq!(pll.mean_squared_error(), 0.0);

        let signal = tone(100.0, 1000, 10000.0);
        pll.process(&signal);

        let mse = pll.mean_squared_error();
        assert!(mse >= 0.0, "MSE must be non-negative");
        assert!(mse < 3.0, "MSE should be reasonable for a clean tone, got {mse}");
    }

    // ---- process returns correct length ----

    #[test]
    fn test_process_output_length() {
        let mut pll = PllBiquad::new(default_config());
        let signal = tone(100.0, 256, 10000.0);
        let output = pll.process(&signal);
        assert_eq!(output.len(), 256);
        assert_eq!(pll.samples_processed(), 256);
    }
}
