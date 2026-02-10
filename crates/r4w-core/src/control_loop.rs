//! # Control Loop
//!
//! Generic 2nd-order digital control loop, the base building block
//! for PLL, Costas loop, FLL, and other feedback-based synchronizers.
//! Implements a proportional-integral (PI) controller with configurable
//! loop bandwidth and damping factor.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::control_loop::ControlLoop;
//!
//! let mut cl = ControlLoop::new(0.0628, 1.0, -1.0); // BW=0.0628 rad
//! cl.advance_loop(0.1); // Feed error signal
//! let freq = cl.get_frequency();
//! let phase = cl.get_phase();
//! ```

use std::f64::consts::PI;

/// Generic 2nd-order digital control loop.
///
/// Implements a PI filter to track phase/frequency:
///   freq  += beta * error
///   phase += freq + alpha * error
///
/// Where alpha and beta are derived from loop bandwidth and damping factor.
#[derive(Debug, Clone)]
pub struct ControlLoop {
    phase: f64,
    freq: f64,
    alpha: f64,
    beta: f64,
    max_freq: f64,
    min_freq: f64,
    damping: f64,
    bw: f64,
}

impl ControlLoop {
    /// Create a new control loop.
    ///
    /// * `bw` - Loop bandwidth in radians per sample (typ. 2π * BW/fs)
    /// * `max_freq` - Maximum frequency (radians per sample)
    /// * `min_freq` - Minimum frequency (radians per sample)
    pub fn new(bw: f64, max_freq: f64, min_freq: f64) -> Self {
        let damping = 1.0 / 2.0_f64.sqrt(); // critically damped
        let (alpha, beta) = compute_gains(bw, damping);
        Self {
            phase: 0.0,
            freq: 0.0,
            alpha,
            beta,
            max_freq,
            min_freq,
            damping,
            bw,
        }
    }

    /// Create with custom damping factor.
    pub fn with_damping(bw: f64, damping: f64, max_freq: f64, min_freq: f64) -> Self {
        let (alpha, beta) = compute_gains(bw, damping);
        Self {
            phase: 0.0,
            freq: 0.0,
            alpha,
            beta,
            max_freq,
            min_freq,
            damping,
            bw,
        }
    }

    /// Advance the loop with an error signal.
    ///
    /// Updates frequency and phase based on the error.
    pub fn advance_loop(&mut self, error: f64) {
        self.freq += self.beta * error;
        self.freq = self.freq.clamp(self.min_freq, self.max_freq);
        self.phase += self.freq + self.alpha * error;
    }

    /// Wrap phase to [-π, π].
    pub fn phase_wrap(&mut self) {
        self.phase = wrap_phase(self.phase);
    }

    /// Wrap frequency to [min_freq, max_freq].
    pub fn frequency_limit(&mut self) {
        self.freq = self.freq.clamp(self.min_freq, self.max_freq);
    }

    /// Set loop bandwidth (recomputes gains).
    pub fn set_loop_bandwidth(&mut self, bw: f64) {
        self.bw = bw;
        let (alpha, beta) = compute_gains(bw, self.damping);
        self.alpha = alpha;
        self.beta = beta;
    }

    /// Set damping factor (recomputes gains).
    pub fn set_damping_factor(&mut self, damping: f64) {
        self.damping = damping;
        let (alpha, beta) = compute_gains(self.bw, damping);
        self.alpha = alpha;
        self.beta = beta;
    }

    /// Get current phase estimate.
    pub fn get_phase(&self) -> f64 {
        self.phase
    }

    /// Get current frequency estimate.
    pub fn get_frequency(&self) -> f64 {
        self.freq
    }

    /// Set phase directly.
    pub fn set_phase(&mut self, phase: f64) {
        self.phase = phase;
    }

    /// Set frequency directly.
    pub fn set_frequency(&mut self, freq: f64) {
        self.freq = freq.clamp(self.min_freq, self.max_freq);
    }

    /// Get proportional gain (alpha).
    pub fn get_alpha(&self) -> f64 {
        self.alpha
    }

    /// Get integral gain (beta).
    pub fn get_beta(&self) -> f64 {
        self.beta
    }

    /// Get loop bandwidth.
    pub fn get_loop_bandwidth(&self) -> f64 {
        self.bw
    }

    /// Get damping factor.
    pub fn get_damping_factor(&self) -> f64 {
        self.damping
    }

    /// Reset loop state (phase and frequency to zero).
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.freq = 0.0;
    }
}

/// Compute PI controller gains from loop bandwidth and damping.
///
/// Uses the standard 2nd-order loop formula:
///   denom = 1 + 2*damping*bw + bw²
///   alpha = 4*damping*bw / denom
///   beta  = 4*bw² / denom
fn compute_gains(bw: f64, damping: f64) -> (f64, f64) {
    let denom = 1.0 + 2.0 * damping * bw + bw * bw;
    let alpha = 4.0 * damping * bw / denom;
    let beta = 4.0 * bw * bw / denom;
    (alpha, beta)
}

/// Wrap phase to [-π, π].
pub fn wrap_phase(phase: f64) -> f64 {
    let mut p = phase;
    while p > PI {
        p -= 2.0 * PI;
    }
    while p < -PI {
        p += 2.0 * PI;
    }
    p
}

/// Compute loop bandwidth from natural frequency and damping.
///
/// BW ≈ (ωn / fs) * (damping + 1/(4*damping))
pub fn bandwidth_from_natural_freq(omega_n: f64, damping: f64, sample_rate: f64) -> f64 {
    (omega_n / sample_rate) * (damping + 1.0 / (4.0 * damping))
}

/// Compute natural frequency from loop bandwidth and damping.
pub fn natural_freq_from_bandwidth(bw: f64, damping: f64, sample_rate: f64) -> f64 {
    bw * sample_rate / (damping + 1.0 / (4.0 * damping))
}

/// Simple phase detector: atan2(imag, real) of x * conj(ref).
pub fn phase_detector(signal: (f64, f64), reference: (f64, f64)) -> f64 {
    let re = signal.0 * reference.0 + signal.1 * reference.1;
    let im = signal.1 * reference.0 - signal.0 * reference.1;
    im.atan2(re)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_advance() {
        let mut cl = ControlLoop::new(0.1, 1.0, -1.0);
        assert_eq!(cl.get_phase(), 0.0);
        assert_eq!(cl.get_frequency(), 0.0);
        cl.advance_loop(1.0);
        assert!(cl.get_frequency() > 0.0);
        assert!(cl.get_phase() > 0.0);
    }

    #[test]
    fn test_frequency_clamping() {
        let mut cl = ControlLoop::new(0.5, 0.1, -0.1);
        // Push frequency positive hard.
        for _ in 0..100 {
            cl.advance_loop(10.0);
        }
        assert!(cl.get_frequency() <= 0.1);
        // Push negative.
        for _ in 0..100 {
            cl.advance_loop(-10.0);
        }
        assert!(cl.get_frequency() >= -0.1);
    }

    #[test]
    fn test_phase_wrap() {
        let mut cl = ControlLoop::new(0.1, 1.0, -1.0);
        cl.set_phase(5.0 * PI);
        cl.phase_wrap();
        assert!(cl.get_phase().abs() <= PI + 1e-10);
    }

    #[test]
    fn test_set_bandwidth() {
        let mut cl = ControlLoop::new(0.1, 1.0, -1.0);
        let alpha1 = cl.get_alpha();
        cl.set_loop_bandwidth(0.2);
        let alpha2 = cl.get_alpha();
        assert!(alpha2 > alpha1); // Higher BW → higher gain.
    }

    #[test]
    fn test_gains_computation() {
        let (alpha, beta) = compute_gains(0.1, 1.0 / 2.0_f64.sqrt());
        assert!(alpha > 0.0);
        assert!(beta > 0.0);
        assert!(alpha > beta); // Proportional > integral for low BW.
    }

    #[test]
    fn test_convergence() {
        // Loop should converge to track a constant frequency offset.
        let mut cl = ControlLoop::new(0.0628, 1.0, -1.0);
        let target_freq = 0.1;
        for i in 0..200 {
            let error = (target_freq * i as f64 - cl.get_phase()).sin();
            cl.advance_loop(error);
            cl.phase_wrap();
        }
        // After convergence, frequency should be near target.
        assert!((cl.get_frequency() - target_freq).abs() < 0.05);
    }

    #[test]
    fn test_wrap_phase_fn() {
        assert!((wrap_phase(0.0)).abs() < 1e-10);
        assert!((wrap_phase(2.0 * PI) - 0.0).abs() < 1e-10);
        assert!((wrap_phase(-2.0 * PI) - 0.0).abs() < 1e-10);
        assert!((wrap_phase(PI) - PI).abs() < 1e-10);
    }

    #[test]
    fn test_phase_detector() {
        // Same signal → zero error.
        let pd = phase_detector((1.0, 0.0), (1.0, 0.0));
        assert!(pd.abs() < 1e-10);
        // 90° offset.
        let pd = phase_detector((0.0, 1.0), (1.0, 0.0));
        assert!((pd - PI / 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_reset() {
        let mut cl = ControlLoop::new(0.1, 1.0, -1.0);
        cl.advance_loop(0.5);
        cl.advance_loop(0.5);
        assert!(cl.get_phase() != 0.0);
        cl.reset();
        assert_eq!(cl.get_phase(), 0.0);
        assert_eq!(cl.get_frequency(), 0.0);
    }

    #[test]
    fn test_bandwidth_conversions() {
        let damping = 1.0 / 2.0_f64.sqrt();
        let fs = 48000.0;
        let bw = 0.01;
        let omega_n = natural_freq_from_bandwidth(bw, damping, fs);
        let bw_back = bandwidth_from_natural_freq(omega_n, damping, fs);
        assert!((bw - bw_back).abs() < 1e-10);
    }
}
