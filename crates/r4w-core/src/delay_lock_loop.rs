//! # Delay Lock Loop (DLL) for Code Tracking
//!
//! Implements a Delay Lock Loop for code phase tracking in GNSS and
//! spread-spectrum receivers. The DLL uses early, prompt, and late correlator
//! outputs to estimate and track the code phase offset, maintaining
//! synchronization with the incoming spreading code.
//!
//! ## Features
//!
//! - Early-minus-late (E-L) discriminator with configurable correlator spacing
//! - Normalized early-minus-late discriminator `(E-L)/(E+L)`
//! - Dot-product discriminator for non-coherent tracking
//! - First-order and second-order loop filters
//! - Code phase tracking with fractional chip resolution
//! - Loop bandwidth and damping ratio configuration
//! - Lock detection via code phase variance monitoring
//! - NCO for code phase generation
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::delay_lock_loop::{DelayLockLoop, Discriminator};
//!
//! // Create a DLL for GPS L1 C/A code tracking
//! let mut dll = DelayLockLoop::new(1.023e6, 4.092e6)
//!     .with_bandwidth(2.0)
//!     .with_spacing(0.5)
//!     .with_order(2);
//!
//! // Simulate tracking with correlator outputs
//! // Early correlator is slightly stronger => code phase is late
//! let early = 0.8;
//! let prompt = 1.0;
//! let late = 0.6;
//!
//! let phase = dll.update(early, prompt, late);
//! assert!(dll.code_phase().is_finite());
//! assert!(dll.code_frequency().is_finite());
//! ```

use std::f64::consts::PI;

/// Discriminator type for the DLL.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Discriminator {
    /// Early minus late: `E - L`
    ///
    /// Simple and effective when prompt power is stable. Gain depends on
    /// signal amplitude.
    EarlyMinusLate,

    /// Normalized early minus late: `(E - L) / (E + L)`
    ///
    /// Amplitude-independent discriminator that provides consistent loop gain
    /// regardless of signal power variations. Most common for GNSS receivers.
    NormalizedEML,

    /// Dot-product discriminator: `(E - L) * P`
    ///
    /// Non-coherent discriminator useful when carrier phase is not tracked.
    /// The prompt correlation serves as a sign reference.
    DotProduct,
}

/// Lock detector for monitoring DLL tracking quality.
///
/// Tracks the variance of recent discriminator outputs. When the DLL is locked,
/// discriminator outputs cluster near zero with low variance. A rising variance
/// indicates loss of lock.
#[derive(Debug, Clone)]
pub struct LockDetector {
    /// Variance threshold below which the loop is considered locked.
    threshold: f64,
    /// Recent discriminator output history.
    history: Vec<f64>,
    /// Maximum number of history samples to retain.
    max_history: usize,
}

impl LockDetector {
    /// Create a new lock detector.
    ///
    /// # Arguments
    /// * `threshold` - Variance threshold for lock declaration
    /// * `max_history` - Number of recent samples to consider
    pub fn new(threshold: f64, max_history: usize) -> Self {
        Self {
            threshold,
            history: Vec::with_capacity(max_history),
            max_history,
        }
    }

    /// Record a new discriminator output sample.
    pub fn update(&mut self, error: f64) {
        if self.history.len() >= self.max_history {
            self.history.remove(0);
        }
        self.history.push(error);
    }

    /// Return `true` if the loop appears locked (low discriminator variance).
    pub fn is_locked(&self) -> bool {
        if self.history.len() < 2 {
            return false;
        }
        self.variance() < self.threshold
    }

    /// Compute the variance of the discriminator history.
    pub fn variance(&self) -> f64 {
        if self.history.len() < 2 {
            return f64::MAX;
        }
        let n = self.history.len() as f64;
        let mean = self.history.iter().sum::<f64>() / n;
        let var = self.history.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (n - 1.0);
        var
    }

    /// Reset the lock detector state.
    pub fn reset(&mut self) {
        self.history.clear();
    }
}

/// Delay Lock Loop for code phase tracking.
///
/// Tracks the code phase offset between a locally generated replica and the
/// received spreading code. The loop uses early, prompt, and late correlator
/// outputs spaced symmetrically around the prompt to generate an error signal
/// that drives a loop filter and NCO.
///
/// The loop filter order determines tracking dynamics:
/// - **Order 1**: Proportional-only control. Simple, tracks constant code phase
///   offsets with zero steady-state error, but cannot track a code frequency offset.
/// - **Order 2**: Proportional-integral control. Tracks constant code frequency
///   offsets (Doppler) with zero steady-state error.
#[derive(Debug, Clone)]
pub struct DelayLockLoop {
    /// Code chipping rate in chips per second.
    chip_rate: f64,
    /// Sample rate in samples per second.
    sample_rate: f64,
    /// Early-late correlator spacing in chips (typically 0.5 to 1.0).
    correlator_spacing: f64,
    /// Loop noise bandwidth in Hz.
    bandwidth_hz: f64,
    /// Damping ratio (typically 0.707 for critical damping).
    damping: f64,
    /// Loop filter order (1 or 2).
    order: usize,
    /// Current code phase estimate in chips.
    code_phase: f64,
    /// Current code frequency estimate in chips/s.
    code_freq: f64,
    /// Loop filter integrator state (used by 2nd-order filter).
    integrator: f64,
    /// Previous error value for the loop filter.
    prev_error: f64,
    /// Discriminator type.
    discriminator: Discriminator,
    /// Lock detector.
    lock_detector: LockDetector,
    /// Coherent integration time in seconds (set from sample_rate and integration length).
    integration_time: f64,
}

impl DelayLockLoop {
    /// Create a new DLL with the given chip rate and sample rate.
    ///
    /// Defaults: bandwidth 1 Hz, spacing 0.5 chips, 2nd-order filter,
    /// damping 0.707, normalized E-L discriminator.
    ///
    /// # Arguments
    /// * `chip_rate` - Code chipping rate in chips/second (e.g., 1.023e6 for GPS L1)
    /// * `sample_rate` - Receiver sample rate in samples/second
    pub fn new(chip_rate: f64, sample_rate: f64) -> Self {
        Self {
            chip_rate,
            sample_rate,
            correlator_spacing: 0.5,
            bandwidth_hz: 1.0,
            damping: 0.707,
            order: 2,
            code_phase: 0.0,
            code_freq: chip_rate,
            integrator: 0.0,
            prev_error: 0.0,
            discriminator: Discriminator::NormalizedEML,
            lock_detector: LockDetector::new(0.01, 20),
            integration_time: 0.001, // 1 ms default (GPS L1 C/A code period)
        }
    }

    /// Set the loop noise bandwidth in Hz (builder pattern).
    pub fn with_bandwidth(mut self, bw: f64) -> Self {
        self.bandwidth_hz = bw;
        self
    }

    /// Set the early-late correlator spacing in chips (builder pattern).
    pub fn with_spacing(mut self, spacing: f64) -> Self {
        self.correlator_spacing = spacing;
        self
    }

    /// Set the loop filter order (1 or 2) (builder pattern).
    ///
    /// # Panics
    /// Panics if `order` is not 1 or 2.
    pub fn with_order(mut self, order: usize) -> Self {
        assert!(order == 1 || order == 2, "DLL order must be 1 or 2");
        self.order = order;
        self
    }

    /// Set the damping ratio (builder pattern).
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = damping;
        self
    }

    /// Set the discriminator type (builder pattern).
    pub fn with_discriminator(mut self, disc: Discriminator) -> Self {
        self.discriminator = disc;
        self
    }

    /// Set the coherent integration time in seconds (builder pattern).
    pub fn with_integration_time(mut self, t: f64) -> Self {
        self.integration_time = t;
        self
    }

    /// Set the lock detector threshold (builder pattern).
    pub fn with_lock_threshold(mut self, threshold: f64) -> Self {
        self.lock_detector.threshold = threshold;
        self
    }

    /// Compute loop filter coefficients for the configured bandwidth and order.
    ///
    /// Returns `(c1, c2)` where:
    /// - `c1` is the proportional gain
    /// - `c2` is the integral gain (zero for first-order)
    fn loop_coefficients(&self) -> (f64, f64) {
        let bn_t = self.bandwidth_hz * self.integration_time;
        let zeta = self.damping;

        match self.order {
            1 => {
                // First-order loop: proportional only
                // c1 = 4 * Bn * T / (1 + 4 * Bn * T)  -- bilinear approximation
                // Simplified: for small Bn*T, c1 ~ 4 * Bn * T
                let c1 = 4.0 * bn_t;
                (c1, 0.0)
            }
            2 => {
                // Second-order loop: PI controller
                // Natural frequency: wn = Bn / (zeta + 1/(4*zeta))
                let wn = self.bandwidth_hz * 2.0 * PI
                    / (2.0 * zeta + 1.0 / (2.0 * zeta));
                let wn_t = wn * self.integration_time;

                let c1 = 2.0 * zeta * wn_t;
                let c2 = wn_t * wn_t;
                (c1, c2)
            }
            _ => unreachable!(),
        }
    }

    /// Compute the discriminator output from envelope (power) correlator values.
    ///
    /// This is the static discriminator computation that can be used independently
    /// of the loop.
    ///
    /// # Arguments
    /// * `early` - Early correlator magnitude
    /// * `prompt` - Prompt correlator magnitude
    /// * `late` - Late correlator magnitude
    /// * `disc` - Discriminator type
    ///
    /// # Returns
    /// Discriminator output in chips (scaled by correlator spacing).
    pub fn discriminator_output(early: f64, prompt: f64, late: f64, disc: Discriminator) -> f64 {
        match disc {
            Discriminator::EarlyMinusLate => {
                early - late
            }
            Discriminator::NormalizedEML => {
                let sum = early + late;
                if sum.abs() < 1e-15 {
                    0.0
                } else {
                    (early - late) / sum
                }
            }
            Discriminator::DotProduct => {
                (early - late) * prompt
            }
        }
    }

    /// Update the DLL with new correlator outputs (real/coherent mode).
    ///
    /// Takes the magnitudes of early, prompt, and late correlators and updates
    /// the code phase and frequency estimates.
    ///
    /// # Arguments
    /// * `early` - Early correlator output (magnitude or real part)
    /// * `prompt` - Prompt correlator output
    /// * `late` - Late correlator output
    ///
    /// # Returns
    /// Updated code phase in chips.
    pub fn update(&mut self, early: f64, prompt: f64, late: f64) -> f64 {
        // Compute discriminator error
        let error = Self::discriminator_output(early, prompt, late, self.discriminator);

        // Scale error by correlator spacing to normalize to chips.
        // For a triangular autocorrelation with spacing d, the E-L slope is 2/d,
        // so multiplying by d/2 converts the discriminator output to chip offset.
        let error_chips = error * self.correlator_spacing / 2.0;

        // Apply loop filter.
        // The loop filter architecture follows standard GNSS receiver design:
        //   - First-order: proportional term drives code phase directly
        //   - Second-order: proportional term drives code phase, integral drives code frequency
        let (c1, c2) = self.loop_coefficients();

        match self.order {
            1 => {
                // First-order: proportional correction applied directly to code phase
                self.code_phase += c1 * error_chips;
            }
            2 => {
                // Second-order: PI filter
                // Proportional arm adjusts code phase directly
                self.code_phase += c1 * error_chips;
                // Integral arm adjusts code frequency (tracks Doppler)
                self.integrator += c2 * error_chips;
                self.code_freq = self.chip_rate + self.integrator / self.integration_time;
            }
            _ => unreachable!(),
        }

        // For second-order loop, also advance phase by frequency offset
        if self.order == 2 {
            self.code_phase += (self.code_freq - self.chip_rate) * self.integration_time;
        }

        // Store for next iteration
        self.prev_error = error_chips;

        // Update lock detector
        self.lock_detector.update(error_chips);

        self.code_phase
    }

    /// Update the DLL with complex correlator outputs (non-coherent mode).
    ///
    /// Uses the magnitude of each complex correlator output for the
    /// discriminator, which makes it insensitive to carrier phase.
    ///
    /// # Arguments
    /// * `early` - Complex early correlator output `(I, Q)`
    /// * `prompt` - Complex prompt correlator output `(I, Q)`
    /// * `late` - Complex late correlator output `(I, Q)`
    ///
    /// # Returns
    /// Updated code phase in chips.
    pub fn update_complex(
        &mut self,
        early: (f64, f64),
        prompt: (f64, f64),
        late: (f64, f64),
    ) -> f64 {
        let e_mag = (early.0 * early.0 + early.1 * early.1).sqrt();
        let p_mag = (prompt.0 * prompt.0 + prompt.1 * prompt.1).sqrt();
        let l_mag = (late.0 * late.0 + late.1 * late.1).sqrt();
        self.update(e_mag, p_mag, l_mag)
    }

    /// Return the current code phase estimate in chips.
    pub fn code_phase(&self) -> f64 {
        self.code_phase
    }

    /// Return the current code frequency estimate in chips/second.
    pub fn code_frequency(&self) -> f64 {
        self.code_freq
    }

    /// Return `true` if the DLL is locked (low discriminator variance).
    pub fn is_locked(&self) -> bool {
        self.lock_detector.is_locked()
    }

    /// Return the current discriminator variance from the lock detector.
    pub fn lock_variance(&self) -> f64 {
        self.lock_detector.variance()
    }

    /// Reset the DLL to its initial state.
    pub fn reset(&mut self) {
        self.code_phase = 0.0;
        self.code_freq = self.chip_rate;
        self.integrator = 0.0;
        self.prev_error = 0.0;
        self.lock_detector.reset();
    }

    /// Return the early-late correlator spacing in chips.
    pub fn correlator_spacing(&self) -> f64 {
        self.correlator_spacing
    }

    /// Return the loop noise bandwidth in Hz.
    pub fn bandwidth(&self) -> f64 {
        self.bandwidth_hz
    }

    /// Return the loop filter order.
    pub fn order(&self) -> usize {
        self.order
    }

    /// Return the damping ratio.
    pub fn damping(&self) -> f64 {
        self.damping
    }

    /// Generate the current NCO code phase for a given sample index within
    /// the current integration period.
    ///
    /// # Arguments
    /// * `sample_index` - Sample index within the integration period
    ///
    /// # Returns
    /// Code phase in chips for the given sample.
    pub fn nco_phase(&self, sample_index: usize) -> f64 {
        let samples_per_chip = self.sample_rate / self.code_freq;
        self.code_phase + (sample_index as f64) / samples_per_chip
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_default_parameters() {
        let dll = DelayLockLoop::new(1.023e6, 4.092e6);
        assert_eq!(dll.chip_rate, 1.023e6);
        assert_eq!(dll.sample_rate, 4.092e6);
        assert_eq!(dll.correlator_spacing, 0.5);
        assert_eq!(dll.bandwidth_hz, 1.0);
        assert!((dll.damping - 0.707).abs() < 1e-10);
        assert_eq!(dll.order, 2);
        assert_eq!(dll.code_phase, 0.0);
        assert_eq!(dll.code_freq, 1.023e6);
    }

    #[test]
    fn test_builder_pattern() {
        let dll = DelayLockLoop::new(1.023e6, 4.092e6)
            .with_bandwidth(2.0)
            .with_spacing(1.0)
            .with_order(1)
            .with_damping(1.0)
            .with_discriminator(Discriminator::EarlyMinusLate)
            .with_integration_time(0.002)
            .with_lock_threshold(0.05);

        assert_eq!(dll.bandwidth_hz, 2.0);
        assert_eq!(dll.correlator_spacing, 1.0);
        assert_eq!(dll.order, 1);
        assert!((dll.damping - 1.0).abs() < 1e-10);
        assert_eq!(dll.discriminator, Discriminator::EarlyMinusLate);
        assert!((dll.integration_time - 0.002).abs() < 1e-10);
        assert!((dll.lock_detector.threshold - 0.05).abs() < 1e-10);
    }

    #[test]
    #[should_panic(expected = "DLL order must be 1 or 2")]
    fn test_invalid_order_panics() {
        let _ = DelayLockLoop::new(1.023e6, 4.092e6).with_order(3);
    }

    #[test]
    fn test_discriminator_early_minus_late() {
        // Early > Late => positive error (code phase is late)
        let err = DelayLockLoop::discriminator_output(
            0.8, 1.0, 0.6, Discriminator::EarlyMinusLate,
        );
        assert!((err - 0.2).abs() < 1e-10);

        // Early < Late => negative error (code phase is early)
        let err = DelayLockLoop::discriminator_output(
            0.6, 1.0, 0.8, Discriminator::EarlyMinusLate,
        );
        assert!((err - (-0.2)).abs() < 1e-10);

        // Equal => zero error (aligned)
        let err = DelayLockLoop::discriminator_output(
            0.7, 1.0, 0.7, Discriminator::EarlyMinusLate,
        );
        assert!(err.abs() < 1e-10);
    }

    #[test]
    fn test_discriminator_normalized_eml() {
        // Normalized: (E - L) / (E + L)
        let err = DelayLockLoop::discriminator_output(
            0.8, 1.0, 0.6, Discriminator::NormalizedEML,
        );
        let expected = (0.8 - 0.6) / (0.8 + 0.6);
        assert!((err - expected).abs() < 1e-10);

        // Zero correlators => zero output (no division by zero)
        let err = DelayLockLoop::discriminator_output(
            0.0, 0.0, 0.0, Discriminator::NormalizedEML,
        );
        assert!(err.abs() < 1e-10);
    }

    #[test]
    fn test_discriminator_dot_product() {
        // (E - L) * P
        let err = DelayLockLoop::discriminator_output(
            0.8, 1.0, 0.6, Discriminator::DotProduct,
        );
        let expected = (0.8 - 0.6) * 1.0;
        assert!((err - expected).abs() < 1e-10);

        // Negative prompt flips sign
        let err = DelayLockLoop::discriminator_output(
            0.8, -1.0, 0.6, Discriminator::DotProduct,
        );
        let expected = (0.8 - 0.6) * (-1.0);
        assert!((err - expected).abs() < 1e-10);
    }

    #[test]
    fn test_update_first_order() {
        let mut dll = DelayLockLoop::new(1.023e6, 4.092e6)
            .with_order(1)
            .with_bandwidth(1.0)
            .with_spacing(1.0);

        // With equal E and L, error is zero => no phase change
        dll.update(0.7, 1.0, 0.7);
        assert!(dll.code_phase.abs() < 1e-10);
        assert!((dll.code_frequency() - 1.023e6).abs() < 1.0);
    }

    #[test]
    fn test_update_second_order() {
        let mut dll = DelayLockLoop::new(1.023e6, 4.092e6)
            .with_order(2)
            .with_bandwidth(2.0)
            .with_spacing(0.5);

        // Positive error (early stronger) should increase code phase
        let phase_before = dll.code_phase();
        dll.update(0.9, 1.0, 0.5);
        let phase_after = dll.code_phase();
        assert!(
            phase_after > phase_before,
            "Code phase should increase for positive discriminator error"
        );
    }

    #[test]
    fn test_update_complex() {
        let mut dll = DelayLockLoop::new(1.023e6, 4.092e6)
            .with_order(2)
            .with_bandwidth(2.0);

        // Complex correlator outputs with carrier phase rotation
        let theta: f64 = 0.5; // arbitrary carrier phase
        let e_mag: f64 = 0.8;
        let p_mag: f64 = 1.0;
        let l_mag: f64 = 0.6;

        let early = (e_mag * theta.cos(), e_mag * theta.sin());
        let prompt = (p_mag * theta.cos(), p_mag * theta.sin());
        let late = (l_mag * theta.cos(), l_mag * theta.sin());

        let phase = dll.update_complex(early, prompt, late);
        assert!(phase.is_finite());

        // Should produce the same result as real update with magnitudes
        let mut dll2 = DelayLockLoop::new(1.023e6, 4.092e6)
            .with_order(2)
            .with_bandwidth(2.0);
        let phase2 = dll2.update(e_mag, p_mag, l_mag);

        assert!(
            (phase - phase2).abs() < 1e-10,
            "Complex update with magnitudes should match real update"
        );
    }

    #[test]
    fn test_lock_detection() {
        let mut dll = DelayLockLoop::new(1.023e6, 4.092e6)
            .with_bandwidth(1.0)
            .with_lock_threshold(0.01);

        // Not locked initially (not enough history)
        assert!(!dll.is_locked());

        // Feed consistent zero-error updates to achieve lock
        for _ in 0..30 {
            dll.update(0.7, 1.0, 0.7); // balanced => zero error
        }
        assert!(
            dll.is_locked(),
            "DLL should be locked after consistent zero-error updates, variance = {}",
            dll.lock_variance()
        );
    }

    #[test]
    fn test_reset() {
        let mut dll = DelayLockLoop::new(1.023e6, 4.092e6);

        // Perturb the state
        dll.update(0.9, 1.0, 0.5);
        dll.update(0.9, 1.0, 0.5);
        assert!(dll.code_phase.abs() > 0.0 || dll.integrator.abs() > 0.0);

        dll.reset();
        assert_eq!(dll.code_phase, 0.0);
        assert_eq!(dll.code_freq, 1.023e6);
        assert_eq!(dll.integrator, 0.0);
        assert_eq!(dll.prev_error, 0.0);
        assert!(!dll.is_locked());
    }

    #[test]
    fn test_nco_phase_generation() {
        let dll = DelayLockLoop::new(1.023e6, 4.092e6);

        // At 4x oversampling, 4 samples per chip
        let phase_0 = dll.nco_phase(0);
        let phase_1 = dll.nco_phase(1);
        let phase_4 = dll.nco_phase(4);

        // Phase at sample 0 should equal code_phase
        assert!((phase_0 - dll.code_phase()).abs() < 1e-10);

        // One sample advances by 1/4 chip (4.092 MHz / 1.023 MHz = 4 samples/chip)
        let expected_step = 1.0 / 4.0;
        assert!(
            (phase_1 - phase_0 - expected_step).abs() < 1e-6,
            "NCO step: expected {}, got {}",
            expected_step,
            phase_1 - phase_0
        );

        // 4 samples should advance exactly 1 chip
        assert!(
            (phase_4 - phase_0 - 1.0).abs() < 1e-6,
            "4 samples should advance 1 chip"
        );
    }

    #[test]
    fn test_tracking_convergence() {
        // Simulate a DLL tracking a constant code phase offset.
        // Use first-order loop with EarlyMinusLate discriminator for
        // straightforward gain characteristics in this closed-loop test.
        let mut dll = DelayLockLoop::new(1.023e6, 4.092e6)
            .with_bandwidth(100.0)
            .with_spacing(1.0)
            .with_order(1)
            .with_discriminator(Discriminator::EarlyMinusLate)
            .with_lock_threshold(0.001);

        // Simulate a triangular autocorrelation function with a 0.1 chip offset.
        // For a chip-spaced correlator (d=1.0), the correlation at offset tau
        // from the true peak is R(tau) = 1 - |tau| for |tau| <= 1.
        let true_offset = 0.1_f64;

        for _ in 0..1000 {
            // Current error between our estimate and truth
            let err = true_offset - dll.code_phase();

            // Triangular correlation model
            let d = dll.correlator_spacing();
            let early_offset = err - d / 2.0;
            let late_offset = err + d / 2.0;
            let prompt_offset = err;

            let early = (1.0 - early_offset.abs()).max(0.0);
            let prompt = (1.0 - prompt_offset.abs()).max(0.0);
            let late = (1.0 - late_offset.abs()).max(0.0);

            dll.update(early, prompt, late);
        }

        // Verify the DLL converged toward the true offset
        let final_error = (true_offset - dll.code_phase()).abs();
        assert!(
            final_error < 0.05,
            "DLL should converge toward true offset, got error = {}",
            final_error
        );

        // Verify the error decreased from the initial offset
        assert!(
            final_error < true_offset,
            "Final error ({}) should be less than initial offset ({})",
            final_error,
            true_offset
        );
    }

    #[test]
    fn test_lock_detector_standalone() {
        let mut ld = LockDetector::new(0.01, 10);

        // Not locked with empty history
        assert!(!ld.is_locked());

        // Feed low-variance data
        for i in 0..15 {
            ld.update(0.001 * (i as f64 % 3.0 - 1.0));
        }
        assert!(
            ld.is_locked(),
            "Should be locked with low-variance data, variance = {}",
            ld.variance()
        );

        // Reset and feed high-variance data
        ld.reset();
        for i in 0..15 {
            let val = if i % 2 == 0 { 1.0 } else { -1.0 };
            ld.update(val);
        }
        assert!(
            !ld.is_locked(),
            "Should not be locked with high-variance data, variance = {}",
            ld.variance()
        );
    }

    #[test]
    fn test_loop_coefficients() {
        // First-order coefficients
        let dll1 = DelayLockLoop::new(1.023e6, 4.092e6)
            .with_order(1)
            .with_bandwidth(1.0);
        let (c1, c2) = dll1.loop_coefficients();
        assert!(c1 > 0.0, "Proportional gain must be positive");
        assert_eq!(c2, 0.0, "First-order has no integral gain");

        // Second-order coefficients
        let dll2 = DelayLockLoop::new(1.023e6, 4.092e6)
            .with_order(2)
            .with_bandwidth(1.0);
        let (c1, c2) = dll2.loop_coefficients();
        assert!(c1 > 0.0, "Proportional gain must be positive");
        assert!(c2 > 0.0, "Second-order must have positive integral gain");

        // Higher bandwidth => larger gains
        let dll3 = DelayLockLoop::new(1.023e6, 4.092e6)
            .with_order(2)
            .with_bandwidth(10.0);
        let (c1_wide, c2_wide) = dll3.loop_coefficients();
        assert!(c1_wide > c1, "Wider bandwidth should have larger proportional gain");
        assert!(c2_wide > c2, "Wider bandwidth should have larger integral gain");
    }
}
