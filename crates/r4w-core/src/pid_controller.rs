//! PID (Proportional-Integral-Derivative) controller for SDR servo loops.
//!
//! Provides a general-purpose PID controller suitable for AGC loops, frequency
//! tracking (NCO / AFC), timing recovery, and thermal management in SDR systems.
//!
//! # Features
//!
//! - Proportional, integral, and derivative terms with independent gains
//! - Output clamping with configurable limits
//! - Anti-windup: integral clamping when the output is saturated
//! - Derivative low-pass filter (EMA) to suppress noise amplification
//! - Builder-pattern configuration via [`PidConfig`]
//! - Factory constructors for common SDR use cases
//!
//! # Example
//!
//! ```rust
//! use r4w_core::pid_controller::{PidController, PidConfig};
//!
//! // Build a PID controller with output limits
//! let mut pid = PidConfig::new(1.0, 0.1, 0.01)
//!     .with_output_limits(-10.0, 10.0)
//!     .with_integral_limits(-5.0, 5.0)
//!     .with_derivative_filter(0.3)
//!     .build();
//!
//! // Run a few update steps
//! let dt = 0.001; // 1 ms sample period
//! let output1 = pid.update(1.0, dt);
//! let output2 = pid.update(0.5, dt);
//!
//! assert!(output1 > 0.0);
//! assert!(pid.proportional() >= 0.0);
//! ```

// ---------------------------------------------------------------------------
// PidConfig — builder
// ---------------------------------------------------------------------------

/// Builder for [`PidController`] with optional limits and derivative filtering.
#[derive(Debug, Clone)]
pub struct PidConfig {
    kp: f64,
    ki: f64,
    kd: f64,
    output_min: f64,
    output_max: f64,
    integral_min: f64,
    integral_max: f64,
    derivative_alpha: f64,
}

impl PidConfig {
    /// Create a new configuration with the given PID gains.
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            output_min: f64::NEG_INFINITY,
            output_max: f64::INFINITY,
            integral_min: f64::NEG_INFINITY,
            integral_max: f64::INFINITY,
            derivative_alpha: 0.0,
        }
    }

    /// Set symmetric output clamp limits.
    pub fn with_output_limits(mut self, min: f64, max: f64) -> Self {
        self.output_min = min;
        self.output_max = max;
        self
    }

    /// Set symmetric integral (anti-windup) clamp limits.
    pub fn with_integral_limits(mut self, min: f64, max: f64) -> Self {
        self.integral_min = min;
        self.integral_max = max;
        self
    }

    /// Set the derivative EMA filter coefficient.
    ///
    /// `alpha = 0.0` means no filtering (raw derivative).
    /// `alpha = 0.9` means heavy filtering (90% of previous value retained).
    pub fn with_derivative_filter(mut self, alpha: f64) -> Self {
        self.derivative_alpha = alpha.clamp(0.0, 1.0);
        self
    }

    /// Consume the config and produce a [`PidController`].
    pub fn build(self) -> PidController {
        let mut pid = PidController::new(self.kp, self.ki, self.kd);
        pid.set_output_limits(self.output_min, self.output_max);
        pid.set_integral_limits(self.integral_min, self.integral_max);
        pid.set_derivative_filter(self.derivative_alpha);
        pid
    }
}

// ---------------------------------------------------------------------------
// PidController
// ---------------------------------------------------------------------------

/// A PID controller with anti-windup, output clamping, and derivative filtering.
#[derive(Debug, Clone)]
pub struct PidController {
    // Gains
    kp: f64,
    ki: f64,
    kd: f64,

    // State
    integral: f64,
    prev_error: f64,
    prev_derivative: f64,
    last_output: f64,

    // Individual terms (stored for introspection)
    p_term: f64,
    i_term: f64,
    d_term: f64,

    // Limits
    output_min: f64,
    output_max: f64,
    integral_min: f64,
    integral_max: f64,

    // Derivative EMA filter coefficient (0 = off, 0.9 = heavy)
    derivative_alpha: f64,

    // Tracks whether this is the very first update (no valid prev_error yet)
    first_update: bool,
}

impl PidController {
    // -- Construction -------------------------------------------------------

    /// Create a new PID controller with the given gains.
    ///
    /// Output and integral limits default to +/-infinity (unconstrained).
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: 0.0,
            prev_derivative: 0.0,
            last_output: 0.0,
            p_term: 0.0,
            i_term: 0.0,
            d_term: 0.0,
            output_min: f64::NEG_INFINITY,
            output_max: f64::INFINITY,
            integral_min: f64::NEG_INFINITY,
            integral_max: f64::INFINITY,
            derivative_alpha: 0.0,
            first_update: true,
        }
    }

    // -- Gain setters -------------------------------------------------------

    /// Replace all three PID gains at once.
    pub fn set_gains(&mut self, kp: f64, ki: f64, kd: f64) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    // -- Limit setters ------------------------------------------------------

    /// Set the output clamp range.  The control output will be clamped to
    /// `[min, max]` after each [`update`](Self::update).
    pub fn set_output_limits(&mut self, min: f64, max: f64) {
        self.output_min = min;
        self.output_max = max;
    }

    /// Set the integral accumulator clamp range (anti-windup).
    pub fn set_integral_limits(&mut self, min: f64, max: f64) {
        self.integral_min = min;
        self.integral_max = max;
    }

    // -- Derivative filter --------------------------------------------------

    /// Set the derivative EMA filter coefficient.
    ///
    /// * `alpha = 0.0` — no filtering (raw derivative)
    /// * `alpha = 0.9` — heavy filtering (smoothed derivative)
    pub fn set_derivative_filter(&mut self, alpha: f64) {
        self.derivative_alpha = alpha.clamp(0.0, 1.0);
    }

    // -- Core update --------------------------------------------------------

    /// Compute one PID update step.
    ///
    /// * `error` — the current error signal (setpoint - measurement)
    /// * `dt`    — time elapsed since the previous update (seconds)
    ///
    /// Returns the clamped control output.
    pub fn update(&mut self, error: f64, dt: f64) -> f64 {
        // Proportional
        self.p_term = self.kp * error;

        // Integral with anti-windup clamping
        self.integral += error * dt;
        self.integral = self.integral.clamp(self.integral_min, self.integral_max);
        self.i_term = self.ki * self.integral;

        // Derivative (with optional EMA filter)
        let raw_derivative = if self.first_update {
            0.0
        } else if dt > 0.0 {
            (error - self.prev_error) / dt
        } else {
            0.0
        };

        let filtered_derivative = if self.derivative_alpha > 0.0 {
            self.derivative_alpha * self.prev_derivative
                + (1.0 - self.derivative_alpha) * raw_derivative
        } else {
            raw_derivative
        };

        self.d_term = self.kd * filtered_derivative;
        self.prev_derivative = filtered_derivative;
        self.prev_error = error;
        self.first_update = false;

        // Sum and clamp
        let output = self.p_term + self.i_term + self.d_term;
        let clamped = output.clamp(self.output_min, self.output_max);

        // Additional anti-windup: if output is saturated, prevent further
        // integral accumulation in the saturating direction.
        if (clamped >= self.output_max && error > 0.0)
            || (clamped <= self.output_min && error < 0.0)
        {
            // Undo the integral contribution from this step
            self.integral -= error * dt;
            self.integral = self.integral.clamp(self.integral_min, self.integral_max);
            self.i_term = self.ki * self.integral;
        }

        self.last_output = clamped;
        clamped
    }

    // -- Reset --------------------------------------------------------------

    /// Clear all internal state (integral, derivative memory, output).
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.prev_derivative = 0.0;
        self.last_output = 0.0;
        self.p_term = 0.0;
        self.i_term = 0.0;
        self.d_term = 0.0;
        self.first_update = true;
    }

    // -- Introspection ------------------------------------------------------

    /// The most recent control output returned by [`update`](Self::update).
    pub fn output(&self) -> f64 {
        self.last_output
    }

    /// The proportional term from the most recent update.
    pub fn proportional(&self) -> f64 {
        self.p_term
    }

    /// The integral term from the most recent update.
    pub fn integral(&self) -> f64 {
        self.i_term
    }

    /// The derivative term from the most recent update.
    pub fn derivative(&self) -> f64 {
        self.d_term
    }
}

// ---------------------------------------------------------------------------
// Factory constructors
// ---------------------------------------------------------------------------

/// Create a PID controller tuned for Automatic Gain Control.
///
/// Characteristics: fast proportional response, slow integral to remove
/// steady-state offset, no derivative (AGC feedback is already noisy).
pub fn agc_pid() -> PidController {
    PidConfig::new(0.8, 0.05, 0.0)
        .with_output_limits(-1.0, 1.0)
        .with_integral_limits(-0.5, 0.5)
        .build()
}

/// Create a PID controller tuned for NCO / AFC frequency tracking.
///
/// Characteristics: moderate proportional, strong integral for zero
/// steady-state frequency error, light derivative for damping.
pub fn frequency_tracking_pid() -> PidController {
    PidConfig::new(0.5, 0.3, 0.05)
        .with_output_limits(-1000.0, 1000.0)
        .with_integral_limits(-500.0, 500.0)
        .with_derivative_filter(0.2)
        .build()
}

/// Create a PID controller tuned for slow thermal / temperature control.
///
/// Characteristics: low gains across the board, heavy derivative filtering.
pub fn temperature_pid() -> PidController {
    PidConfig::new(0.1, 0.02, 0.005)
        .with_output_limits(-1.0, 1.0)
        .with_integral_limits(-0.5, 0.5)
        .with_derivative_filter(0.8)
        .build()
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.001; // 1 ms

    // 1. Construction and initial state
    #[test]
    fn test_construction_and_initial_state() {
        let pid = PidController::new(1.0, 0.1, 0.01);
        assert_eq!(pid.output(), 0.0);
        assert_eq!(pid.proportional(), 0.0);
        assert_eq!(pid.integral(), 0.0);
        assert_eq!(pid.derivative(), 0.0);
    }

    // 2. Proportional-only response
    #[test]
    fn test_proportional_only() {
        let mut pid = PidController::new(2.0, 0.0, 0.0);
        let out = pid.update(5.0, DT);
        // P = kp * error = 2.0 * 5.0 = 10.0
        assert!((out - 10.0).abs() < 1e-12);
        assert!((pid.proportional() - 10.0).abs() < 1e-12);
        assert!((pid.integral()).abs() < 1e-12);
        assert!((pid.derivative()).abs() < 1e-12);
    }

    // 3. Integral accumulation over time
    #[test]
    fn test_integral_accumulation() {
        let mut pid = PidController::new(0.0, 10.0, 0.0);
        // Apply constant error = 1.0 for 100 steps at dt = 0.001
        for _ in 0..100 {
            pid.update(1.0, DT);
        }
        // Integral should be approximately 100 * 1.0 * 0.001 = 0.1
        // I term = ki * integral = 10.0 * 0.1 = 1.0
        let i_term = pid.integral();
        assert!(
            (i_term - 1.0).abs() < 1e-9,
            "Expected integral term ~1.0, got {}",
            i_term
        );
    }

    // 4. Derivative response to step change
    #[test]
    fn test_derivative_step_response() {
        let mut pid = PidController::new(0.0, 0.0, 1.0);
        // First update: derivative is zero (no prior sample)
        let out1 = pid.update(0.0, DT);
        assert!((out1).abs() < 1e-12, "First output should be ~0");

        // Step to error = 1.0
        let out2 = pid.update(1.0, DT);
        // derivative = (1.0 - 0.0) / 0.001 = 1000.0
        // D term = kd * derivative = 1.0 * 1000.0 = 1000.0
        assert!(
            (out2 - 1000.0).abs() < 1e-6,
            "Expected derivative output ~1000.0, got {}",
            out2
        );
    }

    // 5. Output clamping
    #[test]
    fn test_output_clamping() {
        let mut pid = PidController::new(10.0, 0.0, 0.0);
        pid.set_output_limits(-5.0, 5.0);

        let out = pid.update(100.0, DT);
        assert!(
            (out - 5.0).abs() < 1e-12,
            "Output should be clamped to 5.0, got {}",
            out
        );

        let out_neg = pid.update(-100.0, DT);
        assert!(
            (out_neg - (-5.0)).abs() < 1e-12,
            "Output should be clamped to -5.0, got {}",
            out_neg
        );
    }

    // 6. Anti-windup prevents integral blow-up
    #[test]
    fn test_anti_windup() {
        let mut pid = PidController::new(0.0, 100.0, 0.0);
        pid.set_output_limits(-1.0, 1.0);
        pid.set_integral_limits(-0.01, 0.01);

        // Drive with large positive error for many iterations
        for _ in 0..10_000 {
            pid.update(100.0, DT);
        }

        // The integral term should be bounded by ki * integral_max = 100 * 0.01 = 1.0
        let i_term = pid.integral();
        assert!(
            i_term <= 1.0 + 1e-9,
            "Integral term should be bounded, got {}",
            i_term
        );

        // Now reverse the error: the controller should respond quickly
        // because the integral hasn't blown up.
        let out = pid.update(-1.0, DT);
        assert!(
            out < 1.0,
            "After error reversal output should drop, got {}",
            out
        );
    }

    // 7. Derivative filter reduces noise
    #[test]
    fn test_derivative_filter() {
        // Unfiltered controller
        let mut pid_raw = PidController::new(0.0, 0.0, 1.0);
        pid_raw.set_derivative_filter(0.0);

        // Filtered controller
        let mut pid_filt = PidController::new(0.0, 0.0, 1.0);
        pid_filt.set_derivative_filter(0.9);

        // Warm up both with zero error
        pid_raw.update(0.0, DT);
        pid_filt.update(0.0, DT);

        // Apply a noisy step: alternate between +1 and -1
        let mut raw_sum = 0.0;
        let mut filt_sum = 0.0;
        for i in 0..100 {
            let error = if i % 2 == 0 { 1.0 } else { -1.0 };
            raw_sum += pid_raw.update(error, DT).abs();
            filt_sum += pid_filt.update(error, DT).abs();
        }

        // The filtered version should accumulate less total absolute output
        assert!(
            filt_sum < raw_sum,
            "Filtered sum ({}) should be less than raw sum ({})",
            filt_sum,
            raw_sum
        );
    }

    // 8. Reset clears all state
    #[test]
    fn test_reset() {
        let mut pid = PidController::new(1.0, 1.0, 1.0);
        // Run some updates to build up state
        pid.update(5.0, DT);
        pid.update(3.0, DT);
        pid.update(1.0, DT);

        assert!(pid.output() != 0.0);

        pid.reset();

        assert_eq!(pid.output(), 0.0);
        assert_eq!(pid.proportional(), 0.0);
        assert_eq!(pid.integral(), 0.0);
        assert_eq!(pid.derivative(), 0.0);
    }

    // 9. Config builder
    #[test]
    fn test_config_builder() {
        let pid = PidConfig::new(2.0, 0.5, 0.1)
            .with_output_limits(-10.0, 10.0)
            .with_integral_limits(-3.0, 3.0)
            .with_derivative_filter(0.5)
            .build();

        // Verify the gains were applied by checking proportional response
        let mut pid = pid;
        let out = pid.update(4.0, DT);
        // P = 2.0 * 4.0 = 8.0; I = 0.5 * 0.004 = 0.002; D = 0 (first step)
        // total ~ 8.002
        assert!(
            (out - 8.002).abs() < 1e-6,
            "Config builder output mismatch, got {}",
            out
        );

        // Verify output limits: push hard
        let out_big = pid.update(1000.0, DT);
        assert!(
            (out_big - 10.0).abs() < 1e-12,
            "Output should be clamped to 10.0, got {}",
            out_big
        );
    }

    // 10. Factory constructors (AGC and frequency tracking)
    #[test]
    fn test_factory_constructors() {
        // AGC PID
        let mut agc = agc_pid();
        let agc_out = agc.update(0.5, DT);
        assert!(
            agc_out > 0.0 && agc_out <= 1.0,
            "AGC output should be positive and bounded, got {}",
            agc_out
        );
        // AGC has no derivative
        assert!(
            agc.derivative().abs() < 1e-12,
            "AGC should have zero derivative term"
        );

        // Frequency tracking PID
        let mut freq = frequency_tracking_pid();
        let freq_out = freq.update(100.0, DT);
        assert!(
            freq_out > 0.0 && freq_out <= 1000.0,
            "Frequency PID output should be positive and bounded, got {}",
            freq_out
        );

        // Temperature PID
        let mut temp = temperature_pid();
        let temp_out = temp.update(1.0, DT);
        assert!(
            temp_out > 0.0 && temp_out <= 1.0,
            "Temperature PID output should be positive and bounded, got {}",
            temp_out
        );
    }
}
