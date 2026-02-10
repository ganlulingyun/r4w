//! Polyphase matched filter bank for multi-rate symbol timing recovery.
//!
//! This module implements a Root Raised Cosine (RRC) polyphase filter bank
//! for use in symbol timing recovery. The filter bank decomposes a prototype
//! RRC filter into N sub-filters (phases), enabling efficient fractional-delay
//! interpolation without recomputing filter coefficients at runtime.
//!
//! The derivative filters (computed via finite differences of adjacent phases)
//! are used as part of a Timing Error Detector (TED) in a PI-controlled
//! feedback loop for symbol timing tracking.
//!
//! # Example
//!
//! ```
//! use r4w_core::root_raised_cosine_matched_filter_bank::{
//!     RrcFilterBank, FilterBankConfig, TimingRecoveryMfb,
//! };
//!
//! // Configure a 32-phase RRC filter bank with rolloff 0.35, 4 sps
//! let config = FilterBankConfig {
//!     num_phases: 32,
//!     taps_per_phase: 8,
//!     rolloff: 0.35,
//!     sps: 4,
//! };
//!
//! let bank = RrcFilterBank::new(config);
//! assert_eq!(bank.num_phases(), 32);
//!
//! // Create a timing recovery engine from the filter bank
//! let mut recovery = TimingRecoveryMfb::new(bank);
//!
//! // Generate some simple test samples (4 sps, one BPSK symbol = +1)
//! let samples: Vec<(f64, f64)> = (0..8)
//!     .map(|i| if i == 0 { (1.0, 0.0) } else { (0.0, 0.0) })
//!     .collect();
//!
//! let result = recovery.process_symbol(&samples);
//! assert!(result.phase_index < 32);
//! // The fractional timing offset mu is in [0, 1)
//! assert!(recovery.get_mu() >= 0.0 && recovery.get_mu() < 1.0);
//! ```

use std::f64::consts::PI;

/// Configuration for the polyphase RRC filter bank.
#[derive(Debug, Clone)]
pub struct FilterBankConfig {
    /// Number of polyphase sub-filters (phases). Typically 16–64.
    pub num_phases: usize,
    /// Number of taps in each polyphase sub-filter.
    pub taps_per_phase: usize,
    /// RRC rolloff factor, in the range \[0.0, 1.0\].
    pub rolloff: f64,
    /// Samples per symbol in the input signal.
    pub sps: usize,
}

/// Result of filter-bank-based timing recovery for one symbol.
#[derive(Debug, Clone)]
pub struct InterpolationResult {
    /// The interpolated complex sample at the recovered timing instant.
    pub interpolated_sample: (f64, f64),
    /// Index of the polyphase filter phase that was selected.
    pub phase_index: usize,
    /// Estimated timing error from the TED.
    pub timing_error: f64,
}

/// Polyphase Root Raised Cosine filter bank.
///
/// Decomposes a prototype RRC filter of length `num_phases * taps_per_phase`
/// into `num_phases` sub-filters. Also pre-computes derivative filters via
/// finite differences of adjacent phases for use in timing error detection.
#[derive(Debug, Clone)]
pub struct RrcFilterBank {
    config: FilterBankConfig,
    /// Polyphase sub-filters stored as complex taps: `phases[phase][tap]`.
    phases: Vec<Vec<(f64, f64)>>,
    /// Derivative filters computed from finite differences of adjacent phases.
    derivative_phases: Vec<Vec<(f64, f64)>>,
}

impl RrcFilterBank {
    /// Create a new polyphase RRC filter bank from the given configuration.
    ///
    /// # Panics
    ///
    /// Panics if `num_phases < 2`, `taps_per_phase < 1`, `rolloff` is outside
    /// \[0.0, 1.0\], or `sps < 1`.
    pub fn new(config: FilterBankConfig) -> Self {
        assert!(config.num_phases >= 2, "num_phases must be >= 2");
        assert!(config.taps_per_phase >= 1, "taps_per_phase must be >= 1");
        assert!(
            (0.0..=1.0).contains(&config.rolloff),
            "rolloff must be in [0.0, 1.0]"
        );
        assert!(config.sps >= 1, "sps must be >= 1");

        let total_taps = config.num_phases * config.taps_per_phase;
        let prototype = Self::design_rrc_prototype(total_taps, config.rolloff, config.sps, config.num_phases);

        // Polyphase decomposition: split prototype into N sub-filters.
        // Phase p gets taps at indices p, p + N, p + 2N, ...
        let n = config.num_phases;
        let mut phases = vec![vec![(0.0, 0.0); config.taps_per_phase]; n];
        for p in 0..n {
            for k in 0..config.taps_per_phase {
                let idx = p + k * n;
                if idx < prototype.len() {
                    phases[p][k] = (prototype[idx], 0.0);
                }
            }
        }

        // Derivative filters via finite differences of adjacent phases.
        let mut derivative_phases = vec![vec![(0.0, 0.0); config.taps_per_phase]; n];
        for p in 0..n {
            let next = (p + 1) % n;
            for k in 0..config.taps_per_phase {
                let di = phases[next][k].0 - phases[p][k].0;
                let dq = phases[next][k].1 - phases[p][k].1;
                derivative_phases[p] = derivative_phases[p].clone();
                derivative_phases[p][k] = (di, dq);
            }
        }

        Self {
            config,
            phases,
            derivative_phases,
        }
    }

    /// Design the prototype RRC filter impulse response.
    ///
    /// Uses the standard RRC pulse shape formula:
    ///
    /// h(t) = \[sin(π·t·(1−α)) + 4·α·t·cos(π·t·(1+α))\] / \[π·t·(1−(4·α·t)²)\]
    ///
    /// with proper handling of the singularities at t=0 and t=±1/(4α).
    fn design_rrc_prototype(
        total_taps: usize,
        rolloff: f64,
        sps: usize,
        num_phases: usize,
    ) -> Vec<f64> {
        let mut h = vec![0.0; total_taps];
        let center = (total_taps as f64 - 1.0) / 2.0;
        let a = rolloff;
        // Effective samples-per-symbol for the oversampled prototype.
        let t_sps = (sps * num_phases) as f64;

        for i in 0..total_taps {
            let t = (i as f64 - center) / t_sps;

            if t.abs() < 1e-12 {
                // t == 0: h(0) = 1 - a + 4*a/pi
                h[i] = 1.0 - a + 4.0 * a / PI;
            } else if a > 0.0 && (1.0 - (4.0 * a * t).powi(2)).abs() < 1e-12 {
                // t = ±1/(4α): special limit
                h[i] = (a / (2.0_f64).sqrt())
                    * ((1.0 + 2.0 / PI) * (PI / (4.0 * a)).sin()
                        + (1.0 - 2.0 / PI) * (PI / (4.0 * a)).cos());
            } else {
                let num = (PI * t * (1.0 - a)).sin() + 4.0 * a * t * (PI * t * (1.0 + a)).cos();
                let den = PI * t * (1.0 - (4.0 * a * t).powi(2));
                h[i] = num / den;
            }
        }

        // Normalize energy
        let energy: f64 = h.iter().map(|x| x * x).sum::<f64>().sqrt();
        if energy > 1e-12 {
            for x in h.iter_mut() {
                *x /= energy;
            }
        }

        h
    }

    /// Return the number of polyphase phases.
    pub fn num_phases(&self) -> usize {
        self.config.num_phases
    }

    /// Return the configuration.
    pub fn config(&self) -> &FilterBankConfig {
        &self.config
    }

    /// Filter the input through a specific polyphase phase, producing one output
    /// sample per `num_phases` input samples.
    ///
    /// The output length equals `input.len() / taps_per_phase` (rounded down),
    /// computed by convolving the sub-filter with the input at the appropriate
    /// decimation rate.
    pub fn filter_phase(&self, input: &[(f64, f64)], phase: usize) -> Vec<(f64, f64)> {
        assert!(phase < self.config.num_phases, "phase index out of range");

        let taps = &self.phases[phase];
        let n_taps = taps.len();
        if input.len() < n_taps {
            return vec![];
        }

        let out_len = input.len() - n_taps + 1;
        let mut output = Vec::with_capacity(out_len);

        for i in 0..out_len {
            let mut acc_i = 0.0;
            let mut acc_q = 0.0;
            for (k, tap) in taps.iter().enumerate() {
                let s = input[i + k];
                // Complex multiply: (tap.0 + j*tap.1) * (s.0 + j*s.1)
                acc_i += tap.0 * s.0 - tap.1 * s.1;
                acc_q += tap.0 * s.1 + tap.1 * s.0;
            }
            output.push((acc_i, acc_q));
        }

        output
    }

    /// Perform fractional-delay interpolation on the input signal.
    ///
    /// `mu` is the fractional timing offset in \[0.0, 1.0), mapped to the
    /// closest polyphase phase. The input must have at least `taps_per_phase`
    /// samples.
    pub fn interpolate(&self, input: &[(f64, f64)], mu: f64) -> (f64, f64) {
        assert!(
            input.len() >= self.config.taps_per_phase,
            "input too short for interpolation"
        );

        let mu_clamped = mu.clamp(0.0, 1.0 - 1e-12);
        let phase_f = mu_clamped * self.config.num_phases as f64;
        let phase_idx = phase_f as usize;
        let frac = phase_f - phase_idx as f64;

        let taps = &self.phases[phase_idx % self.config.num_phases];
        let dtaps = &self.derivative_phases[phase_idx % self.config.num_phases];

        let mut acc_i = 0.0;
        let mut acc_q = 0.0;

        // Linear interpolation between phase and its derivative for sub-phase accuracy.
        for k in 0..self.config.taps_per_phase {
            if k >= input.len() {
                break;
            }
            let tap_i = taps[k].0 + frac * dtaps[k].0;
            let tap_q = taps[k].1 + frac * dtaps[k].1;
            let s = input[k];
            acc_i += tap_i * s.0 - tap_q * s.1;
            acc_q += tap_i * s.1 + tap_q * s.0;
        }

        (acc_i, acc_q)
    }

    /// Return a reference to the derivative filter taps for the given phase.
    pub fn derivative_filter(&self, phase: usize) -> &[(f64, f64)] {
        assert!(phase < self.config.num_phases, "phase index out of range");
        &self.derivative_phases[phase]
    }

    /// Return a reference to the filter taps for the given phase.
    pub fn phase_taps(&self, phase: usize) -> &[(f64, f64)] {
        assert!(phase < self.config.num_phases, "phase index out of range");
        &self.phases[phase]
    }
}

/// Filter bank-based timing recovery with a PI-controlled TED loop.
///
/// Uses the polyphase matched filter bank output and its derivative to compute
/// a timing error, then adjusts a fractional phase offset `mu` via a PI loop
/// filter.
#[derive(Debug, Clone)]
pub struct TimingRecoveryMfb {
    bank: RrcFilterBank,
    /// Fractional timing offset in \[0.0, 1.0).
    mu: f64,
    /// PI loop proportional gain.
    kp: f64,
    /// PI loop integral gain.
    ki: f64,
    /// Loop integrator state.
    integrator: f64,
    /// Previous interpolated sample (for TED).
    prev_sample: (f64, f64),
}

impl TimingRecoveryMfb {
    /// Create a new timing recovery engine from the given filter bank.
    ///
    /// Uses default PI loop gains suitable for moderate loop bandwidth.
    pub fn new(bank: RrcFilterBank) -> Self {
        // Default loop gains for a normalized loop bandwidth ~ 0.01
        let kp = 0.01;
        let ki = kp * kp / 4.0; // critically damped
        Self {
            bank,
            mu: 0.5,
            kp,
            ki,
            integrator: 0.0,
            prev_sample: (0.0, 0.0),
        }
    }

    /// Create a new timing recovery engine with custom PI loop gains.
    pub fn with_gains(bank: RrcFilterBank, kp: f64, ki: f64) -> Self {
        Self {
            bank,
            mu: 0.5,
            kp,
            ki,
            integrator: 0.0,
            prev_sample: (0.0, 0.0),
        }
    }

    /// Process one symbol's worth of samples and return the interpolation result.
    ///
    /// The input `samples` should contain at least `taps_per_phase` samples
    /// centered around the expected symbol timing instant.
    pub fn process_symbol(&mut self, samples: &[(f64, f64)]) -> InterpolationResult {
        let n = self.bank.num_phases();
        let mu_clamped = self.mu.clamp(0.0, 1.0 - 1e-12);

        // Map mu to a phase index
        let phase_f = mu_clamped * n as f64;
        let phase_idx = (phase_f as usize).min(n - 1);

        // Interpolate using the matched filter bank
        let interp = self.bank.interpolate(samples, mu_clamped);

        // Compute timing error using the derivative filter (ML TED).
        // e = Re{ conj(y) * dy/dmu }
        let dtaps = self.bank.derivative_filter(phase_idx);
        let mut dy_i = 0.0;
        let mut dy_q = 0.0;
        for k in 0..dtaps.len().min(samples.len()) {
            let s = samples[k];
            dy_i += dtaps[k].0 * s.0 - dtaps[k].1 * s.1;
            dy_q += dtaps[k].0 * s.1 + dtaps[k].1 * s.0;
        }

        // Error = Re{ conj(interp) * (dy_i, dy_q) }
        let timing_error = interp.0 * dy_i + interp.1 * dy_q;

        // PI loop filter
        self.integrator += self.ki * timing_error;
        let loop_out = self.kp * timing_error + self.integrator;

        // Update mu
        self.mu -= loop_out;

        // Keep mu in [0, 1)
        while self.mu >= 1.0 {
            self.mu -= 1.0;
        }
        while self.mu < 0.0 {
            self.mu += 1.0;
        }

        self.prev_sample = interp;

        InterpolationResult {
            interpolated_sample: interp,
            phase_index: phase_idx,
            timing_error,
        }
    }

    /// Return the current fractional timing offset mu in \[0.0, 1.0).
    pub fn get_mu(&self) -> f64 {
        self.mu
    }

    /// Return the number of polyphase phases in the underlying filter bank.
    pub fn num_phases(&self) -> usize {
        self.bank.num_phases()
    }

    /// Set the fractional timing offset mu directly (for testing or manual override).
    pub fn set_mu(&mut self, mu: f64) {
        self.mu = mu.rem_euclid(1.0);
    }

    /// Reset the loop filter state.
    pub fn reset(&mut self) {
        self.mu = 0.5;
        self.integrator = 0.0;
        self.prev_sample = (0.0, 0.0);
    }

    /// Return a reference to the underlying filter bank.
    pub fn filter_bank(&self) -> &RrcFilterBank {
        &self.bank
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> FilterBankConfig {
        FilterBankConfig {
            num_phases: 32,
            taps_per_phase: 8,
            rolloff: 0.35,
            sps: 4,
        }
    }

    #[test]
    fn test_config_creation() {
        let config = default_config();
        assert_eq!(config.num_phases, 32);
        assert_eq!(config.taps_per_phase, 8);
        assert!((config.rolloff - 0.35).abs() < 1e-10);
        assert_eq!(config.sps, 4);
    }

    #[test]
    fn test_bank_creation() {
        let bank = RrcFilterBank::new(default_config());
        assert_eq!(bank.num_phases(), 32);
        assert_eq!(bank.phases.len(), 32);
        assert_eq!(bank.derivative_phases.len(), 32);
    }

    #[test]
    fn test_phase_taps_length() {
        let config = default_config();
        let bank = RrcFilterBank::new(config.clone());
        for p in 0..config.num_phases {
            assert_eq!(bank.phase_taps(p).len(), config.taps_per_phase);
        }
    }

    #[test]
    fn test_derivative_filter_length() {
        let config = default_config();
        let bank = RrcFilterBank::new(config.clone());
        for p in 0..config.num_phases {
            assert_eq!(bank.derivative_filter(p).len(), config.taps_per_phase);
        }
    }

    #[test]
    fn test_derivative_is_finite_difference() {
        let bank = RrcFilterBank::new(default_config());
        // Derivative of phase p should equal phase(p+1) - phase(p)
        for p in 0..bank.num_phases() {
            let next = (p + 1) % bank.num_phases();
            let d = bank.derivative_filter(p);
            let cur = bank.phase_taps(p);
            let nxt = bank.phase_taps(next);
            for k in 0..d.len() {
                assert!(
                    (d[k].0 - (nxt[k].0 - cur[k].0)).abs() < 1e-12,
                    "derivative mismatch at phase {}, tap {}",
                    p,
                    k
                );
            }
        }
    }

    #[test]
    fn test_filter_phase_empty_input() {
        let bank = RrcFilterBank::new(default_config());
        let result = bank.filter_phase(&[], 0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_filter_phase_short_input() {
        let bank = RrcFilterBank::new(default_config());
        // Input shorter than taps_per_phase should return empty
        let input: Vec<(f64, f64)> = (0..4).map(|_| (1.0, 0.0)).collect();
        let result = bank.filter_phase(&input, 0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_filter_phase_output_length() {
        let config = default_config();
        let bank = RrcFilterBank::new(config.clone());
        let input: Vec<(f64, f64)> = (0..20).map(|_| (1.0, 0.0)).collect();
        let result = bank.filter_phase(&input, 0);
        assert_eq!(result.len(), 20 - config.taps_per_phase + 1);
    }

    #[test]
    fn test_interpolate_zero_mu() {
        let bank = RrcFilterBank::new(default_config());
        let input: Vec<(f64, f64)> = (0..8).map(|_| (1.0, 0.0)).collect();
        let result = bank.interpolate(&input, 0.0);
        // Should produce a finite result
        assert!(result.0.is_finite());
        assert!(result.1.is_finite());
    }

    #[test]
    fn test_interpolate_half_mu() {
        let bank = RrcFilterBank::new(default_config());
        let input: Vec<(f64, f64)> = (0..8).map(|_| (1.0, 0.0)).collect();
        let result = bank.interpolate(&input, 0.5);
        assert!(result.0.is_finite());
        assert!(result.1.is_finite());
    }

    #[test]
    fn test_interpolate_near_one_mu() {
        let bank = RrcFilterBank::new(default_config());
        let input: Vec<(f64, f64)> = (0..8).map(|_| (1.0, 0.0)).collect();
        let result = bank.interpolate(&input, 0.999);
        assert!(result.0.is_finite());
        assert!(result.1.is_finite());
    }

    #[test]
    fn test_timing_recovery_creation() {
        let bank = RrcFilterBank::new(default_config());
        let recovery = TimingRecoveryMfb::new(bank);
        assert_eq!(recovery.num_phases(), 32);
        assert!((recovery.get_mu() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_timing_recovery_process_symbol() {
        let bank = RrcFilterBank::new(default_config());
        let mut recovery = TimingRecoveryMfb::new(bank);
        let samples: Vec<(f64, f64)> = (0..8)
            .map(|i| if i == 0 { (1.0, 0.0) } else { (0.0, 0.0) })
            .collect();

        let result = recovery.process_symbol(&samples);
        assert!(result.phase_index < 32);
        assert!(result.timing_error.is_finite());
        assert!(result.interpolated_sample.0.is_finite());
        assert!(result.interpolated_sample.1.is_finite());
    }

    #[test]
    fn test_timing_recovery_mu_stays_bounded() {
        let bank = RrcFilterBank::new(default_config());
        let mut recovery = TimingRecoveryMfb::new(bank);
        let samples: Vec<(f64, f64)> = (0..8)
            .map(|i| if i == 0 { (1.0, 0.0) } else { (0.0, 0.0) })
            .collect();

        for _ in 0..100 {
            recovery.process_symbol(&samples);
            let mu = recovery.get_mu();
            assert!(
                mu >= 0.0 && mu < 1.0,
                "mu out of bounds: {}",
                mu
            );
        }
    }

    #[test]
    fn test_timing_recovery_reset() {
        let bank = RrcFilterBank::new(default_config());
        let mut recovery = TimingRecoveryMfb::new(bank);
        let samples: Vec<(f64, f64)> = (0..8).map(|_| (1.0, 0.0)).collect();
        recovery.process_symbol(&samples);
        recovery.reset();
        assert!((recovery.get_mu() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_set_mu() {
        let bank = RrcFilterBank::new(default_config());
        let mut recovery = TimingRecoveryMfb::new(bank);
        recovery.set_mu(0.75);
        assert!((recovery.get_mu() - 0.75).abs() < 1e-10);
        // Negative wraps
        recovery.set_mu(-0.25);
        assert!((recovery.get_mu() - 0.75).abs() < 1e-10);
        // > 1 wraps
        recovery.set_mu(1.3);
        assert!((recovery.get_mu() - 0.3).abs() < 1e-10);
    }

    #[test]
    fn test_different_num_phases() {
        for n in [16, 32, 48, 64] {
            let config = FilterBankConfig {
                num_phases: n,
                taps_per_phase: 8,
                rolloff: 0.25,
                sps: 4,
            };
            let bank = RrcFilterBank::new(config);
            assert_eq!(bank.num_phases(), n);
        }
    }

    #[test]
    fn test_rolloff_zero() {
        // rolloff = 0 is valid (sinc filter)
        let config = FilterBankConfig {
            num_phases: 16,
            taps_per_phase: 8,
            rolloff: 0.0,
            sps: 2,
        };
        let bank = RrcFilterBank::new(config);
        let input: Vec<(f64, f64)> = (0..8).map(|_| (1.0, 0.0)).collect();
        let result = bank.interpolate(&input, 0.5);
        assert!(result.0.is_finite());
    }

    #[test]
    fn test_rolloff_one() {
        // rolloff = 1.0 is valid (maximum bandwidth)
        let config = FilterBankConfig {
            num_phases: 16,
            taps_per_phase: 8,
            rolloff: 1.0,
            sps: 4,
        };
        let bank = RrcFilterBank::new(config);
        let input: Vec<(f64, f64)> = (0..8).map(|_| (1.0, 0.0)).collect();
        let result = bank.interpolate(&input, 0.25);
        assert!(result.0.is_finite());
    }

    #[test]
    #[should_panic(expected = "num_phases must be >= 2")]
    fn test_num_phases_too_small() {
        let config = FilterBankConfig {
            num_phases: 1,
            taps_per_phase: 8,
            rolloff: 0.35,
            sps: 4,
        };
        RrcFilterBank::new(config);
    }

    #[test]
    #[should_panic(expected = "phase index out of range")]
    fn test_filter_phase_out_of_range() {
        let bank = RrcFilterBank::new(default_config());
        bank.filter_phase(&[(1.0, 0.0)], 32);
    }

    #[test]
    fn test_prototype_filter_energy_normalized() {
        let config = default_config();
        let total_taps = config.num_phases * config.taps_per_phase;
        let h = RrcFilterBank::design_rrc_prototype(
            total_taps,
            config.rolloff,
            config.sps,
            config.num_phases,
        );
        let energy: f64 = h.iter().map(|x| x * x).sum::<f64>().sqrt();
        assert!(
            (energy - 1.0).abs() < 1e-6,
            "prototype energy not normalized: {}",
            energy
        );
    }

    #[test]
    fn test_custom_gains() {
        let bank = RrcFilterBank::new(default_config());
        let recovery = TimingRecoveryMfb::with_gains(bank, 0.05, 0.001);
        assert_eq!(recovery.num_phases(), 32);
        assert!((recovery.get_mu() - 0.5).abs() < 1e-10);
    }
}
