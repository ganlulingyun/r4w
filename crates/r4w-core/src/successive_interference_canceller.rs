//! Successive Interference Cancellation (SIC) for multi-user detection.
//!
//! This module implements serial interference cancellation used in
//! Non-Orthogonal Multiple Access (NOMA) and Code Division Multiple Access
//! (CDMA) systems. The strongest user signal is detected first, reconstructed,
//! and subtracted from the composite signal. This process repeats for each
//! user in descending power order.
//!
//! # Example
//!
//! ```
//! use r4w_core::successive_interference_canceller::{
//!     SuccessiveInterferenceCanceller, Modulation, SicResult,
//! };
//!
//! // Two users with different power levels
//! let mut sic = SuccessiveInterferenceCanceller::new(2);
//! sic.set_modulation(Modulation::Bpsk);
//!
//! // User 0 at amplitude 2.0, user 1 at amplitude 1.0
//! let amplitudes = vec![2.0, 1.0];
//! // Composite received signal (two BPSK symbols superimposed)
//! // User 0 sent +2.0, user 1 sent +1.0 => composite = 3.0
//! let received: Vec<(f64, f64)> = vec![(3.0, 0.0), (-1.0, 0.0)];
//!
//! let result = sic.cancel(&received, &amplitudes);
//! assert_eq!(result.detected_symbols.len(), 2);
//! assert_eq!(result.detected_symbols[0].len(), 2);
//! ```

use std::f64::consts::PI;

// ─── Complex helpers ────────────────────────────────────────────────────────

type C64 = (f64, f64);

#[inline]
fn c_add(a: C64, b: C64) -> C64 {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: C64, b: C64) -> C64 {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_mul(a: C64, b: C64) -> C64 {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_scale(a: C64, s: f64) -> C64 {
    (a.0 * s, a.1 * s)
}

#[inline]
fn c_conj(a: C64) -> C64 {
    (a.0, -a.1)
}

#[inline]
fn c_mag_sq(a: C64) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

#[inline]
fn c_mag(a: C64) -> f64 {
    c_mag_sq(a).sqrt()
}

fn power(signal: &[C64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    signal.iter().map(|&s| c_mag_sq(s)).sum::<f64>() / signal.len() as f64
}

// ─── Public types ───────────────────────────────────────────────────────────

/// Modulation scheme for hard-decision detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Modulation {
    /// Binary Phase-Shift Keying (1 bit/symbol).
    Bpsk,
    /// Quadrature Phase-Shift Keying (2 bits/symbol).
    Qpsk,
}

/// Per-user SINR estimate.
#[derive(Debug, Clone)]
pub struct SinrEstimate {
    /// User index (power-ordered, 0 = strongest).
    pub user: usize,
    /// SINR in dB before cancellation of this user's interferers.
    pub sinr_before_db: f64,
    /// SINR in dB after prior users have been cancelled.
    pub sinr_after_db: f64,
}

/// Residual power measurement after one cancellation stage.
#[derive(Debug, Clone)]
pub struct StageResidual {
    /// Which user was cancelled (power-ordered index).
    pub user_cancelled: usize,
    /// Average residual power remaining after subtraction.
    pub residual_power: f64,
}

/// Output of the SIC process.
#[derive(Debug, Clone)]
pub struct SicResult {
    /// Detected symbols per user (outer index = user in power order).
    pub detected_symbols: Vec<Vec<C64>>,
    /// Residual signal after all users have been cancelled.
    pub residual: Vec<C64>,
    /// Residual power per cancellation stage.
    pub stage_residuals: Vec<StageResidual>,
    /// SINR estimates per user.
    pub sinr_estimates: Vec<SinrEstimate>,
    /// Number of iterations actually performed (for iterative mode).
    pub iterations_performed: usize,
}

/// Configuration and state for successive interference cancellation.
#[derive(Debug, Clone)]
pub struct SuccessiveInterferenceCanceller {
    num_users: usize,
    modulation: Modulation,
    max_iterations: usize,
    convergence_threshold_db: f64,
    /// Optional per-user spreading codes for CDMA-style despreading.
    spreading_codes: Option<Vec<Vec<C64>>>,
}

impl SuccessiveInterferenceCanceller {
    /// Create a new SIC processor for the given number of users.
    ///
    /// Defaults: BPSK modulation, 1 iteration, -40 dB convergence threshold.
    pub fn new(num_users: usize) -> Self {
        assert!(num_users > 0, "num_users must be >= 1");
        Self {
            num_users,
            modulation: Modulation::Bpsk,
            max_iterations: 1,
            convergence_threshold_db: -40.0,
            spreading_codes: None,
        }
    }

    /// Set the modulation scheme used for hard-decision detection.
    pub fn set_modulation(&mut self, m: Modulation) {
        self.modulation = m;
    }

    /// Set the maximum number of iterative cancellation passes.
    pub fn set_max_iterations(&mut self, n: usize) {
        assert!(n >= 1, "max_iterations must be >= 1");
        self.max_iterations = n;
    }

    /// Set the convergence threshold in dB. Iteration stops when the
    /// change in residual power between passes falls below this value.
    pub fn set_convergence_threshold_db(&mut self, db: f64) {
        self.convergence_threshold_db = db;
    }

    /// Assign per-user spreading codes for CDMA-style despreading.
    ///
    /// Each code must have the same length (the spreading factor).
    /// The received signal length must be a multiple of that spreading factor.
    pub fn set_spreading_codes(&mut self, codes: Vec<Vec<C64>>) {
        assert_eq!(codes.len(), self.num_users, "one code per user required");
        if codes.len() > 1 {
            let sf = codes[0].len();
            for c in &codes {
                assert_eq!(c.len(), sf, "all spreading codes must have the same length");
            }
        }
        self.spreading_codes = Some(codes);
    }

    /// Return the configured number of users.
    pub fn num_users(&self) -> usize {
        self.num_users
    }

    /// Return the configured modulation.
    pub fn modulation(&self) -> Modulation {
        self.modulation
    }

    // ── Core algorithm ──────────────────────────────────────────────────

    /// Run SIC on the received composite signal.
    ///
    /// `amplitudes` contains the known (or estimated) amplitude of each user.
    /// Users are internally sorted by descending power; the result vectors
    /// follow the same power-descending order.
    pub fn cancel(&self, received: &[C64], amplitudes: &[f64]) -> SicResult {
        assert_eq!(
            amplitudes.len(),
            self.num_users,
            "amplitude count must match num_users"
        );

        // Build power-sorted user order (descending amplitude).
        let mut order: Vec<usize> = (0..self.num_users).collect();
        order.sort_by(|&a, &b| {
            amplitudes[b]
                .partial_cmp(&amplitudes[a])
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut residual: Vec<C64> = received.to_vec();
        let mut detected_symbols: Vec<Vec<C64>> = vec![vec![]; self.num_users];
        let mut stage_residuals: Vec<StageResidual> = Vec::new();
        let mut sinr_estimates: Vec<SinrEstimate> = Vec::new();

        let mut prev_residual_power = power(&residual);
        let mut iterations_performed = 0;

        for iter in 0..self.max_iterations {
            iterations_performed = iter + 1;

            // Re-run detection/subtraction per user in power order.
            residual = received.to_vec();

            // On iterations > 0, subtract all previously-detected users first
            // (this gives iterative refinement).
            if iter > 0 {
                // Re-subtract all users using previous iteration's symbols,
                // then re-detect each one. But we do this incrementally below.
                residual = received.to_vec();
            }

            stage_residuals.clear();
            sinr_estimates.clear();

            for (rank, &uid) in order.iter().enumerate() {
                // SINR before cancellation: signal power of this user vs everything else.
                let user_power = amplitudes[uid] * amplitudes[uid];
                let interference_power = power(&residual);
                let sinr_before = if interference_power > 0.0 {
                    10.0 * (user_power / interference_power).log10()
                } else {
                    f64::INFINITY
                };

                // Detect symbols for this user.
                let symbols = self.detect_user(&residual, amplitudes[uid], uid);
                detected_symbols[rank] = symbols.clone();

                // Reconstruct and subtract.
                let reconstructed = self.reconstruct(&symbols, amplitudes[uid], uid);
                for (r, &recon) in residual.iter_mut().zip(reconstructed.iter()) {
                    *r = c_sub(*r, recon);
                }

                let res_power = power(&residual);
                stage_residuals.push(StageResidual {
                    user_cancelled: rank,
                    residual_power: res_power,
                });

                // SINR after: this user's power vs residual.
                let sinr_after = if res_power > 0.0 {
                    10.0 * (user_power / res_power).log10()
                } else {
                    f64::INFINITY
                };
                sinr_estimates.push(SinrEstimate {
                    user: rank,
                    sinr_before_db: sinr_before,
                    sinr_after_db: sinr_after,
                });
            }

            // Check convergence.
            let current_residual_power = power(&residual);
            if iter > 0 {
                let change_db = if prev_residual_power > 0.0 {
                    10.0 * ((current_residual_power - prev_residual_power).abs()
                        / prev_residual_power)
                        .log10()
                } else {
                    f64::NEG_INFINITY
                };
                if change_db < self.convergence_threshold_db {
                    break;
                }
            }
            prev_residual_power = current_residual_power;
        }

        SicResult {
            detected_symbols,
            residual,
            stage_residuals,
            sinr_estimates,
            iterations_performed,
        }
    }

    // ── Internal helpers ────────────────────────────────────────────────

    /// Detect symbols for one user from the (partially-cleaned) signal.
    fn detect_user(&self, signal: &[C64], amplitude: f64, user_idx: usize) -> Vec<C64> {
        match &self.spreading_codes {
            Some(codes) => self.detect_user_cdma(signal, amplitude, &codes[user_idx]),
            None => self.detect_user_direct(signal, amplitude),
        }
    }

    /// Direct (NOMA-style) detection — one symbol per sample.
    fn detect_user_direct(&self, signal: &[C64], amplitude: f64) -> Vec<C64> {
        signal
            .iter()
            .map(|&s| {
                let normalised = if amplitude > 0.0 {
                    c_scale(s, 1.0 / amplitude)
                } else {
                    s
                };
                self.hard_decision(normalised)
            })
            .collect()
    }

    /// CDMA-style detection: despread, hard-decide, one symbol per chip period.
    fn detect_user_cdma(&self, signal: &[C64], amplitude: f64, code: &[C64]) -> Vec<C64> {
        let sf = code.len();
        if sf == 0 {
            return vec![];
        }
        let num_symbols = signal.len() / sf;
        let mut symbols = Vec::with_capacity(num_symbols);
        let code_energy: f64 = code.iter().map(|&c| c_mag_sq(c)).sum();

        for i in 0..num_symbols {
            let chunk = &signal[i * sf..(i + 1) * sf];
            // Correlate with conjugate of spreading code.
            let mut corr: C64 = (0.0, 0.0);
            for (s, c) in chunk.iter().zip(code.iter()) {
                corr = c_add(corr, c_mul(*s, c_conj(*c)));
            }
            // Normalise by code energy and user amplitude.
            let scale = if amplitude * code_energy > 0.0 {
                1.0 / (amplitude * code_energy)
            } else {
                1.0
            };
            let normalised = c_scale(corr, scale);
            symbols.push(self.hard_decision(normalised));
        }
        symbols
    }

    /// Hard-decision slicer.
    fn hard_decision(&self, s: C64) -> C64 {
        match self.modulation {
            Modulation::Bpsk => {
                if s.0 >= 0.0 {
                    (1.0, 0.0)
                } else {
                    (-1.0, 0.0)
                }
            }
            Modulation::Qpsk => {
                let inv_sqrt2 = 1.0 / 2.0_f64.sqrt();
                let re = if s.0 >= 0.0 { inv_sqrt2 } else { -inv_sqrt2 };
                let im = if s.1 >= 0.0 { inv_sqrt2 } else { -inv_sqrt2 };
                (re, im)
            }
        }
    }

    /// Reconstruct the transmitted signal for a user given detected symbols.
    fn reconstruct(&self, symbols: &[C64], amplitude: f64, user_idx: usize) -> Vec<C64> {
        match &self.spreading_codes {
            Some(codes) => self.reconstruct_cdma(symbols, amplitude, &codes[user_idx]),
            None => self.reconstruct_direct(symbols, amplitude),
        }
    }

    /// Direct reconstruction: scale symbols by amplitude.
    fn reconstruct_direct(&self, symbols: &[C64], amplitude: f64) -> Vec<C64> {
        symbols.iter().map(|&s| c_scale(s, amplitude)).collect()
    }

    /// CDMA reconstruction: spread each symbol back with the code, scale by amplitude.
    fn reconstruct_cdma(&self, symbols: &[C64], amplitude: f64, code: &[C64]) -> Vec<C64> {
        let sf = code.len();
        let mut out = Vec::with_capacity(symbols.len() * sf);
        for &sym in symbols {
            for &chip in code {
                out.push(c_scale(c_mul(sym, chip), amplitude));
            }
        }
        out
    }

    /// Estimate SINR for each user given known amplitudes.
    ///
    /// Returns SINR in dB for each user assuming all other users are
    /// interference (no cancellation), plus thermal noise power `noise_power`.
    pub fn estimate_sinr_no_cancellation(
        &self,
        amplitudes: &[f64],
        noise_power: f64,
    ) -> Vec<f64> {
        assert_eq!(amplitudes.len(), self.num_users);
        let mut sinrs = Vec::with_capacity(self.num_users);
        for i in 0..self.num_users {
            let sig = amplitudes[i] * amplitudes[i];
            let interference: f64 = amplitudes
                .iter()
                .enumerate()
                .filter(|&(j, _)| j != i)
                .map(|(_, &a)| a * a)
                .sum();
            let sinr = sig / (interference + noise_power);
            sinrs.push(10.0 * sinr.log10());
        }
        sinrs
    }
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-9;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn c_approx_eq(a: C64, b: C64, tol: f64) -> bool {
        approx_eq(a.0, b.0, tol) && approx_eq(a.1, b.1, tol)
    }

    // 1. Basic construction
    #[test]
    fn test_new_default() {
        let sic = SuccessiveInterferenceCanceller::new(3);
        assert_eq!(sic.num_users(), 3);
        assert_eq!(sic.modulation(), Modulation::Bpsk);
    }

    // 2. Panics on zero users
    #[test]
    #[should_panic(expected = "num_users must be >= 1")]
    fn test_zero_users_panics() {
        SuccessiveInterferenceCanceller::new(0);
    }

    // 3. BPSK hard decision
    #[test]
    fn test_bpsk_hard_decision() {
        let sic = SuccessiveInterferenceCanceller::new(1);
        assert_eq!(sic.hard_decision((0.5, 0.0)), (1.0, 0.0));
        assert_eq!(sic.hard_decision((-0.3, 0.0)), (-1.0, 0.0));
        assert_eq!(sic.hard_decision((0.0, 0.0)), (1.0, 0.0)); // boundary -> +1
    }

    // 4. QPSK hard decision
    #[test]
    fn test_qpsk_hard_decision() {
        let mut sic = SuccessiveInterferenceCanceller::new(1);
        sic.set_modulation(Modulation::Qpsk);
        let inv = 1.0 / 2.0_f64.sqrt();
        // First quadrant
        let d = sic.hard_decision((0.8, 0.3));
        assert!(c_approx_eq(d, (inv, inv), TOL));
        // Third quadrant
        let d = sic.hard_decision((-0.1, -0.9));
        assert!(c_approx_eq(d, (-inv, -inv), TOL));
    }

    // 5. Single user BPSK — trivial cancellation
    #[test]
    fn test_single_user_bpsk() {
        let sic = SuccessiveInterferenceCanceller::new(1);
        let received = vec![(2.0, 0.0), (-2.0, 0.0), (2.0, 0.0)];
        let amps = vec![2.0];
        let result = sic.cancel(&received, &amps);
        assert_eq!(result.detected_symbols.len(), 1);
        assert_eq!(result.detected_symbols[0].len(), 3);
        assert!(c_approx_eq(result.detected_symbols[0][0], (1.0, 0.0), TOL));
        assert!(c_approx_eq(result.detected_symbols[0][1], (-1.0, 0.0), TOL));
        assert!(c_approx_eq(result.detected_symbols[0][2], (1.0, 0.0), TOL));
        // Residual should be near zero.
        for &r in &result.residual {
            assert!(c_mag(r) < TOL);
        }
    }

    // 6. Two-user BPSK NOMA — same-sign symbols
    #[test]
    fn test_two_user_bpsk_same_sign() {
        let sic = SuccessiveInterferenceCanceller::new(2);
        // User 0: amp 3.0, symbol +1; User 1: amp 1.0, symbol +1
        // Composite: +4.0
        let received = vec![(4.0, 0.0)];
        let amps = vec![3.0, 1.0];
        let result = sic.cancel(&received, &amps);
        // User 0 detected first (stronger)
        assert!(c_approx_eq(result.detected_symbols[0][0], (1.0, 0.0), TOL));
        assert!(c_approx_eq(result.detected_symbols[1][0], (1.0, 0.0), TOL));
    }

    // 7. Two-user BPSK NOMA — opposite-sign symbols
    #[test]
    fn test_two_user_bpsk_opposite_sign() {
        let sic = SuccessiveInterferenceCanceller::new(2);
        // User 0: amp 3.0, symbol +1; User 1: amp 1.0, symbol -1
        // Composite: +3.0 - 1.0 = +2.0
        let received = vec![(2.0, 0.0)];
        let amps = vec![3.0, 1.0];
        let result = sic.cancel(&received, &amps);
        assert!(c_approx_eq(result.detected_symbols[0][0], (1.0, 0.0), TOL));
        assert!(c_approx_eq(result.detected_symbols[1][0], (-1.0, 0.0), TOL));
    }

    // 8. Power ordering — weaker user listed first in amplitudes
    #[test]
    fn test_power_ordering() {
        let sic = SuccessiveInterferenceCanceller::new(2);
        // Amplitudes given as [weak, strong] — SIC should still detect strong first.
        // User 0 (weak): amp 1.0 symbol +1; User 1 (strong): amp 5.0 symbol -1
        // Composite: 1.0 - 5.0 = -4.0
        let received = vec![(-4.0, 0.0)];
        let amps = vec![1.0, 5.0];
        let result = sic.cancel(&received, &amps);
        // Rank 0 = strongest user = user 1 (amp 5.0), detected symbol = -1
        assert!(c_approx_eq(result.detected_symbols[0][0], (-1.0, 0.0), TOL));
        // Rank 1 = weaker user = user 0 (amp 1.0), residual after subtracting 5*(-1)= +5
        // residual = -4 - (5*(-1)) = -4+5 = +1, so detected +1
        assert!(c_approx_eq(result.detected_symbols[1][0], (1.0, 0.0), TOL));
    }

    // 9. Residual power tracking
    #[test]
    fn test_stage_residuals_decrease() {
        let sic = SuccessiveInterferenceCanceller::new(3);
        // Three users: amps 4, 2, 1; all send +1
        // Composite = 7.0
        let received = vec![(7.0, 0.0)];
        let amps = vec![4.0, 2.0, 1.0];
        let result = sic.cancel(&received, &amps);
        assert_eq!(result.stage_residuals.len(), 3);
        // Residual power should decrease (or stay same) at each stage.
        for i in 1..result.stage_residuals.len() {
            assert!(
                result.stage_residuals[i].residual_power
                    <= result.stage_residuals[i - 1].residual_power + TOL
            );
        }
    }

    // 10. SINR estimates are populated
    #[test]
    fn test_sinr_estimates_populated() {
        let sic = SuccessiveInterferenceCanceller::new(2);
        let received = vec![(3.0, 0.0)];
        let amps = vec![2.0, 1.0];
        let result = sic.cancel(&received, &amps);
        assert_eq!(result.sinr_estimates.len(), 2);
        // After cancellation, SINR should improve (after >= before) for the weaker user.
        let weaker = &result.sinr_estimates[1];
        assert!(weaker.sinr_after_db >= weaker.sinr_before_db);
    }

    // 11. Multiple iterations field
    #[test]
    fn test_iterations_performed() {
        let mut sic = SuccessiveInterferenceCanceller::new(2);
        sic.set_max_iterations(5);
        let received = vec![(3.0, 0.0)];
        let amps = vec![2.0, 1.0];
        let result = sic.cancel(&received, &amps);
        assert!(result.iterations_performed >= 1);
        assert!(result.iterations_performed <= 5);
    }

    // 12. Convergence stops iteration early
    #[test]
    fn test_convergence_stops_early() {
        let mut sic = SuccessiveInterferenceCanceller::new(2);
        sic.set_max_iterations(10);
        sic.set_convergence_threshold_db(-10.0);
        // Clean signal — should converge in 2 iterations.
        let received = vec![(5.0, 0.0), (-3.0, 0.0)];
        let amps = vec![4.0, 1.0];
        let result = sic.cancel(&received, &amps);
        assert!(result.iterations_performed <= 10);
        // With a clean signal the result should converge quickly.
        assert!(result.iterations_performed <= 3);
    }

    // 13. QPSK two-user detection
    #[test]
    fn test_qpsk_two_user() {
        let mut sic = SuccessiveInterferenceCanceller::new(2);
        sic.set_modulation(Modulation::Qpsk);
        let inv = 1.0 / 2.0_f64.sqrt();
        // User 0 (strong, amp 4): symbol (+inv, +inv)
        // User 1 (weak, amp 1): symbol (-inv, +inv)
        let u0 = c_scale((inv, inv), 4.0);
        let u1 = c_scale((-inv, inv), 1.0);
        let composite = c_add(u0, u1);
        let received = vec![composite];
        let amps = vec![4.0, 1.0];
        let result = sic.cancel(&received, &amps);
        assert!(c_approx_eq(result.detected_symbols[0][0], (inv, inv), 1e-6));
        assert!(c_approx_eq(result.detected_symbols[1][0], (-inv, inv), 1e-6));
    }

    // 14. CDMA spreading/despreading — single user
    #[test]
    fn test_cdma_single_user() {
        let mut sic = SuccessiveInterferenceCanceller::new(1);
        // Spreading code: [+1, +1, -1, +1]
        let code = vec![(1.0, 0.0), (1.0, 0.0), (-1.0, 0.0), (1.0, 0.0)];
        sic.set_spreading_codes(vec![code.clone()]);
        let amp = 2.0;
        // User sends symbol +1; spread signal = amp * [+1, +1, -1, +1]
        let received: Vec<C64> = code.iter().map(|&c| c_scale(c, amp)).collect();
        let result = sic.cancel(&received, &[amp]);
        assert_eq!(result.detected_symbols[0].len(), 1);
        assert!(c_approx_eq(result.detected_symbols[0][0], (1.0, 0.0), 1e-6));
    }

    // 15. CDMA two-user with orthogonal codes
    #[test]
    fn test_cdma_two_user_orthogonal() {
        let mut sic = SuccessiveInterferenceCanceller::new(2);
        // Walsh-Hadamard codes of length 4
        let code0 = vec![(1.0, 0.0), (1.0, 0.0), (1.0, 0.0), (1.0, 0.0)];
        let code1 = vec![(1.0, 0.0), (-1.0, 0.0), (1.0, 0.0), (-1.0, 0.0)];
        sic.set_spreading_codes(vec![code0.clone(), code1.clone()]);
        let amp0 = 3.0;
        let amp1 = 1.5;
        // User 0 sends +1, user 1 sends -1
        let mut received = vec![(0.0, 0.0); 4];
        for i in 0..4 {
            received[i] = c_add(c_scale(code0[i], amp0), c_scale(code1[i], -amp1));
        }
        let result = sic.cancel(&received, &[amp0, amp1]);
        // Strongest user (amp0=3.0) detected first -> +1
        assert!(c_approx_eq(result.detected_symbols[0][0], (1.0, 0.0), 1e-6));
        // Weaker user (amp1=1.5) -> -1
        assert!(c_approx_eq(result.detected_symbols[1][0], (-1.0, 0.0), 1e-6));
    }

    // 16. Direct SINR estimation (no cancellation)
    #[test]
    fn test_estimate_sinr_no_cancellation() {
        let sic = SuccessiveInterferenceCanceller::new(2);
        let amps = vec![4.0, 1.0];
        let noise = 0.01;
        let sinrs = sic.estimate_sinr_no_cancellation(&amps, noise);
        assert_eq!(sinrs.len(), 2);
        // Stronger user should have higher SINR.
        assert!(sinrs[0] > sinrs[1]);
        // User 0 SINR = 16/(1+0.01) ≈ 15.84 → ~12.0 dB
        let expected_0 = 10.0 * (16.0 / 1.01_f64).log10();
        assert!(approx_eq(sinrs[0], expected_0, 0.01));
    }

    // 17. Empty signal handling
    #[test]
    fn test_empty_signal() {
        let sic = SuccessiveInterferenceCanceller::new(1);
        let result = sic.cancel(&[], &[1.0]);
        assert!(result.detected_symbols[0].is_empty());
        assert!(result.residual.is_empty());
    }

    // 18. Multi-symbol sequence
    #[test]
    fn test_multi_symbol_sequence() {
        let sic = SuccessiveInterferenceCanceller::new(2);
        let amp0 = 5.0;
        let amp1 = 2.0;
        // User 0 symbols: +1, -1, +1, -1
        // User 1 symbols: +1, +1, -1, -1
        let user0_syms: Vec<f64> = vec![1.0, -1.0, 1.0, -1.0];
        let user1_syms: Vec<f64> = vec![1.0, 1.0, -1.0, -1.0];
        let received: Vec<C64> = user0_syms
            .iter()
            .zip(user1_syms.iter())
            .map(|(&s0, &s1)| (s0 * amp0 + s1 * amp1, 0.0))
            .collect();
        let result = sic.cancel(&received, &[amp0, amp1]);
        // Verify all user 0 symbols (strongest)
        for (i, &s) in user0_syms.iter().enumerate() {
            assert!(c_approx_eq(
                result.detected_symbols[0][i],
                (s, 0.0),
                TOL
            ), "user 0 symbol {} mismatch", i);
        }
        // Verify all user 1 symbols (weaker)
        for (i, &s) in user1_syms.iter().enumerate() {
            assert!(c_approx_eq(
                result.detected_symbols[1][i],
                (s, 0.0),
                TOL
            ), "user 1 symbol {} mismatch", i);
        }
    }

    // 19. Residual near zero for perfect cancellation
    #[test]
    fn test_perfect_cancellation_residual() {
        let sic = SuccessiveInterferenceCanceller::new(2);
        let amp0 = 3.0;
        let amp1 = 1.0;
        // Both send +1 => composite = 4.0
        let received = vec![(4.0, 0.0)];
        let result = sic.cancel(&received, &[amp0, amp1]);
        assert!(power(&result.residual) < 1e-15);
    }

    // 20. Three-user BPSK
    #[test]
    fn test_three_user_bpsk() {
        let sic = SuccessiveInterferenceCanceller::new(3);
        let amps = vec![6.0, 3.0, 1.0];
        // Symbols: +1, -1, +1
        // Composite: 6 - 3 + 1 = 4
        let received = vec![(4.0, 0.0)];
        let result = sic.cancel(&received, &amps);
        assert!(c_approx_eq(result.detected_symbols[0][0], (1.0, 0.0), TOL));
        assert!(c_approx_eq(result.detected_symbols[1][0], (-1.0, 0.0), TOL));
        assert!(c_approx_eq(result.detected_symbols[2][0], (1.0, 0.0), TOL));
    }

    // 21. Reconstruct round-trip (direct)
    #[test]
    fn test_reconstruct_round_trip() {
        let sic = SuccessiveInterferenceCanceller::new(1);
        let symbols = vec![(1.0, 0.0), (-1.0, 0.0)];
        let amp = 3.5;
        let recon = sic.reconstruct_direct(&symbols, amp);
        assert!(c_approx_eq(recon[0], (3.5, 0.0), TOL));
        assert!(c_approx_eq(recon[1], (-3.5, 0.0), TOL));
    }

    // 22. CDMA reconstruct round-trip
    #[test]
    fn test_cdma_reconstruct_round_trip() {
        let sic = SuccessiveInterferenceCanceller::new(1);
        let code = vec![(1.0, 0.0), (-1.0, 0.0)];
        let symbols = vec![(1.0, 0.0)];
        let amp = 2.0;
        let recon = sic.reconstruct_cdma(&symbols, amp, &code);
        assert_eq!(recon.len(), 2);
        assert!(c_approx_eq(recon[0], (2.0, 0.0), TOL));
        assert!(c_approx_eq(recon[1], (-2.0, 0.0), TOL));
    }

    // 23. Modulation setter
    #[test]
    fn test_set_modulation() {
        let mut sic = SuccessiveInterferenceCanceller::new(1);
        sic.set_modulation(Modulation::Qpsk);
        assert_eq!(sic.modulation(), Modulation::Qpsk);
    }

    // 24. Max iterations setter panics on zero
    #[test]
    #[should_panic(expected = "max_iterations must be >= 1")]
    fn test_zero_iterations_panics() {
        let mut sic = SuccessiveInterferenceCanceller::new(1);
        sic.set_max_iterations(0);
    }

    // 25. Power helper
    #[test]
    fn test_power_helper() {
        let signal = vec![(3.0, 4.0)]; // mag^2 = 25
        assert!(approx_eq(power(&signal), 25.0, TOL));
        assert!(approx_eq(power(&[]), 0.0, TOL));
    }

    // 26. Complex arithmetic helpers
    #[test]
    fn test_complex_helpers() {
        assert_eq!(c_add((1.0, 2.0), (3.0, 4.0)), (4.0, 6.0));
        assert_eq!(c_sub((5.0, 3.0), (2.0, 1.0)), (3.0, 2.0));
        // (1+2i)*(3+4i) = 3+4i+6i+8i^2 = (3-8)+(4+6)i = (-5, 10)
        assert!(c_approx_eq(c_mul((1.0, 2.0), (3.0, 4.0)), (-5.0, 10.0), TOL));
        assert_eq!(c_conj((1.0, 2.0)), (1.0, -2.0));
        assert!(approx_eq(c_mag((3.0, 4.0)), 5.0, TOL));
    }
}
