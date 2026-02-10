//! # Frequency Lock Detector
//!
//! Determines whether a frequency tracking loop (PLL, FLL, or carrier recovery)
//! has achieved lock. Provides three complementary approaches:
//!
//! - [`FreqLockDetector`] -- EWMA-based frequency error monitoring with
//!   hysteresis and hold counter to prevent chatter.
//! - [`PhaseCoherenceDetector`] -- Sliding-window phase variance detector;
//!   low variance implies coherent (locked) carrier.
//! - [`CombinedLockDetector`] -- Fuses both metrics for robust lock indication.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::freq_lock_detector::FreqLockDetector;
//!
//! let mut det = FreqLockDetector::new(0.1, 0.05);
//!
//! // Feed small frequency errors -- detector should lock
//! for _ in 0..100 {
//!     det.update(0.001);
//! }
//! assert!(det.is_locked());
//! assert!(det.confidence() > 0.9);
//!
//! // Feed large alternating errors -- detector should unlock
//! for i in 0..100 {
//!     let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
//!     det.update(sign * 1.0);
//! }
//! assert!(!det.is_locked());
//! ```

// ---------------------------------------------------------------------------
// FreqLockDetector
// ---------------------------------------------------------------------------

/// EWMA-based frequency lock detector.
///
/// Tracks the exponentially weighted moving average of the absolute frequency
/// error and its variance.  Lock is declared when the variance drops below
/// `lock_threshold` and held for at least `hold_threshold` consecutive
/// samples.  Unlock requires the variance to exceed `unlock_threshold`
/// (which defaults to 2x `lock_threshold`) providing hysteresis.
#[derive(Debug, Clone)]
pub struct FreqLockDetector {
    /// EWMA smoothing factor (0 < alpha <= 1). Smaller = slower response.
    alpha: f64,
    /// Variance threshold to declare lock.
    lock_threshold: f64,
    /// Variance threshold to declare unlock (hysteresis).
    unlock_threshold: f64,
    /// EWMA of frequency error.
    avg_freq_error: f64,
    /// EWMA of frequency error variance.
    avg_freq_error_var: f64,
    /// Current lock state.
    locked: bool,
    /// Counter of consecutive samples meeting lock criterion.
    hold_count: usize,
    /// Number of consecutive samples required before declaring lock.
    hold_threshold: usize,
}

impl FreqLockDetector {
    /// Create a new detector.
    ///
    /// * `alpha` -- EWMA smoothing factor (e.g. 0.01 - 0.2).
    /// * `lock_threshold` -- variance below which lock is declared.
    ///
    /// The unlock threshold defaults to `2 * lock_threshold` and the hold
    /// threshold defaults to 10 samples.
    pub fn new(alpha: f64, lock_threshold: f64) -> Self {
        Self {
            alpha,
            lock_threshold,
            unlock_threshold: 2.0 * lock_threshold,
            avg_freq_error: 0.0,
            avg_freq_error_var: 0.0,
            locked: false,
            hold_count: 0,
            hold_threshold: 10,
        }
    }

    /// Feed a new frequency error sample and return the current lock status.
    ///
    /// Internally updates the EWMA mean and variance, then applies
    /// lock/unlock logic with hysteresis and hold counting.
    pub fn update(&mut self, freq_error: f64) -> bool {
        // Update EWMA of the error.
        self.avg_freq_error =
            self.alpha * freq_error + (1.0 - self.alpha) * self.avg_freq_error;

        // Update EWMA of the variance (squared deviation from the mean).
        let deviation = freq_error - self.avg_freq_error;
        let inst_var = deviation * deviation;
        self.avg_freq_error_var =
            self.alpha * inst_var + (1.0 - self.alpha) * self.avg_freq_error_var;

        if self.locked {
            // Already locked -- check for unlock.
            if self.avg_freq_error_var > self.unlock_threshold {
                self.hold_count = 0;
                self.locked = false;
            }
        } else {
            // Not locked -- check for lock.
            if self.avg_freq_error_var < self.lock_threshold {
                self.hold_count += 1;
                if self.hold_count >= self.hold_threshold {
                    self.locked = true;
                }
            } else {
                self.hold_count = 0;
            }
        }

        self.locked
    }

    /// Returns `true` if the detector currently considers the loop locked.
    pub fn is_locked(&self) -> bool {
        self.locked
    }

    /// Lock confidence in `[0.0, 1.0]`.
    ///
    /// Returns 1.0 when the error variance is zero and approaches 0.0 as
    /// the variance grows beyond the lock threshold.  The mapping is
    /// `1 - clamp(variance / lock_threshold, 0, 1)`.
    pub fn confidence(&self) -> f64 {
        (1.0 - (self.avg_freq_error_var / self.lock_threshold)).clamp(0.0, 1.0)
    }

    /// Reset all internal state (unlocked, zero averages).
    pub fn reset(&mut self) {
        self.avg_freq_error = 0.0;
        self.avg_freq_error_var = 0.0;
        self.locked = false;
        self.hold_count = 0;
    }

    /// Override the lock and unlock thresholds.
    pub fn set_thresholds(&mut self, lock: f64, unlock: f64) {
        self.lock_threshold = lock;
        self.unlock_threshold = unlock;
    }
}

// ---------------------------------------------------------------------------
// PhaseCoherenceDetector
// ---------------------------------------------------------------------------

/// Phase-coherence lock detector.
///
/// Maintains a sliding window of phase samples and computes the circular
/// variance.  A coherent (locked) carrier produces low phase variance.
#[derive(Debug, Clone)]
pub struct PhaseCoherenceDetector {
    /// Ring buffer of recent phase samples.
    window: Vec<f64>,
    /// Write index into `window`.
    index: usize,
    /// Whether the window has been completely filled at least once.
    filled: bool,
    /// Coherence threshold -- lock is declared when `coherence() >= threshold`.
    threshold: f64,
}

impl PhaseCoherenceDetector {
    /// Create a new phase coherence detector.
    ///
    /// * `window_size` -- number of phase samples in the sliding window.
    /// * `threshold` -- coherence value (0..1) above which lock is declared.
    pub fn new(window_size: usize, threshold: f64) -> Self {
        Self {
            window: vec![0.0; window_size],
            index: 0,
            filled: false,
            threshold,
        }
    }

    /// Push a new phase sample (radians) and return the current lock status.
    pub fn update(&mut self, phase: f64) -> bool {
        self.window[self.index] = phase;
        self.index += 1;
        if self.index >= self.window.len() {
            self.index = 0;
            self.filled = true;
        }
        self.coherence() >= self.threshold
    }

    /// Circular coherence in `[0.0, 1.0]`.
    ///
    /// Computed as the magnitude of the mean unit-phase vector:
    ///   `R = |mean(exp(j * phase_k))|`
    ///
    /// * 1.0 -- all phases identical (perfectly coherent / locked).
    /// * 0.0 -- phases uniformly distributed (incoherent / unlocked).
    pub fn coherence(&self) -> f64 {
        let n = if self.filled {
            self.window.len()
        } else {
            self.index
        };
        if n == 0 {
            return 0.0;
        }

        let (sum_sin, sum_cos) = self.window[..n]
            .iter()
            .fold((0.0_f64, 0.0_f64), |(s, c), &ph| {
                (s + ph.sin(), c + ph.cos())
            });

        let mean_sin = sum_sin / n as f64;
        let mean_cos = sum_cos / n as f64;
        (mean_sin * mean_sin + mean_cos * mean_cos).sqrt()
    }

    /// Reset the detector (empty the window).
    pub fn reset(&mut self) {
        self.window.iter_mut().for_each(|v| *v = 0.0);
        self.index = 0;
        self.filled = false;
    }
}

// ---------------------------------------------------------------------------
// CombinedLockDetector
// ---------------------------------------------------------------------------

/// Combined frequency-error and phase-coherence lock detector.
///
/// Declares lock only when **both** the [`FreqLockDetector`] and the
/// [`PhaseCoherenceDetector`] agree that the loop is locked.
#[derive(Debug, Clone)]
pub struct CombinedLockDetector {
    /// Frequency error component.
    pub freq: FreqLockDetector,
    /// Phase coherence component.
    pub phase: PhaseCoherenceDetector,
}

impl CombinedLockDetector {
    /// Create a combined detector.
    ///
    /// * `freq_alpha` -- EWMA smoothing factor for the frequency detector.
    /// * `freq_threshold` -- lock threshold for the frequency detector.
    /// * `phase_window` -- sliding window size for the phase detector.
    /// * `phase_threshold` -- coherence threshold for the phase detector.
    pub fn new(
        freq_alpha: f64,
        freq_threshold: f64,
        phase_window: usize,
        phase_threshold: f64,
    ) -> Self {
        Self {
            freq: FreqLockDetector::new(freq_alpha, freq_threshold),
            phase: PhaseCoherenceDetector::new(phase_window, phase_threshold),
        }
    }

    /// Feed new frequency error and phase samples; returns `true` when both
    /// sub-detectors indicate lock.
    pub fn update(&mut self, freq_error: f64, phase: f64) -> bool {
        let freq_locked = self.freq.update(freq_error);
        let phase_locked = self.phase.update(phase);
        freq_locked && phase_locked
    }

    /// Reset both sub-detectors.
    pub fn reset(&mut self) {
        self.freq.reset();
        self.phase.reset();
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_lock_on_small_error() {
        let mut det = FreqLockDetector::new(0.1, 0.05);
        // Feed consistently small errors -- should eventually lock.
        for _ in 0..200 {
            det.update(0.001);
        }
        assert!(det.is_locked(), "detector should lock on small errors");
    }

    #[test]
    fn test_unlock_on_large_error() {
        let mut det = FreqLockDetector::new(0.1, 0.05);
        // Lock first.
        for _ in 0..200 {
            det.update(0.001);
        }
        assert!(det.is_locked());

        // Now feed large *varying* errors -- alternating sign ensures high
        // variance around a near-zero mean.
        for i in 0..200 {
            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            det.update(sign * 5.0);
        }
        assert!(!det.is_locked(), "detector should unlock on large errors");
    }

    #[test]
    fn test_hysteresis() {
        let mut det = FreqLockDetector::new(0.2, 0.01);
        // unlock_threshold = 0.02 by default (2x lock_threshold).

        // Lock with tiny errors.
        for _ in 0..200 {
            det.update(0.0001);
        }
        assert!(det.is_locked());

        // Feed moderate errors that put variance between lock and unlock
        // thresholds.  Should stay locked due to hysteresis.
        // An error that produces variance ~0.015 (between 0.01 and 0.02).
        // With alpha=0.2 and already-low variance, a moderate bump
        // should not immediately break the unlock threshold.
        for _ in 0..5 {
            det.update(0.08);
        }
        // We should still be locked because the variance has not exceeded
        // the unlock threshold (0.02) after only a few moderate samples.
        assert!(
            det.is_locked(),
            "hysteresis should prevent unlock on moderate errors"
        );
    }

    #[test]
    fn test_confidence() {
        let mut det = FreqLockDetector::new(0.1, 1.0);
        // Initially variance is 0 -> confidence = 1.0.
        assert!((det.confidence() - 1.0).abs() < 1e-9);

        // After feeding large alternating errors, variance grows and
        // confidence should drop.
        for i in 0..200 {
            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            det.update(sign * 5.0);
        }
        assert!(
            det.confidence() < 0.5,
            "confidence should be low after large errors, got {}",
            det.confidence()
        );
    }

    #[test]
    fn test_reset() {
        let mut det = FreqLockDetector::new(0.1, 0.05);
        for _ in 0..200 {
            det.update(0.001);
        }
        assert!(det.is_locked());

        det.reset();
        assert!(!det.is_locked(), "reset should clear lock state");
        assert!(
            (det.confidence() - 1.0).abs() < 1e-9,
            "reset should zero variance (confidence=1)"
        );
    }

    #[test]
    fn test_phase_coherence_locked() {
        let mut det = PhaseCoherenceDetector::new(64, 0.9);
        // Feed identical phases -- perfectly coherent.
        for _ in 0..100 {
            det.update(0.5);
        }
        assert!(
            det.coherence() > 0.99,
            "identical phases should give coherence ~1.0, got {}",
            det.coherence()
        );
    }

    #[test]
    fn test_phase_coherence_unlocked() {
        let mut det = PhaseCoherenceDetector::new(64, 0.9);
        // Feed uniformly spaced phases around the circle -- incoherent.
        for i in 0..64 {
            let phase = 2.0 * PI * (i as f64) / 64.0;
            det.update(phase);
        }
        assert!(
            det.coherence() < 0.2,
            "uniformly spread phases should give low coherence, got {}",
            det.coherence()
        );
    }

    #[test]
    fn test_combined_detector() {
        let mut det = CombinedLockDetector::new(0.1, 0.05, 32, 0.8);

        // Feed small freq errors and coherent phases -- both should lock.
        for _ in 0..200 {
            det.update(0.001, 0.3);
        }
        assert!(
            det.freq.is_locked(),
            "freq sub-detector should be locked"
        );
        assert!(
            det.phase.coherence() > 0.9,
            "phase sub-detector should be coherent"
        );
        // Combined result should be locked.
        let status = det.update(0.001, 0.3);
        assert!(status, "combined detector should report locked");
    }

    #[test]
    fn test_hold_counter() {
        let mut det = FreqLockDetector::new(0.5, 1.0);
        // hold_threshold = 10 by default.
        // With alpha=0.5, variance converges quickly.

        // Feed zero errors so variance stays 0.
        // Should not lock until hold_count >= 10.
        for i in 0..9 {
            let locked = det.update(0.0);
            // Should not be locked until we reach hold_threshold.
            assert!(
                !locked,
                "should not be locked after only {} updates",
                i + 1
            );
        }
        // The 10th update should trigger lock.
        let locked = det.update(0.0);
        assert!(locked, "should lock on the 10th update (hold_threshold)");
    }

    #[test]
    fn test_empty_initial_state() {
        let det = FreqLockDetector::new(0.1, 0.05);
        assert!(!det.is_locked(), "should start unlocked");
        assert!(
            (det.avg_freq_error).abs() < 1e-15,
            "avg_freq_error should start at 0"
        );
        assert!(
            (det.avg_freq_error_var).abs() < 1e-15,
            "avg_freq_error_var should start at 0"
        );
        assert_eq!(det.hold_count, 0);

        let phase_det = PhaseCoherenceDetector::new(16, 0.9);
        assert!(
            phase_det.coherence() < 1e-15,
            "empty phase detector should have 0 coherence"
        );

        let combined = CombinedLockDetector::new(0.1, 0.05, 16, 0.9);
        assert!(!combined.freq.is_locked());
    }
}
