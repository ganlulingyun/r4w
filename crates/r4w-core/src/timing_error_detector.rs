//! Timing Error Detector — Symbol timing error estimation
//!
//! Standalone timing error detectors for clock recovery: Gardner,
//! Mueller-Müller, Early-Late Gate, and Zero-Crossing. Decoupled
//! from the PLL loop filter for flexible timing recovery architectures.
//! GNU Radio equivalent: `timing_error_detector_type`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::timing_error_detector::{gardner_ted, mueller_muller_ted};
//!
//! // Gardner TED: needs previous, mid, and current samples
//! let error = gardner_ted(1.0, 0.5, -1.0);
//! assert!(error.abs() > 0.0); // Non-zero timing error
//! ```

use num_complex::Complex64;

/// Gardner Timing Error Detector (for BPSK/QPSK).
///
/// `e = (prev - current) * mid`
///
/// Works on real-valued (decision-directed) samples.
/// - `prev`: previous symbol sample
/// - `mid`: midpoint sample (between prev and current)
/// - `current`: current symbol sample
pub fn gardner_ted(prev: f64, mid: f64, current: f64) -> f64 {
    (prev - current) * mid
}

/// Gardner TED for complex samples.
///
/// `e = Re{(prev - current) * conj(mid)}`
pub fn gardner_ted_complex(prev: Complex64, mid: Complex64, current: Complex64) -> f64 {
    ((prev - current) * mid.conj()).re
}

/// Mueller-Müller Timing Error Detector.
///
/// `e = prev_decision * current - current_decision * prev`
///
/// Decision-directed: uses hard decisions on symbol values.
/// - `prev`: previous sample
/// - `current`: current sample
/// - `prev_decision`: hard decision on previous symbol
/// - `current_decision`: hard decision on current symbol
pub fn mueller_muller_ted(
    prev: f64,
    current: f64,
    prev_decision: f64,
    current_decision: f64,
) -> f64 {
    prev_decision * current - current_decision * prev
}

/// Mueller-Müller TED for complex samples.
pub fn mueller_muller_ted_complex(
    prev: Complex64,
    current: Complex64,
    prev_decision: Complex64,
    current_decision: Complex64,
) -> f64 {
    (prev_decision * current.conj() - current_decision * prev.conj()).re
}

/// Early-Late Gate Timing Error Detector.
///
/// `e = |early|² - |late|²`
///
/// - `early`: sample taken slightly before optimal point
/// - `late`: sample taken slightly after optimal point
pub fn early_late_ted(early: f64, late: f64) -> f64 {
    early * early - late * late
}

/// Early-Late Gate TED for complex samples.
pub fn early_late_ted_complex(early: Complex64, late: Complex64) -> f64 {
    early.norm_sqr() - late.norm_sqr()
}

/// Zero-Crossing Timing Error Detector.
///
/// Detects transitions: `e = mid * sign(prev - current)`
///
/// Best for NRZ signals with frequent transitions.
pub fn zero_crossing_ted(prev: f64, mid: f64, current: f64) -> f64 {
    mid * (prev - current).signum()
}

/// Streaming timing error detector with configurable algorithm.
#[derive(Debug, Clone)]
pub struct TimingErrorDetector {
    /// Algorithm type.
    algorithm: TedAlgorithm,
    /// Previous symbol sample.
    prev_sample: f64,
    /// Previous hard decision.
    prev_decision: f64,
    /// Samples per symbol.
    sps: f64,
}

/// TED algorithm selection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TedAlgorithm {
    /// Gardner (non-decision-directed).
    Gardner,
    /// Mueller-Müller (decision-directed).
    MuellerMuller,
    /// Early-Late Gate.
    EarlyLate,
    /// Zero-Crossing.
    ZeroCrossing,
}

impl TimingErrorDetector {
    /// Create a new TED.
    pub fn new(algorithm: TedAlgorithm, sps: f64) -> Self {
        Self {
            algorithm,
            prev_sample: 0.0,
            prev_decision: 0.0,
            sps,
        }
    }

    /// Compute timing error for Gardner/Zero-Crossing.
    ///
    /// - `mid`: midpoint sample
    /// - `current`: current symbol sample
    pub fn compute(&mut self, mid: f64, current: f64) -> f64 {
        let error = match self.algorithm {
            TedAlgorithm::Gardner => gardner_ted(self.prev_sample, mid, current),
            TedAlgorithm::ZeroCrossing => zero_crossing_ted(self.prev_sample, mid, current),
            TedAlgorithm::EarlyLate => early_late_ted(mid, current), // mid=early, current=late
            TedAlgorithm::MuellerMuller => {
                let decision = if current >= 0.0 { 1.0 } else { -1.0 };
                let error =
                    mueller_muller_ted(self.prev_sample, current, self.prev_decision, decision);
                self.prev_decision = decision;
                error
            }
        };
        self.prev_sample = current;
        error
    }

    /// Get the algorithm type.
    pub fn algorithm(&self) -> TedAlgorithm {
        self.algorithm
    }

    /// Get samples per symbol.
    pub fn sps(&self) -> f64 {
        self.sps
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.prev_sample = 0.0;
        self.prev_decision = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gardner_basic() {
        // Transition from +1 to -1 with mid=0 → error = (1-(-1))*0 = 0
        assert!((gardner_ted(1.0, 0.0, -1.0) - 0.0).abs() < 1e-10);
        // Early timing: mid > 0 → positive error
        let e = gardner_ted(1.0, 0.5, -1.0);
        assert!(e > 0.0);
    }

    #[test]
    fn test_gardner_complex() {
        let prev = Complex64::new(1.0, 0.0);
        let mid = Complex64::new(0.5, 0.0);
        let current = Complex64::new(-1.0, 0.0);
        let e = gardner_ted_complex(prev, mid, current);
        assert!(e > 0.0);
    }

    #[test]
    fn test_mueller_muller() {
        let e = mueller_muller_ted(1.0, -0.8, 1.0, -1.0);
        // 1.0 * (-0.8) - (-1.0) * 1.0 = -0.8 + 1.0 = 0.2
        assert!((e - 0.2).abs() < 1e-10);
    }

    #[test]
    fn test_mueller_muller_complex() {
        let prev = Complex64::new(0.9, 0.0);
        let current = Complex64::new(-0.8, 0.0);
        let prev_dec = Complex64::new(1.0, 0.0);
        let curr_dec = Complex64::new(-1.0, 0.0);
        let e = mueller_muller_ted_complex(prev, current, prev_dec, curr_dec);
        assert!(e.abs() > 0.0);
    }

    #[test]
    fn test_early_late() {
        // Early stronger than late → positive error (early timing)
        let e = early_late_ted(0.9, 0.7);
        assert!(e > 0.0);
        // Late stronger → negative error
        let e = early_late_ted(0.5, 0.8);
        assert!(e < 0.0);
    }

    #[test]
    fn test_early_late_complex() {
        let early = Complex64::new(0.9, 0.0);
        let late = Complex64::new(0.7, 0.0);
        assert!(early_late_ted_complex(early, late) > 0.0);
    }

    #[test]
    fn test_zero_crossing() {
        // Positive-to-negative transition, mid at zero → error = 0
        let e = zero_crossing_ted(1.0, 0.0, -1.0);
        assert!((e - 0.0).abs() < 1e-10);
        // Mid offset → non-zero error
        let e = zero_crossing_ted(1.0, 0.3, -1.0);
        assert!(e > 0.0);
    }

    #[test]
    fn test_streaming_gardner() {
        let mut ted = TimingErrorDetector::new(TedAlgorithm::Gardner, 4.0);
        ted.prev_sample = 1.0;
        let e = ted.compute(0.5, -1.0);
        assert!(e > 0.0);
    }

    #[test]
    fn test_streaming_mm() {
        let mut ted = TimingErrorDetector::new(TedAlgorithm::MuellerMuller, 4.0);
        ted.prev_sample = 0.9;
        ted.prev_decision = 1.0;
        let e = ted.compute(0.0, -0.8);
        assert!(e.abs() > 0.0);
    }

    #[test]
    fn test_reset() {
        let mut ted = TimingErrorDetector::new(TedAlgorithm::Gardner, 2.0);
        ted.prev_sample = 5.0;
        ted.reset();
        assert_eq!(ted.prev_sample, 0.0);
    }

    #[test]
    fn test_accessors() {
        let ted = TimingErrorDetector::new(TedAlgorithm::EarlyLate, 4.0);
        assert_eq!(ted.algorithm(), TedAlgorithm::EarlyLate);
        assert_eq!(ted.sps(), 4.0);
    }

    #[test]
    fn test_no_transition() {
        // No transition → zero error for Gardner
        assert!((gardner_ted(1.0, 1.0, 1.0) - 0.0).abs() < 1e-10);
    }
}
