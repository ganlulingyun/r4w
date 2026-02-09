//! Peak Hold — Track and hold signal peaks with configurable decay
//!
//! Maintains a running peak (maximum) that decays toward the current
//! signal level at a configurable rate. Useful for spectrum display
//! peak-hold, signal level metering, and envelope following.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::peak_hold::PeakHold;
//!
//! let mut ph = PeakHold::new(0.01); // Slow decay
//! ph.process_sample(1.0);  // Peak = 1.0
//! ph.process_sample(0.5);  // Peak decays slightly from 1.0
//! assert!(ph.peak() > 0.99); // Still near 1.0
//! assert!(ph.peak() < 1.0);  // But slightly decayed
//! ```

/// Peak hold with exponential decay.
#[derive(Debug, Clone)]
pub struct PeakHold {
    /// Current peak value.
    peak: f64,
    /// Decay rate per sample (0 = infinite hold, 1 = no hold).
    decay: f64,
    /// Whether initialized.
    initialized: bool,
}

impl PeakHold {
    /// Create a peak hold block.
    ///
    /// `decay`: decay rate per sample. 0 = hold forever, 1 = follow input.
    /// Typical: 0.001 (slow decay) to 0.1 (fast decay).
    pub fn new(decay: f64) -> Self {
        Self {
            peak: 0.0,
            decay: decay.clamp(0.0, 1.0),
            initialized: false,
        }
    }

    /// Create with infinite hold (no decay).
    pub fn infinite_hold() -> Self {
        Self::new(0.0)
    }

    /// Process a single sample.
    #[inline]
    pub fn process_sample(&mut self, x: f64) -> f64 {
        if !self.initialized {
            self.peak = x;
            self.initialized = true;
        } else if x >= self.peak {
            self.peak = x;
        } else {
            // Decay toward current value
            self.peak = self.peak - self.decay * (self.peak - x);
        }
        self.peak
    }

    /// Process a block of samples, returning peak-held output.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&x| self.process_sample(x)).collect()
    }

    /// Get current peak.
    pub fn peak(&self) -> f64 {
        self.peak
    }

    /// Get decay rate.
    pub fn decay(&self) -> f64 {
        self.decay
    }

    /// Set decay rate.
    pub fn set_decay(&mut self, decay: f64) {
        self.decay = decay.clamp(0.0, 1.0);
    }

    /// Reset peak to zero.
    pub fn reset(&mut self) {
        self.peak = 0.0;
        self.initialized = false;
    }
}

/// Peak hold for absolute values (tracks peak of |x|).
#[derive(Debug, Clone)]
pub struct AbsPeakHold {
    inner: PeakHold,
}

impl AbsPeakHold {
    /// Create an absolute peak hold.
    pub fn new(decay: f64) -> Self {
        Self {
            inner: PeakHold::new(decay),
        }
    }

    /// Process a sample (takes |x| first).
    pub fn process_sample(&mut self, x: f64) -> f64 {
        self.inner.process_sample(x.abs())
    }

    /// Process a block.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&x| self.process_sample(x)).collect()
    }

    /// Get current peak.
    pub fn peak(&self) -> f64 {
        self.inner.peak()
    }

    /// Reset.
    pub fn reset(&mut self) {
        self.inner.reset();
    }
}

/// Peak hold for dB values (tracks peak of 10*log10(|x|²)).
#[derive(Debug, Clone)]
pub struct PeakHoldDb {
    inner: PeakHold,
}

impl PeakHoldDb {
    /// Create a dB peak hold.
    pub fn new(decay: f64) -> Self {
        Self {
            inner: PeakHold::new(decay),
        }
    }

    /// Process a power value, returning peak-held dB.
    pub fn process_sample(&mut self, power: f64) -> f64 {
        let db = 10.0 * power.max(1e-30).log10();
        self.inner.process_sample(db)
    }

    /// Process a block.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&x| self.process_sample(x)).collect()
    }

    /// Get current peak in dB.
    pub fn peak_db(&self) -> f64 {
        self.inner.peak()
    }

    /// Reset.
    pub fn reset(&mut self) {
        self.inner.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_peak() {
        let mut ph = PeakHold::new(0.01);
        assert_eq!(ph.process_sample(0.5), 0.5);
        assert_eq!(ph.process_sample(1.0), 1.0); // New peak
    }

    #[test]
    fn test_decay() {
        let mut ph = PeakHold::new(0.1);
        ph.process_sample(1.0); // Peak = 1.0
        let v = ph.process_sample(0.0); // Decay from 1.0 toward 0.0
        assert!((v - 0.9).abs() < 1e-10); // 1.0 - 0.1*(1.0-0.0) = 0.9
    }

    #[test]
    fn test_infinite_hold() {
        let mut ph = PeakHold::infinite_hold();
        ph.process_sample(1.0);
        for _ in 0..1000 {
            ph.process_sample(0.0);
        }
        assert_eq!(ph.peak(), 1.0); // Never decays
    }

    #[test]
    fn test_no_hold() {
        let mut ph = PeakHold::new(1.0);
        ph.process_sample(1.0);
        let v = ph.process_sample(0.5);
        assert!((v - 0.5).abs() < 1e-10); // Immediately follows
    }

    #[test]
    fn test_block_process() {
        let mut ph = PeakHold::new(0.0); // Infinite hold
        let out = ph.process(&[0.5, 1.0, 0.3, 0.8]);
        assert_eq!(out, vec![0.5, 1.0, 1.0, 1.0]); // Peak stays at 1.0
    }

    #[test]
    fn test_reset() {
        let mut ph = PeakHold::new(0.01);
        ph.process_sample(1.0);
        ph.reset();
        assert_eq!(ph.peak(), 0.0);
    }

    #[test]
    fn test_set_decay() {
        let mut ph = PeakHold::new(0.5);
        ph.set_decay(0.1);
        assert!((ph.decay() - 0.1).abs() < 1e-10);
    }

    #[test]
    fn test_abs_peak_hold() {
        let mut ph = AbsPeakHold::new(0.0);
        ph.process_sample(-5.0); // |x| = 5.0
        ph.process_sample(3.0);  // |x| = 3.0 < 5.0
        assert_eq!(ph.peak(), 5.0);
    }

    #[test]
    fn test_abs_peak_block() {
        let mut ph = AbsPeakHold::new(0.0);
        let out = ph.process(&[-1.0, 2.0, -3.0, 1.0]);
        assert_eq!(out, vec![1.0, 2.0, 3.0, 3.0]);
    }

    #[test]
    fn test_peak_hold_db() {
        let mut ph = PeakHoldDb::new(0.0);
        ph.process_sample(1.0); // 0 dB
        ph.process_sample(0.1); // -10 dB, but peak holds at 0
        assert!(ph.peak_db().abs() < 0.01); // Still ~0 dB
    }

    #[test]
    fn test_decay_clamp() {
        let ph = PeakHold::new(2.0);
        assert_eq!(ph.decay(), 1.0); // Clamped to max
        let ph = PeakHold::new(-1.0);
        assert_eq!(ph.decay(), 0.0); // Clamped to min
    }

    #[test]
    fn test_empty() {
        let mut ph = PeakHold::new(0.1);
        assert!(ph.process(&[]).is_empty());
    }

    #[test]
    fn test_monotonic_during_hold() {
        let mut ph = PeakHold::new(0.01);
        ph.process_sample(1.0);
        let mut prev = 1.0;
        for _ in 0..100 {
            let v = ph.process_sample(0.0);
            assert!(v <= prev, "{} > {}", v, prev);
            prev = v;
        }
    }
}
