//! EVM Calculator — Error Vector Magnitude measurement
//!
//! Computes Error Vector Magnitude (EVM) between ideal reference
//! constellation points and received symbols. Reports RMS, peak,
//! and percentile EVM in both linear and dB. Essential for
//! transmitter quality assessment and modulation accuracy testing.
//! GNU Radio equivalent: `evm_cf` / probe functionality.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::evm_calculator::EvmCalculator;
//! use num_complex::Complex64;
//!
//! let mut evm = EvmCalculator::new();
//! let reference = vec![Complex64::new(1.0, 0.0), Complex64::new(-1.0, 0.0)];
//! let received = vec![Complex64::new(0.95, 0.05), Complex64::new(-1.1, -0.03)];
//! evm.update(&reference, &received);
//! assert!(evm.rms_evm_percent() < 20.0);
//! ```

use num_complex::Complex64;

/// Compute EVM (linear) for a single symbol pair.
///
/// `evm = |received - reference| / |reference|`
pub fn evm_single(reference: Complex64, received: Complex64) -> f64 {
    let ref_mag = reference.norm();
    if ref_mag < 1e-30 {
        return 0.0;
    }
    (received - reference).norm() / ref_mag
}

/// Compute RMS EVM for a block of symbols.
///
/// Returns EVM as a fraction (0.0 = perfect, 1.0 = 100%).
pub fn rms_evm(reference: &[Complex64], received: &[Complex64]) -> f64 {
    let n = reference.len().min(received.len());
    if n == 0 {
        return 0.0;
    }

    let ref_power: f64 = reference[..n].iter().map(|s| s.norm_sqr()).sum::<f64>() / n as f64;
    if ref_power < 1e-30 {
        return 0.0;
    }

    let error_power: f64 = reference[..n]
        .iter()
        .zip(received[..n].iter())
        .map(|(&r, &rx)| (rx - r).norm_sqr())
        .sum::<f64>()
        / n as f64;

    (error_power / ref_power).sqrt()
}

/// Compute peak EVM for a block of symbols.
///
/// Returns the maximum per-symbol EVM as a fraction.
pub fn peak_evm(reference: &[Complex64], received: &[Complex64]) -> f64 {
    let n = reference.len().min(received.len());
    if n == 0 {
        return 0.0;
    }

    let ref_rms = (reference[..n].iter().map(|s| s.norm_sqr()).sum::<f64>() / n as f64).sqrt();
    if ref_rms < 1e-30 {
        return 0.0;
    }

    reference[..n]
        .iter()
        .zip(received[..n].iter())
        .map(|(&r, &rx)| (rx - r).norm() / ref_rms)
        .fold(0.0f64, f64::max)
}

/// Convert linear EVM to dB: `20 * log10(evm)`.
pub fn evm_to_db(evm_linear: f64) -> f64 {
    20.0 * evm_linear.max(1e-30).log10()
}

/// Convert linear EVM to percent: `evm * 100`.
pub fn evm_to_percent(evm_linear: f64) -> f64 {
    evm_linear * 100.0
}

/// Streaming EVM calculator with history and statistics.
#[derive(Debug, Clone)]
pub struct EvmCalculator {
    /// Per-symbol EVM values (linear).
    history: Vec<f64>,
    /// Maximum history length.
    max_history: usize,
    /// Running sum of squared EVM.
    sum_sq_evm: f64,
    /// Running count of symbols.
    count: usize,
    /// Peak EVM seen.
    peak: f64,
    /// Reference power accumulator.
    ref_power_sum: f64,
    /// Error power accumulator.
    error_power_sum: f64,
}

impl EvmCalculator {
    /// Create a new EVM calculator.
    pub fn new() -> Self {
        Self {
            history: Vec::new(),
            max_history: 10_000,
            sum_sq_evm: 0.0,
            count: 0,
            peak: 0.0,
            ref_power_sum: 0.0,
            error_power_sum: 0.0,
        }
    }

    /// Create with a maximum history size.
    pub fn with_max_history(max_history: usize) -> Self {
        let mut calc = Self::new();
        calc.max_history = max_history;
        calc
    }

    /// Update with a block of reference/received symbol pairs.
    pub fn update(&mut self, reference: &[Complex64], received: &[Complex64]) {
        let n = reference.len().min(received.len());

        for i in 0..n {
            let error = (received[i] - reference[i]).norm_sqr();
            let ref_pwr = reference[i].norm_sqr();
            self.error_power_sum += error;
            self.ref_power_sum += ref_pwr;

            // Per-symbol EVM
            let sym_evm = if ref_pwr > 1e-30 {
                (error / ref_pwr).sqrt()
            } else {
                0.0
            };

            self.sum_sq_evm += sym_evm * sym_evm;
            self.count += 1;
            if sym_evm > self.peak {
                self.peak = sym_evm;
            }

            // History ring buffer
            if self.history.len() >= self.max_history {
                self.history.remove(0);
            }
            self.history.push(sym_evm);
        }
    }

    /// RMS EVM as a fraction (power-normalized).
    pub fn rms_evm(&self) -> f64 {
        if self.ref_power_sum < 1e-30 || self.count == 0 {
            return 0.0;
        }
        let avg_error = self.error_power_sum / self.count as f64;
        let avg_ref = self.ref_power_sum / self.count as f64;
        (avg_error / avg_ref).sqrt()
    }

    /// RMS EVM in percent.
    pub fn rms_evm_percent(&self) -> f64 {
        evm_to_percent(self.rms_evm())
    }

    /// RMS EVM in dB.
    pub fn rms_evm_db(&self) -> f64 {
        evm_to_db(self.rms_evm())
    }

    /// Peak EVM as a fraction.
    pub fn peak_evm(&self) -> f64 {
        self.peak
    }

    /// Peak EVM in percent.
    pub fn peak_evm_percent(&self) -> f64 {
        evm_to_percent(self.peak)
    }

    /// Peak EVM in dB.
    pub fn peak_evm_db(&self) -> f64 {
        evm_to_db(self.peak)
    }

    /// Number of symbols processed.
    pub fn count(&self) -> usize {
        self.count
    }

    /// Get the EVM history.
    pub fn history(&self) -> &[f64] {
        &self.history
    }

    /// Compute percentile EVM from history.
    ///
    /// `percentile` should be 0.0 to 100.0 (e.g., 95.0 for 95th percentile).
    pub fn percentile_evm(&self, percentile: f64) -> f64 {
        if self.history.is_empty() {
            return 0.0;
        }
        let mut sorted = self.history.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let idx = ((percentile / 100.0) * (sorted.len() - 1) as f64).round() as usize;
        let idx = idx.min(sorted.len() - 1);
        sorted[idx]
    }

    /// Reset all statistics.
    pub fn reset(&mut self) {
        self.history.clear();
        self.sum_sq_evm = 0.0;
        self.count = 0;
        self.peak = 0.0;
        self.ref_power_sum = 0.0;
        self.error_power_sum = 0.0;
    }
}

impl Default for EvmCalculator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_perfect_symbols() {
        let symbols = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(-1.0, 0.0),
            Complex64::new(0.0, 1.0),
        ];
        let evm = rms_evm(&symbols, &symbols);
        assert!(evm < 1e-10, "Perfect symbols should have 0 EVM");
    }

    #[test]
    fn test_rms_evm_nonzero() {
        let reference = vec![Complex64::new(1.0, 0.0), Complex64::new(-1.0, 0.0)];
        let received = vec![Complex64::new(0.9, 0.1), Complex64::new(-1.1, -0.05)];
        let evm = rms_evm(&reference, &received);
        assert!(evm > 0.0);
        assert!(evm < 1.0);
    }

    #[test]
    fn test_peak_evm() {
        let reference = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(-1.0, 0.0),
            Complex64::new(0.0, 1.0),
        ];
        let received = vec![
            Complex64::new(1.0, 0.0),   // Perfect
            Complex64::new(-0.5, 0.0),  // Large error
            Complex64::new(0.0, 1.0),   // Perfect
        ];
        let peak = peak_evm(&reference, &received);
        assert!(peak > 0.0);
    }

    #[test]
    fn test_evm_single() {
        let evm = evm_single(Complex64::new(1.0, 0.0), Complex64::new(1.0, 0.0));
        assert!(evm < 1e-10);

        let evm = evm_single(Complex64::new(1.0, 0.0), Complex64::new(0.9, 0.0));
        assert!((evm - 0.1).abs() < 1e-10);
    }

    #[test]
    fn test_evm_to_db() {
        // EVM = 0.1 (10%) → -20 dB
        let db = evm_to_db(0.1);
        assert!((db - (-20.0)).abs() < 1e-6);

        // EVM = 1.0 (100%) → 0 dB
        let db = evm_to_db(1.0);
        assert!(db.abs() < 1e-6);
    }

    #[test]
    fn test_evm_to_percent() {
        assert!((evm_to_percent(0.05) - 5.0).abs() < 1e-10);
        assert!((evm_to_percent(0.1) - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_streaming_calculator() {
        let mut calc = EvmCalculator::new();
        let reference = vec![Complex64::new(1.0, 0.0), Complex64::new(-1.0, 0.0)];
        let received = vec![Complex64::new(0.95, 0.05), Complex64::new(-1.05, -0.02)];

        calc.update(&reference, &received);
        assert_eq!(calc.count(), 2);
        assert!(calc.rms_evm() > 0.0);
        assert!(calc.peak_evm() >= calc.rms_evm());
        assert!(calc.rms_evm_percent() > 0.0);
        assert!(calc.rms_evm_db() < 0.0); // Should be negative dB for small EVM
    }

    #[test]
    fn test_streaming_multiple_updates() {
        let mut calc = EvmCalculator::new();
        let reference = vec![Complex64::new(1.0, 0.0)];
        let received = vec![Complex64::new(0.9, 0.1)];

        for _ in 0..100 {
            calc.update(&reference, &received);
        }
        assert_eq!(calc.count(), 100);
        assert_eq!(calc.history().len(), 100);
    }

    #[test]
    fn test_percentile() {
        let mut calc = EvmCalculator::new();
        // Create varied EVM values
        for i in 0..100 {
            let noise = (i as f64) * 0.001;
            let reference = vec![Complex64::new(1.0, 0.0)];
            let received = vec![Complex64::new(1.0 - noise, noise)];
            calc.update(&reference, &received);
        }
        let p50 = calc.percentile_evm(50.0);
        let p95 = calc.percentile_evm(95.0);
        assert!(p95 >= p50, "95th percentile should be >= 50th");
    }

    #[test]
    fn test_max_history() {
        let mut calc = EvmCalculator::with_max_history(10);
        let reference = vec![Complex64::new(1.0, 0.0)];
        let received = vec![Complex64::new(0.9, 0.0)];

        for _ in 0..100 {
            calc.update(&reference, &received);
        }
        assert_eq!(calc.history().len(), 10);
        assert_eq!(calc.count(), 100); // Total count still tracks all
    }

    #[test]
    fn test_reset() {
        let mut calc = EvmCalculator::new();
        let reference = vec![Complex64::new(1.0, 0.0)];
        let received = vec![Complex64::new(0.9, 0.0)];
        calc.update(&reference, &received);
        assert!(calc.count() > 0);
        calc.reset();
        assert_eq!(calc.count(), 0);
        assert_eq!(calc.rms_evm(), 0.0);
        assert_eq!(calc.peak_evm(), 0.0);
    }

    #[test]
    fn test_empty_input() {
        assert_eq!(rms_evm(&[], &[]), 0.0);
        assert_eq!(peak_evm(&[], &[]), 0.0);
        let calc = EvmCalculator::new();
        assert_eq!(calc.rms_evm(), 0.0);
        assert_eq!(calc.percentile_evm(50.0), 0.0);
    }

    #[test]
    fn test_default() {
        let calc = EvmCalculator::default();
        assert_eq!(calc.count(), 0);
    }
}
