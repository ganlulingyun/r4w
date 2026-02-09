//! BER/PER Measurement Tool — Bit and Packet Error Rate analysis
//!
//! Automated error rate measurement with statistical convergence detection
//! and confidence intervals. Essential for system validation and comparison.
//! GNU Radio equivalent: `probe_ber_b`, `ber_bf`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ber_tool::{BerTester, BerConfig, PerTester};
//!
//! let mut ber = BerTester::new(BerConfig::default());
//! let tx = vec![true, false, true, true, false, true, false, false, true, true];
//! let rx = vec![true, false, true, false, false, true, false, true, true, true];
//! //                                 ^                         ^  -- 2 errors
//! ber.update(&tx, &rx);
//! assert_eq!(ber.error_bits(), 2);
//! assert!((ber.ber() - 0.2).abs() < 1e-10);
//! ```

/// BER measurement configuration.
#[derive(Debug, Clone)]
pub struct BerConfig {
    /// Minimum number of errors before declaring convergence.
    pub min_errors: u64,
    /// Minimum number of bits to process.
    pub min_bits: u64,
    /// Target confidence level (e.g., 0.95).
    pub target_confidence: f64,
}

impl Default for BerConfig {
    fn default() -> Self {
        Self {
            min_errors: 100,
            min_bits: 10_000,
            target_confidence: 0.95,
        }
    }
}

/// Bit Error Rate tester.
#[derive(Debug, Clone)]
pub struct BerTester {
    config: BerConfig,
    total_bits: u64,
    error_bits: u64,
    /// Sliding window for instantaneous BER (if enabled).
    window: Option<BerWindow>,
}

#[derive(Debug, Clone)]
struct BerWindow {
    errors: Vec<bool>,
    idx: usize,
    size: usize,
}

impl BerWindow {
    fn new(size: usize) -> Self {
        Self {
            errors: vec![false; size],
            idx: 0,
            size,
        }
    }

    fn push(&mut self, is_error: bool) {
        self.errors[self.idx] = is_error;
        self.idx = (self.idx + 1) % self.size;
    }

    fn ber(&self) -> f64 {
        let errs = self.errors.iter().filter(|&&e| e).count();
        errs as f64 / self.size as f64
    }
}

impl BerTester {
    /// Create a new BER tester.
    pub fn new(config: BerConfig) -> Self {
        Self {
            config,
            total_bits: 0,
            error_bits: 0,
            window: None,
        }
    }

    /// Create with a sliding window for instantaneous BER.
    pub fn with_window(config: BerConfig, window_size: usize) -> Self {
        Self {
            config,
            total_bits: 0,
            error_bits: 0,
            window: Some(BerWindow::new(window_size)),
        }
    }

    /// Update with transmitted and received bit vectors.
    pub fn update(&mut self, tx_bits: &[bool], rx_bits: &[bool]) {
        let len = tx_bits.len().min(rx_bits.len());
        for i in 0..len {
            let is_error = tx_bits[i] != rx_bits[i];
            if is_error {
                self.error_bits += 1;
            }
            if let Some(ref mut w) = self.window {
                w.push(is_error);
            }
        }
        self.total_bits += len as u64;
    }

    /// Update with byte vectors (compares all 8 bits per byte).
    pub fn update_bytes(&mut self, tx: &[u8], rx: &[u8]) {
        let len = tx.len().min(rx.len());
        for i in 0..len {
            let diff = tx[i] ^ rx[i];
            let errs = diff.count_ones() as u64;
            self.error_bits += errs;
            self.total_bits += 8;
            if let Some(ref mut w) = self.window {
                for bit in 0..8 {
                    w.push((diff >> bit) & 1 == 1);
                }
            }
        }
    }

    /// Get the overall BER.
    pub fn ber(&self) -> f64 {
        if self.total_bits == 0 {
            return 0.0;
        }
        self.error_bits as f64 / self.total_bits as f64
    }

    /// Get the windowed (instantaneous) BER.
    pub fn windowed_ber(&self) -> Option<f64> {
        self.window.as_ref().map(|w| w.ber())
    }

    /// Get total bits processed.
    pub fn total_bits(&self) -> u64 {
        self.total_bits
    }

    /// Get total error bits.
    pub fn error_bits(&self) -> u64 {
        self.error_bits
    }

    /// Check if measurement has converged.
    ///
    /// Converged when both min_errors and min_bits thresholds are met.
    pub fn has_converged(&self) -> bool {
        self.error_bits >= self.config.min_errors && self.total_bits >= self.config.min_bits
    }

    /// Compute confidence interval using normal approximation.
    ///
    /// Returns (lower, upper) BER bounds for given confidence level.
    pub fn confidence_interval(&self, confidence: f64) -> (f64, f64) {
        if self.total_bits == 0 {
            return (0.0, 1.0);
        }
        let p = self.ber();
        let n = self.total_bits as f64;
        // z-score for confidence level
        let z = Self::z_score(confidence);
        let margin = z * (p * (1.0 - p) / n).sqrt();
        ((p - margin).max(0.0), (p + margin).min(1.0))
    }

    /// Normal distribution z-score approximation.
    fn z_score(confidence: f64) -> f64 {
        // Common values
        match () {
            _ if (confidence - 0.90).abs() < 0.001 => 1.645,
            _ if (confidence - 0.95).abs() < 0.001 => 1.960,
            _ if (confidence - 0.99).abs() < 0.001 => 2.576,
            _ => {
                // Rational approximation for probit function
                let p = (1.0 - confidence) / 2.0;
                let t = (-2.0 * p.ln()).sqrt();
                t - (2.515517 + 0.802853 * t + 0.010328 * t * t) /
                    (1.0 + 1.432788 * t + 0.189269 * t * t + 0.001308 * t * t * t)
            }
        }
    }

    /// Reset all counters.
    pub fn reset(&mut self) {
        self.total_bits = 0;
        self.error_bits = 0;
        if let Some(ref mut w) = self.window {
            w.errors.fill(false);
            w.idx = 0;
        }
    }

    /// Get a summary string.
    pub fn summary(&self) -> String {
        let (lo, hi) = self.confidence_interval(self.config.target_confidence);
        format!(
            "BER: {:.6} ({} errors / {} bits) [{:.6}, {:.6}] {}% CI{}",
            self.ber(),
            self.error_bits,
            self.total_bits,
            lo, hi,
            (self.config.target_confidence * 100.0) as u32,
            if self.has_converged() { " [CONVERGED]" } else { "" },
        )
    }
}

/// Packet Error Rate tester.
#[derive(Debug, Clone)]
pub struct PerTester {
    total_packets: u64,
    failed_packets: u64,
}

impl PerTester {
    /// Create a new PER tester.
    pub fn new() -> Self {
        Self {
            total_packets: 0,
            failed_packets: 0,
        }
    }

    /// Update with a single packet result (tx payload vs rx payload).
    pub fn update_packet(&mut self, tx: &[u8], rx: &[u8]) {
        self.total_packets += 1;
        if tx != rx {
            self.failed_packets += 1;
        }
    }

    /// Record a packet result directly.
    pub fn record(&mut self, success: bool) {
        self.total_packets += 1;
        if !success {
            self.failed_packets += 1;
        }
    }

    /// Get the PER.
    pub fn per(&self) -> f64 {
        if self.total_packets == 0 {
            return 0.0;
        }
        self.failed_packets as f64 / self.total_packets as f64
    }

    /// Get total packets tested.
    pub fn total_packets(&self) -> u64 {
        self.total_packets
    }

    /// Get number of failed packets.
    pub fn failed_packets(&self) -> u64 {
        self.failed_packets
    }

    /// Reset counters.
    pub fn reset(&mut self) {
        self.total_packets = 0;
        self.failed_packets = 0;
    }

    /// Get a summary string.
    pub fn summary(&self) -> String {
        format!(
            "PER: {:.6} ({} failed / {} total)",
            self.per(),
            self.failed_packets,
            self.total_packets,
        )
    }
}

impl Default for PerTester {
    fn default() -> Self {
        Self::new()
    }
}

/// BER curve data point.
#[derive(Debug, Clone, Copy)]
pub struct BerPoint {
    /// SNR in dB.
    pub snr_db: f64,
    /// Measured BER.
    pub ber: f64,
    /// Number of bits tested.
    pub bits_tested: u64,
    /// Number of errors.
    pub errors: u64,
}

/// BER vs SNR curve sweeper.
#[derive(Debug, Clone)]
pub struct BerCurveSweep {
    points: Vec<BerPoint>,
}

impl BerCurveSweep {
    /// Create a new BER curve.
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// Add a measurement point.
    pub fn add_point(&mut self, snr_db: f64, ber: f64, bits: u64, errors: u64) {
        self.points.push(BerPoint { snr_db, ber, bits_tested: bits, errors });
    }

    /// Get all points.
    pub fn points(&self) -> &[BerPoint] {
        &self.points
    }

    /// Theoretical BPSK BER: Q(√(2·Eb/N0)).
    pub fn theoretical_bpsk(snr_range: &[f64]) -> Vec<(f64, f64)> {
        snr_range.iter().map(|&snr| {
            let eb_n0 = 10f64.powf(snr / 10.0);
            let ber = 0.5 * erfc((eb_n0).sqrt());
            (snr, ber)
        }).collect()
    }

    /// Theoretical QPSK BER (same as BPSK per bit).
    pub fn theoretical_qpsk(snr_range: &[f64]) -> Vec<(f64, f64)> {
        Self::theoretical_bpsk(snr_range)
    }

    /// Theoretical 16-QAM BER approximation.
    pub fn theoretical_16qam(snr_range: &[f64]) -> Vec<(f64, f64)> {
        snr_range.iter().map(|&snr| {
            let eb_n0 = 10f64.powf(snr / 10.0);
            let ber = 0.75 * erfc((0.4 * eb_n0).sqrt());
            (snr, ber)
        }).collect()
    }

    /// Export to CSV format.
    pub fn to_csv(&self) -> String {
        let mut csv = String::from("snr_db,ber,bits,errors\n");
        for p in &self.points {
            csv.push_str(&format!("{:.2},{:.10},{},{}\n", p.snr_db, p.ber, p.bits_tested, p.errors));
        }
        csv
    }
}

impl Default for BerCurveSweep {
    fn default() -> Self {
        Self::new()
    }
}

/// Complementary error function approximation.
fn erfc(x: f64) -> f64 {
    // Abramowitz & Stegun approximation 7.1.26
    let t = 1.0 / (1.0 + 0.3275911 * x.abs());
    let poly = t * (0.254829592 + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    let result = poly * (-x * x).exp();
    if x >= 0.0 { result } else { 2.0 - result }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zero_ber() {
        let mut ber = BerTester::new(BerConfig::default());
        let bits = vec![true, false, true, false, true];
        ber.update(&bits, &bits);
        assert_eq!(ber.ber(), 0.0);
        assert_eq!(ber.error_bits(), 0);
        assert_eq!(ber.total_bits(), 5);
    }

    #[test]
    fn test_all_errors() {
        let mut ber = BerTester::new(BerConfig::default());
        let tx = vec![true, true, true, true];
        let rx = vec![false, false, false, false];
        ber.update(&tx, &rx);
        assert_eq!(ber.ber(), 1.0);
        assert_eq!(ber.error_bits(), 4);
    }

    #[test]
    fn test_partial_errors() {
        let mut ber = BerTester::new(BerConfig::default());
        let tx = vec![true, false, true, true, false, true, false, false, true, true];
        let rx = vec![true, false, true, false, false, true, false, true, true, true];
        ber.update(&tx, &rx);
        assert_eq!(ber.error_bits(), 2);
        assert!((ber.ber() - 0.2).abs() < 1e-10);
    }

    #[test]
    fn test_update_bytes() {
        let mut ber = BerTester::new(BerConfig::default());
        ber.update_bytes(&[0xFF], &[0xFE]); // 1 bit error (LSB)
        assert_eq!(ber.total_bits(), 8);
        assert_eq!(ber.error_bits(), 1);
    }

    #[test]
    fn test_update_bytes_all_different() {
        let mut ber = BerTester::new(BerConfig::default());
        ber.update_bytes(&[0x00], &[0xFF]);
        assert_eq!(ber.error_bits(), 8);
        assert_eq!(ber.ber(), 1.0);
    }

    #[test]
    fn test_convergence() {
        let mut ber = BerTester::new(BerConfig {
            min_errors: 5,
            min_bits: 20,
            target_confidence: 0.95,
        });
        assert!(!ber.has_converged());

        let tx = vec![true; 100];
        let rx = vec![false; 100];
        ber.update(&tx, &rx);
        assert!(ber.has_converged());
    }

    #[test]
    fn test_confidence_interval() {
        let mut ber = BerTester::new(BerConfig::default());
        // 10 errors in 100 bits = BER 0.1
        let mut tx = vec![true; 100];
        let mut rx = tx.clone();
        for i in 0..10 {
            rx[i] = !tx[i];
        }
        ber.update(&tx, &rx);
        let (lo, hi) = ber.confidence_interval(0.95);
        assert!(lo < 0.1);
        assert!(hi > 0.1);
        assert!(lo > 0.0);
        assert!(hi < 0.5);
    }

    #[test]
    fn test_confidence_empty() {
        let ber = BerTester::new(BerConfig::default());
        let (lo, hi) = ber.confidence_interval(0.95);
        assert_eq!(lo, 0.0);
        assert_eq!(hi, 1.0);
    }

    #[test]
    fn test_reset() {
        let mut ber = BerTester::new(BerConfig::default());
        ber.update(&[true; 100], &[false; 100]);
        ber.reset();
        assert_eq!(ber.total_bits(), 0);
        assert_eq!(ber.error_bits(), 0);
        assert_eq!(ber.ber(), 0.0);
    }

    #[test]
    fn test_windowed_ber() {
        let mut ber = BerTester::with_window(BerConfig::default(), 10);
        let tx = vec![true; 10];
        let mut rx = tx.clone();
        rx[0] = false; // 1 error in 10
        ber.update(&tx, &rx);
        let w = ber.windowed_ber().unwrap();
        assert!((w - 0.1).abs() < 1e-10);
    }

    #[test]
    fn test_summary() {
        let mut ber = BerTester::new(BerConfig::default());
        ber.update(&[true; 10], &[false; 10]);
        let s = ber.summary();
        assert!(s.contains("BER:"));
        assert!(s.contains("10 errors"));
    }

    #[test]
    fn test_per_zero() {
        let mut per = PerTester::new();
        per.update_packet(&[1, 2, 3], &[1, 2, 3]);
        assert_eq!(per.per(), 0.0);
        assert_eq!(per.total_packets(), 1);
        assert_eq!(per.failed_packets(), 0);
    }

    #[test]
    fn test_per_failure() {
        let mut per = PerTester::new();
        per.update_packet(&[1, 2, 3], &[1, 2, 4]);
        assert_eq!(per.per(), 1.0);
        assert_eq!(per.failed_packets(), 1);
    }

    #[test]
    fn test_per_mixed() {
        let mut per = PerTester::new();
        per.record(true);
        per.record(true);
        per.record(false);
        per.record(true);
        assert_eq!(per.total_packets(), 4);
        assert_eq!(per.failed_packets(), 1);
        assert!((per.per() - 0.25).abs() < 1e-10);
    }

    #[test]
    fn test_per_reset() {
        let mut per = PerTester::new();
        per.record(false);
        per.reset();
        assert_eq!(per.total_packets(), 0);
    }

    #[test]
    fn test_per_summary() {
        let mut per = PerTester::new();
        per.record(false);
        per.record(true);
        let s = per.summary();
        assert!(s.contains("PER:"));
    }

    #[test]
    fn test_erfc() {
        // erfc(0) = 1.0
        assert!((erfc(0.0) - 1.0).abs() < 1e-6);
        // erfc(large) ≈ 0
        assert!(erfc(5.0) < 1e-10);
        // erfc(-large) ≈ 2
        assert!((erfc(-5.0) - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_theoretical_bpsk() {
        let snrs = vec![0.0, 5.0, 10.0, 15.0, 20.0];
        let curve = BerCurveSweep::theoretical_bpsk(&snrs);
        assert_eq!(curve.len(), 5);
        // BER should decrease with SNR
        for i in 1..curve.len() {
            assert!(curve[i].1 < curve[i - 1].1);
        }
        // At 0 dB, BPSK BER ≈ 0.0786
        assert!((curve[0].1 - 0.0786).abs() < 0.01);
    }

    #[test]
    fn test_theoretical_16qam() {
        let snrs = vec![5.0, 10.0, 15.0, 20.0];
        let curve = BerCurveSweep::theoretical_16qam(&snrs);
        // BER should decrease with SNR
        for i in 1..curve.len() {
            assert!(curve[i].1 < curve[i - 1].1);
        }
    }

    #[test]
    fn test_curve_csv() {
        let mut sweep = BerCurveSweep::new();
        sweep.add_point(0.0, 0.1, 1000, 100);
        sweep.add_point(5.0, 0.01, 10000, 100);
        let csv = sweep.to_csv();
        assert!(csv.contains("snr_db,ber,bits,errors"));
        assert!(csv.contains("0.00"));
        assert!(csv.contains("5.00"));
    }

    #[test]
    fn test_ber_curve_points() {
        let mut sweep = BerCurveSweep::new();
        sweep.add_point(0.0, 0.1, 1000, 100);
        assert_eq!(sweep.points().len(), 1);
        assert_eq!(sweep.points()[0].snr_db, 0.0);
    }

    #[test]
    fn test_incremental_updates() {
        let mut ber = BerTester::new(BerConfig::default());
        ber.update(&[true; 50], &[false; 50]); // 50 errors
        ber.update(&[true; 50], &[true; 50]);   // 0 errors
        assert_eq!(ber.total_bits(), 100);
        assert_eq!(ber.error_bits(), 50);
        assert!((ber.ber() - 0.5).abs() < 1e-10);
    }
}
