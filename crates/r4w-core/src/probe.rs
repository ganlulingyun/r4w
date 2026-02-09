//! Probe / Measurement Blocks
//!
//! Non-intrusive measurement blocks for monitoring signal quality
//! in processing pipelines. All probes pass through data unchanged
//! while computing statistics.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::probe::{PowerProbe, EvmProbe};
//! use num_complex::Complex64;
//!
//! let mut power_probe = PowerProbe::new();
//! let samples = vec![Complex64::new(1.0, 0.0); 100];
//! power_probe.update(&samples);
//! assert!((power_probe.power_db() - 0.0).abs() < 0.1);
//! ```

use num_complex::Complex64;

/// Power measurement probe.
///
/// Computes average, peak, and PAPR of the signal.
#[derive(Debug, Clone)]
pub struct PowerProbe {
    /// Running sum of power.
    sum_power: f64,
    /// Peak instantaneous power.
    peak_power: f64,
    /// Number of samples processed.
    count: u64,
}

impl PowerProbe {
    /// Create a new power probe.
    pub fn new() -> Self {
        Self {
            sum_power: 0.0,
            peak_power: 0.0,
            count: 0,
        }
    }

    /// Update with new samples.
    pub fn update(&mut self, samples: &[Complex64]) {
        for s in samples {
            let p = s.norm_sqr();
            self.sum_power += p;
            if p > self.peak_power {
                self.peak_power = p;
            }
            self.count += 1;
        }
    }

    /// Average power in linear scale.
    pub fn power_linear(&self) -> f64 {
        if self.count > 0 {
            self.sum_power / self.count as f64
        } else {
            0.0
        }
    }

    /// Average power in dB.
    pub fn power_db(&self) -> f64 {
        let p = self.power_linear();
        if p > 1e-30 {
            10.0 * p.log10()
        } else {
            -300.0
        }
    }

    /// Peak power in dB.
    pub fn peak_power_db(&self) -> f64 {
        if self.peak_power > 1e-30 {
            10.0 * self.peak_power.log10()
        } else {
            -300.0
        }
    }

    /// Peak-to-Average Power Ratio in dB.
    pub fn papr_db(&self) -> f64 {
        let avg = self.power_linear();
        if avg > 1e-30 && self.peak_power > 1e-30 {
            10.0 * (self.peak_power / avg).log10()
        } else {
            0.0
        }
    }

    /// Total number of samples processed.
    pub fn count(&self) -> u64 {
        self.count
    }

    /// Reset all measurements.
    pub fn reset(&mut self) {
        self.sum_power = 0.0;
        self.peak_power = 0.0;
        self.count = 0;
    }
}

impl Default for PowerProbe {
    fn default() -> Self {
        Self::new()
    }
}

/// EVM (Error Vector Magnitude) probe.
///
/// Measures the deviation of received symbols from ideal constellation points.
/// EVM is reported as a percentage of the reference constellation amplitude.
#[derive(Debug, Clone)]
pub struct EvmProbe {
    /// Reference constellation points.
    constellation: Vec<Complex64>,
    /// Sum of squared error magnitudes.
    sum_error_sq: f64,
    /// Sum of squared reference magnitudes.
    sum_ref_sq: f64,
    /// Number of symbols processed.
    count: u64,
    /// Maximum EVM seen.
    peak_evm: f64,
}

impl EvmProbe {
    /// Create an EVM probe for M-PSK constellation.
    pub fn psk(order: usize) -> Self {
        let constellation: Vec<Complex64> = (0..order)
            .map(|i| {
                let angle = 2.0 * std::f64::consts::PI * i as f64 / order as f64;
                Complex64::new(angle.cos(), angle.sin())
            })
            .collect();
        Self {
            constellation,
            sum_error_sq: 0.0,
            sum_ref_sq: 0.0,
            count: 0,
            peak_evm: 0.0,
        }
    }

    /// Create an EVM probe with a custom constellation.
    pub fn custom(constellation: Vec<Complex64>) -> Self {
        Self {
            constellation,
            sum_error_sq: 0.0,
            sum_ref_sq: 0.0,
            count: 0,
            peak_evm: 0.0,
        }
    }

    /// Update with received symbols.
    pub fn update(&mut self, symbols: &[Complex64]) {
        for s in symbols {
            // Find nearest constellation point
            let nearest = self
                .constellation
                .iter()
                .min_by(|a, b| {
                    (s - *a)
                        .norm_sqr()
                        .partial_cmp(&(s - *b).norm_sqr())
                        .unwrap()
                })
                .unwrap();

            let error = (s - nearest).norm_sqr();
            let ref_pwr = nearest.norm_sqr();

            self.sum_error_sq += error;
            self.sum_ref_sq += ref_pwr;
            self.count += 1;

            if ref_pwr > 1e-30 {
                let inst_evm = (error / ref_pwr).sqrt();
                if inst_evm > self.peak_evm {
                    self.peak_evm = inst_evm;
                }
            }
        }
    }

    /// RMS EVM as a ratio (0.0 = perfect, 1.0 = 100% error).
    pub fn evm_rms(&self) -> f64 {
        if self.sum_ref_sq > 1e-30 && self.count > 0 {
            (self.sum_error_sq / self.sum_ref_sq).sqrt()
        } else {
            0.0
        }
    }

    /// RMS EVM in percent.
    pub fn evm_percent(&self) -> f64 {
        self.evm_rms() * 100.0
    }

    /// RMS EVM in dB.
    pub fn evm_db(&self) -> f64 {
        let evm = self.evm_rms();
        if evm > 1e-30 {
            20.0 * evm.log10()
        } else {
            -100.0
        }
    }

    /// Peak EVM in percent.
    pub fn peak_evm_percent(&self) -> f64 {
        self.peak_evm * 100.0
    }

    /// Estimated SNR from EVM: SNR ≈ -20*log10(EVM_rms).
    pub fn estimated_snr_db(&self) -> f64 {
        -self.evm_db()
    }

    /// Reset all measurements.
    pub fn reset(&mut self) {
        self.sum_error_sq = 0.0;
        self.sum_ref_sq = 0.0;
        self.count = 0;
        self.peak_evm = 0.0;
    }
}

/// Constellation scatter probe.
///
/// Collects received constellation points for I/Q scatter plotting.
#[derive(Debug, Clone)]
pub struct ConstellationProbe {
    /// Collected points.
    points: Vec<Complex64>,
    /// Maximum number of points to keep.
    max_points: usize,
}

impl ConstellationProbe {
    /// Create a new constellation probe.
    pub fn new(max_points: usize) -> Self {
        Self {
            points: Vec::with_capacity(max_points),
            max_points,
        }
    }

    /// Add received symbols.
    pub fn update(&mut self, symbols: &[Complex64]) {
        for &s in symbols {
            if self.points.len() >= self.max_points {
                self.points.remove(0);
            }
            self.points.push(s);
        }
    }

    /// Get collected constellation points.
    pub fn points(&self) -> &[Complex64] {
        &self.points
    }

    /// Compute centroid (average position).
    pub fn centroid(&self) -> Complex64 {
        if self.points.is_empty() {
            return Complex64::new(0.0, 0.0);
        }
        let sum: Complex64 = self.points.iter().sum();
        sum / self.points.len() as f64
    }

    /// Compute I/Q imbalance metrics.
    pub fn iq_imbalance(&self) -> (f64, f64) {
        if self.points.is_empty() {
            return (0.0, 0.0);
        }
        let i_power: f64 = self.points.iter().map(|s| s.re * s.re).sum::<f64>()
            / self.points.len() as f64;
        let q_power: f64 = self.points.iter().map(|s| s.im * s.im).sum::<f64>()
            / self.points.len() as f64;

        let gain_imbalance_db = if q_power > 1e-30 {
            10.0 * (i_power / q_power).log10()
        } else {
            0.0
        };

        // Phase imbalance from I/Q correlation
        let iq_corr: f64 = self.points.iter().map(|s| s.re * s.im).sum::<f64>()
            / self.points.len() as f64;
        let phase_imbalance_deg = if i_power > 1e-30 && q_power > 1e-30 {
            (iq_corr / (i_power * q_power).sqrt()).asin().to_degrees()
        } else {
            0.0
        };

        (gain_imbalance_db, phase_imbalance_deg)
    }

    /// Reset collected points.
    pub fn reset(&mut self) {
        self.points.clear();
    }
}

/// Throughput / rate probe.
///
/// Measures samples per second for performance monitoring.
#[derive(Debug, Clone)]
pub struct RateProbe {
    /// Total samples counted.
    total_samples: u64,
    /// Samples in current window.
    window_samples: u64,
    /// Window start time (as sample count at some reference rate).
    window_start: u64,
    /// Window size in samples.
    window_size: u64,
    /// Last measured rate (samples/sec).
    last_rate: f64,
    /// Sample rate for time reference.
    sample_rate: f64,
}

impl RateProbe {
    /// Create a new rate probe.
    pub fn new(sample_rate: f64, window_seconds: f64) -> Self {
        Self {
            total_samples: 0,
            window_samples: 0,
            window_start: 0,
            window_size: (sample_rate * window_seconds) as u64,
            last_rate: 0.0,
            sample_rate,
        }
    }

    /// Count new samples.
    pub fn update(&mut self, num_samples: u64) {
        self.total_samples += num_samples;
        self.window_samples += num_samples;

        if self.window_samples >= self.window_size {
            self.last_rate = self.window_samples as f64
                / (self.window_size as f64 / self.sample_rate);
            self.window_samples = 0;
        }
    }

    /// Last measured throughput rate (samples/sec).
    pub fn rate(&self) -> f64 {
        self.last_rate
    }

    /// Total samples processed.
    pub fn total_samples(&self) -> u64 {
        self.total_samples
    }

    /// Reset measurements.
    pub fn reset(&mut self) {
        self.total_samples = 0;
        self.window_samples = 0;
        self.last_rate = 0.0;
    }
}

/// Frequency offset probe.
///
/// Estimates carrier frequency offset from received samples.
#[derive(Debug, Clone)]
pub struct FreqOffsetProbe {
    /// Previous sample for differential detection.
    prev: Complex64,
    /// Running sum of phase differences.
    sum_phase_diff: f64,
    /// Count of measurements.
    count: u64,
    /// Sample rate.
    sample_rate: f64,
}

impl FreqOffsetProbe {
    /// Create a new frequency offset probe.
    pub fn new(sample_rate: f64) -> Self {
        Self {
            prev: Complex64::new(1.0, 0.0),
            sum_phase_diff: 0.0,
            count: 0,
            sample_rate,
        }
    }

    /// Update with new samples.
    pub fn update(&mut self, samples: &[Complex64]) {
        for &s in samples {
            let diff = s * self.prev.conj();
            self.sum_phase_diff += diff.arg();
            self.prev = s;
            self.count += 1;
        }
    }

    /// Estimated frequency offset in Hz.
    pub fn offset_hz(&self) -> f64 {
        if self.count > 0 {
            let avg_phase_diff = self.sum_phase_diff / self.count as f64;
            avg_phase_diff * self.sample_rate / (2.0 * std::f64::consts::PI)
        } else {
            0.0
        }
    }

    /// Estimated frequency offset in PPM.
    pub fn offset_ppm(&self, carrier_freq: f64) -> f64 {
        if carrier_freq > 0.0 {
            self.offset_hz() / carrier_freq * 1e6
        } else {
            0.0
        }
    }

    /// Reset measurements.
    pub fn reset(&mut self) {
        self.prev = Complex64::new(1.0, 0.0);
        self.sum_phase_diff = 0.0;
        self.count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_power_probe_unit_signal() {
        let mut probe = PowerProbe::new();
        let samples = vec![Complex64::new(1.0, 0.0); 1000];
        probe.update(&samples);

        assert!((probe.power_db() - 0.0).abs() < 0.1, "Unit signal should be 0 dBFS");
        assert!((probe.papr_db() - 0.0).abs() < 0.1, "Constant signal should have 0 dB PAPR");
    }

    #[test]
    fn test_power_probe_half_power() {
        let mut probe = PowerProbe::new();
        let v = std::f64::consts::FRAC_1_SQRT_2;
        let samples = vec![Complex64::new(v, 0.0); 100];
        probe.update(&samples);

        assert!(
            (probe.power_db() - (-3.0)).abs() < 0.2,
            "Half-power signal should be ~-3 dBFS: got {:.1}",
            probe.power_db()
        );
    }

    #[test]
    fn test_power_probe_papr() {
        let mut probe = PowerProbe::new();
        // OFDM-like signal with high PAPR
        let mut samples = vec![Complex64::new(0.1, 0.0); 100];
        samples[50] = Complex64::new(3.0, 0.0); // One peak
        probe.update(&samples);

        assert!(
            probe.papr_db() > 10.0,
            "Signal with peak should have high PAPR: got {:.1}",
            probe.papr_db()
        );
    }

    #[test]
    fn test_evm_perfect_qpsk() {
        let mut probe = EvmProbe::psk(4);
        // Perfect QPSK symbols
        let symbols: Vec<Complex64> = (0..400)
            .map(|i| {
                let angle = 2.0 * std::f64::consts::PI * (i % 4) as f64 / 4.0;
                Complex64::new(angle.cos(), angle.sin())
            })
            .collect();
        probe.update(&symbols);

        assert!(
            probe.evm_percent() < 0.01,
            "Perfect QPSK should have ~0% EVM: got {:.3}%",
            probe.evm_percent()
        );
    }

    #[test]
    fn test_evm_noisy_qpsk() {
        let mut probe = EvmProbe::psk(4);
        let mut rng = 0x1234u64;
        let symbols: Vec<Complex64> = (0..1000)
            .map(|i| {
                let angle = 2.0 * std::f64::consts::PI * (i % 4) as f64 / 4.0;
                let ideal = Complex64::new(angle.cos(), angle.sin());
                rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
                let ni = (rng >> 11) as f64 / (1u64 << 53) as f64 * 0.2 - 0.1;
                rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
                let nq = (rng >> 11) as f64 / (1u64 << 53) as f64 * 0.2 - 0.1;
                ideal + Complex64::new(ni, nq)
            })
            .collect();
        probe.update(&symbols);

        assert!(
            probe.evm_percent() > 1.0 && probe.evm_percent() < 20.0,
            "Noisy QPSK should have moderate EVM: got {:.1}%",
            probe.evm_percent()
        );
    }

    #[test]
    fn test_constellation_probe() {
        let mut probe = ConstellationProbe::new(100);
        let symbols: Vec<Complex64> = (0..50)
            .map(|i| Complex64::new(i as f64 * 0.01, 0.0))
            .collect();
        probe.update(&symbols);

        assert_eq!(probe.points().len(), 50);
    }

    #[test]
    fn test_constellation_max_points() {
        let mut probe = ConstellationProbe::new(10);
        let symbols: Vec<Complex64> = (0..20)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        probe.update(&symbols);

        assert_eq!(probe.points().len(), 10);
        // Should keep the most recent 10
        assert!((probe.points()[0].re - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_freq_offset_probe() {
        let sr = 48000.0;
        let offset = 100.0; // 100 Hz offset
        let mut probe = FreqOffsetProbe::new(sr);

        // Generate tone at 100 Hz offset
        let samples: Vec<Complex64> = (0..4800)
            .map(|i| {
                let phase = 2.0 * std::f64::consts::PI * offset * i as f64 / sr;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();
        probe.update(&samples);

        assert!(
            (probe.offset_hz() - offset).abs() < 1.0,
            "Should estimate ~100 Hz offset: got {:.1}",
            probe.offset_hz()
        );
    }

    #[test]
    fn test_freq_offset_zero() {
        let mut probe = FreqOffsetProbe::new(48000.0);
        let samples = vec![Complex64::new(1.0, 0.0); 1000];
        probe.update(&samples);

        assert!(
            probe.offset_hz().abs() < 1.0,
            "DC signal should have ~0 Hz offset: got {:.1}",
            probe.offset_hz()
        );
    }

    #[test]
    fn test_rate_probe() {
        let mut probe = RateProbe::new(48000.0, 0.1); // 100ms window
        // Process 4800 samples (100ms worth)
        probe.update(4800);

        assert!(
            (probe.rate() - 48000.0).abs() < 100.0,
            "Rate should be ~48000: got {:.0}",
            probe.rate()
        );
        assert_eq!(probe.total_samples(), 4800);
    }

    #[test]
    fn test_iq_imbalance() {
        let mut probe = ConstellationProbe::new(1000);

        // Balanced signal
        let balanced: Vec<Complex64> = (0..1000)
            .map(|i| {
                let phase = 2.0 * std::f64::consts::PI * i as f64 / 100.0;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();
        probe.update(&balanced);

        let (gain_db, phase_deg) = probe.iq_imbalance();
        assert!(
            gain_db.abs() < 0.5,
            "Balanced signal should have ~0 dB gain imbalance: got {gain_db:.2}"
        );
        assert!(
            phase_deg.abs() < 2.0,
            "Balanced signal should have ~0° phase imbalance: got {phase_deg:.2}"
        );
    }

    #[test]
    fn test_probe_reset() {
        let mut probe = PowerProbe::new();
        probe.update(&[Complex64::new(1.0, 0.0); 100]);
        assert_eq!(probe.count(), 100);
        probe.reset();
        assert_eq!(probe.count(), 0);
    }
}
