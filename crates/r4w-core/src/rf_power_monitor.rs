//! Real-time RF power measurement and statistics collection with time/frequency domain analysis.
//!
//! This module provides [`RfPowerMonitor`] for continuous RF signal power monitoring,
//! including RMS/peak/instantaneous/average measurement modes, PAPR calculation,
//! CCDF statistics, threshold-based alarms, and frequency-selective per-bin power analysis.
//!
//! # Example
//!
//! ```
//! use r4w_core::rf_power_monitor::{RfPowerMonitor, MeasurementMode};
//!
//! // Create a monitor with a 1024-sample window
//! let mut monitor = RfPowerMonitor::new(1024);
//!
//! // Push some IQ samples (I, Q) representing a tone
//! let samples: Vec<(f64, f64)> = (0..2048)
//!     .map(|i| {
//!         let phase = 2.0 * std::f64::consts::PI * i as f64 / 64.0;
//!         (0.5 * phase.cos(), 0.5 * phase.sin())
//!     })
//!     .collect();
//! monitor.push_samples(&samples);
//!
//! // Measure power
//! let rms = monitor.power_rms_dbm();
//! let peak = monitor.power_peak_dbm();
//! let papr = monitor.papr_db();
//!
//! assert!(rms.is_finite());
//! assert!(peak >= rms);
//! // A pure tone has ~0 dB PAPR
//! assert!(papr < 1.0);
//!
//! // CCDF: probability that instantaneous power exceeds average by threshold
//! let ccdf = monitor.ccdf(3.0);
//! assert!(ccdf >= 0.0 && ccdf <= 1.0);
//!
//! // Statistics
//! let stats = monitor.statistics();
//! assert!(stats.mean_power_dbm.is_finite());
//! assert!(!stats.histogram_bins.is_empty());
//!
//! // Alarms
//! monitor.set_high_alarm_dbm(-10.0);
//! monitor.set_low_alarm_dbm(-50.0);
//! let alarms = monitor.check_alarms();
//! // alarms is a Vec<PowerAlarm>
//! ```

use std::f64::consts::PI;

/// Measurement mode for the RF power monitor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeasurementMode {
    /// Single-sample instantaneous power.
    Instantaneous,
    /// Sliding-window average power (linear mean).
    Average,
    /// Sliding-window peak power.
    Peak,
    /// Root-mean-square power (default).
    Rms,
}

/// Power alarm event triggered when thresholds are exceeded.
#[derive(Debug, Clone, PartialEq)]
pub struct PowerAlarm {
    /// Human-readable description of the alarm condition.
    pub description: String,
    /// Measured power level in dBm that triggered the alarm.
    pub measured_dbm: f64,
    /// Threshold that was crossed, in dBm.
    pub threshold_dbm: f64,
    /// Whether this is a high-power or low-power alarm.
    pub kind: AlarmKind,
}

/// The kind of power alarm.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlarmKind {
    /// Power exceeded the high threshold.
    High,
    /// Power dropped below the low threshold.
    Low,
}

/// Power statistics including histogram, percentiles, and CCDF data.
#[derive(Debug, Clone)]
pub struct PowerStatistics {
    /// Mean power in dBm over the measurement window.
    pub mean_power_dbm: f64,
    /// Peak power in dBm over the measurement window.
    pub peak_power_dbm: f64,
    /// RMS power in dBm.
    pub rms_power_dbm: f64,
    /// PAPR in dB.
    pub papr_db: f64,
    /// Histogram bin edges in dBm (length = `histogram_counts.len() + 1`).
    pub histogram_bins: Vec<f64>,
    /// Histogram counts per bin.
    pub histogram_counts: Vec<usize>,
    /// Percentile values: (percentile, dBm). Typically 1st, 5th, 25th, 50th, 75th, 95th, 99th.
    pub percentiles: Vec<(f64, f64)>,
}

/// Per-bin frequency-domain power result.
#[derive(Debug, Clone)]
pub struct FrequencyBinPower {
    /// Bin index.
    pub bin: usize,
    /// Power in dBm for this frequency bin.
    pub power_dbm: f64,
}

/// Real-time RF power monitor with configurable sliding window.
///
/// Maintains a circular buffer of instantaneous power values (in watts, assuming 50 ohm)
/// and provides time-domain and frequency-domain power measurements.
#[derive(Debug)]
pub struct RfPowerMonitor {
    /// Sliding window of instantaneous power values (watts).
    power_window: Vec<f64>,
    /// Current write position in the circular buffer.
    write_pos: usize,
    /// Number of valid samples in the window.
    count: usize,
    /// Maximum window size.
    window_size: usize,
    /// Raw IQ ring buffer for frequency-domain analysis.
    iq_buffer: Vec<(f64, f64)>,
    /// Write position for the IQ buffer.
    iq_write_pos: usize,
    /// Number of valid IQ samples.
    iq_count: usize,
    /// Active measurement mode.
    pub mode: MeasurementMode,
    /// High-power alarm threshold in dBm (None = disabled).
    high_alarm_dbm: Option<f64>,
    /// Low-power alarm threshold in dBm (None = disabled).
    low_alarm_dbm: Option<f64>,
    /// Impedance for dBm conversion (default 50 ohms).
    impedance_ohms: f64,
    /// Number of histogram bins for statistics.
    histogram_bin_count: usize,
}

// Reference impedance for dBm: P_dbm = 10*log10(V^2 / R / 0.001)
// For IQ samples treated as voltage: P = (I^2 + Q^2) / R
// We simplify: power_watts = (I^2 + Q^2) / impedance
// dBm = 10 * log10(power_watts / 0.001) = 10 * log10(power_watts) + 30

fn watts_to_dbm(watts: f64) -> f64 {
    if watts <= 0.0 {
        return -200.0; // floor
    }
    10.0 * watts.log10() + 30.0
}

fn mag_squared(sample: (f64, f64)) -> f64 {
    sample.0 * sample.0 + sample.1 * sample.1
}

impl RfPowerMonitor {
    /// Create a new `RfPowerMonitor` with the given sliding window size.
    ///
    /// The window size determines how many samples are used for average/RMS/peak
    /// calculations. A larger window provides more stable readings.
    pub fn new(window_size: usize) -> Self {
        let ws = window_size.max(1);
        Self {
            power_window: vec![0.0; ws],
            write_pos: 0,
            count: 0,
            window_size: ws,
            iq_buffer: vec![(0.0, 0.0); ws],
            iq_write_pos: 0,
            iq_count: 0,
            mode: MeasurementMode::Rms,
            high_alarm_dbm: None,
            low_alarm_dbm: None,
            impedance_ohms: 50.0,
            histogram_bin_count: 50,
        }
    }

    /// Set the reference impedance (default 50 ohms).
    pub fn set_impedance(&mut self, ohms: f64) {
        assert!(ohms > 0.0, "Impedance must be positive");
        self.impedance_ohms = ohms;
    }

    /// Set the high-power alarm threshold in dBm.
    pub fn set_high_alarm_dbm(&mut self, threshold: f64) {
        self.high_alarm_dbm = Some(threshold);
    }

    /// Set the low-power alarm threshold in dBm.
    pub fn set_low_alarm_dbm(&mut self, threshold: f64) {
        self.low_alarm_dbm = Some(threshold);
    }

    /// Clear all alarm thresholds.
    pub fn clear_alarms(&mut self) {
        self.high_alarm_dbm = None;
        self.low_alarm_dbm = None;
    }

    /// Set the number of histogram bins for statistics (default 50).
    pub fn set_histogram_bins(&mut self, bins: usize) {
        self.histogram_bin_count = bins.max(2);
    }

    /// Push a batch of IQ samples into the monitor.
    ///
    /// Each sample is `(I, Q)` where I and Q are voltage amplitudes.
    /// Power is computed as `(I^2 + Q^2) / impedance`.
    pub fn push_samples(&mut self, samples: &[(f64, f64)]) {
        for &sample in samples {
            let power_watts = mag_squared(sample) / self.impedance_ohms;

            self.power_window[self.write_pos] = power_watts;
            self.write_pos = (self.write_pos + 1) % self.window_size;
            if self.count < self.window_size {
                self.count += 1;
            }

            self.iq_buffer[self.iq_write_pos] = sample;
            self.iq_write_pos = (self.iq_write_pos + 1) % self.window_size;
            if self.iq_count < self.window_size {
                self.iq_count += 1;
            }
        }
    }

    /// Return the number of valid samples currently in the window.
    pub fn sample_count(&self) -> usize {
        self.count
    }

    /// Get the valid power samples in chronological order.
    fn valid_powers(&self) -> Vec<f64> {
        if self.count == 0 {
            return vec![];
        }
        let mut result = Vec::with_capacity(self.count);
        if self.count < self.window_size {
            // Buffer hasn't wrapped yet
            result.extend_from_slice(&self.power_window[..self.count]);
        } else {
            // Buffer has wrapped: read from write_pos to end, then start to write_pos
            result.extend_from_slice(&self.power_window[self.write_pos..]);
            result.extend_from_slice(&self.power_window[..self.write_pos]);
        }
        result
    }

    /// Get the valid IQ samples in chronological order.
    fn valid_iq(&self) -> Vec<(f64, f64)> {
        if self.iq_count == 0 {
            return vec![];
        }
        let mut result = Vec::with_capacity(self.iq_count);
        if self.iq_count < self.window_size {
            result.extend_from_slice(&self.iq_buffer[..self.iq_count]);
        } else {
            result.extend_from_slice(&self.iq_buffer[self.iq_write_pos..]);
            result.extend_from_slice(&self.iq_buffer[..self.iq_write_pos]);
        }
        result
    }

    /// RMS power in dBm across the measurement window.
    ///
    /// RMS power = mean of instantaneous power values.
    pub fn power_rms_dbm(&self) -> f64 {
        if self.count == 0 {
            return -200.0;
        }
        let powers = self.valid_powers();
        let mean = powers.iter().sum::<f64>() / powers.len() as f64;
        watts_to_dbm(mean)
    }

    /// Peak power in dBm across the measurement window.
    pub fn power_peak_dbm(&self) -> f64 {
        if self.count == 0 {
            return -200.0;
        }
        let powers = self.valid_powers();
        let peak = powers.iter().cloned().fold(0.0_f64, f64::max);
        watts_to_dbm(peak)
    }

    /// Average (mean) power in dBm — identical to RMS for power quantities.
    pub fn power_average_dbm(&self) -> f64 {
        self.power_rms_dbm()
    }

    /// Instantaneous power in dBm of the most recently pushed sample.
    pub fn power_instantaneous_dbm(&self) -> f64 {
        if self.count == 0 {
            return -200.0;
        }
        let last_pos = if self.write_pos == 0 {
            self.window_size - 1
        } else {
            self.write_pos - 1
        };
        watts_to_dbm(self.power_window[last_pos])
    }

    /// Report power in dBm according to the currently selected [`MeasurementMode`].
    pub fn power_dbm(&self) -> f64 {
        match self.mode {
            MeasurementMode::Instantaneous => self.power_instantaneous_dbm(),
            MeasurementMode::Average => self.power_average_dbm(),
            MeasurementMode::Peak => self.power_peak_dbm(),
            MeasurementMode::Rms => self.power_rms_dbm(),
        }
    }

    /// Peak-to-Average Power Ratio in dB.
    ///
    /// PAPR = 10 * log10(P_peak / P_avg)
    pub fn papr_db(&self) -> f64 {
        if self.count == 0 {
            return 0.0;
        }
        let powers = self.valid_powers();
        let mean = powers.iter().sum::<f64>() / powers.len() as f64;
        let peak = powers.iter().cloned().fold(0.0_f64, f64::max);
        if mean <= 0.0 {
            return 0.0;
        }
        10.0 * (peak / mean).log10()
    }

    /// Complementary Cumulative Distribution Function (CCDF).
    ///
    /// Returns the probability that the instantaneous power exceeds the
    /// average power by `threshold_db_above_avg` dB.
    pub fn ccdf(&self, threshold_db_above_avg: f64) -> f64 {
        if self.count == 0 {
            return 0.0;
        }
        let powers = self.valid_powers();
        let mean = powers.iter().sum::<f64>() / powers.len() as f64;
        if mean <= 0.0 {
            return 0.0;
        }
        // Threshold in watts: mean * 10^(threshold_dB/10)
        let threshold_watts = mean * 10.0_f64.powf(threshold_db_above_avg / 10.0);
        let exceeding = powers.iter().filter(|&&p| p > threshold_watts).count();
        exceeding as f64 / powers.len() as f64
    }

    /// Compute full power statistics including histogram, percentiles, and CCDF data.
    pub fn statistics(&self) -> PowerStatistics {
        let powers = self.valid_powers();
        if powers.is_empty() {
            return PowerStatistics {
                mean_power_dbm: -200.0,
                peak_power_dbm: -200.0,
                rms_power_dbm: -200.0,
                papr_db: 0.0,
                histogram_bins: vec![],
                histogram_counts: vec![],
                percentiles: vec![],
            };
        }

        let mean_w = powers.iter().sum::<f64>() / powers.len() as f64;
        let peak_w = powers.iter().cloned().fold(0.0_f64, f64::max);

        // Convert to dBm for histogram/percentile analysis
        let mut powers_dbm: Vec<f64> = powers.iter().map(|&w| watts_to_dbm(w)).collect();
        powers_dbm.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        let min_dbm = powers_dbm[0];
        let max_dbm = powers_dbm[powers_dbm.len() - 1];

        // Histogram
        let n_bins = self.histogram_bin_count;
        let bin_width = if (max_dbm - min_dbm).abs() < 1e-12 {
            1.0
        } else {
            (max_dbm - min_dbm) / n_bins as f64
        };
        let mut histogram_bins = Vec::with_capacity(n_bins + 1);
        for i in 0..=n_bins {
            histogram_bins.push(min_dbm + i as f64 * bin_width);
        }
        let mut histogram_counts = vec![0usize; n_bins];
        for &p in &powers_dbm {
            let mut bin = ((p - min_dbm) / bin_width) as usize;
            if bin >= n_bins {
                bin = n_bins - 1;
            }
            histogram_counts[bin] += 1;
        }

        // Percentiles
        let percentile_values = [1.0, 5.0, 25.0, 50.0, 75.0, 95.0, 99.0];
        let percentiles: Vec<(f64, f64)> = percentile_values
            .iter()
            .map(|&pct| {
                let idx = ((pct / 100.0) * (powers_dbm.len() - 1) as f64).round() as usize;
                let idx = idx.min(powers_dbm.len() - 1);
                (pct, powers_dbm[idx])
            })
            .collect();

        PowerStatistics {
            mean_power_dbm: watts_to_dbm(mean_w),
            peak_power_dbm: watts_to_dbm(peak_w),
            rms_power_dbm: watts_to_dbm(mean_w),
            papr_db: self.papr_db(),
            histogram_bins,
            histogram_counts,
            percentiles,
        }
    }

    /// Check all configured alarm thresholds and return any active alarms.
    pub fn check_alarms(&self) -> Vec<PowerAlarm> {
        let mut alarms = Vec::new();
        let current = self.power_rms_dbm();

        if let Some(high) = self.high_alarm_dbm {
            if current > high {
                alarms.push(PowerAlarm {
                    description: format!(
                        "High power alarm: {:.1} dBm exceeds {:.1} dBm threshold",
                        current, high
                    ),
                    measured_dbm: current,
                    threshold_dbm: high,
                    kind: AlarmKind::High,
                });
            }
        }

        if let Some(low) = self.low_alarm_dbm {
            if current < low {
                alarms.push(PowerAlarm {
                    description: format!(
                        "Low power alarm: {:.1} dBm below {:.1} dBm threshold",
                        current, low
                    ),
                    measured_dbm: current,
                    threshold_dbm: low,
                    kind: AlarmKind::Low,
                });
            }
        }

        alarms
    }

    /// Frequency-selective power monitoring using a basic DFT.
    ///
    /// Computes per-bin power for `fft_size` frequency bins using the stored IQ samples.
    /// If fewer than `fft_size` samples are available, the remaining are zero-padded.
    ///
    /// Returns a vector of [`FrequencyBinPower`] sorted by bin index.
    pub fn frequency_bin_powers(&self, fft_size: usize) -> Vec<FrequencyBinPower> {
        let fft_size = fft_size.max(1);
        let iq = self.valid_iq();
        let n = iq.len().min(fft_size);

        // Simple DFT (no external FFT crate)
        let mut result = Vec::with_capacity(fft_size);
        for k in 0..fft_size {
            let mut re = 0.0;
            let mut im = 0.0;
            for (idx, &(i_val, q_val)) in iq.iter().take(n).enumerate() {
                let angle = -2.0 * PI * k as f64 * idx as f64 / fft_size as f64;
                let cos_a = angle.cos();
                let sin_a = angle.sin();
                re += i_val * cos_a - q_val * sin_a;
                im += i_val * sin_a + q_val * cos_a;
            }
            let power_watts = (re * re + im * im) / (fft_size as f64 * self.impedance_ohms);
            result.push(FrequencyBinPower {
                bin: k,
                power_dbm: watts_to_dbm(power_watts),
            });
        }
        result
    }

    /// Time-domain power envelope as a vector of (sample_index, power_dbm) pairs.
    ///
    /// Returns the instantaneous power for each sample currently in the sliding window.
    pub fn power_envelope(&self) -> Vec<(usize, f64)> {
        self.valid_powers()
            .iter()
            .enumerate()
            .map(|(i, &w)| (i, watts_to_dbm(w)))
            .collect()
    }

    /// Reset the monitor, clearing all stored samples.
    pub fn reset(&mut self) {
        self.write_pos = 0;
        self.count = 0;
        self.iq_write_pos = 0;
        self.iq_count = 0;
        for v in self.power_window.iter_mut() {
            *v = 0.0;
        }
        for v in self.iq_buffer.iter_mut() {
            *v = (0.0, 0.0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a tone at a given amplitude.
    fn tone_samples(amplitude: f64, n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let phase = 2.0 * PI * i as f64 / 64.0;
                (amplitude * phase.cos(), amplitude * phase.sin())
            })
            .collect()
    }

    /// Helper: generate Gaussian-like noise using a simple PRNG (xorshift).
    fn noise_samples(n: usize) -> Vec<(f64, f64)> {
        let mut state: u64 = 0xDEAD_BEEF_CAFE_1234;
        let mut samples = Vec::with_capacity(n);
        for _ in 0..n {
            // Box-Muller via xorshift pseudo-random
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            let u1 = (state as f64) / (u64::MAX as f64);
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            let u2 = (state as f64) / (u64::MAX as f64);
            let u1 = u1.max(1e-15); // avoid log(0)
            let r = (-2.0 * u1.ln()).sqrt();
            let theta = 2.0 * PI * u2;
            samples.push((r * theta.cos() * 0.1, r * theta.sin() * 0.1));
        }
        samples
    }

    #[test]
    fn test_new_monitor_defaults() {
        let mon = RfPowerMonitor::new(512);
        assert_eq!(mon.window_size, 512);
        assert_eq!(mon.sample_count(), 0);
        assert_eq!(mon.mode, MeasurementMode::Rms);
    }

    #[test]
    fn test_empty_monitor_returns_floor() {
        let mon = RfPowerMonitor::new(64);
        assert_eq!(mon.power_rms_dbm(), -200.0);
        assert_eq!(mon.power_peak_dbm(), -200.0);
        assert_eq!(mon.power_instantaneous_dbm(), -200.0);
    }

    #[test]
    fn test_push_samples_count() {
        let mut mon = RfPowerMonitor::new(1024);
        let samples = tone_samples(1.0, 500);
        mon.push_samples(&samples);
        assert_eq!(mon.sample_count(), 500);

        mon.push_samples(&tone_samples(1.0, 600));
        // Window wraps at 1024
        assert_eq!(mon.sample_count(), 1024);
    }

    #[test]
    fn test_constant_amplitude_tone_power() {
        let mut mon = RfPowerMonitor::new(2048);
        let amp = 1.0;
        let samples = tone_samples(amp, 2048);
        mon.push_samples(&samples);

        // For amplitude A, power = A^2 / R = 1.0 / 50 = 0.02 W
        // dBm = 10*log10(0.02) + 30 = 10*(-1.699) + 30 ≈ 13.01 dBm
        let expected = watts_to_dbm(amp * amp / 50.0);
        let rms = mon.power_rms_dbm();
        assert!(
            (rms - expected).abs() < 0.5,
            "RMS {rms} not close to expected {expected}"
        );
    }

    #[test]
    fn test_peak_geq_rms() {
        let mut mon = RfPowerMonitor::new(1024);
        mon.push_samples(&noise_samples(1024));
        assert!(mon.power_peak_dbm() >= mon.power_rms_dbm());
    }

    #[test]
    fn test_papr_pure_tone() {
        // A constant-envelope signal (pure tone) has PAPR ≈ 0 dB
        let mut mon = RfPowerMonitor::new(2048);
        mon.push_samples(&tone_samples(0.5, 2048));
        let papr = mon.papr_db();
        assert!(
            papr.abs() < 0.5,
            "Pure tone PAPR should be near 0 dB, got {papr}"
        );
    }

    #[test]
    fn test_papr_noisy_signal() {
        // Noise should have higher PAPR than a pure tone
        let mut mon = RfPowerMonitor::new(4096);
        mon.push_samples(&noise_samples(4096));
        let papr = mon.papr_db();
        assert!(papr > 0.5, "Noise PAPR should be > 0.5 dB, got {papr}");
    }

    #[test]
    fn test_ccdf_zero_threshold_is_less_than_one() {
        let mut mon = RfPowerMonitor::new(1024);
        mon.push_samples(&noise_samples(1024));
        let ccdf_0 = mon.ccdf(0.0);
        // At 0 dB above average, roughly half the samples should exceed (for noise)
        assert!(
            ccdf_0 > 0.0 && ccdf_0 < 1.0,
            "CCDF at 0 dB should be between 0 and 1, got {ccdf_0}"
        );
    }

    #[test]
    fn test_ccdf_high_threshold_near_zero() {
        let mut mon = RfPowerMonitor::new(1024);
        mon.push_samples(&noise_samples(1024));
        let ccdf_20 = mon.ccdf(20.0);
        // At 20 dB above average, very few samples should exceed
        assert!(
            ccdf_20 < 0.1,
            "CCDF at 20 dB above avg should be near 0, got {ccdf_20}"
        );
    }

    #[test]
    fn test_statistics_has_valid_fields() {
        let mut mon = RfPowerMonitor::new(512);
        mon.push_samples(&tone_samples(0.3, 512));
        let stats = mon.statistics();
        assert!(stats.mean_power_dbm.is_finite());
        assert!(stats.peak_power_dbm >= stats.mean_power_dbm - 0.01, "peak {:.6} < mean {:.6}", stats.peak_power_dbm, stats.mean_power_dbm);
        assert!(!stats.histogram_bins.is_empty());
        assert!(!stats.histogram_counts.is_empty());
        assert_eq!(stats.percentiles.len(), 7);
        // 50th percentile should be near the mean for constant envelope
        let median = stats.percentiles.iter().find(|p| (p.0 - 50.0).abs() < 0.01).unwrap();
        assert!(
            (median.1 - stats.mean_power_dbm).abs() < 1.0,
            "Median {:.1} should be near mean {:.1}",
            median.1,
            stats.mean_power_dbm,
        );
    }

    #[test]
    fn test_alarm_high_triggered() {
        let mut mon = RfPowerMonitor::new(256);
        mon.push_samples(&tone_samples(1.0, 256));
        // Power is ~13 dBm, set threshold below that
        mon.set_high_alarm_dbm(0.0);
        let alarms = mon.check_alarms();
        assert_eq!(alarms.len(), 1);
        assert_eq!(alarms[0].kind, AlarmKind::High);
    }

    #[test]
    fn test_alarm_low_triggered() {
        let mut mon = RfPowerMonitor::new(256);
        mon.push_samples(&tone_samples(0.001, 256));
        // Very low power, set threshold above it
        mon.set_low_alarm_dbm(0.0);
        let alarms = mon.check_alarms();
        assert_eq!(alarms.len(), 1);
        assert_eq!(alarms[0].kind, AlarmKind::Low);
    }

    #[test]
    fn test_alarm_none_when_in_range() {
        let mut mon = RfPowerMonitor::new(256);
        mon.push_samples(&tone_samples(0.5, 256));
        let rms = mon.power_rms_dbm();
        mon.set_high_alarm_dbm(rms + 10.0);
        mon.set_low_alarm_dbm(rms - 10.0);
        let alarms = mon.check_alarms();
        assert!(alarms.is_empty(), "No alarms expected, got {alarms:?}");
    }

    #[test]
    fn test_frequency_bin_powers() {
        let mut mon = RfPowerMonitor::new(64);
        // Pure tone at bin 1 (1 cycle in 64 samples)
        let samples: Vec<(f64, f64)> = (0..64)
            .map(|i| {
                let phase = 2.0 * PI * i as f64 / 64.0;
                (phase.cos(), phase.sin())
            })
            .collect();
        mon.push_samples(&samples);
        let bins = mon.frequency_bin_powers(64);
        assert_eq!(bins.len(), 64);
        // Bin 1 should have the highest power
        let max_bin = bins
            .iter()
            .max_by(|a, b| a.power_dbm.partial_cmp(&b.power_dbm).unwrap())
            .unwrap();
        assert_eq!(max_bin.bin, 1, "Peak should be at bin 1, got bin {}", max_bin.bin);
    }

    #[test]
    fn test_power_envelope_length() {
        let mut mon = RfPowerMonitor::new(128);
        mon.push_samples(&tone_samples(0.5, 100));
        let env = mon.power_envelope();
        assert_eq!(env.len(), 100);
    }

    #[test]
    fn test_reset_clears_state() {
        let mut mon = RfPowerMonitor::new(256);
        mon.push_samples(&tone_samples(1.0, 200));
        assert_eq!(mon.sample_count(), 200);
        mon.reset();
        assert_eq!(mon.sample_count(), 0);
        assert_eq!(mon.power_rms_dbm(), -200.0);
    }

    #[test]
    fn test_measurement_modes() {
        let mut mon = RfPowerMonitor::new(512);
        mon.push_samples(&tone_samples(0.5, 512));

        mon.mode = MeasurementMode::Rms;
        let rms = mon.power_dbm();

        mon.mode = MeasurementMode::Average;
        let avg = mon.power_dbm();

        mon.mode = MeasurementMode::Peak;
        let peak = mon.power_dbm();

        mon.mode = MeasurementMode::Instantaneous;
        let inst = mon.power_dbm();

        // RMS == Average for power
        assert!((rms - avg).abs() < 0.01);
        // Peak >= RMS
        assert!(peak >= rms - 0.01);
        // Instantaneous should be finite
        assert!(inst.is_finite());
    }

    #[test]
    fn test_window_wrapping() {
        let mut mon = RfPowerMonitor::new(64);
        // Push more than window size to test wrapping
        mon.push_samples(&tone_samples(0.1, 40));
        mon.push_samples(&tone_samples(1.0, 40));
        assert_eq!(mon.sample_count(), 64);
        // After wrapping, the second batch dominates. RMS should be closer to amp=1.0 power.
        let rms = mon.power_rms_dbm();
        let expected_high = watts_to_dbm(1.0 / 50.0);
        let expected_low = watts_to_dbm(0.01 / 50.0);
        // Blended: 24 old samples (amp=0.1) + 40 new samples (amp=1.0)
        assert!(
            rms > expected_low + 5.0,
            "Wrapped RMS {rms} should be well above low-amp power {expected_low}"
        );
        assert!(
            rms < expected_high + 1.0,
            "Wrapped RMS {rms} should be near high-amp power {expected_high}"
        );
    }

    #[test]
    fn test_set_impedance() {
        let mut mon = RfPowerMonitor::new(256);
        mon.set_impedance(75.0);
        mon.push_samples(&tone_samples(1.0, 256));
        let rms_75 = mon.power_rms_dbm();

        let mut mon50 = RfPowerMonitor::new(256);
        mon50.push_samples(&tone_samples(1.0, 256));
        let rms_50 = mon50.power_rms_dbm();

        // Higher impedance => lower power => lower dBm
        assert!(
            rms_75 < rms_50,
            "75 ohm ({rms_75}) should give lower dBm than 50 ohm ({rms_50})"
        );
    }
}
