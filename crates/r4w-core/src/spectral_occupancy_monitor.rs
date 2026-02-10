//! Wideband spectrum occupancy measurement for cognitive radio and spectrum management.
//!
//! This module provides a [`SpectralOccupancyMonitor`] that tracks per-channel power levels
//! over time, computes duty cycles, and generates occupancy reports. It is designed for
//! cognitive radio applications where spectrum awareness is essential for dynamic spectrum
//! access.
//!
//! # Example
//!
//! ```
//! use r4w_core::spectral_occupancy_monitor::{SpectralOccupancyMonitor, DetectionMethod};
//!
//! // Create a monitor with 1 MHz sample rate and 256-point FFT
//! let mut monitor = SpectralOccupancyMonitor::new(1_000_000.0, 256);
//!
//! // Define two channels to watch
//! monitor.add_channel(100_000.0, 50_000.0, "Channel A");
//! monitor.add_channel(300_000.0, 80_000.0, "Channel B");
//!
//! // Set detection threshold (dB above noise floor)
//! monitor.set_threshold_db(6.0);
//!
//! // Push some IQ samples (here, a tone at ~100 kHz)
//! let sample_rate = 1_000_000.0;
//! let n = 512;
//! let samples: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / sample_rate;
//!         let phase = 2.0 * std::f64::consts::PI * 100_000.0 * t;
//!         (phase.cos() * 1.0, phase.sin() * 1.0)
//!     })
//!     .collect();
//! monitor.push_samples(&samples);
//!
//! // Generate a report
//! let report = monitor.occupancy_report();
//! assert_eq!(report.channels.len(), 2);
//! assert!(report.measurement_duration_s >= 0.0);
//!
//! // Check per-channel duty cycle
//! let duty_a = monitor.channel_duty_cycle(0);
//! assert!(duty_a >= 0.0 && duty_a <= 1.0);
//!
//! // Overall utilization
//! let util = monitor.overall_utilization();
//! assert!(util >= 0.0 && util <= 1.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Defines a frequency channel to monitor.
#[derive(Debug, Clone)]
pub struct ChannelDefinition {
    /// Center frequency in Hz (relative to baseband center = 0 Hz).
    pub center_freq: f64,
    /// Channel bandwidth in Hz.
    pub bandwidth: f64,
    /// Human-readable label.
    pub label: String,
}

/// Occupancy statistics for a single channel.
#[derive(Debug, Clone)]
pub struct ChannelOccupancy {
    /// Fraction of measurement windows where the channel was occupied (0.0 -- 1.0).
    pub duty_cycle: f64,
    /// Mean power across all measurement windows in dBm.
    pub mean_power_dbm: f64,
    /// Peak power observed across all measurement windows in dBm.
    pub peak_power_dbm: f64,
    /// Cumulative time the channel was detected as busy (seconds).
    pub time_busy_s: f64,
    /// Cumulative time the channel was detected as idle (seconds).
    pub time_idle_s: f64,
}

/// Aggregate occupancy report covering all monitored channels.
#[derive(Debug, Clone)]
pub struct OccupancyReport {
    /// Per-channel occupancy statistics (same order as channels were added).
    pub channels: Vec<ChannelOccupancy>,
    /// Fraction of channels that are occupied (weighted by bandwidth).
    pub overall_utilization: f64,
    /// Total measurement duration in seconds.
    pub measurement_duration_s: f64,
}

/// Detection method used to determine channel occupancy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectionMethod {
    /// Simple power threshold comparison (default).
    EnergyDetection,
    /// Cyclostationary feature detection (placeholder for future implementation).
    CyclostationaryFeature,
    /// Matched-filter detection (placeholder for future implementation).
    MatchedFilter,
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Per-channel accumulator used during measurement.
#[derive(Debug, Clone)]
struct ChannelAccumulator {
    /// Number of measurement windows processed.
    window_count: usize,
    /// Number of windows where channel was occupied.
    busy_count: usize,
    /// Sum of linear power values (for mean calculation).
    power_sum: f64,
    /// Peak linear power observed.
    peak_power: f64,
}

impl ChannelAccumulator {
    fn new() -> Self {
        Self {
            window_count: 0,
            busy_count: 0,
            power_sum: 0.0,
            peak_power: 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// SpectralOccupancyMonitor
// ---------------------------------------------------------------------------

/// Core spectrum occupancy tracker.
///
/// Accepts a stream of IQ samples, computes per-window FFTs, and evaluates
/// per-channel power against a configurable threshold to determine occupancy.
#[derive(Debug)]
pub struct SpectralOccupancyMonitor {
    sample_rate: f64,
    fft_size: usize,
    channels: Vec<ChannelDefinition>,
    accumulators: Vec<ChannelAccumulator>,
    detection_method: DetectionMethod,
    threshold_db: f64,
    /// Buffer for accumulating incoming samples until we have a full FFT window.
    sample_buffer: Vec<(f64, f64)>,
    /// Total number of samples processed (for timing).
    total_samples: usize,
    /// Pre-computed Hann window coefficients.
    window: Vec<f64>,
    /// Twiddle factors for the radix-2 DIT FFT.
    twiddles: Vec<(f64, f64)>,
}

impl SpectralOccupancyMonitor {
    /// Create a new monitor.
    ///
    /// # Arguments
    /// * `sample_rate` - Sample rate in Hz.
    /// * `fft_size` - FFT length (will be used as the measurement window size).
    ///   For best results use a power of two, but any positive size is accepted.
    pub fn new(sample_rate: f64, fft_size: usize) -> Self {
        assert!(sample_rate > 0.0, "sample_rate must be positive");
        assert!(fft_size > 0, "fft_size must be > 0");

        // Pre-compute Hann window
        let window: Vec<f64> = (0..fft_size)
            .map(|n| 0.5 * (1.0 - (2.0 * PI * n as f64 / fft_size as f64).cos()))
            .collect();

        // Pre-compute twiddle factors for radix-2 DIT FFT
        let twiddles: Vec<(f64, f64)> = (0..fft_size)
            .map(|k| {
                let angle = -2.0 * PI * k as f64 / fft_size as f64;
                (angle.cos(), angle.sin())
            })
            .collect();

        Self {
            sample_rate,
            fft_size,
            channels: Vec::new(),
            accumulators: Vec::new(),
            detection_method: DetectionMethod::EnergyDetection,
            threshold_db: 6.0,
            sample_buffer: Vec::with_capacity(fft_size),
            total_samples: 0,
            window,
            twiddles,
        }
    }

    /// Add a channel to monitor.
    ///
    /// `center_hz` and `bw_hz` are in the baseband frame (center of the sampled
    /// band is 0 Hz; Nyquist range is +/-sample_rate/2).
    pub fn add_channel(&mut self, center_hz: f64, bw_hz: f64, label: &str) {
        self.channels.push(ChannelDefinition {
            center_freq: center_hz,
            bandwidth: bw_hz,
            label: label.to_string(),
        });
        self.accumulators.push(ChannelAccumulator::new());
    }

    /// Set the detection method.
    pub fn set_detection_method(&mut self, method: DetectionMethod) {
        self.detection_method = method;
    }

    /// Set the detection threshold in dB above the estimated noise floor.
    pub fn set_threshold_db(&mut self, threshold: f64) {
        self.threshold_db = threshold;
    }

    /// Push a batch of IQ samples into the monitor.
    ///
    /// Internally buffers samples and processes complete FFT windows as they
    /// become available.
    pub fn push_samples(&mut self, samples: &[(f64, f64)]) {
        let mut offset = 0;
        while offset < samples.len() {
            let remaining_in_window = self.fft_size - self.sample_buffer.len();
            let take = remaining_in_window.min(samples.len() - offset);
            self.sample_buffer
                .extend_from_slice(&samples[offset..offset + take]);
            offset += take;

            if self.sample_buffer.len() == self.fft_size {
                self.process_window();
                self.total_samples += self.fft_size;
                self.sample_buffer.clear();
            }
        }
    }

    /// Return the duty cycle for a given channel index (0.0 -- 1.0).
    ///
    /// Returns 0.0 if no windows have been processed yet or the index is out
    /// of range.
    pub fn channel_duty_cycle(&self, channel_idx: usize) -> f64 {
        self.accumulators
            .get(channel_idx)
            .map(|a| {
                if a.window_count == 0 {
                    0.0
                } else {
                    a.busy_count as f64 / a.window_count as f64
                }
            })
            .unwrap_or(0.0)
    }

    /// Return the overall bandwidth-weighted utilization across all channels.
    pub fn overall_utilization(&self) -> f64 {
        if self.channels.is_empty() {
            return 0.0;
        }
        let total_bw: f64 = self.channels.iter().map(|c| c.bandwidth).sum();
        if total_bw == 0.0 {
            return 0.0;
        }
        let weighted: f64 = self
            .channels
            .iter()
            .zip(self.accumulators.iter())
            .map(|(ch, acc)| {
                let dc = if acc.window_count == 0 {
                    0.0
                } else {
                    acc.busy_count as f64 / acc.window_count as f64
                };
                dc * ch.bandwidth
            })
            .sum();
        weighted / total_bw
    }

    /// Generate a full occupancy report.
    pub fn occupancy_report(&self) -> OccupancyReport {
        let duration = self.total_samples as f64 / self.sample_rate;
        let window_duration = self.fft_size as f64 / self.sample_rate;

        let channel_stats: Vec<ChannelOccupancy> = self
            .channels
            .iter()
            .zip(self.accumulators.iter())
            .map(|(_ch, acc)| {
                let (duty_cycle, mean_power_dbm, peak_power_dbm, time_busy, time_idle) =
                    if acc.window_count == 0 {
                        (0.0, f64::NEG_INFINITY, f64::NEG_INFINITY, 0.0, 0.0)
                    } else {
                        let dc = acc.busy_count as f64 / acc.window_count as f64;
                        let mean_lin = acc.power_sum / acc.window_count as f64;
                        let mean_dbm = linear_to_dbm(mean_lin);
                        let peak_dbm = linear_to_dbm(acc.peak_power);
                        let busy_s = acc.busy_count as f64 * window_duration;
                        let idle_s =
                            (acc.window_count - acc.busy_count) as f64 * window_duration;
                        (dc, mean_dbm, peak_dbm, busy_s, idle_s)
                    };
                ChannelOccupancy {
                    duty_cycle,
                    mean_power_dbm,
                    peak_power_dbm,
                    time_busy_s: time_busy,
                    time_idle_s: time_idle,
                }
            })
            .collect();

        OccupancyReport {
            channels: channel_stats,
            overall_utilization: self.overall_utilization(),
            measurement_duration_s: duration,
        }
    }

    // -----------------------------------------------------------------------
    // Internal
    // -----------------------------------------------------------------------

    /// Process one FFT window worth of samples.
    fn process_window(&mut self) {
        // Apply window function
        let windowed: Vec<(f64, f64)> = self
            .sample_buffer
            .iter()
            .zip(self.window.iter())
            .map(|(&(re, im), &w)| (re * w, im * w))
            .collect();

        // Compute FFT (DFT for arbitrary sizes, optimised Cooley-Tukey for power-of-2)
        let spectrum = self.fft(&windowed);

        // Compute power spectral density (magnitude squared, normalised by N^2)
        let n2 = (self.fft_size * self.fft_size) as f64;
        let psd: Vec<f64> = spectrum
            .iter()
            .map(|&(re, im)| (re * re + im * im) / n2)
            .collect();

        // Estimate noise floor as the median of the PSD bins
        let noise_floor = median_of(&psd);
        let threshold_linear = noise_floor * db_to_linear(self.threshold_db);

        // Evaluate each channel
        for (ch_idx, ch) in self.channels.iter().enumerate() {
            let (bin_lo, bin_hi) = self.channel_bins(ch);
            if bin_lo > bin_hi {
                // Channel falls outside Nyquist range -- skip
                continue;
            }

            // Sum power in channel bins
            let mut channel_power = 0.0;
            let mut count = 0usize;
            for bin in bin_lo..=bin_hi {
                channel_power += psd[bin % self.fft_size];
                count += 1;
            }
            if count == 0 {
                continue;
            }
            let mean_power = channel_power / count as f64;

            let acc = &mut self.accumulators[ch_idx];
            acc.window_count += 1;
            acc.power_sum += mean_power;
            if mean_power > acc.peak_power {
                acc.peak_power = mean_power;
            }

            let occupied = match self.detection_method {
                DetectionMethod::EnergyDetection => mean_power > threshold_linear,
                // Placeholder: fall back to energy detection
                DetectionMethod::CyclostationaryFeature => mean_power > threshold_linear,
                DetectionMethod::MatchedFilter => mean_power > threshold_linear,
            };
            if occupied {
                acc.busy_count += 1;
            }
        }
    }

    /// Map a channel definition to a range of FFT bin indices.
    ///
    /// FFT bins are arranged as [0, df, 2*df, ..., (N/2-1)*df, -N/2*df, ..., -df]
    /// where df = sample_rate / fft_size.
    fn channel_bins(&self, ch: &ChannelDefinition) -> (usize, usize) {
        let df = self.sample_rate / self.fft_size as f64;
        let lo_freq = ch.center_freq - ch.bandwidth / 2.0;
        let hi_freq = ch.center_freq + ch.bandwidth / 2.0;

        let freq_to_bin = |f: f64| -> usize {
            let mut freq = f;
            // Wrap into [-sample_rate/2, sample_rate/2)
            let half_sr = self.sample_rate / 2.0;
            while freq < -half_sr {
                freq += self.sample_rate;
            }
            while freq >= half_sr {
                freq -= self.sample_rate;
            }
            // Map to FFT bin
            if freq >= 0.0 {
                (freq / df).round() as usize % self.fft_size
            } else {
                ((self.fft_size as f64 + freq / df).round() as usize) % self.fft_size
            }
        };

        (freq_to_bin(lo_freq), freq_to_bin(hi_freq))
    }

    /// Compute the DFT of `x`. Uses Cooley-Tukey radix-2 DIT when `x.len()` is
    /// a power of two, otherwise falls back to a direct DFT.
    fn fft(&self, x: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n = x.len();
        if n <= 1 {
            return x.to_vec();
        }
        if n.is_power_of_two() {
            self.fft_radix2(x)
        } else {
            self.dft_direct(x)
        }
    }

    fn dft_direct(&self, x: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n = x.len();
        let mut out = vec![(0.0, 0.0); n];
        for k in 0..n {
            let mut re_sum = 0.0;
            let mut im_sum = 0.0;
            for (j, &(xr, xi)) in x.iter().enumerate() {
                let angle = -2.0 * PI * (k * j) as f64 / n as f64;
                let (s, c) = angle.sin_cos();
                re_sum += xr * c - xi * s;
                im_sum += xr * s + xi * c;
            }
            out[k] = (re_sum, im_sum);
        }
        out
    }

    fn fft_radix2(&self, x: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n = x.len();
        if n == 1 {
            return x.to_vec();
        }

        // Bit-reversal permutation
        let mut a = x.to_vec();
        let bits = n.trailing_zeros();
        for i in 0..n {
            let j = i.reverse_bits() >> (usize::BITS - bits);
            if i < j {
                a.swap(i, j);
            }
        }

        // Iterative Cooley-Tukey
        let mut len = 2;
        while len <= n {
            let half = len / 2;
            let step = n / len;
            for start in (0..n).step_by(len) {
                for k in 0..half {
                    let tw = self.twiddles[k * step];
                    let i1 = start + k;
                    let i2 = start + k + half;
                    let (ar, ai) = a[i2];
                    let tr = ar * tw.0 - ai * tw.1;
                    let ti = ar * tw.1 + ai * tw.0;
                    let (ur, ui) = a[i1];
                    a[i1] = (ur + tr, ui + ti);
                    a[i2] = (ur - tr, ui - ti);
                }
            }
            len *= 2;
        }
        a
    }
}

// ---------------------------------------------------------------------------
// Free helper functions
// ---------------------------------------------------------------------------

/// Convert linear power to dBm (assuming 1 mW reference, i.e. dBm = 10*log10(p) + 30).
/// For our normalised PSD values the absolute reference is arbitrary; we use
/// 10*log10(p) which gives a "dBFS-like" scale. Callers who need true dBm
/// should calibrate externally.
fn linear_to_dbm(p: f64) -> f64 {
    if p <= 0.0 {
        f64::NEG_INFINITY
    } else {
        10.0 * p.log10()
    }
}

/// Convert dB to a linear ratio.
fn db_to_linear(db: f64) -> f64 {
    10.0_f64.powf(db / 10.0)
}

/// Compute the median of a slice (non-destructive; clones internally).
fn median_of(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    let mut sorted = data.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mid = sorted.len() / 2;
    if sorted.len() % 2 == 0 {
        (sorted[mid - 1] + sorted[mid]) / 2.0
    } else {
        sorted[mid]
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_RATE: f64 = 1_000_000.0;
    const FFT_SIZE: usize = 256;

    fn make_tone(freq: f64, amplitude: f64, n: usize, sr: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let t = i as f64 / sr;
                let phase = 2.0 * PI * freq * t;
                (amplitude * phase.cos(), amplitude * phase.sin())
            })
            .collect()
    }

    fn make_noise(amplitude: f64, n: usize) -> Vec<(f64, f64)> {
        // Simple deterministic pseudo-noise using an LCG
        let mut seed: u64 = 12345;
        (0..n)
            .map(|_| {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                let r1 = (seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5;
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                let r2 = (seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5;
                (amplitude * r1, amplitude * r2)
            })
            .collect()
    }

    // -- Test 1: Construction --
    #[test]
    fn test_construction() {
        let m = SpectralOccupancyMonitor::new(SAMPLE_RATE, FFT_SIZE);
        assert_eq!(m.fft_size, FFT_SIZE);
        assert_eq!(m.channels.len(), 0);
        assert_eq!(m.total_samples, 0);
    }

    // -- Test 2: Add channels --
    #[test]
    fn test_add_channels() {
        let mut m = SpectralOccupancyMonitor::new(SAMPLE_RATE, FFT_SIZE);
        m.add_channel(100_000.0, 50_000.0, "A");
        m.add_channel(200_000.0, 30_000.0, "B");
        assert_eq!(m.channels.len(), 2);
        assert_eq!(m.accumulators.len(), 2);
        assert_eq!(m.channels[0].label, "A");
        assert_eq!(m.channels[1].label, "B");
    }

    // -- Test 3: Empty report --
    #[test]
    fn test_empty_report() {
        let mut m = SpectralOccupancyMonitor::new(SAMPLE_RATE, FFT_SIZE);
        m.add_channel(100_000.0, 50_000.0, "A");
        let report = m.occupancy_report();
        assert_eq!(report.channels.len(), 1);
        assert_eq!(report.channels[0].duty_cycle, 0.0);
        assert_eq!(report.measurement_duration_s, 0.0);
    }

    // -- Test 4: Duty cycle with no samples --
    #[test]
    fn test_duty_cycle_no_samples() {
        let mut m = SpectralOccupancyMonitor::new(SAMPLE_RATE, FFT_SIZE);
        m.add_channel(0.0, 50_000.0, "DC");
        assert_eq!(m.channel_duty_cycle(0), 0.0);
    }

    // -- Test 5: Duty cycle out-of-range index --
    #[test]
    fn test_duty_cycle_out_of_range() {
        let m = SpectralOccupancyMonitor::new(SAMPLE_RATE, FFT_SIZE);
        assert_eq!(m.channel_duty_cycle(99), 0.0);
    }

    // -- Test 6: Overall utilization with no channels --
    #[test]
    fn test_utilization_no_channels() {
        let m = SpectralOccupancyMonitor::new(SAMPLE_RATE, FFT_SIZE);
        assert_eq!(m.overall_utilization(), 0.0);
    }

    // -- Test 7: Strong tone should be detected --
    #[test]
    fn test_strong_tone_detected() {
        let mut m = SpectralOccupancyMonitor::new(SAMPLE_RATE, FFT_SIZE);
        m.add_channel(100_000.0, 20_000.0, "Tone");
        m.set_threshold_db(6.0);

        // Generate a strong tone at 100 kHz plus weak background noise
        let tone = make_tone(100_000.0, 10.0, FFT_SIZE * 4, SAMPLE_RATE);
        m.push_samples(&tone);

        let dc = m.channel_duty_cycle(0);
        // The tone channel should show high occupancy
        assert!(
            dc > 0.5,
            "Expected high duty cycle for strong tone, got {dc}"
        );
    }

    // -- Test 8: Quiet channel should be idle --
    #[test]
    fn test_quiet_channel_idle() {
        let mut m = SpectralOccupancyMonitor::new(SAMPLE_RATE, FFT_SIZE);
        // Tone at 100 kHz but we monitor at 400 kHz
        m.add_channel(400_000.0, 20_000.0, "Quiet");
        m.set_threshold_db(6.0);

        let tone = make_tone(100_000.0, 10.0, FFT_SIZE * 4, SAMPLE_RATE);
        m.push_samples(&tone);

        let dc = m.channel_duty_cycle(0);
        assert!(
            dc < 0.5,
            "Expected low duty cycle for quiet channel, got {dc}"
        );
    }

    // -- Test 9: Measurement duration --
    #[test]
    fn test_measurement_duration() {
        let mut m = SpectralOccupancyMonitor::new(SAMPLE_RATE, FFT_SIZE);
        m.add_channel(0.0, 50_000.0, "X");
        let n = FFT_SIZE * 3; // exactly 3 windows
        let noise = make_noise(0.01, n);
        m.push_samples(&noise);

        let report = m.occupancy_report();
        let expected_s = (FFT_SIZE * 3) as f64 / SAMPLE_RATE;
        assert!(
            (report.measurement_duration_s - expected_s).abs() < 1e-9,
            "Duration mismatch: {} vs {}",
            report.measurement_duration_s,
            expected_s
        );
    }

    // -- Test 10: Partial buffer not processed --
    #[test]
    fn test_partial_buffer_not_processed() {
        let mut m = SpectralOccupancyMonitor::new(SAMPLE_RATE, FFT_SIZE);
        m.add_channel(0.0, 50_000.0, "X");
        // Push less than one full FFT window
        let short = make_noise(0.01, FFT_SIZE - 1);
        m.push_samples(&short);
        assert_eq!(m.total_samples, 0); // nothing processed yet
    }

    // -- Test 11: Detection method setter --
    #[test]
    fn test_set_detection_method() {
        let mut m = SpectralOccupancyMonitor::new(SAMPLE_RATE, FFT_SIZE);
        m.set_detection_method(DetectionMethod::CyclostationaryFeature);
        assert_eq!(m.detection_method, DetectionMethod::CyclostationaryFeature);
        m.set_detection_method(DetectionMethod::MatchedFilter);
        assert_eq!(m.detection_method, DetectionMethod::MatchedFilter);
    }

    // -- Test 12: Peak power >= mean power --
    #[test]
    fn test_peak_ge_mean() {
        let mut m = SpectralOccupancyMonitor::new(SAMPLE_RATE, FFT_SIZE);
        m.add_channel(0.0, SAMPLE_RATE, "Wide");
        let samples = make_noise(1.0, FFT_SIZE * 5);
        m.push_samples(&samples);

        let report = m.occupancy_report();
        let ch = &report.channels[0];
        assert!(
            ch.peak_power_dbm >= ch.mean_power_dbm,
            "Peak ({}) should be >= mean ({})",
            ch.peak_power_dbm,
            ch.mean_power_dbm
        );
    }

    // -- Test 13: Busy + idle time equals total duration --
    #[test]
    fn test_busy_plus_idle_eq_duration() {
        let mut m = SpectralOccupancyMonitor::new(SAMPLE_RATE, FFT_SIZE);
        m.add_channel(0.0, SAMPLE_RATE, "Full");
        let samples = make_noise(0.5, FFT_SIZE * 10);
        m.push_samples(&samples);

        let report = m.occupancy_report();
        let ch = &report.channels[0];
        let sum = ch.time_busy_s + ch.time_idle_s;
        assert!(
            (sum - report.measurement_duration_s).abs() < 1e-9,
            "busy({}) + idle({}) = {} vs duration {}",
            ch.time_busy_s,
            ch.time_idle_s,
            sum,
            report.measurement_duration_s
        );
    }

    // -- Test 14: Threshold changes detection --
    #[test]
    fn test_threshold_affects_detection() {
        // With a very high threshold the channel should appear idle
        let mut m_high = SpectralOccupancyMonitor::new(SAMPLE_RATE, FFT_SIZE);
        m_high.add_channel(100_000.0, 20_000.0, "T");
        m_high.set_threshold_db(60.0); // very high

        let tone = make_tone(100_000.0, 1.0, FFT_SIZE * 4, SAMPLE_RATE);
        m_high.push_samples(&tone);
        let dc_high = m_high.channel_duty_cycle(0);

        // With a very low threshold the channel should appear busy
        let mut m_low = SpectralOccupancyMonitor::new(SAMPLE_RATE, FFT_SIZE);
        m_low.add_channel(100_000.0, 20_000.0, "T");
        m_low.set_threshold_db(0.01);

        let tone2 = make_tone(100_000.0, 1.0, FFT_SIZE * 4, SAMPLE_RATE);
        m_low.push_samples(&tone2);
        let dc_low = m_low.channel_duty_cycle(0);

        assert!(
            dc_low >= dc_high,
            "Lower threshold should yield >= duty cycle: low={dc_low}, high={dc_high}"
        );
    }

    // -- Test 15: Multiple channels independent --
    #[test]
    fn test_multiple_channels_independent() {
        let mut m = SpectralOccupancyMonitor::new(SAMPLE_RATE, FFT_SIZE);
        // Channel at 100 kHz and at 400 kHz
        m.add_channel(100_000.0, 20_000.0, "Active");
        m.add_channel(400_000.0, 20_000.0, "Silent");
        m.set_threshold_db(6.0);

        // Only put energy at 100 kHz
        let tone = make_tone(100_000.0, 10.0, FFT_SIZE * 8, SAMPLE_RATE);
        m.push_samples(&tone);

        let dc_active = m.channel_duty_cycle(0);
        let dc_silent = m.channel_duty_cycle(1);

        assert!(
            dc_active > dc_silent,
            "Active channel ({dc_active}) should have higher duty cycle than silent ({dc_silent})"
        );
    }

    // -- Test 16: linear_to_dbm helper --
    #[test]
    fn test_linear_to_dbm() {
        assert!((linear_to_dbm(1.0) - 0.0).abs() < 1e-10);
        assert!((linear_to_dbm(10.0) - 10.0).abs() < 1e-10);
        assert!((linear_to_dbm(0.001) - (-30.0)).abs() < 1e-10);
        assert_eq!(linear_to_dbm(0.0), f64::NEG_INFINITY);
    }

    // -- Test 17: db_to_linear helper --
    #[test]
    fn test_db_to_linear() {
        assert!((db_to_linear(0.0) - 1.0).abs() < 1e-10);
        assert!((db_to_linear(10.0) - 10.0).abs() < 1e-10);
        assert!((db_to_linear(20.0) - 100.0).abs() < 1e-10);
    }

    // -- Test 18: median_of helper --
    #[test]
    fn test_median_of() {
        assert_eq!(median_of(&[]), 0.0);
        assert_eq!(median_of(&[5.0]), 5.0);
        assert!((median_of(&[1.0, 3.0]) - 2.0).abs() < 1e-10);
        assert!((median_of(&[3.0, 1.0, 2.0]) - 2.0).abs() < 1e-10);
    }

    // -- Test 19: Channel definition fields --
    #[test]
    fn test_channel_definition_fields() {
        let mut m = SpectralOccupancyMonitor::new(SAMPLE_RATE, FFT_SIZE);
        m.add_channel(150_000.0, 25_000.0, "Test");
        assert_eq!(m.channels[0].center_freq, 150_000.0);
        assert_eq!(m.channels[0].bandwidth, 25_000.0);
        assert_eq!(m.channels[0].label, "Test");
    }

    // -- Test 20: FFT of pure DC --
    #[test]
    fn test_fft_dc() {
        let m = SpectralOccupancyMonitor::new(SAMPLE_RATE, 8);
        let dc: Vec<(f64, f64)> = vec![(1.0, 0.0); 8];
        let spectrum = m.fft(&dc);
        // Bin 0 should have all the energy
        let mag0 = (spectrum[0].0 * spectrum[0].0 + spectrum[0].1 * spectrum[0].1).sqrt();
        assert!((mag0 - 8.0).abs() < 1e-9, "DC bin magnitude should be 8, got {mag0}");
        // Other bins should be near zero
        for k in 1..8 {
            let mag = (spectrum[k].0 * spectrum[k].0 + spectrum[k].1 * spectrum[k].1).sqrt();
            assert!(mag < 1e-9, "Bin {k} should be ~0, got {mag}");
        }
    }
}
