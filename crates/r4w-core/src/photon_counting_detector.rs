//! Single-photon counting detector signal processor for SPAD arrays and PMT systems.
//!
//! This module implements the core signal processing algorithms used in photon
//! counting detection systems based on Single-Photon Avalanche Diodes (SPADs)
//! and Photomultiplier Tubes (PMTs). It provides dead time correction, photon
//! statistics analysis, time-correlated single photon counting (TCSPC), and
//! multi-channel coincidence detection.
//!
//! # Background
//!
//! Photon counting detectors convert individual photon arrivals into discrete
//! electrical pulses. After each detection event, the detector enters a "dead
//! time" period during which it cannot register another photon. At high count
//! rates this leads to systematic undercounting that must be corrected.
//!
//! The statistical properties of detected photons reveal the quantum nature
//! of the light source. Coherent light (laser) follows Poisson statistics,
//! thermal light shows super-Poisson bunching, and quantum light sources
//! can exhibit sub-Poisson antibunching — a purely quantum effect.
//!
//! # Components
//!
//! | Struct / Function | Purpose |
//! |---|---|
//! | [`PhotonEvent`] | Single photon detection event with timestamp and channel |
//! | [`PhotonCountingDetector`] | Detector model with dead time, dark counts, efficiency |
//! | [`DeadTimeModel`] | Non-paralyzable (Type I) and paralyzable (Type II) correction |
//! | [`PhotonStatistics`] | Poisson/super-Poisson/sub-Poisson classification |
//! | [`TcspcHistogram`] | Time-correlated single photon counting histogram |
//! | [`CoincidenceCounter`] | Multi-channel coincidence detection |
//! | [`PhotonNumberResolver`] | Threshold-based multi-photon discrimination |
//! | [`g2_correlation`] | Second-order correlation function g²(τ) |
//! | [`mandel_q_parameter`] | Mandel Q parameter for photon statistics |
//! | [`afterpulse_probability`] | After-pulsing estimation from inter-arrival times |
//!
//! # Example
//!
//! ```rust
//! use r4w_core::photon_counting_detector::{
//!     PhotonEvent, PhotonCountingDetector, DeadTimeModel,
//!     correct_dead_time_nonparalyzable, mandel_q_parameter,
//! };
//!
//! // Configure a SPAD detector with 50 ns dead time
//! let detector = PhotonCountingDetector {
//!     dead_time_ns: 50.0,
//!     timing_resolution_ps: 100.0,
//!     dark_count_rate_hz: 100.0,
//!     detection_efficiency: 0.25,
//!     dead_time_model: DeadTimeModel::NonParalyzable,
//! };
//!
//! // Correct measured count rate for dead time losses
//! let measured_rate = 1_000_000.0; // 1 MHz measured
//! let true_rate = detector.correct_count_rate(measured_rate);
//! assert!(true_rate > measured_rate);
//!
//! // Check photon statistics (Poisson => Q ≈ 0)
//! let counts = vec![10, 11, 9, 10, 12, 8, 10, 11, 9, 10];
//! let q = mandel_q_parameter(&counts);
//! assert!(q.abs() < 2.0); // near-Poisson
//! ```

use std::f64::consts::PI;

// ─── Data types ──────────────────────────────────────────────────────────────

/// A single photon detection event.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PhotonEvent {
    /// Timestamp of detection in nanoseconds.
    pub timestamp_ns: f64,
    /// Detector channel index (0-based).
    pub channel: usize,
    /// Pulse amplitude (arbitrary units, 1.0 = single photon nominal).
    pub amplitude: f64,
}

impl PhotonEvent {
    /// Create a new photon event.
    pub fn new(timestamp_ns: f64, channel: usize, amplitude: f64) -> Self {
        Self { timestamp_ns, channel, amplitude }
    }

    /// Create a single-photon event on channel 0.
    pub fn single(timestamp_ns: f64) -> Self {
        Self { timestamp_ns, channel: 0, amplitude: 1.0 }
    }
}

/// Dead time behaviour model.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeadTimeModel {
    /// Type I: detector is insensitive for a fixed duration after each
    /// *detected* event. Subsequent photons during dead time are simply lost.
    NonParalyzable,
    /// Type II: each photon arrival (detected or not) restarts the dead time
    /// window. At very high rates the detector can become completely paralyzed.
    Paralyzable,
}

/// Classification of photon number statistics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhotonStatistics {
    /// Variance equals mean (coherent / laser light).
    Poisson,
    /// Variance exceeds mean (thermal / chaotic light, bunching).
    SuperPoisson,
    /// Variance less than mean (quantum light, antibunching).
    SubPoisson,
}

// ─── Detector ────────────────────────────────────────────────────────────────

/// Configuration and processing for a single-photon counting detector.
#[derive(Debug, Clone)]
pub struct PhotonCountingDetector {
    /// Dead time duration in nanoseconds.
    pub dead_time_ns: f64,
    /// Timing resolution in picoseconds (e.g. 100 ps for a typical TCSPC card).
    pub timing_resolution_ps: f64,
    /// Intrinsic dark count rate in Hz.
    pub dark_count_rate_hz: f64,
    /// Quantum detection efficiency (0.0 .. 1.0).
    pub detection_efficiency: f64,
    /// Dead time behaviour.
    pub dead_time_model: DeadTimeModel,
}

impl PhotonCountingDetector {
    /// Create a detector with typical SPAD parameters.
    pub fn spad_default() -> Self {
        Self {
            dead_time_ns: 50.0,
            timing_resolution_ps: 100.0,
            dark_count_rate_hz: 100.0,
            detection_efficiency: 0.25,
            dead_time_model: DeadTimeModel::NonParalyzable,
        }
    }

    /// Create a detector with typical PMT parameters.
    pub fn pmt_default() -> Self {
        Self {
            dead_time_ns: 10.0,
            timing_resolution_ps: 50.0,
            dark_count_rate_hz: 10.0,
            detection_efficiency: 0.15,
            dead_time_model: DeadTimeModel::NonParalyzable,
        }
    }

    /// Correct a measured count rate (Hz) for dead time losses.
    pub fn correct_count_rate(&self, measured_rate_hz: f64) -> f64 {
        let tau = self.dead_time_ns * 1e-9; // convert to seconds
        match self.dead_time_model {
            DeadTimeModel::NonParalyzable => {
                correct_dead_time_nonparalyzable(measured_rate_hz, tau)
            }
            DeadTimeModel::Paralyzable => {
                correct_dead_time_paralyzable(measured_rate_hz, tau)
            }
        }
    }

    /// Subtract dark counts from a measured rate, returning the signal rate.
    pub fn subtract_dark_counts(&self, measured_rate_hz: f64) -> f64 {
        (measured_rate_hz - self.dark_count_rate_hz).max(0.0)
    }

    /// Convert measured rate to estimated true photon rate, applying dead time
    /// correction, dark count subtraction, and detection efficiency scaling.
    pub fn true_photon_rate(&self, measured_rate_hz: f64) -> f64 {
        let corrected = self.correct_count_rate(measured_rate_hz);
        let signal = self.subtract_dark_counts(corrected);
        if self.detection_efficiency > 0.0 {
            signal / self.detection_efficiency
        } else {
            0.0
        }
    }

    /// Filter photon events to remove dead-time-suppressed events.
    /// Returns events that would actually be detected (non-paralyzable model).
    pub fn apply_dead_time(&self, events: &[PhotonEvent]) -> Vec<PhotonEvent> {
        if events.is_empty() {
            return Vec::new();
        }
        let mut result = Vec::new();
        let mut last_detected_time = f64::NEG_INFINITY;

        match self.dead_time_model {
            DeadTimeModel::NonParalyzable => {
                for ev in events {
                    if ev.timestamp_ns - last_detected_time >= self.dead_time_ns {
                        result.push(*ev);
                        last_detected_time = ev.timestamp_ns;
                    }
                }
            }
            DeadTimeModel::Paralyzable => {
                // Each arrival resets the dead window; only detect if the
                // gap since *any* arrival exceeds dead time.
                let mut last_arrival_time = f64::NEG_INFINITY;
                for ev in events {
                    if ev.timestamp_ns - last_arrival_time >= self.dead_time_ns {
                        result.push(*ev);
                    }
                    last_arrival_time = ev.timestamp_ns;
                }
            }
        }
        result
    }

    /// Maximum detectable count rate (Hz) before saturation.
    /// For non-paralyzable: 1/tau. For paralyzable: 1/(tau*e).
    pub fn max_count_rate(&self) -> f64 {
        let tau = self.dead_time_ns * 1e-9;
        if tau <= 0.0 {
            return f64::INFINITY;
        }
        match self.dead_time_model {
            DeadTimeModel::NonParalyzable => 1.0 / tau,
            DeadTimeModel::Paralyzable => 1.0 / (tau * std::f64::consts::E),
        }
    }
}

// ─── Dead time correction functions ──────────────────────────────────────────

/// Non-paralyzable (Type I) dead time correction.
///
/// N_true = N_measured / (1 - N_measured * tau)
///
/// Returns the estimated true count rate given the measured rate and dead time.
pub fn correct_dead_time_nonparalyzable(measured_rate: f64, dead_time_s: f64) -> f64 {
    let denom = 1.0 - measured_rate * dead_time_s;
    if denom <= 0.0 {
        // Saturated — return a very large rate
        return measured_rate / 1e-12;
    }
    measured_rate / denom
}

/// Paralyzable (Type II) dead time correction via Newton's method.
///
/// Solves: N_true * exp(-N_true * tau) = N_measured
///
/// Returns the estimated true count rate.
pub fn correct_dead_time_paralyzable(measured_rate: f64, dead_time_s: f64) -> f64 {
    if dead_time_s <= 0.0 || measured_rate <= 0.0 {
        return measured_rate;
    }

    // Newton iteration on f(n) = n * exp(-n * tau) - m = 0
    // f'(n) = exp(-n*tau) * (1 - n*tau)
    let m = measured_rate;
    let tau = dead_time_s;

    // Initial guess: non-paralyzable approximation
    let denom = 1.0 - m * tau;
    let mut n = if denom > 0.1 { m / denom } else { m * 2.0 };

    for _ in 0..100 {
        let e = (-n * tau).exp();
        let f = n * e - m;
        let fp = e * (1.0 - n * tau);

        if fp.abs() < 1e-30 {
            break;
        }

        let step = f / fp;
        n -= step;

        // Guard against negative
        if n < m {
            n = m;
        }

        if step.abs() < 1e-6 * n {
            break;
        }
    }
    n
}

// ─── Photon statistics ───────────────────────────────────────────────────────

/// Compute the Mandel Q parameter from photon count bins.
///
/// Q = (var(n) - mean(n)) / mean(n)
///
/// - Q = 0  → Poisson (coherent)
/// - Q > 0  → super-Poisson (bunched / thermal)
/// - Q < 0  → sub-Poisson (antibunched / quantum)
pub fn mandel_q_parameter(counts: &[u64]) -> f64 {
    if counts.is_empty() {
        return 0.0;
    }
    let n = counts.len() as f64;
    let mean = counts.iter().map(|&c| c as f64).sum::<f64>() / n;
    if mean <= 0.0 {
        return 0.0;
    }
    let variance = counts.iter().map(|&c| {
        let d = c as f64 - mean;
        d * d
    }).sum::<f64>() / n;
    (variance - mean) / mean
}

/// Classify photon statistics from the Mandel Q parameter.
pub fn classify_statistics(q: f64, threshold: f64) -> PhotonStatistics {
    if q > threshold {
        PhotonStatistics::SuperPoisson
    } else if q < -threshold {
        PhotonStatistics::SubPoisson
    } else {
        PhotonStatistics::Poisson
    }
}

/// Compute Fano factor F = var(n) / mean(n).
///
/// F = 1 for Poisson, F > 1 for super-Poisson, F < 1 for sub-Poisson.
pub fn fano_factor(counts: &[u64]) -> f64 {
    if counts.is_empty() {
        return 1.0;
    }
    let n = counts.len() as f64;
    let mean = counts.iter().map(|&c| c as f64).sum::<f64>() / n;
    if mean <= 0.0 {
        return 1.0;
    }
    let variance = counts.iter().map(|&c| {
        let d = c as f64 - mean;
        d * d
    }).sum::<f64>() / n;
    variance / mean
}

/// Compute the second-order correlation function g²(τ) from photon timestamps.
///
/// g²(τ) = <n(t) n(t+τ)> / <n(t)>²
///
/// This computes a histogram of inter-arrival time differences and normalizes
/// to give g²(τ) at each time bin.
///
/// - `timestamps_ns`: sorted photon arrival times in nanoseconds
/// - `bin_width_ns`: width of each histogram bin
/// - `max_tau_ns`: maximum delay to compute
///
/// Returns (tau_values, g2_values) where tau_values are bin centres.
pub fn g2_correlation(
    timestamps_ns: &[f64],
    bin_width_ns: f64,
    max_tau_ns: f64,
) -> (Vec<f64>, Vec<f64>) {
    let n_bins = (max_tau_ns / bin_width_ns).ceil() as usize;
    if n_bins == 0 || timestamps_ns.len() < 2 {
        return (Vec::new(), Vec::new());
    }

    let mut histogram = vec![0u64; n_bins];
    let n = timestamps_ns.len();

    // Count coincidences in each delay bin
    for i in 0..n {
        for j in (i + 1)..n {
            let dt = timestamps_ns[j] - timestamps_ns[i];
            if dt >= max_tau_ns {
                break;
            }
            let bin = (dt / bin_width_ns) as usize;
            if bin < n_bins {
                histogram[bin] += 1;
            }
        }
    }

    // Normalize: g2(tau) = coincidences / (rate^2 * T * dt)
    let t_total = timestamps_ns[n - 1] - timestamps_ns[0];
    if t_total <= 0.0 {
        return (Vec::new(), Vec::new());
    }
    let rate = n as f64 / t_total;

    let tau_values: Vec<f64> = (0..n_bins)
        .map(|i| (i as f64 + 0.5) * bin_width_ns)
        .collect();

    let g2_values: Vec<f64> = histogram
        .iter()
        .map(|&count| {
            let expected = rate * rate * t_total * bin_width_ns;
            if expected > 0.0 {
                count as f64 / expected
            } else {
                0.0
            }
        })
        .collect();

    (tau_values, g2_values)
}

// ─── TCSPC (Time-Correlated Single Photon Counting) ─────────────────────────

/// TCSPC histogram builder and lifetime analyzer.
#[derive(Debug, Clone)]
pub struct TcspcHistogram {
    /// Histogram bin counts.
    pub bins: Vec<u64>,
    /// Bin width in nanoseconds.
    pub bin_width_ns: f64,
    /// Total number of start-stop events accumulated.
    pub total_events: u64,
}

impl TcspcHistogram {
    /// Create an empty histogram with the given number of bins and bin width.
    pub fn new(n_bins: usize, bin_width_ns: f64) -> Self {
        Self {
            bins: vec![0; n_bins],
            bin_width_ns,
            total_events: 0,
        }
    }

    /// Add a start-stop time difference (in nanoseconds) to the histogram.
    pub fn add_event(&mut self, dt_ns: f64) {
        if dt_ns < 0.0 {
            return;
        }
        let bin = (dt_ns / self.bin_width_ns) as usize;
        if bin < self.bins.len() {
            self.bins[bin] += 1;
            self.total_events += 1;
        }
    }

    /// Add multiple start-stop differences.
    pub fn add_events(&mut self, dts_ns: &[f64]) {
        for &dt in dts_ns {
            self.add_event(dt);
        }
    }

    /// Bin centre times in nanoseconds.
    pub fn bin_centres(&self) -> Vec<f64> {
        (0..self.bins.len())
            .map(|i| (i as f64 + 0.5) * self.bin_width_ns)
            .collect()
    }

    /// Fit a single-exponential decay: I(t) = A * exp(-t / tau) + B
    ///
    /// Uses linearized least-squares on log(I - B) with iterative background
    /// refinement. The background is estimated from the minimum of the last
    /// bins, then the fit is refined.
    ///
    /// Returns (amplitude_A, lifetime_tau_ns, background_B).
    pub fn fit_exponential(&self) -> (f64, f64, f64) {
        if self.bins.is_empty() || self.total_events == 0 {
            return (0.0, 0.0, 0.0);
        }

        // Find peak bin
        let peak_bin = self.bins.iter()
            .enumerate()
            .max_by_key(|(_, &c)| c)
            .map(|(i, _)| i)
            .unwrap_or(0);

        let peak_time = (peak_bin as f64 + 0.5) * self.bin_width_ns;

        // Estimate background: use the minimum of the last 10% of bins.
        // This avoids over-estimating background when the tail still has
        // residual exponential signal.
        let tail_start = (self.bins.len() as f64 * 0.9) as usize;
        let tail_bins = &self.bins[tail_start..];
        let background = if !tail_bins.is_empty() {
            tail_bins.iter().map(|&c| c as f64).fold(f64::MAX, f64::min)
        } else {
            0.0
        };

        // Linearized least squares: ln(I - B) = ln(A) - t/tau
        // Only use bins from peak onwards where signal is above background.
        let fit = |bg: f64| -> (f64, f64, f64) {
            let mut sum_x = 0.0f64;
            let mut sum_y = 0.0f64;
            let mut sum_xx = 0.0f64;
            let mut sum_xy = 0.0f64;
            let mut n_fit = 0.0f64;

            for i in peak_bin..self.bins.len() {
                let val = self.bins[i] as f64 - bg;
                if val > 1.0 {
                    let t = (i as f64 + 0.5) * self.bin_width_ns - peak_time;
                    let ln_val = val.ln();
                    sum_x += t;
                    sum_y += ln_val;
                    sum_xx += t * t;
                    sum_xy += t * ln_val;
                    n_fit += 1.0;
                }
            }

            if n_fit >= 2.0 {
                let denom = n_fit * sum_xx - sum_x * sum_x;
                if denom.abs() > 1e-30 {
                    let slope = (n_fit * sum_xy - sum_x * sum_y) / denom;
                    let intercept = (sum_y - slope * sum_x) / n_fit;
                    let tau = if slope < 0.0 { -1.0 / slope } else { 1.0 };
                    let amp = intercept.exp();
                    return (amp, tau, bg);
                }
            }

            // Fallback: method of moments
            let mut sum_t = 0.0f64;
            let mut sum_w = 0.0f64;
            for i in peak_bin..self.bins.len() {
                let t = (i as f64 + 0.5) * self.bin_width_ns - peak_time;
                let w = (self.bins[i] as f64 - bg).max(0.0);
                sum_t += t * w;
                sum_w += w;
            }
            let tau = if sum_w > 0.0 { sum_t / sum_w } else { 1.0 };
            let amp = (self.bins[peak_bin] as f64 - bg).max(0.0);
            (amp, tau, bg)
        };

        fit(background)
    }

    /// Deconvolve the instrument response function (IRF) from this histogram
    /// using iterative reconvolution (expectation maximization).
    ///
    /// `irf` is the measured IRF histogram (same bin width).
    /// `iterations` is the number of EM iterations.
    ///
    /// Returns the deconvolved decay curve as f64 bin values.
    pub fn deconvolve_irf(&self, irf: &[f64], iterations: usize) -> Vec<f64> {
        let n = self.bins.len();
        if n == 0 || irf.is_empty() {
            return Vec::new();
        }

        let data: Vec<f64> = self.bins.iter().map(|&c| c as f64).collect();

        // Normalize IRF
        let irf_sum: f64 = irf.iter().sum();
        let irf_norm: Vec<f64> = if irf_sum > 0.0 {
            irf.iter().map(|&v| v / irf_sum).collect()
        } else {
            return data;
        };

        // Initial guess: flat or copy of data
        let mut decay = vec![1.0f64; n];
        let total: f64 = data.iter().sum();
        if total > 0.0 {
            for d in &mut decay {
                *d = total / n as f64;
            }
        }

        let irf_len = irf_norm.len();

        for _ in 0..iterations {
            // Forward convolution: model[i] = sum_j decay[i-j] * irf[j]
            let model = convolve_truncated(&decay, &irf_norm, n);

            // Compute correction ratio
            let mut correction = vec![1.0f64; n];
            for i in 0..n {
                if model[i] > 1e-30 {
                    correction[i] = data[i] / model[i];
                }
            }

            // Back-project: update[i] = sum_j correction[i+j] * irf[j]
            // This is the cross-correlation of correction with irf.
            let mut update = vec![0.0f64; n];
            for i in 0..n {
                let mut s = 0.0;
                for j in 0..irf_len {
                    if i + j < n {
                        s += correction[i + j] * irf_norm[j];
                    }
                }
                update[i] = s;
            }

            // Multiply decay by update
            for i in 0..n {
                decay[i] *= update[i];
                if decay[i] < 0.0 {
                    decay[i] = 0.0;
                }
            }
        }

        decay
    }
}

/// Truncated convolution: output[k] = sum_j a[k-j] * b[j], length = n.
fn convolve_truncated(a: &[f64], b: &[f64], n: usize) -> Vec<f64> {
    let mut out = vec![0.0f64; n];
    let na = a.len();
    let nb = b.len();
    for k in 0..n {
        let j_start = if k + 1 >= na { k + 1 - na } else { 0 };
        let j_end = nb.min(k + 1);
        for j in j_start..j_end {
            out[k] += a[k - j] * b[j];
        }
    }
    out
}

// ─── Coincidence counting ────────────────────────────────────────────────────

/// Multi-channel coincidence counter.
#[derive(Debug, Clone)]
pub struct CoincidenceCounter {
    /// Coincidence time window in nanoseconds.
    pub window_ns: f64,
    /// Number of channels required for a coincidence (typically 2).
    pub required_channels: usize,
}

impl CoincidenceCounter {
    /// Create a two-channel coincidence counter.
    pub fn two_channel(window_ns: f64) -> Self {
        Self { window_ns, required_channels: 2 }
    }

    /// Find coincidence events from sorted, multi-channel photon events.
    ///
    /// Returns groups of events that are within the coincidence window
    /// and span at least `required_channels` distinct channels.
    pub fn find_coincidences(&self, events: &[PhotonEvent]) -> Vec<Vec<PhotonEvent>> {
        if events.is_empty() {
            return Vec::new();
        }

        let mut coincidences = Vec::new();
        let n = events.len();

        for i in 0..n {
            let mut group = vec![events[i]];
            let mut channels = 1u64 << events[i].channel;
            let mut n_channels = 1usize;

            for j in (i + 1)..n {
                if events[j].timestamp_ns - events[i].timestamp_ns > self.window_ns {
                    break;
                }
                let ch_bit = 1u64 << events[j].channel;
                if channels & ch_bit == 0 {
                    n_channels += 1;
                    channels |= ch_bit;
                }
                group.push(events[j]);
            }

            if n_channels >= self.required_channels {
                coincidences.push(group);
            }
        }

        coincidences
    }

    /// Count total coincidences between two channels.
    pub fn count_coincidences_two_channel(
        &self,
        ch0_timestamps_ns: &[f64],
        ch1_timestamps_ns: &[f64],
    ) -> u64 {
        let mut count = 0u64;
        let mut j_start = 0usize;

        for &t0 in ch0_timestamps_ns {
            // Advance j_start to the first ch1 event within window
            while j_start < ch1_timestamps_ns.len()
                && ch1_timestamps_ns[j_start] < t0 - self.window_ns
            {
                j_start += 1;
            }

            let mut j = j_start;
            while j < ch1_timestamps_ns.len()
                && ch1_timestamps_ns[j] <= t0 + self.window_ns
            {
                count += 1;
                j += 1;
            }
        }
        count
    }

    /// Estimate the accidental coincidence rate.
    ///
    /// R_acc = 2 * tau_window * R1 * R2
    pub fn accidental_rate(&self, rate1_hz: f64, rate2_hz: f64) -> f64 {
        let tau = self.window_ns * 1e-9;
        2.0 * tau * rate1_hz * rate2_hz
    }
}

// ─── After-pulsing ───────────────────────────────────────────────────────────

/// Estimate after-pulsing probability from photon event inter-arrival times.
///
/// After-pulses appear as an excess of short inter-arrival times above
/// what is expected from Poisson statistics. The probability is estimated
/// as the fraction of inter-arrival times below a characteristic time
/// threshold, minus the expected Poisson fraction.
///
/// - `timestamps_ns`: sorted photon arrival times
/// - `afterpulse_window_ns`: maximum inter-arrival time considered as after-pulse
///
/// Returns the estimated after-pulsing probability (0.0 to 1.0).
pub fn afterpulse_probability(timestamps_ns: &[f64], afterpulse_window_ns: f64) -> f64 {
    if timestamps_ns.len() < 2 {
        return 0.0;
    }

    let n = timestamps_ns.len();

    // Compute inter-arrival times
    let mut inter_arrivals = Vec::with_capacity(n - 1);
    for i in 1..n {
        inter_arrivals.push(timestamps_ns[i] - timestamps_ns[i - 1]);
    }

    let total = inter_arrivals.len() as f64;
    if total == 0.0 {
        return 0.0;
    }

    // Count events within afterpulse window
    let n_short = inter_arrivals.iter()
        .filter(|&&dt| dt < afterpulse_window_ns)
        .count() as f64;

    // Expected fraction for Poisson process: 1 - exp(-rate * window)
    let mean_iat = inter_arrivals.iter().sum::<f64>() / total;
    let rate = if mean_iat > 0.0 { 1.0 / mean_iat } else { return 0.0 };
    let expected_fraction = 1.0 - (-rate * afterpulse_window_ns).exp();

    // After-pulse probability = excess fraction
    let measured_fraction = n_short / total;
    (measured_fraction - expected_fraction).max(0.0)
}

/// Compute the inter-arrival time histogram.
///
/// Returns (bin_centres_ns, counts) for visualization or further analysis.
pub fn inter_arrival_histogram(
    timestamps_ns: &[f64],
    bin_width_ns: f64,
    max_time_ns: f64,
) -> (Vec<f64>, Vec<u64>) {
    let n_bins = (max_time_ns / bin_width_ns).ceil() as usize;
    let mut histogram = vec![0u64; n_bins];

    for i in 1..timestamps_ns.len() {
        let dt = timestamps_ns[i] - timestamps_ns[i - 1];
        if dt >= 0.0 && dt < max_time_ns {
            let bin = (dt / bin_width_ns) as usize;
            if bin < n_bins {
                histogram[bin] += 1;
            }
        }
    }

    let centres: Vec<f64> = (0..n_bins)
        .map(|i| (i as f64 + 0.5) * bin_width_ns)
        .collect();

    (centres, histogram)
}

// ─── Photon number resolving ─────────────────────────────────────────────────

/// Photon number resolver using amplitude thresholds.
///
/// Discriminates between 0, 1, 2, ... photon events based on pulse
/// amplitude relative to configurable thresholds.
#[derive(Debug, Clone)]
pub struct PhotonNumberResolver {
    /// Amplitude thresholds. thresholds[i] is the minimum amplitude for
    /// (i+1) photons. E.g. thresholds = [0.5, 1.5, 2.5] discriminates
    /// 0/1/2/3 photon events.
    pub thresholds: Vec<f64>,
}

impl PhotonNumberResolver {
    /// Create a resolver with evenly spaced thresholds.
    ///
    /// `max_photons` = maximum photon number to resolve.
    /// `single_photon_amplitude` = nominal amplitude for a single photon.
    pub fn uniform(max_photons: usize, single_photon_amplitude: f64) -> Self {
        let thresholds = (0..max_photons)
            .map(|i| (i as f64 + 0.5) * single_photon_amplitude)
            .collect();
        Self { thresholds }
    }

    /// Create a resolver with explicit thresholds.
    pub fn with_thresholds(thresholds: Vec<f64>) -> Self {
        Self { thresholds }
    }

    /// Resolve the number of photons from a pulse amplitude.
    pub fn resolve(&self, amplitude: f64) -> usize {
        let mut n = 0;
        for &thr in &self.thresholds {
            if amplitude >= thr {
                n += 1;
            } else {
                break;
            }
        }
        n
    }

    /// Resolve photon numbers for a batch of events.
    pub fn resolve_events(&self, events: &[PhotonEvent]) -> Vec<usize> {
        events.iter().map(|ev| self.resolve(ev.amplitude)).collect()
    }

    /// Compute the photon number distribution from a set of amplitudes.
    ///
    /// Returns a vector where index i gives the count of i-photon events.
    pub fn photon_number_distribution(&self, amplitudes: &[f64]) -> Vec<u64> {
        let max_n = self.thresholds.len() + 1;
        let mut distribution = vec![0u64; max_n];
        for &a in amplitudes {
            let n = self.resolve(a);
            if n < max_n {
                distribution[n] += 1;
            }
        }
        distribution
    }
}

// ─── Poisson distribution utilities ──────────────────────────────────────────

/// Compute the Poisson probability P(k; lambda) = lambda^k * exp(-lambda) / k!
pub fn poisson_pmf(k: u64, lambda: f64) -> f64 {
    if lambda <= 0.0 {
        return if k == 0 { 1.0 } else { 0.0 };
    }
    // Use log to avoid overflow: ln(P) = k*ln(lambda) - lambda - ln(k!)
    let log_p = k as f64 * lambda.ln() - lambda - ln_factorial(k);
    log_p.exp()
}

/// Natural log of factorial: ln(n!)
fn ln_factorial(n: u64) -> f64 {
    if n <= 1 {
        return 0.0;
    }
    // Use Stirling for large n, direct sum for small n
    if n <= 20 {
        let mut result = 0.0f64;
        for i in 2..=n {
            result += (i as f64).ln();
        }
        result
    } else {
        // Stirling's approximation
        let nf = n as f64;
        0.5 * (2.0 * PI * nf).ln() + nf * (nf.ln() - 1.0) + 1.0 / (12.0 * nf)
    }
}

/// Generate Poisson-distributed random counts using the inverse transform method.
/// Uses a simple LCG PRNG with the given seed.
pub fn poisson_random(lambda: f64, count: usize, seed: u64) -> Vec<u64> {
    let mut rng = LcgRng::new(seed);
    let mut result = Vec::with_capacity(count);

    for _ in 0..count {
        let l = (-lambda).exp();
        let mut k = 0u64;
        let mut p = 1.0f64;
        loop {
            k += 1;
            p *= rng.next_f64();
            if p <= l {
                break;
            }
        }
        result.push(k - 1);
    }

    result
}

/// Simple LCG PRNG (for reproducible test data, not cryptographic use).
struct LcgRng {
    state: u64,
}

impl LcgRng {
    fn new(seed: u64) -> Self {
        Self { state: seed.wrapping_add(1) }
    }

    fn next_u64(&mut self) -> u64 {
        // LCG constants from Knuth
        self.state = self.state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        self.state
    }

    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// Generate exponentially distributed inter-arrival times (for simulating
/// a Poisson process).
pub fn exponential_inter_arrivals(rate_hz: f64, count: usize, seed: u64) -> Vec<f64> {
    let mut rng = LcgRng::new(seed);
    let mut result = Vec::with_capacity(count);
    for _ in 0..count {
        let u = rng.next_f64();
        if u > 0.0 {
            // Convert to nanoseconds
            let dt_ns = -1e9 * u.ln() / rate_hz;
            result.push(dt_ns);
        }
    }
    result
}

/// Generate sorted photon timestamps from a Poisson process.
pub fn generate_poisson_timestamps(rate_hz: f64, count: usize, seed: u64) -> Vec<f64> {
    let iats = exponential_inter_arrivals(rate_hz, count, seed);
    let mut timestamps = Vec::with_capacity(count);
    let mut t = 0.0;
    for dt in iats {
        t += dt;
        timestamps.push(t);
    }
    timestamps
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    // --- PhotonEvent ---

    #[test]
    fn test_photon_event_new() {
        let ev = PhotonEvent::new(100.0, 2, 0.95);
        assert!((ev.timestamp_ns - 100.0).abs() < EPSILON);
        assert_eq!(ev.channel, 2);
        assert!((ev.amplitude - 0.95).abs() < EPSILON);
    }

    #[test]
    fn test_photon_event_single() {
        let ev = PhotonEvent::single(42.0);
        assert_eq!(ev.channel, 0);
        assert!((ev.amplitude - 1.0).abs() < EPSILON);
    }

    // --- Dead time correction: non-paralyzable ---

    #[test]
    fn test_nonparalyzable_low_rate() {
        // At low rates, correction should be small
        let tau = 50e-9; // 50 ns
        let measured = 1000.0; // 1 kHz — negligible dead time loss
        let true_rate = correct_dead_time_nonparalyzable(measured, tau);
        // Expected: 1000 / (1 - 1000 * 50e-9) = 1000 / 0.99995 ≈ 1000.05
        assert!((true_rate - measured).abs() < 1.0);
        assert!(true_rate > measured);
    }

    #[test]
    fn test_nonparalyzable_high_rate() {
        // At higher rates, correction is significant
        let tau = 50e-9;
        let measured = 10_000_000.0; // 10 MHz
        let true_rate = correct_dead_time_nonparalyzable(measured, tau);
        // N_true = 10e6 / (1 - 10e6 * 50e-9) = 10e6 / 0.5 = 20e6
        assert!((true_rate - 20_000_000.0).abs() < 1.0);
    }

    #[test]
    fn test_nonparalyzable_saturated() {
        // When measured * tau >= 1, detector is saturated
        let tau = 50e-9;
        let measured = 20_000_001.0; // just over 1/tau
        let true_rate = correct_dead_time_nonparalyzable(measured, tau);
        // Should return a very large value
        assert!(true_rate > 1e15);
    }

    // --- Dead time correction: paralyzable ---

    #[test]
    fn test_paralyzable_low_rate() {
        let tau = 50e-9;
        let measured = 1000.0;
        let true_rate = correct_dead_time_paralyzable(measured, tau);
        // At low rates, paralyzable ≈ non-paralyzable
        assert!((true_rate - measured).abs() < 1.0);
    }

    #[test]
    fn test_paralyzable_moderate_rate() {
        let tau = 50e-9;
        // For N_true = 5e6: measured = 5e6 * exp(-5e6 * 50e-9) = 5e6 * exp(-0.25)
        let n_true_expected = 5_000_000.0;
        let measured = n_true_expected * ((-n_true_expected * tau) as f64).exp();
        let recovered = correct_dead_time_paralyzable(measured, tau);
        assert!((recovered - n_true_expected).abs() / n_true_expected < 0.01);
    }

    #[test]
    fn test_paralyzable_zero_rate() {
        let result = correct_dead_time_paralyzable(0.0, 50e-9);
        assert!((result - 0.0).abs() < EPSILON);
    }

    // --- Detector ---

    #[test]
    fn test_detector_spad_default() {
        let det = PhotonCountingDetector::spad_default();
        assert!((det.dead_time_ns - 50.0).abs() < EPSILON);
        assert!((det.detection_efficiency - 0.25).abs() < EPSILON);
    }

    #[test]
    fn test_detector_pmt_default() {
        let det = PhotonCountingDetector::pmt_default();
        assert!((det.dead_time_ns - 10.0).abs() < EPSILON);
    }

    #[test]
    fn test_detector_correct_count_rate_nonparalyzable() {
        let det = PhotonCountingDetector {
            dead_time_ns: 100.0,
            timing_resolution_ps: 100.0,
            dark_count_rate_hz: 0.0,
            detection_efficiency: 1.0,
            dead_time_model: DeadTimeModel::NonParalyzable,
        };
        let measured = 5_000_000.0;
        let corrected = det.correct_count_rate(measured);
        // tau = 100e-9, N/(1 - N*tau) = 5e6/(1-0.5) = 10e6
        assert!((corrected - 10_000_000.0).abs() < 1.0);
    }

    #[test]
    fn test_detector_subtract_dark_counts() {
        let det = PhotonCountingDetector {
            dead_time_ns: 50.0,
            timing_resolution_ps: 100.0,
            dark_count_rate_hz: 500.0,
            detection_efficiency: 0.5,
            dead_time_model: DeadTimeModel::NonParalyzable,
        };
        assert!((det.subtract_dark_counts(1000.0) - 500.0).abs() < EPSILON);
        assert!((det.subtract_dark_counts(100.0) - 0.0).abs() < EPSILON); // clamped
    }

    #[test]
    fn test_detector_true_photon_rate() {
        let det = PhotonCountingDetector {
            dead_time_ns: 0.0, // no dead time for simplicity
            timing_resolution_ps: 100.0,
            dark_count_rate_hz: 100.0,
            detection_efficiency: 0.5,
            dead_time_model: DeadTimeModel::NonParalyzable,
        };
        // measured=1100, no dead time corr, subtract 100 dark => 1000, /0.5 => 2000
        let rate = det.true_photon_rate(1100.0);
        assert!((rate - 2000.0).abs() < 1.0);
    }

    #[test]
    fn test_detector_max_count_rate_nonparalyzable() {
        let det = PhotonCountingDetector::spad_default();
        // 1/(50e-9) = 20 MHz
        assert!((det.max_count_rate() - 20_000_000.0).abs() < 1.0);
    }

    #[test]
    fn test_detector_max_count_rate_paralyzable() {
        let mut det = PhotonCountingDetector::spad_default();
        det.dead_time_model = DeadTimeModel::Paralyzable;
        // 1/(50e-9 * e) ≈ 7.358e6
        let expected = 1.0 / (50e-9 * std::f64::consts::E);
        assert!((det.max_count_rate() - expected).abs() / expected < 1e-6);
    }

    // --- Dead time event filtering ---

    #[test]
    fn test_apply_dead_time_nonparalyzable() {
        let det = PhotonCountingDetector {
            dead_time_ns: 10.0,
            timing_resolution_ps: 100.0,
            dark_count_rate_hz: 0.0,
            detection_efficiency: 1.0,
            dead_time_model: DeadTimeModel::NonParalyzable,
        };
        let events = vec![
            PhotonEvent::single(0.0),
            PhotonEvent::single(5.0),   // within dead time, lost
            PhotonEvent::single(10.0),  // exactly at dead time boundary
            PhotonEvent::single(25.0),  // detected
        ];
        let detected = det.apply_dead_time(&events);
        assert_eq!(detected.len(), 3); // events at 0, 10, 25
        assert!((detected[0].timestamp_ns - 0.0).abs() < EPSILON);
        assert!((detected[1].timestamp_ns - 10.0).abs() < EPSILON);
        assert!((detected[2].timestamp_ns - 25.0).abs() < EPSILON);
    }

    #[test]
    fn test_apply_dead_time_paralyzable() {
        let det = PhotonCountingDetector {
            dead_time_ns: 10.0,
            timing_resolution_ps: 100.0,
            dark_count_rate_hz: 0.0,
            detection_efficiency: 1.0,
            dead_time_model: DeadTimeModel::Paralyzable,
        };
        let events = vec![
            PhotonEvent::single(0.0),
            PhotonEvent::single(5.0),   // resets dead time, previous lost — but first was detected
            PhotonEvent::single(12.0),  // only 7 ns after last arrival (5.0), lost and resets
            PhotonEvent::single(25.0),  // 13 ns after last arrival (12.0), detected
        ];
        let detected = det.apply_dead_time(&events);
        // Event at 0: detected (gap from -inf is > 10)
        // Event at 5: gap from 0 (last arrival) = 5 < 10, not detected, last_arrival = 5
        // Event at 12: gap from 5 = 7 < 10, not detected, last_arrival = 12
        // Event at 25: gap from 12 = 13 >= 10, detected
        assert_eq!(detected.len(), 2);
        assert!((detected[0].timestamp_ns - 0.0).abs() < EPSILON);
        assert!((detected[1].timestamp_ns - 25.0).abs() < EPSILON);
    }

    #[test]
    fn test_apply_dead_time_empty() {
        let det = PhotonCountingDetector::spad_default();
        let detected = det.apply_dead_time(&[]);
        assert!(detected.is_empty());
    }

    // --- Mandel Q parameter ---

    #[test]
    fn test_mandel_q_poisson() {
        // For Poisson: var = mean => Q = 0
        // Use exact Poisson samples: mean=10
        let counts: Vec<u64> = vec![10, 10, 10, 10, 10]; // zero variance
        let q = mandel_q_parameter(&counts);
        // var=0, mean=10 => Q = (0-10)/10 = -1
        assert!((q - (-1.0)).abs() < EPSILON);

        // More realistic Poisson-like data
        let counts2 = poisson_random(10.0, 10000, 12345);
        let q2 = mandel_q_parameter(&counts2);
        assert!(q2.abs() < 1.0); // should be near zero
    }

    #[test]
    fn test_mandel_q_super_poisson() {
        // Super-Poisson: variance >> mean => Q > 0
        // Alternating very high and very low counts
        let counts: Vec<u64> = vec![0, 20, 0, 20, 0, 20, 0, 20];
        let q = mandel_q_parameter(&counts);
        assert!(q > 0.0);
    }

    #[test]
    fn test_mandel_q_empty() {
        let q = mandel_q_parameter(&[]);
        assert!((q - 0.0).abs() < EPSILON);
    }

    // --- Fano factor ---

    #[test]
    fn test_fano_factor_poisson_like() {
        let counts = poisson_random(20.0, 10000, 99);
        let f = fano_factor(&counts);
        // Should be close to 1.0 for Poisson
        assert!((f - 1.0).abs() < 0.2);
    }

    #[test]
    fn test_fano_factor_super_poisson() {
        let counts: Vec<u64> = vec![0, 50, 0, 50, 0, 50];
        let f = fano_factor(&counts);
        assert!(f > 1.0);
    }

    // --- Classify statistics ---

    #[test]
    fn test_classify_statistics() {
        assert_eq!(classify_statistics(0.05, 0.1), PhotonStatistics::Poisson);
        assert_eq!(classify_statistics(0.5, 0.1), PhotonStatistics::SuperPoisson);
        assert_eq!(classify_statistics(-0.5, 0.1), PhotonStatistics::SubPoisson);
    }

    // --- g2 correlation ---

    #[test]
    fn test_g2_coherent_light() {
        // Poisson process => g2(tau) ≈ 1.0 for all tau
        let timestamps = generate_poisson_timestamps(1_000_000.0, 5000, 42);
        let (taus, g2) = g2_correlation(&timestamps, 10.0, 500.0);

        assert!(!taus.is_empty());
        assert_eq!(taus.len(), g2.len());

        // Average g2 should be near 1.0 for coherent (Poisson) light
        let mean_g2 = g2.iter().sum::<f64>() / g2.len() as f64;
        assert!((mean_g2 - 1.0).abs() < 0.3, "mean g2 = {}", mean_g2);
    }

    #[test]
    fn test_g2_empty() {
        let (taus, g2) = g2_correlation(&[], 10.0, 100.0);
        assert!(taus.is_empty());
        assert!(g2.is_empty());
    }

    #[test]
    fn test_g2_two_events() {
        let (taus, g2) = g2_correlation(&[0.0, 50.0], 10.0, 100.0);
        assert!(!taus.is_empty());
        // Should have a peak at tau ≈ 50
        let peak_bin = (50.0 / 10.0) as usize; // bin 5
        if peak_bin < g2.len() {
            assert!(g2[peak_bin] > 0.0);
        }
    }

    // --- TCSPC ---

    #[test]
    fn test_tcspc_histogram_add_events() {
        let mut hist = TcspcHistogram::new(100, 0.5);
        hist.add_event(1.0);   // bin 2
        hist.add_event(1.25);  // bin 2
        hist.add_event(5.0);   // bin 10
        hist.add_event(-1.0);  // rejected (negative)
        hist.add_event(999.0); // rejected (out of range)

        assert_eq!(hist.total_events, 3);
        assert_eq!(hist.bins[2], 2);
        assert_eq!(hist.bins[10], 1);
    }

    #[test]
    fn test_tcspc_bin_centres() {
        let hist = TcspcHistogram::new(5, 2.0);
        let centres = hist.bin_centres();
        assert_eq!(centres.len(), 5);
        assert!((centres[0] - 1.0).abs() < EPSILON);
        assert!((centres[1] - 3.0).abs() < EPSILON);
        assert!((centres[4] - 9.0).abs() < EPSILON);
    }

    #[test]
    fn test_tcspc_exponential_fit() {
        // Generate synthetic exponential decay: I(t) = 10000 * exp(-t/5.0) + 5
        // Use enough bins so the tail reaches background level (>6*tau)
        let n_bins = 500;
        let bin_width = 0.1; // ns → total range = 50 ns = 10*tau
        let tau_true = 5.0;
        let amp_true = 10000.0;
        let bg_true = 5.0;

        let mut hist = TcspcHistogram::new(n_bins, bin_width);
        for i in 0..n_bins {
            let t = (i as f64 + 0.5) * bin_width;
            let val = (amp_true * (-t / tau_true).exp() + bg_true) as u64;
            hist.bins[i] = val;
            hist.total_events += val;
        }

        let (_amp, tau, _bg) = hist.fit_exponential();
        assert!((tau - tau_true).abs() / tau_true < 0.1,
                "tau={}, expected={}", tau, tau_true);
    }

    #[test]
    fn test_tcspc_fit_empty() {
        let hist = TcspcHistogram::new(0, 1.0);
        let (a, t, b) = hist.fit_exponential();
        assert!((a - 0.0).abs() < EPSILON);
        assert!((t - 0.0).abs() < EPSILON);
        assert!((b - 0.0).abs() < EPSILON);
    }

    // --- IRF deconvolution ---

    #[test]
    fn test_deconvolve_irf_basic() {
        // Create a narrow gaussian IRF centered at bin 2
        let n = 64;
        let bin_width = 1.0;
        let mut irf = vec![0.0f64; n];
        for i in 0..n {
            let t = i as f64 - 2.0;
            irf[i] = (-t * t / 2.0).exp();
        }

        // Create a pure exponential decay
        let tau = 15.0;
        let mut decay_true = vec![0.0f64; n];
        for i in 0..n {
            decay_true[i] = (-(i as f64) / tau).exp() * 1000.0;
        }
        let convolved = convolve_truncated(&decay_true, &irf, n);

        // Build histogram from convolved data
        let mut hist = TcspcHistogram::new(n, bin_width);
        for i in 0..n {
            hist.bins[i] = convolved[i].round().max(0.0) as u64;
            hist.total_events += hist.bins[i];
        }

        // Deconvolve with 200 iterations
        let recovered = hist.deconvolve_irf(&irf, 200);
        assert_eq!(recovered.len(), n);

        // The recovered should be monotonically decreasing overall
        // (exponential shape). Check that early bins > later bins on average.
        let first_quarter_sum: f64 = recovered[0..n / 4].iter().sum();
        let third_quarter_sum: f64 = recovered[n / 2..3 * n / 4].iter().sum();
        assert!(first_quarter_sum > third_quarter_sum,
                "first quarter sum={} should be > third quarter sum={}",
                first_quarter_sum, third_quarter_sum);
    }

    // --- Coincidence counting ---

    #[test]
    fn test_coincidence_two_channel_simple() {
        let cc = CoincidenceCounter::two_channel(5.0);
        let events = vec![
            PhotonEvent::new(10.0, 0, 1.0),
            PhotonEvent::new(12.0, 1, 1.0), // within 5 ns of ch0 event
            PhotonEvent::new(100.0, 0, 1.0),
            PhotonEvent::new(200.0, 1, 1.0), // too far from any ch0 event
        ];
        let coincs = cc.find_coincidences(&events);
        // Events at 10,12 form a coincidence (2 channels within window)
        assert!(!coincs.is_empty());
        // First coincidence group should include events at t=10 and t=12
        assert!(coincs[0].len() >= 2);
    }

    #[test]
    fn test_coincidence_count_two_channel() {
        let cc = CoincidenceCounter::two_channel(5.0);
        let ch0 = vec![10.0, 50.0, 100.0];
        let ch1 = vec![12.0, 80.0, 102.0];
        let count = cc.count_coincidences_two_channel(&ch0, &ch1);
        // 10 ↔ 12 (dt=2 < 5): yes
        // 50 ↔ 80 (dt=30 > 5): no
        // 100 ↔ 102 (dt=2 < 5): yes
        assert_eq!(count, 2);
    }

    #[test]
    fn test_accidental_coincidence_rate() {
        let cc = CoincidenceCounter::two_channel(10.0); // 10 ns window
        let rate = cc.accidental_rate(1_000_000.0, 1_000_000.0);
        // R_acc = 2 * 10e-9 * 1e6 * 1e6 = 2 * 10e-9 * 1e12 = 20000 Hz
        assert!((rate - 20_000.0).abs() < 0.01);
    }

    // --- After-pulsing ---

    #[test]
    fn test_afterpulse_probability_poisson() {
        // Pure Poisson process should show near-zero after-pulsing
        let timestamps = generate_poisson_timestamps(100_000.0, 10000, 777);
        let ap = afterpulse_probability(&timestamps, 100.0); // 100 ns window
        assert!(ap < 0.05, "afterpulse prob = {} (expected near 0)", ap);
    }

    #[test]
    fn test_afterpulse_probability_with_afterpulses() {
        // Simulate after-pulses by adding closely-spaced events
        let mut timestamps = generate_poisson_timestamps(100_000.0, 1000, 888);
        let n_orig = timestamps.len();
        // Add after-pulses: 50% of events get a follow-up within 50 ns
        let mut after = Vec::new();
        for i in (0..n_orig).step_by(2) {
            after.push(timestamps[i] + 20.0); // 20 ns after-pulse
        }
        timestamps.extend(after);
        timestamps.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let ap = afterpulse_probability(&timestamps, 50.0);
        assert!(ap > 0.01, "afterpulse prob = {} (expected > 0)", ap);
    }

    #[test]
    fn test_afterpulse_empty() {
        assert!((afterpulse_probability(&[], 100.0) - 0.0).abs() < EPSILON);
        assert!((afterpulse_probability(&[1.0], 100.0) - 0.0).abs() < EPSILON);
    }

    // --- Inter-arrival histogram ---

    #[test]
    fn test_inter_arrival_histogram() {
        let timestamps = vec![0.0, 10.0, 15.0, 30.0, 35.0];
        let (centres, hist) = inter_arrival_histogram(&timestamps, 5.0, 25.0);
        assert_eq!(centres.len(), 5);
        // IATs: 10, 5, 15, 5
        // bin 0 [0,5): none (5.0 is not < 5.0)
        // bin 1 [5,10): 5, 5 => 2
        // bin 2 [10,15): 10 => 1
        // bin 3 [15,20): 15 => 1
        // bin 4 [20,25): none
        assert_eq!(hist[0], 0);
        assert_eq!(hist[1], 2);
        assert_eq!(hist[2], 1);
        assert_eq!(hist[3], 1);
        assert_eq!(hist[4], 0);
    }

    // --- Photon number resolver ---

    #[test]
    fn test_photon_number_resolver_uniform() {
        let pnr = PhotonNumberResolver::uniform(4, 1.0);
        assert_eq!(pnr.resolve(0.3), 0);  // below 0.5 threshold
        assert_eq!(pnr.resolve(0.7), 1);  // above 0.5, below 1.5
        assert_eq!(pnr.resolve(1.6), 2);  // above 1.5, below 2.5
        assert_eq!(pnr.resolve(2.8), 3);  // above 2.5, below 3.5
        assert_eq!(pnr.resolve(3.6), 4);  // above 3.5
    }

    #[test]
    fn test_photon_number_resolver_custom_thresholds() {
        let pnr = PhotonNumberResolver::with_thresholds(vec![0.3, 1.2, 2.8]);
        assert_eq!(pnr.resolve(0.1), 0);
        assert_eq!(pnr.resolve(0.5), 1);
        assert_eq!(pnr.resolve(1.5), 2);
        assert_eq!(pnr.resolve(3.0), 3);
    }

    #[test]
    fn test_photon_number_distribution() {
        let pnr = PhotonNumberResolver::uniform(3, 1.0);
        let amplitudes = vec![0.2, 0.8, 1.0, 1.7, 2.0, 2.6, 0.1];
        let dist = pnr.photon_number_distribution(&amplitudes);
        // 0.2 => 0, 0.8 => 1, 1.0 => 1, 1.7 => 2, 2.0 => 2, 2.6 => 3, 0.1 => 0
        assert_eq!(dist[0], 2); // 0.2, 0.1
        assert_eq!(dist[1], 2); // 0.8, 1.0
        assert_eq!(dist[2], 2); // 1.7, 2.0
        assert_eq!(dist[3], 1); // 2.6
    }

    #[test]
    fn test_resolve_events() {
        let pnr = PhotonNumberResolver::uniform(3, 1.0);
        let events = vec![
            PhotonEvent::new(0.0, 0, 0.3),
            PhotonEvent::new(1.0, 0, 1.2),
            PhotonEvent::new(2.0, 0, 2.7),
        ];
        let resolved = pnr.resolve_events(&events);
        assert_eq!(resolved, vec![0, 1, 3]);
    }

    // --- Poisson distribution ---

    #[test]
    fn test_poisson_pmf() {
        // P(0; 5) = exp(-5) ≈ 0.006738
        let p0 = poisson_pmf(0, 5.0);
        assert!((p0 - (-5.0f64).exp()).abs() < 1e-6);

        // P(5; 5) = 5^5 * exp(-5) / 120 ≈ 0.17547
        let p5 = poisson_pmf(5, 5.0);
        assert!((p5 - 0.17547).abs() < 0.001);

        // Sum should be near 1
        let total: f64 = (0..30).map(|k| poisson_pmf(k, 5.0)).sum();
        assert!((total - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_poisson_pmf_zero_lambda() {
        assert!((poisson_pmf(0, 0.0) - 1.0).abs() < EPSILON);
        assert!((poisson_pmf(1, 0.0) - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_poisson_random_mean() {
        let lambda = 10.0;
        let samples = poisson_random(lambda, 10000, 54321);
        let mean = samples.iter().map(|&c| c as f64).sum::<f64>() / samples.len() as f64;
        assert!((mean - lambda).abs() < 0.5, "mean={}, expected={}", mean, lambda);
    }

    // --- Timestamp generation ---

    #[test]
    fn test_generate_poisson_timestamps_sorted() {
        let ts = generate_poisson_timestamps(1_000_000.0, 100, 42);
        assert_eq!(ts.len(), 100);
        for i in 1..ts.len() {
            assert!(ts[i] > ts[i - 1], "timestamps must be strictly increasing");
        }
    }

    #[test]
    fn test_generate_poisson_timestamps_rate() {
        let rate = 1_000_000.0; // 1 MHz
        let n = 10000;
        let ts = generate_poisson_timestamps(rate, n, 123);
        let duration_s = (ts[n - 1] - ts[0]) * 1e-9;
        let measured_rate = (n - 1) as f64 / duration_s;
        // Should be within 10% of expected rate
        assert!((measured_rate - rate).abs() / rate < 0.1,
                "measured rate = {}, expected = {}", measured_rate, rate);
    }

    // --- Exponential inter-arrivals ---

    #[test]
    fn test_exponential_inter_arrivals_positive() {
        let iats = exponential_inter_arrivals(1_000_000.0, 100, 7);
        assert_eq!(iats.len(), 100);
        for &dt in &iats {
            assert!(dt > 0.0, "inter-arrival times must be positive");
        }
    }

    // --- Convolve truncated ---

    #[test]
    fn test_convolve_truncated_identity() {
        let a = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let b = vec![1.0]; // identity
        let result = convolve_truncated(&a, &b, 5);
        for i in 0..5 {
            assert!((result[i] - a[i]).abs() < EPSILON);
        }
    }

    #[test]
    fn test_convolve_truncated_shift() {
        let a = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let b = vec![0.0, 1.0]; // delay by 1
        let result = convolve_truncated(&a, &b, 5);
        assert!((result[0] - 0.0).abs() < EPSILON);
        assert!((result[1] - 1.0).abs() < EPSILON);
        assert!((result[2] - 2.0).abs() < EPSILON);
    }

    // --- ln_factorial ---

    #[test]
    fn test_ln_factorial() {
        assert!((ln_factorial(0) - 0.0).abs() < EPSILON);
        assert!((ln_factorial(1) - 0.0).abs() < EPSILON);
        assert!((ln_factorial(5) - (120.0f64).ln()).abs() < 1e-10);
        // For large n, check Stirling's approximation
        let lf20_direct = (1..=20).map(|i| (i as f64).ln()).sum::<f64>();
        assert!((ln_factorial(20) - lf20_direct).abs() < 1e-10);
        // n=25 uses Stirling
        let lf25 = ln_factorial(25);
        assert!(lf25 > 0.0);
    }

    // --- Dead time model enum ---

    #[test]
    fn test_dead_time_model_eq() {
        assert_eq!(DeadTimeModel::NonParalyzable, DeadTimeModel::NonParalyzable);
        assert_ne!(DeadTimeModel::NonParalyzable, DeadTimeModel::Paralyzable);
    }

    // --- Edge cases ---

    #[test]
    fn test_detector_zero_efficiency() {
        let det = PhotonCountingDetector {
            dead_time_ns: 0.0,
            timing_resolution_ps: 100.0,
            dark_count_rate_hz: 0.0,
            detection_efficiency: 0.0,
            dead_time_model: DeadTimeModel::NonParalyzable,
        };
        assert!((det.true_photon_rate(1000.0) - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_coincidence_no_events() {
        let cc = CoincidenceCounter::two_channel(5.0);
        let coincs = cc.find_coincidences(&[]);
        assert!(coincs.is_empty());
    }

    #[test]
    fn test_fano_factor_empty() {
        assert!((fano_factor(&[]) - 1.0).abs() < EPSILON);
    }
}
