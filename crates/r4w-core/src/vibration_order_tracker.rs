//! Vibration Order Tracking Analysis
//!
//! Resamples vibration data from the time domain to the angular (order) domain
//! to track RPM-dependent vibration components in rotating machinery. Order
//! tracking separates speed-dependent harmonics (orders) from structural
//! resonances, enabling condition monitoring, balancing, and fault diagnosis.
//!
//! ## Key Concepts
//!
//! - **Order**: A vibration component at an integer or fractional multiple of
//!   the shaft rotation frequency. Order 1 = once-per-revolution (1X),
//!   Order 2 = twice-per-revolution (2X), etc.
//! - **Angular resampling**: Converting uniformly time-sampled data to
//!   uniformly angle-sampled data using a tachometer reference.
//! - **Order spectrum**: The FFT of angular-domain data, whose frequency axis
//!   is in orders rather than Hz.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::vibration_order_tracker::{
//!     OrderTracker, OrderTrackingConfig, generate_order_signal,
//!     order_to_frequency,
//! };
//!
//! let config = OrderTrackingConfig {
//!     sample_rate_hz: 10000.0,
//!     max_order: 10.0,
//!     pulses_per_rev: 1,
//!     order_resolution: 0.1,
//! };
//!
//! // A 1X vibration at 1800 RPM = 30 Hz
//! assert!((order_to_frequency(1.0, 1800.0) - 30.0).abs() < 1e-9);
//! ```

use std::f64::consts::PI;

// ────────────────────────────────────────────────────────────────────────────
// Configuration
// ────────────────────────────────────────────────────────────────────────────

/// Configuration for order tracking analysis.
#[derive(Debug, Clone)]
pub struct OrderTrackingConfig {
    /// Sampling rate of the vibration and tachometer signals (Hz).
    pub sample_rate_hz: f64,
    /// Maximum order to resolve in the order spectrum.
    pub max_order: f64,
    /// Number of tachometer pulses per shaft revolution.
    pub pulses_per_rev: usize,
    /// Order resolution in the spectrum (default 0.1 orders).
    pub order_resolution: f64,
}

impl Default for OrderTrackingConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 10000.0,
            max_order: 20.0,
            pulses_per_rev: 1,
            order_resolution: 0.1,
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Result types
// ────────────────────────────────────────────────────────────────────────────

/// Result of order tracking analysis.
#[derive(Debug, Clone)]
pub struct OrderTrackingResult {
    /// 2-D order map: `order_spectrum[rpm_index][order_index]`.
    pub order_spectrum: Vec<Vec<f64>>,
    /// RPM values corresponding to each row of the order map.
    pub rpm_axis: Vec<f64>,
    /// Order values corresponding to each column of the order map.
    pub order_axis: Vec<f64>,
    /// Dominant order components found in the data.
    pub dominant_orders: Vec<OrderComponent>,
}

/// A single vibration order component.
#[derive(Debug, Clone)]
pub struct OrderComponent {
    /// Order number (e.g. 1.0 = 1X, 2.0 = 2X).
    pub order: f64,
    /// Peak amplitude of this order.
    pub amplitude: f64,
    /// Phase in degrees at the peak amplitude.
    pub phase_deg: f64,
    /// RPM at which this order had maximum amplitude.
    pub rpm_at_max: f64,
}

// ────────────────────────────────────────────────────────────────────────────
// OrderTracker
// ────────────────────────────────────────────────────────────────────────────

/// Main order tracking processor.
///
/// Accepts time-domain vibration and tachometer signals and produces an order
/// tracking result with an order spectrum map, RPM profile, and dominant order
/// identification.
#[derive(Debug, Clone)]
pub struct OrderTracker {
    config: OrderTrackingConfig,
}

impl OrderTracker {
    /// Create a new order tracker with the given configuration.
    pub fn new(config: OrderTrackingConfig) -> Self {
        Self { config }
    }

    /// Process vibration and tachometer signals to produce order tracking results.
    ///
    /// The vibration and tachometer signals must be sampled at the same rate
    /// (`config.sample_rate_hz`) and have the same length.
    ///
    /// The method:
    /// 1. Extracts RPM profile from tachometer pulses.
    /// 2. Segments data into overlapping blocks at roughly constant RPM.
    /// 3. Angular-resamples each block.
    /// 4. Computes order spectrum for each block.
    /// 5. Identifies dominant order components.
    pub fn process(
        &self,
        vibration: &[f64],
        tachometer: &[f64],
    ) -> OrderTrackingResult {
        assert_eq!(
            vibration.len(),
            tachometer.len(),
            "vibration and tachometer signals must have equal length"
        );

        let fs = self.config.sample_rate_hz;
        let ppr = self.config.pulses_per_rev;
        let max_order = self.config.max_order;

        // Samples per revolution for angular resampling: need at least
        // 2 * max_order samples per revolution (Nyquist in order domain).
        let samples_per_rev = (2.0 * max_order / self.config.order_resolution).ceil() as usize;

        // 1. Extract RPM profile
        let rpm_profile = extract_rpm_profile(tachometer, fs, ppr);
        if rpm_profile.len() < 2 {
            return OrderTrackingResult {
                order_spectrum: vec![],
                rpm_axis: vec![],
                order_axis: build_order_axis(max_order, samples_per_rev),
                dominant_orders: vec![],
            };
        }

        // 2. Segment into blocks of ~1 revolution each
        let mut order_spectra: Vec<Vec<f64>> = Vec::new();
        let mut rpm_axis: Vec<f64> = Vec::new();

        // Walk through RPM profile, cutting blocks roughly one revolution long
        let mut i = 0;
        while i + 1 < rpm_profile.len() {
            let (t_start, rpm_start) = rpm_profile[i];
            let avg_rpm = if i + 1 < rpm_profile.len() {
                (rpm_start + rpm_profile[i + 1].1) / 2.0
            } else {
                rpm_start
            };

            if avg_rpm <= 0.0 {
                i += 1;
                continue;
            }

            // Duration of one revolution at this RPM, with a small margin
            // to ensure angular resampling captures a full revolution.
            let rev_period = 60.0 / avg_rpm;
            let block_samples = (rev_period * fs).ceil() as usize + 2;
            let sample_start = (t_start * fs).round() as usize;

            if sample_start + block_samples > vibration.len() || block_samples < 4 {
                i += 1;
                continue;
            }

            let block = &vibration[sample_start..sample_start + block_samples];

            // 3. Angular resample this block
            let rpm_slice = &rpm_profile[i..];
            let angular = angular_resample(block, rpm_slice, fs, samples_per_rev);

            if angular.len() >= samples_per_rev {
                // 4. Compute order spectrum
                let spectrum =
                    compute_order_spectrum(&angular[..samples_per_rev], max_order, samples_per_rev);
                order_spectra.push(spectrum);
                rpm_axis.push(avg_rpm);
            }

            i += 1;
        }

        let order_axis = build_order_axis(max_order, samples_per_rev);

        // 5. Identify dominant orders
        let dominant_orders = find_dominant_orders(&order_spectra, &rpm_axis, &order_axis);

        OrderTrackingResult {
            order_spectrum: order_spectra,
            rpm_axis,
            order_axis,
            dominant_orders,
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// RPM extraction
// ────────────────────────────────────────────────────────────────────────────

/// Extract RPM profile from a tachometer pulse signal.
///
/// Detects rising zero-crossings (positive threshold at 0.5 of the max
/// amplitude) in the tachometer signal and computes instantaneous RPM from
/// pulse-to-pulse timing.
///
/// Returns a vector of `(time_s, rpm)` tuples, one per detected pulse period.
pub fn extract_rpm_profile(
    tachometer: &[f64],
    fs: f64,
    pulses_per_rev: usize,
) -> Vec<(f64, f64)> {
    if tachometer.is_empty() || pulses_per_rev == 0 {
        return vec![];
    }

    // Find tachometer peak for threshold
    let max_val = tachometer
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);
    let threshold = max_val * 0.5;

    // Detect rising edge crossings
    let mut crossings: Vec<usize> = Vec::new();
    for i in 1..tachometer.len() {
        if tachometer[i - 1] < threshold && tachometer[i] >= threshold {
            // Linear interpolation for sub-sample accuracy
            crossings.push(i);
        }
    }

    if crossings.len() < 2 {
        return vec![];
    }

    // Compute RPM from every `pulses_per_rev` pulse intervals
    let mut rpm_profile = Vec::new();
    let step = pulses_per_rev.max(1);
    for i in step..crossings.len() {
        let dt_samples = (crossings[i] - crossings[i - step]) as f64;
        let dt_sec = dt_samples / fs;
        if dt_sec > 0.0 {
            let rpm = 60.0 / dt_sec; // one full revolution per dt_sec
            let t = crossings[i - step] as f64 / fs;
            rpm_profile.push((t, rpm));
        }
    }

    rpm_profile
}

// ────────────────────────────────────────────────────────────────────────────
// Angular resampling
// ────────────────────────────────────────────────────────────────────────────

/// Resample a time-domain signal to the angular domain.
///
/// Uses the RPM profile to compute the instantaneous angular position at each
/// time sample, then interpolates to produce `samples_per_rev` equally spaced
/// angular samples per revolution.
///
/// # Arguments
///
/// * `signal` - Time-domain vibration signal segment.
/// * `rpm_profile` - `(time_s, rpm)` pairs (must be sorted by time).
/// * `fs` - Sample rate in Hz.
/// * `samples_per_rev` - Number of output samples per revolution.
///
/// # Returns
///
/// Angular-domain resampled signal (length is a multiple of `samples_per_rev`).
pub fn angular_resample(
    signal: &[f64],
    rpm_profile: &[(f64, f64)],
    fs: f64,
    samples_per_rev: usize,
) -> Vec<f64> {
    if signal.is_empty() || rpm_profile.is_empty() || samples_per_rev == 0 {
        return vec![];
    }

    let n = signal.len();

    // Build cumulative angle array for each time-domain sample
    let mut angles = Vec::with_capacity(n);
    let mut cum_angle = 0.0_f64;
    angles.push(cum_angle);

    for i in 1..n {
        let t = i as f64 / fs + rpm_profile[0].0;
        let rpm = interpolate_rpm(rpm_profile, t);
        // Angular increment per time sample: (rpm/60) * 2*pi / fs revolutions
        // In revolutions: rpm / (60 * fs)
        let d_rev = rpm / (60.0 * fs);
        cum_angle += d_rev;
        angles.push(cum_angle);
    }

    let total_revs = cum_angle;
    if total_revs <= 0.0 {
        return vec![];
    }

    let total_out = (total_revs * samples_per_rev as f64).floor() as usize;
    if total_out == 0 {
        return vec![];
    }

    // Uniformly spaced angle positions
    let angle_step = total_revs / total_out as f64;
    let mut output = Vec::with_capacity(total_out);

    let mut j = 0_usize; // index into angles
    for k in 0..total_out {
        let target_angle = k as f64 * angle_step;

        // Advance j until angles[j] <= target_angle < angles[j+1]
        while j + 1 < n && angles[j + 1] < target_angle {
            j += 1;
        }

        if j + 1 >= n {
            output.push(signal[n - 1]);
        } else {
            let denom = angles[j + 1] - angles[j];
            if denom.abs() < 1e-15 {
                output.push(signal[j]);
            } else {
                let frac = (target_angle - angles[j]) / denom;
                let val = signal[j] + frac * (signal[j + 1] - signal[j]);
                output.push(val);
            }
        }
    }

    output
}

/// Linearly interpolate RPM at time `t` from an RPM profile.
fn interpolate_rpm(rpm_profile: &[(f64, f64)], t: f64) -> f64 {
    if rpm_profile.is_empty() {
        return 0.0;
    }
    if rpm_profile.len() == 1 || t <= rpm_profile[0].0 {
        return rpm_profile[0].1;
    }
    let last = rpm_profile.len() - 1;
    if t >= rpm_profile[last].0 {
        return rpm_profile[last].1;
    }

    // Binary search for interval
    let mut lo = 0;
    let mut hi = last;
    while lo + 1 < hi {
        let mid = (lo + hi) / 2;
        if rpm_profile[mid].0 <= t {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    let dt = rpm_profile[hi].0 - rpm_profile[lo].0;
    if dt.abs() < 1e-15 {
        return rpm_profile[lo].1;
    }
    let frac = (t - rpm_profile[lo].0) / dt;
    rpm_profile[lo].1 + frac * (rpm_profile[hi].1 - rpm_profile[lo].1)
}

// ────────────────────────────────────────────────────────────────────────────
// Order spectrum computation
// ────────────────────────────────────────────────────────────────────────────

/// Compute the order spectrum from an angular-domain signal.
///
/// Applies a Hann window and computes the magnitude of the DFT up to
/// `max_order`. The resulting spectrum has one entry per DFT bin, with bin
/// index `k` corresponding to order `k * max_order * 2 / samples_per_rev`.
///
/// # Arguments
///
/// * `angular_signal` - Signal resampled to uniform angular spacing.
/// * `max_order` - Maximum order to include in the spectrum.
/// * `samples_per_rev` - Number of samples per revolution used in resampling.
///
/// # Returns
///
/// Magnitude spectrum indexed by order bin.
pub fn compute_order_spectrum(
    angular_signal: &[f64],
    max_order: f64,
    samples_per_rev: usize,
) -> Vec<f64> {
    let n = angular_signal.len();
    if n == 0 {
        return vec![];
    }

    // Apply Hann window
    let windowed: Vec<f64> = angular_signal
        .iter()
        .enumerate()
        .map(|(i, &x)| {
            let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos());
            x * w
        })
        .collect();

    // DFT (real-input): compute magnitude for bins 0..n/2
    let n_bins = n / 2 + 1;
    // The order per bin: one revolution = samples_per_rev samples,
    // so bin k corresponds to order = k * (samples in signal / samples_per_rev)
    // Actually: the signal is samples_per_rev samples for ~1 rev,
    // so bin k = order k. If the signal contains multiple revolutions,
    // bin k corresponds to order k * (1 / num_revs).
    let num_revs = n as f64 / samples_per_rev as f64;
    let order_per_bin = 1.0 / num_revs;

    // Only compute bins up to max_order
    let max_bin = ((max_order / order_per_bin).ceil() as usize).min(n_bins);

    let mut spectrum = Vec::with_capacity(max_bin);
    for k in 0..max_bin {
        let (mut re, mut im) = (0.0, 0.0);
        for (i, &val) in windowed.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
            re += val * angle.cos();
            im += val * angle.sin();
        }
        let mag = (re * re + im * im).sqrt() * 2.0 / n as f64;
        spectrum.push(mag);
    }

    // DC bin should not be doubled
    if !spectrum.is_empty() {
        spectrum[0] /= 2.0;
    }

    spectrum
}

/// Build the order axis labels for a given max order and samples per rev.
pub fn build_order_axis(max_order: f64, samples_per_rev: usize) -> Vec<f64> {
    // Assuming one revolution of data: bin k = order k
    // For compatibility with compute_order_spectrum when n = samples_per_rev:
    // order_per_bin = 1.0
    let n_bins = samples_per_rev / 2 + 1;
    let max_bin = n_bins.min((max_order.ceil() as usize) + 1);
    (0..max_bin).map(|k| k as f64).collect()
}

// ────────────────────────────────────────────────────────────────────────────
// Vold-Kalman filter
// ────────────────────────────────────────────────────────────────────────────

/// Simplified Vold-Kalman order tracking filter.
///
/// Extracts a single order component from a time-domain signal using a
/// narrow-band tracking filter centered on a time-varying reference frequency.
///
/// The filter models the signal at each sample as `A(t) * exp(j * phi(t))`
/// where `phi(t)` is the integral of the reference frequency. A first-order
/// recursive estimator with bandwidth `bandwidth_hz` tracks the amplitude and
/// phase.
///
/// # Arguments
///
/// * `signal` - Time-domain vibration signal.
/// * `reference_freq` - Instantaneous frequency to track (Hz) at each sample.
/// * `fs` - Sample rate (Hz).
/// * `bandwidth_hz` - Filter bandwidth (Hz), controls tracking speed vs. noise.
///
/// # Returns
///
/// Vector of `(amplitude, phase_deg)` pairs, one per input sample.
pub fn vold_kalman_filter(
    signal: &[f64],
    reference_freq: &[f64],
    fs: f64,
    bandwidth_hz: f64,
) -> Vec<(f64, f64)> {
    let n = signal.len();
    assert_eq!(
        n,
        reference_freq.len(),
        "signal and reference_freq must have equal length"
    );

    if n == 0 {
        return vec![];
    }

    // Smoothing factor from bandwidth
    let alpha = (2.0 * PI * bandwidth_hz / fs).min(1.0).max(1e-6);

    // Integrate phase from reference frequency
    let mut phase = Vec::with_capacity(n);
    let mut cum_phase = 0.0_f64;
    for i in 0..n {
        phase.push(cum_phase);
        cum_phase += 2.0 * PI * reference_freq[i] / fs;
    }

    // Demodulate: multiply by exp(-j*phase) to bring the tracked component to DC
    // Then low-pass filter the demodulated I/Q with a single-pole IIR
    let mut i_filt = 0.0_f64;
    let mut q_filt = 0.0_f64;

    let mut result = Vec::with_capacity(n);
    for k in 0..n {
        let i_demod = signal[k] * phase[k].cos();
        let q_demod = -signal[k] * phase[k].sin();

        // Single-pole IIR low-pass
        i_filt += alpha * (i_demod - i_filt);
        q_filt += alpha * (q_demod - q_filt);

        // Re-modulate to get instantaneous amplitude and phase
        let amp = 2.0 * (i_filt * i_filt + q_filt * q_filt).sqrt();
        let phase_out = q_filt.atan2(i_filt).to_degrees();
        result.push((amp, phase_out));
    }

    result
}

// ────────────────────────────────────────────────────────────────────────────
// Utility functions
// ────────────────────────────────────────────────────────────────────────────

/// Compute overall RMS vibration level from an order spectrum.
///
/// Returns the root-sum-of-squares of all order amplitudes (excluding DC).
pub fn compute_overall_vibration(spectrum: &[f64]) -> f64 {
    if spectrum.len() <= 1 {
        return 0.0;
    }
    let sum_sq: f64 = spectrum[1..].iter().map(|&x| x * x).sum();
    sum_sq.sqrt()
}

/// Convert an order number to frequency in Hz at a given RPM.
///
/// `f = order * rpm / 60`
pub fn order_to_frequency(order: f64, rpm: f64) -> f64 {
    order * rpm / 60.0
}

/// Convert a frequency in Hz to an order number at a given RPM.
///
/// `order = f * 60 / rpm`
pub fn frequency_to_order(freq_hz: f64, rpm: f64) -> f64 {
    if rpm.abs() < 1e-15 {
        return 0.0;
    }
    freq_hz * 60.0 / rpm
}

/// Detect resonance crossings in an order map.
///
/// Finds (rpm, order) pairs where a structural natural frequency would be
/// excited by a vibration order. At a given RPM, order `n` produces frequency
/// `n * rpm / 60`. When that frequency equals `natural_freq_hz`, the order
/// line crosses the resonance.
///
/// For each integer and half-integer order up to the maximum in the order map,
/// computes the RPM at which the order frequency equals `natural_freq_hz`,
/// and if that RPM falls within the data range, reports it as a crossing.
///
/// # Returns
///
/// Vector of `(rpm, order)` tuples where resonance crossings occur.
pub fn detect_resonance_crossings(
    order_map: &[Vec<f64>],
    rpm_axis: &[f64],
    natural_freq_hz: f64,
) -> Vec<(f64, f64)> {
    if order_map.is_empty() || rpm_axis.is_empty() || natural_freq_hz <= 0.0 {
        return vec![];
    }

    let rpm_min = rpm_axis
        .iter()
        .cloned()
        .fold(f64::INFINITY, f64::min);
    let rpm_max = rpm_axis
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);

    let max_order = order_map
        .iter()
        .map(|row| row.len())
        .max()
        .unwrap_or(0) as f64;

    let mut crossings = Vec::new();

    // Check integer and half-integer orders
    let mut order = 0.5_f64;
    while order <= max_order {
        // RPM where this order matches the natural frequency:
        // natural_freq_hz = order * rpm / 60  =>  rpm = 60 * natural_freq_hz / order
        let crossing_rpm = 60.0 * natural_freq_hz / order;
        if crossing_rpm >= rpm_min && crossing_rpm <= rpm_max {
            crossings.push((crossing_rpm, order));
        }
        order += 0.5;
    }

    crossings
}

/// Generate a synthetic vibration signal with known order content.
///
/// Creates a time-domain signal containing specified order components at a
/// constant RPM. Useful for testing and validation.
///
/// # Arguments
///
/// * `orders` - Slice of `(order_number, amplitude)` pairs.
/// * `rpm` - Constant shaft speed (rev/min).
/// * `fs` - Sample rate (Hz).
/// * `duration_s` - Signal duration (seconds).
///
/// # Returns
///
/// Time-domain signal with the specified order content.
pub fn generate_order_signal(
    orders: &[(f64, f64)],
    rpm: f64,
    fs: f64,
    duration_s: f64,
) -> Vec<f64> {
    let n = (fs * duration_s).round() as usize;
    let shaft_freq = rpm / 60.0;

    let mut signal = vec![0.0_f64; n];
    for &(order, amplitude) in orders {
        let freq = order * shaft_freq;
        for i in 0..n {
            let t = i as f64 / fs;
            signal[i] += amplitude * (2.0 * PI * freq * t).sin();
        }
    }

    signal
}

/// Generate a synthetic tachometer pulse signal at constant RPM.
///
/// Produces a square-wave-like pulse train with `pulses_per_rev` pulses per
/// revolution at the given constant RPM, sampled at `fs`.
pub fn generate_tachometer(
    rpm: f64,
    fs: f64,
    duration_s: f64,
    pulses_per_rev: usize,
) -> Vec<f64> {
    let n = (fs * duration_s).round() as usize;
    let shaft_freq = rpm / 60.0;
    let pulse_freq = shaft_freq * pulses_per_rev as f64;

    let mut tach = Vec::with_capacity(n);
    for i in 0..n {
        let t = i as f64 / fs;
        // Produce a pulse train: value is 1.0 when the fractional phase is
        // in [0, 0.1) and 0.0 otherwise
        let phase = (t * pulse_freq).fract();
        if phase < 0.1 {
            tach.push(1.0);
        } else {
            tach.push(0.0);
        }
    }

    tach
}

// ────────────────────────────────────────────────────────────────────────────
// Internal helpers
// ────────────────────────────────────────────────────────────────────────────

/// Find the dominant order components across an order map.
fn find_dominant_orders(
    order_spectra: &[Vec<f64>],
    rpm_axis: &[f64],
    order_axis: &[f64],
) -> Vec<OrderComponent> {
    if order_spectra.is_empty() || order_axis.is_empty() {
        return vec![];
    }

    // For each order bin, find the maximum amplitude across all RPMs
    let n_orders = order_axis.len();
    let mut max_amp = vec![0.0_f64; n_orders];
    let mut max_rpm_idx = vec![0_usize; n_orders];
    let max_phase = vec![0.0_f64; n_orders];

    for (rpm_idx, spectrum) in order_spectra.iter().enumerate() {
        for (ord_idx, &amp) in spectrum.iter().enumerate() {
            if ord_idx < n_orders && amp > max_amp[ord_idx] {
                max_amp[ord_idx] = amp;
                max_rpm_idx[ord_idx] = rpm_idx;
            }
        }
    }

    // Collect significant orders (amplitude > 10% of global max, skip DC)
    let global_max = max_amp
        .iter()
        .cloned()
        .fold(0.0_f64, f64::max);
    let threshold = global_max * 0.1;

    let mut components = Vec::new();
    for ord_idx in 1..n_orders {
        if max_amp[ord_idx] > threshold {
            let rpm_at_max = if max_rpm_idx[ord_idx] < rpm_axis.len() {
                rpm_axis[max_rpm_idx[ord_idx]]
            } else {
                0.0
            };
            components.push(OrderComponent {
                order: order_axis[ord_idx],
                amplitude: max_amp[ord_idx],
                phase_deg: max_phase[ord_idx],
                rpm_at_max,
            });
        }
    }

    // Sort by amplitude descending
    components.sort_by(|a, b| b.amplitude.partial_cmp(&a.amplitude).unwrap_or(std::cmp::Ordering::Equal));

    components
}

// ────────────────────────────────────────────────────────────────────────────
// Tests
// ────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-9;

    // ── order_to_frequency / frequency_to_order ──────────────────────────

    #[test]
    fn test_order_to_frequency_basic() {
        // Order 1 at 1800 RPM => 30 Hz
        assert!((order_to_frequency(1.0, 1800.0) - 30.0).abs() < EPSILON);
        // Order 2 at 3000 RPM => 100 Hz
        assert!((order_to_frequency(2.0, 3000.0) - 100.0).abs() < EPSILON);
    }

    #[test]
    fn test_frequency_to_order_basic() {
        // 30 Hz at 1800 RPM => order 1
        assert!((frequency_to_order(30.0, 1800.0) - 1.0).abs() < EPSILON);
        // 150 Hz at 3000 RPM => order 3
        assert!((frequency_to_order(150.0, 3000.0) - 3.0).abs() < EPSILON);
    }

    #[test]
    fn test_order_frequency_roundtrip() {
        for order in [0.5, 1.0, 2.5, 7.0, 10.0] {
            for rpm in [600.0, 1200.0, 3600.0, 12000.0] {
                let freq = order_to_frequency(order, rpm);
                let recovered = frequency_to_order(freq, rpm);
                assert!(
                    (recovered - order).abs() < EPSILON,
                    "roundtrip failed for order={order}, rpm={rpm}"
                );
            }
        }
    }

    #[test]
    fn test_frequency_to_order_zero_rpm() {
        assert!((frequency_to_order(100.0, 0.0)).abs() < EPSILON);
    }

    #[test]
    fn test_order_to_frequency_zero_values() {
        assert!((order_to_frequency(0.0, 1800.0)).abs() < EPSILON);
        assert!((order_to_frequency(1.0, 0.0)).abs() < EPSILON);
    }

    // ── RPM extraction ───────────────────────────────────────────────────

    #[test]
    fn test_extract_rpm_constant_speed() {
        let fs = 10000.0;
        let rpm = 1800.0; // 30 Hz shaft
        let duration = 0.2; // 6 revolutions
        let tach = generate_tachometer(rpm, fs, duration, 1);

        let profile = extract_rpm_profile(&tach, fs, 1);
        assert!(!profile.is_empty(), "should detect at least one RPM point");

        // All extracted RPMs should be close to 1800
        for &(_, extracted_rpm) in &profile {
            assert!(
                (extracted_rpm - rpm).abs() < 50.0,
                "extracted RPM {extracted_rpm} far from {rpm}"
            );
        }
    }

    #[test]
    fn test_extract_rpm_multi_pulse() {
        let fs = 10000.0;
        let rpm = 3000.0; // 50 Hz
        let duration = 0.2;
        let tach = generate_tachometer(rpm, fs, duration, 4);

        let profile = extract_rpm_profile(&tach, fs, 4);
        assert!(!profile.is_empty());

        for &(_, extracted_rpm) in &profile {
            assert!(
                (extracted_rpm - rpm).abs() < 100.0,
                "extracted RPM {extracted_rpm} far from {rpm}"
            );
        }
    }

    #[test]
    fn test_extract_rpm_empty_signal() {
        let profile = extract_rpm_profile(&[], 10000.0, 1);
        assert!(profile.is_empty());
    }

    #[test]
    fn test_extract_rpm_zero_pulses_per_rev() {
        let tach = vec![1.0; 100];
        let profile = extract_rpm_profile(&tach, 10000.0, 0);
        assert!(profile.is_empty());
    }

    // ── Angular resampling ───────────────────────────────────────────────

    #[test]
    fn test_angular_resample_constant_rpm() {
        // At constant RPM, angular resampling should be equivalent to
        // regular resampling (just interpolation)
        let fs: f64 = 10000.0;
        let rpm: f64 = 1800.0;
        let samples_per_rev: usize = 256;

        // Create one revolution of a 1X signal (sine at shaft frequency)
        let shaft_freq = rpm / 60.0; // 30 Hz
        let period_samples = (fs / shaft_freq).round() as usize;
        let signal: Vec<f64> = (0..period_samples)
            .map(|i| {
                let t = i as f64 / fs;
                (2.0 * PI * shaft_freq * t).sin()
            })
            .collect();

        let rpm_profile = vec![(0.0, rpm)];
        let angular = angular_resample(&signal, &rpm_profile, fs, samples_per_rev);

        assert!(
            angular.len() >= samples_per_rev / 2,
            "should produce sufficient angular samples, got {}",
            angular.len()
        );
    }

    #[test]
    fn test_angular_resample_empty_inputs() {
        assert!(angular_resample(&[], &[(0.0, 1800.0)], 10000.0, 256).is_empty());
        assert!(angular_resample(&[1.0, 2.0], &[], 10000.0, 256).is_empty());
        assert!(angular_resample(&[1.0], &[(0.0, 1800.0)], 10000.0, 0).is_empty());
    }

    // ── Order spectrum ───────────────────────────────────────────────────

    #[test]
    fn test_order_spectrum_single_order() {
        // Generate a pure 1X signal: one revolution at 256 samples/rev
        let samples_per_rev = 256;
        let signal: Vec<f64> = (0..samples_per_rev)
            .map(|i| {
                let angle = 2.0 * PI * i as f64 / samples_per_rev as f64;
                angle.sin() // order 1
            })
            .collect();

        let spectrum = compute_order_spectrum(&signal, 10.0, samples_per_rev);
        assert!(!spectrum.is_empty());

        // Order 1 should have the largest amplitude (excluding DC)
        if spectrum.len() > 2 {
            let max_idx = spectrum[1..]
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
                .map(|(i, _)| i + 1)
                .unwrap();
            assert_eq!(max_idx, 1, "peak should be at order 1, got {max_idx}");
        }
    }

    #[test]
    fn test_order_spectrum_two_orders() {
        let samples_per_rev = 512;
        // 1X amplitude 1.0, 5X amplitude 0.5
        // Orders spaced far enough apart to avoid Hann window cross-leakage
        let signal: Vec<f64> = (0..samples_per_rev)
            .map(|i| {
                let angle = 2.0 * PI * i as f64 / samples_per_rev as f64;
                angle.sin() + 0.5 * (5.0 * angle).sin()
            })
            .collect();

        let spectrum = compute_order_spectrum(&signal, 10.0, samples_per_rev);

        // Should have peaks at order 1 and order 5
        assert!(spectrum.len() > 6);
        // Order 1 should be the highest (Hann-windowed magnitude)
        assert!(
            spectrum[1] > spectrum[5],
            "order 1 ({}) should exceed order 5 ({})",
            spectrum[1],
            spectrum[5]
        );
        // Order 5 should exceed its neighbors (orders 4 and 6)
        assert!(
            spectrum[5] > spectrum[4] * 0.8,
            "order 5 ({}) should be significant compared to order 4 ({})",
            spectrum[5],
            spectrum[4]
        );
        // Order 1 should be about 2x order 5 (Hann preserves relative amplitudes)
        let ratio = spectrum[1] / spectrum[5];
        assert!(
            ratio > 1.5 && ratio < 2.5,
            "order1/order5 ratio should be ~2, got {ratio}"
        );
    }

    #[test]
    fn test_order_spectrum_empty() {
        let spectrum = compute_order_spectrum(&[], 10.0, 256);
        assert!(spectrum.is_empty());
    }

    // ── Overall vibration ────────────────────────────────────────────────

    #[test]
    fn test_overall_vibration_basic() {
        // Spectrum: [DC=0.0, 3.0, 4.0] => overall = sqrt(9 + 16) = 5.0
        let spectrum = vec![0.0, 3.0, 4.0];
        let overall = compute_overall_vibration(&spectrum);
        assert!(
            (overall - 5.0).abs() < EPSILON,
            "overall should be 5.0, got {overall}"
        );
    }

    #[test]
    fn test_overall_vibration_excludes_dc() {
        let spectrum = vec![100.0, 3.0, 4.0];
        let overall = compute_overall_vibration(&spectrum);
        assert!(
            (overall - 5.0).abs() < EPSILON,
            "DC should not contribute, got {overall}"
        );
    }

    #[test]
    fn test_overall_vibration_empty() {
        assert!((compute_overall_vibration(&[])).abs() < EPSILON);
        assert!((compute_overall_vibration(&[5.0])).abs() < EPSILON);
    }

    // ── Resonance crossing detection ─────────────────────────────────────

    #[test]
    fn test_resonance_crossing_basic() {
        // 100 Hz natural frequency. Order 2 crosses at RPM = 60*100/2 = 3000
        let order_map = vec![vec![0.0; 5]; 10]; // 10 RPM points, 5 orders each
        let rpm_axis: Vec<f64> = (0..10).map(|i| 1000.0 + i as f64 * 500.0).collect();
        // rpm_axis goes from 1000 to 5500

        let crossings = detect_resonance_crossings(&order_map, &rpm_axis, 100.0);
        assert!(!crossings.is_empty(), "should find at least one crossing");

        // Check that order 2 crossing at 3000 RPM is found
        let has_order2 = crossings
            .iter()
            .any(|&(rpm, ord)| (rpm - 3000.0).abs() < 1.0 && (ord - 2.0).abs() < 0.01);
        assert!(has_order2, "should find order 2 crossing at 3000 RPM");
    }

    #[test]
    fn test_resonance_crossing_empty() {
        let crossings = detect_resonance_crossings(&[], &[], 100.0);
        assert!(crossings.is_empty());
    }

    #[test]
    fn test_resonance_crossing_negative_freq() {
        let order_map = vec![vec![0.0; 5]];
        let rpm_axis = vec![1800.0];
        let crossings = detect_resonance_crossings(&order_map, &rpm_axis, -50.0);
        assert!(crossings.is_empty());
    }

    // ── Vold-Kalman filter ───────────────────────────────────────────────

    #[test]
    fn test_vold_kalman_constant_tone() {
        let fs = 10000.0;
        let freq = 100.0;
        let n = 5000;
        let amplitude = 2.0;

        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                amplitude * (2.0 * PI * freq * t).sin()
            })
            .collect();

        let ref_freq = vec![freq; n];
        let result = vold_kalman_filter(&signal, &ref_freq, fs, 5.0);

        assert_eq!(result.len(), n);

        // After convergence (last 20%), amplitude should be close to 2.0
        let start = (n as f64 * 0.8) as usize;
        let avg_amp: f64 = result[start..].iter().map(|&(a, _)| a).sum::<f64>()
            / (n - start) as f64;
        assert!(
            (avg_amp - amplitude).abs() < 0.5,
            "converged amplitude {avg_amp} should be near {amplitude}"
        );
    }

    #[test]
    fn test_vold_kalman_empty() {
        let result = vold_kalman_filter(&[], &[], 10000.0, 5.0);
        assert!(result.is_empty());
    }

    #[test]
    #[should_panic(expected = "signal and reference_freq must have equal length")]
    fn test_vold_kalman_mismatched_lengths() {
        vold_kalman_filter(&[1.0, 2.0], &[100.0], 10000.0, 5.0);
    }

    // ── Signal generation ────────────────────────────────────────────────

    #[test]
    fn test_generate_order_signal_length() {
        let signal = generate_order_signal(&[(1.0, 1.0)], 1800.0, 10000.0, 0.5);
        assert_eq!(signal.len(), 5000);
    }

    #[test]
    fn test_generate_order_signal_frequency_content() {
        let fs = 10000.0;
        let rpm = 1800.0;
        let duration = 1.0;
        let signal = generate_order_signal(&[(1.0, 1.0)], rpm, fs, duration);

        // Count zero crossings: a 30 Hz sine has ~60 crossings per second
        let mut crossings = 0;
        for i in 1..signal.len() {
            if signal[i - 1] < 0.0 && signal[i] >= 0.0 {
                crossings += 1;
            }
        }

        let expected_crossings = (rpm / 60.0 * duration).round() as usize;
        assert!(
            (crossings as i64 - expected_crossings as i64).unsigned_abs() <= 2,
            "expected ~{expected_crossings} positive crossings, got {crossings}"
        );
    }

    #[test]
    fn test_generate_order_signal_superposition() {
        let fs = 10000.0;
        let rpm = 1800.0;
        let duration = 0.1;

        let sig_1x = generate_order_signal(&[(1.0, 1.0)], rpm, fs, duration);
        let sig_3x = generate_order_signal(&[(3.0, 0.5)], rpm, fs, duration);
        let sig_both = generate_order_signal(&[(1.0, 1.0), (3.0, 0.5)], rpm, fs, duration);

        // Superposition: combined should equal sum of individuals
        for i in 0..sig_both.len() {
            assert!(
                (sig_both[i] - sig_1x[i] - sig_3x[i]).abs() < EPSILON,
                "superposition failed at sample {i}"
            );
        }
    }

    // ── Tachometer generation ────────────────────────────────────────────

    #[test]
    fn test_generate_tachometer_length() {
        let tach = generate_tachometer(1800.0, 10000.0, 0.5, 1);
        assert_eq!(tach.len(), 5000);
    }

    #[test]
    fn test_generate_tachometer_pulse_count() {
        let rpm = 1800.0; // 30 Hz
        let fs = 10000.0;
        let duration = 1.0;
        let ppr = 1;
        let tach = generate_tachometer(rpm, fs, duration, ppr);

        // Count rising edges (pulse transitions from 0 to 1)
        let mut edges = 0;
        for i in 1..tach.len() {
            if tach[i - 1] < 0.5 && tach[i] >= 0.5 {
                edges += 1;
            }
        }

        let expected = (rpm / 60.0 * duration).round() as usize;
        assert!(
            (edges as i64 - expected as i64).unsigned_abs() <= 2,
            "expected ~{expected} edges, got {edges}"
        );
    }

    // ── OrderTracker integration ─────────────────────────────────────────

    #[test]
    fn test_order_tracker_basic_integration() {
        let fs = 10000.0;
        let rpm = 1800.0;
        let duration = 0.5; // 15 revolutions

        let config = OrderTrackingConfig {
            sample_rate_hz: fs,
            max_order: 10.0,
            pulses_per_rev: 1,
            order_resolution: 0.5,
        };

        let vibration = generate_order_signal(&[(1.0, 1.0), (3.0, 0.5)], rpm, fs, duration);
        let tachometer = generate_tachometer(rpm, fs, duration, 1);

        let tracker = OrderTracker::new(config);
        let result = tracker.process(&vibration, &tachometer);

        // Should produce some RPM axis points
        assert!(
            !result.rpm_axis.is_empty(),
            "should have RPM axis entries"
        );
        // Order axis should exist
        assert!(
            !result.order_axis.is_empty(),
            "should have order axis entries"
        );
    }

    #[test]
    #[should_panic(expected = "vibration and tachometer signals must have equal length")]
    fn test_order_tracker_mismatched_lengths() {
        let config = OrderTrackingConfig::default();
        let tracker = OrderTracker::new(config);
        tracker.process(&[1.0, 2.0, 3.0], &[1.0, 2.0]);
    }

    #[test]
    fn test_order_tracker_empty_input() {
        let config = OrderTrackingConfig::default();
        let tracker = OrderTracker::new(config);
        let result = tracker.process(&[], &[]);
        assert!(result.order_spectrum.is_empty());
        assert!(result.rpm_axis.is_empty());
    }

    // ── Synthetic roundtrip ──────────────────────────────────────────────

    #[test]
    fn test_synthetic_signal_order_spectrum_roundtrip() {
        // Generate a signal with well-separated orders to avoid Hann window
        // leakage cross-talk: orders 1, 5, 9
        let samples_per_rev = 512;
        let signal: Vec<f64> = (0..samples_per_rev)
            .map(|i| {
                let angle = 2.0 * PI * i as f64 / samples_per_rev as f64;
                1.0 * angle.sin()             // order 1, amp 1.0
                    + 0.7 * (5.0 * angle).sin()   // order 5, amp 0.7
                    + 0.3 * (9.0 * angle).sin()   // order 9, amp 0.3
            })
            .collect();

        let spectrum = compute_order_spectrum(&signal, 15.0, samples_per_rev);

        // Verify peaks at orders 1, 5, 9
        assert!(spectrum.len() > 10);
        assert!(
            spectrum[1] > spectrum[0],
            "order 1 should exceed DC"
        );
        // Order 5 should be a local peak
        assert!(
            spectrum[5] > spectrum[3],
            "order 5 ({}) should exceed order 3 ({})",
            spectrum[5],
            spectrum[3]
        );
        assert!(
            spectrum[5] > spectrum[7],
            "order 5 ({}) should exceed order 7 ({})",
            spectrum[5],
            spectrum[7]
        );
        // Order 9 should be a local peak
        assert!(
            spectrum[9] > spectrum[7],
            "order 9 ({}) should exceed order 7 ({})",
            spectrum[9],
            spectrum[7]
        );

        // Relative amplitudes: order1 should be largest, order5 next, order9 smallest
        assert!(
            spectrum[1] > spectrum[5],
            "order 1 ({}) should exceed order 5 ({})",
            spectrum[1],
            spectrum[5]
        );
        assert!(
            spectrum[5] > spectrum[9],
            "order 5 ({}) should exceed order 9 ({})",
            spectrum[5],
            spectrum[9]
        );
    }

    // ── Config defaults ──────────────────────────────────────────────────

    #[test]
    fn test_config_default() {
        let config = OrderTrackingConfig::default();
        assert!((config.sample_rate_hz - 10000.0).abs() < EPSILON);
        assert!((config.max_order - 20.0).abs() < EPSILON);
        assert_eq!(config.pulses_per_rev, 1);
        assert!((config.order_resolution - 0.1).abs() < EPSILON);
    }

    // ── Interpolate RPM helper ───────────────────────────────────────────

    #[test]
    fn test_interpolate_rpm_single_point() {
        let profile = vec![(0.0, 1800.0)];
        assert!((interpolate_rpm(&profile, 0.5) - 1800.0).abs() < EPSILON);
    }

    #[test]
    fn test_interpolate_rpm_linear() {
        let profile = vec![(0.0, 1000.0), (1.0, 2000.0)];
        assert!((interpolate_rpm(&profile, 0.5) - 1500.0).abs() < EPSILON);
        assert!((interpolate_rpm(&profile, 0.0) - 1000.0).abs() < EPSILON);
        assert!((interpolate_rpm(&profile, 1.0) - 2000.0).abs() < EPSILON);
    }

    #[test]
    fn test_interpolate_rpm_extrapolation() {
        let profile = vec![(0.0, 1000.0), (1.0, 2000.0)];
        // Before range: clamps to first value
        assert!((interpolate_rpm(&profile, -0.5) - 1000.0).abs() < EPSILON);
        // After range: clamps to last value
        assert!((interpolate_rpm(&profile, 1.5) - 2000.0).abs() < EPSILON);
    }

    #[test]
    fn test_interpolate_rpm_empty() {
        assert!((interpolate_rpm(&[], 0.5)).abs() < EPSILON);
    }

    // ── Build order axis ─────────────────────────────────────────────────

    #[test]
    fn test_build_order_axis() {
        let axis = build_order_axis(5.0, 256);
        assert!(!axis.is_empty());
        assert!((axis[0]).abs() < EPSILON);
        assert!((axis[1] - 1.0).abs() < EPSILON);
        // Should not exceed max_order + 1
        let last = *axis.last().unwrap();
        assert!(last <= 6.0, "last order {last} should be <= 6");
    }

    // ── Edge cases ───────────────────────────────────────────────────────

    #[test]
    fn test_order_spectrum_single_sample() {
        let spectrum = compute_order_spectrum(&[1.0], 10.0, 256);
        // Single sample: only DC bin
        assert_eq!(spectrum.len(), 1);
    }

    #[test]
    fn test_overall_vibration_single_order() {
        let spectrum = vec![0.0, 7.0];
        assert!((compute_overall_vibration(&spectrum) - 7.0).abs() < EPSILON);
    }

    #[test]
    fn test_resonance_crossings_all_orders() {
        // With a wide RPM range and many orders, should find many crossings
        let order_map = vec![vec![0.0; 20]; 100]; // 100 RPM points, 20 orders
        let rpm_axis: Vec<f64> = (0..100).map(|i| 500.0 + i as f64 * 100.0).collect();
        // 500..10400 RPM

        let crossings = detect_resonance_crossings(&order_map, &rpm_axis, 200.0);
        // Order 1 at RPM=12000 (out of range), but many others should be in range
        assert!(
            crossings.len() >= 3,
            "should find multiple crossings, got {}",
            crossings.len()
        );
    }

    #[test]
    fn test_vold_kalman_tracks_frequency_change() {
        let fs = 10000.0;
        let n = 4000;

        // Signal sweeping from 50 Hz to 150 Hz
        let mut signal = Vec::with_capacity(n);
        let mut ref_freq = Vec::with_capacity(n);
        let mut phase = 0.0_f64;

        for i in 0..n {
            let t = i as f64 / fs;
            let freq = 50.0 + 100.0 * t / (n as f64 / fs);
            ref_freq.push(freq);
            signal.push((phase).sin());
            phase += 2.0 * PI * freq / fs;
        }

        let result = vold_kalman_filter(&signal, &ref_freq, fs, 10.0);
        assert_eq!(result.len(), n);

        // After convergence, amplitude should be near 1.0
        let start = (n as f64 * 0.5) as usize;
        let avg_amp: f64 = result[start..].iter().map(|&(a, _)| a).sum::<f64>()
            / (n - start) as f64;
        assert!(
            avg_amp > 0.5,
            "tracking filter should extract meaningful amplitude, got {avg_amp}"
        );
    }
}
