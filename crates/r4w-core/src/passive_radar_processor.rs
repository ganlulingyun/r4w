//! Passive/bistatic radar processing using illuminators of opportunity.
//!
//! This module implements passive radar signal processing for exploiting
//! broadcast transmitters (FM radio, DVB-T, cellular, WiFi) as illuminators
//! of opportunity. It provides:
//!
//! - **Reference signal extraction** from the direct-path illuminator
//! - **Surveillance channel processing** for target echoes
//! - **Cross-Ambiguity Function (CAF)** computation for range-Doppler mapping
//! - **Direct Signal Interference (DSI) cancellation** via LMS adaptive filtering
//! - **CA-CFAR target detection** on the range-Doppler map
//! - **Bistatic geometry** calculations (range, velocity, resolution)
//! - **Processing gain** estimation
//!
//! # Example
//!
//! ```rust
//! use r4w_core::passive_radar_processor::{
//!     PassiveRadarConfig, PassiveRadarProcessor,
//!     cross_ambiguity_function, cancel_dsi, detect_targets_cfar,
//!     bistatic_range, bistatic_velocity, compute_snr_improvement,
//!     range_resolution,
//! };
//!
//! let config = PassiveRadarConfig {
//!     sample_rate_hz: 1e6,
//!     caf_doppler_bins: 64,
//!     caf_range_bins: 128,
//!     dsi_filter_taps: 32,
//! };
//!
//! // Create a simple reference signal (tone)
//! let n = 256;
//! let reference: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / config.sample_rate_hz;
//!         let phase = 2.0 * std::f64::consts::PI * 1000.0 * t;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! // Surveillance channel with a delayed, Doppler-shifted copy
//! let delay = 5;
//! let doppler_hz = 50.0;
//! let mut surveillance: Vec<(f64, f64)> = vec![(0.0, 0.0); n];
//! for i in delay..n {
//!     let t = i as f64 / config.sample_rate_hz;
//!     let phase_shift = 2.0 * std::f64::consts::PI * doppler_hz * t;
//!     let (re, im) = reference[i - delay];
//!     let (cs, sn) = (phase_shift.cos(), phase_shift.sin());
//!     surveillance[i] = (re * cs - im * sn, re * sn + im * cs);
//! }
//!
//! // Compute cross-ambiguity function
//! let caf = cross_ambiguity_function(&reference, &surveillance, 20, 200.0, 32, config.sample_rate_hz);
//! assert!(caf.map.len() > 0);
//! assert!(caf.range_resolution_m > 0.0);
//!
//! // Detect targets
//! let targets = detect_targets_cfar(&caf.map, 2, 4, 3.0);
//!
//! // Bistatic geometry
//! let range_m = bistatic_range(delay, config.sample_rate_hz);
//! assert!(range_m > 0.0);
//!
//! let res = range_resolution(1e6);
//! assert!(res > 0.0);
//!
//! let gain = compute_snr_improvement(0.01, 1e6);
//! assert!(gain > 0.0);
//! ```

use std::f64::consts::PI;

/// Speed of light in m/s.
const SPEED_OF_LIGHT: f64 = 299_792_458.0;

// ── Complex arithmetic helpers ──────────────────────────────────────────────

/// Multiply two complex numbers represented as (f64, f64) tuples.
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex conjugate.
#[inline]
fn conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Add two complex numbers.
#[inline]
fn cadd(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Subtract two complex numbers.
#[inline]
fn csub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Magnitude squared of a complex number.
#[inline]
fn mag_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Scale a complex number by a real scalar.
#[inline]
fn cscale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

// ── Configuration ───────────────────────────────────────────────────────────

/// Configuration parameters for the passive radar processor.
///
/// Controls sample rate, CAF dimensions, and DSI filter length.
#[derive(Debug, Clone)]
pub struct PassiveRadarConfig {
    /// ADC / receiver sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Number of Doppler bins in the cross-ambiguity function.
    pub caf_doppler_bins: usize,
    /// Number of range (delay) bins in the cross-ambiguity function.
    pub caf_range_bins: usize,
    /// Number of taps for the LMS-based DSI cancellation filter.
    pub dsi_filter_taps: usize,
}

impl Default for PassiveRadarConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 2.048e6,
            caf_doppler_bins: 256,
            caf_range_bins: 512,
            dsi_filter_taps: 64,
        }
    }
}

// ── CAF Result ──────────────────────────────────────────────────────────────

/// Result of a cross-ambiguity function computation.
///
/// Contains the 2-D range-Doppler power map plus resolution metadata.
#[derive(Debug, Clone)]
pub struct CafResult {
    /// 2-D power map indexed as `map[range_bin][doppler_bin]`.
    pub map: Vec<Vec<f64>>,
    /// Range resolution in metres, determined by bandwidth.
    pub range_resolution_m: f64,
    /// Doppler resolution in Hz, determined by integration time.
    pub doppler_resolution_hz: f64,
}

// ── Main processor ──────────────────────────────────────────────────────────

/// Passive radar processor combining DSI cancellation, CAF, and detection.
///
/// Wraps the individual processing functions into a convenient pipeline.
#[derive(Debug, Clone)]
pub struct PassiveRadarProcessor {
    /// Processor configuration.
    pub config: PassiveRadarConfig,
}

impl PassiveRadarProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: PassiveRadarConfig) -> Self {
        Self { config }
    }

    /// Run the full processing pipeline: DSI cancellation, CAF, CFAR detection.
    ///
    /// Returns the CAF result and a list of detected targets
    /// `(range_bin, doppler_bin, power)`.
    pub fn process(
        &self,
        reference: &[(f64, f64)],
        surveillance: &[(f64, f64)],
        max_doppler_hz: f64,
        guard_cells: usize,
        ref_cells: usize,
        cfar_threshold: f64,
    ) -> (CafResult, Vec<(usize, usize, f64)>) {
        // DSI cancellation on a mutable copy of surveillance
        let mut surv = surveillance.to_vec();
        cancel_dsi(reference, &mut surv, self.config.dsi_filter_taps);

        // CAF computation
        let caf = cross_ambiguity_function(
            reference,
            &surv,
            self.config.caf_range_bins,
            max_doppler_hz,
            self.config.caf_doppler_bins,
            self.config.sample_rate_hz,
        );

        // CFAR detection
        let targets = detect_targets_cfar(&caf.map, guard_cells, ref_cells, cfar_threshold);

        (caf, targets)
    }

    /// Extract a reference signal by bandpass filtering around the illuminator.
    ///
    /// Applies a simple frequency-shift-and-decimate approach: shifts the
    /// signal by `center_freq_hz`, then applies a rectangular low-pass of
    /// width `bandwidth_hz`.
    pub fn extract_reference(
        &self,
        signal: &[(f64, f64)],
        center_freq_hz: f64,
        bandwidth_hz: f64,
    ) -> Vec<(f64, f64)> {
        extract_reference_signal(
            signal,
            center_freq_hz,
            bandwidth_hz,
            self.config.sample_rate_hz,
        )
    }
}

// ── Public free functions ───────────────────────────────────────────────────

/// Compute the cross-ambiguity function (CAF) between reference and
/// surveillance signals.
///
/// Evaluates the correlation at each (delay, Doppler) hypothesis and returns
/// the power map.
///
/// # Arguments
///
/// * `reference`    - Reference channel samples (direct-path illuminator)
/// * `surveillance` - Surveillance channel samples (target echoes)
/// * `max_delay`    - Maximum delay in samples (number of range bins)
/// * `max_doppler_hz` - Maximum Doppler shift to search (plus/minus)
/// * `doppler_bins` - Number of Doppler bins
/// * `sample_rate`  - Sample rate in Hz
pub fn cross_ambiguity_function(
    reference: &[(f64, f64)],
    surveillance: &[(f64, f64)],
    max_delay: usize,
    max_doppler_hz: f64,
    doppler_bins: usize,
    sample_rate: f64,
) -> CafResult {
    let n = reference.len().min(surveillance.len());
    if n == 0 || max_delay == 0 || doppler_bins == 0 {
        return CafResult {
            map: vec![],
            range_resolution_m: 0.0,
            doppler_resolution_hz: 0.0,
        };
    }

    let doppler_step = if doppler_bins > 1 {
        2.0 * max_doppler_hz / (doppler_bins - 1) as f64
    } else {
        0.0
    };
    let doppler_start = -max_doppler_hz;
    let integration_time = n as f64 / sample_rate;
    let doppler_resolution = if integration_time > 0.0 {
        1.0 / integration_time
    } else {
        0.0
    };
    let range_res = SPEED_OF_LIGHT / (2.0 * sample_rate);

    let mut map = Vec::with_capacity(max_delay);

    for delay in 0..max_delay {
        let mut row = Vec::with_capacity(doppler_bins);
        for d_idx in 0..doppler_bins {
            let doppler_hz = doppler_start + d_idx as f64 * doppler_step;

            // Correlate: sum over k of surv[k+delay] * conj(ref[k]) * exp(-j*2*pi*fd*k/fs)
            let mut accum = (0.0_f64, 0.0_f64);
            let upper = n.saturating_sub(delay);

            for k in 0..upper {
                let surv_k = if k + delay < surveillance.len() {
                    surveillance[k + delay]
                } else {
                    (0.0, 0.0)
                };
                let ref_k = reference[k];
                let phase = -2.0 * PI * doppler_hz * (k as f64) / sample_rate;
                let steering = (phase.cos(), phase.sin());

                let product = cmul(surv_k, conj(ref_k));
                let steered = cmul(product, steering);
                accum = cadd(accum, steered);
            }

            row.push(mag_sq(accum));
        }
        map.push(row);
    }

    CafResult {
        map,
        range_resolution_m: range_res,
        doppler_resolution_hz: doppler_resolution,
    }
}

/// Cancel direct-signal interference (DSI) from the surveillance channel
/// using an LMS adaptive filter.
///
/// The filter estimates the direct-path contribution from the reference
/// signal and subtracts it from the surveillance channel in-place.
///
/// # Arguments
///
/// * `reference`    - Reference channel samples
/// * `surveillance` - Surveillance channel samples (modified in-place)
/// * `num_taps`     - Number of adaptive filter taps
pub fn cancel_dsi(
    reference: &[(f64, f64)],
    surveillance: &mut Vec<(f64, f64)>,
    num_taps: usize,
) {
    if num_taps == 0 || reference.is_empty() || surveillance.is_empty() {
        return;
    }

    let n = reference.len().min(surveillance.len());
    let mu = 0.01; // LMS step size

    // Adaptive filter weights
    let mut weights: Vec<(f64, f64)> = vec![(0.0, 0.0); num_taps];

    for k in 0..n {
        // Build the reference tap vector for sample k
        // y_hat = sum(w[m] * ref[k - m])
        let mut y_hat = (0.0_f64, 0.0_f64);
        for m in 0..num_taps {
            if k >= m {
                y_hat = cadd(y_hat, cmul(weights[m], reference[k - m]));
            }
        }

        // Error signal: surveillance - estimated direct path
        let error = csub(surveillance[k], y_hat);

        // Update weights: w[m] += mu * error * conj(ref[k - m])
        for m in 0..num_taps {
            if k >= m {
                let update = cmul(cscale(error, mu), conj(reference[k - m]));
                weights[m] = cadd(weights[m], update);
            }
        }

        // Replace surveillance sample with the error (DSI removed)
        surveillance[k] = error;
    }
}

/// Detect targets in a CAF range-Doppler map using Cell-Averaging CFAR.
///
/// Returns a vector of `(range_bin, doppler_bin, power)` for each detection.
///
/// # Arguments
///
/// * `caf`              - 2-D power map `[range_bin][doppler_bin]`
/// * `guard_cells`      - Number of guard cells on each side
/// * `ref_cells`        - Number of reference cells on each side
/// * `threshold_factor` - Multiplicative threshold above noise floor
pub fn detect_targets_cfar(
    caf: &[Vec<f64>],
    guard_cells: usize,
    ref_cells: usize,
    threshold_factor: f64,
) -> Vec<(usize, usize, f64)> {
    let mut detections = Vec::new();
    let num_range = caf.len();
    if num_range == 0 {
        return detections;
    }
    let num_doppler = caf[0].len();
    if num_doppler == 0 {
        return detections;
    }

    let window = guard_cells + ref_cells;

    for r in window..num_range.saturating_sub(window) {
        for d in window..num_doppler.saturating_sub(window) {
            let mut sum = 0.0;
            let mut count = 0usize;

            // 2-D CA-CFAR: average over reference cells in range and Doppler
            for dr in 1..=ref_cells {
                let r_lo = r.wrapping_sub(guard_cells + dr);
                let r_hi = r + guard_cells + dr;
                if r_lo < num_range {
                    sum += caf[r_lo][d];
                    count += 1;
                }
                if r_hi < num_range {
                    sum += caf[r_hi][d];
                    count += 1;
                }
            }
            for dd in 1..=ref_cells {
                let d_lo = d.wrapping_sub(guard_cells + dd);
                let d_hi = d + guard_cells + dd;
                if d_lo < num_doppler {
                    sum += caf[r][d_lo];
                    count += 1;
                }
                if d_hi < num_doppler {
                    sum += caf[r][d_hi];
                    count += 1;
                }
            }

            if count > 0 {
                let threshold = threshold_factor * sum / count as f64;
                let power = caf[r][d];
                if power > threshold {
                    detections.push((r, d, power));
                }
            }
        }
    }

    detections
}

/// Convert a delay in samples to bistatic range in metres.
///
/// Bistatic range = c * delay / sample_rate (one-way equivalent).
pub fn bistatic_range(delay_samples: usize, sample_rate: f64) -> f64 {
    SPEED_OF_LIGHT * delay_samples as f64 / sample_rate
}

/// Convert a Doppler bin index to bistatic velocity in m/s.
///
/// The Doppler axis spans `[-1/(2*T), +1/(2*T)]` where `T` is the
/// integration time, mapped onto `num_bins` bins. The velocity is
/// `v = fd * wavelength / 2`.
///
/// # Arguments
///
/// * `doppler_bin`       - Index into the Doppler axis
/// * `num_bins`          - Total number of Doppler bins
/// * `wavelength_m`      - Carrier wavelength in metres
/// * `integration_time_s` - Coherent integration time in seconds
pub fn bistatic_velocity(
    doppler_bin: usize,
    num_bins: usize,
    wavelength_m: f64,
    integration_time_s: f64,
) -> f64 {
    if num_bins == 0 || integration_time_s <= 0.0 {
        return 0.0;
    }
    let max_doppler = 1.0 / (2.0 * integration_time_s);
    let doppler_step = 2.0 * max_doppler / num_bins as f64;
    let fd = -max_doppler + doppler_bin as f64 * doppler_step;
    fd * wavelength_m / 2.0
}

/// Compute the processing gain (SNR improvement) in dB from coherent
/// integration.
///
/// `gain_dB = 10 * log10(integration_time * bandwidth)`
pub fn compute_snr_improvement(integration_time_s: f64, bandwidth_hz: f64) -> f64 {
    let product = integration_time_s * bandwidth_hz;
    if product <= 0.0 {
        return 0.0;
    }
    10.0 * product.log10()
}

/// Compute bistatic range resolution given the signal bandwidth.
///
/// `delta_R = c / (2 * B)`
pub fn range_resolution(bandwidth_hz: f64) -> f64 {
    if bandwidth_hz <= 0.0 {
        return f64::INFINITY;
    }
    SPEED_OF_LIGHT / (2.0 * bandwidth_hz)
}

/// Extract a reference signal by frequency-shifting and low-pass filtering.
///
/// Shifts the input signal down by `center_freq_hz` and retains only the
/// components within `+/- bandwidth_hz/2` using a simple moving-average LPF.
pub fn extract_reference_signal(
    signal: &[(f64, f64)],
    center_freq_hz: f64,
    bandwidth_hz: f64,
    sample_rate_hz: f64,
) -> Vec<(f64, f64)> {
    if signal.is_empty() || sample_rate_hz <= 0.0 {
        return vec![];
    }

    // Frequency-shift the signal
    let shifted: Vec<(f64, f64)> = signal
        .iter()
        .enumerate()
        .map(|(k, &s)| {
            let phase = -2.0 * PI * center_freq_hz * k as f64 / sample_rate_hz;
            cmul(s, (phase.cos(), phase.sin()))
        })
        .collect();

    // Simple moving-average low-pass filter as a placeholder
    // Filter length ~ sample_rate / bandwidth (at least 1)
    let filter_len = ((sample_rate_hz / bandwidth_hz).round() as usize).max(1);
    let inv_len = 1.0 / filter_len as f64;

    let mut output = Vec::with_capacity(shifted.len());
    for k in 0..shifted.len() {
        let mut accum = (0.0, 0.0);
        let start = k.saturating_sub(filter_len - 1);
        for j in start..=k {
            accum = cadd(accum, shifted[j]);
        }
        output.push(cscale(accum, inv_len));
    }

    output
}

/// Compute the Doppler resolution for a given integration time.
///
/// `delta_fd = 1 / T_int`
pub fn doppler_resolution(integration_time_s: f64) -> f64 {
    if integration_time_s <= 0.0 {
        return f64::INFINITY;
    }
    1.0 / integration_time_s
}

/// Convert carrier frequency to wavelength in metres.
///
/// `lambda = c / f`
pub fn frequency_to_wavelength(frequency_hz: f64) -> f64 {
    if frequency_hz <= 0.0 {
        return f64::INFINITY;
    }
    SPEED_OF_LIGHT / frequency_hz
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a complex tone at the given frequency.
    fn tone(freq_hz: f64, sample_rate: f64, n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|k| {
                let phase = 2.0 * PI * freq_hz * k as f64 / sample_rate;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    /// Helper: add two signal vectors element-wise.
    fn add_signals(a: &[(f64, f64)], b: &[(f64, f64)]) -> Vec<(f64, f64)> {
        a.iter().zip(b.iter()).map(|(&x, &y)| cadd(x, y)).collect()
    }

    /// Helper: delay a signal by `d` samples (zero-padded front).
    fn delay_signal(signal: &[(f64, f64)], d: usize) -> Vec<(f64, f64)> {
        let mut out = vec![(0.0, 0.0); signal.len()];
        for i in d..signal.len() {
            out[i] = signal[i - d];
        }
        out
    }

    /// Helper: apply a Doppler shift to a signal.
    fn doppler_shift(signal: &[(f64, f64)], fd: f64, sample_rate: f64) -> Vec<(f64, f64)> {
        signal
            .iter()
            .enumerate()
            .map(|(k, &s)| {
                let phase = 2.0 * PI * fd * k as f64 / sample_rate;
                cmul(s, (phase.cos(), phase.sin()))
            })
            .collect()
    }

    #[test]
    fn test_config_default() {
        let cfg = PassiveRadarConfig::default();
        assert_eq!(cfg.caf_doppler_bins, 256);
        assert_eq!(cfg.caf_range_bins, 512);
        assert_eq!(cfg.dsi_filter_taps, 64);
        assert!((cfg.sample_rate_hz - 2.048e6).abs() < 1.0);
    }

    #[test]
    fn test_caf_basic_peak() {
        // A delayed wideband signal should produce a peak at the correct delay.
        // We use a pseudo-random noise-like signal (multi-tone) for good
        // autocorrelation properties (a single tone has a flat autocorrelation).
        let fs = 10_000.0;
        let n = 512;
        // Create a wideband reference: sum of several incommensurate tones
        let reference: Vec<(f64, f64)> = (0..n)
            .map(|k| {
                let t = k as f64 / fs;
                let mut re = 0.0;
                let mut im = 0.0;
                for &freq in &[137.0, 523.0, 1171.0, 2377.0, 3571.0] {
                    let phase = 2.0 * PI * freq * t;
                    re += phase.cos();
                    im += phase.sin();
                }
                (re, im)
            })
            .collect();

        let delay = 10;
        let surveillance = delay_signal(&reference, delay);

        let caf = cross_ambiguity_function(&reference, &surveillance, 30, 100.0, 11, fs);

        // Find peak range bin
        let mut max_power = 0.0;
        let mut peak_range = 0;
        for (r, row) in caf.map.iter().enumerate() {
            for &p in row {
                if p > max_power {
                    max_power = p;
                    peak_range = r;
                }
            }
        }
        // The peak should be near the true delay
        assert!(
            (peak_range as i64 - delay as i64).unsigned_abs() <= 1,
            "Expected peak near delay {}, got {}",
            delay,
            peak_range
        );
    }

    #[test]
    fn test_caf_doppler_peak() {
        // A Doppler-shifted copy should produce a peak at the correct Doppler bin
        let fs = 10_000.0;
        let n = 1024;
        let reference = tone(1000.0, fs, n);
        let fd = 50.0;
        let surveillance = doppler_shift(&reference, fd, fs);

        let max_doppler = 200.0;
        let doppler_bins = 41; // centered on 0
        let caf = cross_ambiguity_function(&reference, &surveillance, 5, max_doppler, doppler_bins, fs);

        // At range bin 0, find the Doppler bin with max power
        let row = &caf.map[0];
        let mut peak_dbin = 0;
        let mut max_p = 0.0;
        for (d, &p) in row.iter().enumerate() {
            if p > max_p {
                max_p = p;
                peak_dbin = d;
            }
        }

        // Convert bin to Hz
        let doppler_step = 2.0 * max_doppler / (doppler_bins - 1) as f64;
        let detected_fd = -max_doppler + peak_dbin as f64 * doppler_step;
        assert!(
            (detected_fd - fd).abs() < doppler_step * 1.5,
            "Expected Doppler ~{} Hz, got {} Hz",
            fd,
            detected_fd
        );
    }

    #[test]
    fn test_caf_empty_inputs() {
        let caf = cross_ambiguity_function(&[], &[], 10, 100.0, 5, 1e6);
        assert!(caf.map.is_empty());
    }

    #[test]
    fn test_caf_zero_delay() {
        let caf = cross_ambiguity_function(&[(1.0, 0.0)], &[(1.0, 0.0)], 0, 100.0, 5, 1e6);
        assert!(caf.map.is_empty());
    }

    #[test]
    fn test_caf_resolution_metadata() {
        let fs = 2e6;
        let n = 2000;
        let reference = tone(100.0, fs, n);
        let caf = cross_ambiguity_function(&reference, &reference, 10, 500.0, 21, fs);

        let expected_range_res = SPEED_OF_LIGHT / (2.0 * fs);
        assert!((caf.range_resolution_m - expected_range_res).abs() < 1e-3);

        let integration_time = n as f64 / fs;
        let expected_doppler_res = 1.0 / integration_time;
        assert!((caf.doppler_resolution_hz - expected_doppler_res).abs() < 1e-3);
    }

    #[test]
    fn test_cancel_dsi_reduces_power() {
        let fs = 10_000.0;
        let n = 2000;
        let reference = tone(500.0, fs, n);

        // Surveillance = direct path (strong) + weak echo
        let echo_delay = 20;
        let echo = delay_signal(&reference, echo_delay);
        let echo_scaled: Vec<(f64, f64)> = echo.iter().map(|&s| cscale(s, 0.01)).collect();
        let mut surveillance = add_signals(&reference, &echo_scaled);

        // Measure power before cancellation (skip transient)
        let power_before: f64 = surveillance[100..]
            .iter()
            .map(|s| mag_sq(*s))
            .sum::<f64>()
            / (n - 100) as f64;

        cancel_dsi(&reference, &mut surveillance, 64);

        // Power after should be significantly reduced
        let power_after: f64 = surveillance[100..]
            .iter()
            .map(|s| mag_sq(*s))
            .sum::<f64>()
            / (n - 100) as f64;

        assert!(
            power_after < power_before * 0.5,
            "DSI cancellation should reduce power: before={:.4}, after={:.4}",
            power_before,
            power_after
        );
    }

    #[test]
    fn test_cancel_dsi_empty() {
        let mut surv = vec![(1.0, 0.0); 10];
        cancel_dsi(&[], &mut surv, 5);
        // Should return without modification
        assert!((surv[0].0 - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_cancel_dsi_zero_taps() {
        let reference = vec![(1.0, 0.0); 10];
        let mut surv = vec![(1.0, 0.0); 10];
        cancel_dsi(&reference, &mut surv, 0);
        // No cancellation should occur
        assert!((surv[0].0 - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_detect_targets_cfar_basic() {
        // Create a map with a clear peak
        let rows = 30;
        let cols = 20;
        let mut map = vec![vec![1.0; cols]; rows];
        // Insert a strong target
        map[15][10] = 100.0;

        let targets = detect_targets_cfar(&map, 2, 4, 5.0);
        assert!(
            !targets.is_empty(),
            "Should detect at least one target"
        );

        // The detected target should be at or near (15, 10)
        let found = targets.iter().any(|&(r, d, _)| r == 15 && d == 10);
        assert!(found, "Target at (15, 10) should be detected");
    }

    #[test]
    fn test_detect_targets_cfar_no_targets() {
        // Uniform noise floor - no targets should be detected
        let map = vec![vec![1.0; 30]; 30];
        let targets = detect_targets_cfar(&map, 2, 4, 2.0);
        assert!(
            targets.is_empty(),
            "Uniform map should yield no detections"
        );
    }

    #[test]
    fn test_detect_targets_cfar_empty() {
        let targets = detect_targets_cfar(&[], 2, 4, 5.0);
        assert!(targets.is_empty());

        let targets2 = detect_targets_cfar(&[vec![]], 2, 4, 5.0);
        assert!(targets2.is_empty());
    }

    #[test]
    fn test_bistatic_range_values() {
        let fs = 1e6;
        let delay = 100;
        let range = bistatic_range(delay, fs);
        let expected = SPEED_OF_LIGHT * 100.0 / 1e6;
        assert!(
            (range - expected).abs() < 1e-3,
            "Expected {:.3} m, got {:.3} m",
            expected,
            range
        );
    }

    #[test]
    fn test_bistatic_range_zero() {
        assert!((bistatic_range(0, 1e6)).abs() < 1e-10);
    }

    #[test]
    fn test_bistatic_velocity_center_bin() {
        // Center bin should give ~0 velocity
        let num_bins = 101;
        let center = 50;
        let wavelength = 3.0; // ~100 MHz
        let t_int = 0.01;

        let vel = bistatic_velocity(center, num_bins, wavelength, t_int);
        // Should be approximately zero (within one bin's velocity)
        let max_doppler = 1.0 / (2.0 * t_int);
        let doppler_step = 2.0 * max_doppler / num_bins as f64;
        let vel_resolution = doppler_step * wavelength / 2.0;
        assert!(
            vel.abs() < vel_resolution * 1.5,
            "Center bin velocity should be ~0, got {}",
            vel
        );
    }

    #[test]
    fn test_bistatic_velocity_edge_cases() {
        assert_eq!(bistatic_velocity(0, 0, 3.0, 0.01), 0.0);
        assert_eq!(bistatic_velocity(0, 10, 3.0, 0.0), 0.0);
        assert_eq!(bistatic_velocity(0, 10, 3.0, -1.0), 0.0);
    }

    #[test]
    fn test_compute_snr_improvement() {
        let gain = compute_snr_improvement(0.001, 1e6);
        // 10*log10(0.001 * 1e6) = 10*log10(1000) = 30 dB
        assert!(
            (gain - 30.0).abs() < 0.01,
            "Expected 30 dB, got {:.2} dB",
            gain
        );
    }

    #[test]
    fn test_compute_snr_improvement_edge() {
        assert_eq!(compute_snr_improvement(0.0, 1e6), 0.0);
        assert_eq!(compute_snr_improvement(-1.0, 1e6), 0.0);
        assert_eq!(compute_snr_improvement(1.0, 0.0), 0.0);
    }

    #[test]
    fn test_range_resolution_fm() {
        // FM broadcast: ~200 kHz bandwidth
        let res = range_resolution(200e3);
        let expected = SPEED_OF_LIGHT / (2.0 * 200e3);
        assert!(
            (res - expected).abs() < 0.1,
            "Expected {:.1} m, got {:.1} m",
            expected,
            res
        );
        // Should be ~750 m
        assert!(res > 500.0 && res < 1000.0);
    }

    #[test]
    fn test_range_resolution_edge() {
        assert_eq!(range_resolution(0.0), f64::INFINITY);
        assert_eq!(range_resolution(-1.0), f64::INFINITY);
    }

    #[test]
    fn test_doppler_resolution_function() {
        let dr = doppler_resolution(0.1);
        assert!((dr - 10.0).abs() < 1e-10);

        assert_eq!(doppler_resolution(0.0), f64::INFINITY);
        assert_eq!(doppler_resolution(-1.0), f64::INFINITY);
    }

    #[test]
    fn test_frequency_to_wavelength() {
        // FM radio ~100 MHz => ~3 m
        let wl = frequency_to_wavelength(100e6);
        assert!(
            (wl - 2.998).abs() < 0.01,
            "Expected ~3.0 m, got {:.3} m",
            wl
        );

        assert_eq!(frequency_to_wavelength(0.0), f64::INFINITY);
        assert_eq!(frequency_to_wavelength(-1.0), f64::INFINITY);
    }

    #[test]
    fn test_extract_reference_signal() {
        let fs = 10_000.0;
        let n = 500;
        let sig = tone(1000.0, fs, n);

        let extracted = extract_reference_signal(&sig, 1000.0, 500.0, fs);
        assert_eq!(extracted.len(), n);

        // After shifting 1000 Hz tone down by 1000 Hz, we should have ~DC
        // The output should have non-zero energy
        let power: f64 = extracted.iter().map(|s| mag_sq(*s)).sum::<f64>() / n as f64;
        assert!(power > 0.01, "Extracted reference should have energy");
    }

    #[test]
    fn test_extract_reference_empty() {
        let result = extract_reference_signal(&[], 1000.0, 200.0, 10_000.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_processor_full_pipeline() {
        let config = PassiveRadarConfig {
            sample_rate_hz: 10_000.0,
            caf_doppler_bins: 21,
            caf_range_bins: 30,
            dsi_filter_taps: 32,
        };

        let n = 1024;
        let reference = tone(500.0, config.sample_rate_hz, n);

        // Target: delayed and Doppler-shifted echo
        let delay = 8;
        let fd = 30.0;
        let echo = delay_signal(&reference, delay);
        let echo_doppler = doppler_shift(&echo, fd, config.sample_rate_hz);
        let echo_weak: Vec<(f64, f64)> = echo_doppler.iter().map(|&s| cscale(s, 0.1)).collect();

        // Surveillance = strong direct path + weak target echo
        let surveillance = add_signals(&reference, &echo_weak);

        let processor = PassiveRadarProcessor::new(config);
        let (caf, _targets) = processor.process(&reference, &surveillance, 200.0, 2, 4, 3.0);

        // CAF should have been computed
        assert!(!caf.map.is_empty());
        assert!(caf.range_resolution_m > 0.0);
        assert!(caf.doppler_resolution_hz > 0.0);
    }

    #[test]
    fn test_processor_extract_reference() {
        let config = PassiveRadarConfig {
            sample_rate_hz: 10_000.0,
            caf_doppler_bins: 32,
            caf_range_bins: 64,
            dsi_filter_taps: 16,
        };

        let processor = PassiveRadarProcessor::new(config);
        let signal = tone(2000.0, 10_000.0, 500);
        let extracted = processor.extract_reference(&signal, 2000.0, 1000.0);
        assert_eq!(extracted.len(), 500);
    }

    #[test]
    fn test_cmul_identity() {
        let a = (3.0, 4.0);
        let one = (1.0, 0.0);
        let result = cmul(a, one);
        assert!((result.0 - 3.0).abs() < 1e-10);
        assert!((result.1 - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_cfar_multiple_targets() {
        let rows = 50;
        let cols = 50;
        let mut map = vec![vec![1.0; cols]; rows];

        // Two well-separated targets
        map[15][15] = 80.0;
        map[35][35] = 60.0;

        let targets = detect_targets_cfar(&map, 2, 4, 5.0);
        assert!(
            targets.len() >= 2,
            "Should detect at least 2 targets, got {}",
            targets.len()
        );

        let found_a = targets.iter().any(|&(r, d, _)| r == 15 && d == 15);
        let found_b = targets.iter().any(|&(r, d, _)| r == 35 && d == 35);
        assert!(found_a, "Target A at (15,15) not detected");
        assert!(found_b, "Target B at (35,35) not detected");
    }
}
