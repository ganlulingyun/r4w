//! Structural Health Monitoring (SHM) for civil infrastructure.
//!
//! This module provides tools for monitoring the structural integrity of bridges,
//! dams, buildings, and other civil infrastructure. It implements modal analysis
//! (natural frequency extraction), damping ratio estimation (logarithmic decrement
//! and half-power bandwidth methods), mode shape extraction, damage index
//! calculation, operational deflection shape analysis, acceleration-to-displacement
//! double integration with detrending, structural capacity rating, and alarm
//! threshold management per ISO 4866.
//!
//! # Example
//!
//! ```
//! use r4w_core::structural_health_monitor::{
//!     ShmConfig, StructuralMonitor, extract_natural_frequencies,
//!     rms_acceleration, peak_acceleration,
//! };
//!
//! let config = ShmConfig {
//!     sample_rate_hz: 1000.0,
//!     num_channels: 4,
//!     fft_size: 1024,
//! };
//! let monitor = StructuralMonitor::new(config);
//!
//! // Generate a test signal with a 10 Hz component
//! let sample_rate = 1000.0;
//! let n = 2048;
//! let signal: Vec<f64> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / sample_rate;
//!         (2.0 * std::f64::consts::PI * 10.0 * t).sin()
//!     })
//!     .collect();
//!
//! let freqs = extract_natural_frequencies(&signal, sample_rate, 1);
//! assert!((freqs[0] - 10.0).abs() < 1.0);
//!
//! let rms = rms_acceleration(&signal);
//! assert!(rms > 0.0);
//!
//! let peak = peak_acceleration(&signal);
//! assert!((peak - 1.0).abs() < 0.01);
//! ```

use std::f64::consts::PI;

/// Configuration for the structural health monitor.
#[derive(Debug, Clone)]
pub struct ShmConfig {
    /// Sampling rate of the sensor data in Hz.
    pub sample_rate_hz: f64,
    /// Number of sensor channels (measurement points).
    pub num_channels: usize,
    /// FFT size used for spectral analysis.
    pub fft_size: usize,
}

/// Results of modal analysis: natural frequencies, damping ratios, and mode shapes.
#[derive(Debug, Clone)]
pub struct ModalResult {
    /// Identified natural frequencies in Hz.
    pub natural_frequencies_hz: Vec<f64>,
    /// Corresponding damping ratios (fraction of critical).
    pub damping_ratios: Vec<f64>,
    /// Mode shapes: each inner Vec represents the relative displacement at each channel.
    pub mode_shapes: Vec<Vec<f64>>,
}

/// Damage index for a particular structural mode.
#[derive(Debug, Clone)]
pub struct DamageIndex {
    /// Index of the mode where damage is detected.
    pub location_index: usize,
    /// Severity of damage as a normalized value in [0, 1].
    pub severity: f64,
    /// Baseline natural frequency for this mode (Hz).
    pub baseline_freq_hz: f64,
    /// Current measured natural frequency for this mode (Hz).
    pub current_freq_hz: f64,
}

/// Alarm threshold configuration per ISO 4866 guidelines.
#[derive(Debug, Clone)]
pub struct AlarmThreshold {
    /// Name or label of this threshold.
    pub name: String,
    /// Peak velocity limit in mm/s.
    pub peak_velocity_mms: f64,
    /// Frequency range lower bound in Hz.
    pub freq_min_hz: f64,
    /// Frequency range upper bound in Hz.
    pub freq_max_hz: f64,
    /// Classification of the structure type (e.g., "residential", "commercial", "historic").
    pub structure_class: String,
}

/// Structural capacity rating result.
#[derive(Debug, Clone)]
pub struct CapacityRating {
    /// Overall capacity ratio (current / baseline), 1.0 = undamaged.
    pub ratio: f64,
    /// Descriptive rating category.
    pub category: &'static str,
    /// Individual mode frequency shifts as percentages.
    pub mode_shifts_percent: Vec<f64>,
}

/// Main structural health monitor.
#[derive(Debug, Clone)]
pub struct StructuralMonitor {
    /// Monitor configuration.
    pub config: ShmConfig,
    /// Alarm thresholds currently registered.
    pub thresholds: Vec<AlarmThreshold>,
    /// Baseline modal result (set after initial characterization).
    pub baseline: Option<ModalResult>,
}

impl StructuralMonitor {
    /// Create a new structural health monitor with the given configuration.
    pub fn new(config: ShmConfig) -> Self {
        Self {
            config,
            thresholds: Vec::new(),
            baseline: None,
        }
    }

    /// Set the baseline modal result from an initial characterization measurement.
    pub fn set_baseline(&mut self, baseline: ModalResult) {
        self.baseline = Some(baseline);
    }

    /// Add an alarm threshold.
    pub fn add_threshold(&mut self, threshold: AlarmThreshold) {
        self.thresholds.push(threshold);
    }

    /// Remove all alarm thresholds.
    pub fn clear_thresholds(&mut self) {
        self.thresholds.clear();
    }

    /// Check a measurement against all registered alarm thresholds.
    /// Returns a list of (threshold_name, exceeded: bool) pairs.
    pub fn check_alarms(&self, peak_velocity_mms: f64, freq_hz: f64) -> Vec<(String, bool)> {
        self.thresholds
            .iter()
            .map(|t| {
                let in_range = freq_hz >= t.freq_min_hz && freq_hz <= t.freq_max_hz;
                let exceeded = in_range && peak_velocity_mms > t.peak_velocity_mms;
                (t.name.clone(), exceeded)
            })
            .collect()
    }

    /// Run a full modal analysis on multi-channel data and compare to baseline.
    /// Returns the current modal result and optional damage indices if a baseline is set.
    pub fn analyze(
        &self,
        channels: &[Vec<f64>],
        num_modes: usize,
    ) -> (ModalResult, Vec<DamageIndex>) {
        let sr = self.config.sample_rate_hz;

        // Extract natural frequencies from first channel as reference
        let freqs = if !channels.is_empty() && !channels[0].is_empty() {
            extract_natural_frequencies(&channels[0], sr, num_modes)
        } else {
            Vec::new()
        };

        // Compute damping ratios via log decrement for each mode frequency
        let damping_ratios: Vec<f64> = freqs
            .iter()
            .map(|_freq| {
                if !channels.is_empty() && channels[0].len() > 2 {
                    damping_log_decrement(&channels[0], sr)
                } else {
                    0.0
                }
            })
            .collect();

        // Extract mode shapes
        let mode_shapes: Vec<Vec<f64>> = freqs
            .iter()
            .map(|&f| {
                if channels.len() > 1 {
                    mode_shape_extract(channels, f, sr)
                } else if channels.len() == 1 {
                    vec![1.0]
                } else {
                    Vec::new()
                }
            })
            .collect();

        let modal_result = ModalResult {
            natural_frequencies_hz: freqs.clone(),
            damping_ratios,
            mode_shapes,
        };

        // Compare with baseline if available
        let damage = if let Some(ref baseline) = self.baseline {
            compute_damage_index(&baseline.natural_frequencies_hz, &freqs)
        } else {
            Vec::new()
        };

        (modal_result, damage)
    }

    /// Compute structural capacity rating based on baseline and current frequencies.
    pub fn capacity_rating(&self, current_freqs: &[f64]) -> Option<CapacityRating> {
        let baseline = self.baseline.as_ref()?;
        let n = baseline.natural_frequencies_hz.len().min(current_freqs.len());
        if n == 0 {
            return None;
        }

        let mut shifts = Vec::with_capacity(n);
        let mut sum_ratio = 0.0;
        for i in 0..n {
            let bf = baseline.natural_frequencies_hz[i];
            let cf = current_freqs[i];
            if bf > 0.0 {
                let shift_pct = ((bf - cf) / bf) * 100.0;
                shifts.push(shift_pct);
                sum_ratio += cf / bf;
            } else {
                shifts.push(0.0);
                sum_ratio += 1.0;
            }
        }
        let ratio = sum_ratio / n as f64;

        let category = if ratio >= 0.95 {
            "Good"
        } else if ratio >= 0.85 {
            "Fair"
        } else if ratio >= 0.70 {
            "Poor"
        } else {
            "Critical"
        };

        Some(CapacityRating {
            ratio,
            category,
            mode_shifts_percent: shifts,
        })
    }
}

// ---------------------------------------------------------------------------
// Standalone public functions
// ---------------------------------------------------------------------------

/// Compute the power spectrum magnitude of a real signal using a DFT.
///
/// Returns `fft_size / 2 + 1` magnitude values (one-sided spectrum).
fn compute_spectrum(signal: &[f64], fft_size: usize) -> Vec<f64> {
    let n = fft_size.min(signal.len());
    let mut spectrum = vec![0.0; fft_size / 2 + 1];

    for k in 0..spectrum.len() {
        let mut re = 0.0;
        let mut im = 0.0;
        for i in 0..n {
            let angle = -2.0 * PI * k as f64 * i as f64 / fft_size as f64;
            re += signal[i] * angle.cos();
            im += signal[i] * angle.sin();
        }
        spectrum[k] = (re * re + im * im).sqrt();
    }
    spectrum
}

/// Extract natural frequencies from a single-channel signal via peak picking in the
/// frequency domain.
///
/// Performs a DFT, then picks the `num_modes` largest spectral peaks (excluding DC).
/// Returns frequencies in Hz sorted from lowest to highest.
pub fn extract_natural_frequencies(signal: &[f64], sample_rate: f64, num_modes: usize) -> Vec<f64> {
    if signal.is_empty() || num_modes == 0 {
        return Vec::new();
    }

    let fft_size = signal.len();
    let spectrum = compute_spectrum(signal, fft_size);
    let freq_resolution = sample_rate / fft_size as f64;

    // Find peaks (local maxima), skip DC bin 0
    let mut peaks: Vec<(usize, f64)> = Vec::new();
    for k in 1..spectrum.len() - 1 {
        if spectrum[k] > spectrum[k - 1] && spectrum[k] > spectrum[k + 1] {
            peaks.push((k, spectrum[k]));
        }
    }
    // Also consider the last bin if it is larger than its neighbor
    if spectrum.len() > 2 {
        let last = spectrum.len() - 1;
        if spectrum[last] > spectrum[last - 1] {
            peaks.push((last, spectrum[last]));
        }
    }

    // Sort by magnitude descending
    peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

    // Take top num_modes
    let mut freqs: Vec<f64> = peaks
        .iter()
        .take(num_modes)
        .map(|&(bin, _)| bin as f64 * freq_resolution)
        .collect();

    freqs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    freqs
}

/// Estimate damping ratio using the logarithmic decrement method.
///
/// Finds successive positive peaks in the time-domain signal and computes the
/// average logarithmic decrement. Returns the damping ratio (zeta).
pub fn damping_log_decrement(signal: &[f64], _sample_rate: f64) -> f64 {
    if signal.len() < 3 {
        return 0.0;
    }

    // Find positive peaks (local maxima above zero)
    let mut peaks: Vec<f64> = Vec::new();
    for i in 1..signal.len() - 1 {
        if signal[i] > signal[i - 1] && signal[i] > signal[i + 1] && signal[i] > 0.0 {
            peaks.push(signal[i]);
        }
    }

    if peaks.len() < 2 {
        return 0.0;
    }

    // Compute average log decrement over consecutive peak pairs
    let mut total_delta = 0.0;
    let mut count = 0usize;
    for i in 0..peaks.len() - 1 {
        if peaks[i] > 1e-15 && peaks[i + 1] > 1e-15 {
            let delta = (peaks[i] / peaks[i + 1]).ln();
            if delta.is_finite() && delta > 0.0 {
                total_delta += delta;
                count += 1;
            }
        }
    }

    if count == 0 {
        return 0.0;
    }

    let avg_delta = total_delta / count as f64;

    // zeta = delta / sqrt(4*pi^2 + delta^2)
    avg_delta / (4.0 * PI * PI + avg_delta * avg_delta).sqrt()
}

/// Estimate damping ratio using the half-power bandwidth method.
///
/// Given a one-sided spectrum magnitude array, a peak bin index, and the frequency
/// resolution (Hz per bin), computes the damping ratio from the -3 dB bandwidth
/// around the peak.
pub fn damping_half_power(spectrum: &[f64], peak_bin: usize, freq_resolution: f64) -> f64 {
    if spectrum.is_empty() || peak_bin >= spectrum.len() || freq_resolution <= 0.0 {
        return 0.0;
    }

    let peak_mag = spectrum[peak_bin];
    if peak_mag <= 0.0 {
        return 0.0;
    }

    // Half-power level = peak / sqrt(2)
    let half_power = peak_mag / 2.0_f64.sqrt();

    // Search left for the -3 dB crossing
    for k in (0..peak_bin).rev() {
        if spectrum[k] < half_power {
            // Linear interpolation for left crossing
            let frac_l = if (spectrum[k + 1] - spectrum[k]).abs() > 1e-15 {
                (half_power - spectrum[k]) / (spectrum[k + 1] - spectrum[k])
            } else {
                0.5
            };
            let left_freq = (k as f64 + frac_l) * freq_resolution;

            // Search right for the -3 dB crossing
            let mut right_freq = (peak_bin as f64) * freq_resolution;
            for j in (peak_bin + 1)..spectrum.len() {
                if spectrum[j] < half_power {
                    let frac_r = if (spectrum[j - 1] - spectrum[j]).abs() > 1e-15 {
                        (half_power - spectrum[j]) / (spectrum[j - 1] - spectrum[j])
                    } else {
                        0.5
                    };
                    right_freq = (j as f64 - frac_r) * freq_resolution;
                    break;
                }
            }

            let bandwidth = right_freq - left_freq;
            let center_freq = peak_bin as f64 * freq_resolution;
            if center_freq > 0.0 {
                return bandwidth / (2.0 * center_freq);
            } else {
                return 0.0;
            }
        }
    }

    0.0
}

/// Extract a mode shape from multi-channel data at a specific frequency.
///
/// For each channel, computes the DFT coefficient at the target frequency and
/// returns the normalized magnitude and phase-relative amplitudes.
pub fn mode_shape_extract(channels: &[Vec<f64>], freq_hz: f64, sample_rate: f64) -> Vec<f64> {
    if channels.is_empty() || sample_rate <= 0.0 {
        return Vec::new();
    }

    let mut amplitudes: Vec<f64> = Vec::with_capacity(channels.len());
    let mut phases: Vec<f64> = Vec::with_capacity(channels.len());

    for ch in channels {
        if ch.is_empty() {
            amplitudes.push(0.0);
            phases.push(0.0);
            continue;
        }
        let n = ch.len();
        let mut re = 0.0;
        let mut im = 0.0;
        for i in 0..n {
            let angle = -2.0 * PI * freq_hz * i as f64 / sample_rate;
            re += ch[i] * angle.cos();
            im += ch[i] * angle.sin();
        }
        let mag = (re * re + im * im).sqrt();
        let phase = im.atan2(re);
        amplitudes.push(mag);
        phases.push(phase);
    }

    // Normalize to reference channel (the one with max amplitude)
    let max_amp = amplitudes
        .iter()
        .cloned()
        .fold(0.0_f64, f64::max);

    if max_amp < 1e-15 {
        return vec![0.0; channels.len()];
    }

    // Find reference phase (phase of channel with max amplitude)
    let ref_idx = amplitudes
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(i, _)| i)
        .unwrap_or(0);
    let ref_phase = phases[ref_idx];

    // Return signed mode shape: normalized amplitude * sign based on phase relation
    amplitudes
        .iter()
        .zip(phases.iter())
        .map(|(&amp, &ph)| {
            let relative = (ph - ref_phase).cos();
            (amp / max_amp) * relative.signum()
        })
        .collect()
}

/// Compute damage indices by comparing baseline and current natural frequencies.
///
/// For each mode, the severity is based on the relative frequency shift:
/// `severity = |f_baseline - f_current| / f_baseline`.
pub fn compute_damage_index(baseline_freqs: &[f64], current_freqs: &[f64]) -> Vec<DamageIndex> {
    let n = baseline_freqs.len().min(current_freqs.len());
    let mut indices = Vec::with_capacity(n);

    for i in 0..n {
        let bf = baseline_freqs[i];
        let cf = current_freqs[i];
        let severity = if bf > 0.0 {
            ((bf - cf) / bf).abs().min(1.0)
        } else {
            0.0
        };
        indices.push(DamageIndex {
            location_index: i,
            severity,
            baseline_freq_hz: bf,
            current_freq_hz: cf,
        });
    }

    indices
}

/// Double-integrate acceleration to obtain displacement.
///
/// Applies trapezoidal integration twice (acceleration -> velocity -> displacement)
/// with linear detrending after each integration step to remove drift.
pub fn integrate_to_displacement(acceleration: &[f64], sample_rate: f64) -> Vec<f64> {
    if acceleration.is_empty() || sample_rate <= 0.0 {
        return Vec::new();
    }

    let dt = 1.0 / sample_rate;

    // First integration: acceleration -> velocity
    let velocity = trapezoidal_integrate(acceleration, dt);
    let velocity = linear_detrend(&velocity);

    // Second integration: velocity -> displacement
    let displacement = trapezoidal_integrate(&velocity, dt);
    linear_detrend(&displacement)
}

/// Trapezoidal integration of a signal.
fn trapezoidal_integrate(signal: &[f64], dt: f64) -> Vec<f64> {
    let mut result = vec![0.0; signal.len()];
    for i in 1..signal.len() {
        result[i] = result[i - 1] + 0.5 * (signal[i - 1] + signal[i]) * dt;
    }
    result
}

/// Linear detrend: remove the best-fit straight line from the signal.
fn linear_detrend(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    if n < 2 {
        return signal.to_vec();
    }

    let n_f = n as f64;
    let sum_x: f64 = (0..n).map(|i| i as f64).sum();
    let sum_y: f64 = signal.iter().sum();
    let sum_xy: f64 = signal.iter().enumerate().map(|(i, &y)| i as f64 * y).sum();
    let sum_xx: f64 = (0..n).map(|i| (i as f64) * (i as f64)).sum();

    let denom = n_f * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-15 {
        return signal.to_vec();
    }

    let slope = (n_f * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / n_f;

    signal
        .iter()
        .enumerate()
        .map(|(i, &y)| y - (slope * i as f64 + intercept))
        .collect()
}

/// Compute the RMS (root mean square) acceleration of a signal in m/s^2.
pub fn rms_acceleration(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    let sum_sq: f64 = signal.iter().map(|&x| x * x).sum();
    (sum_sq / signal.len() as f64).sqrt()
}

/// Compute the peak absolute acceleration of a signal in m/s^2.
pub fn peak_acceleration(signal: &[f64]) -> f64 {
    signal
        .iter()
        .map(|x| x.abs())
        .fold(0.0_f64, f64::max)
}

/// Classify vibration damage risk per ISO 4866 guidelines.
///
/// Based on peak particle velocity (mm/s) and vibration frequency (Hz),
/// returns a damage risk classification string.
///
/// Categories follow DIN 4150-3 / ISO 4866 simplified thresholds:
/// - "Negligible": well below any damage threshold
/// - "Caution": approaching limits for sensitive structures
/// - "Warning": exceeding limits for residential/historic buildings
/// - "Severe": exceeding limits for commercial/industrial buildings
pub fn classify_vibration_iso4866(peak_velocity_mms: f64, freq_hz: f64) -> &'static str {
    // Simplified thresholds inspired by DIN 4150-3 Table 1
    // Low frequency (<10 Hz) structures are more vulnerable
    let threshold_residential;
    let threshold_commercial;

    if freq_hz < 10.0 {
        threshold_residential = 3.0; // mm/s
        threshold_commercial = 8.0;
    } else if freq_hz < 50.0 {
        // Linear interpolation between low and high frequency thresholds
        let frac = (freq_hz - 10.0) / 40.0;
        threshold_residential = 3.0 + frac * (8.0 - 3.0);
        threshold_commercial = 8.0 + frac * (20.0 - 8.0);
    } else {
        threshold_residential = 8.0;
        threshold_commercial = 20.0;
    }

    if peak_velocity_mms < threshold_residential * 0.5 {
        "Negligible"
    } else if peak_velocity_mms < threshold_residential {
        "Caution"
    } else if peak_velocity_mms < threshold_commercial {
        "Warning"
    } else {
        "Severe"
    }
}

/// Compute the operational deflection shape (ODS) from multi-channel time data
/// at a specific frequency.
///
/// This is conceptually similar to mode shape extraction but specifically
/// represents the actual vibration pattern at operating conditions (which may
/// be a superposition of multiple modes).
pub fn operational_deflection_shape(
    channels: &[Vec<f64>],
    freq_hz: f64,
    sample_rate: f64,
) -> Vec<f64> {
    // ODS at a single frequency is the DFT amplitude/phase pattern across channels
    mode_shape_extract(channels, freq_hz, sample_rate)
}

/// Convert peak velocity (mm/s) to peak displacement (mm) at a given frequency.
///
/// For sinusoidal vibration: displacement = velocity / (2 * pi * freq).
pub fn velocity_to_displacement(peak_velocity_mms: f64, freq_hz: f64) -> f64 {
    if freq_hz <= 0.0 {
        return 0.0;
    }
    peak_velocity_mms / (2.0 * PI * freq_hz)
}

/// Convert peak acceleration (m/s^2) to peak velocity (mm/s) at a given frequency.
///
/// For sinusoidal vibration: velocity = acceleration / (2 * pi * freq) * 1000.
pub fn acceleration_to_velocity(peak_accel_ms2: f64, freq_hz: f64) -> f64 {
    if freq_hz <= 0.0 {
        return 0.0;
    }
    (peak_accel_ms2 / (2.0 * PI * freq_hz)) * 1000.0
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a sinusoidal signal.
    fn sine_signal(freq_hz: f64, sample_rate: f64, duration_s: f64) -> Vec<f64> {
        let n = (sample_rate * duration_s) as usize;
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                (2.0 * PI * freq_hz * t).sin()
            })
            .collect()
    }

    /// Helper: generate a decaying sinusoid.
    fn decaying_sine(freq_hz: f64, sample_rate: f64, duration_s: f64, zeta: f64) -> Vec<f64> {
        let n = (sample_rate * duration_s) as usize;
        let omega_n = 2.0 * PI * freq_hz;
        let omega_d = omega_n * (1.0 - zeta * zeta).sqrt();
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                (-zeta * omega_n * t).exp() * (omega_d * t).sin()
            })
            .collect()
    }

    #[test]
    fn test_shm_config_creation() {
        let config = ShmConfig {
            sample_rate_hz: 1000.0,
            num_channels: 8,
            fft_size: 2048,
        };
        assert_eq!(config.sample_rate_hz, 1000.0);
        assert_eq!(config.num_channels, 8);
        assert_eq!(config.fft_size, 2048);
    }

    #[test]
    fn test_structural_monitor_new() {
        let config = ShmConfig {
            sample_rate_hz: 500.0,
            num_channels: 4,
            fft_size: 1024,
        };
        let monitor = StructuralMonitor::new(config);
        assert!(monitor.thresholds.is_empty());
        assert!(monitor.baseline.is_none());
        assert_eq!(monitor.config.num_channels, 4);
    }

    #[test]
    fn test_extract_single_frequency() {
        let sr = 1000.0;
        let signal = sine_signal(25.0, sr, 2.0);
        let freqs = extract_natural_frequencies(&signal, sr, 1);
        assert_eq!(freqs.len(), 1);
        assert!((freqs[0] - 25.0).abs() < 1.0, "Expected ~25 Hz, got {}", freqs[0]);
    }

    #[test]
    fn test_extract_multiple_frequencies() {
        let sr = 1000.0;
        let n = 2000;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sr;
                (2.0 * PI * 10.0 * t).sin() + 0.5 * (2.0 * PI * 30.0 * t).sin()
            })
            .collect();
        let freqs = extract_natural_frequencies(&signal, sr, 2);
        assert_eq!(freqs.len(), 2);
        // Should find both 10 Hz and 30 Hz
        assert!((freqs[0] - 10.0).abs() < 1.5, "Expected ~10 Hz, got {}", freqs[0]);
        assert!((freqs[1] - 30.0).abs() < 1.5, "Expected ~30 Hz, got {}", freqs[1]);
    }

    #[test]
    fn test_extract_empty_signal() {
        let freqs = extract_natural_frequencies(&[], 1000.0, 3);
        assert!(freqs.is_empty());
    }

    #[test]
    fn test_extract_zero_modes() {
        let signal = sine_signal(10.0, 1000.0, 1.0);
        let freqs = extract_natural_frequencies(&signal, 1000.0, 0);
        assert!(freqs.is_empty());
    }

    #[test]
    fn test_damping_log_decrement_known_zeta() {
        // Generate a decaying sinusoid with known damping
        let zeta_input = 0.05;
        let signal = decaying_sine(10.0, 1000.0, 2.0, zeta_input);
        let zeta_est = damping_log_decrement(&signal, 1000.0);
        // Should be close to 0.05
        assert!(
            (zeta_est - zeta_input).abs() < 0.02,
            "Expected ~{}, got {}",
            zeta_input,
            zeta_est
        );
    }

    #[test]
    fn test_damping_log_decrement_short_signal() {
        let signal = vec![1.0, 0.5];
        let zeta = damping_log_decrement(&signal, 1000.0);
        assert_eq!(zeta, 0.0);
    }

    #[test]
    fn test_damping_half_power() {
        // Create a simple peaked spectrum (Lorentzian)
        let mut spectrum = vec![0.0; 100];
        for k in 0..100 {
            let x = (k as f64 - 50.0) / 3.0;
            spectrum[k] = 1.0 / (1.0 + x * x);
        }
        let freq_res = 1.0; // 1 Hz per bin
        let zeta = damping_half_power(&spectrum, 50, freq_res);
        // The half-power bandwidth of a Lorentzian 1/(1+x^2) with width parameter 3
        // is 2*3 = 6 Hz. zeta = BW / (2 * f_center) = 6 / (2 * 50) = 0.06
        assert!(
            zeta > 0.01 && zeta < 0.15,
            "Expected reasonable damping ratio, got {}",
            zeta
        );
    }

    #[test]
    fn test_damping_half_power_empty() {
        assert_eq!(damping_half_power(&[], 0, 1.0), 0.0);
    }

    #[test]
    fn test_mode_shape_extract_uniform() {
        let sr = 1000.0;
        let freq = 20.0;
        // Three channels with the same signal should give mode shape [1, 1, 1]
        let ch = sine_signal(freq, sr, 1.0);
        let channels = vec![ch.clone(), ch.clone(), ch.clone()];
        let shape = mode_shape_extract(&channels, freq, sr);
        assert_eq!(shape.len(), 3);
        for &v in &shape {
            assert!(
                (v.abs() - 1.0).abs() < 0.1,
                "Expected ~1.0, got {}",
                v
            );
        }
    }

    #[test]
    fn test_mode_shape_extract_antiphase() {
        let sr = 1000.0;
        let freq = 20.0;
        let n = 1000;
        // Channel 0: positive sine, Channel 1: negative sine (180 deg out of phase)
        let ch0: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / sr).sin())
            .collect();
        let ch1: Vec<f64> = (0..n)
            .map(|i| -(2.0 * PI * freq * i as f64 / sr).sin())
            .collect();
        let channels = vec![ch0, ch1];
        let shape = mode_shape_extract(&channels, freq, sr);
        assert_eq!(shape.len(), 2);
        // They should have opposite signs
        assert!(
            shape[0] * shape[1] < 0.0,
            "Expected opposite signs: {:?}",
            shape
        );
    }

    #[test]
    fn test_mode_shape_empty_channels() {
        let shape = mode_shape_extract(&[], 10.0, 1000.0);
        assert!(shape.is_empty());
    }

    #[test]
    fn test_compute_damage_index_no_damage() {
        let baseline = vec![10.0, 20.0, 30.0];
        let current = vec![10.0, 20.0, 30.0];
        let indices = compute_damage_index(&baseline, &current);
        assert_eq!(indices.len(), 3);
        for idx in &indices {
            assert!(idx.severity < 1e-10, "Expected zero damage, got {}", idx.severity);
        }
    }

    #[test]
    fn test_compute_damage_index_with_damage() {
        let baseline = vec![10.0, 20.0, 30.0];
        let current = vec![9.0, 18.0, 27.0]; // 10% shift in all modes
        let indices = compute_damage_index(&baseline, &current);
        assert_eq!(indices.len(), 3);
        for idx in &indices {
            assert!(
                (idx.severity - 0.1).abs() < 0.01,
                "Expected ~0.1 severity, got {}",
                idx.severity
            );
        }
    }

    #[test]
    fn test_compute_damage_index_different_lengths() {
        let baseline = vec![10.0, 20.0];
        let current = vec![9.5];
        let indices = compute_damage_index(&baseline, &current);
        assert_eq!(indices.len(), 1);
    }

    #[test]
    fn test_integrate_to_displacement() {
        // Constant acceleration should give parabolic displacement
        let sr = 1000.0;
        let n = 1000;
        let accel = vec![1.0; n]; // 1 m/s^2 constant
        let disp = integrate_to_displacement(&accel, sr);
        assert_eq!(disp.len(), n);
        // After detrending a parabola, residuals should be present
        // Just verify it returns valid values
        for &d in &disp {
            assert!(d.is_finite(), "Displacement should be finite");
        }
    }

    #[test]
    fn test_integrate_to_displacement_sine() {
        // For a sinusoidal acceleration a = A*sin(wt),
        // displacement should be approximately -A/(w^2)*sin(wt)
        let sr = 1000.0;
        let freq = 5.0;
        let duration = 2.0;
        let accel = sine_signal(freq, sr, duration);
        let disp = integrate_to_displacement(&accel, sr);
        assert_eq!(disp.len(), accel.len());

        // The displacement should be sinusoidal with reduced amplitude
        let peak_disp = peak_acceleration(&disp); // reusing for peak abs value
        let expected_peak = 1.0 / (2.0 * PI * freq).powi(2);
        // Allow generous tolerance due to detrending effects at boundaries
        assert!(
            peak_disp < expected_peak * 2.0,
            "Displacement peak {} too large (expected ~{})",
            peak_disp,
            expected_peak
        );
    }

    #[test]
    fn test_integrate_empty() {
        let disp = integrate_to_displacement(&[], 1000.0);
        assert!(disp.is_empty());
    }

    #[test]
    fn test_rms_acceleration() {
        // RMS of a sine wave with amplitude A is A/sqrt(2)
        let signal = sine_signal(10.0, 1000.0, 1.0);
        let rms = rms_acceleration(&signal);
        let expected = 1.0 / 2.0_f64.sqrt();
        assert!(
            (rms - expected).abs() < 0.01,
            "Expected ~{}, got {}",
            expected,
            rms
        );
    }

    #[test]
    fn test_rms_acceleration_empty() {
        assert_eq!(rms_acceleration(&[]), 0.0);
    }

    #[test]
    fn test_peak_acceleration() {
        let signal = sine_signal(10.0, 1000.0, 1.0);
        let peak = peak_acceleration(&signal);
        assert!((peak - 1.0).abs() < 0.01, "Expected ~1.0, got {}", peak);
    }

    #[test]
    fn test_peak_acceleration_negative() {
        let signal = vec![-5.0, -3.0, -1.0, -4.0];
        assert!((peak_acceleration(&signal) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_classify_vibration_iso4866_negligible() {
        let class = classify_vibration_iso4866(0.5, 5.0);
        assert_eq!(class, "Negligible");
    }

    #[test]
    fn test_classify_vibration_iso4866_caution() {
        let class = classify_vibration_iso4866(2.0, 5.0);
        assert_eq!(class, "Caution");
    }

    #[test]
    fn test_classify_vibration_iso4866_warning() {
        let class = classify_vibration_iso4866(5.0, 5.0);
        assert_eq!(class, "Warning");
    }

    #[test]
    fn test_classify_vibration_iso4866_severe() {
        let class = classify_vibration_iso4866(25.0, 5.0);
        assert_eq!(class, "Severe");
    }

    #[test]
    fn test_classify_vibration_high_freq() {
        // At high frequency, thresholds are higher
        let class = classify_vibration_iso4866(5.0, 60.0);
        assert_eq!(class, "Caution");
    }

    #[test]
    fn test_alarm_thresholds() {
        let config = ShmConfig {
            sample_rate_hz: 1000.0,
            num_channels: 2,
            fft_size: 1024,
        };
        let mut monitor = StructuralMonitor::new(config);
        monitor.add_threshold(AlarmThreshold {
            name: "Residential".into(),
            peak_velocity_mms: 5.0,
            freq_min_hz: 1.0,
            freq_max_hz: 100.0,
            structure_class: "residential".into(),
        });
        monitor.add_threshold(AlarmThreshold {
            name: "Historic".into(),
            peak_velocity_mms: 2.0,
            freq_min_hz: 1.0,
            freq_max_hz: 50.0,
            structure_class: "historic".into(),
        });

        let results = monitor.check_alarms(3.0, 10.0);
        assert_eq!(results.len(), 2);
        // Residential (limit 5.0) not exceeded at 3.0
        assert_eq!(results[0].1, false);
        // Historic (limit 2.0) exceeded at 3.0
        assert_eq!(results[1].1, true);
    }

    #[test]
    fn test_alarm_out_of_freq_range() {
        let config = ShmConfig {
            sample_rate_hz: 1000.0,
            num_channels: 2,
            fft_size: 1024,
        };
        let mut monitor = StructuralMonitor::new(config);
        monitor.add_threshold(AlarmThreshold {
            name: "LowFreq".into(),
            peak_velocity_mms: 3.0,
            freq_min_hz: 1.0,
            freq_max_hz: 10.0,
            structure_class: "residential".into(),
        });

        // 50 Hz is outside the threshold range -> not exceeded
        let results = monitor.check_alarms(100.0, 50.0);
        assert_eq!(results[0].1, false);
    }

    #[test]
    fn test_clear_thresholds() {
        let config = ShmConfig {
            sample_rate_hz: 1000.0,
            num_channels: 1,
            fft_size: 512,
        };
        let mut monitor = StructuralMonitor::new(config);
        monitor.add_threshold(AlarmThreshold {
            name: "Test".into(),
            peak_velocity_mms: 1.0,
            freq_min_hz: 0.0,
            freq_max_hz: 100.0,
            structure_class: "test".into(),
        });
        assert_eq!(monitor.thresholds.len(), 1);
        monitor.clear_thresholds();
        assert!(monitor.thresholds.is_empty());
    }

    #[test]
    fn test_capacity_rating_good() {
        let config = ShmConfig {
            sample_rate_hz: 1000.0,
            num_channels: 1,
            fft_size: 1024,
        };
        let mut monitor = StructuralMonitor::new(config);
        monitor.set_baseline(ModalResult {
            natural_frequencies_hz: vec![10.0, 20.0, 30.0],
            damping_ratios: vec![0.02, 0.03, 0.04],
            mode_shapes: vec![vec![1.0], vec![1.0], vec![1.0]],
        });
        let rating = monitor.capacity_rating(&[10.0, 20.0, 30.0]).unwrap();
        assert!((rating.ratio - 1.0).abs() < 1e-10);
        assert_eq!(rating.category, "Good");
    }

    #[test]
    fn test_capacity_rating_critical() {
        let config = ShmConfig {
            sample_rate_hz: 1000.0,
            num_channels: 1,
            fft_size: 1024,
        };
        let mut monitor = StructuralMonitor::new(config);
        monitor.set_baseline(ModalResult {
            natural_frequencies_hz: vec![10.0, 20.0],
            damping_ratios: vec![0.02, 0.03],
            mode_shapes: vec![vec![1.0], vec![1.0]],
        });
        // 50% frequency drop -> critical
        let rating = monitor.capacity_rating(&[5.0, 10.0]).unwrap();
        assert!(rating.ratio < 0.70);
        assert_eq!(rating.category, "Critical");
    }

    #[test]
    fn test_capacity_rating_no_baseline() {
        let config = ShmConfig {
            sample_rate_hz: 1000.0,
            num_channels: 1,
            fft_size: 1024,
        };
        let monitor = StructuralMonitor::new(config);
        assert!(monitor.capacity_rating(&[10.0]).is_none());
    }

    #[test]
    fn test_full_analyze_workflow() {
        let sr = 1000.0;
        let config = ShmConfig {
            sample_rate_hz: sr,
            num_channels: 2,
            fft_size: 1024,
        };
        let mut monitor = StructuralMonitor::new(config);

        // Baseline: structure with 10 Hz mode
        let baseline_signal = sine_signal(10.0, sr, 2.0);
        let baseline_freqs = extract_natural_frequencies(&baseline_signal, sr, 1);
        monitor.set_baseline(ModalResult {
            natural_frequencies_hz: baseline_freqs,
            damping_ratios: vec![0.02],
            mode_shapes: vec![vec![1.0, 0.7]],
        });

        // Current measurement: slightly shifted frequency (damage)
        let ch0 = sine_signal(9.5, sr, 2.0);
        let ch1: Vec<f64> = sine_signal(9.5, sr, 2.0)
            .iter()
            .map(|x| x * 0.7)
            .collect();

        let (modal, damage) = monitor.analyze(&[ch0, ch1], 1);
        assert!(!modal.natural_frequencies_hz.is_empty());
        // Damage should be detected (frequency shift)
        if !damage.is_empty() {
            assert!(damage[0].severity > 0.0);
        }
    }

    #[test]
    fn test_velocity_to_displacement() {
        let disp = velocity_to_displacement(10.0, 5.0);
        let expected = 10.0 / (2.0 * PI * 5.0);
        assert!((disp - expected).abs() < 1e-10);
    }

    #[test]
    fn test_velocity_to_displacement_zero_freq() {
        assert_eq!(velocity_to_displacement(10.0, 0.0), 0.0);
    }

    #[test]
    fn test_acceleration_to_velocity() {
        let vel = acceleration_to_velocity(1.0, 10.0);
        let expected = (1.0 / (2.0 * PI * 10.0)) * 1000.0;
        assert!((vel - expected).abs() < 1e-10);
    }

    #[test]
    fn test_operational_deflection_shape() {
        let sr = 1000.0;
        let freq = 15.0;
        let ch0 = sine_signal(freq, sr, 1.0);
        let ch1: Vec<f64> = ch0.iter().map(|x| x * 0.5).collect();
        let ods = operational_deflection_shape(&[ch0, ch1], freq, sr);
        assert_eq!(ods.len(), 2);
        // Channel 0 should have larger contribution
        assert!(ods[0].abs() >= ods[1].abs());
    }

    #[test]
    fn test_linear_detrend() {
        // Linear signal should be zeroed out
        let signal: Vec<f64> = (0..100).map(|i| 2.0 * i as f64 + 5.0).collect();
        let detrended = linear_detrend(&signal);
        let max_residual = detrended.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
        assert!(max_residual < 1e-10, "Detrended linear signal should be ~0, max: {}", max_residual);
    }
}
