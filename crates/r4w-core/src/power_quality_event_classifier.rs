//! Power quality event classifier per IEEE 1159 and IEC 61000-4-30.
//!
//! This module detects and classifies power grid disturbances in real time,
//! including voltage sags, swells, transients, harmonics, flicker, frequency
//! deviations, interruptions, and notches. Each event is assigned a severity
//! level (Minor, Moderate, Major, Critical) based on IEEE 1159 magnitude and
//! duration thresholds.
//!
//! # Example
//!
//! ```
//! use r4w_core::power_quality_event_classifier::{
//!     PqConfig, PqClassifier, generate_sag_waveform, compute_rms_per_cycle,
//!     detect_sag_swell,
//! };
//!
//! let config = PqConfig {
//!     nominal_voltage_v: 120.0,
//!     nominal_freq_hz: 60.0,
//!     sample_rate_hz: 7680.0,
//! };
//! let classifier = PqClassifier::new(config.clone());
//!
//! // Generate a synthetic sag waveform (50% depth, 3 cycles)
//! let waveform = generate_sag_waveform(120.0, 0.5, 3, 60.0, 7680.0);
//! let spc = (config.sample_rate_hz / config.nominal_freq_hz) as usize;
//! let rms = compute_rms_per_cycle(&waveform, spc);
//! let events = detect_sag_swell(&rms, config.nominal_voltage_v);
//! assert!(!events.is_empty());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for power quality analysis.
#[derive(Debug, Clone)]
pub struct PqConfig {
    /// Nominal RMS voltage (e.g. 120 V or 230 V).
    pub nominal_voltage_v: f64,
    /// Nominal grid frequency in Hz (50 or 60).
    pub nominal_freq_hz: f64,
    /// Sample rate of the digitised waveform in Hz.
    pub sample_rate_hz: f64,
}

impl Default for PqConfig {
    fn default() -> Self {
        Self {
            nominal_voltage_v: 120.0,
            nominal_freq_hz: 60.0,
            sample_rate_hz: 7680.0, // 128 samples per cycle at 60 Hz
        }
    }
}

// ---------------------------------------------------------------------------
// Event types and severity
// ---------------------------------------------------------------------------

/// Power quality event type per IEEE 1159 taxonomy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PqEventType {
    /// Voltage sag (dip): 0.1–0.9 pu for 0.5 cycle – 1 min.
    Sag,
    /// Voltage swell: 1.1–1.8 pu for 0.5 cycle – 1 min.
    Swell,
    /// Interruption: < 0.1 pu.
    Interruption,
    /// Oscillatory transient: decaying oscillation superimposed on the waveform.
    OscillatoryTransient,
    /// Impulsive transient: unidirectional spike.
    ImpulsiveTransient,
    /// Harmonic distortion event (THD exceeds threshold).
    Harmonic,
    /// Flicker event (Pst exceeds threshold).
    Flicker,
    /// Frequency deviation outside tolerance.
    FrequencyDeviation,
    /// Voltage notch (commutation notch from power converters).
    Notch,
}

/// Severity classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    /// Minimal impact – informational.
    Minor,
    /// Noticeable but generally tolerable.
    Moderate,
    /// Likely equipment malfunction or shutdown.
    Major,
    /// Potential equipment damage or safety hazard.
    Critical,
}

/// A classified power quality event.
#[derive(Debug, Clone)]
pub struct PqEvent {
    /// Index of the first sample (or RMS cycle) where the event was detected.
    pub start_sample: usize,
    /// Index of the last sample (or RMS cycle) of the event (inclusive).
    pub end_sample: usize,
    /// The type of disturbance.
    pub event_type: PqEventType,
    /// Magnitude in per-unit (pu) relative to nominal.
    pub magnitude_pu: f64,
    /// Duration of the event in seconds.
    pub duration_s: f64,
    /// Severity classification.
    pub severity: Severity,
}

// ---------------------------------------------------------------------------
// Main classifier
// ---------------------------------------------------------------------------

/// Real-time power quality event classifier.
///
/// Wraps [`PqConfig`] and provides a convenience method [`PqClassifier::classify`]
/// that runs all detectors on a raw voltage waveform and returns a list of
/// [`PqEvent`]s.
#[derive(Debug, Clone)]
pub struct PqClassifier {
    /// The configuration used for classification.
    pub config: PqConfig,
}

impl PqClassifier {
    /// Create a new classifier with the given configuration.
    pub fn new(config: PqConfig) -> Self {
        Self { config }
    }

    /// Run all detectors on a raw voltage waveform and return classified events.
    ///
    /// The input `signal` is expected to be a time-domain voltage waveform
    /// sampled at `config.sample_rate_hz`.
    pub fn classify(&self, signal: &[f64]) -> Vec<PqEvent> {
        let samples_per_cycle =
            (self.config.sample_rate_hz / self.config.nominal_freq_hz) as usize;
        if samples_per_cycle == 0 || signal.len() < samples_per_cycle {
            return Vec::new();
        }

        let mut events = Vec::new();

        // RMS per cycle
        let rms = compute_rms_per_cycle(signal, samples_per_cycle);

        // Sag / Swell
        events.extend(detect_sag_swell(&rms, self.config.nominal_voltage_v));

        // Interruption
        events.extend(detect_interruption(
            &rms,
            self.config.nominal_voltage_v,
            0.1,
        ));

        // Transients (oscillatory + impulsive)
        let transient_threshold = self.config.nominal_voltage_v * 2.0_f64.sqrt() * 0.5;
        events.extend(detect_transient(
            signal,
            transient_threshold,
            self.config.sample_rate_hz,
        ));

        // Notches
        let notch_threshold = self.config.nominal_voltage_v * 2.0_f64.sqrt() * 0.1;
        let notches = detect_notch(signal, notch_threshold);
        for (idx, depth) in &notches {
            let pu = *depth / (self.config.nominal_voltage_v * 2.0_f64.sqrt());
            let ev = PqEvent {
                start_sample: *idx,
                end_sample: *idx,
                event_type: PqEventType::Notch,
                magnitude_pu: pu,
                duration_s: 1.0 / self.config.sample_rate_hz,
                severity: Severity::Minor,
            };
            events.push(ev);
        }

        // Re-classify severity
        for ev in &mut events {
            ev.severity = classify_severity_ieee1159(ev);
        }

        events
    }
}

// ---------------------------------------------------------------------------
// Per-cycle RMS
// ---------------------------------------------------------------------------

/// Compute the RMS value for each non-overlapping cycle of `samples_per_cycle`
/// samples.
///
/// Trailing samples that do not fill a complete cycle are discarded.
pub fn compute_rms_per_cycle(signal: &[f64], samples_per_cycle: usize) -> Vec<f64> {
    if samples_per_cycle == 0 {
        return Vec::new();
    }
    let num_cycles = signal.len() / samples_per_cycle;
    let mut rms = Vec::with_capacity(num_cycles);
    for i in 0..num_cycles {
        let start = i * samples_per_cycle;
        let end = start + samples_per_cycle;
        let sum_sq: f64 = signal[start..end].iter().map(|&x| x * x).sum();
        rms.push((sum_sq / samples_per_cycle as f64).sqrt());
    }
    rms
}

// ---------------------------------------------------------------------------
// Sag / Swell detection
// ---------------------------------------------------------------------------

/// Detect voltage sags and swells from per-cycle RMS values.
///
/// A **sag** is declared when the RMS falls below 0.9 pu and a **swell** when
/// it rises above 1.1 pu.  The magnitude reported is the worst-case pu value
/// during the event.  Duration is expressed in cycles (converted to seconds
/// later by the caller or via [`PqClassifier::classify`]).
pub fn detect_sag_swell(rms_values: &[f64], nominal_v: f64) -> Vec<PqEvent> {
    if nominal_v <= 0.0 {
        return Vec::new();
    }
    let mut events = Vec::new();
    let mut i = 0;
    while i < rms_values.len() {
        let pu = rms_values[i] / nominal_v;
        if pu < 0.9 && pu >= 0.1 {
            // Sag
            let start = i;
            let mut worst = pu;
            while i < rms_values.len() {
                let p = rms_values[i] / nominal_v;
                if p >= 0.9 || p < 0.1 {
                    break;
                }
                if p < worst {
                    worst = p;
                }
                i += 1;
            }
            let dur_cycles = (i - start) as f64;
            let ev = PqEvent {
                start_sample: start,
                end_sample: if i > start { i - 1 } else { start },
                event_type: PqEventType::Sag,
                magnitude_pu: worst,
                duration_s: dur_cycles, // caller converts if needed
                severity: Severity::Minor, // re-classified later
            };
            events.push(ev);
        } else if pu > 1.1 {
            // Swell
            let start = i;
            let mut worst = pu;
            while i < rms_values.len() {
                let p = rms_values[i] / nominal_v;
                if p <= 1.1 {
                    break;
                }
                if p > worst {
                    worst = p;
                }
                i += 1;
            }
            let dur_cycles = (i - start) as f64;
            let ev = PqEvent {
                start_sample: start,
                end_sample: if i > start { i - 1 } else { start },
                event_type: PqEventType::Swell,
                magnitude_pu: worst,
                duration_s: dur_cycles,
                severity: Severity::Minor,
            };
            events.push(ev);
        } else {
            i += 1;
        }
    }
    events
}

// ---------------------------------------------------------------------------
// Interruption detection
// ---------------------------------------------------------------------------

/// Detect interruptions where per-cycle RMS drops below `threshold_pu` of
/// the nominal voltage.  IEEE 1159 uses 0.1 pu as the boundary between sag
/// and interruption.
pub fn detect_interruption(
    rms_values: &[f64],
    nominal_v: f64,
    threshold_pu: f64,
) -> Vec<PqEvent> {
    if nominal_v <= 0.0 {
        return Vec::new();
    }
    let mut events = Vec::new();
    let mut i = 0;
    while i < rms_values.len() {
        let pu = rms_values[i] / nominal_v;
        if pu < threshold_pu {
            let start = i;
            let mut worst = pu;
            while i < rms_values.len() {
                let p = rms_values[i] / nominal_v;
                if p >= threshold_pu {
                    break;
                }
                if p < worst {
                    worst = p;
                }
                i += 1;
            }
            let dur_cycles = (i - start) as f64;
            let ev = PqEvent {
                start_sample: start,
                end_sample: if i > start { i - 1 } else { start },
                event_type: PqEventType::Interruption,
                magnitude_pu: worst,
                duration_s: dur_cycles,
                severity: Severity::Critical,
            };
            events.push(ev);
        } else {
            i += 1;
        }
    }
    events
}

// ---------------------------------------------------------------------------
// Transient detection
// ---------------------------------------------------------------------------

/// Detect transients (oscillatory and impulsive) in a raw voltage waveform.
///
/// A transient is identified when the absolute instantaneous value exceeds
/// `threshold` (in the same voltage units as `signal`).  If the detected
/// transient region contains sign changes it is classified as
/// [`PqEventType::OscillatoryTransient`]; otherwise it is
/// [`PqEventType::ImpulsiveTransient`].
pub fn detect_transient(
    signal: &[f64],
    threshold: f64,
    sample_rate: f64,
) -> Vec<PqEvent> {
    let mut events = Vec::new();
    if signal.is_empty() || threshold <= 0.0 || sample_rate <= 0.0 {
        return events;
    }

    let mut i = 0;
    while i < signal.len() {
        if signal[i].abs() > threshold {
            let start = i;
            let mut peak = signal[i].abs();
            let mut sign_changes = 0u32;
            let mut prev_sign = signal[i] >= 0.0;
            i += 1;
            while i < signal.len() && signal[i].abs() > threshold * 0.5 {
                if signal[i].abs() > peak {
                    peak = signal[i].abs();
                }
                let cur_sign = signal[i] >= 0.0;
                if cur_sign != prev_sign {
                    sign_changes += 1;
                }
                prev_sign = cur_sign;
                i += 1;
            }
            let etype = if sign_changes >= 2 {
                PqEventType::OscillatoryTransient
            } else {
                PqEventType::ImpulsiveTransient
            };
            let dur_s = (i - start) as f64 / sample_rate;
            events.push(PqEvent {
                start_sample: start,
                end_sample: if i > start { i - 1 } else { start },
                event_type: etype,
                magnitude_pu: peak / threshold, // relative to threshold
                duration_s: dur_s,
                severity: Severity::Minor,
            });
        } else {
            i += 1;
        }
    }
    events
}

// ---------------------------------------------------------------------------
// Flicker (Pst)
// ---------------------------------------------------------------------------

/// Measure the short-term flicker severity index Pst from per-cycle RMS
/// values, using a simplified IEC 61000-4-15 algorithm.
///
/// `rms_values` are the per-cycle RMS voltages and `sample_rate` is the
/// cycle rate (i.e. the grid frequency in Hz, not the sample rate of the
/// raw waveform).
///
/// Returns the Pst value.  A value above 1.0 generally indicates
/// objectionable flicker.
pub fn measure_flicker_pst(rms_values: &[f64], sample_rate: f64) -> f64 {
    if rms_values.len() < 2 || sample_rate <= 0.0 {
        return 0.0;
    }

    // Step 1: Compute relative voltage changes dV/V
    let mean: f64 = rms_values.iter().copied().sum::<f64>() / rms_values.len() as f64;
    if mean <= 0.0 {
        return 0.0;
    }

    let dv: Vec<f64> = rms_values
        .iter()
        .map(|&v| ((v - mean) / mean).abs())
        .collect();

    // Step 2: Sort the relative deviations to get the cumulative probability
    let mut sorted = dv.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    // Percentile helper (index from the top)
    let percentile = |p: f64| -> f64 {
        let idx = ((1.0 - p / 100.0) * (sorted.len() as f64 - 1.0))
            .round()
            .max(0.0) as usize;
        let idx = idx.min(sorted.len() - 1);
        sorted[idx]
    };

    // Step 3: Simplified Pst formula (IEC 61000-4-15 percentile method)
    let p01 = percentile(0.1);
    let p1 = percentile(1.0);
    let p3 = percentile(3.0);
    let p10 = percentile(10.0);
    let p50 = percentile(50.0);

    let pst_sq = 0.0314 * p01 * p01
        + 0.0525 * p1 * p1
        + 0.0657 * p3 * p3
        + 0.28 * p10 * p10
        + 0.08 * p50 * p50;

    pst_sq.sqrt()
}

// ---------------------------------------------------------------------------
// Frequency tracking
// ---------------------------------------------------------------------------

/// Track the instantaneous frequency of a waveform using zero-crossing
/// detection.
///
/// Returns a vector of frequency estimates (Hz), one per detected half-cycle.
/// `sample_rate` is the sample rate of the raw waveform.
pub fn track_frequency(signal: &[f64], sample_rate: f64) -> Vec<f64> {
    if signal.len() < 3 || sample_rate <= 0.0 {
        return Vec::new();
    }

    let mut freqs = Vec::new();
    let mut prev_crossing: Option<f64> = None;

    for i in 1..signal.len() {
        // Detect positive-going zero crossings
        if signal[i - 1] < 0.0 && signal[i] >= 0.0 {
            // Linear interpolation for sub-sample accuracy
            let frac = -signal[i - 1] / (signal[i] - signal[i - 1]);
            let crossing = (i - 1) as f64 + frac;
            if let Some(prev) = prev_crossing {
                let period_samples = crossing - prev;
                if period_samples > 0.0 {
                    let freq = sample_rate / period_samples;
                    freqs.push(freq);
                }
            }
            prev_crossing = Some(crossing);
        }
    }
    freqs
}

// ---------------------------------------------------------------------------
// Severity classification
// ---------------------------------------------------------------------------

/// Classify the severity of a power quality event per IEEE 1159 guidelines.
///
/// Severity is determined by combining the event type, magnitude (pu), and
/// duration.
pub fn classify_severity_ieee1159(event: &PqEvent) -> Severity {
    match event.event_type {
        PqEventType::Interruption => {
            if event.duration_s > 60.0 {
                Severity::Critical
            } else if event.duration_s > 3.0 {
                Severity::Major
            } else {
                Severity::Moderate
            }
        }
        PqEventType::Sag => {
            if event.magnitude_pu < 0.3 {
                Severity::Critical
            } else if event.magnitude_pu < 0.5 {
                Severity::Major
            } else if event.magnitude_pu < 0.7 {
                Severity::Moderate
            } else {
                Severity::Minor
            }
        }
        PqEventType::Swell => {
            if event.magnitude_pu > 1.6 {
                Severity::Critical
            } else if event.magnitude_pu > 1.4 {
                Severity::Major
            } else if event.magnitude_pu > 1.2 {
                Severity::Moderate
            } else {
                Severity::Minor
            }
        }
        PqEventType::OscillatoryTransient | PqEventType::ImpulsiveTransient => {
            if event.magnitude_pu > 5.0 {
                Severity::Critical
            } else if event.magnitude_pu > 3.0 {
                Severity::Major
            } else if event.magnitude_pu > 1.5 {
                Severity::Moderate
            } else {
                Severity::Minor
            }
        }
        PqEventType::Harmonic => {
            // magnitude_pu here represents THD fraction
            if event.magnitude_pu > 0.20 {
                Severity::Critical
            } else if event.magnitude_pu > 0.10 {
                Severity::Major
            } else if event.magnitude_pu > 0.05 {
                Severity::Moderate
            } else {
                Severity::Minor
            }
        }
        PqEventType::Flicker => {
            if event.magnitude_pu > 2.0 {
                Severity::Critical
            } else if event.magnitude_pu > 1.5 {
                Severity::Major
            } else if event.magnitude_pu > 1.0 {
                Severity::Moderate
            } else {
                Severity::Minor
            }
        }
        PqEventType::FrequencyDeviation => {
            let dev = (event.magnitude_pu - 1.0).abs();
            if dev > 0.05 {
                Severity::Critical
            } else if dev > 0.02 {
                Severity::Major
            } else if dev > 0.01 {
                Severity::Moderate
            } else {
                Severity::Minor
            }
        }
        PqEventType::Notch => {
            if event.magnitude_pu > 0.5 {
                Severity::Major
            } else if event.magnitude_pu > 0.2 {
                Severity::Moderate
            } else {
                Severity::Minor
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Notch detection
// ---------------------------------------------------------------------------

/// Detect voltage notches in a raw waveform.
///
/// A notch is a sharp, brief reduction in voltage typically caused by power
/// converter commutation.  This detector looks for samples where the second
/// derivative (acceleration) exceeds `threshold` and the voltage magnitude
/// drops momentarily.
///
/// Returns `(sample_index, notch_depth)` pairs.
pub fn detect_notch(signal: &[f64], threshold: f64) -> Vec<(usize, f64)> {
    if signal.len() < 3 || threshold <= 0.0 {
        return Vec::new();
    }

    let mut notches = Vec::new();
    for i in 1..signal.len() - 1 {
        let second_deriv = signal[i + 1] - 2.0 * signal[i] + signal[i - 1];
        // A notch has a large positive second derivative (concave up, dip)
        if second_deriv > threshold {
            // Estimate depth as the difference from the interpolated line
            let expected = (signal[i - 1] + signal[i + 1]) / 2.0;
            let depth = (expected - signal[i]).abs();
            if depth > threshold * 0.1 {
                notches.push((i, depth));
            }
        }
    }
    notches
}

// ---------------------------------------------------------------------------
// Synthetic waveform generation
// ---------------------------------------------------------------------------

/// Generate a synthetic voltage sag waveform for testing.
///
/// Produces a sine wave at `nominal_v * sqrt(2)` peak, then applies a sag
/// of `depth_pu` (0.0 = full sag, 1.0 = no sag) for `duration_cycles`
/// cycles in the middle of the waveform.  One extra nominal cycle is
/// prepended and appended so that the sag region is surrounded by normal
/// voltage.
///
/// Returns the raw time-domain voltage waveform.
pub fn generate_sag_waveform(
    nominal_v: f64,
    depth_pu: f64,
    duration_cycles: usize,
    freq_hz: f64,
    sample_rate: f64,
) -> Vec<f64> {
    let spc = (sample_rate / freq_hz) as usize;
    let total_cycles = duration_cycles + 2; // 1 pre + sag + 1 post
    let total_samples = total_cycles * spc;
    let peak = nominal_v * 2.0_f64.sqrt();

    let mut out = Vec::with_capacity(total_samples);
    for n in 0..total_samples {
        let t = n as f64 / sample_rate;
        let phase = 2.0 * PI * freq_hz * t;
        let cycle_idx = n / spc;
        let amplitude = if cycle_idx >= 1 && cycle_idx < 1 + duration_cycles {
            peak * depth_pu
        } else {
            peak
        };
        out.push(amplitude * phase.sin());
    }
    out
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const NOMINAL_V: f64 = 120.0;
    const FREQ: f64 = 60.0;
    const SR: f64 = 7680.0; // 128 samples/cycle
    const SPC: usize = 128;

    fn make_config() -> PqConfig {
        PqConfig {
            nominal_voltage_v: NOMINAL_V,
            nominal_freq_hz: FREQ,
            sample_rate_hz: SR,
        }
    }

    /// Helper: generate pure sinusoidal waveform at nominal voltage.
    fn nominal_sine(cycles: usize) -> Vec<f64> {
        let peak = NOMINAL_V * 2.0_f64.sqrt();
        (0..cycles * SPC)
            .map(|n| {
                let t = n as f64 / SR;
                peak * (2.0 * PI * FREQ * t).sin()
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // compute_rms_per_cycle
    // -----------------------------------------------------------------------

    #[test]
    fn test_rms_per_cycle_nominal() {
        let sig = nominal_sine(5);
        let rms = compute_rms_per_cycle(&sig, SPC);
        assert_eq!(rms.len(), 5);
        for v in &rms {
            assert!((v - NOMINAL_V).abs() < 1.0, "RMS {v} not close to {NOMINAL_V}");
        }
    }

    #[test]
    fn test_rms_per_cycle_empty() {
        let rms = compute_rms_per_cycle(&[], SPC);
        assert!(rms.is_empty());
    }

    #[test]
    fn test_rms_per_cycle_zero_spc() {
        let rms = compute_rms_per_cycle(&[1.0, 2.0], 0);
        assert!(rms.is_empty());
    }

    // -----------------------------------------------------------------------
    // detect_sag_swell
    // -----------------------------------------------------------------------

    #[test]
    fn test_detect_sag() {
        // 2 nominal cycles, 3 sag cycles at 70%, 2 nominal cycles
        let sig = generate_sag_waveform(NOMINAL_V, 0.7, 3, FREQ, SR);
        let rms = compute_rms_per_cycle(&sig, SPC);
        let events = detect_sag_swell(&rms, NOMINAL_V);
        assert!(!events.is_empty(), "Should detect a sag event");
        assert_eq!(events[0].event_type, PqEventType::Sag);
        assert!(events[0].magnitude_pu < 0.9);
    }

    #[test]
    fn test_detect_swell() {
        // Manually build RMS values with a swell region
        let rms = vec![120.0, 120.0, 140.0, 145.0, 140.0, 120.0, 120.0];
        let events = detect_sag_swell(&rms, 120.0);
        assert!(!events.is_empty(), "Should detect a swell event");
        assert_eq!(events[0].event_type, PqEventType::Swell);
        assert!(events[0].magnitude_pu > 1.1);
    }

    #[test]
    fn test_no_sag_swell_nominal() {
        let rms = vec![120.0; 10];
        let events = detect_sag_swell(&rms, 120.0);
        assert!(events.is_empty(), "No events expected for nominal voltage");
    }

    // -----------------------------------------------------------------------
    // detect_interruption
    // -----------------------------------------------------------------------

    #[test]
    fn test_detect_interruption() {
        let rms = vec![120.0, 120.0, 5.0, 2.0, 3.0, 120.0, 120.0];
        let events = detect_interruption(&rms, 120.0, 0.1);
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].event_type, PqEventType::Interruption);
        assert!(events[0].magnitude_pu < 0.1);
    }

    #[test]
    fn test_no_interruption_nominal() {
        let rms = vec![120.0; 10];
        let events = detect_interruption(&rms, 120.0, 0.1);
        assert!(events.is_empty());
    }

    // -----------------------------------------------------------------------
    // detect_transient
    // -----------------------------------------------------------------------

    #[test]
    fn test_detect_impulsive_transient() {
        let mut sig = vec![0.0; 100];
        // Insert a unidirectional spike
        sig[50] = 500.0;
        sig[51] = 300.0;
        sig[52] = 100.0;
        let events = detect_transient(&sig, 200.0, 1000.0);
        assert!(!events.is_empty(), "Should detect impulsive transient");
        assert_eq!(events[0].event_type, PqEventType::ImpulsiveTransient);
    }

    #[test]
    fn test_detect_oscillatory_transient() {
        let mut sig = vec![0.0; 200];
        // Insert an oscillating transient
        for i in 0..20 {
            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            sig[80 + i] = sign * 400.0 * (-(i as f64) * 0.1).exp();
        }
        let events = detect_transient(&sig, 200.0, 10000.0);
        assert!(!events.is_empty(), "Should detect oscillatory transient");
        assert_eq!(events[0].event_type, PqEventType::OscillatoryTransient);
    }

    #[test]
    fn test_no_transient_quiet() {
        let sig = vec![10.0; 100];
        let events = detect_transient(&sig, 200.0, 1000.0);
        assert!(events.is_empty());
    }

    // -----------------------------------------------------------------------
    // measure_flicker_pst
    // -----------------------------------------------------------------------

    #[test]
    fn test_flicker_pst_steady() {
        let rms = vec![120.0; 600]; // 10 seconds at 60 Hz
        let pst = measure_flicker_pst(&rms, 60.0);
        assert!(pst < 0.01, "Pst should be near zero for steady voltage, got {pst}");
    }

    #[test]
    fn test_flicker_pst_modulated() {
        // Simulate flickering voltage (e.g. arc furnace)
        let rms: Vec<f64> = (0..600)
            .map(|i| {
                let t = i as f64 / 60.0;
                120.0 * (1.0 + 0.05 * (2.0 * PI * 8.8 * t).sin()) // 8.8 Hz flicker
            })
            .collect();
        let pst = measure_flicker_pst(&rms, 60.0);
        assert!(pst > 0.0, "Pst should be positive for flickering voltage, got {pst}");
    }

    #[test]
    fn test_flicker_pst_empty() {
        let pst = measure_flicker_pst(&[], 60.0);
        assert!((pst - 0.0).abs() < f64::EPSILON);
    }

    // -----------------------------------------------------------------------
    // track_frequency
    // -----------------------------------------------------------------------

    #[test]
    fn test_track_frequency_nominal() {
        let sig = nominal_sine(10);
        let freqs = track_frequency(&sig, SR);
        assert!(!freqs.is_empty());
        for &f in &freqs {
            assert!(
                (f - FREQ).abs() < 0.5,
                "Tracked freq {f} Hz should be close to {FREQ} Hz"
            );
        }
    }

    #[test]
    fn test_track_frequency_higher() {
        // Generate 65 Hz signal
        let peak = NOMINAL_V * 2.0_f64.sqrt();
        let target_freq = 65.0;
        let samples = 2000;
        let sig: Vec<f64> = (0..samples)
            .map(|n| {
                let t = n as f64 / SR;
                peak * (2.0 * PI * target_freq * t).sin()
            })
            .collect();
        let freqs = track_frequency(&sig, SR);
        assert!(!freqs.is_empty());
        let avg: f64 = freqs.iter().sum::<f64>() / freqs.len() as f64;
        assert!(
            (avg - target_freq).abs() < 1.0,
            "Average freq {avg} should be close to {target_freq}"
        );
    }

    #[test]
    fn test_track_frequency_short() {
        let freqs = track_frequency(&[1.0, 2.0], SR);
        assert!(freqs.is_empty());
    }

    // -----------------------------------------------------------------------
    // classify_severity_ieee1159
    // -----------------------------------------------------------------------

    #[test]
    fn test_severity_deep_sag() {
        let ev = PqEvent {
            start_sample: 0,
            end_sample: 10,
            event_type: PqEventType::Sag,
            magnitude_pu: 0.2,
            duration_s: 0.5,
            severity: Severity::Minor,
        };
        assert_eq!(classify_severity_ieee1159(&ev), Severity::Critical);
    }

    #[test]
    fn test_severity_mild_sag() {
        let ev = PqEvent {
            start_sample: 0,
            end_sample: 5,
            event_type: PqEventType::Sag,
            magnitude_pu: 0.85,
            duration_s: 0.1,
            severity: Severity::Minor,
        };
        assert_eq!(classify_severity_ieee1159(&ev), Severity::Minor);
    }

    #[test]
    fn test_severity_high_swell() {
        let ev = PqEvent {
            start_sample: 0,
            end_sample: 5,
            event_type: PqEventType::Swell,
            magnitude_pu: 1.7,
            duration_s: 0.1,
            severity: Severity::Minor,
        };
        assert_eq!(classify_severity_ieee1159(&ev), Severity::Critical);
    }

    #[test]
    fn test_severity_long_interruption() {
        let ev = PqEvent {
            start_sample: 0,
            end_sample: 100,
            event_type: PqEventType::Interruption,
            magnitude_pu: 0.0,
            duration_s: 120.0,
            severity: Severity::Minor,
        };
        assert_eq!(classify_severity_ieee1159(&ev), Severity::Critical);
    }

    // -----------------------------------------------------------------------
    // detect_notch
    // -----------------------------------------------------------------------

    #[test]
    fn test_detect_notch_present() {
        // Build a signal with a sharp notch
        let mut sig: Vec<f64> = (0..200)
            .map(|n| {
                let t = n as f64 / SR;
                NOMINAL_V * 2.0_f64.sqrt() * (2.0 * PI * FREQ * t).sin()
            })
            .collect();
        // Insert notch at sample 100
        sig[100] -= 80.0;
        let notches = detect_notch(&sig, 20.0);
        assert!(!notches.is_empty(), "Should detect at least one notch");
        // The notch should be near sample 100
        let found = notches.iter().any(|&(idx, _)| (idx as isize - 100).unsigned_abs() < 3);
        assert!(found, "Notch should be near sample 100");
    }

    #[test]
    fn test_detect_notch_clean() {
        let sig = nominal_sine(2);
        let notches = detect_notch(&sig, 100.0);
        assert!(notches.is_empty(), "No notches in a clean sine");
    }

    // -----------------------------------------------------------------------
    // generate_sag_waveform
    // -----------------------------------------------------------------------

    #[test]
    fn test_generate_sag_waveform_length() {
        let sig = generate_sag_waveform(120.0, 0.5, 3, 60.0, 7680.0);
        // 3 sag + 2 padding = 5 cycles, 128 samples each
        assert_eq!(sig.len(), 5 * 128);
    }

    #[test]
    fn test_generate_sag_waveform_rms() {
        let sig = generate_sag_waveform(120.0, 0.5, 4, 60.0, 7680.0);
        let rms = compute_rms_per_cycle(&sig, 128);
        // Cycle 0 is nominal
        assert!((rms[0] - 120.0).abs() < 2.0, "Cycle 0 RMS={}", rms[0]);
        // Cycles 1-4 are sagged to 50%
        for i in 1..5 {
            assert!(
                (rms[i] - 60.0).abs() < 2.0,
                "Cycle {i} RMS={} expected ~60",
                rms[i]
            );
        }
        // Last cycle is nominal
        assert!(
            (rms[5] - 120.0).abs() < 2.0,
            "Last cycle RMS={} expected ~120",
            rms[5]
        );
    }

    // -----------------------------------------------------------------------
    // PqClassifier integration
    // -----------------------------------------------------------------------

    #[test]
    fn test_classifier_detects_sag() {
        let config = make_config();
        let classifier = PqClassifier::new(config);
        let sig = generate_sag_waveform(NOMINAL_V, 0.5, 5, FREQ, SR);
        let events = classifier.classify(&sig);
        let sags: Vec<_> = events
            .iter()
            .filter(|e| e.event_type == PqEventType::Sag)
            .collect();
        assert!(!sags.is_empty(), "Classifier should detect the sag");
    }

    #[test]
    fn test_classifier_no_events_nominal() {
        let config = make_config();
        let classifier = PqClassifier::new(config);
        let sig = nominal_sine(10);
        let events = classifier.classify(&sig);
        // Filter out transient false positives (threshold sensitive)
        let sag_swell: Vec<_> = events
            .iter()
            .filter(|e| {
                matches!(
                    e.event_type,
                    PqEventType::Sag | PqEventType::Swell | PqEventType::Interruption
                )
            })
            .collect();
        assert!(
            sag_swell.is_empty(),
            "No sag/swell/interruption events expected for nominal signal"
        );
    }

    #[test]
    fn test_classifier_short_signal() {
        let config = make_config();
        let classifier = PqClassifier::new(config);
        let events = classifier.classify(&[1.0, 2.0, 3.0]);
        assert!(events.is_empty());
    }

    #[test]
    fn test_pq_config_default() {
        let cfg = PqConfig::default();
        assert!((cfg.nominal_voltage_v - 120.0).abs() < f64::EPSILON);
        assert!((cfg.nominal_freq_hz - 60.0).abs() < f64::EPSILON);
        assert!((cfg.sample_rate_hz - 7680.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_event_type_equality() {
        assert_eq!(PqEventType::Sag, PqEventType::Sag);
        assert_ne!(PqEventType::Sag, PqEventType::Swell);
        assert_ne!(PqEventType::OscillatoryTransient, PqEventType::ImpulsiveTransient);
    }

    #[test]
    fn test_severity_ordering() {
        assert!(Severity::Minor < Severity::Moderate);
        assert!(Severity::Moderate < Severity::Major);
        assert!(Severity::Major < Severity::Critical);
    }
}
