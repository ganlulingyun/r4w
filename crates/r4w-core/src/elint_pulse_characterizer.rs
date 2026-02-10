//! ELINT (Electronic Intelligence) radar pulse analysis for characterizing unknown radar emitters.
//!
//! This module provides tools for detecting radar pulses, extracting pulse descriptor words (PDW),
//! analyzing pulse repetition intervals (PRI), classifying intra-pulse modulation, recognizing
//! antenna scan patterns, and deinterleaving pulses from multiple emitters.
//!
//! # Overview
//!
//! The ELINT processing chain typically follows these stages:
//!
//! 1. **Pulse Detection** — Leading/trailing edge detection against a threshold
//! 2. **PDW Extraction** — Time of arrival (TOA), pulse width (PW), pulse amplitude (PA),
//!    angle of arrival (AOA), and carrier frequency
//! 3. **PRI Analysis** — Classify pulse repetition interval patterns (constant, stagger,
//!    jitter, dwell-switch)
//! 4. **Intra-Pulse Modulation** — Identify CW, LFM, Barker, or polyphase modulation
//! 5. **Scan Pattern Recognition** — Determine antenna scan type from amplitude envelopes
//! 6. **Emitter Deinterleaving** — Separate co-channel pulses from multiple emitters
//!
//! # Example
//!
//! ```
//! use r4w_core::elint_pulse_characterizer::{ElintProcessor, detect_pulses, analyze_pri, mean_pri};
//!
//! let sample_rate = 100_000.0; // 100 kHz
//! let threshold = 0.5;
//!
//! // Synthesize a simple pulsed signal: 3 pulses at regular intervals
//! let mut signal = vec![0.0_f64; 10000];
//! for pulse_idx in 0..3 {
//!     let start = 1000 + pulse_idx * 3000;
//!     for i in start..start + 200 {
//!         signal[i] = 1.0;
//!     }
//! }
//!
//! let pulses = detect_pulses(&signal, threshold, sample_rate);
//! assert_eq!(pulses.len(), 3);
//!
//! let toas: Vec<f64> = pulses.iter().map(|p| p.toa_s).collect();
//! let pri_pattern = analyze_pri(&toas);
//! let avg_pri = mean_pri(&toas);
//! assert!((avg_pri - 0.03).abs() < 0.001); // ~30 ms PRI
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Data types
// ---------------------------------------------------------------------------

/// A pulse descriptor word (PDW) capturing the measured parameters of a single radar pulse.
#[derive(Debug, Clone, PartialEq)]
pub struct PulseDescriptor {
    /// Time of arrival in seconds (leading edge).
    pub toa_s: f64,
    /// Pulse width in seconds (leading edge to trailing edge).
    pub pulse_width_s: f64,
    /// Peak amplitude (linear scale) within the pulse.
    pub amplitude: f64,
    /// Estimated carrier frequency in Hz (0.0 if not measured).
    pub frequency_hz: f64,
    /// Angle of arrival in degrees (0.0 if not measured).
    pub aoa_deg: f64,
}

/// Classification of the pulse repetition interval (PRI) pattern.
#[derive(Debug, Clone, PartialEq)]
pub enum PriPattern {
    /// Constant PRI — all intervals are approximately equal.
    Constant(f64),
    /// Stagger — a repeating sequence of distinct PRI values.
    Stagger(Vec<f64>),
    /// Jitter — PRI varies randomly about a mean with a given standard deviation.
    /// Fields: (mean_pri, std_dev).
    Jitter(f64, f64),
    /// Dwell-and-switch — groups of constant PRI, each with a count.
    /// Fields: Vec<(pri_value, count)>.
    DwellSwitch(Vec<(f64, usize)>),
}

/// Classification of intra-pulse modulation on a single pulse.
#[derive(Debug, Clone, PartialEq)]
pub enum IntraPulseMod {
    /// Continuous wave (unmodulated).
    Cw,
    /// Linear frequency modulation with the given bandwidth in Hz.
    Lfm(f64),
    /// Barker phase code of the given length (e.g., 5, 7, 11, 13).
    Barker(usize),
    /// Polyphase code (Frank, P1-P4, etc.).
    Polyphase,
    /// Unable to classify.
    Unknown,
}

/// Classification of the antenna scan pattern.
#[derive(Debug, Clone, PartialEq)]
pub enum ScanPattern {
    /// Circular (rotating) scan with the given period in seconds.
    Circular(f64),
    /// Sector scan with period (s) and sector width (degrees).
    Sector(f64, f64),
    /// Conical scan with period in seconds.
    Conical(f64),
    /// Raster (track-while-scan) pattern.
    Raster,
    /// Unable to classify.
    Unknown,
}

// ---------------------------------------------------------------------------
// ElintProcessor
// ---------------------------------------------------------------------------

/// Main ELINT processor that holds configuration and accumulated pulse data.
#[derive(Debug, Clone)]
pub struct ElintProcessor {
    /// Detection threshold (linear amplitude).
    pub threshold: f64,
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Collected pulse descriptors.
    pub pulses: Vec<PulseDescriptor>,
}

impl ElintProcessor {
    /// Create a new processor with the given detection threshold and sample rate.
    pub fn new(threshold: f64, sample_rate: f64) -> Self {
        Self {
            threshold,
            sample_rate,
            pulses: Vec::new(),
        }
    }

    /// Detect pulses in the given signal and append them to the internal store.
    /// Returns the number of new pulses detected.
    pub fn ingest(&mut self, signal: &[f64]) -> usize {
        let new_pulses = detect_pulses(signal, self.threshold, self.sample_rate);
        let count = new_pulses.len();
        self.pulses.extend(new_pulses);
        count
    }

    /// Analyze the PRI pattern of all ingested pulses.
    pub fn analyze_pri(&self) -> PriPattern {
        let toas: Vec<f64> = self.pulses.iter().map(|p| p.toa_s).collect();
        analyze_pri(&toas)
    }

    /// Clear all accumulated pulses.
    pub fn clear(&mut self) {
        self.pulses.clear();
    }

    /// Return the number of accumulated pulses.
    pub fn pulse_count(&self) -> usize {
        self.pulses.len()
    }
}

// ---------------------------------------------------------------------------
// Pulse detection
// ---------------------------------------------------------------------------

/// Detect pulses in an amplitude signal using leading/trailing edge detection.
///
/// Samples whose absolute value meets or exceeds `threshold` are considered "on".
/// A pulse is recorded from the leading edge to the trailing edge.
///
/// The `frequency_hz` and `aoa_deg` fields are set to `0.0` (not measurable from
/// a real-valued signal alone).
pub fn detect_pulses(signal: &[f64], threshold: f64, sample_rate: f64) -> Vec<PulseDescriptor> {
    let mut pulses = Vec::new();
    let mut in_pulse = false;
    let mut start_idx: usize = 0;
    let mut peak_amp: f64 = 0.0;

    for (i, &s) in signal.iter().enumerate() {
        let above = s.abs() >= threshold;
        if above && !in_pulse {
            in_pulse = true;
            start_idx = i;
            peak_amp = s.abs();
        } else if above && in_pulse {
            if s.abs() > peak_amp {
                peak_amp = s.abs();
            }
        } else if !above && in_pulse {
            in_pulse = false;
            pulses.push(PulseDescriptor {
                toa_s: start_idx as f64 / sample_rate,
                pulse_width_s: (i - start_idx) as f64 / sample_rate,
                amplitude: peak_amp,
                frequency_hz: 0.0,
                aoa_deg: 0.0,
            });
        }
    }

    if in_pulse {
        pulses.push(PulseDescriptor {
            toa_s: start_idx as f64 / sample_rate,
            pulse_width_s: (signal.len() - start_idx) as f64 / sample_rate,
            amplitude: peak_amp,
            frequency_hz: 0.0,
            aoa_deg: 0.0,
        });
    }

    pulses
}

// ---------------------------------------------------------------------------
// PRI analysis helpers
// ---------------------------------------------------------------------------

/// Compute the mean PRI from a sorted list of TOAs.
/// Returns `0.0` if fewer than 2 TOAs are provided.
pub fn mean_pri(toas: &[f64]) -> f64 {
    if toas.len() < 2 {
        return 0.0;
    }
    let intervals: Vec<f64> = toas.windows(2).map(|w| w[1] - w[0]).collect();
    intervals.iter().sum::<f64>() / intervals.len() as f64
}

/// Compute PRI jitter as a percentage of the mean PRI.
/// Returns `0.0` if fewer than 3 TOAs are provided.
pub fn pri_jitter_percent(toas: &[f64]) -> f64 {
    if toas.len() < 3 {
        return 0.0;
    }
    let intervals: Vec<f64> = toas.windows(2).map(|w| w[1] - w[0]).collect();
    let mean = intervals.iter().sum::<f64>() / intervals.len() as f64;
    if mean.abs() < 1e-15 {
        return 0.0;
    }
    let variance =
        intervals.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / intervals.len() as f64;
    (variance.sqrt() / mean) * 100.0
}

/// Compute the duty cycle from pulse width and PRI.
/// Returns a fraction in [0.0, 1.0]. Returns `0.0` if `pri_s` is zero.
pub fn duty_cycle(pulse_width_s: f64, pri_s: f64) -> f64 {
    if pri_s.abs() < 1e-15 {
        return 0.0;
    }
    (pulse_width_s / pri_s).clamp(0.0, 1.0)
}

/// Compute a histogram of PRI values from a list of TOAs.
///
/// Returns a vector of `(bin_center, count)` tuples.
pub fn histogram_pri(toas: &[f64], num_bins: usize) -> Vec<(f64, u64)> {
    if toas.len() < 2 || num_bins == 0 {
        return Vec::new();
    }
    let intervals: Vec<f64> = toas.windows(2).map(|w| w[1] - w[0]).collect();
    let min_val = intervals.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_val = intervals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

    if (max_val - min_val).abs() < 1e-15 {
        return vec![(min_val, intervals.len() as u64)];
    }

    let bin_width = (max_val - min_val) / num_bins as f64;
    let mut counts = vec![0u64; num_bins];

    for &iv in &intervals {
        let mut bin = ((iv - min_val) / bin_width) as usize;
        if bin >= num_bins {
            bin = num_bins - 1;
        }
        counts[bin] += 1;
    }

    counts
        .iter()
        .enumerate()
        .map(|(i, &c)| (min_val + (i as f64 + 0.5) * bin_width, c))
        .collect()
}

/// Analyze PRI pattern from a sorted list of time-of-arrival values.
///
/// Classification order: Constant, Stagger, DwellSwitch, Jitter.
pub fn analyze_pri(toas: &[f64]) -> PriPattern {
    if toas.len() < 2 {
        return PriPattern::Constant(0.0);
    }

    let intervals: Vec<f64> = toas.windows(2).map(|w| w[1] - w[0]).collect();
    let n = intervals.len();
    let mean = intervals.iter().sum::<f64>() / n as f64;

    if mean.abs() < 1e-15 {
        return PriPattern::Constant(0.0);
    }

    let variance = intervals.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n as f64;
    let std_dev = variance.sqrt();
    let cv = std_dev / mean;

    if cv < 0.02 {
        return PriPattern::Constant(mean);
    }

    if let Some(pattern) = detect_stagger(&intervals, 0.05) {
        return PriPattern::Stagger(pattern);
    }

    if let Some(dwells) = detect_dwell_switch(&intervals, 0.05) {
        if dwells.len() >= 2 {
            return PriPattern::DwellSwitch(dwells);
        }
    }

    PriPattern::Jitter(mean, std_dev)
}

fn detect_stagger(intervals: &[f64], rel_tol: f64) -> Option<Vec<f64>> {
    let n = intervals.len();
    if n < 4 {
        return None;
    }
    let max_period = 8.min(n / 2);

    'period: for period in 2..=max_period {
        let pattern: Vec<f64> = intervals[..period].to_vec();
        for (i, &iv) in intervals.iter().enumerate() {
            let expected = pattern[i % period];
            if expected.abs() < 1e-15 {
                continue 'period;
            }
            if (iv - expected).abs() / expected > rel_tol {
                continue 'period;
            }
        }
        let mut distinct = false;
        for i in 1..pattern.len() {
            if (pattern[i] - pattern[0]).abs() / pattern[0].abs().max(1e-15) > rel_tol {
                distinct = true;
                break;
            }
        }
        if distinct {
            return Some(pattern);
        }
    }
    None
}

fn detect_dwell_switch(intervals: &[f64], rel_tol: f64) -> Option<Vec<(f64, usize)>> {
    const MIN_DWELL_LEN: usize = 3;
    if intervals.len() < MIN_DWELL_LEN * 2 {
        return None;
    }

    let mut dwells: Vec<(f64, usize)> = Vec::new();
    let mut seg_start = 0;

    while seg_start < intervals.len() {
        let base = intervals[seg_start];
        let mut seg_end = seg_start + 1;
        while seg_end < intervals.len() {
            if base.abs() < 1e-15 {
                break;
            }
            if (intervals[seg_end] - base).abs() / base > rel_tol {
                break;
            }
            seg_end += 1;
        }
        let count = seg_end - seg_start;
        if count < MIN_DWELL_LEN {
            return None;
        }
        let seg_mean = intervals[seg_start..seg_end].iter().sum::<f64>() / count as f64;
        dwells.push((seg_mean, count));
        seg_start = seg_end;
    }

    if dwells.len() >= 2 {
        let first = dwells[0].0;
        let has_distinct = dwells
            .iter()
            .any(|(pri, _)| (pri - first).abs() / first.abs().max(1e-15) > rel_tol);
        if has_distinct {
            return Some(dwells);
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Intra-pulse modulation classification
// ---------------------------------------------------------------------------

/// Classify the intra-pulse modulation of a single pulse.
///
/// The input `pulse` should be baseband real-valued samples sampled at `sample_rate` Hz.
///
/// Detection order:
/// 1. Check for rectangular (BPSK) phase codes first (Barker, Polyphase).
/// 2. Check for LFM via zero-crossing rate difference between halves.
/// 3. Fall back to CW or Unknown.
pub fn classify_intra_pulse(pulse: &[f64], sample_rate: f64) -> IntraPulseMod {
    if pulse.len() < 4 {
        return IntraPulseMod::Unknown;
    }

    let n = pulse.len();

    // Compute RMS amplitude
    let rms = (pulse.iter().map(|&x| x * x).sum::<f64>() / n as f64).sqrt();

    // --- Step 1: Check for rectangular (BPSK-like) phase codes ---
    // Rectangular signals have most samples near +rms or -rms.
    let is_rectangular = if rms > 1e-10 {
        let near_level = pulse
            .iter()
            .filter(|&&x| (x.abs() - rms).abs() / rms < 0.3)
            .count();
        near_level as f64 / n as f64 > 0.8
    } else {
        false
    };

    if is_rectangular {
        // Find sign-transition positions
        let transitions = find_sign_transitions(pulse);

        if !transitions.is_empty() {
            // Build run boundaries: [0, first_transition+1, ..., n]
            let mut boundaries = Vec::with_capacity(transitions.len() + 2);
            boundaries.push(0usize);
            for &tp in &transitions {
                boundaries.push(tp + 1);
            }
            boundaries.push(n);

            let run_lengths: Vec<usize> =
                boundaries.windows(2).map(|w| w[1] - w[0]).collect();
            let min_run = *run_lengths.iter().min().unwrap();

            if min_run >= 2 {
                // Check that all runs are integer multiples of the minimum
                let all_multiples = run_lengths.iter().all(|&r| {
                    let ratio = r as f64 / min_run as f64;
                    (ratio - ratio.round()).abs() < 0.3
                });

                if all_multiples {
                    let estimated_chips = (n as f64 / min_run as f64).round() as usize;
                    let known_barker: [usize; 7] = [2, 3, 4, 5, 7, 11, 13];
                    for &bl in &known_barker {
                        if estimated_chips == bl {
                            return IntraPulseMod::Barker(bl);
                        }
                    }
                    if estimated_chips > 13 {
                        return IntraPulseMod::Polyphase;
                    }
                }
            }
        }

        // Rectangular with no transitions is CW
        if transitions.is_empty() {
            return IntraPulseMod::Cw;
        }
    }

    // --- Step 2: LFM detection via zero-crossing rate change ---
    let mid = n / 2;
    let zc_first = zero_crossing_count(&pulse[..mid]);
    let zc_second = zero_crossing_count(&pulse[mid..]);
    let total_zc = zc_first + zc_second;
    let total_zc_rate = total_zc as f64 / n as f64;

    if total_zc >= 4 {
        let zc_rate_first = zc_first as f64 / mid as f64;
        let zc_rate_second = zc_second as f64 / (n - mid) as f64;
        let zc_avg = (zc_rate_first + zc_rate_second) / 2.0;

        if zc_avg > 0.01 {
            let zc_ratio = if zc_rate_first > 1e-10 && zc_rate_second > 1e-10 {
                zc_rate_second.max(zc_rate_first) / zc_rate_second.min(zc_rate_first)
            } else if (zc_rate_first < 1e-10) != (zc_rate_second < 1e-10) {
                10.0
            } else {
                1.0
            };

            if zc_ratio > 1.5 {
                let f_low = zc_rate_first.min(zc_rate_second) * sample_rate / 2.0;
                let f_high = zc_rate_first.max(zc_rate_second) * sample_rate / 2.0;
                return IntraPulseMod::Lfm(f_high - f_low);
            }
        }
    }

    // --- Step 3: CW ---
    if total_zc_rate < 0.05 {
        return IntraPulseMod::Cw;
    }

    // --- Step 4: Non-rectangular phase codes ---
    let transitions = find_sign_transitions(pulse);
    if transitions.len() > 13 {
        if transitions.len() >= 2 {
            let spacings: Vec<usize> = transitions.windows(2).map(|w| w[1] - w[0]).collect();
            let avg = spacings.iter().sum::<usize>() as f64 / spacings.len() as f64;
            let var = spacings
                .iter()
                .map(|&s| (s as f64 - avg).powi(2))
                .sum::<f64>()
                / spacings.len() as f64;
            if var.sqrt() / avg.max(1e-15) < 1.0 {
                return IntraPulseMod::Polyphase;
            }
        }
    }

    IntraPulseMod::Unknown
}

/// Find indices where the signal changes sign.
fn find_sign_transitions(signal: &[f64]) -> Vec<usize> {
    let mut positions = Vec::new();
    for i in 0..signal.len().saturating_sub(1) {
        if signal[i].signum() != signal[i + 1].signum()
            && signal[i].abs() > 1e-10
            && signal[i + 1].abs() > 1e-10
        {
            positions.push(i);
        }
    }
    positions
}

/// Count zero crossings in a signal slice.
fn zero_crossing_count(signal: &[f64]) -> usize {
    find_sign_transitions(signal).len()
}

// ---------------------------------------------------------------------------
// Scan pattern estimation
// ---------------------------------------------------------------------------

/// Estimate antenna scan pattern from pulse amplitude envelopes and TOAs.
///
/// Looks for periodic amplitude modulation indicative of rotating or scanning antennas.
/// Returns [`ScanPattern::Unknown`] if no clear pattern is detected or fewer than
/// 6 measurements are provided.
pub fn estimate_scan_pattern(amplitudes: &[f64], toas: &[f64]) -> ScanPattern {
    if amplitudes.len() < 6 || toas.len() < 6 || amplitudes.len() != toas.len() {
        return ScanPattern::Unknown;
    }

    let n = amplitudes.len();
    let max_amp = amplitudes.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let min_amp = amplitudes.iter().cloned().fold(f64::INFINITY, f64::min);
    if (max_amp - min_amp).abs() < 1e-10 {
        return ScanPattern::Unknown;
    }

    let norm: Vec<f64> = amplitudes
        .iter()
        .map(|&a| (a - min_amp) / (max_amp - min_amp))
        .collect();

    let total_duration = toas[n - 1] - toas[0];
    if total_duration <= 0.0 {
        return ScanPattern::Unknown;
    }

    // Autocorrelation to find the scan period
    let mean_norm = norm.iter().sum::<f64>() / n as f64;
    let centered: Vec<f64> = norm.iter().map(|&x| x - mean_norm).collect();
    let energy: f64 = centered.iter().map(|&x| x * x).sum();
    if energy < 1e-15 {
        return ScanPattern::Unknown;
    }

    let max_lag = n / 2;
    let mut autocorr = vec![1.0_f64]; // lag 0

    for lag in 1..=max_lag {
        let mut corr = 0.0;
        for i in 0..(n - lag) {
            corr += centered[i] * centered[i + lag];
        }
        autocorr.push(corr / energy);
    }

    // Find the first peak after the initial decay.
    // The autocorrelation drops from 1.0; we look for where it first drops
    // below 0.5, then find the subsequent peak.
    let mut decay_end = 1;
    for lag in 1..autocorr.len() {
        if autocorr[lag] < 0.5 {
            decay_end = lag;
            break;
        }
    }

    let mut best_lag = 0;
    let mut best_corr = 0.0_f64;
    for lag in decay_end..autocorr.len() {
        if autocorr[lag] > best_corr {
            best_corr = autocorr[lag];
            best_lag = lag;
        }
    }

    if best_corr < 0.3 || best_lag == 0 {
        return ScanPattern::Unknown;
    }

    let avg_interval = total_duration / (n - 1) as f64;
    let period_s = best_lag as f64 * avg_interval;

    // Shape analysis within one period
    let period_samples = best_lag.min(n);
    let near_peak_in_period = norm[..period_samples].iter().filter(|&&x| x > 0.9).count();
    let near_peak_frac = near_peak_in_period as f64 / period_samples as f64;

    // Raster: asymmetric rise/fall
    let mut rising = 0usize;
    let mut falling = 0usize;
    for w in norm.windows(2) {
        if w[1] > w[0] + 0.01 {
            rising += 1;
        } else if w[0] > w[1] + 0.01 {
            falling += 1;
        }
    }
    let asymmetry = if rising > 0 && falling > 0 {
        (rising as f64 / falling as f64).max(falling as f64 / rising as f64)
    } else {
        1.0
    };
    if asymmetry > 3.0 {
        return ScanPattern::Raster;
    }

    // Sector: flat top (>40% of period at near-peak)
    if near_peak_frac > 0.4 {
        let sector_width = near_peak_frac * 360.0;
        return ScanPattern::Sector(period_s, sector_width);
    }

    // Conical: high zero-crossing rate relative to period
    let zc = centered
        .windows(2)
        .filter(|w| w[0].signum() != w[1].signum())
        .count();
    let zc_rate = zc as f64 / total_duration;
    if zc_rate > 4.0 / period_s {
        return ScanPattern::Conical(period_s);
    }

    ScanPattern::Circular(period_s)
}

// ---------------------------------------------------------------------------
// Emitter deinterleaving
// ---------------------------------------------------------------------------

/// Deinterleave pulses from multiple emitters based on PRI consistency.
///
/// Groups pulse indices into clusters where consecutive TOA differences
/// within each cluster are self-consistent (within `tolerance` of that
/// cluster's estimated PRI).
pub fn deinterleave(pulses: &[PulseDescriptor], tolerance: f64) -> Vec<Vec<usize>> {
    if pulses.is_empty() {
        return Vec::new();
    }
    if pulses.len() == 1 {
        return vec![vec![0]];
    }

    let mut groups: Vec<Vec<usize>> = Vec::new();
    let mut assigned = vec![false; pulses.len()];

    for start in 0..pulses.len() {
        if assigned[start] {
            continue;
        }
        let mut group = vec![start];
        assigned[start] = true;
        let mut last_idx = start;

        for candidate in (start + 1)..pulses.len() {
            if assigned[candidate] {
                continue;
            }
            let dt = pulses[candidate].toa_s - pulses[last_idx].toa_s;

            if group.len() == 1 {
                if dt > 0.0 {
                    group.push(candidate);
                    assigned[candidate] = true;
                    last_idx = candidate;
                }
            } else {
                let established_pri = pulses[group[1]].toa_s - pulses[group[0]].toa_s;
                if established_pri > 0.0 {
                    let ratio = dt / established_pri;
                    let nearest_int = ratio.round();
                    if nearest_int >= 1.0 {
                        let rel_err = (ratio - nearest_int).abs() / nearest_int;
                        if rel_err < tolerance {
                            group.push(candidate);
                            assigned[candidate] = true;
                            last_idx = candidate;
                        }
                    }
                }
            }
        }
        groups.push(group);
    }

    groups
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_pulse_signal(
        num_samples: usize,
        pulse_starts: &[usize],
        pulse_width: usize,
        amplitude: f64,
    ) -> Vec<f64> {
        let mut signal = vec![0.0; num_samples];
        for &start in pulse_starts {
            for i in start..start + pulse_width {
                if i < num_samples {
                    signal[i] = amplitude;
                }
            }
        }
        signal
    }

    #[test]
    fn test_detect_single_pulse() {
        let signal = make_pulse_signal(1000, &[200], 50, 1.0);
        let pulses = detect_pulses(&signal, 0.5, 1000.0);
        assert_eq!(pulses.len(), 1);
        assert!((pulses[0].toa_s - 0.2).abs() < 1e-6);
        assert!((pulses[0].pulse_width_s - 0.05).abs() < 1e-6);
        assert!((pulses[0].amplitude - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_detect_multiple_pulses() {
        let signal = make_pulse_signal(10000, &[500, 2500, 4500, 6500], 100, 0.8);
        let pulses = detect_pulses(&signal, 0.5, 10000.0);
        assert_eq!(pulses.len(), 4);
        for p in &pulses {
            assert!((p.pulse_width_s - 0.01).abs() < 1e-6);
            assert!((p.amplitude - 0.8).abs() < 1e-6);
        }
    }

    #[test]
    fn test_detect_no_pulses_below_threshold() {
        let signal = vec![0.1; 1000];
        let pulses = detect_pulses(&signal, 0.5, 1000.0);
        assert!(pulses.is_empty());
    }

    #[test]
    fn test_detect_pulse_at_end_of_buffer() {
        let mut signal = vec![0.0; 1000];
        for i in 900..1000 {
            signal[i] = 1.0;
        }
        let pulses = detect_pulses(&signal, 0.5, 1000.0);
        assert_eq!(pulses.len(), 1);
        assert!((pulses[0].toa_s - 0.9).abs() < 1e-6);
        assert!((pulses[0].pulse_width_s - 0.1).abs() < 1e-6);
    }

    #[test]
    fn test_detect_varying_amplitude_pulses() {
        let mut signal = vec![0.0; 3000];
        for i in 100..200 {
            signal[i] = 0.6;
        }
        for i in 1000..1100 {
            signal[i] = 1.5;
        }
        let pulses = detect_pulses(&signal, 0.5, 1000.0);
        assert_eq!(pulses.len(), 2);
        assert!((pulses[0].amplitude - 0.6).abs() < 1e-6);
        assert!((pulses[1].amplitude - 1.5).abs() < 1e-6);
    }

    #[test]
    fn test_mean_pri_constant() {
        let toas = vec![0.0, 0.01, 0.02, 0.03, 0.04];
        assert!((mean_pri(&toas) - 0.01).abs() < 1e-10);
    }

    #[test]
    fn test_mean_pri_too_few() {
        assert_eq!(mean_pri(&[1.0]), 0.0);
        assert_eq!(mean_pri(&[]), 0.0);
    }

    #[test]
    fn test_pri_jitter_percent_constant() {
        let toas = vec![0.0, 0.01, 0.02, 0.03, 0.04, 0.05];
        let jitter = pri_jitter_percent(&toas);
        assert!(jitter < 0.01, "Constant PRI should have near-zero jitter, got {jitter}");
    }

    #[test]
    fn test_pri_jitter_percent_variable() {
        let toas = vec![0.0, 0.010, 0.025, 0.032, 0.048, 0.055];
        let jitter = pri_jitter_percent(&toas);
        assert!(jitter > 1.0, "Variable PRI should have measurable jitter, got {jitter}");
    }

    #[test]
    fn test_duty_cycle() {
        assert!((duty_cycle(0.001, 0.01) - 0.1).abs() < 1e-10);
        assert!((duty_cycle(0.005, 0.01) - 0.5).abs() < 1e-10);
        assert_eq!(duty_cycle(1.0, 0.0), 0.0);
        assert!((duty_cycle(0.02, 0.01) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_analyze_pri_constant() {
        let toas: Vec<f64> = (0..20).map(|i| i as f64 * 0.001).collect();
        match analyze_pri(&toas) {
            PriPattern::Constant(pri) => {
                assert!((pri - 0.001).abs() < 1e-6, "Expected ~0.001, got {pri}");
            }
            other => panic!("Expected Constant, got {:?}", other),
        }
    }

    #[test]
    fn test_analyze_pri_stagger() {
        let mut toas = vec![0.0];
        let mut t = 0.0;
        for i in 0..20 {
            t += if i % 2 == 0 { 0.001 } else { 0.002 };
            toas.push(t);
        }
        match analyze_pri(&toas) {
            PriPattern::Stagger(pattern) => {
                assert_eq!(pattern.len(), 2);
                assert!((pattern[0] - 0.001).abs() < 1e-5);
                assert!((pattern[1] - 0.002).abs() < 1e-5);
            }
            other => panic!("Expected Stagger, got {:?}", other),
        }
    }

    #[test]
    fn test_analyze_pri_jitter() {
        let pris = [
            0.00095, 0.00107, 0.00098, 0.00103, 0.00092, 0.00111, 0.00096, 0.00105, 0.00099,
            0.00101, 0.00094, 0.00108, 0.00097, 0.00102, 0.00093, 0.00106, 0.00098, 0.00104,
            0.00100, 0.00095,
        ];
        let mut toas = vec![0.0];
        let mut t = 0.0;
        for &p in &pris {
            t += p;
            toas.push(t);
        }
        match analyze_pri(&toas) {
            PriPattern::Jitter(mean, std) => {
                assert!((mean - 0.001).abs() < 0.0002, "Expected ~0.001, got {mean}");
                assert!(std > 0.0);
            }
            other => panic!("Expected Jitter, got {:?}", other),
        }
    }

    #[test]
    fn test_histogram_pri() {
        let toas = vec![0.0, 0.01, 0.02, 0.03, 0.04, 0.05];
        let hist = histogram_pri(&toas, 10);
        assert_eq!(hist.len(), 1);
        assert_eq!(hist[0].1, 5);
    }

    #[test]
    fn test_histogram_pri_multi_bin() {
        let toas = vec![0.0, 0.01, 0.03, 0.04, 0.06, 0.07];
        let hist = histogram_pri(&toas, 4);
        assert!(!hist.is_empty());
        let total: u64 = hist.iter().map(|(_, c)| c).sum();
        assert_eq!(total, 5);
    }

    #[test]
    fn test_classify_cw_pulse() {
        let pulse = vec![1.0; 200];
        assert_eq!(classify_intra_pulse(&pulse, 1e6), IntraPulseMod::Cw);
    }

    #[test]
    fn test_classify_lfm_pulse() {
        let n = 500;
        let sample_rate = 1e6;
        let bw = 1e5;
        let duration = n as f64 / sample_rate;
        let chirp_rate = bw / duration;
        let pulse: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                (2.0 * PI * (0.5 * chirp_rate * t * t)).sin()
            })
            .collect();
        match classify_intra_pulse(&pulse, sample_rate) {
            IntraPulseMod::Lfm(_) => {}
            other => panic!("Expected Lfm, got {:?}", other),
        }
    }

    #[test]
    fn test_classify_barker_13() {
        let barker13 = [1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1];
        let samples_per_chip = 20;
        let pulse: Vec<f64> = barker13
            .iter()
            .flat_map(|&c| vec![c as f64; samples_per_chip])
            .collect();
        assert_eq!(classify_intra_pulse(&pulse, 1e6), IntraPulseMod::Barker(13));
    }

    #[test]
    fn test_classify_unknown_short() {
        let pulse = vec![1.0, -1.0, 0.5];
        assert_eq!(classify_intra_pulse(&pulse, 1e6), IntraPulseMod::Unknown);
    }

    #[test]
    fn test_scan_pattern_circular() {
        let n = 200;
        let period = 4.0;
        let toas: Vec<f64> = (0..n).map(|i| i as f64 * 0.05).collect();
        let amplitudes: Vec<f64> = toas
            .iter()
            .map(|&t| 0.5 + 0.5 * (2.0 * PI * t / period).sin())
            .collect();
        match estimate_scan_pattern(&amplitudes, &toas) {
            ScanPattern::Circular(p) => {
                assert!((p - period).abs() < 1.0, "Expected ~{period}, got {p}");
            }
            other => panic!("Expected Circular, got {:?}", other),
        }
    }

    #[test]
    fn test_scan_pattern_unknown_flat() {
        let n = 50;
        let toas: Vec<f64> = (0..n).map(|i| i as f64 * 0.01).collect();
        assert_eq!(estimate_scan_pattern(&vec![1.0; n], &toas), ScanPattern::Unknown);
    }

    #[test]
    fn test_scan_pattern_too_few_samples() {
        assert_eq!(estimate_scan_pattern(&[1.0, 2.0], &[0.0, 1.0]), ScanPattern::Unknown);
    }

    #[test]
    fn test_deinterleave_single_emitter() {
        let pulses: Vec<PulseDescriptor> = (0..10)
            .map(|i| PulseDescriptor {
                toa_s: i as f64 * 0.001,
                pulse_width_s: 0.0001,
                amplitude: 1.0,
                frequency_hz: 1e9,
                aoa_deg: 45.0,
            })
            .collect();
        let groups = deinterleave(&pulses, 0.1);
        assert_eq!(groups.len(), 1);
        assert_eq!(groups[0].len(), 10);
    }

    #[test]
    fn test_deinterleave_two_emitters() {
        let mut pulses = Vec::new();
        for i in 0..8 {
            pulses.push(PulseDescriptor {
                toa_s: i as f64 * 0.001,
                pulse_width_s: 0.0001,
                amplitude: 1.0,
                frequency_hz: 1e9,
                aoa_deg: 0.0,
            });
        }
        for i in 0..6 {
            pulses.push(PulseDescriptor {
                toa_s: 0.0003 + i as f64 * 0.0015,
                pulse_width_s: 0.0002,
                amplitude: 0.8,
                frequency_hz: 2e9,
                aoa_deg: 90.0,
            });
        }
        pulses.sort_by(|a, b| a.toa_s.partial_cmp(&b.toa_s).unwrap());
        let groups = deinterleave(&pulses, 0.15);
        assert!(groups.len() >= 2, "Expected >=2 groups, got {}", groups.len());
    }

    #[test]
    fn test_deinterleave_empty() {
        assert!(deinterleave(&[], 0.1).is_empty());
    }

    #[test]
    fn test_processor_ingest_and_analyze() {
        let mut proc = ElintProcessor::new(0.5, 10000.0);
        let signal = make_pulse_signal(10000, &[500, 2500, 4500, 6500, 8500], 100, 1.0);
        assert_eq!(proc.ingest(&signal), 5);
        assert_eq!(proc.pulse_count(), 5);
        match proc.analyze_pri() {
            PriPattern::Constant(pri) => {
                assert!((pri - 0.2).abs() < 0.01, "Expected ~0.2, got {pri}");
            }
            other => panic!("Expected Constant, got {:?}", other),
        }
    }

    #[test]
    fn test_processor_clear() {
        let mut proc = ElintProcessor::new(0.5, 10000.0);
        proc.ingest(&make_pulse_signal(2000, &[100, 500, 900], 50, 1.0));
        assert_eq!(proc.pulse_count(), 3);
        proc.clear();
        assert_eq!(proc.pulse_count(), 0);
    }

    #[test]
    fn test_processor_multiple_ingests() {
        let mut proc = ElintProcessor::new(0.5, 1000.0);
        proc.ingest(&make_pulse_signal(500, &[100], 50, 1.0));
        proc.ingest(&make_pulse_signal(500, &[200], 50, 1.0));
        assert_eq!(proc.pulse_count(), 2);
    }

    #[test]
    fn test_analyze_pri_dwell_switch() {
        let mut toas = vec![0.0];
        let mut t = 0.0;
        for _ in 0..10 {
            t += 0.001;
            toas.push(t);
        }
        for _ in 0..10 {
            t += 0.002;
            toas.push(t);
        }
        match analyze_pri(&toas) {
            PriPattern::DwellSwitch(dwells) => {
                assert!(dwells.len() >= 2, "Expected >=2 dwell segments, got {}", dwells.len());
            }
            PriPattern::Stagger(_) => {} // acceptable
            other => panic!("Expected DwellSwitch or Stagger, got {:?}", other),
        }
    }

    #[test]
    fn test_negative_amplitude_pulse_detection() {
        let mut signal = vec![0.0; 1000];
        for i in 100..200 {
            signal[i] = -1.0;
        }
        let pulses = detect_pulses(&signal, 0.5, 1000.0);
        assert_eq!(pulses.len(), 1);
        assert!((pulses[0].amplitude - 1.0).abs() < 1e-6);
    }
}
