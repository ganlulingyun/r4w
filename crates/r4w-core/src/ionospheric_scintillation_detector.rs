//! Ionospheric Scintillation Detector
//!
//! Detects and characterizes amplitude and phase scintillations affecting
//! GNSS and HF signals propagating through the ionosphere. Ionospheric
//! irregularities cause rapid fluctuations in signal amplitude and phase,
//! degrading receiver performance and positioning accuracy.
//!
//! ## Provided Metrics
//!
//! - **S4 amplitude index**: normalized standard deviation of signal power
//! - **sigma-phi (σφ)**: standard deviation of detrended carrier phase (rad)
//! - **Fade statistics**: number of fades, duration, depth, recovery slope
//! - **Cycle slip detection**: identifies phase jumps near 2π multiples
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ionospheric_scintillation_detector::{
//!     ScintillationAnalyzer, ScintillationSeverity,
//! };
//!
//! let mut analyzer = ScintillationAnalyzer::new(1000.0, 256);
//!
//! // Constant-amplitude signal: no scintillation
//! let i_samples: Vec<f64> = (0..256).map(|n| (0.1 * n as f64).cos()).collect();
//! let q_samples: Vec<f64> = (0..256).map(|n| (0.1 * n as f64).sin()).collect();
//!
//! let metrics = analyzer.process_iq(&i_samples, &q_samples);
//! assert!(metrics.amplitude_index_s4 < 0.05);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Data types
// ---------------------------------------------------------------------------

/// Summary metrics describing ionospheric scintillation on a signal.
#[derive(Debug, Clone)]
pub struct ScintillationMetrics {
    /// S4 amplitude scintillation index (dimensionless, typically 0..1+).
    /// S4 = sqrt(var(P) / mean(P)^2).
    pub amplitude_index_s4: f64,

    /// Phase scintillation index sigma-phi (radians).
    /// Standard deviation of detrended carrier phase.
    pub phase_index_sigma_phi: f64,

    /// Estimated mean fade duration in milliseconds (0 if no fades).
    pub fade_duration_ms: f64,

    /// Mean recovery slope (dB per sample) after a fade event.
    pub recovery_slope: f64,

    /// Number of detected cycle slips in the phase history.
    pub num_cycle_slips: usize,
}

/// Statistics describing signal fade events.
#[derive(Debug, Clone)]
pub struct FadeStats {
    /// Number of fade events detected.
    pub num_fades: usize,

    /// Mean fade duration in samples.
    pub mean_duration_samples: f64,

    /// Maximum fade depth below the threshold (positive dB value).
    pub max_fade_depth_db: f64,

    /// Mean slope of signal recovery after a fade (dB/sample).
    pub mean_recovery_slope: f64,
}

/// Qualitative severity of amplitude scintillation based on S4.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScintillationSeverity {
    /// S4 effectively zero -- no scintillation.
    None,
    /// S4 < 0.3 -- minor fluctuations, usually negligible impact.
    Mild,
    /// 0.3 <= S4 < 0.6 -- noticeable fading, may degrade tracking.
    Moderate,
    /// 0.6 <= S4 < 1.0 -- deep fades possible, cycle slips likely.
    Strong,
    /// S4 >= 1.0 -- Rayleigh-like fading, loss of lock probable.
    Extreme,
}

impl ScintillationSeverity {
    /// Classify scintillation severity from an S4 index value.
    pub fn classify(s4: f64) -> Self {
        if s4 < 0.05 {
            ScintillationSeverity::None
        } else if s4 < 0.3 {
            ScintillationSeverity::Mild
        } else if s4 < 0.6 {
            ScintillationSeverity::Moderate
        } else if s4 < 1.0 {
            ScintillationSeverity::Strong
        } else {
            ScintillationSeverity::Extreme
        }
    }
}

// ---------------------------------------------------------------------------
// ScintillationAnalyzer -- stateful IQ processor
// ---------------------------------------------------------------------------

/// Main scintillation analysis processor.
///
/// Accepts raw I/Q sample vectors and produces [`ScintillationMetrics`].
/// Internally computes instantaneous amplitude and unwrapped phase from the
/// IQ data, then delegates to the individual estimation functions.
#[derive(Debug, Clone)]
pub struct ScintillationAnalyzer {
    sample_rate_hz: f64,
    window_size: usize,
}

impl ScintillationAnalyzer {
    /// Create a new analyzer.
    ///
    /// * `sample_rate_hz` -- sample rate of the incoming IQ stream (Hz).
    /// * `window_size` -- number of samples per analysis window.
    pub fn new(sample_rate_hz: f64, window_size: usize) -> Self {
        Self {
            sample_rate_hz,
            window_size,
        }
    }

    /// Process one window of IQ data and return scintillation metrics.
    ///
    /// `i` and `q` must have the same length; the analysis uses up to
    /// `window_size` samples (or fewer if the slices are shorter).
    pub fn process_iq(&mut self, i: &[f64], q: &[f64]) -> ScintillationMetrics {
        let n = i.len().min(q.len()).min(self.window_size);
        if n == 0 {
            return ScintillationMetrics {
                amplitude_index_s4: 0.0,
                phase_index_sigma_phi: 0.0,
                fade_duration_ms: 0.0,
                recovery_slope: 0.0,
                num_cycle_slips: 0,
            };
        }

        // Instantaneous power and phase
        let mut power = Vec::with_capacity(n);
        let mut phase_raw = Vec::with_capacity(n);

        for idx in 0..n {
            let ii = i[idx];
            let qq = q[idx];
            power.push(ii * ii + qq * qq);
            phase_raw.push(qq.atan2(ii));
        }

        // Unwrap phase to remove 2π discontinuities before analysis
        let phase = unwrap_phase(&phase_raw);

        // S4 amplitude index
        let s4 = estimate_amplitude_scintillation(&power);

        // Detrended phase scintillation
        let sigma_phi = estimate_phase_scintillation(&phase);

        // Cycle slips (threshold = 0.8 * 2π ≈ 5.03 rad on detrended phase)
        let cycle_slip_threshold = 0.8 * 2.0 * PI;
        let slips = detect_cycle_slips(&phase, cycle_slip_threshold);

        // Fade statistics (threshold = -3 dB below mean power)
        let power_db: Vec<f64> = power.iter().map(|&p| {
            if p > 0.0 { 10.0 * p.log10() } else { -120.0 }
        }).collect();
        let fade_threshold_db = -3.0; // 3 dB below mean
        let fade_stats = compute_fade_statistics(&power_db, fade_threshold_db);

        // Convert mean fade duration from samples to milliseconds
        let fade_duration_ms = if self.sample_rate_hz > 0.0 {
            fade_stats.mean_duration_samples / self.sample_rate_hz * 1000.0
        } else {
            0.0
        };

        ScintillationMetrics {
            amplitude_index_s4: s4,
            phase_index_sigma_phi: sigma_phi,
            fade_duration_ms,
            recovery_slope: fade_stats.mean_recovery_slope,
            num_cycle_slips: slips.len(),
        }
    }
}

// ---------------------------------------------------------------------------
// Public estimation functions
// ---------------------------------------------------------------------------

/// Compute the S4 amplitude scintillation index.
///
/// S4 = sqrt( var(P) / mean(P)^2 )
///
/// where P is instantaneous signal power. For a constant-envelope signal
/// S4 = 0; for Rayleigh fading S4 approaches 1.
///
/// Returns 0.0 for empty or single-sample input.
pub fn estimate_amplitude_scintillation(power_samples: &[f64]) -> f64 {
    let n = power_samples.len();
    if n < 2 {
        return 0.0;
    }

    let mean_p = power_samples.iter().copied().sum::<f64>() / n as f64;
    if mean_p.abs() < 1e-30 {
        return 0.0;
    }

    let var_p = power_samples
        .iter()
        .map(|&p| {
            let d = p - mean_p;
            d * d
        })
        .sum::<f64>()
        / n as f64;

    (var_p / (mean_p * mean_p)).sqrt()
}

/// Compute the phase scintillation index sigma-phi (radians).
///
/// The input phase is first detrended (linear trend removed) to eliminate
/// carrier frequency offset. Sigma-phi is the standard deviation of the
/// residual.
///
/// Returns 0.0 for empty or single-sample input.
pub fn estimate_phase_scintillation(phase_samples: &[f64]) -> f64 {
    let n = phase_samples.len();
    if n < 2 {
        return 0.0;
    }

    let detrended = detrend_linear(phase_samples);
    let mean = detrended.iter().copied().sum::<f64>() / n as f64;
    let var = detrended
        .iter()
        .map(|&x| {
            let d = x - mean;
            d * d
        })
        .sum::<f64>()
        / n as f64;

    var.sqrt()
}

/// Detect cycle slips in a phase history.
///
/// A cycle slip is declared at index `k` when:
///
///   |phase[k] - phase[k-1]| >= threshold_rad
///
/// and the jump is close to a multiple of 2π. The returned vector contains
/// the indices where slips were detected.
///
/// A typical threshold is 0.8 * 2π (≈ 5.03 rad).
pub fn detect_cycle_slips(phase_history: &[f64], threshold_rad: f64) -> Vec<usize> {
    let mut slips = Vec::new();
    if phase_history.len() < 2 {
        return slips;
    }

    let two_pi = 2.0 * PI;

    for k in 1..phase_history.len() {
        let diff = (phase_history[k] - phase_history[k - 1]).abs();
        if diff >= threshold_rad {
            // Check that the jump is near a 2π multiple
            let nearest_multiple = (diff / two_pi).round();
            if nearest_multiple >= 1.0 {
                let residual = (diff - nearest_multiple * two_pi).abs();
                // Allow ±30% tolerance around the 2π multiple
                if residual < 0.3 * two_pi {
                    slips.push(k);
                }
            }
        }
    }

    slips
}

/// Compute fade statistics from a signal power trace in dB.
///
/// A **fade** is a contiguous region where the signal power falls below
/// `threshold_db` relative to the mean power level.
///
/// Returns [`FadeStats`] with counts, durations, depths, and recovery slopes.
pub fn compute_fade_statistics(signal_power_db: &[f64], threshold_db: f64) -> FadeStats {
    let n = signal_power_db.len();
    if n == 0 {
        return FadeStats {
            num_fades: 0,
            mean_duration_samples: 0.0,
            max_fade_depth_db: 0.0,
            mean_recovery_slope: 0.0,
        };
    }

    // Mean power in dB
    let mean_db = signal_power_db.iter().copied().sum::<f64>() / n as f64;
    let abs_threshold = mean_db + threshold_db; // threshold_db is typically negative

    let mut num_fades: usize = 0;
    let mut total_duration: usize = 0;
    let mut max_depth: f64 = 0.0;
    let mut recovery_slopes: Vec<f64> = Vec::new();

    let mut in_fade = false;
    let mut fade_start: usize = 0;
    let mut fade_min_db: f64 = f64::MAX;

    for k in 0..n {
        let below = signal_power_db[k] < abs_threshold;
        if below && !in_fade {
            // Entering a fade
            in_fade = true;
            fade_start = k;
            fade_min_db = signal_power_db[k];
        } else if below && in_fade {
            if signal_power_db[k] < fade_min_db {
                fade_min_db = signal_power_db[k];
            }
        } else if !below && in_fade {
            // Exiting a fade
            in_fade = false;
            let duration = k - fade_start;
            num_fades += 1;
            total_duration += duration;

            let depth = abs_threshold - fade_min_db;
            if depth > max_depth {
                max_depth = depth;
            }

            // Recovery slope: dB rise per sample from minimum to crossing
            if duration > 1 {
                let rise = signal_power_db[k] - fade_min_db;
                // Find how many samples from min to exit
                let min_idx = (fade_start..k)
                    .min_by(|&a, &b| {
                        signal_power_db[a]
                            .partial_cmp(&signal_power_db[b])
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .unwrap_or(fade_start);
                let recovery_samples = k - min_idx;
                if recovery_samples > 0 {
                    recovery_slopes.push(rise / recovery_samples as f64);
                }
            }

            fade_min_db = f64::MAX;
        }
    }

    // Handle fade that runs to end of buffer
    if in_fade {
        let duration = n - fade_start;
        num_fades += 1;
        total_duration += duration;
        let depth = abs_threshold - fade_min_db;
        if depth > max_depth {
            max_depth = depth;
        }
    }

    let mean_duration = if num_fades > 0 {
        total_duration as f64 / num_fades as f64
    } else {
        0.0
    };

    let mean_recovery = if !recovery_slopes.is_empty() {
        recovery_slopes.iter().copied().sum::<f64>() / recovery_slopes.len() as f64
    } else {
        0.0
    };

    FadeStats {
        num_fades,
        mean_duration_samples: mean_duration,
        max_fade_depth_db: max_depth,
        mean_recovery_slope: mean_recovery,
    }
}

/// Remove a linear trend from a signal using least-squares fit.
///
/// Fits y = a + b*x and returns the residual y - (a + b*x).
/// For a single sample or empty input the output equals the input.
pub fn detrend_linear(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    if n <= 1 {
        return signal.to_vec();
    }

    // Least-squares linear regression: y = a + b*x
    // Using x = 0, 1, 2, ..., n-1
    let n_f = n as f64;

    // sum_x  = n*(n-1)/2
    // sum_x2 = n*(n-1)*(2n-1)/6
    let sum_x = n_f * (n_f - 1.0) / 2.0;
    let sum_x2 = n_f * (n_f - 1.0) * (2.0 * n_f - 1.0) / 6.0;

    let sum_y: f64 = signal.iter().copied().sum();
    let sum_xy: f64 = signal
        .iter()
        .enumerate()
        .map(|(i, &y)| i as f64 * y)
        .sum();

    let denom = n_f * sum_x2 - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        // Degenerate case -- all x identical (shouldn't happen for n >= 2)
        return signal.to_vec();
    }

    let b = (n_f * sum_xy - sum_x * sum_y) / denom;
    let a = (sum_y - b * sum_x) / n_f;

    signal
        .iter()
        .enumerate()
        .map(|(i, &y)| y - (a + b * i as f64))
        .collect()
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Unwrap a phase sequence so that consecutive samples differ by less than π.
fn unwrap_phase(phase: &[f64]) -> Vec<f64> {
    if phase.is_empty() {
        return Vec::new();
    }

    let mut out = Vec::with_capacity(phase.len());
    out.push(phase[0]);

    for k in 1..phase.len() {
        let mut diff = phase[k] - phase[k - 1];
        // Wrap diff into (-π, π]
        while diff > PI {
            diff -= 2.0 * PI;
        }
        while diff <= -PI {
            diff += 2.0 * PI;
        }
        out.push(out[k - 1] + diff);
    }

    out
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-9;
    const TWO_PI: f64 = 2.0 * PI;

    // ----- S4 amplitude index -----

    #[test]
    fn test_s4_constant_power_is_zero() {
        // Constant power -> var = 0 -> S4 = 0
        let power = vec![1.0; 1000];
        let s4 = estimate_amplitude_scintillation(&power);
        assert!(s4.abs() < EPSILON, "S4 should be 0 for constant power, got {s4}");
    }

    #[test]
    fn test_s4_known_am_modulation() {
        // Power = 1.0 + 0.5*sin(2π*f*t), so P varies between 0.5 and 1.5
        // mean(P) = 1.0, var(P) = 0.5^2 / 2 = 0.125
        // S4 = sqrt(0.125 / 1.0) = 0.3536
        let n = 10000;
        let power: Vec<f64> = (0..n)
            .map(|k| 1.0 + 0.5 * (TWO_PI * k as f64 / 100.0).sin())
            .collect();

        let s4 = estimate_amplitude_scintillation(&power);
        let expected = (0.125_f64).sqrt(); // 0.35355...
        assert!(
            (s4 - expected).abs() < 0.01,
            "S4 expected ~{expected:.4}, got {s4:.4}"
        );
    }

    #[test]
    fn test_s4_binary_switching() {
        // Power alternates between 0.5 and 1.5
        // mean = 1.0, var = 0.25, S4 = 0.5
        let power: Vec<f64> = (0..2000).map(|k| if k % 2 == 0 { 0.5 } else { 1.5 }).collect();
        let s4 = estimate_amplitude_scintillation(&power);
        assert!(
            (s4 - 0.5).abs() < 0.01,
            "S4 expected ~0.5, got {s4:.4}"
        );
    }

    #[test]
    fn test_s4_empty_returns_zero() {
        assert_eq!(estimate_amplitude_scintillation(&[]), 0.0);
    }

    #[test]
    fn test_s4_single_sample_returns_zero() {
        assert_eq!(estimate_amplitude_scintillation(&[42.0]), 0.0);
    }

    #[test]
    fn test_s4_all_zeros_returns_zero() {
        let power = vec![0.0; 100];
        let s4 = estimate_amplitude_scintillation(&power);
        assert_eq!(s4, 0.0);
    }

    // ----- Phase scintillation -----

    #[test]
    fn test_sigma_phi_pure_linear_phase_is_zero() {
        // A linear phase is entirely removed by detrending => sigma = 0
        let phase: Vec<f64> = (0..500).map(|k| 0.01 * k as f64).collect();
        let sigma = estimate_phase_scintillation(&phase);
        assert!(sigma < 1e-10, "sigma_phi should be ~0 for linear phase, got {sigma}");
    }

    #[test]
    fn test_sigma_phi_known_sinusoidal_modulation() {
        // phase = 0.1 * sin(2π*f*t)  (after detrend the linear part is zero)
        // var = amplitude^2 / 2 = 0.005, sigma = sqrt(0.005) ≈ 0.07071
        let n = 10000;
        let amplitude = 0.1;
        let phase: Vec<f64> = (0..n)
            .map(|k| amplitude * (TWO_PI * k as f64 / 200.0).sin())
            .collect();
        let sigma = estimate_phase_scintillation(&phase);
        let expected = (amplitude * amplitude / 2.0).sqrt();
        assert!(
            (sigma - expected).abs() < 0.005,
            "sigma_phi expected ~{expected:.4}, got {sigma:.4}"
        );
    }

    #[test]
    fn test_sigma_phi_empty() {
        assert_eq!(estimate_phase_scintillation(&[]), 0.0);
    }

    #[test]
    fn test_sigma_phi_single_sample() {
        assert_eq!(estimate_phase_scintillation(&[1.23]), 0.0);
    }

    // ----- Cycle slip detection -----

    #[test]
    fn test_cycle_slip_no_slips_smooth_phase() {
        let phase: Vec<f64> = (0..200).map(|k| 0.05 * k as f64).collect();
        let slips = detect_cycle_slips(&phase, 0.8 * TWO_PI);
        assert!(slips.is_empty(), "No slips expected in smooth phase");
    }

    #[test]
    fn test_cycle_slip_single_2pi_jump() {
        // Insert a single +2π jump at index 100
        let mut phase: Vec<f64> = (0..200).map(|k| 0.01 * k as f64).collect();
        phase[100] += TWO_PI;
        for k in 101..200 {
            phase[k] += TWO_PI;
        }

        let slips = detect_cycle_slips(&phase, 0.8 * TWO_PI);
        assert_eq!(slips.len(), 1, "Expected 1 cycle slip, got {}", slips.len());
        assert_eq!(slips[0], 100);
    }

    #[test]
    fn test_cycle_slip_multiple_jumps() {
        let mut phase: Vec<f64> = (0..400).map(|k| 0.01 * k as f64).collect();
        // Insert +2π at 100 and -2π at 250
        let offset1 = TWO_PI;
        for k in 100..400 {
            phase[k] += offset1;
        }
        let offset2 = -TWO_PI;
        for k in 250..400 {
            phase[k] += offset2;
        }

        let slips = detect_cycle_slips(&phase, 0.8 * TWO_PI);
        assert_eq!(slips.len(), 2, "Expected 2 cycle slips, got {:?}", slips);
        assert_eq!(slips[0], 100);
        assert_eq!(slips[1], 250);
    }

    #[test]
    fn test_cycle_slip_4pi_jump_detected() {
        // A 4π jump (2 cycle slips worth) should still be detected as near a 2π multiple
        let mut phase: Vec<f64> = (0..200).map(|k| 0.01 * k as f64).collect();
        for k in 100..200 {
            phase[k] += 4.0 * PI; // = 2 * 2π
        }

        let slips = detect_cycle_slips(&phase, 0.8 * TWO_PI);
        assert!(!slips.is_empty(), "4π jump should be detected");
        assert_eq!(slips[0], 100);
    }

    #[test]
    fn test_cycle_slip_empty_input() {
        assert!(detect_cycle_slips(&[], 1.0).is_empty());
    }

    #[test]
    fn test_cycle_slip_single_sample() {
        assert!(detect_cycle_slips(&[0.0], 1.0).is_empty());
    }

    // ----- Fade statistics -----

    #[test]
    fn test_fade_stats_no_fades_flat_signal() {
        let power_db = vec![0.0; 100];
        let stats = compute_fade_statistics(&power_db, -3.0);
        assert_eq!(stats.num_fades, 0);
        assert_eq!(stats.mean_duration_samples, 0.0);
    }

    #[test]
    fn test_fade_stats_synthetic_single_fade() {
        // Signal at 0 dB with a dip to -10 dB from sample 40..60
        let mut power_db = vec![0.0; 100];
        for k in 40..60 {
            power_db[k] = -10.0;
        }

        let stats = compute_fade_statistics(&power_db, -3.0);
        assert_eq!(stats.num_fades, 1, "Expected 1 fade, got {}", stats.num_fades);
        assert_eq!(stats.mean_duration_samples, 20.0);
        // Depth should be at least 7 dB (mean ~= -2.0, threshold = mean - 3 = -5.0, min = -10, depth = 5)
        assert!(stats.max_fade_depth_db > 0.0, "Fade depth should be positive");
    }

    #[test]
    fn test_fade_stats_two_fades() {
        let mut power_db = vec![0.0; 200];
        // Fade 1: samples 20..30
        for k in 20..30 {
            power_db[k] = -15.0;
        }
        // Fade 2: samples 100..120
        for k in 100..120 {
            power_db[k] = -15.0;
        }

        let stats = compute_fade_statistics(&power_db, -3.0);
        assert_eq!(stats.num_fades, 2, "Expected 2 fades, got {}", stats.num_fades);
    }

    #[test]
    fn test_fade_stats_empty() {
        let stats = compute_fade_statistics(&[], -3.0);
        assert_eq!(stats.num_fades, 0);
    }

    #[test]
    fn test_fade_stats_fade_at_end_of_buffer() {
        // Fade that doesn't recover before buffer ends
        let mut power_db = vec![0.0; 100];
        for k in 80..100 {
            power_db[k] = -20.0;
        }

        let stats = compute_fade_statistics(&power_db, -3.0);
        assert_eq!(stats.num_fades, 1, "Fade at end should still be counted");
    }

    #[test]
    fn test_fade_stats_recovery_slope_positive() {
        let mut power_db = vec![0.0; 100];
        // Triangular fade: ramp down then ramp up
        for k in 40..50 {
            power_db[k] = -2.0 * (k - 40) as f64; // going from 0 to -18
        }
        for k in 50..60 {
            power_db[k] = -18.0 + 2.0 * (k - 50) as f64; // recovering
        }

        let stats = compute_fade_statistics(&power_db, -3.0);
        if stats.num_fades > 0 {
            assert!(
                stats.mean_recovery_slope >= 0.0,
                "Recovery slope should be non-negative"
            );
        }
    }

    // ----- Severity classification -----

    #[test]
    fn test_severity_none() {
        assert_eq!(ScintillationSeverity::classify(0.0), ScintillationSeverity::None);
        assert_eq!(ScintillationSeverity::classify(0.04), ScintillationSeverity::None);
    }

    #[test]
    fn test_severity_mild() {
        assert_eq!(ScintillationSeverity::classify(0.05), ScintillationSeverity::Mild);
        assert_eq!(ScintillationSeverity::classify(0.15), ScintillationSeverity::Mild);
        assert_eq!(ScintillationSeverity::classify(0.29), ScintillationSeverity::Mild);
    }

    #[test]
    fn test_severity_moderate() {
        assert_eq!(ScintillationSeverity::classify(0.3), ScintillationSeverity::Moderate);
        assert_eq!(ScintillationSeverity::classify(0.45), ScintillationSeverity::Moderate);
        assert_eq!(ScintillationSeverity::classify(0.59), ScintillationSeverity::Moderate);
    }

    #[test]
    fn test_severity_strong() {
        assert_eq!(ScintillationSeverity::classify(0.6), ScintillationSeverity::Strong);
        assert_eq!(ScintillationSeverity::classify(0.8), ScintillationSeverity::Strong);
        assert_eq!(ScintillationSeverity::classify(0.99), ScintillationSeverity::Strong);
    }

    #[test]
    fn test_severity_extreme() {
        assert_eq!(ScintillationSeverity::classify(1.0), ScintillationSeverity::Extreme);
        assert_eq!(ScintillationSeverity::classify(2.5), ScintillationSeverity::Extreme);
    }

    // ----- Detrend -----

    #[test]
    fn test_detrend_removes_linear() {
        // y = 3.0 + 2.0 * x  =>  residual should be ~0
        let signal: Vec<f64> = (0..100).map(|k| 3.0 + 2.0 * k as f64).collect();
        let residual = detrend_linear(&signal);
        for (i, &r) in residual.iter().enumerate() {
            assert!(
                r.abs() < 1e-8,
                "Residual at index {i} = {r}, expected ~0"
            );
        }
    }

    #[test]
    fn test_detrend_preserves_zero_mean_signal() {
        // A signal with zero mean and no linear trend should be mostly unchanged
        let n = 1000;
        let signal: Vec<f64> = (0..n)
            .map(|k| (TWO_PI * k as f64 / 50.0).sin())
            .collect();
        let detrended = detrend_linear(&signal);

        // Energy should be preserved (no trend to remove)
        let orig_energy: f64 = signal.iter().map(|x| x * x).sum();
        let det_energy: f64 = detrended.iter().map(|x| x * x).sum();
        let ratio = det_energy / orig_energy;
        assert!(
            (ratio - 1.0).abs() < 0.01,
            "Energy ratio {ratio} should be ~1.0"
        );
    }

    #[test]
    fn test_detrend_linear_plus_sine() {
        // y = 5.0 + 0.3*x + sin(2π*x/50)
        // After detrend, only the sine should remain
        let n = 1000;
        let signal: Vec<f64> = (0..n)
            .map(|k| 5.0 + 0.3 * k as f64 + (TWO_PI * k as f64 / 50.0).sin())
            .collect();
        let detrended = detrend_linear(&signal);

        // The detrended signal should have mean ~0
        let mean: f64 = detrended.iter().copied().sum::<f64>() / n as f64;
        assert!(mean.abs() < 0.01, "Detrended mean should be ~0, got {mean}");

        // And should look like a sine: check variance ~ 0.5
        let var: f64 = detrended.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n as f64;
        assert!(
            (var - 0.5).abs() < 0.05,
            "Detrended variance should be ~0.5, got {var}"
        );
    }

    #[test]
    fn test_detrend_empty() {
        assert!(detrend_linear(&[]).is_empty());
    }

    #[test]
    fn test_detrend_single() {
        let result = detrend_linear(&[42.0]);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0], 42.0);
    }

    // ----- Phase unwrap helper -----

    #[test]
    fn test_unwrap_phase_monotonic() {
        // A linearly increasing phase that wraps through ±π multiple times
        let n = 1000;
        let raw: Vec<f64> = (0..n)
            .map(|k| {
                let p = 0.1 * k as f64;
                // Wrap to (-π, π]
                ((p + PI) % TWO_PI) - PI
            })
            .collect();

        let unwrapped = unwrap_phase(&raw);

        // Unwrapped should be monotonically increasing (approximately)
        for k in 1..unwrapped.len() {
            let diff = unwrapped[k] - unwrapped[k - 1];
            assert!(
                diff > -0.5 && diff < 0.5,
                "Unwrapped diff at {k} = {diff}, expected ~0.1"
            );
        }
    }

    #[test]
    fn test_unwrap_phase_empty() {
        assert!(unwrap_phase(&[]).is_empty());
    }

    // ----- ScintillationAnalyzer end-to-end -----

    #[test]
    fn test_analyzer_constant_iq_no_scintillation() {
        let mut analyzer = ScintillationAnalyzer::new(1000.0, 512);
        let n = 512;
        let i_samples: Vec<f64> = (0..n).map(|k| (0.1 * k as f64).cos()).collect();
        let q_samples: Vec<f64> = (0..n).map(|k| (0.1 * k as f64).sin()).collect();

        let metrics = analyzer.process_iq(&i_samples, &q_samples);

        // Constant amplitude -> S4 near zero
        assert!(
            metrics.amplitude_index_s4 < 0.05,
            "S4 = {}, expected < 0.05",
            metrics.amplitude_index_s4
        );
    }

    #[test]
    fn test_analyzer_am_signal_gives_nonzero_s4() {
        let mut analyzer = ScintillationAnalyzer::new(1000.0, 2000);
        let n = 2000;
        // Amplitude-modulated IQ: amplitude = 1 + 0.5*sin(low_freq)
        let i_samples: Vec<f64> = (0..n)
            .map(|k| {
                let carrier = 0.3 * k as f64;
                let envelope = 1.0 + 0.5 * (TWO_PI * k as f64 / 200.0).sin();
                envelope * carrier.cos()
            })
            .collect();
        let q_samples: Vec<f64> = (0..n)
            .map(|k| {
                let carrier = 0.3 * k as f64;
                let envelope = 1.0 + 0.5 * (TWO_PI * k as f64 / 200.0).sin();
                envelope * carrier.sin()
            })
            .collect();

        let metrics = analyzer.process_iq(&i_samples, &q_samples);
        assert!(
            metrics.amplitude_index_s4 > 0.1,
            "AM signal should have noticeable S4, got {}",
            metrics.amplitude_index_s4
        );
    }

    #[test]
    fn test_analyzer_empty_input() {
        let mut analyzer = ScintillationAnalyzer::new(1000.0, 256);
        let metrics = analyzer.process_iq(&[], &[]);

        assert_eq!(metrics.amplitude_index_s4, 0.0);
        assert_eq!(metrics.phase_index_sigma_phi, 0.0);
        assert_eq!(metrics.fade_duration_ms, 0.0);
        assert_eq!(metrics.num_cycle_slips, 0);
    }

    #[test]
    fn test_analyzer_mismatched_lengths_uses_shorter() {
        let mut analyzer = ScintillationAnalyzer::new(1000.0, 1000);
        let i_samples = vec![1.0; 100];
        let q_samples = vec![0.0; 200]; // longer

        // Should not panic, uses min(100, 200, 1000) = 100
        let metrics = analyzer.process_iq(&i_samples, &q_samples);
        assert!(metrics.amplitude_index_s4.is_finite());
    }

    #[test]
    fn test_analyzer_with_phase_modulation() {
        let mut analyzer = ScintillationAnalyzer::new(1000.0, 2000);
        let n = 2000;
        // IQ with phase modulation: phase = carrier + 0.5*sin(low_freq)
        let i_samples: Vec<f64> = (0..n)
            .map(|k| {
                let phase = 0.2 * k as f64 + 0.5 * (TWO_PI * k as f64 / 100.0).sin();
                phase.cos()
            })
            .collect();
        let q_samples: Vec<f64> = (0..n)
            .map(|k| {
                let phase = 0.2 * k as f64 + 0.5 * (TWO_PI * k as f64 / 100.0).sin();
                phase.sin()
            })
            .collect();

        let metrics = analyzer.process_iq(&i_samples, &q_samples);
        // Phase modulation should produce noticeable sigma_phi
        assert!(
            metrics.phase_index_sigma_phi > 0.1,
            "PM signal should have sigma_phi > 0.1, got {}",
            metrics.phase_index_sigma_phi
        );
        // Amplitude should be constant (S4 ~0)
        assert!(
            metrics.amplitude_index_s4 < 0.05,
            "PM-only signal should have S4 ~0, got {}",
            metrics.amplitude_index_s4
        );
    }

    #[test]
    fn test_analyzer_cycle_slips_in_iq() {
        let mut analyzer = ScintillationAnalyzer::new(1000.0, 500);
        let n = 500;
        // Build IQ with a forced 2π phase discontinuity at sample 250
        // Phase = 0.01*k for k < 250, then 0.01*k + 2π for k >= 250
        // After atan2 and unwrap, the jump should be detected.
        // But atan2 wraps to (-π,π), and unwrap will smooth it out.
        // To create a detectable cycle slip, we need the raw atan2 to
        // jump. Let's inject a discontinuity that doesn't unwrap cleanly.
        //
        // Strategy: use a very slow carrier so atan2 barely changes,
        // then insert a 2π jump in the underlying phase.
        let i_samples: Vec<f64> = (0..n)
            .map(|k| {
                let phase = if k < 250 {
                    0.001 * k as f64
                } else {
                    // Jump by 2π at k=250
                    0.001 * k as f64 + TWO_PI
                };
                phase.cos()
            })
            .collect();
        let q_samples: Vec<f64> = (0..n)
            .map(|k| {
                let phase = if k < 250 {
                    0.001 * k as f64
                } else {
                    0.001 * k as f64 + TWO_PI
                };
                phase.sin()
            })
            .collect();

        let metrics = analyzer.process_iq(&i_samples, &q_samples);
        // cos(x + 2π) == cos(x), sin(x + 2π) == sin(x)
        // So the IQ is actually continuous -- 2π jump is invisible in IQ domain.
        // This is expected: real cycle slips in GNSS manifest as tracking loop failures.
        // The metric should still be computed (0 slips in this case).
        assert_eq!(metrics.num_cycle_slips, 0,
            "2π phase jump is invisible in IQ domain, so 0 slips expected");
    }

    #[test]
    fn test_cycle_slip_detection_on_raw_phase_with_slip() {
        // Test the detect_cycle_slips function directly with a constructed phase
        // that has a genuine discontinuity (not IQ-derived)
        let mut phase: Vec<f64> = (0..200).map(|k| 0.01 * k as f64).collect();
        // Insert a jump of exactly 2π at index 100
        for k in 100..200 {
            phase[k] += TWO_PI;
        }

        let slips = detect_cycle_slips(&phase, 5.0);
        assert_eq!(slips.len(), 1);
        assert_eq!(slips[0], 100);
    }

    #[test]
    fn test_s4_rayleigh_like_signal() {
        // Simulate a signal with S4 near 1.0 (exponential power distribution)
        // For Rayleigh amplitude, power is exponential, and S4 = 1.
        // Use a simple model: power = -ln(U) where U ~ uniform(0,1)
        // This gives exponential(1) distribution with mean=1, var=1, so S4 = 1.
        //
        // Use a deterministic pseudo-random sequence for reproducibility.
        let n = 5000;
        let mut power = Vec::with_capacity(n);
        let mut seed: u64 = 12345;
        for _ in 0..n {
            // Simple LCG for reproducible pseudo-random
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let u = (seed >> 33) as f64 / (1u64 << 31) as f64; // uniform in (0, 1)
            let u_clamped = u.max(1e-10); // avoid log(0)
            power.push(-u_clamped.ln()); // exponential(1)
        }

        let s4 = estimate_amplitude_scintillation(&power);
        assert!(
            (s4 - 1.0).abs() < 0.15,
            "Rayleigh-like signal should have S4 near 1.0, got {s4:.4}"
        );
    }

    #[test]
    fn test_severity_from_s4_round_trip() {
        // Verify the classify function covers the full range
        let test_cases = [
            (0.0, ScintillationSeverity::None),
            (0.02, ScintillationSeverity::None),
            (0.1, ScintillationSeverity::Mild),
            (0.3, ScintillationSeverity::Moderate),
            (0.6, ScintillationSeverity::Strong),
            (1.0, ScintillationSeverity::Extreme),
            (1.5, ScintillationSeverity::Extreme),
        ];

        for (s4, expected) in &test_cases {
            let severity = ScintillationSeverity::classify(*s4);
            assert_eq!(
                severity, *expected,
                "S4={s4}: expected {expected:?}, got {severity:?}"
            );
        }
    }

    #[test]
    fn test_detrend_two_samples() {
        // y = [1.0, 3.0] -> line through them -> residual = [0, 0]
        let result = detrend_linear(&[1.0, 3.0]);
        assert_eq!(result.len(), 2);
        assert!(result[0].abs() < 1e-10, "residual[0] = {}", result[0]);
        assert!(result[1].abs() < 1e-10, "residual[1] = {}", result[1]);
    }

    #[test]
    fn test_fade_stats_all_below_threshold() {
        // Entire signal is in a fade (all samples below threshold)
        let power_db = vec![-20.0; 100];
        let stats = compute_fade_statistics(&power_db, -3.0);
        // Mean is -20, threshold is -23. All samples are at -20 which is above -23.
        // So actually no fades.
        assert_eq!(stats.num_fades, 0);

        // Now try with mean = -20, threshold = 0 relative, so abs_threshold = -20
        // All samples are at exactly -20, not below -20.
        let stats2 = compute_fade_statistics(&power_db, 0.0);
        assert_eq!(stats2.num_fades, 0);
    }
}
