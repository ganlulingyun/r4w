//! PAPR Reduction — Peak-to-Average Power Ratio Optimization
//!
//! Algorithms to reduce PAPR in OFDM and multi-carrier waveforms,
//! enabling efficient power amplifier operation. Implements clipping
//! and filtering, Selected Mapping (SLM), Tone Reservation (TR),
//! and Active Constellation Extension (ACE).
//! GNU Radio equivalent: No direct equivalent in core GNU Radio.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::papr_reduction::{measure_papr, clip_and_filter};
//! use num_complex::Complex64;
//!
//! let signal: Vec<Complex64> = (0..64).map(|i| {
//!     Complex64::new((i as f64 * 0.1).cos(), (i as f64 * 0.1).sin())
//! }).collect();
//! let papr = measure_papr(&signal);
//! assert!(papr > 0.0);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// PAPR reduction method.
#[derive(Debug, Clone)]
pub enum PaprMethod {
    /// Clipping with optional filtering.
    ClippingAndFiltering {
        /// Clipping threshold in dB above average.
        threshold_db: f64,
        /// Number of filter iterations.
        filter_iterations: usize,
    },
    /// Selected Mapping: try multiple phase rotations.
    SelectedMapping {
        /// Number of candidate sequences to try.
        num_candidates: usize,
    },
    /// Tone Reservation: use reserved carriers for PAPR reduction.
    ToneReservation {
        /// Indices of reserved subcarriers.
        reserved_carriers: Vec<usize>,
        /// Maximum number of iterations.
        max_iterations: usize,
        /// Target PAPR in dB.
        target_papr_db: f64,
    },
    /// Active Constellation Extension.
    ActiveConstellationExtension {
        /// Maximum extension factor.
        max_extension: f64,
    },
}

/// PAPR reduction result.
#[derive(Debug, Clone)]
pub struct PaprResult {
    /// Reduced signal.
    pub signal: Vec<Complex64>,
    /// Original PAPR in dB.
    pub original_papr_db: f64,
    /// Reduced PAPR in dB.
    pub reduced_papr_db: f64,
    /// EVM (error vector magnitude) introduced.
    pub evm_percent: f64,
}

/// PAPR reducer.
#[derive(Debug, Clone)]
pub struct PaprReducer {
    method: PaprMethod,
    ofdm_size: usize,
}

impl PaprReducer {
    /// Create a new PAPR reducer.
    pub fn new(method: PaprMethod, ofdm_size: usize) -> Self {
        Self { method, ofdm_size }
    }

    /// Apply PAPR reduction to OFDM signal.
    pub fn reduce(&self, signal: &[Complex64]) -> PaprResult {
        let original_papr_db = measure_papr(signal);

        let reduced = match &self.method {
            PaprMethod::ClippingAndFiltering {
                threshold_db,
                filter_iterations,
            } => clip_and_filter(signal, *threshold_db, *filter_iterations),
            PaprMethod::SelectedMapping { num_candidates } => {
                selected_mapping(signal, *num_candidates, self.ofdm_size)
            }
            PaprMethod::ToneReservation {
                reserved_carriers,
                max_iterations,
                target_papr_db,
            } => tone_reservation(
                signal,
                reserved_carriers,
                *max_iterations,
                *target_papr_db,
                self.ofdm_size,
            ),
            PaprMethod::ActiveConstellationExtension { max_extension } => {
                active_constellation_extension(signal, *max_extension)
            }
        };

        let reduced_papr_db = measure_papr(&reduced);
        let evm = compute_evm(signal, &reduced);

        PaprResult {
            signal: reduced,
            original_papr_db,
            reduced_papr_db,
            evm_percent: evm,
        }
    }

    /// Get the reduction method.
    pub fn method(&self) -> &PaprMethod {
        &self.method
    }
}

/// Measure PAPR of a signal in dB.
pub fn measure_papr(signal: &[Complex64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }

    let peak = signal
        .iter()
        .map(|s| s.norm_sqr())
        .fold(0.0f64, f64::max);
    let avg = signal.iter().map(|s| s.norm_sqr()).sum::<f64>() / signal.len() as f64;

    if avg > 0.0 {
        10.0 * (peak / avg).log10()
    } else {
        0.0
    }
}

/// Compute CCDF (Complementary Cumulative Distribution Function) of PAPR.
///
/// Returns (PAPR_threshold_dB, probability) pairs.
pub fn compute_ccdf(signal: &[Complex64], num_symbols: usize, ofdm_size: usize) -> Vec<(f64, f64)> {
    if signal.is_empty() || ofdm_size == 0 {
        return Vec::new();
    }

    // Measure PAPR of each OFDM symbol
    let mut paprs: Vec<f64> = Vec::new();
    for chunk in signal.chunks(ofdm_size) {
        if chunk.len() == ofdm_size {
            paprs.push(measure_papr(chunk));
        }
    }
    if paprs.is_empty() {
        // Use entire signal as one symbol
        paprs.push(measure_papr(signal));
    }

    paprs.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let n = paprs.len();
    let mut ccdf = Vec::new();
    let num_points = 20;
    let min_papr = paprs[0];
    let max_papr = paprs[n - 1];

    if (max_papr - min_papr).abs() < 1e-6 {
        ccdf.push((min_papr, 1.0));
        return ccdf;
    }

    for i in 0..num_points {
        let threshold = min_papr + (max_papr - min_papr) * i as f64 / (num_points - 1) as f64;
        let count = paprs.iter().filter(|&&p| p > threshold).count();
        ccdf.push((threshold, count as f64 / n as f64));
    }

    ccdf
}

/// Clipping and filtering PAPR reduction.
pub fn clip_and_filter(signal: &[Complex64], threshold_db: f64, iterations: usize) -> Vec<Complex64> {
    let avg_power = signal.iter().map(|s| s.norm_sqr()).sum::<f64>() / signal.len() as f64;
    let clip_level = (avg_power * 10.0f64.powf(threshold_db / 10.0)).sqrt();

    let mut result = signal.to_vec();

    for _ in 0..iterations.max(1) {
        // Clip
        for s in result.iter_mut() {
            let mag = s.norm();
            if mag > clip_level {
                *s = *s * (clip_level / mag);
            }
        }

        // Simple low-pass filtering (moving average as approximation)
        if result.len() > 3 {
            let mut filtered = result.clone();
            for i in 1..result.len() - 1 {
                filtered[i] = (result[i - 1] + result[i] * 2.0 + result[i + 1]) / 4.0;
            }
            result = filtered;
        }
    }

    result
}

/// Selected Mapping (SLM) PAPR reduction.
pub fn selected_mapping(
    signal: &[Complex64],
    num_candidates: usize,
    _ofdm_size: usize,
) -> Vec<Complex64> {
    let mut best = signal.to_vec();
    let mut best_papr = measure_papr(signal);

    let mut rng_state = 42u64;

    for _ in 0..num_candidates {
        // Generate random phase sequence
        let rotated: Vec<Complex64> = signal
            .iter()
            .enumerate()
            .map(|(i, &s)| {
                rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
                let phase_idx = ((rng_state >> 32) as usize) % 4;
                let phase = phase_idx as f64 * PI / 2.0;
                s * Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        let papr = measure_papr(&rotated);
        if papr < best_papr {
            best_papr = papr;
            best = rotated;
        }
    }

    best
}

/// Tone Reservation PAPR reduction.
pub fn tone_reservation(
    signal: &[Complex64],
    reserved: &[usize],
    max_iterations: usize,
    target_papr_db: f64,
    ofdm_size: usize,
) -> Vec<Complex64> {
    let mut result = signal.to_vec();

    if reserved.is_empty() || result.is_empty() {
        return result;
    }

    let avg_power = result.iter().map(|s| s.norm_sqr()).sum::<f64>() / result.len() as f64;

    for _ in 0..max_iterations {
        let current_papr = measure_papr(&result);
        if current_papr <= target_papr_db {
            break;
        }

        // Find peak sample
        let (peak_idx, peak_mag) = result
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.norm_sqr().partial_cmp(&b.1.norm_sqr()).unwrap())
            .map(|(i, s)| (i, s.norm()))
            .unwrap_or((0, 0.0));

        let target_mag = (avg_power * 10.0f64.powf(target_papr_db / 10.0)).sqrt();
        if peak_mag <= target_mag {
            break;
        }

        // Compute cancellation signal using reserved tones
        let reduction = (peak_mag - target_mag) / reserved.len() as f64;
        for &carrier in reserved {
            if carrier < result.len() {
                let phase = -2.0 * PI * carrier as f64 * peak_idx as f64 / ofdm_size as f64;
                result[carrier] -= Complex64::new(phase.cos(), phase.sin()) * reduction;
            }
        }
    }

    result
}

/// Active Constellation Extension PAPR reduction.
pub fn active_constellation_extension(
    signal: &[Complex64],
    max_extension: f64,
) -> Vec<Complex64> {
    let avg_power = signal.iter().map(|s| s.norm_sqr()).sum::<f64>() / signal.len() as f64;
    let clip_level = avg_power.sqrt() * (1.0 + max_extension);

    signal
        .iter()
        .map(|&s| {
            let mag = s.norm();
            if mag > clip_level {
                // Extend away from origin (push constellation points outward)
                s * (clip_level / mag)
            } else {
                s
            }
        })
        .collect()
}

/// Compute EVM (Error Vector Magnitude) between original and modified signal.
pub fn compute_evm(original: &[Complex64], modified: &[Complex64]) -> f64 {
    let n = original.len().min(modified.len());
    if n == 0 {
        return 0.0;
    }

    let ref_power: f64 = original[..n].iter().map(|s| s.norm_sqr()).sum::<f64>() / n as f64;
    let error_power: f64 = original[..n]
        .iter()
        .zip(modified[..n].iter())
        .map(|(a, b)| (a - b).norm_sqr())
        .sum::<f64>()
        / n as f64;

    if ref_power > 0.0 {
        (error_power / ref_power).sqrt() * 100.0
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_ofdm_signal(size: usize) -> Vec<Complex64> {
        // Sum of random-phase subcarriers creates high PAPR
        let mut signal = vec![Complex64::new(0.0, 0.0); size];
        let num_carriers = size / 4;
        for k in 0..num_carriers {
            let phase = 2.0 * PI * k as f64 * 0.37; // arbitrary phases
            for (n, s) in signal.iter_mut().enumerate() {
                let freq = 2.0 * PI * k as f64 * n as f64 / size as f64 + phase;
                *s += Complex64::new(freq.cos(), freq.sin());
            }
        }
        signal
    }

    #[test]
    fn test_measure_papr_constant() {
        let signal: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 64];
        let papr = measure_papr(&signal);
        assert!((papr - 0.0).abs() < 0.01, "Constant signal should have 0 dB PAPR");
    }

    #[test]
    fn test_measure_papr_ofdm() {
        let signal = make_ofdm_signal(64);
        let papr = measure_papr(&signal);
        assert!(papr > 2.0, "OFDM signal should have high PAPR: {}", papr);
    }

    #[test]
    fn test_clip_and_filter() {
        let signal = make_ofdm_signal(64);
        let original_papr = measure_papr(&signal);
        let reduced = clip_and_filter(&signal, 3.0, 2);
        let reduced_papr = measure_papr(&reduced);
        assert!(
            reduced_papr <= original_papr + 0.5,
            "Clipping should not increase PAPR significantly"
        );
    }

    #[test]
    fn test_slm_reduction() {
        let signal = make_ofdm_signal(64);
        let original_papr = measure_papr(&signal);
        let reduced = selected_mapping(&signal, 8, 64);
        let reduced_papr = measure_papr(&reduced);
        // SLM should find a candidate with lower or equal PAPR
        assert!(
            reduced_papr <= original_papr + 0.1,
            "SLM should not increase PAPR: orig={:.1}, reduced={:.1}",
            original_papr,
            reduced_papr
        );
    }

    #[test]
    fn test_papr_reducer() {
        let method = PaprMethod::ClippingAndFiltering {
            threshold_db: 4.0,
            filter_iterations: 1,
        };
        let reducer = PaprReducer::new(method, 64);
        let signal = make_ofdm_signal(64);
        let result = reducer.reduce(&signal);
        assert!(result.original_papr_db > 0.0);
        assert!(result.evm_percent >= 0.0);
        assert_eq!(result.signal.len(), signal.len());
    }

    #[test]
    fn test_tone_reservation() {
        let signal = make_ofdm_signal(64);
        let reserved = vec![0, 1, 62, 63]; // edge carriers
        let reduced = tone_reservation(&signal, &reserved, 10, 6.0, 64);
        assert_eq!(reduced.len(), signal.len());
    }

    #[test]
    fn test_ace() {
        let signal = make_ofdm_signal(64);
        let reduced = active_constellation_extension(&signal, 0.5);
        assert_eq!(reduced.len(), signal.len());
    }

    #[test]
    fn test_compute_evm_identical() {
        let signal: Vec<Complex64> = (0..32)
            .map(|i| Complex64::new(i as f64 * 0.1, 0.0))
            .collect();
        let evm = compute_evm(&signal, &signal);
        assert!(evm < 0.001, "EVM of identical signals should be ~0");
    }

    #[test]
    fn test_ccdf() {
        let signal = make_ofdm_signal(256);
        let ccdf = compute_ccdf(&signal, 4, 64);
        assert!(!ccdf.is_empty());
        // First point should have probability ≤ 1.0
        assert!(ccdf[0].1 <= 1.0 + 1e-6);
    }

    #[test]
    fn test_measure_papr_empty() {
        let signal: Vec<Complex64> = vec![];
        let papr = measure_papr(&signal);
        assert_eq!(papr, 0.0);
    }

    #[test]
    fn test_evm_with_noise() {
        let signal: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 100];
        let noisy: Vec<Complex64> = signal
            .iter()
            .enumerate()
            .map(|(i, s)| s + Complex64::new(0.01 * (i as f64).sin(), 0.0))
            .collect();
        let evm = compute_evm(&signal, &noisy);
        assert!(evm > 0.0 && evm < 5.0, "EVM should be small: {}%", evm);
    }
}
