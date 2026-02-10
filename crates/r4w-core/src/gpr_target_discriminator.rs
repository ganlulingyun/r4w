//! GPR target classification for mine/UXO (unexploded ordnance) detection.
//!
//! This module implements feature extraction and classification for targets
//! detected by ground-penetrating radar. It provides A-scan peak detection,
//! B-scan cross-range profile extraction, hyperbola fitting for buried objects,
//! target size estimation, metal vs. non-metal discrimination (response polarity
//! analysis), depth estimation, shape-factor and symmetry metrics, clutter
//! rejection, and confidence scoring.
//!
//! # Overview
//!
//! The processing chain is:
//! 1. **A-scan analysis** – detect amplitude peaks above a threshold.
//! 2. **B-scan profiling** – extract cross-range response at a given range bin.
//! 3. **Hyperbola fitting** – estimate propagation velocity and target depth.
//! 4. **Feature extraction** – shape factor, symmetry, polarity, width.
//! 5. **Classification** – combine features to assign a [`TargetType`].
//!
//! # Example
//!
//! ```
//! use r4w_core::gpr_target_discriminator::{
//!     TargetConfig, GprDiscriminator, TargetResponse, TargetType,
//!     extract_ascan_features, estimate_depth, shape_factor, symmetry_index,
//!     classify_target, confidence_score,
//! };
//!
//! let config = TargetConfig {
//!     sample_rate_hz: 10.0e9,
//!     propagation_velocity_mps: 1.0e8,
//!     antenna_height_m: 0.3,
//! };
//!
//! // Extract peaks from a simple A-scan
//! let ascan = vec![0.0, 0.1, 0.8, 0.2, 0.0, 0.05, 0.9, 0.1, 0.0];
//! let peaks = extract_ascan_features(&ascan, 0.5);
//! assert_eq!(peaks.len(), 2);
//!
//! // Depth from two-way travel time
//! let depth = estimate_depth(10.0, 1.0e8);
//! assert!((depth - 0.5).abs() < 1e-9);
//!
//! // Shape factor of a Gaussian-like profile
//! let profile: Vec<f64> = (0..21).map(|i| {
//!     let x = (i as f64 - 10.0) / 3.0;
//!     (-0.5 * x * x).exp()
//! }).collect();
//! let sf = shape_factor(&profile);
//! assert!(sf > 0.9);
//!
//! // Classify a target response
//! let response = TargetResponse {
//!     range_bin: 50,
//!     cross_range_bin: 100,
//!     amplitude: 0.95,
//!     depth_m: 0.15,
//!     shape_factor: 0.92,
//!     symmetry: 0.88,
//!     is_metallic: true,
//!     confidence: 0.85,
//! };
//! let target_type = classify_target(&response);
//! assert!(matches!(target_type, TargetType::Mine | TargetType::UXO));
//! ```

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration parameters for GPR target discrimination.
#[derive(Debug, Clone)]
pub struct TargetConfig {
    /// Sampling rate of the GPR system in Hz.
    pub sample_rate_hz: f64,
    /// Estimated propagation velocity of EM waves in the ground (m/s).
    pub propagation_velocity_mps: f64,
    /// Height of the antenna above the ground surface in metres.
    pub antenna_height_m: f64,
}

// ---------------------------------------------------------------------------
// Target response
// ---------------------------------------------------------------------------

/// Extracted feature vector for a single detected target.
#[derive(Debug, Clone)]
pub struct TargetResponse {
    /// Range-bin index (fast-time sample) of the target apex.
    pub range_bin: usize,
    /// Cross-range bin index (trace number) of the target apex.
    pub cross_range_bin: usize,
    /// Peak amplitude (absolute value, normalised 0–1).
    pub amplitude: f64,
    /// Estimated depth below the ground surface in metres.
    pub depth_m: f64,
    /// Shape factor: Gaussian-likeness metric in \[0, 1\].
    pub shape_factor: f64,
    /// Symmetry index: left-right symmetry in \[0, 1\].
    pub symmetry: f64,
    /// `true` if the response polarity indicates a metallic target.
    pub is_metallic: bool,
    /// Overall detection confidence in \[0, 1\].
    pub confidence: f64,
}

// ---------------------------------------------------------------------------
// Target type
// ---------------------------------------------------------------------------

/// Classification label for a detected GPR target.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetType {
    /// Landmine – shallow, symmetric, metallic, compact response.
    Mine,
    /// Unexploded ordnance – moderate depth, metallic, wider response.
    UXO,
    /// Pipe or utility – elongated, possibly metallic.
    Pipe,
    /// Natural rock or boulder – deep, non-metallic, irregular.
    Rock,
    /// Root or organic clutter – shallow, non-metallic, asymmetric.
    RootClutter,
    /// Unclassifiable target.
    Unknown,
}

// ---------------------------------------------------------------------------
// Main discriminator
// ---------------------------------------------------------------------------

/// GPR target discriminator.
///
/// Holds configuration and provides methods for end-to-end target detection
/// and classification on B-scan data.
#[derive(Debug, Clone)]
pub struct GprDiscriminator {
    config: TargetConfig,
}

impl GprDiscriminator {
    /// Create a new discriminator with the given configuration.
    pub fn new(config: TargetConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the configuration.
    pub fn config(&self) -> &TargetConfig {
        &self.config
    }

    /// Run the full detection pipeline on a B-scan.
    ///
    /// `bscan` is a slice of A-scans (traces). `threshold` controls peak
    /// detection sensitivity, and `dx_m` is the cross-range spacing in metres.
    ///
    /// Returns a list of `(TargetResponse, TargetType)` pairs for every
    /// detected target whose confidence exceeds 0.1.
    pub fn detect_targets(
        &self,
        bscan: &[Vec<f64>],
        threshold: f64,
        dx_m: f64,
    ) -> Vec<(TargetResponse, TargetType)> {
        if bscan.is_empty() {
            return Vec::new();
        }

        let mut results = Vec::new();

        for (trace_idx, trace) in bscan.iter().enumerate() {
            let peaks = extract_ascan_features(trace, threshold);

            for &(range_bin, amplitude) in &peaks {
                // Extract cross-range profile at this range bin.
                let profile = extract_bscan_profile(bscan, range_bin);

                // Check if this trace is actually near the apex of the profile.
                let apex_trace = profile
                    .iter()
                    .enumerate()
                    .max_by(|a, b| {
                        a.1.abs()
                            .partial_cmp(&b.1.abs())
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .map(|(i, _)| i)
                    .unwrap_or(0);

                // Only process near-apex traces to avoid duplicates.
                if trace_idx != apex_trace {
                    continue;
                }

                let sf = shape_factor(&profile);
                let sym = symmetry_index(&profile);
                let metallic = detect_metallic(trace, range_bin);
                let width = estimate_target_size(&profile, threshold * 0.5, dx_m);

                // Depth from range bin index.
                let dt_s = 1.0 / self.config.sample_rate_hz;
                let two_way_time_ns = range_bin as f64 * dt_s * 1e9;
                let depth = estimate_depth(two_way_time_ns, self.config.propagation_velocity_mps);

                // Adjust depth for antenna height.
                let depth_corrected = (depth - self.config.antenna_height_m).max(0.0);

                let norm_amp = amplitude.abs().min(1.0);

                let features = [norm_amp, sf, sym, if metallic { 1.0 } else { 0.0 }, width];
                let conf = confidence_score(&features);

                if conf > 0.1 {
                    let response = TargetResponse {
                        range_bin,
                        cross_range_bin: trace_idx,
                        amplitude: norm_amp,
                        depth_m: depth_corrected,
                        shape_factor: sf,
                        symmetry: sym,
                        is_metallic: metallic,
                        confidence: conf,
                    };
                    let target_type = classify_target(&response);
                    results.push((response, target_type));
                }
            }
        }

        results
    }

    /// Reject clutter targets from a list of detections.
    ///
    /// Removes entries classified as `RootClutter` or whose confidence is
    /// below `min_confidence`.
    pub fn reject_clutter(
        detections: &[(TargetResponse, TargetType)],
        min_confidence: f64,
    ) -> Vec<(TargetResponse, TargetType)> {
        detections
            .iter()
            .filter(|(resp, tt)| {
                *tt != TargetType::RootClutter && resp.confidence >= min_confidence
            })
            .cloned()
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Free functions – A-scan analysis
// ---------------------------------------------------------------------------

/// Extract peaks from an A-scan that exceed `threshold`.
///
/// Returns `(bin_index, amplitude)` for each detected peak. A peak is defined
/// as a sample whose absolute value is greater than both of its immediate
/// neighbours and exceeds `threshold`.
pub fn extract_ascan_features(ascan: &[f64], threshold: f64) -> Vec<(usize, f64)> {
    if ascan.len() < 3 {
        return Vec::new();
    }

    let mut peaks = Vec::new();
    for i in 1..ascan.len() - 1 {
        let v = ascan[i].abs();
        if v > threshold && v > ascan[i - 1].abs() && v > ascan[i + 1].abs() {
            peaks.push((i, ascan[i]));
        }
    }
    peaks
}

// ---------------------------------------------------------------------------
// Free functions – B-scan profiling
// ---------------------------------------------------------------------------

/// Extract the cross-range amplitude profile at a given `range_bin`.
///
/// Returns a vector whose length equals the number of traces in the B-scan.
/// If a trace is shorter than `range_bin + 1`, its entry is zero.
pub fn extract_bscan_profile(bscan: &[Vec<f64>], range_bin: usize) -> Vec<f64> {
    bscan
        .iter()
        .map(|trace| {
            if range_bin < trace.len() {
                trace[range_bin]
            } else {
                0.0
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Free functions – Hyperbola fitting
// ---------------------------------------------------------------------------

/// Fit a diffraction hyperbola to a B-scan around the given `apex` (trace, sample).
///
/// Uses a least-squares fit of the two-way-time model
///     t(x) = (2 / v) * sqrt(d^2 + x^2)
/// where `v` is the propagation velocity and `d` is the depth.
///
/// Returns `(velocity, depth)`. The velocity is in the same units as the
/// trace/sample spacing (dimensionless bin-units unless the caller scales).
/// If the B-scan is too small or the fit degenerates, a fallback is returned.
pub fn fit_hyperbola(bscan: &[Vec<f64>], apex: (usize, usize)) -> (f64, f64) {
    let (apex_trace, apex_sample) = apex;

    if bscan.is_empty() {
        return (1.0e8, 0.0);
    }

    // Collect (offset_x, time_offset) pairs by finding the peak near the
    // expected hyperbola branch for each trace.
    let search_radius = 20usize;
    let mut observations: Vec<(f64, f64)> = Vec::new();

    for (ti, trace) in bscan.iter().enumerate() {
        let dx = ti as f64 - apex_trace as f64;
        if dx.abs() < 0.5 {
            continue; // skip the apex trace itself
        }

        // Search for the local maximum near the expected arrival.
        let expected_shift = dx.abs().sqrt();
        let centre = (apex_sample as f64 + expected_shift * 0.5) as usize;
        let lo = centre.saturating_sub(search_radius);
        let hi = (centre + search_radius).min(trace.len());

        if lo >= hi || lo >= trace.len() {
            continue;
        }

        let (best_bin, _best_val) = trace[lo..hi]
            .iter()
            .enumerate()
            .max_by(|a, b| {
                a.1.abs()
                    .partial_cmp(&b.1.abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, v)| (lo + i, v.abs()))
            .unwrap_or((lo, 0.0));

        let dt = best_bin as f64 - apex_sample as f64;
        if dt > 0.0 {
            observations.push((dx, dt));
        }
    }

    if observations.is_empty() {
        return (1.0e8, 0.0);
    }

    // Linearised least-squares:
    //   (t_apex + dt)^2 = t_apex^2 + 4*x^2/v^2
    //   dt^2 + 2*t_apex*dt = 4*x^2/v^2
    // Let y = dt^2 + 2*t_apex*dt, X = x^2
    // y = a * X  where  a = 4/v^2
    // Simple regression through origin: a = sum(X*y) / sum(X*X)

    let t0 = apex_sample as f64;
    let mut sum_xy = 0.0;
    let mut sum_xx = 0.0;

    for &(x, dt) in &observations {
        let y = dt * dt + 2.0 * t0 * dt;
        let xx = x * x;
        sum_xy += xx * y;
        sum_xx += xx * xx;
    }

    if sum_xx.abs() < 1e-30 {
        return (1.0e8, 0.0);
    }

    let a = sum_xy / sum_xx; // a = 4/v^2

    let velocity = if a > 1e-30 { 2.0 / a.sqrt() } else { 1.0e8 };

    // depth = v * t_apex / 2
    let depth = velocity * t0 / 2.0;

    (velocity, depth)
}

// ---------------------------------------------------------------------------
// Free functions – target size
// ---------------------------------------------------------------------------

/// Estimate target size from a cross-range profile.
///
/// Measures the width (in metres) of the profile region that exceeds
/// `threshold`, multiplied by `dx_m` (cross-range spacing).
pub fn estimate_target_size(profile: &[f64], threshold: f64, dx_m: f64) -> f64 {
    let count = profile.iter().filter(|v| v.abs() > threshold).count();
    count as f64 * dx_m
}

// ---------------------------------------------------------------------------
// Free functions – metallic detection
// ---------------------------------------------------------------------------

/// Detect whether a target is metallic based on response polarity.
///
/// A metallic target produces a strong **negative** reflection (phase reversal)
/// at the top surface due to the high-impedance contrast. This function checks
/// if there is a negative excursion in the samples preceding the peak.
pub fn detect_metallic(ascan: &[f64], target_bin: usize) -> bool {
    if target_bin == 0 || ascan.is_empty() {
        return false;
    }

    // Look at a small window before the peak for a negative excursion.
    let window_start = target_bin.saturating_sub(5);
    let window = &ascan[window_start..target_bin];

    if window.is_empty() {
        return false;
    }

    // A metallic target has a strong negative-going precursor.
    let min_val = window.iter().cloned().fold(f64::INFINITY, f64::min);
    let peak_val = ascan[target_bin].abs();

    if peak_val < 1e-30 {
        return false;
    }

    // If the precursor dip is at least 30% of the peak magnitude and negative,
    // we classify as metallic.
    min_val < 0.0 && min_val.abs() > 0.3 * peak_val
}

// ---------------------------------------------------------------------------
// Free functions – depth estimation
// ---------------------------------------------------------------------------

/// Estimate depth from two-way travel time.
///
/// `two_way_time_ns` is the round-trip travel time in nanoseconds.
/// `velocity_mps` is the propagation velocity in the medium in m/s.
///
/// Returns depth in metres: `d = v * t / 2`, with t converted from ns to s.
pub fn estimate_depth(two_way_time_ns: f64, velocity_mps: f64) -> f64 {
    velocity_mps * two_way_time_ns * 1e-9 / 2.0
}

// ---------------------------------------------------------------------------
// Free functions – shape analysis
// ---------------------------------------------------------------------------

/// Compute a Gaussian-likeness shape factor for a 1-D profile.
///
/// Fits a Gaussian to the profile by estimating the mean and standard
/// deviation from the amplitude-weighted distribution, then computes the
/// normalised correlation between the profile and the fitted Gaussian.
///
/// Returns a value in \[0, 1\] where 1 means a perfect Gaussian.
pub fn shape_factor(profile: &[f64]) -> f64 {
    if profile.is_empty() {
        return 0.0;
    }

    let abs_profile: Vec<f64> = profile.iter().map(|v| v.abs()).collect();
    let total: f64 = abs_profile.iter().sum();
    if total < 1e-30 {
        return 0.0;
    }

    // Weighted mean (centre of mass).
    let mean: f64 = abs_profile
        .iter()
        .enumerate()
        .map(|(i, &v)| i as f64 * v)
        .sum::<f64>()
        / total;

    // Weighted variance.
    let var: f64 = abs_profile
        .iter()
        .enumerate()
        .map(|(i, &v)| {
            let d = i as f64 - mean;
            d * d * v
        })
        .sum::<f64>()
        / total;

    let sigma = var.sqrt();
    if sigma < 1e-30 {
        return 0.0;
    }

    // Build reference Gaussian.
    let peak = abs_profile.iter().cloned().fold(0.0_f64, f64::max);
    let gaussian: Vec<f64> = (0..profile.len())
        .map(|i| {
            let x = (i as f64 - mean) / sigma;
            peak * (-0.5 * x * x).exp()
        })
        .collect();

    // Normalised cross-correlation.
    normalised_correlation(&abs_profile, &gaussian)
}

/// Compute the left-right symmetry index of a 1-D profile.
///
/// The profile is folded around its amplitude-weighted centre and compared
/// to itself. Returns a value in \[0, 1\] where 1 is perfectly symmetric.
pub fn symmetry_index(profile: &[f64]) -> f64 {
    if profile.len() < 3 {
        return 1.0; // trivially symmetric
    }

    let abs_profile: Vec<f64> = profile.iter().map(|v| v.abs()).collect();
    let total: f64 = abs_profile.iter().sum();
    if total < 1e-30 {
        return 0.0;
    }

    // Centre of mass.
    let centre: f64 = abs_profile
        .iter()
        .enumerate()
        .map(|(i, &v)| i as f64 * v)
        .sum::<f64>()
        / total;

    let centre_idx = centre.round() as usize;
    let half_len = centre_idx.min(profile.len() - 1 - centre_idx);
    if half_len == 0 {
        return 1.0;
    }

    // Compare left and right halves.
    let mut numerator = 0.0;
    let mut denom_l = 0.0;
    let mut denom_r = 0.0;

    for k in 1..=half_len {
        let left = abs_profile[centre_idx - k];
        let right = abs_profile[centre_idx + k];
        numerator += left * right;
        denom_l += left * left;
        denom_r += right * right;
    }

    let denom = (denom_l * denom_r).sqrt();
    if denom < 1e-30 {
        return 0.0;
    }

    (numerator / denom).clamp(0.0, 1.0)
}

// ---------------------------------------------------------------------------
// Free functions – classification
// ---------------------------------------------------------------------------

/// Classify a target based on its extracted features.
///
/// Uses a simple decision-tree approach combining depth, shape factor,
/// symmetry, metallicity, and amplitude.
pub fn classify_target(response: &TargetResponse) -> TargetType {
    let TargetResponse {
        amplitude: _,
        depth_m,
        shape_factor: sf,
        symmetry: sym,
        is_metallic,
        confidence,
        ..
    } = *response;

    // Very low confidence -> Unknown
    if confidence < 0.2 {
        return TargetType::Unknown;
    }

    // High symmetry + metallic + shallow + compact shape -> Mine
    if is_metallic && depth_m < 0.3 && sf > 0.6 && sym > 0.6 {
        return TargetType::Mine;
    }

    // Metallic + moderate depth + moderate shape -> UXO
    if is_metallic && depth_m >= 0.3 && sf > 0.4 && sym > 0.4 {
        return TargetType::UXO;
    }

    // Metallic but low symmetry -> Pipe
    if is_metallic && sym < 0.4 {
        return TargetType::Pipe;
    }

    // Non-metallic, deep, irregular -> Rock
    if !is_metallic && depth_m > 0.5 && sf < 0.5 {
        return TargetType::Rock;
    }

    // Non-metallic, shallow, asymmetric -> RootClutter
    if !is_metallic && depth_m < 0.3 && sym < 0.5 {
        return TargetType::RootClutter;
    }

    // Non-metallic, shallow, high symmetry + shape -> Mine (plastic)
    if !is_metallic && depth_m < 0.3 && sf > 0.7 && sym > 0.7 && response.amplitude > 0.5 {
        return TargetType::Mine;
    }

    TargetType::Unknown
}

// ---------------------------------------------------------------------------
// Free functions – confidence scoring
// ---------------------------------------------------------------------------

/// Compute a combined confidence score from a feature vector.
///
/// Expected features (order matters):
///   \[amplitude, shape_factor, symmetry, metallic_flag (0 or 1), width\]
///
/// Returns a value in \[0, 1\]. Higher is more confident.
pub fn confidence_score(features: &[f64]) -> f64 {
    if features.is_empty() {
        return 0.0;
    }

    // Weighted combination (amplitude, shape factor, and symmetry are the
    // strongest indicators).
    let weights = [0.30, 0.25, 0.25, 0.10, 0.10];
    let n = features.len().min(weights.len());

    let mut score = 0.0;
    for i in 0..n {
        score += weights[i] * features[i].clamp(0.0, 1.0);
    }

    score.clamp(0.0, 1.0)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Normalised cross-correlation between two equal-length vectors.
fn normalised_correlation(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len().min(b.len());
    if n == 0 {
        return 0.0;
    }

    let mean_a: f64 = a[..n].iter().sum::<f64>() / n as f64;
    let mean_b: f64 = b[..n].iter().sum::<f64>() / n as f64;

    let mut num = 0.0;
    let mut den_a = 0.0;
    let mut den_b = 0.0;

    for i in 0..n {
        let da = a[i] - mean_a;
        let db = b[i] - mean_b;
        num += da * db;
        den_a += da * da;
        den_b += db * db;
    }

    let denom = (den_a * den_b).sqrt();
    if denom < 1e-30 {
        return 0.0;
    }

    (num / denom).clamp(0.0, 1.0)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -- TargetConfig -------------------------------------------------------

    #[test]
    fn test_target_config_creation() {
        let cfg = TargetConfig {
            sample_rate_hz: 10.0e9,
            propagation_velocity_mps: 1.0e8,
            antenna_height_m: 0.3,
        };
        assert_eq!(cfg.sample_rate_hz, 10.0e9);
        assert_eq!(cfg.propagation_velocity_mps, 1.0e8);
        assert_eq!(cfg.antenna_height_m, 0.3);
    }

    // -- GprDiscriminator ---------------------------------------------------

    #[test]
    fn test_discriminator_new_and_config() {
        let cfg = TargetConfig {
            sample_rate_hz: 5.0e9,
            propagation_velocity_mps: 1.5e8,
            antenna_height_m: 0.2,
        };
        let disc = GprDiscriminator::new(cfg);
        assert_eq!(disc.config().sample_rate_hz, 5.0e9);
        assert_eq!(disc.config().antenna_height_m, 0.2);
    }

    #[test]
    fn test_discriminator_detect_empty_bscan() {
        let cfg = TargetConfig {
            sample_rate_hz: 10.0e9,
            propagation_velocity_mps: 1.0e8,
            antenna_height_m: 0.3,
        };
        let disc = GprDiscriminator::new(cfg);
        let results = disc.detect_targets(&[], 0.5, 0.02);
        assert!(results.is_empty());
    }

    #[test]
    fn test_discriminator_detect_single_target() {
        let cfg = TargetConfig {
            sample_rate_hz: 10.0e9,
            propagation_velocity_mps: 1.0e8,
            antenna_height_m: 0.0,
        };
        let disc = GprDiscriminator::new(cfg);

        // Build a B-scan with a peak at trace 5, sample 30.
        let num_traces = 11;
        let num_samples = 64;
        let mut bscan: Vec<Vec<f64>> = vec![vec![0.0; num_samples]; num_traces];

        // Put a Gaussian-shaped target centred at trace 5.
        for t in 0..num_traces {
            let dx = t as f64 - 5.0;
            let amp = (-dx * dx / 4.0).exp();
            // Negative precursor for metallic detection.
            bscan[t][27] = -0.4 * amp;
            bscan[t][30] = amp;
        }

        let results = disc.detect_targets(&bscan, 0.3, 0.02);
        assert!(!results.is_empty(), "Should detect at least one target");
    }

    // -- extract_ascan_features ---------------------------------------------

    #[test]
    fn test_ascan_features_basic_peaks() {
        let ascan = vec![0.0, 0.1, 0.8, 0.2, 0.0, 0.05, 0.9, 0.1, 0.0];
        let peaks = extract_ascan_features(&ascan, 0.5);
        assert_eq!(peaks.len(), 2);
        assert_eq!(peaks[0].0, 2); // bin 2
        assert_eq!(peaks[1].0, 6); // bin 6
    }

    #[test]
    fn test_ascan_features_no_peaks() {
        let ascan = vec![0.1, 0.2, 0.3, 0.2, 0.1];
        let peaks = extract_ascan_features(&ascan, 0.5);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_ascan_features_negative_peak() {
        // Absolute value should be used for threshold comparison.
        let ascan = vec![0.0, 0.1, -0.9, 0.2, 0.0];
        let peaks = extract_ascan_features(&ascan, 0.5);
        assert_eq!(peaks.len(), 1);
        assert_eq!(peaks[0].0, 2);
        assert!(peaks[0].1 < 0.0); // preserves sign
    }

    #[test]
    fn test_ascan_features_short_input() {
        let ascan = vec![1.0, 2.0];
        let peaks = extract_ascan_features(&ascan, 0.0);
        assert!(peaks.is_empty()); // too short for peak detection
    }

    // -- extract_bscan_profile ----------------------------------------------

    #[test]
    fn test_bscan_profile_extraction() {
        let bscan = vec![
            vec![0.0, 0.1, 0.2],
            vec![0.0, 0.5, 0.3],
            vec![0.0, 0.9, 0.1],
            vec![0.0, 0.4, 0.2],
        ];
        let profile = extract_bscan_profile(&bscan, 1);
        assert_eq!(profile.len(), 4);
        assert!((profile[2] - 0.9).abs() < 1e-10);
    }

    #[test]
    fn test_bscan_profile_out_of_range() {
        let bscan = vec![vec![1.0, 2.0], vec![3.0]];
        let profile = extract_bscan_profile(&bscan, 1);
        assert_eq!(profile.len(), 2);
        assert!((profile[0] - 2.0).abs() < 1e-10);
        assert!((profile[1] - 0.0).abs() < 1e-10); // trace too short
    }

    // -- fit_hyperbola ------------------------------------------------------

    #[test]
    fn test_fit_hyperbola_basic() {
        // Create a synthetic hyperbola: t(x) = sqrt(t0^2 + (2x/v)^2)
        let v = 2.0;
        let t0 = 20.0;
        let num_traces = 21;
        let num_samples = 60;
        let apex_trace = 10;

        let mut bscan: Vec<Vec<f64>> = vec![vec![0.0; num_samples]; num_traces];
        for t in 0..num_traces {
            let x = t as f64 - apex_trace as f64;
            let arrival = (t0 * t0 + (2.0 * x / v) * (2.0 * x / v)).sqrt();
            let bin = arrival.round() as usize;
            if bin < num_samples {
                bscan[t][bin] = 1.0;
            }
        }

        let (vel, depth) = fit_hyperbola(&bscan, (apex_trace, t0 as usize));
        assert!(vel > 0.0, "Velocity should be positive");
        assert!(depth >= 0.0, "Depth should be non-negative");
    }

    #[test]
    fn test_fit_hyperbola_empty() {
        let (v, d) = fit_hyperbola(&[], (0, 0));
        assert!(v > 0.0);
        assert!((d - 0.0).abs() < 1e-10);
    }

    // -- estimate_target_size -----------------------------------------------

    #[test]
    fn test_estimate_target_size() {
        let profile = vec![0.0, 0.1, 0.6, 0.9, 0.7, 0.2, 0.0];
        let size = estimate_target_size(&profile, 0.5, 0.02);
        // Three samples above 0.5 -> 3 * 0.02 = 0.06
        assert!((size - 0.06).abs() < 1e-10);
    }

    #[test]
    fn test_estimate_target_size_none_above() {
        let profile = vec![0.1, 0.2, 0.3];
        let size = estimate_target_size(&profile, 0.5, 0.02);
        assert!((size - 0.0).abs() < 1e-10);
    }

    // -- detect_metallic ----------------------------------------------------

    #[test]
    fn test_detect_metallic_positive() {
        // Negative precursor followed by positive peak.
        let ascan = vec![0.0, 0.0, -0.6, -0.4, 0.1, 1.0, 0.3, 0.0];
        assert!(detect_metallic(&ascan, 5));
    }

    #[test]
    fn test_detect_metallic_negative() {
        // No negative precursor.
        let ascan = vec![0.0, 0.1, 0.3, 0.5, 1.0, 0.5, 0.2];
        assert!(!detect_metallic(&ascan, 4));
    }

    #[test]
    fn test_detect_metallic_edge_cases() {
        assert!(!detect_metallic(&[], 0));
        assert!(!detect_metallic(&[1.0], 0));
    }

    // -- estimate_depth -----------------------------------------------------

    #[test]
    fn test_estimate_depth() {
        // 10 ns two-way time at 1e8 m/s -> depth = 0.5 m
        let d = estimate_depth(10.0, 1.0e8);
        assert!((d - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_estimate_depth_zero() {
        assert!((estimate_depth(0.0, 1.0e8) - 0.0).abs() < 1e-10);
    }

    // -- shape_factor -------------------------------------------------------

    #[test]
    fn test_shape_factor_gaussian() {
        let profile: Vec<f64> = (0..41)
            .map(|i| {
                let x = (i as f64 - 20.0) / 5.0;
                (-0.5 * x * x).exp()
            })
            .collect();
        let sf = shape_factor(&profile);
        assert!(sf > 0.95, "Gaussian profile should have sf > 0.95, got {sf}");
    }

    #[test]
    fn test_shape_factor_flat() {
        let profile = vec![1.0; 20];
        let sf = shape_factor(&profile);
        // A flat profile is not Gaussian-like.
        assert!(sf < 0.8, "Flat profile should have low sf, got {sf}");
    }

    #[test]
    fn test_shape_factor_empty() {
        assert!((shape_factor(&[]) - 0.0).abs() < 1e-10);
    }

    // -- symmetry_index -----------------------------------------------------

    #[test]
    fn test_symmetry_index_symmetric() {
        let profile = vec![0.0, 0.2, 0.5, 1.0, 0.5, 0.2, 0.0];
        let sym = symmetry_index(&profile);
        assert!(
            sym > 0.95,
            "Symmetric profile should have sym > 0.95, got {sym}"
        );
    }

    #[test]
    fn test_symmetry_index_asymmetric() {
        let profile = vec![0.0, 0.0, 1.0, 0.8, 0.1, 0.0, 0.0];
        let sym = symmetry_index(&profile);
        let profile2 = vec![0.0, 0.0, 0.0, 1.0, 0.9, 0.8, 0.7];
        let sym2 = symmetry_index(&profile2);
        assert!(
            sym2 < sym,
            "More asymmetric profile should have lower symmetry"
        );
    }

    #[test]
    fn test_symmetry_index_trivial() {
        assert!((symmetry_index(&[1.0, 2.0]) - 1.0).abs() < 1e-10);
    }

    // -- classify_target ----------------------------------------------------

    #[test]
    fn test_classify_mine() {
        let response = TargetResponse {
            range_bin: 50,
            cross_range_bin: 10,
            amplitude: 0.9,
            depth_m: 0.15,
            shape_factor: 0.85,
            symmetry: 0.80,
            is_metallic: true,
            confidence: 0.8,
        };
        assert_eq!(classify_target(&response), TargetType::Mine);
    }

    #[test]
    fn test_classify_uxo() {
        let response = TargetResponse {
            range_bin: 100,
            cross_range_bin: 20,
            amplitude: 0.85,
            depth_m: 0.5,
            shape_factor: 0.7,
            symmetry: 0.65,
            is_metallic: true,
            confidence: 0.75,
        };
        assert_eq!(classify_target(&response), TargetType::UXO);
    }

    #[test]
    fn test_classify_root_clutter() {
        let response = TargetResponse {
            range_bin: 20,
            cross_range_bin: 5,
            amplitude: 0.4,
            depth_m: 0.1,
            shape_factor: 0.3,
            symmetry: 0.2,
            is_metallic: false,
            confidence: 0.5,
        };
        assert_eq!(classify_target(&response), TargetType::RootClutter);
    }

    #[test]
    fn test_classify_rock() {
        let response = TargetResponse {
            range_bin: 200,
            cross_range_bin: 30,
            amplitude: 0.6,
            depth_m: 1.0,
            shape_factor: 0.3,
            symmetry: 0.4,
            is_metallic: false,
            confidence: 0.6,
        };
        assert_eq!(classify_target(&response), TargetType::Rock);
    }

    #[test]
    fn test_classify_unknown_low_confidence() {
        let response = TargetResponse {
            range_bin: 10,
            cross_range_bin: 5,
            amplitude: 0.5,
            depth_m: 0.2,
            shape_factor: 0.5,
            symmetry: 0.5,
            is_metallic: false,
            confidence: 0.1,
        };
        assert_eq!(classify_target(&response), TargetType::Unknown);
    }

    // -- confidence_score ---------------------------------------------------

    #[test]
    fn test_confidence_score_high() {
        let features = [1.0, 1.0, 1.0, 1.0, 1.0];
        let score = confidence_score(&features);
        assert!((score - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_confidence_score_zero() {
        let features = [0.0, 0.0, 0.0, 0.0, 0.0];
        let score = confidence_score(&features);
        assert!((score - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_confidence_score_empty() {
        assert!((confidence_score(&[]) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_confidence_score_clamped() {
        let features = [2.0, 2.0, 2.0, 2.0, 2.0];
        let score = confidence_score(&features);
        assert!((score - 1.0).abs() < 1e-10, "Should clamp to 1.0");
    }

    // -- reject_clutter -----------------------------------------------------

    #[test]
    fn test_reject_clutter() {
        let mine = TargetResponse {
            range_bin: 50,
            cross_range_bin: 10,
            amplitude: 0.9,
            depth_m: 0.15,
            shape_factor: 0.85,
            symmetry: 0.80,
            is_metallic: true,
            confidence: 0.8,
        };
        let clutter = TargetResponse {
            range_bin: 20,
            cross_range_bin: 5,
            amplitude: 0.4,
            depth_m: 0.1,
            shape_factor: 0.3,
            symmetry: 0.2,
            is_metallic: false,
            confidence: 0.3,
        };
        let low_conf = TargetResponse {
            range_bin: 30,
            cross_range_bin: 7,
            amplitude: 0.5,
            depth_m: 0.2,
            shape_factor: 0.5,
            symmetry: 0.5,
            is_metallic: true,
            confidence: 0.2,
        };

        let detections = vec![
            (mine, TargetType::Mine),
            (clutter, TargetType::RootClutter),
            (low_conf, TargetType::UXO),
        ];

        let filtered = GprDiscriminator::reject_clutter(&detections, 0.5);
        assert_eq!(filtered.len(), 1);
        assert_eq!(filtered[0].1, TargetType::Mine);
    }
}
