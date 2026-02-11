//! Tidal Harmonic Analyzer
//!
//! Performs tidal harmonic analysis on water level time series data.
//! Extracts tidal constituents (M2, S2, N2, K1, O1, etc.) and predicts
//! future tide levels using harmonic synthesis.
//!
//! The analysis uses least-squares fitting to decompose observed water levels
//! into sinusoidal components at known astronomical frequencies. The Rayleigh
//! criterion is applied to determine which constituents can be independently
//! resolved given the record length.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::tidal_harmonic_analyzer::{
//!     TidalConfig, TidalAnalyzer, predict_tide, standard_constituents,
//!     generate_synthetic_tide,
//! };
//!
//! // Generate synthetic tide data (1 hour intervals, 30 days)
//! let constituents_input: Vec<(f64, f64, f64)> = vec![
//!     (0.0805114, 1.0, 0.0),   // M2: freq_cph, amplitude_m, phase_deg
//!     (0.0833333, 0.35, 30.0), // S2
//!     (0.0417807, 0.45, 60.0), // K1
//! ];
//! let data = generate_synthetic_tide(&constituents_input, 2.0, 1.0, 720);
//!
//! // Analyze the tide data
//! let config = TidalConfig {
//!     sample_interval_hours: 1.0,
//!     analysis_period_days: 30.0,
//!     num_constituents: 8,
//! };
//! let analyzer = TidalAnalyzer::new(config);
//! let result = analyzer.analyze(&data);
//!
//! // Predict tides for the next 24 hours
//! let hours: Vec<f64> = (0..24).map(|h| h as f64).collect();
//! let predicted = predict_tide(&result.constituents, result.mean_level, &hours);
//! assert_eq!(predicted.len(), 24);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for tidal harmonic analysis.
#[derive(Debug, Clone)]
pub struct TidalConfig {
    /// Sampling interval of the water level observations (hours).
    pub sample_interval_hours: f64,
    /// Duration of the analysis window (days). Longer records resolve more
    /// constituents via the Rayleigh criterion.
    pub analysis_period_days: f64,
    /// Maximum number of tidal constituents to extract.
    pub num_constituents: usize,
}

impl Default for TidalConfig {
    fn default() -> Self {
        Self {
            sample_interval_hours: 1.0,
            analysis_period_days: 29.53, // ~1 synodic month
            num_constituents: 8,
        }
    }
}

// ---------------------------------------------------------------------------
// Result types
// ---------------------------------------------------------------------------

/// A single tidal constituent extracted from the harmonic analysis.
#[derive(Debug, Clone)]
pub struct TidalConstituent {
    /// Traditional Doodson/IHO name (e.g. "M2", "K1").
    pub name: String,
    /// Angular frequency in cycles per hour (cph).
    pub frequency_cph: f64,
    /// Amplitude in the same units as the input water levels.
    pub amplitude: f64,
    /// Phase in degrees (0-360), referenced to the start of the record.
    pub phase_deg: f64,
    /// Signal-to-noise ratio for this constituent (amplitude / residual_rms).
    pub snr: f64,
}

/// Result of a tidal harmonic analysis.
#[derive(Debug, Clone)]
pub struct TidalResult {
    /// Extracted tidal constituents sorted by descending amplitude.
    pub constituents: Vec<TidalConstituent>,
    /// Mean water level (Z0) over the analysis period.
    pub mean_level: f64,
    /// RMS of the residual after removing all fitted constituents.
    pub residual_rms: f64,
}

// ---------------------------------------------------------------------------
// Analyzer
// ---------------------------------------------------------------------------

/// Tidal harmonic analyzer.
///
/// Decomposes a water level time series into tidal constituents using
/// least-squares fitting at known astronomical frequencies.
#[derive(Debug, Clone)]
pub struct TidalAnalyzer {
    config: TidalConfig,
}

impl TidalAnalyzer {
    /// Create a new analyzer with the given configuration.
    pub fn new(config: TidalConfig) -> Self {
        Self { config }
    }

    /// Perform harmonic analysis on the water level time series.
    ///
    /// Returns a [`TidalResult`] containing the extracted constituents, mean
    /// level, and residual RMS.
    pub fn analyze(&self, water_levels: &[f64]) -> TidalResult {
        assert!(
            !water_levels.is_empty(),
            "water_levels must not be empty"
        );

        let n = water_levels.len();
        let dt = self.config.sample_interval_hours;
        let record_hours = (n - 1) as f64 * dt;

        // Compute mean level
        let mean_level = water_levels.iter().sum::<f64>() / n as f64;

        // Remove mean
        let demeaned: Vec<f64> = water_levels.iter().map(|&v| v - mean_level).collect();

        // Get standard constituents and filter by Rayleigh criterion
        let all_constituents = standard_constituents();
        let mut freqs: Vec<(String, f64)> = Vec::new();
        for (name, freq) in all_constituents.iter() {
            if freqs.len() >= self.config.num_constituents {
                break;
            }
            // Check Rayleigh criterion against all already-selected frequencies
            let mut resolvable = true;
            for (_, existing_freq) in &freqs {
                if !rayleigh_criterion(*freq, *existing_freq, record_hours) {
                    resolvable = false;
                    break;
                }
            }
            if resolvable {
                freqs.push((name.clone(), *freq));
            }
        }

        // Extract just the frequency values for least-squares
        let freq_vals: Vec<f64> = freqs.iter().map(|(_, f)| *f).collect();

        // Perform least-squares harmonic fit
        let amp_phase = least_squares_harmonic(&demeaned, &freq_vals, dt);

        // Compute residual
        let mut residual = demeaned.clone();
        for (i, &(amp, phase_rad)) in amp_phase.iter().enumerate() {
            let omega = 2.0 * PI * freq_vals[i];
            for j in 0..n {
                let t = j as f64 * dt;
                residual[j] -= amp * (omega * t + phase_rad).cos();
            }
        }
        let residual_rms = (residual.iter().map(|r| r * r).sum::<f64>() / n as f64).sqrt();

        // Build constituent results
        let mut constituents: Vec<TidalConstituent> = freqs
            .iter()
            .zip(amp_phase.iter())
            .map(|((name, freq), &(amp, phase_rad))| {
                let mut phase_deg = phase_rad.to_degrees();
                // Normalize to [0, 360)
                while phase_deg < 0.0 {
                    phase_deg += 360.0;
                }
                while phase_deg >= 360.0 {
                    phase_deg -= 360.0;
                }
                let snr = if residual_rms > 1e-15 {
                    amp / residual_rms
                } else {
                    f64::INFINITY
                };
                TidalConstituent {
                    name: name.clone(),
                    frequency_cph: *freq,
                    amplitude: amp,
                    phase_deg,
                    snr,
                }
            })
            .collect();

        // Sort by descending amplitude
        constituents.sort_by(|a, b| {
            b.amplitude
                .partial_cmp(&a.amplitude)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        TidalResult {
            constituents,
            mean_level,
            residual_rms,
        }
    }
}

// ---------------------------------------------------------------------------
// Prediction
// ---------------------------------------------------------------------------

/// Predict tide levels at specified hours using harmonic synthesis.
///
/// Each predicted value is computed as:
/// ```text
/// h(t) = Z0 + sum_i [ A_i * cos(2*pi*f_i*t + phi_i) ]
/// ```
/// where Z0 is the mean level, A_i, f_i, phi_i are amplitude, frequency
/// (cph), and phase (converted to radians) of each constituent.
pub fn predict_tide(
    constituents: &[TidalConstituent],
    mean_level: f64,
    hours: &[f64],
) -> Vec<f64> {
    hours
        .iter()
        .map(|&t| {
            let mut h = mean_level;
            for c in constituents {
                let omega = 2.0 * PI * c.frequency_cph;
                let phase_rad = c.phase_deg.to_radians();
                h += c.amplitude * (omega * t + phase_rad).cos();
            }
            h
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Standard constituents
// ---------------------------------------------------------------------------

/// Return the standard tidal constituents with their astronomical frequencies
/// in cycles per hour (cph).
///
/// The eight major constituents are returned in order of typical dominance:
///
/// | Name | Period (h)  | Frequency (cph) | Description                |
/// |------|-------------|-----------------|----------------------------|
/// | M2   | 12.4206     | 0.0805114       | Principal lunar semidiurnal |
/// | S2   | 12.0000     | 0.0833333       | Principal solar semidiurnal |
/// | N2   | 12.6583     | 0.0789992       | Larger lunar elliptic       |
/// | K1   | 23.9345     | 0.0417807       | Lunisolar diurnal           |
/// | O1   | 25.8193     | 0.0387307       | Principal lunar diurnal     |
/// | K2   | 11.9672     | 0.0835615       | Lunisolar semidiurnal       |
/// | P1   | 24.0659     | 0.0415526       | Principal solar diurnal     |
/// | Q1   | 26.8684     | 0.0372185       | Larger lunar elliptic diurn.|
pub fn standard_constituents() -> Vec<(String, f64)> {
    vec![
        ("M2".to_string(), 1.0 / 12.4206012),  // 0.080511...
        ("S2".to_string(), 1.0 / 12.0),         // 0.083333...
        ("N2".to_string(), 1.0 / 12.6583482),   // 0.078999...
        ("K1".to_string(), 1.0 / 23.9344696),   // 0.041781...
        ("O1".to_string(), 1.0 / 25.8193417),   // 0.038731...
        ("K2".to_string(), 1.0 / 11.9672348),   // 0.083562...
        ("P1".to_string(), 1.0 / 24.0658902),   // 0.041553...
        ("Q1".to_string(), 1.0 / 26.8683567),   // 0.037219...
    ]
}

// ---------------------------------------------------------------------------
// Least-squares harmonic analysis
// ---------------------------------------------------------------------------

/// Solve for tidal harmonic amplitudes and phases using the normal equations.
///
/// For each frequency f_k, the model is:
/// ```text
/// h(t) = sum_k [ a_k * cos(2*pi*f_k*t) + b_k * sin(2*pi*f_k*t) ]
/// ```
///
/// We solve the linear system `A^T A x = A^T d` where the columns of A are
/// cosine and sine basis functions at each frequency.
///
/// Returns a vector of `(amplitude, phase_radians)` pairs, one per frequency.
pub fn least_squares_harmonic(
    data: &[f64],
    frequencies: &[f64],
    dt_hours: f64,
) -> Vec<(f64, f64)> {
    let n = data.len();
    let m = frequencies.len() * 2; // cos and sin for each frequency

    // Build basis matrix columns (stored column-major for efficiency)
    // Column 2*k   = cos(omega_k * t_j)
    // Column 2*k+1 = sin(omega_k * t_j)
    let mut basis: Vec<Vec<f64>> = Vec::with_capacity(m);
    for &freq in frequencies {
        let omega = 2.0 * PI * freq;
        let mut cos_col = Vec::with_capacity(n);
        let mut sin_col = Vec::with_capacity(n);
        for j in 0..n {
            let t = j as f64 * dt_hours;
            let arg = omega * t;
            cos_col.push(arg.cos());
            sin_col.push(arg.sin());
        }
        basis.push(cos_col);
        basis.push(sin_col);
    }

    // Compute A^T * A  (m x m symmetric matrix)
    let mut ata = vec![vec![0.0f64; m]; m];
    for i in 0..m {
        for j in i..m {
            let mut dot = 0.0;
            for k in 0..n {
                dot += basis[i][k] * basis[j][k];
            }
            ata[i][j] = dot;
            ata[j][i] = dot;
        }
    }

    // Compute A^T * d  (m-vector)
    let mut atd = vec![0.0f64; m];
    for i in 0..m {
        let mut dot = 0.0;
        for k in 0..n {
            dot += basis[i][k] * data[k];
        }
        atd[i] = dot;
    }

    // Solve via Cholesky decomposition (A^T*A is positive-definite for well-conditioned data)
    let x = solve_symmetric_positive_definite(&ata, &atd);

    // Convert (a_k, b_k) to (amplitude, phase) for each frequency
    let mut result = Vec::with_capacity(frequencies.len());
    for k in 0..frequencies.len() {
        let a = x[2 * k];     // cosine coefficient
        let b = x[2 * k + 1]; // sine coefficient
        let amplitude = (a * a + b * b).sqrt();
        // h(t) = a*cos(wt) + b*sin(wt) = A*cos(wt + phi)
        // where A = sqrt(a^2+b^2), phi = atan2(-b, a)
        let phase_rad = (-b).atan2(a);
        result.push((amplitude, phase_rad));
    }

    result
}

/// Solve A*x = b for symmetric positive-definite A using Cholesky decomposition.
///
/// Decomposes A = L * L^T, then solves L*y = b (forward substitution)
/// followed by L^T*x = y (back substitution).
fn solve_symmetric_positive_definite(a: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    let n = a.len();
    assert_eq!(b.len(), n);

    // Cholesky decomposition: A = L * L^T
    let mut l = vec![vec![0.0f64; n]; n];
    for i in 0..n {
        for j in 0..=i {
            let mut sum = 0.0;
            for k in 0..j {
                sum += l[i][k] * l[j][k];
            }
            if i == j {
                let diag = a[i][i] - sum;
                // Add small regularization if near-singular
                l[i][j] = if diag > 1e-15 {
                    diag.sqrt()
                } else {
                    1e-8_f64.sqrt()
                };
            } else {
                l[i][j] = (a[i][j] - sum) / l[j][j];
            }
        }
    }

    // Forward substitution: L * y = b
    let mut y = vec![0.0f64; n];
    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..i {
            sum += l[i][j] * y[j];
        }
        y[i] = (b[i] - sum) / l[i][i];
    }

    // Back substitution: L^T * x = y
    let mut x = vec![0.0f64; n];
    for i in (0..n).rev() {
        let mut sum = 0.0;
        for j in (i + 1)..n {
            sum += l[j][i] * x[j]; // L^T[i][j] = L[j][i]
        }
        x[i] = (y[i] - sum) / l[i][i];
    }

    x
}

// ---------------------------------------------------------------------------
// Rayleigh criterion
// ---------------------------------------------------------------------------

/// Determine whether two tidal constituents can be independently resolved
/// given the record length.
///
/// The Rayleigh criterion requires:
/// ```text
/// |f1 - f2| >= 1 / T
/// ```
/// where T is the record length in hours and f1, f2 are frequencies in cph.
///
/// Returns `true` if the two constituents are resolvable.
pub fn rayleigh_criterion(freq1: f64, freq2: f64, record_length_hours: f64) -> bool {
    let delta_f = (freq1 - freq2).abs();
    let resolution = 1.0 / record_length_hours;
    delta_f >= resolution
}

// ---------------------------------------------------------------------------
// Nodal corrections
// ---------------------------------------------------------------------------

/// Compute the 18.61-year nodal corrections for a tidal constituent.
///
/// Returns `(f, u)` where:
/// - `f` is the amplitude correction factor (dimensionless, ~1.0)
/// - `u` is the phase correction in degrees
///
/// These corrections account for the regression of the lunar nodes over
/// the 18.61-year cycle. The longitude of the ascending node N is
/// approximated as:
/// ```text
/// N = 259.1560564 - 19.328185764 * T
/// ```
/// where T is centuries from J2000.0 (year 2000.0).
///
/// Supported constituents: M2, S2, N2, K1, O1, K2, P1, Q1.
/// Unknown constituents return `(1.0, 0.0)`.
pub fn nodal_corrections(year: f64, constituent: &str) -> (f64, f64) {
    // Compute longitude of lunar ascending node
    let t = (year - 2000.0) / 100.0; // centuries from J2000.0
    let n_deg = 259.1560564 - 19.328185764 * 360.0 * t;
    let n_rad = normalize_angle_deg(n_deg).to_radians();

    match constituent {
        "M2" => {
            // f = 1 - 0.037 * cos(N)
            // u = -2.1 * sin(N)  degrees
            let f = 1.0 - 0.037 * n_rad.cos();
            let u = -2.1 * n_rad.sin();
            (f, u)
        }
        "S2" => {
            // Solar tide: no nodal correction
            (1.0, 0.0)
        }
        "N2" => {
            // Same nodal behavior as M2
            let f = 1.0 - 0.037 * n_rad.cos();
            let u = -2.1 * n_rad.sin();
            (f, u)
        }
        "K1" => {
            // f = 1.006 + 0.115 * cos(N)
            // u = -8.9 * sin(N)  degrees
            let f = 1.006 + 0.115 * n_rad.cos();
            let u = -8.9 * n_rad.sin();
            (f, u)
        }
        "O1" => {
            // f = 1.009 + 0.187 * cos(N)
            // u = 10.8 * sin(N)  degrees
            let f = 1.009 + 0.187 * n_rad.cos();
            let u = 10.8 * n_rad.sin();
            (f, u)
        }
        "K2" => {
            // f = 1.024 + 0.286 * cos(N)
            // u = -17.7 * sin(N)  degrees
            let f = 1.024 + 0.286 * n_rad.cos();
            let u = -17.7 * n_rad.sin();
            (f, u)
        }
        "P1" => {
            // Solar tide: no nodal correction
            (1.0, 0.0)
        }
        "Q1" => {
            // Same nodal behavior as O1
            let f = 1.009 + 0.187 * n_rad.cos();
            let u = 10.8 * n_rad.sin();
            (f, u)
        }
        _ => (1.0, 0.0),
    }
}

/// Normalize an angle to [0, 360) degrees.
fn normalize_angle_deg(deg: f64) -> f64 {
    let mut d = deg % 360.0;
    if d < 0.0 {
        d += 360.0;
    }
    d
}

// ---------------------------------------------------------------------------
// Tidal form number
// ---------------------------------------------------------------------------

/// Compute the tidal form number F.
///
/// ```text
/// F = (K1 + O1) / (M2 + S2)
/// ```
///
/// Classification:
/// - F < 0.25  : Semidiurnal
/// - 0.25 <= F < 1.5 : Mixed, mainly semidiurnal
/// - 1.5  <= F < 3.0 : Mixed, mainly diurnal
/// - F >= 3.0  : Diurnal
///
/// All amplitudes should be in the same units.
pub fn tidal_form_number(
    k1_amp: f64,
    o1_amp: f64,
    m2_amp: f64,
    s2_amp: f64,
) -> f64 {
    let denom = m2_amp + s2_amp;
    if denom.abs() < 1e-15 {
        return f64::INFINITY;
    }
    (k1_amp + o1_amp) / denom
}

// ---------------------------------------------------------------------------
// Synthetic tide generator
// ---------------------------------------------------------------------------

/// Generate a synthetic tide signal for testing purposes.
///
/// Each element of `constituents` is `(frequency_cph, amplitude, phase_deg)`.
///
/// ```text
/// h(t) = mean_level + sum_i [ A_i * cos(2*pi*f_i*t + phi_i) ]
/// ```
///
/// Returns `num_points` samples at the given time step.
pub fn generate_synthetic_tide(
    constituents: &[(f64, f64, f64)],
    mean_level: f64,
    dt_hours: f64,
    num_points: usize,
) -> Vec<f64> {
    (0..num_points)
        .map(|j| {
            let t = j as f64 * dt_hours;
            let mut h = mean_level;
            for &(freq, amp, phase_deg) in constituents {
                let omega = 2.0 * PI * freq;
                let phi = phase_deg.to_radians();
                h += amp * (omega * t + phi).cos();
            }
            h
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Helper: classify tide type from form number
// ---------------------------------------------------------------------------

/// Classify the tide type from the form number.
///
/// Returns a string describing the tidal regime.
pub fn classify_tide(form_number: f64) -> &'static str {
    if form_number < 0.25 {
        "Semidiurnal"
    } else if form_number < 1.5 {
        "Mixed, mainly semidiurnal"
    } else if form_number < 3.0 {
        "Mixed, mainly diurnal"
    } else {
        "Diurnal"
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOLERANCE: f64 = 1e-6;

    // --- Standard constituent tests ---

    #[test]
    fn test_standard_constituent_count() {
        let sc = standard_constituents();
        assert_eq!(sc.len(), 8);
    }

    #[test]
    fn test_standard_constituent_names() {
        let sc = standard_constituents();
        let names: Vec<&str> = sc.iter().map(|(n, _)| n.as_str()).collect();
        assert!(names.contains(&"M2"));
        assert!(names.contains(&"S2"));
        assert!(names.contains(&"N2"));
        assert!(names.contains(&"K1"));
        assert!(names.contains(&"O1"));
        assert!(names.contains(&"K2"));
        assert!(names.contains(&"P1"));
        assert!(names.contains(&"Q1"));
    }

    #[test]
    fn test_m2_frequency() {
        let sc = standard_constituents();
        let m2_freq = sc.iter().find(|(n, _)| n == "M2").unwrap().1;
        // M2 period is approximately 12.4206 hours
        let expected_period = 12.4206012;
        let computed_period = 1.0 / m2_freq;
        assert!(
            (computed_period - expected_period).abs() < 0.001,
            "M2 period: expected ~{}, got {}",
            expected_period,
            computed_period
        );
    }

    #[test]
    fn test_s2_frequency() {
        let sc = standard_constituents();
        let s2_freq = sc.iter().find(|(n, _)| n == "S2").unwrap().1;
        // S2 period is exactly 12.0 hours
        let expected = 1.0 / 12.0;
        assert!(
            (s2_freq - expected).abs() < TOLERANCE,
            "S2 freq: expected {}, got {}",
            expected,
            s2_freq
        );
    }

    #[test]
    fn test_k1_frequency() {
        let sc = standard_constituents();
        let k1_freq = sc.iter().find(|(n, _)| n == "K1").unwrap().1;
        // K1 period ~23.934 hours => freq ~0.04178 cph
        assert!(k1_freq > 0.0417 && k1_freq < 0.0419);
    }

    #[test]
    fn test_frequencies_are_positive() {
        let sc = standard_constituents();
        for (name, freq) in &sc {
            assert!(
                *freq > 0.0,
                "Constituent {} has non-positive frequency {}",
                name,
                freq
            );
        }
    }

    #[test]
    fn test_semidiurnal_faster_than_diurnal() {
        let sc = standard_constituents();
        let m2_freq = sc.iter().find(|(n, _)| n == "M2").unwrap().1;
        let k1_freq = sc.iter().find(|(n, _)| n == "K1").unwrap().1;
        assert!(
            m2_freq > k1_freq,
            "Semidiurnal M2 should have higher frequency than diurnal K1"
        );
    }

    // --- Rayleigh criterion tests ---

    #[test]
    fn test_rayleigh_criterion_resolvable() {
        // M2 and S2: delta_f ~ 0.00282 cph
        // Need T >= 1/0.00282 ~ 354.6 hours (~14.8 days)
        let sc = standard_constituents();
        let m2_freq = sc.iter().find(|(n, _)| n == "M2").unwrap().1;
        let s2_freq = sc.iter().find(|(n, _)| n == "S2").unwrap().1;

        // 30-day record should resolve M2 and S2
        assert!(rayleigh_criterion(m2_freq, s2_freq, 30.0 * 24.0));
    }

    #[test]
    fn test_rayleigh_criterion_not_resolvable() {
        let sc = standard_constituents();
        let m2_freq = sc.iter().find(|(n, _)| n == "M2").unwrap().1;
        let s2_freq = sc.iter().find(|(n, _)| n == "S2").unwrap().1;

        // 5-day record: too short to resolve M2 and S2
        assert!(!rayleigh_criterion(m2_freq, s2_freq, 5.0 * 24.0));
    }

    #[test]
    fn test_rayleigh_criterion_same_frequency() {
        assert!(!rayleigh_criterion(0.0805, 0.0805, 720.0));
    }

    #[test]
    fn test_rayleigh_criterion_widely_separated() {
        // M2 (~0.0805) and K1 (~0.0418): delta_f ~ 0.0387
        // Even a 2-day record (48 h) resolves them: 1/48 = 0.0208 < 0.0387
        assert!(rayleigh_criterion(0.0805, 0.0418, 48.0));
    }

    // --- Synthetic tide generation tests ---

    #[test]
    fn test_generate_synthetic_tide_length() {
        let data = generate_synthetic_tide(&[], 2.0, 1.0, 100);
        assert_eq!(data.len(), 100);
    }

    #[test]
    fn test_generate_synthetic_tide_mean_only() {
        let data = generate_synthetic_tide(&[], 3.5, 1.0, 50);
        for &v in &data {
            assert!((v - 3.5).abs() < TOLERANCE, "Expected 3.5, got {}", v);
        }
    }

    #[test]
    fn test_generate_synthetic_tide_single_cosine() {
        // Single constituent: freq=1/24 cph (24-hr period), amp=1.0, phase=0
        let freq = 1.0 / 24.0;
        let data = generate_synthetic_tide(&[(freq, 1.0, 0.0)], 0.0, 1.0, 25);
        // At t=0: cos(0) = 1.0
        assert!((data[0] - 1.0).abs() < TOLERANCE);
        // At t=6 (quarter period): cos(pi/2) ~ 0
        assert!(data[6].abs() < 0.02);
        // At t=12 (half period): cos(pi) = -1.0
        assert!((data[12] + 1.0).abs() < TOLERANCE);
    }

    // --- Least-squares harmonic analysis tests ---

    #[test]
    fn test_least_squares_single_frequency() {
        // Generate a clean cosine and recover amplitude + phase
        let freq = 0.08; // cph
        let amp = 2.5;
        let phase_deg = 45.0;
        let dt = 0.5;
        let n = 1000;

        let data = generate_synthetic_tide(&[(freq, amp, phase_deg)], 0.0, dt, n);
        let result = least_squares_harmonic(&data, &[freq], dt);

        assert_eq!(result.len(), 1);
        let (recovered_amp, recovered_phase_rad) = result[0];
        let recovered_phase_deg = recovered_phase_rad.to_degrees();

        assert!(
            (recovered_amp - amp).abs() < 0.01,
            "Amplitude: expected {}, got {}",
            amp,
            recovered_amp
        );

        // Normalize phases for comparison
        let mut pd = recovered_phase_deg;
        while pd < 0.0 {
            pd += 360.0;
        }
        assert!(
            (pd - phase_deg).abs() < 0.5,
            "Phase: expected {}deg, got {}deg",
            phase_deg,
            pd
        );
    }

    #[test]
    fn test_least_squares_two_frequencies() {
        let f1 = 0.0805114; // M2
        let f2 = 0.0417807; // K1
        let a1 = 1.5;
        let a2 = 0.8;
        let p1 = 30.0;
        let p2 = 120.0;
        let dt = 0.5;
        let n = 2000;

        let data = generate_synthetic_tide(&[(f1, a1, p1), (f2, a2, p2)], 0.0, dt, n);
        let result = least_squares_harmonic(&data, &[f1, f2], dt);

        assert_eq!(result.len(), 2);

        // Check first frequency
        assert!(
            (result[0].0 - a1).abs() < 0.02,
            "f1 amplitude: expected {}, got {}",
            a1,
            result[0].0
        );

        // Check second frequency
        assert!(
            (result[1].0 - a2).abs() < 0.02,
            "f2 amplitude: expected {}, got {}",
            a2,
            result[1].0
        );
    }

    // --- TidalAnalyzer roundtrip tests ---

    #[test]
    fn test_analyzer_roundtrip_single_constituent() {
        let m2_freq = 1.0 / 12.4206012;
        let amp = 1.0;
        let phase_deg = 0.0;
        let mean = 2.0;
        let dt = 1.0;
        let n = 720; // 30 days

        let data = generate_synthetic_tide(&[(m2_freq, amp, phase_deg)], mean, dt, n);
        let config = TidalConfig {
            sample_interval_hours: dt,
            analysis_period_days: 30.0,
            num_constituents: 1,
        };
        let analyzer = TidalAnalyzer::new(config);
        let result = analyzer.analyze(&data);

        assert!((result.mean_level - mean).abs() < 0.01, "Mean level mismatch");
        assert!(!result.constituents.is_empty());
        let m2 = &result.constituents[0];
        assert!(
            (m2.amplitude - amp).abs() < 0.05,
            "M2 amp: expected {}, got {}",
            amp,
            m2.amplitude
        );
    }

    #[test]
    fn test_analyzer_roundtrip_multiple_constituents() {
        let m2_freq = 1.0 / 12.4206012;
        let k1_freq = 1.0 / 23.9344696;
        let data = generate_synthetic_tide(
            &[(m2_freq, 1.0, 0.0), (k1_freq, 0.5, 90.0)],
            3.0,
            1.0,
            720,
        );

        let config = TidalConfig {
            sample_interval_hours: 1.0,
            analysis_period_days: 30.0,
            num_constituents: 8,
        };
        let analyzer = TidalAnalyzer::new(config);
        let result = analyzer.analyze(&data);

        assert!((result.mean_level - 3.0).abs() < 0.01);
        assert!(result.residual_rms < 0.1, "Residual RMS too large: {}", result.residual_rms);

        // Find M2 and K1 in results
        let m2 = result.constituents.iter().find(|c| c.name == "M2");
        let k1 = result.constituents.iter().find(|c| c.name == "K1");
        assert!(m2.is_some(), "M2 not found in results");
        assert!(k1.is_some(), "K1 not found in results");

        let m2 = m2.unwrap();
        let k1 = k1.unwrap();
        assert!(
            (m2.amplitude - 1.0).abs() < 0.1,
            "M2 amplitude mismatch: {}",
            m2.amplitude
        );
        assert!(
            (k1.amplitude - 0.5).abs() < 0.1,
            "K1 amplitude mismatch: {}",
            k1.amplitude
        );
    }

    // --- Prediction tests ---

    #[test]
    fn test_predict_tide_empty_constituents() {
        let hours = vec![0.0, 1.0, 2.0];
        let result = predict_tide(&[], 5.0, &hours);
        assert_eq!(result.len(), 3);
        for &v in &result {
            assert!((v - 5.0).abs() < TOLERANCE);
        }
    }

    #[test]
    fn test_predict_tide_single_constituent() {
        let c = TidalConstituent {
            name: "M2".to_string(),
            frequency_cph: 1.0 / 12.0, // 12-hour period for simplicity
            amplitude: 1.0,
            phase_deg: 0.0,
            snr: 100.0,
        };
        let result = predict_tide(&[c], 0.0, &[0.0, 3.0, 6.0]);
        // t=0: cos(0) = 1.0
        assert!((result[0] - 1.0).abs() < TOLERANCE);
        // t=3: cos(2*pi*(1/12)*3) = cos(pi/2) = 0
        assert!(result[1].abs() < TOLERANCE);
        // t=6: cos(pi) = -1.0
        assert!((result[2] + 1.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_predict_tide_roundtrip_consistency() {
        // Generate synthetic data, analyze, then predict and compare
        let m2_freq = 1.0 / 12.4206012;
        let amp = 1.5;
        let phase = 30.0;
        let mean = 2.5;
        let dt = 0.5;
        let n = 1440; // 30 days at half-hourly

        let data = generate_synthetic_tide(&[(m2_freq, amp, phase)], mean, dt, n);

        let config = TidalConfig {
            sample_interval_hours: dt,
            analysis_period_days: 30.0,
            num_constituents: 4,
        };
        let analyzer = TidalAnalyzer::new(config);
        let result = analyzer.analyze(&data);

        // Predict at the same times as the original data
        let hours: Vec<f64> = (0..n).map(|j| j as f64 * dt).collect();
        let predicted = predict_tide(&result.constituents, result.mean_level, &hours);

        // Compare original and predicted
        let max_err = data
            .iter()
            .zip(predicted.iter())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0f64, f64::max);

        assert!(
            max_err < 0.15,
            "Max prediction error too large: {}",
            max_err
        );
    }

    // --- Tidal form number tests ---

    #[test]
    fn test_form_number_semidiurnal() {
        // F < 0.25: semidiurnal (e.g., North Atlantic)
        let f = tidal_form_number(0.1, 0.08, 1.0, 0.5);
        assert!(f < 0.25, "Expected semidiurnal, got F={}", f);
    }

    #[test]
    fn test_form_number_mixed_semidiurnal() {
        // 0.25 <= F < 1.5: mixed, mainly semidiurnal
        let f = tidal_form_number(0.5, 0.4, 1.0, 0.5);
        assert!(
            f >= 0.25 && f < 1.5,
            "Expected mixed semidiurnal, got F={}",
            f
        );
    }

    #[test]
    fn test_form_number_diurnal() {
        // F >= 3.0: diurnal (e.g., Gulf of Mexico)
        let f = tidal_form_number(1.0, 0.8, 0.2, 0.1);
        assert!(f >= 3.0, "Expected diurnal, got F={}", f);
    }

    #[test]
    fn test_form_number_zero_denominator() {
        let f = tidal_form_number(1.0, 0.5, 0.0, 0.0);
        assert!(f.is_infinite());
    }

    #[test]
    fn test_classify_tide_types() {
        assert_eq!(classify_tide(0.1), "Semidiurnal");
        assert_eq!(classify_tide(0.5), "Mixed, mainly semidiurnal");
        assert_eq!(classify_tide(2.0), "Mixed, mainly diurnal");
        assert_eq!(classify_tide(5.0), "Diurnal");
    }

    // --- Nodal correction tests ---

    #[test]
    fn test_nodal_corrections_s2_no_correction() {
        let (f, u) = nodal_corrections(2020.0, "S2");
        assert!((f - 1.0).abs() < TOLERANCE);
        assert!(u.abs() < TOLERANCE);
    }

    #[test]
    fn test_nodal_corrections_p1_no_correction() {
        let (f, u) = nodal_corrections(2025.0, "P1");
        assert!((f - 1.0).abs() < TOLERANCE);
        assert!(u.abs() < TOLERANCE);
    }

    #[test]
    fn test_nodal_corrections_m2_range() {
        // f for M2 should be approximately 1.0 +/- 0.037
        for year in [2000.0, 2005.0, 2010.0, 2015.0, 2020.0, 2025.0] {
            let (f, u) = nodal_corrections(year, "M2");
            assert!(
                f > 0.96 && f < 1.04,
                "M2 f={} out of range for year {}",
                f,
                year
            );
            assert!(
                u.abs() < 2.2,
                "M2 u={} out of range for year {}",
                u,
                year
            );
        }
    }

    #[test]
    fn test_nodal_corrections_k1_range() {
        // f for K1 should be approximately 1.006 +/- 0.115
        for year in [2000.0, 2010.0, 2020.0] {
            let (f, _u) = nodal_corrections(year, "K1");
            assert!(
                f > 0.89 && f < 1.13,
                "K1 f={} out of range for year {}",
                f,
                year
            );
        }
    }

    #[test]
    fn test_nodal_corrections_unknown_constituent() {
        let (f, u) = nodal_corrections(2020.0, "UNKNOWN");
        assert!((f - 1.0).abs() < TOLERANCE);
        assert!(u.abs() < TOLERANCE);
    }

    #[test]
    fn test_nodal_corrections_n2_equals_m2() {
        // N2 uses same nodal formulas as M2
        let (f_m2, u_m2) = nodal_corrections(2020.0, "M2");
        let (f_n2, u_n2) = nodal_corrections(2020.0, "N2");
        assert!((f_m2 - f_n2).abs() < TOLERANCE);
        assert!((u_m2 - u_n2).abs() < TOLERANCE);
    }

    #[test]
    fn test_nodal_corrections_q1_equals_o1() {
        // Q1 uses same nodal formulas as O1
        let (f_o1, u_o1) = nodal_corrections(2020.0, "O1");
        let (f_q1, u_q1) = nodal_corrections(2020.0, "Q1");
        assert!((f_o1 - f_q1).abs() < TOLERANCE);
        assert!((u_o1 - u_q1).abs() < TOLERANCE);
    }

    // --- Edge case tests ---

    #[test]
    #[should_panic(expected = "water_levels must not be empty")]
    fn test_analyzer_empty_data_panics() {
        let config = TidalConfig::default();
        let analyzer = TidalAnalyzer::new(config);
        analyzer.analyze(&[]);
    }

    #[test]
    fn test_analyzer_constant_signal() {
        let data = vec![5.0; 100];
        let config = TidalConfig {
            sample_interval_hours: 1.0,
            analysis_period_days: 4.0,
            num_constituents: 2,
        };
        let analyzer = TidalAnalyzer::new(config);
        let result = analyzer.analyze(&data);

        assert!((result.mean_level - 5.0).abs() < TOLERANCE);
        // All constituent amplitudes should be ~0
        for c in &result.constituents {
            assert!(
                c.amplitude < 0.01,
                "Constituent {} should have near-zero amplitude, got {}",
                c.name,
                c.amplitude
            );
        }
    }

    #[test]
    fn test_analyzer_single_sample() {
        let data = vec![3.14];
        let config = TidalConfig {
            sample_interval_hours: 1.0,
            analysis_period_days: 1.0,
            num_constituents: 1,
        };
        let analyzer = TidalAnalyzer::new(config);
        let result = analyzer.analyze(&data);
        assert!((result.mean_level - 3.14).abs() < TOLERANCE);
    }

    #[test]
    fn test_default_config() {
        let config = TidalConfig::default();
        assert!((config.sample_interval_hours - 1.0).abs() < TOLERANCE);
        assert!((config.analysis_period_days - 29.53).abs() < 0.01);
        assert_eq!(config.num_constituents, 8);
    }

    #[test]
    fn test_constituent_snr_positive() {
        let m2_freq = 1.0 / 12.4206012;
        let data = generate_synthetic_tide(&[(m2_freq, 1.0, 0.0)], 2.0, 1.0, 720);

        let config = TidalConfig {
            sample_interval_hours: 1.0,
            analysis_period_days: 30.0,
            num_constituents: 4,
        };
        let analyzer = TidalAnalyzer::new(config);
        let result = analyzer.analyze(&data);

        for c in &result.constituents {
            assert!(c.snr >= 0.0, "SNR must be non-negative for {}", c.name);
        }
    }

    #[test]
    fn test_constituents_sorted_by_amplitude() {
        let m2_freq = 1.0 / 12.4206012;
        let k1_freq = 1.0 / 23.9344696;
        let data = generate_synthetic_tide(
            &[(m2_freq, 2.0, 0.0), (k1_freq, 0.5, 45.0)],
            1.0,
            1.0,
            720,
        );

        let config = TidalConfig {
            sample_interval_hours: 1.0,
            analysis_period_days: 30.0,
            num_constituents: 8,
        };
        let analyzer = TidalAnalyzer::new(config);
        let result = analyzer.analyze(&data);

        for i in 1..result.constituents.len() {
            assert!(
                result.constituents[i - 1].amplitude >= result.constituents[i].amplitude,
                "Constituents not sorted by amplitude at index {}",
                i
            );
        }
    }

    #[test]
    fn test_normalize_angle_deg() {
        assert!((normalize_angle_deg(0.0) - 0.0).abs() < TOLERANCE);
        assert!((normalize_angle_deg(360.0) - 0.0).abs() < TOLERANCE);
        assert!((normalize_angle_deg(-90.0) - 270.0).abs() < TOLERANCE);
        assert!((normalize_angle_deg(450.0) - 90.0).abs() < TOLERANCE);
        assert!((normalize_angle_deg(-360.0) - 0.0).abs() < TOLERANCE);
    }
}
