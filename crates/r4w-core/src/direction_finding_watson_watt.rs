//! # Watson-Watt Direction Finding
//!
//! Implements the Watson-Watt direction finding algorithm for radio bearing
//! estimation using crossed-loop (or Adcock) antennas with an optional
//! omnidirectional sense antenna for 180-degree ambiguity resolution.
//!
//! ## Background
//!
//! The Watson-Watt technique (patented 1919, refined through the 1930s) uses
//! two orthogonal loop antennas — one aligned North-South (NS), the other
//! East-West (EW) — to measure the bearing of an incoming signal.  The ratio
//! of the voltages induced in the two loops gives the tangent of the bearing
//! angle.  A third, omnidirectional (sense) antenna resolves the inherent
//! 180-degree ambiguity of the loop-only measurement.
//!
//! ## Algorithm
//!
//! 1. Sample the NS-loop, EW-loop, and sense-antenna voltages.
//! 2. Compute the raw bearing: `θ = atan2(V_ew, V_ns)`.
//! 3. Use the sense-antenna phase to resolve the front/back ambiguity.
//! 4. Optionally apply a site-error correction table.
//!
//! ## Supported Modes
//!
//! | Mode                | Description                                        |
//! |---------------------|----------------------------------------------------|
//! | Amplitude comparison| Uses voltage magnitudes on the loops                |
//! | Phase comparison    | Uses phase difference between loops and sense       |
//! | Multi-sample        | Averages many snapshots for improved accuracy       |
//! | Adcock array        | Uses spaced vertical dipoles instead of loops       |
//!
//! ## Example
//!
//! ```
//! use r4w_core::direction_finding_watson_watt::{WatsonWattDf, AntennaType};
//!
//! let df = WatsonWattDf::new(AntennaType::CrossedLoop, 100.0e6);
//!
//! // Simulate a signal arriving from 45 degrees (NE)
//! let bearing_rad = 45.0_f64.to_radians();
//! let ns = bearing_rad.cos();
//! let ew = bearing_rad.sin();
//! let sense = 1.0; // positive => front hemisphere
//!
//! let result = df.estimate_bearing_amplitude(ns, ew, sense);
//! let bearing_deg = result.bearing_deg;
//! assert!((bearing_deg - 45.0).abs() < 1.0);
//! ```

use std::f64::consts::PI;

// ── Complex-number helpers using (f64, f64) tuples ───────────────────────

/// Magnitude of a complex number represented as `(re, im)`.
#[inline]
fn c_mag(c: (f64, f64)) -> f64 {
    (c.0 * c.0 + c.1 * c.1).sqrt()
}

/// Phase (argument) of a complex number in radians.
#[inline]
fn c_phase(c: (f64, f64)) -> f64 {
    c.1.atan2(c.0)
}

/// Multiply two complex numbers.
#[inline]
fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Conjugate of a complex number.
#[inline]
fn c_conj(c: (f64, f64)) -> (f64, f64) {
    (c.0, -c.1)
}

/// Add two complex numbers.
#[inline]
fn c_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Scale a complex number by a real scalar.
#[inline]
fn c_scale(c: (f64, f64), s: f64) -> (f64, f64) {
    (c.0 * s, c.1 * s)
}

// ── Public types ─────────────────────────────────────────────────────────

/// Type of antenna array used for direction finding.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AntennaType {
    /// Traditional crossed-loop antennas (two orthogonal loops).
    CrossedLoop,
    /// Adcock array: four vertical dipoles arranged in a square,
    /// differentially combined to form NS and EW pairs.
    Adcock {
        /// Spacing between dipole pairs in metres.
        spacing_m: f64,
    },
}

/// Result of a single bearing estimation.
#[derive(Debug, Clone, Copy)]
pub struct BearingResult {
    /// Estimated bearing in degrees from true north, clockwise \[0, 360).
    pub bearing_deg: f64,
    /// Estimated bearing in radians \[0, 2π).
    pub bearing_rad: f64,
    /// Whether the sense antenna was used to resolve ambiguity.
    pub ambiguity_resolved: bool,
    /// Estimated signal quality (0.0 .. 1.0); higher is better.
    pub quality: f64,
}

/// Statistics from multi-sample or circular-statistics operations.
#[derive(Debug, Clone, Copy)]
pub struct BearingStatistics {
    /// Mean bearing in degrees \[0, 360).
    pub mean_bearing_deg: f64,
    /// Circular standard deviation in degrees.
    pub circular_std_deg: f64,
    /// Number of samples used.
    pub num_samples: usize,
    /// RMS bearing error in degrees (if a reference is available).
    pub rms_error_deg: Option<f64>,
}

/// Entry in a site-error correction lookup table.
#[derive(Debug, Clone, Copy)]
pub struct SiteErrorEntry {
    /// Nominal bearing in degrees at which the correction applies.
    pub nominal_deg: f64,
    /// Correction to *add* to the raw bearing, in degrees.
    pub correction_deg: f64,
}

/// Watson-Watt direction finding processor.
#[derive(Debug, Clone)]
pub struct WatsonWattDf {
    /// Antenna type.
    antenna_type: AntennaType,
    /// Operating frequency in Hz.
    frequency_hz: f64,
    /// Site-error correction table (sorted by `nominal_deg`).
    site_error_table: Vec<SiteErrorEntry>,
    /// NS-loop amplitude calibration factor (multiplicative).
    cal_ns: f64,
    /// EW-loop amplitude calibration factor (multiplicative).
    cal_ew: f64,
}

impl WatsonWattDf {
    // ── Construction ─────────────────────────────────────────────────

    /// Create a new Watson-Watt DF processor.
    ///
    /// * `antenna_type` – the physical antenna arrangement.
    /// * `frequency_hz` – the centre frequency of the signal of interest.
    pub fn new(antenna_type: AntennaType, frequency_hz: f64) -> Self {
        Self {
            antenna_type,
            frequency_hz,
            site_error_table: Vec::new(),
            cal_ns: 1.0,
            cal_ew: 1.0,
        }
    }

    /// Set amplitude calibration factors for the NS and EW loops.
    ///
    /// These multiplicative factors correct for gain imbalance between the
    /// two antenna channels.  A factor of `1.0` means no correction.
    pub fn set_calibration(&mut self, cal_ns: f64, cal_ew: f64) {
        self.cal_ns = cal_ns;
        self.cal_ew = cal_ew;
    }

    /// Load a site-error correction table.
    ///
    /// The table is a set of `(nominal_bearing_deg, correction_deg)` pairs.
    /// Between table entries the correction is linearly interpolated.
    pub fn set_site_error_table(&mut self, table: Vec<SiteErrorEntry>) {
        let mut t = table;
        t.sort_by(|a, b| a.nominal_deg.partial_cmp(&b.nominal_deg).unwrap());
        self.site_error_table = t;
    }

    /// Return the configured antenna type.
    pub fn antenna_type(&self) -> AntennaType {
        self.antenna_type
    }

    /// Return the operating frequency in Hz.
    pub fn frequency_hz(&self) -> f64 {
        self.frequency_hz
    }

    // ── Amplitude-comparison bearing ─────────────────────────────────

    /// Estimate bearing using amplitude comparison.
    ///
    /// * `v_ns` – voltage from the North-South loop (real-valued envelope).
    /// * `v_ew` – voltage from the East-West loop.
    /// * `v_sense` – voltage from the omnidirectional sense antenna.  A
    ///   positive value indicates the signal is in the forward hemisphere
    ///   (0..180 deg), negative for the rear (180..360 deg).
    pub fn estimate_bearing_amplitude(
        &self,
        v_ns: f64,
        v_ew: f64,
        v_sense: f64,
    ) -> BearingResult {
        let ns = v_ns * self.cal_ns;
        let ew = v_ew * self.cal_ew;

        // Apply Adcock spacing correction if needed.
        let (ns_corr, ew_corr) = self.apply_adcock_correction(ns, ew);

        // Raw bearing from atan2 — result is in (-π, π].
        let raw_rad = ew_corr.atan2(ns_corr);

        // Resolve 180° ambiguity using sense antenna.
        let (resolved_rad, resolved) = self.resolve_ambiguity(raw_rad, v_sense);

        // Site-error correction.
        let corrected_rad = self.apply_site_correction(resolved_rad);

        let bearing_deg = wrap_deg(corrected_rad.to_degrees());
        let bearing_rad = bearing_deg.to_radians();

        let quality = self.compute_quality(ns_corr, ew_corr, v_sense);

        BearingResult {
            bearing_deg,
            bearing_rad,
            ambiguity_resolved: resolved,
            quality,
        }
    }

    // ── Phase-comparison bearing ─────────────────────────────────────

    /// Estimate bearing using phase comparison.
    ///
    /// Each argument is a complex-valued (I/Q) sample from the respective
    /// antenna channel, represented as `(re, im)`.
    pub fn estimate_bearing_phase(
        &self,
        ns_iq: (f64, f64),
        ew_iq: (f64, f64),
        sense_iq: (f64, f64),
    ) -> BearingResult {
        // Compute phase of each channel relative to the sense antenna.
        let ns_rel = c_mul(ns_iq, c_conj(sense_iq));
        let ew_rel = c_mul(ew_iq, c_conj(sense_iq));

        let ns_phase = c_phase(ns_rel);
        let ew_phase = c_phase(ew_rel);

        // Apply calibration to the effective amplitudes.
        let ns_val = c_mag(ns_iq) * self.cal_ns * ns_phase.signum();
        let ew_val = c_mag(ew_iq) * self.cal_ew * ew_phase.signum();

        // Apply Adcock correction.
        let (ns_corr, ew_corr) = self.apply_adcock_correction(ns_val, ew_val);

        let raw_rad = ew_corr.atan2(ns_corr);

        // Use sense phase to resolve ambiguity: if the NS channel is
        // roughly in-phase with sense, signal is in the front hemisphere.
        let sense_indicator = ns_rel.0; // real part of relative phase
        let (resolved_rad, resolved) = self.resolve_ambiguity(raw_rad, sense_indicator);

        let corrected_rad = self.apply_site_correction(resolved_rad);
        let bearing_deg = wrap_deg(corrected_rad.to_degrees());
        let bearing_rad = bearing_deg.to_radians();

        let quality = self.compute_quality(ns_corr, ew_corr, c_mag(sense_iq));

        BearingResult {
            bearing_deg,
            bearing_rad,
            ambiguity_resolved: resolved,
            quality,
        }
    }

    // ── Multi-sample averaging ───────────────────────────────────────

    /// Estimate bearing by averaging multiple amplitude samples.
    ///
    /// Each element is `(v_ns, v_ew, v_sense)`.  Returns the circular mean
    /// bearing and associated statistics.
    pub fn estimate_bearing_multi_amplitude(
        &self,
        samples: &[(f64, f64, f64)],
    ) -> (BearingResult, BearingStatistics) {
        assert!(!samples.is_empty(), "need at least one sample");

        let bearings: Vec<BearingResult> = samples
            .iter()
            .map(|&(ns, ew, sense)| self.estimate_bearing_amplitude(ns, ew, sense))
            .collect();

        let angles_deg: Vec<f64> = bearings.iter().map(|b| b.bearing_deg).collect();
        let stats = circular_statistics(&angles_deg, None);

        let mean_rad = stats.mean_bearing_deg.to_radians();
        let avg_quality =
            bearings.iter().map(|b| b.quality).sum::<f64>() / bearings.len() as f64;

        let result = BearingResult {
            bearing_deg: stats.mean_bearing_deg,
            bearing_rad: mean_rad,
            ambiguity_resolved: bearings.iter().all(|b| b.ambiguity_resolved),
            quality: avg_quality,
        };

        (result, stats)
    }

    /// Estimate bearing by averaging multiple phase (IQ) samples.
    ///
    /// Each element is `(ns_iq, ew_iq, sense_iq)`.
    pub fn estimate_bearing_multi_phase(
        &self,
        samples: &[((f64, f64), (f64, f64), (f64, f64))],
    ) -> (BearingResult, BearingStatistics) {
        assert!(!samples.is_empty(), "need at least one sample");

        let bearings: Vec<BearingResult> = samples
            .iter()
            .map(|&(ns, ew, sense)| self.estimate_bearing_phase(ns, ew, sense))
            .collect();

        let angles_deg: Vec<f64> = bearings.iter().map(|b| b.bearing_deg).collect();
        let stats = circular_statistics(&angles_deg, None);

        let mean_rad = stats.mean_bearing_deg.to_radians();
        let avg_quality =
            bearings.iter().map(|b| b.quality).sum::<f64>() / bearings.len() as f64;

        let result = BearingResult {
            bearing_deg: stats.mean_bearing_deg,
            bearing_rad: mean_rad,
            ambiguity_resolved: bearings.iter().all(|b| b.ambiguity_resolved),
            quality: avg_quality,
        };

        (result, stats)
    }

    // ── Bearing accuracy / RMS error ─────────────────────────────────

    /// Compute the RMS bearing error of a set of estimates against a known
    /// true bearing.
    pub fn rms_bearing_error(estimates_deg: &[f64], true_bearing_deg: f64) -> f64 {
        if estimates_deg.is_empty() {
            return 0.0;
        }
        let sum_sq: f64 = estimates_deg
            .iter()
            .map(|&est| {
                let err = wrap_pm180(est - true_bearing_deg);
                err * err
            })
            .sum();
        (sum_sq / estimates_deg.len() as f64).sqrt()
    }

    // ── Private helpers ──────────────────────────────────────────────

    /// Apply Adcock spacing correction.
    ///
    /// For an Adcock array the effective loop area (and hence induced voltage)
    /// is proportional to `sin(π d / λ)` where `d` is the element spacing
    /// and `λ` the wavelength.  We normalise so that the bearing angle is
    /// not biased by the spacing factor.
    fn apply_adcock_correction(&self, ns: f64, ew: f64) -> (f64, f64) {
        match self.antenna_type {
            AntennaType::CrossedLoop => (ns, ew),
            AntennaType::Adcock { spacing_m } => {
                let lambda = 3.0e8 / self.frequency_hz;
                let correction = (PI * spacing_m / lambda).sin();
                if correction.abs() < 1e-12 {
                    (ns, ew) // degenerate — spacing is a multiple of λ
                } else {
                    (ns / correction, ew / correction)
                }
            }
        }
    }

    /// Resolve the 180-degree ambiguity using the sense antenna.
    ///
    /// The sense antenna indicates which hemisphere the signal is actually
    /// in: positive means front (0..180°), negative means back (180..360°).
    /// If the raw bearing from atan2 disagrees with the sense indication,
    /// we flip the bearing by 180°.
    ///
    /// Returns `(resolved_bearing_rad, was_resolved)`.
    fn resolve_ambiguity(&self, raw_rad: f64, sense_indicator: f64) -> (f64, bool) {
        if sense_indicator == 0.0 {
            // Cannot resolve — return raw bearing.
            return (raw_rad, false);
        }
        // Determine which hemisphere the raw bearing falls in.
        let raw_deg = wrap_deg(raw_rad.to_degrees());
        let raw_in_front = raw_deg < 180.0 || raw_deg >= 360.0 - 1e-9;
        let sense_says_front = sense_indicator > 0.0;

        // Flip by 180° if the raw bearing and sense disagree.
        if raw_in_front != sense_says_front {
            (raw_rad + PI, true)
        } else {
            (raw_rad, true)
        }
    }

    /// Apply site-error correction via linear interpolation of the table.
    fn apply_site_correction(&self, bearing_rad: f64) -> f64 {
        if self.site_error_table.is_empty() {
            return bearing_rad;
        }

        let bearing_deg = wrap_deg(bearing_rad.to_degrees());
        let correction = interpolate_site_error(&self.site_error_table, bearing_deg);
        (bearing_deg + correction).to_radians()
    }

    /// Heuristic quality metric ∈ [0, 1].
    fn compute_quality(&self, ns: f64, ew: f64, sense: f64) -> f64 {
        let loop_mag = (ns * ns + ew * ew).sqrt();
        if loop_mag < 1e-30 {
            return 0.0;
        }
        // Quality is high when the sense antenna has a strong signal relative
        // to the loops, and when the loop signal is well above noise.
        let sense_ratio = sense.abs() / loop_mag;
        // Sigmoid-like mapping so quality saturates near 1.0.
        let q = 1.0 - (-sense_ratio).exp();
        q.clamp(0.0, 1.0)
    }
}

// ── Circular statistics ──────────────────────────────────────────────────

/// Compute circular statistics for a set of bearing angles in degrees.
///
/// If `true_bearing_deg` is provided the RMS error is also computed.
pub fn circular_statistics(
    angles_deg: &[f64],
    true_bearing_deg: Option<f64>,
) -> BearingStatistics {
    let n = angles_deg.len();
    if n == 0 {
        return BearingStatistics {
            mean_bearing_deg: 0.0,
            circular_std_deg: 0.0,
            num_samples: 0,
            rms_error_deg: None,
        };
    }

    // Compute the mean resultant vector.
    let (mut sum_sin, mut sum_cos) = (0.0_f64, 0.0_f64);
    for &a in angles_deg {
        let r = a.to_radians();
        sum_sin += r.sin();
        sum_cos += r.cos();
    }
    let mean_sin = sum_sin / n as f64;
    let mean_cos = sum_cos / n as f64;

    // Mean direction.
    let mean_rad = mean_sin.atan2(mean_cos);
    let mean_deg = wrap_deg(mean_rad.to_degrees());

    // Mean resultant length R̄.
    let r_bar = (mean_sin * mean_sin + mean_cos * mean_cos).sqrt();

    // Circular standard deviation (Mardia & Jupp, 2000).
    // σ_circ = sqrt(-2 ln R̄)  — in radians.
    let circ_std_rad = if r_bar > 1e-15 {
        (-2.0 * r_bar.ln()).sqrt()
    } else {
        PI // maximally spread
    };
    let circ_std_deg = circ_std_rad.to_degrees();

    let rms = true_bearing_deg
        .map(|tb| WatsonWattDf::rms_bearing_error(angles_deg, tb));

    BearingStatistics {
        mean_bearing_deg: mean_deg,
        circular_std_deg: circ_std_deg,
        num_samples: n,
        rms_error_deg: rms,
    }
}

// ── Site-error interpolation ─────────────────────────────────────────────

/// Linearly interpolate a site-error correction table.
///
/// The table must be sorted by `nominal_deg`.  Bearings that wrap around
/// 360° are handled by interpolating between the last and first entries.
fn interpolate_site_error(table: &[SiteErrorEntry], bearing_deg: f64) -> f64 {
    debug_assert!(!table.is_empty());

    let b = wrap_deg(bearing_deg);

    // Find the two surrounding entries.
    let idx = table.partition_point(|e| e.nominal_deg <= b);

    if idx == 0 {
        // Before the first entry — wrap around.
        let lo = &table[table.len() - 1];
        let hi = &table[0];
        let span = wrap_deg(hi.nominal_deg - lo.nominal_deg + 360.0);
        if span.abs() < 1e-12 {
            return lo.correction_deg;
        }
        let frac = wrap_deg(b - lo.nominal_deg + 360.0) / span;
        lo.correction_deg + frac * (hi.correction_deg - lo.correction_deg)
    } else if idx >= table.len() {
        // After the last entry — wrap around.
        let lo = &table[table.len() - 1];
        let hi = &table[0];
        let span = wrap_deg(hi.nominal_deg - lo.nominal_deg + 360.0);
        if span.abs() < 1e-12 {
            return lo.correction_deg;
        }
        let frac = wrap_deg(b - lo.nominal_deg) / span;
        lo.correction_deg + frac * (hi.correction_deg - lo.correction_deg)
    } else {
        let lo = &table[idx - 1];
        let hi = &table[idx];
        let span = hi.nominal_deg - lo.nominal_deg;
        if span.abs() < 1e-12 {
            return lo.correction_deg;
        }
        let frac = (b - lo.nominal_deg) / span;
        lo.correction_deg + frac * (hi.correction_deg - lo.correction_deg)
    }
}

// ── Utility functions ────────────────────────────────────────────────────

/// Wrap an angle in degrees to the range \[0, 360).
fn wrap_deg(deg: f64) -> f64 {
    let mut d = deg % 360.0;
    if d < 0.0 {
        d += 360.0;
    }
    d
}

/// Wrap an angle in degrees to (-180, 180\].
fn wrap_pm180(deg: f64) -> f64 {
    let mut d = deg % 360.0;
    if d > 180.0 {
        d -= 360.0;
    } else if d <= -180.0 {
        d += 360.0;
    }
    d
}

// ── Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1.0; // 1-degree tolerance for bearing tests

    /// Helper: generate amplitude samples for a signal at the given true
    /// bearing (degrees) with optional noise.
    fn amp_sample(bearing_deg: f64) -> (f64, f64, f64) {
        let rad = bearing_deg.to_radians();
        let ns = rad.cos();
        let ew = rad.sin();
        let sense = 1.0; // front hemisphere
        (ns, ew, sense)
    }

    /// Helper: generate IQ samples for a signal at the given true bearing.
    fn iq_sample(bearing_deg: f64) -> ((f64, f64), (f64, f64), (f64, f64)) {
        let rad = bearing_deg.to_radians();
        let ns_mag = rad.cos().abs();
        let ew_mag = rad.sin().abs();

        // Phase: 0 when cos/sin >= 0, π when < 0.
        let ns_phase = if rad.cos() >= 0.0 { 0.0 } else { PI };
        let ew_phase = if rad.sin() >= 0.0 { 0.0 } else { PI };

        let ns_iq = (ns_mag * ns_phase.cos(), ns_mag * ns_phase.sin());
        let ew_iq = (ew_mag * ew_phase.cos(), ew_mag * ew_phase.sin());
        let sense_iq = (1.0, 0.0);

        (ns_iq, ew_iq, sense_iq)
    }

    // ── Basic amplitude-comparison tests ─────────────────────────────

    #[test]
    fn test_amplitude_bearing_north() {
        let df = WatsonWattDf::new(AntennaType::CrossedLoop, 100.0e6);
        let result = df.estimate_bearing_amplitude(1.0, 0.0, 1.0);
        assert!(
            result.bearing_deg.abs() < TOL || (360.0 - result.bearing_deg) < TOL,
            "expected ~0°, got {:.2}°",
            result.bearing_deg
        );
    }

    #[test]
    fn test_amplitude_bearing_east() {
        let df = WatsonWattDf::new(AntennaType::CrossedLoop, 100.0e6);
        let result = df.estimate_bearing_amplitude(0.0, 1.0, 1.0);
        assert!(
            (result.bearing_deg - 90.0).abs() < TOL,
            "expected ~90°, got {:.2}°",
            result.bearing_deg
        );
    }

    #[test]
    fn test_amplitude_bearing_south() {
        let df = WatsonWattDf::new(AntennaType::CrossedLoop, 100.0e6);
        // Negative NS, zero EW, negative sense → 180°.
        let result = df.estimate_bearing_amplitude(-1.0, 0.0, -1.0);
        assert!(
            (result.bearing_deg - 180.0).abs() < TOL,
            "expected ~180°, got {:.2}°",
            result.bearing_deg
        );
    }

    #[test]
    fn test_amplitude_bearing_west() {
        let df = WatsonWattDf::new(AntennaType::CrossedLoop, 100.0e6);
        let result = df.estimate_bearing_amplitude(0.0, -1.0, -1.0);
        assert!(
            (result.bearing_deg - 270.0).abs() < TOL,
            "expected ~270°, got {:.2}°",
            result.bearing_deg
        );
    }

    #[test]
    fn test_amplitude_bearing_northeast() {
        let df = WatsonWattDf::new(AntennaType::CrossedLoop, 100.0e6);
        let (ns, ew, sense) = amp_sample(45.0);
        let result = df.estimate_bearing_amplitude(ns, ew, sense);
        assert!(
            (result.bearing_deg - 45.0).abs() < TOL,
            "expected ~45°, got {:.2}°",
            result.bearing_deg
        );
    }

    #[test]
    fn test_amplitude_bearing_southwest() {
        let df = WatsonWattDf::new(AntennaType::CrossedLoop, 100.0e6);
        let rad = 225.0_f64.to_radians();
        let ns = rad.cos();
        let ew = rad.sin();
        // sense negative for back hemisphere
        let result = df.estimate_bearing_amplitude(ns, ew, -1.0);
        assert!(
            (result.bearing_deg - 225.0).abs() < TOL,
            "expected ~225°, got {:.2}°",
            result.bearing_deg
        );
    }

    // ── Phase-comparison tests ───────────────────────────────────────

    #[test]
    fn test_phase_bearing_north() {
        let df = WatsonWattDf::new(AntennaType::CrossedLoop, 100.0e6);
        let (ns, ew, sense) = iq_sample(0.0);
        let result = df.estimate_bearing_phase(ns, ew, sense);
        // Near 0 or 360.
        let err = (result.bearing_deg).min(360.0 - result.bearing_deg);
        assert!(err < 2.0, "expected ~0°, got {:.2}°", result.bearing_deg);
    }

    #[test]
    fn test_phase_bearing_east() {
        let df = WatsonWattDf::new(AntennaType::CrossedLoop, 100.0e6);
        let (ns, ew, sense) = iq_sample(90.0);
        let result = df.estimate_bearing_phase(ns, ew, sense);
        assert!(
            (result.bearing_deg - 90.0).abs() < 2.0,
            "expected ~90°, got {:.2}°",
            result.bearing_deg
        );
    }

    // ── Ambiguity resolution ─────────────────────────────────────────

    #[test]
    fn test_ambiguity_no_sense() {
        let df = WatsonWattDf::new(AntennaType::CrossedLoop, 100.0e6);
        // sense = 0 means we cannot resolve ambiguity.
        let result = df.estimate_bearing_amplitude(1.0, 0.0, 0.0);
        assert!(!result.ambiguity_resolved);
    }

    #[test]
    fn test_ambiguity_resolved_front() {
        let df = WatsonWattDf::new(AntennaType::CrossedLoop, 100.0e6);
        let result = df.estimate_bearing_amplitude(1.0, 0.0, 1.0);
        assert!(result.ambiguity_resolved);
        // Should be in the front hemisphere (near 0°).
        let err = result.bearing_deg.min(360.0 - result.bearing_deg);
        assert!(err < TOL);
    }

    #[test]
    fn test_ambiguity_resolved_back() {
        let df = WatsonWattDf::new(AntennaType::CrossedLoop, 100.0e6);
        // Same loop voltages as north, but sense negative → 180°.
        let result = df.estimate_bearing_amplitude(1.0, 0.0, -1.0);
        assert!(result.ambiguity_resolved);
        assert!(
            (result.bearing_deg - 180.0).abs() < TOL,
            "expected ~180°, got {:.2}°",
            result.bearing_deg
        );
    }

    // ── Multi-sample averaging ───────────────────────────────────────

    #[test]
    fn test_multi_sample_amplitude() {
        let df = WatsonWattDf::new(AntennaType::CrossedLoop, 100.0e6);
        let samples: Vec<(f64, f64, f64)> = (0..20)
            .map(|_| amp_sample(90.0))
            .collect();
        let (result, stats) = df.estimate_bearing_multi_amplitude(&samples);
        assert!(
            (result.bearing_deg - 90.0).abs() < TOL,
            "expected ~90°, got {:.2}°",
            result.bearing_deg
        );
        assert_eq!(stats.num_samples, 20);
        assert!(stats.circular_std_deg < 1.0, "std too high: {:.2}", stats.circular_std_deg);
    }

    #[test]
    fn test_multi_sample_phase() {
        let df = WatsonWattDf::new(AntennaType::CrossedLoop, 100.0e6);
        let samples: Vec<_> = (0..10)
            .map(|_| iq_sample(45.0))
            .collect();
        let (result, stats) = df.estimate_bearing_multi_phase(&samples);
        assert!(
            (result.bearing_deg - 45.0).abs() < 2.0,
            "expected ~45°, got {:.2}°",
            result.bearing_deg
        );
        assert_eq!(stats.num_samples, 10);
    }

    // ── Circular statistics ──────────────────────────────────────────

    #[test]
    fn test_circular_mean_simple() {
        let angles = vec![10.0, 20.0, 30.0];
        let stats = circular_statistics(&angles, None);
        assert!(
            (stats.mean_bearing_deg - 20.0).abs() < 0.5,
            "expected ~20°, got {:.2}°",
            stats.mean_bearing_deg
        );
    }

    #[test]
    fn test_circular_mean_wraparound() {
        // 350° and 10° should average to ~0° (or 360°).
        let angles = vec![350.0, 10.0];
        let stats = circular_statistics(&angles, None);
        let err = stats.mean_bearing_deg.min(360.0 - stats.mean_bearing_deg);
        assert!(
            err < 1.0,
            "expected near 0/360°, got {:.2}°",
            stats.mean_bearing_deg
        );
    }

    #[test]
    fn test_circular_std_identical() {
        let angles = vec![90.0; 50];
        let stats = circular_statistics(&angles, None);
        assert!(
            stats.circular_std_deg < 0.01,
            "identical angles should have ~0 std, got {:.4}°",
            stats.circular_std_deg
        );
    }

    // ── RMS error ────────────────────────────────────────────────────

    #[test]
    fn test_rms_error_perfect() {
        let estimates = vec![90.0; 10];
        let rms = WatsonWattDf::rms_bearing_error(&estimates, 90.0);
        assert!(rms < 1e-10, "expected ~0 rms, got {:.6}", rms);
    }

    #[test]
    fn test_rms_error_known() {
        // Two estimates: 89° and 91° vs true 90° → error = ±1° → RMS = 1°.
        let estimates = vec![89.0, 91.0];
        let rms = WatsonWattDf::rms_bearing_error(&estimates, 90.0);
        assert!(
            (rms - 1.0).abs() < 0.01,
            "expected ~1.0° rms, got {:.4}°",
            rms
        );
    }

    // ── Site-error correction ────────────────────────────────────────

    #[test]
    fn test_site_error_correction() {
        let mut df = WatsonWattDf::new(AntennaType::CrossedLoop, 100.0e6);
        df.set_site_error_table(vec![
            SiteErrorEntry { nominal_deg: 0.0, correction_deg: 2.0 },
            SiteErrorEntry { nominal_deg: 90.0, correction_deg: -1.0 },
            SiteErrorEntry { nominal_deg: 180.0, correction_deg: 3.0 },
            SiteErrorEntry { nominal_deg: 270.0, correction_deg: -2.0 },
        ]);

        // Signal from ~90°: raw bearing 90° + correction ≈ -1° → 89°.
        let result = df.estimate_bearing_amplitude(0.0, 1.0, 1.0);
        assert!(
            (result.bearing_deg - 89.0).abs() < TOL,
            "expected ~89° after correction, got {:.2}°",
            result.bearing_deg
        );
    }

    // ── Adcock antenna ───────────────────────────────────────────────

    #[test]
    fn test_adcock_bearing() {
        let freq = 100.0e6;
        let lambda = 3.0e8 / freq;
        // Quarter-wave spacing.
        let df = WatsonWattDf::new(
            AntennaType::Adcock { spacing_m: lambda / 4.0 },
            freq,
        );
        let (ns, ew, sense) = amp_sample(60.0);
        let result = df.estimate_bearing_amplitude(ns, ew, sense);
        assert!(
            (result.bearing_deg - 60.0).abs() < TOL,
            "expected ~60° with Adcock, got {:.2}°",
            result.bearing_deg
        );
    }

    // ── Calibration ──────────────────────────────────────────────────

    #[test]
    fn test_calibration_effect() {
        let mut df = WatsonWattDf::new(AntennaType::CrossedLoop, 100.0e6);
        // Without calibration: 45° signal.
        let (ns, ew, sense) = amp_sample(45.0);
        let r1 = df.estimate_bearing_amplitude(ns, ew, sense);
        assert!((r1.bearing_deg - 45.0).abs() < TOL);

        // Apply a cal factor that doubles the EW channel → bearing shifts
        // towards the EW axis (i.e., towards 90°).
        df.set_calibration(1.0, 2.0);
        let r2 = df.estimate_bearing_amplitude(ns, ew, sense);
        assert!(
            r2.bearing_deg > r1.bearing_deg,
            "calibration should shift bearing towards 90°"
        );
    }

    // ── Quality metric ───────────────────────────────────────────────

    #[test]
    fn test_quality_range() {
        let df = WatsonWattDf::new(AntennaType::CrossedLoop, 100.0e6);
        let result = df.estimate_bearing_amplitude(1.0, 0.5, 2.0);
        assert!(
            (0.0..=1.0).contains(&result.quality),
            "quality must be in [0,1], got {:.4}",
            result.quality
        );
    }

    // ── Edge cases / helpers ─────────────────────────────────────────

    #[test]
    fn test_wrap_deg() {
        assert!((wrap_deg(0.0) - 0.0).abs() < 1e-12);
        assert!((wrap_deg(360.0) - 0.0).abs() < 1e-12);
        assert!((wrap_deg(-90.0) - 270.0).abs() < 1e-12);
        assert!((wrap_deg(450.0) - 90.0).abs() < 1e-12);
    }

    #[test]
    fn test_wrap_pm180() {
        assert!((wrap_pm180(0.0) - 0.0).abs() < 1e-12);
        assert!((wrap_pm180(190.0) - (-170.0)).abs() < 1e-12);
        assert!((wrap_pm180(-190.0) - 170.0).abs() < 1e-12);
    }

    #[test]
    fn test_complex_helpers() {
        let a = (3.0, 4.0);
        assert!((c_mag(a) - 5.0).abs() < 1e-12);

        let b = (1.0, 0.0);
        let ab = c_mul(a, b);
        assert!((ab.0 - 3.0).abs() < 1e-12);
        assert!((ab.1 - 4.0).abs() < 1e-12);

        let ac = c_mul(a, c_conj(a));
        // |a|^2 = 25, imaginary part = 0.
        assert!((ac.0 - 25.0).abs() < 1e-12);
        assert!(ac.1.abs() < 1e-12);

        let s = c_add(a, b);
        assert!((s.0 - 4.0).abs() < 1e-12);
        assert!((s.1 - 4.0).abs() < 1e-12);

        let sc = c_scale(a, 2.0);
        assert!((sc.0 - 6.0).abs() < 1e-12);
        assert!((sc.1 - 8.0).abs() < 1e-12);
    }

    #[test]
    fn test_circular_statistics_with_rms() {
        let angles = vec![88.0, 89.0, 90.0, 91.0, 92.0];
        let stats = circular_statistics(&angles, Some(90.0));
        assert!(stats.rms_error_deg.is_some());
        let rms = stats.rms_error_deg.unwrap();
        // Errors are -2, -1, 0, 1, 2 → RMS = sqrt(10/5) ≈ 1.414
        assert!(
            (rms - (2.0_f64).sqrt()).abs() < 0.1,
            "expected ~1.414° rms, got {:.4}°",
            rms
        );
    }

    #[test]
    fn test_rms_error_empty() {
        let rms = WatsonWattDf::rms_bearing_error(&[], 90.0);
        assert!((rms - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_antenna_type_accessor() {
        let df = WatsonWattDf::new(AntennaType::CrossedLoop, 433.0e6);
        assert_eq!(df.antenna_type(), AntennaType::CrossedLoop);
        assert!((df.frequency_hz() - 433.0e6).abs() < 1.0);
    }
}
