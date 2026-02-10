//! # Rain Attenuation Predictor
//!
//! Estimates rainfall-induced absorption and scattering losses for microwave and
//! millimeter-wave radio links using ITU-R rain models (P.838, P.837, P.530).
//!
//! This module provides specific attenuation from rain rate via the power-law model
//! (γ = k · R^α), effective path length reduction, total path attenuation for both
//! terrestrial and earth-space links, rain rate exceedance statistics, link margin
//! calculations, frequency scaling, and cross-polarization discrimination (XPD)
//! degradation due to rain.
//!
//! All complex-number arithmetic uses `(f64, f64)` tuples representing `(re, im)`.
//!
//! # Example
//!
//! ```
//! use r4w_core::rain_attenuation_predictor::{RainAttenuationPredictor, Polarization};
//!
//! // 28 GHz terrestrial link, 5 km distance, 0° elevation (horizontal path)
//! let predictor = RainAttenuationPredictor::new(28.0, 5.0, 0.0);
//!
//! // Specific attenuation at 25 mm/h rain rate, horizontal polarization
//! let gamma = predictor.specific_attenuation(25.0, Polarization::Horizontal);
//! assert!(gamma > 0.0, "specific attenuation must be positive for non-zero rain");
//!
//! // Total path attenuation
//! let total = predictor.total_attenuation_terrestrial(25.0, Polarization::Horizontal);
//! assert!(total > gamma, "total > specific since path > 1 km at this config");
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Polarization of the radio link.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Polarization {
    /// Electric-field vector is horizontal.
    Horizontal,
    /// Electric-field vector is vertical.
    Vertical,
    /// Circular polarization – coefficients are averaged from H and V.
    Circular,
}

/// Link type for path attenuation calculations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkType {
    /// Horizontal or near-horizontal terrestrial link.
    Terrestrial,
    /// Slant path to/from a satellite (uses elevation angle).
    EarthSpace,
}

/// Result of a link-margin calculation.
#[derive(Debug, Clone)]
pub struct LinkMarginResult {
    /// Required fade margin (dB) to achieve the target availability.
    pub required_margin_db: f64,
    /// Rain rate (mm/h) exceeded for the given percentage of time.
    pub rain_rate_exceeded: f64,
    /// Total path attenuation (dB) at that rain rate.
    pub path_attenuation_db: f64,
    /// Effective path length (km) after reduction factor.
    pub effective_path_km: f64,
}

/// Result of cross-polarization discrimination degradation.
#[derive(Debug, Clone)]
pub struct XpdResult {
    /// Baseline XPD in clear sky (dB).
    pub xpd_clear_db: f64,
    /// XPD degradation caused by rain (dB, positive value).
    pub xpd_rain_degradation_db: f64,
    /// Resulting XPD during rain (dB).
    pub xpd_rain_db: f64,
    /// Co-polar attenuation used for the calculation (dB).
    pub co_polar_attenuation_db: f64,
}

/// Frequency-scaling result.
#[derive(Debug, Clone)]
pub struct FrequencyScalingResult {
    /// Attenuation at the reference frequency (dB).
    pub attenuation_ref_db: f64,
    /// Scaled attenuation at the target frequency (dB).
    pub attenuation_target_db: f64,
    /// Reference frequency (GHz).
    pub freq_ref_ghz: f64,
    /// Target frequency (GHz).
    pub freq_target_ghz: f64,
    /// Scaling ratio (target / reference).
    pub scaling_ratio: f64,
}

// ---------------------------------------------------------------------------
// ITU-R P.838 coefficient table
// ---------------------------------------------------------------------------

/// A single row of the ITU-R P.838 power-law coefficient table.
/// Each row carries frequency (GHz), k_h, α_h, k_v, α_v.
#[derive(Debug, Clone, Copy)]
struct P838Row {
    freq: f64,
    k_h: f64,
    alpha_h: f64,
    k_v: f64,
    alpha_v: f64,
}

/// Subset of the ITU-R P.838-3 table covering 1–100 GHz.
/// Values are representative of the recommendation (log-interpolation is used
/// between entries).
const P838_TABLE: &[P838Row] = &[
    P838Row { freq: 1.0,   k_h: 0.0000387, alpha_h: 0.912,  k_v: 0.0000352, alpha_v: 0.880 },
    P838Row { freq: 2.0,   k_h: 0.000154,  alpha_h: 0.963,  k_v: 0.000138,  alpha_v: 0.923 },
    P838Row { freq: 4.0,   k_h: 0.000650,  alpha_h: 1.121,  k_v: 0.000591,  alpha_v: 1.075 },
    P838Row { freq: 6.0,   k_h: 0.00175,   alpha_h: 1.308,  k_v: 0.00155,   alpha_v: 1.265 },
    P838Row { freq: 7.0,   k_h: 0.00301,   alpha_h: 1.332,  k_v: 0.00265,   alpha_v: 1.312 },
    P838Row { freq: 8.0,   k_h: 0.00454,   alpha_h: 1.327,  k_v: 0.00395,   alpha_v: 1.310 },
    P838Row { freq: 10.0,  k_h: 0.0101,    alpha_h: 1.276,  k_v: 0.00887,   alpha_v: 1.264 },
    P838Row { freq: 12.0,  k_h: 0.0188,    alpha_h: 1.217,  k_v: 0.0168,    alpha_v: 1.200 },
    P838Row { freq: 15.0,  k_h: 0.0367,    alpha_h: 1.154,  k_v: 0.0335,    alpha_v: 1.128 },
    P838Row { freq: 20.0,  k_h: 0.0751,    alpha_h: 1.099,  k_v: 0.0691,    alpha_v: 1.065 },
    P838Row { freq: 25.0,  k_h: 0.124,     alpha_h: 1.061,  k_v: 0.113,     alpha_v: 1.030 },
    P838Row { freq: 30.0,  k_h: 0.187,     alpha_h: 1.021,  k_v: 0.167,     alpha_v: 1.000 },
    P838Row { freq: 35.0,  k_h: 0.263,     alpha_h: 0.979,  k_v: 0.233,     alpha_v: 0.963 },
    P838Row { freq: 40.0,  k_h: 0.350,     alpha_h: 0.939,  k_v: 0.310,     alpha_v: 0.929 },
    P838Row { freq: 50.0,  k_h: 0.536,     alpha_h: 0.873,  k_v: 0.479,     alpha_v: 0.868 },
    P838Row { freq: 60.0,  k_h: 0.707,     alpha_h: 0.826,  k_v: 0.642,     alpha_v: 0.824 },
    P838Row { freq: 70.0,  k_h: 0.851,     alpha_h: 0.793,  k_v: 0.784,     alpha_v: 0.793 },
    P838Row { freq: 80.0,  k_h: 0.975,     alpha_h: 0.769,  k_v: 0.906,     alpha_v: 0.769 },
    P838Row { freq: 90.0,  k_h: 1.06,      alpha_h: 0.753,  k_v: 0.999,     alpha_v: 0.754 },
    P838Row { freq: 100.0, k_h: 1.12,      alpha_h: 0.743,  k_v: 1.06,      alpha_v: 0.744 },
];

// ---------------------------------------------------------------------------
// Helper: log-linear interpolation in the coefficient table
// ---------------------------------------------------------------------------

/// Look up (k, α) for a given frequency and polarization by log-interpolating
/// the P.838 table.
fn lookup_coefficients(freq_ghz: f64, pol: Polarization) -> (f64, f64) {
    let table = P838_TABLE;
    let clamped = freq_ghz.clamp(table[0].freq, table[table.len() - 1].freq);

    // Find bracketing entries.
    let mut lo = 0usize;
    for i in 0..table.len() - 1 {
        if table[i + 1].freq >= clamped {
            lo = i;
            break;
        }
    }
    let hi = lo + 1;

    let (k_lo, a_lo, k_hi, a_hi) = match pol {
        Polarization::Horizontal => (table[lo].k_h, table[lo].alpha_h, table[hi].k_h, table[hi].alpha_h),
        Polarization::Vertical   => (table[lo].k_v, table[lo].alpha_v, table[hi].k_v, table[hi].alpha_v),
        Polarization::Circular   => {
            // Average of H and V in log domain for k, linear for α.
            let kh_lo = table[lo].k_h; let kv_lo = table[lo].k_v;
            let kh_hi = table[hi].k_h; let kv_hi = table[hi].k_v;
            let ah_lo = table[lo].alpha_h; let av_lo = table[lo].alpha_v;
            let ah_hi = table[hi].alpha_h; let av_hi = table[hi].alpha_v;
            let k_lo_c = (kh_lo * kv_lo).sqrt();
            let k_hi_c = (kh_hi * kv_hi).sqrt();
            let a_lo_c = (ah_lo + av_lo) / 2.0;
            let a_hi_c = (ah_hi + av_hi) / 2.0;
            (k_lo_c, a_lo_c, k_hi_c, a_hi_c)
        }
    };

    // Log-interpolation for k, linear interpolation for α.
    if (table[hi].freq - table[lo].freq).abs() < 1e-12 {
        return (k_lo, a_lo);
    }
    let frac = (clamped.ln() - table[lo].freq.ln()) / (table[hi].freq.ln() - table[lo].freq.ln());
    let k = (k_lo.ln() + frac * (k_hi.ln() - k_lo.ln())).exp();
    let alpha = a_lo + frac * (a_hi - a_lo);
    (k, alpha)
}

// ---------------------------------------------------------------------------
// Complex tuple helpers (re, im)
// ---------------------------------------------------------------------------

/// Multiply two complex tuples.
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex magnitude.
#[inline]
fn cabs(a: (f64, f64)) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

/// Complex exponential e^{j·θ}.
#[inline]
fn cexp_j(theta: f64) -> (f64, f64) {
    (theta.cos(), theta.sin())
}

/// Add two complex tuples.
#[inline]
#[allow(dead_code)]
fn cadd(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Scale a complex tuple by a real.
#[inline]
#[allow(dead_code)]
fn cscale(s: f64, a: (f64, f64)) -> (f64, f64) {
    (s * a.0, s * a.1)
}

// ---------------------------------------------------------------------------
// RainAttenuationPredictor
// ---------------------------------------------------------------------------

/// Predicts rain attenuation for a radio link using ITU-R models.
///
/// # Fields (set at construction)
///
/// * `freq_ghz` – operating frequency (GHz), clamped to 1–100.
/// * `distance_km` – path distance (km) for terrestrial links.
/// * `elevation_deg` – elevation angle (degrees) for earth-space links.
#[derive(Debug, Clone)]
pub struct RainAttenuationPredictor {
    /// Operating frequency in GHz (1–100).
    pub freq_ghz: f64,
    /// Path distance in km (terrestrial links).
    pub distance_km: f64,
    /// Elevation angle in degrees (earth-space links).
    pub elevation_deg: f64,
}

impl RainAttenuationPredictor {
    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------

    /// Create a new predictor.
    ///
    /// * `freq_ghz` – frequency in GHz (will be clamped to 1–100 internally).
    /// * `distance_km` – terrestrial path length in km.
    /// * `elevation_deg` – elevation angle in degrees (0 = horizontal).
    pub fn new(freq_ghz: f64, distance_km: f64, elevation_deg: f64) -> Self {
        Self {
            freq_ghz: freq_ghz.clamp(1.0, 100.0),
            distance_km: distance_km.max(0.0),
            elevation_deg: elevation_deg.clamp(0.0, 90.0),
        }
    }

    // -----------------------------------------------------------------------
    // ITU-R P.838 specific attenuation
    // -----------------------------------------------------------------------

    /// Return the power-law coefficients `(k, α)` for the configured frequency
    /// and the given polarization.
    pub fn coefficients(&self, pol: Polarization) -> (f64, f64) {
        lookup_coefficients(self.freq_ghz, pol)
    }

    /// Specific attenuation γ (dB/km) for a rain rate `R` (mm/h).
    ///
    /// Uses the ITU-R P.838 power-law model: γ = k · R^α.
    pub fn specific_attenuation(&self, rain_rate_mm_h: f64, pol: Polarization) -> f64 {
        if rain_rate_mm_h <= 0.0 {
            return 0.0;
        }
        let (k, alpha) = self.coefficients(pol);
        k * rain_rate_mm_h.powf(alpha)
    }

    // -----------------------------------------------------------------------
    // Effective path length (ITU-R P.530-17 §2.4.1 simplified)
    // -----------------------------------------------------------------------

    /// Distance reduction factor `r` for a terrestrial link.
    ///
    /// This is a simplified version of the ITU-R P.530 model:
    ///   r = 1 / (1 + d / d₀)
    /// where d₀ = 35 · e^{-0.015 R} for rain rate R (mm/h).
    pub fn distance_reduction_factor(&self, rain_rate_mm_h: f64) -> f64 {
        if rain_rate_mm_h <= 0.0 || self.distance_km <= 0.0 {
            return 1.0;
        }
        let d0 = 35.0 * (-0.015 * rain_rate_mm_h).exp();
        1.0 / (1.0 + self.distance_km / d0)
    }

    /// Effective path length (km) after applying the reduction factor.
    pub fn effective_path_length(&self, rain_rate_mm_h: f64) -> f64 {
        self.distance_km * self.distance_reduction_factor(rain_rate_mm_h)
    }

    // -----------------------------------------------------------------------
    // Total path attenuation
    // -----------------------------------------------------------------------

    /// Total attenuation (dB) for a terrestrial link.
    ///
    /// A = γ · d_eff
    pub fn total_attenuation_terrestrial(
        &self,
        rain_rate_mm_h: f64,
        pol: Polarization,
    ) -> f64 {
        let gamma = self.specific_attenuation(rain_rate_mm_h, pol);
        let d_eff = self.effective_path_length(rain_rate_mm_h);
        gamma * d_eff
    }

    /// Total attenuation (dB) for an earth-space (slant) link.
    ///
    /// The rain path through the atmosphere is modelled as `h_rain / sin(θ)`
    /// where `h_rain` is the effective rain height (≈ 5 km) and `θ` is the
    /// elevation angle.  A minimum elevation of 5° is enforced.
    pub fn total_attenuation_earth_space(
        &self,
        rain_rate_mm_h: f64,
        pol: Polarization,
    ) -> f64 {
        let gamma = self.specific_attenuation(rain_rate_mm_h, pol);
        let h_rain_km = 5.0; // effective rain height
        let elev = self.elevation_deg.max(5.0);
        let sin_elev = (elev * PI / 180.0).sin();
        let slant_km = h_rain_km / sin_elev;
        // Apply a reduction factor on the slant path as well.
        let r = 1.0 / (1.0 + slant_km / (35.0 * (-0.015 * rain_rate_mm_h).exp()));
        gamma * slant_km * r
    }

    /// Total path attenuation for the given link type.
    pub fn total_attenuation(
        &self,
        rain_rate_mm_h: f64,
        pol: Polarization,
        link_type: LinkType,
    ) -> f64 {
        match link_type {
            LinkType::Terrestrial => self.total_attenuation_terrestrial(rain_rate_mm_h, pol),
            LinkType::EarthSpace  => self.total_attenuation_earth_space(rain_rate_mm_h, pol),
        }
    }

    // -----------------------------------------------------------------------
    // Rain rate exceedance (ITU-R P.837 simplified)
    // -----------------------------------------------------------------------

    /// Estimate the rain rate (mm/h) exceeded for `p` percent of an average
    /// year, given a long-term mean annual rainfall rate `r_mean` (mm/h at
    /// 0.01 % exceedance).
    ///
    /// Uses a simplified log-normal exceedance model:
    ///   R(p) = r_mean · (0.01 / p)^β  where β ≈ 0.55.
    ///
    /// `p` is in percent (e.g., 0.01 for 99.99 % availability).
    pub fn rain_rate_exceedance(r_001: f64, p_percent: f64) -> f64 {
        if p_percent <= 0.0 || r_001 <= 0.0 {
            return 0.0;
        }
        let p = p_percent.clamp(0.001, 10.0);
        let beta = 0.55;
        r_001 * (0.01 / p).powf(beta)
    }

    // -----------------------------------------------------------------------
    // Link margin for a target availability
    // -----------------------------------------------------------------------

    /// Calculate the fade margin required to achieve `availability` (as a
    /// fraction, e.g., 0.9999 for 99.99 %) given the R₀.₀₁ rain rate.
    ///
    /// Returns a [`LinkMarginResult`] with the margin and supporting values.
    pub fn link_margin(
        &self,
        r_001: f64,
        availability: f64,
        pol: Polarization,
        link_type: LinkType,
    ) -> LinkMarginResult {
        let p_percent = (1.0 - availability.clamp(0.0, 0.99999)) * 100.0;
        let rain_rate = Self::rain_rate_exceedance(r_001, p_percent);
        let attenuation = self.total_attenuation(rain_rate, pol, link_type);
        let eff_km = match link_type {
            LinkType::Terrestrial => self.effective_path_length(rain_rate),
            LinkType::EarthSpace => {
                let h = 5.0;
                let elev = self.elevation_deg.max(5.0);
                h / (elev * PI / 180.0).sin()
            }
        };
        LinkMarginResult {
            required_margin_db: attenuation,
            rain_rate_exceeded: rain_rate,
            path_attenuation_db: attenuation,
            effective_path_km: eff_km,
        }
    }

    // -----------------------------------------------------------------------
    // Frequency scaling of rain attenuation (ITU-R P.530 §2.4.2)
    // -----------------------------------------------------------------------

    /// Scale a known attenuation at `freq_ref_ghz` to `freq_target_ghz`.
    ///
    /// Uses the ratio of specific attenuations (same rain rate) as the
    /// scaling factor:
    ///   A₂ = A₁ · (γ₂ / γ₁)
    ///
    /// A reference rain rate of 25 mm/h is used internally to compute γ.
    pub fn frequency_scaling(
        &self,
        attenuation_ref_db: f64,
        freq_ref_ghz: f64,
        freq_target_ghz: f64,
        pol: Polarization,
    ) -> FrequencyScalingResult {
        let r_ref: f64 = 25.0; // reference rain rate for ratio calculation
        let (k1, a1) = lookup_coefficients(freq_ref_ghz, pol);
        let (k2, a2) = lookup_coefficients(freq_target_ghz, pol);
        let gamma1: f64 = k1 * r_ref.powf(a1);
        let gamma2: f64 = k2 * r_ref.powf(a2);
        let ratio = if gamma1 > 0.0 { gamma2 / gamma1 } else { 1.0 };
        FrequencyScalingResult {
            attenuation_ref_db,
            attenuation_target_db: attenuation_ref_db * ratio,
            freq_ref_ghz,
            freq_target_ghz,
            scaling_ratio: ratio,
        }
    }

    // -----------------------------------------------------------------------
    // Cross-polarization discrimination (XPD) degradation
    // -----------------------------------------------------------------------

    /// Estimate XPD degradation due to rain (ITU-R P.530 simplified model).
    ///
    /// `xpd_clear_db` – baseline XPD in clear-sky conditions (dB).
    /// Returns the degraded XPD during rain and supporting figures.
    ///
    /// Model: XPD_rain ≈ XPD_clear - Q(f) · log10(A_copol)
    /// where Q(f) = 20 for f < 10 GHz, 30 for f > 30 GHz, linearly between.
    pub fn xpd_degradation(
        &self,
        rain_rate_mm_h: f64,
        pol: Polarization,
        link_type: LinkType,
        xpd_clear_db: f64,
    ) -> XpdResult {
        let a_copol = self.total_attenuation(rain_rate_mm_h, pol, link_type).max(0.01);
        // Q factor varies with frequency.
        let q = if self.freq_ghz <= 10.0 {
            20.0
        } else if self.freq_ghz >= 30.0 {
            30.0
        } else {
            20.0 + 10.0 * (self.freq_ghz - 10.0) / 20.0
        };
        let degradation = q * a_copol.log10();
        let xpd_rain = xpd_clear_db - degradation;
        XpdResult {
            xpd_clear_db,
            xpd_rain_degradation_db: degradation,
            xpd_rain_db: xpd_rain,
            co_polar_attenuation_db: a_copol,
        }
    }

    // -----------------------------------------------------------------------
    // Convenience: complex-valued rain-fade phasor
    // -----------------------------------------------------------------------

    /// Return a complex (re, im) phasor representing the amplitude and phase
    /// shift caused by rain attenuation of `atten_db` dB.
    ///
    /// The amplitude is 10^{-atten_db/20} and the phase shift is a simple
    /// propagation-delay model: Δφ = 2π f d n / c  (n ≈ 1 + 1e-6 · R).
    pub fn rain_fade_phasor(
        &self,
        rain_rate_mm_h: f64,
        pol: Polarization,
        link_type: LinkType,
    ) -> (f64, f64) {
        let atten_db = self.total_attenuation(rain_rate_mm_h, pol, link_type);
        let amplitude = 10.0_f64.powf(-atten_db / 20.0);
        // Simple excess-phase model: rain increases refractive index slightly.
        let n_excess = 1.0e-6 * rain_rate_mm_h;
        let freq_hz = self.freq_ghz * 1e9;
        let d_km = self.distance_km;
        let delta_phase = 2.0 * PI * freq_hz * d_km * 1e3 * n_excess / 2.998e8;
        cmul((amplitude, 0.0), cexp_j(delta_phase))
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-9;

    // Helper to check approximate equality.
    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // 1. Basic construction
    #[test]
    fn test_construction_basic() {
        let p = RainAttenuationPredictor::new(28.0, 5.0, 30.0);
        assert!((p.freq_ghz - 28.0).abs() < EPSILON);
        assert!((p.distance_km - 5.0).abs() < EPSILON);
        assert!((p.elevation_deg - 30.0).abs() < EPSILON);
    }

    // 2. Frequency clamping
    #[test]
    fn test_freq_clamping() {
        let low = RainAttenuationPredictor::new(0.1, 1.0, 0.0);
        assert!((low.freq_ghz - 1.0).abs() < EPSILON);
        let high = RainAttenuationPredictor::new(200.0, 1.0, 0.0);
        assert!((high.freq_ghz - 100.0).abs() < EPSILON);
    }

    // 3. Specific attenuation is zero for zero rain
    #[test]
    fn test_specific_attenuation_zero_rain() {
        let p = RainAttenuationPredictor::new(28.0, 5.0, 0.0);
        let gamma = p.specific_attenuation(0.0, Polarization::Horizontal);
        assert!((gamma).abs() < EPSILON);
    }

    // 4. Specific attenuation is positive for positive rain
    #[test]
    fn test_specific_attenuation_positive() {
        let p = RainAttenuationPredictor::new(28.0, 5.0, 0.0);
        let gamma = p.specific_attenuation(25.0, Polarization::Horizontal);
        assert!(gamma > 0.0, "gamma = {gamma}");
    }

    // 5. Specific attenuation increases with rain rate
    #[test]
    fn test_specific_attenuation_monotonic() {
        let p = RainAttenuationPredictor::new(28.0, 5.0, 0.0);
        let g10 = p.specific_attenuation(10.0, Polarization::Horizontal);
        let g50 = p.specific_attenuation(50.0, Polarization::Horizontal);
        let g100 = p.specific_attenuation(100.0, Polarization::Horizontal);
        assert!(g10 < g50, "g10={g10}, g50={g50}");
        assert!(g50 < g100, "g50={g50}, g100={g100}");
    }

    // 6. Specific attenuation increases with frequency
    #[test]
    fn test_specific_attenuation_vs_frequency() {
        let p10 = RainAttenuationPredictor::new(10.0, 5.0, 0.0);
        let p30 = RainAttenuationPredictor::new(30.0, 5.0, 0.0);
        let p60 = RainAttenuationPredictor::new(60.0, 5.0, 0.0);
        let rain = 25.0;
        let pol = Polarization::Horizontal;
        let g10 = p10.specific_attenuation(rain, pol);
        let g30 = p30.specific_attenuation(rain, pol);
        let g60 = p60.specific_attenuation(rain, pol);
        assert!(g10 < g30, "g10={g10}, g30={g30}");
        assert!(g30 < g60, "g30={g30}, g60={g60}");
    }

    // 7. H vs V polarization difference
    #[test]
    fn test_h_vs_v_polarization() {
        let p = RainAttenuationPredictor::new(28.0, 5.0, 0.0);
        let gh = p.specific_attenuation(25.0, Polarization::Horizontal);
        let gv = p.specific_attenuation(25.0, Polarization::Vertical);
        // At most frequencies H attenuation >= V attenuation.
        assert!(gh > 0.0);
        assert!(gv > 0.0);
        // They should differ (not exactly equal).
        assert!((gh - gv).abs() > 1e-6, "gh={gh}, gv={gv}");
    }

    // 8. Circular polarization is between H and V
    #[test]
    fn test_circular_between_h_and_v() {
        let p = RainAttenuationPredictor::new(28.0, 5.0, 0.0);
        let gh = p.specific_attenuation(25.0, Polarization::Horizontal);
        let gv = p.specific_attenuation(25.0, Polarization::Vertical);
        let gc = p.specific_attenuation(25.0, Polarization::Circular);
        let lo = gh.min(gv);
        let hi = gh.max(gv);
        assert!(gc >= lo - 0.01 && gc <= hi + 0.01,
            "gc={gc} not between lo={lo} and hi={hi}");
    }

    // 9. Distance reduction factor
    #[test]
    fn test_distance_reduction_factor() {
        let p = RainAttenuationPredictor::new(28.0, 10.0, 0.0);
        let r = p.distance_reduction_factor(25.0);
        assert!(r > 0.0 && r < 1.0, "r={r} should be in (0, 1)");
    }

    // 10. Distance reduction factor is 1.0 for zero rain
    #[test]
    fn test_distance_reduction_factor_zero_rain() {
        let p = RainAttenuationPredictor::new(28.0, 10.0, 0.0);
        let r = p.distance_reduction_factor(0.0);
        assert!((r - 1.0).abs() < EPSILON);
    }

    // 11. Effective path length < actual distance for heavy rain
    #[test]
    fn test_effective_path_length() {
        let p = RainAttenuationPredictor::new(28.0, 20.0, 0.0);
        let eff = p.effective_path_length(50.0);
        assert!(eff > 0.0 && eff < 20.0, "eff={eff}");
    }

    // 12. Total terrestrial attenuation sanity
    #[test]
    fn test_total_terrestrial_attenuation() {
        let p = RainAttenuationPredictor::new(28.0, 5.0, 0.0);
        let a = p.total_attenuation_terrestrial(25.0, Polarization::Horizontal);
        // Should be positive and reasonable (a few dB to tens of dB).
        assert!(a > 0.1, "a={a}");
        assert!(a < 200.0, "a={a} unreasonably large");
    }

    // 13. Total earth-space attenuation
    #[test]
    fn test_total_earth_space_attenuation() {
        let p = RainAttenuationPredictor::new(20.0, 0.0, 30.0);
        let a = p.total_attenuation_earth_space(25.0, Polarization::Vertical);
        assert!(a > 0.0, "a={a}");
        // Lower elevation = longer slant path = more attenuation.
        let p_low = RainAttenuationPredictor::new(20.0, 0.0, 10.0);
        let a_low = p_low.total_attenuation_earth_space(25.0, Polarization::Vertical);
        assert!(a_low > a, "a_low={a_low} should exceed a={a}");
    }

    // 14. Rain rate exceedance
    #[test]
    fn test_rain_rate_exceedance() {
        // At 0.01% exceedance, R should equal r_001.
        let r = RainAttenuationPredictor::rain_rate_exceedance(50.0, 0.01);
        assert!(approx_eq(r, 50.0, 0.01), "r={r}");
        // At higher exceedance percentages, R should be lower.
        let r1 = RainAttenuationPredictor::rain_rate_exceedance(50.0, 1.0);
        assert!(r1 < 50.0, "r1={r1}");
    }

    // 15. Link margin result
    #[test]
    fn test_link_margin() {
        let p = RainAttenuationPredictor::new(28.0, 5.0, 0.0);
        let lm = p.link_margin(50.0, 0.9999, Polarization::Horizontal, LinkType::Terrestrial);
        assert!(lm.required_margin_db > 0.0);
        assert!(lm.rain_rate_exceeded > 0.0);
        assert!(lm.effective_path_km > 0.0);
        assert!(lm.effective_path_km <= 5.0);
    }

    // 16. Frequency scaling
    #[test]
    fn test_frequency_scaling() {
        let p = RainAttenuationPredictor::new(28.0, 5.0, 0.0);
        let fs = p.frequency_scaling(10.0, 12.0, 28.0, Polarization::Horizontal);
        // 28 GHz should have higher attenuation than 12 GHz.
        assert!(fs.scaling_ratio > 1.0, "ratio={}", fs.scaling_ratio);
        assert!(fs.attenuation_target_db > fs.attenuation_ref_db);
    }

    // 17. XPD degradation
    #[test]
    fn test_xpd_degradation() {
        let p = RainAttenuationPredictor::new(20.0, 10.0, 0.0);
        let xpd = p.xpd_degradation(30.0, Polarization::Horizontal, LinkType::Terrestrial, 40.0);
        assert!(xpd.xpd_rain_db < xpd.xpd_clear_db, "rain should degrade XPD");
        assert!(xpd.xpd_rain_degradation_db > 0.0);
        assert!(xpd.co_polar_attenuation_db > 0.0);
    }

    // 18. Rain fade phasor has magnitude <= 1
    #[test]
    fn test_rain_fade_phasor() {
        let p = RainAttenuationPredictor::new(28.0, 5.0, 0.0);
        let phasor = p.rain_fade_phasor(25.0, Polarization::Horizontal, LinkType::Terrestrial);
        let mag = cabs(phasor);
        assert!(mag > 0.0 && mag <= 1.0 + 1e-12, "mag={mag}");
    }

    // 19. Complex helpers correctness
    #[test]
    fn test_complex_helpers() {
        let a = (3.0, 4.0);
        assert!(approx_eq(cabs(a), 5.0, 1e-12));

        let unit = cexp_j(0.0);
        assert!(approx_eq(unit.0, 1.0, 1e-12));
        assert!(approx_eq(unit.1, 0.0, 1e-12));

        let neg = cexp_j(PI);
        assert!(approx_eq(neg.0, -1.0, 1e-12));
        assert!(neg.1.abs() < 1e-12);

        let prod = cmul((1.0, 2.0), (3.0, 4.0));
        // (1+2i)(3+4i) = 3+4i+6i+8i² = -5+10i
        assert!(approx_eq(prod.0, -5.0, 1e-12));
        assert!(approx_eq(prod.1, 10.0, 1e-12));

        let sum = cadd((1.0, 2.0), (3.0, 4.0));
        assert!(approx_eq(sum.0, 4.0, 1e-12));
        assert!(approx_eq(sum.1, 6.0, 1e-12));

        let scaled = cscale(2.0, (3.0, 4.0));
        assert!(approx_eq(scaled.0, 6.0, 1e-12));
        assert!(approx_eq(scaled.1, 8.0, 1e-12));
    }

    // 20. Coefficients table boundary (1 GHz)
    #[test]
    fn test_coefficients_boundary_low() {
        let p = RainAttenuationPredictor::new(1.0, 1.0, 0.0);
        let (k, alpha) = p.coefficients(Polarization::Horizontal);
        assert!(k > 0.0 && k < 0.001, "k={k}");
        assert!(alpha > 0.5 && alpha < 1.5, "alpha={alpha}");
    }

    // 21. Coefficients table boundary (100 GHz)
    #[test]
    fn test_coefficients_boundary_high() {
        let p = RainAttenuationPredictor::new(100.0, 1.0, 0.0);
        let (k, alpha) = p.coefficients(Polarization::Horizontal);
        assert!(k > 0.5, "k={k}");
        assert!(alpha > 0.5 && alpha < 1.0, "alpha={alpha}");
    }

    // 22. total_attenuation dispatches correctly
    #[test]
    fn test_total_attenuation_dispatch() {
        let p = RainAttenuationPredictor::new(20.0, 5.0, 30.0);
        let a_terr = p.total_attenuation(25.0, Polarization::Horizontal, LinkType::Terrestrial);
        let a_es   = p.total_attenuation(25.0, Polarization::Horizontal, LinkType::EarthSpace);
        // Both should be positive and distinct.
        assert!(a_terr > 0.0);
        assert!(a_es > 0.0);
        assert!((a_terr - a_es).abs() > 0.001);
    }
}
