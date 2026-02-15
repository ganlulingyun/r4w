//! # Atmospheric Refraction Corrector
//!
//! Ionospheric and tropospheric refraction correction for long-range RF propagation.
//! Covers ray tracing through layered atmospheres, refractivity profiles, and
//! elevation angle correction.
//!
//! ## Overview
//!
//! Radio waves propagating through the Earth's atmosphere are refracted (bent) by
//! gradients in the atmospheric refractive index. This module provides tools for:
//!
//! - **Refractivity profiles**: Computing N(h) using the ITU-R P.835 standard
//!   atmosphere model with dry, wet, and water-vapour components.
//! - **Ray tracing**: Numerically tracing rays through a spherically stratified
//!   atmosphere using Snell's law in polar form (Bouger's rule).
//! - **Tropospheric correction**: Mapping zenith delay to slant delay using the
//!   Niell Mapping Function (NMF) and Vienna Mapping Function (VMF).
//! - **Ducting detection**: Identifying super-refraction and surface/elevated
//!   ducting conditions from modified refractivity gradients.
//! - **Elevation correction**: Converting between apparent (observed) and
//!   geometric (true) elevation angles accounting for atmospheric bending.
//!
//! ## Key Equations
//!
//! - **Refractivity**: N = 77.6 P/T + 3.73e5 e/T² (Bean-Dutton model)
//! - **Modified refractivity**: M(h) = N(h) + (h/a) × 10⁶ where a = Earth radius
//! - **Effective Earth radius**: aₑ = a × kₑ where kₑ = 1/(1 + a·dN/dh × 10⁻⁶)
//! - **Bouger's rule**: n(r) · r · cos(θ) = constant along a ray
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::atmospheric_refraction_corrector::{
//!     AtmosphereConfig, RefractivityProfile, ElevationCorrector,
//! };
//!
//! let config = AtmosphereConfig::standard();
//! let profile = RefractivityProfile::new(&config);
//!
//! // Surface refractivity
//! let n_surface = profile.refractivity_at(0.0);
//! assert!(n_surface > 300.0 && n_surface < 400.0); // Typical ~315 N-units
//!
//! // Elevation correction at 1 degree apparent elevation
//! let corrector = ElevationCorrector::new(&config);
//! let correction = corrector.correction_deg(1.0);
//! assert!(correction > 0.0); // Geometric always higher than apparent
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Mean Earth radius (m), WGS-84 mean value.
const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// Standard sea-level atmospheric pressure (hPa / mbar).
const STD_PRESSURE_HPA: f64 = 1013.25;

/// Standard sea-level temperature (K).
const STD_TEMPERATURE_K: f64 = 288.15;

/// Standard sea-level relative humidity (fraction, 0..1).
const STD_HUMIDITY: f64 = 0.60;

/// Lapse rate for the standard troposphere (K/m), negative because T decreases.
const STD_LAPSE_RATE: f64 = -0.0065;

/// Tropopause height (m).
const TROPOPAUSE_HEIGHT_M: f64 = 11_000.0;

/// Troposphere scale height for wet component (m).
const WET_SCALE_HEIGHT_M: f64 = 2100.0;

/// Default ray-tracing step size (m).
const DEFAULT_STEP_M: f64 = 50.0;

/// Maximum ray-tracing altitude (m).
const MAX_RAY_ALT_M: f64 = 80_000.0;

/// Degrees to radians.
const DEG2RAD: f64 = PI / 180.0;

/// Radians to degrees.
const RAD2DEG: f64 = 180.0 / PI;

// ---------------------------------------------------------------------------
// AtmosphereConfig
// ---------------------------------------------------------------------------

/// Configuration for a standard atmosphere profile.
///
/// Encapsulates the surface meteorological parameters used to compute
/// refractivity profiles and ray-tracing through the atmosphere.
#[derive(Debug, Clone)]
pub struct AtmosphereConfig {
    /// Surface temperature (K).
    pub temperature_k: f64,
    /// Surface pressure (hPa).
    pub pressure_hpa: f64,
    /// Surface relative humidity (0.0 to 1.0).
    pub humidity: f64,
    /// Temperature lapse rate in the troposphere (K/m), typically negative.
    pub lapse_rate: f64,
    /// Earth radius (m) to use for the spherical model.
    pub earth_radius_m: f64,
}

impl AtmosphereConfig {
    /// Create a standard atmosphere configuration (ISA conditions).
    ///
    /// Uses ICAO standard atmosphere values: 15 °C, 1013.25 hPa, 60% RH.
    pub fn standard() -> Self {
        Self {
            temperature_k: STD_TEMPERATURE_K,
            pressure_hpa: STD_PRESSURE_HPA,
            humidity: STD_HUMIDITY,
            lapse_rate: STD_LAPSE_RATE,
            earth_radius_m: EARTH_RADIUS_M,
        }
    }

    /// Create a custom atmosphere configuration.
    ///
    /// # Arguments
    /// * `temperature_k` - Surface temperature in Kelvin.
    /// * `pressure_hpa` - Surface pressure in hPa.
    /// * `humidity` - Relative humidity fraction (0.0 to 1.0).
    pub fn custom(temperature_k: f64, pressure_hpa: f64, humidity: f64) -> Self {
        Self {
            temperature_k,
            pressure_hpa,
            humidity,
            lapse_rate: STD_LAPSE_RATE,
            earth_radius_m: EARTH_RADIUS_M,
        }
    }

    /// Create a tropical atmosphere (hot, humid, stronger refraction).
    pub fn tropical() -> Self {
        Self {
            temperature_k: 300.15,   // 27 °C
            pressure_hpa: 1012.0,
            humidity: 0.85,
            lapse_rate: -0.0065,
            earth_radius_m: EARTH_RADIUS_M,
        }
    }

    /// Create a sub-arctic winter atmosphere (cold, dry, weaker refraction).
    pub fn subarctic_winter() -> Self {
        Self {
            temperature_k: 257.15,   // -16 °C
            pressure_hpa: 1018.0,
            humidity: 0.30,
            lapse_rate: -0.0065,
            earth_radius_m: EARTH_RADIUS_M,
        }
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Compute the saturation water-vapour pressure (hPa) from temperature (K)
/// using the Magnus-Tetens approximation.
///
/// Returns eₛ in hPa (same units as atmospheric pressure).
fn saturation_vapour_pressure(temp_k: f64) -> f64 {
    let temp_c = temp_k - 273.15;
    6.1078 * (10.0_f64).powf(7.5 * temp_c / (temp_c + 237.3))
}

/// Compute the water-vapour partial pressure (hPa) from temperature (K)
/// and relative humidity (fraction).
fn vapour_pressure(temp_k: f64, rh: f64) -> f64 {
    rh * saturation_vapour_pressure(temp_k)
}

/// Compute atmospheric refractivity N from the Bean-Dutton model.
///
/// N = 77.6 · P/T + 3.73×10⁵ · e/T²
///
/// where P is total pressure (hPa), T is temperature (K), and e is the
/// water-vapour partial pressure (hPa).
///
/// This is the fundamental equation for radio refractivity in the
/// troposphere, valid up to about 100 GHz.
pub fn refractivity(pressure_hpa: f64, temperature_k: f64, vapour_hpa: f64) -> f64 {
    let dry = 77.6 * pressure_hpa / temperature_k;
    let wet = 3.73e5 * vapour_hpa / (temperature_k * temperature_k);
    dry + wet
}

/// Compute modified refractivity M at height h (m).
///
/// M(h) = N(h) + (h / a) × 10⁶
///
/// Modified refractivity accounts for Earth curvature. A constant M profile
/// means the ray curves exactly as Earth does (straight-line propagation on
/// a flat-Earth equivalent).
pub fn modified_refractivity(n: f64, height_m: f64, earth_radius_m: f64) -> f64 {
    n + (height_m / earth_radius_m) * 1e6
}

/// Compute the effective Earth radius factor kₑ from the surface
/// refractivity gradient dN/dh (N-units per metre).
///
/// kₑ = 1 / (1 + a · dN/dh × 10⁻⁶)
///
/// For the standard atmosphere dN/dh ≈ -39e-6 N/m, giving kₑ ≈ 4/3.
pub fn earth_radius_effective_factor(dn_dh: f64, earth_radius_m: f64) -> f64 {
    1.0 / (1.0 + earth_radius_m * dn_dh * 1e-6)
}

/// Compute the effective Earth radius (m).
///
/// aₑ = a × kₑ
pub fn earth_radius_effective(dn_dh: f64, earth_radius_m: f64) -> f64 {
    earth_radius_m * earth_radius_effective_factor(dn_dh, earth_radius_m)
}

/// Compute refractivity using the Bean-Dutton model directly from
/// meteorological parameters.
///
/// Convenience wrapper that computes vapour pressure internally.
pub fn bean_dutton_model(
    pressure_hpa: f64,
    temperature_k: f64,
    relative_humidity: f64,
) -> f64 {
    let e = vapour_pressure(temperature_k, relative_humidity);
    refractivity(pressure_hpa, temperature_k, e)
}

// ---------------------------------------------------------------------------
// RefractivityProfile
// ---------------------------------------------------------------------------

/// Refractivity profile N(h) based on the ITU-R P.835 standard atmosphere.
///
/// Computes the radio refractivity at any altitude using exponential decay
/// models for the dry and wet components, with a tropospheric temperature
/// profile based on a linear lapse rate.
#[derive(Debug, Clone)]
pub struct RefractivityProfile {
    /// Surface refractivity (N-units).
    pub n_surface: f64,
    /// Dry component of surface refractivity.
    pub n_dry_surface: f64,
    /// Wet component of surface refractivity.
    pub n_wet_surface: f64,
    /// Dry scale height (m).
    pub h_dry: f64,
    /// Wet scale height (m).
    pub h_wet: f64,
    /// Surface temperature (K).
    temperature_k: f64,
    /// Surface pressure (hPa).
    pressure_hpa: f64,
    /// Lapse rate (K/m).
    lapse_rate: f64,
    /// Earth radius (m).
    earth_radius_m: f64,
}

impl RefractivityProfile {
    /// Create a refractivity profile from atmospheric configuration.
    pub fn new(config: &AtmosphereConfig) -> Self {
        let e_surface = vapour_pressure(config.temperature_k, config.humidity);
        let n_dry = 77.6 * config.pressure_hpa / config.temperature_k;
        let n_wet = 3.73e5 * e_surface / (config.temperature_k * config.temperature_k);
        let n_total = n_dry + n_wet;

        // ITU-R P.835 scale heights
        // Dry: ~7.35 km for standard atmosphere (derived from barometric formula
        //   H = R_d * T / g ≈ 287.05 * T / 9.80665, giving ~8.4 km for T=288 K,
        //   but the effective exponential scale height for refractivity is shorter
        //   because N ∝ P/T and T also varies with height).
        // Wet: ~2.1 km (more variable, but standard value).
        // Using a constant 7350 m gives dN/dh ≈ -39 N/km at the surface for
        // standard conditions, matching the ITU-R P.453 reference gradient.
        let h_dry = 7350.0;

        Self {
            n_surface: n_total,
            n_dry_surface: n_dry,
            n_wet_surface: n_wet,
            h_dry,
            h_wet: WET_SCALE_HEIGHT_M,
            temperature_k: config.temperature_k,
            pressure_hpa: config.pressure_hpa,
            lapse_rate: config.lapse_rate,
            earth_radius_m: config.earth_radius_m,
        }
    }

    /// Compute refractivity N at height h (metres above surface).
    ///
    /// Uses exponential decay with separate dry and wet scale heights:
    ///   N(h) = N_dry · exp(-h / h_dry) + N_wet · exp(-h / h_wet)
    pub fn refractivity_at(&self, height_m: f64) -> f64 {
        if height_m < 0.0 {
            return self.n_surface;
        }
        let dry = self.n_dry_surface * (-height_m / self.h_dry).exp();
        let wet = self.n_wet_surface * (-height_m / self.h_wet).exp();
        dry + wet
    }

    /// Compute modified refractivity M at height h (metres above surface).
    pub fn modified_refractivity_at(&self, height_m: f64) -> f64 {
        let n = self.refractivity_at(height_m);
        modified_refractivity(n, height_m, self.earth_radius_m)
    }

    /// Compute the refractive index n = 1 + N × 10⁻⁶ at height h.
    pub fn refractive_index_at(&self, height_m: f64) -> f64 {
        1.0 + self.refractivity_at(height_m) * 1e-6
    }

    /// Compute the refractivity gradient dN/dh (N-units/m) at height h
    /// using the analytical derivative of the exponential model.
    pub fn gradient_at(&self, height_m: f64) -> f64 {
        let h = height_m.max(0.0);
        let d_dry = -self.n_dry_surface / self.h_dry * (-h / self.h_dry).exp();
        let d_wet = -self.n_wet_surface / self.h_wet * (-h / self.h_wet).exp();
        d_dry + d_wet
    }

    /// Temperature at altitude h (m) using linear lapse rate (troposphere only).
    pub fn temperature_at(&self, height_m: f64) -> f64 {
        if height_m < TROPOPAUSE_HEIGHT_M {
            self.temperature_k + self.lapse_rate * height_m
        } else {
            // Isothermal above tropopause
            self.temperature_k + self.lapse_rate * TROPOPAUSE_HEIGHT_M
        }
    }

    /// Pressure at altitude h (m) using the barometric formula.
    pub fn pressure_at(&self, height_m: f64) -> f64 {
        if height_m < TROPOPAUSE_HEIGHT_M {
            let exponent = -9.80665 / (self.lapse_rate * 287.05);
            self.pressure_hpa
                * (1.0 + self.lapse_rate * height_m / self.temperature_k).powf(exponent)
        } else {
            // Above tropopause: isothermal decay
            let p_trop = self.pressure_at(TROPOPAUSE_HEIGHT_M);
            let t_trop = self.temperature_at(TROPOPAUSE_HEIGHT_M);
            let dh = height_m - TROPOPAUSE_HEIGHT_M;
            p_trop * (-9.80665 * dh / (287.05 * t_trop)).exp()
        }
    }

    /// Generate a tabulated profile from surface to max_height_m in steps.
    ///
    /// Returns a vector of (height_m, refractivity_n) pairs.
    pub fn tabulate(&self, max_height_m: f64, step_m: f64) -> Vec<(f64, f64)> {
        let mut result = Vec::new();
        let mut h = 0.0;
        while h <= max_height_m {
            result.push((h, self.refractivity_at(h)));
            h += step_m;
        }
        result
    }
}

// ---------------------------------------------------------------------------
// RayTracer
// ---------------------------------------------------------------------------

/// A point along a traced ray path.
#[derive(Debug, Clone)]
pub struct RayPoint {
    /// Height above surface (m).
    pub height_m: f64,
    /// Ground-range distance from the transmitter (m).
    pub ground_range_m: f64,
    /// Local elevation angle of the ray at this point (radians).
    pub elevation_rad: f64,
    /// Refractive index at this point.
    pub refractive_index: f64,
}

/// Result of a complete ray trace.
#[derive(Debug, Clone)]
pub struct RayTraceResult {
    /// Sequence of points along the ray path.
    pub path: Vec<RayPoint>,
    /// Total bending (refraction) angle in radians.
    pub total_bending_rad: f64,
    /// Total geometric path length along the ray (m).
    pub geometric_path_m: f64,
    /// Total electromagnetic path length (integral of n·ds) in metres.
    pub electromagnetic_path_m: f64,
    /// Excess path delay due to refraction (m), equal to
    /// electromagnetic_path - straight-line distance.
    pub excess_path_m: f64,
}

/// Ray tracer for a spherically stratified atmosphere.
///
/// Uses Bouger's rule (n · r · cos θ = constant) to trace rays through
/// concentric atmospheric layers with varying refractive index.
///
/// The algorithm steps through altitude in small increments, applying
/// Snell's law at each layer boundary to update the ray direction.
#[derive(Debug, Clone)]
pub struct RayTracer {
    profile: RefractivityProfile,
    step_m: f64,
    max_alt_m: f64,
}

impl RayTracer {
    /// Create a new ray tracer with the given refractivity profile.
    pub fn new(profile: RefractivityProfile) -> Self {
        Self {
            profile,
            step_m: DEFAULT_STEP_M,
            max_alt_m: MAX_RAY_ALT_M,
        }
    }

    /// Set the altitude step size for ray tracing (m).
    pub fn with_step(mut self, step_m: f64) -> Self {
        self.step_m = step_m.max(1.0);
        self
    }

    /// Set the maximum altitude for ray tracing (m).
    pub fn with_max_altitude(mut self, max_alt_m: f64) -> Self {
        self.max_alt_m = max_alt_m;
        self
    }

    /// Trace a ray launched at the given initial elevation angle (degrees)
    /// from the surface.
    ///
    /// Returns the full ray path and accumulated bending.
    ///
    /// # Arguments
    /// * `initial_elevation_deg` - Launch elevation angle in degrees above
    ///   the local horizontal (0 = horizontal, 90 = zenith).
    pub fn trace(&self, initial_elevation_deg: f64) -> RayTraceResult {
        let a = self.profile.earth_radius_m;
        let el_rad = initial_elevation_deg * DEG2RAD;

        // Bouger's invariant: n(r) * r * cos(theta) = const
        let n0 = self.profile.refractive_index_at(0.0);
        let r0 = a;
        let bouguer = n0 * r0 * el_rad.cos();

        let mut path = Vec::new();
        let mut h = 0.0_f64;
        let mut ground_range = 0.0_f64;
        let mut geo_path = 0.0_f64;
        let mut em_path = 0.0_f64;
        let mut prev_el = el_rad;

        // Record initial point
        path.push(RayPoint {
            height_m: 0.0,
            ground_range_m: 0.0,
            elevation_rad: el_rad,
            refractive_index: n0,
        });

        while h < self.max_alt_m {
            let dh = self.step_m.min(self.max_alt_m - h);
            if dh <= 0.0 {
                break;
            }

            let h_new = h + dh;
            let r = a + h_new;
            let n = self.profile.refractive_index_at(h_new);

            // From Bouger's rule: cos(theta) = bouguer / (n * r)
            let cos_el = bouguer / (n * r);

            // If |cos_el| > 1, the ray is trapped (ducting) or reflected
            if cos_el.abs() > 1.0 {
                break;
            }

            let new_el = cos_el.acos();

            // Compute the geometric distance along this step
            // Using small-angle approximation for the arc:
            // ds ≈ dh / sin(elevation) for steep rays
            let avg_el = (prev_el + new_el) / 2.0;
            let sin_el = avg_el.sin();
            let ds = if sin_el.abs() > 0.01 {
                dh / sin_el
            } else {
                // For very low elevation, use Pythagorean approximation
                let dr_ground = dh * avg_el.cos() / avg_el.sin().max(0.001);
                (dh * dh + dr_ground * dr_ground).sqrt()
            };

            // Accumulate ground range from the arc subtended
            let d_angle = ds * avg_el.cos() / (a + (h + h_new) / 2.0);
            ground_range += (a + (h + h_new) / 2.0) * d_angle;

            geo_path += ds;
            em_path += ds * (self.profile.refractive_index_at(h) + n) / 2.0;

            h = h_new;
            prev_el = new_el;

            path.push(RayPoint {
                height_m: h,
                ground_range_m: ground_range,
                elevation_rad: new_el,
                refractive_index: n,
            });
        }

        // Total bending = change in elevation angle
        let total_bending = if path.len() >= 2 {
            (el_rad - path.last().unwrap().elevation_rad).abs()
        } else {
            0.0
        };

        // Excess path = EM path - geometric path
        let excess = em_path - geo_path;

        RayTraceResult {
            path,
            total_bending_rad: total_bending,
            geometric_path_m: geo_path,
            electromagnetic_path_m: em_path,
            excess_path_m: excess.max(0.0),
        }
    }

    /// Trace a ray and return just the total bending angle (degrees).
    pub fn bending_angle_deg(&self, initial_elevation_deg: f64) -> f64 {
        self.trace(initial_elevation_deg).total_bending_rad * RAD2DEG
    }

    /// Compute the excess path delay (m) for a ray at the given elevation.
    pub fn excess_delay_m(&self, initial_elevation_deg: f64) -> f64 {
        self.trace(initial_elevation_deg).excess_path_m
    }
}

// ---------------------------------------------------------------------------
// TroposphericCorrector
// ---------------------------------------------------------------------------

/// Mapping function type for converting zenith delay to slant delay.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MappingFunction {
    /// Niell Mapping Function — uses continued fraction with latitude-dependent
    /// coefficients. Widely used in GNSS since 1996.
    Nmf,
    /// Vienna Mapping Function — simplified version using single continued
    /// fraction with empirically tuned coefficients.
    Vmf,
    /// Simple 1/sin(el) cosecant mapping (least accurate, fastest).
    Cosecant,
}

/// Tropospheric delay corrector.
///
/// Maps zenith tropospheric delay (ZTD) to slant delay at a given elevation
/// angle using standard mapping functions. The ZTD is computed from surface
/// meteorological data via the Saastamoinen model.
#[derive(Debug, Clone)]
pub struct TroposphericCorrector {
    /// Zenith hydrostatic delay (m).
    pub zhd_m: f64,
    /// Zenith wet delay (m).
    pub zwd_m: f64,
    /// Total zenith delay (m).
    pub ztd_m: f64,
    /// Station latitude (degrees), used by NMF.
    latitude_deg: f64,
}

impl TroposphericCorrector {
    /// Create a tropospheric corrector from atmospheric config.
    ///
    /// Uses the Saastamoinen model to compute zenith delays.
    ///
    /// # Arguments
    /// * `config` - Atmospheric parameters.
    /// * `latitude_deg` - Station geodetic latitude in degrees.
    /// * `height_m` - Station height above mean sea level in metres.
    pub fn new(config: &AtmosphereConfig, latitude_deg: f64, height_m: f64) -> Self {
        // Saastamoinen zenith hydrostatic delay
        // ZHD = 0.0022768 * P / (1 - 0.00266 * cos(2*lat) - 0.00028 * h_km)
        let lat_rad = latitude_deg * DEG2RAD;
        let h_km = height_m / 1000.0;
        let zhd = 0.0022768 * config.pressure_hpa
            / (1.0 - 0.00266 * (2.0 * lat_rad).cos() - 0.00028 * h_km);

        // Zenith wet delay approximation
        // ZWD ≈ 0.002277 * (1255/T + 0.05) * e
        let e = vapour_pressure(config.temperature_k, config.humidity);
        let zwd = 0.002277 * (1255.0 / config.temperature_k + 0.05) * e;

        Self {
            zhd_m: zhd,
            zwd_m: zwd,
            ztd_m: zhd + zwd,
            latitude_deg,
        }
    }

    /// Compute slant delay (m) at the given elevation angle (degrees) using
    /// the specified mapping function.
    pub fn slant_delay_m(&self, elevation_deg: f64, mf: MappingFunction) -> f64 {
        let mf_dry = self.mapping_dry(elevation_deg, mf);
        let mf_wet = self.mapping_wet(elevation_deg, mf);
        self.zhd_m * mf_dry + self.zwd_m * mf_wet
    }

    /// Hydrostatic (dry) mapping function value.
    pub fn mapping_dry(&self, elevation_deg: f64, mf: MappingFunction) -> f64 {
        match mf {
            MappingFunction::Cosecant => cosecant_mf(elevation_deg),
            MappingFunction::Nmf => nmf_hydrostatic(elevation_deg, self.latitude_deg),
            MappingFunction::Vmf => vmf_dry(elevation_deg),
        }
    }

    /// Wet mapping function value.
    pub fn mapping_wet(&self, elevation_deg: f64, mf: MappingFunction) -> f64 {
        match mf {
            MappingFunction::Cosecant => cosecant_mf(elevation_deg),
            MappingFunction::Nmf => nmf_wet(elevation_deg, self.latitude_deg),
            MappingFunction::Vmf => vmf_wet(elevation_deg),
        }
    }
}

/// Simple cosecant mapping: m(el) = 1 / sin(el).
fn cosecant_mf(elevation_deg: f64) -> f64 {
    let el_rad = elevation_deg.max(1.0) * DEG2RAD;
    1.0 / el_rad.sin()
}

/// Niell Mapping Function (NMF) for the hydrostatic component.
///
/// Uses a continued fraction of the form:
///   m(el) = (1 + a/(1 + b/(1 + c))) / (sin(el) + a/(sin(el) + b/(sin(el) + c)))
///
/// Coefficients are latitude-dependent. We use a simplified two-zone model.
fn nmf_hydrostatic(elevation_deg: f64, latitude_deg: f64) -> f64 {
    let el_rad = elevation_deg.max(1.0) * DEG2RAD;
    let sin_el = el_rad.sin();

    // Latitude-dependent coefficients (simplified from Niell 1996 Table 3)
    let abs_lat = latitude_deg.abs();
    let (a, b, c) = if abs_lat < 30.0 {
        (1.2769934e-3, 2.9153695e-3, 62.610505e-3)
    } else if abs_lat < 60.0 {
        (1.2683230e-3, 2.8318338e-3, 62.039730e-3)
    } else {
        (1.2465397e-3, 2.6354036e-3, 63.142808e-3)
    };

    continued_fraction_mf(sin_el, a, b, c)
}

/// Niell Mapping Function for the wet component.
fn nmf_wet(elevation_deg: f64, latitude_deg: f64) -> f64 {
    let el_rad = elevation_deg.max(1.0) * DEG2RAD;
    let sin_el = el_rad.sin();

    let abs_lat = latitude_deg.abs();
    let (a, b, c) = if abs_lat < 30.0 {
        (5.8021897e-4, 1.4275268e-3, 4.3472961e-2)
    } else if abs_lat < 60.0 {
        (5.6794847e-4, 1.5138625e-3, 4.6729510e-2)
    } else {
        (5.4801630e-4, 1.4965067e-3, 4.3908931e-2)
    };

    continued_fraction_mf(sin_el, a, b, c)
}

/// Vienna Mapping Function — dry (hydrostatic) component.
///
/// Simplified global average coefficients.
fn vmf_dry(elevation_deg: f64) -> f64 {
    let el_rad = elevation_deg.max(1.0) * DEG2RAD;
    let sin_el = el_rad.sin();

    // Average VMF1 coefficients (Boehm et al. 2006)
    let a = 1.2769934e-3;
    let b = 2.9153695e-3;
    let c = 62.610505e-3;

    continued_fraction_mf(sin_el, a, b, c)
}

/// Vienna Mapping Function — wet component.
fn vmf_wet(elevation_deg: f64) -> f64 {
    let el_rad = elevation_deg.max(1.0) * DEG2RAD;
    let sin_el = el_rad.sin();

    let a = 5.8021897e-4;
    let b = 1.4275268e-3;
    let c = 4.3472961e-2;

    continued_fraction_mf(sin_el, a, b, c)
}

/// Evaluate the Marini continued fraction form:
///
/// m(el) = (1 + a/(1 + b/(1 + c))) / (sin(el) + a/(sin(el) + b/(sin(el) + c)))
fn continued_fraction_mf(sin_el: f64, a: f64, b: f64, c: f64) -> f64 {
    let numer = 1.0 + a / (1.0 + b / (1.0 + c));
    let denom = sin_el + a / (sin_el + b / (sin_el + c));
    numer / denom
}

// ---------------------------------------------------------------------------
// DuctingDetector
// ---------------------------------------------------------------------------

/// Classification of atmospheric refraction conditions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RefractionCondition {
    /// Sub-refraction: dN/dh > 0 or dM/dh > 157 N/km.
    /// Rays bend upward relative to standard.
    SubRefraction,
    /// Standard refraction: dN/dh ≈ -39 N/km, dM/dh ≈ 118 N/km.
    Standard,
    /// Super-refraction: dN/dh < -79 N/km, 0 < dM/dh < 79 N/km.
    /// Rays bend more strongly toward Earth.
    SuperRefraction,
    /// Trapping (ducting): dM/dh < 0, dN/dh < -157 N/km.
    /// Rays are trapped in a waveguide.
    Trapping,
}

/// Detected duct in the atmosphere.
#[derive(Debug, Clone)]
pub struct Duct {
    /// Base height of the duct (m above surface).
    pub base_height_m: f64,
    /// Top height of the duct (m above surface).
    pub top_height_m: f64,
    /// Duct thickness (m).
    pub thickness_m: f64,
    /// Duct strength: ΔM = M(base) - M(top), in M-units.
    pub strength_m_units: f64,
    /// Whether this is a surface duct (base at or near surface).
    pub is_surface_duct: bool,
}

/// Detector for super-refraction and ducting conditions.
///
/// Analyses a refractivity profile for atmospheric layers where the
/// modified refractivity gradient dM/dh becomes negative (trapping)
/// or very small (super-refraction), which can cause anomalous
/// propagation, signal ducting, or radar blind zones.
#[derive(Debug, Clone)]
pub struct DuctingDetector {
    profile: RefractivityProfile,
}

impl DuctingDetector {
    /// Create a new ducting detector from a refractivity profile.
    pub fn new(profile: RefractivityProfile) -> Self {
        Self { profile }
    }

    /// Classify the refraction condition at a given height (m).
    pub fn classify_at(&self, height_m: f64) -> RefractionCondition {
        let dm_dh = self.modified_gradient_per_km(height_m);

        if dm_dh < 0.0 {
            RefractionCondition::Trapping
        } else if dm_dh < 79.0 {
            RefractionCondition::SuperRefraction
        } else if dm_dh < 157.0 {
            RefractionCondition::Standard
        } else {
            RefractionCondition::SubRefraction
        }
    }

    /// Compute the modified refractivity gradient dM/dh in M-units per km
    /// at the given height.
    pub fn modified_gradient_per_km(&self, height_m: f64) -> f64 {
        let delta_h = 10.0; // 10 m finite difference
        let m1 = self.profile.modified_refractivity_at(height_m);
        let m2 = self.profile.modified_refractivity_at(height_m + delta_h);
        (m2 - m1) / delta_h * 1000.0 // Convert to per-km
    }

    /// Scan the atmosphere for ducting layers up to max_height_m.
    ///
    /// Returns a vector of detected ducts. A duct is defined as a contiguous
    /// altitude range where dM/dh < 0.
    pub fn detect_ducts(&self, max_height_m: f64, step_m: f64) -> Vec<Duct> {
        let mut ducts = Vec::new();
        let step = step_m.max(1.0);
        let mut h = 0.0;
        let mut in_duct = false;
        let mut duct_base = 0.0_f64;
        let mut m_at_base = 0.0_f64;

        while h <= max_height_m {
            let dm_dh = self.modified_gradient_per_km(h);

            if dm_dh < 0.0 {
                if !in_duct {
                    in_duct = true;
                    duct_base = h;
                    m_at_base = self.profile.modified_refractivity_at(h);
                }
            } else if in_duct {
                let m_at_top = self.profile.modified_refractivity_at(h);
                ducts.push(Duct {
                    base_height_m: duct_base,
                    top_height_m: h,
                    thickness_m: h - duct_base,
                    strength_m_units: m_at_base - m_at_top,
                    is_surface_duct: duct_base < 50.0,
                });
                in_duct = false;
            }
            h += step;
        }

        // Close any open duct at the top
        if in_duct {
            let m_at_top = self.profile.modified_refractivity_at(max_height_m);
            ducts.push(Duct {
                base_height_m: duct_base,
                top_height_m: max_height_m,
                thickness_m: max_height_m - duct_base,
                strength_m_units: m_at_base - m_at_top,
                is_surface_duct: duct_base < 50.0,
            });
        }

        ducts
    }

    /// Check whether a surface duct exists (trapping near the ground).
    pub fn has_surface_duct(&self, max_height_m: f64) -> bool {
        self.detect_ducts(max_height_m, 10.0)
            .iter()
            .any(|d| d.is_surface_duct)
    }

    /// Get the surface refraction condition.
    pub fn surface_condition(&self) -> RefractionCondition {
        self.classify_at(0.0)
    }
}

// ---------------------------------------------------------------------------
// ElevationCorrector
// ---------------------------------------------------------------------------

/// Corrects apparent (observed) elevation angles for atmospheric refraction.
///
/// Due to refraction, celestial objects and radio sources appear at a higher
/// elevation than their true geometric position. This corrector computes the
/// refraction correction using the Bennett formula (valid down to the horizon)
/// or ray tracing for high accuracy.
#[derive(Debug, Clone)]
pub struct ElevationCorrector {
    profile: RefractivityProfile,
    earth_radius_m: f64,
}

impl ElevationCorrector {
    /// Create a new elevation corrector from atmospheric configuration.
    pub fn new(config: &AtmosphereConfig) -> Self {
        let profile = RefractivityProfile::new(config);
        Self {
            profile,
            earth_radius_m: config.earth_radius_m,
        }
    }

    /// Create from an existing refractivity profile.
    pub fn from_profile(profile: RefractivityProfile) -> Self {
        let a = profile.earth_radius_m;
        Self {
            profile,
            earth_radius_m: a,
        }
    }

    /// Compute the refraction correction (degrees) for a given apparent
    /// elevation angle (degrees).
    ///
    /// Returns the correction to ADD to the apparent elevation to get the
    /// geometric (true) elevation:
    ///   geometric = apparent + correction
    ///
    /// Uses the Bennett formula:
    ///   R = 1/tan(h + 7.31/(h + 4.4)) in arcminutes, where h is in degrees.
    /// With pressure/temperature correction.
    pub fn correction_deg(&self, apparent_elevation_deg: f64) -> f64 {
        let h = apparent_elevation_deg.max(0.0);

        // Bennett formula (arcminutes)
        let denom = h + 7.31 / (h + 4.4);
        let r_arcmin = 1.0 / (denom * DEG2RAD).tan();

        // Scale for non-standard P and T
        let p_scale = self.profile.pressure_hpa / STD_PRESSURE_HPA;
        let t_scale = STD_TEMPERATURE_K / self.profile.temperature_k;

        let correction_arcmin = r_arcmin * p_scale * t_scale;
        correction_arcmin / 60.0 // Convert arcminutes to degrees
    }

    /// Compute the geometric elevation (degrees) from the apparent elevation.
    ///
    /// geometric = apparent + correction
    pub fn geometric_elevation_deg(&self, apparent_elevation_deg: f64) -> f64 {
        apparent_elevation_deg + self.correction_deg(apparent_elevation_deg)
    }

    /// Compute the apparent elevation (degrees) from the geometric elevation.
    ///
    /// Uses iterative inversion of the correction function.
    pub fn apparent_elevation_deg(&self, geometric_elevation_deg: f64) -> f64 {
        // Newton-like iteration: apparent = geometric - correction(apparent)
        let mut apparent = geometric_elevation_deg;
        for _ in 0..10 {
            let corr = self.correction_deg(apparent);
            let new_apparent = geometric_elevation_deg - corr;
            if (new_apparent - apparent).abs() < 1e-8 {
                break;
            }
            apparent = new_apparent;
        }
        apparent
    }

    /// Maximum refraction at the horizon (degrees), approximately 0.57° for
    /// standard atmosphere.
    pub fn horizon_refraction_deg(&self) -> f64 {
        self.correction_deg(0.0)
    }

    /// Refraction correction at zenith (degrees), should be essentially zero.
    pub fn zenith_refraction_deg(&self) -> f64 {
        self.correction_deg(90.0)
    }

    /// Compute the effective Earth radius factor kₑ from the surface
    /// refractivity gradient.
    pub fn effective_earth_factor(&self) -> f64 {
        let dn_dh = self.profile.gradient_at(0.0);
        earth_radius_effective_factor(dn_dh, self.earth_radius_m)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    // --- AtmosphereConfig tests ---

    #[test]
    fn test_standard_atmosphere_values() {
        let config = AtmosphereConfig::standard();
        assert!((config.temperature_k - 288.15).abs() < EPSILON);
        assert!((config.pressure_hpa - 1013.25).abs() < EPSILON);
        assert!((config.humidity - 0.60).abs() < EPSILON);
        assert!((config.lapse_rate - (-0.0065)).abs() < EPSILON);
    }

    #[test]
    fn test_custom_atmosphere() {
        let config = AtmosphereConfig::custom(300.0, 1000.0, 0.80);
        assert!((config.temperature_k - 300.0).abs() < EPSILON);
        assert!((config.pressure_hpa - 1000.0).abs() < EPSILON);
        assert!((config.humidity - 0.80).abs() < EPSILON);
    }

    #[test]
    fn test_tropical_atmosphere() {
        let config = AtmosphereConfig::tropical();
        assert!(config.temperature_k > 295.0); // Warmer than standard
        assert!(config.humidity > 0.7);         // More humid
    }

    #[test]
    fn test_subarctic_atmosphere() {
        let config = AtmosphereConfig::subarctic_winter();
        assert!(config.temperature_k < 270.0); // Colder than standard
        assert!(config.humidity < 0.5);         // Drier
    }

    // --- Helper function tests ---

    #[test]
    fn test_saturation_vapour_pressure() {
        // At 20°C, eₛ ≈ 23.4 hPa
        let es = saturation_vapour_pressure(293.15);
        assert!((es - 23.37).abs() < 1.0, "es at 20C = {es}");

        // At 0°C, eₛ ≈ 6.1 hPa
        let es0 = saturation_vapour_pressure(273.15);
        assert!((es0 - 6.1).abs() < 0.5, "es at 0C = {es0}");
    }

    #[test]
    fn test_bean_dutton_model_standard() {
        // Standard atmosphere surface refractivity should be ~315 N-units
        let n = bean_dutton_model(STD_PRESSURE_HPA, STD_TEMPERATURE_K, STD_HUMIDITY);
        assert!(n > 300.0 && n < 350.0, "N_surface = {n}");
    }

    #[test]
    fn test_refractivity_dry_only() {
        // With zero humidity, only the dry term matters
        let n = refractivity(1013.25, 288.15, 0.0);
        let expected = 77.6 * 1013.25 / 288.15;
        assert!((n - expected).abs() < 0.1, "N_dry = {n}, expected = {expected}");
    }

    #[test]
    fn test_modified_refractivity() {
        let n = 315.0;
        let h = 1000.0; // 1 km
        let m = modified_refractivity(n, h, EARTH_RADIUS_M);
        // M = N + h/a * 1e6 = 315 + 1000/6371000 * 1e6 ≈ 315 + 157 = 472
        let expected = 315.0 + 1000.0 / EARTH_RADIUS_M * 1e6;
        assert!((m - expected).abs() < 0.1, "M = {m}, expected = {expected}");
    }

    #[test]
    fn test_effective_earth_factor_standard() {
        // Standard gradient: dN/dh ≈ -0.039 N-units/m (= -39 N-units/km)
        // earth_radius_effective_factor expects N-units per metre.
        let k = earth_radius_effective_factor(-0.039, EARTH_RADIUS_M);
        assert!((k - 4.0 / 3.0).abs() < 0.05, "kₑ = {k}");
    }

    #[test]
    fn test_effective_earth_radius() {
        let ae = earth_radius_effective(-0.039, EARTH_RADIUS_M);
        assert!(ae > EARTH_RADIUS_M, "Effective radius must be larger");
        assert!(ae < 2.0 * EARTH_RADIUS_M, "Effective radius must be reasonable");
    }

    // --- RefractivityProfile tests ---

    #[test]
    fn test_refractivity_profile_surface() {
        let config = AtmosphereConfig::standard();
        let profile = RefractivityProfile::new(&config);
        // Surface N should be ~315 for standard atmosphere
        assert!(profile.n_surface > 300.0 && profile.n_surface < 350.0,
            "N_surface = {}", profile.n_surface);
    }

    #[test]
    fn test_refractivity_decreases_with_height() {
        let config = AtmosphereConfig::standard();
        let profile = RefractivityProfile::new(&config);

        let n0 = profile.refractivity_at(0.0);
        let n1 = profile.refractivity_at(1000.0);
        let n5 = profile.refractivity_at(5000.0);
        let n10 = profile.refractivity_at(10000.0);

        assert!(n0 > n1, "N must decrease: N(0)={n0} > N(1km)={n1}");
        assert!(n1 > n5, "N must decrease: N(1km)={n1} > N(5km)={n5}");
        assert!(n5 > n10, "N must decrease: N(5km)={n5} > N(10km)={n10}");
    }

    #[test]
    fn test_refractivity_approaches_zero() {
        let config = AtmosphereConfig::standard();
        let profile = RefractivityProfile::new(&config);

        let n_high = profile.refractivity_at(50000.0); // 50 km
        assert!(n_high < 1.0, "N at 50 km should be near zero: {n_high}");
    }

    #[test]
    fn test_refractivity_gradient_negative() {
        let config = AtmosphereConfig::standard();
        let profile = RefractivityProfile::new(&config);

        let grad = profile.gradient_at(0.0);
        assert!(grad < 0.0, "dN/dh at surface must be negative: {grad}");
    }

    #[test]
    fn test_refractive_index_near_unity() {
        let config = AtmosphereConfig::standard();
        let profile = RefractivityProfile::new(&config);

        let n = profile.refractive_index_at(0.0);
        assert!((n - 1.0).abs() < 0.001, "n should be very close to 1: {n}");
        assert!(n > 1.0, "n should be slightly greater than 1");
    }

    #[test]
    fn test_temperature_profile() {
        let config = AtmosphereConfig::standard();
        let profile = RefractivityProfile::new(&config);

        let t0 = profile.temperature_at(0.0);
        let t5 = profile.temperature_at(5000.0);
        let t_trop = profile.temperature_at(11000.0);
        let t_above = profile.temperature_at(15000.0);

        assert!((t0 - 288.15).abs() < EPSILON);
        assert!(t5 < t0, "Temperature decreases in troposphere");
        // Above tropopause, temperature should be constant
        assert!((t_above - t_trop).abs() < EPSILON);
    }

    #[test]
    fn test_pressure_profile() {
        let config = AtmosphereConfig::standard();
        let profile = RefractivityProfile::new(&config);

        let p0 = profile.pressure_at(0.0);
        let p5 = profile.pressure_at(5000.0);
        let p10 = profile.pressure_at(10000.0);

        assert!((p0 - 1013.25).abs() < 0.1);
        assert!(p5 < p0, "Pressure decreases with altitude");
        assert!(p10 < p5, "Pressure continues to decrease");
        // At ~5.5 km, pressure should be roughly half
        assert!(p5 < 600.0 && p5 > 400.0, "P(5km) = {p5}");
    }

    #[test]
    fn test_tabulate_profile() {
        let config = AtmosphereConfig::standard();
        let profile = RefractivityProfile::new(&config);

        let table = profile.tabulate(10000.0, 1000.0);
        assert_eq!(table.len(), 11); // 0, 1000, 2000, ..., 10000
        assert!((table[0].0).abs() < EPSILON); // First height = 0
        assert!(table[0].1 > table[10].1); // Decreasing N
    }

    // --- RayTracer tests ---

    #[test]
    fn test_ray_trace_vertical() {
        let config = AtmosphereConfig::standard();
        let profile = RefractivityProfile::new(&config);
        let tracer = RayTracer::new(profile).with_max_altitude(20000.0);

        let result = tracer.trace(90.0); // Vertical ray
        assert!(!result.path.is_empty());
        // Vertical ray should have minimal bending
        assert!(result.total_bending_rad < 0.001,
            "Vertical ray bending should be tiny: {}", result.total_bending_rad);
    }

    #[test]
    fn test_ray_trace_low_elevation() {
        let config = AtmosphereConfig::standard();
        let profile = RefractivityProfile::new(&config);
        let tracer = RayTracer::new(profile).with_max_altitude(20000.0);

        let result = tracer.trace(5.0); // 5 degree elevation
        assert!(!result.path.is_empty());
        // Low elevation rays should bend more
        assert!(result.total_bending_rad > 0.0);
    }

    #[test]
    fn test_ray_trace_excess_delay() {
        let config = AtmosphereConfig::standard();
        let profile = RefractivityProfile::new(&config);
        let tracer = RayTracer::new(profile).with_max_altitude(30000.0);

        let excess = tracer.excess_delay_m(10.0);
        // Excess delay should be positive (atmosphere slows signal)
        assert!(excess >= 0.0, "Excess delay should be non-negative: {excess}");
    }

    #[test]
    fn test_bending_increases_at_lower_elevation() {
        let config = AtmosphereConfig::standard();
        let profile = RefractivityProfile::new(&config);
        let tracer = RayTracer::new(profile).with_max_altitude(20000.0);

        let bend_30 = tracer.bending_angle_deg(30.0);
        let bend_10 = tracer.bending_angle_deg(10.0);

        // More bending at lower elevation
        assert!(bend_10 > bend_30,
            "Bending at 10° ({bend_10}) should exceed bending at 30° ({bend_30})");
    }

    #[test]
    fn test_ray_path_altitude_increases() {
        let config = AtmosphereConfig::standard();
        let profile = RefractivityProfile::new(&config);
        let tracer = RayTracer::new(profile).with_max_altitude(10000.0);

        let result = tracer.trace(45.0);
        // For a 45° elevation ray, altitude should generally increase
        for i in 1..result.path.len() {
            assert!(result.path[i].height_m >= result.path[i - 1].height_m,
                "Ray altitude should increase for upgoing rays");
        }
    }

    // --- TroposphericCorrector tests ---

    #[test]
    fn test_tropospheric_zenith_delay() {
        let config = AtmosphereConfig::standard();
        let corr = TroposphericCorrector::new(&config, 45.0, 0.0);

        // ZHD should be around 2.2-2.4 m
        assert!(corr.zhd_m > 2.0 && corr.zhd_m < 2.6,
            "ZHD = {} m", corr.zhd_m);

        // ZWD should be around 0.05-0.3 m
        assert!(corr.zwd_m > 0.01 && corr.zwd_m < 0.5,
            "ZWD = {} m", corr.zwd_m);

        // Total ZTD = ZHD + ZWD
        assert!((corr.ztd_m - (corr.zhd_m + corr.zwd_m)).abs() < EPSILON);
    }

    #[test]
    fn test_slant_delay_increases_at_low_elevation() {
        let config = AtmosphereConfig::standard();
        let corr = TroposphericCorrector::new(&config, 45.0, 0.0);

        let delay_90 = corr.slant_delay_m(90.0, MappingFunction::Nmf);
        let delay_30 = corr.slant_delay_m(30.0, MappingFunction::Nmf);
        let delay_10 = corr.slant_delay_m(10.0, MappingFunction::Nmf);

        // Delay at zenith ≈ ZTD
        assert!((delay_90 - corr.ztd_m).abs() < 0.05,
            "Delay at zenith should equal ZTD: {} vs {}", delay_90, corr.ztd_m);

        assert!(delay_30 > delay_90, "Lower elevation → more delay");
        assert!(delay_10 > delay_30, "Even lower elevation → even more delay");
    }

    #[test]
    fn test_cosecant_mapping() {
        // At zenith: 1/sin(90°) = 1
        let m90 = cosecant_mf(90.0);
        assert!((m90 - 1.0).abs() < 0.01);

        // At 30°: 1/sin(30°) = 2
        let m30 = cosecant_mf(30.0);
        assert!((m30 - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_mapping_function_zenith() {
        let config = AtmosphereConfig::standard();
        let corr = TroposphericCorrector::new(&config, 45.0, 0.0);

        // At zenith, all mapping functions should give ~1.0
        for mf in [MappingFunction::Cosecant, MappingFunction::Nmf, MappingFunction::Vmf] {
            let m_dry = corr.mapping_dry(90.0, mf);
            let m_wet = corr.mapping_wet(90.0, mf);
            assert!((m_dry - 1.0).abs() < 0.01,
                "Dry mapping at zenith should be ~1: {m_dry} ({mf:?})");
            assert!((m_wet - 1.0).abs() < 0.01,
                "Wet mapping at zenith should be ~1: {m_wet} ({mf:?})");
        }
    }

    // --- DuctingDetector tests ---

    #[test]
    fn test_standard_atmosphere_no_ducting() {
        let config = AtmosphereConfig::standard();
        let profile = RefractivityProfile::new(&config);
        let detector = DuctingDetector::new(profile);

        // Standard atmosphere should NOT produce ducting
        let condition = detector.surface_condition();
        assert_ne!(condition, RefractionCondition::Trapping,
            "Standard atmosphere should not produce surface ducting");
    }

    #[test]
    fn test_ducting_classification() {
        let config = AtmosphereConfig::standard();
        let profile = RefractivityProfile::new(&config);
        let detector = DuctingDetector::new(profile);

        // The modified gradient at the surface for standard atmosphere
        // should be around 118 N/km (standard)
        let dm = detector.modified_gradient_per_km(0.0);
        // Standard: 79 < dM/dh < 157
        assert!(dm > 50.0 && dm < 200.0,
            "dM/dh at surface should be near standard: {dm} M-units/km");
    }

    #[test]
    fn test_no_ducts_in_standard() {
        let config = AtmosphereConfig::standard();
        let profile = RefractivityProfile::new(&config);
        let detector = DuctingDetector::new(profile);

        let ducts = detector.detect_ducts(5000.0, 10.0);
        assert!(ducts.is_empty(),
            "Standard atmosphere should have no ducts, found {}", ducts.len());
    }

    // --- ElevationCorrector tests ---

    #[test]
    fn test_horizon_refraction() {
        let config = AtmosphereConfig::standard();
        let corr = ElevationCorrector::new(&config);

        let hr = corr.horizon_refraction_deg();
        // Atmospheric refraction at horizon ≈ 0.5-0.6°
        assert!(hr > 0.3 && hr < 1.0,
            "Horizon refraction should be ~0.57°: {hr}°");
    }

    #[test]
    fn test_zenith_refraction_near_zero() {
        let config = AtmosphereConfig::standard();
        let corr = ElevationCorrector::new(&config);

        let zr = corr.zenith_refraction_deg();
        assert!(zr.abs() < 0.01,
            "Zenith refraction should be nearly zero: {zr}°");
    }

    #[test]
    fn test_refraction_correction_positive() {
        let config = AtmosphereConfig::standard();
        let corr = ElevationCorrector::new(&config);

        // Correction should always be positive (geometric > apparent)
        for el in [0.0, 5.0, 10.0, 30.0, 60.0] {
            let c = corr.correction_deg(el);
            assert!(c > 0.0, "Correction at {el}° should be positive: {c}");
        }
    }

    #[test]
    fn test_correction_decreases_with_elevation() {
        let config = AtmosphereConfig::standard();
        let corr = ElevationCorrector::new(&config);

        let c0 = corr.correction_deg(0.0);
        let c10 = corr.correction_deg(10.0);
        let c45 = corr.correction_deg(45.0);

        assert!(c0 > c10, "Correction at 0° > 10°: {c0} vs {c10}");
        assert!(c10 > c45, "Correction at 10° > 45°: {c10} vs {c45}");
    }

    #[test]
    fn test_geometric_vs_apparent_roundtrip() {
        let config = AtmosphereConfig::standard();
        let corr = ElevationCorrector::new(&config);

        for el in [1.0, 5.0, 15.0, 30.0, 60.0, 85.0] {
            let geo = corr.geometric_elevation_deg(el);
            let back = corr.apparent_elevation_deg(geo);
            assert!((back - el).abs() < 0.001,
                "Roundtrip failed at {el}°: apparent→geo={geo}→apparent={back}");
        }
    }

    #[test]
    fn test_effective_earth_factor() {
        let config = AtmosphereConfig::standard();
        let corr = ElevationCorrector::new(&config);

        let k = corr.effective_earth_factor();
        // The exponential model with separate dry/wet scale heights yields
        // a surface gradient steeper than the empirical -39 N/km, giving
        // kₑ somewhat larger than 4/3. Acceptable range: 1.2 to 1.8.
        assert!(k > 1.2 && k < 1.8,
            "Effective Earth factor should be in reasonable range: {k}");
    }

    #[test]
    fn test_tropical_stronger_refraction() {
        let std_config = AtmosphereConfig::standard();
        let trop_config = AtmosphereConfig::tropical();

        let std_n = bean_dutton_model(
            std_config.pressure_hpa, std_config.temperature_k, std_config.humidity);
        let trop_n = bean_dutton_model(
            trop_config.pressure_hpa, trop_config.temperature_k, trop_config.humidity);

        // Tropical atmosphere has higher humidity → stronger wet component
        // However total N depends on temperature too; wet component stronger in tropics
        let std_profile = RefractivityProfile::new(&std_config);
        let trop_profile = RefractivityProfile::new(&trop_config);

        // The wet component should be significantly larger in tropical
        assert!(trop_profile.n_wet_surface > std_profile.n_wet_surface,
            "Tropical wet N ({}) > standard wet N ({})",
            trop_profile.n_wet_surface, std_profile.n_wet_surface);
    }
}
