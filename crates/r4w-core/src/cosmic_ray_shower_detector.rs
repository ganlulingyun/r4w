//! Cosmic Ray Extensive Air Shower (EAS) Detector
//!
//! Signal processing for extensive air shower detection in astroparticle physics.
//! Implements the core algorithms used by ultra-high-energy cosmic ray observatories
//! such as Pierre Auger and Telescope Array.
//!
//! ## Physics Background
//!
//! When a cosmic ray (proton, helium, iron nucleus, etc.) enters the atmosphere
//! at ultra-high energies (10^15 - 10^20 eV), it initiates a cascade of secondary
//! particles called an extensive air shower (EAS). The shower develops through the
//! atmosphere, reaching maximum particle count at a depth X_max that depends on
//! primary energy and composition.
//!
//! Key observables:
//! - **Lateral distribution**: particle density vs distance from shower core (NKG function)
//! - **Arrival times**: plane-wave front timing for axis reconstruction
//! - **S(1000)**: signal at 1000m from core as energy proxy
//! - **X_max**: depth of shower maximum for composition studies
//! - **Muon content**: muon/electron ratio distinguishes light vs heavy primaries
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cosmic_ray_shower_detector::{
//!     ShowerConfig, CorePositionEstimator, EnergyEstimator,
//!     LateralDistributionFitter, ArrivalTimeReconstructor,
//! };
//!
//! let config = ShowerConfig::auger_default();
//!
//! // Station positions (x, y) in meters and measured signals
//! let stations = vec![(0.0, 0.0), (1500.0, 0.0), (0.0, 1500.0), (-1500.0, 0.0)];
//! let signals = vec![100.0, 25.0, 20.0, 30.0];
//!
//! // Estimate core position from signal distribution
//! let core_est = CorePositionEstimator::new();
//! let (cx, cy) = core_est.center_of_gravity(&stations, &signals);
//!
//! // Estimate energy from S(1000)
//! let energy_est = EnergyEstimator::new(config.clone());
//! let s1000 = 40.0; // VEM at 1000m
//! let energy_ev = energy_est.energy_from_s1000(s1000);
//! assert!(energy_ev > 1e18);
//! ```

use std::f64::consts::PI;

// ─── Speed of light ───────────────────────────────────────────────────────────
const C_LIGHT: f64 = 2.99792458e8; // m/s

// ─── Atmospheric constants ────────────────────────────────────────────────────
const X_VERTICAL: f64 = 1036.0; // Vertical atmospheric depth at sea level, g/cm²
const CRITICAL_ENERGY: f64 = 8.1e7; // Critical energy in eV (~81 MeV)
const RADIATION_LENGTH: f64 = 36.66; // Radiation length in air, g/cm²
const MOLIERE_RADIUS: f64 = 9.3e3; // Moliere radius at sea level, cm (~93 m)

// ─── ShowerConfig ─────────────────────────────────────────────────────────────

/// Configuration for a cosmic ray shower detector array.
#[derive(Debug, Clone)]
pub struct ShowerConfig {
    /// Station spacing in meters (e.g., 1500 m for Auger SD).
    pub station_spacing_m: f64,
    /// Number of stations in the array.
    pub num_stations: usize,
    /// Timing resolution per station in nanoseconds.
    pub timing_resolution_ns: f64,
    /// Array altitude above sea level in meters.
    pub altitude_m: f64,
    /// Reference distance for S(r_ref) energy estimator in meters.
    pub reference_distance_m: f64,
    /// NKG reference distance (Moliere radius) in meters.
    pub nkg_r_ref_m: f64,
    /// Atmospheric vertical depth at the array altitude (g/cm^2).
    pub vertical_depth: f64,
}

impl ShowerConfig {
    /// Default configuration matching Pierre Auger Observatory surface detector.
    ///
    /// - 1500 m triangular grid spacing
    /// - 1660 stations
    /// - ~12 ns GPS timing resolution
    /// - 1400 m altitude (Pampa Amarilla, Argentina)
    pub fn auger_default() -> Self {
        Self {
            station_spacing_m: 1500.0,
            num_stations: 1660,
            timing_resolution_ns: 12.0,
            altitude_m: 1400.0,
            reference_distance_m: 1000.0,
            nkg_r_ref_m: 93.0, // Moliere radius ~93 m at sea level
            vertical_depth: 875.0, // g/cm² at 1400m altitude
        }
    }

    /// Configuration for Telescope Array surface detector (Utah, USA).
    ///
    /// - 1200 m square grid spacing
    /// - 507 stations
    /// - ~20 ns timing resolution
    /// - 1400 m altitude
    pub fn telescope_array() -> Self {
        Self {
            station_spacing_m: 1200.0,
            num_stations: 507,
            timing_resolution_ns: 20.0,
            altitude_m: 1400.0,
            reference_distance_m: 800.0,
            nkg_r_ref_m: 93.0,
            vertical_depth: 875.0,
        }
    }

    /// Configuration for a dense infill array (lower-energy threshold).
    ///
    /// - 750 m spacing
    /// - 61 stations
    /// - 10 ns timing
    /// - 1400 m altitude
    pub fn auger_infill() -> Self {
        Self {
            station_spacing_m: 750.0,
            num_stations: 61,
            timing_resolution_ns: 10.0,
            altitude_m: 1400.0,
            reference_distance_m: 450.0,
            nkg_r_ref_m: 93.0,
            vertical_depth: 875.0,
        }
    }
}

// ─── Station data ─────────────────────────────────────────────────────────────

/// Measured data from a single surface detector station.
#[derive(Debug, Clone)]
pub struct StationData {
    /// Station position x-coordinate in meters.
    pub x: f64,
    /// Station position y-coordinate in meters.
    pub y: f64,
    /// Station altitude in meters (if different from array altitude).
    pub z: f64,
    /// Measured signal in VEM (Vertical Equivalent Muons).
    pub signal_vem: f64,
    /// Trigger time in nanoseconds relative to an arbitrary t0.
    pub time_ns: f64,
}

// ─── LateralDistributionFitter ────────────────────────────────────────────────

/// Fits the NKG (Nishimura-Kamata-Greisen) lateral distribution function
/// to measured ground-level particle densities.
///
/// The NKG function describes particle density as a function of distance
/// from the shower core:
///
///   S(r) = S_ref * (r / r_ref)^(s - 2) * (1 + r / r_ref)^(s - 4.5)
///
/// where:
/// - `s` is the shower age parameter (0 < s < 2, with s=1 at maximum)
/// - `r_ref` is the Moliere radius (~93 m at sea level)
/// - `S_ref` is the signal normalization
#[derive(Debug, Clone)]
pub struct LateralDistributionFitter {
    /// Moliere radius (reference distance) in meters.
    pub r_ref: f64,
}

impl LateralDistributionFitter {
    /// Create a new fitter with the given Moliere radius.
    pub fn new(r_ref: f64) -> Self {
        Self { r_ref }
    }

    /// Evaluate the NKG function at distance `r` from core.
    ///
    /// Returns S(r) = s_ref * (r/r_ref)^(s-2) * (1 + r/r_ref)^(s-4.5)
    pub fn nkg(&self, r: f64, s_ref: f64, age: f64) -> f64 {
        if r <= 0.0 {
            return 0.0;
        }
        let rr = r / self.r_ref;
        s_ref * rr.powf(age - 2.0) * (1.0 + rr).powf(age - 4.5)
    }

    /// Fit the NKG parameters (s_ref, age) to station data using
    /// iterative least-squares grid search.
    ///
    /// `distances` — distances from core to each station in meters.
    /// `signals` — measured signal at each station in VEM.
    ///
    /// Returns `(s_ref, age, chi2)`.
    pub fn fit(&self, distances: &[f64], signals: &[f64]) -> (f64, f64, f64) {
        assert_eq!(distances.len(), signals.len());
        assert!(!distances.is_empty());

        let mut best_s_ref = 1.0;
        let mut best_age = 1.0;
        let mut best_chi2 = f64::MAX;

        // Coarse grid search over age parameter
        let age_steps = 40;
        let s_ref_steps = 50;

        for ai in 0..age_steps {
            let age = 0.2 + (ai as f64) * 1.6 / (age_steps as f64);

            // For each age, find optimal s_ref analytically:
            // Minimize sum (signals[i] - s_ref * f(r_i, age))^2
            // => s_ref = sum(signals[i] * f_i) / sum(f_i^2)
            let mut num = 0.0;
            let mut den = 0.0;
            for (i, &r) in distances.iter().enumerate() {
                let fi = self.nkg(r, 1.0, age);
                num += signals[i] * fi;
                den += fi * fi;
            }

            if den < 1e-30 {
                continue;
            }

            let s_ref_opt = num / den;
            if s_ref_opt <= 0.0 {
                continue;
            }

            let chi2: f64 = distances
                .iter()
                .zip(signals.iter())
                .map(|(&r, &s)| {
                    let pred = self.nkg(r, s_ref_opt, age);
                    let diff = s - pred;
                    diff * diff / (s.max(1.0))
                })
                .sum();

            if chi2 < best_chi2 {
                best_chi2 = chi2;
                best_s_ref = s_ref_opt;
                best_age = age;
            }
        }

        // Refine with finer search around best
        let age_lo = (best_age - 0.1).max(0.1);
        let age_hi = (best_age + 0.1).min(1.9);
        for ai in 0..s_ref_steps {
            let age = age_lo + (ai as f64) * (age_hi - age_lo) / (s_ref_steps as f64);

            let mut num = 0.0;
            let mut den = 0.0;
            for (i, &r) in distances.iter().enumerate() {
                let fi = self.nkg(r, 1.0, age);
                num += signals[i] * fi;
                den += fi * fi;
            }

            if den < 1e-30 {
                continue;
            }

            let s_ref_opt = num / den;
            if s_ref_opt <= 0.0 {
                continue;
            }

            let chi2: f64 = distances
                .iter()
                .zip(signals.iter())
                .map(|(&r, &s)| {
                    let pred = self.nkg(r, s_ref_opt, age);
                    let diff = s - pred;
                    diff * diff / (s.max(1.0))
                })
                .sum();

            if chi2 < best_chi2 {
                best_chi2 = chi2;
                best_s_ref = s_ref_opt;
                best_age = age;
            }
        }

        (best_s_ref, best_age, best_chi2)
    }
}

// ─── ArrivalTimeReconstructor ─────────────────────────────────────────────────

/// Reconstructs the shower arrival direction from station trigger times
/// using a plane-wave front model.
///
/// The plane-wave model assumes:
///   t_i = t0 + (1/c) * (n_x * x_i + n_y * y_i)
///
/// where (n_x, n_y) is the projection of the shower axis unit vector
/// onto the ground plane, and c is the speed of light.
///
/// This is solved as a linear least-squares problem.
#[derive(Debug, Clone)]
pub struct ArrivalTimeReconstructor {
    _private: (),
}

impl ArrivalTimeReconstructor {
    /// Create a new arrival time reconstructor.
    pub fn new() -> Self {
        Self { _private: () }
    }

    /// Reconstruct shower direction from station positions and trigger times.
    ///
    /// Returns `(n_x, n_y, t0)` where `(n_x, n_y)` are direction cosines
    /// and `t0` is the reference time offset (ns).
    ///
    /// The plane-wave model: t_i = t0 + (1/c) * (n_x * x_i + n_y * y_i)
    ///
    /// We solve via least squares: minimize sum(t_i - t0 - (nx*xi + ny*yi)/c)^2
    pub fn reconstruct(
        &self,
        stations: &[(f64, f64)],
        times_ns: &[f64],
    ) -> (f64, f64, f64) {
        assert_eq!(stations.len(), times_ns.len());
        let n = stations.len();
        assert!(n >= 3, "Need at least 3 stations for plane-wave fit");

        // Convert times to seconds for c calculation
        let c_ns = C_LIGHT * 1e-9; // m/ns

        // Linear system: t_i = a + b*x_i + c*y_i
        // where a = t0, b = n_x/c_ns, c = n_y/c_ns
        // Normal equations: A^T A [a,b,c]^T = A^T t

        let mut ata = [[0.0f64; 3]; 3];
        let mut atb = [0.0f64; 3];

        for i in 0..n {
            let (xi, yi) = stations[i];
            let ti = times_ns[i];

            let row = [1.0, xi, yi];
            for r in 0..3 {
                for col in 0..3 {
                    ata[r][col] += row[r] * row[col];
                }
                atb[r] += row[r] * ti;
            }
        }

        // Solve 3x3 system using Cramer's rule
        let params = solve_3x3(&ata, &atb);

        let t0 = params[0];
        let nx = params[1] * c_ns; // direction cosine
        let ny = params[2] * c_ns;

        (nx, ny, t0)
    }

    /// Convert direction cosines (n_x, n_y) to zenith and azimuth angles.
    ///
    /// Returns `(zenith_rad, azimuth_rad)`.
    pub fn direction_to_angles(nx: f64, ny: f64) -> (f64, f64) {
        let n_perp = (nx * nx + ny * ny).sqrt();
        let zenith = n_perp.asin().min(PI / 2.0);
        let azimuth = ny.atan2(nx);
        (zenith, azimuth)
    }
}

/// Solve a 3x3 linear system Ax = b using Cramer's rule.
fn solve_3x3(a: &[[f64; 3]; 3], b: &[f64; 3]) -> [f64; 3] {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    if det.abs() < 1e-30 {
        return [0.0; 3];
    }

    let inv_det = 1.0 / det;

    let x0 = (b[0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (b[1] * a[2][2] - a[1][2] * b[2])
        + a[0][2] * (b[1] * a[2][1] - a[1][1] * b[2]))
        * inv_det;

    let x1 = (a[0][0] * (b[1] * a[2][2] - a[1][2] * b[2])
        - b[0] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * b[2] - b[1] * a[2][0]))
        * inv_det;

    let x2 = (a[0][0] * (a[1][1] * b[2] - b[1] * a[2][1])
        - a[0][1] * (a[1][0] * b[2] - b[1] * a[2][0])
        + b[0] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
        * inv_det;

    [x0, x1, x2]
}

// ─── CorePositionEstimator ────────────────────────────────────────────────────

/// Estimates the shower core position on the ground from the spatial
/// distribution of detector signals.
///
/// Two methods:
/// 1. **Center of gravity** — signal-weighted barycenter (fast, approximate)
/// 2. **Chi-squared minimization** — grid search around CoG (accurate)
#[derive(Debug, Clone)]
pub struct CorePositionEstimator {
    _private: (),
}

impl CorePositionEstimator {
    /// Create a new core position estimator.
    pub fn new() -> Self {
        Self { _private: () }
    }

    /// Signal-weighted center of gravity.
    ///
    /// Returns `(x_core, y_core)` in meters.
    pub fn center_of_gravity(
        &self,
        stations: &[(f64, f64)],
        signals: &[f64],
    ) -> (f64, f64) {
        assert_eq!(stations.len(), signals.len());
        let mut wx = 0.0;
        let mut wy = 0.0;
        let mut ws = 0.0;

        for (i, &(x, y)) in stations.iter().enumerate() {
            let w = signals[i];
            wx += w * x;
            wy += w * y;
            ws += w;
        }

        if ws <= 0.0 {
            return (0.0, 0.0);
        }

        (wx / ws, wy / ws)
    }

    /// Chi-squared minimization around center of gravity estimate.
    ///
    /// Uses the NKG function with given age parameter to predict signals
    /// and minimizes chi2 over a grid of candidate core positions.
    ///
    /// Returns `(x_core, y_core, chi2_min)`.
    pub fn chi2_minimization(
        &self,
        stations: &[(f64, f64)],
        signals: &[f64],
        fitter: &LateralDistributionFitter,
        age: f64,
        s_ref: f64,
        search_radius_m: f64,
        grid_steps: usize,
    ) -> (f64, f64, f64) {
        let (cx, cy) = self.center_of_gravity(stations, signals);

        let mut best_x = cx;
        let mut best_y = cy;
        let mut best_chi2 = f64::MAX;

        let step = 2.0 * search_radius_m / (grid_steps as f64);

        for ix in 0..=grid_steps {
            let xc = cx - search_radius_m + (ix as f64) * step;
            for iy in 0..=grid_steps {
                let yc = cy - search_radius_m + (iy as f64) * step;

                let chi2: f64 = stations
                    .iter()
                    .zip(signals.iter())
                    .map(|(&(sx, sy), &sig)| {
                        let r = ((sx - xc).powi(2) + (sy - yc).powi(2)).sqrt();
                        let r_safe = r.max(1.0); // avoid singularity at core
                        let predicted = fitter.nkg(r_safe, s_ref, age);
                        let diff = sig - predicted;
                        if sig > 0.0 {
                            diff * diff / sig
                        } else {
                            diff * diff
                        }
                    })
                    .sum();

                if chi2 < best_chi2 {
                    best_chi2 = chi2;
                    best_x = xc;
                    best_y = yc;
                }
            }
        }

        (best_x, best_y, best_chi2)
    }
}

// ─── EnergyEstimator ──────────────────────────────────────────────────────────

/// Estimates primary cosmic ray energy from ground-level observables.
///
/// The primary method uses S(1000) — the signal at 1000 m from the shower
/// core — as an energy proxy. The relationship is approximately:
///
///   E = A * S(1000)^B
///
/// where A and B are calibration constants (e.g., from fluorescence detector).
/// For Auger: A ~ 2.19e17 eV, B ~ 1.025 (CIC-corrected).
///
/// The signal at the reference distance is largely independent of zenith
/// angle effects (the "constant intensity cut" method).
#[derive(Debug, Clone)]
pub struct EnergyEstimator {
    /// Calibration constant A in eV.
    pub a_cal: f64,
    /// Calibration exponent B.
    pub b_cal: f64,
    /// Reference distance in meters.
    pub ref_distance_m: f64,
    /// Configuration reference.
    pub config: ShowerConfig,
}

impl EnergyEstimator {
    /// Create energy estimator with Auger-like calibration.
    pub fn new(config: ShowerConfig) -> Self {
        Self {
            a_cal: 2.19e17,
            b_cal: 1.025,
            ref_distance_m: config.reference_distance_m,
            config,
        }
    }

    /// Create with custom calibration constants.
    pub fn with_calibration(config: ShowerConfig, a_cal: f64, b_cal: f64) -> Self {
        Self {
            a_cal,
            b_cal,
            ref_distance_m: config.reference_distance_m,
            config,
        }
    }

    /// Estimate energy from S(1000) in VEM.
    ///
    /// E = A * S(1000)^B
    pub fn energy_from_s1000(&self, s1000: f64) -> f64 {
        self.a_cal * s1000.powf(self.b_cal)
    }

    /// Apply zenith angle correction (CIC method).
    ///
    /// S_38 = S(1000) / CIC(theta)
    /// CIC(theta) = 1 + a*x + b*x^2, x = cos^2(theta) - cos^2(38deg)
    pub fn cic_correction(&self, s1000: f64, zenith_rad: f64) -> f64 {
        let cos2_theta = zenith_rad.cos().powi(2);
        let cos2_38 = (38.0_f64.to_radians()).cos().powi(2);
        let x = cos2_theta - cos2_38;

        // Empirical CIC coefficients (Auger)
        let a_cic = 0.94;
        let b_cic = -1.21;
        let cic = 1.0 + a_cic * x + b_cic * x * x;

        if cic > 0.0 {
            s1000 / cic
        } else {
            s1000
        }
    }

    /// Interpolate signal at reference distance from NKG fit parameters.
    pub fn signal_at_reference(
        &self,
        fitter: &LateralDistributionFitter,
        s_ref: f64,
        age: f64,
    ) -> f64 {
        fitter.nkg(self.ref_distance_m, s_ref, age)
    }

    /// Estimate energy using Heitler model: E ~ N_max * E_c
    ///
    /// Simple model relating total shower size to primary energy.
    pub fn energy_from_heitler(n_max: f64) -> f64 {
        n_max * CRITICAL_ENERGY
    }
}

// ─── ZenithAngleCalculator ────────────────────────────────────────────────────

/// Calculates shower zenith and azimuth angles and atmospheric depth.
///
/// The slant depth traversed by the shower is:
///   X_slant = X_vertical / cos(theta)
///
/// where X_vertical is the vertical atmospheric depth at the observation
/// level and theta is the zenith angle.
#[derive(Debug, Clone)]
pub struct ZenithAngleCalculator {
    /// Vertical atmospheric depth at observation level (g/cm^2).
    pub vertical_depth: f64,
}

impl ZenithAngleCalculator {
    /// Create calculator for the given observation altitude.
    pub fn new(config: &ShowerConfig) -> Self {
        Self {
            vertical_depth: config.vertical_depth,
        }
    }

    /// Create with explicit vertical depth.
    pub fn with_depth(vertical_depth: f64) -> Self {
        Self { vertical_depth }
    }

    /// Calculate zenith and azimuth from direction cosines.
    ///
    /// Returns `(zenith_rad, azimuth_rad)`.
    pub fn from_direction_cosines(&self, nx: f64, ny: f64) -> (f64, f64) {
        ArrivalTimeReconstructor::direction_to_angles(nx, ny)
    }

    /// Calculate slant depth from zenith angle.
    ///
    /// X_slant = X_vertical / cos(theta) for theta < 90 deg.
    pub fn slant_depth(&self, zenith_rad: f64) -> f64 {
        let cos_theta = zenith_rad.cos();
        if cos_theta > 0.01 {
            self.vertical_depth / cos_theta
        } else {
            // Near-horizontal showers: use curved atmosphere approximation
            // X ~ X_v * sec(theta) is inaccurate; cap at a large value
            self.vertical_depth * 100.0
        }
    }

    /// Estimate atmospheric depth at a given altitude using barometric formula.
    ///
    /// X(h) = X_0 * exp(-h / H), H ~ 8.4 km scale height
    pub fn depth_at_altitude(altitude_m: f64) -> f64 {
        let h_scale = 8400.0; // scale height in meters
        X_VERTICAL * (-altitude_m / h_scale).exp()
    }

    /// Compute the sec(theta) attenuation factor for inclined showers.
    pub fn sec_theta(zenith_rad: f64) -> f64 {
        let cos_theta = zenith_rad.cos();
        if cos_theta.abs() > 1e-6 {
            1.0 / cos_theta
        } else {
            1e6
        }
    }
}

// ─── ShowerAgeParameter ───────────────────────────────────────────────────────

/// Computes the shower age parameter from longitudinal profile observables.
///
/// The shower age `s` describes the evolutionary stage:
/// - s < 1: shower still growing
/// - s = 1: at shower maximum
/// - s > 1: shower declining
///
/// From the Gaisser-Hillas longitudinal profile:
///   N(X) = N_max * ((X - X0) / (X_max - X0))^((X_max - X0) / lambda) * exp((X_max - X) / lambda)
///
/// The age at depth X is approximately:
///   s = 3 * X / (X + 2 * X_max)
#[derive(Debug, Clone)]
pub struct ShowerAgeParameter {
    _private: (),
}

impl ShowerAgeParameter {
    pub fn new() -> Self {
        Self { _private: () }
    }

    /// Compute shower age at atmospheric depth X given X_max.
    ///
    /// s = 3 * X / (X + 2 * X_max)
    ///
    /// Returns a value between 0 and 3 (typically 0.5--1.5 at ground).
    pub fn age_at_depth(&self, x: f64, x_max: f64) -> f64 {
        if x_max <= 0.0 || x <= 0.0 {
            return 0.0;
        }
        3.0 * x / (x + 2.0 * x_max)
    }

    /// Estimate X_max from primary energy using Heitler model.
    ///
    /// X_max = X0 + lambda * ln(E / E_c)
    ///
    /// where X0 is the first interaction depth and lambda is the radiation length.
    pub fn xmax_from_energy(&self, energy_ev: f64) -> f64 {
        if energy_ev <= CRITICAL_ENERGY {
            return 0.0;
        }
        // X0 ~ 0 for electromagnetic cascades; use radiation length as scale
        RADIATION_LENGTH * (energy_ev / CRITICAL_ENERGY).ln()
    }

    /// Estimate X_max for proton primary at given energy.
    ///
    /// X_max^p ~ 750 + 58 * log10(E/10^18 eV) g/cm²
    pub fn xmax_proton(&self, energy_ev: f64) -> f64 {
        750.0 + 58.0 * (energy_ev / 1e18).log10()
    }

    /// Estimate X_max for iron primary at given energy.
    ///
    /// X_max^Fe ~ 690 + 58 * log10(E/10^18 eV) g/cm²
    /// (Iron showers develop earlier by ~60 g/cm² due to superposition)
    pub fn xmax_iron(&self, energy_ev: f64) -> f64 {
        690.0 + 58.0 * (energy_ev / 1e18).log10()
    }

    /// Compute the Gaisser-Hillas longitudinal profile.
    ///
    /// N(X) = N_max * ((X - X0) / (X_max - X0))^((X_max - X0) / lambda) * exp((X_max - X) / lambda)
    ///
    /// Returns particle number at slant depth X.
    pub fn gaisser_hillas(
        &self,
        x: f64,
        n_max: f64,
        x_max: f64,
        x0: f64,
        lambda: f64,
    ) -> f64 {
        if x <= x0 || lambda <= 0.0 || x_max <= x0 {
            return 0.0;
        }

        let dx = x - x0;
        let dx_max = x_max - x0;
        let exponent = dx_max / lambda;

        // N_max * ((X-X0)/(Xmax-X0))^((Xmax-X0)/lambda) * exp((Xmax-X)/lambda)
        let ratio = dx / dx_max;
        if ratio <= 0.0 {
            return 0.0;
        }

        n_max * ratio.powf(exponent) * ((x_max - x) / lambda).exp()
    }

    /// Compute the elongation rate: dX_max/dlog10(E).
    ///
    /// For electromagnetic showers: D_10 ~ 58 g/cm² per decade.
    /// For hadronic showers: D_10 ~ 58 g/cm² per decade (proton), less for nuclei.
    pub fn elongation_rate(x_max_low: f64, x_max_high: f64, log10_e_low: f64, log10_e_high: f64) -> f64 {
        if (log10_e_high - log10_e_low).abs() < 1e-10 {
            return 0.0;
        }
        (x_max_high - x_max_low) / (log10_e_high - log10_e_low)
    }
}

// ─── MuonContentAnalyzer ──────────────────────────────────────────────────────

/// Analyzes muon content of air showers for composition studies.
///
/// Heavier primaries (iron) produce more muons relative to electrons
/// than lighter primaries (protons) at the same energy:
///
///   N_mu ~ A^(1-alpha) * (E/A)^alpha
///
/// where A is the mass number and alpha ~ 0.9.
///
/// The muon-to-electron ratio is a key composition discriminant.
#[derive(Debug, Clone)]
pub struct MuonContentAnalyzer {
    /// Muon production exponent alpha (typically ~0.9).
    pub alpha: f64,
}

impl MuonContentAnalyzer {
    /// Create analyzer with default alpha = 0.9.
    pub fn new() -> Self {
        Self { alpha: 0.9 }
    }

    /// Create with custom alpha exponent.
    pub fn with_alpha(alpha: f64) -> Self {
        Self { alpha }
    }

    /// Predict muon number for given primary energy and mass number.
    ///
    /// N_mu ~ A^(1-alpha) * (E/A)^alpha
    ///
    /// Normalized so that proton at 10^19 eV gives ~10^7 muons.
    pub fn predicted_muon_number(&self, energy_ev: f64, mass_number: f64) -> f64 {
        let n_mu_ref = 1e7; // Reference: proton at 10^19 eV
        let e_ref = 1e19;

        let proton_scaling = (energy_ev / e_ref).powf(self.alpha);
        let mass_scaling = mass_number.powf(1.0 - self.alpha);

        n_mu_ref * proton_scaling * mass_scaling
    }

    /// Compute the muon-to-electron ratio.
    ///
    /// Higher ratio indicates heavier primary composition.
    pub fn muon_electron_ratio(n_muon: f64, n_electron: f64) -> f64 {
        if n_electron > 0.0 {
            n_muon / n_electron
        } else {
            0.0
        }
    }

    /// Estimate primary mass number from observed muon excess.
    ///
    /// Given N_mu_obs at energy E and the expected N_mu for proton at that energy,
    /// infer the mass number A:
    ///
    /// A ~ (N_mu_obs / N_mu_proton)^(1/(1-alpha))
    pub fn infer_mass_number(&self, n_mu_observed: f64, n_mu_proton: f64) -> f64 {
        if n_mu_proton <= 0.0 || n_mu_observed <= 0.0 {
            return 1.0;
        }
        let ratio = n_mu_observed / n_mu_proton;
        ratio.powf(1.0 / (1.0 - self.alpha))
    }

    /// Classify composition from X_max measurement.
    ///
    /// Returns a string label based on where X_max falls between
    /// proton and iron predictions.
    pub fn classify_composition(x_max_measured: f64, x_max_proton: f64, x_max_iron: f64) -> &'static str {
        if x_max_proton <= x_max_iron {
            return "unknown";
        }
        let frac = (x_max_proton - x_max_measured) / (x_max_proton - x_max_iron);
        if frac < 0.2 {
            "proton-like"
        } else if frac < 0.4 {
            "helium-like"
        } else if frac < 0.6 {
            "CNO-like"
        } else if frac < 0.8 {
            "silicon-like"
        } else {
            "iron-like"
        }
    }
}

// ─── FluorescenceProfiler ─────────────────────────────────────────────────────

/// Processes fluorescence detector data to reconstruct the longitudinal
/// shower profile.
///
/// Fluorescence detectors observe UV light emitted by nitrogen molecules
/// excited by shower particles. The light yield is proportional to the
/// energy deposited:
///
///   dE/dX = alpha_f * N_ch(X)
///
/// where N_ch is the charged particle number and alpha_f ~ 2.2 MeV/(g/cm²)
/// is the mean energy loss per unit atmospheric depth.
///
/// The profile is fitted with the Gaisser-Hillas function to extract
/// N_max and X_max.
#[derive(Debug, Clone)]
pub struct FluorescenceProfiler {
    /// Fluorescence yield in photons per MeV deposited.
    pub yield_photons_per_mev: f64,
    /// Mean energy loss per g/cm² in MeV.
    pub alpha_f: f64,
}

impl FluorescenceProfiler {
    /// Create profiler with standard atmospheric fluorescence yield.
    pub fn new() -> Self {
        Self {
            yield_photons_per_mev: 4.0, // ~4 photons/MeV at 337 nm
            alpha_f: 2.2,               // MeV/(g/cm²)
        }
    }

    /// Convert observed photon counts at each depth to energy deposit profile.
    ///
    /// `photon_counts` — measured photons at each slant depth bin.
    /// `depths` — slant depth values in g/cm².
    /// `distance_m` — distance from detector to shower axis.
    ///
    /// Returns dE/dX profile in MeV/(g/cm²).
    pub fn photons_to_energy_deposit(
        &self,
        photon_counts: &[f64],
        distance_m: f64,
    ) -> Vec<f64> {
        // Geometric correction: photons spread as 1/(4*pi*r^2)
        let geom_factor = 4.0 * PI * distance_m * distance_m;

        photon_counts
            .iter()
            .map(|&n| {
                let corrected = n * geom_factor;
                corrected / self.yield_photons_per_mev
            })
            .collect()
    }

    /// Fit Gaisser-Hillas profile to energy deposit data.
    ///
    /// Returns `(n_max, x_max, x0, lambda, chi2)`.
    pub fn fit_gaisser_hillas(
        &self,
        depths: &[f64],
        de_dx: &[f64],
    ) -> (f64, f64, f64, f64, f64) {
        assert_eq!(depths.len(), de_dx.len());
        assert!(!depths.is_empty());

        let age_calc = ShowerAgeParameter::new();

        // Find approximate maximum
        let mut max_idx = 0;
        let mut max_val = 0.0_f64;
        for (i, &val) in de_dx.iter().enumerate() {
            if val > max_val {
                max_val = val;
                max_idx = i;
            }
        }

        let n_max_est = max_val / self.alpha_f;
        let x_max_est = depths[max_idx];

        // Grid search over X0 and lambda
        let mut best = (n_max_est, x_max_est, 0.0, RADIATION_LENGTH, f64::MAX);

        for x0_step in 0..20 {
            let x0 = -100.0 + (x0_step as f64) * 10.0;
            if x0 >= x_max_est {
                continue;
            }

            for lam_step in 1..30 {
                let lambda = 20.0 + (lam_step as f64) * 5.0;

                // Find best n_max for this x0, lambda
                let mut num = 0.0;
                let mut den = 0.0;
                for (i, &x) in depths.iter().enumerate() {
                    let gh_unit = age_calc.gaisser_hillas(x, 1.0, x_max_est, x0, lambda);
                    let obs = de_dx[i] / self.alpha_f;
                    num += obs * gh_unit;
                    den += gh_unit * gh_unit;
                }

                if den < 1e-30 {
                    continue;
                }
                let n_max_opt = num / den;
                if n_max_opt <= 0.0 {
                    continue;
                }

                let chi2: f64 = depths
                    .iter()
                    .zip(de_dx.iter())
                    .map(|(&x, &obs)| {
                        let pred =
                            self.alpha_f * age_calc.gaisser_hillas(x, n_max_opt, x_max_est, x0, lambda);
                        let diff = obs - pred;
                        diff * diff / (obs.abs().max(1.0))
                    })
                    .sum();

                if chi2 < best.4 {
                    best = (n_max_opt, x_max_est, x0, lambda, chi2);
                }
            }
        }

        best
    }

    /// Calculate calorimetric energy from longitudinal profile integral.
    ///
    /// E_cal = integral(dE/dX dX) over the full profile.
    /// Uses trapezoidal integration.
    pub fn calorimetric_energy(depths: &[f64], de_dx: &[f64]) -> f64 {
        assert_eq!(depths.len(), de_dx.len());
        if depths.len() < 2 {
            return 0.0;
        }

        let mut integral = 0.0;
        for i in 1..depths.len() {
            let dx = depths[i] - depths[i - 1];
            integral += 0.5 * (de_dx[i] + de_dx[i - 1]) * dx;
        }

        // Convert MeV to eV
        integral * 1e6
    }
}

// ─── CherenkovDetector ────────────────────────────────────────────────────────

/// Analyzes Cherenkov light pool from extensive air showers.
///
/// Relativistic shower particles emit Cherenkov radiation when traveling
/// faster than light in the atmosphere. The Cherenkov angle is:
///
///   cos(theta_c) = 1 / (n * beta)
///
/// where n ~ 1.000293 at sea level. The Cherenkov light pool on the ground
/// extends to ~130 m radius for vertical showers, providing information
/// about shower development.
#[derive(Debug, Clone)]
pub struct CherenkovDetector {
    /// Refractive index at observation level.
    pub n_refract: f64,
}

impl CherenkovDetector {
    /// Create detector with sea-level refractive index.
    pub fn new() -> Self {
        Self {
            n_refract: 1.000293,
        }
    }

    /// Create with custom refractive index (altitude-dependent).
    ///
    /// Approximate: n(h) - 1 ~ (n_0 - 1) * exp(-h / H)
    pub fn at_altitude(altitude_m: f64) -> Self {
        let n0_minus_1 = 0.000293;
        let h_scale = 8400.0;
        let n = 1.0 + n0_minus_1 * (-altitude_m / h_scale).exp();
        Self { n_refract: n }
    }

    /// Compute the Cherenkov angle in radians for a particle with
    /// velocity beta*c in medium with refractive index n.
    ///
    /// cos(theta_c) = 1 / (n * beta)
    pub fn cherenkov_angle(&self, beta: f64) -> f64 {
        let cos_theta = 1.0 / (self.n_refract * beta);
        if cos_theta.abs() <= 1.0 {
            cos_theta.acos()
        } else {
            0.0 // Below Cherenkov threshold
        }
    }

    /// Cherenkov threshold: minimum beta for emission.
    ///
    /// beta_threshold = 1 / n
    pub fn threshold_beta(&self) -> f64 {
        1.0 / self.n_refract
    }

    /// Threshold kinetic energy for a particle of given mass (MeV/c²).
    ///
    /// gamma_threshold = 1 / sqrt(1 - 1/n^2)
    /// T_threshold = (gamma - 1) * m * c^2
    pub fn threshold_energy_mev(&self, mass_mev: f64) -> f64 {
        let n2 = self.n_refract * self.n_refract;
        let gamma = 1.0 / (1.0 - 1.0 / n2).sqrt();
        (gamma - 1.0) * mass_mev
    }

    /// Number of Cherenkov photons per unit path length per unit wavelength.
    ///
    /// dN/dxdlambda = (2*pi*alpha) / lambda^2 * sin^2(theta_c)
    ///
    /// Integrated over visible range (300-600 nm), approximately:
    /// dN/dx ~ 490 * sin^2(theta_c) photons/cm for ultra-relativistic particles.
    pub fn photons_per_cm(&self, beta: f64) -> f64 {
        let cos_theta = 1.0 / (self.n_refract * beta);
        if cos_theta.abs() > 1.0 {
            return 0.0; // Below threshold
        }
        let sin2_theta = 1.0 - cos_theta * cos_theta;

        // Frank-Tamm formula integrated over 300-600 nm
        // 2*pi*alpha * (1/lambda1 - 1/lambda2) * sin^2(theta_c)
        // alpha ~ 1/137, lambda in cm
        let alpha_em = 1.0 / 137.036;
        let lambda1 = 300e-7; // 300 nm in cm
        let lambda2 = 600e-7; // 600 nm in cm
        2.0 * PI * alpha_em * (1.0 / lambda1 - 1.0 / lambda2) * sin2_theta
    }

    /// Estimate lateral distribution of Cherenkov photons on the ground.
    ///
    /// Simple model: photon density ~ Q * exp(-r / r0) for r < r_ring,
    /// with a ring enhancement at the Cherenkov pool edge.
    ///
    /// Returns photon density at distance `r` (meters) from shower axis.
    pub fn lateral_photon_density(&self, r: f64, total_photons: f64, r0: f64) -> f64 {
        if r < 0.0 || r0 <= 0.0 {
            return 0.0;
        }
        // Exponential core + ring at ~120 m
        let r_ring = 120.0;
        let ring_width = 20.0;

        let core_density = total_photons / (2.0 * PI * r0 * r0) * (-r / r0).exp();
        let ring_factor = 1.0 + 0.5 * (-((r - r_ring) / ring_width).powi(2)).exp();

        core_density * ring_factor
    }
}

// ─── Helper: simple 2D vector distance ────────────────────────────────────────

/// Compute Euclidean distance between two 2D points.
#[inline]
pub fn distance_2d(x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    ((x2 - x1).powi(2) + (y2 - y1).powi(2)).sqrt()
}

// ─── Complete shower analysis pipeline ────────────────────────────────────────

/// Result of a full shower analysis.
#[derive(Debug, Clone)]
pub struct ShowerAnalysisResult {
    /// Estimated core position (x, y) in meters.
    pub core_x: f64,
    pub core_y: f64,
    /// Zenith angle in radians.
    pub zenith_rad: f64,
    /// Azimuth angle in radians.
    pub azimuth_rad: f64,
    /// Shower age parameter s.
    pub age: f64,
    /// NKG normalization S_ref.
    pub s_ref: f64,
    /// Signal at reference distance (VEM).
    pub s_ref_dist: f64,
    /// Estimated energy in eV.
    pub energy_ev: f64,
    /// Fit chi-squared.
    pub chi2: f64,
}

/// Run a complete shower analysis on station data.
///
/// This combines core position estimation, arrival direction reconstruction,
/// lateral distribution fitting, and energy estimation.
pub fn analyze_shower(config: &ShowerConfig, stations: &[StationData]) -> ShowerAnalysisResult {
    let positions: Vec<(f64, f64)> = stations.iter().map(|s| (s.x, s.y)).collect();
    let signals: Vec<f64> = stations.iter().map(|s| s.signal_vem).collect();
    let times: Vec<f64> = stations.iter().map(|s| s.time_ns).collect();

    // 1. Core position (center of gravity)
    let core_est = CorePositionEstimator::new();
    let (core_x, core_y) = core_est.center_of_gravity(&positions, &signals);

    // 2. Arrival direction
    let time_recon = ArrivalTimeReconstructor::new();
    let (nx, ny, _t0) = if positions.len() >= 3 {
        time_recon.reconstruct(&positions, &times)
    } else {
        (0.0, 0.0, 0.0)
    };
    let (zenith_rad, azimuth_rad) = ArrivalTimeReconstructor::direction_to_angles(nx, ny);

    // 3. Lateral distribution fit
    let distances: Vec<f64> = positions
        .iter()
        .map(|&(x, y)| distance_2d(x, y, core_x, core_y).max(1.0))
        .collect();
    let fitter = LateralDistributionFitter::new(config.nkg_r_ref_m);
    let (s_ref, age, chi2) = fitter.fit(&distances, &signals);

    // 4. Energy estimation
    let energy_est = EnergyEstimator::new(config.clone());
    let s_ref_dist = energy_est.signal_at_reference(&fitter, s_ref, age);
    let energy_ev = energy_est.energy_from_s1000(s_ref_dist);

    ShowerAnalysisResult {
        core_x,
        core_y,
        zenith_rad,
        azimuth_rad,
        age,
        s_ref,
        s_ref_dist,
        energy_ev,
        chi2,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    // ── ShowerConfig tests ────────────────────────────────────────────

    #[test]
    fn test_auger_config_defaults() {
        let cfg = ShowerConfig::auger_default();
        assert_eq!(cfg.station_spacing_m, 1500.0);
        assert_eq!(cfg.num_stations, 1660);
        assert!((cfg.timing_resolution_ns - 12.0).abs() < TOL);
        assert_eq!(cfg.altitude_m, 1400.0);
        assert_eq!(cfg.reference_distance_m, 1000.0);
    }

    #[test]
    fn test_telescope_array_config() {
        let cfg = ShowerConfig::telescope_array();
        assert_eq!(cfg.station_spacing_m, 1200.0);
        assert_eq!(cfg.num_stations, 507);
        assert_eq!(cfg.reference_distance_m, 800.0);
    }

    #[test]
    fn test_auger_infill_config() {
        let cfg = ShowerConfig::auger_infill();
        assert_eq!(cfg.station_spacing_m, 750.0);
        assert_eq!(cfg.num_stations, 61);
    }

    // ── NKG lateral distribution tests ────────────────────────────────

    #[test]
    fn test_nkg_decreases_with_distance() {
        let fitter = LateralDistributionFitter::new(93.0);
        let s_near = fitter.nkg(100.0, 10.0, 1.0);
        let s_far = fitter.nkg(1000.0, 10.0, 1.0);
        assert!(s_near > s_far, "Signal should decrease with distance");
        assert!(s_near > 0.0);
        assert!(s_far > 0.0);
    }

    #[test]
    fn test_nkg_zero_at_origin() {
        let fitter = LateralDistributionFitter::new(93.0);
        let s = fitter.nkg(0.0, 10.0, 1.0);
        assert!((s - 0.0).abs() < TOL);
    }

    #[test]
    fn test_nkg_scales_with_s_ref() {
        let fitter = LateralDistributionFitter::new(93.0);
        let s1 = fitter.nkg(500.0, 10.0, 1.0);
        let s2 = fitter.nkg(500.0, 20.0, 1.0);
        assert!((s2 / s1 - 2.0).abs() < TOL, "NKG should scale linearly with s_ref");
    }

    #[test]
    fn test_nkg_age_effect() {
        let fitter = LateralDistributionFitter::new(93.0);
        // Younger shower (s < 1) should be steeper (more particles near core)
        let young_near = fitter.nkg(50.0, 10.0, 0.5);
        let young_far = fitter.nkg(500.0, 10.0, 0.5);
        let old_near = fitter.nkg(50.0, 10.0, 1.5);
        let old_far = fitter.nkg(500.0, 10.0, 1.5);

        let ratio_young = young_near / young_far;
        let ratio_old = old_near / old_far;
        assert!(ratio_young > ratio_old, "Younger showers should be steeper");
    }

    #[test]
    fn test_nkg_fit_recovers_parameters() {
        let fitter = LateralDistributionFitter::new(93.0);
        let true_s_ref = 50.0;
        let true_age = 1.2;

        let distances = vec![100.0, 200.0, 400.0, 600.0, 800.0, 1000.0, 1500.0];
        let signals: Vec<f64> = distances
            .iter()
            .map(|&r| fitter.nkg(r, true_s_ref, true_age))
            .collect();

        let (fit_s_ref, fit_age, chi2) = fitter.fit(&distances, &signals);

        assert!(
            (fit_age - true_age).abs() < 0.1,
            "Fitted age {} should be close to true age {}", fit_age, true_age
        );
        assert!(chi2 < 1.0, "Chi2 should be small for perfect data");
    }

    // ── Arrival time reconstruction tests ─────────────────────────────

    #[test]
    fn test_vertical_shower_zero_delays() {
        let recon = ArrivalTimeReconstructor::new();
        // Vertical shower: all stations trigger at the same time
        let stations = vec![(0.0, 0.0), (1000.0, 0.0), (0.0, 1000.0), (1000.0, 1000.0)];
        let times = vec![100.0, 100.0, 100.0, 100.0];

        let (nx, ny, _t0) = recon.reconstruct(&stations, &times);
        assert!(nx.abs() < 0.01, "Vertical shower should have nx~0, got {}", nx);
        assert!(ny.abs() < 0.01, "Vertical shower should have ny~0, got {}", ny);
    }

    #[test]
    fn test_inclined_shower_direction() {
        let recon = ArrivalTimeReconstructor::new();
        let c_ns = C_LIGHT * 1e-9; // m/ns

        // Shower coming from +x direction at 30 degrees zenith
        let zenith = 30.0_f64.to_radians();
        let nx_true = zenith.sin();
        let ny_true = 0.0;
        // Need 2D spread of stations (not collinear) for full plane-wave fit
        let stations = vec![
            (0.0, 0.0),
            (1000.0, 0.0),
            (0.0, 1000.0),
            (1000.0, 1000.0),
        ];
        let times: Vec<f64> = stations
            .iter()
            .map(|&(x, y)| (x * nx_true + y * ny_true) / c_ns)
            .collect();

        let (nx, ny, _t0) = recon.reconstruct(&stations, &times);
        assert!(
            (nx - nx_true).abs() < 0.01,
            "Recovered nx {} should match true {}", nx, nx_true
        );
        assert!(ny.abs() < 0.01, "ny should be ~0, got {}", ny);
    }

    #[test]
    fn test_direction_to_angles_vertical() {
        let (zen, _az) = ArrivalTimeReconstructor::direction_to_angles(0.0, 0.0);
        assert!(zen.abs() < TOL, "Vertical shower zenith should be 0");
    }

    #[test]
    fn test_direction_to_angles_inclined() {
        let (zen, _az) = ArrivalTimeReconstructor::direction_to_angles(0.5, 0.0);
        assert!((zen - 0.5_f64.asin()).abs() < TOL);
    }

    // ── Core position estimator tests ─────────────────────────────────

    #[test]
    fn test_center_of_gravity_uniform() {
        let est = CorePositionEstimator::new();
        let stations = vec![(0.0, 0.0), (1000.0, 0.0), (0.0, 1000.0), (1000.0, 1000.0)];
        let signals = vec![1.0, 1.0, 1.0, 1.0];

        let (cx, cy) = est.center_of_gravity(&stations, &signals);
        assert!((cx - 500.0).abs() < TOL);
        assert!((cy - 500.0).abs() < TOL);
    }

    #[test]
    fn test_center_of_gravity_weighted() {
        let est = CorePositionEstimator::new();
        let stations = vec![(0.0, 0.0), (1000.0, 0.0)];
        let signals = vec![100.0, 10.0];

        let (cx, _cy) = est.center_of_gravity(&stations, &signals);
        // Core should be closer to the stronger station
        assert!(cx < 500.0, "Core {} should be closer to strong station at 0", cx);
    }

    #[test]
    fn test_chi2_minimization_near_cog() {
        let est = CorePositionEstimator::new();
        let fitter = LateralDistributionFitter::new(93.0);

        // Generate signals from a known core at (200, 300)
        let true_cx = 200.0;
        let true_cy = 300.0;
        let age = 1.1;
        let s_ref = 50.0;

        let stations = vec![
            (0.0, 0.0),
            (500.0, 0.0),
            (0.0, 500.0),
            (500.0, 500.0),
            (250.0, 250.0),
        ];
        let signals: Vec<f64> = stations
            .iter()
            .map(|&(x, y)| {
                let r = distance_2d(x, y, true_cx, true_cy).max(1.0);
                fitter.nkg(r, s_ref, age)
            })
            .collect();

        let (cx, cy, _chi2) = est.chi2_minimization(
            &stations, &signals, &fitter, age, s_ref, 300.0, 30,
        );

        assert!(
            distance_2d(cx, cy, true_cx, true_cy) < 100.0,
            "Estimated core ({}, {}) should be near true ({}, {})",
            cx, cy, true_cx, true_cy
        );
    }

    // ── Energy estimator tests ────────────────────────────────────────

    #[test]
    fn test_energy_from_s1000() {
        let config = ShowerConfig::auger_default();
        let est = EnergyEstimator::new(config);

        // S(1000) = 1 VEM should give ~2.19e17 eV
        let e = est.energy_from_s1000(1.0);
        assert!((e - 2.19e17).abs() / 2.19e17 < 0.01);

        // S(1000) = 40 VEM should give ~1e19 eV range
        let e40 = est.energy_from_s1000(40.0);
        assert!(e40 > 1e18 && e40 < 1e20, "Energy {} should be ~10^19 eV", e40);
    }

    #[test]
    fn test_energy_increases_with_signal() {
        let config = ShowerConfig::auger_default();
        let est = EnergyEstimator::new(config);

        let e_low = est.energy_from_s1000(10.0);
        let e_high = est.energy_from_s1000(100.0);
        assert!(e_high > e_low);
    }

    #[test]
    fn test_cic_correction_at_38_deg() {
        let config = ShowerConfig::auger_default();
        let est = EnergyEstimator::new(config);

        // At 38 degrees, CIC(38) = 1, so S_38 = S(1000)
        let s1000 = 50.0;
        let s38 = est.cic_correction(s1000, 38.0_f64.to_radians());
        assert!((s38 - s1000).abs() < 0.5, "CIC at 38deg should be ~identity");
    }

    #[test]
    fn test_heitler_energy() {
        // N_max ~ 10^9 => E ~ 10^9 * 8.1e7 = 8.1e16 eV
        let e = EnergyEstimator::energy_from_heitler(1e9);
        assert!((e - 8.1e16).abs() / 8.1e16 < TOL);
    }

    // ── Zenith angle calculator tests ─────────────────────────────────

    #[test]
    fn test_slant_depth_vertical() {
        let calc = ZenithAngleCalculator::with_depth(875.0);
        let x = calc.slant_depth(0.0); // Vertical
        assert!((x - 875.0).abs() < TOL);
    }

    #[test]
    fn test_slant_depth_inclined() {
        let calc = ZenithAngleCalculator::with_depth(875.0);
        let zen = 60.0_f64.to_radians();
        let x = calc.slant_depth(zen);
        // sec(60) = 2
        assert!((x - 1750.0).abs() < 1.0, "Slant depth at 60 deg = {}", x);
    }

    #[test]
    fn test_depth_at_altitude() {
        // Sea level
        let x0 = ZenithAngleCalculator::depth_at_altitude(0.0);
        assert!((x0 - X_VERTICAL).abs() < TOL);

        // Higher altitude should have less depth
        let x_high = ZenithAngleCalculator::depth_at_altitude(5000.0);
        assert!(x_high < x0);
    }

    #[test]
    fn test_sec_theta() {
        assert!((ZenithAngleCalculator::sec_theta(0.0) - 1.0).abs() < TOL);
        assert!((ZenithAngleCalculator::sec_theta(60.0_f64.to_radians()) - 2.0).abs() < 0.01);
    }

    // ── Shower age parameter tests ────────────────────────────────────

    #[test]
    fn test_age_at_maximum() {
        let age_calc = ShowerAgeParameter::new();
        // At X = X_max: s = 3*X_max/(X_max + 2*X_max) = 3/3 = 1
        let s = age_calc.age_at_depth(700.0, 700.0);
        assert!((s - 1.0).abs() < TOL, "Age at maximum should be 1, got {}", s);
    }

    #[test]
    fn test_age_increases_with_depth() {
        let age_calc = ShowerAgeParameter::new();
        let x_max = 700.0;
        let s1 = age_calc.age_at_depth(500.0, x_max);
        let s2 = age_calc.age_at_depth(900.0, x_max);
        assert!(s2 > s1, "Age should increase with atmospheric depth");
        assert!(s1 < 1.0, "Before maximum, age should be < 1");
        assert!(s2 > 1.0, "After maximum, age should be > 1");
    }

    #[test]
    fn test_xmax_proton_vs_iron() {
        let age_calc = ShowerAgeParameter::new();
        let e = 1e19;
        let xmax_p = age_calc.xmax_proton(e);
        let xmax_fe = age_calc.xmax_iron(e);

        assert!(xmax_p > xmax_fe, "Proton X_max should be deeper than iron");
        assert!((xmax_p - xmax_fe - 60.0).abs() < 1.0, "Difference should be ~60 g/cm²");
    }

    #[test]
    fn test_xmax_increases_with_energy() {
        let age_calc = ShowerAgeParameter::new();
        let xmax_low = age_calc.xmax_proton(1e18);
        let xmax_high = age_calc.xmax_proton(1e19);
        assert!(xmax_high > xmax_low, "X_max should increase with energy (elongation rate)");
    }

    #[test]
    fn test_gaisser_hillas_profile() {
        let age_calc = ShowerAgeParameter::new();
        let n_max = 1e9;
        let x_max = 700.0;
        let x0 = -50.0;
        let lambda = 70.0;

        // At X_max, N(X_max) should equal N_max
        let n_at_max = age_calc.gaisser_hillas(x_max, n_max, x_max, x0, lambda);
        assert!(
            (n_at_max - n_max).abs() / n_max < 0.01,
            "N at Xmax should be Nmax, got {}", n_at_max
        );

        // Before maximum, N should be less
        let n_before = age_calc.gaisser_hillas(400.0, n_max, x_max, x0, lambda);
        assert!(n_before < n_max);

        // After maximum, N should decrease
        let n_after = age_calc.gaisser_hillas(1000.0, n_max, x_max, x0, lambda);
        assert!(n_after < n_max);
    }

    #[test]
    fn test_gaisser_hillas_below_x0() {
        let age_calc = ShowerAgeParameter::new();
        let n = age_calc.gaisser_hillas(-100.0, 1e9, 700.0, -50.0, 70.0);
        assert!((n - 0.0).abs() < TOL, "No particles below X0");
    }

    #[test]
    fn test_elongation_rate() {
        let rate = ShowerAgeParameter::elongation_rate(750.0, 808.0, 18.0, 19.0);
        assert!((rate - 58.0).abs() < 1.0, "Elongation rate should be ~58 g/cm², got {}", rate);
    }

    // ── Muon content analyzer tests ───────────────────────────────────

    #[test]
    fn test_muon_number_proton() {
        let analyzer = MuonContentAnalyzer::new();
        let n_mu = analyzer.predicted_muon_number(1e19, 1.0);
        assert!(
            (n_mu - 1e7).abs() / 1e7 < 0.01,
            "Proton at 10^19 should give ~10^7 muons"
        );
    }

    #[test]
    fn test_muon_number_iron_more_than_proton() {
        let analyzer = MuonContentAnalyzer::new();
        let n_mu_p = analyzer.predicted_muon_number(1e19, 1.0);
        let n_mu_fe = analyzer.predicted_muon_number(1e19, 56.0);
        assert!(n_mu_fe > n_mu_p, "Iron should produce more muons than proton");
    }

    #[test]
    fn test_infer_mass_number_proton() {
        let analyzer = MuonContentAnalyzer::new();
        let n_mu_p = analyzer.predicted_muon_number(1e19, 1.0);
        let a = analyzer.infer_mass_number(n_mu_p, n_mu_p);
        assert!((a - 1.0).abs() < 0.1, "Proton should infer A~1, got {}", a);
    }

    #[test]
    fn test_infer_mass_number_iron() {
        let analyzer = MuonContentAnalyzer::new();
        let n_mu_p = analyzer.predicted_muon_number(1e19, 1.0);
        let n_mu_fe = analyzer.predicted_muon_number(1e19, 56.0);
        let a = analyzer.infer_mass_number(n_mu_fe, n_mu_p);
        assert!((a - 56.0).abs() < 5.0, "Iron should infer A~56, got {}", a);
    }

    #[test]
    fn test_muon_electron_ratio() {
        let ratio = MuonContentAnalyzer::muon_electron_ratio(1e6, 1e7);
        assert!((ratio - 0.1).abs() < TOL);
    }

    #[test]
    fn test_classify_composition() {
        let label = MuonContentAnalyzer::classify_composition(745.0, 750.0, 690.0);
        assert_eq!(label, "proton-like");

        let label = MuonContentAnalyzer::classify_composition(695.0, 750.0, 690.0);
        assert_eq!(label, "iron-like");

        let label = MuonContentAnalyzer::classify_composition(720.0, 750.0, 690.0);
        assert_eq!(label, "CNO-like");
    }

    // ── Fluorescence profiler tests ───────────────────────────────────

    #[test]
    fn test_photons_to_energy_deposit() {
        let profiler = FluorescenceProfiler::new();
        let photons = vec![100.0, 200.0, 150.0];
        let distance = 10000.0; // 10 km

        let de_dx = profiler.photons_to_energy_deposit(&photons, distance);
        assert_eq!(de_dx.len(), 3);
        // Higher photon count -> higher energy deposit
        assert!(de_dx[1] > de_dx[0]);
        assert!(de_dx[1] > de_dx[2]);
    }

    #[test]
    fn test_calorimetric_energy() {
        // Constant dE/dX = 1 MeV/(g/cm²) over 1000 g/cm²
        let depths: Vec<f64> = (0..=100).map(|i| i as f64 * 10.0).collect();
        let de_dx: Vec<f64> = vec![1.0; 101];

        let e = FluorescenceProfiler::calorimetric_energy(&depths, &de_dx);
        // Integral = 1.0 * 1000 = 1000 MeV = 1e9 eV
        assert!((e - 1e9).abs() / 1e9 < 0.01, "Calorimetric energy = {} eV", e);
    }

    #[test]
    fn test_gaisser_hillas_fit() {
        let profiler = FluorescenceProfiler::new();
        let age_calc = ShowerAgeParameter::new();

        // Generate a Gaisser-Hillas profile
        let n_max = 1e9;
        let x_max = 700.0;
        let x0 = -50.0;
        let lambda = 70.0;

        let depths: Vec<f64> = (10..=120).map(|i| i as f64 * 10.0).collect();
        let de_dx: Vec<f64> = depths
            .iter()
            .map(|&x| profiler.alpha_f * age_calc.gaisser_hillas(x, n_max, x_max, x0, lambda))
            .collect();

        let (fit_nmax, fit_xmax, _fit_x0, _fit_lambda, chi2) =
            profiler.fit_gaisser_hillas(&depths, &de_dx);

        assert!(
            (fit_xmax - x_max).abs() < 10.0,
            "Fitted Xmax {} should be near true {}", fit_xmax, x_max
        );
        assert!(
            fit_nmax > 0.0,
            "Fitted Nmax should be positive"
        );
    }

    // ── Cherenkov detector tests ──────────────────────────────────────

    #[test]
    fn test_cherenkov_angle_ultrarelativistic() {
        let det = CherenkovDetector::new();
        let theta = det.cherenkov_angle(1.0);
        // cos(theta) = 1/n, theta ~ 1.36 degrees ~ 0.0237 rad
        let expected = (1.0 / 1.000293_f64).acos();
        assert!((theta - expected).abs() < 1e-6);
    }

    #[test]
    fn test_cherenkov_below_threshold() {
        let det = CherenkovDetector::new();
        let theta = det.cherenkov_angle(0.99);
        // Below threshold: 1/(n*beta) > 1
        assert!((theta - 0.0).abs() < TOL, "Below threshold, angle should be 0");
    }

    #[test]
    fn test_cherenkov_threshold_beta() {
        let det = CherenkovDetector::new();
        let beta_thr = det.threshold_beta();
        assert!((beta_thr - 1.0 / 1.000293).abs() < 1e-8);
    }

    #[test]
    fn test_cherenkov_threshold_energy_electron() {
        let det = CherenkovDetector::new();
        let mass_electron = 0.511; // MeV
        let t_thr = det.threshold_energy_mev(mass_electron);
        // Cherenkov threshold for electrons in air: ~21 MeV
        assert!(t_thr > 10.0 && t_thr < 50.0, "Electron threshold = {} MeV", t_thr);
    }

    #[test]
    fn test_cherenkov_at_altitude() {
        let det_sea = CherenkovDetector::new();
        let det_high = CherenkovDetector::at_altitude(5000.0);
        // Higher altitude = lower refractive index
        assert!(det_high.n_refract < det_sea.n_refract);
    }

    #[test]
    fn test_cherenkov_photons_per_cm() {
        let det = CherenkovDetector::new();
        let n_photons = det.photons_per_cm(1.0);
        // Approximately ~490 * sin^2(theta_c) ~ 490 * (2*0.000293) ~ 0.287
        // Actually more carefully: sin^2(theta) = 1 - 1/n^2 for beta=1
        assert!(n_photons > 0.0);
        assert!(n_photons < 1.0, "Photons/cm = {} should be < 1 in air", n_photons);
    }

    #[test]
    fn test_cherenkov_lateral_density_decreases() {
        let det = CherenkovDetector::new();
        let d_near = det.lateral_photon_density(50.0, 1e6, 50.0);
        let d_far = det.lateral_photon_density(500.0, 1e6, 50.0);
        assert!(d_near > d_far, "Density should decrease with distance");
    }

    // ── Distance helper test ──────────────────────────────────────────

    #[test]
    fn test_distance_2d() {
        assert!((distance_2d(0.0, 0.0, 3.0, 4.0) - 5.0).abs() < TOL);
        assert!((distance_2d(1.0, 1.0, 1.0, 1.0) - 0.0).abs() < TOL);
    }

    // ── Full analysis pipeline test ───────────────────────────────────

    #[test]
    fn test_full_analysis_pipeline() {
        let config = ShowerConfig::auger_default();
        let fitter = LateralDistributionFitter::new(config.nkg_r_ref_m);

        // Simulate a shower with core at (100, 200)
        let core_x = 100.0;
        let core_y = 200.0;
        let s_ref = 80.0;
        let age = 1.1;

        let station_positions = vec![
            (0.0, 0.0),
            (1500.0, 0.0),
            (0.0, 1500.0),
            (-1500.0, 0.0),
            (0.0, -1500.0),
            (750.0, 750.0),
        ];

        let stations: Vec<StationData> = station_positions
            .iter()
            .enumerate()
            .map(|(i, &(x, y))| {
                let r = distance_2d(x, y, core_x, core_y).max(1.0);
                let signal = fitter.nkg(r, s_ref, age);
                StationData {
                    x,
                    y,
                    z: config.altitude_m,
                    signal_vem: signal,
                    time_ns: (i as f64) * 10.0, // Simplified timing
                }
            })
            .collect();

        let result = analyze_shower(&config, &stations);

        assert!(result.energy_ev > 0.0, "Energy should be positive");
        assert!(result.age > 0.0 && result.age < 2.0, "Age {} should be reasonable", result.age);
        assert!(
            distance_2d(result.core_x, result.core_y, core_x, core_y) < 500.0,
            "Core estimate should be in the right area"
        );
    }

    #[test]
    fn test_solve_3x3_identity() {
        let a = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let b = [3.0, 5.0, 7.0];
        let x = solve_3x3(&a, &b);
        assert!((x[0] - 3.0).abs() < TOL);
        assert!((x[1] - 5.0).abs() < TOL);
        assert!((x[2] - 7.0).abs() < TOL);
    }

    #[test]
    fn test_solve_3x3_general() {
        // 2x + y = 5
        // x + 3z = 7
        // y + 2z = 6
        let a = [[2.0, 1.0, 0.0], [1.0, 0.0, 3.0], [0.0, 1.0, 2.0]];
        let b = [5.0, 7.0, 6.0];
        let x = solve_3x3(&a, &b);

        // Verify solution
        let r0 = 2.0 * x[0] + x[1] - 5.0;
        let r1 = x[0] + 3.0 * x[2] - 7.0;
        let r2 = x[1] + 2.0 * x[2] - 6.0;
        assert!(r0.abs() < TOL, "Residual 0 = {}", r0);
        assert!(r1.abs() < TOL, "Residual 1 = {}", r1);
        assert!(r2.abs() < TOL, "Residual 2 = {}", r2);
    }
}
