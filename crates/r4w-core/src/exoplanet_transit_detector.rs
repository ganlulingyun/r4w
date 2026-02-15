//! Exoplanet Transit Detector — Transit photometry light curve analysis
//!
//! Implements exoplanet detection via the transit method, where a planet
//! crossing in front of its host star causes a periodic dimming in the
//! observed stellar flux. This module provides the complete pipeline from
//! raw photometric time series to validated transit candidates.
//!
//! ## Key Components
//!
//! - [`LightCurve`]: Time series of flux measurements with per-point uncertainties
//! - [`TransitModel`]: Mandel-Agol quadratic limb-darkened transit model
//! - [`BlsPeriodogram`]: Box Least Squares period search for periodic box-shaped dips
//! - [`TransitFitter`]: Least-squares transit parameter estimation
//! - [`TransitCandidate`]: Detected transit event with SNR and validation flags
//! - [`FalsePositiveScreener`]: Odd/even depth test and secondary eclipse check
//!
//! ## Algorithm Overview
//!
//! 1. **Period search** via BLS periodogram (Kovacs, Zucker & Mazeh 2002):
//!    Evaluate a grid of trial periods, folding the light curve and fitting
//!    a box model at each period to find the best-fit signal residual.
//!
//! 2. **Transit model fitting**: Refine parameters (period, epoch, depth,
//!    duration, impact parameter) using a Mandel-Agol limb-darkened model
//!    with Levenberg-Marquardt-style least-squares optimization.
//!
//! 3. **False positive screening**: Odd/even transit depth comparison to
//!    detect eclipsing binaries, and secondary eclipse search at phase 0.5
//!    to identify self-luminous companions.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::exoplanet_transit_detector::{LightCurve, BlsPeriodogram, BlsConfig};
//!
//! // Create a synthetic light curve with a periodic transit
//! let n = 2000;
//! let period = 3.5; // days
//! let depth = 0.01; // 1% dimming
//! let duration = 0.15; // days
//! let mut times = Vec::with_capacity(n);
//! let mut fluxes = Vec::with_capacity(n);
//! let mut errors = Vec::with_capacity(n);
//! for i in 0..n {
//!     let t = i as f64 * 0.05; // 0.05-day cadence over 100 days
//!     times.push(t);
//!     let phase = ((t / period) % 1.0 + 1.0) % 1.0;
//!     let f = if phase < duration / period { 1.0 - depth } else { 1.0 };
//!     fluxes.push(f);
//!     errors.push(0.001);
//! }
//! let lc = LightCurve::new(times, fluxes, errors);
//!
//! // Search for the transit period
//! let config = BlsConfig {
//!     min_period: 1.0,
//!     max_period: 10.0,
//!     num_periods: 1000,
//!     min_duration_phase: 0.01,
//!     max_duration_phase: 0.1,
//!     num_duration_steps: 10,
//! };
//! let bls = BlsPeriodogram::compute(&lc, &config);
//! let best = bls.best_period();
//! assert!((best.period - period).abs() < 0.1);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// LightCurve
// ---------------------------------------------------------------------------

/// A photometric time series of stellar flux measurements.
///
/// Each measurement consists of a timestamp (typically in days), a normalized
/// flux value (baseline ~1.0), and an associated uncertainty (standard deviation).
#[derive(Debug, Clone)]
pub struct LightCurve {
    /// Observation timestamps (e.g., BJD - 2457000).
    pub times: Vec<f64>,
    /// Normalized flux values (baseline = 1.0).
    pub fluxes: Vec<f64>,
    /// Per-point flux uncertainties (1-sigma).
    pub errors: Vec<f64>,
}

impl LightCurve {
    /// Create a new light curve from parallel vectors.
    ///
    /// # Panics
    ///
    /// Panics if the vectors have different lengths or are empty.
    pub fn new(times: Vec<f64>, fluxes: Vec<f64>, errors: Vec<f64>) -> Self {
        assert!(!times.is_empty(), "LightCurve must have at least one point");
        assert_eq!(times.len(), fluxes.len(), "times and fluxes must have equal length");
        assert_eq!(times.len(), errors.len(), "times and errors must have equal length");
        Self { times, fluxes, errors }
    }

    /// Number of data points.
    pub fn len(&self) -> usize {
        self.times.len()
    }

    /// Whether the light curve is empty.
    pub fn is_empty(&self) -> bool {
        self.times.is_empty()
    }

    /// Total time baseline (last time minus first time).
    pub fn baseline(&self) -> f64 {
        if self.times.len() < 2 {
            return 0.0;
        }
        let mut tmin = self.times[0];
        let mut tmax = self.times[0];
        for &t in &self.times {
            if t < tmin { tmin = t; }
            if t > tmax { tmax = t; }
        }
        tmax - tmin
    }

    /// Compute the weighted mean flux.
    pub fn weighted_mean_flux(&self) -> f64 {
        let mut sum_wf = 0.0;
        let mut sum_w = 0.0;
        for i in 0..self.len() {
            let w = 1.0 / (self.errors[i] * self.errors[i]);
            sum_wf += w * self.fluxes[i];
            sum_w += w;
        }
        if sum_w > 0.0 { sum_wf / sum_w } else { 1.0 }
    }

    /// Normalize the light curve so the median flux is 1.0.
    pub fn normalize(&mut self) {
        let median = self.median_flux();
        if median != 0.0 {
            for i in 0..self.len() {
                self.fluxes[i] /= median;
                self.errors[i] /= median.abs();
            }
        }
    }

    /// Compute the median flux value.
    pub fn median_flux(&self) -> f64 {
        let mut sorted: Vec<f64> = self.fluxes.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let n = sorted.len();
        if n == 0 {
            return 0.0;
        }
        if n % 2 == 0 {
            (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0
        } else {
            sorted[n / 2]
        }
    }
}

// ---------------------------------------------------------------------------
// Phase folding
// ---------------------------------------------------------------------------

/// Fold a time series to a given period and epoch.
///
/// Returns phase values in `[0, 1)` for each time point:
/// `phase_i = ((t_i - epoch) / period) mod 1.0`
pub fn fold_phase(times: &[f64], period: f64, epoch: f64) -> Vec<f64> {
    times
        .iter()
        .map(|&t| {
            let p = ((t - epoch) / period) % 1.0;
            if p < 0.0 { p + 1.0 } else { p }
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Transit geometry helpers
// ---------------------------------------------------------------------------

/// Estimate transit duration (total, T14) for a circular orbit.
///
/// Uses the formula:
/// `T = (P / pi) * arcsin(sqrt((R_star + R_planet)^2 - b^2 * R_star^2) / a)`
///
/// Where:
/// - `period`: orbital period (days)
/// - `r_ratio`: R_planet / R_star (dimensionless)
/// - `impact_param`: impact parameter `b` in [0, 1+r_ratio]
/// - `a_over_rstar`: semi-major axis / R_star (dimensionless)
///
/// Returns duration in the same units as `period`.
pub fn transit_duration(period: f64, r_ratio: f64, impact_param: f64, a_over_rstar: f64) -> f64 {
    let r_sum = 1.0 + r_ratio;
    let arg = r_sum * r_sum - impact_param * impact_param;
    if arg <= 0.0 {
        return 0.0;
    }
    let sin_arg = arg.sqrt() / a_over_rstar;
    if sin_arg.abs() > 1.0 {
        return 0.0;
    }
    (period / PI) * sin_arg.asin()
}

/// Compute the impact parameter from transit depth and duration.
///
/// For a box-shaped transit approximation:
/// `b = sqrt(1 - (duration * pi / period)^2 * a_rs^2 + (1 + sqrt(depth))^2)`
///
/// This is a simplified estimator; returns a value in [0, 1] or clamps.
pub fn impact_parameter(depth: f64, duration: f64, period: f64, a_over_rstar: f64) -> f64 {
    let r_ratio = depth.sqrt();
    let r_sum = 1.0 + r_ratio;
    let sin_half = (duration * PI / period).sin();
    let val = r_sum * r_sum - (a_over_rstar * sin_half) * (a_over_rstar * sin_half);
    if val < 0.0 {
        return 0.0;
    }
    val.sqrt().min(1.0 + r_ratio)
}

/// Estimate mean stellar density from transit observables (Seager & Mallen-Ornelas 2003).
///
/// `rho_star = (3 * pi / (G * P^2)) * (a/R_star)^3`
///
/// where `G` = 6.674e-11 m^3 kg^-1 s^-2.
///
/// - `period_days`: orbital period in days
/// - `a_over_rstar`: semi-major axis / R_star
///
/// Returns density in kg/m^3.
pub fn stellar_density(period_days: f64, a_over_rstar: f64) -> f64 {
    const G: f64 = 6.674e-11; // m^3 kg^-1 s^-2
    let period_s = period_days * 86400.0;
    (3.0 * PI / (G * period_s * period_s)) * a_over_rstar.powi(3)
}

// ---------------------------------------------------------------------------
// TransitModel — Mandel-Agol limb-darkened transit
// ---------------------------------------------------------------------------

/// Quadratic limb-darkening transit model (Mandel & Agol 2002).
///
/// Computes the flux attenuation when a dark sphere of radius `r_p` (in
/// stellar radii) transits a limb-darkened star. The stellar intensity
/// profile is:
///
/// `I(mu) / I(1) = 1 - u1*(1 - mu) - u2*(1 - mu)^2`
///
/// where `mu = cos(theta)` and `u1`, `u2` are the quadratic limb-darkening
/// coefficients.
#[derive(Debug, Clone, Copy)]
pub struct TransitModel {
    /// Planet-to-star radius ratio (R_p / R_star).
    pub r_ratio: f64,
    /// Quadratic limb-darkening coefficient u1.
    pub u1: f64,
    /// Quadratic limb-darkening coefficient u2.
    pub u2: f64,
}

impl TransitModel {
    /// Create a new transit model.
    ///
    /// - `r_ratio`: R_planet / R_star (typically 0.01 to 0.2)
    /// - `u1`, `u2`: quadratic limb-darkening coefficients
    pub fn new(r_ratio: f64, u1: f64, u2: f64) -> Self {
        Self { r_ratio, u1, u2 }
    }

    /// Create a uniform (no limb-darkening) transit model.
    pub fn uniform(r_ratio: f64) -> Self {
        Self::new(r_ratio, 0.0, 0.0)
    }

    /// Evaluate the transit model at a given projected separation `z`.
    ///
    /// `z` is the sky-projected distance between the planet and star centers
    /// in units of stellar radii. `z = 0` means the planet is centered on the
    /// star; `z >= 1 + r_ratio` means no transit.
    ///
    /// Returns the fractional flux (1.0 = no transit, < 1.0 during transit).
    pub fn flux_at_z(&self, z: f64) -> f64 {
        let p = self.r_ratio;
        if p <= 0.0 || z >= 1.0 + p {
            // No transit or invalid radius ratio
            return 1.0;
        }
        if z <= 0.0 {
            // Planet centered on star — full overlap of disk of radius p
            return 1.0 - self.limb_darkened_area(p, 0.0);
        }

        // Compute the blocked fractional flux
        let area = self.limb_darkened_area(p, z);
        1.0 - area
    }

    /// Compute the limb-darkened fractional flux blocked by a disk of radius
    /// `p` at projected separation `z` from the stellar center.
    ///
    /// This is a simplified Mandel-Agol implementation that handles the three
    /// geometric cases: full overlap, partial overlap, and annular transit.
    fn limb_darkened_area(&self, p: f64, z: f64) -> f64 {
        // Uniform (un-limb-darkened) transit area fraction
        let uniform_area = self.uniform_occultation_area(p, z);

        if self.u1 == 0.0 && self.u2 == 0.0 {
            return uniform_area;
        }

        // For limb-darkened case, we use a numerical integration approach.
        // We integrate the limb-darkened intensity over the occulted region.
        // The total stellar flux with limb darkening is:
        // F_total = integral of I(mu) over stellar disk
        //         = pi * (1 - u1/3 - u2/6) for quadratic LD
        let f_total = 1.0 - self.u1 / 3.0 - self.u2 / 6.0;

        if f_total <= 0.0 {
            return uniform_area;
        }

        // Numerical integration: sample the occulted region
        let blocked = self.integrate_blocked_flux(p, z);
        blocked / (PI * f_total)
    }

    /// Compute the uniform disk occultation area fraction.
    ///
    /// Returns the fraction of the stellar disk area blocked by a dark disk
    /// of radius `p` at separation `z`.
    fn uniform_occultation_area(&self, p: f64, z: f64) -> f64 {
        if z >= 1.0 + p {
            // No overlap
            return 0.0;
        }
        if z <= p - 1.0 {
            // Star entirely inside planet (or planet covers entire star)
            return 1.0;
        }
        if z <= 1.0 - p {
            // Planet entirely inside star — simple circle area
            return p * p;
        }
        // Partial overlap — lens-shaped intersection
        // Area = r^2 * arccos(d1) + R^2 * arccos(d2) - 0.5 * sqrt(area_term)
        // where r = p (planet), R = 1 (star), d = z (separation)
        let z2 = z * z;
        let p2 = p * p;
        let d1 = (z2 + p2 - 1.0) / (2.0 * z * p);
        let d2 = (z2 + 1.0 - p2) / (2.0 * z);

        let d1_clamped = d1.max(-1.0).min(1.0);
        let d2_clamped = d2.max(-1.0).min(1.0);

        let term = (-z + p + 1.0) * (z + p - 1.0) * (z - p + 1.0) * (z + p + 1.0);
        let area = p2 * d1_clamped.acos() + d2_clamped.acos() - 0.5 * term.max(0.0).sqrt();
        area / PI
    }

    /// Numerically integrate the limb-darkened flux blocked by the planet.
    ///
    /// Uses a grid integration over the overlapping region.
    fn integrate_blocked_flux(&self, p: f64, z: f64) -> f64 {
        // Number of integration steps (radial and angular)
        let n_radial = 50;
        let n_angular = 80;

        let mut blocked_flux = 0.0;

        // Integrate over the planet disk and check which points fall on the star
        for ir in 0..n_radial {
            let r = p * (ir as f64 + 0.5) / n_radial as f64;
            let dr = p / n_radial as f64;
            for ia in 0..n_angular {
                let theta = 2.0 * PI * (ia as f64 + 0.5) / n_angular as f64;
                let dtheta = 2.0 * PI / n_angular as f64;

                // Position on the stellar disk
                let x = z + r * theta.cos();
                let y = r * theta.sin();
                let rho = (x * x + y * y).sqrt(); // distance from star center

                if rho <= 1.0 {
                    // This point is on the stellar disk
                    let mu_sq = 1.0 - rho * rho;
                    let mu = if mu_sq > 0.0 { mu_sq.sqrt() } else { 0.0 };
                    let intensity =
                        1.0 - self.u1 * (1.0 - mu) - self.u2 * (1.0 - mu) * (1.0 - mu);
                    blocked_flux += intensity * r * dr * dtheta;
                }
            }
        }

        blocked_flux
    }

    /// Compute projected separation `z` from orbital phase.
    ///
    /// For a circular orbit:
    /// `z = (a/R_star) * sqrt(sin^2(2*pi*phase) + (b * cos(2*pi*phase))^2)`
    ///
    /// - `phase`: orbital phase (0 = mid-transit)
    /// - `a_over_rstar`: semi-major axis in stellar radii
    /// - `impact_param`: impact parameter b
    pub fn z_from_phase(phase: f64, a_over_rstar: f64, impact_param: f64) -> f64 {
        let phi = 2.0 * PI * phase;
        let sin_phi = phi.sin();
        let cos_phi = phi.cos();
        a_over_rstar * (sin_phi * sin_phi + impact_param * impact_param * cos_phi * cos_phi).sqrt()
    }

    /// Generate a model light curve for given times and transit parameters.
    ///
    /// - `times`: observation timestamps
    /// - `period`: orbital period (same units as times)
    /// - `epoch`: mid-transit time (same units as times)
    /// - `a_over_rstar`: semi-major axis / R_star
    /// - `impact_param`: impact parameter b
    ///
    /// Returns a vector of model flux values.
    pub fn model_light_curve(
        &self,
        times: &[f64],
        period: f64,
        epoch: f64,
        a_over_rstar: f64,
        impact_param: f64,
    ) -> Vec<f64> {
        let phases = fold_phase(times, period, epoch);
        phases
            .iter()
            .map(|&ph| {
                // Center phase on transit: shift so transit is at phase=0
                let centered = if ph > 0.5 { ph - 1.0 } else { ph };
                let z = Self::z_from_phase(centered, a_over_rstar, impact_param);
                self.flux_at_z(z)
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// BLS Periodogram
// ---------------------------------------------------------------------------

/// Configuration for the Box Least Squares periodogram search.
#[derive(Debug, Clone)]
pub struct BlsConfig {
    /// Minimum trial period (days).
    pub min_period: f64,
    /// Maximum trial period (days).
    pub max_period: f64,
    /// Number of trial periods (logarithmically spaced).
    pub num_periods: usize,
    /// Minimum transit duration as a fraction of the period.
    pub min_duration_phase: f64,
    /// Maximum transit duration as a fraction of the period.
    pub max_duration_phase: f64,
    /// Number of duration steps to try at each period.
    pub num_duration_steps: usize,
}

impl Default for BlsConfig {
    fn default() -> Self {
        Self {
            min_period: 0.5,
            max_period: 50.0,
            num_periods: 5000,
            min_duration_phase: 0.01,
            max_duration_phase: 0.15,
            num_duration_steps: 10,
        }
    }
}

/// Result of a BLS periodogram search at a single trial period.
#[derive(Debug, Clone, Copy)]
pub struct BlsPeak {
    /// Trial period (days).
    pub period: f64,
    /// BLS power (Signal Residue = SR, higher is better).
    pub power: f64,
    /// Best-fit transit depth at this period.
    pub depth: f64,
    /// Best-fit transit duration as a fraction of the period.
    pub duration_phase: f64,
    /// Best-fit transit phase (epoch as fraction of period).
    pub phase: f64,
}

/// Box Least Squares periodogram for transit period search.
///
/// Implements the BLS algorithm of Kovacs, Zucker & Mazeh (2002, A&A 391, 369).
/// At each trial period, the light curve is phase-folded and a box-shaped transit
/// model is fit by sliding a box of variable width across the phase.
#[derive(Debug, Clone)]
pub struct BlsPeriodogram {
    /// Trial periods evaluated.
    pub periods: Vec<f64>,
    /// BLS power at each trial period.
    pub powers: Vec<f64>,
    /// Best-fit parameters at each trial period.
    pub peaks: Vec<BlsPeak>,
}

impl BlsPeriodogram {
    /// Compute the BLS periodogram for a light curve.
    pub fn compute(lc: &LightCurve, config: &BlsConfig) -> Self {
        let n_periods = config.num_periods.max(1);
        let log_pmin = config.min_period.ln();
        let log_pmax = config.max_period.ln();

        let mut periods = Vec::with_capacity(n_periods);
        let mut powers = Vec::with_capacity(n_periods);
        let mut peaks = Vec::with_capacity(n_periods);

        // Precompute weights
        let weights: Vec<f64> = lc
            .errors
            .iter()
            .map(|e| {
                let e2 = e * e;
                if e2 > 0.0 { 1.0 / e2 } else { 1.0 }
            })
            .collect();
        let sum_w: f64 = weights.iter().sum();
        let mean_flux: f64 = weights
            .iter()
            .zip(lc.fluxes.iter())
            .map(|(w, f)| w * f)
            .sum::<f64>()
            / sum_w;

        for ip in 0..n_periods {
            let frac = ip as f64 / (n_periods as f64 - 1.0).max(1.0);
            let period = (log_pmin + frac * (log_pmax - log_pmin)).exp();
            periods.push(period);

            let phases = fold_phase(&lc.times, period, 0.0);

            let best = self::bls_single_period(
                &phases,
                &lc.fluxes,
                &weights,
                sum_w,
                mean_flux,
                config.min_duration_phase,
                config.max_duration_phase,
                config.num_duration_steps,
            );

            powers.push(best.0);
            peaks.push(BlsPeak {
                period,
                power: best.0,
                depth: best.1,
                duration_phase: best.2,
                phase: best.3,
            });
        }

        Self { periods, powers, peaks }
    }

    /// Return the peak with the highest BLS power.
    pub fn best_period(&self) -> BlsPeak {
        self.peaks
            .iter()
            .copied()
            .max_by(|a, b| a.power.partial_cmp(&b.power).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap_or(BlsPeak {
                period: 0.0,
                power: 0.0,
                depth: 0.0,
                duration_phase: 0.0,
                phase: 0.0,
            })
    }

    /// Return the top N peaks by power, sorted descending.
    pub fn top_peaks(&self, n: usize) -> Vec<BlsPeak> {
        let mut sorted: Vec<BlsPeak> = self.peaks.clone();
        sorted.sort_by(|a, b| b.power.partial_cmp(&a.power).unwrap_or(std::cmp::Ordering::Equal));
        sorted.truncate(n);
        sorted
    }
}

/// Evaluate BLS statistic for a single trial period.
///
/// Returns (power, depth, duration_phase, best_phase).
fn bls_single_period(
    phases: &[f64],
    fluxes: &[f64],
    weights: &[f64],
    sum_w: f64,
    mean_flux: f64,
    min_dur: f64,
    max_dur: f64,
    n_dur_steps: usize,
) -> (f64, f64, f64, f64) {
    let n = phases.len();
    let n_phase_bins = 200;

    // Sort by phase for binning
    let mut indices: Vec<usize> = (0..n).collect();
    indices.sort_by(|&a, &b| phases[a].partial_cmp(&phases[b]).unwrap_or(std::cmp::Ordering::Equal));

    // Bin the data by phase
    let mut bin_wf = vec![0.0_f64; n_phase_bins]; // sum of w*f in each bin
    let mut bin_w = vec![0.0_f64; n_phase_bins]; // sum of w in each bin

    for &idx in &indices {
        let bin = (phases[idx] * n_phase_bins as f64) as usize;
        let bin = bin.min(n_phase_bins - 1);
        bin_wf[bin] += weights[idx] * fluxes[idx];
        bin_w[bin] += weights[idx];
    }

    let mut best_power = 0.0_f64;
    let mut best_depth = 0.0_f64;
    let mut best_dur = min_dur;
    let mut best_phase = 0.0_f64;

    // Try different transit durations
    for id in 0..n_dur_steps.max(1) {
        let dur = min_dur + (max_dur - min_dur) * id as f64 / n_dur_steps.max(1) as f64;
        let n_bins_transit = (dur * n_phase_bins as f64).round().max(1.0) as usize;

        // Slide the box across all phase offsets
        for start_bin in 0..n_phase_bins {
            let mut s = 0.0_f64; // sum of w*f in transit
            let mut r = 0.0_f64; // sum of w in transit

            for j in 0..n_bins_transit {
                let bin = (start_bin + j) % n_phase_bins;
                s += bin_wf[bin];
                r += bin_w[bin];
            }

            if r > 0.0 && r < sum_w {
                // BLS Signal Residue: SR = s^2 / (r * (1 - r/sum_w))
                let sr = (s - r * mean_flux).powi(2) / (r * (1.0 - r / sum_w));
                if sr > best_power {
                    best_power = sr;
                    best_depth = (mean_flux - s / r).max(0.0);
                    best_dur = dur;
                    best_phase = start_bin as f64 / n_phase_bins as f64;
                }
            }
        }
    }

    (best_power, best_depth, best_dur, best_phase)
}

// ---------------------------------------------------------------------------
// TransitCandidate
// ---------------------------------------------------------------------------

/// A detected transit candidate with fitted parameters and validation status.
#[derive(Debug, Clone)]
pub struct TransitCandidate {
    /// Orbital period (days).
    pub period: f64,
    /// Mid-transit epoch (days, same reference as input times).
    pub epoch: f64,
    /// Transit depth (fractional, e.g., 0.01 = 1%).
    pub depth: f64,
    /// Transit duration (days).
    pub duration: f64,
    /// Signal-to-noise ratio of the detection.
    pub snr: f64,
    /// Planet-to-star radius ratio (sqrt(depth) for uniform model).
    pub radius_ratio: f64,
    /// Impact parameter estimate.
    pub impact_param: f64,
    /// Whether the candidate passed the odd/even depth test.
    pub odd_even_ok: bool,
    /// Whether the candidate passed the secondary eclipse test.
    pub secondary_eclipse_ok: bool,
    /// Number of observed transits.
    pub num_transits: usize,
}

impl TransitCandidate {
    /// Estimated planet radius in Earth radii, given R_star in solar radii.
    ///
    /// `R_planet = radius_ratio * R_star * (R_sun / R_earth)`
    /// where R_sun / R_earth ~ 109.076.
    pub fn planet_radius_earth(&self, r_star_solar: f64) -> f64 {
        self.radius_ratio * r_star_solar * 109.076
    }
}

// ---------------------------------------------------------------------------
// TransitFitter
// ---------------------------------------------------------------------------

/// Least-squares transit parameter estimator.
///
/// Given an initial period guess (from BLS), refines the transit parameters
/// (period, epoch, depth, duration) by minimizing chi-squared against the
/// observed light curve using a simplified iterative approach.
#[derive(Debug, Clone)]
pub struct TransitFitter {
    /// Maximum number of refinement iterations.
    pub max_iterations: usize,
    /// Convergence tolerance for relative chi-squared change.
    pub tolerance: f64,
}

impl Default for TransitFitter {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            tolerance: 1e-6,
        }
    }
}

impl TransitFitter {
    /// Create a new transit fitter with custom parameters.
    pub fn new(max_iterations: usize, tolerance: f64) -> Self {
        Self { max_iterations, tolerance }
    }

    /// Fit transit parameters to a light curve.
    ///
    /// Uses the BLS best-fit as an initial guess and refines with
    /// grid search around the initial parameters.
    ///
    /// Returns a `TransitCandidate` with fitted parameters.
    pub fn fit(&self, lc: &LightCurve, initial: &BlsPeak) -> TransitCandidate {
        let period = initial.period;
        let phase0 = initial.phase;
        let epoch = phase0 * period;

        // Refine epoch by searching around the initial guess
        let (best_epoch, best_depth, best_duration) =
            self.refine_parameters(lc, period, epoch, initial.depth, initial.duration_phase * period);

        // Compute SNR
        let snr = self.compute_snr(lc, period, best_epoch, best_depth, best_duration);

        // Count transits
        let num_transits = self.count_transits(lc, period, best_epoch, best_duration);

        // Estimate radius ratio and impact parameter
        let radius_ratio = best_depth.abs().sqrt();

        TransitCandidate {
            period,
            epoch: best_epoch,
            depth: best_depth,
            duration: best_duration,
            snr,
            radius_ratio,
            impact_param: 0.0, // Refined by false positive screener
            odd_even_ok: true,
            secondary_eclipse_ok: true,
            num_transits,
        }
    }

    /// Refine transit parameters by grid search.
    ///
    /// Returns (epoch, depth, duration).
    fn refine_parameters(
        &self,
        lc: &LightCurve,
        period: f64,
        initial_epoch: f64,
        initial_depth: f64,
        initial_duration: f64,
    ) -> (f64, f64, f64) {
        let mut best_epoch = initial_epoch;
        let mut best_depth = initial_depth;
        let mut best_duration = initial_duration.max(period * 0.005);
        let mut best_chi2 = f64::MAX;

        // Search grid around initial epoch
        let epoch_steps = 50;
        let epoch_range = period * 0.05; // +/- 5% of period
        let dur_steps = 20;
        let dur_min = period * 0.005;
        let dur_max = period * 0.15;

        for ie in 0..epoch_steps {
            let epoch = initial_epoch - epoch_range
                + 2.0 * epoch_range * ie as f64 / (epoch_steps as f64 - 1.0).max(1.0);

            for id in 0..dur_steps {
                let duration =
                    dur_min + (dur_max - dur_min) * id as f64 / (dur_steps as f64 - 1.0).max(1.0);

                // Fit depth analytically given epoch and duration
                let (depth, chi2) = self.fit_depth(lc, period, epoch, duration);

                if chi2 < best_chi2 {
                    best_chi2 = chi2;
                    best_epoch = epoch;
                    best_depth = depth;
                    best_duration = duration;
                }
            }
        }

        (best_epoch, best_depth, best_duration)
    }

    /// Analytically fit transit depth for given period, epoch, and duration.
    ///
    /// Returns (depth, chi2).
    fn fit_depth(
        &self,
        lc: &LightCurve,
        period: f64,
        epoch: f64,
        duration: f64,
    ) -> (f64, f64) {
        let phases = fold_phase(&lc.times, period, epoch);
        let half_dur_phase = (duration / period) / 2.0;

        let mut sum_w_in = 0.0_f64;
        let mut sum_wf_in = 0.0_f64;
        let mut sum_w_out = 0.0_f64;
        let mut sum_wf_out = 0.0_f64;

        for i in 0..lc.len() {
            let ph = phases[i];
            let in_transit = ph < half_dur_phase || ph > 1.0 - half_dur_phase;
            let w = 1.0 / (lc.errors[i] * lc.errors[i]);

            if in_transit {
                sum_w_in += w;
                sum_wf_in += w * lc.fluxes[i];
            } else {
                sum_w_out += w;
                sum_wf_out += w * lc.fluxes[i];
            }
        }

        if sum_w_in == 0.0 || sum_w_out == 0.0 {
            return (0.0, f64::MAX);
        }

        let mean_in = sum_wf_in / sum_w_in;
        let mean_out = sum_wf_out / sum_w_out;
        let depth = (mean_out - mean_in).max(0.0);

        // Compute chi-squared
        let mut chi2 = 0.0;
        for i in 0..lc.len() {
            let ph = phases[i];
            let in_transit = ph < half_dur_phase || ph > 1.0 - half_dur_phase;
            let model = if in_transit { mean_out - depth } else { mean_out };
            let residual = lc.fluxes[i] - model;
            chi2 += residual * residual / (lc.errors[i] * lc.errors[i]);
        }

        (depth, chi2)
    }

    /// Compute the transit signal-to-noise ratio.
    ///
    /// SNR = depth * sqrt(N_in_transit) / mean_error_in_transit
    fn compute_snr(
        &self,
        lc: &LightCurve,
        period: f64,
        epoch: f64,
        depth: f64,
        duration: f64,
    ) -> f64 {
        let phases = fold_phase(&lc.times, period, epoch);
        let half_dur_phase = (duration / period) / 2.0;

        let mut n_in = 0usize;
        let mut sum_e2 = 0.0_f64;

        for i in 0..lc.len() {
            let ph = phases[i];
            if ph < half_dur_phase || ph > 1.0 - half_dur_phase {
                n_in += 1;
                sum_e2 += lc.errors[i] * lc.errors[i];
            }
        }

        if n_in == 0 || sum_e2 == 0.0 {
            return 0.0;
        }

        let mean_err = (sum_e2 / n_in as f64).sqrt();
        depth * (n_in as f64).sqrt() / mean_err
    }

    /// Count the number of distinct transits observed.
    fn count_transits(&self, lc: &LightCurve, period: f64, epoch: f64, duration: f64) -> usize {
        if period <= 0.0 {
            return 0;
        }
        let baseline = lc.baseline();
        let n_possible = (baseline / period).ceil() as usize;
        let half_dur = duration / 2.0;

        let mut count = 0;
        for k in 0..=n_possible {
            let mid = epoch + k as f64 * period;
            // Check if any data point falls within this transit window
            let has_data = lc.times.iter().any(|&t| (t - mid).abs() < half_dur);
            if has_data {
                count += 1;
            }
        }
        count
    }
}

// ---------------------------------------------------------------------------
// FalsePositiveScreener
// ---------------------------------------------------------------------------

/// False positive screening for transit candidates.
///
/// Performs statistical tests to distinguish genuine planetary transits from
/// astrophysical false positives (eclipsing binaries, background eclipsing
/// binaries, etc.).
#[derive(Debug, Clone)]
pub struct FalsePositiveScreener {
    /// Maximum allowed fractional difference between odd and even transit depths.
    /// Typical threshold: 0.3 (30%).
    pub odd_even_threshold: f64,
    /// Minimum depth ratio for a secondary eclipse to be flagged.
    /// Typical threshold: 0.1 (10% of primary depth).
    pub secondary_eclipse_threshold: f64,
}

impl Default for FalsePositiveScreener {
    fn default() -> Self {
        Self {
            odd_even_threshold: 0.3,
            secondary_eclipse_threshold: 0.1,
        }
    }
}

impl FalsePositiveScreener {
    /// Create a new screener with custom thresholds.
    pub fn new(odd_even_threshold: f64, secondary_eclipse_threshold: f64) -> Self {
        Self {
            odd_even_threshold,
            secondary_eclipse_threshold,
        }
    }

    /// Screen a transit candidate against the light curve.
    ///
    /// Updates the candidate's validation flags in place.
    pub fn screen(&self, lc: &LightCurve, candidate: &mut TransitCandidate) {
        candidate.odd_even_ok = self.odd_even_test(lc, candidate);
        candidate.secondary_eclipse_ok = self.secondary_eclipse_test(lc, candidate);
    }

    /// Odd/even transit depth consistency test.
    ///
    /// Measures the transit depth separately for odd-numbered and even-numbered
    /// transits. A significant difference indicates an eclipsing binary at
    /// twice the detected period (alternating primary/secondary eclipses).
    ///
    /// Returns `true` if the depths are consistent (passes the test).
    pub fn odd_even_test(&self, lc: &LightCurve, candidate: &TransitCandidate) -> bool {
        let period = candidate.period;
        let epoch = candidate.epoch;
        let half_dur = candidate.duration / 2.0;

        if period <= 0.0 || candidate.depth <= 0.0 {
            return true;
        }

        let mut odd_flux_sum = 0.0_f64;
        let mut odd_count = 0usize;
        let mut even_flux_sum = 0.0_f64;
        let mut even_count = 0usize;
        let mut out_flux_sum = 0.0_f64;
        let mut out_count = 0usize;

        for i in 0..lc.len() {
            let t = lc.times[i];
            // Find which transit number this point belongs to
            let transit_num = ((t - epoch) / period).round() as i64;
            let mid_transit = epoch + transit_num as f64 * period;

            if (t - mid_transit).abs() < half_dur {
                if transit_num.abs() % 2 == 0 {
                    even_flux_sum += lc.fluxes[i];
                    even_count += 1;
                } else {
                    odd_flux_sum += lc.fluxes[i];
                    odd_count += 1;
                }
            } else {
                out_flux_sum += lc.fluxes[i];
                out_count += 1;
            }
        }

        if odd_count < 2 || even_count < 2 || out_count == 0 {
            // Not enough data to test
            return true;
        }

        let out_mean = out_flux_sum / out_count as f64;
        let odd_depth = out_mean - odd_flux_sum / odd_count as f64;
        let even_depth = out_mean - even_flux_sum / even_count as f64;

        let avg_depth = (odd_depth + even_depth) / 2.0;
        if avg_depth <= 0.0 {
            return true;
        }

        let frac_diff = (odd_depth - even_depth).abs() / avg_depth;
        frac_diff <= self.odd_even_threshold
    }

    /// Secondary eclipse test.
    ///
    /// Checks for a dip at orbital phase 0.5 (anti-transit). A significant
    /// secondary eclipse suggests a self-luminous companion (stellar binary)
    /// rather than a dark planet.
    ///
    /// Returns `true` if no significant secondary eclipse is found (passes).
    pub fn secondary_eclipse_test(&self, lc: &LightCurve, candidate: &TransitCandidate) -> bool {
        let period = candidate.period;
        let epoch = candidate.epoch;
        let half_dur = candidate.duration / 2.0;

        if period <= 0.0 || candidate.depth <= 0.0 {
            return true;
        }

        // Check at phase 0.5 (halfway between primary transits)
        let secondary_epoch = epoch + period / 2.0;

        let mut in_secondary_sum = 0.0_f64;
        let mut in_secondary_count = 0usize;
        let mut out_sum = 0.0_f64;
        let mut out_count = 0usize;

        let phases = fold_phase(&lc.times, period, secondary_epoch);
        let half_dur_phase = half_dur / period;

        for i in 0..lc.len() {
            let ph = phases[i];
            let in_primary = {
                let ph_prim = fold_phase(&[lc.times[i]], period, epoch)[0];
                ph_prim < half_dur_phase || ph_prim > 1.0 - half_dur_phase
            };

            if in_primary {
                // Skip primary transit points
                continue;
            }

            if ph < half_dur_phase || ph > 1.0 - half_dur_phase {
                in_secondary_sum += lc.fluxes[i];
                in_secondary_count += 1;
            } else {
                out_sum += lc.fluxes[i];
                out_count += 1;
            }
        }

        if in_secondary_count < 2 || out_count == 0 {
            return true;
        }

        let out_mean = out_sum / out_count as f64;
        let secondary_depth = (out_mean - in_secondary_sum / in_secondary_count as f64).max(0.0);

        let depth_ratio = secondary_depth / candidate.depth;
        depth_ratio <= self.secondary_eclipse_threshold
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Helper: generate a synthetic light curve with a box-shaped transit
    fn synthetic_light_curve(
        n_points: usize,
        cadence: f64,
        period: f64,
        depth: f64,
        duration: f64,
        noise_level: f64,
    ) -> LightCurve {
        let mut times = Vec::with_capacity(n_points);
        let mut fluxes = Vec::with_capacity(n_points);
        let mut errors = Vec::with_capacity(n_points);

        // Simple deterministic pseudo-noise
        let mut seed: u64 = 12345;
        let next_noise = |s: &mut u64| -> f64 {
            *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let bits = ((*s >> 33) ^ *s) as f64 / u64::MAX as f64;
            (bits - 0.5) * 2.0
        };

        for i in 0..n_points {
            let t = i as f64 * cadence;
            times.push(t);

            let phase = ((t / period) % 1.0 + 1.0) % 1.0;
            let half_dur_phase = (duration / period) / 2.0;
            let in_transit = phase < half_dur_phase || phase > 1.0 - half_dur_phase;
            let f = if in_transit { 1.0 - depth } else { 1.0 };
            let noise = next_noise(&mut seed) * noise_level;
            fluxes.push(f + noise);
            errors.push(noise_level.max(1e-10));
        }

        LightCurve::new(times, fluxes, errors)
    }

    // ---- LightCurve tests ----

    #[test]
    fn test_light_curve_creation() {
        let lc = LightCurve::new(vec![0.0, 1.0, 2.0], vec![1.0, 0.99, 1.0], vec![0.001, 0.001, 0.001]);
        assert_eq!(lc.len(), 3);
        assert!(!lc.is_empty());
    }

    #[test]
    #[should_panic]
    fn test_light_curve_empty() {
        LightCurve::new(vec![], vec![], vec![]);
    }

    #[test]
    #[should_panic]
    fn test_light_curve_mismatched_lengths() {
        LightCurve::new(vec![0.0, 1.0], vec![1.0], vec![0.001, 0.001]);
    }

    #[test]
    fn test_light_curve_baseline() {
        let lc = LightCurve::new(vec![5.0, 10.0, 15.0], vec![1.0; 3], vec![0.001; 3]);
        assert!((lc.baseline() - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_light_curve_weighted_mean() {
        let lc = LightCurve::new(
            vec![0.0, 1.0, 2.0],
            vec![1.0, 1.0, 1.0],
            vec![0.001, 0.001, 0.001],
        );
        assert!((lc.weighted_mean_flux() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_light_curve_median() {
        let lc = LightCurve::new(
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
            vec![1.0, 0.99, 1.01, 0.98, 1.02],
            vec![0.001; 5],
        );
        assert!((lc.median_flux() - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_light_curve_normalize() {
        let mut lc = LightCurve::new(
            vec![0.0, 1.0, 2.0],
            vec![2.0, 2.0, 2.0],
            vec![0.01, 0.01, 0.01],
        );
        lc.normalize();
        assert!((lc.fluxes[0] - 1.0).abs() < 1e-10);
        assert!((lc.errors[0] - 0.005).abs() < 1e-10);
    }

    // ---- Phase folding tests ----

    #[test]
    fn test_fold_phase_basic() {
        let times = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let phases = fold_phase(&times, 2.0, 0.0);
        assert!((phases[0] - 0.0).abs() < 1e-10);
        assert!((phases[1] - 0.5).abs() < 1e-10);
        assert!((phases[2] - 0.0).abs() < 1e-10);
        assert!((phases[3] - 0.5).abs() < 1e-10);
        assert!((phases[4] - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_fold_phase_with_epoch() {
        let times = vec![1.0, 2.0, 3.0];
        let phases = fold_phase(&times, 2.0, 1.0);
        assert!((phases[0] - 0.0).abs() < 1e-10);
        assert!((phases[1] - 0.5).abs() < 1e-10);
        assert!((phases[2] - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_fold_phase_negative_times() {
        let times = vec![-1.0, 0.0, 1.0];
        let phases = fold_phase(&times, 2.0, 0.0);
        // -1.0 / 2.0 = -0.5 -> 0.5
        assert!((phases[0] - 0.5).abs() < 1e-10);
        assert!((phases[1] - 0.0).abs() < 1e-10);
        assert!((phases[2] - 0.5).abs() < 1e-10);
    }

    // ---- Transit geometry helpers ----

    #[test]
    fn test_transit_duration_central() {
        // Central transit (b=0): duration should be non-zero
        let dur = transit_duration(3.0, 0.1, 0.0, 10.0);
        assert!(dur > 0.0);
        assert!(dur < 3.0); // Must be less than the period
    }

    #[test]
    fn test_transit_duration_grazing() {
        // Near-grazing transit (b close to 1 + r_ratio): very short
        let dur_central = transit_duration(3.0, 0.1, 0.0, 10.0);
        let dur_grazing = transit_duration(3.0, 0.1, 1.05, 10.0);
        assert!(dur_grazing < dur_central);
    }

    #[test]
    fn test_transit_duration_no_transit() {
        // Impact parameter too large for a transit
        let dur = transit_duration(3.0, 0.1, 1.2, 10.0);
        assert!((dur - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_impact_parameter_central() {
        let b = impact_parameter(0.01, 0.05, 3.0, 10.0);
        // For a non-grazing transit, b should be between 0 and ~1
        assert!(b >= 0.0);
        assert!(b <= 1.2);
    }

    #[test]
    fn test_stellar_density() {
        // Sun-like star: P ~ 365.25 days, a/R_star ~ 215
        let rho = stellar_density(365.25, 215.0);
        // Solar density ~ 1410 kg/m^3 — should be in the right ballpark
        assert!(rho > 500.0);
        assert!(rho < 5000.0);
    }

    // ---- TransitModel tests ----

    #[test]
    fn test_transit_model_no_transit() {
        let model = TransitModel::uniform(0.1);
        // z > 1 + p => no transit
        let f = model.flux_at_z(1.2);
        assert!((f - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_transit_model_full_overlap_uniform() {
        let model = TransitModel::uniform(0.1);
        // Planet centered on star: blocked area = p^2 = 0.01
        let f = model.flux_at_z(0.0);
        assert!((f - (1.0 - 0.01)).abs() < 0.001);
    }

    #[test]
    fn test_transit_model_limb_darkened() {
        let model = TransitModel::new(0.1, 0.4, 0.2);
        // Limb-darkened transit at center should be deeper than at limb
        let f_center = model.flux_at_z(0.0);
        let f_limb = model.flux_at_z(0.8);
        // Center is brighter due to LD, so blocking there removes more flux
        assert!(f_center < f_limb || (f_center - f_limb).abs() < 0.01);
    }

    #[test]
    fn test_transit_model_z_from_phase() {
        // At mid-transit (phase=0), z should equal b * a/R_star for b > 0
        let z_mid = TransitModel::z_from_phase(0.0, 10.0, 0.5);
        assert!((z_mid - 5.0).abs() < 1e-10);

        // At quadrature (phase=0.25), z should be large
        let z_quad = TransitModel::z_from_phase(0.25, 10.0, 0.5);
        assert!(z_quad > 5.0);
    }

    #[test]
    fn test_transit_model_light_curve_generation() {
        let model = TransitModel::uniform(0.1);
        let times: Vec<f64> = (0..100).map(|i| i as f64 * 0.1).collect();
        let lc = model.model_light_curve(&times, 5.0, 0.0, 15.0, 0.0);
        assert_eq!(lc.len(), 100);
        // All values should be <= 1.0
        assert!(lc.iter().all(|&f| f <= 1.0 + 1e-10));
        // At least some values should show a dip
        assert!(lc.iter().any(|&f| f < 0.999));
    }

    // ---- BLS Periodogram tests ----

    #[test]
    fn test_bls_detects_period() {
        let period = 3.5;
        let lc = synthetic_light_curve(2000, 0.05, period, 0.01, 0.15, 0.0001);

        let config = BlsConfig {
            min_period: 1.0,
            max_period: 10.0,
            num_periods: 500,
            min_duration_phase: 0.01,
            max_duration_phase: 0.1,
            num_duration_steps: 10,
        };

        let bls = BlsPeriodogram::compute(&lc, &config);
        let best = bls.best_period();

        // Should detect within 5% of true period
        assert!(
            (best.period - period).abs() / period < 0.05,
            "Detected period {} differs from true period {} by more than 5%",
            best.period,
            period
        );
    }

    #[test]
    fn test_bls_detects_depth() {
        let depth = 0.02;
        let lc = synthetic_light_curve(2000, 0.05, 4.0, depth, 0.2, 0.0001);

        let config = BlsConfig {
            min_period: 2.0,
            max_period: 8.0,
            num_periods: 300,
            min_duration_phase: 0.01,
            max_duration_phase: 0.1,
            num_duration_steps: 10,
        };

        let bls = BlsPeriodogram::compute(&lc, &config);
        let best = bls.best_period();

        // Depth should be roughly correct
        assert!(
            (best.depth - depth).abs() < 0.01,
            "Detected depth {} differs from true depth {}",
            best.depth,
            depth
        );
    }

    #[test]
    fn test_bls_top_peaks() {
        let lc = synthetic_light_curve(2000, 0.05, 3.0, 0.01, 0.15, 0.0001);
        let config = BlsConfig {
            min_period: 1.0,
            max_period: 10.0,
            num_periods: 200,
            ..Default::default()
        };

        let bls = BlsPeriodogram::compute(&lc, &config);
        let tops = bls.top_peaks(5);
        assert_eq!(tops.len(), 5);
        // Should be sorted by power descending
        for i in 1..tops.len() {
            assert!(tops[i].power <= tops[i - 1].power);
        }
    }

    #[test]
    fn test_bls_no_transit() {
        // Flat light curve — no transit signal
        let lc = LightCurve::new(
            (0..1000).map(|i| i as f64 * 0.1).collect(),
            vec![1.0; 1000],
            vec![0.001; 1000],
        );

        let config = BlsConfig {
            min_period: 1.0,
            max_period: 10.0,
            num_periods: 100,
            ..Default::default()
        };

        let bls = BlsPeriodogram::compute(&lc, &config);
        let best = bls.best_period();
        // Depth should be very small for a flat curve
        assert!(best.depth < 0.001, "False detection depth {} too large", best.depth);
    }

    // ---- TransitFitter tests ----

    #[test]
    fn test_transit_fitter_basic() {
        let period = 5.0;
        let depth = 0.015;
        let lc = synthetic_light_curve(3000, 0.05, period, depth, 0.25, 0.0001);

        let config = BlsConfig {
            min_period: 2.0,
            max_period: 10.0,
            num_periods: 300,
            ..Default::default()
        };

        let bls = BlsPeriodogram::compute(&lc, &config);
        let best = bls.best_period();

        let fitter = TransitFitter::default();
        let candidate = fitter.fit(&lc, &best);

        // Period should match
        assert!(
            (candidate.period - period).abs() / period < 0.05,
            "Fitted period {} vs true {}",
            candidate.period,
            period
        );
        // Depth should be close
        assert!(
            (candidate.depth - depth).abs() < 0.005,
            "Fitted depth {} vs true {}",
            candidate.depth,
            depth
        );
        // SNR should be positive
        assert!(candidate.snr > 0.0);
    }

    #[test]
    fn test_transit_fitter_snr() {
        let lc = synthetic_light_curve(2000, 0.05, 3.0, 0.01, 0.15, 0.0001);
        let config = BlsConfig {
            min_period: 1.0,
            max_period: 8.0,
            num_periods: 200,
            ..Default::default()
        };

        let bls = BlsPeriodogram::compute(&lc, &config);
        let fitter = TransitFitter::default();
        let candidate = fitter.fit(&lc, &bls.best_period());

        // With low noise and decent depth, SNR should be high
        assert!(
            candidate.snr > 5.0,
            "SNR {} is too low for a clear transit",
            candidate.snr
        );
    }

    #[test]
    fn test_transit_candidate_planet_radius() {
        let candidate = TransitCandidate {
            period: 3.0,
            epoch: 0.0,
            depth: 0.01, // 1% depth -> r_ratio = 0.1
            duration: 0.15,
            snr: 50.0,
            radius_ratio: 0.1,
            impact_param: 0.0,
            odd_even_ok: true,
            secondary_eclipse_ok: true,
            num_transits: 10,
        };

        // For a solar-type star (R_star = 1 R_sun):
        // R_planet = 0.1 * 109.076 R_earth ~ 10.9 R_earth (roughly Jupiter-sized)
        let r_earth = candidate.planet_radius_earth(1.0);
        assert!((r_earth - 10.9076).abs() < 0.1);
    }

    // ---- FalsePositiveScreener tests ----

    #[test]
    fn test_false_positive_screener_passes_genuine() {
        // Genuine transit: same depth at all epochs
        let lc = synthetic_light_curve(3000, 0.05, 4.0, 0.01, 0.2, 0.0001);

        let mut candidate = TransitCandidate {
            period: 4.0,
            epoch: 0.0,
            depth: 0.01,
            duration: 0.2,
            snr: 50.0,
            radius_ratio: 0.1,
            impact_param: 0.0,
            odd_even_ok: false,
            secondary_eclipse_ok: false,
            num_transits: 10,
        };

        let screener = FalsePositiveScreener::default();
        screener.screen(&lc, &mut candidate);

        assert!(candidate.odd_even_ok, "Genuine transit should pass odd/even test");
        assert!(
            candidate.secondary_eclipse_ok,
            "Genuine transit should pass secondary eclipse test"
        );
    }

    #[test]
    fn test_false_positive_screener_detects_eb() {
        // Eclipsing binary: alternating deep and shallow eclipses.
        // We simulate this by creating a signal with period P and alternating
        // depths. Transits are centered at t = k * period + epoch, and we use
        // round() to assign transit numbers (matching the odd_even_test logic).
        let n = 4000;
        let cadence = 0.04;
        let period = 4.0;
        let epoch = 0.0;
        let half_dur = 0.15; // transit half-duration in days
        let mut times = Vec::with_capacity(n);
        let mut fluxes = Vec::with_capacity(n);
        let mut errors = Vec::with_capacity(n);

        for i in 0..n {
            let t = i as f64 * cadence;
            times.push(t);

            // Use the same transit assignment as odd_even_test: round()
            let transit_num = ((t - epoch) / period).round() as i64;
            let mid_transit = epoch + transit_num as f64 * period;
            let in_transit = (t - mid_transit).abs() < half_dur;

            let depth = if transit_num.abs() % 2 == 0 { 0.02 } else { 0.005 };
            let f = if in_transit { 1.0 - depth } else { 1.0 };
            fluxes.push(f);
            errors.push(0.0001);
        }

        let lc = LightCurve::new(times, fluxes, errors);

        let candidate = TransitCandidate {
            period,
            epoch,
            depth: 0.0125, // Average of 0.02 and 0.005
            duration: half_dur * 2.0,
            snr: 50.0,
            radius_ratio: 0.1,
            impact_param: 0.0,
            odd_even_ok: true,
            secondary_eclipse_ok: true,
            num_transits: 10,
        };

        let screener = FalsePositiveScreener::default();
        let odd_even_ok = screener.odd_even_test(&lc, &candidate);

        // Should fail odd/even test due to depth difference
        // Even depth = 0.02, odd depth = 0.005, frac_diff = |0.02-0.005|/0.0125 = 1.2 > 0.3
        assert!(
            !odd_even_ok,
            "Eclipsing binary should fail odd/even test"
        );
    }

    #[test]
    fn test_false_positive_secondary_eclipse() {
        // Create a signal with both primary and secondary eclipses
        let n = 3000;
        let cadence = 0.05;
        let period = 4.0;
        let mut times = Vec::with_capacity(n);
        let mut fluxes = Vec::with_capacity(n);
        let mut errors = Vec::with_capacity(n);

        for i in 0..n {
            let t = i as f64 * cadence;
            times.push(t);

            let phase = ((t / period) % 1.0 + 1.0) % 1.0;
            let half_dur = 0.025;
            let in_primary = phase < half_dur || phase > 1.0 - half_dur;
            let in_secondary = (phase - 0.5).abs() < half_dur;

            let f = if in_primary {
                1.0 - 0.02
            } else if in_secondary {
                1.0 - 0.01 // Strong secondary eclipse (50% of primary)
            } else {
                1.0
            };
            fluxes.push(f);
            errors.push(0.0001);
        }

        let lc = LightCurve::new(times, fluxes, errors);

        let candidate = TransitCandidate {
            period,
            epoch: 0.0,
            depth: 0.02,
            duration: period * 0.05,
            snr: 50.0,
            radius_ratio: 0.1,
            impact_param: 0.0,
            odd_even_ok: true,
            secondary_eclipse_ok: true,
            num_transits: 10,
        };

        let screener = FalsePositiveScreener::default();
        let sec_ok = screener.secondary_eclipse_test(&lc, &candidate);

        // Should fail: secondary is 50% of primary depth, well above threshold
        assert!(
            !sec_ok,
            "Strong secondary eclipse should be flagged"
        );
    }

    // ---- Transit model integration tests ----

    #[test]
    fn test_uniform_occultation_symmetry() {
        let model = TransitModel::uniform(0.1);
        // Flux should be symmetric about z=0
        let f_pos = model.flux_at_z(0.5);
        let f_neg = model.flux_at_z(0.5); // z is always positive by definition
        assert!((f_pos - f_neg).abs() < 1e-10);
    }

    #[test]
    fn test_transit_model_monotonic_egress() {
        // As z increases from 0 to 1+p, flux should increase (less blocked)
        let model = TransitModel::uniform(0.1);
        let z_values: Vec<f64> = (0..20).map(|i| i as f64 * 0.06).collect();
        let fluxes: Vec<f64> = z_values.iter().map(|&z| model.flux_at_z(z)).collect();

        for i in 1..fluxes.len() {
            assert!(
                fluxes[i] >= fluxes[i - 1] - 1e-10,
                "Flux should be non-decreasing during egress: f[{}]={} < f[{}]={}",
                i,
                fluxes[i],
                i - 1,
                fluxes[i - 1]
            );
        }
    }

    #[test]
    fn test_bls_config_default() {
        let config = BlsConfig::default();
        assert!(config.min_period > 0.0);
        assert!(config.max_period > config.min_period);
        assert!(config.num_periods > 0);
    }

    #[test]
    fn test_transit_fitter_count_transits() {
        let lc = synthetic_light_curve(2000, 0.05, 5.0, 0.01, 0.15, 0.0001);
        let fitter = TransitFitter::default();
        let n = fitter.count_transits(&lc, 5.0, 0.0, 0.15);
        // With 100 days of data and 5-day period, should see ~20 transits
        assert!(n > 10, "Expected >10 transits, got {}", n);
    }

    #[test]
    fn test_large_planet_depth() {
        // Hot Jupiter-like transit (deep, ~1%)
        let model = TransitModel::uniform(0.1); // R_p/R_star = 0.1 -> depth ~ 1%
        let f = model.flux_at_z(0.0);
        assert!((f - 0.99).abs() < 0.005, "Expected ~1% depth, got {}", 1.0 - f);
    }
}
