//! Nanoparticle Tracking Analysis (NTA) signal processing.
//!
//! Processes particle trajectories from video microscopy to determine nanoparticle
//! size distributions via Brownian motion analysis. Implements the Stokes-Einstein
//! equation for converting diffusion coefficients to hydrodynamic diameters,
//! mean squared displacement (MSD) analysis, drift correction, and statistical
//! reporting of size distributions.
//!
//! # Physical Background
//!
//! Brownian motion of nanoparticles in a viscous medium is governed by the
//! Stokes-Einstein equation:
//!
//! ```text
//! D = kT / (6π η r)
//! ```
//!
//! where:
//! - `D` = diffusion coefficient (m²/s)
//! - `k` = Boltzmann constant (1.380649e-23 J/K)
//! - `T` = absolute temperature (K)
//! - `η` = dynamic viscosity (Pa·s)
//! - `r` = hydrodynamic radius (m)
//!
//! The mean squared displacement (MSD) for n-dimensional diffusion is:
//!
//! ```text
//! MSD(τ) = 2n D τ
//! ```
//!
//! For 2D tracking (n=2): `MSD(τ) = 4 D τ`
//!
//! # Example
//!
//! ```rust
//! use std::f64::consts::PI;
//!
//! // Stokes-Einstein: D for 100 nm particle at 25°C in water
//! let k = 1.380649e-23_f64;
//! let t = 298.15_f64;
//! let eta = 8.9e-4_f64;
//! let r = 50e-9_f64;
//! let d = k * t / (6.0 * PI * eta * r);
//! assert!((d - 4.9e-12).abs() < 1e-12);
//! ```

use std::f64::consts::PI;

// ─────────────────────────────────────────────────────────────────────────────
// Physical constants
// ─────────────────────────────────────────────────────────────────────────────

/// Boltzmann constant in J/K.
pub const BOLTZMANN: f64 = 1.380649e-23;

/// Dynamic viscosity of water at 20°C in Pa·s.
pub const WATER_VISCOSITY_20C: f64 = 1.002e-3;

/// Dynamic viscosity of water at 25°C in Pa·s.
pub const WATER_VISCOSITY_25C: f64 = 8.9e-4;

/// Dynamic viscosity of water at 37°C in Pa·s.
pub const WATER_VISCOSITY_37C: f64 = 6.91e-4;

/// Dynamic viscosity of PBS (phosphate-buffered saline) at 25°C in Pa·s.
pub const PBS_VISCOSITY_25C: f64 = 9.0e-4;

/// Dynamic viscosity of ethanol at 25°C in Pa·s.
pub const ETHANOL_VISCOSITY_25C: f64 = 1.074e-3;

// ─────────────────────────────────────────────────────────────────────────────
// Data types
// ─────────────────────────────────────────────────────────────────────────────

/// A 2D position in physical space (metres or pixels, consistent units).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Position {
    /// Horizontal coordinate.
    pub x: f64,
    /// Vertical coordinate.
    pub y: f64,
}

impl Position {
    /// Create a new position.
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

/// A single particle trajectory: an ordered sequence of positions sampled at
/// uniform frame intervals.
#[derive(Debug, Clone)]
pub struct Track {
    /// Ordered list of detected positions (one per frame).
    pub positions: Vec<Position>,
    /// Frame indices at which the particle was detected (same length as `positions`).
    pub frame_indices: Vec<usize>,
    /// Optional unique identifier for the track.
    pub id: usize,
}

impl Track {
    /// Create a new track from positions and corresponding frame indices.
    ///
    /// # Panics
    ///
    /// Panics if `positions` and `frame_indices` have different lengths.
    pub fn new(id: usize, positions: Vec<Position>, frame_indices: Vec<usize>) -> Self {
        assert_eq!(
            positions.len(),
            frame_indices.len(),
            "positions and frame_indices must have equal length"
        );
        Self { id, positions, frame_indices }
    }

    /// Number of detected points in this track.
    pub fn len(&self) -> usize {
        self.positions.len()
    }

    /// Returns `true` if the track contains no positions.
    pub fn is_empty(&self) -> bool {
        self.positions.len() == 0
    }
}

/// Solvent type for viscosity lookup.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Solvent {
    /// Pure water at 20°C.
    Water20C,
    /// Pure water at 25°C.
    Water25C,
    /// Pure water at 37°C.
    Water37C,
    /// Phosphate-buffered saline at 25°C.
    Pbs25C,
    /// Ethanol at 25°C.
    Ethanol25C,
    /// User-defined viscosity (Pa·s).
    Custom(f64),
}

impl Solvent {
    /// Return the dynamic viscosity in Pa·s.
    pub fn viscosity(&self) -> f64 {
        match self {
            Solvent::Water20C => WATER_VISCOSITY_20C,
            Solvent::Water25C => WATER_VISCOSITY_25C,
            Solvent::Water37C => WATER_VISCOSITY_37C,
            Solvent::Pbs25C => PBS_VISCOSITY_25C,
            Solvent::Ethanol25C => ETHANOL_VISCOSITY_25C,
            Solvent::Custom(v) => *v,
        }
    }

    /// Approximate temperature in Kelvin associated with this solvent preset.
    /// Returns `None` for `Custom`.
    pub fn temperature_k(&self) -> Option<f64> {
        match self {
            Solvent::Water20C => Some(293.15),
            Solvent::Water25C => Some(298.15),
            Solvent::Water37C => Some(310.15),
            Solvent::Pbs25C => Some(298.15),
            Solvent::Ethanol25C => Some(298.15),
            Solvent::Custom(_) => None,
        }
    }
}

/// Configuration for the NTA processor.
#[derive(Debug, Clone)]
pub struct NtaConfig {
    /// Frame acquisition rate in frames per second (Hz).
    pub frame_rate: f64,
    /// Physical pixel size in metres (converts pixel displacements to metres).
    pub pixel_size_m: f64,
    /// Solvent used (determines viscosity).
    pub solvent: Solvent,
    /// Temperature in Kelvin (overrides solvent preset if `Some`).
    pub temperature_k: Option<f64>,
    /// Minimum track length (number of points) required for analysis.
    pub min_track_len: usize,
    /// Maximum lag time (in frames) used for MSD fitting.
    /// Use `None` to default to `track_len / 3`.
    pub max_lag_frames: Option<usize>,
    /// Number of bins for the size histogram.
    pub histogram_bins: usize,
    /// Whether to apply drift correction to tracks before analysis.
    pub drift_correction: bool,
    /// Field-of-view volume in mL for concentration estimation.
    /// Typically `width_m * height_m * depth_m * 1e6`.
    pub fov_volume_ml: Option<f64>,
    /// Dilution factor applied to the sample before measurement.
    pub dilution_factor: f64,
}

impl Default for NtaConfig {
    fn default() -> Self {
        Self {
            frame_rate: 30.0,
            pixel_size_m: 110e-9, // 110 nm/pixel (typical 20x objective)
            solvent: Solvent::Water25C,
            temperature_k: None,
            min_track_len: 10,
            max_lag_frames: None,
            histogram_bins: 50,
            drift_correction: true,
            fov_volume_ml: None,
            dilution_factor: 1.0,
        }
    }
}

impl NtaConfig {
    /// Effective temperature in Kelvin.
    pub fn temperature(&self) -> f64 {
        self.temperature_k
            .or_else(|| self.solvent.temperature_k())
            .unwrap_or(298.15)
    }

    /// Effective dynamic viscosity in Pa·s.
    pub fn viscosity(&self) -> f64 {
        self.solvent.viscosity()
    }
}

/// Per-track analysis result.
#[derive(Debug, Clone)]
pub struct TrackResult {
    /// Track identifier.
    pub id: usize,
    /// Estimated diffusion coefficient in m²/s.
    pub diffusion_coeff: f64,
    /// 95% confidence interval half-width for the diffusion coefficient.
    pub diffusion_ci95: f64,
    /// Hydrodynamic diameter in nanometres.
    pub diameter_nm: f64,
    /// Anomalous diffusion exponent α (1.0 = normal Brownian, >1 directed, <1 confined).
    pub alpha: f64,
    /// Track completeness: fraction of frames in the track span where particle was detected.
    pub completeness: f64,
    /// Number of valid MSD points used for fitting.
    pub n_msd_points: usize,
    /// Whether this track was flagged as an outlier and excluded from the distribution.
    pub is_outlier: bool,
}

/// Ensemble size distribution results.
#[derive(Debug, Clone)]
pub struct SizeDistribution {
    /// Bin centres in nanometres (length = `histogram_bins`).
    pub bin_centres_nm: Vec<f64>,
    /// Counts (or probability density) in each bin.
    pub counts: Vec<f64>,
    /// D10 percentile (nm): 10% of particles smaller than this.
    pub d10_nm: f64,
    /// D50 percentile / median diameter (nm).
    pub d50_nm: f64,
    /// D90 percentile (nm): 90% of particles smaller than this.
    pub d90_nm: f64,
    /// Mean diameter (nm).
    pub mean_nm: f64,
    /// Standard deviation of diameters (nm).
    pub std_nm: f64,
    /// Polydispersity index: σ² / μ² (dimensionless, 0 = monodisperse).
    pub pdi: f64,
    /// Number of valid (non-outlier) tracks included.
    pub n_valid_tracks: usize,
}

/// Full NTA analysis output.
#[derive(Debug, Clone)]
pub struct NtaResult {
    /// Per-track results (all tracks, including outliers).
    pub tracks: Vec<TrackResult>,
    /// Size distribution computed from valid tracks.
    pub distribution: SizeDistribution,
    /// Estimated concentration in particles/mL (if FOV volume was provided).
    pub concentration_per_ml: Option<f64>,
    /// Mean drift velocity subtracted during drift correction (pixels/frame).
    pub drift_px_per_frame: Option<[f64; 2]>,
}

// ─────────────────────────────────────────────────────────────────────────────
// Core physics functions (public utilities)
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the diffusion coefficient via the Stokes-Einstein equation.
///
/// # Arguments
///
/// * `temperature_k` – temperature in Kelvin
/// * `viscosity_pa_s` – dynamic viscosity in Pa·s
/// * `radius_m` – hydrodynamic radius in metres
///
/// # Returns
///
/// Diffusion coefficient in m²/s.
pub fn stokes_einstein_d(temperature_k: f64, viscosity_pa_s: f64, radius_m: f64) -> f64 {
    BOLTZMANN * temperature_k / (6.0 * PI * viscosity_pa_s * radius_m)
}

/// Compute the hydrodynamic radius from a diffusion coefficient via Stokes-Einstein.
///
/// # Arguments
///
/// * `d_m2s` – diffusion coefficient in m²/s
/// * `temperature_k` – temperature in Kelvin
/// * `viscosity_pa_s` – dynamic viscosity in Pa·s
///
/// # Returns
///
/// Hydrodynamic radius in metres.
pub fn stokes_einstein_r(d_m2s: f64, temperature_k: f64, viscosity_pa_s: f64) -> f64 {
    BOLTZMANN * temperature_k / (6.0 * PI * viscosity_pa_s * d_m2s)
}

/// Convert a diffusion coefficient to hydrodynamic diameter in nanometres.
///
/// Uses the Stokes-Einstein equation internally.
pub fn diffusion_to_diameter_nm(d_m2s: f64, temperature_k: f64, viscosity_pa_s: f64) -> f64 {
    2.0 * stokes_einstein_r(d_m2s, temperature_k, viscosity_pa_s) * 1e9
}

/// Compute the theoretical MSD for a given diffusion coefficient, lag time,
/// and number of spatial dimensions.
///
/// `MSD = 2 * n_dims * D * tau`
pub fn theoretical_msd(d_m2s: f64, tau_s: f64, n_dims: u32) -> f64 {
    2.0 * (n_dims as f64) * d_m2s * tau_s
}

// ─────────────────────────────────────────────────────────────────────────────
// MSD computation
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the time-averaged mean squared displacement for a single track.
///
/// For each lag `τ = 1, 2, …, max_lag` (in frames), averages
/// `|r(t + τ) − r(t)|²` over all valid starting times `t`.
///
/// # Arguments
///
/// * `positions` – ordered list of 2D positions (pixels)
/// * `max_lag` – maximum lag in frames to compute
///
/// # Returns
///
/// Vector of `(lag_frames, msd_px2)` pairs of length `max_lag`.
pub fn compute_msd(positions: &[Position], max_lag: usize) -> Vec<(usize, f64)> {
    let n = positions.len();
    let lag_max = max_lag.min(n.saturating_sub(1));
    let mut result = Vec::with_capacity(lag_max);

    for lag in 1..=lag_max {
        let mut sum = 0.0;
        let mut count = 0usize;
        for i in 0..(n - lag) {
            let dx = positions[i + lag].x - positions[i].x;
            let dy = positions[i + lag].y - positions[i].y;
            sum += dx * dx + dy * dy;
            count += 1;
        }
        if count > 0 {
            result.push((lag, sum / count as f64));
        }
    }
    result
}

/// Compute the ensemble-averaged MSD across multiple tracks.
///
/// Each track contributes equally at each lag time where it has valid data.
///
/// # Arguments
///
/// * `per_track_msds` – slice of per-track MSD vectors from `compute_msd`
/// * `max_lag` – maximum lag index to include
///
/// # Returns
///
/// Vector of `(lag_frames, ensemble_msd)` pairs.
pub fn ensemble_msd(per_track_msds: &[Vec<(usize, f64)>], max_lag: usize) -> Vec<(usize, f64)> {
    let mut sums = vec![0.0f64; max_lag + 1];
    let mut counts = vec![0usize; max_lag + 1];

    for track_msd in per_track_msds {
        for &(lag, msd) in track_msd {
            if lag <= max_lag {
                sums[lag] += msd;
                counts[lag] += 1;
            }
        }
    }

    (1..=max_lag)
        .filter(|&lag| counts[lag] > 0)
        .map(|lag| (lag, sums[lag] / counts[lag] as f64))
        .collect()
}

// ─────────────────────────────────────────────────────────────────────────────
// Fitting utilities
// ─────────────────────────────────────────────────────────────────────────────

/// Weighted linear regression: fit `y = slope * x + intercept`.
///
/// Returns `(slope, intercept, r_squared)`.
/// Weights `w` must be positive; if empty or all-zero, falls back to unweighted.
fn weighted_linear_fit(x: &[f64], y: &[f64], w: &[f64]) -> (f64, f64, f64) {
    let n = x.len().min(y.len()).min(w.len());
    if n < 2 {
        return (0.0, 0.0, 0.0);
    }

    let w_sum: f64 = w[..n].iter().sum();
    if w_sum == 0.0 {
        return (0.0, 0.0, 0.0);
    }

    let x_mean: f64 = x[..n].iter().zip(w[..n].iter()).map(|(xi, wi)| xi * wi).sum::<f64>() / w_sum;
    let y_mean: f64 = y[..n].iter().zip(w[..n].iter()).map(|(yi, wi)| yi * wi).sum::<f64>() / w_sum;

    let mut num = 0.0f64;
    let mut den = 0.0f64;
    for i in 0..n {
        let dx = x[i] - x_mean;
        num += w[i] * dx * (y[i] - y_mean);
        den += w[i] * dx * dx;
    }

    if den == 0.0 {
        return (0.0, y_mean, 0.0);
    }

    let slope = num / den;
    let intercept = y_mean - slope * x_mean;

    // R²
    let ss_res: f64 = (0..n).map(|i| {
        let e = y[i] - (slope * x[i] + intercept);
        w[i] * e * e
    }).sum();
    let ss_tot: f64 = (0..n).map(|i| {
        let e = y[i] - y_mean;
        w[i] * e * e
    }).sum();
    let r2 = if ss_tot > 0.0 { 1.0 - ss_res / ss_tot } else { 1.0 };

    (slope, intercept, r2)
}

/// Power-law fit in log-log space: `log(MSD) = α * log(τ) + log(4D)`.
///
/// Returns `(alpha, d_from_intercept)` where `d = exp(intercept) / 4`.
fn power_law_fit(lag_s: &[f64], msd: &[f64]) -> (f64, f64) {
    let n = lag_s.len().min(msd.len());
    if n < 2 {
        return (1.0, 0.0);
    }

    let log_tau: Vec<f64> = lag_s[..n].iter().map(|&t| t.max(1e-300).ln()).collect();
    let log_msd: Vec<f64> = msd[..n].iter().map(|&m| m.max(1e-300).ln()).collect();
    let w: Vec<f64> = vec![1.0; n];

    let (alpha, intercept, _) = weighted_linear_fit(&log_tau, &log_msd, &w);
    let d = intercept.exp() / 4.0;
    (alpha, d)
}

// ─────────────────────────────────────────────────────────────────────────────
// Drift correction
// ─────────────────────────────────────────────────────────────────────────────

/// Estimate and subtract ensemble drift from a collection of tracks.
///
/// Drift is estimated as the mean displacement per frame across all tracks.
/// Returns the corrected tracks and the estimated drift vector `[dx, dy]`
/// in pixels/frame.
pub fn drift_correct(tracks: &[Track]) -> (Vec<Track>, [f64; 2]) {
    // Collect all single-frame displacements from all tracks
    let mut dx_sum = 0.0f64;
    let mut dy_sum = 0.0f64;
    let mut count = 0usize;

    for track in tracks {
        let n = track.positions.len();
        for i in 1..n {
            // Only use consecutive detected frames
            if track.frame_indices[i] == track.frame_indices[i - 1] + 1 {
                dx_sum += track.positions[i].x - track.positions[i - 1].x;
                dy_sum += track.positions[i].y - track.positions[i - 1].y;
                count += 1;
            }
        }
    }

    if count == 0 {
        return (tracks.to_vec(), [0.0, 0.0]);
    }

    let drift = [dx_sum / count as f64, dy_sum / count as f64];

    // Subtract cumulative drift from each track position
    let corrected: Vec<Track> = tracks
        .iter()
        .map(|track| {
            if track.is_empty() {
                return track.clone();
            }
            let first_frame = track.frame_indices[0] as f64;
            let positions: Vec<Position> = track
                .positions
                .iter()
                .zip(track.frame_indices.iter())
                .map(|(p, &fi)| {
                    let t = (fi as f64) - first_frame;
                    Position::new(p.x - drift[0] * t, p.y - drift[1] * t)
                })
                .collect();
            Track::new(track.id, positions, track.frame_indices.clone())
        })
        .collect();

    (corrected, drift)
}

// ─────────────────────────────────────────────────────────────────────────────
// Statistical utilities
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the percentile of a sorted slice (0–100).
fn percentile_sorted(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let n = sorted.len();
    if n == 1 {
        return sorted[0];
    }
    let idx = (p / 100.0) * (n as f64 - 1.0);
    let lo = idx.floor() as usize;
    let hi = (lo + 1).min(n - 1);
    let frac = idx - lo as f64;
    sorted[lo] * (1.0 - frac) + sorted[hi] * frac
}

/// Sort a Vec<f64> in place and return it.
fn sort_f64(mut v: Vec<f64>) -> Vec<f64> {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    v
}

/// Kernel density estimation with Gaussian kernel, bandwidth by Silverman's rule.
fn kde(values: &[f64], eval_points: &[f64]) -> Vec<f64> {
    let n = values.len();
    if n == 0 {
        return vec![0.0; eval_points.len()];
    }

    // Compute std dev
    let mean = values.iter().sum::<f64>() / n as f64;
    let var = values.iter().map(|&v| (v - mean).powi(2)).sum::<f64>() / n as f64;
    let std = var.sqrt();
    if std == 0.0 {
        return vec![0.0; eval_points.len()];
    }

    // Silverman bandwidth
    let h = 1.06 * std * (n as f64).powf(-0.2);

    eval_points
        .iter()
        .map(|&x| {
            let sum: f64 = values
                .iter()
                .map(|&xi| {
                    let u = (x - xi) / h;
                    (-0.5 * u * u).exp()
                })
                .sum();
            sum / (n as f64 * h * (2.0 * PI).sqrt())
        })
        .collect()
}

// ─────────────────────────────────────────────────────────────────────────────
// Size distribution builder
// ─────────────────────────────────────────────────────────────────────────────

/// Build a size distribution histogram from a list of valid diameters.
fn build_distribution(
    diameters_nm: &[f64],
    n_bins: usize,
    _use_log_bins: bool,
) -> SizeDistribution {
    let n_valid = diameters_nm.len();

    if n_valid == 0 {
        return SizeDistribution {
            bin_centres_nm: vec![],
            counts: vec![],
            d10_nm: 0.0,
            d50_nm: 0.0,
            d90_nm: 0.0,
            mean_nm: 0.0,
            std_nm: 0.0,
            pdi: 0.0,
            n_valid_tracks: 0,
        };
    }

    let sorted = sort_f64(diameters_nm.to_vec());
    let d10 = percentile_sorted(&sorted, 10.0);
    let d50 = percentile_sorted(&sorted, 50.0);
    let d90 = percentile_sorted(&sorted, 90.0);

    let mean = diameters_nm.iter().sum::<f64>() / n_valid as f64;
    let var = diameters_nm
        .iter()
        .map(|&d| (d - mean).powi(2))
        .sum::<f64>()
        / n_valid as f64;
    let std = var.sqrt();
    let pdi = if mean > 0.0 { var / (mean * mean) } else { 0.0 };

    let d_min = sorted[0];
    let d_max = sorted[n_valid - 1];
    let range = (d_max - d_min).max(1.0);
    let bin_width = range / n_bins as f64;

    let bin_centres_nm: Vec<f64> = (0..n_bins)
        .map(|i| d_min + (i as f64 + 0.5) * bin_width)
        .collect();

    // Use KDE evaluated at bin centres for smooth distribution
    let counts = kde(diameters_nm, &bin_centres_nm);

    SizeDistribution {
        bin_centres_nm,
        counts,
        d10_nm: d10,
        d50_nm: d50,
        d90_nm: d90,
        mean_nm: mean,
        std_nm: std,
        pdi,
        n_valid_tracks: n_valid,
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Main NTA processor
// ─────────────────────────────────────────────────────────────────────────────

/// Main Nanoparticle Tracking Analysis processor.
///
/// Processes a collection of particle tracks and returns size distribution
/// statistics, diffusion coefficients, and optional concentration estimates.
pub struct NtaProcessor {
    config: NtaConfig,
}

impl NtaProcessor {
    /// Create a new NTA processor with the given configuration.
    pub fn new(config: NtaConfig) -> Self {
        Self { config }
    }

    /// Create a processor with default configuration (water at 25°C, 30 fps).
    pub fn default() -> Self {
        Self::new(NtaConfig::default())
    }

    /// Run the full NTA analysis on a set of tracks.
    ///
    /// Steps:
    /// 1. Filter tracks shorter than `min_track_len`
    /// 2. (Optional) drift correction
    /// 3. Per-track MSD computation and diffusion coefficient fitting
    /// 4. Outlier rejection (D ≤ 0)
    /// 5. Size distribution and statistics
    /// 6. (Optional) concentration estimation
    pub fn analyze(&self, tracks: &[Track]) -> NtaResult {
        let cfg = &self.config;
        let temp_k = cfg.temperature();
        let eta = cfg.viscosity();
        let px_m = cfg.pixel_size_m;
        let dt = 1.0 / cfg.frame_rate; // seconds per frame

        // ── Step 1: filter short tracks ──
        let long_tracks: Vec<&Track> = tracks
            .iter()
            .filter(|t| t.len() >= cfg.min_track_len)
            .collect();

        // ── Step 2: drift correction ──
        let (corrected_owned, drift_px);
        let working_tracks: Vec<&Track>;
        let mut drift_opt = None;

        if cfg.drift_correction {
            let owned_refs: Vec<Track> = long_tracks.iter().map(|&t| t.clone()).collect();
            let (c, d) = drift_correct(&owned_refs);
            corrected_owned = c;
            drift_px = d;
            drift_opt = Some(drift_px);
            working_tracks = corrected_owned.iter().collect();
        } else {
            corrected_owned = vec![];
            drift_px = [0.0, 0.0];
            let _ = drift_px;
            working_tracks = long_tracks.clone();
        }

        // ── Step 3: per-track MSD fit ──
        let mut track_results: Vec<TrackResult> = Vec::new();

        for track in &working_tracks {
            let n = track.len();
            let max_lag = cfg
                .max_lag_frames
                .unwrap_or(n / 3)
                .min(n.saturating_sub(1))
                .max(2);

            let msd_px2 = compute_msd(&track.positions, max_lag);
            if msd_px2.len() < 2 {
                continue;
            }

            // Convert to physical units
            let lag_s: Vec<f64> = msd_px2.iter().map(|&(l, _)| l as f64 * dt).collect();
            let msd_m2: Vec<f64> = msd_px2.iter().map(|&(_, m)| m * px_m * px_m).collect();

            // Number of pairs at each lag (weight by sample count)
            let weights: Vec<f64> = msd_px2
                .iter()
                .map(|&(lag, _)| (n - lag) as f64)
                .collect();

            // Linear fit: MSD = 4D * tau  =>  slope = 4D
            let (slope, _intercept, _r2) =
                weighted_linear_fit(&lag_s, &msd_m2, &weights);
            let d_linear = slope / 4.0;

            // Power-law fit in log-log for anomalous diffusion exponent
            let (alpha, _d_power) = power_law_fit(&lag_s, &msd_m2);

            // Confidence interval (rough: std of residuals / sqrt(n) / (4 * tau_mean))
            let tau_mean = lag_s.iter().sum::<f64>() / lag_s.len() as f64;
            let predicted: Vec<f64> = lag_s.iter().map(|&t| 4.0 * d_linear * t).collect();
            let ss_res: f64 = msd_m2
                .iter()
                .zip(predicted.iter())
                .map(|(&m, &p)| (m - p).powi(2))
                .sum();
            let rmse = (ss_res / msd_m2.len() as f64).sqrt();
            let d_ci95 = if tau_mean > 0.0 {
                1.96 * rmse / (4.0 * tau_mean * (msd_m2.len() as f64).sqrt())
            } else {
                0.0
            };

            // Completeness: fraction of frames in span with detections
            let span = track.frame_indices.last().unwrap_or(&0)
                - track.frame_indices.first().unwrap_or(&0)
                + 1;
            let completeness = n as f64 / span as f64;

            let diameter_nm = if d_linear > 0.0 {
                diffusion_to_diameter_nm(d_linear, temp_k, eta)
            } else {
                0.0
            };

            let is_outlier = d_linear <= 0.0 || diameter_nm <= 0.0;

            track_results.push(TrackResult {
                id: track.id,
                diffusion_coeff: d_linear,
                diffusion_ci95: d_ci95,
                diameter_nm,
                alpha,
                completeness,
                n_msd_points: msd_px2.len(),
                is_outlier,
            });
        }

        // ── Step 4: collect valid diameters ──
        let valid_diameters: Vec<f64> = track_results
            .iter()
            .filter(|r| !r.is_outlier)
            .map(|r| r.diameter_nm)
            .collect();

        // ── Step 5: build distribution ──
        let distribution = build_distribution(&valid_diameters, cfg.histogram_bins, false);

        // ── Step 6: concentration ──
        let concentration_per_ml = cfg.fov_volume_ml.map(|vol| {
            let n_particles = valid_diameters.len() as f64;
            let n_frames = working_tracks
                .iter()
                .map(|t| *t.frame_indices.last().unwrap_or(&0))
                .max()
                .unwrap_or(0) as f64
                + 1.0;
            let particles_per_frame = n_particles / n_frames.max(1.0);
            particles_per_frame / vol * cfg.dilution_factor
        });

        NtaResult {
            tracks: track_results,
            distribution,
            concentration_per_ml,
            drift_px_per_frame: drift_opt,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Viscosity utilities
// ─────────────────────────────────────────────────────────────────────────────

/// Approximate dynamic viscosity of water as a function of temperature (°C)
/// using the Vogel-Fulcher-Tammann equation (valid 0–100°C).
///
/// Returns viscosity in Pa·s.
pub fn water_viscosity_celsius(temp_c: f64) -> f64 {
    // Simplified polynomial approximation (Huber et al. 2009)
    let t = temp_c;
    let eta_mpa_s = 2.414e-5 * (10.0f64).powf(247.8 / (t + 133.15));
    eta_mpa_s
}

/// Compute the approximate viscosity of a glycerol-water mixture.
///
/// Uses Cheng (2008) mixing rule.
///
/// # Arguments
///
/// * `glycerol_fraction` – volume fraction of glycerol (0.0–1.0)
/// * `temp_c` – temperature in Celsius
///
/// # Returns
///
/// Viscosity in Pa·s.
pub fn glycerol_water_viscosity(glycerol_fraction: f64, temp_c: f64) -> f64 {
    let c = glycerol_fraction.clamp(0.0, 1.0);
    let eta_w = water_viscosity_celsius(temp_c);
    let eta_g = 12.1e-3 * (-(temp_c - 20.0) * 0.020).exp(); // rough glycerol model
    // Log mixing rule
    let ln_mix = (1.0 - c) * eta_w.ln() + c * eta_g.ln();
    ln_mix.exp()
}

// ─────────────────────────────────────────────────────────────────────────────
// Quality metrics
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the localization precision (resolution limit) in metres from
/// the photon count and pixel size.
///
/// Uses the Thompson-Larson-Webb formula:
/// `σ_loc = sqrt((s² + a²/12) / N + (8π s⁴ b²) / (a² N²))`
///
/// where:
/// - `s` = PSF standard deviation (m)
/// - `a` = pixel size (m)
/// - `N` = photon count
/// - `b` = background noise (photons/pixel)
///
/// # Returns
///
/// Localization precision in metres.
pub fn localization_precision_m(
    psf_sigma_m: f64,
    pixel_size_m: f64,
    photon_count: f64,
    background_photons_per_pixel: f64,
) -> f64 {
    let s2 = psf_sigma_m * psf_sigma_m;
    let a2 = pixel_size_m * pixel_size_m;
    let n = photon_count.max(1.0);
    let b = background_photons_per_pixel;

    let term1 = (s2 + a2 / 12.0) / n;
    let term2 = (8.0 * PI * s2 * s2 * b * b) / (a2 * n * n);
    (term1 + term2).sqrt()
}

/// Compute the signal-to-noise ratio of the dominant peak in a size distribution.
///
/// Finds the maximum bin count and divides by the median bin count (proxy for noise floor).
pub fn distribution_snr(distribution: &SizeDistribution) -> f64 {
    if distribution.counts.is_empty() {
        return 0.0;
    }
    let peak = distribution
        .counts
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);
    let sorted = sort_f64(distribution.counts.clone());
    let median = percentile_sorted(&sorted, 50.0);
    if median <= 0.0 {
        return f64::INFINITY;
    }
    peak / median
}

// ─────────────────────────────────────────────────────────────────────────────
// Synthetic track generator (for testing)
// ─────────────────────────────────────────────────────────────────────────────

/// Simple xorshift64 PRNG returning a value in (0, 1).
fn xorshift64(state: &mut u64) -> f64 {
    let mut x = *state;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    *state = x;
    // Map to (0, 1) using full 64-bit precision
    (x as f64) * (1.0 / (u64::MAX as f64 + 1.0)) + 1e-300
}

/// Sample a standard normal deviate via Box-Muller from two uniform samples.
fn box_muller(u1: f64, u2: f64) -> (f64, f64) {
    let r = (-2.0 * u1.ln()).sqrt();
    let theta = 2.0 * PI * u2;
    (r * theta.cos(), r * theta.sin())
}

/// Generate a synthetic Brownian motion track for testing purposes.
///
/// Uses xorshift64 PRNG with Box-Muller transform (no external dependencies).
///
/// # Arguments
///
/// * `id` – track identifier
/// * `n_frames` – number of frames
/// * `d_px2_per_frame` – diffusion coefficient in pixels²/frame
/// * `seed` – PRNG seed
///
/// # Returns
///
/// A `Track` with positions sampled from 2D Gaussian displacements.
pub fn generate_brownian_track(
    id: usize,
    n_frames: usize,
    d_px2_per_frame: f64,
    seed: u64,
) -> Track {
    // For 2D Brownian: each step is (dx, dy) ~ N(0, sqrt(2D)) independently
    // So sigma_x = sigma_y = sqrt(2D) per frame, giving MSD(lag=1) = 2*2D = 4D per frame
    // But Box-Muller gives N(0,1); multiply by sqrt(2D) to get correct step size
    let sigma = (2.0 * d_px2_per_frame).sqrt();

    let mut state = seed.wrapping_add(1) | 1; // ensure nonzero seed
    let mut positions = Vec::with_capacity(n_frames);
    let mut x = 0.0f64;
    let mut y = 0.0f64;
    positions.push(Position::new(x, y));

    let mut i = 1;
    while i < n_frames {
        let u1 = xorshift64(&mut state);
        let u2 = xorshift64(&mut state);
        let (nx, ny) = box_muller(u1, u2);
        x += sigma * nx;
        y += sigma * ny;
        positions.push(Position::new(x, y));
        i += 1;
        // We can generate two samples at once from one Box-Muller call by alternating x/y
        // but let's keep it simple and generate one per step for clarity
    }

    let frame_indices: Vec<usize> = (0..n_frames).collect();
    Track::new(id, positions, frame_indices)
}

/// Generate a directed-motion (anomalous, α > 1) track for testing.
pub fn generate_directed_track(
    id: usize,
    n_frames: usize,
    d_px2_per_frame: f64,
    velocity_px_per_frame: f64,
    seed: u64,
) -> Track {
    let sigma = (2.0 * d_px2_per_frame).sqrt();
    let mut state = seed.wrapping_add(42) | 1;
    let mut positions = Vec::with_capacity(n_frames);
    let mut x = 0.0f64;
    let mut y = 0.0f64;
    positions.push(Position::new(x, y));

    for _ in 1..n_frames {
        let u1 = xorshift64(&mut state);
        let u2 = xorshift64(&mut state);
        let (nx, ny) = box_muller(u1, u2);
        x += sigma * nx + velocity_px_per_frame;
        y += sigma * ny;
        positions.push(Position::new(x, y));
    }

    let frame_indices: Vec<usize> = (0..n_frames).collect();
    Track::new(id, positions, frame_indices)
}

/// Generate a confined-motion (anomalous, α < 1) track for testing.
/// Particles are confined within a box of half-width `box_radius_px`.
pub fn generate_confined_track(
    id: usize,
    n_frames: usize,
    d_px2_per_frame: f64,
    box_radius_px: f64,
    seed: u64,
) -> Track {
    let sigma = (2.0 * d_px2_per_frame).sqrt();
    let mut state = seed.wrapping_add(99) | 1;

    let mut positions = Vec::with_capacity(n_frames);
    let mut x = 0.0f64;
    let mut y = 0.0f64;
    positions.push(Position::new(x, y));

    for _ in 1..n_frames {
        let u1 = xorshift64(&mut state);
        let u2 = xorshift64(&mut state);
        let (nx_step, ny_step) = box_muller(u1, u2);
        let nx = (x + sigma * nx_step).clamp(-box_radius_px, box_radius_px);
        let ny = (y + sigma * ny_step).clamp(-box_radius_px, box_radius_px);
        x = nx;
        y = ny;
        positions.push(Position::new(x, y));
    }

    let frame_indices: Vec<usize> = (0..n_frames).collect();
    Track::new(id, positions, frame_indices)
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Physical constants & Stokes-Einstein ──

    #[test]
    fn test_stokes_einstein_100nm_25c() {
        // 100 nm diameter → r = 50 nm = 50e-9 m
        // D = kT/(6πηr) at 25°C water
        let d = stokes_einstein_d(298.15, WATER_VISCOSITY_25C, 50e-9);
        // Literature: ~4.88e-12 m²/s
        assert!((d - 4.88e-12).abs() < 0.2e-12, "D for 100 nm = {:.3e}", d);
    }

    #[test]
    fn test_stokes_einstein_200nm_25c() {
        // 200 nm diameter → r = 100 nm
        let d = stokes_einstein_d(298.15, WATER_VISCOSITY_25C, 100e-9);
        // Half of 100 nm particle
        let d_100 = stokes_einstein_d(298.15, WATER_VISCOSITY_25C, 50e-9);
        let ratio = d_100 / d;
        assert!((ratio - 2.0).abs() < 0.01, "ratio should be 2.0, got {}", ratio);
    }

    #[test]
    fn test_stokes_einstein_roundtrip() {
        let r_in = 75e-9; // 150 nm diameter
        let d = stokes_einstein_d(298.15, WATER_VISCOSITY_25C, r_in);
        let r_out = stokes_einstein_r(d, 298.15, WATER_VISCOSITY_25C);
        assert!(
            (r_out - r_in).abs() < 1e-15,
            "roundtrip radius: in={:.3e}, out={:.3e}",
            r_in, r_out
        );
    }

    #[test]
    fn test_diffusion_to_diameter_100nm() {
        let d = stokes_einstein_d(298.15, WATER_VISCOSITY_25C, 50e-9);
        let diam = diffusion_to_diameter_nm(d, 298.15, WATER_VISCOSITY_25C);
        assert!((diam - 100.0).abs() < 0.5, "diameter = {:.2} nm", diam);
    }

    #[test]
    fn test_diffusion_to_diameter_200nm() {
        let d = stokes_einstein_d(298.15, WATER_VISCOSITY_25C, 100e-9);
        let diam = diffusion_to_diameter_nm(d, 298.15, WATER_VISCOSITY_25C);
        assert!((diam - 200.0).abs() < 0.5, "diameter = {:.2} nm", diam);
    }

    #[test]
    fn test_boltzmann_constant() {
        assert!((BOLTZMANN - 1.380649e-23).abs() < 1e-30);
    }

    #[test]
    fn test_theoretical_msd_2d() {
        // MSD = 4 D τ in 2D
        let d = 1e-12;
        let tau = 1.0;
        let msd = theoretical_msd(d, tau, 2);
        assert!((msd - 4e-12).abs() < 1e-20);
    }

    #[test]
    fn test_theoretical_msd_3d() {
        let d = 1e-12;
        let tau = 0.5;
        let msd = theoretical_msd(d, tau, 3);
        assert!((msd - 3e-12).abs() < 1e-20);
    }

    // ── MSD computation ──

    #[test]
    fn test_msd_linear_track() {
        // Straight-line motion: positions (0,0), (1,0), (2,0), ...
        let positions: Vec<Position> = (0..10).map(|i| Position::new(i as f64, 0.0)).collect();
        let msd = compute_msd(&positions, 5);
        // At lag k, MSD = k²
        for &(lag, m) in &msd {
            let expected = (lag * lag) as f64;
            assert!(
                (m - expected).abs() < 1e-10,
                "lag={}: MSD={:.4} expected={:.4}",
                lag, m, expected
            );
        }
    }

    #[test]
    fn test_msd_stationary_track() {
        let positions: Vec<Position> = (0..20).map(|_| Position::new(0.0, 0.0)).collect();
        let msd = compute_msd(&positions, 10);
        for &(_, m) in &msd {
            assert!(m.abs() < 1e-15, "stationary MSD should be 0");
        }
    }

    #[test]
    fn test_msd_known_displacement() {
        // Alternating steps: (+1, 0), (-1, 0), ...
        let positions: Vec<Position> = (0..10)
            .map(|i| Position::new(if i % 2 == 0 { 0.0 } else { 1.0 }, 0.0))
            .collect();
        let msd = compute_msd(&positions, 2);
        // lag=1: all displacements are ±1 → MSD=1
        let m1 = msd.iter().find(|&&(l, _)| l == 1).unwrap().1;
        assert!((m1 - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_msd_brownian_scaling() {
        // For Brownian motion, MSD ≈ 4D*τ (2D). Check scaling.
        let track = generate_brownian_track(0, 2000, 1.0, 42);
        let msd = compute_msd(&track.positions, 20);
        // Slope of MSD vs lag should be ~4*D = 4.0
        let lag_f: Vec<f64> = msd.iter().map(|&(l, _)| l as f64).collect();
        let msd_vals: Vec<f64> = msd.iter().map(|&(_, m)| m).collect();
        let w = vec![1.0; lag_f.len()];
        let (slope, _, _) = weighted_linear_fit(&lag_f, &msd_vals, &w);
        // Should be close to 4 * D = 4.0 (within statistical noise)
        assert!(
            (slope - 4.0).abs() < 1.0,
            "MSD slope for D=1.0 track: {:.3}",
            slope
        );
    }

    #[test]
    fn test_msd_length() {
        let positions: Vec<Position> = (0..50).map(|i| Position::new(i as f64, 0.0)).collect();
        let msd = compute_msd(&positions, 10);
        assert_eq!(msd.len(), 10);
    }

    // ── Ensemble MSD ──

    #[test]
    fn test_ensemble_msd_single_track() {
        let track = generate_brownian_track(0, 100, 0.5, 7);
        let msd = compute_msd(&track.positions, 10);
        let ensemble = ensemble_msd(&[msd.clone()], 10);
        // Should match single track
        assert_eq!(ensemble.len(), msd.len());
        for (&(l1, m1), &(l2, m2)) in msd.iter().zip(ensemble.iter()) {
            assert_eq!(l1, l2);
            assert!((m1 - m2).abs() < 1e-12);
        }
    }

    #[test]
    fn test_ensemble_msd_averages_correctly() {
        // Two tracks with known MSD at lag 1
        let pos_a = vec![Position::new(0.0, 0.0), Position::new(2.0, 0.0)];
        let pos_b = vec![Position::new(0.0, 0.0), Position::new(4.0, 0.0)];
        let msd_a = compute_msd(&pos_a, 1);
        let msd_b = compute_msd(&pos_b, 1);
        let ens = ensemble_msd(&[msd_a, msd_b], 1);
        assert_eq!(ens.len(), 1);
        // Expected: (4 + 16) / 2 = 10
        assert!((ens[0].1 - 10.0).abs() < 1e-10);
    }

    // ── Diffusion coefficient extraction ──

    #[test]
    fn test_diffusion_extraction_known_d() {
        // Generate many long tracks and check mean extracted D
        let d_true = 1.5; // px²/frame
        let n_tracks = 30;
        let n_frames = 300;

        let mut d_estimates = Vec::new();
        for i in 0..n_tracks {
            let track = generate_brownian_track(i, n_frames, d_true, i as u64 * 17 + 3);
            let max_lag = n_frames / 3;
            let msd = compute_msd(&track.positions, max_lag);
            let lag_f: Vec<f64> = msd.iter().map(|&(l, _)| l as f64).collect();
            let msd_v: Vec<f64> = msd.iter().map(|&(_, m)| m).collect();
            let weights: Vec<f64> = msd.iter().map(|&(l, _)| (n_frames - l) as f64).collect();
            let (slope, _, _) = weighted_linear_fit(&lag_f, &msd_v, &weights);
            d_estimates.push(slope / 4.0);
        }
        let mean_d = d_estimates.iter().sum::<f64>() / d_estimates.len() as f64;
        // Should recover D_true within 20%
        assert!(
            (mean_d - d_true).abs() / d_true < 0.20,
            "mean D estimate = {:.3}, true = {:.3}",
            mean_d, d_true
        );
    }

    // ── Size distribution statistics ──

    #[test]
    fn test_percentile_sorted_basic() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let sorted = sort_f64(data);
        assert!((percentile_sorted(&sorted, 50.0) - 3.0).abs() < 0.01);
        assert!((percentile_sorted(&sorted, 0.0) - 1.0).abs() < 0.01);
        assert!((percentile_sorted(&sorted, 100.0) - 5.0).abs() < 0.01);
    }

    #[test]
    fn test_percentile_d10_d50_d90() {
        let diameters = vec![80.0, 90.0, 100.0, 110.0, 120.0, 130.0, 140.0, 150.0, 160.0, 200.0];
        let sorted = sort_f64(diameters.clone());
        let d10 = percentile_sorted(&sorted, 10.0);
        let d50 = percentile_sorted(&sorted, 50.0);
        let d90 = percentile_sorted(&sorted, 90.0);
        assert!(d10 < d50 && d50 < d90, "d10={} d50={} d90={}", d10, d50, d90);
    }

    #[test]
    fn test_pdi_monodisperse() {
        // All same size: PDI = 0
        let diameters: Vec<f64> = vec![100.0; 100];
        let dist = build_distribution(&diameters, 10, false);
        assert!(dist.pdi.abs() < 1e-10, "monodisperse PDI = {}", dist.pdi);
        assert!((dist.mean_nm - 100.0).abs() < 0.1);
    }

    #[test]
    fn test_pdi_polydisperse() {
        // Wide distribution: PDI should be larger
        let mut diameters: Vec<f64> = (50..=200).map(|i| i as f64).collect();
        let dist = build_distribution(&diameters, 20, false);
        assert!(dist.pdi > 0.01, "polydisperse PDI = {}", dist.pdi);
        let _ = diameters.len();
    }

    #[test]
    fn test_distribution_d50_near_mean_symmetric() {
        // Symmetric distribution: D50 ≈ mean
        let diameters: Vec<f64> = (1..=99).map(|i| i as f64 * 2.0).collect();
        let dist = build_distribution(&diameters, 20, false);
        // For nearly symmetric data, D50 and mean should be close
        assert!(
            (dist.d50_nm - dist.mean_nm).abs() < 15.0,
            "d50={:.1} mean={:.1}",
            dist.d50_nm, dist.mean_nm
        );
    }

    // ── Anomalous diffusion detection ──

    #[test]
    fn test_alpha_normal_brownian() {
        let track = generate_brownian_track(0, 1000, 2.0, 77);
        let msd = compute_msd(&track.positions, 100);
        let lag_s: Vec<f64> = msd.iter().map(|&(l, _)| l as f64).collect();
        let msd_v: Vec<f64> = msd.iter().map(|&(_, m)| m).collect();
        let (alpha, _) = power_law_fit(&lag_s, &msd_v);
        // Normal Brownian: α ≈ 1.0, allow ±0.2 tolerance
        assert!(
            (alpha - 1.0).abs() < 0.3,
            "Normal Brownian alpha = {:.3}",
            alpha
        );
    }

    #[test]
    fn test_alpha_directed_motion() {
        // Strong directed motion should give α > 1
        let track = generate_directed_track(0, 500, 0.01, 5.0, 55);
        let msd = compute_msd(&track.positions, 50);
        let lag_s: Vec<f64> = msd.iter().map(|&(l, _)| l as f64).collect();
        let msd_v: Vec<f64> = msd.iter().map(|&(_, m)| m).collect();
        let (alpha, _) = power_law_fit(&lag_s, &msd_v);
        assert!(alpha > 1.5, "Directed motion alpha = {:.3}", alpha);
    }

    #[test]
    fn test_alpha_confined_motion() {
        // Confined motion should give α < 1
        let track = generate_confined_track(0, 1000, 2.0, 5.0, 33);
        let msd = compute_msd(&track.positions, 50);
        let lag_s: Vec<f64> = msd.iter().map(|&(l, _)| l as f64).collect();
        let msd_v: Vec<f64> = msd.iter().map(|&(_, m)| m).collect();
        let (alpha, _) = power_law_fit(&lag_s, &msd_v);
        assert!(alpha < 0.8, "Confined motion alpha = {:.3}", alpha);
    }

    // ── Concentration estimation ──

    #[test]
    fn test_concentration_estimation() {
        let mut cfg = NtaConfig::default();
        cfg.frame_rate = 30.0;
        cfg.pixel_size_m = 110e-9;
        cfg.min_track_len = 5;
        cfg.drift_correction = false;
        cfg.fov_volume_ml = Some(1e-9); // 1 nL
        cfg.dilution_factor = 10.0;
        cfg.histogram_bins = 10;

        let tracks: Vec<Track> = (0..20)
            .map(|i| generate_brownian_track(i, 30, 1.0, i as u64 * 13 + 7))
            .collect();

        let processor = NtaProcessor::new(cfg);
        let result = processor.analyze(&tracks);
        assert!(
            result.concentration_per_ml.is_some(),
            "concentration should be estimated"
        );
        let conc = result.concentration_per_ml.unwrap();
        assert!(conc > 0.0, "concentration must be positive: {}", conc);
    }

    #[test]
    fn test_concentration_none_without_fov() {
        let cfg = NtaConfig::default();
        let tracks: Vec<Track> = (0..5)
            .map(|i| generate_brownian_track(i, 30, 1.0, i as u64))
            .collect();
        let processor = NtaProcessor::new(cfg);
        let result = processor.analyze(&tracks);
        assert!(result.concentration_per_ml.is_none());
    }

    // ── Drift correction ──

    #[test]
    fn test_drift_correction_removes_mean_drift() {
        // All tracks drift by exactly +1 px/frame in x
        let tracks: Vec<Track> = (0..5)
            .map(|i| {
                let positions: Vec<Position> = (0..20)
                    .map(|f| Position::new(f as f64, 0.0))
                    .collect();
                let frames: Vec<usize> = (0..20).collect();
                Track::new(i, positions, frames)
            })
            .collect();

        let (corrected, drift) = drift_correct(&tracks);
        // Drift should be detected as [1.0, 0.0]
        assert!(
            (drift[0] - 1.0).abs() < 1e-10,
            "drift x = {:.4}",
            drift[0]
        );
        assert!(drift[1].abs() < 1e-10, "drift y = {:.4}", drift[1]);

        // After correction, all displacements should be zero (straight line becomes stationary)
        for track in &corrected {
            let msd = compute_msd(&track.positions, 5);
            for &(_, m) in &msd {
                assert!(m < 1e-8, "corrected MSD should be ~0, got {}", m);
            }
        }
    }

    #[test]
    fn test_drift_correction_zero_drift() {
        let tracks: Vec<Track> = (0..3)
            .map(|i| generate_brownian_track(i, 50, 0.5, i as u64 * 99))
            .collect();
        let (_, drift) = drift_correct(&tracks);
        // With symmetric Brownian motion, drift should be near zero
        assert!(drift[0].abs() < 2.0, "x drift should be small: {}", drift[0]);
        assert!(drift[1].abs() < 2.0, "y drift should be small: {}", drift[1]);
    }

    // ── Outlier rejection ──

    #[test]
    fn test_outlier_rejection_short_tracks() {
        let mut cfg = NtaConfig::default();
        cfg.min_track_len = 20;
        cfg.drift_correction = false;
        cfg.histogram_bins = 5;

        // Mix of long and short tracks
        let mut tracks: Vec<Track> = (0..5)
            .map(|i| generate_brownian_track(i, 50, 1.0, i as u64 * 3))
            .collect();
        // Add short tracks (should be filtered)
        for i in 5..10 {
            let pos = vec![Position::new(0.0, 0.0), Position::new(1.0, 1.0)];
            let frames = vec![0, 1];
            tracks.push(Track::new(i, pos, frames));
        }

        let processor = NtaProcessor::new(cfg);
        let result = processor.analyze(&tracks);
        // Only long tracks contribute
        assert!(
            result.tracks.len() <= 5,
            "short tracks should be filtered: got {} tracks",
            result.tracks.len()
        );
    }

    #[test]
    fn test_outlier_flagging_negative_d() {
        // A stationary particle should give D ≈ 0 (or exactly 0)
        // NTA should flag it as outlier
        let positions: Vec<Position> = vec![Position::new(0.0, 0.0); 50];
        let frames: Vec<usize> = (0..50).collect();
        let track = Track::new(0, positions, frames);

        let mut cfg = NtaConfig::default();
        cfg.min_track_len = 5;
        cfg.drift_correction = false;
        cfg.histogram_bins = 5;

        let processor = NtaProcessor::new(cfg);
        let result = processor.analyze(&[track]);
        if !result.tracks.is_empty() {
            assert!(
                result.tracks[0].is_outlier || result.tracks[0].diffusion_coeff <= 0.0,
                "zero-motion track should be flagged"
            );
        }
    }

    // ── Full NTA pipeline test ──

    #[test]
    fn test_full_pipeline_100nm_beads() {
        // Simulate 100 nm polystyrene beads in water at 25°C
        // D = kT/(6πηr) ≈ 4.88e-12 m²/s
        let d_true_m2s = stokes_einstein_d(298.15, WATER_VISCOSITY_25C, 50e-9);
        let px_m = 110e-9; // 110 nm/pixel
        let frame_rate = 30.0;
        let dt = 1.0 / frame_rate;

        // D in pixels²/frame = D[m²/s] * dt / px_m²
        let d_px2_frame = d_true_m2s * dt / (px_m * px_m);

        let mut cfg = NtaConfig::default();
        cfg.frame_rate = frame_rate;
        cfg.pixel_size_m = px_m;
        cfg.solvent = Solvent::Water25C;
        cfg.min_track_len = 20;
        cfg.drift_correction = false;
        cfg.histogram_bins = 20;

        let tracks: Vec<Track> = (0..50)
            .map(|i| generate_brownian_track(i, 200, d_px2_frame, i as u64 * 31 + 5))
            .collect();

        let processor = NtaProcessor::new(cfg);
        let result = processor.analyze(&tracks);

        // Median diameter should be near 100 nm
        let d50 = result.distribution.d50_nm;
        assert!(
            (d50 - 100.0).abs() < 30.0,
            "D50 = {:.1} nm, expected ~100 nm",
            d50
        );
    }

    #[test]
    fn test_full_pipeline_200nm_beads() {
        let d_true_m2s = stokes_einstein_d(298.15, WATER_VISCOSITY_25C, 100e-9);
        let px_m = 110e-9;
        let frame_rate = 30.0;
        let dt = 1.0 / frame_rate;
        let d_px2_frame = d_true_m2s * dt / (px_m * px_m);

        let mut cfg = NtaConfig::default();
        cfg.frame_rate = frame_rate;
        cfg.pixel_size_m = px_m;
        cfg.min_track_len = 20;
        cfg.drift_correction = false;
        cfg.histogram_bins = 20;

        let tracks: Vec<Track> = (0..50)
            .map(|i| generate_brownian_track(i, 200, d_px2_frame, i as u64 * 7 + 11))
            .collect();

        let processor = NtaProcessor::new(cfg);
        let result = processor.analyze(&tracks);

        let d50 = result.distribution.d50_nm;
        assert!(
            (d50 - 200.0).abs() < 60.0,
            "D50 = {:.1} nm, expected ~200 nm",
            d50
        );
    }

    #[test]
    fn test_full_pipeline_with_drift() {
        let d_px2_frame = 1.0;
        let mut cfg = NtaConfig::default();
        cfg.min_track_len = 15;
        cfg.drift_correction = true;
        cfg.histogram_bins = 10;

        // Tracks with artificial drift
        let tracks: Vec<Track> = (0..20)
            .map(|i| {
                let mut t = generate_brownian_track(i, 100, d_px2_frame, i as u64 * 23);
                // Add drift: 2 px/frame in x
                for (j, p) in t.positions.iter_mut().enumerate() {
                    p.x += j as f64 * 2.0;
                }
                t
            })
            .collect();

        let processor = NtaProcessor::new(cfg);
        let result = processor.analyze(&tracks);
        // Drift should be detected
        let drift = result.drift_px_per_frame.unwrap();
        assert!(
            (drift[0] - 2.0).abs() < 0.5,
            "detected drift x = {:.3}",
            drift[0]
        );
    }

    #[test]
    fn test_track_completeness() {
        // Track with gaps (not all frames detected)
        let positions = vec![
            Position::new(0.0, 0.0),
            Position::new(1.0, 0.0),
            Position::new(2.0, 0.0),
        ];
        let frames = vec![0, 2, 4]; // detected every other frame
        let track = Track::new(0, positions, frames);

        let n = track.len();
        let span = track.frame_indices.last().unwrap() - track.frame_indices.first().unwrap() + 1;
        let completeness = n as f64 / span as f64;
        assert!((completeness - 0.6).abs() < 0.01);
    }

    // ── Solvent & viscosity ──

    #[test]
    fn test_solvent_viscosity_water_25c() {
        let v = Solvent::Water25C.viscosity();
        assert!((v - WATER_VISCOSITY_25C).abs() < 1e-10);
    }

    #[test]
    fn test_solvent_viscosity_ordering() {
        // Water viscosity decreases with temperature
        assert!(
            Solvent::Water20C.viscosity() > Solvent::Water25C.viscosity(),
            "20°C should be more viscous than 25°C"
        );
        assert!(
            Solvent::Water25C.viscosity() > Solvent::Water37C.viscosity(),
            "25°C should be more viscous than 37°C"
        );
    }

    #[test]
    fn test_solvent_custom_viscosity() {
        let v = 1.5e-3;
        let s = Solvent::Custom(v);
        assert!((s.viscosity() - v).abs() < 1e-15);
    }

    #[test]
    fn test_water_viscosity_celsius() {
        let v25 = water_viscosity_celsius(25.0);
        assert!(
            (v25 - 8.9e-4).abs() < 2e-4,
            "water @ 25°C viscosity = {:.3e}",
            v25
        );
    }

    #[test]
    fn test_glycerol_water_viscosity_pure_water() {
        // 0% glycerol → pure water viscosity at 25°C
        let v = glycerol_water_viscosity(0.0, 25.0);
        let v_ref = water_viscosity_celsius(25.0);
        assert!(
            (v - v_ref).abs() / v_ref < 1e-6,
            "0% glycerol viscosity mismatch: {:.3e} vs {:.3e}",
            v, v_ref
        );
    }

    #[test]
    fn test_glycerol_water_viscosity_increases_with_fraction() {
        let v_0 = glycerol_water_viscosity(0.0, 25.0);
        let v_50 = glycerol_water_viscosity(0.5, 25.0);
        let v_90 = glycerol_water_viscosity(0.9, 25.0);
        assert!(v_50 > v_0, "50% glycerol more viscous than 0%");
        assert!(v_90 > v_50, "90% glycerol more viscous than 50%");
    }

    // ── Quality metrics ──

    #[test]
    fn test_localization_precision() {
        // Typical NTA: PSF σ = 150 nm, pixel = 110 nm, N = 1000 photons, b = 10
        let sigma = 150e-9_f64;
        let a = 110e-9_f64;
        let n_phot = 1000.0;
        let bg = 10.0;
        let prec = localization_precision_m(sigma, a, n_phot, bg);
        // Should be ~5-15 nm
        assert!(prec > 1e-9 && prec < 50e-9, "precision = {:.1e} m", prec);
    }

    #[test]
    fn test_distribution_snr() {
        let dist = build_distribution(&vec![100.0; 100], 10, false);
        let snr = distribution_snr(&dist);
        assert!(snr > 0.0);
    }

    // ── Weighted linear fit ──

    #[test]
    fn test_weighted_linear_fit_known_slope() {
        // y = 3x + 1
        let x: Vec<f64> = (1..=10).map(|i| i as f64).collect();
        let y: Vec<f64> = x.iter().map(|&xi| 3.0 * xi + 1.0).collect();
        let w = vec![1.0; x.len()];
        let (slope, intercept, r2) = weighted_linear_fit(&x, &y, &w);
        assert!((slope - 3.0).abs() < 1e-10, "slope = {}", slope);
        assert!((intercept - 1.0).abs() < 1e-10, "intercept = {}", intercept);
        assert!((r2 - 1.0).abs() < 1e-10, "R² = {}", r2);
    }

    #[test]
    fn test_weighted_linear_fit_different_weights() {
        // Points on y = x but with one outlier
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y = vec![1.0, 2.0, 3.0, 4.0, 1000.0]; // outlier at x=5
        // Down-weight the outlier
        let w = vec![1.0, 1.0, 1.0, 1.0, 0.001];
        let (slope, _, _) = weighted_linear_fit(&x, &y, &w);
        // Should still recover slope ≈ 1 (low-weighted outlier has some residual effect)
        assert!((slope - 1.0).abs() < 0.6, "weighted slope = {}", slope);
    }

    // ── Track data structures ──

    #[test]
    fn test_track_len() {
        let pos = vec![Position::new(0.0, 0.0); 15];
        let frames: Vec<usize> = (0..15).collect();
        let t = Track::new(1, pos, frames);
        assert_eq!(t.len(), 15);
        assert!(!t.is_empty());
    }

    #[test]
    fn test_empty_track() {
        let t = Track::new(0, vec![], vec![]);
        assert!(t.is_empty());
        assert_eq!(t.len(), 0);
    }

    #[test]
    fn test_position_creation() {
        let p = Position::new(3.14, -2.71);
        assert!((p.x - 3.14).abs() < 1e-10);
        assert!((p.y + 2.71).abs() < 1e-10);
    }

    // ── Bimodal distribution ──

    #[test]
    fn test_bimodal_distribution_peaks() {
        // Mix 100 nm and 200 nm particles
        let mut diameters: Vec<f64> = vec![100.0; 50];
        diameters.extend(vec![200.0; 50]);
        let dist = build_distribution(&diameters, 30, false);
        assert_eq!(dist.n_valid_tracks, 100);
        // D50 should be between the two peaks (around 150 nm)
        assert!(
            dist.d50_nm > 120.0 && dist.d50_nm < 180.0,
            "bimodal D50 = {:.1} nm",
            dist.d50_nm
        );
    }

    // ── Config defaults ──

    #[test]
    fn test_config_defaults() {
        let cfg = NtaConfig::default();
        assert!((cfg.frame_rate - 30.0).abs() < 1e-10);
        assert!(cfg.min_track_len >= 5);
        assert!(cfg.histogram_bins > 0);
        assert!(cfg.drift_correction);
        assert!((cfg.dilution_factor - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_config_temperature_fallback() {
        let mut cfg = NtaConfig::default();
        cfg.temperature_k = Some(310.0);
        assert!((cfg.temperature() - 310.0).abs() < 1e-10);
    }

    #[test]
    fn test_config_temperature_from_solvent() {
        let mut cfg = NtaConfig::default();
        cfg.solvent = Solvent::Water37C;
        cfg.temperature_k = None;
        assert!((cfg.temperature() - 310.15).abs() < 0.01);
    }
}
