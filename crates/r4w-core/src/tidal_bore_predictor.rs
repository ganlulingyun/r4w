//! # Tidal Bore Propagation Predictor
//!
//! This module implements tidal bore propagation modeling from tide gauge signals.
//! Tidal bores are surge waves that travel up rivers when incoming tides are funneled
//! into narrowing estuarine channels. Famous examples include the Severn Bore (UK),
//! the Qiantang River bore (China), and the Mascaret on the Dordogne (France).
//!
//! ## Physics Background
//!
//! A tidal bore forms when the flood tide propagates into a funnel-shaped estuary
//! with a large tidal range and shallow depth. The bore is essentially a hydraulic
//! jump that moves upstream. The key dimensionless parameter is the Froude number:
//!
//! ```text
//! Fr = (V_bore + V_river) / sqrt(g * h_upstream)
//! ```
//!
//! where `V_bore` is the bore propagation speed, `V_river` is the river current
//! (opposing the bore), `g` is gravitational acceleration, and `h_upstream` is the
//! undisturbed upstream water depth.
//!
//! ## Bore Classification
//!
//! - **Undular bore** (Fr < ~1.5): A train of smooth waves (whelps) with no breaking.
//! - **Breaking bore** (Fr > ~1.5): A turbulent wall of water with white water at the front.
//! - **Transitional**: Near the critical Froude number, showing partial breaking.
//!
//! ## Key Equations
//!
//! **Belanger equation** (conjugate depth ratio):
//! ```text
//! h2/h1 = 0.5 * (sqrt(1 + 8*Fr^2) - 1)
//! ```
//!
//! **Energy dissipation** at bore front:
//! ```text
//! dE = g * (h2 - h1)^3 / (4 * h1 * h2)
//! ```
//!
//! **Manning's equation** for open channel flow:
//! ```text
//! V = (1/n) * R_h^(2/3) * S^(1/2)
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::tidal_bore_predictor::{BoreConfig, TidalBorePredictor, BoreType};
//!
//! let config = BoreConfig {
//!     channel_width_m: 200.0,
//!     channel_depth_m: 3.0,
//!     tidal_range_m: 10.0,
//!     river_flow_m3s: 50.0,
//!     channel_slope: 1e-4,
//!     bed_roughness_manning: 0.03,
//!     convergence_rate: 0.01,
//! };
//!
//! let predictor = TidalBorePredictor::new(config);
//!
//! // Shallow water wave speed at 3m depth
//! let c = predictor.shallow_water_speed(3.0);
//! assert!((c - 5.42).abs() < 0.1);
//!
//! // Froude number for a fast bore
//! let fr = predictor.froude_number(8.0, 3.0);
//! assert!(fr > 1.0);
//!
//! // Classify bore type
//! let bore_type = predictor.classify_bore(fr);
//! ```

/// Standard gravitational acceleration in m/s^2 (WGS-84 standard gravity).
pub fn gravitational_acceleration() -> f64 {
    9.80665
}

/// Compute conjugate (sequent) depth from the Belanger equation.
///
/// Given upstream depth `h1` and Froude number `froude`, returns the
/// downstream depth `h2` after the hydraulic jump / bore front.
///
/// # Formula
///
/// ```text
/// h2 = h1 * 0.5 * (sqrt(1 + 8 * Fr^2) - 1)
/// ```
///
/// # Arguments
///
/// * `h1` - Upstream (undisturbed) depth in meters. Must be positive.
/// * `froude` - Froude number of the bore. Must be >= 1.0 for a bore to exist.
///
/// # Returns
///
/// Conjugate depth `h2` in meters.
pub fn conjugate_depth(h1: f64, froude: f64) -> f64 {
    h1 * 0.5 * ((1.0 + 8.0 * froude * froude).sqrt() - 1.0)
}

/// Compute specific energy of open channel flow.
///
/// Specific energy is the total energy per unit weight of water measured
/// relative to the channel bed.
///
/// # Formula
///
/// ```text
/// E = h + v^2 / (2 * g)
/// ```
///
/// # Arguments
///
/// * `depth` - Water depth in meters.
/// * `velocity` - Flow velocity in m/s.
///
/// # Returns
///
/// Specific energy in meters.
pub fn specific_energy(depth: f64, velocity: f64) -> f64 {
    let g = gravitational_acceleration();
    depth + velocity * velocity / (2.0 * g)
}

/// Compute critical depth for a given discharge per unit width.
///
/// At critical depth, the Froude number equals 1 and specific energy is minimized
/// for the given discharge.
///
/// # Formula
///
/// ```text
/// h_c = (q^2 / g)^(1/3)
/// ```
///
/// where `q` is discharge per unit width (m^2/s).
///
/// # Arguments
///
/// * `discharge_per_width` - Volumetric flow rate per unit channel width (m^2/s).
///
/// # Returns
///
/// Critical depth in meters.
pub fn critical_depth(discharge_per_width: f64) -> f64 {
    let g = gravitational_acceleration();
    (discharge_per_width * discharge_per_width / g).powf(1.0 / 3.0)
}

// ---------------------------------------------------------------------------
// BoreConfig
// ---------------------------------------------------------------------------

/// Configuration parameters for tidal bore prediction.
///
/// These describe the estuarine channel geometry and hydraulic conditions
/// that determine whether a tidal bore forms and how it propagates.
#[derive(Debug, Clone)]
pub struct BoreConfig {
    /// Channel width at the reference cross-section (meters).
    pub channel_width_m: f64,
    /// Mean channel depth at the reference cross-section (meters).
    pub channel_depth_m: f64,
    /// Tidal range at the estuary mouth (meters). A larger tidal range
    /// increases the likelihood and strength of bore formation.
    pub tidal_range_m: f64,
    /// River discharge (m^3/s). Opposing river flow increases the Froude number.
    pub river_flow_m3s: f64,
    /// Longitudinal bed slope (dimensionless). Typically ~1e-4 for tidal rivers.
    pub channel_slope: f64,
    /// Manning's roughness coefficient (s/m^(1/3)). Typical values:
    /// - 0.025: clean, straight channel
    /// - 0.030: winding channel with some weeds
    /// - 0.040: rough channel with stones and vegetation
    pub bed_roughness_manning: f64,
    /// Rate of channel width convergence (dimensionless). Describes how quickly
    /// the estuary narrows upstream. Higher values produce stronger bores.
    pub convergence_rate: f64,
}

// ---------------------------------------------------------------------------
// BoreType
// ---------------------------------------------------------------------------

/// Classification of tidal bore based on the Froude number.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoreType {
    /// Weak bore (Fr between 1.0 and 1.5) characterized by a train of
    /// undular waves (whelps) behind the bore front with no breaking.
    Undular,
    /// Strong bore (Fr > 1.5) with a steep, turbulent, breaking front.
    Breaking,
    /// Near the critical Froude number (~1.5), the bore shows partial
    /// breaking characteristics with some undulations.
    Transitional,
}

// ---------------------------------------------------------------------------
// TidalBorePredictor
// ---------------------------------------------------------------------------

/// Predictor for tidal bore formation and propagation.
///
/// Uses shallow water wave theory, the Belanger conjugate depth equation,
/// and the Froude number criterion to predict bore characteristics.
pub struct TidalBorePredictor {
    config: BoreConfig,
}

impl TidalBorePredictor {
    /// Create a new predictor from the given channel and tidal configuration.
    pub fn new(config: BoreConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the configuration.
    pub fn config(&self) -> &BoreConfig {
        &self.config
    }

    /// Compute shallow water (long wave) speed at a given depth.
    ///
    /// # Formula
    ///
    /// ```text
    /// c = sqrt(g * h)
    /// ```
    ///
    /// # Arguments
    ///
    /// * `depth_m` - Water depth in meters. Must be non-negative.
    ///
    /// # Returns
    ///
    /// Wave celerity in m/s.
    pub fn shallow_water_speed(&self, depth_m: f64) -> f64 {
        let g = gravitational_acceleration();
        (g * depth_m).sqrt()
    }

    /// Compute the Froude number for a bore.
    ///
    /// The Froude number relates the bore speed (relative to the upstream
    /// water) to the shallow water wave speed.
    ///
    /// # Formula
    ///
    /// ```text
    /// Fr = (V_bore + V_river) / sqrt(g * h)
    /// ```
    ///
    /// Here `bore_speed` is the absolute speed of the bore front upstream,
    /// and the river velocity is computed from the configuration (discharge / area).
    ///
    /// # Arguments
    ///
    /// * `bore_speed` - Absolute bore propagation speed in m/s (positive upstream).
    /// * `depth_m` - Upstream undisturbed depth in meters.
    ///
    /// # Returns
    ///
    /// Froude number (dimensionless). Fr > 1 indicates supercritical bore.
    pub fn froude_number(&self, bore_speed: f64, depth_m: f64) -> f64 {
        let river_velocity =
            self.config.river_flow_m3s / (self.config.channel_width_m * depth_m);
        let c = self.shallow_water_speed(depth_m);
        (bore_speed + river_velocity) / c
    }

    /// Compute bore height (downstream conjugate depth) from the Belanger equation.
    ///
    /// # Arguments
    ///
    /// * `froude` - Froude number of the bore.
    /// * `upstream_depth` - Undisturbed upstream depth in meters.
    ///
    /// # Returns
    ///
    /// Downstream depth `h2` in meters.
    pub fn bore_height(&self, froude: f64, upstream_depth: f64) -> f64 {
        conjugate_depth(upstream_depth, froude)
    }

    /// Compute bore propagation speed from mass conservation across the bore front.
    ///
    /// For a bore propagating into still water of depth `h1` with downstream
    /// depth `h2`, mass conservation gives:
    ///
    /// ```text
    /// V_bore = sqrt(g * h2 * (h1 + h2) / (2 * h1))
    /// ```
    ///
    /// # Arguments
    ///
    /// * `downstream_depth` - Depth behind the bore front (`h2`) in meters.
    /// * `upstream_depth` - Undisturbed depth ahead of bore (`h1`) in meters.
    ///
    /// # Returns
    ///
    /// Bore propagation speed in m/s.
    pub fn bore_speed(&self, downstream_depth: f64, upstream_depth: f64) -> f64 {
        let g = gravitational_acceleration();
        (g * downstream_depth * (upstream_depth + downstream_depth) / (2.0 * upstream_depth))
            .sqrt()
    }

    /// Classify the bore type based on Froude number thresholds.
    ///
    /// - Fr < 1.0: subcritical, no bore forms (still returns `Undular`)
    /// - 1.0 <= Fr < 1.3: undular bore
    /// - 1.3 <= Fr <= 1.5: transitional
    /// - Fr > 1.5: breaking bore
    pub fn classify_bore(&self, froude: f64) -> BoreType {
        if froude <= 1.3 {
            BoreType::Undular
        } else if froude <= 1.5 {
            BoreType::Transitional
        } else {
            BoreType::Breaking
        }
    }

    /// Compute energy dissipation per unit width at the bore front.
    ///
    /// The energy loss in a hydraulic jump / bore is:
    ///
    /// ```text
    /// dE = g * (h2 - h1)^3 / (4 * h1 * h2)
    /// ```
    ///
    /// # Arguments
    ///
    /// * `h1` - Upstream depth in meters.
    /// * `h2` - Downstream depth in meters (h2 > h1 for a bore).
    ///
    /// # Returns
    ///
    /// Energy dissipation rate in m^3/s^2 (energy per unit width per unit weight).
    pub fn energy_dissipation(&self, h1: f64, h2: f64) -> f64 {
        let g = gravitational_acceleration();
        let dh = h2 - h1;
        g * dh * dh * dh / (4.0 * h1 * h2)
    }

    /// Predict bore arrival time by integrating travel time along a channel
    /// with a varying depth profile.
    ///
    /// The bore speed at each segment is approximated as the shallow water
    /// wave speed `sqrt(g * h)` at the local depth. The total travel time is
    /// the sum of `dx / c(x)` for each segment.
    ///
    /// # Arguments
    ///
    /// * `distance_m` - Total distance from estuary mouth to prediction point (meters).
    ///   This is used as a scaling reference but the actual integration uses
    ///   the depth profile.
    /// * `depth_profile` - Slice of `(distance_m, depth_m)` pairs giving the
    ///   channel depth at discrete stations along the river. Must be sorted by
    ///   distance and contain at least 2 points.
    ///
    /// # Returns
    ///
    /// Estimated travel time in seconds, or 0.0 if the profile is insufficient.
    pub fn predict_arrival_time(
        &self,
        _distance_m: f64,
        depth_profile: &[(f64, f64)],
    ) -> f64 {
        if depth_profile.len() < 2 {
            return 0.0;
        }

        let mut total_time = 0.0;
        for i in 1..depth_profile.len() {
            let dx = depth_profile[i].0 - depth_profile[i - 1].0;
            let avg_depth = 0.5 * (depth_profile[i].1 + depth_profile[i - 1].1);
            let c = self.shallow_water_speed(avg_depth);
            if c > 0.0 {
                total_time += dx / c;
            }
        }
        total_time
    }
}

// ---------------------------------------------------------------------------
// TideGaugeProcessor
// ---------------------------------------------------------------------------

/// Processor for tide gauge water level data.
///
/// Provides methods to detect bore passage, measure bore height, perform
/// tidal harmonic analysis, and compute rate-of-rise from water level records.
pub struct TideGaugeProcessor {
    sample_rate_hz: f64,
}

impl TideGaugeProcessor {
    /// Create a new processor for tide gauge data sampled at the given rate.
    ///
    /// # Arguments
    ///
    /// * `sample_rate_hz` - Sampling frequency of the water level sensor (Hz).
    pub fn new(sample_rate_hz: f64) -> Self {
        Self { sample_rate_hz }
    }

    /// Return the configured sample rate.
    pub fn sample_rate_hz(&self) -> f64 {
        self.sample_rate_hz
    }

    /// Detect bore passage as a rapid rise in water level.
    ///
    /// Scans the water level time series for the first sample where the
    /// forward difference exceeds `threshold_m`. This indicates the steep
    /// wavefront of a bore.
    ///
    /// # Arguments
    ///
    /// * `water_level` - Time series of water level measurements (meters).
    /// * `threshold_m` - Minimum rise between consecutive samples to count
    ///   as a bore passage (meters).
    ///
    /// # Returns
    ///
    /// Index of the first sample where the rise exceeds the threshold, or
    /// `None` if no bore passage is detected.
    pub fn detect_bore_passage(
        &self,
        water_level: &[f64],
        threshold_m: f64,
    ) -> Option<usize> {
        if water_level.len() < 2 {
            return None;
        }
        for i in 1..water_level.len() {
            let rise = water_level[i] - water_level[i - 1];
            if rise >= threshold_m {
                return Some(i);
            }
        }
        None
    }

    /// Measure bore height from gauge data at the detected bore index.
    ///
    /// Computes the height as the difference between the post-bore peak
    /// (within a short window after the bore index) and the pre-bore level
    /// (average of a short window before the bore index).
    ///
    /// # Arguments
    ///
    /// * `water_level` - Time series of water level measurements (meters).
    /// * `bore_index` - Index of the bore passage as returned by
    ///   [`detect_bore_passage`].
    ///
    /// # Returns
    ///
    /// Estimated bore height in meters.
    pub fn bore_height_from_gauge(
        &self,
        water_level: &[f64],
        bore_index: usize,
    ) -> f64 {
        if water_level.is_empty() || bore_index >= water_level.len() {
            return 0.0;
        }

        // Pre-bore average: up to 10 samples before bore
        let pre_start = if bore_index >= 10 { bore_index - 10 } else { 0 };
        let pre_end = bore_index;
        let pre_count = pre_end - pre_start;
        let pre_avg = if pre_count > 0 {
            water_level[pre_start..pre_end].iter().sum::<f64>() / pre_count as f64
        } else {
            water_level[bore_index]
        };

        // Post-bore peak: up to 10 samples after bore
        let post_end = (bore_index + 11).min(water_level.len());
        let post_peak = water_level[bore_index..post_end]
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);

        post_peak - pre_avg
    }

    /// Perform tidal harmonic analysis on a water level time series.
    ///
    /// Fits sinusoidal components at specified tidal constituent frequencies
    /// using least-squares (correlation) analysis. For each constituent
    /// frequency, the amplitude and phase are estimated.
    ///
    /// # Arguments
    ///
    /// * `water_level` - Time series of water level measurements (meters).
    /// * `dt_s` - Time step between samples (seconds).
    /// * `constituents` - Slice of tidal constituent frequencies to fit
    ///   (in cycles per hour). Common values:
    ///   - M2: 0.0805114 (principal lunar semidiurnal)
    ///   - S2: 0.0833333 (principal solar semidiurnal)
    ///   - K1: 0.0417807 (lunisolar diurnal)
    ///
    /// # Returns
    ///
    /// Vector of `(frequency, amplitude, phase)` tuples. Frequency in cycles
    /// per hour, amplitude in meters, phase in radians.
    pub fn tidal_harmonic_analysis(
        &self,
        water_level: &[f64],
        dt_s: f64,
        constituents: &[f64],
    ) -> Vec<(f64, f64, f64)> {
        let n = water_level.len();
        if n == 0 || dt_s <= 0.0 {
            return Vec::new();
        }

        let dt_hours = dt_s / 3600.0;

        let mut results = Vec::with_capacity(constituents.len());
        for &freq_cph in constituents {
            let omega = 2.0 * std::f64::consts::PI * freq_cph; // radians per hour
            let mut cos_sum = 0.0;
            let mut sin_sum = 0.0;
            for (i, &level) in water_level.iter().enumerate() {
                let t_hours = i as f64 * dt_hours;
                cos_sum += level * (omega * t_hours).cos();
                sin_sum += level * (omega * t_hours).sin();
            }
            let amplitude =
                2.0 * (cos_sum * cos_sum + sin_sum * sin_sum).sqrt() / n as f64;
            let phase = sin_sum.atan2(cos_sum);
            results.push((freq_cph, amplitude, phase));
        }
        results
    }

    /// Compute rate of rise (dh/dt) from water level data.
    ///
    /// Uses a simple forward difference: dh/dt[i] = (h[i+1] - h[i]) * sample_rate.
    ///
    /// # Arguments
    ///
    /// * `water_level` - Time series of water level measurements (meters).
    /// * `dt_s` - Time step between samples (seconds).
    ///
    /// # Returns
    ///
    /// Vector of rate-of-rise values (m/s), one element shorter than input.
    pub fn rate_of_rise(&self, water_level: &[f64], dt_s: f64) -> Vec<f64> {
        if water_level.len() < 2 || dt_s <= 0.0 {
            return Vec::new();
        }
        water_level
            .windows(2)
            .map(|w| (w[1] - w[0]) / dt_s)
            .collect()
    }

    /// Apply a simple lowpass filter to extract tidal signal components.
    ///
    /// Implements a moving average filter with a window size corresponding to
    /// the specified cutoff period. This smooths out bore-scale fluctuations
    /// while preserving the underlying tidal signal.
    ///
    /// # Arguments
    ///
    /// * `water_level` - Time series of water level measurements (meters).
    /// * `cutoff_hours` - Cutoff period in hours. Components shorter than this
    ///   will be attenuated.
    /// * `dt_s` - Time step between samples (seconds).
    ///
    /// # Returns
    ///
    /// Filtered water level time series (same length as input, with edge effects
    /// at boundaries).
    pub fn filter_tidal_signal(
        &self,
        water_level: &[f64],
        cutoff_hours: f64,
        dt_s: f64,
    ) -> Vec<f64> {
        if water_level.is_empty() || dt_s <= 0.0 || cutoff_hours <= 0.0 {
            return water_level.to_vec();
        }

        let cutoff_seconds = cutoff_hours * 3600.0;
        let window_size = (cutoff_seconds / dt_s).round() as usize;
        let window_size = window_size.max(1);
        let half_win = window_size / 2;

        let n = water_level.len();
        let mut filtered = vec![0.0; n];

        for i in 0..n {
            let start = if i >= half_win { i - half_win } else { 0 };
            let end = (i + half_win + 1).min(n);
            let count = end - start;
            let sum: f64 = water_level[start..end].iter().sum();
            filtered[i] = sum / count as f64;
        }
        filtered
    }
}

// ---------------------------------------------------------------------------
// ShallowWaterSolver
// ---------------------------------------------------------------------------

/// 1-D Shallow Water Equations solver using Lax-Friedrichs scheme.
///
/// Solves the conservative form:
///
/// ```text
/// dh/dt + d(h*u)/dx = 0          (mass conservation)
/// d(h*u)/dt + d(h*u^2 + g*h^2/2)/dx = 0  (momentum conservation)
/// ```
///
/// The Lax-Friedrichs scheme is a first-order explicit finite-volume method
/// that adds sufficient numerical diffusion for stability.
pub struct ShallowWaterSolver {
    /// Grid spacing (meters).
    dx: f64,
    /// Number of computational cells.
    num_cells: usize,
}

impl ShallowWaterSolver {
    /// Create a new solver with the given grid spacing and number of cells.
    ///
    /// # Arguments
    ///
    /// * `dx` - Grid spacing in meters.
    /// * `num_cells` - Number of cells in the computational domain.
    pub fn new(dx: f64, num_cells: usize) -> Self {
        Self { dx, num_cells }
    }

    /// Return the grid spacing.
    pub fn dx(&self) -> f64 {
        self.dx
    }

    /// Return the number of cells.
    pub fn num_cells(&self) -> usize {
        self.num_cells
    }

    /// Set up initial conditions for a dam-break (Riemann) problem.
    ///
    /// The left half of the domain is set to `h_left` and the right half
    /// to `h_right`, with zero velocity everywhere. This is the classic
    /// test problem for shallow water solvers and produces a right-going
    /// shock (bore) and a left-going rarefaction.
    ///
    /// # Arguments
    ///
    /// * `h_left` - Water depth in the left half (meters).
    /// * `h_right` - Water depth in the right half (meters).
    ///
    /// # Returns
    ///
    /// Tuple of `(h, u)` vectors: water depth and velocity at each cell.
    pub fn initial_condition_dam_break(
        &self,
        h_left: f64,
        h_right: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let n = self.num_cells;
        let mid = n / 2;
        let mut h = vec![0.0; n];
        let u = vec![0.0; n];

        for i in 0..n {
            h[i] = if i < mid { h_left } else { h_right };
        }
        (h, u)
    }

    /// Perform one time step of the Lax-Friedrichs scheme for the SWE.
    ///
    /// The Lax-Friedrichs scheme for conservation law `dU/dt + dF/dx = 0` is:
    ///
    /// ```text
    /// U_i^{n+1} = 0.5*(U_{i-1}^n + U_{i+1}^n) - 0.5*(dt/dx)*(F_{i+1}^n - F_{i-1}^n)
    /// ```
    ///
    /// # Arguments
    ///
    /// * `h` - Mutable vector of water depths.
    /// * `u` - Mutable vector of velocities.
    /// * `dt` - Time step (seconds).
    /// * `dx` - Grid spacing (meters).
    /// * `g` - Gravitational acceleration (m/s^2).
    pub fn step_lax_friedrichs(
        &self,
        h: &mut Vec<f64>,
        u: &mut Vec<f64>,
        dt: f64,
        dx: f64,
        g: f64,
    ) {
        let n = h.len();
        if n < 3 {
            return;
        }

        // Conservative variables: q1 = h, q2 = h*u
        // Fluxes: f1 = h*u, f2 = h*u^2 + g*h^2/2
        let q1: Vec<f64> = h.clone();
        let q2: Vec<f64> = h.iter().zip(u.iter()).map(|(&hi, &ui)| hi * ui).collect();

        let f1: Vec<f64> = q2.clone();
        let f2: Vec<f64> = h
            .iter()
            .zip(u.iter())
            .map(|(&hi, &ui)| hi * ui * ui + 0.5 * g * hi * hi)
            .collect();

        let ratio = dt / dx;

        let mut h_new = h.clone();
        let mut q2_new = q2.clone();

        for i in 1..n - 1 {
            h_new[i] = 0.5 * (q1[i - 1] + q1[i + 1]) - 0.5 * ratio * (f1[i + 1] - f1[i - 1]);
            q2_new[i] =
                0.5 * (q2[i - 1] + q2[i + 1]) - 0.5 * ratio * (f2[i + 1] - f2[i - 1]);
        }

        // Boundary: copy from interior (transmissive)
        h_new[0] = h_new[1];
        h_new[n - 1] = h_new[n - 2];
        q2_new[0] = q2_new[1];
        q2_new[n - 1] = q2_new[n - 2];

        // Update
        for i in 0..n {
            h[i] = h_new[i].max(1e-10); // prevent negative depth
            u[i] = if h[i] > 1e-10 {
                q2_new[i] / h[i]
            } else {
                0.0
            };
        }
    }

    /// Run the SWE simulation for a given number of time steps.
    ///
    /// # Arguments
    ///
    /// * `h` - Mutable vector of water depths.
    /// * `u` - Mutable vector of velocities.
    /// * `dt` - Time step per iteration (seconds).
    /// * `num_steps` - Number of time steps to advance.
    pub fn run_simulation(
        &self,
        h: &mut Vec<f64>,
        u: &mut Vec<f64>,
        dt: f64,
        num_steps: usize,
    ) {
        let g = gravitational_acceleration();
        for _ in 0..num_steps {
            self.step_lax_friedrichs(h, u, dt, self.dx, g);
        }
    }

    /// Compute the Courant (CFL) number.
    ///
    /// The CFL condition requires that `C <= 1` for stability of explicit schemes.
    ///
    /// # Formula
    ///
    /// ```text
    /// C = max_speed * dt / dx
    /// ```
    ///
    /// # Arguments
    ///
    /// * `max_speed` - Maximum wave speed in the domain (m/s).
    /// * `dt` - Time step (seconds).
    /// * `dx` - Grid spacing (meters).
    ///
    /// # Returns
    ///
    /// Courant number (dimensionless).
    pub fn courant_number(&self, max_speed: f64, dt: f64, dx: f64) -> f64 {
        max_speed * dt / dx
    }

    /// Compute a stable time step that satisfies the CFL condition.
    ///
    /// # Arguments
    ///
    /// * `max_speed` - Maximum wave speed in the domain (m/s).
    /// * `dx` - Grid spacing (meters).
    /// * `cfl` - Desired CFL number (must be <= 1.0 for stability, typically 0.5-0.9).
    ///
    /// # Returns
    ///
    /// Stable time step in seconds.
    pub fn stable_timestep(&self, max_speed: f64, dx: f64, cfl: f64) -> f64 {
        if max_speed <= 0.0 {
            return 0.0;
        }
        cfl * dx / max_speed
    }
}

// ---------------------------------------------------------------------------
// ChannelGeometry
// ---------------------------------------------------------------------------

/// Describes the varying geometry of a tidal river / estuary channel.
///
/// Stores width and depth at discrete stations along the channel and
/// provides interpolation and hydraulic parameter calculations.
pub struct ChannelGeometry {
    /// Channel widths at each station (meters).
    widths: Vec<f64>,
    /// Channel depths at each station (meters).
    depths: Vec<f64>,
    /// Distance from estuary mouth for each station (meters).
    distances: Vec<f64>,
}

impl ChannelGeometry {
    /// Create a new channel geometry from measured cross-section data.
    ///
    /// # Arguments
    ///
    /// * `widths` - Width at each station (meters).
    /// * `depths` - Depth at each station (meters).
    /// * `distances` - Distance from mouth at each station (meters).
    ///   Must be monotonically increasing and same length as `widths` and `depths`.
    pub fn new(widths: Vec<f64>, depths: Vec<f64>, distances: Vec<f64>) -> Self {
        Self {
            widths,
            depths,
            distances,
        }
    }

    /// Number of measurement stations.
    pub fn num_stations(&self) -> usize {
        self.distances.len()
    }

    /// Interpolate channel width at an arbitrary distance.
    ///
    /// Uses linear interpolation between stations. Returns the boundary
    /// value for distances outside the measured range.
    pub fn width_at(&self, distance: f64) -> f64 {
        Self::interpolate(&self.distances, &self.widths, distance)
    }

    /// Interpolate channel depth at an arbitrary distance.
    ///
    /// Uses linear interpolation between stations.
    pub fn depth_at(&self, distance: f64) -> f64 {
        Self::interpolate(&self.distances, &self.depths, distance)
    }

    /// Compute cross-sectional area at a given distance (rectangular approx).
    ///
    /// ```text
    /// A = width * depth
    /// ```
    pub fn cross_section_area(&self, distance: f64) -> f64 {
        self.width_at(distance) * self.depth_at(distance)
    }

    /// Compute hydraulic radius for a rectangular channel cross-section.
    ///
    /// The hydraulic radius is the ratio of cross-sectional area to wetted
    /// perimeter.
    ///
    /// ```text
    /// R_h = (w * d) / (w + 2d)
    /// ```
    ///
    /// # Arguments
    ///
    /// * `width` - Channel width (meters).
    /// * `depth` - Water depth (meters).
    ///
    /// # Returns
    ///
    /// Hydraulic radius in meters.
    pub fn hydraulic_radius(width: f64, depth: f64) -> f64 {
        let area = width * depth;
        let perimeter = width + 2.0 * depth;
        if perimeter > 0.0 {
            area / perimeter
        } else {
            0.0
        }
    }

    /// Compute mean flow velocity using Manning's equation.
    ///
    /// # Formula
    ///
    /// ```text
    /// V = (1/n) * R_h^(2/3) * S^(1/2)
    /// ```
    ///
    /// # Arguments
    ///
    /// * `hydraulic_radius` - Hydraulic radius (meters).
    /// * `slope` - Channel bed slope (dimensionless).
    /// * `manning_n` - Manning's roughness coefficient (s/m^(1/3)).
    ///
    /// # Returns
    ///
    /// Flow velocity in m/s.
    pub fn manning_velocity(hydraulic_radius: f64, slope: f64, manning_n: f64) -> f64 {
        if manning_n <= 0.0 || slope < 0.0 {
            return 0.0;
        }
        (1.0 / manning_n) * hydraulic_radius.powf(2.0 / 3.0) * slope.sqrt()
    }

    /// Linear interpolation helper.
    fn interpolate(xs: &[f64], ys: &[f64], x: f64) -> f64 {
        if xs.is_empty() || ys.is_empty() {
            return 0.0;
        }
        if xs.len() == 1 {
            return ys[0];
        }
        if x <= xs[0] {
            return ys[0];
        }
        if x >= xs[xs.len() - 1] {
            return ys[ys.len() - 1];
        }
        // Find bracketing interval
        for i in 1..xs.len() {
            if x <= xs[i] {
                let t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
                return ys[i - 1] + t * (ys[i] - ys[i - 1]);
            }
        }
        ys[ys.len() - 1]
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn default_config() -> BoreConfig {
        BoreConfig {
            channel_width_m: 200.0,
            channel_depth_m: 3.0,
            tidal_range_m: 10.0,
            river_flow_m3s: 50.0,
            channel_slope: 1e-4,
            bed_roughness_manning: 0.03,
            convergence_rate: 0.01,
        }
    }

    // -----------------------------------------------------------------------
    // Helper function tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_gravitational_acceleration() {
        assert!((gravitational_acceleration() - 9.80665).abs() < 1e-6);
    }

    #[test]
    fn test_shallow_water_speed_unit_depth() {
        // c = sqrt(9.80665 * 1.0) ≈ 3.131
        let predictor = TidalBorePredictor::new(default_config());
        let c = predictor.shallow_water_speed(1.0);
        assert!((c - 3.131).abs() < 0.01, "c = {c}");
    }

    #[test]
    fn test_shallow_water_speed_3m() {
        // c = sqrt(9.80665 * 3.0) ≈ 5.424
        let predictor = TidalBorePredictor::new(default_config());
        let c = predictor.shallow_water_speed(3.0);
        assert!((c - 5.424).abs() < 0.01, "c = {c}");
    }

    #[test]
    fn test_shallow_water_speed_zero_depth() {
        let predictor = TidalBorePredictor::new(default_config());
        let c = predictor.shallow_water_speed(0.0);
        assert_eq!(c, 0.0);
    }

    #[test]
    fn test_froude_number_critical() {
        // At critical flow: bore_speed + river_velocity = sqrt(g * h)
        // With zero river flow:
        let mut config = default_config();
        config.river_flow_m3s = 0.0;
        let predictor = TidalBorePredictor::new(config);
        let c = predictor.shallow_water_speed(3.0);
        let fr = predictor.froude_number(c, 3.0);
        assert!(
            (fr - 1.0).abs() < 1e-10,
            "Expected Fr=1 at critical, got {fr}"
        );
    }

    #[test]
    fn test_froude_number_supercritical() {
        let predictor = TidalBorePredictor::new(default_config());
        let fr = predictor.froude_number(8.0, 3.0);
        assert!(fr > 1.0, "Expected Fr > 1, got {fr}");
    }

    #[test]
    fn test_froude_includes_river_velocity() {
        // River flow adds to effective velocity
        let mut config = default_config();
        config.river_flow_m3s = 0.0;
        let pred_no_river = TidalBorePredictor::new(config.clone());
        let fr_no_river = pred_no_river.froude_number(5.0, 3.0);

        config.river_flow_m3s = 100.0;
        let pred_with_river = TidalBorePredictor::new(config);
        let fr_with_river = pred_with_river.froude_number(5.0, 3.0);

        assert!(
            fr_with_river > fr_no_river,
            "River flow should increase Froude: {fr_with_river} vs {fr_no_river}"
        );
    }

    #[test]
    fn test_conjugate_depth_belanger_fr2() {
        // Fr=2: h2/h1 = 0.5*(sqrt(1 + 8*4) - 1) = 0.5*(sqrt(33) - 1) ≈ 0.5*(5.745 - 1) ≈ 2.372
        let h1 = 1.0;
        let h2 = conjugate_depth(h1, 2.0);
        let ratio = h2 / h1;
        assert!(
            (ratio - 2.372).abs() < 0.01,
            "Expected h2/h1 ≈ 2.372, got {ratio}"
        );
    }

    #[test]
    fn test_conjugate_depth_fr1() {
        // Fr=1: h2/h1 = 0.5*(sqrt(9) - 1) = 0.5*(3-1) = 1.0
        let h1 = 2.0;
        let h2 = conjugate_depth(h1, 1.0);
        assert!(
            (h2 - h1).abs() < 1e-10,
            "At Fr=1, conjugate depth should equal h1"
        );
    }

    #[test]
    fn test_bore_height_wrapper() {
        let predictor = TidalBorePredictor::new(default_config());
        let h2 = predictor.bore_height(2.0, 1.0);
        assert!(
            (h2 - conjugate_depth(1.0, 2.0)).abs() < 1e-12,
            "bore_height should match conjugate_depth"
        );
    }

    #[test]
    fn test_bore_speed_positive() {
        let predictor = TidalBorePredictor::new(default_config());
        let speed = predictor.bore_speed(4.0, 2.0);
        assert!(speed > 0.0, "Bore speed should be positive");
    }

    #[test]
    fn test_bore_speed_increases_with_depth_ratio() {
        let predictor = TidalBorePredictor::new(default_config());
        let speed1 = predictor.bore_speed(3.0, 2.0);
        let speed2 = predictor.bore_speed(5.0, 2.0);
        assert!(
            speed2 > speed1,
            "Greater depth ratio should give higher bore speed"
        );
    }

    #[test]
    fn test_classify_bore_undular() {
        let predictor = TidalBorePredictor::new(default_config());
        assert_eq!(predictor.classify_bore(1.0), BoreType::Undular);
        assert_eq!(predictor.classify_bore(1.2), BoreType::Undular);
    }

    #[test]
    fn test_classify_bore_transitional() {
        let predictor = TidalBorePredictor::new(default_config());
        assert_eq!(predictor.classify_bore(1.4), BoreType::Transitional);
        assert_eq!(predictor.classify_bore(1.5), BoreType::Transitional);
    }

    #[test]
    fn test_classify_bore_breaking() {
        let predictor = TidalBorePredictor::new(default_config());
        assert_eq!(predictor.classify_bore(1.6), BoreType::Breaking);
        assert_eq!(predictor.classify_bore(2.5), BoreType::Breaking);
    }

    #[test]
    fn test_energy_dissipation_positive() {
        let predictor = TidalBorePredictor::new(default_config());
        let de = predictor.energy_dissipation(2.0, 4.0);
        assert!(de > 0.0, "Energy dissipation should be positive for h2 > h1");
    }

    #[test]
    fn test_energy_dissipation_zero_at_equal_depth() {
        let predictor = TidalBorePredictor::new(default_config());
        let de = predictor.energy_dissipation(3.0, 3.0);
        assert!(
            de.abs() < 1e-12,
            "Energy dissipation should be zero when h1 = h2"
        );
    }

    #[test]
    fn test_energy_dissipation_formula() {
        // dE = g * (h2-h1)^3 / (4*h1*h2)
        let g = gravitational_acceleration();
        let h1: f64 = 2.0;
        let h2: f64 = 5.0;
        let expected = g * (h2 - h1).powi(3) / (4.0 * h1 * h2);
        let predictor = TidalBorePredictor::new(default_config());
        let de = predictor.energy_dissipation(h1, h2);
        assert!(
            (de - expected).abs() < 1e-10,
            "Expected {expected}, got {de}"
        );
    }

    #[test]
    fn test_specific_energy() {
        let g = gravitational_acceleration();
        let depth = 2.0;
        let velocity = 3.0;
        let e = specific_energy(depth, velocity);
        let expected = depth + velocity * velocity / (2.0 * g);
        assert!((e - expected).abs() < 1e-10);
    }

    #[test]
    fn test_specific_energy_still_water() {
        let e = specific_energy(5.0, 0.0);
        assert!((e - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_critical_depth() {
        let g = gravitational_acceleration();
        let q = 2.0; // m^2/s
        let hc = critical_depth(q);
        let expected = (q * q / g).powf(1.0 / 3.0);
        assert!((hc - expected).abs() < 1e-10);
    }

    #[test]
    fn test_critical_depth_froude_unity() {
        // At critical depth, Fr = v / sqrt(g*h) should equal 1
        let q = 3.0;
        let hc = critical_depth(q);
        let vc = q / hc;
        let g = gravitational_acceleration();
        let fr = vc / (g * hc).sqrt();
        assert!(
            (fr - 1.0).abs() < 1e-8,
            "Fr at critical depth should be 1, got {fr}"
        );
    }

    #[test]
    fn test_predict_arrival_time() {
        let predictor = TidalBorePredictor::new(default_config());
        // Constant depth of 4m over 1000m
        let profile = vec![(0.0, 4.0), (500.0, 4.0), (1000.0, 4.0)];
        let time = predictor.predict_arrival_time(1000.0, &profile);
        let c = predictor.shallow_water_speed(4.0);
        let expected = 1000.0 / c;
        assert!(
            (time - expected).abs() < 0.1,
            "Expected ~{expected}s, got {time}s"
        );
    }

    #[test]
    fn test_predict_arrival_time_varying_depth() {
        let predictor = TidalBorePredictor::new(default_config());
        // Deeper water = faster bore
        let shallow = vec![(0.0, 1.0), (1000.0, 1.0)];
        let deep = vec![(0.0, 4.0), (1000.0, 4.0)];
        let t_shallow = predictor.predict_arrival_time(1000.0, &shallow);
        let t_deep = predictor.predict_arrival_time(1000.0, &deep);
        assert!(
            t_shallow > t_deep,
            "Shallow channel should have longer travel time"
        );
    }

    #[test]
    fn test_predict_arrival_time_single_point() {
        let predictor = TidalBorePredictor::new(default_config());
        let time = predictor.predict_arrival_time(1000.0, &[(0.0, 3.0)]);
        assert_eq!(time, 0.0, "Single point should return 0");
    }

    // -----------------------------------------------------------------------
    // TideGaugeProcessor tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_detect_bore_passage() {
        let processor = TideGaugeProcessor::new(1.0);
        // Slowly rising then sudden jump
        let mut levels = vec![1.0; 20];
        for i in 0..10 {
            levels[i] = 1.0 + 0.01 * i as f64;
        }
        levels[10] = 2.5; // bore passage: 1.5m jump
        let idx = processor.detect_bore_passage(&levels, 0.5);
        assert_eq!(idx, Some(10));
    }

    #[test]
    fn test_detect_bore_passage_none() {
        let processor = TideGaugeProcessor::new(1.0);
        let levels = vec![1.0, 1.01, 1.02, 1.03, 1.04];
        let idx = processor.detect_bore_passage(&levels, 0.5);
        assert_eq!(idx, None);
    }

    #[test]
    fn test_bore_height_from_gauge() {
        let processor = TideGaugeProcessor::new(1.0);
        let mut levels = vec![2.0; 30];
        // Bore at index 15: jump from 2.0 to 4.0
        for i in 15..30 {
            levels[i] = 4.0;
        }
        let height = processor.bore_height_from_gauge(&levels, 15);
        assert!(
            (height - 2.0).abs() < 0.01,
            "Expected ~2.0m bore, got {height}"
        );
    }

    #[test]
    fn test_tidal_harmonic_analysis_single_sinusoid() {
        let processor = TideGaugeProcessor::new(1.0);
        let dt_s = 60.0; // 1-minute sampling
        let freq_cph = 0.0805114; // M2 constituent
        let amplitude = 3.0;
        let phase = 0.5;
        let omega = 2.0 * PI * freq_cph;

        // Generate 24 hours of data
        let n = (24.0 * 3600.0 / dt_s) as usize;
        let dt_hours = dt_s / 3600.0;
        let water_level: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 * dt_hours;
                amplitude * (omega * t + phase).cos()
            })
            .collect();

        let results = processor.tidal_harmonic_analysis(&water_level, dt_s, &[freq_cph]);
        assert_eq!(results.len(), 1);
        let (f, a, _p) = results[0];
        assert!((f - freq_cph).abs() < 1e-10);
        // Amplitude should be close to 3.0
        assert!(
            (a - amplitude).abs() < 0.5,
            "Expected amplitude ~{amplitude}, got {a}"
        );
    }

    #[test]
    fn test_rate_of_rise() {
        let processor = TideGaugeProcessor::new(1.0);
        let levels = vec![1.0, 1.5, 2.5, 3.0, 3.2];
        let dt_s = 1.0;
        let ror = processor.rate_of_rise(&levels, dt_s);
        assert_eq!(ror.len(), 4);
        assert!((ror[0] - 0.5).abs() < 1e-10);
        assert!((ror[1] - 1.0).abs() < 1e-10);
        assert!((ror[2] - 0.5).abs() < 1e-10);
        assert!((ror[3] - 0.2).abs() < 1e-10);
    }

    #[test]
    fn test_rate_of_rise_empty() {
        let processor = TideGaugeProcessor::new(1.0);
        let ror = processor.rate_of_rise(&[1.0], 1.0);
        assert!(ror.is_empty());
    }

    #[test]
    fn test_filter_tidal_signal_smoothing() {
        let processor = TideGaugeProcessor::new(1.0);
        let dt_s = 60.0;
        // High-frequency noise on top of constant level
        let n = 100;
        let mut levels = vec![5.0; n];
        for i in 0..n {
            levels[i] += if i % 2 == 0 { 0.5 } else { -0.5 };
        }
        let filtered = processor.filter_tidal_signal(&levels, 0.1, dt_s);
        // Interior filtered values should be closer to 5.0 than raw
        let mid = n / 2;
        assert!(
            (filtered[mid] - 5.0).abs() < (levels[mid] - 5.0).abs(),
            "Filtered should be smoother"
        );
    }

    #[test]
    fn test_filter_tidal_signal_preserves_length() {
        let processor = TideGaugeProcessor::new(1.0);
        let levels = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let filtered = processor.filter_tidal_signal(&levels, 0.01, 60.0);
        assert_eq!(filtered.len(), levels.len());
    }

    // -----------------------------------------------------------------------
    // ShallowWaterSolver tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_dam_break_initial_condition() {
        let solver = ShallowWaterSolver::new(1.0, 100);
        let (h, u) = solver.initial_condition_dam_break(5.0, 1.0);
        assert_eq!(h.len(), 100);
        assert_eq!(u.len(), 100);
        assert!((h[0] - 5.0).abs() < 1e-10);
        assert!((h[49] - 5.0).abs() < 1e-10);
        assert!((h[50] - 1.0).abs() < 1e-10);
        assert!((h[99] - 1.0).abs() < 1e-10);
        assert!(u.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_dam_break_symmetric_initial() {
        let solver = ShallowWaterSolver::new(1.0, 100);
        let (h, _u) = solver.initial_condition_dam_break(3.0, 3.0);
        // Equal depths everywhere
        assert!(h.iter().all(|&v| (v - 3.0).abs() < 1e-10));
    }

    #[test]
    fn test_lax_friedrichs_conserves_mass_approximately() {
        let solver = ShallowWaterSolver::new(1.0, 100);
        let (mut h, mut u) = solver.initial_condition_dam_break(5.0, 1.0);
        let mass_before: f64 = h.iter().sum();

        let g = gravitational_acceleration();
        let max_speed = (g * 5.0).sqrt() + 1.0;
        let dt = solver.stable_timestep(max_speed, 1.0, 0.5);
        for _ in 0..10 {
            solver.step_lax_friedrichs(&mut h, &mut u, dt, 1.0, g);
        }
        let mass_after: f64 = h.iter().sum();

        // Mass should be approximately conserved (with boundary effects)
        let rel_error = ((mass_after - mass_before) / mass_before).abs();
        assert!(
            rel_error < 0.05,
            "Mass conservation error {rel_error} too large"
        );
    }

    #[test]
    fn test_run_simulation_produces_bore() {
        let solver = ShallowWaterSolver::new(1.0, 200);
        let (mut h, mut u) = solver.initial_condition_dam_break(5.0, 1.0);

        let g = gravitational_acceleration();
        let max_speed = (g * 5.0).sqrt() + 1.0;
        let dt = solver.stable_timestep(max_speed, 1.0, 0.5);

        solver.run_simulation(&mut h, &mut u, dt, 20);

        // After simulation, the discontinuity should have spread
        // Check that not all left cells are still 5.0
        let mid = 100;
        let any_changed = h[mid - 5..mid + 5]
            .iter()
            .any(|&v| (v - 5.0).abs() > 0.01 && (v - 1.0).abs() > 0.01);
        assert!(any_changed, "Simulation should evolve the dam break");
    }

    #[test]
    fn test_courant_number() {
        let solver = ShallowWaterSolver::new(1.0, 100);
        let c = solver.courant_number(10.0, 0.05, 1.0);
        assert!((c - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_courant_number_unity() {
        let solver = ShallowWaterSolver::new(1.0, 100);
        let c = solver.courant_number(5.0, 0.2, 1.0);
        assert!((c - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_stable_timestep() {
        let solver = ShallowWaterSolver::new(1.0, 100);
        let dt = solver.stable_timestep(10.0, 1.0, 0.5);
        assert!((dt - 0.05).abs() < 1e-10);
    }

    #[test]
    fn test_stable_timestep_zero_speed() {
        let solver = ShallowWaterSolver::new(1.0, 100);
        let dt = solver.stable_timestep(0.0, 1.0, 0.5);
        assert_eq!(dt, 0.0);
    }

    #[test]
    fn test_cfl_condition_satisfied() {
        let solver = ShallowWaterSolver::new(1.0, 100);
        let max_speed = 7.0;
        let dx = 1.0;
        let cfl = 0.8;
        let dt = solver.stable_timestep(max_speed, dx, cfl);
        let c = solver.courant_number(max_speed, dt, dx);
        assert!(
            c <= cfl + 1e-10,
            "CFL condition not satisfied: C={c} > {cfl}"
        );
    }

    // -----------------------------------------------------------------------
    // ChannelGeometry tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_channel_geometry_interpolation() {
        let geom = ChannelGeometry::new(
            vec![200.0, 150.0, 100.0],
            vec![5.0, 4.0, 3.0],
            vec![0.0, 500.0, 1000.0],
        );
        // Midpoint interpolation
        let w = geom.width_at(250.0);
        assert!((w - 175.0).abs() < 1e-10, "Expected 175, got {w}");
        let d = geom.depth_at(250.0);
        assert!((d - 4.5).abs() < 1e-10, "Expected 4.5, got {d}");
    }

    #[test]
    fn test_channel_geometry_boundary_clamp() {
        let geom = ChannelGeometry::new(
            vec![200.0, 100.0],
            vec![5.0, 3.0],
            vec![0.0, 1000.0],
        );
        assert!((geom.width_at(-100.0) - 200.0).abs() < 1e-10);
        assert!((geom.width_at(2000.0) - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_cross_section_area() {
        let geom = ChannelGeometry::new(
            vec![200.0, 100.0],
            vec![5.0, 3.0],
            vec![0.0, 1000.0],
        );
        let area = geom.cross_section_area(0.0);
        assert!((area - 1000.0).abs() < 1e-10, "Expected 200*5=1000");
    }

    #[test]
    fn test_hydraulic_radius_rectangular() {
        // For width=10, depth=2: R = 10*2 / (10+4) = 20/14 ≈ 1.4286
        let rh = ChannelGeometry::hydraulic_radius(10.0, 2.0);
        let expected = 20.0 / 14.0;
        assert!(
            (rh - expected).abs() < 1e-10,
            "Expected {expected}, got {rh}"
        );
    }

    #[test]
    fn test_hydraulic_radius_wide_channel() {
        // Very wide channel: R ≈ depth
        let rh = ChannelGeometry::hydraulic_radius(10000.0, 3.0);
        assert!(
            (rh - 3.0).abs() < 0.01,
            "Wide channel R should ≈ depth, got {rh}"
        );
    }

    #[test]
    fn test_manning_velocity() {
        // V = (1/0.03) * R^(2/3) * S^(1/2)
        let n = 0.03;
        let rh = 1.5;
        let slope = 1e-4;
        let v = ChannelGeometry::manning_velocity(rh, slope, n);
        let expected = (1.0 / n) * rh.powf(2.0 / 3.0) * slope.sqrt();
        assert!(
            (v - expected).abs() < 1e-10,
            "Expected {expected}, got {v}"
        );
        assert!(v > 0.0);
    }

    #[test]
    fn test_manning_velocity_zero_slope() {
        let v = ChannelGeometry::manning_velocity(1.5, 0.0, 0.03);
        assert!((v - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_manning_velocity_zero_n() {
        let v = ChannelGeometry::manning_velocity(1.5, 1e-4, 0.0);
        assert_eq!(v, 0.0);
    }

    #[test]
    fn test_num_stations() {
        let geom = ChannelGeometry::new(
            vec![200.0, 150.0, 100.0],
            vec![5.0, 4.0, 3.0],
            vec![0.0, 500.0, 1000.0],
        );
        assert_eq!(geom.num_stations(), 3);
    }

    // -----------------------------------------------------------------------
    // Integration / cross-validation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_bore_speed_consistent_with_froude() {
        // If bore_speed gives v, then froude_number(v, h1) should match
        // the Froude number implied by the depth ratio via Belanger.
        let mut config = default_config();
        config.river_flow_m3s = 0.0; // simplify by removing river flow
        let predictor = TidalBorePredictor::new(config);

        let h1 = 2.0;
        let h2 = 4.0;
        let v = predictor.bore_speed(h2, h1);
        let fr = predictor.froude_number(v, h1);

        // From Belanger: h2/h1 = 0.5*(sqrt(1+8*Fr^2) - 1)
        let h2_predicted = conjugate_depth(h1, fr);
        assert!(
            (h2_predicted - h2).abs() < 0.1,
            "Bore speed and Belanger should be consistent: predicted h2={h2_predicted}, actual h2={h2}"
        );
    }

    #[test]
    fn test_energy_dissipation_increases_with_height() {
        let predictor = TidalBorePredictor::new(default_config());
        let de1 = predictor.energy_dissipation(2.0, 3.0);
        let de2 = predictor.energy_dissipation(2.0, 5.0);
        assert!(
            de2 > de1,
            "Greater bore height should dissipate more energy"
        );
    }

    #[test]
    fn test_full_bore_prediction_workflow() {
        // End-to-end: configure, compute Froude, classify, get height
        let config = BoreConfig {
            channel_width_m: 150.0,
            channel_depth_m: 2.5,
            tidal_range_m: 8.0,
            river_flow_m3s: 30.0,
            channel_slope: 1e-4,
            bed_roughness_manning: 0.03,
            convergence_rate: 0.02,
        };
        let predictor = TidalBorePredictor::new(config);

        // Estimate bore speed from tidal range
        let h1 = 2.5;
        let h2 = h1 + 3.0; // 3m bore on top of 2.5m depth
        let v = predictor.bore_speed(h2, h1);
        let fr = predictor.froude_number(v, h1);
        let bore_type = predictor.classify_bore(fr);
        let de = predictor.energy_dissipation(h1, h2);

        assert!(v > 0.0);
        assert!(fr > 1.0);
        assert!(de > 0.0);
        // With h2/h1 = 5.5/2.5 = 2.2, this should be a significant bore
        match bore_type {
            BoreType::Undular | BoreType::Transitional | BoreType::Breaking => {}
        }
    }

    #[test]
    fn test_gauge_bore_detection_and_height() {
        let processor = TideGaugeProcessor::new(1.0);

        // Synthesize gauge data: flat at 2m, bore at sample 50, rises to 4m
        let mut levels = vec![2.0; 100];
        for i in 50..100 {
            levels[i] = 4.0;
        }

        let idx = processor.detect_bore_passage(&levels, 1.0);
        assert_eq!(idx, Some(50));

        let height = processor.bore_height_from_gauge(&levels, 50);
        assert!(
            (height - 2.0).abs() < 0.1,
            "Expected ~2.0m bore, got {height}"
        );
    }
}
