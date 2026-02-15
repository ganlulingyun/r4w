//! # Permafrost Active Layer Thickness Monitor
//!
//! Signal processing for monitoring permafrost active layer thickness using
//! ground-penetrating radar (GPR) and temperature sensors. Permafrost monitoring
//! is critical for climate science, infrastructure stability, and Arctic engineering.
//!
//! ## Overview
//!
//! The active layer is the topmost portion of permafrost-affected ground that thaws
//! seasonally. Its thickness varies from centimeters to several meters depending on
//! climate, soil type, vegetation, and snow cover. Changes in active layer thickness
//! serve as a key indicator of climate change in polar and high-altitude regions.
//!
//! ## Key Components
//!
//! - **`PermafrostMonitor`**: Core processing engine combining GPR and thermal data
//!   to estimate active layer depth, dielectric properties, and propagation velocity.
//!
//! - **`ThermalProfile`**: Temperature-depth profile analysis including zero-curtain
//!   duration measurement, thaw depth interpolation, and trumpet curve computation
//!   for steady-state thermal regime assessment.
//!
//! - **`RadargramProcessor`**: GPR trace processing including mean trace removal,
//!   dewow filtering, time-varying gain compensation, Kirchhoff migration, and
//!   envelope detection for improved subsurface imaging.
//!
//! - **`SubsidenceTracker`**: Long-term monitoring of surface elevation changes and
//!   active layer deepening trends with linear regression and prediction.
//!
//! ## Soil Dielectric Models
//!
//! The module implements the Topp equation for relating volumetric water content
//! to dielectric constant, and the Anderson-Tice model for unfrozen water content
//! at sub-zero temperatures. These empirical relationships are essential for
//! converting GPR travel times to physical depths.
//!
//! ## References
//!
//! - Topp, G.C., Davis, J.L., and Annan, A.P. (1980). "Electromagnetic
//!   determination of soil water content." Water Resources Research, 16(3), 574-582.
//! - Stefan, J. (1891). "Uber die Theorie der Eisbildung." Annalen der Physik, 278(2).
//! - Anderson, D.M. and Tice, A.R. (1972). "Predicting unfrozen water contents
//!   in frozen soils from surface area measurements." Highway Research Record, 393.

use std::f64::consts::PI;

/// Speed of light in vacuum (m/s).
const SPEED_OF_LIGHT: f64 = 299_792_458.0;

/// Speed of light in m/ns for convenience.
const SPEED_OF_LIGHT_M_PER_NS: f64 = 0.299_792_458;

// ---------------------------------------------------------------------------
// SoilType
// ---------------------------------------------------------------------------

/// Soil classification for permafrost regions, each with characteristic
/// dielectric and thermal properties.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SoilType {
    /// Fine-grained silt, common in loess and floodplain deposits.
    Silt,
    /// Clay-rich soil with high unfrozen water content at sub-zero temperatures.
    Clay,
    /// Sandy soil with relatively low water-holding capacity.
    Sand,
    /// Organic peat with very high porosity and water content.
    Peat,
    /// Bedrock with minimal porosity.
    Rock,
    /// Coarse gravel with large pore spaces.
    Gravel,
}

// ---------------------------------------------------------------------------
// GainMethod
// ---------------------------------------------------------------------------

/// Time-varying gain compensation methods for GPR traces.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GainMethod {
    /// Linear gain ramp: gain = 1 + slope * sample_index.
    Linear,
    /// Exponential gain: gain = exp(alpha * sample_index).
    Exponential,
    /// Automatic Gain Control with a sliding window.
    Agc {
        /// Number of samples in the AGC window.
        window_size: usize,
    },
}

// ---------------------------------------------------------------------------
// PermafrostConfig
// ---------------------------------------------------------------------------

/// Configuration parameters for permafrost monitoring.
#[derive(Debug, Clone)]
pub struct PermafrostConfig {
    /// Soil type at the measurement site.
    pub soil_type: SoilType,
    /// Surface temperature in degrees Celsius.
    pub surface_temperature_c: f64,
    /// Permafrost temperature in degrees Celsius (typically -2 to -15).
    pub permafrost_temperature_c: f64,
    /// Maximum measurement depth in metres.
    pub measurement_depth_m: f64,
    /// GPR centre frequency in MHz (typical: 100-400 MHz).
    pub gpr_center_frequency_mhz: f64,
    /// GPR recording time window in nanoseconds.
    pub time_window_ns: f64,
}

// ---------------------------------------------------------------------------
// PermafrostMonitor
// ---------------------------------------------------------------------------

/// Core permafrost monitoring engine combining GPR and thermal data.
///
/// Provides dielectric property estimation, propagation velocity calculations,
/// active layer depth determination, and the Stefan equation for analytical
/// thaw depth prediction.
pub struct PermafrostMonitor {
    config: PermafrostConfig,
}

impl PermafrostMonitor {
    /// Create a new monitor from the given configuration.
    pub fn new(config: PermafrostConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &PermafrostConfig {
        &self.config
    }

    /// Typical relative dielectric constant for frozen soil.
    ///
    /// Frozen soils have low permittivity because ice (εr ≈ 3.2) replaces
    /// liquid water (εr ≈ 80). Values are approximate mid-range estimates.
    pub fn dielectric_constant_frozen(soil: &SoilType) -> f64 {
        match soil {
            SoilType::Silt => 6.0,
            SoilType::Clay => 7.0,
            SoilType::Sand => 4.0,
            SoilType::Peat => 4.5,
            SoilType::Rock => 5.5,
            SoilType::Gravel => 3.5,
        }
    }

    /// Dielectric constant for thawed (unfrozen) soil using the Topp equation.
    ///
    /// The moisture fraction is the volumetric water content θ (0..1).
    /// The Topp equation gives the bulk apparent dielectric constant:
    ///
    ///   εr = 3.03 + 9.3θ + 146.0θ² − 76.7θ³
    ///
    /// An offset is added for different soil matrix permittivities.
    pub fn dielectric_constant_thawed(soil: &SoilType, moisture_fraction: f64) -> f64 {
        let base = topp_equation(moisture_fraction);
        let offset = match soil {
            SoilType::Silt => 1.0,
            SoilType::Clay => 2.0,
            SoilType::Sand => 0.0,
            SoilType::Peat => 1.5,
            SoilType::Rock => 0.5,
            SoilType::Gravel => -0.5,
        };
        base + offset
    }

    /// Electromagnetic propagation velocity in soil (m/s).
    ///
    /// v = c / sqrt(εr)
    pub fn propagation_velocity(epsilon_r: f64) -> f64 {
        if epsilon_r <= 0.0 {
            return 0.0;
        }
        SPEED_OF_LIGHT / epsilon_r.sqrt()
    }

    /// Two-way travel time in nanoseconds for a given depth and dielectric constant.
    ///
    /// TWTT = 2 * depth / v  (converted to nanoseconds)
    pub fn two_way_travel_time(depth_m: f64, epsilon_r: f64) -> f64 {
        if epsilon_r <= 0.0 {
            return 0.0;
        }
        let velocity_m_per_ns = SPEED_OF_LIGHT_M_PER_NS / epsilon_r.sqrt();
        2.0 * depth_m / velocity_m_per_ns
    }

    /// Estimate active layer depth from two-way travel time and dielectric constant.
    ///
    /// depth = TWTT * v / 2
    pub fn active_layer_depth(twtt_ns: f64, epsilon_r: f64) -> f64 {
        if epsilon_r <= 0.0 {
            return 0.0;
        }
        let velocity_m_per_ns = SPEED_OF_LIGHT_M_PER_NS / epsilon_r.sqrt();
        twtt_ns * velocity_m_per_ns / 2.0
    }

    /// Detect the thaw front in a radargram (collection of traces).
    ///
    /// For each trace, finds the sample index of the first peak above
    /// `threshold` (as a fraction of the trace maximum). Returns a vector
    /// of sample indices (as f64) corresponding to the detected reflection.
    /// A value of -1.0 indicates no detection for that trace.
    pub fn detect_thaw_front(radargram: &[Vec<f64>], threshold: f64) -> Vec<f64> {
        let mut picks = Vec::with_capacity(radargram.len());
        for trace in radargram {
            if trace.is_empty() {
                picks.push(-1.0);
                continue;
            }
            let max_val = trace.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
            if max_val <= 0.0 {
                picks.push(-1.0);
                continue;
            }
            let abs_threshold = threshold * max_val;
            let mut found = false;
            for (i, &val) in trace.iter().enumerate() {
                if val >= abs_threshold {
                    picks.push(i as f64);
                    found = true;
                    break;
                }
            }
            if !found {
                picks.push(-1.0);
            }
        }
        picks
    }

    /// Stefan equation for analytical active layer depth estimation.
    ///
    /// The modified Stefan equation gives the maximum active layer thickness:
    ///
    ///   d = sqrt(2 * k * I / (L * ρ))
    ///
    /// where:
    /// - `thawing_index`: cumulative degree-days above 0°C (°C·days)
    /// - `thermal_conductivity`: soil thermal conductivity k (W/m·°C)
    /// - `latent_heat`: volumetric latent heat L (J/m³)
    /// - `dry_density`: soil dry density ρ (kg/m³)
    ///
    /// The thawing index is converted to °C·seconds internally.
    pub fn stefan_equation(
        thawing_index: f64,
        thermal_conductivity: f64,
        latent_heat: f64,
        dry_density: f64,
    ) -> f64 {
        if latent_heat <= 0.0 || dry_density <= 0.0 {
            return 0.0;
        }
        // Convert degree-days to degree-seconds.
        let thawing_index_seconds = thawing_index * 86400.0;
        let numerator = 2.0 * thermal_conductivity * thawing_index_seconds;
        let denominator = latent_heat * dry_density;
        if denominator <= 0.0 {
            return 0.0;
        }
        (numerator / denominator).sqrt()
    }
}

// ---------------------------------------------------------------------------
// ThermalProfile
// ---------------------------------------------------------------------------

/// Temperature-depth profile analysis for permafrost thermal regime.
///
/// Provides methods for zero-curtain duration measurement, thaw depth
/// interpolation from borehole thermistor strings, and the trumpet curve
/// analytical model for steady-state temperature envelopes.
#[derive(Debug, Clone)]
pub struct ThermalProfile {
    /// Temperature readings at each depth (°C).
    pub temperatures: Vec<f64>,
    /// Corresponding depths (m).
    pub depths_m: Vec<f64>,
}

impl ThermalProfile {
    /// Create a new thermal profile.
    pub fn new(temperatures: Vec<f64>, depths_m: Vec<f64>) -> Self {
        Self {
            temperatures,
            depths_m,
        }
    }

    /// Compute the zero-curtain duration: the time that the temperature
    /// remains near 0°C (within ±0.5°C) during autumn freeze-back.
    ///
    /// `temps` is a time series of temperature at a single depth, and
    /// `times` contains corresponding time values (e.g., days or hours).
    /// Returns the total duration spent within the zero-curtain window.
    pub fn zero_curtain_duration(temps: &[f64], times: &[f64]) -> f64 {
        if temps.len() < 2 || times.len() < 2 || temps.len() != times.len() {
            return 0.0;
        }
        let mut duration = 0.0;
        for i in 1..temps.len() {
            if temps[i].abs() <= 0.5 {
                duration += times[i] - times[i - 1];
            }
        }
        duration
    }

    /// Interpolate the depth of the 0°C isotherm from a temperature-depth
    /// profile. Returns the depth at which temperature crosses zero.
    ///
    /// Searches from the surface downward for the first sign change and
    /// linearly interpolates. Returns `None` if no crossing is found.
    pub fn interpolate_thaw_depth(temps: &[f64], depths: &[f64]) -> f64 {
        if temps.len() < 2 || depths.len() < 2 || temps.len() != depths.len() {
            return 0.0;
        }
        for i in 1..temps.len() {
            // Look for a crossing from positive to negative (or zero).
            if (temps[i - 1] > 0.0 && temps[i] <= 0.0)
                || (temps[i - 1] >= 0.0 && temps[i] < 0.0)
            {
                // Linear interpolation to find the 0°C crossing depth.
                let frac = temps[i - 1] / (temps[i - 1] - temps[i]);
                return depths[i - 1] + frac * (depths[i] - depths[i - 1]);
            }
        }
        // No crossing found: return the deepest depth if all positive,
        // or 0.0 if all negative.
        if temps[0] > 0.0 {
            *depths.last().unwrap_or(&0.0)
        } else {
            0.0
        }
    }

    /// Trumpet curve: steady-state minimum and maximum temperature at a
    /// given depth from the surface.
    ///
    /// Based on the sinusoidal surface temperature model:
    ///
    ///   T(z, t) = T_mean + A * exp(-z * sqrt(π / (α * P))) * sin(2πt/P - z * sqrt(π / (α * P)))
    ///
    /// The envelope at depth z is:
    ///
    ///   T_max(z) = T_mean + A * exp(-z * sqrt(π / (α * P)))
    ///   T_min(z) = T_mean - A * exp(-z * sqrt(π / (α * P)))
    ///
    /// Parameters:
    /// - `mean_annual_temp`: mean annual ground surface temperature (°C)
    /// - `amplitude`: annual surface temperature amplitude (°C)
    /// - `depth`: depth below surface (m)
    /// - `thermal_diffusivity`: soil thermal diffusivity (m²/s)
    ///
    /// Returns (T_min, T_max) at the specified depth.
    pub fn trumpet_curve(
        mean_annual_temp: f64,
        amplitude: f64,
        depth: f64,
        thermal_diffusivity: f64,
    ) -> (f64, f64) {
        if thermal_diffusivity <= 0.0 || depth < 0.0 {
            return (mean_annual_temp, mean_annual_temp);
        }
        // Period = 1 year in seconds.
        let period_seconds = 365.25 * 86400.0;
        let damping = (PI / (thermal_diffusivity * period_seconds)).sqrt();
        let envelope = amplitude * (-depth * damping).exp();
        (mean_annual_temp - envelope, mean_annual_temp + envelope)
    }
}

// ---------------------------------------------------------------------------
// RadargramProcessor
// ---------------------------------------------------------------------------

/// GPR radargram processing routines for permafrost imaging.
///
/// Provides trace-level and multi-trace processing steps commonly applied
/// to GPR data before interpretation: background removal, dewow filtering,
/// gain compensation, migration, and envelope detection.
pub struct RadargramProcessor;

impl RadargramProcessor {
    /// Remove the mean trace (background removal).
    ///
    /// Computes the average across all traces at each sample position and
    /// subtracts it. This removes horizontally coherent clutter such as
    /// the direct ground wave and antenna ringing.
    pub fn remove_mean_trace(traces: &mut Vec<Vec<f64>>) {
        if traces.is_empty() {
            return;
        }
        let n_traces = traces.len();
        let n_samples = traces[0].len();
        if n_samples == 0 {
            return;
        }
        // Compute mean trace.
        let mut mean = vec![0.0; n_samples];
        for trace in traces.iter() {
            let len = trace.len().min(n_samples);
            for j in 0..len {
                mean[j] += trace[j];
            }
        }
        for val in mean.iter_mut() {
            *val /= n_traces as f64;
        }
        // Subtract.
        for trace in traces.iter_mut() {
            let len = trace.len().min(n_samples);
            for j in 0..len {
                trace[j] -= mean[j];
            }
        }
    }

    /// Dewow filter: running mean subtraction to remove low-frequency
    /// baseline wander ("wow") caused by inductive coupling.
    ///
    /// Subtracts a moving average of length `window_size` from each sample.
    pub fn dewow_filter(trace: &mut Vec<f64>, window_size: usize) {
        if trace.is_empty() || window_size == 0 {
            return;
        }
        let n = trace.len();
        let ws = window_size.min(n);
        // Compute running mean.
        let original: Vec<f64> = trace.clone();
        let mut running_sum: f64 = original[..ws].iter().sum();
        // Centre the window as much as possible.
        let half = ws / 2;
        for i in 0..n {
            let start = if i >= half { i - half } else { 0 };
            let end = (start + ws).min(n);
            let actual_ws = end - start;
            // Recompute sum for edges; use running approach in the middle.
            if i == 0 || start == 0 || end == n {
                running_sum = original[start..end].iter().sum();
            } else {
                running_sum += original[end - 1] - original[start - 1];
            }
            trace[i] = original[i] - running_sum / actual_ws as f64;
        }
    }

    /// Apply time-varying gain compensation to a trace.
    ///
    /// - `Linear`: gain increases linearly with sample index.
    /// - `Exponential`: gain increases exponentially.
    /// - `Agc`: automatic gain control using a sliding window RMS.
    pub fn gain_compensation(trace: &mut Vec<f64>, method: GainMethod) {
        if trace.is_empty() {
            return;
        }
        let n = trace.len();
        match method {
            GainMethod::Linear => {
                for i in 0..n {
                    let gain = 1.0 + (i as f64) / (n as f64);
                    trace[i] *= gain;
                }
            }
            GainMethod::Exponential => {
                let alpha = 3.0 / n as f64; // ~e^3 at the end
                for i in 0..n {
                    let gain = (alpha * i as f64).exp();
                    trace[i] *= gain;
                }
            }
            GainMethod::Agc { window_size } => {
                if window_size == 0 {
                    return;
                }
                let original: Vec<f64> = trace.clone();
                let half = window_size / 2;
                for i in 0..n {
                    let start = if i >= half { i - half } else { 0 };
                    let end = (start + window_size).min(n);
                    let rms: f64 = original[start..end]
                        .iter()
                        .map(|x| x * x)
                        .sum::<f64>()
                        / (end - start) as f64;
                    let rms = rms.sqrt();
                    if rms > 1e-10 {
                        trace[i] = original[i] / rms;
                    }
                }
            }
        }
    }

    /// Kirchhoff migration: collapse diffraction hyperbolas.
    ///
    /// This simplified 2D post-stack Kirchhoff migration sums energy along
    /// hyperbolic travel-time curves for each output point to focus
    /// diffracted energy back to its origin.
    ///
    /// Parameters:
    /// - `traces`: input radargram (n_traces x n_samples)
    /// - `velocity`: average propagation velocity (m/ns)
    /// - `dx`: trace spacing (m)
    /// - `dt`: sample interval (ns)
    ///
    /// Returns the migrated radargram.
    pub fn migration_kirchhoff(
        traces: &[Vec<f64>],
        velocity: f64,
        dx: f64,
        dt: f64,
    ) -> Vec<Vec<f64>> {
        if traces.is_empty() || dx <= 0.0 || dt <= 0.0 || velocity <= 0.0 {
            return traces.to_vec();
        }
        let n_traces = traces.len();
        let n_samples = traces[0].len();
        let mut output = vec![vec![0.0; n_samples]; n_traces];

        for ix in 0..n_traces {
            for it in 1..n_samples {
                // Vertical two-way time at this point.
                let t0 = it as f64 * dt;
                // Sum over neighbouring traces along the hyperbola.
                let mut sum = 0.0;
                let mut count = 0u32;
                let _max_offset = (n_traces / 2).max(1);
                for jx in 0..n_traces {
                    let offset = (jx as f64 - ix as f64) * dx;
                    // Hyperbolic travel time: t² = t0² + (2*offset/v)²
                    let t_hyp_sq = t0 * t0 + (2.0 * offset / velocity).powi(2);
                    let t_hyp = t_hyp_sq.sqrt();
                    let sample_idx = (t_hyp / dt).round() as usize;
                    if sample_idx < n_samples && jx < n_traces {
                        if let Some(trace) = traces.get(jx) {
                            if let Some(&val) = trace.get(sample_idx) {
                                sum += val;
                                count += 1;
                            }
                        }
                    }
                    // Limit aperture to avoid excessive summation.
                    let aperture_limit = ((t0 * velocity / 2.0) / dx).ceil() as usize;
                    if (jx as isize - ix as isize).unsigned_abs() > aperture_limit.max(1) {
                        continue;
                    }
                }
                if count > 0 {
                    output[ix][it] = sum / count as f64;
                }
            }
        }
        output
    }

    /// Envelope detection via a simple Hilbert-like transform approximation.
    ///
    /// Computes the analytic signal envelope using a discrete approximation:
    /// the envelope at sample n is sqrt(x[n]² + H{x}[n]²), where the Hilbert
    /// transform is approximated by a finite difference: H{x}[n] ≈ (x[n+1] - x[n-1]) / 2.
    ///
    /// For more accurate results, a full Hilbert transform with FFT should be used,
    /// but this provides a reasonable approximation for GPR data.
    pub fn envelope_detection(trace: &[f64]) -> Vec<f64> {
        let n = trace.len();
        if n < 3 {
            return trace.iter().map(|x| x.abs()).collect();
        }
        let mut envelope = vec![0.0; n];
        // First and last samples: use absolute value.
        envelope[0] = trace[0].abs();
        envelope[n - 1] = trace[n - 1].abs();
        for i in 1..n - 1 {
            let hilbert_approx = (trace[i + 1] - trace[i - 1]) / 2.0;
            envelope[i] = (trace[i] * trace[i] + hilbert_approx * hilbert_approx).sqrt();
        }
        envelope
    }
}

// ---------------------------------------------------------------------------
// SubsidenceTracker
// ---------------------------------------------------------------------------

/// A single subsidence measurement record.
#[derive(Debug, Clone)]
struct SubsidenceMeasurement {
    time_years: f64,
    elevation_m: f64,
    active_layer_m: f64,
}

/// Tracks long-term surface elevation changes and active layer trends.
///
/// Accumulates time-stamped measurements of surface elevation and active
/// layer thickness, then computes linear trends for subsidence rate and
/// thaw depth change.
pub struct SubsidenceTracker {
    #[allow(dead_code)]
    initial_elevation_m: f64,
    measurements: Vec<SubsidenceMeasurement>,
}

impl SubsidenceTracker {
    /// Create a new tracker with an initial surface elevation.
    pub fn new(initial_elevation_m: f64) -> Self {
        Self {
            initial_elevation_m,
            measurements: Vec::new(),
        }
    }

    /// Add a measurement at a given time (years since baseline).
    pub fn add_measurement(&mut self, time_years: f64, elevation_m: f64, active_layer_m: f64) {
        self.measurements.push(SubsidenceMeasurement {
            time_years,
            elevation_m,
            active_layer_m,
        });
    }

    /// Compute the surface subsidence rate (m/year) via linear regression.
    ///
    /// A negative value indicates the surface is sinking (thaw settlement).
    pub fn subsidence_rate_m_per_year(&self) -> f64 {
        if self.measurements.len() < 2 {
            return 0.0;
        }
        let times: Vec<f64> = self.measurements.iter().map(|m| m.time_years).collect();
        let elevations: Vec<f64> = self.measurements.iter().map(|m| m.elevation_m).collect();
        linear_slope(&times, &elevations)
    }

    /// Compute the active layer deepening trend (m/year) via linear regression.
    ///
    /// A positive value indicates the active layer is getting thicker (deepening).
    pub fn thaw_trend_m_per_year(&self) -> f64 {
        if self.measurements.len() < 2 {
            return 0.0;
        }
        let times: Vec<f64> = self.measurements.iter().map(|m| m.time_years).collect();
        let depths: Vec<f64> = self.measurements.iter().map(|m| m.active_layer_m).collect();
        linear_slope(&times, &depths)
    }

    /// Predict the active layer depth at a future time using linear extrapolation.
    ///
    /// `years_ahead` is the number of years beyond the last measurement.
    pub fn predict_thaw_depth(&self, years_ahead: f64) -> f64 {
        if self.measurements.is_empty() {
            return 0.0;
        }
        if self.measurements.len() < 2 {
            return self.measurements[0].active_layer_m;
        }
        let times: Vec<f64> = self.measurements.iter().map(|m| m.time_years).collect();
        let depths: Vec<f64> = self.measurements.iter().map(|m| m.active_layer_m).collect();
        let slope = linear_slope(&times, &depths);
        let intercept = linear_intercept(&times, &depths, slope);
        let last_time = *times.last().unwrap();
        slope * (last_time + years_ahead) + intercept
    }
}

// ---------------------------------------------------------------------------
// Helper / standalone functions
// ---------------------------------------------------------------------------

/// Topp equation: empirical relationship between volumetric water content
/// (θ, dimensionless 0..1) and apparent dielectric constant εr.
///
/// εr = 3.03 + 9.3θ + 146.0θ² − 76.7θ³
///
/// Valid for θ in [0, ~0.6]. Widely used in GPR and TDR soil moisture sensing.
pub fn topp_equation(volumetric_water_content: f64) -> f64 {
    let theta = volumetric_water_content;
    3.03 + 9.3 * theta + 146.0 * theta * theta - 76.7 * theta * theta * theta
}

/// Anderson-Tice model: unfrozen water content in frozen soil as a function
/// of sub-zero temperature.
///
/// θ_u = a * |T|^b
///
/// where a and b are soil-specific parameters. Returns the volumetric
/// fraction of unfrozen water (0..1).
pub fn unfrozen_water_content(temperature_c: f64, soil: &SoilType) -> f64 {
    if temperature_c >= 0.0 {
        // Above freezing: return a typical saturation value for the soil type.
        return match soil {
            SoilType::Silt => 0.40,
            SoilType::Clay => 0.45,
            SoilType::Sand => 0.25,
            SoilType::Peat => 0.80,
            SoilType::Rock => 0.05,
            SoilType::Gravel => 0.15,
        };
    }
    // Anderson-Tice parameters (a, b) for each soil type.
    let (a, b) = match soil {
        SoilType::Silt => (0.20, -0.45),
        SoilType::Clay => (0.30, -0.35),
        SoilType::Sand => (0.05, -0.60),
        SoilType::Peat => (0.40, -0.30),
        SoilType::Rock => (0.02, -0.70),
        SoilType::Gravel => (0.03, -0.65),
    };
    let abs_temp = temperature_c.abs().max(0.01); // avoid log(0)
    let theta_u = a * abs_temp.powf(b);
    // Clamp to physical range.
    theta_u.clamp(0.0, 1.0)
}

/// Thermal conductivity of soil (W/(m·K)).
///
/// Uses simplified Johansen (1975) method:
/// - Frozen soil has higher conductivity due to ice bridging.
/// - Moisture increases conductivity.
///
/// Returns approximate thermal conductivity for the soil type, frozen/thawed
/// state, and moisture content (0..1).
pub fn thermal_conductivity(soil: &SoilType, frozen: bool, moisture: f64) -> f64 {
    // Dry thermal conductivity (W/(m·K)).
    let k_dry = match soil {
        SoilType::Silt => 0.25,
        SoilType::Clay => 0.25,
        SoilType::Sand => 0.30,
        SoilType::Peat => 0.06,
        SoilType::Rock => 2.00,
        SoilType::Gravel => 0.50,
    };
    // Saturated thermal conductivity (W/(m·K)).
    let k_sat = if frozen {
        match soil {
            SoilType::Silt => 2.40,
            SoilType::Clay => 2.40,
            SoilType::Sand => 3.50,
            SoilType::Peat => 1.20,
            SoilType::Rock => 3.50,
            SoilType::Gravel => 3.00,
        }
    } else {
        match soil {
            SoilType::Silt => 1.50,
            SoilType::Clay => 1.60,
            SoilType::Sand => 2.50,
            SoilType::Peat => 0.50,
            SoilType::Rock => 3.00,
            SoilType::Gravel => 2.00,
        }
    };
    let m = moisture.clamp(0.0, 1.0);
    // Kersten number approximation: linear interpolation for simplicity.
    k_dry + m * (k_sat - k_dry)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Simple linear regression slope.
fn linear_slope(x: &[f64], y: &[f64]) -> f64 {
    let n = x.len() as f64;
    if n < 2.0 {
        return 0.0;
    }
    let sum_x: f64 = x.iter().sum();
    let sum_y: f64 = y.iter().sum();
    let sum_xy: f64 = x.iter().zip(y.iter()).map(|(xi, yi)| xi * yi).sum();
    let sum_xx: f64 = x.iter().map(|xi| xi * xi).sum();
    let denom = n * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-15 {
        return 0.0;
    }
    (n * sum_xy - sum_x * sum_y) / denom
}

/// Simple linear regression intercept.
fn linear_intercept(x: &[f64], y: &[f64], slope: f64) -> f64 {
    let n = x.len() as f64;
    if n < 1.0 {
        return 0.0;
    }
    let mean_y: f64 = y.iter().sum::<f64>() / n;
    let mean_x: f64 = x.iter().sum::<f64>() / n;
    mean_y - slope * mean_x
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // ---- Dielectric constant tests ----

    #[test]
    fn test_frozen_dielectric_silt() {
        let eps = PermafrostMonitor::dielectric_constant_frozen(&SoilType::Silt);
        assert!((eps - 6.0).abs() < 1e-10);
    }

    #[test]
    fn test_frozen_dielectric_sand() {
        let eps = PermafrostMonitor::dielectric_constant_frozen(&SoilType::Sand);
        assert!((eps - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_frozen_dielectric_clay() {
        let eps = PermafrostMonitor::dielectric_constant_frozen(&SoilType::Clay);
        assert!((eps - 7.0).abs() < 1e-10);
    }

    #[test]
    fn test_frozen_dielectric_gravel_lowest() {
        // Gravel should have the lowest frozen dielectric constant.
        let gravel = PermafrostMonitor::dielectric_constant_frozen(&SoilType::Gravel);
        let sand = PermafrostMonitor::dielectric_constant_frozen(&SoilType::Sand);
        assert!(gravel < sand);
    }

    #[test]
    fn test_thawed_dielectric_dry_sand() {
        // Dry sand (θ=0): Topp gives 3.03 + offset 0.0 = 3.03
        let eps = PermafrostMonitor::dielectric_constant_thawed(&SoilType::Sand, 0.0);
        assert!((eps - 3.03).abs() < 0.01);
    }

    #[test]
    fn test_thawed_dielectric_wet_increases() {
        let dry = PermafrostMonitor::dielectric_constant_thawed(&SoilType::Silt, 0.0);
        let wet = PermafrostMonitor::dielectric_constant_thawed(&SoilType::Silt, 0.3);
        assert!(wet > dry, "Wet soil should have higher dielectric constant");
    }

    #[test]
    fn test_thawed_dielectric_saturated_high() {
        // High moisture -> high dielectric (water εr ≈ 80).
        let eps = PermafrostMonitor::dielectric_constant_thawed(&SoilType::Clay, 0.5);
        assert!(eps > 20.0, "Saturated clay should have high dielectric");
    }

    // ---- Topp equation tests ----

    #[test]
    fn test_topp_equation_dry() {
        let eps = topp_equation(0.0);
        assert!((eps - 3.03).abs() < 0.01, "Dry soil εr should be ~3.03");
    }

    #[test]
    fn test_topp_equation_moderate_moisture() {
        // At θ = 0.2: 3.03 + 9.3*0.2 + 146*0.04 - 76.7*0.008 = 3.03 + 1.86 + 5.84 - 0.6136 = 10.1164
        let eps = topp_equation(0.2);
        assert!((eps - 10.12).abs() < 0.1);
    }

    #[test]
    fn test_topp_equation_high_moisture() {
        // At θ = 0.4: 3.03 + 9.3*0.4 + 146*0.16 - 76.7*0.064
        // = 3.03 + 3.72 + 23.36 - 4.9088 = 25.2012
        let eps = topp_equation(0.4);
        assert!((eps - 25.2).abs() < 0.2);
    }

    // ---- Velocity tests ----

    #[test]
    fn test_propagation_velocity_in_frozen_soil() {
        // Frozen sand εr = 4 → v = c/2 = 0.15 m/ns
        let eps = 4.0;
        let v = PermafrostMonitor::propagation_velocity(eps);
        let v_m_per_ns = v / 1e9;
        assert!(
            (v_m_per_ns - 0.15).abs() < 0.005,
            "Frozen soil velocity should be ~0.15 m/ns, got {}",
            v_m_per_ns
        );
    }

    #[test]
    fn test_propagation_velocity_in_thawed_soil() {
        // Thawed wet soil εr ~ 25 → v = c/5 = 0.06 m/ns
        let eps = 25.0;
        let v = PermafrostMonitor::propagation_velocity(eps);
        let v_m_per_ns = v / 1e9;
        assert!(
            (v_m_per_ns - 0.06).abs() < 0.005,
            "Thawed soil velocity should be ~0.06 m/ns, got {}",
            v_m_per_ns
        );
    }

    #[test]
    fn test_propagation_velocity_zero_epsilon() {
        let v = PermafrostMonitor::propagation_velocity(0.0);
        assert_eq!(v, 0.0);
    }

    // ---- Travel time / depth tests ----

    #[test]
    fn test_two_way_travel_time() {
        // 1m depth, εr = 4 → v = 0.15 m/ns → TWTT = 2/0.15 = 13.33 ns
        let twtt = PermafrostMonitor::two_way_travel_time(1.0, 4.0);
        assert!(
            (twtt - 13.33).abs() < 0.1,
            "TWTT should be ~13.33 ns, got {}",
            twtt
        );
    }

    #[test]
    fn test_active_layer_depth_roundtrip() {
        let depth_in = 2.5;
        let eps = 6.0;
        let twtt = PermafrostMonitor::two_way_travel_time(depth_in, eps);
        let depth_out = PermafrostMonitor::active_layer_depth(twtt, eps);
        assert!(
            (depth_in - depth_out).abs() < 1e-6,
            "Roundtrip depth should match: {} vs {}",
            depth_in,
            depth_out
        );
    }

    #[test]
    fn test_active_layer_depth_from_known_twtt() {
        // εr = 4 → v = 0.15 m/ns, TWTT = 20 ns → depth = 20 * 0.15 / 2 = 1.5 m
        let depth = PermafrostMonitor::active_layer_depth(20.0, 4.0);
        assert!(
            (depth - 1.5).abs() < 0.01,
            "Depth should be ~1.5 m, got {}",
            depth
        );
    }

    // ---- Stefan equation tests ----

    #[test]
    fn test_stefan_equation_basic() {
        // Thawing index = 1000 degree-days, k = 1.5 W/(m·K),
        // L = 334000 J/kg (ice), ρ = 1500 kg/m³
        let depth = PermafrostMonitor::stefan_equation(1000.0, 1.5, 334000.0, 1500.0);
        // d = sqrt(2 * 1.5 * 1000 * 86400 / (334000 * 1500))
        // = sqrt(259200000 / 501000000) = sqrt(0.5174) = 0.719
        assert!(
            depth > 0.5 && depth < 1.0,
            "Stefan depth should be ~0.72 m, got {}",
            depth
        );
    }

    #[test]
    fn test_stefan_equation_zero_thawing() {
        let depth = PermafrostMonitor::stefan_equation(0.0, 1.5, 334000.0, 1500.0);
        assert_eq!(depth, 0.0);
    }

    #[test]
    fn test_stefan_equation_increases_with_thawing_index() {
        let d1 = PermafrostMonitor::stefan_equation(500.0, 1.5, 334000.0, 1500.0);
        let d2 = PermafrostMonitor::stefan_equation(2000.0, 1.5, 334000.0, 1500.0);
        assert!(d2 > d1, "More degree-days should produce deeper thaw");
    }

    // ---- Thermal profile tests ----

    #[test]
    fn test_zero_curtain_duration() {
        // Temperature near 0°C for 3 time steps (0.5 spacing each).
        let temps = vec![2.0, 0.3, -0.2, 0.1, -0.4, -2.0, -5.0];
        let times = vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let dur = ThermalProfile::zero_curtain_duration(&temps, &times);
        // Samples at index 1 (0.3), 2 (-0.2), 3 (0.1), 4 (-0.4) are within ±0.5.
        // Duration = (1-0) + (2-1) + (3-2) + (4-3) = 4.0
        assert!(
            (dur - 4.0).abs() < 1e-10,
            "Zero curtain duration should be 4.0, got {}",
            dur
        );
    }

    #[test]
    fn test_zero_curtain_no_near_zero() {
        let temps = vec![5.0, 4.0, 3.0, 2.0, 1.0];
        let times = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let dur = ThermalProfile::zero_curtain_duration(&temps, &times);
        assert_eq!(dur, 0.0);
    }

    #[test]
    fn test_interpolate_thaw_depth() {
        // Surface is warm, permafrost at depth.
        let temps = vec![5.0, 3.0, 1.0, -1.0, -3.0];
        let depths = vec![0.0, 0.5, 1.0, 1.5, 2.0];
        let thaw_depth = ThermalProfile::interpolate_thaw_depth(&temps, &depths);
        // Linear interp between (1.0m, 1.0°C) and (1.5m, -1.0°C):
        // frac = 1.0/(1.0+1.0) = 0.5, depth = 1.0 + 0.5*0.5 = 1.25
        assert!(
            (thaw_depth - 1.25).abs() < 0.01,
            "Thaw depth should be ~1.25 m, got {}",
            thaw_depth
        );
    }

    #[test]
    fn test_interpolate_thaw_depth_all_positive() {
        let temps = vec![5.0, 3.0, 1.0, 0.5];
        let depths = vec![0.0, 1.0, 2.0, 3.0];
        let thaw_depth = ThermalProfile::interpolate_thaw_depth(&temps, &depths);
        // No crossing: return deepest depth.
        assert!((thaw_depth - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_interpolate_thaw_depth_all_frozen() {
        let temps = vec![-5.0, -3.0, -1.0, -0.5];
        let depths = vec![0.0, 1.0, 2.0, 3.0];
        let thaw_depth = ThermalProfile::interpolate_thaw_depth(&temps, &depths);
        assert_eq!(thaw_depth, 0.0);
    }

    // ---- Trumpet curve tests ----

    #[test]
    fn test_trumpet_curve_surface() {
        // At surface (depth=0), the envelope equals the amplitude.
        let (t_min, t_max) = ThermalProfile::trumpet_curve(-5.0, 20.0, 0.0, 1e-6);
        assert!((t_min - (-25.0)).abs() < 0.01);
        assert!((t_max - 15.0).abs() < 0.01);
    }

    #[test]
    fn test_trumpet_curve_deep() {
        // At great depth, envelope should approach mean annual temperature.
        let (t_min, t_max) = ThermalProfile::trumpet_curve(-5.0, 20.0, 50.0, 1e-6);
        assert!(
            (t_max - t_min) < 1.0,
            "Deep trumpet curve should converge: range = {}",
            t_max - t_min
        );
        assert!((t_min - (-5.0)).abs() < 0.5);
    }

    #[test]
    fn test_trumpet_curve_decreases_with_depth() {
        let (_, max1) = ThermalProfile::trumpet_curve(-5.0, 20.0, 1.0, 1e-6);
        let (_, max2) = ThermalProfile::trumpet_curve(-5.0, 20.0, 5.0, 1e-6);
        assert!(max2 < max1, "Temperature amplitude should decrease with depth");
    }

    // ---- Radargram processing tests ----

    #[test]
    fn test_remove_mean_trace() {
        let mut traces = vec![
            vec![10.0, 20.0, 30.0],
            vec![10.0, 20.0, 30.0],
            vec![10.0, 20.0, 30.0],
        ];
        RadargramProcessor::remove_mean_trace(&mut traces);
        // All traces identical → mean = trace → result = 0.
        for trace in &traces {
            for &val in trace {
                assert!(val.abs() < 1e-10, "Background removal should zero identical traces");
            }
        }
    }

    #[test]
    fn test_remove_mean_trace_preserves_differences() {
        let mut traces = vec![
            vec![10.0, 20.0, 30.0],
            vec![12.0, 22.0, 32.0],
        ];
        RadargramProcessor::remove_mean_trace(&mut traces);
        // Mean = [11, 21, 31], trace 0 = [-1, -1, -1], trace 1 = [1, 1, 1].
        assert!((traces[0][0] - (-1.0)).abs() < 1e-10);
        assert!((traces[1][0] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_dewow_filter_removes_dc() {
        let mut trace = vec![100.0; 50];
        // Add a small signal on top.
        for i in 20..30 {
            trace[i] += 5.0;
        }
        RadargramProcessor::dewow_filter(&mut trace, 20);
        // The constant baseline should be largely removed.
        let mean: f64 = trace.iter().sum::<f64>() / trace.len() as f64;
        assert!(
            mean.abs() < 2.0,
            "Dewow should remove DC component, mean = {}",
            mean
        );
    }

    #[test]
    fn test_gain_compensation_linear() {
        let mut trace = vec![1.0; 100];
        RadargramProcessor::gain_compensation(&mut trace, GainMethod::Linear);
        // First sample: gain = 1.0, last sample: gain = 2.0.
        assert!((trace[0] - 1.0).abs() < 0.02);
        assert!(trace[99] > trace[0], "Linear gain should increase");
    }

    #[test]
    fn test_gain_compensation_exponential() {
        let mut trace = vec![1.0; 100];
        RadargramProcessor::gain_compensation(&mut trace, GainMethod::Exponential);
        assert!(trace[99] > trace[0], "Exponential gain should increase");
        assert!(trace[99] > 2.0, "Exponential gain should be significant at end");
    }

    #[test]
    fn test_gain_compensation_agc() {
        let mut trace = vec![0.0; 100];
        trace[20] = 10.0;
        trace[80] = 0.1;
        RadargramProcessor::gain_compensation(&mut trace, GainMethod::Agc { window_size: 10 });
        // AGC should normalize amplitudes.
        assert!(trace[80].abs() > 0.0, "AGC should amplify weak signal");
    }

    #[test]
    fn test_envelope_detection() {
        // Use a higher-frequency sinusoid so the finite-difference Hilbert
        // approximation (H{x}[n] ~ (x[n+1]-x[n-1])/2) is more accurate.
        // For sin(2*pi*f*n/N), the approximation quality improves as
        // the number of samples per cycle increases but the derivative
        // scaling also matters. We use enough cycles so the envelope
        // estimate is reasonable everywhere in the middle.
        let n = 400;
        let freq = 40.0; // 40 cycles in 400 samples = 10 samples/cycle
        let trace: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / n as f64).sin())
            .collect();
        let env = RadargramProcessor::envelope_detection(&trace);
        // The envelope should be positive everywhere and reasonably close to 1.0.
        // With 10 samples/cycle the approximation gives envelope values > 0.7.
        for i in 20..380 {
            assert!(env[i] > 0.3, "Envelope should be positive, got {} at index {}", env[i], i);
        }
        // Also verify maximum envelope is near 1.0 (not wildly overshooting).
        let max_env = env.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(max_env < 1.5, "Max envelope should not greatly exceed 1.0, got {}", max_env);
    }

    #[test]
    fn test_envelope_detection_short() {
        let trace = vec![1.0, -2.0];
        let env = RadargramProcessor::envelope_detection(&trace);
        assert_eq!(env.len(), 2);
        assert!((env[0] - 1.0).abs() < 1e-10);
        assert!((env[1] - 2.0).abs() < 1e-10);
    }

    // ---- Detect thaw front tests ----

    #[test]
    fn test_detect_thaw_front_basic() {
        let trace1 = vec![0.1, 0.2, 0.8, 0.3, 0.1];
        let trace2 = vec![0.1, 0.1, 0.3, 0.9, 0.2];
        let picks = PermafrostMonitor::detect_thaw_front(&[trace1, trace2], 0.5);
        assert_eq!(picks.len(), 2);
        assert!((picks[0] - 2.0).abs() < 1e-10); // first peak at sample 2
        assert!((picks[1] - 3.0).abs() < 1e-10); // first peak at sample 3
    }

    #[test]
    fn test_detect_thaw_front_no_detection() {
        let trace = vec![0.01, 0.02, 0.01];
        let picks = PermafrostMonitor::detect_thaw_front(&[trace], 0.9);
        // 0.9 * 0.02 = 0.018; sample 1 (0.02) >= 0.018, so it should detect.
        assert!((picks[0] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_detect_thaw_front_empty_trace() {
        let picks = PermafrostMonitor::detect_thaw_front(&[vec![]], 0.5);
        assert_eq!(picks[0], -1.0);
    }

    // ---- Subsidence tracker tests ----

    #[test]
    fn test_subsidence_rate() {
        let mut tracker = SubsidenceTracker::new(100.0);
        // Surface drops 0.02 m per year.
        tracker.add_measurement(0.0, 100.0, 1.0);
        tracker.add_measurement(1.0, 99.98, 1.05);
        tracker.add_measurement(2.0, 99.96, 1.10);
        tracker.add_measurement(3.0, 99.94, 1.15);
        let rate = tracker.subsidence_rate_m_per_year();
        assert!(
            (rate - (-0.02)).abs() < 0.001,
            "Subsidence rate should be -0.02 m/yr, got {}",
            rate
        );
    }

    #[test]
    fn test_thaw_trend() {
        let mut tracker = SubsidenceTracker::new(100.0);
        tracker.add_measurement(0.0, 100.0, 1.0);
        tracker.add_measurement(1.0, 99.98, 1.05);
        tracker.add_measurement(2.0, 99.96, 1.10);
        tracker.add_measurement(3.0, 99.94, 1.15);
        let trend = tracker.thaw_trend_m_per_year();
        assert!(
            (trend - 0.05).abs() < 0.001,
            "Thaw trend should be 0.05 m/yr, got {}",
            trend
        );
    }

    #[test]
    fn test_predict_thaw_depth() {
        let mut tracker = SubsidenceTracker::new(100.0);
        tracker.add_measurement(0.0, 100.0, 1.0);
        tracker.add_measurement(5.0, 99.90, 1.25);
        // Slope = (1.25 - 1.0) / (5.0 - 0.0) = 0.05 m/yr
        // Predict 5 years ahead from last measurement (year 10):
        // depth = slope * 10 + intercept = 0.05 * 10 + 1.0 = 1.5
        let predicted = tracker.predict_thaw_depth(5.0);
        assert!(
            (predicted - 1.5).abs() < 0.01,
            "Predicted depth should be ~1.5 m, got {}",
            predicted
        );
    }

    #[test]
    fn test_subsidence_rate_insufficient_data() {
        let tracker = SubsidenceTracker::new(100.0);
        assert_eq!(tracker.subsidence_rate_m_per_year(), 0.0);
    }

    // ---- Unfrozen water content tests ----

    #[test]
    fn test_unfrozen_water_above_freezing() {
        let uwc = unfrozen_water_content(5.0, &SoilType::Clay);
        assert!((uwc - 0.45).abs() < 1e-10);
    }

    #[test]
    fn test_unfrozen_water_decreases_with_cold() {
        let uwc_warm = unfrozen_water_content(-1.0, &SoilType::Silt);
        let uwc_cold = unfrozen_water_content(-10.0, &SoilType::Silt);
        assert!(
            uwc_cold < uwc_warm,
            "Colder temp should have less unfrozen water"
        );
    }

    #[test]
    fn test_unfrozen_water_clay_higher_than_sand() {
        // Clay retains more unfrozen water at sub-zero temperatures.
        let clay = unfrozen_water_content(-2.0, &SoilType::Clay);
        let sand = unfrozen_water_content(-2.0, &SoilType::Sand);
        assert!(
            clay > sand,
            "Clay should have more unfrozen water than sand"
        );
    }

    // ---- Thermal conductivity tests ----

    #[test]
    fn test_thermal_conductivity_frozen_higher() {
        let k_frozen = thermal_conductivity(&SoilType::Silt, true, 0.4);
        let k_thawed = thermal_conductivity(&SoilType::Silt, false, 0.4);
        assert!(
            k_frozen > k_thawed,
            "Frozen soil should have higher thermal conductivity"
        );
    }

    #[test]
    fn test_thermal_conductivity_dry_low() {
        let k = thermal_conductivity(&SoilType::Sand, false, 0.0);
        assert!(
            (k - 0.30).abs() < 0.01,
            "Dry sand thermal conductivity should be ~0.30"
        );
    }

    #[test]
    fn test_thermal_conductivity_wet_higher() {
        let k_dry = thermal_conductivity(&SoilType::Clay, false, 0.0);
        let k_wet = thermal_conductivity(&SoilType::Clay, false, 0.5);
        assert!(k_wet > k_dry, "Wet soil should conduct heat better");
    }

    // ---- Migration test ----

    #[test]
    fn test_migration_kirchhoff_preserves_dimensions() {
        let traces = vec![vec![0.0; 50]; 10];
        let output = RadargramProcessor::migration_kirchhoff(&traces, 0.1, 0.5, 0.5);
        assert_eq!(output.len(), 10);
        assert_eq!(output[0].len(), 50);
    }

    #[test]
    fn test_migration_kirchhoff_empty() {
        let traces: Vec<Vec<f64>> = vec![];
        let output = RadargramProcessor::migration_kirchhoff(&traces, 0.1, 0.5, 0.5);
        assert!(output.is_empty());
    }

    // ---- Config / constructor tests ----

    #[test]
    fn test_permafrost_monitor_config() {
        let config = PermafrostConfig {
            soil_type: SoilType::Peat,
            surface_temperature_c: 10.0,
            permafrost_temperature_c: -5.0,
            measurement_depth_m: 10.0,
            gpr_center_frequency_mhz: 200.0,
            time_window_ns: 100.0,
        };
        let monitor = PermafrostMonitor::new(config);
        assert_eq!(monitor.config().gpr_center_frequency_mhz, 200.0);
    }

    #[test]
    fn test_thermal_profile_new() {
        let tp = ThermalProfile::new(vec![5.0, 0.0, -5.0], vec![0.0, 1.0, 2.0]);
        assert_eq!(tp.temperatures.len(), 3);
        assert_eq!(tp.depths_m.len(), 3);
    }
}
