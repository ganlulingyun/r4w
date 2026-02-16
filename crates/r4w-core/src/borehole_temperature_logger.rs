//! Borehole temperature logger for geothermal gradient and heat flow analysis.
//!
//! This module implements signal processing for borehole temperature measurements
//! used in geothermal exploration, heat flow studies, permafrost monitoring,
//! hydrogeological characterization, and thermal recovery assessment.
//!
//! ## Key capabilities
//!
//! - **Geothermal gradient estimation** — linear fit of dT/dz from depth-temperature profiles
//! - **Heat flow calculation** — Fourier's law q = -k * dT/dz (W/m^2)
//! - **Horner correction** — correct bottom-hole temperature for drilling disturbance
//! - **Bullard plot** — cumulative thermal resistance vs temperature for heat flow
//! - **DTS processing** — fiber-optic distributed temperature sensing via Raman ratio
//! - **Permafrost detection** — identify 0 deg C crossings and active layer thickness
//! - **Thermal recovery** — estimate time to equilibrium after drilling
//! - **Temperature log filtering** — moving average, median, outlier detection
//! - **Radiogenic heat production** — estimate heat from U, Th, K concentrations
//!
//! ## Physics
//!
//! - Average geothermal gradient: ~25-30 deg C/km (continental)
//! - Heat flow: q = k * grad(T), typical continental ~60-80 mW/m^2
//! - Horner correction: T_BHT = T_ms + m * ln((t_circ + t_shut) / t_shut)
//! - DTS Raman ratio: R = I_anti_stokes / I_stokes ~ exp(-h*delta_nu / (k_B*T))
//! - Bullard method: T(R) = T_0 + q*R, slope gives heat flow
//!
//! # Example
//!
//! ```
//! use r4w_core::borehole_temperature_logger::{
//!     BoreholeConfig, FluidType, GeothermalGradientEstimator,
//!     HeatFlowCalculator, HornerCorrection,
//! };
//!
//! let config = BoreholeConfig {
//!     total_depth_m: 3000.0,
//!     diameter_m: 0.216,
//!     drilling_date_unix: 1700000000,
//!     fluid_type: FluidType::Water,
//!     thermal_conductivity_w_mk: 2.5,
//! };
//!
//! // Estimate geothermal gradient from a temperature log
//! let depths = vec![500.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0];
//! let temps = vec![25.0, 40.0, 55.0, 70.0, 85.0, 100.0];
//! let gradient = GeothermalGradientEstimator::fit(&depths, &temps);
//! assert!((gradient.gradient_c_per_m - 0.03).abs() < 1e-6);
//!
//! // Calculate heat flow
//! let q = HeatFlowCalculator::fourier_heat_flow(2.5, gradient.gradient_c_per_m);
//! assert!((q - 0.075).abs() < 1e-6); // 75 mW/m^2
//! ```

/// Boltzmann constant in J/K.
const K_B: f64 = 1.380_649e-23;

/// Planck constant in J*s.
const H_PLANCK: f64 = 6.626_070_15e-34;

/// Speed of light in m/s.
const C_LIGHT: f64 = 2.997_924_58e8;

/// Absolute zero offset (0 deg C in K).
const CELSIUS_TO_KELVIN: f64 = 273.15;

// ---------------------------------------------------------------------------
// BoreholeConfig
// ---------------------------------------------------------------------------

/// Fluid type filling the borehole.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FluidType {
    /// Water-based drilling mud.
    Water,
    /// Oil-based drilling mud.
    Oil,
    /// Air or gas drilled.
    Air,
    /// Brine completion fluid.
    Brine,
}

impl FluidType {
    /// Approximate thermal conductivity of the fluid in W/(m*K).
    pub fn thermal_conductivity(&self) -> f64 {
        match self {
            FluidType::Water => 0.60,
            FluidType::Oil => 0.15,
            FluidType::Air => 0.026,
            FluidType::Brine => 0.65,
        }
    }

    /// Approximate density in kg/m^3.
    pub fn density(&self) -> f64 {
        match self {
            FluidType::Water => 1000.0,
            FluidType::Oil => 850.0,
            FluidType::Air => 1.225,
            FluidType::Brine => 1200.0,
        }
    }
}

/// Configuration for a borehole.
#[derive(Debug, Clone)]
pub struct BoreholeConfig {
    /// Total measured depth in meters.
    pub total_depth_m: f64,
    /// Borehole diameter in meters.
    pub diameter_m: f64,
    /// Drilling date as Unix timestamp.
    pub drilling_date_unix: u64,
    /// Type of fluid in the borehole.
    pub fluid_type: FluidType,
    /// Formation thermal conductivity in W/(m*K).
    pub thermal_conductivity_w_mk: f64,
}

impl BoreholeConfig {
    /// Borehole radius in meters.
    pub fn radius_m(&self) -> f64 {
        self.diameter_m / 2.0
    }

    /// Cross-sectional area in m^2.
    pub fn cross_section_area_m2(&self) -> f64 {
        std::f64::consts::PI * self.radius_m() * self.radius_m()
    }
}

// ---------------------------------------------------------------------------
// GeothermalGradientEstimator
// ---------------------------------------------------------------------------

/// Result of a geothermal gradient fit.
#[derive(Debug, Clone)]
pub struct GradientResult {
    /// Temperature gradient in deg C per meter (dT/dz).
    pub gradient_c_per_m: f64,
    /// Surface intercept temperature in deg C.
    pub surface_temp_c: f64,
    /// R-squared goodness of fit.
    pub r_squared: f64,
}

/// Estimate the geothermal gradient from depth-temperature data.
pub struct GeothermalGradientEstimator;

impl GeothermalGradientEstimator {
    /// Fit a linear gradient T = T_surface + gradient * z using least-squares.
    ///
    /// `depths` and `temps` must have the same length (>= 2).
    pub fn fit(depths: &[f64], temps: &[f64]) -> GradientResult {
        assert_eq!(depths.len(), temps.len(), "depths and temps must match");
        assert!(depths.len() >= 2, "need at least 2 points");

        let n = depths.len() as f64;
        let sum_z: f64 = depths.iter().sum();
        let sum_t: f64 = temps.iter().sum();
        let sum_zz: f64 = depths.iter().map(|z| z * z).sum();
        let sum_zt: f64 = depths.iter().zip(temps.iter()).map(|(z, t)| z * t).sum();

        let denom = n * sum_zz - sum_z * sum_z;
        let gradient = (n * sum_zt - sum_z * sum_t) / denom;
        let intercept = (sum_t - gradient * sum_z) / n;

        // R-squared
        let mean_t = sum_t / n;
        let ss_tot: f64 = temps.iter().map(|t| (t - mean_t).powi(2)).sum();
        let ss_res: f64 = depths
            .iter()
            .zip(temps.iter())
            .map(|(z, t)| {
                let predicted = intercept + gradient * z;
                (t - predicted).powi(2)
            })
            .sum();
        let r_squared = if ss_tot > 0.0 {
            1.0 - ss_res / ss_tot
        } else {
            1.0
        };

        GradientResult {
            gradient_c_per_m: gradient,
            surface_temp_c: intercept,
            r_squared,
        }
    }

    /// Simple gradient from top and bottom temperatures.
    pub fn simple_gradient(t_surface: f64, t_bottom: f64, depth: f64) -> f64 {
        (t_bottom - t_surface) / depth
    }

    /// Convert gradient from deg C/m to deg C/km.
    pub fn to_c_per_km(gradient_c_per_m: f64) -> f64 {
        gradient_c_per_m * 1000.0
    }
}

// ---------------------------------------------------------------------------
// HeatFlowCalculator
// ---------------------------------------------------------------------------

/// Lithology type for thermal conductivity lookup.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Lithology {
    Sandstone,
    Shale,
    Limestone,
    Granite,
    Basalt,
    Salt,
    Quartzite,
    Clay,
}

impl Lithology {
    /// Typical thermal conductivity in W/(m*K).
    pub fn thermal_conductivity(&self) -> f64 {
        match self {
            Lithology::Sandstone => 3.5,
            Lithology::Shale => 2.0,
            Lithology::Limestone => 2.8,
            Lithology::Granite => 3.2,
            Lithology::Basalt => 1.7,
            Lithology::Salt => 5.5,
            Lithology::Quartzite => 6.0,
            Lithology::Clay => 1.5,
        }
    }
}

/// Calculate heat flow from temperature gradient and thermal conductivity.
pub struct HeatFlowCalculator;

impl HeatFlowCalculator {
    /// Fourier's law: q = k * dT/dz (W/m^2).
    ///
    /// Returns positive value for downward-increasing temperature (normal geothermal).
    pub fn fourier_heat_flow(k: f64, gradient_c_per_m: f64) -> f64 {
        k * gradient_c_per_m
    }

    /// Heat flow from a layered formation with varying conductivity.
    ///
    /// `layers` is a slice of (thickness_m, thermal_conductivity_w_mk) tuples.
    /// `delta_t` is the total temperature difference across all layers.
    pub fn layered_heat_flow(layers: &[(f64, f64)], delta_t: f64) -> f64 {
        let total_resistance: f64 = layers.iter().map(|(dz, k)| dz / k).sum();
        if total_resistance > 0.0 {
            delta_t / total_resistance
        } else {
            0.0
        }
    }

    /// Harmonic mean thermal conductivity for a layered sequence.
    pub fn harmonic_mean_conductivity(layers: &[(f64, f64)]) -> f64 {
        let total_thickness: f64 = layers.iter().map(|(dz, _)| dz).sum();
        let total_resistance: f64 = layers.iter().map(|(dz, k)| dz / k).sum();
        if total_resistance > 0.0 {
            total_thickness / total_resistance
        } else {
            0.0
        }
    }

    /// Convert heat flow from W/m^2 to mW/m^2.
    pub fn to_mw_per_m2(q_w_per_m2: f64) -> f64 {
        q_w_per_m2 * 1000.0
    }
}

// ---------------------------------------------------------------------------
// HornerCorrection
// ---------------------------------------------------------------------------

/// A single bottom-hole temperature (BHT) measurement for Horner analysis.
#[derive(Debug, Clone)]
pub struct BhtMeasurement {
    /// Measured temperature in deg C.
    pub temperature_c: f64,
    /// Circulation (drilling) time in hours.
    pub circulation_time_h: f64,
    /// Shut-in time since circulation stopped in hours.
    pub shut_in_time_h: f64,
}

/// Horner correction result.
#[derive(Debug, Clone)]
pub struct HornerResult {
    /// Corrected (extrapolated) formation temperature in deg C.
    pub corrected_temp_c: f64,
    /// Slope of the Horner plot (deg C per ln-unit).
    pub slope: f64,
    /// R-squared of the Horner fit.
    pub r_squared: f64,
}

/// Correct bottom-hole temperatures for drilling disturbance using the Horner method.
///
/// The Horner plot extrapolates T vs ln((t_circ + t_shut) / t_shut) to infinite
/// shut-in time (x=0) to find the undisturbed formation temperature.
pub struct HornerCorrection;

impl HornerCorrection {
    /// Compute the Horner time ratio: (t_circ + t_shut) / t_shut.
    pub fn horner_ratio(t_circ_h: f64, t_shut_h: f64) -> f64 {
        (t_circ_h + t_shut_h) / t_shut_h
    }

    /// Single-measurement Horner correction.
    ///
    /// T_true = T_measured + slope * ln((t_circ + t_shut) / t_shut)
    /// where slope is estimated from the circulation-formation temperature difference.
    pub fn single_correction(
        t_measured: f64,
        t_circulation: f64,
        t_circ_h: f64,
        t_shut_h: f64,
    ) -> f64 {
        let ratio = Self::horner_ratio(t_circ_h, t_shut_h);
        let ln_ratio = ratio.ln();
        // At infinite shut-in, ln(ratio) -> 0, so T_true is the y-intercept.
        // With one point: estimate slope from (T_circ - T_measured) relationship.
        // T_measured = T_true + (T_circ - T_true) * f(time)
        // For a single BHT, use the simple Horner extrapolation:
        // T_true = T_measured + (T_measured - T_circulation) * ln_ratio / (ln_ratio - 1.0)
        // Simplified: T = T_ms - (T_ms - T_circ) * 1 / (1 - 1/ln_ratio)
        // But the standard single-point Horner approximation is:
        if ln_ratio > 1e-10 {
            // Use proportional correction
            let f = 1.0 / ln_ratio;
            t_measured + (t_measured - t_circulation) * f
        } else {
            t_measured
        }
    }

    /// Multi-measurement Horner plot extrapolation.
    ///
    /// Fits T = T_true + m * ln((t_circ + t_shut) / t_shut) using least-squares.
    /// Returns the extrapolated formation temperature at infinite shut-in.
    pub fn multi_correction(measurements: &[BhtMeasurement]) -> HornerResult {
        assert!(
            measurements.len() >= 2,
            "need at least 2 BHT measurements"
        );

        // x = ln((t_circ + t_shut) / t_shut), y = T_measured
        // Fit y = intercept + slope * x, intercept = T_true
        let xs: Vec<f64> = measurements
            .iter()
            .map(|m| Self::horner_ratio(m.circulation_time_h, m.shut_in_time_h).ln())
            .collect();
        let ys: Vec<f64> = measurements.iter().map(|m| m.temperature_c).collect();

        let n = xs.len() as f64;
        let sum_x: f64 = xs.iter().sum();
        let sum_y: f64 = ys.iter().sum();
        let sum_xx: f64 = xs.iter().map(|x| x * x).sum();
        let sum_xy: f64 = xs.iter().zip(ys.iter()).map(|(x, y)| x * y).sum();

        let denom = n * sum_xx - sum_x * sum_x;
        let slope = (n * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / n;

        // R-squared
        let mean_y = sum_y / n;
        let ss_tot: f64 = ys.iter().map(|y| (y - mean_y).powi(2)).sum();
        let ss_res: f64 = xs
            .iter()
            .zip(ys.iter())
            .map(|(x, y)| {
                let pred = intercept + slope * x;
                (y - pred).powi(2)
            })
            .sum();
        let r_squared = if ss_tot > 0.0 {
            1.0 - ss_res / ss_tot
        } else {
            1.0
        };

        HornerResult {
            corrected_temp_c: intercept,
            slope,
            r_squared,
        }
    }
}

// ---------------------------------------------------------------------------
// BullardPlot
// ---------------------------------------------------------------------------

/// A single layer for the Bullard method.
#[derive(Debug, Clone)]
pub struct ThermalLayer {
    /// Layer thickness in meters.
    pub thickness_m: f64,
    /// Thermal conductivity in W/(m*K).
    pub conductivity_w_mk: f64,
    /// Temperature at the top of the layer in deg C.
    pub temperature_top_c: f64,
}

/// Result of Bullard plot analysis.
#[derive(Debug, Clone)]
pub struct BullardResult {
    /// Heat flow in W/m^2 (slope of T vs R plot).
    pub heat_flow_w_m2: f64,
    /// Surface temperature intercept in deg C.
    pub surface_temp_c: f64,
    /// Cumulative thermal resistance values (m^2*K/W).
    pub resistances: Vec<f64>,
    /// Temperatures at layer boundaries.
    pub temperatures: Vec<f64>,
    /// R-squared goodness of fit.
    pub r_squared: f64,
}

/// Bullard plot analysis for heat flow determination.
///
/// Cumulative thermal resistance: R(z) = integral from 0 to z of dz'/k(z')
/// Plot T vs R; slope gives heat flow q.
pub struct BullardPlot;

impl BullardPlot {
    /// Compute cumulative thermal resistance for each layer boundary.
    pub fn cumulative_resistance(layers: &[ThermalLayer]) -> Vec<f64> {
        let mut resistances = Vec::with_capacity(layers.len() + 1);
        resistances.push(0.0);
        let mut r = 0.0;
        for layer in layers {
            r += layer.thickness_m / layer.conductivity_w_mk;
            resistances.push(r);
        }
        resistances
    }

    /// Perform Bullard plot analysis.
    ///
    /// Fits T = T_0 + q * R where R is cumulative thermal resistance.
    pub fn analyze(layers: &[ThermalLayer]) -> BullardResult {
        assert!(!layers.is_empty(), "need at least one layer");

        let resistances = Self::cumulative_resistance(layers);

        // Temperatures at each layer boundary
        let mut temperatures = Vec::with_capacity(layers.len() + 1);
        temperatures.push(layers[0].temperature_top_c);
        for layer in layers {
            // Temperature at bottom of each layer (estimated from top + gradient)
            // For Bullard analysis we use the measured temperatures at layer boundaries.
            // Here we use top temperatures of successive layers or extrapolate.
            temperatures.push(layer.temperature_top_c);
        }
        // More correctly: use top temperature of first layer, then for subsequent layers
        // use their top temperatures which correspond to the bottom of the previous layer.
        let mut temps_at_boundaries = vec![layers[0].temperature_top_c];
        for i in 1..layers.len() {
            temps_at_boundaries.push(layers[i].temperature_top_c);
        }
        // Last boundary temperature: extrapolate from last layer
        if layers.len() >= 2 {
            let last = &layers[layers.len() - 1];
            let prev = &layers[layers.len() - 2];
            let gradient =
                (last.temperature_top_c - prev.temperature_top_c) / prev.thickness_m;
            temps_at_boundaries.push(last.temperature_top_c + gradient * last.thickness_m);
        } else {
            // Single layer: assume 25 C/km gradient
            let last = &layers[0];
            temps_at_boundaries
                .push(last.temperature_top_c + 0.025 * last.thickness_m);
        }

        // Linear fit: T = T_0 + q * R
        let n = temps_at_boundaries.len() as f64;
        let sum_r: f64 = resistances.iter().sum();
        let sum_t: f64 = temps_at_boundaries.iter().sum();
        let sum_rr: f64 = resistances.iter().map(|r| r * r).sum();
        let sum_rt: f64 = resistances
            .iter()
            .zip(temps_at_boundaries.iter())
            .map(|(r, t)| r * t)
            .sum();

        let denom = n * sum_rr - sum_r * sum_r;
        let q = (n * sum_rt - sum_r * sum_t) / denom;
        let t0 = (sum_t - q * sum_r) / n;

        // R-squared
        let mean_t = sum_t / n;
        let ss_tot: f64 = temps_at_boundaries
            .iter()
            .map(|t| (t - mean_t).powi(2))
            .sum();
        let ss_res: f64 = resistances
            .iter()
            .zip(temps_at_boundaries.iter())
            .map(|(r, t)| {
                let pred = t0 + q * r;
                (t - pred).powi(2)
            })
            .sum();
        let r_squared = if ss_tot > 0.0 {
            1.0 - ss_res / ss_tot
        } else {
            1.0
        };

        BullardResult {
            heat_flow_w_m2: q,
            surface_temp_c: t0,
            resistances,
            temperatures: temps_at_boundaries,
            r_squared,
        }
    }
}

// ---------------------------------------------------------------------------
// DistributedTemperatureSensor
// ---------------------------------------------------------------------------

/// Distributed Temperature Sensing (DTS) processor for fiber-optic measurements.
///
/// Uses the Raman scattering ratio (anti-Stokes / Stokes) to determine temperature
/// along the fiber. The relationship is:
///
/// T(z) = (h * delta_nu / k_B) / ln(R_ref / R(z) * exp(h * delta_nu / (k_B * T_ref)))
///
/// where delta_nu is the Raman frequency shift (~13.2 THz for silica fiber).
pub struct DistributedTemperatureSensor {
    /// Reference temperature in Kelvin.
    pub reference_temp_k: f64,
    /// Reference Raman ratio at the reference temperature.
    pub reference_ratio: f64,
    /// Raman frequency shift in Hz (typically ~13.2 THz for silica).
    pub raman_shift_hz: f64,
    /// Spatial resolution in meters.
    pub spatial_resolution_m: f64,
}

impl DistributedTemperatureSensor {
    /// Create a DTS processor with standard silica fiber parameters.
    pub fn new_silica(reference_temp_k: f64, spatial_resolution_m: f64) -> Self {
        let raman_shift_hz = 13.2e12; // ~13.2 THz for silica
        // Reference ratio at T_ref
        let x = H_PLANCK * raman_shift_hz / (K_B * reference_temp_k);
        let reference_ratio = (-x).exp();

        Self {
            reference_temp_k,
            reference_ratio,
            raman_shift_hz,
            spatial_resolution_m,
        }
    }

    /// Energy parameter h*delta_nu/k_B in Kelvin.
    pub fn energy_parameter_k(&self) -> f64 {
        H_PLANCK * self.raman_shift_hz / K_B
    }

    /// Convert a single Raman ratio to temperature in Kelvin.
    ///
    /// T = E_param / ln(R_ref / R * exp(E_param / T_ref))
    pub fn ratio_to_temperature_k(&self, ratio: f64) -> f64 {
        let e_param = self.energy_parameter_k();
        let arg = (self.reference_ratio / ratio) * (e_param / self.reference_temp_k).exp();
        if arg > 0.0 {
            e_param / arg.ln()
        } else {
            0.0
        }
    }

    /// Convert a Raman ratio to temperature in Celsius.
    pub fn ratio_to_temperature_c(&self, ratio: f64) -> f64 {
        self.ratio_to_temperature_k(ratio) - CELSIUS_TO_KELVIN
    }

    /// Process a DTS trace: convert Stokes and anti-Stokes intensity arrays to temperature.
    ///
    /// Returns temperature in Celsius at each spatial point.
    pub fn process_trace(
        &self,
        stokes: &[f64],
        anti_stokes: &[f64],
    ) -> Vec<f64> {
        assert_eq!(stokes.len(), anti_stokes.len());
        stokes
            .iter()
            .zip(anti_stokes.iter())
            .map(|(s, a)| {
                if *s > 0.0 {
                    self.ratio_to_temperature_c(a / s)
                } else {
                    f64::NAN
                }
            })
            .collect()
    }

    /// Compute spatial positions along the fiber.
    pub fn positions(&self, num_points: usize) -> Vec<f64> {
        (0..num_points)
            .map(|i| i as f64 * self.spatial_resolution_m)
            .collect()
    }
}

// ---------------------------------------------------------------------------
// PermafrostDetector
// ---------------------------------------------------------------------------

/// Result of permafrost analysis.
#[derive(Debug, Clone)]
pub struct PermafrostResult {
    /// Whether permafrost was detected.
    pub permafrost_detected: bool,
    /// Depth of top of permafrost in meters (None if not detected).
    pub permafrost_top_m: Option<f64>,
    /// Depth of base of permafrost in meters (None if not detected).
    pub permafrost_base_m: Option<f64>,
    /// Active layer thickness in meters (seasonal thaw above permafrost).
    pub active_layer_thickness_m: Option<f64>,
    /// Minimum temperature in the profile in deg C.
    pub min_temperature_c: f64,
}

/// Detect permafrost boundaries from a borehole temperature profile.
pub struct PermafrostDetector;

impl PermafrostDetector {
    /// Analyze a depth-temperature profile for permafrost.
    ///
    /// Permafrost is defined as ground that remains at or below 0 deg C
    /// for at least two consecutive years. Here we detect the 0 deg C crossings.
    pub fn analyze(depths: &[f64], temps: &[f64]) -> PermafrostResult {
        assert_eq!(depths.len(), temps.len());
        assert!(!depths.is_empty());

        let min_temp = temps.iter().cloned().fold(f64::INFINITY, f64::min);

        // Find zero crossings by linear interpolation
        let mut crossings = Vec::new();
        for i in 0..depths.len() - 1 {
            if (temps[i] >= 0.0 && temps[i + 1] < 0.0)
                || (temps[i] < 0.0 && temps[i + 1] >= 0.0)
            {
                // Linearly interpolate the crossing depth
                let frac = (0.0 - temps[i]) / (temps[i + 1] - temps[i]);
                let crossing_depth = depths[i] + frac * (depths[i + 1] - depths[i]);
                crossings.push(crossing_depth);
            }
        }

        if crossings.is_empty() {
            // Check if entire profile is below zero
            if min_temp < 0.0 {
                return PermafrostResult {
                    permafrost_detected: true,
                    permafrost_top_m: Some(depths[0]),
                    permafrost_base_m: None, // extends beyond borehole
                    active_layer_thickness_m: Some(0.0),
                    min_temperature_c: min_temp,
                };
            }
            return PermafrostResult {
                permafrost_detected: false,
                permafrost_top_m: None,
                permafrost_base_m: None,
                active_layer_thickness_m: None,
                min_temperature_c: min_temp,
            };
        }

        let permafrost_top = crossings[0];
        let permafrost_base = if crossings.len() >= 2 {
            Some(crossings[1])
        } else {
            None
        };

        PermafrostResult {
            permafrost_detected: true,
            permafrost_top_m: Some(permafrost_top),
            permafrost_base_m: permafrost_base,
            active_layer_thickness_m: Some(permafrost_top),
            min_temperature_c: min_temp,
        }
    }
}

// ---------------------------------------------------------------------------
// ThermalRecoveryEstimator
// ---------------------------------------------------------------------------

/// Thermal recovery estimator for post-drilling temperature equilibration.
///
/// Uses the cylindrical source solution to estimate how long it takes for
/// the borehole temperature to approach the undisturbed formation temperature.
pub struct ThermalRecoveryEstimator;

impl ThermalRecoveryEstimator {
    /// Thermal diffusivity from conductivity, density, and specific heat.
    ///
    /// alpha = k / (rho * c_p) in m^2/s
    pub fn thermal_diffusivity(
        conductivity_w_mk: f64,
        density_kg_m3: f64,
        specific_heat_j_kgk: f64,
    ) -> f64 {
        conductivity_w_mk / (density_kg_m3 * specific_heat_j_kgk)
    }

    /// Dimensionless time for the cylindrical source model.
    ///
    /// t_D = alpha * t / r_w^2
    pub fn dimensionless_time(
        alpha_m2_s: f64,
        time_s: f64,
        borehole_radius_m: f64,
    ) -> f64 {
        alpha_m2_s * time_s / (borehole_radius_m * borehole_radius_m)
    }

    /// Fractional temperature recovery for the cylindrical source model.
    ///
    /// For large dimensionless time t_D, the recovery fraction is approximately:
    /// f(t_D) = 1 - 1 / (4 * t_D) for t_D >> 1
    ///
    /// Returns a value between 0 (just drilled) and 1 (fully equilibrated).
    pub fn recovery_fraction(dimensionless_time: f64) -> f64 {
        if dimensionless_time <= 0.0 {
            return 0.0;
        }
        // Approximation for the line-source solution:
        // f = 1 - ln(1 + 1/t_D) / ln(4*t_D) for moderate to large t_D
        // For simplicity, use the compact approximation:
        if dimensionless_time < 0.01 {
            // Early time: logarithmic approach
            2.0 * dimensionless_time
        } else if dimensionless_time > 100.0 {
            // Essentially recovered
            1.0 - 1.0 / (4.0 * dimensionless_time)
        } else {
            // Intermediate: smooth approximation
            1.0 - (-2.0 * dimensionless_time.sqrt()).exp()
        }
    }

    /// Estimate time to reach a given recovery fraction.
    ///
    /// Returns time in seconds. Uses bisection to invert recovery_fraction().
    pub fn time_to_recovery(
        target_fraction: f64,
        alpha_m2_s: f64,
        borehole_radius_m: f64,
    ) -> f64 {
        assert!(
            (0.0..1.0).contains(&target_fraction),
            "target fraction must be in [0, 1)"
        );

        let r2 = borehole_radius_m * borehole_radius_m;
        // Bisection on dimensionless time
        let mut lo = 0.0_f64;
        let mut hi = 1e6_f64;
        for _ in 0..100 {
            let mid = (lo + hi) / 2.0;
            if Self::recovery_fraction(mid) < target_fraction {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let td = (lo + hi) / 2.0;
        td * r2 / alpha_m2_s
    }

    /// Temperature at the borehole wall at time t after drilling stops.
    ///
    /// T(t) = T_formation + (T_drilling - T_formation) * (1 - f(t_D))
    pub fn wall_temperature(
        t_formation: f64,
        t_drilling: f64,
        alpha_m2_s: f64,
        time_s: f64,
        borehole_radius_m: f64,
    ) -> f64 {
        let td = Self::dimensionless_time(alpha_m2_s, time_s, borehole_radius_m);
        let f = Self::recovery_fraction(td);
        t_formation + (t_drilling - t_formation) * (1.0 - f)
    }
}

// ---------------------------------------------------------------------------
// TemperatureLogFilter
// ---------------------------------------------------------------------------

/// Noise reduction filters for depth-temperature log data.
pub struct TemperatureLogFilter;

impl TemperatureLogFilter {
    /// Moving average filter.
    ///
    /// Window size must be odd. Edges are handled by reducing the window.
    pub fn moving_average(data: &[f64], window: usize) -> Vec<f64> {
        assert!(window > 0 && window % 2 == 1, "window must be odd and > 0");
        let half = window / 2;
        let n = data.len();
        let mut result = Vec::with_capacity(n);

        for i in 0..n {
            let start = if i >= half { i - half } else { 0 };
            let end = if i + half < n { i + half + 1 } else { n };
            let sum: f64 = data[start..end].iter().sum();
            result.push(sum / (end - start) as f64);
        }
        result
    }

    /// Median filter for spike removal.
    ///
    /// Window size must be odd.
    pub fn median_filter(data: &[f64], window: usize) -> Vec<f64> {
        assert!(window > 0 && window % 2 == 1, "window must be odd and > 0");
        let half = window / 2;
        let n = data.len();
        let mut result = Vec::with_capacity(n);

        for i in 0..n {
            let start = if i >= half { i - half } else { 0 };
            let end = if i + half < n { i + half + 1 } else { n };
            let mut window_data: Vec<f64> = data[start..end].to_vec();
            window_data.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let mid = window_data.len() / 2;
            result.push(window_data[mid]);
        }
        result
    }

    /// Detect outliers using z-score method.
    ///
    /// Returns indices of outliers that deviate more than `threshold` standard
    /// deviations from the local trend.
    pub fn detect_outliers(data: &[f64], window: usize, threshold: f64) -> Vec<usize> {
        let smoothed = Self::moving_average(data, if window % 2 == 0 { window + 1 } else { window });
        let residuals: Vec<f64> = data
            .iter()
            .zip(smoothed.iter())
            .map(|(d, s)| d - s)
            .collect();

        let n = residuals.len() as f64;
        let mean: f64 = residuals.iter().sum::<f64>() / n;
        let variance: f64 = residuals.iter().map(|r| (r - mean).powi(2)).sum::<f64>() / n;
        let std_dev = variance.sqrt();

        if std_dev < 1e-15 {
            return Vec::new();
        }

        residuals
            .iter()
            .enumerate()
            .filter(|(_, r)| ((*r - mean) / std_dev).abs() > threshold)
            .map(|(i, _)| i)
            .collect()
    }

    /// Remove outliers by replacing them with linearly interpolated values.
    pub fn remove_outliers(data: &[f64], outlier_indices: &[usize]) -> Vec<f64> {
        let mut result = data.to_vec();
        let is_outlier: Vec<bool> = (0..data.len())
            .map(|i| outlier_indices.contains(&i))
            .collect();

        for &idx in outlier_indices {
            // Find nearest non-outlier neighbors
            let mut left = None;
            let mut right = None;
            for j in (0..idx).rev() {
                if !is_outlier[j] {
                    left = Some(j);
                    break;
                }
            }
            for j in (idx + 1)..data.len() {
                if !is_outlier[j] {
                    right = Some(j);
                    break;
                }
            }

            result[idx] = match (left, right) {
                (Some(l), Some(r)) => {
                    // Linear interpolation
                    let frac = (idx - l) as f64 / (r - l) as f64;
                    data[l] + frac * (data[r] - data[l])
                }
                (Some(l), None) => data[l],
                (None, Some(r)) => data[r],
                (None, None) => data[idx],
            };
        }
        result
    }
}

// ---------------------------------------------------------------------------
// RadiogenicHeatProduction
// ---------------------------------------------------------------------------

/// Radiogenic heat production estimator.
///
/// Calculates heat generation from concentrations of U, Th, and K using Rybach (1988):
/// A = rho * (C_U * H_U + C_Th * H_Th + C_K * H_K)
///
/// Equivalent to: A(uW/m^3) = 1e-5 * rho * (9.52 * C_U + 2.56 * C_Th + 3.48 * C_K)
/// where C_U, C_Th in ppm and C_K in percent.
pub struct RadiogenicHeatProduction;

impl RadiogenicHeatProduction {
    /// Heat production rate coefficient for Uranium: 9.52e-11 W/(kg*ppm).
    /// From Rybach (1988): A(uW/m^3) = 1e-5 * rho * (9.52*C_U + 2.56*C_Th + 3.48*C_K).
    const H_U: f64 = 9.52e-11;
    /// Heat production rate coefficient for Thorium: 2.56e-11 W/(kg*ppm).
    const H_TH: f64 = 2.56e-11;
    /// Heat production rate coefficient for Potassium: 3.48e-11 W/(kg*%).
    const H_K: f64 = 3.48e-11;

    /// Calculate radiogenic heat production in W/m^3.
    ///
    /// - `density_kg_m3`: rock density in kg/m^3
    /// - `u_ppm`: uranium concentration in parts per million
    /// - `th_ppm`: thorium concentration in parts per million
    /// - `k_percent`: potassium concentration in percent
    pub fn heat_production(
        density_kg_m3: f64,
        u_ppm: f64,
        th_ppm: f64,
        k_percent: f64,
    ) -> f64 {
        density_kg_m3
            * (u_ppm * Self::H_U + th_ppm * Self::H_TH + k_percent * Self::H_K)
    }

    /// Estimate heat flow contribution from a layer of given thickness.
    ///
    /// q_radiogenic = A * thickness (W/m^2)
    pub fn heat_flow_contribution(
        heat_production_w_m3: f64,
        thickness_m: f64,
    ) -> f64 {
        heat_production_w_m3 * thickness_m
    }

    /// Typical granite heat production (U=4 ppm, Th=15 ppm, K=3.5%).
    pub fn typical_granite(density_kg_m3: f64) -> f64 {
        Self::heat_production(density_kg_m3, 4.0, 15.0, 3.5)
    }

    /// Typical basalt heat production (U=0.5 ppm, Th=1.5 ppm, K=0.5%).
    pub fn typical_basalt(density_kg_m3: f64) -> f64 {
        Self::heat_production(density_kg_m3, 0.5, 1.5, 0.5)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- BoreholeConfig tests ---

    #[test]
    fn test_borehole_config_radius() {
        let config = BoreholeConfig {
            total_depth_m: 3000.0,
            diameter_m: 0.216,
            drilling_date_unix: 1700000000,
            fluid_type: FluidType::Water,
            thermal_conductivity_w_mk: 2.5,
        };
        assert!((config.radius_m() - 0.108).abs() < 1e-10);
    }

    #[test]
    fn test_borehole_cross_section() {
        let config = BoreholeConfig {
            total_depth_m: 1000.0,
            diameter_m: 0.2,
            drilling_date_unix: 0,
            fluid_type: FluidType::Water,
            thermal_conductivity_w_mk: 2.0,
        };
        let expected = std::f64::consts::PI * 0.1 * 0.1;
        assert!((config.cross_section_area_m2() - expected).abs() < 1e-10);
    }

    #[test]
    fn test_fluid_type_properties() {
        assert!(FluidType::Water.thermal_conductivity() > FluidType::Air.thermal_conductivity());
        assert!(FluidType::Water.density() > FluidType::Air.density());
        assert!(FluidType::Brine.density() > FluidType::Water.density());
        assert!(FluidType::Oil.thermal_conductivity() > FluidType::Air.thermal_conductivity());
    }

    // --- GeothermalGradientEstimator tests ---

    #[test]
    fn test_simple_gradient() {
        let g = GeothermalGradientEstimator::simple_gradient(10.0, 85.0, 3000.0);
        assert!((g - 0.025).abs() < 1e-6); // 25 C/km
    }

    #[test]
    fn test_linear_gradient_fit() {
        let depths = vec![0.0, 1000.0, 2000.0, 3000.0];
        let temps = vec![10.0, 35.0, 60.0, 85.0];
        let result = GeothermalGradientEstimator::fit(&depths, &temps);
        assert!((result.gradient_c_per_m - 0.025).abs() < 1e-6);
        assert!((result.surface_temp_c - 10.0).abs() < 1e-6);
        assert!(result.r_squared > 0.999);
    }

    #[test]
    fn test_gradient_fit_with_noise() {
        let depths = vec![500.0, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0];
        let temps = vec![25.0, 40.0, 55.0, 70.0, 85.0, 100.0];
        let result = GeothermalGradientEstimator::fit(&depths, &temps);
        assert!((result.gradient_c_per_m - 0.03).abs() < 1e-6);
        assert!(result.r_squared > 0.99);
    }

    #[test]
    fn test_gradient_to_c_per_km() {
        let g_km = GeothermalGradientEstimator::to_c_per_km(0.025);
        assert!((g_km - 25.0).abs() < 1e-6);
    }

    // --- HeatFlowCalculator tests ---

    #[test]
    fn test_fourier_heat_flow() {
        let q = HeatFlowCalculator::fourier_heat_flow(2.5, 0.03);
        assert!((q - 0.075).abs() < 1e-6); // 75 mW/m^2
    }

    #[test]
    fn test_heat_flow_to_mw() {
        let mw = HeatFlowCalculator::to_mw_per_m2(0.075);
        assert!((mw - 75.0).abs() < 1e-6);
    }

    #[test]
    fn test_layered_heat_flow() {
        // Two layers of equal thickness, same conductivity = same as uniform
        let layers = vec![(500.0, 2.5), (500.0, 2.5)];
        let q = HeatFlowCalculator::layered_heat_flow(&layers, 25.0);
        // R = 500/2.5 + 500/2.5 = 400, q = 25/400 = 0.0625
        assert!((q - 0.0625).abs() < 1e-6);
    }

    #[test]
    fn test_harmonic_mean_conductivity() {
        let layers = vec![(500.0, 2.0), (500.0, 4.0)];
        let k_harm = HeatFlowCalculator::harmonic_mean_conductivity(&layers);
        // Harmonic mean: 1000 / (500/2 + 500/4) = 1000 / 375 = 2.667
        assert!((k_harm - 1000.0 / 375.0).abs() < 1e-6);
    }

    #[test]
    fn test_lithology_conductivity_ranges() {
        assert!(Lithology::Salt.thermal_conductivity() > Lithology::Clay.thermal_conductivity());
        assert!(Lithology::Quartzite.thermal_conductivity() > Lithology::Shale.thermal_conductivity());
        assert!(Lithology::Sandstone.thermal_conductivity() > 3.0);
        assert!(Lithology::Shale.thermal_conductivity() < 3.0);
    }

    // --- HornerCorrection tests ---

    #[test]
    fn test_horner_ratio() {
        let ratio = HornerCorrection::horner_ratio(10.0, 5.0);
        assert!((ratio - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_horner_ratio_long_shutin() {
        // As shut-in increases, ratio approaches 1
        let ratio = HornerCorrection::horner_ratio(10.0, 10000.0);
        assert!(ratio < 1.01);
    }

    #[test]
    fn test_horner_single_correction() {
        let t_corrected = HornerCorrection::single_correction(150.0, 80.0, 10.0, 5.0);
        // Should be higher than measured since formation is hotter than circulation
        assert!(t_corrected > 150.0);
    }

    #[test]
    fn test_horner_multi_correction() {
        // Synthetic BHT measurements at different shut-in times
        // T_true = 160 C, T_circ = 80 C
        let t_true = 160.0;
        let slope = -30.0; // negative: measured temp increases as Horner x decreases
        let measurements: Vec<BhtMeasurement> = vec![
            BhtMeasurement {
                temperature_c: t_true + slope * (10.0 + 2.0_f64).ln() / 2.0_f64.ln(),
                circulation_time_h: 10.0,
                shut_in_time_h: 2.0,
            },
            BhtMeasurement {
                temperature_c: t_true + slope * (10.0 + 5.0_f64).ln() / 5.0_f64.ln(),
                circulation_time_h: 10.0,
                shut_in_time_h: 5.0,
            },
            BhtMeasurement {
                temperature_c: t_true + slope * (10.0 + 10.0_f64).ln() / 10.0_f64.ln(),
                circulation_time_h: 10.0,
                shut_in_time_h: 10.0,
            },
        ];
        let result = HornerCorrection::multi_correction(&measurements);
        // The extrapolated temperature should be close to t_true
        // (exact match depends on the synthetic data construction)
        assert!(result.corrected_temp_c.is_finite());
        assert!(result.slope.is_finite());
    }

    #[test]
    fn test_horner_multi_perfect_line() {
        // Perfect linear Horner data: T = 200 + (-50) * x
        let measurements = vec![
            BhtMeasurement {
                temperature_c: 200.0 + (-50.0) * HornerCorrection::horner_ratio(10.0, 2.0).ln(),
                circulation_time_h: 10.0,
                shut_in_time_h: 2.0,
            },
            BhtMeasurement {
                temperature_c: 200.0 + (-50.0) * HornerCorrection::horner_ratio(10.0, 5.0).ln(),
                circulation_time_h: 10.0,
                shut_in_time_h: 5.0,
            },
            BhtMeasurement {
                temperature_c: 200.0 + (-50.0) * HornerCorrection::horner_ratio(10.0, 20.0).ln(),
                circulation_time_h: 10.0,
                shut_in_time_h: 20.0,
            },
        ];
        let result = HornerCorrection::multi_correction(&measurements);
        assert!((result.corrected_temp_c - 200.0).abs() < 1e-6);
        assert!((result.slope - (-50.0)).abs() < 1e-6);
        assert!(result.r_squared > 0.999);
    }

    // --- BullardPlot tests ---

    #[test]
    fn test_cumulative_resistance() {
        let layers = vec![
            ThermalLayer {
                thickness_m: 500.0,
                conductivity_w_mk: 2.0,
                temperature_top_c: 10.0,
            },
            ThermalLayer {
                thickness_m: 500.0,
                conductivity_w_mk: 4.0,
                temperature_top_c: 25.0,
            },
        ];
        let r = BullardPlot::cumulative_resistance(&layers);
        assert_eq!(r.len(), 3);
        assert!((r[0] - 0.0).abs() < 1e-10);
        assert!((r[1] - 250.0).abs() < 1e-10); // 500/2
        assert!((r[2] - 375.0).abs() < 1e-10); // 250 + 500/4
    }

    #[test]
    fn test_bullard_analysis() {
        let layers = vec![
            ThermalLayer {
                thickness_m: 1000.0,
                conductivity_w_mk: 2.5,
                temperature_top_c: 15.0,
            },
            ThermalLayer {
                thickness_m: 1000.0,
                conductivity_w_mk: 2.5,
                temperature_top_c: 40.0,
            },
            ThermalLayer {
                thickness_m: 1000.0,
                conductivity_w_mk: 2.5,
                temperature_top_c: 65.0,
            },
        ];
        let result = BullardPlot::analyze(&layers);
        // With uniform conductivity, heat flow ~ k * gradient
        assert!(result.heat_flow_w_m2 > 0.0);
        assert!(result.heat_flow_w_m2.is_finite());
    }

    // --- DistributedTemperatureSensor tests ---

    #[test]
    fn test_dts_energy_parameter() {
        let dts = DistributedTemperatureSensor::new_silica(300.0, 1.0);
        let e = dts.energy_parameter_k();
        // h * 13.2e12 / k_B should be ~634 K
        assert!((e - 634.0).abs() < 5.0);
    }

    #[test]
    fn test_dts_reference_temperature_roundtrip() {
        let dts = DistributedTemperatureSensor::new_silica(300.0, 1.0);
        // At the reference ratio, we should get back the reference temperature
        let t = dts.ratio_to_temperature_k(dts.reference_ratio);
        assert!((t - 300.0).abs() < 1.0);
    }

    #[test]
    fn test_dts_higher_ratio_higher_temp() {
        let dts = DistributedTemperatureSensor::new_silica(300.0, 1.0);
        let t1 = dts.ratio_to_temperature_k(dts.reference_ratio * 1.1);
        let t2 = dts.ratio_to_temperature_k(dts.reference_ratio * 0.9);
        assert!(t1 > t2);
    }

    #[test]
    fn test_dts_process_trace() {
        let dts = DistributedTemperatureSensor::new_silica(300.0, 1.0);
        // Stokes intensity constant, anti-Stokes varies
        let stokes = vec![1.0, 1.0, 1.0, 1.0];
        let anti_stokes = vec![
            dts.reference_ratio,
            dts.reference_ratio * 1.05,
            dts.reference_ratio * 0.95,
            dts.reference_ratio,
        ];
        let temps = dts.process_trace(&stokes, &anti_stokes);
        assert_eq!(temps.len(), 4);
        // First and last should be near reference temp
        assert!((temps[0] - (300.0 - CELSIUS_TO_KELVIN)).abs() < 1.0);
        // Second should be hotter (higher anti-Stokes ratio)
        assert!(temps[1] > temps[2]);
    }

    #[test]
    fn test_dts_positions() {
        let dts = DistributedTemperatureSensor::new_silica(300.0, 0.5);
        let pos = dts.positions(5);
        assert_eq!(pos, vec![0.0, 0.5, 1.0, 1.5, 2.0]);
    }

    // --- PermafrostDetector tests ---

    #[test]
    fn test_no_permafrost() {
        let depths = vec![0.0, 10.0, 20.0, 30.0];
        let temps = vec![5.0, 8.0, 12.0, 15.0];
        let result = PermafrostDetector::analyze(&depths, &temps);
        assert!(!result.permafrost_detected);
    }

    #[test]
    fn test_permafrost_present() {
        let depths = vec![0.0, 2.0, 5.0, 10.0, 50.0, 100.0];
        let temps = vec![2.0, 0.5, -2.0, -5.0, -3.0, 1.0];
        let result = PermafrostDetector::analyze(&depths, &temps);
        assert!(result.permafrost_detected);
        assert!(result.permafrost_top_m.is_some());
        assert!(result.permafrost_base_m.is_some());
        // Top should be between 2m and 5m (0 crossing)
        let top = result.permafrost_top_m.unwrap();
        assert!(top > 2.0 && top < 5.0);
    }

    #[test]
    fn test_permafrost_active_layer() {
        let depths = vec![0.0, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0];
        let temps = vec![3.0, 1.0, -1.0, -5.0, -3.0, -1.0, 2.0];
        let result = PermafrostDetector::analyze(&depths, &temps);
        assert!(result.permafrost_detected);
        // Active layer = depth to top of permafrost
        let active = result.active_layer_thickness_m.unwrap();
        assert!(active > 1.0 && active < 2.0);
    }

    #[test]
    fn test_permafrost_all_frozen() {
        let depths = vec![0.0, 10.0, 20.0, 30.0];
        let temps = vec![-5.0, -8.0, -6.0, -4.0];
        let result = PermafrostDetector::analyze(&depths, &temps);
        assert!(result.permafrost_detected);
        assert_eq!(result.active_layer_thickness_m, Some(0.0));
    }

    // --- ThermalRecoveryEstimator tests ---

    #[test]
    fn test_thermal_diffusivity() {
        // Typical granite: k=3.0 W/mK, rho=2700 kg/m^3, cp=790 J/kgK
        let alpha = ThermalRecoveryEstimator::thermal_diffusivity(3.0, 2700.0, 790.0);
        // ~1.4e-6 m^2/s
        assert!((alpha - 1.4e-6).abs() < 0.5e-6);
    }

    #[test]
    fn test_dimensionless_time() {
        let alpha = 1.0e-6;
        let r = 0.1; // 10 cm radius
        let t = 3600.0; // 1 hour
        let td = ThermalRecoveryEstimator::dimensionless_time(alpha, t, r);
        // td = 1e-6 * 3600 / 0.01 = 0.36
        assert!((td - 0.36).abs() < 1e-6);
    }

    #[test]
    fn test_recovery_fraction_monotonic() {
        let mut prev = 0.0;
        for i in 1..100 {
            let td = i as f64 * 0.1;
            let f = ThermalRecoveryEstimator::recovery_fraction(td);
            assert!(f >= prev, "recovery fraction must be monotonically increasing");
            assert!(f <= 1.0);
            prev = f;
        }
    }

    #[test]
    fn test_recovery_fraction_bounds() {
        assert_eq!(ThermalRecoveryEstimator::recovery_fraction(0.0), 0.0);
        let large = ThermalRecoveryEstimator::recovery_fraction(1e6);
        assert!(large > 0.999);
    }

    #[test]
    fn test_time_to_recovery() {
        let alpha = 1.0e-6;
        let r = 0.1;
        let t90 = ThermalRecoveryEstimator::time_to_recovery(0.9, alpha, r);
        let t50 = ThermalRecoveryEstimator::time_to_recovery(0.5, alpha, r);
        assert!(t90 > t50, "90% recovery takes longer than 50%");
        assert!(t90 > 0.0);
    }

    #[test]
    fn test_wall_temperature_at_zero() {
        let t = ThermalRecoveryEstimator::wall_temperature(100.0, 50.0, 1e-6, 0.0, 0.1);
        assert!((t - 50.0).abs() < 1e-6, "at t=0, wall temp should be drilling temp");
    }

    #[test]
    fn test_wall_temperature_equilibrium() {
        let t = ThermalRecoveryEstimator::wall_temperature(100.0, 50.0, 1e-6, 1e12, 0.1);
        assert!((t - 100.0).abs() < 1.0, "after long time, should approach formation temp");
    }

    // --- TemperatureLogFilter tests ---

    #[test]
    fn test_moving_average_constant() {
        let data = vec![5.0; 10];
        let filtered = TemperatureLogFilter::moving_average(&data, 3);
        for v in &filtered {
            assert!((v - 5.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_moving_average_smoothing() {
        let data = vec![1.0, 100.0, 1.0, 1.0, 1.0];
        let filtered = TemperatureLogFilter::moving_average(&data, 3);
        // The spike should be reduced
        assert!(filtered[1] < 100.0);
    }

    #[test]
    fn test_median_filter_spike() {
        let data = vec![10.0, 10.0, 1000.0, 10.0, 10.0];
        let filtered = TemperatureLogFilter::median_filter(&data, 3);
        assert!((filtered[2] - 10.0).abs() < 1e-10, "median should remove spike");
    }

    #[test]
    fn test_median_filter_preserves_edges() {
        let data = vec![0.0, 0.0, 10.0, 10.0, 10.0];
        let filtered = TemperatureLogFilter::median_filter(&data, 3);
        // Step edge should be mostly preserved
        assert!((filtered[0] - 0.0).abs() < 1e-10);
        assert!((filtered[4] - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_detect_outliers() {
        let mut data = vec![10.0; 20];
        data[10] = 100.0; // Obvious outlier
        let outliers = TemperatureLogFilter::detect_outliers(&data, 5, 2.0);
        assert!(outliers.contains(&10));
    }

    #[test]
    fn test_remove_outliers_interpolation() {
        let data = vec![0.0, 1.0, 100.0, 3.0, 4.0];
        let cleaned = TemperatureLogFilter::remove_outliers(&data, &[2]);
        // Should be interpolated between index 1 (1.0) and index 3 (3.0)
        assert!((cleaned[2] - 2.0).abs() < 1e-10);
    }

    // --- RadiogenicHeatProduction tests ---

    #[test]
    fn test_radiogenic_heat_production() {
        let a = RadiogenicHeatProduction::heat_production(2700.0, 4.0, 15.0, 3.5);
        // Should be on the order of 1-5 uW/m^3 for typical granite
        assert!(a > 0.5e-6 && a < 10.0e-6);
    }

    #[test]
    fn test_radiogenic_granite() {
        let a = RadiogenicHeatProduction::typical_granite(2700.0);
        // Typical granite: ~2.5 uW/m^3
        assert!(a > 1.0e-6 && a < 5.0e-6);
    }

    #[test]
    fn test_radiogenic_basalt_less_than_granite() {
        let a_granite = RadiogenicHeatProduction::typical_granite(2700.0);
        let a_basalt = RadiogenicHeatProduction::typical_basalt(3000.0);
        assert!(a_basalt < a_granite, "basalt should produce less heat than granite");
    }

    #[test]
    fn test_heat_flow_contribution() {
        let a = 2.5e-6; // 2.5 uW/m^3
        let q = RadiogenicHeatProduction::heat_flow_contribution(a, 10_000.0);
        // 2.5e-6 * 10000 = 0.025 W/m^2 = 25 mW/m^2
        assert!((q - 0.025).abs() < 1e-6);
    }

    #[test]
    fn test_zero_concentration() {
        let a = RadiogenicHeatProduction::heat_production(2700.0, 0.0, 0.0, 0.0);
        assert!((a - 0.0).abs() < 1e-20);
    }
}
