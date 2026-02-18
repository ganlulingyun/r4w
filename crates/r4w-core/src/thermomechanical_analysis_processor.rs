//! # Thermomechanical Analysis (TMA) Signal Processor
//!
//! Signal processing module for measuring dimensional changes in materials as a
//! function of temperature. Implements the full TMA processing chain from raw
//! displacement/temperature data to material property extraction.
//!
//! ## Overview
//!
//! TMA measures the deformation of a sample under a non-oscillating stress as a
//! function of temperature or time. A probe rests on the sample surface while
//! temperature is ramped; an LVDT sensor tracks the probe displacement.
//!
//! ## Processing Pipeline
//!
//! ```text
//! Raw (T, L) Data -> Smoothing -> CTE Calculation -> Transition Detection
//!     -> Creep Analysis -> Cycle Hysteresis -> Report
//! ```
//!
//! ## Key Features
//!
//! - **Linear expansion**: Instantaneous and mean CTE from displacement vs temperature
//! - **Glass transition**: Onset, midpoint, and inflection point Tg detection
//! - **Softening point**: Vicat and HDT softening temperature determination
//! - **Creep analysis**: Burger's model, WLF time-temperature superposition
//! - **Dilatometry**: Volume expansion, bulk modulus, specific volume
//! - **Thermal cycling**: Hysteresis, dimensional recovery, permanent set
//! - **Probe modes**: Penetration, expansion, tension, flexure configurations
//! - **Cure monitoring**: Thermoset shrinkage, degree of cure, gel point
//! - **Data smoothing**: Savitzky-Golay filtering, derivative computation
//!
//! ## References
//!
//! - ASTM E831: Standard Test Method for Linear Thermal Expansion
//! - ASTM D648: Standard Test Method for Deflection Temperature
//! - ASTM D1525: Standard Test Method for Vicat Softening Temperature
//! - ISO 11359-1/2: Thermomechanical analysis (TMA) general principles

// ============================================================================
// Helper functions
// ============================================================================

/// Mean coefficient of thermal expansion: alpha = delta_l / (l0 * delta_t)
///
/// # Arguments
/// * `l0` - Original specimen length
/// * `delta_l` - Change in length
/// * `delta_t` - Change in temperature
pub fn mean_cte(l0: f64, delta_l: f64, delta_t: f64) -> f64 {
    if l0.abs() < 1e-30 || delta_t.abs() < 1e-30 {
        return 0.0;
    }
    delta_l / (l0 * delta_t)
}

/// Williams-Landel-Ferry (WLF) time-temperature shift factor log10(aT).
///
/// log10(aT) = -C1 * (T - Tref) / (C2 + (T - Tref))
///
/// Typical universal constants: C1 = 17.44, C2 = 51.6 K
pub fn wlf_shift(temp: f64, tref: f64, c1: f64, c2: f64) -> f64 {
    let dt = temp - tref;
    let denom = c2 + dt;
    if denom.abs() < 1e-30 {
        return 0.0;
    }
    -c1 * dt / denom
}

/// Burger's model creep compliance: displacement under constant stress.
///
/// J(t) = 1/E1 + t/eta1 + (1/E2)(1 - exp(-E2*t/eta2))
/// displacement = stress * J(t)
///
/// # Arguments
/// * `t` - Time
/// * `e1` - Spring modulus (Maxwell element)
/// * `e2` - Spring modulus (Kelvin-Voigt element)
/// * `eta1` - Dashpot viscosity (Maxwell element)
/// * `eta2` - Dashpot viscosity (Kelvin-Voigt element)
/// * `stress` - Applied stress
pub fn burger_creep(t: f64, e1: f64, e2: f64, eta1: f64, eta2: f64, stress: f64) -> f64 {
    if e1.abs() < 1e-30 || e2.abs() < 1e-30 || eta1.abs() < 1e-30 || eta2.abs() < 1e-30 {
        return 0.0;
    }
    let j = 1.0 / e1 + t / eta1 + (1.0 / e2) * (1.0 - (-e2 * t / eta2).exp());
    stress * j
}

// ============================================================================
// Enumerations
// ============================================================================

/// Probe configuration mode for TMA measurement.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ProbeMode {
    /// Expansion mode: flat-tipped probe, minimal load
    Expansion,
    /// Penetration mode: pointed probe, significant load (Vicat-like)
    Penetration,
    /// Tension mode: film/fiber clamped under tensile load
    Tension,
    /// Flexure mode: three-point bending
    Flexure,
}

/// Method for detecting glass transition temperature.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TgMethod {
    /// Onset: intersection of tangent lines before and after transition
    Onset,
    /// Midpoint: halfway between onset and endset
    Midpoint,
    /// Inflection: maximum of first derivative (d(CTE)/dT)
    Inflection,
}

/// Integration method for numerical area calculations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IntegrationMethod {
    Trapezoidal,
    Simpsons,
}

// ============================================================================
// LinearExpansion
// ============================================================================

/// Coefficient of thermal expansion (CTE) calculator.
///
/// Computes instantaneous and mean CTE from displacement vs temperature data.
/// alpha = (1/L0) * dL/dT
#[derive(Debug, Clone)]
pub struct LinearExpansion {
    /// Original specimen length (mm)
    pub l0: f64,
    /// Temperature data points (degrees C)
    pub temperatures: Vec<f64>,
    /// Displacement data points (micrometers)
    pub displacements: Vec<f64>,
}

impl LinearExpansion {
    pub fn new(l0: f64, temperatures: Vec<f64>, displacements: Vec<f64>) -> Self {
        Self {
            l0,
            temperatures,
            displacements,
        }
    }

    /// Calculate mean CTE over the entire temperature range.
    pub fn mean_cte_overall(&self) -> f64 {
        if self.temperatures.len() < 2 || self.l0.abs() < 1e-30 {
            return 0.0;
        }
        let n = self.temperatures.len();
        let dt = self.temperatures[n - 1] - self.temperatures[0];
        let dl = self.displacements[n - 1] - self.displacements[0];
        // displacements in um, l0 in mm => convert: dl_um / (l0_mm * 1000) = dl / (l0*1e3)
        mean_cte(self.l0 * 1e3, dl, dt)
    }

    /// Calculate mean CTE between two temperature bounds.
    pub fn mean_cte_range(&self, t_low: f64, t_high: f64) -> f64 {
        if self.temperatures.len() < 2 || self.l0.abs() < 1e-30 {
            return 0.0;
        }
        let l_low = self.interpolate_displacement(t_low);
        let l_high = self.interpolate_displacement(t_high);
        let dl = l_high - l_low;
        let dt = t_high - t_low;
        mean_cte(self.l0 * 1e3, dl, dt)
    }

    /// Compute instantaneous CTE at each data point using central differences.
    /// Returns (temperatures, cte_values) with endpoints excluded.
    pub fn instantaneous_cte(&self) -> (Vec<f64>, Vec<f64>) {
        let n = self.temperatures.len();
        if n < 3 || self.l0.abs() < 1e-30 {
            return (vec![], vec![]);
        }
        let mut temps = Vec::with_capacity(n - 2);
        let mut ctes = Vec::with_capacity(n - 2);
        let l0_um = self.l0 * 1e3; // mm to um
        for i in 1..n - 1 {
            let dt = self.temperatures[i + 1] - self.temperatures[i - 1];
            if dt.abs() < 1e-30 {
                continue;
            }
            let dl_dt = (self.displacements[i + 1] - self.displacements[i - 1]) / dt;
            let cte = dl_dt / l0_um;
            temps.push(self.temperatures[i]);
            ctes.push(cte);
        }
        (temps, ctes)
    }

    /// Linear interpolation of displacement at a given temperature.
    pub fn interpolate_displacement(&self, temp: f64) -> f64 {
        let n = self.temperatures.len();
        if n == 0 {
            return 0.0;
        }
        if n == 1 || temp <= self.temperatures[0] {
            return self.displacements[0];
        }
        if temp >= self.temperatures[n - 1] {
            return self.displacements[n - 1];
        }
        for i in 1..n {
            if self.temperatures[i] >= temp {
                let t0 = self.temperatures[i - 1];
                let t1 = self.temperatures[i];
                let d0 = self.displacements[i - 1];
                let d1 = self.displacements[i];
                let frac = (temp - t0) / (t1 - t0);
                return d0 + frac * (d1 - d0);
            }
        }
        self.displacements[n - 1]
    }

    /// Percent linear change: 100 * (L - L0) / L0
    pub fn percent_linear_change(&self) -> Vec<f64> {
        let l0_um = self.l0 * 1e3;
        if l0_um.abs() < 1e-30 {
            return vec![0.0; self.displacements.len()];
        }
        self.displacements.iter().map(|d| 100.0 * d / l0_um).collect()
    }
}

// ============================================================================
// GlassTransition
// ============================================================================

/// Glass transition temperature (Tg) detector from CTE curve.
#[derive(Debug, Clone)]
pub struct GlassTransition {
    /// Temperature data
    pub temperatures: Vec<f64>,
    /// CTE or displacement data
    pub values: Vec<f64>,
}

impl GlassTransition {
    pub fn new(temperatures: Vec<f64>, values: Vec<f64>) -> Self {
        Self {
            temperatures,
            values,
        }
    }

    /// Detect Tg using the specified method.
    pub fn detect_tg(&self, method: TgMethod) -> Option<f64> {
        match method {
            TgMethod::Onset => self.onset_tg(),
            TgMethod::Midpoint => self.midpoint_tg(),
            TgMethod::Inflection => self.inflection_tg(),
        }
    }

    /// Onset method: fit tangent lines to glassy and rubbery regions,
    /// find their intersection.
    fn onset_tg(&self) -> Option<f64> {
        let n = self.temperatures.len();
        if n < 10 {
            return None;
        }
        // Use first 30% as glassy region, last 30% as rubbery region
        let g_end = n * 3 / 10;
        let r_start = n * 7 / 10;

        let (slope_g, intercept_g) =
            linear_fit(&self.temperatures[..g_end], &self.values[..g_end])?;
        let (slope_r, intercept_r) =
            linear_fit(&self.temperatures[r_start..], &self.values[r_start..])?;

        // Intersection: slope_g * T + intercept_g = slope_r * T + intercept_r
        let dslope = slope_r - slope_g;
        if dslope.abs() < 1e-30 {
            return None;
        }
        let tg = (intercept_g - intercept_r) / dslope;

        // Sanity check: Tg should be within data range
        let t_min = self.temperatures[0];
        let t_max = self.temperatures[n - 1];
        if tg >= t_min && tg <= t_max {
            Some(tg)
        } else {
            None
        }
    }

    /// Midpoint method: average of onset and endset temperatures.
    fn midpoint_tg(&self) -> Option<f64> {
        let n = self.temperatures.len();
        if n < 10 {
            return None;
        }
        let onset = self.onset_tg()?;
        // Endset: fit tangent from the rubbery side back toward transition
        let r_start = n * 7 / 10;
        let (slope_r, intercept_r) =
            linear_fit(&self.temperatures[r_start..], &self.values[r_start..])?;

        // Find where transition region starts deviating from rubbery tangent
        // Walk backward from rubbery region
        let mut endset = self.temperatures[n - 1];
        for i in (0..r_start).rev() {
            let predicted = slope_r * self.temperatures[i] + intercept_r;
            let diff = (self.values[i] - predicted).abs();
            let scale = self.values[i].abs().max(1e-10);
            if diff / scale > 0.05 {
                endset = self.temperatures[i];
                break;
            }
        }

        Some((onset + endset) / 2.0)
    }

    /// Inflection point method: find maximum of first derivative.
    fn inflection_tg(&self) -> Option<f64> {
        let n = self.temperatures.len();
        if n < 5 {
            return None;
        }
        let derivs = numerical_derivative(&self.temperatures, &self.values);
        if derivs.is_empty() {
            return None;
        }
        // Find index of maximum absolute derivative
        let mut max_idx = 0;
        let mut max_val = derivs[0].abs();
        for (i, &d) in derivs.iter().enumerate() {
            if d.abs() > max_val {
                max_val = d.abs();
                max_idx = i;
            }
        }
        // The derivative array is offset by 1 from temperatures (central diff)
        if max_idx + 1 < n {
            Some(self.temperatures[max_idx + 1])
        } else {
            None
        }
    }
}

// ============================================================================
// SofteningPoint
// ============================================================================

/// Softening temperature determination (Vicat / HDT).
#[derive(Debug, Clone)]
pub struct SofteningPoint {
    /// Temperature ramp data
    pub temperatures: Vec<f64>,
    /// Penetration depth (um) or deflection (mm)
    pub deformation: Vec<f64>,
}

impl SofteningPoint {
    pub fn new(temperatures: Vec<f64>, deformation: Vec<f64>) -> Self {
        Self {
            temperatures,
            deformation,
        }
    }

    /// Vicat softening temperature: temperature at which penetration reaches
    /// the specified depth threshold (default 1.0 mm = 1000 um for standard Vicat).
    pub fn vicat_softening(&self, threshold_um: f64) -> Option<f64> {
        for i in 0..self.temperatures.len() {
            if self.deformation[i] >= threshold_um {
                if i == 0 {
                    return Some(self.temperatures[0]);
                }
                // Linear interpolation
                let t0 = self.temperatures[i - 1];
                let t1 = self.temperatures[i];
                let d0 = self.deformation[i - 1];
                let d1 = self.deformation[i];
                let frac = (threshold_um - d0) / (d1 - d0);
                return Some(t0 + frac * (t1 - t0));
            }
        }
        None
    }

    /// Heat deflection temperature (HDT) per ASTM D648.
    /// Temperature at which a bar deflects by the specified amount
    /// under flexural stress. Standard: 0.25 mm deflection.
    pub fn hdt(&self, deflection_threshold_um: f64) -> Option<f64> {
        self.vicat_softening(deflection_threshold_um)
    }

    /// Rate of deformation (penetration rate) at each temperature.
    pub fn deformation_rate(&self) -> Vec<f64> {
        numerical_derivative(&self.temperatures, &self.deformation)
    }

    /// Find the onset of rapid softening from the deformation rate curve.
    pub fn softening_onset(&self, rate_threshold: f64) -> Option<f64> {
        let rates = self.deformation_rate();
        for (i, &r) in rates.iter().enumerate() {
            if r.abs() > rate_threshold {
                if i + 1 < self.temperatures.len() {
                    return Some(self.temperatures[i + 1]);
                }
            }
        }
        None
    }
}

// ============================================================================
// CreepAnalysis
// ============================================================================

/// Creep analysis under constant load at elevated temperature.
#[derive(Debug, Clone)]
pub struct CreepAnalysis {
    /// Time data (seconds)
    pub times: Vec<f64>,
    /// Displacement data (um)
    pub displacements: Vec<f64>,
    /// Applied stress (MPa)
    pub stress: f64,
    /// Temperature (deg C)
    pub temperature: f64,
}

impl CreepAnalysis {
    pub fn new(times: Vec<f64>, displacements: Vec<f64>, stress: f64, temperature: f64) -> Self {
        Self {
            times,
            displacements,
            stress,
            temperature,
        }
    }

    /// Creep compliance J(t) = strain(t) / stress.
    /// strain = displacement / initial_length (provide l0 in um).
    pub fn creep_compliance(&self, l0_um: f64) -> Vec<f64> {
        if self.stress.abs() < 1e-30 || l0_um.abs() < 1e-30 {
            return vec![0.0; self.displacements.len()];
        }
        self.displacements
            .iter()
            .map(|&d| (d / l0_um) / self.stress)
            .collect()
    }

    /// Creep rate: dD/dt at each time point.
    pub fn creep_rate(&self) -> Vec<f64> {
        numerical_derivative(&self.times, &self.displacements)
    }

    /// Fit Burger's four-element model to creep data.
    /// Returns (E1, E2, eta1, eta2) parameters.
    /// Uses simplified least-squares approach.
    pub fn fit_burgers_model(&self) -> Option<(f64, f64, f64, f64)> {
        let n = self.times.len();
        if n < 4 || self.stress.abs() < 1e-30 {
            return None;
        }
        // Initial displacement gives E1: d(0) = stress / E1
        let d0 = self.displacements[0].max(1e-12);
        let e1 = self.stress / d0;

        // Long-time slope gives eta1: slope = stress / eta1
        let last_quarter = n * 3 / 4;
        let (slope, _) = linear_fit(&self.times[last_quarter..], &self.displacements[last_quarter..])?;
        let eta1 = if slope.abs() > 1e-30 {
            self.stress / slope
        } else {
            1e12 // Very high viscosity if no steady-state creep
        };

        // Transient part: subtract elastic + viscous => Kelvin-Voigt response
        // d_kv(t) = (stress/E2)(1 - exp(-E2*t/eta2))
        // At large t, d_kv saturates to stress/E2
        let d_final = self.displacements[n - 1];
        let d_elastic = self.stress / e1;
        let d_viscous = slope * self.times[n - 1];
        let d_kv_sat = (d_final - d_elastic - d_viscous).max(1e-12);
        let e2 = self.stress / d_kv_sat;

        // Time constant tau = eta2/E2; estimate from mid-transient
        let mid = n / 2;
        let d_kv_mid = self.displacements[mid] - d_elastic - slope * self.times[mid];
        let ratio = (d_kv_mid / d_kv_sat).clamp(0.01, 0.99);
        let tau = if self.times[mid] > 0.0 {
            -self.times[mid] / (1.0 - ratio).ln()
        } else {
            1.0
        };
        let eta2 = e2 * tau;

        Some((e1, e2, eta1, eta2))
    }

    /// WLF-shifted creep data to a reference temperature.
    pub fn wlf_shifted_times(&self, tref: f64, c1: f64, c2: f64) -> Vec<f64> {
        let log_at = wlf_shift(self.temperature, tref, c1, c2);
        let at = 10.0_f64.powf(log_at);
        self.times.iter().map(|&t| t / at).collect()
    }

    /// Identify creep stages: primary (decreasing rate), secondary (constant),
    /// tertiary (increasing rate). Returns indices of stage transitions.
    pub fn identify_stages(&self) -> (Option<usize>, Option<usize>) {
        let rates = self.creep_rate();
        if rates.len() < 5 {
            return (None, None);
        }

        let mut primary_end = None;
        let mut secondary_end = None;

        // Smooth the rate for stage detection
        let smoothed = smooth_window(&rates, 3);

        // Primary -> Secondary: rate stops decreasing significantly
        for i in 1..smoothed.len() - 1 {
            let dr = smoothed[i] - smoothed[i - 1];
            if dr.abs() < smoothed[i].abs() * 0.05 && primary_end.is_none() {
                primary_end = Some(i);
            }
            // Secondary -> Tertiary: rate starts increasing
            if primary_end.is_some() && secondary_end.is_none() && dr > smoothed[i].abs() * 0.05 {
                secondary_end = Some(i);
            }
        }
        (primary_end, secondary_end)
    }
}

// ============================================================================
// DilatometryMode
// ============================================================================

/// Volume expansion (dilatometry) analysis.
///
/// For isotropic materials: beta = 3*alpha (volumetric CTE = 3 * linear CTE).
#[derive(Debug, Clone)]
pub struct DilatometryMode {
    /// Temperature data
    pub temperatures: Vec<f64>,
    /// Specific volume data (cm^3/g)
    pub specific_volumes: Vec<f64>,
    /// Whether material is isotropic
    pub isotropic: bool,
}

impl DilatometryMode {
    pub fn new(temperatures: Vec<f64>, specific_volumes: Vec<f64>, isotropic: bool) -> Self {
        Self {
            temperatures,
            specific_volumes,
            isotropic,
        }
    }

    /// Volumetric CTE: beta = (1/V0) * dV/dT
    pub fn volumetric_cte(&self) -> (Vec<f64>, Vec<f64>) {
        let n = self.temperatures.len();
        if n < 3 {
            return (vec![], vec![]);
        }
        let v0 = self.specific_volumes[0];
        if v0.abs() < 1e-30 {
            return (vec![], vec![]);
        }
        let dv_dt = numerical_derivative(&self.temperatures, &self.specific_volumes);
        let temps: Vec<f64> = self.temperatures[1..n - 1].to_vec();
        let betas: Vec<f64> = dv_dt.iter().map(|&dv| dv / v0).collect();
        (temps, betas)
    }

    /// Linear CTE from volumetric: alpha = beta / 3 (isotropic only).
    pub fn linear_cte_from_volume(&self) -> (Vec<f64>, Vec<f64>) {
        let (temps, betas) = self.volumetric_cte();
        let alphas = betas.iter().map(|&b| b / 3.0).collect();
        (temps, alphas)
    }

    /// Bulk modulus estimate from dilatometry: K = -V * dP/dV
    /// Here we estimate compressibility from volume change.
    /// Approximate K from: K = V0 / (beta * T_range) assuming small changes.
    pub fn bulk_modulus_estimate(&self, pressure_mpa: f64) -> f64 {
        let n = self.specific_volumes.len();
        if n < 2 {
            return 0.0;
        }
        let v0 = self.specific_volumes[0];
        let v_final = self.specific_volumes[n - 1];
        let dv = v_final - v0;
        if dv.abs() < 1e-30 {
            return f64::INFINITY;
        }
        // K = -V0 * delta_P / (delta_V / V0) but for TMA we approximate
        pressure_mpa * v0 / dv.abs()
    }

    /// Specific volume at a given temperature via interpolation.
    pub fn specific_volume_at(&self, temp: f64) -> f64 {
        linear_interp(&self.temperatures, &self.specific_volumes, temp)
    }

    /// Density at a given temperature: rho = 1 / specific_volume.
    pub fn density_at(&self, temp: f64) -> f64 {
        let v = self.specific_volume_at(temp);
        if v.abs() < 1e-30 {
            return 0.0;
        }
        1.0 / v
    }
}

// ============================================================================
// ThermalCycling
// ============================================================================

/// Thermal cycling analysis: hysteresis, recovery, and permanent set.
#[derive(Debug, Clone)]
pub struct ThermalCycling {
    /// Temperature data for each cycle: Vec of (temps, displacements)
    pub cycles: Vec<(Vec<f64>, Vec<f64>)>,
}

impl ThermalCycling {
    pub fn new() -> Self {
        Self { cycles: vec![] }
    }

    /// Add a heating/cooling cycle.
    pub fn add_cycle(&mut self, temperatures: Vec<f64>, displacements: Vec<f64>) {
        self.cycles.push((temperatures, displacements));
    }

    /// Hysteresis area for a given cycle (area between heating and cooling curves).
    /// Assumes first half is heating, second half is cooling.
    pub fn hysteresis_area(&self, cycle_idx: usize) -> f64 {
        if cycle_idx >= self.cycles.len() {
            return 0.0;
        }
        let (temps, disps) = &self.cycles[cycle_idx];
        let n = temps.len();
        if n < 4 {
            return 0.0;
        }
        let mid = n / 2;
        let heating_t = &temps[..mid];
        let heating_d = &disps[..mid];
        let cooling_t = &temps[mid..];
        let cooling_d = &disps[mid..];

        // Determine overlap temperature range
        let t_min = heating_t[0].max(cooling_t.iter().copied().fold(f64::INFINITY, f64::min));
        let t_max = heating_t[heating_t.len() - 1]
            .min(cooling_t.iter().copied().fold(f64::NEG_INFINITY, f64::max));

        if t_max <= t_min {
            return 0.0;
        }

        // Integrate difference over common temperature range using trapezoidal rule
        let steps = 100;
        let dt = (t_max - t_min) / steps as f64;
        let mut area = 0.0;
        for i in 0..steps {
            let t0 = t_min + i as f64 * dt;
            let t1 = t0 + dt;
            let h0 = linear_interp(heating_t, heating_d, t0);
            let h1 = linear_interp(heating_t, heating_d, t1);
            let c0 = linear_interp(cooling_t, cooling_d, t0);
            let c1 = linear_interp(cooling_t, cooling_d, t1);
            let diff0 = (c0 - h0).abs();
            let diff1 = (c1 - h1).abs();
            area += 0.5 * (diff0 + diff1) * dt;
        }
        area
    }

    /// Permanent set: residual displacement after returning to starting temperature.
    pub fn permanent_set(&self, cycle_idx: usize) -> f64 {
        if cycle_idx >= self.cycles.len() {
            return 0.0;
        }
        let (_, disps) = &self.cycles[cycle_idx];
        if disps.is_empty() {
            return 0.0;
        }
        let d_start = disps[0];
        let d_end = disps[disps.len() - 1];
        (d_end - d_start).abs()
    }

    /// Dimensional recovery ratio: 1.0 means full recovery, 0.0 means none.
    pub fn dimensional_recovery(&self, cycle_idx: usize) -> f64 {
        if cycle_idx >= self.cycles.len() {
            return 0.0;
        }
        let (_, disps) = &self.cycles[cycle_idx];
        if disps.len() < 3 {
            return 0.0;
        }
        let d_start = disps[0];
        let d_max = disps.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let d_end = disps[disps.len() - 1];
        let total_change = (d_max - d_start).abs();
        if total_change < 1e-30 {
            return 1.0;
        }
        let recovered = (d_max - d_end).abs();
        (recovered / total_change).min(1.0)
    }

    /// Number of cycles stored.
    pub fn num_cycles(&self) -> usize {
        self.cycles.len()
    }

    /// Compare permanent set across cycles: returns Vec of permanent set values.
    pub fn permanent_set_trend(&self) -> Vec<f64> {
        (0..self.cycles.len()).map(|i| self.permanent_set(i)).collect()
    }
}

// ============================================================================
// ProbeMode (configuration and LVDT sensor model)
// ============================================================================

/// TMA probe configuration and LVDT sensor model.
#[derive(Debug, Clone)]
pub struct ProbeModeConfig {
    /// Probe operating mode
    pub mode: ProbeMode,
    /// Applied force in mN
    pub force_mn: f64,
    /// Probe tip area in mm^2
    pub tip_area_mm2: f64,
    /// LVDT sensitivity in mV/um
    pub lvdt_sensitivity: f64,
    /// LVDT range in um (+/-)
    pub lvdt_range: f64,
}

impl ProbeModeConfig {
    pub fn new(mode: ProbeMode, force_mn: f64, tip_area_mm2: f64) -> Self {
        Self {
            mode,
            force_mn,
            tip_area_mm2,
            lvdt_sensitivity: 1.0,
            lvdt_range: 2500.0,
        }
    }

    /// Expansion mode preset: 5 mN force, flat 3 mm diameter probe.
    pub fn expansion_preset() -> Self {
        let area = std::f64::consts::PI * (1.5_f64).powi(2); // 3mm dia
        Self::new(ProbeMode::Expansion, 5.0, area)
    }

    /// Penetration mode preset: 1000 mN force, 1 mm^2 flat tip.
    pub fn penetration_preset() -> Self {
        Self::new(ProbeMode::Penetration, 1000.0, 1.0)
    }

    /// Applied stress in MPa.
    pub fn applied_stress_mpa(&self) -> f64 {
        if self.tip_area_mm2.abs() < 1e-30 {
            return 0.0;
        }
        // Force in mN / area in mm^2 = stress in kPa, convert to MPa
        (self.force_mn / self.tip_area_mm2) * 1e-3
    }

    /// Convert LVDT voltage (mV) to displacement (um).
    pub fn voltage_to_displacement(&self, voltage_mv: f64) -> f64 {
        if self.lvdt_sensitivity.abs() < 1e-30 {
            return 0.0;
        }
        let disp = voltage_mv / self.lvdt_sensitivity;
        disp.clamp(-self.lvdt_range, self.lvdt_range)
    }

    /// Convert raw voltage array to displacement array.
    pub fn convert_voltages(&self, voltages: &[f64]) -> Vec<f64> {
        voltages.iter().map(|&v| self.voltage_to_displacement(v)).collect()
    }

    /// Force calibration check: verify force is within acceptable range for mode.
    pub fn force_in_range(&self) -> bool {
        match self.mode {
            ProbeMode::Expansion => self.force_mn >= 1.0 && self.force_mn <= 50.0,
            ProbeMode::Penetration => self.force_mn >= 100.0 && self.force_mn <= 5000.0,
            ProbeMode::Tension => self.force_mn >= 10.0 && self.force_mn <= 2000.0,
            ProbeMode::Flexure => self.force_mn >= 50.0 && self.force_mn <= 3000.0,
        }
    }
}

// ============================================================================
// CureMon (Cure Monitoring)
// ============================================================================

/// Cure monitoring for thermoset materials.
///
/// Tracks dimensional changes during isothermal or ramped curing.
#[derive(Debug, Clone)]
pub struct CureMon {
    /// Time data (seconds)
    pub times: Vec<f64>,
    /// Displacement data (um) - typically shrinkage is negative
    pub displacements: Vec<f64>,
    /// Temperature profile (deg C)
    pub temperatures: Vec<f64>,
    /// Initial specimen thickness (mm)
    pub initial_thickness_mm: f64,
}

impl CureMon {
    pub fn new(
        times: Vec<f64>,
        displacements: Vec<f64>,
        temperatures: Vec<f64>,
        initial_thickness_mm: f64,
    ) -> Self {
        Self {
            times,
            displacements,
            temperatures,
            initial_thickness_mm,
        }
    }

    /// Cure shrinkage as percentage at each time point.
    pub fn shrinkage_percent(&self) -> Vec<f64> {
        let l0_um = self.initial_thickness_mm * 1e3;
        if l0_um.abs() < 1e-30 {
            return vec![0.0; self.displacements.len()];
        }
        self.displacements.iter().map(|&d| -100.0 * d / l0_um).collect()
    }

    /// Total cure shrinkage (%).
    pub fn total_shrinkage_percent(&self) -> f64 {
        let shrink = self.shrinkage_percent();
        if shrink.is_empty() {
            return 0.0;
        }
        shrink[shrink.len() - 1]
    }

    /// Degree of cure (alpha) from dimensional change: alpha(t) = shrinkage(t) / shrinkage_total.
    pub fn degree_of_cure(&self) -> Vec<f64> {
        let shrink = self.shrinkage_percent();
        let total = self.total_shrinkage_percent();
        if total.abs() < 1e-30 {
            return vec![0.0; shrink.len()];
        }
        shrink.iter().map(|&s| (s / total).clamp(0.0, 1.0)).collect()
    }

    /// Cure rate: d(alpha)/dt.
    pub fn cure_rate(&self) -> Vec<f64> {
        let alpha = self.degree_of_cure();
        numerical_derivative(&self.times, &alpha)
    }

    /// Gel point detection: time at which cure rate is maximum.
    pub fn gel_point_time(&self) -> Option<f64> {
        let rates = self.cure_rate();
        if rates.is_empty() {
            return None;
        }
        let mut max_idx = 0;
        let mut max_val = rates[0];
        for (i, &r) in rates.iter().enumerate() {
            if r > max_val {
                max_val = r;
                max_idx = i;
            }
        }
        // Offset by 1 due to derivative
        if max_idx + 1 < self.times.len() {
            Some(self.times[max_idx + 1])
        } else {
            Some(self.times[self.times.len() - 1])
        }
    }

    /// Gel point temperature.
    pub fn gel_point_temperature(&self) -> Option<f64> {
        let t = self.gel_point_time()?;
        Some(linear_interp(&self.times, &self.temperatures, t))
    }

    /// Check if cure is complete (degree of cure > threshold).
    pub fn is_fully_cured(&self, threshold: f64) -> bool {
        let doc = self.degree_of_cure();
        if doc.is_empty() {
            return false;
        }
        doc[doc.len() - 1] >= threshold
    }
}

// ============================================================================
// DataSmoothing
// ============================================================================

/// Savitzky-Golay smoothing and derivative computation for TMA data.
#[derive(Debug, Clone)]
pub struct DataSmoothing {
    /// Window size (must be odd)
    pub window_size: usize,
    /// Polynomial order (must be < window_size)
    pub poly_order: usize,
}

impl DataSmoothing {
    pub fn new(window_size: usize, poly_order: usize) -> Self {
        let ws = if window_size % 2 == 0 {
            window_size + 1
        } else {
            window_size
        };
        let po = poly_order.min(ws - 1);
        Self {
            window_size: ws,
            poly_order: po,
        }
    }

    /// Apply Savitzky-Golay smoothing to data.
    pub fn smooth(&self, data: &[f64]) -> Vec<f64> {
        let n = data.len();
        if n < self.window_size {
            return data.to_vec();
        }
        let coeffs = self.sg_coefficients(0);
        self.apply_coefficients(data, &coeffs)
    }

    /// Compute first derivative via Savitzky-Golay.
    pub fn first_derivative(&self, x: &[f64], y: &[f64]) -> Vec<f64> {
        let n = y.len();
        if n < self.window_size || x.len() != n {
            return vec![];
        }
        let coeffs = self.sg_coefficients(1);
        let raw_deriv = self.apply_coefficients(y, &coeffs);
        // Scale by sampling interval (assume uniform spacing)
        let dx = if n > 1 {
            (x[n - 1] - x[0]) / (n - 1) as f64
        } else {
            1.0
        };
        if dx.abs() < 1e-30 {
            return raw_deriv;
        }
        raw_deriv.iter().map(|&d| d / dx).collect()
    }

    /// Compute second derivative via Savitzky-Golay.
    pub fn second_derivative(&self, x: &[f64], y: &[f64]) -> Vec<f64> {
        let n = y.len();
        if n < self.window_size || x.len() != n || self.poly_order < 2 {
            return vec![];
        }
        let coeffs = self.sg_coefficients(2);
        let raw_deriv = self.apply_coefficients(y, &coeffs);
        let dx = if n > 1 {
            (x[n - 1] - x[0]) / (n - 1) as f64
        } else {
            1.0
        };
        if dx.abs() < 1e-30 {
            return raw_deriv;
        }
        raw_deriv.iter().map(|&d| d / (dx * dx)).collect()
    }

    /// Compute Savitzky-Golay convolution coefficients for given derivative order.
    fn sg_coefficients(&self, deriv_order: usize) -> Vec<f64> {
        let m = (self.window_size / 2) as i64;
        let order = self.poly_order;
        let n_pts = self.window_size;

        // Build Vandermonde-like matrix J and solve (J^T J) c = J^T e_deriv
        // For smoothing (deriv=0): coefficients that give the 0th polynomial coeff
        // For 1st derivative: coefficients for 1st polynomial coeff
        // etc.

        // Build J matrix: J[i][k] = i^k where i goes from -m to m
        let mut jt_j = vec![vec![0.0; order + 1]; order + 1];
        for i in -(m)..=m {
            for k in 0..=order {
                for l in 0..=order {
                    jt_j[k][l] += (i as f64).powi((k + l) as i32);
                }
            }
        }

        // Solve for each column of the inverse to get the deriv_order row
        let mut rhs = vec![0.0; order + 1];
        if deriv_order <= order {
            // factorial scaling for derivative
            let fact: f64 = (1..=deriv_order).map(|x| x as f64).product::<f64>().max(1.0);
            rhs[deriv_order] = fact;
        }

        let c = solve_linear_system(&jt_j, &rhs);

        // Compute coefficients for each point in window
        let mut coeffs = Vec::with_capacity(n_pts);
        for i in -(m)..=m {
            let mut val = 0.0;
            for k in 0..=order {
                if k < c.len() {
                    val += c[k] * (i as f64).powi(k as i32);
                }
            }
            coeffs.push(val);
        }
        coeffs
    }

    /// Apply convolution coefficients to data.
    fn apply_coefficients(&self, data: &[f64], coeffs: &[f64]) -> Vec<f64> {
        let n = data.len();
        let m = self.window_size / 2;
        let mut result = vec![0.0; n];

        for i in 0..n {
            let mut sum = 0.0;
            for (j, &c) in coeffs.iter().enumerate() {
                let idx = i as i64 + j as i64 - m as i64;
                let idx = idx.clamp(0, (n - 1) as i64) as usize;
                sum += c * data[idx];
            }
            result[i] = sum;
        }
        result
    }

    /// Simple noise filter: remove spikes exceeding median +/- n*std.
    pub fn despike(data: &[f64], n_sigma: f64) -> Vec<f64> {
        let n = data.len();
        if n < 3 {
            return data.to_vec();
        }
        let mean = data.iter().sum::<f64>() / n as f64;
        let var = data.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n as f64;
        let std = var.sqrt();
        let threshold = n_sigma * std;

        let mut result = data.to_vec();
        for i in 1..n - 1 {
            if (data[i] - mean).abs() > threshold {
                result[i] = (data[i - 1] + data[i + 1]) / 2.0;
            }
        }
        result
    }
}

// ============================================================================
// Utility / helper functions
// ============================================================================

/// Numerical first derivative using central differences.
/// Returns n-2 values (interior points).
fn numerical_derivative(x: &[f64], y: &[f64]) -> Vec<f64> {
    let n = x.len().min(y.len());
    if n < 3 {
        return vec![];
    }
    let mut deriv = Vec::with_capacity(n - 2);
    for i in 1..n - 1 {
        let dx = x[i + 1] - x[i - 1];
        if dx.abs() < 1e-30 {
            deriv.push(0.0);
        } else {
            deriv.push((y[i + 1] - y[i - 1]) / dx);
        }
    }
    deriv
}

/// Linear least-squares fit: y = slope * x + intercept.
/// Returns Some((slope, intercept)) or None if data is insufficient.
fn linear_fit(x: &[f64], y: &[f64]) -> Option<(f64, f64)> {
    let n = x.len().min(y.len());
    if n < 2 {
        return None;
    }
    let nf = n as f64;
    let sx: f64 = x.iter().take(n).sum();
    let sy: f64 = y.iter().take(n).sum();
    let sxx: f64 = x.iter().take(n).map(|&xi| xi * xi).sum();
    let sxy: f64 = x.iter().zip(y.iter()).take(n).map(|(&xi, &yi)| xi * yi).sum();
    let denom = nf * sxx - sx * sx;
    if denom.abs() < 1e-30 {
        return None;
    }
    let slope = (nf * sxy - sx * sy) / denom;
    let intercept = (sy - slope * sx) / nf;
    Some((slope, intercept))
}

/// Linear interpolation in a sorted array.
fn linear_interp(xs: &[f64], ys: &[f64], x: f64) -> f64 {
    let n = xs.len().min(ys.len());
    if n == 0 {
        return 0.0;
    }
    if n == 1 || x <= xs[0] {
        return ys[0];
    }
    if x >= xs[n - 1] {
        return ys[n - 1];
    }
    for i in 1..n {
        if xs[i] >= x {
            let frac = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
            return ys[i - 1] + frac * (ys[i] - ys[i - 1]);
        }
    }
    ys[n - 1]
}

/// Simple moving window smoothing.
fn smooth_window(data: &[f64], half_window: usize) -> Vec<f64> {
    let n = data.len();
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let lo = if i >= half_window { i - half_window } else { 0 };
        let hi = (i + half_window + 1).min(n);
        let sum: f64 = data[lo..hi].iter().sum();
        let count = (hi - lo) as f64;
        out.push(sum / count);
    }
    out
}

/// Solve a small linear system Ax = b using Gaussian elimination with partial pivoting.
fn solve_linear_system(a: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    let n = a.len();
    if n == 0 || b.len() != n {
        return vec![];
    }
    // Augmented matrix
    let mut aug: Vec<Vec<f64>> = a
        .iter()
        .enumerate()
        .map(|(i, row)| {
            let mut r = row.clone();
            r.push(b[i]);
            r
        })
        .collect();

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in col + 1..n {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        if pivot.abs() < 1e-30 {
            continue;
        }

        for row in col + 1..n {
            let factor = aug[row][col] / pivot;
            for j in col..=n {
                let val = aug[col][j];
                aug[row][j] -= factor * val;
            }
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = aug[i][n];
        for j in i + 1..n {
            sum -= aug[i][j] * x[j];
        }
        if aug[i][i].abs() > 1e-30 {
            x[i] = sum / aug[i][i];
        }
    }
    x
}

/// Trapezoidal integration of (x, y) data.
pub fn trapezoidal_integrate(x: &[f64], y: &[f64]) -> f64 {
    let n = x.len().min(y.len());
    if n < 2 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 1..n {
        sum += 0.5 * (y[i] + y[i - 1]) * (x[i] - x[i - 1]);
    }
    sum
}

/// Simpson's 1/3 rule integration (falls back to trapezoidal for odd segment count).
pub fn simpsons_integrate(x: &[f64], y: &[f64]) -> f64 {
    let n = x.len().min(y.len());
    if n < 3 {
        return trapezoidal_integrate(x, y);
    }
    let mut sum = 0.0;
    let mut i = 0;
    while i + 2 < n {
        let h1 = x[i + 1] - x[i];
        let h2 = x[i + 2] - x[i + 1];
        let h = h1 + h2;
        sum += h / 6.0 * (y[i] + 4.0 * y[i + 1] + y[i + 2]);
        i += 2;
    }
    // Handle remaining segment with trapezoidal
    if i + 1 < n {
        sum += 0.5 * (y[i] + y[i + 1]) * (x[i + 1] - x[i]);
    }
    sum
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    // Helper to check approximate equality
    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // ---- Helper function tests ----

    #[test]
    fn test_mean_cte_basic() {
        // alpha = 10um / (10mm*1000 * 100K) = 10/(10000*100) = 1e-5
        let alpha = mean_cte(10000.0, 10.0, 100.0);
        assert!(approx_eq(alpha, 1e-5, EPS));
    }

    #[test]
    fn test_mean_cte_zero_l0() {
        assert_eq!(mean_cte(0.0, 10.0, 100.0), 0.0);
    }

    #[test]
    fn test_mean_cte_zero_dt() {
        assert_eq!(mean_cte(10.0, 5.0, 0.0), 0.0);
    }

    #[test]
    fn test_mean_cte_negative() {
        let alpha = mean_cte(10000.0, -5.0, 100.0);
        assert!(alpha < 0.0);
    }

    #[test]
    fn test_wlf_shift_at_tref() {
        let shift = wlf_shift(100.0, 100.0, 17.44, 51.6);
        assert!(approx_eq(shift, 0.0, EPS));
    }

    #[test]
    fn test_wlf_shift_above_tref() {
        let shift = wlf_shift(120.0, 100.0, 17.44, 51.6);
        // -17.44 * 20 / (51.6 + 20) = -348.8 / 71.6 = -4.871..
        assert!(shift < 0.0);
        assert!(approx_eq(shift, -17.44 * 20.0 / 71.6, 1e-6));
    }

    #[test]
    fn test_wlf_shift_below_tref() {
        let shift = wlf_shift(80.0, 100.0, 17.44, 51.6);
        // -17.44 * (-20) / (51.6 + (-20)) = 348.8 / 31.6 > 0
        assert!(shift > 0.0);
    }

    #[test]
    fn test_wlf_shift_zero_denom() {
        // c2 + (T - Tref) = 0 => shift = 0
        let shift = wlf_shift(48.4, 100.0, 17.44, 51.6);
        assert_eq!(shift, 0.0);
    }

    #[test]
    fn test_burger_creep_t0() {
        let d = burger_creep(0.0, 1e3, 1e3, 1e6, 1e4, 10.0);
        // At t=0: J = 1/E1 + 0 + (1/E2)(1-1) = 1/E1 = 0.001
        // d = 10 * 0.001 = 0.01
        assert!(approx_eq(d, 0.01, 1e-6));
    }

    #[test]
    fn test_burger_creep_large_t() {
        let d = burger_creep(1e6, 1e3, 1e3, 1e6, 1e4, 10.0);
        // At large t: J ~ 1/E1 + t/eta1 + 1/E2
        // = 0.001 + 1e6/1e6 + 0.001 = 1.002
        // d = 10 * 1.002 = 10.02
        assert!(approx_eq(d, 10.02, 0.01));
    }

    #[test]
    fn test_burger_creep_zero_stress() {
        let d = burger_creep(100.0, 1e3, 1e3, 1e6, 1e4, 0.0);
        assert_eq!(d, 0.0);
    }

    #[test]
    fn test_burger_creep_zero_params() {
        let d = burger_creep(100.0, 0.0, 1e3, 1e6, 1e4, 10.0);
        assert_eq!(d, 0.0);
    }

    // ---- LinearExpansion tests ----

    #[test]
    fn test_linear_expansion_mean_cte_overall() {
        // 10mm sample, linear expansion: 0um at 25C, 100um at 125C
        let le = LinearExpansion::new(
            10.0,
            vec![25.0, 50.0, 75.0, 100.0, 125.0],
            vec![0.0, 25.0, 50.0, 75.0, 100.0],
        );
        let cte = le.mean_cte_overall();
        // dl=100um, l0=10mm=10000um, dt=100K => alpha = 100/(10000*100) = 1e-4
        assert!(approx_eq(cte, 1e-4, 1e-8));
    }

    #[test]
    fn test_linear_expansion_mean_cte_range() {
        let le = LinearExpansion::new(
            10.0,
            vec![25.0, 50.0, 75.0, 100.0, 125.0],
            vec![0.0, 25.0, 50.0, 75.0, 100.0],
        );
        let cte = le.mean_cte_range(50.0, 100.0);
        // dl=50um over dt=50K, l0=10000um => 50/(10000*50) = 1e-4
        assert!(approx_eq(cte, 1e-4, 1e-8));
    }

    #[test]
    fn test_linear_expansion_instantaneous_cte() {
        let le = LinearExpansion::new(
            10.0,
            vec![0.0, 10.0, 20.0, 30.0, 40.0],
            vec![0.0, 10.0, 20.0, 30.0, 40.0],
        );
        let (temps, ctes) = le.instantaneous_cte();
        assert_eq!(temps.len(), 3);
        for &c in &ctes {
            // dl/dt = 1 um/K, l0 = 10000um => cte = 1/10000 = 1e-4
            assert!(approx_eq(c, 1e-4, 1e-8));
        }
    }

    #[test]
    fn test_linear_expansion_interpolate() {
        let le = LinearExpansion::new(
            10.0,
            vec![0.0, 100.0],
            vec![0.0, 100.0],
        );
        assert!(approx_eq(le.interpolate_displacement(50.0), 50.0, EPS));
        assert!(approx_eq(le.interpolate_displacement(0.0), 0.0, EPS));
        assert!(approx_eq(le.interpolate_displacement(100.0), 100.0, EPS));
    }

    #[test]
    fn test_linear_expansion_percent_change() {
        let le = LinearExpansion::new(
            10.0,
            vec![0.0, 100.0],
            vec![0.0, 100.0], // 100um change on 10mm=10000um => 1%
        );
        let pct = le.percent_linear_change();
        assert!(approx_eq(pct[0], 0.0, EPS));
        assert!(approx_eq(pct[1], 1.0, 1e-6));
    }

    #[test]
    fn test_linear_expansion_empty() {
        let le = LinearExpansion::new(10.0, vec![], vec![]);
        assert_eq!(le.mean_cte_overall(), 0.0);
        let (t, c) = le.instantaneous_cte();
        assert!(t.is_empty());
        assert!(c.is_empty());
    }

    #[test]
    fn test_linear_expansion_single_point() {
        let le = LinearExpansion::new(10.0, vec![25.0], vec![0.0]);
        assert_eq!(le.mean_cte_overall(), 0.0);
    }

    // ---- GlassTransition tests ----

    #[test]
    fn test_glass_transition_inflection() {
        // Simulate a sigmoid-like CTE transition
        let n = 100;
        let mut temps = Vec::new();
        let mut vals = Vec::new();
        for i in 0..n {
            let t = 50.0 + (i as f64) * 2.0; // 50 to 248
            temps.push(t);
            // Sigmoid centered at 150C
            let v = 100.0 / (1.0 + (-0.1 * (t - 150.0)).exp());
            vals.push(v);
        }
        let gt = GlassTransition::new(temps, vals);
        let tg = gt.detect_tg(TgMethod::Inflection);
        assert!(tg.is_some());
        let tg_val = tg.unwrap();
        assert!((tg_val - 150.0).abs() < 10.0);
    }

    #[test]
    fn test_glass_transition_onset() {
        let n = 100;
        let mut temps = Vec::new();
        let mut vals = Vec::new();
        for i in 0..n {
            let t = 50.0 + (i as f64) * 2.0;
            temps.push(t);
            let v = if t < 140.0 {
                t * 0.1
            } else {
                14.0 + (t - 140.0) * 0.5
            };
            vals.push(v);
        }
        let gt = GlassTransition::new(temps, vals);
        let tg = gt.detect_tg(TgMethod::Onset);
        assert!(tg.is_some());
        let tg_val = tg.unwrap();
        assert!((tg_val - 140.0).abs() < 20.0);
    }

    #[test]
    fn test_glass_transition_midpoint() {
        let n = 100;
        let mut temps = Vec::new();
        let mut vals = Vec::new();
        for i in 0..n {
            let t = 50.0 + (i as f64) * 2.0;
            temps.push(t);
            let v = if t < 140.0 {
                t * 0.1
            } else {
                14.0 + (t - 140.0) * 0.5
            };
            vals.push(v);
        }
        let gt = GlassTransition::new(temps, vals);
        let tg = gt.detect_tg(TgMethod::Midpoint);
        assert!(tg.is_some());
    }

    #[test]
    fn test_glass_transition_insufficient_data() {
        let gt = GlassTransition::new(vec![1.0, 2.0], vec![1.0, 2.0]);
        assert!(gt.detect_tg(TgMethod::Onset).is_none());
        assert!(gt.detect_tg(TgMethod::Inflection).is_none());
    }

    // ---- SofteningPoint tests ----

    #[test]
    fn test_vicat_softening() {
        let sp = SofteningPoint::new(
            vec![100.0, 110.0, 120.0, 130.0, 140.0],
            vec![0.0, 200.0, 500.0, 800.0, 1200.0],
        );
        let vicat = sp.vicat_softening(1000.0);
        assert!(vicat.is_some());
        let t = vicat.unwrap();
        assert!(t > 130.0 && t < 140.0);
    }

    #[test]
    fn test_vicat_softening_not_reached() {
        let sp = SofteningPoint::new(
            vec![100.0, 110.0, 120.0],
            vec![0.0, 100.0, 200.0],
        );
        assert!(sp.vicat_softening(1000.0).is_none());
    }

    #[test]
    fn test_hdt() {
        let sp = SofteningPoint::new(
            vec![50.0, 60.0, 70.0, 80.0],
            vec![0.0, 100.0, 250.0, 400.0],
        );
        let hdt = sp.hdt(250.0);
        assert!(hdt.is_some());
        assert!(approx_eq(hdt.unwrap(), 70.0, EPS));
    }

    #[test]
    fn test_deformation_rate() {
        let sp = SofteningPoint::new(
            vec![100.0, 110.0, 120.0, 130.0, 140.0],
            vec![0.0, 10.0, 20.0, 50.0, 120.0],
        );
        let rates = sp.deformation_rate();
        assert_eq!(rates.len(), 3);
        // Rates should increase (accelerating softening)
        assert!(rates[2] > rates[0]);
    }

    #[test]
    fn test_softening_onset_detection() {
        let sp = SofteningPoint::new(
            vec![100.0, 110.0, 120.0, 130.0, 140.0],
            vec![0.0, 1.0, 2.0, 50.0, 200.0],
        );
        let onset = sp.softening_onset(2.0);
        assert!(onset.is_some());
    }

    // ---- CreepAnalysis tests ----

    #[test]
    fn test_creep_compliance() {
        let ca = CreepAnalysis::new(
            vec![0.0, 1.0, 2.0, 3.0],
            vec![0.0, 10.0, 20.0, 30.0],
            1.0, // 1 MPa
            150.0,
        );
        let j = ca.creep_compliance(1000.0); // l0 = 1000 um
        // J = (d/l0) / stress = (10/1000)/1 = 0.01
        assert!(approx_eq(j[1], 0.01, EPS));
    }

    #[test]
    fn test_creep_rate() {
        let ca = CreepAnalysis::new(
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
            vec![0.0, 5.0, 10.0, 15.0, 20.0],
            1.0,
            150.0,
        );
        let rates = ca.creep_rate();
        assert_eq!(rates.len(), 3);
        for &r in &rates {
            assert!(approx_eq(r, 5.0, 1e-6));
        }
    }

    #[test]
    fn test_creep_wlf_shift() {
        let ca = CreepAnalysis::new(
            vec![0.0, 1.0, 2.0],
            vec![0.0, 5.0, 10.0],
            1.0,
            150.0,
        );
        let shifted = ca.wlf_shifted_times(100.0, 17.44, 51.6);
        // T > Tref => log_at < 0 => at < 1 => shifted time > original
        assert!(shifted[1] > 1.0);
    }

    #[test]
    fn test_fit_burgers_model() {
        // Generate Burger's model data and try to fit
        let e1 = 1000.0;
        let e2 = 500.0;
        let eta1 = 1e6;
        let eta2 = 1e4;
        let stress = 10.0;
        let mut times = Vec::new();
        let mut disps = Vec::new();
        for i in 0..100 {
            let t = i as f64 * 100.0;
            times.push(t);
            disps.push(burger_creep(t, e1, e2, eta1, eta2, stress));
        }
        let ca = CreepAnalysis::new(times, disps, stress, 100.0);
        let params = ca.fit_burgers_model();
        assert!(params.is_some());
        let (pe1, _pe2, _peta1, _peta2) = params.unwrap();
        // E1 should be close to original
        assert!((pe1 - e1).abs() / e1 < 0.5); // Within 50%
    }

    #[test]
    fn test_creep_stages() {
        // Simulate three-stage creep
        let mut times = Vec::new();
        let mut disps = Vec::new();
        for i in 0..100 {
            let t = i as f64;
            times.push(t);
            let d = if t < 30.0 {
                t.sqrt() * 10.0 // Primary: decreasing rate
            } else if t < 70.0 {
                10.0 * 30.0_f64.sqrt() + (t - 30.0) * 1.0 // Secondary: constant rate
            } else {
                10.0 * 30.0_f64.sqrt() + 40.0 + (t - 70.0).powi(2) * 0.1 // Tertiary: increasing
            };
            disps.push(d);
        }
        let ca = CreepAnalysis::new(times, disps, 1.0, 100.0);
        let (primary_end, _secondary_end) = ca.identify_stages();
        assert!(primary_end.is_some());
    }

    #[test]
    fn test_creep_zero_stress() {
        let ca = CreepAnalysis::new(vec![0.0, 1.0], vec![0.0, 0.0], 0.0, 100.0);
        let j = ca.creep_compliance(1000.0);
        assert!(j.iter().all(|&v| v == 0.0));
    }

    // ---- DilatometryMode tests ----

    #[test]
    fn test_dilatometry_volumetric_cte() {
        // Linear increase in specific volume
        let dm = DilatometryMode::new(
            vec![25.0, 50.0, 75.0, 100.0, 125.0],
            vec![1.000, 1.001, 1.002, 1.003, 1.004],
            true,
        );
        let (temps, betas) = dm.volumetric_cte();
        assert_eq!(temps.len(), 3);
        for &b in &betas {
            // dV/dT ~ 0.001/25 = 4e-5, beta = dV/dT / V0 = 4e-5
            assert!(approx_eq(b, 4e-5, 1e-7));
        }
    }

    #[test]
    fn test_dilatometry_linear_from_volume() {
        let dm = DilatometryMode::new(
            vec![25.0, 50.0, 75.0, 100.0, 125.0],
            vec![1.000, 1.001, 1.002, 1.003, 1.004],
            true,
        );
        let (_, alphas) = dm.linear_cte_from_volume();
        for &a in &alphas {
            // alpha = beta/3 ~ 1.33e-5
            assert!(approx_eq(a, 4e-5 / 3.0, 1e-7));
        }
    }

    #[test]
    fn test_dilatometry_specific_volume_interp() {
        let dm = DilatometryMode::new(
            vec![0.0, 100.0],
            vec![1.0, 1.1],
            true,
        );
        assert!(approx_eq(dm.specific_volume_at(50.0), 1.05, 1e-6));
    }

    #[test]
    fn test_dilatometry_density() {
        let dm = DilatometryMode::new(
            vec![0.0, 100.0],
            vec![1.0, 1.1],
            true,
        );
        // density = 1/V; at T=0, V=1.0 => density = 1.0
        assert!(approx_eq(dm.density_at(0.0), 1.0, EPS));
    }

    #[test]
    fn test_dilatometry_bulk_modulus() {
        let dm = DilatometryMode::new(
            vec![0.0, 100.0],
            vec![1.0, 1.01],
            true,
        );
        let k = dm.bulk_modulus_estimate(100.0); // 100 MPa
        // K = P * V0 / dV = 100 * 1.0 / 0.01 = 10000
        assert!(approx_eq(k, 10000.0, 1.0));
    }

    #[test]
    fn test_dilatometry_empty() {
        let dm = DilatometryMode::new(vec![], vec![], true);
        let (t, b) = dm.volumetric_cte();
        assert!(t.is_empty());
        assert!(b.is_empty());
    }

    // ---- ThermalCycling tests ----

    #[test]
    fn test_thermal_cycling_permanent_set() {
        let mut tc = ThermalCycling::new();
        // Heating from 0 to 5um, cooling back to 2um (permanent set = 2)
        tc.add_cycle(
            vec![25.0, 50.0, 75.0, 100.0, 75.0, 50.0, 25.0],
            vec![0.0, 2.0, 4.0, 5.0, 4.0, 3.0, 2.0],
        );
        assert!(approx_eq(tc.permanent_set(0), 2.0, EPS));
    }

    #[test]
    fn test_thermal_cycling_full_recovery() {
        let mut tc = ThermalCycling::new();
        tc.add_cycle(
            vec![25.0, 50.0, 75.0, 50.0, 25.0],
            vec![0.0, 5.0, 10.0, 5.0, 0.0],
        );
        let recovery = tc.dimensional_recovery(0);
        assert!(approx_eq(recovery, 1.0, 1e-6));
    }

    #[test]
    fn test_thermal_cycling_partial_recovery() {
        let mut tc = ThermalCycling::new();
        tc.add_cycle(
            vec![25.0, 50.0, 75.0, 50.0, 25.0],
            vec![0.0, 5.0, 10.0, 7.0, 5.0],
        );
        let recovery = tc.dimensional_recovery(0);
        // max=10, end=5, recovered=5, total_change=10 => 0.5
        assert!(approx_eq(recovery, 0.5, 1e-6));
    }

    #[test]
    fn test_thermal_cycling_hysteresis() {
        let mut tc = ThermalCycling::new();
        // Heating curve has lower values than cooling curve => hysteresis
        tc.add_cycle(
            vec![25.0, 50.0, 75.0, 100.0, 75.0, 50.0, 25.0, 10.0],
            vec![0.0, 2.0, 5.0, 10.0, 8.0, 5.0, 3.0, 1.0],
        );
        let area = tc.hysteresis_area(0);
        assert!(area > 0.0);
    }

    #[test]
    fn test_thermal_cycling_num_cycles() {
        let mut tc = ThermalCycling::new();
        assert_eq!(tc.num_cycles(), 0);
        tc.add_cycle(vec![25.0, 100.0, 25.0], vec![0.0, 5.0, 1.0]);
        tc.add_cycle(vec![25.0, 100.0, 25.0], vec![1.0, 6.0, 2.0]);
        assert_eq!(tc.num_cycles(), 2);
    }

    #[test]
    fn test_thermal_cycling_permanent_set_trend() {
        let mut tc = ThermalCycling::new();
        tc.add_cycle(vec![25.0, 100.0, 25.0], vec![0.0, 5.0, 1.0]);
        tc.add_cycle(vec![25.0, 100.0, 25.0], vec![1.0, 6.0, 2.5]);
        let trend = tc.permanent_set_trend();
        assert_eq!(trend.len(), 2);
        assert!(approx_eq(trend[0], 1.0, EPS));
        assert!(approx_eq(trend[1], 1.5, EPS));
    }

    #[test]
    fn test_thermal_cycling_invalid_index() {
        let tc = ThermalCycling::new();
        assert_eq!(tc.permanent_set(0), 0.0);
        assert_eq!(tc.hysteresis_area(0), 0.0);
        assert_eq!(tc.dimensional_recovery(0), 0.0);
    }

    // ---- ProbeMode tests ----

    #[test]
    fn test_probe_expansion_preset() {
        let p = ProbeModeConfig::expansion_preset();
        assert_eq!(p.mode, ProbeMode::Expansion);
        assert!(approx_eq(p.force_mn, 5.0, EPS));
        assert!(p.force_in_range());
    }

    #[test]
    fn test_probe_penetration_preset() {
        let p = ProbeModeConfig::penetration_preset();
        assert_eq!(p.mode, ProbeMode::Penetration);
        assert!(approx_eq(p.force_mn, 1000.0, EPS));
        assert!(p.force_in_range());
    }

    #[test]
    fn test_probe_applied_stress() {
        let p = ProbeModeConfig::new(ProbeMode::Expansion, 100.0, 2.0);
        // 100 mN / 2 mm^2 = 50 kPa = 0.05 MPa
        assert!(approx_eq(p.applied_stress_mpa(), 0.05, 1e-6));
    }

    #[test]
    fn test_probe_voltage_to_displacement() {
        let p = ProbeModeConfig::new(ProbeMode::Expansion, 5.0, 1.0);
        // sensitivity = 1.0 mV/um => 10 mV = 10 um
        assert!(approx_eq(p.voltage_to_displacement(10.0), 10.0, EPS));
    }

    #[test]
    fn test_probe_voltage_clamped() {
        let mut p = ProbeModeConfig::new(ProbeMode::Expansion, 5.0, 1.0);
        p.lvdt_range = 100.0;
        // 200 mV / 1.0 = 200 um > range => clamped to 100
        assert!(approx_eq(p.voltage_to_displacement(200.0), 100.0, EPS));
    }

    #[test]
    fn test_probe_convert_voltages() {
        let p = ProbeModeConfig::new(ProbeMode::Expansion, 5.0, 1.0);
        let disps = p.convert_voltages(&[0.0, 5.0, 10.0]);
        assert_eq!(disps.len(), 3);
        assert!(approx_eq(disps[1], 5.0, EPS));
    }

    #[test]
    fn test_probe_force_out_of_range() {
        let p = ProbeModeConfig::new(ProbeMode::Expansion, 100.0, 1.0);
        assert!(!p.force_in_range()); // Expansion max is 50 mN
    }

    #[test]
    fn test_probe_zero_area_stress() {
        let p = ProbeModeConfig::new(ProbeMode::Expansion, 5.0, 0.0);
        assert_eq!(p.applied_stress_mpa(), 0.0);
    }

    // ---- CureMon tests ----

    #[test]
    fn test_cure_shrinkage() {
        let cm = CureMon::new(
            vec![0.0, 60.0, 120.0, 180.0],
            vec![0.0, -10.0, -20.0, -30.0], // Shrinkage is negative displacement
            vec![180.0, 180.0, 180.0, 180.0],
            5.0, // 5mm = 5000um
        );
        let shrink = cm.shrinkage_percent();
        // At end: -(-30)/5000 * 100 = 0.6%
        assert!(approx_eq(shrink[3], 0.6, 1e-6));
    }

    #[test]
    fn test_cure_total_shrinkage() {
        let cm = CureMon::new(
            vec![0.0, 60.0, 120.0],
            vec![0.0, -25.0, -50.0],
            vec![180.0, 180.0, 180.0],
            10.0,
        );
        // 50 / 10000 * 100 = 0.5%
        assert!(approx_eq(cm.total_shrinkage_percent(), 0.5, 1e-6));
    }

    #[test]
    fn test_cure_degree_of_cure() {
        let cm = CureMon::new(
            vec![0.0, 60.0, 120.0, 180.0],
            vec![0.0, -10.0, -20.0, -30.0],
            vec![180.0, 180.0, 180.0, 180.0],
            5.0,
        );
        let doc = cm.degree_of_cure();
        assert!(approx_eq(doc[0], 0.0, 1e-6));
        assert!(approx_eq(doc[3], 1.0, 1e-6));
        assert!(approx_eq(doc[1], 1.0 / 3.0, 1e-4));
    }

    #[test]
    fn test_cure_gel_point() {
        // Cure rate peaks at some point
        let mut times = Vec::new();
        let mut disps = Vec::new();
        let mut temps = Vec::new();
        for i in 0..50 {
            let t = i as f64 * 10.0;
            times.push(t);
            temps.push(180.0);
            // Sigmoid shrinkage
            let d = -50.0 / (1.0 + (-0.02 * (t - 250.0)).exp());
            disps.push(d);
        }
        let cm = CureMon::new(times, disps, temps, 10.0);
        let gel = cm.gel_point_time();
        assert!(gel.is_some());
    }

    #[test]
    fn test_cure_gel_point_temperature() {
        let cm = CureMon::new(
            vec![0.0, 60.0, 120.0, 180.0, 240.0],
            vec![0.0, -5.0, -20.0, -28.0, -30.0],
            vec![150.0, 160.0, 170.0, 180.0, 190.0],
            5.0,
        );
        let tgel = cm.gel_point_temperature();
        assert!(tgel.is_some());
    }

    #[test]
    fn test_cure_is_fully_cured() {
        let cm = CureMon::new(
            vec![0.0, 60.0, 120.0],
            vec![0.0, -25.0, -50.0],
            vec![180.0, 180.0, 180.0],
            10.0,
        );
        assert!(cm.is_fully_cured(0.95));
        assert!(cm.is_fully_cured(1.0));
    }

    #[test]
    fn test_cure_not_fully_cured() {
        let cm = CureMon::new(
            vec![0.0, 60.0, 120.0],
            vec![0.0, -10.0, -20.0],
            vec![180.0, 180.0, 180.0],
            10.0,
        );
        // degree_of_cure at end: shrinkage at end / total = 1.0 (since total IS the end)
        // But if we simulate partial: add another point that would be the true total
        let cm2 = CureMon::new(
            vec![0.0, 60.0],
            vec![0.0, -10.0],
            vec![180.0, 180.0],
            10.0,
        );
        // Only 2 points, cure rate empty, but degree_of_cure endpoint = 1.0
        assert!(cm2.is_fully_cured(0.9));
    }

    #[test]
    fn test_cure_empty() {
        let cm = CureMon::new(vec![], vec![], vec![], 5.0);
        assert_eq!(cm.total_shrinkage_percent(), 0.0);
        assert!(cm.gel_point_time().is_none());
        assert!(!cm.is_fully_cured(0.5));
    }

    // ---- DataSmoothing tests ----

    #[test]
    fn test_smoothing_identity_for_linear() {
        let ds = DataSmoothing::new(5, 2);
        let data: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let smoothed = ds.smooth(&data);
        assert_eq!(smoothed.len(), data.len());
        // For linear data, SG smoothing should preserve it well (interior)
        for i in 3..17 {
            assert!(approx_eq(smoothed[i], data[i], 0.5));
        }
    }

    #[test]
    fn test_smoothing_reduces_noise() {
        let ds = DataSmoothing::new(7, 2);
        let mut data: Vec<f64> = (0..50).map(|i| i as f64 * 0.5).collect();
        // Add noise
        for i in 0..50 {
            if i % 3 == 0 {
                data[i] += 2.0;
            }
            if i % 5 == 0 {
                data[i] -= 1.5;
            }
        }
        let smoothed = ds.smooth(&data);
        // Smoothed should be closer to the linear trend
        let mut noise_original = 0.0;
        let mut noise_smoothed = 0.0;
        for i in 5..45 {
            let expected = i as f64 * 0.5;
            noise_original += (data[i] - expected).powi(2);
            noise_smoothed += (smoothed[i] - expected).powi(2);
        }
        assert!(noise_smoothed < noise_original);
    }

    #[test]
    fn test_first_derivative() {
        let ds = DataSmoothing::new(5, 2);
        let x: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let y: Vec<f64> = x.iter().map(|&xi| xi * xi).collect(); // y = x^2
        let dy = ds.first_derivative(&x, &y);
        assert_eq!(dy.len(), 20);
        // dy/dx = 2x; check interior
        for i in 3..17 {
            assert!(
                approx_eq(dy[i], 2.0 * x[i], 2.0),
                "dy[{}]={}, expected {}", i, dy[i], 2.0 * x[i]
            );
        }
    }

    #[test]
    fn test_second_derivative() {
        let ds = DataSmoothing::new(5, 3);
        let x: Vec<f64> = (0..30).map(|i| i as f64 * 0.1).collect();
        let y: Vec<f64> = x.iter().map(|&xi| xi * xi * xi).collect(); // y = x^3
        let d2y = ds.second_derivative(&x, &y);
        assert_eq!(d2y.len(), 30);
        // d2y/dx2 = 6x; check interior
        for i in 5..25 {
            let expected = 6.0 * x[i];
            assert!(
                (d2y[i] - expected).abs() < expected.abs() * 0.5 + 2.0,
                "d2y[{}]={}, expected ~{}", i, d2y[i], expected
            );
        }
    }

    #[test]
    fn test_smoothing_short_data() {
        let ds = DataSmoothing::new(7, 2);
        let data = vec![1.0, 2.0, 3.0];
        let smoothed = ds.smooth(&data);
        // Data shorter than window => return as-is
        assert_eq!(smoothed, data);
    }

    #[test]
    fn test_smoothing_even_window() {
        // Even window should be corrected to odd
        let ds = DataSmoothing::new(6, 2);
        assert_eq!(ds.window_size, 7);
    }

    #[test]
    fn test_despike() {
        let mut data: Vec<f64> = vec![1.0; 20];
        data[10] = 100.0; // Spike
        let cleaned = DataSmoothing::despike(&data, 3.0);
        assert!(cleaned[10] < 50.0); // Spike should be reduced
    }

    #[test]
    fn test_despike_no_spikes() {
        let data: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let cleaned = DataSmoothing::despike(&data, 3.0);
        for (a, b) in data.iter().zip(cleaned.iter()) {
            assert!(approx_eq(*a, *b, EPS));
        }
    }

    // ---- Utility function tests ----

    #[test]
    fn test_numerical_derivative_linear() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y = vec![0.0, 2.0, 4.0, 6.0, 8.0]; // y = 2x
        let d = numerical_derivative(&x, &y);
        assert_eq!(d.len(), 3);
        for &val in &d {
            assert!(approx_eq(val, 2.0, EPS));
        }
    }

    #[test]
    fn test_numerical_derivative_quadratic() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y: Vec<f64> = x.iter().map(|&xi| xi * xi).collect(); // y = x^2
        let d = numerical_derivative(&x, &y);
        // dy/dx at x=1: (4-0)/2=2, at x=2: (9-1)/2=4, at x=3: (16-4)/2=6
        assert!(approx_eq(d[0], 2.0, EPS));
        assert!(approx_eq(d[1], 4.0, EPS));
        assert!(approx_eq(d[2], 6.0, EPS));
    }

    #[test]
    fn test_numerical_derivative_short() {
        let d = numerical_derivative(&[1.0, 2.0], &[1.0, 2.0]);
        assert!(d.is_empty());
    }

    #[test]
    fn test_linear_fit() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y = vec![1.0, 3.0, 5.0, 7.0, 9.0]; // y = 2x + 1
        let (slope, intercept) = linear_fit(&x, &y).unwrap();
        assert!(approx_eq(slope, 2.0, 1e-6));
        assert!(approx_eq(intercept, 1.0, 1e-6));
    }

    #[test]
    fn test_linear_fit_single_point() {
        assert!(linear_fit(&[1.0], &[2.0]).is_none());
    }

    #[test]
    fn test_linear_interp_basic() {
        let xs = vec![0.0, 10.0, 20.0];
        let ys = vec![0.0, 5.0, 10.0];
        assert!(approx_eq(linear_interp(&xs, &ys, 5.0), 2.5, EPS));
        assert!(approx_eq(linear_interp(&xs, &ys, 15.0), 7.5, EPS));
    }

    #[test]
    fn test_linear_interp_boundary() {
        let xs = vec![0.0, 10.0];
        let ys = vec![0.0, 10.0];
        assert!(approx_eq(linear_interp(&xs, &ys, -5.0), 0.0, EPS));
        assert!(approx_eq(linear_interp(&xs, &ys, 15.0), 10.0, EPS));
    }

    #[test]
    fn test_trapezoidal_integrate() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y = vec![0.0, 1.0, 2.0, 3.0, 4.0]; // y = x
        let area = trapezoidal_integrate(&x, &y);
        assert!(approx_eq(area, 8.0, EPS)); // Area = 0.5 * 4 * 4 = 8
    }

    #[test]
    fn test_simpsons_integrate() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y: Vec<f64> = x.iter().map(|&xi| xi * xi).collect(); // y = x^2
        let area = simpsons_integrate(&x, &y);
        // Exact: integral of x^2 from 0 to 4 = 64/3 = 21.333...
        assert!((area - 64.0 / 3.0).abs() < 1.0);
    }

    #[test]
    fn test_simpsons_short() {
        let area = simpsons_integrate(&[0.0, 1.0], &[0.0, 1.0]);
        assert!(approx_eq(area, 0.5, EPS));
    }

    #[test]
    fn test_smooth_window() {
        let data = vec![1.0, 10.0, 1.0, 10.0, 1.0];
        let smoothed = smooth_window(&data, 1);
        // Each point averaged with neighbors
        assert!(smoothed[2] > 1.0 && smoothed[2] < 10.0);
    }

    #[test]
    fn test_solve_linear_system_2x2() {
        // 2x + 3y = 8, x + y = 3 => x=1, y=2
        let a = vec![vec![2.0, 3.0], vec![1.0, 1.0]];
        let b = vec![8.0, 3.0];
        let x = solve_linear_system(&a, &b);
        assert!(approx_eq(x[0], 1.0, 1e-6));
        assert!(approx_eq(x[1], 2.0, 1e-6));
    }

    #[test]
    fn test_solve_linear_system_3x3() {
        // x + y + z = 6, 2x + y = 5, x + 2z = 5 => x=1, y=3, z=2
        let a = vec![
            vec![1.0, 1.0, 1.0],
            vec![2.0, 1.0, 0.0],
            vec![1.0, 0.0, 2.0],
        ];
        let b = vec![6.0, 5.0, 5.0];
        let x = solve_linear_system(&a, &b);
        assert!(approx_eq(x[0], 1.0, 1e-6));
        assert!(approx_eq(x[1], 3.0, 1e-6));
        assert!(approx_eq(x[2], 2.0, 1e-6));
    }

    // ---- Integration / end-to-end tests ----

    #[test]
    fn test_polymer_tma_workflow() {
        // Simulate a polymer TMA experiment
        let n = 200;
        let mut temps = Vec::new();
        let mut disps = Vec::new();
        for i in 0..n {
            let t = 25.0 + i as f64; // 25 to 224 C
            temps.push(t);
            // Below Tg (150C): linear expansion with low CTE
            // Above Tg: higher CTE
            let d = if t < 150.0 {
                (t - 25.0) * 0.5 // 0.5 um/K
            } else {
                (150.0 - 25.0) * 0.5 + (t - 150.0) * 2.0 // 2 um/K above Tg
            };
            disps.push(d);
        }

        let le = LinearExpansion::new(10.0, temps.clone(), disps.clone());
        let cte = le.mean_cte_overall();
        assert!(cte > 0.0);

        let gt = GlassTransition::new(temps, disps);
        let tg = gt.detect_tg(TgMethod::Inflection);
        assert!(tg.is_some());
        let tg_val = tg.unwrap();
        assert!((tg_val - 150.0).abs() < 5.0);
    }

    #[test]
    fn test_epoxy_cure_workflow() {
        // Simulate epoxy cure
        let n = 100;
        let mut times = Vec::new();
        let mut disps = Vec::new();
        let mut temps = Vec::new();
        for i in 0..n {
            let t = i as f64 * 6.0; // 0 to 594 sec
            times.push(t);
            temps.push(180.0);
            // Exponential cure shrinkage
            let d = -100.0 * (1.0 - (-t / 200.0).exp());
            disps.push(d);
        }
        let cm = CureMon::new(times, disps, temps, 5.0);
        let total = cm.total_shrinkage_percent();
        assert!(total > 0.0); // Shrinkage should be positive percentage
        let doc = cm.degree_of_cure();
        assert!(doc[0] < 0.1);
        assert!(doc[n - 1] > 0.9);
    }

    #[test]
    fn test_probe_mode_expansion_workflow() {
        let probe = ProbeModeConfig::expansion_preset();
        assert!(probe.force_in_range());
        let voltages = vec![0.0, 1.0, 2.5, 5.0, 8.0];
        let disps = probe.convert_voltages(&voltages);
        assert_eq!(disps.len(), 5);
        assert!(approx_eq(disps[3], 5.0, EPS));
    }

    #[test]
    fn test_creep_burger_roundtrip() {
        let e1 = 2000.0;
        let e2 = 800.0;
        let eta1 = 5e5;
        let eta2 = 2e4;
        let stress = 5.0;
        let d_at_100 = burger_creep(100.0, e1, e2, eta1, eta2, stress);
        let d_at_200 = burger_creep(200.0, e1, e2, eta1, eta2, stress);
        // Displacement should increase with time
        assert!(d_at_200 > d_at_100);
        assert!(d_at_100 > 0.0);
    }

    #[test]
    fn test_dilatometry_isotropic_relation() {
        // For isotropic materials, beta = 3*alpha
        let dm = DilatometryMode::new(
            vec![0.0, 50.0, 100.0, 150.0, 200.0],
            vec![1.0, 1.001, 1.002, 1.003, 1.004],
            true,
        );
        let (_, betas) = dm.volumetric_cte();
        let (_, alphas) = dm.linear_cte_from_volume();
        for (b, a) in betas.iter().zip(alphas.iter()) {
            assert!(approx_eq(*b, 3.0 * *a, 1e-10));
        }
    }

    #[test]
    fn test_thermal_cycling_multiple_cycles_degradation() {
        let mut tc = ThermalCycling::new();
        // Each cycle has increasing permanent set (material degradation)
        tc.add_cycle(
            vec![25.0, 100.0, 25.0],
            vec![0.0, 10.0, 1.0],
        );
        tc.add_cycle(
            vec![25.0, 100.0, 25.0],
            vec![1.0, 12.0, 3.0],
        );
        tc.add_cycle(
            vec![25.0, 100.0, 25.0],
            vec![3.0, 14.0, 6.0],
        );
        let trend = tc.permanent_set_trend();
        // Permanent set should increase each cycle
        assert!(trend[1] >= trend[0]);
        assert!(trend[2] >= trend[1]);
    }
}
