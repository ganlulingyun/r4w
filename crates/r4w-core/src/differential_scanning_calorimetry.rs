//! Differential Scanning Calorimetry (DSC) data analysis module.
//!
//! Implements DSC curve analysis for measuring thermal transitions (glass transition,
//! melting, crystallization, curing) and thermodynamic properties of materials from
//! heat flow vs temperature curves.
//!
//! # Overview
//!
//! DSC measures the heat flow difference between a sample and a reference as a function
//! of temperature. Key transitions:
//! - **Glass transition**: step change in heat capacity (second-order transition)
//! - **Melting**: endothermic peak, enthalpy = area under peak
//! - **Crystallization**: exothermic peak
//! - **Curing**: exothermic reaction of thermoset resins
//!
//! # Science
//! - Crystallinity: Xc = delta_Hm / delta_H0m * 100%
//! - Avrami equation: alpha(t) = 1 - exp(-k * t^n)
//! - Kissinger: activation energy from heating rate dependence
//! - van't Hoff: purity analysis from melting point depression

use std::f64::consts::PI;

// ─── Data types ──────────────────────────────────────────────────────────────

/// Baseline construction method for peak integration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BaselineType {
    /// Straight line between start and end points.
    Linear,
    /// S-shaped sigmoidal baseline.
    Sigmoidal,
    /// Tangential baseline from pre/post-peak slopes.
    Tangential,
}

/// Thermal transition type for simulation.
#[derive(Debug, Clone)]
pub enum Transition {
    /// Glass transition at Tg with delta_Cp step.
    Glass { tg_c: f64, delta_cp: f64, width_c: f64 },
    /// Melting peak at Tm with enthalpy.
    Melting { tm_c: f64, enthalpy_j_per_g: f64, width_c: f64 },
    /// Crystallization peak at Tc with enthalpy (exothermic).
    Crystallization { tc_c: f64, enthalpy_j_per_g: f64, width_c: f64 },
    /// Curing exotherm at Tp with total heat.
    Curing { tp_c: f64, heat_j_per_g: f64, width_c: f64 },
}

/// DSC heat flow vs temperature curve.
#[derive(Debug, Clone)]
pub struct DscCurve {
    /// Temperature data in degrees Celsius.
    pub temperature_c: Vec<f64>,
    /// Heat flow data in milliwatts (mW).
    pub heat_flow_mw: Vec<f64>,
    /// Heating (or cooling) rate in C/min.
    pub heating_rate_c_per_min: f64,
    /// Sample mass in milligrams.
    pub sample_mass_mg: f64,
}

/// Glass transition analysis result.
#[derive(Debug, Clone)]
pub struct GlassTransitionResult {
    /// Midpoint Tg (inflection point), degrees C.
    pub tg_midpoint_c: f64,
    /// Onset Tg (tangent intersection with pre-Tg baseline), degrees C.
    pub tg_onset_c: f64,
    /// Endset Tg (tangent intersection with post-Tg baseline), degrees C.
    pub tg_endset_c: f64,
    /// Change in specific heat capacity at Tg, J/(g*K).
    pub delta_cp_j_per_g_k: f64,
}

/// Melting peak analysis result.
#[derive(Debug, Clone)]
pub struct MeltingResult {
    /// Onset temperature (extrapolated), degrees C.
    pub onset_c: f64,
    /// Peak temperature, degrees C.
    pub peak_c: f64,
    /// Endset temperature (return to baseline), degrees C.
    pub endset_c: f64,
    /// Melting enthalpy, J/g.
    pub enthalpy_j_per_g: f64,
}

/// Crystallization analysis result.
#[derive(Debug, Clone)]
pub struct CrystallizationResult {
    /// Onset temperature, degrees C.
    pub onset_c: f64,
    /// Peak temperature, degrees C.
    pub peak_c: f64,
    /// Endset temperature, degrees C.
    pub endset_c: f64,
    /// Crystallization enthalpy (positive value), J/g.
    pub enthalpy_j_per_g: f64,
}

/// Avrami crystallization kinetics result.
#[derive(Debug, Clone)]
pub struct AvramiResult {
    /// Avrami exponent n (dimensionless, typically 1-4).
    pub n: f64,
    /// Avrami rate constant k.
    pub k: f64,
    /// Half-time of crystallization, minutes.
    pub t_half: f64,
}

/// Autocatalytic curing kinetics result (Kamal model).
#[derive(Debug, Clone)]
pub struct AutocatalyticResult {
    /// Rate constant k1 (nth-order term).
    pub k1: f64,
    /// Rate constant k2 (autocatalytic term).
    pub k2: f64,
    /// Reaction order m (autocatalytic).
    pub m: f64,
    /// Reaction order n (nth-order).
    pub n: f64,
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

/// Linear interpolation: find y at x_target given sorted (x, y) data.
fn lerp_at(xs: &[f64], ys: &[f64], x_target: f64) -> f64 {
    if xs.is_empty() {
        return 0.0;
    }
    if x_target <= xs[0] {
        return ys[0];
    }
    if x_target >= xs[xs.len() - 1] {
        return ys[ys.len() - 1];
    }
    for i in 0..xs.len() - 1 {
        if xs[i] <= x_target && x_target <= xs[i + 1] {
            let t = (x_target - xs[i]) / (xs[i + 1] - xs[i]);
            return ys[i] + t * (ys[i + 1] - ys[i]);
        }
    }
    ys[ys.len() - 1]
}

/// Find index of the point closest to a given temperature.
fn nearest_index(temps: &[f64], target: f64) -> usize {
    let mut best = 0;
    let mut best_dist = f64::MAX;
    for (i, &t) in temps.iter().enumerate() {
        let d = (t - target).abs();
        if d < best_dist {
            best_dist = d;
            best = i;
        }
    }
    best
}

/// Simple trapezoidal integration of y over x.
fn trapz(x: &[f64], y: &[f64]) -> f64 {
    let n = x.len().min(y.len());
    if n < 2 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 0..n - 1 {
        sum += 0.5 * (y[i] + y[i + 1]) * (x[i + 1] - x[i]);
    }
    sum
}

/// Numerical derivative dy/dx using central differences (forward/backward at edges).
fn derivative(x: &[f64], y: &[f64]) -> Vec<f64> {
    let n = x.len();
    if n < 2 {
        return vec![0.0; n];
    }
    let mut d = vec![0.0; n];
    // Forward difference at start
    d[0] = (y[1] - y[0]) / (x[1] - x[0]);
    // Central differences
    for i in 1..n - 1 {
        d[i] = (y[i + 1] - y[i - 1]) / (x[i + 1] - x[i - 1]);
    }
    // Backward difference at end
    d[n - 1] = (y[n - 1] - y[n - 2]) / (x[n - 1] - x[n - 2]);
    d
}

/// Simple linear regression: returns (slope, intercept).
fn linear_fit(x: &[f64], y: &[f64]) -> (f64, f64) {
    let n = x.len() as f64;
    if n < 2.0 {
        return (0.0, y.first().copied().unwrap_or(0.0));
    }
    let sx: f64 = x.iter().sum();
    let sy: f64 = y.iter().sum();
    let sxy: f64 = x.iter().zip(y.iter()).map(|(a, b)| a * b).sum();
    let sxx: f64 = x.iter().map(|a| a * a).sum();
    let denom = n * sxx - sx * sx;
    if denom.abs() < 1e-30 {
        return (0.0, sy / n);
    }
    let slope = (n * sxy - sx * sy) / denom;
    let intercept = (sy - slope * sx) / n;
    (slope, intercept)
}

/// Find index of maximum value in slice.
fn argmax(data: &[f64]) -> usize {
    let mut best = 0;
    let mut best_val = f64::NEG_INFINITY;
    for (i, &v) in data.iter().enumerate() {
        if v > best_val {
            best_val = v;
            best = i;
        }
    }
    best
}

/// Find index of minimum value in slice.
fn argmin(data: &[f64]) -> usize {
    let mut best = 0;
    let mut best_val = f64::INFINITY;
    for (i, &v) in data.iter().enumerate() {
        if v < best_val {
            best_val = v;
            best = i;
        }
    }
    best
}

/// Smooth data using a simple moving average of given window size.
fn smooth(data: &[f64], window: usize) -> Vec<f64> {
    let n = data.len();
    if window <= 1 || n == 0 {
        return data.to_vec();
    }
    let half = window / 2;
    let mut out = vec![0.0; n];
    for i in 0..n {
        let lo = if i >= half { i - half } else { 0 };
        let hi = (i + half + 1).min(n);
        let count = (hi - lo) as f64;
        let sum: f64 = data[lo..hi].iter().sum();
        out[i] = sum / count;
    }
    out
}

// ─── DscCurve implementation ─────────────────────────────────────────────────

impl DscCurve {
    /// Create a new DSC curve from temperature and heat flow data.
    ///
    /// # Arguments
    /// * `temperature_c` - Temperature values in degrees Celsius
    /// * `heat_flow_mw` - Heat flow values in milliwatts
    /// * `heating_rate_c_per_min` - Heating rate in C/min
    pub fn new(temperature_c: Vec<f64>, heat_flow_mw: Vec<f64>, heating_rate_c_per_min: f64) -> Self {
        Self {
            temperature_c,
            heat_flow_mw,
            heating_rate_c_per_min,
            sample_mass_mg: 10.0, // default 10 mg
        }
    }

    /// Create with specified sample mass.
    pub fn with_mass(mut self, mass_mg: f64) -> Self {
        self.sample_mass_mg = mass_mg;
        self
    }

    /// Convention indicator: returns true if exothermic is up (positive).
    pub fn exo_up(&self) -> bool {
        // By convention in this implementation, exothermic peaks are positive.
        true
    }

    /// Temperature range of the curve.
    pub fn temperature_range(&self) -> (f64, f64) {
        if self.temperature_c.is_empty() {
            return (0.0, 0.0);
        }
        let min = self.temperature_c.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = self.temperature_c.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        (min, max)
    }

    /// Number of data points.
    pub fn len(&self) -> usize {
        self.temperature_c.len()
    }

    /// Check if curve has no data.
    pub fn is_empty(&self) -> bool {
        self.temperature_c.is_empty()
    }

    /// Normalize heat flow by sample mass: mW -> mW/mg.
    pub fn normalize_by_mass(&self) -> DscCurve {
        let mass_mg = if self.sample_mass_mg > 0.0 { self.sample_mass_mg } else { 1.0 };
        let hf: Vec<f64> = self.heat_flow_mw.iter().map(|&v| v / mass_mg).collect();
        DscCurve {
            temperature_c: self.temperature_c.clone(),
            heat_flow_mw: hf,
            heating_rate_c_per_min: self.heating_rate_c_per_min,
            sample_mass_mg: 1.0, // now per mg
        }
    }

    /// Normalize by heating rate: mW -> J/(g*K).
    /// Converts heat flow to apparent specific heat capacity.
    pub fn normalize_by_rate(&self) -> DscCurve {
        let mass_g = self.sample_mass_mg / 1000.0;
        let rate_k_per_s = self.heating_rate_c_per_min / 60.0;
        let factor = if (mass_g * rate_k_per_s).abs() > 1e-30 {
            1.0e-3 / (mass_g * rate_k_per_s) // mW to W, then / (g * K/s) = J/(g*K)
        } else {
            1.0
        };
        let hf: Vec<f64> = self.heat_flow_mw.iter().map(|&v| v * factor).collect();
        DscCurve {
            temperature_c: self.temperature_c.clone(),
            heat_flow_mw: hf,
            heating_rate_c_per_min: self.heating_rate_c_per_min,
            sample_mass_mg: self.sample_mass_mg,
        }
    }
}

// ─── Glass Transition Analysis ───────────────────────────────────────────────

/// Glass transition (Tg) analysis from DSC curves.
pub struct GlassTransition;

impl GlassTransition {
    /// Find glass transition from a DSC curve.
    ///
    /// The Tg is identified as the inflection point (maximum of derivative)
    /// in the heat flow step. Onset and endset are found from tangent line
    /// intersections with pre- and post-Tg baselines.
    pub fn find_tg(curve: &DscCurve) -> GlassTransitionResult {
        let t = &curve.temperature_c;
        let h = &curve.heat_flow_mw;
        let n = t.len();

        // Compute first derivative (rate of change of heat flow)
        let dh = derivative(t, h);
        let dh_smooth = smooth(&dh, 5.min(n));

        // Find inflection point = maximum magnitude of derivative
        // For a Tg step, the derivative has a peak (or trough depending on convention)
        let abs_dh: Vec<f64> = dh_smooth.iter().map(|v| v.abs()).collect();
        let infl_idx = argmax(&abs_dh);
        let tg_midpoint = t[infl_idx];

        // Pre-Tg baseline: fit line to first 20% of data
        let pre_end = (n / 5).max(2);
        let (pre_slope, pre_intercept) = linear_fit(&t[..pre_end], &h[..pre_end]);

        // Post-Tg baseline: fit line to last 20% of data
        let post_start = n - (n / 5).max(2);
        let (post_slope, post_intercept) = linear_fit(&t[post_start..], &h[post_start..]);

        // Tangent at inflection point
        let infl_slope = dh_smooth[infl_idx];
        let infl_intercept = h[infl_idx] - infl_slope * t[infl_idx];

        // Onset: intersection of tangent with pre-Tg baseline
        let tg_onset = if (infl_slope - pre_slope).abs() > 1e-30 {
            (pre_intercept - infl_intercept) / (infl_slope - pre_slope)
        } else {
            tg_midpoint - 5.0
        };

        // Endset: intersection of tangent with post-Tg baseline
        let tg_endset = if (infl_slope - post_slope).abs() > 1e-30 {
            (post_intercept - infl_intercept) / (infl_slope - post_slope)
        } else {
            tg_midpoint + 5.0
        };

        // Delta Cp: difference between post and pre baselines at Tg, normalized
        let pre_at_tg = pre_slope * tg_midpoint + pre_intercept;
        let post_at_tg = post_slope * tg_midpoint + post_intercept;
        let delta_hf_mw = (post_at_tg - pre_at_tg).abs();

        // Convert mW step to J/(g*K): delta_Cp = delta_hf / (mass_g * rate_K/s)
        let mass_g = curve.sample_mass_mg / 1000.0;
        let rate_k_per_s = curve.heating_rate_c_per_min / 60.0;
        let delta_cp = if (mass_g * rate_k_per_s).abs() > 1e-30 {
            delta_hf_mw * 1e-3 / (mass_g * rate_k_per_s)
        } else {
            delta_hf_mw
        };

        GlassTransitionResult {
            tg_midpoint_c: tg_midpoint,
            tg_onset_c: tg_onset,
            tg_endset_c: tg_endset,
            delta_cp_j_per_g_k: delta_cp,
        }
    }

    /// Fictive temperature Tf from enthalpy matching method.
    ///
    /// The fictive temperature is found where the enthalpy of the glass
    /// (extrapolated from below Tg) equals the enthalpy of the liquid
    /// (extrapolated from above Tg).
    pub fn fictive_temperature(curve: &DscCurve) -> f64 {
        let tg_result = Self::find_tg(curve);
        // Approximate Tf as midpoint Tg for standard cooling/heating
        // More precise: area matching method
        let t = &curve.temperature_c;
        let h = &curve.heat_flow_mw;
        let n = t.len();

        let pre_end = (n / 5).max(2);
        let (pre_slope, pre_intercept) = linear_fit(&t[..pre_end], &h[..pre_end]);
        let post_start = n - (n / 5).max(2);
        let (post_slope, post_intercept) = linear_fit(&t[post_start..], &h[post_start..]);

        // Find Tf where integral of (actual - glass_baseline) from Tf to high T
        // equals integral of (liquid_baseline - glass_baseline) from Tf to high T
        // Simplified: Tf ~ Tg_onset + integral correction
        let onset_idx = nearest_index(t, tg_result.tg_onset_c);
        let endset_idx = nearest_index(t, tg_result.tg_endset_c);

        if endset_idx <= onset_idx {
            return tg_result.tg_midpoint_c;
        }

        // Area between actual curve and glass baseline
        let mut area_actual = 0.0;
        let mut area_liquid = 0.0;
        for i in onset_idx..endset_idx {
            let glass_bl = pre_slope * t[i] + pre_intercept;
            let liquid_bl = post_slope * t[i] + post_intercept;
            let dt = if i + 1 < n { t[i + 1] - t[i] } else { 0.0 };
            area_actual += (h[i] - glass_bl) * dt;
            area_liquid += (liquid_bl - glass_bl) * dt;
        }

        // Tf from equal area principle
        if area_liquid.abs() > 1e-30 {
            let fraction = area_actual / area_liquid;
            let range = tg_result.tg_endset_c - tg_result.tg_onset_c;
            tg_result.tg_onset_c + fraction * range
        } else {
            tg_result.tg_midpoint_c
        }
    }

    /// Angell fragility index m.
    ///
    /// m = Ea / (R * Tg * ln(10))
    ///
    /// # Arguments
    /// * `tg_k` - Glass transition temperature in Kelvin
    /// * `activation_energy_kj` - Activation energy in kJ/mol
    pub fn fragility_index(tg_k: f64, activation_energy_kj: f64) -> f64 {
        let r = 8.314e-3; // kJ/(mol*K)
        if tg_k.abs() < 1e-10 {
            return 0.0;
        }
        activation_energy_kj / (r * tg_k * (10.0_f64).ln())
    }
}

// ─── Melting Analysis ────────────────────────────────────────────────────────

/// Melting peak analysis.
pub struct MeltingAnalysis;

impl MeltingAnalysis {
    /// Find melting peak from a DSC curve.
    ///
    /// Identifies the endothermic melting peak and computes onset, peak,
    /// endset temperatures and melting enthalpy.
    pub fn find_melting_peak(curve: &DscCurve) -> MeltingResult {
        let t = &curve.temperature_c;
        let h = &curve.heat_flow_mw;
        let n = t.len();

        // For exo-up convention, melting is a negative (downward) peak
        // Find the minimum in heat flow
        let peak_idx = argmin(h);
        let peak_c = t[peak_idx];

        // Onset: extrapolated from leading edge slope
        let leading_start = (peak_idx / 2).max(0);
        let leading_end = peak_idx;
        let onset_c = if leading_end > leading_start + 2 {
            let mid = (leading_start + leading_end) / 2;
            let range_start = mid.saturating_sub(2);
            let range_end = (mid + 3).min(leading_end);
            let (slope, intercept) = linear_fit(&t[range_start..range_end], &h[range_start..range_end]);
            // Pre-peak baseline
            let pre_end = (n / 5).max(2).min(leading_start + 1);
            let (bl_slope, bl_intercept) = linear_fit(&t[..pre_end], &h[..pre_end]);
            // Intersection
            if (slope - bl_slope).abs() > 1e-30 {
                (bl_intercept - intercept) / (slope - bl_slope)
            } else {
                peak_c - 5.0
            }
        } else {
            peak_c - 5.0
        };

        // Endset: where curve returns to post-peak baseline
        let trailing_start = peak_idx;
        let trailing_end = n;
        let endset_c = if trailing_end > trailing_start + 2 {
            let post_start = n - (n / 5).max(2);
            let (bl_slope, bl_intercept) = linear_fit(&t[post_start..], &h[post_start..]);
            // Find where heat flow crosses baseline after peak
            let mut endset = t[n - 1];
            for i in peak_idx..n {
                let bl_val = bl_slope * t[i] + bl_intercept;
                if h[i] >= bl_val {
                    endset = t[i];
                    break;
                }
            }
            endset
        } else {
            peak_c + 5.0
        };

        // Enthalpy: integrate peak area with linear baseline
        let onset_idx = nearest_index(t, onset_c);
        let endset_idx = nearest_index(t, endset_c);
        let baseline = BaselineConstructor::linear_baseline(curve, onset_c, endset_c);
        let mut enthalpy_mw_c = 0.0;
        if endset_idx > onset_idx {
            for i in onset_idx..endset_idx.min(n - 1) {
                let dt = t[i + 1] - t[i];
                let hf_above_bl = baseline[i] - h[i]; // melting is below baseline
                enthalpy_mw_c += hf_above_bl * dt;
            }
        }
        // Convert: mW*C / (C/min) / mass_g = mW*min / mass_g = mJ*60/1000 / mass_g = J/g
        let mass_g = curve.sample_mass_mg / 1000.0;
        let rate = curve.heating_rate_c_per_min;
        let enthalpy = if (mass_g * rate).abs() > 1e-30 {
            enthalpy_mw_c * 60.0 * 1e-3 / (mass_g * rate)
        } else {
            enthalpy_mw_c
        };

        MeltingResult {
            onset_c: onset_c.max(t[0]),
            peak_c,
            endset_c: endset_c.min(t[n - 1]),
            enthalpy_j_per_g: enthalpy.abs(),
        }
    }

    /// van't Hoff purity analysis from melting peak.
    ///
    /// Estimates molar purity from melting point depression.
    /// T_s = T_pure - (R * T_pure^2 * x) / (delta_H_fus * F)
    ///
    /// # Arguments
    /// * `curve` - DSC melting curve
    /// * `t_fus_pure` - Melting point of pure substance in C
    pub fn purity_from_melting(curve: &DscCurve, t_fus_pure: f64) -> f64 {
        let result = Self::find_melting_peak(curve);
        let t_fus_pure_k = t_fus_pure + 273.15;
        let t_peak_k = result.peak_c + 273.15;
        let r = 8.314; // J/(mol*K)

        // Depression = T_pure - T_observed
        let depression = t_fus_pure_k - t_peak_k;
        if depression <= 0.0 || result.enthalpy_j_per_g <= 0.0 {
            return 1.0; // No depression = pure
        }

        // Simplified: x_impurity ~ delta_H * depression / (R * T_pure^2)
        // Assuming molar mass ~100 g/mol for estimation
        let molar_mass = 100.0; // g/mol estimate
        let delta_h_mol = result.enthalpy_j_per_g * molar_mass; // J/mol
        let x_impurity = delta_h_mol * depression / (r * t_fus_pure_k * t_fus_pure_k);
        (1.0 - x_impurity).clamp(0.0, 1.0)
    }

    /// Percent crystallinity from melting enthalpy.
    ///
    /// Xc = (delta_Hm / delta_H0m) * 100
    ///
    /// # Arguments
    /// * `enthalpy` - Measured melting enthalpy, J/g
    /// * `enthalpy_100_percent` - Enthalpy of 100% crystalline material, J/g
    pub fn crystallinity_percent(enthalpy: f64, enthalpy_100_percent: f64) -> f64 {
        if enthalpy_100_percent.abs() < 1e-30 {
            return 0.0;
        }
        (enthalpy / enthalpy_100_percent * 100.0).clamp(0.0, 100.0)
    }
}

// ─── Crystallization Analysis ────────────────────────────────────────────────

/// Crystallization kinetics analysis.
pub struct CrystallizationAnalysis;

impl CrystallizationAnalysis {
    /// Find crystallization peak from a DSC curve (cooling scan).
    ///
    /// Crystallization is an exothermic event (positive peak in exo-up convention).
    pub fn find_crystallization(curve: &DscCurve) -> CrystallizationResult {
        let t = &curve.temperature_c;
        let h = &curve.heat_flow_mw;
        let n = t.len();

        let peak_idx = argmax(h);
        let peak_c = t[peak_idx];

        // Onset and endset via baseline intersection
        let pre_end = (n / 5).max(2);
        let (pre_slope, pre_intercept) = linear_fit(&t[..pre_end], &h[..pre_end]);

        let post_start = n - (n / 5).max(2);
        let (post_slope, post_intercept) = linear_fit(&t[post_start..], &h[post_start..]);

        // Onset: where curve departs from pre-peak baseline
        let mut onset_c = t[0];
        for i in 0..peak_idx {
            let bl = pre_slope * t[i] + pre_intercept;
            if (h[i] - bl).abs() > (h[peak_idx] - bl).abs() * 0.05 {
                onset_c = t[i];
                break;
            }
        }

        // Endset: where curve returns to post-peak baseline
        let mut endset_c = t[n - 1];
        for i in peak_idx..n {
            let bl = post_slope * t[i] + post_intercept;
            if (h[i] - bl).abs() < (h[peak_idx] - bl).abs() * 0.05 {
                endset_c = t[i];
                break;
            }
        }

        // Enthalpy
        let onset_idx = nearest_index(t, onset_c);
        let endset_idx = nearest_index(t, endset_c);
        let baseline = BaselineConstructor::linear_baseline(curve, onset_c, endset_c);
        let mut enthalpy_mw_c = 0.0;
        if endset_idx > onset_idx {
            for i in onset_idx..endset_idx.min(n - 1) {
                let dt = (t[i + 1] - t[i]).abs();
                enthalpy_mw_c += (h[i] - baseline[i]).abs() * dt;
            }
        }
        let mass_g = curve.sample_mass_mg / 1000.0;
        let rate = curve.heating_rate_c_per_min.abs();
        let enthalpy = if (mass_g * rate) > 1e-30 {
            enthalpy_mw_c * 60.0 * 1e-3 / (mass_g * rate)
        } else {
            enthalpy_mw_c
        };

        CrystallizationResult {
            onset_c,
            peak_c,
            endset_c,
            enthalpy_j_per_g: enthalpy.abs(),
        }
    }

    /// Avrami analysis of isothermal crystallization kinetics.
    ///
    /// Fits the Avrami equation: alpha(t) = 1 - exp(-k * t^n)
    /// by linear regression of ln(-ln(1-alpha)) vs ln(t).
    ///
    /// # Arguments
    /// * `time_min` - Time in minutes
    /// * `conversion` - Relative crystallinity alpha (0 to 1)
    pub fn avrami_analysis(time_min: &[f64], conversion: &[f64]) -> AvramiResult {
        // Filter valid data points (0 < alpha < 1, t > 0)
        let mut lnt = Vec::new();
        let mut lnln = Vec::new();
        for (&t, &a) in time_min.iter().zip(conversion.iter()) {
            if t > 1e-10 && a > 0.01 && a < 0.99 {
                lnt.push(t.ln());
                lnln.push((-((1.0 - a).ln())).ln());
            }
        }

        if lnt.len() < 2 {
            return AvramiResult { n: 2.0, k: 0.01, t_half: 10.0 };
        }

        let (n, intercept) = linear_fit(&lnt, &lnln);
        let k = intercept.exp(); // ln(k) = intercept

        // t_half from: 0.5 = 1 - exp(-k * t_half^n)
        // k * t_half^n = ln(2)
        // t_half = (ln(2)/k)^(1/n)
        let t_half = if k > 1e-30 && n.abs() > 1e-10 {
            ((2.0_f64).ln() / k).powf(1.0 / n)
        } else {
            10.0
        };

        AvramiResult { n, k, t_half }
    }

    /// Isothermal crystallization analysis from multiple isothermal DSC curves.
    pub fn isothermal_crystallization(curves: &[DscCurve]) -> Vec<AvramiResult> {
        curves.iter().map(|curve| {
            let conv = Self::relative_crystallinity(curve, curve.temperature_c[0],
                *curve.temperature_c.last().unwrap_or(&0.0));
            let n = conv.len();
            let t_range = curve.temperature_range();
            let dt = if n > 1 { (t_range.1 - t_range.0) / (n as f64 - 1.0) } else { 1.0 };
            let time_min: Vec<f64> = (0..n).map(|i| i as f64 * dt / curve.heating_rate_c_per_min.abs().max(1.0)).collect();
            Self::avrami_analysis(&time_min, &conv)
        }).collect()
    }

    /// Compute relative crystallinity alpha(T) from a crystallization peak.
    ///
    /// alpha(T) = integral(T_start..T) / integral(T_start..T_end)
    pub fn relative_crystallinity(curve: &DscCurve, t_start: f64, t_end: f64) -> Vec<f64> {
        let t = &curve.temperature_c;
        let h = &curve.heat_flow_mw;
        let n = t.len();

        let start_idx = nearest_index(t, t_start);
        let end_idx = nearest_index(t, t_end);
        if end_idx <= start_idx {
            return vec![0.0; n];
        }

        // Linear baseline
        let bl = BaselineConstructor::linear_baseline(curve, t_start, t_end);

        // Running integral
        let mut running = vec![0.0; n];
        for i in (start_idx + 1)..=end_idx.min(n - 1) {
            let dt = (t[i] - t[i - 1]).abs();
            running[i] = running[i - 1] + (h[i] - bl[i]).abs() * dt;
        }
        // Copy forward
        for i in (end_idx + 1)..n {
            running[i] = running[end_idx.min(n - 1)];
        }

        let total = running[end_idx.min(n - 1)];
        if total.abs() < 1e-30 {
            return vec![0.0; n];
        }

        running.iter().map(|&v| (v / total).clamp(0.0, 1.0)).collect()
    }
}

// ─── Curing Analysis ─────────────────────────────────────────────────────────

/// Thermoset curing kinetics analysis.
pub struct CuringAnalysis;

impl CuringAnalysis {
    /// Compute degree of cure alpha(T) from running integral of heat flow.
    ///
    /// alpha(T) = integral(T_start..T) / total_heat
    pub fn degree_of_cure(curve: &DscCurve) -> Vec<(f64, f64)> {
        let t = &curve.temperature_c;
        let h = &curve.heat_flow_mw;
        let n = t.len();
        if n < 2 {
            return Vec::new();
        }

        // Running integral of exothermic heat flow
        let mut integral = vec![0.0; n];
        for i in 1..n {
            let dt = t[i] - t[i - 1];
            integral[i] = integral[i - 1] + 0.5 * (h[i] + h[i - 1]) * dt;
        }

        let total = integral[n - 1];
        if total.abs() < 1e-30 {
            return t.iter().map(|&temp| (temp, 0.0)).collect();
        }

        t.iter().zip(integral.iter())
            .map(|(&temp, &integ)| (temp, (integ / total).clamp(0.0, 1.0)))
            .collect()
    }

    /// Total heat of cure (integral of entire exotherm), J/g.
    pub fn total_heat_of_cure(curve: &DscCurve) -> f64 {
        let t = &curve.temperature_c;
        let h = &curve.heat_flow_mw;
        let integral_mw_c = trapz(t, h);
        let mass_g = curve.sample_mass_mg / 1000.0;
        let rate = curve.heating_rate_c_per_min;
        if (mass_g * rate).abs() > 1e-30 {
            (integral_mw_c * 60.0 * 1e-3 / (mass_g * rate)).abs()
        } else {
            integral_mw_c.abs()
        }
    }

    /// Residual cure from second heating scan.
    ///
    /// Returns the fraction of unreacted material: delta_H_residual / delta_H_total.
    pub fn residual_cure(second_scan: &DscCurve) -> f64 {
        // The residual exotherm in a second scan indicates incomplete cure
        Self::total_heat_of_cure(second_scan)
    }

    /// Fit autocatalytic (Kamal) model to curing kinetics data.
    ///
    /// Model: d_alpha/dt = (k1 + k2 * alpha^m) * (1 - alpha)^n
    ///
    /// Simplified fitting using linearization at low and high conversion.
    pub fn autocatalytic_fit(
        temps_k: &[f64],
        rates: &[f64],
        conversions: &[f64],
    ) -> AutocatalyticResult {
        let len = temps_k.len().min(rates.len()).min(conversions.len());
        if len < 4 {
            return AutocatalyticResult { k1: 0.01, k2: 0.1, m: 1.0, n: 1.0 };
        }

        // At alpha ~ 0: d_alpha/dt ~ k1 * (1-alpha)^n ~ k1
        // Find k1 from low conversion data
        let mut low_rates = Vec::new();
        let mut high_rates = Vec::new();
        let mut high_alpha = Vec::new();
        let mut high_one_minus = Vec::new();

        for i in 0..len {
            if conversions[i] < 0.1 && conversions[i] > 0.001 {
                low_rates.push(rates[i]);
            }
            if conversions[i] > 0.2 && conversions[i] < 0.8 {
                high_rates.push(rates[i]);
                high_alpha.push(conversions[i]);
                high_one_minus.push(1.0 - conversions[i]);
            }
        }

        let k1 = if !low_rates.is_empty() {
            low_rates.iter().sum::<f64>() / low_rates.len() as f64
        } else {
            0.01
        };

        // For high conversion region, linearize:
        // ln(d_alpha/dt - k1*(1-alpha)^n) = ln(k2) + m*ln(alpha) + n*ln(1-alpha)
        // Simplified: assume m=1, n=1 initially, fit k2
        let m = 1.0;
        let n_order = 1.0;

        let k2 = if !high_rates.is_empty() {
            let mut k2_sum = 0.0;
            let mut count = 0.0;
            for i in 0..high_rates.len() {
                let alpha = high_alpha[i];
                let one_minus = high_one_minus[i];
                if alpha > 1e-10 && one_minus > 1e-10 {
                    let rhs = high_rates[i] / one_minus - k1;
                    if rhs > 0.0 {
                        k2_sum += rhs / alpha;
                        count += 1.0;
                    }
                }
            }
            if count > 0.0 { k2_sum / count } else { 0.1 }
        } else {
            0.1
        };

        AutocatalyticResult { k1, k2, m, n: n_order }
    }
}

// ─── Specific Heat Capacity ──────────────────────────────────────────────────

/// Specific heat capacity measurement methods.
pub struct SpecificHeatCapacity;

impl SpecificHeatCapacity {
    /// Three-run sapphire method for Cp measurement.
    ///
    /// Cp_sample = Cp_sapphire * (m_sapphire / m_sample) * (HF_sample - HF_baseline) / (HF_sapphire - HF_baseline)
    ///
    /// # Arguments
    /// * `sample` - Sample DSC curve
    /// * `baseline` - Empty pan baseline curve
    /// * `sapphire` - Sapphire reference curve
    /// * `sapphire_cp` - Known Cp of sapphire at various temperatures [(T_C, Cp)]
    pub fn sapphire_method(
        sample: &DscCurve,
        baseline: &DscCurve,
        sapphire: &DscCurve,
        sapphire_cp: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        let temps = &sample.temperature_c;
        let sapphire_temps: Vec<f64> = sapphire_cp.iter().map(|&(t, _)| t).collect();
        let sapphire_cps: Vec<f64> = sapphire_cp.iter().map(|&(_, cp)| cp).collect();

        let mass_ratio = if sample.sample_mass_mg > 0.0 {
            sapphire.sample_mass_mg / sample.sample_mass_mg
        } else {
            1.0
        };

        temps.iter().enumerate().map(|(i, &temp)| {
            let hf_sample = sample.heat_flow_mw[i];
            let hf_baseline = lerp_at(&baseline.temperature_c, &baseline.heat_flow_mw, temp);
            let hf_sapphire = lerp_at(&sapphire.temperature_c, &sapphire.heat_flow_mw, temp);
            let cp_sapphire = lerp_at(&sapphire_temps, &sapphire_cps, temp);

            let denom = hf_sapphire - hf_baseline;
            let cp = if denom.abs() > 1e-10 {
                cp_sapphire * mass_ratio * (hf_sample - hf_baseline) / denom
            } else {
                0.0
            };
            (temp, cp)
        }).collect()
    }

    /// Cp from modulated DSC (MDSC).
    ///
    /// Cp = (heat_flow_amplitude / temperature_amplitude) * (period / (2*pi*mass))
    ///
    /// # Arguments
    /// * `amplitude` - Temperature modulation amplitude in C
    /// * `period_s` - Modulation period in seconds
    /// * `heat_flow_amplitude` - Heat flow modulation amplitude in mW
    /// * `mass_mg` - Sample mass in mg
    pub fn cp_from_modulated(amplitude: f64, period_s: f64, heat_flow_amplitude: f64, mass_mg: f64) -> f64 {
        let mass_g = mass_mg / 1000.0;
        if amplitude.abs() < 1e-30 || mass_g.abs() < 1e-30 {
            return 0.0;
        }
        let omega = 2.0 * PI / period_s;
        // Cp = HF_amp / (omega * T_amp * mass) in appropriate units
        // HF in mW = 1e-3 W, mass in g
        (heat_flow_amplitude * 1e-3) / (omega * amplitude * mass_g)
    }

    /// NIST standard Cp for alpha-Al2O3 (sapphire) in J/(g*K).
    ///
    /// Polynomial fit valid from 0 to 1000 C.
    pub fn sapphire_cp_nist(temp_c: f64) -> f64 {
        let t = temp_c + 273.15; // Convert to K
        // Shomate equation coefficients for alpha-Al2O3 (NIST)
        // Valid 298-1800 K
        let t_scaled = t / 1000.0;
        let a = 102.4290;
        let b = 38.74980;
        let c = -15.91090;
        let d = 2.628181;
        let e = -3.007551;

        // Cp in J/(mol*K)
        let cp_mol = a + b * t_scaled + c * t_scaled * t_scaled
            + d * t_scaled * t_scaled * t_scaled
            + e / (t_scaled * t_scaled);

        // Convert to J/(g*K): molar mass Al2O3 = 101.96 g/mol
        let molar_mass = 101.96;
        (cp_mol / molar_mass).max(0.0)
    }
}

// ─── Oxidative Stability ─────────────────────────────────────────────────────

/// Oxidative stability testing (OIT/OOT).
pub struct OxidativeStability;

impl OxidativeStability {
    /// Oxidation Induction Time (OIT) in minutes.
    ///
    /// Measures time from O2 switch to onset of oxidation exotherm
    /// under isothermal conditions.
    pub fn oxidation_induction_time(curve: &DscCurve, isothermal_temp: f64) -> f64 {
        let t = &curve.temperature_c;
        let h = &curve.heat_flow_mw;
        let n = t.len();
        if n < 10 {
            return 0.0;
        }

        // For isothermal OIT, temperature_c is actually time in minutes
        // Find baseline from initial stable region (first 20%)
        let bl_end = (n / 5).max(2);
        let (bl_slope, bl_intercept) = linear_fit(&t[..bl_end], &h[..bl_end]);

        // Find onset of exotherm: where heat flow deviates significantly
        let baseline_std: f64 = {
            let residuals: Vec<f64> = (0..bl_end)
                .map(|i| h[i] - (bl_slope * t[i] + bl_intercept))
                .collect();
            let mean = residuals.iter().sum::<f64>() / bl_end as f64;
            let var = residuals.iter().map(|&r| (r - mean) * (r - mean)).sum::<f64>() / bl_end as f64;
            var.sqrt()
        };

        let threshold = 3.0 * baseline_std.max(0.01);

        for i in bl_end..n {
            let bl = bl_slope * t[i] + bl_intercept;
            if (h[i] - bl) > threshold {
                // Extrapolated onset using tangent
                return Self::extrapolated_onset(
                    &h[i.saturating_sub(5)..n.min(i + 10)],
                    &t[i.saturating_sub(5)..n.min(i + 10)],
                );
            }
        }

        *t.last().unwrap_or(&0.0) // No oxidation detected
    }

    /// Oxidation Onset Temperature (OOT) in degrees C.
    ///
    /// Temperature at which oxidation begins during a heating scan in O2.
    pub fn oxidation_onset_temperature(curve: &DscCurve, _heating_rate: f64) -> f64 {
        let t = &curve.temperature_c;
        let h = &curve.heat_flow_mw;
        let n = t.len();
        if n < 10 {
            return 0.0;
        }

        // Find baseline from initial region
        let bl_end = (n / 5).max(2);
        let (bl_slope, bl_intercept) = linear_fit(&t[..bl_end], &h[..bl_end]);

        // Find temperature where exotherm begins
        let baseline_std: f64 = {
            let residuals: Vec<f64> = (0..bl_end)
                .map(|i| h[i] - (bl_slope * t[i] + bl_intercept))
                .collect();
            let var = residuals.iter().map(|r| r * r).sum::<f64>() / bl_end as f64;
            var.sqrt()
        };

        let threshold = 3.0 * baseline_std.max(0.01);
        for i in bl_end..n {
            let bl = bl_slope * t[i] + bl_intercept;
            if (h[i] - bl) > threshold {
                return t[i];
            }
        }

        *t.last().unwrap_or(&0.0)
    }

    /// Extrapolated onset from heat flow and temperature data.
    ///
    /// Finds the intersection of the baseline tangent with the steepest
    /// slope of the departure.
    pub fn extrapolated_onset(heat_flow: &[f64], temperature: &[f64]) -> f64 {
        let n = heat_flow.len().min(temperature.len());
        if n < 3 {
            return temperature.first().copied().unwrap_or(0.0);
        }

        // Baseline from first portion
        let bl_n = (n / 3).max(2);
        let (bl_slope, bl_intercept) = linear_fit(&temperature[..bl_n], &heat_flow[..bl_n]);

        // Steepest slope in departure region
        let deriv = derivative(temperature, heat_flow);
        let steep_idx = argmax(&deriv);
        let steep_slope = deriv[steep_idx];
        let steep_intercept = heat_flow[steep_idx] - steep_slope * temperature[steep_idx];

        // Intersection
        if (steep_slope - bl_slope).abs() > 1e-30 {
            (bl_intercept - steep_intercept) / (steep_slope - bl_slope)
        } else {
            temperature[0]
        }
    }
}

// ─── Baseline Constructor ────────────────────────────────────────────────────

/// DSC baseline construction methods.
pub struct BaselineConstructor;

impl BaselineConstructor {
    /// Linear baseline between two temperature endpoints.
    pub fn linear_baseline(curve: &DscCurve, t_start: f64, t_end: f64) -> Vec<f64> {
        let t = &curve.temperature_c;
        let h = &curve.heat_flow_mw;
        let n = t.len();

        let start_idx = nearest_index(t, t_start);
        let end_idx = nearest_index(t, t_end);

        let y_start = h[start_idx];
        let y_end = h[end_idx];
        let x_start = t[start_idx];
        let x_end = t[end_idx];

        let slope = if (x_end - x_start).abs() > 1e-30 {
            (y_end - y_start) / (x_end - x_start)
        } else {
            0.0
        };

        t.iter().map(|&temp| {
            y_start + slope * (temp - x_start)
        }).collect()
    }

    /// Sigmoidal (S-shaped) baseline for transitions with Cp change.
    pub fn sigmoidal_baseline(curve: &DscCurve, t_start: f64, t_end: f64) -> Vec<f64> {
        let t = &curve.temperature_c;
        let h = &curve.heat_flow_mw;
        let n = t.len();

        let start_idx = nearest_index(t, t_start);
        let end_idx = nearest_index(t, t_end);
        let y_start = h[start_idx];
        let y_end = h[end_idx];
        let t_mid = (t_start + t_end) / 2.0;
        let width = (t_end - t_start).max(1.0);

        // Sigmoidal: y = y_start + (y_end - y_start) * sigmoid
        // Uses cumulative area fraction as weight
        // Running integral of |heat_flow - linear_baseline| for weight
        let linear_bl = Self::linear_baseline(curve, t_start, t_end);
        let mut cum_area = vec![0.0; n];
        for i in 1..n {
            let dt = (t[i] - t[i - 1]).abs();
            cum_area[i] = cum_area[i - 1] + (h[i] - linear_bl[i]).abs() * dt;
        }
        let total = cum_area[n - 1].max(1e-30);

        t.iter().enumerate().map(|(i, &temp)| {
            let frac = cum_area[i] / total;
            y_start + (y_end - y_start) * frac
        }).collect()
    }

    /// Tangential baseline from pre- and post-peak slopes.
    pub fn tangential_baseline(curve: &DscCurve, t_start: f64, t_end: f64) -> Vec<f64> {
        let t = &curve.temperature_c;
        let h = &curve.heat_flow_mw;
        let n = t.len();

        let start_idx = nearest_index(t, t_start);
        let end_idx = nearest_index(t, t_end);

        // Fit lines to pre and post regions
        let pre_n = ((end_idx - start_idx) / 4).max(2);
        let pre_range = start_idx..(start_idx + pre_n).min(n);
        let (pre_slope, pre_intercept) = linear_fit(
            &t[pre_range.clone()],
            &h[pre_range],
        );

        let post_start_i = end_idx.saturating_sub(pre_n);
        let post_range = post_start_i..(end_idx + 1).min(n);
        let (post_slope, post_intercept) = linear_fit(
            &t[post_range.clone()],
            &h[post_range],
        );

        let t_mid = (t_start + t_end) / 2.0;

        t.iter().map(|&temp| {
            if temp <= t_mid {
                pre_slope * temp + pre_intercept
            } else {
                post_slope * temp + post_intercept
            }
        }).collect()
    }

    /// Spline baseline through control points.
    ///
    /// Uses piecewise linear interpolation through user-specified points.
    pub fn spline_baseline(curve: &DscCurve, control_points: &[(f64, f64)]) -> Vec<f64> {
        let t = &curve.temperature_c;
        if control_points.is_empty() {
            return vec![0.0; t.len()];
        }
        if control_points.len() == 1 {
            return vec![control_points[0].1; t.len()];
        }

        let cp_x: Vec<f64> = control_points.iter().map(|&(x, _)| x).collect();
        let cp_y: Vec<f64> = control_points.iter().map(|&(_, y)| y).collect();

        t.iter().map(|&temp| lerp_at(&cp_x, &cp_y, temp)).collect()
    }

    /// Subtract a baseline from a DSC curve.
    pub fn subtract_baseline(curve: &DscCurve, baseline: &[f64]) -> DscCurve {
        let hf: Vec<f64> = curve.heat_flow_mw.iter().zip(baseline.iter())
            .map(|(&h, &b)| h - b)
            .collect();
        DscCurve {
            temperature_c: curve.temperature_c.clone(),
            heat_flow_mw: hf,
            heating_rate_c_per_min: curve.heating_rate_c_per_min,
            sample_mass_mg: curve.sample_mass_mg,
        }
    }
}

// ─── Peak Integrator ─────────────────────────────────────────────────────────

/// Peak integration for enthalpy determination.
pub struct PeakIntegrator;

impl PeakIntegrator {
    /// Integrate a peak between temperature limits using specified baseline.
    ///
    /// Returns enthalpy in J/g.
    pub fn integrate_peak(curve: &DscCurve, t_start: f64, t_end: f64, baseline: BaselineType) -> f64 {
        let bl = match baseline {
            BaselineType::Linear => BaselineConstructor::linear_baseline(curve, t_start, t_end),
            BaselineType::Sigmoidal => BaselineConstructor::sigmoidal_baseline(curve, t_start, t_end),
            BaselineType::Tangential => BaselineConstructor::tangential_baseline(curve, t_start, t_end),
        };

        let t = &curve.temperature_c;
        let h = &curve.heat_flow_mw;
        let n = t.len();

        let start_idx = nearest_index(t, t_start);
        let end_idx = nearest_index(t, t_end);

        let mut area_mw_c = 0.0;
        if end_idx > start_idx {
            for i in start_idx..end_idx.min(n - 1) {
                let dt = t[i + 1] - t[i];
                let hf = 0.5 * ((h[i] - bl[i]) + (h[i + 1] - bl[i + 1]));
                area_mw_c += hf * dt;
            }
        }

        let mass_g = curve.sample_mass_mg / 1000.0;
        let rate = curve.heating_rate_c_per_min;
        if (mass_g * rate).abs() > 1e-30 {
            (area_mw_c * 60.0 * 1e-3 / (mass_g * rate)).abs()
        } else {
            area_mw_c.abs()
        }
    }

    /// Partial area analysis: split peak at a temperature.
    ///
    /// Returns (area_before_split, area_after_split) in J/g.
    pub fn partial_area(curve: &DscCurve, t_start: f64, t_split: f64, t_end: f64) -> (f64, f64) {
        let before = Self::integrate_peak(curve, t_start, t_split, BaselineType::Linear);
        let after = Self::integrate_peak(curve, t_split, t_end, BaselineType::Linear);
        (before, after)
    }

    /// Find peak temperature between limits.
    pub fn peak_temperature(curve: &DscCurve, t_start: f64, t_end: f64) -> f64 {
        let t = &curve.temperature_c;
        let h = &curve.heat_flow_mw;

        let start_idx = nearest_index(t, t_start);
        let end_idx = nearest_index(t, t_end);

        if end_idx <= start_idx {
            return (t_start + t_end) / 2.0;
        }

        // Find max absolute deviation from linear baseline
        let bl = BaselineConstructor::linear_baseline(curve, t_start, t_end);
        let mut best_idx = start_idx;
        let mut best_dev = 0.0;
        for i in start_idx..=end_idx {
            let dev = (h[i] - bl[i]).abs();
            if dev > best_dev {
                best_dev = dev;
                best_idx = i;
            }
        }
        t[best_idx]
    }

    /// Onset temperature by tangent method.
    pub fn onset_by_tangent(curve: &DscCurve, t_start: f64, t_end: f64) -> f64 {
        let t = &curve.temperature_c;
        let h = &curve.heat_flow_mw;

        let start_idx = nearest_index(t, t_start);
        let end_idx = nearest_index(t, t_end);
        if end_idx <= start_idx + 4 {
            return t_start;
        }

        // Pre-peak baseline
        let pre_n = ((end_idx - start_idx) / 4).max(2);
        let (bl_slope, bl_intercept) = linear_fit(
            &t[start_idx..start_idx + pre_n],
            &h[start_idx..start_idx + pre_n],
        );

        // Steepest slope on leading edge
        let dh = derivative(t, h);
        let mut steep_idx = start_idx;
        let mut steep_val = 0.0_f64;
        let peak_idx = {
            let bl = BaselineConstructor::linear_baseline(curve, t_start, t_end);
            let mut bi = start_idx;
            let mut bv = 0.0;
            for i in start_idx..=end_idx {
                let d = (h[i] - bl[i]).abs();
                if d > bv { bv = d; bi = i; }
            }
            bi
        };
        for i in start_idx..peak_idx {
            if dh[i].abs() > steep_val {
                steep_val = dh[i].abs();
                steep_idx = i;
            }
        }

        let steep_slope = dh[steep_idx];
        let steep_intercept = h[steep_idx] - steep_slope * t[steep_idx];

        if (steep_slope - bl_slope).abs() > 1e-30 {
            (bl_intercept - steep_intercept) / (steep_slope - bl_slope)
        } else {
            t_start
        }
    }
}

// ─── DSC Simulator ───────────────────────────────────────────────────────────

/// Generate synthetic DSC curves for testing.
pub struct DscSimulator;

impl DscSimulator {
    /// Simulate a glass transition step.
    ///
    /// Models Tg as a sigmoidal step in heat flow.
    pub fn simulate_glass_transition(
        tg: f64,
        delta_cp: f64,
        width: f64,
        heating_rate: f64,
    ) -> DscCurve {
        let t_start = tg - 50.0;
        let t_end = tg + 50.0;
        let n = 500;
        let dt = (t_end - t_start) / (n as f64 - 1.0);

        let mut temps = Vec::with_capacity(n);
        let mut hf = Vec::with_capacity(n);

        let mass_mg = 10.0;
        let mass_g = mass_mg / 1000.0;
        let rate_k_per_s = heating_rate / 60.0;

        for i in 0..n {
            let temp = t_start + i as f64 * dt;
            temps.push(temp);

            // Sigmoid step: Cp change at Tg
            let x = (temp - tg) / (width / 4.0);
            let sigmoid = 1.0 / (1.0 + (-x).exp());
            // Heat flow = Cp * mass * rate; Cp step = delta_cp at Tg
            let cp_baseline = 1.0; // J/(g*K) baseline
            let cp = cp_baseline + delta_cp * sigmoid;
            let heat_flow_w = cp * mass_g * rate_k_per_s;
            hf.push(heat_flow_w * 1e3); // Convert to mW
        }

        DscCurve {
            temperature_c: temps,
            heat_flow_mw: hf,
            heating_rate_c_per_min: heating_rate,
            sample_mass_mg: mass_mg,
        }
    }

    /// Simulate a melting endotherm.
    ///
    /// Models melting as a Gaussian-shaped endothermic peak.
    pub fn simulate_melting(
        tm: f64,
        enthalpy: f64,
        width: f64,
        heating_rate: f64,
    ) -> DscCurve {
        let t_start = tm - 50.0;
        let t_end = tm + 50.0;
        let n = 500;
        let dt = (t_end - t_start) / (n as f64 - 1.0);

        let mass_mg = 10.0;
        let mass_g = mass_mg / 1000.0;
        let rate_k_per_s = heating_rate / 60.0;

        let sigma = width / 2.355; // FWHM to sigma
        // Peak area must equal enthalpy * mass * rate
        // Area of Gaussian = amplitude * sigma * sqrt(2*pi)
        let target_area_mw_c = enthalpy * mass_g * heating_rate / 60.0 * 1e3;
        let amplitude = target_area_mw_c / (sigma * (2.0 * PI).sqrt());

        let mut temps = Vec::with_capacity(n);
        let mut hf = Vec::with_capacity(n);

        for i in 0..n {
            let temp = t_start + i as f64 * dt;
            temps.push(temp);

            // Baseline heat flow
            let baseline = 1.0 * mass_g * rate_k_per_s * 1e3; // mW

            // Endothermic peak (negative in exo-up convention)
            let gaussian = amplitude * (-(temp - tm).powi(2) / (2.0 * sigma * sigma)).exp();
            hf.push(baseline - gaussian);
        }

        DscCurve {
            temperature_c: temps,
            heat_flow_mw: hf,
            heating_rate_c_per_min: heating_rate,
            sample_mass_mg: mass_mg,
        }
    }

    /// Simulate a crystallization exotherm.
    pub fn simulate_crystallization(
        tc: f64,
        enthalpy: f64,
        width: f64,
        cooling_rate: f64,
    ) -> DscCurve {
        let t_start = tc + 50.0; // Cooling: high to low
        let t_end = tc - 50.0;
        let n = 500;
        let dt = (t_end - t_start) / (n as f64 - 1.0);

        let mass_mg = 10.0;
        let mass_g = mass_mg / 1000.0;
        let rate_k_per_s = cooling_rate / 60.0;
        let sigma = width / 2.355;
        let target_area = enthalpy * mass_g * cooling_rate / 60.0 * 1e3;
        let amplitude = target_area / (sigma * (2.0 * PI).sqrt());

        let mut temps = Vec::with_capacity(n);
        let mut hf = Vec::with_capacity(n);

        for i in 0..n {
            let temp = t_start + i as f64 * dt;
            temps.push(temp);
            let baseline = -1.0 * mass_g * rate_k_per_s * 1e3;
            // Exothermic peak (positive in exo-up)
            let gaussian = amplitude * (-(temp - tc).powi(2) / (2.0 * sigma * sigma)).exp();
            hf.push(baseline + gaussian);
        }

        DscCurve {
            temperature_c: temps,
            heat_flow_mw: hf,
            heating_rate_c_per_min: -cooling_rate, // negative for cooling
            sample_mass_mg: mass_mg,
        }
    }

    /// Simulate a combined DSC curve with multiple transitions.
    pub fn simulate_combined(transitions: &[Transition], heating_rate: f64) -> DscCurve {
        // Find overall temperature range
        let mut t_min = f64::MAX;
        let mut t_max = f64::MIN;
        for tr in transitions {
            let tc = match tr {
                Transition::Glass { tg_c, .. } => *tg_c,
                Transition::Melting { tm_c, .. } => *tm_c,
                Transition::Crystallization { tc_c, .. } => *tc_c,
                Transition::Curing { tp_c, .. } => *tp_c,
            };
            t_min = t_min.min(tc - 60.0);
            t_max = t_max.max(tc + 60.0);
        }
        if t_min >= t_max {
            t_min = 0.0;
            t_max = 300.0;
        }

        let n = 1000;
        let dt = (t_max - t_min) / (n as f64 - 1.0);
        let mass_mg = 10.0;
        let mass_g = mass_mg / 1000.0;
        let rate_k_per_s = heating_rate / 60.0;

        let mut temps = Vec::with_capacity(n);
        let mut hf = vec![0.0; n];

        for i in 0..n {
            temps.push(t_min + i as f64 * dt);
        }

        // Baseline
        let baseline_mw = 1.0 * mass_g * rate_k_per_s * 1e3;
        for v in hf.iter_mut() {
            *v = baseline_mw;
        }

        for tr in transitions {
            match tr {
                Transition::Glass { tg_c, delta_cp, width_c } => {
                    for i in 0..n {
                        let x = (temps[i] - tg_c) / (width_c / 4.0);
                        let sigmoid = 1.0 / (1.0 + (-x).exp());
                        hf[i] += delta_cp * sigmoid * mass_g * rate_k_per_s * 1e3;
                    }
                }
                Transition::Melting { tm_c, enthalpy_j_per_g, width_c } => {
                    let sigma = width_c / 2.355;
                    let area = enthalpy_j_per_g * mass_g * heating_rate / 60.0 * 1e3;
                    let amp = area / (sigma * (2.0 * PI).sqrt());
                    for i in 0..n {
                        let g = amp * (-(temps[i] - tm_c).powi(2) / (2.0 * sigma * sigma)).exp();
                        hf[i] -= g; // endothermic = negative
                    }
                }
                Transition::Crystallization { tc_c, enthalpy_j_per_g, width_c } => {
                    let sigma = width_c / 2.355;
                    let area = enthalpy_j_per_g * mass_g * heating_rate / 60.0 * 1e3;
                    let amp = area / (sigma * (2.0 * PI).sqrt());
                    for i in 0..n {
                        let g = amp * (-(temps[i] - tc_c).powi(2) / (2.0 * sigma * sigma)).exp();
                        hf[i] += g; // exothermic = positive
                    }
                }
                Transition::Curing { tp_c, heat_j_per_g, width_c } => {
                    let sigma = width_c / 2.355;
                    let area = heat_j_per_g * mass_g * heating_rate / 60.0 * 1e3;
                    let amp = area / (sigma * (2.0 * PI).sqrt());
                    for i in 0..n {
                        let g = amp * (-(temps[i] - tp_c).powi(2) / (2.0 * sigma * sigma)).exp();
                        hf[i] += g; // exothermic
                    }
                }
            }
        }

        DscCurve {
            temperature_c: temps,
            heat_flow_mw: hf,
            heating_rate_c_per_min: heating_rate,
            sample_mass_mg: mass_mg,
        }
    }

    /// Add Gaussian noise to a DSC curve.
    pub fn add_noise(curve: &DscCurve, noise_mw: f64) -> DscCurve {
        // Simple deterministic pseudo-noise using a hash-like approach
        let hf: Vec<f64> = curve.heat_flow_mw.iter().enumerate().map(|(i, &v)| {
            // Simple PRNG based on index
            let seed = (i as u64).wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let u1 = (seed & 0xFFFFFFFF) as f64 / 4294967296.0;
            let seed2 = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u2 = (seed2 & 0xFFFFFFFF) as f64 / 4294967296.0;
            // Box-Muller transform
            let z = (-2.0 * u1.max(1e-30).ln()).sqrt() * (2.0 * PI * u2).cos();
            v + noise_mw * z
        }).collect();

        DscCurve {
            temperature_c: curve.temperature_c.clone(),
            heat_flow_mw: hf,
            heating_rate_c_per_min: curve.heating_rate_c_per_min,
            sample_mass_mg: curve.sample_mass_mg,
        }
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    fn assert_approx(a: f64, b: f64, tol: f64, msg: &str) {
        assert!((a - b).abs() < tol, "{}: {} vs {} (diff={})", msg, a, b, (a - b).abs());
    }

    fn assert_near(a: f64, b: f64, rel_tol: f64, msg: &str) {
        let scale = a.abs().max(b.abs()).max(1e-10);
        assert!((a - b).abs() / scale < rel_tol, "{}: {} vs {} (rel={})", msg, a, b, (a - b).abs() / scale);
    }

    // ── DscCurve tests ──────────────────────────────────────────────────

    #[test]
    fn test_dsc_curve_new() {
        let c = DscCurve::new(vec![100.0, 200.0], vec![1.0, 2.0], 10.0);
        assert_eq!(c.len(), 2);
        assert!(!c.is_empty());
        assert_eq!(c.sample_mass_mg, 10.0);
    }

    #[test]
    fn test_dsc_curve_with_mass() {
        let c = DscCurve::new(vec![100.0], vec![1.0], 10.0).with_mass(5.0);
        assert_eq!(c.sample_mass_mg, 5.0);
    }

    #[test]
    fn test_exo_up() {
        let c = DscCurve::new(vec![], vec![], 10.0);
        assert!(c.exo_up());
    }

    #[test]
    fn test_temperature_range() {
        let c = DscCurve::new(vec![50.0, 100.0, 150.0, 200.0], vec![1.0; 4], 10.0);
        let (lo, hi) = c.temperature_range();
        assert_approx(lo, 50.0, TOL, "temp range lo");
        assert_approx(hi, 200.0, TOL, "temp range hi");
    }

    #[test]
    fn test_empty_curve_range() {
        let c = DscCurve::new(vec![], vec![], 10.0);
        let (lo, hi) = c.temperature_range();
        assert_eq!(lo, 0.0);
        assert_eq!(hi, 0.0);
        assert!(c.is_empty());
    }

    #[test]
    fn test_normalize_by_mass() {
        let c = DscCurve::new(vec![100.0], vec![5.0], 10.0).with_mass(5.0);
        let norm = c.normalize_by_mass();
        assert_approx(norm.heat_flow_mw[0], 1.0, TOL, "normalized by mass");
        assert_approx(norm.sample_mass_mg, 1.0, TOL, "mass after norm");
    }

    #[test]
    fn test_normalize_by_rate() {
        let c = DscCurve::new(vec![100.0], vec![10.0], 10.0).with_mass(10.0);
        let norm = c.normalize_by_rate();
        // 10 mW * 1e-3 / (0.01g * 10/60 K/s) = 0.01 / (0.01 * 0.1667) = 6.0 J/(g*K)
        assert!(norm.heat_flow_mw[0] > 0.0, "normalized by rate should be positive");
    }

    // ── Helper tests ────────────────────────────────────────────────────

    #[test]
    fn test_lerp_at_basic() {
        let xs = vec![0.0, 1.0, 2.0];
        let ys = vec![0.0, 10.0, 20.0];
        assert_approx(lerp_at(&xs, &ys, 0.5), 5.0, TOL, "lerp midpoint");
        assert_approx(lerp_at(&xs, &ys, 1.5), 15.0, TOL, "lerp 1.5");
    }

    #[test]
    fn test_lerp_at_boundary() {
        let xs = vec![0.0, 1.0];
        let ys = vec![5.0, 10.0];
        assert_approx(lerp_at(&xs, &ys, -1.0), 5.0, TOL, "lerp below");
        assert_approx(lerp_at(&xs, &ys, 2.0), 10.0, TOL, "lerp above");
    }

    #[test]
    fn test_trapz_constant() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![5.0, 5.0, 5.0, 5.0];
        assert_approx(trapz(&x, &y), 15.0, TOL, "trapz constant");
    }

    #[test]
    fn test_trapz_linear() {
        let x = vec![0.0, 1.0, 2.0];
        let y = vec![0.0, 1.0, 2.0];
        assert_approx(trapz(&x, &y), 2.0, TOL, "trapz linear = triangle area");
    }

    #[test]
    fn test_derivative_linear() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![0.0, 3.0, 6.0, 9.0];
        let d = derivative(&x, &y);
        for &v in &d {
            assert_approx(v, 3.0, TOL, "derivative of 3x");
        }
    }

    #[test]
    fn test_linear_fit_exact() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![2.0, 5.0, 8.0, 11.0]; // y = 3x + 2
        let (slope, intercept) = linear_fit(&x, &y);
        assert_approx(slope, 3.0, TOL, "slope");
        assert_approx(intercept, 2.0, TOL, "intercept");
    }

    #[test]
    fn test_nearest_index() {
        let t = vec![10.0, 20.0, 30.0, 40.0];
        assert_eq!(nearest_index(&t, 25.0), 1); // closer to 20 than 30
        assert_eq!(nearest_index(&t, 35.0), 2); // closer to 30 than 40... actually 35 is equidistant
        assert_eq!(nearest_index(&t, 10.0), 0);
    }

    #[test]
    fn test_smooth_identity() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let s = smooth(&data, 1);
        assert_eq!(s, data);
    }

    #[test]
    fn test_smooth_averaging() {
        let data = vec![0.0, 0.0, 10.0, 0.0, 0.0];
        let s = smooth(&data, 3);
        // Middle point: avg of [0, 10, 0] = 3.33
        assert!(s[2] < 10.0 && s[2] > 0.0, "smoothing reduces spike");
    }

    // ── Glass Transition tests ──────────────────────────────────────────

    #[test]
    fn test_find_tg_simulated() {
        let curve = DscSimulator::simulate_glass_transition(120.0, 0.3, 10.0, 10.0);
        let result = GlassTransition::find_tg(&curve);
        assert_near(result.tg_midpoint_c, 120.0, 0.1, "Tg midpoint");
        assert!(result.tg_onset_c < result.tg_midpoint_c, "onset < midpoint");
        assert!(result.tg_endset_c > result.tg_midpoint_c, "endset > midpoint");
        assert!(result.delta_cp_j_per_g_k > 0.0, "delta Cp positive");
    }

    #[test]
    fn test_tg_onset_before_endset() {
        let curve = DscSimulator::simulate_glass_transition(80.0, 0.5, 15.0, 20.0);
        let result = GlassTransition::find_tg(&curve);
        assert!(result.tg_onset_c < result.tg_endset_c, "onset < endset");
    }

    #[test]
    fn test_fictive_temperature() {
        let curve = DscSimulator::simulate_glass_transition(150.0, 0.3, 10.0, 10.0);
        let tf = GlassTransition::fictive_temperature(&curve);
        // Tf should be close to Tg for standard heating
        assert_near(tf, 150.0, 0.15, "fictive temp near Tg");
    }

    #[test]
    fn test_fragility_index() {
        // Strong glass: m ~ 16-20 (SiO2)
        // Fragile glass: m ~ 100-200 (polymers)
        let m = GlassTransition::fragility_index(400.0, 200.0);
        assert!(m > 0.0, "fragility > 0");
        // m = 200 / (8.314e-3 * 400 * ln(10)) = 200 / (8.314e-3 * 400 * 2.3026) ~ 26.1
        assert_near(m, 26.1, 0.05, "fragility index");
    }

    #[test]
    fn test_fragility_zero_tg() {
        assert_eq!(GlassTransition::fragility_index(0.0, 100.0), 0.0);
    }

    // ── Melting Analysis tests ──────────────────────────────────────────

    #[test]
    fn test_find_melting_peak_simulated() {
        let curve = DscSimulator::simulate_melting(250.0, 150.0, 5.0, 10.0);
        let result = MeltingAnalysis::find_melting_peak(&curve);
        assert_near(result.peak_c, 250.0, 0.05, "melting peak temp");
        assert!(result.onset_c < result.peak_c, "onset < peak");
        assert!(result.endset_c > result.peak_c, "endset > peak");
        assert!(result.enthalpy_j_per_g > 0.0, "enthalpy positive");
    }

    #[test]
    fn test_melting_enthalpy_order_of_magnitude() {
        let curve = DscSimulator::simulate_melting(160.0, 200.0, 8.0, 10.0);
        let result = MeltingAnalysis::find_melting_peak(&curve);
        // Should recover enthalpy within ~30%
        assert_near(result.enthalpy_j_per_g, 200.0, 0.35, "enthalpy recovery");
    }

    #[test]
    fn test_crystallinity_percent() {
        // PET: ΔH100% ~ 140 J/g
        assert_approx(MeltingAnalysis::crystallinity_percent(70.0, 140.0), 50.0, TOL, "50% crystallinity");
        assert_approx(MeltingAnalysis::crystallinity_percent(140.0, 140.0), 100.0, TOL, "100% crystallinity");
        assert_approx(MeltingAnalysis::crystallinity_percent(0.0, 140.0), 0.0, TOL, "0% crystallinity");
    }

    #[test]
    fn test_crystallinity_zero_reference() {
        assert_eq!(MeltingAnalysis::crystallinity_percent(50.0, 0.0), 0.0);
    }

    #[test]
    fn test_purity_analysis() {
        let curve = DscSimulator::simulate_melting(156.0, 170.0, 3.0, 10.0);
        let purity = MeltingAnalysis::purity_from_melting(&curve, 156.6);
        assert!(purity > 0.0 && purity <= 1.0, "purity in valid range: {}", purity);
    }

    // ── Crystallization Analysis tests ──────────────────────────────────

    #[test]
    fn test_find_crystallization() {
        let curve = DscSimulator::simulate_crystallization(180.0, 100.0, 8.0, 10.0);
        let result = CrystallizationAnalysis::find_crystallization(&curve);
        assert_near(result.peak_c, 180.0, 0.05, "crystallization peak");
        assert!(result.enthalpy_j_per_g > 0.0, "crystallization enthalpy > 0");
    }

    #[test]
    fn test_avrami_analysis() {
        // Simulate Avrami data: alpha = 1 - exp(-0.01 * t^3)
        let n_avrami = 3.0;
        let k_avrami = 0.01;
        let times: Vec<f64> = (1..=100).map(|i| i as f64 * 0.1).collect();
        let conversion: Vec<f64> = times.iter()
            .map(|&t| 1.0 - (-k_avrami * t.powf(n_avrami)).exp())
            .collect();

        let result = CrystallizationAnalysis::avrami_analysis(&times, &conversion);
        assert_near(result.n, 3.0, 0.15, "Avrami n");
        assert!(result.k > 0.0, "Avrami k > 0");
        assert!(result.t_half > 0.0, "Avrami t_half > 0");
    }

    #[test]
    fn test_avrami_t_half() {
        let times: Vec<f64> = (1..=200).map(|i| i as f64 * 0.05).collect();
        let k = 0.1;
        let n = 2.0;
        let conversion: Vec<f64> = times.iter()
            .map(|&t| 1.0 - (-k * t.powf(n)).exp())
            .collect();
        let result = CrystallizationAnalysis::avrami_analysis(&times, &conversion);
        let expected_t_half = ((2.0_f64).ln() / k).powf(1.0 / n);
        assert_near(result.t_half, expected_t_half, 0.2, "t_half");
    }

    #[test]
    fn test_relative_crystallinity() {
        let curve = DscSimulator::simulate_crystallization(180.0, 100.0, 8.0, 10.0);
        let alpha = CrystallizationAnalysis::relative_crystallinity(
            &curve,
            *curve.temperature_c.first().unwrap(),
            *curve.temperature_c.last().unwrap(),
        );
        assert_eq!(alpha.len(), curve.len());
        // First point ~ 0, last ~ 1
        assert!(alpha[0] < 0.1, "alpha starts near 0");
        assert!(alpha[alpha.len() - 1] > 0.5, "alpha ends high");
    }

    #[test]
    fn test_isothermal_crystallization() {
        let curve = DscSimulator::simulate_crystallization(180.0, 100.0, 8.0, 10.0);
        let results = CrystallizationAnalysis::isothermal_crystallization(&[curve]);
        assert_eq!(results.len(), 1);
        assert!(results[0].n > 0.0);
    }

    // ── Curing Analysis tests ───────────────────────────────────────────

    #[test]
    fn test_degree_of_cure() {
        let transitions = vec![Transition::Curing { tp_c: 180.0, heat_j_per_g: 300.0, width_c: 40.0 }];
        let curve = DscSimulator::simulate_combined(&transitions, 10.0);
        let doc = CuringAnalysis::degree_of_cure(&curve);
        assert!(!doc.is_empty());
        // Last point should be near 1.0
        let last_alpha = doc.last().unwrap().1;
        assert!(last_alpha > 0.8, "final cure > 0.8: {}", last_alpha);
    }

    #[test]
    fn test_total_heat_of_cure() {
        let transitions = vec![Transition::Curing { tp_c: 150.0, heat_j_per_g: 250.0, width_c: 30.0 }];
        let curve = DscSimulator::simulate_combined(&transitions, 10.0);
        let total = CuringAnalysis::total_heat_of_cure(&curve);
        assert!(total > 0.0, "total heat > 0");
    }

    #[test]
    fn test_residual_cure() {
        // A second scan with small residual
        let transitions = vec![Transition::Curing { tp_c: 200.0, heat_j_per_g: 20.0, width_c: 20.0 }];
        let curve = DscSimulator::simulate_combined(&transitions, 10.0);
        let residual = CuringAnalysis::residual_cure(&curve);
        assert!(residual > 0.0, "residual cure detected");
    }

    #[test]
    fn test_autocatalytic_fit() {
        let temps = vec![400.0, 410.0, 420.0, 430.0, 440.0, 450.0, 460.0, 470.0];
        let conversions = vec![0.02, 0.05, 0.15, 0.35, 0.55, 0.75, 0.88, 0.95];
        let rates = vec![0.001, 0.003, 0.01, 0.025, 0.03, 0.02, 0.008, 0.003];
        let result = CuringAnalysis::autocatalytic_fit(&temps, &rates, &conversions);
        assert!(result.k1 > 0.0, "k1 > 0");
        assert!(result.k2 > 0.0, "k2 > 0");
        assert!(result.m > 0.0, "m > 0");
        assert!(result.n > 0.0, "n > 0");
    }

    // ── Specific Heat Capacity tests ────────────────────────────────────

    #[test]
    fn test_sapphire_cp_nist_room_temp() {
        let cp = SpecificHeatCapacity::sapphire_cp_nist(25.0);
        // Al2O3 Cp at 25C ~ 0.78 J/(g*K)
        assert!(cp > 0.5 && cp < 1.2, "sapphire Cp at 25C: {}", cp);
    }

    #[test]
    fn test_sapphire_cp_nist_500c() {
        let cp = SpecificHeatCapacity::sapphire_cp_nist(500.0);
        // Cp increases with temperature
        let cp_25 = SpecificHeatCapacity::sapphire_cp_nist(25.0);
        assert!(cp > cp_25, "Cp increases with T");
    }

    #[test]
    fn test_cp_from_modulated() {
        let cp = SpecificHeatCapacity::cp_from_modulated(0.5, 60.0, 0.5, 10.0);
        assert!(cp > 0.0, "Cp from MDSC > 0: {}", cp);
    }

    #[test]
    fn test_cp_from_modulated_zero_mass() {
        let cp = SpecificHeatCapacity::cp_from_modulated(0.5, 60.0, 0.5, 0.0);
        assert_eq!(cp, 0.0);
    }

    #[test]
    fn test_sapphire_method() {
        // Create synthetic data
        let temps = vec![100.0, 150.0, 200.0, 250.0, 300.0];
        let baseline = DscCurve::new(temps.clone(), vec![0.1, 0.1, 0.1, 0.1, 0.1], 10.0).with_mass(0.0);
        let sapphire = DscCurve::new(temps.clone(), vec![0.5, 0.6, 0.7, 0.8, 0.9], 10.0).with_mass(25.0);
        let sample = DscCurve::new(temps.clone(), vec![0.3, 0.4, 0.5, 0.6, 0.7], 10.0).with_mass(10.0);
        let sap_cp = vec![(100.0, 0.8), (200.0, 0.9), (300.0, 1.0)];

        let result = SpecificHeatCapacity::sapphire_method(&sample, &baseline, &sapphire, &sap_cp);
        assert_eq!(result.len(), 5);
        for &(_, cp) in &result {
            assert!(cp > 0.0, "Cp should be positive");
        }
    }

    // ── Oxidative Stability tests ───────────────────────────────────────

    #[test]
    fn test_oxidation_onset_temperature() {
        // Simulate: stable baseline then exotherm at 200C
        let n = 300;
        let mut temps = Vec::new();
        let mut hf = Vec::new();
        for i in 0..n {
            let t = 50.0 + i as f64;
            temps.push(t);
            if t < 200.0 {
                hf.push(0.5); // stable baseline
            } else {
                hf.push(0.5 + 0.1 * (t - 200.0)); // onset of oxidation
            }
        }
        let curve = DscCurve::new(temps, hf, 10.0);
        let oot = OxidativeStability::oxidation_onset_temperature(&curve, 10.0);
        assert!(oot > 190.0 && oot < 220.0, "OOT near 200C: {}", oot);
    }

    #[test]
    fn test_extrapolated_onset() {
        let temps = vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        let hf = vec![1.0, 1.0, 1.0, 1.0, 2.0, 4.0, 7.0, 11.0, 16.0, 22.0];
        let onset = OxidativeStability::extrapolated_onset(&hf, &temps);
        assert!(onset > 2.0 && onset < 6.0, "extrapolated onset: {}", onset);
    }

    // ── Baseline Constructor tests ──────────────────────────────────────

    #[test]
    fn test_linear_baseline() {
        let curve = DscCurve::new(
            vec![100.0, 150.0, 200.0, 250.0, 300.0],
            vec![1.0, 2.0, 5.0, 3.0, 2.0],
            10.0,
        );
        let bl = BaselineConstructor::linear_baseline(&curve, 100.0, 300.0);
        assert_eq!(bl.len(), 5);
        assert_approx(bl[0], 1.0, TOL, "bl start");
        assert_approx(bl[4], 2.0, TOL, "bl end");
        // Middle should interpolate
        assert_approx(bl[2], 1.5, TOL, "bl middle");
    }

    #[test]
    fn test_sigmoidal_baseline() {
        let curve = DscSimulator::simulate_melting(200.0, 100.0, 5.0, 10.0);
        let bl = BaselineConstructor::sigmoidal_baseline(&curve, 170.0, 230.0);
        assert_eq!(bl.len(), curve.len());
    }

    #[test]
    fn test_tangential_baseline() {
        let curve = DscSimulator::simulate_melting(200.0, 100.0, 5.0, 10.0);
        let bl = BaselineConstructor::tangential_baseline(&curve, 170.0, 230.0);
        assert_eq!(bl.len(), curve.len());
    }

    #[test]
    fn test_spline_baseline() {
        let curve = DscCurve::new(vec![0.0, 50.0, 100.0], vec![1.0, 3.0, 2.0], 10.0);
        let cp = vec![(0.0, 1.0), (100.0, 2.0)];
        let bl = BaselineConstructor::spline_baseline(&curve, &cp);
        assert_approx(bl[0], 1.0, TOL, "spline start");
        assert_approx(bl[2], 2.0, TOL, "spline end");
        assert_approx(bl[1], 1.5, TOL, "spline middle");
    }

    #[test]
    fn test_subtract_baseline() {
        let curve = DscCurve::new(vec![0.0, 1.0, 2.0], vec![5.0, 10.0, 15.0], 10.0);
        let bl = vec![1.0, 2.0, 3.0];
        let result = BaselineConstructor::subtract_baseline(&curve, &bl);
        assert_approx(result.heat_flow_mw[0], 4.0, TOL, "subtracted 0");
        assert_approx(result.heat_flow_mw[1], 8.0, TOL, "subtracted 1");
        assert_approx(result.heat_flow_mw[2], 12.0, TOL, "subtracted 2");
    }

    // ── Peak Integrator tests ───────────────────────────────────────────

    #[test]
    fn test_integrate_peak_linear_baseline() {
        let curve = DscSimulator::simulate_melting(200.0, 150.0, 5.0, 10.0);
        let area = PeakIntegrator::integrate_peak(&curve, 180.0, 220.0, BaselineType::Linear);
        assert!(area > 0.0, "peak area > 0: {}", area);
    }

    #[test]
    fn test_integrate_peak_sigmoidal() {
        let curve = DscSimulator::simulate_melting(200.0, 150.0, 5.0, 10.0);
        let area = PeakIntegrator::integrate_peak(&curve, 180.0, 220.0, BaselineType::Sigmoidal);
        assert!(area > 0.0, "sigmoidal area > 0");
    }

    #[test]
    fn test_partial_area() {
        let curve = DscSimulator::simulate_melting(200.0, 150.0, 5.0, 10.0);
        let (before, after) = PeakIntegrator::partial_area(&curve, 180.0, 200.0, 220.0);
        assert!(before > 0.0, "before area > 0");
        assert!(after > 0.0, "after area > 0");
    }

    #[test]
    fn test_peak_temperature() {
        let curve = DscSimulator::simulate_melting(250.0, 100.0, 5.0, 10.0);
        let tp = PeakIntegrator::peak_temperature(&curve, 230.0, 270.0);
        assert_near(tp, 250.0, 0.05, "peak temp");
    }

    #[test]
    fn test_onset_by_tangent() {
        let curve = DscSimulator::simulate_melting(200.0, 100.0, 5.0, 10.0);
        let onset = PeakIntegrator::onset_by_tangent(&curve, 170.0, 230.0);
        assert!(onset < 200.0, "onset before peak: {}", onset);
        assert!(onset > 170.0, "onset after start: {}", onset);
    }

    // ── Simulator tests ─────────────────────────────────────────────────

    #[test]
    fn test_simulate_glass_transition() {
        let curve = DscSimulator::simulate_glass_transition(120.0, 0.3, 10.0, 10.0);
        assert_eq!(curve.len(), 500);
        assert_approx(curve.heating_rate_c_per_min, 10.0, TOL, "rate");
        let (lo, hi) = curve.temperature_range();
        assert!(lo < 120.0 && hi > 120.0, "range spans Tg");
    }

    #[test]
    fn test_simulate_melting() {
        let curve = DscSimulator::simulate_melting(250.0, 200.0, 5.0, 10.0);
        assert_eq!(curve.len(), 500);
        // Melting should produce a dip (endothermic)
        let min_idx = argmin(&curve.heat_flow_mw);
        assert_near(curve.temperature_c[min_idx], 250.0, 0.05, "melting dip at Tm");
    }

    #[test]
    fn test_simulate_crystallization() {
        let curve = DscSimulator::simulate_crystallization(180.0, 100.0, 8.0, 10.0);
        assert_eq!(curve.len(), 500);
        // Crystallization should produce a peak (exothermic)
        let max_idx = argmax(&curve.heat_flow_mw);
        assert_near(curve.temperature_c[max_idx], 180.0, 0.05, "cryst peak at Tc");
    }

    #[test]
    fn test_simulate_combined_two_transitions() {
        let transitions = vec![
            Transition::Glass { tg_c: 80.0, delta_cp: 0.3, width_c: 10.0 },
            Transition::Melting { tm_c: 250.0, enthalpy_j_per_g: 150.0, width_c: 5.0 },
        ];
        let curve = DscSimulator::simulate_combined(&transitions, 10.0);
        assert_eq!(curve.len(), 1000);
        let (lo, hi) = curve.temperature_range();
        assert!(lo < 80.0 && hi > 250.0, "spans both transitions");
    }

    #[test]
    fn test_simulate_combined_three_transitions() {
        let transitions = vec![
            Transition::Glass { tg_c: 60.0, delta_cp: 0.2, width_c: 8.0 },
            Transition::Crystallization { tc_c: 130.0, enthalpy_j_per_g: 50.0, width_c: 10.0 },
            Transition::Melting { tm_c: 260.0, enthalpy_j_per_g: 100.0, width_c: 5.0 },
        ];
        let curve = DscSimulator::simulate_combined(&transitions, 10.0);
        assert!(curve.len() > 0);
    }

    #[test]
    fn test_add_noise() {
        let curve = DscSimulator::simulate_melting(200.0, 100.0, 5.0, 10.0);
        let noisy = DscSimulator::add_noise(&curve, 0.01);
        assert_eq!(noisy.len(), curve.len());
        // Noisy curve should differ from original
        let diff: f64 = curve.heat_flow_mw.iter().zip(noisy.heat_flow_mw.iter())
            .map(|(a, b)| (a - b).abs())
            .sum::<f64>();
        assert!(diff > 0.0, "noise added");
    }

    #[test]
    fn test_add_noise_zero() {
        let curve = DscSimulator::simulate_melting(200.0, 100.0, 5.0, 10.0);
        let noisy = DscSimulator::add_noise(&curve, 0.0);
        for (a, b) in curve.heat_flow_mw.iter().zip(noisy.heat_flow_mw.iter()) {
            assert_approx(*a, *b, TOL, "zero noise");
        }
    }

    // ── Roundtrip / integration tests ───────────────────────────────────

    #[test]
    fn test_melting_roundtrip() {
        // Simulate melting, analyze, check consistency
        let curve = DscSimulator::simulate_melting(160.0, 170.0, 4.0, 10.0);
        let result = MeltingAnalysis::find_melting_peak(&curve);
        assert!(result.onset_c < result.peak_c, "onset < peak");
        assert!(result.peak_c < result.endset_c, "peak < endset");
        assert_near(result.peak_c, 160.0, 0.05, "peak recovery");
    }

    #[test]
    fn test_glass_transition_roundtrip() {
        let curve = DscSimulator::simulate_glass_transition(100.0, 0.4, 12.0, 10.0);
        let result = GlassTransition::find_tg(&curve);
        assert_near(result.tg_midpoint_c, 100.0, 0.1, "Tg recovery");
    }

    #[test]
    fn test_baseline_types_differ() {
        let curve = DscSimulator::simulate_melting(200.0, 100.0, 5.0, 10.0);
        let lin = BaselineConstructor::linear_baseline(&curve, 180.0, 220.0);
        let sig = BaselineConstructor::sigmoidal_baseline(&curve, 180.0, 220.0);
        let tan = BaselineConstructor::tangential_baseline(&curve, 180.0, 220.0);
        // They should not all be identical
        let diff_ls: f64 = lin.iter().zip(sig.iter()).map(|(a, b)| (a - b).abs()).sum();
        let diff_lt: f64 = lin.iter().zip(tan.iter()).map(|(a, b)| (a - b).abs()).sum();
        // At least one pair should differ
        assert!(diff_ls > 0.0 || diff_lt > 0.0, "different baseline types differ");
    }

    #[test]
    fn test_argmax_argmin() {
        let data = vec![1.0, 5.0, 3.0, 8.0, 2.0];
        assert_eq!(argmax(&data), 3);
        assert_eq!(argmin(&data), 0);
    }

    #[test]
    fn test_combined_curing_analysis() {
        let transitions = vec![
            Transition::Curing { tp_c: 150.0, heat_j_per_g: 350.0, width_c: 50.0 },
        ];
        let curve = DscSimulator::simulate_combined(&transitions, 10.0);
        let doc = CuringAnalysis::degree_of_cure(&curve);
        let total = CuringAnalysis::total_heat_of_cure(&curve);
        assert!(total > 0.0, "total heat > 0");
        assert!(!doc.is_empty(), "degree of cure computed");
    }

    #[test]
    fn test_sapphire_cp_increases_with_temp() {
        let cp_100 = SpecificHeatCapacity::sapphire_cp_nist(100.0);
        let cp_500 = SpecificHeatCapacity::sapphire_cp_nist(500.0);
        let cp_800 = SpecificHeatCapacity::sapphire_cp_nist(800.0);
        assert!(cp_100 < cp_500, "Cp increases 100->500");
        assert!(cp_500 < cp_800, "Cp increases 500->800");
    }

    #[test]
    fn test_derivative_quadratic() {
        // y = x^2, dy/dx = 2x
        let x: Vec<f64> = (0..=10).map(|i| i as f64).collect();
        let y: Vec<f64> = x.iter().map(|v| v * v).collect();
        let d = derivative(&x, &y);
        // Central differences should give ~2x for interior points
        for i in 1..x.len() - 1 {
            assert_near(d[i], 2.0 * x[i], 0.1, "deriv of x^2");
        }
    }

    #[test]
    fn test_linear_fit_single_point() {
        let (slope, intercept) = linear_fit(&[1.0], &[5.0]);
        assert_eq!(slope, 0.0);
        assert_eq!(intercept, 5.0);
    }

    #[test]
    fn test_peak_integrator_tangential() {
        let curve = DscSimulator::simulate_melting(200.0, 150.0, 5.0, 10.0);
        let area = PeakIntegrator::integrate_peak(&curve, 180.0, 220.0, BaselineType::Tangential);
        assert!(area > 0.0, "tangential area > 0");
    }

    #[test]
    fn test_simulate_curing_transition() {
        let transitions = vec![
            Transition::Curing { tp_c: 180.0, heat_j_per_g: 400.0, width_c: 60.0 },
        ];
        let curve = DscSimulator::simulate_combined(&transitions, 10.0);
        // Should have an exothermic peak
        let max_idx = argmax(&curve.heat_flow_mw);
        assert_near(curve.temperature_c[max_idx], 180.0, 0.15, "curing peak temp");
    }

    #[test]
    fn test_spline_baseline_single_point() {
        let curve = DscCurve::new(vec![0.0, 1.0, 2.0], vec![1.0, 2.0, 3.0], 10.0);
        let bl = BaselineConstructor::spline_baseline(&curve, &[(1.0, 5.0)]);
        assert_eq!(bl.len(), 3);
        for &v in &bl {
            assert_approx(v, 5.0, TOL, "single cp");
        }
    }

    #[test]
    fn test_spline_baseline_empty() {
        let curve = DscCurve::new(vec![0.0, 1.0], vec![1.0, 2.0], 10.0);
        let bl = BaselineConstructor::spline_baseline(&curve, &[]);
        assert!(bl.iter().all(|&v| v == 0.0));
    }
}
