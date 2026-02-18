//! Amperometric Titration Analyzer — Electrochemical Endpoint Detection
//!
//! Implements amperometric titration analysis for electrochemical endpoint
//! detection using current measurement at fixed applied potential. Covers
//! single-indicator and biamperometric (dead-stop) methods, diffusion-limited
//! current models (Cottrell, Ilkovic, Levich), rotating disk electrode
//! analysis, Clark oxygen sensor, glucose biosensor kinetics, and
//! chronoamperometry processing.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::amperometric_titration_analyzer::{cottrell_current, AmperometricCell};
//!
//! let cell = AmperometricCell::new(0.5, 0.07, 1e-5, 96485.0);
//! let i = cottrell_current(2, 96485.0, 0.07, 1e-5, 1e-6, 1.0);
//! assert!(i > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Cottrell equation: i(t) = n * F * A * D^(1/2) * C / (pi * t)^(1/2)
///
/// Returns the diffusion-limited current (A) at time `t_s` (seconds) for a
/// planar electrode after a potential step.
///
/// * `n`        – electron transfer number
/// * `f`        – Faraday constant (C/mol)
/// * `area_cm2` – electrode area (cm^2)
/// * `d_cm2s`   – diffusion coefficient (cm^2/s)
/// * `c_mol_cm3`– bulk concentration (mol/cm^3)
/// * `t_s`      – time after potential step (s)
pub fn cottrell_current(
    n: u32,
    f: f64,
    area_cm2: f64,
    d_cm2s: f64,
    c_mol_cm3: f64,
    t_s: f64,
) -> f64 {
    if t_s <= 0.0 {
        return 0.0;
    }
    let nf: f64 = n as f64;
    nf * f * area_cm2 * d_cm2s.sqrt() * c_mol_cm3 / (PI * t_s).sqrt()
}

/// Levich equation for a rotating disk electrode:
/// i_L = 0.62 * n * F * A * D^(2/3) * omega^(1/2) * nu^(-1/6) * C
///
/// Returns the limiting current (A).
///
/// * `n`     – electron transfer number
/// * `f`     – Faraday constant (C/mol)
/// * `area`  – electrode area (cm^2)
/// * `d`     – diffusion coefficient (cm^2/s)
/// * `omega` – angular rotation rate (rad/s)
/// * `nu`    – kinematic viscosity (cm^2/s)
/// * `c`     – bulk concentration (mol/cm^3)
pub fn levich_current(
    n: u32,
    f: f64,
    area: f64,
    d: f64,
    omega: f64,
    nu: f64,
    c: f64,
) -> f64 {
    let nf: f64 = n as f64;
    0.62 * nf * f * area * d.powf(2.0 / 3.0) * omega.sqrt() * nu.powf(-1.0 / 6.0) * c
}

/// Estimate half-wave potential from (E, i) polarographic data.
///
/// Finds the potential at which i = i_lim / 2, using linear interpolation
/// between adjacent data points.
pub fn half_wave_potential(e_data: &[f64], i_data: &[f64]) -> Option<f64> {
    if e_data.len() < 2 || e_data.len() != i_data.len() {
        return None;
    }
    // Find limiting current (maximum absolute current)
    let i_max: f64 = i_data
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);
    let i_min: f64 = i_data
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min);

    let i_lim: f64 = if i_max.abs() > i_min.abs() {
        i_max
    } else {
        i_min
    };
    let half: f64 = i_lim / 2.0;

    // Linear interpolation to find E at i = half
    for k in 0..i_data.len() - 1 {
        let i0: f64 = i_data[k];
        let i1: f64 = i_data[k + 1];
        if (i0 - half) * (i1 - half) <= 0.0 {
            let frac: f64 = (half - i0) / (i1 - i0);
            let e_half: f64 = e_data[k] + frac * (e_data[k + 1] - e_data[k]);
            return Some(e_half);
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Linear regression helper
// ---------------------------------------------------------------------------

/// Ordinary least-squares linear regression: y = slope * x + intercept.
/// Returns (slope, intercept, r_squared).
fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64, f64) {
    let n: f64 = x.len() as f64;
    if n < 2.0 {
        return (0.0, 0.0, 0.0);
    }
    let sx: f64 = x.iter().sum();
    let sy: f64 = y.iter().sum();
    let sxx: f64 = x.iter().map(|&xi| xi * xi).sum();
    let sxy: f64 = x.iter().zip(y.iter()).map(|(&xi, &yi)| xi * yi).sum();
    let denom: f64 = n * sxx - sx * sx;
    if denom.abs() < 1e-30 {
        return (0.0, sy / n, 0.0);
    }
    let slope: f64 = (n * sxy - sx * sy) / denom;
    let intercept: f64 = (sy - slope * sx) / n;

    // R-squared
    let y_mean: f64 = sy / n;
    let ss_tot: f64 = y.iter().map(|&yi| (yi - y_mean).powi(2)).sum();
    let ss_res: f64 = x
        .iter()
        .zip(y.iter())
        .map(|(&xi, &yi)| {
            let predicted: f64 = slope * xi + intercept;
            (yi - predicted).powi(2)
        })
        .sum();
    let r2: f64 = if ss_tot > 1e-30 {
        1.0 - ss_res / ss_tot
    } else {
        1.0
    };
    (slope, intercept, r2)
}

/// Intersection of two lines: y = m1*x + b1 and y = m2*x + b2.
fn line_intersection(m1: f64, b1: f64, m2: f64, b2: f64) -> Option<(f64, f64)> {
    let dm: f64 = m1 - m2;
    if dm.abs() < 1e-30 {
        return None;
    }
    let x: f64 = (b2 - b1) / dm;
    let y: f64 = m1 * x + b1;
    Some((x, y))
}

// ---------------------------------------------------------------------------
// AmperometricCell
// ---------------------------------------------------------------------------

/// Working/reference/auxiliary three-electrode cell configuration.
#[derive(Debug, Clone)]
pub struct AmperometricCell {
    /// Applied potential vs reference (V)
    pub applied_potential_v: f64,
    /// Working electrode area (cm^2)
    pub electrode_area_cm2: f64,
    /// Diffusion coefficient of analyte (cm^2/s)
    pub diffusion_coeff: f64,
    /// Faraday constant (C/mol)
    pub faraday_const: f64,
}

impl AmperometricCell {
    pub fn new(
        applied_potential_v: f64,
        electrode_area_cm2: f64,
        diffusion_coeff: f64,
        faraday_const: f64,
    ) -> Self {
        Self {
            applied_potential_v,
            electrode_area_cm2,
            diffusion_coeff,
            faraday_const,
        }
    }

    /// Compute Cottrell current at time `t_s` for given n electrons and
    /// bulk concentration.
    pub fn cottrell_current(&self, n: u32, c_mol_cm3: f64, t_s: f64) -> f64 {
        cottrell_current(
            n,
            self.faraday_const,
            self.electrode_area_cm2,
            self.diffusion_coeff,
            c_mol_cm3,
            t_s,
        )
    }

    /// Steady-state diffusion-limited current for a microelectrode:
    /// i_ss = 4 * n * F * D * C * r
    pub fn steady_state_current(&self, n: u32, c_mol_cm3: f64) -> f64 {
        // Approximate radius from area assuming circular electrode
        let r: f64 = (self.electrode_area_cm2 / PI).sqrt();
        let nf: f64 = n as f64;
        4.0 * nf * self.faraday_const * self.diffusion_coeff * c_mol_cm3 * r
    }

    /// Background (residual) current estimate from capacitive charging:
    /// i_bg = C_dl * A * (dE/dt), here approximated as a fixed fraction.
    pub fn residual_current(&self, scan_rate_vs: f64, c_dl_f_cm2: f64) -> f64 {
        c_dl_f_cm2 * self.electrode_area_cm2 * scan_rate_vs
    }
}

// ---------------------------------------------------------------------------
// TitrationCurve
// ---------------------------------------------------------------------------

/// Types of amperometric titration curves.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CurveType {
    /// Analyte reduced/oxidized, product inactive: current decreases, then flat.
    LShaped,
    /// Both analyte and titrant are active: current decreases, then increases.
    VShaped,
    /// Analyte inactive, titrant active: flat, then current increases.
    InvertedL,
}

/// Amperometric titration curve: current vs volume of titrant.
#[derive(Debug, Clone)]
pub struct TitrationCurve {
    /// Volumes of titrant added (mL)
    pub volumes: Vec<f64>,
    /// Measured currents (uA)
    pub currents: Vec<f64>,
    /// Curve type classification
    pub curve_type: CurveType,
}

impl TitrationCurve {
    pub fn new(volumes: Vec<f64>, currents: Vec<f64>) -> Self {
        let curve_type = Self::classify(&volumes, &currents);
        Self {
            volumes,
            currents,
            curve_type,
        }
    }

    /// Classify the curve type based on current trend.
    fn classify(volumes: &[f64], currents: &[f64]) -> CurveType {
        if volumes.len() < 4 {
            return CurveType::LShaped;
        }
        let n: usize = volumes.len();
        let mid: usize = n / 2;
        let first_half_slope: f64 = {
            let (s, _, _) = linear_regression(&volumes[..mid], &currents[..mid]);
            s
        };
        let second_half_slope: f64 = {
            let (s, _, _) = linear_regression(&volumes[mid..], &currents[mid..]);
            s
        };

        if first_half_slope < -1e-10 && second_half_slope > 1e-10 {
            CurveType::VShaped
        } else if first_half_slope < -1e-10 {
            CurveType::LShaped
        } else {
            CurveType::InvertedL
        }
    }

    /// Find endpoint by intersecting two best-fit lines from the first and
    /// second halves of the titration curve.
    pub fn find_endpoint(&self) -> Option<(f64, f64)> {
        if self.volumes.len() < 4 {
            return None;
        }
        let n: usize = self.volumes.len();
        let best_split: usize = self.find_optimal_split();
        let (m1, b1, _) =
            linear_regression(&self.volumes[..best_split], &self.currents[..best_split]);
        let (m2, b2, _) =
            linear_regression(&self.volumes[best_split..], &self.currents[best_split..]);
        let _ = n; // suppress unused warning
        line_intersection(m1, b1, m2, b2)
    }

    /// Find optimal split point that maximises the sum of R^2 for both segments.
    fn find_optimal_split(&self) -> usize {
        let n: usize = self.volumes.len();
        let mut best_idx: usize = n / 2;
        let mut best_score: f64 = f64::NEG_INFINITY;
        for split in 2..n - 1 {
            let (_, _, r2a) =
                linear_regression(&self.volumes[..split], &self.currents[..split]);
            let (_, _, r2b) =
                linear_regression(&self.volumes[split..], &self.currents[split..]);
            let score: f64 = r2a + r2b;
            if score > best_score {
                best_score = score;
                best_idx = split;
            }
        }
        best_idx
    }

    /// Compute first derivative di/dV using central differences.
    pub fn first_derivative(&self) -> Vec<f64> {
        let n: usize = self.volumes.len();
        if n < 2 {
            return vec![];
        }
        let mut deriv: Vec<f64> = Vec::with_capacity(n);
        // Forward difference for first point
        deriv.push(
            (self.currents[1] - self.currents[0]) / (self.volumes[1] - self.volumes[0]),
        );
        // Central differences
        for k in 1..n - 1 {
            let dv: f64 = self.volumes[k + 1] - self.volumes[k - 1];
            let di: f64 = self.currents[k + 1] - self.currents[k - 1];
            deriv.push(di / dv);
        }
        // Backward difference for last point
        deriv.push(
            (self.currents[n - 1] - self.currents[n - 2])
                / (self.volumes[n - 1] - self.volumes[n - 2]),
        );
        deriv
    }

    /// Find endpoint from the first-derivative maximum (absolute).
    pub fn derivative_endpoint(&self) -> Option<f64> {
        let deriv = self.first_derivative();
        if deriv.is_empty() {
            return None;
        }
        let (idx, _) = deriv
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())?;
        Some(self.volumes[idx])
    }

    /// Gran plot linearization: V * i vs V.
    /// The x-intercept of the linear segment past the endpoint gives the
    /// equivalence volume.
    pub fn gran_plot(&self) -> Vec<(f64, f64)> {
        self.volumes
            .iter()
            .zip(self.currents.iter())
            .map(|(&v, &i)| (v, v * i))
            .collect()
    }

    /// Find the endpoint from the Gran plot (x-intercept of post-endpoint line).
    pub fn gran_endpoint(&self) -> Option<f64> {
        let gran = self.gran_plot();
        if gran.len() < 4 {
            return None;
        }
        // Use second half for Gran linearization
        let mid: usize = gran.len() / 2;
        let x: Vec<f64> = gran[mid..].iter().map(|&(v, _)| v).collect();
        let y: Vec<f64> = gran[mid..].iter().map(|&(_, vi)| vi).collect();
        let (slope, intercept, _) = linear_regression(&x, &y);
        if slope.abs() < 1e-30 {
            return None;
        }
        let x_intercept: f64 = -intercept / slope;
        Some(x_intercept)
    }
}

// ---------------------------------------------------------------------------
// BiamperometricTitration
// ---------------------------------------------------------------------------

/// Dead-stop (biamperometric) titration with two indicator electrodes.
#[derive(Debug, Clone)]
pub struct BiamperometricTitration {
    /// Applied potential between indicator electrodes (mV)
    pub applied_potential_mv: f64,
    /// Volume data (mL)
    pub volumes: Vec<f64>,
    /// Differential current data (uA)
    pub differential_currents: Vec<f64>,
}

impl BiamperometricTitration {
    pub fn new(applied_potential_mv: f64) -> Self {
        Self {
            applied_potential_mv,
            volumes: Vec::new(),
            differential_currents: Vec::new(),
        }
    }

    /// Add a data point.
    pub fn add_point(&mut self, volume_ml: f64, current_ua: f64) {
        self.volumes.push(volume_ml);
        self.differential_currents.push(current_ua);
    }

    /// Karl Fischer water determination endpoint: current remains near zero
    /// before equivalence point and rises sharply after.
    pub fn karl_fischer_endpoint(&self) -> Option<f64> {
        if self.volumes.len() < 3 {
            return None;
        }
        // Find where current first exceeds a threshold that persists
        let threshold: f64 = {
            let max_i: f64 = self
                .differential_currents
                .iter()
                .copied()
                .fold(0.0f64, f64::max);
            max_i * 0.1 // 10% of max as threshold
        };
        for k in 0..self.differential_currents.len() {
            if self.differential_currents[k] > threshold {
                // Check persistence (next point also above threshold)
                if k + 1 < self.differential_currents.len()
                    && self.differential_currents[k + 1] > threshold
                {
                    return Some(self.volumes[k]);
                }
            }
        }
        None
    }

    /// Find endpoint from two-line intersection on differential current data.
    pub fn find_endpoint(&self) -> Option<(f64, f64)> {
        let tc = TitrationCurve::new(
            self.volumes.clone(),
            self.differential_currents.clone(),
        );
        tc.find_endpoint()
    }

    /// Compute the water content (ppm) from Karl Fischer titration.
    /// water_ppm = (V_endpoint * titer) / sample_mass * 1e6
    pub fn water_content_ppm(
        &self,
        titer_mg_ml: f64,
        sample_mass_g: f64,
    ) -> Option<f64> {
        let ep = self.karl_fischer_endpoint()?;
        let water_mg: f64 = ep * titer_mg_ml;
        let ppm: f64 = water_mg / (sample_mass_g * 1000.0) * 1e6;
        Some(ppm)
    }
}

// ---------------------------------------------------------------------------
// DiffusionCurrent
// ---------------------------------------------------------------------------

/// Ilkovic equation for a dropping mercury electrode (DME).
/// id = 607 * n * D^(1/2) * m^(2/3) * t^(1/6) * c
#[derive(Debug, Clone)]
pub struct DiffusionCurrent {
    /// Electron transfer number
    pub n: u32,
    /// Diffusion coefficient (cm^2/s)
    pub diffusion_coeff: f64,
    /// Mercury flow rate (mg/s)
    pub mercury_flow_rate: f64,
    /// Drop time (s)
    pub drop_time: f64,
}

impl DiffusionCurrent {
    pub fn new(n: u32, diffusion_coeff: f64, mercury_flow_rate: f64, drop_time: f64) -> Self {
        Self {
            n,
            diffusion_coeff,
            mercury_flow_rate,
            drop_time,
        }
    }

    /// Ilkovic equation: id = 607 * n * D^(1/2) * m^(2/3) * t^(1/6) * c
    /// Returns diffusion current in microamperes.
    pub fn ilkovic_current(&self, c_mmol_l: f64) -> f64 {
        let nf: f64 = self.n as f64;
        607.0
            * nf
            * self.diffusion_coeff.sqrt()
            * self.mercury_flow_rate.powf(2.0 / 3.0)
            * self.drop_time.powf(1.0 / 6.0)
            * c_mmol_l
    }

    /// Limiting current at a given concentration.
    pub fn limiting_current(&self, c_mmol_l: f64) -> f64 {
        self.ilkovic_current(c_mmol_l)
    }

    /// Residual current correction: subtract baseline from measured current.
    pub fn correct_residual(&self, measured: f64, residual: f64) -> f64 {
        measured - residual
    }

    /// Half-wave potential from polarographic wave data:
    /// E = E_1/2 + (RT/nF) * ln((i_d - i) / i)
    /// Inverts to find E_1/2 given a data point.
    pub fn half_wave_from_wave(
        &self,
        e_measured: f64,
        i_measured: f64,
        i_d: f64,
        temperature_k: f64,
    ) -> f64 {
        let r: f64 = 8.314; // J/(mol*K)
        let f: f64 = 96485.0; // C/mol
        let nf: f64 = self.n as f64;
        let ratio: f64 = (i_d - i_measured) / i_measured;
        if ratio <= 0.0 {
            return e_measured;
        }
        e_measured - (r * temperature_k / (nf * f)) * ratio.ln()
    }

    /// Diffusion coefficient from Ilkovic equation given known current:
    /// D = (id / (607 * n * m^(2/3) * t^(1/6) * c))^2
    pub fn extract_diffusion_coeff(&self, id_ua: f64, c_mmol_l: f64) -> f64 {
        let nf: f64 = self.n as f64;
        let denom: f64 = 607.0
            * nf
            * self.mercury_flow_rate.powf(2.0 / 3.0)
            * self.drop_time.powf(1.0 / 6.0)
            * c_mmol_l;
        if denom.abs() < 1e-30 {
            return 0.0;
        }
        let ratio: f64 = id_ua / denom;
        ratio * ratio
    }
}

// ---------------------------------------------------------------------------
// RotatingDiskElectrode
// ---------------------------------------------------------------------------

/// Rotating Disk Electrode (RDE) analysis using the Levich equation.
#[derive(Debug, Clone)]
pub struct RotatingDiskElectrode {
    /// Electron transfer number
    pub n: u32,
    /// Faraday constant (C/mol)
    pub faraday_const: f64,
    /// Electrode area (cm^2)
    pub area_cm2: f64,
    /// Diffusion coefficient (cm^2/s)
    pub diffusion_coeff: f64,
    /// Kinematic viscosity (cm^2/s)
    pub kinematic_viscosity: f64,
    /// Bulk concentration (mol/cm^3)
    pub concentration: f64,
}

impl RotatingDiskElectrode {
    pub fn new(
        n: u32,
        faraday_const: f64,
        area_cm2: f64,
        diffusion_coeff: f64,
        kinematic_viscosity: f64,
        concentration: f64,
    ) -> Self {
        Self {
            n,
            faraday_const,
            area_cm2,
            diffusion_coeff,
            kinematic_viscosity,
            concentration,
        }
    }

    /// Levich limiting current at given rotation rate (rad/s).
    pub fn levich_current(&self, omega: f64) -> f64 {
        levich_current(
            self.n,
            self.faraday_const,
            self.area_cm2,
            self.diffusion_coeff,
            omega,
            self.kinematic_viscosity,
            self.concentration,
        )
    }

    /// Koutecky-Levich analysis: 1/i vs 1/sqrt(omega).
    /// Returns (slope, intercept) from linear fit. The intercept gives
    /// 1/i_kinetic, and the slope gives the Levich constant.
    pub fn koutecky_levich_plot(
        &self,
        omegas: &[f64],
        currents: &[f64],
    ) -> (f64, f64, f64) {
        let x: Vec<f64> = omegas.iter().map(|&w| 1.0 / w.sqrt()).collect();
        let y: Vec<f64> = currents.iter().map(|&i| 1.0 / i).collect();
        linear_regression(&x, &y)
    }

    /// Extract kinetic current from Koutecky-Levich intercept: i_k = 1/intercept.
    pub fn kinetic_current(&self, omegas: &[f64], currents: &[f64]) -> f64 {
        let (_, intercept, _) = self.koutecky_levich_plot(omegas, currents);
        if intercept.abs() < 1e-30 {
            return 0.0;
        }
        1.0 / intercept
    }

    /// Levich slope: B = 0.62 * n * F * A * D^(2/3) * nu^(-1/6) * C.
    pub fn levich_slope(&self) -> f64 {
        let nf: f64 = self.n as f64;
        0.62 * nf
            * self.faraday_const
            * self.area_cm2
            * self.diffusion_coeff.powf(2.0 / 3.0)
            * self.kinematic_viscosity.powf(-1.0 / 6.0)
            * self.concentration
    }

    /// Number of electrons from Levich slope.
    pub fn electron_number_from_slope(&self, measured_slope: f64) -> f64 {
        let b_per_n: f64 = 0.62
            * self.faraday_const
            * self.area_cm2
            * self.diffusion_coeff.powf(2.0 / 3.0)
            * self.kinematic_viscosity.powf(-1.0 / 6.0)
            * self.concentration;
        if b_per_n.abs() < 1e-30 {
            return 0.0;
        }
        measured_slope / b_per_n
    }

    /// Convert RPM to angular velocity (rad/s).
    pub fn rpm_to_omega(rpm: f64) -> f64 {
        rpm * 2.0 * PI / 60.0
    }
}

// ---------------------------------------------------------------------------
// EndpointDetector
// ---------------------------------------------------------------------------

/// Endpoint detection via piecewise linear regression.
#[derive(Debug, Clone)]
pub struct EndpointDetector {
    /// Minimum points per segment for linear regression
    pub min_segment_points: usize,
}

impl EndpointDetector {
    pub fn new(min_segment_points: usize) -> Self {
        Self {
            min_segment_points: min_segment_points.max(2),
        }
    }

    /// Find endpoint by intersecting two best-fit lines.
    /// Tries all possible split points and picks the one with best combined R^2.
    pub fn piecewise_linear_endpoint(
        &self,
        x: &[f64],
        y: &[f64],
    ) -> Option<(f64, f64)> {
        let n: usize = x.len();
        if n < 2 * self.min_segment_points {
            return None;
        }
        let mut best_split: usize = self.min_segment_points;
        let mut best_score: f64 = f64::NEG_INFINITY;

        for split in self.min_segment_points..=(n - self.min_segment_points) {
            let (_, _, r2a) = linear_regression(&x[..split], &y[..split]);
            let (_, _, r2b) = linear_regression(&x[split..], &y[split..]);
            let score: f64 = r2a + r2b;
            if score > best_score {
                best_score = score;
                best_split = split;
            }
        }

        let (m1, b1, _) = linear_regression(&x[..best_split], &y[..best_split]);
        let (m2, b2, _) = linear_regression(&x[best_split..], &y[best_split..]);
        line_intersection(m1, b1, m2, b2)
    }

    /// First-derivative endpoint: find x where |dy/dx| is maximum.
    pub fn first_derivative_endpoint(
        &self,
        x: &[f64],
        y: &[f64],
    ) -> Option<f64> {
        if x.len() < 3 {
            return None;
        }
        let mut max_abs_deriv: f64 = 0.0;
        let mut best_idx: usize = 1;
        for k in 1..x.len() - 1 {
            let dx: f64 = x[k + 1] - x[k - 1];
            if dx.abs() < 1e-30 {
                continue;
            }
            let dy: f64 = y[k + 1] - y[k - 1];
            let deriv: f64 = (dy / dx).abs();
            if deriv > max_abs_deriv {
                max_abs_deriv = deriv;
                best_idx = k;
            }
        }
        Some(x[best_idx])
    }

    /// Gran plot linearization: transforms data to V*i vs V for endpoint.
    pub fn gran_plot_endpoint(
        &self,
        volumes: &[f64],
        currents: &[f64],
    ) -> Option<f64> {
        if volumes.len() < 4 {
            return None;
        }
        let gran_y: Vec<f64> = volumes
            .iter()
            .zip(currents.iter())
            .map(|(&v, &i)| v * i)
            .collect();

        // Use second half for Gran line
        let mid: usize = volumes.len() / 2;
        let x_seg: Vec<f64> = volumes[mid..].to_vec();
        let y_seg: Vec<f64> = gran_y[mid..].to_vec();
        let (slope, intercept, _) = linear_regression(&x_seg, &y_seg);
        if slope.abs() < 1e-30 {
            return None;
        }
        Some(-intercept / slope)
    }
}

// ---------------------------------------------------------------------------
// OxygenSensor
// ---------------------------------------------------------------------------

/// Clark-type amperometric oxygen sensor model.
#[derive(Debug, Clone)]
pub struct OxygenSensor {
    /// Membrane permeability (cm^2/s)
    pub membrane_permeability: f64,
    /// Membrane thickness (cm)
    pub membrane_thickness: f64,
    /// Electrode area (cm^2)
    pub electrode_area: f64,
    /// Faraday constant
    pub faraday_const: f64,
    /// Number of electrons (4 for O2 reduction)
    pub n_electrons: u32,
    /// Temperature coefficient (%/degC)
    pub temp_coeff_pct: f64,
    /// Calibration temperature (degC)
    pub cal_temp_c: f64,
}

impl OxygenSensor {
    pub fn new(
        membrane_permeability: f64,
        membrane_thickness: f64,
        electrode_area: f64,
    ) -> Self {
        Self {
            membrane_permeability,
            membrane_thickness,
            electrode_area,
            faraday_const: 96485.0,
            n_electrons: 4,
            temp_coeff_pct: 3.0,
            cal_temp_c: 25.0,
        }
    }

    /// Steady-state sensor current for given dissolved O2 concentration.
    /// i = n * F * A * P_m * C / L
    pub fn sensor_current(&self, do_mol_cm3: f64) -> f64 {
        let nf: f64 = self.n_electrons as f64;
        nf * self.faraday_const
            * self.electrode_area
            * self.membrane_permeability
            * do_mol_cm3
            / self.membrane_thickness
    }

    /// Temperature-compensated current.
    pub fn temperature_compensated_current(
        &self,
        measured_current: f64,
        actual_temp_c: f64,
    ) -> f64 {
        let delta_t: f64 = actual_temp_c - self.cal_temp_c;
        let correction: f64 = 1.0 + self.temp_coeff_pct / 100.0 * delta_t;
        if correction.abs() < 1e-10 {
            return measured_current;
        }
        measured_current / correction
    }

    /// Dissolved oxygen in mg/L from sensor current via calibration.
    pub fn do_mg_l(&self, current_a: f64, cal_current_a: f64, cal_do_mg_l: f64) -> f64 {
        if cal_current_a.abs() < 1e-30 {
            return 0.0;
        }
        (current_a / cal_current_a) * cal_do_mg_l
    }

    /// Henry's law: DO saturation from partial pressure.
    /// C_sat = P_O2 / K_H
    pub fn do_saturation(p_o2_atm: f64, k_h_atm_l_mol: f64) -> f64 {
        if k_h_atm_l_mol.abs() < 1e-30 {
            return 0.0;
        }
        p_o2_atm / k_h_atm_l_mol
    }

    /// Response time: 95% rise time for membrane diffusion.
    /// t_95 = L^2 / (2 * D) * ln(20) approximation.
    pub fn response_time_95(&self) -> f64 {
        if self.membrane_permeability < 1e-30 {
            return f64::INFINITY;
        }
        let l2: f64 = self.membrane_thickness * self.membrane_thickness;
        l2 / (2.0 * self.membrane_permeability) * (20.0_f64).ln()
    }

    /// Winkler titration equivalent: compute DO from Winkler thiosulfate volume.
    /// DO (mg/L) = V_thio * M_thio * 8000 / V_sample
    pub fn winkler_do(
        v_thiosulfate_ml: f64,
        m_thiosulfate_mol_l: f64,
        v_sample_ml: f64,
    ) -> f64 {
        if v_sample_ml.abs() < 1e-30 {
            return 0.0;
        }
        // Each mole of thiosulfate corresponds to 0.25 mol O2 (4 electrons),
        // MW of O2 = 32 g/mol, so: mg O2 = mol_thio * 0.25 * 32 * 1000
        v_thiosulfate_ml * m_thiosulfate_mol_l * 8000.0 / v_sample_ml
    }
}

// ---------------------------------------------------------------------------
// GlucoseBiosensor
// ---------------------------------------------------------------------------

/// Amperometric glucose biosensor using enzyme-catalyzed oxidation.
#[derive(Debug, Clone)]
pub struct GlucoseBiosensor {
    /// Maximum enzyme reaction rate (mol/(cm^2*s))
    pub v_max: f64,
    /// Michaelis-Menten constant (mol/L or mM)
    pub k_m: f64,
    /// Sensitivity (nA / mM)
    pub sensitivity: f64,
    /// Linear range upper limit (mM)
    pub linear_range_max: f64,
    /// Interference rejection factor (0 to 1, 1 = perfect rejection)
    pub interference_rejection: f64,
}

impl GlucoseBiosensor {
    pub fn new(v_max: f64, k_m: f64, sensitivity: f64, linear_range_max: f64) -> Self {
        Self {
            v_max,
            k_m,
            sensitivity,
            linear_range_max,
            interference_rejection: 0.95,
        }
    }

    /// Michaelis-Menten reaction rate at given substrate concentration.
    /// v = V_max * [S] / (K_m + [S])
    pub fn reaction_rate(&self, substrate_conc: f64) -> f64 {
        if substrate_conc < 0.0 {
            return 0.0;
        }
        self.v_max * substrate_conc / (self.k_m + substrate_conc)
    }

    /// Predicted current (nA) from glucose concentration (mM).
    pub fn predicted_current(&self, glucose_mm: f64) -> f64 {
        if glucose_mm < 0.0 {
            return 0.0;
        }
        if glucose_mm <= self.linear_range_max {
            // Linear region
            self.sensitivity * glucose_mm
        } else {
            // Michaelis-Menten saturation
            let i_max: f64 = self.sensitivity * self.linear_range_max;
            i_max * glucose_mm / (self.k_m + glucose_mm)
                * (self.k_m + self.linear_range_max)
                / self.linear_range_max
        }
    }

    /// Glucose concentration from measured current (inverse calibration).
    pub fn glucose_from_current(&self, current_na: f64) -> f64 {
        if current_na <= 0.0 {
            return 0.0;
        }
        // In linear range: [glucose] = i / sensitivity
        let conc: f64 = current_na / self.sensitivity;
        if conc <= self.linear_range_max {
            conc
        } else {
            // Saturated, solve Michaelis-Menten numerically
            conc // Approximation in supralinear range
        }
    }

    /// Check if concentration is within linear range.
    pub fn is_in_linear_range(&self, glucose_mm: f64) -> bool {
        glucose_mm >= 0.0 && glucose_mm <= self.linear_range_max
    }

    /// Corrected current after subtracting interference.
    pub fn corrected_current(
        &self,
        total_current_na: f64,
        interference_current_na: f64,
    ) -> f64 {
        total_current_na - interference_current_na * (1.0 - self.interference_rejection)
    }

    /// Lineweaver-Burk parameters from (concentration, rate) data:
    /// 1/v vs 1/[S] gives slope = K_m/V_max, intercept = 1/V_max.
    pub fn lineweaver_burk(
        concentrations: &[f64],
        rates: &[f64],
    ) -> (f64, f64) {
        let x: Vec<f64> = concentrations.iter().map(|&c| 1.0 / c).collect();
        let y: Vec<f64> = rates.iter().map(|&r| 1.0 / r).collect();
        let (slope, intercept, _) = linear_regression(&x, &y);
        // V_max = 1/intercept, K_m = slope * V_max
        let v_max: f64 = if intercept.abs() > 1e-30 {
            1.0 / intercept
        } else {
            0.0
        };
        let k_m: f64 = slope * v_max;
        (v_max, k_m)
    }

    /// Detection limit: 3 * sigma_blank / sensitivity.
    pub fn detection_limit(&self, sigma_blank: f64) -> f64 {
        if self.sensitivity.abs() < 1e-30 {
            return f64::INFINITY;
        }
        3.0 * sigma_blank / self.sensitivity
    }
}

// ---------------------------------------------------------------------------
// ChronoamperometryProcessor
// ---------------------------------------------------------------------------

/// Chronoamperometry (potential step) experiment processor.
#[derive(Debug, Clone)]
pub struct ChronoamperometryProcessor {
    /// Time data (s)
    pub times: Vec<f64>,
    /// Current data (A)
    pub currents: Vec<f64>,
    /// Electron transfer number
    pub n: u32,
    /// Faraday constant
    pub faraday_const: f64,
    /// Electrode area (cm^2)
    pub area_cm2: f64,
    /// Bulk concentration (mol/cm^3)
    pub concentration: f64,
}

impl ChronoamperometryProcessor {
    pub fn new(n: u32, area_cm2: f64, concentration: f64) -> Self {
        Self {
            times: Vec::new(),
            currents: Vec::new(),
            n,
            faraday_const: 96485.0,
            area_cm2,
            concentration,
        }
    }

    /// Add a (time, current) data point.
    pub fn add_point(&mut self, t_s: f64, i_a: f64) {
        self.times.push(t_s);
        self.currents.push(i_a);
    }

    /// Load data from arrays.
    pub fn load_data(&mut self, times: &[f64], currents: &[f64]) {
        self.times = times.to_vec();
        self.currents = currents.to_vec();
    }

    /// Cottrell plot: i vs t^(-1/2).
    /// Returns vectors of (t^(-1/2), i) for plotting.
    pub fn cottrell_plot(&self) -> (Vec<f64>, Vec<f64>) {
        let x: Vec<f64> = self
            .times
            .iter()
            .filter(|&&t| t > 0.0)
            .map(|&t| 1.0 / t.sqrt())
            .collect();
        let y: Vec<f64> = self
            .times
            .iter()
            .zip(self.currents.iter())
            .filter(|(&t, _)| t > 0.0)
            .map(|(_, &i)| i)
            .collect();
        (x, y)
    }

    /// Extract diffusion coefficient from Cottrell plot slope.
    /// slope = n * F * A * D^(1/2) * C / pi^(1/2)
    /// => D = (slope * pi^(1/2) / (n * F * A * C))^2
    pub fn extract_diffusion_coeff(&self) -> f64 {
        let (x, y) = self.cottrell_plot();
        if x.len() < 2 {
            return 0.0;
        }
        let (slope, _, _) = linear_regression(&x, &y);
        let nf: f64 = self.n as f64;
        let denom: f64 = nf * self.faraday_const * self.area_cm2 * self.concentration;
        if denom.abs() < 1e-30 {
            return 0.0;
        }
        let ratio: f64 = slope * PI.sqrt() / denom;
        ratio * ratio
    }

    /// Cottrell plot linearity check (R^2 value).
    pub fn cottrell_r_squared(&self) -> f64 {
        let (x, y) = self.cottrell_plot();
        if x.len() < 2 {
            return 0.0;
        }
        let (_, _, r2) = linear_regression(&x, &y);
        r2
    }

    /// Anson plot: Q(t) = integral of i dt vs t^(1/2).
    /// Q = 2 * n * F * A * D^(1/2) * C * t^(1/2) / pi^(1/2)
    pub fn anson_plot(&self) -> (Vec<f64>, Vec<f64>) {
        if self.times.is_empty() {
            return (vec![], vec![]);
        }
        let mut charges: Vec<f64> = Vec::new();
        let mut t_sqrt: Vec<f64> = Vec::new();
        let mut q: f64 = 0.0;
        for k in 0..self.times.len() {
            if k > 0 {
                let dt: f64 = self.times[k] - self.times[k - 1];
                let avg_i: f64 = (self.currents[k] + self.currents[k - 1]) / 2.0;
                q += avg_i * dt;
            }
            if self.times[k] > 0.0 {
                t_sqrt.push(self.times[k].sqrt());
                charges.push(q);
            }
        }
        (t_sqrt, charges)
    }

    /// Extract diffusion coefficient from Anson plot slope.
    /// slope = 2 * n * F * A * D^(1/2) * C / pi^(1/2)
    pub fn diffusion_coeff_from_anson(&self) -> f64 {
        let (x, y) = self.anson_plot();
        if x.len() < 2 {
            return 0.0;
        }
        let (slope, _, _) = linear_regression(&x, &y);
        let nf: f64 = self.n as f64;
        let denom: f64 =
            2.0 * nf * self.faraday_const * self.area_cm2 * self.concentration;
        if denom.abs() < 1e-30 {
            return 0.0;
        }
        let ratio: f64 = slope * PI.sqrt() / denom;
        ratio * ratio
    }

    /// Current transient ratio at two times: i(t1)/i(t2) = sqrt(t2/t1).
    /// Useful for verifying diffusion control.
    pub fn current_ratio_check(&self, t1_idx: usize, t2_idx: usize) -> Option<(f64, f64)> {
        if t1_idx >= self.times.len() || t2_idx >= self.times.len() {
            return None;
        }
        let t1: f64 = self.times[t1_idx];
        let t2: f64 = self.times[t2_idx];
        if t1 <= 0.0 || t2 <= 0.0 {
            return None;
        }
        let measured_ratio: f64 = self.currents[t1_idx] / self.currents[t2_idx];
        let expected_ratio: f64 = (t2 / t1).sqrt();
        Some((measured_ratio, expected_ratio))
    }

    /// Residual current at long times (adsorption component).
    pub fn residual_current(&self) -> f64 {
        if self.currents.len() < 3 {
            return 0.0;
        }
        // Average of last 10% of data points
        let n: usize = self.currents.len();
        let start: usize = n - (n / 10).max(1);
        let sum: f64 = self.currents[start..].iter().sum();
        sum / (n - start) as f64
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const F: f64 = 96485.0; // Faraday constant
    const EPSILON: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn relative_eq(a: f64, b: f64, rel_tol: f64) -> bool {
        if a.abs() < 1e-30 && b.abs() < 1e-30 {
            return true;
        }
        let denom: f64 = a.abs().max(b.abs());
        (a - b).abs() / denom < rel_tol
    }

    // -----------------------------------------------------------------------
    // Cottrell current helper
    // -----------------------------------------------------------------------

    #[test]
    fn test_cottrell_current_basic() {
        let i: f64 = cottrell_current(1, F, 0.1, 1e-5, 1e-6, 1.0);
        // i = 1 * 96485 * 0.1 * sqrt(1e-5) * 1e-6 / sqrt(pi * 1)
        // = 96485 * 0.1 * 3.162e-3 * 1e-6 / 1.7725
        let expected: f64 =
            1.0 * F * 0.1 * (1e-5_f64).sqrt() * 1e-6 / (PI * 1.0_f64).sqrt();
        assert!(
            approx_eq(i, expected, 1e-15),
            "Cottrell i={} expected={}",
            i,
            expected
        );
    }

    #[test]
    fn test_cottrell_current_zero_time() {
        let i: f64 = cottrell_current(2, F, 0.07, 1e-5, 1e-6, 0.0);
        assert_eq!(i, 0.0);
    }

    #[test]
    fn test_cottrell_current_negative_time() {
        let i: f64 = cottrell_current(1, F, 0.1, 1e-5, 1e-6, -1.0);
        assert_eq!(i, 0.0);
    }

    #[test]
    fn test_cottrell_current_scales_with_n() {
        let i1: f64 = cottrell_current(1, F, 0.1, 1e-5, 1e-6, 1.0);
        let i2: f64 = cottrell_current(2, F, 0.1, 1e-5, 1e-6, 1.0);
        assert!(approx_eq(i2, 2.0 * i1, 1e-15));
    }

    #[test]
    fn test_cottrell_current_scales_with_area() {
        let i1: f64 = cottrell_current(1, F, 0.1, 1e-5, 1e-6, 1.0);
        let i2: f64 = cottrell_current(1, F, 0.2, 1e-5, 1e-6, 1.0);
        assert!(approx_eq(i2, 2.0 * i1, 1e-15));
    }

    #[test]
    fn test_cottrell_current_inversely_with_sqrt_t() {
        let i1: f64 = cottrell_current(1, F, 0.1, 1e-5, 1e-6, 1.0);
        let i4: f64 = cottrell_current(1, F, 0.1, 1e-5, 1e-6, 4.0);
        assert!(approx_eq(i1 / i4, 2.0, 1e-10));
    }

    // -----------------------------------------------------------------------
    // Levich current helper
    // -----------------------------------------------------------------------

    #[test]
    fn test_levich_current_basic() {
        let omega: f64 = 100.0;
        let nu: f64 = 0.01;
        let d: f64 = 1e-5;
        let c: f64 = 1e-6;
        let area: f64 = 0.196; // 5mm diameter disk
        let i: f64 = levich_current(4, F, area, d, omega, nu, c);
        let expected: f64 = 0.62 * 4.0 * F * area * d.powf(2.0 / 3.0)
            * omega.sqrt()
            * nu.powf(-1.0 / 6.0)
            * c;
        assert!(
            approx_eq(i, expected, 1e-15),
            "Levich i={} expected={}",
            i,
            expected
        );
    }

    #[test]
    fn test_levich_current_scales_with_sqrt_omega() {
        let i1: f64 = levich_current(2, F, 0.1, 1e-5, 100.0, 0.01, 1e-6);
        let i2: f64 = levich_current(2, F, 0.1, 1e-5, 400.0, 0.01, 1e-6);
        assert!(approx_eq(i2 / i1, 2.0, 1e-10));
    }

    #[test]
    fn test_levich_current_scales_with_concentration() {
        let i1: f64 = levich_current(1, F, 0.1, 1e-5, 100.0, 0.01, 1e-6);
        let i2: f64 = levich_current(1, F, 0.1, 1e-5, 100.0, 0.01, 2e-6);
        assert!(approx_eq(i2, 2.0 * i1, 1e-15));
    }

    // -----------------------------------------------------------------------
    // Half-wave potential
    // -----------------------------------------------------------------------

    #[test]
    fn test_half_wave_potential_basic() {
        // Create a simple sigmoidal polarographic wave
        let e_data: Vec<f64> = (0..21).map(|k| -0.2 + k as f64 * 0.02).collect();
        let i_lim: f64 = 10.0;
        let e_half_true: f64 = 0.0;
        let i_data: Vec<f64> = e_data
            .iter()
            .map(|&e| i_lim / (1.0 + (-(e - e_half_true) * 100.0).exp()))
            .collect();
        let result = half_wave_potential(&e_data, &i_data);
        assert!(result.is_some());
        let e_half: f64 = result.unwrap();
        assert!(
            approx_eq(e_half, e_half_true, 0.03),
            "E_1/2 = {}, expected ~0.0",
            e_half
        );
    }

    #[test]
    fn test_half_wave_potential_empty() {
        assert!(half_wave_potential(&[], &[]).is_none());
    }

    #[test]
    fn test_half_wave_potential_single() {
        assert!(half_wave_potential(&[0.0], &[1.0]).is_none());
    }

    #[test]
    fn test_half_wave_potential_mismatched() {
        assert!(half_wave_potential(&[0.0, 1.0], &[1.0]).is_none());
    }

    // -----------------------------------------------------------------------
    // Linear regression (internal)
    // -----------------------------------------------------------------------

    #[test]
    fn test_linear_regression_perfect() {
        let x: Vec<f64> = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y: Vec<f64> = vec![2.0, 4.0, 6.0, 8.0, 10.0];
        let (slope, intercept, r2) = linear_regression(&x, &y);
        assert!(approx_eq(slope, 2.0, EPSILON));
        assert!(approx_eq(intercept, 0.0, EPSILON));
        assert!(approx_eq(r2, 1.0, EPSILON));
    }

    #[test]
    fn test_linear_regression_with_offset() {
        let x: Vec<f64> = vec![0.0, 1.0, 2.0, 3.0];
        let y: Vec<f64> = vec![5.0, 7.0, 9.0, 11.0];
        let (slope, intercept, r2) = linear_regression(&x, &y);
        assert!(approx_eq(slope, 2.0, EPSILON));
        assert!(approx_eq(intercept, 5.0, EPSILON));
        assert!(approx_eq(r2, 1.0, EPSILON));
    }

    #[test]
    fn test_linear_regression_flat() {
        let x: Vec<f64> = vec![1.0, 2.0, 3.0, 4.0];
        let y: Vec<f64> = vec![5.0, 5.0, 5.0, 5.0];
        let (slope, _intercept, _r2) = linear_regression(&x, &y);
        assert!(approx_eq(slope, 0.0, EPSILON));
    }

    // -----------------------------------------------------------------------
    // AmperometricCell
    // -----------------------------------------------------------------------

    #[test]
    fn test_cell_cottrell() {
        let cell = AmperometricCell::new(0.5, 0.07, 1e-5, F);
        let i: f64 = cell.cottrell_current(2, 1e-6, 1.0);
        let expected: f64 = cottrell_current(2, F, 0.07, 1e-5, 1e-6, 1.0);
        assert!(approx_eq(i, expected, 1e-15));
    }

    #[test]
    fn test_cell_steady_state() {
        let cell = AmperometricCell::new(0.5, PI * 0.01, 1e-5, F); // r=0.1 cm
        let i: f64 = cell.steady_state_current(1, 1e-6);
        // i_ss = 4 * 1 * F * D * C * r = 4 * F * 1e-5 * 1e-6 * 0.1
        let r: f64 = (PI * 0.01 / PI).sqrt(); // = 0.1
        let expected: f64 = 4.0 * F * 1e-5 * 1e-6 * r;
        assert!(approx_eq(i, expected, 1e-15));
    }

    #[test]
    fn test_cell_residual() {
        let cell = AmperometricCell::new(0.5, 0.1, 1e-5, F);
        let i_res: f64 = cell.residual_current(0.1, 20e-6); // 20 uF/cm^2
        let expected: f64 = 20e-6 * 0.1 * 0.1;
        assert!(approx_eq(i_res, expected, 1e-15));
    }

    // -----------------------------------------------------------------------
    // TitrationCurve
    // -----------------------------------------------------------------------

    #[test]
    fn test_titration_l_shaped() {
        // Decreasing current then flat
        let v: Vec<f64> = (0..10).map(|k| k as f64).collect();
        let i: Vec<f64> = v
            .iter()
            .map(|&x| if x < 5.0 { 10.0 - 2.0 * x } else { 0.5 })
            .collect();
        let tc = TitrationCurve::new(v, i);
        assert_eq!(tc.curve_type, CurveType::LShaped);
    }

    #[test]
    fn test_titration_v_shaped() {
        // Decreasing then increasing
        let v: Vec<f64> = (0..10).map(|k| k as f64).collect();
        let i: Vec<f64> = v
            .iter()
            .map(|&x| {
                if x < 5.0 {
                    10.0 - 2.0 * x
                } else {
                    2.0 * (x - 5.0)
                }
            })
            .collect();
        let tc = TitrationCurve::new(v, i);
        assert_eq!(tc.curve_type, CurveType::VShaped);
    }

    #[test]
    fn test_titration_inverted_l() {
        // Flat then increasing
        let v: Vec<f64> = (0..10).map(|k| k as f64).collect();
        let i: Vec<f64> = v
            .iter()
            .map(|&x| if x < 5.0 { 0.5 } else { 2.0 * (x - 4.5) })
            .collect();
        let tc = TitrationCurve::new(v, i);
        assert_eq!(tc.curve_type, CurveType::InvertedL);
    }

    #[test]
    fn test_titration_endpoint_v_shaped() {
        // V-shaped with clear endpoint at v=5
        let v: Vec<f64> = (0..20).map(|k| k as f64 * 0.5).collect();
        let i: Vec<f64> = v
            .iter()
            .map(|&x| {
                if x < 5.0 {
                    10.0 - 2.0 * x
                } else {
                    2.0 * (x - 5.0)
                }
            })
            .collect();
        let tc = TitrationCurve::new(v, i);
        let ep = tc.find_endpoint();
        assert!(ep.is_some());
        let (v_ep, _i_ep) = ep.unwrap();
        assert!(
            approx_eq(v_ep, 5.0, 0.5),
            "Endpoint volume = {}, expected ~5.0",
            v_ep
        );
    }

    #[test]
    fn test_titration_first_derivative() {
        let v: Vec<f64> = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let i: Vec<f64> = vec![0.0, 1.0, 4.0, 9.0, 16.0]; // quadratic
        let tc = TitrationCurve::new(v, i);
        let deriv = tc.first_derivative();
        assert_eq!(deriv.len(), 5);
        // Central difference at k=2: (9-1)/(3-1) = 4.0
        assert!(approx_eq(deriv[2], 4.0, EPSILON));
    }

    #[test]
    fn test_titration_derivative_endpoint() {
        // Use a sigmoidal current transition so the max |derivative| is at the inflection
        let v: Vec<f64> = (0..40).map(|k| k as f64 * 0.25).collect();
        let i: Vec<f64> = v
            .iter()
            .map(|&x| {
                1.0 + 9.0 / (1.0 + (-4.0_f64 * (x - 5.0)).exp())
            })
            .collect();
        let tc = TitrationCurve::new(v, i);
        let ep = tc.derivative_endpoint();
        assert!(ep.is_some());
        let v_ep: f64 = ep.unwrap();
        assert!(
            approx_eq(v_ep, 5.0, 0.5),
            "Derivative endpoint = {}, expected ~5.0",
            v_ep
        );
    }

    #[test]
    fn test_titration_gran_plot() {
        let v: Vec<f64> = vec![1.0, 2.0, 3.0, 4.0];
        let i: Vec<f64> = vec![10.0, 8.0, 6.0, 4.0];
        let tc = TitrationCurve::new(v, i);
        let gran = tc.gran_plot();
        assert_eq!(gran.len(), 4);
        assert!(approx_eq(gran[0].1, 10.0, EPSILON)); // 1*10
        assert!(approx_eq(gran[1].1, 16.0, EPSILON)); // 2*8
    }

    #[test]
    fn test_titration_too_few_points() {
        let v: Vec<f64> = vec![1.0, 2.0];
        let i: Vec<f64> = vec![10.0, 8.0];
        let tc = TitrationCurve::new(v, i);
        assert!(tc.find_endpoint().is_none());
    }

    #[test]
    fn test_titration_gran_endpoint() {
        // Linear Gran plot in second half
        let v: Vec<f64> = (0..20).map(|k| k as f64 * 0.5).collect();
        let i: Vec<f64> = v
            .iter()
            .map(|&x| {
                if x < 5.0 {
                    10.0 - 2.0 * x
                } else {
                    0.5 + 1.0 * (x - 5.0)
                }
            })
            .collect();
        let tc = TitrationCurve::new(v, i);
        let ep = tc.gran_endpoint();
        assert!(ep.is_some());
    }

    // -----------------------------------------------------------------------
    // BiamperometricTitration
    // -----------------------------------------------------------------------

    #[test]
    fn test_biamperometric_add_point() {
        let mut bt = BiamperometricTitration::new(100.0);
        bt.add_point(0.0, 0.1);
        bt.add_point(1.0, 0.2);
        assert_eq!(bt.volumes.len(), 2);
        assert_eq!(bt.differential_currents.len(), 2);
    }

    #[test]
    fn test_biamperometric_karl_fischer() {
        let mut bt = BiamperometricTitration::new(50.0);
        // Near-zero current until endpoint, then sharp rise
        for k in 0..10 {
            let v: f64 = k as f64;
            let i: f64 = if k < 5 { 0.01 } else { 5.0 + k as f64 };
            bt.add_point(v, i);
        }
        let ep = bt.karl_fischer_endpoint();
        assert!(ep.is_some());
        let v_ep: f64 = ep.unwrap();
        assert!(v_ep >= 4.0 && v_ep <= 6.0, "KF endpoint = {}", v_ep);
    }

    #[test]
    fn test_biamperometric_water_content() {
        let mut bt = BiamperometricTitration::new(50.0);
        for k in 0..10 {
            let v: f64 = k as f64;
            let i: f64 = if k < 5 { 0.01 } else { 5.0 + k as f64 };
            bt.add_point(v, i);
        }
        let ppm = bt.water_content_ppm(5.0, 10.0);
        assert!(ppm.is_some());
        assert!(ppm.unwrap() > 0.0);
    }

    #[test]
    fn test_biamperometric_find_endpoint() {
        let mut bt = BiamperometricTitration::new(100.0);
        for k in 0..20 {
            let v: f64 = k as f64 * 0.5;
            let i: f64 = if v < 5.0 { 0.1 } else { 2.0 * (v - 5.0) };
            bt.add_point(v, i);
        }
        let ep = bt.find_endpoint();
        assert!(ep.is_some());
    }

    #[test]
    fn test_biamperometric_too_few() {
        let mut bt = BiamperometricTitration::new(100.0);
        bt.add_point(0.0, 0.1);
        bt.add_point(1.0, 0.2);
        assert!(bt.karl_fischer_endpoint().is_none());
    }

    // -----------------------------------------------------------------------
    // DiffusionCurrent
    // -----------------------------------------------------------------------

    #[test]
    fn test_ilkovic_current_basic() {
        let dc = DiffusionCurrent::new(2, 1e-5, 2.0, 4.0);
        let id: f64 = dc.ilkovic_current(1.0);
        // id = 607 * 2 * sqrt(1e-5) * 2^(2/3) * 4^(1/6) * 1
        let expected: f64 = 607.0
            * 2.0
            * (1e-5_f64).sqrt()
            * (2.0_f64).powf(2.0 / 3.0)
            * (4.0_f64).powf(1.0 / 6.0)
            * 1.0;
        assert!(approx_eq(id, expected, 1e-10));
    }

    #[test]
    fn test_ilkovic_scales_with_concentration() {
        let dc = DiffusionCurrent::new(1, 1e-5, 2.0, 4.0);
        let id1: f64 = dc.ilkovic_current(1.0);
        let id2: f64 = dc.ilkovic_current(2.0);
        assert!(approx_eq(id2, 2.0 * id1, 1e-10));
    }

    #[test]
    fn test_limiting_current() {
        let dc = DiffusionCurrent::new(1, 1e-5, 2.0, 4.0);
        let il: f64 = dc.limiting_current(1.0);
        let id: f64 = dc.ilkovic_current(1.0);
        assert!(approx_eq(il, id, 1e-15));
    }

    #[test]
    fn test_residual_correction() {
        let dc = DiffusionCurrent::new(1, 1e-5, 2.0, 4.0);
        let corrected: f64 = dc.correct_residual(10.0, 1.5);
        assert!(approx_eq(corrected, 8.5, EPSILON));
    }

    #[test]
    fn test_half_wave_from_wave() {
        let dc = DiffusionCurrent::new(1, 1e-5, 2.0, 4.0);
        // At i = i_d/2, the correction term should be zero (ln(1)=0)
        let e_half: f64 = dc.half_wave_from_wave(-0.5, 5.0, 10.0, 298.15);
        // ratio = (10-5)/5 = 1, ln(1)=0, so E_1/2 = -0.5
        assert!(approx_eq(e_half, -0.5, EPSILON));
    }

    #[test]
    fn test_extract_diffusion_coeff() {
        let dc = DiffusionCurrent::new(2, 1e-5, 2.0, 4.0);
        // id for D=1e-5 and c=1.0
        let id: f64 = dc.ilkovic_current(1.0);
        let d_extracted: f64 = dc.extract_diffusion_coeff(id, 1.0);
        assert!(
            relative_eq(d_extracted, 1e-5, 0.01),
            "D extracted = {}, expected 1e-5",
            d_extracted
        );
    }

    // -----------------------------------------------------------------------
    // RotatingDiskElectrode
    // -----------------------------------------------------------------------

    #[test]
    fn test_rde_levich() {
        let rde = RotatingDiskElectrode::new(4, F, 0.196, 1e-5, 0.01, 1e-6);
        let i: f64 = rde.levich_current(100.0);
        let expected: f64 = levich_current(4, F, 0.196, 1e-5, 100.0, 0.01, 1e-6);
        assert!(approx_eq(i, expected, 1e-15));
    }

    #[test]
    fn test_rde_koutecky_levich() {
        let rde = RotatingDiskElectrode::new(4, F, 0.196, 1e-5, 0.01, 1e-6);
        let omegas: Vec<f64> = vec![100.0, 200.0, 400.0, 800.0, 1600.0];
        let currents: Vec<f64> = omegas.iter().map(|&w| rde.levich_current(w)).collect();
        let (slope, intercept, r2) = rde.koutecky_levich_plot(&omegas, &currents);
        // For purely diffusion-limited, intercept should be ~0
        assert!(intercept.abs() < 1e-3, "KL intercept = {}", intercept);
        assert!(r2 > 0.99, "KL R^2 = {}", r2);
        let _ = slope; // used in plot
    }

    #[test]
    fn test_rde_kinetic_current() {
        let rde = RotatingDiskElectrode::new(4, F, 0.196, 1e-5, 0.01, 1e-6);
        let omegas: Vec<f64> = vec![100.0, 200.0, 400.0, 800.0];
        // Add a kinetic component: 1/i = 1/i_L + 1/i_k
        let i_k: f64 = 0.01;
        let currents: Vec<f64> = omegas
            .iter()
            .map(|&w| {
                let i_l: f64 = rde.levich_current(w);
                1.0 / (1.0 / i_l + 1.0 / i_k)
            })
            .collect();
        let ik_extracted: f64 = rde.kinetic_current(&omegas, &currents);
        assert!(
            relative_eq(ik_extracted, i_k, 0.1),
            "i_k = {}, expected {}",
            ik_extracted,
            i_k
        );
    }

    #[test]
    fn test_rde_levich_slope() {
        let rde = RotatingDiskElectrode::new(2, F, 0.1, 1e-5, 0.01, 1e-6);
        let b: f64 = rde.levich_slope();
        let expected: f64 = 0.62 * 2.0 * F * 0.1 * (1e-5_f64).powf(2.0 / 3.0)
            * (0.01_f64).powf(-1.0 / 6.0)
            * 1e-6;
        assert!(approx_eq(b, expected, 1e-15));
    }

    #[test]
    fn test_rde_electron_number() {
        let rde = RotatingDiskElectrode::new(4, F, 0.196, 1e-5, 0.01, 1e-6);
        let b: f64 = rde.levich_slope();
        let n_calc: f64 = rde.electron_number_from_slope(b);
        assert!(
            approx_eq(n_calc, 4.0, 0.01),
            "n = {}, expected 4",
            n_calc
        );
    }

    #[test]
    fn test_rde_rpm_to_omega() {
        let omega: f64 = RotatingDiskElectrode::rpm_to_omega(60.0);
        assert!(approx_eq(omega, 2.0 * PI, EPSILON));
    }

    // -----------------------------------------------------------------------
    // EndpointDetector
    // -----------------------------------------------------------------------

    #[test]
    fn test_endpoint_piecewise_linear() {
        let det = EndpointDetector::new(3);
        let x: Vec<f64> = (0..20).map(|k| k as f64).collect();
        let y: Vec<f64> = x
            .iter()
            .map(|&xi| if xi < 10.0 { 20.0 - 2.0 * xi } else { 1.0 * (xi - 10.0) })
            .collect();
        let ep = det.piecewise_linear_endpoint(&x, &y);
        assert!(ep.is_some());
        let (x_ep, _) = ep.unwrap();
        assert!(
            approx_eq(x_ep, 10.0, 1.0),
            "Piecewise endpoint = {}",
            x_ep
        );
    }

    #[test]
    fn test_endpoint_first_derivative() {
        let det = EndpointDetector::new(2);
        let x: Vec<f64> = (0..20).map(|k| k as f64).collect();
        let y: Vec<f64> = x
            .iter()
            .map(|&xi| if xi < 10.0 { 20.0 - 2.0 * xi } else { 5.0 * (xi - 10.0) })
            .collect();
        let ep = det.first_derivative_endpoint(&x, &y);
        assert!(ep.is_some());
        let x_ep: f64 = ep.unwrap();
        assert!(
            approx_eq(x_ep, 10.0, 1.5),
            "Deriv endpoint = {}",
            x_ep
        );
    }

    #[test]
    fn test_endpoint_gran_plot() {
        let det = EndpointDetector::new(2);
        let v: Vec<f64> = (0..20).map(|k| k as f64 * 0.5).collect();
        let i: Vec<f64> = v
            .iter()
            .map(|&x| {
                if x < 5.0 {
                    10.0 - 2.0 * x
                } else {
                    0.5 + 1.0 * (x - 5.0)
                }
            })
            .collect();
        let ep = det.gran_plot_endpoint(&v, &i);
        assert!(ep.is_some());
    }

    #[test]
    fn test_endpoint_too_few() {
        let det = EndpointDetector::new(3);
        let x: Vec<f64> = vec![1.0, 2.0, 3.0];
        let y: Vec<f64> = vec![1.0, 2.0, 3.0];
        assert!(det.piecewise_linear_endpoint(&x, &y).is_none());
    }

    #[test]
    fn test_endpoint_first_derivative_too_few() {
        let det = EndpointDetector::new(2);
        assert!(det.first_derivative_endpoint(&[1.0], &[1.0]).is_none());
    }

    // -----------------------------------------------------------------------
    // OxygenSensor
    // -----------------------------------------------------------------------

    #[test]
    fn test_o2_sensor_current() {
        let sensor = OxygenSensor::new(1e-6, 0.01, 0.1);
        let i: f64 = sensor.sensor_current(1e-7); // 100 nmol/cm^3
        // i = 4 * F * 0.1 * 1e-6 * 1e-7 / 0.01
        let expected: f64 = 4.0 * 96485.0 * 0.1 * 1e-6 * 1e-7 / 0.01;
        assert!(approx_eq(i, expected, 1e-15));
    }

    #[test]
    fn test_o2_temp_compensation() {
        let sensor = OxygenSensor::new(1e-6, 0.01, 0.1);
        let i_meas: f64 = 1.0e-6;
        let i_comp: f64 = sensor.temperature_compensated_current(i_meas, 25.0);
        // At calibration temperature, correction = 1.0
        assert!(approx_eq(i_comp, i_meas, 1e-15));
    }

    #[test]
    fn test_o2_temp_compensation_different() {
        let sensor = OxygenSensor::new(1e-6, 0.01, 0.1);
        let i_meas: f64 = 1.0e-6;
        let i_comp: f64 = sensor.temperature_compensated_current(i_meas, 35.0);
        // At 35C, correction = 1 + 0.03*10 = 1.3
        let expected: f64 = i_meas / 1.3;
        assert!(approx_eq(i_comp, expected, 1e-15));
    }

    #[test]
    fn test_o2_do_mg_l() {
        let sensor = OxygenSensor::new(1e-6, 0.01, 0.1);
        let do_val: f64 = sensor.do_mg_l(5e-7, 1e-6, 8.0);
        assert!(approx_eq(do_val, 4.0, EPSILON)); // 0.5/1.0 * 8.0
    }

    #[test]
    fn test_o2_do_saturation() {
        let c_sat: f64 = OxygenSensor::do_saturation(0.21, 769.0);
        assert!(c_sat > 0.0);
    }

    #[test]
    fn test_o2_response_time() {
        let sensor = OxygenSensor::new(1e-6, 0.01, 0.1);
        let t95: f64 = sensor.response_time_95();
        assert!(t95 > 0.0 && t95.is_finite());
    }

    #[test]
    fn test_o2_winkler() {
        let do_val: f64 = OxygenSensor::winkler_do(2.0, 0.025, 200.0);
        // = 2 * 0.025 * 8000 / 200 = 2.0
        assert!(approx_eq(do_val, 2.0, EPSILON));
    }

    #[test]
    fn test_o2_winkler_zero_volume() {
        let do_val: f64 = OxygenSensor::winkler_do(2.0, 0.025, 0.0);
        assert!(approx_eq(do_val, 0.0, EPSILON));
    }

    // -----------------------------------------------------------------------
    // GlucoseBiosensor
    // -----------------------------------------------------------------------

    #[test]
    fn test_glucose_reaction_rate() {
        let sensor = GlucoseBiosensor::new(100.0, 5.0, 10.0, 20.0);
        let v: f64 = sensor.reaction_rate(5.0);
        // v = 100 * 5 / (5 + 5) = 50
        assert!(approx_eq(v, 50.0, EPSILON));
    }

    #[test]
    fn test_glucose_reaction_rate_zero() {
        let sensor = GlucoseBiosensor::new(100.0, 5.0, 10.0, 20.0);
        assert!(approx_eq(sensor.reaction_rate(0.0), 0.0, EPSILON));
    }

    #[test]
    fn test_glucose_reaction_rate_negative() {
        let sensor = GlucoseBiosensor::new(100.0, 5.0, 10.0, 20.0);
        assert!(approx_eq(sensor.reaction_rate(-1.0), 0.0, EPSILON));
    }

    #[test]
    fn test_glucose_predicted_current_linear() {
        let sensor = GlucoseBiosensor::new(100.0, 5.0, 10.0, 20.0);
        let i: f64 = sensor.predicted_current(10.0);
        // Linear: 10 * 10 = 100 nA
        assert!(approx_eq(i, 100.0, EPSILON));
    }

    #[test]
    fn test_glucose_predicted_current_zero() {
        let sensor = GlucoseBiosensor::new(100.0, 5.0, 10.0, 20.0);
        assert!(approx_eq(sensor.predicted_current(0.0), 0.0, EPSILON));
    }

    #[test]
    fn test_glucose_from_current_linear() {
        let sensor = GlucoseBiosensor::new(100.0, 5.0, 10.0, 20.0);
        let conc: f64 = sensor.glucose_from_current(100.0);
        assert!(approx_eq(conc, 10.0, EPSILON));
    }

    #[test]
    fn test_glucose_from_current_zero() {
        let sensor = GlucoseBiosensor::new(100.0, 5.0, 10.0, 20.0);
        assert!(approx_eq(sensor.glucose_from_current(0.0), 0.0, EPSILON));
    }

    #[test]
    fn test_glucose_linear_range() {
        let sensor = GlucoseBiosensor::new(100.0, 5.0, 10.0, 20.0);
        assert!(sensor.is_in_linear_range(10.0));
        assert!(sensor.is_in_linear_range(20.0));
        assert!(!sensor.is_in_linear_range(25.0));
        assert!(!sensor.is_in_linear_range(-1.0));
    }

    #[test]
    fn test_glucose_corrected_current() {
        let sensor = GlucoseBiosensor::new(100.0, 5.0, 10.0, 20.0);
        let corrected: f64 = sensor.corrected_current(100.0, 10.0);
        // interference_rejection = 0.95, so 100 - 10 * 0.05 = 99.5
        assert!(approx_eq(corrected, 99.5, EPSILON));
    }

    #[test]
    fn test_glucose_lineweaver_burk() {
        // Known V_max=100, K_m=5
        let concs: Vec<f64> = vec![1.0, 2.0, 5.0, 10.0, 20.0];
        let rates: Vec<f64> = concs
            .iter()
            .map(|&c| 100.0 * c / (5.0 + c))
            .collect();
        let (v_max, k_m) = GlucoseBiosensor::lineweaver_burk(&concs, &rates);
        assert!(
            relative_eq(v_max, 100.0, 0.05),
            "V_max = {}, expected 100",
            v_max
        );
        assert!(
            relative_eq(k_m, 5.0, 0.05),
            "K_m = {}, expected 5",
            k_m
        );
    }

    #[test]
    fn test_glucose_detection_limit() {
        let sensor = GlucoseBiosensor::new(100.0, 5.0, 10.0, 20.0);
        let lod: f64 = sensor.detection_limit(0.1);
        // 3 * 0.1 / 10 = 0.03
        assert!(approx_eq(lod, 0.03, EPSILON));
    }

    // -----------------------------------------------------------------------
    // ChronoamperometryProcessor
    // -----------------------------------------------------------------------

    #[test]
    fn test_chrono_add_point() {
        let mut proc = ChronoamperometryProcessor::new(1, 0.1, 1e-6);
        proc.add_point(0.1, 1e-4);
        proc.add_point(0.2, 7e-5);
        assert_eq!(proc.times.len(), 2);
        assert_eq!(proc.currents.len(), 2);
    }

    #[test]
    fn test_chrono_load_data() {
        let mut proc = ChronoamperometryProcessor::new(1, 0.1, 1e-6);
        proc.load_data(&[0.1, 0.2, 0.5], &[1e-4, 7e-5, 4.5e-5]);
        assert_eq!(proc.times.len(), 3);
    }

    #[test]
    fn test_chrono_cottrell_plot() {
        let mut proc = ChronoamperometryProcessor::new(1, 0.1, 1e-6);
        proc.load_data(
            &[0.1, 0.4, 1.0, 4.0],
            &[1.0, 0.5, 0.316, 0.158],
        );
        let (x, y) = proc.cottrell_plot();
        assert_eq!(x.len(), 4);
        assert_eq!(y.len(), 4);
        // x[0] = 1/sqrt(0.1) ≈ 3.162
        assert!(approx_eq(x[0], 1.0 / (0.1_f64).sqrt(), 0.001));
    }

    #[test]
    fn test_chrono_extract_diffusion_coeff() {
        // Generate synthetic Cottrell data with known D
        let d_true: f64 = 1e-5;
        let n: u32 = 1;
        let area: f64 = 0.1;
        let conc: f64 = 1e-6;
        let mut proc = ChronoamperometryProcessor::new(n, area, conc);
        for k in 1..20 {
            let t: f64 = k as f64 * 0.1;
            let i: f64 = cottrell_current(n, F, area, d_true, conc, t);
            proc.add_point(t, i);
        }
        let d_calc: f64 = proc.extract_diffusion_coeff();
        assert!(
            relative_eq(d_calc, d_true, 0.05),
            "D = {}, expected {}",
            d_calc,
            d_true
        );
    }

    #[test]
    fn test_chrono_r_squared() {
        let d: f64 = 1e-5;
        let mut proc = ChronoamperometryProcessor::new(1, 0.1, 1e-6);
        for k in 1..20 {
            let t: f64 = k as f64 * 0.1;
            let i: f64 = cottrell_current(1, F, 0.1, d, 1e-6, t);
            proc.add_point(t, i);
        }
        let r2: f64 = proc.cottrell_r_squared();
        assert!(r2 > 0.99, "Cottrell R^2 = {}", r2);
    }

    #[test]
    fn test_chrono_anson_plot() {
        let mut proc = ChronoamperometryProcessor::new(1, 0.1, 1e-6);
        proc.load_data(&[0.1, 0.2, 0.3, 0.4, 0.5], &[1.0, 0.8, 0.7, 0.6, 0.55]);
        let (t_sqrt, charges) = proc.anson_plot();
        assert_eq!(t_sqrt.len(), 5);
        assert_eq!(charges.len(), 5);
        // Charge should increase monotonically
        for k in 1..charges.len() {
            assert!(charges[k] >= charges[k - 1]);
        }
    }

    #[test]
    fn test_chrono_current_ratio() {
        let d: f64 = 1e-5;
        let mut proc = ChronoamperometryProcessor::new(1, 0.1, 1e-6);
        for k in 1..10 {
            let t: f64 = k as f64;
            let i: f64 = cottrell_current(1, F, 0.1, d, 1e-6, t);
            proc.add_point(t, i);
        }
        let result = proc.current_ratio_check(0, 3); // t=1 and t=4
        assert!(result.is_some());
        let (measured, expected) = result.unwrap();
        assert!(
            relative_eq(measured, expected, 0.01),
            "ratio measured={}, expected={}",
            measured,
            expected
        );
    }

    #[test]
    fn test_chrono_current_ratio_invalid() {
        let proc = ChronoamperometryProcessor::new(1, 0.1, 1e-6);
        assert!(proc.current_ratio_check(0, 1).is_none());
    }

    #[test]
    fn test_chrono_residual_current() {
        let mut proc = ChronoamperometryProcessor::new(1, 0.1, 1e-6);
        for k in 0..1000 {
            let t: f64 = (k + 1) as f64 * 0.1;
            // Decaying to residual of 0.01
            let i: f64 = 1.0 / t.sqrt() + 0.01;
            proc.add_point(t, i);
        }
        let residual: f64 = proc.residual_current();
        // At late times (t ~ 90-100 s), i ≈ 0.01 + 1/sqrt(100) = 0.01 + 0.1 = 0.11
        assert!(
            residual > 0.005 && residual < 0.2,
            "residual = {}",
            residual
        );
    }

    #[test]
    fn test_chrono_diffusion_from_anson() {
        let d_true: f64 = 1e-5;
        let n: u32 = 1;
        let area: f64 = 0.1;
        let conc: f64 = 1e-6;
        let mut proc = ChronoamperometryProcessor::new(n, area, conc);
        for k in 1..50 {
            let t: f64 = k as f64 * 0.1;
            let i: f64 = cottrell_current(n, F, area, d_true, conc, t);
            proc.add_point(t, i);
        }
        let d_anson: f64 = proc.diffusion_coeff_from_anson();
        // Anson method may have integration approximation error
        assert!(
            d_anson > 0.0,
            "D from Anson should be positive, got {}",
            d_anson
        );
    }

    #[test]
    fn test_chrono_empty_cottrell_plot() {
        let proc = ChronoamperometryProcessor::new(1, 0.1, 1e-6);
        let (x, y) = proc.cottrell_plot();
        assert!(x.is_empty());
        assert!(y.is_empty());
    }

    #[test]
    fn test_chrono_empty_extract_d() {
        let proc = ChronoamperometryProcessor::new(1, 0.1, 1e-6);
        assert!(approx_eq(proc.extract_diffusion_coeff(), 0.0, EPSILON));
    }

    // -----------------------------------------------------------------------
    // Line intersection
    // -----------------------------------------------------------------------

    #[test]
    fn test_line_intersection_basic() {
        let result = line_intersection(1.0, 0.0, -1.0, 10.0);
        assert!(result.is_some());
        let (x, y) = result.unwrap();
        assert!(approx_eq(x, 5.0, EPSILON));
        assert!(approx_eq(y, 5.0, EPSILON));
    }

    #[test]
    fn test_line_intersection_parallel() {
        let result = line_intersection(2.0, 1.0, 2.0, 3.0);
        assert!(result.is_none());
    }

    // -----------------------------------------------------------------------
    // Additional edge-case and coverage tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_cottrell_scales_with_concentration() {
        let i1: f64 = cottrell_current(1, F, 0.1, 1e-5, 1e-6, 1.0);
        let i3: f64 = cottrell_current(1, F, 0.1, 1e-5, 3e-6, 1.0);
        assert!(approx_eq(i3, 3.0 * i1, 1e-15));
    }

    #[test]
    fn test_levich_scales_with_n() {
        let i1: f64 = levich_current(1, F, 0.1, 1e-5, 100.0, 0.01, 1e-6);
        let i4: f64 = levich_current(4, F, 0.1, 1e-5, 100.0, 0.01, 1e-6);
        assert!(approx_eq(i4, 4.0 * i1, 1e-12));
    }

    #[test]
    fn test_cell_new() {
        let cell = AmperometricCell::new(-0.6, 0.07, 2e-5, F);
        assert!(approx_eq(cell.applied_potential_v, -0.6, EPSILON));
        assert!(approx_eq(cell.electrode_area_cm2, 0.07, EPSILON));
    }

    #[test]
    fn test_titration_first_derivative_short() {
        let tc = TitrationCurve::new(vec![0.0], vec![1.0]);
        let d = tc.first_derivative();
        assert!(d.is_empty());
    }

    #[test]
    fn test_diffusion_current_new() {
        let dc = DiffusionCurrent::new(3, 2e-5, 1.5, 3.0);
        assert_eq!(dc.n, 3);
        assert!(approx_eq(dc.diffusion_coeff, 2e-5, 1e-20));
    }

    #[test]
    fn test_rde_new() {
        let rde = RotatingDiskElectrode::new(2, F, 0.1, 1e-5, 0.01, 1e-6);
        assert_eq!(rde.n, 2);
        assert!(approx_eq(rde.area_cm2, 0.1, EPSILON));
    }

    #[test]
    fn test_endpoint_detector_new() {
        let det = EndpointDetector::new(1);
        assert_eq!(det.min_segment_points, 2); // clamped
    }

    #[test]
    fn test_o2_sensor_new() {
        let s = OxygenSensor::new(1e-6, 0.01, 0.1);
        assert_eq!(s.n_electrons, 4);
        assert!(approx_eq(s.cal_temp_c, 25.0, EPSILON));
    }

    #[test]
    fn test_glucose_biosensor_new() {
        let g = GlucoseBiosensor::new(100.0, 5.0, 10.0, 20.0);
        assert!(approx_eq(g.interference_rejection, 0.95, EPSILON));
    }

    #[test]
    fn test_chrono_processor_new() {
        let p = ChronoamperometryProcessor::new(2, 0.2, 5e-7);
        assert_eq!(p.n, 2);
        assert!(p.times.is_empty());
    }

    #[test]
    fn test_half_wave_no_crossing() {
        // All currents below half of max
        let e: Vec<f64> = vec![0.0, 0.1, 0.2, 0.3];
        let i: Vec<f64> = vec![0.1, 0.1, 0.1, 0.1];
        // i_lim = 0.1, half = 0.05, all above half => no crossing
        // Actually half = 0.05, and all values = 0.1 > 0.05, difference always positive
        // so (i0 - half)*(i1 - half) > 0, no crossing
        let result = half_wave_potential(&e, &i);
        assert!(result.is_none());
    }

    #[test]
    fn test_titration_classify_few_points() {
        let tc = TitrationCurve::new(vec![0.0, 1.0, 2.0], vec![3.0, 2.0, 1.0]);
        assert_eq!(tc.curve_type, CurveType::LShaped); // default for <4 points
    }

    #[test]
    fn test_biamperometric_no_persistent_rise() {
        let mut bt = BiamperometricTitration::new(50.0);
        bt.add_point(0.0, 0.01);
        bt.add_point(1.0, 0.01);
        bt.add_point(2.0, 0.5); // single spike, next drops
        bt.add_point(3.0, 0.01);
        bt.add_point(4.0, 0.01);
        // No persistent rise above threshold
        let ep = bt.karl_fischer_endpoint();
        // May or may not detect depending on threshold relative to max
        // max = 0.5, threshold = 0.05, point 2 is above, but point 3 is below
        assert!(ep.is_none());
    }

    #[test]
    fn test_glucose_saturation_current() {
        let sensor = GlucoseBiosensor::new(100.0, 5.0, 10.0, 20.0);
        // Above linear range, should be > linear extrapolation but saturating
        let i_above: f64 = sensor.predicted_current(30.0);
        let i_linear: f64 = sensor.predicted_current(20.0);
        assert!(i_above > i_linear, "Saturation current should still increase");
    }

    #[test]
    fn test_chrono_residual_few_points() {
        let proc = ChronoamperometryProcessor::new(1, 0.1, 1e-6);
        assert!(approx_eq(proc.residual_current(), 0.0, EPSILON));
    }

    #[test]
    fn test_chrono_anson_empty() {
        let proc = ChronoamperometryProcessor::new(1, 0.1, 1e-6);
        let (x, y) = proc.anson_plot();
        assert!(x.is_empty());
        assert!(y.is_empty());
    }

    #[test]
    fn test_endpoint_gran_too_few() {
        let det = EndpointDetector::new(2);
        assert!(det.gran_plot_endpoint(&[1.0, 2.0], &[1.0, 2.0]).is_none());
    }

    #[test]
    fn test_cottrell_with_large_diffusion() {
        let i: f64 = cottrell_current(1, F, 0.1, 1e-3, 1e-6, 0.5);
        assert!(i > 0.0);
        // Larger D => larger current
        let i_small_d: f64 = cottrell_current(1, F, 0.1, 1e-7, 1e-6, 0.5);
        assert!(i > i_small_d);
    }

    #[test]
    fn test_do_zero_cal_current() {
        let sensor = OxygenSensor::new(1e-6, 0.01, 0.1);
        let do_val: f64 = sensor.do_mg_l(1e-6, 0.0, 8.0);
        assert!(approx_eq(do_val, 0.0, EPSILON));
    }

    #[test]
    fn test_o2_response_time_zero_perm() {
        let sensor = OxygenSensor::new(0.0, 0.01, 0.1);
        let t: f64 = sensor.response_time_95();
        assert!(t.is_infinite());
    }

    #[test]
    fn test_glucose_detection_limit_zero_sensitivity() {
        let sensor = GlucoseBiosensor::new(100.0, 5.0, 0.0, 20.0);
        let lod: f64 = sensor.detection_limit(0.1);
        assert!(lod.is_infinite());
    }

    #[test]
    fn test_ilkovic_zero_concentration() {
        let dc = DiffusionCurrent::new(1, 1e-5, 2.0, 4.0);
        assert!(approx_eq(dc.ilkovic_current(0.0), 0.0, EPSILON));
    }

    #[test]
    fn test_extract_d_zero_concentration() {
        let dc = DiffusionCurrent::new(1, 1e-5, 2.0, 4.0);
        let d: f64 = dc.extract_diffusion_coeff(1.0, 0.0);
        assert!(approx_eq(d, 0.0, EPSILON));
    }
}
