//! Nephelometric immunoassay signal processing for protein quantitation.
//!
//! Implements light scattering measurement from antigen-antibody complex formation
//! for clinical chemistry applications. Covers Rayleigh and Mie scattering theory,
//! Heidelberger-Kendall precipitation curves, rate and endpoint nephelometry,
//! calibration curve fitting (4PL logistic), clinical protein panels with reference
//! ranges, interference detection, and latex-enhanced nephelometry.
//!
//! # Example
//!
//! ```
//! use r4w_core::nephelometric_immunoassay_processor::{
//!     rayleigh_intensity, size_parameter, four_parameter_logistic,
//!     CalibrationCurve, CalibrationPoint, RateNephelometry,
//! };
//!
//! // Rayleigh scattering from a 50 nm particle at 670 nm laser
//! let intensity = rayleigh_intensity(50.0, 670.0, 1.05, 1.2);
//! assert!(intensity > 0.0);
//!
//! // Size parameter determines scattering regime
//! let x = size_parameter(50.0, 670.0);
//! assert!(x < 1.0); // Rayleigh regime
//!
//! // 4-parameter logistic calibration
//! let y = four_parameter_logistic(100.0, 0.1, 1.5, 50.0, 1.0);
//! assert!(y > 0.1 && y < 1.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Rayleigh scattering intensity for a small particle.
///
/// I ∝ (1 + cos²θ) / r² × ((n²-1)/(n²+2))² × d⁶ / λ⁴
///
/// Returns relative intensity (arbitrary units, r=1 assumed).
///
/// * `d_nm` - particle diameter in nanometers
/// * `lambda_nm` - wavelength in nanometers
/// * `n_rel` - relative refractive index (particle / medium)
/// * `theta_rad` - scattering angle in radians
pub fn rayleigh_intensity(d_nm: f64, lambda_nm: f64, n_rel: f64, theta_rad: f64) -> f64 {
    let n2: f64 = n_rel * n_rel;
    let clausius_mossotti: f64 = (n2 - 1.0) / (n2 + 2.0);
    let angular: f64 = 1.0 + theta_rad.cos() * theta_rad.cos();
    let d6: f64 = d_nm.powi(6);
    let l4: f64 = lambda_nm.powi(4);
    angular * clausius_mossotti * clausius_mossotti * d6 / l4
}

/// Mie size parameter x = π·d / λ.
///
/// * `d_nm` - particle diameter in nanometers
/// * `lambda_nm` - wavelength in nanometers
pub fn size_parameter(d_nm: f64, lambda_nm: f64) -> f64 {
    PI * d_nm / lambda_nm
}

/// Four-parameter logistic (4PL) function: y = d + (a - d) / (1 + (x/c)^b).
///
/// Used for immunoassay calibration curves (Logit-Log).
///
/// * `x` - analyte concentration
/// * `a` - minimum asymptote (response at zero concentration)
/// * `b` - Hill slope (steepness)
/// * `c` - inflection point (EC50)
/// * `d` - maximum asymptote (response at infinite concentration)
pub fn four_parameter_logistic(x: f64, a: f64, b: f64, c: f64, d: f64) -> f64 {
    if c <= 0.0 || x < 0.0 {
        return a;
    }
    let ratio: f64 = (x / c).powf(b);
    d + (a - d) / (1.0 + ratio)
}

// ---------------------------------------------------------------------------
// RayleighScattering
// ---------------------------------------------------------------------------

/// Rayleigh scattering model for particles much smaller than the wavelength (d << λ).
#[derive(Debug, Clone)]
pub struct RayleighScattering {
    /// Laser wavelength in nm.
    pub wavelength_nm: f64,
    /// Relative refractive index of particle to medium.
    pub n_rel: f64,
}

impl RayleighScattering {
    pub fn new(wavelength_nm: f64, n_rel: f64) -> Self {
        Self { wavelength_nm, n_rel }
    }

    /// Scattering intensity at angle theta (radians) for particle diameter d_nm.
    pub fn intensity(&self, d_nm: f64, theta_rad: f64) -> f64 {
        rayleigh_intensity(d_nm, self.wavelength_nm, self.n_rel, theta_rad)
    }

    /// Rayleigh ratio R_θ = I_s · r² / (I_0 · V), simplified relative measure.
    /// Returns ratio proportional to (n²-1)²/(n²+2)² · d⁶/λ⁴ at the given angle.
    pub fn rayleigh_ratio(&self, d_nm: f64, theta_rad: f64) -> f64 {
        // r²·I / I_0 ∝ angular_term · CM² · d⁶/λ⁴
        self.intensity(d_nm, theta_rad)
    }

    /// Dissymmetry ratio z = I(45°) / I(135°).
    /// For true Rayleigh scatterers z ≈ 1; z > 1 indicates larger particles.
    pub fn dissymmetry_ratio(&self, d_nm: f64) -> f64 {
        let i45: f64 = self.intensity(d_nm, PI / 4.0);
        let i135: f64 = self.intensity(d_nm, 3.0 * PI / 4.0);
        if i135 <= 0.0 {
            return f64::INFINITY;
        }
        i45 / i135
    }

    /// Wavelength dependence factor (λ⁻⁴).
    pub fn wavelength_factor(&self) -> f64 {
        1.0 / self.wavelength_nm.powi(4)
    }
}

// ---------------------------------------------------------------------------
// MieScattering
// ---------------------------------------------------------------------------

/// Scattering regime classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScatteringRegime {
    /// x < 0.1: Rayleigh regime (d << λ)
    Rayleigh,
    /// 0.1 ≤ x < 1: Rayleigh-Gans-Debye transition
    RayleighGansDebye,
    /// 1 ≤ x < 10: Mie regime (d ~ λ)
    Mie,
    /// x ≥ 10: Geometric optics regime (d >> λ)
    GeometricOptics,
}

/// Mie scattering model for particles comparable to or larger than the wavelength.
#[derive(Debug, Clone)]
pub struct MieScattering {
    /// Laser wavelength in nm.
    pub wavelength_nm: f64,
    /// Relative refractive index.
    pub n_rel: f64,
}

impl MieScattering {
    pub fn new(wavelength_nm: f64, n_rel: f64) -> Self {
        Self { wavelength_nm, n_rel }
    }

    /// Mie size parameter x = πd/λ.
    pub fn size_param(&self, d_nm: f64) -> f64 {
        size_parameter(d_nm, self.wavelength_nm)
    }

    /// Classify scattering regime by size parameter.
    pub fn regime(&self, d_nm: f64) -> ScatteringRegime {
        let x: f64 = self.size_param(d_nm);
        if x < 0.1 {
            ScatteringRegime::Rayleigh
        } else if x < 1.0 {
            ScatteringRegime::RayleighGansDebye
        } else if x < 10.0 {
            ScatteringRegime::Mie
        } else {
            ScatteringRegime::GeometricOptics
        }
    }

    /// Forward scatter enhancement factor (approximate).
    /// Larger particles scatter more light forward; modeled as (1 + x²).
    pub fn forward_scatter_enhancement(&self, d_nm: f64) -> f64 {
        let x: f64 = self.size_param(d_nm);
        1.0 + x * x
    }

    /// Approximate angular scattering using simplified Mie model.
    /// Forward-biased: I(θ) ∝ (1 + cos²θ) × (1 + x²·cos²(θ/2)).
    pub fn angular_intensity(&self, d_nm: f64, theta_rad: f64) -> f64 {
        let x: f64 = self.size_param(d_nm);
        let rayleigh_term: f64 = 1.0 + theta_rad.cos().powi(2);
        let mie_forward: f64 = 1.0 + x * x * (theta_rad / 2.0).cos().powi(2);
        rayleigh_term * mie_forward
    }

    /// Turbidimetry geometry: measures attenuation along beam (0°).
    pub fn turbidimetry_signal(&self, d_nm: f64) -> f64 {
        self.angular_intensity(d_nm, 0.0)
    }

    /// Nephelometry geometry: measures scatter at angle (typically 70° or 90°).
    pub fn nephelometry_signal(&self, d_nm: f64, angle_deg: f64) -> f64 {
        let theta: f64 = angle_deg * PI / 180.0;
        self.angular_intensity(d_nm, theta)
    }

    /// Scattering cross-section estimate Q_sca ∝ x⁴ for small x, ∝ x² for large x.
    pub fn scattering_efficiency(&self, d_nm: f64) -> f64 {
        let x: f64 = self.size_param(d_nm);
        if x < 1.0 {
            // Rayleigh regime: Q ∝ x⁴
            let n2: f64 = self.n_rel * self.n_rel;
            let cm: f64 = (n2 - 1.0) / (n2 + 2.0);
            (8.0 / 3.0) * x.powi(4) * cm * cm
        } else {
            // Approximate Mie: approaches 2 for large x (extinction paradox)
            2.0 * (1.0 - (2.0 / x).sin() / x)
        }
    }
}

// ---------------------------------------------------------------------------
// HeidelbergerKendall
// ---------------------------------------------------------------------------

/// Zone of the Heidelberger-Kendall precipitation curve.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrecipitationZone {
    /// Excess antibody (low antigen concentration).
    AntibodyExcess,
    /// Equivalence zone (optimal lattice formation, maximum precipitation).
    Equivalence,
    /// Excess antigen (prozone effect, decreasing signal).
    AntigenExcess,
}

/// Heidelberger-Kendall antigen-antibody precipitation curve model.
///
/// Models the bell-shaped relationship between antigen concentration and
/// immune complex (scatter signal) formation.
#[derive(Debug, Clone)]
pub struct HeidelbergerKendall {
    /// Antibody concentration (arbitrary units).
    pub antibody_conc: f64,
    /// Equivalence ratio (antigen/antibody at peak).
    pub equivalence_ratio: f64,
    /// Width parameter controlling curve shape.
    pub width: f64,
}

impl HeidelbergerKendall {
    pub fn new(antibody_conc: f64, equivalence_ratio: f64, width: f64) -> Self {
        Self {
            antibody_conc,
            equivalence_ratio,
            width,
        }
    }

    /// Default for IgG nephelometry.
    pub fn igg_default() -> Self {
        Self::new(10.0, 2.0, 1.5)
    }

    /// Equivalence point antigen concentration.
    pub fn equivalence_point(&self) -> f64 {
        self.antibody_conc * self.equivalence_ratio
    }

    /// Signal (scatter intensity) at a given antigen concentration.
    /// Bell-shaped curve: rises to equivalence, then falls (prozone).
    pub fn signal(&self, antigen_conc: f64) -> f64 {
        if antigen_conc <= 0.0 {
            return 0.0;
        }
        let eq_pt: f64 = self.equivalence_point();
        let ratio: f64 = antigen_conc / eq_pt;
        // Model: S = 4·ratio / (1 + ratio)² normalized so peak = 1 at ratio = 1
        // With width parameter for broader/narrower transition
        let r_w: f64 = ratio.powf(1.0 / self.width);
        4.0 * r_w / (1.0 + r_w).powi(2)
    }

    /// Classify which zone a given antigen concentration falls in.
    pub fn zone(&self, antigen_conc: f64) -> PrecipitationZone {
        let eq_pt: f64 = self.equivalence_point();
        let lower: f64 = eq_pt * 0.8;
        let upper: f64 = eq_pt * 1.2;
        if antigen_conc < lower {
            PrecipitationZone::AntibodyExcess
        } else if antigen_conc > upper {
            PrecipitationZone::AntigenExcess
        } else {
            PrecipitationZone::Equivalence
        }
    }

    /// Check if antigen excess (prozone) is likely.
    pub fn is_prozone(&self, antigen_conc: f64) -> bool {
        self.zone(antigen_conc) == PrecipitationZone::AntigenExcess
    }

    /// Maximum signal value (at equivalence).
    pub fn max_signal(&self) -> f64 {
        self.signal(self.equivalence_point())
    }
}

// ---------------------------------------------------------------------------
// RateNephelometry
// ---------------------------------------------------------------------------

/// Rate (kinetic) nephelometry: measures speed of immune complex formation.
#[derive(Debug, Clone)]
pub struct RateNephelometry {
    /// Time points in seconds.
    pub times: Vec<f64>,
    /// Scatter signal values at each time point.
    pub signals: Vec<f64>,
}

impl RateNephelometry {
    pub fn new() -> Self {
        Self {
            times: Vec::new(),
            signals: Vec::new(),
        }
    }

    /// Add a measurement point.
    pub fn add_point(&mut self, time_s: f64, signal: f64) {
        self.times.push(time_s);
        self.signals.push(signal);
    }

    /// Build from parallel vectors of time and signal.
    pub fn from_data(times: Vec<f64>, signals: Vec<f64>) -> Self {
        Self { times, signals }
    }

    /// Compute the rate of change (dS/dt) using finite differences.
    pub fn rates(&self) -> Vec<f64> {
        let n: usize = self.signals.len();
        if n < 2 {
            return vec![];
        }
        let mut rates: Vec<f64> = Vec::with_capacity(n - 1);
        for i in 0..n - 1 {
            let dt: f64 = self.times[i + 1] - self.times[i];
            if dt > 0.0 {
                let rate: f64 = (self.signals[i + 1] - self.signals[i]) / dt;
                rates.push(rate);
            } else {
                rates.push(0.0);
            }
        }
        rates
    }

    /// Peak rate (maximum dS/dt).
    pub fn peak_rate(&self) -> f64 {
        let r = self.rates();
        if r.is_empty() {
            return 0.0;
        }
        let mut max_val: f64 = r[0];
        for &v in &r[1..] {
            if v > max_val {
                max_val = v;
            }
        }
        max_val
    }

    /// Time at which peak rate occurs.
    pub fn time_to_peak_rate(&self) -> f64 {
        let r = self.rates();
        if r.is_empty() {
            return 0.0;
        }
        let mut max_idx: usize = 0;
        let mut max_val: f64 = r[0];
        for (i, &v) in r.iter().enumerate().skip(1) {
            if v > max_val {
                max_val = v;
                max_idx = i;
            }
        }
        // Rate[i] corresponds to midpoint between times[i] and times[i+1]
        (self.times[max_idx] + self.times[max_idx + 1]) / 2.0
    }

    /// Fixed-time interval measurement: signal change between two time points.
    pub fn fixed_time_delta(&self, t_start: f64, t_end: f64) -> f64 {
        let s_start: f64 = self.interpolate(t_start);
        let s_end: f64 = self.interpolate(t_end);
        s_end - s_start
    }

    /// Linear interpolation of signal at arbitrary time.
    pub fn interpolate(&self, t: f64) -> f64 {
        let n: usize = self.times.len();
        if n == 0 {
            return 0.0;
        }
        if n == 1 || t <= self.times[0] {
            return self.signals[0];
        }
        if t >= self.times[n - 1] {
            return self.signals[n - 1];
        }
        for i in 0..n - 1 {
            if t >= self.times[i] && t <= self.times[i + 1] {
                let frac: f64 =
                    (t - self.times[i]) / (self.times[i + 1] - self.times[i]);
                return self.signals[i] + frac * (self.signals[i + 1] - self.signals[i]);
            }
        }
        self.signals[n - 1]
    }

    /// Average rate over entire measurement.
    pub fn average_rate(&self) -> f64 {
        let n: usize = self.times.len();
        if n < 2 {
            return 0.0;
        }
        let dt: f64 = self.times[n - 1] - self.times[0];
        if dt <= 0.0 {
            return 0.0;
        }
        (self.signals[n - 1] - self.signals[0]) / dt
    }

    /// Lag time: time before reaction rate exceeds a threshold.
    pub fn lag_time(&self, rate_threshold: f64) -> f64 {
        let r = self.rates();
        for (i, &v) in r.iter().enumerate() {
            if v > rate_threshold {
                return (self.times[i] + self.times[i + 1]) / 2.0;
            }
        }
        // Never exceeded threshold
        if self.times.is_empty() {
            0.0
        } else {
            self.times[self.times.len() - 1]
        }
    }
}

// ---------------------------------------------------------------------------
// EndpointNephelometry
// ---------------------------------------------------------------------------

/// Endpoint (equilibrium) nephelometry: final scatter intensity after reaction completion.
#[derive(Debug, Clone)]
pub struct EndpointNephelometry {
    /// Blank (reagent-only) signal.
    pub blank_signal: f64,
    /// Reaction signal at equilibrium.
    pub reaction_signal: f64,
    /// Incubation time in seconds.
    pub incubation_time_s: f64,
}

impl EndpointNephelometry {
    pub fn new(blank_signal: f64, reaction_signal: f64, incubation_time_s: f64) -> Self {
        Self {
            blank_signal,
            reaction_signal,
            incubation_time_s,
        }
    }

    /// Net scatter signal (reaction minus blank).
    pub fn net_signal(&self) -> f64 {
        self.reaction_signal - self.blank_signal
    }

    /// Signal-to-blank ratio.
    pub fn signal_to_blank_ratio(&self) -> f64 {
        if self.blank_signal <= 0.0 {
            return f64::INFINITY;
        }
        self.reaction_signal / self.blank_signal
    }

    /// Check if equilibrium was likely reached (signal stable).
    /// Takes a series of measurements at end of incubation.
    pub fn is_equilibrium(signals: &[f64], tolerance_pct: f64) -> bool {
        if signals.len() < 2 {
            return true;
        }
        let last: f64 = signals[signals.len() - 1];
        if last.abs() < 1e-12 {
            return true;
        }
        for &s in &signals[signals.len().saturating_sub(3)..] {
            let diff_pct: f64 = ((s - last) / last).abs() * 100.0;
            if diff_pct > tolerance_pct {
                return false;
            }
        }
        true
    }

    /// Concentration from a simple linear calibration (slope, intercept).
    pub fn concentration_linear(&self, slope: f64, intercept: f64) -> f64 {
        if slope.abs() < 1e-15 {
            return 0.0;
        }
        (self.net_signal() - intercept) / slope
    }
}

// ---------------------------------------------------------------------------
// CalibrationCurve
// ---------------------------------------------------------------------------

/// A single calibration point (concentration, signal).
#[derive(Debug, Clone, Copy)]
pub struct CalibrationPoint {
    pub concentration: f64,
    pub signal: f64,
}

/// Calibration curve fitting method.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FitMethod {
    /// Linear regression y = mx + b.
    Linear,
    /// 4-parameter logistic (Logit-Log).
    FourPL,
    /// Quadratic polynomial y = ax² + bx + c.
    Quadratic,
    /// Point-to-point linear interpolation (spline-like).
    PointToPoint,
}

/// Multi-point calibration curve for immunoassay quantitation.
#[derive(Debug, Clone)]
pub struct CalibrationCurve {
    /// Calibration points sorted by concentration.
    pub points: Vec<CalibrationPoint>,
    /// Fitting method used.
    pub method: FitMethod,
    /// Fitted parameters (interpretation depends on method).
    /// Linear: [slope, intercept]
    /// FourPL: [a, b, c, d]
    /// Quadratic: [a, b, c]
    pub params: Vec<f64>,
}

impl CalibrationCurve {
    /// Create and fit a calibration curve.
    pub fn fit(points: &[CalibrationPoint], method: FitMethod) -> Self {
        let mut sorted: Vec<CalibrationPoint> = points.to_vec();
        sorted.sort_by(|a, b| {
            a.concentration
                .partial_cmp(&b.concentration)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let params: Vec<f64> = match method {
            FitMethod::Linear => Self::fit_linear(&sorted),
            FitMethod::FourPL => Self::fit_4pl(&sorted),
            FitMethod::Quadratic => Self::fit_quadratic(&sorted),
            FitMethod::PointToPoint => vec![],
        };

        Self {
            points: sorted,
            method,
            params,
        }
    }

    fn fit_linear(pts: &[CalibrationPoint]) -> Vec<f64> {
        let n: f64 = pts.len() as f64;
        if n < 2.0 {
            return vec![1.0, 0.0];
        }
        let mut sx: f64 = 0.0;
        let mut sy: f64 = 0.0;
        let mut sxx: f64 = 0.0;
        let mut sxy: f64 = 0.0;
        for p in pts {
            sx += p.concentration;
            sy += p.signal;
            sxx += p.concentration * p.concentration;
            sxy += p.concentration * p.signal;
        }
        let denom: f64 = n * sxx - sx * sx;
        if denom.abs() < 1e-15 {
            return vec![0.0, sy / n];
        }
        let slope: f64 = (n * sxy - sx * sy) / denom;
        let intercept: f64 = (sy - slope * sx) / n;
        vec![slope, intercept]
    }

    fn fit_quadratic(pts: &[CalibrationPoint]) -> Vec<f64> {
        // Fit y = a·x² + b·x + c via normal equations
        let n: usize = pts.len();
        if n < 3 {
            let lin = Self::fit_linear(pts);
            return vec![0.0, lin[0], lin[1]];
        }
        // Build sums for normal equations
        let mut s0: f64 = 0.0;
        let mut s1: f64 = 0.0;
        let mut s2: f64 = 0.0;
        let mut s3: f64 = 0.0;
        let mut s4: f64 = 0.0;
        let mut sy0: f64 = 0.0;
        let mut sy1: f64 = 0.0;
        let mut sy2: f64 = 0.0;
        for p in pts {
            let x: f64 = p.concentration;
            let x2: f64 = x * x;
            s0 += 1.0;
            s1 += x;
            s2 += x2;
            s3 += x2 * x;
            s4 += x2 * x2;
            sy0 += p.signal;
            sy1 += x * p.signal;
            sy2 += x2 * p.signal;
        }
        // Solve 3x3 system using Cramer's rule
        // [s4 s3 s2] [a]   [sy2]
        // [s3 s2 s1] [b] = [sy1]
        // [s2 s1 s0] [c]   [sy0]
        let det: f64 = s4 * (s2 * s0 - s1 * s1) - s3 * (s3 * s0 - s1 * s2)
            + s2 * (s3 * s1 - s2 * s2);
        if det.abs() < 1e-15 {
            let lin = Self::fit_linear(pts);
            return vec![0.0, lin[0], lin[1]];
        }
        let a: f64 = (sy2 * (s2 * s0 - s1 * s1) - s3 * (sy1 * s0 - s1 * sy0)
            + s2 * (sy1 * s1 - s2 * sy0))
            / det;
        let b: f64 = (s4 * (sy1 * s0 - s1 * sy0) - sy2 * (s3 * s0 - s1 * s2)
            + s2 * (s3 * sy0 - sy1 * s2))
            / det;
        let c: f64 = (s4 * (s2 * sy0 - sy1 * s1) - s3 * (s3 * sy0 - sy1 * s2)
            + sy2 * (s3 * s1 - s2 * s2))
            / det;
        vec![a, b, c]
    }

    fn fit_4pl(pts: &[CalibrationPoint]) -> Vec<f64> {
        // Iterative 4PL fit: y = d + (a-d)/(1 + (x/c)^b)
        // Initialize from data extremes
        if pts.is_empty() {
            return vec![0.0, 1.0, 1.0, 1.0];
        }
        let a_init: f64 = pts[0].signal; // min conc signal
        let d_init: f64 = pts[pts.len() - 1].signal; // max conc signal
        let c_init: f64 = if pts.len() > 1 {
            (pts[0].concentration + pts[pts.len() - 1].concentration) / 2.0
        } else {
            pts[0].concentration.max(1.0)
        };
        let b_init: f64 = 1.0;

        let mut a: f64 = a_init;
        let mut b: f64 = b_init;
        let mut c: f64 = c_init.max(0.001);
        let mut d: f64 = d_init;

        // Simple Gauss-Newton-like iterations
        let step: f64 = 0.001;
        for _ in 0..200 {
            let residual_sum = |aa: f64, bb: f64, cc: f64, dd: f64| -> f64 {
                let mut ss: f64 = 0.0;
                for p in pts {
                    let y_hat: f64 = four_parameter_logistic(p.concentration, aa, bb, cc, dd);
                    let r: f64 = p.signal - y_hat;
                    ss += r * r;
                }
                ss
            };

            let base_err: f64 = residual_sum(a, b, c, d);
            if base_err < 1e-15 {
                break;
            }

            // Gradient descent on each parameter
            let da: f64 = (residual_sum(a + step, b, c, d) - base_err) / step;
            let db: f64 = (residual_sum(a, b + step, c, d) - base_err) / step;
            let dc: f64 = (residual_sum(a, b, c + step, d) - base_err) / step;
            let dd: f64 = (residual_sum(a, b, c, d + step) - base_err) / step;

            let grad_norm: f64 = (da * da + db * db + dc * dc + dd * dd).sqrt();
            if grad_norm < 1e-12 {
                break;
            }
            let lr: f64 = 0.01 * base_err / grad_norm;

            a -= lr * da;
            b -= lr * db;
            c -= lr * dc;
            d -= lr * dd;

            c = c.max(0.001);
            b = b.max(0.01);
        }

        vec![a, b, c, d]
    }

    /// Predict signal from concentration.
    pub fn predict(&self, concentration: f64) -> f64 {
        match self.method {
            FitMethod::Linear => {
                self.params[0] * concentration + self.params[1]
            }
            FitMethod::FourPL => {
                four_parameter_logistic(
                    concentration,
                    self.params[0],
                    self.params[1],
                    self.params[2],
                    self.params[3],
                )
            }
            FitMethod::Quadratic => {
                let x: f64 = concentration;
                self.params[0] * x * x + self.params[1] * x + self.params[2]
            }
            FitMethod::PointToPoint => self.interpolate_ptp(concentration),
        }
    }

    fn interpolate_ptp(&self, conc: f64) -> f64 {
        let n: usize = self.points.len();
        if n == 0 {
            return 0.0;
        }
        if n == 1 || conc <= self.points[0].concentration {
            return self.points[0].signal;
        }
        if conc >= self.points[n - 1].concentration {
            return self.points[n - 1].signal;
        }
        for i in 0..n - 1 {
            if conc >= self.points[i].concentration
                && conc <= self.points[i + 1].concentration
            {
                let frac: f64 = (conc - self.points[i].concentration)
                    / (self.points[i + 1].concentration - self.points[i].concentration);
                return self.points[i].signal
                    + frac * (self.points[i + 1].signal - self.points[i].signal);
            }
        }
        self.points[n - 1].signal
    }

    /// Inverse prediction: concentration from signal (bisection method).
    pub fn inverse_predict(&self, signal: f64) -> f64 {
        if self.points.is_empty() {
            return 0.0;
        }
        let lo_conc: f64 = self.points[0].concentration;
        let hi_conc: f64 = self.points[self.points.len() - 1].concentration * 2.0;
        self.bisect_inverse(signal, lo_conc.max(0.0), hi_conc.max(1.0), 50)
    }

    fn bisect_inverse(&self, target_signal: f64, mut lo: f64, mut hi: f64, max_iter: usize) -> f64 {
        for _ in 0..max_iter {
            let mid: f64 = (lo + hi) / 2.0;
            let s: f64 = self.predict(mid);
            if (s - target_signal).abs() < 1e-10 {
                return mid;
            }
            // Determine monotonicity from first and last points
            let increasing: bool = self.predict(hi) > self.predict(lo);
            if (increasing && s < target_signal) || (!increasing && s > target_signal) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        (lo + hi) / 2.0
    }

    /// Working range: concentration range with acceptable back-fit recovery.
    /// Returns (lower_limit, upper_limit) of linear range.
    pub fn working_range(&self, recovery_pct_tol: f64) -> (f64, f64) {
        if self.points.len() < 2 {
            return (0.0, 0.0);
        }
        let mut lower: f64 = self.points[0].concentration;
        let mut upper: f64 = self.points[self.points.len() - 1].concentration;

        for p in &self.points {
            let predicted_signal: f64 = self.predict(p.concentration);
            if p.signal.abs() < 1e-15 {
                continue;
            }
            let recovery: f64 = (predicted_signal / p.signal) * 100.0;
            if (recovery - 100.0).abs() > recovery_pct_tol {
                if p.concentration < (lower + upper) / 2.0 {
                    lower = p.concentration;
                } else {
                    upper = p.concentration;
                }
            }
        }
        (lower, upper)
    }

    /// Coefficient of determination R².
    pub fn r_squared(&self) -> f64 {
        let n: usize = self.points.len();
        if n < 2 {
            return 1.0;
        }
        let mean_signal: f64 = self.points.iter().map(|p| p.signal).sum::<f64>() / n as f64;
        let mut ss_res: f64 = 0.0;
        let mut ss_tot: f64 = 0.0;
        for p in &self.points {
            let predicted: f64 = self.predict(p.concentration);
            ss_res += (p.signal - predicted).powi(2);
            ss_tot += (p.signal - mean_signal).powi(2);
        }
        if ss_tot < 1e-15 {
            return 1.0;
        }
        1.0 - ss_res / ss_tot
    }
}

// ---------------------------------------------------------------------------
// ProteinsPanel
// ---------------------------------------------------------------------------

/// Clinical protein analyte identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProteinAnalyte {
    IgG,
    IgA,
    IgM,
    CRP,
    ComplementC3,
    ComplementC4,
    Transferrin,
    Albumin,
}

/// Reference range for a clinical protein analyte.
#[derive(Debug, Clone)]
pub struct ReferenceRange {
    pub analyte: ProteinAnalyte,
    pub lower_mg_dl: f64,
    pub upper_mg_dl: f64,
    pub unit: &'static str,
}

/// Clinical interpretation of a result against its reference range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClinicalInterpretation {
    BelowNormal,
    Normal,
    AboveNormal,
}

/// Clinical protein panel with standard reference ranges.
#[derive(Debug, Clone)]
pub struct ProteinsPanel {
    pub ranges: Vec<ReferenceRange>,
}

impl ProteinsPanel {
    /// Standard adult reference ranges.
    pub fn standard() -> Self {
        Self {
            ranges: vec![
                ReferenceRange {
                    analyte: ProteinAnalyte::IgG,
                    lower_mg_dl: 700.0,
                    upper_mg_dl: 1600.0,
                    unit: "mg/dL",
                },
                ReferenceRange {
                    analyte: ProteinAnalyte::IgA,
                    lower_mg_dl: 70.0,
                    upper_mg_dl: 400.0,
                    unit: "mg/dL",
                },
                ReferenceRange {
                    analyte: ProteinAnalyte::IgM,
                    lower_mg_dl: 40.0,
                    upper_mg_dl: 230.0,
                    unit: "mg/dL",
                },
                ReferenceRange {
                    analyte: ProteinAnalyte::CRP,
                    lower_mg_dl: 0.0,
                    upper_mg_dl: 0.5,
                    unit: "mg/dL",
                },
                ReferenceRange {
                    analyte: ProteinAnalyte::ComplementC3,
                    lower_mg_dl: 90.0,
                    upper_mg_dl: 180.0,
                    unit: "mg/dL",
                },
                ReferenceRange {
                    analyte: ProteinAnalyte::ComplementC4,
                    lower_mg_dl: 10.0,
                    upper_mg_dl: 40.0,
                    unit: "mg/dL",
                },
                ReferenceRange {
                    analyte: ProteinAnalyte::Transferrin,
                    lower_mg_dl: 200.0,
                    upper_mg_dl: 360.0,
                    unit: "mg/dL",
                },
                ReferenceRange {
                    analyte: ProteinAnalyte::Albumin,
                    lower_mg_dl: 3500.0,
                    upper_mg_dl: 5000.0,
                    unit: "mg/dL",
                },
            ],
        }
    }

    /// Get reference range for a specific analyte.
    pub fn get_range(&self, analyte: ProteinAnalyte) -> Option<&ReferenceRange> {
        self.ranges.iter().find(|r| r.analyte == analyte)
    }

    /// Interpret a result against reference range.
    pub fn interpret(&self, analyte: ProteinAnalyte, value_mg_dl: f64) -> ClinicalInterpretation {
        if let Some(r) = self.get_range(analyte) {
            if value_mg_dl < r.lower_mg_dl {
                ClinicalInterpretation::BelowNormal
            } else if value_mg_dl > r.upper_mg_dl {
                ClinicalInterpretation::AboveNormal
            } else {
                ClinicalInterpretation::Normal
            }
        } else {
            ClinicalInterpretation::Normal
        }
    }

    /// Check all analytes in a result set.
    pub fn interpret_panel(
        &self,
        results: &[(ProteinAnalyte, f64)],
    ) -> Vec<(ProteinAnalyte, f64, ClinicalInterpretation)> {
        results
            .iter()
            .map(|&(analyte, value)| (analyte, value, self.interpret(analyte, value)))
            .collect()
    }

    /// Number of analytes in this panel.
    pub fn analyte_count(&self) -> usize {
        self.ranges.len()
    }
}

// ---------------------------------------------------------------------------
// InterferenceDetection
// ---------------------------------------------------------------------------

/// Type of sample interference.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterferenceType {
    Lipemia,
    Hemolysis,
    Bilirubin,
    RheumatoidFactor,
}

/// Severity of an interference.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterferenceSeverity {
    None,
    Mild,
    Moderate,
    Severe,
}

/// Result of interference detection.
#[derive(Debug, Clone)]
pub struct InterferenceResult {
    pub interference_type: InterferenceType,
    pub severity: InterferenceSeverity,
    pub index_value: f64,
    pub correction_factor: f64,
}

/// Detection and correction of common nephelometric interferences.
#[derive(Debug, Clone)]
pub struct InterferenceDetection {
    /// Lipemia index threshold (NTU-equivalent).
    pub lipemia_mild: f64,
    pub lipemia_moderate: f64,
    pub lipemia_severe: f64,
    /// Hemolysis index threshold (mg/dL hemoglobin).
    pub hemolysis_mild: f64,
    pub hemolysis_moderate: f64,
    pub hemolysis_severe: f64,
    /// Bilirubin threshold (mg/dL).
    pub bilirubin_mild: f64,
    pub bilirubin_moderate: f64,
    pub bilirubin_severe: f64,
}

impl InterferenceDetection {
    /// Standard interference thresholds.
    pub fn standard() -> Self {
        Self {
            lipemia_mild: 100.0,
            lipemia_moderate: 300.0,
            lipemia_severe: 600.0,
            hemolysis_mild: 50.0,
            hemolysis_moderate: 200.0,
            hemolysis_severe: 500.0,
            bilirubin_mild: 2.0,
            bilirubin_moderate: 10.0,
            bilirubin_severe: 20.0,
        }
    }

    /// Check lipemia interference from sample blank signal.
    pub fn check_lipemia(&self, sample_blank_ntu: f64) -> InterferenceResult {
        let severity: InterferenceSeverity = if sample_blank_ntu >= self.lipemia_severe {
            InterferenceSeverity::Severe
        } else if sample_blank_ntu >= self.lipemia_moderate {
            InterferenceSeverity::Moderate
        } else if sample_blank_ntu >= self.lipemia_mild {
            InterferenceSeverity::Mild
        } else {
            InterferenceSeverity::None
        };

        // Correction factor: subtract blank scatter contribution
        let correction: f64 = if sample_blank_ntu > 0.0 {
            1.0 / (1.0 + sample_blank_ntu / 1000.0)
        } else {
            1.0
        };

        InterferenceResult {
            interference_type: InterferenceType::Lipemia,
            severity,
            index_value: sample_blank_ntu,
            correction_factor: correction,
        }
    }

    /// Check hemolysis interference.
    pub fn check_hemolysis(&self, hemoglobin_mg_dl: f64) -> InterferenceResult {
        let severity: InterferenceSeverity = if hemoglobin_mg_dl >= self.hemolysis_severe {
            InterferenceSeverity::Severe
        } else if hemoglobin_mg_dl >= self.hemolysis_moderate {
            InterferenceSeverity::Moderate
        } else if hemoglobin_mg_dl >= self.hemolysis_mild {
            InterferenceSeverity::Mild
        } else {
            InterferenceSeverity::None
        };

        let correction: f64 = if hemoglobin_mg_dl > 0.0 {
            1.0 / (1.0 + hemoglobin_mg_dl / 500.0)
        } else {
            1.0
        };

        InterferenceResult {
            interference_type: InterferenceType::Hemolysis,
            severity,
            index_value: hemoglobin_mg_dl,
            correction_factor: correction,
        }
    }

    /// Check bilirubin (icterus) interference.
    pub fn check_bilirubin(&self, bilirubin_mg_dl: f64) -> InterferenceResult {
        let severity: InterferenceSeverity = if bilirubin_mg_dl >= self.bilirubin_severe {
            InterferenceSeverity::Severe
        } else if bilirubin_mg_dl >= self.bilirubin_moderate {
            InterferenceSeverity::Moderate
        } else if bilirubin_mg_dl >= self.bilirubin_mild {
            InterferenceSeverity::Mild
        } else {
            InterferenceSeverity::None
        };

        let correction: f64 = if bilirubin_mg_dl > 0.0 {
            1.0 / (1.0 + bilirubin_mg_dl / 30.0)
        } else {
            1.0
        };

        InterferenceResult {
            interference_type: InterferenceType::Bilirubin,
            severity,
            index_value: bilirubin_mg_dl,
            correction_factor: correction,
        }
    }

    /// Check rheumatoid factor cross-reactivity.
    /// RF can form immune complexes that increase scatter non-specifically.
    pub fn check_rheumatoid_factor(&self, rf_iu_ml: f64) -> InterferenceResult {
        let severity: InterferenceSeverity = if rf_iu_ml >= 100.0 {
            InterferenceSeverity::Severe
        } else if rf_iu_ml >= 40.0 {
            InterferenceSeverity::Moderate
        } else if rf_iu_ml >= 20.0 {
            InterferenceSeverity::Mild
        } else {
            InterferenceSeverity::None
        };

        let correction: f64 = if rf_iu_ml > 0.0 {
            1.0 / (1.0 + rf_iu_ml / 200.0)
        } else {
            1.0
        };

        InterferenceResult {
            interference_type: InterferenceType::RheumatoidFactor,
            severity,
            index_value: rf_iu_ml,
            correction_factor: correction,
        }
    }

    /// Run all interference checks and return results.
    pub fn check_all(
        &self,
        lipemia_ntu: f64,
        hemoglobin_mg_dl: f64,
        bilirubin_mg_dl: f64,
        rf_iu_ml: f64,
    ) -> Vec<InterferenceResult> {
        vec![
            self.check_lipemia(lipemia_ntu),
            self.check_hemolysis(hemoglobin_mg_dl),
            self.check_bilirubin(bilirubin_mg_dl),
            self.check_rheumatoid_factor(rf_iu_ml),
        ]
    }

    /// Combined correction factor from all interferences.
    pub fn combined_correction(
        &self,
        lipemia_ntu: f64,
        hemoglobin_mg_dl: f64,
        bilirubin_mg_dl: f64,
        rf_iu_ml: f64,
    ) -> f64 {
        let results = self.check_all(lipemia_ntu, hemoglobin_mg_dl, bilirubin_mg_dl, rf_iu_ml);
        let mut factor: f64 = 1.0;
        for r in &results {
            factor *= r.correction_factor;
        }
        factor
    }

    /// Whether any interference is severe (result unreliable).
    pub fn has_severe_interference(results: &[InterferenceResult]) -> bool {
        results.iter().any(|r| r.severity == InterferenceSeverity::Severe)
    }
}

// ---------------------------------------------------------------------------
// LatexEnhanced
// ---------------------------------------------------------------------------

/// Latex-enhanced nephelometry: antibody-coated latex beads amplify scatter signal.
#[derive(Debug, Clone)]
pub struct LatexEnhanced {
    /// Latex bead diameter in nm (typically 100-300 nm).
    pub bead_diameter_nm: f64,
    /// Laser wavelength in nm.
    pub wavelength_nm: f64,
    /// Signal amplification factor vs non-enhanced.
    pub amplification_factor: f64,
    /// Lower detection limit improvement factor.
    pub sensitivity_gain: f64,
}

impl LatexEnhanced {
    pub fn new(bead_diameter_nm: f64, wavelength_nm: f64) -> Self {
        // Amplification depends on bead size relative to wavelength
        let x: f64 = size_parameter(bead_diameter_nm, wavelength_nm);
        // Larger beads scatter more: amplification ∝ x² for Mie regime
        let amp: f64 = 1.0 + x * x;
        Self {
            bead_diameter_nm,
            wavelength_nm,
            amplification_factor: amp,
            sensitivity_gain: amp.sqrt(),
        }
    }

    /// Standard configuration for high-sensitivity CRP (hs-CRP).
    pub fn hs_crp() -> Self {
        // Typical: 200 nm beads, 840 nm near-IR laser
        Self::new(200.0, 840.0)
    }

    /// Standard for immunoglobulin measurement.
    pub fn immunoglobulin() -> Self {
        Self::new(150.0, 670.0)
    }

    /// Enhanced scatter signal given a base (non-enhanced) signal.
    pub fn enhanced_signal(&self, base_signal: f64) -> f64 {
        base_signal * self.amplification_factor
    }

    /// Lower detection limit (LDL) given baseline noise.
    pub fn lower_detection_limit(&self, noise_level: f64, snr_required: f64) -> f64 {
        if self.amplification_factor <= 0.0 {
            return f64::INFINITY;
        }
        noise_level * snr_required / self.amplification_factor
    }

    /// Agglutination index: relative size of complexes formed.
    /// Higher = more agglutination = more scatter.
    pub fn agglutination_index(&self, analyte_conc: f64, antibody_conc: f64) -> f64 {
        if antibody_conc <= 0.0 {
            return 0.0;
        }
        let ratio: f64 = analyte_conc / antibody_conc;
        // Peak agglutination at equivalence (ratio ~ 1)
        4.0 * ratio / (1.0 + ratio).powi(2)
    }

    /// Size parameter of the latex beads.
    pub fn bead_size_parameter(&self) -> f64 {
        size_parameter(self.bead_diameter_nm, self.wavelength_nm)
    }

    /// Effective particle diameter after immune complex formation.
    /// Complexes are larger than bare beads.
    pub fn effective_diameter(&self, analyte_conc: f64, max_growth_nm: f64) -> f64 {
        // Growth saturates with Michaelis-Menten-like kinetics
        let km: f64 = 10.0; // half-max concentration
        let growth: f64 = max_growth_nm * analyte_conc / (km + analyte_conc);
        self.bead_diameter_nm + growth
    }

    /// Scatter signal ratio (enhanced / non-enhanced).
    pub fn signal_ratio(&self) -> f64 {
        self.amplification_factor
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-6;

    // === Helper function tests ===

    #[test]
    fn test_rayleigh_intensity_positive() {
        let i: f64 = rayleigh_intensity(50.0, 670.0, 1.05, PI / 2.0);
        assert!(i > 0.0);
    }

    #[test]
    fn test_rayleigh_intensity_zero_angle() {
        let i0: f64 = rayleigh_intensity(50.0, 670.0, 1.05, 0.0);
        let i90: f64 = rayleigh_intensity(50.0, 670.0, 1.05, PI / 2.0);
        // Forward scatter (0°) has angular factor 2, 90° has factor 1
        assert!(i0 > i90);
    }

    #[test]
    fn test_rayleigh_intensity_d6_dependence() {
        let i1: f64 = rayleigh_intensity(50.0, 670.0, 1.05, PI / 4.0);
        let i2: f64 = rayleigh_intensity(100.0, 670.0, 1.05, PI / 4.0);
        // Doubling diameter should increase intensity by 2^6 = 64
        let ratio: f64 = i2 / i1;
        assert!((ratio - 64.0).abs() < 0.1);
    }

    #[test]
    fn test_rayleigh_intensity_lambda4_dependence() {
        let i1: f64 = rayleigh_intensity(50.0, 400.0, 1.05, PI / 4.0);
        let i2: f64 = rayleigh_intensity(50.0, 800.0, 1.05, PI / 4.0);
        // Doubling wavelength should decrease intensity by 2^4 = 16
        let ratio: f64 = i1 / i2;
        assert!((ratio - 16.0).abs() < 0.1);
    }

    #[test]
    fn test_size_parameter_small() {
        let x: f64 = size_parameter(50.0, 670.0);
        assert!(x < 1.0);
    }

    #[test]
    fn test_size_parameter_large() {
        let x: f64 = size_parameter(1000.0, 670.0);
        assert!(x > 1.0);
    }

    #[test]
    fn test_size_parameter_value() {
        let x: f64 = size_parameter(670.0, 670.0);
        assert!((x - PI).abs() < TOL);
    }

    #[test]
    fn test_four_parameter_logistic_at_zero() {
        let y: f64 = four_parameter_logistic(0.0, 0.1, 1.0, 50.0, 1.0);
        assert!((y - 0.1).abs() < TOL);
    }

    #[test]
    fn test_four_parameter_logistic_at_ec50() {
        let y: f64 = four_parameter_logistic(50.0, 0.0, 1.0, 50.0, 1.0);
        // At x = c, y = d + (a-d)/2 = 1 + (0-1)/2 = 0.5
        assert!((y - 0.5).abs() < TOL);
    }

    #[test]
    fn test_four_parameter_logistic_high_conc() {
        let y: f64 = four_parameter_logistic(1e6, 0.0, 1.0, 50.0, 1.0);
        assert!((y - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_four_parameter_logistic_negative_x() {
        let y: f64 = four_parameter_logistic(-5.0, 0.1, 1.0, 50.0, 1.0);
        assert!((y - 0.1).abs() < TOL);
    }

    // === RayleighScattering tests ===

    #[test]
    fn test_rayleigh_scattering_new() {
        let rs = RayleighScattering::new(670.0, 1.05);
        assert!((rs.wavelength_nm - 670.0).abs() < TOL);
        assert!((rs.n_rel - 1.05).abs() < TOL);
    }

    #[test]
    fn test_rayleigh_scattering_intensity() {
        let rs = RayleighScattering::new(670.0, 1.05);
        let i: f64 = rs.intensity(50.0, PI / 2.0);
        assert!(i > 0.0);
    }

    #[test]
    fn test_rayleigh_ratio() {
        let rs = RayleighScattering::new(670.0, 1.05);
        let r: f64 = rs.rayleigh_ratio(50.0, PI / 4.0);
        assert!(r > 0.0);
    }

    #[test]
    fn test_dissymmetry_ratio_small_particle() {
        let rs = RayleighScattering::new(670.0, 1.05);
        let z: f64 = rs.dissymmetry_ratio(10.0);
        // For Rayleigh, z should be close to (1+cos²45°)/(1+cos²135°) = 1.5/1.5 = 1.0
        assert!((z - 1.0).abs() < TOL);
    }

    #[test]
    fn test_wavelength_factor() {
        let rs = RayleighScattering::new(670.0, 1.05);
        let wf: f64 = rs.wavelength_factor();
        let expected: f64 = 1.0 / 670.0_f64.powi(4);
        assert!((wf - expected).abs() / expected < TOL);
    }

    // === MieScattering tests ===

    #[test]
    fn test_mie_scattering_new() {
        let ms = MieScattering::new(670.0, 1.05);
        assert!((ms.wavelength_nm - 670.0).abs() < TOL);
    }

    #[test]
    fn test_mie_regime_rayleigh() {
        let ms = MieScattering::new(670.0, 1.05);
        assert_eq!(ms.regime(10.0), ScatteringRegime::Rayleigh);
    }

    #[test]
    fn test_mie_regime_rgd() {
        let ms = MieScattering::new(670.0, 1.05);
        assert_eq!(ms.regime(50.0), ScatteringRegime::RayleighGansDebye);
    }

    #[test]
    fn test_mie_regime_mie() {
        let ms = MieScattering::new(670.0, 1.05);
        assert_eq!(ms.regime(400.0), ScatteringRegime::Mie);
    }

    #[test]
    fn test_mie_regime_geometric() {
        let ms = MieScattering::new(670.0, 1.05);
        assert_eq!(ms.regime(5000.0), ScatteringRegime::GeometricOptics);
    }

    #[test]
    fn test_forward_scatter_enhancement() {
        let ms = MieScattering::new(670.0, 1.05);
        let enh_small: f64 = ms.forward_scatter_enhancement(10.0);
        let enh_large: f64 = ms.forward_scatter_enhancement(500.0);
        assert!(enh_large > enh_small);
    }

    #[test]
    fn test_angular_intensity_forward_bias() {
        let ms = MieScattering::new(670.0, 1.05);
        let i_fwd: f64 = ms.angular_intensity(300.0, 0.0);
        let i_back: f64 = ms.angular_intensity(300.0, PI);
        assert!(i_fwd > i_back);
    }

    #[test]
    fn test_turbidimetry_signal() {
        let ms = MieScattering::new(670.0, 1.05);
        let sig: f64 = ms.turbidimetry_signal(200.0);
        assert!(sig > 0.0);
    }

    #[test]
    fn test_nephelometry_signal_70deg() {
        let ms = MieScattering::new(670.0, 1.05);
        let sig: f64 = ms.nephelometry_signal(200.0, 70.0);
        assert!(sig > 0.0);
    }

    #[test]
    fn test_scattering_efficiency_small() {
        let ms = MieScattering::new(670.0, 1.05);
        let q: f64 = ms.scattering_efficiency(10.0);
        assert!(q > 0.0 && q < 0.1);
    }

    #[test]
    fn test_scattering_efficiency_large() {
        let ms = MieScattering::new(670.0, 1.05);
        let q: f64 = ms.scattering_efficiency(5000.0);
        // For large particles, approaches 2 (extinction paradox)
        assert!(q > 1.0 && q < 3.0);
    }

    // === HeidelbergerKendall tests ===

    #[test]
    fn test_hk_equivalence_point() {
        let hk = HeidelbergerKendall::new(10.0, 2.0, 1.5);
        assert!((hk.equivalence_point() - 20.0).abs() < TOL);
    }

    #[test]
    fn test_hk_max_signal_at_equivalence() {
        let hk = HeidelbergerKendall::new(10.0, 2.0, 1.0);
        let eq: f64 = hk.equivalence_point();
        let s_eq: f64 = hk.signal(eq);
        // Signal should be maximal (1.0) at equivalence
        assert!((s_eq - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_hk_signal_at_zero() {
        let hk = HeidelbergerKendall::new(10.0, 2.0, 1.5);
        assert!((hk.signal(0.0)).abs() < TOL);
    }

    #[test]
    fn test_hk_bell_shaped() {
        let hk = HeidelbergerKendall::new(10.0, 2.0, 1.0);
        let eq: f64 = hk.equivalence_point();
        let s_low: f64 = hk.signal(eq * 0.1);
        let s_eq: f64 = hk.signal(eq);
        let s_high: f64 = hk.signal(eq * 10.0);
        assert!(s_eq > s_low);
        assert!(s_eq > s_high);
    }

    #[test]
    fn test_hk_zone_antibody_excess() {
        let hk = HeidelbergerKendall::new(10.0, 2.0, 1.5);
        assert_eq!(hk.zone(5.0), PrecipitationZone::AntibodyExcess);
    }

    #[test]
    fn test_hk_zone_equivalence() {
        let hk = HeidelbergerKendall::new(10.0, 2.0, 1.5);
        assert_eq!(hk.zone(20.0), PrecipitationZone::Equivalence);
    }

    #[test]
    fn test_hk_zone_antigen_excess() {
        let hk = HeidelbergerKendall::new(10.0, 2.0, 1.5);
        assert_eq!(hk.zone(100.0), PrecipitationZone::AntigenExcess);
    }

    #[test]
    fn test_hk_prozone_detection() {
        let hk = HeidelbergerKendall::new(10.0, 2.0, 1.5);
        assert!(!hk.is_prozone(5.0));
        assert!(hk.is_prozone(100.0));
    }

    #[test]
    fn test_hk_igg_default() {
        let hk = HeidelbergerKendall::igg_default();
        assert!(hk.antibody_conc > 0.0);
        assert!(hk.max_signal() > 0.0);
    }

    // === RateNephelometry tests ===

    #[test]
    fn test_rate_neph_new() {
        let rn = RateNephelometry::new();
        assert!(rn.times.is_empty());
        assert!(rn.signals.is_empty());
    }

    #[test]
    fn test_rate_neph_add_point() {
        let mut rn = RateNephelometry::new();
        rn.add_point(0.0, 0.0);
        rn.add_point(1.0, 0.5);
        assert_eq!(rn.times.len(), 2);
    }

    #[test]
    fn test_rate_neph_rates() {
        let rn = RateNephelometry::from_data(
            vec![0.0, 1.0, 2.0, 3.0],
            vec![0.0, 1.0, 3.0, 3.5],
        );
        let rates = rn.rates();
        assert_eq!(rates.len(), 3);
        assert!((rates[0] - 1.0).abs() < TOL);
        assert!((rates[1] - 2.0).abs() < TOL);
        assert!((rates[2] - 0.5).abs() < TOL);
    }

    #[test]
    fn test_rate_neph_peak_rate() {
        let rn = RateNephelometry::from_data(
            vec![0.0, 1.0, 2.0, 3.0],
            vec![0.0, 1.0, 3.0, 3.5],
        );
        let peak: f64 = rn.peak_rate();
        assert!((peak - 2.0).abs() < TOL);
    }

    #[test]
    fn test_rate_neph_time_to_peak() {
        let rn = RateNephelometry::from_data(
            vec![0.0, 1.0, 2.0, 3.0],
            vec![0.0, 1.0, 3.0, 3.5],
        );
        let t: f64 = rn.time_to_peak_rate();
        // Peak rate is between t=1 and t=2, midpoint = 1.5
        assert!((t - 1.5).abs() < TOL);
    }

    #[test]
    fn test_rate_neph_fixed_time_delta() {
        let rn = RateNephelometry::from_data(
            vec![0.0, 1.0, 2.0, 3.0],
            vec![0.0, 1.0, 3.0, 3.5],
        );
        let delta: f64 = rn.fixed_time_delta(0.5, 2.5);
        // At t=0.5: interpolated = 0.5; at t=2.5: interpolated = 3.25
        assert!((delta - 2.75).abs() < TOL);
    }

    #[test]
    fn test_rate_neph_interpolate() {
        let rn = RateNephelometry::from_data(vec![0.0, 2.0], vec![0.0, 4.0]);
        let v: f64 = rn.interpolate(1.0);
        assert!((v - 2.0).abs() < TOL);
    }

    #[test]
    fn test_rate_neph_interpolate_clamped() {
        let rn = RateNephelometry::from_data(vec![1.0, 3.0], vec![2.0, 6.0]);
        assert!((rn.interpolate(0.0) - 2.0).abs() < TOL);
        assert!((rn.interpolate(5.0) - 6.0).abs() < TOL);
    }

    #[test]
    fn test_rate_neph_average_rate() {
        let rn = RateNephelometry::from_data(
            vec![0.0, 1.0, 2.0, 3.0],
            vec![0.0, 1.0, 3.0, 6.0],
        );
        let avg: f64 = rn.average_rate();
        assert!((avg - 2.0).abs() < TOL);
    }

    #[test]
    fn test_rate_neph_lag_time() {
        let rn = RateNephelometry::from_data(
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
            vec![0.0, 0.1, 0.2, 1.0, 2.0],
        );
        let lag: f64 = rn.lag_time(0.5);
        // Rate exceeds 0.5 between t=2 and t=3 (rate=0.8)
        assert!(lag > 2.0 && lag < 3.5);
    }

    #[test]
    fn test_rate_neph_empty() {
        let rn = RateNephelometry::new();
        assert_eq!(rn.peak_rate(), 0.0);
        assert_eq!(rn.time_to_peak_rate(), 0.0);
        assert_eq!(rn.average_rate(), 0.0);
    }

    // === EndpointNephelometry tests ===

    #[test]
    fn test_endpoint_net_signal() {
        let ep = EndpointNephelometry::new(0.5, 3.5, 600.0);
        assert!((ep.net_signal() - 3.0).abs() < TOL);
    }

    #[test]
    fn test_endpoint_signal_to_blank() {
        let ep = EndpointNephelometry::new(0.5, 3.5, 600.0);
        assert!((ep.signal_to_blank_ratio() - 7.0).abs() < TOL);
    }

    #[test]
    fn test_endpoint_equilibrium_reached() {
        assert!(EndpointNephelometry::is_equilibrium(
            &[1.0, 1.01, 1.005, 1.003],
            2.0
        ));
    }

    #[test]
    fn test_endpoint_equilibrium_not_reached() {
        assert!(!EndpointNephelometry::is_equilibrium(
            &[1.0, 1.5, 2.0, 2.5],
            2.0
        ));
    }

    #[test]
    fn test_endpoint_concentration_linear() {
        let ep = EndpointNephelometry::new(0.0, 1.0, 600.0);
        let conc: f64 = ep.concentration_linear(0.01, 0.0);
        assert!((conc - 100.0).abs() < TOL);
    }

    // === CalibrationCurve tests ===

    fn make_linear_points() -> Vec<CalibrationPoint> {
        vec![
            CalibrationPoint { concentration: 0.0, signal: 0.1 },
            CalibrationPoint { concentration: 50.0, signal: 0.6 },
            CalibrationPoint { concentration: 100.0, signal: 1.1 },
            CalibrationPoint { concentration: 200.0, signal: 2.1 },
        ]
    }

    #[test]
    fn test_calibration_linear_fit() {
        let pts = make_linear_points();
        let cal = CalibrationCurve::fit(&pts, FitMethod::Linear);
        assert!(cal.r_squared() > 0.99);
    }

    #[test]
    fn test_calibration_linear_predict() {
        let pts = make_linear_points();
        let cal = CalibrationCurve::fit(&pts, FitMethod::Linear);
        let pred: f64 = cal.predict(100.0);
        assert!((pred - 1.1).abs() < 0.1);
    }

    #[test]
    fn test_calibration_linear_inverse() {
        let pts = make_linear_points();
        let cal = CalibrationCurve::fit(&pts, FitMethod::Linear);
        let conc: f64 = cal.inverse_predict(1.1);
        assert!((conc - 100.0).abs() < 1.0);
    }

    #[test]
    fn test_calibration_quadratic_fit() {
        let pts = vec![
            CalibrationPoint { concentration: 0.0, signal: 0.0 },
            CalibrationPoint { concentration: 10.0, signal: 1.0 },
            CalibrationPoint { concentration: 20.0, signal: 4.0 },
            CalibrationPoint { concentration: 30.0, signal: 9.0 },
        ];
        let cal = CalibrationCurve::fit(&pts, FitMethod::Quadratic);
        let pred: f64 = cal.predict(20.0);
        assert!((pred - 4.0).abs() < 0.5);
    }

    #[test]
    fn test_calibration_ptp() {
        let pts = make_linear_points();
        let cal = CalibrationCurve::fit(&pts, FitMethod::PointToPoint);
        let pred: f64 = cal.predict(75.0);
        // Between 50->0.6 and 100->1.1, at 75 should be 0.85
        assert!((pred - 0.85).abs() < 0.01);
    }

    #[test]
    fn test_calibration_ptp_clamp() {
        let pts = make_linear_points();
        let cal = CalibrationCurve::fit(&pts, FitMethod::PointToPoint);
        assert!((cal.predict(-10.0) - 0.1).abs() < TOL);
        assert!((cal.predict(500.0) - 2.1).abs() < TOL);
    }

    #[test]
    fn test_calibration_4pl_fit() {
        // Generate 4PL data
        let pts: Vec<CalibrationPoint> = (0..6)
            .map(|i| {
                let conc: f64 = (i as f64) * 20.0;
                let sig: f64 = four_parameter_logistic(conc, 0.1, 1.5, 50.0, 1.0);
                CalibrationPoint {
                    concentration: conc,
                    signal: sig,
                }
            })
            .collect();
        let cal = CalibrationCurve::fit(&pts, FitMethod::FourPL);
        // Should fit reasonably well (gradient descent may not fully converge)
        let r2: f64 = cal.r_squared();
        assert!(r2 > 0.8, "R² = {} should be > 0.8", r2);
    }

    #[test]
    fn test_calibration_working_range() {
        let pts = make_linear_points();
        let cal = CalibrationCurve::fit(&pts, FitMethod::Linear);
        let (lo, hi) = cal.working_range(10.0);
        assert!(lo >= 0.0);
        assert!(hi <= 200.0);
    }

    #[test]
    fn test_calibration_r_squared_perfect() {
        let pts = vec![
            CalibrationPoint { concentration: 0.0, signal: 0.0 },
            CalibrationPoint { concentration: 1.0, signal: 1.0 },
            CalibrationPoint { concentration: 2.0, signal: 2.0 },
        ];
        let cal = CalibrationCurve::fit(&pts, FitMethod::Linear);
        assert!((cal.r_squared() - 1.0).abs() < 0.001);
    }

    // === ProteinsPanel tests ===

    #[test]
    fn test_panel_standard() {
        let panel = ProteinsPanel::standard();
        assert_eq!(panel.analyte_count(), 8);
    }

    #[test]
    fn test_panel_igg_range() {
        let panel = ProteinsPanel::standard();
        let r = panel.get_range(ProteinAnalyte::IgG).unwrap();
        assert!((r.lower_mg_dl - 700.0).abs() < TOL);
        assert!((r.upper_mg_dl - 1600.0).abs() < TOL);
    }

    #[test]
    fn test_panel_crp_range() {
        let panel = ProteinsPanel::standard();
        let r = panel.get_range(ProteinAnalyte::CRP).unwrap();
        assert!((r.upper_mg_dl - 0.5).abs() < TOL);
    }

    #[test]
    fn test_panel_interpret_normal() {
        let panel = ProteinsPanel::standard();
        assert_eq!(
            panel.interpret(ProteinAnalyte::IgG, 1000.0),
            ClinicalInterpretation::Normal
        );
    }

    #[test]
    fn test_panel_interpret_low() {
        let panel = ProteinsPanel::standard();
        assert_eq!(
            panel.interpret(ProteinAnalyte::IgG, 200.0),
            ClinicalInterpretation::BelowNormal
        );
    }

    #[test]
    fn test_panel_interpret_high() {
        let panel = ProteinsPanel::standard();
        assert_eq!(
            panel.interpret(ProteinAnalyte::IgG, 2000.0),
            ClinicalInterpretation::AboveNormal
        );
    }

    #[test]
    fn test_panel_crp_elevated() {
        let panel = ProteinsPanel::standard();
        assert_eq!(
            panel.interpret(ProteinAnalyte::CRP, 5.0),
            ClinicalInterpretation::AboveNormal
        );
    }

    #[test]
    fn test_panel_interpret_all() {
        let panel = ProteinsPanel::standard();
        let results = panel.interpret_panel(&[
            (ProteinAnalyte::IgG, 1000.0),
            (ProteinAnalyte::CRP, 0.1),
            (ProteinAnalyte::ComplementC3, 50.0),
        ]);
        assert_eq!(results.len(), 3);
        assert_eq!(results[0].2, ClinicalInterpretation::Normal);
        assert_eq!(results[1].2, ClinicalInterpretation::Normal);
        assert_eq!(results[2].2, ClinicalInterpretation::BelowNormal);
    }

    #[test]
    fn test_panel_c4_range() {
        let panel = ProteinsPanel::standard();
        let r = panel.get_range(ProteinAnalyte::ComplementC4).unwrap();
        assert!((r.lower_mg_dl - 10.0).abs() < TOL);
        assert!((r.upper_mg_dl - 40.0).abs() < TOL);
    }

    #[test]
    fn test_panel_transferrin_range() {
        let panel = ProteinsPanel::standard();
        let r = panel.get_range(ProteinAnalyte::Transferrin).unwrap();
        assert!((r.lower_mg_dl - 200.0).abs() < TOL);
    }

    #[test]
    fn test_panel_albumin_range() {
        let panel = ProteinsPanel::standard();
        let r = panel.get_range(ProteinAnalyte::Albumin).unwrap();
        assert!((r.lower_mg_dl - 3500.0).abs() < TOL);
        assert!((r.upper_mg_dl - 5000.0).abs() < TOL);
    }

    // === InterferenceDetection tests ===

    #[test]
    fn test_interference_standard() {
        let id = InterferenceDetection::standard();
        assert!(id.lipemia_mild > 0.0);
    }

    #[test]
    fn test_lipemia_none() {
        let id = InterferenceDetection::standard();
        let r = id.check_lipemia(10.0);
        assert_eq!(r.severity, InterferenceSeverity::None);
    }

    #[test]
    fn test_lipemia_mild() {
        let id = InterferenceDetection::standard();
        let r = id.check_lipemia(150.0);
        assert_eq!(r.severity, InterferenceSeverity::Mild);
    }

    #[test]
    fn test_lipemia_moderate() {
        let id = InterferenceDetection::standard();
        let r = id.check_lipemia(400.0);
        assert_eq!(r.severity, InterferenceSeverity::Moderate);
    }

    #[test]
    fn test_lipemia_severe() {
        let id = InterferenceDetection::standard();
        let r = id.check_lipemia(700.0);
        assert_eq!(r.severity, InterferenceSeverity::Severe);
    }

    #[test]
    fn test_hemolysis_none() {
        let id = InterferenceDetection::standard();
        let r = id.check_hemolysis(10.0);
        assert_eq!(r.severity, InterferenceSeverity::None);
    }

    #[test]
    fn test_hemolysis_severe() {
        let id = InterferenceDetection::standard();
        let r = id.check_hemolysis(600.0);
        assert_eq!(r.severity, InterferenceSeverity::Severe);
    }

    #[test]
    fn test_bilirubin_none() {
        let id = InterferenceDetection::standard();
        let r = id.check_bilirubin(0.5);
        assert_eq!(r.severity, InterferenceSeverity::None);
    }

    #[test]
    fn test_bilirubin_moderate() {
        let id = InterferenceDetection::standard();
        let r = id.check_bilirubin(12.0);
        assert_eq!(r.severity, InterferenceSeverity::Moderate);
    }

    #[test]
    fn test_rf_none() {
        let id = InterferenceDetection::standard();
        let r = id.check_rheumatoid_factor(5.0);
        assert_eq!(r.severity, InterferenceSeverity::None);
    }

    #[test]
    fn test_rf_severe() {
        let id = InterferenceDetection::standard();
        let r = id.check_rheumatoid_factor(150.0);
        assert_eq!(r.severity, InterferenceSeverity::Severe);
    }

    #[test]
    fn test_check_all() {
        let id = InterferenceDetection::standard();
        let results = id.check_all(50.0, 30.0, 1.0, 5.0);
        assert_eq!(results.len(), 4);
    }

    #[test]
    fn test_combined_correction() {
        let id = InterferenceDetection::standard();
        let cf: f64 = id.combined_correction(0.0, 0.0, 0.0, 0.0);
        assert!((cf - 1.0).abs() < TOL);
    }

    #[test]
    fn test_combined_correction_with_interference() {
        let id = InterferenceDetection::standard();
        let cf: f64 = id.combined_correction(200.0, 100.0, 5.0, 50.0);
        assert!(cf < 1.0);
        assert!(cf > 0.0);
    }

    #[test]
    fn test_has_severe_interference() {
        let id = InterferenceDetection::standard();
        let results = id.check_all(700.0, 0.0, 0.0, 0.0);
        assert!(InterferenceDetection::has_severe_interference(&results));
    }

    #[test]
    fn test_no_severe_interference() {
        let id = InterferenceDetection::standard();
        let results = id.check_all(10.0, 10.0, 0.5, 5.0);
        assert!(!InterferenceDetection::has_severe_interference(&results));
    }

    #[test]
    fn test_correction_factor_lipemia() {
        let id = InterferenceDetection::standard();
        let r = id.check_lipemia(500.0);
        // correction = 1/(1 + 500/1000) = 1/1.5 ≈ 0.667
        assert!((r.correction_factor - 1.0 / 1.5).abs() < 0.01);
    }

    // === LatexEnhanced tests ===

    #[test]
    fn test_latex_new() {
        let le = LatexEnhanced::new(200.0, 670.0);
        assert!(le.amplification_factor > 1.0);
    }

    #[test]
    fn test_latex_hs_crp() {
        let le = LatexEnhanced::hs_crp();
        assert!((le.bead_diameter_nm - 200.0).abs() < TOL);
        assert!((le.wavelength_nm - 840.0).abs() < TOL);
    }

    #[test]
    fn test_latex_immunoglobulin() {
        let le = LatexEnhanced::immunoglobulin();
        assert!((le.bead_diameter_nm - 150.0).abs() < TOL);
    }

    #[test]
    fn test_latex_enhanced_signal() {
        let le = LatexEnhanced::new(200.0, 670.0);
        let base: f64 = 1.0;
        let enhanced: f64 = le.enhanced_signal(base);
        assert!(enhanced > base);
    }

    #[test]
    fn test_latex_ldl() {
        let le = LatexEnhanced::new(200.0, 670.0);
        let ldl: f64 = le.lower_detection_limit(0.01, 3.0);
        assert!(ldl > 0.0);
        assert!(ldl < 0.03); // Better than non-enhanced (0.03)
    }

    #[test]
    fn test_latex_agglutination_peak() {
        let le = LatexEnhanced::new(200.0, 670.0);
        let ai_eq: f64 = le.agglutination_index(1.0, 1.0);
        let ai_low: f64 = le.agglutination_index(0.1, 1.0);
        let ai_high: f64 = le.agglutination_index(10.0, 1.0);
        // Peak at equivalence (ratio=1)
        assert!(ai_eq > ai_low);
        assert!(ai_eq > ai_high);
        assert!((ai_eq - 1.0).abs() < TOL);
    }

    #[test]
    fn test_latex_bead_size_parameter() {
        let le = LatexEnhanced::new(200.0, 670.0);
        let x: f64 = le.bead_size_parameter();
        let expected: f64 = PI * 200.0 / 670.0;
        assert!((x - expected).abs() < TOL);
    }

    #[test]
    fn test_latex_effective_diameter() {
        let le = LatexEnhanced::new(200.0, 670.0);
        let d0: f64 = le.effective_diameter(0.0, 100.0);
        let d100: f64 = le.effective_diameter(100.0, 100.0);
        assert!((d0 - 200.0).abs() < TOL); // No analyte, no growth
        assert!(d100 > 200.0); // With analyte, complex grows
        assert!(d100 < 300.0 + TOL); // Growth bounded by max_growth_nm
    }

    #[test]
    fn test_latex_signal_ratio() {
        let le = LatexEnhanced::new(200.0, 670.0);
        assert!((le.signal_ratio() - le.amplification_factor).abs() < TOL);
    }

    #[test]
    fn test_latex_larger_bead_more_amplification() {
        let small = LatexEnhanced::new(100.0, 670.0);
        let large = LatexEnhanced::new(300.0, 670.0);
        assert!(large.amplification_factor > small.amplification_factor);
    }

    #[test]
    fn test_latex_sensitivity_gain_positive() {
        let le = LatexEnhanced::new(200.0, 670.0);
        assert!(le.sensitivity_gain > 1.0);
    }
}
