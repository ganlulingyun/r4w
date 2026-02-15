//! # Pharmaceutical Dissolution Testing Signal Processor
//!
//! This module implements signal processing for pharmaceutical dissolution testing,
//! which measures the rate at which a drug substance dissolves from a solid dosage
//! form (tablet, capsule, etc.) into a dissolution medium. Dissolution testing is
//! a critical quality control procedure mandated by pharmacopeias (USP, EP, JP) and
//! regulatory agencies (FDA, EMA) for:
//!
//! - **Batch release testing**: Ensuring each manufactured batch meets dissolution specs
//! - **Bioequivalence studies**: Demonstrating generic drug equivalence via f2 similarity
//! - **Stability testing**: Monitoring dissolution changes over shelf life
//! - **Formulation development**: Optimizing drug release kinetics
//!
//! ## Measurement Principle
//!
//! UV-Vis spectrophotometry measures absorbance at a characteristic wavelength.
//! The Beer-Lambert law relates absorbance to concentration:
//!
//! ```text
//! A = epsilon * l * c
//! ```
//!
//! where `A` is absorbance (AU), `epsilon` is the molar extinction coefficient
//! (L/(mol*cm)), `l` is the optical path length (cm), and `c` is concentration.
//!
//! ## Dissolution Models
//!
//! Several mathematical models describe drug release kinetics:
//!
//! - **First-order**: `Q(t) = Q_inf * (1 - exp(-k*t))` -- most common for immediate release
//! - **Weibull**: `Q(t) = Q_inf * (1 - exp(-(t/a)^b))` -- flexible shape parameter
//! - **Higuchi**: `Q(t) = K_H * sqrt(t)` -- diffusion-controlled matrix systems
//! - **Hixson-Crowell**: `W_0^(1/3) - W_t^(1/3) = kappa * t` -- cube root erosion law
//!
//! ## USP Acceptance Criteria
//!
//! The USP defines three stages of acceptance testing (S1, S2, S3) with
//! progressively relaxed criteria for tablet dissolution:
//!
//! - **S1** (6 units): Each unit >= Q + 5%
//! - **S2** (12 units): Average >= Q + 5%, no unit < Q - 15%
//! - **S3** (24 units): Average >= Q, no more than 2 units < Q - 15%, none < Q - 25%
//!
//! ## Similarity and Difference Factors
//!
//! The FDA guidance uses f1 (difference factor) and f2 (similarity factor) to
//! compare dissolution profiles:
//!
//! ```text
//! f2 = 50 * log10(100 / sqrt(1 + (1/n) * sum((R_t - T_t)^2)))
//! f1 = 100 * sum(|R_t - T_t|) / sum(R_t)
//! ```
//!
//! Two profiles are considered similar when f2 is between 50 and 100.

/// USP apparatus types for dissolution testing.
///
/// Each apparatus provides different hydrodynamic conditions suited for
/// particular dosage forms.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ApparatusType {
    /// USP Apparatus 1 - Rotating basket (40-100 RPM).
    /// Suitable for capsules and floating dosage forms.
    BasketUSP1,
    /// USP Apparatus 2 - Rotating paddle (25-75 RPM).
    /// Most commonly used; suitable for tablets and capsules.
    PaddleUSP2,
    /// USP Apparatus 3 - Reciprocating cylinder (6-35 dpm).
    /// Used for extended-release dosage forms and bead-type products.
    ReciprocatingCylinderUSP3,
    /// USP Apparatus 4 - Flow-through cell (4-16 mL/min).
    /// Suitable for poorly soluble drugs and implants.
    FlowThroughUSP4,
}

/// Configuration for a dissolution test run.
///
/// Holds the experimental parameters that define how the dissolution
/// test is conducted, including the spectrophotometric measurement
/// wavelength, sampling schedule, and physical apparatus settings.
#[derive(Debug, Clone)]
pub struct DissolutionConfig {
    /// Measurement wavelength in nanometers (typically 200-800 nm UV-Vis range).
    pub wavelength_nm: f64,
    /// Time interval between sample pulls in minutes.
    pub sample_interval_min: f64,
    /// Volume of dissolution medium in each vessel (mL), typically 500 or 900.
    pub vessel_volume_ml: f64,
    /// Labeled dose of drug substance in the dosage form (mg).
    pub dose_mg: f64,
    /// pH of the dissolution medium (e.g., 1.2 for SGF, 6.8 for SIF, 4.5 for acetate buffer).
    pub media_ph: f64,
    /// Temperature of the dissolution medium in degrees Celsius.
    /// USP specifies 37.0 +/- 0.5 C for oral dosage forms.
    pub temperature_c: f64,
    /// Rotation speed: RPM for paddle/basket, dips per minute for reciprocating cylinder.
    pub rpm: f64,
    /// USP apparatus type used for the test.
    pub apparatus: ApparatusType,
}

impl Default for DissolutionConfig {
    /// Returns a default configuration matching a standard USP dissolution test:
    /// - 500 mg tablet in 900 mL pH 6.8 phosphate buffer
    /// - USP Apparatus 2 (paddle) at 50 RPM
    /// - 37.0 C, sampling every 5 minutes at 254 nm
    fn default() -> Self {
        Self {
            wavelength_nm: 254.0,
            sample_interval_min: 5.0,
            vessel_volume_ml: 900.0,
            dose_mg: 500.0,
            media_ph: 6.8,
            temperature_c: 37.0,
            rpm: 50.0,
            apparatus: ApparatusType::PaddleUSP2,
        }
    }
}

/// Pharmaceutical dissolution testing signal processor.
///
/// Provides methods for converting spectrophotometric absorbance readings
/// into dissolution profiles, fitting mathematical release models, computing
/// regulatory comparison metrics (f1/f2), and evaluating USP acceptance criteria.
///
/// # Example
///
/// ```
/// use r4w_core::drug_dissolution_monitor::{DissolutionMonitor, DissolutionConfig};
///
/// let config = DissolutionConfig::default();
/// let monitor = DissolutionMonitor::new(config);
///
/// // Convert absorbance to concentration via Beer-Lambert law
/// let concentration = monitor.absorbance_to_concentration(0.5, 100.0, 1.0);
/// assert!((concentration - 0.005).abs() < 1e-10);
/// ```
pub struct DissolutionMonitor {
    /// The dissolution test configuration.
    pub config: DissolutionConfig,
}

impl DissolutionMonitor {
    /// Create a new dissolution monitor with the given configuration.
    pub fn new(config: DissolutionConfig) -> Self {
        Self { config }
    }

    /// Convert an absorbance reading to concentration using the Beer-Lambert law.
    ///
    /// ```text
    /// A = epsilon * l * c  =>  c = A / (epsilon * l)
    /// ```
    ///
    /// # Arguments
    ///
    /// * `absorbance` - Measured absorbance in absorbance units (AU)
    /// * `extinction_coeff` - Molar extinction coefficient in L/(mol*cm)
    /// * `path_length_cm` - Optical path length in cm (typically 1.0 cm for standard cuvettes)
    ///
    /// # Returns
    ///
    /// Concentration in the same units implied by the extinction coefficient.
    /// If extinction_coeff is in L/(g*cm), result is in g/L (i.e., mg/mL).
    pub fn absorbance_to_concentration(
        &self,
        absorbance: f64,
        extinction_coeff: f64,
        path_length_cm: f64,
    ) -> f64 {
        if extinction_coeff * path_length_cm == 0.0 {
            return 0.0;
        }
        absorbance / (extinction_coeff * path_length_cm)
    }

    /// Convert a concentration reading to percent of drug dissolved.
    ///
    /// ```text
    /// %dissolved = (concentration * vessel_volume / dose) * 100
    /// ```
    ///
    /// # Arguments
    ///
    /// * `concentration_mg_ml` - Drug concentration in mg/mL
    /// * `vessel_volume_ml` - Total volume of dissolution medium in mL
    /// * `dose_mg` - Labeled drug dose in mg
    ///
    /// # Returns
    ///
    /// Percent of the dose dissolved (0-100+).
    pub fn concentration_to_percent_dissolved(
        &self,
        concentration_mg_ml: f64,
        vessel_volume_ml: f64,
        dose_mg: f64,
    ) -> f64 {
        if dose_mg == 0.0 {
            return 0.0;
        }
        (concentration_mg_ml * vessel_volume_ml / dose_mg) * 100.0
    }

    /// Apply cumulative volume correction to a dissolution profile.
    ///
    /// When aliquots are withdrawn for measurement and replaced with fresh medium,
    /// the drug removed in previous samples must be accounted for. The corrected
    /// value at each time point is:
    ///
    /// ```text
    /// Q_corrected(n) = Q_measured(n) + sum_{i=1}^{n-1} Q_measured(i) * (V_sample / V_vessel)
    /// ```
    ///
    /// # Arguments
    ///
    /// * `dissolved` - Measured percent dissolved values (uncorrected)
    /// * `sample_volume_ml` - Volume withdrawn at each sampling point (mL)
    /// * `vessel_volume_ml` - Total vessel volume (mL)
    ///
    /// # Returns
    ///
    /// Corrected percent dissolved values. Values will be >= the uncorrected values.
    pub fn cumulative_correction(
        &self,
        dissolved: &[f64],
        sample_volume_ml: f64,
        vessel_volume_ml: f64,
    ) -> Vec<f64> {
        if dissolved.is_empty() || vessel_volume_ml == 0.0 {
            return dissolved.to_vec();
        }
        let correction_factor = sample_volume_ml / vessel_volume_ml;
        let mut corrected = Vec::with_capacity(dissolved.len());
        let mut cumulative_removed = 0.0;

        for &d in dissolved {
            let corrected_val = d + cumulative_removed;
            corrected.push(corrected_val);
            cumulative_removed += d * correction_factor;
        }

        corrected
    }

    /// Fit a first-order kinetic model to dissolution data.
    ///
    /// ```text
    /// Q(t) = Q_inf * (1 - exp(-k * t))
    /// ```
    ///
    /// Uses linearized regression on `ln(Q_inf - Q)` vs `t`. The algorithm
    /// iteratively refines Q_inf starting from the maximum observed dissolution.
    ///
    /// # Arguments
    ///
    /// * `times` - Time points in minutes
    /// * `dissolved` - Corresponding percent dissolved values
    ///
    /// # Returns
    ///
    /// Tuple `(q_inf, k)` where `q_inf` is the asymptotic dissolution (%) and
    /// `k` is the first-order rate constant (1/min).
    pub fn fit_first_order(&self, times: &[f64], dissolved: &[f64]) -> (f64, f64) {
        let n = times.len().min(dissolved.len());
        if n < 2 {
            return (0.0, 0.0);
        }

        let max_q = dissolved[..n].iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        if max_q <= 0.0 {
            return (100.0, 0.0);
        }

        // Fine grid search over Q_inf, then for each Q_inf solve k via linearization.
        // ln(1 - Q/Q_inf) = -k*t  =>  y = -k*t  (through origin)
        // k = -sum(y_i * t_i) / sum(t_i^2)
        let mut best_k = 0.0;
        let mut best_q_inf = max_q;
        let mut best_residual = f64::MAX;

        // Search Q_inf from max_q * 1.001 up to max_q * 2.0, in 200 steps
        let q_lo = max_q * 1.001;
        let q_hi = max_q * 2.0;
        let num_steps = 200;

        for step in 0..=num_steps {
            let trial_q_inf = q_lo + (q_hi - q_lo) * step as f64 / num_steps as f64;

            // Linearize: y_i = ln(1 - Q_i / Q_inf), fit y = -k * t through origin
            let mut sum_yt = 0.0;
            let mut sum_tt = 0.0;
            let mut valid = true;

            for i in 0..n {
                let ratio = dissolved[i] / trial_q_inf;
                if ratio >= 1.0 {
                    valid = false;
                    break;
                }
                let y = (1.0 - ratio).ln();
                if !y.is_finite() {
                    valid = false;
                    break;
                }
                sum_yt += y * times[i];
                sum_tt += times[i] * times[i];
            }

            if !valid || sum_tt < 1e-30 {
                continue;
            }

            // k = -sum(y*t) / sum(t^2)
            let k = -sum_yt / sum_tt;
            if k <= 0.0 {
                continue;
            }

            // Compute sum of squared residuals
            let mut residual = 0.0;
            for i in 0..n {
                let predicted = trial_q_inf * (1.0 - (-k * times[i]).exp());
                let diff = dissolved[i] - predicted;
                residual += diff * diff;
            }

            if residual < best_residual {
                best_residual = residual;
                best_k = k;
                best_q_inf = trial_q_inf;
            }
        }

        (best_q_inf, best_k)
    }

    /// Fit a Weibull model to dissolution data.
    ///
    /// ```text
    /// Q(t) = Q_inf * (1 - exp(-(t/a)^b))
    /// ```
    ///
    /// The Weibull model is widely used because parameter `b` (shape) indicates
    /// the release mechanism:
    /// - b < 0.75: Fickian diffusion
    /// - 0.75 < b < 1.0: combined mechanisms
    /// - b > 1.0: complex release (sigmoid profile)
    ///
    /// Uses double-log linearization: `ln(-ln(1 - Q/Q_inf)) = b*ln(t) - b*ln(a)`
    ///
    /// # Returns
    ///
    /// Tuple `(q_inf, a, b)` -- asymptotic dissolution, scale parameter, shape parameter.
    pub fn fit_weibull(&self, times: &[f64], dissolved: &[f64]) -> (f64, f64, f64) {
        let n = times.len().min(dissolved.len());
        if n < 3 {
            return (100.0, 1.0, 1.0);
        }

        let max_q = dissolved[..n].iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let q_inf = max_q * 1.05;
        if q_inf <= 0.0 {
            return (100.0, 1.0, 1.0);
        }

        // Double-log linearization: ln(-ln(1 - Q/Qinf)) = b*ln(t) - b*ln(a)
        let mut x_vals = Vec::new(); // ln(t)
        let mut y_vals = Vec::new(); // ln(-ln(1 - Q/Qinf))

        for i in 0..n {
            if times[i] > 0.0 && dissolved[i] > 0.0 {
                let ratio = dissolved[i] / q_inf;
                if ratio > 0.0 && ratio < 1.0 {
                    let inner = 1.0 - ratio;
                    if inner > 0.0 {
                        let log_val = (-inner.ln()).ln();
                        if log_val.is_finite() {
                            x_vals.push(times[i].ln());
                            y_vals.push(log_val);
                        }
                    }
                }
            }
        }

        let m = x_vals.len();
        if m < 2 {
            return (q_inf, 1.0, 1.0);
        }

        // Linear regression: y = b*x + c, where c = -b*ln(a)
        let sum_x: f64 = x_vals.iter().sum();
        let sum_y: f64 = y_vals.iter().sum();
        let sum_xx: f64 = x_vals.iter().map(|x| x * x).sum();
        let sum_xy: f64 = x_vals.iter().zip(y_vals.iter()).map(|(x, y)| x * y).sum();
        let mf = m as f64;

        let denom = mf * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return (q_inf, 1.0, 1.0);
        }

        let b = (mf * sum_xy - sum_x * sum_y) / denom;
        let c = (sum_y - b * sum_x) / mf;

        // c = -b * ln(a)  =>  a = exp(-c/b)
        let a = if b.abs() > 1e-30 {
            (-c / b).exp()
        } else {
            1.0
        };

        let b = if b > 0.0 { b } else { 1.0 };
        let a = if a > 0.0 { a } else { 1.0 };

        (q_inf, a, b)
    }

    /// Fit the Higuchi square-root model to dissolution data.
    ///
    /// ```text
    /// Q(t) = K_H * sqrt(t)
    /// ```
    ///
    /// The Higuchi model describes drug release from a matrix system where
    /// diffusion is the rate-limiting step. A linear plot of Q vs sqrt(t)
    /// with zero intercept confirms diffusion-controlled release.
    ///
    /// # Returns
    ///
    /// The Higuchi dissolution constant `K_H` (%/min^0.5).
    pub fn fit_higuchi(&self, times: &[f64], dissolved: &[f64]) -> f64 {
        let n = times.len().min(dissolved.len());
        if n < 2 {
            return 0.0;
        }

        // Fit Q = K_H * sqrt(t) through the origin
        // Minimize sum (Q_i - K_H * sqrt(t_i))^2
        // dE/dK_H = 0  =>  K_H = sum(Q_i * sqrt(t_i)) / sum(t_i)
        let mut sum_q_sqrt_t = 0.0;
        let mut sum_t = 0.0;

        for i in 0..n {
            if times[i] >= 0.0 {
                let sqrt_t = times[i].sqrt();
                sum_q_sqrt_t += dissolved[i] * sqrt_t;
                sum_t += times[i]; // sqrt(t) * sqrt(t) = t
            }
        }

        if sum_t < 1e-30 {
            return 0.0;
        }

        sum_q_sqrt_t / sum_t
    }

    /// Fit the Hixson-Crowell cube root model to dissolution data.
    ///
    /// ```text
    /// W_0^(1/3) - W_t^(1/3) = kappa * t
    /// ```
    ///
    /// where `W_0` is the initial amount and `W_t` is the remaining amount at time `t`.
    /// Rewritten in terms of percent dissolved `Q`:
    ///
    /// ```text
    /// 100^(1/3) - (100 - Q)^(1/3) = kappa * t
    /// ```
    ///
    /// This model applies when dissolution occurs proportionally to the surface
    /// area of the dissolving particle (erosion-controlled).
    ///
    /// # Returns
    ///
    /// The Hixson-Crowell rate constant `kappa` (1/min).
    pub fn fit_hixson_crowell(&self, times: &[f64], dissolved: &[f64]) -> f64 {
        let n = times.len().min(dissolved.len());
        if n < 2 {
            return 0.0;
        }

        let w0_cbrt = 100.0_f64.cbrt(); // cube root of initial amount (100%)

        // y = W0^(1/3) - Wt^(1/3) = kappa * t
        // Fit through origin: kappa = sum(y_i * t_i) / sum(t_i^2)
        let mut sum_yt = 0.0;
        let mut sum_tt = 0.0;

        for i in 0..n {
            if times[i] > 0.0 {
                let remaining = (100.0 - dissolved[i]).max(0.0);
                let y = w0_cbrt - remaining.cbrt();
                sum_yt += y * times[i];
                sum_tt += times[i] * times[i];
            }
        }

        if sum_tt < 1e-30 {
            return 0.0;
        }

        sum_yt / sum_tt
    }

    /// Find the time to reach a given percent dissolved by linear interpolation.
    ///
    /// Commonly used to determine T50 (time to 50% dissolution) or T80 (time to
    /// 80% dissolution), which are key pharmacopeial endpoints.
    ///
    /// # Arguments
    ///
    /// * `profile` - Slice of `(time, percent_dissolved)` tuples, must be time-ordered
    /// * `percent` - Target percent dissolved (e.g., 50.0 for T50, 80.0 for T80)
    ///
    /// # Returns
    ///
    /// Interpolated time in minutes to reach the target. Returns `f64::INFINITY`
    /// if the target is never reached. Returns 0.0 if already exceeded at t=0.
    pub fn t_percent(&self, profile: &[(f64, f64)], percent: f64) -> f64 {
        if profile.is_empty() {
            return f64::INFINITY;
        }

        // Check if already exceeded at the first point
        if profile[0].1 >= percent {
            return profile[0].0;
        }

        // Find the interval where the target is crossed
        for i in 1..profile.len() {
            if profile[i].1 >= percent {
                // Linear interpolation between points i-1 and i
                let t0 = profile[i - 1].0;
                let t1 = profile[i].0;
                let q0 = profile[i - 1].1;
                let q1 = profile[i].1;
                let dq = q1 - q0;
                if dq.abs() < 1e-30 {
                    return t0;
                }
                return t0 + (percent - q0) * (t1 - t0) / dq;
            }
        }

        f64::INFINITY
    }

    /// Compute the f2 similarity factor between reference and test dissolution profiles.
    ///
    /// ```text
    /// f2 = 50 * log10(100 / sqrt(1 + (1/n) * sum((R_t - T_t)^2)))
    /// ```
    ///
    /// FDA guidance: f2 >= 50 indicates profile similarity. Identical profiles
    /// give f2 = 100.
    ///
    /// # Arguments
    ///
    /// * `reference` - Reference dissolution profile (percent dissolved at each time point)
    /// * `test` - Test dissolution profile (same time points)
    ///
    /// # Returns
    ///
    /// The f2 similarity factor (theoretically 0 to 100). Returns 100.0 for
    /// identical profiles.
    pub fn f2_similarity(&self, reference: &[f64], test: &[f64]) -> f64 {
        let n = reference.len().min(test.len());
        if n == 0 {
            return 0.0;
        }

        let mean_sq_diff: f64 = reference[..n]
            .iter()
            .zip(test[..n].iter())
            .map(|(r, t)| {
                let diff = r - t;
                diff * diff
            })
            .sum::<f64>()
            / n as f64;

        let inner = (1.0 + mean_sq_diff).sqrt();
        if inner < 1e-30 {
            return 100.0;
        }

        50.0 * (100.0 / inner).log10()
    }

    /// Compute the f1 difference factor between reference and test dissolution profiles.
    ///
    /// ```text
    /// f1 = 100 * sum(|R_t - T_t|) / sum(R_t)
    /// ```
    ///
    /// FDA guidance: f1 <= 15 indicates acceptable difference.
    ///
    /// # Arguments
    ///
    /// * `reference` - Reference dissolution profile
    /// * `test` - Test dissolution profile
    ///
    /// # Returns
    ///
    /// The f1 difference factor (0 = identical, higher = more different).
    pub fn f1_difference(&self, reference: &[f64], test: &[f64]) -> f64 {
        let n = reference.len().min(test.len());
        if n == 0 {
            return 0.0;
        }

        let sum_abs_diff: f64 = reference[..n]
            .iter()
            .zip(test[..n].iter())
            .map(|(r, t)| (r - t).abs())
            .sum();

        let sum_ref: f64 = reference[..n].iter().sum();

        if sum_ref.abs() < 1e-30 {
            return 0.0;
        }

        100.0 * sum_abs_diff / sum_ref
    }

    /// Evaluate whether a dissolution result passes USP acceptance criteria.
    ///
    /// The USP defines a staged testing protocol:
    ///
    /// - **Stage S1** (6 units): Each individual unit must be >= Q + 5%
    /// - **Stage S2** (12 units): Average of 12 must be >= Q, and no individual
    ///   unit < Q - 15%
    /// - **Stage S3** (24 units): Average of 24 must be >= Q, no more than 2
    ///   units < Q - 25%, and no unit < Q - 25%
    ///
    /// This simplified method evaluates a single unit value against the criterion
    /// for the specified stage.
    ///
    /// # Arguments
    ///
    /// * `dissolved_at_time` - Measured percent dissolved for one unit
    /// * `q_value` - The Q specification value (e.g., Q = 80 means 80% dissolved)
    /// * `stage` - USP testing stage (1, 2, or 3)
    ///
    /// # Returns
    ///
    /// `true` if the individual unit passes the criterion for the given stage.
    pub fn passes_usp_criterion(&self, dissolved_at_time: f64, q_value: f64, stage: usize) -> bool {
        match stage {
            1 => dissolved_at_time >= q_value + 5.0,
            2 => dissolved_at_time >= q_value - 15.0,
            3 => dissolved_at_time >= q_value - 25.0,
            _ => false,
        }
    }

    /// Generate a synthetic first-order dissolution profile for testing.
    ///
    /// ```text
    /// Q(t) = q_inf * (1 - exp(-k * t))
    /// ```
    ///
    /// # Arguments
    ///
    /// * `q_inf` - Asymptotic percent dissolved
    /// * `k` - First-order rate constant (1/min)
    /// * `times` - Time points at which to evaluate
    ///
    /// # Returns
    ///
    /// Vector of percent dissolved values at each time point.
    pub fn generate_first_order_profile(
        &self,
        q_inf: f64,
        k: f64,
        times: &[f64],
    ) -> Vec<f64> {
        times.iter().map(|&t| q_inf * (1.0 - (-k * t).exp())).collect()
    }

    /// Generate a synthetic Weibull dissolution profile for testing.
    ///
    /// ```text
    /// Q(t) = q_inf * (1 - exp(-(t/a)^b))
    /// ```
    pub fn generate_weibull_profile(
        &self,
        q_inf: f64,
        a: f64,
        b: f64,
        times: &[f64],
    ) -> Vec<f64> {
        times
            .iter()
            .map(|&t| {
                if t <= 0.0 || a <= 0.0 {
                    0.0
                } else {
                    q_inf * (1.0 - (-(t / a).powf(b)).exp())
                }
            })
            .collect()
    }

    /// Compute the dissolution efficiency (DE) as the area under the dissolution
    /// curve up to time T, expressed as a percentage of the area of the rectangle
    /// described by 100% dissolution at time T.
    ///
    /// ```text
    /// DE = (AUC_0_T / (100 * T)) * 100
    /// ```
    ///
    /// Uses the trapezoidal rule for integration.
    pub fn dissolution_efficiency(&self, profile: &[(f64, f64)]) -> f64 {
        if profile.len() < 2 {
            return 0.0;
        }

        let t_max = profile.last().unwrap().0;
        if t_max <= 0.0 {
            return 0.0;
        }

        // Trapezoidal integration
        let mut auc = 0.0;
        for i in 1..profile.len() {
            let dt = profile[i].0 - profile[i - 1].0;
            let avg_q = (profile[i].1 + profile[i - 1].1) / 2.0;
            auc += dt * avg_q;
        }

        (auc / (100.0 * t_max)) * 100.0
    }

    /// Compute the mean dissolution time (MDT) from a dissolution profile.
    ///
    /// ```text
    /// MDT = sum(t_mid * delta_Q) / sum(delta_Q)
    /// ```
    ///
    /// where `t_mid` is the midpoint between consecutive time points and
    /// `delta_Q` is the increment in percent dissolved over that interval.
    pub fn mean_dissolution_time(&self, profile: &[(f64, f64)]) -> f64 {
        if profile.len() < 2 {
            return 0.0;
        }

        let mut sum_t_dq = 0.0;
        let mut sum_dq = 0.0;

        for i in 1..profile.len() {
            let t_mid = (profile[i].0 + profile[i - 1].0) / 2.0;
            let dq = profile[i].1 - profile[i - 1].1;
            if dq > 0.0 {
                sum_t_dq += t_mid * dq;
                sum_dq += dq;
            }
        }

        if sum_dq < 1e-30 {
            return 0.0;
        }

        sum_t_dq / sum_dq
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_monitor() -> DissolutionMonitor {
        DissolutionMonitor::new(DissolutionConfig::default())
    }

    // ──── Beer-Lambert Law Tests ────

    #[test]
    fn test_beer_lambert_basic() {
        let m = default_monitor();
        // A = epsilon * l * c => c = A / (epsilon * l)
        // A = 0.5, epsilon = 100, l = 1.0 => c = 0.005
        let c = m.absorbance_to_concentration(0.5, 100.0, 1.0);
        assert!((c - 0.005).abs() < 1e-10);
    }

    #[test]
    fn test_beer_lambert_roundtrip() {
        let m = default_monitor();
        let epsilon = 150.0;
        let path_len = 1.0;
        let known_conc = 0.01; // 0.01 mg/mL
        // Forward: A = epsilon * l * c
        let absorbance = epsilon * path_len * known_conc;
        // Inverse: c = A / (epsilon * l)
        let recovered = m.absorbance_to_concentration(absorbance, epsilon, path_len);
        assert!((recovered - known_conc).abs() < 1e-12, "Roundtrip failed: {} vs {}", recovered, known_conc);
    }

    #[test]
    fn test_beer_lambert_zero_absorbance() {
        let m = default_monitor();
        let c = m.absorbance_to_concentration(0.0, 100.0, 1.0);
        assert!((c - 0.0).abs() < 1e-15);
    }

    #[test]
    fn test_beer_lambert_zero_extinction() {
        let m = default_monitor();
        let c = m.absorbance_to_concentration(0.5, 0.0, 1.0);
        assert!((c - 0.0).abs() < 1e-15);
    }

    #[test]
    fn test_beer_lambert_varying_path_length() {
        let m = default_monitor();
        let c1 = m.absorbance_to_concentration(0.5, 100.0, 1.0);
        let c2 = m.absorbance_to_concentration(0.5, 100.0, 2.0);
        // Doubling path length should halve the inferred concentration
        assert!((c1 - 2.0 * c2).abs() < 1e-12);
    }

    // ──── Concentration to Percent Dissolved ────

    #[test]
    fn test_percent_dissolved_basic() {
        let m = default_monitor();
        // 0.5 mg/mL * 900 mL / 500 mg * 100 = 90%
        let pct = m.concentration_to_percent_dissolved(0.5, 900.0, 500.0);
        assert!((pct - 90.0).abs() < 1e-10);
    }

    #[test]
    fn test_percent_dissolved_full() {
        let m = default_monitor();
        // concentration that gives 100%: dose/volume = 500/900
        let full_conc = 500.0 / 900.0;
        let pct = m.concentration_to_percent_dissolved(full_conc, 900.0, 500.0);
        assert!((pct - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_percent_dissolved_zero_dose() {
        let m = default_monitor();
        let pct = m.concentration_to_percent_dissolved(0.5, 900.0, 0.0);
        assert!((pct - 0.0).abs() < 1e-15);
    }

    // ──── Volume Correction ────

    #[test]
    fn test_volume_correction_increases_values() {
        let m = default_monitor();
        let dissolved = vec![10.0, 25.0, 45.0, 65.0, 80.0];
        let corrected = m.cumulative_correction(&dissolved, 5.0, 900.0);

        // Every corrected value (except the first) should be >= the uncorrected value
        for i in 1..dissolved.len() {
            assert!(
                corrected[i] >= dissolved[i],
                "Corrected[{}]={} should be >= dissolved[{}]={}",
                i, corrected[i], i, dissolved[i]
            );
        }
    }

    #[test]
    fn test_volume_correction_first_unchanged() {
        let m = default_monitor();
        let dissolved = vec![10.0, 25.0, 45.0];
        let corrected = m.cumulative_correction(&dissolved, 5.0, 900.0);
        assert!((corrected[0] - dissolved[0]).abs() < 1e-12);
    }

    #[test]
    fn test_volume_correction_zero_sample_volume() {
        let m = default_monitor();
        let dissolved = vec![10.0, 25.0, 45.0];
        let corrected = m.cumulative_correction(&dissolved, 0.0, 900.0);
        for (c, d) in corrected.iter().zip(dissolved.iter()) {
            assert!((c - d).abs() < 1e-12);
        }
    }

    #[test]
    fn test_volume_correction_empty_input() {
        let m = default_monitor();
        let corrected = m.cumulative_correction(&[], 5.0, 900.0);
        assert!(corrected.is_empty());
    }

    #[test]
    fn test_volume_correction_magnitude() {
        let m = default_monitor();
        // With 10 mL sample from 900 mL vessel, correction factor = 10/900 ~ 1.11%
        let dissolved = vec![50.0, 50.0, 50.0];
        let corrected = m.cumulative_correction(&dissolved, 10.0, 900.0);
        // First: 50.0, Second: 50 + 50*(10/900) = 50.556, Third: 50 + 50*(10/900) + 50.556*(10/900)...
        assert!((corrected[0] - 50.0).abs() < 1e-10);
        let expected_1 = 50.0 + 50.0 * (10.0 / 900.0);
        assert!((corrected[1] - expected_1).abs() < 1e-10);
    }

    // ──── First-Order Fit ────

    #[test]
    fn test_first_order_fit_recovery() {
        let m = default_monitor();
        let true_q_inf = 95.0;
        let true_k = 0.05;
        let times: Vec<f64> = (0..20).map(|i| i as f64 * 5.0).collect();
        let dissolved = m.generate_first_order_profile(true_q_inf, true_k, &times);

        let (q_inf, k) = m.fit_first_order(&times, &dissolved);

        // The fit should recover parameters reasonably well
        assert!(
            (q_inf - true_q_inf).abs() < 10.0,
            "q_inf={} should be near {}", q_inf, true_q_inf
        );
        assert!(
            (k - true_k).abs() < 0.02,
            "k={} should be near {}", k, true_k
        );
    }

    #[test]
    fn test_first_order_fit_fast_release() {
        let m = default_monitor();
        let times: Vec<f64> = (0..10).map(|i| i as f64 * 2.0).collect();
        let dissolved = m.generate_first_order_profile(100.0, 0.2, &times);
        let (q_inf, k) = m.fit_first_order(&times, &dissolved);
        // Fast release: k should be large
        assert!(k > 0.1, "k={} should be > 0.1 for fast release", k);
        assert!(q_inf > 80.0, "q_inf={} should be > 80%", q_inf);
    }

    #[test]
    fn test_first_order_fit_insufficient_data() {
        let m = default_monitor();
        let (q, k) = m.fit_first_order(&[5.0], &[30.0]);
        assert!((q - 0.0).abs() < 1e-10);
        assert!((k - 0.0).abs() < 1e-10);
    }

    // ──── Weibull Fit ────

    #[test]
    fn test_weibull_fit_shape_parameter() {
        let m = default_monitor();
        let true_a = 30.0;
        let true_b = 0.8;
        let q_inf = 100.0;
        let times: Vec<f64> = (1..25).map(|i| i as f64 * 5.0).collect();
        let dissolved = m.generate_weibull_profile(q_inf, true_a, true_b, &times);

        let (_, a, b) = m.fit_weibull(&times, &dissolved);

        assert!(
            (b - true_b).abs() < 0.3,
            "b={} should be near {}", b, true_b
        );
        assert!(a > 0.0, "a={} should be positive", a);
    }

    #[test]
    fn test_weibull_sigmoid_shape() {
        let m = default_monitor();
        // b > 1 gives sigmoid (S-shaped) profile
        let times: Vec<f64> = (1..30).map(|i| i as f64 * 3.0).collect();
        let dissolved = m.generate_weibull_profile(100.0, 40.0, 2.0, &times);
        let (_, _, b) = m.fit_weibull(&times, &dissolved);
        assert!(b > 1.0, "b={} should be > 1 for sigmoid profile", b);
    }

    #[test]
    fn test_weibull_insufficient_data() {
        let m = default_monitor();
        let (q, a, b) = m.fit_weibull(&[5.0, 10.0], &[20.0, 40.0]);
        // Should return defaults or reasonable values
        assert!(q > 0.0);
        assert!(a > 0.0);
        assert!(b > 0.0);
    }

    // ──── Higuchi Fit ────

    #[test]
    fn test_higuchi_sqrt_proportionality() {
        let m = default_monitor();
        let true_kh = 12.0;
        let times: Vec<f64> = (1..15).map(|i| i as f64 * 5.0).collect();
        let dissolved: Vec<f64> = times.iter().map(|&t| true_kh * t.sqrt()).collect();

        let kh = m.fit_higuchi(&times, &dissolved);

        assert!(
            (kh - true_kh).abs() < 0.5,
            "K_H={} should be near {}", kh, true_kh
        );
    }

    #[test]
    fn test_higuchi_positive_constant() {
        let m = default_monitor();
        let times = vec![5.0, 10.0, 15.0, 20.0, 30.0];
        let dissolved = vec![15.0, 22.0, 28.0, 33.0, 40.0];
        let kh = m.fit_higuchi(&times, &dissolved);
        assert!(kh > 0.0, "K_H should be positive for increasing dissolution");
    }

    #[test]
    fn test_higuchi_empty() {
        let m = default_monitor();
        let kh = m.fit_higuchi(&[], &[]);
        assert!((kh - 0.0).abs() < 1e-15);
    }

    // ──── Hixson-Crowell Fit ────

    #[test]
    fn test_hixson_crowell_fit() {
        let m = default_monitor();
        let true_kappa = 0.01;
        let w0_cbrt = 100.0_f64.cbrt();
        let times: Vec<f64> = (1..20).map(|i| i as f64 * 5.0).collect();
        // W0^(1/3) - Wt^(1/3) = kappa * t => Wt = (W0^(1/3) - kappa*t)^3
        // Q = 100 - Wt
        let dissolved: Vec<f64> = times
            .iter()
            .map(|&t| {
                let wt_cbrt = (w0_cbrt - true_kappa * t).max(0.0);
                100.0 - wt_cbrt.powi(3)
            })
            .collect();

        let kappa = m.fit_hixson_crowell(&times, &dissolved);
        assert!(
            (kappa - true_kappa).abs() < 0.005,
            "kappa={} should be near {}", kappa, true_kappa
        );
    }

    #[test]
    fn test_hixson_crowell_positive() {
        let m = default_monitor();
        let times = vec![5.0, 10.0, 15.0, 20.0, 30.0];
        let dissolved = vec![10.0, 20.0, 30.0, 40.0, 55.0];
        let kappa = m.fit_hixson_crowell(&times, &dissolved);
        assert!(kappa > 0.0, "kappa should be positive for dissolving drug");
    }

    // ──── T-percent ────

    #[test]
    fn test_t50_known_profile() {
        let m = default_monitor();
        // Linear profile: 10% per 10 minutes
        let profile = vec![(0.0, 0.0), (10.0, 10.0), (20.0, 20.0), (30.0, 30.0),
                           (40.0, 40.0), (50.0, 50.0), (60.0, 60.0)];
        let t50 = m.t_percent(&profile, 50.0);
        assert!((t50 - 50.0).abs() < 1e-10, "T50={} should be 50.0", t50);
    }

    #[test]
    fn test_t80_interpolation() {
        let m = default_monitor();
        let profile = vec![(0.0, 0.0), (15.0, 60.0), (30.0, 90.0), (45.0, 95.0)];
        let t80 = m.t_percent(&profile, 80.0);
        // Interpolate between (15, 60) and (30, 90): t = 15 + (80-60)/(90-60) * 15 = 25
        assert!((t80 - 25.0).abs() < 1e-10, "T80={} should be 25.0", t80);
    }

    #[test]
    fn test_t_percent_never_reached() {
        let m = default_monitor();
        let profile = vec![(0.0, 0.0), (10.0, 20.0), (20.0, 30.0)];
        let t90 = m.t_percent(&profile, 90.0);
        assert!(t90.is_infinite(), "T90 should be infinity when 90% never reached");
    }

    #[test]
    fn test_t_percent_already_reached() {
        let m = default_monitor();
        let profile = vec![(5.0, 85.0), (10.0, 90.0)];
        let t80 = m.t_percent(&profile, 80.0);
        assert!((t80 - 5.0).abs() < 1e-10, "T80 should be at first time point");
    }

    // ──── f2 Similarity Factor ────

    #[test]
    fn test_f2_identical_profiles() {
        let m = default_monitor();
        let reference = vec![20.0, 40.0, 60.0, 80.0, 95.0];
        let f2 = m.f2_similarity(&reference, &reference);
        assert!((f2 - 100.0).abs() < 1e-6, "f2={} should be 100.0 for identical profiles", f2);
    }

    #[test]
    fn test_f2_range() {
        let m = default_monitor();
        let reference = vec![20.0, 40.0, 60.0, 80.0, 95.0];
        let test = vec![10.0, 30.0, 50.0, 70.0, 85.0];
        let f2 = m.f2_similarity(&reference, &test);
        assert!(f2 >= 0.0 && f2 <= 100.0, "f2={} should be in [0, 100]", f2);
    }

    #[test]
    fn test_f2_similar_profiles() {
        let m = default_monitor();
        // Small differences should give f2 > 50
        let reference = vec![20.0, 40.0, 60.0, 80.0, 95.0];
        let test = vec![22.0, 42.0, 62.0, 82.0, 96.0];
        let f2 = m.f2_similarity(&reference, &test);
        assert!(f2 > 50.0, "f2={} should be > 50 for similar profiles", f2);
    }

    #[test]
    fn test_f2_dissimilar_profiles() {
        let m = default_monitor();
        let reference = vec![20.0, 40.0, 60.0, 80.0, 95.0];
        let test = vec![5.0, 10.0, 20.0, 30.0, 40.0];
        let f2 = m.f2_similarity(&reference, &test);
        assert!(f2 < 50.0, "f2={} should be < 50 for dissimilar profiles", f2);
    }

    #[test]
    fn test_f2_empty_profiles() {
        let m = default_monitor();
        let f2 = m.f2_similarity(&[], &[]);
        assert!((f2 - 0.0).abs() < 1e-10);
    }

    // ──── f1 Difference Factor ────

    #[test]
    fn test_f1_identical_profiles() {
        let m = default_monitor();
        let reference = vec![20.0, 40.0, 60.0, 80.0, 95.0];
        let f1 = m.f1_difference(&reference, &reference);
        assert!((f1 - 0.0).abs() < 1e-10, "f1={} should be 0 for identical profiles", f1);
    }

    #[test]
    fn test_f1_nonzero_difference() {
        let m = default_monitor();
        let reference = vec![20.0, 40.0, 60.0, 80.0, 95.0];
        let test = vec![15.0, 35.0, 55.0, 75.0, 90.0];
        let f1 = m.f1_difference(&reference, &test);
        assert!(f1 > 0.0, "f1={} should be > 0 for different profiles", f1);
    }

    #[test]
    fn test_f1_acceptable() {
        let m = default_monitor();
        // Small differences: f1 should be <= 15 per FDA guidance
        let reference = vec![20.0, 40.0, 60.0, 80.0, 95.0];
        let test = vec![21.0, 41.0, 61.0, 81.0, 96.0];
        let f1 = m.f1_difference(&reference, &test);
        assert!(f1 <= 15.0, "f1={} should be <= 15 for small differences", f1);
    }

    // ──── USP Acceptance Criteria ────

    #[test]
    fn test_usp_s1_pass() {
        let m = default_monitor();
        // S1: dissolved >= Q + 5%
        assert!(m.passes_usp_criterion(86.0, 80.0, 1)); // 86 >= 85 -> pass
    }

    #[test]
    fn test_usp_s1_fail() {
        let m = default_monitor();
        assert!(!m.passes_usp_criterion(84.0, 80.0, 1)); // 84 < 85 -> fail
    }

    #[test]
    fn test_usp_s1_borderline() {
        let m = default_monitor();
        assert!(m.passes_usp_criterion(85.0, 80.0, 1)); // 85 >= 85 -> pass (borderline)
    }

    #[test]
    fn test_usp_s2_pass() {
        let m = default_monitor();
        // S2: dissolved >= Q - 15%
        assert!(m.passes_usp_criterion(66.0, 80.0, 2)); // 66 >= 65 -> pass
    }

    #[test]
    fn test_usp_s2_fail() {
        let m = default_monitor();
        assert!(!m.passes_usp_criterion(64.0, 80.0, 2)); // 64 < 65 -> fail
    }

    #[test]
    fn test_usp_s3_pass() {
        let m = default_monitor();
        // S3: dissolved >= Q - 25%
        assert!(m.passes_usp_criterion(56.0, 80.0, 3)); // 56 >= 55 -> pass
    }

    #[test]
    fn test_usp_s3_fail() {
        let m = default_monitor();
        assert!(!m.passes_usp_criterion(54.0, 80.0, 3)); // 54 < 55 -> fail
    }

    #[test]
    fn test_usp_invalid_stage() {
        let m = default_monitor();
        assert!(!m.passes_usp_criterion(100.0, 80.0, 4));
    }

    // ──── Dissolution Efficiency ────

    #[test]
    fn test_dissolution_efficiency_full() {
        let m = default_monitor();
        // 100% dissolved immediately -> DE = 100%
        let profile = vec![(0.0, 100.0), (60.0, 100.0)];
        let de = m.dissolution_efficiency(&profile);
        assert!((de - 100.0).abs() < 1e-10, "DE={} should be 100%", de);
    }

    #[test]
    fn test_dissolution_efficiency_linear() {
        let m = default_monitor();
        // Linear from 0 to 100 over 60 min -> AUC = 0.5*60*100 = 3000
        // DE = 3000 / (100*60) * 100 = 50%
        let profile = vec![(0.0, 0.0), (60.0, 100.0)];
        let de = m.dissolution_efficiency(&profile);
        assert!((de - 50.0).abs() < 1e-10, "DE={} should be 50%", de);
    }

    // ──── Mean Dissolution Time ────

    #[test]
    fn test_mean_dissolution_time_linear() {
        let m = default_monitor();
        // Linear dissolution: equal increments at each interval
        let profile = vec![(0.0, 0.0), (10.0, 25.0), (20.0, 50.0), (30.0, 75.0), (40.0, 100.0)];
        let mdt = m.mean_dissolution_time(&profile);
        // Equal increments of 25% at midpoints 5, 15, 25, 35 -> MDT = (5+15+25+35)/4 = 20
        assert!((mdt - 20.0).abs() < 1e-10, "MDT={} should be 20.0", mdt);
    }

    // ──── Apparatus Type ────

    #[test]
    fn test_apparatus_types() {
        let config1 = DissolutionConfig {
            apparatus: ApparatusType::BasketUSP1,
            ..DissolutionConfig::default()
        };
        assert_eq!(config1.apparatus, ApparatusType::BasketUSP1);

        let config2 = DissolutionConfig {
            apparatus: ApparatusType::FlowThroughUSP4,
            ..DissolutionConfig::default()
        };
        assert_eq!(config2.apparatus, ApparatusType::FlowThroughUSP4);

        let config3 = DissolutionConfig {
            apparatus: ApparatusType::ReciprocatingCylinderUSP3,
            ..DissolutionConfig::default()
        };
        assert_eq!(config3.apparatus, ApparatusType::ReciprocatingCylinderUSP3);
    }

    // ──── Default Config ────

    #[test]
    fn test_default_config() {
        let config = DissolutionConfig::default();
        assert!((config.temperature_c - 37.0).abs() < 1e-10);
        assert!((config.vessel_volume_ml - 900.0).abs() < 1e-10);
        assert_eq!(config.apparatus, ApparatusType::PaddleUSP2);
        assert!((config.rpm - 50.0).abs() < 1e-10);
    }

    // ──── Generate Profile Tests ────

    #[test]
    fn test_generate_first_order_at_zero() {
        let m = default_monitor();
        let profile = m.generate_first_order_profile(100.0, 0.1, &[0.0]);
        assert!((profile[0] - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_generate_first_order_asymptote() {
        let m = default_monitor();
        let profile = m.generate_first_order_profile(95.0, 0.1, &[1000.0]);
        assert!((profile[0] - 95.0).abs() < 0.01, "Should approach Q_inf at large t");
    }

    #[test]
    fn test_generate_weibull_at_zero() {
        let m = default_monitor();
        let profile = m.generate_weibull_profile(100.0, 30.0, 1.0, &[0.0]);
        assert!((profile[0] - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_generate_weibull_monotonic() {
        let m = default_monitor();
        let times: Vec<f64> = (1..20).map(|i| i as f64 * 5.0).collect();
        let profile = m.generate_weibull_profile(100.0, 30.0, 1.5, &times);
        for i in 1..profile.len() {
            assert!(
                profile[i] >= profile[i - 1],
                "Weibull profile should be monotonically increasing"
            );
        }
    }
}
