// trace:FR-RHEO | ai:claude
//! # Rheology Viscoelastic Analyzer
//!
//! Rheological data analysis for characterizing viscoelastic materials via
//! oscillatory shear, creep, and flow curve measurements. Computes storage/loss
//! moduli (G'/G''), complex viscosity, relaxation spectra, and more.
//!
//! ## Science Background
//!
//! - **G' (storage modulus)**: elastic/solid-like response
//! - **G'' (loss modulus)**: viscous/liquid-like response
//! - **tan(delta) = G''/G'**: >1 liquid-like, <1 solid-like
//! - **Maxwell model**: spring + dashpot in series, tau = eta/G
//! - **Power law**: eta = K * gamma_dot^(n-1), n=1 Newtonian, n<1 shear-thinning
//! - **WLF**: log(a_T) = -C1*(T-Tr)/(C2+T-Tr) for polymer melts
//! - **Cox-Merz rule**: |eta*(omega)| = eta(gamma_dot) at omega = gamma_dot

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Result structs
// ---------------------------------------------------------------------------

/// Result of power-law fit: eta = K * gamma_dot^(n-1)
#[derive(Debug, Clone)]
pub struct PowerLawResult {
    /// Consistency index (Pa.s^n)
    pub k: f64,
    /// Flow behaviour index (dimensionless)
    pub n: f64,
}

/// Result of Carreau model fit:
/// eta = eta_inf + (eta_0 - eta_inf) * (1 + (lambda*gamma_dot)^2)^((n-1)/2)
#[derive(Debug, Clone)]
pub struct CarreauResult {
    pub eta_0: f64,
    pub eta_inf: f64,
    pub lambda: f64,
    pub n: f64,
}

/// Result of Cross model fit:
/// eta = eta_inf + (eta_0 - eta_inf) / (1 + (C*gamma_dot)^m)
#[derive(Debug, Clone)]
pub struct CrossResult {
    pub eta_0: f64,
    pub eta_inf: f64,
    pub c: f64,
    pub m: f64,
}

/// Result of WLF (Williams-Landel-Ferry) fit:
/// log(a_T) = -C1*(T - Tr) / (C2 + T - Tr)
#[derive(Debug, Clone)]
pub struct WlfResult {
    pub c1: f64,
    pub c2: f64,
    pub r_squared: f64,
}

/// Result of thixotropy analysis from three-interval test
#[derive(Debug, Clone)]
pub struct ThixotropyResult {
    /// Relative viscosity drop during high-shear interval
    pub viscosity_drop: f64,
    /// Percentage of viscosity recovered after high-shear interval
    pub recovery_percent: f64,
    /// Time to reach 90% recovery (seconds)
    pub recovery_time_s: f64,
}

/// Result of Cox-Merz rule check
#[derive(Debug, Clone)]
pub struct CoxMerzResult {
    /// Whether the Cox-Merz rule holds (max deviation < 10%)
    pub is_valid: bool,
    /// Maximum percentage deviation between |eta*| and eta
    pub max_deviation_percent: f64,
}

// ---------------------------------------------------------------------------
// OscillatoryData - Small amplitude oscillatory shear (SAOS)
// ---------------------------------------------------------------------------

/// Small amplitude oscillatory shear (SAOS) frequency sweep data.
///
/// Stores frequency-dependent storage modulus G', loss modulus G'', and
/// angular frequency omega.
#[derive(Debug, Clone)]
pub struct OscillatoryData {
    /// Angular frequency (rad/s)
    pub frequency_rad_per_s: Vec<f64>,
    /// Storage modulus G' (Pa)
    pub g_prime_pa: Vec<f64>,
    /// Loss modulus G'' (Pa)
    pub g_double_prime_pa: Vec<f64>,
}

impl OscillatoryData {
    /// Create new oscillatory data from matched frequency/G'/G'' vectors.
    ///
    /// # Panics
    /// Panics if vectors have different lengths or are empty.
    pub fn new(
        frequency_rad_per_s: Vec<f64>,
        g_prime_pa: Vec<f64>,
        g_double_prime_pa: Vec<f64>,
    ) -> Self {
        assert!(!frequency_rad_per_s.is_empty(), "Data must not be empty");
        assert_eq!(
            frequency_rad_per_s.len(),
            g_prime_pa.len(),
            "frequency and G' must have same length"
        );
        assert_eq!(
            frequency_rad_per_s.len(),
            g_double_prime_pa.len(),
            "frequency and G'' must have same length"
        );
        Self {
            frequency_rad_per_s,
            g_prime_pa,
            g_double_prime_pa,
        }
    }

    /// Number of data points.
    pub fn len(&self) -> usize {
        self.frequency_rad_per_s.len()
    }

    /// Whether the dataset is empty.
    pub fn is_empty(&self) -> bool {
        self.frequency_rad_per_s.is_empty()
    }

    /// Complex modulus magnitude |G*| = sqrt(G'^2 + G''^2) at given index.
    pub fn complex_modulus_at(&self, index: usize) -> f64 {
        let gp = self.g_prime_pa[index];
        let gpp = self.g_double_prime_pa[index];
        (gp * gp + gpp * gpp).sqrt()
    }

    /// Loss tangent tan(delta) = G'' / G' at given index.
    pub fn loss_tangent_at(&self, index: usize) -> f64 {
        self.g_double_prime_pa[index] / self.g_prime_pa[index]
    }

    /// Complex viscosity |eta*| = |G*| / omega at given index.
    pub fn complex_viscosity_at(&self, index: usize) -> f64 {
        self.complex_modulus_at(index) / self.frequency_rad_per_s[index]
    }

    /// Find crossover frequency where G' = G'' (by linear interpolation).
    /// Returns None if no crossover exists in the data.
    pub fn crossover_frequency(&self) -> Option<f64> {
        if self.len() < 2 {
            return None;
        }
        for i in 0..self.len() - 1 {
            let diff_a = self.g_prime_pa[i] - self.g_double_prime_pa[i];
            let diff_b = self.g_prime_pa[i + 1] - self.g_double_prime_pa[i + 1];
            // Sign change means crossover between i and i+1
            if diff_a * diff_b < 0.0 || diff_a == 0.0 {
                if diff_a == 0.0 {
                    return Some(self.frequency_rad_per_s[i]);
                }
                // Linear interpolation
                let t = diff_a / (diff_a - diff_b);
                let omega = self.frequency_rad_per_s[i]
                    + t * (self.frequency_rad_per_s[i + 1] - self.frequency_rad_per_s[i]);
                return Some(omega);
            }
        }
        // Check last point
        if (self.g_prime_pa.last().unwrap() - self.g_double_prime_pa.last().unwrap()).abs() < 1e-10
        {
            return Some(*self.frequency_rad_per_s.last().unwrap());
        }
        None
    }

    /// Modulus at the crossover frequency (G' = G'' = Gc).
    pub fn crossover_modulus(&self) -> Option<f64> {
        let omega_c = self.crossover_frequency()?;
        // Interpolate G' at crossover frequency
        for i in 0..self.len() - 1 {
            if self.frequency_rad_per_s[i] <= omega_c
                && omega_c <= self.frequency_rad_per_s[i + 1]
            {
                let t = (omega_c - self.frequency_rad_per_s[i])
                    / (self.frequency_rad_per_s[i + 1] - self.frequency_rad_per_s[i]);
                let g = self.g_prime_pa[i] + t * (self.g_prime_pa[i + 1] - self.g_prime_pa[i]);
                return Some(g);
            }
        }
        Some(self.g_prime_pa[0])
    }
}

// ---------------------------------------------------------------------------
// FlowCurve - Steady shear viscosity eta(gamma_dot)
// ---------------------------------------------------------------------------

/// Steady-state shear flow curve: viscosity eta as a function of shear rate gamma_dot.
#[derive(Debug, Clone)]
pub struct FlowCurve {
    /// Shear rate (1/s)
    pub shear_rate_per_s: Vec<f64>,
    /// Viscosity (Pa.s)
    pub viscosity_pa_s: Vec<f64>,
}

impl FlowCurve {
    /// Create a new flow curve from matched shear rate and viscosity vectors.
    pub fn new(shear_rate_per_s: Vec<f64>, viscosity_pa_s: Vec<f64>) -> Self {
        assert!(!shear_rate_per_s.is_empty(), "Data must not be empty");
        assert_eq!(
            shear_rate_per_s.len(),
            viscosity_pa_s.len(),
            "shear_rate and viscosity must have same length"
        );
        Self {
            shear_rate_per_s,
            viscosity_pa_s,
        }
    }

    pub fn len(&self) -> usize {
        self.shear_rate_per_s.len()
    }

    pub fn is_empty(&self) -> bool {
        self.shear_rate_per_s.is_empty()
    }

    /// Zero-shear viscosity eta_0 — estimated from the lowest-shear-rate plateau.
    /// Uses the average of the first few points (up to 3) where the shear rate is lowest.
    pub fn zero_shear_viscosity(&self) -> f64 {
        // Sort by shear rate and take average of lowest few
        let mut indexed: Vec<(f64, f64)> = self
            .shear_rate_per_s
            .iter()
            .zip(self.viscosity_pa_s.iter())
            .map(|(&r, &v)| (r, v))
            .collect();
        indexed.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        let n = indexed.len().min(3);
        let sum: f64 = indexed[..n].iter().map(|(_, v)| v).sum();
        sum / n as f64
    }

    /// Fit a power-law model eta = K * gamma_dot^(n-1) using log-log linear regression
    /// over the specified shear rate range.
    pub fn power_law_fit(&self, gamma_min: f64, gamma_max: f64) -> PowerLawResult {
        // Filter data within range
        let mut log_rate = Vec::new();
        let mut log_visc = Vec::new();
        for i in 0..self.len() {
            let r = self.shear_rate_per_s[i];
            if r >= gamma_min && r <= gamma_max && r > 0.0 && self.viscosity_pa_s[i] > 0.0 {
                log_rate.push(r.ln());
                log_visc.push(self.viscosity_pa_s[i].ln());
            }
        }
        if log_rate.len() < 2 {
            return PowerLawResult { k: 1.0, n: 1.0 };
        }
        // Linear regression: ln(eta) = ln(K) + (n-1)*ln(gamma_dot)
        let (slope, intercept) = linear_regression(&log_rate, &log_visc);
        let n = slope + 1.0; // n-1 = slope => n = slope + 1
        let k = intercept.exp();
        PowerLawResult { k, n }
    }

    /// Fit the Carreau model using iterative Gauss-Newton approach.
    /// eta = eta_inf + (eta_0 - eta_inf) * (1 + (lambda*gamma_dot)^2)^((n-1)/2)
    pub fn carreau_fit(&self) -> CarreauResult {
        // Initial estimates
        let eta_0 = self.zero_shear_viscosity();
        let eta_inf = *self
            .viscosity_pa_s
            .iter()
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(&0.0);
        let eta_inf = eta_inf * 0.1; // Lower bound estimate

        // Estimate lambda from midpoint of transition
        let mid_visc = (eta_0 + eta_inf) / 2.0;
        let mut lambda = 1.0;
        for i in 0..self.len() {
            if self.viscosity_pa_s[i] < mid_visc && self.shear_rate_per_s[i] > 0.0 {
                lambda = 1.0 / self.shear_rate_per_s[i];
                break;
            }
        }

        // Power-law index from high-shear region
        let pl = self.power_law_fit(
            self.shear_rate_per_s.last().unwrap_or(&1.0) * 0.1,
            *self.shear_rate_per_s.last().unwrap_or(&1e6),
        );
        let n = pl.n.max(0.01).min(1.5);

        // Simple iterative refinement (Levenberg-Marquardt-like)
        let mut params = [eta_0, eta_inf.max(0.0), lambda, n];
        let _step = 0.01;

        for _iter in 0..50 {
            let mut gradient = [0.0f64; 4];
            let mut total_err = 0.0;

            for i in 0..self.len() {
                let gd = self.shear_rate_per_s[i];
                let eta_meas = self.viscosity_pa_s[i];
                let arg = 1.0 + (params[2] * gd).powi(2);
                let eta_pred = params[1] + (params[0] - params[1]) * arg.powf((params[3] - 1.0) / 2.0);
                let err = eta_pred - eta_meas;
                total_err += err * err;

                // Partial derivatives
                let d_eta0 = arg.powf((params[3] - 1.0) / 2.0);
                let d_eta_inf = 1.0 - d_eta0;
                let d_lambda = if params[2] > 0.0 {
                    (params[0] - params[1])
                        * ((params[3] - 1.0) / 2.0)
                        * arg.powf((params[3] - 1.0) / 2.0 - 1.0)
                        * 2.0
                        * params[2]
                        * gd
                        * gd
                } else {
                    0.0
                };
                let d_n = if arg > 0.0 {
                    (params[0] - params[1]) * 0.5 * arg.powf((params[3] - 1.0) / 2.0) * arg.ln()
                } else {
                    0.0
                };

                gradient[0] += err * d_eta0;
                gradient[1] += err * d_eta_inf;
                gradient[2] += err * d_lambda;
                gradient[3] += err * d_n;
            }

            if total_err < 1e-20 {
                break;
            }

            let lr = 1e-8 / (total_err / self.len() as f64 + 1e-30);
            let lr = lr.min(0.01);
            for j in 0..4 {
                params[j] -= lr * gradient[j];
            }
            params[0] = params[0].max(1e-15);
            params[1] = params[1].max(0.0);
            params[2] = params[2].max(1e-15);
            params[3] = params[3].max(0.01).min(2.0);
        }

        CarreauResult {
            eta_0: params[0],
            eta_inf: params[1],
            lambda: params[2],
            n: params[3],
        }
    }

    /// Fit the Cross model:
    /// eta = eta_inf + (eta_0 - eta_inf) / (1 + (C*gamma_dot)^m)
    pub fn cross_model_fit(&self) -> CrossResult {
        let eta_0 = self.zero_shear_viscosity();
        let eta_inf_est = self
            .viscosity_pa_s
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min)
            * 0.1;

        // Estimate C from midpoint
        let mid_visc = (eta_0 + eta_inf_est) / 2.0;
        let mut c = 1.0;
        for i in 0..self.len() {
            if self.viscosity_pa_s[i] < mid_visc && self.shear_rate_per_s[i] > 0.0 {
                c = 1.0 / self.shear_rate_per_s[i];
                break;
            }
        }
        let m = 0.8; // initial guess

        let mut params = [eta_0, eta_inf_est.max(0.0), c, m];

        for _iter in 0..50 {
            let mut gradient = [0.0f64; 4];
            let mut total_err = 0.0;

            for i in 0..self.len() {
                let gd = self.shear_rate_per_s[i];
                let eta_meas = self.viscosity_pa_s[i];
                let cg_m = (params[2] * gd).powf(params[3]);
                let denom = 1.0 + cg_m;
                let eta_pred = params[1] + (params[0] - params[1]) / denom;
                let err = eta_pred - eta_meas;
                total_err += err * err;

                let d_eta0 = 1.0 / denom;
                let d_eta_inf = 1.0 - d_eta0;
                let d_c = if params[2] > 0.0 && gd > 0.0 {
                    -(params[0] - params[1]) * params[3] * cg_m / (params[2] * denom * denom)
                } else {
                    0.0
                };
                let d_m = if params[2] * gd > 0.0 {
                    -(params[0] - params[1]) * cg_m * (params[2] * gd).ln()
                        / (denom * denom)
                } else {
                    0.0
                };

                gradient[0] += err * d_eta0;
                gradient[1] += err * d_eta_inf;
                gradient[2] += err * d_c;
                gradient[3] += err * d_m;
            }

            if total_err < 1e-20 {
                break;
            }

            let lr = 1e-8 / (total_err / self.len() as f64 + 1e-30);
            let lr = lr.min(0.01);
            for j in 0..4 {
                params[j] -= lr * gradient[j];
            }
            params[0] = params[0].max(1e-15);
            params[1] = params[1].max(0.0);
            params[2] = params[2].max(1e-15);
            params[3] = params[3].max(0.1).min(3.0);
        }

        CrossResult {
            eta_0: params[0],
            eta_inf: params[1],
            c: params[2],
            m: params[3],
        }
    }

    /// Check if the material is shear-thinning (n < 1 from power law fit over full range).
    pub fn is_shear_thinning(&self) -> bool {
        let r_min = self
            .shear_rate_per_s
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let r_max = self
            .shear_rate_per_s
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let pl = self.power_law_fit(r_min, r_max);
        pl.n < 0.98 // small tolerance to avoid floating-point edge cases
    }
}

// ---------------------------------------------------------------------------
// CreepRecovery - Creep compliance J(t)
// ---------------------------------------------------------------------------

/// Creep-recovery compliance data J(t).
#[derive(Debug, Clone)]
pub struct CreepRecovery {
    /// Time (seconds)
    pub time_s: Vec<f64>,
    /// Compliance J (1/Pa)
    pub compliance_per_pa: Vec<f64>,
}

impl CreepRecovery {
    pub fn new(time_s: Vec<f64>, compliance_per_pa: Vec<f64>) -> Self {
        assert!(!time_s.is_empty(), "Data must not be empty");
        assert_eq!(
            time_s.len(),
            compliance_per_pa.len(),
            "time and compliance must have same length"
        );
        Self {
            time_s,
            compliance_per_pa,
        }
    }

    pub fn len(&self) -> usize {
        self.time_s.len()
    }

    pub fn is_empty(&self) -> bool {
        self.time_s.is_empty()
    }

    /// Instantaneous compliance J_0 = J(0).
    pub fn instantaneous_compliance(&self) -> f64 {
        self.compliance_per_pa[0]
    }

    /// Steady-state viscosity eta_0 from the slope of J(t) at long times.
    /// eta_0 = lim (t / J(t)) as t -> inf, or from dJ/dt = 1/eta_0.
    pub fn steady_state_viscosity(&self) -> f64 {
        let n = self.len();
        if n < 3 {
            return f64::INFINITY;
        }
        // Use the last third of data to estimate the linear slope
        let start = 2 * n / 3;
        let (slope, _) =
            linear_regression(&self.time_s[start..], &self.compliance_per_pa[start..]);
        if slope > 1e-30 {
            1.0 / slope
        } else {
            f64::INFINITY
        }
    }

    /// Recoverable compliance J_r — the compliance that is recovered after
    /// removing the applied stress (maximum J minus final J in a recovery test).
    pub fn recoverable_compliance(&self) -> f64 {
        let j_max = self
            .compliance_per_pa
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let j_final = *self.compliance_per_pa.last().unwrap_or(&0.0);
        j_max - j_final
    }

    /// Retardation spectrum — discrete spectrum of (tau, J) pairs fitted to the
    /// creep compliance using a generalized Kelvin-Voigt model:
    /// J(t) = J_0 + sum_i J_i * (1 - exp(-t/tau_i)) + t/eta_0
    pub fn retardation_spectrum(&self, num_elements: usize) -> Vec<(f64, f64)> {
        if num_elements == 0 || self.len() < 2 {
            return Vec::new();
        }

        let t_min = self.time_s.iter().cloned().fold(f64::INFINITY, f64::min);
        let t_max = self.time_s.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let t_min = t_min.max(1e-6);

        // Logarithmically spaced retardation times
        let log_min = t_min.ln();
        let log_max = t_max.ln();
        let mut taus = Vec::with_capacity(num_elements);
        for k in 0..num_elements {
            let frac = k as f64 / (num_elements as f64 - 1.0).max(1.0);
            taus.push((log_min + frac * (log_max - log_min)).exp());
        }

        let j0 = self.instantaneous_compliance();
        let eta0 = self.steady_state_viscosity();

        // Non-negative least squares by projected gradient
        let mut j_vals = vec![1e-6; num_elements];

        for _iter in 0..200 {
            for k in 0..num_elements {
                let mut numerator = 0.0;
                let mut denominator = 0.0;

                for i in 0..self.len() {
                    let t = self.time_s[i];
                    let j_meas = self.compliance_per_pa[i];

                    // Predicted J without element k
                    let mut j_pred = j0 + if eta0.is_finite() { t / eta0 } else { 0.0 };
                    for m in 0..num_elements {
                        if m != k {
                            j_pred += j_vals[m] * (1.0 - (-t / taus[m]).exp());
                        }
                    }

                    let basis = 1.0 - (-t / taus[k]).exp();
                    let residual = j_meas - j_pred;
                    numerator += residual * basis;
                    denominator += basis * basis;
                }

                if denominator > 1e-30 {
                    j_vals[k] = (numerator / denominator).max(0.0);
                }
            }
        }

        taus.into_iter().zip(j_vals).collect()
    }
}

// ---------------------------------------------------------------------------
// MaxwellModel - Generalized Maxwell (Wiechert) model
// ---------------------------------------------------------------------------

/// Generalized Maxwell (Wiechert) model with multiple (G_i, tau_i) elements.
#[derive(Debug, Clone)]
pub struct MaxwellModel {
    /// Maxwell elements: (modulus G_i in Pa, relaxation time tau_i in s)
    pub elements: Vec<(f64, f64)>,
}

impl MaxwellModel {
    /// Fit a generalized Maxwell model to oscillatory data using
    /// logarithmically spaced relaxation times and non-negative least squares.
    pub fn from_oscillatory(data: &OscillatoryData, num_elements: usize) -> Self {
        if num_elements == 0 || data.is_empty() {
            return Self {
                elements: Vec::new(),
            };
        }

        // Relaxation times: tau_i = 1/omega_i, logarithmically spaced
        let omega_min = data
            .frequency_rad_per_s
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let omega_max = data
            .frequency_rad_per_s
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);

        let log_tau_min = (1.0 / omega_max).ln();
        let log_tau_max = (1.0 / omega_min).ln();

        let mut taus = Vec::with_capacity(num_elements);
        for k in 0..num_elements {
            let frac = k as f64 / (num_elements as f64 - 1.0).max(1.0);
            taus.push((log_tau_min + frac * (log_tau_max - log_tau_min)).exp());
        }

        // Solve for G_i using alternating NNLS on G' and G'' simultaneously
        let n_data = data.len();
        let g_max: f64 = data.g_prime_pa.iter().cloned().fold(0.0_f64, f64::max);
        let mut g_vals = vec![g_max / num_elements as f64; num_elements];

        for _iter in 0..500 {
            for k in 0..num_elements {
                let tau_k = taus[k];
                let mut num = 0.0;
                let mut den = 0.0;

                for i in 0..n_data {
                    let omega = data.frequency_rad_per_s[i];
                    let wt = omega * tau_k;
                    let wt2 = wt * wt;
                    let denom_term = 1.0 + wt2;

                    // G' contribution: G_i * (wt)^2 / (1 + (wt)^2)
                    let basis_gp = wt2 / denom_term;
                    // G'' contribution: G_i * wt / (1 + (wt)^2)
                    let basis_gpp = wt / denom_term;

                    // Residuals without element k
                    let mut gp_pred = 0.0;
                    let mut gpp_pred = 0.0;
                    for m in 0..num_elements {
                        if m != k {
                            let wt_m = omega * taus[m];
                            let wt2_m = wt_m * wt_m;
                            let d_m = 1.0 + wt2_m;
                            gp_pred += g_vals[m] * wt2_m / d_m;
                            gpp_pred += g_vals[m] * wt_m / d_m;
                        }
                    }

                    let res_gp = data.g_prime_pa[i] - gp_pred;
                    let res_gpp = data.g_double_prime_pa[i] - gpp_pred;

                    num += res_gp * basis_gp + res_gpp * basis_gpp;
                    den += basis_gp * basis_gp + basis_gpp * basis_gpp;
                }

                if den > 1e-30 {
                    g_vals[k] = (num / den).max(0.0);
                }
            }
        }

        let elements: Vec<(f64, f64)> = g_vals
            .into_iter()
            .zip(taus)
            .filter(|(g, _)| *g > 1e-10)
            .collect();

        Self { elements }
    }

    /// Predict storage modulus G'(omega) = sum_i G_i * (omega*tau_i)^2 / (1 + (omega*tau_i)^2)
    pub fn predict_g_prime(&self, omega: f64) -> f64 {
        self.elements
            .iter()
            .map(|(g_i, tau_i)| {
                let wt = omega * tau_i;
                let wt2 = wt * wt;
                g_i * wt2 / (1.0 + wt2)
            })
            .sum()
    }

    /// Predict loss modulus G''(omega) = sum_i G_i * (omega*tau_i) / (1 + (omega*tau_i)^2)
    pub fn predict_g_double_prime(&self, omega: f64) -> f64 {
        self.elements
            .iter()
            .map(|(g_i, tau_i)| {
                let wt = omega * tau_i;
                g_i * wt / (1.0 + wt * wt)
            })
            .sum()
    }

    /// Relaxation modulus G(t) = sum_i G_i * exp(-t/tau_i)
    pub fn relaxation_modulus(&self, t: f64) -> f64 {
        self.elements
            .iter()
            .map(|(g_i, tau_i)| g_i * (-t / tau_i).exp())
            .sum()
    }

    /// Return the (G_i, tau_i) element pairs.
    pub fn elements(&self) -> &[(f64, f64)] {
        &self.elements
    }
}

// ---------------------------------------------------------------------------
// RelaxationSpectrum - Continuous relaxation spectrum H(tau)
// ---------------------------------------------------------------------------

/// Continuous relaxation spectrum H(tau) computation.
pub struct RelaxationSpectrum;

impl RelaxationSpectrum {
    /// Compute the continuous relaxation spectrum H(tau) from oscillatory data
    /// using Tikhonov regularization.
    ///
    /// Returns (tau, H) pairs on a logarithmic tau grid.
    pub fn from_oscillatory(data: &OscillatoryData, num_points: usize) -> Vec<(f64, f64)> {
        if num_points == 0 || data.is_empty() {
            return Vec::new();
        }

        let omega_min = data
            .frequency_rad_per_s
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let omega_max = data
            .frequency_rad_per_s
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);

        let log_tau_min = (1.0 / omega_max).ln();
        let log_tau_max = (1.0 / omega_min).ln();

        let mut taus = Vec::with_capacity(num_points);
        for k in 0..num_points {
            let frac = k as f64 / (num_points as f64 - 1.0).max(1.0);
            taus.push((log_tau_min + frac * (log_tau_max - log_tau_min)).exp());
        }

        let n_data = data.len();
        let n = num_points;

        // Build kernel matrix A and data vector b from G' and G'' data
        // G'(omega) = integral H(tau) * (omega*tau)^2 / (1+(omega*tau)^2) d(ln tau)
        // G''(omega) = integral H(tau) * (omega*tau) / (1+(omega*tau)^2) d(ln tau)
        // Discretize with trapezoidal weights on log-tau grid

        let d_ln_tau = if n > 1 {
            (log_tau_max - log_tau_min) / (n as f64 - 1.0)
        } else {
            1.0
        };

        // System: [A_gp; A_gpp] * h = [gp; gpp]
        let m = 2 * n_data; // rows
        let mut ata = vec![0.0; n * n]; // A^T A
        let mut atb = vec![0.0; n]; // A^T b

        for i in 0..n_data {
            let omega = data.frequency_rad_per_s[i];
            for j in 0..n {
                let wt = omega * taus[j];
                let wt2 = wt * wt;
                let d = 1.0 + wt2;
                let a_gp = wt2 / d * d_ln_tau;
                let a_gpp = wt / d * d_ln_tau;

                atb[j] += a_gp * data.g_prime_pa[i] + a_gpp * data.g_double_prime_pa[i];

                for k in 0..n {
                    let wt_k = omega * taus[k];
                    let wt2_k = wt_k * wt_k;
                    let d_k = 1.0 + wt2_k;
                    let a_gp_k = wt2_k / d_k * d_ln_tau;
                    let a_gpp_k = wt_k / d_k * d_ln_tau;
                    ata[j * n + k] += a_gp * a_gp_k + a_gpp * a_gpp_k;
                }
            }
        }

        // Tikhonov regularization: (A^T A + alpha * I) h = A^T b
        // Choose alpha based on matrix norm estimate
        let trace: f64 = (0..n).map(|i| ata[i * n + i]).sum();
        let alpha = trace / (m as f64) * 0.01; // regularization parameter

        for i in 0..n {
            ata[i * n + i] += alpha;
        }

        // Solve using Gauss-Seidel with non-negativity
        let mut h = vec![1.0; n];
        for _iter in 0..500 {
            for j in 0..n {
                let mut sum = atb[j];
                for k in 0..n {
                    if k != j {
                        sum -= ata[j * n + k] * h[k];
                    }
                }
                if ata[j * n + j].abs() > 1e-30 {
                    h[j] = (sum / ata[j * n + j]).max(0.0);
                }
            }
        }

        taus.into_iter().zip(h).collect()
    }

    /// Longest relaxation time — the tau where H(tau) is maximum.
    pub fn longest_relaxation_time(spectrum: &[(f64, f64)]) -> f64 {
        spectrum
            .iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .map(|(tau, _)| *tau)
            .unwrap_or(0.0)
    }

    /// Average relaxation time: <tau> = sum(H_i * tau_i) / sum(H_i)
    pub fn average_relaxation_time(spectrum: &[(f64, f64)]) -> f64 {
        let sum_h: f64 = spectrum.iter().map(|(_, h)| h).sum();
        if sum_h < 1e-30 {
            return 0.0;
        }
        let sum_ht: f64 = spectrum.iter().map(|(tau, h)| tau * h).sum();
        sum_ht / sum_h
    }
}

// ---------------------------------------------------------------------------
// TimeTemperatureSuperposition - TTS master curve construction
// ---------------------------------------------------------------------------

/// Time-Temperature Superposition (TTS) for constructing master curves
/// from oscillatory data at multiple temperatures.
pub struct TimeTemperatureSuperposition;

impl TimeTemperatureSuperposition {
    /// Compute horizontal shift factors a_T for each temperature relative to
    /// a reference temperature. Uses the crossover frequency method.
    ///
    /// Input: slice of (temperature_C, OscillatoryData) tuples.
    /// Output: (temperature_C, a_T) pairs.
    pub fn shift_factors(
        data_at_temps: &[(f64, OscillatoryData)],
        ref_temp_c: f64,
    ) -> Vec<(f64, f64)> {
        if data_at_temps.is_empty() {
            return Vec::new();
        }

        // Find reference dataset
        let ref_idx = data_at_temps
            .iter()
            .enumerate()
            .min_by(|(_, (t1, _)), (_, (t2, _))| {
                (t1 - ref_temp_c)
                    .abs()
                    .partial_cmp(&(t2 - ref_temp_c).abs())
                    .unwrap()
            })
            .map(|(i, _)| i)
            .unwrap_or(0);

        // Use the mean frequency of G' as a characteristic frequency for shift
        let ref_mean_gp: f64 =
            data_at_temps[ref_idx].1.g_prime_pa.iter().sum::<f64>() / data_at_temps[ref_idx].1.len() as f64;

        let mut results = Vec::with_capacity(data_at_temps.len());
        for (temp, osc_data) in data_at_temps {
            if osc_data.is_empty() {
                continue;
            }
            let mean_gp: f64 = osc_data.g_prime_pa.iter().sum::<f64>() / osc_data.len() as f64;

            // Shift factor: ratio of characteristic frequencies
            // Using the frequency at which G' matches a reference value
            // Simplified: use ratio of mean moduli as proxy for frequency shift
            let a_t = if ref_mean_gp > 1e-30 {
                // Find shift by matching G' curves in log space
                Self::compute_shift_factor(&data_at_temps[ref_idx].1, osc_data)
            } else {
                1.0
            };
            results.push((*temp, a_t));
        }

        results
    }

    /// Compute shift factor between two datasets by minimizing log-overlap error.
    fn compute_shift_factor(reference: &OscillatoryData, shifted: &OscillatoryData) -> f64 {
        // Find the log-shift that minimizes the difference between log(G') curves
        // Search in log space
        let mut best_shift = 0.0_f64;
        let mut best_error = f64::INFINITY;

        for trial in -40..=40 {
            let log_shift = trial as f64 * 0.1;
            let shift = (10.0_f64).powf(log_shift);
            let mut error = 0.0;
            let mut count = 0;

            for i in 0..shifted.len() {
                let shifted_omega = shifted.frequency_rad_per_s[i] * shift;
                // Find closest reference point
                for j in 0..reference.len() {
                    let log_ratio =
                        (shifted_omega / reference.frequency_rad_per_s[j]).abs().ln();
                    if log_ratio.abs() < 0.3 {
                        let diff = (shifted.g_prime_pa[i].ln() - reference.g_prime_pa[j].ln()).abs();
                        error += diff;
                        count += 1;
                    }
                }
            }

            if count > 0 {
                error /= count as f64;
                if error < best_error {
                    best_error = error;
                    best_shift = log_shift;
                }
            }
        }

        (10.0_f64).powf(best_shift)
    }

    /// Fit WLF equation: log10(a_T) = -C1*(T - Tr) / (C2 + T - Tr)
    pub fn wlf_fit(shift_factors: &[(f64, f64)], ref_temp_c: f64) -> WlfResult {
        if shift_factors.len() < 2 {
            return WlfResult {
                c1: 0.0,
                c2: 0.0,
                r_squared: 0.0,
            };
        }

        // Linearize: (T - Tr)/log10(a_T) = -C2/C1 - (T - Tr)/C1
        // Let Y = (T - Tr) / log10(a_T), X = (T - Tr)
        // Y = -(1/C1)*X - C2/C1
        let mut x_vals = Vec::new();
        let mut y_vals = Vec::new();
        for (temp, a_t) in shift_factors {
            let dt = temp - ref_temp_c;
            let log_at = a_t.log10();
            if log_at.abs() > 1e-10 && dt.abs() > 1e-10 {
                x_vals.push(dt);
                y_vals.push(dt / log_at);
            }
        }

        if x_vals.len() < 2 {
            return WlfResult {
                c1: 8.86,
                c2: 101.6,
                r_squared: 0.0,
            };
        }

        let (slope, intercept) = linear_regression(&x_vals, &y_vals);
        // slope = -1/C1, intercept = -C2/C1
        let c1 = if slope.abs() > 1e-30 {
            -1.0 / slope
        } else {
            8.86
        };
        let c2 = if slope.abs() > 1e-30 {
            intercept / slope
        } else {
            101.6
        };

        // R-squared
        let mean_y: f64 = y_vals.iter().sum::<f64>() / y_vals.len() as f64;
        let ss_tot: f64 = y_vals.iter().map(|y| (y - mean_y).powi(2)).sum();
        let ss_res: f64 = x_vals
            .iter()
            .zip(y_vals.iter())
            .map(|(x, y)| {
                let pred = slope * x + intercept;
                (y - pred).powi(2)
            })
            .sum();
        let r_squared = if ss_tot > 1e-30 {
            1.0 - ss_res / ss_tot
        } else {
            0.0
        };

        WlfResult {
            c1,
            c2,
            r_squared,
        }
    }

    /// Fit Arrhenius equation: ln(a_T) = Ea/R * (1/T - 1/Tr)
    /// Returns activation energy Ea in J/mol.
    pub fn arrhenius_fit(shift_factors: &[(f64, f64)]) -> f64 {
        let r_gas = 8.314; // J/(mol·K)
        let mut x_vals = Vec::new();
        let mut y_vals = Vec::new();

        for (temp_c, a_t) in shift_factors {
            let temp_k = temp_c + 273.15;
            if temp_k > 0.0 && *a_t > 0.0 {
                x_vals.push(1.0 / temp_k);
                y_vals.push(a_t.ln());
            }
        }

        if x_vals.len() < 2 {
            return 0.0;
        }

        let (slope, _) = linear_regression(&x_vals, &y_vals);
        slope * r_gas // Ea = slope * R
    }

    /// Construct master curve by shifting all datasets to the reference temperature.
    pub fn construct_master_curve(
        data: &[(f64, OscillatoryData)],
        shifts: &[(f64, f64)],
    ) -> OscillatoryData {
        let mut all_freq = Vec::new();
        let mut all_gp = Vec::new();
        let mut all_gpp = Vec::new();

        for (temp, osc) in data {
            // Find shift factor for this temperature
            let a_t = shifts
                .iter()
                .find(|(t, _)| (t - temp).abs() < 0.01)
                .map(|(_, a)| *a)
                .unwrap_or(1.0);

            for i in 0..osc.len() {
                all_freq.push(osc.frequency_rad_per_s[i] * a_t);
                all_gp.push(osc.g_prime_pa[i]);
                all_gpp.push(osc.g_double_prime_pa[i]);
            }
        }

        // Sort by frequency
        let mut indices: Vec<usize> = (0..all_freq.len()).collect();
        indices.sort_by(|&a, &b| all_freq[a].partial_cmp(&all_freq[b]).unwrap());

        let sorted_freq: Vec<f64> = indices.iter().map(|&i| all_freq[i]).collect();
        let sorted_gp: Vec<f64> = indices.iter().map(|&i| all_gp[i]).collect();
        let sorted_gpp: Vec<f64> = indices.iter().map(|&i| all_gpp[i]).collect();

        OscillatoryData::new(sorted_freq, sorted_gp, sorted_gpp)
    }
}

// ---------------------------------------------------------------------------
// AmplitudeSweep - Linear viscoelastic region (LVR)
// ---------------------------------------------------------------------------

/// Amplitude (strain) sweep data for determining the linear viscoelastic region.
#[derive(Debug, Clone)]
pub struct AmplitudeSweep {
    /// Strain (%)
    pub strain_percent: Vec<f64>,
    /// Storage modulus G' (Pa)
    pub g_prime: Vec<f64>,
    /// Loss modulus G'' (Pa)
    pub g_double_prime: Vec<f64>,
}

impl AmplitudeSweep {
    pub fn new(strain_percent: Vec<f64>, g_prime: Vec<f64>, g_double_prime: Vec<f64>) -> Self {
        assert!(!strain_percent.is_empty(), "Data must not be empty");
        assert_eq!(strain_percent.len(), g_prime.len());
        assert_eq!(strain_percent.len(), g_double_prime.len());
        Self {
            strain_percent,
            g_prime,
            g_double_prime,
        }
    }

    pub fn len(&self) -> usize {
        self.strain_percent.len()
    }

    pub fn is_empty(&self) -> bool {
        self.strain_percent.is_empty()
    }

    /// Critical strain — the strain at which G' deviates by more than 5%
    /// from its plateau value (end of the linear viscoelastic region).
    pub fn critical_strain(&self) -> f64 {
        let plateau = self.lvr_modulus();
        let threshold = 0.95 * plateau; // 5% deviation

        for i in 0..self.len() {
            if self.g_prime[i] < threshold {
                if i > 0 {
                    // Interpolate between i-1 and i
                    let t = (threshold - self.g_prime[i - 1])
                        / (self.g_prime[i] - self.g_prime[i - 1]);
                    return self.strain_percent[i - 1]
                        + t * (self.strain_percent[i] - self.strain_percent[i - 1]);
                }
                return self.strain_percent[i];
            }
        }
        // No deviation found — return last strain
        *self.strain_percent.last().unwrap_or(&0.0)
    }

    /// Yield stress — the stress at the G'/G'' crossover point.
    /// Requires the corresponding stress values.
    pub fn yield_stress(&self, stress: &[f64]) -> f64 {
        assert_eq!(stress.len(), self.len());
        for i in 0..self.len() - 1 {
            let diff_a = self.g_prime[i] - self.g_double_prime[i];
            let diff_b = self.g_prime[i + 1] - self.g_double_prime[i + 1];
            if diff_a * diff_b < 0.0 {
                let t = diff_a / (diff_a - diff_b);
                return stress[i] + t * (stress[i + 1] - stress[i]);
            }
        }
        // No crossover — return last stress
        *stress.last().unwrap_or(&0.0)
    }

    /// Plateau G' in the linear viscoelastic region (average of first few points
    /// where G' is approximately constant).
    pub fn lvr_modulus(&self) -> f64 {
        let n = self.len().min(5).max(1);
        let sum: f64 = self.g_prime[..n].iter().sum();
        sum / n as f64
    }
}

// ---------------------------------------------------------------------------
// ThixotropyAnalyzer - Time-dependent viscosity
// ---------------------------------------------------------------------------

/// Thixotropy analysis for time-dependent viscosity behaviour.
pub struct ThixotropyAnalyzer;

impl ThixotropyAnalyzer {
    /// Analyze a three-interval thixotropy test (3ITT):
    /// 1. Low shear rate (rest state)
    /// 2. High shear rate (structure breakdown)
    /// 3. Low shear rate again (recovery)
    pub fn three_interval_test(
        low_rate: &FlowCurve,
        high_rate: &FlowCurve,
        recovery: &FlowCurve,
    ) -> ThixotropyResult {
        let eta_initial = low_rate.viscosity_pa_s.last().copied().unwrap_or(1.0);
        let eta_broken = high_rate.viscosity_pa_s.last().copied().unwrap_or(0.1);
        let eta_recovered = recovery.viscosity_pa_s.last().copied().unwrap_or(0.5);

        let viscosity_drop = if eta_initial > 1e-30 {
            (eta_initial - eta_broken) / eta_initial
        } else {
            0.0
        };

        let recovery_percent = if (eta_initial - eta_broken).abs() > 1e-30 {
            (eta_recovered - eta_broken) / (eta_initial - eta_broken) * 100.0
        } else {
            100.0
        };

        // Find time to 90% recovery
        let target = eta_broken + 0.9 * (eta_initial - eta_broken);
        let mut recovery_time = *recovery
            .shear_rate_per_s
            .last()
            .unwrap_or(&0.0);
        // In a recovery test, time is stored in shear_rate field by convention
        // or we estimate from the index
        for i in 0..recovery.len() {
            if recovery.viscosity_pa_s[i] >= target {
                recovery_time = recovery.shear_rate_per_s[i]; // time surrogate
                break;
            }
        }

        ThixotropyResult {
            viscosity_drop,
            recovery_percent,
            recovery_time_s: recovery_time,
        }
    }

    /// Calculate the area of the thixotropic hysteresis loop between
    /// the up-ramp and down-ramp flow curves (using the trapezoidal rule).
    pub fn hysteresis_area(up_curve: &FlowCurve, down_curve: &FlowCurve) -> f64 {
        // Integrate |eta_up - eta_down| over the common shear-rate range
        let r_min = up_curve
            .shear_rate_per_s
            .iter()
            .chain(down_curve.shear_rate_per_s.iter())
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let r_max = up_curve
            .shear_rate_per_s
            .iter()
            .chain(down_curve.shear_rate_per_s.iter())
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);

        let n_points = 100;
        let dr = (r_max - r_min) / n_points as f64;
        let mut area = 0.0;

        for k in 0..n_points {
            let r1 = r_min + k as f64 * dr;
            let r2 = r_min + (k + 1) as f64 * dr;

            let eta_up_1 = interpolate_linear(&up_curve.shear_rate_per_s, &up_curve.viscosity_pa_s, r1);
            let eta_up_2 = interpolate_linear(&up_curve.shear_rate_per_s, &up_curve.viscosity_pa_s, r2);
            let eta_dn_1 = interpolate_linear(&down_curve.shear_rate_per_s, &down_curve.viscosity_pa_s, r1);
            let eta_dn_2 = interpolate_linear(&down_curve.shear_rate_per_s, &down_curve.viscosity_pa_s, r2);

            let diff1 = (eta_up_1 - eta_dn_1).abs();
            let diff2 = (eta_up_2 - eta_dn_2).abs();
            area += 0.5 * (diff1 + diff2) * dr;
        }

        area
    }

    /// Structural recovery rate — initial slope of viscosity recovery curve.
    pub fn structural_recovery_rate(recovery: &FlowCurve) -> f64 {
        if recovery.len() < 2 {
            return 0.0;
        }
        // Use the first few points to estimate the initial recovery rate
        let n = recovery.len().min(5);
        let (slope, _) = linear_regression(
            &recovery.shear_rate_per_s[..n],
            &recovery.viscosity_pa_s[..n],
        );
        slope
    }
}

// ---------------------------------------------------------------------------
// RheologySimulator - Generate synthetic rheological data
// ---------------------------------------------------------------------------

/// Generate synthetic rheological data for testing and validation.
pub struct RheologySimulator;

impl RheologySimulator {
    /// Simulate oscillatory data from a generalized Maxwell model.
    pub fn simulate_maxwell(
        elements: &[(f64, f64)],
        freq_range: (f64, f64),
        num_points: usize,
    ) -> OscillatoryData {
        let log_min = freq_range.0.ln();
        let log_max = freq_range.1.ln();
        let mut freq = Vec::with_capacity(num_points);
        let mut gp = Vec::with_capacity(num_points);
        let mut gpp = Vec::with_capacity(num_points);

        for k in 0..num_points {
            let frac = k as f64 / (num_points as f64 - 1.0).max(1.0);
            let omega = (log_min + frac * (log_max - log_min)).exp();

            let mut g_prime = 0.0;
            let mut g_double_prime = 0.0;
            for &(g_i, tau_i) in elements {
                let wt = omega * tau_i;
                let wt2 = wt * wt;
                let denom = 1.0 + wt2;
                g_prime += g_i * wt2 / denom;
                g_double_prime += g_i * wt / denom;
            }

            freq.push(omega);
            gp.push(g_prime);
            gpp.push(g_double_prime);
        }

        OscillatoryData::new(freq, gp, gpp)
    }

    /// Simulate a power-law flow curve: eta = K * gamma_dot^(n-1).
    pub fn simulate_power_law(
        k: f64,
        n: f64,
        rate_range: (f64, f64),
        num_points: usize,
    ) -> FlowCurve {
        let log_min = rate_range.0.ln();
        let log_max = rate_range.1.ln();
        let mut rates = Vec::with_capacity(num_points);
        let mut visc = Vec::with_capacity(num_points);

        for i in 0..num_points {
            let frac = i as f64 / (num_points as f64 - 1.0).max(1.0);
            let gamma_dot = (log_min + frac * (log_max - log_min)).exp();
            let eta = k * gamma_dot.powf(n - 1.0);
            rates.push(gamma_dot);
            visc.push(eta);
        }

        FlowCurve::new(rates, visc)
    }

    /// Simulate creep compliance:
    /// J(t) = J_0 + sum_i J_i * (1 - exp(-t/tau_i)) + t/eta_0
    pub fn simulate_creep(
        j0: f64,
        elements: &[(f64, f64)],
        eta0: f64,
        duration: f64,
    ) -> CreepRecovery {
        let num_points = 100;
        let dt = duration / (num_points as f64 - 1.0);
        let mut time = Vec::with_capacity(num_points);
        let mut compliance = Vec::with_capacity(num_points);

        for k in 0..num_points {
            let t = k as f64 * dt;
            let mut j = j0;
            for &(tau_i, j_i) in elements {
                j += j_i * (1.0 - (-t / tau_i).exp());
            }
            if eta0.is_finite() && eta0 > 0.0 {
                j += t / eta0;
            }
            time.push(t);
            compliance.push(j);
        }

        CreepRecovery::new(time, compliance)
    }

    /// Add Gaussian noise to oscillatory data.
    pub fn add_noise(data: &OscillatoryData, noise_percent: f64) -> OscillatoryData {
        let mut rng = SimpleRng::new(42);
        let frac = noise_percent / 100.0;

        let gp: Vec<f64> = data
            .g_prime_pa
            .iter()
            .map(|&g| {
                let noise = rng.next_gaussian() * g * frac;
                (g + noise).max(0.0)
            })
            .collect();
        let gpp: Vec<f64> = data
            .g_double_prime_pa
            .iter()
            .map(|&g| {
                let noise = rng.next_gaussian() * g * frac;
                (g + noise).max(0.0)
            })
            .collect();

        OscillatoryData::new(data.frequency_rad_per_s.clone(), gp, gpp)
    }
}

// ---------------------------------------------------------------------------
// CoxMerzRule - Verify Cox-Merz equivalence
// ---------------------------------------------------------------------------

/// Cox-Merz rule verification: |eta*(omega)| should equal eta(gamma_dot)
/// when omega = gamma_dot for well-behaved polymer melts.
pub struct CoxMerzRule;

impl CoxMerzRule {
    /// Check whether the Cox-Merz rule holds between oscillatory and
    /// steady-shear data. Compares |eta*(omega)| with eta(gamma_dot)
    /// at matching omega = gamma_dot values.
    pub fn check(oscillatory: &OscillatoryData, flow: &FlowCurve) -> CoxMerzResult {
        let mut max_deviation = 0.0_f64;
        let mut comparisons = 0;

        for i in 0..oscillatory.len() {
            let omega = oscillatory.frequency_rad_per_s[i];
            let eta_star = oscillatory.complex_viscosity_at(i);

            // Interpolate steady-shear viscosity at gamma_dot = omega
            let eta_ss = interpolate_linear(
                &flow.shear_rate_per_s,
                &flow.viscosity_pa_s,
                omega,
            );

            if eta_ss > 1e-30 && eta_star > 1e-30 {
                let deviation = ((eta_star - eta_ss) / eta_ss).abs() * 100.0;
                if deviation > max_deviation {
                    max_deviation = deviation;
                }
                comparisons += 1;
            }
        }

        CoxMerzResult {
            is_valid: comparisons > 0 && max_deviation < 10.0,
            max_deviation_percent: max_deviation,
        }
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Simple linear regression y = slope * x + intercept.
fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64) {
    let n = x.len() as f64;
    if n < 2.0 {
        return (0.0, y.first().copied().unwrap_or(0.0));
    }
    let sum_x: f64 = x.iter().sum();
    let sum_y: f64 = y.iter().sum();
    let sum_xy: f64 = x.iter().zip(y.iter()).map(|(xi, yi)| xi * yi).sum();
    let sum_x2: f64 = x.iter().map(|xi| xi * xi).sum();

    let denom = n * sum_x2 - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return (0.0, sum_y / n);
    }

    let slope = (n * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / n;
    (slope, intercept)
}

/// Linear interpolation in sorted (x, y) data.
fn interpolate_linear(xs: &[f64], ys: &[f64], x_target: f64) -> f64 {
    if xs.is_empty() {
        return 0.0;
    }
    if x_target <= xs[0] {
        return ys[0];
    }
    if x_target >= *xs.last().unwrap() {
        return *ys.last().unwrap();
    }

    for i in 0..xs.len() - 1 {
        if xs[i] <= x_target && x_target <= xs[i + 1] {
            let t = (x_target - xs[i]) / (xs[i + 1] - xs[i]);
            return ys[i] + t * (ys[i + 1] - ys[i]);
        }
    }
    *ys.last().unwrap()
}

/// Minimal xorshift64 RNG for reproducible noise generation.
struct SimpleRng {
    state: u64,
}

impl SimpleRng {
    fn new(seed: u64) -> Self {
        Self {
            state: seed.max(1),
        }
    }

    fn next_u64(&mut self) -> u64 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 7;
        self.state ^= self.state << 17;
        self.state
    }

    fn next_f64(&mut self) -> f64 {
        (self.next_u64() as f64) / (u64::MAX as f64)
    }

    /// Box-Muller Gaussian
    fn next_gaussian(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-15);
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;
    const LOOSE_TOL: f64 = 0.05; // 5% relative tolerance for fitted parameters

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn relative_err(actual: f64, expected: f64) -> f64 {
        if expected.abs() < 1e-15 {
            actual.abs()
        } else {
            ((actual - expected) / expected).abs()
        }
    }

    // -----------------------------------------------------------------------
    // OscillatoryData tests
    // -----------------------------------------------------------------------

    fn make_oscillatory() -> OscillatoryData {
        // Single Maxwell element: G=1000 Pa, tau=1 s
        RheologySimulator::simulate_maxwell(&[(1000.0, 1.0)], (0.01, 100.0), 50)
    }

    #[test]
    fn test_oscillatory_new() {
        let data = make_oscillatory();
        assert_eq!(data.len(), 50);
        assert!(!data.is_empty());
    }

    #[test]
    #[should_panic]
    fn test_oscillatory_empty() {
        OscillatoryData::new(vec![], vec![], vec![]);
    }

    #[test]
    #[should_panic]
    fn test_oscillatory_mismatched_lengths() {
        OscillatoryData::new(vec![1.0, 2.0], vec![1.0], vec![1.0, 2.0]);
    }

    #[test]
    fn test_complex_modulus() {
        let data = OscillatoryData::new(vec![1.0], vec![3.0], vec![4.0]);
        assert!(approx_eq(data.complex_modulus_at(0), 5.0, TOL));
    }

    #[test]
    fn test_loss_tangent() {
        let data = OscillatoryData::new(vec![1.0], vec![100.0], vec![200.0]);
        assert!(approx_eq(data.loss_tangent_at(0), 2.0, TOL));
    }

    #[test]
    fn test_loss_tangent_less_than_one() {
        let data = OscillatoryData::new(vec![1.0], vec![200.0], vec![100.0]);
        assert!(data.loss_tangent_at(0) < 1.0);
    }

    #[test]
    fn test_complex_viscosity() {
        // |eta*| = |G*| / omega
        let data = OscillatoryData::new(vec![2.0], vec![3.0], vec![4.0]);
        let expected = 5.0 / 2.0;
        assert!(approx_eq(data.complex_viscosity_at(0), expected, TOL));
    }

    #[test]
    fn test_crossover_frequency_exists() {
        // G' starts below G'' and crosses
        let data = OscillatoryData::new(
            vec![0.1, 1.0, 10.0],
            vec![10.0, 100.0, 1000.0],
            vec![100.0, 100.0, 100.0],
        );
        let xo = data.crossover_frequency();
        assert!(xo.is_some());
        let omega_c = xo.unwrap();
        assert!(omega_c > 0.1 && omega_c < 10.0);
    }

    #[test]
    fn test_crossover_frequency_maxwell() {
        // For a single Maxwell element, crossover is at omega = 1/tau
        let data = make_oscillatory();
        let xo = data.crossover_frequency();
        assert!(xo.is_some());
        let omega_c = xo.unwrap();
        // Should be near 1.0 rad/s for tau=1
        assert!(relative_err(omega_c, 1.0) < 0.15);
    }

    #[test]
    fn test_crossover_frequency_none() {
        // G' always > G'' — no crossover
        let data = OscillatoryData::new(
            vec![0.1, 1.0, 10.0],
            vec![1000.0, 1000.0, 1000.0],
            vec![10.0, 10.0, 10.0],
        );
        assert!(data.crossover_frequency().is_none());
    }

    #[test]
    fn test_crossover_modulus() {
        let data = make_oscillatory();
        let gc = data.crossover_modulus();
        assert!(gc.is_some());
        // For a single Maxwell element, Gc = G/2 = 500 Pa
        assert!(relative_err(gc.unwrap(), 500.0) < 0.15);
    }

    // -----------------------------------------------------------------------
    // FlowCurve tests
    // -----------------------------------------------------------------------

    fn make_power_law_flow(k: f64, n: f64) -> FlowCurve {
        RheologySimulator::simulate_power_law(k, n, (0.01, 1000.0), 50)
    }

    #[test]
    fn test_flow_curve_new() {
        let fc = make_power_law_flow(10.0, 0.5);
        assert_eq!(fc.len(), 50);
    }

    #[test]
    #[should_panic]
    fn test_flow_curve_empty() {
        FlowCurve::new(vec![], vec![]);
    }

    #[test]
    fn test_zero_shear_viscosity_newtonian() {
        // Newtonian: n=1, so eta=K everywhere
        let fc = make_power_law_flow(100.0, 1.0);
        let eta0 = fc.zero_shear_viscosity();
        assert!(relative_err(eta0, 100.0) < 0.01);
    }

    #[test]
    fn test_power_law_fit_known() {
        let k = 50.0;
        let n = 0.4;
        let fc = make_power_law_flow(k, n);
        let result = fc.power_law_fit(0.01, 1000.0);
        assert!(relative_err(result.k, k) < LOOSE_TOL);
        assert!(relative_err(result.n, n) < LOOSE_TOL);
    }

    #[test]
    fn test_power_law_fit_newtonian() {
        let fc = make_power_law_flow(100.0, 1.0);
        let result = fc.power_law_fit(0.01, 1000.0);
        assert!(relative_err(result.n, 1.0) < LOOSE_TOL);
    }

    #[test]
    fn test_is_shear_thinning_yes() {
        let fc = make_power_law_flow(10.0, 0.3);
        assert!(fc.is_shear_thinning());
    }

    #[test]
    fn test_is_shear_thinning_no() {
        let fc = make_power_law_flow(10.0, 1.0);
        assert!(!fc.is_shear_thinning());
    }

    #[test]
    fn test_carreau_fit_runs() {
        let fc = make_power_law_flow(100.0, 0.5);
        let result = fc.carreau_fit();
        assert!(result.eta_0 > 0.0);
        assert!(result.n > 0.0 && result.n < 2.0);
    }

    #[test]
    fn test_cross_model_fit_runs() {
        let fc = make_power_law_flow(100.0, 0.5);
        let result = fc.cross_model_fit();
        assert!(result.eta_0 > 0.0);
        assert!(result.m > 0.0);
    }

    // -----------------------------------------------------------------------
    // CreepRecovery tests
    // -----------------------------------------------------------------------

    fn make_creep() -> CreepRecovery {
        RheologySimulator::simulate_creep(
            1e-4,                       // J_0
            &[(1.0, 5e-4), (10.0, 3e-4)], // (tau, J) elements
            1e4,                        // eta_0
            100.0,                      // duration
        )
    }

    #[test]
    fn test_creep_new() {
        let c = make_creep();
        assert_eq!(c.len(), 100);
        assert!(!c.is_empty());
    }

    #[test]
    fn test_instantaneous_compliance() {
        let c = make_creep();
        assert!(approx_eq(c.instantaneous_compliance(), 1e-4, 1e-6));
    }

    #[test]
    fn test_steady_state_viscosity() {
        let c = make_creep();
        let eta0 = c.steady_state_viscosity();
        // Should be close to 1e4
        assert!(relative_err(eta0, 1e4) < 0.2);
    }

    #[test]
    fn test_recoverable_compliance() {
        let c = make_creep();
        let jr = c.recoverable_compliance();
        assert!(jr >= 0.0);
    }

    #[test]
    fn test_retardation_spectrum() {
        let c = make_creep();
        let spectrum = c.retardation_spectrum(5);
        assert_eq!(spectrum.len(), 5);
        for (tau, j) in &spectrum {
            assert!(*tau > 0.0);
            assert!(*j >= 0.0);
        }
    }

    #[test]
    fn test_retardation_spectrum_zero_elements() {
        let c = make_creep();
        let spectrum = c.retardation_spectrum(0);
        assert!(spectrum.is_empty());
    }

    // -----------------------------------------------------------------------
    // MaxwellModel tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_maxwell_from_oscillatory() {
        let data = make_oscillatory();
        let model = MaxwellModel::from_oscillatory(&data, 5);
        assert!(!model.elements().is_empty());
    }

    #[test]
    fn test_maxwell_predict_g_prime() {
        // Known single Maxwell element
        let model = MaxwellModel {
            elements: vec![(1000.0, 1.0)],
        };
        // At omega = 1: G' = 1000 * 1 / (1+1) = 500
        let gp = model.predict_g_prime(1.0);
        assert!(approx_eq(gp, 500.0, TOL));
    }

    #[test]
    fn test_maxwell_predict_g_double_prime() {
        let model = MaxwellModel {
            elements: vec![(1000.0, 1.0)],
        };
        // At omega = 1: G'' = 1000 * 1 / (1+1) = 500
        let gpp = model.predict_g_double_prime(1.0);
        assert!(approx_eq(gpp, 500.0, TOL));
    }

    #[test]
    fn test_maxwell_relaxation_modulus() {
        let model = MaxwellModel {
            elements: vec![(1000.0, 1.0)],
        };
        // G(0) = 1000
        assert!(approx_eq(model.relaxation_modulus(0.0), 1000.0, TOL));
        // G(t) decays exponentially
        let g_1 = model.relaxation_modulus(1.0);
        assert!(approx_eq(g_1, 1000.0 * (-1.0_f64).exp(), TOL));
    }

    #[test]
    fn test_maxwell_two_elements() {
        let model = MaxwellModel {
            elements: vec![(500.0, 0.1), (500.0, 10.0)],
        };
        let gp = model.predict_g_prime(1.0);
        let gpp = model.predict_g_double_prime(1.0);
        assert!(gp > 0.0);
        assert!(gpp > 0.0);
    }

    #[test]
    fn test_maxwell_roundtrip() {
        // Simulate then fit, predictions should be close to original
        let elements = vec![(800.0, 0.5), (200.0, 5.0)];
        let data = RheologySimulator::simulate_maxwell(&elements, (0.01, 100.0), 30);
        let model = MaxwellModel::from_oscillatory(&data, 8);

        // Check at mid-range frequencies where values are significant
        let max_gp = data.g_prime_pa.iter().cloned().fold(0.0_f64, f64::max);
        let max_gpp = data.g_double_prime_pa.iter().cloned().fold(0.0_f64, f64::max);
        let mut gp_errors = Vec::new();
        let mut gpp_errors = Vec::new();
        for i in 0..data.len() {
            let omega = data.frequency_rad_per_s[i];
            let pred_gp = model.predict_g_prime(omega);
            let pred_gpp = model.predict_g_double_prime(omega);
            // Only check where values are significant (> 10% of max)
            if data.g_prime_pa[i] > 0.1 * max_gp {
                gp_errors.push(relative_err(pred_gp, data.g_prime_pa[i]));
            }
            if data.g_double_prime_pa[i] > 0.1 * max_gpp {
                gpp_errors.push(relative_err(pred_gpp, data.g_double_prime_pa[i]));
            }
        }
        // Mean relative error should be under 25%
        let mean_gp_err: f64 = gp_errors.iter().sum::<f64>() / gp_errors.len().max(1) as f64;
        let mean_gpp_err: f64 = gpp_errors.iter().sum::<f64>() / gpp_errors.len().max(1) as f64;
        assert!(
            mean_gp_err < 0.25,
            "Mean G' error too high: {:.1}%",
            mean_gp_err * 100.0
        );
        assert!(
            mean_gpp_err < 0.25,
            "Mean G'' error too high: {:.1}%",
            mean_gpp_err * 100.0
        );
    }

    // -----------------------------------------------------------------------
    // RelaxationSpectrum tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_relaxation_spectrum_from_oscillatory() {
        let data = make_oscillatory();
        let spectrum = RelaxationSpectrum::from_oscillatory(&data, 10);
        assert_eq!(spectrum.len(), 10);
        for (tau, h) in &spectrum {
            assert!(*tau > 0.0);
            assert!(*h >= 0.0);
        }
    }

    #[test]
    fn test_longest_relaxation_time() {
        let spectrum = vec![(0.1, 10.0), (1.0, 50.0), (10.0, 5.0)];
        let tau_max = RelaxationSpectrum::longest_relaxation_time(&spectrum);
        assert!(approx_eq(tau_max, 1.0, TOL));
    }

    #[test]
    fn test_average_relaxation_time() {
        let spectrum = vec![(1.0, 10.0), (2.0, 10.0)];
        let tau_avg = RelaxationSpectrum::average_relaxation_time(&spectrum);
        assert!(approx_eq(tau_avg, 1.5, TOL));
    }

    #[test]
    fn test_relaxation_spectrum_empty() {
        let spectrum = RelaxationSpectrum::from_oscillatory(
            &OscillatoryData::new(vec![1.0], vec![1.0], vec![1.0]),
            0,
        );
        assert!(spectrum.is_empty());
    }

    // -----------------------------------------------------------------------
    // TimeTemperatureSuperposition tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_shift_factors_single_temp() {
        let data = make_oscillatory();
        let datasets = vec![(25.0, data)];
        let shifts = TimeTemperatureSuperposition::shift_factors(&datasets, 25.0);
        assert_eq!(shifts.len(), 1);
    }

    #[test]
    fn test_shift_factors_multiple_temps() {
        let d1 = RheologySimulator::simulate_maxwell(&[(1000.0, 1.0)], (0.01, 100.0), 20);
        let d2 = RheologySimulator::simulate_maxwell(&[(1000.0, 0.1)], (0.01, 100.0), 20);
        let d3 = RheologySimulator::simulate_maxwell(&[(1000.0, 10.0)], (0.01, 100.0), 20);
        let datasets = vec![(25.0, d1), (50.0, d2), (0.0, d3)];
        let shifts = TimeTemperatureSuperposition::shift_factors(&datasets, 25.0);
        assert_eq!(shifts.len(), 3);
    }

    #[test]
    fn test_wlf_fit() {
        // Known WLF-like shift factors
        let c1 = 8.86;
        let c2 = 101.6;
        let t_ref = 25.0;
        let mut sf = Vec::new();
        for t in &[0.0, 10.0, 25.0, 40.0, 60.0, 80.0] {
            let dt = t - t_ref;
            let log_at = -c1 * dt / (c2 + dt);
            let a_t = (10.0_f64).powf(log_at);
            sf.push((*t, a_t));
        }
        let result = TimeTemperatureSuperposition::wlf_fit(&sf, t_ref);
        assert!(relative_err(result.c1, c1) < 0.1);
        assert!(relative_err(result.c2, c2) < 0.1);
    }

    #[test]
    fn test_arrhenius_fit() {
        // Generate Arrhenius-type shift factors
        let ea = 50000.0; // J/mol
        let r = 8.314;
        let t_ref_k = 298.15;
        let mut sf = Vec::new();
        for t_c in &[0.0, 25.0, 50.0, 75.0, 100.0] {
            let t_k = t_c + 273.15;
            let ln_at: f64 = ea / r * (1.0 / t_k - 1.0 / t_ref_k);
            sf.push((*t_c, ln_at.exp()));
        }
        let ea_fit = TimeTemperatureSuperposition::arrhenius_fit(&sf);
        assert!(relative_err(ea_fit, ea) < 0.05);
    }

    #[test]
    fn test_construct_master_curve() {
        let d1 = RheologySimulator::simulate_maxwell(&[(1000.0, 1.0)], (0.1, 10.0), 10);
        let d2 = RheologySimulator::simulate_maxwell(&[(1000.0, 1.0)], (0.1, 10.0), 10);
        let data = vec![(25.0, d1), (50.0, d2)];
        let shifts = vec![(25.0, 1.0), (50.0, 2.0)];
        let master = TimeTemperatureSuperposition::construct_master_curve(&data, &shifts);
        assert_eq!(master.len(), 20); // Combined from both datasets
    }

    // -----------------------------------------------------------------------
    // AmplitudeSweep tests
    // -----------------------------------------------------------------------

    fn make_amplitude_sweep() -> AmplitudeSweep {
        // G' constant at low strain, drops at high strain
        let mut strain = Vec::new();
        let mut gp = Vec::new();
        let mut gpp = Vec::new();
        for i in 0..20 {
            let s = 10.0_f64.powf(-2.0 + i as f64 * 0.2);
            strain.push(s);
            let g_prime = 1000.0 * (-((s / 10.0).powi(2))).exp();
            let g_double_prime = 200.0 * (1.0 + 0.5 * (s / 5.0));
            gp.push(g_prime);
            gpp.push(g_double_prime);
        }
        AmplitudeSweep::new(strain, gp, gpp)
    }

    #[test]
    fn test_amplitude_sweep_new() {
        let as_ = make_amplitude_sweep();
        assert_eq!(as_.len(), 20);
    }

    #[test]
    fn test_critical_strain() {
        let as_ = make_amplitude_sweep();
        let gamma_c = as_.critical_strain();
        assert!(gamma_c > 0.0);
    }

    #[test]
    fn test_lvr_modulus() {
        let as_ = make_amplitude_sweep();
        let g_lvr = as_.lvr_modulus();
        // Should be close to 1000 Pa (plateau G')
        assert!(relative_err(g_lvr, 1000.0) < 0.1);
    }

    #[test]
    fn test_yield_stress() {
        let as_ = make_amplitude_sweep();
        let stress: Vec<f64> = as_
            .strain_percent
            .iter()
            .enumerate()
            .map(|(i, s)| s * as_.g_prime[i] / 100.0)
            .collect();
        let ys = as_.yield_stress(&stress);
        assert!(ys > 0.0);
    }

    // -----------------------------------------------------------------------
    // ThixotropyAnalyzer tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_three_interval_test() {
        let low = FlowCurve::new(vec![0.1, 0.2, 0.3], vec![100.0, 100.0, 100.0]);
        let high = FlowCurve::new(vec![100.0, 200.0, 300.0], vec![10.0, 8.0, 5.0]);
        let recovery = FlowCurve::new(vec![1.0, 2.0, 3.0], vec![50.0, 70.0, 85.0]);
        let result = ThixotropyAnalyzer::three_interval_test(&low, &high, &recovery);
        assert!(result.viscosity_drop > 0.0);
        assert!(result.recovery_percent > 0.0);
    }

    #[test]
    fn test_hysteresis_area() {
        let up = FlowCurve::new(vec![1.0, 10.0, 100.0], vec![100.0, 50.0, 10.0]);
        let down = FlowCurve::new(vec![1.0, 10.0, 100.0], vec![80.0, 40.0, 10.0]);
        let area = ThixotropyAnalyzer::hysteresis_area(&up, &down);
        assert!(area > 0.0);
    }

    #[test]
    fn test_hysteresis_area_identical_curves() {
        let fc = FlowCurve::new(vec![1.0, 10.0, 100.0], vec![100.0, 50.0, 10.0]);
        let area = ThixotropyAnalyzer::hysteresis_area(&fc, &fc);
        assert!(area < 1e-6); // Should be essentially zero
    }

    #[test]
    fn test_structural_recovery_rate() {
        let recovery = FlowCurve::new(
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
            vec![10.0, 30.0, 50.0, 65.0, 75.0],
        );
        let rate = ThixotropyAnalyzer::structural_recovery_rate(&recovery);
        assert!(rate > 0.0);
    }

    // -----------------------------------------------------------------------
    // RheologySimulator tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_simulate_maxwell_single() {
        let data = RheologySimulator::simulate_maxwell(&[(1000.0, 1.0)], (0.01, 100.0), 20);
        assert_eq!(data.len(), 20);
        // G' should be increasing with frequency
        assert!(data.g_prime_pa.last().unwrap() > data.g_prime_pa.first().unwrap());
    }

    #[test]
    fn test_simulate_maxwell_multiple() {
        let data = RheologySimulator::simulate_maxwell(
            &[(500.0, 0.1), (300.0, 1.0), (200.0, 10.0)],
            (0.001, 1000.0),
            100,
        );
        assert_eq!(data.len(), 100);
    }

    #[test]
    fn test_simulate_power_law() {
        let fc = RheologySimulator::simulate_power_law(10.0, 0.5, (0.1, 100.0), 30);
        assert_eq!(fc.len(), 30);
        // Shear-thinning: viscosity should decrease
        assert!(fc.viscosity_pa_s.last().unwrap() < fc.viscosity_pa_s.first().unwrap());
    }

    #[test]
    fn test_simulate_power_law_newtonian() {
        let fc = RheologySimulator::simulate_power_law(100.0, 1.0, (0.1, 100.0), 20);
        // All viscosities should be approximately 100
        for eta in &fc.viscosity_pa_s {
            assert!(relative_err(*eta, 100.0) < 0.01);
        }
    }

    #[test]
    fn test_simulate_creep() {
        let cr = RheologySimulator::simulate_creep(
            1e-3,
            &[(1.0, 2e-3), (10.0, 1e-3)],
            1e5,
            50.0,
        );
        assert_eq!(cr.len(), 100);
        // Compliance should be monotonically increasing
        for i in 1..cr.len() {
            assert!(cr.compliance_per_pa[i] >= cr.compliance_per_pa[i - 1]);
        }
    }

    #[test]
    fn test_add_noise() {
        let data = make_oscillatory();
        let noisy = RheologySimulator::add_noise(&data, 5.0);
        assert_eq!(noisy.len(), data.len());
        // Values should be slightly different but same order of magnitude
        let mut any_different = false;
        for i in 0..data.len() {
            if (noisy.g_prime_pa[i] - data.g_prime_pa[i]).abs() > 1e-15 {
                any_different = true;
                break;
            }
        }
        assert!(any_different);
    }

    #[test]
    fn test_add_noise_zero() {
        let data = OscillatoryData::new(vec![1.0], vec![100.0], vec![50.0]);
        let noisy = RheologySimulator::add_noise(&data, 0.0);
        assert!(approx_eq(noisy.g_prime_pa[0], 100.0, 1.0));
    }

    // -----------------------------------------------------------------------
    // CoxMerzRule tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_cox_merz_valid() {
        // For a Newtonian fluid, Cox-Merz holds exactly
        // eta = eta_0 everywhere, |eta*| = sqrt(G'^2 + G''^2)/omega
        let eta0 = 100.0;
        let freqs = vec![0.1, 1.0, 10.0, 100.0];
        let gp: Vec<f64> = freqs.iter().map(|w| eta0 * w).collect(); // approximate
        let gpp: Vec<f64> = freqs.iter().map(|w| eta0 * w).collect();

        // eta* = sqrt(2) * eta0 for equal G'/G'' -- not exact but tests the mechanism
        let osc = OscillatoryData::new(freqs.clone(), gp, gpp);
        let rates = freqs.clone();
        let visc = vec![eta0 * 2.0_f64.sqrt(); rates.len()]; // match eta*
        let flow = FlowCurve::new(rates, visc);

        let result = CoxMerzRule::check(&osc, &flow);
        // Should be valid since we constructed matching data
        assert!(result.max_deviation_percent < 15.0);
    }

    #[test]
    fn test_cox_merz_invalid() {
        let osc = OscillatoryData::new(
            vec![1.0, 10.0],
            vec![1000.0, 5000.0],
            vec![500.0, 2500.0],
        );
        let flow = FlowCurve::new(vec![1.0, 10.0], vec![1.0, 1.0]); // Very different
        let result = CoxMerzRule::check(&osc, &flow);
        assert!(!result.is_valid);
        assert!(result.max_deviation_percent > 10.0);
    }

    // -----------------------------------------------------------------------
    // Helper function tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_linear_regression() {
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y = vec![2.0, 4.0, 6.0, 8.0, 10.0];
        let (slope, intercept) = linear_regression(&x, &y);
        assert!(approx_eq(slope, 2.0, TOL));
        assert!(approx_eq(intercept, 0.0, TOL));
    }

    #[test]
    fn test_linear_regression_with_offset() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![5.0, 7.0, 9.0, 11.0];
        let (slope, intercept) = linear_regression(&x, &y);
        assert!(approx_eq(slope, 2.0, TOL));
        assert!(approx_eq(intercept, 5.0, TOL));
    }

    #[test]
    fn test_linear_regression_single_point() {
        let x = vec![1.0];
        let y = vec![5.0];
        let (slope, intercept) = linear_regression(&x, &y);
        assert!(approx_eq(slope, 0.0, TOL));
        assert!(approx_eq(intercept, 5.0, TOL));
    }

    #[test]
    fn test_interpolate_linear() {
        let xs = vec![0.0, 1.0, 2.0, 3.0];
        let ys = vec![0.0, 10.0, 20.0, 30.0];
        assert!(approx_eq(interpolate_linear(&xs, &ys, 0.5), 5.0, TOL));
        assert!(approx_eq(interpolate_linear(&xs, &ys, 1.5), 15.0, TOL));
        // Extrapolation clamped
        assert!(approx_eq(interpolate_linear(&xs, &ys, -1.0), 0.0, TOL));
        assert!(approx_eq(interpolate_linear(&xs, &ys, 5.0), 30.0, TOL));
    }

    #[test]
    fn test_interpolate_linear_exact() {
        let xs = vec![1.0, 2.0, 3.0];
        let ys = vec![10.0, 20.0, 30.0];
        assert!(approx_eq(interpolate_linear(&xs, &ys, 2.0), 20.0, TOL));
    }

    #[test]
    fn test_simple_rng_deterministic() {
        let mut rng1 = SimpleRng::new(123);
        let mut rng2 = SimpleRng::new(123);
        for _ in 0..10 {
            assert_eq!(rng1.next_u64(), rng2.next_u64());
        }
    }

    #[test]
    fn test_simple_rng_f64_range() {
        let mut rng = SimpleRng::new(42);
        for _ in 0..100 {
            let v = rng.next_f64();
            assert!(v >= 0.0 && v <= 1.0);
        }
    }

    #[test]
    fn test_simple_rng_gaussian() {
        let mut rng = SimpleRng::new(42);
        let mut sum = 0.0;
        let n = 1000;
        for _ in 0..n {
            sum += rng.next_gaussian();
        }
        let mean = sum / n as f64;
        assert!(mean.abs() < 0.2); // Should be near zero
    }

    // -----------------------------------------------------------------------
    // Integration / edge-case tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_full_pipeline_simulate_fit() {
        // Simulate -> fit -> predict
        let elements = vec![(1000.0, 0.1), (500.0, 1.0), (200.0, 10.0)];
        let data = RheologySimulator::simulate_maxwell(&elements, (0.001, 1000.0), 60);
        let model = MaxwellModel::from_oscillatory(&data, 8);

        // Check that predictions are reasonable at the crossover region
        let omega_test = 1.0;
        let gp = model.predict_g_prime(omega_test);
        let gpp = model.predict_g_double_prime(omega_test);
        assert!(gp > 0.0);
        assert!(gpp > 0.0);
    }

    #[test]
    fn test_g_prime_increases_with_frequency() {
        // For a Maxwell fluid, G' should generally increase with frequency
        let data = make_oscillatory();
        let first = data.g_prime_pa[0];
        let last = *data.g_prime_pa.last().unwrap();
        assert!(last > first);
    }

    #[test]
    fn test_g_double_prime_has_maximum() {
        // For a single Maxwell element, G'' peaks at omega = 1/tau
        let data = RheologySimulator::simulate_maxwell(&[(1000.0, 1.0)], (0.01, 100.0), 100);
        let max_idx = data
            .g_double_prime_pa
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        // Max should be roughly in the middle of the log-frequency range
        assert!(max_idx > 10 && max_idx < 90);
    }

    #[test]
    fn test_complex_modulus_bounds() {
        let data = OscillatoryData::new(vec![1.0], vec![300.0], vec![400.0]);
        let gstar = data.complex_modulus_at(0);
        // |G*| >= max(G', G'')
        assert!(gstar >= 300.0);
        assert!(gstar >= 400.0);
    }

    #[test]
    fn test_loss_tangent_crossover_region() {
        // At crossover, tan(delta) = 1
        let data = make_oscillatory();
        if let Some(omega_c) = data.crossover_frequency() {
            // Find nearest index
            let mut best_i = 0;
            let mut best_dist = f64::INFINITY;
            for i in 0..data.len() {
                let dist = (data.frequency_rad_per_s[i] - omega_c).abs();
                if dist < best_dist {
                    best_dist = dist;
                    best_i = i;
                }
            }
            let tan_d = data.loss_tangent_at(best_i);
            assert!(relative_err(tan_d, 1.0) < 0.3);
        }
    }

    #[test]
    fn test_relaxation_modulus_monotonic_decay() {
        let model = MaxwellModel {
            elements: vec![(500.0, 0.5), (500.0, 5.0)],
        };
        let mut prev = model.relaxation_modulus(0.0);
        for k in 1..20 {
            let t = k as f64 * 0.5;
            let g = model.relaxation_modulus(t);
            assert!(g <= prev + 1e-10);
            prev = g;
        }
    }

    #[test]
    fn test_amplitude_sweep_is_empty() {
        let a = AmplitudeSweep::new(vec![1.0], vec![100.0], vec![50.0]);
        assert!(!a.is_empty());
    }

    #[test]
    fn test_flow_curve_is_empty() {
        let fc = FlowCurve::new(vec![1.0], vec![100.0]);
        assert!(!fc.is_empty());
    }
}
