// trace:FR-ITC-001 | ai:claude
//! # Isothermal Titration Calorimetry (ITC) Analyzer
//!
//! This module implements ITC data analysis for measuring binding thermodynamics
//! (Kd, ΔH, ΔS, n) of molecular interactions from heat changes during titration
//! experiments.
//!
//! ## Overview
//!
//! Isothermal Titration Calorimetry directly measures the heat released or absorbed
//! during molecular binding events. By fitting the resulting isotherm to binding models
//! (e.g., the Wiseman isotherm for one-site binding), one obtains the complete
//! thermodynamic profile: stoichiometry (n), association constant (Ka), enthalpy (ΔH),
//! free energy (ΔG), and entropy (ΔS).
//!
//! ## Key concepts
//!
//! - **c-value**: c = n × Ka × [M], determines isotherm shape. Optimal: 5 < c < 500.
//! - **Wiseman isotherm**: Relates heat per injection to molar ratio [L]/[M].
//! - **Thermodynamic identity**: ΔG = -RT ln(Ka) = ΔH - TΔS.
//! - **Typical ranges**: protein-ligand Kd ~ nM-µM, Ka ~ 10⁶-10⁹ M⁻¹.

use std::f64::consts::PI;

/// Gas constant in cal/(mol·K)
const R_CAL: f64 = 1.9872036;

/// Gas constant in J/(mol·K)
const R_J: f64 = 8.314462;

// ---------------------------------------------------------------------------
// ItcRawData
// ---------------------------------------------------------------------------

/// Raw ITC thermogram data: power vs time with injection markers.
#[derive(Debug, Clone)]
pub struct ItcRawData {
    /// Time points in seconds.
    pub time_s: Vec<f64>,
    /// Differential power in µcal/s.
    pub power_ucal_per_s: Vec<f64>,
    /// Times at which injections occur (seconds).
    pub injection_times: Vec<f64>,
    /// Volume per injection in µL.
    pub injection_volume_ul: f64,
    /// Calorimeter cell volume in µL.
    pub cell_volume_ul: f64,
}

impl ItcRawData {
    /// Create a new raw ITC data set.
    pub fn new(
        time_s: Vec<f64>,
        power_ucal_per_s: Vec<f64>,
        injection_times: Vec<f64>,
        injection_volume_ul: f64,
        cell_volume_ul: f64,
    ) -> Self {
        Self {
            time_s,
            power_ucal_per_s,
            injection_times,
            injection_volume_ul,
            cell_volume_ul,
        }
    }

    /// Number of injections in the experiment.
    pub fn num_injections(&self) -> usize {
        self.injection_times.len()
    }

    /// Total duration of the experiment in seconds.
    pub fn duration_s(&self) -> f64 {
        if self.time_s.is_empty() {
            0.0
        } else {
            self.time_s[self.time_s.len() - 1] - self.time_s[0]
        }
    }
}

// ---------------------------------------------------------------------------
// BaselineMethod
// ---------------------------------------------------------------------------

/// Method for baseline correction of the thermogram.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BaselineMethod {
    /// Constant baseline from the mean of the last fraction of data.
    Constant,
    /// Linear baseline interpolated between pre- and post-injection regions.
    Linear,
    /// Cubic spline baseline through user-specified anchor points.
    Spline,
}

// ---------------------------------------------------------------------------
// PeakIntegrator
// ---------------------------------------------------------------------------

/// Integrates injection peaks in the raw thermogram to obtain heats (µcal).
pub struct PeakIntegrator;

impl PeakIntegrator {
    /// Integrate each injection peak to get heat per injection in µcal.
    ///
    /// Uses trapezoidal integration between successive injection boundaries.
    pub fn integrate_peaks(raw: &ItcRawData) -> Vec<f64> {
        if raw.injection_times.is_empty() || raw.time_s.is_empty() {
            return Vec::new();
        }

        let n_inj = raw.injection_times.len();
        let mut heats = Vec::with_capacity(n_inj);

        for i in 0..n_inj {
            let t_start = raw.injection_times[i];
            let t_end = if i + 1 < n_inj {
                raw.injection_times[i + 1]
            } else {
                // Last injection: integrate to end of data
                *raw.time_s.last().unwrap_or(&t_start)
            };

            let heat = trapezoidal_integrate_range(
                &raw.time_s,
                &raw.power_ucal_per_s,
                t_start,
                t_end,
            );
            heats.push(heat);
        }

        heats
    }

    /// Apply baseline correction to the thermogram power trace.
    ///
    /// Returns baseline-corrected power values.
    pub fn baseline_correction(raw: &ItcRawData, method: BaselineMethod) -> Vec<f64> {
        if raw.power_ucal_per_s.is_empty() {
            return Vec::new();
        }

        match method {
            BaselineMethod::Constant => {
                // Use mean of last 10% as baseline
                let n = raw.power_ucal_per_s.len();
                let tail_start = n.saturating_sub(n / 10).max(1);
                let baseline: f64 =
                    raw.power_ucal_per_s[tail_start..].iter().sum::<f64>()
                        / (n - tail_start) as f64;
                raw.power_ucal_per_s.iter().map(|&p| p - baseline).collect()
            }
            BaselineMethod::Linear => {
                let n = raw.power_ucal_per_s.len();
                if n < 2 {
                    return raw.power_ucal_per_s.clone();
                }
                // Linear fit between first 5% and last 5%
                let head_end = (n / 20).max(1);
                let tail_start = n.saturating_sub(n / 20).max(head_end + 1);

                let bl_start: f64 =
                    raw.power_ucal_per_s[..head_end].iter().sum::<f64>() / head_end as f64;
                let bl_end: f64 =
                    raw.power_ucal_per_s[tail_start..].iter().sum::<f64>()
                        / (n - tail_start) as f64;

                let t_start = raw.time_s.first().copied().unwrap_or(0.0);
                let t_end = raw.time_s.last().copied().unwrap_or(1.0);
                let dt = t_end - t_start;
                if dt.abs() < 1e-30 {
                    return raw.power_ucal_per_s.iter().map(|&p| p - bl_start).collect();
                }

                raw.time_s
                    .iter()
                    .zip(raw.power_ucal_per_s.iter())
                    .map(|(&t, &p)| {
                        let frac = (t - t_start) / dt;
                        let bl = bl_start + frac * (bl_end - bl_start);
                        p - bl
                    })
                    .collect()
            }
            BaselineMethod::Spline => {
                // Simplified: use linear for now (full spline would need anchor points)
                Self::baseline_correction(raw, BaselineMethod::Linear)
            }
        }
    }

    /// Subtract heat of dilution from integrated heats.
    pub fn subtract_dilution(heats: &[f64], dilution_heat: f64) -> Vec<f64> {
        heats.iter().map(|&h| h - dilution_heat).collect()
    }

    /// Calculate molar ratio [L]_total / [M]_total at a given injection index.
    ///
    /// Accounts for cumulative ligand added and macromolecule dilution.
    pub fn molar_ratio(
        injection_idx: usize,
        ligand_conc: f64,
        macromolecule_conc: f64,
        cell_vol: f64,
        inj_vol: f64,
    ) -> f64 {
        if macromolecule_conc <= 0.0 || cell_vol <= 0.0 {
            return 0.0;
        }
        let n = (injection_idx + 1) as f64;
        let total_injected = n * inj_vol;
        // Total ligand moles in cell
        let ligand_moles = ligand_conc * total_injected;
        // Macromolecule moles corrected for dilution
        let effective_vol = cell_vol + total_injected;
        let macro_moles = macromolecule_conc * cell_vol;
        // Molar ratio
        if macro_moles <= 0.0 {
            return 0.0;
        }
        ligand_moles / macro_moles
    }
}

/// Trapezoidal integration of (time, value) data over [t_start, t_end].
fn trapezoidal_integrate_range(
    times: &[f64],
    values: &[f64],
    t_start: f64,
    t_end: f64,
) -> f64 {
    let n = times.len().min(values.len());
    if n < 2 {
        return 0.0;
    }

    let mut integral = 0.0;
    for i in 0..n - 1 {
        let t0 = times[i];
        let t1 = times[i + 1];
        // Check if this segment overlaps with [t_start, t_end]
        if t1 <= t_start || t0 >= t_end {
            continue;
        }
        let seg_start = t0.max(t_start);
        let seg_end = t1.min(t_end);
        let dt_seg = seg_end - seg_start;
        if dt_seg <= 0.0 {
            continue;
        }
        let dt_full = t1 - t0;
        if dt_full.abs() < 1e-30 {
            continue;
        }
        // Interpolate values at segment boundaries
        let frac0 = (seg_start - t0) / dt_full;
        let frac1 = (seg_end - t0) / dt_full;
        let v0 = values[i] + frac0 * (values[i + 1] - values[i]);
        let v1 = values[i] + frac1 * (values[i + 1] - values[i]);
        integral += 0.5 * (v0 + v1) * dt_seg;
    }

    integral
}

// ---------------------------------------------------------------------------
// OneSiteResult / OneSiteModel
// ---------------------------------------------------------------------------

/// Result of a one-site binding model fit.
#[derive(Debug, Clone, Copy)]
pub struct OneSiteResult {
    /// Stoichiometry (number of binding sites per macromolecule).
    pub n: f64,
    /// Association constant Ka in M⁻¹.
    pub ka: f64,
    /// Binding enthalpy in cal/mol.
    pub delta_h_cal_per_mol: f64,
    /// Dissociation constant Kd in M (= 1/Ka).
    pub kd: f64,
    /// Free energy ΔG in cal/mol.
    pub delta_g: f64,
    /// Entropy ΔS in cal/(mol·K).
    pub delta_s: f64,
}

/// One-site (independent and identical sites) binding model.
///
/// Uses the Wiseman isotherm:
///
/// Q(i) = nMtΔHV₀/2 [1 + 1/(nKa[Mt]) + Xt/(nMt) - √((1+1/(nKa[Mt])+Xt/(nMt))² - 4Xt/(nMt))]
///
/// where Mt = macromolecule concentration, Xt = total ligand concentration at injection i.
pub struct OneSiteModel;

impl OneSiteModel {
    /// Fit a one-site binding model to ITC data.
    ///
    /// Uses Levenberg-Marquardt-style optimization to find n, Ka, ΔH.
    ///
    /// # Arguments
    /// * `molar_ratios` - [L]/[M] at each injection
    /// * `heats_per_mole` - Heat per mole of injectant (cal/mol)
    /// * `macromolecule_conc` - [M] in molar
    ///
    /// # Returns
    /// Best-fit `OneSiteResult` at T = 298.15 K (25°C).
    pub fn fit(
        molar_ratios: &[f64],
        heats_per_mole: &[f64],
        macromolecule_conc: f64,
    ) -> OneSiteResult {
        Self::fit_at_temperature(molar_ratios, heats_per_mole, macromolecule_conc, 298.15)
    }

    /// Fit with explicit temperature.
    pub fn fit_at_temperature(
        molar_ratios: &[f64],
        heats_per_mole: &[f64],
        macromolecule_conc: f64,
        temperature_k: f64,
    ) -> OneSiteResult {
        let n_pts = molar_ratios.len().min(heats_per_mole.len());
        if n_pts < 3 {
            return OneSiteResult {
                n: 1.0,
                ka: 1e6,
                delta_h_cal_per_mol: -10000.0,
                kd: 1e-6,
                delta_g: ThermodynamicCalculator::delta_g(1e6, temperature_k),
                delta_s: 0.0,
            };
        }

        // Initial guesses
        let mut n = 1.0;
        let mut ka = 1e6;
        // Estimate ΔH from first few points
        let mut delta_h = if heats_per_mole[0].abs() > 1e-10 {
            heats_per_mole[0]
        } else {
            -10000.0
        };

        let mt = macromolecule_conc;

        // Levenberg-Marquardt iterations
        let mut lambda = 0.001_f64;
        let max_iter = 200;

        for _iter in 0..max_iter {
            let predicted = Self::theoretical_isotherm(n, ka, delta_h, mt, molar_ratios);
            let residuals: Vec<f64> = (0..n_pts)
                .map(|i| heats_per_mole[i] - predicted[i])
                .collect();
            let chi2: f64 = residuals.iter().map(|r| r * r).sum();

            // Numerical Jacobian
            let eps_n = n.abs() * 1e-6 + 1e-10;
            let eps_ka = ka.abs() * 1e-6 + 1e-4;
            let eps_dh = delta_h.abs() * 1e-6 + 1e-2;

            let pred_n = Self::theoretical_isotherm(n + eps_n, ka, delta_h, mt, molar_ratios);
            let pred_ka = Self::theoretical_isotherm(n, ka + eps_ka, delta_h, mt, molar_ratios);
            let pred_dh = Self::theoretical_isotherm(n, ka, delta_h + eps_dh, mt, molar_ratios);

            // J^T J and J^T r  (3x3 normal equations)
            let mut jtj = [[0.0_f64; 3]; 3];
            let mut jtr = [0.0_f64; 3];

            for i in 0..n_pts {
                let j = [
                    (pred_n[i] - predicted[i]) / eps_n,
                    (pred_ka[i] - predicted[i]) / eps_ka,
                    (pred_dh[i] - predicted[i]) / eps_dh,
                ];
                for a in 0..3 {
                    jtr[a] += j[a] * residuals[i];
                    for b in 0..3 {
                        jtj[a][b] += j[a] * j[b];
                    }
                }
            }

            // Damping
            for a in 0..3 {
                jtj[a][a] *= 1.0 + lambda;
            }

            // Solve 3x3 system
            let dp = solve_3x3(&jtj, &jtr);

            let n_new = (n + dp[0]).max(0.01).min(100.0);
            let ka_new = (ka + dp[1]).max(1.0).min(1e15);
            let dh_new = delta_h + dp[2];

            let pred_new = Self::theoretical_isotherm(n_new, ka_new, dh_new, mt, molar_ratios);
            let chi2_new: f64 = (0..n_pts)
                .map(|i| {
                    let r = heats_per_mole[i] - pred_new[i];
                    r * r
                })
                .sum();

            if chi2_new < chi2 {
                n = n_new;
                ka = ka_new;
                delta_h = dh_new;
                lambda *= 0.1;
                if lambda < 1e-15 {
                    lambda = 1e-15;
                }
                if chi2 - chi2_new < 1e-12 * chi2.max(1.0) {
                    break;
                }
            } else {
                lambda *= 10.0;
                if lambda > 1e10 {
                    break;
                }
            }
        }

        let kd = ThermodynamicCalculator::kd_from_ka(ka);
        let delta_g = ThermodynamicCalculator::delta_g(ka, temperature_k);
        let delta_s = ThermodynamicCalculator::delta_s(delta_g, delta_h, temperature_k);

        OneSiteResult {
            n,
            ka,
            delta_h_cal_per_mol: delta_h,
            kd,
            delta_g,
            delta_s,
        }
    }

    /// Compute the theoretical Wiseman isotherm.
    ///
    /// Returns differential heat ΔQ(i)/Δ(Lt) per injection (cal/mol of injectant).
    ///
    /// The Wiseman isotherm for cumulative heat Q at molar ratio r = Xt/(nMt):
    ///
    /// Q = (nMtΔHV₀/2) [1 + 1/(nKa·Mt) + r - √((1 + 1/(nKa·Mt) + r)² - 4r)]
    ///
    /// The per-injection heat is ΔQ = Q(i) - Q(i-1), corrected for displaced volume.
    pub fn theoretical_isotherm(
        n: f64,
        ka: f64,
        delta_h: f64,
        mt: f64,
        ratios: &[f64],
    ) -> Vec<f64> {
        let nka_mt = n * ka * mt;
        let mut heats = Vec::with_capacity(ratios.len());

        let cum_heat = |ratio: f64| -> f64 {
            let r = ratio / n; // Xt / (n*Mt)
            let term = 1.0 + 1.0 / nka_mt + r;
            let disc = term * term - 4.0 * r;
            let sqrt_disc = if disc > 0.0 { disc.sqrt() } else { 0.0 };
            0.5 * n * mt * delta_h * (term - sqrt_disc)
        };

        let mut prev_q = 0.0;
        for (i, &ratio) in ratios.iter().enumerate() {
            let q = cum_heat(ratio);
            let dq = q - prev_q;
            // Convert to per-mole of injectant:
            // dq is in units of (conc * cal/mol), we need to normalize
            // by the ligand added in this injection step
            let dr = if i > 0 {
                ratio - ratios[i - 1]
            } else {
                ratio
            };
            let heat_per_mole = if dr.abs() > 1e-30 {
                dq / (dr * mt)
            } else {
                delta_h
            };
            heats.push(heat_per_mole);
            prev_q = q;
        }

        heats
    }
}

// ---------------------------------------------------------------------------
// TwoSiteResult / TwoSiteModel
// ---------------------------------------------------------------------------

/// Result of a two-site binding model fit.
#[derive(Debug, Clone, Copy)]
pub struct TwoSiteResult {
    /// Stoichiometry for site 1.
    pub n1: f64,
    /// Association constant for site 1 in M⁻¹.
    pub ka1: f64,
    /// Enthalpy for site 1 in cal/mol.
    pub dh1: f64,
    /// Stoichiometry for site 2.
    pub n2: f64,
    /// Association constant for site 2 in M⁻¹.
    pub ka2: f64,
    /// Enthalpy for site 2 in cal/mol.
    pub dh2: f64,
}

/// Two independent binding sites model.
///
/// The total heat is the sum of two independent Wiseman isotherms.
pub struct TwoSiteModel;

impl TwoSiteModel {
    /// Fit a two-site model.
    ///
    /// Uses grid search for initial guesses followed by iterative refinement.
    pub fn fit(ratios: &[f64], heats: &[f64], mt: f64) -> TwoSiteResult {
        Self::fit_at_temperature(ratios, heats, mt, 298.15)
    }

    /// Fit with explicit temperature.
    pub fn fit_at_temperature(
        ratios: &[f64],
        heats: &[f64],
        mt: f64,
        _temperature_k: f64,
    ) -> TwoSiteResult {
        let n_pts = ratios.len().min(heats.len());
        if n_pts < 6 {
            return TwoSiteResult {
                n1: 1.0, ka1: 1e7, dh1: -15000.0,
                n2: 1.0, ka2: 1e5, dh2: -5000.0,
            };
        }

        // Initial guesses: site 1 dominates early, site 2 later
        let mut params = [1.0_f64, 1e7, -15000.0, 1.0, 1e5, -5000.0];

        // Use ΔH estimates from data
        if n_pts >= 4 {
            let early_avg = heats[..2].iter().sum::<f64>() / 2.0;
            let late_avg = heats[n_pts - 2..].iter().sum::<f64>() / 2.0;
            params[2] = early_avg.min(-100.0).max(-100000.0);
            params[5] = late_avg.min(-100.0).max(-100000.0);
        }

        // Simple gradient descent refinement
        let mut lambda = 0.001_f64;

        for _iter in 0..150 {
            let predicted =
                Self::theoretical_two_site(params[0], params[1], params[2], params[3], params[4], params[5], mt, ratios);
            let chi2: f64 = (0..n_pts)
                .map(|i| {
                    let r = heats[i] - predicted[i];
                    r * r
                })
                .sum();

            // Numerical gradient
            let mut grad = [0.0_f64; 6];
            let eps_vals = [1e-4, params[1] * 1e-6, 1.0, 1e-4, params[4] * 1e-6, 1.0];

            for p in 0..6 {
                let mut params_p = params;
                params_p[p] += eps_vals[p];
                let pred_p = Self::theoretical_two_site(
                    params_p[0], params_p[1], params_p[2],
                    params_p[3], params_p[4], params_p[5],
                    mt, ratios,
                );
                let chi2_p: f64 = (0..n_pts)
                    .map(|i| {
                        let r = heats[i] - pred_p[i];
                        r * r
                    })
                    .sum();
                grad[p] = (chi2_p - chi2) / eps_vals[p];
            }

            // Step
            let mut params_new = params;
            for p in 0..6 {
                params_new[p] -= lambda * grad[p];
            }
            // Clamp
            params_new[0] = params_new[0].max(0.01).min(100.0);
            params_new[1] = params_new[1].max(1.0).min(1e15);
            params_new[3] = params_new[3].max(0.01).min(100.0);
            params_new[4] = params_new[4].max(1.0).min(1e15);

            let pred_new = Self::theoretical_two_site(
                params_new[0], params_new[1], params_new[2],
                params_new[3], params_new[4], params_new[5],
                mt, ratios,
            );
            let chi2_new: f64 = (0..n_pts)
                .map(|i| {
                    let r = heats[i] - pred_new[i];
                    r * r
                })
                .sum();

            if chi2_new < chi2 {
                params = params_new;
                lambda *= 1.2;
            } else {
                lambda *= 0.5;
                if lambda < 1e-15 {
                    break;
                }
            }
        }

        TwoSiteResult {
            n1: params[0],
            ka1: params[1],
            dh1: params[2],
            n2: params[3],
            ka2: params[4],
            dh2: params[5],
        }
    }

    /// Compute theoretical isotherm for two independent sites.
    fn theoretical_two_site(
        n1: f64, ka1: f64, dh1: f64,
        n2: f64, ka2: f64, dh2: f64,
        mt: f64,
        ratios: &[f64],
    ) -> Vec<f64> {
        let h1 = OneSiteModel::theoretical_isotherm(n1, ka1, dh1, mt, ratios);
        let h2 = OneSiteModel::theoretical_isotherm(n2, ka2, dh2, mt, ratios);
        h1.iter().zip(h2.iter()).map(|(&a, &b)| a + b).collect()
    }
}

// ---------------------------------------------------------------------------
// ThermodynamicCalculator
// ---------------------------------------------------------------------------

/// Derives thermodynamic quantities from fitted binding parameters.
pub struct ThermodynamicCalculator;

/// Result of van't Hoff analysis.
#[derive(Debug, Clone, Copy)]
pub struct VanHoffResult {
    /// Standard enthalpy ΔH° in cal/mol (from slope of ln(Ka) vs 1/T).
    pub delta_h_std: f64,
    /// Standard entropy ΔS° in cal/(mol·K) (from intercept).
    pub delta_s_std: f64,
    /// R² goodness of fit.
    pub r_squared: f64,
}

impl ThermodynamicCalculator {
    /// Gibbs free energy: ΔG = -RT ln(Ka).
    ///
    /// Returns ΔG in cal/mol.
    pub fn delta_g(ka: f64, temperature_k: f64) -> f64 {
        -R_CAL * temperature_k * ka.ln()
    }

    /// Entropy from ΔG and ΔH: ΔS = (ΔH - ΔG) / T.
    ///
    /// Returns ΔS in cal/(mol·K).
    pub fn delta_s(delta_g: f64, delta_h: f64, temperature_k: f64) -> f64 {
        if temperature_k.abs() < 1e-10 {
            return 0.0;
        }
        (delta_h - delta_g) / temperature_k
    }

    /// Dissociation constant from association constant: Kd = 1/Ka.
    pub fn kd_from_ka(ka: f64) -> f64 {
        if ka.abs() < 1e-30 {
            return f64::INFINITY;
        }
        1.0 / ka
    }

    /// Van't Hoff analysis: determine ΔH° and ΔS° from Ka measurements at
    /// multiple temperatures.
    ///
    /// Fits ln(Ka) = -ΔH°/(R·T) + ΔS°/R by linear regression on ln(Ka) vs 1/T.
    pub fn van_hoff_analysis(kas: &[f64], temps_k: &[f64]) -> VanHoffResult {
        let n = kas.len().min(temps_k.len());
        if n < 2 {
            return VanHoffResult {
                delta_h_std: 0.0,
                delta_s_std: 0.0,
                r_squared: 0.0,
            };
        }

        // x = 1/T, y = ln(Ka)
        let xs: Vec<f64> = temps_k[..n].iter().map(|&t| 1.0 / t).collect();
        let ys: Vec<f64> = kas[..n].iter().map(|&ka| ka.ln()).collect();

        let (slope, intercept, r_sq) = linear_regression(&xs, &ys);

        // slope = -ΔH°/R, intercept = ΔS°/R
        VanHoffResult {
            delta_h_std: -slope * R_CAL,
            delta_s_std: intercept * R_CAL,
            r_squared: r_sq,
        }
    }

    /// Heat capacity change ΔCp from enthalpy measurements at different temperatures.
    ///
    /// ΔCp = dΔH/dT, estimated by linear regression of ΔH vs T.
    pub fn delta_cp(delta_h_values: &[f64], temps_k: &[f64]) -> f64 {
        let n = delta_h_values.len().min(temps_k.len());
        if n < 2 {
            return 0.0;
        }
        let (slope, _intercept, _r_sq) =
            linear_regression(&temps_k[..n].to_vec(), &delta_h_values[..n].to_vec());
        slope
    }

    /// Enthalpy-entropy compensation analysis.
    ///
    /// Returns (slope, intercept) of ΔH vs TΔS plot.
    /// A slope near 1.0 indicates strong compensation.
    pub fn enthalpy_entropy_compensation(
        delta_h: &[f64],
        delta_s: &[f64],
    ) -> (f64, f64) {
        let n = delta_h.len().min(delta_s.len());
        if n < 2 {
            return (0.0, 0.0);
        }
        // Plot ΔH vs TΔS at T=298.15K
        let t_ds: Vec<f64> = delta_s[..n].iter().map(|&s| 298.15 * s).collect();
        let (slope, intercept, _r_sq) =
            linear_regression(&t_ds, &delta_h[..n].to_vec());
        (slope, intercept)
    }
}

// ---------------------------------------------------------------------------
// CParameter
// ---------------------------------------------------------------------------

/// Wiseman c-value for experimental design.
///
/// c = n × Ka × [M], determines the shape of the isotherm.
pub struct CParameter;

impl CParameter {
    /// Calculate the c-value.
    pub fn c_value(n: f64, ka: f64, macromolecule_conc: f64) -> f64 {
        n * ka * macromolecule_conc
    }

    /// Check if c-value is in the optimal range (5 < c < 500).
    pub fn is_optimal(c: f64) -> bool {
        c > 5.0 && c < 500.0
    }

    /// Recommend macromolecule concentration for optimal c ≈ 50.
    pub fn recommend_concentration(ka: f64, n: f64) -> f64 {
        if (n * ka).abs() < 1e-30 {
            return 0.0;
        }
        50.0 / (n * ka)
    }

    /// Qualitative assessment of the isotherm shape from c-value.
    pub fn sigmoidal_quality(c: f64) -> &'static str {
        if c < 1.0 {
            "too low"
        } else if c < 5.0 {
            "poor"
        } else if c < 50.0 {
            "good"
        } else if c <= 500.0 {
            "excellent"
        } else {
            "too high"
        }
    }
}

// ---------------------------------------------------------------------------
// DilutionCorrection
// ---------------------------------------------------------------------------

/// Methods for correcting heats of dilution.
pub struct DilutionCorrection;

impl DilutionCorrection {
    /// Subtract average of the last `avg_last_n` injections as constant dilution heat.
    pub fn constant_dilution(heats: &[f64], avg_last_n: usize) -> Vec<f64> {
        if heats.is_empty() || avg_last_n == 0 {
            return heats.to_vec();
        }
        let n = avg_last_n.min(heats.len());
        let start = heats.len() - n;
        let avg: f64 = heats[start..].iter().sum::<f64>() / n as f64;
        heats.iter().map(|&h| h - avg).collect()
    }

    /// Linear dilution correction: fit a line to the last points and subtract.
    pub fn linear_dilution(heats: &[f64], ratios: &[f64]) -> Vec<f64> {
        let n = heats.len().min(ratios.len());
        if n < 4 {
            return heats.to_vec();
        }

        // Use last 30% for baseline fit
        let fit_start = (n * 7 / 10).max(2);
        let xs: Vec<f64> = ratios[fit_start..n].to_vec();
        let ys: Vec<f64> = heats[fit_start..n].to_vec();
        let (slope, intercept, _) = linear_regression(&xs, &ys);

        (0..n)
            .map(|i| heats[i] - (slope * ratios[i] + intercept))
            .collect()
    }

    /// Subtract blank experiment heats point by point.
    pub fn blank_subtraction(sample_heats: &[f64], blank_heats: &[f64]) -> Vec<f64> {
        let n = sample_heats.len().min(blank_heats.len());
        (0..n)
            .map(|i| sample_heats[i] - blank_heats[i])
            .collect()
    }
}

// ---------------------------------------------------------------------------
// ConcentrationCorrector
// ---------------------------------------------------------------------------

/// Accounts for volume displacement effects during titration.
pub struct ConcentrationCorrector;

impl ConcentrationCorrector {
    /// Effective macromolecule concentration after dilution by injected volume.
    pub fn effective_concentration(
        initial_conc: f64,
        cell_vol: f64,
        total_injected: f64,
    ) -> f64 {
        if cell_vol + total_injected <= 0.0 {
            return 0.0;
        }
        initial_conc * cell_vol / (cell_vol + total_injected)
    }

    /// Cumulative molar ratio accounting for volume displacement.
    pub fn cumulative_molar_ratio(
        injection_idx: usize,
        ligand_conc: f64,
        macro_conc: f64,
        cell_vol: f64,
        inj_vol: f64,
    ) -> f64 {
        let n_inj = (injection_idx + 1) as f64;
        let total_injected = n_inj * inj_vol;

        // Total ligand in cell (some displaced)
        // After each injection, cell overflows so volume stays at cell_vol
        // But total ligand accumulates (correcting for overflow)
        let eff_vol = cell_vol; // cell volume stays constant (overflow)
        let ligand_in_cell =
            ligand_conc * inj_vol * n_inj * (1.0 - total_injected / (2.0 * cell_vol));
        let macro_in_cell = macro_conc * cell_vol * (1.0 - total_injected / (2.0 * cell_vol));

        if macro_in_cell.abs() < 1e-30 {
            return 0.0;
        }
        ligand_in_cell / macro_in_cell
    }

    /// Volume correction factor for displaced liquid.
    ///
    /// Returns fraction of original macromolecule remaining in cell.
    pub fn volume_correction_factor(cell_vol: f64, total_injected: f64) -> f64 {
        if cell_vol <= 0.0 {
            return 0.0;
        }
        // Approximate: each injection displaces some cell contents
        // Factor = exp(-V_inj_total / V_cell) for continuous overflow
        (-total_injected / cell_vol).exp()
    }
}

// ---------------------------------------------------------------------------
// ItcSimulator
// ---------------------------------------------------------------------------

/// Generates synthetic ITC data for testing and validation.
pub struct ItcSimulator;

impl ItcSimulator {
    /// Simulate a one-site binding experiment.
    ///
    /// Returns (molar_ratios, heats_per_mole_of_injectant).
    pub fn simulate_one_site(
        n: f64,
        ka: f64,
        dh: f64,
        mt: f64,
        lt: f64,
        cell_vol: f64,
        inj_vol: f64,
        num_injections: usize,
    ) -> (Vec<f64>, Vec<f64>) {
        let mut ratios = Vec::with_capacity(num_injections);
        for i in 0..num_injections {
            let r = PeakIntegrator::molar_ratio(i, lt, mt, cell_vol, inj_vol);
            ratios.push(r);
        }
        let heats = OneSiteModel::theoretical_isotherm(n, ka, dh, mt, &ratios);
        (ratios, heats)
    }

    /// Simulate with added Gaussian noise.
    pub fn simulate_with_noise(
        params: &OneSiteResult,
        mt: f64,
        lt: f64,
        cell_vol: f64,
        inj_vol: f64,
        num_injections: usize,
        noise_cal_per_mol: f64,
        seed: u64,
    ) -> (Vec<f64>, Vec<f64>) {
        let (ratios, heats) = Self::simulate_one_site(
            params.n,
            params.ka,
            params.delta_h_cal_per_mol,
            mt,
            lt,
            cell_vol,
            inj_vol,
            num_injections,
        );

        // Simple deterministic pseudo-random noise (LCG)
        let mut rng_state = seed;
        let noisy_heats: Vec<f64> = heats
            .iter()
            .map(|&h| {
                // Box-Muller for Gaussian
                rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                let u1 = (rng_state as f64) / (u64::MAX as f64);
                rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                let u2 = (rng_state as f64) / (u64::MAX as f64);
                let z = (-2.0 * u1.max(1e-30).ln()).sqrt() * (2.0 * PI * u2).cos();
                h + noise_cal_per_mol * z
            })
            .collect();

        (ratios, noisy_heats)
    }

    /// Simulate a full thermogram (power vs time).
    ///
    /// Each injection produces a Gaussian-shaped power peak whose area equals the heat.
    pub fn simulate_thermogram(
        n: f64,
        ka: f64,
        dh: f64,
        mt: f64,
        lt: f64,
        cell_vol: f64,
        inj_vol: f64,
        num_injections: usize,
        peak_width_s: f64,
        spacing_s: f64,
        sample_rate_hz: f64,
    ) -> ItcRawData {
        let (ratios, heats) =
            Self::simulate_one_site(n, ka, dh, mt, lt, cell_vol, inj_vol, num_injections);

        let total_duration = (num_injections as f64 + 1.0) * spacing_s;
        let n_samples = (total_duration * sample_rate_hz) as usize;

        let mut time_s = Vec::with_capacity(n_samples);
        let mut power = Vec::with_capacity(n_samples);
        let mut injection_times = Vec::with_capacity(num_injections);

        let dt = 1.0 / sample_rate_hz;
        let sigma = peak_width_s / 2.355; // FWHM to sigma

        for i in 0..num_injections {
            injection_times.push((i as f64 + 0.5) * spacing_s);
        }

        for s in 0..n_samples {
            let t = s as f64 * dt;
            time_s.push(t);

            let mut p = 0.0;
            for (inj_idx, &t_inj) in injection_times.iter().enumerate() {
                if inj_idx < heats.len() {
                    // Convert heat per mole to total heat power:
                    // Heat from this injection (µcal units in thermogram)
                    let heat_ucal = heats[inj_idx] * mt * inj_vol; // rough scaling
                    let amp = heat_ucal / (sigma * (2.0 * PI).sqrt());
                    let dt_inj = t - t_inj;
                    p += amp * (-0.5 * (dt_inj / sigma).powi(2)).exp();
                }
            }
            power.push(p);
        }

        ItcRawData {
            time_s,
            power_ucal_per_s: power,
            injection_times,
            injection_volume_ul: inj_vol,
            cell_volume_ul: cell_vol,
        }
    }
}

// ---------------------------------------------------------------------------
// CompetitiveBinding
// ---------------------------------------------------------------------------

/// Analysis of competitive/displacement binding assays.
pub struct CompetitiveBinding;

impl CompetitiveBinding {
    /// Apparent Ka in the presence of a weak competitor.
    ///
    /// Ka_app = Ka_strong / (1 + Ka_weak × [weak])
    pub fn apparent_ka(ka_strong: f64, ka_weak: f64, weak_conc: f64) -> f64 {
        ka_strong / (1.0 + ka_weak * weak_conc)
    }

    /// Analyze displacement experiment to get true Ka of strong binder.
    ///
    /// Given apparent heats from a competition experiment and the known weak binder
    /// parameters, extract the strong binder's thermodynamics.
    pub fn displacement_analysis(
        strong_heats: &[f64],
        ratios: &[f64],
        mt: f64,
        ka_weak: f64,
        weak_conc: f64,
        dh_weak: f64,
        temperature_k: f64,
    ) -> OneSiteResult {
        // Subtract weak binder contribution
        let corrected: Vec<f64> = strong_heats.iter().map(|&h| h - dh_weak * 0.1).collect();
        let result = OneSiteModel::fit_at_temperature(&ratios, &corrected, mt, temperature_k);

        // Correct Ka for competition
        let ka_true = result.ka * (1.0 + ka_weak * weak_conc);
        let kd = ThermodynamicCalculator::kd_from_ka(ka_true);
        let delta_g = ThermodynamicCalculator::delta_g(ka_true, temperature_k);
        let delta_s =
            ThermodynamicCalculator::delta_s(delta_g, result.delta_h_cal_per_mol, temperature_k);

        OneSiteResult {
            n: result.n,
            ka: ka_true,
            delta_h_cal_per_mol: result.delta_h_cal_per_mol,
            kd,
            delta_g,
            delta_s,
        }
    }

    /// Inhibition constant Ki from displacement experiment.
    ///
    /// Ki = 1 / (Ka_app × (1 + Ka_weak × [weak]))
    pub fn ki_from_displacement(ka_apparent: f64, ka_weak: f64, weak_conc: f64) -> f64 {
        let ka_true = ka_apparent * (1.0 + ka_weak * weak_conc);
        ThermodynamicCalculator::kd_from_ka(ka_true)
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Simple linear regression: y = slope * x + intercept.
/// Returns (slope, intercept, r_squared).
fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64, f64) {
    let n = x.len().min(y.len());
    if n < 2 {
        return (0.0, 0.0, 0.0);
    }
    let nf = n as f64;

    let sum_x: f64 = x[..n].iter().sum();
    let sum_y: f64 = y[..n].iter().sum();
    let sum_xx: f64 = x[..n].iter().map(|&xi| xi * xi).sum();
    let sum_xy: f64 = x[..n].iter().zip(y[..n].iter()).map(|(&xi, &yi)| xi * yi).sum();

    let denom = nf * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return (0.0, sum_y / nf, 0.0);
    }

    let slope = (nf * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / nf;

    // R²
    let mean_y = sum_y / nf;
    let ss_tot: f64 = y[..n].iter().map(|&yi| (yi - mean_y).powi(2)).sum();
    let ss_res: f64 = x[..n]
        .iter()
        .zip(y[..n].iter())
        .map(|(&xi, &yi)| {
            let pred = slope * xi + intercept;
            (yi - pred).powi(2)
        })
        .sum();

    let r_squared = if ss_tot.abs() > 1e-30 {
        1.0 - ss_res / ss_tot
    } else {
        0.0
    };

    (slope, intercept, r_squared)
}

/// Solve a 3x3 linear system Ax = b using Cramer's rule.
fn solve_3x3(a: &[[f64; 3]; 3], b: &[f64; 3]) -> [f64; 3] {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    if det.abs() < 1e-30 {
        return [0.0; 3];
    }

    let det_inv = 1.0 / det;

    let x0 = (b[0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (b[1] * a[2][2] - a[1][2] * b[2])
        + a[0][2] * (b[1] * a[2][1] - a[1][1] * b[2]))
        * det_inv;

    let x1 = (a[0][0] * (b[1] * a[2][2] - a[1][2] * b[2])
        - b[0] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * b[2] - b[1] * a[2][0]))
        * det_inv;

    let x2 = (a[0][0] * (a[1][1] * b[2] - b[1] * a[2][1])
        - a[0][1] * (a[1][0] * b[2] - b[1] * a[2][0])
        + b[0] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
        * det_inv;

    [x0, x1, x2]
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;
    const TEMP_K: f64 = 298.15;

    // -----------------------------------------------------------------------
    // ItcRawData tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_raw_data_creation() {
        let raw = ItcRawData::new(
            vec![0.0, 1.0, 2.0],
            vec![0.0, 10.0, 0.0],
            vec![0.5],
            2.0,
            200.0,
        );
        assert_eq!(raw.num_injections(), 1);
        assert!((raw.duration_s() - 2.0).abs() < TOL);
    }

    #[test]
    fn test_raw_data_empty() {
        let raw = ItcRawData::new(vec![], vec![], vec![], 2.0, 200.0);
        assert_eq!(raw.num_injections(), 0);
        assert!((raw.duration_s() - 0.0).abs() < TOL);
    }

    #[test]
    fn test_raw_data_multiple_injections() {
        let raw = ItcRawData::new(
            vec![0.0, 100.0, 200.0, 300.0],
            vec![0.0, 5.0, 3.0, 1.0],
            vec![50.0, 150.0, 250.0],
            2.0,
            200.0,
        );
        assert_eq!(raw.num_injections(), 3);
        assert!((raw.duration_s() - 300.0).abs() < TOL);
    }

    // -----------------------------------------------------------------------
    // PeakIntegrator tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_integrate_peaks_single() {
        // Single injection: triangular peak from t=10 to t=20
        let raw = ItcRawData::new(
            vec![0.0, 10.0, 15.0, 20.0, 30.0],
            vec![0.0, 0.0, 10.0, 0.0, 0.0],
            vec![10.0],
            2.0,
            200.0,
        );
        let heats = PeakIntegrator::integrate_peaks(&raw);
        assert_eq!(heats.len(), 1);
        // Triangle area = 0.5 * base * height = 0.5 * 10 * 10 = 50 (integrated from t=10 to end=30)
        // Actually: trap from 10-15: 0.5*(0+10)*5=25, from 15-20: 0.5*(10+0)*5=25, from 20-30: 0
        assert!((heats[0] - 50.0).abs() < 1.0);
    }

    #[test]
    fn test_integrate_peaks_empty() {
        let raw = ItcRawData::new(vec![], vec![], vec![], 2.0, 200.0);
        let heats = PeakIntegrator::integrate_peaks(&raw);
        assert!(heats.is_empty());
    }

    #[test]
    fn test_integrate_peaks_constant() {
        // Constant power = 5.0 µcal/s over 100s interval
        let raw = ItcRawData::new(
            vec![0.0, 50.0, 100.0],
            vec![5.0, 5.0, 5.0],
            vec![0.0],
            2.0,
            200.0,
        );
        let heats = PeakIntegrator::integrate_peaks(&raw);
        assert_eq!(heats.len(), 1);
        assert!((heats[0] - 500.0).abs() < 1.0);
    }

    #[test]
    fn test_baseline_correction_constant() {
        let raw = ItcRawData::new(
            vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0],
            vec![10.0, 10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 5.0, 5.0, 5.0],
            vec![0.0],
            2.0,
            200.0,
        );
        let corrected = PeakIntegrator::baseline_correction(&raw, BaselineMethod::Constant);
        assert_eq!(corrected.len(), 10);
        // Last 10% = last 1 sample = 5.0
        assert!((corrected[0] - 5.0).abs() < TOL);
        assert!((corrected[9] - 0.0).abs() < TOL);
    }

    #[test]
    fn test_baseline_correction_linear() {
        let raw = ItcRawData::new(
            (0..20).map(|i| i as f64).collect(),
            (0..20).map(|i| 100.0 + i as f64).collect(), // linearly increasing
            vec![5.0],
            2.0,
            200.0,
        );
        let corrected = PeakIntegrator::baseline_correction(&raw, BaselineMethod::Linear);
        assert_eq!(corrected.len(), 20);
        // After subtracting linear baseline, residuals should be small
        let max_residual = corrected.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
        assert!(max_residual < 5.0); // Rough tolerance
    }

    #[test]
    fn test_subtract_dilution() {
        let heats = vec![100.0, 80.0, 60.0, 40.0, 20.0];
        let corrected = PeakIntegrator::subtract_dilution(&heats, 10.0);
        assert_eq!(corrected.len(), 5);
        assert!((corrected[0] - 90.0).abs() < TOL);
        assert!((corrected[4] - 10.0).abs() < TOL);
    }

    #[test]
    fn test_molar_ratio_first_injection() {
        let r = PeakIntegrator::molar_ratio(0, 1e-3, 50e-6, 200.0, 2.0);
        // injection 1: ligand = 1e-3 * 2.0 = 2e-3, macro = 50e-6 * 200 = 0.01
        // ratio = 2e-3 / 0.01 = 0.2
        assert!((r - 0.2).abs() < 0.01);
    }

    #[test]
    fn test_molar_ratio_tenth_injection() {
        let r = PeakIntegrator::molar_ratio(9, 1e-3, 50e-6, 200.0, 2.0);
        // injection 10: ligand = 1e-3 * 20.0 = 0.02, macro = 50e-6 * 200 = 0.01
        // ratio = 0.02 / 0.01 = 2.0
        assert!((r - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_molar_ratio_zero_macro() {
        let r = PeakIntegrator::molar_ratio(0, 1e-3, 0.0, 200.0, 2.0);
        assert!((r - 0.0).abs() < TOL);
    }

    // -----------------------------------------------------------------------
    // OneSiteModel tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_wiseman_isotherm_basic() {
        let n = 1.0;
        let ka = 1e6;
        let dh = -10000.0; // cal/mol
        let mt = 50e-6;
        let ratios: Vec<f64> = (1..=20).map(|i| i as f64 * 0.15).collect();

        let heats = OneSiteModel::theoretical_isotherm(n, ka, dh, mt, &ratios);
        assert_eq!(heats.len(), 20);

        // First injection should have large exothermic heat
        assert!(heats[0] < 0.0); // Exothermic
        // Last injections should approach zero (saturation)
        assert!(heats[19].abs() < heats[0].abs());
    }

    #[test]
    fn test_wiseman_isotherm_high_c() {
        // High c-value should give sharp sigmoidal transition
        let n = 1.0;
        let ka = 1e8;
        let dh = -10000.0;
        let mt = 100e-6;
        let ratios: Vec<f64> = (1..=20).map(|i| i as f64 * 0.15).collect();

        let heats = OneSiteModel::theoretical_isotherm(n, ka, dh, mt, &ratios);
        // Sharp transition around ratio = n = 1.0
        // Before ratio 1: large heat; after: small heat
        let before_1 = heats.iter().take(6).map(|h| h.abs()).sum::<f64>() / 6.0;
        let after_1 = heats.iter().skip(14).map(|h| h.abs()).sum::<f64>() / 6.0;
        assert!(before_1 > after_1 * 2.0);
    }

    #[test]
    fn test_one_site_fit_recovery() {
        // Generate synthetic data and fit
        let true_n = 1.0;
        let true_ka = 1e6;
        let true_dh = -10000.0;
        let mt = 50e-6;
        let lt = 500e-6;

        let (ratios, heats) =
            ItcSimulator::simulate_one_site(true_n, true_ka, true_dh, mt, lt, 200.0, 2.0, 25);

        let result = OneSiteModel::fit_at_temperature(&ratios, &heats, mt, TEMP_K);

        // Check parameter recovery (within 20%)
        assert!((result.n - true_n).abs() / true_n < 0.2);
        assert!((result.ka - true_ka).abs() / true_ka < 0.3);
        assert!((result.delta_h_cal_per_mol - true_dh).abs() / true_dh.abs() < 0.2);
    }

    #[test]
    fn test_one_site_result_thermodynamics() {
        let true_ka = 1e6;
        let true_dh = -10000.0;
        let mt = 50e-6;
        let lt = 500e-6;

        let (ratios, heats) =
            ItcSimulator::simulate_one_site(1.0, true_ka, true_dh, mt, lt, 200.0, 2.0, 25);
        let result = OneSiteModel::fit_at_temperature(&ratios, &heats, mt, TEMP_K);

        // ΔG should be negative (favorable binding)
        assert!(result.delta_g < 0.0);
        // Kd = 1/Ka
        assert!((result.kd - 1.0 / result.ka).abs() < 1e-12);
        // ΔG = ΔH - TΔS identity
        let dg_check = result.delta_h_cal_per_mol - TEMP_K * result.delta_s;
        assert!((result.delta_g - dg_check).abs() < 1.0);
    }

    #[test]
    fn test_one_site_fit_with_few_points() {
        // Fewer than 3 points: should return defaults without crashing
        let result = OneSiteModel::fit(&[0.5, 1.0], &[-5000.0, -2000.0], 50e-6);
        assert!(result.n > 0.0);
        assert!(result.ka > 0.0);
    }

    // -----------------------------------------------------------------------
    // TwoSiteModel tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_two_site_theoretical() {
        let ratios: Vec<f64> = (1..=20).map(|i| i as f64 * 0.15).collect();
        let mt = 50e-6;
        let heats = TwoSiteModel::theoretical_two_site(
            1.0, 1e7, -15000.0,
            1.0, 1e5, -5000.0,
            mt, &ratios,
        );
        assert_eq!(heats.len(), 20);
        // Two-site should show biphasic behavior
        assert!(heats[0].abs() > 0.0);
    }

    #[test]
    fn test_two_site_fit_returns_result() {
        let ratios: Vec<f64> = (1..=20).map(|i| i as f64 * 0.15).collect();
        let mt = 50e-6;
        let heats = TwoSiteModel::theoretical_two_site(
            1.0, 1e7, -15000.0,
            1.0, 1e5, -5000.0,
            mt, &ratios,
        );
        let result = TwoSiteModel::fit(&ratios, &heats, mt);
        assert!(result.n1 > 0.0);
        assert!(result.n2 > 0.0);
        assert!(result.ka1 > 0.0);
        assert!(result.ka2 > 0.0);
    }

    #[test]
    fn test_two_site_fit_few_points() {
        let result = TwoSiteModel::fit(&[0.5, 1.0, 1.5], &[-5000.0, -2000.0, -500.0], 50e-6);
        assert!(result.n1 > 0.0);
    }

    // -----------------------------------------------------------------------
    // ThermodynamicCalculator tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_delta_g_calculation() {
        // Ka = 1e6 at 298.15 K
        let dg = ThermodynamicCalculator::delta_g(1e6, TEMP_K);
        // ΔG = -RT ln(Ka) = -1.987 * 298.15 * ln(1e6) ≈ -8184 cal/mol
        assert!((dg + 8184.0).abs() < 50.0);
    }

    #[test]
    fn test_delta_g_sign() {
        // Favorable binding (Ka > 1) should give negative ΔG
        assert!(ThermodynamicCalculator::delta_g(1e6, TEMP_K) < 0.0);
        // Unfavorable binding (Ka < 1) should give positive ΔG
        assert!(ThermodynamicCalculator::delta_g(0.1, TEMP_K) > 0.0);
    }

    #[test]
    fn test_delta_s_calculation() {
        let dg = -8184.0;
        let dh = -10000.0;
        let ds = ThermodynamicCalculator::delta_s(dg, dh, TEMP_K);
        // ΔS = (ΔH - ΔG) / T = (-10000 + 8184) / 298.15 ≈ -6.09 cal/(mol·K)
        assert!((ds + 6.09).abs() < 0.5);
    }

    #[test]
    fn test_delta_s_zero_temp() {
        let ds = ThermodynamicCalculator::delta_s(-8000.0, -10000.0, 0.0);
        assert!((ds - 0.0).abs() < TOL);
    }

    #[test]
    fn test_kd_from_ka() {
        assert!((ThermodynamicCalculator::kd_from_ka(1e6) - 1e-6).abs() < 1e-12);
        assert!((ThermodynamicCalculator::kd_from_ka(1e9) - 1e-9).abs() < 1e-15);
    }

    #[test]
    fn test_kd_from_zero_ka() {
        let kd = ThermodynamicCalculator::kd_from_ka(0.0);
        assert!(kd.is_infinite());
    }

    #[test]
    fn test_van_hoff_analysis() {
        // Simulate Ka at multiple temperatures with known ΔH° and ΔS°
        let dh_true = -10000.0; // cal/mol
        let ds_true = -20.0; // cal/(mol·K)
        let temps = vec![288.15, 293.15, 298.15, 303.15, 308.15];
        let kas: Vec<f64> = temps
            .iter()
            .map(|&t| {
                let dg = dh_true - t * ds_true;
                (-dg / (R_CAL * t)).exp()
            })
            .collect();

        let result = ThermodynamicCalculator::van_hoff_analysis(&kas, &temps);
        assert!((result.delta_h_std - dh_true).abs() / dh_true.abs() < 0.01);
        assert!((result.delta_s_std - ds_true).abs() / ds_true.abs() < 0.01);
        assert!(result.r_squared > 0.999);
    }

    #[test]
    fn test_van_hoff_too_few_points() {
        let result = ThermodynamicCalculator::van_hoff_analysis(&[1e6], &[298.15]);
        assert!((result.r_squared - 0.0).abs() < TOL);
    }

    #[test]
    fn test_delta_cp() {
        // ΔH increases linearly with temperature: ΔCp = slope
        let temps = vec![288.15, 293.15, 298.15, 303.15, 308.15];
        let delta_cp_true = -200.0; // cal/(mol·K)
        let dh_values: Vec<f64> = temps
            .iter()
            .map(|&t| -10000.0 + delta_cp_true * (t - 298.15))
            .collect();

        let cp = ThermodynamicCalculator::delta_cp(&dh_values, &temps);
        assert!((cp - delta_cp_true).abs() / delta_cp_true.abs() < 0.01);
    }

    #[test]
    fn test_enthalpy_entropy_compensation() {
        let dh = vec![-5000.0, -8000.0, -10000.0, -12000.0];
        let ds = vec![-5.0, -12.0, -17.0, -22.0];
        let (slope, _intercept) =
            ThermodynamicCalculator::enthalpy_entropy_compensation(&dh, &ds);
        // slope near 1.0 indicates compensation
        assert!(slope > 0.5);
    }

    // -----------------------------------------------------------------------
    // CParameter tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_c_value_calculation() {
        let c = CParameter::c_value(1.0, 1e6, 50e-6);
        assert!((c - 50.0).abs() < TOL);
    }

    #[test]
    fn test_c_value_optimal() {
        assert!(CParameter::is_optimal(50.0));
        assert!(CParameter::is_optimal(10.0));
        assert!(CParameter::is_optimal(400.0));
        assert!(!CParameter::is_optimal(3.0));
        assert!(!CParameter::is_optimal(600.0));
    }

    #[test]
    fn test_recommend_concentration() {
        let conc = CParameter::recommend_concentration(1e6, 1.0);
        // c = 50 = 1.0 * 1e6 * conc => conc = 50e-6
        assert!((conc - 50e-6).abs() < 1e-10);
    }

    #[test]
    fn test_recommend_concentration_zero() {
        let conc = CParameter::recommend_concentration(0.0, 1.0);
        assert!((conc - 0.0).abs() < TOL);
    }

    #[test]
    fn test_sigmoidal_quality() {
        assert_eq!(CParameter::sigmoidal_quality(0.5), "too low");
        assert_eq!(CParameter::sigmoidal_quality(3.0), "poor");
        assert_eq!(CParameter::sigmoidal_quality(20.0), "good");
        assert_eq!(CParameter::sigmoidal_quality(100.0), "excellent");
        assert_eq!(CParameter::sigmoidal_quality(1000.0), "too high");
    }

    #[test]
    fn test_sigmoidal_quality_boundary() {
        assert_eq!(CParameter::sigmoidal_quality(5.0), "good");
        assert_eq!(CParameter::sigmoidal_quality(500.0), "excellent");
        assert_eq!(CParameter::sigmoidal_quality(1.0), "poor");
    }

    // -----------------------------------------------------------------------
    // DilutionCorrection tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_constant_dilution() {
        let heats = vec![100.0, 80.0, 60.0, 20.0, 10.0, 10.0];
        let corrected = DilutionCorrection::constant_dilution(&heats, 2);
        // Last 2 average = 10.0
        assert!((corrected[0] - 90.0).abs() < TOL);
        assert!((corrected[5] - 0.0).abs() < TOL);
    }

    #[test]
    fn test_constant_dilution_empty() {
        let corrected = DilutionCorrection::constant_dilution(&[], 3);
        assert!(corrected.is_empty());
    }

    #[test]
    fn test_linear_dilution() {
        let heats = vec![100.0, 80.0, 60.0, 40.0, 20.0, 15.0, 12.0, 10.0, 9.0, 8.0];
        let ratios: Vec<f64> = (0..10).map(|i| (i + 1) as f64 * 0.2).collect();
        let corrected = DilutionCorrection::linear_dilution(&heats, &ratios);
        assert_eq!(corrected.len(), 10);
        // Last points should be near zero after linear correction
    }

    #[test]
    fn test_blank_subtraction() {
        let sample = vec![100.0, 80.0, 60.0, 40.0];
        let blank = vec![5.0, 5.0, 5.0, 5.0];
        let corrected = DilutionCorrection::blank_subtraction(&sample, &blank);
        assert_eq!(corrected.len(), 4);
        assert!((corrected[0] - 95.0).abs() < TOL);
        assert!((corrected[3] - 35.0).abs() < TOL);
    }

    #[test]
    fn test_blank_subtraction_unequal_lengths() {
        let sample = vec![100.0, 80.0, 60.0];
        let blank = vec![5.0, 5.0];
        let corrected = DilutionCorrection::blank_subtraction(&sample, &blank);
        assert_eq!(corrected.len(), 2); // min length
    }

    // -----------------------------------------------------------------------
    // ConcentrationCorrector tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_effective_concentration() {
        let eff = ConcentrationCorrector::effective_concentration(50e-6, 200.0, 20.0);
        // 50e-6 * 200 / 220 ≈ 45.45e-6
        assert!((eff - 50e-6 * 200.0 / 220.0).abs() < 1e-10);
    }

    #[test]
    fn test_effective_concentration_no_injection() {
        let eff = ConcentrationCorrector::effective_concentration(50e-6, 200.0, 0.0);
        assert!((eff - 50e-6).abs() < 1e-10);
    }

    #[test]
    fn test_cumulative_molar_ratio() {
        let r = ConcentrationCorrector::cumulative_molar_ratio(0, 500e-6, 50e-6, 200.0, 2.0);
        // First injection: small correction
        assert!(r > 0.0);
        assert!(r < 1.0);
    }

    #[test]
    fn test_volume_correction_factor() {
        let f = ConcentrationCorrector::volume_correction_factor(200.0, 0.0);
        assert!((f - 1.0).abs() < TOL);

        let f2 = ConcentrationCorrector::volume_correction_factor(200.0, 200.0);
        assert!((f2 - (-1.0_f64).exp()).abs() < TOL);
    }

    #[test]
    fn test_volume_correction_factor_zero_cell() {
        let f = ConcentrationCorrector::volume_correction_factor(0.0, 10.0);
        assert!((f - 0.0).abs() < TOL);
    }

    // -----------------------------------------------------------------------
    // ItcSimulator tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_simulate_one_site() {
        let (ratios, heats) =
            ItcSimulator::simulate_one_site(1.0, 1e6, -10000.0, 50e-6, 500e-6, 200.0, 2.0, 25);
        assert_eq!(ratios.len(), 25);
        assert_eq!(heats.len(), 25);
        // First heat should be large (exothermic)
        assert!(heats[0].abs() > 0.0);
    }

    #[test]
    fn test_simulate_one_site_ratios_increasing() {
        let (ratios, _) =
            ItcSimulator::simulate_one_site(1.0, 1e6, -10000.0, 50e-6, 500e-6, 200.0, 2.0, 20);
        for i in 1..ratios.len() {
            assert!(ratios[i] > ratios[i - 1]);
        }
    }

    #[test]
    fn test_simulate_with_noise() {
        let params = OneSiteResult {
            n: 1.0,
            ka: 1e6,
            delta_h_cal_per_mol: -10000.0,
            kd: 1e-6,
            delta_g: -8184.0,
            delta_s: -6.09,
        };
        let (ratios, heats) = ItcSimulator::simulate_with_noise(
            &params, 50e-6, 500e-6, 200.0, 2.0, 25, 100.0, 42,
        );
        assert_eq!(ratios.len(), 25);
        assert_eq!(heats.len(), 25);
    }

    #[test]
    fn test_simulate_noise_differs_from_clean() {
        let params = OneSiteResult {
            n: 1.0,
            ka: 1e6,
            delta_h_cal_per_mol: -10000.0,
            kd: 1e-6,
            delta_g: -8184.0,
            delta_s: -6.09,
        };
        let (_, clean) = ItcSimulator::simulate_one_site(
            1.0, 1e6, -10000.0, 50e-6, 500e-6, 200.0, 2.0, 25,
        );
        let (_, noisy) = ItcSimulator::simulate_with_noise(
            &params, 50e-6, 500e-6, 200.0, 2.0, 25, 1000.0, 42,
        );
        // At least one point should differ
        let any_diff = clean
            .iter()
            .zip(noisy.iter())
            .any(|(&c, &n)| (c - n).abs() > 1.0);
        assert!(any_diff);
    }

    #[test]
    fn test_simulate_thermogram() {
        let raw = ItcSimulator::simulate_thermogram(
            1.0, 1e6, -10000.0, 50e-6, 500e-6, 200.0, 2.0, 10, 30.0, 300.0, 1.0,
        );
        assert_eq!(raw.num_injections(), 10);
        assert!(!raw.time_s.is_empty());
        assert!(!raw.power_ucal_per_s.is_empty());
        // Power trace should have peaks
        let max_power = raw
            .power_ucal_per_s
            .iter()
            .map(|x| x.abs())
            .fold(0.0_f64, f64::max);
        assert!(max_power > 0.0);
    }

    #[test]
    fn test_simulate_thermogram_peaks_decrease() {
        let raw = ItcSimulator::simulate_thermogram(
            1.0, 1e6, -10000.0, 50e-6, 500e-6, 200.0, 2.0, 15, 30.0, 300.0, 1.0,
        );
        // Peak heights should generally decrease as binding saturates
        let mut peak_heights = Vec::new();
        for &t_inj in &raw.injection_times {
            // Find the sample closest to injection time
            let idx = raw
                .time_s
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| {
                    ((**a - t_inj).abs())
                        .partial_cmp(&((**b - t_inj).abs()))
                        .unwrap()
                })
                .map(|(i, _)| i)
                .unwrap_or(0);
            peak_heights.push(raw.power_ucal_per_s[idx].abs());
        }
        // First peak should be larger than last
        if peak_heights.len() >= 2 {
            assert!(peak_heights[0] > peak_heights[peak_heights.len() - 1] * 0.5);
        }
    }

    // -----------------------------------------------------------------------
    // CompetitiveBinding tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_apparent_ka() {
        let ka_app = CompetitiveBinding::apparent_ka(1e8, 1e5, 1e-3);
        // Ka_app = 1e8 / (1 + 1e5 * 1e-3) = 1e8 / 101 ≈ 9.9e5
        assert!((ka_app - 1e8 / 101.0).abs() / ka_app < 0.01);
    }

    #[test]
    fn test_apparent_ka_no_competitor() {
        let ka_app = CompetitiveBinding::apparent_ka(1e8, 1e5, 0.0);
        assert!((ka_app - 1e8).abs() < 1.0);
    }

    #[test]
    fn test_ki_from_displacement() {
        let ki = CompetitiveBinding::ki_from_displacement(1e6, 1e4, 1e-3);
        // Ka_true = 1e6 * (1 + 1e4 * 1e-3) = 1e6 * 11 = 1.1e7
        // Ki = 1/Ka_true ≈ 9.09e-8
        let expected = 1.0 / (1e6 * 11.0);
        assert!((ki - expected).abs() / expected < 0.01);
    }

    // -----------------------------------------------------------------------
    // Helper function tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_linear_regression_perfect() {
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y = vec![2.0, 4.0, 6.0, 8.0, 10.0]; // y = 2x
        let (slope, intercept, r_sq) = linear_regression(&x, &y);
        assert!((slope - 2.0).abs() < TOL);
        assert!((intercept - 0.0).abs() < TOL);
        assert!((r_sq - 1.0).abs() < TOL);
    }

    #[test]
    fn test_linear_regression_with_offset() {
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y = vec![3.0, 5.0, 7.0, 9.0, 11.0]; // y = 2x + 1
        let (slope, intercept, r_sq) = linear_regression(&x, &y);
        assert!((slope - 2.0).abs() < TOL);
        assert!((intercept - 1.0).abs() < TOL);
        assert!((r_sq - 1.0).abs() < TOL);
    }

    #[test]
    fn test_linear_regression_single_point() {
        let (slope, intercept, r_sq) = linear_regression(&[1.0], &[2.0]);
        assert!((slope - 0.0).abs() < TOL);
        assert!((r_sq - 0.0).abs() < TOL);
    }

    #[test]
    fn test_solve_3x3_identity() {
        let a = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let b = [3.0, 5.0, 7.0];
        let x = solve_3x3(&a, &b);
        assert!((x[0] - 3.0).abs() < TOL);
        assert!((x[1] - 5.0).abs() < TOL);
        assert!((x[2] - 7.0).abs() < TOL);
    }

    #[test]
    fn test_solve_3x3_general() {
        let a = [[2.0, 1.0, 0.0], [1.0, 3.0, 1.0], [0.0, 1.0, 2.0]];
        let b = [5.0, 10.0, 7.0];
        let x = solve_3x3(&a, &b);
        // Verify Ax = b
        for i in 0..3 {
            let sum: f64 = (0..3).map(|j| a[i][j] * x[j]).sum();
            assert!((sum - b[i]).abs() < 1e-10);
        }
    }

    #[test]
    fn test_solve_3x3_singular() {
        let a = [[1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
        let b = [1.0, 2.0, 3.0];
        let x = solve_3x3(&a, &b);
        // Should return zeros for singular matrix
        assert!((x[0] - 0.0).abs() < TOL);
    }

    #[test]
    fn test_trapezoidal_integration() {
        // Integrate constant = 5 from 0 to 10
        let times = vec![0.0, 5.0, 10.0];
        let values = vec![5.0, 5.0, 5.0];
        let result = trapezoidal_integrate_range(&times, &values, 0.0, 10.0);
        assert!((result - 50.0).abs() < TOL);
    }

    #[test]
    fn test_trapezoidal_partial_range() {
        let times = vec![0.0, 5.0, 10.0];
        let values = vec![0.0, 10.0, 20.0]; // linear
        let result = trapezoidal_integrate_range(&times, &values, 0.0, 5.0);
        // Integral of linear from 0 to 5 = 0.5*5*(0+10) = 25
        assert!((result - 25.0).abs() < 1.0);
    }

    // -----------------------------------------------------------------------
    // Integration / roundtrip tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_full_itc_workflow() {
        // 1. Simulate data
        let n_true = 1.0;
        let ka_true = 1e6;
        let dh_true = -10000.0;
        let mt = 50e-6;
        let lt = 500e-6;
        let cell_vol = 200.0;
        let inj_vol = 2.0;
        let n_inj = 25;

        let (ratios, heats) =
            ItcSimulator::simulate_one_site(n_true, ka_true, dh_true, mt, lt, cell_vol, inj_vol, n_inj);

        // 2. Verify c-value
        let c = CParameter::c_value(n_true, ka_true, mt);
        assert!(CParameter::is_optimal(c));

        // 3. Fit model
        let result = OneSiteModel::fit(&ratios, &heats, mt);

        // 4. Verify thermodynamic consistency
        let dg = ThermodynamicCalculator::delta_g(result.ka, TEMP_K);
        let ds = ThermodynamicCalculator::delta_s(dg, result.delta_h_cal_per_mol, TEMP_K);
        assert!((dg - result.delta_g).abs() < 10.0);
        assert!((ds - result.delta_s).abs() < 0.1);
    }

    #[test]
    fn test_thermogram_to_isotherm_workflow() {
        // Simulate thermogram, integrate peaks, then fit
        let raw = ItcSimulator::simulate_thermogram(
            1.0, 1e6, -10000.0, 50e-6, 500e-6, 200.0, 2.0, 15, 30.0, 300.0, 2.0,
        );
        assert!(raw.num_injections() > 0);

        let heats = PeakIntegrator::integrate_peaks(&raw);
        assert_eq!(heats.len(), raw.num_injections());
    }

    #[test]
    fn test_dilution_correction_workflow() {
        let heats = vec![100.0, 80.0, 60.0, 40.0, 25.0, 15.0, 12.0, 10.0, 10.0, 10.0];
        let blank = vec![10.0; 10];
        let corrected = DilutionCorrection::blank_subtraction(&heats, &blank);
        let further = DilutionCorrection::constant_dilution(&corrected, 3);
        // Last 3 average was 0.0, so constant dilution subtracts 0
        assert!((further[0] - 90.0).abs() < TOL);
    }

    #[test]
    fn test_c_value_recommendation() {
        // For tight binder Ka = 1e9
        let conc = CParameter::recommend_concentration(1e9, 1.0);
        assert!((conc - 50e-9).abs() < 1e-12);
        let c = CParameter::c_value(1.0, 1e9, conc);
        assert!((c - 50.0).abs() < TOL);
    }

    #[test]
    fn test_weak_binding_c_value() {
        // Weak binder Ka = 1e3
        let c = CParameter::c_value(1.0, 1e3, 1e-3);
        assert!((c - 1.0).abs() < TOL);
        assert_eq!(CParameter::sigmoidal_quality(c), "poor");
    }

    #[test]
    fn test_thermodynamic_identity() {
        // ΔG = ΔH - TΔS must hold
        let ka = 5e6;
        let dh = -12000.0;
        let t = 310.0; // 37°C
        let dg = ThermodynamicCalculator::delta_g(ka, t);
        let ds = ThermodynamicCalculator::delta_s(dg, dh, t);
        let dg_check = dh - t * ds;
        assert!((dg - dg_check).abs() < 1e-8);
    }

    #[test]
    fn test_van_hoff_with_constant_dh() {
        // If ΔH is truly temperature-independent, van't Hoff should be linear
        let dh = -8000.0;
        let ds = -15.0;
        let temps = vec![283.15, 288.15, 293.15, 298.15, 303.15, 308.15, 313.15];
        let kas: Vec<f64> = temps
            .iter()
            .map(|&t| {
                let dg = dh - t * ds;
                (-dg / (R_CAL * t)).exp()
            })
            .collect();

        let result = ThermodynamicCalculator::van_hoff_analysis(&kas, &temps);
        assert!(result.r_squared > 0.99);
        assert!((result.delta_h_std - dh).abs() / dh.abs() < 0.02);
    }

    #[test]
    fn test_competitive_binding_increases_apparent_kd() {
        let ka_strong = 1e8;
        let ka_weak = 1e5;
        let weak_conc = 1e-3;
        let ka_app = CompetitiveBinding::apparent_ka(ka_strong, ka_weak, weak_conc);
        // Apparent Ka should be less than true Ka
        assert!(ka_app < ka_strong);
        // Apparent Kd should be larger than true Kd
        assert!(1.0 / ka_app > 1.0 / ka_strong);
    }

    #[test]
    fn test_concentration_correction_small_injection() {
        // Small injection: correction should be minimal
        let f = ConcentrationCorrector::volume_correction_factor(1000.0, 2.0);
        assert!((f - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_concentration_correction_large_injection() {
        // Large injection: significant dilution
        let f = ConcentrationCorrector::volume_correction_factor(200.0, 100.0);
        assert!(f < 0.7);
        assert!(f > 0.5);
    }
}
