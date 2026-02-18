//! # Thermogravimetric Analyzer (TGA)
//!
//! Implements TGA data analysis for studying thermal decomposition, oxidation,
//! and mass change kinetics of materials as a function of temperature or time.
//!
//! ## Key Components
//!
//! - **TgaCurve** - Mass vs temperature/time data container
//! - **DtgCurve** - Derivative thermogravimetry (dm/dT) analysis
//! - **DecompositionStep** - Multi-step mass loss identification
//! - **KineticsAnalyzer** - Kissinger, OFW, Friedman isoconversional methods
//! - **ProximateAnalysis** - Moisture, volatiles, fixed carbon, ash (coal/biomass)
//! - **OxidationAnalysis** - Oxidation onset, parabolic rate, Arrhenius activation energy
//! - **ThermalStability** - T5%, T10%, T50%, IPDT, char yield metrics
//! - **BaselineCorrection** - Buoyancy and drift correction
//! - **TgaSimulator** - Synthetic TGA curve generation
//! - **ReportGenerator** - Summary statistics and curve comparison

use std::f64::consts::PI;

/// Universal gas constant in kJ/(mol·K)
const R_KJ: f64 = 8.314e-3;

/// Universal gas constant in J/(mol·K)
const R_J: f64 = 8.314;

// ─── TgaCurve ────────────────────────────────────────────────────────────────

/// TGA mass vs temperature/time data.
#[derive(Debug, Clone)]
pub struct TgaCurve {
    /// Temperature in degrees Celsius.
    pub temperature_c: Vec<f64>,
    /// Mass in milligrams.
    pub mass_mg: Vec<f64>,
    /// Time in minutes.
    pub time_min: Vec<f64>,
}

impl TgaCurve {
    /// Create a new TGA curve from temperature, mass, and time vectors.
    ///
    /// All vectors must have the same length and contain at least 2 points.
    pub fn new(temperature_c: Vec<f64>, mass_mg: Vec<f64>, time_min: Vec<f64>) -> Self {
        assert!(temperature_c.len() >= 2, "Need at least 2 data points");
        assert_eq!(temperature_c.len(), mass_mg.len(), "temperature and mass length mismatch");
        assert_eq!(temperature_c.len(), time_min.len(), "temperature and time length mismatch");
        Self { temperature_c, mass_mg, time_min }
    }

    /// Return mass as percentage of initial mass.
    pub fn mass_percent(&self) -> Vec<f64> {
        let m0 = self.mass_mg[0];
        if m0 == 0.0 {
            return vec![0.0; self.mass_mg.len()];
        }
        self.mass_mg.iter().map(|m| 100.0 * m / m0).collect()
    }

    /// Total mass loss as a percentage of initial mass.
    pub fn mass_loss_total(&self) -> f64 {
        let m0 = self.mass_mg[0];
        let mf = *self.mass_mg.last().unwrap();
        if m0 == 0.0 { return 0.0; }
        100.0 * (m0 - mf) / m0
    }

    /// Residue percentage (final mass / initial mass * 100).
    pub fn residue_percent(&self) -> f64 {
        100.0 - self.mass_loss_total()
    }

    /// Initial mass in mg.
    pub fn initial_mass(&self) -> f64 {
        self.mass_mg[0]
    }

    /// Final mass in mg.
    pub fn final_mass(&self) -> f64 {
        *self.mass_mg.last().unwrap()
    }

    /// Number of data points.
    pub fn len(&self) -> usize {
        self.mass_mg.len()
    }

    /// Whether the curve is empty.
    pub fn is_empty(&self) -> bool {
        self.mass_mg.is_empty()
    }

    /// Interpolate mass at a given temperature using linear interpolation.
    pub fn mass_at_temperature(&self, temp_c: f64) -> f64 {
        linear_interp(&self.temperature_c, &self.mass_mg, temp_c)
    }

    /// Interpolate mass percent at a given temperature.
    pub fn mass_percent_at_temperature(&self, temp_c: f64) -> f64 {
        let m0 = self.mass_mg[0];
        if m0 == 0.0 { return 0.0; }
        100.0 * self.mass_at_temperature(temp_c) / m0
    }
}

// ─── DtgCurve ────────────────────────────────────────────────────────────────

/// Derivative Thermogravimetry (DTG): dm/dT analysis.
pub struct DtgCurve;

impl DtgCurve {
    /// Compute the DTG curve (dm/dT) from a TGA curve using central differences.
    /// Returns (temperature, dm_dT) pairs in %/°C (mass-percent basis).
    pub fn compute(tga: &TgaCurve) -> Vec<(f64, f64)> {
        let pct = tga.mass_percent();
        let n = pct.len();
        if n < 2 {
            return vec![];
        }
        let mut result = Vec::with_capacity(n);

        // Forward difference for first point
        let dt0 = tga.temperature_c[1] - tga.temperature_c[0];
        if dt0.abs() > 1e-15 {
            result.push((tga.temperature_c[0], (pct[1] - pct[0]) / dt0));
        } else {
            result.push((tga.temperature_c[0], 0.0));
        }

        // Central differences for interior points
        for i in 1..n - 1 {
            let dt = tga.temperature_c[i + 1] - tga.temperature_c[i - 1];
            if dt.abs() > 1e-15 {
                result.push((tga.temperature_c[i], (pct[i + 1] - pct[i - 1]) / dt));
            } else {
                result.push((tga.temperature_c[i], 0.0));
            }
        }

        // Backward difference for last point
        let dtl = tga.temperature_c[n - 1] - tga.temperature_c[n - 2];
        if dtl.abs() > 1e-15 {
            result.push((tga.temperature_c[n - 1], (pct[n - 1] - pct[n - 2]) / dtl));
        } else {
            result.push((tga.temperature_c[n - 1], 0.0));
        }

        result
    }

    /// Find peak temperatures in the DTG (most negative = fastest mass loss).
    /// Returns temperatures where dm/dT has local minima (most negative derivative).
    pub fn peak_temperatures(dtg: &[(f64, f64)]) -> Vec<f64> {
        if dtg.len() < 3 {
            return vec![];
        }
        let mut peaks = Vec::new();
        for i in 1..dtg.len() - 1 {
            // Local minimum in dm/dT (most negative = fastest decomposition)
            if dtg[i].1 < dtg[i - 1].1 && dtg[i].1 < dtg[i + 1].1 {
                peaks.push(dtg[i].0);
            }
        }
        peaks
    }

    /// Extrapolated onset temperature: intersection of baseline tangent and
    /// steepest-descent tangent before the first DTG peak.
    pub fn onset_temperature(tga: &TgaCurve) -> f64 {
        let dtg = Self::compute(tga);
        let peaks = Self::peak_temperatures(&dtg);
        if peaks.is_empty() {
            return tga.temperature_c[0];
        }
        let peak_t = peaks[0];
        let pct = tga.mass_percent();

        // Baseline: average slope in first 10% of data
        let baseline_end = tga.temperature_c.len() / 10;
        let baseline_end = if baseline_end < 2 { 2 } else { baseline_end };
        let (bl_slope, bl_intercept) = linear_regression(
            &tga.temperature_c[..baseline_end],
            &pct[..baseline_end],
        );

        // Steepest descent: find index closest to peak temperature
        let peak_idx = nearest_index(&tga.temperature_c, peak_t);
        let start = if peak_idx > 3 { peak_idx - 3 } else { 0 };
        let end = if peak_idx + 3 < tga.temperature_c.len() {
            peak_idx + 3
        } else {
            tga.temperature_c.len()
        };
        let (sd_slope, sd_intercept) = linear_regression(
            &tga.temperature_c[start..end],
            &pct[start..end],
        );

        // Intersection of the two lines
        let denom = bl_slope - sd_slope;
        if denom.abs() < 1e-15 {
            return peak_t;
        }
        (sd_intercept - bl_intercept) / denom
    }

    /// Extrapolated endset temperature: intersection of post-step baseline
    /// tangent and steepest-descent tangent after the last DTG peak.
    pub fn endset_temperature(tga: &TgaCurve) -> f64 {
        let dtg = Self::compute(tga);
        let peaks = Self::peak_temperatures(&dtg);
        if peaks.is_empty() {
            return *tga.temperature_c.last().unwrap();
        }
        let peak_t = *peaks.last().unwrap();
        let pct = tga.mass_percent();
        let n = tga.temperature_c.len();

        // Post-step baseline: last 10% of data
        let baseline_start = n - n / 10;
        let baseline_start = if n - baseline_start < 2 { n - 2 } else { baseline_start };
        let (bl_slope, bl_intercept) = linear_regression(
            &tga.temperature_c[baseline_start..],
            &pct[baseline_start..],
        );

        // Steepest descent near last peak
        let peak_idx = nearest_index(&tga.temperature_c, peak_t);
        let start = if peak_idx > 3 { peak_idx - 3 } else { 0 };
        let end = if peak_idx + 3 < n { peak_idx + 3 } else { n };
        let (sd_slope, sd_intercept) = linear_regression(
            &tga.temperature_c[start..end],
            &pct[start..end],
        );

        let denom = bl_slope - sd_slope;
        if denom.abs() < 1e-15 {
            return peak_t;
        }
        (sd_intercept - bl_intercept) / denom
    }

    /// Savitzky-Golay-like moving average smoothing on (x, y) data.
    pub fn smooth(data: &[(f64, f64)], window: usize) -> Vec<(f64, f64)> {
        if data.is_empty() || window < 2 {
            return data.to_vec();
        }
        let half = window / 2;
        let n = data.len();
        let mut result = Vec::with_capacity(n);
        for i in 0..n {
            let lo = if i >= half { i - half } else { 0 };
            let hi = if i + half < n { i + half + 1 } else { n };
            let count = (hi - lo) as f64;
            let sum_y: f64 = data[lo..hi].iter().map(|(_, y)| y).sum();
            result.push((data[i].0, sum_y / count));
        }
        result
    }
}

// ─── TgaStep / DecompositionStep ─────────────────────────────────────────────

/// A single identified mass loss step.
#[derive(Debug, Clone)]
pub struct TgaStep {
    /// Onset temperature in °C.
    pub onset_c: f64,
    /// Endset temperature in °C.
    pub endset_c: f64,
    /// DTG peak temperature in °C.
    pub peak_c: f64,
    /// Mass loss for this step in percent.
    pub mass_loss_pct: f64,
    /// Maximum mass loss rate (%/°C) at peak.
    pub rate_max: f64,
}

/// Effect of atmosphere on a decomposition step.
#[derive(Debug, Clone, PartialEq)]
pub enum AtmosphereEffect {
    /// Step present in both atmospheres - purely thermal decomposition.
    ThermalDecomposition,
    /// Step shifts significantly in air - oxidative decomposition.
    OxidativeDecomposition,
    /// Step only present in air - combustion.
    Combustion,
    /// Step only present in N2 - pyrolysis.
    Pyrolysis,
}

/// Identify and characterize mass-loss steps in a TGA curve.
pub struct DecompositionStep;

impl DecompositionStep {
    /// Find mass loss steps where each step has at least `min_loss_percent` loss.
    pub fn find_steps(tga: &TgaCurve, min_loss_percent: f64) -> Vec<TgaStep> {
        let dtg = DtgCurve::compute(tga);
        let smoothed = DtgCurve::smooth(&dtg, 5);
        let peaks = DtgCurve::peak_temperatures(&smoothed);
        let pct = tga.mass_percent();

        if peaks.is_empty() {
            return vec![];
        }

        let mut steps = Vec::new();

        for &peak_t in &peaks {
            let peak_idx = nearest_index(&tga.temperature_c, peak_t);

            // Find onset: scan backward from peak to find where DTG magnitude drops
            let mut onset_idx = peak_idx;
            let threshold = smoothed[peak_idx].1 * 0.1; // 10% of peak rate
            for i in (0..peak_idx).rev() {
                if smoothed[i].1 > threshold || smoothed[i].1.abs() < threshold.abs() * 0.1 {
                    onset_idx = i;
                    break;
                }
            }
            if onset_idx == peak_idx && peak_idx > 0 {
                onset_idx = 0;
            }

            // Find endset: scan forward
            let mut endset_idx = peak_idx;
            for i in peak_idx + 1..smoothed.len() {
                if smoothed[i].1 > threshold || smoothed[i].1.abs() < threshold.abs() * 0.1 {
                    endset_idx = i;
                    break;
                }
            }
            if endset_idx == peak_idx && peak_idx < smoothed.len() - 1 {
                endset_idx = smoothed.len() - 1;
            }

            let mass_loss = pct[onset_idx] - pct[endset_idx];

            if mass_loss >= min_loss_percent {
                steps.push(TgaStep {
                    onset_c: tga.temperature_c[onset_idx],
                    endset_c: tga.temperature_c[endset_idx],
                    peak_c: peak_t,
                    mass_loss_pct: mass_loss,
                    rate_max: smoothed[peak_idx].1.abs(),
                });
            }
        }

        steps
    }

    /// Temperature at 50% of a step's mass loss.
    pub fn step_midpoint(step: &TgaStep) -> f64 {
        (step.onset_c + step.endset_c) / 2.0
    }

    /// Compare steps in air vs N2 to classify atmospheric effects.
    pub fn assign_atmosphere_effect(
        steps_air: &[TgaStep],
        steps_n2: &[TgaStep],
    ) -> Vec<AtmosphereEffect> {
        let mut effects = Vec::new();

        for air_step in steps_air {
            let matching_n2 = steps_n2.iter().find(|n2| {
                (n2.peak_c - air_step.peak_c).abs() < 50.0
            });

            match matching_n2 {
                Some(n2_step) => {
                    let temp_shift = (air_step.peak_c - n2_step.peak_c).abs();
                    if temp_shift > 20.0 {
                        effects.push(AtmosphereEffect::OxidativeDecomposition);
                    } else {
                        effects.push(AtmosphereEffect::ThermalDecomposition);
                    }
                }
                None => {
                    effects.push(AtmosphereEffect::Combustion);
                }
            }
        }

        // Check for N2-only steps (pyrolysis)
        for n2_step in steps_n2 {
            let has_air_match = steps_air.iter().any(|air| {
                (air.peak_c - n2_step.peak_c).abs() < 50.0
            });
            if !has_air_match {
                effects.push(AtmosphereEffect::Pyrolysis);
            }
        }

        effects
    }
}

// ─── KineticsAnalyzer ────────────────────────────────────────────────────────

/// Result from Kissinger analysis.
#[derive(Debug, Clone)]
pub struct KissingerResult {
    /// Activation energy in kJ/mol.
    pub activation_energy_kj_mol: f64,
    /// Pre-exponential factor (1/s).
    pub pre_exponential: f64,
    /// Coefficient of determination.
    pub r_squared: f64,
}

/// Thermal decomposition kinetics analysis.
pub struct KineticsAnalyzer;

impl KineticsAnalyzer {
    /// Kissinger method: plot ln(β/Tp²) vs 1/Tp.
    ///
    /// - `heating_rates`: β in K/min (or °C/min)
    /// - `peak_temps_k`: peak temperatures in Kelvin
    ///
    /// Returns activation energy, pre-exponential factor, and R².
    pub fn kissinger(heating_rates: &[f64], peak_temps_k: &[f64]) -> KissingerResult {
        assert_eq!(heating_rates.len(), peak_temps_k.len());
        assert!(heating_rates.len() >= 2, "Need at least 2 heating rates");

        let x: Vec<f64> = peak_temps_k.iter().map(|tp| 1.0 / tp).collect();
        let y: Vec<f64> = heating_rates
            .iter()
            .zip(peak_temps_k.iter())
            .map(|(beta, tp)| (beta / (tp * tp)).ln())
            .collect();

        let (slope, intercept) = linear_regression(&x, &y);
        let ea = -slope * R_KJ; // kJ/mol
        let r2 = r_squared(&x, &y, slope, intercept);

        // Pre-exponential: intercept = ln(A*R/Ea) → A = exp(intercept) * Ea / R
        let a = (intercept).exp() * ea * 1000.0 / R_J;

        KissingerResult {
            activation_energy_kj_mol: ea,
            pre_exponential: a,
            r_squared: r2,
        }
    }

    /// Ozawa-Flynn-Wall isoconversional method.
    ///
    /// - `heating_rates`: β values in K/min
    /// - `temps_at_conversion`: for each conversion level α, the temperatures (K)
    ///   at that α for each heating rate
    ///
    /// Returns activation energy (kJ/mol) at each conversion level.
    pub fn ozawa_flynn_wall(
        heating_rates: &[f64],
        temps_at_conversion: &[Vec<f64>],
    ) -> Vec<f64> {
        let log_betas: Vec<f64> = heating_rates.iter().map(|b| b.ln()).collect();

        temps_at_conversion
            .iter()
            .map(|temps_k| {
                assert_eq!(temps_k.len(), heating_rates.len());
                let inv_t: Vec<f64> = temps_k.iter().map(|t| 1.0 / t).collect();
                let (slope, _) = linear_regression(&inv_t, &log_betas);
                // ln(β) = const - 1.052 * Ea/(R*T)
                // slope of ln(β) vs 1/T = -1.052 * Ea / R
                -slope * R_KJ / 1.052
            })
            .collect()
    }

    /// Friedman differential isoconversional method.
    ///
    /// - `tga_curves`: TGA curves at different heating rates
    /// - `conversions`: α values at which to evaluate Ea
    ///
    /// Returns activation energy (kJ/mol) at each conversion level.
    pub fn friedman(tga_curves: &[TgaCurve], conversions: &[f64]) -> Vec<f64> {
        conversions
            .iter()
            .map(|&alpha| {
                let mut x_vals = Vec::new(); // 1/T
                let mut y_vals = Vec::new(); // ln(dα/dt)

                for curve in tga_curves {
                    let pct = curve.mass_percent();
                    let m0 = pct[0];
                    let mf = *pct.last().unwrap();

                    // Find index where conversion = alpha
                    for i in 1..pct.len() {
                        let a_i = Self::conversion(pct[i], m0, mf);
                        let a_prev = Self::conversion(pct[i - 1], m0, mf);

                        if (a_prev <= alpha && a_i >= alpha) || (a_prev >= alpha && a_i <= alpha) {
                            // Approximate dα/dt at this point
                            let dt = curve.time_min[i] - curve.time_min[i - 1];
                            if dt.abs() > 1e-15 {
                                let da_dt = (a_i - a_prev) / dt;
                                if da_dt.abs() > 1e-15 {
                                    // Linear interpolation for temperature
                                    let frac = if (a_i - a_prev).abs() > 1e-15 {
                                        (alpha - a_prev) / (a_i - a_prev)
                                    } else {
                                        0.5
                                    };
                                    let temp_c = curve.temperature_c[i - 1]
                                        + frac * (curve.temperature_c[i] - curve.temperature_c[i - 1]);
                                    let temp_k = temp_c + 273.15;

                                    x_vals.push(1.0 / temp_k);
                                    y_vals.push(da_dt.abs().ln());
                                }
                            }
                            break;
                        }
                    }
                }

                if x_vals.len() >= 2 {
                    let (slope, _) = linear_regression(&x_vals, &y_vals);
                    -slope * R_KJ
                } else {
                    0.0
                }
            })
            .collect()
    }

    /// Conversion degree α = (m₀ - m) / (m₀ - m_f).
    ///
    /// Where m₀ is initial mass %, m is current mass %, m_f is final mass %.
    pub fn conversion(mass_pct: f64, initial_pct: f64, final_pct: f64) -> f64 {
        let denom = initial_pct - final_pct;
        if denom.abs() < 1e-15 {
            return 0.0;
        }
        let alpha = (initial_pct - mass_pct) / denom;
        alpha.max(0.0).min(1.0)
    }
}

// ─── ProximateAnalysis ───────────────────────────────────────────────────────

/// Proximate analysis of coal, biomass, or similar materials.
pub struct ProximateAnalysis;

impl ProximateAnalysis {
    /// Moisture content: mass loss up to `t_end_c` (typically 105°C).
    pub fn moisture_content(tga: &TgaCurve, t_end_c: f64) -> f64 {
        let m0 = tga.mass_mg[0];
        if m0 == 0.0 { return 0.0; }
        let m_at_t = tga.mass_at_temperature(t_end_c);
        100.0 * (m0 - m_at_t) / m0
    }

    /// Volatile matter: mass loss between `t_start` and `t_end` in inert atmosphere.
    /// Typically 105°C to 900°C under N₂.
    pub fn volatile_matter(tga: &TgaCurve, t_start: f64, t_end: f64) -> f64 {
        let m0 = tga.mass_mg[0];
        if m0 == 0.0 { return 0.0; }
        let m_start = tga.mass_at_temperature(t_start);
        let m_end = tga.mass_at_temperature(t_end);
        100.0 * (m_start - m_end) / m0
    }

    /// Fixed carbon by difference: 100 - moisture - volatile - ash.
    pub fn fixed_carbon(volatile: f64, ash: f64, moisture: f64) -> f64 {
        let fc = 100.0 - moisture - volatile - ash;
        fc.max(0.0)
    }

    /// Ash content: residue remaining after complete combustion in air.
    /// Uses the final mass value of the TGA curve.
    pub fn ash_content(tga: &TgaCurve) -> f64 {
        tga.residue_percent()
    }
}

// ─── OxidationAnalysis ───────────────────────────────────────────────────────

/// Oxidation behavior analysis.
pub struct OxidationAnalysis;

impl OxidationAnalysis {
    /// Oxidation onset temperature: where mass change rate exceeds a threshold.
    /// Detects either mass gain (oxidation) or mass loss (combustion).
    pub fn oxidation_onset(tga: &TgaCurve) -> f64 {
        let dtg = DtgCurve::compute(tga);
        if dtg.len() < 3 {
            return tga.temperature_c[0];
        }

        // Baseline noise: standard deviation of first 10% of DTG
        let baseline_end = dtg.len() / 10;
        let baseline_end = if baseline_end < 3 { 3.min(dtg.len()) } else { baseline_end };
        let bl_mean: f64 = dtg[..baseline_end].iter().map(|(_, y)| y).sum::<f64>()
            / baseline_end as f64;
        let bl_var: f64 = dtg[..baseline_end]
            .iter()
            .map(|(_, y)| (y - bl_mean).powi(2))
            .sum::<f64>()
            / baseline_end as f64;
        let threshold = 3.0 * bl_var.sqrt(); // 3-sigma

        for &(t, rate) in &dtg[baseline_end..] {
            if (rate - bl_mean).abs() > threshold.max(0.01) {
                return t;
            }
        }

        *tga.temperature_c.last().unwrap()
    }

    /// Mass change rate (dm/dt in mg/min) at a given temperature.
    pub fn oxidation_rate(tga: &TgaCurve, temp_c: f64) -> f64 {
        let idx = nearest_index(&tga.temperature_c, temp_c);
        let n = tga.mass_mg.len();
        if n < 2 {
            return 0.0;
        }
        if idx == 0 {
            let dt = tga.time_min[1] - tga.time_min[0];
            if dt.abs() < 1e-15 { return 0.0; }
            return (tga.mass_mg[1] - tga.mass_mg[0]) / dt;
        }
        if idx >= n - 1 {
            let dt = tga.time_min[n - 1] - tga.time_min[n - 2];
            if dt.abs() < 1e-15 { return 0.0; }
            return (tga.mass_mg[n - 1] - tga.mass_mg[n - 2]) / dt;
        }
        let dt = tga.time_min[idx + 1] - tga.time_min[idx - 1];
        if dt.abs() < 1e-15 { return 0.0; }
        (tga.mass_mg[idx + 1] - tga.mass_mg[idx - 1]) / dt
    }

    /// Parabolic rate constant k from Δm² = k·t (Wagner's law).
    ///
    /// - `times_min`: time values in minutes
    /// - `mass_gains`: mass gains (Δm) in mg
    ///
    /// Returns k in mg²/min.
    pub fn parabolic_rate_constant(times_min: &[f64], mass_gains: &[f64]) -> f64 {
        assert_eq!(times_min.len(), mass_gains.len());
        let dm_sq: Vec<f64> = mass_gains.iter().map(|dm| dm * dm).collect();
        let (slope, _) = linear_regression(times_min, &dm_sq);
        slope
    }

    /// Activation energy from Arrhenius fit of rate vs temperature.
    ///
    /// - `temps_k`: temperatures in Kelvin
    /// - `rates`: reaction rates (same units)
    ///
    /// Returns Ea in kJ/mol from ln(rate) = ln(A) - Ea/(R·T).
    pub fn activation_energy_from_rates(temps_k: &[f64], rates: &[f64]) -> f64 {
        assert_eq!(temps_k.len(), rates.len());
        assert!(temps_k.len() >= 2);
        let x: Vec<f64> = temps_k.iter().map(|t| 1.0 / t).collect();
        let y: Vec<f64> = rates.iter().map(|r| r.abs().max(1e-30).ln()).collect();
        let (slope, _) = linear_regression(&x, &y);
        -slope * R_KJ
    }
}

// ─── ThermalStability ────────────────────────────────────────────────────────

/// Thermal stability metrics.
pub struct ThermalStability;

impl ThermalStability {
    /// Temperature at which X% mass loss occurs (from initial).
    fn t_x_percent_loss(tga: &TgaCurve, loss_pct: f64) -> f64 {
        let pct = tga.mass_percent();
        let target = 100.0 - loss_pct;

        for i in 1..pct.len() {
            if pct[i] <= target {
                // Linear interpolation
                if (pct[i - 1] - pct[i]).abs() < 1e-15 {
                    return tga.temperature_c[i];
                }
                let frac = (pct[i - 1] - target) / (pct[i - 1] - pct[i]);
                return tga.temperature_c[i - 1]
                    + frac * (tga.temperature_c[i] - tga.temperature_c[i - 1]);
            }
        }

        // If never reached, return last temperature
        *tga.temperature_c.last().unwrap()
    }

    /// Temperature at 5% mass loss (T5%).
    pub fn t5_percent_loss(tga: &TgaCurve) -> f64 {
        Self::t_x_percent_loss(tga, 5.0)
    }

    /// Temperature at 10% mass loss (T10%).
    pub fn t10_percent_loss(tga: &TgaCurve) -> f64 {
        Self::t_x_percent_loss(tga, 10.0)
    }

    /// Temperature at 50% mass loss (T50%).
    pub fn t50_percent_loss(tga: &TgaCurve) -> f64 {
        Self::t_x_percent_loss(tga, 50.0)
    }

    /// Integral Procedural Decomposition Temperature (IPDT).
    ///
    /// IPDT = A* × K* × (Tf - Ti) + Ti
    /// where A* = area under normalized TGA / total area
    ///       K* = (A* + 1) / 2
    ///       Ti = initial temperature, Tf = final temperature
    pub fn integral_procedural_decomposition_temp(tga: &TgaCurve) -> f64 {
        let pct = tga.mass_percent();
        let n = pct.len();
        if n < 2 {
            return tga.temperature_c[0];
        }

        let ti = tga.temperature_c[0];
        let tf = *tga.temperature_c.last().unwrap();
        let temp_range = tf - ti;
        if temp_range.abs() < 1e-15 {
            return ti;
        }

        // Normalize mass to 0-1 range (fraction retained)
        let norm: Vec<f64> = pct.iter().map(|p| p / 100.0).collect();

        // Trapezoidal integration of normalized mass vs temperature
        let mut area = 0.0;
        for i in 1..n {
            let dt = tga.temperature_c[i] - tga.temperature_c[i - 1];
            area += 0.5 * (norm[i - 1] + norm[i]) * dt;
        }

        let a_star = area / temp_range;
        let k_star = (a_star + 1.0) / 2.0;

        a_star * k_star * temp_range + ti
    }

    /// Char yield: residue percentage at a given temperature.
    pub fn char_yield(tga: &TgaCurve, temp_c: f64) -> f64 {
        tga.mass_percent_at_temperature(temp_c)
    }
}

// ─── BaselineCorrection ──────────────────────────────────────────────────────

/// Buoyancy and drift correction for TGA data.
pub struct BaselineCorrection;

impl BaselineCorrection {
    /// Subtract a blank run from the sample run (buoyancy correction).
    pub fn buoyancy_correction(tga: &TgaCurve, blank: &TgaCurve) -> TgaCurve {
        let corrected_mass: Vec<f64> = tga
            .temperature_c
            .iter()
            .zip(tga.mass_mg.iter())
            .map(|(&t, &m)| {
                let blank_m = blank.mass_at_temperature(t);
                let blank_m0 = blank.mass_mg[0];
                // Subtract the buoyancy drift (blank mass change)
                m - (blank_m - blank_m0)
            })
            .collect();

        TgaCurve::new(
            tga.temperature_c.clone(),
            corrected_mass,
            tga.time_min.clone(),
        )
    }

    /// Remove linear drift from mass data.
    ///
    /// `drift_rate` in mg/min.
    pub fn drift_correction(tga: &TgaCurve, drift_rate: f64) -> TgaCurve {
        let corrected: Vec<f64> = tga
            .time_min
            .iter()
            .zip(tga.mass_mg.iter())
            .map(|(&t, &m)| m - drift_rate * t)
            .collect();

        TgaCurve::new(
            tga.temperature_c.clone(),
            corrected,
            tga.time_min.clone(),
        )
    }

    /// Moving average smoothing of the mass data.
    pub fn smooth_curve(tga: &TgaCurve, window: usize) -> TgaCurve {
        let smoothed = moving_average(&tga.mass_mg, window);
        TgaCurve::new(
            tga.temperature_c.clone(),
            smoothed,
            tga.time_min.clone(),
        )
    }
}

// ─── TgaSimulator ────────────────────────────────────────────────────────────

/// Generate synthetic TGA curves for testing and validation.
pub struct TgaSimulator;

impl TgaSimulator {
    /// Simulate a single-step decomposition.
    ///
    /// - `onset_c`: decomposition onset temperature in °C
    /// - `endset_c`: decomposition end temperature in °C
    /// - `loss_pct`: mass loss as percentage
    /// - `heating_rate`: °C/min
    ///
    /// Generates 1000 data points from 25°C to `endset_c + 100°C`.
    pub fn simulate_single_step(
        onset_c: f64,
        endset_c: f64,
        loss_pct: f64,
        heating_rate: f64,
    ) -> TgaCurve {
        let t_start = 25.0;
        let t_end = endset_c + 100.0;
        let n = 1000;
        let dt = (t_end - t_start) / (n - 1) as f64;

        let mut temps = Vec::with_capacity(n);
        let mut masses = Vec::with_capacity(n);
        let mut times = Vec::with_capacity(n);

        let initial_mass = 10.0; // 10 mg
        let midpoint = (onset_c + endset_c) / 2.0;
        let width = (endset_c - onset_c) / 6.0; // ~6 sigma width

        for i in 0..n {
            let t = t_start + i as f64 * dt;
            temps.push(t);
            times.push(t / heating_rate);

            // Sigmoid mass loss
            let x = (t - midpoint) / width.max(1.0);
            let sigmoid = 1.0 / (1.0 + (-x).exp());
            let mass = initial_mass * (1.0 - loss_pct / 100.0 * sigmoid);
            masses.push(mass);
        }

        TgaCurve::new(temps, masses, times)
    }

    /// Simulate multi-step decomposition.
    ///
    /// `steps`: vec of (onset_c, endset_c, loss_pct) tuples.
    pub fn simulate_multi_step(
        steps: &[(f64, f64, f64)],
        heating_rate: f64,
    ) -> TgaCurve {
        let t_start = 25.0;
        let t_end = steps
            .iter()
            .map(|(_, e, _)| *e)
            .fold(f64::NEG_INFINITY, f64::max)
            + 100.0;
        let n = 1000;
        let dt = (t_end - t_start) / (n - 1) as f64;
        let initial_mass = 10.0;

        let mut temps = Vec::with_capacity(n);
        let mut masses = Vec::with_capacity(n);
        let mut times = Vec::with_capacity(n);

        for i in 0..n {
            let t = t_start + i as f64 * dt;
            temps.push(t);
            times.push(t / heating_rate);

            let mut total_loss_fraction = 0.0;
            for &(onset, endset, loss) in steps {
                let mid = (onset + endset) / 2.0;
                let w = (endset - onset) / 6.0;
                let x = (t - mid) / w.max(1.0);
                let sigmoid = 1.0 / (1.0 + (-x).exp());
                total_loss_fraction += loss / 100.0 * sigmoid;
            }

            let mass = initial_mass * (1.0 - total_loss_fraction.min(1.0));
            masses.push(mass);
        }

        TgaCurve::new(temps, masses, times)
    }

    /// Add Gaussian-like noise to a TGA curve.
    pub fn add_noise(tga: &TgaCurve, noise_mg: f64) -> TgaCurve {
        let mut masses = Vec::with_capacity(tga.mass_mg.len());
        let mut seed: u64 = 42;
        for &m in &tga.mass_mg {
            // Simple LCG pseudo-random noise (Box-Muller approximation)
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u1 = (seed >> 33) as f64 / (1u64 << 31) as f64;
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u2 = (seed >> 33) as f64 / (1u64 << 31) as f64;

            let u1 = u1.max(1e-10);
            let gauss = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos();
            masses.push(m + noise_mg * gauss);
        }

        TgaCurve::new(tga.temperature_c.clone(), masses, tga.time_min.clone())
    }

    /// Simulate isothermal decomposition: f(α) = (1-α)^n.
    ///
    /// - `temperature_c`: constant temperature in °C
    /// - `time_min`: total time in minutes
    /// - `k`: rate constant (1/min)
    /// - `n`: reaction order
    pub fn simulate_isothermal(
        temperature_c: f64,
        time_min: f64,
        k: f64,
        n: f64,
    ) -> TgaCurve {
        let num_points = 500;
        let dt = time_min / (num_points - 1) as f64;
        let initial_mass = 10.0;

        let mut temps = Vec::with_capacity(num_points);
        let mut masses = Vec::with_capacity(num_points);
        let mut times = Vec::with_capacity(num_points);

        let mut alpha = 0.0_f64;

        for i in 0..num_points {
            let t = i as f64 * dt;
            temps.push(temperature_c);
            times.push(t);

            let mass = initial_mass * (1.0 - alpha);
            masses.push(mass);

            // Euler integration of dα/dt = k * (1-α)^n
            let da_dt = k * (1.0 - alpha).max(0.0).powf(n);
            alpha += da_dt * dt;
            alpha = alpha.min(1.0);
        }

        TgaCurve::new(temps, masses, times)
    }
}

// ─── ReportGenerator ─────────────────────────────────────────────────────────

/// TGA analysis summary.
#[derive(Debug, Clone)]
pub struct TgaSummary {
    /// Initial mass in mg.
    pub initial_mass: f64,
    /// Final mass in mg.
    pub final_mass: f64,
    /// Total mass loss in percent.
    pub total_loss: f64,
    /// Number of decomposition steps.
    pub num_steps: usize,
    /// Individual step details.
    pub steps: Vec<TgaStep>,
}

/// Curve comparison result.
#[derive(Debug, Clone)]
pub struct ComparisonResult {
    /// Number of curves compared.
    pub num_curves: usize,
    /// Mean total mass loss.
    pub mean_total_loss: f64,
    /// Std deviation of total mass loss.
    pub std_total_loss: f64,
    /// Mean residue percent.
    pub mean_residue: f64,
    /// Maximum difference in mass percent at any temperature.
    pub max_divergence: f64,
}

/// Summary reporting for TGA analysis.
pub struct ReportGenerator;

impl ReportGenerator {
    /// Generate a summary of TGA data and identified steps.
    pub fn summary(tga: &TgaCurve, steps: &[TgaStep]) -> TgaSummary {
        TgaSummary {
            initial_mass: tga.initial_mass(),
            final_mass: tga.final_mass(),
            total_loss: tga.mass_loss_total(),
            num_steps: steps.len(),
            steps: steps.to_vec(),
        }
    }

    /// Compare multiple TGA curves (e.g., repeatability study).
    pub fn compare_curves(curves: &[TgaCurve]) -> ComparisonResult {
        let n = curves.len();
        if n == 0 {
            return ComparisonResult {
                num_curves: 0,
                mean_total_loss: 0.0,
                std_total_loss: 0.0,
                mean_residue: 0.0,
                max_divergence: 0.0,
            };
        }

        let losses: Vec<f64> = curves.iter().map(|c| c.mass_loss_total()).collect();
        let residues: Vec<f64> = curves.iter().map(|c| c.residue_percent()).collect();

        let mean_loss = losses.iter().sum::<f64>() / n as f64;
        let mean_residue = residues.iter().sum::<f64>() / n as f64;
        let var_loss = losses.iter().map(|l| (l - mean_loss).powi(2)).sum::<f64>() / n as f64;

        // Find max divergence: sample each curve at common temperature points
        let mut max_div = 0.0_f64;
        if n >= 2 {
            // Use 100 evenly-spaced temperature points
            let t_min = curves
                .iter()
                .map(|c| c.temperature_c[0])
                .fold(f64::NEG_INFINITY, f64::max);
            let t_max = curves
                .iter()
                .map(|c| *c.temperature_c.last().unwrap())
                .fold(f64::INFINITY, f64::min);

            if t_max > t_min {
                for k in 0..100 {
                    let t = t_min + (t_max - t_min) * k as f64 / 99.0;
                    let pcts: Vec<f64> = curves.iter().map(|c| c.mass_percent_at_temperature(t)).collect();
                    let pmax = pcts.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                    let pmin = pcts.iter().cloned().fold(f64::INFINITY, f64::min);
                    max_div = max_div.max(pmax - pmin);
                }
            }
        }

        ComparisonResult {
            num_curves: n,
            mean_total_loss: mean_loss,
            std_total_loss: var_loss.sqrt(),
            mean_residue,
            max_divergence: max_div,
        }
    }
}

// ─── Helper functions ────────────────────────────────────────────────────────

/// Linear interpolation in a (x, y) dataset.
fn linear_interp(x: &[f64], y: &[f64], x_val: f64) -> f64 {
    let n = x.len();
    if n == 0 {
        return 0.0;
    }
    if n == 1 || x_val <= x[0] {
        return y[0];
    }
    if x_val >= x[n - 1] {
        return y[n - 1];
    }

    // Binary search
    let mut lo = 0;
    let mut hi = n - 1;
    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        if x[mid] <= x_val {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    let dx = x[hi] - x[lo];
    if dx.abs() < 1e-15 {
        return y[lo];
    }
    let frac = (x_val - x[lo]) / dx;
    y[lo] + frac * (y[hi] - y[lo])
}

/// Simple linear regression: y = slope * x + intercept.
fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64) {
    let n = x.len() as f64;
    if n < 2.0 {
        return (0.0, if y.is_empty() { 0.0 } else { y[0] });
    }

    let sum_x: f64 = x.iter().sum();
    let sum_y: f64 = y.iter().sum();
    let sum_xy: f64 = x.iter().zip(y.iter()).map(|(xi, yi)| xi * yi).sum();
    let sum_xx: f64 = x.iter().map(|xi| xi * xi).sum();

    let denom = n * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return (0.0, sum_y / n);
    }

    let slope = (n * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / n;
    (slope, intercept)
}

/// R² coefficient of determination for a linear fit.
fn r_squared(x: &[f64], y: &[f64], slope: f64, intercept: f64) -> f64 {
    let n = y.len() as f64;
    let mean_y = y.iter().sum::<f64>() / n;

    let ss_tot: f64 = y.iter().map(|yi| (yi - mean_y).powi(2)).sum();
    let ss_res: f64 = x
        .iter()
        .zip(y.iter())
        .map(|(xi, yi)| {
            let pred = slope * xi + intercept;
            (yi - pred).powi(2)
        })
        .sum();

    if ss_tot < 1e-30 {
        return 1.0;
    }
    1.0 - ss_res / ss_tot
}

/// Find nearest index in a sorted array.
fn nearest_index(arr: &[f64], val: f64) -> usize {
    let mut best_idx = 0;
    let mut best_dist = f64::INFINITY;
    for (i, &v) in arr.iter().enumerate() {
        let d = (v - val).abs();
        if d < best_dist {
            best_dist = d;
            best_idx = i;
        }
    }
    best_idx
}

/// Moving average filter.
fn moving_average(data: &[f64], window: usize) -> Vec<f64> {
    if data.is_empty() || window < 2 {
        return data.to_vec();
    }
    let half = window / 2;
    let n = data.len();
    let mut result = Vec::with_capacity(n);
    for i in 0..n {
        let lo = if i >= half { i - half } else { 0 };
        let hi = if i + half < n { i + half + 1 } else { n };
        let count = (hi - lo) as f64;
        let sum: f64 = data[lo..hi].iter().sum();
        result.push(sum / count);
    }
    result
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // Helper to create a simple decomposition curve
    fn make_simple_tga() -> TgaCurve {
        TgaSimulator::simulate_single_step(200.0, 400.0, 30.0, 10.0)
    }

    fn make_two_step_tga() -> TgaCurve {
        TgaSimulator::simulate_multi_step(
            &[(200.0, 350.0, 20.0), (450.0, 600.0, 25.0)],
            10.0,
        )
    }

    // ─── TgaCurve tests ──────────────────────────────────────────────

    #[test]
    fn test_tga_curve_new() {
        let tga = TgaCurve::new(vec![25.0, 100.0], vec![10.0, 9.0], vec![0.0, 7.5]);
        assert_eq!(tga.len(), 2);
        assert!(!tga.is_empty());
    }

    #[test]
    #[should_panic]
    fn test_tga_curve_empty() {
        TgaCurve::new(vec![25.0], vec![10.0, 9.0], vec![0.0]);
    }

    #[test]
    fn test_mass_percent() {
        let tga = TgaCurve::new(vec![25.0, 100.0, 200.0], vec![10.0, 9.0, 7.0], vec![0.0, 7.5, 17.5]);
        let pct = tga.mass_percent();
        assert!(approx_eq(pct[0], 100.0, 1e-10));
        assert!(approx_eq(pct[1], 90.0, 1e-10));
        assert!(approx_eq(pct[2], 70.0, 1e-10));
    }

    #[test]
    fn test_mass_loss_total() {
        let tga = TgaCurve::new(vec![25.0, 200.0], vec![10.0, 7.0], vec![0.0, 17.5]);
        assert!(approx_eq(tga.mass_loss_total(), 30.0, 1e-10));
    }

    #[test]
    fn test_residue_percent() {
        let tga = TgaCurve::new(vec![25.0, 200.0], vec![10.0, 7.0], vec![0.0, 17.5]);
        assert!(approx_eq(tga.residue_percent(), 70.0, 1e-10));
    }

    #[test]
    fn test_initial_final_mass() {
        let tga = TgaCurve::new(vec![25.0, 200.0], vec![10.0, 7.0], vec![0.0, 17.5]);
        assert!(approx_eq(tga.initial_mass(), 10.0, 1e-10));
        assert!(approx_eq(tga.final_mass(), 7.0, 1e-10));
    }

    #[test]
    fn test_mass_at_temperature() {
        let tga = TgaCurve::new(
            vec![25.0, 100.0, 200.0],
            vec![10.0, 9.0, 7.0],
            vec![0.0, 7.5, 17.5],
        );
        // Midpoint interpolation
        let m = tga.mass_at_temperature(62.5);
        assert!(approx_eq(m, 9.5, 1e-10));
    }

    #[test]
    fn test_mass_at_temperature_boundary() {
        let tga = TgaCurve::new(vec![25.0, 100.0], vec![10.0, 9.0], vec![0.0, 7.5]);
        assert!(approx_eq(tga.mass_at_temperature(0.0), 10.0, 1e-10));
        assert!(approx_eq(tga.mass_at_temperature(200.0), 9.0, 1e-10));
    }

    #[test]
    fn test_mass_percent_at_temperature() {
        let tga = TgaCurve::new(vec![25.0, 100.0], vec![10.0, 8.0], vec![0.0, 7.5]);
        let pct = tga.mass_percent_at_temperature(62.5);
        assert!(approx_eq(pct, 90.0, 1e-10));
    }

    // ─── DtgCurve tests ─────────────────────────────────────────────

    #[test]
    fn test_dtg_compute_length() {
        let tga = make_simple_tga();
        let dtg = DtgCurve::compute(&tga);
        assert_eq!(dtg.len(), tga.len());
    }

    #[test]
    fn test_dtg_values_negative_during_loss() {
        let tga = make_simple_tga();
        let dtg = DtgCurve::compute(&tga);
        // During decomposition, dm/dT should be negative
        let peak_idx = dtg.len() / 2; // approximately at midpoint
        assert!(dtg[peak_idx].1 < 0.0, "DTG should be negative during mass loss");
    }

    #[test]
    fn test_dtg_peak_temperatures() {
        let tga = make_simple_tga();
        let dtg = DtgCurve::compute(&tga);
        let peaks = DtgCurve::peak_temperatures(&dtg);
        assert!(!peaks.is_empty(), "Should find at least one peak");
        // Peak should be near the midpoint of decomposition (200+400)/2 = 300
        assert!(peaks[0] > 200.0 && peaks[0] < 400.0, "Peak at {} outside expected range", peaks[0]);
    }

    #[test]
    fn test_dtg_onset_temperature() {
        let tga = make_simple_tga();
        let onset = DtgCurve::onset_temperature(&tga);
        // Onset should be before peak and near 200°C
        assert!(onset < 350.0, "Onset {} should be well before peak", onset);
    }

    #[test]
    fn test_dtg_endset_temperature() {
        let tga = make_simple_tga();
        let endset = DtgCurve::endset_temperature(&tga);
        // Endset should be after peak and near 400°C
        assert!(endset > 250.0, "Endset {} should be after peak", endset);
    }

    #[test]
    fn test_dtg_smooth() {
        let tga = make_simple_tga();
        let dtg = DtgCurve::compute(&tga);
        let smoothed = DtgCurve::smooth(&dtg, 5);
        assert_eq!(smoothed.len(), dtg.len());
    }

    #[test]
    fn test_dtg_smooth_reduces_variance() {
        let tga = TgaSimulator::add_noise(&make_simple_tga(), 0.01);
        let dtg = DtgCurve::compute(&tga);
        let smoothed = DtgCurve::smooth(&dtg, 11);

        let var_orig: f64 = dtg.iter().map(|(_, y)| y * y).sum::<f64>() / dtg.len() as f64;
        let var_smooth: f64 = smoothed.iter().map(|(_, y)| y * y).sum::<f64>() / smoothed.len() as f64;

        // Smoothed should have less or equal energy
        assert!(var_smooth <= var_orig * 1.1, "Smoothing should reduce variance");
    }

    // ─── DecompositionStep tests ────────────────────────────────────

    #[test]
    fn test_find_single_step() {
        let tga = make_simple_tga();
        let steps = DecompositionStep::find_steps(&tga, 5.0);
        assert!(!steps.is_empty(), "Should find at least one step");
    }

    #[test]
    fn test_find_steps_threshold() {
        let tga = make_simple_tga();
        let steps_low = DecompositionStep::find_steps(&tga, 1.0);
        let steps_high = DecompositionStep::find_steps(&tga, 50.0);
        assert!(steps_low.len() >= steps_high.len());
    }

    #[test]
    fn test_step_midpoint() {
        let step = TgaStep {
            onset_c: 200.0,
            endset_c: 400.0,
            peak_c: 300.0,
            mass_loss_pct: 30.0,
            rate_max: 0.5,
        };
        assert!(approx_eq(DecompositionStep::step_midpoint(&step), 300.0, 1e-10));
    }

    #[test]
    fn test_atmosphere_effect_thermal() {
        let air_steps = vec![TgaStep {
            onset_c: 200.0, endset_c: 350.0, peak_c: 280.0, mass_loss_pct: 20.0, rate_max: 0.5,
        }];
        let n2_steps = vec![TgaStep {
            onset_c: 200.0, endset_c: 350.0, peak_c: 285.0, mass_loss_pct: 20.0, rate_max: 0.5,
        }];
        let effects = DecompositionStep::assign_atmosphere_effect(&air_steps, &n2_steps);
        assert_eq!(effects[0], AtmosphereEffect::ThermalDecomposition);
    }

    #[test]
    fn test_atmosphere_effect_oxidative() {
        let air_steps = vec![TgaStep {
            onset_c: 200.0, endset_c: 350.0, peak_c: 260.0, mass_loss_pct: 20.0, rate_max: 0.5,
        }];
        let n2_steps = vec![TgaStep {
            onset_c: 200.0, endset_c: 400.0, peak_c: 295.0, mass_loss_pct: 20.0, rate_max: 0.5,
        }];
        let effects = DecompositionStep::assign_atmosphere_effect(&air_steps, &n2_steps);
        assert_eq!(effects[0], AtmosphereEffect::OxidativeDecomposition);
    }

    #[test]
    fn test_atmosphere_effect_combustion() {
        let air_steps = vec![TgaStep {
            onset_c: 500.0, endset_c: 650.0, peak_c: 580.0, mass_loss_pct: 30.0, rate_max: 0.8,
        }];
        let n2_steps: Vec<TgaStep> = vec![]; // No matching step in N2
        let effects = DecompositionStep::assign_atmosphere_effect(&air_steps, &n2_steps);
        assert_eq!(effects[0], AtmosphereEffect::Combustion);
    }

    #[test]
    fn test_atmosphere_effect_pyrolysis() {
        let air_steps: Vec<TgaStep> = vec![];
        let n2_steps = vec![TgaStep {
            onset_c: 400.0, endset_c: 550.0, peak_c: 480.0, mass_loss_pct: 15.0, rate_max: 0.3,
        }];
        let effects = DecompositionStep::assign_atmosphere_effect(&air_steps, &n2_steps);
        assert_eq!(effects[0], AtmosphereEffect::Pyrolysis);
    }

    // ─── KineticsAnalyzer tests ─────────────────────────────────────

    #[test]
    fn test_kissinger_basic() {
        // Simulated data: heating rates 5, 10, 20 K/min
        // Peak temperatures shift with heating rate
        let betas = vec![5.0, 10.0, 20.0];
        let tp_k = vec![573.0, 583.0, 595.0]; // Kelvin

        let result = KineticsAnalyzer::kissinger(&betas, &tp_k);
        assert!(result.activation_energy_kj_mol > 0.0, "Ea should be positive");
        assert!(result.r_squared > 0.9, "R² should be high: {}", result.r_squared);
    }

    #[test]
    fn test_kissinger_four_rates() {
        let betas = vec![2.0, 5.0, 10.0, 20.0];
        let tp_k = vec![560.0, 573.0, 583.0, 595.0];
        let result = KineticsAnalyzer::kissinger(&betas, &tp_k);
        assert!(result.activation_energy_kj_mol > 50.0, "Ea={} kJ/mol seems too low", result.activation_energy_kj_mol);
        assert!(result.activation_energy_kj_mol < 500.0, "Ea={} kJ/mol seems too high", result.activation_energy_kj_mol);
    }

    #[test]
    fn test_conversion() {
        assert!(approx_eq(KineticsAnalyzer::conversion(100.0, 100.0, 60.0), 0.0, 1e-10));
        assert!(approx_eq(KineticsAnalyzer::conversion(80.0, 100.0, 60.0), 0.5, 1e-10));
        assert!(approx_eq(KineticsAnalyzer::conversion(60.0, 100.0, 60.0), 1.0, 1e-10));
    }

    #[test]
    fn test_conversion_clamped() {
        assert!(approx_eq(KineticsAnalyzer::conversion(110.0, 100.0, 60.0), 0.0, 1e-10));
        assert!(approx_eq(KineticsAnalyzer::conversion(50.0, 100.0, 60.0), 1.0, 1e-10));
    }

    #[test]
    fn test_conversion_zero_range() {
        assert!(approx_eq(KineticsAnalyzer::conversion(100.0, 100.0, 100.0), 0.0, 1e-10));
    }

    #[test]
    fn test_ozawa_flynn_wall() {
        let betas = vec![5.0, 10.0, 20.0];
        // Temperatures at α=0.5 for each heating rate
        let temps = vec![vec![573.0, 583.0, 595.0]];
        let ea = KineticsAnalyzer::ozawa_flynn_wall(&betas, &temps);
        assert_eq!(ea.len(), 1);
        assert!(ea[0] > 0.0, "Ea should be positive: {}", ea[0]);
    }

    #[test]
    fn test_friedman_with_simulated_curves() {
        // Three curves at different heating rates
        let curve1 = TgaSimulator::simulate_single_step(200.0, 400.0, 50.0, 5.0);
        let curve2 = TgaSimulator::simulate_single_step(210.0, 420.0, 50.0, 10.0);
        let curve3 = TgaSimulator::simulate_single_step(220.0, 440.0, 50.0, 20.0);

        let curves = vec![curve1, curve2, curve3];
        let ea = KineticsAnalyzer::friedman(&curves, &[0.3, 0.5, 0.7]);
        assert_eq!(ea.len(), 3);
    }

    // ─── ProximateAnalysis tests ────────────────────────────────────

    #[test]
    fn test_moisture_content() {
        // 10 mg sample loses 0.5 mg by 105°C
        let tga = TgaCurve::new(
            vec![25.0, 50.0, 80.0, 105.0, 200.0, 400.0, 900.0],
            vec![10.0, 9.9, 9.7, 9.5, 8.0, 6.0, 2.0],
            vec![0.0, 2.5, 5.5, 8.0, 17.5, 37.5, 87.5],
        );
        let moisture = ProximateAnalysis::moisture_content(&tga, 105.0);
        assert!(approx_eq(moisture, 5.0, 0.1));
    }

    #[test]
    fn test_volatile_matter() {
        let tga = TgaCurve::new(
            vec![25.0, 105.0, 500.0, 900.0],
            vec![10.0, 9.5, 6.0, 3.0],
            vec![0.0, 8.0, 47.5, 87.5],
        );
        let vm = ProximateAnalysis::volatile_matter(&tga, 105.0, 900.0);
        // (9.5 - 3.0)/10.0 * 100 = 65%
        assert!(approx_eq(vm, 65.0, 0.1));
    }

    #[test]
    fn test_fixed_carbon() {
        let fc = ProximateAnalysis::fixed_carbon(35.0, 10.0, 5.0);
        assert!(approx_eq(fc, 50.0, 1e-10));
    }

    #[test]
    fn test_fixed_carbon_non_negative() {
        let fc = ProximateAnalysis::fixed_carbon(60.0, 45.0, 5.0);
        assert!(fc >= 0.0, "Fixed carbon should not be negative");
    }

    #[test]
    fn test_ash_content() {
        let tga = TgaCurve::new(
            vec![25.0, 900.0],
            vec![10.0, 1.5],
            vec![0.0, 87.5],
        );
        assert!(approx_eq(ProximateAnalysis::ash_content(&tga), 15.0, 1e-10));
    }

    #[test]
    fn test_proximate_sums_to_100() {
        let moisture = 5.0;
        let volatile = 35.0;
        let ash = 10.0;
        let fc = ProximateAnalysis::fixed_carbon(volatile, ash, moisture);
        let total = moisture + volatile + fc + ash;
        assert!(approx_eq(total, 100.0, 1e-10));
    }

    // ─── OxidationAnalysis tests ────────────────────────────────────

    #[test]
    fn test_oxidation_onset_basic() {
        let tga = make_simple_tga();
        let onset = OxidationAnalysis::oxidation_onset(&tga);
        assert!(onset > 25.0 && onset < 500.0, "Onset {} out of range", onset);
    }

    #[test]
    fn test_oxidation_rate() {
        let tga = make_simple_tga();
        let rate = OxidationAnalysis::oxidation_rate(&tga, 300.0);
        // During decomposition the mass is decreasing → negative rate
        assert!(rate < 0.0, "Rate should be negative during mass loss: {}", rate);
    }

    #[test]
    fn test_parabolic_rate_constant() {
        // Δm² = k*t → linear in time
        let times = vec![0.0, 10.0, 20.0, 30.0, 40.0];
        let gains = vec![0.0, 1.0, 1.414, 1.732, 2.0]; // sqrt(k*t) with k=0.1
        let k = OxidationAnalysis::parabolic_rate_constant(&times, &gains);
        assert!(k > 0.0, "Rate constant should be positive");
        assert!(approx_eq(k, 0.1, 0.02));
    }

    #[test]
    fn test_activation_energy_from_rates() {
        // Arrhenius: rate = A * exp(-Ea/RT)
        // Using Ea = 100 kJ/mol
        let ea_true = 100.0; // kJ/mol
        let temps_k = vec![500.0, 550.0, 600.0, 650.0];
        let rates: Vec<f64> = temps_k
            .iter()
            .map(|t| 1e10 * (-ea_true / (R_KJ * t)).exp())
            .collect();

        let ea = OxidationAnalysis::activation_energy_from_rates(&temps_k, &rates);
        assert!(approx_eq(ea, ea_true, 1.0), "Ea={} should be ~{}", ea, ea_true);
    }

    // ─── ThermalStability tests ─────────────────────────────────────

    #[test]
    fn test_t5_percent_loss() {
        let tga = make_simple_tga();
        let t5 = ThermalStability::t5_percent_loss(&tga);
        assert!(t5 > 25.0 && t5 < 500.0, "T5% = {}", t5);
    }

    #[test]
    fn test_t10_percent_loss() {
        let tga = make_simple_tga();
        let t10 = ThermalStability::t10_percent_loss(&tga);
        let t5 = ThermalStability::t5_percent_loss(&tga);
        assert!(t10 >= t5, "T10%={} should be >= T5%={}", t10, t5);
    }

    #[test]
    fn test_t50_percent_loss() {
        // 30% total loss → T50% should be at last temp since we never reach 50% loss
        let tga = make_simple_tga();
        let t50 = ThermalStability::t50_percent_loss(&tga);
        // Since max loss is ~30%, t50 should be the last temperature
        assert!(t50 > 400.0, "T50%={} for 30% loss curve", t50);
    }

    #[test]
    fn test_t_ordering() {
        // Curve with 60% loss to ensure all thresholds are crossed
        let tga = TgaSimulator::simulate_single_step(200.0, 400.0, 60.0, 10.0);
        let t5 = ThermalStability::t5_percent_loss(&tga);
        let t10 = ThermalStability::t10_percent_loss(&tga);
        let t50 = ThermalStability::t50_percent_loss(&tga);
        assert!(t5 <= t10, "T5%={} should be <= T10%={}", t5, t10);
        assert!(t10 <= t50, "T10%={} should be <= T50%={}", t10, t50);
    }

    #[test]
    fn test_ipdt() {
        let tga = make_simple_tga();
        let ipdt = ThermalStability::integral_procedural_decomposition_temp(&tga);
        // IPDT should be within the temperature range
        assert!(ipdt > tga.temperature_c[0], "IPDT={}", ipdt);
        assert!(ipdt < *tga.temperature_c.last().unwrap(), "IPDT={}", ipdt);
    }

    #[test]
    fn test_char_yield() {
        let tga = make_simple_tga();
        let cy_low = ThermalStability::char_yield(&tga, 100.0);
        let cy_high = ThermalStability::char_yield(&tga, 400.0);
        assert!(cy_low > cy_high, "Char yield should decrease: {} vs {}", cy_low, cy_high);
    }

    #[test]
    fn test_char_yield_initial() {
        let tga = make_simple_tga();
        let cy = ThermalStability::char_yield(&tga, 25.0);
        assert!(approx_eq(cy, 100.0, 0.5));
    }

    // ─── BaselineCorrection tests ───────────────────────────────────

    #[test]
    fn test_buoyancy_correction() {
        let sample = TgaCurve::new(
            vec![25.0, 200.0, 400.0],
            vec![10.0, 9.5, 7.0],
            vec![0.0, 17.5, 37.5],
        );
        let blank = TgaCurve::new(
            vec![25.0, 200.0, 400.0],
            vec![0.1, 0.12, 0.15], // Slight buoyancy drift
            vec![0.0, 17.5, 37.5],
        );
        let corrected = BaselineCorrection::buoyancy_correction(&sample, &blank);
        // Initial mass unchanged (blank drift at t=25 is 0)
        assert!(approx_eq(corrected.mass_mg[0], 10.0, 1e-10));
        // Later points should be slightly adjusted
        assert!(corrected.mass_mg[1] < sample.mass_mg[1], "Buoyancy correction should reduce mass");
    }

    #[test]
    fn test_drift_correction() {
        let tga = TgaCurve::new(
            vec![25.0, 100.0, 200.0],
            vec![10.0, 9.1, 8.0],
            vec![0.0, 7.5, 17.5],
        );
        let corrected = BaselineCorrection::drift_correction(&tga, 0.01);
        // At t=0, correction is 0
        assert!(approx_eq(corrected.mass_mg[0], 10.0, 1e-10));
        // At t=7.5, correction is -0.075
        assert!(approx_eq(corrected.mass_mg[1], 9.1 - 0.01 * 7.5, 1e-10));
    }

    #[test]
    fn test_smooth_curve() {
        let tga = TgaSimulator::add_noise(&make_simple_tga(), 0.05);
        let smoothed = BaselineCorrection::smooth_curve(&tga, 5);
        assert_eq!(smoothed.len(), tga.len());
        // Temperature should be preserved
        assert!(approx_eq(smoothed.temperature_c[0], tga.temperature_c[0], 1e-10));
    }

    // ─── TgaSimulator tests ────────────────────────────────────────

    #[test]
    fn test_simulate_single_step_length() {
        let tga = TgaSimulator::simulate_single_step(200.0, 400.0, 30.0, 10.0);
        assert_eq!(tga.len(), 1000);
    }

    #[test]
    fn test_simulate_single_step_loss() {
        let tga = TgaSimulator::simulate_single_step(200.0, 400.0, 30.0, 10.0);
        let loss = tga.mass_loss_total();
        assert!(approx_eq(loss, 30.0, 1.0), "Loss={}, expected ~30%", loss);
    }

    #[test]
    fn test_simulate_single_step_monotonic() {
        let tga = TgaSimulator::simulate_single_step(200.0, 400.0, 30.0, 10.0);
        for i in 1..tga.mass_mg.len() {
            assert!(
                tga.mass_mg[i] <= tga.mass_mg[i - 1] + 1e-12,
                "Mass should be monotonically non-increasing"
            );
        }
    }

    #[test]
    fn test_simulate_multi_step() {
        let tga = make_two_step_tga();
        let loss = tga.mass_loss_total();
        assert!(loss > 35.0 && loss < 50.0, "Total loss={} for 20%+25% steps", loss);
    }

    #[test]
    fn test_simulate_multi_step_three_steps() {
        let tga = TgaSimulator::simulate_multi_step(
            &[(150.0, 250.0, 10.0), (300.0, 400.0, 20.0), (500.0, 650.0, 15.0)],
            10.0,
        );
        let loss = tga.mass_loss_total();
        assert!(loss > 35.0, "Total loss={} expected ~45%", loss);
    }

    #[test]
    fn test_add_noise() {
        let clean = make_simple_tga();
        let noisy = TgaSimulator::add_noise(&clean, 0.01);
        assert_eq!(noisy.len(), clean.len());
        // At least some points should differ
        let diffs: f64 = clean
            .mass_mg
            .iter()
            .zip(noisy.mass_mg.iter())
            .map(|(a, b)| (a - b).abs())
            .sum();
        assert!(diffs > 0.0, "Noise should change some values");
    }

    #[test]
    fn test_simulate_isothermal() {
        let tga = TgaSimulator::simulate_isothermal(300.0, 60.0, 0.05, 1.0);
        assert_eq!(tga.len(), 500);
        // Temperature should be constant
        for &t in &tga.temperature_c {
            assert!(approx_eq(t, 300.0, 1e-10));
        }
        // Mass should decrease
        assert!(tga.final_mass() < tga.initial_mass());
    }

    #[test]
    fn test_isothermal_first_order() {
        // First order: α(t) = 1 - exp(-kt) for n=1
        let k = 0.1;
        let tga = TgaSimulator::simulate_isothermal(300.0, 30.0, k, 1.0);
        let m0 = tga.initial_mass();
        let mf = tga.final_mass();
        let alpha_final = 1.0 - mf / m0;
        let alpha_theory = 1.0 - (-k * 30.0_f64).exp();
        assert!(
            approx_eq(alpha_final, alpha_theory, 0.05),
            "α_sim={} vs α_theory={}",
            alpha_final,
            alpha_theory
        );
    }

    #[test]
    fn test_isothermal_higher_order() {
        // Higher order should decompose slower initially
        let tga_n1 = TgaSimulator::simulate_isothermal(300.0, 30.0, 0.05, 1.0);
        let tga_n2 = TgaSimulator::simulate_isothermal(300.0, 30.0, 0.05, 2.0);
        // Both should lose mass
        assert!(tga_n1.final_mass() < tga_n1.initial_mass());
        assert!(tga_n2.final_mass() < tga_n2.initial_mass());
    }

    // ─── ReportGenerator tests ──────────────────────────────────────

    #[test]
    fn test_summary() {
        let tga = make_simple_tga();
        let steps = DecompositionStep::find_steps(&tga, 5.0);
        let summary = ReportGenerator::summary(&tga, &steps);
        assert!(summary.initial_mass > 0.0);
        assert!(summary.total_loss > 0.0);
        assert_eq!(summary.num_steps, steps.len());
    }

    #[test]
    fn test_compare_curves_identical() {
        let c1 = make_simple_tga();
        let c2 = make_simple_tga();
        let result = ReportGenerator::compare_curves(&[c1, c2]);
        assert_eq!(result.num_curves, 2);
        assert!(result.std_total_loss < 0.1, "Identical curves should have ~0 std");
        assert!(result.max_divergence < 0.1, "Identical curves should not diverge");
    }

    #[test]
    fn test_compare_curves_different() {
        let c1 = TgaSimulator::simulate_single_step(200.0, 400.0, 30.0, 10.0);
        let c2 = TgaSimulator::simulate_single_step(200.0, 400.0, 50.0, 10.0);
        let result = ReportGenerator::compare_curves(&[c1, c2]);
        assert!(result.std_total_loss > 5.0, "Different curves should have high std");
        assert!(result.max_divergence > 5.0, "Different curves should diverge");
    }

    #[test]
    fn test_compare_curves_empty() {
        let result = ReportGenerator::compare_curves(&[]);
        assert_eq!(result.num_curves, 0);
    }

    #[test]
    fn test_compare_curves_single() {
        let c1 = make_simple_tga();
        let result = ReportGenerator::compare_curves(&[c1]);
        assert_eq!(result.num_curves, 1);
        assert!(approx_eq(result.std_total_loss, 0.0, 1e-10));
    }

    // ─── Helper function tests ──────────────────────────────────────

    #[test]
    fn test_linear_interp_basic() {
        let x = vec![0.0, 1.0, 2.0];
        let y = vec![0.0, 10.0, 20.0];
        assert!(approx_eq(linear_interp(&x, &y, 0.5), 5.0, 1e-10));
        assert!(approx_eq(linear_interp(&x, &y, 1.5), 15.0, 1e-10));
    }

    #[test]
    fn test_linear_interp_extrapolate() {
        let x = vec![0.0, 1.0];
        let y = vec![0.0, 10.0];
        // Clamp at boundaries
        assert!(approx_eq(linear_interp(&x, &y, -1.0), 0.0, 1e-10));
        assert!(approx_eq(linear_interp(&x, &y, 2.0), 10.0, 1e-10));
    }

    #[test]
    fn test_linear_regression_perfect() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![1.0, 3.0, 5.0, 7.0]; // y = 2x + 1
        let (slope, intercept) = linear_regression(&x, &y);
        assert!(approx_eq(slope, 2.0, 1e-10));
        assert!(approx_eq(intercept, 1.0, 1e-10));
    }

    #[test]
    fn test_r_squared_perfect() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![1.0, 3.0, 5.0, 7.0];
        let r2 = r_squared(&x, &y, 2.0, 1.0);
        assert!(approx_eq(r2, 1.0, 1e-10));
    }

    #[test]
    fn test_nearest_index() {
        let arr = vec![10.0, 20.0, 30.0, 40.0, 50.0];
        assert_eq!(nearest_index(&arr, 10.0), 0);
        assert_eq!(nearest_index(&arr, 25.0), 1); // equidistant, picks first
        assert_eq!(nearest_index(&arr, 50.0), 4);
        assert_eq!(nearest_index(&arr, 0.0), 0);
        assert_eq!(nearest_index(&arr, 100.0), 4);
    }

    #[test]
    fn test_moving_average_identity() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let result = moving_average(&data, 1);
        assert_eq!(result, data);
    }

    #[test]
    fn test_moving_average_constant() {
        let data = vec![5.0, 5.0, 5.0, 5.0, 5.0];
        let result = moving_average(&data, 3);
        for &v in &result {
            assert!(approx_eq(v, 5.0, 1e-10));
        }
    }

    // ─── Integration / end-to-end tests ─────────────────────────────

    #[test]
    fn test_full_workflow() {
        // Generate → Analyze → Report
        let tga = TgaSimulator::simulate_single_step(250.0, 400.0, 40.0, 10.0);
        let steps = DecompositionStep::find_steps(&tga, 5.0);
        let summary = ReportGenerator::summary(&tga, &steps);
        let t5 = ThermalStability::t5_percent_loss(&tga);
        let t10 = ThermalStability::t10_percent_loss(&tga);

        assert!(summary.total_loss > 30.0, "Total loss = {}", summary.total_loss);
        assert!(t5 < t10, "T5 < T10");
    }

    #[test]
    fn test_multi_step_workflow() {
        let tga = make_two_step_tga();
        let dtg = DtgCurve::compute(&tga);
        let peaks = DtgCurve::peak_temperatures(&dtg);

        // Two-step curve should have peaks
        assert!(!peaks.is_empty(), "Should detect peaks in two-step curve");
    }

    #[test]
    fn test_noisy_curve_analysis() {
        let clean = make_simple_tga();
        let noisy = TgaSimulator::add_noise(&clean, 0.01);
        let smoothed = BaselineCorrection::smooth_curve(&noisy, 5);

        let loss_clean = clean.mass_loss_total();
        let loss_smoothed = smoothed.mass_loss_total();
        assert!(
            (loss_clean - loss_smoothed).abs() < 2.0,
            "Smoothed loss={} vs clean loss={}",
            loss_smoothed,
            loss_clean
        );
    }

    #[test]
    fn test_proximate_coal_like() {
        // Simulated coal-like TGA: moisture loss < 105, volatiles 105-900, ash residue
        let tga = TgaCurve::new(
            vec![25.0, 105.0, 300.0, 600.0, 900.0],
            vec![10.0, 9.5, 7.5, 5.0, 1.0],
            vec![0.0, 8.0, 27.5, 57.5, 87.5],
        );
        let moisture = ProximateAnalysis::moisture_content(&tga, 105.0);
        let volatile = ProximateAnalysis::volatile_matter(&tga, 105.0, 900.0);
        let ash = ProximateAnalysis::ash_content(&tga);
        let fc = ProximateAnalysis::fixed_carbon(volatile, ash, moisture);

        assert!(moisture > 0.0);
        assert!(volatile > 0.0);
        assert!(ash > 0.0);
        assert!(fc >= 0.0);
        let total = moisture + volatile + fc + ash;
        assert!(approx_eq(total, 100.0, 0.1), "Proximate should sum to 100: {}", total);
    }

    #[test]
    fn test_ipdt_higher_stability_means_higher_ipdt() {
        let stable = TgaSimulator::simulate_single_step(350.0, 550.0, 40.0, 10.0);
        let unstable = TgaSimulator::simulate_single_step(150.0, 300.0, 40.0, 10.0);

        let ipdt_stable = ThermalStability::integral_procedural_decomposition_temp(&stable);
        let ipdt_unstable = ThermalStability::integral_procedural_decomposition_temp(&unstable);

        assert!(
            ipdt_stable > ipdt_unstable,
            "Stable IPDT={} should > unstable IPDT={}",
            ipdt_stable,
            ipdt_unstable
        );
    }
}
