//! # Turbidimetry & Nephelometry Processor
//!
//! Turbidity and nephelometry data analysis for measuring suspended particle
//! concentration via light scattering and attenuation. Implements NTU calibration,
//! Beer-Lambert attenuation, Rayleigh-Mie scattering theory, multi-angle detection,
//! kinetics analysis, water treatment monitoring, and concentration estimation.
//!
//! ## Physics Background
//!
//! - **Turbidity**: Measured in NTU (Nephelometric Turbidity Units) from 90-degree
//!   scattered light intensity relative to formazin standards.
//! - **Beer-Lambert**: T = I/I0 = exp(-tau * l), OD = -log10(T).
//! - **Rayleigh scattering**: I proportional to d^6 / lambda^4 for d << lambda.
//! - **Mie scattering**: Full solution for d ~ lambda (larger particles).
//! - **Formazin**: Primary turbidity standard, 4000 NTU stock solution.
//! - **WHO drinking water**: < 1 NTU; EPA filtered water: < 0.3 NTU.

use std::f64::consts::{LN_2, PI};

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

/// Water quality classification based on NTU.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaterQuality {
    /// < 1 NTU – meets WHO drinking water guideline
    Drinking,
    /// 1..5 NTU – acceptable for most uses
    Acceptable,
    /// 5..50 NTU – marginal, needs treatment
    Marginal,
    /// >= 50 NTU – poor, significant treatment required
    Poor,
}

/// Regulatory standard for compliance checking.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegulatoryStandard {
    /// WHO guideline: < 1 NTU
    Who,
    /// US EPA filtered surface water: < 0.3 NTU (95th percentile)
    EpaFiltered,
    /// US EPA unfiltered: < 5 NTU
    EpaUnfiltered,
    /// EU Drinking Water Directive: < 1 NTU
    EuDrinkingWater,
}

/// Scattering pattern classification from angular distribution.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScatteringPattern {
    /// Symmetric (d << lambda)
    Rayleigh,
    /// Forward-biased (d ~ lambda)
    Mie,
    /// Strongly forward-biased (d >> lambda)
    Large,
}

// ---------------------------------------------------------------------------
// Helper results
// ---------------------------------------------------------------------------

/// Result of polynomial calibration.
#[derive(Debug, Clone)]
pub struct CalibResult {
    /// Polynomial coefficients [a0, a1, a2, ...] so that NTU = a0 + a1*s + a2*s^2 + ...
    pub coefficients: Vec<f64>,
    /// R-squared goodness-of-fit.
    pub r_squared: f64,
}

/// Formazin dilution recipe from 4000 NTU stock.
#[derive(Debug, Clone)]
pub struct FormazinRecipe {
    /// Volume of 4000 NTU stock to add (mL).
    pub stock_volume_ml: f64,
    /// Volume of dilution water (mL).
    pub dilution_water_ml: f64,
    /// Target NTU of the diluted standard.
    pub target_ntu: f64,
}

/// Jar test analysis result.
#[derive(Debug, Clone)]
pub struct JarTestResult {
    /// Coagulant dose producing minimum residual turbidity (mg/L).
    pub optimal_dose: f64,
    /// Minimum residual NTU achieved.
    pub minimum_ntu: f64,
    /// Slope of dose–response around the optimum (NTU per mg/L).
    pub dose_response_slope: f64,
}

// ---------------------------------------------------------------------------
// 1. TurbidityMeasurement
// ---------------------------------------------------------------------------

/// Container for a time-series of NTU readings.
#[derive(Debug, Clone)]
pub struct TurbidityMeasurement {
    pub readings_ntu: Vec<f64>,
    pub timestamps_s: Vec<f64>,
}

impl TurbidityMeasurement {
    pub fn new(readings_ntu: Vec<f64>, timestamps_s: Vec<f64>) -> Self {
        assert_eq!(readings_ntu.len(), timestamps_s.len(), "length mismatch");
        Self { readings_ntu, timestamps_s }
    }

    pub fn mean_ntu(&self) -> f64 {
        if self.readings_ntu.is_empty() {
            return 0.0;
        }
        self.readings_ntu.iter().sum::<f64>() / self.readings_ntu.len() as f64
    }

    pub fn median_ntu(&self) -> f64 {
        if self.readings_ntu.is_empty() {
            return 0.0;
        }
        let mut sorted = self.readings_ntu.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let n = sorted.len();
        if n % 2 == 0 {
            (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0
        } else {
            sorted[n / 2]
        }
    }

    pub fn std_deviation(&self) -> f64 {
        let n = self.readings_ntu.len();
        if n < 2 {
            return 0.0;
        }
        let mean = self.mean_ntu();
        let var = self.readings_ntu.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (n - 1) as f64;
        var.sqrt()
    }

    pub fn max_ntu(&self) -> f64 {
        self.readings_ntu.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
    }

    pub fn min_ntu(&self) -> f64 {
        self.readings_ntu.iter().cloned().fold(f64::INFINITY, f64::min)
    }

    /// Classify water quality based on NTU threshold.
    pub fn classify_water_quality(ntu: f64) -> WaterQuality {
        if ntu < 1.0 {
            WaterQuality::Drinking
        } else if ntu < 5.0 {
            WaterQuality::Acceptable
        } else if ntu < 50.0 {
            WaterQuality::Marginal
        } else {
            WaterQuality::Poor
        }
    }
}

// ---------------------------------------------------------------------------
// 2. CalibrationCurve
// ---------------------------------------------------------------------------

/// Polynomial calibration curve mapping detector signal to NTU.
#[derive(Debug, Clone)]
pub struct CalibrationCurve {
    standards_ntu: Vec<f64>,
    detector_signals: Vec<f64>,
    coefficients: Option<Vec<f64>>,
}

impl CalibrationCurve {
    pub fn new(standards_ntu: Vec<f64>, detector_signals: Vec<f64>) -> Self {
        assert_eq!(standards_ntu.len(), detector_signals.len(), "length mismatch");
        assert!(standards_ntu.len() >= 2, "need at least 2 calibration points");
        Self { standards_ntu, detector_signals, coefficients: None }
    }

    /// Perform polynomial fit (degree = min(n-1, 3)).
    pub fn calibrate(&mut self) -> CalibResult {
        let degree = (self.standards_ntu.len() - 1).min(3);
        let coeffs = poly_fit(&self.detector_signals, &self.standards_ntu, degree);
        let r2 = r_squared(&self.detector_signals, &self.standards_ntu, &coeffs);
        self.coefficients = Some(coeffs.clone());
        CalibResult { coefficients: coeffs, r_squared: r2 }
    }

    /// Convert a raw detector signal to NTU using the calibration polynomial.
    pub fn ntu_from_signal(&self, signal: f64) -> f64 {
        let coeffs = self.coefficients.as_ref().expect("call calibrate() first");
        poly_eval(coeffs, signal)
    }

    /// R-squared linearity check (linear fit).
    pub fn linearity_check(&self) -> f64 {
        let coeffs = poly_fit(&self.detector_signals, &self.standards_ntu, 1);
        r_squared(&self.detector_signals, &self.standards_ntu, &coeffs)
    }

    /// Dilution recipe to prepare a target NTU standard from 4000 NTU formazin stock.
    pub fn formazin_preparation(target_ntu: f64, total_volume_ml: f64) -> FormazinRecipe {
        let stock_vol = target_ntu * total_volume_ml / 4000.0;
        FormazinRecipe {
            stock_volume_ml: stock_vol,
            dilution_water_ml: total_volume_ml - stock_vol,
            target_ntu,
        }
    }
}

// ---------------------------------------------------------------------------
// 3. NephelometricRatio
// ---------------------------------------------------------------------------

/// Ratio-mode nephelometry for colour/absorption compensation.
pub struct NephelometricRatio;

impl NephelometricRatio {
    /// Ratio of 90-degree scatter to transmitted light – compensates colour.
    pub fn ratio_mode(scattered_90: f64, transmitted: f64) -> f64 {
        if transmitted <= 0.0 {
            return f64::INFINITY;
        }
        scattered_90 / transmitted
    }

    /// Backscatter ratio: I(90) / I(135).
    pub fn backscatter_ratio(scattered_90: f64, backscatter_135: f64) -> f64 {
        if backscatter_135 <= 0.0 {
            return f64::INFINITY;
        }
        scattered_90 / backscatter_135
    }

    /// Correct NTU reading for colour absorption.
    /// Empirical: NTU_corrected = NTU - k * colour_abs, where k ~ 0.5.
    pub fn forward_scatter_correction(ntu: f64, color_abs: f64) -> f64 {
        let k = 0.5;
        (ntu - k * color_abs).max(0.0)
    }
}

// ---------------------------------------------------------------------------
// 4. BeerLambertTurbidity
// ---------------------------------------------------------------------------

/// Attenuation-based (transmittance) turbidity measurements.
pub struct BeerLambertTurbidity;

impl BeerLambertTurbidity {
    /// Turbidity coefficient tau = -ln(T) / l where T = I_transmitted / I0.
    pub fn turbidity_from_transmittance(i0: f64, i_transmitted: f64, path_cm: f64) -> f64 {
        if i0 <= 0.0 || i_transmitted <= 0.0 || path_cm <= 0.0 {
            return 0.0;
        }
        let t = i_transmitted / i0;
        -(t.ln()) / path_cm
    }

    /// Optical density OD = -log10(T).
    pub fn optical_density(transmittance: f64) -> f64 {
        if transmittance <= 0.0 {
            return f64::INFINITY;
        }
        -(transmittance.log10())
    }

    /// Convert attenuation coefficient to NTU via a calibration factor.
    pub fn ntu_from_attenuation(attenuation: f64, calibration_factor: f64) -> f64 {
        attenuation * calibration_factor
    }

    /// Specific turbidity = NTU / concentration (mg/L).
    pub fn specific_turbidity(ntu: f64, concentration_mg_per_l: f64) -> f64 {
        if concentration_mg_per_l <= 0.0 {
            return 0.0;
        }
        ntu / concentration_mg_per_l
    }
}

// ---------------------------------------------------------------------------
// 5. ParticleSizeEstimator
// ---------------------------------------------------------------------------

/// Estimate particle size from scattering wavelength dependence.
pub struct ParticleSizeEstimator;

impl ParticleSizeEstimator {
    /// Rayleigh regime check: d < lambda / 10.
    pub fn rayleigh_regime(wavelength_nm: f64, particle_size_nm: f64) -> bool {
        particle_size_nm < wavelength_nm / 10.0
    }

    /// Rayleigh scattering intensity (relative).
    /// I proportional to (d^6 / lambda^4) * ((n_p/n_m)^2 - 1)^2 / ((n_p/n_m)^2 + 2)^2
    pub fn rayleigh_scattering_intensity(
        size_nm: f64,
        wavelength_nm: f64,
        n_particle: f64,
        n_medium: f64,
    ) -> f64 {
        if wavelength_nm <= 0.0 || n_medium <= 0.0 {
            return 0.0;
        }
        let m = n_particle / n_medium;
        let m2 = m * m;
        let factor = ((m2 - 1.0) / (m2 + 2.0)).powi(2);
        let geo = size_nm.powi(6) / wavelength_nm.powi(4);
        geo * factor
    }

    /// Mie size parameter x = pi * d / lambda.
    pub fn mie_parameter(size_nm: f64, wavelength_nm: f64) -> f64 {
        PI * size_nm / wavelength_nm
    }

    /// Turbidity ratio at two wavelengths (red / IR) for size estimation.
    pub fn turbidity_ratio(ntu_red: f64, ntu_ir: f64) -> f64 {
        if ntu_ir <= 0.0 {
            return f64::INFINITY;
        }
        ntu_red / ntu_ir
    }

    /// Approximate mean particle size (nm) from wavelength ratio.
    /// Empirical mapping: ratio ~ 1 => large; ratio >> 1 => small (Rayleigh d^6/lambda^4).
    /// Uses a simple power-law model: d_nm ≈ 200 / ratio^1.5 (crude but illustrative).
    pub fn approximate_size_from_ratio(ratio: f64) -> f64 {
        if ratio <= 0.0 {
            return 0.0;
        }
        200.0 / ratio.powf(1.5)
    }
}

// ---------------------------------------------------------------------------
// 6. KineticsTurbidity
// ---------------------------------------------------------------------------

/// Time-resolved turbidity change analysis.
pub struct KineticsTurbidity;

impl KineticsTurbidity {
    /// Initial aggregation rate (slope of the first few points via linear regression).
    pub fn aggregation_rate(times: &[f64], ntu_values: &[f64]) -> f64 {
        assert_eq!(times.len(), ntu_values.len());
        if times.len() < 2 {
            return 0.0;
        }
        let n = times.len().min(10); // use first 10 points max
        linear_slope(&times[..n], &ntu_values[..n])
    }

    /// Settling rate: fit exponential decay y = A * exp(-k*t) + C, return k.
    pub fn settling_rate(times: &[f64], ntu_values: &[f64]) -> f64 {
        let (_, k, _) = Self::fit_exponential_decay(times, ntu_values);
        k
    }

    /// Half-life from rate constant: t1/2 = ln(2) / k.
    pub fn half_life(rate_constant: f64) -> f64 {
        if rate_constant <= 0.0 {
            return f64::INFINITY;
        }
        LN_2 / rate_constant
    }

    /// Fit y = A * exp(-k * t) + offset via linearised least-squares.
    /// Tries multiple offset candidates and selects the fit with minimum residual error.
    /// Returns (A, k, offset).
    pub fn fit_exponential_decay(times: &[f64], values: &[f64]) -> (f64, f64, f64) {
        assert_eq!(times.len(), values.len());
        let n = times.len();
        if n < 3 {
            return (0.0, 0.0, 0.0);
        }

        let val_min = values.iter().cloned().fold(f64::INFINITY, f64::min);
        let tail_n = (n / 10).max(1);
        let tail_avg = values[n - tail_n..].iter().sum::<f64>() / tail_n as f64;

        // Try several offset candidates.
        let candidates = [
            0.0,
            val_min * 0.5,
            val_min * 0.9,
            val_min * 0.99,
            tail_avg * 0.5,
            tail_avg * 0.9,
            tail_avg * 0.99,
            tail_avg,
        ];

        let mut best_sse = f64::INFINITY;
        let mut best = (values[0], 0.0, 0.0);

        for &offset_try in &candidates {
            if offset_try < 0.0 {
                continue;
            }
            let mut ln_vals = Vec::with_capacity(n);
            let mut valid_t = Vec::with_capacity(n);
            for i in 0..n {
                let shifted = values[i] - offset_try;
                if shifted > 1e-12 {
                    ln_vals.push(shifted.ln());
                    valid_t.push(times[i]);
                }
            }
            if valid_t.len() < 2 {
                continue;
            }
            let slope = linear_slope(&valid_t, &ln_vals);
            let intercept_val = linear_intercept(&valid_t, &ln_vals);
            let a = intercept_val.exp();
            let k = -slope;
            if k <= 0.0 {
                continue; // not a decay
            }
            // Compute SSE.
            let sse: f64 = times.iter().zip(values.iter()).map(|(&t, &v)| {
                let pred = a * (-k * t).exp() + offset_try;
                (v - pred).powi(2)
            }).sum();
            if sse < best_sse {
                best_sse = sse;
                best = (a, k, offset_try);
            }
        }
        best
    }

    /// Induction time: index where NTU starts rising above baseline + 3*sigma.
    /// Returns the time at which the rise begins.
    pub fn induction_time(times: &[f64], ntu_values: &[f64]) -> f64 {
        assert_eq!(times.len(), ntu_values.len());
        if ntu_values.len() < 10 {
            return times[0];
        }
        // Baseline from first 10% of data.
        let baseline_n = (ntu_values.len() / 10).max(3);
        let baseline_mean: f64 = ntu_values[..baseline_n].iter().sum::<f64>() / baseline_n as f64;
        let baseline_std = {
            let var = ntu_values[..baseline_n]
                .iter()
                .map(|x| (x - baseline_mean).powi(2))
                .sum::<f64>()
                / (baseline_n - 1) as f64;
            var.sqrt()
        };
        let threshold = baseline_mean + 3.0 * baseline_std.max(baseline_mean * 0.01);
        for i in baseline_n..ntu_values.len() {
            if ntu_values[i] > threshold {
                return times[i];
            }
        }
        *times.last().unwrap()
    }
}

// ---------------------------------------------------------------------------
// 7. WaterTreatmentMonitor
// ---------------------------------------------------------------------------

/// Process monitoring for water treatment plants.
pub struct WaterTreatmentMonitor;

impl WaterTreatmentMonitor {
    /// Analyse a jar test: find optimal coagulant dose.
    pub fn jar_test_analysis(doses: &[f64], residual_ntu: &[f64]) -> JarTestResult {
        assert_eq!(doses.len(), residual_ntu.len());
        assert!(!doses.is_empty());
        // Find index of minimum residual NTU.
        let mut min_idx = 0;
        let mut min_val = residual_ntu[0];
        for (i, &v) in residual_ntu.iter().enumerate() {
            if v < min_val {
                min_val = v;
                min_idx = i;
            }
        }
        // Slope around the optimum.
        let slope = if doses.len() >= 2 {
            linear_slope(doses, residual_ntu)
        } else {
            0.0
        };
        JarTestResult {
            optimal_dose: doses[min_idx],
            minimum_ntu: min_val,
            dose_response_slope: slope,
        }
    }

    /// Filter log-removal: LRV = log10(inlet_ntu / outlet_ntu).
    pub fn filter_performance(inlet_ntu: f64, outlet_ntu: f64) -> f64 {
        if outlet_ntu <= 0.0 || inlet_ntu <= 0.0 {
            return 0.0;
        }
        (inlet_ntu / outlet_ntu).log10()
    }

    /// Detect filter breakthrough: first index where NTU > threshold.
    pub fn breakthrough_detection(ntu_series: &[f64], threshold: f64) -> Option<usize> {
        ntu_series.iter().position(|&v| v > threshold)
    }

    /// Regulatory compliance check.
    pub fn regulatory_compliance(ntu: f64, standard: RegulatoryStandard) -> bool {
        match standard {
            RegulatoryStandard::Who => ntu < 1.0,
            RegulatoryStandard::EpaFiltered => ntu < 0.3,
            RegulatoryStandard::EpaUnfiltered => ntu < 5.0,
            RegulatoryStandard::EuDrinkingWater => ntu < 1.0,
        }
    }
}

// ---------------------------------------------------------------------------
// 8. ConcentrationEstimator
// ---------------------------------------------------------------------------

/// Estimate suspended-solids concentration from turbidity.
pub struct ConcentrationEstimator;

impl ConcentrationEstimator {
    /// TSS (mg/L) = slope * NTU + intercept.
    pub fn tss_from_ntu(ntu: f64, correlation_slope: f64, intercept: f64) -> f64 {
        correlation_slope * ntu + intercept
    }

    /// Calibrate TSS-NTU correlation from paired data, returns (slope, intercept, R²).
    pub fn calibrate_tss_ntu(ntu_values: &[f64], tss_values: &[f64]) -> (f64, f64, f64) {
        assert_eq!(ntu_values.len(), tss_values.len());
        let slope = linear_slope(ntu_values, tss_values);
        let intercept = linear_intercept(ntu_values, tss_values);
        let coeffs = vec![intercept, slope];
        let r2 = r_squared(ntu_values, tss_values, &coeffs);
        (slope, intercept, r2)
    }

    /// Secchi depth (m) from NTU: SD ≈ 244 / NTU^0.662 (empirical, Davies-Colley & Smith).
    pub fn secchi_depth_from_ntu(ntu: f64) -> f64 {
        if ntu <= 0.0 {
            return f64::INFINITY;
        }
        244.0 / ntu.powf(0.662)
    }

    /// Expected NTU contribution from chlorophyll-a (algae).
    /// Empirical: NTU ≈ 0.06 * Chl-a (ug/L) for typical algal suspensions.
    pub fn chlorophyll_contribution(chl_ug_per_l: f64) -> f64 {
        0.06 * chl_ug_per_l
    }
}

// ---------------------------------------------------------------------------
// 9. TurbiditySimulator
// ---------------------------------------------------------------------------

/// Generate synthetic turbidity data for testing.
pub struct TurbiditySimulator;

impl TurbiditySimulator {
    /// Steady-state with Gaussian noise (deterministic seeded PRNG).
    pub fn simulate_steady_state(mean_ntu: f64, noise: f64, num_points: usize) -> TurbidityMeasurement {
        let mut readings = Vec::with_capacity(num_points);
        let mut timestamps = Vec::with_capacity(num_points);
        let mut seed: u64 = 42;
        for i in 0..num_points {
            timestamps.push(i as f64);
            let r = pseudo_gaussian(&mut seed) * noise + mean_ntu;
            readings.push(r.max(0.0));
        }
        TurbidityMeasurement::new(readings, timestamps)
    }

    /// Exponential settling from initial_ntu toward 0 with rate constant.
    pub fn simulate_settling(initial_ntu: f64, rate: f64, duration_s: f64) -> TurbidityMeasurement {
        let num = 200usize;
        let dt = duration_s / num as f64;
        let mut readings = Vec::with_capacity(num);
        let mut timestamps = Vec::with_capacity(num);
        for i in 0..num {
            let t = i as f64 * dt;
            timestamps.push(t);
            readings.push(initial_ntu * (-rate * t).exp());
        }
        TurbidityMeasurement::new(readings, timestamps)
    }

    /// Simulate jar test dose-response (parabolic around optimal dose).
    pub fn simulate_jar_test(
        optimal_dose: f64,
        initial_ntu: f64,
        num_doses: usize,
    ) -> (Vec<f64>, Vec<f64>) {
        let dose_max = optimal_dose * 2.0;
        let mut doses = Vec::with_capacity(num_doses);
        let mut residuals = Vec::with_capacity(num_doses);
        for i in 0..num_doses {
            let dose = dose_max * i as f64 / (num_doses - 1).max(1) as f64;
            doses.push(dose);
            // Parabolic response: minimum at optimal_dose with value ~5% of initial.
            let frac = (dose - optimal_dose) / optimal_dose;
            let ntu = initial_ntu * (0.05 + 0.95 * frac * frac);
            residuals.push(ntu.max(0.0));
        }
        (doses, residuals)
    }

    /// Simulate filter breakthrough: steady low NTU then exponential rise.
    pub fn simulate_breakthrough(
        steady_ntu: f64,
        breakthrough_time: f64,
        rise_rate: f64,
    ) -> TurbidityMeasurement {
        let total_time = breakthrough_time * 2.0;
        let num = 300usize;
        let dt = total_time / num as f64;
        let mut readings = Vec::with_capacity(num);
        let mut timestamps = Vec::with_capacity(num);
        for i in 0..num {
            let t = i as f64 * dt;
            timestamps.push(t);
            if t < breakthrough_time {
                readings.push(steady_ntu);
            } else {
                let elapsed = t - breakthrough_time;
                readings.push(steady_ntu + steady_ntu * (rise_rate * elapsed).exp_m1());
            }
        }
        TurbidityMeasurement::new(readings, timestamps)
    }
}

// ---------------------------------------------------------------------------
// 10. MultiAngleDetector
// ---------------------------------------------------------------------------

/// Multi-angle light scattering analysis.
#[derive(Debug, Clone)]
pub struct MultiAngleDetector {
    pub angles_deg: Vec<f64>,
    pub intensities: Vec<f64>,
}

impl MultiAngleDetector {
    pub fn new(angles_deg: Vec<f64>, intensities: Vec<f64>) -> Self {
        assert_eq!(angles_deg.len(), intensities.len(), "length mismatch");
        Self { angles_deg, intensities }
    }

    /// Dissymmetry ratio z = I(45 deg) / I(135 deg).
    pub fn dissymmetry_ratio(i_45: f64, i_135: f64) -> f64 {
        if i_135 <= 0.0 {
            return f64::INFINITY;
        }
        i_45 / i_135
    }

    /// Approximate radius of gyration from dissymmetry ratio.
    /// Rg ≈ lambda / (2 * pi) * sqrt(3 * (z - 1) / (z + 1)) for z close to 1.
    pub fn gyration_radius_from_dissymmetry(z: f64, wavelength_nm: f64) -> f64 {
        if z <= 0.0 {
            return 0.0;
        }
        let arg = 3.0 * (z - 1.0) / (z + 1.0);
        if arg < 0.0 {
            return 0.0;
        }
        wavelength_nm / (2.0 * PI) * arg.sqrt()
    }

    /// Classify angular scattering pattern.
    /// Forward/back ratio > 3 => Large; > 1.3 => Mie; else Rayleigh.
    pub fn angular_pattern_classification(signals: &[(f64, f64)]) -> ScatteringPattern {
        // Separate forward (< 90) and backward (> 90) signals.
        let mut forward_sum = 0.0;
        let mut forward_count = 0;
        let mut backward_sum = 0.0;
        let mut backward_count = 0;
        for &(angle, intensity) in signals {
            if angle < 90.0 {
                forward_sum += intensity;
                forward_count += 1;
            } else if angle > 90.0 {
                backward_sum += intensity;
                backward_count += 1;
            }
        }
        if forward_count == 0 || backward_count == 0 {
            return ScatteringPattern::Mie; // insufficient data
        }
        let forward_avg = forward_sum / forward_count as f64;
        let backward_avg = backward_sum / backward_count as f64;
        if backward_avg <= 0.0 {
            return ScatteringPattern::Large;
        }
        let ratio = forward_avg / backward_avg;
        if ratio > 3.0 {
            ScatteringPattern::Large
        } else if ratio > 1.3 {
            ScatteringPattern::Mie
        } else {
            ScatteringPattern::Rayleigh
        }
    }
}

// ---------------------------------------------------------------------------
// Private helper functions
// ---------------------------------------------------------------------------

/// Simple linear slope via least-squares: slope = Σ(xi - x̄)(yi - ȳ) / Σ(xi - x̄)²
fn linear_slope(x: &[f64], y: &[f64]) -> f64 {
    let n = x.len() as f64;
    let x_mean = x.iter().sum::<f64>() / n;
    let y_mean = y.iter().sum::<f64>() / n;
    let num: f64 = x.iter().zip(y.iter()).map(|(&xi, &yi)| (xi - x_mean) * (yi - y_mean)).sum();
    let den: f64 = x.iter().map(|&xi| (xi - x_mean).powi(2)).sum();
    if den.abs() < 1e-30 {
        return 0.0;
    }
    num / den
}

/// Linear intercept = ȳ - slope * x̄.
fn linear_intercept(x: &[f64], y: &[f64]) -> f64 {
    let n = x.len() as f64;
    let x_mean = x.iter().sum::<f64>() / n;
    let y_mean = y.iter().sum::<f64>() / n;
    let slope = linear_slope(x, y);
    y_mean - slope * x_mean
}

/// Polynomial fit of degree `deg` via normal equations (Vandermonde).
fn poly_fit(x: &[f64], y: &[f64], deg: usize) -> Vec<f64> {
    let n = x.len();
    let m = deg + 1;
    // Build Vandermonde matrix X^T X and X^T y.
    let mut xtx = vec![0.0; m * m];
    let mut xty = vec![0.0; m];
    for i in 0..n {
        let mut xi_pow = 1.0;
        for j in 0..m {
            xty[j] += xi_pow * y[i];
            let mut xi_pow2 = 1.0;
            for k in 0..m {
                xtx[j * m + k] += xi_pow * xi_pow2;
                xi_pow2 *= x[i];
            }
            xi_pow *= x[i];
        }
    }
    // Solve via Gaussian elimination.
    gauss_solve(m, &mut xtx, &mut xty)
}

/// Solve A*x = b in-place via Gaussian elimination with partial pivoting.
fn gauss_solve(n: usize, a: &mut [f64], b: &mut [f64]) -> Vec<f64> {
    // Forward elimination.
    for col in 0..n {
        // Partial pivot.
        let mut max_row = col;
        let mut max_val = a[col * n + col].abs();
        for row in (col + 1)..n {
            let v = a[row * n + col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_row != col {
            for k in 0..n {
                a.swap(col * n + k, max_row * n + k);
            }
            b.swap(col, max_row);
        }
        let pivot = a[col * n + col];
        if pivot.abs() < 1e-30 {
            continue;
        }
        for row in (col + 1)..n {
            let factor = a[row * n + col] / pivot;
            for k in col..n {
                a[row * n + k] -= factor * a[col * n + k];
            }
            b[row] -= factor * b[col];
        }
    }
    // Back substitution.
    let mut x = vec![0.0; n];
    for col in (0..n).rev() {
        let pivot = a[col * n + col];
        if pivot.abs() < 1e-30 {
            continue;
        }
        let mut s = b[col];
        for k in (col + 1)..n {
            s -= a[col * n + k] * x[k];
        }
        x[col] = s / pivot;
    }
    x
}

/// Evaluate polynomial coefficients [a0, a1, a2, ...] at x.
fn poly_eval(coeffs: &[f64], x: f64) -> f64 {
    let mut result = 0.0;
    let mut xi = 1.0;
    for &c in coeffs {
        result += c * xi;
        xi *= x;
    }
    result
}

/// R-squared = 1 - SS_res / SS_tot.
fn r_squared(x: &[f64], y: &[f64], coeffs: &[f64]) -> f64 {
    let y_mean = y.iter().sum::<f64>() / y.len() as f64;
    let ss_tot: f64 = y.iter().map(|&yi| (yi - y_mean).powi(2)).sum();
    let ss_res: f64 = x
        .iter()
        .zip(y.iter())
        .map(|(&xi, &yi)| {
            let pred = poly_eval(coeffs, xi);
            (yi - pred).powi(2)
        })
        .sum();
    if ss_tot < 1e-30 {
        return 1.0;
    }
    1.0 - ss_res / ss_tot
}

/// Deterministic pseudo-Gaussian via Box-Muller with LCG.
fn pseudo_gaussian(seed: &mut u64) -> f64 {
    let u1 = lcg_uniform(seed);
    let u2 = lcg_uniform(seed);
    (-2.0 * u1.max(1e-15).ln()).sqrt() * (2.0 * PI * u2).cos()
}

/// LCG uniform [0, 1).
fn lcg_uniform(seed: &mut u64) -> f64 {
    *seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
    (*seed >> 33) as f64 / (1u64 << 31) as f64
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // TurbidityMeasurement tests
    // -----------------------------------------------------------------------
    #[test]
    fn test_mean_ntu() {
        let m = TurbidityMeasurement::new(vec![1.0, 2.0, 3.0, 4.0, 5.0], vec![0.0, 1.0, 2.0, 3.0, 4.0]);
        assert!((m.mean_ntu() - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_mean_ntu_empty() {
        let m = TurbidityMeasurement::new(vec![], vec![]);
        assert_eq!(m.mean_ntu(), 0.0);
    }

    #[test]
    fn test_median_ntu_odd() {
        let m = TurbidityMeasurement::new(vec![5.0, 1.0, 3.0], vec![0.0, 1.0, 2.0]);
        assert!((m.median_ntu() - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_median_ntu_even() {
        let m = TurbidityMeasurement::new(vec![1.0, 2.0, 3.0, 4.0], vec![0.0, 1.0, 2.0, 3.0]);
        assert!((m.median_ntu() - 2.5).abs() < 1e-10);
    }

    #[test]
    fn test_std_deviation() {
        let m = TurbidityMeasurement::new(vec![2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0], (0..8).map(|i| i as f64).collect());
        // Mean = 5.0, SS = 32, sample variance = 32/7, sample std dev = sqrt(32/7) ≈ 2.138
        let expected = (32.0_f64 / 7.0).sqrt();
        assert!((m.std_deviation() - expected).abs() < 0.01);
    }

    #[test]
    fn test_std_deviation_single() {
        let m = TurbidityMeasurement::new(vec![5.0], vec![0.0]);
        assert_eq!(m.std_deviation(), 0.0);
    }

    #[test]
    fn test_max_min_ntu() {
        let m = TurbidityMeasurement::new(vec![3.0, 1.0, 5.0, 2.0], vec![0.0, 1.0, 2.0, 3.0]);
        assert!((m.max_ntu() - 5.0).abs() < 1e-10);
        assert!((m.min_ntu() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_classify_drinking() {
        assert_eq!(TurbidityMeasurement::classify_water_quality(0.5), WaterQuality::Drinking);
    }

    #[test]
    fn test_classify_acceptable() {
        assert_eq!(TurbidityMeasurement::classify_water_quality(3.0), WaterQuality::Acceptable);
    }

    #[test]
    fn test_classify_marginal() {
        assert_eq!(TurbidityMeasurement::classify_water_quality(25.0), WaterQuality::Marginal);
    }

    #[test]
    fn test_classify_poor() {
        assert_eq!(TurbidityMeasurement::classify_water_quality(100.0), WaterQuality::Poor);
    }

    #[test]
    fn test_classify_boundary_1() {
        assert_eq!(TurbidityMeasurement::classify_water_quality(0.999), WaterQuality::Drinking);
        assert_eq!(TurbidityMeasurement::classify_water_quality(1.0), WaterQuality::Acceptable);
    }

    #[test]
    fn test_classify_boundary_5() {
        assert_eq!(TurbidityMeasurement::classify_water_quality(4.999), WaterQuality::Acceptable);
        assert_eq!(TurbidityMeasurement::classify_water_quality(5.0), WaterQuality::Marginal);
    }

    // -----------------------------------------------------------------------
    // CalibrationCurve tests
    // -----------------------------------------------------------------------
    #[test]
    fn test_calibration_linear() {
        let mut cal = CalibrationCurve::new(
            vec![0.0, 10.0, 20.0, 30.0, 40.0],
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
        );
        let result = cal.calibrate();
        assert!(result.r_squared > 0.999);
        // signal=2.5 should give NTU ~25
        let ntu = cal.ntu_from_signal(2.5);
        assert!((ntu - 25.0).abs() < 0.5);
    }

    #[test]
    fn test_calibration_r_squared() {
        let mut cal = CalibrationCurve::new(
            vec![0.0, 10.0, 20.0, 30.0],
            vec![0.0, 1.0, 2.0, 3.0],
        );
        let result = cal.calibrate();
        assert!(result.r_squared > 0.99);
    }

    #[test]
    fn test_linearity_check() {
        let cal = CalibrationCurve::new(
            vec![0.0, 10.0, 20.0, 30.0],
            vec![0.0, 1.0, 2.0, 3.0],
        );
        let r2 = cal.linearity_check();
        assert!(r2 > 0.999);
    }

    #[test]
    fn test_formazin_preparation() {
        let recipe = CalibrationCurve::formazin_preparation(100.0, 1000.0);
        assert!((recipe.stock_volume_ml - 25.0).abs() < 0.01);
        assert!((recipe.dilution_water_ml - 975.0).abs() < 0.01);
        assert!((recipe.target_ntu - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_formazin_4000_ntu() {
        let recipe = CalibrationCurve::formazin_preparation(4000.0, 1000.0);
        assert!((recipe.stock_volume_ml - 1000.0).abs() < 0.01);
        assert!((recipe.dilution_water_ml).abs() < 0.01);
    }

    // -----------------------------------------------------------------------
    // NephelometricRatio tests
    // -----------------------------------------------------------------------
    #[test]
    fn test_ratio_mode() {
        let ratio = NephelometricRatio::ratio_mode(50.0, 100.0);
        assert!((ratio - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_ratio_mode_zero_transmitted() {
        assert!(NephelometricRatio::ratio_mode(50.0, 0.0).is_infinite());
    }

    #[test]
    fn test_backscatter_ratio() {
        let ratio = NephelometricRatio::backscatter_ratio(100.0, 50.0);
        assert!((ratio - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_forward_scatter_correction() {
        let corrected = NephelometricRatio::forward_scatter_correction(10.0, 4.0);
        assert!((corrected - 8.0).abs() < 1e-10);
    }

    #[test]
    fn test_forward_scatter_correction_clamp() {
        let corrected = NephelometricRatio::forward_scatter_correction(1.0, 10.0);
        assert_eq!(corrected, 0.0);
    }

    // -----------------------------------------------------------------------
    // BeerLambertTurbidity tests
    // -----------------------------------------------------------------------
    #[test]
    fn test_turbidity_from_transmittance() {
        // T = 0.5, path = 1.0 cm => tau = -ln(0.5)/1 = ln(2)
        let tau = BeerLambertTurbidity::turbidity_from_transmittance(100.0, 50.0, 1.0);
        assert!((tau - LN_2).abs() < 1e-10);
    }

    #[test]
    fn test_turbidity_full_transmission() {
        let tau = BeerLambertTurbidity::turbidity_from_transmittance(100.0, 100.0, 1.0);
        assert!(tau.abs() < 1e-10);
    }

    #[test]
    fn test_optical_density() {
        // T = 0.01 => OD = 2.0
        let od = BeerLambertTurbidity::optical_density(0.01);
        assert!((od - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_optical_density_zero() {
        assert!(BeerLambertTurbidity::optical_density(0.0).is_infinite());
    }

    #[test]
    fn test_ntu_from_attenuation() {
        let ntu = BeerLambertTurbidity::ntu_from_attenuation(0.5, 200.0);
        assert!((ntu - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_specific_turbidity() {
        let st = BeerLambertTurbidity::specific_turbidity(50.0, 25.0);
        assert!((st - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_specific_turbidity_zero_conc() {
        assert_eq!(BeerLambertTurbidity::specific_turbidity(50.0, 0.0), 0.0);
    }

    // -----------------------------------------------------------------------
    // ParticleSizeEstimator tests
    // -----------------------------------------------------------------------
    #[test]
    fn test_rayleigh_regime_true() {
        assert!(ParticleSizeEstimator::rayleigh_regime(633.0, 50.0));
    }

    #[test]
    fn test_rayleigh_regime_false() {
        assert!(!ParticleSizeEstimator::rayleigh_regime(633.0, 200.0));
    }

    #[test]
    fn test_rayleigh_scattering_intensity() {
        let i = ParticleSizeEstimator::rayleigh_scattering_intensity(50.0, 633.0, 1.5, 1.333);
        assert!(i > 0.0);
    }

    #[test]
    fn test_rayleigh_scattering_proportional_d6() {
        let i1 = ParticleSizeEstimator::rayleigh_scattering_intensity(50.0, 633.0, 1.5, 1.333);
        let i2 = ParticleSizeEstimator::rayleigh_scattering_intensity(100.0, 633.0, 1.5, 1.333);
        // Doubling size => 2^6 = 64x intensity.
        let ratio = i2 / i1;
        assert!((ratio - 64.0).abs() < 0.1);
    }

    #[test]
    fn test_rayleigh_scattering_proportional_lambda_inv4() {
        let i1 = ParticleSizeEstimator::rayleigh_scattering_intensity(50.0, 400.0, 1.5, 1.333);
        let i2 = ParticleSizeEstimator::rayleigh_scattering_intensity(50.0, 800.0, 1.5, 1.333);
        // Doubling wavelength => 1/16 intensity.
        let ratio = i2 / i1;
        assert!((ratio - 1.0 / 16.0).abs() < 0.001);
    }

    #[test]
    fn test_mie_parameter() {
        let x = ParticleSizeEstimator::mie_parameter(633.0, 633.0);
        assert!((x - PI).abs() < 1e-10);
    }

    #[test]
    fn test_turbidity_ratio() {
        assert!((ParticleSizeEstimator::turbidity_ratio(10.0, 5.0) - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_approximate_size_from_ratio() {
        let size = ParticleSizeEstimator::approximate_size_from_ratio(1.0);
        assert!((size - 200.0).abs() < 1e-10);
    }

    #[test]
    fn test_approximate_size_ratio_larger_means_smaller() {
        let s1 = ParticleSizeEstimator::approximate_size_from_ratio(1.0);
        let s2 = ParticleSizeEstimator::approximate_size_from_ratio(2.0);
        assert!(s2 < s1); // higher ratio => smaller particles
    }

    // -----------------------------------------------------------------------
    // KineticsTurbidity tests
    // -----------------------------------------------------------------------
    #[test]
    fn test_aggregation_rate_linear() {
        let t: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let v: Vec<f64> = t.iter().map(|&ti| 2.0 * ti + 1.0).collect();
        let rate = KineticsTurbidity::aggregation_rate(&t, &v);
        assert!((rate - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_settling_rate() {
        let sim = TurbiditySimulator::simulate_settling(100.0, 0.1, 50.0);
        let k = KineticsTurbidity::settling_rate(&sim.timestamps_s, &sim.readings_ntu);
        assert!((k - 0.1).abs() < 0.05, "settling rate k={k}, expected ~0.1");
    }

    #[test]
    fn test_half_life() {
        let t_half = KineticsTurbidity::half_life(0.1);
        assert!((t_half - LN_2 / 0.1).abs() < 1e-10);
    }

    #[test]
    fn test_half_life_zero_rate() {
        assert!(KineticsTurbidity::half_life(0.0).is_infinite());
    }

    #[test]
    fn test_fit_exponential_decay() {
        let times: Vec<f64> = (0..100).map(|i| i as f64 * 0.5).collect();
        let values: Vec<f64> = times.iter().map(|&t| 50.0 * (-0.05 * t).exp() + 2.0).collect();
        let (a, k, offset) = KineticsTurbidity::fit_exponential_decay(&times, &values);
        assert!((a - 50.0).abs() < 15.0, "A={a}, expected ~50");
        assert!((k - 0.05).abs() < 0.02, "k={k}, expected ~0.05");
        assert!((offset - 2.0).abs() < 2.0, "offset={offset}, expected ~2.0");
    }

    #[test]
    fn test_induction_time() {
        let n = 100;
        let mut times: Vec<f64> = (0..n).map(|i| i as f64).collect();
        let mut values: Vec<f64> = vec![1.0; n];
        // Introduce a rise at t=50.
        for i in 50..n {
            values[i] = 1.0 + (i - 50) as f64 * 2.0;
        }
        let t_ind = KineticsTurbidity::induction_time(&times, &values);
        assert!(t_ind >= 49.0 && t_ind <= 55.0);
    }

    // -----------------------------------------------------------------------
    // WaterTreatmentMonitor tests
    // -----------------------------------------------------------------------
    #[test]
    fn test_jar_test_analysis() {
        let doses = vec![0.0, 10.0, 20.0, 30.0, 40.0, 50.0];
        let residuals = vec![100.0, 40.0, 10.0, 5.0, 8.0, 20.0];
        let result = WaterTreatmentMonitor::jar_test_analysis(&doses, &residuals);
        assert!((result.optimal_dose - 30.0).abs() < 1e-10);
        assert!((result.minimum_ntu - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_filter_performance() {
        let lrv = WaterTreatmentMonitor::filter_performance(100.0, 1.0);
        assert!((lrv - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_filter_performance_zero() {
        assert_eq!(WaterTreatmentMonitor::filter_performance(100.0, 0.0), 0.0);
    }

    #[test]
    fn test_breakthrough_detection_found() {
        let series = vec![0.1, 0.1, 0.1, 0.5, 1.0, 2.0];
        assert_eq!(WaterTreatmentMonitor::breakthrough_detection(&series, 0.3), Some(3));
    }

    #[test]
    fn test_breakthrough_detection_none() {
        let series = vec![0.1, 0.1, 0.1, 0.2];
        assert_eq!(WaterTreatmentMonitor::breakthrough_detection(&series, 0.3), None);
    }

    #[test]
    fn test_regulatory_who() {
        assert!(WaterTreatmentMonitor::regulatory_compliance(0.5, RegulatoryStandard::Who));
        assert!(!WaterTreatmentMonitor::regulatory_compliance(1.5, RegulatoryStandard::Who));
    }

    #[test]
    fn test_regulatory_epa_filtered() {
        assert!(WaterTreatmentMonitor::regulatory_compliance(0.2, RegulatoryStandard::EpaFiltered));
        assert!(!WaterTreatmentMonitor::regulatory_compliance(0.5, RegulatoryStandard::EpaFiltered));
    }

    #[test]
    fn test_regulatory_epa_unfiltered() {
        assert!(WaterTreatmentMonitor::regulatory_compliance(4.0, RegulatoryStandard::EpaUnfiltered));
        assert!(!WaterTreatmentMonitor::regulatory_compliance(6.0, RegulatoryStandard::EpaUnfiltered));
    }

    #[test]
    fn test_regulatory_eu() {
        assert!(WaterTreatmentMonitor::regulatory_compliance(0.8, RegulatoryStandard::EuDrinkingWater));
        assert!(!WaterTreatmentMonitor::regulatory_compliance(1.5, RegulatoryStandard::EuDrinkingWater));
    }

    // -----------------------------------------------------------------------
    // ConcentrationEstimator tests
    // -----------------------------------------------------------------------
    #[test]
    fn test_tss_from_ntu() {
        let tss = ConcentrationEstimator::tss_from_ntu(10.0, 2.5, 1.0);
        assert!((tss - 26.0).abs() < 1e-10);
    }

    #[test]
    fn test_calibrate_tss_ntu() {
        let ntu = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let tss = vec![2.5, 5.0, 7.5, 10.0, 12.5];
        let (slope, intercept, r2) = ConcentrationEstimator::calibrate_tss_ntu(&ntu, &tss);
        assert!((slope - 2.5).abs() < 0.01);
        assert!(intercept.abs() < 0.1);
        assert!(r2 > 0.999);
    }

    #[test]
    fn test_secchi_depth() {
        // NTU = 10 => SD ≈ 244 / 10^0.662 ≈ 53.2 m
        let sd = ConcentrationEstimator::secchi_depth_from_ntu(10.0);
        assert!(sd > 40.0 && sd < 70.0);
    }

    #[test]
    fn test_secchi_depth_zero() {
        assert!(ConcentrationEstimator::secchi_depth_from_ntu(0.0).is_infinite());
    }

    #[test]
    fn test_chlorophyll_contribution() {
        let ntu = ConcentrationEstimator::chlorophyll_contribution(50.0);
        assert!((ntu - 3.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // TurbiditySimulator tests
    // -----------------------------------------------------------------------
    #[test]
    fn test_simulate_steady_state() {
        let m = TurbiditySimulator::simulate_steady_state(10.0, 0.5, 1000);
        assert_eq!(m.readings_ntu.len(), 1000);
        assert!((m.mean_ntu() - 10.0).abs() < 1.0);
    }

    #[test]
    fn test_simulate_settling() {
        let m = TurbiditySimulator::simulate_settling(100.0, 0.1, 50.0);
        assert!(m.readings_ntu[0] > m.readings_ntu[m.readings_ntu.len() - 1]);
        assert!((m.readings_ntu[0] - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_simulate_jar_test() {
        let (doses, residuals) = TurbiditySimulator::simulate_jar_test(20.0, 100.0, 11);
        assert_eq!(doses.len(), 11);
        assert_eq!(residuals.len(), 11);
        // The minimum residual should be near the optimal dose.
        let min_idx = residuals
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap()
            .0;
        assert!((doses[min_idx] - 20.0).abs() < 5.0);
    }

    #[test]
    fn test_simulate_breakthrough() {
        let m = TurbiditySimulator::simulate_breakthrough(0.1, 50.0, 0.1);
        // Before breakthrough should be ~0.1.
        assert!((m.readings_ntu[0] - 0.1).abs() < 0.01);
        // After breakthrough should be much higher.
        let last = *m.readings_ntu.last().unwrap();
        assert!(last > 0.5);
    }

    // -----------------------------------------------------------------------
    // MultiAngleDetector tests
    // -----------------------------------------------------------------------
    #[test]
    fn test_dissymmetry_ratio() {
        assert!((MultiAngleDetector::dissymmetry_ratio(100.0, 50.0) - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_dissymmetry_ratio_symmetric() {
        assert!((MultiAngleDetector::dissymmetry_ratio(100.0, 100.0) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_gyration_radius_isotropic() {
        // z = 1 (symmetric) => Rg = 0.
        let rg = MultiAngleDetector::gyration_radius_from_dissymmetry(1.0, 633.0);
        assert!(rg.abs() < 1e-10);
    }

    #[test]
    fn test_gyration_radius_positive() {
        let rg = MultiAngleDetector::gyration_radius_from_dissymmetry(2.0, 633.0);
        assert!(rg > 0.0);
    }

    #[test]
    fn test_angular_classification_rayleigh() {
        let signals = vec![(30.0, 100.0), (60.0, 100.0), (120.0, 95.0), (150.0, 95.0)];
        assert_eq!(MultiAngleDetector::angular_pattern_classification(&signals), ScatteringPattern::Rayleigh);
    }

    #[test]
    fn test_angular_classification_mie() {
        let signals = vec![(30.0, 200.0), (60.0, 150.0), (120.0, 80.0), (150.0, 70.0)];
        assert_eq!(MultiAngleDetector::angular_pattern_classification(&signals), ScatteringPattern::Mie);
    }

    #[test]
    fn test_angular_classification_large() {
        let signals = vec![(30.0, 500.0), (60.0, 400.0), (120.0, 50.0), (150.0, 40.0)];
        assert_eq!(MultiAngleDetector::angular_pattern_classification(&signals), ScatteringPattern::Large);
    }

    // -----------------------------------------------------------------------
    // Helper function tests
    // -----------------------------------------------------------------------
    #[test]
    fn test_linear_slope_positive() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y = vec![1.0, 3.0, 5.0, 7.0, 9.0];
        assert!((linear_slope(&x, &y) - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_linear_intercept_value() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![5.0, 7.0, 9.0, 11.0];
        assert!((linear_intercept(&x, &y) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_poly_eval_constant() {
        assert!((poly_eval(&[3.0], 100.0) - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_poly_eval_linear() {
        // 2 + 3x at x=4 => 14
        assert!((poly_eval(&[2.0, 3.0], 4.0) - 14.0).abs() < 1e-10);
    }

    #[test]
    fn test_poly_eval_quadratic() {
        // 1 + 0*x + 2*x^2 at x=3 => 19
        assert!((poly_eval(&[1.0, 0.0, 2.0], 3.0) - 19.0).abs() < 1e-10);
    }

    #[test]
    fn test_poly_fit_linear() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![1.0, 3.0, 5.0, 7.0];
        let coeffs = poly_fit(&x, &y, 1);
        assert!((coeffs[0] - 1.0).abs() < 1e-6); // intercept
        assert!((coeffs[1] - 2.0).abs() < 1e-6); // slope
    }

    #[test]
    fn test_r_squared_perfect() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![1.0, 3.0, 5.0, 7.0];
        let coeffs = vec![1.0, 2.0];
        let r2 = r_squared(&x, &y, &coeffs);
        assert!((r2 - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_lcg_uniform_range() {
        let mut seed = 12345u64;
        for _ in 0..100 {
            let u = lcg_uniform(&mut seed);
            assert!(u >= 0.0 && u < 1.0);
        }
    }

    #[test]
    fn test_pseudo_gaussian_distribution() {
        let mut seed = 42u64;
        let n = 10_000;
        let mut sum = 0.0;
        let mut sum2 = 0.0;
        for _ in 0..n {
            let g = pseudo_gaussian(&mut seed);
            sum += g;
            sum2 += g * g;
        }
        let mean = sum / n as f64;
        let var = sum2 / n as f64 - mean * mean;
        // Should be ~N(0,1).
        assert!(mean.abs() < 0.1);
        assert!((var - 1.0).abs() < 0.2);
    }

    // -----------------------------------------------------------------------
    // Integration / cross-component tests
    // -----------------------------------------------------------------------
    #[test]
    fn test_jar_test_roundtrip() {
        let (doses, residuals) = TurbiditySimulator::simulate_jar_test(25.0, 80.0, 21);
        let result = WaterTreatmentMonitor::jar_test_analysis(&doses, &residuals);
        assert!((result.optimal_dose - 25.0).abs() < 5.0);
        assert!(result.minimum_ntu < 10.0);
    }

    #[test]
    fn test_settling_kinetics_roundtrip() {
        let sim = TurbiditySimulator::simulate_settling(200.0, 0.05, 100.0);
        let (a, k, _offset) = KineticsTurbidity::fit_exponential_decay(&sim.timestamps_s, &sim.readings_ntu);
        assert!((a - 200.0).abs() < 60.0, "A={a}, expected ~200");
        assert!((k - 0.05).abs() < 0.03, "k={k}, expected ~0.05");
    }

    #[test]
    fn test_beer_lambert_symmetry() {
        // tau(T=0.5) = ln(2), and OD(0.5) = log10(2)
        let tau = BeerLambertTurbidity::turbidity_from_transmittance(1.0, 0.5, 1.0);
        let od = BeerLambertTurbidity::optical_density(0.5);
        // tau / OD = ln(10)
        assert!((tau / od - 10.0_f64.ln()).abs() < 1e-10);
    }

    #[test]
    fn test_calibration_ntu_roundtrip() {
        let signals = vec![0.0, 0.5, 1.0, 1.5, 2.0];
        let standards = vec![0.0, 25.0, 50.0, 75.0, 100.0];
        let mut cal = CalibrationCurve::new(standards, signals);
        cal.calibrate();
        for i in 0..5 {
            let sig = i as f64 * 0.5;
            let expected_ntu = i as f64 * 25.0;
            let got = cal.ntu_from_signal(sig);
            assert!((got - expected_ntu).abs() < 1.0, "signal={sig}, expected={expected_ntu}, got={got}");
        }
    }

    #[test]
    fn test_breakthrough_with_simulator() {
        let m = TurbiditySimulator::simulate_breakthrough(0.1, 100.0, 0.05);
        let bt = WaterTreatmentMonitor::breakthrough_detection(&m.readings_ntu, 0.3);
        assert!(bt.is_some());
        let idx = bt.unwrap();
        // Index should be roughly at the midpoint.
        assert!(idx > 50 && idx < 250);
    }

    #[test]
    fn test_multi_angle_constructor() {
        let det = MultiAngleDetector::new(vec![30.0, 90.0, 150.0], vec![100.0, 80.0, 60.0]);
        assert_eq!(det.angles_deg.len(), 3);
    }

    #[test]
    fn test_concentration_roundtrip() {
        let ntu_vals = vec![5.0, 10.0, 15.0, 20.0, 25.0];
        let tss_vals = vec![10.0, 20.0, 30.0, 40.0, 50.0];
        let (slope, intercept, r2) = ConcentrationEstimator::calibrate_tss_ntu(&ntu_vals, &tss_vals);
        assert!(r2 > 0.999);
        let tss = ConcentrationEstimator::tss_from_ntu(12.0, slope, intercept);
        assert!((tss - 24.0).abs() < 1.0);
    }
}
