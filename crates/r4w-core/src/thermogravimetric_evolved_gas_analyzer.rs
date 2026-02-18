// Thermogravimetric Evolved Gas Analyzer (TGA-EGA)
//
// Coupled thermal analysis and evolved gas detection:
// - TGA weight loss (mass% vs temperature)
// - Derivative thermogravimetry (DTG = dm/dT)
// - Onset/endset temperature determination
// - Weight loss step decomposition
// - Gas evolution rate from weight loss
// - Kissinger kinetic analysis (activation energy from multiple heating rates)
// - Ozawa-Flynn-Wall isoconversional method
// - Buoyancy/drag correction
// - Residual mass calculation
// - TGA-MS correlation (m/z intensity vs temperature)

use std::f64::consts::PI;

/// Single TGA data point
#[derive(Debug, Clone, Copy)]
pub struct TgaPoint {
    pub temperature_c: f64,
    pub mass_mg: f64,
    pub time_s: f64,
}

/// DTG result (derivative thermogravimetry)
#[derive(Debug, Clone)]
pub struct DtgResult {
    pub temperatures: Vec<f64>,
    pub dm_dt: Vec<f64>,        // %/min (rate of mass change)
    pub dm_d_temp: Vec<f64>,    // %/°C (mass change per degree)
}

/// Compute mass percent from raw TGA data
pub fn mass_percent(data: &[TgaPoint]) -> Vec<f64> {
    if data.is_empty() {
        return Vec::new();
    }
    let m0 = data[0].mass_mg;
    if m0 <= 0.0 {
        return vec![100.0; data.len()];
    }
    data.iter().map(|p| 100.0 * p.mass_mg / m0).collect()
}

/// Derivative thermogravimetry: dm/dT (%/°C) and dm/dt (%/min)
pub fn derivative_tga(data: &[TgaPoint]) -> DtgResult {
    let n = data.len();
    if n < 3 {
        return DtgResult {
            temperatures: data.iter().map(|p| p.temperature_c).collect(),
            dm_dt: vec![0.0; n],
            dm_d_temp: vec![0.0; n],
        };
    }

    let m0 = data[0].mass_mg;
    let mut temps = Vec::with_capacity(n);
    let mut dm_dt = Vec::with_capacity(n);
    let mut dm_d_temp = Vec::with_capacity(n);

    for i in 0..n {
        temps.push(data[i].temperature_c);
        if i == 0 {
            let dt = data[1].time_s - data[0].time_s;
            let d_temp = data[1].temperature_c - data[0].temperature_c;
            let dm_pct = 100.0 * (data[1].mass_mg - data[0].mass_mg) / m0;
            dm_dt.push(if dt > 0.0 { dm_pct / (dt / 60.0) } else { 0.0 });
            dm_d_temp.push(if d_temp.abs() > 1e-12 { dm_pct / d_temp } else { 0.0 });
        } else if i == n - 1 {
            let dt = data[n - 1].time_s - data[n - 2].time_s;
            let d_temp = data[n - 1].temperature_c - data[n - 2].temperature_c;
            let dm_pct = 100.0 * (data[n - 1].mass_mg - data[n - 2].mass_mg) / m0;
            dm_dt.push(if dt > 0.0 { dm_pct / (dt / 60.0) } else { 0.0 });
            dm_d_temp.push(if d_temp.abs() > 1e-12 { dm_pct / d_temp } else { 0.0 });
        } else {
            // Central difference
            let dt = data[i + 1].time_s - data[i - 1].time_s;
            let d_temp = data[i + 1].temperature_c - data[i - 1].temperature_c;
            let dm_pct = 100.0 * (data[i + 1].mass_mg - data[i - 1].mass_mg) / m0;
            dm_dt.push(if dt > 0.0 { dm_pct / (dt / 60.0) } else { 0.0 });
            dm_d_temp.push(if d_temp.abs() > 1e-12 { dm_pct / d_temp } else { 0.0 });
        }
    }

    DtgResult { temperatures: temps, dm_dt, dm_d_temp }
}

/// Find onset temperature: tangent intersection method
/// Returns (onset_temp, endset_temp) for the steepest mass loss step
pub fn onset_endset_temperature(
    temperatures: &[f64],
    mass_pct: &[f64],
) -> (f64, f64) {
    let n = temperatures.len().min(mass_pct.len());
    if n < 5 {
        return (0.0, 0.0);
    }

    // Find steepest slope point (maximum |dm/dT|)
    let mut max_slope = 0.0_f64;
    let mut max_idx = 1;
    for i in 1..n - 1 {
        let dt = temperatures[i + 1] - temperatures[i - 1];
        if dt.abs() < 1e-12 {
            continue;
        }
        let slope = (mass_pct[i + 1] - mass_pct[i - 1]) / dt;
        if slope.abs() > max_slope.abs() {
            max_slope = slope;
            max_idx = i;
        }
    }

    // Tangent line at steepest point: y = mass_pct[max_idx] + max_slope*(T - T_max_idx)
    let t_steep = temperatures[max_idx];
    let m_steep = mass_pct[max_idx];

    // Baseline before step (average of first 10% of points)
    let n_base = (n / 10).max(3).min(n);
    let baseline_start: f64 = mass_pct[..n_base].iter().sum::<f64>() / n_base as f64;

    // Baseline after step (average of last 10% of points)
    let n_end = (n / 10).max(3).min(n);
    let baseline_end: f64 = mass_pct[n - n_end..].iter().sum::<f64>() / n_end as f64;

    // Onset: intersection of tangent with pre-step baseline
    let onset = if max_slope.abs() > 1e-15 {
        t_steep + (baseline_start - m_steep) / max_slope
    } else {
        temperatures[0]
    };

    // Endset: intersection of tangent with post-step baseline
    let endset = if max_slope.abs() > 1e-15 {
        t_steep + (baseline_end - m_steep) / max_slope
    } else {
        temperatures[n - 1]
    };

    (onset, endset)
}

/// Weight loss step: mass loss between two temperatures
pub fn weight_loss_step(
    temperatures: &[f64],
    mass_pct: &[f64],
    t_start: f64,
    t_end: f64,
) -> f64 {
    let n = temperatures.len().min(mass_pct.len());
    if n < 2 {
        return 0.0;
    }

    let m_start = interpolate_at_temp(temperatures, mass_pct, t_start);
    let m_end = interpolate_at_temp(temperatures, mass_pct, t_end);
    m_start - m_end
}

/// Linear interpolation at a given temperature
fn interpolate_at_temp(temperatures: &[f64], values: &[f64], t: f64) -> f64 {
    let n = temperatures.len().min(values.len());
    if n == 0 {
        return 0.0;
    }
    if t <= temperatures[0] {
        return values[0];
    }
    if t >= temperatures[n - 1] {
        return values[n - 1];
    }
    for i in 1..n {
        if temperatures[i] >= t {
            let frac = (t - temperatures[i - 1]) / (temperatures[i] - temperatures[i - 1]);
            return values[i - 1] + frac * (values[i] - values[i - 1]);
        }
    }
    values[n - 1]
}

/// Residual mass (mass remaining at final temperature)
pub fn residual_mass_pct(data: &[TgaPoint]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    let m0 = data[0].mass_mg;
    if m0 <= 0.0 {
        return 0.0;
    }
    100.0 * data.last().unwrap().mass_mg / m0
}

/// Conversion fraction alpha(T) = (m0 - m(T)) / (m0 - mf)
pub fn conversion_fraction(data: &[TgaPoint]) -> Vec<f64> {
    if data.len() < 2 {
        return vec![0.0; data.len()];
    }
    let m0 = data[0].mass_mg;
    let mf = data.last().unwrap().mass_mg;
    let dm = m0 - mf;
    if dm.abs() < 1e-15 {
        return vec![0.0; data.len()];
    }
    data.iter().map(|p| ((m0 - p.mass_mg) / dm).clamp(0.0, 1.0)).collect()
}

/// Kissinger method: determine activation energy from multiple heating rates
/// For each heating rate beta, find the temperature Tp of the DTG peak.
/// Plot ln(beta/Tp^2) vs 1/Tp: slope = -Ea/R
///
/// Returns (activation_energy_kj_mol, pre_exponential_ln_a)
pub fn kissinger_activation_energy(
    heating_rates_k_per_min: &[f64],  // beta values
    peak_temperatures_k: &[f64],       // Tp values in Kelvin
) -> (f64, f64) {
    let n = heating_rates_k_per_min.len().min(peak_temperatures_k.len());
    if n < 2 {
        return (0.0, 0.0);
    }

    let r = 8.314; // J/(mol·K)

    // y = ln(beta/Tp^2), x = 1/Tp
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut sum_xx = 0.0;
    let mut sum_xy = 0.0;

    for i in 0..n {
        let tp = peak_temperatures_k[i];
        let beta = heating_rates_k_per_min[i];
        if tp <= 0.0 || beta <= 0.0 {
            continue;
        }
        let x = 1.0 / tp;
        let y = (beta / (tp * tp)).ln();
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
    }

    let nf = n as f64;
    let denom = nf * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return (0.0, 0.0);
    }

    let slope = (nf * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / nf;

    let ea_j = -slope * r;
    let ea_kj = ea_j / 1000.0;

    // Pre-exponential factor: intercept = ln(A*R/Ea) + constant
    // ln(A) ≈ intercept + ln(Ea/R) (approximate)
    let ln_a = intercept + (ea_j / r).ln();

    (ea_kj, ln_a)
}

/// Ozawa-Flynn-Wall isoconversional method
/// At fixed conversion alpha, plot ln(beta) vs 1/T
/// Slope = -1.052 * Ea/R
///
/// Returns activation energy (kJ/mol) at the specified conversion
pub fn ozawa_flynn_wall(
    heating_rates_k_per_min: &[f64],
    temperatures_at_alpha_k: &[f64], // T at which alpha is reached for each beta
) -> f64 {
    let n = heating_rates_k_per_min.len().min(temperatures_at_alpha_k.len());
    if n < 2 {
        return 0.0;
    }

    let r = 8.314;

    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut sum_xx = 0.0;
    let mut sum_xy = 0.0;

    for i in 0..n {
        let t = temperatures_at_alpha_k[i];
        let beta = heating_rates_k_per_min[i];
        if t <= 0.0 || beta <= 0.0 {
            continue;
        }
        let x = 1.0 / t;
        let y = beta.ln();
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
    }

    let nf = n as f64;
    let denom = nf * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return 0.0;
    }

    let slope = (nf * sum_xy - sum_x * sum_y) / denom;
    // slope = -1.052 * Ea / R
    let ea_j = -slope * r / 1.052;
    ea_j / 1000.0
}

/// Gas evolution rate from TGA weight loss rate
/// Assumes evolved gas with known molar mass
/// rate (mol/min) = |dm/dt| (mg/min) / (molar_mass * 1000)
pub fn gas_evolution_rate_mol_per_min(
    dm_dt_mg_per_min: f64,
    molar_mass_g_per_mol: f64,
) -> f64 {
    if molar_mass_g_per_mol <= 0.0 {
        return 0.0;
    }
    dm_dt_mg_per_min.abs() / (molar_mass_g_per_mol * 1000.0)
}

/// Buoyancy correction for TGA
/// Apparent mass change due to gas density variation with temperature
/// dm_buoy = -V_sample * rho_gas(T) where rho_gas ∝ P/(RT)
pub fn buoyancy_correction(
    mass_mg: &[f64],
    temperatures_k: &[f64],
    sample_volume_cm3: f64,
    gas_molar_mass: f64,    // g/mol (e.g., 28.97 for air)
    pressure_atm: f64,
) -> Vec<f64> {
    let n = mass_mg.len().min(temperatures_k.len());
    let r = 82.057; // cm³·atm/(mol·K)

    let mut corrected = Vec::with_capacity(n);
    for i in 0..n {
        let t = temperatures_k[i];
        if t <= 0.0 {
            corrected.push(mass_mg[i]);
            continue;
        }
        // Gas density at T
        let rho = gas_molar_mass * pressure_atm / (r * t); // g/cm³
        // Gas density at reference (first point)
        let t_ref = temperatures_k[0];
        let rho_ref = gas_molar_mass * pressure_atm / (r * t_ref);
        // Buoyancy correction = V * (rho_ref - rho) in mg
        let correction = sample_volume_cm3 * (rho_ref - rho) * 1000.0;
        corrected.push(mass_mg[i] + correction);
    }
    corrected
}

/// Find DTG peak temperatures (local minima in dm/dT, since mass loss gives negative dm/dT)
pub fn find_dtg_peaks(
    temperatures: &[f64],
    dm_d_temp: &[f64],
) -> Vec<(f64, f64)> {
    let n = temperatures.len().min(dm_d_temp.len());
    let mut peaks = Vec::new();
    for i in 1..n.saturating_sub(1) {
        // Looking for minima (most negative dm/dT = fastest mass loss)
        if dm_d_temp[i] < dm_d_temp[i - 1] && dm_d_temp[i] < dm_d_temp[i + 1] {
            peaks.push((temperatures[i], dm_d_temp[i]));
        }
    }
    peaks
}

/// Decompose weight loss into steps using DTG valleys
/// Returns: Vec<(start_temp, end_temp, mass_loss_pct)>
pub fn decompose_steps(
    temperatures: &[f64],
    mass_pct: &[f64],
    dm_d_temp: &[f64],
    threshold: f64, // minimum |dm/dT| to consider as active decomposition
) -> Vec<(f64, f64, f64)> {
    let n = temperatures.len().min(mass_pct.len()).min(dm_d_temp.len());
    if n < 3 {
        return Vec::new();
    }

    let mut steps = Vec::new();
    let mut in_step = false;
    let mut step_start = 0;

    for i in 0..n {
        if dm_d_temp[i].abs() > threshold && !in_step {
            in_step = true;
            step_start = i;
        } else if (dm_d_temp[i].abs() <= threshold || i == n - 1) && in_step {
            in_step = false;
            let t_start = temperatures[step_start];
            let t_end = temperatures[i];
            let loss = mass_pct[step_start] - mass_pct[i];
            if loss.abs() > 0.1 {
                steps.push((t_start, t_end, loss));
            }
        }
    }
    steps
}

/// TGA-MS correlation: align mass spectrometer m/z channel with TGA temperature axis
#[derive(Debug, Clone)]
pub struct TgaMsPoint {
    pub temperature_c: f64,
    pub mz_intensity: f64,
}

/// Correlate MS intensity with TGA mass loss rate
/// Returns Pearson correlation coefficient
pub fn tga_ms_correlation(
    dm_dt_values: &[f64],
    ms_intensities: &[f64],
) -> f64 {
    let n = dm_dt_values.len().min(ms_intensities.len());
    if n < 3 {
        return 0.0;
    }

    let mean_x: f64 = dm_dt_values[..n].iter().sum::<f64>() / n as f64;
    let mean_y: f64 = ms_intensities[..n].iter().sum::<f64>() / n as f64;

    let mut sum_xy = 0.0;
    let mut sum_xx = 0.0;
    let mut sum_yy = 0.0;

    for i in 0..n {
        let dx = dm_dt_values[i] - mean_x;
        let dy = ms_intensities[i] - mean_y;
        sum_xy += dx * dy;
        sum_xx += dx * dx;
        sum_yy += dy * dy;
    }

    let denom = (sum_xx * sum_yy).sqrt();
    if denom < 1e-30 {
        return 0.0;
    }
    sum_xy / denom
}

/// Smooth TGA data using Savitzky-Golay-like moving average
pub fn smooth_tga(values: &[f64], window: usize) -> Vec<f64> {
    let n = values.len();
    if n == 0 || window < 2 {
        return values.to_vec();
    }
    let half = window / 2;
    let mut result = Vec::with_capacity(n);
    for i in 0..n {
        let start = if i >= half { i - half } else { 0 };
        let end = (i + half + 1).min(n);
        let sum: f64 = values[start..end].iter().sum();
        result.push(sum / (end - start) as f64);
    }
    result
}

/// Temperature at a given conversion fraction
pub fn temperature_at_conversion(
    data: &[TgaPoint],
    alpha_target: f64,
) -> f64 {
    let alpha = conversion_fraction(data);
    let n = data.len().min(alpha.len());
    if n < 2 {
        return 0.0;
    }
    for i in 1..n {
        if alpha[i] >= alpha_target && alpha[i - 1] < alpha_target {
            let frac = (alpha_target - alpha[i - 1]) / (alpha[i] - alpha[i - 1]);
            return data[i - 1].temperature_c
                + frac * (data[i].temperature_c - data[i - 1].temperature_c);
        }
    }
    data[n - 1].temperature_c
}

/// TGA processor - orchestrates full analysis
pub struct TgaEgaProcessor {
    pub data: Vec<TgaPoint>,
    pub heating_rate_c_per_min: f64,
}

impl TgaEgaProcessor {
    pub fn new(data: Vec<TgaPoint>, heating_rate: f64) -> Self {
        Self {
            data,
            heating_rate_c_per_min: heating_rate,
        }
    }

    pub fn mass_percent(&self) -> Vec<f64> {
        mass_percent(&self.data)
    }

    pub fn derivative(&self) -> DtgResult {
        derivative_tga(&self.data)
    }

    pub fn residual(&self) -> f64 {
        residual_mass_pct(&self.data)
    }

    pub fn conversion(&self) -> Vec<f64> {
        conversion_fraction(&self.data)
    }

    pub fn onset_endset(&self) -> (f64, f64) {
        let mp = self.mass_percent();
        let temps: Vec<f64> = self.data.iter().map(|p| p.temperature_c).collect();
        onset_endset_temperature(&temps, &mp)
    }

    pub fn weight_loss(&self, t_start: f64, t_end: f64) -> f64 {
        let mp = self.mass_percent();
        let temps: Vec<f64> = self.data.iter().map(|p| p.temperature_c).collect();
        weight_loss_step(&temps, &mp, t_start, t_end)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn make_tga_data(n: usize, heating_rate: f64) -> Vec<TgaPoint> {
        // Simulate single-step decomposition: 100mg → 60mg around 300°C
        let m0 = 100.0;
        let mf = 60.0;
        let t_mid = 300.0;
        let width = 30.0;
        (0..n)
            .map(|i| {
                let t = 25.0 + (i as f64) * 475.0 / (n as f64 - 1.0); // 25-500°C
                let time_s = (t - 25.0) / heating_rate * 60.0;
                // Sigmoidal mass loss
                let alpha = 0.5 * (1.0 + tanh_approx((t - t_mid) / width));
                let mass = m0 - (m0 - mf) * alpha;
                TgaPoint { temperature_c: t, mass_mg: mass, time_s }
            })
            .collect()
    }

    fn tanh_approx(x: f64) -> f64 {
        let e2x = (2.0 * x).exp();
        if e2x.is_infinite() { return 1.0; }
        (e2x - 1.0) / (e2x + 1.0)
    }

    #[test]
    fn test_mass_percent_initial() {
        let data = make_tga_data(100, 10.0);
        let mp = mass_percent(&data);
        assert!(approx_eq(mp[0], 100.0, 0.01));
    }

    #[test]
    fn test_mass_percent_final() {
        let data = make_tga_data(100, 10.0);
        let mp = mass_percent(&data);
        assert!(approx_eq(*mp.last().unwrap(), 60.0, 1.0));
    }

    #[test]
    fn test_mass_percent_empty() {
        let mp = mass_percent(&[]);
        assert!(mp.is_empty());
    }

    #[test]
    fn test_mass_percent_monotonic_decrease() {
        let data = make_tga_data(100, 10.0);
        let mp = mass_percent(&data);
        for i in 1..mp.len() {
            assert!(mp[i] <= mp[i - 1] + 0.01, "mass should decrease: {} > {}", mp[i], mp[i - 1]);
        }
    }

    #[test]
    fn test_derivative_tga_size() {
        let data = make_tga_data(50, 10.0);
        let dtg = derivative_tga(&data);
        assert_eq!(dtg.temperatures.len(), 50);
        assert_eq!(dtg.dm_dt.len(), 50);
        assert_eq!(dtg.dm_d_temp.len(), 50);
    }

    #[test]
    fn test_derivative_tga_sign() {
        let data = make_tga_data(200, 10.0);
        let dtg = derivative_tga(&data);
        // Most dm_d_temp should be negative (mass loss)
        let neg_count = dtg.dm_d_temp.iter().filter(|&&v| v < 0.0).count();
        assert!(neg_count > dtg.dm_d_temp.len() / 3);
    }

    #[test]
    fn test_derivative_short() {
        let data = vec![
            TgaPoint { temperature_c: 25.0, mass_mg: 10.0, time_s: 0.0 },
        ];
        let dtg = derivative_tga(&data);
        assert_eq!(dtg.dm_dt.len(), 1);
    }

    #[test]
    fn test_onset_endset() {
        let data = make_tga_data(500, 10.0);
        let mp = mass_percent(&data);
        let temps: Vec<f64> = data.iter().map(|p| p.temperature_c).collect();
        let (onset, endset) = onset_endset_temperature(&temps, &mp);
        // Onset should be before 300°C (midpoint), endset after
        assert!(onset < 310.0, "onset = {}", onset);
        assert!(endset > 290.0, "endset = {}", endset);
        assert!(onset < endset);
    }

    #[test]
    fn test_weight_loss_step() {
        let data = make_tga_data(500, 10.0);
        let mp = mass_percent(&data);
        let temps: Vec<f64> = data.iter().map(|p| p.temperature_c).collect();
        let loss = weight_loss_step(&temps, &mp, 200.0, 400.0);
        // Should capture most of the 40% loss
        assert!(loss > 30.0, "loss = {}", loss);
        assert!(loss <= 41.0, "loss = {}", loss);
    }

    #[test]
    fn test_weight_loss_step_no_range() {
        let temps = vec![100.0, 200.0, 300.0];
        let mp = vec![100.0, 90.0, 80.0];
        let loss = weight_loss_step(&temps, &mp, 150.0, 150.0);
        assert!(approx_eq(loss, 0.0, 0.1));
    }

    #[test]
    fn test_residual_mass() {
        let data = make_tga_data(100, 10.0);
        let res = residual_mass_pct(&data);
        assert!(approx_eq(res, 60.0, 1.0));
    }

    #[test]
    fn test_residual_mass_empty() {
        assert_eq!(residual_mass_pct(&[]), 0.0);
    }

    #[test]
    fn test_conversion_fraction() {
        let data = make_tga_data(100, 10.0);
        let alpha = conversion_fraction(&data);
        assert!(approx_eq(alpha[0], 0.0, 0.01));
        assert!(approx_eq(*alpha.last().unwrap(), 1.0, 0.01));
    }

    #[test]
    fn test_conversion_fraction_monotonic() {
        let data = make_tga_data(100, 10.0);
        let alpha = conversion_fraction(&data);
        for i in 1..alpha.len() {
            assert!(alpha[i] >= alpha[i - 1] - 0.001);
        }
    }

    #[test]
    fn test_kissinger_activation_energy() {
        // Direct synthetic: construct Tp values that satisfy the Kissinger relation
        // ln(beta/Tp^2) = C - Ea/(R*Tp)
        // For Ea = 100 kJ/mol = 100000 J/mol, pick Tp values and compute
        let r: f64 = 8.314;
        let ea_true: f64 = 100000.0; // J/mol

        // Realistic peak temperatures (K) for different heating rates
        // Higher heating rate → higher peak temperature
        let tps: Vec<f64> = vec![580.0, 595.0, 610.0, 625.0];
        // Compute betas from the Kissinger equation: ln(beta/Tp^2) = C - Ea/(R*Tp)
        // Using C chosen to give reasonable beta values
        let c_val = (r * 1e12 / ea_true).ln(); // ln(AR/Ea)
        let betas: Vec<f64> = tps.iter().map(|&tp| {
            let ln_beta_tp2 = c_val - ea_true / (r * tp);
            (tp * tp * ln_beta_tp2.exp())
        }).collect();

        let (ea_calc, _ln_a) = kissinger_activation_energy(&betas, &tps);
        assert!(approx_eq(ea_calc, 100.0, 5.0), "Ea = {} kJ/mol", ea_calc);
    }

    #[test]
    fn test_kissinger_two_points() {
        let (ea, _) = kissinger_activation_energy(&[10.0, 20.0], &[600.0, 610.0]);
        assert!(ea > 0.0);
    }

    #[test]
    fn test_kissinger_insufficient() {
        let (ea, _) = kissinger_activation_energy(&[10.0], &[600.0]);
        assert_eq!(ea, 0.0);
    }

    #[test]
    fn test_ozawa_flynn_wall() {
        let r = 8.314;
        let ea_true = 120000.0; // J/mol

        let betas = vec![5.0, 10.0, 15.0, 20.0];
        let mut temps = Vec::new();

        // At fixed alpha, ln(beta) = const - 1.052*Ea/(R*T)
        // T = 1.052*Ea / (R * (const - ln(beta)))
        let c = 30.0; // arbitrary constant
        for &beta in &betas {
            let beta: f64 = beta;
            let t = 1.052 * ea_true / (r * (c - beta.ln()));
            temps.push(t);
        }

        let ea_calc = ozawa_flynn_wall(&betas, &temps);
        assert!(approx_eq(ea_calc, 120.0, 5.0), "Ea = {} kJ/mol", ea_calc);
    }

    #[test]
    fn test_gas_evolution_rate() {
        // 1 mg/min of CO2 (M=44)
        let rate = gas_evolution_rate_mol_per_min(1.0, 44.0);
        assert!(approx_eq(rate, 1.0 / 44000.0, 1e-8));
    }

    #[test]
    fn test_gas_evolution_rate_zero() {
        assert_eq!(gas_evolution_rate_mol_per_min(1.0, 0.0), 0.0);
    }

    #[test]
    fn test_buoyancy_correction() {
        let mass = vec![100.0; 5];
        let temps = vec![300.0, 400.0, 500.0, 600.0, 700.0]; // K
        let corrected = buoyancy_correction(&mass, &temps, 0.1, 28.97, 1.0);
        // Higher temp → lower gas density → less buoyancy → apparent mass increases
        assert_eq!(corrected.len(), 5);
        assert!(approx_eq(corrected[0], 100.0, 0.001)); // reference: no correction
        assert!(corrected[4] > corrected[0]); // correction positive at higher T
    }

    #[test]
    fn test_find_dtg_peaks() {
        let temps = vec![100.0, 200.0, 300.0, 400.0, 500.0];
        let dm_dt = vec![-0.1, -0.5, -2.0, -0.3, -0.05];
        let peaks = find_dtg_peaks(&temps, &dm_dt);
        assert_eq!(peaks.len(), 1);
        assert!(approx_eq(peaks[0].0, 300.0, 0.1));
    }

    #[test]
    fn test_find_dtg_no_peaks() {
        let temps = vec![100.0, 200.0, 300.0];
        let dm_dt = vec![-0.1, -0.5, -1.0]; // monotonically decreasing
        let peaks = find_dtg_peaks(&temps, &dm_dt);
        assert_eq!(peaks.len(), 0);
    }

    #[test]
    fn test_decompose_steps() {
        let temps = vec![100.0, 200.0, 250.0, 300.0, 350.0, 400.0, 500.0];
        let mp = vec![100.0, 99.0, 90.0, 80.0, 79.0, 78.5, 78.0];
        let dm_dt = vec![0.0, -0.05, -0.5, -0.6, -0.05, -0.02, -0.01];
        let steps = decompose_steps(&temps, &mp, &dm_dt, 0.1);
        assert!(!steps.is_empty());
    }

    #[test]
    fn test_tga_ms_correlation() {
        // Perfect negative correlation
        let dm = vec![-1.0, -2.0, -3.0, -2.0, -1.0];
        let ms = vec![1.0, 2.0, 3.0, 2.0, 1.0];
        let r = tga_ms_correlation(&dm, &ms);
        assert!(approx_eq(r, -1.0, 0.01));
    }

    #[test]
    fn test_tga_ms_correlation_uncorrelated() {
        let dm = vec![1.0, -1.0, 1.0, -1.0, 1.0, -1.0];
        let ms = vec![1.0, 1.0, -1.0, -1.0, 1.0, 1.0];
        let r = tga_ms_correlation(&dm, &ms);
        assert!(r.abs() < 0.5);
    }

    #[test]
    fn test_smooth_tga() {
        let values = vec![1.0, 10.0, 1.0, 10.0, 1.0];
        let smoothed = smooth_tga(&values, 3);
        assert_eq!(smoothed.len(), 5);
        // Middle values should be smoothed
        assert!(smoothed[2] > 1.0 && smoothed[2] < 10.0);
    }

    #[test]
    fn test_smooth_tga_no_window() {
        let values = vec![1.0, 2.0, 3.0];
        let smoothed = smooth_tga(&values, 1);
        assert_eq!(smoothed, values);
    }

    #[test]
    fn test_temperature_at_conversion() {
        let data = make_tga_data(500, 10.0);
        let t50 = temperature_at_conversion(&data, 0.5);
        assert!(approx_eq(t50, 300.0, 5.0), "T50 = {}", t50);
    }

    #[test]
    fn test_temperature_at_conversion_low() {
        let data = make_tga_data(500, 10.0);
        let t10 = temperature_at_conversion(&data, 0.1);
        assert!(t10 < 300.0, "T10 = {}", t10);
    }

    #[test]
    fn test_interpolate_at_temp() {
        let temps = vec![100.0, 200.0, 300.0];
        let vals = vec![10.0, 20.0, 30.0];
        let v = interpolate_at_temp(&temps, &vals, 150.0);
        assert!(approx_eq(v, 15.0, 0.01));
    }

    #[test]
    fn test_interpolate_at_temp_boundary() {
        let temps = vec![100.0, 200.0];
        let vals = vec![10.0, 20.0];
        assert!(approx_eq(interpolate_at_temp(&temps, &vals, 50.0), 10.0, 0.01));
        assert!(approx_eq(interpolate_at_temp(&temps, &vals, 300.0), 20.0, 0.01));
    }

    #[test]
    fn test_processor_new() {
        let data = make_tga_data(100, 10.0);
        let proc = TgaEgaProcessor::new(data, 10.0);
        assert!(approx_eq(proc.heating_rate_c_per_min, 10.0, 0.01));
    }

    #[test]
    fn test_processor_mass_percent() {
        let data = make_tga_data(100, 10.0);
        let proc = TgaEgaProcessor::new(data, 10.0);
        let mp = proc.mass_percent();
        assert!(approx_eq(mp[0], 100.0, 0.01));
    }

    #[test]
    fn test_processor_derivative() {
        let data = make_tga_data(100, 10.0);
        let proc = TgaEgaProcessor::new(data, 10.0);
        let dtg = proc.derivative();
        assert_eq!(dtg.temperatures.len(), 100);
    }

    #[test]
    fn test_processor_residual() {
        let data = make_tga_data(100, 10.0);
        let proc = TgaEgaProcessor::new(data, 10.0);
        assert!(approx_eq(proc.residual(), 60.0, 1.0));
    }

    #[test]
    fn test_processor_conversion() {
        let data = make_tga_data(100, 10.0);
        let proc = TgaEgaProcessor::new(data, 10.0);
        let alpha = proc.conversion();
        assert!(approx_eq(alpha[0], 0.0, 0.01));
    }

    #[test]
    fn test_processor_onset_endset() {
        let data = make_tga_data(500, 10.0);
        let proc = TgaEgaProcessor::new(data, 10.0);
        let (onset, endset) = proc.onset_endset();
        assert!(onset < endset);
    }

    #[test]
    fn test_processor_weight_loss() {
        let data = make_tga_data(500, 10.0);
        let proc = TgaEgaProcessor::new(data, 10.0);
        let loss = proc.weight_loss(100.0, 500.0);
        assert!(loss > 35.0);
    }

    #[test]
    fn test_two_step_decomposition() {
        // Two-step: loss at 200°C and 400°C
        let n = 500;
        let data: Vec<TgaPoint> = (0..n)
            .map(|i| {
                let t = 25.0 + i as f64;
                let time = i as f64 * 6.0;
                let a1 = 0.5 * (1.0 + tanh_approx((t - 200.0) / 20.0));
                let a2 = 0.5 * (1.0 + tanh_approx((t - 400.0) / 20.0));
                let mass = 100.0 - 20.0 * a1 - 15.0 * a2;
                TgaPoint { temperature_c: t, mass_mg: mass, time_s: time }
            })
            .collect();
        let mp = mass_percent(&data);
        // First step ~20%, second ~15%, residual ~65%
        assert!(approx_eq(*mp.last().unwrap(), 65.0, 2.0));
    }
}
