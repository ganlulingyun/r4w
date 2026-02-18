// quartz_crystal_microbalance_processor.rs
//
// Quartz Crystal Microbalance (QCM / QCM-D) signal processing.
// Sauerbrey mass, overtone analysis, Voigt viscoelastic model, Kanazawa-Gordon,
// adsorption kinetics, BVD equivalent circuit.

/// Physical constants for QCM.
pub const QUARTZ_DENSITY: f64 = 2.648e3;   // kg/m³ (AT-cut quartz)
pub const QUARTZ_SHEAR_MODULUS: f64 = 2.947e10; // Pa (AT-cut quartz)

/// QCM data point.
#[derive(Debug, Clone, Copy)]
pub struct QcmDataPoint {
    pub time_s: f64,
    pub delta_f_hz: f64,
    pub delta_d: f64, // dissipation change (dimensionless, typically 1e-6)
}

/// QCM time series data.
#[derive(Debug, Clone)]
pub struct QcmData {
    pub points: Vec<QcmDataPoint>,
    pub fundamental_hz: f64,
}

/// Sauerbrey result.
#[derive(Debug, Clone)]
pub struct SauerbreyResult {
    pub delta_m_ng_cm2: f64,
    pub sauerbrey_constant: f64,
    pub overtone: u32,
}

/// Voigt model fit result.
#[derive(Debug, Clone)]
pub struct VoigtResult {
    pub thickness_nm: f64,
    pub density_kg_m3: f64,
    pub shear_modulus_pa: f64,
    pub viscosity_pa_s: f64,
    pub chi_squared: f64,
}

/// Adsorption kinetics result.
#[derive(Debug, Clone)]
pub struct AdsorptionResult {
    pub model: AdsorptionModel,
    pub coverage_max: f64,
    pub rate_constant: f64,
    pub r_squared: f64,
}

/// Adsorption model type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AdsorptionModel {
    Langmuir,
    BET,
    FirstOrder,
}

impl QcmData {
    pub fn new(fundamental_hz: f64) -> Self {
        Self { points: Vec::new(), fundamental_hz }
    }

    pub fn add_point(&mut self, time_s: f64, delta_f_hz: f64, delta_d: f64) {
        self.points.push(QcmDataPoint { time_s, delta_f_hz, delta_d });
    }

    pub fn len(&self) -> usize { self.points.len() }

    /// Get frequency shift at latest point.
    pub fn latest_df(&self) -> f64 {
        self.points.last().map_or(0.0, |p| p.delta_f_hz)
    }

    /// Get dissipation at latest point.
    pub fn latest_dd(&self) -> f64 {
        self.points.last().map_or(0.0, |p| p.delta_d)
    }

    /// Frequency shift time series.
    pub fn frequency_shifts(&self) -> Vec<f64> {
        self.points.iter().map(|p| p.delta_f_hz).collect()
    }

    /// Dissipation time series.
    pub fn dissipations(&self) -> Vec<f64> {
        self.points.iter().map(|p| p.delta_d).collect()
    }

    /// Time axis.
    pub fn times(&self) -> Vec<f64> {
        self.points.iter().map(|p| p.time_s).collect()
    }
}

// ---------------------------------------------------------------------------
// 2. Sauerbrey Equation
// ---------------------------------------------------------------------------

/// Sauerbrey constant: C = sqrt(ρ_q * μ_q) / (2 * f0²).
/// Returns C in kg/(m²·Hz), convert to ng/(cm²·Hz) with factor 1e13.
pub fn sauerbrey_constant(f0_hz: f64) -> f64 {
    (QUARTZ_DENSITY * QUARTZ_SHEAR_MODULUS).sqrt() / (2.0 * f0_hz * f0_hz)
}

/// Sauerbrey mass change: Δm = -C * Δf/n.
/// Returns mass in ng/cm².
pub fn sauerbrey_mass(f0_hz: f64, delta_f_hz: f64, overtone: u32) -> f64 {
    let c = sauerbrey_constant(f0_hz);
    let dm_kg_m2 = -c * delta_f_hz / overtone as f64;
    dm_kg_m2 * 1e13 // kg/m² to ng/cm²
}

/// Full Sauerbrey analysis.
pub fn sauerbrey_analysis(f0_hz: f64, delta_f_hz: f64, overtone: u32) -> SauerbreyResult {
    let c = sauerbrey_constant(f0_hz);
    SauerbreyResult {
        delta_m_ng_cm2: sauerbrey_mass(f0_hz, delta_f_hz, overtone),
        sauerbrey_constant: c,
        overtone,
    }
}

// ---------------------------------------------------------------------------
// 3. Overtone Analysis
// ---------------------------------------------------------------------------

/// Overtone-normalized frequency shift: Δf/n for each harmonic.
pub fn normalized_frequency_shifts(
    delta_f_per_overtone: &[(u32, f64)],
) -> Vec<(u32, f64)> {
    delta_f_per_overtone.iter()
        .map(|&(n, df)| (n, df / n as f64))
        .collect()
}

/// Check Sauerbrey validity: Δf/n should be constant across overtones.
/// Returns coefficient of variation (CV).
pub fn sauerbrey_validity(delta_f_per_overtone: &[(u32, f64)]) -> f64 {
    let normalized: Vec<f64> = delta_f_per_overtone.iter()
        .map(|&(n, df)| df / n as f64)
        .collect();
    if normalized.len() < 2 { return 0.0; }
    let mean: f64 = normalized.iter().sum::<f64>() / normalized.len() as f64;
    if mean.abs() < 1e-30 { return 0.0; }
    let var: f64 = normalized.iter().map(|v| (v - mean).powi(2)).sum::<f64>()
        / (normalized.len() - 1) as f64;
    var.sqrt() / mean.abs()
}

/// ΔD/(-Δf/n) ratio for each overtone (viscoelastic indicator).
pub fn dissipation_frequency_ratio(
    overtone_data: &[(u32, f64, f64)], // (n, Δf, ΔD)
) -> Vec<(u32, f64)> {
    overtone_data.iter()
        .map(|&(n, df, dd)| {
            let df_n = df / n as f64;
            let ratio = if df_n.abs() < 1e-30 { 0.0 } else { dd / (-df_n) };
            (n, ratio)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// 4. Kanazawa-Gordon Equation
// ---------------------------------------------------------------------------

/// Kanazawa-Gordon equation for viscous liquid loading:
/// Δf = -f0^(3/2) * sqrt(ρ_L * η_L / (π * ρ_q * μ_q))
pub fn kanazawa_gordon(f0_hz: f64, rho_l_kg_m3: f64, eta_l_pa_s: f64) -> f64 {
    let pi = std::f64::consts::PI;
    -f0_hz.powf(1.5) * (rho_l_kg_m3 * eta_l_pa_s / (pi * QUARTZ_DENSITY * QUARTZ_SHEAR_MODULUS)).sqrt()
}

/// Extract viscosity-density product from frequency shift in liquid.
pub fn viscosity_density_from_df(f0_hz: f64, delta_f_hz: f64) -> f64 {
    let pi = std::f64::consts::PI;
    let f03_2 = f0_hz.powf(1.5);
    if f03_2.abs() < 1e-30 { return 0.0; }
    let ratio = delta_f_hz / (-f03_2);
    ratio * ratio * pi * QUARTZ_DENSITY * QUARTZ_SHEAR_MODULUS
}

// ---------------------------------------------------------------------------
// 5. BVD Equivalent Circuit
// ---------------------------------------------------------------------------

/// Butterworth-van Dyke circuit parameters.
#[derive(Debug, Clone)]
pub struct BvdParams {
    pub r1_ohm: f64,   // motional resistance
    pub l1_h: f64,     // motional inductance
    pub c1_f: f64,     // motional capacitance
    pub c0_f: f64,     // static capacitance
}

impl BvdParams {
    /// Quality factor: Q = 2πf0*L1/R1.
    pub fn quality_factor(&self, f0_hz: f64) -> f64 {
        let pi = std::f64::consts::PI;
        if self.r1_ohm.abs() < 1e-30 { return f64::INFINITY; }
        2.0 * pi * f0_hz * self.l1_h / self.r1_ohm
    }

    /// Resonant frequency from L1*C1: f0 = 1/(2π*sqrt(L1*C1)).
    pub fn resonant_frequency(&self) -> f64 {
        let pi = std::f64::consts::PI;
        let lc = self.l1_h * self.c1_f;
        if lc <= 0.0 { return 0.0; }
        1.0 / (2.0 * pi * lc.sqrt())
    }

    /// Dissipation: D = R1/(2πf0*L1) = 1/Q.
    pub fn dissipation(&self, f0_hz: f64) -> f64 {
        let q = self.quality_factor(f0_hz);
        if q.abs() < 1e-30 { return f64::INFINITY; }
        1.0 / q
    }
}

/// Estimate BVD parameters from f0, Q, and static capacitance.
pub fn estimate_bvd(f0_hz: f64, q_factor: f64, c0_f: f64) -> BvdParams {
    let pi = std::f64::consts::PI;
    let omega0 = 2.0 * pi * f0_hz;
    // For AT-cut quartz: C1 ≈ 8*C0*K² where K² is piezoelectric coupling
    let c1 = c0_f * 0.01; // typical C1/C0 ratio
    let l1 = 1.0 / (omega0 * omega0 * c1);
    let r1 = omega0 * l1 / q_factor;
    BvdParams { r1_ohm: r1, l1_h: l1, c1_f: c1, c0_f }
}

// ---------------------------------------------------------------------------
// 6. Film Thickness
// ---------------------------------------------------------------------------

/// Film thickness from mass and density: d = Δm / ρ.
/// Returns thickness in nm.
pub fn film_thickness_nm(delta_m_ng_cm2: f64, density_kg_m3: f64) -> f64 {
    if density_kg_m3.abs() < 1e-30 { return 0.0; }
    // ng/cm² to kg/m²: (1e-12 kg/ng) / (1e-4 m²/cm²) = 1e-8
    let dm_kg_m2 = delta_m_ng_cm2 * 1e-8;
    let d_m = dm_kg_m2 / density_kg_m3;
    d_m * 1e9
}

/// Voigt model: simplified frequency and dissipation dependence on overtone.
/// For a thin viscoelastic film in liquid:
/// Δf/n ≈ -(1/(2πρ_q*h_q)) * [h_f*ρ_f*ω - 2*h_f*(η_f/(δ_f))²*(1/(ρ_L*η_L*ω))]
/// This is a simplified version; returns (Δf/n, ΔD) for given parameters.
pub fn voigt_model_prediction(
    f0_hz: f64, overtone: u32,
    h_f_m: f64, rho_f: f64, mu_f: f64, eta_f: f64,
    rho_l: f64, eta_l: f64,
) -> (f64, f64) {
    let pi = std::f64::consts::PI;
    let n = overtone as f64;
    let omega = 2.0 * pi * f0_hz * n;
    let h_q = (QUARTZ_SHEAR_MODULUS / QUARTZ_DENSITY).sqrt() / (2.0 * f0_hz);
    // Sauerbrey-like mass term
    let df_mass = -n * f0_hz * h_f_m * rho_f / (pi * QUARTZ_DENSITY * h_q);
    // Viscous correction from liquid and film
    let delta_l = (2.0 * eta_l / (rho_l * omega)).sqrt(); // penetration depth in liquid
    let df_visc = n * f0_hz * h_f_m * h_f_m * rho_f * eta_f * omega
        / (3.0 * pi * QUARTZ_DENSITY * h_q * mu_f * mu_f * delta_l).max(1e-30);
    let df = df_mass + df_visc;
    // Dissipation from energy loss
    let dd = -2.0 * df_visc / (n * f0_hz).max(1e-30);
    (df / n, dd)
}

// ---------------------------------------------------------------------------
// 7. Adsorption Kinetics
// ---------------------------------------------------------------------------

/// Langmuir adsorption: θ = Ka*C / (1 + Ka*C).
pub fn langmuir_coverage(ka: f64, concentration: f64) -> f64 {
    let kc = ka * concentration;
    kc / (1.0 + kc)
}

/// Langmuir kinetics: θ(t) = θ_eq * (1 - exp(-k_obs * t)).
/// k_obs = ka*C + kd.
pub fn langmuir_kinetics(theta_eq: f64, k_obs: f64, t: f64) -> f64 {
    theta_eq * (1.0 - (-k_obs * t).exp())
}

/// BET isotherm: V/Vm = C*x / ((1-x)*(1 + (C-1)*x)) where x = P/P0.
pub fn bet_coverage(c_bet: f64, x: f64) -> f64 {
    if x >= 1.0 || x <= 0.0 { return 0.0; }
    c_bet * x / ((1.0 - x) * (1.0 + (c_bet - 1.0) * x))
}

/// Fit first-order adsorption kinetics to Δf(t).
pub fn fit_first_order(time: &[f64], delta_f: &[f64]) -> AdsorptionResult {
    let n = time.len().min(delta_f.len());
    if n < 3 {
        return AdsorptionResult { model: AdsorptionModel::FirstOrder, coverage_max: 0.0, rate_constant: 0.0, r_squared: 0.0 };
    }
    let df_eq = delta_f[n - 1]; // equilibrium value (last point)
    if df_eq.abs() < 1e-30 {
        return AdsorptionResult { model: AdsorptionModel::FirstOrder, coverage_max: 0.0, rate_constant: 0.0, r_squared: 0.0 };
    }
    // ln(1 - Δf/Δf_eq) = -k*t
    let mut sx = 0.0_f64; let mut sy = 0.0_f64;
    let mut sxy = 0.0_f64; let mut sxx = 0.0_f64;
    let mut count = 0.0_f64;
    for i in 0..n {
        let ratio = 1.0 - delta_f[i] / df_eq;
        if ratio <= 0.0 { continue; }
        let x = time[i];
        let y = ratio.ln();
        sx += x; sy += y; sxy += x * y; sxx += x * x;
        count += 1.0;
    }
    if count < 2.0 {
        return AdsorptionResult { model: AdsorptionModel::FirstOrder, coverage_max: df_eq, rate_constant: 0.0, r_squared: 0.0 };
    }
    let denom = count * sxx - sx * sx;
    if denom.abs() < 1e-30 {
        return AdsorptionResult { model: AdsorptionModel::FirstOrder, coverage_max: df_eq, rate_constant: 0.0, r_squared: 0.0 };
    }
    let slope = (count * sxy - sx * sy) / denom;
    let k = -slope;
    let y_mean = sy / count;
    let ss_tot: f64 = (0..n).filter_map(|i| {
        let ratio = 1.0 - delta_f[i] / df_eq;
        if ratio <= 0.0 { None } else { Some((ratio.ln() - y_mean).powi(2)) }
    }).sum();
    let ss_res: f64 = (0..n).filter_map(|i| {
        let ratio = 1.0 - delta_f[i] / df_eq;
        if ratio <= 0.0 { None } else {
            let pred = slope * time[i] + (sy - slope * sx) / count;
            Some((ratio.ln() - pred).powi(2))
        }
    }).sum();
    let r2 = if ss_tot > 1e-30 { 1.0 - ss_res / ss_tot } else { 0.0 };
    AdsorptionResult {
        model: AdsorptionModel::FirstOrder,
        coverage_max: df_eq,
        rate_constant: k,
        r_squared: r2,
    }
}

// ---------------------------------------------------------------------------
// 8. QCM Processor
// ---------------------------------------------------------------------------

/// Main QCM processor.
pub struct QcmProcessor {
    pub fundamental_hz: f64,
    pub mass_ng_cm2: Option<f64>,
    pub thickness_nm: Option<f64>,
    pub adsorption: Option<AdsorptionResult>,
}

impl QcmProcessor {
    pub fn new(fundamental_hz: f64) -> Self {
        Self { fundamental_hz, mass_ng_cm2: None, thickness_nm: None, adsorption: None }
    }

    /// Standard 5 MHz AT-cut quartz.
    pub fn standard_5mhz() -> Self { Self::new(5e6) }

    /// Calculate mass from frequency shift.
    pub fn calculate_mass(&mut self, delta_f_hz: f64, overtone: u32) -> f64 {
        let m = sauerbrey_mass(self.fundamental_hz, delta_f_hz, overtone);
        self.mass_ng_cm2 = Some(m);
        m
    }

    /// Calculate thickness from mass and density.
    pub fn calculate_thickness(&mut self, density_kg_m3: f64) -> f64 {
        let m = self.mass_ng_cm2.unwrap_or(0.0);
        let t = film_thickness_nm(m, density_kg_m3);
        self.thickness_nm = Some(t);
        t
    }

    /// Fit adsorption kinetics.
    pub fn fit_adsorption(&mut self, data: &QcmData) {
        let times = data.times();
        let dfs = data.frequency_shifts();
        self.adsorption = Some(fit_first_order(&times, &dfs));
    }

    /// Kanazawa-Gordon liquid loading prediction.
    pub fn liquid_loading(&self, rho_l: f64, eta_l: f64) -> f64 {
        kanazawa_gordon(self.fundamental_hz, rho_l, eta_l)
    }

    /// Check Sauerbrey applicability from overtone data.
    pub fn check_sauerbrey(&self, overtone_data: &[(u32, f64)]) -> f64 {
        sauerbrey_validity(overtone_data)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

fn approx_eq(a: f64, b: f64, tol: f64) -> bool { (a - b).abs() < tol }

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sauerbrey_constant() {
        let c = sauerbrey_constant(5e6);
        // For 5 MHz AT-cut: C ≈ 17.7 ng/(cm²·Hz) → 17.7e-6 kg/(m²·Hz) → 1.77e-5
        // Actually C in kg/(m²·Hz): sqrt(2648*2.947e10) / (2*(5e6)²) = 8.83e6 / 5e13 = 1.77e-7
        assert!(c > 1e-8 && c < 1e-5);
    }

    #[test]
    fn test_sauerbrey_mass_negative_df() {
        // Negative Δf → positive mass (material added)
        let m = sauerbrey_mass(5e6, -100.0, 1);
        assert!(m > 0.0);
    }

    #[test]
    fn test_sauerbrey_mass_positive_df() {
        // Positive Δf → negative mass (material removed)
        let m = sauerbrey_mass(5e6, 100.0, 1);
        assert!(m < 0.0);
    }

    #[test]
    fn test_sauerbrey_overtone_scaling() {
        let m1 = sauerbrey_mass(5e6, -100.0, 1);
        let m3 = sauerbrey_mass(5e6, -300.0, 3);
        assert!(approx_eq(m1, m3, m1 * 0.01));
    }

    #[test]
    fn test_sauerbrey_analysis() {
        let result = sauerbrey_analysis(5e6, -50.0, 1);
        assert!(result.delta_m_ng_cm2 > 0.0);
        assert_eq!(result.overtone, 1);
    }

    #[test]
    fn test_normalized_frequency_shifts() {
        let data = vec![(1, -100.0), (3, -300.0), (5, -500.0)];
        let norm = normalized_frequency_shifts(&data);
        assert!(approx_eq(norm[0].1, -100.0, 0.01));
        assert!(approx_eq(norm[1].1, -100.0, 0.01));
        assert!(approx_eq(norm[2].1, -100.0, 0.01));
    }

    #[test]
    fn test_sauerbrey_validity_good() {
        let data = vec![(1, -100.0), (3, -300.0), (5, -500.0)];
        let cv = sauerbrey_validity(&data);
        assert!(cv < 0.01); // Very low CV = good Sauerbrey
    }

    #[test]
    fn test_sauerbrey_validity_bad() {
        let data = vec![(1, -100.0), (3, -250.0), (5, -350.0)];
        let cv = sauerbrey_validity(&data);
        assert!(cv > 0.05); // High CV = viscoelastic film
    }

    #[test]
    fn test_dissipation_frequency_ratio() {
        let data = vec![(1, -100.0, 2e-6), (3, -300.0, 6e-6)];
        let ratios = dissipation_frequency_ratio(&data);
        assert!(approx_eq(ratios[0].1, ratios[1].1, 1e-6));
    }

    #[test]
    fn test_kanazawa_gordon_water() {
        // Water at 25°C: ρ=997, η=8.9e-4 Pa·s
        let df = kanazawa_gordon(5e6, 997.0, 8.9e-4);
        assert!(df < 0.0); // Frequency decreases in liquid
        assert!(df > -2000.0 && df < -500.0); // Typical ~-700 Hz range for 5 MHz in water
    }

    #[test]
    fn test_kanazawa_viscosity_increases() {
        let df1 = kanazawa_gordon(5e6, 997.0, 8.9e-4);
        let df2 = kanazawa_gordon(5e6, 997.0, 3e-3); // glycerol mixture
        assert!(df2 < df1); // More viscous → more negative
    }

    #[test]
    fn test_viscosity_density_roundtrip() {
        let rho_eta = 997.0 * 8.9e-4;
        let df = kanazawa_gordon(5e6, 997.0, 8.9e-4);
        let calc = viscosity_density_from_df(5e6, df);
        assert!(approx_eq(calc, rho_eta, rho_eta * 0.05));
    }

    #[test]
    fn test_bvd_quality_factor() {
        let bvd = BvdParams { r1_ohm: 10.0, l1_h: 0.04, c1_f: 25e-15, c0_f: 5e-12 };
        let q = bvd.quality_factor(5e6);
        assert!(q > 10000.0); // QCM Q > 10000 typical
    }

    #[test]
    fn test_bvd_resonant_frequency() {
        let bvd = estimate_bvd(5e6, 50000.0, 5e-12);
        let f = bvd.resonant_frequency();
        assert!(approx_eq(f, 5e6, 5e5)); // Close to 5 MHz
    }

    #[test]
    fn test_bvd_dissipation() {
        let bvd = estimate_bvd(5e6, 50000.0, 5e-12);
        let d = bvd.dissipation(5e6);
        assert!(approx_eq(d, 1.0 / 50000.0, 1e-6));
    }

    #[test]
    fn test_film_thickness() {
        // 1000 ng/cm² of protein (ρ ≈ 1400 kg/m³)
        let t = film_thickness_nm(1000.0, 1400.0);
        // 1000 ng/cm² = 1e-5 kg/m², d = 1e-5/1400 = 7.14e-9 m = 7.14 nm
        assert!(approx_eq(t, 7.14, 0.1));
    }

    #[test]
    fn test_langmuir_coverage() {
        let theta = langmuir_coverage(100.0, 0.01);
        // Ka*C = 1.0, θ = 0.5
        assert!(approx_eq(theta, 0.5, 0.01));
    }

    #[test]
    fn test_langmuir_coverage_saturation() {
        let theta = langmuir_coverage(100.0, 100.0);
        assert!(theta > 0.99); // Near saturation
    }

    #[test]
    fn test_langmuir_kinetics() {
        let theta = langmuir_kinetics(0.9, 0.5, 0.0);
        assert!(approx_eq(theta, 0.0, 1e-10));
        let theta2 = langmuir_kinetics(0.9, 0.5, 100.0);
        assert!(approx_eq(theta2, 0.9, 0.01)); // Equilibrium
    }

    #[test]
    fn test_bet_coverage() {
        let v = bet_coverage(50.0, 0.1);
        assert!(v > 0.0);
    }

    #[test]
    fn test_bet_coverage_at_boundary() {
        let v = bet_coverage(50.0, 0.0);
        assert!(approx_eq(v, 0.0, 1e-10));
    }

    #[test]
    fn test_fit_first_order() {
        let k_true = 0.1;
        let df_eq = -200.0;
        let time: Vec<f64> = (0..100).map(|i| i as f64 * 0.5).collect();
        let df: Vec<f64> = time.iter().map(|&t| df_eq * (1.0 - (-k_true * t).exp())).collect();
        let result = fit_first_order(&time, &df);
        assert!(approx_eq(result.rate_constant, k_true, 0.05));
        assert!(result.r_squared > 0.95);
    }

    #[test]
    fn test_qcm_data() {
        let mut data = QcmData::new(5e6);
        data.add_point(0.0, 0.0, 0.0);
        data.add_point(1.0, -50.0, 1e-6);
        assert_eq!(data.len(), 2);
        assert!(approx_eq(data.latest_df(), -50.0, 0.01));
        assert!(approx_eq(data.latest_dd(), 1e-6, 1e-9));
    }

    #[test]
    fn test_qcm_processor_mass() {
        let mut proc = QcmProcessor::standard_5mhz();
        let m = proc.calculate_mass(-100.0, 1);
        assert!(m > 0.0);
    }

    #[test]
    fn test_qcm_processor_thickness() {
        let mut proc = QcmProcessor::standard_5mhz();
        proc.calculate_mass(-100.0, 1);
        let t = proc.calculate_thickness(1400.0);
        assert!(t > 0.0);
    }

    #[test]
    fn test_qcm_processor_liquid_loading() {
        let proc = QcmProcessor::standard_5mhz();
        let df = proc.liquid_loading(997.0, 8.9e-4);
        assert!(df < 0.0);
    }

    #[test]
    fn test_qcm_processor_sauerbrey_check() {
        let proc = QcmProcessor::standard_5mhz();
        let data = vec![(1, -100.0), (3, -300.0), (5, -500.0)];
        let cv = proc.check_sauerbrey(&data);
        assert!(cv < 0.01);
    }

    #[test]
    fn test_qcm_processor_adsorption() {
        let mut proc = QcmProcessor::standard_5mhz();
        let mut data = QcmData::new(5e6);
        for i in 0..50 {
            let t = i as f64;
            data.add_point(t, -200.0 * (1.0 - (-0.1 * t).exp()), 0.0);
        }
        proc.fit_adsorption(&data);
        assert!(proc.adsorption.is_some());
    }

    #[test]
    fn test_voigt_model_prediction() {
        let (df, dd) = voigt_model_prediction(
            5e6, 1,
            10e-9, 1400.0, 1e6, 0.001,
            997.0, 8.9e-4,
        );
        assert!(df < 0.0); // Mass causes frequency decrease
        assert!(dd.is_finite());
    }

    #[test]
    fn test_qcm_data_time_series() {
        let mut data = QcmData::new(5e6);
        data.add_point(0.0, 0.0, 0.0);
        data.add_point(1.0, -10.0, 1e-7);
        data.add_point(2.0, -20.0, 2e-7);
        assert_eq!(data.times().len(), 3);
        assert_eq!(data.frequency_shifts().len(), 3);
        assert_eq!(data.dissipations().len(), 3);
    }
}
