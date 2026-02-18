// potentiostatic_sweep_analyzer.rs
//
// Potentiostatic sweep (cyclic voltammetry, linear sweep) analysis.
// Randles-Sevcik, Cottrell, Levich, Tafel analysis, peak finding.

use std::f64::consts::PI;

/// Physical constants for electrochemistry.
pub const FARADAY: f64 = 96485.3321; // C/mol
pub const GAS_CONST: f64 = 8.31446;  // J/(mol·K)

/// A voltammogram: potential (V) vs current (A).
#[derive(Debug, Clone)]
pub struct Voltammogram {
    pub potential_v: Vec<f64>,
    pub current_a: Vec<f64>,
}

/// Detected CV peak.
#[derive(Debug, Clone)]
pub struct CvPeak {
    pub potential_v: f64,
    pub current_a: f64,
    pub index: usize,
    pub is_anodic: bool,
}

/// Scan rate study result.
#[derive(Debug, Clone)]
pub struct ScanRateResult {
    pub scan_rates_vs: Vec<f64>,
    pub peak_currents_a: Vec<f64>,
    pub slope: f64,
    pub intercept: f64,
    pub r_squared: f64,
    pub is_diffusion_controlled: bool,
}

/// Tafel analysis result.
#[derive(Debug, Clone)]
pub struct TafelResult {
    pub anodic_slope_v_dec: f64,
    pub cathodic_slope_v_dec: f64,
    pub exchange_current_a: f64,
    pub corrosion_potential_v: f64,
    pub transfer_coeff_anodic: f64,
    pub transfer_coeff_cathodic: f64,
}

impl Voltammogram {
    pub fn new(potential_v: Vec<f64>, current_a: Vec<f64>) -> Self {
        Self { potential_v, current_a }
    }

    pub fn len(&self) -> usize {
        self.potential_v.len().min(self.current_a.len())
    }

    /// Find anodic (positive current) peaks.
    pub fn find_anodic_peaks(&self) -> Vec<CvPeak> {
        find_peaks_in(&self.potential_v, &self.current_a, true)
    }

    /// Find cathodic (negative current) peaks.
    pub fn find_cathodic_peaks(&self) -> Vec<CvPeak> {
        find_peaks_in(&self.potential_v, &self.current_a, false)
    }

    /// Peak separation: ΔEp = Epa - Epc (V).
    pub fn peak_separation(&self) -> Option<f64> {
        let anodic = self.find_anodic_peaks();
        let cathodic = self.find_cathodic_peaks();
        if anodic.is_empty() || cathodic.is_empty() { return None; }
        Some(anodic[0].potential_v - cathodic[0].potential_v)
    }

    /// Formal potential: E0' = (Epa + Epc) / 2.
    pub fn formal_potential(&self) -> Option<f64> {
        let anodic = self.find_anodic_peaks();
        let cathodic = self.find_cathodic_peaks();
        if anodic.is_empty() || cathodic.is_empty() { return None; }
        Some((anodic[0].potential_v + cathodic[0].potential_v) / 2.0)
    }

    /// Current ratio |ipa/ipc|.
    pub fn current_ratio(&self) -> Option<f64> {
        let anodic = self.find_anodic_peaks();
        let cathodic = self.find_cathodic_peaks();
        if anodic.is_empty() || cathodic.is_empty() { return None; }
        let ipc = cathodic[0].current_a.abs();
        if ipc < 1e-30 { return None; }
        Some(anodic[0].current_a.abs() / ipc)
    }

    /// Check if reversible: ΔEp ≈ 59/n mV and |ipa/ipc| ≈ 1.
    pub fn is_reversible(&self, n_electrons: u32) -> bool {
        let expected_dep = 0.059 / n_electrons as f64;
        if let Some(dep) = self.peak_separation() {
            if let Some(ratio) = self.current_ratio() {
                return dep.abs() < expected_dep * 1.5 && (ratio - 1.0).abs() < 0.2;
            }
        }
        false
    }
}

fn find_peaks_in(pot: &[f64], cur: &[f64], anodic: bool) -> Vec<CvPeak> {
    let n = pot.len().min(cur.len());
    if n < 3 { return vec![]; }
    let mut peaks = Vec::new();
    for i in 1..n - 1 {
        let is_peak = if anodic {
            cur[i] > 0.0 && cur[i] >= cur[i - 1] && cur[i] >= cur[i + 1]
        } else {
            cur[i] < 0.0 && cur[i] <= cur[i - 1] && cur[i] <= cur[i + 1]
        };
        if is_peak {
            peaks.push(CvPeak {
                potential_v: pot[i],
                current_a: cur[i],
                index: i,
                is_anodic: anodic,
            });
        }
    }
    // Sort by |current| descending
    peaks.sort_by(|a, b| b.current_a.abs().partial_cmp(&a.current_a.abs())
        .unwrap_or(std::cmp::Ordering::Equal));
    peaks
}

// ---------------------------------------------------------------------------
// 2. Randles-Sevcik Equation
// ---------------------------------------------------------------------------

/// Randles-Sevcik equation for reversible process (at 25°C):
/// ip = 0.4463 * n * F * A * C * sqrt(n * F * v * D / (R * T))
///
/// Returns peak current in Amperes.
/// n: electron count, a_cm2: electrode area, c_mol_cm3: concentration,
/// v_vs: scan rate (V/s), d_cm2_s: diffusion coefficient, t_k: temperature.
pub fn randles_sevcik(
    n: u32, a_cm2: f64, c_mol_cm3: f64, v_vs: f64, d_cm2_s: f64, t_k: f64,
) -> f64 {
    let nf = n as f64;
    0.4463 * nf * FARADAY * a_cm2 * c_mol_cm3
        * (nf * FARADAY * v_vs * d_cm2_s / (GAS_CONST * t_k)).sqrt()
}

/// Randles-Sevcik simplified at 25°C: ip = 2.69e5 * n^(3/2) * A * D^(1/2) * C * v^(1/2).
pub fn randles_sevcik_25c(n: u32, a_cm2: f64, c_mol_cm3: f64, v_vs: f64, d_cm2_s: f64) -> f64 {
    let nf = n as f64;
    2.69e5 * nf.powf(1.5) * a_cm2 * d_cm2_s.sqrt() * c_mol_cm3 * v_vs.sqrt()
}

/// Extract diffusion coefficient from peak current and scan rate.
/// D = (ip / (2.69e5 * n^(3/2) * A * C))^2 / v
pub fn diffusion_from_randles(
    ip: f64, n: u32, a_cm2: f64, c_mol_cm3: f64, v_vs: f64,
) -> f64 {
    let nf = n as f64;
    let denom = 2.69e5 * nf.powf(1.5) * a_cm2 * c_mol_cm3;
    if denom.abs() < 1e-30 || v_vs.abs() < 1e-30 { return 0.0; }
    (ip / denom).powi(2) / v_vs
}

// ---------------------------------------------------------------------------
// 3. Scan Rate Dependence
// ---------------------------------------------------------------------------

/// Analyze ip vs sqrt(v) to determine if diffusion-controlled.
pub fn scan_rate_analysis(
    scan_rates_vs: &[f64], peak_currents_a: &[f64],
) -> ScanRateResult {
    let n = scan_rates_vs.len().min(peak_currents_a.len());
    let sqrt_v: Vec<f64> = scan_rates_vs.iter().take(n).map(|v| v.sqrt()).collect();
    let (slope, intercept, r_squared) = linear_regression(&sqrt_v, &peak_currents_a[..n]);
    ScanRateResult {
        scan_rates_vs: scan_rates_vs[..n].to_vec(),
        peak_currents_a: peak_currents_a[..n].to_vec(),
        slope, intercept, r_squared,
        is_diffusion_controlled: r_squared > 0.95,
    }
}

fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64, f64) {
    let n = x.len().min(y.len()) as f64;
    if n < 2.0 { return (0.0, 0.0, 0.0); }
    let sx: f64 = x.iter().sum();
    let sy: f64 = y.iter().sum();
    let sxy: f64 = x.iter().zip(y.iter()).map(|(a, b)| a * b).sum();
    let sxx: f64 = x.iter().map(|a| a * a).sum();
    let denom = n * sxx - sx * sx;
    if denom.abs() < 1e-30 { return (0.0, 0.0, 0.0); }
    let slope = (n * sxy - sx * sy) / denom;
    let intercept = (sy - slope * sx) / n;
    let y_mean = sy / n;
    let ss_tot: f64 = y.iter().map(|yi| (yi - y_mean).powi(2)).sum();
    let ss_res: f64 = x.iter().zip(y.iter()).map(|(xi, yi)| (yi - (slope * xi + intercept)).powi(2)).sum();
    let r2 = if ss_tot > 1e-30 { 1.0 - ss_res / ss_tot } else { 0.0 };
    (slope, intercept, r2)
}

// ---------------------------------------------------------------------------
// 4. Cottrell Equation
// ---------------------------------------------------------------------------

/// Cottrell equation: I(t) = n*F*A*D^(1/2)*C / (π*t)^(1/2).
/// For chronoamperometry (potential step).
pub fn cottrell_current(n: u32, a_cm2: f64, d_cm2_s: f64, c_mol_cm3: f64, t_s: f64) -> f64 {
    if t_s <= 0.0 { return 0.0; }
    let nf = n as f64;
    nf * FARADAY * a_cm2 * d_cm2_s.sqrt() * c_mol_cm3 / (PI * t_s).sqrt()
}

/// Extract D from Cottrell plot (I vs 1/sqrt(t)).
pub fn diffusion_from_cottrell(slope: f64, n: u32, a_cm2: f64, c_mol_cm3: f64) -> f64 {
    let nf = n as f64;
    let denom = nf * FARADAY * a_cm2 * c_mol_cm3;
    if denom.abs() < 1e-30 { return 0.0; }
    let x = slope * PI.sqrt() / denom;
    x * x
}

// ---------------------------------------------------------------------------
// 5. Levich Equation (Rotating Disk Electrode)
// ---------------------------------------------------------------------------

/// Levich equation: I_L = 0.620 * n * F * A * D^(2/3) * ω^(1/2) * ν^(-1/6) * C.
/// ω in rad/s, ν = kinematic viscosity (cm²/s).
pub fn levich_current(
    n: u32, a_cm2: f64, d_cm2_s: f64, omega_rad_s: f64,
    nu_cm2_s: f64, c_mol_cm3: f64,
) -> f64 {
    let nf = n as f64;
    0.620 * nf * FARADAY * a_cm2 * d_cm2_s.powf(2.0 / 3.0)
        * omega_rad_s.sqrt() * nu_cm2_s.powf(-1.0 / 6.0) * c_mol_cm3
}

/// Convert RPM to rad/s.
pub fn rpm_to_rad_s(rpm: f64) -> f64 {
    rpm * 2.0 * PI / 60.0
}

// ---------------------------------------------------------------------------
// 6. Tafel Analysis
// ---------------------------------------------------------------------------

/// Tafel analysis: log|i| vs η (overpotential).
/// Returns Tafel slopes and exchange current density.
pub fn tafel_analysis(
    overpotential_v: &[f64], current_a: &[f64], temperature_k: f64,
) -> TafelResult {
    let n = overpotential_v.len().min(current_a.len());
    // Separate anodic (η > 0) and cathodic (η < 0) branches
    let mut anodic_eta = Vec::new();
    let mut anodic_log_i = Vec::new();
    let mut cathodic_eta = Vec::new();
    let mut cathodic_log_i = Vec::new();
    for i in 0..n {
        let eta = overpotential_v[i];
        let cur = current_a[i];
        if eta > 0.05 && cur > 0.0 {
            anodic_eta.push(eta);
            anodic_log_i.push(cur.log10());
        } else if eta < -0.05 && cur < 0.0 {
            cathodic_eta.push(eta);
            cathodic_log_i.push(cur.abs().log10());
        }
    }
    let (ba, int_a, _) = linear_regression(&anodic_eta, &anodic_log_i);
    let (bc, int_c, _) = linear_regression(&cathodic_eta, &cathodic_log_i);
    let ba_v = if ba.abs() > 1e-30 { 1.0 / ba } else { 0.0 };
    let bc_v = if bc.abs() > 1e-30 { -1.0 / bc } else { 0.0 };
    // Exchange current from y-intercept: log(i0) at η=0
    let log_i0 = if !anodic_log_i.is_empty() { int_a } else { int_c };
    let i0 = 10.0_f64.powf(log_i0);
    // Transfer coefficients: b = 2.303*R*T / (α*n*F), assume n=1
    let rt_f = GAS_CONST * temperature_k / FARADAY;
    let alpha_a = if ba_v.abs() > 1e-30 { 2.303 * rt_f / ba_v.abs() } else { 0.0 };
    let alpha_c = if bc_v.abs() > 1e-30 { 2.303 * rt_f / bc_v.abs() } else { 0.0 };
    // Corrosion potential: η where anodic = cathodic (approximate at η=0)
    TafelResult {
        anodic_slope_v_dec: ba_v,
        cathodic_slope_v_dec: bc_v,
        exchange_current_a: i0,
        corrosion_potential_v: 0.0,
        transfer_coeff_anodic: alpha_a,
        transfer_coeff_cathodic: alpha_c,
    }
}

/// Butler-Volmer equation: i = i0 * [exp(αa*F*η/(R*T)) - exp(-αc*F*η/(R*T))].
pub fn butler_volmer(i0: f64, alpha_a: f64, alpha_c: f64, eta_v: f64, t_k: f64) -> f64 {
    let f_rt = FARADAY / (GAS_CONST * t_k);
    i0 * ((alpha_a * f_rt * eta_v).exp() - (-alpha_c * f_rt * eta_v).exp())
}

// ---------------------------------------------------------------------------
// 7. Nernst Equation
// ---------------------------------------------------------------------------

/// Nernst equation: E = E0 + (R*T/(n*F)) * ln([Ox]/[Red]).
pub fn nernst_potential(e0_v: f64, n: u32, t_k: f64, ox_red_ratio: f64) -> f64 {
    if ox_red_ratio <= 0.0 { return e0_v; }
    e0_v + (GAS_CONST * t_k) / (n as f64 * FARADAY) * ox_red_ratio.ln()
}

// ---------------------------------------------------------------------------
// 8. Sweep Analyzer
// ---------------------------------------------------------------------------

/// Main sweep analyzer combining CV analysis steps.
pub struct SweepAnalyzer {
    pub temperature_k: f64,
    pub electrode_area_cm2: f64,
    pub n_electrons: u32,
    pub anodic_peaks: Vec<CvPeak>,
    pub cathodic_peaks: Vec<CvPeak>,
}

impl SweepAnalyzer {
    pub fn new(n_electrons: u32, area_cm2: f64, temp_k: f64) -> Self {
        Self {
            temperature_k: temp_k,
            electrode_area_cm2: area_cm2,
            n_electrons: n_electrons,
            anodic_peaks: vec![],
            cathodic_peaks: vec![],
        }
    }

    /// Analyze a voltammogram.
    pub fn analyze(&mut self, cv: &Voltammogram) {
        self.anodic_peaks = cv.find_anodic_peaks();
        self.cathodic_peaks = cv.find_cathodic_peaks();
    }

    /// Peak separation.
    pub fn delta_ep(&self) -> Option<f64> {
        if self.anodic_peaks.is_empty() || self.cathodic_peaks.is_empty() { return None; }
        Some(self.anodic_peaks[0].potential_v - self.cathodic_peaks[0].potential_v)
    }

    /// Formal potential.
    pub fn e_formal(&self) -> Option<f64> {
        if self.anodic_peaks.is_empty() || self.cathodic_peaks.is_empty() { return None; }
        Some((self.anodic_peaks[0].potential_v + self.cathodic_peaks[0].potential_v) / 2.0)
    }

    /// Estimate D from peak current and scan rate using Randles-Sevcik.
    pub fn estimate_d(&self, ip_a: f64, c_mol_cm3: f64, v_vs: f64) -> f64 {
        diffusion_from_randles(ip_a, self.n_electrons, self.electrode_area_cm2, c_mol_cm3, v_vs)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

fn approx_eq(a: f64, b: f64, tol: f64) -> bool { (a - b).abs() < tol }

#[cfg(test)]
mod tests {
    use super::*;

    fn make_reversible_cv(e0: f64, ip: f64, n_points: usize) -> Voltammogram {
        let dep = 0.059; // 59 mV for n=1
        let epa = e0 + dep / 2.0;
        let epc = e0 - dep / 2.0;
        let mut pot = Vec::with_capacity(n_points);
        let mut cur = Vec::with_capacity(n_points);
        // Forward scan: -0.5 to 0.5 V
        let half = n_points / 2;
        for i in 0..half {
            let e = -0.5 + i as f64 / half as f64;
            pot.push(e);
            let sigma = 0.03;
            cur.push(ip * (-0.5 * ((e - epa) / sigma).powi(2)).exp()
                     - ip * 0.1 * (-0.5 * ((e - epc) / sigma).powi(2)).exp());
        }
        // Reverse scan: 0.5 to -0.5 V
        for i in 0..half {
            let e = 0.5 - i as f64 / half as f64;
            pot.push(e);
            let sigma = 0.03;
            cur.push(ip * 0.1 * (-0.5 * ((e - epa) / sigma).powi(2)).exp()
                     - ip * (-0.5 * ((e - epc) / sigma).powi(2)).exp());
        }
        Voltammogram::new(pot, cur)
    }

    #[test]
    fn test_randles_sevcik_25c() {
        // Typical values: n=1, A=0.07 cm², D=7e-6 cm²/s, C=1e-6 mol/cm³, v=0.1 V/s
        let ip = randles_sevcik_25c(1, 0.07, 1e-6, 0.1, 7e-6);
        assert!(ip > 0.0);
        assert!(ip < 1e-3); // microamp range
    }

    #[test]
    fn test_randles_sevcik_full() {
        let ip = randles_sevcik(1, 0.07, 1e-6, 0.1, 7e-6, 298.15);
        let ip_25c = randles_sevcik_25c(1, 0.07, 1e-6, 0.1, 7e-6);
        assert!(approx_eq(ip, ip_25c, ip_25c * 0.1));
    }

    #[test]
    fn test_randles_scan_rate_dependence() {
        // ip proportional to sqrt(v)
        let ip1 = randles_sevcik_25c(1, 0.07, 1e-6, 0.1, 7e-6);
        let ip2 = randles_sevcik_25c(1, 0.07, 1e-6, 0.4, 7e-6);
        assert!(approx_eq(ip2 / ip1, 2.0, 0.01)); // sqrt(0.4/0.1) = 2
    }

    #[test]
    fn test_diffusion_from_randles() {
        let d_true = 7e-6;
        let ip = randles_sevcik_25c(1, 0.07, 1e-6, 0.1, d_true);
        let d_calc = diffusion_from_randles(ip, 1, 0.07, 1e-6, 0.1);
        assert!(approx_eq(d_calc, d_true, d_true * 0.01));
    }

    #[test]
    fn test_cottrell_current() {
        let i = cottrell_current(1, 0.07, 7e-6, 1e-6, 1.0);
        assert!(i > 0.0);
    }

    #[test]
    fn test_cottrell_zero_time() {
        let i = cottrell_current(1, 0.07, 7e-6, 1e-6, 0.0);
        assert!(approx_eq(i, 0.0, 1e-10));
    }

    #[test]
    fn test_cottrell_decreases() {
        let i1 = cottrell_current(1, 0.07, 7e-6, 1e-6, 0.1);
        let i2 = cottrell_current(1, 0.07, 7e-6, 1e-6, 1.0);
        assert!(i1 > i2);
    }

    #[test]
    fn test_levich_current() {
        let omega = rpm_to_rad_s(1600.0);
        let il = levich_current(1, 0.196, 7e-6, omega, 0.01, 1e-6);
        assert!(il > 0.0);
    }

    #[test]
    fn test_rpm_to_rad_s() {
        let w = rpm_to_rad_s(60.0);
        assert!(approx_eq(w, 2.0 * PI, 0.001));
    }

    #[test]
    fn test_butler_volmer_zero_eta() {
        let i = butler_volmer(1e-6, 0.5, 0.5, 0.0, 298.15);
        assert!(approx_eq(i, 0.0, 1e-10));
    }

    #[test]
    fn test_butler_volmer_positive_eta() {
        let i = butler_volmer(1e-6, 0.5, 0.5, 0.1, 298.15);
        assert!(i > 0.0);
    }

    #[test]
    fn test_butler_volmer_negative_eta() {
        let i = butler_volmer(1e-6, 0.5, 0.5, -0.1, 298.15);
        assert!(i < 0.0);
    }

    #[test]
    fn test_nernst_at_standard() {
        let e = nernst_potential(0.0, 1, 298.15, 1.0);
        assert!(approx_eq(e, 0.0, 1e-10));
    }

    #[test]
    fn test_nernst_ratio_10() {
        let e = nernst_potential(0.0, 1, 298.15, 10.0);
        // RT/F * ln(10) at 25°C ≈ 0.02569 * 2.303 = 0.0592 V
        assert!(approx_eq(e, 0.0592, 0.001));
    }

    #[test]
    fn test_find_anodic_peaks() {
        let cv = make_reversible_cv(0.0, 1e-5, 200);
        let peaks = cv.find_anodic_peaks();
        assert!(!peaks.is_empty());
        assert!(peaks[0].is_anodic);
    }

    #[test]
    fn test_find_cathodic_peaks() {
        let cv = make_reversible_cv(0.0, 1e-5, 200);
        let peaks = cv.find_cathodic_peaks();
        assert!(!peaks.is_empty());
        assert!(!peaks[0].is_anodic);
    }

    #[test]
    fn test_peak_separation() {
        let cv = make_reversible_cv(0.0, 1e-5, 200);
        let dep = cv.peak_separation();
        assert!(dep.is_some());
        assert!(dep.unwrap().abs() < 0.2);
    }

    #[test]
    fn test_formal_potential() {
        let cv = make_reversible_cv(0.25, 1e-5, 200);
        let e0 = cv.formal_potential();
        assert!(e0.is_some());
    }

    #[test]
    fn test_scan_rate_analysis() {
        let rates = vec![0.01, 0.025, 0.05, 0.1, 0.2];
        let currents: Vec<f64> = rates.iter().map(|v| randles_sevcik_25c(1, 0.07, 1e-6, *v, 7e-6)).collect();
        let result = scan_rate_analysis(&rates, &currents);
        assert!(result.r_squared > 0.99);
        assert!(result.is_diffusion_controlled);
    }

    #[test]
    fn test_tafel_analysis() {
        let mut eta = Vec::new();
        let mut cur = Vec::new();
        let i0 = 1e-6;
        for i in -20..=20 {
            let e = i as f64 * 0.01;
            eta.push(e);
            cur.push(butler_volmer(i0, 0.5, 0.5, e, 298.15));
        }
        let result = tafel_analysis(&eta, &cur, 298.15);
        assert!(result.exchange_current_a > 0.0);
    }

    #[test]
    fn test_sweep_analyzer_basic() {
        let cv = make_reversible_cv(0.0, 1e-5, 200);
        let mut analyzer = SweepAnalyzer::new(1, 0.07, 298.15);
        analyzer.analyze(&cv);
        assert!(!analyzer.anodic_peaks.is_empty());
        assert!(!analyzer.cathodic_peaks.is_empty());
    }

    #[test]
    fn test_sweep_analyzer_delta_ep() {
        let cv = make_reversible_cv(0.0, 1e-5, 200);
        let mut analyzer = SweepAnalyzer::new(1, 0.07, 298.15);
        analyzer.analyze(&cv);
        let dep = analyzer.delta_ep();
        assert!(dep.is_some());
    }

    #[test]
    fn test_sweep_analyzer_e_formal() {
        let cv = make_reversible_cv(0.0, 1e-5, 200);
        let mut analyzer = SweepAnalyzer::new(1, 0.07, 298.15);
        analyzer.analyze(&cv);
        let e0 = analyzer.e_formal();
        assert!(e0.is_some());
    }

    #[test]
    fn test_sweep_analyzer_estimate_d() {
        let analyzer = SweepAnalyzer::new(1, 0.07, 298.15);
        let ip = randles_sevcik_25c(1, 0.07, 1e-6, 0.1, 7e-6);
        let d = analyzer.estimate_d(ip, 1e-6, 0.1);
        assert!(approx_eq(d, 7e-6, 1e-6));
    }

    #[test]
    fn test_voltammogram_len() {
        let cv = Voltammogram::new(vec![0.0, 0.1, 0.2], vec![1e-6, 2e-6]);
        assert_eq!(cv.len(), 2);
    }

    #[test]
    fn test_diffusion_from_cottrell() {
        let d_true = 7e-6_f64;
        let slope = 1.0 * FARADAY * 0.07 * d_true.sqrt() * 1e-6 / PI.sqrt();
        let d = diffusion_from_cottrell(slope, 1, 0.07, 1e-6);
        assert!(approx_eq(d, d_true, d_true * 0.01));
    }

    #[test]
    fn test_levich_increases_with_rotation() {
        let il1 = levich_current(1, 0.196, 7e-6, rpm_to_rad_s(400.0), 0.01, 1e-6);
        let il2 = levich_current(1, 0.196, 7e-6, rpm_to_rad_s(1600.0), 0.01, 1e-6);
        assert!(il2 > il1);
        assert!(approx_eq(il2 / il1, 2.0, 0.01)); // sqrt(1600/400)=2
    }

    #[test]
    fn test_nernst_negative_ratio() {
        let e = nernst_potential(0.0, 1, 298.15, -1.0);
        assert!(approx_eq(e, 0.0, 1e-10)); // returns e0 for invalid ratio
    }

    #[test]
    fn test_current_ratio() {
        let cv = make_reversible_cv(0.0, 1e-5, 200);
        let ratio = cv.current_ratio();
        assert!(ratio.is_some());
    }
}
