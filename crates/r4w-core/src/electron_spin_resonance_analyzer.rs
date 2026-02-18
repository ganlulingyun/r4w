// trace:FR-ESR-ANAL | ai:claude
//! # Electron Spin Resonance Analyzer
//!
//! Advanced ESR/EPR analysis beyond the basic processor: multi-frequency ESR,
//! pulsed ESR (echo-detected, ESEEM, HYSCORE), spin-labeling distance measurement,
//! DEER/PELDOR analysis, and spectral simulation with anisotropic parameters.
//!
//! ## Physics Background
//!
//! - **Pulsed ESR**: π/2–τ–π echo sequence, echo amplitude vs τ gives T2
//! - **ESEEM**: electron spin echo envelope modulation from nuclear hyperfine
//! - **DEER/PELDOR**: dipolar coupling → inter-spin distance r ∝ 1/ν_dd^(1/3)
//! - **Spin Hamiltonian**: H = β_e * B · g · S + S · A · I + S · D · S
//! - **g-tensor anisotropy**: gx, gy, gz (powder pattern from random orientations)

use std::f64::consts::PI;

/// Planck constant in J*s.
pub const PLANCK_H: f64 = 6.626e-34;
/// Bohr magneton in J/T.
pub const BOHR_MAGNETON: f64 = 9.274e-24;
/// Nuclear magneton in J/T.
pub const NUCLEAR_MAGNETON: f64 = 5.051e-27;
/// Free electron g-factor.
pub const GE_FREE: f64 = 2.002319;
/// Boltzmann constant in J/K.
pub const K_BOLTZMANN: f64 = 1.381e-23;

// ---------------------------------------------------------------------------
// 1. g-Tensor and Anisotropy
// ---------------------------------------------------------------------------

/// Anisotropic g-tensor (orthorhombic symmetry).
#[derive(Debug, Clone, Copy)]
pub struct GTensor {
    pub gx: f64,
    pub gy: f64,
    pub gz: f64,
}

impl GTensor {
    /// Create isotropic g-tensor.
    pub fn isotropic(g: f64) -> Self {
        Self { gx: g, gy: g, gz: g }
    }

    /// Create axial g-tensor (gx = gy = g_perp, gz = g_par).
    pub fn axial(g_perp: f64, g_par: f64) -> Self {
        Self { gx: g_perp, gy: g_perp, gz: g_par }
    }

    /// Isotropic g-value: g_iso = (gx + gy + gz) / 3.
    pub fn g_iso(&self) -> f64 {
        (self.gx + self.gy + self.gz) / 3.0
    }

    /// Effective g for given polar angle theta.
    pub fn g_eff(&self, theta: f64, phi: f64) -> f64 {
        let st: f64 = theta.sin();
        let ct: f64 = theta.cos();
        let sp: f64 = phi.sin();
        let cp: f64 = phi.cos();
        let gx2: f64 = self.gx * self.gx * st * st * cp * cp;
        let gy2: f64 = self.gy * self.gy * st * st * sp * sp;
        let gz2: f64 = self.gz * self.gz * ct * ct;
        (gx2 + gy2 + gz2).sqrt()
    }

    /// Resonant field (mT) for given angle.
    pub fn resonant_field_mt(&self, freq_ghz: f64, theta: f64, phi: f64) -> f64 {
        let g: f64 = self.g_eff(theta, phi);
        if g <= 0.0 { return 0.0; }
        PLANCK_H * freq_ghz * 1e9 / (g * BOHR_MAGNETON) * 1000.0
    }
}

/// Simulate powder pattern spectrum for anisotropic g-tensor.
pub fn simulate_powder_pattern(
    g_tensor: &GTensor,
    freq_ghz: f64,
    field_start_mt: f64,
    field_end_mt: f64,
    n_field_points: usize,
    n_theta: usize,
    n_phi: usize,
    linewidth_mt: f64,
) -> (Vec<f64>, Vec<f64>) {
    let field_step: f64 = (field_end_mt - field_start_mt) / (n_field_points as f64 - 1.0);
    let mut fields: Vec<f64> = Vec::with_capacity(n_field_points);
    let mut spectrum: Vec<f64> = vec![0.0; n_field_points];
    for i in 0..n_field_points {
        fields.push(field_start_mt + i as f64 * field_step);
    }
    let theta_step: f64 = PI / n_theta as f64;
    let phi_step: f64 = 2.0 * PI / n_phi as f64;
    for it in 0..n_theta {
        let theta: f64 = (it as f64 + 0.5) * theta_step;
        let weight: f64 = theta.sin(); // solid angle weighting
        for ip in 0..n_phi {
            let phi: f64 = ip as f64 * phi_step;
            let b_res: f64 = g_tensor.resonant_field_mt(freq_ghz, theta, phi);
            // Add Gaussian line at b_res
            for (k, &b) in fields.iter().enumerate() {
                let z: f64 = (b - b_res) / linewidth_mt;
                spectrum[k] += weight * (-0.5 * z * z).exp();
            }
        }
    }
    // Normalize
    let max_val: f64 = spectrum.iter().cloned().fold(0.0_f64, f64::max);
    if max_val > 0.0 {
        for v in &mut spectrum { *v /= max_val; }
    }
    (fields, spectrum)
}

// ---------------------------------------------------------------------------
// 2. Pulsed ESR: Echo Decay
// ---------------------------------------------------------------------------

/// Single exponential echo decay: V(τ) = V0 * exp(-2τ/T2)
pub fn echo_decay(tau: f64, v0: f64, t2: f64) -> f64 {
    v0 * (-2.0 * tau / t2).exp()
}

/// Stretched exponential: V(τ) = V0 * exp(-(2τ/Tm)^x)
pub fn stretched_exponential(tau: f64, v0: f64, tm: f64, x: f64) -> f64 {
    v0 * (-(2.0 * tau / tm).powf(x)).exp()
}

/// Fit T2 from echo decay data (linearized: ln(V) vs 2τ).
pub fn fit_t2(taus: &[f64], amplitudes: &[f64]) -> (f64, f64) {
    let n: usize = taus.len().min(amplitudes.len());
    if n < 2 { return (1.0, 1.0); }
    let mut sx: f64 = 0.0;
    let mut sy: f64 = 0.0;
    let mut sxy: f64 = 0.0;
    let mut sxx: f64 = 0.0;
    let mut count: f64 = 0.0;
    for i in 0..n {
        if amplitudes[i] <= 0.0 { continue; }
        let x: f64 = 2.0 * taus[i];
        let y: f64 = amplitudes[i].ln();
        sx += x;
        sy += y;
        sxy += x * y;
        sxx += x * x;
        count += 1.0;
    }
    if count < 2.0 { return (1.0, 1.0); }
    let denom: f64 = count * sxx - sx * sx;
    if denom.abs() < 1e-30 { return (1.0, 1.0); }
    let slope: f64 = (count * sxy - sx * sy) / denom;
    let intercept: f64 = (sy - slope * sx) / count;
    let t2: f64 = -1.0 / slope;
    let v0: f64 = intercept.exp();
    (v0, t2)
}

// ---------------------------------------------------------------------------
// 3. DEER/PELDOR Distance Measurement
// ---------------------------------------------------------------------------

/// Dipolar frequency from inter-spin distance.
/// ν_dd = 52.04 / r³ MHz (r in nm)
pub fn dipolar_frequency_mhz(distance_nm: f64) -> f64 {
    if distance_nm <= 0.0 { return f64::INFINITY; }
    52.04 / (distance_nm * distance_nm * distance_nm)
}

/// Inter-spin distance from dipolar frequency.
/// r = (52.04 / ν_dd)^(1/3) nm
pub fn distance_from_dipolar_freq(freq_mhz: f64) -> f64 {
    if freq_mhz <= 0.0 { return f64::INFINITY; }
    (52.04 / freq_mhz).powf(1.0 / 3.0)
}

/// DEER time-domain signal for a single distance.
/// V(t) = 1 - λ(1 - cos(2π * ν_dd * t))
pub fn deer_signal_single(t_us: f64, distance_nm: f64, modulation_depth: f64) -> f64 {
    let freq: f64 = dipolar_frequency_mhz(distance_nm);
    1.0 - modulation_depth * (1.0 - (2.0 * PI * freq * t_us).cos())
}

/// Simulate DEER trace for a Gaussian distance distribution.
pub fn deer_signal_gaussian(
    t_values_us: &[f64],
    mean_dist_nm: f64,
    sigma_nm: f64,
    modulation_depth: f64,
    n_dist_points: usize,
) -> Vec<f64> {
    let dist_min: f64 = (mean_dist_nm - 4.0 * sigma_nm).max(0.5);
    let dist_max: f64 = mean_dist_nm + 4.0 * sigma_nm;
    let dist_step: f64 = (dist_max - dist_min) / (n_dist_points as f64 - 1.0);
    let mut output: Vec<f64> = vec![0.0; t_values_us.len()];
    let mut total_weight: f64 = 0.0;
    for k in 0..n_dist_points {
        let r: f64 = dist_min + k as f64 * dist_step;
        let z: f64 = (r - mean_dist_nm) / sigma_nm;
        let w: f64 = (-0.5 * z * z).exp();
        total_weight += w;
        for (i, &t) in t_values_us.iter().enumerate() {
            output[i] += w * deer_signal_single(t, r, modulation_depth);
        }
    }
    if total_weight > 0.0 {
        for v in &mut output { *v /= total_weight; }
    }
    output
}

// ---------------------------------------------------------------------------
// 4. ESEEM (Electron Spin Echo Envelope Modulation)
// ---------------------------------------------------------------------------

/// ESEEM modulation for weakly coupled nucleus.
/// k(τ) = 1 - k/2 * [1 - cos(ω_α τ)] * [1 - cos(ω_β τ)]
/// where ω_α,β = 2π(ν_I ± A/2), k = modulation depth
pub fn eseem_modulation(
    tau: f64,
    nuclear_freq_mhz: f64,
    hyperfine_mhz: f64,
    mod_depth: f64,
) -> f64 {
    let omega_alpha: f64 = 2.0 * PI * (nuclear_freq_mhz + hyperfine_mhz / 2.0);
    let omega_beta: f64 = 2.0 * PI * (nuclear_freq_mhz - hyperfine_mhz / 2.0);
    1.0 - mod_depth / 2.0
        * (1.0 - (omega_alpha * tau).cos())
        * (1.0 - (omega_beta * tau).cos())
}

// ---------------------------------------------------------------------------
// 5. Spin Relaxation
// ---------------------------------------------------------------------------

/// Inversion recovery: Mz(t) = M0 * (1 - 2 * exp(-t/T1))
pub fn inversion_recovery(t: f64, m0: f64, t1: f64) -> f64 {
    m0 * (1.0 - 2.0 * (-t / t1).exp())
}

/// Saturation recovery: Mz(t) = M0 * (1 - exp(-t/T1))
pub fn saturation_recovery(t: f64, m0: f64, t1: f64) -> f64 {
    m0 * (1.0 - (-t / t1).exp())
}

/// Fit T1 from inversion recovery data.
pub fn fit_t1_inversion(times: &[f64], mz_values: &[f64]) -> (f64, f64) {
    let n: usize = times.len().min(mz_values.len());
    if n < 3 { return (1.0, 1.0); }
    // Estimate M0 from last point
    let m0_est: f64 = mz_values[n - 1].abs().max(1e-10);
    // Linearize: ln((M0 - Mz)/(2*M0)) = -t/T1
    let mut sx: f64 = 0.0;
    let mut sy: f64 = 0.0;
    let mut sxy: f64 = 0.0;
    let mut sxx: f64 = 0.0;
    let mut count: f64 = 0.0;
    for i in 0..n {
        let ratio: f64 = (m0_est - mz_values[i]) / (2.0 * m0_est);
        if ratio <= 0.0 { continue; }
        let x: f64 = times[i];
        let y: f64 = ratio.ln();
        sx += x;
        sy += y;
        sxy += x * y;
        sxx += x * x;
        count += 1.0;
    }
    if count < 2.0 { return (m0_est, 1.0); }
    let denom: f64 = count * sxx - sx * sx;
    if denom.abs() < 1e-30 { return (m0_est, 1.0); }
    let slope: f64 = (count * sxy - sx * sy) / denom;
    let t1: f64 = -1.0 / slope;
    (m0_est, t1)
}

// ---------------------------------------------------------------------------
// 6. Spin Counting
// ---------------------------------------------------------------------------

/// Boltzmann polarization: P = tanh(g * μ_B * B / (2 * k_B * T))
pub fn boltzmann_polarization(g: f64, field_t: f64, temperature_k: f64) -> f64 {
    if temperature_k <= 0.0 { return 1.0; }
    let arg: f64 = g * BOHR_MAGNETON * field_t / (2.0 * K_BOLTZMANN * temperature_k);
    arg.tanh()
}

/// Number of spins from double integral.
/// N_spins = DI_sample / DI_reference * N_reference
pub fn spin_count(
    di_sample: f64,
    di_reference: f64,
    n_reference: f64,
) -> f64 {
    if di_reference.abs() < 1e-30 { return 0.0; }
    di_sample / di_reference * n_reference
}

/// Double integral of first-derivative spectrum.
pub fn double_integral(spectrum: &[f64], field_step: f64) -> f64 {
    let n: usize = spectrum.len();
    if n < 2 { return 0.0; }
    // First integral
    let mut first_int: Vec<f64> = vec![0.0; n];
    for i in 1..n {
        first_int[i] = first_int[i - 1] + spectrum[i] * field_step;
    }
    // Second integral
    let mut di: f64 = 0.0;
    for i in 1..n {
        di += first_int[i] * field_step;
    }
    di
}

// ---------------------------------------------------------------------------
// 7. EsrAnalyzer Orchestrator
// ---------------------------------------------------------------------------

/// ESR analysis orchestrator.
#[derive(Debug, Clone)]
pub struct EsrAnalyzer {
    pub g_tensor: GTensor,
    pub frequency_ghz: f64,
    pub t1_us: Option<f64>,
    pub t2_us: Option<f64>,
}

impl EsrAnalyzer {
    /// Create analyzer for X-band (9.5 GHz).
    pub fn x_band() -> Self {
        Self {
            g_tensor: GTensor::isotropic(GE_FREE),
            frequency_ghz: 9.5,
            t1_us: None,
            t2_us: None,
        }
    }

    /// Create analyzer for Q-band (34 GHz).
    pub fn q_band() -> Self {
        Self {
            g_tensor: GTensor::isotropic(GE_FREE),
            frequency_ghz: 34.0,
            t1_us: None,
            t2_us: None,
        }
    }

    /// Create analyzer for W-band (94 GHz).
    pub fn w_band() -> Self {
        Self {
            g_tensor: GTensor::isotropic(GE_FREE),
            frequency_ghz: 94.0,
            t1_us: None,
            t2_us: None,
        }
    }

    /// Set g-tensor.
    pub fn set_g_tensor(&mut self, gt: GTensor) {
        self.g_tensor = gt;
    }

    /// Resonant field for isotropic case.
    pub fn resonant_field_mt(&self) -> f64 {
        self.g_tensor.resonant_field_mt(self.frequency_ghz, 0.0, 0.0)
    }

    /// Fit T2 from echo decay and store.
    pub fn fit_t2(&mut self, taus: &[f64], amps: &[f64]) -> f64 {
        let (_, t2) = fit_t2(taus, amps);
        self.t2_us = Some(t2);
        t2
    }

    /// Fit T1 from inversion recovery and store.
    pub fn fit_t1(&mut self, times: &[f64], mz: &[f64]) -> f64 {
        let (_, t1) = fit_t1_inversion(times, mz);
        self.t1_us = Some(t1);
        t1
    }

    /// Simulate powder pattern.
    pub fn simulate_powder(&self, field_range_mt: (f64, f64), n_points: usize, linewidth_mt: f64) -> (Vec<f64>, Vec<f64>) {
        simulate_powder_pattern(
            &self.g_tensor,
            self.frequency_ghz,
            field_range_mt.0,
            field_range_mt.1,
            n_points,
            30,
            30,
            linewidth_mt,
        )
    }
}

impl Default for EsrAnalyzer {
    fn default() -> Self {
        Self::x_band()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn test_g_tensor_isotropic() {
        let g = GTensor::isotropic(2.0);
        assert!(approx_eq(g.g_iso(), 2.0, 0.001));
    }

    #[test]
    fn test_g_tensor_axial() {
        let g = GTensor::axial(2.05, 2.00);
        assert!(approx_eq(g.g_iso(), (2.05 + 2.05 + 2.0) / 3.0, 0.001));
    }

    #[test]
    fn test_g_eff_isotropic() {
        let g = GTensor::isotropic(2.0);
        assert!(approx_eq(g.g_eff(0.0, 0.0), 2.0, 0.001));
        assert!(approx_eq(g.g_eff(PI / 2.0, 0.0), 2.0, 0.001));
    }

    #[test]
    fn test_g_eff_axial() {
        let g = GTensor::axial(2.05, 2.00);
        // At theta=0 (z-axis): g_eff = gz = 2.00
        assert!(approx_eq(g.g_eff(0.0, 0.0), 2.00, 0.001));
        // At theta=pi/2, phi=0 (x-axis): g_eff = gx = 2.05
        assert!(approx_eq(g.g_eff(PI / 2.0, 0.0), 2.05, 0.001));
    }

    #[test]
    fn test_resonant_field() {
        let g = GTensor::isotropic(GE_FREE);
        let b: f64 = g.resonant_field_mt(9.5, 0.0, 0.0);
        // At 9.5 GHz, free electron: ~339 mT
        assert!(approx_eq(b, 339.0, 2.0));
    }

    #[test]
    fn test_powder_pattern() {
        let g = GTensor::axial(2.10, 2.00);
        let (fields, spectrum) = simulate_powder_pattern(
            &g, 9.5, 320.0, 345.0, 100, 20, 20, 0.5,
        );
        assert_eq!(fields.len(), 100);
        assert_eq!(spectrum.len(), 100);
        let max_val: f64 = spectrum.iter().cloned().fold(0.0_f64, f64::max);
        assert!(approx_eq(max_val, 1.0, 0.01));
    }

    #[test]
    fn test_echo_decay() {
        let v: f64 = echo_decay(0.0, 1.0, 2.0);
        assert!(approx_eq(v, 1.0, 0.001));
        let v2: f64 = echo_decay(1.0, 1.0, 2.0);
        // exp(-2/2) = exp(-1) ≈ 0.368
        assert!(approx_eq(v2, (-1.0_f64).exp(), 0.01));
    }

    #[test]
    fn test_stretched_exponential() {
        let v: f64 = stretched_exponential(0.0, 1.0, 2.0, 1.0);
        assert!(approx_eq(v, 1.0, 0.001));
    }

    #[test]
    fn test_fit_t2() {
        let t2_true: f64 = 3.0;
        let v0_true: f64 = 1.0;
        let taus: Vec<f64> = (0..30).map(|i| i as f64 * 0.2).collect();
        let amps: Vec<f64> = taus.iter()
            .map(|&tau| v0_true * (-2.0 * tau / t2_true).exp())
            .collect();
        let (v0, t2) = fit_t2(&taus, &amps);
        assert!(approx_eq(v0, v0_true, 0.05));
        assert!(approx_eq(t2, t2_true, 0.1));
    }

    #[test]
    fn test_dipolar_frequency() {
        let freq: f64 = dipolar_frequency_mhz(3.0);
        // 52.04 / 27 = 1.927 MHz
        assert!(approx_eq(freq, 1.927, 0.01));
    }

    #[test]
    fn test_distance_from_dipolar() {
        let d: f64 = distance_from_dipolar_freq(1.927);
        assert!(approx_eq(d, 3.0, 0.1));
    }

    #[test]
    fn test_deer_roundtrip() {
        let d: f64 = 4.0;
        let freq: f64 = dipolar_frequency_mhz(d);
        let d_back: f64 = distance_from_dipolar_freq(freq);
        assert!(approx_eq(d_back, d, 0.01));
    }

    #[test]
    fn test_deer_signal_single() {
        let v: f64 = deer_signal_single(0.0, 3.0, 0.3);
        assert!(approx_eq(v, 1.0, 0.001));
    }

    #[test]
    fn test_deer_signal_gaussian() {
        let times: Vec<f64> = (0..50).map(|i| i as f64 * 0.01).collect();
        let signal = deer_signal_gaussian(&times, 3.0, 0.3, 0.3, 50);
        assert!(approx_eq(signal[0], 1.0, 0.01));
        assert!(signal.len() == 50);
    }

    #[test]
    fn test_eseem_modulation() {
        let v: f64 = eseem_modulation(0.0, 14.0, 5.0, 0.5);
        assert!(approx_eq(v, 1.0, 0.001));
    }

    #[test]
    fn test_inversion_recovery() {
        let mz: f64 = inversion_recovery(0.0, 1.0, 1.0);
        assert!(approx_eq(mz, -1.0, 0.01)); // starts at -M0
        let mz2: f64 = inversion_recovery(100.0, 1.0, 1.0);
        assert!(approx_eq(mz2, 1.0, 0.01)); // recovers to M0
    }

    #[test]
    fn test_saturation_recovery() {
        let mz: f64 = saturation_recovery(0.0, 1.0, 1.0);
        assert!(approx_eq(mz, 0.0, 0.01)); // starts at 0
        let mz2: f64 = saturation_recovery(100.0, 1.0, 1.0);
        assert!(approx_eq(mz2, 1.0, 0.01)); // recovers to M0
    }

    #[test]
    fn test_fit_t1_inversion() {
        let t1_true: f64 = 5.0;
        let m0: f64 = 1.0;
        let times: Vec<f64> = (0..40).map(|i| i as f64 * 0.5).collect();
        let mz: Vec<f64> = times.iter()
            .map(|&t| m0 * (1.0 - 2.0 * (-t / t1_true).exp()))
            .collect();
        let (_, t1) = fit_t1_inversion(&times, &mz);
        assert!(approx_eq(t1, t1_true, 1.5));
    }

    #[test]
    fn test_boltzmann_polarization() {
        // Room temperature, X-band: very small polarization
        let p: f64 = boltzmann_polarization(2.0, 0.34, 300.0);
        assert!(p > 0.0 && p < 0.01);
    }

    #[test]
    fn test_boltzmann_low_temp() {
        // Very low temperature: high polarization
        let p: f64 = boltzmann_polarization(2.0, 10.0, 0.01);
        assert!(p > 0.99);
    }

    #[test]
    fn test_spin_count() {
        let n: f64 = spin_count(50.0, 100.0, 1e15);
        assert!(approx_eq(n, 5e14, 1e12));
    }

    #[test]
    fn test_double_integral() {
        // First derivative of a Lorentzian
        let n: usize = 100;
        let step: f64 = 0.1;
        let spectrum: Vec<f64> = (0..n).map(|i| {
            let x: f64 = (i as f64 - 50.0) * step;
            -2.0 * x / (1.0 + x * x).powi(2) // derivative of Lorentzian
        }).collect();
        let di: f64 = double_integral(&spectrum, step);
        assert!(di > 0.0); // Should be positive for absorption
    }

    #[test]
    fn test_esr_analyzer_x_band() {
        let analyzer = EsrAnalyzer::x_band();
        assert!(approx_eq(analyzer.frequency_ghz, 9.5, 0.01));
    }

    #[test]
    fn test_esr_analyzer_q_band() {
        let analyzer = EsrAnalyzer::q_band();
        assert!(approx_eq(analyzer.frequency_ghz, 34.0, 0.01));
    }

    #[test]
    fn test_esr_analyzer_w_band() {
        let analyzer = EsrAnalyzer::w_band();
        assert!(approx_eq(analyzer.frequency_ghz, 94.0, 0.01));
    }

    #[test]
    fn test_esr_analyzer_resonant_field() {
        let analyzer = EsrAnalyzer::x_band();
        let b: f64 = analyzer.resonant_field_mt();
        assert!(approx_eq(b, 339.0, 2.0));
    }

    #[test]
    fn test_esr_analyzer_fit_t2() {
        let mut analyzer = EsrAnalyzer::x_band();
        let taus: Vec<f64> = (0..20).map(|i| i as f64 * 0.1).collect();
        let amps: Vec<f64> = taus.iter().map(|&t| (-2.0 * t / 2.5).exp()).collect();
        let t2: f64 = analyzer.fit_t2(&taus, &amps);
        assert!(approx_eq(t2, 2.5, 0.2));
        assert!(analyzer.t2_us.is_some());
    }

    #[test]
    fn test_esr_analyzer_fit_t1() {
        let mut analyzer = EsrAnalyzer::x_band();
        let times: Vec<f64> = (0..30).map(|i| i as f64 * 0.5).collect();
        let mz: Vec<f64> = times.iter().map(|&t| 1.0 * (1.0 - 2.0 * (-t / 4.0).exp())).collect();
        let t1: f64 = analyzer.fit_t1(&times, &mz);
        assert!(approx_eq(t1, 4.0, 1.5));
        assert!(analyzer.t1_us.is_some());
    }

    #[test]
    fn test_esr_analyzer_default() {
        let analyzer = EsrAnalyzer::default();
        assert!(approx_eq(analyzer.frequency_ghz, 9.5, 0.01));
    }

    #[test]
    fn test_esr_analyzer_set_g_tensor() {
        let mut analyzer = EsrAnalyzer::x_band();
        analyzer.set_g_tensor(GTensor::axial(2.1, 2.0));
        assert!(approx_eq(analyzer.g_tensor.gx, 2.1, 0.001));
    }

    #[test]
    fn test_esr_analyzer_powder() {
        let mut analyzer = EsrAnalyzer::x_band();
        analyzer.set_g_tensor(GTensor::axial(2.10, 2.00));
        let (fields, spec) = analyzer.simulate_powder((320.0, 345.0), 50, 0.5);
        assert_eq!(fields.len(), 50);
        assert_eq!(spec.len(), 50);
    }

    #[test]
    fn test_dipolar_frequency_limits() {
        assert!(dipolar_frequency_mhz(0.0).is_infinite());
        let f: f64 = dipolar_frequency_mhz(10.0);
        assert!(f < 0.1); // Long distance, low frequency
    }

    #[test]
    fn test_distance_limits() {
        assert!(distance_from_dipolar_freq(0.0).is_infinite());
    }

    #[test]
    fn test_stretched_exp_x1() {
        // x=1 should equal regular exponential
        let v1: f64 = stretched_exponential(1.0, 1.0, 2.0, 1.0);
        let v2: f64 = echo_decay(1.0, 1.0, 2.0);
        assert!(approx_eq(v1, v2, 0.001));
    }

    #[test]
    fn test_eseem_max() {
        // At tau=0, modulation should be 1
        let v: f64 = eseem_modulation(0.0, 14.0, 3.0, 0.8);
        assert!(approx_eq(v, 1.0, 0.001));
    }

    #[test]
    fn test_spin_count_zero_ref() {
        let n: f64 = spin_count(50.0, 0.0, 1e15);
        assert!(approx_eq(n, 0.0, 0.001));
    }

    #[test]
    fn test_g_tensor_orthorhombic() {
        let g = GTensor { gx: 2.08, gy: 2.05, gz: 2.00 };
        let g_iso: f64 = g.g_iso();
        assert!(approx_eq(g_iso, (2.08 + 2.05 + 2.00) / 3.0, 0.001));
    }

    #[test]
    fn test_boltzmann_zero_field() {
        let p: f64 = boltzmann_polarization(2.0, 0.0, 300.0);
        assert!(approx_eq(p, 0.0, 0.001));
    }

    #[test]
    fn test_double_integral_constant() {
        // Constant spectrum (no signal) should give positive DI
        let spectrum: Vec<f64> = vec![1.0; 20];
        let di: f64 = double_integral(&spectrum, 0.1);
        assert!(di > 0.0);
    }
}
