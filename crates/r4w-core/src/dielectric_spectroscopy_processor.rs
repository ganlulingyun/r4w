//! # Dielectric Spectroscopy Processor
//!
//! Broadband dielectric spectroscopy (BDS) data analysis for measuring complex
//! permittivity epsilon*(omega) = epsilon'(omega) - i*epsilon''(omega) to characterize
//! relaxation processes, ionic conductivity, and electrode polarization in materials.
//!
//! ## Key Components
//!
//! - **DielectricSpectrum** - Complex permittivity storage and conversion
//! - **HavriliakNegami** - Broadened/asymmetric relaxation model fitting
//! - **DebyeRelaxation** - Simple single-relaxation-time model
//! - **ConductivityAnalysis** - DC/AC conductivity extraction and subtraction
//! - **ElectrodePolarization** - Low-frequency electrode effect detection/removal
//! - **RelaxationMap** - Temperature-frequency activation diagram (Arrhenius/VFT)
//! - **MixingRules** - Effective permittivity of composite materials
//! - **DielectricModulus** - Electric modulus formalism M*(omega)
//! - **DielectricSimulator** - Synthetic spectra generation
//! - **FrequencyAnalysis** - Spectral analysis utilities (peaks, Kramers-Kronig)
//!
//! ## Physics
//!
//! - epsilon*(omega) = epsilon'(omega) - i*epsilon''(omega)
//!   - Real part epsilon': energy storage (capacitive)
//!   - Imaginary part epsilon'': dielectric loss (dissipative)
//! - Debye: epsilon* = epsilon_inf + delta_eps / (1 + i*omega*tau)
//! - Havriliak-Negami: epsilon* = epsilon_inf + delta_eps / (1 + (i*omega*tau)^alpha)^beta
//! - DC conductivity contributes sigma_DC / (epsilon_0 * omega) to epsilon''
//! - VFT: f = f0 * exp(-B / (T - T0)) for super-Arrhenius dynamics

use std::f64::consts::PI;

// Vacuum permittivity in F/m
const EPS0: f64 = 8.854187817e-12;
// Boltzmann constant in eV/K
const KB_EV: f64 = 8.617333262e-5;

// ============================================================================
// DielectricSpectrum
// ============================================================================

/// Complex permittivity spectrum epsilon*(f) = epsilon'(f) - i*epsilon''(f)
#[derive(Debug, Clone)]
pub struct DielectricSpectrum {
    /// Frequency points in Hz
    pub frequency_hz: Vec<f64>,
    /// Real part of permittivity (epsilon')
    pub eps_real: Vec<f64>,
    /// Imaginary part of permittivity (epsilon'', positive convention for loss)
    pub eps_imag: Vec<f64>,
}

impl DielectricSpectrum {
    /// Create a new dielectric spectrum from frequency, real, and imaginary parts.
    pub fn new(frequency_hz: Vec<f64>, eps_real: Vec<f64>, eps_imag: Vec<f64>) -> Self {
        assert_eq!(frequency_hz.len(), eps_real.len());
        assert_eq!(frequency_hz.len(), eps_imag.len());
        assert!(!frequency_hz.is_empty());
        Self {
            frequency_hz,
            eps_real,
            eps_imag,
        }
    }

    /// Number of frequency points.
    pub fn len(&self) -> usize {
        self.frequency_hz.len()
    }

    /// Whether the spectrum is empty.
    pub fn is_empty(&self) -> bool {
        self.frequency_hz.is_empty()
    }

    /// Get permittivity at a given index: (epsilon', epsilon'').
    pub fn permittivity_at(&self, index: usize) -> (f64, f64) {
        (self.eps_real[index], self.eps_imag[index])
    }

    /// Loss tangent tan(delta) = epsilon'' / epsilon' at a given index.
    pub fn loss_tangent_at(&self, index: usize) -> f64 {
        if self.eps_real[index].abs() < 1e-30 {
            return 0.0;
        }
        self.eps_imag[index] / self.eps_real[index]
    }

    /// Convert to electric modulus M* = 1/epsilon*.
    pub fn to_modulus(&self) -> DielectricModulus {
        DielectricModulus::from_permittivity(self)
    }

    /// Convert to complex conductivity sigma* = i*omega*epsilon_0*epsilon*.
    /// sigma' = omega * epsilon_0 * epsilon''
    /// sigma'' = omega * epsilon_0 * epsilon'
    pub fn to_conductivity(&self, eps0: f64) -> ConductivitySpectrum {
        let n = self.len();
        let mut sigma_real = Vec::with_capacity(n);
        let mut sigma_imag = Vec::with_capacity(n);
        for i in 0..n {
            let omega = 2.0 * PI * self.frequency_hz[i];
            sigma_real.push(omega * eps0 * self.eps_imag[i]);
            sigma_imag.push(omega * eps0 * self.eps_real[i]);
        }
        ConductivitySpectrum {
            frequency_hz: self.frequency_hz.clone(),
            sigma_real,
            sigma_imag,
        }
    }

    /// Convert to impedance Z* for a parallel-plate sample.
    /// Z* = d / (i*omega*epsilon_0*epsilon* * A)
    /// Returns Vec<(Z_real, Z_imag)>.
    pub fn to_impedance(&self, area_m2: f64, thickness_m: f64) -> Vec<(f64, f64)> {
        let n = self.len();
        let mut result = Vec::with_capacity(n);
        for i in 0..n {
            let omega = 2.0 * PI * self.frequency_hz[i];
            // epsilon* = eps' - i*eps''
            // i*omega*epsilon_0*epsilon* = i*omega*eps0*(eps' - i*eps'')
            //   = omega*eps0*eps'' + i*omega*eps0*eps'
            let denom_re = omega * EPS0 * self.eps_imag[i];
            let denom_im = omega * EPS0 * self.eps_real[i];
            let denom_mag_sq = denom_re * denom_re + denom_im * denom_im;
            if denom_mag_sq < 1e-60 {
                result.push((f64::INFINITY, f64::INFINITY));
                continue;
            }
            let scale = thickness_m / (area_m2 * denom_mag_sq);
            // Z = (d/A) * conj(denom) / |denom|^2
            // = (d/A) * (denom_re - i*denom_im) / |denom|^2
            let z_re = scale * denom_re;
            let z_im = -scale * denom_im;
            result.push((z_re, z_im));
        }
        result
    }
}

// ============================================================================
// ConductivitySpectrum
// ============================================================================

/// Complex conductivity spectrum sigma*(f).
#[derive(Debug, Clone)]
pub struct ConductivitySpectrum {
    pub frequency_hz: Vec<f64>,
    /// Real part of conductivity (sigma') in S/m
    pub sigma_real: Vec<f64>,
    /// Imaginary part of conductivity (sigma'') in S/m
    pub sigma_imag: Vec<f64>,
}

// ============================================================================
// HavriliakNegami
// ============================================================================

/// Result of Havriliak-Negami fit.
#[derive(Debug, Clone)]
pub struct HnFitResult {
    pub eps_inf: f64,
    pub delta_eps: f64,
    pub tau_s: f64,
    pub alpha: f64,
    pub beta: f64,
    pub chi_squared: f64,
}

/// Havriliak-Negami relaxation model.
///
/// epsilon*(omega) = epsilon_inf + delta_eps / (1 + (i*omega*tau)^alpha)^beta
///
/// Special cases:
/// - Debye: alpha=1, beta=1
/// - Cole-Cole: beta=1
/// - Davidson-Cole: alpha=1
pub struct HavriliakNegami;

impl HavriliakNegami {
    /// Evaluate HN function at a single frequency.
    /// Returns (epsilon', epsilon'').
    ///
    /// epsilon*(omega) = epsilon_inf + delta_eps / (1 + (i*omega*tau)^alpha)^beta
    pub fn evaluate(
        frequency_hz: f64,
        eps_inf: f64,
        delta_eps: f64,
        tau_s: f64,
        alpha: f64,
        beta: f64,
    ) -> (f64, f64) {
        let omega = 2.0 * PI * frequency_hz;
        let wt = omega * tau_s;

        // (i*omega*tau)^alpha using polar form
        // i*omega*tau has magnitude omega*tau and angle pi/2
        // so (i*omega*tau)^alpha has magnitude (omega*tau)^alpha and angle alpha*pi/2
        let r_alpha = wt.powf(alpha);
        let theta_alpha = alpha * PI / 2.0;

        // 1 + (i*omega*tau)^alpha
        let sum_re = 1.0 + r_alpha * theta_alpha.cos();
        let sum_im = r_alpha * theta_alpha.sin();

        // Convert to polar for raising to power beta
        let sum_mag = (sum_re * sum_re + sum_im * sum_im).sqrt();
        let sum_angle = sum_im.atan2(sum_re);

        // (1 + (i*omega*tau)^alpha)^beta
        let denom_mag = sum_mag.powf(beta);
        let denom_angle = beta * sum_angle;

        // delta_eps / denom = (delta_eps / |denom|) * exp(-i * denom_angle)
        // eps* = eps' - i*eps'' convention:
        //   eps' contribution = (delta_eps/|denom|) * cos(denom_angle)
        //   eps'' (positive loss) = (delta_eps/|denom|) * sin(denom_angle)
        let inv_mag = delta_eps / denom_mag;
        let result_re = inv_mag * denom_angle.cos();
        let result_im = inv_mag * denom_angle.sin();

        (eps_inf + result_re, result_im)
    }

    /// Fit HN model to a spectrum using Levenberg-Marquardt-style optimization.
    pub fn fit(spectrum: &DielectricSpectrum) -> HnFitResult {
        let n = spectrum.len();
        if n < 3 {
            return HnFitResult {
                eps_inf: 0.0,
                delta_eps: 0.0,
                tau_s: 1e-6,
                alpha: 1.0,
                beta: 1.0,
                chi_squared: f64::INFINITY,
            };
        }

        // Initial guesses
        let eps_inf = spectrum.eps_real[n - 1]; // high-frequency limit
        let eps_s = spectrum.eps_real[0]; // low-frequency limit
        let delta_eps = (eps_s - eps_inf).max(0.1);

        // Find peak of eps_imag to estimate tau
        let mut peak_idx = 0;
        let mut peak_val = 0.0_f64;
        for i in 0..n {
            if spectrum.eps_imag[i] > peak_val {
                peak_val = spectrum.eps_imag[i];
                peak_idx = i;
            }
        }
        let tau_init = 1.0 / (2.0 * PI * spectrum.frequency_hz[peak_idx]);

        // Use coordinate descent for simplicity (no external crate)
        let mut params = [eps_inf, delta_eps, tau_init, 0.9, 0.9];
        let bounds_lo = [0.0, 0.01, 1e-15, 0.05, 0.05];
        let bounds_hi = [1e6, 1e6, 1e3, 1.0, 1.0];

        let cost = |p: &[f64; 5]| -> f64 {
            let mut chi2 = 0.0;
            for i in 0..n {
                let (er, ei) =
                    Self::evaluate(spectrum.frequency_hz[i], p[0], p[1], p[2], p[3], p[4]);
                let dr = er - spectrum.eps_real[i];
                let di = ei - spectrum.eps_imag[i];
                chi2 += dr * dr + di * di;
            }
            chi2
        };

        let mut best_cost = cost(&params);

        // Multiple rounds of coordinate descent with shrinking steps
        for round in 0..30 {
            let scale = 0.1_f64.powf(round as f64 / 15.0);
            for dim in 0..5 {
                let step_sizes = [
                    scale * params[dim].abs().max(1e-12) * 0.3,
                    scale * params[dim].abs().max(1e-12) * 0.1,
                    scale * params[dim].abs().max(1e-12) * 0.03,
                ];
                for &step in &step_sizes {
                    for &direction in &[1.0_f64, -1.0] {
                        let mut trial = params;
                        trial[dim] += direction * step;
                        trial[dim] = trial[dim].clamp(bounds_lo[dim], bounds_hi[dim]);
                        let c = cost(&trial);
                        if c < best_cost {
                            best_cost = c;
                            params = trial;
                        }
                    }
                }
            }
        }

        HnFitResult {
            eps_inf: params[0],
            delta_eps: params[1],
            tau_s: params[2],
            alpha: params[3],
            beta: params[4],
            chi_squared: best_cost,
        }
    }
}

// ============================================================================
// DebyeRelaxation
// ============================================================================

/// Result of Debye model fit.
#[derive(Debug, Clone)]
pub struct DebyeFitResult {
    pub eps_s: f64,
    pub eps_inf: f64,
    pub tau: f64,
    pub chi_squared: f64,
}

/// Debye single-relaxation-time model.
///
/// epsilon*(omega) = epsilon_inf + (epsilon_s - epsilon_inf) / (1 + i*omega*tau)
pub struct DebyeRelaxation;

impl DebyeRelaxation {
    /// Evaluate Debye model at a single frequency.
    /// Returns (epsilon', epsilon'').
    pub fn evaluate(freq: f64, eps_s: f64, eps_inf: f64, tau: f64) -> (f64, f64) {
        let omega = 2.0 * PI * freq;
        let wt = omega * tau;
        let denom = 1.0 + wt * wt;
        let delta_eps = eps_s - eps_inf;
        let eps_real = eps_inf + delta_eps / denom;
        let eps_imag = delta_eps * wt / denom;
        (eps_real, eps_imag)
    }

    /// Estimate relaxation time from peak frequency of epsilon''.
    /// tau = 1 / (2 * pi * f_peak)
    pub fn relaxation_time_from_peak(f_peak_hz: f64) -> f64 {
        1.0 / (2.0 * PI * f_peak_hz)
    }

    /// Generate Cole-Cole plot data (epsilon'' vs epsilon') - Argand diagram.
    /// Returns Vec<(epsilon', epsilon'')>.
    pub fn cole_cole_plot(spectrum: &DielectricSpectrum) -> Vec<(f64, f64)> {
        spectrum
            .eps_real
            .iter()
            .zip(spectrum.eps_imag.iter())
            .map(|(&r, &i)| (r, i))
            .collect()
    }

    /// Fit Debye model to spectrum data.
    pub fn fit_debye(spectrum: &DielectricSpectrum) -> DebyeFitResult {
        let n = spectrum.len();
        if n < 2 {
            return DebyeFitResult {
                eps_s: 0.0,
                eps_inf: 0.0,
                tau: 1e-6,
                chi_squared: f64::INFINITY,
            };
        }

        // Initial estimates
        let eps_s = spectrum.eps_real[0]; // low frequency
        let eps_inf = spectrum.eps_real[n - 1]; // high frequency

        // Find peak of eps_imag for tau estimate
        let mut peak_idx = 0;
        let mut peak_val = 0.0_f64;
        for i in 0..n {
            if spectrum.eps_imag[i] > peak_val {
                peak_val = spectrum.eps_imag[i];
                peak_idx = i;
            }
        }
        let tau_init = 1.0 / (2.0 * PI * spectrum.frequency_hz[peak_idx]);

        let mut params = [eps_s, eps_inf, tau_init];

        let cost = |p: &[f64; 3]| -> f64 {
            let mut chi2 = 0.0;
            for i in 0..n {
                let (er, ei) = Self::evaluate(spectrum.frequency_hz[i], p[0], p[1], p[2]);
                let dr = er - spectrum.eps_real[i];
                let di = ei - spectrum.eps_imag[i];
                chi2 += dr * dr + di * di;
            }
            chi2
        };

        let mut best_cost = cost(&params);

        // Coordinate descent optimization
        for round in 0..40 {
            let scale = 0.1_f64.powf(round as f64 / 20.0);
            for dim in 0..3 {
                let base = params[dim].abs().max(1e-12);
                for &frac in &[0.3, 0.1, 0.03, 0.01] {
                    let step = scale * base * frac;
                    for &dir in &[1.0_f64, -1.0] {
                        let mut trial = params;
                        trial[dim] += dir * step;
                        if dim == 2 {
                            trial[dim] = trial[dim].max(1e-15);
                        }
                        let c = cost(&trial);
                        if c < best_cost {
                            best_cost = c;
                            params = trial;
                        }
                    }
                }
            }
        }

        DebyeFitResult {
            eps_s: params[0],
            eps_inf: params[1],
            tau: params[2],
            chi_squared: best_cost,
        }
    }
}

// ============================================================================
// ConductivityAnalysis
// ============================================================================

/// DC and AC conductivity analysis.
pub struct ConductivityAnalysis;

impl ConductivityAnalysis {
    /// Estimate DC conductivity from the low-frequency plateau of sigma'(f) = omega*eps0*eps''.
    /// Uses the lowest-frequency points where sigma' is approximately constant.
    pub fn dc_conductivity(spectrum: &DielectricSpectrum) -> f64 {
        let n = spectrum.len();
        if n == 0 {
            return 0.0;
        }
        // sigma' = omega * eps0 * eps''
        // At low frequencies, sigma' -> sigma_DC
        let n_low = (n / 5).max(1).min(5);
        let mut sigma_sum = 0.0;
        for i in 0..n_low {
            let omega = 2.0 * PI * spectrum.frequency_hz[i];
            sigma_sum += omega * EPS0 * spectrum.eps_imag[i];
        }
        sigma_sum / n_low as f64
    }

    /// Subtract DC conductivity contribution from epsilon''.
    /// epsilon''_corrected = epsilon'' - sigma_DC / (epsilon_0 * omega)
    pub fn subtract_conductivity(
        spectrum: &DielectricSpectrum,
        sigma_dc: f64,
    ) -> DielectricSpectrum {
        let n = spectrum.len();
        let mut eps_imag_corrected = Vec::with_capacity(n);
        for i in 0..n {
            let omega = 2.0 * PI * spectrum.frequency_hz[i];
            let conductivity_contribution = sigma_dc / (EPS0 * omega);
            eps_imag_corrected.push((spectrum.eps_imag[i] - conductivity_contribution).max(0.0));
        }
        DielectricSpectrum {
            frequency_hz: spectrum.frequency_hz.clone(),
            eps_real: spectrum.eps_real.clone(),
            eps_imag: eps_imag_corrected,
        }
    }

    /// Jonscher universal power law for AC conductivity.
    /// sigma(omega) = sigma_DC + A * omega^s
    pub fn jonscher_power_law(freq: f64, sigma_dc: f64, a: f64, s: f64) -> f64 {
        let omega = 2.0 * PI * freq;
        sigma_dc + a * omega.powf(s)
    }

    /// Barton-Nakajima-Namikawa (BNN) relation.
    /// sigma_DC = p * epsilon_0 * delta_eps / tau
    /// where p is typically ~1 for ionic conductors.
    /// Returns the BNN relation value (should be ~1 for consistency).
    pub fn barton_nakajima_namikawa(sigma_dc: f64, delta_eps: f64, tau: f64) -> f64 {
        if delta_eps.abs() < 1e-30 || sigma_dc.abs() < 1e-30 {
            return 0.0;
        }
        // p = sigma_DC * tau / (epsilon_0 * delta_eps)
        sigma_dc * tau / (EPS0 * delta_eps)
    }

    /// Activation energy from Arrhenius plot of conductivity vs temperature.
    /// sigma(T) = sigma_0 * exp(-Ea / (k_B * T))
    /// ln(sigma) = ln(sigma_0) - Ea / (k_B * T)
    /// Linear fit of ln(sigma) vs 1/T gives slope = -Ea/k_B.
    /// Input: &[(temperature_K, sigma)]
    /// Returns activation energy in eV.
    pub fn activation_energy(sigma_at_temps: &[(f64, f64)]) -> f64 {
        if sigma_at_temps.len() < 2 {
            return 0.0;
        }
        // Linear regression of ln(sigma) vs 1/T
        let n = sigma_at_temps.len() as f64;
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xx = 0.0;
        let mut sum_xy = 0.0;

        for &(temp, sigma) in sigma_at_temps {
            if temp <= 0.0 || sigma <= 0.0 {
                continue;
            }
            let x = 1.0 / temp;
            let y = sigma.ln();
            sum_x += x;
            sum_y += y;
            sum_xx += x * x;
            sum_xy += x * y;
        }

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return 0.0;
        }
        let slope = (n * sum_xy - sum_x * sum_y) / denom;
        // slope = -Ea / k_B => Ea = -slope * k_B
        -slope * KB_EV
    }
}

// ============================================================================
// ElectrodePolarization
// ============================================================================

/// Correction type for electrode polarization removal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CorrectionType {
    /// Simple subtraction of power-law fit at low frequencies
    PowerLaw,
    /// Series capacitor model subtraction
    SeriesCapacitor,
}

/// Electrode polarization detection and removal.
pub struct ElectrodePolarization;

impl ElectrodePolarization {
    /// Detect electrode polarization from anomalous low-frequency upturn in epsilon'.
    /// Returns true if epsilon' increases dramatically at low frequencies.
    pub fn detect_electrode_polarization(spectrum: &DielectricSpectrum) -> bool {
        let n = spectrum.len();
        if n < 5 {
            return false;
        }
        // Check if epsilon' at lowest frequencies is much larger than mid-range
        let mid_idx = n / 2;
        let low_eps = spectrum.eps_real[0];
        let mid_eps = spectrum.eps_real[mid_idx];

        if mid_eps <= 0.0 {
            return false;
        }

        // Also check slope: d(eps')/d(log f) should be strongly negative at low f
        let log_f_ratio = (spectrum.frequency_hz[1] / spectrum.frequency_hz[0]).ln();
        if log_f_ratio.abs() < 1e-10 {
            return false;
        }
        let slope = (spectrum.eps_real[1] - spectrum.eps_real[0]) / log_f_ratio;

        // Electrode polarization produces much steeper upturn than normal dispersion.
        // Normal Debye relaxation: eps'(low)/eps'(mid) < ~20 over a few decades.
        // Electrode polarization: ratio > 50 with very steep negative slope in log-f.
        low_eps / mid_eps > 50.0 && slope < -10.0 * mid_eps
    }

    /// Remove electrode polarization contribution from the spectrum.
    pub fn remove_electrode_contribution(
        spectrum: &DielectricSpectrum,
        correction_type: CorrectionType,
    ) -> DielectricSpectrum {
        let n = spectrum.len();
        match correction_type {
            CorrectionType::PowerLaw => {
                // Fit eps'_electrode ~ A * f^(-m) to low-frequency region
                // Use first few points to estimate A, m
                let n_fit = (n / 5).max(2).min(5);
                let (a, m) = Self::fit_power_law(
                    &spectrum.frequency_hz[..n_fit],
                    &spectrum.eps_real[..n_fit],
                );
                let mut eps_real_corrected = Vec::with_capacity(n);
                let mut eps_imag_corrected = Vec::with_capacity(n);
                for i in 0..n {
                    let electrode_re = a * spectrum.frequency_hz[i].powf(-m);
                    eps_real_corrected.push(spectrum.eps_real[i] - electrode_re);
                    // The imaginary part also has a contribution ~ B * f^(-m')
                    // For simplicity, subtract proportional contribution
                    let electrode_im = a * 0.5 * spectrum.frequency_hz[i].powf(-m);
                    eps_imag_corrected
                        .push((spectrum.eps_imag[i] - electrode_im).max(0.0));
                }
                DielectricSpectrum {
                    frequency_hz: spectrum.frequency_hz.clone(),
                    eps_real: eps_real_corrected,
                    eps_imag: eps_imag_corrected,
                }
            }
            CorrectionType::SeriesCapacitor => {
                // Model electrode as series capacitance: subtract 1/(omega*C_el) from impedance
                // Equivalent to subtracting from eps: use highest freq as bulk estimate
                let eps_bulk = spectrum.eps_real[n - 1];
                let mut eps_real_corrected = Vec::with_capacity(n);
                for i in 0..n {
                    let corrected = if spectrum.eps_real[i] > eps_bulk * 2.0 {
                        eps_bulk
                    } else {
                        spectrum.eps_real[i]
                    };
                    eps_real_corrected.push(corrected);
                }
                DielectricSpectrum {
                    frequency_hz: spectrum.frequency_hz.clone(),
                    eps_real: eps_real_corrected,
                    eps_imag: spectrum.eps_imag.clone(),
                }
            }
        }
    }

    /// Blocking electrode model for a sample with mobile ions.
    /// Returns (epsilon', epsilon'') including electrode blocking effects.
    pub fn blocking_electrode_model(
        freq: f64,
        eps_bulk: f64,
        sigma: f64,
        d_m: f64,
    ) -> (f64, f64) {
        let omega = 2.0 * PI * freq;
        // Simplified model: electrode double layer adds low-frequency capacitance
        // C_dl ~ epsilon_0 * A / d_Debye, but here parameterized by d_m (Debye length)
        let tau_el = EPS0 * eps_bulk / sigma;
        let omega_tau = omega * tau_el;
        let debye_factor = d_m / (d_m + 1.0 / (omega * omega + 1.0));
        let eps_re = eps_bulk * (1.0 + debye_factor / (omega_tau * omega_tau + 1.0));
        let eps_im = sigma / (EPS0 * omega)
            + eps_bulk * omega_tau / (omega_tau * omega_tau + 1.0);
        (eps_re, eps_im)
    }

    /// Fit power law y = A * x^(-m) using log-linear regression.
    fn fit_power_law(x: &[f64], y: &[f64]) -> (f64, f64) {
        let n = x.len() as f64;
        let mut sum_lx = 0.0;
        let mut sum_ly = 0.0;
        let mut sum_lxlx = 0.0;
        let mut sum_lxly = 0.0;
        let mut count = 0.0;

        for i in 0..x.len() {
            if x[i] > 0.0 && y[i] > 0.0 {
                let lx = x[i].ln();
                let ly = y[i].ln();
                sum_lx += lx;
                sum_ly += ly;
                sum_lxlx += lx * lx;
                sum_lxly += lx * ly;
                count += 1.0;
            }
        }

        if count < 2.0 {
            return (1.0, 1.0);
        }

        let denom = count * sum_lxlx - sum_lx * sum_lx;
        if denom.abs() < 1e-30 {
            return (1.0, 1.0);
        }

        let slope = (count * sum_lxly - sum_lx * sum_ly) / denom;
        let intercept = (sum_ly - slope * sum_lx) / count;

        let a = intercept.exp();
        let m = -slope; // y = A * x^(-m) => ln(y) = ln(A) - m*ln(x)
        (a.max(0.0), m.max(0.0))
    }
}

// ============================================================================
// RelaxationMap
// ============================================================================

/// Arrhenius fit result.
#[derive(Debug, Clone)]
pub struct ArrheniusResult {
    /// Activation energy in eV
    pub ea_ev: f64,
    /// Pre-exponential frequency in Hz
    pub f0_hz: f64,
    /// Coefficient of determination
    pub r_squared: f64,
}

/// Vogel-Fulcher-Tammann fit result.
#[derive(Debug, Clone)]
pub struct VftResult {
    /// Pre-exponential frequency in Hz
    pub f0_hz: f64,
    /// Activation parameter B in K
    pub b_k: f64,
    /// Vogel temperature T0 in K
    pub t0_k: f64,
    /// Glass transition temperature Tg in K
    pub tg_k: f64,
    /// Fragility index m
    pub fragility: f64,
}

/// KWW stretched exponential fit result.
#[derive(Debug, Clone)]
pub struct KwwResult {
    /// Relaxation time in seconds
    pub tau: f64,
    /// Stretching exponent (0 < beta_kww <= 1)
    pub beta_kww: f64,
    /// Chi-squared residual
    pub chi_squared: f64,
}

/// Temperature-frequency activation diagram (relaxation map).
pub struct RelaxationMap {
    /// (temperature_K, peak_frequency_Hz)
    points: Vec<(f64, f64)>,
}

impl RelaxationMap {
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// Add a relaxation point (temperature in K, peak frequency in Hz).
    pub fn add_point(&mut self, temperature_k: f64, peak_frequency_hz: f64) {
        self.points.push((temperature_k, peak_frequency_hz));
    }

    /// Number of stored data points.
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Whether the map is empty.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Arrhenius fit: ln(f) = ln(f0) - Ea/(k_B * T).
    /// Input: &[(temperature_K, frequency_Hz)].
    pub fn arrhenius_fit(points: &[(f64, f64)]) -> ArrheniusResult {
        if points.len() < 2 {
            return ArrheniusResult {
                ea_ev: 0.0,
                f0_hz: 1.0,
                r_squared: 0.0,
            };
        }

        // ln(f) = ln(f0) - Ea/(k_B * T)
        // y = a + b*x where x = 1/T, y = ln(f), b = -Ea/k_B, a = ln(f0)
        let n = points.len() as f64;
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xx = 0.0;
        let mut sum_xy = 0.0;

        for &(t, f) in points {
            if t <= 0.0 || f <= 0.0 {
                continue;
            }
            let x = 1.0 / t;
            let y = f.ln();
            sum_x += x;
            sum_y += y;
            sum_xx += x * x;
            sum_xy += x * y;
        }

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return ArrheniusResult {
                ea_ev: 0.0,
                f0_hz: 1.0,
                r_squared: 0.0,
            };
        }

        let b = (n * sum_xy - sum_x * sum_y) / denom;
        let a = (sum_y - b * sum_x) / n;

        // b = -Ea/k_B => Ea = -b * k_B
        let ea_ev = -b * KB_EV;
        let f0_hz = a.exp();

        // R-squared
        let y_mean = sum_y / n;
        let mut ss_tot = 0.0;
        let mut ss_res = 0.0;
        for &(t, f) in points {
            if t <= 0.0 || f <= 0.0 {
                continue;
            }
            let x = 1.0 / t;
            let y = f.ln();
            let y_pred = a + b * x;
            ss_tot += (y - y_mean) * (y - y_mean);
            ss_res += (y - y_pred) * (y - y_pred);
        }
        let r_squared = if ss_tot > 1e-30 {
            1.0 - ss_res / ss_tot
        } else {
            0.0
        };

        ArrheniusResult {
            ea_ev,
            f0_hz,
            r_squared,
        }
    }

    /// Vogel-Fulcher-Tammann fit: f = f0 * exp(-B / (T - T0)).
    /// Uses iterative search over T0 with linear regression for f0, B.
    pub fn vft_fit(points: &[(f64, f64)]) -> VftResult {
        if points.len() < 3 {
            return VftResult {
                f0_hz: 1e12,
                b_k: 1000.0,
                t0_k: 150.0,
                tg_k: 0.0,
                fragility: 0.0,
            };
        }

        let t_min = points
            .iter()
            .map(|&(t, _)| t)
            .fold(f64::INFINITY, f64::min);

        let mut best_r2 = f64::NEG_INFINITY;
        let mut best_f0 = 1e12_f64;
        let mut best_b = 1000.0_f64;
        let mut best_t0 = 100.0_f64;

        // Scan T0 from 0 to below T_min
        let t0_max = t_min - 5.0;
        let t0_min_scan = 0.0;
        let n_scan = 200;

        for k in 0..n_scan {
            let t0 = t0_min_scan + (t0_max - t0_min_scan) * k as f64 / (n_scan - 1) as f64;
            if t0 <= 0.0 {
                continue;
            }

            // ln(f) = ln(f0) - B/(T - T0)
            // y = a + b*x where x = 1/(T-T0), y = ln(f), b = -B, a = ln(f0)
            let n = points.len() as f64;
            let mut sum_x = 0.0;
            let mut sum_y = 0.0;
            let mut sum_xx = 0.0;
            let mut sum_xy = 0.0;
            let mut valid = true;

            for &(t, f) in points {
                if t - t0 <= 0.0 || f <= 0.0 {
                    valid = false;
                    break;
                }
                let x = 1.0 / (t - t0);
                let y = f.ln();
                sum_x += x;
                sum_y += y;
                sum_xx += x * x;
                sum_xy += x * y;
            }

            if !valid {
                continue;
            }

            let denom = n * sum_xx - sum_x * sum_x;
            if denom.abs() < 1e-30 {
                continue;
            }

            let slope = (n * sum_xy - sum_x * sum_y) / denom;
            let intercept = (sum_y - slope * sum_x) / n;

            let y_mean = sum_y / n;
            let mut ss_tot = 0.0;
            let mut ss_res = 0.0;
            for &(t, f) in points {
                let x = 1.0 / (t - t0);
                let y = f.ln();
                let y_pred = intercept + slope * x;
                ss_tot += (y - y_mean) * (y - y_mean);
                ss_res += (y - y_pred) * (y - y_pred);
            }
            let r2 = if ss_tot > 1e-30 {
                1.0 - ss_res / ss_tot
            } else {
                0.0
            };

            if r2 > best_r2 {
                best_r2 = r2;
                best_f0 = intercept.exp();
                best_b = -slope; // slope = -B
                best_t0 = t0;
            }
        }

        // Glass transition: T at which f = 0.01 Hz (tau ~ 100 s)
        let tg = Self::glass_transition_from_vft_params(best_f0, best_b, best_t0, 0.01);

        // Fragility index m = d(log10 tau)/d(Tg/T) at T=Tg
        // m = B * Tg / (ln(10) * (Tg - T0)^2)
        let fragility = if (tg - best_t0).abs() > 1.0 {
            best_b * tg / (2.302585 * (tg - best_t0) * (tg - best_t0))
        } else {
            0.0
        };

        VftResult {
            f0_hz: best_f0,
            b_k: best_b,
            t0_k: best_t0,
            tg_k: tg,
            fragility,
        }
    }

    /// Glass transition temperature from VFT parameters at a given criterion frequency.
    /// f = f0 * exp(-B / (T - T0)) => T = T0 + B / ln(f0/f)
    pub fn glass_transition_from_vft(vft: &VftResult, criterion_hz: f64) -> f64 {
        Self::glass_transition_from_vft_params(vft.f0_hz, vft.b_k, vft.t0_k, criterion_hz)
    }

    fn glass_transition_from_vft_params(f0: f64, b: f64, t0: f64, criterion_hz: f64) -> f64 {
        if criterion_hz <= 0.0 || f0 <= 0.0 {
            return t0;
        }
        let ratio = f0 / criterion_hz;
        if ratio <= 1.0 {
            return t0;
        }
        t0 + b / ratio.ln()
    }
}

impl Default for RelaxationMap {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// MixingRules
// ============================================================================

/// Effective permittivity mixing rules for composite materials.
pub struct MixingRules;

impl MixingRules {
    /// Maxwell-Garnett mixing rule for dilute inclusions in a host matrix.
    /// eps_eff = eps_host * (1 + 3*f*(eps_inc - eps_host) / (eps_inc + 2*eps_host - f*(eps_inc - eps_host)))
    pub fn maxwell_garnett(eps_host: f64, eps_inclusion: f64, volume_fraction: f64) -> f64 {
        let f = volume_fraction;
        let diff = eps_inclusion - eps_host;
        let denom = eps_inclusion + 2.0 * eps_host - f * diff;
        if denom.abs() < 1e-30 {
            return eps_host;
        }
        eps_host * (1.0 + 3.0 * f * diff / denom)
    }

    /// Bruggeman effective medium approximation (self-consistent).
    /// Solves: f_a * (eps_a - eps_eff)/(eps_a + 2*eps_eff) + f_b * (eps_b - eps_eff)/(eps_b + 2*eps_eff) = 0
    /// Uses iterative solution.
    pub fn bruggeman(eps_a: f64, eps_b: f64, fraction_a: f64) -> f64 {
        let f_a = fraction_a;
        let f_b = 1.0 - fraction_a;

        // Initial guess: linear interpolation
        let mut eps_eff = f_a * eps_a + f_b * eps_b;

        for _ in 0..100 {
            let denom_a = eps_a + 2.0 * eps_eff;
            let denom_b = eps_b + 2.0 * eps_eff;
            if denom_a.abs() < 1e-30 || denom_b.abs() < 1e-30 {
                break;
            }
            let g = f_a * (eps_a - eps_eff) / denom_a + f_b * (eps_b - eps_eff) / denom_b;
            // Derivative dg/d(eps_eff)
            let dg = -f_a * (eps_a + 2.0 * eps_a) / (denom_a * denom_a)
                - f_b * (eps_b + 2.0 * eps_b) / (denom_b * denom_b);
            if dg.abs() < 1e-30 {
                break;
            }
            let delta = -g / dg;
            eps_eff += delta;
            if delta.abs() < 1e-12 * eps_eff.abs() {
                break;
            }
        }

        eps_eff
    }

    /// Lichtenecker logarithmic mixing rule.
    /// ln(eps_eff) = f_a * ln(eps_a) + f_b * ln(eps_b) when k=0
    /// General: eps_eff^k = f_a * eps_a^k + f_b * eps_b^k
    pub fn lichtenecker(eps_a: f64, eps_b: f64, fraction_a: f64, k: f64) -> f64 {
        let f_b = 1.0 - fraction_a;
        if k.abs() < 1e-10 {
            // Logarithmic mixing
            (fraction_a * eps_a.ln() + f_b * eps_b.ln()).exp()
        } else {
            let mix = fraction_a * eps_a.powf(k) + f_b * eps_b.powf(k);
            mix.powf(1.0 / k)
        }
    }

    /// Parallel (Voigt) model - upper Wiener bound.
    /// eps_eff = f_a * eps_a + f_b * eps_b
    pub fn parallel_model(eps_a: f64, eps_b: f64, fraction_a: f64) -> f64 {
        fraction_a * eps_a + (1.0 - fraction_a) * eps_b
    }

    /// Series (Reuss) model - lower Wiener bound.
    /// 1/eps_eff = f_a/eps_a + f_b/eps_b
    pub fn series_model(eps_a: f64, eps_b: f64, fraction_a: f64) -> f64 {
        let f_b = 1.0 - fraction_a;
        if eps_a.abs() < 1e-30 || eps_b.abs() < 1e-30 {
            return 0.0;
        }
        1.0 / (fraction_a / eps_a + f_b / eps_b)
    }
}

// ============================================================================
// DielectricModulus
// ============================================================================

/// Electric modulus M*(omega) = 1/epsilon*(omega).
#[derive(Debug, Clone)]
pub struct DielectricModulus {
    pub frequency_hz: Vec<f64>,
    /// Real part M'
    pub m_real: Vec<f64>,
    /// Imaginary part M''
    pub m_imag: Vec<f64>,
}

impl DielectricModulus {
    /// Create a new modulus spectrum directly.
    pub fn new(frequency_hz: Vec<f64>, m_real: Vec<f64>, m_imag: Vec<f64>) -> Self {
        assert_eq!(frequency_hz.len(), m_real.len());
        assert_eq!(frequency_hz.len(), m_imag.len());
        Self {
            frequency_hz,
            m_real,
            m_imag,
        }
    }

    /// Convert from permittivity: M* = 1/epsilon*.
    /// M' = eps' / (eps'^2 + eps''^2)
    /// M'' = eps'' / (eps'^2 + eps''^2)
    pub fn from_permittivity(spectrum: &DielectricSpectrum) -> Self {
        let n = spectrum.len();
        let mut m_real = Vec::with_capacity(n);
        let mut m_imag = Vec::with_capacity(n);
        for i in 0..n {
            let er = spectrum.eps_real[i];
            let ei = spectrum.eps_imag[i];
            let mag_sq = er * er + ei * ei;
            if mag_sq < 1e-30 {
                m_real.push(0.0);
                m_imag.push(0.0);
            } else {
                m_real.push(er / mag_sq);
                m_imag.push(ei / mag_sq);
            }
        }
        Self {
            frequency_hz: spectrum.frequency_hz.clone(),
            m_real,
            m_imag,
        }
    }

    /// Find the peak frequency of M'' (conductivity relaxation frequency).
    pub fn peak_frequency(&self) -> f64 {
        let mut max_val = f64::NEG_INFINITY;
        let mut max_idx = 0;
        for (i, &m) in self.m_imag.iter().enumerate() {
            if m > max_val {
                max_val = m;
                max_idx = i;
            }
        }
        self.frequency_hz[max_idx]
    }

    /// KWW (Kohlrausch-Williams-Watts) stretched exponential fit to M'' peak.
    /// phi(t) = exp(-(t/tau)^beta_kww), 0 < beta_kww <= 1
    /// Fits in frequency domain using approximate HN representation.
    pub fn kww_fit(modulus: &DielectricModulus) -> KwwResult {
        let n = modulus.frequency_hz.len();
        if n < 3 {
            return KwwResult {
                tau: 1e-6,
                beta_kww: 1.0,
                chi_squared: f64::INFINITY,
            };
        }

        // Find peak of M'' for initial tau estimate
        let peak_f = modulus.peak_frequency();
        let tau_init = 1.0 / (2.0 * PI * peak_f);

        // KWW in frequency domain is approximately HN with
        // alpha_hn ~ beta_kww, beta_hn ~ 1 (for moderate beta_kww)
        // M'' ~ M_max * 2 / (exp(beta*(x-x0)) + exp(-beta*(x-x0)))
        // where x = ln(f/f_peak)

        let m_max = modulus
            .m_imag
            .iter()
            .cloned()
            .fold(0.0_f64, f64::max);

        let mut best_tau = tau_init;
        let mut best_beta = 0.8;
        let mut best_cost = f64::INFINITY;

        // Grid search + refinement
        for beta_step in 0..20 {
            let beta = 0.3 + 0.7 * beta_step as f64 / 19.0;
            for tau_mult in 0..20 {
                let tau = tau_init * (0.1_f64).powf((tau_mult as f64 - 10.0) / 10.0);
                let cost = Self::kww_cost(modulus, tau, beta, m_max);
                if cost < best_cost {
                    best_cost = cost;
                    best_tau = tau;
                    best_beta = beta;
                }
            }
        }

        // Refine with finer search
        for _ in 0..20 {
            let dt = best_tau * 0.05;
            let db = 0.02;
            let mut improved = false;
            for &tau_off in &[-dt, 0.0, dt] {
                for &beta_off in &[-db, 0.0, db] {
                    let tau = (best_tau + tau_off).max(1e-15);
                    let beta = (best_beta + beta_off).clamp(0.1, 1.0);
                    let cost = Self::kww_cost(modulus, tau, beta, m_max);
                    if cost < best_cost {
                        best_cost = cost;
                        best_tau = tau;
                        best_beta = beta;
                        improved = true;
                    }
                }
            }
            if !improved {
                break;
            }
        }

        KwwResult {
            tau: best_tau,
            beta_kww: best_beta,
            chi_squared: best_cost,
        }
    }

    fn kww_cost(modulus: &DielectricModulus, tau: f64, beta_kww: f64, m_max: f64) -> f64 {
        let f_peak = 1.0 / (2.0 * PI * tau);
        let mut cost = 0.0;
        for i in 0..modulus.frequency_hz.len() {
            let x = (modulus.frequency_hz[i] / f_peak).ln();
            // Approximate KWW frequency-domain shape
            let m_pred = m_max / (1.0 + (beta_kww * x).cosh());
            let diff = m_pred - modulus.m_imag[i];
            cost += diff * diff;
        }
        cost
    }
}

// ============================================================================
// DielectricSimulator
// ============================================================================

/// Generate synthetic dielectric spectra for testing and validation.
pub struct DielectricSimulator;

impl DielectricSimulator {
    /// Simulate a Debye relaxation spectrum.
    pub fn simulate_debye(
        eps_s: f64,
        eps_inf: f64,
        tau: f64,
        f_range: (f64, f64),
        n: usize,
    ) -> DielectricSpectrum {
        let log_f_min = f_range.0.log10();
        let log_f_max = f_range.1.log10();
        let mut freq = Vec::with_capacity(n);
        let mut eps_real = Vec::with_capacity(n);
        let mut eps_imag = Vec::with_capacity(n);

        for i in 0..n {
            let log_f = log_f_min + (log_f_max - log_f_min) * i as f64 / (n - 1).max(1) as f64;
            let f = 10.0_f64.powf(log_f);
            let (er, ei) = DebyeRelaxation::evaluate(f, eps_s, eps_inf, tau);
            freq.push(f);
            eps_real.push(er);
            eps_imag.push(ei);
        }

        DielectricSpectrum::new(freq, eps_real, eps_imag)
    }

    /// Simulate a Havriliak-Negami relaxation spectrum.
    pub fn simulate_hn(params: &HnFitResult, f_range: (f64, f64), n: usize) -> DielectricSpectrum {
        let log_f_min = f_range.0.log10();
        let log_f_max = f_range.1.log10();
        let mut freq = Vec::with_capacity(n);
        let mut eps_real = Vec::with_capacity(n);
        let mut eps_imag = Vec::with_capacity(n);

        for i in 0..n {
            let log_f = log_f_min + (log_f_max - log_f_min) * i as f64 / (n - 1).max(1) as f64;
            let f = 10.0_f64.powf(log_f);
            let (er, ei) = HavriliakNegami::evaluate(
                f,
                params.eps_inf,
                params.delta_eps,
                params.tau_s,
                params.alpha,
                params.beta,
            );
            freq.push(f);
            eps_real.push(er);
            eps_imag.push(ei);
        }

        DielectricSpectrum::new(freq, eps_real, eps_imag)
    }

    /// Add DC conductivity contribution to a relaxation spectrum.
    pub fn simulate_with_conductivity(
        relaxation: &DielectricSpectrum,
        sigma_dc: f64,
    ) -> DielectricSpectrum {
        let n = relaxation.len();
        let mut eps_imag_with_cond = Vec::with_capacity(n);
        for i in 0..n {
            let omega = 2.0 * PI * relaxation.frequency_hz[i];
            let cond_contribution = sigma_dc / (EPS0 * omega);
            eps_imag_with_cond.push(relaxation.eps_imag[i] + cond_contribution);
        }
        DielectricSpectrum {
            frequency_hz: relaxation.frequency_hz.clone(),
            eps_real: relaxation.eps_real.clone(),
            eps_imag: eps_imag_with_cond,
        }
    }

    /// Add Gaussian noise to a spectrum.
    pub fn add_noise(spectrum: &DielectricSpectrum, noise_percent: f64) -> DielectricSpectrum {
        let n = spectrum.len();
        let mut eps_real_noisy = Vec::with_capacity(n);
        let mut eps_imag_noisy = Vec::with_capacity(n);

        // Simple deterministic pseudo-random noise using a linear congruential generator
        let mut seed: u64 = 12345;
        let next_rand = |s: &mut u64| -> f64 {
            *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            // Box-Muller-like: use two uniform values for Gaussian approximation
            let u1 = (*s >> 33) as f64 / (1u64 << 31) as f64;
            *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let u2 = (*s >> 33) as f64 / (1u64 << 31) as f64;
            // Central limit theorem approximation
            (u1 + u2 - 1.0) * 1.7320508 // sqrt(3) to normalize
        };

        let frac = noise_percent / 100.0;
        for i in 0..n {
            let noise_re = next_rand(&mut seed) * spectrum.eps_real[i].abs() * frac;
            let noise_im = next_rand(&mut seed) * spectrum.eps_imag[i].abs() * frac;
            eps_real_noisy.push(spectrum.eps_real[i] + noise_re);
            eps_imag_noisy.push((spectrum.eps_imag[i] + noise_im).max(0.0));
        }

        DielectricSpectrum {
            frequency_hz: spectrum.frequency_hz.clone(),
            eps_real: eps_real_noisy,
            eps_imag: eps_imag_noisy,
        }
    }
}

// ============================================================================
// FrequencyAnalysis
// ============================================================================

/// Spectral analysis utilities for dielectric data.
pub struct FrequencyAnalysis;

impl FrequencyAnalysis {
    /// Find loss peaks in epsilon'' spectrum.
    /// Returns Vec<(frequency_Hz, eps_imag_peak)>.
    pub fn peak_finder(eps_imag: &[f64], frequencies: &[f64]) -> Vec<(f64, f64)> {
        let n = eps_imag.len();
        if n < 3 {
            return vec![];
        }
        let mut peaks = Vec::new();
        for i in 1..n - 1 {
            if eps_imag[i] > eps_imag[i - 1] && eps_imag[i] > eps_imag[i + 1] {
                peaks.push((frequencies[i], eps_imag[i]));
            }
        }
        peaks
    }

    /// Kramers-Kronig consistency check.
    /// Computes the KK integral to estimate epsilon' from epsilon'' and compares.
    /// Returns the normalized mean squared error between measured and KK-predicted epsilon'.
    pub fn kramers_kronig_check(spectrum: &DielectricSpectrum) -> f64 {
        let n = spectrum.len();
        if n < 5 {
            return f64::INFINITY;
        }

        // KK relation: eps'(omega) - eps_inf = (2/pi) * P integral[omega'*eps''(omega') / (omega'^2 - omega^2)] d omega'
        // Use numerical integration with principal value (skip the singularity)
        let eps_inf = spectrum.eps_real[n - 1];
        let mut total_error = 0.0;
        let mut total_ref = 0.0;

        for i in 1..n - 1 {
            let omega = 2.0 * PI * spectrum.frequency_hz[i];
            let mut integral = 0.0;

            for j in 0..n - 1 {
                if j == i || j + 1 == i {
                    continue; // Skip singularity
                }
                let omega_j = 2.0 * PI * spectrum.frequency_hz[j];
                let omega_j1 = 2.0 * PI * spectrum.frequency_hz[j + 1];
                let d_omega = omega_j1 - omega_j;
                let eps_im_mid = (spectrum.eps_imag[j] + spectrum.eps_imag[j + 1]) * 0.5;
                let omega_mid = (omega_j + omega_j1) * 0.5;
                let denom = omega_mid * omega_mid - omega * omega;
                if denom.abs() > 1e-20 {
                    integral += omega_mid * eps_im_mid / denom * d_omega;
                }
            }

            let eps_real_kk = eps_inf + (2.0 / PI) * integral;
            let diff = eps_real_kk - spectrum.eps_real[i];
            total_error += diff * diff;
            total_ref += spectrum.eps_real[i] * spectrum.eps_real[i];
        }

        if total_ref < 1e-30 {
            return 0.0;
        }
        (total_error / total_ref).sqrt()
    }

    /// Derivative analysis of epsilon'' to resolve overlapping peaks.
    /// Returns d(epsilon'')/d(log f) for each frequency point.
    pub fn derivative_analysis(spectrum: &DielectricSpectrum) -> Vec<(f64, f64)> {
        let n = spectrum.len();
        if n < 2 {
            return vec![];
        }

        let mut result = Vec::with_capacity(n);

        // Forward difference for first point
        if n >= 2 {
            let d_logf =
                (spectrum.frequency_hz[1] / spectrum.frequency_hz[0]).ln();
            if d_logf.abs() > 1e-20 {
                let deriv = (spectrum.eps_imag[1] - spectrum.eps_imag[0]) / d_logf;
                result.push((spectrum.frequency_hz[0], deriv));
            } else {
                result.push((spectrum.frequency_hz[0], 0.0));
            }
        }

        // Central difference for interior points
        for i in 1..n - 1 {
            let d_logf =
                (spectrum.frequency_hz[i + 1] / spectrum.frequency_hz[i - 1]).ln();
            if d_logf.abs() > 1e-20 {
                let deriv =
                    (spectrum.eps_imag[i + 1] - spectrum.eps_imag[i - 1]) / d_logf;
                result.push((spectrum.frequency_hz[i], deriv));
            } else {
                result.push((spectrum.frequency_hz[i], 0.0));
            }
        }

        // Backward difference for last point
        if n >= 2 {
            let d_logf = (spectrum.frequency_hz[n - 1] / spectrum.frequency_hz[n - 2]).ln();
            if d_logf.abs() > 1e-20 {
                let deriv =
                    (spectrum.eps_imag[n - 1] - spectrum.eps_imag[n - 2]) / d_logf;
                result.push((spectrum.frequency_hz[n - 1], deriv));
            } else {
                result.push((spectrum.frequency_hz[n - 1], 0.0));
            }
        }

        result
    }

    /// Estimate dielectric strength from the area under the epsilon'' curve.
    /// delta_eps = (2/pi) * integral[eps''(f) d(ln f)] over the specified range.
    pub fn strength_from_integral(
        spectrum: &DielectricSpectrum,
        f_min: f64,
        f_max: f64,
    ) -> f64 {
        let n = spectrum.len();
        if n < 2 {
            return 0.0;
        }

        let mut integral = 0.0;
        for i in 0..n - 1 {
            let f1 = spectrum.frequency_hz[i];
            let f2 = spectrum.frequency_hz[i + 1];
            if f2 < f_min || f1 > f_max {
                continue;
            }
            let d_lnf = (f2 / f1).ln();
            let eps_im_avg = (spectrum.eps_imag[i] + spectrum.eps_imag[i + 1]) * 0.5;
            integral += eps_im_avg * d_lnf;
        }

        (2.0 / PI) * integral
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;
    const RTOL: f64 = 0.05; // 5% relative tolerance for fitted quantities

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn relative_eq(a: f64, b: f64, rtol: f64) -> bool {
        if a.abs() < 1e-10 && b.abs() < 1e-10 {
            return true;
        }
        let denom = a.abs().max(b.abs());
        (a - b).abs() / denom < rtol
    }

    // --- DielectricSpectrum tests ---

    #[test]
    fn test_spectrum_creation() {
        let freqs = vec![1e3, 1e4, 1e5];
        let er = vec![10.0, 8.0, 6.0];
        let ei = vec![1.0, 2.0, 0.5];
        let spec = DielectricSpectrum::new(freqs, er, ei);
        assert_eq!(spec.len(), 3);
        assert!(!spec.is_empty());
    }

    #[test]
    fn test_permittivity_at() {
        let spec = DielectricSpectrum::new(
            vec![1e3, 1e4],
            vec![10.0, 8.0],
            vec![1.0, 2.0],
        );
        let (er, ei) = spec.permittivity_at(0);
        assert!(approx_eq(er, 10.0, TOL));
        assert!(approx_eq(ei, 1.0, TOL));
    }

    #[test]
    fn test_loss_tangent() {
        let spec = DielectricSpectrum::new(
            vec![1e3],
            vec![10.0],
            vec![2.0],
        );
        let tan_d = spec.loss_tangent_at(0);
        assert!(approx_eq(tan_d, 0.2, TOL));
    }

    #[test]
    fn test_loss_tangent_zero_eps_real() {
        let spec = DielectricSpectrum::new(vec![1e3], vec![0.0], vec![1.0]);
        let tan_d = spec.loss_tangent_at(0);
        assert!(approx_eq(tan_d, 0.0, TOL));
    }

    #[test]
    fn test_to_modulus() {
        let spec = DielectricSpectrum::new(
            vec![1e6],
            vec![4.0],
            vec![3.0],
        );
        let mod_spec = spec.to_modulus();
        // M' = 4/(16+9) = 4/25 = 0.16
        // M'' = 3/(16+9) = 3/25 = 0.12
        assert!(approx_eq(mod_spec.m_real[0], 0.16, TOL));
        assert!(approx_eq(mod_spec.m_imag[0], 0.12, TOL));
    }

    #[test]
    fn test_to_conductivity() {
        let spec = DielectricSpectrum::new(
            vec![1e6],
            vec![4.0],
            vec![0.1],
        );
        let cond = spec.to_conductivity(EPS0);
        let omega = 2.0 * PI * 1e6;
        let expected_sigma_real = omega * EPS0 * 0.1;
        assert!(relative_eq(cond.sigma_real[0], expected_sigma_real, 1e-6));
    }

    #[test]
    fn test_to_impedance() {
        let spec = DielectricSpectrum::new(
            vec![1e6],
            vec![4.0],
            vec![0.1],
        );
        let z = spec.to_impedance(1e-4, 1e-3);
        assert_eq!(z.len(), 1);
        assert!(z[0].0.is_finite());
        assert!(z[0].1.is_finite());
    }

    // --- Debye tests ---

    #[test]
    fn test_debye_evaluate_dc_limit() {
        // At very low frequency, eps' -> eps_s
        let (er, ei) = DebyeRelaxation::evaluate(0.001, 80.0, 5.0, 1e-3);
        assert!(approx_eq(er, 80.0, 0.01));
        assert!(ei < 1.0); // Small loss at low f
    }

    #[test]
    fn test_debye_evaluate_hf_limit() {
        // At very high frequency, eps' -> eps_inf
        let (er, ei) = DebyeRelaxation::evaluate(1e12, 80.0, 5.0, 1e-6);
        assert!(approx_eq(er, 5.0, 0.1));
        assert!(ei.abs() < 0.1);
    }

    #[test]
    fn test_debye_peak_frequency() {
        let tau = 1e-6;
        let f_peak = 1.0 / (2.0 * PI * tau);
        // At the peak, eps'' should be maximum
        let (_, ei_peak) = DebyeRelaxation::evaluate(f_peak, 80.0, 5.0, tau);
        let (_, ei_off1) = DebyeRelaxation::evaluate(f_peak * 0.1, 80.0, 5.0, tau);
        let (_, ei_off2) = DebyeRelaxation::evaluate(f_peak * 10.0, 80.0, 5.0, tau);
        assert!(ei_peak > ei_off1);
        assert!(ei_peak > ei_off2);
    }

    #[test]
    fn test_debye_peak_value() {
        // For Debye, eps''_max = delta_eps / 2
        let tau = 1e-6;
        let f_peak = 1.0 / (2.0 * PI * tau);
        let eps_s = 80.0;
        let eps_inf = 5.0;
        let (_, ei) = DebyeRelaxation::evaluate(f_peak, eps_s, eps_inf, tau);
        let expected = (eps_s - eps_inf) / 2.0;
        assert!(approx_eq(ei, expected, 0.01));
    }

    #[test]
    fn test_relaxation_time_from_peak() {
        let f_peak = 1e6;
        let tau = DebyeRelaxation::relaxation_time_from_peak(f_peak);
        let expected = 1.0 / (2.0 * PI * f_peak);
        assert!(approx_eq(tau, expected, 1e-12));
    }

    #[test]
    fn test_cole_cole_plot() {
        let spec = DielectricSpectrum::new(
            vec![1e3, 1e4, 1e5],
            vec![10.0, 8.0, 6.0],
            vec![1.0, 2.0, 0.5],
        );
        let cc = DebyeRelaxation::cole_cole_plot(&spec);
        assert_eq!(cc.len(), 3);
        assert!(approx_eq(cc[0].0, 10.0, TOL));
        assert!(approx_eq(cc[0].1, 1.0, TOL));
    }

    #[test]
    fn test_debye_fit() {
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e3, 1e9), 50);
        let fit = DebyeRelaxation::fit_debye(&spec);
        assert!(relative_eq(fit.eps_s, 80.0, RTOL));
        assert!(relative_eq(fit.eps_inf, 5.0, RTOL));
        assert!(relative_eq(fit.tau, 1e-6, RTOL));
    }

    #[test]
    fn test_debye_fit_chi_squared() {
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e3, 1e9), 50);
        let fit = DebyeRelaxation::fit_debye(&spec);
        // Clean data should give very small chi-squared
        assert!(fit.chi_squared < 1.0);
    }

    // --- HN tests ---

    #[test]
    fn test_hn_debye_limit() {
        // HN with alpha=1, beta=1 should equal Debye
        let f = 1e6;
        let eps_inf = 5.0;
        let delta_eps = 75.0;
        let tau = 1e-6;
        let (er_hn, ei_hn) = HavriliakNegami::evaluate(f, eps_inf, delta_eps, tau, 1.0, 1.0);
        let (er_d, ei_d) = DebyeRelaxation::evaluate(f, eps_inf + delta_eps, eps_inf, tau);
        assert!(approx_eq(er_hn, er_d, 0.01));
        assert!(approx_eq(ei_hn, ei_d, 0.01));
    }

    #[test]
    fn test_hn_broadening() {
        // Cole-Cole (beta=1, alpha<1) should produce broader peaks
        let tau = 1e-6;
        let f_peak = 1.0 / (2.0 * PI * tau);
        let (_, ei_debye) =
            HavriliakNegami::evaluate(f_peak, 5.0, 75.0, tau, 1.0, 1.0);
        let (_, ei_cc) =
            HavriliakNegami::evaluate(f_peak, 5.0, 75.0, tau, 0.7, 1.0);
        // Cole-Cole peak should be lower (broader)
        assert!(ei_cc < ei_debye);
    }

    #[test]
    fn test_hn_asymmetry() {
        // Davidson-Cole (alpha=1, beta<1) produces asymmetric peaks
        let tau = 1e-6;
        let f_peak = 1.0 / (2.0 * PI * tau);
        let f_low = f_peak * 0.01;
        let f_high = f_peak * 100.0;
        let (_, ei_low) =
            HavriliakNegami::evaluate(f_low, 5.0, 75.0, tau, 1.0, 0.5);
        let (_, ei_high) =
            HavriliakNegami::evaluate(f_high, 5.0, 75.0, tau, 1.0, 0.5);
        // DC peak should be asymmetric (different values at equal distances from peak)
        assert!((ei_low - ei_high).abs() > 0.1);
    }

    #[test]
    fn test_hn_fit() {
        let params = HnFitResult {
            eps_inf: 3.0,
            delta_eps: 10.0,
            tau_s: 1e-5,
            alpha: 0.85,
            beta: 0.7,
            chi_squared: 0.0,
        };
        let spec = DielectricSimulator::simulate_hn(&params, (1e1, 1e9), 60);
        let fit = HavriliakNegami::fit(&spec);
        assert!(relative_eq(fit.eps_inf, 3.0, 0.15));
        assert!(relative_eq(fit.delta_eps, 10.0, 0.15));
        assert!(relative_eq(fit.tau_s, 1e-5, 0.2));
    }

    // --- Conductivity tests ---

    #[test]
    fn test_dc_conductivity_extraction() {
        // Create spectrum with known DC conductivity
        let sigma_dc = 1e-4;
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e-1, 1e3), 50);
        let spec_with_cond = DielectricSimulator::simulate_with_conductivity(&spec, sigma_dc);
        let sigma_est = ConductivityAnalysis::dc_conductivity(&spec_with_cond);
        // At low frequencies, sigma' should be close to sigma_dc
        assert!(sigma_est > 0.0);
    }

    #[test]
    fn test_subtract_conductivity() {
        let sigma_dc = 1e-4;
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e0, 1e6), 30);
        let spec_with_cond = DielectricSimulator::simulate_with_conductivity(&spec, sigma_dc);
        let spec_cleaned =
            ConductivityAnalysis::subtract_conductivity(&spec_with_cond, sigma_dc);
        // eps'' should be reduced after subtraction
        assert!(spec_cleaned.eps_imag[0] < spec_with_cond.eps_imag[0]);
    }

    #[test]
    fn test_jonscher_power_law() {
        let sigma = ConductivityAnalysis::jonscher_power_law(1e6, 1e-4, 1e-10, 0.7);
        assert!(sigma > 1e-4); // AC part adds to DC
    }

    #[test]
    fn test_jonscher_dc_limit() {
        // At f=0, should approach sigma_DC (using very low frequency)
        let sigma =
            ConductivityAnalysis::jonscher_power_law(1e-10, 1e-4, 1e-10, 0.7);
        assert!(relative_eq(sigma, 1e-4, 0.01));
    }

    #[test]
    fn test_bnn_relation() {
        let sigma_dc = 1e-4;
        let delta_eps = 10.0;
        let tau = 1e-6;
        let p = ConductivityAnalysis::barton_nakajima_namikawa(sigma_dc, delta_eps, tau);
        assert!(p > 0.0);
        assert!(p.is_finite());
    }

    #[test]
    fn test_activation_energy() {
        // Create Arrhenius data: sigma = sigma_0 * exp(-Ea/(k_B*T))
        let ea_ev = 0.5; // 0.5 eV
        let sigma_0 = 1e3;
        let temps = [300.0, 350.0, 400.0, 450.0, 500.0];
        let data: Vec<(f64, f64)> = temps
            .iter()
            .map(|&t| {
                let sigma = sigma_0 * (-ea_ev / (KB_EV * t)).exp();
                (t, sigma)
            })
            .collect();
        let ea_est = ConductivityAnalysis::activation_energy(&data);
        assert!(relative_eq(ea_est, ea_ev, RTOL));
    }

    // --- ElectrodePolarization tests ---

    #[test]
    fn test_detect_electrode_polarization_true() {
        // Create spectrum with large low-frequency upturn
        let mut freqs = Vec::new();
        let mut er = Vec::new();
        let mut ei = Vec::new();
        for i in 0..20 {
            let f = 10.0_f64.powf(i as f64 * 0.5);
            freqs.push(f);
            // Strong electrode polarization: eps' ~ 1/f^2 at low f
            let eps = 10.0 + 1e6 / (f * f + 1.0);
            er.push(eps);
            ei.push(1.0);
        }
        let spec = DielectricSpectrum::new(freqs, er, ei);
        assert!(ElectrodePolarization::detect_electrode_polarization(&spec));
    }

    #[test]
    fn test_detect_electrode_polarization_false() {
        // Clean Debye with no electrode effects
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e3, 1e9), 20);
        assert!(!ElectrodePolarization::detect_electrode_polarization(&spec));
    }

    #[test]
    fn test_remove_electrode_power_law() {
        let mut freqs = Vec::new();
        let mut er = Vec::new();
        let mut ei = Vec::new();
        for i in 0..20 {
            let f = 10.0_f64.powf(1.0 + i as f64 * 0.5);
            freqs.push(f);
            let eps = 10.0 + 1e6 / (f * f);
            er.push(eps);
            ei.push(1.0);
        }
        let spec = DielectricSpectrum::new(freqs, er, ei);
        let corrected =
            ElectrodePolarization::remove_electrode_contribution(&spec, CorrectionType::PowerLaw);
        // Corrected eps' at high frequencies should be similar
        let n = corrected.len();
        assert!(corrected.eps_real[n - 1] < spec.eps_real[0]);
    }

    #[test]
    fn test_remove_electrode_series_cap() {
        let mut freqs = Vec::new();
        let mut er = Vec::new();
        let mut ei = Vec::new();
        for i in 0..20 {
            let f = 10.0_f64.powf(1.0 + i as f64 * 0.5);
            freqs.push(f);
            let eps = 10.0 + 1e4 / (f + 1.0);
            er.push(eps);
            ei.push(1.0);
        }
        let spec = DielectricSpectrum::new(freqs, er, ei);
        let corrected = ElectrodePolarization::remove_electrode_contribution(
            &spec,
            CorrectionType::SeriesCapacitor,
        );
        // Low-f eps' should be capped
        assert!(corrected.eps_real[0] <= spec.eps_real[0]);
    }

    #[test]
    fn test_blocking_electrode_model() {
        let (er, ei) =
            ElectrodePolarization::blocking_electrode_model(1e3, 10.0, 1e-4, 1e-7);
        assert!(er > 10.0); // Should be enhanced by electrode effects
        assert!(ei > 0.0);
    }

    // --- RelaxationMap tests ---

    #[test]
    fn test_relaxation_map_add_point() {
        let mut rm = RelaxationMap::new();
        rm.add_point(300.0, 1e6);
        rm.add_point(350.0, 1e7);
        assert_eq!(rm.len(), 2);
    }

    #[test]
    fn test_arrhenius_fit() {
        let ea = 0.3; // eV
        let f0 = 1e12; // Hz
        let points: Vec<(f64, f64)> = (0..10)
            .map(|i| {
                let t = 300.0 + i as f64 * 30.0;
                let f = f0 * (-ea / (KB_EV * t)).exp();
                (t, f)
            })
            .collect();
        let result = RelaxationMap::arrhenius_fit(&points);
        assert!(relative_eq(result.ea_ev, ea, RTOL));
        assert!(result.r_squared > 0.99);
    }

    #[test]
    fn test_arrhenius_f0() {
        let ea = 0.3;
        let f0 = 1e12;
        let points: Vec<(f64, f64)> = (0..10)
            .map(|i| {
                let t = 300.0 + i as f64 * 30.0;
                let f = f0 * (-ea / (KB_EV * t)).exp();
                (t, f)
            })
            .collect();
        let result = RelaxationMap::arrhenius_fit(&points);
        assert!(relative_eq(result.f0_hz, f0, 0.1)); // 10% tolerance for log-space
    }

    #[test]
    fn test_vft_fit() {
        let f0 = 1e12;
        let b = 1500.0;
        let t0 = 150.0;
        let points: Vec<(f64, f64)> = (0..15)
            .map(|i| {
                let t = 200.0 + i as f64 * 20.0;
                let f = f0 * (-b / (t - t0)).exp();
                (t, f)
            })
            .collect();
        let result = RelaxationMap::vft_fit(&points);
        assert!(relative_eq(result.t0_k, t0, 0.1));
        assert!(relative_eq(result.b_k, b, 0.15));
    }

    #[test]
    fn test_glass_transition_from_vft() {
        let vft = VftResult {
            f0_hz: 1e12,
            b_k: 1500.0,
            t0_k: 150.0,
            tg_k: 0.0,
            fragility: 0.0,
        };
        let tg = RelaxationMap::glass_transition_from_vft(&vft, 0.01);
        // T = T0 + B / ln(f0/f_criterion)
        let expected = 150.0 + 1500.0 / (1e12_f64 / 0.01).ln();
        assert!(approx_eq(tg, expected, 0.1));
    }

    #[test]
    fn test_relaxation_map_default() {
        let rm = RelaxationMap::default();
        assert!(rm.is_empty());
    }

    // --- MixingRules tests ---

    #[test]
    fn test_maxwell_garnett_zero_fraction() {
        let result = MixingRules::maxwell_garnett(10.0, 80.0, 0.0);
        assert!(approx_eq(result, 10.0, TOL));
    }

    #[test]
    fn test_maxwell_garnett_small_fraction() {
        let result = MixingRules::maxwell_garnett(10.0, 80.0, 0.01);
        assert!(result > 10.0);
        assert!(result < 80.0);
    }

    #[test]
    fn test_bruggeman_equal_fractions() {
        // With equal fractions of same material, should return that material
        let result = MixingRules::bruggeman(10.0, 10.0, 0.5);
        assert!(approx_eq(result, 10.0, 0.1));
    }

    #[test]
    fn test_bruggeman_bounds() {
        let result = MixingRules::bruggeman(5.0, 80.0, 0.3);
        assert!(result > 5.0);
        assert!(result < 80.0);
    }

    #[test]
    fn test_lichtenecker_log_mixing() {
        // k=0: geometric mean for equal fractions
        let result = MixingRules::lichtenecker(4.0, 16.0, 0.5, 0.0);
        let expected = (4.0_f64 * 16.0).sqrt();
        assert!(approx_eq(result, expected, 0.01));
    }

    #[test]
    fn test_lichtenecker_k1() {
        // k=1: linear mixing (parallel model)
        let result = MixingRules::lichtenecker(4.0, 16.0, 0.5, 1.0);
        assert!(approx_eq(result, 10.0, TOL));
    }

    #[test]
    fn test_parallel_model() {
        let result = MixingRules::parallel_model(4.0, 16.0, 0.5);
        assert!(approx_eq(result, 10.0, TOL));
    }

    #[test]
    fn test_series_model() {
        let result = MixingRules::series_model(4.0, 16.0, 0.5);
        // 1/eps = 0.5/4 + 0.5/16 = 0.125 + 0.03125 = 0.15625 => eps = 6.4
        assert!(approx_eq(result, 6.4, TOL));
    }

    #[test]
    fn test_wiener_bounds() {
        // Series < Lichtenecker(k=0) < Parallel
        let eps_a = 4.0;
        let eps_b = 20.0;
        let f = 0.3;
        let lower = MixingRules::series_model(eps_a, eps_b, f);
        let log_mix = MixingRules::lichtenecker(eps_a, eps_b, f, 0.0);
        let upper = MixingRules::parallel_model(eps_a, eps_b, f);
        assert!(lower < log_mix);
        assert!(log_mix < upper);
    }

    // --- DielectricModulus tests ---

    #[test]
    fn test_modulus_from_permittivity() {
        let spec = DielectricSpectrum::new(
            vec![1e6],
            vec![5.0],
            vec![0.0],
        );
        let modulus = DielectricModulus::from_permittivity(&spec);
        // M' = 5/(25+0) = 0.2, M'' = 0
        assert!(approx_eq(modulus.m_real[0], 0.2, TOL));
        assert!(approx_eq(modulus.m_imag[0], 0.0, TOL));
    }

    #[test]
    fn test_modulus_roundtrip() {
        let spec = DielectricSpectrum::new(
            vec![1e6],
            vec![4.0],
            vec![3.0],
        );
        let modulus = spec.to_modulus();
        // Check: eps' = M'/(M'^2+M''^2) should give back original
        let m_mag_sq = modulus.m_real[0].powi(2) + modulus.m_imag[0].powi(2);
        let eps_re_back = modulus.m_real[0] / m_mag_sq;
        let eps_im_back = modulus.m_imag[0] / m_mag_sq;
        assert!(approx_eq(eps_re_back, 4.0, TOL));
        assert!(approx_eq(eps_im_back, 3.0, TOL));
    }

    #[test]
    fn test_modulus_peak_frequency() {
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e3, 1e9), 100);
        let modulus = spec.to_modulus();
        let f_peak = modulus.peak_frequency();
        assert!(f_peak > 1e3);
        assert!(f_peak < 1e9);
    }

    #[test]
    fn test_modulus_new() {
        let modulus = DielectricModulus::new(
            vec![1e6],
            vec![0.1],
            vec![0.05],
        );
        assert_eq!(modulus.frequency_hz.len(), 1);
    }

    #[test]
    fn test_kww_fit() {
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e3, 1e9), 60);
        let modulus = spec.to_modulus();
        let kww = DielectricModulus::kww_fit(&modulus);
        assert!(kww.tau > 0.0);
        assert!(kww.beta_kww > 0.0 && kww.beta_kww <= 1.0);
    }

    // --- DielectricSimulator tests ---

    #[test]
    fn test_simulate_debye_length() {
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1.0, 1e9), 100);
        assert_eq!(spec.len(), 100);
    }

    #[test]
    fn test_simulate_debye_values() {
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e-3, 1e12), 200);
        // Low f: eps' close to 80
        assert!(approx_eq(spec.eps_real[0], 80.0, 0.1));
        // High f: eps' close to 5
        let n = spec.len();
        assert!(approx_eq(spec.eps_real[n - 1], 5.0, 0.1));
    }

    #[test]
    fn test_simulate_hn() {
        let params = HnFitResult {
            eps_inf: 3.0,
            delta_eps: 10.0,
            tau_s: 1e-5,
            alpha: 0.8,
            beta: 0.9,
            chi_squared: 0.0,
        };
        let spec = DielectricSimulator::simulate_hn(&params, (1e1, 1e9), 50);
        assert_eq!(spec.len(), 50);
        assert!(spec.eps_real[0] > 3.0); // Should be > eps_inf
    }

    #[test]
    fn test_simulate_with_conductivity() {
        let base = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e0, 1e6), 30);
        let with_cond = DielectricSimulator::simulate_with_conductivity(&base, 1e-4);
        // eps'' should be larger at low frequencies
        assert!(with_cond.eps_imag[0] > base.eps_imag[0]);
    }

    #[test]
    fn test_add_noise() {
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e3, 1e9), 50);
        let noisy = DielectricSimulator::add_noise(&spec, 5.0);
        // Some points should differ
        let mut any_different = false;
        for i in 0..spec.len() {
            if (noisy.eps_real[i] - spec.eps_real[i]).abs() > 1e-10 {
                any_different = true;
                break;
            }
        }
        assert!(any_different);
    }

    #[test]
    fn test_add_noise_preserves_length() {
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e3, 1e9), 50);
        let noisy = DielectricSimulator::add_noise(&spec, 10.0);
        assert_eq!(noisy.len(), spec.len());
    }

    // --- FrequencyAnalysis tests ---

    #[test]
    fn test_peak_finder_single_peak() {
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e3, 1e9), 100);
        let peaks = FrequencyAnalysis::peak_finder(&spec.eps_imag, &spec.frequency_hz);
        assert!(!peaks.is_empty());
    }

    #[test]
    fn test_peak_finder_peak_near_expected() {
        let tau = 1e-6;
        let f_expected = 1.0 / (2.0 * PI * tau);
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, tau, (1e3, 1e9), 200);
        let peaks = FrequencyAnalysis::peak_finder(&spec.eps_imag, &spec.frequency_hz);
        assert!(!peaks.is_empty());
        // The peak should be near f_expected
        let (f_peak, _) = peaks[0];
        assert!(relative_eq(f_peak, f_expected, 0.1));
    }

    #[test]
    fn test_kramers_kronig_check() {
        // A proper Debye spectrum should be KK-consistent
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e3, 1e9), 100);
        let error = FrequencyAnalysis::kramers_kronig_check(&spec);
        assert!(error.is_finite());
        // The error should be relatively small for consistent data
        assert!(error < 1.0);
    }

    #[test]
    fn test_derivative_analysis_length() {
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e3, 1e9), 50);
        let deriv = FrequencyAnalysis::derivative_analysis(&spec);
        assert_eq!(deriv.len(), spec.len());
    }

    #[test]
    fn test_derivative_analysis_sign_change() {
        // The derivative of eps'' should change sign around the peak
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e3, 1e9), 100);
        let deriv = FrequencyAnalysis::derivative_analysis(&spec);
        let mut has_positive = false;
        let mut has_negative = false;
        for &(_, d) in &deriv {
            if d > 0.1 {
                has_positive = true;
            }
            if d < -0.1 {
                has_negative = true;
            }
        }
        assert!(has_positive && has_negative);
    }

    #[test]
    fn test_strength_from_integral() {
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e0, 1e12), 500);
        let delta_eps_est = FrequencyAnalysis::strength_from_integral(
            &spec,
            spec.frequency_hz[0],
            *spec.frequency_hz.last().unwrap(),
        );
        // Should be close to delta_eps = 75
        assert!(relative_eq(delta_eps_est, 75.0, 0.15));
    }

    #[test]
    fn test_strength_from_integral_partial_range() {
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e0, 1e12), 500);
        let partial = FrequencyAnalysis::strength_from_integral(&spec, 1e4, 1e8);
        let full = FrequencyAnalysis::strength_from_integral(
            &spec,
            spec.frequency_hz[0],
            *spec.frequency_hz.last().unwrap(),
        );
        assert!(partial < full);
        assert!(partial > 0.0);
    }

    // --- Integration tests ---

    #[test]
    fn test_full_pipeline_debye() {
        // Simulate -> fit -> compare
        let spec = DielectricSimulator::simulate_debye(50.0, 3.0, 1e-7, (1e3, 1e12), 80);
        let fit = DebyeRelaxation::fit_debye(&spec);
        assert!(relative_eq(fit.eps_s, 50.0, RTOL));
        assert!(relative_eq(fit.eps_inf, 3.0, RTOL));
    }

    #[test]
    fn test_full_pipeline_hn() {
        let params = HnFitResult {
            eps_inf: 5.0,
            delta_eps: 20.0,
            tau_s: 1e-4,
            alpha: 0.9,
            beta: 0.8,
            chi_squared: 0.0,
        };
        let spec = DielectricSimulator::simulate_hn(&params, (1e0, 1e8), 80);
        let fit = HavriliakNegami::fit(&spec);
        assert!(relative_eq(fit.eps_inf, 5.0, 0.2));
        assert!(relative_eq(fit.delta_eps, 20.0, 0.2));
    }

    #[test]
    fn test_conductivity_subtraction_roundtrip() {
        let sigma_dc = 1e-4;
        let base = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e2, 1e8), 50);
        let with_cond = DielectricSimulator::simulate_with_conductivity(&base, sigma_dc);
        let cleaned = ConductivityAnalysis::subtract_conductivity(&with_cond, sigma_dc);
        // High-frequency eps'' should be similar to base
        let n = base.len();
        assert!(relative_eq(
            cleaned.eps_imag[n - 1],
            base.eps_imag[n - 1],
            0.1
        ));
    }

    #[test]
    fn test_modulus_conductivity_relaxation() {
        // The M'' peak should shift with sigma_DC
        let base1 = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e3, 1e9), 100);
        let with_cond1 = DielectricSimulator::simulate_with_conductivity(&base1, 1e-5);
        let with_cond2 = DielectricSimulator::simulate_with_conductivity(&base1, 1e-3);
        let mod1 = with_cond1.to_modulus();
        let mod2 = with_cond2.to_modulus();
        let fp1 = mod1.peak_frequency();
        let fp2 = mod2.peak_frequency();
        // Higher conductivity -> higher M'' peak frequency
        assert!(fp2 > fp1 || (fp2 - fp1).abs() < 1.0);
    }

    #[test]
    fn test_eps0_constant() {
        assert!(approx_eq(EPS0, 8.854e-12, 1e-15));
    }

    #[test]
    fn test_kb_constant() {
        assert!(approx_eq(KB_EV, 8.617e-5, 1e-8));
    }

    #[test]
    fn test_spectrum_frequencies_monotonic() {
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e3, 1e9), 50);
        for i in 1..spec.len() {
            assert!(spec.frequency_hz[i] > spec.frequency_hz[i - 1]);
        }
    }

    #[test]
    fn test_debye_eps_imag_positive() {
        let spec = DielectricSimulator::simulate_debye(80.0, 5.0, 1e-6, (1e3, 1e9), 50);
        for &ei in &spec.eps_imag {
            assert!(ei >= 0.0);
        }
    }

    #[test]
    fn test_hn_eps_imag_positive() {
        let params = HnFitResult {
            eps_inf: 3.0,
            delta_eps: 10.0,
            tau_s: 1e-5,
            alpha: 0.7,
            beta: 0.6,
            chi_squared: 0.0,
        };
        let spec = DielectricSimulator::simulate_hn(&params, (1e1, 1e9), 50);
        for &ei in &spec.eps_imag {
            assert!(ei >= 0.0);
        }
    }

    #[test]
    fn test_mixing_rules_fraction_zero_gives_host() {
        let mg = MixingRules::maxwell_garnett(10.0, 80.0, 0.0);
        assert!(approx_eq(mg, 10.0, TOL));
        let par = MixingRules::parallel_model(10.0, 80.0, 0.0);
        assert!(approx_eq(par, 80.0, TOL)); // fraction_a=0 -> all b
        let ser = MixingRules::series_model(10.0, 80.0, 0.0);
        assert!(approx_eq(ser, 80.0, TOL));
    }

    #[test]
    fn test_mixing_rules_fraction_one() {
        let mg = MixingRules::maxwell_garnett(10.0, 80.0, 1.0);
        // Approximately equal to inclusion eps for high fraction
        assert!(mg > 10.0);
        let par = MixingRules::parallel_model(10.0, 80.0, 1.0);
        assert!(approx_eq(par, 10.0, TOL));
        let ser = MixingRules::series_model(10.0, 80.0, 1.0);
        assert!(approx_eq(ser, 10.0, TOL));
    }
}
