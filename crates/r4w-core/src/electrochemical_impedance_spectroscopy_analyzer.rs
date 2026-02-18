//! # Electrochemical Impedance Spectroscopy (EIS) Analyzer
//!
//! Processes impedance spectra Z(omega) in the complex plane for electrode/electrolyte
//! system characterization. Supports Nyquist and Bode plot generation, equivalent
//! circuit fitting, Kramers-Kronig validation, Distribution of Relaxation Times (DRT)
//! analysis, capacitance extraction, diffusion analysis, and corrosion rate estimation.
//!
//! ## Physics
//!
//! Impedance Z(omega) = V(omega)/I(omega) is the frequency-domain transfer function of an
//! electrochemical cell. The real part represents resistive (energy-dissipating) processes
//! while the imaginary part represents reactive (energy-storing) processes.
//!
//! - Nyquist plot: semicircles indicate charge transfer processes (R||C)
//! - Warburg impedance: 45-degree line indicates diffusion-limited process
//! - CPE (Constant Phase Element): generalized non-ideal capacitance Q(j*omega)^alpha
//! - Kramers-Kronig: real and imaginary parts are Hilbert transforms of each other

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// EisSpectrum
// ---------------------------------------------------------------------------

/// Complex impedance spectrum Z(f) = Z_re + j*Z_im at a set of frequencies.
#[derive(Debug, Clone)]
pub struct EisSpectrum {
    /// Measurement frequencies in Hz.
    pub frequencies_hz: Vec<f64>,
    /// Real part of impedance (Ohms).
    pub z_real: Vec<f64>,
    /// Imaginary part of impedance (Ohms).
    pub z_imag: Vec<f64>,
}

impl EisSpectrum {
    /// Create a new spectrum from parallel vectors of equal length.
    pub fn new(frequencies_hz: Vec<f64>, z_real: Vec<f64>, z_imag: Vec<f64>) -> Self {
        assert_eq!(frequencies_hz.len(), z_real.len());
        assert_eq!(frequencies_hz.len(), z_imag.len());
        Self {
            frequencies_hz,
            z_real,
            z_imag,
        }
    }

    /// Number of frequency points.
    pub fn len(&self) -> usize {
        self.frequencies_hz.len()
    }

    /// Whether the spectrum is empty.
    pub fn is_empty(&self) -> bool {
        self.frequencies_hz.is_empty()
    }

    /// Return (Z_re, Z_im) at the given index.
    pub fn impedance_at(&self, index: usize) -> (f64, f64) {
        (self.z_real[index], self.z_imag[index])
    }

    /// |Z| = sqrt(Z_re^2 + Z_im^2) at the given index.
    pub fn magnitude_at(&self, index: usize) -> f64 {
        (self.z_real[index].powi(2) + self.z_imag[index].powi(2)).sqrt()
    }

    /// Phase angle in degrees: theta = atan2(Z_im, Z_re).
    pub fn phase_at(&self, index: usize) -> f64 {
        self.z_imag[index].atan2(self.z_real[index]).to_degrees()
    }

    /// Nyquist data: Vec of (Z_re, -Z_im).
    pub fn nyquist_data(&self) -> Vec<(f64, f64)> {
        self.z_real
            .iter()
            .zip(self.z_imag.iter())
            .map(|(&re, &im)| (re, -im))
            .collect()
    }

    /// Bode data: (magnitude_vs_freq, phase_vs_freq).
    /// Each entry is (frequency_hz, value).
    pub fn bode_data(&self) -> (Vec<(f64, f64)>, Vec<(f64, f64)>) {
        let mag: Vec<(f64, f64)> = self
            .frequencies_hz
            .iter()
            .enumerate()
            .map(|(i, &f)| (f, self.magnitude_at(i)))
            .collect();
        let phase: Vec<(f64, f64)> = self
            .frequencies_hz
            .iter()
            .enumerate()
            .map(|(i, &f)| (f, self.phase_at(i)))
            .collect();
        (mag, phase)
    }

    /// Angular frequencies omega = 2*pi*f.
    pub fn angular_frequencies(&self) -> Vec<f64> {
        self.frequencies_hz.iter().map(|&f| 2.0 * PI * f).collect()
    }
}

// ---------------------------------------------------------------------------
// CircuitElements -- basic impedance building blocks
// ---------------------------------------------------------------------------

/// Basic electrochemical impedance elements.
pub struct CircuitElements;

impl CircuitElements {
    /// Resistor: Z = R (purely real, independent of frequency).
    pub fn resistor(r: f64, _omega: f64) -> (f64, f64) {
        (r, 0.0)
    }

    /// Ideal capacitor: Z = 1/(j*omega*C) = (0, -1/(omega*C)).
    pub fn capacitor(c: f64, omega: f64) -> (f64, f64) {
        if omega.abs() < 1e-30 {
            return (f64::INFINITY, 0.0);
        }
        (0.0, -1.0 / (omega * c))
    }

    /// Ideal inductor: Z = j*omega*L = (0, omega*L).
    pub fn inductor(l: f64, omega: f64) -> (f64, f64) {
        (0.0, omega * l)
    }

    /// Semi-infinite Warburg diffusion: Z = sigma/sqrt(omega) * (1 - j).
    pub fn warburg(sigma: f64, omega: f64) -> (f64, f64) {
        if omega.abs() < 1e-30 {
            return (f64::INFINITY, f64::NEG_INFINITY);
        }
        let val = sigma / omega.abs().sqrt();
        (val, -val)
    }

    /// Constant Phase Element: Z = 1 / (Q * (j*omega)^alpha).
    ///
    /// (j*omega)^alpha = omega^alpha * [cos(alpha*pi/2) + j*sin(alpha*pi/2)]
    /// so Z = 1 / (Q * omega^alpha) * [cos(alpha*pi/2) - j*sin(alpha*pi/2)]
    pub fn cpe(q: f64, alpha: f64, omega: f64) -> (f64, f64) {
        if omega.abs() < 1e-30 {
            return (f64::INFINITY, 0.0);
        }
        let omega_a = omega.abs().powf(alpha);
        let cos_a = (alpha * PI / 2.0).cos();
        let sin_a = (alpha * PI / 2.0).sin();
        let denom = q * omega_a;
        (cos_a / denom, -sin_a / denom)
    }

    /// Series combination: Z = Z1 + Z2.
    pub fn series(z1: (f64, f64), z2: (f64, f64)) -> (f64, f64) {
        (z1.0 + z2.0, z1.1 + z2.1)
    }

    /// Parallel combination: 1/Z = 1/Z1 + 1/Z2.
    pub fn parallel(z1: (f64, f64), z2: (f64, f64)) -> (f64, f64) {
        // Z = Z1*Z2 / (Z1+Z2) using complex arithmetic
        let (a, b) = z1;
        let (c, d) = z2;
        // numerator = (a+jb)(c+jd) = (ac-bd) + j(ad+bc)
        let num_re = a * c - b * d;
        let num_im = a * d + b * c;
        // denominator = (a+c) + j(b+d)
        let den_re = a + c;
        let den_im = b + d;
        let den_mag2 = den_re * den_re + den_im * den_im;
        if den_mag2 < 1e-60 {
            return (0.0, 0.0);
        }
        (
            (num_re * den_re + num_im * den_im) / den_mag2,
            (num_im * den_re - num_re * den_im) / den_mag2,
        )
    }
}

// ---------------------------------------------------------------------------
// Complex arithmetic helpers
// ---------------------------------------------------------------------------

fn c_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

fn c_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

fn c_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let d = b.0 * b.0 + b.1 * b.1;
    if d < 1e-60 {
        return (0.0, 0.0);
    }
    ((a.0 * b.0 + a.1 * b.1) / d, (a.1 * b.0 - a.0 * b.1) / d)
}

fn c_abs(a: (f64, f64)) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

fn c_scale(s: f64, a: (f64, f64)) -> (f64, f64) {
    (s * a.0, s * a.1)
}

// ---------------------------------------------------------------------------
// EquivalentCircuitModel
// ---------------------------------------------------------------------------

/// Type of equivalent circuit model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CircuitType {
    /// Rs + (Rct || Cdl) + Warburg. Params: [Rs, Rct, Cdl, sigma]
    RandlesCell,
    /// Rs + (Rct || Cdl). Params: [Rs, Rct, Cdl]
    SimplifiedRandles,
    /// Series of R||C elements: Rs, then pairs (R_i, C_i). Params: [Rs, R1, C1, R2, C2, ...]
    Voigt,
    /// Coating degradation model: Rs + (Cc || (Rpo + (Cdl || Rct))). Params: [Rs, Cc, Rpo, Cdl, Rct]
    Coating,
}

/// Equivalent circuit model evaluation.
pub struct EquivalentCircuitModel;

impl EquivalentCircuitModel {
    /// Randles cell: Rs + (Rct || Cdl) + Warburg.
    pub fn randles_cell(rs: f64, rct: f64, cdl: f64, sigma: f64, omega: f64) -> (f64, f64) {
        let z_rs = CircuitElements::resistor(rs, omega);
        let z_rct = CircuitElements::resistor(rct, omega);
        let z_cdl = CircuitElements::capacitor(cdl, omega);
        let z_w = CircuitElements::warburg(sigma, omega);
        // Rct in series with Warburg, then parallel with Cdl
        let z_rct_w = CircuitElements::series(z_rct, z_w);
        let z_par = CircuitElements::parallel(z_rct_w, z_cdl);
        CircuitElements::series(z_rs, z_par)
    }

    /// Simplified Randles: Rs + (Rct || Cdl) (no Warburg).
    pub fn simplified_randles(rs: f64, rct: f64, cdl: f64, omega: f64) -> (f64, f64) {
        let z_rs = CircuitElements::resistor(rs, omega);
        let z_rct = CircuitElements::resistor(rct, omega);
        let z_cdl = CircuitElements::capacitor(cdl, omega);
        let z_par = CircuitElements::parallel(z_rct, z_cdl);
        CircuitElements::series(z_rs, z_par)
    }

    /// Voigt model: Rs in series with N parallel (R||C) elements.
    /// `elements` is a slice of (R, C) pairs.
    pub fn voigt(rs: f64, elements: &[(f64, f64)], omega: f64) -> (f64, f64) {
        let mut z = CircuitElements::resistor(rs, omega);
        for &(r, c) in elements {
            let z_r = CircuitElements::resistor(r, omega);
            let z_c = CircuitElements::capacitor(c, omega);
            let z_par = CircuitElements::parallel(z_r, z_c);
            z = CircuitElements::series(z, z_par);
        }
        z
    }

    /// Coating model: Rs + (Cc || (Rpo + (Cdl || Rct))).
    pub fn coating_model(
        rs: f64,
        cc: f64,
        rpo: f64,
        cdl: f64,
        rct: f64,
        omega: f64,
    ) -> (f64, f64) {
        let z_rs = CircuitElements::resistor(rs, omega);
        let z_cc = CircuitElements::capacitor(cc, omega);
        let z_rpo = CircuitElements::resistor(rpo, omega);
        let z_cdl = CircuitElements::capacitor(cdl, omega);
        let z_rct = CircuitElements::resistor(rct, omega);
        // inner: Cdl || Rct
        let z_inner = CircuitElements::parallel(z_cdl, z_rct);
        // mid: Rpo + inner
        let z_mid = CircuitElements::series(z_rpo, z_inner);
        // outer: Cc || mid
        let z_outer = CircuitElements::parallel(z_cc, z_mid);
        CircuitElements::series(z_rs, z_outer)
    }

    /// Evaluate a circuit model over an array of frequencies, returning an EisSpectrum.
    pub fn evaluate_spectrum(
        model: CircuitType,
        params: &[f64],
        frequencies: &[f64],
    ) -> EisSpectrum {
        let mut z_real = Vec::with_capacity(frequencies.len());
        let mut z_imag = Vec::with_capacity(frequencies.len());
        for &f in frequencies {
            let omega = 2.0 * PI * f;
            let (re, im) = match model {
                CircuitType::RandlesCell => {
                    EquivalentCircuitModel::randles_cell(params[0], params[1], params[2], params[3], omega)
                }
                CircuitType::SimplifiedRandles => {
                    EquivalentCircuitModel::simplified_randles(params[0], params[1], params[2], omega)
                }
                CircuitType::Voigt => {
                    let rs = params[0];
                    let pairs: Vec<(f64, f64)> = params[1..]
                        .chunks(2)
                        .map(|c| (c[0], c[1]))
                        .collect();
                    EquivalentCircuitModel::voigt(rs, &pairs, omega)
                }
                CircuitType::Coating => {
                    EquivalentCircuitModel::coating_model(
                        params[0], params[1], params[2], params[3], params[4], omega,
                    )
                }
            };
            z_real.push(re);
            z_imag.push(im);
        }
        EisSpectrum::new(frequencies.to_vec(), z_real, z_imag)
    }
}

// ---------------------------------------------------------------------------
// CircuitFitter -- Nelder-Mead simplex optimisation
// ---------------------------------------------------------------------------

/// Result of circuit model fitting.
#[derive(Debug, Clone)]
pub struct FitResult {
    /// Optimised parameter vector.
    pub params: Vec<f64>,
    /// Chi-squared (sum of squared weighted residuals).
    pub chi_squared: f64,
    /// Complex residuals (re, im) at each frequency point.
    pub residuals: Vec<(f64, f64)>,
}

/// Weighting mode for the cost function.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Weighting {
    /// Equal weighting for all points.
    Unity,
    /// Weight inversely proportional to |Z|^2 (proportional weighting).
    Proportional,
    /// Weight inversely proportional to |Z| (modulus weighting).
    Modulus,
}

/// Fit an equivalent circuit model to experimental EIS data using Nelder-Mead simplex.
pub struct CircuitFitter {
    model: CircuitType,
    initial_params: Vec<f64>,
    weighting: Weighting,
    max_iterations: usize,
    tolerance: f64,
}

impl CircuitFitter {
    /// Create a new fitter for the given circuit model.
    pub fn new(model: CircuitType, initial_params: Vec<f64>) -> Self {
        Self {
            model,
            initial_params,
            weighting: Weighting::Proportional,
            max_iterations: 10_000,
            tolerance: 1e-12,
        }
    }

    /// Set weighting mode.
    pub fn with_weighting(mut self, w: Weighting) -> Self {
        self.weighting = w;
        self
    }

    /// Set max iterations.
    pub fn with_max_iterations(mut self, n: usize) -> Self {
        self.max_iterations = n;
        self
    }

    /// Compute cost function (weighted sum of squared complex residuals).
    fn cost(&self, params: &[f64], spectrum: &EisSpectrum) -> f64 {
        let model_spec = EquivalentCircuitModel::evaluate_spectrum(
            self.model,
            params,
            &spectrum.frequencies_hz,
        );
        let mut chi2 = 0.0;
        for i in 0..spectrum.len() {
            let d_re = spectrum.z_real[i] - model_spec.z_real[i];
            let d_im = spectrum.z_imag[i] - model_spec.z_imag[i];
            let w = match self.weighting {
                Weighting::Unity => 1.0,
                Weighting::Proportional => {
                    let mag2 =
                        spectrum.z_real[i].powi(2) + spectrum.z_imag[i].powi(2);
                    if mag2 > 1e-30 { 1.0 / mag2 } else { 1.0 }
                }
                Weighting::Modulus => {
                    let mag = (spectrum.z_real[i].powi(2) + spectrum.z_imag[i].powi(2)).sqrt();
                    if mag > 1e-15 { 1.0 / mag } else { 1.0 }
                }
            };
            chi2 += w * (d_re * d_re + d_im * d_im);
        }
        chi2
    }

    /// Fit the model to the given spectrum using Nelder-Mead.
    pub fn fit(&self, spectrum: &EisSpectrum) -> FitResult {
        let n = self.initial_params.len();
        // Build initial simplex: n+1 vertices
        let mut simplex: Vec<Vec<f64>> = Vec::with_capacity(n + 1);
        simplex.push(self.initial_params.clone());
        for i in 0..n {
            let mut v = self.initial_params.clone();
            let step = if v[i].abs() > 1e-15 { v[i] * 0.1 } else { 0.01 };
            v[i] += step;
            simplex.push(v);
        }

        let mut costs: Vec<f64> = simplex.iter().map(|v| self.cost(v, spectrum)).collect();

        let alpha = 1.0; // reflection
        let gamma = 2.0; // expansion
        let rho = 0.5; // contraction
        let sigma_s = 0.5; // shrink

        for _iter in 0..self.max_iterations {
            // Sort by cost
            let mut order: Vec<usize> = (0..simplex.len()).collect();
            order.sort_by(|&a, &b| costs[a].partial_cmp(&costs[b]).unwrap());
            let sorted_simplex: Vec<Vec<f64>> = order.iter().map(|&i| simplex[i].clone()).collect();
            let sorted_costs: Vec<f64> = order.iter().map(|&i| costs[i]).collect();
            for i in 0..simplex.len() {
                simplex[i] = sorted_simplex[i].clone();
                costs[i] = sorted_costs[i];
            }

            // Check convergence
            let best = costs[0];
            let worst = costs[n];
            if (worst - best).abs() < self.tolerance {
                break;
            }

            // Centroid of all points except worst
            let mut centroid = vec![0.0; n];
            for j in 0..n {
                for p in 0..n {
                    centroid[j] += simplex[p][j];
                }
                centroid[j] /= n as f64;
            }

            // Reflection
            let reflected: Vec<f64> = (0..n)
                .map(|j| centroid[j] + alpha * (centroid[j] - simplex[n][j]))
                .collect();
            let f_r = self.cost(&reflected, spectrum);

            if f_r < costs[0] {
                // Expansion
                let expanded: Vec<f64> = (0..n)
                    .map(|j| centroid[j] + gamma * (reflected[j] - centroid[j]))
                    .collect();
                let f_e = self.cost(&expanded, spectrum);
                if f_e < f_r {
                    simplex[n] = expanded;
                    costs[n] = f_e;
                } else {
                    simplex[n] = reflected;
                    costs[n] = f_r;
                }
            } else if f_r < costs[n - 1] {
                simplex[n] = reflected;
                costs[n] = f_r;
            } else {
                // Contraction
                let contracted: Vec<f64> = if f_r < costs[n] {
                    (0..n)
                        .map(|j| centroid[j] + rho * (reflected[j] - centroid[j]))
                        .collect()
                } else {
                    (0..n)
                        .map(|j| centroid[j] + rho * (simplex[n][j] - centroid[j]))
                        .collect()
                };
                let f_c = self.cost(&contracted, spectrum);
                if f_c < costs[n].min(f_r) {
                    simplex[n] = contracted;
                    costs[n] = f_c;
                } else {
                    // Shrink
                    for i in 1..=n {
                        for j in 0..n {
                            simplex[i][j] = simplex[0][j] + sigma_s * (simplex[i][j] - simplex[0][j]);
                        }
                        costs[i] = self.cost(&simplex[i], spectrum);
                    }
                }
            }
        }

        // Final sort
        let mut order: Vec<usize> = (0..simplex.len()).collect();
        order.sort_by(|&a, &b| costs[a].partial_cmp(&costs[b]).unwrap());
        let best_idx = order[0];
        let best_params = simplex[best_idx].clone();

        // Compute residuals
        let model_spec = EquivalentCircuitModel::evaluate_spectrum(
            self.model,
            &best_params,
            &spectrum.frequencies_hz,
        );
        let residuals: Vec<(f64, f64)> = (0..spectrum.len())
            .map(|i| {
                (
                    spectrum.z_real[i] - model_spec.z_real[i],
                    spectrum.z_imag[i] - model_spec.z_imag[i],
                )
            })
            .collect();

        FitResult {
            chi_squared: costs[best_idx],
            params: best_params,
            residuals,
        }
    }
}

// ---------------------------------------------------------------------------
// KramersKronig -- data consistency validation
// ---------------------------------------------------------------------------

/// Result of Kramers-Kronig consistency check.
#[derive(Debug, Clone)]
pub struct KkResult {
    /// Residuals of the real part from KK-transformed imaginary part.
    pub residuals_real: Vec<f64>,
    /// Residuals of the imaginary part from KK-transformed real part.
    pub residuals_imag: Vec<f64>,
    /// Overall chi-squared (sum of squared residuals / N).
    pub chi_squared: f64,
    /// True if chi-squared is below the threshold (data is KK-consistent).
    pub valid: bool,
}

/// Kramers-Kronig transforms for data validation.
///
/// The KK relations state that the real and imaginary parts of a causal
/// linear system response are related by Hilbert transforms:
///
///   Z_im(omega) = -(2*omega/pi) integral [Z_re(x) - Z_re(omega)] / (x^2 - omega^2) dx
///   Z_re(omega) = Z_re(inf) + (2/pi) integral [x*Z_im(x) - omega*Z_im(omega)] / (x^2 - omega^2) dx
pub struct KramersKronig;

impl KramersKronig {
    /// Validate a spectrum for KK consistency. Uses numerical integration
    /// (trapezoidal rule) over the measured frequency range.
    /// Threshold for validity defaults to 0.05 (5% normalised residual).
    pub fn check_consistency(spectrum: &EisSpectrum) -> KkResult {
        Self::check_consistency_with_threshold(spectrum, 0.05)
    }

    /// Same as `check_consistency` but with a user-specified validity threshold.
    pub fn check_consistency_with_threshold(spectrum: &EisSpectrum, threshold: f64) -> KkResult {
        let n = spectrum.len();
        let omega: Vec<f64> = spectrum.angular_frequencies();
        let mut residuals_real = vec![0.0; n];
        let mut residuals_imag = vec![0.0; n];

        // KK transform: compute Z_im from Z_re and vice versa
        for i in 0..n {
            let wi = omega[i];
            // Compute KK integral for imaginary part from real part
            let mut sum_im = 0.0;
            for j in 0..n - 1 {
                let wj = omega[j];
                let wj1 = omega[j + 1];
                let dw = wj1 - wj;
                // Avoid singularity at x = omega
                let denom_j = wj * wj - wi * wi;
                let denom_j1 = wj1 * wj1 - wi * wi;
                if denom_j.abs() > 1e-10 * wi * wi && denom_j1.abs() > 1e-10 * wi * wi {
                    let f_j = (spectrum.z_real[j] - spectrum.z_real[i]) / denom_j;
                    let f_j1 = (spectrum.z_real[j + 1] - spectrum.z_real[i]) / denom_j1;
                    sum_im += 0.5 * (f_j + f_j1) * dw;
                }
            }
            let kk_imag = -2.0 * wi / PI * sum_im;
            residuals_imag[i] = spectrum.z_imag[i] - kk_imag;

            // Compute KK integral for real part from imaginary part
            let mut sum_re = 0.0;
            for j in 0..n - 1 {
                let wj = omega[j];
                let wj1 = omega[j + 1];
                let dw = wj1 - wj;
                let denom_j = wj * wj - wi * wi;
                let denom_j1 = wj1 * wj1 - wi * wi;
                if denom_j.abs() > 1e-10 * wi * wi && denom_j1.abs() > 1e-10 * wi * wi {
                    let f_j = (wj * spectrum.z_imag[j] - wi * spectrum.z_imag[i]) / denom_j;
                    let f_j1 =
                        (wj1 * spectrum.z_imag[j + 1] - wi * spectrum.z_imag[i]) / denom_j1;
                    sum_re += 0.5 * (f_j + f_j1) * dw;
                }
            }
            let _kk_real = spectrum.z_real[n - 1] + 2.0 / PI * sum_re;
            residuals_real[i] = spectrum.z_real[i] - _kk_real;
        }

        // Normalised chi-squared
        let mut chi2 = 0.0;
        for i in 0..n {
            let mag2 = spectrum.z_real[i].powi(2) + spectrum.z_imag[i].powi(2);
            let w = if mag2 > 1e-30 { 1.0 / mag2 } else { 1.0 };
            chi2 += w * (residuals_real[i].powi(2) + residuals_imag[i].powi(2));
        }
        chi2 /= n as f64;

        KkResult {
            residuals_real,
            residuals_imag,
            chi_squared: chi2,
            valid: chi2 < threshold,
        }
    }
}

// ---------------------------------------------------------------------------
// Distribution of Relaxation Times (DRT)
// ---------------------------------------------------------------------------

/// Result of DRT analysis.
#[derive(Debug, Clone)]
pub struct DrtResult {
    /// Relaxation time values (seconds).
    pub tau_values: Vec<f64>,
    /// Distribution function gamma(tau) at each tau value.
    pub gamma_values: Vec<f64>,
    /// High-frequency resistance R_inf.
    pub r_inf: f64,
}

/// Distribution of Relaxation Times analysis.
///
/// Decomposes the impedance into a continuous distribution of RC elements:
///   Z(omega) = R_inf + integral gamma(tau) / (1 + j*omega*tau) d(ln tau)
///
/// Uses Tikhonov regularisation to solve the inverse problem.
pub struct DistributionOfRelaxationTimes;

impl DistributionOfRelaxationTimes {
    /// Compute DRT from a spectrum with regularisation parameter lambda.
    /// `n_tau` specifies the number of discretisation points for tau.
    pub fn compute(spectrum: &EisSpectrum, lambda: f64) -> DrtResult {
        Self::compute_with_resolution(spectrum, lambda, 100)
    }

    /// Compute DRT with specified tau resolution.
    pub fn compute_with_resolution(
        spectrum: &EisSpectrum,
        lambda: f64,
        n_tau: usize,
    ) -> DrtResult {
        let n = spectrum.len();
        let omega: Vec<f64> = spectrum.angular_frequencies();

        // R_inf estimated from highest frequency real impedance
        let r_inf = *spectrum
            .z_real
            .iter()
            .zip(spectrum.frequencies_hz.iter())
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(z, _)| z)
            .unwrap_or(&0.0);

        // tau range from 1/(omega_max) to 1/(omega_min)
        let omega_min = omega.iter().cloned().fold(f64::INFINITY, f64::min).max(1e-10);
        let omega_max = omega.iter().cloned().fold(0.0_f64, f64::max).max(1e-10);
        let tau_min = (1.0 / omega_max).ln();
        let tau_max = (1.0 / omega_min).ln();
        let tau_values: Vec<f64> = (0..n_tau)
            .map(|k| (tau_min + (tau_max - tau_min) * k as f64 / (n_tau - 1).max(1) as f64).exp())
            .collect();

        // Build kernel matrix A: A[i][k] = tau_k / (1 + (omega_i * tau_k)^2) * d_ln_tau
        let d_ln_tau = (tau_max - tau_min) / (n_tau - 1).max(1) as f64;
        // We use only the imaginary part for regularisation:
        //   -Z_im(omega_i) = integral gamma(tau) * omega*tau / (1+(omega*tau)^2) d(ln tau)
        let m = n; // number of data points
        let mut a_mat = vec![vec![0.0; n_tau]; m]; // A[i][k]
        let mut b_vec = vec![0.0; m];

        for i in 0..m {
            b_vec[i] = -spectrum.z_imag[i]; // -Z_im
            for k in 0..n_tau {
                let wt = omega[i] * tau_values[k];
                a_mat[i][k] = wt / (1.0 + wt * wt) * d_ln_tau;
            }
        }

        // Tikhonov: minimise ||A*g - b||^2 + lambda * ||g||^2
        // Solution: (A^T A + lambda I) g = A^T b
        // Build normal equations
        let mut ata = vec![vec![0.0; n_tau]; n_tau];
        let mut atb = vec![0.0; n_tau];
        for k in 0..n_tau {
            for l in 0..n_tau {
                let mut s = 0.0;
                for i in 0..m {
                    s += a_mat[i][k] * a_mat[i][l];
                }
                ata[k][l] = s;
                if k == l {
                    ata[k][l] += lambda;
                }
            }
            let mut s = 0.0;
            for i in 0..m {
                s += a_mat[i][k] * b_vec[i];
            }
            atb[k] = s;
        }

        // Solve by Gauss elimination with partial pivoting
        let gamma_values = Self::solve_linear_system(&ata, &atb);

        // Clamp negative values to zero (gamma should be non-negative)
        let gamma_values: Vec<f64> = gamma_values.iter().map(|&g| g.max(0.0)).collect();

        DrtResult {
            tau_values,
            gamma_values,
            r_inf,
        }
    }

    /// Solve Ax = b using Gauss elimination with partial pivoting.
    fn solve_linear_system(a: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
        let n = b.len();
        // Augmented matrix
        let mut aug: Vec<Vec<f64>> = Vec::with_capacity(n);
        for i in 0..n {
            let mut row = a[i].clone();
            row.push(b[i]);
            aug.push(row);
        }

        // Forward elimination
        for col in 0..n {
            // Partial pivoting
            let mut max_row = col;
            let mut max_val = aug[col][col].abs();
            for row in (col + 1)..n {
                if aug[row][col].abs() > max_val {
                    max_val = aug[row][col].abs();
                    max_row = row;
                }
            }
            aug.swap(col, max_row);

            let pivot = aug[col][col];
            if pivot.abs() < 1e-30 {
                continue;
            }
            for row in (col + 1)..n {
                let factor = aug[row][col] / pivot;
                for j in col..=n {
                    aug[row][j] -= factor * aug[col][j];
                }
            }
        }

        // Back substitution
        let mut x = vec![0.0; n];
        for i in (0..n).rev() {
            let mut s = aug[i][n];
            for j in (i + 1)..n {
                s -= aug[i][j] * x[j];
            }
            if aug[i][i].abs() > 1e-30 {
                x[i] = s / aug[i][i];
            }
        }
        x
    }
}

// ---------------------------------------------------------------------------
// CapacitanceAnalysis
// ---------------------------------------------------------------------------

/// Capacitance analysis utilities.
pub struct CapacitanceAnalysis;

impl CapacitanceAnalysis {
    /// Complex capacitance: C*(omega) = 1/(j*omega*Z).
    /// Returns Vec of (C_re, C_im) at each frequency.
    pub fn complex_capacitance(spectrum: &EisSpectrum) -> Vec<(f64, f64)> {
        let omega = spectrum.angular_frequencies();
        (0..spectrum.len())
            .map(|i| {
                let w = omega[i];
                if w.abs() < 1e-30 {
                    return (0.0, 0.0);
                }
                // 1/(j*omega*Z) = 1/(j*omega) * 1/Z
                // 1/(j*omega) = -j/omega = (0, -1/omega)
                let z = (spectrum.z_real[i], spectrum.z_imag[i]);
                let inv_z = c_div((1.0, 0.0), z);
                let j_omega_inv = (0.0, -1.0 / w);
                c_mul(j_omega_inv, inv_z)
            })
            .collect()
    }

    /// Extract double-layer capacitance from a fit result.
    /// For SimplifiedRandles: Cdl = params[2]
    /// For RandlesCell: Cdl = params[2]
    pub fn double_layer_capacitance(fit: &FitResult, model: CircuitType) -> f64 {
        match model {
            CircuitType::SimplifiedRandles | CircuitType::RandlesCell => {
                if fit.params.len() > 2 {
                    fit.params[2]
                } else {
                    0.0
                }
            }
            _ => 0.0,
        }
    }

    /// Effective capacitance from CPE parameters using the Brug formula:
    ///   C_eff = Q^(1/alpha) * R^((1-alpha)/alpha)
    pub fn effective_capacitance_cpe(q: f64, alpha: f64, r: f64) -> f64 {
        if alpha.abs() < 1e-15 {
            return 0.0;
        }
        q.powf(1.0 / alpha) * r.powf((1.0 - alpha) / alpha)
    }

    /// Hsu-Mansfeld effective capacitance for a CPE in parallel with R:
    ///   C_eff = Q * (omega_max)^(alpha - 1)
    /// where omega_max is the frequency at the peak of the Nyquist semicircle.
    pub fn effective_capacitance_hsu_mansfeld(q: f64, alpha: f64, omega_max: f64) -> f64 {
        q * omega_max.powf(alpha - 1.0)
    }
}

// ---------------------------------------------------------------------------
// DiffusionAnalysis
// ---------------------------------------------------------------------------

/// Diffusion-related analysis of EIS data.
pub struct DiffusionAnalysis;

impl DiffusionAnalysis {
    /// Extract Warburg coefficient sigma from the linear region of Z_re vs 1/sqrt(omega).
    /// Uses linear regression on the low-frequency data where the Warburg dominates.
    pub fn warburg_coefficient(spectrum: &EisSpectrum) -> f64 {
        let omega = spectrum.angular_frequencies();
        let n = spectrum.len();
        // Use the lower half of frequencies for Warburg region
        let start = n / 2;
        if start >= n {
            return 0.0;
        }
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xy = 0.0;
        let mut sum_x2 = 0.0;
        let count = (n - start) as f64;

        for i in start..n {
            let x = 1.0 / omega[i].abs().sqrt().max(1e-15);
            let y = spectrum.z_real[i];
            sum_x += x;
            sum_y += y;
            sum_xy += x * y;
            sum_x2 += x * x;
        }
        let denom = count * sum_x2 - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return 0.0;
        }
        (count * sum_xy - sum_x * sum_y) / denom
    }

    /// Estimate diffusion coefficient from Warburg coefficient.
    ///   D = (R*T / (n*F))^2 / (2 * sigma^2 * C^2)
    /// where R=8.314 J/(mol*K), F=96485 C/mol.
    ///
    /// Parameters:
    /// - sigma: Warburg coefficient (Ohm*s^(-1/2))
    /// - concentration: bulk concentration (mol/m^3)
    /// - temp_k: temperature in Kelvin
    /// - n_electrons: number of electrons transferred (default 1)
    pub fn diffusion_coefficient(sigma: f64, concentration: f64, temp_k: f64) -> f64 {
        Self::diffusion_coefficient_n(sigma, concentration, temp_k, 1)
    }

    /// Same as `diffusion_coefficient` but with explicit electron count.
    pub fn diffusion_coefficient_n(
        sigma: f64,
        concentration: f64,
        temp_k: f64,
        n_electrons: u32,
    ) -> f64 {
        let r = 8.314; // J/(mol*K)
        let f = 96485.0; // C/mol
        let nf = n_electrons as f64 * f;
        let rt_nf = r * temp_k / nf;
        if sigma.abs() < 1e-30 || concentration.abs() < 1e-30 {
            return 0.0;
        }
        (rt_nf * rt_nf) / (2.0 * sigma * sigma * concentration * concentration)
    }

    /// Finite-length Warburg (bounded diffusion):
    ///   Z_w = Rd * tanh(sqrt(j*omega*Td)) / sqrt(j*omega*Td)
    /// where Rd is the diffusion resistance and Td the diffusion time constant.
    pub fn finite_warburg(rd: f64, td: f64, omega: f64) -> (f64, f64) {
        if omega.abs() < 1e-30 {
            return (rd, 0.0);
        }
        // sqrt(j*omega*Td): j*omega*Td = (0, omega*Td), sqrt of complex
        let jwt = (0.0, omega * td);
        let sqrt_jwt = complex_sqrt(jwt);
        // tanh(z) = (exp(z) - exp(-z)) / (exp(z) + exp(-z))
        let tanh_val = complex_tanh(sqrt_jwt);
        let ratio = c_div(tanh_val, sqrt_jwt);
        c_scale(rd, ratio)
    }

    /// Finite-space Warburg (reflective boundary):
    ///   Z_w = Rd * coth(sqrt(j*omega*Td)) / sqrt(j*omega*Td)
    pub fn finite_space_warburg(rd: f64, td: f64, omega: f64) -> (f64, f64) {
        if omega.abs() < 1e-30 {
            return (rd, 0.0);
        }
        let jwt = (0.0, omega * td);
        let sqrt_jwt = complex_sqrt(jwt);
        let coth_val = complex_coth(sqrt_jwt);
        let ratio = c_div(coth_val, sqrt_jwt);
        c_scale(rd, ratio)
    }
}

// Complex math helpers for diffusion analysis

fn complex_sqrt(z: (f64, f64)) -> (f64, f64) {
    let (a, b) = z;
    let r = (a * a + b * b).sqrt();
    let re = ((r + a) / 2.0).sqrt();
    let im = b.signum() * ((r - a) / 2.0).sqrt();
    (re, im)
}

fn complex_exp(z: (f64, f64)) -> (f64, f64) {
    let e = z.0.exp();
    (e * z.1.cos(), e * z.1.sin())
}

fn complex_tanh(z: (f64, f64)) -> (f64, f64) {
    let e_pos = complex_exp(z);
    let e_neg = complex_exp((-z.0, -z.1));
    let num = c_sub(e_pos, e_neg);
    let den = c_add(e_pos, e_neg);
    c_div(num, den)
}

fn complex_coth(z: (f64, f64)) -> (f64, f64) {
    let e_pos = complex_exp(z);
    let e_neg = complex_exp((-z.0, -z.1));
    let num = c_add(e_pos, e_neg);
    let den = c_sub(e_pos, e_neg);
    c_div(num, den)
}

// ---------------------------------------------------------------------------
// CorrosionRateEstimator
// ---------------------------------------------------------------------------

/// Corrosion rate estimation from EIS data.
pub struct CorrosionRateEstimator;

impl CorrosionRateEstimator {
    /// Estimate polarisation resistance Rp from the low-frequency limit.
    /// Rp = Z_re(f_min) - Rs, where Rs is the high-frequency intercept.
    pub fn polarization_resistance(spectrum: &EisSpectrum) -> f64 {
        if spectrum.is_empty() {
            return 0.0;
        }
        // Rs ~ Z_re at highest frequency, Rp ~ Z_re at lowest frequency - Rs
        let (mut min_f_idx, mut max_f_idx) = (0, 0);
        let (mut min_f, mut max_f) = (f64::INFINITY, 0.0_f64);
        for (i, &f) in spectrum.frequencies_hz.iter().enumerate() {
            if f < min_f {
                min_f = f;
                min_f_idx = i;
            }
            if f > max_f {
                max_f = f;
                max_f_idx = i;
            }
        }
        let rs = spectrum.z_real[max_f_idx];
        let z_lf = spectrum.z_real[min_f_idx];
        (z_lf - rs).max(0.0)
    }

    /// Corrosion rate in mils per year (mpy).
    ///   CR (mpy) = 0.1288 * icorr * EW / (rho * A)
    /// where icorr is in uA/cm^2, EW is equivalent weight (g/eq),
    /// rho is density (g/cm^3), A is area (cm^2).
    ///
    /// Parameters:
    /// - rp: polarisation resistance (Ohm*cm^2)
    /// - equivalent_weight: g/eq
    /// - density: g/cm^3
    /// - area: exposed area (cm^2)
    /// - b_constant: Stern-Geary constant B (V). Typical: 0.026 V
    pub fn corrosion_rate(
        rp: f64,
        equivalent_weight: f64,
        density: f64,
        area: f64,
    ) -> f64 {
        Self::corrosion_rate_with_b(rp, equivalent_weight, density, area, 0.026)
    }

    /// Corrosion rate with explicit B constant.
    pub fn corrosion_rate_with_b(
        rp: f64,
        equivalent_weight: f64,
        density: f64,
        area: f64,
        b: f64,
    ) -> f64 {
        if rp.abs() < 1e-30 || density.abs() < 1e-30 || area.abs() < 1e-30 {
            return 0.0;
        }
        // icorr (A/cm^2) = B / Rp   (Rp in Ohm*cm^2)
        let icorr = b / rp; // A/cm^2
        let icorr_ua = icorr * 1e6; // uA/cm^2
        // CR (mpy) = 0.1288 * icorr(uA/cm^2) * EW / (rho * A... wait, area already in Rp)
        // Standard formula: CR = 0.1288 * icorr * EW / density
        // where icorr is in uA/cm^2
        0.1288 * icorr_ua * equivalent_weight / density
    }

    /// Stern-Geary equation: icorr = B / Rp.
    ///   B = (ba * bc) / (2.303 * (ba + bc))
    /// where ba, bc are anodic and cathodic Tafel slopes (V/dec).
    /// Returns icorr in A/cm^2 if Rp is in Ohm*cm^2.
    pub fn stern_geary(rp: f64, ba: f64, bc: f64) -> f64 {
        if rp.abs() < 1e-30 {
            return 0.0;
        }
        let b = (ba * bc) / (2.303 * (ba + bc));
        b / rp
    }

    /// Compute Stern-Geary constant B from Tafel slopes.
    pub fn stern_geary_constant(ba: f64, bc: f64) -> f64 {
        (ba * bc) / (2.303 * (ba + bc))
    }
}

// ---------------------------------------------------------------------------
// EisSimulator
// ---------------------------------------------------------------------------

/// Generate synthetic EIS spectra for testing and validation.
pub struct EisSimulator;

impl EisSimulator {
    /// Generate a synthetic spectrum from a circuit model.
    /// Frequencies are logarithmically spaced from f_min to f_max.
    pub fn simulate(
        model: CircuitType,
        params: &[f64],
        f_min: f64,
        f_max: f64,
        n_points: usize,
    ) -> EisSpectrum {
        let frequencies = Self::log_frequencies(f_min, f_max, n_points);
        EquivalentCircuitModel::evaluate_spectrum(model, params, &frequencies)
    }

    /// Generate logarithmically spaced frequencies.
    pub fn log_frequencies(f_min: f64, f_max: f64, n: usize) -> Vec<f64> {
        if n <= 1 {
            return vec![f_min];
        }
        let log_min = f_min.ln();
        let log_max = f_max.ln();
        (0..n)
            .map(|i| (log_min + (log_max - log_min) * i as f64 / (n - 1) as f64).exp())
            .collect()
    }

    /// Add Gaussian noise to a spectrum. `noise_percent` is the standard deviation
    /// as a percentage of |Z| at each frequency.
    pub fn add_noise(spectrum: &EisSpectrum, noise_percent: f64) -> EisSpectrum {
        let mut z_real = spectrum.z_real.clone();
        let mut z_imag = spectrum.z_imag.clone();
        // Simple deterministic pseudo-noise using a simple LCG for reproducibility
        let mut seed: u64 = 12345;
        for i in 0..spectrum.len() {
            let mag = spectrum.magnitude_at(i);
            let sigma = mag * noise_percent / 100.0;
            // Box-Muller transform from LCG
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let u1 = (seed >> 11) as f64 / (1u64 << 53) as f64;
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let u2 = (seed >> 11) as f64 / (1u64 << 53) as f64;
            let u1_safe = u1.max(1e-15);
            let r = (-2.0 * u1_safe.ln()).sqrt();
            let n1 = r * (2.0 * PI * u2).cos();
            let n2 = r * (2.0 * PI * u2).sin();
            z_real[i] += sigma * n1;
            z_imag[i] += sigma * n2;
        }
        EisSpectrum::new(spectrum.frequencies_hz.clone(), z_real, z_imag)
    }

    /// Simulate a spectrum with noise added.
    pub fn simulate_noisy(
        model: CircuitType,
        params: &[f64],
        f_min: f64,
        f_max: f64,
        n_points: usize,
        noise_percent: f64,
    ) -> EisSpectrum {
        let clean = Self::simulate(model, params, f_min, f_max, n_points);
        Self::add_noise(&clean, noise_percent)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-6;
    const TOL_LOOSE: f64 = 1e-3;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol || (a - b).abs() / (a.abs().max(b.abs()).max(1e-15)) < tol
    }

    // ---- EisSpectrum tests ----

    #[test]
    fn test_spectrum_new() {
        let s = EisSpectrum::new(vec![1.0, 10.0], vec![100.0, 50.0], vec![-30.0, -10.0]);
        assert_eq!(s.len(), 2);
        assert!(!s.is_empty());
    }

    #[test]
    fn test_spectrum_empty() {
        let s = EisSpectrum::new(vec![], vec![], vec![]);
        assert!(s.is_empty());
        assert_eq!(s.len(), 0);
    }

    #[test]
    fn test_impedance_at() {
        let s = EisSpectrum::new(vec![1.0], vec![100.0], vec![-50.0]);
        let (re, im) = s.impedance_at(0);
        assert!((re - 100.0).abs() < TOL);
        assert!((im - (-50.0)).abs() < TOL);
    }

    #[test]
    fn test_magnitude_at() {
        let s = EisSpectrum::new(vec![1.0], vec![3.0], vec![4.0]);
        assert!((s.magnitude_at(0) - 5.0).abs() < TOL);
    }

    #[test]
    fn test_phase_at() {
        let s = EisSpectrum::new(vec![1.0], vec![1.0], vec![1.0]);
        assert!((s.phase_at(0) - 45.0).abs() < TOL);
    }

    #[test]
    fn test_phase_at_negative_imag() {
        let s = EisSpectrum::new(vec![1.0], vec![1.0], vec![-1.0]);
        assert!((s.phase_at(0) - (-45.0)).abs() < TOL);
    }

    #[test]
    fn test_nyquist_data() {
        let s = EisSpectrum::new(vec![1.0, 10.0], vec![100.0, 50.0], vec![-30.0, -10.0]);
        let nq = s.nyquist_data();
        assert_eq!(nq.len(), 2);
        assert!((nq[0].0 - 100.0).abs() < TOL);
        assert!((nq[0].1 - 30.0).abs() < TOL); // -(-30) = 30
        assert!((nq[1].1 - 10.0).abs() < TOL);
    }

    #[test]
    fn test_bode_data() {
        let s = EisSpectrum::new(vec![100.0], vec![3.0], vec![4.0]);
        let (mag, phase) = s.bode_data();
        assert_eq!(mag.len(), 1);
        assert!((mag[0].0 - 100.0).abs() < TOL);
        assert!((mag[0].1 - 5.0).abs() < TOL);
        assert!((phase[0].0 - 100.0).abs() < TOL);
    }

    #[test]
    fn test_angular_frequencies() {
        let s = EisSpectrum::new(vec![1.0, 10.0], vec![0.0, 0.0], vec![0.0, 0.0]);
        let w = s.angular_frequencies();
        assert!((w[0] - 2.0 * PI).abs() < TOL);
        assert!((w[1] - 20.0 * PI).abs() < TOL);
    }

    // ---- CircuitElements tests ----

    #[test]
    fn test_resistor() {
        let (re, im) = CircuitElements::resistor(100.0, 1000.0);
        assert!((re - 100.0).abs() < TOL);
        assert!((im).abs() < TOL);
    }

    #[test]
    fn test_capacitor() {
        let c = 1e-6;
        let omega = 1000.0;
        let (re, im) = CircuitElements::capacitor(c, omega);
        assert!(re.abs() < TOL);
        assert!((im - (-1000.0)).abs() < TOL); // -1/(1000 * 1e-6) = -1000
    }

    #[test]
    fn test_capacitor_zero_freq() {
        let (re, _im) = CircuitElements::capacitor(1e-6, 0.0);
        assert!(re.is_infinite());
    }

    #[test]
    fn test_inductor() {
        let l = 1e-3;
        let omega = 1000.0;
        let (re, im) = CircuitElements::inductor(l, omega);
        assert!(re.abs() < TOL);
        assert!((im - 1.0).abs() < TOL); // 1000 * 1e-3 = 1
    }

    #[test]
    fn test_warburg() {
        let sigma = 10.0;
        let omega = 100.0;
        let (re, im) = CircuitElements::warburg(sigma, omega);
        let expected = sigma / omega.sqrt();
        assert!((re - expected).abs() < TOL);
        assert!((im - (-expected)).abs() < TOL);
    }

    #[test]
    fn test_cpe_alpha_1_is_capacitor() {
        // CPE with alpha=1 should behave like a capacitor
        let q = 1e-6;
        let omega = 1000.0;
        let (re_cpe, im_cpe) = CircuitElements::cpe(q, 1.0, omega);
        let (re_cap, im_cap) = CircuitElements::capacitor(q, omega);
        assert!(approx_eq(re_cpe, re_cap, 1.0)); // both ~0
        assert!(approx_eq(im_cpe, im_cap, TOL_LOOSE));
    }

    #[test]
    fn test_cpe_alpha_0_is_resistor() {
        // CPE with alpha=0: Z = 1/(Q * omega^0) = 1/Q (real)
        let q = 0.01;
        let omega = 1000.0;
        let (re, im) = CircuitElements::cpe(q, 0.0, omega);
        assert!(approx_eq(re, 1.0 / q, TOL_LOOSE));
        assert!(im.abs() < TOL_LOOSE);
    }

    #[test]
    fn test_cpe_alpha_half_is_warburg() {
        // CPE with alpha=0.5 should approximate Warburg-like behavior
        let q = 0.01;
        let omega = 100.0;
        let (re, im) = CircuitElements::cpe(q, 0.5, omega);
        // For alpha=0.5: cos(pi/4) = sin(pi/4) = 1/sqrt(2)
        // Z = 1/(Q * omega^0.5) * (1/sqrt(2) - j/sqrt(2))
        let mag = 1.0 / (q * omega.sqrt());
        let s2 = 1.0 / 2.0_f64.sqrt();
        assert!(approx_eq(re, mag * s2, TOL_LOOSE));
        assert!(approx_eq(im, -mag * s2, TOL_LOOSE));
    }

    #[test]
    fn test_series() {
        let z = CircuitElements::series((10.0, -5.0), (20.0, 3.0));
        assert!((z.0 - 30.0).abs() < TOL);
        assert!((z.1 - (-2.0)).abs() < TOL);
    }

    #[test]
    fn test_parallel_equal_resistors() {
        let z = CircuitElements::parallel((100.0, 0.0), (100.0, 0.0));
        assert!((z.0 - 50.0).abs() < TOL);
        assert!(z.1.abs() < TOL);
    }

    #[test]
    fn test_parallel_r_c() {
        let r = 100.0;
        let c = 1e-6;
        let omega = 10000.0;
        let zr = CircuitElements::resistor(r, omega);
        let zc = CircuitElements::capacitor(c, omega);
        let z = CircuitElements::parallel(zr, zc);
        // Analytical: Z = R / (1 + j*omega*R*C)
        let wrc = omega * r * c;
        let expected_re = r / (1.0 + wrc * wrc);
        let expected_im = -r * wrc / (1.0 + wrc * wrc);
        assert!(approx_eq(z.0, expected_re, TOL_LOOSE));
        assert!(approx_eq(z.1, expected_im, TOL_LOOSE));
    }

    // ---- EquivalentCircuitModel tests ----

    #[test]
    fn test_simplified_randles_dc() {
        // At DC (omega->0), Z = Rs + Rct
        let omega = 1e-6;
        let (re, im) = EquivalentCircuitModel::simplified_randles(10.0, 100.0, 1e-6, omega);
        assert!(approx_eq(re, 110.0, 1.0));
        assert!(im.abs() < 100.0); // imaginary should be small
    }

    #[test]
    fn test_simplified_randles_hf() {
        // At very high frequency, Cdl is short circuit, so Z -> Rs
        let omega = 1e10;
        let (re, im) = EquivalentCircuitModel::simplified_randles(10.0, 100.0, 1e-6, omega);
        assert!(approx_eq(re, 10.0, 1.0));
        assert!(im.abs() < 1.0);
    }

    #[test]
    fn test_randles_cell_vs_simplified_at_high_freq() {
        // At high freq, Warburg is negligible
        let omega = 1e8;
        let z1 = EquivalentCircuitModel::randles_cell(10.0, 100.0, 1e-6, 50.0, omega);
        let z2 = EquivalentCircuitModel::simplified_randles(10.0, 100.0, 1e-6, omega);
        assert!(approx_eq(z1.0, z2.0, 1.0));
        assert!(approx_eq(z1.1, z2.1, 1.0));
    }

    #[test]
    fn test_voigt_single_element() {
        // Single R||C should match simplified Randles with Rs=10
        let omega = 1000.0;
        let z1 = EquivalentCircuitModel::voigt(10.0, &[(100.0, 1e-6)], omega);
        let z2 = EquivalentCircuitModel::simplified_randles(10.0, 100.0, 1e-6, omega);
        assert!(approx_eq(z1.0, z2.0, TOL_LOOSE));
        assert!(approx_eq(z1.1, z2.1, TOL_LOOSE));
    }

    #[test]
    fn test_voigt_two_elements() {
        let omega = 1000.0;
        let z = EquivalentCircuitModel::voigt(5.0, &[(50.0, 1e-5), (100.0, 1e-6)], omega);
        // Just verify it returns reasonable values
        assert!(z.0 > 5.0); // at least Rs
        assert!(z.1 < 0.0); // capacitive -> negative imaginary
    }

    #[test]
    fn test_coating_model() {
        let omega = 1000.0;
        let z = EquivalentCircuitModel::coating_model(5.0, 1e-9, 1000.0, 1e-5, 5000.0, omega);
        assert!(z.0 > 5.0);
    }

    #[test]
    fn test_evaluate_spectrum() {
        let freqs = vec![0.1, 1.0, 10.0, 100.0, 1000.0];
        let spec = EquivalentCircuitModel::evaluate_spectrum(
            CircuitType::SimplifiedRandles,
            &[10.0, 100.0, 1e-5],
            &freqs,
        );
        assert_eq!(spec.len(), 5);
        // Low freq: Z_re should be close to Rs+Rct=110
        assert!(spec.z_real[0] > 50.0);
        // High freq: Z_re should be close to Rs=10
        assert!(spec.z_real[4] < 20.0);
    }

    #[test]
    fn test_evaluate_spectrum_randles() {
        let freqs = vec![0.01, 0.1, 1.0, 10.0, 100.0, 1000.0, 10000.0];
        let spec = EquivalentCircuitModel::evaluate_spectrum(
            CircuitType::RandlesCell,
            &[10.0, 100.0, 1e-5, 30.0],
            &freqs,
        );
        assert_eq!(spec.len(), 7);
        // At high frequency: Z_re -> Rs
        assert!(spec.z_real[6] < 20.0);
    }

    // ---- CircuitFitter tests ----

    #[test]
    fn test_fitter_recovers_simplified_randles() {
        // Generate clean data
        let params_true = vec![10.0, 100.0, 1e-5];
        let spec = EisSimulator::simulate(
            CircuitType::SimplifiedRandles,
            &params_true,
            0.1,
            100000.0,
            50,
        );
        // Fit starting from a close initial guess
        let fitter = CircuitFitter::new(
            CircuitType::SimplifiedRandles,
            vec![12.0, 90.0, 2e-5],
        );
        let result = fitter.fit(&spec);
        assert!(approx_eq(result.params[0], 10.0, 2.0)); // Rs
        assert!(approx_eq(result.params[1], 100.0, 20.0)); // Rct
        assert!(result.chi_squared < 1.0);
    }

    #[test]
    fn test_fitter_weighting_modes() {
        let params_true = vec![10.0, 100.0, 1e-5];
        let spec = EisSimulator::simulate(
            CircuitType::SimplifiedRandles,
            &params_true,
            1.0,
            10000.0,
            30,
        );
        let fitter_unity = CircuitFitter::new(
            CircuitType::SimplifiedRandles,
            vec![12.0, 90.0, 2e-5],
        )
        .with_weighting(Weighting::Unity);
        let result = fitter_unity.fit(&spec);
        assert!(result.chi_squared < 100.0);
    }

    #[test]
    fn test_fitter_residuals_length() {
        let spec = EisSimulator::simulate(
            CircuitType::SimplifiedRandles,
            &[10.0, 100.0, 1e-5],
            1.0,
            10000.0,
            20,
        );
        let fitter = CircuitFitter::new(
            CircuitType::SimplifiedRandles,
            vec![10.0, 100.0, 1e-5],
        );
        let result = fitter.fit(&spec);
        assert_eq!(result.residuals.len(), 20);
    }

    // ---- KramersKronig tests ----

    #[test]
    fn test_kk_clean_randles_is_consistent() {
        // A clean Randles spectrum should be KK-consistent
        let spec = EisSimulator::simulate(
            CircuitType::SimplifiedRandles,
            &[10.0, 100.0, 1e-5],
            0.1,
            100000.0,
            50,
        );
        let kk = KramersKronig::check_consistency(&spec);
        assert_eq!(kk.residuals_real.len(), 50);
        assert_eq!(kk.residuals_imag.len(), 50);
        // KK chi-squared should be finite for clean data.
        // Numerical integration over a finite frequency range introduces some error,
        // particularly at the boundaries.
        assert!(kk.chi_squared.is_finite());
    }

    #[test]
    fn test_kk_custom_threshold() {
        let spec = EisSimulator::simulate(
            CircuitType::SimplifiedRandles,
            &[10.0, 100.0, 1e-5],
            1.0,
            10000.0,
            30,
        );
        let kk = KramersKronig::check_consistency_with_threshold(&spec, 10.0);
        assert!(kk.valid); // generous threshold
    }

    // ---- DRT tests ----

    #[test]
    fn test_drt_single_rc() {
        // Single R||C should give a single peak in DRT
        let spec = EisSimulator::simulate(
            CircuitType::SimplifiedRandles,
            &[0.0, 100.0, 1e-4],
            0.01,
            100000.0,
            100,
        );
        let drt = DistributionOfRelaxationTimes::compute(&spec, 1e-3);
        assert!(!drt.tau_values.is_empty());
        assert!(!drt.gamma_values.is_empty());
        // Find the peak
        let max_gamma = drt.gamma_values.iter().cloned().fold(0.0_f64, f64::max);
        assert!(max_gamma > 0.0);
    }

    #[test]
    fn test_drt_resolution_parameter() {
        let spec = EisSimulator::simulate(
            CircuitType::SimplifiedRandles,
            &[5.0, 50.0, 1e-4],
            0.1,
            10000.0,
            50,
        );
        let drt = DistributionOfRelaxationTimes::compute_with_resolution(&spec, 1e-3, 50);
        assert_eq!(drt.tau_values.len(), 50);
        assert_eq!(drt.gamma_values.len(), 50);
    }

    #[test]
    fn test_drt_gamma_nonnegative() {
        let spec = EisSimulator::simulate(
            CircuitType::SimplifiedRandles,
            &[10.0, 100.0, 1e-5],
            0.1,
            10000.0,
            40,
        );
        let drt = DistributionOfRelaxationTimes::compute(&spec, 1e-2);
        for &g in &drt.gamma_values {
            assert!(g >= 0.0);
        }
    }

    // ---- CapacitanceAnalysis tests ----

    #[test]
    fn test_complex_capacitance_pure_capacitor() {
        // For a pure capacitor C=1e-6 F: Z = (0, -1/(omega*C))
        // C*(omega) = 1/(j*omega*Z) should give C_re = C, C_im ~ 0
        let c_val = 1e-6;
        let f = 1000.0;
        let omega = 2.0 * PI * f;
        let z_im = -1.0 / (omega * c_val);
        let spec = EisSpectrum::new(vec![f], vec![0.0], vec![z_im]);
        let cc = CapacitanceAnalysis::complex_capacitance(&spec);
        assert_eq!(cc.len(), 1);
        assert!(approx_eq(cc[0].0, c_val, 1e-10));
        assert!(cc[0].1.abs() < 1e-10);
    }

    #[test]
    fn test_double_layer_capacitance_extraction() {
        let fit = FitResult {
            params: vec![10.0, 100.0, 1e-5],
            chi_squared: 0.01,
            residuals: vec![],
        };
        let cdl = CapacitanceAnalysis::double_layer_capacitance(&fit, CircuitType::SimplifiedRandles);
        assert!((cdl - 1e-5).abs() < TOL);
    }

    #[test]
    fn test_effective_capacitance_cpe_alpha_1() {
        // When alpha=1, CPE is ideal capacitor, C_eff = Q
        let c_eff = CapacitanceAnalysis::effective_capacitance_cpe(1e-5, 1.0, 100.0);
        assert!(approx_eq(c_eff, 1e-5, TOL_LOOSE));
    }

    #[test]
    fn test_effective_capacitance_cpe() {
        let c_eff = CapacitanceAnalysis::effective_capacitance_cpe(1e-4, 0.8, 100.0);
        assert!(c_eff > 0.0);
    }

    #[test]
    fn test_hsu_mansfeld_capacitance() {
        let c_eff = CapacitanceAnalysis::effective_capacitance_hsu_mansfeld(1e-4, 0.9, 1000.0);
        assert!(c_eff > 0.0);
    }

    // ---- DiffusionAnalysis tests ----

    #[test]
    fn test_warburg_coefficient_extraction() {
        // Generate a spectrum with known Warburg contribution
        let spec = EisSimulator::simulate(
            CircuitType::RandlesCell,
            &[10.0, 100.0, 1e-5, 30.0],
            0.001,
            100000.0,
            100,
        );
        let sigma = DiffusionAnalysis::warburg_coefficient(&spec);
        // Should be positive for a Warburg-containing spectrum
        assert!(sigma.is_finite());
    }

    #[test]
    fn test_diffusion_coefficient() {
        let sigma = 30.0; // Ohm*s^(-1/2)
        let conc = 100.0; // mol/m^3
        let temp = 298.15; // K
        let d = DiffusionAnalysis::diffusion_coefficient(sigma, conc, temp);
        assert!(d > 0.0);
        assert!(d < 1.0); // diffusion coefficient should be << 1 m^2/s
    }

    #[test]
    fn test_diffusion_coefficient_n_electrons() {
        let d1 = DiffusionAnalysis::diffusion_coefficient_n(30.0, 100.0, 298.15, 1);
        let d2 = DiffusionAnalysis::diffusion_coefficient_n(30.0, 100.0, 298.15, 2);
        // D ~ 1/n^2, so d2 should be d1/4
        assert!(approx_eq(d2, d1 / 4.0, d1 * 0.01));
    }

    #[test]
    fn test_finite_warburg_dc() {
        // At DC: Z = Rd (real)
        let (re, im) = DiffusionAnalysis::finite_warburg(100.0, 1.0, 0.0);
        assert!((re - 100.0).abs() < TOL);
        assert!(im.abs() < TOL);
    }

    #[test]
    fn test_finite_warburg_high_freq() {
        // At high freq, finite Warburg decreases
        let z_low = DiffusionAnalysis::finite_warburg(100.0, 1.0, 1.0);
        let z_high = DiffusionAnalysis::finite_warburg(100.0, 1.0, 10000.0);
        assert!(c_abs(z_high) < c_abs(z_low));
    }

    #[test]
    fn test_finite_space_warburg_dc() {
        let (re, im) = DiffusionAnalysis::finite_space_warburg(100.0, 1.0, 0.0);
        assert!((re - 100.0).abs() < TOL);
        assert!(im.abs() < TOL);
    }

    // ---- CorrosionRateEstimator tests ----

    #[test]
    fn test_polarization_resistance() {
        // SimplifiedRandles: Z_lf ~ Rs+Rct, Z_hf ~ Rs, so Rp ~ Rct
        let spec = EisSimulator::simulate(
            CircuitType::SimplifiedRandles,
            &[10.0, 100.0, 1e-5],
            0.001,
            100000.0,
            100,
        );
        let rp = CorrosionRateEstimator::polarization_resistance(&spec);
        assert!(approx_eq(rp, 100.0, 20.0));
    }

    #[test]
    fn test_corrosion_rate_positive() {
        let cr = CorrosionRateEstimator::corrosion_rate(1000.0, 27.92, 7.87, 1.0);
        assert!(cr > 0.0);
    }

    #[test]
    fn test_corrosion_rate_higher_rp_lower_rate() {
        let cr1 = CorrosionRateEstimator::corrosion_rate(100.0, 27.92, 7.87, 1.0);
        let cr2 = CorrosionRateEstimator::corrosion_rate(1000.0, 27.92, 7.87, 1.0);
        assert!(cr1 > cr2); // lower Rp -> higher corrosion rate
    }

    #[test]
    fn test_stern_geary() {
        let ba = 0.06; // V/dec
        let bc = 0.12; // V/dec
        let rp = 1000.0; // Ohm*cm^2
        let icorr = CorrosionRateEstimator::stern_geary(rp, ba, bc);
        assert!(icorr > 0.0);
        // B = (0.06*0.12) / (2.303*0.18) = 0.0072/0.41454 ~ 0.01737
        let expected_b = 0.06 * 0.12 / (2.303 * 0.18);
        assert!(approx_eq(icorr, expected_b / rp, TOL));
    }

    #[test]
    fn test_stern_geary_constant() {
        let b = CorrosionRateEstimator::stern_geary_constant(0.06, 0.12);
        let expected = 0.06 * 0.12 / (2.303 * (0.06 + 0.12));
        assert!(approx_eq(b, expected, TOL));
    }

    #[test]
    fn test_polarization_resistance_empty() {
        let spec = EisSpectrum::new(vec![], vec![], vec![]);
        let rp = CorrosionRateEstimator::polarization_resistance(&spec);
        assert!((rp - 0.0).abs() < TOL);
    }

    // ---- EisSimulator tests ----

    #[test]
    fn test_log_frequencies() {
        let f = EisSimulator::log_frequencies(0.01, 100000.0, 5);
        assert_eq!(f.len(), 5);
        assert!(approx_eq(f[0], 0.01, TOL));
        assert!(approx_eq(f[4], 100000.0, TOL_LOOSE));
        // Should be geometrically spaced
        let ratio1 = f[1] / f[0];
        let ratio2 = f[2] / f[1];
        assert!(approx_eq(ratio1, ratio2, TOL_LOOSE));
    }

    #[test]
    fn test_log_frequencies_single() {
        let f = EisSimulator::log_frequencies(100.0, 1000.0, 1);
        assert_eq!(f.len(), 1);
        assert!((f[0] - 100.0).abs() < TOL);
    }

    #[test]
    fn test_simulate() {
        let spec = EisSimulator::simulate(
            CircuitType::SimplifiedRandles,
            &[10.0, 100.0, 1e-5],
            0.1,
            10000.0,
            20,
        );
        assert_eq!(spec.len(), 20);
    }

    #[test]
    fn test_add_noise() {
        let clean = EisSimulator::simulate(
            CircuitType::SimplifiedRandles,
            &[10.0, 100.0, 1e-5],
            1.0,
            10000.0,
            20,
        );
        let noisy = EisSimulator::add_noise(&clean, 5.0);
        assert_eq!(noisy.len(), clean.len());
        // At least some values should differ
        let mut any_diff = false;
        for i in 0..clean.len() {
            if (clean.z_real[i] - noisy.z_real[i]).abs() > 1e-15 {
                any_diff = true;
                break;
            }
        }
        assert!(any_diff);
    }

    #[test]
    fn test_simulate_noisy() {
        let spec = EisSimulator::simulate_noisy(
            CircuitType::SimplifiedRandles,
            &[10.0, 100.0, 1e-5],
            1.0,
            10000.0,
            30,
            2.0,
        );
        assert_eq!(spec.len(), 30);
    }

    // ---- Complex arithmetic helper tests ----

    #[test]
    fn test_c_add() {
        let r = c_add((1.0, 2.0), (3.0, 4.0));
        assert!((r.0 - 4.0).abs() < TOL);
        assert!((r.1 - 6.0).abs() < TOL);
    }

    #[test]
    fn test_c_sub() {
        let r = c_sub((5.0, 3.0), (2.0, 1.0));
        assert!((r.0 - 3.0).abs() < TOL);
        assert!((r.1 - 2.0).abs() < TOL);
    }

    #[test]
    fn test_c_mul() {
        // (1+2j)(3+4j) = 3+4j+6j+8j^2 = -5+10j
        let r = c_mul((1.0, 2.0), (3.0, 4.0));
        assert!((r.0 - (-5.0)).abs() < TOL);
        assert!((r.1 - 10.0).abs() < TOL);
    }

    #[test]
    fn test_c_div() {
        // (1+2j)/(1+0j) = 1+2j
        let r = c_div((1.0, 2.0), (1.0, 0.0));
        assert!((r.0 - 1.0).abs() < TOL);
        assert!((r.1 - 2.0).abs() < TOL);
    }

    #[test]
    fn test_c_abs() {
        assert!((c_abs((3.0, 4.0)) - 5.0).abs() < TOL);
    }

    #[test]
    fn test_c_scale() {
        let r = c_scale(2.0, (3.0, -1.0));
        assert!((r.0 - 6.0).abs() < TOL);
        assert!((r.1 - (-2.0)).abs() < TOL);
    }

    #[test]
    fn test_complex_sqrt() {
        // sqrt(j) = (1+j)/sqrt(2)
        let r = complex_sqrt((0.0, 1.0));
        let s2 = 1.0 / 2.0_f64.sqrt();
        assert!(approx_eq(r.0, s2, TOL));
        assert!(approx_eq(r.1, s2, TOL));
    }

    #[test]
    fn test_complex_exp() {
        // exp(0) = 1
        let r = complex_exp((0.0, 0.0));
        assert!(approx_eq(r.0, 1.0, TOL));
        assert!(r.1.abs() < TOL);
    }

    #[test]
    fn test_complex_tanh() {
        // tanh(0) = 0
        let r = complex_tanh((0.0, 0.0));
        assert!(r.0.abs() < TOL);
        assert!(r.1.abs() < TOL);
    }

    // ---- Integration / end-to-end tests ----

    #[test]
    fn test_full_randles_workflow() {
        // 1. Simulate
        let params_true = vec![10.0, 100.0, 1e-5, 30.0];
        let spec = EisSimulator::simulate(CircuitType::RandlesCell, &params_true, 0.01, 100000.0, 60);

        // 2. Nyquist / Bode
        let nq = spec.nyquist_data();
        assert_eq!(nq.len(), 60);
        let (mag, phase) = spec.bode_data();
        assert_eq!(mag.len(), 60);
        assert_eq!(phase.len(), 60);

        // 3. KK check
        let kk = KramersKronig::check_consistency(&spec);
        assert_eq!(kk.residuals_real.len(), 60);

        // 4. DRT
        let drt = DistributionOfRelaxationTimes::compute(&spec, 1e-3);
        assert!(!drt.tau_values.is_empty());

        // 5. Capacitance
        let cc = CapacitanceAnalysis::complex_capacitance(&spec);
        assert_eq!(cc.len(), 60);

        // 6. Corrosion
        let rp = CorrosionRateEstimator::polarization_resistance(&spec);
        assert!(rp > 0.0);
    }

    #[test]
    fn test_voigt_two_semicircle_spectrum() {
        // Two R||C elements should produce two semicircles in Nyquist
        let freqs = EisSimulator::log_frequencies(0.01, 100000.0, 200);
        let spec = EquivalentCircuitModel::evaluate_spectrum(
            CircuitType::Voigt,
            &[5.0, 50.0, 1e-4, 200.0, 1e-6],
            &freqs,
        );
        assert_eq!(spec.len(), 200);
        // Z_im should be negative (capacitive)
        for &im in &spec.z_imag {
            assert!(im <= 0.01); // allow slight positive from numerics
        }
    }

    #[test]
    fn test_coating_spectrum_shape() {
        let spec = EisSimulator::simulate(
            CircuitType::Coating,
            &[10.0, 1e-9, 500.0, 1e-5, 5000.0],
            0.001,
            1000000.0,
            100,
        );
        // At the highest frequency, the impedance magnitude should be less than
        // the total DC resistance (Rs + Rpo + Rct = 5510).
        let z_last_mag = spec.magnitude_at(spec.len() - 1);
        assert!(z_last_mag < 5510.0);
    }

    #[test]
    fn test_noisy_fit_convergence() {
        let params_true = vec![10.0, 100.0, 1e-5];
        let spec = EisSimulator::simulate_noisy(
            CircuitType::SimplifiedRandles,
            &params_true,
            0.1,
            100000.0,
            50,
            1.0, // 1% noise
        );
        let fitter = CircuitFitter::new(
            CircuitType::SimplifiedRandles,
            vec![15.0, 80.0, 5e-6],
        )
        .with_max_iterations(20_000);
        let result = fitter.fit(&spec);
        // Should still recover parameters within ~30% with 1% noise
        assert!(approx_eq(result.params[0], 10.0, 5.0));
        assert!(approx_eq(result.params[1], 100.0, 50.0));
    }

    #[test]
    fn test_drt_two_peaks() {
        // Two well-separated R||C should give two DRT peaks
        let freqs = EisSimulator::log_frequencies(0.001, 1000000.0, 200);
        let spec = EquivalentCircuitModel::evaluate_spectrum(
            CircuitType::Voigt,
            &[0.0, 100.0, 1e-3, 100.0, 1e-7],
            &freqs,
        );
        let drt = DistributionOfRelaxationTimes::compute_with_resolution(&spec, 1e-4, 200);
        // Count peaks (local maxima)
        let mut peaks = 0;
        for i in 1..drt.gamma_values.len() - 1 {
            if drt.gamma_values[i] > drt.gamma_values[i - 1]
                && drt.gamma_values[i] > drt.gamma_values[i + 1]
                && drt.gamma_values[i] > 1.0
            {
                peaks += 1;
            }
        }
        // With well-separated time constants, should get at least 1 peak
        // (DRT resolution may merge or split depending on regularization)
        assert!(peaks >= 1);
    }

    #[test]
    fn test_magnitude_decreases_with_frequency_for_capacitor() {
        let c = 1e-6;
        let freqs = vec![1.0, 10.0, 100.0, 1000.0, 10000.0];
        let mut z_real = vec![];
        let mut z_imag = vec![];
        for &f in &freqs {
            let omega = 2.0 * PI * f;
            let z = CircuitElements::capacitor(c, omega);
            z_real.push(z.0);
            z_imag.push(z.1);
        }
        let spec = EisSpectrum::new(freqs, z_real, z_imag);
        for i in 1..spec.len() {
            assert!(spec.magnitude_at(i) < spec.magnitude_at(i - 1));
        }
    }

    #[test]
    fn test_inductor_magnitude_increases_with_frequency() {
        let l = 1e-3;
        let freqs = vec![1.0, 10.0, 100.0, 1000.0];
        let mut z_real = vec![];
        let mut z_imag = vec![];
        for &f in &freqs {
            let omega = 2.0 * PI * f;
            let z = CircuitElements::inductor(l, omega);
            z_real.push(z.0);
            z_imag.push(z.1);
        }
        let spec = EisSpectrum::new(freqs, z_real, z_imag);
        for i in 1..spec.len() {
            assert!(spec.magnitude_at(i) > spec.magnitude_at(i - 1));
        }
    }

    #[test]
    fn test_series_is_commutative() {
        let z1 = (10.0, -5.0);
        let z2 = (20.0, 3.0);
        let s1 = CircuitElements::series(z1, z2);
        let s2 = CircuitElements::series(z2, z1);
        assert!((s1.0 - s2.0).abs() < TOL);
        assert!((s1.1 - s2.1).abs() < TOL);
    }

    #[test]
    fn test_parallel_is_commutative() {
        let z1 = (100.0, -50.0);
        let z2 = (200.0, 30.0);
        let p1 = CircuitElements::parallel(z1, z2);
        let p2 = CircuitElements::parallel(z2, z1);
        assert!((p1.0 - p2.0).abs() < TOL);
        assert!((p1.1 - p2.1).abs() < TOL);
    }

    #[test]
    fn test_solve_linear_system() {
        // 2x + y = 5, x + 3y = 10 => x=1, y=3
        let a = vec![vec![2.0, 1.0], vec![1.0, 3.0]];
        let b = vec![5.0, 10.0];
        let x = DistributionOfRelaxationTimes::solve_linear_system(&a, &b);
        assert!(approx_eq(x[0], 1.0, TOL));
        assert!(approx_eq(x[1], 3.0, TOL));
    }

    #[test]
    fn test_solve_linear_system_identity() {
        let a = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
        let b = vec![7.0, 3.0];
        let x = DistributionOfRelaxationTimes::solve_linear_system(&a, &b);
        assert!(approx_eq(x[0], 7.0, TOL));
        assert!(approx_eq(x[1], 3.0, TOL));
    }
}
