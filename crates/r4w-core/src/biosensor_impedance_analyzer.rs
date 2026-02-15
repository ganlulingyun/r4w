//! Electrochemical Impedance Spectroscopy (EIS) analysis for biosensor applications.
//!
//! Implements impedance data handling, equivalent circuit modeling (Randles circuit,
//! CPE, Warburg diffusion), Levenberg-Marquardt parameter fitting, Nyquist/Bode plot
//! generation, Kramers-Kronig consistency validation, and biosensor-specific analytics
//! (calibration curves, LOD estimation, diffusion coefficient extraction).
//!
//! # Example
//!
//! ```
//! use r4w_core::biosensor_impedance_analyzer::{
//!     ImpedanceData, RandlesParams, randles_impedance, fit_randles,
//!     analyte_calibration_langmuir, limit_of_detection,
//! };
//!
//! // Generate synthetic Randles circuit data
//! let freqs: Vec<f64> = (0..50).map(|i| 10f64.powf(0.1 * i as f64)).collect();
//! let params = RandlesParams { rs: 100.0, rct: 1000.0, cdl: 1e-6, sigma: 50.0 };
//! let data = ImpedanceData::from_randles(&freqs, &params);
//!
//! // Fit parameters back
//! let fitted = fit_randles(&data, 200);
//! assert!((fitted.rs - 100.0).abs() < 20.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex number helpers using (f64, f64) tuples
// ---------------------------------------------------------------------------

/// Complex number as (real, imaginary) tuple.
type Complex = (f64, f64);

/// Complex zero.
const CZERO: Complex = (0.0, 0.0);

/// Complex one.
const CONE: Complex = (1.0, 0.0);

/// Complex addition.
fn cadd(a: Complex, b: Complex) -> Complex {
    (a.0 + b.0, a.1 + b.1)
}

/// Complex subtraction.
fn csub(a: Complex, b: Complex) -> Complex {
    (a.0 - b.0, a.1 - b.1)
}

/// Complex multiplication.
fn cmul(a: Complex, b: Complex) -> Complex {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex division.
fn cdiv(a: Complex, b: Complex) -> Complex {
    let denom = b.0 * b.0 + b.1 * b.1;
    if denom < 1e-300 {
        return (0.0, 0.0);
    }
    ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
}

/// Complex magnitude.
fn cabs(a: Complex) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

/// Complex magnitude squared.
#[allow(dead_code)]
fn cabs2(a: Complex) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Complex phase (radians).
fn carg(a: Complex) -> f64 {
    a.1.atan2(a.0)
}

/// Complex conjugate.
#[allow(dead_code)]
fn cconj(a: Complex) -> Complex {
    (a.0, -a.1)
}

/// Scale complex by real scalar.
fn cscale(s: f64, a: Complex) -> Complex {
    (s * a.0, s * a.1)
}

/// Complex reciprocal: 1/z.
fn cinv(a: Complex) -> Complex {
    cdiv(CONE, a)
}

/// Complex exponential: e^z = e^x * (cos(y) + j*sin(y)).
fn cexp(a: Complex) -> Complex {
    let r = a.0.exp();
    (r * a.1.cos(), r * a.1.sin())
}

/// Complex power: z^p for real p, using polar form.
fn cpow_real(z: Complex, p: f64) -> Complex {
    let r = cabs(z);
    if r < 1e-300 {
        return CZERO;
    }
    let theta = carg(z);
    let rp = r.powf(p);
    (rp * (p * theta).cos(), rp * (p * theta).sin())
}

/// Complex natural logarithm: ln(|z|) + j*arg(z).
#[allow(dead_code)]
fn cln(z: Complex) -> Complex {
    (cabs(z).ln(), carg(z))
}

// ---------------------------------------------------------------------------
// Impedance data container
// ---------------------------------------------------------------------------

/// Measured or simulated impedance spectrum data.
#[derive(Debug, Clone)]
pub struct ImpedanceData {
    /// Frequency points in Hz.
    pub frequency_hz: Vec<f64>,
    /// Real part of impedance (ohms).
    pub z_real: Vec<f64>,
    /// Imaginary part of impedance (ohms).
    pub z_imag: Vec<f64>,
}

impl ImpedanceData {
    /// Create from frequency and complex impedance arrays.
    pub fn new(frequency_hz: Vec<f64>, z_real: Vec<f64>, z_imag: Vec<f64>) -> Self {
        assert_eq!(frequency_hz.len(), z_real.len());
        assert_eq!(frequency_hz.len(), z_imag.len());
        Self { frequency_hz, z_real, z_imag }
    }

    /// Number of frequency points.
    pub fn len(&self) -> usize {
        self.frequency_hz.len()
    }

    /// Whether the data is empty.
    pub fn is_empty(&self) -> bool {
        self.frequency_hz.is_empty()
    }

    /// Impedance magnitude |Z| at each frequency.
    pub fn magnitude(&self) -> Vec<f64> {
        self.z_real.iter().zip(self.z_imag.iter())
            .map(|(&re, &im)| (re * re + im * im).sqrt())
            .collect()
    }

    /// Impedance phase (radians) at each frequency.
    pub fn phase_rad(&self) -> Vec<f64> {
        self.z_real.iter().zip(self.z_imag.iter())
            .map(|(&re, &im)| im.atan2(re))
            .collect()
    }

    /// Impedance phase (degrees) at each frequency.
    pub fn phase_deg(&self) -> Vec<f64> {
        self.phase_rad().iter().map(|&p| p.to_degrees()).collect()
    }

    /// Generate Nyquist plot data: (Z_real, -Z_imag) pairs.
    pub fn nyquist(&self) -> Vec<(f64, f64)> {
        self.z_real.iter().zip(self.z_imag.iter())
            .map(|(&re, &im)| (re, -im))
            .collect()
    }

    /// Generate Bode magnitude data: (frequency_hz, |Z| in ohms).
    pub fn bode_magnitude(&self) -> Vec<(f64, f64)> {
        let mag = self.magnitude();
        self.frequency_hz.iter().zip(mag.iter())
            .map(|(&f, &m)| (f, m))
            .collect()
    }

    /// Generate Bode phase data: (frequency_hz, phase in degrees).
    pub fn bode_phase(&self) -> Vec<(f64, f64)> {
        let phase = self.phase_deg();
        self.frequency_hz.iter().zip(phase.iter())
            .map(|(&f, &p)| (f, p))
            .collect()
    }

    /// Create impedance data from a Randles circuit model.
    pub fn from_randles(freqs: &[f64], params: &RandlesParams) -> Self {
        let mut z_real = Vec::with_capacity(freqs.len());
        let mut z_imag = Vec::with_capacity(freqs.len());
        for &f in freqs {
            let z = randles_impedance(f, params);
            z_real.push(z.0);
            z_imag.push(z.1);
        }
        Self {
            frequency_hz: freqs.to_vec(),
            z_real,
            z_imag,
        }
    }

    /// Get complex impedance at index i.
    pub fn z_at(&self, i: usize) -> Complex {
        (self.z_real[i], self.z_imag[i])
    }
}

// ---------------------------------------------------------------------------
// Circuit element impedances
// ---------------------------------------------------------------------------

/// Impedance of a resistor: Z = R (real).
pub fn z_resistor(r: f64) -> Complex {
    (r, 0.0)
}

/// Impedance of a capacitor: Z = 1/(j*omega*C) = -j/(omega*C).
pub fn z_capacitor(c: f64, freq_hz: f64) -> Complex {
    let omega = 2.0 * PI * freq_hz;
    if omega * c < 1e-300 {
        return (0.0, -1e15);
    }
    (0.0, -1.0 / (omega * c))
}

/// Impedance of an inductor: Z = j*omega*L.
pub fn z_inductor(l: f64, freq_hz: f64) -> Complex {
    let omega = 2.0 * PI * freq_hz;
    (0.0, omega * l)
}

/// Impedance of a Constant Phase Element: Z = 1/(Q*(j*omega)^n).
///
/// When n=1 this is an ideal capacitor (Q=C), n=0.5 is Warburg-like,
/// n=0 is a resistor (Q=1/R).
pub fn z_cpe(q: f64, n: f64, freq_hz: f64) -> Complex {
    let omega = 2.0 * PI * freq_hz;
    if omega < 1e-300 || q < 1e-300 {
        return (1e15, 0.0);
    }
    // (j*omega)^n = omega^n * (cos(n*pi/2) + j*sin(n*pi/2))
    let jw_n = (
        omega.powf(n) * (n * PI / 2.0).cos(),
        omega.powf(n) * (n * PI / 2.0).sin(),
    );
    cinv(cscale(q, jw_n))
}

/// Warburg impedance for semi-infinite diffusion.
///
/// Z_W = sigma * (1 - j) / sqrt(omega)
/// where sigma is the Warburg coefficient (ohm/sqrt(s)).
pub fn z_warburg(sigma: f64, freq_hz: f64) -> Complex {
    let omega = 2.0 * PI * freq_hz;
    if omega < 1e-300 {
        return (1e15, -1e15);
    }
    let sqrt_omega = omega.sqrt();
    (sigma / sqrt_omega, -sigma / sqrt_omega)
}

/// Series combination: Z_total = Z1 + Z2.
pub fn z_series(z1: Complex, z2: Complex) -> Complex {
    cadd(z1, z2)
}

/// Parallel combination: 1/Z_total = 1/Z1 + 1/Z2.
pub fn z_parallel(z1: Complex, z2: Complex) -> Complex {
    let inv_sum = cadd(cinv(z1), cinv(z2));
    cinv(inv_sum)
}

// ---------------------------------------------------------------------------
// Randles circuit model
// ---------------------------------------------------------------------------

/// Parameters for the Randles equivalent circuit.
///
/// Circuit topology: Rs + (Cdl || (Rct + Zw))
/// where Zw is Warburg impedance for semi-infinite diffusion.
#[derive(Debug, Clone, Copy)]
pub struct RandlesParams {
    /// Solution resistance (ohms).
    pub rs: f64,
    /// Charge transfer resistance (ohms).
    pub rct: f64,
    /// Double-layer capacitance (Farads).
    pub cdl: f64,
    /// Warburg coefficient (ohm * s^-0.5).
    pub sigma: f64,
}

impl Default for RandlesParams {
    fn default() -> Self {
        Self {
            rs: 100.0,
            rct: 1000.0,
            cdl: 1e-6,
            sigma: 50.0,
        }
    }
}

/// Compute Randles circuit impedance at a given frequency.
///
/// Z = Rs + (Cdl || (Rct + Zw))
pub fn randles_impedance(freq_hz: f64, p: &RandlesParams) -> Complex {
    let z_rs = z_resistor(p.rs);
    let z_rct = z_resistor(p.rct);
    let z_w = z_warburg(p.sigma, freq_hz);
    let z_cdl = z_capacitor(p.cdl, freq_hz);
    // Faradaic branch: Rct + Zw
    let z_faradaic = z_series(z_rct, z_w);
    // Parallel: Cdl || faradaic
    let z_par = z_parallel(z_cdl, z_faradaic);
    // Total: Rs + parallel
    z_series(z_rs, z_par)
}

// ---------------------------------------------------------------------------
// CPE-based Randles circuit
// ---------------------------------------------------------------------------

/// Parameters for Randles circuit with CPE instead of ideal capacitor.
#[derive(Debug, Clone, Copy)]
pub struct RandlesCpeParams {
    /// Solution resistance (ohms).
    pub rs: f64,
    /// Charge transfer resistance (ohms).
    pub rct: f64,
    /// CPE coefficient Q.
    pub q: f64,
    /// CPE exponent n (0 < n <= 1, n=1 is ideal capacitor).
    pub n: f64,
    /// Warburg coefficient (ohm * s^-0.5).
    pub sigma: f64,
}

/// Compute Randles-CPE circuit impedance.
///
/// Z = Rs + (CPE || (Rct + Zw))
pub fn randles_cpe_impedance(freq_hz: f64, p: &RandlesCpeParams) -> Complex {
    let z_rs = z_resistor(p.rs);
    let z_rct = z_resistor(p.rct);
    let z_w = z_warburg(p.sigma, freq_hz);
    let z_cpe_val = z_cpe(p.q, p.n, freq_hz);
    let z_faradaic = z_series(z_rct, z_w);
    let z_par = z_parallel(z_cpe_val, z_faradaic);
    z_series(z_rs, z_par)
}

// ---------------------------------------------------------------------------
// Levenberg-Marquardt fitting for Randles circuit
// ---------------------------------------------------------------------------

/// Fit Randles circuit parameters to measured impedance data.
///
/// Uses the Levenberg-Marquardt algorithm to minimize
/// sum_i |Z_measured(f_i) - Z_model(f_i)|^2.
///
/// Returns the fitted parameters. `max_iter` controls the maximum
/// number of LM iterations (typically 100-500).
pub fn fit_randles(data: &ImpedanceData, max_iter: usize) -> RandlesParams {
    let n = data.len();
    if n == 0 {
        return RandlesParams::default();
    }

    // Initial guess from data heuristics
    // Rs ~ Z_real at highest frequency
    let rs_init = data.z_real.iter().copied().fold(f64::INFINITY, f64::min).max(1.0);
    // Rct ~ max(Z_real) - Rs (from semicircle diameter)
    let z_real_max = data.z_real.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let rct_init = (z_real_max - rs_init).max(10.0);
    // Cdl ~ 1/(2*pi*f_peak*Rct) where f_peak is at max(-Z_imag)
    let max_neg_imag_idx = data.z_imag.iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .map(|(i, _)| i)
        .unwrap_or(n / 2);
    let f_peak = data.frequency_hz[max_neg_imag_idx].max(1.0);
    let cdl_init = 1.0 / (2.0 * PI * f_peak * rct_init.max(1.0));
    let sigma_init = 50.0;

    // Parameter vector: [rs, rct, cdl, sigma]
    let mut params = [rs_init, rct_init, cdl_init, sigma_init];
    let mut lambda = 1e-3_f64;
    let nu = 10.0_f64;

    let residuals = |p: &[f64; 4]| -> Vec<f64> {
        let rp = RandlesParams { rs: p[0], rct: p[1], cdl: p[2], sigma: p[3] };
        let mut res = Vec::with_capacity(2 * n);
        for i in 0..n {
            let z_model = randles_impedance(data.frequency_hz[i], &rp);
            res.push(data.z_real[i] - z_model.0);
            res.push(data.z_imag[i] - z_model.1);
        }
        res
    };

    let cost = |p: &[f64; 4]| -> f64 {
        residuals(p).iter().map(|r| r * r).sum::<f64>()
    };

    let jacobian = |p: &[f64; 4]| -> Vec<[f64; 4]> {
        let eps = [p[0] * 1e-6 + 1e-10, p[1] * 1e-6 + 1e-10, p[2] * 1e-12 + 1e-16, p[3] * 1e-6 + 1e-10];
        let r0 = residuals(p);
        let m = r0.len();
        let mut jac = vec![[0.0f64; 4]; m];
        for k in 0..4 {
            let mut pp = *p;
            pp[k] += eps[k];
            // Clamp to positive
            pp[k] = pp[k].max(1e-15);
            let r1 = residuals(&pp);
            for j in 0..m {
                jac[j][k] = (r1[j] - r0[j]) / eps[k];
            }
        }
        jac
    };

    let mut current_cost = cost(&params);

    for _iter in 0..max_iter {
        let res = residuals(&params);
        let jac = jacobian(&params);
        let m = res.len();

        // J^T * J (4x4) and J^T * r (4x1)
        let mut jtj = [[0.0f64; 4]; 4];
        let mut jtr = [0.0f64; 4];
        for i in 0..m {
            for a in 0..4 {
                jtr[a] += jac[i][a] * res[i];
                for b in 0..4 {
                    jtj[a][b] += jac[i][a] * jac[i][b];
                }
            }
        }

        // (J^T*J + lambda*diag(J^T*J)) * delta = J^T * r
        let mut aug = [[0.0f64; 5]; 4]; // augmented matrix for Gaussian elimination
        for a in 0..4 {
            for b in 0..4 {
                aug[a][b] = jtj[a][b];
            }
            aug[a][a] += lambda * jtj[a][a].max(1e-10);
            aug[a][4] = jtr[a];
        }

        // Gaussian elimination with partial pivoting
        let mut delta = [0.0f64; 4];
        let solved = gauss_solve_4x4(&mut aug, &mut delta);
        if !solved {
            lambda *= nu;
            continue;
        }

        // Trial update
        let mut trial = params;
        for k in 0..4 {
            trial[k] += delta[k];
            trial[k] = trial[k].max(1e-15); // keep positive
        }

        let trial_cost = cost(&trial);
        if trial_cost < current_cost {
            params = trial;
            current_cost = trial_cost;
            lambda /= nu;
            lambda = lambda.max(1e-12);
        } else {
            lambda *= nu;
            lambda = lambda.min(1e10);
        }

        // Convergence check
        if delta.iter().zip(params.iter()).all(|(d, p)| d.abs() < 1e-10 * p.abs().max(1e-15)) {
            break;
        }
    }

    RandlesParams {
        rs: params[0],
        rct: params[1],
        cdl: params[2],
        sigma: params[3],
    }
}

/// Solve 4x4 linear system via Gaussian elimination with partial pivoting.
/// `aug` is a 4x5 augmented matrix. Returns solution in `x`.
fn gauss_solve_4x4(aug: &mut [[f64; 5]; 4], x: &mut [f64; 4]) -> bool {
    for col in 0..4 {
        // Partial pivoting
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col + 1)..4 {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        if max_val < 1e-30 {
            return false;
        }
        if max_row != col {
            aug.swap(col, max_row);
        }
        // Eliminate below
        for row in (col + 1)..4 {
            let factor = aug[row][col] / aug[col][col];
            for j in col..5 {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }
    // Back substitution
    for col in (0..4).rev() {
        if aug[col][col].abs() < 1e-30 {
            x[col] = 0.0;
            continue;
        }
        let mut sum = aug[col][4];
        for j in (col + 1)..4 {
            sum -= aug[col][j] * x[j];
        }
        x[col] = sum / aug[col][col];
    }
    true
}

// ---------------------------------------------------------------------------
// Nyquist and Bode plot helpers
// ---------------------------------------------------------------------------

/// Generate Nyquist plot points for a Randles circuit over a frequency range.
pub fn randles_nyquist(
    params: &RandlesParams,
    freq_start: f64,
    freq_end: f64,
    num_points: usize,
) -> Vec<(f64, f64)> {
    let log_start = freq_start.log10();
    let log_end = freq_end.log10();
    (0..num_points)
        .map(|i| {
            let t = i as f64 / (num_points - 1).max(1) as f64;
            let f = 10f64.powf(log_start + t * (log_end - log_start));
            let z = randles_impedance(f, params);
            (z.0, -z.1)
        })
        .collect()
}

/// Generate Bode magnitude and phase data for a Randles circuit.
///
/// Returns (frequencies, magnitudes_ohms, phases_degrees).
pub fn randles_bode(
    params: &RandlesParams,
    freq_start: f64,
    freq_end: f64,
    num_points: usize,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let log_start = freq_start.log10();
    let log_end = freq_end.log10();
    let mut freqs = Vec::with_capacity(num_points);
    let mut mags = Vec::with_capacity(num_points);
    let mut phases = Vec::with_capacity(num_points);
    for i in 0..num_points {
        let t = i as f64 / (num_points - 1).max(1) as f64;
        let f = 10f64.powf(log_start + t * (log_end - log_start));
        let z = randles_impedance(f, params);
        freqs.push(f);
        mags.push(cabs(z));
        phases.push(carg(z).to_degrees());
    }
    (freqs, mags, phases)
}

// ---------------------------------------------------------------------------
// Kramers-Kronig validation
// ---------------------------------------------------------------------------

/// Kramers-Kronig residual analysis for data consistency using the
/// linear K-K test (Boukamp's method).
///
/// Fits the measured data to a series of R-C Voigt elements whose
/// time constants are distributed logarithmically across the frequency
/// range. Because each Voigt element is inherently K-K compliant,
/// the residuals between the K-K-compliant fit and the data reveal
/// non-causal or non-linear artifacts.
///
/// Low residuals (< 1-2%) indicate self-consistent data.
pub fn kramers_kronig_residuals(data: &ImpedanceData) -> KramersKronigResult {
    let n = data.len();
    if n < 4 {
        return KramersKronigResult {
            residual_real: vec![],
            residual_imag: vec![],
            mean_residual: 0.0,
            is_valid: true,
        };
    }

    // Number of Voigt elements (use roughly n/2 for good fit without overfitting)
    let m = (n / 2).max(3).min(40);

    // Distribute time constants logarithmically across the frequency range
    let f_min = data.frequency_hz.iter().copied().fold(f64::INFINITY, f64::min).max(1e-6);
    let f_max = data.frequency_hz.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let tau_min = 1.0 / (2.0 * PI * f_max);
    let tau_max = 1.0 / (2.0 * PI * f_min);
    let log_tau_min = tau_min.ln();
    let log_tau_max = tau_max.ln();

    let taus: Vec<f64> = (0..m)
        .map(|k| {
            let t = k as f64 / (m - 1).max(1) as f64;
            (log_tau_min + t * (log_tau_max - log_tau_min)).exp()
        })
        .collect();

    // Build the linear system: Z_model(omega) = R_inf + sum_k R_k / (1 + j*omega*tau_k)
    // For each Voigt element k at frequency i:
    //   Re contribution: R_k / (1 + (omega*tau_k)^2)
    //   Im contribution: -R_k * omega * tau_k / (1 + (omega*tau_k)^2)
    // We solve for R_inf and R_k using least-squares on both real and imag parts.

    // Number of unknowns: m + 1 (R_inf + m resistances)
    let p = m + 1;
    // Build normal equations A^T*A and A^T*b from [real; imag] stacking
    let mut ata = vec![vec![0.0f64; p]; p]; // A^T A
    let mut atb = vec![0.0f64; p];          // A^T b

    // Accumulate A^T A and A^T b row by row to avoid storing full A matrix
    for i in 0..n {
        let omega = 2.0 * PI * data.frequency_hz[i];
        // Real part row
        let mut a_real = vec![0.0f64; p];
        a_real[0] = 1.0; // R_inf coefficient
        for k in 0..m {
            let wt = omega * taus[k];
            a_real[k + 1] = 1.0 / (1.0 + wt * wt);
        }
        let b_real = data.z_real[i];

        // Imag part row
        let mut a_imag = vec![0.0f64; p];
        a_imag[0] = 0.0; // R_inf has no imaginary contribution
        for k in 0..m {
            let wt = omega * taus[k];
            a_imag[k + 1] = -wt / (1.0 + wt * wt);
        }
        let b_imag = data.z_imag[i];

        // Accumulate into normal equations
        for a in 0..p {
            for b in 0..p {
                ata[a][b] += a_real[a] * a_real[b] + a_imag[a] * a_imag[b];
            }
            atb[a] += a_real[a] * b_real + a_imag[a] * b_imag;
        }
    }

    // Add small Tikhonov regularization to stabilize
    for a in 0..p {
        ata[a][a] += 1e-8 * ata[a][a].max(1e-15);
    }

    // Solve normal equations via Cholesky-like approach (simple Gaussian elimination)
    let coeffs = solve_linear_system(&ata, &atb);

    // Compute residuals between measured data and K-K-compliant model
    let mut residual_real = vec![0.0; n];
    let mut residual_imag = vec![0.0; n];

    for i in 0..n {
        let omega = 2.0 * PI * data.frequency_hz[i];
        let mut z_model_re = coeffs[0]; // R_inf
        let mut z_model_im = 0.0;
        for k in 0..m {
            let wt = omega * taus[k];
            let denom = 1.0 + wt * wt;
            z_model_re += coeffs[k + 1] / denom;
            z_model_im += -coeffs[k + 1] * wt / denom;
        }
        let z_mag = (data.z_real[i].powi(2) + data.z_imag[i].powi(2)).sqrt().max(1e-15);
        residual_real[i] = (data.z_real[i] - z_model_re) / z_mag;
        residual_imag[i] = (data.z_imag[i] - z_model_im) / z_mag;
    }

    let sum: f64 = residual_real.iter().chain(residual_imag.iter())
        .map(|r| r.abs())
        .sum();
    let count = 2 * n;
    let mean_residual = if count > 0 { sum / count as f64 } else { 0.0 };

    KramersKronigResult {
        residual_real,
        residual_imag,
        mean_residual,
        is_valid: mean_residual < 0.05, // 5% threshold for validity
    }
}

/// Solve a general NxN linear system Ax = b using Gaussian elimination.
fn solve_linear_system(a: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    let n = b.len();
    // Build augmented matrix
    let mut aug: Vec<Vec<f64>> = (0..n)
        .map(|i| {
            let mut row = a[i].clone();
            row.push(b[i]);
            row
        })
        .collect();

    // Forward elimination with partial pivoting
    for col in 0..n {
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col + 1)..n {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        if max_val < 1e-30 {
            continue;
        }
        aug.swap(col, max_row);
        for row in (col + 1)..n {
            let factor = aug[row][col] / aug[col][col];
            for j in col..=n {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for col in (0..n).rev() {
        if aug[col][col].abs() < 1e-30 {
            x[col] = 0.0;
            continue;
        }
        let mut s = aug[col][n];
        for j in (col + 1)..n {
            s -= aug[col][j] * x[j];
        }
        x[col] = s / aug[col][col];
    }
    x
}

/// Result of Kramers-Kronig consistency analysis.
#[derive(Debug, Clone)]
pub struct KramersKronigResult {
    /// Normalized residual for real part at each frequency.
    pub residual_real: Vec<f64>,
    /// Normalized residual for imaginary part at each frequency.
    pub residual_imag: Vec<f64>,
    /// Mean absolute residual across all points.
    pub mean_residual: f64,
    /// Whether the data passes the K-K consistency check (< 5% mean residual).
    pub is_valid: bool,
}

// ---------------------------------------------------------------------------
// Biosensor-specific analysis
// ---------------------------------------------------------------------------

/// Langmuir isotherm: fractional surface coverage.
///
/// theta = K * C / (1 + K * C)
/// where K is the binding constant (1/M) and C is analyte concentration (M).
pub fn langmuir_isotherm(k: f64, c: f64) -> f64 {
    k * c / (1.0 + k * c)
}

/// Generate an analyte concentration vs Rct calibration curve using Langmuir model.
///
/// Returns (concentrations, rct_values) where:
/// Rct(C) = rct_base + delta_rct * theta(C)
///
/// - `rct_base`: baseline Rct without analyte (ohms)
/// - `delta_rct`: maximum Rct change at saturation (ohms)
/// - `k`: Langmuir binding constant (1/M)
/// - `concentrations`: array of analyte concentrations (M)
pub fn analyte_calibration_langmuir(
    rct_base: f64,
    delta_rct: f64,
    k: f64,
    concentrations: &[f64],
) -> Vec<(f64, f64)> {
    concentrations.iter()
        .map(|&c| {
            let theta = langmuir_isotherm(k, c);
            (c, rct_base + delta_rct * theta)
        })
        .collect()
}

/// Generate a linear calibration curve.
///
/// Rct(C) = rct_base + sensitivity * C
pub fn analyte_calibration_linear(
    rct_base: f64,
    sensitivity: f64,
    concentrations: &[f64],
) -> Vec<(f64, f64)> {
    concentrations.iter()
        .map(|&c| (c, rct_base + sensitivity * c))
        .collect()
}

/// Estimate sensitivity (dRct/dC) from calibration data.
///
/// Uses linear regression on (concentration, Rct) pairs.
/// Returns (sensitivity, intercept) in units of (ohms/M, ohms).
pub fn estimate_sensitivity(calibration: &[(f64, f64)]) -> (f64, f64) {
    let n = calibration.len() as f64;
    if n < 2.0 {
        return (0.0, 0.0);
    }
    let sum_x: f64 = calibration.iter().map(|(c, _)| c).sum();
    let sum_y: f64 = calibration.iter().map(|(_, r)| r).sum();
    let sum_xy: f64 = calibration.iter().map(|(c, r)| c * r).sum();
    let sum_x2: f64 = calibration.iter().map(|(c, _)| c * c).sum();

    let denom = n * sum_x2 - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return (0.0, sum_y / n);
    }
    let slope = (n * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / n;
    (slope, intercept)
}

/// Estimate Limit of Detection (LOD) from calibration data.
///
/// LOD = 3 * sigma_blank / sensitivity
/// where sigma_blank is the standard deviation of blank (no analyte) measurements
/// and sensitivity is the slope of the calibration curve.
///
/// - `blank_rct_values`: repeated Rct measurements at zero concentration
/// - `sensitivity`: dRct/dC slope from calibration curve (ohms/M)
///
/// Returns LOD in concentration units (M).
pub fn limit_of_detection(blank_rct_values: &[f64], sensitivity: f64) -> f64 {
    if blank_rct_values.len() < 2 || sensitivity.abs() < 1e-30 {
        return f64::INFINITY;
    }
    let n = blank_rct_values.len() as f64;
    let mean = blank_rct_values.iter().sum::<f64>() / n;
    let variance = blank_rct_values.iter()
        .map(|&x| (x - mean).powi(2))
        .sum::<f64>() / (n - 1.0);
    let sigma = variance.sqrt();
    3.0 * sigma / sensitivity.abs()
}

// ---------------------------------------------------------------------------
// Capacitance extraction
// ---------------------------------------------------------------------------

/// Effective capacitance from CPE parameters.
///
/// C_eff = Q^(1/n) * R^((1-n)/n)
///
/// - `q`: CPE coefficient
/// - `n`: CPE exponent
/// - `r`: associated resistance (typically Rct)
pub fn effective_capacitance_from_cpe(q: f64, n: f64, r: f64) -> f64 {
    if n.abs() < 1e-15 || q <= 0.0 || r <= 0.0 {
        return 0.0;
    }
    q.powf(1.0 / n) * r.powf((1.0 - n) / n)
}

/// Estimate electrode area from double-layer capacitance.
///
/// A = C_dl / C_dl_specific
///
/// Typical C_dl_specific for gold in aqueous: ~20-40 uF/cm^2.
pub fn electrode_area_from_capacitance(c_dl: f64, c_dl_specific: f64) -> f64 {
    if c_dl_specific.abs() < 1e-30 {
        return 0.0;
    }
    c_dl / c_dl_specific
}

// ---------------------------------------------------------------------------
// Diffusion coefficient extraction
// ---------------------------------------------------------------------------

/// Physical constants for electrochemistry.
pub const FARADAY_CONSTANT: f64 = 96485.33212; // C/mol
pub const GAS_CONSTANT: f64 = 8.314462618;     // J/(mol*K)

/// Extract diffusion coefficient from Warburg coefficient.
///
/// For a simple redox couple with equal concentrations of oxidized and
/// reduced species:
///
/// sigma = RT / (n^2 * F^2 * A * sqrt(2)) * (1/(D_O^0.5 * C_O) + 1/(D_R^0.5 * C_R))
///
/// For the simplified case D_O = D_R = D, C_O = C_R = C:
///
/// D = (RT / (n^2 * F^2 * A * sigma * C * sqrt(2)))^2 / 2
///
/// - `sigma_w`: Warburg coefficient (ohm * s^-0.5)
/// - `n_electrons`: number of electrons transferred
/// - `area_cm2`: electrode area in cm^2
/// - `conc_mol_per_cm3`: bulk concentration in mol/cm^3
/// - `temperature_k`: temperature in Kelvin
///
/// Returns diffusion coefficient in cm^2/s.
pub fn diffusion_coefficient_from_warburg(
    sigma_w: f64,
    n_electrons: f64,
    area_cm2: f64,
    conc_mol_per_cm3: f64,
    temperature_k: f64,
) -> f64 {
    if sigma_w.abs() < 1e-30 || area_cm2 < 1e-30 || conc_mol_per_cm3 < 1e-30 {
        return 0.0;
    }
    let rt = GAS_CONSTANT * temperature_k;
    let nf = n_electrons * FARADAY_CONSTANT;
    // sigma = RT/(n^2*F^2*A*sqrt(2)) * 2/(D^0.5 * C)  (factor of 2 for equal D_O=D_R)
    // D^0.5 = RT * 2 / (n^2 * F^2 * A * sqrt(2) * sigma * C)
    // => D^0.5 = 2*RT / (n^2 * F^2 * A * sqrt(2) * sigma * C)
    let d_sqrt = 2.0 * rt / (nf * nf * area_cm2 * 2.0_f64.sqrt() * sigma_w * conc_mol_per_cm3);
    d_sqrt * d_sqrt
}

/// Compute Warburg coefficient from known diffusion parameters.
///
/// Inverse of `diffusion_coefficient_from_warburg` for the symmetric case.
pub fn warburg_coefficient_from_diffusion(
    d_cm2_per_s: f64,
    n_electrons: f64,
    area_cm2: f64,
    conc_mol_per_cm3: f64,
    temperature_k: f64,
) -> f64 {
    if d_cm2_per_s < 1e-30 || area_cm2 < 1e-30 || conc_mol_per_cm3 < 1e-30 {
        return 0.0;
    }
    let rt = GAS_CONSTANT * temperature_k;
    let nf = n_electrons * FARADAY_CONSTANT;
    2.0 * rt / (nf * nf * area_cm2 * 2.0_f64.sqrt() * d_cm2_per_s.sqrt() * conc_mol_per_cm3)
}

// ---------------------------------------------------------------------------
// Fitting quality metrics
// ---------------------------------------------------------------------------

/// Compute the residual sum of squares between measured and modeled impedance.
pub fn residual_sum_of_squares(data: &ImpedanceData, params: &RandlesParams) -> f64 {
    let mut rss = 0.0;
    for i in 0..data.len() {
        let z_model = randles_impedance(data.frequency_hz[i], params);
        let dr = data.z_real[i] - z_model.0;
        let di = data.z_imag[i] - z_model.1;
        rss += dr * dr + di * di;
    }
    rss
}

/// Compute normalized root mean square error of fit.
pub fn fit_nrmse(data: &ImpedanceData, params: &RandlesParams) -> f64 {
    let rss = residual_sum_of_squares(data, params);
    let n = data.len() as f64;
    if n < 1.0 {
        return 0.0;
    }
    let rmse = (rss / (2.0 * n)).sqrt(); // 2*n because real + imag
    // Normalize by range of |Z|
    let mags = data.magnitude();
    let z_range = mags.iter().copied().fold(f64::NEG_INFINITY, f64::max)
        - mags.iter().copied().fold(f64::INFINITY, f64::min);
    if z_range < 1e-15 { 0.0 } else { rmse / z_range }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-10;

    // --- Complex number helper tests ---

    #[test]
    fn test_complex_add() {
        let a = (1.0, 2.0);
        let b = (3.0, 4.0);
        let c = cadd(a, b);
        assert!((c.0 - 4.0).abs() < TOL);
        assert!((c.1 - 6.0).abs() < TOL);
    }

    #[test]
    fn test_complex_sub() {
        let c = csub((5.0, 7.0), (2.0, 3.0));
        assert!((c.0 - 3.0).abs() < TOL);
        assert!((c.1 - 4.0).abs() < TOL);
    }

    #[test]
    fn test_complex_mul() {
        // (1+2j)*(3+4j) = 3+4j+6j+8j^2 = -5+10j
        let c = cmul((1.0, 2.0), (3.0, 4.0));
        assert!((c.0 - (-5.0)).abs() < TOL);
        assert!((c.1 - 10.0).abs() < TOL);
    }

    #[test]
    fn test_complex_div() {
        // (1+2j)/(3+4j) = (1+2j)(3-4j)/25 = (11+2j)/25
        let c = cdiv((1.0, 2.0), (3.0, 4.0));
        assert!((c.0 - 11.0 / 25.0).abs() < TOL);
        assert!((c.1 - 2.0 / 25.0).abs() < TOL);
    }

    #[test]
    fn test_complex_abs_phase() {
        let z = (3.0, 4.0);
        assert!((cabs(z) - 5.0).abs() < TOL);
        assert!((carg(z) - (4.0_f64).atan2(3.0)).abs() < TOL);
    }

    #[test]
    fn test_complex_inv() {
        // 1/(3+4j) = (3-4j)/25
        let z = cinv((3.0, 4.0));
        assert!((z.0 - 3.0 / 25.0).abs() < TOL);
        assert!((z.1 - (-4.0 / 25.0)).abs() < TOL);
    }

    #[test]
    fn test_complex_exp() {
        // e^(0+j*pi) = -1 + 0j
        let z = cexp((0.0, PI));
        assert!((z.0 - (-1.0)).abs() < 1e-14);
        assert!(z.1.abs() < 1e-14);
    }

    #[test]
    fn test_complex_pow_real() {
        // (1+j)^2 = 2j
        let z = cpow_real((1.0, 1.0), 2.0);
        assert!(z.0.abs() < 1e-14);
        assert!((z.1 - 2.0).abs() < 1e-14);
    }

    // --- Circuit element tests ---

    #[test]
    fn test_z_resistor() {
        let z = z_resistor(100.0);
        assert!((z.0 - 100.0).abs() < TOL);
        assert!(z.1.abs() < TOL);
    }

    #[test]
    fn test_z_capacitor() {
        let f = 1000.0;
        let c_val = 1e-6;
        let z = z_capacitor(c_val, f);
        let expected = -1.0 / (2.0 * PI * f * c_val);
        assert!(z.0.abs() < TOL);
        assert!((z.1 - expected).abs() < 1e-6);
    }

    #[test]
    fn test_z_inductor() {
        let f = 1000.0;
        let l = 1e-3;
        let z = z_inductor(l, f);
        let expected = 2.0 * PI * f * l;
        assert!(z.0.abs() < TOL);
        assert!((z.1 - expected).abs() < 1e-6);
    }

    #[test]
    fn test_z_cpe_ideal_capacitor() {
        // CPE with n=1 should approximate capacitor
        let f = 1000.0;
        let q = 1e-6; // Q = C for n=1
        let z_cpe_val = z_cpe(q, 1.0, f);
        let z_cap = z_capacitor(q, f);
        assert!((z_cpe_val.0 - z_cap.0).abs() < 1e-4);
        assert!((z_cpe_val.1 - z_cap.1).abs() < 1e-4);
    }

    #[test]
    fn test_z_warburg() {
        let sigma = 50.0;
        let f = 100.0;
        let z = z_warburg(sigma, f);
        let omega = 2.0 * PI * f;
        let expected_mag = sigma / omega.sqrt();
        assert!((z.0 - expected_mag).abs() < 1e-8);
        assert!((z.1 - (-expected_mag)).abs() < 1e-8);
        // Warburg has -45 degree phase
        let phase = carg(z).to_degrees();
        assert!((phase - (-45.0)).abs() < 1e-8);
    }

    #[test]
    fn test_z_series() {
        let z1 = (100.0, 0.0);
        let z2 = (0.0, -50.0);
        let z = z_series(z1, z2);
        assert!((z.0 - 100.0).abs() < TOL);
        assert!((z.1 - (-50.0)).abs() < TOL);
    }

    #[test]
    fn test_z_parallel_equal_resistors() {
        // Two 100 ohm resistors in parallel = 50 ohms
        let z = z_parallel((100.0, 0.0), (100.0, 0.0));
        assert!((z.0 - 50.0).abs() < 1e-10);
        assert!(z.1.abs() < 1e-10);
    }

    #[test]
    fn test_z_parallel_rc() {
        // R || C: known formula
        let r = 1000.0;
        let c_val = 1e-6;
        let f = 159.15; // ~ 1/(2*pi*R*C) for R=1000, C=1e-6
        let z = z_parallel(z_resistor(r), z_capacitor(c_val, f));
        // At the time constant frequency, |Z| ~ R/sqrt(2)
        let mag = cabs(z);
        assert!((mag - r / 2.0_f64.sqrt()).abs() < 5.0); // approximate
    }

    // --- Randles circuit tests ---

    #[test]
    fn test_randles_high_frequency() {
        // At very high frequency, Cdl shorts, Z -> Rs
        let params = RandlesParams { rs: 100.0, rct: 1000.0, cdl: 1e-6, sigma: 50.0 };
        let z = randles_impedance(1e8, &params);
        assert!((z.0 - 100.0).abs() < 5.0);
        assert!(z.1.abs() < 20.0);
    }

    #[test]
    fn test_randles_low_frequency() {
        // At very low frequency (but not DC), Z_real -> Rs + Rct + Warburg_real
        let params = RandlesParams { rs: 100.0, rct: 1000.0, cdl: 1e-6, sigma: 0.0 };
        let z = randles_impedance(1e-3, &params);
        // With sigma=0 (no Warburg), at low freq: Z -> Rs + Rct
        assert!((z.0 - 1100.0).abs() < 50.0);
    }

    #[test]
    fn test_randles_semicircle_diameter() {
        // Nyquist semicircle diameter ~ Rct for Randles without Warburg
        let params = RandlesParams { rs: 100.0, rct: 500.0, cdl: 1e-6, sigma: 0.001 };
        let nyquist = randles_nyquist(&params, 0.1, 1e6, 1000);
        let x_min = nyquist.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
        let x_max = nyquist.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
        let diameter = x_max - x_min;
        // Should be approximately Rct
        assert!((diameter - 500.0).abs() < 100.0);
    }

    // --- ImpedanceData tests ---

    #[test]
    fn test_impedance_data_creation() {
        let data = ImpedanceData::new(
            vec![1.0, 10.0, 100.0],
            vec![100.0, 80.0, 50.0],
            vec![-20.0, -40.0, -10.0],
        );
        assert_eq!(data.len(), 3);
        assert!(!data.is_empty());
    }

    #[test]
    fn test_impedance_magnitude() {
        let data = ImpedanceData::new(
            vec![1.0],
            vec![3.0],
            vec![4.0],
        );
        let mag = data.magnitude();
        assert!((mag[0] - 5.0).abs() < TOL);
    }

    #[test]
    fn test_impedance_phase() {
        let data = ImpedanceData::new(
            vec![1.0],
            vec![1.0],
            vec![-1.0],
        );
        let phase = data.phase_deg();
        assert!((phase[0] - (-45.0)).abs() < 1e-10);
    }

    #[test]
    fn test_nyquist_data() {
        let data = ImpedanceData::new(
            vec![1.0, 10.0],
            vec![100.0, 80.0],
            vec![-20.0, -40.0],
        );
        let nyq = data.nyquist();
        assert_eq!(nyq.len(), 2);
        assert!((nyq[0].0 - 100.0).abs() < TOL);
        assert!((nyq[0].1 - 20.0).abs() < TOL); // -(-20) = +20
    }

    #[test]
    fn test_bode_data() {
        let data = ImpedanceData::new(
            vec![1.0, 100.0],
            vec![3.0, 4.0],
            vec![4.0, 3.0],
        );
        let bode_mag = data.bode_magnitude();
        assert!((bode_mag[0].1 - 5.0).abs() < TOL);
        assert!((bode_mag[1].1 - 5.0).abs() < TOL);
    }

    #[test]
    fn test_from_randles() {
        let freqs: Vec<f64> = (0..50).map(|i| 10f64.powf(0.1 * i as f64)).collect();
        let params = RandlesParams::default();
        let data = ImpedanceData::from_randles(&freqs, &params);
        assert_eq!(data.len(), 50);
        // Check first point matches direct calculation
        let z0 = randles_impedance(freqs[0], &params);
        assert!((data.z_real[0] - z0.0).abs() < TOL);
        assert!((data.z_imag[0] - z0.1).abs() < TOL);
    }

    // --- Fitting tests ---

    #[test]
    fn test_fit_randles_basic() {
        let true_params = RandlesParams { rs: 100.0, rct: 1000.0, cdl: 1e-6, sigma: 50.0 };
        let freqs: Vec<f64> = (0..60).map(|i| 10f64.powf(-1.0 + 0.12 * i as f64)).collect();
        let data = ImpedanceData::from_randles(&freqs, &true_params);
        let fitted = fit_randles(&data, 500);
        // Check that fitted parameters are close
        assert!((fitted.rs - 100.0).abs() / 100.0 < 0.15, "Rs: expected ~100, got {}", fitted.rs);
        assert!((fitted.rct - 1000.0).abs() / 1000.0 < 0.15, "Rct: expected ~1000, got {}", fitted.rct);
    }

    #[test]
    fn test_fit_quality_metric() {
        let params = RandlesParams { rs: 100.0, rct: 500.0, cdl: 1e-6, sigma: 30.0 };
        let freqs: Vec<f64> = (0..40).map(|i| 10f64.powf(-1.0 + 0.15 * i as f64)).collect();
        let data = ImpedanceData::from_randles(&freqs, &params);
        // Perfect model should have very small RSS
        let rss = residual_sum_of_squares(&data, &params);
        assert!(rss < 1e-10, "RSS for exact model should be ~0, got {}", rss);
    }

    #[test]
    fn test_fit_nrmse() {
        let params = RandlesParams { rs: 100.0, rct: 500.0, cdl: 1e-6, sigma: 30.0 };
        let freqs: Vec<f64> = (0..40).map(|i| 10f64.powf(-1.0 + 0.15 * i as f64)).collect();
        let data = ImpedanceData::from_randles(&freqs, &params);
        let nrmse = fit_nrmse(&data, &params);
        assert!(nrmse < 1e-10, "NRMSE for exact model should be ~0, got {}", nrmse);
    }

    // --- Biosensor analysis tests ---

    #[test]
    fn test_langmuir_isotherm() {
        let k = 1e6; // 1/M
        // At C = 1/K, theta = 0.5
        let c = 1.0 / k;
        let theta = langmuir_isotherm(k, c);
        assert!((theta - 0.5).abs() < TOL);
    }

    #[test]
    fn test_langmuir_saturation() {
        let k = 1e6;
        // At very high concentration, theta -> 1
        let theta = langmuir_isotherm(k, 1.0); // 1 M (very high)
        assert!(theta > 0.999);
    }

    #[test]
    fn test_langmuir_zero() {
        let theta = langmuir_isotherm(1e6, 0.0);
        assert!(theta.abs() < TOL);
    }

    #[test]
    fn test_calibration_langmuir() {
        let concs = vec![0.0, 1e-9, 1e-8, 1e-7, 1e-6, 1e-5];
        let curve = analyte_calibration_langmuir(500.0, 2000.0, 1e6, &concs);
        assert_eq!(curve.len(), 6);
        // At zero concentration, Rct = rct_base
        assert!((curve[0].1 - 500.0).abs() < TOL);
        // Monotonically increasing
        for i in 1..curve.len() {
            assert!(curve[i].1 >= curve[i - 1].1);
        }
    }

    #[test]
    fn test_calibration_linear() {
        let concs = vec![0.0, 1e-6, 2e-6, 3e-6];
        let sensitivity = 1e8; // ohm/M
        let curve = analyte_calibration_linear(500.0, sensitivity, &concs);
        assert!((curve[0].1 - 500.0).abs() < TOL);
        assert!((curve[1].1 - 600.0).abs() < TOL);
        assert!((curve[2].1 - 700.0).abs() < TOL);
    }

    #[test]
    fn test_estimate_sensitivity() {
        let data: Vec<(f64, f64)> = vec![
            (0.0, 500.0), (1e-6, 600.0), (2e-6, 700.0), (3e-6, 800.0),
        ];
        let (slope, intercept) = estimate_sensitivity(&data);
        assert!((slope - 1e8).abs() < 1e2, "Expected slope ~1e8, got {}", slope);
        assert!((intercept - 500.0).abs() < 1.0, "Expected intercept ~500, got {}", intercept);
    }

    #[test]
    fn test_limit_of_detection() {
        // Blank measurements with some noise
        let blanks = vec![500.0, 502.0, 498.0, 501.0, 499.0, 503.0, 497.0];
        let sensitivity = 1e8; // ohm/M
        let lod = limit_of_detection(&blanks, sensitivity);
        // sigma ~ 2 ohm, LOD = 3*2/1e8 = 6e-8 M
        assert!(lod > 0.0);
        assert!(lod < 1e-6, "LOD should be sub-micromolar, got {}", lod);
    }

    #[test]
    fn test_lod_zero_sensitivity() {
        let blanks = vec![500.0, 502.0, 498.0];
        let lod = limit_of_detection(&blanks, 0.0);
        assert!(lod.is_infinite());
    }

    // --- Capacitance extraction tests ---

    #[test]
    fn test_effective_capacitance_ideal() {
        // For n=1, C_eff = Q^(1/1) * R^(0/1) = Q
        let c_eff = effective_capacitance_from_cpe(1e-6, 1.0, 1000.0);
        assert!((c_eff - 1e-6).abs() < TOL);
    }

    #[test]
    fn test_effective_capacitance_cpe() {
        // For typical biosensor: Q=1e-6, n=0.9, R=1000
        let c_eff = effective_capacitance_from_cpe(1e-6, 0.9, 1000.0);
        assert!(c_eff > 0.0);
        // Should be in the microfarad range
        assert!(c_eff > 1e-8 && c_eff < 1e-4);
    }

    #[test]
    fn test_electrode_area() {
        // 1 uF with 20 uF/cm^2 -> 0.05 cm^2
        let area = electrode_area_from_capacitance(1e-6, 20e-6);
        assert!((area - 0.05).abs() < 1e-10);
    }

    // --- Diffusion coefficient tests ---

    #[test]
    fn test_diffusion_roundtrip() {
        let d_true = 7e-6; // cm^2/s typical for Fe(CN)6
        let n = 1.0;
        let area = 0.07; // cm^2
        let conc = 5e-6; // mol/cm^3 (5 mM)
        let temp = 298.15; // K

        let sigma = warburg_coefficient_from_diffusion(d_true, n, area, conc, temp);
        let d_calc = diffusion_coefficient_from_warburg(sigma, n, area, conc, temp);
        assert!((d_calc - d_true).abs() / d_true < 1e-10,
            "Roundtrip diffusion: expected {}, got {}", d_true, d_calc);
    }

    #[test]
    fn test_warburg_coefficient_positive() {
        let sigma = warburg_coefficient_from_diffusion(7e-6, 1.0, 0.07, 5e-6, 298.15);
        assert!(sigma > 0.0);
    }

    // --- Kramers-Kronig tests ---

    #[test]
    fn test_kk_valid_data() {
        // Generate Randles data (should be K-K consistent)
        let params = RandlesParams { rs: 100.0, rct: 500.0, cdl: 1e-6, sigma: 30.0 };
        let freqs: Vec<f64> = (0..100).map(|i| 10f64.powf(-1.0 + 0.07 * i as f64)).collect();
        let data = ImpedanceData::from_randles(&freqs, &params);
        let kk = kramers_kronig_residuals(&data);
        // Valid circuit data should have low residuals
        assert!(kk.is_valid, "Randles data should pass K-K, mean_residual={}", kk.mean_residual);
    }

    #[test]
    fn test_kk_empty_data() {
        let data = ImpedanceData::new(vec![], vec![], vec![]);
        let kk = kramers_kronig_residuals(&data);
        assert!(kk.is_valid);
    }

    // --- Randles CPE tests ---

    #[test]
    fn test_randles_cpe_n1_matches_ideal() {
        let freq = 100.0;
        let ideal = RandlesParams { rs: 100.0, rct: 1000.0, cdl: 1e-6, sigma: 50.0 };
        let cpe = RandlesCpeParams { rs: 100.0, rct: 1000.0, q: 1e-6, n: 1.0, sigma: 50.0 };
        let z_ideal = randles_impedance(freq, &ideal);
        let z_cpe = randles_cpe_impedance(freq, &cpe);
        assert!((z_ideal.0 - z_cpe.0).abs() < 1e-6, "Real: {} vs {}", z_ideal.0, z_cpe.0);
        assert!((z_ideal.1 - z_cpe.1).abs() < 1e-6, "Imag: {} vs {}", z_ideal.1, z_cpe.1);
    }

    // --- Bode/Nyquist generation tests ---

    #[test]
    fn test_randles_bode_generation() {
        let params = RandlesParams::default();
        let (freqs, mags, phases) = randles_bode(&params, 0.1, 1e6, 100);
        assert_eq!(freqs.len(), 100);
        assert_eq!(mags.len(), 100);
        assert_eq!(phases.len(), 100);
        // All magnitudes should be positive
        assert!(mags.iter().all(|&m| m > 0.0));
        // Phases should be in [-180, 180]
        assert!(phases.iter().all(|&p| p >= -180.0 && p <= 180.0));
    }

    #[test]
    fn test_randles_nyquist_generation() {
        let params = RandlesParams::default();
        let nyq = randles_nyquist(&params, 0.1, 1e6, 200);
        assert_eq!(nyq.len(), 200);
        // Nyquist for Randles: -Z_imag should be mostly positive (upper half-plane)
        let positive_count = nyq.iter().filter(|p| p.1 > 0.0).count();
        assert!(positive_count > nyq.len() / 2, "Most Nyquist points should be in upper half");
    }

    // --- Gauss solver test ---

    #[test]
    fn test_gauss_solve_identity() {
        // [1 0 0 0 | 1]
        // [0 1 0 0 | 2]
        // [0 0 1 0 | 3]
        // [0 0 0 1 | 4]
        let mut aug = [
            [1.0, 0.0, 0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0, 0.0, 2.0],
            [0.0, 0.0, 1.0, 0.0, 3.0],
            [0.0, 0.0, 0.0, 1.0, 4.0],
        ];
        let mut x = [0.0; 4];
        assert!(gauss_solve_4x4(&mut aug, &mut x));
        assert!((x[0] - 1.0).abs() < TOL);
        assert!((x[1] - 2.0).abs() < TOL);
        assert!((x[2] - 3.0).abs() < TOL);
        assert!((x[3] - 4.0).abs() < TOL);
    }

    #[test]
    fn test_gauss_solve_general() {
        // 2x + y = 5, x + 3y = 10 (padded to 4x4 with identity for extra dims)
        let mut aug = [
            [2.0, 1.0, 0.0, 0.0, 5.0],
            [1.0, 3.0, 0.0, 0.0, 10.0],
            [0.0, 0.0, 1.0, 0.0, 7.0],
            [0.0, 0.0, 0.0, 1.0, 9.0],
        ];
        let mut x = [0.0; 4];
        assert!(gauss_solve_4x4(&mut aug, &mut x));
        assert!((x[0] - 1.0).abs() < TOL);
        assert!((x[1] - 3.0).abs() < TOL);
        assert!((x[2] - 7.0).abs() < TOL);
        assert!((x[3] - 9.0).abs() < TOL);
    }
}
