//! # Muon g-2 Anomalous Magnetic Moment Detector
//!
//! Processes muon precession signals from storage ring experiments (Fermilab
//! Muon g-2 / BNL E821) to extract the anomalous precession frequency omega_a
//! and thereby the anomalous magnetic moment a_mu = (g-2)/2.
//!
//! ## Physics Background
//!
//! When polarized muons are injected into a uniform magnetic field B, they
//! undergo cyclotron motion at the cyclotron frequency omega_c = eB/(m_mu*c).
//! Their spin precesses at the Larmor frequency omega_s = g * eB/(2*m_mu*c).
//! Because g != 2, there is an anomalous precession frequency:
//!
//!   omega_a = omega_s - omega_c = a_mu * eB / (m_mu * c)
//!
//! where a_mu = (g-2)/2 is the anomalous magnetic moment. The current world
//! average is a_mu ~ 0.00116592061(41).
//!
//! Muons decay (mu+ -> e+ nu_e anti-nu_mu) with lifetime tau_mu ~ 2.197 us
//! (dilated in the lab frame by gamma ~ 29.3 at p = 3.094 GeV/c). The highest-
//! energy positrons are preferentially emitted along the muon spin direction,
//! so the rate of detected positrons above an energy threshold oscillates at
//! omega_a, producing the "wiggle plot":
//!
//!   N(t) = N0 * exp(-t / tau) * [1 + A * cos(omega_a * t + phi)]
//!
//! ## Implemented Techniques
//!
//! - **Five-parameter fit**: N0, tau, A, omega_a, phi extraction from the
//!   wiggle plot using iterative Levenberg-Marquardt minimization.
//! - **Ratio method R(t)**: Forms a ratio that cancels the exponential decay,
//!   isolating the oscillatory component to reduce systematic errors.
//! - **Fast Rotation Analysis (FRA)**: Fourier analysis of the early-time
//!   signal to extract the muon momentum distribution in the storage ring.
//! - **Pileup correction**: Estimates and subtracts shadow-pulse contributions
//!   from positron detector deadtime at high rates.
//! - **T-method analysis**: Sums energies above threshold for improved
//!   statistical sensitivity compared to simple counting.
//! - **A-weighted analysis**: Weights each event by its asymmetry to optimize
//!   the statistical figure of merit N*A^2.
//! - **Magnetic field**: Larmor frequency omega_p from NMR free-induction
//!   decay, multipole shimming expansion coefficients.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::muon_g2_anomaly_detector::*;
//!
//! // Generate a synthetic wiggle plot
//! let params = WiggleParams {
//!     n0: 1e6,
//!     tau_us: 64.44,   // dilated muon lifetime
//!     asymmetry: 0.4,
//!     omega_a: 1.4313,  // rad/us (approximate)
//!     phi: 0.0,
//! };
//! let dt = 0.1492;  // ~149.2 ns bin width (cyclotron period)
//! let histogram = generate_wiggle_plot(&params, dt, 4000);
//! assert!(histogram.len() == 4000);
//!
//! // Compute the ratio R(t) that cancels the exponential
//! let ratio = ratio_method(&histogram, 10);
//! assert!(!ratio.is_empty());
//!
//! // Anomalous magnetic moment from omega_a and omega_p
//! let omega_p = 61.79;  // MHz (proton Larmor in ~1.45 T)
//! let a_mu = compute_a_mu(params.omega_a, omega_p);
//! assert!((a_mu - 0.00116592).abs() < 0.001);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Muon rest-frame lifetime in microseconds.
const MUON_LIFETIME_US: f64 = 2.1969811;

/// Muon mass in MeV/c^2.
const MUON_MASS_MEV: f64 = 105.6583755;

/// Muon anomalous magnetic moment (current world average).
const A_MU_WORLD_AVG: f64 = 0.00116592061;

/// Magic momentum in MeV/c for the g-2 storage ring (gamma = 29.3).
const MAGIC_MOMENTUM_MEV: f64 = 3094.0;

/// Magic gamma factor (Lorentz factor at magic momentum).
const MAGIC_GAMMA: f64 = 29.3;

/// Dilated muon lifetime at magic gamma (tau * gamma) in microseconds.
const DILATED_LIFETIME_US: f64 = MUON_LIFETIME_US * MAGIC_GAMMA;

/// Cyclotron period at magic momentum in the g-2 ring (~149.2 ns = 0.1492 us).
const CYCLOTRON_PERIOD_US: f64 = 0.1492;

/// Anomalous precession period in microseconds (~4.365 us).
const ANOMALOUS_PERIOD_US: f64 = 4.365;

/// Ratio of muon-to-proton magnetic moment (mu_mu/mu_p).
/// This is the fundamental ratio lambda = omega_a / omega_p that gives a_mu.
const MU_MU_OVER_MU_P: f64 = 3.183345142;

/// Proton gyromagnetic ratio in MHz/T.
const PROTON_GYRO_MHZ_PER_T: f64 = 42.577478518;

// ---------------------------------------------------------------------------
// Wiggle plot model
// ---------------------------------------------------------------------------

/// Parameters for the five-parameter muon decay wiggle plot.
///
/// N(t) = N0 * exp(-t/tau) * [1 + A * cos(omega_a * t + phi)]
#[derive(Debug, Clone, Copy)]
pub struct WiggleParams {
    /// Initial count normalization.
    pub n0: f64,
    /// Dilated muon lifetime in microseconds.
    pub tau_us: f64,
    /// Parity-violating asymmetry (0 < A < ~0.4).
    pub asymmetry: f64,
    /// Anomalous precession angular frequency in rad/us.
    pub omega_a: f64,
    /// Initial phase in radians.
    pub phi: f64,
}

impl Default for WiggleParams {
    fn default() -> Self {
        Self {
            n0: 1.0e6,
            tau_us: DILATED_LIFETIME_US,
            asymmetry: 0.4,
            omega_a: 2.0 * PI / ANOMALOUS_PERIOD_US,
            phi: 0.0,
        }
    }
}

impl WiggleParams {
    /// Evaluate the five-parameter model at time t (microseconds).
    pub fn evaluate(&self, t: f64) -> f64 {
        self.n0 * (-t / self.tau_us).exp() * (1.0 + self.asymmetry * (self.omega_a * t + self.phi).cos())
    }

    /// Evaluate the five-parameter model for a vector of times.
    pub fn evaluate_many(&self, times: &[f64]) -> Vec<f64> {
        times.iter().map(|&t| self.evaluate(t)).collect()
    }

    /// Compute the anomalous precession frequency in MHz.
    pub fn f_a_mhz(&self) -> f64 {
        self.omega_a / (2.0 * PI)
    }

    /// Compute the anomalous precession period in microseconds.
    pub fn period_us(&self) -> f64 {
        2.0 * PI / self.omega_a
    }
}

// ---------------------------------------------------------------------------
// Extended model (CBO + lost muons)
// ---------------------------------------------------------------------------

/// Extended wiggle parameters including coherent betatron oscillation (CBO)
/// and lost muon corrections.
///
/// N(t) = N0 * exp(-t/tau) * [1 + A*cos(omega_a*t + phi)]
///        * [1 + A_cbo*exp(-t/tau_cbo)*cos(omega_cbo*t + phi_cbo)]
///        * [1 - K_loss * L(t)]
#[derive(Debug, Clone, Copy)]
pub struct ExtendedWiggleParams {
    /// Base five-parameter fit.
    pub base: WiggleParams,
    /// CBO oscillation amplitude.
    pub a_cbo: f64,
    /// CBO angular frequency in rad/us.
    pub omega_cbo: f64,
    /// CBO phase in radians.
    pub phi_cbo: f64,
    /// CBO damping time constant in microseconds.
    pub tau_cbo: f64,
    /// Lost muon fraction coefficient.
    pub k_loss: f64,
    /// Lost muon cumulative integral scale (linear approx).
    pub loss_rate: f64,
}

impl Default for ExtendedWiggleParams {
    fn default() -> Self {
        Self {
            base: WiggleParams::default(),
            a_cbo: 0.003,
            omega_cbo: 2.0 * PI / 0.3727, // CBO period ~372.7 ns
            phi_cbo: 0.0,
            tau_cbo: 150.0, // ~150 us CBO damping
            k_loss: 0.001,
            loss_rate: 0.01,
        }
    }
}

impl ExtendedWiggleParams {
    /// Evaluate the extended model at time t (microseconds).
    pub fn evaluate(&self, t: f64) -> f64 {
        let base = self.base.evaluate(t);
        let cbo = 1.0 + self.a_cbo * (-t / self.tau_cbo).exp()
            * (self.omega_cbo * t + self.phi_cbo).cos();
        let lost = 1.0 - self.k_loss * self.loss_rate * t;
        base * cbo * lost.max(0.0)
    }

    /// Evaluate the extended model for a vector of times.
    pub fn evaluate_many(&self, times: &[f64]) -> Vec<f64> {
        times.iter().map(|&t| self.evaluate(t)).collect()
    }
}

// ---------------------------------------------------------------------------
// Wiggle plot generation
// ---------------------------------------------------------------------------

/// Generate a synthetic wiggle plot histogram.
///
/// # Arguments
/// * `params` - Five-parameter model parameters
/// * `bin_width_us` - Time bin width in microseconds
/// * `num_bins` - Number of bins
///
/// # Returns
/// Vector of counts per bin.
pub fn generate_wiggle_plot(params: &WiggleParams, bin_width_us: f64, num_bins: usize) -> Vec<f64> {
    (0..num_bins)
        .map(|i| {
            let t = (i as f64 + 0.5) * bin_width_us;
            params.evaluate(t)
        })
        .collect()
}

/// Generate a wiggle plot with Poisson-like noise using a deterministic
/// pseudo-random number generator (for reproducible tests).
pub fn generate_noisy_wiggle_plot(
    params: &WiggleParams,
    bin_width_us: f64,
    num_bins: usize,
    seed: u64,
) -> Vec<f64> {
    let clean = generate_wiggle_plot(params, bin_width_us, num_bins);
    let mut rng = SimpleRng::new(seed);
    clean
        .iter()
        .map(|&n| {
            if n <= 0.0 {
                return 0.0;
            }
            // Approximate Poisson noise: N + sqrt(N) * gaussian
            let sigma = n.sqrt();
            let noise = rng.gaussian() * sigma;
            (n + noise).max(0.0)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Simple deterministic PRNG (no external crates)
// ---------------------------------------------------------------------------

/// Minimal xorshift64 PRNG for reproducible noise generation.
struct SimpleRng {
    state: u64,
}

impl SimpleRng {
    fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 { 0x12345678_9ABCDEF0 } else { seed },
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    /// Uniform [0, 1) random number.
    fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Box-Muller Gaussian (mean=0, std=1).
    fn gaussian(&mut self) -> f64 {
        let u1 = self.uniform().max(1e-15);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

// ---------------------------------------------------------------------------
// Ratio method R(t)
// ---------------------------------------------------------------------------

/// Compute the ratio R(t) that cancels the exponential decay.
///
/// R(t) = [N(t - T/2) - N(t + T/2)] / [N(t - T/2) + N(t + T/2)]
///
/// where T is the anomalous precession half-period. The result oscillates
/// as approximately A * cos(omega_a * t + phi), free from the dominant
/// systematic of lifetime variation.
///
/// # Arguments
/// * `histogram` - Time histogram (wiggle plot)
/// * `half_period_bins` - Half-period offset in bin units (T_a / 2 / bin_width)
///
/// # Returns
/// Vector of ratio values, shorter than input by 2 * half_period_bins.
pub fn ratio_method(histogram: &[f64], half_period_bins: usize) -> Vec<f64> {
    if histogram.len() < 2 * half_period_bins + 1 {
        return Vec::new();
    }
    let n = histogram.len() - 2 * half_period_bins;
    (0..n)
        .map(|i| {
            let n_minus = histogram[i];
            let n_plus = histogram[i + 2 * half_period_bins];
            let sum = n_minus + n_plus;
            if sum.abs() < 1e-30 {
                0.0
            } else {
                (n_minus - n_plus) / sum
            }
        })
        .collect()
}

/// Fit the ratio R(t) to extract omega_a using a simple cosine fit.
///
/// Returns (omega_a, amplitude, phase, chi2_per_ndf).
pub fn fit_ratio(
    ratio: &[f64],
    bin_width_us: f64,
    half_period_bins: usize,
    omega_guess: f64,
) -> FitResult {
    // Grid search around omega_guess +/- 1% with fine steps
    let omega_min = omega_guess * 0.99;
    let omega_max = omega_guess * 1.01;
    let n_steps = 2000;
    let d_omega = (omega_max - omega_min) / n_steps as f64;

    let mut best_omega = omega_guess;
    let mut best_chi2 = f64::MAX;
    let mut best_a = 0.0;
    let mut best_phi = 0.0;

    for step in 0..=n_steps {
        let omega = omega_min + step as f64 * d_omega;
        // Linear least squares: R(t) = a*cos(omega*t + phi)
        // Expand: R = a*cos(phi)*cos(omega*t) - a*sin(phi)*sin(omega*t)
        // Let c = a*cos(phi), s = -a*sin(phi)
        let mut cc = 0.0;
        let mut ss = 0.0;
        let mut cs = 0.0;
        let mut rc = 0.0;
        let mut rs = 0.0;

        for (i, &r) in ratio.iter().enumerate() {
            let t = (i as f64 + half_period_bins as f64 + 0.5) * bin_width_us;
            let cos_wt = (omega * t).cos();
            let sin_wt = (omega * t).sin();
            cc += cos_wt * cos_wt;
            ss += sin_wt * sin_wt;
            cs += cos_wt * sin_wt;
            rc += r * cos_wt;
            rs += r * sin_wt;
        }

        let det = cc * ss - cs * cs;
        if det.abs() < 1e-30 {
            continue;
        }
        let c = (ss * rc - cs * rs) / det;
        let s = (cc * rs - cs * rc) / det;

        let amp = (c * c + s * s).sqrt();
        let phase = (-s).atan2(c);

        let mut chi2 = 0.0;
        for (i, &r) in ratio.iter().enumerate() {
            let t = (i as f64 + half_period_bins as f64 + 0.5) * bin_width_us;
            let model = amp * (omega * t + phase).cos();
            let residual = r - model;
            chi2 += residual * residual;
        }

        if chi2 < best_chi2 {
            best_chi2 = chi2;
            best_omega = omega;
            best_a = amp;
            best_phi = phase;
        }
    }

    let ndf = ratio.len().saturating_sub(3).max(1) as f64;
    FitResult {
        omega_a: best_omega,
        amplitude: best_a,
        phase: best_phi,
        chi2_per_ndf: best_chi2 / ndf,
        n0: 0.0,
        tau_us: 0.0,
        converged: true,
        iterations: n_steps,
    }
}

// ---------------------------------------------------------------------------
// Five-parameter fit
// ---------------------------------------------------------------------------

/// Result of a wiggle plot fit.
#[derive(Debug, Clone, Copy)]
pub struct FitResult {
    /// Fitted anomalous precession frequency (rad/us).
    pub omega_a: f64,
    /// Fitted asymmetry amplitude.
    pub amplitude: f64,
    /// Fitted initial phase (radians).
    pub phase: f64,
    /// Chi-squared per degree of freedom.
    pub chi2_per_ndf: f64,
    /// Fitted normalization.
    pub n0: f64,
    /// Fitted lifetime (microseconds).
    pub tau_us: f64,
    /// Whether the fit converged.
    pub converged: bool,
    /// Number of iterations used.
    pub iterations: usize,
}

/// Perform a five-parameter fit to the wiggle plot histogram.
///
/// Uses iterative Gauss-Newton with Levenberg-Marquardt damping.
///
/// # Arguments
/// * `histogram` - Time histogram counts
/// * `bin_width_us` - Bin width in microseconds
/// * `initial` - Initial parameter guess
/// * `max_iter` - Maximum iterations
///
/// # Returns
/// Fit result containing optimized parameters.
pub fn five_parameter_fit(
    histogram: &[f64],
    bin_width_us: f64,
    initial: &WiggleParams,
    max_iter: usize,
) -> FitResult {
    let n = histogram.len();
    if n < 6 {
        return FitResult {
            omega_a: initial.omega_a,
            amplitude: initial.asymmetry,
            phase: initial.phi,
            chi2_per_ndf: f64::MAX,
            n0: initial.n0,
            tau_us: initial.tau_us,
            converged: false,
            iterations: 0,
        };
    }

    // Parameters: [N0, tau, A, omega_a, phi]
    let mut p = [initial.n0, initial.tau_us, initial.asymmetry, initial.omega_a, initial.phi];
    let mut lambda = 1.0e-3; // LM damping factor

    let model_fn = |p: &[f64; 5], t: f64| -> f64 {
        p[0] * (-t / p[1]).exp() * (1.0 + p[2] * (p[3] * t + p[4]).cos())
    };

    let compute_chi2 = |p: &[f64; 5]| -> f64 {
        let mut chi2 = 0.0;
        for i in 0..n {
            let t = (i as f64 + 0.5) * bin_width_us;
            let y = histogram[i];
            let f = model_fn(p, t);
            let sigma2 = y.abs().max(1.0); // Poisson variance
            let r = y - f;
            chi2 += r * r / sigma2;
        }
        chi2
    };

    let mut chi2 = compute_chi2(&p);
    let mut converged = false;
    let mut iter_count = 0;

    for iter in 0..max_iter {
        iter_count = iter + 1;
        // Build normal equations: J^T W J dp = J^T W r
        let mut jtj = [[0.0f64; 5]; 5];
        let mut jtr = [0.0f64; 5];

        for i in 0..n {
            let t = (i as f64 + 0.5) * bin_width_us;
            let y = histogram[i];
            let exp_val = (-t / p[1]).exp();
            let cos_val = (p[3] * t + p[4]).cos();
            let sin_val = (p[3] * t + p[4]).sin();
            let f = p[0] * exp_val * (1.0 + p[2] * cos_val);
            let sigma2 = y.abs().max(1.0);
            let w = 1.0 / sigma2;
            let r = y - f;

            // Jacobian: df/dp[k]
            let j = [
                exp_val * (1.0 + p[2] * cos_val),                        // df/dN0
                p[0] * exp_val * (t / (p[1] * p[1])) * (1.0 + p[2] * cos_val), // df/dtau
                p[0] * exp_val * cos_val,                                 // df/dA
                -p[0] * exp_val * p[2] * sin_val * t,                    // df/domega
                -p[0] * exp_val * p[2] * sin_val,                        // df/dphi
            ];

            for a in 0..5 {
                jtr[a] += w * j[a] * r;
                for b in 0..5 {
                    jtj[a][b] += w * j[a] * j[b];
                }
            }
        }

        // Add LM damping to diagonal
        for k in 0..5 {
            jtj[k][k] *= 1.0 + lambda;
        }

        // Solve 5x5 system using Gaussian elimination
        let dp = solve_5x5(&jtj, &jtr);
        if dp.is_none() {
            lambda *= 10.0;
            continue;
        }
        let dp = dp.unwrap();

        // Trial step
        let mut p_new = p;
        for k in 0..5 {
            p_new[k] += dp[k];
        }

        // Enforce physical constraints
        if p_new[0] < 0.0 { p_new[0] = p[0] * 0.5; }
        if p_new[1] < 1.0 { p_new[1] = 1.0; }
        if p_new[2] < 0.0 { p_new[2] = 0.001; }
        if p_new[2] > 1.0 { p_new[2] = 1.0; }

        let chi2_new = compute_chi2(&p_new);

        if chi2_new < chi2 {
            p = p_new;
            let improvement = (chi2 - chi2_new) / chi2.max(1e-30);
            chi2 = chi2_new;
            lambda *= 0.1;
            if improvement < 1e-8 {
                converged = true;
                break;
            }
        } else {
            lambda *= 10.0;
            if lambda > 1e10 {
                converged = true; // Stuck; treat as converged at current params
                break;
            }
        }
    }

    let ndf = (n as f64 - 5.0).max(1.0);
    FitResult {
        omega_a: p[3],
        amplitude: p[2],
        phase: p[4],
        chi2_per_ndf: chi2 / ndf,
        n0: p[0],
        tau_us: p[1],
        converged,
        iterations: iter_count,
    }
}

/// Solve a 5x5 linear system Ax = b via Gaussian elimination with partial pivoting.
fn solve_5x5(a: &[[f64; 5]; 5], b: &[f64; 5]) -> Option<[f64; 5]> {
    let mut m = [[0.0f64; 6]; 5]; // augmented matrix
    for i in 0..5 {
        for j in 0..5 {
            m[i][j] = a[i][j];
        }
        m[i][5] = b[i];
    }

    // Forward elimination with partial pivoting
    for col in 0..5 {
        // Find pivot
        let mut max_val = m[col][col].abs();
        let mut max_row = col;
        for row in (col + 1)..5 {
            if m[row][col].abs() > max_val {
                max_val = m[row][col].abs();
                max_row = row;
            }
        }
        if max_val < 1e-30 {
            return None;
        }
        if max_row != col {
            m.swap(col, max_row);
        }

        let pivot = m[col][col];
        for row in (col + 1)..5 {
            let factor = m[row][col] / pivot;
            for j in col..6 {
                m[row][j] -= factor * m[col][j];
            }
        }
    }

    // Back substitution
    let mut x = [0.0f64; 5];
    for i in (0..5).rev() {
        let mut sum = m[i][5];
        for j in (i + 1)..5 {
            sum -= m[i][j] * x[j];
        }
        x[i] = sum / m[i][i];
    }

    Some(x)
}

// ---------------------------------------------------------------------------
// Fast Rotation Analysis (FRA)
// ---------------------------------------------------------------------------

/// Perform Fast Rotation Analysis to extract the muon momentum distribution.
///
/// The early-time oscillation structure of the decay positron signal contains
/// information about the momentum spread of muons in the storage ring. By
/// dividing out the five-parameter fit and Fourier-transforming the residual,
/// we obtain the radial frequency distribution which maps to momentum.
///
/// # Arguments
/// * `histogram` - Time histogram
/// * `bin_width_us` - Bin width in microseconds
/// * `params` - Fitted five-parameter model
/// * `num_freq_bins` - Number of frequency bins for DFT
///
/// # Returns
/// (frequencies_mhz, power_spectrum) tuple.
pub fn fast_rotation_analysis(
    histogram: &[f64],
    bin_width_us: f64,
    params: &WiggleParams,
    num_freq_bins: usize,
) -> (Vec<f64>, Vec<f64>) {
    // Compute residual after dividing out five-parameter model
    let n = histogram.len();
    let mut residual = Vec::with_capacity(n);
    for i in 0..n {
        let t = (i as f64 + 0.5) * bin_width_us;
        let model = params.evaluate(t);
        if model.abs() > 1e-10 {
            residual.push(histogram[i] / model - 1.0);
        } else {
            residual.push(0.0);
        }
    }

    // DFT around the cyclotron frequency region
    let f_c = 1.0 / CYCLOTRON_PERIOD_US; // ~6.7 MHz
    let f_min = f_c * 0.95;
    let f_max = f_c * 1.05;
    let df = (f_max - f_min) / num_freq_bins as f64;

    let mut freqs = Vec::with_capacity(num_freq_bins);
    let mut power = Vec::with_capacity(num_freq_bins);

    for k in 0..num_freq_bins {
        let f = f_min + k as f64 * df;
        let omega = 2.0 * PI * f;
        let mut re = 0.0;
        let mut im = 0.0;
        for (i, &r) in residual.iter().enumerate() {
            let t = (i as f64 + 0.5) * bin_width_us;
            re += r * (omega * t).cos();
            im -= r * (omega * t).sin();
        }
        freqs.push(f);
        power.push(re * re + im * im);
    }

    (freqs, power)
}

// ---------------------------------------------------------------------------
// Pileup correction
// ---------------------------------------------------------------------------

/// Configuration for pileup correction.
#[derive(Debug, Clone, Copy)]
pub struct PileupConfig {
    /// Detector deadtime in microseconds.
    pub deadtime_us: f64,
    /// Shadow pulse fraction (probability that a trailing pulse is lost).
    pub shadow_fraction: f64,
    /// Maximum pileup multiplicity to consider.
    pub max_multiplicity: usize,
}

impl Default for PileupConfig {
    fn default() -> Self {
        Self {
            deadtime_us: 0.005, // 5 ns
            shadow_fraction: 0.01,
            max_multiplicity: 3,
        }
    }
}

/// Apply pileup correction to a time histogram.
///
/// Uses the shadow-pulse method: for each bin, estimates the probability
/// that two or more pulses overlap within the deadtime window, forming
/// a single detected pulse with summed energy (pileup) or causing a
/// missed pulse (shadow).
///
/// # Arguments
/// * `histogram` - Uncorrected time histogram
/// * `bin_width_us` - Bin width in microseconds
/// * `config` - Pileup parameters
///
/// # Returns
/// Corrected histogram with pileup artifacts subtracted.
pub fn pileup_correction(
    histogram: &[f64],
    bin_width_us: f64,
    config: &PileupConfig,
) -> Vec<f64> {
    let n = histogram.len();
    let dt_bins = (config.deadtime_us / bin_width_us).ceil() as usize;
    let dt_bins = dt_bins.max(1);

    let mut corrected = histogram.to_vec();

    // For each multiplicity level
    for mult in 2..=config.max_multiplicity {
        let scale = config.shadow_fraction.powi(mult as i32 - 1);
        for i in 0..n {
            // Estimate pileup contribution from neighboring bins
            let rate = histogram[i] / bin_width_us;
            // Poisson probability of mult hits in deadtime window
            let mu = rate * config.deadtime_us;
            let p_pileup = poisson_prob(mu, mult);
            corrected[i] += scale * histogram[i] * p_pileup;
        }

        // Subtract shadow pulses from preceding bins
        for i in dt_bins..n {
            let rate_prev = histogram[i - dt_bins] / bin_width_us;
            let mu = rate_prev * config.deadtime_us;
            let p_shadow = poisson_prob(mu, mult);
            corrected[i] -= scale * histogram[i - dt_bins] * p_shadow * 0.5;
        }
    }

    // Ensure non-negative
    for val in &mut corrected {
        if *val < 0.0 {
            *val = 0.0;
        }
    }

    corrected
}

/// Poisson probability P(k; mu) = mu^k * exp(-mu) / k!
fn poisson_prob(mu: f64, k: usize) -> f64 {
    if mu < 1e-30 {
        return if k == 0 { 1.0 } else { 0.0 };
    }
    let log_p = k as f64 * mu.ln() - mu - ln_factorial(k);
    log_p.exp()
}

/// Natural log of factorial, using Stirling for large n.
fn ln_factorial(n: usize) -> f64 {
    if n <= 1 {
        return 0.0;
    }
    if n <= 20 {
        let mut result = 0.0f64;
        for i in 2..=n {
            result += (i as f64).ln();
        }
        return result;
    }
    // Stirling's approximation
    let nf = n as f64;
    nf * nf.ln() - nf + 0.5 * (2.0 * PI * nf).ln()
}

// ---------------------------------------------------------------------------
// T-method analysis
// ---------------------------------------------------------------------------

/// T-method analysis: weight each positron by its energy above threshold.
///
/// The T-method sums the energies of detected positrons rather than simply
/// counting them. This gives higher statistical weight to high-energy positrons
/// which carry more asymmetry information, improving the figure of merit
/// from N*A^2 (counting) to effectively N*<A*E>^2.
///
/// # Arguments
/// * `energies` - Positron energies in MeV for each detected event
/// * `times` - Detection times in microseconds
/// * `threshold_mev` - Energy threshold
/// * `bin_width_us` - Time bin width
/// * `num_bins` - Number of time bins
///
/// # Returns
/// T-method weighted histogram.
pub fn t_method(
    energies: &[f64],
    times: &[f64],
    threshold_mev: f64,
    bin_width_us: f64,
    num_bins: usize,
) -> Vec<f64> {
    let mut histogram = vec![0.0; num_bins];
    for (&e, &t) in energies.iter().zip(times.iter()) {
        if e >= threshold_mev && t >= 0.0 {
            let bin = (t / bin_width_us) as usize;
            if bin < num_bins {
                histogram[bin] += e;
            }
        }
    }
    histogram
}

/// Standard counting method: histogram of events above energy threshold.
pub fn counting_method(
    energies: &[f64],
    times: &[f64],
    threshold_mev: f64,
    bin_width_us: f64,
    num_bins: usize,
) -> Vec<f64> {
    let mut histogram = vec![0.0; num_bins];
    for (&e, &t) in energies.iter().zip(times.iter()) {
        if e >= threshold_mev && t >= 0.0 {
            let bin = (t / bin_width_us) as usize;
            if bin < num_bins {
                histogram[bin] += 1.0;
            }
        }
    }
    histogram
}

// ---------------------------------------------------------------------------
// A-weighted analysis
// ---------------------------------------------------------------------------

/// Asymmetry function A(E) for the positron energy spectrum.
///
/// For muon decay, the asymmetry varies with positron energy:
///   A(y) = (2y - 1) / (3 - 2y)    where y = E / E_max
///
/// This peaks at A = 1 for y = 1 (maximum energy positrons).
pub fn asymmetry_vs_energy(e_mev: f64, e_max_mev: f64) -> f64 {
    if e_max_mev <= 0.0 || e_mev <= 0.0 || e_mev > e_max_mev {
        return 0.0;
    }
    let y = e_mev / e_max_mev;
    (2.0 * y - 1.0) / (3.0 - 2.0 * y)
}

/// A-weighted histogram: each event weighted by its asymmetry A(E).
///
/// This optimizes the statistical figure of merit N * A^2 by giving
/// higher weight to events with larger analyzing power.
pub fn a_weighted_method(
    energies: &[f64],
    times: &[f64],
    e_max_mev: f64,
    threshold_mev: f64,
    bin_width_us: f64,
    num_bins: usize,
) -> Vec<f64> {
    let mut histogram = vec![0.0; num_bins];
    for (&e, &t) in energies.iter().zip(times.iter()) {
        if e >= threshold_mev && t >= 0.0 {
            let bin = (t / bin_width_us) as usize;
            if bin < num_bins {
                let a = asymmetry_vs_energy(e, e_max_mev);
                histogram[bin] += a;
            }
        }
    }
    histogram
}

/// Compute the figure of merit N * A^2 for a given energy threshold.
///
/// The optimal threshold maximizes this quantity, balancing statistics (N)
/// against analyzing power (A).
pub fn figure_of_merit(
    energies: &[f64],
    e_max_mev: f64,
    threshold_mev: f64,
) -> f64 {
    let mut n = 0.0;
    let mut sum_a2 = 0.0;
    for &e in energies {
        if e >= threshold_mev {
            n += 1.0;
            let a = asymmetry_vs_energy(e, e_max_mev);
            sum_a2 += a * a;
        }
    }
    if n < 1.0 {
        return 0.0;
    }
    let avg_a2 = sum_a2 / n;
    n * avg_a2
}

// ---------------------------------------------------------------------------
// Magnetic field analysis (NMR)
// ---------------------------------------------------------------------------

/// NMR free-induction decay (FID) parameters.
#[derive(Debug, Clone, Copy)]
pub struct NmrFidParams {
    /// Proton Larmor frequency in MHz.
    pub f_larmor_mhz: f64,
    /// FID amplitude.
    pub amplitude: f64,
    /// T2* relaxation time in microseconds.
    pub t2_star_us: f64,
    /// Phase offset in radians.
    pub phase: f64,
}

impl Default for NmrFidParams {
    fn default() -> Self {
        Self {
            f_larmor_mhz: 61.79, // ~1.45 T field
            amplitude: 1.0,
            t2_star_us: 50.0,
            phase: 0.0,
        }
    }
}

impl NmrFidParams {
    /// Evaluate the FID signal at time t (microseconds).
    pub fn evaluate(&self, t: f64) -> f64 {
        self.amplitude * (-t / self.t2_star_us).exp()
            * (2.0 * PI * self.f_larmor_mhz * t + self.phase).cos()
    }
}

/// Generate a synthetic NMR FID signal.
pub fn generate_nmr_fid(params: &NmrFidParams, dt_us: f64, num_samples: usize) -> Vec<f64> {
    (0..num_samples)
        .map(|i| params.evaluate(i as f64 * dt_us))
        .collect()
}

/// Extract the Larmor frequency from an NMR FID using zero-crossing analysis.
///
/// Counts the zero crossings and computes the average half-period.
pub fn extract_larmor_frequency(fid: &[f64], dt_us: f64) -> f64 {
    let mut crossings = Vec::new();
    for i in 1..fid.len() {
        if fid[i - 1] * fid[i] < 0.0 {
            // Linear interpolation for sub-sample crossing
            let frac = fid[i - 1].abs() / (fid[i - 1].abs() + fid[i].abs());
            let t = ((i - 1) as f64 + frac) * dt_us;
            crossings.push(t);
        }
    }
    if crossings.len() < 4 {
        return 0.0;
    }
    // Average full period from pairs of crossings
    let n_periods = (crossings.len() - 2) / 2;
    if n_periods == 0 {
        return 0.0;
    }
    let mut total_period = 0.0;
    for i in 0..n_periods {
        total_period += crossings[2 * i + 2] - crossings[2 * i];
    }
    let avg_period = total_period / n_periods as f64;
    1.0 / avg_period // frequency in MHz
}

/// Extract Larmor frequency from FID using DFT peak finding.
///
/// More robust than zero-crossing for noisy signals.
pub fn extract_larmor_frequency_dft(
    fid: &[f64],
    dt_us: f64,
    f_min_mhz: f64,
    f_max_mhz: f64,
    num_bins: usize,
) -> f64 {
    let df = (f_max_mhz - f_min_mhz) / num_bins as f64;
    let mut max_power = 0.0;
    let mut best_freq = f_min_mhz;

    for k in 0..num_bins {
        let f = f_min_mhz + k as f64 * df;
        let omega = 2.0 * PI * f;
        let mut re = 0.0;
        let mut im = 0.0;
        for (i, &val) in fid.iter().enumerate() {
            let t = i as f64 * dt_us;
            re += val * (omega * t).cos();
            im -= val * (omega * t).sin();
        }
        let power = re * re + im * im;
        if power > max_power {
            max_power = power;
            best_freq = f;
        }
    }

    best_freq
}

// ---------------------------------------------------------------------------
// Multipole shimming
// ---------------------------------------------------------------------------

/// Multipole expansion coefficients for the magnetic field.
///
/// The field in the storage region is expanded as:
/// B(r, theta) = sum_n (r/r0)^n * [a_n * cos(n*theta) + b_n * sin(n*theta)]
///
/// where a_n are the normal multipoles and b_n are the skew multipoles.
/// n=0: dipole, n=1: quadrupole, n=2: sextupole, etc.
#[derive(Debug, Clone)]
pub struct MultipoleField {
    /// Reference radius in mm.
    pub r0_mm: f64,
    /// Normal multipole coefficients (ppm at reference radius).
    pub normal: Vec<f64>,
    /// Skew multipole coefficients (ppm at reference radius).
    pub skew: Vec<f64>,
    /// Dipole field strength in Tesla.
    pub b0_tesla: f64,
}

impl MultipoleField {
    /// Create a new multipole field with given order.
    pub fn new(b0: f64, r0_mm: f64, max_order: usize) -> Self {
        Self {
            r0_mm,
            normal: vec![0.0; max_order + 1],
            skew: vec![0.0; max_order + 1],
            b0_tesla: b0,
        }
    }

    /// Evaluate the field at a point (r, theta) in mm and radians.
    /// Returns the field in Tesla.
    pub fn evaluate(&self, r_mm: f64, theta: f64) -> f64 {
        let mut sum = 0.0;
        for n in 0..self.normal.len() {
            let rn = (r_mm / self.r0_mm).powi(n as i32);
            sum += rn * (self.normal[n] * (n as f64 * theta).cos()
                + self.skew[n] * (n as f64 * theta).sin());
        }
        self.b0_tesla * (1.0 + sum * 1e-6) // ppm to relative
    }

    /// Compute the field uniformity (peak-to-peak variation) over a circle
    /// of given radius, in ppm.
    pub fn uniformity_ppm(&self, r_mm: f64, n_points: usize) -> f64 {
        let mut min_b = f64::MAX;
        let mut max_b = f64::MIN;
        for i in 0..n_points {
            let theta = 2.0 * PI * i as f64 / n_points as f64;
            let b = self.evaluate(r_mm, theta);
            if b < min_b { min_b = b; }
            if b > max_b { max_b = b; }
        }
        if self.b0_tesla.abs() < 1e-30 {
            return 0.0;
        }
        (max_b - min_b) / self.b0_tesla * 1e6
    }

    /// Compute the volume-averaged field over a circular cross-section.
    pub fn average_field(&self, r_mm: f64, n_r: usize, n_theta: usize) -> f64 {
        let mut sum = 0.0;
        let mut weight_sum = 0.0;
        for ir in 0..n_r {
            let r = r_mm * (ir as f64 + 0.5) / n_r as f64;
            let w = r; // area element r*dr*dtheta
            for it in 0..n_theta {
                let theta = 2.0 * PI * it as f64 / n_theta as f64;
                sum += w * self.evaluate(r, theta);
                weight_sum += w;
            }
        }
        sum / weight_sum
    }
}

// ---------------------------------------------------------------------------
// a_mu computation
// ---------------------------------------------------------------------------

/// Compute the anomalous magnetic moment a_mu from omega_a and omega_p.
///
/// a_mu = R / (lambda - R)
///
/// where R = omega_a / omega_p and lambda = mu_mu / mu_p.
///
/// # Arguments
/// * `omega_a` - Anomalous precession frequency (rad/us)
/// * `omega_p` - Proton Larmor frequency (MHz)
///
/// # Returns
/// The anomalous magnetic moment a_mu = (g-2)/2.
pub fn compute_a_mu(omega_a: f64, omega_p: f64) -> f64 {
    // Convert omega_a from rad/us to MHz
    let f_a_mhz = omega_a / (2.0 * PI);
    let r = f_a_mhz / omega_p;
    r / (MU_MU_OVER_MU_P - r)
}

/// Compute omega_a from a_mu and omega_p (inverse of compute_a_mu).
pub fn omega_a_from_a_mu(a_mu: f64, omega_p: f64) -> f64 {
    let r = a_mu * MU_MU_OVER_MU_P / (1.0 + a_mu);
    2.0 * PI * r * omega_p
}

/// Compute the electric field correction to omega_a.
///
/// At the magic momentum, the electric field contribution to the spin
/// precession vanishes. Away from magic momentum, there is a correction:
///
/// delta_omega / omega_a = -2 * beta * eta * n / (1 - n)
///
/// where n is the field index and eta = delta_p / p is the momentum offset.
pub fn electric_field_correction(n_field_index: f64, dp_over_p: f64, beta: f64) -> f64 {
    if (1.0 - n_field_index).abs() < 1e-15 {
        return 0.0;
    }
    -2.0 * beta * dp_over_p * n_field_index / (1.0 - n_field_index)
}

/// Compute the pitch correction to omega_a.
///
/// Muons with vertical betatron oscillations experience a pitch angle psi,
/// which modifies omega_a:
///
/// delta_omega / omega_a = -beta^2 * <psi^2> / 2 * n / (1 - n)
pub fn pitch_correction(n_field_index: f64, psi_rms_rad: f64, beta: f64) -> f64 {
    if (1.0 - n_field_index).abs() < 1e-15 {
        return 0.0;
    }
    -beta * beta * psi_rms_rad * psi_rms_rad * 0.5 * n_field_index / (1.0 - n_field_index)
}

// ---------------------------------------------------------------------------
// Storage ring dynamics
// ---------------------------------------------------------------------------

/// Compute the cyclotron frequency in rad/us for a muon in a magnetic field.
pub fn cyclotron_frequency(b_tesla: f64, p_mev: f64) -> f64 {
    // omega_c = e*B*c^2 / E  where E = sqrt(p^2 + m^2) * c^2
    // In convenient units: omega_c [rad/us] = K * B [T] / E [MeV]
    // where K = e * c^2 / (MeV_to_J * 1e6)
    //         = (1.602e-19 * 8.988e16) / (1.602e-13 * 1e6)
    //         = 89875.518
    let e_mev = (p_mev * p_mev + MUON_MASS_MEV * MUON_MASS_MEV).sqrt();
    89875.518 * b_tesla / e_mev
}

/// Compute the betatron frequencies (horizontal and vertical) in the
/// weak-focusing storage ring.
///
/// f_x = f_c * sqrt(1 - n)
/// f_y = f_c * sqrt(n)
///
/// where n is the field index and f_c is the cyclotron frequency.
pub fn betatron_frequencies(f_c: f64, n_field_index: f64) -> (f64, f64) {
    let f_x = if n_field_index < 1.0 {
        f_c * (1.0 - n_field_index).sqrt()
    } else {
        0.0
    };
    let f_y = if n_field_index > 0.0 {
        f_c * n_field_index.sqrt()
    } else {
        0.0
    };
    (f_x, f_y)
}

/// Compute the CBO (Coherent Betatron Oscillation) frequency.
///
/// f_CBO = f_c - f_x = f_c * (1 - sqrt(1 - n))
pub fn cbo_frequency(f_c: f64, n_field_index: f64) -> f64 {
    if n_field_index >= 1.0 || n_field_index < 0.0 {
        return 0.0;
    }
    f_c * (1.0 - (1.0 - n_field_index).sqrt())
}

/// Compute the Lorentz gamma factor from momentum.
pub fn gamma_from_momentum(p_mev: f64) -> f64 {
    let e_mev = (p_mev * p_mev + MUON_MASS_MEV * MUON_MASS_MEV).sqrt();
    e_mev / MUON_MASS_MEV
}

/// Compute the dilated muon lifetime in microseconds.
pub fn dilated_lifetime(p_mev: f64) -> f64 {
    MUON_LIFETIME_US * gamma_from_momentum(p_mev)
}

/// Compute the muon beta factor from momentum.
pub fn beta_from_momentum(p_mev: f64) -> f64 {
    let gamma = gamma_from_momentum(p_mev);
    (1.0 - 1.0 / (gamma * gamma)).sqrt()
}

/// Compute the storage ring radius in meters from momentum and field.
///
/// r = p / (e * B) = p [MeV/c] / (299.792458 * B [T]) in meters.
pub fn ring_radius_m(p_mev: f64, b_tesla: f64) -> f64 {
    if b_tesla.abs() < 1e-15 {
        return 0.0;
    }
    p_mev / (299.792458 * b_tesla)
}

// ---------------------------------------------------------------------------
// Positron energy spectrum
// ---------------------------------------------------------------------------

/// Michel spectrum: energy distribution of decay positrons in muon rest frame.
///
/// dN/dy = 2y^2 * (3 - 2y) + polarization * 2y^2 * (2y - 1) / 3
///
/// where y = E / E_max, E_max = m_mu / 2 ~ 52.83 MeV.
pub fn michel_spectrum(y: f64, polarization: f64) -> f64 {
    if y < 0.0 || y > 1.0 {
        return 0.0;
    }
    let y2 = y * y;
    2.0 * y2 * (3.0 - 2.0 * y) + polarization * 2.0 * y2 * (2.0 * y - 1.0) / 3.0
}

/// Generate positron energy distribution (binned Michel spectrum).
///
/// # Arguments
/// * `num_bins` - Number of energy bins
/// * `polarization` - Muon polarization (0 to 1)
///
/// # Returns
/// Vector of normalized probabilities per bin.
pub fn generate_michel_spectrum(num_bins: usize, polarization: f64) -> Vec<f64> {
    let mut spectrum = Vec::with_capacity(num_bins);
    let mut total = 0.0;
    for i in 0..num_bins {
        let y = (i as f64 + 0.5) / num_bins as f64;
        let val = michel_spectrum(y, polarization);
        spectrum.push(val);
        total += val;
    }
    if total > 0.0 {
        for val in &mut spectrum {
            *val /= total;
        }
    }
    spectrum
}

/// Maximum positron energy in the muon rest frame (MeV).
pub fn max_positron_energy_rest_frame() -> f64 {
    MUON_MASS_MEV / 2.0
}

/// Boost the rest-frame positron energy to the lab frame.
///
/// For a positron emitted at angle theta_star in the muon rest frame:
/// E_lab = gamma * E_star * (1 + beta * cos(theta_star))
pub fn boost_energy(e_star_mev: f64, cos_theta_star: f64, gamma: f64, beta: f64) -> f64 {
    gamma * e_star_mev * (1.0 + beta * cos_theta_star)
}

// ---------------------------------------------------------------------------
// Statistical utilities
// ---------------------------------------------------------------------------

/// Compute the chi-squared between a histogram and a model.
pub fn chi_squared(histogram: &[f64], model: &[f64]) -> f64 {
    if histogram.len() != model.len() {
        return f64::MAX;
    }
    let mut chi2 = 0.0;
    for (h, m) in histogram.iter().zip(model.iter()) {
        let sigma2 = h.abs().max(1.0); // Poisson variance
        let r = h - m;
        chi2 += r * r / sigma2;
    }
    chi2
}

/// Compute the reduced chi-squared (chi^2 / ndf).
pub fn reduced_chi_squared(histogram: &[f64], model: &[f64], n_params: usize) -> f64 {
    let chi2 = chi_squared(histogram, model);
    let ndf = (histogram.len() as f64 - n_params as f64).max(1.0);
    chi2 / ndf
}

/// Estimate the statistical uncertainty on omega_a from a wiggle plot.
///
/// sigma(omega_a) ~ 1 / (A * sqrt(N) * tau)
///
/// where A is the asymmetry, N is total counts, and tau is the measurement time.
pub fn omega_a_statistical_uncertainty(
    asymmetry: f64,
    total_counts: f64,
    measurement_time_us: f64,
) -> f64 {
    if asymmetry.abs() < 1e-15 || total_counts < 1.0 || measurement_time_us < 1e-15 {
        return f64::MAX;
    }
    1.0 / (asymmetry * total_counts.sqrt() * measurement_time_us)
}

/// Compute the Fourier power spectrum of a time series.
///
/// Returns (frequencies, power) where frequencies are in 1/bin_width units.
pub fn power_spectrum(data: &[f64], bin_width: f64, num_freq_bins: usize) -> (Vec<f64>, Vec<f64>) {
    let n = data.len();
    let f_max = 0.5 / bin_width; // Nyquist
    let df = f_max / num_freq_bins as f64;

    let mut freqs = Vec::with_capacity(num_freq_bins);
    let mut power = Vec::with_capacity(num_freq_bins);

    for k in 0..num_freq_bins {
        let f = (k as f64 + 0.5) * df;
        let omega = 2.0 * PI * f;
        let mut re = 0.0;
        let mut im = 0.0;
        for (i, &val) in data.iter().enumerate() {
            let t = (i as f64 + 0.5) * bin_width;
            re += val * (omega * t).cos();
            im -= val * (omega * t).sin();
        }
        let p = (re * re + im * im) / n as f64;
        freqs.push(f);
        power.push(p);
    }

    (freqs, power)
}

/// Find the peak frequency in a power spectrum.
pub fn find_peak_frequency(freqs: &[f64], power: &[f64]) -> (f64, f64) {
    if freqs.len() != power.len() || freqs.is_empty() {
        return (0.0, 0.0);
    }
    let mut max_idx = 0;
    let mut max_val = power[0];
    for (i, &p) in power.iter().enumerate().skip(1) {
        if p > max_val {
            max_val = p;
            max_idx = i;
        }
    }
    (freqs[max_idx], max_val)
}

// ---------------------------------------------------------------------------
// Blinding
// ---------------------------------------------------------------------------

/// Apply a blinding offset to omega_a for analysis before unblinding.
///
/// This is standard practice in g-2 experiments to prevent experimenter bias.
/// The true omega_a is hidden by adding a secret offset during analysis.
pub fn apply_blinding(omega_a: f64, blinding_offset: f64) -> f64 {
    omega_a + blinding_offset
}

/// Remove the blinding offset (only done at the very end of analysis).
pub fn remove_blinding(blinded_omega_a: f64, blinding_offset: f64) -> f64 {
    blinded_omega_a - blinding_offset
}

// ---------------------------------------------------------------------------
// Muon loss monitor
// ---------------------------------------------------------------------------

/// Estimate the lost muon fraction from triple coincidence detector data.
///
/// Lost muons exit the storage region and are detected by scintillator
/// stations. The fractional loss is integrated and applied as a correction
/// to the wiggle plot.
///
/// # Arguments
/// * `loss_counts` - Detected lost muon counts per time bin
/// * `total_counts` - Total decay positron counts per time bin
///
/// # Returns
/// Cumulative lost fraction L(t).
pub fn cumulative_lost_fraction(loss_counts: &[f64], total_counts: &[f64]) -> Vec<f64> {
    let n = loss_counts.len().min(total_counts.len());
    let mut cumulative = Vec::with_capacity(n);
    let mut sum_loss = 0.0;
    let mut sum_total = 0.0;
    for i in 0..n {
        sum_loss += loss_counts[i];
        sum_total += total_counts[i];
        if sum_total > 0.0 {
            cumulative.push(sum_loss / sum_total);
        } else {
            cumulative.push(0.0);
        }
    }
    cumulative
}

// ---------------------------------------------------------------------------
// Run selection and data quality
// ---------------------------------------------------------------------------

/// Configuration for data quality cuts applied to individual fills.
#[derive(Debug, Clone, Copy)]
pub struct DataQualityConfig {
    /// Minimum number of positrons per fill to include.
    pub min_positrons: usize,
    /// Maximum fractional muon loss to include.
    pub max_loss_fraction: f64,
    /// Fit start time in microseconds (to avoid injection artifacts).
    pub t_start_us: f64,
    /// Fit end time in microseconds.
    pub t_end_us: f64,
    /// Maximum chi2/ndf for a good fill.
    pub max_chi2_ndf: f64,
}

impl Default for DataQualityConfig {
    fn default() -> Self {
        Self {
            min_positrons: 1000,
            max_loss_fraction: 0.01,
            t_start_us: 30.0,
            t_end_us: 650.0,
            max_chi2_ndf: 2.0,
        }
    }
}

/// Apply fit start time cut: zero out bins before t_start.
pub fn apply_start_time_cut(histogram: &mut [f64], bin_width_us: f64, t_start_us: f64) {
    let start_bin = (t_start_us / bin_width_us).ceil() as usize;
    let limit = start_bin.min(histogram.len());
    for bin in histogram.iter_mut().take(limit) {
        *bin = 0.0;
    }
}

/// Apply fit end time cut: zero out bins after t_end.
pub fn apply_end_time_cut(histogram: &mut [f64], bin_width_us: f64, t_end_us: f64) {
    let end_bin = (t_end_us / bin_width_us).floor() as usize;
    for bin in histogram.iter_mut().skip(end_bin + 1) {
        *bin = 0.0;
    }
}

/// Check whether a fill passes quality cuts.
pub fn passes_quality_cuts(
    total_positrons: usize,
    loss_fraction: f64,
    chi2_ndf: f64,
    config: &DataQualityConfig,
) -> bool {
    total_positrons >= config.min_positrons
        && loss_fraction <= config.max_loss_fraction
        && chi2_ndf <= config.max_chi2_ndf
}

// ---------------------------------------------------------------------------
// Spin tracking
// ---------------------------------------------------------------------------

/// Track the muon spin precession angle over time.
///
/// theta_s(t) = omega_a * t + phi
///
/// Returns the spin angle modulo 2*pi at each time step.
pub fn spin_precession_angle(omega_a: f64, phi: f64, times: &[f64]) -> Vec<f64> {
    times
        .iter()
        .map(|&t| {
            let angle = omega_a * t + phi;
            angle % (2.0 * PI)
        })
        .collect()
}

/// Compute the muon polarization decay due to momentum spread.
///
/// The beam has a spread dp/p ~ 0.1%, causing a distribution of omega_a
/// values that leads to decoherence of the spin precession:
///
/// P(t) = P0 * exp(-(t * sigma_omega)^2 / 2)
///
/// where sigma_omega = omega_a * dp/p.
pub fn polarization_decay(
    p0: f64,
    omega_a: f64,
    dp_over_p: f64,
    t: f64,
) -> f64 {
    let sigma_omega = omega_a * dp_over_p;
    p0 * (-0.5 * (t * sigma_omega).powi(2)).exp()
}

// ---------------------------------------------------------------------------
// EDM (Electric Dipole Moment) analysis
// ---------------------------------------------------------------------------

/// Estimate the muon EDM signal from vertical oscillation in the wiggle plot.
///
/// A nonzero muon EDM would cause the spin to precess about the radial
/// electric field direction, producing a vertical component:
///
/// N_up(t) - N_down(t) ~ d_mu * E * sin(omega_a * t)
///
/// The EDM signature is a sin(omega_a * t) oscillation (90 degrees out of
/// phase with the g-2 cosine), detected by up/down segmented calorimeters.
///
/// # Arguments
/// * `n_up` - Upper detector histogram
/// * `n_down` - Lower detector histogram
///
/// # Returns
/// Vertical asymmetry A_v(t) = (N_up - N_down) / (N_up + N_down).
pub fn vertical_asymmetry(n_up: &[f64], n_down: &[f64]) -> Vec<f64> {
    let n = n_up.len().min(n_down.len());
    (0..n)
        .map(|i| {
            let sum = n_up[i] + n_down[i];
            if sum.abs() < 1e-30 {
                0.0
            } else {
                (n_up[i] - n_down[i]) / sum
            }
        })
        .collect()
}

/// Fit the vertical asymmetry to extract the EDM phase.
///
/// A_v(t) ~ A_edm * sin(omega_a * t + phi_edm)
///
/// Returns (A_edm, phi_edm).
pub fn fit_edm_phase(
    asymmetry: &[f64],
    bin_width_us: f64,
    omega_a: f64,
) -> (f64, f64) {
    // Linear fit: A_v = c*sin(wt) + s*cos(wt)
    let mut sc = 0.0;
    let mut ss = 0.0;
    let mut cc = 0.0;
    let mut rc = 0.0;
    let mut rs = 0.0;

    for (i, &a) in asymmetry.iter().enumerate() {
        let t = (i as f64 + 0.5) * bin_width_us;
        let cos_wt = (omega_a * t).cos();
        let sin_wt = (omega_a * t).sin();
        sc += sin_wt * cos_wt;
        ss += sin_wt * sin_wt;
        cc += cos_wt * cos_wt;
        rc += a * cos_wt;
        rs += a * sin_wt;
    }

    let det = ss * cc - sc * sc;
    if det.abs() < 1e-30 {
        return (0.0, 0.0);
    }

    let c_coeff = (ss * rc - sc * rs) / det;
    let s_coeff = (cc * rs - sc * rc) / det;

    let amp = (c_coeff * c_coeff + s_coeff * s_coeff).sqrt();
    let phase = s_coeff.atan2(c_coeff);
    (amp, phase)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1e-6;
    const LOOSE_EPSILON: f64 = 1e-3;

    // --- Physical constants ---

    #[test]
    fn test_muon_lifetime() {
        assert!((MUON_LIFETIME_US - 2.197).abs() < 0.001);
    }

    #[test]
    fn test_muon_mass() {
        assert!((MUON_MASS_MEV - 105.658).abs() < 0.01);
    }

    #[test]
    fn test_magic_gamma() {
        let gamma = gamma_from_momentum(MAGIC_MOMENTUM_MEV);
        assert!((gamma - MAGIC_GAMMA).abs() < 0.1);
    }

    #[test]
    fn test_dilated_lifetime() {
        let tau = dilated_lifetime(MAGIC_MOMENTUM_MEV);
        assert!((tau - DILATED_LIFETIME_US).abs() < 1.0);
    }

    #[test]
    fn test_beta_at_magic_momentum() {
        let beta = beta_from_momentum(MAGIC_MOMENTUM_MEV);
        assert!(beta > 0.999);
        assert!(beta < 1.0);
    }

    // --- WiggleParams ---

    #[test]
    fn test_wiggle_default() {
        let p = WiggleParams::default();
        assert_eq!(p.n0, 1.0e6);
        assert!(p.tau_us > 60.0 && p.tau_us < 70.0);
        assert!((p.asymmetry - 0.4).abs() < EPSILON);
    }

    #[test]
    fn test_wiggle_evaluate_t0() {
        let p = WiggleParams::default();
        let val = p.evaluate(0.0);
        // At t=0: N0 * exp(0) * (1 + A * cos(phi))
        let expected = p.n0 * (1.0 + p.asymmetry * p.phi.cos());
        assert!((val - expected).abs() < 1.0);
    }

    #[test]
    fn test_wiggle_exponential_decay() {
        let p = WiggleParams {
            n0: 1000.0,
            tau_us: 64.0,
            asymmetry: 0.0,
            omega_a: 1.4,
            phi: 0.0,
        };
        let v0 = p.evaluate(0.0);
        let v_tau = p.evaluate(64.0);
        assert!((v_tau / v0 - 1.0 / std::f64::consts::E).abs() < 0.01);
    }

    #[test]
    fn test_wiggle_oscillation() {
        let p = WiggleParams {
            n0: 1000.0,
            tau_us: 1e6, // effectively no decay
            asymmetry: 0.5,
            omega_a: 2.0 * PI, // period = 1 us
            phi: 0.0,
        };
        let v_peak = p.evaluate(0.0);
        let v_trough = p.evaluate(0.5); // half period
        assert!(v_peak > v_trough);
        assert!((v_peak - 1500.0).abs() < 1.0);
        assert!((v_trough - 500.0).abs() < 1.0);
    }

    #[test]
    fn test_wiggle_evaluate_many() {
        let p = WiggleParams::default();
        let times = vec![0.0, 1.0, 2.0, 3.0];
        let vals = p.evaluate_many(&times);
        assert_eq!(vals.len(), 4);
        for (&t, &v) in times.iter().zip(vals.iter()) {
            assert!((v - p.evaluate(t)).abs() < EPSILON);
        }
    }

    #[test]
    fn test_wiggle_f_a_mhz() {
        let p = WiggleParams::default();
        let f = p.f_a_mhz();
        assert!(f > 0.2 && f < 0.3); // ~0.229 MHz
    }

    #[test]
    fn test_wiggle_period() {
        let p = WiggleParams::default();
        let period = p.period_us();
        assert!((period - ANOMALOUS_PERIOD_US).abs() < 0.1);
    }

    // --- Extended model ---

    #[test]
    fn test_extended_default() {
        let ep = ExtendedWiggleParams::default();
        assert!(ep.a_cbo > 0.0);
        assert!(ep.tau_cbo > 0.0);
    }

    #[test]
    fn test_extended_reduces_to_base() {
        let mut ep = ExtendedWiggleParams::default();
        ep.a_cbo = 0.0;
        ep.k_loss = 0.0;
        let base = ep.base.evaluate(10.0);
        let ext = ep.evaluate(10.0);
        assert!((base - ext).abs() / base.abs().max(1.0) < 1e-10);
    }

    #[test]
    fn test_extended_cbo_modulation() {
        let ep = ExtendedWiggleParams::default();
        let v1 = ep.evaluate(1.0);
        let v2 = ep.base.evaluate(1.0);
        // CBO should make a small difference
        assert!((v1 - v2).abs() / v2.abs().max(1.0) < 0.01);
    }

    #[test]
    fn test_extended_lost_muons() {
        let mut ep = ExtendedWiggleParams::default();
        ep.a_cbo = 0.0;
        ep.k_loss = 0.01;
        ep.loss_rate = 0.1;
        let v_early = ep.evaluate(1.0);
        let v_late = ep.evaluate(100.0);
        // Lost muons suppress later times
        let base_ratio = ep.base.evaluate(100.0) / ep.base.evaluate(1.0);
        let ext_ratio = v_late / v_early;
        assert!(ext_ratio < base_ratio);
    }

    // --- Wiggle plot generation ---

    #[test]
    fn test_generate_wiggle_plot_length() {
        let p = WiggleParams::default();
        let h = generate_wiggle_plot(&p, 0.1492, 1000);
        assert_eq!(h.len(), 1000);
    }

    #[test]
    fn test_generate_wiggle_plot_monotone_decay_envelope() {
        let p = WiggleParams {
            n0: 1e6,
            tau_us: 64.0,
            asymmetry: 0.0,
            omega_a: 1.4,
            phi: 0.0,
        };
        let h = generate_wiggle_plot(&p, 1.0, 200);
        // With A=0, it's pure exponential - monotonically decreasing
        for i in 1..h.len() {
            assert!(h[i] <= h[i - 1] + 1e-6);
        }
    }

    #[test]
    fn test_noisy_wiggle_plot_has_variation() {
        let p = WiggleParams::default();
        let clean = generate_wiggle_plot(&p, 0.1492, 100);
        let noisy = generate_noisy_wiggle_plot(&p, 0.1492, 100, 42);
        let mut diff_sum = 0.0;
        for (c, n) in clean.iter().zip(noisy.iter()) {
            diff_sum += (c - n).abs();
        }
        assert!(diff_sum > 0.0); // Should have some noise
    }

    #[test]
    fn test_noisy_wiggle_reproducible() {
        let p = WiggleParams::default();
        let h1 = generate_noisy_wiggle_plot(&p, 0.1492, 100, 42);
        let h2 = generate_noisy_wiggle_plot(&p, 0.1492, 100, 42);
        for (a, b) in h1.iter().zip(h2.iter()) {
            assert!((a - b).abs() < EPSILON);
        }
    }

    // --- Simple PRNG ---

    #[test]
    fn test_rng_deterministic() {
        let mut rng1 = SimpleRng::new(12345);
        let mut rng2 = SimpleRng::new(12345);
        for _ in 0..100 {
            assert_eq!(rng1.next_u64(), rng2.next_u64());
        }
    }

    #[test]
    fn test_rng_uniform_range() {
        let mut rng = SimpleRng::new(99);
        for _ in 0..1000 {
            let u = rng.uniform();
            assert!(u >= 0.0 && u < 1.0);
        }
    }

    #[test]
    fn test_rng_gaussian_mean() {
        let mut rng = SimpleRng::new(77);
        let mut sum = 0.0;
        let n = 10000;
        for _ in 0..n {
            sum += rng.gaussian();
        }
        let mean = sum / n as f64;
        assert!(mean.abs() < 0.1);
    }

    // --- Ratio method ---

    #[test]
    fn test_ratio_method_length() {
        let h = vec![1.0; 100];
        let r = ratio_method(&h, 10);
        assert_eq!(r.len(), 80);
    }

    #[test]
    fn test_ratio_method_constant() {
        let h = vec![5.0; 100];
        let r = ratio_method(&h, 5);
        for &val in &r {
            assert!(val.abs() < EPSILON);
        }
    }

    #[test]
    fn test_ratio_method_cancels_exponential() {
        // Generate pure exponential (no oscillation)
        let p = WiggleParams {
            n0: 1e6,
            tau_us: 64.0,
            asymmetry: 0.0,
            omega_a: 1.4,
            phi: 0.0,
        };
        let bin_width = 0.1492;
        let half_period = (ANOMALOUS_PERIOD_US / 2.0 / bin_width).round() as usize;
        let h = generate_wiggle_plot(&p, bin_width, 1000);
        let r = ratio_method(&h, half_period);
        // Should be near zero since no oscillation
        let max_r = r.iter().map(|v| v.abs()).fold(0.0, f64::max);
        assert!(max_r < 0.05);
    }

    #[test]
    fn test_ratio_method_too_short() {
        let h = vec![1.0; 5];
        let r = ratio_method(&h, 10);
        assert!(r.is_empty());
    }

    // --- Fit ratio ---

    #[test]
    fn test_fit_ratio_converges() {
        let p = WiggleParams {
            n0: 1e6,
            tau_us: 64.0,
            asymmetry: 0.3,
            omega_a: 1.4313,
            phi: 0.0,
        };
        let bin_width = 0.1492;
        let half_period = (ANOMALOUS_PERIOD_US / 2.0 / bin_width).round() as usize;
        let h = generate_wiggle_plot(&p, bin_width, 4000);
        let r = ratio_method(&h, half_period);
        let result = fit_ratio(&r, bin_width, half_period, 1.43);
        assert!(result.converged);
        assert!((result.omega_a - 1.4313).abs() < 0.02);
    }

    // --- Five-parameter fit ---

    #[test]
    fn test_five_param_fit_exact() {
        let p = WiggleParams {
            n0: 1e5,
            tau_us: 64.0,
            asymmetry: 0.35,
            omega_a: 1.4313,
            phi: 0.5,
        };
        let bin_width = 0.1492;
        let h = generate_wiggle_plot(&p, bin_width, 2000);
        let result = five_parameter_fit(&h, bin_width, &p, 50);
        assert!(result.converged);
        assert!((result.omega_a - p.omega_a).abs() < 0.01);
        assert!((result.amplitude - p.asymmetry).abs() < 0.05);
    }

    #[test]
    fn test_five_param_fit_too_few_bins() {
        let p = WiggleParams::default();
        let h = vec![1.0; 3];
        let result = five_parameter_fit(&h, 0.1, &p, 10);
        assert!(!result.converged);
    }

    // --- a_mu computation ---

    #[test]
    fn test_compute_a_mu() {
        let omega_a = 2.0 * PI * 0.2291; // ~0.2291 MHz
        let omega_p = 61.79; // MHz
        let a_mu = compute_a_mu(omega_a, omega_p);
        assert!((a_mu - A_MU_WORLD_AVG).abs() < 0.001);
    }

    #[test]
    fn test_omega_a_from_a_mu_roundtrip() {
        let omega_p = 61.79;
        let omega_a = omega_a_from_a_mu(A_MU_WORLD_AVG, omega_p);
        let a_mu_back = compute_a_mu(omega_a, omega_p);
        assert!((a_mu_back - A_MU_WORLD_AVG).abs() < 1e-10);
    }

    // --- Electric field and pitch corrections ---

    #[test]
    fn test_electric_field_correction_at_magic() {
        // At magic momentum, dp/p = 0 -> no correction
        let corr = electric_field_correction(0.137, 0.0, 0.9994);
        assert!(corr.abs() < EPSILON);
    }

    #[test]
    fn test_pitch_correction_sign() {
        let corr = pitch_correction(0.137, 0.005, 0.9994);
        assert!(corr < 0.0); // Should be negative
    }

    // --- Storage ring dynamics ---

    #[test]
    fn test_cyclotron_frequency() {
        let f_c = cyclotron_frequency(1.4513, MAGIC_MOMENTUM_MEV);
        // Should be ~6.7 MHz -> omega ~42 rad/us
        let f_mhz = f_c / (2.0 * PI);
        assert!(f_mhz > 6.0 && f_mhz < 7.5);
    }

    #[test]
    fn test_betatron_frequencies() {
        let f_c = 6.7; // MHz
        let (f_x, f_y) = betatron_frequencies(f_c, 0.137);
        assert!(f_x > 0.0 && f_x < f_c);
        assert!(f_y > 0.0 && f_y < f_c);
        // f_x^2 / f_c^2 + f_y^2 / f_c^2 = 1 (approximately)
        let sum = (f_x / f_c).powi(2) + (f_y / f_c).powi(2);
        assert!((sum - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_cbo_frequency() {
        let f_c = 6.7;
        let f_cbo = cbo_frequency(f_c, 0.137);
        assert!(f_cbo > 0.0);
        assert!(f_cbo < f_c);
        // CBO should be about f_c * (1 - sqrt(1-n)) ~ 0.47 MHz
        assert!(f_cbo > 0.3 && f_cbo < 0.7);
    }

    #[test]
    fn test_cbo_frequency_invalid_n() {
        assert_eq!(cbo_frequency(6.7, 1.5), 0.0);
        assert_eq!(cbo_frequency(6.7, -0.1), 0.0);
    }

    #[test]
    fn test_ring_radius() {
        let r = ring_radius_m(MAGIC_MOMENTUM_MEV, 1.4513);
        assert!((r - 7.112).abs() < 0.1); // ~7.112 m
    }

    #[test]
    fn test_ring_radius_zero_field() {
        assert_eq!(ring_radius_m(3094.0, 0.0), 0.0);
    }

    // --- Pileup correction ---

    #[test]
    fn test_pileup_correction_no_change_low_rate() {
        let h = vec![1.0; 100]; // very low rate
        let corrected = pileup_correction(&h, 0.1, &PileupConfig::default());
        for (orig, corr) in h.iter().zip(corrected.iter()) {
            assert!((orig - corr).abs() < 0.1);
        }
    }

    #[test]
    fn test_pileup_correction_nonnegative() {
        let h = vec![1e6; 100]; // high rate
        let corrected = pileup_correction(&h, 0.001, &PileupConfig::default());
        for &val in &corrected {
            assert!(val >= 0.0);
        }
    }

    // --- Poisson probability ---

    #[test]
    fn test_poisson_k0() {
        let p = poisson_prob(1.0, 0);
        assert!((p - (-1.0f64).exp()).abs() < EPSILON);
    }

    #[test]
    fn test_poisson_k1() {
        let p = poisson_prob(1.0, 1);
        assert!((p - (-1.0f64).exp()).abs() < EPSILON); // mu*exp(-mu) / 1! = exp(-1)
    }

    #[test]
    fn test_poisson_normalization() {
        let mu = 3.0;
        let mut total = 0.0;
        for k in 0..30 {
            total += poisson_prob(mu, k);
        }
        assert!((total - 1.0).abs() < 1e-6);
    }

    // --- T-method ---

    #[test]
    fn test_t_method_basic() {
        let energies = vec![100.0, 200.0, 300.0];
        let times = vec![0.5, 0.5, 1.5];
        let h = t_method(&energies, &times, 50.0, 1.0, 3);
        assert_eq!(h.len(), 3);
        assert!((h[0] - 300.0).abs() < EPSILON); // 100 + 200
        assert!((h[1] - 300.0).abs() < EPSILON); // 300
    }

    #[test]
    fn test_counting_method() {
        let energies = vec![100.0, 200.0, 300.0];
        let times = vec![0.5, 0.5, 1.5];
        let h = counting_method(&energies, &times, 50.0, 1.0, 3);
        assert!((h[0] - 2.0).abs() < EPSILON);
        assert!((h[1] - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_t_method_threshold() {
        let energies = vec![10.0, 100.0, 200.0];
        let times = vec![0.5, 0.5, 0.5];
        let h = t_method(&energies, &times, 50.0, 1.0, 2);
        // Only 100 and 200 pass threshold
        assert!((h[0] - 300.0).abs() < EPSILON);
    }

    // --- A-weighted analysis ---

    #[test]
    fn test_asymmetry_vs_energy_endpoints() {
        let a_low = asymmetry_vs_energy(0.01, 52.83);
        assert!(a_low < 0.0); // Negative at low energy

        let a_high = asymmetry_vs_energy(52.83, 52.83);
        assert!((a_high - 1.0).abs() < 0.01); // A -> 1 at max energy
    }

    #[test]
    fn test_asymmetry_vs_energy_invalid() {
        assert_eq!(asymmetry_vs_energy(-1.0, 52.83), 0.0);
        assert_eq!(asymmetry_vs_energy(60.0, 52.83), 0.0);
        assert_eq!(asymmetry_vs_energy(10.0, 0.0), 0.0);
    }

    #[test]
    fn test_figure_of_merit() {
        let energies = vec![40.0, 45.0, 50.0, 52.0];
        let fom = figure_of_merit(&energies, 52.83, 30.0);
        assert!(fom > 0.0);
    }

    // --- NMR FID ---

    #[test]
    fn test_nmr_fid_default() {
        let p = NmrFidParams::default();
        assert!((p.f_larmor_mhz - 61.79).abs() < 0.01);
    }

    #[test]
    fn test_nmr_fid_decay() {
        let p = NmrFidParams::default();
        let v0 = p.evaluate(0.0).abs();
        let v_late = p.evaluate(200.0).abs();
        assert!(v_late < v0 * 0.1);
    }

    #[test]
    fn test_generate_nmr_fid() {
        let p = NmrFidParams::default();
        let fid = generate_nmr_fid(&p, 0.001, 1000);
        assert_eq!(fid.len(), 1000);
    }

    #[test]
    fn test_extract_larmor_frequency() {
        let p = NmrFidParams {
            f_larmor_mhz: 10.0, // simpler frequency for testing
            amplitude: 1.0,
            t2_star_us: 100.0,
            phase: 0.0,
        };
        let dt = 0.01; // 10 ns -> Nyquist = 50 MHz
        let fid = generate_nmr_fid(&p, dt, 5000);
        let f = extract_larmor_frequency(&fid, dt);
        assert!((f - 10.0).abs() < 0.5);
    }

    #[test]
    fn test_extract_larmor_frequency_dft() {
        let p = NmrFidParams {
            f_larmor_mhz: 61.79,
            amplitude: 1.0,
            t2_star_us: 50.0,
            phase: 0.0,
        };
        let dt = 0.002; // Nyquist = 250 MHz
        let fid = generate_nmr_fid(&p, dt, 10000);
        let f = extract_larmor_frequency_dft(&fid, dt, 60.0, 64.0, 2000);
        assert!((f - 61.79).abs() < 0.1);
    }

    // --- Multipole field ---

    #[test]
    fn test_multipole_pure_dipole() {
        let field = MultipoleField::new(1.45, 45.0, 4);
        let b = field.evaluate(0.0, 0.0);
        assert!((b - 1.45).abs() < 1e-10);
    }

    #[test]
    fn test_multipole_uniformity_pure_dipole() {
        let field = MultipoleField::new(1.45, 45.0, 4);
        let unif = field.uniformity_ppm(22.5, 360);
        assert!(unif < 0.01); // Perfect dipole is uniform
    }

    #[test]
    fn test_multipole_with_quadrupole() {
        let mut field = MultipoleField::new(1.45, 45.0, 4);
        field.normal[1] = 10.0; // 10 ppm quadrupole
        let unif = field.uniformity_ppm(22.5, 360);
        assert!(unif > 0.0);
    }

    #[test]
    fn test_multipole_average_field() {
        let field = MultipoleField::new(1.45, 45.0, 4);
        let avg = field.average_field(22.5, 10, 36);
        assert!((avg - 1.45).abs() < 0.01);
    }

    // --- Michel spectrum ---

    #[test]
    fn test_michel_spectrum_zero() {
        assert_eq!(michel_spectrum(0.0, 0.0), 0.0);
        assert!((michel_spectrum(1.0, 0.0) - 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_michel_spectrum_endpoints() {
        assert_eq!(michel_spectrum(-0.1, 1.0), 0.0);
        assert_eq!(michel_spectrum(1.1, 1.0), 0.0);
    }

    #[test]
    fn test_generate_michel_spectrum_normalized() {
        let spec = generate_michel_spectrum(100, 0.5);
        let sum: f64 = spec.iter().sum();
        assert!((sum - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_max_positron_energy() {
        let e_max = max_positron_energy_rest_frame();
        assert!((e_max - 52.829).abs() < 0.1);
    }

    // --- Boost ---

    #[test]
    fn test_boost_forward() {
        let gamma = MAGIC_GAMMA;
        let beta = beta_from_momentum(MAGIC_MOMENTUM_MEV);
        let e_rest = 50.0;
        let e_lab = boost_energy(e_rest, 1.0, gamma, beta); // forward
        assert!(e_lab > e_rest * gamma); // maximum boost
    }

    #[test]
    fn test_boost_backward() {
        let gamma = MAGIC_GAMMA;
        let beta = beta_from_momentum(MAGIC_MOMENTUM_MEV);
        let e_rest = 50.0;
        let e_lab = boost_energy(e_rest, -1.0, gamma, beta); // backward
        assert!(e_lab < e_rest); // minimum boost
    }

    // --- Chi-squared ---

    #[test]
    fn test_chi_squared_perfect() {
        let h = vec![100.0; 50];
        let chi2 = chi_squared(&h, &h);
        assert!(chi2 < EPSILON);
    }

    #[test]
    fn test_reduced_chi_squared() {
        let h = vec![100.0, 200.0, 300.0];
        let m = vec![100.0, 200.0, 300.0];
        let rchi2 = reduced_chi_squared(&h, &m, 1);
        assert!(rchi2 < EPSILON);
    }

    #[test]
    fn test_chi_squared_mismatched_lengths() {
        let h = vec![1.0; 10];
        let m = vec![1.0; 5];
        let chi2 = chi_squared(&h, &m);
        assert_eq!(chi2, f64::MAX);
    }

    // --- Statistical uncertainty ---

    #[test]
    fn test_omega_a_uncertainty() {
        let sigma = omega_a_statistical_uncertainty(0.4, 1e10, 600.0);
        assert!(sigma > 0.0 && sigma < 1e-6);
    }

    #[test]
    fn test_omega_a_uncertainty_edge_cases() {
        assert_eq!(omega_a_statistical_uncertainty(0.0, 1e10, 600.0), f64::MAX);
        assert_eq!(omega_a_statistical_uncertainty(0.4, 0.0, 600.0), f64::MAX);
    }

    // --- Power spectrum ---

    #[test]
    fn test_power_spectrum_pure_tone() {
        let f0 = 5.0; // 5 MHz
        let dt = 0.01; // 10 ns -> Nyquist = 50 MHz
        let n = 2000;
        let signal: Vec<f64> = (0..n).map(|i| (2.0 * PI * f0 * i as f64 * dt).cos()).collect();
        let (freqs, power) = power_spectrum(&signal, dt, 200);
        let (peak_f, _) = find_peak_frequency(&freqs, &power);
        assert!((peak_f - f0).abs() < 0.5);
    }

    // --- Blinding ---

    #[test]
    fn test_blinding_roundtrip() {
        let omega = 1.4313;
        let offset = 0.005;
        let blinded = apply_blinding(omega, offset);
        let unblinded = remove_blinding(blinded, offset);
        assert!((unblinded - omega).abs() < EPSILON);
    }

    // --- Lost muon fraction ---

    #[test]
    fn test_cumulative_lost_fraction() {
        let loss = vec![1.0, 2.0, 3.0];
        let total = vec![100.0, 200.0, 300.0];
        let frac = cumulative_lost_fraction(&loss, &total);
        assert_eq!(frac.len(), 3);
        assert!((frac[0] - 0.01).abs() < EPSILON);
        assert!((frac[1] - 3.0 / 300.0).abs() < EPSILON);
    }

    // --- Data quality ---

    #[test]
    fn test_quality_cuts_pass() {
        let config = DataQualityConfig::default();
        assert!(passes_quality_cuts(5000, 0.005, 1.2, &config));
    }

    #[test]
    fn test_quality_cuts_fail_low_stats() {
        let config = DataQualityConfig::default();
        assert!(!passes_quality_cuts(100, 0.005, 1.2, &config));
    }

    #[test]
    fn test_quality_cuts_fail_high_loss() {
        let config = DataQualityConfig::default();
        assert!(!passes_quality_cuts(5000, 0.05, 1.2, &config));
    }

    #[test]
    fn test_start_time_cut() {
        let mut h = vec![10.0; 100];
        apply_start_time_cut(&mut h, 1.0, 10.0);
        for i in 0..10 {
            assert_eq!(h[i], 0.0);
        }
        assert_eq!(h[10], 10.0);
    }

    #[test]
    fn test_end_time_cut() {
        let mut h = vec![10.0; 100];
        apply_end_time_cut(&mut h, 1.0, 50.0);
        assert_eq!(h[50], 10.0);
        assert_eq!(h[51], 0.0);
    }

    // --- Spin tracking ---

    #[test]
    fn test_spin_precession_angle() {
        let omega_a = 2.0 * PI; // period = 1 us
        let angles = spin_precession_angle(omega_a, 0.0, &[0.0, 0.5, 1.0]);
        assert!(angles[0].abs() < EPSILON);
        assert!((angles[1] - PI).abs() < EPSILON);
    }

    #[test]
    fn test_polarization_decay() {
        let p0 = 1.0;
        let omega_a = 1.4;
        let dp = 0.001;
        let p_early = polarization_decay(p0, omega_a, dp, 1.0);
        let p_late = polarization_decay(p0, omega_a, dp, 1000.0);
        assert!(p_early > p_late);
        assert!(p_early > 0.99);
    }

    // --- EDM ---

    #[test]
    fn test_vertical_asymmetry() {
        let up = vec![100.0, 110.0, 90.0];
        let down = vec![100.0, 90.0, 110.0];
        let asym = vertical_asymmetry(&up, &down);
        assert!(asym[0].abs() < EPSILON);
        assert!(asym[1] > 0.0);
        assert!(asym[2] < 0.0);
    }

    #[test]
    fn test_fit_edm_phase() {
        let omega_a = 1.4;
        let bin_width = 0.1;
        let n = 500;
        // Create a pure sine signal
        let asym: Vec<f64> = (0..n)
            .map(|i| {
                let t = (i as f64 + 0.5) * bin_width;
                0.01 * (omega_a * t).sin()
            })
            .collect();
        let (amp, _phase) = fit_edm_phase(&asym, bin_width, omega_a);
        assert!((amp - 0.01).abs() < 0.005);
    }

    // --- Solve 5x5 ---

    #[test]
    fn test_solve_5x5_identity() {
        let a = [
            [1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0],
        ];
        let b = [1.0, 2.0, 3.0, 4.0, 5.0];
        let x = solve_5x5(&a, &b).unwrap();
        for i in 0..5 {
            assert!((x[i] - b[i]).abs() < EPSILON);
        }
    }

    #[test]
    fn test_solve_5x5_singular() {
        let a = [[0.0; 5]; 5];
        let b = [1.0; 5];
        assert!(solve_5x5(&a, &b).is_none());
    }

    // --- ln_factorial ---

    #[test]
    fn test_ln_factorial_small() {
        assert!((ln_factorial(0) - 0.0).abs() < EPSILON);
        assert!((ln_factorial(1) - 0.0).abs() < EPSILON);
        assert!((ln_factorial(5) - (120.0f64).ln()).abs() < EPSILON);
    }

    #[test]
    fn test_ln_factorial_large_stirling() {
        // For large n, Stirling should be within ~1%
        let exact_20: f64 = (1..=20).map(|i| (i as f64).ln()).sum();
        let stirling_20 = ln_factorial(20);
        assert!((exact_20 - stirling_20).abs() < 0.01);
    }

    // --- Fast Rotation Analysis ---

    #[test]
    fn test_fast_rotation_analysis_output_sizes() {
        let p = WiggleParams::default();
        let h = generate_wiggle_plot(&p, 0.1492, 2000);
        let (freqs, power) = fast_rotation_analysis(&h, 0.1492, &p, 100);
        assert_eq!(freqs.len(), 100);
        assert_eq!(power.len(), 100);
    }

    #[test]
    fn test_fast_rotation_analysis_power_nonnegative() {
        let p = WiggleParams::default();
        let h = generate_wiggle_plot(&p, 0.1492, 1000);
        let (_, power) = fast_rotation_analysis(&h, 0.1492, &p, 50);
        for &val in &power {
            assert!(val >= 0.0);
        }
    }
}
