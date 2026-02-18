// trace:FR-DMA | ai:claude
//! # Dynamic Mechanical Analysis (DMA) Processor
//!
//! Signal processing for measuring viscoelastic properties of polymers and composites.
//! Implements complex modulus calculation, frequency/temperature sweeps, time-temperature
//! superposition (TTS), Maxwell model fitting, and Havriliak-Negami relaxation.

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex Modulus
// ---------------------------------------------------------------------------

/// Complex modulus representation: E* = E' + iE''
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ComplexModulus {
    /// Storage modulus (elastic component) in Pa
    pub storage: f64,
    /// Loss modulus (viscous component) in Pa
    pub loss: f64,
}

impl ComplexModulus {
    pub fn new(storage: f64, loss: f64) -> Self {
        Self { storage, loss }
    }

    /// |E*| = sqrt(E'^2 + E''^2)
    pub fn magnitude(&self) -> f64 {
        (self.storage * self.storage + self.loss * self.loss).sqrt()
    }

    /// tan delta = E'' / E'
    pub fn tan_delta(&self) -> f64 {
        if self.storage.abs() < 1e-30 {
            return f64::INFINITY;
        }
        self.loss / self.storage
    }

    /// Phase angle delta in radians
    pub fn phase_angle_rad(&self) -> f64 {
        self.loss.atan2(self.storage)
    }

    /// Phase angle delta in degrees
    pub fn phase_angle_deg(&self) -> f64 {
        self.phase_angle_rad() * 180.0 / PI
    }

    /// Convert tensile E* to shear G* assuming Poisson's ratio nu
    /// G = E / (2(1+nu))
    pub fn to_shear(&self, poisson_ratio: f64) -> ComplexModulus {
        let factor = 1.0 / (2.0 * (1.0 + poisson_ratio));
        ComplexModulus {
            storage: self.storage * factor,
            loss: self.loss * factor,
        }
    }

    /// Loss factor (same as tan_delta, common alias)
    pub fn loss_factor(&self) -> f64 {
        self.tan_delta()
    }
}

/// Compute storage and loss modulus from stress amplitude, strain amplitude, and phase lag.
///
/// E' = (stress_amp / strain_amp) * cos(delta)
/// E'' = (stress_amp / strain_amp) * sin(delta)
pub fn complex_modulus(stress_amp: f64, strain_amp: f64, delta_rad: f64) -> (f64, f64) {
    let ratio = stress_amp / strain_amp;
    (ratio * delta_rad.cos(), ratio * delta_rad.sin())
}

/// Build a ComplexModulus from raw amplitudes and phase lag.
pub fn complex_modulus_struct(stress_amp: f64, strain_amp: f64, delta_rad: f64) -> ComplexModulus {
    let (s, l) = complex_modulus(stress_amp, strain_amp, delta_rad);
    ComplexModulus::new(s, l)
}

// ---------------------------------------------------------------------------
// Clamp Geometry
// ---------------------------------------------------------------------------

/// Sample geometry / clamping mode for DMA
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ClampGeometry {
    /// Single cantilever bending
    SingleCantilever,
    /// Dual cantilever bending
    DualCantilever,
    /// Three-point bending
    ThreePointBend,
    /// Tension (film / fiber)
    Tension,
    /// Compression
    Compression,
    /// Shear sandwich
    ShearSandwich,
}

/// Sample dimensions in metres
#[derive(Debug, Clone, Copy)]
pub struct SampleDimensions {
    pub length: f64,
    pub width: f64,
    pub thickness: f64,
}

impl SampleDimensions {
    pub fn new(length: f64, width: f64, thickness: f64) -> Self {
        Self { length, width, thickness }
    }

    /// Cross-sectional area (width * thickness)
    pub fn cross_section_area(&self) -> f64 {
        self.width * self.thickness
    }

    /// Second moment of area for rectangular cross-section
    pub fn moment_of_inertia(&self) -> f64 {
        self.width * self.thickness.powi(3) / 12.0
    }
}

/// Compute geometry stiffness factor K_g such that E = K_g * (force / displacement).
/// Units: 1/m  (so E [Pa] = K_g [1/m] * stiffness [N/m] ... careful with conventions)
pub fn geometry_stiffness_factor(geom: ClampGeometry, dim: &SampleDimensions) -> f64 {
    let l = dim.length;
    let w = dim.width;
    let t = dim.thickness;
    let i = dim.moment_of_inertia();
    match geom {
        ClampGeometry::SingleCantilever => {
            // K = 3EI/L^3  =>  E = K * L^3 / (3I)
            l.powi(3) / (3.0 * i)
        }
        ClampGeometry::DualCantilever => {
            // K = 192EI/L^3  =>  E = K * L^3 / (192I)
            l.powi(3) / (192.0 * i)
        }
        ClampGeometry::ThreePointBend => {
            // K = 48EI/L^3  =>  E = K * L^3 / (48I)
            l.powi(3) / (48.0 * i)
        }
        ClampGeometry::Tension => {
            // sigma = F/A, epsilon = dL/L  =>  E = (F/dL) * (L/A)
            l / (w * t)
        }
        ClampGeometry::Compression => {
            // Same as tension but sign convention differs; factor identical
            l / (w * t)
        }
        ClampGeometry::ShearSandwich => {
            // G = (F/A) / (dx/t)  =>  G = (F/dx) * t/A
            // Two shear layers: effective thickness = t/2 each
            t / (2.0 * w * l)
        }
    }
}

// ---------------------------------------------------------------------------
// WLF Equation (Williams-Landel-Ferry)
// ---------------------------------------------------------------------------

/// Universal WLF constants (referenced to Tg)
pub const WLF_C1_UNIVERSAL: f64 = 17.44;
pub const WLF_C2_UNIVERSAL: f64 = 51.6;

/// WLF shift factor: log10(aT) = -C1*(T - Tref) / (C2 + (T - Tref))
/// Returns log10(aT).
pub fn wlf_shift_factor(temp: f64, tref: f64, c1: f64, c2: f64) -> f64 {
    let dt = temp - tref;
    -c1 * dt / (c2 + dt)
}

/// WLF shift factor as the actual multiplier aT (not log).
pub fn wlf_shift_factor_linear(temp: f64, tref: f64, c1: f64, c2: f64) -> f64 {
    let log_at = wlf_shift_factor(temp, tref, c1, c2);
    10.0_f64.powf(log_at)
}

/// Inverse WLF: given log10(aT), solve for T.
/// T = Tref + C2 * log_at / (-C1 - log_at)
pub fn wlf_inverse_temperature(log_at: f64, tref: f64, c1: f64, c2: f64) -> f64 {
    tref + c2 * log_at / (-c1 - log_at)
}

// ---------------------------------------------------------------------------
// Arrhenius Activation Energy
// ---------------------------------------------------------------------------

/// Gas constant R in J/(mol*K)
pub const GAS_CONSTANT_R: f64 = 8.314;

/// Arrhenius shift factor: ln(aT) = (Ea/R) * (1/T - 1/Tref)
/// Returns ln(aT).
pub fn arrhenius_shift_factor(temp_k: f64, tref_k: f64, ea: f64) -> f64 {
    (ea / GAS_CONSTANT_R) * (1.0 / temp_k - 1.0 / tref_k)
}

/// Estimate activation energy from two (frequency, Tg) pairs.
/// Ea = -R * ln(f2/f1) / (1/Tg2 - 1/Tg1)
pub fn activation_energy_from_tg_pair(f1: f64, tg1_k: f64, f2: f64, tg2_k: f64) -> f64 {
    let ln_ratio = (f2 / f1).ln();
    let inv_t_diff = 1.0 / tg2_k - 1.0 / tg1_k;
    -GAS_CONSTANT_R * ln_ratio / inv_t_diff
}

/// Estimate activation energy from multiple (frequency, Tg) pairs using least-squares
/// on the Arrhenius plot: ln(f) vs 1/Tg => slope = -Ea/R.
pub fn activation_energy_least_squares(freq_tg_pairs: &[(f64, f64)]) -> f64 {
    let n = freq_tg_pairs.len() as f64;
    if n < 2.0 {
        return 0.0;
    }
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut sum_xy = 0.0;
    let mut sum_xx = 0.0;
    for &(f, tg_k) in freq_tg_pairs {
        let x = 1.0 / tg_k;
        let y = f.ln();
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_xx += x * x;
    }
    let slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
    // slope = -Ea/R  =>  Ea = -slope * R
    -slope * GAS_CONSTANT_R
}

// ---------------------------------------------------------------------------
// Generalized Maxwell (Wiechert) Model / Prony Series
// ---------------------------------------------------------------------------

/// Evaluate Prony series relaxation modulus:
/// E(t) = E_inf + sum_i E_i * exp(-t / tau_i)
pub fn prony_relaxation(t: f64, e_inf: f64, e_i: &[f64], tau_i: &[f64]) -> f64 {
    let mut val = e_inf;
    for (ei, ti) in e_i.iter().zip(tau_i.iter()) {
        val += ei * (-t / ti).exp();
    }
    val
}

/// Evaluate storage modulus from Prony series at angular frequency omega:
/// E'(omega) = E_inf + sum_i E_i * (omega*tau_i)^2 / (1 + (omega*tau_i)^2)
pub fn prony_storage_modulus(omega: f64, e_inf: f64, e_i: &[f64], tau_i: &[f64]) -> f64 {
    let mut val = e_inf;
    for (ei, ti) in e_i.iter().zip(tau_i.iter()) {
        let wt = omega * ti;
        val += ei * wt * wt / (1.0 + wt * wt);
    }
    val
}

/// Evaluate loss modulus from Prony series at angular frequency omega:
/// E''(omega) = sum_i E_i * omega*tau_i / (1 + (omega*tau_i)^2)
pub fn prony_loss_modulus(omega: f64, e_i: &[f64], tau_i: &[f64]) -> f64 {
    let mut val = 0.0;
    for (ei, ti) in e_i.iter().zip(tau_i.iter()) {
        let wt = omega * ti;
        val += ei * wt / (1.0 + wt * wt);
    }
    val
}

/// Complex modulus from Prony series at angular frequency omega.
pub fn prony_complex_modulus(omega: f64, e_inf: f64, e_i: &[f64], tau_i: &[f64]) -> ComplexModulus {
    ComplexModulus {
        storage: prony_storage_modulus(omega, e_inf, e_i, tau_i),
        loss: prony_loss_modulus(omega, e_i, tau_i),
    }
}

/// Generalized Maxwell model parameters.
#[derive(Debug, Clone)]
pub struct MaxwellModel {
    /// Equilibrium (rubbery) modulus
    pub e_inf: f64,
    /// Spring constants of Maxwell elements
    pub e_i: Vec<f64>,
    /// Relaxation times of Maxwell elements
    pub tau_i: Vec<f64>,
}

impl MaxwellModel {
    pub fn new(e_inf: f64, e_i: Vec<f64>, tau_i: Vec<f64>) -> Self {
        assert_eq!(e_i.len(), tau_i.len());
        Self { e_inf, e_i, tau_i }
    }

    pub fn num_elements(&self) -> usize {
        self.e_i.len()
    }

    /// Relaxation modulus E(t)
    pub fn relaxation(&self, t: f64) -> f64 {
        prony_relaxation(t, self.e_inf, &self.e_i, &self.tau_i)
    }

    /// Storage modulus E'(omega)
    pub fn storage_modulus(&self, omega: f64) -> f64 {
        prony_storage_modulus(omega, self.e_inf, &self.e_i, &self.tau_i)
    }

    /// Loss modulus E''(omega)
    pub fn loss_modulus(&self, omega: f64) -> f64 {
        prony_loss_modulus(omega, &self.e_i, &self.tau_i)
    }

    /// Complex modulus at angular frequency omega
    pub fn complex_modulus(&self, omega: f64) -> ComplexModulus {
        prony_complex_modulus(omega, self.e_inf, &self.e_i, &self.tau_i)
    }

    /// tan delta at angular frequency omega
    pub fn tan_delta(&self, omega: f64) -> f64 {
        self.complex_modulus(omega).tan_delta()
    }

    /// Instantaneous (glassy) modulus: E(0) = E_inf + sum(E_i)
    pub fn glassy_modulus(&self) -> f64 {
        self.e_inf + self.e_i.iter().sum::<f64>()
    }

    /// Fit a generalized Maxwell model from frequency-domain data using
    /// iterative non-negative least squares on a pre-defined relaxation time grid.
    /// tau_grid: relaxation time grid (e.g., logarithmically spaced)
    /// omega_data, storage_data, loss_data: measured frequency sweep data
    pub fn fit_from_frequency_data(
        tau_grid: &[f64],
        omega_data: &[f64],
        storage_data: &[f64],
        loss_data: &[f64],
        e_inf_guess: f64,
        iterations: usize,
    ) -> Self {
        let n_tau = tau_grid.len();
        let n_freq = omega_data.len();

        // Use simple iterative approach:
        // For each iteration, solve for E_i using gradient descent
        let mut e_i = vec![0.0; n_tau];
        let mut e_inf = e_inf_guess;

        // Initialize with uniform distribution
        let total_init = if !storage_data.is_empty() {
            storage_data[storage_data.len() - 1] - e_inf_guess
        } else {
            1.0
        };
        let per_element = (total_init / n_tau as f64).max(0.0);
        for ei in e_i.iter_mut() {
            *ei = per_element;
        }

        let lr = 0.01;

        for _iter in 0..iterations {
            // Compute gradients
            let mut grad_ei = vec![0.0; n_tau];
            let mut grad_einf = 0.0;

            for k in 0..n_freq {
                let w = omega_data[k];

                // Storage modulus error
                let s_pred = prony_storage_modulus(w, e_inf, &e_i, tau_grid);
                let s_err = s_pred - storage_data[k];

                // Loss modulus error
                let l_pred = prony_loss_modulus(w, &e_i, tau_grid);
                let l_err = l_pred - loss_data[k];

                grad_einf += 2.0 * s_err;

                for j in 0..n_tau {
                    let wt = w * tau_grid[j];
                    let wt2 = wt * wt;
                    let denom = 1.0 + wt2;
                    grad_ei[j] += 2.0 * s_err * wt2 / denom;
                    grad_ei[j] += 2.0 * l_err * wt / denom;
                }
            }

            // Update
            let scale = 1.0 / (n_freq as f64).max(1.0);
            e_inf -= lr * grad_einf * scale;
            e_inf = e_inf.max(0.0);
            for j in 0..n_tau {
                e_i[j] -= lr * grad_ei[j] * scale;
                e_i[j] = e_i[j].max(0.0); // non-negative constraint
            }
        }

        Self {
            e_inf,
            e_i,
            tau_i: tau_grid.to_vec(),
        }
    }
}

// ---------------------------------------------------------------------------
// Havriliak-Negami Relaxation
// ---------------------------------------------------------------------------

/// Havriliak-Negami relaxation model.
///
/// epsilon*(omega) = eps_inf + (eps_s - eps_inf) / [1 + (i*omega*tau)^alpha]^beta
///
/// Special cases:
/// - alpha=1, beta=1: Debye relaxation
/// - alpha<1, beta=1: Cole-Cole
/// - alpha=1, beta<1: Cole-Davidson
#[derive(Debug, Clone, Copy)]
pub struct HavriliakNegami {
    /// High-frequency permittivity (or modulus)
    pub eps_inf: f64,
    /// Static permittivity (or modulus)
    pub eps_s: f64,
    /// Characteristic relaxation time
    pub tau: f64,
    /// Broadening parameter (0 < alpha <= 1)
    pub alpha: f64,
    /// Asymmetry parameter (0 < beta <= 1)
    pub beta: f64,
}

impl HavriliakNegami {
    pub fn new(eps_inf: f64, eps_s: f64, tau: f64, alpha: f64, beta: f64) -> Self {
        Self { eps_inf, eps_s, tau, alpha, beta }
    }

    /// Debye model (alpha=1, beta=1)
    pub fn debye(eps_inf: f64, eps_s: f64, tau: f64) -> Self {
        Self::new(eps_inf, eps_s, tau, 1.0, 1.0)
    }

    /// Cole-Cole model (beta=1)
    pub fn cole_cole(eps_inf: f64, eps_s: f64, tau: f64, alpha: f64) -> Self {
        Self::new(eps_inf, eps_s, tau, alpha, 1.0)
    }

    /// Cole-Davidson model (alpha=1)
    pub fn cole_davidson(eps_inf: f64, eps_s: f64, tau: f64, beta: f64) -> Self {
        Self::new(eps_inf, eps_s, tau, 1.0, beta)
    }

    /// Evaluate complex permittivity at angular frequency omega.
    /// Returns (real_part, imag_part).
    ///
    /// [1 + (i*omega*tau)^alpha]^beta computed via polar form.
    pub fn evaluate(&self, omega: f64) -> (f64, f64) {
        let wt = omega * self.tau;

        // (i*wt)^alpha: i = e^(i*pi/2), so (i*wt)^alpha = wt^alpha * e^(i*alpha*pi/2)
        let r_alpha = wt.powf(self.alpha);
        let theta_alpha = self.alpha * PI / 2.0;

        // 1 + r_alpha * (cos(theta_alpha) + i*sin(theta_alpha))
        let re1 = 1.0 + r_alpha * theta_alpha.cos();
        let im1 = r_alpha * theta_alpha.sin();

        // Convert to polar for exponentiation by beta
        let mag1 = (re1 * re1 + im1 * im1).sqrt();
        let ang1 = im1.atan2(re1);

        // [...]^beta
        let mag_beta = mag1.powf(self.beta);
        let ang_beta = self.beta * ang1;

        // 1 / [...]^beta
        let inv_mag = 1.0 / mag_beta;
        let inv_re = inv_mag * ang_beta.cos();
        let inv_im = -inv_mag * ang_beta.sin();

        // eps* = eps_inf + delta_eps * (inv_re + i*inv_im)
        let delta = self.eps_s - self.eps_inf;
        (self.eps_inf + delta * inv_re, delta * inv_im)
    }

    /// Storage component at omega
    pub fn storage(&self, omega: f64) -> f64 {
        self.evaluate(omega).0
    }

    /// Loss component at omega
    pub fn loss(&self, omega: f64) -> f64 {
        // Loss is -imag part (convention: eps'' > 0 for loss)
        -self.evaluate(omega).1
    }

    /// tan delta at omega
    pub fn tan_delta(&self, omega: f64) -> f64 {
        let (re, im) = self.evaluate(omega);
        -im / re
    }

    /// Relaxation strength delta_eps = eps_s - eps_inf
    pub fn relaxation_strength(&self) -> f64 {
        self.eps_s - self.eps_inf
    }
}

// ---------------------------------------------------------------------------
// Frequency Sweep
// ---------------------------------------------------------------------------

/// A single data point in a frequency sweep.
#[derive(Debug, Clone, Copy)]
pub struct FrequencySweepPoint {
    pub frequency_hz: f64,
    pub storage_modulus: f64,
    pub loss_modulus: f64,
}

impl FrequencySweepPoint {
    pub fn tan_delta(&self) -> f64 {
        if self.storage_modulus.abs() < 1e-30 {
            return f64::INFINITY;
        }
        self.loss_modulus / self.storage_modulus
    }

    pub fn complex_modulus_mag(&self) -> f64 {
        (self.storage_modulus.powi(2) + self.loss_modulus.powi(2)).sqrt()
    }
}

/// Frequency sweep data set.
#[derive(Debug, Clone)]
pub struct FrequencySweep {
    pub points: Vec<FrequencySweepPoint>,
    pub temperature_c: f64,
}

impl FrequencySweep {
    pub fn new(temperature_c: f64) -> Self {
        Self { points: Vec::new(), temperature_c }
    }

    pub fn add_point(&mut self, freq_hz: f64, storage: f64, loss: f64) {
        self.points.push(FrequencySweepPoint {
            frequency_hz: freq_hz,
            storage_modulus: storage,
            loss_modulus: loss,
        });
    }

    /// Cole-Cole plot data: returns (E', E'') pairs.
    pub fn cole_cole_data(&self) -> Vec<(f64, f64)> {
        self.points.iter().map(|p| (p.storage_modulus, p.loss_modulus)).collect()
    }

    /// Wicket plot data: returns (log10(|E*|), tan_delta) pairs.
    pub fn wicket_plot_data(&self) -> Vec<(f64, f64)> {
        self.points.iter().map(|p| {
            let mag = p.complex_modulus_mag();
            (mag.log10(), p.tan_delta())
        }).collect()
    }

    /// Find frequency of peak loss modulus
    pub fn peak_loss_frequency(&self) -> Option<f64> {
        self.points.iter()
            .max_by(|a, b| a.loss_modulus.partial_cmp(&b.loss_modulus).unwrap())
            .map(|p| p.frequency_hz)
    }

    /// Find frequency of peak tan delta
    pub fn peak_tan_delta_frequency(&self) -> Option<f64> {
        self.points.iter()
            .max_by(|a, b| a.tan_delta().partial_cmp(&b.tan_delta()).unwrap())
            .map(|p| p.frequency_hz)
    }
}

// ---------------------------------------------------------------------------
// Time-Temperature Superposition (TTS) / Master Curve
// ---------------------------------------------------------------------------

/// Apply WLF shift to a frequency sweep, producing shifted frequencies.
pub fn apply_tts_shift(sweep: &FrequencySweep, tref: f64, c1: f64, c2: f64) -> Vec<FrequencySweepPoint> {
    let log_at = wlf_shift_factor(sweep.temperature_c, tref, c1, c2);
    let at = 10.0_f64.powf(log_at);
    sweep.points.iter().map(|p| FrequencySweepPoint {
        frequency_hz: p.frequency_hz * at,
        storage_modulus: p.storage_modulus,
        loss_modulus: p.loss_modulus,
    }).collect()
}

/// Construct a master curve from multiple frequency sweeps at different temperatures.
pub fn construct_master_curve(
    sweeps: &[FrequencySweep],
    tref: f64,
    c1: f64,
    c2: f64,
) -> Vec<FrequencySweepPoint> {
    let mut all_points = Vec::new();
    for sweep in sweeps {
        let shifted = apply_tts_shift(sweep, tref, c1, c2);
        all_points.extend(shifted);
    }
    // Sort by frequency
    all_points.sort_by(|a, b| a.frequency_hz.partial_cmp(&b.frequency_hz).unwrap());
    all_points
}

// ---------------------------------------------------------------------------
// Temperature Sweep
// ---------------------------------------------------------------------------

/// A single data point in a temperature sweep.
#[derive(Debug, Clone, Copy)]
pub struct TemperatureSweepPoint {
    pub temperature_c: f64,
    pub storage_modulus: f64,
    pub loss_modulus: f64,
}

impl TemperatureSweepPoint {
    pub fn tan_delta(&self) -> f64 {
        if self.storage_modulus.abs() < 1e-30 {
            return f64::INFINITY;
        }
        self.loss_modulus / self.storage_modulus
    }
}

/// Temperature sweep data set.
#[derive(Debug, Clone)]
pub struct TemperatureSweep {
    pub points: Vec<TemperatureSweepPoint>,
    pub frequency_hz: f64,
}

impl TemperatureSweep {
    pub fn new(frequency_hz: f64) -> Self {
        Self { points: Vec::new(), frequency_hz }
    }

    pub fn add_point(&mut self, temp_c: f64, storage: f64, loss: f64) {
        self.points.push(TemperatureSweepPoint {
            temperature_c: temp_c,
            storage_modulus: storage,
            loss_modulus: loss,
        });
    }

    /// Detect Tg from tan delta peak (most common method).
    pub fn tg_from_tan_delta_peak(&self) -> Option<f64> {
        self.points.iter()
            .max_by(|a, b| a.tan_delta().partial_cmp(&b.tan_delta()).unwrap())
            .map(|p| p.temperature_c)
    }

    /// Detect Tg from loss modulus (E'') peak.
    pub fn tg_from_loss_peak(&self) -> Option<f64> {
        self.points.iter()
            .max_by(|a, b| a.loss_modulus.partial_cmp(&b.loss_modulus).unwrap())
            .map(|p| p.temperature_c)
    }

    /// Detect Tg from storage modulus (E') onset: point of maximum negative slope.
    /// Uses finite differences on sorted data.
    pub fn tg_from_storage_onset(&self) -> Option<f64> {
        if self.points.len() < 3 {
            return None;
        }
        let mut sorted: Vec<_> = self.points.clone();
        sorted.sort_by(|a, b| a.temperature_c.partial_cmp(&b.temperature_c).unwrap());

        let mut min_slope = f64::MAX;
        let mut tg = sorted[0].temperature_c;

        for i in 1..sorted.len() {
            let dt = sorted[i].temperature_c - sorted[i - 1].temperature_c;
            if dt.abs() < 1e-12 {
                continue;
            }
            let slope = (sorted[i].storage_modulus - sorted[i - 1].storage_modulus) / dt;
            if slope < min_slope {
                min_slope = slope;
                tg = (sorted[i].temperature_c + sorted[i - 1].temperature_c) / 2.0;
            }
        }
        Some(tg)
    }

    /// Find relaxation peaks in tan delta curve.
    /// Returns temperatures of local maxima (alpha, beta, gamma transitions).
    pub fn find_relaxation_peaks(&self) -> Vec<f64> {
        if self.points.len() < 3 {
            return Vec::new();
        }
        let mut sorted: Vec<_> = self.points.clone();
        sorted.sort_by(|a, b| a.temperature_c.partial_cmp(&b.temperature_c).unwrap());

        let tan_deltas: Vec<f64> = sorted.iter().map(|p| p.tan_delta()).collect();
        let mut peaks = Vec::new();

        for i in 1..tan_deltas.len() - 1 {
            if tan_deltas[i] > tan_deltas[i - 1] && tan_deltas[i] > tan_deltas[i + 1] {
                peaks.push(sorted[i].temperature_c);
            }
        }
        peaks
    }

    /// Compute the ratio E'(glassy) / E'(rubbery) from extremes of the sweep.
    pub fn modulus_drop_ratio(&self) -> f64 {
        if self.points.is_empty() {
            return 1.0;
        }
        let mut sorted: Vec<_> = self.points.clone();
        sorted.sort_by(|a, b| a.temperature_c.partial_cmp(&b.temperature_c).unwrap());
        let e_low = sorted.first().unwrap().storage_modulus;
        let e_high = sorted.last().unwrap().storage_modulus;
        if e_high.abs() < 1e-30 {
            return f64::INFINITY;
        }
        e_low / e_high
    }
}

// ---------------------------------------------------------------------------
// Linear Viscoelastic Region (LVR)
// ---------------------------------------------------------------------------

/// Amplitude (strain) sweep data point.
#[derive(Debug, Clone, Copy)]
pub struct AmplitudeSweepPoint {
    pub strain: f64,
    pub storage_modulus: f64,
    pub loss_modulus: f64,
}

/// Determine the linear viscoelastic region from an amplitude sweep.
#[derive(Debug, Clone)]
pub struct LinearViscoelasticRegion {
    pub points: Vec<AmplitudeSweepPoint>,
}

impl LinearViscoelasticRegion {
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    pub fn add_point(&mut self, strain: f64, storage: f64, loss: f64) {
        self.points.push(AmplitudeSweepPoint { strain, storage_modulus: storage, loss_modulus: loss });
    }

    /// Find the critical strain where E' drops by a given percentage from its plateau.
    /// threshold_pct: e.g. 0.05 for 5% drop
    pub fn critical_strain(&self, threshold_pct: f64) -> Option<f64> {
        if self.points.is_empty() {
            return None;
        }
        let mut sorted: Vec<_> = self.points.clone();
        sorted.sort_by(|a, b| a.strain.partial_cmp(&b.strain).unwrap());

        // Plateau is taken from the first few points (lowest strain)
        let n_plateau = (sorted.len() / 4).max(1);
        let plateau: f64 = sorted[..n_plateau].iter()
            .map(|p| p.storage_modulus)
            .sum::<f64>() / n_plateau as f64;

        let limit = plateau * (1.0 - threshold_pct);

        for p in &sorted {
            if p.storage_modulus < limit {
                return Some(p.strain);
            }
        }
        None
    }

    /// Average storage modulus in the LVR (plateau region).
    pub fn lvr_modulus(&self) -> Option<f64> {
        if self.points.is_empty() {
            return None;
        }
        let mut sorted: Vec<_> = self.points.clone();
        sorted.sort_by(|a, b| a.strain.partial_cmp(&b.strain).unwrap());
        let n_plateau = (sorted.len() / 4).max(1);
        let avg = sorted[..n_plateau].iter()
            .map(|p| p.storage_modulus)
            .sum::<f64>() / n_plateau as f64;
        Some(avg)
    }

    /// Determine if a given strain is within the LVR.
    pub fn is_in_lvr(&self, strain: f64, threshold_pct: f64) -> bool {
        match self.critical_strain(threshold_pct) {
            Some(crit) => strain < crit,
            None => true, // No drop detected, assume all in LVR
        }
    }
}

// ---------------------------------------------------------------------------
// DMA Processor (streaming)
// ---------------------------------------------------------------------------

/// Configuration for the DMA processor.
#[derive(Debug, Clone)]
pub struct DmaProcessorConfig {
    pub sample_rate_hz: f64,
    pub excitation_frequency_hz: f64,
    pub geometry: ClampGeometry,
    pub dimensions: SampleDimensions,
}

/// Streaming DMA signal processor.
/// Processes raw force/displacement waveforms to extract viscoelastic properties.
pub struct DmaProcessor {
    config: DmaProcessorConfig,
    /// Accumulator for force signal (sine fitting)
    force_sin_acc: f64,
    force_cos_acc: f64,
    /// Accumulator for displacement signal
    disp_sin_acc: f64,
    disp_cos_acc: f64,
    /// Sample count in current measurement window
    sample_count: usize,
    /// Phase increment per sample
    phase_increment: f64,
    /// Current phase
    phase: f64,
    /// Geometry factor
    geom_factor: f64,
}

impl DmaProcessor {
    pub fn new(config: DmaProcessorConfig) -> Self {
        let phase_increment = 2.0 * PI * config.excitation_frequency_hz / config.sample_rate_hz;
        let geom_factor = geometry_stiffness_factor(config.geometry, &config.dimensions);
        Self {
            config,
            force_sin_acc: 0.0,
            force_cos_acc: 0.0,
            disp_sin_acc: 0.0,
            disp_cos_acc: 0.0,
            sample_count: 0,
            phase_increment,
            phase: 0.0,
            geom_factor,
        }
    }

    /// Process a pair of (force, displacement) samples.
    pub fn process_sample(&mut self, force: f64, displacement: f64) {
        let sin_val = self.phase.sin();
        let cos_val = self.phase.cos();

        self.force_sin_acc += force * sin_val;
        self.force_cos_acc += force * cos_val;
        self.disp_sin_acc += displacement * sin_val;
        self.disp_cos_acc += displacement * cos_val;

        self.sample_count += 1;
        self.phase += self.phase_increment;
        if self.phase > 2.0 * PI {
            self.phase -= 2.0 * PI;
        }
    }

    /// Process a batch of (force, displacement) pairs.
    pub fn process_batch(&mut self, force: &[f64], displacement: &[f64]) {
        let n = force.len().min(displacement.len());
        for i in 0..n {
            self.process_sample(force[i], displacement[i]);
        }
    }

    /// Extract the complex modulus from accumulated data.
    /// Call after processing at least one full cycle.
    pub fn extract_modulus(&self) -> Option<ComplexModulus> {
        if self.sample_count == 0 {
            return None;
        }
        let n = self.sample_count as f64;

        // Force amplitude and phase from Fourier coefficients
        let f_sin = 2.0 * self.force_sin_acc / n;
        let f_cos = 2.0 * self.force_cos_acc / n;
        let force_amp = (f_sin * f_sin + f_cos * f_cos).sqrt();
        let force_phase = f_sin.atan2(f_cos);

        // Displacement amplitude and phase
        let d_sin = 2.0 * self.disp_sin_acc / n;
        let d_cos = 2.0 * self.disp_cos_acc / n;
        let disp_amp = (d_sin * d_sin + d_cos * d_cos).sqrt();
        let disp_phase = d_sin.atan2(d_cos);

        if disp_amp < 1e-30 {
            return None;
        }

        // Phase difference (delta): displacement lags force in viscoelastic materials
        let delta = disp_phase - force_phase;

        // Stiffness = force_amp / disp_amp
        let stiffness = force_amp / disp_amp;

        // Modulus = geometry_factor * stiffness
        let modulus_mag = self.geom_factor * stiffness;

        Some(ComplexModulus {
            storage: modulus_mag * delta.cos(),
            loss: modulus_mag * delta.sin(),
        })
    }

    /// Reset accumulators for a new measurement window.
    pub fn reset(&mut self) {
        self.force_sin_acc = 0.0;
        self.force_cos_acc = 0.0;
        self.disp_sin_acc = 0.0;
        self.disp_cos_acc = 0.0;
        self.sample_count = 0;
        self.phase = 0.0;
    }

    /// Number of samples per excitation cycle.
    pub fn samples_per_cycle(&self) -> usize {
        (self.config.sample_rate_hz / self.config.excitation_frequency_hz).round() as usize
    }

    /// Get the geometry factor.
    pub fn geometry_factor(&self) -> f64 {
        self.geom_factor
    }
}

// ---------------------------------------------------------------------------
// Creep compliance and relaxation interconversion
// ---------------------------------------------------------------------------

/// Compute creep compliance J(t) from Prony series (approximate).
/// J(t) ~ 1/E_inf * [1 - sum_i (E_i/E_0) * exp(-t/tau_i)]
/// where E_0 = E_inf + sum(E_i)
pub fn prony_creep_compliance(t: f64, e_inf: f64, e_i: &[f64], tau_i: &[f64]) -> f64 {
    let e0 = e_inf + e_i.iter().sum::<f64>();
    if e_inf.abs() < 1e-30 || e0.abs() < 1e-30 {
        return 0.0;
    }
    let mut j = 1.0 / e_inf;
    for (ei, ti) in e_i.iter().zip(tau_i.iter()) {
        let retardation = ei / (e_inf * e0);
        j -= retardation * (-t / ti).exp();
    }
    // Simplified approximation; clamp to positive
    j.max(0.0)
}

/// Compute loss compliance J''(omega) from Prony parameters.
pub fn prony_loss_compliance(omega: f64, e_inf: f64, e_i: &[f64], tau_i: &[f64]) -> f64 {
    let cm = prony_complex_modulus(omega, e_inf, e_i, tau_i);
    let denom = cm.storage * cm.storage + cm.loss * cm.loss;
    if denom < 1e-30 {
        return 0.0;
    }
    cm.loss / denom
}

// ---------------------------------------------------------------------------
// Utility: logarithmic spacing
// ---------------------------------------------------------------------------

/// Generate logarithmically spaced values from 10^start to 10^stop.
pub fn logspace(start: f64, stop: f64, n: usize) -> Vec<f64> {
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return vec![10.0_f64.powf(start)];
    }
    let step = (stop - start) / (n - 1) as f64;
    (0..n).map(|i| 10.0_f64.powf(start + i as f64 * step)).collect()
}

/// Generate a relaxation time spectrum on a log grid.
pub fn relaxation_time_grid(tau_min: f64, tau_max: f64, n: usize) -> Vec<f64> {
    logspace(tau_min.log10(), tau_max.log10(), n)
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
        (a - b).abs() < tol
    }

    // ---- ComplexModulus ----

    #[test]
    fn test_complex_modulus_new() {
        let m = ComplexModulus::new(1e9, 1e8);
        assert_eq!(m.storage, 1e9);
        assert_eq!(m.loss, 1e8);
    }

    #[test]
    fn test_complex_modulus_magnitude() {
        let m = ComplexModulus::new(3.0, 4.0);
        assert!(approx_eq(m.magnitude(), 5.0, TOL));
    }

    #[test]
    fn test_complex_modulus_tan_delta() {
        let m = ComplexModulus::new(1e9, 5e7);
        assert!(approx_eq(m.tan_delta(), 0.05, TOL));
    }

    #[test]
    fn test_complex_modulus_tan_delta_zero_storage() {
        let m = ComplexModulus::new(0.0, 1.0);
        assert!(m.tan_delta().is_infinite());
    }

    #[test]
    fn test_complex_modulus_phase_angle_rad() {
        let m = ComplexModulus::new(1.0, 1.0);
        assert!(approx_eq(m.phase_angle_rad(), PI / 4.0, TOL));
    }

    #[test]
    fn test_complex_modulus_phase_angle_deg() {
        let m = ComplexModulus::new(1.0, 1.0);
        assert!(approx_eq(m.phase_angle_deg(), 45.0, TOL));
    }

    #[test]
    fn test_complex_modulus_to_shear() {
        let e = ComplexModulus::new(3e9, 1.5e8);
        let g = e.to_shear(0.35);
        let factor = 1.0 / (2.0 * 1.35);
        assert!(approx_eq(g.storage, 3e9 * factor, 1.0));
        assert!(approx_eq(g.loss, 1.5e8 * factor, 1.0));
    }

    #[test]
    fn test_complex_modulus_loss_factor() {
        let m = ComplexModulus::new(2.0, 0.5);
        assert!(approx_eq(m.loss_factor(), 0.25, TOL));
    }

    #[test]
    fn test_complex_modulus_pure_elastic() {
        let m = ComplexModulus::new(1e9, 0.0);
        assert!(approx_eq(m.tan_delta(), 0.0, TOL));
        assert!(approx_eq(m.phase_angle_rad(), 0.0, TOL));
    }

    #[test]
    fn test_complex_modulus_pure_viscous() {
        let m = ComplexModulus::new(0.0, 1e8);
        assert!(approx_eq(m.phase_angle_deg(), 90.0, TOL));
    }

    // ---- complex_modulus function ----

    #[test]
    fn test_complex_modulus_fn_zero_delta() {
        let (s, l) = complex_modulus(1e6, 0.01, 0.0);
        assert!(approx_eq(s, 1e8, TOL));
        assert!(approx_eq(l, 0.0, TOL));
    }

    #[test]
    fn test_complex_modulus_fn_45deg() {
        let (s, l) = complex_modulus(1e6, 0.01, PI / 4.0);
        let expected = 1e8 / 2.0_f64.sqrt();
        assert!(approx_eq(s, expected, 1.0));
        assert!(approx_eq(l, expected, 1.0));
    }

    #[test]
    fn test_complex_modulus_struct_fn() {
        let m = complex_modulus_struct(1e6, 0.01, 0.1);
        assert!(m.storage > 0.0);
        assert!(m.loss > 0.0);
        assert!(approx_eq(m.tan_delta(), 0.1_f64.tan(), TOL_LOOSE));
    }

    // ---- ClampGeometry ----

    #[test]
    fn test_sample_dimensions_area() {
        let d = SampleDimensions::new(0.04, 0.01, 0.002);
        assert!(approx_eq(d.cross_section_area(), 0.00002, TOL));
    }

    #[test]
    fn test_sample_dimensions_moment_of_inertia() {
        let d = SampleDimensions::new(0.04, 0.01, 0.002);
        let expected = 0.01 * 0.002_f64.powi(3) / 12.0;
        assert!(approx_eq(d.moment_of_inertia(), expected, 1e-15));
    }

    #[test]
    fn test_geometry_factor_tension() {
        let dim = SampleDimensions::new(0.02, 0.005, 0.001);
        let k = geometry_stiffness_factor(ClampGeometry::Tension, &dim);
        assert!(approx_eq(k, 0.02 / (0.005 * 0.001), TOL));
    }

    #[test]
    fn test_geometry_factor_compression_equals_tension() {
        let dim = SampleDimensions::new(0.02, 0.005, 0.001);
        let kt = geometry_stiffness_factor(ClampGeometry::Tension, &dim);
        let kc = geometry_stiffness_factor(ClampGeometry::Compression, &dim);
        assert!(approx_eq(kt, kc, TOL));
    }

    #[test]
    fn test_geometry_factor_single_cantilever() {
        let dim = SampleDimensions::new(0.02, 0.01, 0.002);
        let k = geometry_stiffness_factor(ClampGeometry::SingleCantilever, &dim);
        assert!(k > 0.0);
    }

    #[test]
    fn test_geometry_factor_dual_cantilever() {
        let dim = SampleDimensions::new(0.04, 0.01, 0.002);
        let k = geometry_stiffness_factor(ClampGeometry::DualCantilever, &dim);
        assert!(k > 0.0);
        // Dual cantilever should give smaller factor than single (stiffer)
        let ks = geometry_stiffness_factor(ClampGeometry::SingleCantilever, &dim);
        assert!(k < ks);
    }

    #[test]
    fn test_geometry_factor_three_point_bend() {
        let dim = SampleDimensions::new(0.04, 0.01, 0.002);
        let k = geometry_stiffness_factor(ClampGeometry::ThreePointBend, &dim);
        assert!(k > 0.0);
    }

    #[test]
    fn test_geometry_factor_shear_sandwich() {
        let dim = SampleDimensions::new(0.01, 0.01, 0.002);
        let k = geometry_stiffness_factor(ClampGeometry::ShearSandwich, &dim);
        assert!(k > 0.0);
    }

    // ---- WLF Equation ----

    #[test]
    fn test_wlf_at_tref() {
        let log_at = wlf_shift_factor(100.0, 100.0, WLF_C1_UNIVERSAL, WLF_C2_UNIVERSAL);
        assert!(approx_eq(log_at, 0.0, TOL));
    }

    #[test]
    fn test_wlf_above_tref() {
        let log_at = wlf_shift_factor(120.0, 100.0, WLF_C1_UNIVERSAL, WLF_C2_UNIVERSAL);
        assert!(log_at < 0.0); // Higher T => lower shift factor (faster relaxation)
    }

    #[test]
    fn test_wlf_below_tref() {
        let log_at = wlf_shift_factor(80.0, 100.0, WLF_C1_UNIVERSAL, WLF_C2_UNIVERSAL);
        assert!(log_at > 0.0); // Lower T => higher shift factor
    }

    #[test]
    fn test_wlf_shift_factor_linear() {
        let at = wlf_shift_factor_linear(100.0, 100.0, WLF_C1_UNIVERSAL, WLF_C2_UNIVERSAL);
        assert!(approx_eq(at, 1.0, TOL));
    }

    #[test]
    fn test_wlf_inverse_roundtrip() {
        let t_orig = 130.0;
        let tref = 100.0;
        let log_at = wlf_shift_factor(t_orig, tref, WLF_C1_UNIVERSAL, WLF_C2_UNIVERSAL);
        let t_back = wlf_inverse_temperature(log_at, tref, WLF_C1_UNIVERSAL, WLF_C2_UNIVERSAL);
        assert!(approx_eq(t_back, t_orig, TOL_LOOSE));
    }

    #[test]
    fn test_wlf_known_value() {
        // T=110, Tref=100, C1=17.44, C2=51.6
        // log(aT) = -17.44 * 10 / (51.6 + 10) = -174.4 / 61.6 = -2.831...
        let log_at = wlf_shift_factor(110.0, 100.0, 17.44, 51.6);
        assert!(approx_eq(log_at, -174.4 / 61.6, TOL_LOOSE));
    }

    #[test]
    fn test_wlf_custom_constants() {
        let log_at = wlf_shift_factor(150.0, 100.0, 8.86, 101.6);
        let expected = -8.86 * 50.0 / (101.6 + 50.0);
        assert!(approx_eq(log_at, expected, TOL));
    }

    // ---- Arrhenius ----

    #[test]
    fn test_arrhenius_at_tref() {
        let ln_at = arrhenius_shift_factor(373.0, 373.0, 200e3);
        assert!(approx_eq(ln_at, 0.0, TOL));
    }

    #[test]
    fn test_arrhenius_higher_temp() {
        let ln_at = arrhenius_shift_factor(400.0, 373.0, 200e3);
        assert!(ln_at < 0.0);
    }

    #[test]
    fn test_activation_energy_from_pair() {
        // Two frequencies with known Tg shift
        let ea = activation_energy_from_tg_pair(1.0, 373.0, 10.0, 383.0);
        assert!(ea > 0.0); // Positive activation energy
    }

    #[test]
    fn test_activation_energy_least_squares_two_points() {
        let pairs = vec![(1.0, 373.0), (10.0, 383.0)];
        let ea_ls = activation_energy_least_squares(&pairs);
        let ea_pair = activation_energy_from_tg_pair(1.0, 373.0, 10.0, 383.0);
        assert!(approx_eq(ea_ls, ea_pair, 1.0));
    }

    #[test]
    fn test_activation_energy_least_squares_multiple() {
        let pairs = vec![
            (0.1, 363.0),
            (1.0, 373.0),
            (10.0, 383.0),
            (100.0, 393.0),
        ];
        let ea = activation_energy_least_squares(&pairs);
        assert!(ea > 0.0);
    }

    #[test]
    fn test_activation_energy_single_point() {
        let ea = activation_energy_least_squares(&[(1.0, 373.0)]);
        assert!(approx_eq(ea, 0.0, TOL));
    }

    // ---- Maxwell Model / Prony Series ----

    #[test]
    fn test_prony_relaxation_at_zero() {
        let val = prony_relaxation(0.0, 1e6, &[1e9, 5e8], &[0.1, 1.0]);
        assert!(approx_eq(val, 1e6 + 1e9 + 5e8, TOL));
    }

    #[test]
    fn test_prony_relaxation_at_infinity() {
        let val = prony_relaxation(1e10, 1e6, &[1e9, 5e8], &[0.1, 1.0]);
        assert!(approx_eq(val, 1e6, 1.0));
    }

    #[test]
    fn test_prony_relaxation_monotone_decreasing() {
        let e_i = &[1e9, 5e8];
        let tau_i = &[0.1, 1.0];
        let v1 = prony_relaxation(0.01, 1e6, e_i, tau_i);
        let v2 = prony_relaxation(0.1, 1e6, e_i, tau_i);
        let v3 = prony_relaxation(1.0, 1e6, e_i, tau_i);
        assert!(v1 > v2);
        assert!(v2 > v3);
    }

    #[test]
    fn test_prony_storage_modulus_limits() {
        let e_i = &[1e9];
        let tau_i = &[1.0];
        let e_inf = 1e6;
        // At very low frequency: E' -> E_inf
        let low = prony_storage_modulus(1e-10, e_inf, e_i, tau_i);
        assert!(approx_eq(low, e_inf, 1.0));
        // At very high frequency: E' -> E_inf + sum(E_i)
        let high = prony_storage_modulus(1e10, e_inf, e_i, tau_i);
        assert!(approx_eq(high, e_inf + 1e9, 1.0));
    }

    #[test]
    fn test_prony_loss_modulus_peak() {
        // Single Maxwell element: loss peak at omega = 1/tau
        let e_i = &[1e9];
        let tau_i = &[1.0];
        let omega_peak = 1.0;
        let at_peak = prony_loss_modulus(omega_peak, e_i, tau_i);
        let below = prony_loss_modulus(omega_peak * 0.1, e_i, tau_i);
        let above = prony_loss_modulus(omega_peak * 10.0, e_i, tau_i);
        assert!(at_peak > below);
        assert!(at_peak > above);
    }

    #[test]
    fn test_prony_complex_modulus_consistency() {
        let e_inf = 1e6;
        let e_i = &[1e9, 5e8];
        let tau_i = &[0.1, 1.0];
        let omega = 10.0;
        let cm = prony_complex_modulus(omega, e_inf, e_i, tau_i);
        let s = prony_storage_modulus(omega, e_inf, e_i, tau_i);
        let l = prony_loss_modulus(omega, e_i, tau_i);
        assert!(approx_eq(cm.storage, s, TOL));
        assert!(approx_eq(cm.loss, l, TOL));
    }

    #[test]
    fn test_maxwell_model_new() {
        let m = MaxwellModel::new(1e6, vec![1e9, 5e8], vec![0.1, 1.0]);
        assert_eq!(m.num_elements(), 2);
    }

    #[test]
    fn test_maxwell_model_glassy_modulus() {
        let m = MaxwellModel::new(1e6, vec![1e9, 5e8], vec![0.1, 1.0]);
        assert!(approx_eq(m.glassy_modulus(), 1e6 + 1e9 + 5e8, TOL));
    }

    #[test]
    fn test_maxwell_model_relaxation() {
        let m = MaxwellModel::new(1e6, vec![1e9], vec![1.0]);
        let r = m.relaxation(0.0);
        assert!(approx_eq(r, 1e6 + 1e9, TOL));
    }

    #[test]
    fn test_maxwell_model_tan_delta() {
        let m = MaxwellModel::new(1e6, vec![1e9], vec![1.0]);
        let td = m.tan_delta(1.0);
        assert!(td > 0.0);
    }

    #[test]
    fn test_maxwell_model_complex_modulus() {
        let m = MaxwellModel::new(1e6, vec![1e9], vec![1.0]);
        let cm = m.complex_modulus(1.0);
        assert!(cm.storage > 0.0);
        assert!(cm.loss > 0.0);
    }

    #[test]
    fn test_maxwell_model_fit_basic() {
        // Generate synthetic data from a known model, then fit
        let true_model = MaxwellModel::new(1e6, vec![1e9], vec![1.0]);
        let freqs = logspace(-2.0, 2.0, 20);
        let omegas: Vec<f64> = freqs.iter().map(|f| 2.0 * PI * f).collect();
        let storage: Vec<f64> = omegas.iter().map(|w| true_model.storage_modulus(*w)).collect();
        let loss: Vec<f64> = omegas.iter().map(|w| true_model.loss_modulus(*w)).collect();

        let tau_grid = logspace(-2.0, 2.0, 5);
        let fitted = MaxwellModel::fit_from_frequency_data(&tau_grid, &omegas, &storage, &loss, 1e6, 500);
        // Just check it doesn't blow up and produces reasonable values
        assert!(fitted.e_inf >= 0.0);
        assert!(fitted.e_i.iter().all(|e| *e >= 0.0));
    }

    // ---- Havriliak-Negami ----

    #[test]
    fn test_hn_debye_at_zero_freq() {
        let hn = HavriliakNegami::debye(2.0, 10.0, 1.0);
        let (re, im) = hn.evaluate(0.0);
        assert!(approx_eq(re, 10.0, TOL));
        assert!(approx_eq(im, 0.0, TOL));
    }

    #[test]
    fn test_hn_debye_at_high_freq() {
        let hn = HavriliakNegami::debye(2.0, 10.0, 1.0);
        let (re, _im) = hn.evaluate(1e10);
        assert!(approx_eq(re, 2.0, TOL_LOOSE));
    }

    #[test]
    fn test_hn_debye_loss_peak() {
        let hn = HavriliakNegami::debye(2.0, 10.0, 1.0);
        // Debye peak at omega = 1/tau
        let at_peak = hn.loss(1.0);
        let below = hn.loss(0.1);
        let above = hn.loss(10.0);
        assert!(at_peak > below);
        assert!(at_peak > above);
    }

    #[test]
    fn test_hn_cole_cole() {
        let hn = HavriliakNegami::cole_cole(2.0, 10.0, 1.0, 0.8);
        assert!(approx_eq(hn.alpha, 0.8, TOL));
        assert!(approx_eq(hn.beta, 1.0, TOL));
        let s = hn.storage(1.0);
        assert!(s > 2.0 && s < 10.0);
    }

    #[test]
    fn test_hn_cole_davidson() {
        let hn = HavriliakNegami::cole_davidson(2.0, 10.0, 1.0, 0.6);
        assert!(approx_eq(hn.alpha, 1.0, TOL));
        assert!(approx_eq(hn.beta, 0.6, TOL));
        let s = hn.storage(1.0);
        assert!(s > 2.0 && s < 10.0);
    }

    #[test]
    fn test_hn_relaxation_strength() {
        let hn = HavriliakNegami::new(2.0, 10.0, 1.0, 0.9, 0.7);
        assert!(approx_eq(hn.relaxation_strength(), 8.0, TOL));
    }

    #[test]
    fn test_hn_tan_delta_positive() {
        let hn = HavriliakNegami::debye(2.0, 10.0, 1.0);
        let td = hn.tan_delta(1.0);
        assert!(td > 0.0);
    }

    #[test]
    fn test_hn_general_between_limits() {
        let hn = HavriliakNegami::new(2.0, 10.0, 1.0, 0.85, 0.55);
        for &f in &[0.01, 0.1, 1.0, 10.0, 100.0] {
            let s = hn.storage(f);
            assert!(s >= 2.0 - 0.1 && s <= 10.0 + 0.1,
                "storage {} at freq {} out of bounds", s, f);
        }
    }

    // ---- Frequency Sweep ----

    #[test]
    fn test_frequency_sweep_add_point() {
        let mut fs = FrequencySweep::new(25.0);
        fs.add_point(1.0, 1e9, 5e7);
        fs.add_point(10.0, 1.1e9, 6e7);
        assert_eq!(fs.points.len(), 2);
    }

    #[test]
    fn test_frequency_sweep_point_tan_delta() {
        let p = FrequencySweepPoint {
            frequency_hz: 1.0,
            storage_modulus: 1e9,
            loss_modulus: 5e7,
        };
        assert!(approx_eq(p.tan_delta(), 0.05, TOL));
    }

    #[test]
    fn test_frequency_sweep_point_magnitude() {
        let p = FrequencySweepPoint {
            frequency_hz: 1.0,
            storage_modulus: 3.0,
            loss_modulus: 4.0,
        };
        assert!(approx_eq(p.complex_modulus_mag(), 5.0, TOL));
    }

    #[test]
    fn test_frequency_sweep_cole_cole() {
        let mut fs = FrequencySweep::new(25.0);
        fs.add_point(1.0, 100.0, 10.0);
        fs.add_point(10.0, 200.0, 20.0);
        let cc = fs.cole_cole_data();
        assert_eq!(cc.len(), 2);
        assert!(approx_eq(cc[0].0, 100.0, TOL));
        assert!(approx_eq(cc[0].1, 10.0, TOL));
    }

    #[test]
    fn test_frequency_sweep_wicket_plot() {
        let mut fs = FrequencySweep::new(25.0);
        fs.add_point(1.0, 1e9, 5e7);
        let wp = fs.wicket_plot_data();
        assert_eq!(wp.len(), 1);
    }

    #[test]
    fn test_frequency_sweep_peak_loss() {
        let mut fs = FrequencySweep::new(25.0);
        fs.add_point(1.0, 1e9, 5e7);
        fs.add_point(10.0, 1e9, 8e7);
        fs.add_point(100.0, 1e9, 3e7);
        assert!(approx_eq(fs.peak_loss_frequency().unwrap(), 10.0, TOL));
    }

    #[test]
    fn test_frequency_sweep_peak_tan_delta() {
        let mut fs = FrequencySweep::new(25.0);
        fs.add_point(1.0, 1e9, 5e7);    // td = 0.05
        fs.add_point(10.0, 5e8, 8e7);   // td = 0.16
        fs.add_point(100.0, 1e9, 3e7);  // td = 0.03
        assert!(approx_eq(fs.peak_tan_delta_frequency().unwrap(), 10.0, TOL));
    }

    // ---- TTS / Master Curve ----

    #[test]
    fn test_tts_shift_at_tref() {
        let mut fs = FrequencySweep::new(100.0);
        fs.add_point(1.0, 1e9, 5e7);
        let shifted = apply_tts_shift(&fs, 100.0, WLF_C1_UNIVERSAL, WLF_C2_UNIVERSAL);
        assert!(approx_eq(shifted[0].frequency_hz, 1.0, TOL));
    }

    #[test]
    fn test_tts_shift_above_tref() {
        let mut fs = FrequencySweep::new(120.0);
        fs.add_point(1.0, 1e9, 5e7);
        let shifted = apply_tts_shift(&fs, 100.0, WLF_C1_UNIVERSAL, WLF_C2_UNIVERSAL);
        // Higher temperature => smaller aT => lower shifted frequency
        assert!(shifted[0].frequency_hz < 1.0);
    }

    #[test]
    fn test_master_curve_construction() {
        let mut s1 = FrequencySweep::new(80.0);
        s1.add_point(1.0, 2e9, 1e8);
        let mut s2 = FrequencySweep::new(100.0);
        s2.add_point(1.0, 1e9, 5e7);
        let mut s3 = FrequencySweep::new(120.0);
        s3.add_point(1.0, 5e8, 2e7);

        let mc = construct_master_curve(&[s1, s2, s3], 100.0, WLF_C1_UNIVERSAL, WLF_C2_UNIVERSAL);
        assert_eq!(mc.len(), 3);
        // Should be sorted by frequency
        assert!(mc[0].frequency_hz <= mc[1].frequency_hz);
        assert!(mc[1].frequency_hz <= mc[2].frequency_hz);
    }

    // ---- Temperature Sweep ----

    #[test]
    fn test_temperature_sweep_tg_tan_delta() {
        let mut ts = TemperatureSweep::new(1.0);
        ts.add_point(50.0, 3e9, 1e7);
        ts.add_point(80.0, 2e9, 5e7);
        ts.add_point(100.0, 5e8, 3e8);  // Highest tan delta = 0.6
        ts.add_point(120.0, 5e8, 1e8);  // tan delta = 0.2
        ts.add_point(150.0, 1e7, 5e5);
        let tg = ts.tg_from_tan_delta_peak().unwrap();
        assert!(approx_eq(tg, 100.0, TOL)); // Peak at 100 C (td=0.6)
    }

    #[test]
    fn test_temperature_sweep_tg_loss_peak() {
        let mut ts = TemperatureSweep::new(1.0);
        ts.add_point(50.0, 3e9, 1e7);
        ts.add_point(80.0, 2e9, 5e8);   // Peak loss
        ts.add_point(100.0, 1e9, 2e8);
        ts.add_point(120.0, 5e8, 1e7);
        let tg = ts.tg_from_loss_peak().unwrap();
        assert!(approx_eq(tg, 80.0, TOL));
    }

    #[test]
    fn test_temperature_sweep_tg_storage_onset() {
        let mut ts = TemperatureSweep::new(1.0);
        ts.add_point(50.0, 3e9, 1e7);
        ts.add_point(80.0, 2.8e9, 5e7);
        ts.add_point(90.0, 1e9, 2e8);   // Steepest drop
        ts.add_point(100.0, 5e8, 1e8);
        ts.add_point(120.0, 1e7, 5e5);
        let tg = ts.tg_from_storage_onset().unwrap();
        assert!(tg > 70.0 && tg < 100.0);
    }

    #[test]
    fn test_temperature_sweep_relaxation_peaks() {
        let mut ts = TemperatureSweep::new(1.0);
        // Create two peaks in tan delta: beta (~-50) and alpha (~100)
        ts.add_point(-100.0, 3e9, 1e6);
        ts.add_point(-50.0, 2.9e9, 5e7);  // beta peak (td high)
        ts.add_point(0.0, 2.8e9, 1e7);
        ts.add_point(50.0, 2.5e9, 2e7);
        ts.add_point(100.0, 1e9, 3e8);    // alpha peak (td high)
        ts.add_point(150.0, 1e7, 1e5);
        let peaks = ts.find_relaxation_peaks();
        assert!(peaks.len() >= 1); // Should find at least the alpha peak
    }

    #[test]
    fn test_temperature_sweep_modulus_drop() {
        let mut ts = TemperatureSweep::new(1.0);
        ts.add_point(50.0, 3e9, 1e7);
        ts.add_point(150.0, 1e7, 5e5);
        let ratio = ts.modulus_drop_ratio();
        assert!(ratio > 100.0); // Large drop across Tg
    }

    #[test]
    fn test_temperature_sweep_point_tan_delta() {
        let p = TemperatureSweepPoint {
            temperature_c: 100.0,
            storage_modulus: 1e9,
            loss_modulus: 1e8,
        };
        assert!(approx_eq(p.tan_delta(), 0.1, TOL));
    }

    #[test]
    fn test_temperature_sweep_empty() {
        let ts = TemperatureSweep::new(1.0);
        assert!(ts.tg_from_tan_delta_peak().is_none());
    }

    // ---- Linear Viscoelastic Region ----

    #[test]
    fn test_lvr_critical_strain() {
        let mut lvr = LinearViscoelasticRegion::new();
        lvr.add_point(0.001, 1e9, 5e7);
        lvr.add_point(0.01, 1e9, 5e7);
        lvr.add_point(0.1, 9.8e8, 5.1e7);
        lvr.add_point(0.5, 8e8, 6e7);
        lvr.add_point(1.0, 5e8, 4e7);
        let crit = lvr.critical_strain(0.05);
        assert!(crit.is_some());
        let c = crit.unwrap();
        assert!(c > 0.01 && c <= 1.0);
    }

    #[test]
    fn test_lvr_no_nonlinearity() {
        let mut lvr = LinearViscoelasticRegion::new();
        lvr.add_point(0.001, 1e9, 5e7);
        lvr.add_point(0.01, 1e9, 5e7);
        lvr.add_point(0.1, 1e9, 5e7);
        let crit = lvr.critical_strain(0.05);
        assert!(crit.is_none());
    }

    #[test]
    fn test_lvr_modulus() {
        let mut lvr = LinearViscoelasticRegion::new();
        lvr.add_point(0.001, 1e9, 5e7);
        lvr.add_point(0.01, 1.01e9, 4.9e7);
        lvr.add_point(0.1, 9e8, 6e7);
        lvr.add_point(1.0, 5e8, 4e7);
        let m = lvr.lvr_modulus().unwrap();
        assert!(m > 9e8 && m < 1.1e9);
    }

    #[test]
    fn test_lvr_is_in_lvr() {
        let mut lvr = LinearViscoelasticRegion::new();
        lvr.add_point(0.001, 1e9, 5e7);
        lvr.add_point(0.01, 1e9, 5e7);
        lvr.add_point(0.1, 8e8, 6e7);
        lvr.add_point(1.0, 5e8, 4e7);
        assert!(lvr.is_in_lvr(0.005, 0.05));
        assert!(!lvr.is_in_lvr(0.5, 0.05));
    }

    #[test]
    fn test_lvr_empty() {
        let lvr = LinearViscoelasticRegion::new();
        assert!(lvr.critical_strain(0.05).is_none());
        assert!(lvr.lvr_modulus().is_none());
    }

    // ---- DMA Processor ----

    #[test]
    fn test_dma_processor_creation() {
        let config = DmaProcessorConfig {
            sample_rate_hz: 1000.0,
            excitation_frequency_hz: 1.0,
            geometry: ClampGeometry::Tension,
            dimensions: SampleDimensions::new(0.02, 0.005, 0.001),
        };
        let proc = DmaProcessor::new(config);
        assert_eq!(proc.samples_per_cycle(), 1000);
    }

    #[test]
    fn test_dma_processor_pure_elastic() {
        let config = DmaProcessorConfig {
            sample_rate_hz: 1000.0,
            excitation_frequency_hz: 1.0,
            geometry: ClampGeometry::Tension,
            dimensions: SampleDimensions::new(0.02, 0.005, 0.001),
        };
        let mut proc = DmaProcessor::new(config);
        let n = 1000;
        // Force and displacement in phase (delta=0 => pure elastic)
        for i in 0..n {
            let t = i as f64 / 1000.0;
            let phase = 2.0 * PI * t;
            let disp = 1e-6 * phase.sin();
            let force = 10.0 * phase.sin(); // In phase
            proc.process_sample(force, disp);
        }
        let m = proc.extract_modulus().unwrap();
        // Should have near-zero loss
        assert!(m.loss.abs() < m.storage.abs() * 0.1);
        assert!(m.storage > 0.0);
    }

    #[test]
    fn test_dma_processor_viscoelastic() {
        let config = DmaProcessorConfig {
            sample_rate_hz: 10000.0,
            excitation_frequency_hz: 1.0,
            geometry: ClampGeometry::Tension,
            dimensions: SampleDimensions::new(0.02, 0.005, 0.001),
        };
        let mut proc = DmaProcessor::new(config);
        let n = 10000;
        let delta = 0.3; // 0.3 rad phase lag
        for i in 0..n {
            let t = i as f64 / 10000.0;
            let phase = 2.0 * PI * t;
            let disp = 1e-6 * phase.sin();
            let force = 10.0 * (phase + delta).sin();
            proc.process_sample(force, disp);
        }
        let m = proc.extract_modulus().unwrap();
        assert!(m.storage > 0.0);
        assert!(m.loss > 0.0);
        // tan(delta) should be close to tan(0.3)
        let expected_td = delta.tan();
        assert!(approx_eq(m.tan_delta(), expected_td, 0.05));
    }

    #[test]
    fn test_dma_processor_reset() {
        let config = DmaProcessorConfig {
            sample_rate_hz: 1000.0,
            excitation_frequency_hz: 1.0,
            geometry: ClampGeometry::Tension,
            dimensions: SampleDimensions::new(0.02, 0.005, 0.001),
        };
        let mut proc = DmaProcessor::new(config);
        proc.process_sample(1.0, 1e-6);
        proc.reset();
        assert!(proc.extract_modulus().is_none());
    }

    #[test]
    fn test_dma_processor_batch() {
        let config = DmaProcessorConfig {
            sample_rate_hz: 1000.0,
            excitation_frequency_hz: 1.0,
            geometry: ClampGeometry::ThreePointBend,
            dimensions: SampleDimensions::new(0.04, 0.01, 0.002),
        };
        let mut proc = DmaProcessor::new(config);
        let force: Vec<f64> = (0..1000).map(|i| {
            let t = i as f64 / 1000.0;
            10.0 * (2.0 * PI * t).sin()
        }).collect();
        let disp: Vec<f64> = (0..1000).map(|i| {
            let t = i as f64 / 1000.0;
            1e-6 * (2.0 * PI * t).sin()
        }).collect();
        proc.process_batch(&force, &disp);
        let m = proc.extract_modulus();
        assert!(m.is_some());
    }

    #[test]
    fn test_dma_processor_geometry_factor() {
        let config = DmaProcessorConfig {
            sample_rate_hz: 1000.0,
            excitation_frequency_hz: 1.0,
            geometry: ClampGeometry::Tension,
            dimensions: SampleDimensions::new(0.02, 0.005, 0.001),
        };
        let proc = DmaProcessor::new(config);
        assert!(proc.geometry_factor() > 0.0);
    }

    // ---- Creep / Compliance ----

    #[test]
    fn test_prony_creep_compliance_at_zero() {
        let j0 = prony_creep_compliance(0.0, 1e6, &[1e9], &[1.0]);
        assert!(j0 > 0.0);
    }

    #[test]
    fn test_prony_creep_compliance_increases() {
        let j1 = prony_creep_compliance(0.1, 1e6, &[1e9], &[1.0]);
        let j2 = prony_creep_compliance(1.0, 1e6, &[1e9], &[1.0]);
        let j3 = prony_creep_compliance(10.0, 1e6, &[1e9], &[1.0]);
        assert!(j2 >= j1);
        assert!(j3 >= j2);
    }

    #[test]
    fn test_prony_loss_compliance() {
        let jl = prony_loss_compliance(1.0, 1e6, &[1e9], &[1.0]);
        assert!(jl > 0.0);
    }

    // ---- Utility ----

    #[test]
    fn test_logspace() {
        let v = logspace(0.0, 3.0, 4);
        assert_eq!(v.len(), 4);
        assert!(approx_eq(v[0], 1.0, TOL));
        assert!(approx_eq(v[1], 10.0, TOL));
        assert!(approx_eq(v[2], 100.0, TOL));
        assert!(approx_eq(v[3], 1000.0, TOL));
    }

    #[test]
    fn test_logspace_single() {
        let v = logspace(2.0, 2.0, 1);
        assert_eq!(v.len(), 1);
        assert!(approx_eq(v[0], 100.0, TOL));
    }

    #[test]
    fn test_logspace_empty() {
        let v = logspace(0.0, 3.0, 0);
        assert!(v.is_empty());
    }

    #[test]
    fn test_relaxation_time_grid() {
        let grid = relaxation_time_grid(1e-3, 1e3, 7);
        assert_eq!(grid.len(), 7);
        assert!(approx_eq(grid[0], 1e-3, 1e-6));
        assert!(approx_eq(grid[6], 1e3, TOL_LOOSE));
    }
}
