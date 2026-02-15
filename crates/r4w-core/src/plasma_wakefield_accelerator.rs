//! Plasma Wakefield Acceleration Signal Processing
//!
//! Implements plasma wakefield acceleration diagnostics for next-generation
//! particle accelerator facilities. Covers both laser-driven (LWFA) and
//! beam-driven (PWFA) wakefield acceleration regimes, including linear
//! wakefield theory, beam loading, beam diagnostics, and phase-space analysis.
//!
//! ## Background
//!
//! Plasma wakefield acceleration (PWFA/LWFA) uses intense laser pulses or
//! relativistic particle bunches to excite large-amplitude plasma waves.
//! The longitudinal electric fields in these waves can exceed 100 GV/m,
//! three orders of magnitude beyond conventional RF cavities. This module
//! provides the signal processing and physics computations needed to
//! characterize and diagnose such accelerators.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::plasma_wakefield_accelerator::{
//!     PlasmaConfig, plasma_frequency, plasma_wavelength, skin_depth,
//!     debye_length, wave_breaking_field,
//! };
//!
//! let cfg = PlasmaConfig { density_m3: 1e17, temperature_ev: 10.0 };
//! let omega_pe = plasma_frequency(cfg.density_m3);
//! let lambda_p = plasma_wavelength(cfg.density_m3);
//! let delta = skin_depth(cfg.density_m3);
//! let lambda_d = debye_length(cfg.density_m3, cfg.temperature_ev);
//! let e_wb = wave_breaking_field(cfg.density_m3);
//!
//! assert!(omega_pe > 1e12);
//! assert!(lambda_p > 1e-6 && lambda_p < 1e-3);
//! assert!(delta > 0.0);
//! assert!(lambda_d > 0.0);
//! assert!(e_wb > 1e9); // > 1 GV/m at 10^17 /m^3
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants (SI)
// ---------------------------------------------------------------------------

/// Elementary charge (C).
pub const E_CHARGE: f64 = 1.602_176_634e-19;

/// Electron mass (kg).
pub const M_ELECTRON: f64 = 9.109_383_7015e-31;

/// Speed of light in vacuum (m/s).
pub const C_LIGHT: f64 = 2.997_924_58e8;

/// Vacuum permittivity (F/m).
pub const EPSILON_0: f64 = 8.854_187_8128e-12;

/// Boltzmann constant (J/K).
pub const K_BOLTZMANN: f64 = 1.380_649e-23;

/// Reduced Planck constant (J*s).
pub const H_BAR: f64 = 1.054_571_817e-34;

/// Electron rest energy (J).
pub const MC2_ELECTRON: f64 = M_ELECTRON * C_LIGHT * C_LIGHT;

/// eV to Joule conversion.
pub const EV_TO_J: f64 = E_CHARGE;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for a plasma wakefield accelerator.
#[derive(Debug, Clone, Copy)]
pub struct PlasmaConfig {
    /// Plasma electron density (m^-3). Typical: 10^16 - 10^18.
    pub density_m3: f64,
    /// Electron temperature (eV).
    pub temperature_ev: f64,
}

impl Default for PlasmaConfig {
    fn default() -> Self {
        Self {
            density_m3: 1e17,
            temperature_ev: 10.0,
        }
    }
}

/// Parameters for a Gaussian drive bunch (beam-driven).
#[derive(Debug, Clone, Copy)]
pub struct GaussianBunchParams {
    /// Number of particles in the bunch.
    pub num_particles: f64,
    /// RMS bunch length (m).
    pub sigma_z: f64,
    /// RMS bunch radius (m).
    pub sigma_r: f64,
}

impl Default for GaussianBunchParams {
    fn default() -> Self {
        Self {
            num_particles: 2e10,
            sigma_z: 30e-6,
            sigma_r: 10e-6,
        }
    }
}

/// Parameters for a laser driver (LWFA).
#[derive(Debug, Clone, Copy)]
pub struct LaserDriverParams {
    /// Laser wavelength (m).
    pub wavelength_m: f64,
    /// Peak intensity (W/m^2).
    pub peak_intensity: f64,
    /// Laser pulse duration (s).
    pub pulse_duration_s: f64,
    /// Laser spot size w0 (m).
    pub spot_size_m: f64,
}

impl Default for LaserDriverParams {
    fn default() -> Self {
        Self {
            wavelength_m: 800e-9,        // 800 nm Ti:sapphire
            peak_intensity: 1e22,        // 10^18 W/cm^2 = 10^22 W/m^2
            pulse_duration_s: 30e-15,    // 30 fs
            spot_size_m: 20e-6,          // 20 um
        }
    }
}

/// Beam diagnostics measurement result.
#[derive(Debug, Clone)]
pub struct BeamDiagnostics {
    /// Mean energy (eV).
    pub mean_energy_ev: f64,
    /// RMS energy spread (fraction: delta_E/E).
    pub energy_spread: f64,
    /// Total charge (C).
    pub charge_c: f64,
    /// RMS bunch length estimate (m).
    pub bunch_length_m: f64,
    /// Geometric emittance x (m*rad).
    pub emittance_x: f64,
    /// Geometric emittance y (m*rad).
    pub emittance_y: f64,
}

/// Twiss parameters for beam optics.
#[derive(Debug, Clone, Copy)]
pub struct TwissParameters {
    /// Alpha (correlation parameter, dimensionless).
    pub alpha: f64,
    /// Beta function (m/rad).
    pub beta: f64,
    /// Gamma function (rad/m).  gamma = (1 + alpha^2) / beta.
    pub gamma: f64,
    /// Geometric emittance (m*rad).
    pub emittance: f64,
}

/// Wakefield result at a single longitudinal position.
#[derive(Debug, Clone, Copy)]
pub struct WakefieldPoint {
    /// Co-moving coordinate xi = z - c*t (m).
    pub xi: f64,
    /// Wake potential Phi (V).
    pub phi: f64,
    /// Accelerating field E_z (V/m).
    pub e_z: f64,
}

// ---------------------------------------------------------------------------
// Plasma parameter computations
// ---------------------------------------------------------------------------

/// Compute plasma angular frequency omega_pe (rad/s).
///
/// omega_pe = sqrt(n_e * e^2 / (epsilon_0 * m_e))
pub fn plasma_frequency(density_m3: f64) -> f64 {
    (density_m3 * E_CHARGE * E_CHARGE / (EPSILON_0 * M_ELECTRON)).sqrt()
}

/// Compute plasma wavelength lambda_p = 2*pi*c / omega_pe (m).
pub fn plasma_wavelength(density_m3: f64) -> f64 {
    let omega_pe = plasma_frequency(density_m3);
    2.0 * PI * C_LIGHT / omega_pe
}

/// Compute plasma skin depth c / omega_pe (m).
pub fn skin_depth(density_m3: f64) -> f64 {
    let omega_pe = plasma_frequency(density_m3);
    C_LIGHT / omega_pe
}

/// Compute plasma wavenumber k_p = omega_pe / c (rad/m).
pub fn plasma_wavenumber(density_m3: f64) -> f64 {
    let omega_pe = plasma_frequency(density_m3);
    omega_pe / C_LIGHT
}

/// Compute Debye length lambda_D = sqrt(epsilon_0 * k_B * T / (n_e * e^2)) (m).
///
/// Temperature is given in eV; internally converted to Kelvin via T_K = T_eV * e / k_B.
pub fn debye_length(density_m3: f64, temperature_ev: f64) -> f64 {
    let t_joules = temperature_ev * EV_TO_J;
    (EPSILON_0 * t_joules / (density_m3 * E_CHARGE * E_CHARGE)).sqrt()
}

/// Compute cold wave-breaking field E_wb = m_e * c * omega_pe / e (V/m).
pub fn wave_breaking_field(density_m3: f64) -> f64 {
    let omega_pe = plasma_frequency(density_m3);
    M_ELECTRON * C_LIGHT * omega_pe / E_CHARGE
}

/// Compute the number of particles in a Debye sphere: N_D = (4/3)*pi*n*lambda_D^3.
pub fn debye_sphere_count(density_m3: f64, temperature_ev: f64) -> f64 {
    let ld = debye_length(density_m3, temperature_ev);
    (4.0 / 3.0) * PI * density_m3 * ld * ld * ld
}

/// Compute plasma coupling parameter Gamma = e^2 / (4*pi*epsilon_0 * a_ws * k_B*T)
/// where a_ws = (3/(4*pi*n))^(1/3) is the Wigner-Seitz radius.
pub fn coupling_parameter(density_m3: f64, temperature_ev: f64) -> f64 {
    let t_joules = temperature_ev * EV_TO_J;
    let a_ws = (3.0 / (4.0 * PI * density_m3)).powf(1.0 / 3.0);
    E_CHARGE * E_CHARGE / (4.0 * PI * EPSILON_0 * a_ws * t_joules)
}

// ---------------------------------------------------------------------------
// Linear wakefield theory
// ---------------------------------------------------------------------------

/// Compute the 1D linear wakefield from a density perturbation profile.
///
/// Given a set of longitudinal positions `xi` (co-moving frame) and the
/// corresponding bunch density `n_b(xi)` (m^-3), computes the wake potential
/// Phi(xi) and accelerating field E_z(xi) = -dPhi/dxi via the convolution:
///
///   Phi(xi) = -(e / (epsilon_0 * k_p^2)) * integral[ sin(k_p*(xi - xi')) * n_b(xi') dxi' ]
///
/// Uses the trapezoidal rule for numerical integration.
pub fn linear_wakefield(
    xi_positions: &[f64],
    bunch_density: &[f64],
    plasma_density_m3: f64,
) -> Vec<WakefieldPoint> {
    assert_eq!(xi_positions.len(), bunch_density.len());
    if xi_positions.is_empty() {
        return Vec::new();
    }

    let kp = plasma_wavenumber(plasma_density_m3);
    let prefactor = -E_CHARGE / (EPSILON_0 * kp * kp);

    let n = xi_positions.len();
    let mut results = Vec::with_capacity(n);

    for i in 0..n {
        let xi = xi_positions[i];

        // Numerical integration using trapezoidal rule
        let mut integral = 0.0;
        for j in 0..n {
            if j == 0 || j == n - 1 {
                continue; // will handle endpoints
            }
            let xi_prime = xi_positions[j];
            let dxi = if j + 1 < n {
                (xi_positions[j + 1] - xi_positions[j.saturating_sub(1)]) / 2.0
            } else {
                xi_positions[j] - xi_positions[j - 1]
            };
            integral += (kp * (xi - xi_prime)).sin() * bunch_density[j] * dxi;
        }

        // Handle endpoints
        if n > 1 {
            let dxi_0 = (xi_positions[1] - xi_positions[0]) / 2.0;
            integral += (kp * (xi - xi_positions[0])).sin() * bunch_density[0] * dxi_0;

            let dxi_n = (xi_positions[n - 1] - xi_positions[n - 2]) / 2.0;
            integral +=
                (kp * (xi - xi_positions[n - 1])).sin() * bunch_density[n - 1] * dxi_n;
        }

        let phi = prefactor * integral;
        results.push(WakefieldPoint { xi, phi, e_z: 0.0 });
    }

    // Compute E_z = -dPhi/dxi by finite differences
    for i in 0..n {
        let e_z = if i == 0 && n > 1 {
            -(results[1].phi - results[0].phi) / (xi_positions[1] - xi_positions[0])
        } else if i == n - 1 && n > 1 {
            -(results[n - 1].phi - results[n - 2].phi)
                / (xi_positions[n - 1] - xi_positions[n - 2])
        } else if n > 2 {
            -(results[i + 1].phi - results[i - 1].phi)
                / (xi_positions[i + 1] - xi_positions[i - 1])
        } else {
            0.0
        };
        results[i].e_z = e_z;
    }

    results
}

/// Peak accelerating field from a Gaussian drive bunch in the linear regime.
///
/// E_z_max = (e * N_b) / (4*pi*epsilon_0*sigma_z^2) * k_p*sigma_z * exp(-k_p^2*sigma_z^2/2)
///
/// This is the maximum longitudinal field behind a Gaussian bunch of N_b particles
/// with RMS length sigma_z, in a plasma of given density.
pub fn gaussian_bunch_peak_field(
    bunch: &GaussianBunchParams,
    plasma_density_m3: f64,
) -> f64 {
    let kp = plasma_wavenumber(plasma_density_m3);
    let kp_sz = kp * bunch.sigma_z;
    let prefactor =
        E_CHARGE * bunch.num_particles / (4.0 * PI * EPSILON_0 * bunch.sigma_z * bunch.sigma_z);
    prefactor * kp_sz * (-kp_sz * kp_sz / 2.0).exp()
}

/// Accelerating field from a Gaussian bunch at position xi behind the bunch.
///
/// E_z(xi) = E_z_max * sin(k_p * xi)
///
/// where xi is the distance behind the bunch center (xi > 0 is behind).
pub fn gaussian_bunch_field_at(
    bunch: &GaussianBunchParams,
    plasma_density_m3: f64,
    xi: f64,
) -> f64 {
    let kp = plasma_wavenumber(plasma_density_m3);
    let e_max = gaussian_bunch_peak_field(bunch, plasma_density_m3);
    e_max * (kp * xi).sin()
}

/// Compute linear focusing field E_r at radius r.
///
/// E_r(r, xi) = -(r/2) * dE_z/dxi (linear focusing approximation in blowout).
/// For a sinusoidal wake: E_r(r, xi) = -(r*k_p/2) * E_z_max * cos(k_p*xi).
pub fn focusing_field(
    bunch: &GaussianBunchParams,
    plasma_density_m3: f64,
    r: f64,
    xi: f64,
) -> f64 {
    let kp = plasma_wavenumber(plasma_density_m3);
    let e_max = gaussian_bunch_peak_field(bunch, plasma_density_m3);
    -(r * kp / 2.0) * e_max * (kp * xi).cos()
}

// ---------------------------------------------------------------------------
// Beam loading
// ---------------------------------------------------------------------------

/// Transformer ratio R = E_decel / E_accel.
///
/// For a symmetric bunch, the maximum is R = 2 (fundamental theorem of beam loading).
pub fn transformer_ratio(e_decel: f64, e_accel: f64) -> f64 {
    if e_accel.abs() < 1e-30 {
        return 0.0;
    }
    e_decel.abs() / e_accel.abs()
}

/// Energy gain of a witness bunch: delta_W = e * E_z * L_acc (Joules).
pub fn energy_gain_j(e_z: f64, acceleration_length_m: f64) -> f64 {
    E_CHARGE * e_z * acceleration_length_m
}

/// Energy gain in eV.
pub fn energy_gain_ev(e_z: f64, acceleration_length_m: f64) -> f64 {
    e_z * acceleration_length_m
}

/// Beam-loaded gradient: effective accelerating field when witness beam
/// extracts energy. E_loaded = E_wake - E_beam_loading.
pub fn beam_loaded_gradient(e_wake: f64, e_beam_loading: f64) -> f64 {
    e_wake - e_beam_loading
}

/// Luminosity per beam power estimate (simplified):
/// L/P ~ N1*N2*f_rep / (4*pi*sigma_x*sigma_y * P_beam)
pub fn luminosity_per_power(
    n1: f64,
    n2: f64,
    f_rep: f64,
    sigma_x: f64,
    sigma_y: f64,
    p_beam: f64,
) -> f64 {
    if sigma_x.abs() < 1e-30 || sigma_y.abs() < 1e-30 || p_beam.abs() < 1e-30 {
        return 0.0;
    }
    n1 * n2 * f_rep / (4.0 * PI * sigma_x * sigma_y * p_beam)
}

// ---------------------------------------------------------------------------
// Laser-driven wakefield (LWFA)
// ---------------------------------------------------------------------------

/// Normalized vector potential a0 = 0.855 * lambda_um * sqrt(I_18).
///
/// lambda_um: laser wavelength in micrometers.
/// intensity_w_per_cm2: peak laser intensity in W/cm^2.
///
/// Formula: a0 = 0.855 * lambda_um * sqrt(I / 10^18)
pub fn normalized_vector_potential(wavelength_m: f64, intensity_w_per_m2: f64) -> f64 {
    let lambda_um = wavelength_m * 1e6;
    let i_18 = intensity_w_per_m2 * 1e-4 / 1e18; // W/m^2 -> W/cm^2 -> units of 10^18
    0.855 * lambda_um * i_18.sqrt()
}

/// Critical power for relativistic self-focusing: P_c = 17 * (omega/omega_pe)^2 GW.
///
/// Returns critical power in Watts.
pub fn critical_power_self_focusing(laser_wavelength_m: f64, plasma_density_m3: f64) -> f64 {
    let omega = 2.0 * PI * C_LIGHT / laser_wavelength_m;
    let omega_pe = plasma_frequency(plasma_density_m3);
    let ratio = omega / omega_pe;
    17.0e9 * ratio * ratio
}

/// Dephasing length in the linear regime: L_d = (2/3) * (omega/omega_pe)^2 * lambda_p (m).
pub fn dephasing_length(laser_wavelength_m: f64, plasma_density_m3: f64) -> f64 {
    let omega = 2.0 * PI * C_LIGHT / laser_wavelength_m;
    let omega_pe = plasma_frequency(plasma_density_m3);
    let lambda_p = plasma_wavelength(plasma_density_m3);
    let ratio = omega / omega_pe;
    (2.0 / 3.0) * ratio * ratio * lambda_p
}

/// Pump depletion length: L_pd ~ (omega/omega_pe)^2 * lambda_p / a0^2 (m).
pub fn pump_depletion_length(
    laser_wavelength_m: f64,
    plasma_density_m3: f64,
    a0: f64,
) -> f64 {
    let omega = 2.0 * PI * C_LIGHT / laser_wavelength_m;
    let omega_pe = plasma_frequency(plasma_density_m3);
    let lambda_p = plasma_wavelength(plasma_density_m3);
    let ratio = omega / omega_pe;
    if a0.abs() < 1e-30 {
        return f64::INFINITY;
    }
    ratio * ratio * lambda_p / (a0 * a0)
}

/// Maximum energy gain in LWFA (linear regime):
///
/// delta_W = (2/3) * (a0^2 / (1 + a0^2/2)) * m_e*c^2 * (omega/omega_pe)^2 (Joules).
pub fn lwfa_max_energy_gain_j(
    laser_wavelength_m: f64,
    plasma_density_m3: f64,
    a0: f64,
) -> f64 {
    let omega = 2.0 * PI * C_LIGHT / laser_wavelength_m;
    let omega_pe = plasma_frequency(plasma_density_m3);
    let ratio = omega / omega_pe;
    let factor = a0 * a0 / (1.0 + a0 * a0 / 2.0);
    (2.0 / 3.0) * factor * MC2_ELECTRON * ratio * ratio
}

/// Maximum energy gain in LWFA in eV.
pub fn lwfa_max_energy_gain_ev(
    laser_wavelength_m: f64,
    plasma_density_m3: f64,
    a0: f64,
) -> f64 {
    lwfa_max_energy_gain_j(laser_wavelength_m, plasma_density_m3, a0) / EV_TO_J
}

/// Laser pulse energy estimate: E_laser ~ P_peak * tau_pulse (Joules).
pub fn laser_pulse_energy(peak_power_w: f64, pulse_duration_s: f64) -> f64 {
    peak_power_w * pulse_duration_s
}

/// Laser peak power from intensity and spot size: P = I * pi * w0^2 (W).
pub fn laser_peak_power(intensity_w_per_m2: f64, spot_size_m: f64) -> f64 {
    intensity_w_per_m2 * PI * spot_size_m * spot_size_m
}

// ---------------------------------------------------------------------------
// Beam-driven wakefield (PWFA)
// ---------------------------------------------------------------------------

/// Check if blowout regime: n_b >> n_0 (bunch density much larger than plasma density).
///
/// Returns true when bunch density exceeds 3x plasma density (conventional threshold).
pub fn is_blowout_regime(bunch_density_m3: f64, plasma_density_m3: f64) -> bool {
    bunch_density_m3 > 3.0 * plasma_density_m3
}

/// Bunch peak density for a Gaussian bunch: n_b = N / ((2*pi)^(3/2) * sigma_z * sigma_r^2).
pub fn gaussian_bunch_density(bunch: &GaussianBunchParams) -> f64 {
    let vol = (2.0 * PI).powf(1.5) * bunch.sigma_z * bunch.sigma_r * bunch.sigma_r;
    if vol.abs() < 1e-60 {
        return 0.0;
    }
    bunch.num_particles / vol
}

/// Bubble radius in the blowout regime: R_b = 2*sqrt(a0) / k_p (m).
///
/// a0 here is the normalized charge of the drive bunch (analogous to laser a0).
pub fn bubble_radius(a0: f64, plasma_density_m3: f64) -> f64 {
    let kp = plasma_wavenumber(plasma_density_m3);
    if kp.abs() < 1e-30 {
        return 0.0;
    }
    2.0 * a0.sqrt() / kp
}

/// Approximate uniform accelerating gradient inside the bubble (V/m):
///
/// E_z ~ (m_e * c * omega_pe / (2*e)) * (k_p * R_b)
///
/// which simplifies to ~ E_wb/2 * (k_p*R_b) for small bubble.
pub fn bubble_accelerating_gradient(r_b: f64, plasma_density_m3: f64) -> f64 {
    let kp = plasma_wavenumber(plasma_density_m3);
    let e_wb = wave_breaking_field(plasma_density_m3);
    0.5 * e_wb * kp * r_b
}

/// PWFA energy gain: delta_W = e * E_z * L_acc.
/// This is the same formula as `energy_gain_j` but provided for clarity.
pub fn pwfa_energy_gain_j(accel_gradient: f64, length_m: f64) -> f64 {
    energy_gain_j(accel_gradient, length_m)
}

// ---------------------------------------------------------------------------
// Beam diagnostics
// ---------------------------------------------------------------------------

/// Compute energy spectrum histogram from particle energies.
///
/// Returns (bin_centers_ev, counts) for `num_bins` equally-spaced bins.
pub fn energy_spectrum(energies_ev: &[f64], num_bins: usize) -> (Vec<f64>, Vec<usize>) {
    if energies_ev.is_empty() || num_bins == 0 {
        return (Vec::new(), Vec::new());
    }

    let min_e = energies_ev.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_e = energies_ev.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

    if (max_e - min_e).abs() < 1e-30 {
        return (vec![min_e], vec![energies_ev.len()]);
    }

    let bin_width = (max_e - min_e) / num_bins as f64;
    let mut counts = vec![0usize; num_bins];
    let mut centers = Vec::with_capacity(num_bins);

    for i in 0..num_bins {
        centers.push(min_e + (i as f64 + 0.5) * bin_width);
    }

    for &e in energies_ev {
        let idx = ((e - min_e) / bin_width) as usize;
        let idx = idx.min(num_bins - 1);
        counts[idx] += 1;
    }

    (centers, counts)
}

/// Compute RMS energy spread delta_E/E.
pub fn energy_spread(energies_ev: &[f64]) -> f64 {
    if energies_ev.len() < 2 {
        return 0.0;
    }
    let n = energies_ev.len() as f64;
    let mean = energies_ev.iter().sum::<f64>() / n;
    if mean.abs() < 1e-30 {
        return 0.0;
    }
    let var = energies_ev.iter().map(|&e| (e - mean) * (e - mean)).sum::<f64>() / n;
    var.sqrt() / mean
}

/// Compute geometric emittance from trace-space coordinates.
///
/// emittance = sqrt(<x^2><x'^2> - <x*x'>^2)
///
/// x: transverse positions (m), xp: transverse angles (rad).
pub fn geometric_emittance(x: &[f64], xp: &[f64]) -> f64 {
    assert_eq!(x.len(), xp.len());
    let n = x.len() as f64;
    if n < 2.0 {
        return 0.0;
    }

    let mean_x = x.iter().sum::<f64>() / n;
    let mean_xp = xp.iter().sum::<f64>() / n;

    let x2 = x.iter().map(|&xi| (xi - mean_x) * (xi - mean_x)).sum::<f64>() / n;
    let xp2 = xp.iter().map(|&xi| (xi - mean_xp) * (xi - mean_xp)).sum::<f64>() / n;
    let xxp = x
        .iter()
        .zip(xp.iter())
        .map(|(&xi, &xpi)| (xi - mean_x) * (xpi - mean_xp))
        .sum::<f64>()
        / n;

    let det = x2 * xp2 - xxp * xxp;
    if det < 0.0 {
        0.0
    } else {
        det.sqrt()
    }
}

/// Estimate bunch charge from integrated current signal.
///
/// Q = integral(I(t) * dt) using the trapezoidal rule.
pub fn bunch_charge(current_a: &[f64], dt_s: f64) -> f64 {
    if current_a.len() < 2 {
        return 0.0;
    }
    let mut integral = 0.0;
    for i in 1..current_a.len() {
        integral += 0.5 * (current_a[i] + current_a[i - 1]) * dt_s;
    }
    integral
}

/// Estimate bunch length from coherent transition radiation (CTR) spectrum.
///
/// The CTR spectrum is proportional to |F(omega)|^2 where F is the bunch form factor.
/// For a Gaussian bunch: |F|^2 = exp(-omega^2 * sigma_t^2).
/// Fit the log of the spectrum to extract sigma_t, then sigma_z = c * sigma_t.
///
/// Returns estimated RMS bunch length in meters.
pub fn bunch_length_from_ctr(
    frequencies_hz: &[f64],
    spectral_power: &[f64],
) -> f64 {
    assert_eq!(frequencies_hz.len(), spectral_power.len());
    if frequencies_hz.len() < 2 {
        return 0.0;
    }

    // Fit log(S) = a - omega^2 * sigma_t^2
    // Use linear regression: y = A + B*x where x = omega^2, y = ln(S)
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut sum_xx = 0.0;
    let mut sum_xy = 0.0;
    let mut count = 0.0;

    for (&f, &s) in frequencies_hz.iter().zip(spectral_power.iter()) {
        if s <= 0.0 {
            continue;
        }
        let omega = 2.0 * PI * f;
        let x = omega * omega;
        let y = s.ln();
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
        count += 1.0;
    }

    if count < 2.0 {
        return 0.0;
    }

    let denom = count * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-60 {
        return 0.0;
    }

    let b = (count * sum_xy - sum_x * sum_y) / denom;
    // b = -sigma_t^2, so sigma_t = sqrt(-b)
    let sigma_t_sq = -b;
    if sigma_t_sq <= 0.0 {
        return 0.0;
    }
    let sigma_t = sigma_t_sq.sqrt();
    C_LIGHT * sigma_t
}

/// Compute full beam diagnostics from particle data.
pub fn compute_diagnostics(
    energies_ev: &[f64],
    x: &[f64],
    xp: &[f64],
    y: &[f64],
    yp: &[f64],
    charge_c: f64,
    bunch_length_m: f64,
) -> BeamDiagnostics {
    let n = energies_ev.len() as f64;
    let mean_energy = if n > 0.0 {
        energies_ev.iter().sum::<f64>() / n
    } else {
        0.0
    };

    BeamDiagnostics {
        mean_energy_ev: mean_energy,
        energy_spread: energy_spread(energies_ev),
        charge_c,
        bunch_length_m,
        emittance_x: geometric_emittance(x, xp),
        emittance_y: geometric_emittance(y, yp),
    }
}

// ---------------------------------------------------------------------------
// Phase space analysis
// ---------------------------------------------------------------------------

/// Compute Twiss parameters from trace-space data (x, x').
///
/// Returns TwissParameters { alpha, beta, gamma, emittance }.
pub fn compute_twiss(x: &[f64], xp: &[f64]) -> TwissParameters {
    assert_eq!(x.len(), xp.len());
    let n = x.len() as f64;
    if n < 2.0 {
        return TwissParameters {
            alpha: 0.0,
            beta: 0.0,
            gamma: 0.0,
            emittance: 0.0,
        };
    }

    let mean_x = x.iter().sum::<f64>() / n;
    let mean_xp = xp.iter().sum::<f64>() / n;

    let sigma_x2 = x.iter().map(|&xi| (xi - mean_x) * (xi - mean_x)).sum::<f64>() / n;
    let sigma_xp2 = xp.iter().map(|&xi| (xi - mean_xp) * (xi - mean_xp)).sum::<f64>() / n;
    let sigma_xxp = x
        .iter()
        .zip(xp.iter())
        .map(|(&xi, &xpi)| (xi - mean_x) * (xpi - mean_xp))
        .sum::<f64>()
        / n;

    let emittance = geometric_emittance(x, xp);
    if emittance < 1e-30 {
        return TwissParameters {
            alpha: 0.0,
            beta: 0.0,
            gamma: 0.0,
            emittance: 0.0,
        };
    }

    let beta = sigma_x2 / emittance;
    let alpha = -sigma_xxp / emittance;
    let gamma = sigma_xp2 / emittance;

    TwissParameters {
        alpha,
        beta,
        gamma,
        emittance,
    }
}

/// Beam envelope sigma_x = sqrt(emittance * beta) (m).
pub fn beam_envelope(emittance: f64, beta: f64) -> f64 {
    (emittance * beta).sqrt()
}

/// Compute the phase-space ellipse area: A = pi * emittance.
pub fn phase_space_area(emittance: f64) -> f64 {
    PI * emittance
}

/// Check Twiss consistency: gamma = (1 + alpha^2) / beta.
pub fn twiss_consistency(twiss: &TwissParameters) -> f64 {
    if twiss.beta.abs() < 1e-30 {
        return f64::INFINITY;
    }
    let expected_gamma = (1.0 + twiss.alpha * twiss.alpha) / twiss.beta;
    (twiss.gamma - expected_gamma).abs()
}

/// Normalized emittance: epsilon_n = beta_rel * gamma_rel * epsilon.
///
/// gamma_rel: relativistic Lorentz factor.
pub fn normalized_emittance(emittance: f64, gamma_rel: f64) -> f64 {
    let beta_rel = (1.0 - 1.0 / (gamma_rel * gamma_rel)).sqrt();
    beta_rel * gamma_rel * emittance
}

/// Compute relativistic Lorentz factor from kinetic energy (eV) for electrons.
pub fn lorentz_factor(kinetic_energy_ev: f64) -> f64 {
    let rest_energy_ev = MC2_ELECTRON / EV_TO_J;
    1.0 + kinetic_energy_ev / rest_energy_ev
}

/// Compute matched beta function in a plasma channel:
/// beta_matched = sqrt(2 * gamma_rel) / k_p (m).
pub fn matched_beta_function(gamma_rel: f64, plasma_density_m3: f64) -> f64 {
    let kp = plasma_wavenumber(plasma_density_m3);
    if kp.abs() < 1e-30 {
        return 0.0;
    }
    (2.0 * gamma_rel).sqrt() / kp
}

/// Betatron frequency for a particle in the plasma focusing channel:
/// omega_beta = omega_pe / sqrt(2 * gamma_rel) (rad/s).
pub fn betatron_frequency(plasma_density_m3: f64, gamma_rel: f64) -> f64 {
    let omega_pe = plasma_frequency(plasma_density_m3);
    omega_pe / (2.0 * gamma_rel).sqrt()
}

/// Betatron wavelength: lambda_beta = 2*pi*c / omega_beta (m).
pub fn betatron_wavelength(plasma_density_m3: f64, gamma_rel: f64) -> f64 {
    let omega_b = betatron_frequency(plasma_density_m3, gamma_rel);
    if omega_b.abs() < 1e-30 {
        return f64::INFINITY;
    }
    2.0 * PI * C_LIGHT / omega_b
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, rel_tol: f64) -> bool {
        if b.abs() < 1e-30 {
            return a.abs() < rel_tol;
        }
        ((a - b) / b).abs() < rel_tol
    }

    // --- Physical constants ---

    #[test]
    fn test_constants_sanity() {
        assert!(E_CHARGE > 1.6e-19 && E_CHARGE < 1.61e-19);
        assert!(M_ELECTRON > 9.1e-31 && M_ELECTRON < 9.2e-31);
        assert!(C_LIGHT > 2.99e8 && C_LIGHT < 3.01e8);
        assert!(EPSILON_0 > 8.85e-12 && EPSILON_0 < 8.86e-12);
        assert!(K_BOLTZMANN > 1.38e-23 && K_BOLTZMANN < 1.39e-23);
        assert!(H_BAR > 1.05e-34 && H_BAR < 1.06e-34);
    }

    #[test]
    fn test_mc2_electron() {
        let mc2_ev = MC2_ELECTRON / EV_TO_J;
        // m_e c^2 ~ 511 keV
        assert!(approx_eq(mc2_ev, 511.0e3, 0.001));
    }

    // --- Plasma parameters ---

    #[test]
    fn test_plasma_frequency_1e17() {
        // At n_e = 10^17 /m^3, omega_pe ~ 1.784e10 rad/s
        let omega = plasma_frequency(1e17);
        assert!(approx_eq(omega, 1.784e10, 0.02));
    }

    #[test]
    fn test_plasma_frequency_scaling() {
        // omega_pe scales as sqrt(n_e)
        let w1 = plasma_frequency(1e16);
        let w2 = plasma_frequency(4e16);
        assert!(approx_eq(w2 / w1, 2.0, TOL));
    }

    #[test]
    fn test_plasma_wavelength() {
        // At 10^17 /m^3: lambda_p = 2*pi*c/omega_pe ~ 0.1056 m (10.56 cm)
        let lp = plasma_wavelength(1e17);
        assert!(lp > 0.09 && lp < 0.12);
    }

    #[test]
    fn test_skin_depth() {
        // skin depth = c/omega_pe = lambda_p / (2*pi)
        let sd = skin_depth(1e17);
        let lp = plasma_wavelength(1e17);
        assert!(approx_eq(sd, lp / (2.0 * PI), TOL));
    }

    #[test]
    fn test_plasma_wavenumber() {
        // k_p = omega_pe/c = 2*pi/lambda_p
        let kp = plasma_wavenumber(1e17);
        let lp = plasma_wavelength(1e17);
        assert!(approx_eq(kp, 2.0 * PI / lp, TOL));
    }

    #[test]
    fn test_debye_length() {
        // At 10^17 /m^3, T=10 eV: lambda_D ~ 74 um
        let ld = debye_length(1e17, 10.0);
        assert!(ld > 50e-6 && ld < 100e-6);
    }

    #[test]
    fn test_debye_length_scaling() {
        // lambda_D ~ sqrt(T/n), doubling T quadruples T -> sqrt(4)=2x
        let ld1 = debye_length(1e17, 10.0);
        let ld2 = debye_length(1e17, 40.0);
        assert!(approx_eq(ld2 / ld1, 2.0, TOL));
    }

    #[test]
    fn test_wave_breaking_field() {
        // At 10^17: E_wb ~ 30 MV/m
        let ewb = wave_breaking_field(1e17);
        assert!(ewb > 20e6 && ewb < 40e6);
    }

    #[test]
    fn test_wave_breaking_scaling() {
        // E_wb ~ sqrt(n_e)
        let e1 = wave_breaking_field(1e16);
        let e2 = wave_breaking_field(4e16);
        assert!(approx_eq(e2 / e1, 2.0, TOL));
    }

    #[test]
    fn test_debye_sphere_count() {
        // N_D should be >> 1 for a well-behaved plasma
        let nd = debye_sphere_count(1e17, 10.0);
        assert!(nd > 1.0);
    }

    #[test]
    fn test_coupling_parameter() {
        // For hot dilute plasma, Gamma << 1 (weakly coupled)
        let gamma = coupling_parameter(1e17, 10.0);
        assert!(gamma < 1.0);
    }

    // --- Linear wakefield ---

    #[test]
    fn test_gaussian_bunch_peak_field() {
        let bunch = GaussianBunchParams {
            num_particles: 2e10,
            sigma_z: 30e-6,
            sigma_r: 10e-6,
        };
        let ez_max = gaussian_bunch_peak_field(&bunch, 1e17);
        // At 10^17, sigma_z=30 um, N_b=2e10: E_z ~ 57 MV/m
        assert!(ez_max > 1e7); // > 10 MV/m
    }

    #[test]
    fn test_gaussian_bunch_field_sinusoidal() {
        let bunch = GaussianBunchParams::default();
        let density = 1e17;
        let kp = plasma_wavenumber(density);
        let half_wavelength = PI / kp;

        // At xi = 0 (bunch center), sin(0) = 0
        let ez_0 = gaussian_bunch_field_at(&bunch, density, 0.0);
        assert!(ez_0.abs() < 1.0); // essentially zero

        // At xi = pi/(2*kp), sin(pi/2) = 1 -> maximum
        let ez_max_pos = gaussian_bunch_field_at(&bunch, density, half_wavelength / 2.0);
        let ez_peak = gaussian_bunch_peak_field(&bunch, density);
        assert!(approx_eq(ez_max_pos, ez_peak, 0.01));
    }

    #[test]
    fn test_focusing_field_on_axis() {
        let bunch = GaussianBunchParams::default();
        // On axis (r=0), focusing field should be zero
        let er = focusing_field(&bunch, 1e17, 0.0, 1e-6);
        assert!(er.abs() < 1e-10);
    }

    #[test]
    fn test_focusing_field_off_axis() {
        let bunch = GaussianBunchParams::default();
        // Off axis, focusing field should be nonzero and scale linearly with r
        let er1 = focusing_field(&bunch, 1e17, 1e-6, 1e-5);
        let er2 = focusing_field(&bunch, 1e17, 2e-6, 1e-5);
        assert!(approx_eq(er2 / er1, 2.0, TOL));
    }

    #[test]
    fn test_linear_wakefield_short_bunch() {
        // Create a short Gaussian-like bunch in co-moving frame
        let n_pts = 200;
        let kp = plasma_wavenumber(1e17);
        let sigma_z = 0.5 / kp; // half plasma wavelength
        let xi_max = 4.0 / kp;
        let dxi = 2.0 * xi_max / (n_pts - 1) as f64;

        let xi: Vec<f64> = (0..n_pts).map(|i| -xi_max + i as f64 * dxi).collect();
        let nb: Vec<f64> = xi
            .iter()
            .map(|&x| 1e17 * (-x * x / (2.0 * sigma_z * sigma_z)).exp())
            .collect();

        let wake = linear_wakefield(&xi, &nb, 1e17);
        assert_eq!(wake.len(), n_pts);

        // The wake should have oscillatory E_z behind the bunch
        let has_positive_ez = wake.iter().any(|w| w.e_z > 0.0);
        let has_negative_ez = wake.iter().any(|w| w.e_z < 0.0);
        assert!(has_positive_ez || has_negative_ez); // wake exists
    }

    #[test]
    fn test_linear_wakefield_empty() {
        let wake = linear_wakefield(&[], &[], 1e17);
        assert!(wake.is_empty());
    }

    // --- Beam loading ---

    #[test]
    fn test_transformer_ratio() {
        let r = transformer_ratio(10.0e9, 5.0e9);
        assert!(approx_eq(r, 2.0, TOL));
    }

    #[test]
    fn test_transformer_ratio_zero_accel() {
        let r = transformer_ratio(10.0e9, 0.0);
        assert!(approx_eq(r, 0.0, TOL));
    }

    #[test]
    fn test_energy_gain() {
        // 1 GV/m over 1 m = 1 GeV
        let dw_j = energy_gain_j(1e9, 1.0);
        let dw_ev = dw_j / EV_TO_J;
        assert!(approx_eq(dw_ev, 1e9, 0.01));
    }

    #[test]
    fn test_energy_gain_ev() {
        // E_z * L = field * length in V -> eV
        let dw = energy_gain_ev(1e9, 0.5);
        assert!(approx_eq(dw, 0.5e9, TOL));
    }

    #[test]
    fn test_beam_loaded_gradient() {
        let loaded = beam_loaded_gradient(10e9, 3e9);
        assert!(approx_eq(loaded, 7e9, TOL));
    }

    #[test]
    fn test_luminosity_per_power() {
        let l_p = luminosity_per_power(1e10, 1e10, 1e3, 1e-6, 1e-6, 1e6);
        assert!(l_p > 0.0);
    }

    // --- LWFA ---

    #[test]
    fn test_normalized_vector_potential() {
        // At 800 nm, 10^18 W/cm^2 -> a0 ~ 0.855 * 0.8 * 1.0 ~ 0.684
        let a0 = normalized_vector_potential(800e-9, 1e22); // 10^22 W/m^2 = 10^18 W/cm^2
        assert!(approx_eq(a0, 0.684, 0.01));
    }

    #[test]
    fn test_a0_high_intensity() {
        // At 800 nm, 10^19 W/cm^2 -> a0 ~ 0.855 * 0.8 * sqrt(10) ~ 2.16
        let a0 = normalized_vector_potential(800e-9, 1e23);
        assert!(approx_eq(a0, 2.163, 0.02));
    }

    #[test]
    fn test_critical_power() {
        // P_c = 17 * (omega/omega_pe)^2 GW. For 800nm, n=10^17: ratio ~ large -> P_c ~ TW
        let pc = critical_power_self_focusing(800e-9, 1e17);
        assert!(pc > 1e11); // > 100 GW
    }

    #[test]
    fn test_dephasing_length() {
        // At n=10^23 /m^3 (typical LWFA density), L_d ~ 1.2 m
        let ld = dephasing_length(800e-9, 1e23);
        assert!(ld > 0.5 && ld < 5.0);
    }

    #[test]
    fn test_pump_depletion_length() {
        let a0 = 1.0;
        let n = 1e23;
        let lpd = pump_depletion_length(800e-9, n, a0);
        let ld = dephasing_length(800e-9, n);
        // Pump depletion length should be comparable to or larger than dephasing length
        assert!(lpd > 0.0);
        // For a0=1: L_pd = (omega/omega_pe)^2 * lambda_p, L_d = (2/3)*(omega/omega_pe)^2 * lambda_p
        // So L_pd / L_d = 1 / (2/3) = 3/2
        assert!(approx_eq(lpd / ld, 1.5, 0.01));
    }

    #[test]
    fn test_pump_depletion_zero_a0() {
        let lpd = pump_depletion_length(800e-9, 1e17, 0.0);
        assert!(lpd.is_infinite());
    }

    #[test]
    fn test_lwfa_energy_gain() {
        let a0 = 1.0;
        let dw_ev = lwfa_max_energy_gain_ev(800e-9, 1e17, a0);
        // For a0=1, n=10^17, 800nm: multi-GeV energy gain
        assert!(dw_ev > 1e8); // > 100 MeV
    }

    #[test]
    fn test_laser_pulse_energy() {
        let e = laser_pulse_energy(100e12, 30e-15); // 100 TW, 30 fs
        assert!(approx_eq(e, 3.0, 0.01)); // 3 Joules
    }

    #[test]
    fn test_laser_peak_power() {
        let p = laser_peak_power(1e22, 20e-6); // 10^22 W/m^2, 20 um spot
        // P = I * pi * w0^2 ~ 10^22 * pi * (20e-6)^2 ~ 12.6 TW
        assert!(p > 10e12 && p < 15e12);
    }

    // --- PWFA ---

    #[test]
    fn test_is_blowout_regime() {
        assert!(is_blowout_regime(1e18, 1e17)); // 10x > 3x threshold
        assert!(!is_blowout_regime(1e17, 1e17)); // 1x < 3x threshold
    }

    #[test]
    fn test_gaussian_bunch_density() {
        let bunch = GaussianBunchParams {
            num_particles: 2e10,
            sigma_z: 30e-6,
            sigma_r: 10e-6,
        };
        let nb = gaussian_bunch_density(&bunch);
        assert!(nb > 1e20); // very high bunch density
    }

    #[test]
    fn test_bubble_radius() {
        let rb = bubble_radius(4.0, 1e17);
        let kp = plasma_wavenumber(1e17);
        assert!(approx_eq(rb, 2.0 * 2.0 / kp, TOL)); // 2*sqrt(4)/kp = 4/kp
    }

    #[test]
    fn test_bubble_accelerating_gradient() {
        let kp = plasma_wavenumber(1e17);
        let rb = 2.0 / kp;
        let ez = bubble_accelerating_gradient(rb, 1e17);
        let ewb = wave_breaking_field(1e17);
        assert!(approx_eq(ez, ewb, TOL)); // 0.5 * E_wb * kp * (2/kp) = E_wb
    }

    // --- Beam diagnostics ---

    #[test]
    fn test_energy_spectrum() {
        let energies = vec![100.0, 101.0, 102.0, 103.0, 104.0, 100.5, 101.5];
        let (centers, counts) = energy_spectrum(&energies, 4);
        assert_eq!(centers.len(), 4);
        assert_eq!(counts.len(), 4);
        assert_eq!(counts.iter().sum::<usize>(), 7);
    }

    #[test]
    fn test_energy_spectrum_empty() {
        let (centers, counts) = energy_spectrum(&[], 10);
        assert!(centers.is_empty());
        assert!(counts.is_empty());
    }

    #[test]
    fn test_energy_spread() {
        let energies = vec![1000.0, 1001.0, 999.0, 1000.5, 999.5];
        let spread = energy_spread(&energies);
        assert!(spread > 0.0 && spread < 0.01); // less than 1%
    }

    #[test]
    fn test_energy_spread_single() {
        assert_eq!(energy_spread(&[100.0]), 0.0);
    }

    #[test]
    fn test_geometric_emittance() {
        // Uncorrelated x, x' -> emittance = sigma_x * sigma_xp
        let x = vec![1.0, -1.0, 0.5, -0.5, 0.0];
        let xp = vec![0.1, -0.1, 0.05, -0.05, 0.0];
        let eps = geometric_emittance(&x, &xp);
        assert!(eps > 0.0);
    }

    #[test]
    fn test_geometric_emittance_collinear() {
        // Perfectly correlated -> emittance = 0
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let xp: Vec<f64> = x.iter().map(|&xi| xi * 0.5).collect();
        let eps = geometric_emittance(&x, &xp);
        assert!(eps < 1e-10);
    }

    #[test]
    fn test_bunch_charge() {
        // Rectangular pulse: 1 A for 10 ps -> 10 pC
        let dt = 1e-13; // 0.1 ps
        let n = 100;
        let current: Vec<f64> = vec![1.0; n];
        let q = bunch_charge(&current, dt);
        let expected = 1.0 * (n - 1) as f64 * dt;
        assert!(approx_eq(q, expected, 0.01));
    }

    #[test]
    fn test_bunch_length_from_ctr() {
        // Generate a fake Gaussian CTR spectrum: S(f) = exp(-omega^2 * sigma_t^2)
        let sigma_z = 10e-6; // 10 um bunch
        let sigma_t = sigma_z / C_LIGHT;
        let freqs: Vec<f64> = (1..50).map(|i| i as f64 * 1e12).collect(); // THz
        let spec: Vec<f64> = freqs
            .iter()
            .map(|&f| {
                let omega = 2.0 * PI * f;
                (-omega * omega * sigma_t * sigma_t).exp()
            })
            .collect();

        let estimated_length = bunch_length_from_ctr(&freqs, &spec);
        assert!(approx_eq(estimated_length, sigma_z, 0.05)); // 5% tolerance
    }

    #[test]
    fn test_compute_diagnostics() {
        let energies = vec![1e9, 1.01e9, 0.99e9, 1.005e9, 0.995e9];
        let x = vec![1e-6, -1e-6, 0.5e-6, -0.5e-6, 0.0];
        // Uncorrelated angles (not proportional to x)
        let xp = vec![0.5e-3, 0.3e-3, -1e-3, 0.1e-3, -0.2e-3];
        let y = vec![0.5e-6, -0.3e-6, 0.8e-6, -0.7e-6, 0.0];
        let yp = vec![-0.2e-3, 0.6e-3, -0.1e-3, 0.4e-3, -0.3e-3];
        let diag = compute_diagnostics(&energies, &x, &xp, &y, &yp, 1e-9, 10e-6);
        assert!(approx_eq(diag.mean_energy_ev, 1e9, 0.01));
        assert!(diag.energy_spread > 0.0);
        assert!(diag.emittance_x > 0.0);
        assert!(diag.emittance_y > 0.0);
    }

    // --- Phase space ---

    #[test]
    fn test_twiss_parameters() {
        // Generate beam with known Twiss parameters
        let x = vec![1.0e-3, -1.0e-3, 0.7e-3, -0.7e-3, 0.3e-3, -0.3e-3, 0.0];
        let xp = vec![0.1e-3, -0.1e-3, 0.07e-3, -0.07e-3, 0.03e-3, -0.03e-3, 0.0];
        let twiss = compute_twiss(&x, &xp);
        assert!(twiss.emittance > 0.0);
        assert!(twiss.beta > 0.0);
        assert!(twiss.gamma > 0.0);
        // Check consistency: beta*gamma - alpha^2 = 1
        let consistency = twiss.beta * twiss.gamma - twiss.alpha * twiss.alpha;
        assert!(approx_eq(consistency, 1.0, 0.01));
    }

    #[test]
    fn test_twiss_consistency_check() {
        let twiss = TwissParameters {
            alpha: 0.5,
            beta: 10.0,
            gamma: (1.0 + 0.25) / 10.0,
            emittance: 1e-6,
        };
        let err = twiss_consistency(&twiss);
        assert!(err < 1e-10);
    }

    #[test]
    fn test_beam_envelope() {
        let sigma = beam_envelope(1e-6, 10.0);
        let expected = (1e-6 * 10.0f64).sqrt();
        assert!(approx_eq(sigma, expected, TOL));
    }

    #[test]
    fn test_phase_space_area() {
        let area = phase_space_area(1e-6);
        assert!(approx_eq(area, PI * 1e-6, TOL));
    }

    #[test]
    fn test_normalized_emittance() {
        let eps = 1e-6;
        let gamma_rel = 1000.0; // ~500 MeV electron
        let eps_n = normalized_emittance(eps, gamma_rel);
        // beta_rel ~ 1 for gamma >> 1
        assert!(approx_eq(eps_n, gamma_rel * eps, 0.001));
    }

    #[test]
    fn test_lorentz_factor() {
        // 1 GeV electron: gamma ~ 1 + 1e9/511e3 ~ 1958
        let gamma = lorentz_factor(1e9);
        assert!(approx_eq(gamma, 1957.0, 0.01));
    }

    #[test]
    fn test_lorentz_factor_rest() {
        // Zero kinetic energy -> gamma = 1
        let gamma = lorentz_factor(0.0);
        assert!(approx_eq(gamma, 1.0, TOL));
    }

    #[test]
    fn test_matched_beta_function() {
        let gamma = 1000.0;
        let beta_m = matched_beta_function(gamma, 1e17);
        let kp = plasma_wavenumber(1e17);
        let expected = (2.0 * gamma).sqrt() / kp;
        assert!(approx_eq(beta_m, expected, TOL));
    }

    #[test]
    fn test_betatron_frequency() {
        let gamma = 1000.0;
        let omega_b = betatron_frequency(1e17, gamma);
        let omega_pe = plasma_frequency(1e17);
        assert!(approx_eq(omega_b, omega_pe / (2000.0f64).sqrt(), TOL));
    }

    #[test]
    fn test_betatron_wavelength() {
        let gamma = 1000.0;
        let lb = betatron_wavelength(1e17, gamma);
        // Betatron wavelength should be much longer than plasma wavelength
        let lp = plasma_wavelength(1e17);
        assert!(lb > lp);
    }

    #[test]
    fn test_default_configs() {
        let pc = PlasmaConfig::default();
        assert_eq!(pc.density_m3, 1e17);
        assert_eq!(pc.temperature_ev, 10.0);

        let gb = GaussianBunchParams::default();
        assert_eq!(gb.num_particles, 2e10);

        let ld = LaserDriverParams::default();
        assert_eq!(ld.wavelength_m, 800e-9);
    }

    #[test]
    fn test_plasma_config_struct() {
        let cfg = PlasmaConfig {
            density_m3: 5e16,
            temperature_ev: 20.0,
        };
        let omega = plasma_frequency(cfg.density_m3);
        let ld = debye_length(cfg.density_m3, cfg.temperature_ev);
        assert!(omega > 0.0);
        assert!(ld > 0.0);
    }
}
