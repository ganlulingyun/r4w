//! # Optical Tweezers Controller
//!
//! Laser-based particle trapping and manipulation using tightly focused
//! Gaussian beams. Optical tweezers use radiation pressure and gradient
//! forces to confine dielectric particles at the beam focus.
//!
//! ## Background
//!
//! A tightly focused laser beam creates an intensity gradient near the focal
//! point. In the Rayleigh regime (particle radius << wavelength), two forces
//! act on a dielectric particle:
//!
//! - **Gradient force**: Pulls the particle toward the intensity maximum
//!   (beam focus). Proportional to the gradient of |E|^2.
//! - **Scattering force**: Pushes the particle along the beam propagation
//!   direction due to momentum transfer from scattered photons.
//!
//! Stable 3D trapping occurs when the axial gradient force exceeds the
//! scattering force. A trapped particle undergoes Brownian motion confined
//! by the harmonic potential of the trap.
//!
//! ## Applications
//!
//! - **Biophysics**: Measure piconewton forces on DNA, molecular motors, cell membranes
//! - **Colloidal science**: Probe interactions between microscopic particles
//! - **Microrheology**: Measure local viscosity from trapped particle fluctuations
//! - **Force spectroscopy**: Unfold proteins, stretch polymers with controlled force
//!
//! ## Key Equations
//!
//! - Polarizability: alpha = 4 pi n_m^2 r^3 (m^2 - 1)/(m^2 + 2), m = n_p/n_m
//! - Scattering cross-section: sigma = (128 pi^5 r^6)/(3 lambda^4) * ((m^2-1)/(m^2+2))^2
//! - Gradient force: F_grad = -(n_m alpha / 2) * grad(|E|^2)
//! - Scattering force: F_scat = (n_m / c) * sigma * I
//! - Stokes drag: gamma = 6 pi eta r
//! - Corner frequency: f_c = k / (2 pi gamma)
//! - Equipartition: k = k_B T / <x^2>
//! - WLC model: F = (k_B T / L_p) [1/(4(1-x/L)^2) - 1/4 + x/L]

use std::f64::consts::PI;

// ──────────────────────────────────────────────────────────
// Physical constants
// ──────────────────────────────────────────────────────────

/// Speed of light in vacuum (m/s).
const C_LIGHT: f64 = 2.998e8;

/// Boltzmann constant (J/K).
const K_BOLTZMANN: f64 = 1.380649e-23;

/// Vacuum permittivity (F/m).
const EPSILON_0: f64 = 8.854187817e-12;

// ──────────────────────────────────────────────────────────
// TrapConfig
// ──────────────────────────────────────────────────────────

/// Configuration for an optical trap.
#[derive(Debug, Clone)]
pub struct TrapConfig {
    /// Laser wavelength in nanometers.
    pub wavelength_nm: f64,
    /// Numerical aperture of the focusing objective.
    pub numerical_aperture: f64,
    /// Laser power at the sample in milliwatts.
    pub laser_power_mw: f64,
    /// Refractive index of the surrounding medium (e.g. 1.33 for water).
    pub medium_refractive_index: f64,
    /// Particle radius in micrometers.
    pub particle_radius_um: f64,
    /// Refractive index of the particle (e.g. 1.59 for polystyrene).
    pub particle_refractive_index: f64,
}

impl TrapConfig {
    /// Create a typical optical tweezers configuration:
    /// 1064 nm Nd:YAG, NA 1.25 oil-immersion, 100 mW,
    /// 0.5 um polystyrene bead in water.
    pub fn default_config() -> Self {
        Self {
            wavelength_nm: 1064.0,
            numerical_aperture: 1.25,
            laser_power_mw: 100.0,
            medium_refractive_index: 1.33,
            particle_radius_um: 0.5,
            particle_refractive_index: 1.59,
        }
    }

    /// Wavelength in meters.
    pub fn wavelength_m(&self) -> f64 {
        self.wavelength_nm * 1.0e-9
    }

    /// Particle radius in meters.
    pub fn radius_m(&self) -> f64 {
        self.particle_radius_um * 1.0e-6
    }

    /// Laser power in watts.
    pub fn power_w(&self) -> f64 {
        self.laser_power_mw * 1.0e-3
    }

    /// Relative refractive index m = n_p / n_m.
    pub fn relative_index(&self) -> f64 {
        self.particle_refractive_index / self.medium_refractive_index
    }

    /// Clausius-Mossotti factor: (m^2 - 1) / (m^2 + 2).
    pub fn clausius_mossotti(&self) -> f64 {
        let m2 = self.relative_index().powi(2);
        (m2 - 1.0) / (m2 + 2.0)
    }

    /// Polarizability alpha = 4 pi n_m^2 r^3 (m^2 - 1)/(m^2 + 2).
    ///
    /// Units: m^3 (in Gaussian-like form scaled by n_m^2).
    pub fn polarizability(&self) -> f64 {
        let r = self.radius_m();
        let n_m = self.medium_refractive_index;
        4.0 * PI * n_m * n_m * r.powi(3) * self.clausius_mossotti()
    }

    /// Rayleigh scattering cross-section:
    /// sigma = (128 pi^5 r^6) / (3 lambda^4) * ((m^2-1)/(m^2+2))^2.
    pub fn scattering_cross_section(&self) -> f64 {
        let r = self.radius_m();
        let lambda = self.wavelength_m();
        let cm = self.clausius_mossotti();
        (128.0 * PI.powi(5) * r.powi(6)) / (3.0 * lambda.powi(4)) * cm * cm
    }
}

// ──────────────────────────────────────────────────────────
// Gaussian beam model
// ──────────────────────────────────────────────────────────

/// Gaussian beam parameters derived from a TrapConfig.
#[derive(Debug, Clone)]
pub struct GaussianBeam {
    /// Beam waist radius w0 in meters.
    pub w0: f64,
    /// Rayleigh range z_R in meters.
    pub z_r: f64,
    /// Wavelength in meters.
    pub wavelength: f64,
    /// Peak intensity I0 at focus in W/m^2.
    pub i0: f64,
}

impl GaussianBeam {
    /// Construct beam parameters from a trap configuration.
    ///
    /// Beam waist: w0 ~ lambda / (pi * NA)
    /// Rayleigh range: z_R = pi * w0^2 / lambda
    /// Peak intensity: I0 = 2P / (pi * w0^2)
    pub fn from_config(config: &TrapConfig) -> Self {
        let lambda = config.wavelength_m();
        let w0 = lambda / (PI * config.numerical_aperture);
        let z_r = PI * w0 * w0 / lambda;
        let power = config.power_w();
        let i0 = 2.0 * power / (PI * w0 * w0);

        Self {
            w0,
            z_r,
            wavelength: lambda,
            i0,
        }
    }

    /// Beam radius at axial position z: w(z) = w0 * sqrt(1 + (z/z_R)^2).
    pub fn beam_radius(&self, z: f64) -> f64 {
        self.w0 * (1.0 + (z / self.z_r).powi(2)).sqrt()
    }

    /// Intensity at position (r, z) in cylindrical coordinates.
    ///
    /// I(r,z) = I0 * (w0/w(z))^2 * exp(-2 r^2 / w(z)^2)
    pub fn intensity(&self, r: f64, z: f64) -> f64 {
        let wz = self.beam_radius(z);
        let ratio = self.w0 / wz;
        self.i0 * ratio * ratio * (-2.0 * r * r / (wz * wz)).exp()
    }

    /// Radial intensity gradient dI/dr at position (r, z).
    ///
    /// dI/dr = I(r,z) * (-4r / w(z)^2)
    pub fn intensity_gradient_r(&self, r: f64, z: f64) -> f64 {
        let wz = self.beam_radius(z);
        self.intensity(r, z) * (-4.0 * r / (wz * wz))
    }

    /// Axial intensity gradient dI/dz at position (r, z), computed numerically.
    pub fn intensity_gradient_z(&self, r: f64, z: f64) -> f64 {
        let dz = self.wavelength * 0.001;
        (self.intensity(r, z + dz) - self.intensity(r, z - dz)) / (2.0 * dz)
    }
}

// ──────────────────────────────────────────────────────────
// Optical force calculations
// ──────────────────────────────────────────────────────────

/// Gradient force magnitude in the radial direction at position (r, z).
///
/// F_grad_r = -(n_m * alpha / 2) * dI/dr
/// In SI with the intensity-based formulation using (n_m / (2 * epsilon_0 * c)):
/// F_grad = -(2 * pi * n_m * alpha) / c * dI/dr  (simplified Rayleigh regime)
///
/// We use the common formulation:
/// F_grad = (n_m / (2 * epsilon_0 * c)) * alpha_real * grad(I) / (2 * c * n_m)
///
/// For the Rayleigh regime, the radial gradient force is:
/// F_grad_r = (n_m * alpha) / (2 * c * epsilon_0) * dI/dr
pub fn gradient_force_radial(config: &TrapConfig, beam: &GaussianBeam, r: f64, z: f64) -> f64 {
    let alpha = config.polarizability();
    let _n_m = config.medium_refractive_index;
    let di_dr = beam.intensity_gradient_r(r, z);
    // F_grad = (n_m * alpha) / (2 * c * epsilon_0) * grad(I)
    // But alpha already has n_m^2 factor. The standard Rayleigh gradient force is:
    // F_grad = (2 * pi * r^3 / c) * (n_m^2) * CM * grad(I)
    // Simplify: use alpha / (epsilon_0 * c) * grad(I) / 2
    (alpha / (2.0 * EPSILON_0 * C_LIGHT)) * di_dr
}

/// Gradient force in the axial direction at position (r, z).
pub fn gradient_force_axial(config: &TrapConfig, beam: &GaussianBeam, r: f64, z: f64) -> f64 {
    let alpha = config.polarizability();
    let di_dz = beam.intensity_gradient_z(r, z);
    (alpha / (2.0 * EPSILON_0 * C_LIGHT)) * di_dz
}

/// Scattering force along the beam propagation axis.
///
/// F_scat = (n_m / c) * sigma * I(r,z)
pub fn scattering_force(config: &TrapConfig, beam: &GaussianBeam, r: f64, z: f64) -> f64 {
    let sigma = config.scattering_cross_section();
    let n_m = config.medium_refractive_index;
    let intensity = beam.intensity(r, z);
    (n_m / C_LIGHT) * sigma * intensity
}

/// Estimate radial trap stiffness k_r from the linear gradient force region.
///
/// k = -F_grad / displacement for small displacement.
/// We evaluate at a small offset from the beam axis.
pub fn trap_stiffness_radial(config: &TrapConfig, beam: &GaussianBeam) -> f64 {
    let dr = beam.w0 * 0.01; // small displacement
    let force = gradient_force_radial(config, beam, dr, 0.0);
    // force is negative (restoring), stiffness is positive
    -force / dr
}

/// Estimate axial trap stiffness k_z.
pub fn trap_stiffness_axial(config: &TrapConfig, beam: &GaussianBeam) -> f64 {
    let dz = beam.z_r * 0.01;
    let force = gradient_force_axial(config, beam, 0.0, dz);
    -force / dz
}

/// Ratio of gradient to scattering force at focus. Must be > 1 for stable trapping.
pub fn trapping_efficiency(config: &TrapConfig, beam: &GaussianBeam) -> f64 {
    let dr = beam.w0 * 0.1;
    let f_grad = gradient_force_radial(config, beam, dr, 0.0).abs();
    let f_scat = scattering_force(config, beam, 0.0, 0.0);
    if f_scat > 0.0 {
        f_grad / f_scat
    } else {
        f64::INFINITY
    }
}

// ──────────────────────────────────────────────────────────
// Brownian motion simulation
// ──────────────────────────────────────────────────────────

/// Configuration for Brownian motion simulation of a trapped particle.
#[derive(Debug, Clone)]
pub struct BrownianConfig {
    /// Temperature in Kelvin.
    pub temperature_k: f64,
    /// Dynamic viscosity in Pa*s (water at 20 C ~ 0.001).
    pub viscosity_pa_s: f64,
    /// Trap stiffness in N/m.
    pub trap_stiffness: f64,
    /// Particle radius in meters.
    pub particle_radius_m: f64,
    /// Time step for integration in seconds.
    pub dt: f64,
}

impl BrownianConfig {
    /// Stokes drag coefficient gamma = 6 pi eta r.
    pub fn drag_coefficient(&self) -> f64 {
        6.0 * PI * self.viscosity_pa_s * self.particle_radius_m
    }

    /// Diffusion coefficient D = k_B T / gamma.
    pub fn diffusion_coefficient(&self) -> f64 {
        K_BOLTZMANN * self.temperature_k / self.drag_coefficient()
    }

    /// Corner frequency f_c = k / (2 pi gamma).
    pub fn corner_frequency(&self) -> f64 {
        self.trap_stiffness / (2.0 * PI * self.drag_coefficient())
    }

    /// Thermal equilibrium position variance <x^2> = k_B T / k.
    pub fn thermal_variance(&self) -> f64 {
        K_BOLTZMANN * self.temperature_k / self.trap_stiffness
    }
}

/// Simple linear congruential pseudo-random number generator for reproducibility.
/// Produces uniform values in [0, 1).
struct SimplePrng {
    state: u64,
}

impl SimplePrng {
    fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(1),
        }
    }

    fn next_u64(&mut self) -> u64 {
        // LCG parameters (Numerical Recipes)
        self.state = self.state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        self.state
    }

    fn next_uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Box-Muller transform for Gaussian random numbers.
    fn next_gaussian(&mut self) -> f64 {
        loop {
            let u1 = self.next_uniform();
            let u2 = self.next_uniform();
            if u1 > 1.0e-30 {
                return (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos();
            }
        }
    }
}

/// Simulate 1D Brownian motion of a trapped particle using Euler-Maruyama.
///
/// Overdamped Langevin equation:
///   gamma * dx/dt = -k*x + F_random
///   dx = (-k*x/gamma)*dt + sqrt(2*k_B*T*dt/gamma) * dW
///
/// # Arguments
/// * `config` - Brownian motion parameters
/// * `num_steps` - Number of time steps to simulate
/// * `seed` - Random seed for reproducibility
///
/// # Returns
/// Vector of positions at each time step (in meters).
pub fn simulate_brownian_1d(config: &BrownianConfig, num_steps: usize, seed: u64) -> Vec<f64> {
    let gamma = config.drag_coefficient();
    let k = config.trap_stiffness;
    let dt = config.dt;
    let noise_amplitude = (2.0 * K_BOLTZMANN * config.temperature_k * dt / gamma).sqrt();

    let mut rng = SimplePrng::new(seed);
    let mut positions = Vec::with_capacity(num_steps);
    let mut x = 0.0_f64;

    for _ in 0..num_steps {
        positions.push(x);
        let deterministic = -(k / gamma) * x * dt;
        let stochastic = noise_amplitude * rng.next_gaussian();
        x += deterministic + stochastic;
    }

    positions
}

/// Simulate 2D Brownian motion in the trap focal plane.
///
/// # Returns
/// Vector of (x, y) positions in meters.
pub fn simulate_brownian_2d(
    config: &BrownianConfig,
    num_steps: usize,
    seed: u64,
) -> Vec<(f64, f64)> {
    let gamma = config.drag_coefficient();
    let k = config.trap_stiffness;
    let dt = config.dt;
    let noise_amp = (2.0 * K_BOLTZMANN * config.temperature_k * dt / gamma).sqrt();

    let mut rng = SimplePrng::new(seed);
    let mut positions = Vec::with_capacity(num_steps);
    let mut x = 0.0;
    let mut y = 0.0;

    for _ in 0..num_steps {
        positions.push((x, y));
        x += -(k / gamma) * x * dt + noise_amp * rng.next_gaussian();
        y += -(k / gamma) * y * dt + noise_amp * rng.next_gaussian();
    }

    positions
}

// ──────────────────────────────────────────────────────────
// Trap calibration
// ──────────────────────────────────────────────────────────

/// Calibrate trap stiffness using the equipartition theorem.
///
/// k = k_B * T / <x^2>
///
/// # Arguments
/// * `positions` - Measured particle positions (m)
/// * `temperature_k` - Temperature in Kelvin
pub fn calibrate_equipartition(positions: &[f64], temperature_k: f64) -> f64 {
    if positions.is_empty() {
        return 0.0;
    }
    let mean = positions.iter().sum::<f64>() / positions.len() as f64;
    let variance =
        positions.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / positions.len() as f64;
    if variance <= 0.0 {
        return 0.0;
    }
    K_BOLTZMANN * temperature_k / variance
}

/// Compute the power spectral density (PSD) of a position time series.
///
/// Uses a simple DFT approach (no windowing for simplicity).
///
/// # Returns
/// Vector of (frequency_Hz, PSD_value) pairs.
pub fn compute_psd(positions: &[f64], sample_rate: f64) -> Vec<(f64, f64)> {
    let n = positions.len();
    if n == 0 {
        return vec![];
    }
    let mean = positions.iter().sum::<f64>() / n as f64;
    let centered: Vec<f64> = positions.iter().map(|x| x - mean).collect();

    let n_freq = n / 2 + 1;
    let mut psd = Vec::with_capacity(n_freq);

    for k in 0..n_freq {
        let mut re = 0.0;
        let mut im = 0.0;
        for (i, &val) in centered.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
            re += val * angle.cos();
            im += val * angle.sin();
        }
        let power = (re * re + im * im) / (n as f64 * sample_rate);
        let freq = k as f64 * sample_rate / n as f64;
        psd.push((freq, power));
    }

    psd
}

/// Lorentzian model for the PSD of a trapped particle:
///
/// S(f) = S0 / (f_c^2 + f^2)
///
/// where f_c = k / (2 pi gamma) is the corner frequency.
pub fn lorentzian_psd(f: f64, s0: f64, f_c: f64) -> f64 {
    s0 / (f_c * f_c + f * f)
}

/// Fit the corner frequency from a PSD by finding the half-power point.
///
/// The Lorentzian S(f) drops to half its DC value at f = f_c.
///
/// # Arguments
/// * `psd` - Vector of (frequency, PSD_value) pairs, sorted by frequency
///
/// # Returns
/// Estimated corner frequency in Hz.
pub fn fit_corner_frequency(psd: &[(f64, f64)]) -> f64 {
    if psd.len() < 3 {
        return 0.0;
    }
    // Find the DC (lowest frequency, non-zero) power
    let s_dc = psd
        .iter()
        .find(|(f, _)| *f > 0.0)
        .map(|(_, s)| *s)
        .unwrap_or(0.0);

    if s_dc <= 0.0 {
        return 0.0;
    }

    let half_power = s_dc / 2.0;

    // Find where PSD drops below half power
    for window in psd.windows(2) {
        let (f1, s1) = window[0];
        let (f2, s2) = window[1];
        if f1 <= 0.0 {
            continue;
        }
        if s1 >= half_power && s2 < half_power {
            // Linear interpolation
            if (s1 - s2).abs() > 1.0e-30 {
                return f1 + (f2 - f1) * (s1 - half_power) / (s1 - s2);
            }
        }
    }

    // Fallback: return the highest frequency
    psd.last().map(|(f, _)| *f).unwrap_or(0.0)
}

/// Calibrate trap stiffness from corner frequency.
///
/// k = 2 pi gamma f_c
pub fn calibrate_from_corner_frequency(f_c: f64, gamma: f64) -> f64 {
    2.0 * PI * gamma * f_c
}

/// Drag force calibration: apply known velocity v, measure displacement x.
///
/// F_drag = gamma * v = k * x  =>  k = gamma * v / x
pub fn calibrate_drag_force(gamma: f64, velocity: f64, displacement: f64) -> f64 {
    if displacement.abs() < 1.0e-30 {
        return 0.0;
    }
    gamma * velocity / displacement
}

// ──────────────────────────────────────────────────────────
// Position detection (QPD simulation)
// ──────────────────────────────────────────────────────────

/// Quadrant photodiode signal for back-focal-plane interferometry.
///
/// Models the differential and sum signals from a QPD when the particle
/// is displaced from the trap center.
#[derive(Debug, Clone)]
pub struct QpdSignal {
    /// X-axis differential: (A+D) - (B+C), proportional to x-displacement.
    pub x_diff: f64,
    /// Y-axis differential: (A+B) - (C+D), proportional to y-displacement.
    pub y_diff: f64,
    /// Sum signal: A+B+C+D (total power).
    pub sum: f64,
    /// Normalized X signal: x_diff / sum.
    pub x_norm: f64,
    /// Normalized Y signal: y_diff / sum.
    pub y_norm: f64,
}

/// Simulate QPD signals for a given particle displacement.
///
/// The back-focal-plane interference pattern is modeled as a shifted
/// Gaussian distribution across four quadrants.
///
/// # Arguments
/// * `dx` - Particle displacement in x (meters)
/// * `dy` - Particle displacement in y (meters)
/// * `beam` - Gaussian beam parameters
/// * `sensitivity` - Detection sensitivity factor (V/m or arbitrary)
pub fn simulate_qpd(dx: f64, dy: f64, beam: &GaussianBeam, sensitivity: f64) -> QpdSignal {
    // Model: the spot shifts proportionally to particle displacement
    // QPD quadrants integrate the shifted Gaussian
    let w = beam.w0;

    // Approximate QPD response as linear for small displacements
    // and saturating for large displacements (erf-like)
    let x_response = (2.0 * dx / w) * (-dx * dx / (w * w)).exp();
    let y_response = (2.0 * dy / w) * (-dy * dy / (w * w)).exp();

    // Total power is roughly constant for small displacements
    let total_power = sensitivity * beam.i0 * PI * w * w / 2.0;

    let x_diff = sensitivity * x_response * total_power;
    let y_diff = sensitivity * y_response * total_power;
    let sum = total_power;

    let x_norm = if sum.abs() > 1.0e-30 {
        x_diff / sum
    } else {
        0.0
    };
    let y_norm = if sum.abs() > 1.0e-30 {
        y_diff / sum
    } else {
        0.0
    };

    QpdSignal {
        x_diff,
        y_diff,
        sum,
        x_norm,
        y_norm,
    }
}

/// Position calibration factor: convert QPD voltage to displacement.
///
/// beta (nm/V) = displacement / QPD_normalized_signal
pub fn position_calibration_factor(
    known_displacement_m: f64,
    qpd_normalized: f64,
) -> f64 {
    if qpd_normalized.abs() < 1.0e-30 {
        return 0.0;
    }
    known_displacement_m / qpd_normalized
}

// ──────────────────────────────────────────────────────────
// Force spectroscopy models
// ──────────────────────────────────────────────────────────

/// Worm-Like Chain (WLC) force-extension model.
///
/// F = (k_B T / L_p) * [1/(4(1 - x/L)^2) - 1/4 + x/L]
///
/// # Arguments
/// * `extension` - End-to-end distance (m)
/// * `contour_length` - Total contour length L (m)
/// * `persistence_length` - Persistence length L_p (m)
/// * `temperature_k` - Temperature in Kelvin
///
/// # Returns
/// Force in Newtons, or f64::INFINITY if extension >= contour_length.
pub fn wlc_force(
    extension: f64,
    contour_length: f64,
    persistence_length: f64,
    temperature_k: f64,
) -> f64 {
    if extension <= 0.0 {
        return 0.0;
    }
    let x_rel = extension / contour_length;
    if x_rel >= 1.0 {
        return f64::INFINITY;
    }
    let thermal_energy = K_BOLTZMANN * temperature_k;
    let t1 = 1.0 / (4.0 * (1.0 - x_rel).powi(2));
    let t2 = -0.25;
    let t3 = x_rel;
    (thermal_energy / persistence_length) * (t1 + t2 + t3)
}

/// Approximate inverse Langevin function.
///
/// L^{-1}(x) ~ x * (3 - x^2) / (1 - x^2)
///
/// Valid for |x| < 1.
pub fn inverse_langevin_approx(x: f64) -> f64 {
    if x.abs() >= 1.0 {
        return f64::INFINITY * x.signum();
    }
    x * (3.0 - x * x) / (1.0 - x * x)
}

/// Freely-Jointed Chain (FJC) force-extension model.
///
/// F = (k_B T / b) * L^{-1}(x / (N * b))
///
/// where b is the Kuhn length, N is the number of segments,
/// and L^{-1} is the inverse Langevin function.
///
/// # Arguments
/// * `extension` - End-to-end distance (m)
/// * `kuhn_length` - Kuhn length b (m)
/// * `num_segments` - Number of Kuhn segments N
/// * `temperature_k` - Temperature in Kelvin
pub fn fjc_force(
    extension: f64,
    kuhn_length: f64,
    num_segments: usize,
    temperature_k: f64,
) -> f64 {
    if extension <= 0.0 {
        return 0.0;
    }
    let max_extension = num_segments as f64 * kuhn_length;
    let x_rel = extension / max_extension;
    let thermal_energy = K_BOLTZMANN * temperature_k;
    (thermal_energy / kuhn_length) * inverse_langevin_approx(x_rel)
}

/// Simulate a constant-velocity pulling experiment.
///
/// Returns (extension, force) pairs.
///
/// # Arguments
/// * `velocity` - Pulling velocity (m/s)
/// * `trap_stiffness` - Trap stiffness (N/m)
/// * `contour_length` - Polymer contour length (m)
/// * `persistence_length` - Persistence length (m)
/// * `temperature_k` - Temperature (K)
/// * `dt` - Time step (s)
/// * `num_steps` - Number of time steps
pub fn simulate_pulling_wlc(
    velocity: f64,
    trap_stiffness: f64,
    contour_length: f64,
    persistence_length: f64,
    temperature_k: f64,
    dt: f64,
    num_steps: usize,
) -> Vec<(f64, f64)> {
    let mut results = Vec::with_capacity(num_steps);
    for i in 0..num_steps {
        let t = i as f64 * dt;
        let stage_position = velocity * t;
        // Assume bead tracks closely; extension ~ stage position (simplified)
        let ext = stage_position.min(contour_length * 0.999);
        let polymer_force = wlc_force(ext, contour_length, persistence_length, temperature_k);
        // Trap force = k * (stage_position - bead_position)
        // At equilibrium, trap force ~ polymer force
        let trap_force = if polymer_force.is_finite() {
            polymer_force
        } else {
            trap_stiffness * stage_position
        };
        results.push((ext, trap_force));
    }
    results
}

// ──────────────────────────────────────────────────────────
// Viscosity measurement
// ──────────────────────────────────────────────────────────

/// Estimate viscosity from the diffusion coefficient of a trapped particle.
///
/// From Einstein-Stokes relation: eta = k_B T / (6 pi r D)
///
/// # Arguments
/// * `diffusion_coeff` - Diffusion coefficient D (m^2/s)
/// * `particle_radius_m` - Particle radius (m)
/// * `temperature_k` - Temperature (K)
pub fn viscosity_from_diffusion(
    diffusion_coeff: f64,
    particle_radius_m: f64,
    temperature_k: f64,
) -> f64 {
    if diffusion_coeff <= 0.0 {
        return 0.0;
    }
    K_BOLTZMANN * temperature_k / (6.0 * PI * particle_radius_m * diffusion_coeff)
}

/// Estimate diffusion coefficient from a position time series using
/// Mean Squared Displacement: <(x(t+tau) - x(t))^2> = 2*D*tau (1D).
///
/// Uses the first lag point.
pub fn estimate_diffusion_msd(positions: &[f64], dt: f64) -> f64 {
    if positions.len() < 2 {
        return 0.0;
    }
    let msd: f64 = positions
        .windows(2)
        .map(|w| (w[1] - w[0]).powi(2))
        .sum::<f64>()
        / (positions.len() - 1) as f64;
    msd / (2.0 * dt)
}

// ──────────────────────────────────────────────────────────
// Boltzmann distribution analysis
// ──────────────────────────────────────────────────────────

/// Reconstruct the potential energy landscape U(x) from position histogram.
///
/// From Boltzmann statistics: P(x) ~ exp(-U(x) / (k_B T))
/// Therefore: U(x) = -k_B T * ln(P(x)) + const
///
/// # Arguments
/// * `positions` - Measured position time series (m)
/// * `num_bins` - Number of histogram bins
/// * `temperature_k` - Temperature (K)
///
/// # Returns
/// Vector of (position_center, potential_energy_J) pairs.
pub fn boltzmann_potential(
    positions: &[f64],
    num_bins: usize,
    temperature_k: f64,
) -> Vec<(f64, f64)> {
    if positions.is_empty() || num_bins == 0 {
        return vec![];
    }

    let mut min_val = f64::INFINITY;
    let mut max_val = f64::NEG_INFINITY;
    for &x in positions {
        if x < min_val {
            min_val = x;
        }
        if x > max_val {
            max_val = x;
        }
    }

    if (max_val - min_val).abs() < 1.0e-30 {
        return vec![];
    }

    let bin_width = (max_val - min_val) / num_bins as f64;
    let mut histogram = vec![0usize; num_bins];

    for &x in positions {
        let bin = ((x - min_val) / bin_width).floor() as usize;
        let bin = bin.min(num_bins - 1);
        histogram[bin] += 1;
    }

    let kt = K_BOLTZMANN * temperature_k;
    let max_count = *histogram.iter().max().unwrap_or(&1) as f64;

    let mut result = Vec::new();
    for (i, &count) in histogram.iter().enumerate() {
        if count > 0 {
            let x_center = min_val + (i as f64 + 0.5) * bin_width;
            // U(x) = -k_B T * ln(P(x)), normalized so minimum U = 0
            let prob = count as f64 / max_count;
            let potential = -kt * prob.ln();
            result.push((x_center, potential));
        }
    }

    // Shift so minimum potential is zero
    let u_min = result
        .iter()
        .map(|(_, u)| *u)
        .fold(f64::INFINITY, f64::min);
    for entry in &mut result {
        entry.1 -= u_min;
    }

    result
}

/// Fit a harmonic potential U(x) = 0.5 * k * x^2 to the reconstructed potential.
///
/// Uses least-squares fitting of parabola to (position, potential) data.
///
/// # Returns
/// Estimated trap stiffness k (N/m).
pub fn fit_harmonic_potential(potential: &[(f64, f64)]) -> f64 {
    if potential.len() < 3 {
        return 0.0;
    }

    // Fit U = 0.5 * k * x^2 => k = 2 * sum(x^2 * U) / sum(x^4)
    // where x is relative to the minimum
    let x_min = potential
        .iter()
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(x, _)| *x)
        .unwrap_or(0.0);

    let mut sum_x2u = 0.0;
    let mut sum_x4 = 0.0;
    for &(x, u) in potential {
        let dx = x - x_min;
        sum_x2u += dx * dx * u;
        sum_x4 += dx.powi(4);
    }

    if sum_x4 > 1.0e-60 {
        2.0 * sum_x2u / sum_x4
    } else {
        0.0
    }
}

// ──────────────────────────────────────────────────────────
// Statistics helpers
// ──────────────────────────────────────────────────────────

/// Compute mean of a slice.
pub fn mean(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    data.iter().sum::<f64>() / data.len() as f64
}

/// Compute variance of a slice.
pub fn variance(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    let m = mean(data);
    data.iter().map(|x| (x - m).powi(2)).sum::<f64>() / data.len() as f64
}

/// Compute standard deviation.
pub fn std_dev(data: &[f64]) -> f64 {
    variance(data).sqrt()
}

// ──────────────────────────────────────────────────────────
// Tests
// ──────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1.0e-6;

    fn approx_eq(a: f64, b: f64, rel_tol: f64) -> bool {
        if a == b {
            return true;
        }
        let denom = a.abs().max(b.abs());
        if denom < 1.0e-30 {
            return (a - b).abs() < 1.0e-30;
        }
        (a - b).abs() / denom < rel_tol
    }

    // ── TrapConfig tests ──

    #[test]
    fn test_default_config() {
        let cfg = TrapConfig::default_config();
        assert!((cfg.wavelength_nm - 1064.0).abs() < TOL);
        assert!((cfg.numerical_aperture - 1.25).abs() < TOL);
        assert!((cfg.laser_power_mw - 100.0).abs() < TOL);
        assert!((cfg.medium_refractive_index - 1.33).abs() < TOL);
        assert!((cfg.particle_radius_um - 0.5).abs() < TOL);
        assert!((cfg.particle_refractive_index - 1.59).abs() < TOL);
    }

    #[test]
    fn test_unit_conversions() {
        let cfg = TrapConfig::default_config();
        assert!(approx_eq(cfg.wavelength_m(), 1.064e-6, 1e-6));
        assert!(approx_eq(cfg.radius_m(), 0.5e-6, 1e-6));
        assert!(approx_eq(cfg.power_w(), 0.1, 1e-6));
    }

    #[test]
    fn test_relative_index() {
        let cfg = TrapConfig::default_config();
        let m = cfg.relative_index();
        assert!(approx_eq(m, 1.59 / 1.33, 1e-6));
    }

    #[test]
    fn test_clausius_mossotti_positive() {
        let cfg = TrapConfig::default_config();
        let cm = cfg.clausius_mossotti();
        // For n_p > n_m, CM factor should be positive
        assert!(cm > 0.0);
        assert!(cm < 1.0);
    }

    #[test]
    fn test_polarizability_positive() {
        let cfg = TrapConfig::default_config();
        let alpha = cfg.polarizability();
        assert!(alpha > 0.0, "Polarizability should be positive for n_p > n_m");
    }

    #[test]
    fn test_scattering_cross_section_positive() {
        // Use a small particle deep in the Rayleigh regime (r << lambda)
        let mut cfg = TrapConfig::default_config();
        cfg.particle_radius_um = 0.05; // 50 nm, well below 1064 nm wavelength
        let sigma = cfg.scattering_cross_section();
        assert!(sigma > 0.0);
        // In Rayleigh regime, sigma << geometric cross-section
        let geo = PI * cfg.radius_m().powi(2);
        assert!(
            sigma < geo,
            "Rayleigh scattering cross-section ({:e}) should be << geometric ({:e})",
            sigma,
            geo
        );
    }

    #[test]
    fn test_scattering_scales_with_r6() {
        let mut cfg = TrapConfig::default_config();
        let sigma1 = cfg.scattering_cross_section();
        cfg.particle_radius_um *= 2.0;
        let sigma2 = cfg.scattering_cross_section();
        // sigma ~ r^6, so doubling r should give 64x
        assert!(approx_eq(sigma2 / sigma1, 64.0, 0.01));
    }

    // ── Gaussian beam tests ──

    #[test]
    fn test_beam_waist() {
        let cfg = TrapConfig::default_config();
        let beam = GaussianBeam::from_config(&cfg);
        // w0 = lambda / (pi * NA) = 1.064e-6 / (pi * 1.25) ~ 2.71e-7
        let expected = cfg.wavelength_m() / (PI * cfg.numerical_aperture);
        assert!(approx_eq(beam.w0, expected, 1e-6));
    }

    #[test]
    fn test_rayleigh_range() {
        let cfg = TrapConfig::default_config();
        let beam = GaussianBeam::from_config(&cfg);
        let expected = PI * beam.w0 * beam.w0 / cfg.wavelength_m();
        assert!(approx_eq(beam.z_r, expected, 1e-6));
    }

    #[test]
    fn test_intensity_at_focus() {
        let cfg = TrapConfig::default_config();
        let beam = GaussianBeam::from_config(&cfg);
        // At r=0, z=0: I = I0
        let i = beam.intensity(0.0, 0.0);
        assert!(approx_eq(i, beam.i0, 1e-6));
    }

    #[test]
    fn test_intensity_decays_radially() {
        let cfg = TrapConfig::default_config();
        let beam = GaussianBeam::from_config(&cfg);
        let i_center = beam.intensity(0.0, 0.0);
        let i_w0 = beam.intensity(beam.w0, 0.0);
        // At r = w0: I = I0 * exp(-2)
        assert!(approx_eq(i_w0 / i_center, (-2.0_f64).exp(), 1e-4));
    }

    #[test]
    fn test_intensity_decays_axially() {
        let cfg = TrapConfig::default_config();
        let beam = GaussianBeam::from_config(&cfg);
        let i_focus = beam.intensity(0.0, 0.0);
        let i_zr = beam.intensity(0.0, beam.z_r);
        // At z = z_R: I = I0 / 2
        assert!(approx_eq(i_zr / i_focus, 0.5, 1e-4));
    }

    #[test]
    fn test_beam_radius_at_rayleigh() {
        let cfg = TrapConfig::default_config();
        let beam = GaussianBeam::from_config(&cfg);
        let w_at_zr = beam.beam_radius(beam.z_r);
        // w(z_R) = w0 * sqrt(2)
        assert!(approx_eq(w_at_zr, beam.w0 * 2.0_f64.sqrt(), 1e-6));
    }

    #[test]
    fn test_radial_gradient_restoring() {
        let cfg = TrapConfig::default_config();
        let beam = GaussianBeam::from_config(&cfg);
        // Gradient should be negative for positive displacement (restoring)
        let grad = beam.intensity_gradient_r(beam.w0 * 0.1, 0.0);
        assert!(grad < 0.0, "Radial gradient should point inward");
    }

    // ── Force calculation tests ──

    #[test]
    fn test_gradient_force_restoring() {
        let cfg = TrapConfig::default_config();
        let beam = GaussianBeam::from_config(&cfg);
        let f = gradient_force_radial(&cfg, &beam, beam.w0 * 0.1, 0.0);
        // Force should be negative (toward center) for positive displacement
        assert!(
            f < 0.0,
            "Gradient force should be restoring (toward center)"
        );
    }

    #[test]
    fn test_scattering_force_positive() {
        let cfg = TrapConfig::default_config();
        let beam = GaussianBeam::from_config(&cfg);
        let f = scattering_force(&cfg, &beam, 0.0, 0.0);
        assert!(
            f > 0.0,
            "Scattering force should push along propagation direction"
        );
    }

    #[test]
    fn test_trap_stiffness_positive() {
        let cfg = TrapConfig::default_config();
        let beam = GaussianBeam::from_config(&cfg);
        let k_r = trap_stiffness_radial(&cfg, &beam);
        assert!(k_r > 0.0, "Trap stiffness should be positive");
    }

    #[test]
    fn test_radial_stiffer_than_axial() {
        let cfg = TrapConfig::default_config();
        let beam = GaussianBeam::from_config(&cfg);
        let k_r = trap_stiffness_radial(&cfg, &beam);
        let k_z = trap_stiffness_axial(&cfg, &beam);
        // Radial stiffness is typically stronger than axial
        assert!(
            k_r > k_z,
            "Radial stiffness ({}) should exceed axial ({})",
            k_r,
            k_z
        );
    }

    #[test]
    fn test_trapping_efficiency_positive() {
        let cfg = TrapConfig::default_config();
        let beam = GaussianBeam::from_config(&cfg);
        let eff = trapping_efficiency(&cfg, &beam);
        assert!(eff > 0.0);
    }

    // ── Brownian motion tests ──

    #[test]
    fn test_drag_coefficient() {
        let bc = BrownianConfig {
            temperature_k: 293.0,
            viscosity_pa_s: 0.001,
            trap_stiffness: 1.0e-6,
            particle_radius_m: 0.5e-6,
            dt: 1.0e-4,
        };
        let gamma = bc.drag_coefficient();
        let expected = 6.0 * PI * 0.001 * 0.5e-6;
        assert!(approx_eq(gamma, expected, 1e-6));
    }

    #[test]
    fn test_diffusion_coefficient() {
        let bc = BrownianConfig {
            temperature_k: 293.0,
            viscosity_pa_s: 0.001,
            trap_stiffness: 1.0e-6,
            particle_radius_m: 0.5e-6,
            dt: 1.0e-4,
        };
        let d = bc.diffusion_coefficient();
        assert!(d > 0.0);
        // D = k_B T / gamma, should be on order of 1e-12 m^2/s for 1um bead
        assert!(d > 1.0e-14 && d < 1.0e-10);
    }

    #[test]
    fn test_corner_frequency() {
        let bc = BrownianConfig {
            temperature_k: 293.0,
            viscosity_pa_s: 0.001,
            trap_stiffness: 1.0e-5,
            particle_radius_m: 0.5e-6,
            dt: 1.0e-4,
        };
        let fc = bc.corner_frequency();
        assert!(fc > 0.0);
        // fc = k / (2 pi gamma)
        let expected = bc.trap_stiffness / (2.0 * PI * bc.drag_coefficient());
        assert!(approx_eq(fc, expected, 1e-6));
    }

    #[test]
    fn test_thermal_variance() {
        let bc = BrownianConfig {
            temperature_k: 293.0,
            viscosity_pa_s: 0.001,
            trap_stiffness: 1.0e-5,
            particle_radius_m: 0.5e-6,
            dt: 1.0e-4,
        };
        let var = bc.thermal_variance();
        let expected = K_BOLTZMANN * 293.0 / 1.0e-5;
        assert!(approx_eq(var, expected, 1e-6));
    }

    #[test]
    fn test_brownian_simulation_length() {
        let bc = BrownianConfig {
            temperature_k: 293.0,
            viscosity_pa_s: 0.001,
            trap_stiffness: 1.0e-5,
            particle_radius_m: 0.5e-6,
            dt: 1.0e-4,
        };
        let pos = simulate_brownian_1d(&bc, 1000, 42);
        assert_eq!(pos.len(), 1000);
    }

    #[test]
    fn test_brownian_starts_at_zero() {
        let bc = BrownianConfig {
            temperature_k: 293.0,
            viscosity_pa_s: 0.001,
            trap_stiffness: 1.0e-5,
            particle_radius_m: 0.5e-6,
            dt: 1.0e-4,
        };
        let pos = simulate_brownian_1d(&bc, 100, 42);
        assert!((pos[0]).abs() < 1.0e-30);
    }

    #[test]
    fn test_brownian_variance_consistent() {
        let k = 1.0e-5;
        let t = 293.0;
        let bc = BrownianConfig {
            temperature_k: t,
            viscosity_pa_s: 0.001,
            trap_stiffness: k,
            particle_radius_m: 0.5e-6,
            dt: 1.0e-5,
        };
        // Long simulation for statistical convergence
        let pos = simulate_brownian_1d(&bc, 500_000, 123);
        let var = variance(&pos);
        let expected_var = K_BOLTZMANN * t / k;
        // Allow 20% tolerance for statistical fluctuation
        assert!(
            approx_eq(var, expected_var, 0.2),
            "Variance {} should be close to theoretical {} (within 20%)",
            var,
            expected_var
        );
    }

    #[test]
    fn test_brownian_2d_length() {
        let bc = BrownianConfig {
            temperature_k: 293.0,
            viscosity_pa_s: 0.001,
            trap_stiffness: 1.0e-5,
            particle_radius_m: 0.5e-6,
            dt: 1.0e-4,
        };
        let pos = simulate_brownian_2d(&bc, 500, 99);
        assert_eq!(pos.len(), 500);
    }

    #[test]
    fn test_brownian_deterministic_with_seed() {
        let bc = BrownianConfig {
            temperature_k: 293.0,
            viscosity_pa_s: 0.001,
            trap_stiffness: 1.0e-5,
            particle_radius_m: 0.5e-6,
            dt: 1.0e-4,
        };
        let pos1 = simulate_brownian_1d(&bc, 100, 42);
        let pos2 = simulate_brownian_1d(&bc, 100, 42);
        for (a, b) in pos1.iter().zip(pos2.iter()) {
            assert_eq!(*a, *b, "Same seed should produce same trajectory");
        }
    }

    // ── Calibration tests ──

    #[test]
    fn test_equipartition_calibration() {
        let k = 1.0e-5;
        let t = 293.0;
        let bc = BrownianConfig {
            temperature_k: t,
            viscosity_pa_s: 0.001,
            trap_stiffness: k,
            particle_radius_m: 0.5e-6,
            dt: 1.0e-5,
        };
        let pos = simulate_brownian_1d(&bc, 500_000, 456);
        let k_est = calibrate_equipartition(&pos, t);
        assert!(
            approx_eq(k_est, k, 0.2),
            "Equipartition k={} should be close to true k={}",
            k_est,
            k
        );
    }

    #[test]
    fn test_equipartition_empty() {
        let k = calibrate_equipartition(&[], 293.0);
        assert!((k).abs() < TOL);
    }

    #[test]
    fn test_calibrate_from_corner_freq() {
        let gamma = 6.0 * PI * 0.001 * 0.5e-6;
        let k_true = 1.0e-5;
        let fc = k_true / (2.0 * PI * gamma);
        let k_est = calibrate_from_corner_frequency(fc, gamma);
        assert!(approx_eq(k_est, k_true, 1e-6));
    }

    #[test]
    fn test_drag_force_calibration() {
        let gamma = 6.0 * PI * 0.001 * 0.5e-6;
        let k_true = 1.0e-5;
        let v = 100.0e-6; // 100 um/s
        let disp = gamma * v / k_true; // expected displacement
        let k_est = calibrate_drag_force(gamma, v, disp);
        assert!(approx_eq(k_est, k_true, 1e-6));
    }

    #[test]
    fn test_drag_force_zero_displacement() {
        let k = calibrate_drag_force(1.0e-8, 1.0e-4, 0.0);
        assert!((k).abs() < TOL);
    }

    // ── PSD and corner frequency tests ──

    #[test]
    fn test_lorentzian_psd_dc() {
        let s0 = 1.0e-20;
        let fc = 100.0;
        let s_dc = lorentzian_psd(0.0, s0, fc);
        assert!(approx_eq(s_dc, s0 / (fc * fc), 1e-6));
    }

    #[test]
    fn test_lorentzian_psd_half_power() {
        let s0 = 1.0e-20;
        let fc = 100.0;
        let s_at_fc = lorentzian_psd(fc, s0, fc);
        let s_dc = lorentzian_psd(0.0, s0, fc);
        assert!(approx_eq(s_at_fc, s_dc / 2.0, 1e-6));
    }

    #[test]
    fn test_compute_psd_length() {
        let positions: Vec<f64> = (0..128).map(|i| (i as f64 * 0.01).sin()).collect();
        let psd = compute_psd(&positions, 1000.0);
        assert_eq!(psd.len(), 65); // N/2 + 1
    }

    #[test]
    fn test_fit_corner_frequency_from_lorentzian() {
        let fc_true = 200.0;
        let s0 = 1.0e-20;
        let sample_rate = 10000.0;
        let n = 1000;
        // Generate synthetic Lorentzian PSD
        let psd: Vec<(f64, f64)> = (0..=n / 2)
            .map(|k| {
                let f = k as f64 * sample_rate / n as f64;
                (f, lorentzian_psd(f, s0, fc_true))
            })
            .collect();
        let fc_est = fit_corner_frequency(&psd);
        assert!(
            approx_eq(fc_est, fc_true, 0.05),
            "Estimated fc={} should be close to true fc={}",
            fc_est,
            fc_true
        );
    }

    // ── QPD tests ──

    #[test]
    fn test_qpd_zero_displacement() {
        let cfg = TrapConfig::default_config();
        let beam = GaussianBeam::from_config(&cfg);
        let qpd = simulate_qpd(0.0, 0.0, &beam, 1.0);
        assert!((qpd.x_norm).abs() < 1e-10);
        assert!((qpd.y_norm).abs() < 1e-10);
    }

    #[test]
    fn test_qpd_x_displacement() {
        let cfg = TrapConfig::default_config();
        let beam = GaussianBeam::from_config(&cfg);
        let dx = beam.w0 * 0.1;
        let qpd = simulate_qpd(dx, 0.0, &beam, 1.0);
        assert!(qpd.x_norm > 0.0, "Positive x displacement -> positive x signal");
        assert!(
            qpd.y_norm.abs() < 1e-10,
            "No y displacement -> zero y signal"
        );
    }

    #[test]
    fn test_qpd_antisymmetric() {
        let cfg = TrapConfig::default_config();
        let beam = GaussianBeam::from_config(&cfg);
        let dx = beam.w0 * 0.05;
        let qpd_pos = simulate_qpd(dx, 0.0, &beam, 1.0);
        let qpd_neg = simulate_qpd(-dx, 0.0, &beam, 1.0);
        assert!(
            approx_eq(qpd_pos.x_norm, -qpd_neg.x_norm, 1e-6),
            "QPD should be antisymmetric"
        );
    }

    #[test]
    fn test_position_calibration_factor() {
        let known_disp = 100.0e-9; // 100 nm
        let qpd_signal = 0.5; // 0.5 V normalized
        let beta = position_calibration_factor(known_disp, qpd_signal);
        assert!(approx_eq(beta, 200.0e-9, 1e-6));
    }

    // ── Force spectroscopy tests ──

    #[test]
    fn test_wlc_zero_extension() {
        let f = wlc_force(0.0, 1.0e-6, 50.0e-9, 293.0);
        assert!((f).abs() < TOL);
    }

    #[test]
    fn test_wlc_diverges_at_contour_length() {
        let f = wlc_force(0.999e-6, 1.0e-6, 50.0e-9, 293.0);
        assert!(f > 1.0e-9, "WLC force should be very large near contour length");
    }

    #[test]
    fn test_wlc_at_contour_length_infinite() {
        let f = wlc_force(1.0e-6, 1.0e-6, 50.0e-9, 293.0);
        assert!(f.is_infinite());
    }

    #[test]
    fn test_wlc_monotonically_increasing() {
        let lc = 1.0e-6;
        let lp = 50.0e-9;
        let t = 293.0;
        let extensions: Vec<f64> = (1..10).map(|i| i as f64 * lc * 0.1).collect();
        for w in extensions.windows(2) {
            let f1 = wlc_force(w[0], lc, lp, t);
            let f2 = wlc_force(w[1], lc, lp, t);
            assert!(
                f2 >= f1,
                "WLC should be monotonically increasing: f({})={} < f({})={}",
                w[0],
                f1,
                w[1],
                f2
            );
        }
    }

    #[test]
    fn test_inverse_langevin_small_x() {
        // For small x, L^{-1}(x) ~ 3x
        let x = 0.01;
        let result = inverse_langevin_approx(x);
        assert!(approx_eq(result, 3.0 * x, 0.01));
    }

    #[test]
    fn test_inverse_langevin_diverges() {
        let result = inverse_langevin_approx(1.0);
        assert!(result.is_infinite());
    }

    #[test]
    fn test_fjc_force_positive() {
        let f = fjc_force(50.0e-9, 0.34e-9, 1000, 293.0);
        assert!(f > 0.0);
    }

    #[test]
    fn test_fjc_zero_extension() {
        let f = fjc_force(0.0, 0.34e-9, 1000, 293.0);
        assert!((f).abs() < TOL);
    }

    #[test]
    fn test_pulling_simulation_length() {
        let result = simulate_pulling_wlc(1.0e-6, 1.0e-5, 1.0e-6, 50.0e-9, 293.0, 1.0e-3, 500);
        assert_eq!(result.len(), 500);
    }

    #[test]
    fn test_pulling_force_increases() {
        let result =
            simulate_pulling_wlc(1.0e-6, 1.0e-5, 1.0e-6, 50.0e-9, 293.0, 1.0e-4, 100);
        // Force should generally increase with extension
        let f_early = result[10].1;
        let f_late = result[90].1;
        assert!(f_late > f_early, "Force should increase during pulling");
    }

    // ── Viscosity measurement tests ──

    #[test]
    fn test_viscosity_from_diffusion() {
        let r = 0.5e-6;
        let t = 293.0;
        let eta_true = 0.001; // water
        let gamma = 6.0 * PI * eta_true * r;
        let d = K_BOLTZMANN * t / gamma;
        let eta_est = viscosity_from_diffusion(d, r, t);
        assert!(approx_eq(eta_est, eta_true, 1e-6));
    }

    #[test]
    fn test_viscosity_zero_diffusion() {
        let eta = viscosity_from_diffusion(0.0, 0.5e-6, 293.0);
        assert!((eta).abs() < TOL);
    }

    #[test]
    fn test_estimate_diffusion_msd() {
        // For free diffusion (no trap), MSD = 2*D*dt per step (1D)
        // For trapped particle, short-time MSD ~ 2*D*dt
        let bc = BrownianConfig {
            temperature_k: 293.0,
            viscosity_pa_s: 0.001,
            trap_stiffness: 1.0e-5,
            particle_radius_m: 0.5e-6,
            dt: 1.0e-5,
        };
        let pos = simulate_brownian_1d(&bc, 200_000, 789);
        let d_est = estimate_diffusion_msd(&pos, bc.dt);
        let d_true = bc.diffusion_coefficient();
        // MSD-based estimate includes trap restoring force effects
        // so it will underestimate free diffusion D, but should be same order
        assert!(
            d_est > 0.0 && d_est < d_true * 2.0,
            "D_est={} should be reasonable vs D_true={}",
            d_est,
            d_true
        );
    }

    // ── Boltzmann potential tests ──

    #[test]
    fn test_boltzmann_potential_nonempty() {
        let bc = BrownianConfig {
            temperature_k: 293.0,
            viscosity_pa_s: 0.001,
            trap_stiffness: 1.0e-5,
            particle_radius_m: 0.5e-6,
            dt: 1.0e-5,
        };
        let pos = simulate_brownian_1d(&bc, 100_000, 321);
        let potential = boltzmann_potential(&pos, 50, bc.temperature_k);
        assert!(!potential.is_empty());
    }

    #[test]
    fn test_boltzmann_potential_minimum_at_zero() {
        let bc = BrownianConfig {
            temperature_k: 293.0,
            viscosity_pa_s: 0.001,
            trap_stiffness: 1.0e-5,
            particle_radius_m: 0.5e-6,
            dt: 1.0e-5,
        };
        let pos = simulate_brownian_1d(&bc, 200_000, 654);
        let potential = boltzmann_potential(&pos, 50, bc.temperature_k);
        // Find minimum potential position
        let (x_min, _) = potential
            .iter()
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();
        // Minimum should be near zero (trap center)
        let x_range = potential.last().unwrap().0 - potential.first().unwrap().0;
        assert!(
            x_min.abs() < x_range * 0.25,
            "Potential minimum at x={} should be near center",
            x_min
        );
    }

    #[test]
    fn test_boltzmann_empty_input() {
        let result = boltzmann_potential(&[], 50, 293.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_fit_harmonic_potential() {
        // Generate a perfect parabola: U = 0.5 * k * x^2
        let k_true = 1.0e-5;
        let potential: Vec<(f64, f64)> = (-50..=50)
            .map(|i| {
                let x = i as f64 * 1.0e-8;
                (x, 0.5 * k_true * x * x)
            })
            .collect();
        let k_est = fit_harmonic_potential(&potential);
        assert!(
            approx_eq(k_est, k_true, 0.01),
            "Fitted k={} should match true k={}",
            k_est,
            k_true
        );
    }

    // ── Statistics tests ──

    #[test]
    fn test_mean_simple() {
        let data = [1.0, 2.0, 3.0, 4.0, 5.0];
        assert!(approx_eq(mean(&data), 3.0, 1e-10));
    }

    #[test]
    fn test_variance_simple() {
        let data = [2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0];
        let v = variance(&data);
        assert!(approx_eq(v, 4.0, 1e-10));
    }

    #[test]
    fn test_std_dev() {
        let data = [2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0];
        let s = std_dev(&data);
        assert!(approx_eq(s, 2.0, 1e-10));
    }
}
