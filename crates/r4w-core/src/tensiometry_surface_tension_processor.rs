//! # Tensiometry Surface Tension Processor
//!
//! Surface tension measurement and analysis processor implementing multiple
//! classical and modern tensiometric methods.
//!
//! ## Methods Implemented
//!
//! - **Du Nouy Ring Method**: Ring detachment force measurement with Harkins-Jordan correction
//! - **Wilhelmy Plate Method**: Plate wetting force measurement with contact angle awareness
//! - **Pendant Drop Analysis**: Young-Laplace ODE integration for drop shape fitting
//! - **Maximum Bubble Pressure**: Dynamic surface tension from bubble lifetime/pressure curves
//! - **Capillary Rise (Jurin's Law)**: Capillary height to surface tension conversion
//! - **Gibbs Adsorption Isotherm**: Surface excess from d-gamma/d(ln c)
//! - **CMC Determination**: Critical micelle concentration from isotherm breakpoints
//! - **Szyszkowski Model**: Surfactant gamma(c) empirical modeling
//! - **Langmuir Adsorption**: Langmuir-Szyszkowski isotherm for surface coverage
//!
//! ## Physical Constants
//!
//! | Substance | Temperature | Surface Tension (mN/m) |
//! |-----------|-------------|----------------------|
//! | Water     | 20 deg C    | 72.75                |
//! | Water     | 25 deg C    | 71.97                |
//! | Ethanol   | 20 deg C    | 22.10                |
//!
//! ## Key Equations
//!
//! - **Du Nouy**: gamma = F / (4 * pi * R * f), where f is the Harkins-Jordan correction
//! - **Wilhelmy**: gamma = F / (L * cos(theta))
//! - **Jurin's Law**: h = 2 * gamma * cos(theta) / (rho * g * r)
//! - **Young-Laplace**: Delta_P = gamma * (1/R1 + 1/R2)
//! - **Gibbs Adsorption**: Gamma = -(1/RT) * d(gamma)/d(ln c)
//! - **Szyszkowski**: gamma_0 - gamma = a * ln(1 + c/b)
//! - **Dupre-Young**: W_a = gamma_L * (1 + cos(theta))

use std::f64::consts::PI;

/// Gravitational acceleration (m/s^2)
const G: f64 = 9.80665;

/// Gas constant (J/(mol*K))
const R_GAS: f64 = 8.314462;

/// Water surface tension at 20 deg C (mN/m)
pub const GAMMA_WATER_20C: f64 = 72.75;

/// Water surface tension at 25 deg C (mN/m)
pub const GAMMA_WATER_25C: f64 = 71.97;

/// Ethanol surface tension at 20 deg C (mN/m)
pub const GAMMA_ETHANOL_20C: f64 = 22.10;

/// Typical CMC for SDS in mM
pub const CMC_SDS_MM: f64 = 8.0;

/// Typical minimum surface tension for SDS at CMC (mN/m)
pub const GAMMA_MIN_SDS: f64 = 33.0;

// ---------------------------------------------------------------------------
// Du Nouy Ring Method
// ---------------------------------------------------------------------------

/// Du Nouy ring tensiometer for surface tension measurement.
///
/// The du Nouy ring method measures the force required to detach a ring
/// from the liquid surface. The Harkins-Jordan correction factor accounts
/// for the non-ideal meniscus shape.
///
/// gamma = F / (4 * pi * R) * correction_factor
#[derive(Debug, Clone)]
pub struct DuNouyRing {
    /// Ring radius in meters
    pub ring_radius: f64,
    /// Wire radius of the ring in meters
    pub wire_radius: f64,
}

impl DuNouyRing {
    /// Create a new du Nouy ring with given ring radius and wire radius.
    pub fn new(ring_radius: f64, wire_radius: f64) -> Self {
        Self {
            ring_radius,
            wire_radius,
        }
    }

    /// Standard platinum-iridium ring (R = 9.545 mm, wire r = 0.185 mm).
    pub fn standard() -> Self {
        Self {
            ring_radius: 9.545e-3,
            wire_radius: 0.185e-3,
        }
    }

    /// Compute Harkins-Jordan correction factor.
    ///
    /// The correction factor depends on the ratio R^3/V and R/r where V is
    /// the volume of liquid raised. A simplified empirical approximation is
    /// used here based on the ratio R/r.
    pub fn harkins_jordan_correction(&self, density_diff: f64, gamma_apparent: f64) -> f64 {
        // Dimensionless parameter: R^3 / V approximated via Zuidema-Waters
        let r_ratio = self.ring_radius / self.wire_radius;
        let cap_length = (gamma_apparent * 1e-3 / (density_diff * G)).sqrt();
        let p = self.ring_radius / cap_length;

        // Empirical Harkins-Jordan correction (Zuidema-Waters form)
        // f = 0.7250 + sqrt(0.01452 * gamma / (pi^2 * R^2 * density_diff * g) + 0.04534 - 1.679 * r/R)
        let term1 = 0.01452 * gamma_apparent * 1e-3
            / (PI * PI * self.ring_radius * self.ring_radius * density_diff * G);
        let term2 = 0.04534 - 1.679 * self.wire_radius / self.ring_radius;

        let inner = term1 + term2;
        if inner > 0.0 {
            0.7250 + inner.sqrt()
        } else {
            // Fallback: use simplified correction based on R/r ratio
            // For large R/r ratios, correction approaches 1.0
            1.0 - 0.01 * (50.0 / r_ratio).powi(2).min(0.1)
        }
    }

    /// Compute surface tension from detachment force.
    ///
    /// # Arguments
    /// * `force` - Detachment force in Newtons
    /// * `correction_factor` - Harkins-Jordan correction factor (typically 0.9-1.05)
    ///
    /// # Returns
    /// Surface tension in mN/m
    pub fn compute_tension(&self, force: f64, correction_factor: f64) -> f64 {
        // gamma = F / (4 * pi * R) * correction
        // Convert to mN/m
        (force / (4.0 * PI * self.ring_radius)) * correction_factor * 1000.0
    }

    /// Compute surface tension with automatic Harkins-Jordan correction.
    ///
    /// Uses iterative correction: start with uncorrected value, compute
    /// correction, then refine.
    pub fn compute_tension_corrected(&self, force: f64, density_diff: f64) -> f64 {
        // Initial uncorrected value
        let mut gamma = self.compute_tension(force, 1.0);

        // Iterate correction
        for _ in 0..10 {
            let f = self.harkins_jordan_correction(density_diff, gamma);
            gamma = self.compute_tension(force, f);
        }
        gamma
    }
}

/// Compute surface tension from du Nouy ring detachment force.
///
/// # Arguments
/// * `force` - Detachment force in Newtons
/// * `ring_radius` - Ring radius in meters
/// * `correction_factor` - Harkins-Jordan correction factor
///
/// # Returns
/// Surface tension in mN/m
pub fn du_nouy_tension(force: f64, ring_radius: f64, correction_factor: f64) -> f64 {
    (force / (4.0 * PI * ring_radius)) * correction_factor * 1000.0
}

// ---------------------------------------------------------------------------
// Wilhelmy Plate Method
// ---------------------------------------------------------------------------

/// Wilhelmy plate tensiometer for surface tension measurement.
///
/// The Wilhelmy plate method measures the wetting force on a thin plate
/// partially immersed in the liquid. For a perfectly wetting plate
/// (contact angle = 0), gamma = F / perimeter.
#[derive(Debug, Clone)]
pub struct WilhelmyPlate {
    /// Plate width in meters
    pub width: f64,
    /// Plate thickness in meters
    pub thickness: f64,
}

impl WilhelmyPlate {
    /// Create a new Wilhelmy plate with given dimensions.
    pub fn new(width: f64, thickness: f64) -> Self {
        Self { width, thickness }
    }

    /// Standard platinum plate (width = 19.9 mm, thickness = 0.1 mm).
    pub fn standard_platinum() -> Self {
        Self {
            width: 19.9e-3,
            thickness: 0.1e-3,
        }
    }

    /// Standard filter paper plate (width = 20 mm, thickness = 0.15 mm).
    pub fn standard_paper() -> Self {
        Self {
            width: 20.0e-3,
            thickness: 0.15e-3,
        }
    }

    /// Compute the wetted perimeter.
    pub fn perimeter(&self) -> f64 {
        2.0 * (self.width + self.thickness)
    }

    /// Compute surface tension from wetting force.
    ///
    /// # Arguments
    /// * `force` - Wetting force in Newtons
    /// * `contact_angle_rad` - Contact angle in radians (0 for perfectly wetting)
    ///
    /// # Returns
    /// Surface tension in mN/m
    pub fn compute_tension(&self, force: f64, contact_angle_rad: f64) -> f64 {
        let p = self.perimeter();
        (force / (p * contact_angle_rad.cos())) * 1000.0
    }
}

/// Compute surface tension from Wilhelmy plate measurement.
///
/// # Arguments
/// * `force` - Wetting force in Newtons
/// * `perimeter` - Wetted perimeter in meters
/// * `contact_angle` - Contact angle in radians
///
/// # Returns
/// Surface tension in mN/m
pub fn wilhelmy_tension(force: f64, perimeter: f64, contact_angle: f64) -> f64 {
    (force / (perimeter * contact_angle.cos())) * 1000.0
}

// ---------------------------------------------------------------------------
// Pendant Drop Analysis (Young-Laplace)
// ---------------------------------------------------------------------------

/// Pendant drop analyzer using Young-Laplace equation fitting.
///
/// Numerically integrates the Young-Laplace ODE for axisymmetric drops
/// to determine surface tension from drop shape parameters.
///
/// The ODE system for the drop profile (s = arc length):
///   dx/ds = cos(phi)
///   dz/ds = sin(phi)
///   dphi/ds = 2/b - Delta_rho * g * z / gamma - sin(phi)/x
///
/// where b is the radius of curvature at the apex.
#[derive(Debug, Clone)]
pub struct PendantDrop {
    /// Density difference (rho_liquid - rho_vapor) in kg/m^3
    pub density_diff: f64,
    /// Apex radius of curvature in meters
    pub apex_radius: f64,
}

impl PendantDrop {
    /// Create a new pendant drop analyzer.
    pub fn new(density_diff: f64, apex_radius: f64) -> Self {
        Self {
            density_diff,
            apex_radius,
        }
    }

    /// Compute the Bond number (Bo = Delta_rho * g * b^2 / gamma).
    pub fn bond_number(&self, gamma_mn_m: f64) -> f64 {
        self.density_diff * G * self.apex_radius * self.apex_radius / (gamma_mn_m * 1e-3)
    }

    /// Compute surface tension from the shape factor S = ds/de.
    ///
    /// The shape factor S is the ratio of the equatorial diameter (de)
    /// to the diameter at a distance de from the apex (ds).
    ///
    /// Uses the empirical relation: gamma = Delta_rho * g * de^2 / H
    /// where H is a function of S = ds/de.
    pub fn tension_from_shape_factor(&self, equatorial_diameter: f64, shape_factor_s: f64) -> f64 {
        // Empirical H factor (Andreas et al., 1938)
        // H = f(S) where S = ds/de
        let h = self.andreas_h_factor(shape_factor_s);
        let de = equatorial_diameter;
        // gamma in mN/m
        self.density_diff * G * de * de / h * 1000.0
    }

    /// Andreas H factor from shape factor S.
    ///
    /// Polynomial fit to the tabulated values.
    fn andreas_h_factor(&self, s: f64) -> f64 {
        // For 0.9 <= S <= 1.0 (elongated drops):
        // H = 0.31280 - 0.46797*S + 0.46763*S^2 + 0.28815*S^3 (approximate)
        if s < 0.401 {
            // Very elongated drops — extrapolate
            0.32720 - 0.97553 * s + 0.84059 * s * s
        } else if s <= 1.0 {
            // Standard range
            let x = s;
            -0.12836 + 0.7577 * x - 1.7713 * x * x + 2.0979 * x * x * x
                - 0.54781 * x * x * x * x
        } else {
            // S > 1 (oblate drops)
            let x = 1.0 / s;
            -0.12836 + 0.7577 * x - 1.7713 * x * x + 2.0979 * x * x * x
                - 0.54781 * x * x * x * x
        }
    }

    /// Integrate the Young-Laplace ODE for axisymmetric drop profile.
    ///
    /// Returns the drop profile as a vector of (x, z) coordinates in meters.
    ///
    /// # Arguments
    /// * `gamma_mn_m` - Surface tension in mN/m
    /// * `n_steps` - Number of integration steps
    /// * `ds` - Arc length step size in meters
    pub fn integrate_profile(
        &self,
        gamma_mn_m: f64,
        n_steps: usize,
        ds: f64,
    ) -> Vec<(f64, f64)> {
        let gamma = gamma_mn_m * 1e-3; // Convert to N/m
        let b = self.apex_radius;

        let mut profile = Vec::with_capacity(n_steps + 1);
        let mut x: f64 = 1e-10; // Avoid division by zero at apex
        let mut z: f64 = 0.0;
        let mut phi: f64 = 0.0;

        profile.push((x, z));

        for _ in 0..n_steps {
            // dphi/ds = 2/b - Delta_rho * g * z / gamma - sin(phi)/x
            let dphi_ds = 2.0 / b - self.density_diff * G * z / gamma - phi.sin() / x;
            let dx_ds = phi.cos();
            let dz_ds = phi.sin();

            // RK2 midpoint method
            let x_mid = x + 0.5 * ds * dx_ds;
            let z_mid = z + 0.5 * ds * dz_ds;
            let phi_mid = phi + 0.5 * ds * dphi_ds;

            let x_mid_safe = if x_mid.abs() < 1e-15 { 1e-15 } else { x_mid };
            let dphi_mid = 2.0 / b - self.density_diff * G * z_mid / gamma
                - phi_mid.sin() / x_mid_safe;
            let dx_mid = phi_mid.cos();
            let dz_mid = phi_mid.sin();

            x += ds * dx_mid;
            z += ds * dz_mid;
            phi += ds * dphi_mid;

            if x < 0.0 {
                break; // Drop has closed back on itself
            }

            profile.push((x, z));
        }

        profile
    }

    /// Estimate surface tension from equatorial and neck diameters.
    ///
    /// Uses the selected plane method: measures diameters at equator
    /// and at a distance de above the equator.
    pub fn estimate_tension_selected_plane(
        &self,
        de: f64,
        ds: f64,
    ) -> f64 {
        let s = ds / de;
        self.tension_from_shape_factor(de, s)
    }
}

// ---------------------------------------------------------------------------
// Maximum Bubble Pressure Method
// ---------------------------------------------------------------------------

/// Maximum bubble pressure tensiometer for dynamic surface tension.
///
/// Measures surface tension as a function of surface age by analyzing
/// the maximum pressure required to form bubbles through a capillary.
///
/// P_max = 2 * gamma / r + rho * g * h
#[derive(Debug, Clone)]
pub struct MaxBubblePressure {
    /// Capillary inner radius in meters
    pub capillary_radius: f64,
    /// Immersion depth in meters
    pub immersion_depth: f64,
    /// Liquid density in kg/m^3
    pub liquid_density: f64,
}

impl MaxBubblePressure {
    /// Create a new maximum bubble pressure tensiometer.
    pub fn new(capillary_radius: f64, immersion_depth: f64, liquid_density: f64) -> Self {
        Self {
            capillary_radius,
            immersion_depth,
            liquid_density,
        }
    }

    /// Compute surface tension from maximum bubble pressure.
    ///
    /// # Arguments
    /// * `pressure_max` - Maximum bubble pressure in Pa
    ///
    /// # Returns
    /// Surface tension in mN/m
    pub fn compute_tension(&self, pressure_max: f64) -> f64 {
        // Subtract hydrostatic pressure
        let p_hydro = self.liquid_density * G * self.immersion_depth;
        let p_laplace = pressure_max - p_hydro;
        // gamma = P * r / 2
        (p_laplace * self.capillary_radius / 2.0) * 1000.0
    }

    /// Compute dynamic surface tension from a series of (lifetime, pressure) measurements.
    ///
    /// # Arguments
    /// * `measurements` - Pairs of (bubble_lifetime_s, max_pressure_Pa)
    ///
    /// # Returns
    /// Vector of (lifetime, surface_tension_mN_m)
    pub fn dynamic_surface_tension(
        &self,
        measurements: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        measurements
            .iter()
            .map(|&(lifetime, p_max)| {
                let gamma = self.compute_tension(p_max);
                (lifetime, gamma)
            })
            .collect()
    }

    /// Compute dead time correction.
    ///
    /// The actual surface age is less than the bubble lifetime due to
    /// the dead time (bubble growth and detachment).
    ///
    /// t_surface = t_lifetime - t_dead
    pub fn surface_age(&self, bubble_lifetime: f64, dead_time: f64) -> f64 {
        (bubble_lifetime - dead_time).max(0.0)
    }

    /// Estimate equilibrium surface tension by extrapolating dynamic data
    /// to infinite surface age using 1/sqrt(t) regression.
    ///
    /// gamma(t) = gamma_eq + k / sqrt(t)
    pub fn extrapolate_equilibrium(&self, dynamic_data: &[(f64, f64)]) -> Option<f64> {
        if dynamic_data.len() < 2 {
            return None;
        }

        // Linear regression of gamma vs 1/sqrt(t)
        let n = dynamic_data.len() as f64;
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xx = 0.0;
        let mut sum_xy = 0.0;

        for &(t, gamma) in dynamic_data {
            if t <= 0.0 {
                continue;
            }
            let x = 1.0 / t.sqrt();
            sum_x += x;
            sum_y += gamma;
            sum_xx += x * x;
            sum_xy += x * gamma;
        }

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return None;
        }

        // Intercept = gamma_eq (at x = 0, i.e., t -> infinity)
        let gamma_eq = (sum_y * sum_xx - sum_x * sum_xy) / denom;
        Some(gamma_eq)
    }
}

// ---------------------------------------------------------------------------
// Capillary Rise (Jurin's Law)
// ---------------------------------------------------------------------------

/// Capillary height calculation using Jurin's law.
///
/// h = 2 * gamma * cos(theta) / (rho * g * r)
///
/// For a perfectly wetting liquid (theta = 0):
/// h = 2 * gamma / (rho * g * r)
pub fn jurin_height(gamma_mn_m: f64, contact_angle_rad: f64, density: f64, tube_radius: f64) -> f64 {
    let gamma = gamma_mn_m * 1e-3; // Convert to N/m
    2.0 * gamma * contact_angle_rad.cos() / (density * G * tube_radius)
}

/// Compute surface tension from capillary rise height.
///
/// Inverse of Jurin's law:
/// gamma = rho * g * r * h / (2 * cos(theta))
///
/// # Returns
/// Surface tension in mN/m
pub fn tension_from_capillary_rise(
    height: f64,
    contact_angle_rad: f64,
    density: f64,
    tube_radius: f64,
) -> f64 {
    density * G * tube_radius * height / (2.0 * contact_angle_rad.cos()) * 1000.0
}

/// Capillary rise analyzer for multiple tube radii.
#[derive(Debug, Clone)]
pub struct CapillaryRise {
    /// Liquid density in kg/m^3
    pub density: f64,
    /// Contact angle in radians
    pub contact_angle: f64,
}

impl CapillaryRise {
    /// Create a new capillary rise analyzer.
    pub fn new(density: f64, contact_angle: f64) -> Self {
        Self {
            density,
            contact_angle,
        }
    }

    /// Water at 20 deg C with perfect wetting.
    pub fn water_20c() -> Self {
        Self {
            density: 998.2,
            contact_angle: 0.0,
        }
    }

    /// Compute expected capillary rise height.
    pub fn height(&self, gamma_mn_m: f64, tube_radius: f64) -> f64 {
        jurin_height(gamma_mn_m, self.contact_angle, self.density, tube_radius)
    }

    /// Compute surface tension from measured height.
    pub fn tension(&self, height: f64, tube_radius: f64) -> f64 {
        tension_from_capillary_rise(height, self.contact_angle, self.density, tube_radius)
    }

    /// Compute heights for multiple tube radii.
    pub fn height_series(&self, gamma_mn_m: f64, radii: &[f64]) -> Vec<(f64, f64)> {
        radii
            .iter()
            .map(|&r| (r, self.height(gamma_mn_m, r)))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Young-Laplace Pressure
// ---------------------------------------------------------------------------

/// Compute Laplace pressure from surface tension and principal radii of curvature.
///
/// Delta_P = gamma * (1/R1 + 1/R2)
///
/// For a sphere: Delta_P = 2 * gamma / R
///
/// # Arguments
/// * `gamma_mn_m` - Surface tension in mN/m
/// * `r1` - First principal radius of curvature in meters
/// * `r2` - Second principal radius of curvature in meters
///
/// # Returns
/// Pressure difference in Pa
pub fn laplace_pressure(gamma_mn_m: f64, r1: f64, r2: f64) -> f64 {
    let gamma = gamma_mn_m * 1e-3; // Convert to N/m
    gamma * (1.0 / r1 + 1.0 / r2)
}

/// Laplace pressure for a spherical interface.
///
/// Delta_P = 2 * gamma / R
pub fn laplace_pressure_sphere(gamma_mn_m: f64, radius: f64) -> f64 {
    laplace_pressure(gamma_mn_m, radius, radius)
}

/// Laplace pressure for a cylindrical interface.
///
/// Delta_P = gamma / R (one radius is infinite)
pub fn laplace_pressure_cylinder(gamma_mn_m: f64, radius: f64) -> f64 {
    let gamma = gamma_mn_m * 1e-3;
    gamma / radius
}

// ---------------------------------------------------------------------------
// Spreading Coefficient and Work of Adhesion
// ---------------------------------------------------------------------------

/// Compute the spreading coefficient.
///
/// S = gamma_S - gamma_L - gamma_SL
///
/// S > 0: liquid spreads completely
/// S < 0: liquid forms a lens/droplet
///
/// # Arguments
/// * `gamma_s` - Solid (or substrate) surface energy in mN/m
/// * `gamma_l` - Liquid surface tension in mN/m
/// * `gamma_sl` - Solid-liquid interfacial tension in mN/m
///
/// # Returns
/// Spreading coefficient in mN/m
pub fn spreading_coefficient(gamma_s: f64, gamma_l: f64, gamma_sl: f64) -> f64 {
    gamma_s - gamma_l - gamma_sl
}

/// Compute the work of adhesion (Dupre-Young equation).
///
/// W_a = gamma_L * (1 + cos(theta))
///
/// # Arguments
/// * `gamma_l` - Liquid surface tension in mN/m
/// * `contact_angle` - Contact angle in radians
///
/// # Returns
/// Work of adhesion in mN/m (= mJ/m^2)
pub fn work_of_adhesion(gamma_l: f64, contact_angle: f64) -> f64 {
    gamma_l * (1.0 + contact_angle.cos())
}

/// Compute the work of cohesion.
///
/// W_c = 2 * gamma_L
pub fn work_of_cohesion(gamma_l: f64) -> f64 {
    2.0 * gamma_l
}

/// Young's equation: relates contact angle to interfacial tensions.
///
/// cos(theta) = (gamma_SV - gamma_SL) / gamma_LV
///
/// # Returns
/// Contact angle in radians
pub fn young_contact_angle(gamma_sv: f64, gamma_sl: f64, gamma_lv: f64) -> f64 {
    let cos_theta = (gamma_sv - gamma_sl) / gamma_lv;
    cos_theta.clamp(-1.0, 1.0).acos()
}

// ---------------------------------------------------------------------------
// Gibbs Adsorption Isotherm
// ---------------------------------------------------------------------------

/// Gibbs adsorption isotherm analyzer.
///
/// Computes surface excess concentration from the slope of
/// surface tension vs log(concentration):
///
/// Gamma = -(1 / (n * R * T)) * d(gamma) / d(ln c)
///
/// where n is the dissociation factor (1 for nonionic, 2 for 1:1 ionic).
#[derive(Debug, Clone)]
pub struct GibbsAdsorption {
    /// Temperature in Kelvin
    pub temperature: f64,
    /// Dissociation factor (1 for nonionic, 2 for 1:1 ionic surfactants)
    pub n_factor: f64,
}

impl GibbsAdsorption {
    /// Create a new Gibbs adsorption analyzer.
    pub fn new(temperature: f64, n_factor: f64) -> Self {
        Self {
            temperature,
            n_factor,
        }
    }

    /// Standard conditions: 25 deg C, nonionic surfactant.
    pub fn nonionic_25c() -> Self {
        Self {
            temperature: 298.15,
            n_factor: 1.0,
        }
    }

    /// Standard conditions: 25 deg C, ionic surfactant.
    pub fn ionic_25c() -> Self {
        Self {
            temperature: 298.15,
            n_factor: 2.0,
        }
    }

    /// Compute surface excess from gamma vs concentration data.
    ///
    /// Uses finite differences of d(gamma)/d(ln c).
    ///
    /// # Arguments
    /// * `data` - Pairs of (concentration_mol_L, gamma_mN_m)
    ///
    /// # Returns
    /// Vector of (concentration, surface_excess_mol_m2)
    pub fn surface_excess(&self, data: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if data.len() < 2 {
            return Vec::new();
        }

        let mut result = Vec::with_capacity(data.len() - 1);

        for i in 0..data.len() - 1 {
            let (c1, g1) = data[i];
            let (c2, g2) = data[i + 1];

            if c1 <= 0.0 || c2 <= 0.0 {
                continue;
            }

            let ln_c1 = c1.ln();
            let ln_c2 = c2.ln();

            if (ln_c2 - ln_c1).abs() < 1e-30 {
                continue;
            }

            // d(gamma)/d(ln c) in mN/m
            let dgamma_dlnc = (g2 - g1) / (ln_c2 - ln_c1);

            // Gamma = -(1/(n*R*T)) * dgamma/d(ln c)
            // Convert gamma from mN/m to N/m for SI units
            let gamma_excess = -(1.0 / (self.n_factor * R_GAS * self.temperature))
                * dgamma_dlnc * 1e-3;

            let c_mid = (c1 * c2).sqrt(); // Geometric mean
            result.push((c_mid, gamma_excess));
        }

        result
    }

    /// Compute area per molecule from surface excess.
    ///
    /// A = 1 / (N_A * Gamma)
    ///
    /// # Returns
    /// Area per molecule in nm^2
    pub fn area_per_molecule(surface_excess_mol_m2: f64) -> f64 {
        let avogadro = 6.02214076e23;
        if surface_excess_mol_m2.abs() < 1e-30 {
            return f64::INFINITY;
        }
        1.0 / (avogadro * surface_excess_mol_m2) * 1e18 // Convert m^2 to nm^2
    }

    /// Compute maximum surface excess from Langmuir model.
    ///
    /// Gamma_max = gamma_0 / (n * R * T * ln(1 + c_max/a))
    /// This is derived from the Szyszkowski model at saturation.
    pub fn max_surface_excess(&self, gamma_0: f64, a_param: f64, gamma_cmc: f64) -> f64 {
        let delta_gamma = (gamma_0 - gamma_cmc) * 1e-3; // N/m
        delta_gamma / (self.n_factor * R_GAS * self.temperature)
    }
}

// ---------------------------------------------------------------------------
// CMC Determination
// ---------------------------------------------------------------------------

/// Critical Micelle Concentration determiner.
///
/// Identifies the CMC from surface tension vs concentration data
/// by finding the breakpoint where surface tension stops decreasing.
#[derive(Debug, Clone)]
pub struct CmcDeterminer {
    /// Minimum number of points required in each linear region
    pub min_points_per_region: usize,
}

impl CmcDeterminer {
    /// Create a new CMC determiner.
    pub fn new(min_points_per_region: usize) -> Self {
        Self {
            min_points_per_region,
        }
    }

    /// Default: at least 3 points per region.
    pub fn default_config() -> Self {
        Self {
            min_points_per_region: 3,
        }
    }

    /// Determine CMC from surface tension isotherm data.
    ///
    /// Uses a piecewise linear fit (two-line intersection method).
    ///
    /// # Arguments
    /// * `data` - Pairs of (concentration, gamma_mN_m), must be sorted by concentration
    ///
    /// # Returns
    /// (CMC concentration, gamma at CMC) or None if data insufficient
    pub fn determine_cmc(&self, data: &[(f64, f64)]) -> Option<(f64, f64)> {
        let n = data.len();
        if n < 2 * self.min_points_per_region {
            return None;
        }

        // Convert concentrations to log scale
        let log_data: Vec<(f64, f64)> = data
            .iter()
            .filter(|(c, _)| *c > 0.0)
            .map(|(c, g)| (c.ln(), *g))
            .collect();

        if log_data.len() < 2 * self.min_points_per_region {
            return None;
        }

        // Find best breakpoint by minimizing total residual
        let mut best_breakpoint = self.min_points_per_region;
        let mut best_residual = f64::MAX;

        for bp in self.min_points_per_region..log_data.len() - self.min_points_per_region + 1 {
            let (_, _, r1) = linear_regression(&log_data[..bp]);
            let (_, _, r2) = linear_regression(&log_data[bp..]);
            let total_residual = r1 + r2;

            if total_residual < best_residual {
                best_residual = total_residual;
                best_breakpoint = bp;
            }
        }

        // Fit two lines at the best breakpoint
        let (a1, b1, _) = linear_regression(&log_data[..best_breakpoint]);
        let (a2, b2, _) = linear_regression(&log_data[best_breakpoint..]);

        // Find intersection: a1*x + b1 = a2*x + b2
        if (a1 - a2).abs() < 1e-15 {
            return None; // Parallel lines
        }

        let ln_cmc = (b2 - b1) / (a1 - a2);
        let gamma_cmc = a1 * ln_cmc + b1;
        let cmc = ln_cmc.exp();

        Some((cmc, gamma_cmc))
    }

    /// Determine CMC using the first derivative method.
    ///
    /// Finds the concentration where d(gamma)/d(log c) reaches
    /// a minimum (steepest decline), then transitions to near-zero slope.
    pub fn determine_cmc_derivative(&self, data: &[(f64, f64)]) -> Option<(f64, f64)> {
        if data.len() < 4 {
            return None;
        }

        let log_data: Vec<(f64, f64)> = data
            .iter()
            .filter(|(c, _)| *c > 0.0)
            .map(|(c, g)| (c.ln(), *g))
            .collect();

        if log_data.len() < 4 {
            return None;
        }

        // Compute first derivative
        let mut derivatives: Vec<(f64, f64)> = Vec::new();
        for i in 0..log_data.len() - 1 {
            let dlnc = log_data[i + 1].0 - log_data[i].0;
            if dlnc.abs() < 1e-30 {
                continue;
            }
            let dgamma = log_data[i + 1].1 - log_data[i].1;
            let deriv = dgamma / dlnc;
            let lnc_mid = (log_data[i].0 + log_data[i + 1].0) / 2.0;
            derivatives.push((lnc_mid, deriv));
        }

        if derivatives.len() < 2 {
            return None;
        }

        // Find where derivative transitions from negative to near-zero
        // Look for the last point with a significant negative derivative
        let threshold = -2.0; // mN/m per unit ln(c)
        let mut cmc_idx = derivatives.len() - 1;

        for i in 0..derivatives.len() {
            if derivatives[i].1 > threshold && i > 0 {
                cmc_idx = i;
                break;
            }
        }

        let ln_cmc = derivatives[cmc_idx].0;
        let cmc = ln_cmc.exp();

        // Interpolate gamma at CMC
        let mut gamma_at_cmc = data[0].1;
        for i in 0..log_data.len() - 1 {
            if log_data[i].0 <= ln_cmc && log_data[i + 1].0 >= ln_cmc {
                let frac = (ln_cmc - log_data[i].0) / (log_data[i + 1].0 - log_data[i].0);
                gamma_at_cmc = log_data[i].1 + frac * (log_data[i + 1].1 - log_data[i].1);
                break;
            }
        }

        Some((cmc, gamma_at_cmc))
    }
}

/// Simple linear regression y = a*x + b.
///
/// Returns (slope, intercept, sum_of_squared_residuals).
fn linear_regression(data: &[(f64, f64)]) -> (f64, f64, f64) {
    let n = data.len() as f64;
    if data.len() < 2 {
        return (0.0, data.first().map_or(0.0, |d| d.1), 0.0);
    }

    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut sum_xx = 0.0;
    let mut sum_xy = 0.0;

    for &(x, y) in data {
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
    }

    let denom = n * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return (0.0, sum_y / n, 0.0);
    }

    let a = (n * sum_xy - sum_x * sum_y) / denom;
    let b = (sum_y * sum_xx - sum_x * sum_xy) / denom;

    // Sum of squared residuals
    let mut ssr = 0.0;
    for &(x, y) in data {
        let residual = y - (a * x + b);
        ssr += residual * residual;
    }

    (a, b, ssr)
}

// ---------------------------------------------------------------------------
// Szyszkowski Model
// ---------------------------------------------------------------------------

/// Szyszkowski equation for surfactant surface tension modeling.
///
/// gamma = gamma_0 - a * ln(1 + c/b)
///
/// where:
/// - gamma_0 is the pure solvent surface tension
/// - a and b are empirical constants
/// - c is the surfactant concentration
#[derive(Debug, Clone)]
pub struct SzyszkowskiModel {
    /// Pure solvent surface tension (mN/m)
    pub gamma_0: f64,
    /// Szyszkowski parameter a (mN/m)
    pub a: f64,
    /// Szyszkowski parameter b (same units as concentration)
    pub b: f64,
}

impl SzyszkowskiModel {
    /// Create a new Szyszkowski model.
    pub fn new(gamma_0: f64, a: f64, b: f64) -> Self {
        Self { gamma_0, a, b }
    }

    /// Compute surface tension at a given concentration.
    pub fn tension(&self, concentration: f64) -> f64 {
        self.gamma_0 - self.a * (1.0 + concentration / self.b).ln()
    }

    /// Compute surface tension for a range of concentrations.
    pub fn tension_isotherm(&self, concentrations: &[f64]) -> Vec<(f64, f64)> {
        concentrations
            .iter()
            .map(|&c| (c, self.tension(c)))
            .collect()
    }

    /// Compute the surface pressure at a given concentration.
    ///
    /// pi = gamma_0 - gamma = a * ln(1 + c/b)
    pub fn surface_pressure(&self, concentration: f64) -> f64 {
        self.a * (1.0 + concentration / self.b).ln()
    }

    /// Fit Szyszkowski parameters from experimental data.
    ///
    /// Uses a simple iterative approach: linearize and fit.
    ///
    /// # Arguments
    /// * `gamma_0` - Known pure solvent surface tension
    /// * `data` - Pairs of (concentration, gamma_mN_m)
    ///
    /// # Returns
    /// Fitted SzyszkowskiModel
    pub fn fit(gamma_0: f64, data: &[(f64, f64)]) -> Self {
        if data.len() < 2 {
            return Self::new(gamma_0, 1.0, 1.0);
        }

        // For the Szyszkowski equation: gamma_0 - gamma = a * ln(1 + c/b)
        // Try multiple b values and find best fit
        let mut best_a = 1.0;
        let mut best_b = 1.0;
        let mut best_err = f64::MAX;

        // Estimate b from the data range
        let c_min = data.iter().map(|(c, _)| *c).fold(f64::MAX, f64::min);
        let c_max = data.iter().map(|(c, _)| *c).fold(0.0_f64, f64::max);

        for i in 0..50 {
            let frac = i as f64 / 49.0;
            let b_trial = c_min * 0.01 + frac * c_max * 2.0;
            if b_trial <= 0.0 {
                continue;
            }

            // Linear fit: (gamma_0 - gamma) = a * ln(1 + c/b)
            let mut sum_x = 0.0;
            let mut sum_y = 0.0;
            let mut sum_xx = 0.0;
            let mut sum_xy = 0.0;
            let n = data.len() as f64;

            for &(c, g) in data {
                let x = (1.0 + c / b_trial).ln();
                let y = gamma_0 - g;
                sum_x += x;
                sum_y += y;
                sum_xx += x * x;
                sum_xy += x * y;
            }

            let denom = n * sum_xx - sum_x * sum_x;
            if denom.abs() < 1e-30 {
                continue;
            }

            let a_trial = (n * sum_xy - sum_x * sum_y) / denom;

            // Compute error
            let mut err = 0.0;
            for &(c, g) in data {
                let g_pred = gamma_0 - a_trial * (1.0 + c / b_trial).ln();
                err += (g - g_pred).powi(2);
            }

            if err < best_err && a_trial > 0.0 {
                best_err = err;
                best_a = a_trial;
                best_b = b_trial;
            }
        }

        Self::new(gamma_0, best_a, best_b)
    }

    /// Compute the Gibbs elasticity E_G = -d(gamma)/d(ln A)
    /// where A is the area per molecule.
    ///
    /// For the Szyszkowski model:
    /// E_G = a * c / (b + c)
    pub fn gibbs_elasticity(&self, concentration: f64) -> f64 {
        self.a * concentration / (self.b + concentration)
    }
}

// ---------------------------------------------------------------------------
// Langmuir Adsorption Isotherm
// ---------------------------------------------------------------------------

/// Langmuir adsorption isotherm for surface coverage.
///
/// theta = K * c / (1 + K * c)
///
/// Combined with Szyszkowski gives the Langmuir-Szyszkowski equation:
/// gamma = gamma_0 - Gamma_max * R * T * ln(1 + K * c)
#[derive(Debug, Clone)]
pub struct LangmuirAdsorption {
    /// Langmuir equilibrium constant K (L/mol or 1/M)
    pub k_eq: f64,
    /// Maximum surface excess Gamma_max (mol/m^2)
    pub gamma_max: f64,
    /// Temperature in Kelvin
    pub temperature: f64,
}

impl LangmuirAdsorption {
    /// Create a new Langmuir adsorption model.
    pub fn new(k_eq: f64, gamma_max: f64, temperature: f64) -> Self {
        Self {
            k_eq,
            gamma_max,
            temperature,
        }
    }

    /// Compute fractional surface coverage.
    ///
    /// theta = K*c / (1 + K*c)
    pub fn coverage(&self, concentration: f64) -> f64 {
        self.k_eq * concentration / (1.0 + self.k_eq * concentration)
    }

    /// Compute surface excess (mol/m^2).
    ///
    /// Gamma = Gamma_max * theta
    pub fn surface_excess(&self, concentration: f64) -> f64 {
        self.gamma_max * self.coverage(concentration)
    }

    /// Compute surface tension using Langmuir-Szyszkowski equation.
    ///
    /// gamma = gamma_0 - Gamma_max * R * T * ln(1 + K * c)
    ///
    /// # Returns
    /// Surface tension in mN/m
    pub fn tension(&self, gamma_0_mn_m: f64, concentration: f64) -> f64 {
        let delta = self.gamma_max * R_GAS * self.temperature
            * (1.0 + self.k_eq * concentration).ln();
        gamma_0_mn_m - delta * 1000.0
    }

    /// Compute area per molecule at given concentration.
    ///
    /// # Returns
    /// Area per molecule in nm^2
    pub fn area_per_molecule(&self, concentration: f64) -> f64 {
        let gamma = self.surface_excess(concentration);
        GibbsAdsorption::area_per_molecule(gamma)
    }

    /// Fit Langmuir parameters from (concentration, surface_excess) data
    /// using linearized Langmuir equation: c/Gamma = c/Gamma_max + 1/(K*Gamma_max)
    pub fn fit_from_excess(data: &[(f64, f64)], temperature: f64) -> Self {
        if data.len() < 2 {
            return Self::new(1.0, 1e-6, temperature);
        }

        // Linearized: c/Gamma = (1/Gamma_max)*c + 1/(K*Gamma_max)
        let lin_data: Vec<(f64, f64)> = data
            .iter()
            .filter(|(_, g)| *g > 1e-30)
            .map(|(c, g)| (*c, c / g))
            .collect();

        if lin_data.len() < 2 {
            return Self::new(1.0, 1e-6, temperature);
        }

        let (slope, intercept, _) = linear_regression(&lin_data);

        let gamma_max = if slope.abs() > 1e-30 { 1.0 / slope } else { 1e-6 };
        let k_eq = if intercept.abs() > 1e-30 {
            gamma_max / intercept
        } else {
            1.0
        };

        Self::new(k_eq.abs(), gamma_max.abs(), temperature)
    }
}

// ---------------------------------------------------------------------------
// Surface Tension Temperature Dependence
// ---------------------------------------------------------------------------

/// Surface tension temperature dependence models.
pub struct SurfaceTensionTemperature;

impl SurfaceTensionTemperature {
    /// Eotvos rule: gamma * V^(2/3) = k * (Tc - T)
    ///
    /// # Arguments
    /// * `k` - Eotvos constant (typically ~2.1 for many organic liquids)
    /// * `tc` - Critical temperature in Kelvin
    /// * `t` - Temperature in Kelvin
    /// * `molar_volume` - Molar volume in m^3/mol
    ///
    /// # Returns
    /// Surface tension in mN/m
    pub fn eotvos(k: f64, tc: f64, t: f64, molar_volume: f64) -> f64 {
        if t >= tc {
            return 0.0;
        }
        let v23 = molar_volume.powf(2.0 / 3.0);
        k * (tc - t) / v23 * 1000.0
    }

    /// Guggenheim-Katayama equation: gamma = gamma_0 * (1 - T/Tc)^n
    ///
    /// # Arguments
    /// * `gamma_0` - Reference surface tension parameter (mN/m)
    /// * `tc` - Critical temperature in Kelvin
    /// * `t` - Temperature in Kelvin
    /// * `n` - Exponent (typically 11/9 ≈ 1.222 for many liquids)
    ///
    /// # Returns
    /// Surface tension in mN/m
    pub fn guggenheim(gamma_0: f64, tc: f64, t: f64, n: f64) -> f64 {
        if t >= tc {
            return 0.0;
        }
        gamma_0 * (1.0 - t / tc).powf(n)
    }

    /// Linear approximation for small temperature ranges.
    ///
    /// gamma(T) = gamma_ref + d_gamma/dT * (T - T_ref)
    pub fn linear(gamma_ref: f64, dgamma_dt: f64, t_ref: f64, t: f64) -> f64 {
        gamma_ref + dgamma_dt * (t - t_ref)
    }

    /// Water surface tension as a function of temperature using IAPWS-IF97
    /// simplified correlation.
    ///
    /// Valid for 0.01 deg C to 373.9 deg C (critical point).
    ///
    /// # Arguments
    /// * `temp_c` - Temperature in degrees Celsius
    ///
    /// # Returns
    /// Surface tension in mN/m
    pub fn water(temp_c: f64) -> f64 {
        let tc = 647.096; // Critical temperature of water in K
        let t = temp_c + 273.15;
        if t >= tc {
            return 0.0;
        }
        let tau = 1.0 - t / tc;
        // IAPWS correlation coefficients
        let b = 235.8; // mN/m
        let mu = 1.256;
        let b2 = -0.625;
        b * tau.powf(mu) * (1.0 + b2 * tau) // mN/m
    }
}

// ---------------------------------------------------------------------------
// Contact Angle Analysis
// ---------------------------------------------------------------------------

/// Contact angle measurement and analysis.
#[derive(Debug, Clone)]
pub struct ContactAngleAnalyzer;

impl ContactAngleAnalyzer {
    /// Compute contact angle from sessile drop height and base radius.
    ///
    /// For small drops where gravity is negligible:
    /// theta = 2 * atan(h / r_base)
    pub fn from_drop_geometry(height: f64, base_radius: f64) -> f64 {
        2.0 * (height / base_radius).atan()
    }

    /// Compute contact angle hysteresis.
    ///
    /// Delta_theta = theta_advancing - theta_receding
    pub fn hysteresis(advancing_angle: f64, receding_angle: f64) -> f64 {
        advancing_angle - receding_angle
    }

    /// Cassie-Baxter equation for rough/composite surfaces.
    ///
    /// cos(theta*) = f1 * cos(theta_1) + f2 * cos(theta_2)
    ///
    /// For a surface with fraction f1 solid and f2 = 1-f1 air:
    /// cos(theta*) = f1 * (cos(theta) + 1) - 1
    pub fn cassie_baxter(
        f1: f64,
        theta1_rad: f64,
        f2: f64,
        theta2_rad: f64,
    ) -> f64 {
        let cos_star = f1 * theta1_rad.cos() + f2 * theta2_rad.cos();
        cos_star.clamp(-1.0, 1.0).acos()
    }

    /// Wenzel equation for rough surfaces.
    ///
    /// cos(theta*) = r * cos(theta)
    ///
    /// where r is the roughness ratio (actual area / projected area, r >= 1).
    pub fn wenzel(roughness_ratio: f64, theta_flat_rad: f64) -> f64 {
        let cos_star = roughness_ratio * theta_flat_rad.cos();
        cos_star.clamp(-1.0, 1.0).acos()
    }

    /// Zisman critical surface tension estimation.
    ///
    /// Plot cos(theta) vs gamma_L for a series of liquids.
    /// Extrapolate to cos(theta) = 1 to find gamma_c.
    ///
    /// # Arguments
    /// * `data` - Pairs of (liquid_surface_tension_mN_m, contact_angle_rad)
    ///
    /// # Returns
    /// Critical surface tension in mN/m
    pub fn zisman_critical_tension(data: &[(f64, f64)]) -> Option<f64> {
        if data.len() < 2 {
            return None;
        }

        // Linear regression of cos(theta) vs gamma_L
        let cos_data: Vec<(f64, f64)> = data
            .iter()
            .map(|(g, theta)| (*g, theta.cos()))
            .collect();

        let (slope, intercept, _) = linear_regression(&cos_data);

        if slope.abs() < 1e-15 {
            return None;
        }

        // At cos(theta) = 1: 1 = slope * gamma_c + intercept
        let gamma_c = (1.0 - intercept) / slope;

        if gamma_c > 0.0 {
            Some(gamma_c)
        } else {
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Detergency and Wetting Analysis
// ---------------------------------------------------------------------------

/// Wettability analysis combining surface tension and contact angle.
pub struct WettabilityAnalyzer;

impl WettabilityAnalyzer {
    /// Compute the capillary number.
    ///
    /// Ca = mu * v / gamma
    ///
    /// where mu is dynamic viscosity, v is velocity, gamma is surface tension.
    pub fn capillary_number(viscosity: f64, velocity: f64, gamma_mn_m: f64) -> f64 {
        viscosity * velocity / (gamma_mn_m * 1e-3)
    }

    /// Compute the Weber number.
    ///
    /// We = rho * v^2 * L / gamma
    pub fn weber_number(density: f64, velocity: f64, length: f64, gamma_mn_m: f64) -> f64 {
        density * velocity * velocity * length / (gamma_mn_m * 1e-3)
    }

    /// Compute the Bond number (Eotvos number).
    ///
    /// Bo = Delta_rho * g * L^2 / gamma
    pub fn bond_number(density_diff: f64, length: f64, gamma_mn_m: f64) -> f64 {
        density_diff * G * length * length / (gamma_mn_m * 1e-3)
    }

    /// Compute the capillary length.
    ///
    /// l_c = sqrt(gamma / (Delta_rho * g))
    ///
    /// # Returns
    /// Capillary length in meters
    pub fn capillary_length(gamma_mn_m: f64, density_diff: f64) -> f64 {
        (gamma_mn_m * 1e-3 / (density_diff * G)).sqrt()
    }

    /// Compute Ohnesorge number.
    ///
    /// Oh = mu / sqrt(rho * gamma * L) = sqrt(We) / Re
    pub fn ohnesorge_number(viscosity: f64, density: f64, gamma_mn_m: f64, length: f64) -> f64 {
        viscosity / (density * gamma_mn_m * 1e-3 * length).sqrt()
    }
}

// ---------------------------------------------------------------------------
// Interfacial Tension (Liquid-Liquid)
// ---------------------------------------------------------------------------

/// Interfacial tension measurement between two immiscible liquids.
#[derive(Debug, Clone)]
pub struct InterfacialTension;

impl InterfacialTension {
    /// Antonov's rule approximation.
    ///
    /// gamma_12 ≈ |gamma_1 - gamma_2|
    ///
    /// This is a rough approximation that works for non-polar pairs.
    pub fn antonov(gamma_1: f64, gamma_2: f64) -> f64 {
        (gamma_1 - gamma_2).abs()
    }

    /// Good-Girifalco-Fowkes equation.
    ///
    /// gamma_12 = gamma_1 + gamma_2 - 2 * sqrt(gamma_1^d * gamma_2^d)
    ///
    /// where gamma^d is the dispersive component.
    pub fn good_girifalco(gamma_1_d: f64, gamma_2_d: f64, gamma_1: f64, gamma_2: f64) -> f64 {
        gamma_1 + gamma_2 - 2.0 * (gamma_1_d * gamma_2_d).sqrt()
    }

    /// Owens-Wendt equation for solid surface energy.
    ///
    /// gamma_SL = gamma_S + gamma_L - 2*(sqrt(gamma_S^d * gamma_L^d) + sqrt(gamma_S^p * gamma_L^p))
    ///
    /// where d = dispersive, p = polar components.
    pub fn owens_wendt(
        gamma_s_d: f64,
        gamma_s_p: f64,
        gamma_l_d: f64,
        gamma_l_p: f64,
    ) -> f64 {
        let gamma_s = gamma_s_d + gamma_s_p;
        let gamma_l = gamma_l_d + gamma_l_p;
        gamma_s + gamma_l
            - 2.0 * ((gamma_s_d * gamma_l_d).sqrt() + (gamma_s_p * gamma_l_p).sqrt())
    }

    /// Compute interfacial tension from spinning drop tensiometry.
    ///
    /// gamma = Delta_rho * omega^2 * r^3 / 4
    ///
    /// For elongated drops (L/d > 4, Vonnegut approximation).
    ///
    /// # Arguments
    /// * `density_diff` - Density difference (kg/m^3)
    /// * `angular_velocity` - Angular velocity (rad/s)
    /// * `drop_radius` - Drop radius at equator (m)
    ///
    /// # Returns
    /// Interfacial tension in mN/m
    pub fn spinning_drop(density_diff: f64, angular_velocity: f64, drop_radius: f64) -> f64 {
        density_diff * angular_velocity * angular_velocity * drop_radius.powi(3) / 4.0 * 1000.0
    }
}

// ---------------------------------------------------------------------------
// Surface Elasticity
// ---------------------------------------------------------------------------

/// Surface dilational elasticity and rheology.
#[derive(Debug, Clone)]
pub struct SurfaceElasticity;

impl SurfaceElasticity {
    /// Gibbs elasticity.
    ///
    /// E = -d(gamma)/d(ln A) = -A * d(gamma)/dA
    ///
    /// For a Langmuir monolayer:
    /// E = Gamma * R * T / (1 - theta)
    pub fn gibbs(gamma_max_excess: f64, coverage: f64, temperature: f64) -> f64 {
        if coverage >= 1.0 {
            return f64::INFINITY;
        }
        gamma_max_excess * R_GAS * temperature / (1.0 - coverage) * 1000.0
    }

    /// Marangoni number.
    ///
    /// Ma = -d(gamma)/dT * L * Delta_T / (mu * alpha)
    ///
    /// where alpha is thermal diffusivity.
    pub fn marangoni_number(
        dgamma_dt: f64,
        length: f64,
        delta_t: f64,
        viscosity: f64,
        thermal_diffusivity: f64,
    ) -> f64 {
        (-dgamma_dt * 1e-3 * length * delta_t) / (viscosity * thermal_diffusivity)
    }

    /// Surface dilational modulus from oscillating barrier measurement.
    ///
    /// E* = delta_gamma / (delta_A / A)
    ///
    /// # Returns
    /// Complex modulus as (storage_modulus, loss_modulus) in mN/m
    pub fn dilational_modulus(
        gamma_amplitude: f64,
        area_strain: f64,
        phase_lag_rad: f64,
    ) -> (f64, f64) {
        let e_abs = gamma_amplitude / area_strain;
        let storage = e_abs * phase_lag_rad.cos(); // E'
        let loss = e_abs * phase_lag_rad.sin(); // E''
        (storage, loss)
    }
}

// ---------------------------------------------------------------------------
// Foam and Emulsion Stability
// ---------------------------------------------------------------------------

/// Foam stability analysis from surface tension data.
pub struct FoamStability;

impl FoamStability {
    /// Compute disjoining pressure from DLVO theory (simplified).
    ///
    /// Pi = A_H / (6 * pi * h^3) + 64 * c * k_B * T * tanh(psi/(4*k_B*T))^2 * exp(-kappa*h)
    ///
    /// Simplified to the van der Waals attraction only:
    /// Pi_vdW = -A_H / (6 * pi * h^3)
    pub fn disjoining_pressure_vdw(hamaker_constant: f64, film_thickness: f64) -> f64 {
        -hamaker_constant / (6.0 * PI * film_thickness.powi(3))
    }

    /// Entry coefficient for oil drops into foam films.
    ///
    /// E = gamma_w/a + gamma_w/o - gamma_o/a
    ///
    /// E > 0: oil enters the film (destabilizing)
    /// E < 0: oil does not enter
    pub fn entry_coefficient(gamma_wa: f64, gamma_wo: f64, gamma_oa: f64) -> f64 {
        gamma_wa + gamma_wo - gamma_oa
    }

    /// Bridging coefficient for oil drops spanning a foam film.
    ///
    /// B = gamma_w/a^2 + gamma_w/o^2 - gamma_o/a^2
    pub fn bridging_coefficient(gamma_wa: f64, gamma_wo: f64, gamma_oa: f64) -> f64 {
        gamma_wa * gamma_wa + gamma_wo * gamma_wo - gamma_oa * gamma_oa
    }

    /// Foam stability index from drainage rate.
    ///
    /// Relates to the Marangoni effect: higher surface elasticity = more stable foam.
    pub fn stability_index(surface_elasticity: f64, viscosity: f64, film_thickness: f64) -> f64 {
        surface_elasticity * film_thickness / viscosity
    }
}

// ---------------------------------------------------------------------------
// Surface Excess Computation (numerical)
// ---------------------------------------------------------------------------

/// Compute numerical derivative d(gamma)/d(ln c) from discrete data.
///
/// Uses central differences where possible, forward/backward at boundaries.
pub fn dgamma_dlnc(concentrations: &[f64], gammas: &[f64]) -> Vec<f64> {
    let n = concentrations.len();
    if n != gammas.len() || n < 2 {
        return Vec::new();
    }

    let ln_c: Vec<f64> = concentrations.iter().map(|c| c.ln()).collect();
    let mut derivs = Vec::with_capacity(n);

    for i in 0..n {
        let deriv = if i == 0 {
            // Forward difference
            (gammas[1] - gammas[0]) / (ln_c[1] - ln_c[0])
        } else if i == n - 1 {
            // Backward difference
            (gammas[n - 1] - gammas[n - 2]) / (ln_c[n - 1] - ln_c[n - 2])
        } else {
            // Central difference
            (gammas[i + 1] - gammas[i - 1]) / (ln_c[i + 1] - ln_c[i - 1])
        };
        derivs.push(deriv);
    }

    derivs
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/// Convert surface tension from dyn/cm to mN/m.
///
/// 1 dyn/cm = 1 mN/m (they are the same unit)
pub fn dyn_cm_to_mn_m(dyn_cm: f64) -> f64 {
    dyn_cm
}

/// Convert surface tension from N/m to mN/m.
pub fn n_m_to_mn_m(n_m: f64) -> f64 {
    n_m * 1000.0
}

/// Convert surface tension from mN/m to N/m.
pub fn mn_m_to_n_m(mn_m: f64) -> f64 {
    mn_m * 1e-3
}

/// Compute capillary pressure in a pore.
///
/// P_c = 2 * gamma * cos(theta) / r
///
/// Used in oil recovery and soil science.
pub fn capillary_pressure(gamma_mn_m: f64, contact_angle: f64, pore_radius: f64) -> f64 {
    let gamma = gamma_mn_m * 1e-3;
    2.0 * gamma * contact_angle.cos() / pore_radius
}

/// Kelvin equation: vapor pressure above a curved surface.
///
/// ln(p/p0) = 2 * gamma * V_m / (R * T * r)
///
/// # Arguments
/// * `gamma_mn_m` - Surface tension in mN/m
/// * `molar_volume` - Molar volume of liquid in m^3/mol
/// * `temperature` - Temperature in Kelvin
/// * `radius` - Radius of curvature in meters (positive for convex/droplet)
///
/// # Returns
/// Ratio p/p0
pub fn kelvin_equation(gamma_mn_m: f64, molar_volume: f64, temperature: f64, radius: f64) -> f64 {
    let gamma = gamma_mn_m * 1e-3;
    let exponent = 2.0 * gamma * molar_volume / (R_GAS * temperature * radius);
    exponent.exp()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1e-6;
    const EPSILON_LOOSE: f64 = 0.5; // For empirical approximations

    fn approx_eq(a: f64, b: f64, eps: f64) -> bool {
        (a - b).abs() < eps
    }

    // -----------------------------------------------------------------------
    // Du Nouy Ring Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_du_nouy_standard_ring() {
        let ring = DuNouyRing::standard();
        assert!(approx_eq(ring.ring_radius, 9.545e-3, 1e-6));
        assert!(approx_eq(ring.wire_radius, 0.185e-3, 1e-6));
    }

    #[test]
    fn test_du_nouy_custom_ring() {
        let ring = DuNouyRing::new(0.01, 0.0002);
        assert_eq!(ring.ring_radius, 0.01);
        assert_eq!(ring.wire_radius, 0.0002);
    }

    #[test]
    fn test_du_nouy_tension_no_correction() {
        // For a ring of radius 10 mm, force of ~9.14 mN should give ~72.75 mN/m
        let r = 0.01; // 10 mm
        let gamma_expected = 72.75;
        let force = gamma_expected * 1e-3 * 4.0 * PI * r;
        let gamma = du_nouy_tension(force, r, 1.0);
        assert!(approx_eq(gamma, gamma_expected, 0.01));
    }

    #[test]
    fn test_du_nouy_tension_with_correction() {
        let r = 0.01;
        let correction = 0.95;
        let force = 72.75e-3 * 4.0 * PI * r;
        let gamma = du_nouy_tension(force, r, correction);
        assert!(approx_eq(gamma, 72.75 * 0.95, 0.01));
    }

    #[test]
    fn test_du_nouy_ring_compute_tension() {
        let ring = DuNouyRing::standard();
        let force = 72.75e-3 * 4.0 * PI * ring.ring_radius;
        let gamma = ring.compute_tension(force, 1.0);
        assert!(approx_eq(gamma, 72.75, 0.01));
    }

    #[test]
    fn test_du_nouy_correction_factor() {
        let ring = DuNouyRing::standard();
        let f = ring.harkins_jordan_correction(998.0, 72.75);
        // Correction factor should be between 0.7 and 1.1
        assert!(f > 0.7 && f < 1.1, "Correction factor {} out of range", f);
    }

    #[test]
    fn test_du_nouy_corrected_tension() {
        let ring = DuNouyRing::standard();
        let force = 72.75e-3 * 4.0 * PI * ring.ring_radius;
        let gamma = ring.compute_tension_corrected(force, 998.0);
        // Should be close to 72.75 mN/m after correction
        assert!(gamma > 50.0 && gamma < 100.0, "Corrected tension {} mN/m out of range", gamma);
    }

    // -----------------------------------------------------------------------
    // Wilhelmy Plate Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_wilhelmy_standard_platinum() {
        let plate = WilhelmyPlate::standard_platinum();
        assert!(approx_eq(plate.width, 19.9e-3, 1e-6));
        assert!(approx_eq(plate.thickness, 0.1e-3, 1e-6));
    }

    #[test]
    fn test_wilhelmy_perimeter() {
        let plate = WilhelmyPlate::new(0.02, 0.001);
        let p = plate.perimeter();
        assert!(approx_eq(p, 2.0 * (0.02 + 0.001), EPSILON));
    }

    #[test]
    fn test_wilhelmy_tension_zero_angle() {
        let plate = WilhelmyPlate::new(0.02, 0.0001);
        let perimeter = plate.perimeter();
        let gamma_expected = 72.75;
        let force = gamma_expected * 1e-3 * perimeter;
        let gamma = plate.compute_tension(force, 0.0);
        assert!(approx_eq(gamma, gamma_expected, 0.01));
    }

    #[test]
    fn test_wilhelmy_tension_with_angle() {
        let perimeter = 0.04;
        let contact_angle = PI / 6.0; // 30 degrees
        let force = 72.75e-3 * perimeter * contact_angle.cos();
        let gamma = wilhelmy_tension(force, perimeter, contact_angle);
        assert!(approx_eq(gamma, 72.75, 0.01));
    }

    #[test]
    fn test_wilhelmy_function() {
        let force = 0.001; // 1 mN
        let perimeter = 0.04; // 40 mm
        let gamma = wilhelmy_tension(force, perimeter, 0.0);
        assert!(approx_eq(gamma, 25.0, 0.01));
    }

    #[test]
    fn test_wilhelmy_paper_plate() {
        let plate = WilhelmyPlate::standard_paper();
        assert!(approx_eq(plate.width, 20.0e-3, 1e-6));
        assert!(approx_eq(plate.thickness, 0.15e-3, 1e-6));
    }

    // -----------------------------------------------------------------------
    // Pendant Drop Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_pendant_drop_creation() {
        let pd = PendantDrop::new(998.0, 0.001);
        assert_eq!(pd.density_diff, 998.0);
        assert_eq!(pd.apex_radius, 0.001);
    }

    #[test]
    fn test_pendant_drop_bond_number() {
        let pd = PendantDrop::new(998.0, 0.002);
        let bo = pd.bond_number(72.75);
        // Bo = 998 * 9.81 * 0.002^2 / 0.07275
        let expected = 998.0 * G * 0.004e-3 / 72.75e-3;
        assert!(approx_eq(bo, expected, 0.01));
    }

    #[test]
    fn test_pendant_drop_profile_integration() {
        let pd = PendantDrop::new(998.0, 0.001);
        let profile = pd.integrate_profile(72.75, 100, 1e-5);
        assert!(profile.len() > 50);
        // Profile should start near apex
        assert!(profile[0].0 < 1e-3);
        assert!(profile[0].1.abs() < 1e-3);
    }

    #[test]
    fn test_pendant_drop_profile_grows() {
        let pd = PendantDrop::new(998.0, 0.001);
        let profile = pd.integrate_profile(72.75, 200, 1e-5);
        // x should increase initially (drop widens from apex)
        assert!(profile[50].0 > profile[0].0);
    }

    #[test]
    fn test_pendant_drop_shape_factor() {
        let pd = PendantDrop::new(998.0, 0.001);
        let de = 0.003; // 3 mm equatorial diameter
        let s = 0.7;
        let gamma = pd.tension_from_shape_factor(de, s);
        // Should give a positive surface tension
        assert!(gamma > 0.0);
    }

    #[test]
    fn test_pendant_drop_selected_plane() {
        let pd = PendantDrop::new(998.0, 0.001);
        let gamma = pd.estimate_tension_selected_plane(0.003, 0.0021);
        assert!(gamma > 0.0);
    }

    // -----------------------------------------------------------------------
    // Maximum Bubble Pressure Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_mbp_creation() {
        let mbp = MaxBubblePressure::new(0.0001, 0.01, 998.0);
        assert_eq!(mbp.capillary_radius, 0.0001);
        assert_eq!(mbp.immersion_depth, 0.01);
    }

    #[test]
    fn test_mbp_compute_tension() {
        let mbp = MaxBubblePressure::new(0.0001, 0.0, 998.0);
        // P = 2*gamma/r => gamma = P*r/2
        // For gamma = 72.75 mN/m: P = 2 * 0.07275 / 0.0001 = 1455 Pa
        let p = 2.0 * 72.75e-3 / 0.0001;
        let gamma = mbp.compute_tension(p);
        assert!(approx_eq(gamma, 72.75, 0.01));
    }

    #[test]
    fn test_mbp_hydrostatic_correction() {
        let mbp = MaxBubblePressure::new(0.0001, 0.01, 998.0);
        // Hydrostatic pressure = rho * g * h
        let p_hydro = 998.0 * G * 0.01;
        let p_laplace = 2.0 * 72.75e-3 / 0.0001;
        let gamma = mbp.compute_tension(p_laplace + p_hydro);
        assert!(approx_eq(gamma, 72.75, 0.1));
    }

    #[test]
    fn test_mbp_dynamic_tension() {
        let mbp = MaxBubblePressure::new(0.0001, 0.0, 998.0);
        let p_ref = 2.0 * 72.75e-3 / 0.0001;
        let measurements = vec![
            (0.01, p_ref * 1.1),
            (0.1, p_ref * 1.05),
            (1.0, p_ref),
        ];
        let dynamic = mbp.dynamic_surface_tension(&measurements);
        assert_eq!(dynamic.len(), 3);
        assert!(dynamic[2].1 < dynamic[0].1); // Equilibrium < initial
    }

    #[test]
    fn test_mbp_surface_age() {
        let mbp = MaxBubblePressure::new(0.0001, 0.0, 998.0);
        assert!(approx_eq(mbp.surface_age(1.0, 0.1), 0.9, EPSILON));
        assert!(approx_eq(mbp.surface_age(0.05, 0.1), 0.0, EPSILON));
    }

    #[test]
    fn test_mbp_extrapolate_equilibrium() {
        let mbp = MaxBubblePressure::new(0.0001, 0.0, 998.0);
        let data = vec![
            (0.01, 80.0),
            (0.1, 75.0),
            (1.0, 73.5),
            (10.0, 73.0),
        ];
        let gamma_eq = mbp.extrapolate_equilibrium(&data);
        assert!(gamma_eq.is_some());
        // Equilibrium value should be around 72-73 mN/m
        let eq = gamma_eq.unwrap();
        assert!(eq > 60.0 && eq < 85.0, "Equilibrium {} out of range", eq);
    }

    // -----------------------------------------------------------------------
    // Capillary Rise (Jurin's Law) Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_jurin_height_water() {
        // Water in a 0.5 mm radius tube, perfect wetting
        let h = jurin_height(GAMMA_WATER_20C, 0.0, 998.2, 0.0005);
        // h = 2 * 0.07275 / (998.2 * 9.807 * 0.0005) ≈ 0.0297 m = 29.7 mm
        let expected = 2.0 * 72.75e-3 / (998.2 * G * 0.0005);
        assert!(approx_eq(h, expected, 1e-5));
    }

    #[test]
    fn test_jurin_height_with_contact_angle() {
        let theta = PI / 4.0; // 45 degrees
        let h = jurin_height(72.75, theta, 998.2, 0.001);
        let h_zero = jurin_height(72.75, 0.0, 998.2, 0.001);
        // Height with angle should be cos(45) * height at zero angle
        assert!(approx_eq(h, h_zero * theta.cos(), 1e-5));
    }

    #[test]
    fn test_capillary_rise_inverse() {
        let gamma = 72.75;
        let h = jurin_height(gamma, 0.0, 998.2, 0.001);
        let gamma_back = tension_from_capillary_rise(h, 0.0, 998.2, 0.001);
        assert!(approx_eq(gamma_back, gamma, 0.01));
    }

    #[test]
    fn test_capillary_rise_water_20c() {
        let cr = CapillaryRise::water_20c();
        assert_eq!(cr.density, 998.2);
        assert_eq!(cr.contact_angle, 0.0);
    }

    #[test]
    fn test_capillary_rise_height_series() {
        let cr = CapillaryRise::water_20c();
        let radii = vec![0.0005, 0.001, 0.002, 0.005];
        let series = cr.height_series(GAMMA_WATER_20C, &radii);
        assert_eq!(series.len(), 4);
        // Height should decrease with increasing radius
        assert!(series[0].1 > series[1].1);
        assert!(series[1].1 > series[2].1);
        assert!(series[2].1 > series[3].1);
    }

    #[test]
    fn test_capillary_rise_tension() {
        let cr = CapillaryRise::water_20c();
        let h = cr.height(72.75, 0.001);
        let gamma = cr.tension(h, 0.001);
        assert!(approx_eq(gamma, 72.75, 0.01));
    }

    // -----------------------------------------------------------------------
    // Laplace Pressure Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_laplace_pressure_sphere() {
        // Water bubble of radius 1 mm
        let p = laplace_pressure_sphere(GAMMA_WATER_20C, 0.001);
        // P = 2 * 0.07275 / 0.001 = 145.5 Pa
        let expected = 2.0 * 72.75e-3 / 0.001;
        assert!(approx_eq(p, expected, 0.01));
    }

    #[test]
    fn test_laplace_pressure_cylinder() {
        let p = laplace_pressure_cylinder(72.75, 0.001);
        // P = 0.07275 / 0.001 = 72.75 Pa
        assert!(approx_eq(p, 72.75, 0.01));
    }

    #[test]
    fn test_laplace_pressure_general() {
        let p = laplace_pressure(72.75, 0.001, 0.002);
        // P = gamma * (1/R1 + 1/R2) = 0.07275 * (1000 + 500) = 109.125 Pa
        let expected = 72.75e-3 * (1.0 / 0.001 + 1.0 / 0.002);
        assert!(approx_eq(p, expected, 0.01));
    }

    #[test]
    fn test_laplace_sphere_equals_general() {
        let r = 0.001;
        let p_sphere = laplace_pressure_sphere(72.75, r);
        let p_general = laplace_pressure(72.75, r, r);
        assert!(approx_eq(p_sphere, p_general, EPSILON));
    }

    // -----------------------------------------------------------------------
    // Spreading Coefficient and Adhesion Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_spreading_coefficient_positive() {
        // When solid energy > liquid + interfacial: spreading occurs
        let s = spreading_coefficient(100.0, 30.0, 20.0);
        assert!(s > 0.0);
        assert!(approx_eq(s, 50.0, EPSILON));
    }

    #[test]
    fn test_spreading_coefficient_negative() {
        // When solid energy < liquid + interfacial: droplet forms
        let s = spreading_coefficient(30.0, 72.75, 20.0);
        assert!(s < 0.0);
    }

    #[test]
    fn test_work_of_adhesion_zero_angle() {
        // Perfect wetting: theta = 0
        let wa = work_of_adhesion(72.75, 0.0);
        assert!(approx_eq(wa, 2.0 * 72.75, EPSILON));
    }

    #[test]
    fn test_work_of_adhesion_90_deg() {
        let wa = work_of_adhesion(72.75, PI / 2.0);
        assert!(approx_eq(wa, 72.75, 0.01));
    }

    #[test]
    fn test_work_of_cohesion() {
        let wc = work_of_cohesion(72.75);
        assert!(approx_eq(wc, 145.5, EPSILON));
    }

    #[test]
    fn test_young_contact_angle() {
        // gamma_SV = 100, gamma_SL = 20, gamma_LV = 72.75
        // cos(theta) = (100 - 20) / 72.75 ≈ 1.099 → clamped to 1.0 → theta = 0
        let theta = young_contact_angle(100.0, 20.0, 72.75);
        assert!(approx_eq(theta, 0.0, 0.01));
    }

    #[test]
    fn test_young_contact_angle_finite() {
        // cos(theta) = (50 - 20) / 72.75 ≈ 0.412
        let theta = young_contact_angle(50.0, 20.0, 72.75);
        let expected = (30.0_f64 / 72.75).acos();
        assert!(approx_eq(theta, expected, EPSILON));
    }

    // -----------------------------------------------------------------------
    // Gibbs Adsorption Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_gibbs_nonionic_25c() {
        let ga = GibbsAdsorption::nonionic_25c();
        assert!(approx_eq(ga.temperature, 298.15, EPSILON));
        assert_eq!(ga.n_factor, 1.0);
    }

    #[test]
    fn test_gibbs_ionic_25c() {
        let ga = GibbsAdsorption::ionic_25c();
        assert_eq!(ga.n_factor, 2.0);
    }

    #[test]
    fn test_gibbs_surface_excess() {
        let ga = GibbsAdsorption::nonionic_25c();
        // Synthetic data: gamma decreasing with concentration
        let data = vec![
            (0.001, 72.0),
            (0.002, 65.0),
            (0.004, 55.0),
            (0.008, 45.0),
        ];
        let excess = ga.surface_excess(&data);
        assert_eq!(excess.len(), 3);
        // Surface excess should be positive (gamma decreases with concentration)
        for (_, g) in &excess {
            assert!(*g > 0.0, "Surface excess should be positive: {}", g);
        }
    }

    #[test]
    fn test_gibbs_surface_excess_empty() {
        let ga = GibbsAdsorption::nonionic_25c();
        let excess = ga.surface_excess(&[]);
        assert!(excess.is_empty());
    }

    #[test]
    fn test_gibbs_area_per_molecule() {
        // Typical SDS: Gamma_max ~ 3e-6 mol/m^2
        let area = GibbsAdsorption::area_per_molecule(3e-6);
        // Expected: ~0.55 nm^2 per molecule
        assert!(area > 0.1 && area < 5.0, "Area {} nm^2 out of range", area);
    }

    #[test]
    fn test_gibbs_area_per_molecule_zero() {
        let area = GibbsAdsorption::area_per_molecule(0.0);
        assert!(area.is_infinite());
    }

    #[test]
    fn test_gibbs_max_surface_excess() {
        let ga = GibbsAdsorption::nonionic_25c();
        let gamma_max = ga.max_surface_excess(72.75, 0.001, 33.0);
        // Should be positive
        assert!(gamma_max > 0.0);
    }

    // -----------------------------------------------------------------------
    // CMC Determination Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_cmc_default_config() {
        let cmc = CmcDeterminer::default_config();
        assert_eq!(cmc.min_points_per_region, 3);
    }

    #[test]
    fn test_cmc_determination() {
        let cmc = CmcDeterminer::new(3);
        // Synthetic SDS-like data
        let data: Vec<(f64, f64)> = vec![
            (0.001, 72.0),
            (0.002, 68.0),
            (0.003, 60.0),
            (0.004, 50.0),
            (0.005, 42.0),
            (0.006, 36.0),
            (0.007, 34.0),
            (0.008, 33.5),
            (0.009, 33.3),
            (0.010, 33.2),
            (0.012, 33.1),
            (0.015, 33.0),
        ];
        let result = cmc.determine_cmc(&data);
        assert!(result.is_some());
        let (cmc_val, gamma_cmc) = result.unwrap();
        // CMC should be in reasonable range for this synthetic data
        assert!(cmc_val > 0.003 && cmc_val < 0.025,
            "CMC {} mol/L out of expected range", cmc_val);
        assert!(gamma_cmc > 25.0 && gamma_cmc < 50.0,
            "Gamma at CMC {} mN/m out of range", gamma_cmc);
    }

    #[test]
    fn test_cmc_insufficient_data() {
        let cmc = CmcDeterminer::new(3);
        let data = vec![(0.001, 72.0), (0.01, 33.0)];
        assert!(cmc.determine_cmc(&data).is_none());
    }

    #[test]
    fn test_cmc_derivative_method() {
        let cmc = CmcDeterminer::new(3);
        let data: Vec<(f64, f64)> = vec![
            (0.001, 72.0),
            (0.002, 65.0),
            (0.003, 55.0),
            (0.005, 40.0),
            (0.007, 34.0),
            (0.008, 33.5),
            (0.010, 33.2),
            (0.015, 33.0),
        ];
        let result = cmc.determine_cmc_derivative(&data);
        assert!(result.is_some());
        let (cmc_val, _) = result.unwrap();
        assert!(cmc_val > 0.003 && cmc_val < 0.015,
            "CMC derivative {} out of range", cmc_val);
    }

    // -----------------------------------------------------------------------
    // Szyszkowski Model Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_szyszkowski_creation() {
        let model = SzyszkowskiModel::new(72.75, 20.0, 0.002);
        assert_eq!(model.gamma_0, 72.75);
        assert_eq!(model.a, 20.0);
        assert_eq!(model.b, 0.002);
    }

    #[test]
    fn test_szyszkowski_zero_concentration() {
        let model = SzyszkowskiModel::new(72.75, 20.0, 0.002);
        let gamma = model.tension(0.0);
        assert!(approx_eq(gamma, 72.75, EPSILON));
    }

    #[test]
    fn test_szyszkowski_high_concentration() {
        let model = SzyszkowskiModel::new(72.75, 20.0, 0.002);
        let gamma = model.tension(1.0);
        // Should be well below gamma_0
        assert!(gamma < 72.75);
    }

    #[test]
    fn test_szyszkowski_monotonic_decrease() {
        let model = SzyszkowskiModel::new(72.75, 20.0, 0.002);
        let concs = vec![0.0, 0.001, 0.002, 0.005, 0.01, 0.05, 0.1];
        let isotherm = model.tension_isotherm(&concs);
        for i in 1..isotherm.len() {
            assert!(isotherm[i].1 < isotherm[i - 1].1,
                "Surface tension should decrease with concentration");
        }
    }

    #[test]
    fn test_szyszkowski_surface_pressure() {
        let model = SzyszkowskiModel::new(72.75, 20.0, 0.002);
        let sp = model.surface_pressure(0.0);
        assert!(approx_eq(sp, 0.0, EPSILON));
        let sp2 = model.surface_pressure(0.01);
        assert!(sp2 > 0.0);
    }

    #[test]
    fn test_szyszkowski_fit() {
        let original = SzyszkowskiModel::new(72.75, 15.0, 0.005);
        let concs = vec![0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1];
        let data: Vec<(f64, f64)> = concs.iter().map(|&c| (c, original.tension(c))).collect();
        let fitted = SzyszkowskiModel::fit(72.75, &data);
        // Fitted model should reproduce the data reasonably well
        for &(c, g_expected) in &data {
            let g_fitted = fitted.tension(c);
            assert!(approx_eq(g_fitted, g_expected, 2.0),
                "At c={}: expected {}, got {}", c, g_expected, g_fitted);
        }
    }

    #[test]
    fn test_szyszkowski_gibbs_elasticity() {
        let model = SzyszkowskiModel::new(72.75, 20.0, 0.002);
        let e0 = model.gibbs_elasticity(0.0);
        assert!(approx_eq(e0, 0.0, EPSILON));
        let e_high = model.gibbs_elasticity(1.0);
        // Should approach 'a' at high concentration
        assert!(e_high > 0.0 && e_high <= model.a + EPSILON);
    }

    // -----------------------------------------------------------------------
    // Langmuir Adsorption Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_langmuir_coverage_zero() {
        let lm = LangmuirAdsorption::new(1000.0, 3e-6, 298.15);
        assert!(approx_eq(lm.coverage(0.0), 0.0, EPSILON));
    }

    #[test]
    fn test_langmuir_coverage_high() {
        let lm = LangmuirAdsorption::new(1000.0, 3e-6, 298.15);
        let theta = lm.coverage(100.0);
        // Should approach 1.0 at very high concentration
        assert!(theta > 0.99);
    }

    #[test]
    fn test_langmuir_coverage_at_half() {
        let k = 1000.0;
        let lm = LangmuirAdsorption::new(k, 3e-6, 298.15);
        // At c = 1/K, theta = 0.5
        let theta = lm.coverage(1.0 / k);
        assert!(approx_eq(theta, 0.5, EPSILON));
    }

    #[test]
    fn test_langmuir_surface_excess() {
        let lm = LangmuirAdsorption::new(1000.0, 3e-6, 298.15);
        let gamma = lm.surface_excess(0.01);
        assert!(gamma > 0.0 && gamma <= 3e-6);
    }

    #[test]
    fn test_langmuir_tension() {
        let lm = LangmuirAdsorption::new(1000.0, 3e-6, 298.15);
        let g0 = lm.tension(72.75, 0.0);
        // At zero concentration, should equal gamma_0
        assert!(approx_eq(g0, 72.75, 0.01));
        let g_high = lm.tension(72.75, 0.1);
        assert!(g_high < 72.75);
    }

    #[test]
    fn test_langmuir_area_per_molecule() {
        let lm = LangmuirAdsorption::new(1000.0, 3e-6, 298.15);
        let area = lm.area_per_molecule(0.01);
        assert!(area > 0.0);
    }

    #[test]
    fn test_langmuir_fit() {
        let original = LangmuirAdsorption::new(500.0, 4e-6, 298.15);
        let data: Vec<(f64, f64)> = (1..=10)
            .map(|i| {
                let c = i as f64 * 0.002;
                (c, original.surface_excess(c))
            })
            .collect();
        let fitted = LangmuirAdsorption::fit_from_excess(&data, 298.15);
        // gamma_max should be close
        assert!(
            approx_eq(fitted.gamma_max, 4e-6, 1e-6),
            "Gamma_max: expected ~4e-6, got {}",
            fitted.gamma_max
        );
    }

    // -----------------------------------------------------------------------
    // Temperature Dependence Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_water_surface_tension_20c() {
        let gamma = SurfaceTensionTemperature::water(20.0);
        assert!(approx_eq(gamma, GAMMA_WATER_20C, 1.0),
            "Water at 20C: expected ~{}, got {}", GAMMA_WATER_20C, gamma);
    }

    #[test]
    fn test_water_surface_tension_25c() {
        let gamma = SurfaceTensionTemperature::water(25.0);
        assert!(approx_eq(gamma, GAMMA_WATER_25C, 1.0),
            "Water at 25C: expected ~{}, got {}", GAMMA_WATER_25C, gamma);
    }

    #[test]
    fn test_water_surface_tension_decreases() {
        let g20 = SurfaceTensionTemperature::water(20.0);
        let g50 = SurfaceTensionTemperature::water(50.0);
        let g80 = SurfaceTensionTemperature::water(80.0);
        assert!(g20 > g50, "Should decrease: {} vs {}", g20, g50);
        assert!(g50 > g80, "Should decrease: {} vs {}", g50, g80);
    }

    #[test]
    fn test_water_surface_tension_critical() {
        let gamma = SurfaceTensionTemperature::water(374.0);
        assert!(approx_eq(gamma, 0.0, 0.1));
    }

    #[test]
    fn test_guggenheim_model() {
        let gamma = SurfaceTensionTemperature::guggenheim(100.0, 647.0, 293.15, 11.0 / 9.0);
        assert!(gamma > 0.0);
        let gamma_tc = SurfaceTensionTemperature::guggenheim(100.0, 647.0, 647.0, 11.0 / 9.0);
        assert!(approx_eq(gamma_tc, 0.0, EPSILON));
    }

    #[test]
    fn test_linear_temperature_model() {
        let gamma = SurfaceTensionTemperature::linear(72.75, -0.15, 293.15, 298.15);
        // gamma(25C) = 72.75 - 0.15 * 5 = 72.0
        assert!(approx_eq(gamma, 72.0, EPSILON));
    }

    #[test]
    fn test_eotvos_rule() {
        let gamma = SurfaceTensionTemperature::eotvos(2.1, 647.0, 293.15, 18e-6);
        assert!(gamma > 0.0);
    }

    // -----------------------------------------------------------------------
    // Contact Angle Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_contact_angle_from_drop() {
        let theta = ContactAngleAnalyzer::from_drop_geometry(0.001, 0.002);
        // theta = 2 * atan(0.5) ≈ 53.13 degrees = 0.9273 rad
        let expected = 2.0 * (0.5_f64).atan();
        assert!(approx_eq(theta, expected, EPSILON));
    }

    #[test]
    fn test_contact_angle_hysteresis() {
        let h = ContactAngleAnalyzer::hysteresis(1.2, 0.8);
        assert!(approx_eq(h, 0.4, EPSILON));
    }

    #[test]
    fn test_cassie_baxter() {
        // 50% solid (theta=PI/3), 50% air (theta=PI)
        let theta = ContactAngleAnalyzer::cassie_baxter(0.5, PI / 3.0, 0.5, PI);
        assert!(theta > PI / 3.0); // More hydrophobic than flat
    }

    #[test]
    fn test_wenzel_hydrophilic() {
        // Roughness amplifies hydrophilicity
        let theta_flat = PI / 4.0; // 45 degrees (hydrophilic)
        let theta_rough = ContactAngleAnalyzer::wenzel(2.0, theta_flat);
        assert!(theta_rough < theta_flat);
    }

    #[test]
    fn test_wenzel_hydrophobic() {
        // Roughness amplifies hydrophobicity
        let theta_flat = 2.0; // > 90 degrees (hydrophobic)
        let theta_rough = ContactAngleAnalyzer::wenzel(2.0, theta_flat);
        assert!(theta_rough > theta_flat);
    }

    #[test]
    fn test_zisman_critical_tension() {
        let data = vec![
            (20.0, 0.0_f64),      // Low tension, complete wetting
            (30.0, PI / 6.0),      // 30 degrees
            (50.0, PI / 3.0),      // 60 degrees
            (72.0, 1.3),           // ~74.5 degrees
        ];
        let gamma_c = ContactAngleAnalyzer::zisman_critical_tension(&data);
        assert!(gamma_c.is_some());
        let gc = gamma_c.unwrap();
        assert!(gc > 10.0 && gc < 40.0, "Critical tension {} out of range", gc);
    }

    // -----------------------------------------------------------------------
    // Wettability Analyzer Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_capillary_number() {
        let ca = WettabilityAnalyzer::capillary_number(0.001, 0.01, 72.75);
        // Ca = 0.001 * 0.01 / 0.07275 ≈ 1.37e-4
        let expected = 0.001 * 0.01 / 72.75e-3;
        assert!(approx_eq(ca, expected, EPSILON));
    }

    #[test]
    fn test_weber_number() {
        let we = WettabilityAnalyzer::weber_number(998.0, 1.0, 0.001, 72.75);
        let expected = 998.0 * 1.0 * 0.001 / 72.75e-3;
        assert!(approx_eq(we, expected, 0.01));
    }

    #[test]
    fn test_bond_number() {
        let bo = WettabilityAnalyzer::bond_number(998.0, 0.001, 72.75);
        let expected = 998.0 * G * 0.001 * 0.001 / 72.75e-3;
        assert!(approx_eq(bo, expected, 0.001));
    }

    #[test]
    fn test_capillary_length_water() {
        let lc = WettabilityAnalyzer::capillary_length(72.75, 998.0);
        // l_c ≈ 2.7 mm for water
        assert!(lc > 0.002 && lc < 0.003, "Capillary length {} m", lc);
    }

    #[test]
    fn test_ohnesorge_number() {
        let oh = WettabilityAnalyzer::ohnesorge_number(0.001, 998.0, 72.75, 0.001);
        assert!(oh > 0.0);
    }

    // -----------------------------------------------------------------------
    // Interfacial Tension Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_antonov_rule() {
        let gamma_12 = InterfacialTension::antonov(72.75, 22.1);
        assert!(approx_eq(gamma_12, 50.65, EPSILON));
    }

    #[test]
    fn test_good_girifalco() {
        // Water (dispersive ~21.8 mN/m) vs hexane (dispersive ~18.4 mN/m)
        let gamma_12 = InterfacialTension::good_girifalco(21.8, 18.4, 72.75, 18.4);
        assert!(gamma_12 > 0.0);
    }

    #[test]
    fn test_owens_wendt() {
        let gamma_sl = InterfacialTension::owens_wendt(30.0, 10.0, 21.8, 51.0);
        assert!(gamma_sl >= 0.0);
    }

    #[test]
    fn test_spinning_drop() {
        let gamma = InterfacialTension::spinning_drop(100.0, 2000.0, 0.0005);
        // gamma = 100 * 2000^2 * 0.0005^3 / 4 = 100 * 4e6 * 1.25e-10 / 4 = 12.5e-3 N/m = 12.5 mN/m
        assert!(gamma > 0.0);
    }

    // -----------------------------------------------------------------------
    // Surface Elasticity Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_gibbs_elasticity() {
        let e = SurfaceElasticity::gibbs(3e-6, 0.5, 298.15);
        assert!(e > 0.0);
    }

    #[test]
    fn test_marangoni_number() {
        let ma = SurfaceElasticity::marangoni_number(-0.15, 0.01, 10.0, 0.001, 1.4e-7);
        assert!(ma > 0.0);
    }

    #[test]
    fn test_dilational_modulus() {
        let (storage, loss) = SurfaceElasticity::dilational_modulus(5.0, 0.1, 0.0);
        assert!(approx_eq(storage, 50.0, EPSILON));
        assert!(approx_eq(loss, 0.0, EPSILON));
    }

    #[test]
    fn test_dilational_modulus_with_phase() {
        let (storage, loss) = SurfaceElasticity::dilational_modulus(5.0, 0.1, PI / 4.0);
        let e_abs = 50.0;
        assert!(approx_eq(storage, e_abs * (PI / 4.0).cos(), EPSILON));
        assert!(approx_eq(loss, e_abs * (PI / 4.0).sin(), EPSILON));
    }

    // -----------------------------------------------------------------------
    // Foam Stability Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_disjoining_pressure_vdw() {
        let pi = FoamStability::disjoining_pressure_vdw(1e-20, 1e-8);
        // Should be negative (attractive)
        assert!(pi < 0.0);
    }

    #[test]
    fn test_entry_coefficient() {
        let e = FoamStability::entry_coefficient(72.75, 30.0, 25.0);
        assert!(approx_eq(e, 77.75, EPSILON));
    }

    #[test]
    fn test_bridging_coefficient() {
        let b = FoamStability::bridging_coefficient(72.75, 30.0, 25.0);
        let expected = 72.75 * 72.75 + 30.0 * 30.0 - 25.0 * 25.0;
        assert!(approx_eq(b, expected, 0.01));
    }

    #[test]
    fn test_stability_index() {
        let si = FoamStability::stability_index(50.0, 0.001, 1e-6);
        assert!(si > 0.0);
    }

    // -----------------------------------------------------------------------
    // Numerical Derivative Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_dgamma_dlnc() {
        let concs = vec![0.001, 0.002, 0.004, 0.008];
        let gammas = vec![72.0, 65.0, 55.0, 45.0];
        let derivs = dgamma_dlnc(&concs, &gammas);
        assert_eq!(derivs.len(), 4);
        // Derivatives should be negative (gamma decreases with c)
        for d in &derivs {
            assert!(*d < 0.0, "Derivative should be negative: {}", d);
        }
    }

    #[test]
    fn test_dgamma_dlnc_empty() {
        let derivs = dgamma_dlnc(&[], &[]);
        assert!(derivs.is_empty());
    }

    #[test]
    fn test_dgamma_dlnc_mismatched_lengths() {
        let derivs = dgamma_dlnc(&[1.0, 2.0], &[3.0]);
        assert!(derivs.is_empty());
    }

    // -----------------------------------------------------------------------
    // Unit Conversion Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_dyn_cm_to_mn_m() {
        assert!(approx_eq(dyn_cm_to_mn_m(72.75), 72.75, EPSILON));
    }

    #[test]
    fn test_n_m_to_mn_m() {
        assert!(approx_eq(n_m_to_mn_m(0.07275), 72.75, EPSILON));
    }

    #[test]
    fn test_mn_m_to_n_m() {
        assert!(approx_eq(mn_m_to_n_m(72.75), 0.07275, EPSILON));
    }

    // -----------------------------------------------------------------------
    // Utility Function Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_capillary_pressure() {
        let p = capillary_pressure(72.75, 0.0, 1e-6);
        // P = 2 * 0.07275 / 1e-6 = 145500 Pa
        let expected = 2.0 * 72.75e-3 / 1e-6;
        assert!(approx_eq(p, expected, 1.0));
    }

    #[test]
    fn test_kelvin_equation_large_radius() {
        // For very large radius, p/p0 ≈ 1
        let ratio = kelvin_equation(72.75, 18e-6, 298.15, 1.0);
        assert!(approx_eq(ratio, 1.0, 1e-6));
    }

    #[test]
    fn test_kelvin_equation_small_radius() {
        // For nanometer droplets, p/p0 > 1
        let ratio = kelvin_equation(72.75, 18e-6, 298.15, 5e-9);
        assert!(ratio > 1.0, "Kelvin ratio should be > 1 for small droplets: {}", ratio);
    }

    // -----------------------------------------------------------------------
    // Constants Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_water_constants() {
        assert!(approx_eq(GAMMA_WATER_20C, 72.75, EPSILON));
        assert!(approx_eq(GAMMA_WATER_25C, 71.97, EPSILON));
    }

    #[test]
    fn test_ethanol_constant() {
        assert!(approx_eq(GAMMA_ETHANOL_20C, 22.10, EPSILON));
    }

    #[test]
    fn test_sds_constants() {
        assert!(approx_eq(CMC_SDS_MM, 8.0, EPSILON));
        assert!(approx_eq(GAMMA_MIN_SDS, 33.0, EPSILON));
    }

    // -----------------------------------------------------------------------
    // Linear Regression Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_linear_regression_perfect() {
        let data = vec![(1.0, 3.0), (2.0, 5.0), (3.0, 7.0)];
        let (a, b, ssr) = linear_regression(&data);
        assert!(approx_eq(a, 2.0, EPSILON));
        assert!(approx_eq(b, 1.0, EPSILON));
        assert!(approx_eq(ssr, 0.0, EPSILON));
    }

    #[test]
    fn test_linear_regression_single_point() {
        let data = vec![(1.0, 5.0)];
        let (_, b, _) = linear_regression(&data);
        assert!(approx_eq(b, 5.0, EPSILON));
    }

    // -----------------------------------------------------------------------
    // Integration / End-to-End Tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_roundtrip_wilhelmy_water() {
        let plate = WilhelmyPlate::standard_platinum();
        let gamma = GAMMA_WATER_20C;
        let force = gamma * 1e-3 * plate.perimeter();
        let gamma_measured = plate.compute_tension(force, 0.0);
        assert!(approx_eq(gamma_measured, gamma, 0.01));
    }

    #[test]
    fn test_roundtrip_jurin_law() {
        let cr = CapillaryRise::water_20c();
        let gamma = GAMMA_WATER_20C;
        let r = 0.001;
        let h = cr.height(gamma, r);
        let gamma_back = cr.tension(h, r);
        assert!(approx_eq(gamma_back, gamma, 0.01));
    }

    #[test]
    fn test_laplace_consistency() {
        // Sphere: two equal radii
        let r = 0.001;
        let gamma = 72.75;
        let p1 = laplace_pressure_sphere(gamma, r);
        let p2 = laplace_pressure(gamma, r, r);
        assert!(approx_eq(p1, p2, EPSILON));
        // Cylinder should be exactly half of sphere
        let p3 = laplace_pressure_cylinder(gamma, r);
        assert!(approx_eq(p3, p1 / 2.0, EPSILON));
    }

    #[test]
    fn test_adhesion_cohesion_relation() {
        // W_adhesion at theta=0 should equal W_cohesion
        let gamma = 72.75;
        let wa = work_of_adhesion(gamma, 0.0);
        let wc = work_of_cohesion(gamma);
        assert!(approx_eq(wa, wc, EPSILON));
    }

    #[test]
    fn test_langmuir_isotherm_shape() {
        let lm = LangmuirAdsorption::new(500.0, 3e-6, 298.15);
        let mut prev = 0.0;
        for i in 0..10 {
            let c = (i + 1) as f64 * 0.002;
            let theta = lm.coverage(c);
            assert!(theta > prev, "Coverage should be monotonically increasing");
            assert!(theta <= 1.0, "Coverage should not exceed 1");
            prev = theta;
        }
    }

    #[test]
    fn test_szyszkowski_isotherm_consistency() {
        let model = SzyszkowskiModel::new(72.75, 15.0, 0.003);
        let concs = vec![0.0, 0.001, 0.003, 0.01, 0.05, 0.1];
        let isotherm = model.tension_isotherm(&concs);
        // First point should be gamma_0
        assert!(approx_eq(isotherm[0].1, 72.75, EPSILON));
        // Should be monotonically decreasing
        for i in 1..isotherm.len() {
            assert!(isotherm[i].1 < isotherm[i - 1].1);
        }
    }
}
