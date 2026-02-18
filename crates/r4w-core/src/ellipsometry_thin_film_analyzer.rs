//! Ellipsometry Thin Film Analyzer
//!
//! Spectroscopic ellipsometry data analysis for characterizing thin film thickness,
//! optical constants (n, k), and multilayer structures from polarization state changes
//! upon reflection.
//!
//! # Physics Background
//!
//! Ellipsometry measures the change in polarization state of light reflected from a surface.
//! The fundamental equation relates the ratio of p- and s-polarized reflection coefficients:
//!
//! ```text
//! rho = rp / rs = tan(Psi) * exp(i * Delta)
//! ```
//!
//! where Psi and Delta are the ellipsometric angles measured as functions of wavelength.
//!
//! # Components
//!
//! - `EllipsometryData` - Measured Psi(lambda) and Delta(lambda) spectra
//! - `FresnelCoefficients` - Fresnel reflection/transmission at interfaces
//! - `TransferMatrix` - Transfer matrix method for multilayer stacks
//! - `DispersionModels` - Cauchy, Sellmeier, Tauc-Lorentz, Drude, Lorentz models
//! - `ThicknessFitter` - Single-layer thickness determination via grid search
//! - `MaterialDatabase` - Common material optical constants (Si, SiO2, Si3N4, Au, Al, TiO2)
//! - `EffectiveMediumApproximation` - Maxwell-Garnett, Bruggeman, linear mixing
//! - `MultiAngleAnalysis` - Variable angle spectroscopic ellipsometry
//! - `FilmQuality` - Depolarization, uniformity, sensitivity analysis
//! - `EllipsometrySimulator` - Synthetic data generation with noise

use std::f64::consts::PI;

// ============================================================================
// Complex number helpers (re, im) tuple arithmetic
// ============================================================================

/// Complex addition: (a + bi) + (c + di) = (a+c) + (b+d)i
#[inline]
fn c_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Complex subtraction
#[inline]
fn c_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Complex multiplication: (a + bi)(c + di) = (ac - bd) + (ad + bc)i
#[inline]
fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex division: (a + bi) / (c + di)
#[inline]
fn c_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = b.0 * b.0 + b.1 * b.1;
    if denom < 1e-300 {
        return (0.0, 0.0);
    }
    ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
}

/// Complex modulus squared |z|^2
#[inline]
fn c_abs_sq(z: (f64, f64)) -> f64 {
    z.0 * z.0 + z.1 * z.1
}

/// Complex modulus |z|
#[inline]
fn c_abs(z: (f64, f64)) -> f64 {
    c_abs_sq(z).sqrt()
}

/// Complex argument atan2(im, re)
#[inline]
fn c_arg(z: (f64, f64)) -> f64 {
    z.1.atan2(z.0)
}

/// Complex exponential: exp(a + bi) = exp(a)(cos(b) + i*sin(b))
#[inline]
fn c_exp(z: (f64, f64)) -> (f64, f64) {
    let r = z.0.exp();
    (r * z.1.cos(), r * z.1.sin())
}

/// Complex square root using polar form
#[inline]
fn c_sqrt(z: (f64, f64)) -> (f64, f64) {
    let r = c_abs(z);
    let theta = c_arg(z);
    let sr = r.sqrt();
    (sr * (theta / 2.0).cos(), sr * (theta / 2.0).sin())
}

/// Complex conjugate
#[inline]
fn c_conj(z: (f64, f64)) -> (f64, f64) {
    (z.0, -z.1)
}

/// Scale complex by real
#[inline]
fn c_scale(z: (f64, f64), s: f64) -> (f64, f64) {
    (z.0 * s, z.1 * s)
}

/// Complex cosine: cos(z) = (exp(iz) + exp(-iz)) / 2
#[inline]
fn c_cos(z: (f64, f64)) -> (f64, f64) {
    let iz = (-z.1, z.0);
    let niz = (z.1, -z.0);
    let e1 = c_exp(iz);
    let e2 = c_exp(niz);
    c_scale(c_add(e1, e2), 0.5)
}

/// Complex sine: sin(z) = (exp(iz) - exp(-iz)) / (2i)
#[inline]
fn c_sin(z: (f64, f64)) -> (f64, f64) {
    let iz = (-z.1, z.0);
    let niz = (z.1, -z.0);
    let e1 = c_exp(iz);
    let e2 = c_exp(niz);
    let diff = c_sub(e1, e2);
    c_div(diff, (0.0, 2.0))
}

/// 2x2 complex matrix multiplication
fn mat2_mul(a: &[[(f64, f64); 2]; 2], b: &[[(f64, f64); 2]; 2]) -> [[(f64, f64); 2]; 2] {
    [
        [
            c_add(c_mul(a[0][0], b[0][0]), c_mul(a[0][1], b[1][0])),
            c_add(c_mul(a[0][0], b[0][1]), c_mul(a[0][1], b[1][1])),
        ],
        [
            c_add(c_mul(a[1][0], b[0][0]), c_mul(a[1][1], b[1][0])),
            c_add(c_mul(a[1][0], b[0][1]), c_mul(a[1][1], b[1][1])),
        ],
    ]
}

/// Identity 2x2 complex matrix
fn mat2_identity() -> [[(f64, f64); 2]; 2] {
    [[(1.0, 0.0), (0.0, 0.0)], [(0.0, 0.0), (1.0, 0.0)]]
}

// ============================================================================
// Polarization enum
// ============================================================================

/// Polarization state for Fresnel calculations
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Polarization {
    /// s-polarization (TE, perpendicular to plane of incidence)
    S,
    /// p-polarization (TM, parallel to plane of incidence)
    P,
}

// ============================================================================
// Layer
// ============================================================================

/// A single layer in a multilayer thin film stack.
#[derive(Debug, Clone)]
pub struct Layer {
    /// Thickness in nanometers (0 for substrate/superstrate)
    pub thickness_nm: f64,
    /// Real part of refractive index
    pub n: f64,
    /// Extinction coefficient (imaginary part of complex refractive index)
    pub k: f64,
}

impl Layer {
    /// Create a new layer.
    pub fn new(thickness_nm: f64, n: f64, k: f64) -> Self {
        Self { thickness_nm, n, k }
    }

    /// Return complex refractive index as (n, k).
    pub fn complex_n(&self) -> (f64, f64) {
        (self.n, self.k)
    }
}

// ============================================================================
// Dispersion type for fitting
// ============================================================================

/// Dispersion model type for thickness fitting.
#[derive(Debug, Clone)]
pub enum DispersionType {
    /// Cauchy: n = A + B/lambda^2 + C/lambda^4
    Cauchy { a: f64, b: f64, c: f64 },
    /// Sellmeier: n^2 = 1 + sum(Ai * lambda^2 / (lambda^2 - Bi))
    Sellmeier { coeffs: Vec<(f64, f64)> },
    /// Constant refractive index
    Constant { n: f64, k: f64 },
}

// ============================================================================
// 1. EllipsometryData
// ============================================================================

/// Measured ellipsometric spectra: Psi(lambda) and Delta(lambda).
#[derive(Debug, Clone)]
pub struct EllipsometryData {
    /// Wavelengths in nanometers
    pub wavelength_nm: Vec<f64>,
    /// Psi angles in degrees
    pub psi_deg: Vec<f64>,
    /// Delta angles in degrees
    pub delta_deg: Vec<f64>,
}

impl EllipsometryData {
    /// Create new ellipsometry data from wavelength, Psi, and Delta arrays.
    /// All arrays must have the same length.
    pub fn new(wavelength_nm: Vec<f64>, psi_deg: Vec<f64>, delta_deg: Vec<f64>) -> Self {
        assert_eq!(wavelength_nm.len(), psi_deg.len());
        assert_eq!(wavelength_nm.len(), delta_deg.len());
        assert!(!wavelength_nm.is_empty());
        Self {
            wavelength_nm,
            psi_deg,
            delta_deg,
        }
    }

    /// Compute rho = tan(Psi) * exp(i*Delta) at the given index as (re, im).
    pub fn rho_at(&self, index: usize) -> (f64, f64) {
        let psi_rad = self.psi_deg[index].to_radians();
        let delta_rad = self.delta_deg[index].to_radians();
        let tan_psi = psi_rad.tan();
        (tan_psi * delta_rad.cos(), tan_psi * delta_rad.sin())
    }

    /// Number of spectral points.
    pub fn num_points(&self) -> usize {
        self.wavelength_nm.len()
    }

    /// Wavelength range (min, max) in nm.
    pub fn wavelength_range(&self) -> (f64, f64) {
        let min = self
            .wavelength_nm
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let max = self
            .wavelength_nm
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        (min, max)
    }
}

// ============================================================================
// 2. FresnelCoefficients
// ============================================================================

/// Fresnel reflection and transmission coefficients for interfaces
/// between media with complex refractive indices.
pub struct FresnelCoefficients;

impl FresnelCoefficients {
    /// Compute complex cos(theta) in medium 2 using Snell's law.
    /// n1*sin(theta1) = n2*sin(theta2)
    /// cos(theta2) = sqrt(1 - (n1/n2 * sin(theta1))^2)
    fn snell_cos_theta2(n1: (f64, f64), n2: (f64, f64), theta_rad: f64) -> (f64, f64) {
        let sin_t1 = theta_rad.sin();
        // n1_sin = n1 * sin(theta1)
        let n1_sin = c_scale(n1, sin_t1);
        // ratio = n1_sin / n2
        let ratio = c_div(n1_sin, n2);
        // ratio^2
        let ratio_sq = c_mul(ratio, ratio);
        // 1 - ratio^2
        let one_minus = c_sub((1.0, 0.0), ratio_sq);
        c_sqrt(one_minus)
    }

    /// s-polarization Fresnel reflection coefficient.
    /// rs = (n1*cos(theta1) - n2*cos(theta2)) / (n1*cos(theta1) + n2*cos(theta2))
    pub fn rs(n1: (f64, f64), n2: (f64, f64), theta_rad: f64) -> (f64, f64) {
        let cos_t1 = (theta_rad.cos(), 0.0);
        let cos_t2 = Self::snell_cos_theta2(n1, n2, theta_rad);
        let n1_cos1 = c_mul(n1, cos_t1);
        let n2_cos2 = c_mul(n2, cos_t2);
        c_div(c_sub(n1_cos1, n2_cos2), c_add(n1_cos1, n2_cos2))
    }

    /// p-polarization Fresnel reflection coefficient.
    /// rp = (n2*cos(theta1) - n1*cos(theta2)) / (n2*cos(theta1) + n1*cos(theta2))
    pub fn rp(n1: (f64, f64), n2: (f64, f64), theta_rad: f64) -> (f64, f64) {
        let cos_t1 = (theta_rad.cos(), 0.0);
        let cos_t2 = Self::snell_cos_theta2(n1, n2, theta_rad);
        let n2_cos1 = c_mul(n2, cos_t1);
        let n1_cos2 = c_mul(n1, cos_t2);
        c_div(c_sub(n2_cos1, n1_cos2), c_add(n2_cos1, n1_cos2))
    }

    /// s-polarization Fresnel transmission coefficient.
    /// ts = 2*n1*cos(theta1) / (n1*cos(theta1) + n2*cos(theta2))
    pub fn transmittance(n1: (f64, f64), n2: (f64, f64), theta: f64) -> (f64, f64) {
        let cos_t1 = (theta.cos(), 0.0);
        let cos_t2 = Self::snell_cos_theta2(n1, n2, theta);
        let n1_cos1 = c_mul(n1, cos_t1);
        let n2_cos2 = c_mul(n2, cos_t2);
        let numer = c_scale(n1_cos1, 2.0);
        c_div(numer, c_add(n1_cos1, n2_cos2))
    }

    /// Brewster's angle: theta_B = atan(n2 / n1) for real refractive indices.
    pub fn brewster_angle(n1: f64, n2: f64) -> f64 {
        (n2 / n1).atan()
    }
}

// ============================================================================
// 3. TransferMatrix
// ============================================================================

/// Transfer matrix method for computing reflection from multilayer thin film stacks.
pub struct TransferMatrix;

impl TransferMatrix {
    /// Interface (dynamical) matrix for transition from medium with refractive index n
    /// at incidence angle theta for given polarization.
    ///
    /// For s-pol: D = [[1, 1], [n*cos(theta), -n*cos(theta)]]
    /// For p-pol: D = [[cos(theta), cos(theta)], [n, -n]]
    ///
    /// We use the transfer matrix approach: M = D_in^{-1} * P * D_out
    pub fn interface_matrix(
        n1: (f64, f64),
        n2: (f64, f64),
        theta: f64,
        pol: Polarization,
    ) -> [[(f64, f64); 2]; 2] {
        // Compute complex cosine in each medium
        let cos_t1 = (theta.cos(), 0.0);
        let cos_t2 = FresnelCoefficients::snell_cos_theta2(n1, n2, theta);

        // Fresnel coefficients for this interface
        let (r, t) = match pol {
            Polarization::S => {
                let n1c1 = c_mul(n1, cos_t1);
                let n2c2 = c_mul(n2, cos_t2);
                let r = c_div(c_sub(n1c1, n2c2), c_add(n1c1, n2c2));
                let t = c_div(c_scale(n1c1, 2.0), c_add(n1c1, n2c2));
                (r, t)
            }
            Polarization::P => {
                let n2c1 = c_mul(n2, cos_t1);
                let n1c2 = c_mul(n1, cos_t2);
                let r = c_div(c_sub(n2c1, n1c2), c_add(n2c1, n1c2));
                let t = c_div(c_scale(c_mul(n1, cos_t1), 2.0), c_add(n2c1, n1c2));
                (r, t)
            }
        };

        // Interface matrix: (1/t) * [[1, r], [r, 1]]
        let inv_t = c_div((1.0, 0.0), t);
        [
            [inv_t, c_mul(inv_t, r)],
            [c_mul(inv_t, r), inv_t],
        ]
    }

    /// Propagation matrix for a layer of given complex refractive index, thickness, and wavelength.
    /// P = [[exp(-i*beta), 0], [0, exp(i*beta)]]
    /// where beta = 2*pi*n*d*cos(theta_in_layer)/lambda
    pub fn propagation_matrix(
        n: (f64, f64),
        thickness_nm: f64,
        wavelength_nm: f64,
        theta: f64,
    ) -> [[(f64, f64); 2]; 2] {
        // cos(theta) in this medium (from ambient with n_ambient ~ (1,0))
        // For the propagation matrix, we need cos_theta inside the layer
        // Using Snell's law from ambient: sin(theta_layer) = sin(theta)/n
        let sin_t = theta.sin();
        let ratio = c_div((sin_t, 0.0), n);
        let ratio_sq = c_mul(ratio, ratio);
        let cos_t_layer = c_sqrt(c_sub((1.0, 0.0), ratio_sq));

        // beta = 2*pi * n * d * cos(theta_layer) / lambda
        let beta = c_scale(c_mul(n, cos_t_layer), 2.0 * PI * thickness_nm / wavelength_nm);

        // exp(i*beta) and exp(-i*beta)
        let i_beta = (-beta.1, beta.0); // i * beta
        let neg_i_beta = (beta.1, -beta.0); // -i * beta
        [
            [c_exp(neg_i_beta), (0.0, 0.0)],
            [(0.0, 0.0), c_exp(i_beta)],
        ]
    }

    /// Compute reflection coefficients (r_s, r_p) for a multilayer stack.
    ///
    /// `layers` is ordered: [ambient, layer1, layer2, ..., substrate].
    /// The ambient and substrate have thickness_nm = 0 (semi-infinite).
    /// Internal layers have finite thickness.
    pub fn multilayer_reflection(
        layers: &[Layer],
        wavelength: f64,
        theta: f64,
    ) -> ((f64, f64), (f64, f64)) {
        if layers.len() < 2 {
            return ((0.0, 0.0), (0.0, 0.0));
        }

        let mut rs = (0.0, 0.0);
        let mut rp = (0.0, 0.0);

        for pol in &[Polarization::S, Polarization::P] {
            // Build total transfer matrix: M = I12 * P2 * I23 * P3 * ... * I_{N-1,N}
            let mut m = mat2_identity();

            for i in 0..layers.len() - 1 {
                let n_i = layers[i].complex_n();
                let n_j = layers[i + 1].complex_n();

                // Interface matrix between layer i and i+1
                let imat = Self::interface_matrix(n_i, n_j, theta, *pol);
                m = mat2_mul(&m, &imat);

                // Propagation through layer i+1 (if not the substrate)
                if i + 1 < layers.len() - 1 && layers[i + 1].thickness_nm > 0.0 {
                    let pmat = Self::propagation_matrix(
                        n_j,
                        layers[i + 1].thickness_nm,
                        wavelength,
                        theta,
                    );
                    m = mat2_mul(&m, &pmat);
                }
            }

            // Reflection coefficient r = M[1][0] / M[0][0]
            let r = c_div(m[1][0], m[0][0]);
            match pol {
                Polarization::S => rs = r,
                Polarization::P => rp = r,
            }
        }

        (rs, rp)
    }

    /// Compute Psi (degrees) and Delta (degrees) from rp and rs.
    /// rho = rp/rs = tan(Psi) * exp(i*Delta)
    pub fn psi_delta_from_rho(rp: (f64, f64), rs: (f64, f64)) -> (f64, f64) {
        let rho = c_div(rp, rs);
        let psi_rad = c_abs(rho).atan();
        let delta_rad = c_arg(rho);
        (psi_rad.to_degrees(), delta_rad.to_degrees())
    }
}

// ============================================================================
// 4. DispersionModels
// ============================================================================

/// Optical constant dispersion models: n(lambda) and k(lambda).
pub struct DispersionModels;

impl DispersionModels {
    /// Cauchy model for transparent films: n = A + B/lambda^2 + C/lambda^4
    /// wavelength_nm in nanometers; coefficients assume um internally.
    pub fn cauchy(wavelength_nm: f64, a: f64, b: f64, c: f64) -> f64 {
        let lam_um = wavelength_nm / 1000.0;
        let lam2 = lam_um * lam_um;
        let lam4 = lam2 * lam2;
        a + b / lam2 + c / lam4
    }

    /// Sellmeier model: n^2 = 1 + sum_i(A_i * lambda^2 / (lambda^2 - B_i))
    /// where B_i are squared resonance wavelengths. lambda in nm, coefficients in um^2.
    pub fn sellmeier(wavelength_nm: f64, coeffs: &[(f64, f64)]) -> f64 {
        let lam_um = wavelength_nm / 1000.0;
        let lam2 = lam_um * lam_um;
        let mut n_sq = 1.0;
        for &(ai, bi) in coeffs {
            n_sq += ai * lam2 / (lam2 - bi);
        }
        if n_sq > 0.0 {
            n_sq.sqrt()
        } else {
            1.0
        }
    }

    /// Tauc-Lorentz model for amorphous materials.
    /// Returns (epsilon1, epsilon2) at given photon energy.
    /// epsilon2 = (A * E0 * C * (E - Eg)^2) / ((E^2 - E0^2)^2 + C^2*E^2) / E for E > Eg
    pub fn tauc_lorentz(
        energy_ev: f64,
        amplitude: f64,
        center_ev: f64,
        broadening_ev: f64,
        bandgap_ev: f64,
    ) -> (f64, f64) {
        if energy_ev <= bandgap_ev || energy_ev <= 0.0 {
            return (1.0, 0.0);
        }

        let e = energy_ev;
        let e0 = center_ev;
        let c = broadening_ev;
        let eg = bandgap_ev;
        let a = amplitude;

        // epsilon2
        let numer = a * e0 * c * (e - eg) * (e - eg);
        let denom = ((e * e - e0 * e0).powi(2) + c * c * e * e) * e;
        let eps2 = if denom.abs() > 1e-30 {
            numer / denom
        } else {
            0.0
        };

        // epsilon1 from Kramers-Kronig (simplified constant offset model)
        // For a full KK transform we would need numerical integration.
        // Approximate: eps1 ~ 1 + (2/pi) * P-integral
        // Use a simplified single-oscillator approximation:
        let eps1 = 1.0 + (2.0 / PI) * a * e0 * c / (e0 * e0 - e * e + c * e * 0.5).max(0.01);

        (eps1, eps2)
    }

    /// Drude model for metals: epsilon = 1 - wp^2 / (E^2 + i*gamma*E)
    /// Returns (epsilon1, epsilon2).
    pub fn drude(energy_ev: f64, plasma_freq: f64, damping: f64) -> (f64, f64) {
        let e = energy_ev;
        let wp2 = plasma_freq * plasma_freq;

        // epsilon = 1 - wp^2 / (E^2 + i*gamma*E)
        // denominator = E^2 + i*gamma*E  -> (E^2, gamma*E)
        let denom = (e * e, damping * e);
        let denom_sq = c_abs_sq(denom);
        if denom_sq < 1e-30 {
            return (1.0, 0.0);
        }

        // wp^2 / denom = wp^2 * conj(denom) / |denom|^2
        let conj_d = c_conj(denom);
        let ratio = c_scale(conj_d, wp2 / denom_sq);

        (1.0 - ratio.0, -ratio.1)
    }

    /// Lorentz oscillator model.
    /// epsilon = A * E0^2 / (E0^2 - E^2 - i*C*E)
    /// Returns (epsilon1, epsilon2).
    pub fn lorentz_oscillator(
        energy_ev: f64,
        amplitude: f64,
        center_ev: f64,
        broadening_ev: f64,
    ) -> (f64, f64) {
        let e = energy_ev;
        let e0 = center_ev;
        let c = broadening_ev;

        // denominator = (E0^2 - E^2) - i*C*E
        let denom = (e0 * e0 - e * e, -c * e);
        let numer = (amplitude * e0 * e0, 0.0);
        c_div(numer, denom)
    }

    /// Convert dielectric function (epsilon1, epsilon2) to refractive index (n, k).
    /// n + ik = sqrt(epsilon1 + i*epsilon2)
    pub fn n_k_from_epsilon(eps1: f64, eps2: f64) -> (f64, f64) {
        let eps = (eps1, eps2);
        let nk = c_sqrt(eps);
        // Ensure n >= 0 and k >= 0
        (nk.0.abs(), nk.1.abs())
    }

    /// Convert wavelength in nm to photon energy in eV.
    /// E = hc/lambda = 1239.84198 / lambda_nm
    pub fn wavelength_to_ev(wavelength_nm: f64) -> f64 {
        1239.84198 / wavelength_nm
    }

    /// Convert photon energy in eV to wavelength in nm.
    pub fn ev_to_wavelength(energy_ev: f64) -> f64 {
        1239.84198 / energy_ev
    }
}

// ============================================================================
// 5. ThicknessFitter
// ============================================================================

/// Result of a thickness fit.
#[derive(Debug, Clone)]
pub struct ThicknessFitResult {
    /// Fitted thickness in nanometers
    pub thickness_nm: f64,
    /// Mean squared error of the fit
    pub mse: f64,
    /// Fitted n values at each wavelength
    pub n_values: Vec<f64>,
    /// Fitted k values at each wavelength
    pub k_values: Vec<f64>,
}

/// Single-layer thickness determination via grid search + refinement.
pub struct ThicknessFitter;

impl ThicknessFitter {
    /// Fit thickness for a single transparent film on a substrate.
    ///
    /// Uses grid search over thickness range followed by bisection refinement.
    ///
    /// MSE = sqrt(sum[(Psi_m - Psi_c)^2 + (Delta_m - Delta_c)^2] / (2N - M))
    pub fn fit_thickness(
        data: &EllipsometryData,
        substrate_n: f64,
        film_model: DispersionType,
        angle_deg: f64,
    ) -> ThicknessFitResult {
        let theta = angle_deg.to_radians();
        let n_pts = data.num_points();

        // Grid search: 0 to 2000 nm in 1 nm steps
        let mut best_thickness = 0.0;
        let mut best_mse = f64::MAX;

        for t_nm in 0..=2000 {
            let thickness = t_nm as f64;
            let mse = Self::compute_mse(data, substrate_n, &film_model, thickness, theta);
            if mse < best_mse {
                best_mse = mse;
                best_thickness = thickness;
            }
        }

        // Refine with 0.1 nm steps around best
        let start = (best_thickness - 2.0).max(0.0);
        let end = best_thickness + 2.0;
        let mut t = start;
        while t <= end {
            let mse = Self::compute_mse(data, substrate_n, &film_model, t, theta);
            if mse < best_mse {
                best_mse = mse;
                best_thickness = t;
            }
            t += 0.1;
        }

        // Compute n, k at each wavelength for the best fit
        let mut n_values = Vec::with_capacity(n_pts);
        let mut k_values = Vec::with_capacity(n_pts);
        for i in 0..n_pts {
            let wl = data.wavelength_nm[i];
            let (n, k) = Self::film_nk(&film_model, wl);
            n_values.push(n);
            k_values.push(k);
        }

        ThicknessFitResult {
            thickness_nm: best_thickness,
            mse: best_mse,
            n_values,
            k_values,
        }
    }

    /// Compute MSE for a given thickness.
    fn compute_mse(
        data: &EllipsometryData,
        substrate_n: f64,
        film_model: &DispersionType,
        thickness: f64,
        theta: f64,
    ) -> f64 {
        let n_pts = data.num_points();
        let mut sum_sq = 0.0;

        for i in 0..n_pts {
            let wl = data.wavelength_nm[i];
            let (film_n, film_k) = Self::film_nk(film_model, wl);

            let layers = vec![
                Layer::new(0.0, 1.0, 0.0),         // ambient (air)
                Layer::new(thickness, film_n, film_k), // film
                Layer::new(0.0, substrate_n, 0.0),  // substrate
            ];

            let (rs, rp) = TransferMatrix::multilayer_reflection(&layers, wl, theta);
            let (psi_c, delta_c) = TransferMatrix::psi_delta_from_rho(rp, rs);

            let dpsi = data.psi_deg[i] - psi_c;
            let ddelta = data.delta_deg[i] - delta_c;
            sum_sq += dpsi * dpsi + ddelta * ddelta;
        }

        // MSE = sqrt(sum / (2N - M)), M = 1 (thickness)
        let denom = (2 * n_pts).saturating_sub(1).max(1) as f64;
        (sum_sq / denom).sqrt()
    }

    /// Get (n, k) for film from dispersion model at given wavelength.
    fn film_nk(model: &DispersionType, wavelength_nm: f64) -> (f64, f64) {
        match model {
            DispersionType::Cauchy { a, b, c } => {
                (DispersionModels::cauchy(wavelength_nm, *a, *b, *c), 0.0)
            }
            DispersionType::Sellmeier { coeffs } => {
                (DispersionModels::sellmeier(wavelength_nm, coeffs), 0.0)
            }
            DispersionType::Constant { n, k } => (*n, *k),
        }
    }
}

// ============================================================================
// 6. MaterialDatabase
// ============================================================================

/// Common material optical constants.
///
/// Values are approximate Cauchy/Sellmeier fits for the visible/near-UV range.
/// For absorbing materials (metals), simplified Drude/Lorentz-based tabulations are used.
pub struct MaterialDatabase;

impl MaterialDatabase {
    /// Silicon (crystalline) optical constants.
    /// Approximate fit valid ~300-800 nm.
    pub fn silicon(wavelength_nm: f64) -> (f64, f64) {
        // Si is absorbing in visible range, strongly dispersive
        let ev = DispersionModels::wavelength_to_ev(wavelength_nm);
        // Simplified model: two Lorentz oscillators + offset
        let (e1a, e2a) = DispersionModels::lorentz_oscillator(ev, 20.0, 3.4, 0.3);
        let (e1b, e2b) = DispersionModels::lorentz_oscillator(ev, 15.0, 4.3, 0.5);
        let eps1 = 1.0 + e1a + e1b;
        let eps2 = e2a + e2b;
        DispersionModels::n_k_from_epsilon(eps1, eps2)
    }

    /// Silicon dioxide (SiO2) optical constants.
    /// Cauchy model: n ~ 1.46 in visible, transparent.
    pub fn silicon_dioxide(wavelength_nm: f64) -> (f64, f64) {
        let n = DispersionModels::cauchy(wavelength_nm, 1.4580, 0.00354, 0.0);
        (n, 0.0)
    }

    /// Silicon nitride (Si3N4) optical constants.
    /// Cauchy model: n ~ 2.0 in visible, transparent.
    pub fn silicon_nitride(wavelength_nm: f64) -> (f64, f64) {
        let n = DispersionModels::cauchy(wavelength_nm, 1.98, 0.0155, 0.0);
        (n, 0.0)
    }

    /// Gold (Au) optical constants via Drude model.
    pub fn gold(wavelength_nm: f64) -> (f64, f64) {
        let ev = DispersionModels::wavelength_to_ev(wavelength_nm);
        // Au: plasma frequency ~9.0 eV, damping ~0.07 eV
        let (eps1, eps2) = DispersionModels::drude(ev, 9.0, 0.07);
        // Add interband transition contribution
        let (e1ib, e2ib) = DispersionModels::lorentz_oscillator(ev, 6.0, 2.5, 0.6);
        DispersionModels::n_k_from_epsilon(eps1 + e1ib, eps2 + e2ib)
    }

    /// Aluminum (Al) optical constants via Drude model.
    pub fn aluminum(wavelength_nm: f64) -> (f64, f64) {
        let ev = DispersionModels::wavelength_to_ev(wavelength_nm);
        // Al: plasma frequency ~15.0 eV, damping ~0.6 eV
        let (eps1, eps2) = DispersionModels::drude(ev, 15.0, 0.6);
        DispersionModels::n_k_from_epsilon(eps1, eps2)
    }

    /// Titanium dioxide (TiO2) optical constants.
    /// Cauchy model: n ~ 2.5 in visible, transparent.
    pub fn titanium_dioxide(wavelength_nm: f64) -> (f64, f64) {
        let n = DispersionModels::cauchy(wavelength_nm, 2.45, 0.04, 0.0);
        (n, 0.0)
    }
}

// ============================================================================
// 7. EffectiveMediumApproximation
// ============================================================================

/// Effective medium approximation models for mixed/composite materials.
pub struct EffectiveMediumApproximation;

impl EffectiveMediumApproximation {
    /// Convert (n, k) to complex dielectric constant epsilon = (n + ik)^2.
    fn nk_to_eps(nk: (f64, f64)) -> (f64, f64) {
        let n = nk.0;
        let k = nk.1;
        (n * n - k * k, 2.0 * n * k)
    }

    /// Convert complex epsilon to (n, k).
    fn eps_to_nk(eps: (f64, f64)) -> (f64, f64) {
        DispersionModels::n_k_from_epsilon(eps.0, eps.1)
    }

    /// Maxwell-Garnett effective medium approximation.
    ///
    /// For spherical inclusions in a host matrix:
    /// (eps_eff - eps_h) / (eps_eff + 2*eps_h) = f * (eps_i - eps_h) / (eps_i + 2*eps_h)
    pub fn maxwell_garnett(
        n_host: (f64, f64),
        n_inclusion: (f64, f64),
        volume_fraction: f64,
    ) -> (f64, f64) {
        let eps_h = Self::nk_to_eps(n_host);
        let eps_i = Self::nk_to_eps(n_inclusion);
        let f = volume_fraction;

        // Numerator: eps_i - eps_h
        let num_inner = c_sub(eps_i, eps_h);
        // Denominator: eps_i + 2*eps_h
        let den_inner = c_add(eps_i, c_scale(eps_h, 2.0));

        let ratio = c_div(num_inner, den_inner);

        // eps_eff = eps_h * (1 + 2*f*ratio) / (1 - f*ratio)
        let f_ratio = c_scale(ratio, f);
        let numer = c_add((1.0, 0.0), c_scale(f_ratio, 2.0));
        let denom = c_sub((1.0, 0.0), f_ratio);
        let eps_eff = c_mul(eps_h, c_div(numer, denom));

        Self::eps_to_nk(eps_eff)
    }

    /// Bruggeman effective medium approximation (self-consistent).
    ///
    /// Solves: f_a * (eps_a - eps_eff)/(eps_a + 2*eps_eff) + (1-f_a) * (eps_b - eps_eff)/(eps_b + 2*eps_eff) = 0
    ///
    /// Uses iterative solution starting from linear mixing.
    pub fn bruggeman(
        n_a: (f64, f64),
        n_b: (f64, f64),
        fraction_a: f64,
    ) -> (f64, f64) {
        let eps_a = Self::nk_to_eps(n_a);
        let eps_b = Self::nk_to_eps(n_b);
        let fa = fraction_a;
        let fb = 1.0 - fraction_a;

        // Start with linear mixing
        let mut eps_eff = c_add(c_scale(eps_a, fa), c_scale(eps_b, fb));

        // Iterate to self-consistency
        for _ in 0..50 {
            let term_a = c_div(
                c_sub(eps_a, eps_eff),
                c_add(eps_a, c_scale(eps_eff, 2.0)),
            );
            let term_b = c_div(
                c_sub(eps_b, eps_eff),
                c_add(eps_b, c_scale(eps_eff, 2.0)),
            );
            let residual = c_add(c_scale(term_a, fa), c_scale(term_b, fb));

            // Newton-like update: eps_eff += alpha * residual * eps_eff
            let correction = c_mul(residual, eps_eff);
            eps_eff = c_add(eps_eff, c_scale(correction, 0.3));

            if c_abs(residual) < 1e-10 {
                break;
            }
        }

        Self::eps_to_nk(eps_eff)
    }

    /// Simple linear mixing of optical constants.
    /// n_eff = f_a * n_a + (1 - f_a) * n_b
    pub fn linear_mixing(
        n_a: (f64, f64),
        n_b: (f64, f64),
        fraction_a: f64,
    ) -> (f64, f64) {
        let fb = 1.0 - fraction_a;
        (
            fraction_a * n_a.0 + fb * n_b.0,
            fraction_a * n_a.1 + fb * n_b.1,
        )
    }
}

// ============================================================================
// 8. MultiAngleAnalysis
// ============================================================================

/// Multi-angle ellipsometry data set.
#[derive(Debug, Clone)]
pub struct MultiAngleData {
    /// Individual data sets at different angles
    pub data_sets: Vec<EllipsometryData>,
    /// Angles of incidence in degrees
    pub angles_deg: Vec<f64>,
}

/// Layer model for multi-angle fitting.
#[derive(Debug, Clone)]
pub struct LayerModel {
    /// Layer stack (ambient, film layers, substrate)
    pub layers: Vec<Layer>,
    /// Which layer thicknesses are free parameters (indices into layers)
    pub fit_indices: Vec<usize>,
}

/// Fit result from multi-angle analysis.
#[derive(Debug, Clone)]
pub struct FitResult {
    /// Fitted layer thicknesses (for fit_indices)
    pub thicknesses: Vec<f64>,
    /// Overall MSE
    pub mse: f64,
}

/// Variable angle spectroscopic ellipsometry analysis.
pub struct MultiAngleAnalysis;

impl MultiAngleAnalysis {
    /// Combine data sets from multiple angles into a MultiAngleData structure.
    pub fn combine_angles(
        data_sets: &[EllipsometryData],
        angles_deg: &[f64],
    ) -> MultiAngleData {
        assert_eq!(data_sets.len(), angles_deg.len());
        MultiAngleData {
            data_sets: data_sets.to_vec(),
            angles_deg: angles_deg.to_vec(),
        }
    }

    /// Fit multilayer model using multi-angle data.
    /// Currently supports single free thickness parameter via grid search.
    pub fn fit_multi_angle(
        data: &MultiAngleData,
        model: &LayerModel,
    ) -> FitResult {
        if model.fit_indices.is_empty() {
            return FitResult {
                thicknesses: vec![],
                mse: 0.0,
            };
        }

        // Grid search over first fit parameter
        let idx = model.fit_indices[0];
        let mut best_t = 0.0;
        let mut best_mse = f64::MAX;

        for t_step in 0..=2000 {
            let t = t_step as f64;
            let mut total_mse = 0.0;
            let mut total_pts = 0usize;

            for (ds_idx, ds) in data.data_sets.iter().enumerate() {
                let theta = data.angles_deg[ds_idx].to_radians();
                let mut layers = model.layers.clone();
                layers[idx].thickness_nm = t;

                for i in 0..ds.num_points() {
                    let wl = ds.wavelength_nm[i];
                    let (rs, rp) = TransferMatrix::multilayer_reflection(&layers, wl, theta);
                    let (psi_c, delta_c) = TransferMatrix::psi_delta_from_rho(rp, rs);

                    let dpsi = ds.psi_deg[i] - psi_c;
                    let ddelta = ds.delta_deg[i] - delta_c;
                    total_mse += dpsi * dpsi + ddelta * ddelta;
                    total_pts += 1;
                }
            }

            let mse = if total_pts > 1 {
                (total_mse / (2 * total_pts - 1) as f64).sqrt()
            } else {
                total_mse.sqrt()
            };

            if mse < best_mse {
                best_mse = mse;
                best_t = t;
            }
        }

        FitResult {
            thicknesses: vec![best_t],
            mse: best_mse,
        }
    }

    /// Compute pseudo-dielectric function from bare substrate measurement.
    /// <epsilon> = sin^2(theta) * [1 + tan^2(theta) * ((1-rho)/(1+rho))^2]
    /// where rho = tan(Psi)*exp(i*Delta)
    pub fn pseudo_dielectric(psi: f64, delta: f64, angle_deg: f64) -> (f64, f64) {
        let theta = angle_deg.to_radians();
        let psi_rad = psi.to_radians();
        let delta_rad = delta.to_radians();

        let rho = (psi_rad.tan() * delta_rad.cos(), psi_rad.tan() * delta_rad.sin());
        let one_minus_rho = c_sub((1.0, 0.0), rho);
        let one_plus_rho = c_add((1.0, 0.0), rho);
        let ratio = c_div(one_minus_rho, one_plus_rho);
        let ratio_sq = c_mul(ratio, ratio);

        let sin2 = theta.sin() * theta.sin();
        let tan2 = theta.tan() * theta.tan();

        let bracket = c_add((1.0, 0.0), c_scale(ratio_sq, tan2));
        let eps = c_scale(bracket, sin2);
        (eps.0, eps.1)
    }
}

// ============================================================================
// 9. FilmQuality
// ============================================================================

/// Film and measurement quality assessment tools.
pub struct FilmQuality;

impl FilmQuality {
    /// Estimate depolarization at each wavelength.
    ///
    /// A simple heuristic: measure how far Psi/Delta deviate from
    /// expected smooth behavior. Returns a depolarization index (0 = ideal, 1 = fully depolarized).
    /// For ideal measurement on a perfect film, depolarization ~ 0.
    pub fn depolarization(data: &EllipsometryData) -> Vec<f64> {
        let n = data.num_points();
        if n < 3 {
            return vec![0.0; n];
        }

        let mut dep = vec![0.0; n];

        // Use local smoothness: compare each point to average of neighbors
        for i in 1..n - 1 {
            let psi_avg = (data.psi_deg[i - 1] + data.psi_deg[i + 1]) / 2.0;
            let delta_avg = (data.delta_deg[i - 1] + data.delta_deg[i + 1]) / 2.0;
            let dpsi = (data.psi_deg[i] - psi_avg).abs();
            let ddelta = (data.delta_deg[i] - delta_avg).abs();
            // Normalize to [0, 1] range
            dep[i] = ((dpsi + ddelta) / 90.0).min(1.0);
        }
        dep[0] = dep[1];
        dep[n - 1] = dep[n - 2];

        dep
    }

    /// Check thickness uniformity from a Psi spatial map.
    ///
    /// Returns the coefficient of variation (std_dev / mean) of the Psi values,
    /// which correlates with thickness non-uniformity.
    pub fn uniformity_check(psi_map: &[Vec<f64>]) -> f64 {
        let mut all_vals = Vec::new();
        for row in psi_map {
            for &v in row {
                all_vals.push(v);
            }
        }
        if all_vals.is_empty() {
            return 0.0;
        }

        let n = all_vals.len() as f64;
        let mean = all_vals.iter().sum::<f64>() / n;
        if mean.abs() < 1e-10 {
            return 0.0;
        }

        let variance = all_vals.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n;
        variance.sqrt() / mean.abs()
    }

    /// Sensitivity analysis: dPsi/dp and dDelta/dp for a given parameter.
    ///
    /// Computes numerical partial derivatives of Psi and Delta with respect
    /// to the thickness of the specified layer.
    pub fn sensitivity_analysis(model: &LayerModel, param_index: usize) -> Vec<f64> {
        if param_index >= model.fit_indices.len() {
            return vec![];
        }
        let layer_idx = model.fit_indices[param_index];
        let base_thickness = model.layers[layer_idx].thickness_nm;
        let dt = 1.0; // 1 nm perturbation

        // Use a default wavelength set and angle
        let wavelengths: Vec<f64> = (300..=800).step_by(10).map(|w| w as f64).collect();
        let theta = 70.0_f64.to_radians();
        let mut sensitivities = Vec::with_capacity(wavelengths.len());

        for &wl in &wavelengths {
            // Base calculation
            let (rs0, rp0) = TransferMatrix::multilayer_reflection(&model.layers, wl, theta);
            let (psi0, delta0) = TransferMatrix::psi_delta_from_rho(rp0, rs0);

            // Perturbed calculation
            let mut perturbed = model.layers.clone();
            perturbed[layer_idx].thickness_nm = base_thickness + dt;
            let (rs1, rp1) = TransferMatrix::multilayer_reflection(&perturbed, wl, theta);
            let (psi1, delta1) = TransferMatrix::psi_delta_from_rho(rp1, rs1);

            let dpsi = (psi1 - psi0).abs() / dt;
            let ddelta = (delta1 - delta0).abs() / dt;
            sensitivities.push(dpsi + ddelta);
        }

        sensitivities
    }
}

// ============================================================================
// 10. EllipsometrySimulator
// ============================================================================

/// Generate synthetic ellipsometry data from a multilayer model.
pub struct EllipsometrySimulator;

impl EllipsometrySimulator {
    /// Simulate Psi and Delta spectra for a multilayer structure.
    pub fn simulate(
        layers: &[Layer],
        substrate: (f64, f64),
        wavelengths: &[f64],
        angle_deg: f64,
    ) -> EllipsometryData {
        let theta = angle_deg.to_radians();
        let mut psi_deg = Vec::with_capacity(wavelengths.len());
        let mut delta_deg = Vec::with_capacity(wavelengths.len());

        // Build full stack: ambient + given layers + substrate
        let mut full_stack = vec![Layer::new(0.0, 1.0, 0.0)]; // air ambient
        full_stack.extend(layers.iter().cloned());
        full_stack.push(Layer::new(0.0, substrate.0, substrate.1));

        for &wl in wavelengths {
            let (rs, rp) = TransferMatrix::multilayer_reflection(&full_stack, wl, theta);
            let (psi, delta) = TransferMatrix::psi_delta_from_rho(rp, rs);
            psi_deg.push(psi);
            delta_deg.push(delta);
        }

        EllipsometryData::new(wavelengths.to_vec(), psi_deg, delta_deg)
    }

    /// Add Gaussian noise to ellipsometry data.
    /// Uses a simple deterministic pseudo-random generator for reproducibility.
    pub fn add_noise(
        data: &EllipsometryData,
        psi_noise_deg: f64,
        delta_noise_deg: f64,
    ) -> EllipsometryData {
        let n = data.num_points();
        let mut psi_noisy = Vec::with_capacity(n);
        let mut delta_noisy = Vec::with_capacity(n);

        // Simple deterministic PRNG (LCG-based Box-Muller)
        let mut seed: u64 = 42;
        let next_rand = |s: &mut u64| -> f64 {
            *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (*s >> 33) as f64 / (1u64 << 31) as f64
        };

        for i in 0..n {
            // Box-Muller transform for Gaussian noise
            let u1 = next_rand(&mut seed).max(1e-10);
            let u2 = next_rand(&mut seed);
            let g1 = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos();
            let g2 = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).sin();

            psi_noisy.push(data.psi_deg[i] + g1 * psi_noise_deg);
            delta_noisy.push(data.delta_deg[i] + g2 * delta_noise_deg);
        }

        EllipsometryData::new(data.wavelength_nm.clone(), psi_noisy, delta_noisy)
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-6;
    const TOL_LOOSE: f64 = 1e-3;
    const TOL_VERY_LOOSE: f64 = 0.1;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn c_approx_eq(a: (f64, f64), b: (f64, f64), tol: f64) -> bool {
        (a.0 - b.0).abs() < tol && (a.1 - b.1).abs() < tol
    }

    // ---- Complex arithmetic tests ----

    #[test]
    fn test_c_add() {
        let r = c_add((1.0, 2.0), (3.0, 4.0));
        assert!(approx_eq(r.0, 4.0, TOL));
        assert!(approx_eq(r.1, 6.0, TOL));
    }

    #[test]
    fn test_c_sub() {
        let r = c_sub((5.0, 3.0), (2.0, 1.0));
        assert!(approx_eq(r.0, 3.0, TOL));
        assert!(approx_eq(r.1, 2.0, TOL));
    }

    #[test]
    fn test_c_mul() {
        // (1+2i)(3+4i) = (3-8) + (4+6)i = -5 + 10i
        let r = c_mul((1.0, 2.0), (3.0, 4.0));
        assert!(approx_eq(r.0, -5.0, TOL));
        assert!(approx_eq(r.1, 10.0, TOL));
    }

    #[test]
    fn test_c_div() {
        // (1+2i)/(3+4i) = (1+2i)(3-4i)/25 = (3+8+(-4+6)i)/25 = (11+2i)/25
        let r = c_div((1.0, 2.0), (3.0, 4.0));
        assert!(approx_eq(r.0, 11.0 / 25.0, TOL));
        assert!(approx_eq(r.1, 2.0 / 25.0, TOL));
    }

    #[test]
    fn test_c_abs_and_arg() {
        let z = (3.0, 4.0);
        assert!(approx_eq(c_abs(z), 5.0, TOL));
        assert!(approx_eq(c_arg(z), (4.0_f64).atan2(3.0), TOL));
    }

    #[test]
    fn test_c_exp() {
        // e^(i*pi) = -1 + 0i
        let r = c_exp((0.0, PI));
        assert!(approx_eq(r.0, -1.0, TOL));
        assert!(approx_eq(r.1, 0.0, TOL));
    }

    #[test]
    fn test_c_sqrt() {
        // sqrt(3+4i) should satisfy z^2 = 3+4i
        let z = (3.0, 4.0);
        let s = c_sqrt(z);
        let check = c_mul(s, s);
        assert!(approx_eq(check.0, 3.0, TOL));
        assert!(approx_eq(check.1, 4.0, TOL));
    }

    #[test]
    fn test_c_conj() {
        let z = (3.0, 4.0);
        let c = c_conj(z);
        assert!(approx_eq(c.0, 3.0, TOL));
        assert!(approx_eq(c.1, -4.0, TOL));
    }

    #[test]
    fn test_c_cos_zero() {
        let r = c_cos((0.0, 0.0));
        assert!(approx_eq(r.0, 1.0, TOL));
        assert!(approx_eq(r.1, 0.0, TOL));
    }

    #[test]
    fn test_c_sin_zero() {
        let r = c_sin((0.0, 0.0));
        assert!(approx_eq(r.0, 0.0, TOL));
        assert!(approx_eq(r.1, 0.0, TOL));
    }

    #[test]
    fn test_mat2_identity_mul() {
        let a = [[(2.0, 1.0), (3.0, 0.0)], [(0.0, -1.0), (1.0, 1.0)]];
        let id = mat2_identity();
        let r = mat2_mul(&a, &id);
        assert!(c_approx_eq(r[0][0], a[0][0], TOL));
        assert!(c_approx_eq(r[1][1], a[1][1], TOL));
    }

    // ---- EllipsometryData tests ----

    #[test]
    fn test_ellipsometry_data_new() {
        let data = EllipsometryData::new(
            vec![400.0, 500.0, 600.0],
            vec![30.0, 35.0, 40.0],
            vec![100.0, 110.0, 120.0],
        );
        assert_eq!(data.num_points(), 3);
    }

    #[test]
    fn test_ellipsometry_data_wavelength_range() {
        let data = EllipsometryData::new(
            vec![300.0, 500.0, 800.0],
            vec![30.0, 35.0, 40.0],
            vec![100.0, 110.0, 120.0],
        );
        let (min, max) = data.wavelength_range();
        assert!(approx_eq(min, 300.0, TOL));
        assert!(approx_eq(max, 800.0, TOL));
    }

    #[test]
    fn test_rho_at_psi45_delta0() {
        // Psi=45 deg -> tan(45)=1, Delta=0 -> exp(0)=1 -> rho = (1, 0)
        let data = EllipsometryData::new(vec![500.0], vec![45.0], vec![0.0]);
        let rho = data.rho_at(0);
        assert!(approx_eq(rho.0, 1.0, TOL));
        assert!(approx_eq(rho.1, 0.0, TOL));
    }

    #[test]
    fn test_rho_at_psi45_delta90() {
        // Psi=45 -> tan=1, Delta=90 -> exp(i*pi/2) = i -> rho = (0, 1)
        let data = EllipsometryData::new(vec![500.0], vec![45.0], vec![90.0]);
        let rho = data.rho_at(0);
        assert!(approx_eq(rho.0, 0.0, TOL));
        assert!(approx_eq(rho.1, 1.0, TOL));
    }

    // ---- FresnelCoefficients tests ----

    #[test]
    fn test_fresnel_normal_incidence() {
        // At normal incidence, rs = (n1-n2)/(n1+n2), rp = (n2-n1)/(n2+n1) = -rs
        let n1 = (1.0, 0.0);
        let n2 = (1.5, 0.0);
        let rs = FresnelCoefficients::rs(n1, n2, 0.0);
        let rp = FresnelCoefficients::rp(n1, n2, 0.0);
        // rs = (1-1.5)/(1+1.5) = -0.2
        assert!(approx_eq(rs.0, -0.2, TOL));
        // rp should equal -rs at normal incidence
        assert!(approx_eq(rp.0, 0.2, TOL));
    }

    #[test]
    fn test_fresnel_rs_rp_complement() {
        // |rs|^2 + |ts|^2 * (n2*cos_t2)/(n1*cos_t1) = 1 (energy conservation)
        let n1 = (1.0, 0.0);
        let n2 = (1.5, 0.0);
        let theta = 30.0_f64.to_radians();
        let rs = FresnelCoefficients::rs(n1, n2, theta);
        let ts = FresnelCoefficients::transmittance(n1, n2, theta);
        // Both should be real for real n values
        assert!(rs.1.abs() < TOL_LOOSE);
        assert!(ts.0 > 0.0); // transmission positive
    }

    #[test]
    fn test_brewster_angle() {
        let n1 = 1.0;
        let n2 = 1.5;
        let theta_b = FresnelCoefficients::brewster_angle(n1, n2);
        // At Brewster's angle, rp should be zero
        let rp = FresnelCoefficients::rp((n1, 0.0), (n2, 0.0), theta_b);
        assert!(c_abs(rp) < TOL_LOOSE);
    }

    #[test]
    fn test_fresnel_total_internal_reflection() {
        // Beyond critical angle, |r| = 1
        let n1 = (1.5, 0.0);
        let n2 = (1.0, 0.0);
        let theta = 60.0_f64.to_radians(); // Beyond critical angle ~41.8 deg
        let rs = FresnelCoefficients::rs(n1, n2, theta);
        assert!(approx_eq(c_abs(rs), 1.0, TOL_LOOSE));
    }

    #[test]
    fn test_fresnel_complex_n() {
        // With absorbing medium, Fresnel coefficients should still work
        let n1 = (1.0, 0.0);
        let n2 = (3.5, 0.5); // e.g., silicon-like
        let theta = 70.0_f64.to_radians();
        let rs = FresnelCoefficients::rs(n1, n2, theta);
        let rp = FresnelCoefficients::rp(n1, n2, theta);
        // Both should have |r| < 1
        assert!(c_abs(rs) < 1.0);
        assert!(c_abs(rp) < 1.0);
        assert!(c_abs(rs) > 0.0);
        assert!(c_abs(rp) > 0.0);
    }

    // ---- TransferMatrix tests ----

    #[test]
    fn test_transfer_matrix_bare_substrate() {
        // No film: just air/Si interface
        let layers = vec![
            Layer::new(0.0, 1.0, 0.0),   // air
            Layer::new(0.0, 3.87, 0.02),  // silicon
        ];
        let theta = 70.0_f64.to_radians();
        let (rs, rp) = TransferMatrix::multilayer_reflection(&layers, 632.8, theta);
        // Should get reasonable Psi, Delta
        let (psi, delta) = TransferMatrix::psi_delta_from_rho(rp, rs);
        assert!(psi > 0.0 && psi < 90.0);
        assert!(delta > -180.0 && delta < 180.0);
    }

    #[test]
    fn test_transfer_matrix_single_layer() {
        // Air/SiO2(100nm)/Si
        let layers = vec![
            Layer::new(0.0, 1.0, 0.0),
            Layer::new(100.0, 1.46, 0.0),
            Layer::new(0.0, 3.87, 0.02),
        ];
        let theta = 75.0_f64.to_radians();
        let (rs, rp) = TransferMatrix::multilayer_reflection(&layers, 632.8, theta);
        let (psi, delta) = TransferMatrix::psi_delta_from_rho(rp, rs);
        // Should be different from bare substrate
        assert!(psi > 0.0 && psi < 90.0);
    }

    #[test]
    fn test_psi_delta_from_rho_pure_real() {
        // If rp and rs are both real and positive, Delta = 0
        let rp = (0.5, 0.0);
        let rs = (0.3, 0.0);
        let (psi, delta) = TransferMatrix::psi_delta_from_rho(rp, rs);
        assert!(psi > 0.0);
        assert!(approx_eq(delta, 0.0, TOL_LOOSE));
    }

    #[test]
    fn test_propagation_matrix_zero_thickness() {
        let p = TransferMatrix::propagation_matrix((1.5, 0.0), 0.0, 500.0, 0.5);
        // Should be identity
        assert!(approx_eq(p[0][0].0, 1.0, TOL));
        assert!(approx_eq(p[1][1].0, 1.0, TOL));
    }

    #[test]
    fn test_multilayer_spectral_variation() {
        // Psi/Delta should vary with wavelength for a thin film
        let layers = vec![
            Layer::new(0.0, 1.0, 0.0),
            Layer::new(200.0, 1.46, 0.0),
            Layer::new(0.0, 3.87, 0.02),
        ];
        let theta = 70.0_f64.to_radians();
        let (_, rp1) = TransferMatrix::multilayer_reflection(&layers, 400.0, theta);
        let (_, rp2) = TransferMatrix::multilayer_reflection(&layers, 700.0, theta);
        // Different wavelengths should give different rp
        assert!((c_abs(rp1) - c_abs(rp2)).abs() > 1e-4);
    }

    // ---- DispersionModels tests ----

    #[test]
    fn test_cauchy_glass() {
        // Typical glass: A=1.52, B~0.004, C~0
        let n = DispersionModels::cauchy(500.0, 1.52, 0.004, 0.0);
        assert!(n > 1.5 && n < 1.6);
    }

    #[test]
    fn test_cauchy_dispersion() {
        // n should decrease with wavelength (normal dispersion)
        let n_blue = DispersionModels::cauchy(400.0, 1.52, 0.004, 0.0);
        let n_red = DispersionModels::cauchy(700.0, 1.52, 0.004, 0.0);
        assert!(n_blue > n_red);
    }

    #[test]
    fn test_sellmeier_bk7() {
        // Schott BK7-like coefficients (approximate in um^2)
        let coeffs = vec![
            (1.0396, 0.00600),
            (0.2318, 0.02002),
            (1.0105, 103.56),
        ];
        let n = DispersionModels::sellmeier(589.3, &coeffs); // sodium D line
        assert!(n > 1.4 && n < 1.6);
    }

    #[test]
    fn test_sellmeier_vacuum() {
        // No terms -> n = 1
        let n = DispersionModels::sellmeier(500.0, &[]);
        assert!(approx_eq(n, 1.0, TOL));
    }

    #[test]
    fn test_drude_metal() {
        // At low energy (below plasma freq), eps1 should be negative for metals
        let (eps1, _) = DispersionModels::drude(1.0, 9.0, 0.1);
        assert!(eps1 < 0.0);
    }

    #[test]
    fn test_drude_high_energy() {
        // Well above plasma frequency, eps1 -> 1
        let (eps1, _) = DispersionModels::drude(20.0, 9.0, 0.1);
        assert!(eps1 > 0.5);
    }

    #[test]
    fn test_lorentz_at_resonance() {
        // At resonance E = E0, imaginary part should peak
        let (_, eps2_on) = DispersionModels::lorentz_oscillator(3.0, 10.0, 3.0, 0.3);
        let (_, eps2_off) = DispersionModels::lorentz_oscillator(1.0, 10.0, 3.0, 0.3);
        assert!(eps2_on.abs() > eps2_off.abs());
    }

    #[test]
    fn test_tauc_lorentz_below_bandgap() {
        // Below bandgap, eps2 = 0
        let (_, eps2) = DispersionModels::tauc_lorentz(2.0, 100.0, 4.0, 1.0, 3.0);
        assert!(approx_eq(eps2, 0.0, TOL));
    }

    #[test]
    fn test_tauc_lorentz_above_bandgap() {
        // Above bandgap, eps2 > 0
        let (_, eps2) = DispersionModels::tauc_lorentz(4.0, 100.0, 4.0, 1.0, 3.0);
        assert!(eps2 > 0.0);
    }

    #[test]
    fn test_n_k_from_epsilon_real_positive() {
        // eps = (4, 0) -> n = 2, k = 0
        let (n, k) = DispersionModels::n_k_from_epsilon(4.0, 0.0);
        assert!(approx_eq(n, 2.0, TOL));
        assert!(approx_eq(k, 0.0, TOL));
    }

    #[test]
    fn test_n_k_roundtrip() {
        let (eps1, eps2) = (3.0, 1.5);
        let (n, k) = DispersionModels::n_k_from_epsilon(eps1, eps2);
        // n^2 - k^2 = eps1, 2*n*k = eps2
        assert!(approx_eq(n * n - k * k, eps1, TOL_LOOSE));
        assert!(approx_eq(2.0 * n * k, eps2, TOL_LOOSE));
    }

    #[test]
    fn test_wavelength_energy_conversion() {
        let wl = 500.0;
        let ev = DispersionModels::wavelength_to_ev(wl);
        let wl_back = DispersionModels::ev_to_wavelength(ev);
        assert!(approx_eq(wl, wl_back, TOL));
    }

    #[test]
    fn test_wavelength_to_ev_known() {
        // 1240 nm ~ 1 eV
        let ev = DispersionModels::wavelength_to_ev(1239.84198);
        assert!(approx_eq(ev, 1.0, TOL_LOOSE));
    }

    // ---- ThicknessFitter tests ----

    #[test]
    fn test_thickness_fit_round_trip() {
        // Simulate data for a known 100 nm SiO2 film on Si, then fit
        let layers = vec![Layer::new(100.0, 1.46, 0.0)];
        let substrate = (3.87, 0.02);
        let wavelengths: Vec<f64> = (300..=800).step_by(10).map(|w| w as f64).collect();
        let data = EllipsometrySimulator::simulate(&layers, substrate, &wavelengths, 70.0);

        let result = ThicknessFitter::fit_thickness(
            &data,
            3.87,
            DispersionType::Constant { n: 1.46, k: 0.0 },
            70.0,
        );

        // Should recover ~100 nm
        assert!((result.thickness_nm - 100.0).abs() < 2.0,
            "Expected ~100 nm, got {} nm", result.thickness_nm);
        assert!(result.mse < 1.0);
    }

    #[test]
    fn test_thickness_fit_thin_film() {
        // Fit a known thin (10 nm) SiO2 film and verify recovery
        let layers = vec![Layer::new(10.0, 1.46, 0.0)];
        let substrate = (3.87, 0.02);
        let wavelengths: Vec<f64> = (300..=800).step_by(5).map(|w| w as f64).collect();
        let data = EllipsometrySimulator::simulate(&layers, substrate, &wavelengths, 70.0);

        let result = ThicknessFitter::fit_thickness(
            &data,
            3.87,
            DispersionType::Constant { n: 1.46, k: 0.0 },
            70.0,
        );

        assert!((result.thickness_nm - 10.0).abs() < 3.0,
            "Expected ~10 nm, got {} nm", result.thickness_nm);
        assert!(result.mse < 1.0,
            "MSE should be low for ideal data, got {}", result.mse);
    }

    #[test]
    fn test_thickness_fit_cauchy() {
        // Fit with Cauchy model
        let layers = vec![Layer::new(150.0, 1.46, 0.0)];
        let substrate = (3.87, 0.02);
        let wavelengths: Vec<f64> = (400..=700).step_by(20).map(|w| w as f64).collect();
        let data = EllipsometrySimulator::simulate(&layers, substrate, &wavelengths, 70.0);

        let result = ThicknessFitter::fit_thickness(
            &data,
            3.87,
            DispersionType::Cauchy { a: 1.458, b: 0.00354, c: 0.0 },
            70.0,
        );

        assert!((result.thickness_nm - 150.0).abs() < 5.0,
            "Expected ~150 nm, got {} nm", result.thickness_nm);
    }

    // ---- MaterialDatabase tests ----

    #[test]
    fn test_silicon_dioxide_visible() {
        let (n, k) = MaterialDatabase::silicon_dioxide(550.0);
        assert!(n > 1.4 && n < 1.5, "SiO2 n={}", n);
        assert!(approx_eq(k, 0.0, TOL));
    }

    #[test]
    fn test_silicon_nitride_visible() {
        let (n, k) = MaterialDatabase::silicon_nitride(550.0);
        assert!(n > 1.9 && n < 2.2, "Si3N4 n={}", n);
        assert!(approx_eq(k, 0.0, TOL));
    }

    #[test]
    fn test_titanium_dioxide_visible() {
        let (n, k) = MaterialDatabase::titanium_dioxide(550.0);
        assert!(n > 2.3 && n < 2.7, "TiO2 n={}", n);
        assert!(approx_eq(k, 0.0, TOL));
    }

    #[test]
    fn test_silicon_absorbing() {
        let (n, k) = MaterialDatabase::silicon(550.0);
        assert!(n > 0.0);
        assert!(k > 0.0, "Si should be absorbing at 550 nm, k={}", k);
    }

    #[test]
    fn test_gold_metallic() {
        let (n, k) = MaterialDatabase::gold(632.8);
        // Gold should have significant k in visible
        assert!(k > 0.0, "Au should be absorbing, k={}", k);
    }

    #[test]
    fn test_aluminum_metallic() {
        let (n, k) = MaterialDatabase::aluminum(500.0);
        assert!(k > 0.0, "Al should be absorbing, k={}", k);
    }

    // ---- EffectiveMediumApproximation tests ----

    #[test]
    fn test_linear_mixing_50_50() {
        let n_a = (1.5, 0.0);
        let n_b = (2.5, 0.0);
        let n_eff = EffectiveMediumApproximation::linear_mixing(n_a, n_b, 0.5);
        assert!(approx_eq(n_eff.0, 2.0, TOL));
        assert!(approx_eq(n_eff.1, 0.0, TOL));
    }

    #[test]
    fn test_linear_mixing_pure_a() {
        let n_a = (1.5, 0.1);
        let n_b = (2.5, 0.3);
        let n_eff = EffectiveMediumApproximation::linear_mixing(n_a, n_b, 1.0);
        assert!(approx_eq(n_eff.0, 1.5, TOL));
        assert!(approx_eq(n_eff.1, 0.1, TOL));
    }

    #[test]
    fn test_maxwell_garnett_low_fraction() {
        // Very low volume fraction should be close to host
        let n_host = (1.5, 0.0);
        let n_incl = (3.0, 0.0);
        let n_eff = EffectiveMediumApproximation::maxwell_garnett(n_host, n_incl, 0.01);
        assert!((n_eff.0 - 1.5).abs() < 0.05);
    }

    #[test]
    fn test_bruggeman_symmetric() {
        // 50/50 Bruggeman should give intermediate value
        let n_a = (1.5, 0.0);
        let n_b = (2.5, 0.0);
        let n_eff = EffectiveMediumApproximation::bruggeman(n_a, n_b, 0.5);
        assert!(n_eff.0 > 1.5 && n_eff.0 < 2.5,
            "Bruggeman 50/50 n_eff={} should be between 1.5 and 2.5", n_eff.0);
    }

    #[test]
    fn test_bruggeman_pure_material() {
        // fraction_a = 1.0 should give n_a
        let n_a = (1.5, 0.0);
        let n_b = (2.5, 0.0);
        let n_eff = EffectiveMediumApproximation::bruggeman(n_a, n_b, 1.0);
        assert!((n_eff.0 - 1.5).abs() < TOL_VERY_LOOSE,
            "Pure material A should give n_a, got {}", n_eff.0);
    }

    #[test]
    fn test_maxwell_garnett_surface_roughness() {
        // Model roughness as 50% material / 50% void
        let n_material = (1.46, 0.0);
        let n_void = (1.0, 0.0);
        let n_rough = EffectiveMediumApproximation::maxwell_garnett(
            n_void, n_material, 0.5,
        );
        assert!(n_rough.0 > 1.0 && n_rough.0 < 1.46);
    }

    // ---- MultiAngleAnalysis tests ----

    #[test]
    fn test_combine_angles() {
        let d1 = EllipsometryData::new(vec![500.0], vec![30.0], vec![100.0]);
        let d2 = EllipsometryData::new(vec![500.0], vec![35.0], vec![110.0]);
        let multi = MultiAngleAnalysis::combine_angles(&[d1, d2], &[65.0, 75.0]);
        assert_eq!(multi.data_sets.len(), 2);
        assert_eq!(multi.angles_deg.len(), 2);
    }

    #[test]
    fn test_pseudo_dielectric_bare_si() {
        // For bare Si at 70 degrees, pseudo-dielectric should approximate Si's epsilon
        let layers = vec![
            Layer::new(0.0, 1.0, 0.0),
            Layer::new(0.0, 3.87, 0.02),
        ];
        let theta = 70.0_f64.to_radians();
        let (rs, rp) = TransferMatrix::multilayer_reflection(&layers, 632.8, theta);
        let (psi, delta) = TransferMatrix::psi_delta_from_rho(rp, rs);
        let (eps1, eps2) = MultiAngleAnalysis::pseudo_dielectric(psi, delta, 70.0);
        // Should give eps1 ~ 15-16, eps2 ~ 0 (rough estimate for Si at 632.8 nm)
        assert!(eps1 > 5.0, "eps1={} should be positive and large for Si", eps1);
    }

    #[test]
    fn test_fit_multi_angle_consistency() {
        // Simulate at two angles, fit should find consistent thickness
        let layers = vec![Layer::new(100.0, 1.46, 0.0)];
        let substrate = (3.87, 0.02);
        let wls: Vec<f64> = (400..=700).step_by(50).map(|w| w as f64).collect();

        let d65 = EllipsometrySimulator::simulate(&layers, substrate, &wls, 65.0);
        let d75 = EllipsometrySimulator::simulate(&layers, substrate, &wls, 75.0);

        let multi = MultiAngleAnalysis::combine_angles(&[d65, d75], &[65.0, 75.0]);
        let model = LayerModel {
            layers: vec![
                Layer::new(0.0, 1.0, 0.0),
                Layer::new(50.0, 1.46, 0.0), // initial guess
                Layer::new(0.0, 3.87, 0.02),
            ],
            fit_indices: vec![1],
        };

        let result = MultiAngleAnalysis::fit_multi_angle(&multi, &model);
        assert!((result.thicknesses[0] - 100.0).abs() < 5.0,
            "Multi-angle fit: expected ~100 nm, got {} nm", result.thicknesses[0]);
    }

    // ---- FilmQuality tests ----

    #[test]
    fn test_depolarization_smooth_data() {
        // Smooth data should have near-zero depolarization
        let psi: Vec<f64> = (0..20).map(|i| 30.0 + 0.5 * i as f64).collect();
        let delta: Vec<f64> = (0..20).map(|i| 100.0 + 1.0 * i as f64).collect();
        let wl: Vec<f64> = (0..20).map(|i| 400.0 + 20.0 * i as f64).collect();
        let data = EllipsometryData::new(wl, psi, delta);

        let dep = FilmQuality::depolarization(&data);
        for &d in &dep {
            assert!(d < 0.05, "Smooth data should have low depolarization, got {}", d);
        }
    }

    #[test]
    fn test_uniformity_check_uniform() {
        let map = vec![
            vec![30.0, 30.1, 29.9],
            vec![30.0, 30.0, 30.0],
        ];
        let cv = FilmQuality::uniformity_check(&map);
        assert!(cv < 0.01, "Uniform map should have small CV, got {}", cv);
    }

    #[test]
    fn test_uniformity_check_nonuniform() {
        let map = vec![
            vec![20.0, 40.0, 60.0],
            vec![10.0, 50.0, 70.0],
        ];
        let cv = FilmQuality::uniformity_check(&map);
        assert!(cv > 0.1, "Non-uniform map should have large CV, got {}", cv);
    }

    #[test]
    fn test_sensitivity_analysis_returns_values() {
        let model = LayerModel {
            layers: vec![
                Layer::new(0.0, 1.0, 0.0),
                Layer::new(100.0, 1.46, 0.0),
                Layer::new(0.0, 3.87, 0.02),
            ],
            fit_indices: vec![1],
        };
        let sens = FilmQuality::sensitivity_analysis(&model, 0);
        assert!(!sens.is_empty());
        // Sensitivity should be positive
        for &s in &sens {
            assert!(s >= 0.0);
        }
    }

    // ---- EllipsometrySimulator tests ----

    #[test]
    fn test_simulator_bare_substrate() {
        let substrate = (3.87, 0.02);
        let wls = vec![500.0, 600.0, 700.0];
        let data = EllipsometrySimulator::simulate(&[], substrate, &wls, 70.0);
        assert_eq!(data.num_points(), 3);
        for &psi in &data.psi_deg {
            assert!(psi > 0.0 && psi < 90.0);
        }
    }

    #[test]
    fn test_simulator_thin_film() {
        let layers = vec![Layer::new(100.0, 1.46, 0.0)];
        let substrate = (3.87, 0.02);
        let wls: Vec<f64> = (400..=700).step_by(50).map(|w| w as f64).collect();
        let data = EllipsometrySimulator::simulate(&layers, substrate, &wls, 70.0);

        assert_eq!(data.num_points(), wls.len());
        // Check valid ranges
        for i in 0..data.num_points() {
            assert!(data.psi_deg[i] > 0.0 && data.psi_deg[i] < 90.0);
            assert!(data.delta_deg[i] > -180.0 && data.delta_deg[i] < 180.0);
        }
    }

    #[test]
    fn test_simulator_multilayer() {
        let layers = vec![
            Layer::new(50.0, 2.0, 0.0),   // TiO2-like
            Layer::new(100.0, 1.46, 0.0),  // SiO2
        ];
        let substrate = (3.87, 0.02);
        let wls: Vec<f64> = (400..=700).step_by(50).map(|w| w as f64).collect();
        let data = EllipsometrySimulator::simulate(&layers, substrate, &wls, 70.0);
        assert_eq!(data.num_points(), wls.len());
    }

    #[test]
    fn test_add_noise_preserves_length() {
        let layers = vec![Layer::new(100.0, 1.46, 0.0)];
        let substrate = (3.87, 0.02);
        let wls: Vec<f64> = (400..=700).step_by(50).map(|w| w as f64).collect();
        let data = EllipsometrySimulator::simulate(&layers, substrate, &wls, 70.0);
        let noisy = EllipsometrySimulator::add_noise(&data, 0.1, 0.2);
        assert_eq!(noisy.num_points(), data.num_points());
    }

    #[test]
    fn test_add_noise_changes_values() {
        let layers = vec![Layer::new(100.0, 1.46, 0.0)];
        let substrate = (3.87, 0.02);
        let wls: Vec<f64> = (400..=700).step_by(50).map(|w| w as f64).collect();
        let data = EllipsometrySimulator::simulate(&layers, substrate, &wls, 70.0);
        let noisy = EllipsometrySimulator::add_noise(&data, 1.0, 1.0);

        let mut any_different = false;
        for i in 0..data.num_points() {
            if (data.psi_deg[i] - noisy.psi_deg[i]).abs() > 1e-10 {
                any_different = true;
                break;
            }
        }
        assert!(any_different, "Noise should change at least some values");
    }

    #[test]
    fn test_add_noise_deterministic() {
        // Same call twice should produce same result (deterministic PRNG)
        let data = EllipsometryData::new(
            vec![400.0, 500.0, 600.0],
            vec![30.0, 35.0, 40.0],
            vec![100.0, 110.0, 120.0],
        );
        let n1 = EllipsometrySimulator::add_noise(&data, 0.5, 0.5);
        let n2 = EllipsometrySimulator::add_noise(&data, 0.5, 0.5);
        for i in 0..3 {
            assert!(approx_eq(n1.psi_deg[i], n2.psi_deg[i], TOL));
        }
    }

    // ---- Layer tests ----

    #[test]
    fn test_layer_new() {
        let layer = Layer::new(100.0, 1.46, 0.0);
        assert!(approx_eq(layer.thickness_nm, 100.0, TOL));
        assert!(approx_eq(layer.n, 1.46, TOL));
        assert!(approx_eq(layer.k, 0.0, TOL));
    }

    #[test]
    fn test_layer_complex_n() {
        let layer = Layer::new(50.0, 3.5, 0.5);
        let cn = layer.complex_n();
        assert!(approx_eq(cn.0, 3.5, TOL));
        assert!(approx_eq(cn.1, 0.5, TOL));
    }

    // ---- Integration / round-trip tests ----

    #[test]
    fn test_simulate_and_fit_roundtrip_200nm() {
        let layers = vec![Layer::new(200.0, 1.46, 0.0)];
        let substrate = (3.87, 0.02);
        let wls: Vec<f64> = (300..=800).step_by(5).map(|w| w as f64).collect();
        let data = EllipsometrySimulator::simulate(&layers, substrate, &wls, 70.0);

        let result = ThicknessFitter::fit_thickness(
            &data,
            3.87,
            DispersionType::Constant { n: 1.46, k: 0.0 },
            70.0,
        );

        assert!((result.thickness_nm - 200.0).abs() < 2.0,
            "Expected ~200 nm, got {} nm", result.thickness_nm);
    }

    #[test]
    fn test_fresnel_snell_law_consistency() {
        // Verify Snell's law: n1*sin(theta1) = n2*sin(theta2)
        let n1 = (1.0, 0.0);
        let n2 = (1.5, 0.0);
        let theta1 = 45.0_f64.to_radians();

        let cos_t2 = FresnelCoefficients::snell_cos_theta2(n1, n2, theta1);
        let sin_t2_sq = c_sub((1.0, 0.0), c_mul(cos_t2, cos_t2));

        // n1*sin(theta1)
        let lhs = theta1.sin();
        // n2*sin(theta2) = n2 * sqrt(sin_t2_sq)
        let sin_t2 = c_sqrt(sin_t2_sq);
        let rhs = c_mul(n2, sin_t2);

        assert!(approx_eq(lhs, rhs.0, TOL_LOOSE));
    }

    #[test]
    fn test_thickness_sensitivity_decreases_thick_film() {
        // Thicker films have more oscillations, fitting should still work
        let layers = vec![Layer::new(500.0, 1.46, 0.0)];
        let substrate = (3.87, 0.02);
        let wls: Vec<f64> = (300..=800).step_by(2).map(|w| w as f64).collect();
        let data = EllipsometrySimulator::simulate(&layers, substrate, &wls, 70.0);

        let result = ThicknessFitter::fit_thickness(
            &data,
            3.87,
            DispersionType::Constant { n: 1.46, k: 0.0 },
            70.0,
        );

        assert!((result.thickness_nm - 500.0).abs() < 5.0,
            "Expected ~500 nm, got {} nm", result.thickness_nm);
    }

    #[test]
    fn test_ema_nk_to_eps_roundtrip() {
        let n = 1.5;
        let k = 0.3;
        let eps = EffectiveMediumApproximation::nk_to_eps((n, k));
        let (n2, k2) = EffectiveMediumApproximation::eps_to_nk(eps);
        assert!(approx_eq(n, n2, TOL_LOOSE));
        assert!(approx_eq(k, k2, TOL_LOOSE));
    }

    #[test]
    fn test_interface_matrix_air_glass() {
        let n1 = (1.0, 0.0);
        let n2 = (1.5, 0.0);
        let theta = 0.0;
        let m = TransferMatrix::interface_matrix(n1, n2, theta, Polarization::S);
        // M[0][0] should be 1/t, M[1][0]/M[0][0] = r
        let r = c_div(m[1][0], m[0][0]);
        let rs_direct = FresnelCoefficients::rs(n1, n2, theta);
        assert!(approx_eq(r.0, rs_direct.0, TOL_LOOSE));
    }

    #[test]
    fn test_dispersion_type_constant() {
        let model = DispersionType::Constant { n: 1.5, k: 0.0 };
        let (n, k) = ThicknessFitter::film_nk(&model, 500.0);
        assert!(approx_eq(n, 1.5, TOL));
        assert!(approx_eq(k, 0.0, TOL));
    }

    #[test]
    fn test_polarization_enum_variants() {
        let s = Polarization::S;
        let p = Polarization::P;
        assert_ne!(s, p);
        assert_eq!(s, Polarization::S);
    }

    #[test]
    fn test_fit_result_n_k_length() {
        let layers = vec![Layer::new(100.0, 1.46, 0.0)];
        let substrate = (3.87, 0.02);
        let wls: Vec<f64> = (400..=700).step_by(50).map(|w| w as f64).collect();
        let data = EllipsometrySimulator::simulate(&layers, substrate, &wls, 70.0);

        let result = ThicknessFitter::fit_thickness(
            &data,
            3.87,
            DispersionType::Constant { n: 1.46, k: 0.0 },
            70.0,
        );

        assert_eq!(result.n_values.len(), wls.len());
        assert_eq!(result.k_values.len(), wls.len());
    }
}
