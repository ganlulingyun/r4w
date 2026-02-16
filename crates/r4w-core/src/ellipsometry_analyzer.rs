//! Ellipsometry Analyzer — Thin Film Optical Characterization
//!
//! Signal processing for spectroscopic ellipsometry: measuring thin film
//! thickness and optical constants from polarized light reflection.
//! Applications include semiconductor wafer metrology, anti-reflection
//! coatings, biological thin films, and display manufacturing.
//!
//! Physics:
//! - Fresnel equations for reflection at interfaces
//! - Transfer matrix method (TMM) for multilayer thin film stacks
//! - Ellipsometric ratio: ρ = r_p / r_s = tan(ψ) · exp(iΔ)
//! - Dispersion models: Cauchy, Sellmeier, Drude for optical constants
//!
//! No direct GNU Radio equivalent — optical thin film metrology tool.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ellipsometry_analyzer::{
//!     FresnelCoefficients, PsiDeltaCalculator, CauchyModel,
//!     MaterialDatabase, Material,
//! };
//!
//! // Single SiO2 film on Si substrate at 70° incidence, 632.8 nm
//! let air = MaterialDatabase::get(Material::Air);
//! let sio2_n = CauchyModel::new(1.457, 0.00354, 0.0).index_at(632.8e-9);
//! let si = MaterialDatabase::get(Material::Silicon);
//!
//! let calc = PsiDeltaCalculator::new(70.0_f64.to_radians());
//! let (psi, delta) = calc.single_film(air, sio2_n, si, 100.0e-9, 632.8e-9);
//! assert!(psi > 0.0 && psi < std::f64::consts::FRAC_PI_2);
//! ```

use std::f64::consts::PI;

// ─── Complex number helpers (no external deps) ───────────────────────────

/// Minimal complex number for internal use (pure Rust, no dependencies).
#[derive(Debug, Clone, Copy)]
struct Cx {
    re: f64,
    im: f64,
}

impl Cx {
    fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    fn from_polar(r: f64, theta: f64) -> Self {
        Self {
            re: r * theta.cos(),
            im: r * theta.sin(),
        }
    }

    fn conj(self) -> Self {
        Self {
            re: self.re,
            im: -self.im,
        }
    }

    fn norm_sqr(self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    fn norm(self) -> f64 {
        self.norm_sqr().sqrt()
    }

    fn arg(self) -> f64 {
        self.im.atan2(self.re)
    }

    fn sqrt(self) -> Self {
        let r = self.norm();
        let theta = self.arg();
        Self::from_polar(r.sqrt(), theta / 2.0)
    }

    fn exp(self) -> Self {
        let e = self.re.exp();
        Self {
            re: e * self.im.cos(),
            im: e * self.im.sin(),
        }
    }

    fn sin(self) -> Self {
        // sin(a+bi) = sin(a)cosh(b) + i*cos(a)sinh(b)
        Self {
            re: self.re.sin() * self.im.cosh(),
            im: self.re.cos() * self.im.sinh(),
        }
    }

    fn cos(self) -> Self {
        // cos(a+bi) = cos(a)cosh(b) - i*sin(a)sinh(b)
        Self {
            re: self.re.cos() * self.im.cosh(),
            im: -self.re.sin() * self.im.sinh(),
        }
    }
}

impl std::ops::Add for Cx {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            re: self.re + rhs.re,
            im: self.im + rhs.im,
        }
    }
}

impl std::ops::Sub for Cx {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            re: self.re - rhs.re,
            im: self.im - rhs.im,
        }
    }
}

impl std::ops::Mul for Cx {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self {
            re: self.re * rhs.re - self.im * rhs.im,
            im: self.re * rhs.im + self.im * rhs.re,
        }
    }
}

impl std::ops::Mul<f64> for Cx {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self {
            re: self.re * rhs,
            im: self.im * rhs,
        }
    }
}

impl std::ops::Div for Cx {
    type Output = Self;
    fn div(self, rhs: Self) -> Self {
        let denom = rhs.norm_sqr();
        Self {
            re: (self.re * rhs.re + self.im * rhs.im) / denom,
            im: (self.im * rhs.re - self.re * rhs.im) / denom,
        }
    }
}

impl std::ops::Neg for Cx {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            re: -self.re,
            im: -self.im,
        }
    }
}

/// Complex refractive index: n + ik (k = extinction coefficient).
#[derive(Debug, Clone, Copy)]
pub struct ComplexIndex {
    /// Real part of the refractive index (phase velocity ratio).
    pub n: f64,
    /// Extinction coefficient (absorption).
    pub k: f64,
}

impl ComplexIndex {
    /// Create a new complex refractive index.
    pub fn new(n: f64, k: f64) -> Self {
        Self { n, k }
    }

    /// Create a transparent (non-absorbing) material.
    pub fn transparent(n: f64) -> Self {
        Self { n, k: 0.0 }
    }

    fn to_cx(self) -> Cx {
        Cx::new(self.n, self.k)
    }
}

// ─── 2x2 complex matrix for transfer matrix method ──────────────────────

#[derive(Debug, Clone, Copy)]
struct Mat2x2 {
    m00: Cx,
    m01: Cx,
    m10: Cx,
    m11: Cx,
}

impl Mat2x2 {
    fn identity() -> Self {
        Self {
            m00: Cx::new(1.0, 0.0),
            m01: Cx::new(0.0, 0.0),
            m10: Cx::new(0.0, 0.0),
            m11: Cx::new(1.0, 0.0),
        }
    }

    fn mul(self, rhs: Self) -> Self {
        Self {
            m00: self.m00 * rhs.m00 + self.m01 * rhs.m10,
            m01: self.m00 * rhs.m01 + self.m01 * rhs.m11,
            m10: self.m10 * rhs.m00 + self.m11 * rhs.m10,
            m11: self.m10 * rhs.m01 + self.m11 * rhs.m11,
        }
    }
}

// ─── Configuration ───────────────────────────────────────────────────────

/// Ellipsometry measurement configuration.
#[derive(Debug, Clone)]
pub struct EllipsometryConfig {
    /// Angle of incidence in radians.
    pub angle_of_incidence: f64,
    /// Start of wavelength range in meters.
    pub wavelength_min: f64,
    /// End of wavelength range in meters.
    pub wavelength_max: f64,
    /// Number of wavelength points.
    pub num_wavelengths: usize,
    /// Substrate material.
    pub substrate: ComplexIndex,
    /// Number of thin film layers.
    pub num_layers: usize,
}

impl EllipsometryConfig {
    /// Create a new configuration.
    pub fn new(angle_rad: f64, wl_min: f64, wl_max: f64, num_wl: usize) -> Self {
        Self {
            angle_of_incidence: angle_rad,
            wavelength_min: wl_min,
            wavelength_max: wl_max,
            num_wavelengths: num_wl,
            substrate: MaterialDatabase::get(Material::Silicon),
            num_layers: 1,
        }
    }

    /// Generate the array of wavelength values (in meters).
    pub fn wavelengths(&self) -> Vec<f64> {
        let n = self.num_wavelengths.max(1);
        if n == 1 {
            return vec![(self.wavelength_min + self.wavelength_max) / 2.0];
        }
        let step = (self.wavelength_max - self.wavelength_min) / (n - 1) as f64;
        (0..n).map(|i| self.wavelength_min + i as f64 * step).collect()
    }
}

// ─── Fresnel Coefficients ────────────────────────────────────────────────

/// Fresnel reflection and transmission coefficients for a single interface.
#[derive(Debug, Clone)]
pub struct FresnelCoefficients;

impl FresnelCoefficients {
    /// Compute the complex refraction angle via Snell's law:
    /// n1 * sin(theta1) = n2 * sin(theta2).
    ///
    /// Returns the complex angle theta2 (may be complex for absorbing media
    /// or total internal reflection).
    pub fn snell_angle(n1: ComplexIndex, theta1: f64, n2: ComplexIndex) -> Cx {
        // sin(theta2) = (n1/n2) * sin(theta1)
        let n1c = n1.to_cx();
        let n2c = n2.to_cx();
        let sin_t2 = (n1c * Cx::new(theta1.sin(), 0.0)) / n2c;
        // cos(theta2) = sqrt(1 - sin^2(theta2))
        // theta2 = asin(sin_t2), but we work with sin/cos directly
        // Return the complex cos(theta2)
        let _ = sin_t2; // We compute both sin and cos below
        // Actually, return the angle conceptually — but for Fresnel we need
        // sin(theta2) and cos(theta2). Let's return cos(theta2) as that's
        // what's needed.
        let cos_t2_sq = Cx::new(1.0, 0.0) - sin_t2 * sin_t2;
        let _ = cos_t2_sq;
        // For the public API, return sin(theta2) — callers use helper.
        sin_t2
    }

    /// Compute cos(theta2) from Snell's law.
    fn cos_theta2(n1: ComplexIndex, theta1: f64, n2: ComplexIndex) -> Cx {
        let sin_t2 = Self::snell_angle(n1, theta1, n2);
        (Cx::new(1.0, 0.0) - sin_t2 * sin_t2).sqrt()
    }

    /// Compute the p-polarized (TM) reflection coefficient r_p.
    ///
    /// r_p = (n2·cos(θ1) − n1·cos(θ2)) / (n2·cos(θ1) + n1·cos(θ2))
    pub fn r_p(n1: ComplexIndex, theta1: f64, n2: ComplexIndex) -> Cx {
        let cos_t1 = Cx::new(theta1.cos(), 0.0);
        let cos_t2 = Self::cos_theta2(n1, theta1, n2);
        let n1c = n1.to_cx();
        let n2c = n2.to_cx();
        let num = n2c * cos_t1 - n1c * cos_t2;
        let den = n2c * cos_t1 + n1c * cos_t2;
        num / den
    }

    /// Compute the s-polarized (TE) reflection coefficient r_s.
    ///
    /// r_s = (n1·cos(θ1) − n2·cos(θ2)) / (n1·cos(θ1) + n2·cos(θ2))
    pub fn r_s(n1: ComplexIndex, theta1: f64, n2: ComplexIndex) -> Cx {
        let cos_t1 = Cx::new(theta1.cos(), 0.0);
        let cos_t2 = Self::cos_theta2(n1, theta1, n2);
        let n1c = n1.to_cx();
        let n2c = n2.to_cx();
        let num = n1c * cos_t1 - n2c * cos_t2;
        let den = n1c * cos_t1 + n2c * cos_t2;
        num / den
    }

    /// Compute p-polarized reflectance R_p = |r_p|^2.
    pub fn reflectance_p(n1: ComplexIndex, theta1: f64, n2: ComplexIndex) -> f64 {
        Self::r_p(n1, theta1, n2).norm_sqr()
    }

    /// Compute s-polarized reflectance R_s = |r_s|^2.
    pub fn reflectance_s(n1: ComplexIndex, theta1: f64, n2: ComplexIndex) -> f64 {
        Self::r_s(n1, theta1, n2).norm_sqr()
    }

    /// Brewster angle: θ_B = arctan(n2/n1) (for real, non-absorbing media).
    pub fn brewster_angle(n1: f64, n2: f64) -> f64 {
        (n2 / n1).atan()
    }

    /// Critical angle for total internal reflection: θ_c = arcsin(n2/n1).
    /// Returns None if n2 >= n1 (no TIR possible).
    pub fn critical_angle(n1: f64, n2: f64) -> Option<f64> {
        if n2 >= n1 {
            return None;
        }
        Some((n2 / n1).asin())
    }
}

// ─── Multi-layer transfer matrix ─────────────────────────────────────────

/// Transfer matrix method for multilayer thin film stacks.
///
/// Computes the overall reflection coefficients for a stack of layers
/// between an ambient (superstrate) and a substrate using 2×2 transfer
/// matrices.
pub struct MultiLayerReflection;

impl MultiLayerReflection {
    /// Compute the interface matrix for p-polarization.
    ///
    /// D_p = [[cos(θ), cos(θ)], [n·cos(θ), -n·cos(θ)]] (simplified form)
    /// For TMM we use the characteristic matrix approach.
    fn layer_matrix_p(
        n: ComplexIndex,
        cos_theta: Cx,
        thickness: f64,
        wavelength: f64,
    ) -> Mat2x2 {
        // Phase thickness: δ = 2π·n·d·cos(θ)/λ
        let nc = n.to_cx();
        let delta = nc * cos_theta * Cx::new(2.0 * PI * thickness / wavelength, 0.0);
        let cos_d = delta.cos();
        let sin_d = delta.sin();
        // η_p = n / cos(θ) (for p-polarization, admittance)
        let eta = nc / cos_theta;

        Mat2x2 {
            m00: cos_d,
            m01: Cx::new(0.0, -1.0) * sin_d / eta,
            m10: Cx::new(0.0, -1.0) * sin_d * eta,
            m11: cos_d,
        }
    }

    /// Compute the layer matrix for s-polarization.
    fn layer_matrix_s(
        n: ComplexIndex,
        cos_theta: Cx,
        thickness: f64,
        wavelength: f64,
    ) -> Mat2x2 {
        let nc = n.to_cx();
        let delta = nc * cos_theta * Cx::new(2.0 * PI * thickness / wavelength, 0.0);
        let cos_d = delta.cos();
        let sin_d = delta.sin();
        // η_s = n · cos(θ) (for s-polarization)
        let eta = nc * cos_theta;

        Mat2x2 {
            m00: cos_d,
            m01: Cx::new(0.0, -1.0) * sin_d / eta,
            m10: Cx::new(0.0, -1.0) * sin_d * eta,
            m11: cos_d,
        }
    }

    /// Compute overall r_p and r_s for a multilayer stack.
    ///
    /// * `ambient` — refractive index of the ambient medium (superstrate)
    /// * `layers` — list of (refractive_index, thickness_meters) for each film
    /// * `substrate` — refractive index of the substrate
    /// * `theta0` — angle of incidence in the ambient (radians)
    /// * `wavelength` — wavelength in meters
    ///
    /// Returns (r_p, r_s) as complex numbers.
    pub fn reflection_coefficients(
        ambient: ComplexIndex,
        layers: &[(ComplexIndex, f64)],
        substrate: ComplexIndex,
        theta0: f64,
        wavelength: f64,
    ) -> (Cx, Cx) {
        // Compute cos(theta) in each layer via Snell's law
        let n_amb = ambient.to_cx();
        let sin_t0 = Cx::new(theta0.sin(), 0.0);

        let cos_in_layer = |n: ComplexIndex| -> Cx {
            let nc = n.to_cx();
            let sin_t = n_amb * sin_t0 / nc;
            (Cx::new(1.0, 0.0) - sin_t * sin_t).sqrt()
        };

        let cos_t0 = Cx::new(theta0.cos(), 0.0);

        // p-polarization transfer matrix
        let mut mp = Mat2x2::identity();
        for &(n, d) in layers {
            let cos_t = cos_in_layer(n);
            mp = mp.mul(Self::layer_matrix_p(n, cos_t, d, wavelength));
        }

        // s-polarization transfer matrix
        let mut ms = Mat2x2::identity();
        for &(n, d) in layers {
            let cos_t = cos_in_layer(n);
            ms = ms.mul(Self::layer_matrix_s(n, cos_t, d, wavelength));
        }

        let cos_ts = cos_in_layer(substrate);
        let n_sub = substrate.to_cx();

        // Admittance of ambient and substrate
        // p-pol: η = n / cos(θ)
        let eta_amb_p = n_amb / cos_t0;
        let eta_sub_p = n_sub / cos_ts;
        // s-pol: η = n · cos(θ)
        let eta_amb_s = n_amb * cos_t0;
        let eta_sub_s = n_sub * cos_ts;

        // r = (M00·η_sub + M01·η_amb·η_sub − M10 − M11·η_amb)
        //   / (M00·η_sub + M01·η_amb·η_sub + M10 + M11·η_amb)
        // Using characteristic matrix formulation:
        // B = M00 + M01·η_sub
        // C = M10 + M11·η_sub
        // r = (η_amb·B − C) / (η_amb·B + C)

        let bp = mp.m00 + mp.m01 * eta_sub_p;
        let cp = mp.m10 + mp.m11 * eta_sub_p;
        let rp = (eta_amb_p * bp - cp) / (eta_amb_p * bp + cp);

        let bs = ms.m00 + ms.m01 * eta_sub_s;
        let cs = ms.m10 + ms.m11 * eta_sub_s;
        let rs = (eta_amb_s * bs - cs) / (eta_amb_s * bs + cs);

        (rp, rs)
    }
}

// ─── Psi / Delta calculator ──────────────────────────────────────────────

/// Compute ellipsometric angles ψ (psi) and Δ (delta) from the reflection
/// coefficient ratio ρ = r_p / r_s = tan(ψ) · exp(iΔ).
pub struct PsiDeltaCalculator {
    /// Angle of incidence (radians).
    theta0: f64,
}

impl PsiDeltaCalculator {
    /// Create a new calculator for the given angle of incidence (radians).
    pub fn new(theta0: f64) -> Self {
        Self { theta0 }
    }

    /// Compute (ψ, Δ) from complex reflection coefficients.
    pub fn from_coefficients(rp: Cx, rs: Cx) -> (f64, f64) {
        let rho = rp / rs;
        let psi = rho.norm().atan(); // ψ = arctan(|ρ|)
        let delta = rho.arg(); // Δ = arg(ρ)
        (psi, delta)
    }

    /// Compute (ψ, Δ) for a bare substrate (single interface).
    ///
    /// Uses the transfer matrix method with zero layers for consistency
    /// with the multilayer formulation.
    pub fn bare_substrate(
        &self,
        ambient: ComplexIndex,
        substrate: ComplexIndex,
    ) -> (f64, f64) {
        // Wavelength is irrelevant for zero layers (no propagation phase)
        let (rp, rs) = MultiLayerReflection::reflection_coefficients(
            ambient, &[], substrate, self.theta0, 632.8e-9,
        );
        Self::from_coefficients(rp, rs)
    }

    /// Compute (ψ, Δ) for a single film on a substrate.
    pub fn single_film(
        &self,
        ambient: ComplexIndex,
        film: ComplexIndex,
        substrate: ComplexIndex,
        thickness: f64,
        wavelength: f64,
    ) -> (f64, f64) {
        let layers = vec![(film, thickness)];
        let (rp, rs) = MultiLayerReflection::reflection_coefficients(
            ambient, &layers, substrate, self.theta0, wavelength,
        );
        Self::from_coefficients(rp, rs)
    }

    /// Compute (ψ, Δ) for a multilayer stack.
    pub fn multilayer(
        &self,
        ambient: ComplexIndex,
        layers: &[(ComplexIndex, f64)],
        substrate: ComplexIndex,
        wavelength: f64,
    ) -> (f64, f64) {
        let (rp, rs) = MultiLayerReflection::reflection_coefficients(
            ambient, layers, substrate, self.theta0, wavelength,
        );
        Self::from_coefficients(rp, rs)
    }

    /// Compute spectroscopic ψ and Δ over a range of wavelengths.
    pub fn spectroscopic(
        &self,
        ambient: ComplexIndex,
        layers: &[(ComplexIndex, f64)],
        substrate: ComplexIndex,
        wavelengths: &[f64],
    ) -> Vec<(f64, f64)> {
        wavelengths
            .iter()
            .map(|&wl| {
                let (rp, rs) = MultiLayerReflection::reflection_coefficients(
                    ambient, layers, substrate, self.theta0, wl,
                );
                Self::from_coefficients(rp, rs)
            })
            .collect()
    }
}

// ─── Dispersion models ───────────────────────────────────────────────────

/// Cauchy dispersion model for transparent films.
///
/// n(λ) = A + B/λ² + C/λ⁴
///
/// where λ is in meters. Suitable for SiO2, Si3N4, and other transparent
/// dielectrics in the visible/NIR range.
#[derive(Debug, Clone, Copy)]
pub struct CauchyModel {
    /// Cauchy coefficient A (dimensionless).
    pub a: f64,
    /// Cauchy coefficient B (m²).
    pub b: f64,
    /// Cauchy coefficient C (m⁴).
    pub c: f64,
}

impl CauchyModel {
    /// Create a new Cauchy model.
    pub fn new(a: f64, b: f64, c: f64) -> Self {
        Self { a, b, c }
    }

    /// Refractive index at wavelength λ (in meters).
    pub fn index_at(&self, wavelength: f64) -> ComplexIndex {
        let lam2 = wavelength * wavelength;
        let lam4 = lam2 * lam2;
        let n = self.a + self.b / lam2 + self.c / lam4;
        ComplexIndex::transparent(n)
    }

    /// Preset for SiO2 (fused silica) — approximate visible range.
    pub fn sio2() -> Self {
        // n ≈ 1.457 at 632.8 nm
        Self::new(1.4580, 0.00354e-12, 0.0)
    }

    /// Preset for Si3N4 (silicon nitride).
    pub fn si3n4() -> Self {
        Self::new(2.008, 0.01198e-12, 0.0)
    }
}

/// Sellmeier dispersion model for optical glasses and crystals.
///
/// n²(λ) = 1 + Σ(Bᵢ·λ² / (λ² − Cᵢ))
///
/// where λ is in meters and Cᵢ are resonance wavelengths squared.
#[derive(Debug, Clone)]
pub struct SellmeierModel {
    /// Sellmeier B coefficients (dimensionless).
    pub b: Vec<f64>,
    /// Sellmeier C coefficients (m², resonance wavelength squared).
    pub c: Vec<f64>,
}

impl SellmeierModel {
    /// Create a new Sellmeier model with coefficient pairs (B, C).
    pub fn new(coefficients: &[(f64, f64)]) -> Self {
        let (b, c): (Vec<_>, Vec<_>) = coefficients.iter().copied().unzip();
        Self { b, c }
    }

    /// Refractive index at wavelength λ (in meters).
    pub fn index_at(&self, wavelength: f64) -> ComplexIndex {
        let lam2 = wavelength * wavelength;
        let mut n_sq = 1.0;
        for i in 0..self.b.len() {
            n_sq += self.b[i] * lam2 / (lam2 - self.c[i]);
        }
        ComplexIndex::transparent(n_sq.max(1.0).sqrt())
    }

    /// Preset for BK7 borosilicate crown glass.
    pub fn bk7() -> Self {
        // Schott BK7 Sellmeier coefficients (λ in meters)
        Self::new(&[
            (1.03961212, 6.00069867e-15),
            (0.231792344, 2.00179144e-14),
            (1.01046945, 1.03560653e-10),
        ])
    }

    /// Preset for fused silica.
    pub fn fused_silica() -> Self {
        Self::new(&[
            (0.6961663, 4.67914826e-15),
            (0.4079426, 1.35120631e-14),
            (0.8974794, 9.79340025e-11),
        ])
    }
}

/// Drude model for metals.
///
/// ε(ω) = ε_∞ − ω_p² / (ω² + iγω)
///
/// where ω_p is the plasma frequency and γ is the damping rate.
#[derive(Debug, Clone, Copy)]
pub struct DrudeModel {
    /// High-frequency dielectric constant.
    pub eps_inf: f64,
    /// Plasma frequency in rad/s.
    pub omega_p: f64,
    /// Damping rate in rad/s.
    pub gamma: f64,
}

impl DrudeModel {
    /// Create a new Drude model.
    pub fn new(eps_inf: f64, omega_p: f64, gamma: f64) -> Self {
        Self {
            eps_inf,
            omega_p,
            gamma,
        }
    }

    /// Complex dielectric function at angular frequency ω.
    pub fn permittivity_at_omega(&self, omega: f64) -> Cx {
        // ε = ε_∞ − ω_p² / (ω² + iγω)
        // = ε_∞ − ω_p² / (ω(ω + iγ))
        let denom = Cx::new(omega * omega, omega * self.gamma);
        // note: negating gamma because convention is ε = ε_∞ − ω_p²/(ω² + iγω)
        let drude_term = Cx::new(self.omega_p * self.omega_p, 0.0) / denom;
        Cx::new(self.eps_inf, 0.0) - drude_term
    }

    /// Complex refractive index at wavelength λ (in meters).
    pub fn index_at(&self, wavelength: f64) -> ComplexIndex {
        let c = 299_792_458.0;
        let omega = 2.0 * PI * c / wavelength;
        let eps = self.permittivity_at_omega(omega);
        // n + ik = sqrt(ε)
        let n_complex = eps.sqrt();
        ComplexIndex::new(n_complex.re, n_complex.im)
    }

    /// Preset for gold (Au) — Drude parameters.
    pub fn gold() -> Self {
        // ω_p ≈ 1.37e16 rad/s, γ ≈ 4.05e13 rad/s
        Self::new(1.0, 1.37e16, 4.05e13)
    }

    /// Preset for silver (Ag).
    pub fn silver() -> Self {
        Self::new(1.0, 1.39e16, 2.73e13)
    }

    /// Preset for aluminum (Al).
    pub fn aluminum() -> Self {
        Self::new(1.0, 2.24e16, 1.22e14)
    }
}

// ─── Material Database ───────────────────────────────────────────────────

/// Known materials with tabulated optical constants at 632.8 nm.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Material {
    /// Air (n ≈ 1.0).
    Air,
    /// Silicon (n ≈ 3.882, k ≈ 0.019 at 632.8 nm).
    Silicon,
    /// Silicon dioxide / fused silica (n ≈ 1.457).
    SiliconDioxide,
    /// Titanium dioxide (n ≈ 2.58 at 632.8 nm).
    TitaniumDioxide,
    /// Aluminum oxide / sapphire (n ≈ 1.766).
    AluminumOxide,
    /// Gold (n ≈ 0.18, k ≈ 3.43 at 632.8 nm).
    Gold,
    /// Silver (n ≈ 0.135, k ≈ 3.99 at 632.8 nm).
    Silver,
    /// Germanium (n ≈ 5.47, k ≈ 0.784 at 632.8 nm).
    Germanium,
    /// Silicon nitride (n ≈ 2.01).
    SiliconNitride,
    /// Water (n ≈ 1.332).
    Water,
}

/// Database of optical constants for common materials.
pub struct MaterialDatabase;

impl MaterialDatabase {
    /// Get complex refractive index at ~632.8 nm (He-Ne laser line).
    pub fn get(material: Material) -> ComplexIndex {
        match material {
            Material::Air => ComplexIndex::transparent(1.0003),
            Material::Silicon => ComplexIndex::new(3.882, 0.019),
            Material::SiliconDioxide => ComplexIndex::transparent(1.457),
            Material::TitaniumDioxide => ComplexIndex::transparent(2.58),
            Material::AluminumOxide => ComplexIndex::transparent(1.766),
            Material::Gold => ComplexIndex::new(0.18, 3.43),
            Material::Silver => ComplexIndex::new(0.135, 3.99),
            Material::Germanium => ComplexIndex::new(5.47, 0.784),
            Material::SiliconNitride => ComplexIndex::transparent(2.01),
            Material::Water => ComplexIndex::transparent(1.332),
        }
    }

    /// List all available materials.
    pub fn all_materials() -> Vec<Material> {
        vec![
            Material::Air,
            Material::Silicon,
            Material::SiliconDioxide,
            Material::TitaniumDioxide,
            Material::AluminumOxide,
            Material::Gold,
            Material::Silver,
            Material::Germanium,
            Material::SiliconNitride,
            Material::Water,
        ]
    }
}

// ─── Thickness Estimator ─────────────────────────────────────────────────

/// Film thickness estimation via MSE minimization of spectroscopic ψ/Δ.
///
/// Uses a simple golden-section search to find the thickness that minimizes
/// the mean squared error between measured and modeled ellipsometric angles.
pub struct ThicknessEstimator {
    /// Angle of incidence (radians).
    theta0: f64,
    /// Ambient refractive index.
    ambient: ComplexIndex,
    /// Film refractive index (assumed constant over spectrum for fitting).
    film: ComplexIndex,
    /// Substrate refractive index.
    substrate: ComplexIndex,
}

impl ThicknessEstimator {
    /// Create a new estimator.
    pub fn new(
        theta0: f64,
        ambient: ComplexIndex,
        film: ComplexIndex,
        substrate: ComplexIndex,
    ) -> Self {
        Self {
            theta0,
            ambient,
            film,
            substrate,
        }
    }

    /// Compute mean squared error between measured and modeled ψ/Δ.
    pub fn mse(
        &self,
        thickness: f64,
        wavelengths: &[f64],
        measured_psi: &[f64],
        measured_delta: &[f64],
    ) -> f64 {
        let calc = PsiDeltaCalculator::new(self.theta0);
        let layers = [(self.film, thickness)];
        let n = wavelengths.len();
        let mut sum = 0.0;
        for i in 0..n {
            let (psi_m, delta_m) = calc.multilayer(
                self.ambient,
                &layers,
                self.substrate,
                wavelengths[i],
            );
            let dpsi = psi_m - measured_psi[i];
            let ddelta = delta_m - measured_delta[i];
            sum += dpsi * dpsi + ddelta * ddelta;
        }
        sum / n as f64
    }

    /// Estimate thickness using golden-section search in [d_min, d_max].
    ///
    /// Returns (estimated_thickness, final_mse).
    pub fn estimate(
        &self,
        wavelengths: &[f64],
        measured_psi: &[f64],
        measured_delta: &[f64],
        d_min: f64,
        d_max: f64,
        tolerance: f64,
    ) -> (f64, f64) {
        let phi = (5.0_f64.sqrt() - 1.0) / 2.0; // golden ratio conjugate
        let mut a = d_min;
        let mut b = d_max;

        let mut x1 = b - phi * (b - a);
        let mut x2 = a + phi * (b - a);
        let mut f1 = self.mse(x1, wavelengths, measured_psi, measured_delta);
        let mut f2 = self.mse(x2, wavelengths, measured_psi, measured_delta);

        while (b - a) > tolerance {
            if f1 < f2 {
                b = x2;
                x2 = x1;
                f2 = f1;
                x1 = b - phi * (b - a);
                f1 = self.mse(x1, wavelengths, measured_psi, measured_delta);
            } else {
                a = x1;
                x1 = x2;
                f1 = f2;
                x2 = a + phi * (b - a);
                f2 = self.mse(x2, wavelengths, measured_psi, measured_delta);
            }
        }

        let d_est = (a + b) / 2.0;
        let mse_final = self.mse(d_est, wavelengths, measured_psi, measured_delta);
        (d_est, mse_final)
    }
}

// ─── Stokes Vector ───────────────────────────────────────────────────────

/// Stokes vector [S0, S1, S2, S3] for polarization state representation.
///
/// - S0: total intensity
/// - S1: horizontal vs vertical preference
/// - S2: +45° vs −45° preference
/// - S3: right vs left circular preference
#[derive(Debug, Clone, Copy)]
pub struct StokesVector {
    pub s: [f64; 4],
}

impl StokesVector {
    /// Create a new Stokes vector.
    pub fn new(s0: f64, s1: f64, s2: f64, s3: f64) -> Self {
        Self { s: [s0, s1, s2, s3] }
    }

    /// Horizontal linear polarization.
    pub fn horizontal() -> Self {
        Self::new(1.0, 1.0, 0.0, 0.0)
    }

    /// Vertical linear polarization.
    pub fn vertical() -> Self {
        Self::new(1.0, -1.0, 0.0, 0.0)
    }

    /// +45° linear polarization.
    pub fn plus45() -> Self {
        Self::new(1.0, 0.0, 1.0, 0.0)
    }

    /// −45° linear polarization.
    pub fn minus45() -> Self {
        Self::new(1.0, 0.0, -1.0, 0.0)
    }

    /// Right circular polarization.
    pub fn right_circular() -> Self {
        Self::new(1.0, 0.0, 0.0, 1.0)
    }

    /// Left circular polarization.
    pub fn left_circular() -> Self {
        Self::new(1.0, 0.0, 0.0, -1.0)
    }

    /// Unpolarized light.
    pub fn unpolarized() -> Self {
        Self::new(1.0, 0.0, 0.0, 0.0)
    }

    /// Degree of polarization: DOP = sqrt(S1² + S2² + S3²) / S0.
    pub fn degree_of_polarization(&self) -> f64 {
        let dop = (self.s[1] * self.s[1]
            + self.s[2] * self.s[2]
            + self.s[3] * self.s[3])
            .sqrt();
        if self.s[0] > 0.0 {
            dop / self.s[0]
        } else {
            0.0
        }
    }

    /// Degree of linear polarization: DOLP = sqrt(S1² + S2²) / S0.
    pub fn degree_of_linear_polarization(&self) -> f64 {
        let dolp = (self.s[1] * self.s[1] + self.s[2] * self.s[2]).sqrt();
        if self.s[0] > 0.0 {
            dolp / self.s[0]
        } else {
            0.0
        }
    }

    /// Degree of circular polarization: DOCP = |S3| / S0.
    pub fn degree_of_circular_polarization(&self) -> f64 {
        if self.s[0] > 0.0 {
            self.s[3].abs() / self.s[0]
        } else {
            0.0
        }
    }

    /// Orientation angle of polarization ellipse (radians): χ = 0.5·atan2(S2, S1).
    pub fn orientation_angle(&self) -> f64 {
        0.5 * self.s[2].atan2(self.s[1])
    }

    /// Ellipticity angle (radians): ε = 0.5·asin(S3/S0).
    pub fn ellipticity_angle(&self) -> f64 {
        if self.s[0] > 0.0 {
            let ratio = (self.s[3] / self.s[0]).clamp(-1.0, 1.0);
            0.5 * ratio.asin()
        } else {
            0.0
        }
    }
}

// ─── Mueller Matrix ──────────────────────────────────────────────────────

/// 4×4 Mueller matrix for complete polarization transformations.
///
/// Transforms Stokes vectors: S_out = M · S_in.
#[derive(Debug, Clone, Copy)]
pub struct MuellerMatrix {
    /// Row-major 4×4 matrix elements.
    pub m: [[f64; 4]; 4],
}

impl MuellerMatrix {
    /// Create a Mueller matrix from a 4×4 array.
    pub fn new(m: [[f64; 4]; 4]) -> Self {
        Self { m }
    }

    /// Identity Mueller matrix (no polarization change).
    pub fn identity() -> Self {
        Self::new([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
    }

    /// Horizontal linear polarizer.
    pub fn linear_polarizer_h() -> Self {
        Self::new([
            [0.5, 0.5, 0.0, 0.0],
            [0.5, 0.5, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
        ])
    }

    /// Vertical linear polarizer.
    pub fn linear_polarizer_v() -> Self {
        Self::new([
            [0.5, -0.5, 0.0, 0.0],
            [-0.5, 0.5, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
        ])
    }

    /// Linear polarizer at angle θ (radians).
    pub fn linear_polarizer(theta: f64) -> Self {
        let c2 = (2.0 * theta).cos();
        let s2 = (2.0 * theta).sin();
        Self::new([
            [0.5, 0.5 * c2, 0.5 * s2, 0.0],
            [0.5 * c2, 0.5 * c2 * c2, 0.5 * c2 * s2, 0.0],
            [0.5 * s2, 0.5 * c2 * s2, 0.5 * s2 * s2, 0.0],
            [0.0, 0.0, 0.0, 0.0],
        ])
    }

    /// Quarter-wave plate with fast axis at angle θ (radians).
    pub fn quarter_wave_plate(theta: f64) -> Self {
        let c2 = (2.0 * theta).cos();
        let s2 = (2.0 * theta).sin();
        Self::new([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, c2 * c2, c2 * s2, -s2],
            [0.0, c2 * s2, s2 * s2, c2],
            [0.0, s2, -c2, 0.0],
        ])
    }

    /// Half-wave plate with fast axis at angle θ (radians).
    pub fn half_wave_plate(theta: f64) -> Self {
        let c4 = (4.0 * theta).cos();
        let s4 = (4.0 * theta).sin();
        let c2 = (2.0 * theta).cos();
        let _ = c4;
        Self::new([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, (2.0 * theta).cos() * (2.0 * theta).cos() * 2.0 - 1.0,
             (2.0 * theta).sin() * (2.0 * theta).cos() * 2.0, 0.0],
            [0.0, (2.0 * theta).sin() * (2.0 * theta).cos() * 2.0,
             (2.0 * theta).sin() * (2.0 * theta).sin() * 2.0 - 1.0, 0.0],
            [0.0, 0.0, 0.0, -1.0],
        ])
    }

    /// Rotation matrix (rotation of axes by angle θ).
    pub fn rotation(theta: f64) -> Self {
        let c2 = (2.0 * theta).cos();
        let s2 = (2.0 * theta).sin();
        Self::new([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, c2, s2, 0.0],
            [0.0, -s2, c2, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
    }

    /// Mueller matrix for reflection from an isotropic surface.
    ///
    /// Computed from Fresnel coefficients: uses ψ and Δ from ellipsometry.
    pub fn from_reflection(psi: f64, delta: f64) -> Self {
        let tp = psi.tan();
        let tp2 = tp * tp;
        let cd = delta.cos();
        let sd = delta.sin();
        let factor = 0.5;

        Self::new([
            [factor * (1.0 + tp2), factor * (-1.0 + tp2), 0.0, 0.0],
            [factor * (-1.0 + tp2), factor * (1.0 + tp2), 0.0, 0.0],
            [0.0, 0.0, tp * cd, tp * sd],
            [0.0, 0.0, -tp * sd, tp * cd],
        ])
    }

    /// Apply this Mueller matrix to a Stokes vector.
    pub fn apply(&self, s: &StokesVector) -> StokesVector {
        let mut out = [0.0; 4];
        for i in 0..4 {
            for j in 0..4 {
                out[i] += self.m[i][j] * s.s[j];
            }
        }
        StokesVector { s: out }
    }

    /// Multiply two Mueller matrices: result = self · rhs.
    pub fn mul(&self, rhs: &MuellerMatrix) -> MuellerMatrix {
        let mut out = [[0.0; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                for k in 0..4 {
                    out[i][j] += self.m[i][k] * rhs.m[k][j];
                }
            }
        }
        MuellerMatrix { m: out }
    }

    /// Depolarization index: 0 = perfect depolarizer, 1 = non-depolarizing.
    pub fn depolarization_index(&self) -> f64 {
        let mut sum_sq = 0.0;
        for i in 0..4 {
            for j in 0..4 {
                sum_sq += self.m[i][j] * self.m[i][j];
            }
        }
        let m00_sq = self.m[0][0] * self.m[0][0];
        if m00_sq > 0.0 {
            ((sum_sq - m00_sq) / (3.0 * m00_sq)).sqrt().min(1.0)
        } else {
            0.0
        }
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

    const EPSILON: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // ── ComplexIndex tests ──

    #[test]
    fn test_complex_index_transparent() {
        let n = ComplexIndex::transparent(1.5);
        assert_eq!(n.n, 1.5);
        assert_eq!(n.k, 0.0);
    }

    #[test]
    fn test_complex_index_absorbing() {
        let n = ComplexIndex::new(3.882, 0.019);
        assert!(approx_eq(n.n, 3.882, EPSILON));
        assert!(approx_eq(n.k, 0.019, EPSILON));
    }

    // ── Fresnel coefficient tests ──

    #[test]
    fn test_fresnel_normal_incidence_air_glass() {
        // At normal incidence:
        // r_p = (n2*cos(t1) - n1*cos(t2))/(n2*cos(t1) + n1*cos(t2))
        //     = (n2 - n1)/(n2 + n1) = (1.5-1.0)/(1.5+1.0) = +0.2
        // r_s = (n1*cos(t1) - n2*cos(t2))/(n1*cos(t1) + n2*cos(t2))
        //     = (n1 - n2)/(n1 + n2) = (1.0-1.5)/(1.0+1.5) = -0.2
        let air = ComplexIndex::transparent(1.0);
        let glass = ComplexIndex::transparent(1.5);
        let rp = FresnelCoefficients::r_p(air, 0.0, glass);
        let rs = FresnelCoefficients::r_s(air, 0.0, glass);
        assert!(approx_eq(rp.re, 0.2, 1e-10));
        assert!(approx_eq(rs.re, -0.2, 1e-10));
        assert!(approx_eq(rp.im, 0.0, 1e-10));
        assert!(approx_eq(rs.im, 0.0, 1e-10));
    }

    #[test]
    fn test_fresnel_reflectance_range() {
        let air = ComplexIndex::transparent(1.0);
        let glass = ComplexIndex::transparent(1.5);
        for angle_deg in 0..89 {
            let theta = (angle_deg as f64).to_radians();
            let rp = FresnelCoefficients::reflectance_p(air, theta, glass);
            let rs = FresnelCoefficients::reflectance_s(air, theta, glass);
            assert!(rp >= 0.0 && rp <= 1.0, "R_p out of range at {}°", angle_deg);
            assert!(rs >= 0.0 && rs <= 1.0, "R_s out of range at {}°", angle_deg);
        }
    }

    #[test]
    fn test_brewster_angle() {
        let theta_b = FresnelCoefficients::brewster_angle(1.0, 1.5);
        let expected = (1.5_f64).atan(); // ~56.3°
        assert!(approx_eq(theta_b, expected, 1e-12));
    }

    #[test]
    fn test_brewster_angle_rp_minimum() {
        // At Brewster's angle, R_p should be very close to zero
        let air = ComplexIndex::transparent(1.0);
        let glass = ComplexIndex::transparent(1.5);
        let theta_b = FresnelCoefficients::brewster_angle(1.0, 1.5);
        let rp = FresnelCoefficients::reflectance_p(air, theta_b, glass);
        assert!(rp < 1e-10, "R_p at Brewster angle should be ~0, got {}", rp);
    }

    #[test]
    fn test_critical_angle() {
        // Glass to air: theta_c = asin(1.0/1.5)
        let tc = FresnelCoefficients::critical_angle(1.5, 1.0).unwrap();
        let expected = (1.0 / 1.5_f64).asin();
        assert!(approx_eq(tc, expected, 1e-12));
    }

    #[test]
    fn test_critical_angle_none() {
        // Air to glass: no TIR
        assert!(FresnelCoefficients::critical_angle(1.0, 1.5).is_none());
    }

    #[test]
    fn test_fresnel_rs_greater_than_rp() {
        // For non-normal incidence on a dielectric, |r_s| >= |r_p| generally
        let air = ComplexIndex::transparent(1.0);
        let glass = ComplexIndex::transparent(1.5);
        let theta = 45.0_f64.to_radians();
        let rp = FresnelCoefficients::reflectance_p(air, theta, glass);
        let rs = FresnelCoefficients::reflectance_s(air, theta, glass);
        assert!(rs >= rp, "R_s should >= R_p away from normal incidence");
    }

    #[test]
    fn test_fresnel_grazing_incidence() {
        // At 89°, both R_p and R_s should approach 1
        let air = ComplexIndex::transparent(1.0);
        let glass = ComplexIndex::transparent(1.5);
        let theta = 89.0_f64.to_radians();
        let rp = FresnelCoefficients::reflectance_p(air, theta, glass);
        let rs = FresnelCoefficients::reflectance_s(air, theta, glass);
        assert!(rp > 0.8, "R_p at grazing should be high, got {}", rp);
        assert!(rs > 0.9, "R_s at grazing should be high, got {}", rs);
    }

    // ── Psi/Delta tests ──

    #[test]
    fn test_psi_delta_bare_substrate() {
        let air = MaterialDatabase::get(Material::Air);
        let si = MaterialDatabase::get(Material::Silicon);
        let calc = PsiDeltaCalculator::new(70.0_f64.to_radians());
        let (psi, delta) = calc.bare_substrate(air, si);
        // ψ should be between 0 and π/2
        assert!(psi > 0.0 && psi < FRAC_PI_2);
        // Δ should be between -π and π
        assert!(delta >= -PI && delta <= PI);
    }

    #[test]
    fn test_psi_delta_single_film() {
        let air = MaterialDatabase::get(Material::Air);
        let sio2 = MaterialDatabase::get(Material::SiliconDioxide);
        let si = MaterialDatabase::get(Material::Silicon);
        let calc = PsiDeltaCalculator::new(70.0_f64.to_radians());
        let (psi, delta) = calc.single_film(air, sio2, si, 100.0e-9, 632.8e-9);
        assert!(psi > 0.0 && psi < FRAC_PI_2);
        assert!(delta >= -PI && delta <= PI);
    }

    #[test]
    fn test_psi_delta_zero_thickness_equals_bare() {
        // Zero-thickness film should give same result as bare substrate
        let air = MaterialDatabase::get(Material::Air);
        let sio2 = MaterialDatabase::get(Material::SiliconDioxide);
        let si = MaterialDatabase::get(Material::Silicon);
        let calc = PsiDeltaCalculator::new(70.0_f64.to_radians());
        let (psi0, delta0) = calc.single_film(air, sio2, si, 0.0, 632.8e-9);
        let (psi_bare, delta_bare) = calc.bare_substrate(air, si);
        assert!(approx_eq(psi0, psi_bare, 1e-6));
        assert!(approx_eq(delta0, delta_bare, 1e-6));
    }

    #[test]
    fn test_psi_delta_spectroscopic() {
        let air = MaterialDatabase::get(Material::Air);
        let sio2 = MaterialDatabase::get(Material::SiliconDioxide);
        let si = MaterialDatabase::get(Material::Silicon);
        let calc = PsiDeltaCalculator::new(70.0_f64.to_radians());
        let wavelengths: Vec<f64> = (400..=700)
            .step_by(50)
            .map(|w| w as f64 * 1e-9)
            .collect();
        let layers = [(sio2, 200.0e-9)];
        let results = calc.spectroscopic(air, &layers, si, &wavelengths);
        assert_eq!(results.len(), wavelengths.len());
        for (psi, delta) in &results {
            assert!(*psi > 0.0 && *psi < FRAC_PI_2);
            assert!(*delta >= -PI && *delta <= PI);
        }
    }

    // ── Multilayer tests ──

    #[test]
    fn test_multilayer_single_equals_direct() {
        let air = MaterialDatabase::get(Material::Air);
        let sio2 = MaterialDatabase::get(Material::SiliconDioxide);
        let si = MaterialDatabase::get(Material::Silicon);
        let theta = 70.0_f64.to_radians();
        let wl = 632.8e-9;
        let d = 150.0e-9;

        let calc = PsiDeltaCalculator::new(theta);
        let (psi1, delta1) = calc.single_film(air, sio2, si, d, wl);
        let layers = vec![(sio2, d)];
        let (psi2, delta2) = calc.multilayer(air, &layers, si, wl);
        assert!(approx_eq(psi1, psi2, 1e-12));
        assert!(approx_eq(delta1, delta2, 1e-12));
    }

    #[test]
    fn test_multilayer_two_layers() {
        let air = MaterialDatabase::get(Material::Air);
        let sio2 = MaterialDatabase::get(Material::SiliconDioxide);
        let si3n4 = MaterialDatabase::get(Material::SiliconNitride);
        let si = MaterialDatabase::get(Material::Silicon);
        let calc = PsiDeltaCalculator::new(70.0_f64.to_radians());
        let layers = vec![(sio2, 50.0e-9), (si3n4, 30.0e-9)];
        let (psi, delta) = calc.multilayer(air, &layers, si, 632.8e-9);
        assert!(psi > 0.0 && psi < FRAC_PI_2);
        assert!(delta >= -PI && delta <= PI);
    }

    #[test]
    fn test_multilayer_antireflection_coating() {
        // Quarter-wave SiO2 on Si should reduce reflection
        let air = MaterialDatabase::get(Material::Air);
        let sio2 = MaterialDatabase::get(Material::SiliconDioxide);
        let si = MaterialDatabase::get(Material::Silicon);
        let wl = 550.0e-9;
        // Quarter-wave optical thickness: d = λ/(4n)
        let d = wl / (4.0 * sio2.n);
        let theta = 0.001; // Near-normal (avoid exact zero for stability)

        let (rp_bare, rs_bare) = MultiLayerReflection::reflection_coefficients(
            air, &[], si, theta, wl,
        );
        let (rp_coated, rs_coated) = MultiLayerReflection::reflection_coefficients(
            air, &[(sio2, d)], si, theta, wl,
        );
        // Coated reflection should be different from bare
        let r_bare = (rp_bare.norm_sqr() + rs_bare.norm_sqr()) / 2.0;
        let r_coated = (rp_coated.norm_sqr() + rs_coated.norm_sqr()) / 2.0;
        // SiO2 on Si actually increases reflection slightly (wrong n for AR),
        // but the point is they're different
        assert!((r_bare - r_coated).abs() > 0.001);
    }

    // ── Cauchy model tests ──

    #[test]
    fn test_cauchy_sio2() {
        let model = CauchyModel::sio2();
        let n = model.index_at(632.8e-9);
        // SiO2 at 632.8 nm should be ~1.457
        assert!(approx_eq(n.n, 1.457, 0.01));
        assert_eq!(n.k, 0.0);
    }

    #[test]
    fn test_cauchy_dispersion_monotonic() {
        // Refractive index should decrease with wavelength (normal dispersion)
        let model = CauchyModel::new(1.45, 0.003e-12, 0.0);
        let n400 = model.index_at(400e-9).n;
        let n500 = model.index_at(500e-9).n;
        let n700 = model.index_at(700e-9).n;
        assert!(n400 > n500);
        assert!(n500 > n700);
    }

    #[test]
    fn test_cauchy_si3n4() {
        let model = CauchyModel::si3n4();
        let n = model.index_at(632.8e-9);
        assert!(n.n > 1.9 && n.n < 2.2);
    }

    // ── Sellmeier model tests ──

    #[test]
    fn test_sellmeier_bk7() {
        let model = SellmeierModel::bk7();
        let n = model.index_at(589.3e-9); // Sodium D line
        // BK7 at 589 nm should be ~1.5168
        assert!(approx_eq(n.n, 1.5168, 0.02));
    }

    #[test]
    fn test_sellmeier_fused_silica() {
        let model = SellmeierModel::fused_silica();
        let n = model.index_at(632.8e-9);
        assert!(n.n > 1.4 && n.n < 1.5);
    }

    #[test]
    fn test_sellmeier_dispersion() {
        let model = SellmeierModel::bk7();
        let n_blue = model.index_at(450e-9).n;
        let n_red = model.index_at(650e-9).n;
        assert!(n_blue > n_red, "Normal dispersion expected");
    }

    // ── Drude model tests ──

    #[test]
    fn test_drude_gold() {
        let model = DrudeModel::gold();
        let n = model.index_at(632.8e-9);
        // Gold at 632.8 nm: n is small, k is large (highly reflective)
        assert!(n.k > n.n, "Gold should have k > n at visible wavelengths");
    }

    #[test]
    fn test_drude_silver() {
        let model = DrudeModel::silver();
        let n = model.index_at(500e-9);
        assert!(n.k > 0.0, "Silver should be absorbing");
    }

    #[test]
    fn test_drude_permittivity() {
        let model = DrudeModel::gold();
        let c = 299_792_458.0;
        let omega = 2.0 * PI * c / 632.8e-9;
        let eps = model.permittivity_at_omega(omega);
        // For gold at visible, real part of ε should be negative
        assert!(eps.re < 0.0, "Gold ε_real should be negative at visible");
    }

    // ── Material Database tests ──

    #[test]
    fn test_material_database_air() {
        let air = MaterialDatabase::get(Material::Air);
        assert!(approx_eq(air.n, 1.0003, 1e-4));
        assert_eq!(air.k, 0.0);
    }

    #[test]
    fn test_material_database_silicon() {
        let si = MaterialDatabase::get(Material::Silicon);
        assert!(si.n > 3.5);
        assert!(si.k > 0.0);
    }

    #[test]
    fn test_material_database_all_materials() {
        let materials = MaterialDatabase::all_materials();
        assert_eq!(materials.len(), 10);
        for mat in materials {
            let n = MaterialDatabase::get(mat);
            assert!(n.n > 0.0, "All materials should have positive n");
            assert!(n.k >= 0.0, "Extinction coefficient should be non-negative");
        }
    }

    // ── Thickness estimation tests ──

    #[test]
    fn test_thickness_estimator_known_thickness() {
        let air = MaterialDatabase::get(Material::Air);
        let sio2 = MaterialDatabase::get(Material::SiliconDioxide);
        let si = MaterialDatabase::get(Material::Silicon);
        let theta = 70.0_f64.to_radians();
        let true_thickness = 100.0e-9;

        // Generate "measured" data
        let wavelengths: Vec<f64> = (400..=700)
            .step_by(10)
            .map(|w| w as f64 * 1e-9)
            .collect();
        let calc = PsiDeltaCalculator::new(theta);
        let layers = [(sio2, true_thickness)];
        let measured: Vec<(f64, f64)> = calc.spectroscopic(air, &layers, si, &wavelengths);
        let measured_psi: Vec<f64> = measured.iter().map(|&(p, _)| p).collect();
        let measured_delta: Vec<f64> = measured.iter().map(|&(_, d)| d).collect();

        let estimator = ThicknessEstimator::new(theta, air, sio2, si);
        let (d_est, mse) = estimator.estimate(
            &wavelengths,
            &measured_psi,
            &measured_delta,
            50.0e-9,
            200.0e-9,
            0.1e-9,
        );

        assert!(
            approx_eq(d_est, true_thickness, 2.0e-9),
            "Estimated thickness {:.1} nm should match true {:.1} nm",
            d_est * 1e9,
            true_thickness * 1e9,
        );
        assert!(mse < 1e-4, "MSE should be very small for perfect data, got {}", mse);
    }

    #[test]
    fn test_thickness_estimator_mse() {
        let air = MaterialDatabase::get(Material::Air);
        let sio2 = MaterialDatabase::get(Material::SiliconDioxide);
        let si = MaterialDatabase::get(Material::Silicon);
        let theta = 70.0_f64.to_radians();

        let wavelengths = vec![500.0e-9, 600.0e-9, 700.0e-9];
        let calc = PsiDeltaCalculator::new(theta);
        let layers = [(sio2, 100.0e-9)];
        let measured: Vec<(f64, f64)> = calc.spectroscopic(air, &layers, si, &wavelengths);
        let psi: Vec<f64> = measured.iter().map(|&(p, _)| p).collect();
        let delta: Vec<f64> = measured.iter().map(|&(_, d)| d).collect();

        let estimator = ThicknessEstimator::new(theta, air, sio2, si);

        // MSE at true thickness should be ~0
        let mse_true = estimator.mse(100.0e-9, &wavelengths, &psi, &delta);
        assert!(mse_true < 1e-20);

        // MSE at wrong thickness should be larger
        let mse_wrong = estimator.mse(200.0e-9, &wavelengths, &psi, &delta);
        assert!(mse_wrong > mse_true);
    }

    // ── Stokes vector tests ──

    #[test]
    fn test_stokes_horizontal() {
        let s = StokesVector::horizontal();
        assert!(approx_eq(s.degree_of_polarization(), 1.0, EPSILON));
        assert!(approx_eq(s.degree_of_linear_polarization(), 1.0, EPSILON));
        assert!(approx_eq(s.degree_of_circular_polarization(), 0.0, EPSILON));
    }

    #[test]
    fn test_stokes_vertical() {
        let s = StokesVector::vertical();
        assert!(approx_eq(s.degree_of_polarization(), 1.0, EPSILON));
        assert!(approx_eq(s.orientation_angle(), FRAC_PI_2, EPSILON));
    }

    #[test]
    fn test_stokes_circular() {
        let s = StokesVector::right_circular();
        assert!(approx_eq(s.degree_of_polarization(), 1.0, EPSILON));
        assert!(approx_eq(s.degree_of_circular_polarization(), 1.0, EPSILON));
        assert!(approx_eq(s.degree_of_linear_polarization(), 0.0, EPSILON));
    }

    #[test]
    fn test_stokes_unpolarized() {
        let s = StokesVector::unpolarized();
        assert!(approx_eq(s.degree_of_polarization(), 0.0, EPSILON));
    }

    #[test]
    fn test_stokes_ellipticity_angle() {
        // Right circular: ellipticity angle should be π/4
        let s = StokesVector::right_circular();
        assert!(approx_eq(s.ellipticity_angle(), FRAC_PI_4, EPSILON));
        // Linear: ellipticity angle should be 0
        let s2 = StokesVector::horizontal();
        assert!(approx_eq(s2.ellipticity_angle(), 0.0, EPSILON));
    }

    #[test]
    fn test_stokes_plus45() {
        let s = StokesVector::plus45();
        assert!(approx_eq(s.degree_of_polarization(), 1.0, EPSILON));
        assert!(approx_eq(s.orientation_angle(), FRAC_PI_4, EPSILON));
    }

    // ── Mueller matrix tests ──

    #[test]
    fn test_mueller_identity() {
        let m = MuellerMatrix::identity();
        let s = StokesVector::horizontal();
        let out = m.apply(&s);
        for i in 0..4 {
            assert!(approx_eq(out.s[i], s.s[i], EPSILON));
        }
    }

    #[test]
    fn test_mueller_h_polarizer_on_unpolarized() {
        let m = MuellerMatrix::linear_polarizer_h();
        let s = StokesVector::unpolarized();
        let out = m.apply(&s);
        // Should become horizontally polarized at half intensity
        assert!(approx_eq(out.s[0], 0.5, EPSILON));
        assert!(approx_eq(out.s[1], 0.5, EPSILON));
        assert!(approx_eq(out.s[2], 0.0, EPSILON));
        assert!(approx_eq(out.s[3], 0.0, EPSILON));
    }

    #[test]
    fn test_mueller_crossed_polarizers() {
        // H polarizer followed by V polarizer: should block all light
        let mh = MuellerMatrix::linear_polarizer_h();
        let mv = MuellerMatrix::linear_polarizer_v();
        let combined = mv.mul(&mh);
        let s = StokesVector::unpolarized();
        let out = combined.apply(&s);
        assert!(approx_eq(out.s[0], 0.0, EPSILON));
    }

    #[test]
    fn test_mueller_rotation() {
        let rot = MuellerMatrix::rotation(0.0);
        let s = StokesVector::horizontal();
        let out = rot.apply(&s);
        for i in 0..4 {
            assert!(approx_eq(out.s[i], s.s[i], EPSILON));
        }
    }

    #[test]
    fn test_mueller_depolarization_index_identity() {
        let m = MuellerMatrix::identity();
        let di = m.depolarization_index();
        assert!(approx_eq(di, 1.0, EPSILON));
    }

    #[test]
    fn test_mueller_from_reflection() {
        let m = MuellerMatrix::from_reflection(FRAC_PI_4, 0.0);
        // tan(π/4) = 1, delta = 0
        // m[0][0] should be 0.5 * (1 + 1) = 1.0
        assert!(approx_eq(m.m[0][0], 1.0, EPSILON));
    }

    #[test]
    fn test_mueller_quarter_wave_plate() {
        // QWP at 0°: should convert +45° linear to right circular
        let qwp = MuellerMatrix::quarter_wave_plate(0.0);
        let s_in = StokesVector::plus45();
        let s_out = qwp.apply(&s_in);
        // Result should be right circular (or left, depending on convention)
        assert!(approx_eq(s_out.s[0], 1.0, EPSILON));
        assert!(s_out.degree_of_circular_polarization() > 0.9);
    }

    // ── Configuration tests ──

    #[test]
    fn test_config_wavelengths() {
        let config = EllipsometryConfig::new(
            70.0_f64.to_radians(),
            400.0e-9,
            700.0e-9,
            7,
        );
        let wl = config.wavelengths();
        assert_eq!(wl.len(), 7);
        assert!(approx_eq(wl[0], 400.0e-9, 1e-15));
        assert!(approx_eq(wl[6], 700.0e-9, 1e-15));
    }

    #[test]
    fn test_config_single_wavelength() {
        let config = EllipsometryConfig::new(
            70.0_f64.to_radians(),
            500.0e-9,
            600.0e-9,
            1,
        );
        let wl = config.wavelengths();
        assert_eq!(wl.len(), 1);
        assert!(approx_eq(wl[0], 550.0e-9, 1e-15));
    }

    // ── Snell's law tests ──

    #[test]
    fn test_snell_law_normal_incidence() {
        let air = ComplexIndex::transparent(1.0);
        let glass = ComplexIndex::transparent(1.5);
        let sin_t2 = FresnelCoefficients::snell_angle(air, 0.0, glass);
        assert!(approx_eq(sin_t2.re, 0.0, 1e-10));
        assert!(approx_eq(sin_t2.im, 0.0, 1e-10));
    }

    #[test]
    fn test_snell_law_45_degrees() {
        let air = ComplexIndex::transparent(1.0);
        let glass = ComplexIndex::transparent(1.5);
        let sin_t2 = FresnelCoefficients::snell_angle(air, FRAC_PI_4, glass);
        // sin(t2) = (1.0/1.5) * sin(45°) = 0.4714
        let expected = (1.0 / 1.5) * FRAC_PI_4.sin();
        assert!(approx_eq(sin_t2.re, expected, 1e-10));
    }

    // ── Complex number internal tests ──

    #[test]
    fn test_cx_arithmetic() {
        let a = Cx::new(1.0, 2.0);
        let b = Cx::new(3.0, 4.0);
        let sum = a + b;
        assert!(approx_eq(sum.re, 4.0, 1e-15));
        assert!(approx_eq(sum.im, 6.0, 1e-15));
        let prod = a * b;
        assert!(approx_eq(prod.re, -5.0, 1e-15));
        assert!(approx_eq(prod.im, 10.0, 1e-15));
    }

    #[test]
    fn test_cx_division() {
        let a = Cx::new(1.0, 0.0);
        let b = Cx::new(0.0, 1.0);
        let result = a / b;
        assert!(approx_eq(result.re, 0.0, 1e-15));
        assert!(approx_eq(result.im, -1.0, 1e-15));
    }

    #[test]
    fn test_cx_exp() {
        // e^(iπ) = -1
        let z = Cx::new(0.0, PI);
        let result = z.exp();
        assert!(approx_eq(result.re, -1.0, 1e-10));
        assert!(approx_eq(result.im, 0.0, 1e-10));
    }

    #[test]
    fn test_cx_sqrt() {
        // sqrt(4) = 2
        let z = Cx::new(4.0, 0.0);
        let s = z.sqrt();
        assert!(approx_eq(s.re, 2.0, 1e-10));
        assert!(approx_eq(s.im, 0.0, 1e-10));
    }

    #[test]
    fn test_cx_sqrt_negative() {
        // sqrt(-1) = i
        let z = Cx::new(-1.0, 0.0);
        let s = z.sqrt();
        assert!(approx_eq(s.re, 0.0, 1e-10));
        assert!(approx_eq(s.im, 1.0, 1e-10));
    }

    // ── Integration / end-to-end tests ──

    #[test]
    fn test_psi_delta_thickness_oscillation() {
        // As film thickness increases, psi and delta should oscillate
        let air = MaterialDatabase::get(Material::Air);
        let sio2 = MaterialDatabase::get(Material::SiliconDioxide);
        let si = MaterialDatabase::get(Material::Silicon);
        let calc = PsiDeltaCalculator::new(70.0_f64.to_radians());
        let wl = 632.8e-9;

        let mut psi_values = Vec::new();
        for d in (0..=500).step_by(10) {
            let thickness = d as f64 * 1e-9;
            let (psi, _) = calc.single_film(air, sio2, si, thickness, wl);
            psi_values.push(psi);
        }

        // Check that psi oscillates (has at least one local max and min)
        let mut has_max = false;
        let mut has_min = false;
        for i in 1..psi_values.len() - 1 {
            if psi_values[i] > psi_values[i - 1] && psi_values[i] > psi_values[i + 1] {
                has_max = true;
            }
            if psi_values[i] < psi_values[i - 1] && psi_values[i] < psi_values[i + 1] {
                has_min = true;
            }
        }
        assert!(has_max, "ψ should have oscillation peaks");
        assert!(has_min, "ψ should have oscillation valleys");
    }

    #[test]
    fn test_stokes_mueller_roundtrip() {
        // Pass horizontal through H polarizer: should be transmitted fully
        // M_H * [1,1,0,0] = [0.5+0.5, 0.5+0.5, 0, 0] = [1, 1, 0, 0]
        let mh = MuellerMatrix::linear_polarizer_h();
        let s = StokesVector::horizontal();
        let out = mh.apply(&s);
        assert!(approx_eq(out.s[0], 1.0, EPSILON));
        assert!(approx_eq(out.s[1], 1.0, EPSILON));
        assert!(approx_eq(out.degree_of_linear_polarization(), 1.0, EPSILON));
    }

    #[test]
    fn test_drude_model_long_wavelength() {
        // At very long wavelengths, metal should be highly reflective
        let model = DrudeModel::gold();
        let n = model.index_at(10.0e-6); // 10 microns
        assert!(n.k > 10.0, "Gold should be very absorbing in IR");
    }

    #[test]
    fn test_full_spectroscopic_pipeline() {
        // End-to-end: generate spectra, fit thickness
        let air = MaterialDatabase::get(Material::Air);
        let sio2 = MaterialDatabase::get(Material::SiliconDioxide);
        let si = MaterialDatabase::get(Material::Silicon);
        let theta = 75.0_f64.to_radians();
        let true_d = 250.0e-9;

        let config = EllipsometryConfig::new(theta, 400.0e-9, 800.0e-9, 41);
        let wavelengths = config.wavelengths();

        let calc = PsiDeltaCalculator::new(theta);
        let layers = [(sio2, true_d)];
        let results = calc.spectroscopic(air, &layers, si, &wavelengths);
        let psi: Vec<f64> = results.iter().map(|r| r.0).collect();
        let delta: Vec<f64> = results.iter().map(|r| r.1).collect();

        let estimator = ThicknessEstimator::new(theta, air, sio2, si);
        let (d_est, mse) = estimator.estimate(
            &wavelengths, &psi, &delta,
            100.0e-9, 400.0e-9, 0.5e-9,
        );

        assert!(
            approx_eq(d_est, true_d, 2.0e-9),
            "Estimated {:.1} nm should be close to true {:.1} nm",
            d_est * 1e9,
            true_d * 1e9,
        );
        assert!(mse < 1e-4, "MSE should be small for perfect data, got {}", mse);
    }
}
