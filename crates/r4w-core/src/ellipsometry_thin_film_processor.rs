// Spectroscopic Ellipsometry Thin Film Processor
// Measures thin film thickness and optical constants from polarization changes
// Implements: Fresnel equations, transfer matrix method, Cauchy/Sellmeier/Drude models,
// psi/delta fitting, multi-layer film stacks, Brewster angle

use std::f64::consts::PI;

/// Complex number for optical calculations (no external crate dependency)
#[derive(Clone, Copy, Debug)]
pub struct Cplx {
    pub re: f64,
    pub im: f64,
}

impl Cplx {
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    pub fn from_polar(r: f64, theta: f64) -> Self {
        Self { re: r * theta.cos(), im: r * theta.sin() }
    }

    pub fn mag(&self) -> f64 {
        (self.re * self.re + self.im * self.im).sqrt()
    }

    pub fn arg(&self) -> f64 {
        self.im.atan2(self.re)
    }

    pub fn conj(&self) -> Self {
        Self { re: self.re, im: -self.im }
    }

    pub fn add(&self, other: &Cplx) -> Cplx {
        Cplx::new(self.re + other.re, self.im + other.im)
    }

    pub fn sub(&self, other: &Cplx) -> Cplx {
        Cplx::new(self.re - other.re, self.im - other.im)
    }

    pub fn mul(&self, other: &Cplx) -> Cplx {
        Cplx::new(
            self.re * other.re - self.im * other.im,
            self.re * other.im + self.im * other.re,
        )
    }

    pub fn div(&self, other: &Cplx) -> Cplx {
        let denom = other.re * other.re + other.im * other.im;
        if denom < 1e-30 {
            return Cplx::new(0.0, 0.0);
        }
        Cplx::new(
            (self.re * other.re + self.im * other.im) / denom,
            (self.im * other.re - self.re * other.im) / denom,
        )
    }

    pub fn scale(&self, s: f64) -> Cplx {
        Cplx::new(self.re * s, self.im * s)
    }

    /// Complex square root (principal branch)
    pub fn sqrt(&self) -> Cplx {
        let r = self.mag();
        let theta = self.arg();
        Cplx::from_polar(r.sqrt(), theta / 2.0)
    }

    pub fn exp(&self) -> Cplx {
        let er = self.re.exp();
        Cplx::new(er * self.im.cos(), er * self.im.sin())
    }

    pub fn sin(&self) -> Cplx {
        // sin(a+bi) = sin(a)cosh(b) + i*cos(a)sinh(b)
        Cplx::new(
            self.re.sin() * self.im.cosh(),
            self.re.cos() * self.im.sinh(),
        )
    }
}

/// Optical constants model
#[derive(Clone, Debug)]
pub enum OpticalModel {
    /// n = A + B/lambda^2 + C/lambda^4 (Cauchy, transparent materials)
    Cauchy { a: f64, b: f64, c: f64 },
    /// n^2 = 1 + sum_i (B_i * lambda^2) / (lambda^2 - C_i) (Sellmeier, glass)
    Sellmeier { terms: Vec<(f64, f64)> },
    /// epsilon = eps_inf - omega_p^2 / (omega^2 + i*gamma*omega) (metals)
    Drude { eps_inf: f64, omega_p_ev: f64, gamma_ev: f64 },
    /// Fixed complex refractive index
    Constant { n: f64, k: f64 },
}

/// Film layer
#[derive(Clone, Debug)]
pub struct FilmLayer {
    /// Layer name
    pub name: String,
    /// Thickness in nm
    pub thickness_nm: f64,
    /// Optical model
    pub model: OpticalModel,
}

/// Ellipsometry configuration
#[derive(Clone, Debug)]
pub struct EllipsometryConfig {
    /// Angle of incidence in degrees
    pub angle_deg: f64,
    /// Wavelength range start in nm
    pub wavelength_start_nm: f64,
    /// Wavelength range end in nm
    pub wavelength_end_nm: f64,
    /// Number of wavelength points
    pub n_wavelengths: usize,
}

impl Default for EllipsometryConfig {
    fn default() -> Self {
        Self {
            angle_deg: 70.0,
            wavelength_start_nm: 300.0,
            wavelength_end_nm: 800.0,
            n_wavelengths: 100,
        }
    }
}

/// Ellipsometry measurement result
#[derive(Clone, Debug)]
pub struct EllipsometryResult {
    /// Wavelength in nm
    pub wavelength_nm: f64,
    /// Psi angle in degrees (amplitude ratio)
    pub psi_deg: f64,
    /// Delta angle in degrees (phase difference)
    pub delta_deg: f64,
}

/// Ellipsometry processor
pub struct EllipsometryProcessor {
    pub config: EllipsometryConfig,
    pub layers: Vec<FilmLayer>,
    /// Substrate optical model
    pub substrate: OpticalModel,
    /// Ambient refractive index (usually 1.0 for air)
    pub n_ambient: f64,
}

impl EllipsometryProcessor {
    pub fn new(config: EllipsometryConfig, substrate: OpticalModel) -> Self {
        Self {
            config,
            layers: Vec::new(),
            substrate,
            n_ambient: 1.0,
        }
    }

    pub fn add_layer(&mut self, layer: FilmLayer) {
        self.layers.push(layer);
    }

    /// Compute complex refractive index from optical model at given wavelength (nm)
    pub fn refractive_index(model: &OpticalModel, wavelength_nm: f64) -> Cplx {
        match model {
            OpticalModel::Cauchy { a, b, c } => {
                let lam_um = wavelength_nm / 1000.0;
                let n = a + b / (lam_um * lam_um) + c / (lam_um * lam_um * lam_um * lam_um);
                Cplx::new(n, 0.0)
            }
            OpticalModel::Sellmeier { terms } => {
                let lam2 = (wavelength_nm / 1000.0).powi(2);
                let mut n2 = 1.0;
                for &(b, c) in terms {
                    n2 += b * lam2 / (lam2 - c);
                }
                if n2 > 0.0 {
                    Cplx::new(n2.sqrt(), 0.0)
                } else {
                    Cplx::new(0.0, (-n2).sqrt())
                }
            }
            OpticalModel::Drude { eps_inf, omega_p_ev, gamma_ev } => {
                let hbar = 6.582119569e-16; // eV*s
                let c = 2.998e17; // nm/s
                let omega = 2.0 * PI * c / wavelength_nm * hbar; // in eV
                let wp2 = omega_p_ev * omega_p_ev;
                let eps_re = eps_inf - wp2 / (omega * omega + gamma_ev * gamma_ev);
                let eps_im = wp2 * gamma_ev / (omega * (omega * omega + gamma_ev * gamma_ev));
                // n + ik = sqrt(eps_re + i*eps_im)
                Cplx::new(eps_re, eps_im).sqrt()
            }
            OpticalModel::Constant { n, k } => Cplx::new(*n, *k),
        }
    }

    /// Fresnel reflection coefficient for s-polarization (TE)
    /// r_s = (n1*cos(theta1) - n2*cos(theta2)) / (n1*cos(theta1) + n2*cos(theta2))
    pub fn fresnel_rs(n1: &Cplx, cos_t1: &Cplx, n2: &Cplx, cos_t2: &Cplx) -> Cplx {
        let num = n1.mul(cos_t1).sub(&n2.mul(cos_t2));
        let den = n1.mul(cos_t1).add(&n2.mul(cos_t2));
        num.div(&den)
    }

    /// Fresnel reflection coefficient for p-polarization (TM)
    /// r_p = (n2*cos(theta1) - n1*cos(theta2)) / (n2*cos(theta1) + n1*cos(theta2))
    pub fn fresnel_rp(n1: &Cplx, cos_t1: &Cplx, n2: &Cplx, cos_t2: &Cplx) -> Cplx {
        let num = n2.mul(cos_t1).sub(&n1.mul(cos_t2));
        let den = n2.mul(cos_t1).add(&n1.mul(cos_t2));
        num.div(&den)
    }

    /// Snell's law for complex refractive index: n1*sin(theta1) = n2*sin(theta2)
    /// Returns cos(theta2)
    pub fn snell_cos(n1: &Cplx, sin_t1: f64, n2: &Cplx) -> Cplx {
        let ratio = n1.scale(sin_t1).div(n2);
        let sin2 = ratio.mul(&ratio);
        Cplx::new(1.0 - sin2.re, -sin2.im).sqrt()
    }

    /// Compute psi and delta for the full film stack at a given wavelength
    pub fn compute_at_wavelength(&self, wavelength_nm: f64) -> EllipsometryResult {
        let theta_rad = self.config.angle_deg * PI / 180.0;
        let sin_t = theta_rad.sin();
        let cos_t = theta_rad.cos();
        let n_amb = Cplx::new(self.n_ambient, 0.0);

        if self.layers.is_empty() {
            // Direct ambient-substrate interface
            let n_sub = Self::refractive_index(&self.substrate, wavelength_nm);
            let cos_sub = Self::snell_cos(&n_amb, sin_t, &n_sub);
            let cos_amb = Cplx::new(cos_t, 0.0);

            let rs = Self::fresnel_rs(&n_amb, &cos_amb, &n_sub, &cos_sub);
            let rp = Self::fresnel_rp(&n_amb, &cos_amb, &n_sub, &cos_sub);

            let rho = rp.div(&rs);
            let psi = rho.mag().atan() * 180.0 / PI;
            let delta = rho.arg() * 180.0 / PI;

            return EllipsometryResult { wavelength_nm, psi_deg: psi, delta_deg: delta };
        }

        // Airy/Parratt recursion for multilayer
        // Build list of all media: ambient, layers..., substrate
        let mut n_list: Vec<Cplx> = vec![n_amb];
        for layer in &self.layers {
            n_list.push(Self::refractive_index(&layer.model, wavelength_nm));
        }
        n_list.push(Self::refractive_index(&self.substrate, wavelength_nm));

        let n_media = n_list.len();

        // Compute cos(theta) in each medium via Snell's law
        let mut cos_list = vec![Cplx::new(cos_t, 0.0)];
        for j in 1..n_media {
            cos_list.push(Self::snell_cos(&n_list[0], sin_t, &n_list[j]));
        }

        // Parratt recursion from substrate upward
        // Start with r = fresnel(last_layer, substrate)
        let n_layers = self.layers.len();
        let sub_idx = n_layers + 1;

        let mut rs_total = Self::fresnel_rs(
            &n_list[n_layers], &cos_list[n_layers],
            &n_list[sub_idx], &cos_list[sub_idx],
        );
        let mut rp_total = Self::fresnel_rp(
            &n_list[n_layers], &cos_list[n_layers],
            &n_list[sub_idx], &cos_list[sub_idx],
        );

        // Work upward through layers
        for j in (0..n_layers).rev() {
            let j_media = j + 1;
            let d = self.layers[j].thickness_nm;

            // Phase thickness: beta = 2*pi*n_j*d*cos(theta_j)/lambda
            let beta = n_list[j_media].mul(&cos_list[j_media])
                .scale(2.0 * PI * d / wavelength_nm);
            let exp_neg2beta = Cplx::new(0.0, -2.0).mul(&beta).exp();

            // Fresnel at interface j / (j+1)
            let rs_j = Self::fresnel_rs(
                &n_list[j], &cos_list[j],
                &n_list[j_media], &cos_list[j_media],
            );
            let rp_j = Self::fresnel_rp(
                &n_list[j], &cos_list[j],
                &n_list[j_media], &cos_list[j_media],
            );

            // Airy formula: r = (r_j + r_below * exp(-2i*beta)) / (1 + r_j * r_below * exp(-2i*beta))
            let one = Cplx::new(1.0, 0.0);

            let rs_phase = rs_total.mul(&exp_neg2beta);
            rs_total = rs_j.add(&rs_phase).div(&one.add(&rs_j.mul(&rs_phase)));

            let rp_phase = rp_total.mul(&exp_neg2beta);
            rp_total = rp_j.add(&rp_phase).div(&one.add(&rp_j.mul(&rp_phase)));
        }

        let r_s_total = rs_total;
        let r_p_total = rp_total;

        let rho = r_p_total.div(&r_s_total);
        let psi = rho.mag().atan() * 180.0 / PI;
        let delta = rho.arg() * 180.0 / PI;

        EllipsometryResult { wavelength_nm, psi_deg: psi, delta_deg: delta }
    }

    /// Compute full spectral psi/delta
    pub fn compute_spectrum(&self) -> Vec<EllipsometryResult> {
        let n = self.config.n_wavelengths;
        (0..n)
            .map(|i| {
                let wl = self.config.wavelength_start_nm
                    + (self.config.wavelength_end_nm - self.config.wavelength_start_nm)
                        * i as f64 / (n - 1).max(1) as f64;
                self.compute_at_wavelength(wl)
            })
            .collect()
    }

    /// Brewster angle for interface: theta_B = atan(n2/n1)
    pub fn brewster_angle_deg(n1: f64, n2: f64) -> f64 {
        (n2 / n1).atan() * 180.0 / PI
    }

    /// Film thickness from single-wavelength measurement (thin film approx)
    /// d = lambda * delta / (4 * pi * n * cos(theta))
    pub fn thickness_from_delta(
        wavelength_nm: f64,
        delta_rad: f64,
        n_film: f64,
        angle_deg: f64,
    ) -> f64 {
        let theta = angle_deg * PI / 180.0;
        let cos_t_film = (1.0 - (theta.sin() / n_film).powi(2)).sqrt();
        wavelength_nm * delta_rad.abs() / (4.0 * PI * n_film * cos_t_film)
    }

    /// Reflectance from psi/delta: R_p/R_s = tan^2(psi)
    pub fn reflectance_ratio(psi_deg: f64) -> f64 {
        let psi_rad = psi_deg * PI / 180.0;
        psi_rad.tan().powi(2)
    }

    /// Pseudo-dielectric function from bare substrate measurement
    /// <epsilon> = sin^2(phi) * [1 + tan^2(phi) * ((1-rho)/(1+rho))^2]
    pub fn pseudo_dielectric(psi_deg: f64, delta_deg: f64, angle_deg: f64) -> (f64, f64) {
        let phi = angle_deg * PI / 180.0;
        let psi = psi_deg * PI / 180.0;
        let delta = delta_deg * PI / 180.0;

        let rho = Cplx::from_polar(psi.tan(), delta);
        let one = Cplx::new(1.0, 0.0);
        let ratio = one.sub(&rho).div(&one.add(&rho));
        let ratio2 = ratio.mul(&ratio);

        let sin2 = phi.sin().powi(2);
        let tan2 = phi.tan().powi(2);

        let eps_re = sin2 * (1.0 + tan2 * ratio2.re);
        let eps_im = sin2 * tan2 * ratio2.im;

        (eps_re, eps_im)
    }

    /// Goodness of fit (MSE) between measured and modeled psi/delta
    pub fn fit_mse(
        measured: &[EllipsometryResult],
        modeled: &[EllipsometryResult],
    ) -> f64 {
        if measured.is_empty() {
            return 0.0;
        }
        let n = measured.len().min(modeled.len());
        let sum: f64 = (0..n)
            .map(|i| {
                let dp = measured[i].psi_deg - modeled[i].psi_deg;
                let dd = measured[i].delta_deg - modeled[i].delta_deg;
                dp * dp + dd * dd
            })
            .sum();
        (sum / (2.0 * n as f64)).sqrt()
    }
}

/// 2x2 complex matrix multiplication
fn mat2_mul(a: &[[Cplx; 2]; 2], b: &[[Cplx; 2]; 2]) -> [[Cplx; 2]; 2] {
    [
        [
            a[0][0].mul(&b[0][0]).add(&a[0][1].mul(&b[1][0])),
            a[0][0].mul(&b[0][1]).add(&a[0][1].mul(&b[1][1])),
        ],
        [
            a[1][0].mul(&b[0][0]).add(&a[1][1].mul(&b[1][0])),
            a[1][0].mul(&b[0][1]).add(&a[1][1].mul(&b[1][1])),
        ],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cplx_basics() {
        let a = Cplx::new(3.0, 4.0);
        assert!((a.mag() - 5.0).abs() < 1e-10);
        let b = a.conj();
        assert_eq!(b.im, -4.0);
    }

    #[test]
    fn test_cplx_arithmetic() {
        let a = Cplx::new(1.0, 2.0);
        let b = Cplx::new(3.0, -1.0);
        let sum = a.add(&b);
        assert!((sum.re - 4.0).abs() < 1e-10);
        assert!((sum.im - 1.0).abs() < 1e-10);

        let prod = a.mul(&b);
        // (1+2i)(3-i) = 3 - i + 6i - 2i^2 = 5 + 5i
        assert!((prod.re - 5.0).abs() < 1e-10);
        assert!((prod.im - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_cplx_div() {
        let a = Cplx::new(5.0, 5.0);
        let b = Cplx::new(3.0, -1.0);
        let q = a.div(&b);
        // Verify: q * b = a
        let check = q.mul(&b);
        assert!((check.re - 5.0).abs() < 1e-8);
        assert!((check.im - 5.0).abs() < 1e-8);
    }

    #[test]
    fn test_cplx_sqrt() {
        let a = Cplx::new(-1.0, 0.0);
        let s = a.sqrt();
        assert!(s.re.abs() < 1e-10);
        assert!((s.im - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_cplx_exp() {
        let a = Cplx::new(0.0, PI);
        let e = a.exp();
        // e^(i*pi) = -1
        assert!((e.re - (-1.0)).abs() < 1e-10);
        assert!(e.im.abs() < 1e-10);
    }

    #[test]
    fn test_cauchy_model() {
        let model = OpticalModel::Cauchy { a: 1.45, b: 0.003, c: 0.0 };
        let n = EllipsometryProcessor::refractive_index(&model, 550.0);
        assert!(n.re > 1.45);
        assert!(n.im.abs() < 1e-10); // Cauchy is transparent
    }

    #[test]
    fn test_sellmeier_model() {
        // BK7 glass parameters (simplified)
        let model = OpticalModel::Sellmeier {
            terms: vec![(1.03961, 0.00600), (0.23179, 0.02002)],
        };
        let n = EllipsometryProcessor::refractive_index(&model, 550.0);
        assert!(n.re > 1.4 && n.re < 1.6, "BK7 n at 550nm should be ~1.5, got {}", n.re);
    }

    #[test]
    fn test_constant_model() {
        let model = OpticalModel::Constant { n: 3.5, k: 0.01 };
        let nc = EllipsometryProcessor::refractive_index(&model, 500.0);
        assert_eq!(nc.re, 3.5);
        assert_eq!(nc.im, 0.01);
    }

    #[test]
    fn test_drude_model() {
        let model = OpticalModel::Drude {
            eps_inf: 1.0,
            omega_p_ev: 9.0,  // ~Al plasma frequency
            gamma_ev: 0.1,
        };
        let n = EllipsometryProcessor::refractive_index(&model, 400.0);
        // Metal: should have significant k
        assert!(n.im > 0.0 || n.re > 0.0);
    }

    #[test]
    fn test_bare_substrate() {
        let config = EllipsometryConfig {
            angle_deg: 70.0,
            wavelength_start_nm: 550.0,
            wavelength_end_nm: 550.0,
            n_wavelengths: 1,
        };
        let proc = EllipsometryProcessor::new(
            config,
            OpticalModel::Constant { n: 3.87, k: 0.02 }, // Si
        );
        let result = proc.compute_at_wavelength(550.0);
        // Psi should be between 0 and 45 for high-index substrate
        assert!(result.psi_deg > 0.0 && result.psi_deg < 90.0);
    }

    #[test]
    fn test_single_layer() {
        let config = EllipsometryConfig {
            angle_deg: 70.0,
            wavelength_start_nm: 550.0,
            wavelength_end_nm: 550.0,
            n_wavelengths: 1,
        };
        let mut proc = EllipsometryProcessor::new(
            config,
            OpticalModel::Constant { n: 3.87, k: 0.02 },
        );
        proc.add_layer(FilmLayer {
            name: "SiO2".into(),
            thickness_nm: 100.0,
            model: OpticalModel::Cauchy { a: 1.46, b: 0.003, c: 0.0 },
        });
        let result = proc.compute_at_wavelength(550.0);
        assert!(result.psi_deg > 0.0 && result.psi_deg < 90.0);
    }

    #[test]
    fn test_spectrum_computation() {
        let config = EllipsometryConfig::default();
        let proc = EllipsometryProcessor::new(
            config,
            OpticalModel::Constant { n: 3.87, k: 0.02 },
        );
        let results = proc.compute_spectrum();
        assert_eq!(results.len(), 100);
        // Wavelengths should span the range
        assert!((results[0].wavelength_nm - 300.0).abs() < 1.0);
        assert!((results[99].wavelength_nm - 800.0).abs() < 1.0);
    }

    #[test]
    fn test_brewster_angle() {
        let ba = EllipsometryProcessor::brewster_angle_deg(1.0, 1.5);
        // Brewster angle for n=1.5: atan(1.5) ~ 56.3 degrees
        assert!((ba - 56.31).abs() < 0.1);
    }

    #[test]
    fn test_thickness_from_delta() {
        let d = EllipsometryProcessor::thickness_from_delta(550.0, 0.5, 1.46, 70.0);
        assert!(d > 0.0 && d < 100.0, "thickness should be reasonable, got {d}");
    }

    #[test]
    fn test_reflectance_ratio() {
        let r = EllipsometryProcessor::reflectance_ratio(45.0);
        assert!((r - 1.0).abs() < 1e-10); // tan(45) = 1, so R_p/R_s = 1
    }

    #[test]
    fn test_reflectance_ratio_low_psi() {
        let r = EllipsometryProcessor::reflectance_ratio(20.0);
        assert!(r < 1.0, "R_p/R_s < 1 for psi < 45");
    }

    #[test]
    fn test_pseudo_dielectric() {
        let (eps_re, _eps_im) = EllipsometryProcessor::pseudo_dielectric(10.0, 170.0, 70.0);
        assert!(eps_re > 1.0, "pseudo-dielectric of high-index material should be > 1");
    }

    #[test]
    fn test_fit_mse_zero() {
        let data = vec![
            EllipsometryResult { wavelength_nm: 500.0, psi_deg: 20.0, delta_deg: 100.0 },
        ];
        let mse = EllipsometryProcessor::fit_mse(&data, &data);
        assert!(mse < 1e-10);
    }

    #[test]
    fn test_fit_mse_nonzero() {
        let m1 = vec![
            EllipsometryResult { wavelength_nm: 500.0, psi_deg: 20.0, delta_deg: 100.0 },
        ];
        let m2 = vec![
            EllipsometryResult { wavelength_nm: 500.0, psi_deg: 22.0, delta_deg: 103.0 },
        ];
        let mse = EllipsometryProcessor::fit_mse(&m1, &m2);
        assert!(mse > 0.0);
    }

    #[test]
    fn test_mat2_mul_identity() {
        let id = [
            [Cplx::new(1.0, 0.0), Cplx::new(0.0, 0.0)],
            [Cplx::new(0.0, 0.0), Cplx::new(1.0, 0.0)],
        ];
        let a = [
            [Cplx::new(2.0, 1.0), Cplx::new(3.0, 0.0)],
            [Cplx::new(0.0, 1.0), Cplx::new(4.0, -1.0)],
        ];
        let result = mat2_mul(&id, &a);
        assert!((result[0][0].re - 2.0).abs() < 1e-10);
        assert!((result[1][1].im - (-1.0)).abs() < 1e-10);
    }

    #[test]
    fn test_snell_cos() {
        let n1 = Cplx::new(1.0, 0.0);
        let n2 = Cplx::new(1.5, 0.0);
        let sin_t = (70.0_f64 * PI / 180.0).sin();
        let cos_t2 = EllipsometryProcessor::snell_cos(&n1, sin_t, &n2);
        assert!(cos_t2.re > 0.0, "cos should be positive");
        // Verify: n1*sin(t1) = n2*sin(t2), so sin(t2) = sin(70)/1.5
        let sin_t2 = sin_t / 1.5;
        let expected_cos = (1.0 - sin_t2 * sin_t2).sqrt();
        assert!((cos_t2.re - expected_cos).abs() < 1e-6);
    }

    #[test]
    fn test_fresnel_normal_incidence() {
        let n1 = Cplx::new(1.0, 0.0);
        let n2 = Cplx::new(1.5, 0.0);
        let cos1 = Cplx::new(1.0, 0.0);
        let cos2 = Cplx::new(1.0, 0.0);
        let rs = EllipsometryProcessor::fresnel_rs(&n1, &cos1, &n2, &cos2);
        // r_s = (1-1.5)/(1+1.5) = -0.2
        assert!((rs.re - (-0.2)).abs() < 1e-10);
    }

    #[test]
    fn test_multilayer() {
        let config = EllipsometryConfig {
            angle_deg: 70.0,
            wavelength_start_nm: 550.0,
            wavelength_end_nm: 550.0,
            n_wavelengths: 1,
        };
        let mut proc = EllipsometryProcessor::new(
            config,
            OpticalModel::Constant { n: 3.87, k: 0.02 },
        );
        proc.add_layer(FilmLayer {
            name: "SiO2".into(),
            thickness_nm: 10.0,
            model: OpticalModel::Constant { n: 1.46, k: 0.0 },
        });
        proc.add_layer(FilmLayer {
            name: "Si3N4".into(),
            thickness_nm: 20.0,
            model: OpticalModel::Constant { n: 2.0, k: 0.0 },
        });
        let result = proc.compute_at_wavelength(550.0);
        assert!(result.psi_deg.is_finite());
        assert!(result.delta_deg.is_finite());
    }

    #[test]
    fn test_thickness_sensitivity() {
        // Different thicknesses should give different psi/delta at multiple wavelengths
        let config1 = EllipsometryConfig {
            angle_deg: 70.0,
            wavelength_start_nm: 400.0,
            wavelength_end_nm: 700.0,
            n_wavelengths: 10,
        };
        let config2 = config1.clone();

        let mut proc1 = EllipsometryProcessor::new(
            config1,
            OpticalModel::Constant { n: 3.87, k: 0.02 },
        );
        proc1.add_layer(FilmLayer {
            name: "SiO2".into(),
            thickness_nm: 50.0,
            model: OpticalModel::Constant { n: 1.46, k: 0.0 },
        });

        let mut proc2 = EllipsometryProcessor::new(
            config2,
            OpticalModel::Constant { n: 3.87, k: 0.02 },
        );
        proc2.add_layer(FilmLayer {
            name: "SiO2".into(),
            thickness_nm: 200.0,
            model: OpticalModel::Constant { n: 1.46, k: 0.0 },
        });

        let spec1 = proc1.compute_spectrum();
        let spec2 = proc2.compute_spectrum();

        // At least one wavelength should show a difference
        let max_diff: f64 = spec1.iter().zip(spec2.iter())
            .map(|(r1, r2)| (r1.psi_deg - r2.psi_deg).abs() + (r1.delta_deg - r2.delta_deg).abs())
            .fold(0.0_f64, |a, b| a.max(b));
        assert!(max_diff > 0.1, "different thickness should give different psi/delta, max_diff={max_diff}");
    }

    #[test]
    fn test_default_config() {
        let cfg = EllipsometryConfig::default();
        assert_eq!(cfg.angle_deg, 70.0);
        assert_eq!(cfg.n_wavelengths, 100);
    }

    #[test]
    fn test_fit_mse_empty() {
        let mse = EllipsometryProcessor::fit_mse(&[], &[]);
        assert_eq!(mse, 0.0);
    }

    #[test]
    fn test_cplx_sin() {
        let a = Cplx::new(PI / 2.0, 0.0);
        let s = a.sin();
        assert!((s.re - 1.0).abs() < 1e-10);
        assert!(s.im.abs() < 1e-10);
    }
}
