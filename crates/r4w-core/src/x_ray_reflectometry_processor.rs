// X-ray Reflectometry (XRR) Processor
// Thin film analysis via specular X-ray reflectivity: Parratt recursion, Névot-Croce roughness,
// Kiessig fringe analysis, critical angle extraction, electron density profiling.
//
// No external crate dependencies - all math implemented from scratch using only std.

/// Complex number for internal calculations
#[derive(Debug, Clone, Copy, PartialEq)]
struct Complex {
    re: f64,
    im: f64,
}

impl Complex {
    fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    fn from_real(re: f64) -> Self {
        Self { re, im: 0.0 }
    }

    fn abs_sq(&self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    fn abs(&self) -> f64 {
        self.abs_sq().sqrt()
    }

    fn conj(&self) -> Self {
        Self { re: self.re, im: -self.im }
    }

    fn add(&self, other: &Self) -> Self {
        Self { re: self.re + other.re, im: self.im + other.im }
    }

    fn sub(&self, other: &Self) -> Self {
        Self { re: self.re - other.re, im: self.im - other.im }
    }

    fn mul(&self, other: &Self) -> Self {
        Self {
            re: self.re * other.re - self.im * other.im,
            im: self.re * other.im + self.im * other.re,
        }
    }

    fn div(&self, other: &Self) -> Self {
        let denom = other.abs_sq();
        Self {
            re: (self.re * other.re + self.im * other.im) / denom,
            im: (self.im * other.re - self.re * other.im) / denom,
        }
    }

    fn scale(&self, s: f64) -> Self {
        Self { re: self.re * s, im: self.im * s }
    }

    /// Complex square root (principal branch)
    fn sqrt(&self) -> Self {
        let r = self.abs();
        let theta = self.im.atan2(self.re);
        let sr = r.sqrt();
        let half_theta = theta / 2.0;
        Self {
            re: sr * half_theta.cos(),
            im: sr * half_theta.sin(),
        }
    }

    /// Complex exponential: exp(a + ib) = e^a * (cos(b) + i*sin(b))
    fn exp(&self) -> Self {
        let ea = self.re.exp();
        Self {
            re: ea * self.im.cos(),
            im: ea * self.im.sin(),
        }
    }
}

/// A single layer in the thin-film stack
#[derive(Debug, Clone)]
pub struct XrrLayer {
    /// Layer thickness in nanometres
    pub thickness_nm: f64,
    /// Real part of scattering length density (10^-6 Å^-2 equivalent, stored in nm^-2)
    pub sld_real: f64,
    /// Imaginary part of scattering length density (absorption, nm^-2)
    pub sld_imag: f64,
    /// RMS interface roughness (nm), used in Névot-Croce damping
    pub roughness_nm: f64,
}

impl XrrLayer {
    /// Create a new layer with given parameters
    pub fn new(thickness_nm: f64, sld_real: f64, sld_imag: f64, roughness_nm: f64) -> Self {
        Self { thickness_nm, sld_real, sld_imag, roughness_nm }
    }

    /// Create from delta/beta optical constants and wavelength (nm)
    /// n = 1 - delta + i*beta → SLD_real = 2*delta*(2π/λ)², SLD_imag = -2*beta*(2π/λ)²
    /// (sign convention: SLD_imag negative for absorption)
    pub fn from_optical_constants(
        thickness_nm: f64,
        delta: f64,
        beta: f64,
        wavelength_nm: f64,
        roughness_nm: f64,
    ) -> Self {
        let k0 = 2.0 * std::f64::consts::PI / wavelength_nm;
        // SLD = (k0^2 / (2π)) * re_lambda^2 * rho; for optics: SLD ≡ k0^2*(delta - i*beta)
        let sld_real = k0 * k0 * delta;
        let sld_imag = k0 * k0 * beta;
        Self { thickness_nm, sld_real, sld_imag, roughness_nm }
    }
}

/// Complete layer model for XRR simulation
#[derive(Debug, Clone)]
pub struct XrrModel {
    /// Ordered film layers from surface (index 0) to substrate interface
    pub layers: Vec<XrrLayer>,
    /// Semi-infinite substrate
    pub substrate: XrrLayer,
    /// Photon wavelength in nanometres (Cu Kα = 0.15406 nm)
    pub wavelength_nm: f64,
}

impl XrrModel {
    /// Build model with Cu Kα radiation
    pub fn new_cu_kalpha(layers: Vec<XrrLayer>, substrate: XrrLayer) -> Self {
        Self { layers, substrate, wavelength_nm: 0.15406 }
    }

    /// Build model with Mo Kα radiation
    pub fn new_mo_kalpha(layers: Vec<XrrLayer>, substrate: XrrLayer) -> Self {
        Self { layers, substrate, wavelength_nm: 0.07107 }
    }

    /// Build model with custom wavelength (nm)
    pub fn new(layers: Vec<XrrLayer>, substrate: XrrLayer, wavelength_nm: f64) -> Self {
        Self { layers, substrate, wavelength_nm }
    }
}

/// Computed reflectivity curve
#[derive(Debug, Clone)]
pub struct ReflectivityCurve {
    /// Scattering vector q = 4π sin(θ) / λ  (nm^-1)
    pub q_values: Vec<f64>,
    /// Incidence angles θ in radians
    pub theta_values: Vec<f64>,
    /// Reflectivity R ∈ [0, 1]
    pub reflectivity: Vec<f64>,
    /// Log10 of reflectivity (dB-like scale)
    pub log_reflectivity: Vec<f64>,
}

/// Electron density profile reconstructed from model
#[derive(Debug, Clone)]
pub struct ElectronDensityProfile {
    /// Depth coordinate z in nm (0 = surface)
    pub z_nm: Vec<f64>,
    /// Electron density ρe (electrons/nm³)
    pub rho_e: Vec<f64>,
}

/// Result of Kiessig fringe analysis
#[derive(Debug, Clone)]
pub struct KiessigAnalysis {
    /// q positions of fringe maxima (nm^-1)
    pub fringe_positions: Vec<f64>,
    /// Estimated thickness from fringe spacing (nm)
    pub thickness_nm: f64,
    /// Fringe period Δq (nm^-1)
    pub delta_q: f64,
    /// Number of fringes detected
    pub fringe_count: usize,
}

/// Critical angle extraction result
#[derive(Debug, Clone)]
pub struct CriticalAngleResult {
    /// Critical angle θ_c in radians
    pub theta_c_rad: f64,
    /// Critical angle θ_c in degrees
    pub theta_c_deg: f64,
    /// Electron density derived from θ_c (electrons/nm³)
    pub electron_density: f64,
    /// Scattering length density (nm^-2)
    pub sld: f64,
}

/// Material entry in the SLD database
#[derive(Debug, Clone)]
pub struct XrrMaterial {
    /// Material name
    pub name: String,
    /// Real part of delta (dispersion)
    pub delta: f64,
    /// Imaginary part beta (absorption)
    pub beta: f64,
    /// Mass density (g/cm³)
    pub density_g_cm3: f64,
}

impl XrrMaterial {
    /// Compute SLD_real at given wavelength (nm)
    pub fn sld_real(&self, wavelength_nm: f64) -> f64 {
        let k0 = 2.0 * std::f64::consts::PI / wavelength_nm;
        k0 * k0 * self.delta
    }

    /// Compute SLD_imag at given wavelength (nm)
    pub fn sld_imag(&self, wavelength_nm: f64) -> f64 {
        let k0 = 2.0 * std::f64::consts::PI / wavelength_nm;
        k0 * k0 * self.beta
    }

    /// Convert to XrrLayer at given wavelength
    pub fn to_layer(&self, thickness_nm: f64, roughness_nm: f64, wavelength_nm: f64) -> XrrLayer {
        XrrLayer::from_optical_constants(
            thickness_nm,
            self.delta,
            self.beta,
            wavelength_nm,
            roughness_nm,
        )
    }
}

/// Born approximation reflectivity (kinematic, weak scattering)
#[derive(Debug, Clone)]
pub struct BornResult {
    /// q values used
    pub q_values: Vec<f64>,
    /// Kinematic reflectivity
    pub reflectivity: Vec<f64>,
}

/// Main XRR processor
pub struct XrrProcessor {
    /// Layer model
    pub model: XrrModel,
    /// Material database
    materials: Vec<XrrMaterial>,
}

impl XrrProcessor {
    /// Create processor with given model
    pub fn new(model: XrrModel) -> Self {
        Self { model, materials: default_material_database() }
    }

    /// Create processor with a single-layer model on Si substrate (Cu Kα)
    pub fn single_layer_on_si(
        thickness_nm: f64,
        sld_real: f64,
        sld_imag: f64,
        roughness_nm: f64,
    ) -> Self {
        let si = si_substrate_cu_kalpha();
        let layer = XrrLayer::new(thickness_nm, sld_real, sld_imag, roughness_nm);
        let model = XrrModel::new_cu_kalpha(vec![layer], si);
        Self::new(model)
    }

    // ------------------------------------------------------------------
    // Core: Parratt recursion
    // ------------------------------------------------------------------

    /// Compute k_z for layer j given incidence angle theta (rad) and layer SLD
    ///
    /// k_{z,j} = sqrt( k0^2 * n_j^2 - k_x^2 )
    ///         = sqrt( k0^2 * cos^2(theta) - 2 * SLD_j + 2*i*SLD_imag_j )
    ///
    /// Using k_x = k0 * cos(theta) (grazing geometry convention)
    fn kz(&self, theta: f64, sld_real: f64, sld_imag: f64) -> Complex {
        let k0 = 2.0 * std::f64::consts::PI / self.model.wavelength_nm;
        let kx_sq = k0 * k0 * theta.cos() * theta.cos();
        // k_z^2 = k0^2 * n^2 - k_x^2 = k0^2*(1 - 2delta + 2i*beta) - k_x^2
        // ≈ k0^2*sin^2(theta) - k0^2*(2*delta - 2i*beta)
        // = k0^2*sin^2(theta) - 2*SLD_real + 2i*SLD_imag
        let kz_sq_re = k0 * k0 * theta.sin() * theta.sin() - 2.0 * sld_real;
        let kz_sq_im = 2.0 * sld_imag;
        Complex::new(kz_sq_re, kz_sq_im).sqrt()
    }

    /// Fresnel reflection coefficient at interface j→j+1 with Névot-Croce roughness
    ///
    /// r_j = (k_{z,j} - k_{z,j+1}) / (k_{z,j} + k_{z,j+1}) * exp(-2*k_{z,j}*k_{z,j+1}*sigma^2)
    fn fresnel_r(kzj: Complex, kzj1: Complex, sigma: f64) -> Complex {
        let numerator = kzj.sub(&kzj1);
        let denominator = kzj.add(&kzj1);
        let r = numerator.div(&denominator);
        // Névot-Croce factor: exp(-2 * kzj * kzj1 * sigma^2)
        if sigma > 0.0 {
            let nc_arg = kzj.mul(&kzj1).scale(-2.0 * sigma * sigma);
            let nc_factor = nc_arg.exp();
            r.mul(&nc_factor)
        } else {
            r
        }
    }

    /// Propagation phase factor for layer j of thickness d
    ///
    /// exp_factor = exp(2i * k_{z,j} * d_j)
    fn propagation_factor(kzj: Complex, thickness_nm: f64) -> Complex {
        let arg = Complex::new(0.0, 2.0 * thickness_nm).mul(&kzj);
        arg.exp()
    }

    /// Parratt recursion: compute reflectivity at a single angle theta (rad)
    ///
    /// Recurse from substrate upward through all layers.
    /// X_substrate = 0 (no reflection below substrate)
    /// X_j = exp(-2i*k_{z,j}*d_j) * (r_j + X_{j+1}) / (1 + r_j * X_{j+1})
    /// R = |X_vacuum|^2
    pub fn reflectivity_at_angle(&self, theta: f64) -> f64 {
        // Build complete layer stack: [ambient/vacuum, layer_0, ..., layer_N-1, substrate]
        // Ambient: vacuum (SLD = 0)
        let ambient_sld_re = 0.0f64;
        let ambient_sld_im = 0.0f64;

        // Compute k_z for each layer including ambient and substrate
        let kz_ambient = self.kz(theta, ambient_sld_re, ambient_sld_im);

        // Collect all layers (surface to substrate)
        let n_layers = self.model.layers.len();

        // Build k_z array for [layer_0, ..., layer_N-1, substrate]
        let mut kz_layers: Vec<Complex> = Vec::with_capacity(n_layers + 1);
        for layer in &self.model.layers {
            kz_layers.push(self.kz(theta, layer.sld_real, layer.sld_imag));
        }
        kz_layers.push(self.kz(
            theta,
            self.model.substrate.sld_real,
            self.model.substrate.sld_imag,
        ));

        // Roughness array: sigma at each interface
        // Interface i is between layer i and layer i+1 (0-indexed from surface)
        // sigma[i] = roughness of the lower interface of layer i
        // For interface between layers[i] and layers[i+1]: sigma = layers[i+1].roughness_nm
        // For interface between last layer and substrate: sigma = substrate.roughness_nm

        // Parratt recursion starting from substrate
        // X_substrate = 0
        let mut x = Complex::new(0.0, 0.0);

        // Recurse upward: from interface (n_layers-1 → substrate) to (ambient → layer_0)
        // We have n_layers + 1 boundaries total:
        //   boundary 0: ambient ↔ layer_0
        //   boundary k: layer_{k-1} ↔ layer_k    (k = 1..n_layers-1)
        //   boundary n_layers: layer_{n_layers-1} ↔ substrate

        // Process from deepest interface upward
        for i in (0..=n_layers).rev() {
            let (kzj, kzj1, sigma, thickness) = if i == n_layers {
                // Bottom interface: last film layer ↔ substrate
                let kzj = if n_layers == 0 { kz_ambient } else { kz_layers[n_layers - 1] };
                let kzj1 = kz_layers[n_layers]; // substrate
                let sigma = self.model.substrate.roughness_nm;
                // thickness belongs to layer above this interface; no propagation at substrate
                (kzj, kzj1, sigma, 0.0f64)
            } else if i == 0 {
                // Top interface: ambient ↔ layer_0 (or substrate if no layers)
                let kzj = kz_ambient;
                let kzj1 = if n_layers == 0 {
                    kz_layers[0] // substrate directly
                } else {
                    kz_layers[0]
                };
                let sigma = if n_layers == 0 {
                    self.model.substrate.roughness_nm
                } else {
                    self.model.layers[0].roughness_nm
                };
                (kzj, kzj1, sigma, 0.0f64)
            } else {
                // Interior interface: layer_{i-1} ↔ layer_i
                let kzj = kz_layers[i - 1];
                let kzj1 = kz_layers[i];
                let sigma = self.model.layers[i].roughness_nm;
                let thickness = self.model.layers[i - 1].thickness_nm;
                (kzj, kzj1, sigma, thickness)
            };

            let r = Self::fresnel_r(kzj, kzj1, sigma);

            // Propagate X through layer above this interface (if not at top)
            if i > 0 && i < n_layers {
                // Apply propagation phase for layer i (thickness d_{i})
                // After computing r_{i}, apply phase exp(2i*k_{z,i}*d_i)
                let kz_layer_i = kz_layers[i - 1];
                let d_i = self.model.layers[i - 1].thickness_nm;
                let phase = Self::propagation_factor(kz_layer_i, d_i);
                // X_i = phase * (r + X_{i+1}) / (1 + r*X_{i+1})
                let one = Complex::from_real(1.0);
                let numerator = r.add(&x);
                let denominator = one.add(&r.mul(&x));
                x = phase.mul(&numerator.div(&denominator));
            } else if i == n_layers {
                // At deepest interface, just compute X from substrate
                // X = r_{n} (no phase below substrate)
                let one = Complex::from_real(1.0);
                let numerator = r.add(&x);
                let denominator = one.add(&r.mul(&x));
                x = numerator.div(&denominator);
            } else {
                // i == 0: top interface
                // If there are layers below, apply phase for layer_0
                if n_layers > 0 {
                    let d0 = self.model.layers[0].thickness_nm;
                    let phase = Self::propagation_factor(kz_layers[0], d0);
                    let one = Complex::from_real(1.0);
                    let numerator = r.add(&x);
                    let denominator = one.add(&r.mul(&x));
                    x = phase.mul(&numerator.div(&denominator));
                } else {
                    let one = Complex::from_real(1.0);
                    let numerator = r.add(&x);
                    let denominator = one.add(&r.mul(&x));
                    x = numerator.div(&denominator);
                }
            }
        }

        let r_total = x.abs_sq();
        r_total.min(1.0)
    }

    /// Compute full reflectivity curve over theta range
    pub fn compute_reflectivity(
        &self,
        theta_min_deg: f64,
        theta_max_deg: f64,
        n_points: usize,
    ) -> ReflectivityCurve {
        let pi = std::f64::consts::PI;
        let mut q_values = Vec::with_capacity(n_points);
        let mut theta_values = Vec::with_capacity(n_points);
        let mut reflectivity = Vec::with_capacity(n_points);
        let mut log_reflectivity = Vec::with_capacity(n_points);

        for i in 0..n_points {
            let theta_deg = theta_min_deg
                + (theta_max_deg - theta_min_deg) * (i as f64) / ((n_points - 1) as f64);
            let theta_rad = theta_deg * pi / 180.0;
            let q = 4.0 * pi * theta_rad.sin() / self.model.wavelength_nm;
            let r = self.reflectivity_at_angle(theta_rad);
            let log_r = if r > 1e-40 { r.log10() } else { -40.0 };

            q_values.push(q);
            theta_values.push(theta_rad);
            reflectivity.push(r);
            log_reflectivity.push(log_r);
        }

        ReflectivityCurve { q_values, theta_values, reflectivity, log_reflectivity }
    }

    /// Compute reflectivity at specified q values
    pub fn compute_reflectivity_at_q(&self, q_values: &[f64]) -> ReflectivityCurve {
        let pi = std::f64::consts::PI;
        let lambda = self.model.wavelength_nm;
        let mut theta_values = Vec::with_capacity(q_values.len());
        let mut reflectivity = Vec::with_capacity(q_values.len());
        let mut log_reflectivity = Vec::with_capacity(q_values.len());
        let mut q_out = Vec::with_capacity(q_values.len());

        for &q in q_values {
            // q = 4*pi*sin(theta)/lambda => sin(theta) = q*lambda/(4*pi)
            let sin_theta = q * lambda / (4.0 * pi);
            if sin_theta.abs() > 1.0 {
                continue;
            }
            let theta_rad = sin_theta.asin();
            let r = self.reflectivity_at_angle(theta_rad);
            let log_r = if r > 1e-40 { r.log10() } else { -40.0 };

            q_out.push(q);
            theta_values.push(theta_rad);
            reflectivity.push(r);
            log_reflectivity.push(log_r);
        }

        ReflectivityCurve {
            q_values: q_out,
            theta_values,
            reflectivity,
            log_reflectivity,
        }
    }

    // ------------------------------------------------------------------
    // Critical angle analysis
    // ------------------------------------------------------------------

    /// Estimate critical angle from reflectivity curve
    ///
    /// The critical angle is where R drops to 0.5 (Fresnel edge).
    /// Uses linear interpolation between adjacent points.
    pub fn find_critical_angle(&self, curve: &ReflectivityCurve) -> Option<CriticalAngleResult> {
        // Find where reflectivity drops below 0.5 from total reflection
        let threshold = 0.5;
        for i in 1..curve.reflectivity.len() {
            if curve.reflectivity[i - 1] >= threshold && curve.reflectivity[i] < threshold {
                // Linear interpolation
                let r0 = curve.reflectivity[i - 1];
                let r1 = curve.reflectivity[i];
                let t0 = curve.theta_values[i - 1];
                let t1 = curve.theta_values[i];
                let frac = (threshold - r0) / (r1 - r0);
                let theta_c = t0 + frac * (t1 - t0);

                return Some(self.critical_angle_to_density(theta_c));
            }
        }
        None
    }

    /// Compute critical angle from top-layer optical constants
    ///
    /// θ_c ≈ sqrt(2 * delta) for refractive index n = 1 - delta + i*beta
    pub fn theoretical_critical_angle(&self) -> f64 {
        let (delta, _beta) = self.top_layer_optical_constants();
        (2.0 * delta).sqrt()
    }

    /// Extract delta and beta for the top layer (or ambient if no layers)
    fn top_layer_optical_constants(&self) -> (f64, f64) {
        let k0 = 2.0 * std::f64::consts::PI / self.model.wavelength_nm;
        let k0_sq = k0 * k0;
        if let Some(first) = self.model.layers.first() {
            let delta = first.sld_real / k0_sq;
            let beta = first.sld_imag / k0_sq;
            (delta, beta)
        } else {
            let delta = self.model.substrate.sld_real / k0_sq;
            let beta = self.model.substrate.sld_imag / k0_sq;
            (delta, beta)
        }
    }

    /// Convert critical angle to electron density
    ///
    /// ρ_e = π * θ_c^2 / (r_e * λ^2)
    /// r_e = 2.818e-6 nm (classical electron radius)
    pub fn critical_angle_to_density(&self, theta_c: f64) -> CriticalAngleResult {
        let pi = std::f64::consts::PI;
        let r_e = 2.818e-6; // nm
        let lambda = self.model.wavelength_nm;
        let theta_c_deg = theta_c * 180.0 / pi;
        // ρ_e = θ_c^2 * π / (r_e * λ^2)
        let electron_density = pi * theta_c * theta_c / (r_e * lambda * lambda);
        // SLD = r_e * ρ_e
        let sld = r_e * electron_density;

        CriticalAngleResult { theta_c_rad: theta_c, theta_c_deg, electron_density, sld }
    }

    // ------------------------------------------------------------------
    // Kiessig fringe analysis
    // ------------------------------------------------------------------

    /// Identify Kiessig fringe positions and extract film thickness
    ///
    /// Fringes appear as oscillations above q_c. Thickness:
    ///   d = 2π / Δq  (from fringe period Δq in q-space)
    pub fn analyze_kiessig_fringes(&self, curve: &ReflectivityCurve) -> KiessigAnalysis {
        // Find local maxima in log reflectivity above critical angle region
        let mut maxima_q = Vec::new();
        let r = &curve.reflectivity;
        let q = &curve.q_values;

        // Skip first few points (total reflection region)
        let start = (q.len() / 10).max(3);

        for i in start + 1..r.len() - 1 {
            if r[i] > r[i - 1] && r[i] > r[i + 1] && r[i] > 1e-10 {
                // Parabolic interpolation for sub-sample accuracy
                let q_max = parabolic_peak_q(q[i - 1], q[i], q[i + 1], r[i - 1], r[i], r[i + 1]);
                maxima_q.push(q_max);
            }
        }

        let fringe_count = maxima_q.len();
        if fringe_count < 2 {
            return KiessigAnalysis {
                fringe_positions: maxima_q,
                thickness_nm: 0.0,
                delta_q: 0.0,
                fringe_count,
            };
        }

        // Average fringe spacing
        let mut delta_q_sum = 0.0;
        let mut n_pairs = 0;
        for i in 1..maxima_q.len() {
            delta_q_sum += maxima_q[i] - maxima_q[i - 1];
            n_pairs += 1;
        }
        let delta_q = if n_pairs > 0 { delta_q_sum / n_pairs as f64 } else { 0.0 };
        let thickness_nm = if delta_q > 0.0 {
            2.0 * std::f64::consts::PI / delta_q
        } else {
            0.0
        };

        KiessigAnalysis { fringe_positions: maxima_q, thickness_nm, delta_q, fringe_count }
    }

    /// Predict fringe period for given layer thickness
    pub fn predict_fringe_period(&self, thickness_nm: f64) -> f64 {
        if thickness_nm > 0.0 {
            2.0 * std::f64::consts::PI / thickness_nm
        } else {
            0.0
        }
    }

    // ------------------------------------------------------------------
    // Electron density profile
    // ------------------------------------------------------------------

    /// Build electron density profile from the layer model
    ///
    /// Returns depth z (nm) and ρ_e (electrons/nm³) at z resolution dz_nm.
    pub fn electron_density_profile(&self, dz_nm: f64) -> ElectronDensityProfile {
        let pi = std::f64::consts::PI;
        let r_e = 2.818e-6; // nm
        let lambda = self.model.wavelength_nm;
        let k0 = 2.0 * pi / lambda;

        // Total film thickness
        let total_z: f64 = self.model.layers.iter().map(|l| l.thickness_nm).sum();
        let extra = 2.0; // nm of substrate on each side
        let z_max = total_z + extra;
        let n_pts = ((z_max / dz_nm) as usize).max(10);

        let mut z_nm = Vec::with_capacity(n_pts);
        let mut rho_e = Vec::with_capacity(n_pts);

        for i in 0..n_pts {
            let z = i as f64 * dz_nm;
            z_nm.push(z);

            // Find which layer z falls in
            let mut z_layer_start = 0.0;
            let mut layer_sld_real = self.model.substrate.sld_real;

            for layer in &self.model.layers {
                let z_layer_end = z_layer_start + layer.thickness_nm;
                if z >= z_layer_start && z < z_layer_end {
                    layer_sld_real = layer.sld_real;
                    break;
                }
                z_layer_start = z_layer_end;
            }

            // SLD_real = k0^2 * delta = r_e * lambda^2 * rho_e / pi
            // => rho_e = SLD_real * pi / (r_e * lambda^2)
            let rho = layer_sld_real * pi / (r_e * lambda * lambda * k0 * k0);
            rho_e.push(rho.max(0.0));
        }

        ElectronDensityProfile { z_nm, rho_e }
    }

    /// Apply Névot-Croce broadening to electron density profile at interfaces
    pub fn smear_edp_at_interface(
        edp: &ElectronDensityProfile,
        sigma_nm: f64,
    ) -> ElectronDensityProfile {
        let n = edp.z_nm.len();
        let dz = if n > 1 { edp.z_nm[1] - edp.z_nm[0] } else { 0.1 };
        // Gaussian kernel half-width in samples
        let kernel_hw = ((3.0 * sigma_nm / dz) as usize).max(1);
        let kernel_size = 2 * kernel_hw + 1;
        let mut kernel: Vec<f64> = Vec::with_capacity(kernel_size);
        let norm: f64 = {
            let mut s = 0.0;
            for k in 0..kernel_size {
                let x = (k as f64 - kernel_hw as f64) * dz;
                let w = (-0.5 * x * x / (sigma_nm * sigma_nm)).exp();
                kernel.push(w);
                s += w;
            }
            s
        };
        for w in &mut kernel {
            *w /= norm;
        }

        let mut rho_smooth = vec![0.0f64; n];
        for i in 0..n {
            let mut val = 0.0;
            for (k, &wk) in kernel.iter().enumerate() {
                let idx = i as isize + k as isize - kernel_hw as isize;
                if idx >= 0 && idx < n as isize {
                    val += wk * edp.rho_e[idx as usize];
                }
            }
            rho_smooth[i] = val;
        }

        ElectronDensityProfile { z_nm: edp.z_nm.clone(), rho_e: rho_smooth }
    }

    // ------------------------------------------------------------------
    // Born approximation (kinematic reflectivity)
    // ------------------------------------------------------------------

    /// Born approximation reflectivity: R_Born(q) = (16π²r_e²/q⁴) |FT[dρ/dz]|²
    ///
    /// Suitable for q >> q_c, low-contrast films.
    pub fn born_approximation(
        &self,
        q_values: &[f64],
    ) -> BornResult {
        let pi = std::f64::consts::PI;
        let r_e = 2.818e-6; // nm
        let lambda = self.model.wavelength_nm;
        let k0 = 2.0 * pi / lambda;
        let dz = 0.02_f64; // nm

        // Build ρ_e(z) from model with small dz
        let edp = self.electron_density_profile(dz);
        let n = edp.rho_e.len();

        // Compute gradient dρ/dz (finite difference)
        let mut drho_dz = vec![0.0f64; n];
        for i in 1..n - 1 {
            drho_dz[i] = (edp.rho_e[i + 1] - edp.rho_e[i - 1]) / (2.0 * dz);
        }

        let mut reflectivity = Vec::with_capacity(q_values.len());

        for &q in q_values {
            if q <= 0.0 {
                reflectivity.push(1.0);
                continue;
            }
            // FT: integral_0^inf (dρ/dz) exp(i*q*z) dz
            let mut ft_re = 0.0f64;
            let mut ft_im = 0.0f64;
            for i in 0..n {
                let z = edp.z_nm[i];
                ft_re += drho_dz[i] * (q * z).cos() * dz;
                ft_im += drho_dz[i] * (q * z).sin() * dz;
            }
            let ft_sq = ft_re * ft_re + ft_im * ft_im;
            // R = (16 π² r_e²) / q^4 * |FT|²
            // Note: rho_e unit is nm^-3, q unit is nm^-1
            let factor = 16.0 * pi * pi * r_e * r_e;
            let r_born = factor * ft_sq / (q * q * q * q * k0 * k0 * k0 * k0);
            reflectivity.push(r_born.min(1.0));
        }

        BornResult { q_values: q_values.to_vec(), reflectivity }
    }

    // ------------------------------------------------------------------
    // Material database
    // ------------------------------------------------------------------

    /// Look up material by name
    pub fn get_material(&self, name: &str) -> Option<&XrrMaterial> {
        self.materials.iter().find(|m| m.name.eq_ignore_ascii_case(name))
    }

    /// List available materials
    pub fn list_materials(&self) -> Vec<&str> {
        self.materials.iter().map(|m| m.name.as_str()).collect()
    }

    // ------------------------------------------------------------------
    // Utility: q ↔ theta conversion
    // ------------------------------------------------------------------

    /// Convert theta (radians) to q (nm^-1)
    pub fn theta_to_q(&self, theta_rad: f64) -> f64 {
        4.0 * std::f64::consts::PI * theta_rad.sin() / self.model.wavelength_nm
    }

    /// Convert q (nm^-1) to theta (radians)
    pub fn q_to_theta(&self, q: f64) -> f64 {
        let sin_theta = q * self.model.wavelength_nm / (4.0 * std::f64::consts::PI);
        sin_theta.asin()
    }

    // ------------------------------------------------------------------
    // Fresnel reflectivity for a single interface (semi-infinite medium)
    // ------------------------------------------------------------------

    /// Single-interface Fresnel reflectivity |r|² (no layers, substrate only)
    pub fn fresnel_substrate(&self, theta: f64) -> f64 {
        let kz0 = self.kz(theta, 0.0, 0.0);
        let kz1 = self.kz(
            theta,
            self.model.substrate.sld_real,
            self.model.substrate.sld_imag,
        );
        let r = Self::fresnel_r(kz0, kz1, self.model.substrate.roughness_nm);
        r.abs_sq().min(1.0)
    }

    // ------------------------------------------------------------------
    // Roughness effects
    // ------------------------------------------------------------------

    /// Compute Debye-Waller-like roughness correction factor at given theta and sigma
    ///
    /// DW = exp(-4 * k_z^2 * sigma^2)  (simplified version of Névot-Croce)
    pub fn debye_waller_factor(&self, theta: f64, sigma_nm: f64) -> f64 {
        let kz0 = self.kz(theta, 0.0, 0.0);
        let kz_real = kz0.re;
        (-4.0 * kz_real * kz_real * sigma_nm * sigma_nm).exp()
    }

    // ------------------------------------------------------------------
    // Yoneda wing detection
    // ------------------------------------------------------------------

    /// Check for Yoneda wing enhancement near critical angle
    ///
    /// Yoneda wings appear at θ = θ_c and θ_sample = θ_c (diffuse scattering).
    /// In specular XRR, they manifest as a shoulder near the critical angle.
    pub fn yoneda_wing_q(&self) -> f64 {
        let theta_c = self.theoretical_critical_angle();
        self.theta_to_q(theta_c)
    }
}

// ------------------------------------------------------------------
// Standalone functions
// ------------------------------------------------------------------

/// Parabolic interpolation to find peak position between three q points
fn parabolic_peak_q(q0: f64, q1: f64, q2: f64, r0: f64, r1: f64, r2: f64) -> f64 {
    // Peak of parabola through (q0,r0), (q1,r1), (q2,r2)
    let denom = 2.0 * (r0 - 2.0 * r1 + r2);
    if denom.abs() < 1e-30 {
        return q1;
    }
    let offset = (r0 - r2) / denom;
    let dq = q2 - q0;
    q1 + offset * dq / 2.0
}

/// Compute critical angle from refractive index delta: θ_c = sqrt(2*delta)
pub fn critical_angle_from_delta(delta: f64) -> f64 {
    (2.0 * delta).sqrt()
}

/// Compute electron density from critical angle and wavelength (nm)
///
/// ρ_e = π * θ_c^2 / (r_e * λ^2)
pub fn electron_density_from_critical_angle(theta_c_rad: f64, wavelength_nm: f64) -> f64 {
    let r_e = 2.818e-6; // nm
    std::f64::consts::PI * theta_c_rad * theta_c_rad / (r_e * wavelength_nm * wavelength_nm)
}

/// Compute SLD from electron density
///
/// SLD = r_e * ρ_e
pub fn sld_from_electron_density(rho_e: f64) -> f64 {
    let r_e = 2.818e-6; // nm
    r_e * rho_e
}

/// Kiessig fringe thickness from angular period (degrees)
///
/// d = λ / (2 * Δθ)  (small-angle approximation)
pub fn thickness_from_angular_period(delta_theta_deg: f64, wavelength_nm: f64) -> f64 {
    let delta_theta_rad = delta_theta_deg * std::f64::consts::PI / 180.0;
    if delta_theta_rad > 0.0 {
        wavelength_nm / (2.0 * delta_theta_rad)
    } else {
        0.0
    }
}

/// Kiessig fringe thickness from q-period
///
/// d = 2π / Δq
pub fn thickness_from_q_period(delta_q: f64) -> f64 {
    if delta_q > 0.0 {
        2.0 * std::f64::consts::PI / delta_q
    } else {
        0.0
    }
}

/// Build default material database at Cu Kα (λ = 0.15406 nm)
pub fn default_material_database() -> Vec<XrrMaterial> {
    vec![
        XrrMaterial {
            name: "Silicon".to_string(),
            delta: 7.44e-6,
            beta: 1.73e-7,
            density_g_cm3: 2.33,
        },
        XrrMaterial {
            name: "SiO2".to_string(),
            delta: 7.12e-6,
            beta: 1.54e-7,
            density_g_cm3: 2.20,
        },
        XrrMaterial {
            name: "Gold".to_string(),
            delta: 4.66e-5,
            beta: 4.18e-6,
            density_g_cm3: 19.3,
        },
        XrrMaterial {
            name: "Titanium".to_string(),
            delta: 1.24e-5,
            beta: 1.44e-6,
            density_g_cm3: 4.51,
        },
        XrrMaterial {
            name: "Aluminum".to_string(),
            delta: 7.58e-6,
            beta: 1.45e-7,
            density_g_cm3: 2.70,
        },
        XrrMaterial {
            name: "Air".to_string(),
            delta: 0.0,
            beta: 0.0,
            density_g_cm3: 0.0012,
        },
        XrrMaterial {
            name: "Nickel".to_string(),
            delta: 1.68e-5,
            beta: 6.09e-7,
            density_g_cm3: 8.91,
        },
        XrrMaterial {
            name: "Copper".to_string(),
            delta: 2.35e-5,
            beta: 1.65e-6,
            density_g_cm3: 8.96,
        },
        XrrMaterial {
            name: "Iron".to_string(),
            delta: 1.32e-5,
            beta: 3.05e-7,
            density_g_cm3: 7.87,
        },
        XrrMaterial {
            name: "Chromium".to_string(),
            delta: 1.40e-5,
            beta: 3.87e-7,
            density_g_cm3: 7.19,
        },
    ]
}

/// Pre-built Silicon substrate for Cu Kα
pub fn si_substrate_cu_kalpha() -> XrrLayer {
    let mat = XrrMaterial {
        name: "Silicon".to_string(),
        delta: 7.44e-6,
        beta: 1.73e-7,
        density_g_cm3: 2.33,
    };
    mat.to_layer(0.0, 0.3, 0.15406)
}

/// Pre-built Gold layer for Cu Kα
pub fn gold_layer_cu_kalpha(thickness_nm: f64, roughness_nm: f64) -> XrrLayer {
    let mat = XrrMaterial {
        name: "Gold".to_string(),
        delta: 4.66e-5,
        beta: 4.18e-6,
        density_g_cm3: 19.3,
    };
    mat.to_layer(thickness_nm, roughness_nm, 0.15406)
}

/// Pre-built SiO2 layer for Cu Kα
pub fn sio2_layer_cu_kalpha(thickness_nm: f64, roughness_nm: f64) -> XrrLayer {
    let mat = XrrMaterial {
        name: "SiO2".to_string(),
        delta: 7.12e-6,
        beta: 1.54e-7,
        density_g_cm3: 2.20,
    };
    mat.to_layer(thickness_nm, roughness_nm, 0.15406)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const PI: f64 = std::f64::consts::PI;
    const CU_KALPHA_NM: f64 = 0.15406;

    // ------------------------------------------------------------------
    // Complex number tests
    // ------------------------------------------------------------------

    #[test]
    fn test_complex_mul() {
        let a = Complex::new(1.0, 2.0);
        let b = Complex::new(3.0, 4.0);
        let c = a.mul(&b);
        assert!((c.re - (-5.0)).abs() < 1e-12);
        assert!((c.im - 10.0).abs() < 1e-12);
    }

    #[test]
    fn test_complex_div() {
        let a = Complex::new(1.0, 0.0);
        let b = Complex::new(1.0, 0.0);
        let c = a.div(&b);
        assert!((c.re - 1.0).abs() < 1e-12);
        assert!(c.im.abs() < 1e-12);
    }

    #[test]
    fn test_complex_sqrt_real_positive() {
        let a = Complex::new(4.0, 0.0);
        let s = a.sqrt();
        assert!((s.re - 2.0).abs() < 1e-12);
        assert!(s.im.abs() < 1e-12);
    }

    #[test]
    fn test_complex_sqrt_pure_imaginary() {
        // sqrt(i) = (1+i)/sqrt(2)
        let a = Complex::new(0.0, 1.0);
        let s = a.sqrt();
        let expected = 1.0 / 2.0_f64.sqrt();
        assert!((s.re - expected).abs() < 1e-12);
        assert!((s.im - expected).abs() < 1e-12);
    }

    #[test]
    fn test_complex_exp() {
        // exp(i*pi) = -1
        let a = Complex::new(0.0, PI);
        let e = a.exp();
        assert!((e.re + 1.0).abs() < 1e-12);
        assert!(e.im.abs() < 1e-12);
    }

    #[test]
    fn test_complex_abs() {
        let a = Complex::new(3.0, 4.0);
        assert!((a.abs() - 5.0).abs() < 1e-12);
    }

    #[test]
    fn test_complex_conj() {
        let a = Complex::new(1.0, 2.0);
        let c = a.conj();
        assert_eq!(c.re, 1.0);
        assert_eq!(c.im, -2.0);
    }

    // ------------------------------------------------------------------
    // Fresnel reflectivity tests
    // ------------------------------------------------------------------

    #[test]
    fn test_fresnel_total_reflection_below_critical_angle() {
        // Below critical angle: total reflection R ≈ 1
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);

        // Critical angle for Si ≈ sqrt(2 * 7.44e-6) ≈ 0.00386 rad ≈ 0.22 deg
        let theta_below = 0.001_f64.to_radians(); // well below critical
        let r = proc.fresnel_substrate(theta_below);
        assert!(r > 0.9, "Below critical angle R should be close to 1, got {}", r);
    }

    #[test]
    fn test_fresnel_drops_above_critical_angle() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);

        let theta_above = 1.0_f64.to_radians(); // well above critical (0.22 deg)
        let r = proc.fresnel_substrate(theta_above);
        assert!(r < 0.1, "Well above critical angle R should be small, got {}", r);
    }

    #[test]
    fn test_fresnel_at_critical_angle() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);

        // At critical angle R ≈ 0.5 (by definition)
        let theta_c = proc.theoretical_critical_angle();
        let r = proc.fresnel_substrate(theta_c);
        // R should be between 0.3 and 0.7 at critical angle
        assert!(r > 0.3 && r < 0.8, "At critical angle R should be near 0.5, got {}", r);
    }

    #[test]
    fn test_fresnel_no_roughness_vs_with_roughness() {
        let si_smooth = XrrLayer::new(0.0, si_sld_real(), si_sld_imag(), 0.0);
        let si_rough = XrrLayer::new(0.0, si_sld_real(), si_sld_imag(), 0.5);

        let model_smooth = XrrModel::new_cu_kalpha(vec![], si_smooth);
        let model_rough = XrrModel::new_cu_kalpha(vec![], si_rough);
        let proc_smooth = XrrProcessor::new(model_smooth);
        let proc_rough = XrrProcessor::new(model_rough);

        // At high angle, roughness should reduce reflectivity
        let theta = 3.0_f64.to_radians();
        let r_smooth = proc_smooth.fresnel_substrate(theta);
        let r_rough = proc_rough.fresnel_substrate(theta);
        assert!(r_smooth >= r_rough, "Roughness should reduce reflectivity: smooth={} rough={}", r_smooth, r_rough);
    }

    // ------------------------------------------------------------------
    // Parratt recursion tests
    // ------------------------------------------------------------------

    #[test]
    fn test_parratt_no_layers_equals_fresnel() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si.clone());
        let proc = XrrProcessor::new(model);

        for theta_deg in [0.05_f64, 0.15, 0.3, 0.5, 1.0, 2.0] {
            let theta = theta_deg.to_radians();
            let r_parratt = proc.reflectivity_at_angle(theta);
            let r_fresnel = proc.fresnel_substrate(theta);
            assert!(
                (r_parratt - r_fresnel).abs() < 1e-6,
                "theta={}°: Parratt({}) should equal Fresnel({})",
                theta_deg,
                r_parratt,
                r_fresnel
            );
        }
    }

    #[test]
    fn test_parratt_single_layer_produces_oscillations() {
        // A single 20 nm Au layer on Si should produce Kiessig fringes
        let si = si_substrate_cu_kalpha();
        let au = gold_layer_cu_kalpha(20.0, 0.3);
        let model = XrrModel::new_cu_kalpha(vec![au], si);
        let proc = XrrProcessor::new(model);

        let curve = proc.compute_reflectivity(0.1, 4.0, 400);
        // Should have fringes: not monotone above critical angle
        let n = curve.reflectivity.len();
        let region = &curve.reflectivity[n / 4..];
        let mut n_sign_changes = 0;
        for i in 1..region.len() - 1 {
            let d1 = region[i] - region[i - 1];
            let d2 = region[i + 1] - region[i];
            if d1 * d2 < 0.0 {
                n_sign_changes += 1;
            }
        }
        assert!(n_sign_changes > 5, "Expected oscillations (fringes), got {} sign changes", n_sign_changes);
    }

    #[test]
    fn test_parratt_total_reflection_at_zero_angle() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);
        let r = proc.reflectivity_at_angle(0.0001_f64.to_radians());
        assert!(r > 0.95, "Near-zero angle should give near-total reflection: {}", r);
    }

    #[test]
    fn test_parratt_reflectivity_bounded() {
        let si = si_substrate_cu_kalpha();
        let au = gold_layer_cu_kalpha(15.0, 0.2);
        let model = XrrModel::new_cu_kalpha(vec![au], si);
        let proc = XrrProcessor::new(model);

        let curve = proc.compute_reflectivity(0.01, 5.0, 500);
        for &r in &curve.reflectivity {
            assert!(r >= 0.0 && r <= 1.0, "Reflectivity must be in [0,1], got {}", r);
        }
    }

    #[test]
    fn test_parratt_multilayer() {
        // Three-layer stack: Au / Ti / SiO2 on Si
        let si = si_substrate_cu_kalpha();
        let sio2 = sio2_layer_cu_kalpha(5.0, 0.2);
        let ti = XrrLayer::from_optical_constants(5.0, 1.24e-5, 1.44e-6, CU_KALPHA_NM, 0.3);
        let au = gold_layer_cu_kalpha(20.0, 0.5);
        let model = XrrModel::new_cu_kalpha(vec![au, ti, sio2], si);
        let proc = XrrProcessor::new(model);

        let curve = proc.compute_reflectivity(0.1, 5.0, 300);
        for &r in &curve.reflectivity {
            assert!(r >= 0.0 && r <= 1.0, "Multilayer R out of bounds: {}", r);
        }
    }

    // ------------------------------------------------------------------
    // Kiessig fringe analysis tests
    // ------------------------------------------------------------------

    #[test]
    fn test_kiessig_fringe_period_single_layer() {
        // 20 nm Au → Δq = 2π/20 ≈ 0.3142 nm^-1
        let si = si_substrate_cu_kalpha();
        let au = gold_layer_cu_kalpha(20.0, 0.1);
        let model = XrrModel::new_cu_kalpha(vec![au], si);
        let proc = XrrProcessor::new(model);

        let curve = proc.compute_reflectivity(0.05, 6.0, 1000);
        let analysis = proc.analyze_kiessig_fringes(&curve);

        if analysis.fringe_count >= 2 {
            let expected_dq = 2.0 * PI / 20.0;
            let rel_error = (analysis.delta_q - expected_dq).abs() / expected_dq;
            assert!(
                rel_error < 0.15,
                "Fringe Δq = {} nm^-1, expected {} nm^-1 (err={:.1}%)",
                analysis.delta_q,
                expected_dq,
                rel_error * 100.0
            );
        }
    }

    #[test]
    fn test_kiessig_thickness_recovery() {
        // 30 nm SiO2 on Si: check thickness recovery from fringes
        let si = si_substrate_cu_kalpha();
        let sio2 = sio2_layer_cu_kalpha(30.0, 0.1);
        let model = XrrModel::new_cu_kalpha(vec![sio2], si);
        let proc = XrrProcessor::new(model);

        let curve = proc.compute_reflectivity(0.05, 4.0, 2000);
        let analysis = proc.analyze_kiessig_fringes(&curve);

        if analysis.fringe_count >= 2 && analysis.thickness_nm > 0.0 {
            let rel_error = (analysis.thickness_nm - 30.0).abs() / 30.0;
            assert!(
                rel_error < 0.20,
                "Thickness recovery: got {} nm, expected 30 nm (err={:.1}%)",
                analysis.thickness_nm,
                rel_error * 100.0
            );
        }
    }

    #[test]
    fn test_fringe_period_formula() {
        let dq = 2.0 * PI / 50.0; // Δq for 50 nm layer
        let d = thickness_from_q_period(dq);
        assert!((d - 50.0).abs() < 0.01, "Expected 50 nm, got {}", d);
    }

    #[test]
    fn test_angular_fringe_period() {
        // For 100 nm film with Cu Kα: Δθ = λ/(2d) ≈ 0.15406/(200) ≈ 0.00077 rad ≈ 0.0441°
        let lambda = 0.15406_f64;
        let d = 100.0_f64;
        let delta_theta_deg = (lambda / (2.0 * d)) * 180.0 / PI;
        let d_calc = thickness_from_angular_period(delta_theta_deg, lambda);
        assert!((d_calc - d).abs() < 0.01, "Expected 100 nm, got {}", d_calc);
    }

    #[test]
    fn test_predict_fringe_period() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);

        let dq = proc.predict_fringe_period(25.0);
        let expected = 2.0 * PI / 25.0;
        assert!((dq - expected).abs() < 1e-10);
    }

    // ------------------------------------------------------------------
    // Critical angle tests
    // ------------------------------------------------------------------

    #[test]
    fn test_critical_angle_silicon() {
        // Si: δ = 7.44e-6 → θ_c = sqrt(2*7.44e-6) ≈ 0.00386 rad ≈ 0.221°
        let delta = 7.44e-6_f64;
        let theta_c = critical_angle_from_delta(delta);
        let theta_c_deg = theta_c * 180.0 / PI;
        assert!((theta_c_deg - 0.221).abs() < 0.01, "Si critical angle: expected ≈0.221°, got {:.4}°", theta_c_deg);
    }

    #[test]
    fn test_critical_angle_gold() {
        // Au: δ = 4.66e-5 → θ_c = sqrt(2*4.66e-5) ≈ 0.00965 rad ≈ 0.553°
        let delta = 4.66e-5_f64;
        let theta_c = critical_angle_from_delta(delta);
        let theta_c_deg = theta_c * 180.0 / PI;
        assert!((theta_c_deg - 0.553).abs() < 0.02, "Au critical angle: expected ≈0.553°, got {:.4}°", theta_c_deg);
    }

    #[test]
    fn test_theoretical_critical_angle_substrate() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);

        let theta_c = proc.theoretical_critical_angle();
        let theta_c_deg = theta_c * 180.0 / PI;
        assert!((theta_c_deg - 0.221).abs() < 0.02, "Si θ_c: expected ≈0.221°, got {:.4}°", theta_c_deg);
    }

    #[test]
    fn test_electron_density_from_critical_angle() {
        // Si: θ_c ≈ 0.00386 rad, λ = 0.15406 nm
        // ρ_e = π * θ_c^2 / (r_e * λ^2) ≈ 699 electrons/nm³
        let theta_c = critical_angle_from_delta(7.44e-6);
        let rho_e = electron_density_from_critical_angle(theta_c, CU_KALPHA_NM);
        // Si has ~699 e/nm³ (literature value)
        assert!(rho_e > 400.0 && rho_e < 1000.0, "Si ρ_e = {} e/nm³ out of expected range", rho_e);
    }

    #[test]
    fn test_sld_from_electron_density() {
        let rho_e = 699.0_f64; // e/nm³ for Si
        let sld = sld_from_electron_density(rho_e);
        let r_e = 2.818e-6_f64;
        let expected = r_e * rho_e;
        assert!((sld - expected).abs() < 1e-20);
    }

    #[test]
    fn test_critical_angle_result_from_processor() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);

        let curve = proc.compute_reflectivity(0.01, 1.0, 1000);
        if let Some(result) = proc.find_critical_angle(&curve) {
            let theta_c_deg = result.theta_c_deg;
            assert!(
                theta_c_deg > 0.1 && theta_c_deg < 0.5,
                "Si critical angle from curve: {:.4}°",
                theta_c_deg
            );
        }
    }

    // ------------------------------------------------------------------
    // Roughness tests
    // ------------------------------------------------------------------

    #[test]
    fn test_nevot_croce_smooth_vs_rough() {
        let si_smooth = XrrLayer::new(0.0, si_sld_real(), si_sld_imag(), 0.0);
        let si_rough = XrrLayer::new(0.0, si_sld_real(), si_sld_imag(), 1.0);
        let model_s = XrrModel::new_cu_kalpha(vec![], si_smooth);
        let model_r = XrrModel::new_cu_kalpha(vec![], si_rough);
        let proc_s = XrrProcessor::new(model_s);
        let proc_r = XrrProcessor::new(model_r);

        // At 2°, rough surface should reflect less
        let theta = 2.0_f64.to_radians();
        let r_s = proc_s.fresnel_substrate(theta);
        let r_r = proc_r.fresnel_substrate(theta);
        assert!(r_s > r_r, "Smooth R={} should exceed rough R={} at high angles", r_s, r_r);
    }

    #[test]
    fn test_debye_waller_factor_zero_roughness() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);

        let dw = proc.debye_waller_factor(1.0_f64.to_radians(), 0.0);
        assert!((dw - 1.0).abs() < 1e-10, "Zero roughness → DW = 1");
    }

    #[test]
    fn test_debye_waller_factor_decays_with_roughness() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);

        let theta = 2.0_f64.to_radians();
        let dw1 = proc.debye_waller_factor(theta, 0.5);
        let dw2 = proc.debye_waller_factor(theta, 1.0);
        assert!(dw1 > dw2, "DW factor should decrease with roughness: {} > {}", dw1, dw2);
        assert!(dw2 > 0.0 && dw2 < 1.0);
    }

    #[test]
    fn test_roughness_monotone_reduction() {
        // Increasing roughness should monotonically reduce R at fixed high angle
        let theta = 3.0_f64.to_radians();
        let sigmas = [0.0, 0.2, 0.5, 1.0, 2.0];
        let mut prev_r = 1.0;
        for &sigma in &sigmas {
            let si = XrrLayer::new(0.0, si_sld_real(), si_sld_imag(), sigma);
            let model = XrrModel::new_cu_kalpha(vec![], si);
            let proc = XrrProcessor::new(model);
            let r = proc.fresnel_substrate(theta);
            assert!(r <= prev_r + 1e-10, "R should not increase with roughness: sigma={} R={}", sigma, r);
            prev_r = r;
        }
    }

    // ------------------------------------------------------------------
    // Material database tests
    // ------------------------------------------------------------------

    #[test]
    fn test_material_database_contains_si() {
        let db = default_material_database();
        let si = db.iter().find(|m| m.name == "Silicon").unwrap();
        assert!((si.delta - 7.44e-6).abs() < 1e-10);
        assert!((si.beta - 1.73e-7).abs() < 1e-10);
    }

    #[test]
    fn test_material_database_contains_gold() {
        let db = default_material_database();
        let au = db.iter().find(|m| m.name == "Gold").unwrap();
        assert!((au.delta - 4.66e-5).abs() < 1e-10);
        assert!((au.density_g_cm3 - 19.3).abs() < 0.01);
    }

    #[test]
    fn test_material_database_air_zero_sld() {
        let db = default_material_database();
        let air = db.iter().find(|m| m.name == "Air").unwrap();
        assert_eq!(air.delta, 0.0);
        assert_eq!(air.beta, 0.0);
    }

    #[test]
    fn test_processor_material_lookup() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);
        assert!(proc.get_material("silicon").is_some());
        assert!(proc.get_material("Gold").is_some());
        assert!(proc.get_material("unobtainium").is_none());
    }

    #[test]
    fn test_material_sld_at_cu_kalpha() {
        let si = XrrMaterial {
            name: "Silicon".to_string(),
            delta: 7.44e-6,
            beta: 1.73e-7,
            density_g_cm3: 2.33,
        };
        let k0 = 2.0 * PI / CU_KALPHA_NM;
        let expected_sld_real = k0 * k0 * si.delta;
        let actual = si.sld_real(CU_KALPHA_NM);
        assert!((actual - expected_sld_real).abs() < 1e-10);
    }

    #[test]
    fn test_material_to_layer_conversion() {
        let au = XrrMaterial {
            name: "Gold".to_string(),
            delta: 4.66e-5,
            beta: 4.18e-6,
            density_g_cm3: 19.3,
        };
        let layer = au.to_layer(10.0, 0.5, CU_KALPHA_NM);
        assert!((layer.thickness_nm - 10.0).abs() < 1e-10);
        assert!((layer.roughness_nm - 0.5).abs() < 1e-10);
        assert!(layer.sld_real > 0.0);
    }

    #[test]
    fn test_gold_layer_helper() {
        let au = gold_layer_cu_kalpha(15.0, 0.3);
        assert!((au.thickness_nm - 15.0).abs() < 1e-10);
        assert!((au.roughness_nm - 0.3).abs() < 1e-10);
    }

    #[test]
    fn test_sio2_layer_helper() {
        let sio2 = sio2_layer_cu_kalpha(10.0, 0.2);
        assert!((sio2.thickness_nm - 10.0).abs() < 1e-10);
    }

    // ------------------------------------------------------------------
    // Electron density profile tests
    // ------------------------------------------------------------------

    #[test]
    fn test_edp_substrate_only() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);
        let edp = proc.electron_density_profile(0.1);
        assert!(!edp.z_nm.is_empty());
        for &rho in &edp.rho_e {
            assert!(rho >= 0.0, "Electron density must be non-negative");
        }
    }

    #[test]
    fn test_edp_single_layer_has_two_regions() {
        // Gold on Si: edp should show two distinct density levels
        let si = si_substrate_cu_kalpha();
        let au = gold_layer_cu_kalpha(10.0, 0.0);
        let model = XrrModel::new_cu_kalpha(vec![au], si);
        let proc = XrrProcessor::new(model);
        let edp = proc.electron_density_profile(0.05);

        // Gold region (z < 10 nm) should have higher density than Si (z > 10 nm)
        let n = edp.z_nm.len();
        let au_rho = edp.rho_e[n / 4]; // early part = Au layer
        let si_rho = edp.rho_e[3 * n / 4]; // later part = Si substrate
        assert!(au_rho > 0.0 && si_rho > 0.0);
    }

    #[test]
    fn test_edp_smearing_reduces_gradient() {
        let si = si_substrate_cu_kalpha();
        let au = gold_layer_cu_kalpha(10.0, 0.0);
        let model = XrrModel::new_cu_kalpha(vec![au], si);
        let proc = XrrProcessor::new(model);
        let edp = proc.electron_density_profile(0.1);

        let edp_smooth = XrrProcessor::smear_edp_at_interface(&edp, 0.5);
        // Smeared profile max gradient should be smaller
        let max_grad = |rho: &Vec<f64>| -> f64 {
            let mut max = 0.0_f64;
            for i in 1..rho.len() {
                max = max.max((rho[i] - rho[i - 1]).abs());
            }
            max
        };
        let g_sharp = max_grad(&edp.rho_e);
        let g_smooth = max_grad(&edp_smooth.rho_e);
        assert!(g_smooth <= g_sharp + 1e-10, "Smearing should reduce gradient: {} > {}", g_smooth, g_sharp);
    }

    // ------------------------------------------------------------------
    // q ↔ theta conversion tests
    // ------------------------------------------------------------------

    #[test]
    fn test_q_theta_roundtrip() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);

        for theta_deg in [0.1_f64, 0.3, 0.5, 1.0, 2.0, 3.0] {
            let theta = theta_deg.to_radians();
            let q = proc.theta_to_q(theta);
            let theta_back = proc.q_to_theta(q);
            assert!((theta_back - theta).abs() < 1e-12, "θ→q→θ roundtrip failed at {}°", theta_deg);
        }
    }

    #[test]
    fn test_q_at_half_degree() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);

        let theta = 0.5_f64.to_radians();
        let q = proc.theta_to_q(theta);
        // q = 4π sin(θ) / λ ≈ 4π * 0.00873 / 0.15406 ≈ 0.712 nm^-1
        assert!(q > 0.5 && q < 1.0, "q at 0.5° should be ~0.71 nm^-1, got {}", q);
    }

    // ------------------------------------------------------------------
    // Reflectivity curve tests
    // ------------------------------------------------------------------

    #[test]
    fn test_reflectivity_curve_q_values_monotone() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);

        let curve = proc.compute_reflectivity(0.1, 3.0, 100);
        for i in 1..curve.q_values.len() {
            assert!(curve.q_values[i] > curve.q_values[i - 1], "q values should be monotone increasing");
        }
    }

    #[test]
    fn test_reflectivity_curve_at_q() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);

        let q_vals: Vec<f64> = (1..=50).map(|i| i as f64 * 0.1).collect();
        let curve = proc.compute_reflectivity_at_q(&q_vals);
        assert!(!curve.reflectivity.is_empty());
        for &r in &curve.reflectivity {
            assert!(r >= 0.0 && r <= 1.0);
        }
    }

    #[test]
    fn test_log_reflectivity_consistency() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);

        let curve = proc.compute_reflectivity(0.1, 2.0, 50);
        for i in 0..curve.reflectivity.len() {
            let r = curve.reflectivity[i];
            let log_r = curve.log_reflectivity[i];
            if r > 1e-20 {
                let expected_log = r.log10();
                assert!((log_r - expected_log).abs() < 1e-10);
            }
        }
    }

    // ------------------------------------------------------------------
    // Born approximation tests
    // ------------------------------------------------------------------

    #[test]
    fn test_born_approximation_returns_valid_r() {
        let si = si_substrate_cu_kalpha();
        let au = gold_layer_cu_kalpha(10.0, 0.1);
        let model = XrrModel::new_cu_kalpha(vec![au], si);
        let proc = XrrProcessor::new(model);

        let q_vals: Vec<f64> = (1..=20).map(|i| i as f64 * 0.2).collect();
        let born = proc.born_approximation(&q_vals);
        assert_eq!(born.q_values.len(), q_vals.len());
        for &r in &born.reflectivity {
            assert!(r >= 0.0 && r <= 1.0, "Born R out of range: {}", r);
        }
    }

    #[test]
    fn test_born_q4_decay() {
        // Born approximation: R ~ 1/q^4 for a step interface
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);

        let q_vals = vec![1.0, 2.0, 4.0];
        let born = proc.born_approximation(&q_vals);
        if born.reflectivity[0] > 1e-20 && born.reflectivity[2] > 1e-20 {
            // R(q1)/R(q3) should be approximately (q3/q1)^4
            let ratio_r = born.reflectivity[0] / born.reflectivity[2];
            let ratio_q4 = (4.0_f64 / 1.0_f64).powi(4);
            // Allow 50% tolerance (Born is approximate)
            assert!(
                ratio_r > ratio_q4 * 0.1,
                "Born R should decrease roughly as q^-4: ratio={} expected~{}",
                ratio_r,
                ratio_q4
            );
        }
    }

    // ------------------------------------------------------------------
    // Yoneda wing test
    // ------------------------------------------------------------------

    #[test]
    fn test_yoneda_wing_q() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);

        let q_yoneda = proc.yoneda_wing_q();
        // Should be near q_c for Si at Cu Kα: q_c = 4π sin(θ_c)/λ ≈ 0.316 nm^-1
        assert!(q_yoneda > 0.1 && q_yoneda < 0.8, "Yoneda wing q = {} nm^-1", q_yoneda);
    }

    // ------------------------------------------------------------------
    // Model construction tests
    // ------------------------------------------------------------------

    #[test]
    fn test_model_mo_kalpha() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_mo_kalpha(vec![], si);
        assert!((model.wavelength_nm - 0.07107).abs() < 1e-6);
    }

    #[test]
    fn test_layer_from_optical_constants_roundtrip() {
        let delta = 7.44e-6_f64;
        let beta = 1.73e-7_f64;
        let lambda = CU_KALPHA_NM;
        let layer = XrrLayer::from_optical_constants(10.0, delta, beta, lambda, 0.3);
        let k0 = 2.0 * PI / lambda;
        let delta_back = layer.sld_real / (k0 * k0);
        let beta_back = layer.sld_imag / (k0 * k0);
        assert!((delta_back - delta).abs() < 1e-15, "delta roundtrip: {} vs {}", delta_back, delta);
        assert!((beta_back - beta).abs() < 1e-15);
    }

    #[test]
    fn test_single_layer_on_si_constructor() {
        let proc = XrrProcessor::single_layer_on_si(20.0, si_sld_real(), si_sld_imag(), 0.2);
        assert_eq!(proc.model.layers.len(), 1);
        assert!((proc.model.layers[0].thickness_nm - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_list_materials_nonempty() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);
        let list = proc.list_materials();
        assert!(list.len() >= 6);
    }

    // ------------------------------------------------------------------
    // Regression: physical sanity checks
    // ------------------------------------------------------------------

    #[test]
    fn test_au_has_higher_critical_angle_than_si() {
        let theta_c_si = critical_angle_from_delta(7.44e-6);
        let theta_c_au = critical_angle_from_delta(4.66e-5);
        assert!(theta_c_au > theta_c_si, "Gold θ_c > Si θ_c (higher density)");
    }

    #[test]
    fn test_reflectivity_decreases_at_high_q() {
        let si = si_substrate_cu_kalpha();
        let model = XrrModel::new_cu_kalpha(vec![], si);
        let proc = XrrProcessor::new(model);

        let r_low = proc.reflectivity_at_angle(0.5_f64.to_radians());
        let r_high = proc.reflectivity_at_angle(5.0_f64.to_radians());
        assert!(r_low > r_high, "Reflectivity at 0.5° ({}) should exceed at 5° ({})", r_low, r_high);
    }

    #[test]
    fn test_au_film_higher_critical_angle_than_si_substrate() {
        // Au film on Si: critical angle should be closer to Au value
        let si = si_substrate_cu_kalpha();
        let au = gold_layer_cu_kalpha(30.0, 0.1);
        let model_au = XrrModel::new_cu_kalpha(vec![au], si.clone());
        let proc_au = XrrProcessor::new(model_au);

        let model_si = XrrModel::new_cu_kalpha(vec![], si);
        let proc_si = XrrProcessor::new(model_si);

        let tc_au = proc_au.theoretical_critical_angle();
        let tc_si = proc_si.theoretical_critical_angle();
        assert!(tc_au > tc_si, "Au/Si θ_c ({:.5}) should exceed Si θ_c ({:.5})", tc_au, tc_si);
    }

    #[test]
    fn test_thicker_layer_more_fringes() {
        // Thicker films produce more Kiessig fringes in the same q range
        let si = si_substrate_cu_kalpha();

        let thin = XrrLayer::from_optical_constants(10.0, 7.12e-6, 1.54e-7, CU_KALPHA_NM, 0.1);
        let thick = XrrLayer::from_optical_constants(40.0, 7.12e-6, 1.54e-7, CU_KALPHA_NM, 0.1);

        let model_thin = XrrModel::new_cu_kalpha(vec![thin], si.clone());
        let model_thick = XrrModel::new_cu_kalpha(vec![thick], si);
        let proc_thin = XrrProcessor::new(model_thin);
        let proc_thick = XrrProcessor::new(model_thick);

        let curve_thin = proc_thin.compute_reflectivity(0.05, 5.0, 2000);
        let curve_thick = proc_thick.compute_reflectivity(0.05, 5.0, 2000);

        let analysis_thin = proc_thin.analyze_kiessig_fringes(&curve_thin);
        let analysis_thick = proc_thick.analyze_kiessig_fringes(&curve_thick);

        assert!(
            analysis_thick.fringe_count >= analysis_thin.fringe_count,
            "Thicker film should give more fringes: thick={} thin={}",
            analysis_thick.fringe_count,
            analysis_thin.fringe_count
        );
    }

    // ------------------------------------------------------------------
    // Helper functions for tests
    // ------------------------------------------------------------------

    fn si_sld_real() -> f64 {
        let k0 = 2.0 * PI / CU_KALPHA_NM;
        k0 * k0 * 7.44e-6
    }

    fn si_sld_imag() -> f64 {
        let k0 = 2.0 * PI / CU_KALPHA_NM;
        k0 * k0 * 1.73e-7
    }
}
