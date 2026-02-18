//! Refractometry and Brix Sugar Measurement Processor
//!
//! Implements optical refractometry for refractive index measurement and
//! Brix (sugar concentration) determination. Covers the full measurement
//! chain from Snell's law refraction through critical angle analysis,
//! Abbe refractometer simulation, temperature compensation, dispersion
//! modeling, and prism coupler thin film characterization.
//!
//! ## Physics
//!
//! - **Snell's Law**: n₁ sin(θ₁) = n₂ sin(θ₂)
//! - **Critical Angle**: θc = arcsin(n₂/n₁) for total internal reflection
//! - **Fresnel Equations**: Rs and Rp reflectance at dielectric interfaces
//! - **Brewster's Angle**: θB = arctan(n₂/n₁) where Rp = 0
//! - **Abbe Number**: Vd = (nD − 1) / (nF − nC) for dispersion characterization
//! - **Cauchy Dispersion**: n(λ) = A + B/λ² + C/λ⁴
//! - **Sellmeier Dispersion**: n²(λ) = 1 + Σ Bᵢλ²/(λ² − Cᵢ)
//! - **Lorentz-Lorenz**: Specific refraction r = (n²−1)/((n²+2)·ρ)
//! - **Brix Conversion**: ICUMSA polynomial approximation nD → % sugar
//! - **Temperature Compensation**: dn/dT correction to 20 °C reference
//!
//! ## Applications
//!
//! - Food and beverage quality control (sugar content in juices, syrups)
//! - Pharmaceutical concentration monitoring
//! - Petroleum product characterization
//! - Thin film optical constant measurement via prism coupling
//! - Glass and optical material characterization
//!
//! No direct GNU Radio equivalent — optical refractometry / sugar analysis.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::refractometry_brix_analyzer::{
//!     snells_law, critical_angle, BrixConverter, AbbeRefractometer,
//!     CauchyDispersion, TemperatureCorrector,
//! };
//!
//! // Snell's law: light from glass (n=1.5) to air (n=1.0) at 30°
//! let theta2 = snells_law(1.5, 1.0, 30.0_f64.to_radians());
//! assert!(theta2.is_some()); // refraction occurs
//!
//! // Critical angle for glass-air interface
//! let tc = critical_angle(1.5, 1.0);
//! assert!(tc.is_some());
//!
//! // Brix measurement from refractive index
//! let brix = BrixConverter::new().ri_to_brix(1.3475);
//! assert!(brix > 0.0 && brix < 100.0);
//!
//! // Abbe refractometer simulation
//! let abbe = AbbeRefractometer::new(1.75);
//! let ri = abbe.measure_from_critical_angle(1.3330);
//! assert!((ri - 1.3330).abs() < 0.001);
//! ```

use std::f64::consts::PI;

// ─── Physical Constants ──────────────────────────────────────────────────

/// Sodium D-line wavelength in nanometers.
pub const SODIUM_D_LINE_NM: f64 = 589.3;

/// Hydrogen F-line (blue) wavelength in nanometers.
pub const HYDROGEN_F_LINE_NM: f64 = 486.1;

/// Hydrogen C-line (red) wavelength in nanometers.
pub const HYDROGEN_C_LINE_NM: f64 = 656.3;

/// Refractive index of water at 20 °C for the sodium D-line.
pub const WATER_ND_20C: f64 = 1.33299;

/// Standard reference temperature for refractometry (°C).
pub const REFERENCE_TEMP_C: f64 = 20.0;

/// Refractive index of air at standard conditions (STP).
pub const AIR_RI: f64 = 1.000293;

/// Refractive index of BK7 glass at sodium D-line.
pub const BK7_ND: f64 = 1.5168;

/// Refractive index of dense flint glass (SF10) at sodium D-line.
pub const SF10_ND: f64 = 1.7283;

// ─── Core Optics Functions ──────────────────────────────────────────────

/// Applies Snell's law: n₁ sin(θ₁) = n₂ sin(θ₂).
///
/// Returns `Some(theta2)` in radians if refraction occurs, or `None` if
/// total internal reflection happens (sin(θ₂) > 1).
///
/// # Arguments
/// * `n1` - Refractive index of medium 1
/// * `n2` - Refractive index of medium 2
/// * `theta1` - Angle of incidence in radians
pub fn snells_law(n1: f64, n2: f64, theta1: f64) -> Option<f64> {
    let sin_theta2 = n1 * theta1.sin() / n2;
    if sin_theta2.abs() > 1.0 {
        None // total internal reflection
    } else {
        Some(sin_theta2.asin())
    }
}

/// Computes the critical angle for total internal reflection.
///
/// Returns `Some(angle)` in radians when n1 > n2, or `None` when
/// total internal reflection is not possible (n1 <= n2).
///
/// θc = arcsin(n₂ / n₁)
pub fn critical_angle(n1: f64, n2: f64) -> Option<f64> {
    if n1 <= n2 {
        return None; // TIR not possible
    }
    let ratio = n2 / n1;
    if ratio > 1.0 {
        None
    } else {
        Some(ratio.asin())
    }
}

/// Computes Fresnel reflectance coefficients (Rs, Rp) for unpolarized light.
///
/// Returns (Rs, Rp) — s-polarization and p-polarization power reflectance.
///
/// Rs = |( n₁ cos θ₁ − n₂ cos θ₂ ) / ( n₁ cos θ₁ + n₂ cos θ₂ )|²
/// Rp = |( n₂ cos θ₁ − n₁ cos θ₂ ) / ( n₂ cos θ₁ + n₁ cos θ₂ )|²
///
/// Returns `None` if total internal reflection occurs.
pub fn fresnel_reflectance(n1: f64, n2: f64, theta: f64) -> Option<(f64, f64)> {
    let cos1 = theta.cos();
    let sin1 = theta.sin();
    let sin2_sq = (n1 / n2 * sin1).powi(2);
    if sin2_sq > 1.0 {
        return None; // TIR
    }
    let cos2 = (1.0 - sin2_sq).sqrt();

    let rs_num = n1 * cos1 - n2 * cos2;
    let rs_den = n1 * cos1 + n2 * cos2;
    let rs = (rs_num / rs_den).powi(2);

    let rp_num = n2 * cos1 - n1 * cos2;
    let rp_den = n2 * cos1 + n1 * cos2;
    let rp = (rp_num / rp_den).powi(2);

    Some((rs, rp))
}

/// Computes Brewster's angle where p-polarization reflectance vanishes.
///
/// θB = arctan(n₂ / n₁)
pub fn brewster_angle(n1: f64, n2: f64) -> f64 {
    (n2 / n1).atan()
}

/// Computes the Abbe number (constringence) from nD, nF, nC.
///
/// Vd = (nD − 1) / (nF − nC)
///
/// Higher Abbe number means lower dispersion.
pub fn abbe_number(n_d: f64, n_f: f64, n_c: f64) -> f64 {
    if (n_f - n_c).abs() < 1e-12 {
        return f64::INFINITY;
    }
    (n_d - 1.0) / (n_f - n_c)
}

/// Computes the Lorentz-Lorenz specific refraction.
///
/// r = (n² − 1) / ((n² + 2) · ρ)
///
/// # Arguments
/// * `n` - Refractive index
/// * `density` - Density in g/cm³
pub fn specific_refraction(n: f64, density: f64) -> f64 {
    let n_sq = n * n;
    (n_sq - 1.0) / ((n_sq + 2.0) * density)
}

/// Computes the molar refractivity (Lorentz-Lorenz with molar mass).
///
/// R = ((n² − 1) / (n² + 2)) · (M / ρ)
///
/// # Arguments
/// * `n` - Refractive index
/// * `density` - Density in g/cm³
/// * `molar_mass` - Molar mass in g/mol
pub fn molar_refraction(n: f64, density: f64, molar_mass: f64) -> f64 {
    let n_sq = n * n;
    ((n_sq - 1.0) / (n_sq + 2.0)) * (molar_mass / density)
}

/// Computes the average Fresnel reflectance for unpolarized light.
///
/// R_avg = (Rs + Rp) / 2
pub fn average_reflectance(n1: f64, n2: f64, theta: f64) -> Option<f64> {
    fresnel_reflectance(n1, n2, theta).map(|(rs, rp)| (rs + rp) / 2.0)
}

/// Computes the transmittance from reflectance.
///
/// T = 1 − R (for non-absorbing media)
pub fn transmittance_from_reflectance(reflectance: f64) -> f64 {
    1.0 - reflectance
}

/// Normal incidence reflectance: R = ((n1 - n2)/(n1 + n2))²
pub fn normal_incidence_reflectance(n1: f64, n2: f64) -> f64 {
    let r = (n1 - n2) / (n1 + n2);
    r * r
}

/// Converts refractive index to optical density/path length.
///
/// OPL = n · d
pub fn optical_path_length(n: f64, physical_length: f64) -> f64 {
    n * physical_length
}

/// Group refractive index from phase index and dispersion.
///
/// ng = n − λ · dn/dλ
pub fn group_refractive_index(n: f64, wavelength: f64, dn_dlambda: f64) -> f64 {
    n - wavelength * dn_dlambda
}

// ─── Refractive Index Measurement ───────────────────────────────────────

/// Stores a refractive index measurement at a specific wavelength and temperature.
#[derive(Debug, Clone, Copy)]
pub struct RefractiveIndexMeasurement {
    /// Wavelength of measurement in nanometers.
    pub wavelength_nm: f64,
    /// Temperature at measurement in °C.
    pub temperature_c: f64,
    /// Measured critical or refraction angle in radians.
    pub measured_angle_rad: f64,
    /// Computed refractive index.
    pub refractive_index: f64,
}

impl RefractiveIndexMeasurement {
    /// Creates a new measurement from critical angle observation.
    ///
    /// The refractive index is computed as: n_sample = n_prism · sin(θc)
    pub fn from_critical_angle(
        wavelength_nm: f64,
        temperature_c: f64,
        critical_angle_rad: f64,
        prism_ri: f64,
    ) -> Self {
        let ri = prism_ri * critical_angle_rad.sin();
        Self {
            wavelength_nm,
            temperature_c,
            measured_angle_rad: critical_angle_rad,
            refractive_index: ri,
        }
    }

    /// Creates a new measurement from minimum deviation angle.
    ///
    /// For a prism with apex angle A and minimum deviation angle D:
    /// n = sin((A + D) / 2) / sin(A / 2)
    pub fn from_minimum_deviation(
        wavelength_nm: f64,
        temperature_c: f64,
        apex_angle_rad: f64,
        deviation_angle_rad: f64,
    ) -> Self {
        let ri = ((apex_angle_rad + deviation_angle_rad) / 2.0).sin()
            / (apex_angle_rad / 2.0).sin();
        Self {
            wavelength_nm,
            temperature_c,
            measured_angle_rad: deviation_angle_rad,
            refractive_index: ri,
        }
    }

    /// Creates a measurement from a direct RI value.
    pub fn from_direct(wavelength_nm: f64, temperature_c: f64, refractive_index: f64) -> Self {
        Self {
            wavelength_nm,
            temperature_c,
            measured_angle_rad: 0.0,
            refractive_index,
        }
    }
}

// ─── Abbe Refractometer ─────────────────────────────────────────────────

/// Simulates an Abbe refractometer for refractive index measurement.
///
/// The Abbe refractometer determines refractive index by measuring the
/// critical angle at the interface between a high-RI prism and the sample.
/// The sample is placed between two prisms; the critical angle boundary
/// appears as a light-dark boundary in the eyepiece.
///
/// Range: typically 1.300 to 1.700 nD.
#[derive(Debug, Clone)]
pub struct AbbeRefractometer {
    /// Refractive index of the measuring prism.
    prism_ri: f64,
    /// Wavelength of illumination in nm (default: sodium D-line 589.3 nm).
    wavelength_nm: f64,
    /// Operating temperature in °C.
    temperature_c: f64,
    /// Measurement precision (smallest resolvable RI difference).
    precision: f64,
}

impl AbbeRefractometer {
    /// Creates a new Abbe refractometer with the given prism RI.
    ///
    /// Typical prism materials: dense flint glass (nD ≈ 1.73 for SF10).
    pub fn new(prism_ri: f64) -> Self {
        Self {
            prism_ri,
            wavelength_nm: SODIUM_D_LINE_NM,
            temperature_c: REFERENCE_TEMP_C,
            precision: 0.0001,
        }
    }

    /// Creates a standard Abbe refractometer with SF10 prism.
    pub fn standard() -> Self {
        Self::new(SF10_ND)
    }

    /// Sets the operating temperature.
    pub fn with_temperature(mut self, temp_c: f64) -> Self {
        self.temperature_c = temp_c;
        self
    }

    /// Sets the measurement wavelength.
    pub fn with_wavelength(mut self, wavelength_nm: f64) -> Self {
        self.wavelength_nm = wavelength_nm;
        self
    }

    /// Sets the measurement precision.
    pub fn with_precision(mut self, precision: f64) -> Self {
        self.precision = precision;
        self
    }

    /// Computes the critical angle for a sample with the given RI.
    ///
    /// θc = arcsin(n_sample / n_prism)
    ///
    /// Returns `None` if sample RI > prism RI.
    pub fn critical_angle_for_sample(&self, sample_ri: f64) -> Option<f64> {
        if sample_ri >= self.prism_ri {
            return None; // sample RI must be less than prism RI
        }
        Some((sample_ri / self.prism_ri).asin())
    }

    /// Measures RI from observed critical angle.
    ///
    /// n_sample = n_prism · sin(θc)
    pub fn measure_from_critical_angle(&self, sample_ri: f64) -> f64 {
        // Simulate the measurement: compute critical angle, then recover RI
        if let Some(angle) = self.critical_angle_for_sample(sample_ri) {
            let measured = self.prism_ri * angle.sin();
            // Quantize to precision
            (measured / self.precision).round() * self.precision
        } else {
            sample_ri // can't measure — return input
        }
    }

    /// Returns the measurement range [min_ri, max_ri].
    pub fn measurement_range(&self) -> (f64, f64) {
        // Lower limit: near 1.0 (air)
        // Upper limit: just below prism RI
        (1.0, self.prism_ri - self.precision)
    }

    /// Checks if a given RI is within the measurement range.
    pub fn can_measure(&self, sample_ri: f64) -> bool {
        sample_ri >= 1.0 && sample_ri < self.prism_ri
    }

    /// Computes measurement uncertainty at a given sample RI.
    ///
    /// Uncertainty increases near the prism RI (angle approaches π/2).
    pub fn uncertainty_at(&self, sample_ri: f64) -> f64 {
        if !self.can_measure(sample_ri) {
            return f64::INFINITY;
        }
        let angle = (sample_ri / self.prism_ri).asin();
        // Derivative dn/dθ = n_prism · cos(θ)
        // Uncertainty ~ precision / cos(θ)
        self.precision / angle.cos()
    }

    /// Returns the prism refractive index.
    pub fn prism_ri(&self) -> f64 {
        self.prism_ri
    }

    /// Simulates a full measurement producing a RefractiveIndexMeasurement.
    pub fn full_measurement(&self, sample_ri: f64) -> Option<RefractiveIndexMeasurement> {
        let angle = self.critical_angle_for_sample(sample_ri)?;
        let measured_ri = self.prism_ri * angle.sin();
        let quantized = (measured_ri / self.precision).round() * self.precision;
        Some(RefractiveIndexMeasurement {
            wavelength_nm: self.wavelength_nm,
            temperature_c: self.temperature_c,
            measured_angle_rad: angle,
            refractive_index: quantized,
        })
    }
}

// ─── Brix Converter ─────────────────────────────────────────────────────

/// Converts between refractive index and Brix (sugar concentration).
///
/// Uses the ICUMSA (International Commission for Uniform Methods of
/// Sugar Analysis) polynomial approximation relating nD at 20 °C to
/// mass fraction of sucrose (Brix degrees).
///
/// Valid range: 0–85 °Bx.
#[derive(Debug, Clone)]
pub struct BrixConverter {
    /// Reference temperature (typically 20 °C).
    reference_temp_c: f64,
}

impl BrixConverter {
    /// Creates a new BrixConverter with standard 20 °C reference.
    pub fn new() -> Self {
        Self {
            reference_temp_c: REFERENCE_TEMP_C,
        }
    }

    /// Creates a converter with a custom reference temperature.
    pub fn with_reference_temp(temp_c: f64) -> Self {
        Self {
            reference_temp_c: temp_c,
        }
    }

    /// Converts refractive index at 20 °C to Brix (% sugar by mass).
    ///
    /// Uses a polynomial fit of the ICUMSA tables:
    /// Bx = c₀ + c₁·(nD − 1.33299) + c₂·(nD − 1.33299)² + c₃·(nD − 1.33299)³
    ///
    /// Valid for nD in range [1.33299, 1.50350] corresponding to 0–85 °Bx.
    pub fn ri_to_brix(&self, n_d: f64) -> f64 {
        let delta = n_d - WATER_ND_20C;
        if delta < 0.0 {
            return 0.0;
        }
        // ICUMSA polynomial coefficients (fitted to standard tables)
        // Brix ≈ a₁·Δn + a₂·Δn² + a₃·Δn³
        // where Δn = nD - 1.33299
        let a1 = 5.862e2; // ~586.2
        let a2 = -1.715e4; // ~-17150
        let a3 = 4.656e5; // ~465600

        let brix = a1 * delta + a2 * delta * delta + a3 * delta * delta * delta;
        brix.clamp(0.0, 100.0)
    }

    /// Converts Brix (% sugar by mass) to refractive index at 20 °C.
    ///
    /// Inverse of ri_to_brix using Newton-Raphson iteration.
    pub fn brix_to_ri(&self, brix: f64) -> f64 {
        if brix <= 0.0 {
            return WATER_ND_20C;
        }
        // Good initial guess: linear approximation nD ≈ 1.33299 + brix / 586.2
        let mut n = WATER_ND_20C + brix / 586.2;

        // Newton-Raphson iteration
        for _ in 0..20 {
            let delta = n - WATER_ND_20C;
            let a1 = 5.862e2;
            let a2 = -1.715e4;
            let a3 = 4.656e5;

            let f = a1 * delta + a2 * delta * delta + a3 * delta * delta * delta - brix;
            let f_prime = a1 + 2.0 * a2 * delta + 3.0 * a3 * delta * delta;

            if f_prime.abs() < 1e-15 {
                break;
            }
            let correction = f / f_prime;
            n -= correction;
            if correction.abs() < 1e-10 {
                break;
            }
        }
        n
    }

    /// Converts refractive index measured at a given temperature to Brix,
    /// applying temperature correction to the 20 °C reference.
    pub fn ri_to_brix_with_temp(&self, n_d: f64, temp_c: f64) -> f64 {
        let corrector = TemperatureCorrector::default();
        let n_corrected = corrector.correct(n_d, temp_c);
        self.ri_to_brix(n_corrected)
    }

    /// Returns the RI range corresponding to 0–85 °Bx.
    pub fn valid_ri_range(&self) -> (f64, f64) {
        (WATER_ND_20C, self.brix_to_ri(85.0))
    }

    /// Returns the reference temperature.
    pub fn reference_temp(&self) -> f64 {
        self.reference_temp_c
    }

    /// Quick estimate: converts specific gravity (20/20 °C) to approximate Brix.
    ///
    /// Bx ≈ 261.3 · (1 − 1/SG)
    pub fn sg_to_brix(sg: f64) -> f64 {
        if sg <= 1.0 {
            return 0.0;
        }
        261.3 * (1.0 - 1.0 / sg)
    }

    /// Quick estimate: converts Brix to approximate specific gravity.
    ///
    /// SG ≈ 1 / (1 − Bx/261.3)
    pub fn brix_to_sg(brix: f64) -> f64 {
        if brix >= 261.3 {
            return f64::INFINITY;
        }
        1.0 / (1.0 - brix / 261.3)
    }
}

impl Default for BrixConverter {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Temperature Corrector ──────────────────────────────────────────────

/// Temperature compensation for refractive index measurements.
///
/// Corrects measured RI to the standard 20 °C reference temperature.
/// The temperature coefficient of RI for aqueous sugar solutions is
/// approximately −0.00015 per °C (varies slightly with concentration).
#[derive(Debug, Clone)]
pub struct TemperatureCorrector {
    /// Reference temperature in °C (typically 20.0).
    reference_temp_c: f64,
    /// Temperature coefficient dn/dT in per °C (negative for most liquids).
    dn_dt: f64,
    /// Optional second-order coefficient d²n/dT² in per °C².
    d2n_dt2: f64,
}

impl TemperatureCorrector {
    /// Creates a new TemperatureCorrector with the given coefficient.
    pub fn new(reference_temp_c: f64, dn_dt: f64) -> Self {
        Self {
            reference_temp_c,
            dn_dt,
            d2n_dt2: 0.0,
        }
    }

    /// Creates a corrector for aqueous sugar solutions.
    ///
    /// dn/dT ≈ −1.5 × 10⁻⁴ /°C is typical for water and dilute solutions.
    pub fn for_sugar_solution() -> Self {
        Self {
            reference_temp_c: REFERENCE_TEMP_C,
            dn_dt: -1.5e-4,
            d2n_dt2: 0.0,
        }
    }

    /// Creates a corrector for water specifically.
    ///
    /// dn/dT ≈ −1.0 × 10⁻⁴ /°C for pure water near 20 °C.
    pub fn for_water() -> Self {
        Self {
            reference_temp_c: REFERENCE_TEMP_C,
            dn_dt: -1.0e-4,
            d2n_dt2: 0.0,
        }
    }

    /// Adds a second-order temperature coefficient.
    pub fn with_second_order(mut self, d2n_dt2: f64) -> Self {
        self.d2n_dt2 = d2n_dt2;
        self
    }

    /// Corrects measured RI to the reference temperature.
    ///
    /// n_ref = n_meas + dn/dT · (T_ref − T_meas)
    ///       + ½ · d²n/dT² · (T_ref − T_meas)²
    pub fn correct(&self, n_measured: f64, measurement_temp_c: f64) -> f64 {
        let dt = self.reference_temp_c - measurement_temp_c;
        n_measured + self.dn_dt * dt + 0.5 * self.d2n_dt2 * dt * dt
    }

    /// Returns the correction amount (delta n) for a given temperature difference.
    pub fn correction_amount(&self, measurement_temp_c: f64) -> f64 {
        let dt = self.reference_temp_c - measurement_temp_c;
        self.dn_dt * dt + 0.5 * self.d2n_dt2 * dt * dt
    }

    /// Returns the reference temperature.
    pub fn reference_temp(&self) -> f64 {
        self.reference_temp_c
    }

    /// Returns the first-order temperature coefficient.
    pub fn dn_dt(&self) -> f64 {
        self.dn_dt
    }
}

impl Default for TemperatureCorrector {
    fn default() -> Self {
        Self::for_sugar_solution()
    }
}

// ─── Cauchy Dispersion Model ────────────────────────────────────────────

/// Cauchy dispersion equation for refractive index vs. wavelength.
///
/// n(λ) = A + B/λ² + C/λ⁴
///
/// Where λ is the wavelength in micrometers.
/// Accurate for transparent regions far from absorption bands.
#[derive(Debug, Clone, Copy)]
pub struct CauchyDispersion {
    /// Constant term A (dimensionless).
    pub a: f64,
    /// Coefficient B in μm².
    pub b: f64,
    /// Coefficient C in μm⁴.
    pub c: f64,
}

impl CauchyDispersion {
    /// Creates a Cauchy dispersion model with three coefficients.
    pub fn new(a: f64, b: f64, c: f64) -> Self {
        Self { a, b, c }
    }

    /// Creates a two-term Cauchy model (C = 0).
    pub fn two_term(a: f64, b: f64) -> Self {
        Self { a, b, c: 0.0 }
    }

    /// Predefined Cauchy coefficients for BK7 glass.
    pub fn bk7() -> Self {
        Self {
            a: 1.5046,
            b: 0.004200,
            c: 0.0,
        }
    }

    /// Predefined Cauchy coefficients for fused silica.
    pub fn fused_silica() -> Self {
        Self {
            a: 1.4580,
            b: 0.003540,
            c: 0.0,
        }
    }

    /// Predefined Cauchy coefficients for water at 20 °C.
    pub fn water() -> Self {
        Self {
            a: 1.3199,
            b: 0.006878,
            c: 0.0,
        }
    }

    /// Computes the refractive index at a wavelength given in nanometers.
    pub fn index_at_nm(&self, wavelength_nm: f64) -> f64 {
        let lambda_um = wavelength_nm / 1000.0;
        self.index_at_um(lambda_um)
    }

    /// Computes the refractive index at a wavelength given in micrometers.
    pub fn index_at_um(&self, lambda_um: f64) -> f64 {
        let l2 = lambda_um * lambda_um;
        let l4 = l2 * l2;
        self.a + self.b / l2 + self.c / l4
    }

    /// Computes the dispersion dn/dλ at a wavelength (nm).
    pub fn dispersion_at_nm(&self, wavelength_nm: f64) -> f64 {
        let lambda_um = wavelength_nm / 1000.0;
        let l3 = lambda_um * lambda_um * lambda_um;
        let l5 = l3 * lambda_um * lambda_um;
        // dn/dλ = -2B/λ³ - 4C/λ⁵ (in per μm, convert to per nm)
        (-2.0 * self.b / l3 - 4.0 * self.c / l5) / 1000.0
    }

    /// Computes the Abbe number Vd from this dispersion model.
    pub fn abbe_number(&self) -> f64 {
        let n_d = self.index_at_nm(SODIUM_D_LINE_NM);
        let n_f = self.index_at_nm(HYDROGEN_F_LINE_NM);
        let n_c = self.index_at_nm(HYDROGEN_C_LINE_NM);
        abbe_number(n_d, n_f, n_c)
    }

    /// Fits Cauchy coefficients to measured (wavelength_nm, ri) data points.
    ///
    /// Uses least-squares fitting with at least 2 data points.
    /// Returns None if insufficient data.
    pub fn fit(measurements: &[(f64, f64)]) -> Option<Self> {
        if measurements.len() < 2 {
            return None;
        }

        // For 2-term Cauchy: n = A + B/λ²
        // Linear system: [1, 1/λ²] [A, B]ᵀ = [n]
        // Solve via normal equations
        let mut s00 = 0.0f64;
        let mut s01 = 0.0f64;
        let mut s11 = 0.0f64;
        let mut r0 = 0.0f64;
        let mut r1 = 0.0f64;

        for &(wl_nm, ri) in measurements {
            let l_um = wl_nm / 1000.0;
            let inv_l2 = 1.0 / (l_um * l_um);

            s00 += 1.0;
            s01 += inv_l2;
            s11 += inv_l2 * inv_l2;
            r0 += ri;
            r1 += ri * inv_l2;
        }

        let det = s00 * s11 - s01 * s01;
        if det.abs() < 1e-20 {
            return None;
        }

        let a = (s11 * r0 - s01 * r1) / det;
        let b = (s00 * r1 - s01 * r0) / det;

        Some(Self { a, b, c: 0.0 })
    }
}

// ─── Sellmeier Dispersion Model ─────────────────────────────────────────

/// Sellmeier dispersion equation for accurate multi-resonance modeling.
///
/// n²(λ) = 1 + Σᵢ (Bᵢ · λ²) / (λ² − Cᵢ)
///
/// Where λ is in micrometers and Cᵢ are resonance wavelengths squared.
/// More accurate than Cauchy near absorption bands.
#[derive(Debug, Clone)]
pub struct SellmeierDispersion {
    /// Pairs of (Bᵢ, Cᵢ) Sellmeier coefficients.
    /// Bᵢ is dimensionless, Cᵢ is in μm².
    terms: Vec<(f64, f64)>,
}

impl SellmeierDispersion {
    /// Creates a Sellmeier model with the given coefficient pairs.
    pub fn new(terms: Vec<(f64, f64)>) -> Self {
        Self { terms }
    }

    /// Single-term Sellmeier model.
    pub fn one_term(b1: f64, c1: f64) -> Self {
        Self {
            terms: vec![(b1, c1)],
        }
    }

    /// Standard three-term model.
    pub fn three_term(b1: f64, c1: f64, b2: f64, c2: f64, b3: f64, c3: f64) -> Self {
        Self {
            terms: vec![(b1, c1), (b2, c2), (b3, c3)],
        }
    }

    /// Predefined coefficients for BK7 glass (Schott).
    pub fn bk7() -> Self {
        Self::three_term(
            1.03961212,
            0.00600069867,
            0.231792344,
            0.0200179144,
            1.01046945,
            103.560653,
        )
    }

    /// Predefined coefficients for fused silica (SiO2).
    pub fn fused_silica() -> Self {
        Self::three_term(
            0.6961663,
            0.0684043,    // (0.06840μm)²
            0.4079426,
            0.1162414,    // (0.1162μm)²
            0.8974794,
            9.896161,     // (9.896μm)²
        )
    }

    /// Predefined coefficients for water at 20 °C.
    pub fn water() -> Self {
        Self::three_term(
            5.684027565e-1,
            5.101829712e-3,
            1.726177391e-1,
            1.821153936e-2,
            2.086189578e-2,
            2.620722293e-2,
        )
    }

    /// Computes n²(λ) at a wavelength in micrometers.
    pub fn n_squared_at_um(&self, lambda_um: f64) -> f64 {
        let l2 = lambda_um * lambda_um;
        let mut n2 = 1.0;
        for &(b, c) in &self.terms {
            let denom = l2 - c;
            if denom.abs() < 1e-20 {
                continue; // near resonance — skip
            }
            n2 += b * l2 / denom;
        }
        n2
    }

    /// Computes the refractive index at a wavelength in micrometers.
    pub fn index_at_um(&self, lambda_um: f64) -> f64 {
        let n2 = self.n_squared_at_um(lambda_um);
        if n2 > 0.0 {
            n2.sqrt()
        } else {
            0.0
        }
    }

    /// Computes the refractive index at a wavelength in nanometers.
    pub fn index_at_nm(&self, wavelength_nm: f64) -> f64 {
        self.index_at_um(wavelength_nm / 1000.0)
    }

    /// Computes the group index at a wavelength in nanometers.
    ///
    /// ng = n − λ · dn/dλ (numerical differentiation).
    pub fn group_index_at_nm(&self, wavelength_nm: f64) -> f64 {
        let h = 0.1; // 0.1 nm step
        let n = self.index_at_nm(wavelength_nm);
        let n_plus = self.index_at_nm(wavelength_nm + h);
        let n_minus = self.index_at_nm(wavelength_nm - h);
        let dn_dl = (n_plus - n_minus) / (2.0 * h);
        n - wavelength_nm * dn_dl
    }

    /// Computes the Abbe number from this Sellmeier model.
    pub fn abbe_number(&self) -> f64 {
        let n_d = self.index_at_nm(SODIUM_D_LINE_NM);
        let n_f = self.index_at_nm(HYDROGEN_F_LINE_NM);
        let n_c = self.index_at_nm(HYDROGEN_C_LINE_NM);
        abbe_number(n_d, n_f, n_c)
    }

    /// Returns the number of terms in this model.
    pub fn num_terms(&self) -> usize {
        self.terms.len()
    }
}

// ─── Prism Coupler ──────────────────────────────────────────────────────

/// Prism coupler for thin film refractive index measurement.
///
/// Determines thin film RI and thickness by finding guided mode angles.
/// A high-RI prism is pressed against the film; at specific angles,
/// light couples into guided modes of the thin film waveguide.
///
/// Each guided mode m satisfies the transverse resonance condition:
/// 2·k·nf·d·cos(θf) − φ₁₂ − φ₂₃ = 2πm
#[derive(Debug, Clone)]
pub struct PrismCoupler {
    /// Prism refractive index.
    prism_ri: f64,
    /// Prism base angle in radians.
    prism_angle_rad: f64,
    /// Substrate refractive index.
    substrate_ri: f64,
    /// Wavelength of measurement in nm.
    wavelength_nm: f64,
}

impl PrismCoupler {
    /// Creates a new prism coupler measurement setup.
    pub fn new(
        prism_ri: f64,
        prism_angle_rad: f64,
        substrate_ri: f64,
        wavelength_nm: f64,
    ) -> Self {
        Self {
            prism_ri,
            prism_angle_rad,
            substrate_ri,
            wavelength_nm,
        }
    }

    /// Standard setup: SF10 prism (60° base), glass substrate, 633 nm HeNe laser.
    pub fn standard() -> Self {
        Self {
            prism_ri: SF10_ND,
            prism_angle_rad: 60.0_f64.to_radians(),
            substrate_ri: BK7_ND,
            wavelength_nm: 632.8,
        }
    }

    /// Computes the effective index (mode index) from an external coupling angle.
    ///
    /// n_eff = n_prism · sin(θ_prism + arcsin(sin(θ_ext) / n_prism))
    ///
    /// Simplified for flat prism face:
    /// n_eff = n_prism · sin(α + arcsin(sin(θ)/n_prism))
    /// where α is the prism angle and θ is the external incident angle.
    pub fn effective_index(&self, external_angle_rad: f64) -> f64 {
        let sin_ext = external_angle_rad.sin();
        let inside = (sin_ext / self.prism_ri).asin();
        self.prism_ri * (self.prism_angle_rad + inside).sin()
    }

    /// Given a set of mode angles, estimates the film RI and thickness.
    ///
    /// For TE modes in a slab waveguide:
    /// n_eff² = nf² − (m + 1)²·(λ / (2·d))²  (approximate for high modes)
    ///
    /// Returns (film_ri, thickness_nm) or None if insufficient modes.
    pub fn estimate_film_params(&self, mode_angles_rad: &[f64]) -> Option<(f64, f64)> {
        if mode_angles_rad.len() < 2 {
            return None;
        }

        // Convert angles to effective indices
        let n_effs: Vec<f64> = mode_angles_rad
            .iter()
            .map(|&a| self.effective_index(a))
            .collect();

        // For a symmetric slab waveguide with modes m=0,1,...:
        // n_eff[m]² = nf² - ((m+1) * λ/(2d))²
        // From two modes:
        // nf² - n_eff[0]² = (λ/(2d))²
        // nf² - n_eff[1]² = (2λ/(2d))² = 4·(λ/(2d))²
        // So: n_eff[0]² - n_eff[1]² = 3·(λ/(2d))²
        // → d = λ·√3 / (2·√(n_eff[0]² - n_eff[1]²))
        // → nf² = n_eff[0]² + (λ/(2d))²

        let n0_sq = n_effs[0] * n_effs[0];
        let n1_sq = n_effs[1] * n_effs[1];

        let diff = n0_sq - n1_sq;
        if diff <= 0.0 {
            return None;
        }

        let lambda_um = self.wavelength_nm / 1000.0;
        let lambda_sq = lambda_um * lambda_um;

        // d in micrometers
        let d = lambda_um * (3.0_f64).sqrt() / (2.0 * diff.sqrt());
        let nf_sq = n0_sq + lambda_sq / (4.0 * d * d);

        if nf_sq <= 0.0 {
            return None;
        }

        let nf = nf_sq.sqrt();
        let d_nm = d * 1000.0;

        Some((nf, d_nm))
    }

    /// Maximum number of guided modes for a film with given RI and thickness.
    ///
    /// M = floor(2·d·√(nf² − ns²) / λ)
    pub fn max_guided_modes(&self, film_ri: f64, thickness_nm: f64) -> usize {
        let d_um = thickness_nm / 1000.0;
        let lambda_um = self.wavelength_nm / 1000.0;
        let diff = film_ri * film_ri - self.substrate_ri * self.substrate_ri;
        if diff <= 0.0 {
            return 0;
        }
        let m = 2.0 * d_um * diff.sqrt() / lambda_um;
        m.floor() as usize
    }

    /// Computes expected mode effective indices for a given film.
    pub fn expected_mode_indices(&self, film_ri: f64, thickness_nm: f64) -> Vec<f64> {
        let num_modes = self.max_guided_modes(film_ri, thickness_nm);
        let d_um = thickness_nm / 1000.0;
        let lambda_um = self.wavelength_nm / 1000.0;

        let mut indices = Vec::new();
        for m in 0..num_modes {
            let m_term = ((m as f64 + 0.5) * lambda_um / (2.0 * d_um)).powi(2);
            let n_eff_sq = film_ri * film_ri - m_term;
            if n_eff_sq > self.substrate_ri * self.substrate_ri {
                indices.push(n_eff_sq.sqrt());
            }
        }
        indices
    }

    /// Returns the prism refractive index.
    pub fn prism_ri(&self) -> f64 {
        self.prism_ri
    }

    /// Returns the substrate refractive index.
    pub fn substrate_ri(&self) -> f64 {
        self.substrate_ri
    }
}

// ─── Multi-Wavelength Analysis ──────────────────────────────────────────

/// Multi-wavelength refractive index analyzer.
///
/// Combines measurements at multiple wavelengths to determine dispersion
/// properties and fit Cauchy or Sellmeier models.
#[derive(Debug, Clone)]
pub struct MultiWavelengthAnalyzer {
    /// Collected measurements at different wavelengths.
    measurements: Vec<(f64, f64)>, // (wavelength_nm, ri)
}

impl MultiWavelengthAnalyzer {
    /// Creates a new empty analyzer.
    pub fn new() -> Self {
        Self {
            measurements: Vec::new(),
        }
    }

    /// Adds a measurement at a given wavelength.
    pub fn add_measurement(&mut self, wavelength_nm: f64, ri: f64) {
        self.measurements.push((wavelength_nm, ri));
    }

    /// Returns the number of measurements.
    pub fn count(&self) -> usize {
        self.measurements.len()
    }

    /// Fits a Cauchy dispersion model to the collected data.
    pub fn fit_cauchy(&self) -> Option<CauchyDispersion> {
        CauchyDispersion::fit(&self.measurements)
    }

    /// Computes the Abbe number from measurements at D, F, C lines.
    /// Returns None if not all three wavelengths are measured.
    pub fn compute_abbe_number(&self) -> Option<f64> {
        let n_d = self.interpolate_at(SODIUM_D_LINE_NM)?;
        let n_f = self.interpolate_at(HYDROGEN_F_LINE_NM)?;
        let n_c = self.interpolate_at(HYDROGEN_C_LINE_NM)?;
        Some(abbe_number(n_d, n_f, n_c))
    }

    /// Simple linear interpolation to find RI at a given wavelength.
    fn interpolate_at(&self, target_nm: f64) -> Option<f64> {
        if self.measurements.len() < 2 {
            return None;
        }

        let mut sorted = self.measurements.clone();
        sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        // Find bracketing points
        for window in sorted.windows(2) {
            let (wl0, ri0) = window[0];
            let (wl1, ri1) = window[1];
            if target_nm >= wl0 && target_nm <= wl1 {
                let t = (target_nm - wl0) / (wl1 - wl0);
                return Some(ri0 + t * (ri1 - ri0));
            }
        }

        // Extrapolate from nearest pair
        if target_nm < sorted[0].0 {
            let (wl0, ri0) = sorted[0];
            let (wl1, ri1) = sorted[1];
            let t = (target_nm - wl0) / (wl1 - wl0);
            Some(ri0 + t * (ri1 - ri0))
        } else {
            let n = sorted.len();
            let (wl0, ri0) = sorted[n - 2];
            let (wl1, ri1) = sorted[n - 1];
            let t = (target_nm - wl0) / (wl1 - wl0);
            Some(ri0 + t * (ri1 - ri0))
        }
    }

    /// Computes the mean dispersion (nF − nC) from the measurements.
    pub fn mean_dispersion(&self) -> Option<f64> {
        let n_f = self.interpolate_at(HYDROGEN_F_LINE_NM)?;
        let n_c = self.interpolate_at(HYDROGEN_C_LINE_NM)?;
        Some(n_f - n_c)
    }
}

impl Default for MultiWavelengthAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Utility: Concentration from RI ─────────────────────────────────────

/// Estimates concentration of common solutions from refractive index.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SolutionType {
    /// Sucrose (table sugar) solution.
    Sucrose,
    /// Sodium chloride (salt) solution.
    SodiumChloride,
    /// Ethanol (alcohol) solution.
    Ethanol,
    /// Glucose solution.
    Glucose,
}

/// Estimates weight percent concentration from RI for common solutions.
///
/// Uses linear approximation: n = n_water + k · concentration
pub fn concentration_from_ri(solution: SolutionType, n_d: f64) -> f64 {
    let delta = n_d - WATER_ND_20C;
    if delta <= 0.0 {
        return 0.0;
    }

    // Approximate dn/dc (RI increment per weight percent)
    let k = match solution {
        SolutionType::Sucrose => 0.00143,      // per %w/w
        SolutionType::SodiumChloride => 0.00171, // per %w/w
        SolutionType::Ethanol => 0.000365,      // per %w/w (non-linear, approximate)
        SolutionType::Glucose => 0.00142,       // per %w/w
    };

    (delta / k).clamp(0.0, 100.0)
}

/// Computes RI from concentration for common solutions.
pub fn ri_from_concentration(solution: SolutionType, concentration_pct: f64) -> f64 {
    let k = match solution {
        SolutionType::Sucrose => 0.00143,
        SolutionType::SodiumChloride => 0.00171,
        SolutionType::Ethanol => 0.000365,
        SolutionType::Glucose => 0.00142,
    };

    WATER_ND_20C + k * concentration_pct
}

// ─── Refractometer Calibration ──────────────────────────────────────────

/// Refractometer calibration using known standards.
#[derive(Debug, Clone)]
pub struct RefractometerCalibration {
    /// Calibration offset.
    offset: f64,
    /// Calibration scale factor.
    scale: f64,
}

impl RefractometerCalibration {
    /// Creates a calibration from a single-point water standard.
    ///
    /// At 20 °C, distilled water should read nD = 1.33299.
    pub fn from_water_standard(measured_water_ri: f64) -> Self {
        Self {
            offset: WATER_ND_20C - measured_water_ri,
            scale: 1.0,
        }
    }

    /// Creates a two-point calibration from water and a known standard.
    pub fn from_two_point(
        measured_water: f64,
        measured_standard: f64,
        known_standard: f64,
    ) -> Self {
        let expected_water = WATER_ND_20C;
        let scale = (known_standard - expected_water) / (measured_standard - measured_water);
        let offset = expected_water - scale * measured_water;
        Self { offset, scale }
    }

    /// Applies calibration correction to a raw reading.
    pub fn correct(&self, raw_ri: f64) -> f64 {
        self.scale * raw_ri + self.offset
    }

    /// Returns the calibration offset.
    pub fn offset(&self) -> f64 {
        self.offset
    }

    /// Returns the calibration scale factor.
    pub fn scale(&self) -> f64 {
        self.scale
    }
}

impl Default for RefractometerCalibration {
    fn default() -> Self {
        Self {
            offset: 0.0,
            scale: 1.0,
        }
    }
}

// ─── Material Database ──────────────────────────────────────────────────

/// Known material refractive indices at the sodium D-line.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum KnownMaterial {
    /// Vacuum / air (n ≈ 1.0003).
    Air,
    /// Distilled water at 20 °C.
    Water,
    /// Ethanol (ethyl alcohol).
    Ethanol,
    /// Glycerol (glycerine).
    Glycerol,
    /// Olive oil.
    OliveOil,
    /// Crown glass (BK7).
    CrownGlass,
    /// Dense flint glass (SF10).
    FlintGlass,
    /// Diamond.
    Diamond,
    /// Fused silica (SiO2).
    FusedSilica,
    /// Sapphire (Al2O3).
    Sapphire,
    /// Sodium chloride crystal.
    SodiumChloride,
    /// Calcium fluoride (CaF2).
    CalciumFluoride,
    /// PMMA (acrylic, Plexiglas).
    Pmma,
    /// Polycarbonate.
    Polycarbonate,
}

impl KnownMaterial {
    /// Returns the refractive index at the sodium D-line.
    pub fn refractive_index(&self) -> f64 {
        match self {
            Self::Air => AIR_RI,
            Self::Water => WATER_ND_20C,
            Self::Ethanol => 1.3611,
            Self::Glycerol => 1.4729,
            Self::OliveOil => 1.4670,
            Self::CrownGlass => BK7_ND,
            Self::FlintGlass => SF10_ND,
            Self::Diamond => 2.4168,
            Self::FusedSilica => 1.4585,
            Self::Sapphire => 1.7680,
            Self::SodiumChloride => 1.5441,
            Self::CalciumFluoride => 1.4338,
            Self::Pmma => 1.4914,
            Self::Polycarbonate => 1.5860,
        }
    }

    /// Returns the Abbe number (if known) for the material.
    pub fn abbe_number(&self) -> Option<f64> {
        match self {
            Self::CrownGlass => Some(64.17),
            Self::FlintGlass => Some(28.53),
            Self::Diamond => Some(55.3),
            Self::FusedSilica => Some(67.8),
            Self::Water => Some(55.8),
            Self::Pmma => Some(57.4),
            Self::Polycarbonate => Some(30.0),
            _ => None,
        }
    }

    /// Returns the material name as a string.
    pub fn name(&self) -> &'static str {
        match self {
            Self::Air => "Air",
            Self::Water => "Water",
            Self::Ethanol => "Ethanol",
            Self::Glycerol => "Glycerol",
            Self::OliveOil => "Olive Oil",
            Self::CrownGlass => "Crown Glass (BK7)",
            Self::FlintGlass => "Dense Flint Glass (SF10)",
            Self::Diamond => "Diamond",
            Self::FusedSilica => "Fused Silica (SiO2)",
            Self::Sapphire => "Sapphire (Al2O3)",
            Self::SodiumChloride => "Sodium Chloride",
            Self::CalciumFluoride => "Calcium Fluoride (CaF2)",
            Self::Pmma => "PMMA (Acrylic)",
            Self::Polycarbonate => "Polycarbonate",
        }
    }
}

// ─── Tests ──────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;
    const TOL_COARSE: f64 = 1e-3;

    // --- Snell's Law ---

    #[test]
    fn test_snells_law_air_to_glass() {
        // Air to glass at 30°
        let theta2 = snells_law(1.0, 1.5, 30.0_f64.to_radians()).unwrap();
        // sin(30°) = 0.5, sin(θ2) = 0.5/1.5 = 0.333...
        let expected = (0.5 / 1.5_f64).asin();
        assert!((theta2 - expected).abs() < TOL);
    }

    #[test]
    fn test_snells_law_glass_to_air() {
        let theta2 = snells_law(1.5, 1.0, 20.0_f64.to_radians()).unwrap();
        let expected = (1.5 * 20.0_f64.to_radians().sin()).asin();
        assert!((theta2 - expected).abs() < TOL);
    }

    #[test]
    fn test_snells_law_total_internal_reflection() {
        // Glass to air at steep angle → TIR
        let result = snells_law(1.5, 1.0, 50.0_f64.to_radians());
        assert!(result.is_none());
    }

    #[test]
    fn test_snells_law_normal_incidence() {
        let theta2 = snells_law(1.0, 1.5, 0.0).unwrap();
        assert!(theta2.abs() < TOL);
    }

    #[test]
    fn test_snells_law_same_medium() {
        let angle = 45.0_f64.to_radians();
        let theta2 = snells_law(1.5, 1.5, angle).unwrap();
        assert!((theta2 - angle).abs() < TOL);
    }

    // --- Critical Angle ---

    #[test]
    fn test_critical_angle_glass_air() {
        let tc = critical_angle(1.5, 1.0).unwrap();
        let expected = (1.0 / 1.5_f64).asin();
        assert!((tc - expected).abs() < TOL);
    }

    #[test]
    fn test_critical_angle_not_possible() {
        // Air to glass: no TIR
        let result = critical_angle(1.0, 1.5);
        assert!(result.is_none());
    }

    #[test]
    fn test_critical_angle_equal_media() {
        let result = critical_angle(1.5, 1.5);
        assert!(result.is_none());
    }

    #[test]
    fn test_critical_angle_water_air() {
        let tc = critical_angle(WATER_ND_20C, 1.0).unwrap();
        assert!(tc > 0.0 && tc < PI / 2.0);
        // Should be about 48.6°
        let deg = tc.to_degrees();
        assert!((deg - 48.6).abs() < 0.5);
    }

    #[test]
    fn test_critical_angle_diamond_air() {
        let tc = critical_angle(2.417, 1.0).unwrap();
        let deg = tc.to_degrees();
        // Diamond critical angle ≈ 24.4°
        assert!((deg - 24.4).abs() < 0.5);
    }

    // --- Fresnel Reflectance ---

    #[test]
    fn test_fresnel_normal_incidence() {
        let (rs, rp) = fresnel_reflectance(1.0, 1.5, 0.0).unwrap();
        // At normal incidence Rs = Rp = ((1-1.5)/(1+1.5))² = 0.04
        let expected = ((1.0_f64 - 1.5) / (1.0_f64 + 1.5)).powi(2);
        assert!((rs - expected).abs() < TOL);
        assert!((rp - expected).abs() < TOL);
    }

    #[test]
    fn test_fresnel_at_brewster() {
        let theta_b = brewster_angle(1.0, 1.5);
        let (rs, rp) = fresnel_reflectance(1.0, 1.5, theta_b).unwrap();
        // Rp should be essentially zero at Brewster's angle
        assert!(rp < 1e-10);
        // Rs should be nonzero
        assert!(rs > 0.01);
    }

    #[test]
    fn test_fresnel_tir() {
        let result = fresnel_reflectance(1.5, 1.0, 60.0_f64.to_radians());
        assert!(result.is_none());
    }

    #[test]
    fn test_fresnel_grazing_incidence() {
        let (rs, rp) = fresnel_reflectance(1.0, 1.5, 89.0_f64.to_radians()).unwrap();
        // Near grazing, both should approach 1.0
        assert!(rs > 0.9);
        assert!(rp > 0.9);
    }

    // --- Brewster's Angle ---

    #[test]
    fn test_brewster_angle_air_glass() {
        let tb = brewster_angle(1.0, 1.5);
        let deg = tb.to_degrees();
        // Brewster angle for air→glass ≈ 56.3°
        assert!((deg - 56.31).abs() < 0.1);
    }

    #[test]
    fn test_brewster_angle_symmetry() {
        // Brewster angles from both sides: θ_B1 + θ_B2 = 90°
        let tb1 = brewster_angle(1.0, 1.5);
        let tb2 = brewster_angle(1.5, 1.0);
        assert!((tb1 + tb2 - PI / 2.0).abs() < TOL);
    }

    // --- Abbe Number ---

    #[test]
    fn test_abbe_number_bk7() {
        // BK7: nD=1.5168, nF=1.5224, nC=1.5143
        let vd = abbe_number(1.5168, 1.5224, 1.5143);
        // Expected ≈ 63.8
        assert!((vd - 63.8).abs() < 1.0);
    }

    #[test]
    fn test_abbe_number_high_dispersion() {
        // Flint glass with large nF-nC → low Abbe number
        let vd = abbe_number(1.72, 1.74, 1.71);
        assert!(vd < 30.0);
    }

    #[test]
    fn test_abbe_number_no_dispersion() {
        let vd = abbe_number(1.5, 1.5, 1.5);
        assert!(vd.is_infinite());
    }

    // --- Specific and Molar Refraction ---

    #[test]
    fn test_specific_refraction_water() {
        let r = specific_refraction(WATER_ND_20C, 0.998);
        // Should be positive
        assert!(r > 0.0);
        // Lorentz-Lorenz for water: ≈ 0.206
        assert!((r - 0.206).abs() < 0.01);
    }

    #[test]
    fn test_molar_refraction_water() {
        let rm = molar_refraction(WATER_ND_20C, 0.998, 18.015);
        // Molar refractivity of water ≈ 3.71 cm³/mol
        assert!((rm - 3.71).abs() < 0.1);
    }

    // --- Normal Incidence Reflectance ---

    #[test]
    fn test_normal_incidence_reflectance_air_glass() {
        let r = normal_incidence_reflectance(1.0, 1.5);
        let expected = ((1.0_f64 - 1.5) / (1.0_f64 + 1.5)).powi(2);
        assert!((r - expected).abs() < TOL);
    }

    #[test]
    fn test_normal_incidence_reflectance_symmetric() {
        let r1 = normal_incidence_reflectance(1.0, 1.5);
        let r2 = normal_incidence_reflectance(1.5, 1.0);
        assert!((r1 - r2).abs() < TOL);
    }

    // --- Optical Path Length ---

    #[test]
    fn test_optical_path_length() {
        let opl = optical_path_length(1.5, 10.0);
        assert!((opl - 15.0).abs() < TOL);
    }

    // --- Group Refractive Index ---

    #[test]
    fn test_group_refractive_index() {
        // For positive dispersion (dn/dλ < 0), ng > n
        let ng = group_refractive_index(1.5, 589.3, -0.0001);
        assert!(ng > 1.5);
    }

    // --- Average Reflectance ---

    #[test]
    fn test_average_reflectance_normal() {
        let r = average_reflectance(1.0, 1.5, 0.0).unwrap();
        let expected = ((1.0_f64 - 1.5) / (1.0_f64 + 1.5)).powi(2);
        assert!((r - expected).abs() < TOL);
    }

    #[test]
    fn test_transmittance_from_reflectance() {
        let t = transmittance_from_reflectance(0.04);
        assert!((t - 0.96).abs() < TOL);
    }

    // --- Refractive Index Measurement ---

    #[test]
    fn test_measurement_from_critical_angle() {
        let m = RefractiveIndexMeasurement::from_critical_angle(
            589.3,
            20.0,
            (1.333 / 1.73_f64).asin(),
            1.73,
        );
        assert!((m.refractive_index - 1.333).abs() < TOL_COARSE);
    }

    #[test]
    fn test_measurement_from_minimum_deviation() {
        // Equilateral prism (A=60°) with min deviation D
        let apex = 60.0_f64.to_radians();
        let dev = 40.0_f64.to_radians();
        let m = RefractiveIndexMeasurement::from_minimum_deviation(589.3, 20.0, apex, dev);
        // n = sin((60+40)/2) / sin(30) = sin(50°) / 0.5
        let expected = 50.0_f64.to_radians().sin() / 0.5;
        assert!((m.refractive_index - expected).abs() < TOL);
    }

    #[test]
    fn test_measurement_direct() {
        let m = RefractiveIndexMeasurement::from_direct(589.3, 20.0, 1.3330);
        assert!((m.refractive_index - 1.3330).abs() < TOL);
    }

    // --- Abbe Refractometer ---

    #[test]
    fn test_abbe_standard() {
        let abbe = AbbeRefractometer::standard();
        assert!((abbe.prism_ri() - SF10_ND).abs() < TOL);
    }

    #[test]
    fn test_abbe_measure_water() {
        let abbe = AbbeRefractometer::standard();
        let ri = abbe.measure_from_critical_angle(WATER_ND_20C);
        assert!((ri - WATER_ND_20C).abs() < 0.001);
    }

    #[test]
    fn test_abbe_critical_angle_for_sample() {
        let abbe = AbbeRefractometer::new(1.75);
        let angle = abbe.critical_angle_for_sample(1.333).unwrap();
        let expected = (1.333 / 1.75_f64).asin();
        assert!((angle - expected).abs() < TOL);
    }

    #[test]
    fn test_abbe_cannot_measure_too_high() {
        let abbe = AbbeRefractometer::new(1.5);
        assert!(!abbe.can_measure(1.6));
        assert!(abbe.critical_angle_for_sample(1.6).is_none());
    }

    #[test]
    fn test_abbe_measurement_range() {
        let abbe = AbbeRefractometer::new(1.75);
        let (min, max) = abbe.measurement_range();
        assert!((min - 1.0).abs() < TOL);
        assert!(max < 1.75);
    }

    #[test]
    fn test_abbe_uncertainty_increases() {
        let abbe = AbbeRefractometer::new(1.75);
        let u_low = abbe.uncertainty_at(1.3);
        let u_high = abbe.uncertainty_at(1.7);
        // Uncertainty should be higher near the prism RI
        assert!(u_high > u_low);
    }

    #[test]
    fn test_abbe_full_measurement() {
        let abbe = AbbeRefractometer::standard();
        let m = abbe.full_measurement(1.4500).unwrap();
        assert!((m.refractive_index - 1.4500).abs() < 0.001);
        assert!((m.wavelength_nm - SODIUM_D_LINE_NM).abs() < TOL);
    }

    // --- Brix Converter ---

    #[test]
    fn test_brix_water_is_zero() {
        let bc = BrixConverter::new();
        let brix = bc.ri_to_brix(WATER_ND_20C);
        assert!(brix.abs() < 0.1);
    }

    #[test]
    fn test_brix_positive_for_sugar() {
        let bc = BrixConverter::new();
        // 10% sugar → nD ≈ 1.3479
        let brix = bc.ri_to_brix(1.3479);
        assert!(brix > 5.0 && brix < 15.0);
    }

    #[test]
    fn test_brix_increases_with_ri() {
        let bc = BrixConverter::new();
        let b1 = bc.ri_to_brix(1.34);
        let b2 = bc.ri_to_brix(1.36);
        let b3 = bc.ri_to_brix(1.38);
        assert!(b1 < b2);
        assert!(b2 < b3);
    }

    #[test]
    fn test_brix_roundtrip() {
        let bc = BrixConverter::new();
        for brix_in in [0.0, 10.0, 20.0, 40.0, 60.0] {
            let ri = bc.brix_to_ri(brix_in);
            let brix_out = bc.ri_to_brix(ri);
            assert!(
                (brix_out - brix_in).abs() < 0.5,
                "Roundtrip failed: {} -> {} -> {}",
                brix_in,
                ri,
                brix_out
            );
        }
    }

    #[test]
    fn test_brix_below_water() {
        let bc = BrixConverter::new();
        let brix = bc.ri_to_brix(1.300);
        assert!(brix.abs() < TOL);
    }

    #[test]
    fn test_brix_to_ri_zero() {
        let bc = BrixConverter::new();
        let ri = bc.brix_to_ri(0.0);
        assert!((ri - WATER_ND_20C).abs() < TOL);
    }

    #[test]
    fn test_brix_with_temp() {
        let bc = BrixConverter::new();
        // At 25°C, a measurement should give slightly different Brix
        let brix_20 = bc.ri_to_brix(1.3475);
        let brix_25 = bc.ri_to_brix_with_temp(1.3475, 25.0);
        // Warmer → lower RI → correcting up should give higher Brix
        assert!(brix_25 > brix_20);
    }

    #[test]
    fn test_brix_valid_range() {
        let bc = BrixConverter::new();
        let (lo, hi) = bc.valid_ri_range();
        assert!((lo - WATER_ND_20C).abs() < TOL);
        assert!(hi > 1.45);
    }

    #[test]
    fn test_brix_sg_roundtrip() {
        for brix in [5.0, 15.0, 30.0, 50.0] {
            let sg = BrixConverter::brix_to_sg(brix);
            let brix_out = BrixConverter::sg_to_brix(sg);
            assert!(
                (brix_out - brix).abs() < 0.1,
                "SG roundtrip failed: {} -> {} -> {}",
                brix,
                sg,
                brix_out
            );
        }
    }

    #[test]
    fn test_sg_to_brix_at_one() {
        let brix = BrixConverter::sg_to_brix(1.0);
        assert!(brix.abs() < 0.1);
    }

    // --- Temperature Corrector ---

    #[test]
    fn test_temp_correction_at_reference() {
        let tc = TemperatureCorrector::default();
        let corrected = tc.correct(WATER_ND_20C, REFERENCE_TEMP_C);
        assert!((corrected - WATER_ND_20C).abs() < TOL);
    }

    #[test]
    fn test_temp_correction_warmer() {
        let tc = TemperatureCorrector::for_sugar_solution();
        // At 25°C, RI is lower → correction should increase it
        let measured_at_25 = 1.3470;
        let corrected = tc.correct(measured_at_25, 25.0);
        assert!(corrected > measured_at_25);
    }

    #[test]
    fn test_temp_correction_cooler() {
        let tc = TemperatureCorrector::for_sugar_solution();
        // At 15°C, RI is higher → correction should decrease it
        let measured_at_15 = 1.3470;
        let corrected = tc.correct(measured_at_15, 15.0);
        assert!(corrected < measured_at_15);
    }

    #[test]
    fn test_temp_correction_amount() {
        let tc = TemperatureCorrector::new(20.0, -1.5e-4);
        let amount = tc.correction_amount(25.0);
        // 5°C offset, dn/dT = -1.5e-4 → correction = -1.5e-4 * (20-25) = 7.5e-4
        assert!((amount - 7.5e-4).abs() < TOL);
    }

    #[test]
    fn test_temp_correction_second_order() {
        let tc = TemperatureCorrector::new(20.0, -1.5e-4).with_second_order(1e-6);
        let corrected = tc.correct(1.3330, 30.0);
        let first_order_only = TemperatureCorrector::new(20.0, -1.5e-4).correct(1.3330, 30.0);
        // Second order should make a difference
        assert!((corrected - first_order_only).abs() > 1e-7);
    }

    #[test]
    fn test_temp_corrector_water() {
        let tc = TemperatureCorrector::for_water();
        assert!((tc.dn_dt() - (-1.0e-4)).abs() < TOL);
    }

    // --- Cauchy Dispersion ---

    #[test]
    fn test_cauchy_index_at_d_line() {
        let cauchy = CauchyDispersion::bk7();
        let n = cauchy.index_at_nm(SODIUM_D_LINE_NM);
        // BK7 at D-line ≈ 1.517
        assert!((n - 1.517).abs() < 0.02);
    }

    #[test]
    fn test_cauchy_dispersion_sign() {
        let cauchy = CauchyDispersion::bk7();
        // Normal dispersion: n increases with shorter wavelength
        let n_blue = cauchy.index_at_nm(450.0);
        let n_red = cauchy.index_at_nm(650.0);
        assert!(n_blue > n_red);
    }

    #[test]
    fn test_cauchy_dispersion_derivative() {
        let cauchy = CauchyDispersion::bk7();
        let dn = cauchy.dispersion_at_nm(589.3);
        // dn/dλ should be negative for normal dispersion
        assert!(dn < 0.0);
    }

    #[test]
    fn test_cauchy_two_term() {
        let cauchy = CauchyDispersion::two_term(1.5, 0.004);
        let n = cauchy.index_at_nm(589.3);
        assert!(n > 1.5);
    }

    #[test]
    fn test_cauchy_water() {
        let cauchy = CauchyDispersion::water();
        let n = cauchy.index_at_nm(SODIUM_D_LINE_NM);
        // Should be close to 1.333
        assert!((n - 1.333).abs() < 0.01);
    }

    #[test]
    fn test_cauchy_fused_silica() {
        let cauchy = CauchyDispersion::fused_silica();
        let n = cauchy.index_at_nm(SODIUM_D_LINE_NM);
        assert!((n - 1.458).abs() < 0.01);
    }

    #[test]
    fn test_cauchy_abbe_number() {
        let cauchy = CauchyDispersion::bk7();
        let vd = cauchy.abbe_number();
        // BK7 Abbe number ≈ 64
        assert!(vd > 40.0 && vd < 100.0);
    }

    #[test]
    fn test_cauchy_fit() {
        let original = CauchyDispersion::bk7();
        let measurements: Vec<(f64, f64)> = vec![
            (450.0, original.index_at_nm(450.0)),
            (550.0, original.index_at_nm(550.0)),
            (650.0, original.index_at_nm(650.0)),
        ];
        let fitted = CauchyDispersion::fit(&measurements).unwrap();
        // Fitted should reproduce the data
        for &(wl, ri) in &measurements {
            let n = fitted.index_at_nm(wl);
            assert!(
                (n - ri).abs() < 1e-4,
                "Fit failed at {}nm: expected {}, got {}",
                wl,
                ri,
                n
            );
        }
    }

    #[test]
    fn test_cauchy_fit_insufficient_data() {
        let result = CauchyDispersion::fit(&[(589.3, 1.5)]);
        // Should still work with 2-term fit even with 1 point?
        // Actually our implementation requires ≥ 2
        assert!(result.is_none());
    }

    // --- Sellmeier Dispersion ---

    #[test]
    fn test_sellmeier_bk7_at_d_line() {
        let sell = SellmeierDispersion::bk7();
        let n = sell.index_at_nm(SODIUM_D_LINE_NM);
        // BK7 at D-line ≈ 1.5168
        assert!((n - 1.5168).abs() < 0.005);
    }

    #[test]
    fn test_sellmeier_normal_dispersion() {
        let sell = SellmeierDispersion::bk7();
        let n_blue = sell.index_at_nm(450.0);
        let n_red = sell.index_at_nm(700.0);
        assert!(n_blue > n_red);
    }

    #[test]
    fn test_sellmeier_fused_silica() {
        let sell = SellmeierDispersion::fused_silica();
        let n = sell.index_at_nm(SODIUM_D_LINE_NM);
        assert!((n - 1.458).abs() < 0.01);
    }

    #[test]
    fn test_sellmeier_water() {
        let sell = SellmeierDispersion::water();
        let n = sell.index_at_nm(SODIUM_D_LINE_NM);
        assert!((n - 1.333).abs() < 0.01);
    }

    #[test]
    fn test_sellmeier_abbe_number() {
        let sell = SellmeierDispersion::bk7();
        let vd = sell.abbe_number();
        assert!(vd > 50.0 && vd < 80.0);
    }

    #[test]
    fn test_sellmeier_group_index() {
        let sell = SellmeierDispersion::bk7();
        let ng = sell.group_index_at_nm(SODIUM_D_LINE_NM);
        let n = sell.index_at_nm(SODIUM_D_LINE_NM);
        // Group index > phase index for normal dispersion
        assert!(ng > n);
    }

    #[test]
    fn test_sellmeier_one_term() {
        let sell = SellmeierDispersion::one_term(1.0, 0.01);
        let n = sell.index_at_nm(589.3);
        assert!(n > 1.0);
    }

    #[test]
    fn test_sellmeier_num_terms() {
        let sell = SellmeierDispersion::bk7();
        assert_eq!(sell.num_terms(), 3);
    }

    // --- Prism Coupler ---

    #[test]
    fn test_prism_coupler_standard() {
        let pc = PrismCoupler::standard();
        assert!((pc.prism_ri() - SF10_ND).abs() < TOL);
    }

    #[test]
    fn test_prism_coupler_effective_index() {
        let pc = PrismCoupler::standard();
        let n_eff = pc.effective_index(0.0);
        // At zero external angle: n_eff = n_prism * sin(α)
        let expected = SF10_ND * 60.0_f64.to_radians().sin();
        assert!((n_eff - expected).abs() < 0.01);
    }

    #[test]
    fn test_prism_coupler_max_modes() {
        let pc = PrismCoupler::new(1.73, 60.0_f64.to_radians(), 1.5, 632.8);
        // Thick high-RI film should support multiple modes
        let modes = pc.max_guided_modes(1.65, 5000.0);
        assert!(modes > 0);
    }

    #[test]
    fn test_prism_coupler_thin_film_no_modes() {
        let pc = PrismCoupler::new(1.73, 60.0_f64.to_radians(), 1.5, 632.8);
        // Very thin film may not support any modes
        let modes = pc.max_guided_modes(1.55, 50.0);
        // Thin film of low contrast → 0 or few modes
        assert!(modes < 3);
    }

    #[test]
    fn test_prism_coupler_estimate_film_params() {
        let pc = PrismCoupler::new(1.75, 60.0_f64.to_radians(), 1.45, 632.8);
        // Two mode angles → can estimate film params
        let angles = vec![10.0_f64.to_radians(), 20.0_f64.to_radians()];
        let result = pc.estimate_film_params(&angles);
        // Should return Some (may not be physically meaningful with arbitrary angles)
        assert!(result.is_some());
    }

    #[test]
    fn test_prism_coupler_insufficient_modes() {
        let pc = PrismCoupler::standard();
        let result = pc.estimate_film_params(&[10.0_f64.to_radians()]);
        assert!(result.is_none());
    }

    #[test]
    fn test_prism_coupler_expected_modes() {
        let pc = PrismCoupler::new(1.73, 60.0_f64.to_radians(), 1.45, 632.8);
        let indices = pc.expected_mode_indices(1.65, 3000.0);
        // Should have some modes
        assert!(!indices.is_empty());
        // All effective indices should be between substrate and film RI
        for &n in &indices {
            assert!(n > 1.45, "n_eff {} below substrate", n);
            assert!(n < 1.65, "n_eff {} above film RI", n);
        }
    }

    // --- Multi-Wavelength Analyzer ---

    #[test]
    fn test_multi_wavelength_new() {
        let mwa = MultiWavelengthAnalyzer::new();
        assert_eq!(mwa.count(), 0);
    }

    #[test]
    fn test_multi_wavelength_add() {
        let mut mwa = MultiWavelengthAnalyzer::new();
        mwa.add_measurement(589.3, 1.5168);
        mwa.add_measurement(486.1, 1.5224);
        assert_eq!(mwa.count(), 2);
    }

    #[test]
    fn test_multi_wavelength_fit_cauchy() {
        let mut mwa = MultiWavelengthAnalyzer::new();
        let cauchy_ref = CauchyDispersion::bk7();
        for wl in [450.0, 500.0, 550.0, 600.0, 650.0] {
            mwa.add_measurement(wl, cauchy_ref.index_at_nm(wl));
        }
        let fitted = mwa.fit_cauchy().unwrap();
        // Verify fit at the D-line
        let n_fit = fitted.index_at_nm(SODIUM_D_LINE_NM);
        let n_ref = cauchy_ref.index_at_nm(SODIUM_D_LINE_NM);
        assert!((n_fit - n_ref).abs() < 0.005);
    }

    #[test]
    fn test_multi_wavelength_abbe_number() {
        let mut mwa = MultiWavelengthAnalyzer::new();
        mwa.add_measurement(450.0, 1.526);
        mwa.add_measurement(486.1, 1.5224);
        mwa.add_measurement(550.0, 1.519);
        mwa.add_measurement(589.3, 1.5168);
        mwa.add_measurement(656.3, 1.5143);
        mwa.add_measurement(700.0, 1.512);
        let vd = mwa.compute_abbe_number().unwrap();
        assert!(vd > 50.0 && vd < 80.0);
    }

    #[test]
    fn test_multi_wavelength_mean_dispersion() {
        let mut mwa = MultiWavelengthAnalyzer::new();
        mwa.add_measurement(486.1, 1.5224);
        mwa.add_measurement(656.3, 1.5143);
        let disp = mwa.mean_dispersion().unwrap();
        assert!((disp - 0.0081).abs() < 0.001);
    }

    // --- Concentration Functions ---

    #[test]
    fn test_concentration_sucrose() {
        let c = concentration_from_ri(SolutionType::Sucrose, 1.3473);
        // 10% sucrose → nD ≈ 1.3473
        assert!((c - 10.0).abs() < 1.5);
    }

    #[test]
    fn test_concentration_salt() {
        let c = concentration_from_ri(SolutionType::SodiumChloride, 1.3500);
        assert!(c > 5.0 && c < 15.0);
    }

    #[test]
    fn test_concentration_ethanol() {
        let c = concentration_from_ri(SolutionType::Ethanol, 1.3400);
        assert!(c > 0.0);
    }

    #[test]
    fn test_concentration_glucose() {
        let c = concentration_from_ri(SolutionType::Glucose, 1.3473);
        assert!(c > 5.0 && c < 15.0);
    }

    #[test]
    fn test_concentration_pure_water() {
        let c = concentration_from_ri(SolutionType::Sucrose, WATER_ND_20C);
        assert!(c.abs() < 0.1);
    }

    #[test]
    fn test_ri_from_concentration_roundtrip() {
        for soln in [
            SolutionType::Sucrose,
            SolutionType::SodiumChloride,
            SolutionType::Glucose,
        ] {
            let ri = ri_from_concentration(soln, 10.0);
            let c = concentration_from_ri(soln, ri);
            assert!(
                (c - 10.0).abs() < 0.1,
                "Roundtrip failed for {:?}",
                soln
            );
        }
    }

    // --- Calibration ---

    #[test]
    fn test_calibration_water_standard() {
        let cal = RefractometerCalibration::from_water_standard(1.3325);
        // Should correct 1.3325 to 1.33299
        let corrected = cal.correct(1.3325);
        assert!((corrected - WATER_ND_20C).abs() < TOL);
    }

    #[test]
    fn test_calibration_default() {
        let cal = RefractometerCalibration::default();
        let ri = 1.5000;
        let corrected = cal.correct(ri);
        assert!((corrected - ri).abs() < TOL);
    }

    #[test]
    fn test_calibration_two_point() {
        let cal = RefractometerCalibration::from_two_point(1.3325, 1.4500, 1.4585);
        // Should correct water reading accurately
        let corrected_water = cal.correct(1.3325);
        assert!((corrected_water - WATER_ND_20C).abs() < 0.005);
    }

    // --- Material Database ---

    #[test]
    fn test_known_material_water() {
        let ri = KnownMaterial::Water.refractive_index();
        assert!((ri - WATER_ND_20C).abs() < TOL);
    }

    #[test]
    fn test_known_material_diamond() {
        let ri = KnownMaterial::Diamond.refractive_index();
        assert!(ri > 2.4);
    }

    #[test]
    fn test_known_material_name() {
        assert_eq!(KnownMaterial::Water.name(), "Water");
        assert_eq!(KnownMaterial::Diamond.name(), "Diamond");
    }

    #[test]
    fn test_known_material_abbe() {
        let vd = KnownMaterial::CrownGlass.abbe_number().unwrap();
        assert!(vd > 60.0);
    }

    #[test]
    fn test_known_material_abbe_none() {
        assert!(KnownMaterial::OliveOil.abbe_number().is_none());
    }

    #[test]
    fn test_all_materials_positive_ri() {
        let materials = [
            KnownMaterial::Air,
            KnownMaterial::Water,
            KnownMaterial::Ethanol,
            KnownMaterial::Glycerol,
            KnownMaterial::OliveOil,
            KnownMaterial::CrownGlass,
            KnownMaterial::FlintGlass,
            KnownMaterial::Diamond,
            KnownMaterial::FusedSilica,
            KnownMaterial::Sapphire,
            KnownMaterial::SodiumChloride,
            KnownMaterial::CalciumFluoride,
            KnownMaterial::Pmma,
            KnownMaterial::Polycarbonate,
        ];
        for m in &materials {
            assert!(m.refractive_index() >= 1.0, "{} has RI < 1.0", m.name());
        }
    }

    #[test]
    fn test_all_materials_ordered() {
        // Air should have the lowest RI and Diamond the highest among these
        assert!(KnownMaterial::Air.refractive_index() < KnownMaterial::Water.refractive_index());
        assert!(
            KnownMaterial::Water.refractive_index() < KnownMaterial::Diamond.refractive_index()
        );
    }

    // --- Constants ---

    #[test]
    fn test_wavelength_constants() {
        assert!((SODIUM_D_LINE_NM - 589.3).abs() < TOL);
        assert!((HYDROGEN_F_LINE_NM - 486.1).abs() < TOL);
        assert!((HYDROGEN_C_LINE_NM - 656.3).abs() < TOL);
    }

    #[test]
    fn test_water_constant() {
        assert!((WATER_ND_20C - 1.33299).abs() < TOL);
    }

    #[test]
    fn test_reference_temp() {
        assert!((REFERENCE_TEMP_C - 20.0).abs() < TOL);
    }
}
