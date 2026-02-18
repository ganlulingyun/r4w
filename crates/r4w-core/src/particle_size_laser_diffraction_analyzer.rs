//! # Particle Size Laser Diffraction Analyzer
//!
//! Implements laser diffraction particle size analysis based on Fraunhofer diffraction
//! and Mie scattering theory for particle size distribution (PSD) measurement.
//!
//! ## Overview
//!
//! Laser diffraction is the most widely used technique for measuring particle size
//! distributions in the range of ~0.1 um to ~3000 um. A laser beam illuminates a
//! dispersion of particles, and the angular distribution of scattered light intensity
//! is measured by an array of detectors at various angles.
//!
//! ### Fraunhofer Diffraction
//!
//! For particles much larger than the wavelength (d >> lambda), the Fraunhofer
//! approximation applies. The diffraction pattern from a circular aperture is the
//! Airy pattern:
//!
//! ```text
//!   I(theta) = I_0 * [2 * J_1(x) / x]^2
//! ```
//!
//! where `x = pi * d * sin(theta) / lambda`, `d` is the particle diameter, `lambda`
//! is the laser wavelength, and `J_1` is the Bessel function of the first kind,
//! order one.
//!
//! ### Mie Scattering
//!
//! For smaller particles (d ~ lambda), the full Mie solution to Maxwell's equations
//! is needed. This module provides a simplified Mie efficiency factor calculation
//! suitable for estimating scattering and extinction cross-sections.
//!
//! ## Distribution Models
//!
//! - **Log-Normal**: `f(x) = 1/(x*sigma*sqrt(2*pi)) * exp(-(ln(x)-mu)^2 / (2*sigma^2))`
//! - **Rosin-Rammler**: `Q(x) = 1 - exp(-(x/x')^n)` cumulative mass fraction
//!
//! ## Key Metrics
//!
//! - **D10, D50 (median), D90**: Percentile diameters from cumulative distribution
//! - **D[3,2]** (Sauter mean): Volume-to-surface area mean diameter
//! - **D[4,3]** (De Brouckere mean): Volume-weighted mean diameter
//! - **Span**: (D90 - D10) / D50, a measure of distribution width
//! - **Specific Surface Area (SSA)**: 6 / (rho * D[3,2])
//!
//! ## Standards
//!
//! - ISO 13320:2020 — Particle size analysis by laser diffraction
//!
//! ## Physical Constants
//!
//! - He-Ne laser wavelength: 632.8 nm
//! - Diode laser wavelengths: 405 nm (violet), 470 nm (blue)
//! - Refractive index of water at 632.8 nm: ~1.33
//! - Typical obscuration range: 5–25%
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::particle_size_laser_diffraction_analyzer::*;
//!
//! // Create a particle size distribution with bin edges and volume fractions
//! let bin_edges = vec![1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0]; // micrometers
//! let fractions = vec![0.02, 0.08, 0.25, 0.35, 0.20, 0.10]; // volume fractions
//! let psd = ParticleSizeDistribution::new(&bin_edges, &fractions).unwrap();
//!
//! // Compute percentiles
//! let pc = PercentileCalculator::new(&psd);
//! let d50 = pc.percentile(50.0).unwrap();
//!
//! // Compute mean diameters
//! let means = MeanDiameters::compute(&psd);
//! let sauter = means.d_3_2; // Sauter mean diameter
//! let de_brouckere = means.d_4_3; // De Brouckere mean diameter
//! ```

use std::f64::consts::PI;

// ============================================================================
// Physical Constants
// ============================================================================

/// He-Ne laser wavelength in micrometers
pub const HENE_WAVELENGTH_UM: f64 = 0.6328;

/// Violet diode laser wavelength in micrometers
pub const VIOLET_DIODE_WAVELENGTH_UM: f64 = 0.405;

/// Blue diode laser wavelength in micrometers
pub const BLUE_DIODE_WAVELENGTH_UM: f64 = 0.470;

/// Refractive index of water at 632.8 nm
pub const REFRACTIVE_INDEX_WATER: f64 = 1.33;

/// Minimum recommended obscuration (%)
pub const MIN_OBSCURATION_PCT: f64 = 5.0;

/// Maximum recommended obscuration (%)
pub const MAX_OBSCURATION_PCT: f64 = 25.0;

// ============================================================================
// Bessel Function J_1(x) — polynomial approximation
// ============================================================================

/// Compute the Bessel function of the first kind, order 1: J_1(x).
///
/// Uses rational polynomial approximations for |x| <= 8 and asymptotic
/// expansion for |x| > 8, following Abramowitz & Stegun (1972) formulas
/// 9.4.4 and 9.4.6.
///
/// # Arguments
///
/// * `x` - The argument to J_1(x)
///
/// # Returns
///
/// The value of J_1(x)
pub fn bessel_j1(x: f64) -> f64 {
    let ax = x.abs();

    if ax < 1e-15 {
        return 0.5 * x;
    }

    if ax <= 8.0 {
        // Rational approximation for |x| <= 8
        // Abramowitz & Stegun, 9.4.4
        let y = x * x;

        let p = x
            * (72362614232.0
                + y * (-7895059235.0
                    + y * (242396853.1
                        + y * (-2972611.439
                            + y * (15704.48260 + y * (-30.16036606))))));

        let q = 144725228442.0
            + y * (2300535178.0
                + y * (18583304.74
                    + y * (99447.43394 + y * (376.9991397 + y))));

        p / q
    } else {
        // Asymptotic expansion for |x| > 8
        // J_1(x) ~ sqrt(2/(pi*x)) * [P_1(x)*cos(x - 3pi/4) - Q_1(x)*sin(x - 3pi/4)]
        let z = 8.0 / ax;
        let y = z * z;
        let xx = ax - 2.356194491; // 3*pi/4

        let p = 1.0
            + y * (0.183105e-2
                + y * (-0.3516396496e-4
                    + y * (0.2457520174e-5 + y * (-0.240337019e-6))));

        let q = 0.04687499995
            + y * (-0.2002690873e-3
                + y * (0.8449199096e-5
                    + y * (-0.88228987e-6 + y * 0.105787412e-6)));

        let result = (0.636619772 / ax).sqrt() * (p * xx.cos() - z * q * xx.sin());

        if x < 0.0 {
            -result
        } else {
            result
        }
    }
}

/// Compute the Bessel function of the first kind, order 0: J_0(x).
///
/// Uses rational polynomial approximations (Abramowitz & Stegun).
pub fn bessel_j0(x: f64) -> f64 {
    let ax = x.abs();

    if ax <= 8.0 {
        let y = x * x;

        let p = 57568490574.0
            + y * (-13362590354.0
                + y * (651619640.7
                    + y * (-11214424.18 + y * (77392.33017 + y * (-184.9052456)))));

        let q = 57568490411.0
            + y * (1029532985.0
                + y * (9494680.718
                    + y * (59272.64853 + y * (267.8532712 + y))));

        p / q
    } else {
        let z = 8.0 / ax;
        let y = z * z;
        let xx = ax - 0.785398164; // pi/4

        let p = 1.0
            + y * (-0.1098628627e-2
                + y * (0.2734510407e-4
                    + y * (-0.2073370639e-5 + y * 0.2093887211e-6)));

        let q = -0.1562499995e-1
            + y * (0.1430488765e-3
                + y * (-0.6911147651e-5
                    + y * (0.7621095161e-6 + y * (-0.934935152e-7))));

        (0.636619772 / ax).sqrt() * (p * xx.cos() - z * q * xx.sin())
    }
}

// ============================================================================
// Airy Pattern
// ============================================================================

/// Compute the normalized Airy intensity pattern: [2*J_1(x)/x]^2.
///
/// This is the far-field diffraction pattern from a circular aperture.
/// At x = 0 the function equals 1.0 (the central maximum).
///
/// # Arguments
///
/// * `x` - Dimensionless parameter: x = pi * d * sin(theta) / lambda
///
/// # Returns
///
/// The normalized intensity in range [0, 1]
pub fn airy_pattern(x: f64) -> f64 {
    if x.abs() < 1e-12 {
        return 1.0;
    }
    let jinc = 2.0 * bessel_j1(x) / x;
    jinc * jinc
}

/// Compute the Fraunhofer diffraction angle (radians) to the first minimum
/// of the Airy pattern for a circular particle.
///
/// The first zero of J_1(x) is at x = 3.8317..., giving:
///
/// ```text
///   sin(theta) = 1.22 * lambda / d
/// ```
///
/// # Arguments
///
/// * `wavelength` - Laser wavelength in same units as `diameter`
/// * `diameter` - Particle diameter in same units as `wavelength`
///
/// # Returns
///
/// The angle in radians to the first diffraction minimum
pub fn fraunhofer_angle(wavelength: f64, diameter: f64) -> f64 {
    assert!(diameter > 0.0, "diameter must be positive");
    assert!(wavelength > 0.0, "wavelength must be positive");
    // sin(theta) = 1.22 * lambda / d
    let sin_theta = 1.22 * wavelength / diameter;
    if sin_theta >= 1.0 {
        PI / 2.0
    } else {
        sin_theta.asin()
    }
}

// ============================================================================
// Fraunhofer Diffraction
// ============================================================================

/// Fraunhofer diffraction calculator for a circular aperture (large particles).
///
/// Models the diffraction pattern I(theta) = I_0 * [2*J_1(x)/x]^2 where
/// x = pi * d * sin(theta) / lambda.
#[derive(Debug, Clone)]
pub struct FraunhoferDiffraction {
    /// Laser wavelength in micrometers
    pub wavelength_um: f64,
}

impl FraunhoferDiffraction {
    /// Create a new Fraunhofer diffraction calculator.
    ///
    /// # Arguments
    ///
    /// * `wavelength_um` - Laser wavelength in micrometers
    pub fn new(wavelength_um: f64) -> Self {
        assert!(wavelength_um > 0.0, "wavelength must be positive");
        Self { wavelength_um }
    }

    /// Create with He-Ne laser (632.8 nm).
    pub fn hene() -> Self {
        Self::new(HENE_WAVELENGTH_UM)
    }

    /// Compute the normalized intensity at angle theta for a given particle diameter.
    ///
    /// # Arguments
    ///
    /// * `diameter_um` - Particle diameter in micrometers
    /// * `theta_rad` - Scattering angle in radians
    ///
    /// # Returns
    ///
    /// Normalized intensity [0, 1]
    pub fn intensity(&self, diameter_um: f64, theta_rad: f64) -> f64 {
        let x = PI * diameter_um * theta_rad.sin() / self.wavelength_um;
        airy_pattern(x)
    }

    /// Compute intensity at multiple angles.
    ///
    /// # Arguments
    ///
    /// * `diameter_um` - Particle diameter in micrometers
    /// * `angles_rad` - Array of scattering angles in radians
    ///
    /// # Returns
    ///
    /// Vector of normalized intensities
    pub fn intensity_pattern(&self, diameter_um: f64, angles_rad: &[f64]) -> Vec<f64> {
        angles_rad
            .iter()
            .map(|&theta| self.intensity(diameter_um, theta))
            .collect()
    }

    /// Compute the diffraction angle to the first minimum.
    ///
    /// # Arguments
    ///
    /// * `diameter_um` - Particle diameter in micrometers
    pub fn first_minimum_angle(&self, diameter_um: f64) -> f64 {
        fraunhofer_angle(self.wavelength_um, diameter_um)
    }

    /// Compute the size parameter alpha = pi * d / lambda.
    ///
    /// The Fraunhofer approximation is valid when alpha >> 1 (typically alpha > 40).
    pub fn size_parameter(&self, diameter_um: f64) -> f64 {
        PI * diameter_um / self.wavelength_um
    }

    /// Check whether Fraunhofer approximation is valid for the given particle size.
    ///
    /// Typically valid when alpha = pi*d/lambda > 40.
    pub fn is_valid_approximation(&self, diameter_um: f64) -> bool {
        self.size_parameter(diameter_um) > 40.0
    }

    /// Compute forward-scattering cross-section (Fraunhofer limit).
    ///
    /// In the Fraunhofer regime, Q_ext = 2 (extinction paradox), so:
    ///   C_ext = 2 * pi * (d/2)^2
    pub fn extinction_cross_section(&self, diameter_um: f64) -> f64 {
        let r = diameter_um / 2.0;
        2.0 * PI * r * r
    }
}

// ============================================================================
// Mie Scattering (Simplified)
// ============================================================================

/// Simplified Mie scattering approximation for computing efficiency factors.
///
/// Provides Q_ext (extinction efficiency), Q_sca (scattering efficiency),
/// and Q_abs (absorption efficiency) for spherical particles using
/// van de Hulst's anomalous diffraction approximation.
#[derive(Debug, Clone)]
pub struct MieScatteringApprox {
    /// Laser wavelength in micrometers
    pub wavelength_um: f64,
    /// Real part of particle refractive index relative to medium
    pub n_real: f64,
    /// Imaginary part of particle refractive index (absorption)
    pub n_imag: f64,
    /// Refractive index of the surrounding medium
    pub n_medium: f64,
}

impl MieScatteringApprox {
    /// Create a new Mie scattering calculator.
    ///
    /// # Arguments
    ///
    /// * `wavelength_um` - Laser wavelength in micrometers
    /// * `n_real` - Real part of particle refractive index
    /// * `n_imag` - Imaginary part (absorption), typically >= 0
    /// * `n_medium` - Refractive index of surrounding medium (e.g. 1.33 for water)
    pub fn new(wavelength_um: f64, n_real: f64, n_imag: f64, n_medium: f64) -> Self {
        Self {
            wavelength_um,
            n_real,
            n_imag,
            n_medium,
        }
    }

    /// Create for glass particles (n=1.5) in water with He-Ne laser.
    pub fn glass_in_water() -> Self {
        Self::new(HENE_WAVELENGTH_UM, 1.5, 0.0, REFRACTIVE_INDEX_WATER)
    }

    /// Create for glass particles in air with He-Ne laser.
    pub fn glass_in_air() -> Self {
        Self::new(HENE_WAVELENGTH_UM, 1.5, 0.0, 1.0)
    }

    /// Compute the size parameter x = pi * d * n_medium / lambda.
    pub fn size_parameter(&self, diameter_um: f64) -> f64 {
        PI * diameter_um * self.n_medium / self.wavelength_um
    }

    /// Relative refractive index m = n_particle / n_medium.
    pub fn relative_index(&self) -> f64 {
        self.n_real / self.n_medium
    }

    /// Phase delay parameter rho = 2 * x * (m - 1), where m is relative index.
    ///
    /// This is the phase shift experienced by a ray passing through the
    /// center of the sphere.
    pub fn phase_delay(&self, diameter_um: f64) -> f64 {
        let x = self.size_parameter(diameter_um);
        let m = self.relative_index();
        2.0 * x * (m - 1.0)
    }

    /// Compute extinction efficiency Q_ext using van de Hulst's anomalous
    /// diffraction approximation:
    ///
    /// Q_ext = 2 - (4/rho)*sin(rho) + (4/rho^2)*(1 - cos(rho))
    ///
    /// Valid for large, transparent particles (x >> 1, |m-1| << 1).
    pub fn q_ext(&self, diameter_um: f64) -> f64 {
        let rho = self.phase_delay(diameter_um);
        if rho.abs() < 1e-12 {
            return 0.0;
        }
        2.0 - (4.0 / rho) * rho.sin() + (4.0 / (rho * rho)) * (1.0 - rho.cos())
    }

    /// Compute absorption efficiency Q_abs for absorbing particles.
    ///
    /// Uses the Beer-Lambert absorption through the sphere:
    /// Q_abs = 1 - exp(-4 * pi * n_imag * d / lambda)
    ///
    /// For non-absorbing particles (n_imag = 0), returns 0.
    pub fn q_abs(&self, diameter_um: f64) -> f64 {
        if self.n_imag.abs() < 1e-15 {
            return 0.0;
        }
        let tau = 4.0 * PI * self.n_imag * diameter_um / self.wavelength_um;
        1.0 - (-tau).exp()
    }

    /// Compute scattering efficiency Q_sca = Q_ext - Q_abs.
    pub fn q_sca(&self, diameter_um: f64) -> f64 {
        let ext = self.q_ext(diameter_um);
        let abs = self.q_abs(diameter_um);
        (ext - abs).max(0.0)
    }

    /// Compute extinction cross-section C_ext = Q_ext * pi * (d/2)^2.
    pub fn extinction_cross_section(&self, diameter_um: f64) -> f64 {
        let r = diameter_um / 2.0;
        self.q_ext(diameter_um) * PI * r * r
    }

    /// Compute scattering cross-section C_sca = Q_sca * pi * (d/2)^2.
    pub fn scattering_cross_section(&self, diameter_um: f64) -> f64 {
        let r = diameter_um / 2.0;
        self.q_sca(diameter_um) * PI * r * r
    }
}

// ============================================================================
// Particle Size Distribution
// ============================================================================

/// Volume-weighted particle size distribution with bin edges and fractions.
///
/// Bin edges define the boundaries of each size class. For N bins there are
/// N+1 edges. Each fraction represents the volume (or mass for uniform density)
/// fraction of particles in that bin.
#[derive(Debug, Clone)]
pub struct ParticleSizeDistribution {
    /// Bin edges in micrometers (N+1 values, monotonically increasing)
    pub bin_edges: Vec<f64>,
    /// Volume fractions for each bin (N values, sum to ~1.0)
    pub fractions: Vec<f64>,
}

impl ParticleSizeDistribution {
    /// Create a new particle size distribution.
    ///
    /// # Arguments
    ///
    /// * `bin_edges` - Monotonically increasing bin boundaries (N+1 values)
    /// * `fractions` - Volume fractions for each bin (N values)
    ///
    /// # Returns
    ///
    /// `Ok(ParticleSizeDistribution)` or `Err` with description
    pub fn new(bin_edges: &[f64], fractions: &[f64]) -> Result<Self, &'static str> {
        if bin_edges.len() < 2 {
            return Err("need at least 2 bin edges");
        }
        if fractions.len() != bin_edges.len() - 1 {
            return Err("fractions length must equal bin_edges length - 1");
        }
        // Check monotonically increasing
        for w in bin_edges.windows(2) {
            if w[1] <= w[0] {
                return Err("bin edges must be monotonically increasing");
            }
        }
        // Check non-negative fractions
        for &f in fractions {
            if f < 0.0 {
                return Err("fractions must be non-negative");
            }
        }
        Ok(Self {
            bin_edges: bin_edges.to_vec(),
            fractions: fractions.to_vec(),
        })
    }

    /// Number of bins.
    pub fn num_bins(&self) -> usize {
        self.fractions.len()
    }

    /// Get the geometric mean diameter of a bin (sqrt of product of edges).
    pub fn bin_center_geometric(&self, bin: usize) -> f64 {
        (self.bin_edges[bin] * self.bin_edges[bin + 1]).sqrt()
    }

    /// Get the arithmetic mean diameter of a bin.
    pub fn bin_center_arithmetic(&self, bin: usize) -> f64 {
        (self.bin_edges[bin] + self.bin_edges[bin + 1]) / 2.0
    }

    /// Get all geometric bin centers.
    pub fn bin_centers_geometric(&self) -> Vec<f64> {
        (0..self.num_bins())
            .map(|i| self.bin_center_geometric(i))
            .collect()
    }

    /// Get all arithmetic bin centers.
    pub fn bin_centers_arithmetic(&self) -> Vec<f64> {
        (0..self.num_bins())
            .map(|i| self.bin_center_arithmetic(i))
            .collect()
    }

    /// Compute the cumulative undersize distribution.
    ///
    /// Returns a vector of cumulative volume fractions at each bin edge.
    /// The first value is 0.0, the last is the total sum (ideally 1.0).
    pub fn cumulative_undersize(&self) -> Vec<f64> {
        let mut cum = Vec::with_capacity(self.bin_edges.len());
        cum.push(0.0);
        let mut sum = 0.0;
        for &f in &self.fractions {
            sum += f;
            cum.push(sum);
        }
        cum
    }

    /// Normalize fractions so they sum to exactly 1.0.
    pub fn normalize(&mut self) {
        let total: f64 = self.fractions.iter().sum();
        if total > 0.0 {
            for f in &mut self.fractions {
                *f /= total;
            }
        }
    }

    /// Total volume fraction (should be ~1.0 for a normalized distribution).
    pub fn total_fraction(&self) -> f64 {
        self.fractions.iter().sum()
    }

    /// Create a log-normal distribution with specified parameters.
    ///
    /// # Arguments
    ///
    /// * `median_um` - Median diameter (D50) in micrometers
    /// * `geo_std_dev` - Geometric standard deviation (> 1.0)
    /// * `min_um` - Minimum bin edge in micrometers
    /// * `max_um` - Maximum bin edge in micrometers
    /// * `num_bins` - Number of bins
    pub fn from_log_normal(
        median_um: f64,
        geo_std_dev: f64,
        min_um: f64,
        max_um: f64,
        num_bins: usize,
    ) -> Self {
        assert!(median_um > 0.0);
        assert!(geo_std_dev > 1.0);
        assert!(min_um > 0.0 && max_um > min_um);
        assert!(num_bins >= 1);

        let log_min = min_um.ln();
        let log_max = max_um.ln();
        let step = (log_max - log_min) / num_bins as f64;

        let mut bin_edges = Vec::with_capacity(num_bins + 1);
        for i in 0..=num_bins {
            bin_edges.push((log_min + i as f64 * step).exp());
        }

        let mu = median_um.ln();
        let sigma = geo_std_dev.ln();

        let mut fractions = Vec::with_capacity(num_bins);
        for i in 0..num_bins {
            let center = (bin_edges[i] * bin_edges[i + 1]).sqrt();
            let log_c = center.ln();
            let exponent = -(log_c - mu).powi(2) / (2.0 * sigma * sigma);
            let pdf = exponent.exp() / (center * sigma * (2.0 * PI).sqrt());
            let width = bin_edges[i + 1] - bin_edges[i];
            fractions.push(pdf * width);
        }

        // Normalize
        let total: f64 = fractions.iter().sum();
        if total > 0.0 {
            for f in &mut fractions {
                *f /= total;
            }
        }

        Self {
            bin_edges,
            fractions,
        }
    }

    /// Create a Rosin-Rammler distribution with specified parameters.
    ///
    /// # Arguments
    ///
    /// * `x_prime` - Characteristic size (63.2% cumulative passing) in micrometers
    /// * `n` - Spread parameter (typically 0.5–5.0)
    /// * `min_um` - Minimum bin edge in micrometers
    /// * `max_um` - Maximum bin edge in micrometers
    /// * `num_bins` - Number of bins
    pub fn from_rosin_rammler(
        x_prime: f64,
        n: f64,
        min_um: f64,
        max_um: f64,
        num_bins: usize,
    ) -> Self {
        assert!(x_prime > 0.0);
        assert!(n > 0.0);
        assert!(min_um > 0.0 && max_um > min_um);
        assert!(num_bins >= 1);

        let log_min = min_um.ln();
        let log_max = max_um.ln();
        let step = (log_max - log_min) / num_bins as f64;

        let mut bin_edges = Vec::with_capacity(num_bins + 1);
        for i in 0..=num_bins {
            bin_edges.push((log_min + i as f64 * step).exp());
        }

        // Q(x) = 1 - exp(-(x/x')^n), so fraction in bin [a,b] = Q(b) - Q(a)
        let rr_cumulative = |x: f64| -> f64 { 1.0 - (-(x / x_prime).powf(n)).exp() };

        let mut fractions = Vec::with_capacity(num_bins);
        for i in 0..num_bins {
            let f = rr_cumulative(bin_edges[i + 1]) - rr_cumulative(bin_edges[i]);
            fractions.push(f.max(0.0));
        }

        // Normalize
        let total: f64 = fractions.iter().sum();
        if total > 0.0 {
            for f in &mut fractions {
                *f /= total;
            }
        }

        Self {
            bin_edges,
            fractions,
        }
    }
}

/// Compute the cumulative undersize distribution from bin edges and fractions.
///
/// Returns cumulative fractions at each bin edge (N+1 values). The first
/// value is 0.0 and the last is the total sum.
pub fn cumulative_undersize(bin_edges: &[f64], fractions: &[f64]) -> Vec<f64> {
    assert_eq!(
        fractions.len(),
        bin_edges.len() - 1,
        "fractions length must equal bin_edges length - 1"
    );
    let mut cum = Vec::with_capacity(bin_edges.len());
    cum.push(0.0);
    let mut sum = 0.0;
    for &f in fractions {
        sum += f;
        cum.push(sum);
    }
    cum
}

// ============================================================================
// Log-Normal Distribution Fit
// ============================================================================

/// Log-normal distribution fit to particle size data.
///
/// The log-normal PDF is:
///
/// ```text
///   f(x) = 1 / (x * sigma * sqrt(2*pi)) * exp(-(ln(x) - mu)^2 / (2*sigma^2))
/// ```
///
/// where mu = ln(median) and sigma = ln(geometric_std_dev).
#[derive(Debug, Clone)]
pub struct LogNormalFit {
    /// Median diameter (D50) in micrometers
    pub median: f64,
    /// Geometric standard deviation (> 1.0)
    pub geo_std_dev: f64,
    /// Log-space mean mu = ln(median)
    pub mu: f64,
    /// Log-space std dev sigma = ln(geo_std_dev)
    pub sigma: f64,
    /// Goodness of fit R^2 (0 to 1)
    pub r_squared: f64,
}

impl LogNormalFit {
    /// Fit a log-normal distribution to a PSD using method of moments.
    ///
    /// Computes the mean and variance of ln(d) weighted by volume fractions.
    pub fn fit(psd: &ParticleSizeDistribution) -> Self {
        let n = psd.num_bins();
        let total: f64 = psd.fractions.iter().sum();

        // Weighted mean and variance of ln(d)
        let mut mu = 0.0;
        let mut var = 0.0;

        for i in 0..n {
            let center = psd.bin_center_geometric(i);
            let w = psd.fractions[i] / total;
            mu += w * center.ln();
        }

        for i in 0..n {
            let center = psd.bin_center_geometric(i);
            let w = psd.fractions[i] / total;
            let diff = center.ln() - mu;
            var += w * diff * diff;
        }

        let sigma = var.sqrt();
        let median = mu.exp();
        let geo_std_dev = sigma.exp();

        // Compute R^2 (goodness of fit)
        let r_squared = Self::compute_r_squared(psd, mu, sigma, total);

        Self {
            median,
            geo_std_dev,
            mu,
            sigma,
            r_squared,
        }
    }

    fn compute_r_squared(
        psd: &ParticleSizeDistribution,
        mu: f64,
        sigma: f64,
        total: f64,
    ) -> f64 {
        let n = psd.num_bins();
        if n == 0 || sigma < 1e-15 {
            return 0.0;
        }

        let mean_frac = total / n as f64;
        let mut ss_res = 0.0;
        let mut ss_tot = 0.0;

        for i in 0..n {
            let center = psd.bin_center_geometric(i);
            let width = psd.bin_edges[i + 1] - psd.bin_edges[i];
            let log_c = center.ln();
            let exponent = -(log_c - mu).powi(2) / (2.0 * sigma * sigma);
            let predicted = total * exponent.exp() / (center * sigma * (2.0 * PI).sqrt()) * width;

            let observed = psd.fractions[i];
            ss_res += (observed - predicted).powi(2);
            ss_tot += (observed - mean_frac).powi(2);
        }

        if ss_tot < 1e-15 {
            return 0.0;
        }

        1.0 - ss_res / ss_tot
    }

    /// Evaluate the PDF at a given diameter.
    pub fn pdf(&self, diameter_um: f64) -> f64 {
        if diameter_um <= 0.0 || self.sigma < 1e-15 {
            return 0.0;
        }
        let log_d = diameter_um.ln();
        let exponent = -(log_d - self.mu).powi(2) / (2.0 * self.sigma * self.sigma);
        exponent.exp() / (diameter_um * self.sigma * (2.0 * PI).sqrt())
    }

    /// Evaluate the CDF at a given diameter.
    ///
    /// Uses the error function approximation.
    pub fn cdf(&self, diameter_um: f64) -> f64 {
        if diameter_um <= 0.0 {
            return 0.0;
        }
        if self.sigma < 1e-15 {
            return if diameter_um >= self.median { 1.0 } else { 0.0 };
        }
        let z = (diameter_um.ln() - self.mu) / (self.sigma * 2.0_f64.sqrt());
        0.5 * (1.0 + erf_approx(z))
    }
}

/// Approximation of the error function erf(x) using Horner's method
/// (Abramowitz & Stegun, formula 7.1.26, max error 1.5e-7).
fn erf_approx(x: f64) -> f64 {
    let sign = if x >= 0.0 { 1.0 } else { -1.0 };
    let x = x.abs();

    let p = 0.3275911;
    let a1 = 0.254829592;
    let a2 = -0.284496736;
    let a3 = 1.421413741;
    let a4 = -1.453152027;
    let a5 = 1.061405429;

    let t = 1.0 / (1.0 + p * x);
    let poly = t * (a1 + t * (a2 + t * (a3 + t * (a4 + t * a5))));
    let result = 1.0 - poly * (-x * x).exp();
    sign * result
}

// ============================================================================
// Rosin-Rammler Distribution Fit
// ============================================================================

/// Rosin-Rammler (Weibull) distribution fit.
///
/// The cumulative oversize distribution is:
///
/// ```text
///   R(x) = exp(-(x/x')^n)
/// ```
///
/// Equivalently, the cumulative undersize is:
///
/// ```text
///   Q(x) = 1 - exp(-(x/x')^n)
/// ```
///
/// where x' is the characteristic size (63.2% passing) and n is the spread parameter.
#[derive(Debug, Clone)]
pub struct RosinRammlerFit {
    /// Characteristic size x' in micrometers (size at which 63.2% passes)
    pub x_prime: f64,
    /// Spread parameter n (distribution narrowness)
    pub n: f64,
    /// Goodness of fit R^2
    pub r_squared: f64,
}

impl RosinRammlerFit {
    /// Fit a Rosin-Rammler distribution to a PSD using linearized regression.
    ///
    /// Taking double logarithm of R(x) = exp(-(x/x')^n):
    ///
    /// ```text
    ///   ln(ln(1/R)) = n * ln(x) - n * ln(x')
    /// ```
    ///
    /// This is a linear equation y = n*X + c where:
    ///   - X = ln(x)
    ///   - y = ln(ln(1/R))
    ///   - c = -n * ln(x')
    pub fn fit(psd: &ParticleSizeDistribution) -> Self {
        let cum = psd.cumulative_undersize();
        let total = *cum.last().unwrap();
        if total < 1e-15 {
            return Self {
                x_prime: 1.0,
                n: 1.0,
                r_squared: 0.0,
            };
        }

        // Collect points (X, y) where X = ln(edge), y = ln(ln(1/R))
        // R = 1 - Q/total, skip points where R ≈ 0 or 1
        let mut xs = Vec::new();
        let mut ys = Vec::new();

        for i in 1..cum.len() - 1 {
            let q = cum[i] / total;
            if q > 0.01 && q < 0.99 {
                let r = 1.0 - q;
                let x_val = psd.bin_edges[i].ln();
                let y_val = (1.0 / r).ln().ln();
                if y_val.is_finite() {
                    xs.push(x_val);
                    ys.push(y_val);
                }
            }
        }

        if xs.len() < 2 {
            return Self {
                x_prime: psd.bin_center_geometric(psd.num_bins() / 2),
                n: 2.0,
                r_squared: 0.0,
            };
        }

        // Least-squares linear regression: y = slope*x + intercept
        let n_pts = xs.len() as f64;
        let sum_x: f64 = xs.iter().sum();
        let sum_y: f64 = ys.iter().sum();
        let sum_xy: f64 = xs.iter().zip(&ys).map(|(x, y)| x * y).sum();
        let sum_xx: f64 = xs.iter().map(|x| x * x).sum();

        let denom = n_pts * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-15 {
            return Self {
                x_prime: psd.bin_center_geometric(psd.num_bins() / 2),
                n: 2.0,
                r_squared: 0.0,
            };
        }

        let slope = (n_pts * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / n_pts;

        let n = slope.max(0.1); // n must be positive
        let x_prime = (-intercept / n).exp();

        // R^2
        let mean_y = sum_y / n_pts;
        let ss_tot: f64 = ys.iter().map(|y| (y - mean_y).powi(2)).sum();
        let ss_res: f64 = xs
            .iter()
            .zip(&ys)
            .map(|(x, y)| {
                let predicted = slope * x + intercept;
                (y - predicted).powi(2)
            })
            .sum();

        let r_squared = if ss_tot > 1e-15 {
            1.0 - ss_res / ss_tot
        } else {
            0.0
        };

        Self {
            x_prime,
            n,
            r_squared,
        }
    }

    /// Evaluate the cumulative undersize Q(x) at a given diameter.
    pub fn cdf(&self, diameter_um: f64) -> f64 {
        if diameter_um <= 0.0 {
            return 0.0;
        }
        1.0 - (-(diameter_um / self.x_prime).powf(self.n)).exp()
    }

    /// Evaluate the PDF dQ/dx at a given diameter.
    pub fn pdf(&self, diameter_um: f64) -> f64 {
        if diameter_um <= 0.0 {
            return 0.0;
        }
        let t = diameter_um / self.x_prime;
        (self.n / self.x_prime) * t.powf(self.n - 1.0) * (-t.powf(self.n)).exp()
    }

    /// Compute the mean diameter from the Rosin-Rammler distribution.
    ///
    /// mean = x' * Gamma(1 + 1/n)
    pub fn mean_diameter(&self) -> f64 {
        self.x_prime * gamma_approx(1.0 + 1.0 / self.n)
    }
}

/// Stirling's approximation to the Gamma function for x > 0.5.
/// For x <= 0.5, uses the reflection formula Gamma(x) = pi / (sin(pi*x) * Gamma(1-x)).
fn gamma_approx(x: f64) -> f64 {
    if x < 0.5 {
        return PI / ((PI * x).sin() * gamma_approx(1.0 - x));
    }

    // Lanczos approximation (g=7, N=9)
    let g = 7.0_f64;
    let coefficients = [
        0.99999999999980993,
        676.5203681218851,
        -1259.1392167224028,
        771.32342877765313,
        -176.61502916214059,
        12.507343278686905,
        -0.13857109526572012,
        9.9843695780195716e-6,
        1.5056327351493116e-7,
    ];

    let x = x - 1.0;
    let mut sum = coefficients[0];
    for (i, &c) in coefficients[1..].iter().enumerate() {
        sum += c / (x + i as f64 + 1.0);
    }

    let t = x + g + 0.5;
    (2.0 * PI).sqrt() * t.powf(x + 0.5) * (-t).exp() * sum
}

// ============================================================================
// Percentile Calculator
// ============================================================================

/// Extracts percentile diameters (D10, D50, D90, etc.) from a particle
/// size distribution using linear interpolation on the cumulative curve.
#[derive(Debug)]
pub struct PercentileCalculator {
    /// Bin edges in micrometers
    edges: Vec<f64>,
    /// Cumulative volume fractions at each edge (normalized to [0, 1])
    cumulative: Vec<f64>,
}

impl PercentileCalculator {
    /// Create a percentile calculator from a PSD.
    pub fn new(psd: &ParticleSizeDistribution) -> Self {
        let cum = psd.cumulative_undersize();
        let total = *cum.last().unwrap();
        let cumulative = if total > 0.0 {
            cum.iter().map(|c| c / total).collect()
        } else {
            cum
        };
        Self {
            edges: psd.bin_edges.clone(),
            cumulative,
        }
    }

    /// Interpolate the diameter at a given cumulative percentage.
    ///
    /// # Arguments
    ///
    /// * `pct` - The target percentile (0 to 100)
    ///
    /// # Returns
    ///
    /// `Some(diameter_um)` if the percentile falls within the distribution range,
    /// `None` if it falls outside the range.
    pub fn percentile(&self, pct: f64) -> Option<f64> {
        if pct < 0.0 || pct > 100.0 {
            return None;
        }
        let target = pct / 100.0;

        // Find the interval containing the target
        for i in 0..self.cumulative.len() - 1 {
            let c0 = self.cumulative[i];
            let c1 = self.cumulative[i + 1];
            if target >= c0 && target <= c1 {
                if (c1 - c0).abs() < 1e-15 {
                    return Some(self.edges[i]);
                }
                let t = (target - c0) / (c1 - c0);
                // Interpolate in log space for better accuracy
                let log_d0 = self.edges[i].ln();
                let log_d1 = self.edges[i + 1].ln();
                return Some((log_d0 + t * (log_d1 - log_d0)).exp());
            }
        }

        // Extrapolate to edges
        if target <= 0.0 {
            Some(self.edges[0])
        } else {
            Some(*self.edges.last().unwrap())
        }
    }

    /// Compute D10 (10th percentile diameter).
    pub fn d10(&self) -> Option<f64> {
        self.percentile(10.0)
    }

    /// Compute D50 (median diameter).
    pub fn d50(&self) -> Option<f64> {
        self.percentile(50.0)
    }

    /// Compute D90 (90th percentile diameter).
    pub fn d90(&self) -> Option<f64> {
        self.percentile(90.0)
    }

    /// Compute D1 (1st percentile diameter).
    pub fn d1(&self) -> Option<f64> {
        self.percentile(1.0)
    }

    /// Compute D99 (99th percentile diameter).
    pub fn d99(&self) -> Option<f64> {
        self.percentile(99.0)
    }
}

// ============================================================================
// Span Calculator
// ============================================================================

/// Computes the span of a particle size distribution.
///
/// ```text
///   span = (D90 - D10) / D50
/// ```
///
/// A smaller span indicates a narrower (more uniform) distribution.
pub struct SpanCalculator;

impl SpanCalculator {
    /// Compute the span from a PSD.
    ///
    /// Returns `None` if any of D10, D50, D90 cannot be computed or if D50 = 0.
    pub fn compute(psd: &ParticleSizeDistribution) -> Option<f64> {
        let pc = PercentileCalculator::new(psd);
        let d10 = pc.d10()?;
        let d50 = pc.d50()?;
        let d90 = pc.d90()?;

        if d50.abs() < 1e-15 {
            return None;
        }

        Some((d90 - d10) / d50)
    }

    /// Compute span from pre-computed percentile values.
    pub fn from_values(d10: f64, d50: f64, d90: f64) -> Option<f64> {
        if d50.abs() < 1e-15 {
            return None;
        }
        Some((d90 - d10) / d50)
    }
}

// ============================================================================
// Mean Diameters
// ============================================================================

/// Generalized mean diameters D[p,q] computed from a PSD.
///
/// The generalized mean diameter is defined as:
///
/// ```text
///   D[p,q] = (sum(f_i * d_i^p) / sum(f_i * d_i^q))^(1/(p-q))
/// ```
///
/// where f_i is the volume fraction and d_i is the bin center diameter.
///
/// Important special cases:
/// - D[1,0]: Number-length mean diameter
/// - D[3,2]: Sauter mean diameter (surface-area weighted)
/// - D[4,3]: De Brouckere mean diameter (volume/mass weighted)
#[derive(Debug, Clone)]
pub struct MeanDiameters {
    /// D[1,0] — Number-length mean diameter
    pub d_1_0: f64,
    /// D[2,0] — Number-surface mean diameter
    pub d_2_0: f64,
    /// D[3,0] — Number-volume mean diameter
    pub d_3_0: f64,
    /// D[2,1] — Surface-length mean diameter
    pub d_2_1: f64,
    /// D[3,2] — Sauter mean diameter (volume-to-surface ratio)
    pub d_3_2: f64,
    /// D[4,3] — De Brouckere mean diameter (volume/mass moment mean)
    pub d_4_3: f64,
}

impl MeanDiameters {
    /// Compute all generalized mean diameters from a PSD.
    pub fn compute(psd: &ParticleSizeDistribution) -> Self {
        Self {
            d_1_0: Self::dpq(psd, 1.0, 0.0),
            d_2_0: Self::dpq(psd, 2.0, 0.0),
            d_3_0: Self::dpq(psd, 3.0, 0.0),
            d_2_1: Self::dpq(psd, 2.0, 1.0),
            d_3_2: Self::dpq(psd, 3.0, 2.0),
            d_4_3: Self::dpq(psd, 4.0, 3.0),
        }
    }

    /// Compute generalized mean diameter D[p,q].
    ///
    /// ```text
    ///   D[p,q] = (sum(f_i * d_i^p) / sum(f_i * d_i^q))^(1/(p-q))
    /// ```
    pub fn dpq(psd: &ParticleSizeDistribution, p: f64, q: f64) -> f64 {
        let n = psd.num_bins();
        let mut sum_p = 0.0;
        let mut sum_q = 0.0;

        for i in 0..n {
            let d = psd.bin_center_geometric(i);
            let f = psd.fractions[i];
            sum_p += f * d.powf(p);
            sum_q += f * d.powf(q);
        }

        if sum_q.abs() < 1e-30 {
            return 0.0;
        }

        let diff = p - q;
        if diff.abs() < 1e-12 {
            // p == q: degenerate case
            return (sum_p / sum_q).exp();
        }

        (sum_p / sum_q).powf(1.0 / diff)
    }

    /// Compute the Sauter mean diameter D[3,2].
    ///
    /// This is the diameter of a sphere with the same volume/surface-area ratio
    /// as the entire particle population. Widely used in combustion, catalysis,
    /// and mass transfer calculations.
    pub fn sauter(psd: &ParticleSizeDistribution) -> f64 {
        Self::dpq(psd, 3.0, 2.0)
    }

    /// Compute the De Brouckere mean diameter D[4,3].
    ///
    /// This is the volume-moment mean diameter, which is the most sensitive to
    /// large particles in the distribution.
    pub fn de_brouckere(psd: &ParticleSizeDistribution) -> f64 {
        Self::dpq(psd, 4.0, 3.0)
    }
}

// ============================================================================
// Specific Surface Area
// ============================================================================

/// Compute the specific surface area (SSA) of a particle population.
///
/// ```text
///   SSA = 6 / (rho * D[3,2])
/// ```
///
/// where rho is the particle density (g/cm^3 or kg/m^3) and D[3,2] is
/// the Sauter mean diameter.
///
/// # Arguments
///
/// * `d32_um` - Sauter mean diameter in micrometers
/// * `density` - Particle density in the same mass/volume units desired for SSA
///
/// # Returns
///
/// Specific surface area. If density is in g/cm^3 and d32 in um,
/// result is in m^2/cm^3 (multiply by 1e-6 to get m^2/g... actually:
/// SSA = 6 / (rho * d32) with consistent units).
///
/// For d32 in um and density in g/cm^3:
///   SSA = 6 / (density_g_cm3 * d32_um * 1e-4) in cm^2/cm^3 = cm^-1
///   SSA = 6 / (density_g_cm3 * d32_um * 1e-4) * 1e-4 in m^2/cm^3
///
/// This function returns SSA in consistent units: if d32 and density are
/// in SI, result is m^-1.
pub fn specific_surface_area(d32: f64, density: f64) -> f64 {
    assert!(d32 > 0.0, "Sauter mean diameter must be positive");
    assert!(density > 0.0, "density must be positive");
    6.0 / (density * d32)
}

// ============================================================================
// Obscuration (Beer-Lambert)
// ============================================================================

/// Compute the obscuration (transmission loss) of a laser beam passing through
/// a particle dispersion, based on the Beer-Lambert law.
///
/// ```text
///   T = exp(-c * L * K_ext)
///   obscuration % = (1 - T) * 100
/// ```
///
/// where:
/// - c = particle concentration (volume fraction or number density * cross-section)
/// - L = optical path length
/// - K_ext = extinction coefficient
///
/// # Arguments
///
/// * `concentration` - Effective concentration parameter (dimensionless or per unit length)
/// * `path_length` - Optical path length through the sample
/// * `extinction` - Extinction coefficient / efficiency
///
/// # Returns
///
/// Obscuration as a percentage (0–100)
pub fn obscuration(concentration: f64, path_length: f64, extinction: f64) -> f64 {
    let tau = concentration * path_length * extinction;
    (1.0 - (-tau).exp()) * 100.0
}

/// Check if the obscuration is within the ISO 13320 recommended range (5–25%).
pub fn is_obscuration_optimal(obs_pct: f64) -> bool {
    obs_pct >= MIN_OBSCURATION_PCT && obs_pct <= MAX_OBSCURATION_PCT
}

/// Compute the concentration needed to achieve a target obscuration.
///
/// From: T = exp(-c * L * K), so c = -ln(1 - obs/100) / (L * K)
pub fn concentration_for_obscuration(
    target_obs_pct: f64,
    path_length: f64,
    extinction: f64,
) -> f64 {
    assert!(
        target_obs_pct > 0.0 && target_obs_pct < 100.0,
        "target obscuration must be between 0 and 100"
    );
    assert!(path_length > 0.0, "path length must be positive");
    assert!(extinction > 0.0, "extinction must be positive");
    -(1.0 - target_obs_pct / 100.0).ln() / (path_length * extinction)
}

// ============================================================================
// ISO 13320 Compliance
// ============================================================================

/// ISO 13320 compliance metrics for laser diffraction measurement.
#[derive(Debug, Clone)]
pub struct Iso13320Compliance {
    /// Obscuration percentage
    pub obscuration_pct: f64,
    /// Whether obscuration is in the recommended range (5–25%)
    pub obscuration_ok: bool,
    /// Residual fit error (weighted least-squares residual)
    pub residual: f64,
    /// Whether the residual is acceptable (typically < 1%)
    pub residual_ok: bool,
    /// Whether the distribution is within the measurement range
    pub range_ok: bool,
    /// Distribution span
    pub span: Option<f64>,
    /// D50 in micrometers
    pub d50: Option<f64>,
    /// Overall compliance
    pub compliant: bool,
}

impl Iso13320Compliance {
    /// Evaluate ISO 13320 compliance for a measurement.
    ///
    /// # Arguments
    ///
    /// * `psd` - The measured particle size distribution
    /// * `obscuration_pct` - Measured obscuration percentage
    /// * `residual` - Fitting residual (e.g., from inversion algorithm)
    /// * `instrument_min_um` - Instrument minimum measurable size in um
    /// * `instrument_max_um` - Instrument maximum measurable size in um
    pub fn evaluate(
        psd: &ParticleSizeDistribution,
        obscuration_pct: f64,
        residual: f64,
        instrument_min_um: f64,
        instrument_max_um: f64,
    ) -> Self {
        let obscuration_ok = is_obscuration_optimal(obscuration_pct);
        let residual_ok = residual < 1.0; // < 1% considered acceptable

        // Check if distribution falls within instrument range
        let pc = PercentileCalculator::new(psd);
        let d10 = pc.d10();
        let d90 = pc.d90();
        let d50 = pc.d50();
        let span = SpanCalculator::compute(psd);

        let range_ok = match (d10, d90) {
            (Some(d10v), Some(d90v)) => d10v >= instrument_min_um && d90v <= instrument_max_um,
            _ => false,
        };

        let compliant = obscuration_ok && residual_ok && range_ok;

        Self {
            obscuration_pct,
            obscuration_ok,
            residual,
            residual_ok,
            range_ok,
            span,
            d50,
            compliant,
        }
    }
}

// ============================================================================
// Detector Geometry
// ============================================================================

/// Represents the angular positions of detectors in a laser diffraction instrument.
#[derive(Debug, Clone)]
pub struct DetectorGeometry {
    /// Inner angle of each detector element (radians)
    pub inner_angles: Vec<f64>,
    /// Outer angle of each detector element (radians)
    pub outer_angles: Vec<f64>,
}

impl DetectorGeometry {
    /// Create a detector geometry with logarithmically spaced angles.
    ///
    /// # Arguments
    ///
    /// * `num_detectors` - Number of detector elements
    /// * `min_angle_deg` - Minimum scattering angle in degrees
    /// * `max_angle_deg` - Maximum scattering angle in degrees
    pub fn log_spaced(num_detectors: usize, min_angle_deg: f64, max_angle_deg: f64) -> Self {
        assert!(num_detectors >= 1);
        assert!(min_angle_deg > 0.0 && max_angle_deg > min_angle_deg);

        let log_min = min_angle_deg.ln();
        let log_max = max_angle_deg.ln();
        let step = (log_max - log_min) / num_detectors as f64;

        let mut inner_angles = Vec::with_capacity(num_detectors);
        let mut outer_angles = Vec::with_capacity(num_detectors);

        for i in 0..num_detectors {
            let inner = (log_min + i as f64 * step).exp() * PI / 180.0;
            let outer = (log_min + (i + 1) as f64 * step).exp() * PI / 180.0;
            inner_angles.push(inner);
            outer_angles.push(outer);
        }

        Self {
            inner_angles,
            outer_angles,
        }
    }

    /// Number of detector elements.
    pub fn num_detectors(&self) -> usize {
        self.inner_angles.len()
    }

    /// Get the center angle of a detector element (geometric mean).
    pub fn center_angle(&self, index: usize) -> f64 {
        (self.inner_angles[index] * self.outer_angles[index]).sqrt()
    }

    /// Compute the expected light energy on each detector for a given particle size.
    ///
    /// Uses Fraunhofer diffraction (Airy pattern integration over each detector element).
    pub fn fraunhofer_energy(
        &self,
        fraunhofer: &FraunhoferDiffraction,
        diameter_um: f64,
        num_integration_points: usize,
    ) -> Vec<f64> {
        let mut energies = Vec::with_capacity(self.num_detectors());

        for det in 0..self.num_detectors() {
            let inner = self.inner_angles[det];
            let outer = self.outer_angles[det];
            let d_theta = (outer - inner) / num_integration_points as f64;

            let mut energy = 0.0;
            for j in 0..num_integration_points {
                let theta = inner + (j as f64 + 0.5) * d_theta;
                let intensity = fraunhofer.intensity(diameter_um, theta);
                // Weight by sin(theta) for solid angle on ring detector
                energy += intensity * theta.sin() * d_theta;
            }
            energies.push(energy);
        }

        energies
    }
}

// ============================================================================
// Forward Model (Scattering Matrix)
// ============================================================================

/// Builds the scattering matrix that relates particle size distribution to
/// detector signals, enabling PSD inversion.
pub struct ScatteringMatrix;

impl ScatteringMatrix {
    /// Build a Fraunhofer scattering matrix.
    ///
    /// Each column corresponds to a particle size bin, each row to a detector.
    /// Element (i,j) is the expected energy on detector i from unit volume
    /// of particles in size bin j.
    ///
    /// # Returns
    ///
    /// Matrix stored as `Vec<Vec<f64>>` with dimensions [num_detectors x num_bins].
    pub fn build_fraunhofer(
        detectors: &DetectorGeometry,
        psd_bin_edges: &[f64],
        wavelength_um: f64,
        integration_points: usize,
    ) -> Vec<Vec<f64>> {
        let fraunhofer = FraunhoferDiffraction::new(wavelength_um);
        let num_bins = psd_bin_edges.len() - 1;
        let num_dets = detectors.num_detectors();

        let mut matrix = vec![vec![0.0; num_bins]; num_dets];

        for j in 0..num_bins {
            let d = (psd_bin_edges[j] * psd_bin_edges[j + 1]).sqrt();
            let energies = detectors.fraunhofer_energy(&fraunhofer, d, integration_points);
            for i in 0..num_dets {
                matrix[i][j] = energies[i];
            }
        }

        matrix
    }
}

// ============================================================================
// Uniformity / Modality
// ============================================================================

/// Analyze the modality (number of peaks) of a PSD.
pub fn count_modes(psd: &ParticleSizeDistribution) -> usize {
    let fracs = &psd.fractions;
    let n = fracs.len();
    if n < 3 {
        return 1;
    }

    let mut modes = 0;
    for i in 1..n - 1 {
        if fracs[i] > fracs[i - 1] && fracs[i] > fracs[i + 1] {
            modes += 1;
        }
    }
    // Check endpoints
    if fracs[0] > fracs[1] {
        modes += 1;
    }
    if fracs[n - 1] > fracs[n - 2] {
        modes += 1;
    }

    modes.max(1) // At least 1 mode
}

/// Compute the uniformity coefficient Cu = D60/D10 (used in soil science).
pub fn uniformity_coefficient(psd: &ParticleSizeDistribution) -> Option<f64> {
    let pc = PercentileCalculator::new(psd);
    let d10 = pc.d10()?;
    let d60 = pc.percentile(60.0)?;
    if d10.abs() < 1e-15 {
        return None;
    }
    Some(d60 / d10)
}

/// Compute the coefficient of gradation Cc = D30^2 / (D10 * D60).
pub fn coefficient_of_gradation(psd: &ParticleSizeDistribution) -> Option<f64> {
    let pc = PercentileCalculator::new(psd);
    let d10 = pc.d10()?;
    let d30 = pc.percentile(30.0)?;
    let d60 = pc.percentile(60.0)?;
    let denom = d10 * d60;
    if denom.abs() < 1e-15 {
        return None;
    }
    Some(d30 * d30 / denom)
}

// ============================================================================
// Signal Processing Helpers
// ============================================================================

/// Compute the theoretical diffraction pattern for a polydisperse sample.
///
/// Sums the Airy pattern contributions from each size bin weighted by
/// volume fraction.
pub fn polydisperse_pattern(
    psd: &ParticleSizeDistribution,
    wavelength_um: f64,
    angles_rad: &[f64],
) -> Vec<f64> {
    let fraunhofer = FraunhoferDiffraction::new(wavelength_um);
    let n_angles = angles_rad.len();
    let mut pattern = vec![0.0; n_angles];

    for i in 0..psd.num_bins() {
        let d = psd.bin_center_geometric(i);
        let weight = psd.fractions[i];
        // Cross-section proportional to d^2
        let cs_weight = weight * d * d;

        for (j, &theta) in angles_rad.iter().enumerate() {
            pattern[j] += cs_weight * fraunhofer.intensity(d, theta);
        }
    }

    pattern
}

/// Simple moving average smoothing for detector signal preprocessing.
pub fn smooth_signal(signal: &[f64], window: usize) -> Vec<f64> {
    if window <= 1 || signal.is_empty() {
        return signal.to_vec();
    }

    let n = signal.len();
    let half = window / 2;
    let mut smoothed = Vec::with_capacity(n);

    for i in 0..n {
        let start = if i >= half { i - half } else { 0 };
        let end = (i + half + 1).min(n);
        let sum: f64 = signal[start..end].iter().sum();
        let count = (end - start) as f64;
        smoothed.push(sum / count);
    }

    smoothed
}

/// Compute the scattering vector magnitude q for a given angle and wavelength.
///
/// ```text
///   q = (4 * pi * n_medium / lambda) * sin(theta / 2)
/// ```
///
/// Used in Mie theory and small-angle scattering (SAS) analysis.
pub fn scattering_vector(wavelength_um: f64, n_medium: f64, theta_rad: f64) -> f64 {
    (4.0 * PI * n_medium / wavelength_um) * (theta_rad / 2.0).sin()
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;
    const LOOSE_EPSILON: f64 = 1e-3;

    // --- Bessel J1 tests ---

    #[test]
    fn test_bessel_j1_at_zero() {
        assert!((bessel_j1(0.0)).abs() < EPSILON);
    }

    #[test]
    fn test_bessel_j1_near_zero() {
        // J_1(x) ~ x/2 for small x
        let x = 1e-10;
        assert!((bessel_j1(x) - x / 2.0).abs() < 1e-15);
    }

    #[test]
    fn test_bessel_j1_at_known_values() {
        // J_1(1.0) ≈ 0.44005058574
        assert!((bessel_j1(1.0) - 0.44005058574).abs() < 1e-5);
        // J_1(2.0) ≈ 0.57672480775
        assert!((bessel_j1(2.0) - 0.57672480775).abs() < 1e-5);
        // J_1(5.0) ≈ -0.32757913759
        assert!((bessel_j1(5.0) - (-0.32757913759)).abs() < 1e-4);
    }

    #[test]
    fn test_bessel_j1_negative_argument() {
        // J_1(-x) = -J_1(x)
        let x = 3.5;
        assert!((bessel_j1(-x) + bessel_j1(x)).abs() < EPSILON);
    }

    #[test]
    fn test_bessel_j1_first_zero() {
        // First zero of J_1 at x ≈ 3.8317
        assert!(bessel_j1(3.8317).abs() < 0.001);
    }

    #[test]
    fn test_bessel_j1_large_argument() {
        // J_1(10.0) ≈ 0.04347274617
        assert!((bessel_j1(10.0) - 0.04347274617).abs() < 1e-4);
        // J_1(20.0) ≈ 0.06683312418
        assert!((bessel_j1(20.0) - 0.06683312418).abs() < 1e-4);
    }

    // --- Bessel J0 tests ---

    #[test]
    fn test_bessel_j0_at_zero() {
        assert!((bessel_j0(0.0) - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_bessel_j0_known_values() {
        // J_0(1.0) ≈ 0.76519768656
        assert!((bessel_j0(1.0) - 0.76519768656).abs() < 1e-5);
        // J_0(2.405) ≈ 0 (first zero)
        assert!(bessel_j0(2.4048).abs() < 0.001);
    }

    #[test]
    fn test_bessel_j0_symmetry() {
        // J_0 is even: J_0(-x) = J_0(x)
        assert!((bessel_j0(-3.0) - bessel_j0(3.0)).abs() < EPSILON);
    }

    // --- Airy pattern tests ---

    #[test]
    fn test_airy_pattern_at_zero() {
        assert!((airy_pattern(0.0) - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_airy_pattern_first_minimum() {
        // First zero at x ≈ 3.8317 (first zero of J_1)
        assert!(airy_pattern(3.8317) < 1e-4);
    }

    #[test]
    fn test_airy_pattern_symmetry() {
        assert!((airy_pattern(2.0) - airy_pattern(-2.0)).abs() < EPSILON);
    }

    #[test]
    fn test_airy_pattern_decreasing_from_center() {
        let a0 = airy_pattern(0.0);
        let a1 = airy_pattern(1.0);
        let a2 = airy_pattern(2.0);
        assert!(a0 > a1);
        assert!(a1 > a2);
    }

    #[test]
    fn test_airy_pattern_secondary_maximum() {
        // Secondary max near x ≈ 5.136
        let peak = airy_pattern(5.136);
        assert!(peak > 0.01);
        assert!(peak < 0.02);
    }

    // --- Fraunhofer angle tests ---

    #[test]
    fn test_fraunhofer_angle_basic() {
        // For large particles: sin(theta) = 1.22 * lambda / d
        let angle = fraunhofer_angle(0.6328, 100.0);
        let expected = (1.22_f64 * 0.6328 / 100.0).asin();
        assert!((angle - expected).abs() < EPSILON);
    }

    #[test]
    fn test_fraunhofer_angle_small_particle() {
        // When 1.22*lambda/d >= 1, angle should be pi/2
        let angle = fraunhofer_angle(0.6328, 0.5);
        assert!(angle > 1.0); // Large angle for small particle relative to lambda
    }

    #[test]
    fn test_fraunhofer_angle_large_particle() {
        // Large particle -> small angle
        let angle = fraunhofer_angle(0.6328, 1000.0);
        assert!(angle < 0.001);
    }

    #[test]
    fn test_fraunhofer_angle_inversely_proportional() {
        let a1 = fraunhofer_angle(0.6328, 50.0);
        let a2 = fraunhofer_angle(0.6328, 100.0);
        // Doubling diameter should roughly halve the angle (small angle approx)
        assert!((a1 / a2 - 2.0).abs() < 0.01);
    }

    // --- FraunhoferDiffraction tests ---

    #[test]
    fn test_fraunhofer_diffraction_hene() {
        let fd = FraunhoferDiffraction::hene();
        assert!((fd.wavelength_um - 0.6328).abs() < EPSILON);
    }

    #[test]
    fn test_fraunhofer_intensity_at_zero_angle() {
        let fd = FraunhoferDiffraction::hene();
        assert!((fd.intensity(100.0, 0.0) - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_fraunhofer_intensity_pattern() {
        let fd = FraunhoferDiffraction::hene();
        let angles: Vec<f64> = (0..10).map(|i| i as f64 * 0.001).collect();
        let pattern = fd.intensity_pattern(100.0, &angles);
        assert_eq!(pattern.len(), 10);
        assert!((pattern[0] - 1.0).abs() < EPSILON); // On-axis
    }

    #[test]
    fn test_fraunhofer_size_parameter() {
        let fd = FraunhoferDiffraction::hene();
        let alpha = fd.size_parameter(100.0);
        let expected = PI * 100.0 / 0.6328;
        assert!((alpha - expected).abs() < EPSILON);
    }

    #[test]
    fn test_fraunhofer_validity() {
        let fd = FraunhoferDiffraction::hene();
        assert!(fd.is_valid_approximation(100.0)); // alpha >> 40
        assert!(!fd.is_valid_approximation(1.0)); // alpha ~ 5, not valid
    }

    #[test]
    fn test_fraunhofer_extinction_cross_section() {
        let fd = FraunhoferDiffraction::hene();
        let cs = fd.extinction_cross_section(10.0);
        let expected = 2.0 * PI * 25.0; // 2 * pi * r^2
        assert!((cs - expected).abs() < EPSILON);
    }

    #[test]
    fn test_fraunhofer_first_minimum() {
        let fd = FraunhoferDiffraction::hene();
        let angle = fd.first_minimum_angle(100.0);
        assert!(angle > 0.0);
        assert!(angle < 0.01);
    }

    // --- MieScatteringApprox tests ---

    #[test]
    fn test_mie_glass_in_water() {
        let mie = MieScatteringApprox::glass_in_water();
        assert!((mie.wavelength_um - 0.6328).abs() < EPSILON);
        assert!((mie.n_real - 1.5).abs() < EPSILON);
        assert!((mie.n_medium - 1.33).abs() < EPSILON);
    }

    #[test]
    fn test_mie_size_parameter() {
        let mie = MieScatteringApprox::glass_in_air();
        let x = mie.size_parameter(10.0);
        let expected = PI * 10.0 * 1.0 / 0.6328;
        assert!((x - expected).abs() < EPSILON);
    }

    #[test]
    fn test_mie_relative_index() {
        let mie = MieScatteringApprox::glass_in_water();
        let m = mie.relative_index();
        assert!((m - 1.5 / 1.33).abs() < EPSILON);
    }

    #[test]
    fn test_mie_q_ext_large_particle() {
        // For very large particles, Q_ext oscillates around 2 (extinction paradox)
        let mie = MieScatteringApprox::glass_in_air();
        let q = mie.q_ext(1000.0);
        // Should oscillate around 2 for large rho
        assert!(q > 1.0 && q < 3.0);
    }

    #[test]
    fn test_mie_q_ext_small_particle() {
        let mie = MieScatteringApprox::glass_in_air();
        let q = mie.q_ext(0.1);
        // Small particle: Q_ext should be small
        assert!(q >= 0.0);
    }

    #[test]
    fn test_mie_q_abs_non_absorbing() {
        let mie = MieScatteringApprox::glass_in_air();
        assert!(mie.q_abs(10.0).abs() < EPSILON);
    }

    #[test]
    fn test_mie_q_abs_absorbing() {
        let mie = MieScatteringApprox::new(0.6328, 1.5, 0.01, 1.0);
        let q = mie.q_abs(10.0);
        assert!(q > 0.0);
        assert!(q < 1.0);
    }

    #[test]
    fn test_mie_q_sca_equals_ext_minus_abs() {
        let mie = MieScatteringApprox::new(0.6328, 1.5, 0.005, 1.0);
        let d = 20.0;
        let q_sca = mie.q_sca(d);
        let q_ext = mie.q_ext(d);
        let q_abs = mie.q_abs(d);
        assert!((q_sca - (q_ext - q_abs)).abs() < EPSILON);
    }

    #[test]
    fn test_mie_extinction_cross_section() {
        let mie = MieScatteringApprox::glass_in_air();
        let cs = mie.extinction_cross_section(10.0);
        let expected = mie.q_ext(10.0) * PI * 25.0;
        assert!((cs - expected).abs() < EPSILON);
    }

    // --- ParticleSizeDistribution tests ---

    #[test]
    fn test_psd_creation_valid() {
        let edges = vec![1.0, 2.0, 5.0, 10.0];
        let fracs = vec![0.2, 0.5, 0.3];
        let psd = ParticleSizeDistribution::new(&edges, &fracs);
        assert!(psd.is_ok());
        let psd = psd.unwrap();
        assert_eq!(psd.num_bins(), 3);
    }

    #[test]
    fn test_psd_creation_wrong_length() {
        let edges = vec![1.0, 2.0, 5.0];
        let fracs = vec![0.5]; // Should be 2
        assert!(ParticleSizeDistribution::new(&edges, &fracs).is_err());
    }

    #[test]
    fn test_psd_creation_non_monotonic() {
        let edges = vec![1.0, 5.0, 3.0];
        let fracs = vec![0.5, 0.5];
        assert!(ParticleSizeDistribution::new(&edges, &fracs).is_err());
    }

    #[test]
    fn test_psd_creation_negative_fraction() {
        let edges = vec![1.0, 2.0, 5.0];
        let fracs = vec![0.5, -0.1];
        assert!(ParticleSizeDistribution::new(&edges, &fracs).is_err());
    }

    #[test]
    fn test_psd_bin_centers() {
        let edges = vec![1.0, 4.0, 9.0];
        let fracs = vec![0.5, 0.5];
        let psd = ParticleSizeDistribution::new(&edges, &fracs).unwrap();

        let geo_centers = psd.bin_centers_geometric();
        assert!((geo_centers[0] - 2.0).abs() < EPSILON); // sqrt(1*4)
        assert!((geo_centers[1] - 6.0).abs() < EPSILON); // sqrt(4*9)

        let arith_centers = psd.bin_centers_arithmetic();
        assert!((arith_centers[0] - 2.5).abs() < EPSILON);
        assert!((arith_centers[1] - 6.5).abs() < EPSILON);
    }

    #[test]
    fn test_psd_cumulative_undersize() {
        let edges = vec![1.0, 2.0, 5.0, 10.0];
        let fracs = vec![0.2, 0.5, 0.3];
        let psd = ParticleSizeDistribution::new(&edges, &fracs).unwrap();

        let cum = psd.cumulative_undersize();
        assert_eq!(cum.len(), 4);
        assert!((cum[0] - 0.0).abs() < EPSILON);
        assert!((cum[1] - 0.2).abs() < EPSILON);
        assert!((cum[2] - 0.7).abs() < EPSILON);
        assert!((cum[3] - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_psd_normalize() {
        let edges = vec![1.0, 2.0, 5.0, 10.0];
        let fracs = vec![0.4, 1.0, 0.6];
        let mut psd = ParticleSizeDistribution::new(&edges, &fracs).unwrap();
        psd.normalize();
        assert!((psd.total_fraction() - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_psd_from_log_normal() {
        let psd = ParticleSizeDistribution::from_log_normal(50.0, 2.0, 1.0, 500.0, 20);
        assert_eq!(psd.num_bins(), 20);
        assert!((psd.total_fraction() - 1.0).abs() < EPSILON);

        // The peak should be near the median
        let centers = psd.bin_centers_geometric();
        let max_idx = psd
            .fractions
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        // Peak bin center should be within a factor of 2 of median
        assert!(centers[max_idx] > 25.0 && centers[max_idx] < 100.0);
    }

    #[test]
    fn test_psd_from_rosin_rammler() {
        let psd = ParticleSizeDistribution::from_rosin_rammler(100.0, 2.0, 1.0, 1000.0, 20);
        assert_eq!(psd.num_bins(), 20);
        assert!((psd.total_fraction() - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_psd_too_few_edges() {
        let edges = vec![1.0];
        let fracs: Vec<f64> = vec![];
        assert!(ParticleSizeDistribution::new(&edges, &fracs).is_err());
    }

    // --- Cumulative undersize function tests ---

    #[test]
    fn test_cumulative_undersize_function() {
        let edges = vec![1.0, 5.0, 10.0, 50.0];
        let fracs = vec![0.1, 0.6, 0.3];
        let cum = cumulative_undersize(&edges, &fracs);
        assert_eq!(cum.len(), 4);
        assert!((cum[0]).abs() < EPSILON);
        assert!((cum[3] - 1.0).abs() < EPSILON);
    }

    // --- Log-Normal Fit tests ---

    #[test]
    fn test_log_normal_fit_symmetric() {
        let psd = ParticleSizeDistribution::from_log_normal(50.0, 2.0, 1.0, 500.0, 30);
        let fit = LogNormalFit::fit(&psd);

        // Fitted median should be close to 50 um
        assert!((fit.median - 50.0).abs() < 5.0);
        // Fitted GSD should be close to 2.0
        assert!((fit.geo_std_dev - 2.0).abs() < 0.3);
    }

    #[test]
    fn test_log_normal_pdf() {
        let fit = LogNormalFit {
            median: 50.0,
            geo_std_dev: 2.0,
            mu: 50.0_f64.ln(),
            sigma: 2.0_f64.ln(),
            r_squared: 1.0,
        };

        // PDF should be positive
        let p = fit.pdf(50.0);
        assert!(p > 0.0);

        // PDF at median should be near the peak for log-normal
        let p_low = fit.pdf(10.0);
        let p_high = fit.pdf(200.0);
        assert!(p > p_low);
        assert!(p > p_high);
    }

    #[test]
    fn test_log_normal_cdf() {
        let fit = LogNormalFit {
            median: 50.0,
            geo_std_dev: 2.0,
            mu: 50.0_f64.ln(),
            sigma: 2.0_f64.ln(),
            r_squared: 1.0,
        };

        // CDF at median should be ~0.5
        let c = fit.cdf(50.0);
        assert!((c - 0.5).abs() < 0.01);

        // CDF should be monotonically increasing
        assert!(fit.cdf(10.0) < fit.cdf(50.0));
        assert!(fit.cdf(50.0) < fit.cdf(200.0));
    }

    #[test]
    fn test_log_normal_cdf_at_zero() {
        let fit = LogNormalFit {
            median: 50.0,
            geo_std_dev: 2.0,
            mu: 50.0_f64.ln(),
            sigma: 2.0_f64.ln(),
            r_squared: 1.0,
        };
        assert!((fit.cdf(0.0)).abs() < EPSILON);
    }

    #[test]
    fn test_log_normal_pdf_at_zero() {
        let fit = LogNormalFit {
            median: 50.0,
            geo_std_dev: 2.0,
            mu: 50.0_f64.ln(),
            sigma: 2.0_f64.ln(),
            r_squared: 1.0,
        };
        assert!((fit.pdf(0.0)).abs() < EPSILON);
    }

    // --- Rosin-Rammler Fit tests ---

    #[test]
    fn test_rosin_rammler_fit() {
        let psd = ParticleSizeDistribution::from_rosin_rammler(100.0, 3.0, 1.0, 500.0, 30);
        let fit = RosinRammlerFit::fit(&psd);

        // x' should be close to 100
        assert!((fit.x_prime - 100.0).abs() < 20.0);
        // n should be close to 3.0
        assert!((fit.n - 3.0).abs() < 1.0);
    }

    #[test]
    fn test_rosin_rammler_cdf() {
        let fit = RosinRammlerFit {
            x_prime: 100.0,
            n: 2.0,
            r_squared: 1.0,
        };

        // At x = x', Q = 1 - exp(-1) ≈ 0.6321
        let q = fit.cdf(100.0);
        assert!((q - 0.6321).abs() < 0.001);

        // CDF should be monotonically increasing
        assert!(fit.cdf(50.0) < fit.cdf(100.0));
        assert!(fit.cdf(100.0) < fit.cdf(200.0));
    }

    #[test]
    fn test_rosin_rammler_cdf_at_zero() {
        let fit = RosinRammlerFit {
            x_prime: 100.0,
            n: 2.0,
            r_squared: 1.0,
        };
        assert!((fit.cdf(0.0)).abs() < EPSILON);
    }

    #[test]
    fn test_rosin_rammler_pdf() {
        let fit = RosinRammlerFit {
            x_prime: 100.0,
            n: 2.0,
            r_squared: 1.0,
        };
        let p = fit.pdf(100.0);
        assert!(p > 0.0);
    }

    #[test]
    fn test_rosin_rammler_mean_diameter() {
        let fit = RosinRammlerFit {
            x_prime: 100.0,
            n: 2.0,
            r_squared: 1.0,
        };
        let mean = fit.mean_diameter();
        // For n=2: mean = x' * Gamma(1.5) = 100 * sqrt(pi)/2 ≈ 88.62
        let expected = 100.0 * (PI).sqrt() / 2.0;
        assert!((mean - expected).abs() < 1.0);
    }

    // --- Percentile Calculator tests ---

    #[test]
    fn test_percentile_d50() {
        let edges = vec![1.0, 10.0, 100.0, 1000.0];
        let fracs = vec![0.25, 0.50, 0.25];
        let psd = ParticleSizeDistribution::new(&edges, &fracs).unwrap();
        let pc = PercentileCalculator::new(&psd);

        let d50 = pc.d50().unwrap();
        // D50 should be within the middle bin
        assert!(d50 > 10.0 && d50 < 100.0);
    }

    #[test]
    fn test_percentile_d10_d90() {
        let psd = ParticleSizeDistribution::from_log_normal(50.0, 2.0, 1.0, 500.0, 30);
        let pc = PercentileCalculator::new(&psd);

        let d10 = pc.d10().unwrap();
        let d50 = pc.d50().unwrap();
        let d90 = pc.d90().unwrap();

        assert!(d10 < d50);
        assert!(d50 < d90);
        assert!(d10 > 0.0);
    }

    #[test]
    fn test_percentile_out_of_range() {
        let edges = vec![1.0, 10.0];
        let fracs = vec![1.0];
        let psd = ParticleSizeDistribution::new(&edges, &fracs).unwrap();
        let pc = PercentileCalculator::new(&psd);

        assert!(pc.percentile(-1.0).is_none());
        assert!(pc.percentile(101.0).is_none());
    }

    #[test]
    fn test_percentile_boundary_values() {
        let edges = vec![1.0, 10.0];
        let fracs = vec![1.0];
        let psd = ParticleSizeDistribution::new(&edges, &fracs).unwrap();
        let pc = PercentileCalculator::new(&psd);

        let d0 = pc.percentile(0.0).unwrap();
        let d100 = pc.percentile(100.0).unwrap();
        assert!((d0 - 1.0).abs() < EPSILON);
        assert!((d100 - 10.0).abs() < EPSILON);
    }

    #[test]
    fn test_percentile_d1_d99() {
        let psd = ParticleSizeDistribution::from_log_normal(50.0, 2.0, 1.0, 500.0, 30);
        let pc = PercentileCalculator::new(&psd);
        let d1 = pc.d1().unwrap();
        let d99 = pc.d99().unwrap();
        assert!(d1 < d99);
    }

    // --- Span Calculator tests ---

    #[test]
    fn test_span_calculator() {
        let psd = ParticleSizeDistribution::from_log_normal(50.0, 2.0, 1.0, 500.0, 30);
        let span = SpanCalculator::compute(&psd).unwrap();

        // Span should be positive
        assert!(span > 0.0);
        // For GSD=2 log-normal, span is typically around 1.5-3
        assert!(span < 10.0);
    }

    #[test]
    fn test_span_narrow_distribution() {
        // Narrow distribution should have small span
        let psd = ParticleSizeDistribution::from_log_normal(50.0, 1.2, 10.0, 200.0, 30);
        let span = SpanCalculator::compute(&psd).unwrap();
        assert!(span < 2.0);
    }

    #[test]
    fn test_span_from_values() {
        let span = SpanCalculator::from_values(10.0, 50.0, 90.0).unwrap();
        assert!((span - 1.6).abs() < EPSILON);
    }

    #[test]
    fn test_span_from_values_zero_d50() {
        assert!(SpanCalculator::from_values(10.0, 0.0, 90.0).is_none());
    }

    // --- Mean Diameters tests ---

    #[test]
    fn test_mean_diameters_compute() {
        let edges = vec![1.0, 10.0, 100.0];
        let fracs = vec![0.5, 0.5];
        let psd = ParticleSizeDistribution::new(&edges, &fracs).unwrap();
        let means = MeanDiameters::compute(&psd);

        // All means should be positive
        assert!(means.d_1_0 > 0.0);
        assert!(means.d_2_0 > 0.0);
        assert!(means.d_3_0 > 0.0);
        assert!(means.d_2_1 > 0.0);
        assert!(means.d_3_2 > 0.0);
        assert!(means.d_4_3 > 0.0);

        // D[4,3] >= D[3,2] >= D[1,0] for any distribution (Jensen's inequality)
        assert!(means.d_4_3 >= means.d_3_2 - EPSILON);
    }

    #[test]
    fn test_sauter_mean_diameter() {
        let edges = vec![1.0, 10.0, 100.0];
        let fracs = vec![0.5, 0.5];
        let psd = ParticleSizeDistribution::new(&edges, &fracs).unwrap();
        let d32 = MeanDiameters::sauter(&psd);
        assert!(d32 > 0.0);
    }

    #[test]
    fn test_de_brouckere_mean_diameter() {
        let edges = vec![1.0, 10.0, 100.0];
        let fracs = vec![0.5, 0.5];
        let psd = ParticleSizeDistribution::new(&edges, &fracs).unwrap();
        let d43 = MeanDiameters::de_brouckere(&psd);
        assert!(d43 > 0.0);
    }

    #[test]
    fn test_mean_diameter_uniform() {
        // Uniform distribution: all particles same size -> all means equal
        let edges = vec![9.9, 10.1];
        let fracs = vec![1.0];
        let psd = ParticleSizeDistribution::new(&edges, &fracs).unwrap();
        let means = MeanDiameters::compute(&psd);

        let center = (9.9_f64 * 10.1_f64).sqrt();
        assert!((means.d_1_0 - center).abs() < 0.1);
        assert!((means.d_4_3 - center).abs() < 0.1);
    }

    #[test]
    fn test_dpq_custom() {
        let edges = vec![1.0, 10.0, 100.0, 1000.0];
        let fracs = vec![0.33, 0.34, 0.33];
        let psd = ParticleSizeDistribution::new(&edges, &fracs).unwrap();

        let d_5_3 = MeanDiameters::dpq(&psd, 5.0, 3.0);
        assert!(d_5_3 > 0.0);
    }

    // --- Specific Surface Area tests ---

    #[test]
    fn test_specific_surface_area() {
        let ssa = specific_surface_area(10.0, 2.5);
        let expected = 6.0 / (2.5 * 10.0);
        assert!((ssa - expected).abs() < EPSILON);
    }

    #[test]
    fn test_specific_surface_area_smaller_particles() {
        // Smaller particles -> larger SSA
        let ssa_large = specific_surface_area(100.0, 2.5);
        let ssa_small = specific_surface_area(10.0, 2.5);
        assert!(ssa_small > ssa_large);
    }

    #[test]
    #[should_panic]
    fn test_specific_surface_area_zero_diameter() {
        specific_surface_area(0.0, 2.5);
    }

    // --- Obscuration tests ---

    #[test]
    fn test_obscuration_zero_concentration() {
        let obs = obscuration(0.0, 10.0, 1.0);
        assert!(obs.abs() < EPSILON);
    }

    #[test]
    fn test_obscuration_increases_with_concentration() {
        let obs1 = obscuration(0.01, 10.0, 1.0);
        let obs2 = obscuration(0.1, 10.0, 1.0);
        assert!(obs2 > obs1);
    }

    #[test]
    fn test_obscuration_range() {
        let obs = obscuration(0.1, 10.0, 0.5);
        assert!(obs >= 0.0 && obs <= 100.0);
    }

    #[test]
    fn test_is_obscuration_optimal() {
        assert!(!is_obscuration_optimal(3.0));
        assert!(is_obscuration_optimal(5.0));
        assert!(is_obscuration_optimal(15.0));
        assert!(is_obscuration_optimal(25.0));
        assert!(!is_obscuration_optimal(30.0));
    }

    #[test]
    fn test_concentration_for_obscuration() {
        let c = concentration_for_obscuration(15.0, 10.0, 1.0);
        let obs = obscuration(c, 10.0, 1.0);
        assert!((obs - 15.0).abs() < 0.01);
    }

    #[test]
    fn test_concentration_for_obscuration_roundtrip() {
        let target = 20.0;
        let c = concentration_for_obscuration(target, 5.0, 2.0);
        let actual = obscuration(c, 5.0, 2.0);
        assert!((actual - target).abs() < 0.01);
    }

    // --- ISO 13320 Compliance tests ---

    #[test]
    fn test_iso13320_compliant() {
        let psd = ParticleSizeDistribution::from_log_normal(50.0, 2.0, 1.0, 500.0, 30);
        let compliance = Iso13320Compliance::evaluate(&psd, 15.0, 0.5, 0.1, 1000.0);

        assert!(compliance.obscuration_ok);
        assert!(compliance.residual_ok);
        assert!(compliance.range_ok);
        assert!(compliance.compliant);
    }

    #[test]
    fn test_iso13320_non_compliant_obscuration() {
        let psd = ParticleSizeDistribution::from_log_normal(50.0, 2.0, 1.0, 500.0, 30);
        let compliance = Iso13320Compliance::evaluate(&psd, 2.0, 0.5, 0.1, 1000.0);

        assert!(!compliance.obscuration_ok);
        assert!(!compliance.compliant);
    }

    #[test]
    fn test_iso13320_non_compliant_residual() {
        let psd = ParticleSizeDistribution::from_log_normal(50.0, 2.0, 1.0, 500.0, 30);
        let compliance = Iso13320Compliance::evaluate(&psd, 15.0, 5.0, 0.1, 1000.0);

        assert!(!compliance.residual_ok);
        assert!(!compliance.compliant);
    }

    #[test]
    fn test_iso13320_non_compliant_range() {
        let psd = ParticleSizeDistribution::from_log_normal(50.0, 2.0, 1.0, 500.0, 30);
        // Instrument range too narrow
        let compliance = Iso13320Compliance::evaluate(&psd, 15.0, 0.5, 100.0, 200.0);

        assert!(!compliance.range_ok);
        assert!(!compliance.compliant);
    }

    // --- Detector Geometry tests ---

    #[test]
    fn test_detector_geometry_creation() {
        let det = DetectorGeometry::log_spaced(32, 0.01, 40.0);
        assert_eq!(det.num_detectors(), 32);
    }

    #[test]
    fn test_detector_geometry_angles_increasing() {
        let det = DetectorGeometry::log_spaced(16, 0.01, 30.0);
        for i in 0..det.num_detectors() - 1 {
            assert!(det.inner_angles[i] < det.inner_angles[i + 1]);
            assert!(det.outer_angles[i] < det.outer_angles[i + 1]);
        }
    }

    #[test]
    fn test_detector_center_angle() {
        let det = DetectorGeometry::log_spaced(8, 0.1, 10.0);
        for i in 0..det.num_detectors() {
            let center = det.center_angle(i);
            assert!(center > det.inner_angles[i]);
            assert!(center < det.outer_angles[i]);
        }
    }

    #[test]
    fn test_detector_fraunhofer_energy() {
        let det = DetectorGeometry::log_spaced(8, 0.01, 10.0);
        let fh = FraunhoferDiffraction::hene();
        let energy = det.fraunhofer_energy(&fh, 50.0, 10);
        assert_eq!(energy.len(), 8);
        // All energies should be non-negative
        for &e in &energy {
            assert!(e >= 0.0);
        }
    }

    // --- Scattering Matrix tests ---

    #[test]
    fn test_scattering_matrix_dimensions() {
        let det = DetectorGeometry::log_spaced(4, 0.1, 10.0);
        let edges = vec![1.0, 10.0, 100.0, 1000.0];
        let matrix = ScatteringMatrix::build_fraunhofer(&det, &edges, HENE_WAVELENGTH_UM, 5);
        assert_eq!(matrix.len(), 4); // 4 detectors
        assert_eq!(matrix[0].len(), 3); // 3 bins
    }

    // --- Modality tests ---

    #[test]
    fn test_count_modes_unimodal() {
        let edges = vec![1.0, 2.0, 5.0, 10.0, 20.0, 50.0];
        let fracs = vec![0.05, 0.15, 0.60, 0.15, 0.05];
        let psd = ParticleSizeDistribution::new(&edges, &fracs).unwrap();
        let modes = count_modes(&psd);
        assert_eq!(modes, 1);
    }

    #[test]
    fn test_count_modes_bimodal() {
        let edges = vec![1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0];
        let fracs = vec![0.30, 0.10, 0.02, 0.08, 0.20, 0.30];
        let psd = ParticleSizeDistribution::new(&edges, &fracs).unwrap();
        let modes = count_modes(&psd);
        assert!(modes >= 2);
    }

    #[test]
    fn test_uniformity_coefficient() {
        let psd = ParticleSizeDistribution::from_log_normal(50.0, 2.0, 1.0, 500.0, 30);
        let cu = uniformity_coefficient(&psd);
        assert!(cu.is_some());
        assert!(cu.unwrap() > 1.0); // Cu is always >= 1
    }

    #[test]
    fn test_coefficient_of_gradation() {
        let psd = ParticleSizeDistribution::from_log_normal(50.0, 2.0, 1.0, 500.0, 30);
        let cc = coefficient_of_gradation(&psd);
        assert!(cc.is_some());
        assert!(cc.unwrap() > 0.0);
    }

    // --- Polydisperse pattern tests ---

    #[test]
    fn test_polydisperse_pattern_length() {
        let psd = ParticleSizeDistribution::from_log_normal(50.0, 2.0, 1.0, 500.0, 10);
        let angles: Vec<f64> = (0..20).map(|i| (i as f64 + 1.0) * 0.01).collect();
        let pattern = polydisperse_pattern(&psd, HENE_WAVELENGTH_UM, &angles);
        assert_eq!(pattern.len(), 20);
    }

    #[test]
    fn test_polydisperse_pattern_non_negative() {
        let psd = ParticleSizeDistribution::from_log_normal(50.0, 2.0, 1.0, 500.0, 10);
        let angles: Vec<f64> = (0..10).map(|i| (i as f64 + 1.0) * 0.01).collect();
        let pattern = polydisperse_pattern(&psd, HENE_WAVELENGTH_UM, &angles);
        for &p in &pattern {
            assert!(p >= 0.0);
        }
    }

    // --- Signal processing tests ---

    #[test]
    fn test_smooth_signal_identity() {
        let signal = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let smoothed = smooth_signal(&signal, 1);
        assert_eq!(smoothed.len(), signal.len());
        for (a, b) in smoothed.iter().zip(signal.iter()) {
            assert!((a - b).abs() < EPSILON);
        }
    }

    #[test]
    fn test_smooth_signal_averaging() {
        let signal = vec![0.0, 0.0, 10.0, 0.0, 0.0];
        let smoothed = smooth_signal(&signal, 3);
        // Center value should be average of neighbors
        assert!((smoothed[2] - 10.0 / 3.0).abs() < EPSILON);
    }

    #[test]
    fn test_smooth_signal_empty() {
        let smoothed = smooth_signal(&[], 3);
        assert!(smoothed.is_empty());
    }

    #[test]
    fn test_scattering_vector() {
        let q = scattering_vector(HENE_WAVELENGTH_UM, 1.0, 0.1);
        let expected = (4.0 * PI / HENE_WAVELENGTH_UM) * (0.05_f64).sin();
        assert!((q - expected).abs() < EPSILON);
    }

    // --- erf approximation tests ---

    #[test]
    fn test_erf_at_zero() {
        assert!(erf_approx(0.0).abs() < EPSILON);
    }

    #[test]
    fn test_erf_symmetry() {
        let x = 1.5;
        assert!((erf_approx(x) + erf_approx(-x)).abs() < EPSILON);
    }

    #[test]
    fn test_erf_known_values() {
        // erf(1) ≈ 0.8427
        assert!((erf_approx(1.0) - 0.8427).abs() < 0.001);
        // erf(2) ≈ 0.9953
        assert!((erf_approx(2.0) - 0.9953).abs() < 0.001);
    }

    // --- gamma approximation tests ---

    #[test]
    fn test_gamma_at_integers() {
        // Gamma(1) = 1
        assert!((gamma_approx(1.0) - 1.0).abs() < EPSILON);
        // Gamma(2) = 1
        assert!((gamma_approx(2.0) - 1.0).abs() < EPSILON);
        // Gamma(3) = 2
        assert!((gamma_approx(3.0) - 2.0).abs() < 0.001);
        // Gamma(4) = 6
        assert!((gamma_approx(4.0) - 6.0).abs() < 0.01);
    }

    #[test]
    fn test_gamma_at_half() {
        // Gamma(0.5) = sqrt(pi) ≈ 1.77245
        assert!((gamma_approx(0.5) - PI.sqrt()).abs() < 0.001);
    }

    #[test]
    fn test_gamma_1_5() {
        // Gamma(1.5) = sqrt(pi)/2 ≈ 0.8862
        let expected = PI.sqrt() / 2.0;
        assert!((gamma_approx(1.5) - expected).abs() < 0.001);
    }

    // --- Physical constants tests ---

    #[test]
    fn test_constants() {
        assert!((HENE_WAVELENGTH_UM - 0.6328).abs() < EPSILON);
        assert!((VIOLET_DIODE_WAVELENGTH_UM - 0.405).abs() < EPSILON);
        assert!((BLUE_DIODE_WAVELENGTH_UM - 0.470).abs() < EPSILON);
        assert!((REFRACTIVE_INDEX_WATER - 1.33).abs() < EPSILON);
        assert!((MIN_OBSCURATION_PCT - 5.0).abs() < EPSILON);
        assert!((MAX_OBSCURATION_PCT - 25.0).abs() < EPSILON);
    }

    // --- Integration / end-to-end tests ---

    #[test]
    fn test_end_to_end_lognormal_analysis() {
        // Generate a log-normal PSD, compute all metrics
        let psd = ParticleSizeDistribution::from_log_normal(50.0, 2.0, 1.0, 500.0, 30);

        let pc = PercentileCalculator::new(&psd);
        let d10 = pc.d10().unwrap();
        let d50 = pc.d50().unwrap();
        let d90 = pc.d90().unwrap();
        assert!(d10 < d50);
        assert!(d50 < d90);

        let span = SpanCalculator::compute(&psd).unwrap();
        assert!(span > 0.0);

        let means = MeanDiameters::compute(&psd);
        assert!(means.d_3_2 > 0.0);
        assert!(means.d_4_3 > 0.0);

        let fit = LogNormalFit::fit(&psd);
        assert!(fit.median > 0.0);
        assert!(fit.geo_std_dev > 1.0);
    }

    #[test]
    fn test_end_to_end_rosin_rammler_analysis() {
        let psd = ParticleSizeDistribution::from_rosin_rammler(200.0, 2.5, 1.0, 2000.0, 30);

        let pc = PercentileCalculator::new(&psd);
        let d50 = pc.d50().unwrap();
        assert!(d50 > 50.0 && d50 < 500.0);

        let fit = RosinRammlerFit::fit(&psd);
        assert!(fit.x_prime > 0.0);
        assert!(fit.n > 0.0);
    }

    #[test]
    fn test_mie_vs_fraunhofer_cross_section_large_particle() {
        // For very large particles, Mie Q_ext -> 2, matching Fraunhofer extinction paradox
        let mie = MieScatteringApprox::glass_in_air();
        let d = 500.0;
        let q_ext = mie.q_ext(d);

        // Q_ext oscillates around 2 for large rho; allow wide tolerance
        assert!(q_ext > 0.5);
    }

    #[test]
    fn test_sauter_less_than_de_brouckere() {
        // D[3,2] <= D[4,3] always (by Cauchy-Schwarz)
        let psd = ParticleSizeDistribution::from_log_normal(50.0, 2.0, 1.0, 500.0, 30);
        let means = MeanDiameters::compute(&psd);
        assert!(means.d_3_2 <= means.d_4_3 + EPSILON);
    }
}
