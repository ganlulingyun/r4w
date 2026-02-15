//! Gravitational lensing simulation for radio signals.
//!
//! This module simulates the effects of gravitational lensing on radio-frequency signals
//! passing near massive objects (stars, galaxies, galaxy clusters). Gravitational lensing
//! causes multiple images of a background source, magnification of flux, and differential
//! time delays between images — all observable at radio wavelengths.
//!
//! # Overview
//!
//! - **[`LensConfig`]** — lens geometry: mass, observer-lens and lens-source distances,
//!   Einstein radius
//! - **[`PointMassLens`]** — point-mass (Schwarzschild) lens model, producing exactly two
//!   images for any off-axis source
//! - **[`SisLens`]** — Singular Isothermal Sphere lens model for galaxy-cluster lensing,
//!   producing one or two images depending on source alignment
//! - **[`ImageSolver`]** — numerical solver for image positions from the lens equation
//! - **[`TimeDelayCalculator`]** — Shapiro / geometric time delay between multiple images
//! - **[`MagnificationComputer`]** — flux magnification from the Jacobian determinant
//!   of the lens mapping
//!
//! # Physics background
//!
//! A gravitational lens deflects light (and radio waves) by an angle proportional to
//! the enclosed mass. For a point mass *M* the deflection angle at impact parameter *b* is
//!
//! ```text
//! alpha_hat = 4 G M / (c^2 b)
//! ```
//!
//! The *Einstein radius* defines the angular scale of lensing:
//!
//! ```text
//! theta_E = sqrt( 4 G M / c^2  *  D_ls / (D_l * D_s) )
//! ```
//!
//! where `D_l`, `D_s`, and `D_ls` are angular diameter distances to the lens, to the
//! source, and from the lens to the source respectively.
//!
//! For the **Singular Isothermal Sphere** (SIS) the deflection angle is constant:
//!
//! ```text
//! alpha_hat_SIS = 4 pi (sigma_v / c)^2
//! ```
//!
//! where `sigma_v` is the one-dimensional velocity dispersion of the galaxy/cluster.
//!
//! # Example
//!
//! ```
//! use r4w_core::gravitational_lensing_simulator::{
//!     LensConfig, PointMassLens, einstein_radius, schwarzschild_radius,
//! };
//!
//! // A solar-mass lens at 1 kpc with source at 2 kpc
//! let d_l = 3.086e19;  // 1 kpc in meters
//! let d_s = 6.172e19;  // 2 kpc in meters
//! let mass = 1.989e30; // solar mass in kg
//! let cfg = LensConfig::new(mass, d_l, d_s);
//! assert!(cfg.einstein_radius() > 0.0);
//!
//! let lens = PointMassLens::new(cfg);
//! let images = lens.image_positions(0.5); // source at 0.5 Einstein radii
//! assert_eq!(images.len(), 2);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Gravitational constant in SI units (m^3 kg^-1 s^-2).
const G_SI: f64 = 6.674_30e-11;

/// Speed of light in m/s.
const C_SI: f64 = 2.997_924_58e8;

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Compute the Einstein radius (in radians) for a point-mass lens.
///
/// The Einstein radius is the angular scale at which strong lensing occurs:
///
/// ```text
/// theta_E = sqrt( 4 G M / c^2  *  D_ls / (D_l * D_s) )
/// ```
///
/// # Arguments
///
/// * `mass` — lens mass in kg
/// * `d_l` — angular diameter distance to the lens in meters
/// * `d_s` — angular diameter distance to the source in meters
///
/// # Returns
///
/// Einstein radius in radians.
pub fn einstein_radius(mass: f64, d_l: f64, d_s: f64) -> f64 {
    let d_ls = d_s - d_l;
    assert!(d_ls > 0.0, "Source must be behind the lens (d_s > d_l)");
    let rs = schwarzschild_radius(mass);
    // theta_E = sqrt( r_s * D_ls / (D_l * D_s) )
    // where r_s = 2GM/c^2 (Schwarzschild radius)
    // 4GM/c^2 = 2 * r_s
    let numerator = 2.0 * rs * d_ls;
    let denominator = d_l * d_s;
    (numerator / denominator).sqrt()
}

/// Compute the Schwarzschild radius of a mass.
///
/// ```text
/// r_s = 2 G M / c^2
/// ```
///
/// # Arguments
///
/// * `mass` — mass in kg
///
/// # Returns
///
/// Schwarzschild radius in meters.
pub fn schwarzschild_radius(mass: f64) -> f64 {
    2.0 * G_SI * mass / (C_SI * C_SI)
}

/// Compute the deflection angle for a point mass at impact parameter `b`.
///
/// ```text
/// alpha_hat = 4 G M / (c^2 * b)
/// ```
///
/// # Arguments
///
/// * `mass` — lens mass in kg
/// * `b` — impact parameter in meters (physical distance of closest approach)
///
/// # Returns
///
/// Deflection angle in radians.
pub fn deflection_angle(mass: f64, b: f64) -> f64 {
    assert!(b > 0.0, "Impact parameter must be positive");
    4.0 * G_SI * mass / (C_SI * C_SI * b)
}

/// Compute the critical curve radius for a point-mass lens.
///
/// For a point-mass lens the critical curve is a circle of angular radius equal
/// to the Einstein radius. This function returns the Einstein radius in radians.
///
/// # Arguments
///
/// * `mass` — lens mass in kg
/// * `d_l` — angular diameter distance to the lens in meters
/// * `d_s` — angular diameter distance to the source in meters
///
/// # Returns
///
/// Critical curve angular radius in radians (= Einstein radius).
pub fn critical_curve(mass: f64, d_l: f64, d_s: f64) -> f64 {
    einstein_radius(mass, d_l, d_s)
}

/// Compute the SIS deflection angle for a given velocity dispersion.
///
/// ```text
/// alpha_hat_SIS = 4 pi (sigma_v / c)^2
/// ```
///
/// # Arguments
///
/// * `sigma_v` — one-dimensional velocity dispersion in m/s
///
/// # Returns
///
/// Deflection angle in radians (constant for SIS).
pub fn sis_deflection_angle(sigma_v: f64) -> f64 {
    4.0 * PI * (sigma_v / C_SI).powi(2)
}

/// Compute the SIS Einstein radius in radians.
///
/// ```text
/// theta_E_SIS = 4 pi (sigma_v / c)^2 * D_ls / D_s
/// ```
///
/// # Arguments
///
/// * `sigma_v` — one-dimensional velocity dispersion in m/s
/// * `d_l` — angular diameter distance to the lens in meters
/// * `d_s` — angular diameter distance to the source in meters
///
/// # Returns
///
/// SIS Einstein radius in radians.
pub fn sis_einstein_radius(sigma_v: f64, d_l: f64, d_s: f64) -> f64 {
    let d_ls = d_s - d_l;
    assert!(d_ls > 0.0, "Source must be behind the lens");
    let alpha_hat = sis_deflection_angle(sigma_v);
    alpha_hat * d_ls / d_s
}

// ---------------------------------------------------------------------------
// LensConfig
// ---------------------------------------------------------------------------

/// Configuration for a gravitational lens system.
///
/// Stores the lens mass, observer-lens distance, lens-source distance,
/// and the derived Einstein radius.
#[derive(Debug, Clone)]
pub struct LensConfig {
    /// Lens mass in kg.
    pub mass: f64,
    /// Angular diameter distance from observer to lens in meters.
    pub d_l: f64,
    /// Angular diameter distance from observer to source in meters.
    pub d_s: f64,
    /// Angular diameter distance from lens to source in meters.
    pub d_ls: f64,
    /// Einstein radius in radians.
    theta_e: f64,
}

impl LensConfig {
    /// Create a new lens configuration.
    ///
    /// # Arguments
    ///
    /// * `mass` — lens mass in kg
    /// * `d_l` — observer-to-lens distance in meters
    /// * `d_s` — observer-to-source distance in meters
    ///
    /// # Panics
    ///
    /// Panics if `d_s <= d_l` (source must be behind the lens).
    pub fn new(mass: f64, d_l: f64, d_s: f64) -> Self {
        assert!(d_s > d_l, "Source must be behind the lens");
        let d_ls = d_s - d_l;
        let theta_e = einstein_radius(mass, d_l, d_s);
        Self {
            mass,
            d_l,
            d_s,
            d_ls,
            theta_e,
        }
    }

    /// Return the Einstein radius in radians.
    pub fn einstein_radius(&self) -> f64 {
        self.theta_e
    }

    /// Return the Einstein radius in physical length at the lens plane (meters).
    pub fn einstein_radius_physical(&self) -> f64 {
        self.theta_e * self.d_l
    }

    /// Return the Schwarzschild radius of the lens in meters.
    pub fn schwarzschild_radius(&self) -> f64 {
        schwarzschild_radius(self.mass)
    }
}

// ---------------------------------------------------------------------------
// PointMassLens
// ---------------------------------------------------------------------------

/// Point-mass (Schwarzschild) gravitational lens.
///
/// The lens equation for a point mass in angular coordinates normalised to
/// the Einstein radius is:
///
/// ```text
/// beta = theta - 1 / theta
/// ```
///
/// where `beta` is the (normalised) source position and `theta` is the
/// (normalised) image position. This quadratic has two solutions for any
/// `beta != 0`:
///
/// ```text
/// theta_+/- = ( beta +/- sqrt(beta^2 + 4) ) / 2
/// ```
///
/// For `beta = 0` (perfect alignment) the image is an Einstein ring.
#[derive(Debug, Clone)]
pub struct PointMassLens {
    /// Lens configuration.
    pub config: LensConfig,
}

impl PointMassLens {
    /// Create a new point-mass lens from a [`LensConfig`].
    pub fn new(config: LensConfig) -> Self {
        Self { config }
    }

    /// Solve the lens equation and return image positions in units of
    /// the Einstein radius.
    ///
    /// # Arguments
    ///
    /// * `beta` — source position in units of the Einstein radius
    ///
    /// # Returns
    ///
    /// A vector of image positions (always length 2 for point-mass lens,
    /// or length 1 for exact `beta = 0` producing an Einstein ring).
    pub fn image_positions(&self, beta: f64) -> Vec<f64> {
        if beta.abs() < 1e-15 {
            // Perfect alignment — Einstein ring at theta = 1
            vec![1.0]
        } else {
            let disc = (beta * beta + 4.0).sqrt();
            let theta_plus = (beta + disc) / 2.0;
            let theta_minus = (beta - disc) / 2.0;
            vec![theta_plus, theta_minus]
        }
    }

    /// Compute the magnification of each image.
    ///
    /// For a point-mass lens the magnification of an image at normalised
    /// position `theta` is:
    ///
    /// ```text
    /// mu = 1 / (1 - 1/theta^4)
    /// ```
    ///
    /// The total magnification is `|mu_+| + |mu_-|`.
    ///
    /// # Arguments
    ///
    /// * `beta` — source position in units of the Einstein radius
    ///
    /// # Returns
    ///
    /// Vector of magnifications corresponding to each image.
    pub fn magnifications(&self, beta: f64) -> Vec<f64> {
        let images = self.image_positions(beta);
        images
            .iter()
            .map(|&theta| {
                let t4 = theta.powi(4);
                if t4.abs() < 1e-30 {
                    f64::INFINITY
                } else {
                    (1.0 / (1.0 - 1.0 / t4)).abs()
                }
            })
            .collect()
    }

    /// Compute the total magnification (sum of absolute magnifications of all images).
    ///
    /// # Arguments
    ///
    /// * `beta` — source position in units of the Einstein radius
    pub fn total_magnification(&self, beta: f64) -> f64 {
        self.magnifications(beta).iter().sum()
    }

    /// Compute the time delay between the two images (in seconds).
    ///
    /// The time delay for a point-mass lens between the two images is:
    ///
    /// ```text
    /// Delta_t = (1 + z_l) * (r_s / c) * (D_l * D_s / D_ls) *
    ///           [ (theta_+^2 - theta_-^2) / 2 + ln|theta_- / theta_+| ]
    /// ```
    ///
    /// This method assumes `z_l = 0` (nearby lens, no cosmological redshift).
    ///
    /// # Arguments
    ///
    /// * `beta` — source position in units of the Einstein radius
    ///
    /// # Returns
    ///
    /// Time delay in seconds between the two images. Returns 0 for Einstein ring.
    pub fn time_delay(&self, beta: f64) -> f64 {
        let images = self.image_positions(beta);
        if images.len() < 2 {
            return 0.0;
        }
        let theta_p = images[0];
        let theta_m = images[1];

        // Time delay scale: (4GM/c^3) * (D_l * D_s) / (D_ls)
        // = (2 r_s / c) * (D_l * D_s) / D_ls
        let rs = self.config.schwarzschild_radius();
        let time_scale =
            (2.0 * rs / C_SI) * (self.config.d_l * self.config.d_s) / self.config.d_ls;

        // Geometric delay: (theta^2/2 - beta*theta) for each image, relative
        // We use the standard formula:
        // Delta_t = time_scale * [ (theta_+^2 - theta_-^2)/2 + ln(|theta_-/theta_+|) ]
        let geom = (theta_p * theta_p - theta_m * theta_m) / 2.0;
        let grav = (theta_m.abs() / theta_p.abs()).ln();

        // The overall time delay includes the Einstein radius squared factor
        // since theta is normalised. The full formula is:
        // Delta_t = (theta_E^2 / 2) * time_scale * [ beta * sqrt(beta^2 + 4) + ln(...) ]
        //
        // Equivalently using direct image positions:
        let te2 = self.config.theta_e * self.config.theta_e;
        (te2 * time_scale * (geom + grav)).abs()
    }

    /// Check whether a source at angular position `beta` (in Einstein radii)
    /// is strongly lensed (multiple resolvable images).
    ///
    /// For a point mass, strong lensing always produces two images, but they
    /// become increasingly demagnified and close to the optical axis as
    /// `|beta|` grows. We consider it "strongly lensed" when `|beta| < 1`.
    pub fn is_strongly_lensed(&self, beta: f64) -> bool {
        beta.abs() < 1.0
    }
}

// ---------------------------------------------------------------------------
// SisLens — Singular Isothermal Sphere
// ---------------------------------------------------------------------------

/// Singular Isothermal Sphere (SIS) gravitational lens model.
///
/// Used for modelling galaxy and galaxy-cluster lenses. The deflection angle
/// is constant:
///
/// ```text
/// alpha_hat = 4 pi (sigma_v / c)^2
/// ```
///
/// The lens equation in units of the SIS Einstein radius is:
///
/// ```text
/// beta = theta - sign(theta)
/// ```
///
/// This produces:
/// - Two images when `|beta| < 1` (source inside the Einstein ring)
/// - One image when `|beta| >= 1` (source outside the Einstein ring)
#[derive(Debug, Clone)]
pub struct SisLens {
    /// One-dimensional velocity dispersion in m/s.
    pub sigma_v: f64,
    /// Observer-to-lens distance in meters.
    pub d_l: f64,
    /// Observer-to-source distance in meters.
    pub d_s: f64,
    /// Lens-to-source distance in meters.
    pub d_ls: f64,
    /// SIS Einstein radius in radians.
    pub theta_e: f64,
}

impl SisLens {
    /// Create a new SIS lens.
    ///
    /// # Arguments
    ///
    /// * `sigma_v` — velocity dispersion in m/s
    /// * `d_l` — observer-to-lens distance in meters
    /// * `d_s` — observer-to-source distance in meters
    pub fn new(sigma_v: f64, d_l: f64, d_s: f64) -> Self {
        assert!(d_s > d_l, "Source must be behind the lens");
        let d_ls = d_s - d_l;
        let theta_e = sis_einstein_radius(sigma_v, d_l, d_s);
        Self {
            sigma_v,
            d_l,
            d_s,
            d_ls,
            theta_e,
        }
    }

    /// Return the SIS Einstein radius in radians.
    pub fn einstein_radius(&self) -> f64 {
        self.theta_e
    }

    /// Solve the SIS lens equation.
    ///
    /// # Arguments
    ///
    /// * `beta` — source position in units of the SIS Einstein radius
    ///
    /// # Returns
    ///
    /// Vector of image positions in units of the SIS Einstein radius.
    /// Length 2 when `|beta| < 1`, length 1 otherwise.
    pub fn image_positions(&self, beta: f64) -> Vec<f64> {
        if beta.abs() < 1.0 - 1e-15 {
            // Two images: theta = beta + 1 and theta = beta - 1
            vec![beta + 1.0, beta - 1.0]
        } else {
            // One image on the same side as the source
            if beta >= 0.0 {
                vec![beta + 1.0]
            } else {
                vec![beta - 1.0]
            }
        }
    }

    /// Compute the magnification of each image.
    ///
    /// For SIS, the magnification of an image at normalised position `theta` is:
    ///
    /// ```text
    /// mu = |theta| / |theta| - 1  =  |theta / (|theta| - 1)|
    /// ```
    ///
    /// More precisely, `mu = theta / (theta - sign(theta) * 1)` but in
    /// absolute value: `|mu| = |theta| / ||theta| - 1|`.
    ///
    /// # Arguments
    ///
    /// * `beta` — source position in units of the SIS Einstein radius
    pub fn magnifications(&self, beta: f64) -> Vec<f64> {
        let images = self.image_positions(beta);
        images
            .iter()
            .map(|&theta| {
                let abs_theta = theta.abs();
                if (abs_theta - 1.0).abs() < 1e-15 {
                    f64::INFINITY // on the critical curve
                } else {
                    abs_theta / (abs_theta - 1.0).abs()
                }
            })
            .collect()
    }

    /// Compute the total magnification.
    pub fn total_magnification(&self, beta: f64) -> f64 {
        self.magnifications(beta)
            .iter()
            .filter(|m| m.is_finite())
            .sum()
    }

    /// Check whether the source is inside the Einstein ring (two images).
    pub fn is_strongly_lensed(&self, beta: f64) -> bool {
        beta.abs() < 1.0
    }

    /// Compute the time delay between the two SIS images (seconds).
    ///
    /// The Fermat potential for SIS in normalised coordinates is:
    ///
    /// ```text
    /// tau(theta) = (theta - beta)^2 / 2  -  |theta|
    /// ```
    ///
    /// The differential time delay is `|tau(theta_1) - tau(theta_2)|`
    /// multiplied by the dimensional prefactor:
    ///
    /// ```text
    /// Delta_t = theta_E^2 * (D_l * D_s) / (c * D_ls) * |tau_1 - tau_2|
    /// ```
    ///
    /// Returns 0 if only one image exists.
    pub fn time_delay(&self, beta: f64) -> f64 {
        let images = self.image_positions(beta);
        if images.len() < 2 {
            return 0.0;
        }
        let t1 = images[0];
        let t2 = images[1];

        // Fermat potential: tau(theta) = (theta - beta)^2 / 2 - |theta|
        let tau1 = (t1 - beta).powi(2) / 2.0 - t1.abs();
        let tau2 = (t2 - beta).powi(2) / 2.0 - t2.abs();

        let te2 = self.theta_e * self.theta_e;
        let prefactor = te2 * self.d_l * self.d_s / (C_SI * self.d_ls);

        (prefactor * (tau1 - tau2)).abs()
    }
}

// ---------------------------------------------------------------------------
// ImageSolver
// ---------------------------------------------------------------------------

/// Numerical solver for image positions given arbitrary lens models.
///
/// Uses iterative root-finding (bisection + Newton-Raphson hybrid) to solve
/// the lens equation `beta = theta - alpha(theta)` for `theta`.
#[derive(Debug, Clone)]
pub struct ImageSolver {
    /// Convergence tolerance (relative).
    pub tolerance: f64,
    /// Maximum number of iterations.
    pub max_iter: usize,
}

impl ImageSolver {
    /// Create a new image solver with default parameters.
    pub fn new() -> Self {
        Self {
            tolerance: 1e-12,
            max_iter: 100,
        }
    }

    /// Create a solver with custom tolerance and iteration limit.
    pub fn with_params(tolerance: f64, max_iter: usize) -> Self {
        Self {
            tolerance,
            max_iter,
        }
    }

    /// Solve the point-mass lens equation for image positions.
    ///
    /// Finds roots of `f(theta) = theta - 1/theta - beta = 0`.
    ///
    /// # Arguments
    ///
    /// * `beta` — source position in Einstein radii
    ///
    /// # Returns
    ///
    /// Vector of image positions in Einstein radii.
    pub fn solve_point_mass(&self, beta: f64) -> Vec<f64> {
        // Analytic solution (used for validation / direct computation)
        if beta.abs() < 1e-15 {
            return vec![1.0];
        }
        let disc = (beta * beta + 4.0).sqrt();
        vec![(beta + disc) / 2.0, (beta - disc) / 2.0]
    }

    /// Solve the SIS lens equation for image positions.
    ///
    /// Finds roots of `f(theta) = theta - sign(theta) - beta = 0`.
    ///
    /// # Arguments
    ///
    /// * `beta` — source position in SIS Einstein radii
    ///
    /// # Returns
    ///
    /// Vector of image positions in SIS Einstein radii.
    pub fn solve_sis(&self, beta: f64) -> Vec<f64> {
        if beta.abs() < 1.0 - 1e-15 {
            vec![beta + 1.0, beta - 1.0]
        } else if beta >= 0.0 {
            vec![beta + 1.0]
        } else {
            vec![beta - 1.0]
        }
    }

    /// Solve a general 1D lens equation using Newton-Raphson.
    ///
    /// The lens equation is `beta = theta - alpha(theta)`, i.e., we find
    /// roots of `g(theta) = theta - alpha(theta) - beta`.
    ///
    /// # Arguments
    ///
    /// * `beta` — source position (normalised)
    /// * `alpha` — deflection function `alpha(theta)`
    /// * `alpha_prime` — derivative of deflection `d alpha / d theta`
    /// * `initial_guesses` — starting points for the Newton iteration
    ///
    /// # Returns
    ///
    /// Vector of unique image positions found from the initial guesses.
    pub fn solve_general(
        &self,
        beta: f64,
        alpha: &dyn Fn(f64) -> f64,
        alpha_prime: &dyn Fn(f64) -> f64,
        initial_guesses: &[f64],
    ) -> Vec<f64> {
        let mut roots = Vec::new();
        for &guess in initial_guesses {
            if let Some(root) = self.newton_raphson(beta, alpha, alpha_prime, guess) {
                // Check for duplicates
                let is_dup = roots.iter().any(|&r: &f64| (r - root).abs() < self.tolerance * 100.0);
                if !is_dup {
                    roots.push(root);
                }
            }
        }
        roots
    }

    /// Newton-Raphson iteration for `g(theta) = theta - alpha(theta) - beta = 0`.
    fn newton_raphson(
        &self,
        beta: f64,
        alpha: &dyn Fn(f64) -> f64,
        alpha_prime: &dyn Fn(f64) -> f64,
        initial: f64,
    ) -> Option<f64> {
        let mut theta = initial;
        for _ in 0..self.max_iter {
            let g = theta - alpha(theta) - beta;
            let g_prime = 1.0 - alpha_prime(theta);
            if g_prime.abs() < 1e-30 {
                return None; // Degenerate derivative
            }
            let step = g / g_prime;
            theta -= step;
            if step.abs() < self.tolerance * (1.0 + theta.abs()) {
                return Some(theta);
            }
        }
        None // Did not converge
    }
}

impl Default for ImageSolver {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// TimeDelayCalculator
// ---------------------------------------------------------------------------

/// Calculator for gravitational time delays between lensed images.
///
/// The time delay for a lensed image has two components:
///
/// 1. **Geometric delay** — the extra path length due to bending
/// 2. **Shapiro delay** — the gravitational time dilation near the lens
///
/// For a point mass these combine into:
///
/// ```text
/// tau(theta) = (1+z_l) * (D_l * D_s) / (c * D_ls) *
///              theta_E^2 * [ theta^2/2 - beta*theta - ln|theta| ]
/// ```
///
/// The *differential* time delay between two images is `|tau_1 - tau_2|`.
#[derive(Debug, Clone)]
pub struct TimeDelayCalculator {
    /// Lens configuration.
    pub config: LensConfig,
    /// Lens redshift (0 for nearby lenses).
    pub z_l: f64,
}

impl TimeDelayCalculator {
    /// Create a new time delay calculator.
    ///
    /// # Arguments
    ///
    /// * `config` — lens configuration
    /// * `z_l` — lens redshift (use 0 for nearby / non-cosmological)
    pub fn new(config: LensConfig, z_l: f64) -> Self {
        Self { config, z_l }
    }

    /// Compute the time delay function `tau(theta)` for a single image
    /// (relative to the unperturbed ray), in seconds.
    ///
    /// # Arguments
    ///
    /// * `theta` — image position in Einstein radii
    /// * `beta` — source position in Einstein radii
    pub fn tau(&self, theta: f64, beta: f64) -> f64 {
        let te2 = self.config.theta_e * self.config.theta_e;
        let prefactor = (1.0 + self.z_l)
            * (self.config.d_l * self.config.d_s)
            / (C_SI * self.config.d_ls);

        let geometric = theta * theta / 2.0 - beta * theta;
        let shapiro = -(theta.abs()).ln();

        prefactor * te2 * (geometric + shapiro)
    }

    /// Compute the differential time delay between two images (in seconds).
    ///
    /// # Arguments
    ///
    /// * `theta1` — position of image 1 in Einstein radii
    /// * `theta2` — position of image 2 in Einstein radii
    /// * `beta` — source position in Einstein radii
    pub fn differential_delay(&self, theta1: f64, theta2: f64, beta: f64) -> f64 {
        (self.tau(theta1, beta) - self.tau(theta2, beta)).abs()
    }

    /// Compute the time delay between all image pairs for a point-mass lens.
    ///
    /// # Arguments
    ///
    /// * `beta` — source position in Einstein radii
    ///
    /// # Returns
    ///
    /// Vector of `(image_i, image_j, delay_seconds)` tuples.
    pub fn all_delays(&self, beta: f64) -> Vec<(f64, f64, f64)> {
        let lens = PointMassLens::new(self.config.clone());
        let images = lens.image_positions(beta);
        let mut delays = Vec::new();
        for i in 0..images.len() {
            for j in (i + 1)..images.len() {
                let dt = self.differential_delay(images[i], images[j], beta);
                delays.push((images[i], images[j], dt));
            }
        }
        delays
    }
}

// ---------------------------------------------------------------------------
// MagnificationComputer
// ---------------------------------------------------------------------------

/// Computes the flux magnification of lensed images from the Jacobian
/// determinant of the lens mapping.
///
/// For an axially symmetric lens the magnification is:
///
/// ```text
/// mu = (theta / beta) * (d theta / d beta)
/// ```
///
/// For a point mass this simplifies to:
///
/// ```text
/// mu = 1 / (1 - (theta_E / theta)^4)
/// ```
#[derive(Debug, Clone)]
pub struct MagnificationComputer {
    _private: (),
}

impl MagnificationComputer {
    /// Create a new magnification computer.
    pub fn new() -> Self {
        Self { _private: () }
    }

    /// Compute the magnification for a point-mass lens image.
    ///
    /// # Arguments
    ///
    /// * `theta` — image position in Einstein radii
    ///
    /// # Returns
    ///
    /// Signed magnification (negative means parity-flipped image).
    pub fn point_mass_magnification(&self, theta: f64) -> f64 {
        let t4 = theta.powi(4);
        if t4.abs() < 1e-30 {
            return f64::INFINITY;
        }
        1.0 / (1.0 - 1.0 / t4)
    }

    /// Compute the absolute magnification for a point-mass lens image.
    pub fn point_mass_magnification_abs(&self, theta: f64) -> f64 {
        self.point_mass_magnification(theta).abs()
    }

    /// Compute the magnification for an SIS lens image.
    ///
    /// # Arguments
    ///
    /// * `theta` — image position in SIS Einstein radii
    ///
    /// # Returns
    ///
    /// Absolute magnification.
    pub fn sis_magnification(&self, theta: f64) -> f64 {
        let abs_theta = theta.abs();
        if (abs_theta - 1.0).abs() < 1e-15 {
            f64::INFINITY
        } else {
            abs_theta / (abs_theta - 1.0).abs()
        }
    }

    /// Compute the total magnification for a point-mass lens source.
    ///
    /// ```text
    /// mu_total = (u^2 + 2) / (u * sqrt(u^2 + 4))
    /// ```
    ///
    /// where `u = beta / theta_E` = `beta` (in normalised units).
    ///
    /// # Arguments
    ///
    /// * `beta` — source position in Einstein radii
    pub fn point_mass_total_magnification(&self, beta: f64) -> f64 {
        let u = beta.abs();
        if u < 1e-15 {
            return f64::INFINITY;
        }
        let u2 = u * u;
        (u2 + 2.0) / (u * (u2 + 4.0).sqrt())
    }

    /// Compute the convergence (kappa) for a point-mass lens.
    ///
    /// For a point mass, `kappa = 0` everywhere except at the origin (a delta function).
    /// This returns 0 for all finite theta.
    pub fn point_mass_convergence(&self, _theta: f64) -> f64 {
        0.0
    }

    /// Compute the shear (gamma) for a point-mass lens.
    ///
    /// ```text
    /// gamma = 1 / theta^2     (in Einstein radius units)
    /// ```
    pub fn point_mass_shear(&self, theta: f64) -> f64 {
        if theta.abs() < 1e-15 {
            return f64::INFINITY;
        }
        1.0 / (theta * theta)
    }

    /// Compute the Jacobian determinant `det(A) = (1 - kappa)^2 - gamma^2`
    /// for a point-mass lens.
    ///
    /// The magnification is `mu = 1 / |det(A)|`.
    pub fn point_mass_jacobian_det(&self, theta: f64) -> f64 {
        // For point mass: kappa=0, gamma = 1/theta^2
        // det(A) = 1 - gamma^2 = 1 - 1/theta^4
        1.0 - 1.0 / theta.powi(4)
    }
}

impl Default for MagnificationComputer {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const SOLAR_MASS: f64 = 1.989e30; // kg
    const KPC: f64 = 3.086e19; // meters

    fn test_config() -> LensConfig {
        // Solar-mass lens at 1 kpc, source at 2 kpc
        LensConfig::new(SOLAR_MASS, KPC, 2.0 * KPC)
    }

    // --- Helper function tests ---

    #[test]
    fn test_schwarzschild_radius_solar_mass() {
        let rs = schwarzschild_radius(SOLAR_MASS);
        // Expected ~2953 m for one solar mass
        assert!((rs - 2953.0).abs() < 5.0, "r_s = {rs}");
    }

    #[test]
    fn test_schwarzschild_radius_scales_linearly() {
        let rs1 = schwarzschild_radius(SOLAR_MASS);
        let rs10 = schwarzschild_radius(10.0 * SOLAR_MASS);
        assert!((rs10 / rs1 - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_einstein_radius_positive() {
        let te = einstein_radius(SOLAR_MASS, KPC, 2.0 * KPC);
        assert!(te > 0.0);
    }

    #[test]
    fn test_einstein_radius_scaling_with_mass() {
        // theta_E ~ sqrt(M), so doubling mass multiplies theta_E by sqrt(2)
        let te1 = einstein_radius(SOLAR_MASS, KPC, 2.0 * KPC);
        let te4 = einstein_radius(4.0 * SOLAR_MASS, KPC, 2.0 * KPC);
        assert!((te4 / te1 - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_deflection_angle_positive() {
        let alpha = deflection_angle(SOLAR_MASS, 1e10);
        assert!(alpha > 0.0);
    }

    #[test]
    fn test_deflection_angle_inversely_proportional_to_b() {
        let a1 = deflection_angle(SOLAR_MASS, 1e10);
        let a2 = deflection_angle(SOLAR_MASS, 2e10);
        assert!((a1 / a2 - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_critical_curve_equals_einstein_radius() {
        let te = einstein_radius(SOLAR_MASS, KPC, 2.0 * KPC);
        let cc = critical_curve(SOLAR_MASS, KPC, 2.0 * KPC);
        assert!((te - cc).abs() < 1e-20);
    }

    #[test]
    fn test_sis_deflection_angle_positive() {
        let alpha = sis_deflection_angle(200_000.0); // 200 km/s
        assert!(alpha > 0.0);
    }

    #[test]
    fn test_sis_einstein_radius_positive() {
        let te = sis_einstein_radius(200_000.0, KPC, 2.0 * KPC);
        assert!(te > 0.0);
    }

    // --- LensConfig tests ---

    #[test]
    fn test_lens_config_creation() {
        let cfg = test_config();
        assert!((cfg.mass - SOLAR_MASS).abs() < 1.0);
        assert!((cfg.d_l - KPC).abs() < 1.0);
        assert!((cfg.d_s - 2.0 * KPC).abs() < 1.0);
        assert!((cfg.d_ls - KPC).abs() < 1.0);
    }

    #[test]
    fn test_lens_config_einstein_radius_physical() {
        let cfg = test_config();
        let phys = cfg.einstein_radius_physical();
        // Physical Einstein radius = theta_E * D_l
        let expected = cfg.einstein_radius() * cfg.d_l;
        assert!((phys - expected).abs() / expected < 1e-10);
    }

    #[test]
    #[should_panic(expected = "Source must be behind the lens")]
    fn test_lens_config_panics_source_in_front() {
        LensConfig::new(SOLAR_MASS, 2.0 * KPC, KPC);
    }

    // --- PointMassLens tests ---

    #[test]
    fn test_point_mass_two_images() {
        let lens = PointMassLens::new(test_config());
        let images = lens.image_positions(0.5);
        assert_eq!(images.len(), 2);
    }

    #[test]
    fn test_point_mass_einstein_ring() {
        let lens = PointMassLens::new(test_config());
        let images = lens.image_positions(0.0);
        assert_eq!(images.len(), 1);
        assert!((images[0] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_point_mass_lens_equation_satisfied() {
        let lens = PointMassLens::new(test_config());
        let beta = 0.7;
        let images = lens.image_positions(beta);
        for &theta in &images {
            // lens eq: beta = theta - 1/theta
            let beta_check = theta - 1.0 / theta;
            assert!(
                (beta_check - beta).abs() < 1e-10,
                "Lens equation not satisfied: beta={beta}, got {beta_check}"
            );
        }
    }

    #[test]
    fn test_point_mass_images_opposite_sides() {
        let lens = PointMassLens::new(test_config());
        let images = lens.image_positions(0.5);
        // theta_+ > 0, theta_- < 0
        assert!(images[0] > 0.0);
        assert!(images[1] < 0.0);
    }

    #[test]
    fn test_point_mass_magnification_positive() {
        let lens = PointMassLens::new(test_config());
        let mags = lens.magnifications(0.5);
        assert_eq!(mags.len(), 2);
        for &m in &mags {
            assert!(m > 0.0);
        }
    }

    #[test]
    fn test_point_mass_total_magnification_formula() {
        let lens = PointMassLens::new(test_config());
        let beta = 0.5;
        let mu_total = lens.total_magnification(beta);
        // Analytic: mu_total = (u^2 + 2) / (u * sqrt(u^2 + 4))
        let u = beta.abs();
        let expected = (u * u + 2.0) / (u * (u * u + 4.0).sqrt());
        assert!(
            (mu_total - expected).abs() / expected < 1e-6,
            "mu_total={mu_total}, expected={expected}"
        );
    }

    #[test]
    fn test_point_mass_strongly_lensed() {
        let lens = PointMassLens::new(test_config());
        assert!(lens.is_strongly_lensed(0.5));
        assert!(!lens.is_strongly_lensed(1.5));
    }

    #[test]
    fn test_point_mass_time_delay_positive() {
        let lens = PointMassLens::new(test_config());
        let dt = lens.time_delay(0.5);
        assert!(dt > 0.0, "Time delay should be positive, got {dt}");
    }

    #[test]
    fn test_point_mass_time_delay_zero_for_ring() {
        let lens = PointMassLens::new(test_config());
        let dt = lens.time_delay(0.0);
        assert!((dt - 0.0).abs() < 1e-30, "Einstein ring should have zero delay");
    }

    // --- SIS lens tests ---

    #[test]
    fn test_sis_two_images_inside_ring() {
        let sis = SisLens::new(200_000.0, KPC, 2.0 * KPC);
        let images = sis.image_positions(0.5);
        assert_eq!(images.len(), 2);
    }

    #[test]
    fn test_sis_one_image_outside_ring() {
        let sis = SisLens::new(200_000.0, KPC, 2.0 * KPC);
        let images = sis.image_positions(1.5);
        assert_eq!(images.len(), 1);
    }

    #[test]
    fn test_sis_lens_equation_satisfied() {
        let sis = SisLens::new(200_000.0, KPC, 2.0 * KPC);
        let beta = 0.3;
        let images = sis.image_positions(beta);
        for &theta in &images {
            // SIS lens equation: beta = theta - sign(theta)
            let sign = if theta >= 0.0 { 1.0 } else { -1.0 };
            let beta_check = theta - sign;
            assert!(
                (beta_check - beta).abs() < 1e-10,
                "SIS lens equation not satisfied: beta={beta}, got {beta_check}"
            );
        }
    }

    #[test]
    fn test_sis_magnification_greater_than_one_inside_ring() {
        let sis = SisLens::new(200_000.0, KPC, 2.0 * KPC);
        let mags = sis.magnifications(0.3);
        for &m in &mags {
            assert!(m > 1.0, "SIS magnification inside ring should be > 1, got {m}");
        }
    }

    #[test]
    fn test_sis_time_delay_positive_inside_ring() {
        let sis = SisLens::new(200_000.0, KPC, 2.0 * KPC);
        let dt = sis.time_delay(0.5);
        assert!(dt > 0.0);
    }

    #[test]
    fn test_sis_time_delay_zero_outside_ring() {
        let sis = SisLens::new(200_000.0, KPC, 2.0 * KPC);
        let dt = sis.time_delay(1.5);
        assert!((dt - 0.0).abs() < 1e-30);
    }

    #[test]
    fn test_sis_strongly_lensed() {
        let sis = SisLens::new(200_000.0, KPC, 2.0 * KPC);
        assert!(sis.is_strongly_lensed(0.5));
        assert!(!sis.is_strongly_lensed(1.5));
    }

    // --- ImageSolver tests ---

    #[test]
    fn test_image_solver_point_mass_matches_analytic() {
        let solver = ImageSolver::new();
        let beta = 0.7;
        let solutions = solver.solve_point_mass(beta);
        // Check against direct formula
        let disc = (beta * beta + 4.0).sqrt();
        let expected_p = (beta + disc) / 2.0;
        let expected_m = (beta - disc) / 2.0;
        assert!((solutions[0] - expected_p).abs() < 1e-12);
        assert!((solutions[1] - expected_m).abs() < 1e-12);
    }

    #[test]
    fn test_image_solver_sis_matches() {
        let solver = ImageSolver::new();
        let beta = 0.4;
        let solutions = solver.solve_sis(beta);
        assert_eq!(solutions.len(), 2);
        assert!((solutions[0] - 1.4).abs() < 1e-12);
        assert!((solutions[1] - (-0.6)).abs() < 1e-12);
    }

    #[test]
    fn test_image_solver_general_newton_raphson() {
        let solver = ImageSolver::new();
        // Point mass: alpha(theta) = 1/theta
        let alpha = |theta: f64| 1.0 / theta;
        let alpha_prime = |theta: f64| -1.0 / (theta * theta);
        let beta = 0.5;
        let guesses = vec![2.0, -1.0];
        let roots = solver.solve_general(beta, &alpha, &alpha_prime, &guesses);
        assert_eq!(roots.len(), 2, "Should find 2 roots, found {}", roots.len());
        // Verify each root satisfies the lens equation
        for &r in &roots {
            let residual = r - 1.0 / r - beta;
            assert!(residual.abs() < 1e-10, "residual = {residual}");
        }
    }

    // --- TimeDelayCalculator tests ---

    #[test]
    fn test_time_delay_calculator_positive() {
        let cfg = test_config();
        let calc = TimeDelayCalculator::new(cfg, 0.0);
        let beta = 0.5;
        let delays = calc.all_delays(beta);
        assert_eq!(delays.len(), 1);
        assert!(delays[0].2 > 0.0);
    }

    #[test]
    fn test_time_delay_differential_symmetric() {
        let cfg = test_config();
        let calc = TimeDelayCalculator::new(cfg, 0.0);
        let dt_12 = calc.differential_delay(1.5, -0.3, 0.5);
        let dt_21 = calc.differential_delay(-0.3, 1.5, 0.5);
        assert!((dt_12 - dt_21).abs() < 1e-30);
    }

    #[test]
    fn test_time_delay_increases_with_redshift() {
        let cfg = test_config();
        let calc_z0 = TimeDelayCalculator::new(cfg.clone(), 0.0);
        let calc_z1 = TimeDelayCalculator::new(cfg, 1.0);
        let beta = 0.5;
        let dt_z0 = calc_z0.all_delays(beta)[0].2;
        let dt_z1 = calc_z1.all_delays(beta)[0].2;
        // (1+z_l) factor: dt_z1 should be 2x dt_z0
        assert!(
            (dt_z1 / dt_z0 - 2.0).abs() < 1e-6,
            "Ratio = {}",
            dt_z1 / dt_z0
        );
    }

    // --- MagnificationComputer tests ---

    #[test]
    fn test_magnification_computer_point_mass() {
        let mc = MagnificationComputer::new();
        // For theta = 2 (in Einstein radii): mu = 1/(1 - 1/16) = 16/15
        let mu = mc.point_mass_magnification(2.0);
        assert!((mu - 16.0 / 15.0).abs() < 1e-10);
    }

    #[test]
    fn test_magnification_computer_total() {
        let mc = MagnificationComputer::new();
        let beta = 1.0;
        let mu_total = mc.point_mass_total_magnification(beta);
        // (1 + 2) / (1 * sqrt(1 + 4)) = 3 / sqrt(5) ~ 1.3416
        let expected = 3.0 / 5.0_f64.sqrt();
        assert!((mu_total - expected).abs() < 1e-10);
    }

    #[test]
    fn test_magnification_diverges_at_einstein_ring() {
        let mc = MagnificationComputer::new();
        let mu = mc.point_mass_total_magnification(1e-20);
        assert!(mu > 1e10, "Magnification should diverge near beta=0");
    }

    #[test]
    fn test_point_mass_convergence_zero() {
        let mc = MagnificationComputer::new();
        assert!((mc.point_mass_convergence(1.5) - 0.0).abs() < 1e-15);
    }

    #[test]
    fn test_point_mass_shear() {
        let mc = MagnificationComputer::new();
        // gamma = 1/theta^2 => at theta=2, gamma = 0.25
        assert!((mc.point_mass_shear(2.0) - 0.25).abs() < 1e-12);
    }

    #[test]
    fn test_jacobian_det_matches_magnification() {
        let mc = MagnificationComputer::new();
        let theta = 1.5;
        let det = mc.point_mass_jacobian_det(theta);
        let mu = mc.point_mass_magnification(theta);
        // mu = 1/|det(A)|
        assert!(
            (mu - 1.0 / det.abs()).abs() < 1e-10,
            "mu={mu}, 1/|det|={}",
            1.0 / det.abs()
        );
    }

    #[test]
    fn test_sis_magnification() {
        let mc = MagnificationComputer::new();
        // theta = 1.5 => |theta|/(|theta|-1) = 1.5/0.5 = 3
        assert!((mc.sis_magnification(1.5) - 3.0).abs() < 1e-12);
    }

    #[test]
    fn test_default_impls() {
        let solver = ImageSolver::default();
        assert_eq!(solver.max_iter, 100);
        let mc = MagnificationComputer::default();
        assert!((mc.point_mass_convergence(1.0) - 0.0).abs() < 1e-15);
    }
}
