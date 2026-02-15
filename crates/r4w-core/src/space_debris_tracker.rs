// trace:FR-SPACE-DEBRIS | ai:claude
//! # Space Debris Orbit Determination and Conjunction Assessment
//!
//! This module implements space debris orbit determination and conjunction assessment
//! for maintaining a catalog of orbital objects and predicting close approaches.
//!
//! ## Overview
//!
//! Space debris tracking uses radar and optical observations to maintain a catalog
//! of orbital objects and predict close approaches (conjunctions). This module
//! provides:
//!
//! - **Orbital mechanics**: Keplerian element representation, Kepler equation solver,
//!   orbit propagation in ECI coordinates, J2 perturbation modeling
//! - **Conjunction assessment**: Miss distance calculation, time of closest approach
//!   search, collision probability estimation (simplified Alfriend method)
//! - **Catalog management**: Object storage, altitude-range queries, all-pairs
//!   conjunction screening
//! - **Atmospheric drag**: Exponential atmospheric density model, drag acceleration,
//!   orbital decay rate, and lifetime estimation
//!
//! ## Key Concepts
//!
//! - **Keplerian Elements**: Six parameters (a, e, i, Ω, ω, M) fully describe an
//!   orbit in inertial space at a given epoch
//! - **SGP4 Propagation**: Simplified General Perturbations model (this module uses
//!   a Keplerian + J2 approximation for educational purposes)
//! - **Conjunction Screening**: Systematic evaluation of all object pairs to identify
//!   potential close approaches below a distance threshold
//! - **Collision Probability**: Alfriend's method estimates probability of collision
//!   given miss distance, combined hard-body radius, and position uncertainty
//!
//! ## Constants
//!
//! - `MU_EARTH`: Earth gravitational parameter (3.986004418e5 km³/s²)
//! - `R_EARTH`: Mean Earth radius (6371.0 km)
//! - `J2`: Earth's second dynamic form factor (1.08263e-3)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::space_debris_tracker::*;
//!
//! // Create an ISS-like orbit
//! let iss_elements = OrbitalElements {
//!     semi_major_axis_km: 6778.0,
//!     eccentricity: 0.0001,
//!     inclination_deg: 51.6,
//!     raan_deg: 0.0,
//!     arg_perigee_deg: 0.0,
//!     mean_anomaly_deg: 0.0,
//!     epoch_jd: 2460000.5,
//! };
//!
//! let propagator = OrbitPropagator::new();
//! let period = propagator.orbital_period(iss_elements.semi_major_axis_km);
//! assert!((period / 60.0 - 92.4).abs() < 1.0); // ~92 minutes
//! ```

use std::f64::consts::PI;

/// Earth gravitational parameter in km³/s².
pub const MU_EARTH: f64 = 3.986004418e5;

/// Mean Earth radius in km.
pub const R_EARTH: f64 = 6371.0;

/// Earth's second dynamic form factor (oblateness).
pub const J2: f64 = 1.08263e-3;

/// Two pi constant for convenience.
const TWO_PI: f64 = 2.0 * PI;

// ---------------------------------------------------------------------------
// OrbitalElements
// ---------------------------------------------------------------------------

/// Classical Keplerian orbital elements defining an orbit at a given epoch.
///
/// These six parameters, together with the epoch, uniquely define the position
/// and velocity of an orbiting body in inertial space (assuming two-body motion).
#[derive(Debug, Clone, PartialEq)]
pub struct OrbitalElements {
    /// Semi-major axis in km. Defines the size of the orbit.
    pub semi_major_axis_km: f64,
    /// Eccentricity (0 = circular, <1 = elliptical). Defines the shape.
    pub eccentricity: f64,
    /// Orbital inclination in degrees. Angle between orbit plane and equator.
    pub inclination_deg: f64,
    /// Right Ascension of Ascending Node (RAAN) in degrees.
    /// Angle from vernal equinox to the ascending node.
    pub raan_deg: f64,
    /// Argument of perigee in degrees.
    /// Angle from ascending node to perigee in the orbit plane.
    pub arg_perigee_deg: f64,
    /// Mean anomaly in degrees at epoch. Parameterizes position along the orbit.
    pub mean_anomaly_deg: f64,
    /// Epoch as Julian Date. Reference time for the elements.
    pub epoch_jd: f64,
}

impl OrbitalElements {
    /// Returns the perigee altitude above Earth's surface in km.
    pub fn perigee_altitude_km(&self) -> f64 {
        self.semi_major_axis_km * (1.0 - self.eccentricity) - R_EARTH
    }

    /// Returns the apogee altitude above Earth's surface in km.
    pub fn apogee_altitude_km(&self) -> f64 {
        self.semi_major_axis_km * (1.0 + self.eccentricity) - R_EARTH
    }

    /// Returns inclination in radians.
    pub fn inclination_rad(&self) -> f64 {
        self.inclination_deg.to_radians()
    }

    /// Returns RAAN in radians.
    pub fn raan_rad(&self) -> f64 {
        self.raan_deg.to_radians()
    }

    /// Returns argument of perigee in radians.
    pub fn arg_perigee_rad(&self) -> f64 {
        self.arg_perigee_deg.to_radians()
    }

    /// Returns mean anomaly in radians.
    pub fn mean_anomaly_rad(&self) -> f64 {
        self.mean_anomaly_deg.to_radians()
    }
}

// ---------------------------------------------------------------------------
// ObjectType & DebrisObject
// ---------------------------------------------------------------------------

/// Classification of a tracked orbital object.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObjectType {
    /// Active or defunct satellite payload.
    Payload,
    /// Spent rocket body or upper stage.
    RocketBody,
    /// Fragment or debris piece.
    Debris,
    /// Unclassified object.
    Unknown,
}

/// A tracked orbital debris object with identification, orbital elements,
/// and physical characteristics.
#[derive(Debug, Clone)]
pub struct DebrisObject {
    /// NORAD catalog number (unique identifier).
    pub norad_id: u32,
    /// Object name or designation.
    pub name: String,
    /// Keplerian orbital elements at epoch.
    pub elements: OrbitalElements,
    /// Radar cross section in m² (characterizes detectability).
    pub rcs_m2: f64,
    /// Classification of the object.
    pub object_type: ObjectType,
}

// ---------------------------------------------------------------------------
// OrbitPropagator
// ---------------------------------------------------------------------------

/// Orbit propagation engine using Keplerian mechanics with J2 perturbations.
///
/// Provides methods for solving the Kepler equation, computing ECI position
/// and velocity vectors, and calculating J2 secular perturbation rates for
/// RAAN and argument of perigee.
///
/// # Note
///
/// This is a simplified propagator suitable for educational purposes and
/// short-term predictions. For operational conjunction assessment, a full
/// SGP4/SDP4 propagator with drag and higher-order perturbations is needed.
pub struct OrbitPropagator;

impl OrbitPropagator {
    /// Create a new orbit propagator.
    pub fn new() -> Self {
        OrbitPropagator
    }

    /// Solve Kepler's equation M = E - e*sin(E) for eccentric anomaly E.
    ///
    /// Uses Newton-Raphson iteration starting from M as the initial guess.
    ///
    /// # Arguments
    /// * `mean_anomaly_rad` - Mean anomaly M in radians
    /// * `eccentricity` - Orbital eccentricity e (must be < 1)
    /// * `tolerance` - Convergence tolerance in radians (typically 1e-12)
    ///
    /// # Returns
    /// Eccentric anomaly E in radians.
    pub fn kepler_equation(
        &self,
        mean_anomaly_rad: f64,
        eccentricity: f64,
        tolerance: f64,
    ) -> f64 {
        // Normalize mean anomaly to [0, 2π)
        let m = mean_anomaly_rad % TWO_PI;
        let m = if m < 0.0 { m + TWO_PI } else { m };
        let e = eccentricity;

        // Initial guess: E = M + e*sin(M) for low eccentricity
        let mut big_e = m + e * m.sin();

        // Newton-Raphson iteration: E_{n+1} = E_n - (E_n - e*sin(E_n) - M) / (1 - e*cos(E_n))
        for _ in 0..100 {
            let delta = (big_e - e * big_e.sin() - m) / (1.0 - e * big_e.cos());
            big_e -= delta;
            if delta.abs() < tolerance {
                break;
            }
        }

        big_e
    }

    /// Compute the orbital period in seconds.
    ///
    /// # Arguments
    /// * `a_km` - Semi-major axis in km
    ///
    /// # Returns
    /// Orbital period T = 2π√(a³/μ) in seconds.
    pub fn orbital_period(&self, a_km: f64) -> f64 {
        TWO_PI * (a_km.powi(3) / MU_EARTH).sqrt()
    }

    /// Compute mean motion (angular rate) in rad/s.
    ///
    /// # Arguments
    /// * `a_km` - Semi-major axis in km
    ///
    /// # Returns
    /// Mean motion n = √(μ/a³) in rad/s.
    pub fn mean_motion(&self, a_km: f64) -> f64 {
        (MU_EARTH / a_km.powi(3)).sqrt()
    }

    /// Compute position in ECI (Earth-Centered Inertial) coordinates.
    ///
    /// Propagates from the epoch by `time_since_epoch_s` seconds using Keplerian
    /// two-body dynamics. The returned coordinates are in km.
    ///
    /// # Arguments
    /// * `elements` - Orbital elements at epoch
    /// * `time_since_epoch_s` - Time since epoch in seconds
    ///
    /// # Returns
    /// Position vector [x, y, z] in ECI frame (km).
    pub fn position_in_orbit(
        &self,
        elements: &OrbitalElements,
        time_since_epoch_s: f64,
    ) -> [f64; 3] {
        let a = elements.semi_major_axis_km;
        let e = elements.eccentricity;
        let i = elements.inclination_rad();
        let raan = elements.raan_rad();
        let omega = elements.arg_perigee_rad();
        let m0 = elements.mean_anomaly_rad();

        // Mean motion
        let n = self.mean_motion(a);

        // Mean anomaly at time t
        let m = m0 + n * time_since_epoch_s;

        // Solve Kepler equation for eccentric anomaly
        let big_e = self.kepler_equation(m, e, 1e-12);

        // True anomaly
        let sin_nu = ((1.0 - e * e).sqrt() * big_e.sin()) / (1.0 - e * big_e.cos());
        let cos_nu = (big_e.cos() - e) / (1.0 - e * big_e.cos());
        let nu = sin_nu.atan2(cos_nu);

        // Distance from focus
        let r = a * (1.0 - e * big_e.cos());

        // Position in orbital plane (perifocal frame)
        let x_pf = r * nu.cos();
        let y_pf = r * nu.sin();

        // Rotation from perifocal to ECI
        // R = Rz(-RAAN) * Rx(-i) * Rz(-omega)
        let cos_o = omega.cos();
        let sin_o = omega.sin();
        let cos_r = raan.cos();
        let sin_r = raan.sin();
        let cos_i = i.cos();
        let sin_i = i.sin();

        let x_eci = (cos_r * cos_o - sin_r * sin_o * cos_i) * x_pf
            + (-cos_r * sin_o - sin_r * cos_o * cos_i) * y_pf;
        let y_eci = (sin_r * cos_o + cos_r * sin_o * cos_i) * x_pf
            + (-sin_r * sin_o + cos_r * cos_o * cos_i) * y_pf;
        let z_eci = (sin_o * sin_i) * x_pf + (cos_o * sin_i) * y_pf;

        [x_eci, y_eci, z_eci]
    }

    /// Compute velocity in ECI (Earth-Centered Inertial) coordinates.
    ///
    /// Uses the vis-viva equation and perifocal velocity components to compute
    /// the velocity vector in ECI frame.
    ///
    /// # Arguments
    /// * `elements` - Orbital elements at epoch
    /// * `time_since_epoch_s` - Time since epoch in seconds
    ///
    /// # Returns
    /// Velocity vector [vx, vy, vz] in ECI frame (km/s).
    pub fn velocity_in_orbit(
        &self,
        elements: &OrbitalElements,
        time_since_epoch_s: f64,
    ) -> [f64; 3] {
        let a = elements.semi_major_axis_km;
        let e = elements.eccentricity;
        let i = elements.inclination_rad();
        let raan = elements.raan_rad();
        let omega = elements.arg_perigee_rad();
        let m0 = elements.mean_anomaly_rad();

        // Mean motion
        let n = self.mean_motion(a);

        // Mean anomaly at time t
        let m = m0 + n * time_since_epoch_s;

        // Solve Kepler equation
        let big_e = self.kepler_equation(m, e, 1e-12);

        // Velocity components in perifocal frame
        // v_pf_x = -(n*a / (1 - e*cos(E))) * sin(E)
        // v_pf_y =  (n*a / (1 - e*cos(E))) * sqrt(1 - e^2) * cos(E)
        let denom = 1.0 - e * big_e.cos();
        let coeff = n * a / denom;

        let vx_pf = -coeff * big_e.sin();
        let vy_pf = coeff * (1.0 - e * e).sqrt() * big_e.cos();

        // Rotation from perifocal to ECI
        let cos_o = omega.cos();
        let sin_o = omega.sin();
        let cos_r = raan.cos();
        let sin_r = raan.sin();
        let cos_i = i.cos();
        let sin_i = i.sin();

        let vx_eci = (cos_r * cos_o - sin_r * sin_o * cos_i) * vx_pf
            + (-cos_r * sin_o - sin_r * cos_o * cos_i) * vy_pf;
        let vy_eci = (sin_r * cos_o + cos_r * sin_o * cos_i) * vx_pf
            + (-sin_r * sin_o + cos_r * cos_o * cos_i) * vy_pf;
        let vz_eci = (sin_o * sin_i) * vx_pf + (cos_o * sin_i) * vy_pf;

        [vx_eci, vy_eci, vz_eci]
    }

    /// Compute J2 secular perturbation rate for RAAN (nodal regression).
    ///
    /// The J2 oblateness of the Earth causes the orbital plane to precess.
    /// For prograde orbits (i < 90°), the RAAN drifts westward (negative rate).
    ///
    /// # Arguments
    /// * `a_km` - Semi-major axis in km
    /// * `e` - Eccentricity
    /// * `i_rad` - Inclination in radians
    ///
    /// # Returns
    /// RAAN drift rate in rad/s.
    pub fn j2_perturbation_raan_rate(&self, a_km: f64, e: f64, i_rad: f64) -> f64 {
        let n = self.mean_motion(a_km);
        let p = a_km * (1.0 - e * e); // semi-latus rectum
        let re_over_p = R_EARTH / p;
        -1.5 * n * J2 * re_over_p * re_over_p * i_rad.cos()
    }

    /// Compute J2 secular perturbation rate for argument of perigee.
    ///
    /// J2 also causes the line of apsides to rotate within the orbital plane.
    /// The rate depends on inclination: at the critical inclination (63.4°),
    /// the rate is zero (used by Molniya orbits).
    ///
    /// # Arguments
    /// * `a_km` - Semi-major axis in km
    /// * `e` - Eccentricity
    /// * `i_rad` - Inclination in radians
    ///
    /// # Returns
    /// Argument of perigee drift rate in rad/s.
    pub fn j2_perturbation_argp_rate(&self, a_km: f64, e: f64, i_rad: f64) -> f64 {
        let n = self.mean_motion(a_km);
        let p = a_km * (1.0 - e * e);
        let re_over_p = R_EARTH / p;
        1.5 * n * J2 * re_over_p * re_over_p * (2.0 - 2.5 * i_rad.sin().powi(2))
    }
}

impl Default for OrbitPropagator {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// ConjunctionAssessor
// ---------------------------------------------------------------------------

/// Conjunction assessment engine for evaluating collision risk between
/// orbital objects.
///
/// Provides methods for computing miss distance, time of closest approach,
/// collision probability, and generating conjunction summaries.
pub struct ConjunctionAssessor;

impl ConjunctionAssessor {
    /// Compute the Euclidean distance between two positions in km.
    ///
    /// # Arguments
    /// * `pos1` - First position [x, y, z] in km
    /// * `pos2` - Second position [x, y, z] in km
    ///
    /// # Returns
    /// Distance between the two positions in km.
    pub fn miss_distance(pos1: [f64; 3], pos2: [f64; 3]) -> f64 {
        let dx = pos1[0] - pos2[0];
        let dy = pos1[1] - pos2[1];
        let dz = pos1[2] - pos2[2];
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Find the time of closest approach between two objects by brute-force search.
    ///
    /// Scans from `t_start` to `t_end` in steps of `dt` seconds, then refines
    /// with a bisection-like refinement around the minimum.
    ///
    /// # Arguments
    /// * `obj1` - Orbital elements of the first object
    /// * `obj2` - Orbital elements of the second object
    /// * `t_start` - Start time in seconds since epoch
    /// * `t_end` - End time in seconds since epoch
    /// * `dt` - Search step size in seconds
    ///
    /// # Returns
    /// Tuple of (time_of_closest_approach_s, minimum_distance_km).
    pub fn time_of_closest_approach(
        obj1: &OrbitalElements,
        obj2: &OrbitalElements,
        t_start: f64,
        t_end: f64,
        dt: f64,
    ) -> (f64, f64) {
        let prop = OrbitPropagator::new();
        let mut best_t = t_start;
        let mut best_dist = f64::MAX;

        // Coarse search
        let mut t = t_start;
        while t <= t_end {
            let p1 = prop.position_in_orbit(obj1, t);
            let p2 = prop.position_in_orbit(obj2, t);
            let d = Self::miss_distance(p1, p2);
            if d < best_dist {
                best_dist = d;
                best_t = t;
            }
            t += dt;
        }

        // Fine refinement around best_t with golden section search
        let mut lo = (best_t - dt).max(t_start);
        let mut hi = (best_t + dt).min(t_end);
        let golden = 0.381966011250105; // (3 - sqrt(5)) / 2

        for _ in 0..50 {
            let span = hi - lo;
            if span < 1e-6 {
                break;
            }
            let t1 = lo + golden * span;
            let t2 = hi - golden * span;

            let p1a = prop.position_in_orbit(obj1, t1);
            let p2a = prop.position_in_orbit(obj2, t1);
            let d1 = Self::miss_distance(p1a, p2a);

            let p1b = prop.position_in_orbit(obj1, t2);
            let p2b = prop.position_in_orbit(obj2, t2);
            let d2 = Self::miss_distance(p1b, p2b);

            if d1 < d2 {
                hi = t2;
            } else {
                lo = t1;
            }
        }

        let final_t = (lo + hi) / 2.0;
        let p1 = prop.position_in_orbit(obj1, final_t);
        let p2 = prop.position_in_orbit(obj2, final_t);
        let final_d = Self::miss_distance(p1, p2);

        if final_d < best_dist {
            (final_t, final_d)
        } else {
            (best_t, best_dist)
        }
    }

    /// Compute simplified collision probability using the Alfriend method.
    ///
    /// Models the encounter as a 2D Gaussian probability distribution in the
    /// conjunction plane, with the combined hard-body radius defining the
    /// collision cross-section.
    ///
    /// # Arguments
    /// * `miss_distance_km` - Miss distance at TCA in km
    /// * `combined_hbr_km` - Combined hard-body radius in km
    /// * `pos_uncertainty_km` - Position uncertainty (1-sigma) in km
    ///
    /// # Returns
    /// Collision probability in range [0, 1].
    pub fn collision_probability(
        miss_distance_km: f64,
        combined_hbr_km: f64,
        pos_uncertainty_km: f64,
    ) -> f64 {
        if pos_uncertainty_km <= 0.0 {
            return if miss_distance_km <= combined_hbr_km {
                1.0
            } else {
                0.0
            };
        }

        // Simplified Alfriend: P_c = (r_hbr^2 / (2 * sigma^2)) * exp(-d^2 / (2 * sigma^2))
        let sigma2 = pos_uncertainty_km * pos_uncertainty_km;
        let r2 = combined_hbr_km * combined_hbr_km;
        let d2 = miss_distance_km * miss_distance_km;

        let prob = (r2 / (2.0 * sigma2)) * (-d2 / (2.0 * sigma2)).exp();

        // Clamp to [0, 1]
        prob.min(1.0).max(0.0)
    }

    /// Compute the screening volume for an object.
    ///
    /// The screening volume is a sphere centered on the object's position with
    /// the given threshold radius. Returns the volume in km³.
    ///
    /// # Arguments
    /// * `_obj` - Orbital elements (used for context; volume depends on threshold)
    /// * `threshold_km` - Screening threshold distance in km
    ///
    /// # Returns
    /// Screening volume in km³.
    pub fn screening_volume(_obj: &OrbitalElements, threshold_km: f64) -> f64 {
        (4.0 / 3.0) * PI * threshold_km.powi(3)
    }

    /// Generate a human-readable conjunction summary string.
    ///
    /// # Arguments
    /// * `obj1` - First debris object
    /// * `obj2` - Second debris object
    /// * `tca_s` - Time of closest approach in seconds since epoch
    /// * `miss_km` - Miss distance at TCA in km
    /// * `prob` - Collision probability
    ///
    /// # Returns
    /// Formatted conjunction summary string.
    pub fn conjunction_summary(
        obj1: &DebrisObject,
        obj2: &DebrisObject,
        tca_s: f64,
        miss_km: f64,
        prob: f64,
    ) -> String {
        let risk = if prob > 1e-4 {
            "HIGH"
        } else if prob > 1e-6 {
            "MEDIUM"
        } else {
            "LOW"
        };

        format!(
            "CONJUNCTION SUMMARY\n\
             ===================\n\
             Primary:   {} (NORAD {})\n\
             Secondary: {} (NORAD {})\n\
             TCA:       {:.1} s since epoch\n\
             Miss Dist: {:.3} km\n\
             Prob(Coll): {:.2e}\n\
             Risk Level: {}",
            obj1.name, obj1.norad_id, obj2.name, obj2.norad_id, tca_s, miss_km, prob, risk,
        )
    }
}

// ---------------------------------------------------------------------------
// CatalogManager
// ---------------------------------------------------------------------------

/// Catalog manager for maintaining and querying a collection of tracked
/// orbital debris objects.
///
/// Supports adding objects, querying by NORAD ID, filtering by altitude range,
/// and screening all pairs for potential conjunctions.
pub struct CatalogManager {
    objects: Vec<DebrisObject>,
}

impl CatalogManager {
    /// Create a new empty catalog.
    pub fn new() -> Self {
        CatalogManager {
            objects: Vec::new(),
        }
    }

    /// Add an object to the catalog.
    pub fn add_object(&mut self, obj: DebrisObject) {
        self.objects.push(obj);
    }

    /// Find an object by its NORAD catalog number.
    ///
    /// # Returns
    /// Reference to the object if found, None otherwise.
    pub fn find_by_id(&self, id: u32) -> Option<&DebrisObject> {
        self.objects.iter().find(|o| o.norad_id == id)
    }

    /// Find all objects with perigee altitude within a given range.
    ///
    /// # Arguments
    /// * `min_km` - Minimum altitude in km
    /// * `max_km` - Maximum altitude in km
    ///
    /// # Returns
    /// Vector of references to matching objects.
    pub fn objects_in_altitude_range(&self, min_km: f64, max_km: f64) -> Vec<&DebrisObject> {
        self.objects
            .iter()
            .filter(|o| {
                let alt = o.elements.perigee_altitude_km();
                alt >= min_km && alt <= max_km
            })
            .collect()
    }

    /// Screen all object pairs for potential conjunctions.
    ///
    /// For each pair, computes the miss distance at epoch (t=0) and returns
    /// pairs where the distance is below the threshold.
    ///
    /// # Arguments
    /// * `threshold_km` - Maximum distance to report in km
    ///
    /// # Returns
    /// Vector of (norad_id_1, norad_id_2, distance_km) tuples.
    pub fn screen_all_pairs(&self, threshold_km: f64) -> Vec<(u32, u32, f64)> {
        let prop = OrbitPropagator::new();
        let mut results = Vec::new();

        let n = self.objects.len();
        for i in 0..n {
            for j in (i + 1)..n {
                let p1 = prop.position_in_orbit(&self.objects[i].elements, 0.0);
                let p2 = prop.position_in_orbit(&self.objects[j].elements, 0.0);
                let d = ConjunctionAssessor::miss_distance(p1, p2);
                if d <= threshold_km {
                    results.push((self.objects[i].norad_id, self.objects[j].norad_id, d));
                }
            }
        }

        results
    }

    /// Returns the number of objects in the catalog.
    pub fn len(&self) -> usize {
        self.objects.len()
    }

    /// Returns true if the catalog is empty.
    pub fn is_empty(&self) -> bool {
        self.objects.is_empty()
    }
}

impl Default for CatalogManager {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// AtmosphericDrag
// ---------------------------------------------------------------------------

/// Atmospheric drag model for orbital decay estimation.
///
/// Uses an exponential atmospheric density model and provides methods to
/// compute drag acceleration, orbital decay rate, and estimated orbital lifetime.
///
/// # Atmospheric Model
///
/// The density model uses a piecewise exponential fit:
/// ρ(h) = ρ₀ * exp(-(h - h₀) / H)
///
/// where ρ₀ is the reference density at altitude h₀ and H is the scale height.
pub struct AtmosphericDrag;

impl AtmosphericDrag {
    /// Compute atmospheric density at a given altitude using an exponential model.
    ///
    /// Uses a piecewise exponential model with different scale heights for
    /// different altitude bands, providing a reasonable approximation of the
    /// NRLMSISE-00 atmosphere under moderate solar conditions.
    ///
    /// # Arguments
    /// * `altitude_km` - Altitude above Earth's surface in km
    ///
    /// # Returns
    /// Atmospheric density in kg/m³.
    pub fn atmospheric_density(altitude_km: f64) -> f64 {
        if altitude_km < 0.0 {
            return 1.225; // sea level
        }

        // Piecewise exponential model: (base_altitude_km, base_density_kg_m3, scale_height_km)
        let layers: &[(f64, f64, f64)] = &[
            (0.0, 1.225, 7.249),
            (25.0, 3.899e-2, 6.349),
            (30.0, 1.774e-2, 6.682),
            (40.0, 3.972e-3, 7.554),
            (50.0, 1.057e-3, 8.382),
            (60.0, 3.206e-4, 7.714),
            (70.0, 8.77e-5, 6.549),
            (80.0, 1.905e-5, 5.799),
            (90.0, 3.396e-6, 5.382),
            (100.0, 5.297e-7, 5.877),
            (110.0, 9.661e-8, 7.263),
            (120.0, 2.438e-8, 9.473),
            (130.0, 8.484e-9, 12.636),
            (140.0, 3.845e-9, 16.149),
            (150.0, 2.07e-9, 22.523),
            (180.0, 5.464e-10, 29.74),
            (200.0, 2.789e-10, 37.105),
            (250.0, 7.248e-11, 45.546),
            (300.0, 2.418e-11, 53.628),
            (350.0, 9.518e-12, 53.298),
            (400.0, 3.725e-12, 58.515),
            (450.0, 1.585e-12, 60.828),
            (500.0, 6.967e-13, 63.822),
            (600.0, 1.454e-13, 71.835),
            (700.0, 3.614e-14, 88.667),
            (800.0, 1.17e-14, 124.64),
            (900.0, 5.245e-15, 181.05),
            (1000.0, 3.019e-15, 268.0),
        ];

        // Find the appropriate layer
        let mut layer_idx = 0;
        for (idx, &(base_alt, _, _)) in layers.iter().enumerate() {
            if altitude_km >= base_alt {
                layer_idx = idx;
            } else {
                break;
            }
        }

        let (h0, rho0, scale_h) = layers[layer_idx];
        rho0 * (-(altitude_km - h0) / scale_h).exp()
    }

    /// Compute drag acceleration magnitude.
    ///
    /// # Arguments
    /// * `density` - Atmospheric density in kg/m³
    /// * `velocity` - Velocity magnitude in m/s
    /// * `cd` - Drag coefficient (typically 2.2 for satellites)
    /// * `area_m2` - Cross-sectional area in m²
    /// * `mass_kg` - Object mass in kg
    ///
    /// # Returns
    /// Drag acceleration magnitude in m/s².
    pub fn drag_acceleration(
        density: f64,
        velocity: f64,
        cd: f64,
        area_m2: f64,
        mass_kg: f64,
    ) -> f64 {
        if mass_kg <= 0.0 {
            return 0.0;
        }
        0.5 * density * velocity * velocity * cd * area_m2 / mass_kg
    }

    /// Compute the orbital decay rate (altitude loss rate) in km/day.
    ///
    /// Uses the approximation: dh/dt ≈ -π * a * ρ / B
    /// where B is the ballistic coefficient (mass / (Cd * A)) in kg/m².
    ///
    /// # Arguments
    /// * `altitude_km` - Current altitude in km
    /// * `ballistic_coeff` - Ballistic coefficient B = m/(Cd*A) in kg/m²
    ///
    /// # Returns
    /// Altitude loss rate in km/day (positive value means decreasing altitude).
    pub fn orbital_decay_rate(altitude_km: f64, ballistic_coeff: f64) -> f64 {
        if ballistic_coeff <= 0.0 {
            return 0.0;
        }
        let a = altitude_km + R_EARTH; // Semi-major axis (circular approx)
        let rho = Self::atmospheric_density(altitude_km);

        // Orbital velocity for circular orbit (m/s)
        let v = ((MU_EARTH * 1e9) / (a * 1e3)).sqrt(); // Convert km³/s² → m³/s², km → m

        // dh/dt = -ρ * v * a / B (approximate for circular orbit)
        // The factor accounts for continuous drag over one orbit
        let decay_m_per_s = rho * v * (a * 1e3) / ballistic_coeff;
        let decay_km_per_day = decay_m_per_s * 86400.0 / 1e3;

        decay_km_per_day
    }

    /// Estimate orbital lifetime in days.
    ///
    /// Integrates the decay rate from the current altitude down to 120 km
    /// (approximate re-entry altitude) using a simple forward Euler method.
    ///
    /// # Arguments
    /// * `altitude_km` - Initial altitude in km
    /// * `ballistic_coeff` - Ballistic coefficient B = m/(Cd*A) in kg/m²
    ///
    /// # Returns
    /// Estimated orbital lifetime in days.
    pub fn estimated_lifetime(altitude_km: f64, ballistic_coeff: f64) -> f64 {
        let reentry_alt = 120.0; // km
        if altitude_km <= reentry_alt {
            return 0.0;
        }

        let mut alt = altitude_km;
        let mut total_days = 0.0;
        let dt_days = 1.0; // 1-day steps

        // Forward Euler integration
        while alt > reentry_alt {
            let rate = Self::orbital_decay_rate(alt, ballistic_coeff);
            if rate <= 1e-20 {
                // Effectively no drag; return large value
                return total_days + 1e10;
            }
            alt -= rate * dt_days;
            total_days += dt_days;

            // Safety limit to prevent infinite loops
            if total_days > 1e8 {
                return total_days;
            }
        }

        total_days
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Helper: create ISS-like orbital elements.
    fn iss_elements() -> OrbitalElements {
        OrbitalElements {
            semi_major_axis_km: 6778.0, // ~407 km altitude
            eccentricity: 0.0001,
            inclination_deg: 51.6,
            raan_deg: 0.0,
            arg_perigee_deg: 0.0,
            mean_anomaly_deg: 0.0,
            epoch_jd: 2460000.5,
        }
    }

    /// Helper: create a GEO-like orbit.
    fn geo_elements() -> OrbitalElements {
        OrbitalElements {
            semi_major_axis_km: 42164.0,
            eccentricity: 0.0001,
            inclination_deg: 0.1,
            raan_deg: 75.0,
            arg_perigee_deg: 0.0,
            mean_anomaly_deg: 0.0,
            epoch_jd: 2460000.5,
        }
    }

    /// Helper: create a debris object.
    fn make_debris(
        id: u32,
        name: &str,
        elements: OrbitalElements,
        obj_type: ObjectType,
    ) -> DebrisObject {
        DebrisObject {
            norad_id: id,
            name: name.to_string(),
            elements,
            rcs_m2: 1.0,
            object_type: obj_type,
        }
    }

    // -----------------------------------------------------------------------
    // Kepler equation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_kepler_equation_circular_orbit() {
        // For circular orbit (e=0), E = M
        let prop = OrbitPropagator::new();
        for m_deg in [0.0_f64, 45.0, 90.0, 180.0, 270.0, 359.0] {
            let m = m_deg.to_radians();
            let e_anom = prop.kepler_equation(m, 0.0, 1e-12);
            assert!(
                (e_anom - m).abs() < 1e-10,
                "E should equal M for circular orbit at M={} deg",
                m_deg
            );
        }
    }

    #[test]
    fn test_kepler_equation_low_eccentricity() {
        let prop = OrbitPropagator::new();
        let e = 0.01;
        let m = 1.0_f64; // 1 radian

        let big_e = prop.kepler_equation(m, e, 1e-14);

        // Verify: M = E - e*sin(E)
        let m_check = big_e - e * big_e.sin();
        assert!(
            (m_check - m).abs() < 1e-12,
            "Kepler eq not satisfied: M={}, check={}",
            m,
            m_check
        );
    }

    #[test]
    fn test_kepler_equation_moderate_eccentricity() {
        let prop = OrbitPropagator::new();
        let e = 0.5;
        let m = PI / 3.0;

        let big_e = prop.kepler_equation(m, e, 1e-14);
        let m_check = big_e - e * big_e.sin();
        assert!(
            (m_check - m).abs() < 1e-12,
            "Kepler eq not satisfied for e=0.5"
        );
    }

    #[test]
    fn test_kepler_equation_high_eccentricity() {
        let prop = OrbitPropagator::new();
        let e = 0.9;
        let m = 0.5;

        let big_e = prop.kepler_equation(m, e, 1e-12);
        let m_check = big_e - e * big_e.sin();
        assert!(
            (m_check - m).abs() < 1e-10,
            "Kepler eq not satisfied for e=0.9"
        );
    }

    // -----------------------------------------------------------------------
    // Orbital period tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_orbital_period_iss() {
        let prop = OrbitPropagator::new();
        let period_s = prop.orbital_period(6778.0);
        let period_min = period_s / 60.0;
        // ISS period is approximately 92.4 minutes
        assert!(
            (period_min - 92.4).abs() < 1.0,
            "ISS period should be ~92.4 min, got {}",
            period_min
        );
    }

    #[test]
    fn test_orbital_period_geo() {
        let prop = OrbitPropagator::new();
        let period_s = prop.orbital_period(42164.0);
        let period_hours = period_s / 3600.0;
        // GEO period is approximately 24 hours (sidereal day ~23.93 hr)
        assert!(
            (period_hours - 23.93).abs() < 0.1,
            "GEO period should be ~23.93 hr, got {}",
            period_hours
        );
    }

    // -----------------------------------------------------------------------
    // Mean motion tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_mean_motion_equals_two_pi_over_period() {
        let prop = OrbitPropagator::new();
        let a = 7000.0;
        let n = prop.mean_motion(a);
        let t = prop.orbital_period(a);
        let n_from_t = TWO_PI / t;
        assert!(
            (n - n_from_t).abs() < 1e-12,
            "Mean motion should equal 2*pi/T"
        );
    }

    #[test]
    fn test_mean_motion_positive() {
        let prop = OrbitPropagator::new();
        let n = prop.mean_motion(8000.0);
        assert!(n > 0.0, "Mean motion must be positive");
    }

    // -----------------------------------------------------------------------
    // Position tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_position_magnitude_circular_orbit() {
        // For circular orbit, |r| ≈ a at all times
        let prop = OrbitPropagator::new();
        let elems = iss_elements();
        let a = elems.semi_major_axis_km;

        for t in [0.0, 1000.0, 2000.0, 4000.0] {
            let pos = prop.position_in_orbit(&elems, t);
            let r = (pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]).sqrt();
            assert!(
                (r - a).abs() / a < 0.001,
                "Position magnitude should ≈ a for circular orbit at t={}, r={}, a={}",
                t,
                r,
                a
            );
        }
    }

    #[test]
    fn test_position_at_epoch_near_perigee() {
        // Mean anomaly = 0 at epoch means satellite is near perigee
        let prop = OrbitPropagator::new();
        let elems = OrbitalElements {
            semi_major_axis_km: 7000.0,
            eccentricity: 0.1,
            inclination_deg: 0.0,
            raan_deg: 0.0,
            arg_perigee_deg: 0.0,
            mean_anomaly_deg: 0.0,
            epoch_jd: 2460000.5,
        };

        let pos = prop.position_in_orbit(&elems, 0.0);
        let r = (pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]).sqrt();
        let r_perigee = 7000.0 * (1.0 - 0.1);
        assert!(
            (r - r_perigee).abs() < 1.0,
            "At M=0, r should be perigee distance: r={}, r_p={}",
            r,
            r_perigee
        );
    }

    #[test]
    fn test_position_equatorial_orbit_in_xy_plane() {
        // i=0, RAAN=0, omega=0: orbit is in the x-y plane
        let prop = OrbitPropagator::new();
        let elems = OrbitalElements {
            semi_major_axis_km: 7000.0,
            eccentricity: 0.0,
            inclination_deg: 0.0,
            raan_deg: 0.0,
            arg_perigee_deg: 0.0,
            mean_anomaly_deg: 90.0,
            epoch_jd: 2460000.5,
        };

        let pos = prop.position_in_orbit(&elems, 0.0);
        assert!(
            pos[2].abs() < 1e-6,
            "Equatorial orbit should have z≈0, got z={}",
            pos[2]
        );
    }

    // -----------------------------------------------------------------------
    // Velocity tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_velocity_magnitude_circular_orbit() {
        // v_circ = sqrt(mu/a)
        let prop = OrbitPropagator::new();
        let a = 7000.0;
        let v_expected = (MU_EARTH / a).sqrt();

        let elems = OrbitalElements {
            semi_major_axis_km: a,
            eccentricity: 0.0,
            inclination_deg: 28.5,
            raan_deg: 0.0,
            arg_perigee_deg: 0.0,
            mean_anomaly_deg: 45.0,
            epoch_jd: 2460000.5,
        };

        let vel = prop.velocity_in_orbit(&elems, 0.0);
        let v = (vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]).sqrt();
        assert!(
            (v - v_expected).abs() / v_expected < 0.001,
            "Circular orbit velocity should be sqrt(mu/a): v={}, expected={}",
            v,
            v_expected
        );
    }

    #[test]
    fn test_velocity_perpendicular_to_position_circular() {
        // For circular orbit, v . r = 0
        let prop = OrbitPropagator::new();
        let elems = OrbitalElements {
            semi_major_axis_km: 7000.0,
            eccentricity: 0.0,
            inclination_deg: 45.0,
            raan_deg: 30.0,
            arg_perigee_deg: 0.0,
            mean_anomaly_deg: 60.0,
            epoch_jd: 2460000.5,
        };

        let pos = prop.position_in_orbit(&elems, 500.0);
        let vel = prop.velocity_in_orbit(&elems, 500.0);
        let dot = pos[0] * vel[0] + pos[1] * vel[1] + pos[2] * vel[2];
        let r = (pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]).sqrt();
        let v = (vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]).sqrt();
        let relative_dot = dot / (r * v);
        assert!(
            relative_dot.abs() < 0.01,
            "v . r should be ~0 for circular orbit, got {}",
            relative_dot
        );
    }

    // -----------------------------------------------------------------------
    // J2 perturbation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_j2_raan_rate_negative_for_prograde() {
        let prop = OrbitPropagator::new();
        // Prograde orbit: i < 90 deg → cos(i) > 0 → RAAN rate < 0
        let rate = prop.j2_perturbation_raan_rate(7000.0, 0.001, 51.6_f64.to_radians());
        assert!(
            rate < 0.0,
            "RAAN rate should be negative for prograde orbits, got {}",
            rate
        );
    }

    #[test]
    fn test_j2_raan_rate_positive_for_retrograde() {
        let prop = OrbitPropagator::new();
        // Retrograde orbit: i > 90 deg → cos(i) < 0 → RAAN rate > 0
        let rate = prop.j2_perturbation_raan_rate(7000.0, 0.001, 100.0_f64.to_radians());
        assert!(
            rate > 0.0,
            "RAAN rate should be positive for retrograde orbits, got {}",
            rate
        );
    }

    #[test]
    fn test_j2_raan_rate_zero_at_90_degrees() {
        let prop = OrbitPropagator::new();
        // At i=90 deg (polar orbit), cos(i)=0, so RAAN rate = 0
        let rate = prop.j2_perturbation_raan_rate(7000.0, 0.001, (PI / 2.0));
        assert!(
            rate.abs() < 1e-15,
            "RAAN rate should be zero for polar orbit, got {}",
            rate
        );
    }

    #[test]
    fn test_j2_argp_rate_zero_at_critical_inclination() {
        let prop = OrbitPropagator::new();
        // Critical inclination: sin²(i) = 4/5, i ≈ 63.43 deg
        let i_crit = (4.0_f64 / 5.0).sqrt().asin();
        let rate = prop.j2_perturbation_argp_rate(7000.0, 0.001, i_crit);
        assert!(
            rate.abs() < 1e-12,
            "Arg perigee rate should be ~0 at critical inclination, got {}",
            rate
        );
    }

    // -----------------------------------------------------------------------
    // Miss distance tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_miss_distance_same_position() {
        let pos = [1000.0, 2000.0, 3000.0];
        let d = ConjunctionAssessor::miss_distance(pos, pos);
        assert!(
            d.abs() < 1e-10,
            "Miss distance for same position should be 0"
        );
    }

    #[test]
    fn test_miss_distance_known_value() {
        let p1 = [0.0, 0.0, 0.0];
        let p2 = [3.0, 4.0, 0.0];
        let d = ConjunctionAssessor::miss_distance(p1, p2);
        assert!(
            (d - 5.0).abs() < 1e-10,
            "Miss distance should be 5 for 3-4-5 triangle"
        );
    }

    #[test]
    fn test_miss_distance_3d() {
        let p1 = [1.0, 2.0, 3.0];
        let p2 = [4.0, 6.0, 3.0];
        // d = sqrt(9 + 16 + 0) = 5
        let d = ConjunctionAssessor::miss_distance(p1, p2);
        assert!((d - 5.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Collision probability tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_collision_probability_in_range() {
        let p = ConjunctionAssessor::collision_probability(1.0, 0.01, 0.5);
        assert!(p >= 0.0 && p <= 1.0, "Probability should be in [0,1]");
    }

    #[test]
    fn test_collision_probability_increases_with_closer_approach() {
        let p_far = ConjunctionAssessor::collision_probability(10.0, 0.01, 1.0);
        let p_close = ConjunctionAssessor::collision_probability(0.1, 0.01, 1.0);
        assert!(
            p_close > p_far,
            "Closer approach should yield higher probability"
        );
    }

    #[test]
    fn test_collision_probability_zero_uncertainty_hit() {
        let p = ConjunctionAssessor::collision_probability(0.005, 0.01, 0.0);
        assert!(
            (p - 1.0).abs() < 1e-10,
            "Should be 1.0 when miss < hbr with zero uncertainty"
        );
    }

    #[test]
    fn test_collision_probability_zero_uncertainty_miss() {
        let p = ConjunctionAssessor::collision_probability(0.05, 0.01, 0.0);
        assert!(
            p.abs() < 1e-10,
            "Should be 0.0 when miss > hbr with zero uncertainty"
        );
    }

    #[test]
    fn test_collision_probability_larger_hbr_gives_higher_prob() {
        let p_small = ConjunctionAssessor::collision_probability(1.0, 0.001, 0.5);
        let p_large = ConjunctionAssessor::collision_probability(1.0, 0.1, 0.5);
        assert!(
            p_large > p_small,
            "Larger hard-body radius should increase probability"
        );
    }

    // -----------------------------------------------------------------------
    // Time of closest approach tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_tca_identical_orbits() {
        let elems = iss_elements();
        let (_, dist) =
            ConjunctionAssessor::time_of_closest_approach(&elems, &elems, 0.0, 6000.0, 10.0);
        assert!(
            dist < 0.01,
            "Identical orbits should have ~0 miss distance, got {}",
            dist
        );
    }

    #[test]
    fn test_tca_returns_valid_time() {
        let elem1 = iss_elements();
        let mut elem2 = iss_elements();
        elem2.raan_deg = 1.0; // slightly different plane
        let (tca, _) =
            ConjunctionAssessor::time_of_closest_approach(&elem1, &elem2, 0.0, 6000.0, 10.0);
        assert!(
            tca >= 0.0 && tca <= 6000.0,
            "TCA should be within search window"
        );
    }

    // -----------------------------------------------------------------------
    // Screening volume test
    // -----------------------------------------------------------------------

    #[test]
    fn test_screening_volume() {
        let elems = iss_elements();
        let vol = ConjunctionAssessor::screening_volume(&elems, 10.0);
        let expected = (4.0 / 3.0) * PI * 1000.0; // (4/3)*pi*10^3
        assert!(
            (vol - expected).abs() < 1e-6,
            "Screening volume should be (4/3)*pi*r^3"
        );
    }

    // -----------------------------------------------------------------------
    // Conjunction summary test
    // -----------------------------------------------------------------------

    #[test]
    fn test_conjunction_summary_format() {
        let obj1 = make_debris(25544, "ISS", iss_elements(), ObjectType::Payload);
        let obj2 = make_debris(99999, "DEBRIS-A", iss_elements(), ObjectType::Debris);
        let summary =
            ConjunctionAssessor::conjunction_summary(&obj1, &obj2, 3600.0, 0.5, 1e-5);
        assert!(summary.contains("ISS"));
        assert!(summary.contains("DEBRIS-A"));
        assert!(summary.contains("25544"));
        assert!(summary.contains("99999"));
        assert!(summary.contains("MEDIUM"));
    }

    #[test]
    fn test_conjunction_summary_high_risk() {
        let obj1 = make_debris(1, "OBJ1", iss_elements(), ObjectType::Payload);
        let obj2 = make_debris(2, "OBJ2", iss_elements(), ObjectType::Debris);
        let summary =
            ConjunctionAssessor::conjunction_summary(&obj1, &obj2, 100.0, 0.01, 1e-3);
        assert!(summary.contains("HIGH"));
    }

    #[test]
    fn test_conjunction_summary_low_risk() {
        let obj1 = make_debris(1, "OBJ1", iss_elements(), ObjectType::Payload);
        let obj2 = make_debris(2, "OBJ2", iss_elements(), ObjectType::Debris);
        let summary =
            ConjunctionAssessor::conjunction_summary(&obj1, &obj2, 100.0, 50.0, 1e-10);
        assert!(summary.contains("LOW"));
    }

    // -----------------------------------------------------------------------
    // Catalog manager tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_catalog_add_and_find() {
        let mut cat = CatalogManager::new();
        cat.add_object(make_debris(25544, "ISS", iss_elements(), ObjectType::Payload));

        let found = cat.find_by_id(25544);
        assert!(found.is_some());
        assert_eq!(found.unwrap().name, "ISS");
    }

    #[test]
    fn test_catalog_find_missing() {
        let cat = CatalogManager::new();
        assert!(cat.find_by_id(99999).is_none());
    }

    #[test]
    fn test_catalog_altitude_range() {
        let mut cat = CatalogManager::new();

        // ISS at ~407 km
        cat.add_object(make_debris(25544, "ISS", iss_elements(), ObjectType::Payload));

        // GEO at ~35793 km
        cat.add_object(make_debris(99999, "GEO-SAT", geo_elements(), ObjectType::Payload));

        let leo = cat.objects_in_altitude_range(200.0, 2000.0);
        assert_eq!(leo.len(), 1);
        assert_eq!(leo[0].name, "ISS");

        let geo = cat.objects_in_altitude_range(35000.0, 36000.0);
        assert_eq!(geo.len(), 1);
        assert_eq!(geo[0].name, "GEO-SAT");
    }

    #[test]
    fn test_catalog_screen_identical_objects() {
        let mut cat = CatalogManager::new();
        cat.add_object(make_debris(1, "OBJ-A", iss_elements(), ObjectType::Debris));
        cat.add_object(make_debris(2, "OBJ-B", iss_elements(), ObjectType::Debris));

        // Same orbit → distance ~0 at epoch
        let pairs = cat.screen_all_pairs(1.0);
        assert_eq!(pairs.len(), 1);
        assert!(pairs[0].2 < 0.01, "Same orbit should have ~0 distance");
    }

    #[test]
    fn test_catalog_screen_no_conjunctions() {
        let mut cat = CatalogManager::new();
        cat.add_object(make_debris(1, "LEO", iss_elements(), ObjectType::Debris));
        cat.add_object(make_debris(2, "GEO", geo_elements(), ObjectType::Payload));

        // LEO vs GEO are thousands of km apart
        let pairs = cat.screen_all_pairs(100.0);
        assert!(pairs.is_empty(), "LEO/GEO should not be within 100 km");
    }

    #[test]
    fn test_catalog_len_and_empty() {
        let mut cat = CatalogManager::new();
        assert!(cat.is_empty());
        assert_eq!(cat.len(), 0);

        cat.add_object(make_debris(1, "OBJ", iss_elements(), ObjectType::Unknown));
        assert!(!cat.is_empty());
        assert_eq!(cat.len(), 1);
    }

    // -----------------------------------------------------------------------
    // Atmospheric drag tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_atmospheric_density_decreases_with_altitude() {
        let rho_200 = AtmosphericDrag::atmospheric_density(200.0);
        let rho_400 = AtmosphericDrag::atmospheric_density(400.0);
        let rho_600 = AtmosphericDrag::atmospheric_density(600.0);
        assert!(
            rho_200 > rho_400,
            "Density at 200 km should exceed 400 km"
        );
        assert!(
            rho_400 > rho_600,
            "Density at 400 km should exceed 600 km"
        );
    }

    #[test]
    fn test_atmospheric_density_sea_level() {
        let rho = AtmosphericDrag::atmospheric_density(0.0);
        assert!(
            (rho - 1.225).abs() < 0.01,
            "Sea level density should be ~1.225 kg/m³"
        );
    }

    #[test]
    fn test_atmospheric_density_positive() {
        for alt in [100.0, 300.0, 500.0, 800.0, 1000.0] {
            let rho = AtmosphericDrag::atmospheric_density(alt);
            assert!(
                rho > 0.0,
                "Density should be positive at {} km, got {}",
                alt,
                rho
            );
        }
    }

    #[test]
    fn test_drag_acceleration_positive() {
        let a = AtmosphericDrag::drag_acceleration(1e-12, 7500.0, 2.2, 10.0, 1000.0);
        assert!(a > 0.0, "Drag acceleration should be positive");
    }

    #[test]
    fn test_drag_acceleration_zero_mass() {
        let a = AtmosphericDrag::drag_acceleration(1e-12, 7500.0, 2.2, 10.0, 0.0);
        assert!(
            a.abs() < 1e-20,
            "Drag acceleration should be 0 for zero mass"
        );
    }

    #[test]
    fn test_orbital_decay_rate_higher_at_lower_altitude() {
        let b = 50.0; // ballistic coefficient
        let rate_300 = AtmosphericDrag::orbital_decay_rate(300.0, b);
        let rate_500 = AtmosphericDrag::orbital_decay_rate(500.0, b);
        assert!(
            rate_300 > rate_500,
            "Decay rate at 300 km ({}) should exceed 500 km ({})",
            rate_300,
            rate_500
        );
    }

    #[test]
    fn test_estimated_lifetime_increases_with_altitude() {
        let b = 50.0;
        let life_300 = AtmosphericDrag::estimated_lifetime(300.0, b);
        let life_500 = AtmosphericDrag::estimated_lifetime(500.0, b);
        assert!(
            life_500 > life_300,
            "Lifetime at 500 km ({}) should exceed 300 km ({})",
            life_500,
            life_300
        );
    }

    #[test]
    fn test_estimated_lifetime_zero_below_reentry() {
        let life = AtmosphericDrag::estimated_lifetime(100.0, 50.0);
        assert!(
            life.abs() < 1e-10,
            "Lifetime below reentry altitude should be 0"
        );
    }

    // -----------------------------------------------------------------------
    // OrbitalElements helper tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_orbital_elements_perigee_apogee() {
        let elems = OrbitalElements {
            semi_major_axis_km: 7000.0,
            eccentricity: 0.1,
            inclination_deg: 0.0,
            raan_deg: 0.0,
            arg_perigee_deg: 0.0,
            mean_anomaly_deg: 0.0,
            epoch_jd: 2460000.5,
        };

        let perigee = elems.perigee_altitude_km();
        let apogee = elems.apogee_altitude_km();

        assert!(
            (perigee - (6300.0 - R_EARTH)).abs() < 0.1,
            "Perigee altitude wrong: {}",
            perigee
        );
        assert!(
            (apogee - (7700.0 - R_EARTH)).abs() < 0.1,
            "Apogee altitude wrong: {}",
            apogee
        );
        assert!(apogee > perigee, "Apogee must exceed perigee");
    }

    #[test]
    fn test_orbital_elements_angle_conversions() {
        let elems = OrbitalElements {
            semi_major_axis_km: 7000.0,
            eccentricity: 0.0,
            inclination_deg: 45.0,
            raan_deg: 90.0,
            arg_perigee_deg: 180.0,
            mean_anomaly_deg: 270.0,
            epoch_jd: 2460000.5,
        };

        assert!((elems.inclination_rad() - PI / 4.0).abs() < 1e-10);
        assert!((elems.raan_rad() - PI / 2.0).abs() < 1e-10);
        assert!((elems.arg_perigee_rad() - PI).abs() < 1e-10);
        assert!((elems.mean_anomaly_rad() - 3.0 * PI / 2.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // ObjectType and DebrisObject tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_object_type_enum() {
        assert_ne!(ObjectType::Payload, ObjectType::Debris);
        assert_ne!(ObjectType::RocketBody, ObjectType::Unknown);
        let t = ObjectType::Debris;
        let t2 = t; // Copy
        assert_eq!(t, t2);
    }

    #[test]
    fn test_debris_object_creation() {
        let obj = DebrisObject {
            norad_id: 25544,
            name: "ISS (ZARYA)".to_string(),
            elements: iss_elements(),
            rcs_m2: 400.0,
            object_type: ObjectType::Payload,
        };
        assert_eq!(obj.norad_id, 25544);
        assert_eq!(obj.rcs_m2, 400.0);
        assert_eq!(obj.object_type, ObjectType::Payload);
    }
}
