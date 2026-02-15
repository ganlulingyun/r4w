//! # Muon Tomography Reconstructor
//!
//! Cosmic ray muon tomography for non-invasive imaging of dense objects such as
//! cargo containers, nuclear waste canisters, pyramids, and volcanoes.
//!
//! Atmospheric muons are produced when cosmic ray protons interact with nuclei
//! high in the atmosphere, producing pions and kaons that decay into muons.
//! At sea level the flux is approximately 1 muon/cm^2/min at the zenith, with
//! a mean energy of about 3-4 GeV and a spectrum falling roughly as E^{-2.7}.
//!
//! ## Physics Background
//!
//! - **Multiple Coulomb Scattering (MCS)**: A charged particle traversing
//!   matter undergoes many small-angle deflections from nuclear Coulomb fields.
//!   The RMS projected scattering angle is given by the Highland formula:
//!   theta_rms = (13.6 MeV / (beta*c*p)) * sqrt(x/X0) * (1 + 0.038*ln(x/X0))
//!   where p = momentum, x = thickness, X0 = radiation length.
//!
//! - **Scattering Tomography**: High-Z materials scatter muons more than low-Z
//!   materials because X0 is shorter. By measuring the scattering angle of
//!   each muon (incoming vs outgoing direction), one can reconstruct a 3D map
//!   of scattering density lambda = Z^2 / X0.
//!
//! - **Transmission (Absorption) Tomography**: Alternatively, counting muons
//!   that pass through a volume and comparing to the expected flux yields an
//!   opacity map. This is the technique used for volcano and pyramid imaging.
//!
//! - **POCA (Point of Closest Approach)**: The simplest reconstruction method.
//!   Given the incoming and outgoing rays for each muon, the closest point
//!   between the two skew lines estimates the scattering vertex location.
//!
//! - **MLEM (Maximum Likelihood Expectation Maximization)**: An iterative
//!   statistical reconstruction that models the full muon path through the
//!   voxel grid and updates scattering densities to maximize likelihood.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::muon_tomography_reconstructor::*;
//!
//! let config = DetectorConfig::default();
//! let mut recon = MuonTomographyReconstructor::new(config);
//!
//! // Highland formula: RMS scattering angle in iron at 3 GeV
//! let theta = highland_theta_rms(3.0, RadiationLength::IRON, 0.10);
//! assert!(theta > 0.0 && theta < 0.1);
//!
//! // Sea-level muon flux at zenith
//! let flux = muon_flux_at_zenith();
//! assert!((flux - 1.0).abs() < 0.1); // ~1 muon/cm^2/min
//! ```

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Highland formula constant (MeV).
const HIGHLAND_CONSTANT_MEV: f64 = 13.6;

/// Muon rest mass (GeV/c^2).
const MUON_MASS_GEV: f64 = 0.10566;

/// Sea-level vertical muon flux (muons / cm^2 / min).
const SEA_LEVEL_MUON_FLUX: f64 = 1.0;

/// Mean muon energy at sea level (GeV).
const MEAN_MUON_ENERGY_GEV: f64 = 3.5;

// ---------------------------------------------------------------------------
// Radiation lengths
// ---------------------------------------------------------------------------

/// Radiation lengths for common materials (in metres).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RadiationLength;

impl RadiationLength {
    /// Air: 303.9 m
    pub const AIR: f64 = 303.9;
    /// Water: 0.361 m (36.1 cm)
    pub const WATER: f64 = 0.361;
    /// Iron: 0.0176 m (1.76 cm)
    pub const IRON: f64 = 0.0176;
    /// Lead: 0.0056 m (0.56 cm)
    pub const LEAD: f64 = 0.0056;
    /// Uranium: 0.0032 m (0.32 cm)
    pub const URANIUM: f64 = 0.0032;
    /// Concrete: 0.1155 m (11.55 cm)
    pub const CONCRETE: f64 = 0.1155;
}

// ---------------------------------------------------------------------------
// Material classification
// ---------------------------------------------------------------------------

/// Material class inferred from scattering density lambda.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MaterialClass {
    /// Air or void (very low scattering).
    Air,
    /// Light material such as water, plastic, wood.
    Light,
    /// Medium material such as concrete, aluminium.
    Medium,
    /// Dense material such as iron, copper.
    Dense,
    /// Very dense / high-Z such as lead.
    HighZ,
    /// Special nuclear material: uranium, plutonium.
    SpecialNuclear,
}

/// Classify material from scattering density lambda (rad^2 / m).
///
/// Lambda is proportional to Z^2 / X0; higher values indicate denser, higher-Z
/// materials.
pub fn classify_material(lambda: f64) -> MaterialClass {
    if lambda < 1.0e-6 {
        MaterialClass::Air
    } else if lambda < 1.0e-3 {
        MaterialClass::Light
    } else if lambda < 5.0e-2 {
        MaterialClass::Medium
    } else if lambda < 5.0e-1 {
        MaterialClass::Dense
    } else if lambda < 5.0 {
        MaterialClass::HighZ
    } else {
        MaterialClass::SpecialNuclear
    }
}

// ---------------------------------------------------------------------------
// 3D vector helpers
// ---------------------------------------------------------------------------

/// Simple 3D point / vector.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    pub fn dot(&self, other: &Vec3) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(&self, other: &Vec3) -> Vec3 {
        Vec3 {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    pub fn norm(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn norm_sq(&self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn normalized(&self) -> Vec3 {
        let n = self.norm();
        if n < 1.0e-30 {
            return *self;
        }
        Vec3::new(self.x / n, self.y / n, self.z / n)
    }

    pub fn sub(&self, other: &Vec3) -> Vec3 {
        Vec3::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }

    pub fn add(&self, other: &Vec3) -> Vec3 {
        Vec3::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }

    pub fn scale(&self, s: f64) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }

    /// Direction from spherical angles (theta=polar from +z, phi=azimuthal from +x).
    pub fn from_angles(theta: f64, phi: f64) -> Vec3 {
        Vec3::new(theta.sin() * phi.cos(), theta.sin() * phi.sin(), theta.cos())
    }

    /// Polar angle theta from +z axis.
    pub fn theta(&self) -> f64 {
        let n = self.norm();
        if n < 1.0e-30 {
            return 0.0;
        }
        (self.z / n).acos()
    }

    /// Azimuthal angle phi in x-y plane.
    pub fn phi(&self) -> f64 {
        self.y.atan2(self.x)
    }
}

// ---------------------------------------------------------------------------
// Muon track
// ---------------------------------------------------------------------------

/// A single muon track recorded by the detector.
#[derive(Debug, Clone)]
pub struct MuonTrack {
    /// Entry point on top detector plane (metres).
    pub entry_point: Vec3,
    /// Entry direction (unit vector, generally pointing downward: z < 0).
    pub entry_direction: Vec3,
    /// Exit point on bottom detector plane (metres).
    pub exit_point: Vec3,
    /// Exit direction (unit vector).
    pub exit_direction: Vec3,
    /// 3D scattering angle between entry and exit directions (radians).
    pub scattering_angle_rad: f64,
    /// Muon momentum (GeV/c). Zero means unknown.
    pub momentum_gev: f64,
}

impl MuonTrack {
    /// Compute the 3D scattering angle from the entry and exit direction vectors.
    pub fn compute_scattering_angle(entry_dir: &Vec3, exit_dir: &Vec3) -> f64 {
        let d1 = entry_dir.normalized();
        let d2 = exit_dir.normalized();
        let cos_angle = d1.dot(&d2).clamp(-1.0, 1.0);
        cos_angle.acos()
    }

    /// Create a track and auto-compute the scattering angle.
    pub fn new(
        entry_point: Vec3,
        entry_direction: Vec3,
        exit_point: Vec3,
        exit_direction: Vec3,
        momentum_gev: f64,
    ) -> Self {
        let scattering_angle_rad =
            Self::compute_scattering_angle(&entry_direction, &exit_direction);
        Self {
            entry_point,
            entry_direction: entry_direction.normalized(),
            exit_point,
            exit_direction: exit_direction.normalized(),
            scattering_angle_rad,
            momentum_gev,
        }
    }
}

// ---------------------------------------------------------------------------
// Detector configuration
// ---------------------------------------------------------------------------

/// Configuration for the muon tomography detector / reconstruction volume.
#[derive(Debug, Clone)]
pub struct DetectorConfig {
    /// Z coordinate of the top detector plane (metres).
    pub top_detector_z: f64,
    /// Z coordinate of the bottom detector plane (metres).
    pub bottom_detector_z: f64,
    /// Voxel side length (metres), cubical voxels.
    pub voxel_size_m: f64,
    /// Number of voxels in x.
    pub grid_nx: usize,
    /// Number of voxels in y.
    pub grid_ny: usize,
    /// Number of voxels in z.
    pub grid_nz: usize,
    /// Origin of the voxel grid (minimum corner) in world coordinates.
    pub grid_origin: Vec3,
}

impl Default for DetectorConfig {
    fn default() -> Self {
        Self {
            top_detector_z: 1.0,
            bottom_detector_z: -1.0,
            voxel_size_m: 0.05,
            grid_nx: 20,
            grid_ny: 20,
            grid_nz: 40,
            grid_origin: Vec3::new(-0.5, -0.5, -1.0),
        }
    }
}

impl DetectorConfig {
    /// Total number of voxels.
    pub fn num_voxels(&self) -> usize {
        self.grid_nx * self.grid_ny * self.grid_nz
    }

    /// Convert (ix, iy, iz) to flat index.
    pub fn voxel_index(&self, ix: usize, iy: usize, iz: usize) -> usize {
        iz * self.grid_nx * self.grid_ny + iy * self.grid_nx + ix
    }

    /// Convert flat index to (ix, iy, iz).
    pub fn voxel_coords(&self, idx: usize) -> (usize, usize, usize) {
        let nxy = self.grid_nx * self.grid_ny;
        let iz = idx / nxy;
        let rem = idx % nxy;
        let iy = rem / self.grid_nx;
        let ix = rem % self.grid_nx;
        (ix, iy, iz)
    }

    /// Centre of voxel (ix, iy, iz) in world coordinates.
    pub fn voxel_center(&self, ix: usize, iy: usize, iz: usize) -> Vec3 {
        let s = self.voxel_size_m;
        Vec3::new(
            self.grid_origin.x + (ix as f64 + 0.5) * s,
            self.grid_origin.y + (iy as f64 + 0.5) * s,
            self.grid_origin.z + (iz as f64 + 0.5) * s,
        )
    }

    /// Check if a world point falls inside the voxel grid and return voxel indices.
    pub fn point_to_voxel(&self, p: &Vec3) -> Option<(usize, usize, usize)> {
        let s = self.voxel_size_m;
        let fx = (p.x - self.grid_origin.x) / s;
        let fy = (p.y - self.grid_origin.y) / s;
        let fz = (p.z - self.grid_origin.z) / s;
        if fx < 0.0
            || fy < 0.0
            || fz < 0.0
            || fx >= self.grid_nx as f64
            || fy >= self.grid_ny as f64
            || fz >= self.grid_nz as f64
        {
            return None;
        }
        Some((fx as usize, fy as usize, fz as usize))
    }
}

// ---------------------------------------------------------------------------
// Highland formula
// ---------------------------------------------------------------------------

/// Highland formula: RMS projected scattering angle (radians).
///
/// * `momentum_gev` - Muon momentum in GeV/c.
/// * `radiation_length_m` - X0 of the material in metres.
/// * `thickness_m` - Material thickness traversed in metres.
///
/// Returns theta_rms = (13.6 MeV / (beta*p*c)) * sqrt(x/X0) * (1 + 0.038*ln(x/X0))
///
/// We assume beta ~ 1 for relativistic muons (E >> m_mu c^2).
pub fn highland_theta_rms(momentum_gev: f64, radiation_length_m: f64, thickness_m: f64) -> f64 {
    if momentum_gev <= 0.0 || radiation_length_m <= 0.0 || thickness_m <= 0.0 {
        return 0.0;
    }
    // beta = p / E, E = sqrt(p^2 + m^2)
    let energy = (momentum_gev * momentum_gev + MUON_MASS_GEV * MUON_MASS_GEV).sqrt();
    let beta = momentum_gev / energy;
    let x_over_x0 = thickness_m / radiation_length_m;
    let log_term = if x_over_x0 > 0.0 {
        1.0 + 0.038 * x_over_x0.ln()
    } else {
        1.0
    };
    // 13.6 MeV = 0.0136 GeV
    let theta = (0.0136 / (beta * momentum_gev)) * x_over_x0.sqrt() * log_term;
    theta.abs()
}

/// Estimate muon momentum from measured scattering angle (crude).
///
/// Inverts the Highland formula (ignoring the log correction) to give
/// p ~ 13.6 * sqrt(x/X0) / theta.
pub fn estimate_momentum_from_scattering(
    scattering_angle_rad: f64,
    radiation_length_m: f64,
    thickness_m: f64,
) -> f64 {
    if scattering_angle_rad <= 0.0 || radiation_length_m <= 0.0 || thickness_m <= 0.0 {
        return 0.0;
    }
    let x_over_x0 = thickness_m / radiation_length_m;
    0.0136 * x_over_x0.sqrt() / scattering_angle_rad
}

// ---------------------------------------------------------------------------
// Muon flux model
// ---------------------------------------------------------------------------

/// Sea-level vertical muon flux in muons / cm^2 / min.
pub fn muon_flux_at_zenith() -> f64 {
    SEA_LEVEL_MUON_FLUX
}

/// Muon flux at zenith angle theta: I(theta) = I0 * cos^2(theta).
///
/// Valid for theta < ~70 degrees.
pub fn muon_flux_at_angle(theta_rad: f64) -> f64 {
    let c = theta_rad.cos();
    SEA_LEVEL_MUON_FLUX * c * c
}

/// Differential energy spectrum: dN/dE ~ E^{-2.7} (approximate).
///
/// Returns the relative flux at energy `e_gev` normalised to 1 GeV.
pub fn muon_energy_spectrum(e_gev: f64) -> f64 {
    if e_gev <= 0.0 {
        return 0.0;
    }
    e_gev.powf(-2.7)
}

/// Exposure time (minutes) for `n_muons` through an area `area_cm2` at zenith.
pub fn exposure_time_for_count(n_muons: f64, area_cm2: f64) -> f64 {
    if area_cm2 <= 0.0 {
        return f64::INFINITY;
    }
    n_muons / (SEA_LEVEL_MUON_FLUX * area_cm2)
}

// ---------------------------------------------------------------------------
// POCA (Point of Closest Approach)
// ---------------------------------------------------------------------------

/// Result of a POCA calculation.
#[derive(Debug, Clone)]
pub struct PocaResult {
    /// The estimated scattering vertex (midpoint of closest approach).
    pub poca_point: Vec3,
    /// Distance of closest approach between the two rays.
    pub distance: f64,
    /// Parameter along the incoming ray.
    pub t_in: f64,
    /// Parameter along the outgoing ray.
    pub t_out: f64,
}

/// Compute the Point of Closest Approach for two skew lines (rays).
///
/// Incoming ray: P1 + t * D1
/// Outgoing ray: P2 + s * D2
///
/// Returns the midpoint of the segment of closest approach and the distance.
pub fn poca(
    p1: &Vec3,
    d1: &Vec3,
    p2: &Vec3,
    d2: &Vec3,
) -> PocaResult {
    let w0 = p1.sub(p2);
    let a = d1.dot(d1);
    let b = d1.dot(d2);
    let c = d2.dot(d2);
    let d = d1.dot(&w0);
    let e = d2.dot(&w0);

    let denom = a * c - b * b;

    let (t, s) = if denom.abs() < 1.0e-12 {
        // Parallel lines
        (0.0, e / c)
    } else {
        let t = (b * e - c * d) / denom;
        let s = (a * e - b * d) / denom;
        (t, s)
    };

    let closest_on_1 = p1.add(&d1.scale(t));
    let closest_on_2 = p2.add(&d2.scale(s));

    let midpoint = Vec3::new(
        0.5 * (closest_on_1.x + closest_on_2.x),
        0.5 * (closest_on_1.y + closest_on_2.y),
        0.5 * (closest_on_1.z + closest_on_2.z),
    );

    let dist = closest_on_1.sub(&closest_on_2).norm();

    PocaResult {
        poca_point: midpoint,
        distance: dist,
        t_in: t,
        t_out: s,
    }
}

// ---------------------------------------------------------------------------
// 3D voxel ray traversal (Amanatides & Woo)
// ---------------------------------------------------------------------------

/// A voxel hit during ray traversal with the path length through it.
#[derive(Debug, Clone)]
pub struct VoxelHit {
    pub ix: usize,
    pub iy: usize,
    pub iz: usize,
    /// Path length through this voxel (metres).
    pub length: f64,
}

/// Trace a ray through the voxel grid and return all voxels intersected
/// with path lengths (Amanatides & Woo-style).
///
/// The ray is defined by `origin + t * direction`.
pub fn trace_ray_through_grid(
    config: &DetectorConfig,
    origin: &Vec3,
    direction: &Vec3,
) -> Vec<VoxelHit> {
    let mut hits = Vec::new();
    let s = config.voxel_size_m;
    let dir = direction.normalized();

    // Grid bounds in world coords
    let x_min = config.grid_origin.x;
    let y_min = config.grid_origin.y;
    let z_min = config.grid_origin.z;
    let x_max = x_min + config.grid_nx as f64 * s;
    let y_max = y_min + config.grid_ny as f64 * s;
    let z_max = z_min + config.grid_nz as f64 * s;

    // Find entry and exit t for the AABB
    let (mut t_near, mut t_far) = (f64::NEG_INFINITY, f64::INFINITY);

    for i in 0..3 {
        let (o, d, lo, hi) = match i {
            0 => (origin.x, dir.x, x_min, x_max),
            1 => (origin.y, dir.y, y_min, y_max),
            _ => (origin.z, dir.z, z_min, z_max),
        };
        if d.abs() < 1.0e-15 {
            if o < lo || o > hi {
                return hits; // parallel and outside
            }
        } else {
            let inv_d = 1.0 / d;
            let mut t1 = (lo - o) * inv_d;
            let mut t2 = (hi - o) * inv_d;
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
            }
            t_near = t_near.max(t1);
            t_far = t_far.min(t2);
            if t_near > t_far {
                return hits;
            }
        }
    }

    if t_far < 0.0 {
        return hits;
    }
    t_near = t_near.max(0.0);

    // Starting point
    let start = origin.add(&dir.scale(t_near + 1.0e-9));

    let fx = ((start.x - x_min) / s).floor();
    let fy = ((start.y - y_min) / s).floor();
    let fz = ((start.z - z_min) / s).floor();

    let mut ix = (fx as isize).clamp(0, config.grid_nx as isize - 1) as usize;
    let mut iy = (fy as isize).clamp(0, config.grid_ny as isize - 1) as usize;
    let mut iz = (fz as isize).clamp(0, config.grid_nz as isize - 1) as usize;

    let step_x: isize = if dir.x >= 0.0 { 1 } else { -1 };
    let step_y: isize = if dir.y >= 0.0 { 1 } else { -1 };
    let step_z: isize = if dir.z >= 0.0 { 1 } else { -1 };

    let next_boundary = |idx: usize, step: isize, origin_coord: f64, min_coord: f64| -> f64 {
        let boundary = if step > 0 {
            min_coord + (idx as f64 + 1.0) * s
        } else {
            min_coord + idx as f64 * s
        };
        boundary - origin_coord
    };

    let t_max_x = if dir.x.abs() > 1.0e-15 {
        next_boundary(ix, step_x, origin.x, x_min) / dir.x
    } else {
        f64::INFINITY
    };
    let t_max_y = if dir.y.abs() > 1.0e-15 {
        next_boundary(iy, step_y, origin.y, y_min) / dir.y
    } else {
        f64::INFINITY
    };
    let t_max_z = if dir.z.abs() > 1.0e-15 {
        next_boundary(iz, step_z, origin.z, z_min) / dir.z
    } else {
        f64::INFINITY
    };

    let mut t_max = [t_max_x, t_max_y, t_max_z];

    let t_delta_x = if dir.x.abs() > 1.0e-15 {
        (s / dir.x).abs()
    } else {
        f64::INFINITY
    };
    let t_delta_y = if dir.y.abs() > 1.0e-15 {
        (s / dir.y).abs()
    } else {
        f64::INFINITY
    };
    let t_delta_z = if dir.z.abs() > 1.0e-15 {
        (s / dir.z).abs()
    } else {
        f64::INFINITY
    };

    let t_delta = [t_delta_x, t_delta_y, t_delta_z];

    let mut t_current = t_near;
    let max_steps = config.grid_nx + config.grid_ny + config.grid_nz + 10;

    for _ in 0..max_steps {
        // Find next boundary crossing
        let t_next = t_max[0].min(t_max[1]).min(t_max[2]).min(t_far);
        let length = t_next - t_current;

        if length > 1.0e-12 {
            hits.push(VoxelHit {
                ix,
                iy,
                iz,
                length,
            });
        }

        if t_next >= t_far - 1.0e-12 {
            break;
        }

        t_current = t_next;

        // Step to next voxel
        if t_max[0] <= t_max[1] && t_max[0] <= t_max[2] {
            let new_ix = ix as isize + step_x;
            if new_ix < 0 || new_ix >= config.grid_nx as isize {
                break;
            }
            ix = new_ix as usize;
            t_max[0] += t_delta[0];
        } else if t_max[1] <= t_max[0] && t_max[1] <= t_max[2] {
            let new_iy = iy as isize + step_y;
            if new_iy < 0 || new_iy >= config.grid_ny as isize {
                break;
            }
            iy = new_iy as usize;
            t_max[1] += t_delta[1];
        } else {
            let new_iz = iz as isize + step_z;
            if new_iz < 0 || new_iz >= config.grid_nz as isize {
                break;
            }
            iz = new_iz as usize;
            t_max[2] += t_delta[2];
        }
    }

    hits
}

// ---------------------------------------------------------------------------
// Transmission tomography helpers
// ---------------------------------------------------------------------------

/// Expected muon count through an area `area_cm2` at zenith angle `theta_rad`
/// over `time_min` minutes.
pub fn expected_muon_count(area_cm2: f64, theta_rad: f64, time_min: f64) -> f64 {
    muon_flux_at_angle(theta_rad) * area_cm2 * time_min
}

/// Opacity (transmission ratio) = N_observed / N_expected.
pub fn opacity(n_observed: f64, n_expected: f64) -> f64 {
    if n_expected <= 0.0 {
        return 0.0;
    }
    n_observed / n_expected
}

/// Density-length estimate (g/cm^2) from flux attenuation.
///
/// Uses an exponential attenuation model: T = exp(-rho_L / lambda_att)
/// where lambda_att is the attenuation length (~2500 g/cm^2 for typical muon energies).
pub fn density_length_from_opacity(transmission: f64, attenuation_length: f64) -> f64 {
    if transmission <= 0.0 || transmission >= 1.0 {
        return 0.0;
    }
    -attenuation_length * transmission.ln()
}

// ---------------------------------------------------------------------------
// Statistical analysis
// ---------------------------------------------------------------------------

/// SNR estimate: signal-to-noise ratio scales as sqrt(N).
///
/// Given a signal contrast `delta` (fractional opacity change) and total
/// counts `n`, SNR = delta * sqrt(n).
pub fn snr_from_counts(delta: f64, n: f64) -> f64 {
    if n <= 0.0 {
        return 0.0;
    }
    delta * n.sqrt()
}

/// Minimum detectable contrast for a given SNR threshold and count.
pub fn min_detectable_contrast(snr_threshold: f64, n: f64) -> f64 {
    if n <= 0.0 {
        return f64::INFINITY;
    }
    snr_threshold / n.sqrt()
}

/// Exposure time (minutes) required for a given SNR, contrast, and area.
///
/// SNR = delta * sqrt(flux * area * time)
/// => time = (SNR / delta)^2 / (flux * area)
pub fn required_exposure_time(
    snr_target: f64,
    delta: f64,
    area_cm2: f64,
    theta_rad: f64,
) -> f64 {
    if delta <= 0.0 || area_cm2 <= 0.0 {
        return f64::INFINITY;
    }
    let flux = muon_flux_at_angle(theta_rad);
    if flux <= 0.0 {
        return f64::INFINITY;
    }
    let ratio = snr_target / delta;
    ratio * ratio / (flux * area_cm2)
}

/// Detection probability for a simple threshold test (Gaussian approximation).
///
/// P_d = Phi(SNR - threshold) where Phi is the normal CDF.
pub fn detection_probability(snr: f64, threshold_sigma: f64) -> f64 {
    normal_cdf(snr - threshold_sigma)
}

/// False alarm probability: P_fa = 1 - Phi(threshold).
pub fn false_alarm_probability(threshold_sigma: f64) -> f64 {
    1.0 - normal_cdf(threshold_sigma)
}

/// Approximate normal CDF using the Abramowitz & Stegun error function approximation.
fn normal_cdf(x: f64) -> f64 {
    0.5 * (1.0 + erf_approx(x / std::f64::consts::SQRT_2))
}

/// Approximate error function (maximum error ~1.5e-7).
fn erf_approx(x: f64) -> f64 {
    let sign = if x >= 0.0 { 1.0 } else { -1.0 };
    let x = x.abs();
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let poly = t
        * (0.254829592
            + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    sign * (1.0 - poly * (-x * x).exp())
}

// ---------------------------------------------------------------------------
// Minimum detectable object size
// ---------------------------------------------------------------------------

/// Minimum detectable object linear size (cm) given exposure and detector geometry.
///
/// Rough estimate: size ~ voxel_size / sqrt(SNR), bounded by voxel_size.
pub fn min_detectable_size_cm(voxel_size_cm: f64, snr: f64) -> f64 {
    if snr <= 1.0 {
        return f64::INFINITY;
    }
    voxel_size_cm * (1.0 + 1.0 / snr.sqrt())
}

// ---------------------------------------------------------------------------
// ROC curve
// ---------------------------------------------------------------------------

/// Compute an ROC curve: (P_fa, P_d) for a range of thresholds.
///
/// Returns a vector of (false_alarm_rate, detection_probability) pairs.
pub fn roc_curve(snr: f64, n_points: usize) -> Vec<(f64, f64)> {
    let mut points = Vec::with_capacity(n_points);
    for i in 0..n_points {
        // threshold from -2 to 8 sigma
        let threshold = -2.0 + 10.0 * (i as f64) / (n_points as f64 - 1.0);
        let p_fa = false_alarm_probability(threshold);
        let p_d = detection_probability(snr, threshold);
        points.push((p_fa, p_d));
    }
    points
}

// ---------------------------------------------------------------------------
// Main reconstructor
// ---------------------------------------------------------------------------

/// Muon tomography reconstructor with POCA and MLEM algorithms.
pub struct MuonTomographyReconstructor {
    pub config: DetectorConfig,
    /// Accumulated scattering angle squared per voxel (for POCA).
    scatter_sum: Vec<f64>,
    /// Count of POCA hits per voxel.
    scatter_count: Vec<u32>,
    /// Momentum-weighted scattering sum.
    scatter_weighted_sum: Vec<f64>,
    /// Sum of weights (1/p^2 or 1).
    weight_sum: Vec<f64>,
    /// Transmission counts: muons observed per voxel path.
    transmission_count: Vec<u32>,
    /// Scattering density image (lambda, rad^2/m).
    pub lambda_image: Vec<f64>,
    /// Total tracks processed.
    pub total_tracks: usize,
}

impl MuonTomographyReconstructor {
    /// Create a new reconstructor for the given detector configuration.
    pub fn new(config: DetectorConfig) -> Self {
        let n = config.num_voxels();
        Self {
            config,
            scatter_sum: vec![0.0; n],
            scatter_count: vec![0; n],
            scatter_weighted_sum: vec![0.0; n],
            weight_sum: vec![0.0; n],
            transmission_count: vec![0; n],
            lambda_image: vec![0.0; n],
            total_tracks: 0,
        }
    }

    /// Reset all accumulated data.
    pub fn reset(&mut self) {
        let n = self.config.num_voxels();
        self.scatter_sum = vec![0.0; n];
        self.scatter_count = vec![0; n];
        self.scatter_weighted_sum = vec![0.0; n];
        self.weight_sum = vec![0.0; n];
        self.transmission_count = vec![0; n];
        self.lambda_image = vec![0.0; n];
        self.total_tracks = 0;
    }

    /// Process a single muon track using POCA reconstruction.
    ///
    /// Computes the POCA point, assigns it to a voxel, and accumulates
    /// the scattering angle weighted by momentum.
    pub fn process_track_poca(&mut self, track: &MuonTrack) -> Option<PocaResult> {
        let result = poca(
            &track.entry_point,
            &track.entry_direction,
            &track.exit_point,
            &track.exit_direction,
        );

        if let Some((ix, iy, iz)) = self.config.point_to_voxel(&result.poca_point) {
            let idx = self.config.voxel_index(ix, iy, iz);
            let theta_sq = track.scattering_angle_rad * track.scattering_angle_rad;

            self.scatter_sum[idx] += theta_sq;
            self.scatter_count[idx] += 1;

            // Weight by 1/p^2 if momentum is known (higher momentum muons
            // give less scattering, so weight them more).
            let w = if track.momentum_gev > 0.0 {
                track.momentum_gev * track.momentum_gev
            } else {
                1.0
            };
            self.scatter_weighted_sum[idx] += theta_sq * w;
            self.weight_sum[idx] += w;
        }

        // Count transmission for all traversed voxels
        let hits = trace_ray_through_grid(
            &self.config,
            &track.entry_point,
            &track.entry_direction,
        );
        for hit in &hits {
            let idx = self.config.voxel_index(hit.ix, hit.iy, hit.iz);
            self.transmission_count[idx] += 1;
        }

        self.total_tracks += 1;
        Some(result)
    }

    /// Process a batch of tracks.
    pub fn process_tracks_poca(&mut self, tracks: &[MuonTrack]) {
        for track in tracks {
            self.process_track_poca(track);
        }
    }

    /// Reconstruct the scattering density image using POCA (mean theta^2 per voxel).
    ///
    /// Lambda ~ <theta^2> / path_length, normalised by voxel size.
    pub fn reconstruct_poca(&mut self) {
        let s = self.config.voxel_size_m;
        let n = self.config.num_voxels();
        for i in 0..n {
            if self.scatter_count[i] > 0 {
                let mean_theta_sq = self.scatter_sum[i] / self.scatter_count[i] as f64;
                self.lambda_image[i] = mean_theta_sq / s;
            } else {
                self.lambda_image[i] = 0.0;
            }
        }
    }

    /// Reconstruct using momentum-weighted POCA.
    pub fn reconstruct_poca_weighted(&mut self) {
        let s = self.config.voxel_size_m;
        let n = self.config.num_voxels();
        for i in 0..n {
            if self.weight_sum[i] > 0.0 {
                let weighted_mean = self.scatter_weighted_sum[i] / self.weight_sum[i];
                self.lambda_image[i] = weighted_mean / s;
            } else {
                self.lambda_image[i] = 0.0;
            }
        }
    }

    /// Run MLEM (Maximum Likelihood Expectation Maximization) iterative
    /// reconstruction using line-of-response voxel traversal.
    ///
    /// * `tracks` - The muon tracks to reconstruct from.
    /// * `n_iterations` - Number of MLEM iterations.
    ///
    /// Updates `self.lambda_image` in place.
    pub fn reconstruct_mlem(&mut self, tracks: &[MuonTrack], n_iterations: usize) {
        let n = self.config.num_voxels();
        let s = self.config.voxel_size_m;

        // Initialise image to uniform non-zero value
        for v in self.lambda_image.iter_mut() {
            *v = 1.0e-4;
        }

        // Precompute system matrix: for each track, its voxel path lengths
        let mut system_matrix: Vec<Vec<(usize, f64)>> = Vec::with_capacity(tracks.len());
        let mut measured: Vec<f64> = Vec::with_capacity(tracks.len());

        for track in tracks {
            let midpoint = Vec3::new(
                0.5 * (track.entry_point.x + track.exit_point.x),
                0.5 * (track.entry_point.y + track.exit_point.y),
                0.5 * (track.entry_point.z + track.exit_point.z),
            );
            let direction = track.exit_point.sub(&track.entry_point).normalized();

            let hits = trace_ray_through_grid(&self.config, &track.entry_point, &direction);
            let row: Vec<(usize, f64)> = hits
                .iter()
                .map(|h| {
                    let idx = self.config.voxel_index(h.ix, h.iy, h.iz);
                    (idx, h.length / s)
                })
                .collect();

            measured.push(track.scattering_angle_rad * track.scattering_angle_rad);
            system_matrix.push(row);
        }

        // Sensitivity image: sum of path lengths per voxel
        let mut sensitivity = vec![0.0f64; n];
        for row in &system_matrix {
            for &(idx, len) in row {
                sensitivity[idx] += len;
            }
        }

        // MLEM iterations
        for _ in 0..n_iterations {
            // E-step: compute forward projection (expected measurement)
            let mut ratio = vec![0.0f64; tracks.len()];
            for (j, row) in system_matrix.iter().enumerate() {
                let mut forward = 0.0f64;
                for &(idx, len) in row {
                    forward += self.lambda_image[idx] * len;
                }
                if forward > 1.0e-20 {
                    ratio[j] = measured[j] / forward;
                }
            }

            // M-step: back-project ratio and update
            let mut correction = vec![0.0f64; n];
            for (j, row) in system_matrix.iter().enumerate() {
                for &(idx, len) in row {
                    correction[idx] += len * ratio[j];
                }
            }

            for i in 0..n {
                if sensitivity[i] > 1.0e-20 {
                    self.lambda_image[i] *= correction[i] / sensitivity[i];
                }
            }
        }
    }

    /// Get the scattering density at a voxel.
    pub fn get_lambda(&self, ix: usize, iy: usize, iz: usize) -> f64 {
        let idx = self.config.voxel_index(ix, iy, iz);
        self.lambda_image[idx]
    }

    /// Get the hit count at a voxel from POCA.
    pub fn get_hit_count(&self, ix: usize, iy: usize, iz: usize) -> u32 {
        let idx = self.config.voxel_index(ix, iy, iz);
        self.scatter_count[idx]
    }

    /// Get the transmission count through a voxel.
    pub fn get_transmission_count(&self, ix: usize, iy: usize, iz: usize) -> u32 {
        let idx = self.config.voxel_index(ix, iy, iz);
        self.transmission_count[idx]
    }

    /// Classify the material at each voxel based on reconstructed lambda.
    pub fn classify_all_voxels(&self) -> Vec<MaterialClass> {
        self.lambda_image.iter().map(|&l| classify_material(l)).collect()
    }

    /// Find voxels exceeding a scattering density threshold (potential threats).
    pub fn find_high_z_voxels(&self, threshold: f64) -> Vec<(usize, usize, usize, f64)> {
        let mut results = Vec::new();
        let n = self.config.num_voxels();
        for i in 0..n {
            if self.lambda_image[i] > threshold {
                let (ix, iy, iz) = self.config.voxel_coords(i);
                results.push((ix, iy, iz, self.lambda_image[i]));
            }
        }
        results
    }

    /// Compute the maximum scattering density in the image.
    pub fn max_lambda(&self) -> f64 {
        self.lambda_image
            .iter()
            .cloned()
            .fold(0.0f64, f64::max)
    }

    /// Generate a simple 2D projection (max intensity) along the z axis.
    pub fn project_z_max(&self) -> Vec<Vec<f64>> {
        let nx = self.config.grid_nx;
        let ny = self.config.grid_ny;
        let nz = self.config.grid_nz;
        let mut projection = vec![vec![0.0f64; nx]; ny];
        for iy in 0..ny {
            for ix in 0..nx {
                let mut max_val = 0.0f64;
                for iz in 0..nz {
                    let idx = self.config.voxel_index(ix, iy, iz);
                    max_val = max_val.max(self.lambda_image[idx]);
                }
                projection[iy][ix] = max_val;
            }
        }
        projection
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1.0e-9;

    // --- Vec3 tests ---

    #[test]
    fn test_vec3_basic_ops() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, 5.0, 6.0);
        let sum = a.add(&b);
        assert!((sum.x - 5.0).abs() < EPSILON);
        assert!((sum.y - 7.0).abs() < EPSILON);
        assert!((sum.z - 9.0).abs() < EPSILON);

        let diff = b.sub(&a);
        assert!((diff.x - 3.0).abs() < EPSILON);
        assert!((diff.y - 3.0).abs() < EPSILON);
        assert!((diff.z - 3.0).abs() < EPSILON);
    }

    #[test]
    fn test_vec3_dot_cross() {
        let a = Vec3::new(1.0, 0.0, 0.0);
        let b = Vec3::new(0.0, 1.0, 0.0);
        assert!((a.dot(&b)).abs() < EPSILON);
        let c = a.cross(&b);
        assert!((c.x).abs() < EPSILON);
        assert!((c.y).abs() < EPSILON);
        assert!((c.z - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_vec3_norm_and_normalize() {
        let v = Vec3::new(3.0, 4.0, 0.0);
        assert!((v.norm() - 5.0).abs() < EPSILON);
        let n = v.normalized();
        assert!((n.norm() - 1.0).abs() < EPSILON);
        assert!((n.x - 0.6).abs() < EPSILON);
        assert!((n.y - 0.8).abs() < EPSILON);
    }

    #[test]
    fn test_vec3_from_angles() {
        // Straight down: theta=0 => (0,0,1)
        let v = Vec3::from_angles(0.0, 0.0);
        assert!((v.z - 1.0).abs() < EPSILON);
        assert!(v.x.abs() < EPSILON);

        // theta=pi/2, phi=0 => (1,0,0)
        let v2 = Vec3::from_angles(PI / 2.0, 0.0);
        assert!((v2.x - 1.0).abs() < EPSILON);
        assert!(v2.z.abs() < EPSILON);
    }

    #[test]
    fn test_vec3_theta_phi() {
        let v = Vec3::new(1.0, 0.0, 1.0);
        assert!((v.theta() - PI / 4.0).abs() < 1.0e-6);
        assert!(v.phi().abs() < 1.0e-6);
    }

    // --- Radiation length constants ---

    #[test]
    fn test_radiation_lengths_ordering() {
        // X0 should decrease with Z: air > water > concrete > iron > lead > uranium
        assert!(RadiationLength::AIR > RadiationLength::WATER);
        assert!(RadiationLength::WATER > RadiationLength::CONCRETE);
        assert!(RadiationLength::CONCRETE > RadiationLength::IRON);
        assert!(RadiationLength::IRON > RadiationLength::LEAD);
        assert!(RadiationLength::LEAD > RadiationLength::URANIUM);
    }

    // --- Highland formula ---

    #[test]
    fn test_highland_formula_basic() {
        // 3 GeV muon through 10 cm of iron (X0 = 1.76 cm)
        let theta = highland_theta_rms(3.0, RadiationLength::IRON, 0.10);
        // Should be a small angle (a few mrad)
        assert!(theta > 0.001);
        assert!(theta < 0.1);
    }

    #[test]
    fn test_highland_momentum_dependence() {
        // Higher momentum => less scattering
        let theta_low_p = highland_theta_rms(1.0, RadiationLength::IRON, 0.10);
        let theta_high_p = highland_theta_rms(10.0, RadiationLength::IRON, 0.10);
        assert!(theta_low_p > theta_high_p);
    }

    #[test]
    fn test_highland_material_dependence() {
        // Denser material (shorter X0) => more scattering
        let theta_iron = highland_theta_rms(3.0, RadiationLength::IRON, 0.10);
        let theta_lead = highland_theta_rms(3.0, RadiationLength::LEAD, 0.10);
        let theta_uranium = highland_theta_rms(3.0, RadiationLength::URANIUM, 0.10);
        assert!(theta_uranium > theta_lead);
        assert!(theta_lead > theta_iron);
    }

    #[test]
    fn test_highland_thickness_dependence() {
        // More material => more scattering
        let theta_thin = highland_theta_rms(3.0, RadiationLength::IRON, 0.01);
        let theta_thick = highland_theta_rms(3.0, RadiationLength::IRON, 0.10);
        assert!(theta_thick > theta_thin);
    }

    #[test]
    fn test_highland_zero_inputs() {
        assert_eq!(highland_theta_rms(0.0, RadiationLength::IRON, 0.10), 0.0);
        assert_eq!(highland_theta_rms(3.0, 0.0, 0.10), 0.0);
        assert_eq!(highland_theta_rms(3.0, RadiationLength::IRON, 0.0), 0.0);
    }

    // --- Momentum estimation ---

    #[test]
    fn test_momentum_estimation() {
        let p_true = 3.0;
        let theta = highland_theta_rms(p_true, RadiationLength::IRON, 0.10);
        let p_est = estimate_momentum_from_scattering(theta, RadiationLength::IRON, 0.10);
        // Crude estimate, should be in the right ballpark
        assert!((p_est - p_true).abs() / p_true < 0.3);
    }

    // --- Muon flux ---

    #[test]
    fn test_muon_flux_zenith() {
        let flux = muon_flux_at_zenith();
        assert!((flux - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_muon_flux_angle_dependence() {
        let flux_0 = muon_flux_at_angle(0.0);
        let flux_45 = muon_flux_at_angle(PI / 4.0);
        let flux_60 = muon_flux_at_angle(PI / 3.0);
        // cos^2(0)=1, cos^2(45)=0.5, cos^2(60)=0.25
        assert!((flux_0 - 1.0).abs() < 0.01);
        assert!((flux_45 - 0.5).abs() < 0.01);
        assert!((flux_60 - 0.25).abs() < 0.01);
    }

    #[test]
    fn test_muon_energy_spectrum() {
        // Spectrum should be decreasing
        let f1 = muon_energy_spectrum(1.0);
        let f10 = muon_energy_spectrum(10.0);
        assert!(f1 > f10);
        // At 1 GeV, E^{-2.7} = 1.0
        assert!((f1 - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_exposure_time() {
        // 100 cm^2 area, want 1000 muons at zenith
        let t = exposure_time_for_count(1000.0, 100.0);
        // 1000 / (1.0 * 100) = 10 minutes
        assert!((t - 10.0).abs() < 0.01);
    }

    // --- Detector config ---

    #[test]
    fn test_detector_config_default() {
        let cfg = DetectorConfig::default();
        assert_eq!(cfg.num_voxels(), 20 * 20 * 40);
        assert_eq!(cfg.voxel_index(0, 0, 0), 0);
        assert_eq!(cfg.voxel_index(19, 19, 39), 20 * 20 * 40 - 1);
    }

    #[test]
    fn test_voxel_coords_roundtrip() {
        let cfg = DetectorConfig::default();
        for ix in 0..cfg.grid_nx {
            for iy in 0..cfg.grid_ny {
                for iz in [0, 1, cfg.grid_nz - 1] {
                    let idx = cfg.voxel_index(ix, iy, iz);
                    let (rx, ry, rz) = cfg.voxel_coords(idx);
                    assert_eq!((ix, iy, iz), (rx, ry, rz));
                }
            }
        }
    }

    #[test]
    fn test_voxel_center() {
        let cfg = DetectorConfig::default();
        let c = cfg.voxel_center(0, 0, 0);
        // Origin is (-0.5, -0.5, -1.0), voxel size 0.05
        // Center of (0,0,0) = (-0.5+0.025, -0.5+0.025, -1.0+0.025)
        assert!((c.x - (-0.475)).abs() < EPSILON);
        assert!((c.y - (-0.475)).abs() < EPSILON);
        assert!((c.z - (-0.975)).abs() < EPSILON);
    }

    #[test]
    fn test_point_to_voxel() {
        let cfg = DetectorConfig::default();
        // Point at the centre of the grid
        let mid = Vec3::new(0.0, 0.0, 0.0);
        let vox = cfg.point_to_voxel(&mid);
        assert!(vox.is_some());
        let (ix, iy, iz) = vox.unwrap();
        assert_eq!(ix, 10); // (0.0 - (-0.5)) / 0.05 = 10
        assert_eq!(iy, 10);
        assert_eq!(iz, 20); // (0.0 - (-1.0)) / 0.05 = 20

        // Point outside
        let outside = Vec3::new(100.0, 0.0, 0.0);
        assert!(cfg.point_to_voxel(&outside).is_none());
    }

    // --- POCA ---

    #[test]
    fn test_poca_intersecting_lines() {
        // Two lines that cross at the origin
        let p1 = Vec3::new(-1.0, 0.0, 0.0);
        let d1 = Vec3::new(1.0, 0.0, 0.0);
        let p2 = Vec3::new(0.0, -1.0, 0.0);
        let d2 = Vec3::new(0.0, 1.0, 0.0);
        let result = poca(&p1, &d1, &p2, &d2);
        assert!(result.distance < 1.0e-10);
        assert!((result.poca_point.x).abs() < 1.0e-10);
        assert!((result.poca_point.y).abs() < 1.0e-10);
    }

    #[test]
    fn test_poca_skew_lines() {
        // Two skew lines separated in z
        let p1 = Vec3::new(0.0, 0.0, 1.0);
        let d1 = Vec3::new(1.0, 0.0, 0.0);
        let p2 = Vec3::new(0.0, 0.0, -1.0);
        let d2 = Vec3::new(0.0, 1.0, 0.0);
        let result = poca(&p1, &d1, &p2, &d2);
        // Closest approach distance should be 2.0
        assert!((result.distance - 2.0).abs() < 1.0e-10);
        // Midpoint z should be 0
        assert!((result.poca_point.z).abs() < 1.0e-10);
    }

    // --- Scattering angle ---

    #[test]
    fn test_muon_track_scattering_angle() {
        let entry_dir = Vec3::new(0.0, 0.0, -1.0);
        let exit_dir = Vec3::new(0.01, 0.0, -1.0); // small deflection
        let angle = MuonTrack::compute_scattering_angle(&entry_dir, &exit_dir);
        assert!(angle > 0.0);
        assert!(angle < 0.02); // roughly 0.01 rad
    }

    #[test]
    fn test_muon_track_no_scattering() {
        let dir = Vec3::new(0.0, 0.0, -1.0);
        let angle = MuonTrack::compute_scattering_angle(&dir, &dir);
        assert!(angle.abs() < 1.0e-10);
    }

    // --- Material classification ---

    #[test]
    fn test_classify_material() {
        assert_eq!(classify_material(0.0), MaterialClass::Air);
        assert_eq!(classify_material(1.0e-7), MaterialClass::Air);
        assert_eq!(classify_material(1.0e-4), MaterialClass::Light);
        assert_eq!(classify_material(1.0e-2), MaterialClass::Medium);
        assert_eq!(classify_material(0.1), MaterialClass::Dense);
        assert_eq!(classify_material(1.0), MaterialClass::HighZ);
        assert_eq!(classify_material(10.0), MaterialClass::SpecialNuclear);
    }

    // --- Ray tracing ---

    #[test]
    fn test_ray_trace_vertical() {
        let config = DetectorConfig {
            top_detector_z: 0.5,
            bottom_detector_z: -0.5,
            voxel_size_m: 0.1,
            grid_nx: 5,
            grid_ny: 5,
            grid_nz: 10,
            grid_origin: Vec3::new(-0.25, -0.25, -0.5),
        };

        // Vertical ray through centre
        let origin = Vec3::new(0.0, 0.0, 1.0);
        let direction = Vec3::new(0.0, 0.0, -1.0);
        let hits = trace_ray_through_grid(&config, &origin, &direction);

        // Should pass through nz voxels, each with length ~ voxel_size
        assert!(!hits.is_empty());
        let total_length: f64 = hits.iter().map(|h| h.length).sum();
        // Total path through 10 voxels of 0.1m each = 1.0m
        assert!((total_length - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_ray_trace_misses_grid() {
        let config = DetectorConfig::default();
        let origin = Vec3::new(100.0, 100.0, 10.0);
        let direction = Vec3::new(0.0, 0.0, -1.0);
        let hits = trace_ray_through_grid(&config, &origin, &direction);
        assert!(hits.is_empty());
    }

    // --- Transmission tomography ---

    #[test]
    fn test_expected_muon_count() {
        let count = expected_muon_count(100.0, 0.0, 10.0);
        // 1.0 * 100 * 10 = 1000
        assert!((count - 1000.0).abs() < 0.01);
    }

    #[test]
    fn test_opacity() {
        assert!((opacity(80.0, 100.0) - 0.8).abs() < EPSILON);
        assert!((opacity(0.0, 100.0)).abs() < EPSILON);
    }

    #[test]
    fn test_density_length() {
        // 50% transmission, attenuation length 2500 g/cm^2
        let rho_l = density_length_from_opacity(0.5, 2500.0);
        // -2500 * ln(0.5) = 2500 * 0.693 ~ 1733
        assert!((rho_l - 2500.0 * 2.0f64.ln()).abs() < 1.0);
    }

    // --- Statistical analysis ---

    #[test]
    fn test_snr_from_counts() {
        let snr = snr_from_counts(0.1, 10000.0);
        // 0.1 * sqrt(10000) = 0.1 * 100 = 10
        assert!((snr - 10.0).abs() < EPSILON);
    }

    #[test]
    fn test_min_detectable_contrast() {
        let delta = min_detectable_contrast(5.0, 10000.0);
        // 5 / sqrt(10000) = 5/100 = 0.05
        assert!((delta - 0.05).abs() < EPSILON);
    }

    #[test]
    fn test_required_exposure_time() {
        let t = required_exposure_time(5.0, 0.1, 100.0, 0.0);
        // (5/0.1)^2 / (1.0*100) = 2500/100 = 25 minutes
        assert!((t - 25.0).abs() < 0.1);
    }

    #[test]
    fn test_detection_probability_high_snr() {
        // Very high SNR should give P_d close to 1
        let pd = detection_probability(10.0, 3.0);
        assert!(pd > 0.99);
    }

    #[test]
    fn test_false_alarm_probability() {
        // At 3 sigma threshold, P_fa ~ 0.00135
        let pfa = false_alarm_probability(3.0);
        assert!((pfa - 0.00135).abs() < 0.001);
    }

    #[test]
    fn test_roc_curve() {
        let roc = roc_curve(5.0, 50);
        assert_eq!(roc.len(), 50);
        // First point (low threshold) should have high P_fa and P_d
        assert!(roc[0].0 > 0.9); // high P_fa
        assert!(roc[0].1 > 0.99); // high P_d
        // Last point (high threshold) should have low P_fa and lower P_d
        let last = roc.last().unwrap();
        assert!(last.0 < 0.01);
    }

    // --- Full reconstruction test ---

    #[test]
    fn test_poca_reconstruction_simple() {
        let config = DetectorConfig {
            top_detector_z: 0.5,
            bottom_detector_z: -0.5,
            voxel_size_m: 0.1,
            grid_nx: 10,
            grid_ny: 10,
            grid_nz: 10,
            grid_origin: Vec3::new(-0.5, -0.5, -0.5),
        };

        let mut recon = MuonTomographyReconstructor::new(config);

        // Simulate tracks that scatter near the centre of the volume.
        // Each muon enters from above heading slightly inward, scatters
        // near (0,0,0), then exits below heading slightly outward.
        for i in 0..100 {
            let x_offset = 0.002 * (i as f64 - 50.0) / 50.0;
            // Incoming: enters at (x_offset, 0, 0.5) aimed toward origin
            let entry_pt = Vec3::new(x_offset, 0.0, 0.5);
            let entry_dir = Vec3::new(-x_offset, 0.0, -0.5).normalized();
            // Outgoing: exits at (x_offset + 0.02, 0, -0.5) aimed away
            let exit_pt = Vec3::new(x_offset + 0.02, 0.0, -0.5);
            let exit_dir = Vec3::new(x_offset + 0.02, 0.0, -0.5).normalized();
            let track = MuonTrack::new(entry_pt, entry_dir, exit_pt, exit_dir, 3.0);
            recon.process_track_poca(&track);
        }

        recon.reconstruct_poca();
        assert_eq!(recon.total_tracks, 100);

        // The centre voxel should have some signal
        let max = recon.max_lambda();
        assert!(max > 0.0);
    }

    #[test]
    fn test_mlem_reconstruction() {
        let config = DetectorConfig {
            top_detector_z: 0.5,
            bottom_detector_z: -0.5,
            voxel_size_m: 0.25,
            grid_nx: 4,
            grid_ny: 4,
            grid_nz: 4,
            grid_origin: Vec3::new(-0.5, -0.5, -0.5),
        };

        let mut recon = MuonTomographyReconstructor::new(config);

        // Generate synthetic tracks with higher scattering in centre
        let mut tracks = Vec::new();
        for i in 0..50 {
            let x = -0.3 + 0.6 * (i as f64) / 49.0;
            let scatter = if x.abs() < 0.15 { 0.05 } else { 0.001 };
            let track = MuonTrack::new(
                Vec3::new(x, 0.0, 0.5),
                Vec3::new(0.0, 0.0, -1.0),
                Vec3::new(x + scatter, 0.0, -0.5),
                Vec3::new(scatter, 0.0, -1.0),
                3.0,
            );
            tracks.push(track);
        }

        recon.reconstruct_mlem(&tracks, 5);

        // Should produce a non-trivial image
        let max = recon.max_lambda();
        assert!(max > 0.0);
    }

    #[test]
    fn test_reconstructor_reset() {
        let config = DetectorConfig::default();
        let mut recon = MuonTomographyReconstructor::new(config);

        let track = MuonTrack::new(
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(0.01, 0.0, -1.0),
            Vec3::new(0.01, 0.0, -1.0),
            3.0,
        );
        recon.process_track_poca(&track);
        assert_eq!(recon.total_tracks, 1);

        recon.reset();
        assert_eq!(recon.total_tracks, 0);
        assert!((recon.max_lambda()).abs() < EPSILON);
    }

    #[test]
    fn test_z_projection() {
        let config = DetectorConfig {
            top_detector_z: 0.5,
            bottom_detector_z: -0.5,
            voxel_size_m: 0.25,
            grid_nx: 4,
            grid_ny: 4,
            grid_nz: 4,
            grid_origin: Vec3::new(-0.5, -0.5, -0.5),
        };

        let mut recon = MuonTomographyReconstructor::new(config);
        // Set a known value
        let idx = recon.config.voxel_index(2, 2, 2);
        recon.lambda_image[idx] = 1.0;

        let proj = recon.project_z_max();
        assert_eq!(proj.len(), 4); // ny
        assert_eq!(proj[0].len(), 4); // nx
        assert!((proj[2][2] - 1.0).abs() < EPSILON);
        assert!((proj[0][0]).abs() < EPSILON);
    }

    #[test]
    fn test_find_high_z_voxels() {
        let config = DetectorConfig {
            top_detector_z: 0.5,
            bottom_detector_z: -0.5,
            voxel_size_m: 0.5,
            grid_nx: 2,
            grid_ny: 2,
            grid_nz: 2,
            grid_origin: Vec3::new(-0.5, -0.5, -0.5),
        };

        let mut recon = MuonTomographyReconstructor::new(config);
        let idx = recon.config.voxel_index(1, 1, 1);
        recon.lambda_image[idx] = 10.0;

        let high_z = recon.find_high_z_voxels(5.0);
        assert_eq!(high_z.len(), 1);
        assert_eq!(high_z[0], (1, 1, 1, 10.0));
    }

    #[test]
    fn test_min_detectable_size() {
        let size = min_detectable_size_cm(5.0, 100.0);
        // 5.0 * (1 + 1/sqrt(100)) = 5.0 * 1.1 = 5.5
        assert!((size - 5.5).abs() < 0.01);
    }

    #[test]
    fn test_erf_approx_at_zero() {
        let val = erf_approx(0.0);
        assert!(val.abs() < 1.0e-6);
    }

    #[test]
    fn test_normal_cdf_symmetry() {
        let p_pos = normal_cdf(1.0);
        let p_neg = normal_cdf(-1.0);
        assert!((p_pos + p_neg - 1.0).abs() < 1.0e-5);
    }
}
