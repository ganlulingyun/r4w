//! # Cosmic Ray Muon Tracker
//!
//! Muon tomography signal processing for cargo scanning, volcano imaging, and
//! nuclear waste monitoring. Cosmic ray muons originate from high-energy cosmic
//! ray interactions in the upper atmosphere and pass through matter, where they
//! undergo Coulomb scattering proportional to the material's density and atomic
//! number (Z). By tracking muon trajectories above and below an object,
//! the internal density structure can be reconstructed.
//!
//! ## Physical Background
//!
//! At sea level, the muon flux is approximately 1 muon/cm^2/min with a mean
//! energy of ~3-4 GeV. The angular distribution follows a cos^2(theta) law for
//! the zenith angle. Multiple Coulomb scattering causes angular deflection
//! described by the Highland formula:
//!
//! ```text
//! theta_0 = (13.6 MeV / (beta*c*p)) * sqrt(x/X0) * [1 + 0.038 * ln(x/X0)]
//! ```
//!
//! where `p` is the muon momentum, `x` is the material thickness, and `X0` is
//! the radiation length.
//!
//! ## Reconstruction Techniques
//!
//! - **POCA (Point of Closest Approach)**: Assigns scattering to the midpoint
//!   between incoming and outgoing muon tracks. Simple but effective for
//!   high-contrast objects.
//!
//! - **Transmission Tomography**: Counts muon absorption/attenuation through
//!   different paths. Similar to X-ray CT but using cosmic rays.
//!
//! - **Angular Statistical Reconstruction (ASR)**: Iterative maximum-likelihood
//!   estimation that distributes scattering along the muon path through voxels.
//!
//! ## Applications
//!
//! - **Cargo scanning**: Detecting shielded nuclear materials in shipping containers
//! - **Volcano imaging**: Mapping internal density of volcanoes (muon radiography)
//! - **Nuclear waste**: Non-invasive characterization of sealed waste containers
//! - **Archaeology**: Imaging hidden chambers in pyramids and large structures
//! - **Civil engineering**: Inspecting tunnel linings and dam structures
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cosmic_ray_muon_tracker::*;
//!
//! let config = MuonConfig {
//!     detector_spacing_m: 2.0,
//!     angular_resolution_mrad: 1.0,
//!     position_resolution_mm: 0.5,
//!     energy_spectrum: EnergySpectrum::SeaLevel,
//!     voxel_size_m: 0.05,
//! };
//!
//! let tracker = MuonTracker::new(config);
//!
//! // Compute scattering angle between entry and exit directions
//! let entry_dir = [0.0, 0.0, -1.0]; // straight down
//! let exit_dir = [0.01, 0.005, -0.9999];
//! let angle = tracker.scattering_angle(entry_dir, exit_dir);
//! assert!(angle > 0.0);
//!
//! // Highland formula: expected RMS scattering for 10 cm of lead
//! let theta0 = MuonTracker::highland_formula(0.10, 0.0056, 3.0);
//! assert!(theta0 > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Radiation length constants (in g/cm^2) for common materials
// ---------------------------------------------------------------------------

/// Radiation length of air in g/cm^2.
pub const X0_AIR_G_CM2: f64 = 36.66;
/// Radiation length of water in g/cm^2.
pub const X0_WATER_G_CM2: f64 = 36.08;
/// Radiation length of concrete in g/cm^2.
pub const X0_CONCRETE_G_CM2: f64 = 26.57;
/// Radiation length of aluminum in g/cm^2.
pub const X0_ALUMINUM_G_CM2: f64 = 24.01;
/// Radiation length of iron in g/cm^2.
pub const X0_IRON_G_CM2: f64 = 13.84;
/// Radiation length of lead in g/cm^2.
pub const X0_LEAD_G_CM2: f64 = 6.37;
/// Radiation length of uranium in g/cm^2.
pub const X0_URANIUM_G_CM2: f64 = 6.00;

// Densities in g/cm^3 for converting g/cm^2 to cm or m.
/// Density of air in g/cm^3.
pub const DENSITY_AIR: f64 = 0.001225;
/// Density of water in g/cm^3.
pub const DENSITY_WATER: f64 = 1.0;
/// Density of concrete in g/cm^3.
pub const DENSITY_CONCRETE: f64 = 2.3;
/// Density of aluminum in g/cm^3.
pub const DENSITY_ALUMINUM: f64 = 2.7;
/// Density of iron in g/cm^3.
pub const DENSITY_IRON: f64 = 7.87;
/// Density of lead in g/cm^3.
pub const DENSITY_LEAD: f64 = 11.35;
/// Density of uranium in g/cm^3.
pub const DENSITY_URANIUM: f64 = 19.1;

// ---------------------------------------------------------------------------
// EnergySpectrum
// ---------------------------------------------------------------------------

/// Energy spectrum model for incident cosmic ray muons.
#[derive(Debug, Clone, PartialEq)]
pub enum EnergySpectrum {
    /// Standard sea-level spectrum with mean ~3 GeV.
    SeaLevel,
    /// Underground spectrum attenuated by overburden. `depth_mwe` is the
    /// depth in metres water equivalent.
    Underground {
        /// Depth in metres water equivalent (mwe).
        depth_mwe: f64,
    },
    /// Custom mono-energetic or user-defined mean energy.
    Custom {
        /// Mean muon energy in GeV.
        mean_gev: f64,
    },
}

// ---------------------------------------------------------------------------
// MuonConfig
// ---------------------------------------------------------------------------

/// Configuration for the muon tracking system.
#[derive(Debug, Clone)]
pub struct MuonConfig {
    /// Distance between upper and lower detector planes in metres.
    pub detector_spacing_m: f64,
    /// Angular resolution of the detector in milliradians.
    pub angular_resolution_mrad: f64,
    /// Position resolution of the detector in millimetres.
    pub position_resolution_mm: f64,
    /// Energy spectrum model.
    pub energy_spectrum: EnergySpectrum,
    /// Voxel (volumetric pixel) size for the reconstruction grid in metres.
    pub voxel_size_m: f64,
}

impl Default for MuonConfig {
    fn default() -> Self {
        Self {
            detector_spacing_m: 2.0,
            angular_resolution_mrad: 1.0,
            position_resolution_mm: 0.5,
            energy_spectrum: EnergySpectrum::SeaLevel,
            voxel_size_m: 0.05,
        }
    }
}

// ---------------------------------------------------------------------------
// MuonTrack
// ---------------------------------------------------------------------------

/// A single muon track measured at one detector plane.
///
/// A track is defined by a 3D position and a unit direction vector.
#[derive(Debug, Clone)]
pub struct MuonTrack {
    /// Entry position (x, y, z) in metres on the upper detector plane.
    pub entry_position: [f64; 3],
    /// Entry direction as a unit vector (dx, dy, dz).
    pub entry_direction: [f64; 3],
    /// Exit position (x, y, z) in metres on the lower detector plane.
    pub exit_position: [f64; 3],
    /// Exit direction as a unit vector (dx, dy, dz).
    pub exit_direction: [f64; 3],
    /// Estimated muon momentum in GeV/c.
    pub momentum_gev: f64,
}

// ---------------------------------------------------------------------------
// MuonTracker
// ---------------------------------------------------------------------------

/// Core muon tracking and scattering analysis engine.
///
/// Provides methods for computing scattering angles, the Highland formula
/// for multiple Coulomb scattering, radiation lengths, and geometric
/// operations on muon tracks.
pub struct MuonTracker {
    /// Configuration parameters.
    pub config: MuonConfig,
}

impl MuonTracker {
    /// Create a new `MuonTracker` with the given configuration.
    pub fn new(config: MuonConfig) -> Self {
        Self { config }
    }

    /// Compute the 3D scattering angle between two direction vectors in
    /// milliradians.
    ///
    /// The scattering angle is the angle between the entry and exit direction
    /// unit vectors: `theta = acos(d_in . d_out)`.
    ///
    /// Returns the angle in milliradians. If both directions are parallel, the
    /// result is 0.
    pub fn scattering_angle(&self, entry_dir: [f64; 3], exit_dir: [f64; 3]) -> f64 {
        let dot = entry_dir[0] * exit_dir[0]
            + entry_dir[1] * exit_dir[1]
            + entry_dir[2] * exit_dir[2];
        // Clamp for numerical safety
        let dot_clamped = dot.clamp(-1.0, 1.0);
        dot_clamped.acos() * 1000.0 // radians -> milliradians
    }

    /// Highland formula for RMS multiple Coulomb scattering angle.
    ///
    /// ```text
    /// theta_0 = (13.6 / p) * sqrt(x / X0) * [1 + 0.038 * ln(x / X0)]
    /// ```
    ///
    /// # Arguments
    /// * `thickness_m` - Material thickness in metres (converted internally to
    ///   the appropriate units by assuming density = 1 g/cm^3 for the ratio).
    ///   In practice `x / X0` is dimensionless when both are in the same areal
    ///   density units. Here we treat the inputs as already in consistent linear
    ///   length units (metres).
    /// * `radiation_length_m` - Radiation length in metres.
    /// * `momentum_gev` - Muon momentum in GeV/c.
    ///
    /// # Returns
    /// RMS scattering angle in milliradians.
    pub fn highland_formula(
        thickness_m: f64,
        radiation_length_m: f64,
        momentum_gev: f64,
    ) -> f64 {
        if radiation_length_m <= 0.0 || momentum_gev <= 0.0 || thickness_m <= 0.0 {
            return 0.0;
        }
        let x_over_x0 = thickness_m / radiation_length_m;
        if x_over_x0 <= 0.0 {
            return 0.0;
        }
        let theta_rad =
            (13.6e-3 / momentum_gev) * x_over_x0.sqrt() * (1.0 + 0.038 * x_over_x0.ln());
        // 13.6 MeV = 0.0136 GeV. Return in milliradians.
        theta_rad * 1000.0
    }

    /// Approximate radiation length using the Tsai formula.
    ///
    /// ```text
    /// X0 (g/cm^2) ≈ 716.4 * A / (Z * (Z + 1) * ln(287 / sqrt(Z)))
    /// ```
    ///
    /// # Arguments
    /// * `z` - Atomic number.
    /// * `a` - Atomic mass (g/mol).
    ///
    /// # Returns
    /// Radiation length in g/cm^2.
    pub fn radiation_length(z: f64, a: f64) -> f64 {
        if z <= 0.0 || a <= 0.0 {
            return 0.0;
        }
        let ln_term = (287.0 / z.sqrt()).ln();
        if ln_term <= 0.0 {
            return 0.0;
        }
        716.4 * a / (z * (z + 1.0) * ln_term)
    }

    /// Compute the Point of Closest Approach (POCA) of two 3D lines.
    ///
    /// Given two lines defined by (position, direction), find the 3D point
    /// that is closest to both lines (the midpoint of the shortest segment
    /// connecting the two lines).
    ///
    /// # Arguments
    /// * `track1_pos` - Position on line 1.
    /// * `track1_dir` - Direction of line 1 (need not be unit length).
    /// * `track2_pos` - Position on line 2.
    /// * `track2_dir` - Direction of line 2 (need not be unit length).
    ///
    /// # Returns
    /// The midpoint of the closest approach segment. If the lines are
    /// (near-)parallel, returns the midpoint of the two positions.
    pub fn point_of_closest_approach(
        track1_pos: [f64; 3],
        track1_dir: [f64; 3],
        track2_pos: [f64; 3],
        track2_dir: [f64; 3],
    ) -> [f64; 3] {
        // w = p1 - p2
        let w = [
            track1_pos[0] - track2_pos[0],
            track1_pos[1] - track2_pos[1],
            track1_pos[2] - track2_pos[2],
        ];
        let a = dot3(track1_dir, track1_dir);
        let b = dot3(track1_dir, track2_dir);
        let c = dot3(track2_dir, track2_dir);
        let d = dot3(track1_dir, w);
        let e = dot3(track2_dir, w);

        let denom = a * c - b * b;

        let (s, t) = if denom.abs() < 1e-12 {
            // Near-parallel lines; pick s=0
            (0.0, e / c.max(1e-30))
        } else {
            let s_val = (b * e - c * d) / denom;
            let t_val = (a * e - b * d) / denom;
            (s_val, t_val)
        };

        // Closest point on line 1
        let p1 = [
            track1_pos[0] + s * track1_dir[0],
            track1_pos[1] + s * track1_dir[1],
            track1_pos[2] + s * track1_dir[2],
        ];
        // Closest point on line 2
        let p2 = [
            track2_pos[0] + t * track2_dir[0],
            track2_pos[1] + t * track2_dir[1],
            track2_pos[2] + t * track2_dir[2],
        ];

        // Midpoint
        [
            (p1[0] + p2[0]) * 0.5,
            (p1[1] + p2[1]) * 0.5,
            (p1[2] + p2[2]) * 0.5,
        ]
    }

    /// Compute projected scattering angles in the x-z and y-z planes.
    ///
    /// Decomposes the 3D scattering into two independent 2D projections,
    /// which is useful for reconstruction algorithms that work in projected
    /// views.
    ///
    /// # Returns
    /// `(delta_theta_x, delta_theta_y)` in milliradians.
    pub fn projected_scattering(entry: &MuonTrack, exit: &MuonTrack) -> (f64, f64) {
        // Projected angle in x-z plane: atan2(dx, dz) difference
        let theta_x_in = entry.entry_direction[0].atan2(entry.entry_direction[2]);
        let theta_x_out = exit.exit_direction[0].atan2(exit.exit_direction[2]);
        let delta_x = angle_diff(theta_x_out, theta_x_in) * 1000.0;

        // Projected angle in y-z plane: atan2(dy, dz) difference
        let theta_y_in = entry.entry_direction[1].atan2(entry.entry_direction[2]);
        let theta_y_out = exit.exit_direction[1].atan2(exit.exit_direction[2]);
        let delta_y = angle_diff(theta_y_out, theta_y_in) * 1000.0;

        (delta_x, delta_y)
    }
}

// ---------------------------------------------------------------------------
// DensityReconstructor
// ---------------------------------------------------------------------------

/// 3D voxelized density reconstruction from muon track data.
///
/// Provides multiple reconstruction algorithms: POCA, transmission
/// tomography, and iterative angular statistical reconstruction.
pub struct DensityReconstructor {
    /// Lower corner of the grid bounding box.
    pub grid_min: [f64; 3],
    /// Upper corner of the grid bounding box.
    pub grid_max: [f64; 3],
    /// Voxel edge length in metres.
    pub voxel_size: f64,
    /// Number of voxels along each axis.
    pub grid_dims: [usize; 3],
}

impl DensityReconstructor {
    /// Create a new reconstructor with the specified bounding box and voxel
    /// size.
    ///
    /// # Arguments
    /// * `grid_bounds` - (min_corner, max_corner) in metres.
    /// * `voxel_size` - Voxel edge length in metres.
    pub fn new(grid_bounds: ([f64; 3], [f64; 3]), voxel_size: f64) -> Self {
        let (min, max) = grid_bounds;
        let nx = ((max[0] - min[0]) / voxel_size).ceil() as usize;
        let ny = ((max[1] - min[1]) / voxel_size).ceil() as usize;
        let nz = ((max[2] - min[2]) / voxel_size).ceil() as usize;
        Self {
            grid_min: min,
            grid_max: max,
            voxel_size,
            grid_dims: [nx.max(1), ny.max(1), nz.max(1)],
        }
    }

    /// Convert a 3D world position to a voxel grid index.
    ///
    /// Returns `None` if the position is outside the grid bounds.
    pub fn voxel_index(&self, position: [f64; 3]) -> Option<[usize; 3]> {
        let ix = ((position[0] - self.grid_min[0]) / self.voxel_size).floor() as isize;
        let iy = ((position[1] - self.grid_min[1]) / self.voxel_size).floor() as isize;
        let iz = ((position[2] - self.grid_min[2]) / self.voxel_size).floor() as isize;
        if ix < 0
            || iy < 0
            || iz < 0
            || (ix as usize) >= self.grid_dims[0]
            || (iy as usize) >= self.grid_dims[1]
            || (iz as usize) >= self.grid_dims[2]
        {
            None
        } else {
            Some([ix as usize, iy as usize, iz as usize])
        }
    }

    /// POCA-based 3D density reconstruction.
    ///
    /// For each track pair (entry track, exit track), the Point of Closest
    /// Approach is computed and the scattering angle is accumulated into the
    /// corresponding voxel. The result is a 3D array indexed [x][y][z]
    /// containing the mean squared scattering angle per voxel (proportional
    /// to scattering density).
    ///
    /// # Arguments
    /// * `tracks` - Slice of (entry_track, exit_track) pairs.
    ///
    /// # Returns
    /// 3D voxel grid of scattering density values.
    pub fn poca_reconstruct(
        &self,
        tracks: &[(MuonTrack, MuonTrack)],
    ) -> Vec<Vec<Vec<f64>>> {
        let [nx, ny, nz] = self.grid_dims;
        let mut sum = vec![vec![vec![0.0_f64; nz]; ny]; nx];
        let mut count = vec![vec![vec![0_u32; nz]; ny]; nx];

        for (entry, exit) in tracks {
            let poca = MuonTracker::point_of_closest_approach(
                entry.entry_position,
                entry.entry_direction,
                exit.exit_position,
                exit.exit_direction,
            );
            if let Some([ix, iy, iz]) = self.voxel_index(poca) {
                let dot = entry.entry_direction[0] * exit.exit_direction[0]
                    + entry.entry_direction[1] * exit.exit_direction[1]
                    + entry.entry_direction[2] * exit.exit_direction[2];
                let angle = dot.clamp(-1.0, 1.0).acos(); // radians
                sum[ix][iy][iz] += angle * angle;
                count[ix][iy][iz] += 1;
            }
        }

        // Normalize: mean squared scattering angle per voxel
        for ix in 0..nx {
            for iy in 0..ny {
                for iz in 0..nz {
                    if count[ix][iy][iz] > 0 {
                        sum[ix][iy][iz] /= count[ix][iy][iz] as f64;
                    }
                }
            }
        }
        sum
    }

    /// Estimate scattering density from a set of scattering angles.
    ///
    /// Uses the median of the absolute angles, which is more robust to
    /// outliers than the mean. The median scattering angle is proportional
    /// to `sqrt(Z(Z+1))` for a given thickness and momentum, making it
    /// a proxy for material density/Z.
    ///
    /// # Arguments
    /// * `angles` - Scattering angles in milliradians.
    ///
    /// # Returns
    /// Median absolute scattering angle (mrad).
    pub fn scattering_density(angles: &[f64]) -> f64 {
        if angles.is_empty() {
            return 0.0;
        }
        let mut sorted: Vec<f64> = angles.iter().map(|a| a.abs()).collect();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let n = sorted.len();
        if n % 2 == 0 {
            (sorted[n / 2 - 1] + sorted[n / 2]) * 0.5
        } else {
            sorted[n / 2]
        }
    }

    /// Transmission (absorption) tomography.
    ///
    /// Counts the number of muons passing through each 2D pixel of a
    /// projection grid. Low counts indicate high absorption (dense material).
    ///
    /// The `absorption_map` is a pre-allocated 2D grid (rows x cols) that
    /// will be incremented for each muon whose exit position falls within the
    /// pixel. The grid covers the x-y extent of `grid_min..grid_max`.
    ///
    /// # Arguments
    /// * `tracks` - Slice of exit-side muon tracks.
    /// * `absorption_map` - Mutable reference to a 2D grid to accumulate counts.
    pub fn transmission_tomography(
        &self,
        tracks: &[MuonTrack],
        absorption_map: &mut Vec<Vec<f64>>,
    ) {
        let nx = absorption_map.len();
        if nx == 0 {
            return;
        }
        let ny = absorption_map[0].len();
        if ny == 0 {
            return;
        }

        let dx = (self.grid_max[0] - self.grid_min[0]) / nx as f64;
        let dy = (self.grid_max[1] - self.grid_min[1]) / ny as f64;

        for track in tracks {
            let ix = ((track.exit_position[0] - self.grid_min[0]) / dx).floor() as isize;
            let iy = ((track.exit_position[1] - self.grid_min[1]) / dy).floor() as isize;
            if ix >= 0 && iy >= 0 && (ix as usize) < nx && (iy as usize) < ny {
                absorption_map[ix as usize][iy as usize] += 1.0;
            }
        }
    }

    /// Iterative Angular Statistical Reconstruction (ASR).
    ///
    /// An expectation-maximization style iterative algorithm that distributes
    /// scattering contributions along the muon path through the voxel grid.
    ///
    /// The algorithm:
    /// 1. Initialize all voxels to uniform scattering density.
    /// 2. For each muon: trace the path, compute the expected total scattering
    ///    from current voxel values, and compute a correction ratio from the
    ///    measured vs. expected scattering.
    /// 3. Update voxel values proportionally.
    /// 4. Repeat for `iterations`.
    ///
    /// # Arguments
    /// * `tracks` - Slice of (entry_track, exit_track) pairs.
    /// * `iterations` - Number of EM iterations.
    ///
    /// # Returns
    /// 3D voxel grid of estimated scattering density.
    pub fn angular_statistical_reconstruction(
        &self,
        tracks: &[(MuonTrack, MuonTrack)],
        iterations: usize,
    ) -> Vec<Vec<Vec<f64>>> {
        let [nx, ny, nz] = self.grid_dims;

        // Initialize uniform
        let init_val = 1.0;
        let mut lambda = vec![vec![vec![init_val; nz]; ny]; nx];

        for _iter in 0..iterations {
            let mut numerator = vec![vec![vec![0.0_f64; nz]; ny]; nx];
            let mut denominator = vec![vec![vec![0.0_f64; nz]; ny]; nx];

            for (entry, exit) in tracks {
                // Measured scattering angle squared (radians^2)
                let dot = entry.entry_direction[0] * exit.exit_direction[0]
                    + entry.entry_direction[1] * exit.exit_direction[1]
                    + entry.entry_direction[2] * exit.exit_direction[2];
                let measured_sq = dot.clamp(-1.0, 1.0).acos().powi(2);

                // Ray-trace through voxels: find all voxels the path crosses
                let voxels = self.trace_path(
                    entry.entry_position,
                    entry.entry_direction,
                    exit.exit_position,
                );

                if voxels.is_empty() {
                    continue;
                }

                // Expected scattering from current lambda values
                let expected_sq: f64 = voxels
                    .iter()
                    .map(|&(ix, iy, iz, len)| lambda[ix][iy][iz] * len)
                    .sum();

                if expected_sq < 1e-30 {
                    continue;
                }

                let ratio = measured_sq / expected_sq;

                for &(ix, iy, iz, len) in &voxels {
                    numerator[ix][iy][iz] += ratio * len;
                    denominator[ix][iy][iz] += len;
                }
            }

            // Update step
            for ix in 0..nx {
                for iy in 0..ny {
                    for iz in 0..nz {
                        if denominator[ix][iy][iz] > 0.0 {
                            lambda[ix][iy][iz] *= numerator[ix][iy][iz]
                                / denominator[ix][iy][iz];
                        }
                    }
                }
            }
        }

        lambda
    }

    /// Trace a straight-line path through the voxel grid using a simplified
    /// DDA (Digital Differential Analyzer) algorithm.
    ///
    /// Returns a list of (ix, iy, iz, path_length_in_voxel) tuples.
    fn trace_path(
        &self,
        start: [f64; 3],
        direction: [f64; 3],
        end: [f64; 3],
    ) -> Vec<(usize, usize, usize, f64)> {
        let mut result = Vec::new();

        // Total path length
        let dx = end[0] - start[0];
        let dy = end[1] - start[1];
        let dz = end[2] - start[2];
        let total_len = (dx * dx + dy * dy + dz * dz).sqrt();
        if total_len < 1e-12 {
            return result;
        }

        // Normalise direction along start->end
        let dir = [dx / total_len, dy / total_len, dz / total_len];

        // Step along the path in small increments
        let step = self.voxel_size * 0.5;
        let n_steps = (total_len / step).ceil() as usize;
        let actual_step = total_len / n_steps as f64;

        let mut prev_voxel: Option<[usize; 3]> = None;
        let mut prev_len = 0.0_f64;

        for i in 0..=n_steps {
            let t = i as f64 * actual_step;
            let pos = [
                start[0] + dir[0] * t,
                start[1] + dir[1] * t,
                start[2] + dir[2] * t,
            ];
            if let Some(idx) = self.voxel_index(pos) {
                match prev_voxel {
                    Some(pv) if pv == idx => {
                        prev_len += actual_step;
                    }
                    Some(pv) => {
                        if prev_len > 0.0 {
                            result.push((pv[0], pv[1], pv[2], prev_len));
                        }
                        prev_voxel = Some(idx);
                        prev_len = actual_step;
                    }
                    None => {
                        prev_voxel = Some(idx);
                        prev_len = actual_step;
                    }
                }
            } else {
                // Outside grid - flush current voxel
                if let Some(pv) = prev_voxel.take() {
                    if prev_len > 0.0 {
                        result.push((pv[0], pv[1], pv[2], prev_len));
                    }
                    prev_len = 0.0;
                }
            }
        }
        // Flush last voxel
        if let Some(pv) = prev_voxel {
            if prev_len > 0.0 {
                result.push((pv[0], pv[1], pv[2], prev_len));
            }
        }

        // Normalise path lengths so they sum to total_len (correct rounding)
        let sum: f64 = result.iter().map(|r| r.3).sum();
        if sum > 0.0 && (sum - total_len).abs() > 1e-9 {
            let scale = total_len / sum;
            for r in &mut result {
                r.3 *= scale;
            }
        }

        result
    }
}

// ---------------------------------------------------------------------------
// MuonFluxModel
// ---------------------------------------------------------------------------

/// Models for cosmic ray muon flux at various depths and angles.
pub struct MuonFluxModel;

impl MuonFluxModel {
    /// Sea-level vertical muon flux in muons/cm^2/s/sr.
    ///
    /// The standard reference value is approximately 70 muons/m^2/s/sr
    /// = 0.007 muons/cm^2/s/sr at the vertical.
    const VERTICAL_FLUX: f64 = 0.007;

    /// Sea-level muon flux as a function of zenith angle.
    ///
    /// ```text
    /// I(theta) = I(0) * cos^2(theta)
    /// ```
    ///
    /// # Arguments
    /// * `zenith_deg` - Zenith angle in degrees (0 = straight down from sky).
    ///
    /// # Returns
    /// Flux in muons/cm^2/s/sr.
    pub fn sea_level_flux(zenith_deg: f64) -> f64 {
        let theta = zenith_deg.to_radians();
        Self::VERTICAL_FLUX * theta.cos().powi(2)
    }

    /// Underground (depth-attenuated) muon flux.
    ///
    /// An empirical approximation for the vertical muon intensity at depth:
    ///
    /// ```text
    /// I(d) = I(0) * exp(-d / L)
    /// ```
    ///
    /// where L ~ 500 mwe is the characteristic attenuation length for GeV muons.
    ///
    /// # Arguments
    /// * `depth_mwe` - Depth in metres water equivalent.
    ///
    /// # Returns
    /// Flux in muons/cm^2/s/sr.
    pub fn underground_flux(depth_mwe: f64) -> f64 {
        let attenuation_length = 500.0; // mwe
        Self::VERTICAL_FLUX * (-depth_mwe / attenuation_length).exp()
    }

    /// Expected number of muon counts for a given flux, area, solid angle,
    /// and observation time.
    ///
    /// ```text
    /// N = flux * area * solid_angle * time
    /// ```
    ///
    /// # Arguments
    /// * `flux` - Flux in muons/cm^2/s/sr.
    /// * `area_m2` - Detector area in m^2 (converted internally to cm^2).
    /// * `solid_angle_sr` - Accepted solid angle in steradians.
    /// * `time_s` - Observation time in seconds.
    ///
    /// # Returns
    /// Expected number of muon counts.
    pub fn expected_counts(flux: f64, area_m2: f64, solid_angle_sr: f64, time_s: f64) -> f64 {
        let area_cm2 = area_m2 * 1e4; // m^2 -> cm^2
        flux * area_cm2 * solid_angle_sr * time_s
    }

    /// Muon energy loss rate (stopping power) in GeV/(g/cm^2).
    ///
    /// For minimum-ionizing muons, `dE/dx ~ 2 MeV/(g/cm^2)` is approximately
    /// constant. At higher energies, radiative losses (bremsstrahlung, pair
    /// production) dominate. A simplified parameterisation:
    ///
    /// ```text
    /// dE/dx = a + b * E
    /// ```
    ///
    /// where a ~ 2 MeV/(g/cm^2) = 0.002 GeV/(g/cm^2) and
    /// b ~ 4.0e-6 /(g/cm^2) for standard rock.
    ///
    /// # Arguments
    /// * `energy_gev` - Muon energy in GeV.
    ///
    /// # Returns
    /// Energy loss rate in GeV/(g/cm^2).
    pub fn muon_energy_loss_rate(energy_gev: f64) -> f64 {
        let a = 0.002; // GeV / (g/cm^2) — ionisation
        let b = 4.0e-6; // 1/(g/cm^2) — radiative
        a + b * energy_gev
    }

    /// Approximate penetration depth (range) in metres water equivalent.
    ///
    /// Integrates the energy loss rate from the given energy down to the
    /// muon rest mass. For the linear loss model `dE/dx = a + bE`,
    /// the analytic range is:
    ///
    /// ```text
    /// R = (1/b) * ln((a + b*E) / (a + b*E_min))
    /// ```
    ///
    /// where E_min ~ 0.106 GeV (muon mass).
    ///
    /// # Arguments
    /// * `energy_gev` - Muon energy in GeV.
    ///
    /// # Returns
    /// Range in metres water equivalent (mwe).
    pub fn range_in_rock(energy_gev: f64) -> f64 {
        let a = 0.002; // GeV/(g/cm^2)
        let b = 4.0e-6; // 1/(g/cm^2)
        let e_min = 0.106; // muon mass in GeV
        if energy_gev <= e_min {
            return 0.0;
        }
        // Range in g/cm^2
        let range_g_cm2 = (1.0 / b) * ((a + b * energy_gev) / (a + b * e_min)).ln();
        // Convert g/cm^2 to mwe (1 mwe = 100 g/cm^2 for water density 1 g/cm^3)
        range_g_cm2 / 100.0
    }
}

// ---------------------------------------------------------------------------
// MaterialIdentifier
// ---------------------------------------------------------------------------

/// Material identification from muon scattering data.
///
/// Uses the relationship between RMS scattering angle and material properties
/// (primarily atomic number Z and radiation length) to identify or classify
/// materials.
pub struct MaterialIdentifier;

impl MaterialIdentifier {
    /// Identify the most likely material from the RMS scattering angle.
    ///
    /// Uses the Highland formula inverted: given measured scattering, thickness,
    /// and momentum, estimates the radiation length and matches against known
    /// materials.
    ///
    /// # Arguments
    /// * `scattering_rms_mrad` - Measured RMS scattering angle in mrad.
    /// * `thickness_m` - Object thickness in metres.
    /// * `momentum_gev` - Estimated muon momentum in GeV/c.
    ///
    /// # Returns
    /// Name of the most likely material as a `String`.
    pub fn identify_material(
        scattering_rms_mrad: f64,
        thickness_m: f64,
        momentum_gev: f64,
    ) -> String {
        if scattering_rms_mrad <= 0.0 || thickness_m <= 0.0 || momentum_gev <= 0.0 {
            return "unknown".to_string();
        }

        // Invert Highland: theta_0_rad = (13.6e-3/p)*sqrt(x/X0)*(1+0.038*ln(x/X0))
        // Approximate: ignore the log correction for inversion.
        // theta_0 = 13.6e-3/p * sqrt(x/X0)
        // X0 = x * (13.6e-3/p)^2 / theta_0^2
        let theta_rad = scattering_rms_mrad / 1000.0;
        let ratio = 13.6e-3 / momentum_gev;
        let x0_m = thickness_m * (ratio / theta_rad).powi(2);

        // Convert X0 from metres to g/cm^2 using material-typical density.
        // We compare the estimated X0_m with known X0_m for each material.
        let materials: &[(&str, f64, f64)] = &[
            // (name, X0 in g/cm^2, density in g/cm^3)
            ("air", X0_AIR_G_CM2, DENSITY_AIR),
            ("water", X0_WATER_G_CM2, DENSITY_WATER),
            ("aluminum", X0_ALUMINUM_G_CM2, DENSITY_ALUMINUM),
            ("concrete", X0_CONCRETE_G_CM2, DENSITY_CONCRETE),
            ("iron", X0_IRON_G_CM2, DENSITY_IRON),
            ("lead", X0_LEAD_G_CM2, DENSITY_LEAD),
            ("uranium", X0_URANIUM_G_CM2, DENSITY_URANIUM),
        ];

        // For each material, compute X0 in metres = X0_g_cm2 / (density * 100)
        // (100 converts g/cm^2 per g/cm^3 to cm, then /100 to m)
        let mut best_name = "unknown";
        let mut best_dist = f64::MAX;

        for &(name, x0_gcm2, density) in materials {
            let x0_m_material = x0_gcm2 / (density * 100.0); // metres
            let dist = ((x0_m - x0_m_material) / x0_m_material).abs();
            if dist < best_dist {
                best_dist = dist;
                best_name = name;
            }
        }

        best_name.to_string()
    }

    /// Discrimination metric for distinguishing high-Z from low-Z materials.
    ///
    /// Computes the ratio of the 90th percentile to the median of the
    /// scattering angle distribution. High-Z materials produce a broader
    /// tail due to occasional large-angle Coulomb scattering events.
    ///
    /// # Arguments
    /// * `angles` - Scattering angles in mrad.
    ///
    /// # Returns
    /// Discrimination metric (higher values indicate higher Z).
    pub fn discrimination_metric(angles: &[f64]) -> f64 {
        if angles.len() < 2 {
            return 0.0;
        }
        let mut sorted: Vec<f64> = angles.iter().map(|a| a.abs()).collect();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let n = sorted.len();
        let median_idx = n / 2;
        let p90_idx = (n as f64 * 0.9).floor() as usize;
        let p90_idx = p90_idx.min(n - 1);
        let median = sorted[median_idx];
        if median < 1e-12 {
            return 0.0;
        }
        sorted[p90_idx] / median
    }

    /// Find voxels in a density map that exceed a threat threshold.
    ///
    /// Used for detecting high-density (potentially high-Z) objects in
    /// cargo scanning applications.
    ///
    /// # Arguments
    /// * `density_map` - 3D voxel grid (as returned by POCA reconstruction).
    /// * `threshold` - Minimum scattering density to flag as a threat.
    ///
    /// # Returns
    /// List of (ix, iy, iz) voxel indices that exceed the threshold.
    pub fn threat_detection(
        density_map: &[Vec<Vec<f64>>],
        threshold: f64,
    ) -> Vec<[usize; 3]> {
        let mut threats = Vec::new();
        for (ix, plane) in density_map.iter().enumerate() {
            for (iy, row) in plane.iter().enumerate() {
                for (iz, &val) in row.iter().enumerate() {
                    if val > threshold {
                        threats.push([ix, iy, iz]);
                    }
                }
            }
        }
        threats
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Dot product of two 3D vectors.
fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Normalise a 3D vector to unit length.
fn normalize3(v: [f64; 3]) -> [f64; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len < 1e-15 {
        return [0.0, 0.0, 0.0];
    }
    [v[0] / len, v[1] / len, v[2] / len]
}

/// Signed angular difference wrapped to [-pi, pi].
fn angle_diff(a: f64, b: f64) -> f64 {
    let mut d = a - b;
    while d > PI {
        d -= 2.0 * PI;
    }
    while d < -PI {
        d += 2.0 * PI;
    }
    d
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn default_config() -> MuonConfig {
        MuonConfig::default()
    }

    fn make_tracker() -> MuonTracker {
        MuonTracker::new(default_config())
    }

    // --- Scattering angle tests ---

    #[test]
    fn test_scattering_angle_parallel() {
        let tracker = make_tracker();
        let dir = [0.0, 0.0, -1.0];
        let angle = tracker.scattering_angle(dir, dir);
        assert!(angle.abs() < 1e-6, "Parallel tracks should give 0 scattering");
    }

    #[test]
    fn test_scattering_angle_orthogonal() {
        let tracker = make_tracker();
        let d1 = [0.0, 0.0, -1.0];
        let d2 = [1.0, 0.0, 0.0];
        let angle = tracker.scattering_angle(d1, d2);
        let expected = PI / 2.0 * 1000.0; // 90 degrees in mrad
        assert!(
            (angle - expected).abs() < 1.0,
            "Expected ~{:.1} mrad, got {:.1} mrad",
            expected,
            angle
        );
    }

    #[test]
    fn test_scattering_angle_antiparallel() {
        let tracker = make_tracker();
        let d1 = [0.0, 0.0, -1.0];
        let d2 = [0.0, 0.0, 1.0];
        let angle = tracker.scattering_angle(d1, d2);
        let expected = PI * 1000.0; // 180 degrees in mrad
        assert!(
            (angle - expected).abs() < 1.0,
            "Expected ~{:.1} mrad, got {:.1} mrad",
            expected,
            angle
        );
    }

    #[test]
    fn test_scattering_angle_small_deflection() {
        let tracker = make_tracker();
        let d1 = normalize3([0.0, 0.0, -1.0]);
        let d2 = normalize3([0.01, 0.0, -1.0]);
        let angle = tracker.scattering_angle(d1, d2);
        // Should be approximately atan(0.01) ~ 10 mrad
        assert!(angle > 5.0 && angle < 15.0, "Small deflection: got {} mrad", angle);
    }

    // --- Highland formula tests ---

    #[test]
    fn test_highland_increases_with_thickness() {
        let thin = MuonTracker::highland_formula(0.01, 0.0056, 3.0);
        let thick = MuonTracker::highland_formula(0.10, 0.0056, 3.0);
        assert!(
            thick > thin,
            "Thicker material should scatter more: thin={}, thick={}",
            thin,
            thick
        );
    }

    #[test]
    fn test_highland_decreases_with_momentum() {
        let low_p = MuonTracker::highland_formula(0.10, 0.0056, 1.0);
        let high_p = MuonTracker::highland_formula(0.10, 0.0056, 10.0);
        assert!(
            low_p > high_p,
            "Higher momentum should scatter less: low_p={}, high_p={}",
            low_p,
            high_p
        );
    }

    #[test]
    fn test_highland_zero_thickness() {
        let theta = MuonTracker::highland_formula(0.0, 0.0056, 3.0);
        assert_eq!(theta, 0.0, "Zero thickness should give zero scattering");
    }

    #[test]
    fn test_highland_zero_momentum() {
        let theta = MuonTracker::highland_formula(0.10, 0.0056, 0.0);
        assert_eq!(theta, 0.0, "Zero momentum should return 0 (guard)");
    }

    #[test]
    fn test_highland_positive() {
        let theta = MuonTracker::highland_formula(0.10, 0.0056, 3.0);
        assert!(theta > 0.0, "Highland formula should give positive angle: {}", theta);
    }

    #[test]
    fn test_highland_proportional_to_sqrt_thickness() {
        // For fixed p and X0, theta ~ sqrt(x) (ignoring log correction)
        let t1 = MuonTracker::highland_formula(0.01, 0.01, 3.0);
        let t4 = MuonTracker::highland_formula(0.04, 0.01, 3.0);
        // t4 should be roughly 2x t1 (sqrt(4)=2), but log correction modifies this
        let ratio = t4 / t1;
        assert!(
            ratio > 1.5 && ratio < 2.5,
            "Expected ratio ~2.0, got {}",
            ratio
        );
    }

    // --- Radiation length tests ---

    #[test]
    fn test_radiation_length_lead() {
        let x0 = MuonTracker::radiation_length(82.0, 207.2);
        assert!(
            (x0 - X0_LEAD_G_CM2).abs() < 2.0,
            "Lead X0 should be ~6.37 g/cm^2, got {}",
            x0
        );
    }

    #[test]
    fn test_radiation_length_iron() {
        let x0 = MuonTracker::radiation_length(26.0, 55.85);
        assert!(
            (x0 - X0_IRON_G_CM2).abs() < 3.0,
            "Iron X0 should be ~13.84 g/cm^2, got {}",
            x0
        );
    }

    #[test]
    fn test_radiation_length_aluminum() {
        let x0 = MuonTracker::radiation_length(13.0, 26.98);
        assert!(
            (x0 - X0_ALUMINUM_G_CM2).abs() < 3.0,
            "Aluminum X0 should be ~24.01 g/cm^2, got {}",
            x0
        );
    }

    #[test]
    fn test_radiation_length_high_z_shorter() {
        let x0_al = MuonTracker::radiation_length(13.0, 26.98);
        let x0_pb = MuonTracker::radiation_length(82.0, 207.2);
        assert!(
            x0_pb < x0_al,
            "Higher Z should have shorter radiation length: Al={}, Pb={}",
            x0_al,
            x0_pb
        );
    }

    #[test]
    fn test_radiation_length_zero_z() {
        assert_eq!(MuonTracker::radiation_length(0.0, 1.0), 0.0);
    }

    // --- POCA tests ---

    #[test]
    fn test_poca_symmetric() {
        // Two skew lines symmetric about origin
        let p1 = [1.0, 0.0, 0.0];
        let d1 = [0.0, 1.0, 0.0];
        let p2 = [0.0, 0.0, 0.0];
        let d2 = [0.0, 0.0, 1.0];
        let poca = MuonTracker::point_of_closest_approach(p1, d1, p2, d2);
        // Closest approach: line1 at s=0: (1,0,0), line2 at t=0: (0,0,0)
        // Midpoint should be (0.5, 0, 0)
        assert!(
            (poca[0] - 0.5).abs() < 1e-6,
            "POCA x should be 0.5, got {}",
            poca[0]
        );
        assert!(poca[1].abs() < 1e-6, "POCA y should be 0, got {}", poca[1]);
        assert!(poca[2].abs() < 1e-6, "POCA z should be 0, got {}", poca[2]);
    }

    #[test]
    fn test_poca_intersecting_lines() {
        // Two lines that intersect at the origin
        let p1 = [-1.0, -1.0, 0.0];
        let d1 = [1.0, 1.0, 0.0];
        let p2 = [1.0, -1.0, 0.0];
        let d2 = [-1.0, 1.0, 0.0];
        let poca = MuonTracker::point_of_closest_approach(p1, d1, p2, d2);
        assert!(
            poca[0].abs() < 1e-6 && poca[1].abs() < 1e-6,
            "Intersecting lines: POCA should be at origin, got {:?}",
            poca
        );
    }

    #[test]
    fn test_poca_parallel_lines() {
        let p1 = [0.0, 0.0, 0.0];
        let d1 = [0.0, 0.0, 1.0];
        let p2 = [2.0, 0.0, 0.0];
        let d2 = [0.0, 0.0, 1.0];
        let poca = MuonTracker::point_of_closest_approach(p1, d1, p2, d2);
        // For parallel lines, midpoint of the two positions
        assert!(
            (poca[0] - 1.0).abs() < 1e-3,
            "Parallel POCA x ~1.0, got {}",
            poca[0]
        );
    }

    // --- Projected scattering tests ---

    #[test]
    fn test_projected_scattering_zero_for_parallel() {
        let entry = MuonTrack {
            entry_position: [0.0, 0.0, 1.0],
            entry_direction: [0.0, 0.0, -1.0],
            exit_position: [0.0, 0.0, -1.0],
            exit_direction: [0.0, 0.0, -1.0],
            momentum_gev: 3.0,
        };
        let exit = entry.clone();
        let (dx, dy) = MuonTracker::projected_scattering(&entry, &exit);
        assert!(dx.abs() < 1e-6 && dy.abs() < 1e-6, "No scattering expected");
    }

    #[test]
    fn test_projected_scattering_x_only() {
        let entry = MuonTrack {
            entry_position: [0.0, 0.0, 1.0],
            entry_direction: normalize3([0.0, 0.0, -1.0]),
            exit_position: [0.1, 0.0, -1.0],
            exit_direction: normalize3([0.0, 0.0, -1.0]),
            momentum_gev: 3.0,
        };
        let exit = MuonTrack {
            entry_position: [0.0, 0.0, 1.0],
            entry_direction: normalize3([0.0, 0.0, -1.0]),
            exit_position: [0.1, 0.0, -1.0],
            exit_direction: normalize3([0.01, 0.0, -1.0]),
            momentum_gev: 3.0,
        };
        let (dx, dy) = MuonTracker::projected_scattering(&entry, &exit);
        assert!(dx.abs() > 0.1, "Should have non-zero x-projection scattering");
        assert!(dy.abs() < 1e-6, "Should have ~zero y-projection scattering");
    }

    // --- Flux model tests ---

    #[test]
    fn test_sea_level_flux_zenith() {
        let flux_0 = MuonFluxModel::sea_level_flux(0.0);
        assert!(
            (flux_0 - MuonFluxModel::VERTICAL_FLUX).abs() < 1e-10,
            "Zenith flux should equal vertical flux"
        );
    }

    #[test]
    fn test_sea_level_flux_cos2_dependence() {
        let f0 = MuonFluxModel::sea_level_flux(0.0);
        let f45 = MuonFluxModel::sea_level_flux(45.0);
        let expected_ratio = (45.0_f64.to_radians()).cos().powi(2);
        let ratio = f45 / f0;
        assert!(
            (ratio - expected_ratio).abs() < 1e-6,
            "cos^2 dependence: expected ratio {}, got {}",
            expected_ratio,
            ratio
        );
    }

    #[test]
    fn test_sea_level_flux_peaks_at_zenith() {
        let f0 = MuonFluxModel::sea_level_flux(0.0);
        let f30 = MuonFluxModel::sea_level_flux(30.0);
        let f60 = MuonFluxModel::sea_level_flux(60.0);
        assert!(f0 > f30, "Flux should decrease from zenith");
        assert!(f30 > f60, "Flux should decrease with zenith angle");
    }

    #[test]
    fn test_sea_level_flux_horizon() {
        let f90 = MuonFluxModel::sea_level_flux(90.0);
        assert!(f90.abs() < 1e-10, "Flux at horizon should be ~0");
    }

    #[test]
    fn test_underground_flux_attenuation() {
        let surface = MuonFluxModel::underground_flux(0.0);
        let deep = MuonFluxModel::underground_flux(1000.0);
        assert!(
            deep < surface,
            "Underground flux should be less: surface={}, deep={}",
            surface,
            deep
        );
    }

    #[test]
    fn test_underground_flux_zero_depth() {
        let f = MuonFluxModel::underground_flux(0.0);
        assert!(
            (f - MuonFluxModel::VERTICAL_FLUX).abs() < 1e-10,
            "Zero depth should equal surface flux"
        );
    }

    #[test]
    fn test_expected_counts_proportional_to_time() {
        let c1 = MuonFluxModel::expected_counts(0.007, 1.0, 1.0, 100.0);
        let c2 = MuonFluxModel::expected_counts(0.007, 1.0, 1.0, 200.0);
        assert!(
            (c2 / c1 - 2.0).abs() < 1e-6,
            "Counts should be proportional to time"
        );
    }

    #[test]
    fn test_expected_counts_proportional_to_area() {
        let c1 = MuonFluxModel::expected_counts(0.007, 1.0, 1.0, 100.0);
        let c2 = MuonFluxModel::expected_counts(0.007, 2.0, 1.0, 100.0);
        assert!(
            (c2 / c1 - 2.0).abs() < 1e-6,
            "Counts should be proportional to area"
        );
    }

    #[test]
    fn test_energy_loss_rate_positive() {
        let rate = MuonFluxModel::muon_energy_loss_rate(3.0);
        assert!(rate > 0.0, "Energy loss rate should be positive: {}", rate);
    }

    #[test]
    fn test_energy_loss_increases_with_energy() {
        let low = MuonFluxModel::muon_energy_loss_rate(1.0);
        let high = MuonFluxModel::muon_energy_loss_rate(100.0);
        assert!(
            high > low,
            "Higher energy should have higher loss rate: low={}, high={}",
            low,
            high
        );
    }

    #[test]
    fn test_range_increases_with_energy() {
        let r1 = MuonFluxModel::range_in_rock(1.0);
        let r10 = MuonFluxModel::range_in_rock(10.0);
        assert!(
            r10 > r1,
            "Higher energy muons should penetrate further: r1={}, r10={}",
            r1,
            r10
        );
    }

    #[test]
    fn test_range_zero_below_mass() {
        let r = MuonFluxModel::range_in_rock(0.1);
        assert_eq!(r, 0.0, "Muons below rest mass can't propagate");
    }

    // --- DensityReconstructor tests ---

    #[test]
    fn test_voxel_index_inside() {
        let recon = DensityReconstructor::new(([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]), 0.1);
        let idx = recon.voxel_index([0.05, 0.05, 0.05]);
        assert_eq!(idx, Some([0, 0, 0]));
    }

    #[test]
    fn test_voxel_index_outside() {
        let recon = DensityReconstructor::new(([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]), 0.1);
        assert!(recon.voxel_index([-0.1, 0.5, 0.5]).is_none());
        assert!(recon.voxel_index([1.1, 0.5, 0.5]).is_none());
    }

    #[test]
    fn test_voxel_index_consistency() {
        let recon = DensityReconstructor::new(([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]), 0.25);
        // Position at centre of voxel (1,2,3)
        let pos = [0.375, 0.625, 0.875];
        let idx = recon.voxel_index(pos);
        assert_eq!(idx, Some([1, 2, 3]));
    }

    #[test]
    fn test_poca_reconstruction_with_object() {
        // Simulate a dense object in the centre: tracks hitting the centre
        // should have larger scattering angles.
        // Grid: [0,1]^3 with voxel size 0.5 → 2x2x2 voxels.
        let recon = DensityReconstructor::new(([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]), 0.5);

        let mut tracks = Vec::new();
        // Create skew entry/exit lines whose POCA falls in the centre
        // of the grid. Entry line: at (0.5, 0.5, 1.5) going slightly right.
        // Exit line: at (0.5, 0.5, -0.5) going slightly left.
        // The POCA of these skew lines lands near (0.5, 0.5, 0.5).
        for _ in 0..50 {
            let entry = MuonTrack {
                entry_position: [0.5, 0.5, 1.5],
                entry_direction: normalize3([0.02, 0.0, -1.0]),
                exit_position: [0.5, 0.5, -0.5],
                exit_direction: normalize3([0.02, 0.0, -1.0]),
                momentum_gev: 3.0,
            };
            let exit = MuonTrack {
                entry_position: [0.5, 0.5, 1.5],
                entry_direction: normalize3([0.02, 0.0, -1.0]),
                exit_position: [0.5, 0.5, -0.5],
                exit_direction: normalize3([-0.02, 0.0, -1.0]),
                momentum_gev: 3.0,
            };
            tracks.push((entry, exit));
        }

        let map = recon.poca_reconstruct(&tracks);
        assert!(!map.is_empty(), "Density map should not be empty");
        // Check that at least one voxel has accumulated scattering
        let mut max_val = 0.0_f64;
        for plane in &map {
            for row in plane {
                for &val in row {
                    if val > max_val {
                        max_val = val;
                    }
                }
            }
        }
        assert!(
            max_val > 0.0,
            "At least one voxel should have scattering density > 0, max={}",
            max_val
        );
    }

    #[test]
    fn test_scattering_density_median() {
        let angles = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let med = DensityReconstructor::scattering_density(&angles);
        assert!((med - 3.0).abs() < 1e-6, "Median should be 3.0, got {}", med);
    }

    #[test]
    fn test_scattering_density_even_count() {
        let angles = vec![1.0, 2.0, 3.0, 4.0];
        let med = DensityReconstructor::scattering_density(&angles);
        assert!(
            (med - 2.5).abs() < 1e-6,
            "Median of even count should be 2.5, got {}",
            med
        );
    }

    #[test]
    fn test_scattering_density_empty() {
        let med = DensityReconstructor::scattering_density(&[]);
        assert_eq!(med, 0.0);
    }

    #[test]
    fn test_transmission_tomography() {
        let recon = DensityReconstructor::new(([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]), 0.5);
        let mut map = vec![vec![0.0; 4]; 4];

        let tracks: Vec<MuonTrack> = (0..20)
            .map(|_| MuonTrack {
                entry_position: [0.1, 0.1, 1.0],
                entry_direction: [0.0, 0.0, -1.0],
                exit_position: [0.1, 0.1, 0.0],
                exit_direction: [0.0, 0.0, -1.0],
                momentum_gev: 3.0,
            })
            .collect();

        recon.transmission_tomography(&tracks, &mut map);
        // All tracks land in pixel (0,0) of the 4x4 grid
        assert!(
            map[0][0] > 0.0,
            "Should have counts in pixel (0,0): {}",
            map[0][0]
        );
        assert_eq!(map[0][0], 20.0, "All 20 tracks should land in same pixel");
    }

    // --- MaterialIdentifier tests ---

    #[test]
    fn test_identify_material_high_scattering_high_z() {
        // Large scattering → short X0 → high-Z
        let mat = MaterialIdentifier::identify_material(50.0, 0.1, 3.0);
        // With high scattering, we expect a high-Z material
        assert!(
            mat == "lead" || mat == "uranium" || mat == "iron",
            "High scattering should identify high-Z: got {}",
            mat
        );
    }

    #[test]
    fn test_identify_material_low_scattering_low_z() {
        // Very small scattering → long X0 → low-Z
        let mat = MaterialIdentifier::identify_material(0.5, 0.1, 3.0);
        assert!(
            mat == "air" || mat == "water" || mat == "aluminum" || mat == "concrete",
            "Low scattering should identify low-Z: got {}",
            mat
        );
    }

    #[test]
    fn test_identify_material_zero_scattering() {
        let mat = MaterialIdentifier::identify_material(0.0, 0.1, 3.0);
        assert_eq!(mat, "unknown");
    }

    #[test]
    fn test_discrimination_metric() {
        // Broad distribution (high Z)
        let broad = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];
        // Narrow distribution (low Z)
        let narrow = vec![4.5, 4.8, 5.0, 5.1, 5.2, 5.0, 4.9, 5.3, 5.1, 5.0];
        let dm_broad = MaterialIdentifier::discrimination_metric(&broad);
        let dm_narrow = MaterialIdentifier::discrimination_metric(&narrow);
        assert!(
            dm_broad > dm_narrow,
            "Broad distribution should have higher metric: broad={}, narrow={}",
            dm_broad,
            dm_narrow
        );
    }

    #[test]
    fn test_threat_detection() {
        let density_map = vec![
            vec![vec![0.1, 0.2], vec![0.3, 10.0]],
            vec![vec![0.5, 0.1], vec![0.2, 0.3]],
        ];
        let threats = MaterialIdentifier::threat_detection(&density_map, 5.0);
        assert_eq!(threats.len(), 1, "Should find exactly one threat voxel");
        assert_eq!(threats[0], [0, 1, 1], "Threat at [0][1][1]");
    }

    #[test]
    fn test_threat_detection_none() {
        let density_map = vec![vec![vec![0.1, 0.2], vec![0.3, 0.4]]];
        let threats = MaterialIdentifier::threat_detection(&density_map, 5.0);
        assert!(threats.is_empty(), "No threats expected below threshold");
    }

    // --- ASR tests ---

    #[test]
    fn test_asr_runs_without_panic() {
        let recon = DensityReconstructor::new(([0.0, 0.0, 0.0], [0.5, 0.5, 0.5]), 0.25);
        let tracks: Vec<(MuonTrack, MuonTrack)> = (0..10)
            .map(|_| {
                let entry = MuonTrack {
                    entry_position: [0.25, 0.25, 0.75],
                    entry_direction: normalize3([0.0, 0.0, -1.0]),
                    exit_position: [0.25, 0.25, -0.25],
                    exit_direction: normalize3([0.0, 0.0, -1.0]),
                    momentum_gev: 3.0,
                };
                let exit = MuonTrack {
                    entry_position: [0.25, 0.25, 0.75],
                    entry_direction: normalize3([0.0, 0.0, -1.0]),
                    exit_position: [0.25, 0.25, -0.25],
                    exit_direction: normalize3([0.02, 0.01, -1.0]),
                    momentum_gev: 3.0,
                };
                (entry, exit)
            })
            .collect();

        let result = recon.angular_statistical_reconstruction(&tracks, 3);
        assert!(!result.is_empty(), "ASR should produce a grid");
    }

    // --- Energy spectrum tests ---

    #[test]
    fn test_energy_spectrum_variants() {
        let _sl = EnergySpectrum::SeaLevel;
        let _ug = EnergySpectrum::Underground { depth_mwe: 500.0 };
        let _custom = EnergySpectrum::Custom { mean_gev: 5.0 };
        // Just ensure they can be constructed without panics
    }

    // --- Radiation length constants ---

    #[test]
    fn test_radiation_length_ordering() {
        // Higher Z materials should have shorter radiation lengths (in g/cm^2)
        assert!(
            X0_LEAD_G_CM2 < X0_IRON_G_CM2,
            "Lead < Iron: {} < {}",
            X0_LEAD_G_CM2,
            X0_IRON_G_CM2
        );
        assert!(
            X0_IRON_G_CM2 < X0_ALUMINUM_G_CM2,
            "Iron < Aluminum: {} < {}",
            X0_IRON_G_CM2,
            X0_ALUMINUM_G_CM2
        );
        assert!(
            X0_URANIUM_G_CM2 < X0_LEAD_G_CM2,
            "Uranium < Lead: {} < {}",
            X0_URANIUM_G_CM2,
            X0_LEAD_G_CM2
        );
    }

    #[test]
    fn test_radiation_length_constants_positive() {
        assert!(X0_AIR_G_CM2 > 0.0);
        assert!(X0_WATER_G_CM2 > 0.0);
        assert!(X0_CONCRETE_G_CM2 > 0.0);
        assert!(X0_ALUMINUM_G_CM2 > 0.0);
        assert!(X0_IRON_G_CM2 > 0.0);
        assert!(X0_LEAD_G_CM2 > 0.0);
        assert!(X0_URANIUM_G_CM2 > 0.0);
    }

    // --- Helper tests ---

    #[test]
    fn test_normalize3() {
        let v = normalize3([3.0, 4.0, 0.0]);
        let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        assert!((len - 1.0).abs() < 1e-10, "Should be unit length: {}", len);
    }

    #[test]
    fn test_normalize3_zero() {
        let v = normalize3([0.0, 0.0, 0.0]);
        assert_eq!(v, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_dot3() {
        let a = [1.0, 2.0, 3.0];
        let b = [4.0, 5.0, 6.0];
        assert!((dot3(a, b) - 32.0).abs() < 1e-10);
    }

    #[test]
    fn test_angle_diff_wrap() {
        let d = angle_diff(3.0, -3.0);
        // 6.0 rad wraps to 6.0 - 2*pi ~ -0.283
        assert!(d.abs() < PI, "Should wrap to [-pi, pi]: {}", d);
    }

    #[test]
    fn test_grid_dims() {
        let recon = DensityReconstructor::new(([0.0, 0.0, 0.0], [1.0, 2.0, 3.0]), 0.5);
        assert_eq!(recon.grid_dims, [2, 4, 6]);
    }

    #[test]
    fn test_config_default() {
        let cfg = MuonConfig::default();
        assert_eq!(cfg.detector_spacing_m, 2.0);
        assert_eq!(cfg.angular_resolution_mrad, 1.0);
        assert_eq!(cfg.position_resolution_mm, 0.5);
        assert_eq!(cfg.voxel_size_m, 0.05);
        assert_eq!(cfg.energy_spectrum, EnergySpectrum::SeaLevel);
    }
}
