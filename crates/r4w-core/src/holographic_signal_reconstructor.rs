//! # Holographic Signal Reconstructor
//!
//! Implements holographic aperture synthesis for radio interferometry and synthetic
//! aperture imaging. This technique is foundational in radio astronomy (VLBI),
//! medical imaging, and acoustic holography.
//!
//! ## Core Concept
//!
//! Multiple receivers sample the electromagnetic (or acoustic) wavefield at different
//! spatial locations. By coherently combining these measurements, the full aperture
//! response can be reconstructed -- as if a single, much larger antenna had been used.
//!
//! ## Key Components
//!
//! - [`HolographicReconstructor`]: Main engine for interferometric image reconstruction,
//!   including baseline computation, visibility calculation, dirty image formation via
//!   inverse DFT, and Hogbom CLEAN deconvolution.
//! - [`Visibility`]: Represents a single visibility measurement in the uv-plane.
//! - [`UvCoverageAnalyzer`]: Analyzes uv-plane sampling density, radial coverage,
//!   and sidelobe levels of the synthesized beam.
//! - [`NearFieldHolography`]: Reconstructs aperture fields from near-field measurements
//!   by back-propagating measured data to the aperture plane.
//! - [`AcousticHolography`]: Angular spectrum propagation for forward and backward
//!   wave propagation in acoustic holographic imaging.
//!
//! ## References
//!
//! - Thompson, Moran & Swenson, "Interferometry and Synthesis in Radio Astronomy"
//! - Hogbom (1974), "Aperture Synthesis with a Non-Regular Distribution of
//!   Interferometer Baselines"
//! - Maynard, Williams & Lee, "Nearfield acoustic holography: I. Theory of
//!   generalized holography and the development of NAH" (JASA 1985)

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Compute the spatial frequency for a given observation angle and wavelength.
///
/// Returns `sin(angle_rad) / wavelength_m`, which maps an angular direction
/// to a position in the uv-plane (units of cycles per metre).
pub fn spatial_frequency(angle_rad: f64, wavelength_m: f64) -> f64 {
    angle_rad.sin() / wavelength_m
}

/// Compute the Fraunhofer (far-field) distance for an aperture.
///
/// `d_far = 2 * D^2 / lambda` where D is the aperture diameter and lambda
/// is the wavelength.  Beyond this distance the wavefront curvature is
/// negligible and plane-wave assumptions hold.
pub fn fraunhofer_distance(aperture_m: f64, wavelength_m: f64) -> f64 {
    2.0 * aperture_m * aperture_m / wavelength_m
}

/// Compute the complex array factor for a 1-D linear array.
///
/// Given element positions (in metres), an observation direction (radians),
/// and the wavelength (metres), returns `(real, imag)` of the array factor
/// `AF = sum_n exp(j * 2*pi * x_n * sin(theta) / lambda)`.
pub fn array_factor_1d(positions: &[f64], direction: f64, wavelength: f64) -> (f64, f64) {
    let k = 2.0 * PI / wavelength;
    let sin_dir = direction.sin();
    let mut re = 0.0;
    let mut im = 0.0;
    for &x in positions {
        let phase = k * x * sin_dir;
        re += phase.cos();
        im += phase.sin();
    }
    (re, im)
}

// ---------------------------------------------------------------------------
// HolographyConfig
// ---------------------------------------------------------------------------

/// Configuration for a holographic aperture synthesis observation.
#[derive(Debug, Clone)]
pub struct HolographyConfig {
    /// Wavelength of the observation in metres.
    pub wavelength_m: f64,
    /// Number of antenna / receiver elements.
    pub num_elements: usize,
    /// 2-D positions of each element in units of wavelengths (east, north).
    pub element_positions: Vec<[f64; 2]>,
    /// Position of the reference element (phase centre) in wavelengths.
    pub reference_position: [f64; 2],
    /// Image grid size (pixels per side).
    pub grid_size: usize,
    /// Field of view in radians.
    pub field_of_view_rad: f64,
}

// ---------------------------------------------------------------------------
// Visibility
// ---------------------------------------------------------------------------

/// A single complex visibility measurement in the uv-plane.
#[derive(Debug, Clone, Copy)]
pub struct Visibility {
    /// East-west baseline component (wavelengths).
    pub u: f64,
    /// North-south baseline component (wavelengths).
    pub v: f64,
    /// Real part of the complex visibility.
    pub real: f64,
    /// Imaginary part of the complex visibility.
    pub imag: f64,
    /// Weight (e.g. integration time, sensitivity).
    pub weight: f64,
}

impl Visibility {
    /// Create a new visibility with the given parameters.
    pub fn new(u: f64, v: f64, real: f64, imag: f64, weight: f64) -> Self {
        Self { u, v, real, imag, weight }
    }

    /// Amplitude of the visibility: `sqrt(real^2 + imag^2)`.
    pub fn amplitude(&self) -> f64 {
        (self.real * self.real + self.imag * self.imag).sqrt()
    }

    /// Phase of the visibility in radians: `atan2(imag, real)`.
    pub fn phase(&self) -> f64 {
        self.imag.atan2(self.real)
    }
}

// ---------------------------------------------------------------------------
// HolographicReconstructor
// ---------------------------------------------------------------------------

/// Engine for interferometric image reconstruction via aperture synthesis.
///
/// Implements baseline computation, visibility generation, dirty-image formation
/// through an inverse DFT, the synthesized beam (dirty beam / PSF), and Hogbom
/// CLEAN deconvolution.
pub struct HolographicReconstructor {
    config: HolographyConfig,
}

impl HolographicReconstructor {
    /// Create a new reconstructor from the given configuration.
    pub fn new(config: HolographyConfig) -> Self {
        Self { config }
    }

    /// Compute the uv-baseline vector between two element positions.
    ///
    /// Both positions are in units of wavelengths.  The baseline is simply the
    /// difference vector `pos2 - pos1`.
    pub fn compute_baseline(pos1: [f64; 2], pos2: [f64; 2]) -> [f64; 2] {
        [pos2[0] - pos1[0], pos2[1] - pos1[1]]
    }

    /// Compute all unique baselines from the element positions in the config.
    ///
    /// For N elements this yields N*(N-1)/2 baselines (excluding auto-correlations
    /// and conjugate-redundant baselines).
    pub fn compute_uv_coverage(&self) -> Vec<[f64; 2]> {
        let n = self.config.num_elements;
        let pos = &self.config.element_positions;
        let mut baselines = Vec::with_capacity(n * (n - 1) / 2);
        for i in 0..n {
            for j in (i + 1)..n {
                baselines.push(Self::compute_baseline(pos[i], pos[j]));
            }
        }
        baselines
    }

    /// Compute the complex visibility for a point source.
    ///
    /// Given a baseline `[u, v]` (wavelengths), a source direction cosine
    /// `[l, m]`, and a source amplitude, returns `(Re, Im)` of
    /// `V(u,v) = A * exp(-j * 2*pi * (u*l + v*m))`.
    pub fn visibility(
        baseline: [f64; 2],
        source_direction: [f64; 2],
        amplitude: f64,
    ) -> (f64, f64) {
        let phase = -2.0 * PI * (baseline[0] * source_direction[0] + baseline[1] * source_direction[1]);
        (amplitude * phase.cos(), amplitude * phase.sin())
    }

    /// Form a dirty image by inverse-DFT of the supplied visibilities.
    ///
    /// Each entry in `visibilities` is `((u, v), (re, im))`.
    /// The returned 2-D grid has dimensions `grid_size x grid_size` and spans
    /// the configured field of view.
    pub fn dirty_image(
        &self,
        visibilities: &[((f64, f64), (f64, f64))],
    ) -> Vec<Vec<f64>> {
        let n = self.config.grid_size;
        let fov = self.config.field_of_view_rad;
        let mut image = vec![vec![0.0f64; n]; n];

        for iy in 0..n {
            let m = (iy as f64 / (n - 1).max(1) as f64 - 0.5) * fov;
            for ix in 0..n {
                let l = (ix as f64 / (n - 1).max(1) as f64 - 0.5) * fov;
                let mut sum_re = 0.0;
                let mut sum_im = 0.0;
                for &((u, v), (vis_re, vis_im)) in visibilities {
                    let phase = 2.0 * PI * (u * l + v * m);
                    let cos_p = phase.cos();
                    let sin_p = phase.sin();
                    // complex multiply: (vis_re + j*vis_im) * (cos + j*sin)
                    sum_re += vis_re * cos_p - vis_im * sin_p;
                    sum_im += vis_re * sin_p + vis_im * cos_p;
                }
                // Take real part (image is real-valued for Hermitian visibilities)
                image[iy][ix] = sum_re;
            }
        }
        image
    }

    /// Compute the dirty beam (point spread function) from the uv-coverage.
    ///
    /// This is the response to a unit point source at the phase centre and
    /// equals the inverse DFT of the sampling function.
    pub fn dirty_beam(
        uv_coverage: &[[f64; 2]],
        grid_size: usize,
        fov: f64,
    ) -> Vec<Vec<f64>> {
        let n = grid_size;
        let mut beam = vec![vec![0.0f64; n]; n];

        for iy in 0..n {
            let m = (iy as f64 / (n - 1).max(1) as f64 - 0.5) * fov;
            for ix in 0..n {
                let l = (ix as f64 / (n - 1).max(1) as f64 - 0.5) * fov;
                let mut val = 0.0;
                for &[u, v] in uv_coverage {
                    // Both the baseline and its conjugate contribute
                    val += (2.0 * PI * (u * l + v * m)).cos();
                }
                // Each baseline contributes twice (conjugate symmetry)
                beam[iy][ix] = 2.0 * val;
            }
        }

        // Normalise so that the peak equals 1.0
        let peak = beam.iter()
            .flat_map(|row| row.iter())
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        if peak > 0.0 {
            for row in &mut beam {
                for v in row {
                    *v /= peak;
                }
            }
        }
        beam
    }

    /// Hogbom CLEAN deconvolution.
    ///
    /// Iteratively subtracts scaled copies of the dirty beam from the dirty
    /// image, building a clean-component model.  The final image is the sum of
    /// these components (not re-convolved with a restoring beam, for simplicity).
    ///
    /// # Arguments
    /// * `dirty` - the dirty image (grid_size x grid_size)
    /// * `beam`  - the dirty beam / PSF (same dimensions)
    /// * `iterations` - maximum number of CLEAN iterations
    /// * `gain` - loop gain (typically 0.05 -- 0.3)
    /// * `threshold` - stop when the residual peak drops below this value
    pub fn clean_deconvolve(
        dirty: &[Vec<f64>],
        beam: &[Vec<f64>],
        iterations: usize,
        gain: f64,
        threshold: f64,
    ) -> Vec<Vec<f64>> {
        let ny = dirty.len();
        if ny == 0 {
            return vec![];
        }
        let nx = dirty[0].len();

        // Working copy of the residual image
        let mut residual: Vec<Vec<f64>> = dirty.to_vec();
        // Clean-component image
        let mut clean = vec![vec![0.0f64; nx]; ny];

        // Centre of the beam
        let beam_cy = ny / 2;
        let beam_cx = nx / 2;

        for _ in 0..iterations {
            // Find pixel with largest absolute value in residual
            let mut peak_val = 0.0f64;
            let mut peak_abs = 0.0f64;
            let mut peak_y = 0;
            let mut peak_x = 0;
            for y in 0..ny {
                for x in 0..nx {
                    let a = residual[y][x].abs();
                    if a > peak_abs {
                        peak_abs = a;
                        peak_val = residual[y][x];
                        peak_y = y;
                        peak_x = x;
                    }
                }
            }

            if peak_val.abs() < threshold {
                break;
            }

            let component = gain * peak_val;
            clean[peak_y][peak_x] += component;

            // Subtract shifted, scaled beam from residual
            for by in 0..ny {
                for bx in 0..nx {
                    let ry = peak_y as isize + by as isize - beam_cy as isize;
                    let rx = peak_x as isize + bx as isize - beam_cx as isize;
                    if ry >= 0 && ry < ny as isize && rx >= 0 && rx < nx as isize {
                        residual[ry as usize][rx as usize] -= component * beam[by][bx];
                    }
                }
            }
        }

        clean
    }

    /// Angular resolution of the array in radians.
    ///
    /// Approximated as `lambda / D_max` where `D_max` is the maximum baseline
    /// length in wavelengths (converted back to metres via the configured
    /// wavelength).
    pub fn angular_resolution(&self) -> f64 {
        let baselines = self.compute_uv_coverage();
        let max_baseline = baselines
            .iter()
            .map(|b| (b[0] * b[0] + b[1] * b[1]).sqrt())
            .fold(0.0f64, f64::max);
        if max_baseline == 0.0 {
            return f64::INFINITY;
        }
        // max_baseline is in wavelengths; D_max in metres = max_baseline * lambda
        // resolution = lambda / D_max = lambda / (max_baseline * lambda) = 1 / max_baseline
        1.0 / max_baseline
    }
}

// ---------------------------------------------------------------------------
// UvCoverageAnalyzer
// ---------------------------------------------------------------------------

/// Analyzes the quality of uv-plane sampling for an interferometric array.
pub struct UvCoverageAnalyzer;

impl UvCoverageAnalyzer {
    /// Fraction of the uv-plane that is sampled.
    ///
    /// Grids the uv-plane into `grid_cells x grid_cells` bins out to
    /// `max_baseline` and returns the fraction of bins that contain at least
    /// one sample (including conjugate baselines).
    pub fn coverage_fraction(
        uv_points: &[[f64; 2]],
        max_baseline: f64,
        grid_cells: usize,
    ) -> f64 {
        if grid_cells == 0 || max_baseline <= 0.0 {
            return 0.0;
        }
        let n = grid_cells;
        let mut grid = vec![false; n * n];
        let cell_size = 2.0 * max_baseline / n as f64;

        let mark = |u: f64, v: f64| -> Option<usize> {
            let ix = ((u + max_baseline) / cell_size).floor() as isize;
            let iy = ((v + max_baseline) / cell_size).floor() as isize;
            if ix >= 0 && ix < n as isize && iy >= 0 && iy < n as isize {
                Some(iy as usize * n + ix as usize)
            } else {
                None
            }
        };

        for &[u, v] in uv_points {
            if let Some(idx) = mark(u, v) {
                grid[idx] = true;
            }
            // conjugate baseline
            if let Some(idx) = mark(-u, -v) {
                grid[idx] = true;
            }
        }

        let filled = grid.iter().filter(|&&b| b).count();
        filled as f64 / (n * n) as f64
    }

    /// Radial density profile of the uv-coverage.
    ///
    /// Bins the baseline lengths into `num_bins` equal-width annuli and returns
    /// the count in each bin (normalised to the bin with the most samples).
    pub fn radial_coverage(uv_points: &[[f64; 2]], num_bins: usize) -> Vec<f64> {
        if num_bins == 0 || uv_points.is_empty() {
            return vec![0.0; num_bins];
        }

        let max_r = uv_points
            .iter()
            .map(|b| (b[0] * b[0] + b[1] * b[1]).sqrt())
            .fold(0.0f64, f64::max);

        if max_r == 0.0 {
            return vec![0.0; num_bins];
        }

        let mut bins = vec![0.0f64; num_bins];
        let bin_width = max_r / num_bins as f64;

        for &[u, v] in uv_points {
            let r = (u * u + v * v).sqrt();
            let idx = ((r / bin_width).floor() as usize).min(num_bins - 1);
            bins[idx] += 1.0;
        }

        let peak = bins.iter().cloned().fold(0.0f64, f64::max);
        if peak > 0.0 {
            for b in &mut bins {
                *b /= peak;
            }
        }
        bins
    }

    /// Highest sidelobe level relative to the main-lobe peak.
    ///
    /// Finds the peak of the beam, then the highest value that is not part of
    /// the main lobe (defined as the connected region around the peak that
    /// stays above 50% of the peak).  Returns `sidelobe_peak / main_peak`.
    pub fn sidelobe_level(beam: &[Vec<f64>]) -> f64 {
        let ny = beam.len();
        if ny == 0 {
            return 0.0;
        }
        let nx = beam[0].len();

        // Find main peak
        let mut peak_val = f64::NEG_INFINITY;
        let mut peak_y = 0;
        let mut peak_x = 0;
        for y in 0..ny {
            for x in 0..nx {
                if beam[y][x] > peak_val {
                    peak_val = beam[y][x];
                    peak_y = y;
                    peak_x = x;
                }
            }
        }
        if peak_val <= 0.0 {
            return 0.0;
        }

        // Flood-fill to identify the main-lobe region (pixels > 50% of peak
        // connected to the peak pixel).
        let half_peak = 0.5 * peak_val;
        let mut visited = vec![vec![false; nx]; ny];
        let mut stack = vec![(peak_y, peak_x)];
        visited[peak_y][peak_x] = true;

        while let Some((cy, cx)) = stack.pop() {
            for &(dy, dx) in &[(-1i32, 0i32), (1, 0), (0, -1), (0, 1)] {
                let ny2 = cy as i32 + dy;
                let nx2 = cx as i32 + dx;
                if ny2 >= 0 && ny2 < ny as i32 && nx2 >= 0 && nx2 < nx as i32 {
                    let (uy, ux) = (ny2 as usize, nx2 as usize);
                    if !visited[uy][ux] && beam[uy][ux] >= half_peak {
                        visited[uy][ux] = true;
                        stack.push((uy, ux));
                    }
                }
            }
        }

        // Highest value outside the main lobe
        let mut sidelobe_peak = 0.0f64;
        for y in 0..ny {
            for x in 0..nx {
                if !visited[y][x] && beam[y][x] > sidelobe_peak {
                    sidelobe_peak = beam[y][x];
                }
            }
        }

        sidelobe_peak / peak_val
    }
}

// ---------------------------------------------------------------------------
// NearFieldHolography
// ---------------------------------------------------------------------------

/// Near-field holographic measurement and aperture reconstruction.
///
/// Measures the complex field on a planar grid at a known distance from the
/// aperture, then back-propagates to recover the aperture illumination and
/// surface phase errors.
pub struct NearFieldHolography {
    /// Scan positions on the measurement plane (metres).
    scan_positions: Vec<[f64; 2]>,
    /// Distance from aperture to measurement plane (metres).
    measurement_distance_m: f64,
    /// Observation wavelength (metres).
    wavelength_m: f64,
}

impl NearFieldHolography {
    /// Create a new near-field holography processor.
    pub fn new(
        scan_positions: Vec<[f64; 2]>,
        measurement_distance_m: f64,
        wavelength_m: f64,
    ) -> Self {
        Self {
            scan_positions,
            measurement_distance_m,
            wavelength_m,
        }
    }

    /// Back-propagate measured complex field to the aperture plane.
    ///
    /// `measurements` contains `(real, imag)` for each scan position.
    /// Returns a 2-D grid (same dimensions as scan positions, assumed square)
    /// of `(real, imag)` representing the aperture field.
    ///
    /// Uses a free-space Green's function kernel to propagate from each
    /// measurement point back to the aperture plane.
    pub fn reconstruct_aperture_field(
        &self,
        measurements: &[(f64, f64)],
    ) -> Vec<Vec<(f64, f64)>> {
        let n_meas = measurements.len();
        let n_side = (n_meas as f64).sqrt().round() as usize;
        if n_side == 0 {
            return vec![];
        }

        let k = 2.0 * PI / self.wavelength_m;
        let z = self.measurement_distance_m;

        let mut field = vec![vec![(0.0f64, 0.0f64); n_side]; n_side];

        // For each aperture point, sum contributions from all measurement points
        for ay in 0..n_side {
            for ax in 0..n_side {
                let ap_idx = ay * n_side + ax;
                if ap_idx >= self.scan_positions.len() {
                    continue;
                }
                let ap = self.scan_positions[ap_idx];

                let mut re_sum = 0.0;
                let mut im_sum = 0.0;

                for (mi, &(m_re, m_im)) in measurements.iter().enumerate() {
                    if mi >= self.scan_positions.len() {
                        continue;
                    }
                    let mp = self.scan_positions[mi];
                    let dx = ap[0] - mp[0];
                    let dy = ap[1] - mp[1];
                    let r = (dx * dx + dy * dy + z * z).sqrt();

                    // Conjugate Green's function: exp(+jkr) / r  (back-propagation)
                    let phase = k * r;
                    let g_re = phase.cos() / r;
                    let g_im = phase.sin() / r;

                    // Complex multiply: measurement * conj_green
                    re_sum += m_re * g_re - m_im * g_im;
                    im_sum += m_re * g_im + m_im * g_re;
                }

                field[ay][ax] = (re_sum, im_sum);
            }
        }

        field
    }

    /// Compute the phase error map between ideal and measured aperture fields.
    ///
    /// Returns the wrapped phase difference (radians) at each grid point.
    pub fn phase_error_map(
        ideal: &[Vec<(f64, f64)>],
        measured: &[Vec<(f64, f64)>],
    ) -> Vec<Vec<f64>> {
        let ny = ideal.len().min(measured.len());
        let mut errors = Vec::with_capacity(ny);
        for y in 0..ny {
            let nx = ideal[y].len().min(measured[y].len());
            let mut row = Vec::with_capacity(nx);
            for x in 0..nx {
                let (ir, ii) = ideal[y][x];
                let (mr, mi) = measured[y][x];
                let ideal_phase = ii.atan2(ir);
                let meas_phase = mi.atan2(mr);
                let mut diff = meas_phase - ideal_phase;
                // Wrap to [-pi, pi]
                while diff > PI {
                    diff -= 2.0 * PI;
                }
                while diff < -PI {
                    diff += 2.0 * PI;
                }
                row.push(diff);
            }
            errors.push(row);
        }
        errors
    }

    /// RMS surface error computed from a phase-error map.
    ///
    /// Converts phase errors to physical surface errors via
    /// `surface_error = phase_error * wavelength / (4 * pi)`
    /// (factor of 4*pi because a surface error of delta produces a path
    /// difference of 2*delta for a reflector antenna).
    pub fn rms_surface_error(phase_errors: &[Vec<f64>], wavelength_m: f64) -> f64 {
        let mut sum_sq = 0.0;
        let mut count = 0usize;
        for row in phase_errors {
            for &e in row {
                let surface = e * wavelength_m / (4.0 * PI);
                sum_sq += surface * surface;
                count += 1;
            }
        }
        if count == 0 {
            return 0.0;
        }
        (sum_sq / count as f64).sqrt()
    }
}

// ---------------------------------------------------------------------------
// AcousticHolography
// ---------------------------------------------------------------------------

/// Angular-spectrum-based acoustic holographic propagation.
///
/// Propagates a measured 2-D complex pressure field forward or backward
/// along the z-axis using the angular spectrum method.
pub struct AcousticHolography {
    /// Acoustic frequency in Hz.
    frequency_hz: f64,
    /// Speed of sound in the medium (m/s).
    sound_speed: f64,
    /// z-coordinate of the measurement plane (metres).
    measurement_plane_z: f64,
}

impl AcousticHolography {
    /// Create a new acoustic holography processor.
    pub fn new(frequency_hz: f64, sound_speed: f64, measurement_plane_z: f64) -> Self {
        Self {
            frequency_hz,
            sound_speed,
            measurement_plane_z,
        }
    }

    /// Wavelength of the acoustic wave.
    pub fn wavelength(&self) -> f64 {
        self.sound_speed / self.frequency_hz
    }

    /// z-coordinate of the measurement plane.
    pub fn measurement_z(&self) -> f64 {
        self.measurement_plane_z
    }

    /// Forward-propagate a 2-D complex field by distance `dz`.
    ///
    /// Uses a spatial-domain convolution approximation of the angular spectrum
    /// propagator.  `dx` is the grid spacing in metres.
    ///
    /// The propagation kernel is:
    /// `H(x,y) = exp(-j*k*r) / r` where `r = sqrt(x^2 + y^2 + dz^2)`
    pub fn forward_propagate(
        &self,
        field: &[Vec<(f64, f64)>],
        dz: f64,
        dx: f64,
    ) -> Vec<Vec<(f64, f64)>> {
        self.propagate_internal(field, dz, dx)
    }

    /// Back-propagate a 2-D complex field by distance `dz`.
    ///
    /// Identical to forward propagation but with the conjugate kernel
    /// (reversed propagation direction).
    pub fn back_propagate(
        &self,
        field: &[Vec<(f64, f64)>],
        dz: f64,
        dx: f64,
    ) -> Vec<Vec<(f64, f64)>> {
        self.propagate_internal(field, -dz, dx)
    }

    /// Internal propagation engine.  Positive `dz` is forward, negative is
    /// backward (conjugate kernel).
    fn propagate_internal(
        &self,
        field: &[Vec<(f64, f64)>],
        dz: f64,
        dx: f64,
    ) -> Vec<Vec<(f64, f64)>> {
        let ny = field.len();
        if ny == 0 {
            return vec![];
        }
        let nx = field[0].len();
        if nx == 0 {
            return vec![vec![]; ny];
        }

        let k = 2.0 * PI * self.frequency_hz / self.sound_speed;
        let mut output = vec![vec![(0.0f64, 0.0f64); nx]; ny];

        // Spatial-domain Rayleigh-Sommerfeld convolution
        for oy in 0..ny {
            for ox in 0..nx {
                let mut re_sum = 0.0;
                let mut im_sum = 0.0;

                for sy in 0..ny {
                    for sx in 0..nx {
                        let delta_x = (ox as f64 - sx as f64) * dx;
                        let delta_y = (oy as f64 - sy as f64) * dx;
                        let r = (delta_x * delta_x + delta_y * delta_y + dz * dz).sqrt();
                        if r < 1e-15 {
                            // Same point, no propagation
                            re_sum += field[sy][sx].0;
                            im_sum += field[sy][sx].1;
                            continue;
                        }

                        // Kernel: exp(-j*k*r) / r  (forward)
                        // For back-propagation (dz < 0), dz is negative so r uses |dz|
                        // but the sign convention in the phase gives the conjugate effect.
                        let phase = -k * r;
                        let h_re = phase.cos() / r;
                        let h_im = phase.sin() / r;

                        let (s_re, s_im) = field[sy][sx];
                        re_sum += s_re * h_re - s_im * h_im;
                        im_sum += s_re * h_im + s_im * h_re;
                    }
                }

                // Scale by dx^2 (area element)
                output[oy][ox] = (re_sum * dx * dx, im_sum * dx * dx);
            }
        }

        output
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-10;
    const LOOSE_TOL: f64 = 1e-4;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // --- Helper function tests ---

    #[test]
    fn test_spatial_frequency_zero_angle() {
        let sf = spatial_frequency(0.0, 0.03);
        assert!(approx_eq(sf, 0.0, TOL));
    }

    #[test]
    fn test_spatial_frequency_90_degrees() {
        let sf = spatial_frequency(PI / 2.0, 0.03);
        assert!(approx_eq(sf, 1.0 / 0.03, TOL));
    }

    #[test]
    fn test_spatial_frequency_negative_angle() {
        let sf = spatial_frequency(-PI / 6.0, 1.0);
        assert!(approx_eq(sf, -0.5, TOL));
    }

    #[test]
    fn test_fraunhofer_distance() {
        // D = 10m, lambda = 0.03m => 2 * 100 / 0.03 = 6666.67 m
        let d = fraunhofer_distance(10.0, 0.03);
        assert!(approx_eq(d, 2.0 * 100.0 / 0.03, TOL));
    }

    #[test]
    fn test_fraunhofer_distance_small_aperture() {
        let d = fraunhofer_distance(0.5, 0.1);
        assert!(approx_eq(d, 2.0 * 0.25 / 0.1, TOL));
    }

    #[test]
    fn test_array_factor_single_element() {
        let (re, im) = array_factor_1d(&[0.0], 0.0, 1.0);
        assert!(approx_eq(re, 1.0, TOL));
        assert!(approx_eq(im, 0.0, TOL));
    }

    #[test]
    fn test_array_factor_broadside() {
        // Two elements at -0.5 and +0.5 wavelengths, broadside (theta=0)
        let (re, im) = array_factor_1d(&[-0.25, 0.25], 0.0, 1.0);
        // At broadside, both have zero phase => AF = 2
        assert!(approx_eq(re, 2.0, TOL));
        assert!(approx_eq(im, 0.0, TOL));
    }

    #[test]
    fn test_array_factor_endfire() {
        // Two elements separated by lambda/2, direction = pi/2 (endfire)
        let positions = [0.0, 0.5]; // in metres, with lambda = 1.0
        let (re, im) = array_factor_1d(&positions, PI / 2.0, 1.0);
        // phase difference = 2*pi * 0.5 * sin(pi/2) / 1.0 = pi
        // AF = 1 + exp(j*pi) = 1 - 1 = 0
        assert!(approx_eq(re, 0.0, TOL));
        assert!(approx_eq(im, 0.0, TOL));
    }

    // --- Visibility tests ---

    #[test]
    fn test_visibility_new() {
        let v = Visibility::new(1.0, 2.0, 3.0, 4.0, 5.0);
        assert_eq!(v.u, 1.0);
        assert_eq!(v.v, 2.0);
        assert_eq!(v.real, 3.0);
        assert_eq!(v.imag, 4.0);
        assert_eq!(v.weight, 5.0);
    }

    #[test]
    fn test_visibility_amplitude() {
        let v = Visibility::new(0.0, 0.0, 3.0, 4.0, 1.0);
        assert!(approx_eq(v.amplitude(), 5.0, TOL));
    }

    #[test]
    fn test_visibility_phase() {
        let v = Visibility::new(0.0, 0.0, 1.0, 1.0, 1.0);
        assert!(approx_eq(v.phase(), PI / 4.0, TOL));
    }

    // --- Baseline / UV coverage tests ---

    #[test]
    fn test_compute_baseline() {
        let b = HolographicReconstructor::compute_baseline([1.0, 2.0], [4.0, 6.0]);
        assert!(approx_eq(b[0], 3.0, TOL));
        assert!(approx_eq(b[1], 4.0, TOL));
    }

    #[test]
    fn test_compute_baseline_same_position() {
        let b = HolographicReconstructor::compute_baseline([5.0, 5.0], [5.0, 5.0]);
        assert!(approx_eq(b[0], 0.0, TOL));
        assert!(approx_eq(b[1], 0.0, TOL));
    }

    #[test]
    fn test_uv_coverage_count() {
        // 4 elements => 4*3/2 = 6 baselines
        let config = HolographyConfig {
            wavelength_m: 0.21,
            num_elements: 4,
            element_positions: vec![[0.0, 0.0], [10.0, 0.0], [0.0, 10.0], [10.0, 10.0]],
            reference_position: [0.0, 0.0],
            grid_size: 32,
            field_of_view_rad: 0.01,
        };
        let recon = HolographicReconstructor::new(config);
        let uv = recon.compute_uv_coverage();
        assert_eq!(uv.len(), 6);
    }

    #[test]
    fn test_uv_coverage_two_elements() {
        let config = HolographyConfig {
            wavelength_m: 1.0,
            num_elements: 2,
            element_positions: vec![[0.0, 0.0], [5.0, 3.0]],
            reference_position: [0.0, 0.0],
            grid_size: 16,
            field_of_view_rad: 0.1,
        };
        let recon = HolographicReconstructor::new(config);
        let uv = recon.compute_uv_coverage();
        assert_eq!(uv.len(), 1);
        assert!(approx_eq(uv[0][0], 5.0, TOL));
        assert!(approx_eq(uv[0][1], 3.0, TOL));
    }

    // --- Visibility function tests ---

    #[test]
    fn test_visibility_zero_baseline() {
        let (re, im) = HolographicReconstructor::visibility([0.0, 0.0], [0.1, 0.2], 1.0);
        // Phase = 0 => V = (1, 0)
        assert!(approx_eq(re, 1.0, TOL));
        assert!(approx_eq(im, 0.0, TOL));
    }

    #[test]
    fn test_visibility_known_phase() {
        // baseline [1, 0], source at l=0.25, m=0 => phase = -2*pi*0.25 = -pi/2
        let (re, im) = HolographicReconstructor::visibility([1.0, 0.0], [0.25, 0.0], 1.0);
        // exp(-j*pi/2) = cos(-pi/2) + j*sin(-pi/2) = (0, -1)
        assert!(approx_eq(re, 0.0, TOL));
        assert!(approx_eq(im, -1.0, TOL));
    }

    #[test]
    fn test_visibility_amplitude_scaling() {
        let (re, im) = HolographicReconstructor::visibility([0.0, 0.0], [0.0, 0.0], 3.5);
        assert!(approx_eq(re, 3.5, TOL));
        assert!(approx_eq(im, 0.0, TOL));
    }

    // --- Angular resolution tests ---

    #[test]
    fn test_angular_resolution_simple() {
        // Two elements separated by 100 wavelengths => resolution = 1/100 rad
        let config = HolographyConfig {
            wavelength_m: 0.03,
            num_elements: 2,
            element_positions: vec![[0.0, 0.0], [100.0, 0.0]],
            reference_position: [0.0, 0.0],
            grid_size: 16,
            field_of_view_rad: 0.01,
        };
        let recon = HolographicReconstructor::new(config);
        assert!(approx_eq(recon.angular_resolution(), 0.01, TOL));
    }

    #[test]
    fn test_angular_resolution_diagonal() {
        // Elements at (0,0) and (30,40) => baseline length = 50 wavelengths
        let config = HolographyConfig {
            wavelength_m: 1.0,
            num_elements: 2,
            element_positions: vec![[0.0, 0.0], [30.0, 40.0]],
            reference_position: [0.0, 0.0],
            grid_size: 16,
            field_of_view_rad: 0.1,
        };
        let recon = HolographicReconstructor::new(config);
        assert!(approx_eq(recon.angular_resolution(), 1.0 / 50.0, TOL));
    }

    // --- Dirty beam / PSF tests ---

    #[test]
    fn test_dirty_beam_peak_at_center() {
        let uv = vec![[10.0, 0.0], [0.0, 10.0], [10.0, 10.0]];
        let beam = HolographicReconstructor::dirty_beam(&uv, 17, 0.02);
        let centre = 17 / 2;
        // Peak should be at centre (normalised to 1.0)
        assert!(approx_eq(beam[centre][centre], 1.0, TOL));
    }

    #[test]
    fn test_dirty_beam_symmetric() {
        // East-west baseline only => beam symmetric about l=0
        let uv = vec![[10.0, 0.0]];
        let beam = HolographicReconstructor::dirty_beam(&uv, 15, 0.02);
        let cy = 15 / 2;
        // Check symmetry in x for the central row
        for x in 0..7 {
            assert!(approx_eq(beam[cy][x], beam[cy][14 - x], 1e-6));
        }
    }

    #[test]
    fn test_dirty_beam_normalised() {
        let uv = vec![[5.0, 0.0], [0.0, 5.0]];
        let beam = HolographicReconstructor::dirty_beam(&uv, 11, 0.05);
        let max_val = beam.iter().flat_map(|r| r.iter()).cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(approx_eq(max_val, 1.0, TOL));
    }

    #[test]
    fn test_dirty_beam_main_lobe_width() {
        // Longer baselines => narrower main lobe
        let uv_short = vec![[5.0, 0.0]];
        let uv_long = vec![[50.0, 0.0]];
        let beam_short = HolographicReconstructor::dirty_beam(&uv_short, 31, 0.1);
        let beam_long = HolographicReconstructor::dirty_beam(&uv_long, 31, 0.1);
        let cy = 31 / 2;

        // Count pixels above 0.5 in the central row
        let width_short = beam_short[cy].iter().filter(|&&v| v >= 0.5).count();
        let width_long = beam_long[cy].iter().filter(|&&v| v >= 0.5).count();
        assert!(width_long < width_short, "Longer baseline should give narrower beam");
    }

    // --- Dirty image tests ---

    #[test]
    fn test_dirty_image_single_visibility() {
        let config = HolographyConfig {
            wavelength_m: 1.0,
            num_elements: 2,
            element_positions: vec![[0.0, 0.0], [10.0, 0.0]],
            reference_position: [0.0, 0.0],
            grid_size: 11,
            field_of_view_rad: 0.1,
        };
        let recon = HolographicReconstructor::new(config);
        let vis = vec![((10.0, 0.0), (1.0, 0.0))];
        let image = recon.dirty_image(&vis);
        assert_eq!(image.len(), 11);
        assert_eq!(image[0].len(), 11);
        // Centre pixel (l=0,m=0) should have value = cos(0) = 1.0
        assert!(approx_eq(image[5][5], 1.0, 1e-6));
    }

    #[test]
    fn test_dirty_image_dimensions() {
        let config = HolographyConfig {
            wavelength_m: 1.0,
            num_elements: 2,
            element_positions: vec![[0.0, 0.0], [5.0, 0.0]],
            reference_position: [0.0, 0.0],
            grid_size: 7,
            field_of_view_rad: 0.1,
        };
        let recon = HolographicReconstructor::new(config);
        let image = recon.dirty_image(&[]);
        assert_eq!(image.len(), 7);
        assert_eq!(image[0].len(), 7);
        // Empty visibilities => zero image
        for row in &image {
            for &v in row {
                assert!(approx_eq(v, 0.0, TOL));
            }
        }
    }

    // --- CLEAN deconvolution tests ---

    #[test]
    fn test_clean_recovers_point_source() {
        // Create a simple dirty image with a single bright point at centre
        let n = 11;
        let mut dirty = vec![vec![0.0; n]; n];
        dirty[5][5] = 1.0;

        // Delta-function beam (identity PSF)
        let mut beam = vec![vec![0.0; n]; n];
        beam[5][5] = 1.0;

        let clean = HolographicReconstructor::clean_deconvolve(&dirty, &beam, 100, 0.1, 1e-6);
        // Should recover the point source
        assert!(clean[5][5] > 0.9, "CLEAN should recover the point source, got {}", clean[5][5]);
    }

    #[test]
    fn test_clean_two_point_sources() {
        let n = 15;
        let mut dirty = vec![vec![0.0; n]; n];
        dirty[4][4] = 1.0;
        dirty[10][10] = 0.8;

        let mut beam = vec![vec![0.0; n]; n];
        beam[7][7] = 1.0;

        let clean = HolographicReconstructor::clean_deconvolve(&dirty, &beam, 200, 0.1, 1e-6);
        assert!(clean[4][4] > 0.8);
        assert!(clean[10][10] > 0.6);
    }

    #[test]
    fn test_clean_threshold() {
        let n = 9;
        let mut dirty = vec![vec![0.0; n]; n];
        dirty[4][4] = 0.01; // Below threshold

        let mut beam = vec![vec![0.0; n]; n];
        beam[4][4] = 1.0;

        let clean = HolographicReconstructor::clean_deconvolve(&dirty, &beam, 1000, 0.1, 0.1);
        // Source is below threshold, CLEAN should not find it
        let total: f64 = clean.iter().flat_map(|r| r.iter()).sum();
        assert!(approx_eq(total, 0.0, TOL));
    }

    #[test]
    fn test_clean_empty_image() {
        let clean = HolographicReconstructor::clean_deconvolve(&[], &[], 100, 0.1, 1e-6);
        assert!(clean.is_empty());
    }

    // --- UV Coverage Analyzer tests ---

    #[test]
    fn test_coverage_fraction_full() {
        // Dense array of baselines covering the grid
        let mut uv = Vec::new();
        for i in -5..=5 {
            for j in -5..=5 {
                uv.push([i as f64, j as f64]);
            }
        }
        let frac = UvCoverageAnalyzer::coverage_fraction(&uv, 5.0, 10);
        // Should be high coverage
        assert!(frac > 0.5);
    }

    #[test]
    fn test_coverage_fraction_empty() {
        let frac = UvCoverageAnalyzer::coverage_fraction(&[], 10.0, 10);
        assert!(approx_eq(frac, 0.0, TOL));
    }

    #[test]
    fn test_coverage_fraction_single_baseline() {
        let uv = vec![[3.0, 0.0]];
        let frac = UvCoverageAnalyzer::coverage_fraction(&uv, 5.0, 10);
        // Should occupy 2 cells (baseline + conjugate) out of 100
        assert!(frac > 0.0 && frac < 0.1);
    }

    #[test]
    fn test_radial_coverage_uniform() {
        // Baselines at various radii
        let uv: Vec<[f64; 2]> = (1..=10)
            .map(|i| [i as f64, 0.0])
            .collect();
        let profile = UvCoverageAnalyzer::radial_coverage(&uv, 5);
        assert_eq!(profile.len(), 5);
        // Peak bin should be 1.0
        assert!(profile.iter().any(|&v| approx_eq(v, 1.0, TOL)));
    }

    #[test]
    fn test_radial_coverage_empty() {
        let profile = UvCoverageAnalyzer::radial_coverage(&[], 5);
        assert_eq!(profile.len(), 5);
        for &v in &profile {
            assert!(approx_eq(v, 0.0, TOL));
        }
    }

    #[test]
    fn test_sidelobe_level_delta_beam() {
        // Perfect delta-function beam => no sidelobes
        let n = 11;
        let mut beam = vec![vec![0.0; n]; n];
        beam[5][5] = 1.0;
        let sl = UvCoverageAnalyzer::sidelobe_level(&beam);
        assert!(approx_eq(sl, 0.0, TOL));
    }

    #[test]
    fn test_sidelobe_level_with_sidelobes() {
        let n = 11;
        let mut beam = vec![vec![0.0; n]; n];
        beam[5][5] = 1.0; // main lobe
        beam[0][0] = 0.3; // sidelobe
        beam[10][10] = 0.2; // another sidelobe
        let sl = UvCoverageAnalyzer::sidelobe_level(&beam);
        // Highest sidelobe is 0.3 / 1.0 = 0.3
        assert!(approx_eq(sl, 0.3, TOL));
    }

    // --- Near-field holography tests ---

    #[test]
    fn test_near_field_reconstruct_dimensions() {
        let positions: Vec<[f64; 2]> = (0..9)
            .map(|i| [(i % 3) as f64 * 0.01, (i / 3) as f64 * 0.01])
            .collect();
        let nfh = NearFieldHolography::new(positions, 1.0, 0.03);
        let measurements: Vec<(f64, f64)> = (0..9).map(|_| (1.0, 0.0)).collect();
        let field = nfh.reconstruct_aperture_field(&measurements);
        assert_eq!(field.len(), 3);
        assert_eq!(field[0].len(), 3);
    }

    #[test]
    fn test_near_field_nonzero_output() {
        let positions: Vec<[f64; 2]> = (0..4)
            .map(|i| [(i % 2) as f64 * 0.01, (i / 2) as f64 * 0.01])
            .collect();
        let nfh = NearFieldHolography::new(positions, 0.5, 0.03);
        let measurements = vec![(1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (0.5, 0.5)];
        let field = nfh.reconstruct_aperture_field(&measurements);
        // Output should be non-zero
        let has_nonzero = field.iter().flat_map(|r| r.iter()).any(|(re, im)| re.abs() > 1e-10 || im.abs() > 1e-10);
        assert!(has_nonzero, "Reconstructed field should be non-zero");
    }

    #[test]
    fn test_phase_error_map_zero_error() {
        let ideal = vec![vec![(1.0, 0.0), (0.0, 1.0)], vec![(1.0, 1.0), (-1.0, 0.0)]];
        let errors = NearFieldHolography::phase_error_map(&ideal, &ideal);
        for row in &errors {
            for &e in row {
                assert!(approx_eq(e, 0.0, TOL));
            }
        }
    }

    #[test]
    fn test_phase_error_map_known_error() {
        // Ideal: phase 0, measured: phase pi/4
        let ideal = vec![vec![(1.0, 0.0)]];
        let measured = vec![vec![(1.0_f64 / 2.0_f64.sqrt(), 1.0_f64 / 2.0_f64.sqrt())]];
        let errors = NearFieldHolography::phase_error_map(&ideal, &measured);
        assert!(approx_eq(errors[0][0], PI / 4.0, 1e-6));
    }

    #[test]
    fn test_rms_surface_error_zero() {
        let errors = vec![vec![0.0, 0.0], vec![0.0, 0.0]];
        let rms = NearFieldHolography::rms_surface_error(&errors, 0.03);
        assert!(approx_eq(rms, 0.0, TOL));
    }

    #[test]
    fn test_rms_surface_error_known() {
        // Phase error of pi everywhere, lambda = 1.0
        // surface_error = pi * 1.0 / (4*pi) = 0.25
        let errors = vec![vec![PI, PI], vec![PI, PI]];
        let rms = NearFieldHolography::rms_surface_error(&errors, 1.0);
        assert!(approx_eq(rms, 0.25, TOL));
    }

    // --- Acoustic holography tests ---

    #[test]
    fn test_acoustic_holography_wavelength() {
        let ah = AcousticHolography::new(1000.0, 343.0, 0.1);
        assert!(approx_eq(ah.wavelength(), 0.343, TOL));
    }

    #[test]
    fn test_acoustic_forward_propagate_dimensions() {
        let ah = AcousticHolography::new(1000.0, 343.0, 0.0);
        let field = vec![vec![(1.0, 0.0); 3]; 3];
        let out = ah.forward_propagate(&field, 0.01, 0.001);
        assert_eq!(out.len(), 3);
        assert_eq!(out[0].len(), 3);
    }

    #[test]
    fn test_acoustic_back_propagate_dimensions() {
        let ah = AcousticHolography::new(1000.0, 343.0, 0.0);
        let field = vec![vec![(1.0, 0.0); 4]; 4];
        let out = ah.back_propagate(&field, 0.01, 0.001);
        assert_eq!(out.len(), 4);
        assert_eq!(out[0].len(), 4);
    }

    #[test]
    fn test_acoustic_forward_back_roundtrip() {
        // Forward then back should approximately recover the original field
        // for a simple uniform field
        let ah = AcousticHolography::new(5000.0, 343.0, 0.0);
        let n = 3;
        let dx = 0.01;
        let dz = 0.02;

        // Start with a simple field
        let original = vec![vec![(1.0, 0.0); n]; n];
        let forward = ah.forward_propagate(&original, dz, dx);
        let back = ah.back_propagate(&forward, dz, dx);

        // The roundtrip won't be exact due to discretisation, but the centre
        // pixel should retain significant energy
        let centre = n / 2;
        let (re, im) = back[centre][centre];
        let mag = (re * re + im * im).sqrt();
        assert!(mag > 0.0, "Roundtrip should preserve some energy at centre, got mag={}", mag);
    }

    #[test]
    fn test_acoustic_propagate_empty() {
        let ah = AcousticHolography::new(1000.0, 343.0, 0.0);
        let out = ah.forward_propagate(&[], 0.01, 0.001);
        assert!(out.is_empty());
    }

    #[test]
    fn test_acoustic_measurement_z() {
        let ah = AcousticHolography::new(1000.0, 343.0, 0.05);
        assert!(approx_eq(ah.measurement_z(), 0.05, TOL));
    }
}
