//! # Transmission Electron Microscopy (TEM) Processor
//!
//! Implements image and diffraction pattern processing for Transmission Electron
//! Microscopy (TEM) in materials characterization. TEM uses a beam of electrons
//! transmitted through a thin specimen to form images and diffraction patterns,
//! revealing crystal structure, defects, and composition at atomic resolution.
//!
//! ## Physical Background
//!
//! Electrons accelerated through voltage V acquire wavelength (de Broglie):
//!
//! ```text
//! λ = h / sqrt(2 m_e eV (1 + eV / 2m_e c²))
//! ```
//!
//! Bragg's law for diffraction from crystal planes with spacing d:
//!
//! ```text
//! 2d sin(θ) = nλ    →    (for small angles) d = λL / r
//! ```
//!
//! where L is the camera length and r is the spot radius on the detector.
//!
//! The Contrast Transfer Function (CTF) models the phase contrast transfer:
//!
//! ```text
//! CTF(u) = -2 sin(χ(u)) · E(u)
//! χ(u)  = π λ u² (Δf - ½ Cs λ² u²)
//! ```
//!
//! where Δf is defocus, Cs is spherical aberration, and E(u) is the envelope.
//!
//! ## Key Components
//!
//! - [`ElectronBeam`] - Accelerating voltage to wavelength conversion
//! - [`CameraCalibration`] - Camera length calibration and d-spacing calculation
//! - [`DiffractionPattern`] - Spot/ring detection and d-spacing measurement
//! - [`ZoneAxisIdentifier`] - Zone axis determination from diffraction spots
//! - [`LatticeFringeAnalyzer`] - FFT-based HRTEM lattice fringe analysis
//! - [`ContrastTransferFunction`] - CTF model with Thon ring analysis
//! - [`SaedAnalyzer`] - Selected Area Electron Diffraction ring indexing
//! - [`KikuchiAnalyzer`] - Kikuchi band detection for crystal orientation
//! - [`ThicknessMeasurement`] - Log-ratio EELS thickness measurement
//! - [`ImageFilter`] - Wiener and bandpass filters for TEM images
//! - [`MoireAnalyzer`] - Moiré pattern analysis for overlapping lattices
//! - [`MaterialDatabase`] - Crystal structure database (FCC, BCC, HCP)

use std::f64::consts::PI;

// ─── Physical Constants ───────────────────────────────────────────────────────

/// Planck's constant (J·s).
pub const PLANCK_H: f64 = 6.62607015e-34;

/// Electron rest mass (kg).
pub const ELECTRON_MASS: f64 = 9.1093837015e-31;

/// Electron charge (C).
pub const ELECTRON_CHARGE: f64 = 1.602176634e-19;

/// Speed of light (m/s).
pub const SPEED_OF_LIGHT: f64 = 2.99792458e8;

/// Bohr radius (m).
pub const BOHR_RADIUS: f64 = 5.29177210903e-11;

// ─── ElectronBeam ────────────────────────────────────────────────────────────

/// Electron beam parameters derived from accelerating voltage.
///
/// Accounts for relativistic correction at high voltages (>100 kV).
///
/// # Example
/// ```
/// use r4w_core::transmission_electron_microscopy_processor::ElectronBeam;
/// let beam = ElectronBeam::new(200e3); // 200 kV
/// assert!((beam.wavelength_pm() - 2.508).abs() < 0.01);
/// ```
#[derive(Debug, Clone)]
pub struct ElectronBeam {
    /// Accelerating voltage in volts.
    pub voltage_v: f64,
    /// Relativistic electron wavelength in meters.
    pub wavelength_m: f64,
    /// Relativistic factor gamma.
    pub gamma: f64,
}

impl ElectronBeam {
    /// Create a new electron beam from accelerating voltage.
    ///
    /// Uses the relativistic de Broglie wavelength:
    /// λ = h / sqrt(2 m_e eV (1 + eV/2m_ec²))
    pub fn new(voltage_v: f64) -> Self {
        let ev = ELECTRON_CHARGE * voltage_v;
        let rest_energy = ELECTRON_MASS * SPEED_OF_LIGHT * SPEED_OF_LIGHT;
        let numerator = PLANCK_H;
        let denominator = (2.0 * ELECTRON_MASS * ev * (1.0 + ev / (2.0 * rest_energy))).sqrt();
        let wavelength_m = numerator / denominator;
        let gamma = 1.0 + ev / rest_energy;
        Self { voltage_v, wavelength_m, gamma }
    }

    /// Return wavelength in picometers (pm).
    pub fn wavelength_pm(&self) -> f64 {
        self.wavelength_m * 1e12
    }

    /// Return wavelength in angstroms (Å).
    pub fn wavelength_angstrom(&self) -> f64 {
        self.wavelength_m * 1e10
    }

    /// Relativistic electron velocity as fraction of speed of light (v/c).
    pub fn beta(&self) -> f64 {
        (1.0 - 1.0 / (self.gamma * self.gamma)).sqrt()
    }
}

// ─── CameraCalibration ───────────────────────────────────────────────────────

/// Camera length calibration for electron diffraction.
///
/// Relates spot distance on detector to real-space d-spacing via:
/// d = λL / r   (small-angle approximation)
#[derive(Debug, Clone)]
pub struct CameraCalibration {
    /// Effective camera length L in meters.
    pub camera_length_m: f64,
    /// Electron wavelength in meters.
    pub wavelength_m: f64,
    /// Camera constant λL in m².
    pub camera_constant: f64,
    /// Pixel size in meters (if digital detector).
    pub pixel_size_m: f64,
}

impl CameraCalibration {
    /// Create calibration from camera length and beam parameters.
    pub fn new(camera_length_m: f64, beam: &ElectronBeam, pixel_size_m: f64) -> Self {
        let camera_constant = beam.wavelength_m * camera_length_m;
        Self {
            camera_length_m,
            wavelength_m: beam.wavelength_m,
            camera_constant,
            pixel_size_m,
        }
    }

    /// Calculate d-spacing (in meters) from spot radius (in meters on detector).
    pub fn d_spacing_from_radius(&self, radius_m: f64) -> f64 {
        self.camera_constant / radius_m
    }

    /// Calculate d-spacing (in Å) from spot radius in pixels.
    pub fn d_spacing_angstrom_from_pixels(&self, radius_pixels: f64) -> f64 {
        let radius_m = radius_pixels * self.pixel_size_m;
        self.d_spacing_from_radius(radius_m) * 1e10
    }

    /// Calculate spot radius (pixels) from d-spacing in Å.
    pub fn radius_pixels_from_d_spacing(&self, d_angstrom: f64) -> f64 {
        let d_m = d_angstrom * 1e-10;
        let radius_m = self.camera_constant / d_m;
        radius_m / self.pixel_size_m
    }

    /// Calibrate camera length from a known d-spacing and measured radius.
    pub fn calibrate_from_known(
        wavelength_m: f64,
        known_d_m: f64,
        measured_radius_m: f64,
        pixel_size_m: f64,
    ) -> Self {
        let camera_constant = known_d_m * measured_radius_m;
        let camera_length_m = camera_constant / wavelength_m;
        Self {
            camera_length_m,
            wavelength_m,
            camera_constant,
            pixel_size_m,
        }
    }
}

// ─── DiffractionSpot ─────────────────────────────────────────────────────────

/// A detected diffraction spot or ring.
#[derive(Debug, Clone)]
pub struct DiffractionSpot {
    /// X position (pixels) relative to direct beam center.
    pub x: f64,
    /// Y position (pixels) relative to direct beam center.
    pub y: f64,
    /// Spot intensity (arbitrary units).
    pub intensity: f64,
    /// Calculated d-spacing in Å.
    pub d_spacing_angstrom: f64,
    /// Miller indices (h, k, l) if indexed.
    pub hkl: Option<(i32, i32, i32)>,
}

impl DiffractionSpot {
    /// Create a spot from position and calibration.
    pub fn new(x: f64, y: f64, intensity: f64, cal: &CameraCalibration) -> Self {
        let radius = (x * x + y * y).sqrt();
        let d = if radius > 0.0 {
            cal.d_spacing_angstrom_from_pixels(radius)
        } else {
            f64::INFINITY
        };
        Self { x, y, intensity, d_spacing_angstrom: d, hkl: None }
    }

    /// Radius from direct beam center in pixels.
    pub fn radius(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    /// Azimuthal angle in radians.
    pub fn azimuth(&self) -> f64 {
        self.y.atan2(self.x)
    }

    /// Angle between this spot and another (in radians).
    pub fn angle_to(&self, other: &DiffractionSpot) -> f64 {
        let dot = self.x * other.x + self.y * other.y;
        let mag = self.radius() * other.radius();
        if mag < 1e-12 {
            return 0.0;
        }
        let cos_theta = (dot / mag).clamp(-1.0, 1.0);
        cos_theta.acos()
    }
}

// ─── DiffractionPattern ──────────────────────────────────────────────────────

/// Electron diffraction pattern with spot detection and analysis.
#[derive(Debug, Clone)]
pub struct DiffractionPattern {
    /// Detected spots in the pattern.
    pub spots: Vec<DiffractionSpot>,
    /// Camera calibration used.
    pub calibration: CameraCalibration,
    /// Direct beam center (x, y) in pixels.
    pub center: (f64, f64),
}

impl DiffractionPattern {
    /// Create a new empty diffraction pattern.
    pub fn new(calibration: CameraCalibration, center: (f64, f64)) -> Self {
        Self { spots: Vec::new(), calibration, center }
    }

    /// Add a spot by absolute pixel position.
    pub fn add_spot(&mut self, abs_x: f64, abs_y: f64, intensity: f64) {
        let x = abs_x - self.center.0;
        let y = abs_y - self.center.1;
        let spot = DiffractionSpot::new(x, y, intensity, &self.calibration);
        self.spots.push(spot);
    }

    /// Sort spots by distance from center (ascending radius).
    pub fn sort_by_radius(&mut self) {
        self.spots.sort_by(|a, b| a.radius().partial_cmp(&b.radius()).unwrap());
    }

    /// Find rings in the pattern by clustering spots at similar radii.
    /// Returns list of (d-spacing Å, count) for each ring.
    pub fn find_rings(&self, tolerance_percent: f64) -> Vec<(f64, usize)> {
        let mut sorted: Vec<f64> = self.spots.iter().map(|s| s.d_spacing_angstrom).collect();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let mut rings: Vec<(f64, usize)> = Vec::new();
        let mut i = 0;
        while i < sorted.len() {
            let d_ref = sorted[i];
            let mut sum = d_ref;
            let mut count = 1usize;
            let mut j = i + 1;
            while j < sorted.len() {
                let diff_pct = (sorted[j] - d_ref).abs() / d_ref * 100.0;
                if diff_pct <= tolerance_percent {
                    sum += sorted[j];
                    count += 1;
                    j += 1;
                } else {
                    break;
                }
            }
            rings.push((sum / count as f64, count));
            i = j;
        }
        rings
    }

    /// Calculate ratio of d-spacings between pairs of spots.
    pub fn d_spacing_ratios(&self) -> Vec<f64> {
        let mut ratios = Vec::new();
        for i in 0..self.spots.len() {
            for j in (i + 1)..self.spots.len() {
                let d1 = self.spots[i].d_spacing_angstrom;
                let d2 = self.spots[j].d_spacing_angstrom;
                if d2 > 1e-10 {
                    ratios.push(d1 / d2);
                }
            }
        }
        ratios
    }
}

// ─── ZoneAxisIdentifier ──────────────────────────────────────────────────────

/// Identifies zone axes from diffraction patterns.
///
/// The zone axis [uvw] satisfies hu + kv + lw = 0 for all (hkl) in the pattern.
#[derive(Debug, Clone)]
pub struct ZoneAxisIdentifier {
    /// Crystal lattice parameters.
    pub lattice: LatticeParameters,
}

impl ZoneAxisIdentifier {
    /// Create identifier for given crystal lattice.
    pub fn new(lattice: LatticeParameters) -> Self {
        Self { lattice }
    }

    /// Find zone axis from two diffraction vectors g1=(h1k1l1) and g2=(h2k2l2).
    /// Zone axis = g1 × g2
    pub fn zone_axis_from_two_spots(
        &self,
        g1: (i32, i32, i32),
        g2: (i32, i32, i32),
    ) -> (i32, i32, i32) {
        let u = g1.1 * g2.2 - g1.2 * g2.1;
        let v = g1.2 * g2.0 - g1.0 * g2.2;
        let w = g1.0 * g2.1 - g1.1 * g2.0;
        // Reduce by GCD
        let g = gcd3(u.unsigned_abs(), v.unsigned_abs(), w.unsigned_abs());
        if g == 0 {
            (u, v, w)
        } else {
            let g = g as i32;
            (u / g, v / g, w / g)
        }
    }

    /// Verify that a Miller index set (hkl) belongs to zone axis [uvw].
    /// Condition: h*u + k*v + l*w = 0
    pub fn is_in_zone(&self, hkl: (i32, i32, i32), uvw: (i32, i32, i32)) -> bool {
        hkl.0 * uvw.0 + hkl.1 * uvw.1 + hkl.2 * uvw.2 == 0
    }

    /// Calculate angle between two zone axes [u1v1w1] and [u2v2w2].
    /// Uses the metric tensor of the lattice.
    pub fn angle_between_zone_axes(
        &self,
        uvw1: (i32, i32, i32),
        uvw2: (i32, i32, i32),
    ) -> f64 {
        let (u1, v1, w1) = (uvw1.0 as f64, uvw1.1 as f64, uvw1.2 as f64);
        let (u2, v2, w2) = (uvw2.0 as f64, uvw2.1 as f64, uvw2.2 as f64);
        // For cubic: |uvw| = sqrt(u²+v²+w²), dot = u1u2+v1v2+w1w2
        let dot = u1 * u2 + v1 * v2 + w1 * w2;
        let mag1 = (u1 * u1 + v1 * v1 + w1 * w1).sqrt();
        let mag2 = (u2 * u2 + v2 * v2 + w2 * w2).sqrt();
        if mag1 < 1e-10 || mag2 < 1e-10 {
            return 0.0;
        }
        (dot / (mag1 * mag2)).clamp(-1.0, 1.0).acos()
    }
}

// ─── LatticeParameters ───────────────────────────────────────────────────────

/// Crystal lattice parameters.
#[derive(Debug, Clone)]
pub struct LatticeParameters {
    /// a-axis length in Å.
    pub a: f64,
    /// b-axis length in Å.
    pub b: f64,
    /// c-axis length in Å.
    pub c: f64,
    /// Alpha angle (b-c) in degrees.
    pub alpha: f64,
    /// Beta angle (a-c) in degrees.
    pub beta: f64,
    /// Gamma angle (a-b) in degrees.
    pub gamma: f64,
    /// Crystal system.
    pub system: CrystalSystem,
}

/// Crystal system classification.
#[derive(Debug, Clone, PartialEq)]
pub enum CrystalSystem {
    Cubic,
    Tetragonal,
    Orthorhombic,
    Hexagonal,
    Rhombohedral,
    Monoclinic,
    Triclinic,
}

impl LatticeParameters {
    /// Create cubic lattice (a=b=c, all angles 90°).
    pub fn cubic(a: f64) -> Self {
        Self { a, b: a, c: a, alpha: 90.0, beta: 90.0, gamma: 90.0, system: CrystalSystem::Cubic }
    }

    /// Create hexagonal lattice (a=b≠c, alpha=beta=90°, gamma=120°).
    pub fn hexagonal(a: f64, c: f64) -> Self {
        Self {
            a,
            b: a,
            c,
            alpha: 90.0,
            beta: 90.0,
            gamma: 120.0,
            system: CrystalSystem::Hexagonal,
        }
    }

    /// Create tetragonal lattice (a=b≠c, all angles 90°).
    pub fn tetragonal(a: f64, c: f64) -> Self {
        Self {
            a,
            b: a,
            c,
            alpha: 90.0,
            beta: 90.0,
            gamma: 90.0,
            system: CrystalSystem::Tetragonal,
        }
    }

    /// Calculate d-spacing for Miller indices (hkl) in Å.
    pub fn d_spacing(&self, h: i32, k: i32, l: i32) -> f64 {
        let (h, k, l) = (h as f64, k as f64, l as f64);
        match self.system {
            CrystalSystem::Cubic => {
                let a = self.a;
                a / (h * h + k * k + l * l).sqrt()
            }
            CrystalSystem::Tetragonal => {
                let a = self.a;
                let c = self.c;
                let inv_d_sq = (h * h + k * k) / (a * a) + l * l / (c * c);
                if inv_d_sq < 1e-30 {
                    f64::INFINITY
                } else {
                    inv_d_sq.sqrt().recip()
                }
            }
            CrystalSystem::Hexagonal => {
                let a = self.a;
                let c = self.c;
                let inv_d_sq =
                    (4.0 / 3.0) * (h * h + h * k + k * k) / (a * a) + l * l / (c * c);
                if inv_d_sq < 1e-30 {
                    f64::INFINITY
                } else {
                    inv_d_sq.sqrt().recip()
                }
            }
            _ => {
                // General triclinic formula (simplified orthorhombic fallback)
                let a = self.a;
                let b = self.b;
                let c = self.c;
                let inv_d_sq = h * h / (a * a) + k * k / (b * b) + l * l / (c * c);
                if inv_d_sq < 1e-30 {
                    f64::INFINITY
                } else {
                    inv_d_sq.sqrt().recip()
                }
            }
        }
    }

    /// Generate list of allowed reflections up to max_d in Å.
    /// Returns sorted list of (d_spacing, h, k, l).
    pub fn allowed_reflections(&self, max_d: f64, min_d: f64) -> Vec<(f64, i32, i32, i32)> {
        let mut reflections = Vec::new();
        let max_index = ((max_d / min_d) as i32 + 3).max(10);
        for h in -max_index..=max_index {
            for k in -max_index..=max_index {
                for l in -max_index..=max_index {
                    if h == 0 && k == 0 && l == 0 {
                        continue;
                    }
                    // Skip systematic absences for FCC
                    if self.system == CrystalSystem::Cubic {
                        if !self.is_allowed_cubic(h, k, l) {
                            continue;
                        }
                    }
                    let d = self.d_spacing(h, k, l);
                    if d >= min_d && d <= max_d {
                        reflections.push((d, h, k, l));
                    }
                }
            }
        }
        reflections.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
        reflections.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-6);
        reflections
    }

    /// Check if (hkl) is allowed for cubic FCC structure (h, k, l all even or all odd).
    fn is_allowed_cubic(&self, h: i32, k: i32, l: i32) -> bool {
        let h_even = h % 2 == 0;
        let k_even = k % 2 == 0;
        let l_even = l % 2 == 0;
        (h_even && k_even && l_even) || (!h_even && !k_even && !l_even)
    }
}

// ─── LatticeFringeAnalyzer ───────────────────────────────────────────────────

/// Analyzes lattice fringes in HRTEM images using FFT.
///
/// Converts spatial frequencies from FFT to d-spacings.
#[derive(Debug, Clone)]
pub struct LatticeFringeAnalyzer {
    /// Physical pixel size in Å/pixel.
    pub pixel_size_angstrom: f64,
}

impl LatticeFringeAnalyzer {
    /// Create analyzer with given pixel size.
    pub fn new(pixel_size_angstrom: f64) -> Self {
        Self { pixel_size_angstrom }
    }

    /// Compute 2D FFT power spectrum of an N×N image (row-major, real).
    /// Returns N×N power spectrum array.
    pub fn power_spectrum(&self, image: &[f64], n: usize) -> Vec<f64> {
        assert_eq!(image.len(), n * n, "Image must be N×N");
        // Compute 2D DFT via row then column 1D DFTs
        let mut complex: Vec<(f64, f64)> = image.iter().map(|&v| (v, 0.0)).collect();
        // Row DFTs
        for row in 0..n {
            let slice: Vec<(f64, f64)> = complex[row * n..(row + 1) * n].to_vec();
            let row_dft = dft_1d(&slice);
            complex[row * n..(row + 1) * n].copy_from_slice(&row_dft);
        }
        // Column DFTs
        for col in 0..n {
            let col_data: Vec<(f64, f64)> = (0..n).map(|r| complex[r * n + col]).collect();
            let col_dft = dft_1d(&col_data);
            for (r, val) in col_dft.iter().enumerate() {
                complex[r * n + col] = *val;
            }
        }
        // Power spectrum
        complex.iter().map(|(re, im)| re * re + im * im).collect()
    }

    /// Find peaks in the FFT power spectrum and return d-spacings in Å.
    /// `n` is image dimension, `min_radius_px` excludes direct beam region.
    pub fn find_d_spacings(
        &self,
        power: &[f64],
        n: usize,
        min_radius_px: f64,
        threshold_factor: f64,
    ) -> Vec<FringeResult> {
        let cx = (n / 2) as f64;
        let cy = (n / 2) as f64;
        let mean_power = power.iter().sum::<f64>() / power.len() as f64;
        let threshold = mean_power * threshold_factor;

        // Shift FFT so DC is at center
        let shifted = fftshift(power, n);

        let mut results = Vec::new();
        for row in 0..n {
            for col in 0..n {
                let val = shifted[row * n + col];
                if val < threshold {
                    continue;
                }
                let dx = col as f64 - cx;
                let dy = row as f64 - cy;
                let r = (dx * dx + dy * dy).sqrt();
                if r < min_radius_px {
                    continue;
                }
                // Spatial frequency in 1/Å
                let freq = r / (n as f64 * self.pixel_size_angstrom);
                let d = if freq > 1e-10 { 1.0 / freq } else { f64::INFINITY };
                let angle = dy.atan2(dx).to_degrees();
                results.push(FringeResult { d_angstrom: d, spatial_freq: freq, angle_deg: angle, intensity: val });
            }
        }
        // Sort by intensity descending
        results.sort_by(|a, b| b.intensity.partial_cmp(&a.intensity).unwrap());
        results
    }

    /// Measure lattice parameter from dominant fringe spacing.
    /// For cubic crystal observed along zone axis.
    pub fn measure_lattice_parameter(
        &self,
        d_angstrom: f64,
        h: i32,
        k: i32,
        l: i32,
    ) -> f64 {
        // For cubic: a = d * sqrt(h²+k²+l²)
        let mag = ((h * h + k * k + l * l) as f64).sqrt();
        d_angstrom * mag
    }
}

/// Result from lattice fringe analysis.
#[derive(Debug, Clone)]
pub struct FringeResult {
    /// Measured d-spacing in Å.
    pub d_angstrom: f64,
    /// Spatial frequency in 1/Å.
    pub spatial_freq: f64,
    /// Fringe orientation angle in degrees.
    pub angle_deg: f64,
    /// Peak intensity in power spectrum.
    pub intensity: f64,
}

// ─── ContrastTransferFunction ────────────────────────────────────────────────

/// Contrast Transfer Function (CTF) model for HRTEM imaging.
///
/// Models phase contrast transfer including defocus, spherical aberration,
/// and envelope functions (coherence and chromatic).
///
/// ```text
/// CTF(u) = -2 sin(χ(u)) · E_s(u) · E_c(u)
/// χ(u)  = π λ u² (Δf - ½ Cs λ² u²)
/// ```
#[derive(Debug, Clone)]
pub struct ContrastTransferFunction {
    /// Defocus Δf in nm (positive = underfocus).
    pub defocus_nm: f64,
    /// Spherical aberration Cs in mm.
    pub cs_mm: f64,
    /// Chromatic aberration Cc in mm.
    pub cc_mm: f64,
    /// Electron wavelength in nm.
    pub wavelength_nm: f64,
    /// Convergence semi-angle α in mrad.
    pub convergence_mrad: f64,
    /// Energy spread ΔE in eV.
    pub energy_spread_ev: f64,
    /// Accelerating voltage in kV (for chromatic envelope).
    pub voltage_kv: f64,
}

impl ContrastTransferFunction {
    /// Create CTF for a given electron beam and microscope parameters.
    pub fn new(
        defocus_nm: f64,
        cs_mm: f64,
        cc_mm: f64,
        beam: &ElectronBeam,
        convergence_mrad: f64,
        energy_spread_ev: f64,
    ) -> Self {
        let wavelength_nm = beam.wavelength_m * 1e9;
        let voltage_kv = beam.voltage_v / 1000.0;
        Self {
            defocus_nm,
            cs_mm,
            cc_mm,
            wavelength_nm,
            convergence_mrad,
            energy_spread_ev,
            voltage_kv,
        }
    }

    /// Phase error function χ(u) at spatial frequency u (1/nm).
    pub fn chi(&self, u: f64) -> f64 {
        let lambda = self.wavelength_nm;
        let cs = self.cs_mm * 1e6; // mm to nm
        let df = self.defocus_nm;
        PI * lambda * u * u * (df - 0.5 * cs * lambda * lambda * u * u)
    }

    /// CTF value at spatial frequency u (1/nm).
    /// Returns value in [-1, 1].
    pub fn ctf(&self, u: f64) -> f64 {
        let chi = self.chi(u);
        let es = self.spatial_envelope(u);
        let ec = self.chromatic_envelope(u);
        -2.0 * chi.sin() * es * ec
    }

    /// Spatial coherence (convergence) envelope function E_s(u).
    pub fn spatial_envelope(&self, u: f64) -> f64 {
        let alpha = self.convergence_mrad * 1e-3; // mrad to rad
        let lambda = self.wavelength_nm;
        let cs = self.cs_mm * 1e6; // mm to nm
        let df = self.defocus_nm;
        let arg = PI * alpha * u * (cs * lambda * lambda * u * u - df) / lambda;
        (-arg * arg / 2.0).exp()
    }

    /// Chromatic coherence envelope function E_c(u).
    pub fn chromatic_envelope(&self, u: f64) -> f64 {
        let cc = self.cc_mm * 1e6; // mm to nm
        let lambda = self.wavelength_nm;
        let delta_e = self.energy_spread_ev;
        let voltage_v = self.voltage_kv * 1000.0;
        let delta_f = cc * delta_e / voltage_v;
        let arg = PI * lambda * delta_f * u * u / (2.0 * 2.0_f64.ln().sqrt());
        (-arg * arg).exp()
    }

    /// Compute CTF profile over spatial frequency range.
    /// Returns (frequencies 1/nm, CTF values).
    pub fn profile(&self, u_min: f64, u_max: f64, n_points: usize) -> (Vec<f64>, Vec<f64>) {
        let freqs: Vec<f64> = (0..n_points)
            .map(|i| u_min + (u_max - u_min) * i as f64 / (n_points - 1) as f64)
            .collect();
        let ctf_vals: Vec<f64> = freqs.iter().map(|&u| self.ctf(u)).collect();
        (freqs, ctf_vals)
    }

    /// Scherzer defocus: optimal underfocus for broad pass-band.
    /// Δf_S = sqrt(1.2 * Cs * λ) (in nm).
    pub fn scherzer_defocus(&self) -> f64 {
        let lambda = self.wavelength_nm;
        let cs = self.cs_mm * 1e6; // mm to nm
        (1.2 * cs * lambda).sqrt()
    }

    /// Point resolution at Scherzer defocus (1/u_max in nm).
    pub fn point_resolution(&self) -> f64 {
        let lambda = self.wavelength_nm;
        let cs = self.cs_mm * 1e6; // mm to nm
        0.66 * (cs * lambda * lambda * lambda).powf(0.25)
    }

    /// Find Thon ring zero crossings (defocus estimation from rings).
    /// Returns spatial frequencies (1/nm) where CTF = 0.
    pub fn thon_ring_zeros(&self, u_max: f64, n_points: usize) -> Vec<f64> {
        let (freqs, ctfs) = self.profile(0.01, u_max, n_points);
        let mut zeros = Vec::new();
        for i in 1..ctfs.len() {
            if ctfs[i - 1] * ctfs[i] < 0.0 {
                // Linear interpolation of zero crossing
                let u0 = freqs[i - 1]
                    + (freqs[i] - freqs[i - 1]) * (-ctfs[i - 1]) / (ctfs[i] - ctfs[i - 1]);
                zeros.push(u0);
            }
        }
        zeros
    }

    /// Estimate defocus from observed Thon ring radii.
    /// Thon rings occur at zeros of CTF.
    /// For ring n: π λ u_n² Δf ≈ n π → u_n² ≈ n/(λΔf)
    pub fn estimate_defocus_from_rings(
        lambda_nm: f64,
        ring_freqs_per_nm: &[f64],
    ) -> Option<f64> {
        if ring_freqs_per_nm.len() < 2 {
            return None;
        }
        // Use first two rings: u1² ≈ 1/(λΔf), u2² ≈ 2/(λΔf)
        // Δf ≈ n / (λ * u_n²)
        let mut df_estimates = Vec::new();
        for (n, &u) in ring_freqs_per_nm.iter().enumerate() {
            if u > 1e-10 {
                let n_f = (n + 1) as f64;
                df_estimates.push(n_f / (lambda_nm * u * u));
            }
        }
        if df_estimates.is_empty() {
            return None;
        }
        Some(df_estimates.iter().sum::<f64>() / df_estimates.len() as f64)
    }
}

// ─── SaedAnalyzer ─────────────────────────────────────────────────────────────

/// Selected Area Electron Diffraction (SAED) ring pattern analyzer.
///
/// Used for polycrystalline specimens where rings appear instead of spots.
#[derive(Debug, Clone)]
pub struct SaedAnalyzer {
    pub calibration: CameraCalibration,
    pub material: CrystalMaterial,
}

/// Ring measurement from SAED pattern.
#[derive(Debug, Clone)]
pub struct SaedRing {
    /// Measured ring radius in pixels.
    pub radius_px: f64,
    /// Measured d-spacing in Å.
    pub d_measured: f64,
    /// Indexed (hkl) assignment.
    pub hkl: Option<(i32, i32, i32)>,
    /// Reference d-spacing for matched hkl in Å.
    pub d_reference: Option<f64>,
    /// Relative error in %.
    pub error_pct: Option<f64>,
}

impl SaedAnalyzer {
    /// Create SAED analyzer.
    pub fn new(calibration: CameraCalibration, material: CrystalMaterial) -> Self {
        Self { calibration, material }
    }

    /// Index measured ring radii against known crystal planes.
    pub fn index_rings(
        &self,
        ring_radii_px: &[f64],
        tolerance_pct: f64,
    ) -> Vec<SaedRing> {
        let lattice = self.material.lattice.clone();
        let refs = lattice.allowed_reflections(20.0, 0.5);

        ring_radii_px
            .iter()
            .map(|&r| {
                let d_meas = self.calibration.d_spacing_angstrom_from_pixels(r);
                // Find closest reference
                let best = refs.iter().min_by(|a, b| {
                    let ea = (a.0 - d_meas).abs();
                    let eb = (b.0 - d_meas).abs();
                    ea.partial_cmp(&eb).unwrap()
                });
                if let Some(&(d_ref, h, k, l)) = best {
                    let err = (d_meas - d_ref).abs() / d_ref * 100.0;
                    if err <= tolerance_pct {
                        SaedRing {
                            radius_px: r,
                            d_measured: d_meas,
                            hkl: Some((h, k, l)),
                            d_reference: Some(d_ref),
                            error_pct: Some(err),
                        }
                    } else {
                        SaedRing {
                            radius_px: r,
                            d_measured: d_meas,
                            hkl: None,
                            d_reference: None,
                            error_pct: None,
                        }
                    }
                } else {
                    SaedRing {
                        radius_px: r,
                        d_measured: d_meas,
                        hkl: None,
                        d_reference: None,
                        error_pct: None,
                    }
                }
            })
            .collect()
    }

    /// Calculate lattice parameter from indexed ring.
    /// For cubic: a = d * sqrt(h²+k²+l²)
    pub fn refine_lattice_parameter(&self, rings: &[SaedRing]) -> Option<f64> {
        let indexed: Vec<f64> = rings
            .iter()
            .filter_map(|r| {
                if let (Some(hkl), d) = (r.hkl, r.d_measured) {
                    let mag = ((hkl.0 * hkl.0 + hkl.1 * hkl.1 + hkl.2 * hkl.2) as f64).sqrt();
                    Some(d * mag)
                } else {
                    None
                }
            })
            .collect();
        if indexed.is_empty() {
            None
        } else {
            Some(indexed.iter().sum::<f64>() / indexed.len() as f64)
        }
    }
}

// ─── KikuchiAnalyzer ─────────────────────────────────────────────────────────

/// Kikuchi pattern analyzer for crystal orientation determination.
///
/// Kikuchi lines appear as bright/dark pairs when electron channeling occurs
/// along specific crystal planes. Band width W = 2θ_B * L, where θ_B is the
/// Bragg angle and L is the camera length.
#[derive(Debug, Clone)]
pub struct KikuchiAnalyzer {
    pub calibration: CameraCalibration,
    pub beam: ElectronBeam,
}

/// A detected Kikuchi band.
#[derive(Debug, Clone)]
pub struct KikuchiBand {
    /// Band width in pixels.
    pub width_px: f64,
    /// Bragg angle in radians (from band width).
    pub bragg_angle_rad: f64,
    /// Corresponding d-spacing in Å.
    pub d_spacing_angstrom: f64,
    /// Band center azimuth in degrees.
    pub azimuth_deg: f64,
    /// Indexed (hkl) if known.
    pub hkl: Option<(i32, i32, i32)>,
}

impl KikuchiAnalyzer {
    /// Create Kikuchi analyzer.
    pub fn new(calibration: CameraCalibration, beam: ElectronBeam) -> Self {
        Self { calibration, beam }
    }

    /// Calculate d-spacing from Kikuchi band width.
    /// Band width W = 2 * theta_B * L / pixel_size
    /// d = lambda / (2 * sin(theta_B)) ≈ lambda * L / W (small angle)
    pub fn d_from_band_width(&self, width_px: f64) -> f64 {
        let lambda = self.beam.wavelength_m;
        let l = self.calibration.camera_length_m;
        let px = self.calibration.pixel_size_m;
        let width_m = width_px * px;
        // theta_B = width_m / (2 * L)
        let theta_b = width_m / (2.0 * l);
        // d = lambda / (2 * sin(theta_B))
        if theta_b.abs() < 1e-15 {
            return f64::INFINITY;
        }
        lambda / (2.0 * theta_b.sin()) * 1e10 // return in Å
    }

    /// Analyze a set of detected band widths.
    pub fn analyze_bands(&self, widths_px: &[f64], azimuths_deg: &[f64]) -> Vec<KikuchiBand> {
        widths_px
            .iter()
            .zip(azimuths_deg.iter())
            .map(|(&w, &az)| {
                let d = self.d_from_band_width(w);
                let l = self.calibration.camera_length_m;
                let px = self.calibration.pixel_size_m;
                let width_m = w * px;
                let theta_b = width_m / (2.0 * l);
                KikuchiBand {
                    width_px: w,
                    bragg_angle_rad: theta_b,
                    d_spacing_angstrom: d,
                    azimuth_deg: az,
                    hkl: None,
                }
            })
            .collect()
    }
}

// ─── ThicknessMeasurement ────────────────────────────────────────────────────

/// Electron energy loss spectroscopy (EELS) log-ratio thickness measurement.
///
/// Uses the log-ratio method: t/λ = ln(I_t / I_0)
/// where I_t is total intensity and I_0 is zero-loss peak intensity.
#[derive(Debug, Clone)]
pub struct ThicknessMeasurement {
    /// Mean free path λ in nm for the material.
    pub mfp_nm: f64,
}

/// Material mean free path database for common specimens.
pub struct MfpDatabase;

impl MfpDatabase {
    /// Returns approximate mean free path in nm at 200 kV.
    /// Values from Malis formula: λ ≈ 106 F E_0 / (E_m ln(2βE_0/E_m))
    pub fn mfp_200kv(material: &str) -> Option<f64> {
        match material {
            "Al" | "aluminum" => Some(100.0),
            "Si" | "silicon" => Some(100.0),
            "Cu" | "copper" => Some(90.0),
            "Fe" | "iron" => Some(75.0),
            "Au" | "gold" => Some(60.0),
            "Ti" | "titanium" => Some(85.0),
            "Ni" | "nickel" => Some(80.0),
            "C" | "carbon" => Some(120.0),
            _ => None,
        }
    }
}

impl ThicknessMeasurement {
    /// Create measurement using known mean free path.
    pub fn new(mfp_nm: f64) -> Self {
        Self { mfp_nm }
    }

    /// Calculate thickness from EELS intensities.
    /// t = λ * ln(I_total / I_zero_loss)
    pub fn thickness_nm(&self, i_total: f64, i_zero_loss: f64) -> f64 {
        if i_zero_loss <= 0.0 || i_total <= 0.0 {
            return 0.0;
        }
        self.mfp_nm * (i_total / i_zero_loss).ln()
    }

    /// Relative thickness t/λ (dimensionless).
    pub fn relative_thickness(&self, i_total: f64, i_zero_loss: f64) -> f64 {
        if i_zero_loss <= 0.0 || i_total <= 0.0 {
            return 0.0;
        }
        (i_total / i_zero_loss).ln()
    }

    /// Estimate mean free path from Malis formula (approximate).
    ///
    /// Malis et al. (1988) formula:
    /// λ = 106 * F * E0 / (E_m * ln(2β*F*E0 / E_m))
    ///
    /// where:
    /// - E0   = beam energy in keV (relativistic: use E0*F)
    /// - E_m  = 7.6 * Z^0.36 (characteristic excitation energy)
    /// - F    = (1 + E0/1022) / (1 + E0/511)^2 (relativistic factor)
    /// - β    = 1 (collection angle parameter, normalized)
    ///
    /// Result in nm.
    pub fn malis_mfp(z: f64, e0_kev: f64) -> f64 {
        // Relativistic factor F
        let f = (1.0 + e0_kev / 1022.0) / (1.0 + e0_kev / 511.0).powi(2);
        // Characteristic energy for material
        let em = 7.6 * z.powf(0.36);
        // Effective relativistic energy
        let ef = f * e0_kev;
        // Argument of logarithm (2β*F*E0/Em, β normalized to 1)
        let arg = 2.0 * ef / em;
        if arg <= 1.0 {
            return 100.0; // Fallback for unphysical parameters
        }
        106.0 * f * e0_kev / (em * arg.ln())
    }
}

// ─── ImageFilter ─────────────────────────────────────────────────────────────

/// TEM image filtering utilities.
///
/// Implements Wiener filter and bandpass filtering in Fourier space.
#[derive(Debug, Clone)]
pub struct ImageFilter {
    pub n: usize,
    pub pixel_size_angstrom: f64,
}

impl ImageFilter {
    /// Create image filter for N×N image.
    pub fn new(n: usize, pixel_size_angstrom: f64) -> Self {
        Self { n, pixel_size_angstrom }
    }

    /// Apply Wiener filter to reduce noise.
    /// SNR parameter: higher = less smoothing.
    pub fn wiener_filter(&self, image: &[f64], snr: f64) -> Vec<f64> {
        assert_eq!(image.len(), self.n * self.n);
        let n = self.n;

        // Forward DFT
        let mut complex: Vec<(f64, f64)> = image.iter().map(|&v| (v, 0.0)).collect();
        for row in 0..n {
            let s = complex[row * n..(row + 1) * n].to_vec();
            let d = dft_1d(&s);
            complex[row * n..(row + 1) * n].copy_from_slice(&d);
        }
        for col in 0..n {
            let c: Vec<(f64, f64)> = (0..n).map(|r| complex[r * n + col]).collect();
            let d = dft_1d(&c);
            for (r, v) in d.iter().enumerate() {
                complex[r * n + col] = *v;
            }
        }

        // Apply Wiener filter: H(u,v) = |S|² / (|S|² + 1/SNR)
        for val in complex.iter_mut() {
            let power = val.0 * val.0 + val.1 * val.1;
            let h = power / (power + 1.0 / snr);
            val.0 *= h;
            val.1 *= h;
        }

        // Inverse DFT
        self.idft_2d(&complex)
    }

    /// Apply bandpass filter to select spatial frequencies.
    /// Passes frequencies between d_min and d_max in Å.
    pub fn bandpass_filter(&self, image: &[f64], d_min_angstrom: f64, d_max_angstrom: f64) -> Vec<f64> {
        assert_eq!(image.len(), self.n * self.n);
        let n = self.n;
        let cx = n as f64 / 2.0;
        let cy = n as f64 / 2.0;

        // Forward DFT
        let mut complex: Vec<(f64, f64)> = image.iter().map(|&v| (v, 0.0)).collect();
        for row in 0..n {
            let s = complex[row * n..(row + 1) * n].to_vec();
            let d = dft_1d(&s);
            complex[row * n..(row + 1) * n].copy_from_slice(&d);
        }
        for col in 0..n {
            let c: Vec<(f64, f64)> = (0..n).map(|r| complex[r * n + col]).collect();
            let d = dft_1d(&c);
            for (r, v) in d.iter().enumerate() {
                complex[r * n + col] = *v;
            }
        }

        // Apply bandpass mask
        let shifted = fftshift_complex(&complex, n);
        let mut masked: Vec<(f64, f64)> = shifted
            .iter()
            .enumerate()
            .map(|(idx, &v)| {
                let row = idx / n;
                let col = idx % n;
                let dx = col as f64 - cx;
                let dy = row as f64 - cy;
                let r = (dx * dx + dy * dy).sqrt();
                let freq = r / (n as f64 * self.pixel_size_angstrom);
                let d = if freq > 1e-10 { 1.0 / freq } else { f64::INFINITY };
                if d >= d_min_angstrom && d <= d_max_angstrom {
                    v
                } else {
                    (0.0, 0.0)
                }
            })
            .collect();

        // Inverse shift
        masked = ifftshift_complex(&masked, n);

        // Inverse DFT
        self.idft_2d(&masked)
    }

    /// Compute 2D inverse DFT.
    fn idft_2d(&self, complex: &[(f64, f64)]) -> Vec<f64> {
        let n = self.n;
        // Conjugate, DFT, conjugate, normalize
        let mut c: Vec<(f64, f64)> = complex.iter().map(|(re, im)| (*re, -im)).collect();
        // Row DFTs
        for row in 0..n {
            let s = c[row * n..(row + 1) * n].to_vec();
            let d = dft_1d(&s);
            c[row * n..(row + 1) * n].copy_from_slice(&d);
        }
        // Column DFTs
        for col in 0..n {
            let col_data: Vec<(f64, f64)> = (0..n).map(|r| c[r * n + col]).collect();
            let d = dft_1d(&col_data);
            for (r, v) in d.iter().enumerate() {
                c[r * n + col] = *v;
            }
        }
        let norm = (n * n) as f64;
        c.iter().map(|(re, im)| re / norm).collect()
    }
}

// ─── MoireAnalyzer ───────────────────────────────────────────────────────────

/// Moiré pattern analysis for overlapping crystal planes.
///
/// Moiré fringes form when two periodic lattices overlap at a relative
/// rotation or scale difference. The moiré period is:
///
/// ```text
/// D = d1*d2 / sqrt(d1²+d2²-2*d1*d2*cos(θ))
/// ```
#[derive(Debug, Clone)]
pub struct MoireAnalyzer;

/// Result of Moiré analysis.
#[derive(Debug, Clone)]
pub struct MoireResult {
    /// Moiré period in Å.
    pub period_angstrom: f64,
    /// Moiré angle relative to crystal 1 in degrees.
    pub angle_deg: f64,
    /// Rotation between lattices in degrees.
    pub rotation_deg: f64,
    /// d-spacing of lattice 1 in Å.
    pub d1_angstrom: f64,
    /// d-spacing of lattice 2 in Å.
    pub d2_angstrom: f64,
}

impl MoireAnalyzer {
    /// Calculate Moiré pattern parameters from two overlapping lattices.
    ///
    /// # Arguments
    /// * `d1` - d-spacing of lattice 1 in Å
    /// * `d2` - d-spacing of lattice 2 in Å
    /// * `theta_deg` - Rotation angle between lattices in degrees
    pub fn calculate(d1: f64, d2: f64, theta_deg: f64) -> MoireResult {
        let theta = theta_deg.to_radians();
        let cos_t = theta.cos();
        let period_sq = d1 * d1 + d2 * d2 - 2.0 * d1 * d2 * cos_t;
        let period = if period_sq > 0.0 { (d1 * d1 * d2 * d2 / period_sq).sqrt() } else { f64::INFINITY };

        // Moiré angle: atan(d2*sin(θ) / (d1 - d2*cos(θ)))
        let sin_t = theta.sin();
        let angle = (d2 * sin_t).atan2(d1 - d2 * cos_t).to_degrees();

        MoireResult {
            period_angstrom: period,
            angle_deg: angle,
            rotation_deg: theta_deg,
            d1_angstrom: d1,
            d2_angstrom: d2,
        }
    }

    /// Rotation Moiré (same d-spacing, small twist angle).
    /// D ≈ d / θ for small angles.
    pub fn rotation_moire(d: f64, theta_deg: f64) -> f64 {
        let theta = theta_deg.to_radians();
        if theta.abs() < 1e-10 {
            return f64::INFINITY;
        }
        d / (2.0 * (theta / 2.0).sin())
    }

    /// Parallel Moiré (same orientation, different d-spacings).
    /// D = d1*d2 / |d1 - d2|
    pub fn parallel_moire(d1: f64, d2: f64) -> f64 {
        let diff = (d1 - d2).abs();
        if diff < 1e-10 {
            return f64::INFINITY;
        }
        d1 * d2 / diff
    }
}

// ─── MaterialDatabase ────────────────────────────────────────────────────────

/// Crystal structure types.
#[derive(Debug, Clone, PartialEq)]
pub enum StructureType {
    Fcc,
    Bcc,
    Hcp,
    Diamond,
    NaCl,
}

/// Crystal material with lattice parameters and structure.
#[derive(Debug, Clone)]
pub struct CrystalMaterial {
    /// Material name.
    pub name: &'static str,
    /// Chemical formula.
    pub formula: &'static str,
    /// Crystal structure type.
    pub structure: StructureType,
    /// Lattice parameters.
    pub lattice: LatticeParameters,
    /// Atomic number (for electron scattering factor).
    pub atomic_number: u32,
    /// Density in g/cm³.
    pub density: f64,
}

/// Database of common TEM specimen materials.
pub struct MaterialDatabase;

impl MaterialDatabase {
    /// Get material by name.
    pub fn get(name: &str) -> Option<CrystalMaterial> {
        match name.to_lowercase().as_str() {
            "al" | "aluminum" | "aluminium" => Some(Self::aluminum()),
            "cu" | "copper" => Some(Self::copper()),
            "au" | "gold" => Some(Self::gold()),
            "si" | "silicon" => Some(Self::silicon()),
            "fe" | "iron" => Some(Self::iron()),
            "ti" | "titanium" => Some(Self::titanium()),
            _ => None,
        }
    }

    /// Aluminum (FCC, a = 4.049 Å).
    pub fn aluminum() -> CrystalMaterial {
        CrystalMaterial {
            name: "Aluminum",
            formula: "Al",
            structure: StructureType::Fcc,
            lattice: LatticeParameters::cubic(4.049),
            atomic_number: 13,
            density: 2.70,
        }
    }

    /// Copper (FCC, a = 3.615 Å).
    pub fn copper() -> CrystalMaterial {
        CrystalMaterial {
            name: "Copper",
            formula: "Cu",
            structure: StructureType::Fcc,
            lattice: LatticeParameters::cubic(3.615),
            atomic_number: 29,
            density: 8.96,
        }
    }

    /// Gold (FCC, a = 4.078 Å).
    pub fn gold() -> CrystalMaterial {
        CrystalMaterial {
            name: "Gold",
            formula: "Au",
            structure: StructureType::Fcc,
            lattice: LatticeParameters::cubic(4.078),
            atomic_number: 79,
            density: 19.32,
        }
    }

    /// Silicon (Diamond cubic, a = 5.431 Å).
    pub fn silicon() -> CrystalMaterial {
        CrystalMaterial {
            name: "Silicon",
            formula: "Si",
            structure: StructureType::Diamond,
            lattice: LatticeParameters::cubic(5.431),
            atomic_number: 14,
            density: 2.33,
        }
    }

    /// Iron (BCC, a = 2.870 Å).
    pub fn iron() -> CrystalMaterial {
        CrystalMaterial {
            name: "Iron",
            formula: "Fe",
            structure: StructureType::Bcc,
            lattice: LatticeParameters::cubic(2.870),
            atomic_number: 26,
            density: 7.87,
        }
    }

    /// Titanium (HCP, a = 2.951 Å, c = 4.686 Å).
    pub fn titanium() -> CrystalMaterial {
        CrystalMaterial {
            name: "Titanium",
            formula: "Ti",
            structure: StructureType::Hcp,
            lattice: LatticeParameters::hexagonal(2.951, 4.686),
            atomic_number: 22,
            density: 4.51,
        }
    }
}

// ─── ImageContrastModel ──────────────────────────────────────────────────────

/// Image contrast modes in TEM.
#[derive(Debug, Clone, PartialEq)]
pub enum ContrastMode {
    /// Bright-field: direct beam forms image. Thick/dense areas dark.
    BrightField,
    /// Dark-field: diffracted beam forms image. Diffracting grains bright.
    DarkField,
    /// Mass-thickness contrast: heavier/thicker areas scatter more.
    MassThickness,
    /// Diffraction contrast: strain fields visible via Bragg condition.
    Diffraction,
    /// Phase contrast: interference of beams for atomic resolution (HRTEM).
    Phase,
}

/// Estimate image contrast for given specimen conditions.
pub struct ContrastEstimator;

impl ContrastEstimator {
    /// Bright-field contrast from Poisson scattering model.
    /// I/I0 = exp(-t * μ) where μ is the total cross-section.
    pub fn bright_field_intensity(thickness_nm: f64, mu_per_nm: f64) -> f64 {
        (-thickness_nm * mu_per_nm).exp()
    }

    /// Mass-thickness contrast: ΔI/I ≈ constant * ρt (Rutherford scattering).
    /// C_mt ≈ Z^(4/3) * rho * t / A
    pub fn mass_thickness_contrast(z: f64, density_g_cm3: f64, thickness_nm: f64, atomic_mass: f64) -> f64 {
        let t_cm = thickness_nm * 1e-7;
        z.powf(4.0 / 3.0) * density_g_cm3 * t_cm / atomic_mass
    }

    /// Absorption mean free path from Malis formula (nm).
    pub fn total_mean_free_path(z: f64, e0_kev: f64) -> f64 {
        ThicknessMeasurement::malis_mfp(z, e0_kev)
    }
}

// ─── Helper Functions ─────────────────────────────────────────────────────────

/// 1D discrete Fourier transform (DFT) of complex input.
fn dft_1d(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = input.len();
    let mut output = vec![(0.0_f64, 0.0_f64); n];
    for k in 0..n {
        let mut re = 0.0;
        let mut im = 0.0;
        for (j, &(xr, xi)) in input.iter().enumerate() {
            let angle = -2.0 * PI * (k * j) as f64 / n as f64;
            let (sin_a, cos_a) = angle.sin_cos();
            re += xr * cos_a - xi * sin_a;
            im += xr * sin_a + xi * cos_a;
        }
        output[k] = (re, im);
    }
    output
}

/// FFT shift: move DC from corner to center of 2D array.
fn fftshift(data: &[f64], n: usize) -> Vec<f64> {
    let h = n / 2;
    let mut out = vec![0.0; n * n];
    for row in 0..n {
        for col in 0..n {
            let new_row = (row + h) % n;
            let new_col = (col + h) % n;
            out[new_row * n + new_col] = data[row * n + col];
        }
    }
    out
}

/// FFT shift for complex array.
fn fftshift_complex(data: &[(f64, f64)], n: usize) -> Vec<(f64, f64)> {
    let h = n / 2;
    let mut out = vec![(0.0, 0.0); n * n];
    for row in 0..n {
        for col in 0..n {
            let new_row = (row + h) % n;
            let new_col = (col + h) % n;
            out[new_row * n + new_col] = data[row * n + col];
        }
    }
    out
}

/// Inverse FFT shift.
fn ifftshift_complex(data: &[(f64, f64)], n: usize) -> Vec<(f64, f64)> {
    let h = (n + 1) / 2;
    let mut out = vec![(0.0, 0.0); n * n];
    for row in 0..n {
        for col in 0..n {
            let new_row = (row + h) % n;
            let new_col = (col + h) % n;
            out[new_row * n + new_col] = data[row * n + col];
        }
    }
    out
}

/// Greatest common divisor for three unsigned integers.
fn gcd(a: u32, b: u32) -> u32 {
    if b == 0 {
        a
    } else {
        gcd(b, a % b)
    }
}

fn gcd3(a: u32, b: u32, c: u32) -> u32 {
    gcd(gcd(a, b), c)
}

// ─── Unit Tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-6;

    // ─── ElectronBeam tests ───────────────────────────────────────────────────

    #[test]
    fn test_beam_100kv_wavelength() {
        let beam = ElectronBeam::new(100e3);
        // 100 kV: relativistic λ ≈ 3.70 pm
        let wl = beam.wavelength_pm();
        assert!((wl - 3.70).abs() < 0.05, "100 kV wavelength = {wl} pm");
    }

    #[test]
    fn test_beam_200kv_wavelength() {
        let beam = ElectronBeam::new(200e3);
        // 200 kV: relativistic λ ≈ 2.51 pm
        let wl = beam.wavelength_pm();
        assert!((wl - 2.51).abs() < 0.05, "200 kV wavelength = {wl} pm");
    }

    #[test]
    fn test_beam_300kv_wavelength() {
        let beam = ElectronBeam::new(300e3);
        // 300 kV: relativistic λ ≈ 1.97 pm
        let wl = beam.wavelength_pm();
        assert!((wl - 1.97).abs() < 0.05, "300 kV wavelength = {wl} pm");
    }

    #[test]
    fn test_beam_relativistic_correction() {
        let beam_low = ElectronBeam::new(1e3); // 1 kV: non-relativistic
        let beam_high = ElectronBeam::new(1000e3); // 1 MV: highly relativistic
        assert!(beam_high.gamma > beam_low.gamma);
        assert!(beam_high.wavelength_pm() < beam_low.wavelength_pm());
    }

    #[test]
    fn test_beam_beta() {
        let beam = ElectronBeam::new(200e3);
        let beta = beam.beta();
        assert!(beta > 0.0 && beta < 1.0);
        // At 200 kV, v/c ≈ 0.695
        assert!((beta - 0.695).abs() < 0.01, "beta = {beta}");
    }

    // ─── CameraCalibration tests ──────────────────────────────────────────────

    #[test]
    fn test_camera_constant() {
        let beam = ElectronBeam::new(200e3);
        let cal = CameraCalibration::new(1.0, &beam, 1e-5);
        // Camera constant = lambda * L
        let expected = beam.wavelength_m * 1.0;
        assert!((cal.camera_constant - expected).abs() < 1e-20);
    }

    #[test]
    fn test_d_spacing_from_radius() {
        let beam = ElectronBeam::new(200e3);
        // Camera constant = 2.51e-12 m * 1 m = 2.51e-12 m²
        let cal = CameraCalibration::new(1.0, &beam, 1e-5);
        // For spot at r = 1e-3 m: d = 2.51e-12 / 1e-3 = 2.51e-9 m = 0.251 Å
        let d = cal.d_spacing_from_radius(1e-3);
        assert!(d > 0.0, "d-spacing must be positive");
    }

    #[test]
    fn test_calibrate_from_known() {
        let beam = ElectronBeam::new(200e3);
        let lambda = beam.wavelength_m;
        let known_d = 2.338e-10; // Si (111) = 3.135 Å... use Al (111) = 2.338 Å
        let radius = 1e-3; // 1 mm
        let cal = CameraCalibration::calibrate_from_known(lambda, known_d, radius, 1e-5);
        let recovered = cal.d_spacing_from_radius(radius);
        assert!((recovered - known_d).abs() < 1e-15);
    }

    #[test]
    fn test_radius_pixels_roundtrip() {
        let beam = ElectronBeam::new(200e3);
        let cal = CameraCalibration::new(2.0, &beam, 5e-6);
        let d_original = 2.338; // Al (111) in Å
        let r_px = cal.radius_pixels_from_d_spacing(d_original);
        let d_recovered = cal.d_spacing_angstrom_from_pixels(r_px);
        assert!((d_recovered - d_original).abs() < 1e-10);
    }

    // ─── LatticeParameters tests ──────────────────────────────────────────────

    #[test]
    fn test_cubic_d_spacing_100() {
        let lat = LatticeParameters::cubic(4.049); // Al
        let d = lat.d_spacing(1, 0, 0);
        assert!((d - 4.049).abs() < 1e-6);
    }

    #[test]
    fn test_cubic_d_spacing_111() {
        let lat = LatticeParameters::cubic(4.049); // Al
        let d = lat.d_spacing(1, 1, 1);
        let expected = 4.049 / 3.0_f64.sqrt();
        assert!((d - expected).abs() < 1e-6);
    }

    #[test]
    fn test_cubic_d_spacing_200() {
        let lat = LatticeParameters::cubic(4.049);
        let d = lat.d_spacing(2, 0, 0);
        assert!((d - 2.0245).abs() < 1e-4);
    }

    #[test]
    fn test_hexagonal_d_spacing_0001() {
        let lat = LatticeParameters::hexagonal(2.951, 4.686); // Ti
        let d = lat.d_spacing(0, 0, 1);
        assert!((d - 4.686).abs() < 1e-6);
    }

    #[test]
    fn test_hexagonal_d_spacing_1010() {
        let lat = LatticeParameters::hexagonal(2.951, 4.686);
        let d = lat.d_spacing(1, 0, 0);
        // For hexagonal: 1/d² = 4/3 * (h²+hk+k²)/a² + l²/c²
        // (1,0,0): 1/d² = 4/(3*a²) → d = a*sqrt(3)/2
        let expected = lat.a * 3.0_f64.sqrt() / 2.0;
        assert!((d - expected).abs() < 1e-6);
    }

    #[test]
    fn test_al_111_d_spacing() {
        let al = MaterialDatabase::aluminum();
        let d = al.lattice.d_spacing(1, 1, 1);
        // Al (111): 4.049 / sqrt(3) ≈ 2.338 Å
        assert!((d - 2.338).abs() < 0.001, "Al (111) d = {d} Å");
    }

    #[test]
    fn test_cu_111_d_spacing() {
        let cu = MaterialDatabase::copper();
        let d = cu.lattice.d_spacing(1, 1, 1);
        // Cu (111): 3.615 / sqrt(3) ≈ 2.088 Å
        assert!((d - 2.088).abs() < 0.001, "Cu (111) d = {d} Å");
    }

    #[test]
    fn test_au_200_d_spacing() {
        let au = MaterialDatabase::gold();
        let d = au.lattice.d_spacing(2, 0, 0);
        // Au (200): 4.078 / 2 = 2.039 Å
        assert!((d - 2.039).abs() < 0.001, "Au (200) d = {d} Å");
    }

    // ─── ZoneAxisIdentifier tests ─────────────────────────────────────────────

    #[test]
    fn test_zone_axis_001_from_spots() {
        let al = MaterialDatabase::aluminum();
        let ident = ZoneAxisIdentifier::new(al.lattice);
        // g1=(100), g2=(010) → zone axis (001)
        let za = ident.zone_axis_from_two_spots((1, 0, 0), (0, 1, 0));
        assert_eq!(za, (0, 0, 1));
    }

    #[test]
    fn test_zone_axis_110_from_spots() {
        let al = MaterialDatabase::aluminum();
        let ident = ZoneAxisIdentifier::new(al.lattice);
        // g1=(001), g2=(1-10) → zone axis = (001)×(1-10)
        let za = ident.zone_axis_from_two_spots((0, 0, 1), (1, -1, 0));
        // Cross product: (0*0-1*(-1), 1*1-0*0, 0*(-1)-0*1) = (1, 1, 0)
        assert_eq!(za, (1, 1, 0));
    }

    #[test]
    fn test_is_in_zone() {
        let al = MaterialDatabase::aluminum();
        let ident = ZoneAxisIdentifier::new(al.lattice);
        // (100) is in zone [001]: 1*0 + 0*0 + 0*1 = 0 ✓
        assert!(ident.is_in_zone((1, 0, 0), (0, 0, 1)));
        // (001) not in zone [001]: 0*0 + 0*0 + 1*1 = 1 ≠ 0
        assert!(!ident.is_in_zone((0, 0, 1), (0, 0, 1)));
    }

    #[test]
    fn test_zone_axis_angle() {
        let al = MaterialDatabase::aluminum();
        let ident = ZoneAxisIdentifier::new(al.lattice);
        // Angle between [100] and [010] should be 90°
        let angle = ident.angle_between_zone_axes((1, 0, 0), (0, 1, 0));
        assert!((angle - PI / 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_zone_axis_angle_identity() {
        let al = MaterialDatabase::aluminum();
        let ident = ZoneAxisIdentifier::new(al.lattice);
        let angle = ident.angle_between_zone_axes((1, 1, 0), (1, 1, 0));
        // acos(~1.0) is effectively 0, but floating point sqrt gives small residual
        assert!(angle.abs() < 1e-6, "identity angle = {angle:.e}");
    }

    // ─── DiffractionPattern tests ─────────────────────────────────────────────

    #[test]
    fn test_diffraction_spot_radius() {
        let beam = ElectronBeam::new(200e3);
        let cal = CameraCalibration::new(1.0, &beam, 1e-5);
        let spot = DiffractionSpot::new(3.0, 4.0, 1.0, &cal);
        assert!((spot.radius() - 5.0).abs() < EPS);
    }

    #[test]
    fn test_diffraction_spot_angle() {
        let beam = ElectronBeam::new(200e3);
        let cal = CameraCalibration::new(1.0, &beam, 1e-5);
        let s1 = DiffractionSpot::new(1.0, 0.0, 1.0, &cal);
        let s2 = DiffractionSpot::new(0.0, 1.0, 1.0, &cal);
        let angle = s1.angle_to(&s2);
        assert!((angle - PI / 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_diffraction_pattern_add_sort() {
        let beam = ElectronBeam::new(200e3);
        let cal = CameraCalibration::new(1.0, &beam, 1e-5);
        let mut dp = DiffractionPattern::new(cal, (256.0, 256.0));
        dp.add_spot(296.0, 256.0, 1.0); // r = 40 px
        dp.add_spot(356.0, 256.0, 1.0); // r = 100 px
        dp.add_spot(276.0, 256.0, 1.0); // r = 20 px
        dp.sort_by_radius();
        assert!(dp.spots[0].radius() <= dp.spots[1].radius());
        assert!(dp.spots[1].radius() <= dp.spots[2].radius());
    }

    #[test]
    fn test_diffraction_find_rings() {
        let beam = ElectronBeam::new(200e3);
        let cal = CameraCalibration::new(1.0, &beam, 1e-5);
        let mut dp = DiffractionPattern::new(cal, (256.0, 256.0));
        // Add 4 spots at same radius (one ring)
        for angle in [0.0_f64, 90.0, 180.0, 270.0] {
            let x = 256.0 + 50.0 * angle.to_radians().cos();
            let y = 256.0 + 50.0 * angle.to_radians().sin();
            dp.add_spot(x, y, 1.0);
        }
        let rings = dp.find_rings(5.0);
        assert_eq!(rings.len(), 1);
        assert_eq!(rings[0].1, 4); // 4 spots in the ring
    }

    // ─── CTF tests ────────────────────────────────────────────────────────────

    #[test]
    fn test_ctf_zero_at_zero_freq() {
        let beam = ElectronBeam::new(200e3);
        let ctf = ContrastTransferFunction::new(50.0, 1.2, 1.2, &beam, 0.5, 0.3);
        // CTF(0) = -2*sin(0) = 0
        let val = ctf.ctf(0.0);
        assert!(val.abs() < EPS);
    }

    #[test]
    fn test_ctf_bounded() {
        let beam = ElectronBeam::new(200e3);
        let ctf = ContrastTransferFunction::new(50.0, 1.2, 1.2, &beam, 0.5, 0.3);
        for u in [0.1, 0.5, 1.0, 2.0, 5.0] {
            let val = ctf.ctf(u);
            assert!(val.abs() <= 1.0 + 1e-10, "CTF({u}) = {val} out of bounds");
        }
    }

    #[test]
    fn test_scherzer_defocus() {
        let beam = ElectronBeam::new(200e3);
        let ctf = ContrastTransferFunction::new(50.0, 1.2, 1.2, &beam, 0.5, 0.3);
        let df_s = ctf.scherzer_defocus();
        assert!(df_s > 0.0, "Scherzer defocus must be positive");
        // For 200 kV, Cs=1.2mm: ~40-60 nm
        assert!(df_s > 20.0 && df_s < 200.0, "Scherzer defocus = {df_s} nm");
    }

    #[test]
    fn test_point_resolution() {
        let beam = ElectronBeam::new(200e3);
        let ctf = ContrastTransferFunction::new(50.0, 1.2, 1.2, &beam, 0.5, 0.3);
        let res = ctf.point_resolution();
        // For 200 kV, Cs=1.2mm: ~0.2-0.4 nm
        assert!(res > 0.1 && res < 1.0, "Point resolution = {res} nm");
    }

    #[test]
    fn test_thon_ring_zeros() {
        let beam = ElectronBeam::new(200e3);
        let ctf = ContrastTransferFunction::new(50.0, 1.2, 1.2, &beam, 0.0, 0.0);
        let zeros = ctf.thon_ring_zeros(10.0, 1000);
        assert!(!zeros.is_empty(), "Should find at least one Thon ring zero");
        // Zeros should be at increasing frequencies
        for i in 1..zeros.len() {
            assert!(zeros[i] > zeros[i - 1]);
        }
    }

    #[test]
    fn test_defocus_estimation_from_rings() {
        let lambda_nm = 0.00251; // 200 kV
        let df = 50.0; // nm defocus
        // Thon ring n zeros: u_n ≈ sqrt(n / (lambda * df))
        let rings: Vec<f64> = (1..=4)
            .map(|n| ((n as f64) / (lambda_nm * df)).sqrt())
            .collect();
        let estimated = ContrastTransferFunction::estimate_defocus_from_rings(lambda_nm, &rings);
        assert!(estimated.is_some());
        let est = estimated.unwrap();
        assert!((est - df).abs() / df < 0.05, "Defocus estimate {est} vs {df} nm");
    }

    // ─── ThicknessMeasurement tests ───────────────────────────────────────────

    #[test]
    fn test_thickness_log_ratio() {
        let meas = ThicknessMeasurement::new(100.0); // Al at 200 kV
        // If I_total = e * I_zero (t/lambda = 1), then t = 100 nm
        let t = meas.thickness_nm(std::f64::consts::E, 1.0);
        assert!((t - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_relative_thickness() {
        let meas = ThicknessMeasurement::new(100.0);
        // I_total = 2 * I_zero → t/lambda = ln(2)
        let rel = meas.relative_thickness(2.0, 1.0);
        assert!((rel - 2.0_f64.ln()).abs() < 1e-10);
    }

    #[test]
    fn test_thickness_zero_for_zero_input() {
        let meas = ThicknessMeasurement::new(100.0);
        assert_eq!(meas.thickness_nm(0.0, 1.0), 0.0);
        assert_eq!(meas.thickness_nm(1.0, 0.0), 0.0);
    }

    #[test]
    fn test_malis_mfp_aluminum() {
        let mfp = ThicknessMeasurement::malis_mfp(13.0, 200.0);
        // Malis formula gives ~100-300 nm for light elements at 200 kV
        assert!(mfp > 50.0 && mfp < 400.0, "Al MFP = {mfp} nm");
    }

    #[test]
    fn test_mfp_database() {
        assert!(MfpDatabase::mfp_200kv("Al").is_some());
        assert!(MfpDatabase::mfp_200kv("Si").is_some());
        assert!(MfpDatabase::mfp_200kv("unknown").is_none());
    }

    // ─── MaterialDatabase tests ───────────────────────────────────────────────

    #[test]
    fn test_material_aluminum() {
        let al = MaterialDatabase::get("al").unwrap();
        assert_eq!(al.formula, "Al");
        assert_eq!(al.structure, StructureType::Fcc);
        assert!((al.lattice.a - 4.049).abs() < 0.001);
    }

    #[test]
    fn test_material_silicon() {
        let si = MaterialDatabase::get("silicon").unwrap();
        assert_eq!(si.formula, "Si");
        assert_eq!(si.structure, StructureType::Diamond);
    }

    #[test]
    fn test_material_iron_bcc() {
        let fe = MaterialDatabase::get("iron").unwrap();
        assert_eq!(fe.structure, StructureType::Bcc);
    }

    #[test]
    fn test_material_titanium_hcp() {
        let ti = MaterialDatabase::get("ti").unwrap();
        assert_eq!(ti.structure, StructureType::Hcp);
        assert!((ti.lattice.a - 2.951).abs() < 0.001);
        assert!((ti.lattice.c - 4.686).abs() < 0.001);
    }

    #[test]
    fn test_material_not_found() {
        assert!(MaterialDatabase::get("unobtainium").is_none());
    }

    // ─── MoireAnalyzer tests ──────────────────────────────────────────────────

    #[test]
    fn test_parallel_moire() {
        // Two parallel lattices with d1=2.0 Å, d2=2.1 Å
        let d = MoireAnalyzer::parallel_moire(2.0, 2.1);
        // D = 2.0*2.1 / |2.1-2.0| = 4.2/0.1 = 42 Å
        assert!((d - 42.0).abs() < 0.001, "Parallel moiré = {d} Å");
    }

    #[test]
    fn test_rotation_moire_small_angle() {
        // Rotation moiré with d=2.0 Å, theta=1°
        let d_moire = MoireAnalyzer::rotation_moire(2.0, 1.0);
        // D ≈ d/theta (rad) = 2.0/(pi/180) ≈ 114.6 Å
        let expected = 2.0 / (1.0_f64.to_radians() / 1.0); // simplified small angle
        assert!(d_moire > 50.0 && d_moire < 200.0, "Rotation moiré = {d_moire} Å");
        let _ = expected;
    }

    #[test]
    fn test_moire_calculate_zero_rotation() {
        // Zero rotation: parallel moiré case
        let result = MoireAnalyzer::calculate(2.0, 2.1, 0.0);
        let parallel = MoireAnalyzer::parallel_moire(2.0, 2.1);
        assert!((result.period_angstrom - parallel).abs() < 0.001);
    }

    #[test]
    fn test_moire_calculate_90_degrees() {
        let result = MoireAnalyzer::calculate(2.0, 2.0, 90.0);
        // At 90°: D = d / sqrt(2)
        let expected = 2.0 / 2.0_f64.sqrt();
        assert!((result.period_angstrom - expected).abs() < 0.001);
    }

    // ─── LatticeFringeAnalyzer tests ──────────────────────────────────────────

    #[test]
    fn test_lattice_fringe_power_spectrum_size() {
        let analyzer = LatticeFringeAnalyzer::new(0.02);
        let n = 8;
        let image = vec![1.0; n * n];
        let ps = analyzer.power_spectrum(&image, n);
        assert_eq!(ps.len(), n * n);
    }

    #[test]
    fn test_lattice_parameter_from_fringe() {
        let analyzer = LatticeFringeAnalyzer::new(0.02);
        // For Si (111) d = 3.135 Å: a = d * sqrt(3) = 5.431 Å
        let a = analyzer.measure_lattice_parameter(3.135, 1, 1, 1);
        assert!((a - 5.431).abs() < 0.01, "Si a from (111) = {a} Å");
    }

    #[test]
    fn test_lattice_fringe_dc_dominant() {
        // Uniform image should have dominant DC component
        let analyzer = LatticeFringeAnalyzer::new(0.02);
        let n = 8;
        let image = vec![1.0; n * n];
        let ps = analyzer.power_spectrum(&image, n);
        let dc = ps[0]; // DC at (0,0)
        let max_other = ps[1..].iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(dc >= max_other, "DC component should dominate for uniform image");
    }

    // ─── ImageFilter tests ────────────────────────────────────────────────────

    #[test]
    fn test_wiener_filter_preserves_size() {
        let filt = ImageFilter::new(8, 0.02);
        let image = vec![1.0; 64];
        let result = filt.wiener_filter(&image, 10.0);
        assert_eq!(result.len(), 64);
    }

    #[test]
    fn test_wiener_filter_dc_preservation() {
        // Constant image → Wiener filter should preserve DC
        let filt = ImageFilter::new(4, 0.02);
        let image = vec![2.0; 16];
        let result = filt.wiener_filter(&image, 1000.0);
        // With high SNR, output should be close to input
        let mean_out: f64 = result.iter().sum::<f64>() / 16.0;
        let mean_in = 2.0;
        assert!((mean_out - mean_in).abs() < 0.1, "Mean {mean_out} vs {mean_in}");
    }

    #[test]
    fn test_bandpass_filter_preserves_size() {
        let filt = ImageFilter::new(8, 0.02);
        let image = vec![1.0; 64];
        let result = filt.bandpass_filter(&image, 1.0, 5.0);
        assert_eq!(result.len(), 64);
    }

    // ─── KikuchiAnalyzer tests ────────────────────────────────────────────────

    #[test]
    fn test_kikuchi_d_from_band_width() {
        let beam = ElectronBeam::new(200e3);
        let cal = CameraCalibration::new(1.0, &beam, 1e-5);
        let kik = KikuchiAnalyzer::new(cal, beam.clone());
        // Band width for Al (111) at 200 kV
        // theta_B = lambda / (2d) = 2.51e-12 / (2*2.338e-10) ≈ 5.37e-3 rad
        // width = 2*theta_B*L/px = 2*5.37e-3*1/1e-5 = 1074 pixels
        let d = kik.d_from_band_width(1074.0);
        assert!((d - 2.338).abs() < 0.05, "Kikuchi band gives d = {d} Å");
    }

    #[test]
    fn test_kikuchi_analyze_bands() {
        let beam = ElectronBeam::new(200e3);
        let cal = CameraCalibration::new(1.0, &beam, 1e-5);
        let kik = KikuchiAnalyzer::new(cal, beam.clone());
        let widths = vec![1074.0, 800.0];
        let azimuths = vec![0.0, 45.0];
        let bands = kik.analyze_bands(&widths, &azimuths);
        assert_eq!(bands.len(), 2);
        assert!(bands[0].d_spacing_angstrom > 0.0);
    }

    // ─── SAED tests ───────────────────────────────────────────────────────────

    #[test]
    fn test_saed_index_rings_aluminum() {
        let beam = ElectronBeam::new(200e3);
        let cal = CameraCalibration::new(1.0, &beam, 1e-5);
        let al = MaterialDatabase::aluminum();
        let analyzer = SaedAnalyzer::new(cal.clone(), al.clone());

        // Al (111) d = 2.338 Å → calculate expected radius
        let r_111 = cal.radius_pixels_from_d_spacing(2.338);
        let rings = analyzer.index_rings(&[r_111], 5.0);
        assert_eq!(rings.len(), 1);
        // Should index to (111) or equivalent
        assert!(rings[0].hkl.is_some(), "Should index ring at Al (111)");
    }

    #[test]
    fn test_saed_refine_lattice_parameter() {
        let beam = ElectronBeam::new(200e3);
        let cal = CameraCalibration::new(1.0, &beam, 1e-5);
        let al = MaterialDatabase::aluminum();
        let analyzer = SaedAnalyzer::new(cal.clone(), al.clone());

        let r_111 = cal.radius_pixels_from_d_spacing(2.338);
        let r_200 = cal.radius_pixels_from_d_spacing(2.0245);
        let rings = analyzer.index_rings(&[r_111, r_200], 5.0);
        let a = analyzer.refine_lattice_parameter(&rings);
        assert!(a.is_some());
        let a_val = a.unwrap();
        assert!((a_val - 4.049).abs() < 0.1, "Al lattice param = {a_val} Å");
    }

    // ─── ContrastEstimator tests ──────────────────────────────────────────────

    #[test]
    fn test_bright_field_attenuation() {
        // At t=0, intensity = 1 (no attenuation)
        let i = ContrastEstimator::bright_field_intensity(0.0, 0.01);
        assert!((i - 1.0).abs() < EPS);
        // At large thickness, intensity drops
        let i2 = ContrastEstimator::bright_field_intensity(100.0, 0.01);
        assert!(i2 < i);
    }

    #[test]
    fn test_mass_thickness_contrast_heavier() {
        // Higher Z → higher contrast
        let c_al = ContrastEstimator::mass_thickness_contrast(13.0, 2.70, 10.0, 27.0);
        let c_au = ContrastEstimator::mass_thickness_contrast(79.0, 19.32, 10.0, 197.0);
        assert!(c_au > c_al, "Au should have higher mass-thickness contrast");
    }

    #[test]
    fn test_allowed_reflections_al() {
        let al = MaterialDatabase::aluminum();
        let refs = al.lattice.allowed_reflections(5.0, 0.5);
        // Should include (111), (200), (220)
        let has_111 = refs.iter().any(|&(_, h, k, l)| {
            let (ah, ak, al) = (h.abs(), k.abs(), l.abs());
            (ah == 1 && ak == 1 && al == 1)
        });
        assert!(has_111, "Al FCC should have (111) reflection");
    }

    #[test]
    fn test_gcd_helper() {
        assert_eq!(gcd(12, 8), 4);
        assert_eq!(gcd(7, 3), 1);
        assert_eq!(gcd(0, 5), 5);
    }

    #[test]
    fn test_dft_impulse() {
        // DFT of impulse [1, 0, 0, 0] → constant [1, 1, 1, 1]
        let input = vec![(1.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)];
        let output = dft_1d(&input);
        for (re, im) in &output {
            assert!((re - 1.0).abs() < 1e-10);
            assert!(im.abs() < 1e-10);
        }
    }

    #[test]
    fn test_dft_dc() {
        // DFT of constant [A, A, A, A] → [4A, 0, 0, 0]
        let n = 4;
        let a = 2.5;
        let input: Vec<(f64, f64)> = vec![(a, 0.0); n];
        let output = dft_1d(&input);
        assert!((output[0].0 - n as f64 * a).abs() < 1e-10);
        for k in 1..n {
            assert!(output[k].0.abs() < 1e-10);
            assert!(output[k].1.abs() < 1e-10);
        }
    }

    #[test]
    fn test_fftshift_roundtrip() {
        // Two-shift should return original
        let n = 4;
        let data: Vec<f64> = (0..n * n).map(|i| i as f64).collect();
        let shifted = fftshift(&data, n);
        let shifted2 = fftshift(&shifted, n);
        // For even N, double shift returns to original
        for (a, b) in data.iter().zip(shifted2.iter()) {
            assert!((a - b).abs() < 1e-10);
        }
    }
}
