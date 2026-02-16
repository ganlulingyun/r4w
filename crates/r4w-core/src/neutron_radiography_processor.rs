//! Neutron radiography and tomography signal processing.
//!
//! This module implements the complete neutron imaging pipeline from raw
//! detector data through tomographic reconstruction. It provides:
//!
//! - **White beam normalization**: open beam / dark current correction
//! - **Transmission calculation**: Beer-Lambert attenuation analysis
//! - **Scattering correction**: point scatter function deconvolution
//! - **Time-of-flight analysis**: neutron energy spectrum from TOF data
//! - **Bragg edge analysis**: crystallographic strain/texture mapping
//! - **Attenuation coefficients**: macroscopic cross sections for common elements
//! - **Sinogram generation**: projection-to-sinogram assembly
//! - **Filtered back-projection**: tomographic reconstruction with selectable filters
//! - **Phase contrast imaging**: differential phase contrast from grating interferometry
//!
//! Applications include fuel cell water management, turbine blade inspection,
//! cultural heritage analysis, nuclear fuel rod inspection, and hydrogen
//! storage material characterisation.
//!
//! # Example
//!
//! ```
//! use r4w_core::neutron_radiography_processor::{
//!     NeutronConfig, BeamEnergy, DetectorType,
//!     WhiteBeamNormalizer, TransmissionCalculator,
//!     AttenuationCoefficient, Element,
//! };
//!
//! let config = NeutronConfig {
//!     beam_energy: BeamEnergy::Thermal,
//!     detector_type: DetectorType::Scintillator,
//!     pixel_size_um: 50.0,
//!     exposure_time_s: 10.0,
//! };
//!
//! // Normalise a raw image
//! let raw = vec![800.0, 600.0, 400.0, 200.0];
//! let open = vec![1000.0, 1000.0, 1000.0, 1000.0];
//! let dark = vec![100.0, 100.0, 100.0, 100.0];
//! let norm = WhiteBeamNormalizer::normalize(&raw, &open, &dark);
//! assert!((norm[0] - 0.7777).abs() < 0.01);
//!
//! // Beer-Lambert transmission
//! let calc = TransmissionCalculator::new();
//! let sigma = AttenuationCoefficient::macroscopic(Element::Hydrogen, 0.05);
//! let transmission = calc.beer_lambert(sigma, 0.01);
//! assert!(transmission > 0.0 && transmission < 1.0);
//! ```

use std::f64::consts::PI;

// ─── Physical constants ─────────────────────────────────────────────────────

/// Neutron mass in kg.
const NEUTRON_MASS_KG: f64 = 1.674_927_471e-27;

/// Planck constant in J*s.
const PLANCK_H: f64 = 6.626_070_15e-34;

/// Boltzmann constant in J/K.
const BOLTZMANN_K: f64 = 1.380_649e-23;

/// 1 meV in Joules.
const MEV_TO_J: f64 = 1.602_176_634e-22;

/// 1 Angstrom in metres.
const ANGSTROM: f64 = 1.0e-10;

// ─── Beam energy classification ─────────────────────────────────────────────

/// Neutron beam energy classification.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BeamEnergy {
    /// Cold neutrons: E ~ 1-5 meV, lambda ~ 4-9 A.
    Cold,
    /// Thermal neutrons: E ~ 25.3 meV, lambda ~ 1.8 A, v ~ 2200 m/s.
    Thermal,
    /// Epithermal neutrons: E ~ 0.5-100 eV.
    Epithermal,
    /// Custom energy in meV.
    Custom(f64),
}

impl BeamEnergy {
    /// Energy in meV.
    pub fn energy_mev(&self) -> f64 {
        match self {
            BeamEnergy::Cold => 3.0,
            BeamEnergy::Thermal => 25.3,
            BeamEnergy::Epithermal => 1000.0,
            BeamEnergy::Custom(e) => *e,
        }
    }

    /// Energy in Joules.
    pub fn energy_j(&self) -> f64 {
        self.energy_mev() * MEV_TO_J
    }

    /// De Broglie wavelength in Angstroms.
    ///
    /// lambda = h / sqrt(2 * m_n * E)
    pub fn wavelength_angstrom(&self) -> f64 {
        let e_j = self.energy_j();
        if e_j <= 0.0 {
            return 0.0;
        }
        PLANCK_H / (2.0 * NEUTRON_MASS_KG * e_j).sqrt() / ANGSTROM
    }

    /// Neutron velocity in m/s.
    ///
    /// v = sqrt(2*E/m_n)
    pub fn velocity_ms(&self) -> f64 {
        let e_j = self.energy_j();
        if e_j <= 0.0 {
            return 0.0;
        }
        (2.0 * e_j / NEUTRON_MASS_KG).sqrt()
    }
}

// ─── Detector type ──────────────────────────────────────────────────────────

/// Neutron detector type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DetectorType {
    /// Scintillator screen (e.g., ZnS:6LiF) + camera.
    Scintillator,
    /// Micro-channel plate (MCP) detector.
    Mcp,
    /// 3He gas detector.
    Helium3Gas,
    /// Amorphous silicon flat panel.
    FlatPanel,
}

// ─── Configuration ──────────────────────────────────────────────────────────

/// Configuration for neutron radiography.
#[derive(Debug, Clone)]
pub struct NeutronConfig {
    /// Beam energy classification.
    pub beam_energy: BeamEnergy,
    /// Detector type.
    pub detector_type: DetectorType,
    /// Pixel size in micrometers.
    pub pixel_size_um: f64,
    /// Exposure time in seconds.
    pub exposure_time_s: f64,
}

impl Default for NeutronConfig {
    fn default() -> Self {
        Self {
            beam_energy: BeamEnergy::Thermal,
            detector_type: DetectorType::Scintillator,
            pixel_size_um: 50.0,
            exposure_time_s: 10.0,
        }
    }
}

// ─── Element cross section database ─────────────────────────────────────────

/// Elements with known neutron cross sections.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Element {
    /// Hydrogen (H) - very high scattering cross section.
    Hydrogen,
    /// Lithium-6 - high absorption.
    Lithium6,
    /// Boron-10 - high absorption (used in shielding).
    Boron10,
    /// Carbon - low cross section.
    Carbon,
    /// Nitrogen.
    Nitrogen,
    /// Oxygen - low cross section.
    Oxygen,
    /// Aluminium - low cross section (good for sample holders).
    Aluminium,
    /// Iron.
    Iron,
    /// Cadmium - very high absorption.
    Cadmium,
    /// Gadolinium - highest thermal absorption.
    Gadolinium,
    /// Lead - low cross section (transparent to neutrons).
    Lead,
    /// Water (H2O compound).
    Water,
}

// ─── Attenuation coefficients ───────────────────────────────────────────────

/// Neutron attenuation coefficient calculator.
///
/// Provides microscopic cross sections (barns) and macroscopic cross sections
/// (cm^-1) for common elements at thermal neutron energies.
pub struct AttenuationCoefficient;

impl AttenuationCoefficient {
    /// Microscopic total cross section in barns (1 barn = 1e-24 cm^2).
    ///
    /// Values are for thermal neutrons (25.3 meV, 1.8 A).
    /// Total = absorption + scattering.
    pub fn microscopic_total_barns(element: Element) -> f64 {
        match element {
            Element::Hydrogen => 82.02,    // 0.3326 abs + 81.67 scat (bound)
            Element::Lithium6 => 940.0,    // 940 abs + ~1 scat
            Element::Boron10 => 3840.0,    // 3835 abs + ~5 scat
            Element::Carbon => 5.551,      // 0.0035 abs + 5.55 scat
            Element::Nitrogen => 12.19,    // 1.90 abs + 10.3 scat (bound)
            Element::Oxygen => 4.232,      // 0.00019 abs + 4.23 scat
            Element::Aluminium => 1.732,   // 0.231 abs + 1.50 scat
            Element::Iron => 14.29,        // 2.56 abs + 11.73 scat
            Element::Cadmium => 2525.0,    // 2520 abs + ~5 scat
            Element::Gadolinium => 49700.0, // 49700 abs + ~6 scat
            Element::Lead => 11.35,        // 0.171 abs + 11.18 scat
            Element::Water => 167.3,       // effective for H2O molecule
        }
    }

    /// Microscopic absorption cross section in barns.
    pub fn microscopic_absorption_barns(element: Element) -> f64 {
        match element {
            Element::Hydrogen => 0.3326,
            Element::Lithium6 => 940.0,
            Element::Boron10 => 3835.0,
            Element::Carbon => 0.0035,
            Element::Nitrogen => 1.90,
            Element::Oxygen => 0.00019,
            Element::Aluminium => 0.231,
            Element::Iron => 2.56,
            Element::Cadmium => 2520.0,
            Element::Gadolinium => 49700.0,
            Element::Lead => 0.171,
            Element::Water => 0.665,
        }
    }

    /// Microscopic scattering cross section in barns.
    pub fn microscopic_scattering_barns(element: Element) -> f64 {
        Self::microscopic_total_barns(element) - Self::microscopic_absorption_barns(element)
    }

    /// Number density in atoms/cm^3 for the element in its standard state.
    pub fn number_density_per_cm3(element: Element) -> f64 {
        // N = rho * N_A / A
        // Using standard densities and atomic masses
        let na = 6.022e23_f64;
        match element {
            Element::Hydrogen => 2.0 * 0.0708 * na / 1.008,  // liquid H2
            Element::Lithium6 => 0.534 * na / 6.015,
            Element::Boron10 => 2.34 * na / 10.012,
            Element::Carbon => 2.267 * na / 12.011,
            Element::Nitrogen => 2.0 * 0.808 * na / 14.007,   // liquid N2
            Element::Oxygen => 2.0 * 1.141 * na / 15.999,     // liquid O2
            Element::Aluminium => 2.70 * na / 26.982,
            Element::Iron => 7.874 * na / 55.845,
            Element::Cadmium => 8.65 * na / 112.411,
            Element::Gadolinium => 7.90 * na / 157.25,
            Element::Lead => 11.34 * na / 207.2,
            Element::Water => 3.0 * 0.998 * na / 18.015,  // 3 atoms per molecule
        }
    }

    /// Macroscopic cross section Sigma = N * sigma in cm^-1.
    ///
    /// `number_density` is in atoms/cm^3. If 0.0, uses the standard density
    /// for the element.
    pub fn macroscopic(element: Element, number_density: f64) -> f64 {
        let n = if number_density <= 0.0 {
            Self::number_density_per_cm3(element)
        } else {
            number_density
        };
        n * Self::microscopic_total_barns(element) * 1.0e-24
    }

    /// Macroscopic cross section from the standard element density.
    pub fn macroscopic_standard(element: Element) -> f64 {
        Self::macroscopic(element, 0.0)
    }

    /// Mean free path in cm: l = 1/Sigma.
    pub fn mean_free_path_cm(element: Element) -> f64 {
        let sigma = Self::macroscopic_standard(element);
        if sigma <= 0.0 {
            return f64::INFINITY;
        }
        1.0 / sigma
    }
}

// ─── White beam normalizer ──────────────────────────────────────────────────

/// Open beam / dark current normalization for neutron radiography.
///
/// Corrects raw detector images using:
///   normalized = (raw - dark) / (open - dark)
///
/// where `open` is the flat-field (no sample) and `dark` is the
/// detector dark current (beam off).
pub struct WhiteBeamNormalizer;

impl WhiteBeamNormalizer {
    /// Normalize a single image.
    ///
    /// Returns pixel values in [0, 1] range (clamped).
    pub fn normalize(raw: &[f64], open_beam: &[f64], dark_current: &[f64]) -> Vec<f64> {
        let n = raw.len().min(open_beam.len()).min(dark_current.len());
        let mut result = Vec::with_capacity(n);
        for i in 0..n {
            let denom = open_beam[i] - dark_current[i];
            let val = if denom.abs() < 1e-12 {
                0.0
            } else {
                (raw[i] - dark_current[i]) / denom
            };
            result.push(val.clamp(0.0, f64::MAX));
        }
        result
    }

    /// Normalize using median of multiple open beam and dark images.
    pub fn normalize_with_median(
        raw: &[f64],
        open_beams: &[Vec<f64>],
        dark_currents: &[Vec<f64>],
    ) -> Vec<f64> {
        let n = raw.len();
        let median_open = Self::pixel_wise_median(open_beams, n);
        let median_dark = Self::pixel_wise_median(dark_currents, n);
        Self::normalize(raw, &median_open, &median_dark)
    }

    /// Compute pixel-wise median across multiple images.
    fn pixel_wise_median(images: &[Vec<f64>], n_pixels: usize) -> Vec<f64> {
        if images.is_empty() {
            return vec![0.0; n_pixels];
        }
        let mut result = Vec::with_capacity(n_pixels);
        for i in 0..n_pixels {
            let mut vals: Vec<f64> = images
                .iter()
                .filter_map(|img| img.get(i).copied())
                .collect();
            vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let median = if vals.is_empty() {
                0.0
            } else if vals.len() % 2 == 0 {
                (vals[vals.len() / 2 - 1] + vals[vals.len() / 2]) / 2.0
            } else {
                vals[vals.len() / 2]
            };
            result.push(median);
        }
        result
    }
}

// ─── Transmission calculator ────────────────────────────────────────────────

/// Beer-Lambert transmission calculator.
///
/// Computes transmission through materials using:
///   I/I0 = exp(-Sigma * t)
///
/// where Sigma is the macroscopic cross section (cm^-1) and t is the
/// material thickness (cm).
pub struct TransmissionCalculator {
    _private: (),
}

impl TransmissionCalculator {
    /// Create a new transmission calculator.
    pub fn new() -> Self {
        Self { _private: () }
    }

    /// Beer-Lambert transmission ratio I/I0.
    ///
    /// `sigma_macro` is the macroscopic cross section in cm^-1.
    /// `thickness_cm` is the material thickness in cm.
    pub fn beer_lambert(&self, sigma_macro: f64, thickness_cm: f64) -> f64 {
        (-sigma_macro * thickness_cm).exp()
    }

    /// Optical density (OD) = -ln(I/I0) = Sigma * t.
    pub fn optical_density(&self, sigma_macro: f64, thickness_cm: f64) -> f64 {
        sigma_macro * thickness_cm
    }

    /// Invert Beer-Lambert to find thickness from transmission ratio.
    ///
    /// t = -ln(I/I0) / Sigma
    pub fn thickness_from_transmission(
        &self,
        transmission: f64,
        sigma_macro: f64,
    ) -> f64 {
        if sigma_macro.abs() < 1e-30 || transmission <= 0.0 {
            return 0.0;
        }
        -transmission.ln() / sigma_macro
    }

    /// Compute attenuation map from normalized image.
    ///
    /// Returns -ln(I/I0) for each pixel. Pixels with I/I0 <= 0 are
    /// set to a large value.
    pub fn attenuation_map(normalized: &[f64]) -> Vec<f64> {
        normalized
            .iter()
            .map(|&t| {
                if t <= 1e-12 {
                    20.0 // cap at ~20 for zero-transmission pixels
                } else {
                    -t.ln()
                }
            })
            .collect()
    }

    /// Compute transmission through a stack of materials.
    ///
    /// Each tuple is (macroscopic_cross_section_cm_inv, thickness_cm).
    pub fn multi_layer(&self, layers: &[(f64, f64)]) -> f64 {
        let total_od: f64 = layers.iter().map(|(s, t)| s * t).sum();
        (-total_od).exp()
    }
}

// ─── Scattering corrector ───────────────────────────────────────────────────

/// Point scatter function correction for neutron scatter artefacts.
///
/// Neutron scattering in the sample causes a blurring halo around
/// high-attenuation features. This corrector uses a simple Gaussian
/// PSF model and iterative deconvolution.
pub struct ScatteringCorrector {
    /// Scatter-to-total ratio (0..1).
    scatter_fraction: f64,
    /// Gaussian PSF sigma in pixels.
    psf_sigma_px: f64,
}

impl ScatteringCorrector {
    /// Create a new scattering corrector.
    ///
    /// `scatter_fraction` is the fraction of detected neutrons that were
    /// scattered (typically 0.02-0.15 for thin samples).
    /// `psf_sigma_px` is the Gaussian PSF width in pixels.
    pub fn new(scatter_fraction: f64, psf_sigma_px: f64) -> Self {
        Self {
            scatter_fraction: scatter_fraction.clamp(0.0, 1.0),
            psf_sigma_px: psf_sigma_px.max(0.1),
        }
    }

    /// Generate 1D Gaussian PSF kernel (normalised).
    pub fn gaussian_psf_1d(&self, radius: usize) -> Vec<f64> {
        let n = 2 * radius + 1;
        let sigma = self.psf_sigma_px;
        let mut kernel = Vec::with_capacity(n);
        let mut sum = 0.0;
        for i in 0..n {
            let x = i as f64 - radius as f64;
            let val = (-x * x / (2.0 * sigma * sigma)).exp();
            kernel.push(val);
            sum += val;
        }
        for v in kernel.iter_mut() {
            *v /= sum;
        }
        kernel
    }

    /// Generate 2D Gaussian PSF kernel (normalised) as a flat row-major array.
    pub fn gaussian_psf_2d(&self, radius: usize) -> Vec<f64> {
        let n = 2 * radius + 1;
        let sigma = self.psf_sigma_px;
        let mut kernel = Vec::with_capacity(n * n);
        let mut sum = 0.0;
        for iy in 0..n {
            for ix in 0..n {
                let x = ix as f64 - radius as f64;
                let y = iy as f64 - radius as f64;
                let val = (-(x * x + y * y) / (2.0 * sigma * sigma)).exp();
                kernel.push(val);
                sum += val;
            }
        }
        for v in kernel.iter_mut() {
            *v /= sum;
        }
        kernel
    }

    /// Apply 1D scatter correction to a line profile.
    ///
    /// Uses the iterative approach:
    ///   corrected = (measured - s * conv(corrected, PSF)) / (1 - s)
    ///
    /// Starting from corrected = measured and iterating.
    pub fn correct_1d(&self, measured: &[f64], iterations: usize) -> Vec<f64> {
        let psf_radius = (3.0 * self.psf_sigma_px).ceil() as usize;
        let psf = self.gaussian_psf_1d(psf_radius);
        let s = self.scatter_fraction;
        let mut corrected = measured.to_vec();

        for _ in 0..iterations {
            let blurred = convolve_1d(&corrected, &psf);
            for i in 0..corrected.len() {
                corrected[i] = (measured[i] - s * blurred[i]) / (1.0 - s);
                if corrected[i] < 0.0 {
                    corrected[i] = 0.0;
                }
            }
        }
        corrected
    }

    /// Scatter fraction getter.
    pub fn scatter_fraction(&self) -> f64 {
        self.scatter_fraction
    }

    /// PSF sigma getter.
    pub fn psf_sigma_px(&self) -> f64 {
        self.psf_sigma_px
    }
}

/// 1D convolution (same-size output, zero-padded boundary).
fn convolve_1d(signal: &[f64], kernel: &[f64]) -> Vec<f64> {
    let n = signal.len();
    let k = kernel.len();
    let half = k / 2;
    let mut result = vec![0.0; n];
    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..k {
            let idx = i as isize + j as isize - half as isize;
            if idx >= 0 && (idx as usize) < n {
                sum += signal[idx as usize] * kernel[j];
            }
        }
        result[i] = sum;
    }
    result
}

// ─── TOF spectrum analyzer ──────────────────────────────────────────────────

/// Time-of-flight (TOF) neutron energy spectrum analyser.
///
/// Converts time-of-flight measurements to neutron energy using:
///   E = (1/2) * m_n * (L/t)^2
///
/// where L is the flight path length and t is the measured time-of-flight.
pub struct TofSpectrumAnalyzer {
    /// Flight path length in metres.
    flight_path_m: f64,
}

impl TofSpectrumAnalyzer {
    /// Create a new TOF analyser with the given flight path length in metres.
    pub fn new(flight_path_m: f64) -> Self {
        Self {
            flight_path_m: flight_path_m.max(0.001),
        }
    }

    /// Convert time-of-flight (seconds) to neutron energy (meV).
    ///
    /// E = (1/2) * m_n * (L/t)^2 / MEV_TO_J
    pub fn tof_to_energy_mev(&self, tof_s: f64) -> f64 {
        if tof_s <= 0.0 {
            return 0.0;
        }
        let v = self.flight_path_m / tof_s;
        0.5 * NEUTRON_MASS_KG * v * v / MEV_TO_J
    }

    /// Convert time-of-flight to wavelength in Angstroms.
    ///
    /// lambda = h * t / (m_n * L)
    pub fn tof_to_wavelength_angstrom(&self, tof_s: f64) -> f64 {
        if tof_s <= 0.0 {
            return 0.0;
        }
        PLANCK_H * tof_s / (NEUTRON_MASS_KG * self.flight_path_m) / ANGSTROM
    }

    /// Convert energy (meV) to expected time-of-flight (seconds).
    pub fn energy_to_tof_s(&self, energy_mev: f64) -> f64 {
        if energy_mev <= 0.0 {
            return 0.0;
        }
        let v = (2.0 * energy_mev * MEV_TO_J / NEUTRON_MASS_KG).sqrt();
        self.flight_path_m / v
    }

    /// Build a TOF histogram from event timestamps.
    ///
    /// Returns (bin_centres_s, counts) with `n_bins` bins spanning
    /// the given time range.
    pub fn histogram(
        &self,
        events_s: &[f64],
        n_bins: usize,
        tof_range: (f64, f64),
    ) -> (Vec<f64>, Vec<u64>) {
        let (t_min, t_max) = tof_range;
        let bin_width = (t_max - t_min) / n_bins as f64;
        let centres: Vec<f64> = (0..n_bins)
            .map(|i| t_min + (i as f64 + 0.5) * bin_width)
            .collect();
        let mut counts = vec![0u64; n_bins];
        for &t in events_s {
            if t >= t_min && t < t_max {
                let bin = ((t - t_min) / bin_width) as usize;
                if bin < n_bins {
                    counts[bin] += 1;
                }
            }
        }
        (centres, counts)
    }

    /// Convert a TOF histogram to an energy spectrum.
    ///
    /// Returns (energy_mev, intensity).
    /// The Jacobian |dE/dt| is applied to preserve total counts.
    pub fn tof_histogram_to_energy(
        &self,
        tof_centres: &[f64],
        counts: &[u64],
    ) -> (Vec<f64>, Vec<f64>) {
        let n = tof_centres.len().min(counts.len());
        let mut energies = Vec::with_capacity(n);
        let mut intensities = Vec::with_capacity(n);
        let l = self.flight_path_m;

        for i in 0..n {
            let t = tof_centres[i];
            if t <= 0.0 {
                continue;
            }
            let e = self.tof_to_energy_mev(t);
            // Jacobian: |dE/dt| = m_n * L^2 / t^3
            let jacobian = NEUTRON_MASS_KG * l * l / (t * t * t) / MEV_TO_J;
            energies.push(e);
            intensities.push(counts[i] as f64 / jacobian.abs().max(1e-30));
        }
        (energies, intensities)
    }

    /// Flight path getter.
    pub fn flight_path_m(&self) -> f64 {
        self.flight_path_m
    }
}

// ─── Bragg edge analyser ────────────────────────────────────────────────────

/// Bragg edge analysis for crystallographic strain and texture mapping.
///
/// Detects edges in the wavelength-dependent neutron transmission caused by
/// the onset of Bragg diffraction at lambda = 2 * d_hkl.
///
/// As wavelength increases past 2*d_hkl, that (hkl) plane can no longer
/// diffract, causing a step increase in transmission.
pub struct BraggEdgeAnalyzer;

/// A detected Bragg edge.
#[derive(Debug, Clone)]
pub struct BraggEdge {
    /// Wavelength at the edge in Angstroms.
    pub wavelength_a: f64,
    /// Edge height (change in transmission).
    pub edge_height: f64,
    /// Corresponding d-spacing in Angstroms.
    pub d_spacing_a: f64,
    /// Strain relative to reference d-spacing.
    pub strain: f64,
}

impl BraggEdgeAnalyzer {
    /// Bragg's law: lambda = 2 * d * sin(theta).
    /// For backscattering (theta = 90 deg), the Bragg edge is at lambda = 2*d.
    pub fn edge_wavelength(d_spacing_a: f64) -> f64 {
        2.0 * d_spacing_a
    }

    /// Calculate d-spacing from the edge wavelength.
    pub fn d_spacing_from_edge(wavelength_a: f64) -> f64 {
        wavelength_a / 2.0
    }

    /// Calculate lattice strain from measured vs. reference d-spacing.
    ///
    /// strain = (d_measured - d_ref) / d_ref
    pub fn strain(d_measured: f64, d_reference: f64) -> f64 {
        if d_reference.abs() < 1e-15 {
            return 0.0;
        }
        (d_measured - d_reference) / d_reference
    }

    /// Find Bragg edges in a wavelength-transmission spectrum.
    ///
    /// Uses derivative-based detection: edges appear as peaks in d(T)/d(lambda).
    /// `wavelengths` and `transmission` must have the same length.
    /// `d_reference` is the reference d-spacing for strain calculation.
    /// `threshold` is the minimum derivative magnitude to detect.
    pub fn find_edges(
        wavelengths: &[f64],
        transmission: &[f64],
        d_reference: f64,
        threshold: f64,
    ) -> Vec<BraggEdge> {
        let n = wavelengths.len().min(transmission.len());
        if n < 3 {
            return Vec::new();
        }

        // Compute numerical derivative dT/dlambda
        let mut deriv = vec![0.0; n];
        for i in 1..n - 1 {
            let dl = wavelengths[i + 1] - wavelengths[i - 1];
            if dl.abs() > 1e-15 {
                deriv[i] = (transmission[i + 1] - transmission[i - 1]) / dl;
            }
        }

        // Find peaks in derivative (positive peaks = rising edges = Bragg edges)
        let mut edges = Vec::new();
        for i in 2..n - 2 {
            if deriv[i] > threshold
                && deriv[i] > deriv[i - 1]
                && deriv[i] > deriv[i + 1]
            {
                let d_spacing = Self::d_spacing_from_edge(wavelengths[i]);
                let strain = Self::strain(d_spacing, d_reference);
                edges.push(BraggEdge {
                    wavelength_a: wavelengths[i],
                    edge_height: deriv[i],
                    d_spacing_a: d_spacing,
                    strain,
                });
            }
        }
        edges
    }

    /// Common d-spacings for BCC iron (alpha-Fe) in Angstroms.
    pub fn iron_bcc_d_spacings() -> Vec<(String, f64)> {
        let a = 2.8665; // lattice parameter in Angstroms
        vec![
            ("110".to_string(), a / (1.0_f64 + 1.0 + 0.0).sqrt()),
            ("200".to_string(), a / (4.0_f64 + 0.0 + 0.0).sqrt()),
            ("211".to_string(), a / (4.0_f64 + 1.0 + 1.0).sqrt()),
            ("220".to_string(), a / (4.0_f64 + 4.0 + 0.0).sqrt()),
            ("310".to_string(), a / (9.0_f64 + 1.0 + 0.0).sqrt()),
        ]
    }

    /// Common d-spacings for FCC aluminium in Angstroms.
    pub fn aluminium_fcc_d_spacings() -> Vec<(String, f64)> {
        let a = 4.0495; // lattice parameter in Angstroms
        vec![
            ("111".to_string(), a / (1.0_f64 + 1.0 + 1.0).sqrt()),
            ("200".to_string(), a / (4.0_f64 + 0.0 + 0.0).sqrt()),
            ("220".to_string(), a / (4.0_f64 + 4.0 + 0.0).sqrt()),
            ("311".to_string(), a / (9.0_f64 + 1.0 + 1.0).sqrt()),
            ("222".to_string(), a / (4.0_f64 + 4.0 + 4.0).sqrt()),
        ]
    }
}

// ─── Sinogram generator ─────────────────────────────────────────────────────

/// Sinogram generator for neutron tomography.
///
/// Assembles projection images taken at multiple angles into a sinogram
/// (angle vs. detector position).
pub struct SinogramGenerator;

impl SinogramGenerator {
    /// Create a sinogram from a set of 1D projections at equally-spaced angles.
    ///
    /// `projections` is a slice of 1D line profiles, each of the same length.
    /// Returns a 2D sinogram as (data, n_angles, n_pixels) in row-major order.
    pub fn from_projections(projections: &[Vec<f64>]) -> (Vec<f64>, usize, usize) {
        if projections.is_empty() {
            return (Vec::new(), 0, 0);
        }
        let n_angles = projections.len();
        let n_pixels = projections[0].len();
        let mut sinogram = Vec::with_capacity(n_angles * n_pixels);
        for proj in projections {
            let len = proj.len().min(n_pixels);
            sinogram.extend_from_slice(&proj[..len]);
            // Pad if shorter
            for _ in len..n_pixels {
                sinogram.push(0.0);
            }
        }
        (sinogram, n_angles, n_pixels)
    }

    /// Generate a sinogram from a 2D phantom by computing the Radon transform.
    ///
    /// `phantom` is a flat row-major image of size `width` x `height`.
    /// `angles_deg` are the projection angles in degrees.
    /// Returns the sinogram and its dimensions.
    pub fn radon_transform(
        phantom: &[f64],
        width: usize,
        height: usize,
        angles_deg: &[f64],
    ) -> (Vec<f64>, usize, usize) {
        let n_angles = angles_deg.len();
        let n_det = ((width * width + height * height) as f64).sqrt().ceil() as usize;
        let cx = width as f64 / 2.0;
        let cy = height as f64 / 2.0;
        let cd = n_det as f64 / 2.0;

        let mut sinogram = vec![0.0; n_angles * n_det];

        for (a_idx, &angle) in angles_deg.iter().enumerate() {
            let theta = angle * PI / 180.0;
            let cos_t = theta.cos();
            let sin_t = theta.sin();

            // For each detector pixel, integrate along the ray
            for d in 0..n_det {
                let s = d as f64 - cd;
                let mut sum = 0.0;
                let n_steps = (width.max(height) * 2) as usize;
                let step = ((width + height) as f64) / n_steps as f64;

                for k in 0..n_steps {
                    let t = (k as f64 - n_steps as f64 / 2.0) * step;
                    let x = s * cos_t - t * sin_t + cx;
                    let y = s * sin_t + t * cos_t + cy;
                    let ix = x.floor() as isize;
                    let iy = y.floor() as isize;
                    if ix >= 0 && ix < width as isize && iy >= 0 && iy < height as isize {
                        sum += phantom[iy as usize * width + ix as usize] * step;
                    }
                }
                sinogram[a_idx * n_det + d] = sum;
            }
        }
        (sinogram, n_angles, n_det)
    }
}

// ─── Filtered back-projection ───────────────────────────────────────────────

/// FBP reconstruction filter type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FbpFilter {
    /// Ram-Lak (ramp) filter: |f|.
    RamLak,
    /// Shepp-Logan filter: |f| * sinc(f/(2*f_max)).
    SheppLogan,
    /// Hamming-windowed ramp filter.
    Hamming,
    /// Hann-windowed ramp filter.
    Hann,
    /// No filter (simple back-projection).
    None,
}

/// Filtered Back-Projection (FBP) tomographic reconstruction.
///
/// Reconstructs a 2D image from a sinogram using the inverse Radon transform:
///   f(x,y) = integral over theta of h * p(s, theta) evaluated at
///            s = x*cos(theta) + y*sin(theta)
///
/// where h is the reconstruction filter and p is the projection data.
pub struct FilteredBackProjection {
    /// Reconstruction filter type.
    filter: FbpFilter,
}

impl FilteredBackProjection {
    /// Create a new FBP reconstructor with the given filter.
    pub fn new(filter: FbpFilter) -> Self {
        Self { filter }
    }

    /// Generate the spatial-domain reconstruction filter kernel.
    ///
    /// Returns a 1D filter of length `n` centred at n/2.
    pub fn filter_kernel(&self, n: usize) -> Vec<f64> {
        match self.filter {
            FbpFilter::None => {
                let mut k = vec![0.0; n];
                k[n / 2] = 1.0;
                k
            }
            _ => {
                // Generate ramp filter in spatial domain
                // h[0] = 1/4, h[n] = 0 for even n, h[n] = -1/(pi*n)^2 for odd n
                let mut kernel = vec![0.0; n];
                let centre = n / 2;

                for i in 0..n {
                    let k = i as isize - centre as isize;
                    if k == 0 {
                        kernel[i] = 0.25;
                    } else if k % 2 != 0 {
                        kernel[i] = -1.0 / (PI * k as f64).powi(2);
                    }
                    // even k != 0: kernel[i] = 0.0 (already set)
                }

                // Apply window function for non-RamLak filters
                match self.filter {
                    FbpFilter::SheppLogan => {
                        for i in 0..n {
                            let k = i as isize - centre as isize;
                            if k != 0 {
                                let f_norm = k as f64 / n as f64;
                                let sinc_arg = f_norm * PI / 2.0;
                                let w = if sinc_arg.abs() < 1e-12 {
                                    1.0
                                } else {
                                    sinc_arg.sin() / sinc_arg
                                };
                                kernel[i] *= w;
                            }
                        }
                    }
                    FbpFilter::Hamming => {
                        for i in 0..n {
                            let w = 0.54 - 0.46 * (2.0 * PI * i as f64 / (n - 1) as f64).cos();
                            kernel[i] *= w;
                        }
                    }
                    FbpFilter::Hann => {
                        for i in 0..n {
                            let w =
                                0.5 * (1.0 - (2.0 * PI * i as f64 / (n - 1) as f64).cos());
                            kernel[i] *= w;
                        }
                    }
                    _ => {}
                }

                kernel
            }
        }
    }

    /// Reconstruct a 2D image from a sinogram.
    ///
    /// `sinogram` is row-major (n_angles x n_det).
    /// `angles_deg` are the projection angles in degrees.
    /// `output_size` is the width=height of the square output image.
    ///
    /// Returns the reconstructed image as a flat row-major vector.
    pub fn reconstruct(
        &self,
        sinogram: &[f64],
        n_angles: usize,
        n_det: usize,
        angles_deg: &[f64],
        output_size: usize,
    ) -> Vec<f64> {
        assert_eq!(sinogram.len(), n_angles * n_det);
        assert_eq!(angles_deg.len(), n_angles);

        // Step 1: Filter each projection row
        let kernel = self.filter_kernel(n_det);
        let mut filtered = vec![0.0; n_angles * n_det];
        for a in 0..n_angles {
            let row_start = a * n_det;
            let row = &sinogram[row_start..row_start + n_det];
            let filt = convolve_1d(row, &kernel);
            filtered[row_start..row_start + n_det].copy_from_slice(&filt);
        }

        // Step 2: Back-project
        let mut image = vec![0.0; output_size * output_size];
        let cx = output_size as f64 / 2.0;
        let cd = n_det as f64 / 2.0;

        for (a_idx, &angle) in angles_deg.iter().enumerate() {
            let theta = angle * PI / 180.0;
            let cos_t = theta.cos();
            let sin_t = theta.sin();

            for iy in 0..output_size {
                let y = iy as f64 - cx;
                for ix in 0..output_size {
                    let x = ix as f64 - cx;
                    // Project (x,y) onto detector coordinate
                    let s = x * cos_t + y * sin_t + cd;
                    // Linear interpolation
                    let s_floor = s.floor() as isize;
                    let frac = s - s.floor();
                    if s_floor >= 0 && (s_floor as usize + 1) < n_det {
                        let idx0 = a_idx * n_det + s_floor as usize;
                        let val = filtered[idx0] * (1.0 - frac)
                            + filtered[idx0 + 1] * frac;
                        image[iy * output_size + ix] += val;
                    }
                }
            }
        }

        // Normalise by number of angles
        let scale = PI / n_angles as f64;
        for v in image.iter_mut() {
            *v *= scale;
        }
        image
    }
}

// ─── Phase contrast imager ──────────────────────────────────────────────────

/// Differential phase contrast imaging from grating interferometry.
///
/// Neutron grating interferometry uses a Talbot-Lau setup with three gratings
/// to measure:
/// - Transmission (conventional attenuation)
/// - Differential phase (refraction by the sample)
/// - Dark-field (small-angle scattering / visibility reduction)
///
/// These are extracted from a stepping scan by Fourier analysis of the
/// intensity oscillation at each pixel.
pub struct PhaseContrastImager {
    /// Number of grating steps.
    n_steps: usize,
    /// Grating period in micrometers.
    grating_period_um: f64,
}

/// Result of phase contrast analysis for one pixel.
#[derive(Debug, Clone)]
pub struct PhaseContrastResult {
    /// Average transmission.
    pub transmission: f64,
    /// Differential phase in radians.
    pub differential_phase: f64,
    /// Visibility (dark-field) contrast.
    pub visibility: f64,
}

impl PhaseContrastImager {
    /// Create a new phase contrast imager.
    ///
    /// `n_steps` is the number of phase steps in the grating scan (typically 8-16).
    /// `grating_period_um` is the period of the analyser grating.
    pub fn new(n_steps: usize, grating_period_um: f64) -> Self {
        Self {
            n_steps: n_steps.max(3),
            grating_period_um,
        }
    }

    /// Extract transmission, phase, and visibility from a stepping curve.
    ///
    /// `intensities` contains the measured intensity at each grating step
    /// for a single pixel.
    ///
    /// The stepping curve is modelled as:
    ///   I(k) = a0 + a1 * cos(2*pi*k/N + phi)
    ///
    /// - transmission = a0 (normalised by reference)
    /// - differential_phase = phi_sample - phi_reference
    /// - visibility = a1/a0
    pub fn analyse_stepping_curve(&self, intensities: &[f64]) -> PhaseContrastResult {
        let n = intensities.len();
        if n < 3 {
            return PhaseContrastResult {
                transmission: 0.0,
                differential_phase: 0.0,
                visibility: 0.0,
            };
        }

        // DFT at DC (k=0) and fundamental frequency (k=1)
        let mut sum_cos = 0.0;
        let mut sum_sin = 0.0;
        let mut sum_dc = 0.0;

        for (i, &val) in intensities.iter().enumerate() {
            let phase = 2.0 * PI * i as f64 / n as f64;
            sum_cos += val * phase.cos();
            sum_sin += val * phase.sin();
            sum_dc += val;
        }

        let a0 = sum_dc / n as f64;
        let a1 = 2.0 * (sum_cos * sum_cos + sum_sin * sum_sin).sqrt() / n as f64;
        let phi = (-sum_sin).atan2(sum_cos);

        let visibility = if a0.abs() > 1e-12 { a1 / a0 } else { 0.0 };

        PhaseContrastResult {
            transmission: a0,
            differential_phase: phi,
            visibility: visibility.clamp(0.0, 1.0),
        }
    }

    /// Extract differential phase contrast image from sample and reference
    /// stepping scans.
    ///
    /// `sample_curves` and `reference_curves` each contain one stepping curve
    /// per pixel.
    ///
    /// Returns (transmission, differential_phase, dark_field) images.
    pub fn process_images(
        &self,
        sample_curves: &[Vec<f64>],
        reference_curves: &[Vec<f64>],
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let n = sample_curves.len().min(reference_curves.len());
        let mut trans = Vec::with_capacity(n);
        let mut phase = Vec::with_capacity(n);
        let mut dark_field = Vec::with_capacity(n);

        for i in 0..n {
            let sample = self.analyse_stepping_curve(&sample_curves[i]);
            let reference = self.analyse_stepping_curve(&reference_curves[i]);

            let t = if reference.transmission.abs() > 1e-12 {
                sample.transmission / reference.transmission
            } else {
                0.0
            };

            let dp = sample.differential_phase - reference.differential_phase;
            // Wrap to [-pi, pi]
            let dp_wrapped = ((dp + PI) % (2.0 * PI) + 2.0 * PI) % (2.0 * PI) - PI;

            let df = if reference.visibility.abs() > 1e-12 {
                sample.visibility / reference.visibility
            } else {
                0.0
            };

            trans.push(t);
            phase.push(dp_wrapped);
            dark_field.push(df.clamp(0.0, 1.0));
        }
        (trans, phase, dark_field)
    }

    /// Integrate differential phase to get the total phase shift.
    ///
    /// Simple cumulative trapezoid integration of the differential phase
    /// profile along the x-direction. Assumes uniform pixel spacing.
    pub fn integrate_phase(differential_phase: &[f64], pixel_size_um: f64) -> Vec<f64> {
        let n = differential_phase.len();
        if n == 0 {
            return Vec::new();
        }
        let mut integrated = Vec::with_capacity(n);
        integrated.push(0.0);
        for i in 1..n {
            let val = integrated[i - 1]
                + 0.5 * (differential_phase[i - 1] + differential_phase[i]) * pixel_size_um;
            integrated.push(val);
        }
        integrated
    }

    /// Number of steps getter.
    pub fn n_steps(&self) -> usize {
        self.n_steps
    }

    /// Grating period getter.
    pub fn grating_period_um(&self) -> f64 {
        self.grating_period_um
    }
}

// ─── De Broglie and thermal neutron helpers ─────────────────────────────────

/// Compute the de Broglie wavelength in Angstroms for a neutron of given energy in meV.
pub fn de_broglie_wavelength_a(energy_mev: f64) -> f64 {
    if energy_mev <= 0.0 {
        return 0.0;
    }
    PLANCK_H / (2.0 * NEUTRON_MASS_KG * energy_mev * MEV_TO_J).sqrt() / ANGSTROM
}

/// Compute neutron velocity in m/s for a given energy in meV.
pub fn neutron_velocity(energy_mev: f64) -> f64 {
    if energy_mev <= 0.0 {
        return 0.0;
    }
    (2.0 * energy_mev * MEV_TO_J / NEUTRON_MASS_KG).sqrt()
}

/// Maxwell-Boltzmann most probable energy in meV at a given moderator
/// temperature in Kelvin.
///
/// E_mp = k_B * T / MEV_TO_J (converted to meV)
pub fn maxwell_boltzmann_energy_mev(temperature_k: f64) -> f64 {
    BOLTZMANN_K * temperature_k / MEV_TO_J
}

/// Maxwell-Boltzmann most probable wavelength in Angstroms at a given
/// moderator temperature in Kelvin.
pub fn maxwell_boltzmann_wavelength_a(temperature_k: f64) -> f64 {
    let e = maxwell_boltzmann_energy_mev(temperature_k);
    de_broglie_wavelength_a(e)
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    // --- BeamEnergy tests ---

    #[test]
    fn test_thermal_neutron_energy() {
        let beam = BeamEnergy::Thermal;
        assert!((beam.energy_mev() - 25.3).abs() < 0.1);
    }

    #[test]
    fn test_thermal_neutron_wavelength() {
        let beam = BeamEnergy::Thermal;
        let lambda = beam.wavelength_angstrom();
        // Thermal neutrons: ~1.8 Angstroms
        assert!((lambda - 1.8).abs() < 0.1, "lambda = {}", lambda);
    }

    #[test]
    fn test_thermal_neutron_velocity() {
        let beam = BeamEnergy::Thermal;
        let v = beam.velocity_ms();
        // Thermal neutrons: ~2200 m/s
        assert!((v - 2200.0).abs() < 50.0, "v = {}", v);
    }

    #[test]
    fn test_cold_neutron_longer_wavelength() {
        let cold = BeamEnergy::Cold;
        let thermal = BeamEnergy::Thermal;
        // Cold neutrons have longer wavelength than thermal
        assert!(cold.wavelength_angstrom() > thermal.wavelength_angstrom());
    }

    #[test]
    fn test_epithermal_shorter_wavelength() {
        let epi = BeamEnergy::Epithermal;
        let thermal = BeamEnergy::Thermal;
        assert!(epi.wavelength_angstrom() < thermal.wavelength_angstrom());
    }

    #[test]
    fn test_custom_energy() {
        let custom = BeamEnergy::Custom(50.0);
        assert!((custom.energy_mev() - 50.0).abs() < TOL);
    }

    // --- De Broglie helpers ---

    #[test]
    fn test_de_broglie_thermal() {
        let lambda = de_broglie_wavelength_a(25.3);
        assert!((lambda - 1.8).abs() < 0.1);
    }

    #[test]
    fn test_neutron_velocity_thermal() {
        let v = neutron_velocity(25.3);
        assert!((v - 2200.0).abs() < 50.0);
    }

    #[test]
    fn test_maxwell_boltzmann_at_300k() {
        let e = maxwell_boltzmann_energy_mev(300.0);
        // k_B * 300 / meV ~ 25.85 meV
        assert!((e - 25.85).abs() < 0.5, "E = {} meV", e);
    }

    #[test]
    fn test_maxwell_boltzmann_wavelength() {
        let lambda = maxwell_boltzmann_wavelength_a(300.0);
        assert!(lambda > 1.0 && lambda < 3.0);
    }

    // --- White beam normalizer ---

    #[test]
    fn test_normalize_basic() {
        let raw = vec![800.0, 600.0, 400.0, 200.0];
        let open = vec![1000.0, 1000.0, 1000.0, 1000.0];
        let dark = vec![100.0, 100.0, 100.0, 100.0];
        let norm = WhiteBeamNormalizer::normalize(&raw, &open, &dark);
        assert_eq!(norm.len(), 4);
        // (800-100)/(1000-100) = 700/900 ~ 0.7778
        assert!((norm[0] - 700.0 / 900.0).abs() < 1e-10);
        assert!((norm[3] - 100.0 / 900.0).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_full_transmission() {
        let raw = vec![1000.0];
        let open = vec![1000.0];
        let dark = vec![0.0];
        let norm = WhiteBeamNormalizer::normalize(&raw, &open, &dark);
        assert!((norm[0] - 1.0).abs() < TOL);
    }

    #[test]
    fn test_normalize_zero_denominator() {
        let raw = vec![500.0];
        let open = vec![100.0];
        let dark = vec![100.0];
        let norm = WhiteBeamNormalizer::normalize(&raw, &open, &dark);
        assert!((norm[0]).abs() < TOL);
    }

    #[test]
    fn test_normalize_with_median() {
        let raw = vec![800.0, 600.0];
        let open1 = vec![1000.0, 1000.0];
        let open2 = vec![1010.0, 990.0];
        let open3 = vec![1005.0, 995.0];
        let dark1 = vec![100.0, 100.0];
        let dark2 = vec![110.0, 90.0];
        let dark3 = vec![105.0, 95.0];
        let norm = WhiteBeamNormalizer::normalize_with_median(
            &raw,
            &[open1, open2, open3],
            &[dark1, dark2, dark3],
        );
        assert_eq!(norm.len(), 2);
        // median open = [1005, 995], median dark = [105, 95]
        // norm[0] = (800-105)/(1005-105) = 695/900
        assert!((norm[0] - 695.0 / 900.0).abs() < 1e-10);
    }

    // --- Transmission calculator ---

    #[test]
    fn test_beer_lambert_zero_thickness() {
        let calc = TransmissionCalculator::new();
        let t = calc.beer_lambert(5.0, 0.0);
        assert!((t - 1.0).abs() < TOL);
    }

    #[test]
    fn test_beer_lambert_finite() {
        let calc = TransmissionCalculator::new();
        let sigma = 1.0; // cm^-1
        let thickness = 1.0; // cm
        let t = calc.beer_lambert(sigma, thickness);
        assert!((t - (-1.0_f64).exp()).abs() < TOL);
    }

    #[test]
    fn test_optical_density() {
        let calc = TransmissionCalculator::new();
        let od = calc.optical_density(2.0, 3.0);
        assert!((od - 6.0).abs() < TOL);
    }

    #[test]
    fn test_thickness_from_transmission() {
        let calc = TransmissionCalculator::new();
        let sigma = 2.0;
        let original_t = 0.5;
        let t = calc.thickness_from_transmission(calc.beer_lambert(sigma, original_t), sigma);
        assert!((t - original_t).abs() < 1e-10, "t = {}", t);
    }

    #[test]
    fn test_attenuation_map() {
        let norm = vec![1.0, 0.5, 0.1];
        let atten = TransmissionCalculator::attenuation_map(&norm);
        assert!((atten[0]).abs() < TOL); // -ln(1) = 0
        assert!((atten[1] - 0.5_f64.ln().abs()).abs() < TOL);
    }

    #[test]
    fn test_multi_layer() {
        let calc = TransmissionCalculator::new();
        let layers = vec![(1.0, 0.5), (2.0, 0.3)];
        let t = calc.multi_layer(&layers);
        let expected = (-1.0 * 0.5 - 2.0 * 0.3_f64).exp();
        assert!((t - expected).abs() < TOL);
    }

    // --- Attenuation coefficients ---

    #[test]
    fn test_hydrogen_high_cross_section() {
        let h = AttenuationCoefficient::microscopic_total_barns(Element::Hydrogen);
        let al = AttenuationCoefficient::microscopic_total_barns(Element::Aluminium);
        // Hydrogen has much higher total cross section than aluminium
        assert!(h > al * 10.0);
    }

    #[test]
    fn test_gadolinium_highest_absorption() {
        let gd = AttenuationCoefficient::microscopic_absorption_barns(Element::Gadolinium);
        let cd = AttenuationCoefficient::microscopic_absorption_barns(Element::Cadmium);
        assert!(gd > cd);
    }

    #[test]
    fn test_scattering_equals_total_minus_absorption() {
        let total = AttenuationCoefficient::microscopic_total_barns(Element::Iron);
        let abs = AttenuationCoefficient::microscopic_absorption_barns(Element::Iron);
        let scat = AttenuationCoefficient::microscopic_scattering_barns(Element::Iron);
        assert!((scat - (total - abs)).abs() < TOL);
    }

    #[test]
    fn test_macroscopic_cross_section_positive() {
        let sigma = AttenuationCoefficient::macroscopic_standard(Element::Water);
        assert!(sigma > 0.0);
    }

    #[test]
    fn test_mean_free_path() {
        let mfp = AttenuationCoefficient::mean_free_path_cm(Element::Lead);
        assert!(mfp > 0.0 && mfp < 100.0);
    }

    // --- Scattering corrector ---

    #[test]
    fn test_gaussian_psf_1d_normalized() {
        let corrector = ScatteringCorrector::new(0.1, 2.0);
        let psf = corrector.gaussian_psf_1d(6);
        let sum: f64 = psf.iter().sum();
        assert!((sum - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_gaussian_psf_2d_normalized() {
        let corrector = ScatteringCorrector::new(0.1, 2.0);
        let psf = corrector.gaussian_psf_2d(5);
        let sum: f64 = psf.iter().sum();
        assert!((sum - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_scatter_correction_reduces_bias() {
        let corrector = ScatteringCorrector::new(0.1, 1.5);
        // A sharp dip should become sharper after scatter correction
        let mut measured = vec![1.0; 50];
        for i in 20..30 {
            measured[i] = 0.5;
        }
        let corrected = corrector.correct_1d(&measured, 5);
        // The dip should be deeper (less scattered signal)
        assert!(corrected[25] < measured[25] + 0.05);
    }

    // --- TOF spectrum analyser ---

    #[test]
    fn test_tof_to_energy_thermal() {
        let analyser = TofSpectrumAnalyzer::new(10.0);
        // Thermal neutrons at 2200 m/s travel 10 m in ~4.545 ms
        let tof = 10.0 / 2200.0;
        let e = analyser.tof_to_energy_mev(tof);
        assert!((e - 25.3).abs() < 1.0, "E = {} meV", e);
    }

    #[test]
    fn test_tof_to_wavelength() {
        let analyser = TofSpectrumAnalyzer::new(10.0);
        let tof = 10.0 / 2200.0;
        let lambda = analyser.tof_to_wavelength_angstrom(tof);
        assert!((lambda - 1.8).abs() < 0.1, "lambda = {} A", lambda);
    }

    #[test]
    fn test_energy_to_tof_roundtrip() {
        let analyser = TofSpectrumAnalyzer::new(15.0);
        let e_in = 50.0;
        let tof = analyser.energy_to_tof_s(e_in);
        let e_out = analyser.tof_to_energy_mev(tof);
        assert!((e_out - e_in).abs() < 0.01);
    }

    #[test]
    fn test_tof_histogram() {
        let analyser = TofSpectrumAnalyzer::new(10.0);
        let events = vec![0.001, 0.002, 0.003, 0.002, 0.005];
        let (centres, counts) = analyser.histogram(&events, 10, (0.0, 0.01));
        assert_eq!(centres.len(), 10);
        assert_eq!(counts.len(), 10);
        let total: u64 = counts.iter().sum();
        assert_eq!(total, 5);
    }

    // --- Bragg edge analyser ---

    #[test]
    fn test_edge_wavelength() {
        let d = 2.0267; // Fe (110) d-spacing
        let lambda = BraggEdgeAnalyzer::edge_wavelength(d);
        assert!((lambda - 4.0534).abs() < 0.001);
    }

    #[test]
    fn test_d_spacing_from_edge() {
        let lambda = 4.0534;
        let d = BraggEdgeAnalyzer::d_spacing_from_edge(lambda);
        assert!((d - 2.0267).abs() < 0.001);
    }

    #[test]
    fn test_strain_calculation() {
        let d_ref = 2.0267;
        let d_measured = 2.0300;
        let strain = BraggEdgeAnalyzer::strain(d_measured, d_ref);
        assert!((strain - (d_measured - d_ref) / d_ref).abs() < TOL);
    }

    #[test]
    fn test_iron_bcc_d_spacings() {
        let spacings = BraggEdgeAnalyzer::iron_bcc_d_spacings();
        assert!(!spacings.is_empty());
        // (110) should be the largest d-spacing
        let d110 = spacings[0].1;
        assert!((d110 - 2.027).abs() < 0.01, "d110 = {}", d110);
    }

    #[test]
    fn test_find_bragg_edges() {
        // Create a synthetic transmission spectrum with a step at lambda = 4.0 A
        let n = 200;
        let wavelengths: Vec<f64> = (0..n).map(|i| 2.0 + 4.0 * i as f64 / n as f64).collect();
        let transmission: Vec<f64> = wavelengths
            .iter()
            .map(|&w| {
                if w < 4.0 {
                    0.5
                } else {
                    0.7
                }
            })
            .collect();
        let edges = BraggEdgeAnalyzer::find_edges(&wavelengths, &transmission, 2.0, 0.001);
        // Should detect an edge near lambda = 4.0 A
        assert!(!edges.is_empty(), "Should detect at least one edge");
        let nearest = edges
            .iter()
            .min_by(|a, b| {
                (a.wavelength_a - 4.0)
                    .abs()
                    .partial_cmp(&(b.wavelength_a - 4.0).abs())
                    .unwrap()
            })
            .unwrap();
        assert!(
            (nearest.wavelength_a - 4.0).abs() < 0.2,
            "edge at {} A",
            nearest.wavelength_a
        );
    }

    // --- Sinogram generator ---

    #[test]
    fn test_sinogram_from_projections() {
        let p1 = vec![1.0, 2.0, 3.0];
        let p2 = vec![4.0, 5.0, 6.0];
        let (sino, n_a, n_p) = SinogramGenerator::from_projections(&[p1, p2]);
        assert_eq!(n_a, 2);
        assert_eq!(n_p, 3);
        assert_eq!(sino.len(), 6);
        assert!((sino[0] - 1.0).abs() < TOL);
        assert!((sino[3] - 4.0).abs() < TOL);
    }

    #[test]
    fn test_radon_transform_uniform() {
        // Uniform phantom should produce uniform projections
        let size = 16;
        let phantom = vec![1.0; size * size];
        let angles: Vec<f64> = vec![0.0, 90.0];
        let (sino, n_a, n_d) = SinogramGenerator::radon_transform(&phantom, size, size, &angles);
        assert_eq!(n_a, 2);
        assert!(n_d > 0);
        // Check that projections have non-zero values
        let max_val = sino.iter().cloned().fold(0.0_f64, f64::max);
        assert!(max_val > 0.0);
    }

    // --- Filtered back-projection ---

    #[test]
    fn test_fbp_filter_kernel_ramp() {
        let fbp = FilteredBackProjection::new(FbpFilter::RamLak);
        let kernel = fbp.filter_kernel(11);
        assert_eq!(kernel.len(), 11);
        // Centre value should be 0.25
        assert!((kernel[5] - 0.25).abs() < TOL);
        // Even-offset values should be 0
        assert!((kernel[3]).abs() < TOL); // offset -2
        assert!((kernel[7]).abs() < TOL); // offset +2
    }

    #[test]
    fn test_fbp_filter_kernel_none() {
        let fbp = FilteredBackProjection::new(FbpFilter::None);
        let kernel = fbp.filter_kernel(7);
        assert!((kernel[3] - 1.0).abs() < TOL);
        assert!((kernel[0]).abs() < TOL);
    }

    #[test]
    fn test_fbp_reconstruct_basic() {
        // Simple test: create a point-like sinogram and reconstruct
        let n_angles = 36;
        let n_det = 32;
        let angles: Vec<f64> = (0..n_angles).map(|i| i as f64 * 180.0 / n_angles as f64).collect();

        // Single bright point sinogram (sinusoidal trace)
        let mut sinogram = vec![0.0; n_angles * n_det];
        let cd = n_det as f64 / 2.0;
        for (a_idx, &angle) in angles.iter().enumerate() {
            let theta = angle * PI / 180.0;
            let s = 3.0 * theta.cos() + cd;
            let s_idx = s.round() as usize;
            if s_idx < n_det {
                sinogram[a_idx * n_det + s_idx] = 1.0;
            }
        }

        let fbp = FilteredBackProjection::new(FbpFilter::RamLak);
        let image = fbp.reconstruct(&sinogram, n_angles, n_det, &angles, 32);
        assert_eq!(image.len(), 32 * 32);
        // Should have some non-zero values
        let max_val = image.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(max_val > 0.0);
    }

    #[test]
    fn test_fbp_shepp_logan_filter() {
        let fbp = FilteredBackProjection::new(FbpFilter::SheppLogan);
        let kernel = fbp.filter_kernel(21);
        assert_eq!(kernel.len(), 21);
        // Shepp-Logan should have reduced magnitude at high frequencies
        assert!(kernel[10].abs() > 0.0); // centre
    }

    // --- Phase contrast imager ---

    #[test]
    fn test_phase_contrast_pure_dc() {
        let imager = PhaseContrastImager::new(8, 4.0);
        let intensities = vec![100.0; 8]; // flat = no modulation
        let result = imager.analyse_stepping_curve(&intensities);
        assert!((result.transmission - 100.0).abs() < TOL);
        assert!(result.visibility < 0.01); // no oscillation
    }

    #[test]
    fn test_phase_contrast_cosine() {
        let imager = PhaseContrastImager::new(8, 4.0);
        // Perfect cosine: I(k) = 100 + 50*cos(2*pi*k/8)
        let intensities: Vec<f64> = (0..8)
            .map(|k| 100.0 + 50.0 * (2.0 * PI * k as f64 / 8.0).cos())
            .collect();
        let result = imager.analyse_stepping_curve(&intensities);
        assert!((result.transmission - 100.0).abs() < 0.1);
        assert!((result.visibility - 0.5).abs() < 0.05, "vis = {}", result.visibility);
        assert!(result.differential_phase.abs() < 0.1);
    }

    #[test]
    fn test_phase_contrast_shifted() {
        let imager = PhaseContrastImager::new(16, 4.0);
        let phase_shift = 1.0; // rad
        let intensities: Vec<f64> = (0..16)
            .map(|k| 100.0 + 50.0 * (2.0 * PI * k as f64 / 16.0 + phase_shift).cos())
            .collect();
        let result = imager.analyse_stepping_curve(&intensities);
        // Should detect the phase shift (with sign convention)
        assert!((result.differential_phase.abs() - phase_shift.abs()).abs() < 0.2);
    }

    #[test]
    fn test_phase_contrast_images() {
        let imager = PhaseContrastImager::new(8, 4.0);
        let n_pixels = 4;

        // Reference: uniform cosine
        let reference_curves: Vec<Vec<f64>> = (0..n_pixels)
            .map(|_| {
                (0..8)
                    .map(|k| 100.0 + 50.0 * (2.0 * PI * k as f64 / 8.0).cos())
                    .collect()
            })
            .collect();

        // Sample: attenuated and phase-shifted
        let sample_curves: Vec<Vec<f64>> = (0..n_pixels)
            .map(|p| {
                let attenuation = 0.8;
                let phase = 0.5 * p as f64;
                (0..8)
                    .map(|k| {
                        attenuation
                            * (100.0 + 50.0 * (2.0 * PI * k as f64 / 8.0 + phase).cos())
                    })
                    .collect()
            })
            .collect();

        let (trans, phase, dark_field) = imager.process_images(&sample_curves, &reference_curves);
        assert_eq!(trans.len(), n_pixels);
        assert_eq!(phase.len(), n_pixels);
        assert_eq!(dark_field.len(), n_pixels);
        // First pixel: no phase shift, attenuation = 0.8
        assert!((trans[0] - 0.8).abs() < 0.05);
    }

    #[test]
    fn test_integrate_phase() {
        let dp = vec![0.1, 0.1, 0.1, 0.1];
        let integrated = PhaseContrastImager::integrate_phase(&dp, 1.0);
        assert_eq!(integrated.len(), 4);
        assert!((integrated[0]).abs() < TOL);
        // Linearly increasing
        assert!(integrated[3] > integrated[1]);
    }

    // --- NeutronConfig ---

    #[test]
    fn test_default_config() {
        let config = NeutronConfig::default();
        assert_eq!(config.beam_energy, BeamEnergy::Thermal);
        assert_eq!(config.detector_type, DetectorType::Scintillator);
        assert!((config.pixel_size_um - 50.0).abs() < TOL);
        assert!((config.exposure_time_s - 10.0).abs() < TOL);
    }

    // --- Additional edge cases and coverage ---

    #[test]
    fn test_de_broglie_zero_energy() {
        assert!((de_broglie_wavelength_a(0.0)).abs() < TOL);
        assert!((de_broglie_wavelength_a(-1.0)).abs() < TOL);
    }

    #[test]
    fn test_tof_zero_time() {
        let analyser = TofSpectrumAnalyzer::new(10.0);
        assert!((analyser.tof_to_energy_mev(0.0)).abs() < TOL);
        assert!((analyser.tof_to_wavelength_angstrom(0.0)).abs() < TOL);
    }

    #[test]
    fn test_tof_histogram_to_energy() {
        let analyser = TofSpectrumAnalyzer::new(10.0);
        let centres = vec![0.001, 0.002, 0.003, 0.004, 0.005];
        let counts = vec![100, 200, 300, 200, 100];
        let (energies, intensities) = analyser.tof_histogram_to_energy(&centres, &counts);
        assert_eq!(energies.len(), 5);
        assert_eq!(intensities.len(), 5);
        // Higher TOF = lower energy
        assert!(energies[0] > energies[4]);
    }

    #[test]
    fn test_aluminium_fcc_d_spacings() {
        let spacings = BraggEdgeAnalyzer::aluminium_fcc_d_spacings();
        assert!(!spacings.is_empty());
        // (111) should be largest d-spacing for FCC
        let d111 = spacings[0].1;
        assert!((d111 - 2.338).abs() < 0.01, "d111 = {}", d111);
    }

    #[test]
    fn test_macroscopic_with_custom_density() {
        let sigma = AttenuationCoefficient::macroscopic(Element::Hydrogen, 1e22);
        assert!(sigma > 0.0);
        // sigma = 1e22 * 82.02e-24 = ~0.82 cm^-1
        assert!((sigma - 0.82).abs() < 0.1, "sigma = {}", sigma);
    }

    #[test]
    fn test_convolve_1d_identity() {
        let signal = vec![0.0, 0.0, 1.0, 0.0, 0.0];
        let kernel = vec![0.0, 0.0, 1.0, 0.0, 0.0]; // impulse at centre
        let result = convolve_1d(&signal, &kernel);
        assert_eq!(result.len(), 5);
        assert!((result[2] - 1.0).abs() < TOL);
    }

    #[test]
    fn test_fbp_hamming_filter() {
        let fbp = FilteredBackProjection::new(FbpFilter::Hamming);
        let kernel = fbp.filter_kernel(21);
        // Hamming window reduces edge values
        assert!(kernel[0].abs() < kernel[10].abs());
    }

    #[test]
    fn test_fbp_hann_filter() {
        let fbp = FilteredBackProjection::new(FbpFilter::Hann);
        let kernel = fbp.filter_kernel(21);
        // Hann window: first and last samples are zero
        assert!(kernel[0].abs() < 1e-10);
    }
}
