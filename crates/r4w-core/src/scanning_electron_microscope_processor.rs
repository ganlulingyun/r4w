//! # Scanning Electron Microscope (SEM) Signal Processor
//!
//! Signal processing for Scanning Electron Microscope imaging: secondary electron
//! (SE) and backscattered electron (BSE) detection, Energy Dispersive X-ray (EDX)
//! spectroscopy, image contrast enhancement, noise reduction, and dimensional
//! metrology.
//!
//! ## Applications
//!
//! - Semiconductor inspection and failure analysis
//! - Materials science (grain size, phase identification)
//! - Biological sample imaging
//! - Critical dimension (CD) metrology
//! - Elemental composition mapping via EDX
//!
//! ## Physics
//!
//! - **SE energy**: < 50 eV (surface-sensitive, topographic contrast)
//! - **BSE energy**: > 50 eV (composition-sensitive, Z-contrast)
//! - **X-ray energy**: characteristic lines (K-alpha, L-alpha) for elemental analysis
//! - **Resolution**: ~1 nm (field emission), ~3-5 nm (thermionic)
//! - **Kanaya-Okayama range**: R = 0.0276 * A * E^1.67 / (Z^0.89 * rho) [um]
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::scanning_electron_microscope_processor::{
//!     SemConfig, DetectorType, BackscatterDetector, MagnificationCalibrator,
//! };
//!
//! let config = SemConfig::new(20.0, 100e-12, 10.0, DetectorType::SecondaryElectron);
//! assert!((config.beam_energy_kev - 20.0).abs() < 1e-9);
//!
//! let bse = BackscatterDetector::new();
//! let eta = bse.backscatter_coefficient(29); // Copper
//! assert!(eta > 0.0 && eta < 1.0);
//!
//! let cal = MagnificationCalibrator::new(1000.0, 1024);
//! let ps = cal.pixel_size_um();
//! assert!(ps > 0.0);
//! ```

use std::f64::consts::PI;

// ─── Physical Constants ─────────────────────────────────────────────

/// Electron charge (Coulombs).
const ELECTRON_CHARGE: f64 = 1.602_176_634e-19;

/// Electron rest mass (kg).
const ELECTRON_MASS: f64 = 9.109_383_7015e-31;

/// Avogadro constant.
#[allow(dead_code)]
const AVOGADRO: f64 = 6.022_140_76e23;

// ─── Common X-ray Energies (keV) ───────────────────────────────────

/// Characteristic X-ray energies: (element_name, Z, K_alpha_keV, L_alpha_keV).
/// L_alpha is 0.0 for light elements where L lines are not typically measured.
const XRAY_LINES: &[(&str, u32, f64, f64)] = &[
    ("C", 6, 0.277, 0.0),
    ("N", 7, 0.392, 0.0),
    ("O", 8, 0.525, 0.0),
    ("F", 9, 0.677, 0.0),
    ("Na", 11, 1.041, 0.0),
    ("Mg", 12, 1.254, 0.0),
    ("Al", 13, 1.487, 0.0),
    ("Si", 14, 1.740, 0.0),
    ("P", 15, 2.013, 0.0),
    ("S", 16, 2.307, 0.0),
    ("Cl", 17, 2.622, 0.0),
    ("K", 19, 3.314, 0.0),
    ("Ca", 20, 3.692, 0.0),
    ("Ti", 22, 4.511, 0.452),
    ("Cr", 24, 5.415, 0.573),
    ("Mn", 25, 5.899, 0.637),
    ("Fe", 26, 6.404, 0.705),
    ("Co", 27, 6.930, 0.776),
    ("Ni", 28, 7.478, 0.851),
    ("Cu", 29, 8.048, 0.930),
    ("Zn", 30, 8.639, 1.012),
    ("Ga", 31, 9.252, 1.098),
    ("Ge", 32, 9.886, 1.188),
    ("As", 33, 10.544, 1.282),
    ("Mo", 42, 17.479, 2.293),
    ("Ag", 47, 22.163, 2.984),
    ("Sn", 50, 25.271, 3.444),
    ("W", 74, 59.318, 8.398),
    ("Au", 79, 68.804, 9.713),
    ("Pb", 82, 74.969, 10.551),
    ("U", 92, 98.439, 13.614),
];

// ─── Detector Type ──────────────────────────────────────────────────

/// SEM detector type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DetectorType {
    /// Everhart-Thornley secondary electron detector.
    SecondaryElectron,
    /// Solid-state backscattered electron detector.
    Backscatter,
    /// Electron backscatter diffraction.
    Ebsd,
    /// Energy dispersive X-ray spectrometer.
    Eds,
}

// ─── SEM Configuration ──────────────────────────────────────────────

/// SEM operating parameters.
#[derive(Debug, Clone)]
pub struct SemConfig {
    /// Accelerating voltage / beam energy in keV.
    pub beam_energy_kev: f64,
    /// Beam current in amperes (typically pA to nA).
    pub beam_current_a: f64,
    /// Working distance in mm.
    pub working_distance_mm: f64,
    /// Primary detector type.
    pub detector: DetectorType,
}

impl SemConfig {
    /// Create a new SEM configuration.
    pub fn new(
        beam_energy_kev: f64,
        beam_current_a: f64,
        working_distance_mm: f64,
        detector: DetectorType,
    ) -> Self {
        Self {
            beam_energy_kev,
            beam_current_a,
            working_distance_mm,
            detector,
        }
    }

    /// Standard imaging preset: 20 keV, 100 pA, 10 mm WD, SE detector.
    pub fn standard_imaging() -> Self {
        Self::new(20.0, 100e-12, 10.0, DetectorType::SecondaryElectron)
    }

    /// Low-kV preset for beam-sensitive or non-conductive samples: 1 keV.
    pub fn low_voltage() -> Self {
        Self::new(1.0, 50e-12, 5.0, DetectorType::SecondaryElectron)
    }

    /// EDX analysis preset: 20 keV, higher beam current for good count rate.
    pub fn edx_analysis() -> Self {
        Self::new(20.0, 1e-9, 10.0, DetectorType::Eds)
    }

    /// Beam convergence half-angle (radians) estimated from WD and aperture.
    /// Uses a typical 30 um final aperture radius.
    pub fn convergence_half_angle_rad(&self) -> f64 {
        let aperture_radius_mm = 0.030; // 30 um typical
        (aperture_radius_mm / self.working_distance_mm).atan()
    }

    /// Kanaya-Okayama electron range in micrometres.
    /// R = 0.0276 * A * E^1.67 / (Z^0.89 * rho)
    /// where A = atomic weight (g/mol), E = beam energy (keV),
    /// Z = atomic number, rho = density (g/cm^3).
    pub fn kanaya_okayama_range_um(
        &self,
        atomic_weight: f64,
        atomic_number: u32,
        density_g_cm3: f64,
    ) -> f64 {
        let z = atomic_number as f64;
        let e = self.beam_energy_kev;
        0.0276 * atomic_weight * e.powf(1.67) / (z.powf(0.89) * density_g_cm3)
    }

    /// Beam diameter estimate (nm) from brightness equation.
    /// d_p = sqrt(4 * I_b / (pi^2 * beta * alpha^2))
    /// Using typical FE-SEM brightness ~1e8 A/(cm^2 sr).
    pub fn estimated_beam_diameter_nm(&self) -> f64 {
        let brightness = 1e8 * 1e4; // A/(m^2 sr)
        let alpha = self.convergence_half_angle_rad();
        let numerator = 4.0 * self.beam_current_a;
        let denominator = PI * PI * brightness * alpha * alpha;
        let d_m = (numerator / denominator).sqrt();
        d_m * 1e9 // convert to nm
    }
}

// ─── Secondary Electron Detector ────────────────────────────────────

/// Secondary electron signal processor with edge enhancement and
/// topographic contrast extraction.
#[derive(Debug, Clone)]
pub struct SecondaryElectronDetector {
    /// Edge enhancement strength [0, 1].
    pub edge_strength: f64,
    /// Noise floor for thresholding.
    pub noise_floor: f64,
}

impl SecondaryElectronDetector {
    /// Create a new SE detector processor.
    pub fn new(edge_strength: f64) -> Self {
        Self {
            edge_strength: edge_strength.clamp(0.0, 1.0),
            noise_floor: 0.01,
        }
    }

    /// Apply topographic contrast enhancement to an SE image.
    /// Uses a Laplacian-of-Gaussian approximation for edge emphasis.
    /// `image` is row-major, dimensions `width x height`.
    pub fn enhance_topography(
        &self,
        image: &[f64],
        width: usize,
        height: usize,
    ) -> Vec<f64> {
        if width < 3 || height < 3 || image.len() != width * height {
            return image.to_vec();
        }
        let laplacian = self.laplacian(image, width, height);
        let mut output = Vec::with_capacity(image.len());
        for i in 0..image.len() {
            let enhanced = image[i] + self.edge_strength * laplacian[i];
            output.push(enhanced.max(0.0));
        }
        output
    }

    /// Compute the discrete Laplacian of a 2D image.
    fn laplacian(&self, image: &[f64], width: usize, height: usize) -> Vec<f64> {
        let mut out = vec![0.0; image.len()];
        for y in 1..height - 1 {
            for x in 1..width - 1 {
                let idx = y * width + x;
                let lap = image[idx - 1] + image[idx + 1]
                    + image[idx - width] + image[idx + width]
                    - 4.0 * image[idx];
                out[idx] = lap;
            }
        }
        out
    }

    /// SE yield estimation (delta) using the universal curve approximation.
    /// delta = delta_max * 1.28 * (E / E_max)^(-0.67) * (1 - exp(-1.614 * (E / E_max)^1.67))
    /// where delta_max ~ 1.0-3.0, E_max ~ 0.3-1.0 keV depending on material.
    pub fn se_yield(&self, beam_energy_kev: f64, delta_max: f64, e_max_kev: f64) -> f64 {
        let ratio = beam_energy_kev / e_max_kev;
        if ratio <= 0.0 {
            return 0.0;
        }
        delta_max * 1.28 * ratio.powf(-0.67) * (1.0 - (-1.614 * ratio.powf(1.67)).exp())
    }
}

// ─── Backscatter Electron Detector ──────────────────────────────────

/// BSE signal processor: atomic number contrast and Z-contrast imaging.
#[derive(Debug, Clone)]
pub struct BackscatterDetector {
    /// Low-angle / high-angle detector flag.
    pub compositional_mode: bool,
}

impl BackscatterDetector {
    /// Create a new BSE detector (compositional mode by default).
    pub fn new() -> Self {
        Self {
            compositional_mode: true,
        }
    }

    /// BSE coefficient (eta) from empirical polynomial fit:
    /// eta = -0.0254 + 0.016*Z - 1.86e-4*Z^2 + 8.3e-7*Z^3
    /// Valid for Z = 4..92 approximately.
    pub fn backscatter_coefficient(&self, atomic_number: u32) -> f64 {
        let z = atomic_number as f64;
        -0.0254 + 0.016 * z - 1.86e-4 * z * z + 8.3e-7 * z * z * z
    }

    /// Map an image of measured BSE signal intensities to estimated
    /// atomic numbers using the inverse of the eta polynomial.
    /// Returns f64 atomic numbers (fractional for mixed phases).
    pub fn estimate_atomic_numbers(&self, bse_signal: &[f64]) -> Vec<f64> {
        bse_signal.iter().map(|&eta| self.eta_to_z(eta)).collect()
    }

    /// Invert eta -> Z by Newton-Raphson on the polynomial.
    fn eta_to_z(&self, eta: f64) -> f64 {
        if eta <= 0.0 {
            return 0.0;
        }
        // Initial guess: linear approximation eta ~ 0.016*Z
        let mut z = (eta + 0.0254) / 0.016;
        z = z.clamp(1.0, 100.0);
        for _ in 0..20 {
            let f = -0.0254 + 0.016 * z - 1.86e-4 * z * z + 8.3e-7 * z * z * z - eta;
            let df = 0.016 - 2.0 * 1.86e-4 * z + 3.0 * 8.3e-7 * z * z;
            if df.abs() < 1e-15 {
                break;
            }
            let dz = f / df;
            z -= dz;
            z = z.clamp(1.0, 100.0);
            if dz.abs() < 1e-6 {
                break;
            }
        }
        z
    }

    /// Z-contrast ratio between two elements.
    pub fn z_contrast_ratio(&self, z1: u32, z2: u32) -> f64 {
        let eta1 = self.backscatter_coefficient(z1);
        let eta2 = self.backscatter_coefficient(z2);
        if eta2.abs() < 1e-12 {
            return f64::INFINITY;
        }
        eta1 / eta2
    }

    /// Apply compositional (Z) contrast mapping to a BSE image.
    /// Normalizes to [0, 1] range based on min/max BSE signal.
    pub fn z_contrast_image(&self, image: &[f64]) -> Vec<f64> {
        if image.is_empty() {
            return vec![];
        }
        let min = image.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = image.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let range = max - min;
        if range < 1e-15 {
            return vec![0.5; image.len()];
        }
        image.iter().map(|&v| (v - min) / range).collect()
    }
}

// ─── EDX Spectrum Processor ─────────────────────────────────────────

/// Peak identification result from EDX spectrum.
#[derive(Debug, Clone)]
pub struct EdxPeak {
    /// Element symbol.
    pub element: String,
    /// Atomic number.
    pub atomic_number: u32,
    /// Line type: "K-alpha" or "L-alpha".
    pub line_type: String,
    /// Reference energy in keV.
    pub energy_kev: f64,
    /// Measured peak intensity (counts).
    pub intensity: f64,
}

/// Energy Dispersive X-ray spectrum processor.
/// Identifies elements from characteristic X-ray peaks.
#[derive(Debug, Clone)]
pub struct EdxSpectrumProcessor {
    /// Energy per channel in keV.
    pub ev_per_channel: f64,
    /// Number of channels.
    pub num_channels: usize,
    /// Energy tolerance for peak matching (keV).
    pub match_tolerance_kev: f64,
}

impl EdxSpectrumProcessor {
    /// Create an EDX processor.
    /// `ev_per_channel` is the bin width (typically 10-20 eV = 0.01-0.02 keV).
    /// `num_channels` is the number of spectral bins.
    pub fn new(ev_per_channel: f64, num_channels: usize) -> Self {
        Self {
            ev_per_channel,
            num_channels,
            match_tolerance_kev: 0.1,
        }
    }

    /// Standard SiLi detector: 10 eV/ch, 2048 channels (0-20.48 keV).
    pub fn standard() -> Self {
        Self::new(0.010, 2048)
    }

    /// Convert channel index to energy in keV.
    pub fn channel_to_energy(&self, channel: usize) -> f64 {
        channel as f64 * self.ev_per_channel
    }

    /// Convert energy in keV to nearest channel index.
    pub fn energy_to_channel(&self, energy_kev: f64) -> usize {
        let ch = (energy_kev / self.ev_per_channel).round() as usize;
        ch.min(self.num_channels.saturating_sub(1))
    }

    /// Subtract Bremsstrahlung background using a simple top-hat filter.
    /// The top-hat filter estimates the continuum by averaging a window
    /// around each channel, excluding the central region.
    pub fn subtract_background(&self, spectrum: &[f64], window_half: usize) -> Vec<f64> {
        let n = spectrum.len();
        let gap = window_half / 3; // central exclusion
        let mut background = vec![0.0; n];

        for i in 0..n {
            let mut sum = 0.0;
            let mut count = 0u32;
            // Left wing
            let left_start = if i > window_half { i - window_half } else { 0 };
            let left_end = if i > gap { i - gap } else { 0 };
            for j in left_start..left_end {
                sum += spectrum[j];
                count += 1;
            }
            // Right wing
            let right_start = (i + gap + 1).min(n);
            let right_end = (i + window_half + 1).min(n);
            for j in right_start..right_end {
                sum += spectrum[j];
                count += 1;
            }
            if count > 0 {
                background[i] = sum / count as f64;
            }
        }

        spectrum
            .iter()
            .zip(background.iter())
            .map(|(&s, &b)| (s - b).max(0.0))
            .collect()
    }

    /// Find peaks in a spectrum above `threshold` counts.
    /// Returns (channel_index, intensity) pairs.
    pub fn find_peaks(&self, spectrum: &[f64], threshold: f64) -> Vec<(usize, f64)> {
        let n = spectrum.len();
        if n < 3 {
            return vec![];
        }
        let mut peaks = Vec::new();
        for i in 1..n - 1 {
            if spectrum[i] > threshold
                && spectrum[i] > spectrum[i - 1]
                && spectrum[i] >= spectrum[i + 1]
            {
                peaks.push((i, spectrum[i]));
            }
        }
        peaks
    }

    /// Identify elements from detected peaks by matching against the
    /// characteristic X-ray energy database.
    pub fn identify_elements(&self, spectrum: &[f64], threshold: f64) -> Vec<EdxPeak> {
        let net_spectrum = self.subtract_background(spectrum, 30);
        let peaks = self.find_peaks(&net_spectrum, threshold);
        let mut results = Vec::new();

        for (ch, intensity) in peaks {
            let energy = self.channel_to_energy(ch);
            // Try to match K-alpha lines
            for &(name, z, k_alpha, l_alpha) in XRAY_LINES {
                if (energy - k_alpha).abs() < self.match_tolerance_kev {
                    results.push(EdxPeak {
                        element: name.to_string(),
                        atomic_number: z,
                        line_type: "K-alpha".to_string(),
                        energy_kev: k_alpha,
                        intensity,
                    });
                }
                if l_alpha > 0.0 && (energy - l_alpha).abs() < self.match_tolerance_kev {
                    results.push(EdxPeak {
                        element: name.to_string(),
                        atomic_number: z,
                        line_type: "L-alpha".to_string(),
                        energy_kev: l_alpha,
                        intensity,
                    });
                }
            }
        }
        results
    }

    /// Duane-Hunt limit: maximum X-ray energy = beam energy.
    /// Used to verify spectrometer calibration.
    pub fn duane_hunt_limit_kev(beam_energy_kev: f64) -> f64 {
        beam_energy_kev
    }

    /// Overvoltage ratio U = E0 / Ec where Ec is the critical excitation energy.
    /// U > 2 recommended for efficient X-ray generation.
    pub fn overvoltage_ratio(beam_energy_kev: f64, line_energy_kev: f64) -> f64 {
        if line_energy_kev <= 0.0 {
            return 0.0;
        }
        beam_energy_kev / line_energy_kev
    }
}

// ─── Image Contrast Enhancer ────────────────────────────────────────

/// Image contrast enhancement methods for SEM images.
#[derive(Debug, Clone)]
pub struct ImageContrastEnhancer;

impl ImageContrastEnhancer {
    /// Histogram equalization to maximize dynamic range.
    pub fn histogram_equalize(image: &[f64]) -> Vec<f64> {
        if image.is_empty() {
            return vec![];
        }
        let n = image.len();
        let num_bins = 256usize;

        // Find min/max for scaling
        let min_val = image.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_val = image.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let range = max_val - min_val;
        if range < 1e-15 {
            return vec![0.5; n];
        }

        // Build histogram
        let mut hist = vec![0u32; num_bins];
        for &v in image {
            let bin = (((v - min_val) / range) * (num_bins - 1) as f64).round() as usize;
            let bin = bin.min(num_bins - 1);
            hist[bin] += 1;
        }

        // Cumulative distribution function
        let mut cdf = vec![0u32; num_bins];
        cdf[0] = hist[0];
        for i in 1..num_bins {
            cdf[i] = cdf[i - 1] + hist[i];
        }

        let cdf_min = cdf.iter().find(|&&v| v > 0).copied().unwrap_or(0);

        // Map each pixel
        image
            .iter()
            .map(|&v| {
                let bin = (((v - min_val) / range) * (num_bins - 1) as f64).round() as usize;
                let bin = bin.min(num_bins - 1);
                if n as u32 - cdf_min == 0 {
                    0.5
                } else {
                    (cdf[bin] - cdf_min) as f64 / (n as u32 - cdf_min) as f64
                }
            })
            .collect()
    }

    /// Gamma correction: output = input^gamma.
    /// gamma < 1 brightens, gamma > 1 darkens.
    pub fn gamma_correct(image: &[f64], gamma: f64) -> Vec<f64> {
        if gamma <= 0.0 {
            return image.to_vec();
        }
        // Normalize to [0, 1] first
        let min_val = image.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_val = image.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let range = max_val - min_val;
        if range < 1e-15 {
            return image.to_vec();
        }
        image
            .iter()
            .map(|&v| {
                let norm = ((v - min_val) / range).clamp(0.0, 1.0);
                norm.powf(gamma) * range + min_val
            })
            .collect()
    }

    /// Sobel edge detection. Returns gradient magnitude image.
    pub fn sobel_edges(image: &[f64], width: usize, height: usize) -> Vec<f64> {
        if width < 3 || height < 3 || image.len() != width * height {
            return vec![0.0; image.len()];
        }
        let mut output = vec![0.0; image.len()];
        for y in 1..height - 1 {
            for x in 1..width - 1 {
                let idx = |dx: isize, dy: isize| -> f64 {
                    image[((y as isize + dy) as usize) * width + (x as isize + dx) as usize]
                };
                // Sobel Gx kernel: [[-1,0,1],[-2,0,2],[-1,0,1]]
                let gx = -idx(-1, -1) + idx(1, -1)
                    - 2.0 * idx(-1, 0) + 2.0 * idx(1, 0)
                    - idx(-1, 1) + idx(1, 1);
                // Sobel Gy kernel: [[-1,-2,-1],[0,0,0],[1,2,1]]
                let gy = -idx(-1, -1) - 2.0 * idx(0, -1) - idx(1, -1)
                    + idx(-1, 1) + 2.0 * idx(0, 1) + idx(1, 1);
                output[y * width + x] = (gx * gx + gy * gy).sqrt();
            }
        }
        output
    }

    /// Laplacian edge detection (second derivative).
    pub fn laplacian_edges(image: &[f64], width: usize, height: usize) -> Vec<f64> {
        if width < 3 || height < 3 || image.len() != width * height {
            return vec![0.0; image.len()];
        }
        let mut output = vec![0.0; image.len()];
        for y in 1..height - 1 {
            for x in 1..width - 1 {
                let idx = y * width + x;
                let lap = image[idx - 1] + image[idx + 1]
                    + image[idx - width] + image[idx + width]
                    - 4.0 * image[idx];
                output[idx] = lap.abs();
            }
        }
        output
    }
}

// ─── Noise Reducer ──────────────────────────────────────────────────

/// SEM image noise reduction via frame averaging and Wiener filtering.
#[derive(Debug, Clone)]
pub struct NoiseReducer {
    /// Number of frames to average.
    pub num_frames: usize,
}

impl NoiseReducer {
    /// Create a noise reducer with the given number of frames.
    pub fn new(num_frames: usize) -> Self {
        Self {
            num_frames: num_frames.max(1),
        }
    }

    /// Average multiple frames to reduce random noise.
    /// SNR improvement: sqrt(N) for N frames.
    pub fn frame_average(&self, frames: &[Vec<f64>]) -> Vec<f64> {
        if frames.is_empty() {
            return vec![];
        }
        let n = frames[0].len();
        let mut avg = vec![0.0; n];
        let count = frames.len() as f64;
        for frame in frames {
            for (i, &v) in frame.iter().enumerate().take(n) {
                avg[i] += v;
            }
        }
        for v in &mut avg {
            *v /= count;
        }
        avg
    }

    /// SNR improvement factor from frame averaging.
    pub fn snr_improvement_db(num_frames: usize) -> f64 {
        10.0 * (num_frames as f64).log10()
    }

    /// Simple Wiener filter for noise reduction on a 1D signal or row.
    /// H(f) = |S(f)|^2 / (|S(f)|^2 + N), where N is noise variance.
    pub fn wiener_filter_1d(signal: &[f64], noise_variance: f64) -> Vec<f64> {
        let n = signal.len();
        if n == 0 {
            return vec![];
        }

        // Estimate local signal power using a sliding window
        let window = 5.min(n);
        let mut filtered = vec![0.0; n];
        for i in 0..n {
            let start = if i >= window / 2 { i - window / 2 } else { 0 };
            let end = (i + window / 2 + 1).min(n);
            let local_mean: f64 = signal[start..end].iter().sum::<f64>() / (end - start) as f64;
            let local_var: f64 = signal[start..end]
                .iter()
                .map(|&x| (x - local_mean).powi(2))
                .sum::<f64>()
                / (end - start) as f64;

            // Wiener estimate
            let signal_var = (local_var - noise_variance).max(0.0);
            let gain = if local_var > 1e-15 {
                signal_var / local_var
            } else {
                0.0
            };
            filtered[i] = local_mean + gain * (signal[i] - local_mean);
        }
        filtered
    }

    /// 2D Gaussian smoothing (separable, 3x3 kernel [1,2,1]/4).
    pub fn gaussian_smooth(image: &[f64], width: usize, height: usize) -> Vec<f64> {
        if width < 3 || height < 3 || image.len() != width * height {
            return image.to_vec();
        }
        // Horizontal pass
        let mut temp = vec![0.0; image.len()];
        for y in 0..height {
            for x in 1..width - 1 {
                let idx = y * width + x;
                temp[idx] = 0.25 * image[idx - 1] + 0.5 * image[idx] + 0.25 * image[idx + 1];
            }
            // Edges
            temp[y * width] = image[y * width];
            temp[y * width + width - 1] = image[y * width + width - 1];
        }
        // Vertical pass
        let mut output = vec![0.0; image.len()];
        for y in 1..height - 1 {
            for x in 0..width {
                let idx = y * width + x;
                output[idx] =
                    0.25 * temp[idx - width] + 0.5 * temp[idx] + 0.25 * temp[idx + width];
            }
        }
        // Edge rows
        for x in 0..width {
            output[x] = temp[x];
            output[(height - 1) * width + x] = temp[(height - 1) * width + x];
        }
        output
    }
}

// ─── Magnification Calibrator ───────────────────────────────────────

/// Pixel size and scale bar calculation from SEM magnification.
#[derive(Debug, Clone)]
pub struct MagnificationCalibrator {
    /// Display magnification.
    pub magnification: f64,
    /// Number of pixels across the image.
    pub num_pixels: usize,
    /// Horizontal field of view in micrometres (derived from magnification).
    /// Standard SEM display is 10 cm = 100,000 um.
    pub fov_um: f64,
}

impl MagnificationCalibrator {
    /// Create from magnification and pixel count.
    /// FOV = (display_width_um / magnification).
    /// Standard SEM display = 100 mm = 100,000 um.
    pub fn new(magnification: f64, num_pixels: usize) -> Self {
        let display_width_um = 100_000.0; // 10 cm standard
        let fov_um = display_width_um / magnification;
        Self {
            magnification,
            num_pixels,
            fov_um,
        }
    }

    /// Create from known field of view directly.
    pub fn from_fov(fov_um: f64, num_pixels: usize) -> Self {
        let display_width_um = 100_000.0;
        let magnification = display_width_um / fov_um;
        Self {
            magnification,
            num_pixels,
            fov_um,
        }
    }

    /// Pixel size in micrometres.
    pub fn pixel_size_um(&self) -> f64 {
        self.fov_um / self.num_pixels as f64
    }

    /// Pixel size in nanometres.
    pub fn pixel_size_nm(&self) -> f64 {
        self.pixel_size_um() * 1000.0
    }

    /// Number of pixels required for a given scale bar length (um).
    pub fn scale_bar_pixels(&self, length_um: f64) -> f64 {
        length_um / self.pixel_size_um()
    }

    /// Measure a feature dimension from pixel count.
    pub fn measure_um(&self, pixel_count: f64) -> f64 {
        pixel_count * self.pixel_size_um()
    }

    /// Convert a physical dimension to pixels.
    pub fn dimension_to_pixels(&self, dimension_um: f64) -> f64 {
        dimension_um / self.pixel_size_um()
    }
}

// ─── Charging Detector ──────────────────────────────────────────────

/// Detect sample charging artifacts in SEM images.
/// Charging manifests as abnormally bright regions, image drift,
/// and anomalous contrast.
#[derive(Debug, Clone)]
pub struct ChargingDetector {
    /// Brightness threshold factor above mean for charging detection.
    pub brightness_threshold: f64,
    /// Minimum fraction of bright pixels to flag image as charged.
    pub area_threshold: f64,
}

impl ChargingDetector {
    /// Create a charging detector.
    /// `brightness_threshold`: e.g., 2.0 means 2x mean brightness.
    /// `area_threshold`: fraction of pixels that must be bright, e.g. 0.05.
    pub fn new(brightness_threshold: f64, area_threshold: f64) -> Self {
        Self {
            brightness_threshold,
            area_threshold,
        }
    }

    /// Default detector: bright > 2.5x mean, area > 3%.
    pub fn default_detector() -> Self {
        Self::new(2.5, 0.03)
    }

    /// Detect charging in an image. Returns (is_charging, fraction_bright).
    pub fn detect(&self, image: &[f64]) -> (bool, f64) {
        if image.is_empty() {
            return (false, 0.0);
        }
        let mean: f64 = image.iter().sum::<f64>() / image.len() as f64;
        let threshold = mean * self.brightness_threshold;
        let bright_count = image.iter().filter(|&&v| v > threshold).count();
        let fraction = bright_count as f64 / image.len() as f64;
        (fraction > self.area_threshold, fraction)
    }

    /// Detect image drift by comparing two successive scan lines.
    /// Returns estimated horizontal drift in pixels.
    pub fn estimate_drift(line1: &[f64], line2: &[f64]) -> f64 {
        if line1.len() < 3 || line2.len() < 3 || line1.len() != line2.len() {
            return 0.0;
        }
        let n = line1.len();
        let max_shift = (n / 4).max(1);
        let mut best_shift = 0i32;
        let mut best_corr = f64::NEG_INFINITY;

        for shift in -(max_shift as i32)..=max_shift as i32 {
            let mut corr = 0.0;
            let mut count = 0u32;
            for i in 0..n {
                let j = i as i32 + shift;
                if j >= 0 && (j as usize) < n {
                    corr += line1[i] * line2[j as usize];
                    count += 1;
                }
            }
            if count > 0 {
                corr /= count as f64;
            }
            if corr > best_corr {
                best_corr = corr;
                best_shift = shift;
            }
        }
        best_shift as f64
    }

    /// Compute local contrast variation to detect contrast anomalies.
    /// Returns the coefficient of variation (std/mean) of local contrast.
    pub fn contrast_anomaly_score(image: &[f64], width: usize, height: usize) -> f64 {
        if width < 4 || height < 4 || image.len() != width * height {
            return 0.0;
        }
        let block_size = 4.min(width.min(height));
        let mut contrasts = Vec::new();

        let blocks_x = width / block_size;
        let blocks_y = height / block_size;

        for by in 0..blocks_y {
            for bx in 0..blocks_x {
                let mut block_min = f64::INFINITY;
                let mut block_max = f64::NEG_INFINITY;
                for dy in 0..block_size {
                    for dx in 0..block_size {
                        let val = image[(by * block_size + dy) * width + bx * block_size + dx];
                        block_min = block_min.min(val);
                        block_max = block_max.max(val);
                    }
                }
                if block_max + block_min > 1e-12 {
                    contrasts.push((block_max - block_min) / (block_max + block_min));
                }
            }
        }

        if contrasts.is_empty() {
            return 0.0;
        }
        let mean_c: f64 = contrasts.iter().sum::<f64>() / contrasts.len() as f64;
        let var_c: f64 = contrasts
            .iter()
            .map(|&c| (c - mean_c).powi(2))
            .sum::<f64>()
            / contrasts.len() as f64;
        if mean_c.abs() < 1e-15 {
            0.0
        } else {
            var_c.sqrt() / mean_c
        }
    }
}

// ─── Grain Size Analyzer ────────────────────────────────────────────

/// Grain/particle size measurement from SEM images.
#[derive(Debug, Clone)]
pub struct GrainSizeAnalyzer {
    /// Intensity threshold for segmentation (fraction of max, 0..1).
    pub threshold: f64,
    /// Minimum grain area in pixels.
    pub min_area_pixels: usize,
}

/// Result of grain size analysis.
#[derive(Debug, Clone)]
pub struct GrainStats {
    /// Number of grains detected.
    pub count: usize,
    /// Areas in pixels for each grain.
    pub areas: Vec<usize>,
    /// Equivalent circle diameters in pixels: d = 2*sqrt(area/pi).
    pub diameters: Vec<f64>,
    /// Mean diameter in pixels.
    pub mean_diameter: f64,
    /// Standard deviation of diameters.
    pub std_diameter: f64,
}

impl GrainSizeAnalyzer {
    /// Create a grain size analyzer.
    pub fn new(threshold: f64, min_area_pixels: usize) -> Self {
        Self {
            threshold: threshold.clamp(0.0, 1.0),
            min_area_pixels: min_area_pixels.max(1),
        }
    }

    /// Segment image into binary (grain/not-grain) using threshold.
    pub fn threshold_segment(&self, image: &[f64]) -> Vec<bool> {
        let max_val = image.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let thresh = self.threshold * max_val;
        image.iter().map(|&v| v >= thresh).collect()
    }

    /// Connected-component labeling (4-connected) to identify grains.
    /// Returns a label map where each pixel has a grain ID (0 = background).
    pub fn label_grains(
        &self,
        binary: &[bool],
        width: usize,
        height: usize,
    ) -> Vec<u32> {
        let n = width * height;
        if binary.len() != n {
            return vec![0; n];
        }
        let mut labels = vec![0u32; n];
        let mut next_label = 1u32;
        // Union-Find
        let mut parent: Vec<u32> = Vec::new();
        parent.push(0); // label 0 = background

        for y in 0..height {
            for x in 0..width {
                let idx = y * width + x;
                if !binary[idx] {
                    continue;
                }
                let mut neighbors = Vec::new();
                if x > 0 && labels[idx - 1] > 0 {
                    neighbors.push(labels[idx - 1]);
                }
                if y > 0 && labels[idx - width] > 0 {
                    neighbors.push(labels[idx - width]);
                }
                if neighbors.is_empty() {
                    labels[idx] = next_label;
                    parent.push(next_label);
                    next_label += 1;
                } else {
                    let min_label = *neighbors.iter().min().unwrap();
                    labels[idx] = min_label;
                    for &nl in &neighbors {
                        Self::union(&mut parent, min_label, nl);
                    }
                }
            }
        }

        // Flatten labels
        for i in 0..n {
            if labels[i] > 0 {
                labels[i] = Self::find(&mut parent, labels[i]);
            }
        }

        // Relabel sequentially
        let mut relabel_map = std::collections::HashMap::new();
        let mut seq = 1u32;
        for v in &mut labels {
            if *v > 0 {
                let entry = relabel_map.entry(*v).or_insert_with(|| {
                    let s = seq;
                    seq += 1;
                    s
                });
                *v = *entry;
            }
        }

        labels
    }

    fn find(parent: &mut Vec<u32>, mut x: u32) -> u32 {
        while parent[x as usize] != x {
            parent[x as usize] = parent[parent[x as usize] as usize];
            x = parent[x as usize];
        }
        x
    }

    fn union(parent: &mut Vec<u32>, a: u32, b: u32) {
        let ra = Self::find(parent, a);
        let rb = Self::find(parent, b);
        if ra != rb {
            let (small, big) = if ra < rb { (ra, rb) } else { (rb, ra) };
            parent[big as usize] = small;
        }
    }

    /// Analyze grains: segment, label, compute statistics.
    pub fn analyze(
        &self,
        image: &[f64],
        width: usize,
        height: usize,
    ) -> GrainStats {
        let binary = self.threshold_segment(image);
        let labels = self.label_grains(&binary, width, height);
        let max_label = *labels.iter().max().unwrap_or(&0);

        let mut areas_map = std::collections::HashMap::new();
        for &lbl in &labels {
            if lbl > 0 {
                *areas_map.entry(lbl).or_insert(0usize) += 1;
            }
        }

        // Filter by minimum area
        let areas: Vec<usize> = areas_map
            .values()
            .copied()
            .filter(|&a| a >= self.min_area_pixels)
            .collect();

        let diameters: Vec<f64> = areas
            .iter()
            .map(|&a| 2.0 * (a as f64 / PI).sqrt())
            .collect();

        let count = diameters.len();
        let mean_diameter = if count > 0 {
            diameters.iter().sum::<f64>() / count as f64
        } else {
            0.0
        };
        let std_diameter = if count > 1 {
            let var: f64 = diameters
                .iter()
                .map(|&d| (d - mean_diameter).powi(2))
                .sum::<f64>()
                / (count - 1) as f64;
            var.sqrt()
        } else {
            0.0
        };

        GrainStats {
            count,
            areas,
            diameters,
            mean_diameter,
            std_diameter,
        }
    }
}

// ─── Line Scan Processor ────────────────────────────────────────────

/// Line profile extraction and critical dimension (CD) measurement.
#[derive(Debug, Clone)]
pub struct LineScanProcessor {
    /// Pixel size in nanometres (for converting pixel positions to nm).
    pub pixel_size_nm: f64,
}

/// Line scan measurement result.
#[derive(Debug, Clone)]
pub struct LineScanResult {
    /// Profile values.
    pub profile: Vec<f64>,
    /// Edge positions (pixel indices) detected at threshold crossings.
    pub edge_positions: Vec<f64>,
    /// Measured critical dimensions in nanometres (distances between edge pairs).
    pub critical_dimensions_nm: Vec<f64>,
    /// Line roughness (RMS of profile after detrending).
    pub roughness_rms: f64,
}

impl LineScanProcessor {
    /// Create with known pixel size.
    pub fn new(pixel_size_nm: f64) -> Self {
        Self { pixel_size_nm }
    }

    /// Extract a horizontal line profile from an image.
    pub fn extract_horizontal(
        image: &[f64],
        width: usize,
        _height: usize,
        row: usize,
    ) -> Vec<f64> {
        let start = row * width;
        let end = start + width;
        if end <= image.len() {
            image[start..end].to_vec()
        } else {
            vec![]
        }
    }

    /// Extract a vertical line profile from an image.
    pub fn extract_vertical(
        image: &[f64],
        width: usize,
        height: usize,
        col: usize,
    ) -> Vec<f64> {
        (0..height)
            .filter_map(|y| {
                let idx = y * width + col;
                if idx < image.len() {
                    Some(image[idx])
                } else {
                    None
                }
            })
            .collect()
    }

    /// Detect edge positions in a line profile using threshold crossing.
    /// `threshold_frac`: fraction between min and max for edge detection (e.g., 0.5).
    pub fn detect_edges(&self, profile: &[f64], threshold_frac: f64) -> Vec<f64> {
        if profile.len() < 2 {
            return vec![];
        }
        let min_val = profile.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_val = profile.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let threshold = min_val + threshold_frac * (max_val - min_val);

        let mut edges = Vec::new();
        for i in 0..profile.len() - 1 {
            let crosses_up = profile[i] < threshold && profile[i + 1] >= threshold;
            let crosses_down = profile[i] >= threshold && profile[i + 1] < threshold;
            if crosses_up || crosses_down {
                // Linear interpolation for sub-pixel accuracy
                let frac = (threshold - profile[i]) / (profile[i + 1] - profile[i]);
                edges.push(i as f64 + frac);
            }
        }
        edges
    }

    /// Measure critical dimensions (CD) from a line profile.
    /// Returns pairs of edges and their separations in nm.
    pub fn measure_cd(&self, profile: &[f64], threshold_frac: f64) -> LineScanResult {
        let edges = self.detect_edges(profile, threshold_frac);
        let mut cds = Vec::new();

        // Pair consecutive edges (rising -> falling)
        let mut i = 0;
        while i + 1 < edges.len() {
            let cd_pixels = (edges[i + 1] - edges[i]).abs();
            cds.push(cd_pixels * self.pixel_size_nm);
            i += 2;
        }

        // Compute line roughness (RMS after linear detrend)
        let roughness = self.line_roughness(profile);

        LineScanResult {
            profile: profile.to_vec(),
            edge_positions: edges,
            critical_dimensions_nm: cds,
            roughness_rms: roughness,
        }
    }

    /// Line roughness: RMS of residuals after removing linear trend.
    pub fn line_roughness(&self, profile: &[f64]) -> f64 {
        let n = profile.len();
        if n < 2 {
            return 0.0;
        }
        // Fit linear trend y = a + b*x
        let n_f = n as f64;
        let sum_x: f64 = (0..n).map(|i| i as f64).sum();
        let sum_y: f64 = profile.iter().sum();
        let sum_xy: f64 = (0..n).map(|i| i as f64 * profile[i]).sum();
        let sum_xx: f64 = (0..n).map(|i| (i as f64).powi(2)).sum();

        let denom = n_f * sum_xx - sum_x * sum_x;
        let (a, b) = if denom.abs() > 1e-15 {
            let b = (n_f * sum_xy - sum_x * sum_y) / denom;
            let a = (sum_y - b * sum_x) / n_f;
            (a, b)
        } else {
            (sum_y / n_f, 0.0)
        };

        let rms_sq: f64 = (0..n)
            .map(|i| {
                let residual = profile[i] - (a + b * i as f64);
                residual * residual
            })
            .sum::<f64>()
            / n_f;
        rms_sq.sqrt()
    }
}

// ─── Depth of Field Calculator ──────────────────────────────────────

/// SEM depth of field estimation.
/// DOF = 2 * r_beam / alpha + pixel_size
/// where alpha is the beam convergence half-angle.
#[derive(Debug, Clone)]
pub struct DepthOfFieldCalculator;

impl DepthOfFieldCalculator {
    /// Calculate depth of field in micrometres.
    ///
    /// * `beam_radius_nm` - probe radius in nm
    /// * `convergence_half_angle_rad` - beam half-angle alpha (radians)
    /// * `pixel_size_um` - pixel size in um (sets resolution limit)
    pub fn depth_of_field_um(
        beam_radius_nm: f64,
        convergence_half_angle_rad: f64,
        pixel_size_um: f64,
    ) -> f64 {
        let r_beam_um = beam_radius_nm / 1000.0;
        if convergence_half_angle_rad <= 0.0 {
            return f64::INFINITY;
        }
        2.0 * r_beam_um / convergence_half_angle_rad + pixel_size_um
    }

    /// DOF from SEM config and magnification calibrator.
    pub fn from_config(config: &SemConfig, cal: &MagnificationCalibrator) -> f64 {
        let beam_nm = config.estimated_beam_diameter_nm() / 2.0;
        let alpha = config.convergence_half_angle_rad();
        let pixel_um = cal.pixel_size_um();
        Self::depth_of_field_um(beam_nm, alpha, pixel_um)
    }

    /// Practical DOF approximation: DOF ~ 0.2 / (M * alpha)
    /// where M is magnification and alpha is in radians.
    /// Returns DOF in mm.
    pub fn practical_dof_mm(magnification: f64, convergence_half_angle_mrad: f64) -> f64 {
        let alpha_rad = convergence_half_angle_mrad * 1e-3;
        if magnification * alpha_rad <= 0.0 {
            return f64::INFINITY;
        }
        0.2 / (magnification * alpha_rad)
    }
}

// ─── Beam-Sample Interaction ────────────────────────────────────────

/// Beam-sample interaction volume calculations.
pub struct InteractionVolume;

impl InteractionVolume {
    /// Kanaya-Okayama penetration range in micrometres.
    /// R = 0.0276 * A * E^1.67 / (Z^0.89 * rho)
    pub fn kanaya_okayama_range_um(
        atomic_weight: f64,
        atomic_number: u32,
        density_g_cm3: f64,
        beam_energy_kev: f64,
    ) -> f64 {
        let z = atomic_number as f64;
        0.0276 * atomic_weight * beam_energy_kev.powf(1.67) / (z.powf(0.89) * density_g_cm3)
    }

    /// Electron beam energy in Joules from keV.
    pub fn energy_joules(kev: f64) -> f64 {
        kev * 1000.0 * ELECTRON_CHARGE
    }

    /// Relativistic electron velocity (fraction of c).
    pub fn electron_velocity_fraction_c(kev: f64) -> f64 {
        let e_j = Self::energy_joules(kev);
        let rest_energy = ELECTRON_MASS * 2.997_924_58e8 * 2.997_924_58e8;
        let gamma = 1.0 + e_j / rest_energy;
        (1.0 - 1.0 / (gamma * gamma)).sqrt()
    }

    /// Bethe stopping power approximation (keV/nm) for electron energy loss.
    /// -dE/ds = (78500 * Z * rho) / (A * E) * ln(1.166 * E / J)
    /// where J = mean ionization potential in keV.
    pub fn bethe_stopping_power(
        atomic_number: u32,
        atomic_weight: f64,
        density_g_cm3: f64,
        beam_energy_kev: f64,
    ) -> f64 {
        let z = atomic_number as f64;
        // Mean ionization potential (Berger-Seltzer): J = (9.76*Z + 58.5/Z^0.19) eV
        let j_ev = 9.76 * z + 58.5 / z.powf(0.19);
        let j_kev = j_ev / 1000.0;
        let e = beam_energy_kev;
        if e <= j_kev || e <= 0.0 {
            return 0.0;
        }
        let arg = 1.166 * e / j_kev;
        if arg <= 0.0 {
            return 0.0;
        }
        78500.0 * z * density_g_cm3 / (atomic_weight * e) * arg.ln()
    }

    /// Interaction volume lateral extent ~ 0.6 * R_ko.
    pub fn lateral_extent_um(
        atomic_weight: f64,
        atomic_number: u32,
        density_g_cm3: f64,
        beam_energy_kev: f64,
    ) -> f64 {
        0.6 * Self::kanaya_okayama_range_um(
            atomic_weight,
            atomic_number,
            density_g_cm3,
            beam_energy_kev,
        )
    }
}

// ═══════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    // --- SemConfig tests ---

    #[test]
    fn test_sem_config_creation() {
        let config = SemConfig::new(20.0, 100e-12, 10.0, DetectorType::SecondaryElectron);
        assert!((config.beam_energy_kev - 20.0).abs() < 1e-9);
        assert!((config.beam_current_a - 100e-12).abs() < 1e-20);
        assert!((config.working_distance_mm - 10.0).abs() < 1e-9);
        assert_eq!(config.detector, DetectorType::SecondaryElectron);
    }

    #[test]
    fn test_sem_presets() {
        let std = SemConfig::standard_imaging();
        assert!((std.beam_energy_kev - 20.0).abs() < 1e-9);

        let lv = SemConfig::low_voltage();
        assert!((lv.beam_energy_kev - 1.0).abs() < 1e-9);

        let edx = SemConfig::edx_analysis();
        assert_eq!(edx.detector, DetectorType::Eds);
        assert!(edx.beam_current_a > 500e-12); // Higher current for EDX
    }

    #[test]
    fn test_kanaya_okayama_silicon() {
        let config = SemConfig::standard_imaging();
        // Si: A=28.086, Z=14, rho=2.33
        let range = config.kanaya_okayama_range_um(28.086, 14, 2.33);
        // At 20 keV in Si, range ~ 4 um
        assert!(range > 1.0 && range < 10.0, "Si range = {} um", range);
    }

    #[test]
    fn test_kanaya_okayama_gold() {
        let config = SemConfig::standard_imaging();
        // Au: A=196.97, Z=79, rho=19.3
        let range = config.kanaya_okayama_range_um(196.97, 79, 19.3);
        // Au at 20 keV: denser, so shorter range
        assert!(range > 0.1 && range < 3.0, "Au range = {} um", range);
    }

    #[test]
    fn test_convergence_angle() {
        let config = SemConfig::standard_imaging();
        let alpha = config.convergence_half_angle_rad();
        assert!(alpha > 0.0 && alpha < 0.1, "alpha = {} rad", alpha);
    }

    #[test]
    fn test_beam_diameter() {
        let config = SemConfig::standard_imaging();
        let d = config.estimated_beam_diameter_nm();
        // FE-SEM at 100 pA should give nm-scale probe
        assert!(d > 0.1 && d < 100.0, "beam diameter = {} nm", d);
    }

    // --- BackscatterDetector tests ---

    #[test]
    fn test_bse_coefficient_copper() {
        let bse = BackscatterDetector::new();
        let eta = bse.backscatter_coefficient(29); // Cu
        // eta(Cu) ~ 0.3
        assert!(eta > 0.2 && eta < 0.4, "eta(Cu) = {}", eta);
    }

    #[test]
    fn test_bse_coefficient_carbon() {
        let bse = BackscatterDetector::new();
        let eta = bse.backscatter_coefficient(6); // C
        // eta(C) ~ 0.06
        assert!(eta > 0.01 && eta < 0.15, "eta(C) = {}", eta);
    }

    #[test]
    fn test_bse_coefficient_gold() {
        let bse = BackscatterDetector::new();
        let eta = bse.backscatter_coefficient(79); // Au
        // eta(Au) ~ 0.5
        assert!(eta > 0.4 && eta < 0.6, "eta(Au) = {}", eta);
    }

    #[test]
    fn test_bse_monotonic_increase() {
        let bse = BackscatterDetector::new();
        let mut prev = 0.0;
        // BSE coefficient should generally increase with Z
        for z in [6, 13, 26, 29, 47, 79] {
            let eta = bse.backscatter_coefficient(z);
            assert!(eta > prev, "eta(Z={}) = {} <= prev {}", z, eta, prev);
            prev = eta;
        }
    }

    #[test]
    fn test_eta_to_z_roundtrip() {
        let bse = BackscatterDetector::new();
        for z in [10, 20, 30, 50, 79] {
            let eta = bse.backscatter_coefficient(z);
            let z_est = bse.eta_to_z(eta);
            assert!(
                (z_est - z as f64).abs() < 0.5,
                "Z={}, eta={}, z_est={}",
                z, eta, z_est
            );
        }
    }

    #[test]
    fn test_z_contrast_ratio() {
        let bse = BackscatterDetector::new();
        let ratio = bse.z_contrast_ratio(79, 14); // Au vs Si
        assert!(ratio > 1.0, "Au/Si BSE contrast ratio should be > 1");
    }

    #[test]
    fn test_z_contrast_image() {
        let bse = BackscatterDetector::new();
        let img = vec![0.1, 0.3, 0.5, 0.2];
        let norm = bse.z_contrast_image(&img);
        assert!((norm[0] - 0.0).abs() < 1e-9); // min maps to 0
        assert!((norm[2] - 1.0).abs() < 1e-9); // max maps to 1
    }

    // --- SecondaryElectronDetector tests ---

    #[test]
    fn test_se_yield() {
        let se = SecondaryElectronDetector::new(0.5);
        // Silicon: delta_max ~ 1.2, E_max ~ 0.4 keV
        let yield_1kev = se.se_yield(1.0, 1.2, 0.4);
        let yield_20kev = se.se_yield(20.0, 1.2, 0.4);
        // SE yield decreases at higher beam energy
        assert!(yield_1kev > yield_20kev, "SE yield should decrease with energy");
        assert!(yield_1kev > 0.0);
    }

    #[test]
    fn test_se_topography_enhancement() {
        let se = SecondaryElectronDetector::new(0.5);
        // Create a simple 5x5 image with an edge
        let mut image = vec![0.0; 25];
        for y in 0..5 {
            for x in 0..5 {
                image[y * 5 + x] = if x >= 3 { 1.0 } else { 0.0 };
            }
        }
        let enhanced = se.enhance_topography(&image, 5, 5);
        assert_eq!(enhanced.len(), 25);
        // The edge region should show enhancement
        let edge_val = enhanced[2 * 5 + 2]; // just before the edge
        assert!(edge_val >= 0.0); // Should not go negative
    }

    // --- EdxSpectrumProcessor tests ---

    #[test]
    fn test_edx_channel_energy_conversion() {
        let edx = EdxSpectrumProcessor::standard();
        assert!((edx.channel_to_energy(100) - 1.0).abs() < 0.01);
        assert_eq!(edx.energy_to_channel(1.0), 100);
    }

    #[test]
    fn test_edx_background_subtraction() {
        let edx = EdxSpectrumProcessor::standard();
        // Create a spectrum with a background + peak
        let mut spectrum = vec![10.0; 200];
        // Add a Gaussian peak at channel 100
        for i in 90..110 {
            let x = (i as f64 - 100.0) / 3.0;
            spectrum[i] += 50.0 * (-0.5 * x * x).exp();
        }
        let net = edx.subtract_background(&spectrum, 20);
        // Peak should survive, background should be reduced
        let peak_val = net[100];
        let bg_val = net[50];
        assert!(peak_val > bg_val, "Peak should remain after bg subtraction");
    }

    #[test]
    fn test_edx_find_peaks() {
        let edx = EdxSpectrumProcessor::standard();
        let mut spectrum = vec![1.0; 100];
        spectrum[50] = 100.0;
        spectrum[75] = 80.0;
        let peaks = edx.find_peaks(&spectrum, 10.0);
        assert_eq!(peaks.len(), 2);
        assert_eq!(peaks[0].0, 50);
        assert_eq!(peaks[1].0, 75);
    }

    #[test]
    fn test_edx_identify_iron() {
        let edx = EdxSpectrumProcessor::new(0.010, 1024);
        // Create a spectrum with an Fe K-alpha peak at 6.404 keV
        let mut spectrum = vec![5.0; 1024];
        let fe_ch = edx.energy_to_channel(6.404);
        // Add a Gaussian peak
        for i in 0..1024 {
            let x = (i as f64 - fe_ch as f64) / 5.0;
            spectrum[i] += 200.0 * (-0.5 * x * x).exp();
        }
        let elements = edx.identify_elements(&spectrum, 20.0);
        let has_fe = elements.iter().any(|e| e.element == "Fe");
        assert!(has_fe, "Should identify Fe K-alpha");
    }

    #[test]
    fn test_duane_hunt_limit() {
        assert!((EdxSpectrumProcessor::duane_hunt_limit_kev(20.0) - 20.0).abs() < 1e-9);
    }

    #[test]
    fn test_overvoltage_ratio() {
        let u = EdxSpectrumProcessor::overvoltage_ratio(20.0, 6.404);
        assert!(u > 2.0, "Fe K-alpha overvoltage at 20 keV should be > 2");
    }

    // --- ImageContrastEnhancer tests ---

    #[test]
    fn test_histogram_equalization() {
        let image = vec![0.0, 0.1, 0.2, 0.3, 0.5, 0.7, 0.8, 1.0];
        let eq = ImageContrastEnhancer::histogram_equalize(&image);
        assert_eq!(eq.len(), 8);
        // Last element should map to ~1.0
        assert!((eq[7] - 1.0).abs() < 0.01);
        // First element should map to ~0.0
        assert!(eq[0] < 0.15);
    }

    #[test]
    fn test_gamma_correction() {
        let image = vec![0.0, 0.25, 0.5, 0.75, 1.0];
        let bright = ImageContrastEnhancer::gamma_correct(&image, 0.5);
        // gamma < 1 brightens: middle values should increase
        assert!(bright[2] > image[2]);

        let dark = ImageContrastEnhancer::gamma_correct(&image, 2.0);
        // gamma > 1 darkens: middle values should decrease
        assert!(dark[2] < image[2]);
    }

    #[test]
    fn test_sobel_edges() {
        // 5x5 image with vertical edge
        let mut image = vec![0.0; 25];
        for y in 0..5 {
            for x in 3..5 {
                image[y * 5 + x] = 1.0;
            }
        }
        let edges = ImageContrastEnhancer::sobel_edges(&image, 5, 5);
        // Edge should be strongest near x=2-3
        let edge_strength = edges[2 * 5 + 2];
        let flat_strength = edges[2 * 5 + 0];
        assert!(edge_strength > flat_strength, "Edge should be detected at boundary");
    }

    #[test]
    fn test_laplacian_edges() {
        let mut image = vec![0.0; 25];
        image[2 * 5 + 2] = 1.0; // Single bright pixel
        let edges = ImageContrastEnhancer::laplacian_edges(&image, 5, 5);
        // Laplacian of a point source should be negative at center, positive around it
        assert!(edges[2 * 5 + 2] > 0.0, "Laplacian should respond to point source");
    }

    // --- NoiseReducer tests ---

    #[test]
    fn test_frame_averaging() {
        let nr = NoiseReducer::new(4);
        let frame1 = vec![10.0, 20.0, 30.0];
        let frame2 = vec![12.0, 22.0, 28.0];
        let frame3 = vec![8.0, 18.0, 32.0];
        let avg = nr.frame_average(&[frame1, frame2, frame3]);
        assert!((avg[0] - 10.0).abs() < 0.01);
        assert!((avg[1] - 20.0).abs() < 0.01);
        assert!((avg[2] - 30.0).abs() < 0.01);
    }

    #[test]
    fn test_snr_improvement() {
        let db4 = NoiseReducer::snr_improvement_db(4);
        assert!((db4 - 6.02).abs() < 0.1, "4 frames => ~6 dB improvement");
        let db16 = NoiseReducer::snr_improvement_db(16);
        assert!((db16 - 12.04).abs() < 0.1, "16 frames => ~12 dB improvement");
    }

    #[test]
    fn test_wiener_filter() {
        // Signal with added noise
        let signal: Vec<f64> = (0..50).map(|i| (i as f64 * 0.1).sin()).collect();
        let filtered = NoiseReducer::wiener_filter_1d(&signal, 0.01);
        assert_eq!(filtered.len(), 50);
        // Filtered signal should be close to original for low noise
        let mse: f64 = signal
            .iter()
            .zip(filtered.iter())
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            / signal.len() as f64;
        assert!(mse < 1.0, "Wiener filter should not distort low-noise signal much");
    }

    #[test]
    fn test_gaussian_smooth() {
        let mut image = vec![0.0; 25];
        image[2 * 5 + 2] = 100.0; // Impulse
        let smoothed = NoiseReducer::gaussian_smooth(&image, 5, 5);
        // Peak should be reduced
        assert!(smoothed[2 * 5 + 2] < 100.0);
        // Neighbors should gain some energy
        assert!(smoothed[2 * 5 + 1] > 0.0 || smoothed[1 * 5 + 2] > 0.0);
    }

    // --- MagnificationCalibrator tests ---

    #[test]
    fn test_magnification_pixel_size() {
        let cal = MagnificationCalibrator::new(10_000.0, 1024);
        let ps = cal.pixel_size_um();
        // FOV at 10kx = 100000/10000 = 10 um, pixel = 10/1024 ~ 0.00977 um
        assert!((ps - 10.0 / 1024.0).abs() < 1e-6);
    }

    #[test]
    fn test_magnification_scale_bar() {
        let cal = MagnificationCalibrator::new(10_000.0, 1024);
        let bar_px = cal.scale_bar_pixels(1.0); // 1 um scale bar
        let expected = 1.0 / cal.pixel_size_um();
        assert!((bar_px - expected).abs() < 1e-3);
    }

    #[test]
    fn test_magnification_from_fov() {
        let cal = MagnificationCalibrator::from_fov(100.0, 1024);
        assert!((cal.magnification - 1000.0).abs() < 1e-6);
        assert!((cal.pixel_size_um() - 100.0 / 1024.0).abs() < 1e-9);
    }

    #[test]
    fn test_measure_dimension() {
        let cal = MagnificationCalibrator::new(1000.0, 1024);
        let dim = cal.measure_um(100.0); // 100 pixels
        let expected = 100.0 * cal.pixel_size_um();
        assert!((dim - expected).abs() < 1e-9);
    }

    // --- ChargingDetector tests ---

    #[test]
    fn test_charging_detection_normal() {
        let det = ChargingDetector::default_detector();
        let image = vec![50.0; 100]; // Uniform image
        let (is_charging, frac) = det.detect(&image);
        assert!(!is_charging, "Uniform image should not show charging");
        assert!(frac < 0.01);
    }

    #[test]
    fn test_charging_detection_charged() {
        let det = ChargingDetector::default_detector();
        let mut image = vec![10.0; 100];
        // Add bright charging patches (> 2.5x mean)
        for i in 0..10 {
            image[i] = 200.0;
        }
        let (is_charging, frac) = det.detect(&image);
        assert!(is_charging, "Image with bright patches should show charging");
        assert!(frac > 0.03);
    }

    #[test]
    fn test_drift_estimation() {
        let line1: Vec<f64> = (0..64).map(|i| if i > 30 { 1.0 } else { 0.0 }).collect();
        // Shift by 2 pixels
        let line2: Vec<f64> = (0..64).map(|i| if i > 32 { 1.0 } else { 0.0 }).collect();
        let drift = ChargingDetector::estimate_drift(&line1, &line2);
        // Should detect ~2 pixel shift
        assert!((drift - 2.0).abs() < 2.0, "drift = {}", drift);
    }

    #[test]
    fn test_contrast_anomaly() {
        // Uniform image should have low anomaly score
        let uniform = vec![50.0; 64];
        let score = ChargingDetector::contrast_anomaly_score(&uniform, 8, 8);
        assert!(score < 0.1, "Uniform image anomaly = {}", score);
    }

    // --- GrainSizeAnalyzer tests ---

    #[test]
    fn test_grain_threshold_segment() {
        let analyzer = GrainSizeAnalyzer::new(0.5, 1);
        let image = vec![0.2, 0.8, 0.3, 0.9];
        let binary = analyzer.threshold_segment(&image);
        assert!(!binary[0]); // 0.2 < 0.5*0.9
        assert!(binary[1]); // 0.8 >= 0.45
        assert!(!binary[2]);
        assert!(binary[3]);
    }

    #[test]
    fn test_grain_labeling() {
        let analyzer = GrainSizeAnalyzer::new(0.5, 1);
        // 4x4 image with two distinct grains
        #[rustfmt::skip]
        let binary = vec![
            true, true, false, false,
            true, true, false, false,
            false, false, false, false,
            false, false, true, true,
        ];
        let labels = analyzer.label_grains(&binary, 4, 4);
        // Grain 1: top-left block, Grain 2: bottom-right block
        assert!(labels[0] > 0);
        assert_eq!(labels[0], labels[1]);
        assert_eq!(labels[0], labels[4]);
        assert_eq!(labels[14], labels[15]);
        assert_ne!(labels[0], labels[14]); // Different grains
    }

    #[test]
    fn test_grain_analysis() {
        let analyzer = GrainSizeAnalyzer::new(0.5, 1);
        // 6x6 image with one ~9-pixel grain
        let mut image = vec![0.0; 36];
        for y in 1..4 {
            for x in 1..4 {
                image[y * 6 + x] = 1.0;
            }
        }
        let stats = analyzer.analyze(&image, 6, 6);
        assert_eq!(stats.count, 1);
        assert_eq!(stats.areas[0], 9);
        // ECD = 2*sqrt(9/pi) ~ 3.385
        assert!((stats.diameters[0] - 3.385).abs() < 0.01);
    }

    // --- LineScanProcessor tests ---

    #[test]
    fn test_line_extract_horizontal() {
        let image = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        let line = LineScanProcessor::extract_horizontal(&image, 3, 3, 1);
        assert_eq!(line, vec![4.0, 5.0, 6.0]);
    }

    #[test]
    fn test_line_extract_vertical() {
        let image = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        let line = LineScanProcessor::extract_vertical(&image, 3, 3, 1);
        assert_eq!(line, vec![2.0, 5.0, 8.0]);
    }

    #[test]
    fn test_edge_detection() {
        let lsp = LineScanProcessor::new(10.0);
        // Step profile: 0,0,0,1,1,1
        let profile = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let edges = lsp.detect_edges(&profile, 0.5);
        assert_eq!(edges.len(), 1);
        assert!((edges[0] - 2.5).abs() < 0.1, "edge at {}", edges[0]);
    }

    #[test]
    fn test_cd_measurement() {
        let lsp = LineScanProcessor::new(10.0); // 10 nm/pixel
        // Profile with one line: low-high-low
        let profile = vec![0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0];
        let result = lsp.measure_cd(&profile, 0.5);
        assert!(!result.critical_dimensions_nm.is_empty());
        // CD should be ~ 3 pixels * 10 nm = 30 nm (approximately)
        let cd = result.critical_dimensions_nm[0];
        assert!(cd > 20.0 && cd < 50.0, "CD = {} nm", cd);
    }

    #[test]
    fn test_line_roughness() {
        let lsp = LineScanProcessor::new(1.0);
        // Perfectly linear profile: zero roughness
        let linear: Vec<f64> = (0..10).map(|i| 2.0 + 0.5 * i as f64).collect();
        let rms = lsp.line_roughness(&linear);
        assert!(rms < 1e-10, "Linear profile roughness = {}", rms);
    }

    // --- DepthOfFieldCalculator tests ---

    #[test]
    fn test_depth_of_field() {
        let dof = DepthOfFieldCalculator::depth_of_field_um(5.0, 0.003, 0.01);
        // DOF = 2*0.005/0.003 + 0.01 ~ 3.34 um
        let expected = 2.0 * 0.005 / 0.003 + 0.01;
        assert!((dof - expected).abs() < 1e-6, "DOF = {} um", dof);
    }

    #[test]
    fn test_practical_dof() {
        let dof = DepthOfFieldCalculator::practical_dof_mm(10_000.0, 5.0);
        // DOF = 0.2 / (10000 * 0.005) = 0.004 mm
        assert!((dof - 0.004).abs() < 1e-6, "practical DOF = {} mm", dof);
    }

    #[test]
    fn test_dof_from_config() {
        let config = SemConfig::standard_imaging();
        let cal = MagnificationCalibrator::new(10_000.0, 1024);
        let dof = DepthOfFieldCalculator::from_config(&config, &cal);
        assert!(dof > 0.0 && dof < 1000.0, "DOF from config = {} um", dof);
    }

    // --- InteractionVolume tests ---

    #[test]
    fn test_interaction_volume_silicon() {
        let range = InteractionVolume::kanaya_okayama_range_um(28.086, 14, 2.33, 20.0);
        assert!(range > 1.0 && range < 10.0, "Si range = {} um", range);
    }

    #[test]
    fn test_electron_energy_joules() {
        let j = InteractionVolume::energy_joules(20.0);
        let expected = 20.0 * 1000.0 * ELECTRON_CHARGE;
        assert!((j - expected).abs() < 1e-25);
    }

    #[test]
    fn test_electron_velocity() {
        let v = InteractionVolume::electron_velocity_fraction_c(20.0);
        // 20 keV electrons: ~27% speed of light
        assert!(v > 0.2 && v < 0.4, "v/c = {}", v);
    }

    #[test]
    fn test_bethe_stopping_power() {
        let sp = InteractionVolume::bethe_stopping_power(14, 28.086, 2.33, 20.0);
        assert!(sp > 0.0, "Stopping power should be positive");
    }

    #[test]
    fn test_lateral_extent() {
        let lateral = InteractionVolume::lateral_extent_um(28.086, 14, 2.33, 20.0);
        let range = InteractionVolume::kanaya_okayama_range_um(28.086, 14, 2.33, 20.0);
        assert!((lateral - 0.6 * range).abs() < 1e-9);
    }
}
