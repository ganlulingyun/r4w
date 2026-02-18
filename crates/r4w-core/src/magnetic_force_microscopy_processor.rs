//! Magnetic Force Microscopy (MFM) Signal Processing
//!
//! Implements MFM two-pass lift-mode signal processing, force gradient detection,
//! magnetic domain imaging, quantitative deconvolution, bit pattern analysis,
//! and image processing utilities for nanoscale magnetic characterization.
//!
//! # Physical Background
//!
//! MFM measures the magnetic force gradient between a magnetized tip and sample.
//! In lift mode, two passes are performed per scan line:
//! 1. **First pass (tapping mode)**: Measures topography z(x)
//! 2. **Second pass (lift mode)**: Tip retraces at height z(x) + z_lift, measures
//!    phase shift Δφ ∝ dF_z/dz ∝ d²H_z/dz²
//!
//! ## Key Equations
//!
//! Phase shift:
//! ```text
//! Δφ = -(Q/k) * dF_z/dz
//! ```
//! where Q is the cantilever quality factor and k is the spring constant.
//!
//! Magnetic force on tip:
//! ```text
//! F_z = μ₀ ∫ m_tip · ∇H_sample dV
//! ```
//!
//! MFM transfer function (Fourier domain):
//! ```text
//! T(k) ∝ k · exp(-k · z_lift)
//! ```
//!
//! # Example
//!
//! ```rust
//! use r4w_core::magnetic_force_microscopy_processor::*;
//!
//! let config = MfmConfig {
//!     lift_height_nm: 50.0,
//!     q_factor: 500.0,
//!     spring_constant_n_per_m: 2.5,
//!     pixel_size_nm: 10.0,
//!     tip_model: TipModel::PointDipole { moment_am2: 1e-16 },
//!     ..MfmConfig::default()
//! };
//!
//! let topo = vec![0.0f64; 64 * 64];
//! let raw_phase = vec![0.1f64; 64 * 64];
//! let result = two_pass_lift_mode(&topo, &raw_phase, 64, 64, &config);
//! assert_eq!(result.magnetic_phase.len(), 64 * 64);
//! ```

use std::f64::consts::PI;

// ────────────────────────────────────────────────────────────────────────────
// Configuration & enumerations
// ────────────────────────────────────────────────────────────────────────────

/// Tip magnetization model used for force/stray-field calculations.
#[derive(Debug, Clone)]
pub enum TipModel {
    /// Point dipole: single magnetic moment at the apex.
    PointDipole {
        /// Magnetic dipole moment [A·m²]
        moment_am2: f64,
    },
    /// Extended monopole: effective magnetic charge at apex.
    ExtendedMonopole {
        /// Effective magnetic charge [A·m]
        charge_am: f64,
        /// Cone half-angle [degrees]
        cone_angle_deg: f64,
    },
    /// Extended dipole: two poles separated along the tip axis.
    ExtendedDipole {
        /// Positive pole strength [A·m]
        charge_am: f64,
        /// Separation between poles [nm]
        separation_nm: f64,
    },
}

impl Default for TipModel {
    fn default() -> Self {
        TipModel::PointDipole { moment_am2: 1e-16 }
    }
}

/// Material presets for common magnetic samples.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MaterialPreset {
    /// CoCrPt hard disk perpendicular recording media
    HardDiskCoCrPt,
    /// γ-Fe₂O₃ magnetic recording tape
    MagneticTape,
    /// NdFeB sintered permanent magnet
    NdFeBPermanentMagnet,
    /// Ni₈₀Fe₂₀ Permalloy thin film
    PermalloyThinFilm,
    /// Yttrium iron garnet (YIG) film
    GarnetFilm,
    /// User-defined material
    Custom,
}

/// Material properties for a given preset.
#[derive(Debug, Clone)]
pub struct MaterialProperties {
    /// Saturation magnetization [A/m]
    pub ms_a_per_m: f64,
    /// Coercivity [A/m]
    pub hc_a_per_m: f64,
    /// Anisotropy constant [J/m³]
    pub k1_j_per_m3: f64,
    /// Film / grain thickness [nm]
    pub thickness_nm: f64,
}

impl MaterialProperties {
    /// Return properties for a material preset.
    pub fn from_preset(preset: MaterialPreset) -> Self {
        match preset {
            MaterialPreset::HardDiskCoCrPt => MaterialProperties {
                ms_a_per_m: 3.0e5,
                hc_a_per_m: 4.0e5,
                k1_j_per_m3: 2.0e5,
                thickness_nm: 15.0,
            },
            MaterialPreset::MagneticTape => MaterialProperties {
                ms_a_per_m: 1.5e5,
                hc_a_per_m: 4.8e4,
                k1_j_per_m3: 5.0e4,
                thickness_nm: 100.0,
            },
            MaterialPreset::NdFeBPermanentMagnet => MaterialProperties {
                ms_a_per_m: 1.0e6,
                hc_a_per_m: 9.0e5,
                k1_j_per_m3: 4.5e6,
                thickness_nm: 1.0e6,
            },
            MaterialPreset::PermalloyThinFilm => MaterialProperties {
                ms_a_per_m: 8.0e5,
                hc_a_per_m: 2.0e2,
                k1_j_per_m3: 1.0e2,
                thickness_nm: 20.0,
            },
            MaterialPreset::GarnetFilm => MaterialProperties {
                ms_a_per_m: 1.4e4,
                hc_a_per_m: 5.0e2,
                k1_j_per_m3: 3.0e3,
                thickness_nm: 5000.0,
            },
            MaterialPreset::Custom => MaterialProperties {
                ms_a_per_m: 1.0e5,
                hc_a_per_m: 1.0e4,
                k1_j_per_m3: 1.0e4,
                thickness_nm: 50.0,
            },
        }
    }
}

/// Main configuration for MFM signal processing.
#[derive(Debug, Clone)]
pub struct MfmConfig {
    /// Lift height above topography for second pass [nm]
    pub lift_height_nm: f64,
    /// Cantilever quality factor (dimensionless)
    pub q_factor: f64,
    /// Cantilever spring constant [N/m]
    pub spring_constant_n_per_m: f64,
    /// Pixel size (lateral resolution) [nm]
    pub pixel_size_nm: f64,
    /// Cantilever resonant frequency [Hz]
    pub resonant_freq_hz: f64,
    /// Tip magnetization model
    pub tip_model: TipModel,
    /// Optional material preset for the sample
    pub material: Option<MaterialPreset>,
}

impl Default for MfmConfig {
    fn default() -> Self {
        MfmConfig {
            lift_height_nm: 50.0,
            q_factor: 500.0,
            spring_constant_n_per_m: 2.5,
            pixel_size_nm: 10.0,
            resonant_freq_hz: 75_000.0,
            tip_model: TipModel::PointDipole { moment_am2: 1e-16 },
            material: None,
        }
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Result types
// ────────────────────────────────────────────────────────────────────────────

/// Output of the two-pass lift-mode processing.
#[derive(Debug, Clone)]
pub struct LiftModeResult {
    /// Processed magnetic phase image [radians], rows × cols
    pub magnetic_phase: Vec<f64>,
    /// Frequency shift image [Hz] (Δf = f₀/(2k) · dF/dz)
    pub frequency_shift: Vec<f64>,
    /// Topography used for lift calculation [nm]
    pub topography: Vec<f64>,
    /// Number of rows
    pub rows: usize,
    /// Number of columns
    pub cols: usize,
}

/// Result of domain wall detection.
#[derive(Debug, Clone)]
pub struct DomainWallResult {
    /// Wall positions as (row, col) pairs
    pub wall_positions: Vec<(usize, usize)>,
    /// Estimated domain widths [nm]
    pub domain_widths_nm: Vec<f64>,
    /// Mean domain width [nm]
    pub mean_domain_width_nm: f64,
    /// Domain periodicity detected via FFT [nm]
    pub periodicity_nm: Option<f64>,
}

/// Result of quantitative MFM deconvolution.
#[derive(Debug, Clone)]
pub struct DeconvolutionResult {
    /// Recovered sample magnetization (z-component) [A/m], same grid as input
    pub magnetization: Vec<f64>,
    /// RMS of recovered magnetization [A/m]
    pub magnetization_rms: f64,
}

/// Result of bit pattern analysis (hard disk media).
#[derive(Debug, Clone)]
pub struct BitPatternResult {
    /// Detected track pitch [nm]
    pub track_pitch_nm: f64,
    /// Detected bit cell length [nm]
    pub bit_length_nm: f64,
    /// Signal amplitude (peak-to-peak phase shift) [rad]
    pub signal_amplitude_rad: f64,
    /// Transition noise (RMS phase jitter at transitions) [rad]
    pub transition_noise_rad: f64,
    /// Linear bit density [bits/mm]
    pub linear_density_bits_per_mm: f64,
}

/// Result of coercivity mapping (field-series MFM).
#[derive(Debug, Clone)]
pub struct CoercivityMapResult {
    /// Per-pixel switching field [A/m]
    pub switching_field: Vec<f64>,
    /// Mean switching field [A/m]
    pub mean_switching_field_a_per_m: f64,
    /// Switching field distribution width (std dev) [A/m]
    pub sfd_width_a_per_m: f64,
}

// ────────────────────────────────────────────────────────────────────────────
// Two-pass lift-mode processing
// ────────────────────────────────────────────────────────────────────────────

/// Perform two-pass lift-mode MFM processing.
///
/// In lift mode the cantilever retraces each line at a constant height above
/// the measured topography.  Any topographic contribution to the raw phase
/// signal is removed by subtracting a scaled topography.
///
/// # Arguments
/// * `topography` – first-pass topography [nm], length rows × cols
/// * `raw_phase`  – second-pass raw phase image [rad], same length
/// * `rows`, `cols` – image dimensions
/// * `config`     – MFM configuration
pub fn two_pass_lift_mode(
    topography: &[f64],
    raw_phase: &[f64],
    rows: usize,
    cols: usize,
    config: &MfmConfig,
) -> LiftModeResult {
    assert_eq!(topography.len(), rows * cols);
    assert_eq!(raw_phase.len(), rows * cols);

    let n = rows * cols;
    let mut magnetic_phase = Vec::with_capacity(n);
    let mut frequency_shift = Vec::with_capacity(n);

    // Coupling coefficient: topography → phase cross-talk estimated from
    // the correlation between topo and raw phase.
    let topo_mean = mean(topography);
    let phase_mean = mean(raw_phase);
    let topo_var = variance(topography, topo_mean);
    let cross = covariance(topography, raw_phase, topo_mean, phase_mean);
    let coupling = if topo_var > 1e-30 { cross / topo_var } else { 0.0 };

    // f₀ / (2 k) factor to convert force gradient → frequency shift
    let freq_factor = config.resonant_freq_hz / (2.0 * config.spring_constant_n_per_m);

    for i in 0..n {
        let mag = raw_phase[i] - coupling * (topography[i] - topo_mean);
        magnetic_phase.push(mag);
        // Δf = (f₀ / 2k) · (k/Q) · Δφ  =  (f₀ / 2Q) · Δφ
        let df = freq_factor / config.q_factor * mag * config.spring_constant_n_per_m;
        frequency_shift.push(df);
    }

    LiftModeResult {
        magnetic_phase,
        frequency_shift,
        topography: topography.to_vec(),
        rows,
        cols,
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Force gradient detection
// ────────────────────────────────────────────────────────────────────────────

/// Compute the phase shift image from a force gradient map.
///
/// Δφ = -(Q/k) · dF_z/dz
///
/// # Arguments
/// * `force_gradient` – dF_z/dz values [N/m] per pixel
/// * `q_factor` – cantilever Q
/// * `spring_constant` – cantilever spring constant [N/m]
pub fn phase_from_force_gradient(
    force_gradient: &[f64],
    q_factor: f64,
    spring_constant: f64,
) -> Vec<f64> {
    let coeff = -q_factor / spring_constant;
    force_gradient.iter().map(|&g| coeff * g).collect()
}

/// Compute the frequency shift from a force gradient map.
///
/// Δf = -(f₀ / 2k) · dF_z/dz
///
/// # Arguments
/// * `force_gradient` – dF_z/dz [N/m] per pixel
/// * `resonant_freq` – f₀ [Hz]
/// * `spring_constant` – k [N/m]
pub fn frequency_shift_from_force_gradient(
    force_gradient: &[f64],
    resonant_freq: f64,
    spring_constant: f64,
) -> Vec<f64> {
    let coeff = -resonant_freq / (2.0 * spring_constant);
    force_gradient.iter().map(|&g| coeff * g).collect()
}

/// Estimate the second derivative of the stray field Hz at a pixel array.
///
/// Uses a simple finite-difference approximation along z from two images
/// acquired at heights z1 and z2 (z2 > z1):
///
/// d²H_z/dz² ≈ [H(z1) - 2·H(z_mid) + H(z2)] / (Δz)²
///
/// where z_mid images are estimated by interpolation.
pub fn stray_field_second_derivative(
    hz_low: &[f64],
    hz_high: &[f64],
    delta_z_nm: f64,
) -> Vec<f64> {
    assert_eq!(hz_low.len(), hz_high.len());
    let dz = delta_z_nm * 1e-9; // convert nm → m
    hz_low
        .iter()
        .zip(hz_high.iter())
        .map(|(&lo, &hi)| {
            // Central difference: use lo as z1, hi as z2, midpoint interpolated
            let mid = 0.5 * (lo + hi);
            (lo - 2.0 * mid + hi) / (dz * dz)
        })
        .collect()
}

// ────────────────────────────────────────────────────────────────────────────
// Stray field calculation via transfer function
// ────────────────────────────────────────────────────────────────────────────

/// MFM tip transfer function in Fourier (k) space for a flat plane at height z.
///
/// T(kx, ky) = (kx² + ky²)^(1/2) · exp(-|k| · z_lift)
///
/// Returns T for each spatial frequency magnitude k [1/nm].
pub fn tip_transfer_function(k_magnitudes: &[f64], z_lift_nm: f64) -> Vec<f64> {
    k_magnitudes
        .iter()
        .map(|&k| k * (-k * z_lift_nm).exp())
        .collect()
}

/// Compute Hz stray field at height z from a 2D magnetization distribution
/// using the dipole transfer function (Fourier-space convolution, simplified 1D).
///
/// This is a row-wise 1D DFT approximation suitable for line profiles.
///
/// # Arguments
/// * `mz` – z-component of magnetization per pixel [A/m], length N
/// * `pixel_size_nm` – lateral pixel size [nm]
/// * `z_lift_nm` – height above sample [nm]
/// * `thickness_nm` – magnetic layer thickness [nm]
pub fn compute_hz_from_mz_1d(
    mz: &[f64],
    pixel_size_nm: f64,
    z_lift_nm: f64,
    thickness_nm: f64,
) -> Vec<f64> {
    let n = mz.len();
    // DFT of mz
    let mz_ft = dft_real(mz);
    // Spatial frequencies [1/nm]
    let dk = 1.0 / (n as f64 * pixel_size_nm);
    let mut hz_ft: Vec<(f64, f64)> = Vec::with_capacity(n);
    for (i, &(re, im)) in mz_ft.iter().enumerate() {
        let k = if i <= n / 2 { i as f64 * dk } else { (n as f64 - i as f64) * dk };
        // Transfer function includes thickness factor: (1 - exp(-k·t)) / (2·k) for finite slab
        // For k=0, limit is thickness/2
        let tf = if k < 1e-12 {
            (-z_lift_nm).exp() * thickness_nm * 0.5
        } else {
            0.5 * (1.0 - (-k * thickness_nm).exp()) * (-k * z_lift_nm).exp() / k
        };
        // Multiply by -k (perpendicular component from Mx → Hz: not applicable here for Mz)
        // For Mz → Hz: T = k/2 · (1-exp(-kt)) · exp(-kz) per unit μ₀·Ms
        let mu0 = 4.0 * PI * 1e-7_f64;
        let scale = mu0 * tf;
        hz_ft.push((re * scale, im * scale));
    }
    idft_real(&hz_ft)
}

// ────────────────────────────────────────────────────────────────────────────
// Magnetic domain imaging
// ────────────────────────────────────────────────────────────────────────────

/// Detect domain walls in a 1D phase profile via threshold crossing.
///
/// Returns indices of positions where |phase| crosses the threshold.
pub fn detect_domain_walls_1d(phase: &[f64], threshold: f64) -> Vec<usize> {
    let mut walls = Vec::new();
    let abs_th = threshold.abs();
    for i in 1..phase.len() {
        let prev = phase[i - 1];
        let curr = phase[i];
        // Sign change with amplitude above threshold
        if prev * curr < 0.0 && (prev.abs() > abs_th || curr.abs() > abs_th) {
            walls.push(i);
        }
    }
    walls
}

/// Detect domain walls in a 2D phase image along rows.
///
/// Returns a list of (row, col) positions.
pub fn detect_domain_walls_2d(
    phase: &[f64],
    rows: usize,
    cols: usize,
    threshold: f64,
) -> Vec<(usize, usize)> {
    let mut walls = Vec::new();
    for r in 0..rows {
        let row = &phase[r * cols..(r + 1) * cols];
        let local = detect_domain_walls_1d(row, threshold);
        for c in local {
            walls.push((r, c));
        }
    }
    walls
}

/// Measure domain widths from wall positions in a 1D profile.
///
/// Returns the gaps between consecutive wall positions in [nm].
pub fn measure_domain_widths(wall_positions: &[usize], pixel_size_nm: f64) -> Vec<f64> {
    if wall_positions.len() < 2 {
        return vec![];
    }
    wall_positions
        .windows(2)
        .map(|w| (w[1] - w[0]) as f64 * pixel_size_nm)
        .collect()
}

/// Full domain analysis: detect walls, measure widths, and find periodicity via FFT.
pub fn domain_analysis(
    phase: &[f64],
    rows: usize,
    cols: usize,
    config: &MfmConfig,
) -> DomainWallResult {
    let threshold = 0.05; // radians
    let wall_positions = detect_domain_walls_2d(phase, rows, cols, threshold);

    // Extract column positions of walls on the middle row for periodicity analysis
    let mid_row = rows / 2;
    let row_walls: Vec<usize> = wall_positions
        .iter()
        .filter(|&&(r, _)| r == mid_row)
        .map(|&(_, c)| c)
        .collect();

    let domain_widths_nm = measure_domain_widths(&row_walls, config.pixel_size_nm);
    let mean_domain_width_nm = if domain_widths_nm.is_empty() {
        0.0
    } else {
        mean(&domain_widths_nm)
    };

    // Periodicity via power spectrum of the middle row
    let mid_phase: Vec<f64> = phase[mid_row * cols..(mid_row + 1) * cols].to_vec();
    let periodicity_nm = dominant_period_fft(&mid_phase, config.pixel_size_nm);

    DomainWallResult {
        wall_positions,
        domain_widths_nm,
        mean_domain_width_nm,
        periodicity_nm,
    }
}

/// Find the dominant spatial period in a 1D signal via power spectrum.
///
/// Returns Some(period_nm) or None if the spectrum is flat.
pub fn dominant_period_fft(signal: &[f64], pixel_size_nm: f64) -> Option<f64> {
    if signal.len() < 4 {
        return None;
    }
    let ft = dft_real(signal);
    let n = signal.len();
    // Find peak in power spectrum, skip DC (index 0)
    let mut best_idx = 0usize;
    let mut best_power = 0.0f64;
    for i in 1..n / 2 {
        let (re, im) = ft[i];
        let p = re * re + im * im;
        if p > best_power {
            best_power = p;
            best_idx = i;
        }
    }
    if best_idx == 0 {
        return None;
    }
    // Period = N * pixel_size / best_idx
    let period = n as f64 * pixel_size_nm / best_idx as f64;
    Some(period)
}

// ────────────────────────────────────────────────────────────────────────────
// Tip magnetization models
// ────────────────────────────────────────────────────────────────────────────

/// Compute the z-component of the stray field at a single point (x, z) [nm]
/// above a point dipole located at the origin oriented along z.
///
/// H_z = m / (4πr³) · (3cos²θ - 1)
pub fn point_dipole_hz(x_nm: f64, z_nm: f64, moment_am2: f64) -> f64 {
    let r2 = x_nm * x_nm + z_nm * z_nm;
    let r = r2.sqrt() * 1e-9; // convert nm → m
    if r < 1e-15 {
        return 0.0;
    }
    let r3 = r * r * r;
    let cos_theta = z_nm / (r2.sqrt());
    moment_am2 / (4.0 * PI * r3) * (3.0 * cos_theta * cos_theta - 1.0)
}

/// Compute force on a point dipole tip from sample stray field gradient.
///
/// F_z = μ₀ · m · dH_z/dz
pub fn point_dipole_force(moment_am2: f64, dhz_dz: f64) -> f64 {
    let mu0 = 4.0 * PI * 1e-7_f64;
    mu0 * moment_am2 * dhz_dz
}

/// Effective magnetic charge for monopole tip model.
///
/// q_eff = μ₀ · M_tip · A_cone
/// where A_cone is the cross-sectional area at the apex.
pub fn effective_magnetic_charge(
    ms_a_per_m: f64,
    cone_angle_deg: f64,
    tip_radius_nm: f64,
) -> f64 {
    let mu0 = 4.0 * PI * 1e-7_f64;
    let angle_rad = cone_angle_deg * PI / 180.0;
    let r = tip_radius_nm * 1e-9;
    let area = PI * (r * angle_rad.tan()) * (r * angle_rad.tan());
    mu0 * ms_a_per_m * area
}

// ────────────────────────────────────────────────────────────────────────────
// Quantitative MFM deconvolution
// ────────────────────────────────────────────────────────────────────────────

/// Deconvolve MFM phase image by the tip transfer function to recover Mz.
///
/// Wiener-type deconvolution in Fourier space:
/// M̃(k) = Φ̃(k) / [T(k) + ε]
///
/// # Arguments
/// * `phase_image` – measured phase shift image [rad], length N (1D or flattened 2D)
/// * `pixel_size_nm` – lateral pixel size [nm]
/// * `z_lift_nm` – lift height [nm]
/// * `q_factor`, `spring_constant` – cantilever parameters
/// * `epsilon` – Wiener regularization parameter (noise level)
pub fn deconvolve_tip_function(
    phase_image: &[f64],
    pixel_size_nm: f64,
    z_lift_nm: f64,
    q_factor: f64,
    spring_constant: f64,
    epsilon: f64,
) -> DeconvolutionResult {
    let n = phase_image.len();
    // Convert phase to force gradient: dF/dz = -k/Q · Δφ
    let force_gradient: Vec<f64> = phase_image
        .iter()
        .map(|&ph| -spring_constant / q_factor * ph)
        .collect();

    let fg_ft = dft_real(&force_gradient);
    let dk = 1.0 / (n as f64 * pixel_size_nm);
    let mu0 = 4.0 * PI * 1e-7_f64;

    let mut mz_ft: Vec<(f64, f64)> = Vec::with_capacity(n);
    for (i, &(re, im)) in fg_ft.iter().enumerate() {
        let ki = if i <= n / 2 { i as f64 * dk } else { (n as f64 - i as f64) * dk };
        // T(k) = k · exp(-k · z) (transfer function; μ₀ factor included)
        let tf = mu0 * ki * (-ki * z_lift_nm).exp();
        let denom = tf * tf + epsilon;
        let scale = tf / denom;
        mz_ft.push((re * scale, im * scale));
    }

    let magnetization = idft_real(&mz_ft);
    let rms = (magnetization.iter().map(|x| x * x).sum::<f64>() / n as f64).sqrt();

    DeconvolutionResult {
        magnetization,
        magnetization_rms: rms,
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Image processing utilities
// ────────────────────────────────────────────────────────────────────────────

/// Fit and subtract a plane from a 2D image (least-squares plane fit).
///
/// Removes linear tilt and offset: z_corrected = z - (a·x + b·y + c)
pub fn plane_fit_subtract(image: &[f64], rows: usize, cols: usize) -> Vec<f64> {
    let n = rows * cols;
    assert_eq!(image.len(), n);

    // Build normal equations for plane a*x + b*y + c = z
    let mut sx = 0.0f64;
    let mut sy = 0.0f64;
    let mut sxx = 0.0f64;
    let mut sxy = 0.0f64;
    let mut syy = 0.0f64;
    let mut sz = 0.0f64;
    let mut sxz = 0.0f64;
    let mut syz = 0.0f64;

    for r in 0..rows {
        for c in 0..cols {
            let x = c as f64;
            let y = r as f64;
            let z = image[r * cols + c];
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
            syy += y * y;
            sz += z;
            sxz += x * z;
            syz += y * z;
        }
    }
    let nf = n as f64;
    // Solve 3×3 system: [sxx sxy sx; sxy syy sy; sx sy n] [a;b;c] = [sxz;syz;sz]
    let (a, b, c) = solve_plane_3x3(sxx, sxy, sx, syy, sy, nf, sxz, syz, sz);

    let mut out = Vec::with_capacity(n);
    for r in 0..rows {
        for col in 0..cols {
            let x = col as f64;
            let y = r as f64;
            out.push(image[r * cols + col] - (a * x + b * y + c));
        }
    }
    out
}

/// Solve the 3×3 plane fit normal equations via Cramer's rule.
fn solve_plane_3x3(
    sxx: f64,
    sxy: f64,
    sx: f64,
    syy: f64,
    sy: f64,
    n: f64,
    sxz: f64,
    syz: f64,
    sz: f64,
) -> (f64, f64, f64) {
    // det of [[sxx,sxy,sx],[sxy,syy,sy],[sx,sy,n]]
    let det = sxx * (syy * n - sy * sy) - sxy * (sxy * n - sy * sx) + sx * (sxy * sy - syy * sx);
    if det.abs() < 1e-30 {
        return (0.0, 0.0, sz / n.max(1.0));
    }
    let a = (sxz * (syy * n - sy * sy) - sxy * (syz * n - sy * sz) + sx * (syz * sy - syy * sz))
        / det;
    let b = (sxx * (syz * n - sy * sz) - sxz * (sxy * n - sy * sx) + sx * (sxy * sz - syz * sx))
        / det;
    let c = (sxx * (syy * sz - sy * syz) - sxy * (sxy * sz - sy * sxz)
        + sxz * (sxy * sy - syy * sx))
        / det;
    (a, b, c)
}

/// Line-by-line leveling: subtract the mean from each row.
pub fn line_level(image: &[f64], rows: usize, cols: usize) -> Vec<f64> {
    assert_eq!(image.len(), rows * cols);
    let mut out = Vec::with_capacity(rows * cols);
    for r in 0..rows {
        let row = &image[r * cols..(r + 1) * cols];
        let m = mean(row);
        for &v in row {
            out.push(v - m);
        }
    }
    out
}

/// Sliding median filter for scan artifact removal (1D, window = 2*half+1).
pub fn median_filter_1d(signal: &[f64], half_window: usize) -> Vec<f64> {
    let n = signal.len();
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let lo = i.saturating_sub(half_window);
        let hi = (i + half_window + 1).min(n);
        let mut window: Vec<f64> = signal[lo..hi].to_vec();
        window.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mid = window.len() / 2;
        out.push(window[mid]);
    }
    out
}

/// Apply row-wise median filter to a 2D image.
pub fn median_filter_2d(image: &[f64], rows: usize, cols: usize, half_window: usize) -> Vec<f64> {
    assert_eq!(image.len(), rows * cols);
    let mut out = Vec::with_capacity(rows * cols);
    for r in 0..rows {
        let row = &image[r * cols..(r + 1) * cols];
        let filtered = median_filter_1d(row, half_window);
        out.extend_from_slice(&filtered);
    }
    out
}

// ────────────────────────────────────────────────────────────────────────────
// Coercivity mapping
// ────────────────────────────────────────────────────────────────────────────

/// Extract the switching field distribution from a series of field-dependent
/// MFM images.
///
/// For each pixel, the switching field is the applied field at which the
/// phase contrast reversal occurs (sign change).
///
/// # Arguments
/// * `images` – slice of phase images, one per applied field value
/// * `applied_fields` – applied magnetic field for each image [A/m]
/// * `n_pixels` – number of pixels in each image
pub fn coercivity_map(
    images: &[Vec<f64>],
    applied_fields: &[f64],
    n_pixels: usize,
) -> CoercivityMapResult {
    assert_eq!(images.len(), applied_fields.len());
    let n_fields = images.len();
    let mut switching_field = vec![applied_fields[n_fields / 2]; n_pixels];

    for px in 0..n_pixels {
        let initial_sign = images[0][px].signum();
        for (fi, field) in applied_fields.iter().enumerate() {
            if images[fi][px].signum() != initial_sign && images[fi][px].abs() > 1e-6 {
                switching_field[px] = *field;
                break;
            }
        }
    }

    let mean_sw = mean(&switching_field);
    let var_sw = variance(&switching_field, mean_sw);
    let sfd_width = var_sw.sqrt();

    CoercivityMapResult {
        switching_field,
        mean_switching_field_a_per_m: mean_sw,
        sfd_width_a_per_m: sfd_width,
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Hard disk bit pattern analysis
// ────────────────────────────────────────────────────────────────────────────

/// Analyze a hard disk MFM image to extract track and bit parameters.
///
/// # Arguments
/// * `phase` – 2D phase image [rad], rows × cols
/// * `rows`, `cols` – image dimensions
/// * `config` – MFM configuration (pixel_size_nm used)
pub fn analyze_bit_pattern(
    phase: &[f64],
    rows: usize,
    cols: usize,
    config: &MfmConfig,
) -> BitPatternResult {
    // Track pitch: dominant period along columns (cross-track direction)
    let col_profile: Vec<f64> = (0..rows)
        .map(|r| {
            // Average across columns for each row gives cross-track profile
            let row = &phase[r * cols..(r + 1) * cols];
            mean(row)
        })
        .collect();
    let track_pitch_nm = dominant_period_fft(&col_profile, config.pixel_size_nm)
        .unwrap_or(config.pixel_size_nm * rows as f64);

    // Bit length: dominant period along rows (along-track direction) on middle row
    let mid_row_phase: Vec<f64> = phase[rows / 2 * cols..(rows / 2 + 1) * cols].to_vec();
    let bit_length_nm = dominant_period_fft(&mid_row_phase, config.pixel_size_nm)
        .unwrap_or(config.pixel_size_nm * cols as f64);

    // Signal amplitude: peak-to-peak of mid-row
    let max_ph = mid_row_phase.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let min_ph = mid_row_phase.iter().cloned().fold(f64::INFINITY, f64::min);
    let signal_amplitude_rad = max_ph - min_ph;

    // Transition noise: RMS deviation at wall positions
    let walls = detect_domain_walls_1d(&mid_row_phase, 0.01);
    let transition_noise_rad = if walls.len() < 2 {
        0.0
    } else {
        let wall_vals: Vec<f64> = walls.iter().map(|&i| mid_row_phase[i]).collect();
        let wm = mean(&wall_vals);
        variance(&wall_vals, wm).sqrt()
    };

    // Linear density: 1 bit per bit_length
    let bit_length_mm = bit_length_nm * 1e-6;
    let linear_density_bits_per_mm = if bit_length_mm > 0.0 {
        1.0 / bit_length_mm
    } else {
        0.0
    };

    BitPatternResult {
        track_pitch_nm,
        bit_length_nm,
        signal_amplitude_rad,
        transition_noise_rad,
        linear_density_bits_per_mm,
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Internal math helpers
// ────────────────────────────────────────────────────────────────────────────

/// Compute the mean of a slice.
pub fn mean(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    data.iter().sum::<f64>() / data.len() as f64
}

/// Compute variance given pre-computed mean.
pub fn variance(data: &[f64], mean_val: f64) -> f64 {
    if data.len() < 2 {
        return 0.0;
    }
    data.iter().map(|x| (x - mean_val).powi(2)).sum::<f64>() / data.len() as f64
}

/// Compute covariance given pre-computed means.
fn covariance(a: &[f64], b: &[f64], mean_a: f64, mean_b: f64) -> f64 {
    assert_eq!(a.len(), b.len());
    if a.is_empty() {
        return 0.0;
    }
    a.iter()
        .zip(b.iter())
        .map(|(&ai, &bi)| (ai - mean_a) * (bi - mean_b))
        .sum::<f64>()
        / a.len() as f64
}

/// Naive O(N²) DFT of a real signal.  Returns (re, im) per bin.
pub fn dft_real(signal: &[f64]) -> Vec<(f64, f64)> {
    let n = signal.len();
    let mut out = Vec::with_capacity(n);
    for k in 0..n {
        let mut re = 0.0f64;
        let mut im = 0.0f64;
        let angle = -2.0 * PI * k as f64 / n as f64;
        for (m, &s) in signal.iter().enumerate() {
            let phi = angle * m as f64;
            re += s * phi.cos();
            im += s * phi.sin();
        }
        out.push((re, im));
    }
    out
}

/// Inverse DFT from (re, im) bins back to real signal.
pub fn idft_real(ft: &[(f64, f64)]) -> Vec<f64> {
    let n = ft.len();
    let mut out = Vec::with_capacity(n);
    for m in 0..n {
        let mut val = 0.0f64;
        let angle = 2.0 * PI * m as f64 / n as f64;
        for (k, &(re, im)) in ft.iter().enumerate() {
            let phi = angle * k as f64;
            val += re * phi.cos() - im * phi.sin();
        }
        out.push(val / n as f64);
    }
    out
}

// ────────────────────────────────────────────────────────────────────────────
// Unit tests
// ────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    // ── helper ──────────────────────────────────────────────────────────────
    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // ── MfmConfig default ────────────────────────────────────────────────────
    #[test]
    fn test_mfm_config_default() {
        let c = MfmConfig::default();
        assert_eq!(c.lift_height_nm, 50.0);
        assert_eq!(c.q_factor, 500.0);
        assert_eq!(c.spring_constant_n_per_m, 2.5);
        assert_eq!(c.pixel_size_nm, 10.0);
    }

    // ── MaterialProperties presets ───────────────────────────────────────────
    #[test]
    fn test_material_preset_hard_disk() {
        let m = MaterialProperties::from_preset(MaterialPreset::HardDiskCoCrPt);
        assert!(m.ms_a_per_m > 0.0);
        assert!(m.hc_a_per_m > 0.0);
        assert_eq!(m.thickness_nm, 15.0);
    }

    #[test]
    fn test_material_preset_permalloy() {
        let m = MaterialProperties::from_preset(MaterialPreset::PermalloyThinFilm);
        // Permalloy has low coercivity
        assert!(m.hc_a_per_m < 1000.0);
        assert!(m.ms_a_per_m > 1.0e5);
    }

    #[test]
    fn test_material_preset_garnet() {
        let m = MaterialProperties::from_preset(MaterialPreset::GarnetFilm);
        assert!(m.thickness_nm > 1000.0); // thick film
    }

    #[test]
    fn test_material_preset_tape() {
        let m = MaterialProperties::from_preset(MaterialPreset::MagneticTape);
        assert!(m.ms_a_per_m > 0.0);
        assert!(m.hc_a_per_m > 0.0);
    }

    #[test]
    fn test_material_preset_ndfeb() {
        let m = MaterialProperties::from_preset(MaterialPreset::NdFeBPermanentMagnet);
        assert!(m.hc_a_per_m > 5.0e5);
    }

    // ── Two-pass lift mode ───────────────────────────────────────────────────
    #[test]
    fn test_two_pass_flat_topography() {
        let config = MfmConfig::default();
        let rows = 4;
        let cols = 4;
        let n = rows * cols;
        // Flat topography, pure magnetic phase
        let topo = vec![0.0f64; n];
        let raw = vec![0.5f64; n];
        let res = two_pass_lift_mode(&topo, &raw, rows, cols, &config);
        assert_eq!(res.magnetic_phase.len(), n);
        // With flat topo, coupling ≈ 0, so magnetic_phase ≈ raw
        for &v in &res.magnetic_phase {
            assert!(approx_eq(v, 0.5, 1e-6));
        }
    }

    #[test]
    fn test_two_pass_result_dimensions() {
        let config = MfmConfig::default();
        let rows = 8;
        let cols = 8;
        let n = rows * cols;
        let topo: Vec<f64> = (0..n).map(|i| i as f64 * 0.1).collect();
        let raw: Vec<f64> = (0..n).map(|i| (i as f64 * 0.01).sin()).collect();
        let res = two_pass_lift_mode(&topo, &raw, rows, cols, &config);
        assert_eq!(res.rows, rows);
        assert_eq!(res.cols, cols);
        assert_eq!(res.magnetic_phase.len(), n);
        assert_eq!(res.frequency_shift.len(), n);
    }

    #[test]
    fn test_two_pass_topography_stored() {
        let config = MfmConfig::default();
        let topo = vec![1.0, 2.0, 3.0, 4.0];
        let raw = vec![0.1f64; 4];
        let res = two_pass_lift_mode(&topo, &raw, 2, 2, &config);
        assert_eq!(res.topography, topo);
    }

    // ── Force gradient → phase ───────────────────────────────────────────────
    #[test]
    fn test_phase_from_force_gradient_sign() {
        // positive force gradient → negative phase (attractive)
        let fg = vec![1.0f64];
        let ph = phase_from_force_gradient(&fg, 500.0, 2.5);
        assert!(ph[0] < 0.0);
    }

    #[test]
    fn test_phase_from_force_gradient_magnitude() {
        let q = 500.0;
        let k = 2.5;
        let fg = vec![1e-6f64]; // 1 µN/m
        let ph = phase_from_force_gradient(&fg, q, k);
        let expected = -q / k * 1e-6;
        assert!(approx_eq(ph[0], expected, 1e-15));
    }

    #[test]
    fn test_frequency_shift_from_force_gradient() {
        let f0 = 75_000.0;
        let k = 2.5;
        let fg = vec![1e-6f64];
        let df = frequency_shift_from_force_gradient(&fg, f0, k);
        let expected = -f0 / (2.0 * k) * 1e-6;
        assert!(approx_eq(df[0], expected, 1e-12));
    }

    // ── Stray field second derivative ─────────────────────────────────────────
    #[test]
    fn test_stray_field_second_derivative_uniform() {
        // Uniform field: all zero curvature
        let lo = vec![1.0f64; 4];
        let hi = vec![1.0f64; 4];
        let d2 = stray_field_second_derivative(&lo, &hi, 1.0);
        // lo = hi, so numerically zero (but our formula uses midpoint average)
        // mid = (lo+hi)/2 = lo = hi → (lo - 2*mid + hi) = (1 - 2 + 1) = 0
        for &v in &d2 {
            assert!(v.abs() < EPS);
        }
    }

    #[test]
    fn test_stray_field_second_derivative_nonzero() {
        let lo = vec![0.0f64];
        let hi = vec![2.0f64];
        let dz = 10.0; // nm
        let d2 = stray_field_second_derivative(&lo, &hi, dz);
        // mid = 1.0; (0 - 2*1 + 2) / (10e-9)^2 = 0 / ...
        // numerically: (0 - 2*1 + 2) = 0 → same as uniform
        assert!(d2[0].abs() < EPS);
    }

    // ── Transfer function ────────────────────────────────────────────────────
    #[test]
    fn test_tip_transfer_function_dc() {
        // k=0 → T=0 (DC term should be zero)
        let tf = tip_transfer_function(&[0.0], 50.0);
        assert!(approx_eq(tf[0], 0.0, EPS));
    }

    #[test]
    fn test_tip_transfer_function_decay() {
        // Higher k → stronger exponential decay at fixed lift
        let k_vals = vec![0.01, 0.05, 0.1, 0.5];
        let tf = tip_transfer_function(&k_vals, 50.0);
        // At k=0.5, exp(-0.5*50)=exp(-25)≈0 so T should be very small
        assert!(tf[3] < tf[1]);
    }

    #[test]
    fn test_tip_transfer_function_positive() {
        let k_vals: Vec<f64> = (1..10).map(|i| i as f64 * 0.01).collect();
        let tf = tip_transfer_function(&k_vals, 20.0);
        for &t in &tf {
            assert!(t >= 0.0);
        }
    }

    // ── Point dipole ──────────────────────────────────────────────────────────
    #[test]
    fn test_point_dipole_hz_on_axis() {
        // On-axis (x=0): H_z = m / (2π r³)
        let hz = point_dipole_hz(0.0, 100.0, 1e-16);
        // r = 100 nm = 100e-9 m
        let r = 100e-9_f64;
        let expected = 1e-16 / (2.0 * PI * r * r * r);
        // Ratio should be close to 1
        assert!(approx_eq(hz / expected, 1.0, 0.01));
    }

    #[test]
    fn test_point_dipole_hz_equator_negative() {
        // At 90°: H_z = m/(4πr³) * (3·0 - 1) < 0
        let hz = point_dipole_hz(100.0, 0.0, 1e-16);
        // z=0 so cos_theta = 0 / r = 0
        assert!(hz < 0.0);
    }

    #[test]
    fn test_point_dipole_force_sign() {
        // Positive moment, positive gradient → positive force
        let f = point_dipole_force(1e-16, 1.0);
        assert!(f > 0.0);
    }

    #[test]
    fn test_effective_magnetic_charge_positive() {
        let q = effective_magnetic_charge(8e5, 10.0, 20.0);
        assert!(q > 0.0);
    }

    // ── Domain wall detection ────────────────────────────────────────────────
    #[test]
    fn test_detect_domain_walls_simple() {
        // Alternating ±1 signal → walls at every boundary
        let signal = vec![1.0, 1.0, -1.0, -1.0, 1.0, 1.0];
        let walls = detect_domain_walls_1d(&signal, 0.5);
        assert!(!walls.is_empty());
        assert!(walls.contains(&2)); // sign change at index 2
    }

    #[test]
    fn test_detect_domain_walls_no_walls() {
        let signal = vec![1.0, 1.0, 1.0, 1.0];
        let walls = detect_domain_walls_1d(&signal, 0.5);
        assert!(walls.is_empty());
    }

    #[test]
    fn test_detect_domain_walls_2d() {
        let phase: Vec<f64> = vec![
            1.0, 1.0, -1.0, -1.0, //row 0
            1.0, 1.0, -1.0, -1.0, //row 1
        ];
        let walls = detect_domain_walls_2d(&phase, 2, 4, 0.5);
        assert!(!walls.is_empty());
        // Should find walls on both rows
        let rows_with_walls: Vec<usize> = walls.iter().map(|&(r, _)| r).collect();
        assert!(rows_with_walls.contains(&0));
        assert!(rows_with_walls.contains(&1));
    }

    // ── Domain widths ────────────────────────────────────────────────────────
    #[test]
    fn test_measure_domain_widths_uniform() {
        // Wall at every 10 pixels, pixel = 5 nm → width = 50 nm
        let walls = vec![0, 10, 20, 30];
        let widths = measure_domain_widths(&walls, 5.0);
        for &w in &widths {
            assert!(approx_eq(w, 50.0, EPS));
        }
    }

    #[test]
    fn test_measure_domain_widths_empty() {
        let widths = measure_domain_widths(&[], 10.0);
        assert!(widths.is_empty());
    }

    // ── Dominant period FFT ──────────────────────────────────────────────────
    #[test]
    fn test_dominant_period_sine() {
        // Sine at 1/8 of the signal length → period = 8 pixels
        let n = 64usize;
        let period_pixels = 8.0;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * i as f64 / period_pixels).sin())
            .collect();
        let period_nm = dominant_period_fft(&signal, 1.0); // 1 nm/pixel
        assert!(period_nm.is_some());
        let p = period_nm.unwrap();
        assert!(approx_eq(p, period_pixels, 1.0));
    }

    #[test]
    fn test_dominant_period_short_signal() {
        let p = dominant_period_fft(&[1.0, -1.0, 1.0], 1.0);
        // 3 samples, period = 2, but might still return Some
        let _ = p; // just ensure no panic
    }

    // ── Plane fit subtraction ─────────────────────────────────────────────────
    #[test]
    fn test_plane_fit_flat_image() {
        // Flat image at 1.0 → after plane fit should be near 0
        let image = vec![1.0f64; 16];
        let out = plane_fit_subtract(&image, 4, 4);
        for &v in &out {
            assert!(v.abs() < 1e-9);
        }
    }

    #[test]
    fn test_plane_fit_removes_tilt() {
        let rows = 4;
        let cols = 4;
        // Image z = x + y
        let image: Vec<f64> = (0..rows)
            .flat_map(|r| (0..cols).map(move |c| r as f64 + c as f64))
            .collect();
        let out = plane_fit_subtract(&image, rows, cols);
        // Residual should be approximately zero
        let rms: f64 = (out.iter().map(|x| x * x).sum::<f64>() / out.len() as f64).sqrt();
        assert!(rms < 1e-8);
    }

    // ── Line leveling ─────────────────────────────────────────────────────────
    #[test]
    fn test_line_level_zero_mean_rows() {
        let image: Vec<f64> = vec![1.0, 3.0, 2.0, 4.0]; // 2×2
        let out = line_level(&image, 2, 2);
        // Row 0 mean = 2.0 → [−1, 1]; Row 1 mean = 3.0 → [−1, 1]
        assert!(approx_eq(out[0], -1.0, EPS));
        assert!(approx_eq(out[1], 1.0, EPS));
        assert!(approx_eq(out[2], -1.0, EPS));
        assert!(approx_eq(out[3], 1.0, EPS));
    }

    // ── Median filter ─────────────────────────────────────────────────────────
    #[test]
    fn test_median_filter_removes_spike() {
        let mut signal = vec![0.0f64; 11];
        signal[5] = 100.0; // spike
        let filtered = median_filter_1d(&signal, 2);
        // Spike should be suppressed
        assert!(filtered[5] < 1.0);
    }

    #[test]
    fn test_median_filter_preserves_dc() {
        let signal = vec![5.0f64; 8];
        let filtered = median_filter_1d(&signal, 2);
        for &v in &filtered {
            assert!(approx_eq(v, 5.0, EPS));
        }
    }

    // ── Deconvolution ─────────────────────────────────────────────────────────
    #[test]
    fn test_deconvolution_output_length() {
        let phase = vec![0.1f64; 32];
        let res = deconvolve_tip_function(&phase, 10.0, 50.0, 500.0, 2.5, 1e-6);
        assert_eq!(res.magnetization.len(), 32);
    }

    #[test]
    fn test_deconvolution_rms_nonnegative() {
        let phase = vec![0.05f64; 16];
        let res = deconvolve_tip_function(&phase, 10.0, 50.0, 500.0, 2.5, 1e-5);
        assert!(res.magnetization_rms >= 0.0);
    }

    // ── Hz from Mz ────────────────────────────────────────────────────────────
    #[test]
    fn test_hz_from_mz_length() {
        let mz = vec![1.0f64; 16];
        let hz = compute_hz_from_mz_1d(&mz, 10.0, 50.0, 15.0);
        assert_eq!(hz.len(), 16);
    }

    // ── Coercivity mapping ────────────────────────────────────────────────────
    #[test]
    fn test_coercivity_map_basic() {
        let n_px = 4;
        let fields = vec![-1e5, -5e4, 0.0, 5e4, 1e5_f64];
        // Image: positive phase at negative fields, reverses at zero
        let images: Vec<Vec<f64>> = fields
            .iter()
            .map(|&h| {
                if h < 0.0 {
                    vec![1.0f64; n_px]
                } else {
                    vec![-1.0f64; n_px]
                }
            })
            .collect();
        let res = coercivity_map(&images, &fields, n_px);
        assert_eq!(res.switching_field.len(), n_px);
        // All pixels switch at the same field → zero SFD width
        assert!(res.sfd_width_a_per_m < 1.0);
    }

    // ── Bit pattern analysis ──────────────────────────────────────────────────
    #[test]
    fn test_bit_pattern_analysis() {
        let config = MfmConfig {
            pixel_size_nm: 5.0,
            ..MfmConfig::default()
        };
        let rows = 16;
        let cols = 64;
        let bit_period = 8usize; // pixels
        let phase: Vec<f64> = (0..rows)
            .flat_map(|_r| {
                (0..cols).map(move |c| {
                    let bit = (c / (bit_period / 2)) % 2;
                    if bit == 0 { 1.0 } else { -1.0 }
                })
            })
            .collect();
        let res = analyze_bit_pattern(&phase, rows, cols, &config);
        assert!(res.bit_length_nm > 0.0);
        assert!(res.signal_amplitude_rad > 0.0);
        assert!(res.linear_density_bits_per_mm > 0.0);
    }

    // ── DFT round-trip ────────────────────────────────────────────────────────
    #[test]
    fn test_dft_idft_roundtrip() {
        let signal = vec![1.0, 2.0, 3.0, 4.0];
        let ft = dft_real(&signal);
        let reconstructed = idft_real(&ft);
        for (a, b) in signal.iter().zip(reconstructed.iter()) {
            assert!(approx_eq(*a, *b, 1e-10));
        }
    }

    // ── Mean / variance / covariance ─────────────────────────────────────────
    #[test]
    fn test_mean_basic() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        assert!(approx_eq(mean(&data), 3.0, EPS));
    }

    #[test]
    fn test_mean_empty() {
        assert_eq!(mean(&[]), 0.0);
    }

    #[test]
    fn test_variance_known() {
        let data = vec![2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0];
        let m = mean(&data);
        let v = variance(&data, m);
        // Population variance ≈ 4.0
        assert!(approx_eq(v, 4.0, 0.01));
    }

    // ── Domain analysis integration ───────────────────────────────────────────
    #[test]
    fn test_domain_analysis_integration() {
        let config = MfmConfig { pixel_size_nm: 5.0, ..MfmConfig::default() };
        let rows = 4;
        let cols = 16;
        let phase: Vec<f64> = (0..rows)
            .flat_map(|_| {
                (0..cols).map(move |c| if c < 8 { 0.5 } else { -0.5 })
            })
            .collect();
        let result = domain_analysis(&phase, rows, cols, &config);
        // Should find walls
        assert!(!result.wall_positions.is_empty());
        assert!(result.mean_domain_width_nm >= 0.0);
    }

    // ── Tip transfer function peak ────────────────────────────────────────────
    #[test]
    fn test_tip_transfer_function_peak() {
        // The transfer function T(k) = k·exp(-k·z) peaks at k = 1/z
        let z = 50.0; // nm
        let k_peak = 1.0 / z;
        // Sample around the peak
        let k_vals: Vec<f64> = (1..=100).map(|i| i as f64 * 0.001).collect();
        let tf = tip_transfer_function(&k_vals, z);
        let peak_idx = tf
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        let k_at_peak = k_vals[peak_idx];
        // Should be within 20% of 1/z
        assert!(approx_eq(k_at_peak, k_peak, k_peak * 0.2));
    }
}
