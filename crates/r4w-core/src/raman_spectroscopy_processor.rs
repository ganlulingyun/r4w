//! # Raman Spectroscopy Signal Processor
//!
//! This module implements Raman spectroscopy signal processing for molecular
//! identification and quantitative analysis. Raman spectroscopy measures
//! inelastic light scattering to probe molecular vibrational modes, providing
//! a unique spectral fingerprint for each material.
//!
//! ## Background
//!
//! When monochromatic light (typically a laser) interacts with a molecule,
//! most photons scatter elastically (Rayleigh scattering). A small fraction
//! (~1 in 10^7) scatter inelastically, exchanging energy with molecular
//! vibrations. The energy shift corresponds to vibrational frequencies:
//!
//! - **Stokes scattering**: photon loses energy (red-shifted), positive Raman shift
//! - **Anti-Stokes scattering**: photon gains energy (blue-shifted), negative shift
//!
//! The Raman shift in wavenumbers (cm^-1) is:
//!
//! ```text
//! shift_cm1 = (1/lambda_excitation - 1/lambda_scattered) * 1e7
//! ```
//!
//! where wavelengths are in nanometers.
//!
//! ## Features
//!
//! - [`RamanSpectrum`] -- wavenumber/intensity spectrum with metadata
//! - [`RamanProcessor`] -- configurable preprocessing pipeline
//! - Cosmic ray removal via median filter comparison
//! - Baseline correction: polynomial, asymmetric least squares, rubber band
//! - Savitzky-Golay smoothing (peak-shape preserving)
//! - Normalization: area, peak, min-max, SNV
//! - Peak analysis: detection, Lorentzian/Gaussian/Voigt fitting, FWHM, area
//! - Fluorescence background estimation and removal
//! - Molecular identification via reference library matching (HQI)
//! - Quantitative analysis: calibration curves, LOD estimation
//! - Depolarization ratio for symmetry classification
//! - Temperature extraction from anti-Stokes/Stokes ratio
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::raman_spectroscopy_processor::*;
//!
//! // Create a synthetic Raman spectrum with a Lorentzian peak at 1001 cm-1
//! let wavenumbers: Vec<f64> = (0..500).map(|i| 800.0 + i as f64 * 2.0).collect();
//! let intensities: Vec<f64> = wavenumbers.iter().map(|nu| {
//!     lorentzian_peak(*nu, 100.0, 1001.0, 10.0)
//! }).collect();
//!
//! let spectrum = RamanSpectrum::new(wavenumbers, intensities, 532.0);
//! assert_eq!(spectrum.len(), 500);
//!
//! // Find peaks
//! let peaks = find_peaks(&spectrum.intensity, 5.0, 3);
//! assert!(!peaks.is_empty());
//! ```

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Speed of light in cm/s.
const C_CM_S: f64 = 2.997_924_58e10;

/// Planck constant in J*s.
const H_PLANCK: f64 = 6.626_070_15e-34;

/// Boltzmann constant in J/K.
const K_BOLTZMANN: f64 = 1.380_649e-23;

/// Speed of light in m/s.
const C_M_S: f64 = 2.997_924_58e8;

// ---------------------------------------------------------------------------
// RamanSpectrum
// ---------------------------------------------------------------------------

/// A Raman spectrum: paired wavenumber (cm^-1) and intensity arrays.
#[derive(Debug, Clone)]
pub struct RamanSpectrum {
    /// Raman shift values in cm^-1.
    pub wavenumber_cm1: Vec<f64>,
    /// Intensity values (arbitrary units or counts).
    pub intensity: Vec<f64>,
    /// Excitation laser wavelength in nm.
    pub excitation_wavelength_nm: f64,
}

impl RamanSpectrum {
    /// Create a new spectrum from wavenumber and intensity arrays.
    ///
    /// # Panics
    /// Panics if arrays have different lengths.
    pub fn new(wavenumber_cm1: Vec<f64>, intensity: Vec<f64>, excitation_wavelength_nm: f64) -> Self {
        assert_eq!(
            wavenumber_cm1.len(),
            intensity.len(),
            "wavenumber and intensity arrays must have equal length"
        );
        Self {
            wavenumber_cm1,
            intensity,
            excitation_wavelength_nm,
        }
    }

    /// Number of spectral points.
    pub fn len(&self) -> usize {
        self.wavenumber_cm1.len()
    }

    /// Whether the spectrum is empty.
    pub fn is_empty(&self) -> bool {
        self.wavenumber_cm1.is_empty()
    }

    /// Maximum intensity value.
    pub fn max_intensity(&self) -> f64 {
        self.intensity.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
    }

    /// Minimum intensity value.
    pub fn min_intensity(&self) -> f64 {
        self.intensity.iter().cloned().fold(f64::INFINITY, f64::min)
    }

    /// Total integrated area (trapezoidal rule).
    pub fn integrated_area(&self) -> f64 {
        trapezoidal_integrate(&self.wavenumber_cm1, &self.intensity)
    }
}

// ---------------------------------------------------------------------------
// Wavelength / wavenumber conversion
// ---------------------------------------------------------------------------

/// Convert excitation and scattered wavelengths (nm) to Raman shift (cm^-1).
///
/// `shift = (1/lambda_exc - 1/lambda_scat) * 1e7`
///
/// Positive shift = Stokes (scattered wavelength longer than excitation).
pub fn wavelength_to_raman_shift(excitation_nm: f64, scattered_nm: f64) -> f64 {
    (1.0 / excitation_nm - 1.0 / scattered_nm) * 1e7
}

/// Convert Raman shift (cm^-1) to scattered wavelength (nm) given excitation.
pub fn raman_shift_to_wavelength(excitation_nm: f64, shift_cm1: f64) -> f64 {
    1.0 / (1.0 / excitation_nm - shift_cm1 / 1e7)
}

/// Check if a shift is Stokes (positive) or anti-Stokes (negative).
pub fn is_stokes(shift_cm1: f64) -> bool {
    shift_cm1 > 0.0
}

// ---------------------------------------------------------------------------
// Line shape functions
// ---------------------------------------------------------------------------

/// Lorentzian peak profile: I(nu) = A * (gamma/2)^2 / ((nu - nu0)^2 + (gamma/2)^2)
///
/// - `nu`: wavenumber evaluation point (cm^-1)
/// - `amplitude`: peak height A
/// - `center`: center position nu0 (cm^-1)
/// - `gamma`: full width at half maximum (cm^-1)
pub fn lorentzian_peak(nu: f64, amplitude: f64, center: f64, gamma: f64) -> f64 {
    let half_gamma = gamma / 2.0;
    let dnu = nu - center;
    amplitude * half_gamma * half_gamma / (dnu * dnu + half_gamma * half_gamma)
}

/// Gaussian peak profile: I(nu) = A * exp(-4*ln(2)*(nu-nu0)^2 / gamma^2)
///
/// - `gamma` is FWHM
pub fn gaussian_peak(nu: f64, amplitude: f64, center: f64, gamma: f64) -> f64 {
    let dnu = nu - center;
    let sigma_factor = 4.0 * 2.0_f64.ln() / (gamma * gamma);
    amplitude * (-sigma_factor * dnu * dnu).exp()
}

/// Pseudo-Voigt profile: linear combination of Lorentzian and Gaussian.
///
/// - `eta` in [0, 1]: 0 = pure Gaussian, 1 = pure Lorentzian
pub fn voigt_peak(nu: f64, amplitude: f64, center: f64, gamma: f64, eta: f64) -> f64 {
    let eta = eta.clamp(0.0, 1.0);
    eta * lorentzian_peak(nu, amplitude, center, gamma)
        + (1.0 - eta) * gaussian_peak(nu, amplitude, center, gamma)
}

// ---------------------------------------------------------------------------
// RamanProcessor configuration
// ---------------------------------------------------------------------------

/// Configuration for Raman spectral processing.
#[derive(Debug, Clone)]
pub struct RamanProcessor {
    /// Spectral resolution in cm^-1.
    pub spectral_resolution_cm1: f64,
    /// Excitation wavelength in nm.
    pub excitation_wavelength_nm: f64,
    /// Threshold (in multiples of local MAD) for cosmic ray detection.
    pub cosmic_ray_threshold: f64,
    /// Polynomial order for baseline correction.
    pub baseline_polynomial_order: usize,
}

impl RamanProcessor {
    /// Create a new processor with default parameters.
    pub fn new(excitation_wavelength_nm: f64) -> Self {
        Self {
            spectral_resolution_cm1: 1.0,
            excitation_wavelength_nm,
            cosmic_ray_threshold: 5.0,
            baseline_polynomial_order: 5,
        }
    }

    /// Full preprocessing pipeline: cosmic ray removal, baseline correction,
    /// smoothing, and area normalization.
    pub fn preprocess(&self, spectrum: &RamanSpectrum) -> RamanSpectrum {
        let mut intensity = cosmic_ray_removal(&spectrum.intensity, self.cosmic_ray_threshold);
        let baseline = polynomial_baseline(&spectrum.wavenumber_cm1, &intensity, self.baseline_polynomial_order);
        for (i, b) in baseline.iter().enumerate() {
            intensity[i] -= b;
        }
        intensity = savitzky_golay_smooth(&intensity, 5, 2);
        intensity = normalize_area(&spectrum.wavenumber_cm1, &intensity);
        RamanSpectrum {
            wavenumber_cm1: spectrum.wavenumber_cm1.clone(),
            intensity,
            excitation_wavelength_nm: spectrum.excitation_wavelength_nm,
        }
    }
}

// ---------------------------------------------------------------------------
// Preprocessing: Cosmic ray removal
// ---------------------------------------------------------------------------

/// Remove cosmic ray spikes by comparing each point to a local median.
///
/// Points deviating more than `threshold` times the local MAD (median absolute
/// deviation) are replaced by the local median.
pub fn cosmic_ray_removal(intensity: &[f64], threshold: f64) -> Vec<f64> {
    let n = intensity.len();
    if n < 5 {
        return intensity.to_vec();
    }
    let half_win = 3;
    let mut result = intensity.to_vec();
    for i in 0..n {
        let lo = if i >= half_win { i - half_win } else { 0 };
        let hi = if i + half_win < n { i + half_win + 1 } else { n };
        let mut window: Vec<f64> = intensity[lo..hi].to_vec();
        window.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = window[window.len() / 2];
        let mad: Vec<f64> = window.iter().map(|v| (v - median).abs()).collect();
        let mut mad_sorted = mad.clone();
        mad_sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mad_val = mad_sorted[mad_sorted.len() / 2].max(1e-10);
        if (intensity[i] - median).abs() > threshold * mad_val * 1.4826 {
            result[i] = median;
        }
    }
    result
}

// ---------------------------------------------------------------------------
// Preprocessing: Baseline correction
// ---------------------------------------------------------------------------

/// Polynomial baseline fitting via least-squares.
///
/// Fits a polynomial of given `order` to the spectrum and returns the
/// evaluated baseline.
pub fn polynomial_baseline(wavenumber: &[f64], intensity: &[f64], order: usize) -> Vec<f64> {
    let n = wavenumber.len();
    if n == 0 {
        return vec![];
    }
    // Normalize x to [-1, 1] for numerical stability
    let x_min = wavenumber.iter().cloned().fold(f64::INFINITY, f64::min);
    let x_max = wavenumber.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let x_range = (x_max - x_min).max(1e-10);
    let x_norm: Vec<f64> = wavenumber.iter().map(|w| 2.0 * (w - x_min) / x_range - 1.0).collect();

    let coeffs = polyfit(&x_norm, intensity, order);
    x_norm.iter().map(|x| polyeval(&coeffs, *x)).collect()
}

/// Asymmetric least squares baseline correction (Whittaker smoother variant).
///
/// Iteratively fits a smooth baseline, weighting points below the fit more
/// heavily to push the baseline under the peaks.
pub fn asymmetric_least_squares_baseline(
    intensity: &[f64],
    lambda: f64,
    p: f64,
    iterations: usize,
) -> Vec<f64> {
    let n = intensity.len();
    if n < 3 {
        return intensity.to_vec();
    }
    let mut baseline = intensity.to_vec();
    let mut weights = vec![1.0; n];

    for _ in 0..iterations {
        // Simple smoothing step: weighted moving average approximation
        let mut smoothed = vec![0.0; n];
        let half_win = ((lambda.sqrt()) as usize).max(1).min(n / 2);
        for i in 0..n {
            let lo = if i >= half_win { i - half_win } else { 0 };
            let hi = if i + half_win < n { i + half_win + 1 } else { n };
            let mut sum_w = 0.0;
            let mut sum_wv = 0.0;
            for j in lo..hi {
                sum_w += weights[j];
                sum_wv += weights[j] * intensity[j];
            }
            smoothed[i] = if sum_w > 0.0 { sum_wv / sum_w } else { intensity[i] };
        }
        // Update weights: asymmetric weighting
        for i in 0..n {
            if intensity[i] > smoothed[i] {
                weights[i] = p;
            } else {
                weights[i] = 1.0 - p;
            }
        }
        baseline = smoothed;
    }
    baseline
}

/// Rubber band baseline correction.
///
/// Stretches a "rubber band" under the spectrum by finding the convex hull
/// of the bottom of the spectrum and interpolating.
pub fn rubber_band_baseline(wavenumber: &[f64], intensity: &[f64]) -> Vec<f64> {
    let n = wavenumber.len();
    if n < 3 {
        return vec![0.0; n];
    }

    // Find the lower convex hull of (wavenumber, intensity) points
    let mut hull_indices: Vec<usize> = Vec::new();
    for i in 0..n {
        while hull_indices.len() >= 2 {
            let a = hull_indices[hull_indices.len() - 2];
            let b = hull_indices[hull_indices.len() - 1];
            // Cross product to check if turning right (below)
            let cross = (wavenumber[b] - wavenumber[a]) * (intensity[i] - intensity[a])
                - (intensity[b] - intensity[a]) * (wavenumber[i] - wavenumber[a]);
            if cross <= 0.0 {
                hull_indices.pop();
            } else {
                break;
            }
        }
        hull_indices.push(i);
    }

    // Interpolate between hull points
    let mut baseline = vec![0.0; n];
    let mut seg = 0;
    for i in 0..n {
        while seg + 1 < hull_indices.len() - 1 && i > hull_indices[seg + 1] {
            seg += 1;
        }
        let a = hull_indices[seg];
        let b = hull_indices[(seg + 1).min(hull_indices.len() - 1)];
        if a == b {
            baseline[i] = intensity[a];
        } else {
            let t = (wavenumber[i] - wavenumber[a]) / (wavenumber[b] - wavenumber[a]);
            baseline[i] = intensity[a] + t * (intensity[b] - intensity[a]);
        }
    }
    baseline
}

// ---------------------------------------------------------------------------
// Preprocessing: Smoothing
// ---------------------------------------------------------------------------

/// Savitzky-Golay smoothing filter.
///
/// Fits a polynomial of degree `poly_order` to each window of `window_size`
/// points (must be odd) and evaluates the fit at the center point.
/// Preserves peak shapes better than moving average.
pub fn savitzky_golay_smooth(data: &[f64], window_size: usize, poly_order: usize) -> Vec<f64> {
    let n = data.len();
    let win = if window_size % 2 == 0 { window_size + 1 } else { window_size };
    let win = win.min(n);
    if win < poly_order + 1 || n < win {
        return data.to_vec();
    }
    let half = win / 2;

    // Compute SG coefficients for the center point
    let coeffs = sg_coefficients(win, poly_order);

    let mut result = vec![0.0; n];
    for i in 0..n {
        let mut val = 0.0;
        for j in 0..win {
            let idx = i as i64 + j as i64 - half as i64;
            let idx = idx.max(0).min(n as i64 - 1) as usize;
            val += coeffs[j] * data[idx];
        }
        result[i] = val;
    }
    result
}

/// Compute Savitzky-Golay convolution coefficients for the center point.
fn sg_coefficients(window_size: usize, poly_order: usize) -> Vec<f64> {
    let half = window_size as i64 / 2;
    let m = poly_order + 1;

    // Build the Vandermonde-like matrix J^T J and J^T * e_0
    // where J[i,k] = (i - half)^k and e_0 selects the 0th derivative at center
    let mut jtj = vec![0.0; m * m];
    let mut jte = vec![0.0; m];

    for i in 0..window_size {
        let x = i as f64 - half as f64;
        let mut powers = vec![1.0; m];
        for k in 1..m {
            powers[k] = powers[k - 1] * x;
        }
        for r in 0..m {
            for c in 0..m {
                jtj[r * m + c] += powers[r] * powers[c];
            }
            // We want the smoothed value at center (0th derivative)
            // which corresponds to row i of (J (J^T J)^-1 J^T)
            // For the center point evaluation, we compute row of J * inv(J^T J)
        }
    }

    // Solve (J^T J) a = e_k where e_k = [1, 0, 0, ...] for 0th derivative
    jte[0] = 1.0;
    let a = solve_linear_system(&jtj, &jte, m);

    // Coefficients: c_i = sum_k a_k * (i - half)^k
    let mut coeffs = vec![0.0; window_size];
    for i in 0..window_size {
        let x = i as f64 - half as f64;
        let mut val = 0.0;
        let mut xp = 1.0;
        for k in 0..m {
            val += a[k] * xp;
            xp *= x;
        }
        coeffs[i] = val;
    }
    coeffs
}

// ---------------------------------------------------------------------------
// Preprocessing: Normalization
// ---------------------------------------------------------------------------

/// Normalize intensity so that the integrated area equals 1.
pub fn normalize_area(wavenumber: &[f64], intensity: &[f64]) -> Vec<f64> {
    let area = trapezoidal_integrate(wavenumber, intensity).abs().max(1e-30);
    intensity.iter().map(|v| v / area).collect()
}

/// Normalize intensity to [0, 1] range (min-max).
pub fn normalize_min_max(intensity: &[f64]) -> Vec<f64> {
    let min = intensity.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = intensity.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let range = (max - min).max(1e-30);
    intensity.iter().map(|v| (v - min) / range).collect()
}

/// Normalize to unit peak height.
pub fn normalize_peak(intensity: &[f64]) -> Vec<f64> {
    let max = intensity.iter().cloned().fold(f64::NEG_INFINITY, f64::max).abs().max(1e-30);
    intensity.iter().map(|v| v / max).collect()
}

/// Standard Normal Variate (SNV) normalization.
///
/// Centers to zero mean and scales to unit standard deviation.
pub fn normalize_snv(intensity: &[f64]) -> Vec<f64> {
    let n = intensity.len() as f64;
    if n < 2.0 {
        return intensity.to_vec();
    }
    let mean = intensity.iter().sum::<f64>() / n;
    let var = intensity.iter().map(|v| (v - mean) * (v - mean)).sum::<f64>() / (n - 1.0);
    let std = var.sqrt().max(1e-30);
    intensity.iter().map(|v| (v - mean) / std).collect()
}

// ---------------------------------------------------------------------------
// Peak analysis
// ---------------------------------------------------------------------------

/// A detected peak with its parameters.
#[derive(Debug, Clone)]
pub struct RamanPeak {
    /// Index in the spectrum arrays.
    pub index: usize,
    /// Wavenumber position (cm^-1), if known.
    pub position_cm1: f64,
    /// Peak intensity.
    pub height: f64,
    /// Full width at half maximum (cm^-1).
    pub fwhm_cm1: f64,
    /// Integrated area under the peak.
    pub area: f64,
}

/// Find peaks in intensity data as local maxima above a threshold.
///
/// - `threshold`: minimum intensity for a peak
/// - `min_distance`: minimum separation between peaks (in samples)
///
/// Returns indices of detected peaks, sorted by intensity descending.
pub fn find_peaks(intensity: &[f64], threshold: f64, min_distance: usize) -> Vec<usize> {
    let n = intensity.len();
    if n < 3 {
        return vec![];
    }
    let mut candidates: Vec<(usize, f64)> = Vec::new();
    for i in 1..n - 1 {
        if intensity[i] > intensity[i - 1]
            && intensity[i] > intensity[i + 1]
            && intensity[i] >= threshold
        {
            candidates.push((i, intensity[i]));
        }
    }
    // Sort by intensity descending
    candidates.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

    // Apply minimum distance constraint (greedy NMS)
    let mut selected: Vec<usize> = Vec::new();
    for (idx, _) in &candidates {
        let ok = selected.iter().all(|&s| {
            (*idx as i64 - s as i64).unsigned_abs() as usize >= min_distance
        });
        if ok {
            selected.push(*idx);
        }
    }
    selected
}

/// Measure FWHM of a peak at index `peak_idx` in the spectrum.
///
/// Walks left and right from the peak to find where intensity drops below
/// half the peak height.
pub fn measure_fwhm(wavenumber: &[f64], intensity: &[f64], peak_idx: usize) -> f64 {
    let half_height = intensity[peak_idx] / 2.0;
    let n = intensity.len();

    // Walk left
    let mut left_wn = wavenumber[peak_idx];
    for i in (0..peak_idx).rev() {
        if intensity[i] <= half_height {
            // Linear interpolation
            let frac = (half_height - intensity[i]) / (intensity[i + 1] - intensity[i]).max(1e-30);
            left_wn = wavenumber[i] + frac * (wavenumber[i + 1] - wavenumber[i]);
            break;
        }
        if i == 0 {
            left_wn = wavenumber[0];
        }
    }

    // Walk right
    let mut right_wn = wavenumber[peak_idx];
    for i in (peak_idx + 1)..n {
        if intensity[i] <= half_height {
            let frac =
                (half_height - intensity[i]) / (intensity[i - 1] - intensity[i]).max(1e-30);
            right_wn = wavenumber[i] - frac * (wavenumber[i] - wavenumber[i - 1]);
            break;
        }
        if i == n - 1 {
            right_wn = wavenumber[n - 1];
        }
    }

    (right_wn - left_wn).abs()
}

/// Integrate the area under a peak between `left_idx` and `right_idx`.
pub fn peak_area(wavenumber: &[f64], intensity: &[f64], left_idx: usize, right_idx: usize) -> f64 {
    if left_idx >= right_idx || right_idx >= wavenumber.len() {
        return 0.0;
    }
    trapezoidal_integrate(
        &wavenumber[left_idx..=right_idx],
        &intensity[left_idx..=right_idx],
    )
}

/// Fit a Lorentzian to data near a peak. Returns (amplitude, center, gamma).
///
/// Uses a simple three-point estimation:
/// - center from peak position
/// - amplitude from peak height
/// - gamma from FWHM measurement
pub fn fit_lorentzian(wavenumber: &[f64], intensity: &[f64], peak_idx: usize) -> (f64, f64, f64) {
    let amplitude = intensity[peak_idx];
    let center = wavenumber[peak_idx];
    let gamma = measure_fwhm(wavenumber, intensity, peak_idx);
    (amplitude, center, gamma)
}

/// Fit a Gaussian to data near a peak. Returns (amplitude, center, gamma_fwhm).
pub fn fit_gaussian(wavenumber: &[f64], intensity: &[f64], peak_idx: usize) -> (f64, f64, f64) {
    let amplitude = intensity[peak_idx];
    let center = wavenumber[peak_idx];
    let gamma = measure_fwhm(wavenumber, intensity, peak_idx);
    (amplitude, center, gamma)
}

/// Analyze all peaks in a spectrum, returning detailed peak information.
pub fn analyze_peaks(
    spectrum: &RamanSpectrum,
    threshold: f64,
    min_distance: usize,
) -> Vec<RamanPeak> {
    let peaks_idx = find_peaks(&spectrum.intensity, threshold, min_distance);
    let mut result = Vec::new();
    for &idx in &peaks_idx {
        let fwhm = measure_fwhm(&spectrum.wavenumber_cm1, &spectrum.intensity, idx);
        // Estimate peak region as +/- 2*FWHM
        let wn = &spectrum.wavenumber_cm1;
        let center_wn = wn[idx];
        let left = wn
            .iter()
            .position(|w| *w >= center_wn - 2.0 * fwhm)
            .unwrap_or(0);
        let right = wn
            .iter()
            .rposition(|w| *w <= center_wn + 2.0 * fwhm)
            .unwrap_or(wn.len() - 1);
        let area = peak_area(wn, &spectrum.intensity, left, right);
        result.push(RamanPeak {
            index: idx,
            position_cm1: wn[idx],
            height: spectrum.intensity[idx],
            fwhm_cm1: fwhm,
            area,
        });
    }
    result
}

// ---------------------------------------------------------------------------
// Fluorescence background
// ---------------------------------------------------------------------------

/// Estimate fluorescence background using iterative weighted polynomial fitting.
///
/// This is a variant of the Vancouver algorithm: iteratively fit a polynomial,
/// then clip points above the fit to push the baseline under the peaks.
pub fn fluorescence_baseline(
    wavenumber: &[f64],
    intensity: &[f64],
    poly_order: usize,
    iterations: usize,
) -> Vec<f64> {
    let n = wavenumber.len();
    if n == 0 {
        return vec![];
    }
    let x_min = wavenumber.iter().cloned().fold(f64::INFINITY, f64::min);
    let x_max = wavenumber.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let x_range = (x_max - x_min).max(1e-10);
    let x_norm: Vec<f64> = wavenumber.iter().map(|w| 2.0 * (w - x_min) / x_range - 1.0).collect();

    let mut working = intensity.to_vec();

    for _ in 0..iterations {
        let coeffs = polyfit(&x_norm, &working, poly_order);
        let baseline: Vec<f64> = x_norm.iter().map(|x| polyeval(&coeffs, *x)).collect();
        // Clip: keep minimum of data and baseline for next iteration
        for i in 0..n {
            working[i] = working[i].min(baseline[i]);
        }
    }

    // Final fit
    let coeffs = polyfit(&x_norm, &working, poly_order);
    x_norm.iter().map(|x| polyeval(&coeffs, *x)).collect()
}

/// Compute fluorescence-to-Raman ratio (quality metric).
///
/// Higher ratios indicate more fluorescence interference.
pub fn fluorescence_raman_ratio(
    wavenumber: &[f64],
    intensity: &[f64],
    baseline: &[f64],
) -> f64 {
    let n = intensity.len();
    if n == 0 {
        return 0.0;
    }
    let fluorescence_power: f64 = baseline.iter().map(|b| b * b).sum::<f64>() / n as f64;
    let raman: Vec<f64> = intensity.iter().zip(baseline).map(|(i, b)| i - b).collect();
    let raman_power: f64 = raman.iter().map(|r| r * r).sum::<f64>() / n as f64;
    fluorescence_power.sqrt() / raman_power.sqrt().max(1e-30)
}

// ---------------------------------------------------------------------------
// Molecular identification
// ---------------------------------------------------------------------------

/// A reference spectrum entry in the spectral library.
#[derive(Debug, Clone)]
pub struct ReferenceSpectrum {
    /// Material name.
    pub name: String,
    /// Characteristic peak positions (cm^-1).
    pub peak_positions: Vec<f64>,
    /// Relative peak intensities.
    pub peak_intensities: Vec<f64>,
}

/// Built-in reference library of common materials.
pub fn reference_library() -> Vec<ReferenceSpectrum> {
    vec![
        ReferenceSpectrum {
            name: "Water".to_string(),
            peak_positions: vec![1640.0, 3400.0],
            peak_intensities: vec![0.3, 1.0],
        },
        ReferenceSpectrum {
            name: "Diamond".to_string(),
            peak_positions: vec![1332.0],
            peak_intensities: vec![1.0],
        },
        ReferenceSpectrum {
            name: "Graphite".to_string(),
            peak_positions: vec![1580.0, 1350.0, 2700.0],
            peak_intensities: vec![1.0, 0.3, 0.5],
        },
        ReferenceSpectrum {
            name: "Ethanol".to_string(),
            peak_positions: vec![880.0, 1050.0, 1090.0, 1454.0, 2930.0],
            peak_intensities: vec![1.0, 0.8, 0.6, 0.4, 0.9],
        },
        ReferenceSpectrum {
            name: "Polystyrene".to_string(),
            peak_positions: vec![621.0, 1001.0, 1031.0, 1602.0, 3054.0],
            peak_intensities: vec![0.5, 1.0, 0.7, 0.6, 0.4],
        },
        ReferenceSpectrum {
            name: "Silicon".to_string(),
            peak_positions: vec![520.0],
            peak_intensities: vec![1.0],
        },
        ReferenceSpectrum {
            name: "Calcite".to_string(),
            peak_positions: vec![1086.0, 712.0, 282.0],
            peak_intensities: vec![1.0, 0.3, 0.4],
        },
    ]
}

/// Spectral match result.
#[derive(Debug, Clone)]
pub struct MatchResult {
    /// Material name.
    pub name: String,
    /// Hit Quality Index [0, 1] where 1 is perfect match.
    pub hqi: f64,
    /// Correlation coefficient.
    pub correlation: f64,
    /// Euclidean distance (lower = better match).
    pub distance: f64,
}

/// Match an unknown spectrum against the reference library.
///
/// Compares detected peak positions against reference peaks within
/// a tolerance window.
pub fn identify_material(
    detected_peaks: &[f64],
    tolerance_cm1: f64,
) -> Vec<MatchResult> {
    let library = reference_library();
    let mut results: Vec<MatchResult> = Vec::new();

    for reference in &library {
        let (hqi, corr, dist) =
            compute_match_score(detected_peaks, &reference.peak_positions, tolerance_cm1);
        results.push(MatchResult {
            name: reference.name.clone(),
            hqi,
            correlation: corr,
            distance: dist,
        });
    }

    // Sort by HQI descending
    results.sort_by(|a, b| b.hqi.partial_cmp(&a.hqi).unwrap());
    results
}

/// Compute match metrics between detected and reference peaks.
fn compute_match_score(detected: &[f64], reference: &[f64], tolerance: f64) -> (f64, f64, f64) {
    if reference.is_empty() || detected.is_empty() {
        return (0.0, 0.0, f64::MAX);
    }

    // Count matched peaks
    let mut matched = 0usize;
    let mut total_distance = 0.0;
    for ref_peak in reference {
        let closest = detected
            .iter()
            .map(|d| (d - ref_peak).abs())
            .fold(f64::INFINITY, f64::min);
        if closest <= tolerance {
            matched += 1;
            total_distance += closest;
        } else {
            total_distance += tolerance * 2.0; // penalty
        }
    }

    // HQI: fraction of reference peaks matched, penalized by position error
    let match_fraction = matched as f64 / reference.len() as f64;
    let avg_error = total_distance / reference.len() as f64;
    let position_quality = (1.0 - avg_error / (tolerance * 2.0)).max(0.0);
    let hqi = match_fraction * position_quality;

    // Simple correlation: based on peak matching pattern
    let correlation = match_fraction;

    // Euclidean distance of matched positions
    let distance = total_distance;

    (hqi, correlation, distance)
}

/// Hit Quality Index between two full spectra (interpolated comparison).
///
/// Returns correlation coefficient in [0, 1].
pub fn spectral_correlation(spec_a: &[f64], spec_b: &[f64]) -> f64 {
    if spec_a.len() != spec_b.len() || spec_a.is_empty() {
        return 0.0;
    }
    let n = spec_a.len() as f64;
    let mean_a = spec_a.iter().sum::<f64>() / n;
    let mean_b = spec_b.iter().sum::<f64>() / n;
    let mut cov = 0.0;
    let mut var_a = 0.0;
    let mut var_b = 0.0;
    for i in 0..spec_a.len() {
        let da = spec_a[i] - mean_a;
        let db = spec_b[i] - mean_b;
        cov += da * db;
        var_a += da * da;
        var_b += db * db;
    }
    let denom = (var_a * var_b).sqrt();
    if denom < 1e-30 {
        return 0.0;
    }
    (cov / denom).max(0.0)
}

/// Euclidean spectral distance between two spectra.
pub fn spectral_distance(spec_a: &[f64], spec_b: &[f64]) -> f64 {
    if spec_a.len() != spec_b.len() {
        return f64::MAX;
    }
    spec_a
        .iter()
        .zip(spec_b)
        .map(|(a, b)| (a - b) * (a - b))
        .sum::<f64>()
        .sqrt()
}

// ---------------------------------------------------------------------------
// Quantitative analysis
// ---------------------------------------------------------------------------

/// Calibration data point: (concentration, measured_value).
#[derive(Debug, Clone, Copy)]
pub struct CalibrationPoint {
    pub concentration: f64,
    pub signal: f64,
}

/// Linear calibration curve: signal = slope * concentration + intercept.
#[derive(Debug, Clone)]
pub struct CalibrationCurve {
    pub slope: f64,
    pub intercept: f64,
    pub r_squared: f64,
}

impl CalibrationCurve {
    /// Predict concentration from a measured signal.
    pub fn predict_concentration(&self, signal: f64) -> f64 {
        if self.slope.abs() < 1e-30 {
            return 0.0;
        }
        (signal - self.intercept) / self.slope
    }

    /// Predict signal from a known concentration.
    pub fn predict_signal(&self, concentration: f64) -> f64 {
        self.slope * concentration + self.intercept
    }
}

/// Fit a linear calibration curve to calibration data points.
pub fn fit_calibration_curve(points: &[CalibrationPoint]) -> CalibrationCurve {
    let n = points.len() as f64;
    if n < 2.0 {
        return CalibrationCurve {
            slope: 0.0,
            intercept: 0.0,
            r_squared: 0.0,
        };
    }
    let sum_x: f64 = points.iter().map(|p| p.concentration).sum();
    let sum_y: f64 = points.iter().map(|p| p.signal).sum();
    let sum_xx: f64 = points.iter().map(|p| p.concentration * p.concentration).sum();
    let sum_xy: f64 = points.iter().map(|p| p.concentration * p.signal).sum();

    let denom = n * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return CalibrationCurve {
            slope: 0.0,
            intercept: sum_y / n,
            r_squared: 0.0,
        };
    }

    let slope = (n * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / n;

    // R-squared
    let mean_y = sum_y / n;
    let ss_tot: f64 = points.iter().map(|p| (p.signal - mean_y).powi(2)).sum();
    let ss_res: f64 = points
        .iter()
        .map(|p| {
            let pred = slope * p.concentration + intercept;
            (p.signal - pred).powi(2)
        })
        .sum();
    let r_squared = if ss_tot > 1e-30 {
        1.0 - ss_res / ss_tot
    } else {
        0.0
    };

    CalibrationCurve {
        slope,
        intercept,
        r_squared,
    }
}

/// Estimate limit of detection (LOD) from calibration sensitivity and noise.
///
/// LOD = 3 * sigma_noise / sensitivity
pub fn estimate_lod(noise_std: f64, sensitivity: f64) -> f64 {
    if sensitivity.abs() < 1e-30 {
        return f64::INFINITY;
    }
    3.0 * noise_std / sensitivity.abs()
}

// ---------------------------------------------------------------------------
// Depolarization ratio
// ---------------------------------------------------------------------------

/// Compute depolarization ratio from parallel and perpendicular intensities.
///
/// rho = I_perp / I_parallel
///
/// - Totally symmetric vibrations: rho < 3/4
/// - Asymmetric/degenerate vibrations: rho = 3/4
pub fn depolarization_ratio(i_parallel: f64, i_perpendicular: f64) -> f64 {
    if i_parallel.abs() < 1e-30 {
        return 0.0;
    }
    i_perpendicular / i_parallel
}

/// Classify vibration symmetry from depolarization ratio.
pub fn classify_vibration_symmetry(rho: f64) -> &'static str {
    if rho < 0.0 {
        "invalid"
    } else if rho < 0.75 {
        "symmetric"
    } else {
        "asymmetric"
    }
}

// ---------------------------------------------------------------------------
// Temperature from anti-Stokes/Stokes ratio
// ---------------------------------------------------------------------------

/// Extract temperature from the anti-Stokes/Stokes intensity ratio.
///
/// The ratio of anti-Stokes to Stokes intensity at vibrational frequency nu_v
/// (cm^-1) is:
///
/// ```text
/// I_aS / I_S = ((nu0 + nu_v) / (nu0 - nu_v))^4 * exp(-h*c*nu_v / (k_B * T))
/// ```
///
/// where `nu0` is the laser frequency in cm^-1.
///
/// Returns temperature in Kelvin.
pub fn temperature_from_stokes_ratio(
    ratio_as_s: f64,
    nu_vibration_cm1: f64,
    excitation_wavelength_nm: f64,
) -> f64 {
    if ratio_as_s <= 0.0 || nu_vibration_cm1 <= 0.0 {
        return 0.0;
    }

    // Convert excitation wavelength to wavenumber (cm^-1)
    let nu0 = 1.0e7 / excitation_wavelength_nm;

    // Frequency correction factor
    let freq_ratio = (nu0 + nu_vibration_cm1) / (nu0 - nu_vibration_cm1);
    let freq_factor = freq_ratio.powi(4);

    // ratio = freq_factor * exp(-h*c*nu_v / (k_B * T))
    // => T = -h*c*nu_v / (k_B * ln(ratio / freq_factor))
    let exponent_arg = ratio_as_s / freq_factor;
    if exponent_arg >= 1.0 || exponent_arg <= 0.0 {
        return 0.0; // Unphysical
    }

    let hc_over_kb = H_PLANCK * C_CM_S / K_BOLTZMANN; // cm * K
    let temperature = -hc_over_kb * nu_vibration_cm1 / exponent_arg.ln();

    if temperature > 0.0 {
        temperature
    } else {
        0.0
    }
}

/// Compute the expected anti-Stokes/Stokes ratio at a given temperature.
pub fn stokes_ratio_at_temperature(
    nu_vibration_cm1: f64,
    excitation_wavelength_nm: f64,
    temperature_k: f64,
) -> f64 {
    if temperature_k <= 0.0 || nu_vibration_cm1 <= 0.0 {
        return 0.0;
    }
    let nu0 = 1.0e7 / excitation_wavelength_nm;
    let freq_ratio = (nu0 + nu_vibration_cm1) / (nu0 - nu_vibration_cm1);
    let freq_factor = freq_ratio.powi(4);
    let hc_over_kb = H_PLANCK * C_CM_S / K_BOLTZMANN;
    freq_factor * (-hc_over_kb * nu_vibration_cm1 / temperature_k).exp()
}

// ---------------------------------------------------------------------------
// SNR and quality metrics
// ---------------------------------------------------------------------------

/// Estimate signal-to-noise ratio from a spectrum.
///
/// Uses a simple method: signal power from peak region, noise from
/// a flat region at the spectrum edges.
pub fn estimate_snr(intensity: &[f64], noise_region_fraction: f64) -> f64 {
    let n = intensity.len();
    if n < 10 {
        return 0.0;
    }
    let noise_samples = (n as f64 * noise_region_fraction).max(3.0) as usize;

    // Use first and last samples as noise estimate
    let noise_region: Vec<f64> = intensity[..noise_samples.min(n)]
        .iter()
        .chain(intensity[n.saturating_sub(noise_samples)..].iter())
        .cloned()
        .collect();

    let noise_mean = noise_region.iter().sum::<f64>() / noise_region.len() as f64;
    let noise_var = noise_region
        .iter()
        .map(|v| (v - noise_mean).powi(2))
        .sum::<f64>()
        / noise_region.len() as f64;
    let noise_std = noise_var.sqrt().max(1e-30);

    let signal_max = intensity.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let signal = (signal_max - noise_mean).max(0.0);

    signal / noise_std
}

/// Spectral quality metric combining SNR, baseline flatness, and peak count.
///
/// Returns a score in [0, 1].
pub fn spectral_quality(
    wavenumber: &[f64],
    intensity: &[f64],
    snr: f64,
    peak_count: usize,
) -> f64 {
    // SNR contribution (saturates at ~100)
    let snr_score = (snr / 100.0).min(1.0).max(0.0);

    // Peak count contribution (more peaks = better identified)
    let peak_score = (peak_count as f64 / 10.0).min(1.0);

    // Baseline flatness (variance of first derivative)
    let n = intensity.len();
    let mut deriv_var = 0.0;
    if n > 1 {
        let derivs: Vec<f64> = (1..n)
            .map(|i| {
                (intensity[i] - intensity[i - 1])
                    / (wavenumber[i] - wavenumber[i - 1]).max(1e-10)
            })
            .collect();
        let mean_d = derivs.iter().sum::<f64>() / derivs.len() as f64;
        deriv_var = derivs.iter().map(|d| (d - mean_d).powi(2)).sum::<f64>() / derivs.len() as f64;
    }
    let flatness_score = (1.0 / (1.0 + deriv_var.sqrt())).min(1.0);

    0.5 * snr_score + 0.3 * peak_score + 0.2 * flatness_score
}

// ---------------------------------------------------------------------------
// Math utilities
// ---------------------------------------------------------------------------

/// Trapezoidal integration of (x, y) data.
fn trapezoidal_integrate(x: &[f64], y: &[f64]) -> f64 {
    let n = x.len().min(y.len());
    if n < 2 {
        return 0.0;
    }
    let mut sum = 0.0;
    for i in 1..n {
        sum += (x[i] - x[i - 1]) * (y[i] + y[i - 1]) / 2.0;
    }
    sum
}

/// Polynomial fit via least squares (normal equations).
///
/// Returns coefficients [c0, c1, ..., cn] where y = c0 + c1*x + c2*x^2 + ...
fn polyfit(x: &[f64], y: &[f64], order: usize) -> Vec<f64> {
    let n = x.len();
    let m = order + 1;
    if n < m {
        return vec![0.0; m];
    }

    // Build normal equations: (X^T X) c = X^T y
    let mut xtx = vec![0.0; m * m];
    let mut xty = vec![0.0; m];

    for i in 0..n {
        let mut xi_powers = vec![1.0; m];
        for k in 1..m {
            xi_powers[k] = xi_powers[k - 1] * x[i];
        }
        for r in 0..m {
            for c in 0..m {
                xtx[r * m + c] += xi_powers[r] * xi_powers[c];
            }
            xty[r] += xi_powers[r] * y[i];
        }
    }

    solve_linear_system(&xtx, &xty, m)
}

/// Evaluate polynomial at a point.
fn polyeval(coeffs: &[f64], x: f64) -> f64 {
    // Horner's method
    let mut result = 0.0;
    for c in coeffs.iter().rev() {
        result = result * x + c;
    }
    result
}

/// Solve a linear system Ax = b via Gaussian elimination with partial pivoting.
fn solve_linear_system(a_flat: &[f64], b: &[f64], n: usize) -> Vec<f64> {
    let mut a = vec![0.0; n * n];
    a.copy_from_slice(&a_flat[..n * n]);
    let mut x = b.to_vec();

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_val = a[col * n + col].abs();
        let mut max_row = col;
        for row in (col + 1)..n {
            let val = a[row * n + col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }

        // Swap rows
        if max_row != col {
            for j in 0..n {
                a.swap(col * n + j, max_row * n + j);
            }
            x.swap(col, max_row);
        }

        let pivot = a[col * n + col];
        if pivot.abs() < 1e-15 {
            continue;
        }

        // Eliminate below
        for row in (col + 1)..n {
            let factor = a[row * n + col] / pivot;
            for j in col..n {
                a[row * n + j] -= factor * a[col * n + j];
            }
            x[row] -= factor * x[col];
        }
    }

    // Back substitution
    for col in (0..n).rev() {
        let pivot = a[col * n + col];
        if pivot.abs() < 1e-15 {
            x[col] = 0.0;
            continue;
        }
        for j in (col + 1)..n {
            x[col] -= a[col * n + j] * x[j];
        }
        x[col] /= pivot;
    }

    x
}

// ============================= TESTS =====================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // --- Wavelength/wavenumber conversion ---

    #[test]
    fn test_wavelength_to_raman_shift_stokes() {
        // 532 nm excitation, 550 nm scattered (Stokes)
        let shift = wavelength_to_raman_shift(532.0, 550.0);
        assert!(shift > 0.0, "Stokes shift should be positive");
        // Expected: (1/532 - 1/550) * 1e7 = (0.001879699 - 0.001818182) * 1e7 = 615.17
        assert!(approx_eq(shift, 615.17, 1.0));
    }

    #[test]
    fn test_wavelength_to_raman_shift_anti_stokes() {
        // 532 nm excitation, 515 nm scattered (anti-Stokes)
        let shift = wavelength_to_raman_shift(532.0, 515.0);
        assert!(shift < 0.0, "Anti-Stokes shift should be negative");
    }

    #[test]
    fn test_raman_shift_roundtrip() {
        let exc = 532.0;
        let shift = 1000.0;
        let scattered = raman_shift_to_wavelength(exc, shift);
        let recovered = wavelength_to_raman_shift(exc, scattered);
        assert!(approx_eq(recovered, shift, 0.01));
    }

    #[test]
    fn test_is_stokes() {
        assert!(is_stokes(1000.0));
        assert!(!is_stokes(-500.0));
        assert!(!is_stokes(0.0));
    }

    // --- Line shapes ---

    #[test]
    fn test_lorentzian_peak_center() {
        let val = lorentzian_peak(1000.0, 100.0, 1000.0, 10.0);
        assert!(approx_eq(val, 100.0, EPS));
    }

    #[test]
    fn test_lorentzian_half_max() {
        // At nu = nu0 +/- gamma/2, intensity should be A/2
        let gamma = 10.0;
        let val = lorentzian_peak(1000.0 + gamma / 2.0, 100.0, 1000.0, gamma);
        assert!(approx_eq(val, 50.0, EPS));
    }

    #[test]
    fn test_gaussian_peak_center() {
        let val = gaussian_peak(500.0, 80.0, 500.0, 20.0);
        assert!(approx_eq(val, 80.0, EPS));
    }

    #[test]
    fn test_gaussian_half_max() {
        let gamma = 20.0;
        let val = gaussian_peak(500.0 + gamma / 2.0, 100.0, 500.0, gamma);
        assert!(approx_eq(val, 50.0, 0.01));
    }

    #[test]
    fn test_voigt_pure_lorentzian() {
        let val = voigt_peak(1000.0, 100.0, 1000.0, 10.0, 1.0);
        let lor = lorentzian_peak(1000.0, 100.0, 1000.0, 10.0);
        assert!(approx_eq(val, lor, EPS));
    }

    #[test]
    fn test_voigt_pure_gaussian() {
        let val = voigt_peak(1000.0, 100.0, 1000.0, 10.0, 0.0);
        let gau = gaussian_peak(1000.0, 100.0, 1000.0, 10.0);
        assert!(approx_eq(val, gau, EPS));
    }

    // --- RamanSpectrum ---

    #[test]
    fn test_spectrum_creation() {
        let wn: Vec<f64> = (0..100).map(|i| 200.0 + i as f64).collect();
        let int: Vec<f64> = wn.iter().map(|w| lorentzian_peak(*w, 50.0, 250.0, 10.0)).collect();
        let spec = RamanSpectrum::new(wn, int, 532.0);
        assert_eq!(spec.len(), 100);
        assert!(!spec.is_empty());
        assert!(spec.max_intensity() > 0.0);
    }

    #[test]
    #[should_panic]
    fn test_spectrum_length_mismatch() {
        RamanSpectrum::new(vec![1.0, 2.0], vec![1.0], 532.0);
    }

    #[test]
    fn test_spectrum_integrated_area() {
        // Constant function: area = width * height
        let wn: Vec<f64> = (0..101).map(|i| i as f64).collect();
        let int = vec![5.0; 101];
        let spec = RamanSpectrum::new(wn, int, 532.0);
        assert!(approx_eq(spec.integrated_area(), 500.0, 0.1));
    }

    // --- Cosmic ray removal ---

    #[test]
    fn test_cosmic_ray_removal() {
        let mut data = vec![10.0; 50];
        // Insert a spike
        data[25] = 1000.0;
        let cleaned = cosmic_ray_removal(&data, 3.0);
        assert!(cleaned[25] < 100.0, "Spike should be removed");
        assert!(approx_eq(cleaned[10], 10.0, 0.1));
    }

    #[test]
    fn test_cosmic_ray_preserves_signal() {
        let data: Vec<f64> = (0..100).map(|i| (i as f64 / 10.0).sin() * 10.0 + 50.0).collect();
        let cleaned = cosmic_ray_removal(&data, 5.0);
        // Most points should be unchanged
        let unchanged = data.iter().zip(cleaned.iter())
            .filter(|(a, b)| approx_eq(**a, **b, 0.01))
            .count();
        assert!(unchanged > 90);
    }

    // --- Baseline correction ---

    #[test]
    fn test_polynomial_baseline_flat() {
        let wn: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let int = vec![10.0; 100];
        let baseline = polynomial_baseline(&wn, &int, 1);
        for b in &baseline {
            assert!(approx_eq(*b, 10.0, 0.5));
        }
    }

    #[test]
    fn test_polynomial_baseline_linear() {
        let wn: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let int: Vec<f64> = wn.iter().map(|w| 2.0 * w + 5.0).collect();
        let baseline = polynomial_baseline(&wn, &int, 1);
        for i in 0..100 {
            assert!(approx_eq(baseline[i], int[i], 0.5));
        }
    }

    #[test]
    fn test_rubber_band_baseline() {
        let wn: Vec<f64> = (0..100).map(|i| i as f64).collect();
        // A signal with a peak on a flat baseline
        let int: Vec<f64> = wn.iter().map(|w| {
            5.0 + lorentzian_peak(*w, 100.0, 50.0, 10.0)
        }).collect();
        let baseline = rubber_band_baseline(&wn, &int);
        // Baseline should be below the peak region
        assert!(baseline[50] <= int[50]);
    }

    // --- Savitzky-Golay smoothing ---

    #[test]
    fn test_sg_smooth_constant() {
        let data = vec![10.0; 50];
        let smoothed = savitzky_golay_smooth(&data, 5, 2);
        for v in &smoothed {
            assert!(approx_eq(*v, 10.0, 0.01));
        }
    }

    #[test]
    fn test_sg_smooth_reduces_noise() {
        // Noisy signal
        let data: Vec<f64> = (0..100).map(|i| {
            50.0 + ((i * 73 + 17) % 37) as f64 * 0.1 - 1.8
        }).collect();
        let smoothed = savitzky_golay_smooth(&data, 7, 2);
        // Variance should be reduced
        let mean_d: f64 = data.iter().sum::<f64>() / data.len() as f64;
        let var_d: f64 = data.iter().map(|v| (v - mean_d).powi(2)).sum::<f64>() / data.len() as f64;
        let mean_s: f64 = smoothed.iter().sum::<f64>() / smoothed.len() as f64;
        let var_s: f64 = smoothed.iter().map(|v| (v - mean_s).powi(2)).sum::<f64>() / smoothed.len() as f64;
        assert!(var_s < var_d, "Smoothing should reduce variance: smoothed={} orig={}", var_s, var_d);
    }

    // --- Normalization ---

    #[test]
    fn test_normalize_area() {
        let wn: Vec<f64> = (0..101).map(|i| i as f64).collect();
        let int = vec![10.0; 101];
        let norm = normalize_area(&wn, &int);
        let area = trapezoidal_integrate(&wn, &norm);
        assert!(approx_eq(area, 1.0, 0.01));
    }

    #[test]
    fn test_normalize_min_max() {
        let int = vec![5.0, 10.0, 15.0, 20.0, 25.0];
        let norm = normalize_min_max(&int);
        assert!(approx_eq(norm[0], 0.0, EPS));
        assert!(approx_eq(norm[4], 1.0, EPS));
    }

    #[test]
    fn test_normalize_peak() {
        let int = vec![5.0, 10.0, 100.0, 20.0, 5.0];
        let norm = normalize_peak(&int);
        assert!(approx_eq(norm[2], 1.0, EPS));
        assert!(approx_eq(norm[0], 0.05, EPS));
    }

    #[test]
    fn test_normalize_snv() {
        let int = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let norm = normalize_snv(&int);
        let mean: f64 = norm.iter().sum::<f64>() / norm.len() as f64;
        let var: f64 = norm.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (norm.len() as f64 - 1.0);
        assert!(approx_eq(mean, 0.0, EPS));
        assert!(approx_eq(var, 1.0, 0.01));
    }

    // --- Peak analysis ---

    #[test]
    fn test_find_peaks_single() {
        let data: Vec<f64> = (0..100).map(|i| lorentzian_peak(i as f64, 100.0, 50.0, 5.0)).collect();
        let peaks = find_peaks(&data, 10.0, 3);
        assert!(!peaks.is_empty());
        assert_eq!(peaks[0], 50);
    }

    #[test]
    fn test_find_peaks_multiple() {
        let data: Vec<f64> = (0..200).map(|i| {
            lorentzian_peak(i as f64, 100.0, 50.0, 5.0)
                + lorentzian_peak(i as f64, 80.0, 150.0, 5.0)
        }).collect();
        let peaks = find_peaks(&data, 10.0, 10);
        assert!(peaks.len() >= 2);
    }

    #[test]
    fn test_measure_fwhm_lorentzian() {
        let gamma = 10.0;
        let wn: Vec<f64> = (0..500).map(|i| 800.0 + i as f64 * 0.5).collect();
        let int: Vec<f64> = wn.iter().map(|w| lorentzian_peak(*w, 100.0, 1000.0, gamma)).collect();
        let peak_idx = int.iter().enumerate().max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0;
        let fwhm = measure_fwhm(&wn, &int, peak_idx);
        assert!(approx_eq(fwhm, gamma, 1.0), "FWHM {} should be near gamma {}", fwhm, gamma);
    }

    #[test]
    fn test_peak_area_positive() {
        let wn: Vec<f64> = (0..200).map(|i| 900.0 + i as f64).collect();
        let int: Vec<f64> = wn.iter().map(|w| lorentzian_peak(*w, 100.0, 1000.0, 10.0)).collect();
        let area = peak_area(&wn, &int, 80, 120);
        assert!(area > 0.0);
    }

    #[test]
    fn test_fit_lorentzian() {
        let wn: Vec<f64> = (0..500).map(|i| 800.0 + i as f64 * 0.5).collect();
        let int: Vec<f64> = wn.iter().map(|w| lorentzian_peak(*w, 100.0, 1000.0, 10.0)).collect();
        let peak_idx = int.iter().enumerate().max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0;
        let (amp, center, gamma) = fit_lorentzian(&wn, &int, peak_idx);
        assert!(approx_eq(amp, 100.0, 0.1));
        assert!(approx_eq(center, 1000.0, 1.0));
        assert!(approx_eq(gamma, 10.0, 1.5));
    }

    #[test]
    fn test_analyze_peaks() {
        let wn: Vec<f64> = (0..500).map(|i| 800.0 + i as f64 * 2.0).collect();
        let int: Vec<f64> = wn.iter().map(|w| {
            lorentzian_peak(*w, 100.0, 1001.0, 10.0)
                + lorentzian_peak(*w, 60.0, 1602.0, 15.0)
        }).collect();
        let spec = RamanSpectrum::new(wn, int, 532.0);
        let peaks = analyze_peaks(&spec, 5.0, 5);
        assert!(peaks.len() >= 2);
        // Should find peaks near 1001 and 1602
        let positions: Vec<f64> = peaks.iter().map(|p| p.position_cm1).collect();
        assert!(positions.iter().any(|p| approx_eq(*p, 1001.0, 5.0)));
        assert!(positions.iter().any(|p| approx_eq(*p, 1602.0, 5.0)));
    }

    // --- Fluorescence background ---

    #[test]
    fn test_fluorescence_baseline() {
        let wn: Vec<f64> = (0..200).map(|i| i as f64 * 5.0).collect();
        // Fluorescence = quadratic + Raman peak
        let int: Vec<f64> = wn.iter().map(|w| {
            0.001 * w * w + lorentzian_peak(*w, 100.0, 500.0, 20.0)
        }).collect();
        let baseline = fluorescence_baseline(&wn, &int, 5, 20);
        // Baseline should be close to the quadratic part
        // At 100 (far from peak), baseline should approximate 0.001 * 100^2 = 10
        let idx = 20; // wn = 100
        assert!(baseline[idx] < int[idx], "Baseline should be below signal at peak");
    }

    #[test]
    fn test_fluorescence_raman_ratio() {
        let wn: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let baseline = vec![50.0; 100];
        let int: Vec<f64> = baseline.iter().enumerate().map(|(i, b)| {
            b + lorentzian_peak(i as f64, 100.0, 50.0, 5.0)
        }).collect();
        let ratio = fluorescence_raman_ratio(&wn, &int, &baseline);
        assert!(ratio > 0.0);
    }

    // --- Molecular identification ---

    #[test]
    fn test_reference_library_not_empty() {
        let lib = reference_library();
        assert!(lib.len() >= 5);
    }

    #[test]
    fn test_identify_polystyrene() {
        let detected = vec![1001.0, 1031.0, 1602.0];
        let results = identify_material(&detected, 20.0);
        assert!(!results.is_empty());
        // Polystyrene should be the best match
        assert_eq!(results[0].name, "Polystyrene");
        assert!(results[0].hqi > 0.3);
    }

    #[test]
    fn test_identify_diamond() {
        let detected = vec![1332.0];
        let results = identify_material(&detected, 15.0);
        assert!(!results.is_empty());
        assert_eq!(results[0].name, "Diamond");
    }

    #[test]
    fn test_spectral_correlation_identical() {
        let a = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let corr = spectral_correlation(&a, &a);
        assert!(approx_eq(corr, 1.0, 0.01));
    }

    #[test]
    fn test_spectral_distance_zero() {
        let a = vec![1.0, 2.0, 3.0];
        let d = spectral_distance(&a, &a);
        assert!(approx_eq(d, 0.0, EPS));
    }

    // --- Quantitative analysis ---

    #[test]
    fn test_calibration_curve() {
        let points = vec![
            CalibrationPoint { concentration: 0.0, signal: 1.0 },
            CalibrationPoint { concentration: 1.0, signal: 3.0 },
            CalibrationPoint { concentration: 2.0, signal: 5.0 },
            CalibrationPoint { concentration: 3.0, signal: 7.0 },
        ];
        let curve = fit_calibration_curve(&points);
        assert!(approx_eq(curve.slope, 2.0, 0.01));
        assert!(approx_eq(curve.intercept, 1.0, 0.01));
        assert!(curve.r_squared > 0.99);
    }

    #[test]
    fn test_calibration_predict() {
        let points = vec![
            CalibrationPoint { concentration: 0.0, signal: 0.0 },
            CalibrationPoint { concentration: 10.0, signal: 100.0 },
        ];
        let curve = fit_calibration_curve(&points);
        assert!(approx_eq(curve.predict_concentration(50.0), 5.0, 0.1));
        assert!(approx_eq(curve.predict_signal(5.0), 50.0, 0.1));
    }

    #[test]
    fn test_estimate_lod() {
        let lod = estimate_lod(0.1, 10.0);
        assert!(approx_eq(lod, 0.03, 0.001));
    }

    #[test]
    fn test_estimate_lod_zero_sensitivity() {
        let lod = estimate_lod(0.1, 0.0);
        assert!(lod.is_infinite());
    }

    // --- Depolarization ratio ---

    #[test]
    fn test_depolarization_symmetric() {
        let rho = depolarization_ratio(100.0, 30.0);
        assert!(approx_eq(rho, 0.3, EPS));
        assert_eq!(classify_vibration_symmetry(rho), "symmetric");
    }

    #[test]
    fn test_depolarization_asymmetric() {
        let rho = depolarization_ratio(100.0, 75.0);
        assert!(approx_eq(rho, 0.75, EPS));
        assert_eq!(classify_vibration_symmetry(rho), "asymmetric");
    }

    // --- Temperature from Stokes ratio ---

    #[test]
    fn test_temperature_roundtrip() {
        let temp_expected = 300.0; // K
        let nu_v = 520.0; // cm-1 (silicon)
        let exc = 532.0; // nm

        let ratio = stokes_ratio_at_temperature(nu_v, exc, temp_expected);
        assert!(ratio > 0.0 && ratio < 1.0, "Ratio {} should be between 0 and 1", ratio);

        let temp_recovered = temperature_from_stokes_ratio(ratio, nu_v, exc);
        assert!(
            approx_eq(temp_recovered, temp_expected, 1.0),
            "Recovered temp {} should be near expected {}",
            temp_recovered,
            temp_expected
        );
    }

    #[test]
    fn test_temperature_higher_gives_larger_ratio() {
        let nu_v = 520.0;
        let exc = 532.0;
        let r300 = stokes_ratio_at_temperature(nu_v, exc, 300.0);
        let r600 = stokes_ratio_at_temperature(nu_v, exc, 600.0);
        assert!(r600 > r300, "Higher temp should give higher aS/S ratio");
    }

    // --- SNR ---

    #[test]
    fn test_estimate_snr_clean() {
        let mut data = vec![1.0; 100];
        // Add a strong peak in the middle
        for i in 40..60 {
            data[i] = 100.0;
        }
        let snr = estimate_snr(&data, 0.1);
        assert!(snr > 10.0, "SNR {} should be high for clean peaked signal", snr);
    }

    #[test]
    fn test_spectral_quality_good() {
        let wn: Vec<f64> = (0..200).map(|i| i as f64).collect();
        let int: Vec<f64> = wn.iter().map(|w| lorentzian_peak(*w, 100.0, 100.0, 5.0)).collect();
        let q = spectral_quality(&wn, &int, 50.0, 3);
        assert!(q > 0.0 && q <= 1.0);
    }

    // --- Processor pipeline ---

    #[test]
    fn test_processor_preprocess() {
        let wn: Vec<f64> = (0..200).map(|i| 800.0 + i as f64 * 2.0).collect();
        let int: Vec<f64> = wn.iter().map(|w| {
            5.0 + 0.001 * (w - 1000.0) + lorentzian_peak(*w, 100.0, 1000.0, 10.0)
        }).collect();
        let spec = RamanSpectrum::new(wn, int, 532.0);
        let proc = RamanProcessor::new(532.0);
        let result = proc.preprocess(&spec);
        assert_eq!(result.len(), spec.len());
        // Area should be normalized to ~1
        let area = result.integrated_area();
        assert!(approx_eq(area, 1.0, 0.1), "Area {} should be ~1.0", area);
    }

    #[test]
    fn test_asymmetric_least_squares() {
        let data: Vec<f64> = (0..100).map(|i| {
            10.0 + 0.01 * (i as f64).powi(2) + lorentzian_peak(i as f64, 50.0, 50.0, 5.0)
        }).collect();
        let baseline = asymmetric_least_squares_baseline(&data, 100.0, 0.01, 10);
        assert_eq!(baseline.len(), data.len());
        // Baseline at the peak should be below the data
        assert!(baseline[50] < data[50]);
    }

    // --- Math utilities ---

    #[test]
    fn test_trapezoidal_integrate_linear() {
        // Integral of f(x) = x from 0 to 10 = 50
        let x: Vec<f64> = (0..=100).map(|i| i as f64 * 0.1).collect();
        let y: Vec<f64> = x.clone();
        let area = trapezoidal_integrate(&x, &y);
        assert!(approx_eq(area, 50.0, 0.01));
    }

    #[test]
    fn test_polyfit_linear() {
        let x: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let y: Vec<f64> = x.iter().map(|xi| 3.0 * xi + 1.0).collect();
        let coeffs = polyfit(&x, &y, 1);
        assert!(approx_eq(coeffs[0], 1.0, 0.01));
        assert!(approx_eq(coeffs[1], 3.0, 0.01));
    }
}
