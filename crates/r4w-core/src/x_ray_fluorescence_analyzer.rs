//! X-Ray Fluorescence (XRF) spectral analysis for elemental composition determination.
//!
//! This module implements energy-dispersive XRF spectroscopy processing for
//! non-destructive elemental analysis. It provides:
//!
//! - **Emission line database** with K-alpha, K-beta, L-alpha, L-beta lines for
//!   common elements (Ca, Ti, Cr, Mn, Fe, Co, Ni, Cu, Zn, As, Mo, Ag, Sn, Pb)
//! - **Moseley's law** for characteristic X-ray energy prediction
//! - **Peak detection** with SNR thresholding and Gaussian fitting
//! - **Background estimation** via SNIP algorithm and rolling ball
//! - **Energy calibration** (linear channel-to-energy mapping)
//! - **Quantitative analysis** using fundamental parameters, calibration curves,
//!   and internal standard ratio methods
//! - **Matrix effects** via mass attenuation and Beer-Lambert absorption
//! - **Detection limits** (MDL and LOQ) per element
//! - **Dead time correction** for high count rate operation
//! - **Spectrum simulation** from known element concentrations
//!
//! # Example
//!
//! ```
//! use r4w_core::x_ray_fluorescence_analyzer::{
//!     XrfSpectrum, XrfAnalyzer, XrfAnalyzerConfig, simulate_xrf_spectrum,
//! };
//!
//! let config = XrfAnalyzerConfig {
//!     energy_resolution_ev: 150.0,
//!     min_energy_kev: 0.5,
//!     max_energy_kev: 40.0,
//!     snr_threshold: 3.0,
//!     energy_tolerance_kev: 0.15,
//!     ..XrfAnalyzerConfig::default()
//! };
//!
//! // Simulate a spectrum with iron and copper
//! let concentrations = vec![(26, 0.50), (29, 0.30)]; // Fe 50%, Cu 30%
//! let spectrum = simulate_xrf_spectrum(&concentrations, 4096, 0.0, 40.0, 150.0, 100.0);
//!
//! let analyzer = XrfAnalyzer::new(config);
//! let results = analyzer.analyze(&spectrum);
//!
//! assert!(!results.identified_peaks.is_empty());
//! ```

use std::f64::consts::PI;

// ─── Physical constants ───────────────────────────────────────────────────────

const SQRT_2PI: f64 = 2.506_628_274_631_000_5;
const LN2: f64 = 0.693_147_180_559_945_3;
/// FWHM = 2*sqrt(2*ln2)*sigma ≈ 2.3548*sigma
const FWHM_TO_SIGMA: f64 = 0.424_660_900_144_009_5; // 1/(2*sqrt(2*ln2))

// ─── Element definitions ──────────────────────────────────────────────────────

/// A chemical element.
#[derive(Debug, Clone, PartialEq)]
pub struct Element {
    pub atomic_number: u32,
    pub symbol: &'static str,
    pub name: &'static str,
}

/// Type of characteristic X-ray emission line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LineType {
    /// K-alpha1: transition from L3 to K shell
    Ka,
    /// K-beta1: transition from M3 to K shell
    Kb,
    /// L-alpha1: transition from M5 to L3 shell
    La,
    /// L-beta1: transition from M4 to L2 shell
    Lb,
    /// L-gamma: transition from N to L shell
    Lg,
    /// M-alpha: transition from N to M shell
    Ma,
}

impl LineType {
    /// Return display name.
    pub fn name(&self) -> &'static str {
        match self {
            LineType::Ka => "Kα",
            LineType::Kb => "Kβ",
            LineType::La => "Lα",
            LineType::Lb => "Lβ",
            LineType::Lg => "Lγ",
            LineType::Ma => "Mα",
        }
    }
}

/// A characteristic X-ray emission line for an element.
#[derive(Debug, Clone)]
pub struct EmissionLine {
    pub element: Element,
    pub line_type: LineType,
    /// Characteristic energy in keV.
    pub energy_kev: f64,
    /// Relative intensity (Kα = 1.0 for K-series, Lα = 1.0 for L-series).
    pub relative_intensity: f64,
}

/// Build the emission line lookup table for common elements.
/// Energies are from standard NIST X-ray transition energies.
pub fn emission_line_database() -> Vec<EmissionLine> {
    let mut lines = Vec::new();

    // (Z, symbol, name, Ka_keV, Kb_keV, La_keV, Lb_keV)
    // Ka relative intensity = 1.0, Kb ~0.13-0.17 of Ka for transition metals
    let elements: &[(u32, &str, &str, f64, f64, Option<f64>, Option<f64>)] = &[
        (20, "Ca", "Calcium",     3.692, 4.013, None, None),
        (22, "Ti", "Titanium",    4.511, 4.932, None, None),
        (24, "Cr", "Chromium",    5.415, 5.947, None, None),
        (25, "Mn", "Manganese",   5.899, 6.490, None, None),
        (26, "Fe", "Iron",        6.404, 7.058, None, None),
        (27, "Co", "Cobalt",      6.930, 7.649, None, None),
        (28, "Ni", "Nickel",      7.478, 8.265, None, None),
        (29, "Cu", "Copper",      8.048, 8.905, None, None),
        (30, "Zn", "Zinc",        8.639, 9.572, None, None),
        (33, "As", "Arsenic",    10.544, 11.726, Some(1.282), Some(1.317)),
        (42, "Mo", "Molybdenum", 17.479, 19.608, Some(2.293), Some(2.395)),
        (47, "Ag", "Silver",     22.163, 24.942, Some(2.984), Some(3.151)),
        (50, "Sn", "Tin",        25.271, 28.486, Some(3.444), Some(3.663)),
        (56, "Ba", "Barium",     32.194, 36.378, Some(4.466), Some(4.828)),
        (82, "Pb", "Lead",       74.969, 84.938, Some(10.551), Some(12.614)),
    ];

    for &(z, sym, name, ka, kb, la, lb) in elements {
        let elem = Element {
            atomic_number: z,
            symbol: sym,
            name,
        };

        lines.push(EmissionLine {
            element: elem.clone(),
            line_type: LineType::Ka,
            energy_kev: ka,
            relative_intensity: 1.0,
        });

        // Kb relative intensity ~0.13 for first-row transition metals
        let kb_ratio = if z <= 30 { 0.13 } else { 0.15 };
        lines.push(EmissionLine {
            element: elem.clone(),
            line_type: LineType::Kb,
            energy_kev: kb,
            relative_intensity: kb_ratio,
        });

        if let Some(la_e) = la {
            lines.push(EmissionLine {
                element: elem.clone(),
                line_type: LineType::La,
                energy_kev: la_e,
                relative_intensity: 1.0,
            });
        }

        if let Some(lb_e) = lb {
            lines.push(EmissionLine {
                element: elem.clone(),
                line_type: LineType::Lb,
                energy_kev: lb_e,
                relative_intensity: 0.7,
            });
        }
    }

    lines
}

/// Moseley's law: predict characteristic X-ray energy.
///
/// `E = 13.6 * (Z - sigma)^2 * (1/n1^2 - 1/n2^2)` eV
///
/// - `z`: atomic number
/// - `sigma`: screening constant (1 for K-series, 7.4 for L-series typical)
/// - `n1`: lower shell principal quantum number (1 for K, 2 for L)
/// - `n2`: upper shell principal quantum number
///
/// Returns energy in keV.
pub fn moseley_energy_kev(z: u32, sigma: f64, n1: u32, n2: u32) -> f64 {
    let z_eff = z as f64 - sigma;
    let energy_ev = 13.6 * z_eff * z_eff * (1.0 / (n1 as f64).powi(2) - 1.0 / (n2 as f64).powi(2));
    energy_ev / 1000.0
}

/// Predict K-alpha energy using Moseley's law (sigma=1, n1=1, n2=2).
pub fn moseley_ka_kev(z: u32) -> f64 {
    moseley_energy_kev(z, 1.0, 1, 2)
}

/// Predict K-beta energy using Moseley's law (sigma=1, n1=1, n2=3).
pub fn moseley_kb_kev(z: u32) -> f64 {
    moseley_energy_kev(z, 1.0, 1, 3)
}

// ─── XRF Spectrum ─────────────────────────────────────────────────────────────

/// An XRF spectrum: histogram of photon counts vs energy.
#[derive(Debug, Clone)]
pub struct XrfSpectrum {
    /// Energy axis in keV (one per channel).
    pub energy_kev: Vec<f64>,
    /// Photon counts per channel.
    pub counts: Vec<f64>,
    /// Live time (detector active time) in seconds.
    pub live_time_s: f64,
    /// Real (wall-clock) time in seconds.
    pub real_time_s: f64,
}

impl XrfSpectrum {
    /// Create a new spectrum with given energy and counts arrays.
    pub fn new(energy_kev: Vec<f64>, counts: Vec<f64>, live_time_s: f64, real_time_s: f64) -> Self {
        assert_eq!(energy_kev.len(), counts.len(), "energy and counts must have same length");
        Self { energy_kev, counts, live_time_s, real_time_s }
    }

    /// Number of channels.
    pub fn num_channels(&self) -> usize {
        self.energy_kev.len()
    }

    /// Total counts across all channels.
    pub fn total_counts(&self) -> f64 {
        self.counts.iter().sum()
    }

    /// Dead time fraction: 1 - live_time/real_time.
    pub fn dead_time_fraction(&self) -> f64 {
        if self.real_time_s <= 0.0 {
            return 0.0;
        }
        1.0 - self.live_time_s / self.real_time_s
    }

    /// Find channel index closest to a given energy.
    pub fn channel_at_energy(&self, energy_kev: f64) -> usize {
        let mut best_idx = 0;
        let mut best_dist = f64::MAX;
        for (i, &e) in self.energy_kev.iter().enumerate() {
            let d = (e - energy_kev).abs();
            if d < best_dist {
                best_dist = d;
                best_idx = i;
            }
        }
        best_idx
    }
}

// ─── Energy calibration ───────────────────────────────────────────────────────

/// Linear energy calibration: E(keV) = offset + gain * channel.
#[derive(Debug, Clone, Copy)]
pub struct EnergyCalibration {
    pub offset_kev: f64,
    pub gain_kev_per_channel: f64,
}

impl EnergyCalibration {
    /// Create calibration from two known energy-channel pairs via linear fit.
    pub fn from_two_points(ch1: f64, e1_kev: f64, ch2: f64, e2_kev: f64) -> Self {
        let gain = (e2_kev - e1_kev) / (ch2 - ch1);
        let offset = e1_kev - gain * ch1;
        Self { offset_kev: offset, gain_kev_per_channel: gain }
    }

    /// Convert channel to energy.
    pub fn channel_to_energy(&self, channel: f64) -> f64 {
        self.offset_kev + self.gain_kev_per_channel * channel
    }

    /// Convert energy to channel.
    pub fn energy_to_channel(&self, energy_kev: f64) -> f64 {
        (energy_kev - self.offset_kev) / self.gain_kev_per_channel
    }

    /// Apply calibration to a spectrum, updating energy axis.
    pub fn apply(&self, spectrum: &mut XrfSpectrum) {
        for (i, e) in spectrum.energy_kev.iter_mut().enumerate() {
            *e = self.channel_to_energy(i as f64);
        }
    }
}

// ─── Background estimation ────────────────────────────────────────────────────

/// SNIP (Statistics-sensitive Non-linear Iterative Peak-clipping) algorithm.
///
/// Iteratively clips peaks by replacing each point with the minimum of itself
/// and the average of its neighbors at distance `p` (the iteration window).
/// Returns the estimated background.
pub fn snip_background(counts: &[f64], iterations: usize) -> Vec<f64> {
    let n = counts.len();
    if n == 0 {
        return vec![];
    }

    // Work in sqrt-space for better Poisson statistics handling
    let mut v: Vec<f64> = counts.iter().map(|&c| c.max(0.0).sqrt()).collect();

    for p in (1..=iterations).rev() {
        let prev = v.clone();
        for i in p..n.saturating_sub(p) {
            let avg = (prev[i - p] + prev[i + p]) / 2.0;
            v[i] = v[i].min(avg);
        }
    }

    // Convert back from sqrt-space
    v.iter().map(|&x| x * x).collect()
}

/// Rolling ball background estimation.
///
/// Uses a ball of given `radius` (in channels) rolling under the spectrum.
/// Returns the estimated background.
pub fn rolling_ball_background(counts: &[f64], radius: usize) -> Vec<f64> {
    let n = counts.len();
    if n == 0 {
        return vec![];
    }

    let mut background = vec![0.0; n];

    for i in 0..n {
        let start = if i >= radius { i - radius } else { 0 };
        let end = (i + radius + 1).min(n);

        // Find minimum in the window
        let mut min_val = f64::MAX;
        for j in start..end {
            if counts[j] < min_val {
                min_val = counts[j];
            }
        }
        background[i] = min_val;
    }

    // Smooth the background with a moving average
    let smooth_radius = radius / 2;
    if smooth_radius == 0 {
        return background;
    }

    let mut smoothed = vec![0.0; n];
    for i in 0..n {
        let start = if i >= smooth_radius { i - smooth_radius } else { 0 };
        let end = (i + smooth_radius + 1).min(n);
        let sum: f64 = background[start..end].iter().sum();
        smoothed[i] = sum / (end - start) as f64;
    }

    smoothed
}

/// Linear interpolation background between specified anchor windows.
///
/// `anchors` are (center_channel, half_width) pairs defining background windows.
/// The background is linearly interpolated between window averages.
pub fn interpolated_background(counts: &[f64], anchors: &[(usize, usize)]) -> Vec<f64> {
    let n = counts.len();
    if n == 0 || anchors.is_empty() {
        return vec![0.0; n];
    }

    // Compute average counts in each anchor window
    let mut anchor_points: Vec<(usize, f64)> = Vec::new();
    for &(center, half_w) in anchors {
        let start = center.saturating_sub(half_w);
        let end = (center + half_w + 1).min(n);
        let sum: f64 = counts[start..end].iter().sum();
        let avg = sum / (end - start) as f64;
        anchor_points.push((center, avg));
    }

    anchor_points.sort_by_key(|&(ch, _)| ch);

    let mut background = vec![0.0; n];

    // Extrapolate before first anchor
    if let Some(&(ch0, val0)) = anchor_points.first() {
        for i in 0..=ch0.min(n - 1) {
            background[i] = val0;
        }
    }

    // Interpolate between anchors
    for w in anchor_points.windows(2) {
        let (ch1, v1) = w[0];
        let (ch2, v2) = w[1];
        if ch2 > ch1 {
            for i in ch1..=ch2.min(n - 1) {
                let t = (i - ch1) as f64 / (ch2 - ch1) as f64;
                background[i] = v1 + t * (v2 - v1);
            }
        }
    }

    // Extrapolate after last anchor
    if let Some(&(ch_last, val_last)) = anchor_points.last() {
        for i in ch_last..n {
            background[i] = val_last;
        }
    }

    background
}

// ─── Peak detection and fitting ───────────────────────────────────────────────

/// A detected peak in the spectrum.
#[derive(Debug, Clone)]
pub struct DetectedPeak {
    /// Channel index of the peak center.
    pub channel: usize,
    /// Energy of the peak center in keV.
    pub energy_kev: f64,
    /// Peak amplitude (counts above background).
    pub amplitude: f64,
    /// Gaussian sigma in keV.
    pub sigma_kev: f64,
    /// Signal-to-noise ratio.
    pub snr: f64,
    /// Gross area (total counts in peak region).
    pub gross_area: f64,
    /// Net area (gross minus background).
    pub net_area: f64,
    /// Background under the peak region.
    pub background_area: f64,
}

/// An identified peak matched to an emission line.
#[derive(Debug, Clone)]
pub struct IdentifiedPeak {
    pub peak: DetectedPeak,
    pub emission_line: EmissionLine,
    /// Energy difference between detected and reference (keV).
    pub energy_offset_kev: f64,
}

/// Find local maxima above background with SNR threshold.
pub fn find_peaks(
    energy_kev: &[f64],
    counts: &[f64],
    background: &[f64],
    snr_threshold: f64,
    min_separation_channels: usize,
) -> Vec<DetectedPeak> {
    let n = counts.len();
    if n < 3 {
        return vec![];
    }

    let min_sep = min_separation_channels.max(1);
    let mut peaks = Vec::new();

    for i in 1..n - 1 {
        // Local maximum check
        if counts[i] <= counts[i - 1] || counts[i] <= counts[i + 1] {
            continue;
        }

        let net = counts[i] - background[i];
        if net <= 0.0 {
            continue;
        }

        let noise = background[i].max(1.0).sqrt();
        let snr = net / noise;
        if snr < snr_threshold {
            continue;
        }

        // Estimate sigma from FWHM by looking at half-maximum points
        let half_max = background[i] + net / 2.0;
        let mut left = i;
        while left > 0 && counts[left] > half_max {
            left -= 1;
        }
        let mut right = i;
        while right < n - 1 && counts[right] > half_max {
            right += 1;
        }

        let fwhm_channels = (right - left) as f64;
        let channel_width = if i > 0 && i < n - 1 {
            (energy_kev[i + 1] - energy_kev[i - 1]) / 2.0
        } else if n > 1 {
            energy_kev[1] - energy_kev[0]
        } else {
            1.0
        };
        let sigma_kev = fwhm_channels * channel_width * FWHM_TO_SIGMA;

        // Peak area: sum counts in +/- 3 sigma region
        let half_width = (3.0 * fwhm_channels * FWHM_TO_SIGMA).ceil() as usize;
        let half_width = half_width.max(2);
        let start = i.saturating_sub(half_width);
        let end = (i + half_width + 1).min(n);

        let gross_area: f64 = counts[start..end].iter().sum();
        let bg_area: f64 = background[start..end].iter().sum();
        let net_area = (gross_area - bg_area).max(0.0);

        peaks.push(DetectedPeak {
            channel: i,
            energy_kev: energy_kev[i],
            amplitude: net,
            sigma_kev: sigma_kev.max(channel_width * 0.5),
            snr,
            gross_area,
            net_area,
            background_area: bg_area,
        });
    }

    // Remove peaks that are too close (keep higher SNR)
    peaks.sort_by(|a, b| b.snr.partial_cmp(&a.snr).unwrap_or(std::cmp::Ordering::Equal));
    let mut filtered = Vec::new();
    for p in &peaks {
        let too_close = filtered.iter().any(|q: &DetectedPeak| {
            (p.channel as isize - q.channel as isize).unsigned_abs() < min_sep
        });
        if !too_close {
            filtered.push(p.clone());
        }
    }

    filtered.sort_by_key(|p| p.channel);
    filtered
}

/// Match detected peaks to the emission line database by energy proximity.
pub fn identify_peaks(
    peaks: &[DetectedPeak],
    database: &[EmissionLine],
    energy_tolerance_kev: f64,
) -> Vec<IdentifiedPeak> {
    let mut identified = Vec::new();

    for peak in peaks {
        let mut best_match: Option<(&EmissionLine, f64)> = None;

        for line in database {
            let offset = (peak.energy_kev - line.energy_kev).abs();
            if offset <= energy_tolerance_kev {
                if best_match.is_none() || offset < best_match.unwrap().1 {
                    best_match = Some((line, offset));
                }
            }
        }

        if let Some((line, offset)) = best_match {
            identified.push(IdentifiedPeak {
                peak: peak.clone(),
                emission_line: line.clone(),
                energy_offset_kev: peak.energy_kev - line.energy_kev,
            });
        }
    }

    identified
}

/// Gaussian peak fit: estimate amplitude, center, and sigma by parabolic interpolation
/// on log-counts around a peak channel.
///
/// Returns (amplitude, center_channel, sigma_channels).
pub fn gaussian_fit_peak(counts: &[f64], background: &[f64], peak_channel: usize) -> (f64, f64, f64) {
    let n = counts.len();
    if peak_channel == 0 || peak_channel >= n - 1 {
        return (counts.get(peak_channel).copied().unwrap_or(0.0), peak_channel as f64, 1.0);
    }

    // Use 3-point parabolic interpolation on log(net_counts)
    let y0 = (counts[peak_channel - 1] - background[peak_channel - 1]).max(1.0).ln();
    let y1 = (counts[peak_channel] - background[peak_channel]).max(1.0).ln();
    let y2 = (counts[peak_channel + 1] - background[peak_channel + 1]).max(1.0).ln();

    // Parabolic vertex: x_center = peak_channel + delta
    let denom = 2.0 * (2.0 * y1 - y0 - y2);
    let delta = if denom.abs() > 1e-12 {
        (y0 - y2) / denom
    } else {
        0.0
    };
    let center = peak_channel as f64 + delta.clamp(-0.5, 0.5);

    // Sigma from curvature: sigma^2 = -1 / (d^2/dx^2 ln(f))
    let d2 = y0 - 2.0 * y1 + y2;
    let sigma = if d2 < -1e-12 {
        (-1.0 / d2).sqrt()
    } else {
        2.0 // fallback
    };

    let amplitude = (counts[peak_channel] - background[peak_channel]).max(0.0);

    (amplitude, center, sigma)
}

/// Evaluate Gaussian: A * exp(-(x - x0)^2 / (2*sigma^2)).
pub fn gaussian(x: f64, amplitude: f64, center: f64, sigma: f64) -> f64 {
    let dx = x - center;
    amplitude * (-dx * dx / (2.0 * sigma * sigma)).exp()
}

// ─── Quantitative analysis ────────────────────────────────────────────────────

/// Result of quantitative analysis for one element.
#[derive(Debug, Clone)]
pub struct ElementQuantResult {
    pub element: Element,
    pub line_type: LineType,
    pub energy_kev: f64,
    pub net_area: f64,
    pub concentration_ppm: f64,
    pub mdl_ppm: f64,
    pub loq_ppm: f64,
}

/// Calibration point: known concentration vs measured peak area.
#[derive(Debug, Clone, Copy)]
pub struct CalibrationPoint {
    pub concentration_ppm: f64,
    pub net_peak_area: f64,
}

/// Simple linear regression: y = a + b*x.
/// Returns (intercept, slope, r_squared).
pub fn linear_regression(points: &[CalibrationPoint]) -> (f64, f64, f64) {
    let n = points.len() as f64;
    if points.len() < 2 {
        return (0.0, 1.0, 0.0);
    }

    let sum_x: f64 = points.iter().map(|p| p.net_peak_area).sum();
    let sum_y: f64 = points.iter().map(|p| p.concentration_ppm).sum();
    let sum_xx: f64 = points.iter().map(|p| p.net_peak_area * p.net_peak_area).sum();
    let sum_xy: f64 = points.iter().map(|p| p.net_peak_area * p.concentration_ppm).sum();

    let denom = n * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return (sum_y / n, 0.0, 0.0);
    }

    let slope = (n * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / n;

    // R-squared
    let y_mean = sum_y / n;
    let ss_tot: f64 = points.iter().map(|p| (p.concentration_ppm - y_mean).powi(2)).sum();
    let ss_res: f64 = points.iter().map(|p| {
        let predicted = intercept + slope * p.net_peak_area;
        (p.concentration_ppm - predicted).powi(2)
    }).sum();

    let r_squared = if ss_tot > 1e-30 { 1.0 - ss_res / ss_tot } else { 0.0 };

    (intercept, slope, r_squared)
}

/// Calibration curve method: predict concentration from net peak area.
pub fn calibration_curve_concentration(
    net_area: f64,
    cal_points: &[CalibrationPoint],
) -> f64 {
    let (intercept, slope, _) = linear_regression(cal_points);
    intercept + slope * net_area
}

/// Internal standard ratio method.
///
/// `area_analyte / area_standard * conc_standard * k_factor`
pub fn internal_standard_concentration(
    analyte_net_area: f64,
    standard_net_area: f64,
    standard_concentration_ppm: f64,
    k_factor: f64,
) -> f64 {
    if standard_net_area <= 0.0 {
        return 0.0;
    }
    analyte_net_area / standard_net_area * standard_concentration_ppm * k_factor
}

/// Simplified fundamental parameters: intensity proportional to
/// concentration × fluorescence_yield × absorption_correction.
///
/// Returns estimated concentration in ppm.
pub fn fundamental_parameters_concentration(
    net_area: f64,
    live_time_s: f64,
    fluorescence_yield: f64,
    absorption_correction: f64,
    sensitivity: f64, // counts/s per ppm at unit yield and correction
) -> f64 {
    if live_time_s <= 0.0 || fluorescence_yield <= 0.0 || sensitivity <= 0.0 {
        return 0.0;
    }
    let count_rate = net_area / live_time_s;
    count_rate / (sensitivity * fluorescence_yield * absorption_correction) * 1e6
}

// ─── Detection limits ─────────────────────────────────────────────────────────

/// Minimum Detectable Limit: MDL = 3 * sqrt(N_background) * C / N_net
/// where C is the known concentration and N_net is net counts.
pub fn detection_limit_mdl(background_counts: f64, net_counts: f64, concentration_ppm: f64) -> f64 {
    if net_counts <= 0.0 {
        return f64::INFINITY;
    }
    3.0 * background_counts.max(0.0).sqrt() * concentration_ppm / net_counts
}

/// Limit of Quantification: LOQ = 10 * sqrt(N_background) * C / N_net
pub fn detection_limit_loq(background_counts: f64, net_counts: f64, concentration_ppm: f64) -> f64 {
    if net_counts <= 0.0 {
        return f64::INFINITY;
    }
    10.0 * background_counts.max(0.0).sqrt() * concentration_ppm / net_counts
}

// ─── Dead time correction ─────────────────────────────────────────────────────

/// Dead time correction for paralyzable detector model.
///
/// `N_true = N_measured / (1 - N_measured * tau)`
///
/// - `n_measured`: measured count rate (counts/s)
/// - `tau`: dead time per event (seconds), typically 0.1-10 μs
///
/// Returns corrected count rate. Returns None if measured rate exceeds 1/tau.
pub fn dead_time_correction(n_measured: f64, tau: f64) -> Option<f64> {
    let denom = 1.0 - n_measured * tau;
    if denom <= 0.0 {
        return None; // saturated
    }
    Some(n_measured / denom)
}

/// Apply dead time correction to an entire spectrum (scales counts).
pub fn apply_dead_time_correction(spectrum: &mut XrfSpectrum, tau: f64) {
    if spectrum.live_time_s <= 0.0 {
        return;
    }
    let total_rate = spectrum.total_counts() / spectrum.live_time_s;
    if let Some(corrected_rate) = dead_time_correction(total_rate, tau) {
        let scale = corrected_rate / total_rate;
        for c in spectrum.counts.iter_mut() {
            *c *= scale;
        }
    }
}

// ─── Matrix effects ───────────────────────────────────────────────────────────

/// Approximate mass attenuation coefficient (cm²/g) for a given element at a given energy.
///
/// Uses simplified power-law: mu/rho ~ K * Z^4 / E^3 (rough approximation valid
/// away from absorption edges).
///
/// - `z`: absorber atomic number
/// - `energy_kev`: photon energy
///
/// Returns mu/rho in cm²/g.
pub fn mass_attenuation_approx(z: u32, energy_kev: f64) -> f64 {
    if energy_kev <= 0.0 {
        return 0.0;
    }
    // Empirical approximation: mu/rho ~ C * Z^3.5 / E^2.8
    // Normalized so Fe at 6.4 keV gives ~100 cm²/g
    let z_f = z as f64;
    let c = 100.0 / (26.0_f64.powf(3.5) / 6.4_f64.powf(2.8));
    c * z_f.powf(3.5) / energy_kev.powf(2.8)
}

/// Beer-Lambert absorption: I = I0 * exp(-mu_rho * rho * t)
///
/// - `i0`: incident intensity
/// - `mu_rho`: mass attenuation coefficient (cm²/g)
/// - `rho`: density (g/cm³)
/// - `thickness_cm`: material thickness (cm)
pub fn beer_lambert(i0: f64, mu_rho: f64, rho: f64, thickness_cm: f64) -> f64 {
    i0 * (-mu_rho * rho * thickness_cm).exp()
}

/// Enhancement (secondary fluorescence) correction factor.
///
/// When element j emits X-rays that excite element i, the effective intensity
/// of element i is enhanced. This returns a multiplicative factor > 1.
///
/// Simplified model:
/// `enhancement = 1 + sum_j( w_j * yield_j * (mu_j_at_Ei / mu_total_at_Ej) * geometry )`
///
/// Here we use a simplified single-enhancer model.
pub fn enhancement_factor(
    analyte_energy_kev: f64,
    enhancer_energy_kev: f64,
    enhancer_weight_fraction: f64,
    enhancer_yield: f64,
) -> f64 {
    // Enhancement only if enhancer energy > analyte absorption edge
    if enhancer_energy_kev <= analyte_energy_kev {
        return 1.0;
    }

    // Geometry factor (half-space, simplified)
    let geometry = 0.5;

    // Simplified jump ratio contribution
    let jump = (enhancer_energy_kev / analyte_energy_kev).ln();

    1.0 + enhancer_weight_fraction * enhancer_yield * jump * geometry
}

/// K-shell fluorescence yield approximation (Bambynek et al.).
/// `omega_K ~ Z^4 / (A + Z^4)` where A ~ 10^6.
pub fn fluorescence_yield_k(z: u32) -> f64 {
    let z4 = (z as f64).powi(4);
    z4 / (1.0e6 + z4)
}

// ─── XRF Analyzer ─────────────────────────────────────────────────────────────

/// Configuration for the XRF analyzer.
#[derive(Debug, Clone)]
pub struct XrfAnalyzerConfig {
    /// Detector energy resolution in eV (FWHM at 5.9 keV Mn Kα).
    /// SDD: 125-150 eV, Si-PIN: 180-250 eV, proportional counter: 600-1000 eV.
    pub energy_resolution_ev: f64,
    /// Minimum energy to analyze (keV).
    pub min_energy_kev: f64,
    /// Maximum energy to analyze (keV).
    pub max_energy_kev: f64,
    /// SNR threshold for peak detection.
    pub snr_threshold: f64,
    /// Energy tolerance for peak-to-line matching (keV).
    pub energy_tolerance_kev: f64,
    /// Number of SNIP iterations for background estimation.
    pub snip_iterations: usize,
    /// Minimum peak separation in channels.
    pub min_peak_separation: usize,
}

impl Default for XrfAnalyzerConfig {
    fn default() -> Self {
        Self {
            energy_resolution_ev: 150.0,
            min_energy_kev: 0.5,
            max_energy_kev: 40.0,
            snr_threshold: 3.0,
            energy_tolerance_kev: 0.15,
            snip_iterations: 24,
            min_peak_separation: 5,
        }
    }
}

/// Results from XRF spectrum analysis.
#[derive(Debug, Clone)]
pub struct XrfAnalysisResult {
    /// Background estimate.
    pub background: Vec<f64>,
    /// All detected peaks.
    pub detected_peaks: Vec<DetectedPeak>,
    /// Peaks matched to emission lines.
    pub identified_peaks: Vec<IdentifiedPeak>,
    /// Quantitative results per identified element.
    pub quant_results: Vec<ElementQuantResult>,
    /// Total spectrum counts.
    pub total_counts: f64,
    /// Dead time fraction.
    pub dead_time_fraction: f64,
}

/// XRF spectrum analyzer.
#[derive(Debug, Clone)]
pub struct XrfAnalyzer {
    pub config: XrfAnalyzerConfig,
    pub database: Vec<EmissionLine>,
}

impl XrfAnalyzer {
    /// Create a new analyzer with the given configuration.
    pub fn new(config: XrfAnalyzerConfig) -> Self {
        let database = emission_line_database();
        Self { config, database }
    }

    /// Create with default configuration.
    pub fn default_analyzer() -> Self {
        Self::new(XrfAnalyzerConfig::default())
    }

    /// Analyze an XRF spectrum: detect peaks, identify elements, estimate concentrations.
    pub fn analyze(&self, spectrum: &XrfSpectrum) -> XrfAnalysisResult {
        // 1. Background estimation
        let background = snip_background(&spectrum.counts, self.config.snip_iterations);

        // 2. Peak detection
        let detected_peaks = find_peaks(
            &spectrum.energy_kev,
            &spectrum.counts,
            &background,
            self.config.snr_threshold,
            self.config.min_peak_separation,
        );

        // 3. Filter peaks to energy range and identify
        let filtered_peaks: Vec<DetectedPeak> = detected_peaks
            .into_iter()
            .filter(|p| p.energy_kev >= self.config.min_energy_kev && p.energy_kev <= self.config.max_energy_kev)
            .collect();

        let identified_peaks = identify_peaks(
            &filtered_peaks,
            &self.database,
            self.config.energy_tolerance_kev,
        );

        // 4. Quantitative analysis (simplified: concentration proportional to net area)
        let total_net: f64 = identified_peaks.iter().map(|ip| ip.peak.net_area).sum();
        let quant_results: Vec<ElementQuantResult> = identified_peaks
            .iter()
            .map(|ip| {
                let conc_ppm = if total_net > 0.0 {
                    ip.peak.net_area / total_net * 1_000_000.0
                } else {
                    0.0
                };
                let mdl = detection_limit_mdl(ip.peak.background_area, ip.peak.net_area, conc_ppm);
                let loq = detection_limit_loq(ip.peak.background_area, ip.peak.net_area, conc_ppm);

                ElementQuantResult {
                    element: ip.emission_line.element.clone(),
                    line_type: ip.emission_line.line_type,
                    energy_kev: ip.peak.energy_kev,
                    net_area: ip.peak.net_area,
                    concentration_ppm: conc_ppm,
                    mdl_ppm: mdl,
                    loq_ppm: loq,
                }
            })
            .collect();

        XrfAnalysisResult {
            background,
            detected_peaks: filtered_peaks,
            identified_peaks,
            quant_results,
            total_counts: spectrum.total_counts(),
            dead_time_fraction: spectrum.dead_time_fraction(),
        }
    }
}

// ─── Spectrum simulation ──────────────────────────────────────────────────────

/// Generate a synthetic XRF spectrum from element concentrations.
///
/// - `concentrations`: slice of (atomic_number, weight_fraction) pairs
/// - `num_channels`: number of spectrum channels
/// - `min_energy_kev`, `max_energy_kev`: energy range
/// - `resolution_ev`: detector resolution (FWHM in eV)
/// - `live_time_s`: measurement live time
///
/// Returns a simulated XrfSpectrum with Gaussian peaks on a Bremsstrahlung background.
pub fn simulate_xrf_spectrum(
    concentrations: &[(u32, f64)],
    num_channels: usize,
    min_energy_kev: f64,
    max_energy_kev: f64,
    resolution_ev: f64,
    live_time_s: f64,
) -> XrfSpectrum {
    let channel_width = (max_energy_kev - min_energy_kev) / num_channels as f64;
    let energy_kev: Vec<f64> = (0..num_channels)
        .map(|i| min_energy_kev + (i as f64 + 0.5) * channel_width)
        .collect();

    // Start with Bremsstrahlung background: proportional to 1/E
    let background_scale = 100.0 * live_time_s;
    let mut counts: Vec<f64> = energy_kev
        .iter()
        .map(|&e| {
            if e > 0.1 {
                background_scale / e
            } else {
                background_scale / 0.1
            }
        })
        .collect();

    let database = emission_line_database();
    let sigma_kev = resolution_ev / 1000.0 * FWHM_TO_SIGMA;

    // Add Gaussian peaks for each element
    for &(z, weight_frac) in concentrations {
        let yield_k = fluorescence_yield_k(z);

        for line in &database {
            if line.element.atomic_number != z {
                continue;
            }
            if line.energy_kev < min_energy_kev || line.energy_kev > max_energy_kev {
                continue;
            }

            // Peak intensity: proportional to concentration, yield, and relative intensity
            let peak_counts = weight_frac * yield_k * line.relative_intensity * 10000.0 * live_time_s;

            // Energy-dependent sigma: resolution scales as sqrt(E)
            let sigma = sigma_kev * (line.energy_kev / 5.9).sqrt();

            for (i, &e) in energy_kev.iter().enumerate() {
                counts[i] += gaussian(e, peak_counts, line.energy_kev, sigma);
            }
        }
    }

    // Add Poisson noise approximation: sqrt(counts) variation
    // Using a simple deterministic "noise" for reproducibility
    for i in 0..num_channels {
        let noise_amp = counts[i].max(0.0).sqrt();
        // Deterministic pseudo-noise based on channel index
        let phase = (i as f64 * 2.0 * PI / 17.3).sin() * 0.3
            + (i as f64 * 2.0 * PI / 7.1).sin() * 0.2;
        counts[i] = (counts[i] + noise_amp * phase).max(0.0);
    }

    XrfSpectrum {
        energy_kev,
        counts,
        live_time_s,
        real_time_s: live_time_s * 1.05, // 5% dead time
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    // --- Element and EmissionLine tests ---

    #[test]
    fn test_emission_line_database_not_empty() {
        let db = emission_line_database();
        assert!(db.len() >= 30, "Database should have at least 30 lines, got {}", db.len());
    }

    #[test]
    fn test_emission_line_database_has_iron() {
        let db = emission_line_database();
        let fe_ka = db.iter().find(|l| l.element.symbol == "Fe" && l.line_type == LineType::Ka);
        assert!(fe_ka.is_some(), "Database should contain Fe Kα");
        let fe_ka = fe_ka.unwrap();
        assert!((fe_ka.energy_kev - 6.404).abs() < 0.01, "Fe Kα should be ~6.404 keV");
        assert_eq!(fe_ka.relative_intensity, 1.0);
    }

    #[test]
    fn test_emission_line_database_has_copper() {
        let db = emission_line_database();
        let cu_ka = db.iter().find(|l| l.element.symbol == "Cu" && l.line_type == LineType::Ka);
        assert!(cu_ka.is_some());
        assert!((cu_ka.unwrap().energy_kev - 8.048).abs() < 0.01);
    }

    #[test]
    fn test_emission_line_kb_lower_intensity() {
        let db = emission_line_database();
        let fe_kb = db.iter().find(|l| l.element.symbol == "Fe" && l.line_type == LineType::Kb);
        assert!(fe_kb.is_some());
        assert!(fe_kb.unwrap().relative_intensity < 0.2, "Kβ should be much weaker than Kα");
    }

    #[test]
    fn test_emission_line_lead_has_l_lines() {
        let db = emission_line_database();
        let pb_la = db.iter().find(|l| l.element.symbol == "Pb" && l.line_type == LineType::La);
        assert!(pb_la.is_some(), "Pb should have Lα line");
        assert!((pb_la.unwrap().energy_kev - 10.551).abs() < 0.01);
    }

    #[test]
    fn test_line_type_name() {
        assert_eq!(LineType::Ka.name(), "Kα");
        assert_eq!(LineType::Kb.name(), "Kβ");
        assert_eq!(LineType::La.name(), "Lα");
        assert_eq!(LineType::Lb.name(), "Lβ");
        assert_eq!(LineType::Lg.name(), "Lγ");
        assert_eq!(LineType::Ma.name(), "Mα");
    }

    // --- Moseley's law tests ---

    #[test]
    fn test_moseley_ka_increases_with_z() {
        let e_fe = moseley_ka_kev(26);
        let e_cu = moseley_ka_kev(29);
        let e_zn = moseley_ka_kev(30);
        assert!(e_fe < e_cu, "Fe Kα < Cu Kα");
        assert!(e_cu < e_zn, "Cu Kα < Zn Kα");
    }

    #[test]
    fn test_moseley_ka_roughly_correct() {
        // Moseley's law is approximate; check it gives the right ballpark
        let e_fe = moseley_ka_kev(26);
        // Actual Fe Kα = 6.404 keV, Moseley gives ~6.38 keV
        assert!((e_fe - 6.4).abs() < 0.5, "Moseley Fe Kα should be ~6.4 keV, got {}", e_fe);
    }

    #[test]
    fn test_moseley_kb_higher_than_ka() {
        let ka = moseley_ka_kev(26);
        let kb = moseley_kb_kev(26);
        assert!(kb > ka, "Kβ energy should be higher than Kα");
    }

    #[test]
    fn test_moseley_energy_kev_general() {
        // L-alpha: sigma~7.4, n1=2, n2=3
        let e = moseley_energy_kev(82, 7.4, 2, 3);
        assert!(e > 0.0, "L-alpha energy for Pb should be positive");
    }

    // --- XrfSpectrum tests ---

    #[test]
    fn test_spectrum_creation() {
        let energy = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let counts = vec![10.0, 20.0, 50.0, 20.0, 10.0];
        let spec = XrfSpectrum::new(energy, counts, 100.0, 105.0);
        assert_eq!(spec.num_channels(), 5);
        assert!((spec.total_counts() - 110.0).abs() < EPSILON);
    }

    #[test]
    fn test_spectrum_dead_time_fraction() {
        let spec = XrfSpectrum::new(vec![1.0], vec![100.0], 90.0, 100.0);
        assert!((spec.dead_time_fraction() - 0.1).abs() < EPSILON);
    }

    #[test]
    fn test_spectrum_dead_time_zero_real_time() {
        let spec = XrfSpectrum::new(vec![1.0], vec![100.0], 90.0, 0.0);
        assert!((spec.dead_time_fraction() - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_spectrum_channel_at_energy() {
        let energy = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let counts = vec![0.0; 5];
        let spec = XrfSpectrum::new(energy, counts, 100.0, 100.0);
        assert_eq!(spec.channel_at_energy(3.1), 2);
        assert_eq!(spec.channel_at_energy(1.0), 0);
        assert_eq!(spec.channel_at_energy(5.0), 4);
    }

    // --- Energy calibration tests ---

    #[test]
    fn test_energy_calibration_two_points() {
        let cal = EnergyCalibration::from_two_points(100.0, 5.9, 200.0, 10.0);
        assert!((cal.channel_to_energy(100.0) - 5.9).abs() < EPSILON);
        assert!((cal.channel_to_energy(200.0) - 10.0).abs() < EPSILON);
    }

    #[test]
    fn test_energy_calibration_roundtrip() {
        let cal = EnergyCalibration::from_two_points(0.0, 0.0, 4096.0, 40.0);
        let ch = 2048.0;
        let e = cal.channel_to_energy(ch);
        let ch2 = cal.energy_to_channel(e);
        assert!((ch - ch2).abs() < EPSILON);
    }

    #[test]
    fn test_energy_calibration_apply() {
        let cal = EnergyCalibration::from_two_points(0.0, 1.0, 100.0, 11.0);
        let mut spec = XrfSpectrum::new(vec![0.0; 5], vec![0.0; 5], 1.0, 1.0);
        cal.apply(&mut spec);
        assert!((spec.energy_kev[0] - 1.0).abs() < EPSILON);
        // channel 4 => 1.0 + 0.1*4 = 1.4
        assert!((spec.energy_kev[4] - 1.4).abs() < EPSILON);
    }

    // --- Background estimation tests ---

    #[test]
    fn test_snip_background_flat() {
        let counts = vec![100.0; 50];
        let bg = snip_background(&counts, 10);
        assert_eq!(bg.len(), 50);
        // Flat spectrum: background should be close to counts
        for &b in &bg[10..40] {
            assert!((b - 100.0).abs() < 1.0, "SNIP of flat spectrum should be ~100, got {}", b);
        }
    }

    #[test]
    fn test_snip_background_removes_peak() {
        let mut counts = vec![10.0; 100];
        // Add a peak at channel 50
        for i in 45..56 {
            counts[i] = 10.0 + 90.0 * gaussian(i as f64, 1.0, 50.0, 3.0);
        }
        let bg = snip_background(&counts, 20);
        // Background under peak should be less than the peak
        assert!(bg[50] < counts[50] * 0.5, "SNIP should clip the peak");
        // Background far from peak should be close to baseline
        assert!((bg[10] - 10.0).abs() < 5.0);
    }

    #[test]
    fn test_snip_background_empty() {
        let bg = snip_background(&[], 10);
        assert!(bg.is_empty());
    }

    #[test]
    fn test_rolling_ball_background() {
        let mut counts = vec![10.0; 100];
        counts[50] = 100.0;
        let bg = rolling_ball_background(&counts, 10);
        assert_eq!(bg.len(), 100);
        // Background should be smooth and below the peak
        assert!(bg[50] < 100.0);
        assert!(bg[20] <= 10.0 + 1.0);
    }

    #[test]
    fn test_interpolated_background() {
        let counts = vec![10.0; 100];
        let anchors = vec![(20, 5), (80, 5)];
        let bg = interpolated_background(&counts, &anchors);
        assert_eq!(bg.len(), 100);
        // Should be ~10 everywhere for a flat spectrum
        for &b in &bg[20..81] {
            assert!((b - 10.0).abs() < 0.5);
        }
    }

    // --- Peak detection tests ---

    #[test]
    fn test_find_peaks_single_peak() {
        let n = 200;
        let energy: Vec<f64> = (0..n).map(|i| i as f64 * 0.1).collect();
        let background = vec![10.0; n];
        let mut counts = background.clone();
        // Add Gaussian peak at channel 100 (energy 10.0 keV)
        for i in 0..n {
            counts[i] += gaussian(i as f64, 200.0, 100.0, 5.0);
        }
        let peaks = find_peaks(&energy, &counts, &background, 3.0, 5);
        assert!(!peaks.is_empty(), "Should detect at least one peak");
        // Peak should be near channel 100
        let p = &peaks[0];
        assert!((p.channel as f64 - 100.0).abs() < 3.0, "Peak at wrong channel: {}", p.channel);
        assert!(p.snr > 3.0);
        assert!(p.net_area > 0.0);
    }

    #[test]
    fn test_find_peaks_no_peaks_below_threshold() {
        let n = 100;
        let energy: Vec<f64> = (0..n).map(|i| i as f64 * 0.1).collect();
        let background = vec![100.0; n];
        let mut counts = background.clone();
        // Small bump that shouldn't exceed SNR threshold
        counts[50] = 103.0;
        let peaks = find_peaks(&energy, &counts, &background, 3.0, 5);
        assert!(peaks.is_empty(), "Should not detect peaks below SNR threshold");
    }

    #[test]
    fn test_find_peaks_two_peaks() {
        let n = 300;
        let energy: Vec<f64> = (0..n).map(|i| i as f64 * 0.1).collect();
        let background = vec![10.0; n];
        let mut counts = background.clone();
        for i in 0..n {
            counts[i] += gaussian(i as f64, 200.0, 80.0, 5.0);
            counts[i] += gaussian(i as f64, 150.0, 200.0, 5.0);
        }
        let peaks = find_peaks(&energy, &counts, &background, 3.0, 5);
        assert!(peaks.len() >= 2, "Should detect two peaks, got {}", peaks.len());
    }

    // --- Peak identification tests ---

    #[test]
    fn test_identify_peaks_iron() {
        let peak = DetectedPeak {
            channel: 640,
            energy_kev: 6.40,
            amplitude: 1000.0,
            sigma_kev: 0.05,
            snr: 30.0,
            gross_area: 5000.0,
            net_area: 4500.0,
            background_area: 500.0,
        };
        let db = emission_line_database();
        let identified = identify_peaks(&[peak], &db, 0.15);
        assert!(!identified.is_empty(), "Should identify Fe Kα");
        assert_eq!(identified[0].emission_line.element.symbol, "Fe");
        assert_eq!(identified[0].emission_line.line_type, LineType::Ka);
    }

    #[test]
    fn test_identify_peaks_no_match() {
        let peak = DetectedPeak {
            channel: 100,
            energy_kev: 15.0, // between Mo Kα and Ag Kα
            amplitude: 100.0,
            sigma_kev: 0.05,
            snr: 10.0,
            gross_area: 500.0,
            net_area: 400.0,
            background_area: 100.0,
        };
        let db = emission_line_database();
        let identified = identify_peaks(&[peak], &db, 0.1);
        assert!(identified.is_empty(), "Should not match at 15 keV with tight tolerance");
    }

    // --- Gaussian fit tests ---

    #[test]
    fn test_gaussian_evaluation() {
        let val = gaussian(5.0, 100.0, 5.0, 1.0);
        assert!((val - 100.0).abs() < EPSILON, "Gaussian at center should equal amplitude");

        let val_off = gaussian(6.0, 100.0, 5.0, 1.0);
        assert!(val_off < 100.0, "Gaussian away from center should be less");
        assert!(val_off > 0.0);
    }

    #[test]
    fn test_gaussian_fit_peak_centered() {
        let n = 100;
        let bg = vec![10.0; n];
        let mut counts = bg.clone();
        for i in 0..n {
            counts[i] += gaussian(i as f64, 500.0, 50.0, 4.0);
        }
        let (amp, center, sigma) = gaussian_fit_peak(&counts, &bg, 50);
        assert!((amp - 500.0).abs() < 5.0, "Amplitude off: {}", amp);
        assert!((center - 50.0).abs() < 0.5, "Center off: {}", center);
        assert!((sigma - 4.0).abs() < 1.0, "Sigma off: {}", sigma);
    }

    // --- Quantitative analysis tests ---

    #[test]
    fn test_linear_regression_perfect() {
        let points = vec![
            CalibrationPoint { net_peak_area: 100.0, concentration_ppm: 10.0 },
            CalibrationPoint { net_peak_area: 200.0, concentration_ppm: 20.0 },
            CalibrationPoint { net_peak_area: 300.0, concentration_ppm: 30.0 },
        ];
        let (intercept, slope, r_sq) = linear_regression(&points);
        assert!((slope - 0.1).abs() < EPSILON, "Slope should be 0.1, got {}", slope);
        assert!(intercept.abs() < EPSILON, "Intercept should be ~0, got {}", intercept);
        assert!((r_sq - 1.0).abs() < EPSILON, "R² should be 1.0, got {}", r_sq);
    }

    #[test]
    fn test_calibration_curve_concentration() {
        let points = vec![
            CalibrationPoint { net_peak_area: 100.0, concentration_ppm: 50.0 },
            CalibrationPoint { net_peak_area: 200.0, concentration_ppm: 100.0 },
        ];
        let conc = calibration_curve_concentration(150.0, &points);
        assert!((conc - 75.0).abs() < 0.1, "Should interpolate to 75 ppm, got {}", conc);
    }

    #[test]
    fn test_internal_standard_concentration() {
        let conc = internal_standard_concentration(500.0, 1000.0, 10000.0, 1.2);
        assert!((conc - 6000.0).abs() < EPSILON);
    }

    #[test]
    fn test_internal_standard_zero_standard() {
        let conc = internal_standard_concentration(500.0, 0.0, 10000.0, 1.0);
        assert!((conc - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_fundamental_parameters() {
        let conc = fundamental_parameters_concentration(
            1000.0,  // net area
            100.0,   // live time
            0.3,     // fluorescence yield
            1.0,     // absorption correction
            0.01,    // sensitivity
        );
        // rate = 1000/100 = 10; conc = 10/(0.01*0.3*1.0)*1e6 = 3.33e9
        assert!(conc > 0.0);
    }

    // --- Detection limits tests ---

    #[test]
    fn test_mdl_calculation() {
        let mdl = detection_limit_mdl(1000.0, 500.0, 100.0);
        // MDL = 3 * sqrt(1000) * 100 / 500 = 3*31.62*0.2 = 18.97
        assert!((mdl - 18.974).abs() < 0.1, "MDL should be ~18.97 ppm, got {}", mdl);
    }

    #[test]
    fn test_loq_calculation() {
        let loq = detection_limit_loq(1000.0, 500.0, 100.0);
        // LOQ = 10 * sqrt(1000) * 100 / 500 = 10*31.62*0.2 = 63.25
        assert!((loq - 63.246).abs() < 0.1, "LOQ should be ~63.25 ppm, got {}", loq);
    }

    #[test]
    fn test_mdl_zero_net_counts() {
        let mdl = detection_limit_mdl(1000.0, 0.0, 100.0);
        assert!(mdl.is_infinite());
    }

    // --- Dead time correction tests ---

    #[test]
    fn test_dead_time_correction_normal() {
        // 10000 cps with 1 us dead time
        let corrected = dead_time_correction(10000.0, 1e-6).unwrap();
        // Expected: 10000 / (1 - 0.01) = 10101.01
        assert!((corrected - 10101.01).abs() < 0.1);
    }

    #[test]
    fn test_dead_time_correction_saturated() {
        // Rate exceeds 1/tau
        let result = dead_time_correction(1e6, 1e-6);
        assert!(result.is_none(), "Should return None when saturated");
    }

    #[test]
    fn test_dead_time_correction_zero_tau() {
        let corrected = dead_time_correction(10000.0, 0.0).unwrap();
        assert!((corrected - 10000.0).abs() < EPSILON);
    }

    #[test]
    fn test_apply_dead_time_correction() {
        let mut spec = XrfSpectrum::new(
            vec![1.0, 2.0, 3.0],
            vec![100.0, 200.0, 300.0],
            100.0,
            105.0,
        );
        let total_before = spec.total_counts();
        apply_dead_time_correction(&mut spec, 1e-6);
        let total_after = spec.total_counts();
        assert!(total_after >= total_before, "Dead time correction should increase counts");
    }

    // --- Matrix effects tests ---

    #[test]
    fn test_mass_attenuation_increases_with_z() {
        let mu_fe = mass_attenuation_approx(26, 10.0);
        let mu_pb = mass_attenuation_approx(82, 10.0);
        assert!(mu_pb > mu_fe, "Heavy elements should attenuate more");
    }

    #[test]
    fn test_mass_attenuation_decreases_with_energy() {
        let mu_low = mass_attenuation_approx(26, 5.0);
        let mu_high = mass_attenuation_approx(26, 20.0);
        assert!(mu_low > mu_high, "Attenuation should decrease with energy");
    }

    #[test]
    fn test_beer_lambert_absorption() {
        let transmitted = beer_lambert(1000.0, 100.0, 7.87, 0.001);
        assert!(transmitted < 1000.0, "Transmitted should be less than incident");
        assert!(transmitted > 0.0, "Transmitted should be positive");
    }

    #[test]
    fn test_beer_lambert_zero_thickness() {
        let transmitted = beer_lambert(1000.0, 100.0, 7.87, 0.0);
        assert!((transmitted - 1000.0).abs() < EPSILON);
    }

    #[test]
    fn test_enhancement_factor_no_enhancement() {
        // Enhancer energy lower than analyte: no enhancement
        let ef = enhancement_factor(8.0, 6.0, 0.5, 0.3);
        assert!((ef - 1.0).abs() < EPSILON, "No enhancement when E_enhancer < E_analyte");
    }

    #[test]
    fn test_enhancement_factor_positive() {
        let ef = enhancement_factor(6.0, 8.0, 0.5, 0.3);
        assert!(ef > 1.0, "Enhancement factor should be > 1");
    }

    #[test]
    fn test_fluorescence_yield_k() {
        let yield_fe = fluorescence_yield_k(26);
        assert!(yield_fe > 0.0 && yield_fe < 1.0, "Fluorescence yield should be 0-1");

        let yield_pb = fluorescence_yield_k(82);
        assert!(yield_pb > yield_fe, "Heavier elements should have higher K yield");
    }

    // --- Full analyzer tests ---

    #[test]
    fn test_analyzer_default_creation() {
        let analyzer = XrfAnalyzer::default_analyzer();
        assert!(!analyzer.database.is_empty());
        assert!((analyzer.config.energy_resolution_ev - 150.0).abs() < EPSILON);
    }

    #[test]
    fn test_analyzer_simulated_iron_copper() {
        let concentrations = vec![(26, 0.50), (29, 0.30)];
        let spectrum = simulate_xrf_spectrum(&concentrations, 4096, 0.5, 40.0, 150.0, 100.0);

        let config = XrfAnalyzerConfig {
            energy_resolution_ev: 150.0,
            min_energy_kev: 1.0,
            max_energy_kev: 40.0,
            snr_threshold: 5.0,
            energy_tolerance_kev: 0.2,
            snip_iterations: 24,
            min_peak_separation: 5,
        };
        let analyzer = XrfAnalyzer::new(config);
        let results = analyzer.analyze(&spectrum);

        assert!(!results.identified_peaks.is_empty(), "Should identify peaks");

        // Check that Fe and Cu are found
        let found_fe = results.identified_peaks.iter().any(|ip| ip.emission_line.element.symbol == "Fe");
        let found_cu = results.identified_peaks.iter().any(|ip| ip.emission_line.element.symbol == "Cu");
        assert!(found_fe, "Should identify Fe");
        assert!(found_cu, "Should identify Cu");
    }

    #[test]
    fn test_analyzer_quant_results() {
        let concentrations = vec![(26, 0.60), (28, 0.40)];
        let spectrum = simulate_xrf_spectrum(&concentrations, 4096, 0.5, 40.0, 150.0, 100.0);

        let analyzer = XrfAnalyzer::default_analyzer();
        let results = analyzer.analyze(&spectrum);

        // All quant results should have positive concentrations
        for qr in &results.quant_results {
            assert!(qr.concentration_ppm >= 0.0);
            assert!(qr.net_area >= 0.0);
        }
    }

    #[test]
    fn test_simulate_spectrum_properties() {
        let spectrum = simulate_xrf_spectrum(&[(26, 0.5)], 2048, 0.0, 40.0, 150.0, 60.0);
        assert_eq!(spectrum.num_channels(), 2048);
        assert!((spectrum.live_time_s - 60.0).abs() < EPSILON);
        assert!(spectrum.total_counts() > 0.0);
        assert!(spectrum.dead_time_fraction() > 0.0);
    }

    #[test]
    fn test_simulate_empty_concentrations() {
        let spectrum = simulate_xrf_spectrum(&[], 1024, 0.0, 40.0, 150.0, 10.0);
        assert_eq!(spectrum.num_channels(), 1024);
        // Should still have background Bremsstrahlung
        assert!(spectrum.total_counts() > 0.0);
    }

    // --- Edge cases ---

    #[test]
    fn test_linear_regression_single_point() {
        let points = vec![CalibrationPoint { net_peak_area: 100.0, concentration_ppm: 50.0 }];
        let (intercept, slope, _) = linear_regression(&points);
        // With fewer than 2 points, returns default (intercept=0, slope=1)
        assert!((intercept - 0.0).abs() < EPSILON);
        assert!((slope - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_linear_regression_empty() {
        let (_, slope, _) = linear_regression(&[]);
        assert!((slope - 1.0).abs() < EPSILON, "Empty regression should return default slope=1");
    }

    #[test]
    fn test_mass_attenuation_zero_energy() {
        let mu = mass_attenuation_approx(26, 0.0);
        assert!((mu - 0.0).abs() < EPSILON);
    }
}
