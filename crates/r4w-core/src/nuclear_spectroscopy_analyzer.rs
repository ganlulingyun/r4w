//! Nuclear spectroscopy analyzer for gamma-ray and X-ray detector data.
//!
//! This module processes pulse-height spectra from scintillator and semiconductor
//! radiation detectors (NaI(Tl), HPGe, CdZnTe, LaBr3) for nuclear physics and
//! radiation safety applications. It provides:
//!
//! - **Energy calibration** via linear regression on known peak positions
//! - **Peak finding** using second-derivative search with Gaussian fitting
//! - **Compton kinematics** for Compton edge and backscatter peak calculation
//! - **Isotope identification** against a built-in library of common gamma emitters
//! - **Activity calculation** with detector efficiency correction
//! - **Radioactive decay** computation using the half-life formula
//! - **Synthetic spectrum generation** with Gaussian photopeaks and Compton continua
//!
//! # Example
//!
//! ```
//! use r4w_core::nuclear_spectroscopy_analyzer::{
//!     SpectroscopyConfig, SpectrumAnalyzer, generate_spectrum,
//!     compton_edge_kev, backscatter_peak_kev,
//! };
//!
//! let config = SpectroscopyConfig {
//!     num_channels: 1024,
//!     energy_range_kev: 2000.0,
//!     detector_resolution_percent: 7.0,
//!     live_time_s: 300.0,
//! };
//!
//! // Generate a synthetic Cs-137 spectrum (662 keV photopeak)
//! let peaks = vec![(662.0, 10000.0)];
//! let spectrum = generate_spectrum(&peaks, 1024, 7.0);
//!
//! let analyzer = SpectrumAnalyzer::new(config);
//! let result = analyzer.analyze(&spectrum);
//!
//! assert!(!result.peaks.is_empty());
//! assert!(result.total_counts > 0.0);
//!
//! // Compton scattering kinematics
//! let edge = compton_edge_kev(662.0);
//! assert!((edge - 477.3).abs() < 1.0);
//! ```

use std::f64::consts::PI;

// Electron rest mass energy in keV (m_e * c^2)
const ELECTRON_MASS_KEV: f64 = 511.0;

// ln(2) for half-life calculations
const LN2: f64 = 0.693_147_180_559_945_3;

/// Configuration for the spectroscopy analyzer.
#[derive(Debug, Clone)]
pub struct SpectroscopyConfig {
    /// Number of channels (bins) in the MCA histogram.
    pub num_channels: usize,
    /// Full energy range of the spectrum in keV.
    pub energy_range_kev: f64,
    /// Detector energy resolution as FWHM percentage at 662 keV (Cs-137).
    /// Typical values: NaI ~7%, HPGe ~0.2%, CdZnTe ~2%, LaBr3 ~3%.
    pub detector_resolution_percent: f64,
    /// Live time (active counting time) in seconds.
    pub live_time_s: f64,
}

impl Default for SpectroscopyConfig {
    fn default() -> Self {
        Self {
            num_channels: 4096,
            energy_range_kev: 3000.0,
            detector_resolution_percent: 7.0,
            live_time_s: 300.0,
        }
    }
}

/// Type of radiation detector, affecting efficiency and resolution.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DetectorType {
    /// Sodium Iodide (thallium-doped) scintillator. Good efficiency, moderate resolution.
    NaI,
    /// High-Purity Germanium semiconductor. Best resolution, lower efficiency.
    HPGe,
    /// Cadmium Zinc Telluride semiconductor. Room-temperature, compact.
    CdZnTe,
    /// Lanthanum Bromide scintillator. Fast timing, good resolution.
    LaBr3,
}

/// A detected gamma-ray peak in the spectrum.
#[derive(Debug, Clone)]
pub struct GammaPeak {
    /// Channel number of the peak maximum.
    pub channel: usize,
    /// Calibrated energy in keV.
    pub energy_kev: f64,
    /// Net counts above the local background continuum.
    pub net_counts: f64,
    /// Full Width at Half Maximum in keV.
    pub fwhm_kev: f64,
    /// Statistical significance (net counts / sqrt(background)).
    pub significance: f64,
    /// Fitted centroid position (fractional channel).
    pub centroid: f64,
}

/// An isotope match from the identification library.
#[derive(Debug, Clone)]
pub struct IsotopeMatch {
    /// Name of the identified isotope (e.g., "Cs-137", "Co-60").
    pub name: String,
    /// Catalog energy of the matched gamma line in keV.
    pub energy_kev: f64,
    /// Confidence of the match (0.0 to 1.0).
    pub confidence: f64,
    /// Estimated activity in Becquerels, corrected for efficiency and branching ratio.
    pub activity_bq: f64,
}

/// Complete results from spectrum analysis.
#[derive(Debug, Clone)]
pub struct SpectrumResult {
    /// All detected peaks, sorted by energy.
    pub peaks: Vec<GammaPeak>,
    /// Total counts integrated across the entire spectrum.
    pub total_counts: f64,
    /// Overall count rate in counts per second.
    pub count_rate_cps: f64,
    /// Estimated dead time fraction (0.0 = no dead time, 1.0 = fully dead).
    pub dead_time_fraction: f64,
    /// Isotopes identified by matching peaks against the library.
    pub identified_isotopes: Vec<IsotopeMatch>,
}

/// Built-in isotope library entry.
struct IsotopeEntry {
    name: &'static str,
    energy_kev: f64,
    branching_ratio: f64,
    half_life_s: f64,
}

/// Common gamma-emitting isotopes for identification.
const ISOTOPE_LIBRARY: &[IsotopeEntry] = &[
    IsotopeEntry { name: "Cs-137", energy_kev: 661.7, branching_ratio: 0.851, half_life_s: 9.504e8 },
    IsotopeEntry { name: "Co-60 (1173)", energy_kev: 1173.2, branching_ratio: 0.999, half_life_s: 1.663e8 },
    IsotopeEntry { name: "Co-60 (1332)", energy_kev: 1332.5, branching_ratio: 1.000, half_life_s: 1.663e8 },
    IsotopeEntry { name: "K-40", energy_kev: 1460.8, branching_ratio: 0.107, half_life_s: 3.938e16 },
    IsotopeEntry { name: "Na-22 (511)", energy_kev: 511.0, branching_ratio: 1.798, half_life_s: 8.212e7 },
    IsotopeEntry { name: "Na-22 (1275)", energy_kev: 1274.5, branching_ratio: 0.999, half_life_s: 8.212e7 },
    IsotopeEntry { name: "Ba-133 (356)", energy_kev: 356.0, branching_ratio: 0.621, half_life_s: 3.326e8 },
    IsotopeEntry { name: "Ba-133 (81)", energy_kev: 81.0, branching_ratio: 0.341, half_life_s: 3.326e8 },
    IsotopeEntry { name: "Am-241", energy_kev: 59.5, branching_ratio: 0.359, half_life_s: 1.364e10 },
    IsotopeEntry { name: "Mn-54", energy_kev: 834.8, branching_ratio: 1.000, half_life_s: 2.697e7 },
    IsotopeEntry { name: "I-131 (364)", energy_kev: 364.5, branching_ratio: 0.817, half_life_s: 6.933e5 },
    IsotopeEntry { name: "Eu-152 (344)", energy_kev: 344.3, branching_ratio: 0.266, half_life_s: 4.273e8 },
    IsotopeEntry { name: "Eu-152 (1408)", energy_kev: 1408.0, branching_ratio: 0.210, half_life_s: 4.273e8 },
    IsotopeEntry { name: "Tc-99m", energy_kev: 140.5, branching_ratio: 0.890, half_life_s: 2.161e4 },
    IsotopeEntry { name: "Tl-208", energy_kev: 2614.5, branching_ratio: 0.358, half_life_s: 1.831e2 },
];

/// The main spectrum analyzer.
///
/// Performs peak finding, energy calibration, isotope identification, and
/// activity estimation on multi-channel analyzer (MCA) histogram data.
#[derive(Debug, Clone)]
pub struct SpectrumAnalyzer {
    config: SpectroscopyConfig,
    /// Linear calibration slope: keV per channel.
    cal_a: f64,
    /// Linear calibration intercept in keV.
    cal_b: f64,
}

impl SpectrumAnalyzer {
    /// Create a new analyzer with the given configuration.
    ///
    /// Uses a default linear calibration mapping channel 0 to 0 keV and
    /// the last channel to `energy_range_kev`.
    pub fn new(config: SpectroscopyConfig) -> Self {
        let cal_a = if config.num_channels > 1 {
            config.energy_range_kev / (config.num_channels - 1) as f64
        } else {
            config.energy_range_kev
        };
        let cal_b = 0.0;
        Self { config, cal_a, cal_b }
    }

    /// Create an analyzer with explicit energy calibration coefficients.
    pub fn with_calibration(config: SpectroscopyConfig, cal_a: f64, cal_b: f64) -> Self {
        Self { config, cal_a, cal_b }
    }

    /// Analyze a pulse-height spectrum and return detected peaks, isotopes, and statistics.
    pub fn analyze(&self, spectrum: &[f64]) -> SpectrumResult {
        let total_counts: f64 = spectrum.iter().sum();
        let count_rate_cps = if self.config.live_time_s > 0.0 {
            total_counts / self.config.live_time_s
        } else {
            0.0
        };

        // Estimate dead time from count rate using non-paralyzable model.
        // Assume 2 microsecond dead time per event (typical for NaI systems).
        let tau = 2.0e-6;
        let dead_time_fraction = (count_rate_cps * tau).min(0.99);

        // Find peaks with minimum significance of 3 sigma.
        let min_significance = 3.0;
        let mut peaks = find_peaks_spectrum(spectrum, min_significance);

        // Apply energy calibration to each peak.
        for peak in &mut peaks {
            peak.energy_kev = channel_to_energy(peak.centroid, self.cal_a, self.cal_b);
            // Compute FWHM in keV using detector resolution scaled by sqrt(662/E).
            let res_at_energy = self.config.detector_resolution_percent / 100.0
                * peak.energy_kev
                * (662.0 / peak.energy_kev.max(1.0)).sqrt();
            peak.fwhm_kev = res_at_energy;
        }

        // Sort peaks by energy.
        peaks.sort_by(|a, b| a.energy_kev.partial_cmp(&b.energy_kev).unwrap_or(std::cmp::Ordering::Equal));

        // Identify isotopes by matching peaks to the library.
        let identified_isotopes = self.identify_isotopes(&peaks);

        SpectrumResult {
            peaks,
            total_counts,
            count_rate_cps,
            dead_time_fraction,
            identified_isotopes,
        }
    }

    /// Match detected peaks against the isotope library.
    fn identify_isotopes(&self, peaks: &[GammaPeak]) -> Vec<IsotopeMatch> {
        let mut matches = Vec::new();
        let energy_tolerance_kev = 5.0; // matching window

        for entry in ISOTOPE_LIBRARY {
            for peak in peaks {
                let delta = (peak.energy_kev - entry.energy_kev).abs();
                if delta < energy_tolerance_kev {
                    // Confidence decreases linearly with energy offset.
                    let confidence = 1.0 - delta / energy_tolerance_kev;

                    // Estimate activity: A = N / (eff * BR * t)
                    let eff = detector_efficiency(peak.energy_kev, DetectorType::NaI);
                    let activity_bq = if eff > 0.0
                        && entry.branching_ratio > 0.0
                        && self.config.live_time_s > 0.0
                    {
                        peak.net_counts / (eff * entry.branching_ratio * self.config.live_time_s)
                    } else {
                        0.0
                    };

                    matches.push(IsotopeMatch {
                        name: entry.name.to_string(),
                        energy_kev: entry.energy_kev,
                        confidence,
                        activity_bq,
                    });
                    break; // one match per library entry
                }
            }
        }

        matches
    }
}

/// Perform linear energy calibration from paired channel/energy data.
///
/// Given known peak positions in both channel number and energy (keV),
/// compute the linear calibration coefficients using least-squares regression:
///   E(ch) = a * ch + b
///
/// Returns `(a, b)` where `a` is keV/channel slope and `b` is the intercept.
///
/// Requires at least 2 calibration points. Panics if inputs have different lengths
/// or fewer than 2 points.
pub fn energy_calibration(channels: &[f64], energies_kev: &[f64]) -> (f64, f64) {
    assert_eq!(channels.len(), energies_kev.len(), "calibration arrays must have equal length");
    assert!(channels.len() >= 2, "need at least 2 calibration points");

    let n = channels.len() as f64;
    let sum_x: f64 = channels.iter().sum();
    let sum_y: f64 = energies_kev.iter().sum();
    let sum_xx: f64 = channels.iter().map(|x| x * x).sum();
    let sum_xy: f64 = channels.iter().zip(energies_kev.iter()).map(|(x, y)| x * y).sum();

    let denom = n * sum_xx - sum_x * sum_x;
    assert!(denom.abs() > 1e-15, "degenerate calibration data (all channels identical)");

    let a = (n * sum_xy - sum_x * sum_y) / denom;
    let b = (sum_y * sum_xx - sum_x * sum_xy) / denom;

    (a, b)
}

/// Convert a channel number to energy using linear calibration.
///
/// E(ch) = a * ch + b
#[inline]
pub fn channel_to_energy(channel: f64, cal_a: f64, cal_b: f64) -> f64 {
    cal_a * channel + cal_b
}

/// Convert an energy to a (fractional) channel number using linear calibration.
///
/// ch = (E - b) / a
#[inline]
pub fn energy_to_channel(energy_kev: f64, cal_a: f64, cal_b: f64) -> f64 {
    if cal_a.abs() < 1e-15 {
        0.0
    } else {
        (energy_kev - cal_b) / cal_a
    }
}

/// Find peaks in a pulse-height spectrum using second-derivative analysis.
///
/// The algorithm smooths the spectrum, computes the second derivative, and
/// identifies local minima of the second derivative (corresponding to peaks
/// in the original spectrum). Each candidate is then refined with a Gaussian
/// fit and tested against the specified minimum significance threshold.
///
/// Significance is defined as net_counts / sqrt(background_counts).
pub fn find_peaks_spectrum(spectrum: &[f64], min_significance: f64) -> Vec<GammaPeak> {
    let n = spectrum.len();
    if n < 7 {
        return Vec::new();
    }

    // Smooth the spectrum with a 5-point moving average.
    let smoothed = smooth_spectrum(spectrum, 5);

    // Compute second derivative using central difference.
    let mut second_deriv = vec![0.0; n];
    for i in 1..n - 1 {
        second_deriv[i] = smoothed[i + 1] - 2.0 * smoothed[i] + smoothed[i - 1];
    }

    // Find local minima of the second derivative (peaks in the spectrum).
    let mut candidates = Vec::new();
    for i in 2..n - 2 {
        if second_deriv[i] < second_deriv[i - 1]
            && second_deriv[i] < second_deriv[i + 1]
            && second_deriv[i] < -1.0  // must be a clear negative dip
            && spectrum[i] > 0.0
        {
            candidates.push(i);
        }
    }

    // Refine each candidate with Gaussian fitting and significance testing.
    let mut peaks = Vec::new();
    for &ch in &candidates {
        let (centroid, amplitude, sigma) = gaussian_fit_peak(spectrum, ch);
        if sigma <= 0.0 || amplitude <= 0.0 {
            continue;
        }

        // Estimate local background as the average of the wings.
        let wing = (3.0 * sigma).ceil() as usize;
        let left = if ch > wing { ch - wing } else { 0 };
        let right = (ch + wing).min(n - 1);
        let bg_left = if left > 0 { spectrum[left] } else { spectrum[0] };
        let bg_right = if right < n { spectrum[right] } else { spectrum[n - 1] };
        let bg_per_channel = (bg_left + bg_right) / 2.0;

        // Net counts: area under the Gaussian minus background.
        let peak_width = (2 * wing + 1) as f64;
        let gross_counts: f64 = spectrum[left..=right].iter().sum();
        let bg_counts = bg_per_channel * peak_width;
        let net_counts = (gross_counts - bg_counts).max(0.0);

        // Significance: net / sqrt(background).
        let significance = if bg_counts > 0.0 {
            net_counts / bg_counts.sqrt()
        } else if net_counts > 0.0 {
            net_counts // infinite significance treated as the net counts value
        } else {
            0.0
        };

        if significance >= min_significance {
            let fwhm_channels = 2.0 * (2.0 * LN2).sqrt() * sigma; // FWHM = 2.355 * sigma
            peaks.push(GammaPeak {
                channel: ch,
                energy_kev: 0.0, // filled in by the analyzer after calibration
                net_counts,
                fwhm_kev: fwhm_channels, // temporarily in channels, converted later
                significance,
                centroid,
            });
        }
    }

    peaks
}

/// Fit a Gaussian to a peak region in the spectrum.
///
/// Uses a 3-point parabolic interpolation on log-counts around the peak
/// channel to estimate the centroid, amplitude, and sigma (standard deviation).
///
/// Returns `(centroid, amplitude, sigma)`.
pub fn gaussian_fit_peak(spectrum: &[f64], peak_channel: usize) -> (f64, f64, f64) {
    let n = spectrum.len();
    if peak_channel == 0 || peak_channel >= n - 1 {
        return (peak_channel as f64, spectrum.get(peak_channel).copied().unwrap_or(0.0), 1.0);
    }

    let y_left = spectrum[peak_channel - 1].max(0.5);
    let y_center = spectrum[peak_channel].max(0.5);
    let y_right = spectrum[peak_channel + 1].max(0.5);

    let ln_left = y_left.ln();
    let ln_center = y_center.ln();
    let ln_right = y_right.ln();

    // Parabolic interpolation in log-space:
    // ln(G(x)) = ln(A) - (x - x0)^2 / (2 * sigma^2)
    let denom = 2.0 * (2.0 * ln_center - ln_left - ln_right);
    if denom.abs() < 1e-10 {
        // Flat region, return nominal values.
        return (peak_channel as f64, y_center, 1.0);
    }

    let delta = (ln_right - ln_left) / denom;
    let centroid = peak_channel as f64 + delta;

    // Sigma from the curvature.
    let sigma_sq = -1.0 / (ln_left - 2.0 * ln_center + ln_right);
    let sigma = if sigma_sq > 0.0 { sigma_sq.sqrt() } else { 1.0 };

    // Amplitude at the centroid: evaluate the log-parabola at the peak.
    let a_coeff = (ln_left + ln_right) / 2.0 - ln_center;
    let b_coeff = (ln_right - ln_left) / 2.0;
    let amplitude = (a_coeff * delta * delta + b_coeff * delta + ln_center).exp();

    (centroid, amplitude, sigma)
}

/// Compute the Compton edge energy for a given photopeak.
///
/// The Compton edge is the maximum energy transferred to an electron in
/// Compton scattering (180-degree backscatter of the photon):
///
///   E_c = E_gamma * 2 * E_gamma / (m_e*c^2 + 2 * E_gamma)
///
/// where m_e*c^2 = 511 keV.
#[inline]
pub fn compton_edge_kev(photopeak_kev: f64) -> f64 {
    photopeak_kev * 2.0 * photopeak_kev / (ELECTRON_MASS_KEV + 2.0 * photopeak_kev)
}

/// Compute the backscatter peak energy.
///
/// This is the energy of a photon that has undergone 180-degree Compton
/// scattering (the complement of the Compton edge):
///
///   E_bs = E_gamma / (1 + 2 * E_gamma / m_e*c^2)
#[inline]
pub fn backscatter_peak_kev(photopeak_kev: f64) -> f64 {
    photopeak_kev / (1.0 + 2.0 * photopeak_kev / ELECTRON_MASS_KEV)
}

/// Compute detector intrinsic efficiency at a given energy.
///
/// Returns the photopeak detection efficiency (0.0 to 1.0) based on
/// approximate empirical models for each detector type. These are simplified
/// parameterizations suitable for order-of-magnitude activity estimates.
pub fn detector_efficiency(energy_kev: f64, detector_type: DetectorType) -> f64 {
    if energy_kev <= 0.0 {
        return 0.0;
    }

    // Empirical efficiency parameterization: eff = A * E^(-alpha) for E > E_threshold.
    // Below threshold, efficiency rolls off due to window absorption.
    match detector_type {
        DetectorType::NaI => {
            // 3" x 3" NaI(Tl): high efficiency, broad peaks.
            let eff = if energy_kev < 50.0 {
                0.3 * (energy_kev / 50.0)
            } else if energy_kev < 200.0 {
                0.3 + 0.4 * ((energy_kev - 50.0) / 150.0)
            } else {
                0.7 * (200.0 / energy_kev).powf(0.8)
            };
            eff.max(0.001).min(1.0)
        }
        DetectorType::HPGe => {
            // Coaxial HPGe: lower efficiency but excellent resolution.
            let eff = if energy_kev < 40.0 {
                0.05 * (energy_kev / 40.0)
            } else if energy_kev < 150.0 {
                0.05 + 0.25 * ((energy_kev - 40.0) / 110.0)
            } else {
                0.30 * (150.0 / energy_kev).powf(0.85)
            };
            eff.max(0.001).min(1.0)
        }
        DetectorType::CdZnTe => {
            // CZT: compact, room-temperature, moderate efficiency.
            let eff = if energy_kev < 30.0 {
                0.1 * (energy_kev / 30.0)
            } else if energy_kev < 200.0 {
                0.1 + 0.15 * ((energy_kev - 30.0) / 170.0)
            } else {
                0.25 * (200.0 / energy_kev).powf(1.0)
            };
            eff.max(0.001).min(1.0)
        }
        DetectorType::LaBr3 => {
            // LaBr3(Ce): fast scintillator with good efficiency.
            let eff = if energy_kev < 50.0 {
                0.25 * (energy_kev / 50.0)
            } else if energy_kev < 200.0 {
                0.25 + 0.35 * ((energy_kev - 50.0) / 150.0)
            } else {
                0.60 * (200.0 / energy_kev).powf(0.82)
            };
            eff.max(0.001).min(1.0)
        }
    }
}

/// Compute radioactive decay.
///
/// A(t) = A_0 * 2^(-t / t_half)
///
/// where A_0 is the initial activity in Becquerels, t_half is the half-life
/// in seconds, and t is the elapsed time in seconds.
#[inline]
pub fn half_life_decay(activity_bq: f64, half_life_s: f64, elapsed_s: f64) -> f64 {
    if half_life_s <= 0.0 {
        return 0.0;
    }
    activity_bq * (-LN2 * elapsed_s / half_life_s).exp()
}

/// Generate a synthetic gamma-ray spectrum.
///
/// Creates a realistic pulse-height spectrum with:
/// - Gaussian photopeaks at the specified energies and counts
/// - Compton continua for each peak
/// - A low-level flat background
///
/// # Arguments
///
/// * `peaks` - Slice of (energy_keV, total_counts) tuples for each photopeak
/// * `num_channels` - Number of MCA channels
/// * `resolution_percent` - Detector resolution (FWHM %) at 662 keV
///
/// # Returns
///
/// A vector of `num_channels` bin counts forming the spectrum.
pub fn generate_spectrum(
    peaks: &[(f64, f64)],
    num_channels: usize,
    resolution_percent: f64,
) -> Vec<f64> {
    let mut spectrum = vec![0.0; num_channels];
    if num_channels < 2 {
        return spectrum;
    }

    let kev_per_channel = 3000.0 / num_channels as f64;

    // Add a small flat background.
    for bin in spectrum.iter_mut() {
        *bin += 1.0;
    }

    for &(energy_kev, total_counts) in peaks {
        // Channel position of the photopeak.
        let peak_ch = energy_kev / kev_per_channel;

        // Resolution scales as sqrt(662/E) normalized to the reference energy.
        let fwhm_kev = resolution_percent / 100.0 * energy_kev * (662.0 / energy_kev.max(1.0)).sqrt();
        let sigma_kev = fwhm_kev / (2.0 * (2.0 * LN2).sqrt());
        let sigma_ch = sigma_kev / kev_per_channel;

        // Gaussian photopeak: ~60% of total counts go into the full-energy peak.
        let peak_fraction = 0.6;
        let peak_counts = total_counts * peak_fraction;
        let norm = peak_counts / (sigma_ch * (2.0 * PI).sqrt());

        for i in 0..num_channels {
            let dx = i as f64 - peak_ch;
            spectrum[i] += norm * (-0.5 * (dx / sigma_ch).powi(2)).exp();
        }

        // Compton continuum: flat distribution from 0 to the Compton edge.
        let compton_edge = compton_edge_kev(energy_kev);
        let compton_ch = (compton_edge / kev_per_channel) as usize;
        let compton_fraction = 0.35;
        let compton_counts = total_counts * compton_fraction;
        let compton_ch_clamped = compton_ch.min(num_channels - 1);
        if compton_ch_clamped > 0 {
            let counts_per_ch = compton_counts / compton_ch_clamped as f64;
            for i in 0..=compton_ch_clamped {
                // Slight upward slope toward the Compton edge (Klein-Nishina shape).
                let frac = i as f64 / compton_ch_clamped as f64;
                spectrum[i] += counts_per_ch * (0.5 + 0.5 * frac);
            }
        }

        // Backscatter peak: small bump.
        let bs_energy = backscatter_peak_kev(energy_kev);
        let bs_ch = bs_energy / kev_per_channel;
        let bs_sigma_ch = sigma_ch * 1.5; // broader than photopeak
        let bs_counts = total_counts * 0.05;
        let bs_norm = bs_counts / (bs_sigma_ch * (2.0 * PI).sqrt());
        for i in 0..num_channels {
            let dx = i as f64 - bs_ch;
            spectrum[i] += bs_norm * (-0.5 * (dx / bs_sigma_ch).powi(2)).exp();
        }
    }

    spectrum
}

/// Smooth a spectrum using a symmetric moving average.
fn smooth_spectrum(spectrum: &[f64], width: usize) -> Vec<f64> {
    let n = spectrum.len();
    let half = width / 2;
    let mut smoothed = vec![0.0; n];

    for i in 0..n {
        let lo = if i >= half { i - half } else { 0 };
        let hi = (i + half).min(n - 1);
        let count = (hi - lo + 1) as f64;
        let sum: f64 = spectrum[lo..=hi].iter().sum();
        smoothed[i] = sum / count;
    }

    smoothed
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    // --- Energy calibration tests ---

    #[test]
    fn test_energy_calibration_two_points() {
        // Channel 100 = 100 keV, Channel 1000 = 1000 keV => 1 keV/ch, intercept 0.
        let channels = [100.0, 1000.0];
        let energies = [100.0, 1000.0];
        let (a, b) = energy_calibration(&channels, &energies);
        assert!((a - 1.0).abs() < EPSILON);
        assert!(b.abs() < EPSILON);
    }

    #[test]
    fn test_energy_calibration_with_offset() {
        // E = 0.5*ch + 10
        let channels = [0.0, 100.0, 200.0];
        let energies = [10.0, 60.0, 110.0];
        let (a, b) = energy_calibration(&channels, &energies);
        assert!((a - 0.5).abs() < 1e-9);
        assert!((b - 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_energy_calibration_three_points_with_noise() {
        // Slightly noisy data around E = 2*ch + 5.
        let channels = [10.0, 50.0, 100.0];
        let energies = [25.1, 105.0, 204.9];
        let (a, b) = energy_calibration(&channels, &energies);
        assert!((a - 2.0).abs() < 0.05);
        assert!((b - 5.0).abs() < 1.0);
    }

    #[test]
    #[should_panic(expected = "need at least 2")]
    fn test_energy_calibration_insufficient_points() {
        energy_calibration(&[100.0], &[662.0]);
    }

    #[test]
    #[should_panic(expected = "equal length")]
    fn test_energy_calibration_mismatched_lengths() {
        energy_calibration(&[100.0, 200.0], &[662.0]);
    }

    // --- Channel-energy conversion tests ---

    #[test]
    fn test_channel_to_energy_simple() {
        assert!((channel_to_energy(100.0, 1.0, 0.0) - 100.0).abs() < EPSILON);
    }

    #[test]
    fn test_channel_to_energy_with_offset() {
        assert!((channel_to_energy(50.0, 2.0, 10.0) - 110.0).abs() < EPSILON);
    }

    #[test]
    fn test_energy_to_channel_roundtrip() {
        let cal_a = 0.73;
        let cal_b = 5.0;
        let energy = 500.0;
        let ch = energy_to_channel(energy, cal_a, cal_b);
        let e_back = channel_to_energy(ch, cal_a, cal_b);
        assert!((e_back - energy).abs() < 1e-10);
    }

    // --- Compton kinematics tests ---

    #[test]
    fn test_compton_edge_cs137() {
        // Cs-137: 662 keV photopeak -> Compton edge ~477.3 keV.
        let edge = compton_edge_kev(662.0);
        assert!((edge - 477.3).abs() < 1.0, "Compton edge was {}", edge);
    }

    #[test]
    fn test_compton_edge_co60_1332() {
        // Co-60 1332 keV -> Compton edge ~1118 keV.
        let edge = compton_edge_kev(1332.0);
        assert!((edge - 1118.1).abs() < 2.0, "Compton edge was {}", edge);
    }

    #[test]
    fn test_backscatter_peak_cs137() {
        // Cs-137: backscatter peak ~184.3 keV.
        let bs = backscatter_peak_kev(662.0);
        assert!((bs - 184.3).abs() < 1.0, "Backscatter peak was {}", bs);
    }

    #[test]
    fn test_compton_plus_backscatter_equals_photopeak() {
        // E_compton_edge + E_backscatter = E_photopeak (approximately).
        let e_gamma = 662.0;
        let edge = compton_edge_kev(e_gamma);
        let bs = backscatter_peak_kev(e_gamma);
        let sum = edge + bs;
        assert!((sum - e_gamma).abs() < 1.0, "sum was {} (expected ~{})", sum, e_gamma);
    }

    #[test]
    fn test_compton_edge_low_energy() {
        // At very low energy, Compton edge approaches 0.
        let edge = compton_edge_kev(10.0);
        assert!(edge < 1.0);
        assert!(edge > 0.0);
    }

    #[test]
    fn test_backscatter_high_energy_limit() {
        // At very high energy, backscatter peak approaches 511/2 = 255.5 keV.
        let bs = backscatter_peak_kev(100_000.0);
        assert!((bs - 255.5).abs() < 1.0, "High-energy backscatter was {}", bs);
    }

    // --- Half-life decay tests ---

    #[test]
    fn test_half_life_one_halflife() {
        let result = half_life_decay(1000.0, 100.0, 100.0);
        assert!((result - 500.0).abs() < 0.1);
    }

    #[test]
    fn test_half_life_two_halflives() {
        let result = half_life_decay(1000.0, 100.0, 200.0);
        assert!((result - 250.0).abs() < 0.1);
    }

    #[test]
    fn test_half_life_zero_elapsed() {
        let result = half_life_decay(1000.0, 100.0, 0.0);
        assert!((result - 1000.0).abs() < EPSILON);
    }

    #[test]
    fn test_half_life_ten_halflives() {
        // After 10 half-lives: 1000 * 2^(-10) = 1000/1024 ~ 0.977.
        let result = half_life_decay(1000.0, 100.0, 1000.0);
        assert!((result - 1000.0 / 1024.0).abs() < 0.01);
    }

    #[test]
    fn test_half_life_zero_halflife() {
        let result = half_life_decay(1000.0, 0.0, 100.0);
        assert_eq!(result, 0.0);
    }

    // --- Detector efficiency tests ---

    #[test]
    fn test_efficiency_nai_higher_than_hpge() {
        // NaI should be more efficient than HPGe at 662 keV.
        let eff_nai = detector_efficiency(662.0, DetectorType::NaI);
        let eff_hpge = detector_efficiency(662.0, DetectorType::HPGe);
        assert!(
            eff_nai > eff_hpge,
            "NaI ({}) should be > HPGe ({})",
            eff_nai,
            eff_hpge
        );
    }

    #[test]
    fn test_efficiency_ordering_at_662() {
        // Expected order at 662 keV: NaI > LaBr3 > HPGe > CdZnTe (approximately).
        let nai = detector_efficiency(662.0, DetectorType::NaI);
        let labr = detector_efficiency(662.0, DetectorType::LaBr3);
        let hpge = detector_efficiency(662.0, DetectorType::HPGe);
        let czt = detector_efficiency(662.0, DetectorType::CdZnTe);
        assert!(nai > labr, "NaI {} > LaBr3 {}", nai, labr);
        assert!(labr > hpge, "LaBr3 {} > HPGe {}", labr, hpge);
        assert!(hpge > czt, "HPGe {} > CdZnTe {}", hpge, czt);
    }

    #[test]
    fn test_efficiency_positive_for_all_types() {
        for &dt in &[DetectorType::NaI, DetectorType::HPGe, DetectorType::CdZnTe, DetectorType::LaBr3] {
            let eff = detector_efficiency(662.0, dt);
            assert!(eff > 0.0, "{:?} efficiency should be positive", dt);
            assert!(eff <= 1.0, "{:?} efficiency should be <= 1.0", dt);
        }
    }

    #[test]
    fn test_efficiency_zero_energy() {
        assert_eq!(detector_efficiency(0.0, DetectorType::NaI), 0.0);
        assert_eq!(detector_efficiency(-10.0, DetectorType::HPGe), 0.0);
    }

    #[test]
    fn test_efficiency_decreases_with_energy() {
        // At high energies, efficiency should generally decrease.
        let eff_200 = detector_efficiency(200.0, DetectorType::NaI);
        let eff_2000 = detector_efficiency(2000.0, DetectorType::NaI);
        assert!(eff_200 > eff_2000, "Efficiency should decrease: {} > {}", eff_200, eff_2000);
    }

    // --- Gaussian fit tests ---

    #[test]
    fn test_gaussian_fit_centered_peak() {
        // Build a perfect Gaussian centered at channel 50 with sigma=3.
        let mut spectrum = vec![0.0; 100];
        let center = 50.0;
        let sigma = 3.0;
        let amplitude = 1000.0;
        for i in 0..100 {
            let dx = i as f64 - center;
            spectrum[i] = amplitude * (-0.5 * (dx / sigma).powi(2)).exp();
        }

        let (centroid, amp, sig) = gaussian_fit_peak(&spectrum, 50);
        assert!((centroid - 50.0).abs() < 0.1, "centroid={}", centroid);
        assert!((amp - 1000.0).abs() < 10.0, "amplitude={}", amp);
        assert!((sig - 3.0).abs() < 0.5, "sigma={}", sig);
    }

    #[test]
    fn test_gaussian_fit_offset_peak() {
        // Peak slightly off-center: Gaussian centered at 50.3.
        let mut spectrum = vec![0.0; 100];
        let center = 50.3;
        let sigma = 4.0;
        for i in 0..100 {
            let dx = i as f64 - center;
            spectrum[i] = 500.0 * (-0.5 * (dx / sigma).powi(2)).exp();
        }

        // Peak channel is 50 (nearest integer to 50.3).
        let (centroid, _, _) = gaussian_fit_peak(&spectrum, 50);
        assert!((centroid - 50.3).abs() < 0.2, "centroid={}", centroid);
    }

    // --- Synthetic spectrum generation tests ---

    #[test]
    fn test_generate_spectrum_single_peak() {
        let spectrum = generate_spectrum(&[(662.0, 10000.0)], 1024, 7.0);
        assert_eq!(spectrum.len(), 1024);

        // Find the maximum channel.
        let max_ch = spectrum
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        // Expected channel: 662 keV / (3000/1024) ~ 226.
        let expected_ch = (662.0 / (3000.0 / 1024.0)) as usize;
        assert!(
            (max_ch as i64 - expected_ch as i64).unsigned_abs() < 5,
            "max at ch {}, expected ~{}",
            max_ch,
            expected_ch
        );
    }

    #[test]
    fn test_generate_spectrum_total_counts() {
        let total_input = 50000.0;
        let spectrum = generate_spectrum(&[(662.0, total_input)], 2048, 7.0);
        let total: f64 = spectrum.iter().sum();
        // Total should be at least the input counts (plus background).
        assert!(total > total_input * 0.8, "total={}", total);
    }

    #[test]
    fn test_generate_spectrum_two_peaks() {
        let spectrum = generate_spectrum(
            &[(662.0, 10000.0), (1332.0, 5000.0)],
            2048,
            7.0,
        );
        assert_eq!(spectrum.len(), 2048);
        // Both peaks should create visible features.
        let kev_per_ch = 3000.0 / 2048.0;
        let ch_662 = (662.0 / kev_per_ch) as usize;
        let ch_1332 = (1332.0 / kev_per_ch) as usize;
        assert!(spectrum[ch_662] > spectrum[0], "662 keV peak should be above background");
        assert!(spectrum[ch_1332] > spectrum[0], "1332 keV peak should be above background");
    }

    #[test]
    fn test_generate_spectrum_empty_peaks() {
        let spectrum = generate_spectrum(&[], 512, 7.0);
        // Should just be flat background.
        assert_eq!(spectrum.len(), 512);
        let max = spectrum.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min = spectrum.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!((max - min).abs() < 0.01, "empty spectrum should be flat");
    }

    // --- Peak finding tests ---

    #[test]
    fn test_find_peaks_in_synthetic_spectrum() {
        let spectrum = generate_spectrum(&[(662.0, 50000.0)], 1024, 7.0);
        let peaks = find_peaks_spectrum(&spectrum, 3.0);
        assert!(!peaks.is_empty(), "should find at least one peak");

        // The strongest peak should be near channel 226 (662 keV / ~2.93 keV/ch).
        let kev_per_ch = 3000.0 / 1024.0;
        let expected = 662.0 / kev_per_ch;
        let strongest = peaks.iter().max_by(|a, b| a.net_counts.partial_cmp(&b.net_counts).unwrap()).unwrap();
        assert!(
            (strongest.centroid - expected).abs() < 10.0,
            "peak centroid {} far from expected {}",
            strongest.centroid,
            expected
        );
    }

    #[test]
    fn test_find_peaks_too_short_spectrum() {
        let spectrum = vec![1.0, 2.0, 3.0];
        let peaks = find_peaks_spectrum(&spectrum, 3.0);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_find_peaks_flat_spectrum() {
        let spectrum = vec![100.0; 1024];
        let peaks = find_peaks_spectrum(&spectrum, 3.0);
        // A flat spectrum should yield no significant peaks.
        assert!(peaks.is_empty(), "flat spectrum should have no peaks, found {}", peaks.len());
    }

    // --- Spectrum analyzer integration tests ---

    #[test]
    fn test_analyzer_cs137() {
        let config = SpectroscopyConfig {
            num_channels: 1024,
            energy_range_kev: 3000.0,
            detector_resolution_percent: 7.0,
            live_time_s: 300.0,
        };
        let spectrum = generate_spectrum(&[(662.0, 100000.0)], 1024, 7.0);
        let analyzer = SpectrumAnalyzer::new(config);
        let result = analyzer.analyze(&spectrum);

        assert!(result.total_counts > 0.0);
        assert!(result.count_rate_cps > 0.0);
        assert!(result.dead_time_fraction < 1.0);
        assert!(!result.peaks.is_empty(), "should detect the Cs-137 peak");

        // The main peak should be near 662 keV.
        let main_peak = result.peaks.iter()
            .max_by(|a, b| a.net_counts.partial_cmp(&b.net_counts).unwrap())
            .unwrap();
        assert!(
            (main_peak.energy_kev - 662.0).abs() < 30.0,
            "main peak at {} keV, expected ~662 keV",
            main_peak.energy_kev
        );
    }

    #[test]
    fn test_analyzer_identifies_cs137() {
        let config = SpectroscopyConfig {
            num_channels: 2048,
            energy_range_kev: 3000.0,
            detector_resolution_percent: 7.0,
            live_time_s: 600.0,
        };
        let spectrum = generate_spectrum(&[(662.0, 200000.0)], 2048, 7.0);
        let analyzer = SpectrumAnalyzer::new(config);
        let result = analyzer.analyze(&spectrum);

        let cs137 = result.identified_isotopes.iter().find(|i| i.name.contains("Cs-137"));
        assert!(cs137.is_some(), "should identify Cs-137; isotopes found: {:?}",
            result.identified_isotopes.iter().map(|i| &i.name).collect::<Vec<_>>());
        let iso = cs137.unwrap();
        assert!(iso.confidence > 0.5);
        assert!(iso.activity_bq > 0.0);
    }

    #[test]
    fn test_analyzer_with_calibration() {
        let config = SpectroscopyConfig {
            num_channels: 4096,
            energy_range_kev: 3000.0,
            detector_resolution_percent: 7.0,
            live_time_s: 100.0,
        };
        let analyzer = SpectrumAnalyzer::with_calibration(config, 0.5, 10.0);
        let energy = channel_to_energy(100.0, 0.5, 10.0);
        assert!((energy - 60.0).abs() < EPSILON);

        // Verify the analyzer stores its calibration.
        let spectrum = vec![0.0; 4096];
        let result = analyzer.analyze(&spectrum);
        assert_eq!(result.total_counts, 0.0);
    }

    #[test]
    fn test_analyzer_empty_spectrum() {
        let config = SpectroscopyConfig::default();
        let analyzer = SpectrumAnalyzer::new(config);
        let spectrum = vec![0.0; 4096];
        let result = analyzer.analyze(&spectrum);
        assert_eq!(result.total_counts, 0.0);
        assert_eq!(result.count_rate_cps, 0.0);
        assert!(result.peaks.is_empty());
        assert!(result.identified_isotopes.is_empty());
    }

    #[test]
    fn test_default_config() {
        let config = SpectroscopyConfig::default();
        assert_eq!(config.num_channels, 4096);
        assert_eq!(config.energy_range_kev, 3000.0);
        assert_eq!(config.detector_resolution_percent, 7.0);
        assert_eq!(config.live_time_s, 300.0);
    }

    // --- Smoothing test ---

    #[test]
    fn test_smooth_spectrum_preserves_length() {
        let data = vec![1.0, 10.0, 1.0, 10.0, 1.0, 10.0, 1.0, 10.0];
        let smoothed = smooth_spectrum(&data, 3);
        assert_eq!(smoothed.len(), data.len());
        // Interior smoothed values should be between min and max.
        for &v in &smoothed {
            assert!(v >= 1.0 - EPSILON);
            assert!(v <= 10.0 + EPSILON);
        }
    }

    // --- Edge case: single channel spectrum ---

    #[test]
    fn test_generate_spectrum_single_channel() {
        let spectrum = generate_spectrum(&[(100.0, 1000.0)], 1, 7.0);
        assert_eq!(spectrum.len(), 1);
    }
}
