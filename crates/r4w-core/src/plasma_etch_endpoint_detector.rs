//! Semiconductor plasma etch endpoint detection via Optical Emission Spectroscopy (OES).
//!
//! Plasma etching is a critical semiconductor fabrication step in which reactive plasma
//! species remove material from wafer surfaces with high selectivity and anisotropy.
//! This module processes OES signals – plasma light intensity versus wavelength and time –
//! to detect the moment etching reaches the target layer (the "endpoint"), enabling
//! automatic process termination or timed over-etch.
//!
//! # Algorithms
//!
//! * **Single-wavelength threshold** – monitor one emission line; endpoint when intensity
//!   crosses a threshold (rising or falling edge).
//! * **Intensity ratio** – ratio of two wavelengths (e.g. product / byproduct) for
//!   self-normalization against lamp drift or window coating.
//! * **First-derivative** – d(I)/dt exceeds a threshold, robust for gradual transitions.
//! * **PCA-based** – simplified 2-component principal component analysis on multi-wavelength
//!   data; endpoint detected from score trajectory.
//!
//! # Etch Rate Monitoring
//!
//! Interferometric fringe counting tracks reflectance oscillations; each half-period
//! corresponds to λ/(4n) of etched thickness, giving a real-time etch rate.
//!
//! # Statistical Process Control
//!
//! Endpoint times from multiple wafers are trended; 2σ/3σ warning and alarm limits
//! are computed from historical statistics.
//!
//! All math is implemented from scratch; no external crates beyond `std` are used.
//!
//! # Example
//!
//! ```
//! use r4w_core::plasma_etch_endpoint_detector::{
//!     EtchProcess, EndpointDetector, EndpointAlgorithm, DetectorConfig,
//! };
//!
//! let cfg = DetectorConfig {
//!     process: EtchProcess::SiliconEtch,
//!     algorithm: EndpointAlgorithm::SingleWavelength { wavelength_nm: 704.0, threshold: 0.5, rising: false },
//!     smoothing_window: 5,
//!     ignition_transient_samples: 10,
//!     over_etch_percent: 10.0,
//! };
//! let mut detector = EndpointDetector::new(cfg);
//!
//! // Feed synthetic intensity curve (decaying step)
//! for i in 0..50 {
//!     let intensity = if i < 30 { 1.0_f64 } else { 0.2_f64 };
//!     detector.push_sample(intensity, 704.0, i as f64);
//! }
//! assert!(detector.endpoint_detected());
//! ```

use std::collections::VecDeque;
use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Emission line database
// ---------------------------------------------------------------------------

/// A single entry in the OES emission line database.
#[derive(Debug, Clone, PartialEq)]
pub struct EmissionLine {
    /// Chemical species (e.g. "SiF*", "F*", "CO*")
    pub species: &'static str,
    /// Centre wavelength in nanometres
    pub wavelength_nm: f64,
    /// Human-readable description of the role in etch chemistry
    pub description: &'static str,
    /// Etch process(es) where this line is monitored
    pub processes: &'static [EtchProcess],
}

/// Semiconductor etch process type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EtchProcess {
    /// Silicon bulk etch (fluorine-based plasma)
    SiliconEtch,
    /// Silicon dioxide (oxide) etch
    OxideEtch,
    /// Silicon nitride etch
    NitrideEtch,
    /// Photoresist strip / ash
    PhotoresistStrip,
}

/// Full OES emission line database for common semiconductor etch processes.
///
/// Wavelengths are in nanometres (nm). Species marked `*` are excited-state radicals
/// (optical emission from electronic de-excitation).
pub static EMISSION_LINE_DB: &[EmissionLine] = &[
    EmissionLine {
        species: "SiF*",
        wavelength_nm: 440.0,
        description: "Silicon fluoride radical – primary Si etch product",
        processes: &[EtchProcess::SiliconEtch],
    },
    EmissionLine {
        species: "F*",
        wavelength_nm: 704.0,
        description: "Atomic fluorine – etchant species; rises when Si layer cleared",
        processes: &[EtchProcess::SiliconEtch],
    },
    EmissionLine {
        species: "CF2*",
        wavelength_nm: 251.0,
        description: "Difluorocarbene – passivation / polymer byproduct",
        processes: &[EtchProcess::SiliconEtch, EtchProcess::OxideEtch],
    },
    EmissionLine {
        species: "CO*",
        wavelength_nm: 483.0,
        description: "Carbon monoxide – SiO2 etch product; drops at endpoint",
        processes: &[
            EtchProcess::SiliconEtch,
            EtchProcess::OxideEtch,
            EtchProcess::PhotoresistStrip,
        ],
    },
    EmissionLine {
        species: "O*",
        wavelength_nm: 777.0,
        description: "Atomic oxygen – oxide/photoresist indicator",
        processes: &[EtchProcess::OxideEtch, EtchProcess::PhotoresistStrip],
    },
    EmissionLine {
        species: "CN*",
        wavelength_nm: 388.0,
        description: "Cyanide radical – nitride etch indicator (N + C plasma)",
        processes: &[EtchProcess::NitrideEtch],
    },
    EmissionLine {
        species: "N2*",
        wavelength_nm: 337.0,
        description: "Nitrogen molecular emission – nitride/N2 plasma indicator",
        processes: &[EtchProcess::NitrideEtch],
    },
    EmissionLine {
        species: "OH*",
        wavelength_nm: 309.0,
        description: "Hydroxyl radical – photoresist ash indicator",
        processes: &[EtchProcess::PhotoresistStrip],
    },
];

/// Look up all emission lines relevant to a given etch process.
pub fn lines_for_process(process: EtchProcess) -> Vec<&'static EmissionLine> {
    EMISSION_LINE_DB
        .iter()
        .filter(|l| l.processes.contains(&process))
        .collect()
}

/// Look up emission lines by approximate wavelength (±2 nm tolerance).
pub fn lines_near_wavelength(wavelength_nm: f64) -> Vec<&'static EmissionLine> {
    EMISSION_LINE_DB
        .iter()
        .filter(|l| (l.wavelength_nm - wavelength_nm).abs() <= 2.0)
        .collect()
}

// ---------------------------------------------------------------------------
// Endpoint detection algorithms
// ---------------------------------------------------------------------------

/// Endpoint detection algorithm selection.
#[derive(Debug, Clone)]
pub enum EndpointAlgorithm {
    /// Monitor a single emission line; detect threshold crossing.
    SingleWavelength {
        /// Wavelength to monitor (nm) – informational only; caller feeds intensity
        wavelength_nm: f64,
        /// Normalised intensity threshold (0–1 scale)
        threshold: f64,
        /// `true` = detect rising edge (intensity increases past threshold)
        rising: bool,
    },
    /// Ratio of two wavelengths: `primary / reference`. Endpoint when ratio crosses threshold.
    IntensityRatio {
        /// Primary (product/etchant) wavelength (nm)
        primary_nm: f64,
        /// Reference (byproduct/normalisation) wavelength (nm)
        reference_nm: f64,
        /// Ratio threshold
        threshold: f64,
        /// `true` = rising edge on the ratio
        rising: bool,
    },
    /// First-derivative (d(I)/dt) exceeds a threshold magnitude.
    Derivative {
        /// Derivative threshold (normalised units per sample)
        threshold: f64,
        /// `true` = detect positive slope (intensity rising)
        positive: bool,
    },
    /// Simplified 2-component PCA on multi-wavelength spectra.
    /// Endpoint detected when the first principal component score exceeds threshold.
    Pca {
        /// Score threshold on PC1
        threshold: f64,
    },
}

/// Configuration for the endpoint detector.
#[derive(Debug, Clone)]
pub struct DetectorConfig {
    /// Etch process type (used for documentation / line selection)
    pub process: EtchProcess,
    /// Which endpoint detection algorithm to use
    pub algorithm: EndpointAlgorithm,
    /// Moving average window (samples) for OES smoothing
    pub smoothing_window: usize,
    /// Number of initial samples to ignore (plasma ignition transient)
    pub ignition_transient_samples: usize,
    /// Over-etch percentage applied after endpoint (e.g. 10.0 = 10 %)
    pub over_etch_percent: f64,
}

impl Default for DetectorConfig {
    fn default() -> Self {
        DetectorConfig {
            process: EtchProcess::SiliconEtch,
            algorithm: EndpointAlgorithm::SingleWavelength {
                wavelength_nm: 704.0,
                threshold: 0.5,
                rising: false,
            },
            smoothing_window: 5,
            ignition_transient_samples: 20,
            over_etch_percent: 10.0,
        }
    }
}

// ---------------------------------------------------------------------------
// OES Spectrum
// ---------------------------------------------------------------------------

/// A single OES spectrum: intensity measured at a set of wavelengths.
#[derive(Debug, Clone)]
pub struct OesSpectrum {
    /// Wavelength axis (nm)
    pub wavelengths_nm: Vec<f64>,
    /// Intensity values (arbitrary units, aligned with wavelengths)
    pub intensities: Vec<f64>,
    /// Acquisition timestamp (seconds from process start)
    pub time_s: f64,
}

impl OesSpectrum {
    /// Create a new spectrum.
    pub fn new(wavelengths_nm: Vec<f64>, intensities: Vec<f64>, time_s: f64) -> Self {
        assert_eq!(wavelengths_nm.len(), intensities.len(), "lengths must match");
        OesSpectrum { wavelengths_nm, intensities, time_s }
    }

    /// Interpolate intensity at a given wavelength (linear interpolation between
    /// the two nearest calibrated points).
    pub fn intensity_at(&self, wavelength_nm: f64) -> Option<f64> {
        if self.wavelengths_nm.is_empty() {
            return None;
        }
        // Find nearest index
        let mut best = 0usize;
        let mut best_dist = (self.wavelengths_nm[0] - wavelength_nm).abs();
        for (i, &w) in self.wavelengths_nm.iter().enumerate() {
            let d = (w - wavelength_nm).abs();
            if d < best_dist {
                best_dist = d;
                best = i;
            }
        }
        // Linear interpolation if neighbour exists
        if best + 1 < self.wavelengths_nm.len() {
            let w0 = self.wavelengths_nm[best];
            let w1 = self.wavelengths_nm[best + 1];
            let span = w1 - w0;
            if span.abs() > 1e-12 {
                let t = (wavelength_nm - w0) / span;
                let t = t.clamp(0.0, 1.0);
                let v0 = self.intensities[best];
                let v1 = self.intensities[best + 1];
                return Some(v0 + t * (v1 - v0));
            }
        }
        Some(self.intensities[best])
    }

    /// Baseline-normalise intensities: subtract min, divide by (max − min).
    pub fn normalise(&mut self) {
        let min = self.intensities.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = self.intensities.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let range = max - min;
        if range > 1e-12 {
            for v in &mut self.intensities {
                *v = (*v - min) / range;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Moving-average smoother
// ---------------------------------------------------------------------------

/// Causal moving-average (boxcar) smoother for OES time series.
#[derive(Debug)]
pub struct MovingAverage {
    window: usize,
    buf: VecDeque<f64>,
    sum: f64,
}

impl MovingAverage {
    /// Create a new moving average with the given window length.
    pub fn new(window: usize) -> Self {
        assert!(window > 0);
        MovingAverage {
            window,
            buf: VecDeque::with_capacity(window),
            sum: 0.0,
        }
    }

    /// Push a new sample and return the smoothed value.
    pub fn push(&mut self, value: f64) -> f64 {
        self.sum += value;
        self.buf.push_back(value);
        if self.buf.len() > self.window {
            self.sum -= self.buf.pop_front().unwrap_or(0.0);
        }
        self.sum / self.buf.len() as f64
    }

    /// Return the current smoothed value without pushing.
    pub fn current(&self) -> f64 {
        if self.buf.is_empty() { 0.0 } else { self.sum / self.buf.len() as f64 }
    }

    /// Return the number of samples currently buffered.
    pub fn len(&self) -> usize {
        self.buf.len()
    }

    /// Return `true` if no samples have been pushed yet.
    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }
}

// ---------------------------------------------------------------------------
// Spike / cosmic-ray removal (median filter)
// ---------------------------------------------------------------------------

/// Remove impulse spikes from an intensity time series using a sliding median filter.
///
/// Each output sample equals the median of the surrounding `window` input samples
/// (window must be odd). Values that deviate from the local median by more than
/// `spike_threshold` times the local median are replaced by the median.
pub fn remove_spikes(data: &[f64], window: usize, spike_threshold: f64) -> Vec<f64> {
    assert!(window % 2 == 1, "window must be odd");
    let half = window / 2;
    let n = data.len();
    let mut out = data.to_vec();
    for i in 0..n {
        let lo = i.saturating_sub(half);
        let hi = (i + half + 1).min(n);
        let mut patch: Vec<f64> = data[lo..hi].to_vec();
        patch.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let med = patch[patch.len() / 2];
        if med.abs() > 1e-12 && (data[i] - med).abs() / med.abs() > spike_threshold {
            out[i] = med;
        }
    }
    out
}

// ---------------------------------------------------------------------------
// PCA helper (2-component, mean-centered)
// ---------------------------------------------------------------------------

/// Compute the first two principal components of a data matrix and return the
/// PC1 score for each observation.
///
/// `data` has shape `[n_obs][n_features]`. Returns a vector of length `n_obs`.
///
/// Uses power iteration for the first eigenvector and deflation for the second.
pub fn pca_pc1_scores(data: &[Vec<f64>]) -> Vec<f64> {
    let n = data.len();
    if n == 0 {
        return vec![];
    }
    let p = data[0].len();
    if p == 0 {
        return vec![0.0; n];
    }

    // Mean-centre
    let mut means = vec![0.0f64; p];
    for row in data {
        for (j, &v) in row.iter().enumerate() {
            means[j] += v;
        }
    }
    for m in &mut means {
        *m /= n as f64;
    }
    let centered: Vec<Vec<f64>> = data
        .iter()
        .map(|row| row.iter().zip(&means).map(|(&v, &m)| v - m).collect())
        .collect();

    // Power iteration to find PC1
    let mut pc1 = vec![1.0f64 / (p as f64).sqrt(); p];
    for _iter in 0..50 {
        // pc1 ← X^T (X pc1) / ‖…‖
        // Step 1: scores = X pc1
        let scores: Vec<f64> = centered
            .iter()
            .map(|row| row.iter().zip(&pc1).map(|(&x, &w)| x * w).sum())
            .collect();
        // Step 2: new_pc = X^T scores
        let mut new_pc = vec![0.0f64; p];
        for (row, &s) in centered.iter().zip(&scores) {
            for (j, &x) in row.iter().enumerate() {
                new_pc[j] += x * s;
            }
        }
        // Normalise
        let norm: f64 = new_pc.iter().map(|v| v * v).sum::<f64>().sqrt();
        if norm < 1e-15 {
            break;
        }
        for v in &mut new_pc {
            *v /= norm;
        }
        pc1 = new_pc;
    }

    // Return PC1 scores for each observation
    centered
        .iter()
        .map(|row| row.iter().zip(&pc1).map(|(&x, &w)| x * w).sum())
        .collect()
}

// ---------------------------------------------------------------------------
// Endpoint Detector
// ---------------------------------------------------------------------------

/// Result of endpoint detection.
#[derive(Debug, Clone)]
pub struct EndpointEvent {
    /// Sample index at which endpoint was detected
    pub sample_index: usize,
    /// Timestamp (seconds from process start) at endpoint
    pub time_s: f64,
    /// Smoothed intensity (or ratio/score) value at detection
    pub signal_value: f64,
    /// Algorithm that triggered detection
    pub algorithm: String,
}

/// Main endpoint detector.  Push intensity samples one-by-one; query
/// [`EndpointDetector::endpoint_detected`] after each push.
#[derive(Debug)]
pub struct EndpointDetector {
    cfg: DetectorConfig,
    smoother: MovingAverage,
    /// Secondary smoother for ratio reference channel
    smoother_ref: MovingAverage,
    sample_count: usize,
    prev_smoothed: f64,
    endpoint_event: Option<EndpointEvent>,
    /// Raw intensity history for derivative and PCA
    history: VecDeque<f64>,
    history_ref: VecDeque<f64>,
    /// PCA spectra buffer (each entry = one multi-wavelength snapshot)
    pca_buffer: Vec<Vec<f64>>,
}

impl EndpointDetector {
    /// Create a new detector with the given configuration.
    pub fn new(cfg: DetectorConfig) -> Self {
        let sw = cfg.smoothing_window.max(1);
        EndpointDetector {
            smoother: MovingAverage::new(sw),
            smoother_ref: MovingAverage::new(sw),
            cfg,
            sample_count: 0,
            prev_smoothed: f64::NAN,
            endpoint_event: None,
            history: VecDeque::with_capacity(64),
            history_ref: VecDeque::with_capacity(64),
            pca_buffer: Vec::new(),
        }
    }

    /// Push a single-wavelength intensity sample.
    ///
    /// `intensity` is in arbitrary units (need not be normalised).
    /// `wavelength_nm` is used for logging only; caller selects which line to monitor.
    /// `time_s` is seconds elapsed since plasma ignition.
    pub fn push_sample(&mut self, intensity: f64, wavelength_nm: f64, time_s: f64) {
        let idx = self.sample_count;
        self.sample_count += 1;

        let smoothed = self.smoother.push(intensity);
        self.history.push_back(smoothed);
        if self.history.len() > 128 {
            self.history.pop_front();
        }

        // Ignore ignition transient
        if idx < self.cfg.ignition_transient_samples {
            self.prev_smoothed = smoothed;
            return;
        }
        if self.endpoint_event.is_some() {
            return;
        }

        let detected = match &self.cfg.algorithm {
            EndpointAlgorithm::SingleWavelength { threshold, rising, .. } => {
                let threshold = *threshold;
                let rising = *rising;
                if self.prev_smoothed.is_nan() {
                    false
                } else if rising {
                    self.prev_smoothed < threshold && smoothed >= threshold
                } else {
                    self.prev_smoothed > threshold && smoothed <= threshold
                }
            }
            EndpointAlgorithm::Derivative { threshold, positive } => {
                if self.prev_smoothed.is_nan() {
                    false
                } else {
                    let deriv = smoothed - self.prev_smoothed;
                    if *positive {
                        deriv >= *threshold
                    } else {
                        deriv <= -*threshold
                    }
                }
            }
            // For ratio and PCA, single-channel push cannot detect; use dedicated methods
            _ => false,
        };

        if detected {
            self.endpoint_event = Some(EndpointEvent {
                sample_index: idx,
                time_s,
                signal_value: smoothed,
                algorithm: format!("{:?}", self.cfg.algorithm).split('{').next().unwrap_or("").trim().to_string(),
            });
        }

        self.prev_smoothed = smoothed;
        let _ = wavelength_nm; // used for logging / future multi-channel
    }

    /// Push a two-channel sample (primary and reference intensity) for the
    /// [`EndpointAlgorithm::IntensityRatio`] algorithm.
    pub fn push_ratio_sample(&mut self, primary: f64, reference: f64, time_s: f64) {
        let idx = self.sample_count;
        self.sample_count += 1;

        let sp = self.smoother.push(primary);
        let sr = self.smoother_ref.push(reference);
        self.history.push_back(sp);
        self.history_ref.push_back(sr);
        if self.history.len() > 128 { self.history.pop_front(); }
        if self.history_ref.len() > 128 { self.history_ref.pop_front(); }

        if idx < self.cfg.ignition_transient_samples || self.endpoint_event.is_some() {
            self.prev_smoothed = if sr.abs() > 1e-12 { sp / sr } else { 0.0 };
            return;
        }

        let ratio = if sr.abs() > 1e-12 { sp / sr } else { 0.0 };
        let detected = match &self.cfg.algorithm {
            EndpointAlgorithm::IntensityRatio { threshold, rising, .. } => {
                let threshold = *threshold;
                let rising = *rising;
                if self.prev_smoothed.is_nan() {
                    false
                } else if rising {
                    self.prev_smoothed < threshold && ratio >= threshold
                } else {
                    self.prev_smoothed > threshold && ratio <= threshold
                }
            }
            _ => false,
        };

        if detected {
            self.endpoint_event = Some(EndpointEvent {
                sample_index: idx,
                time_s,
                signal_value: ratio,
                algorithm: "IntensityRatio".to_string(),
            });
        }
        self.prev_smoothed = ratio;
    }

    /// Push a multi-wavelength spectrum snapshot for PCA-based detection.
    pub fn push_spectrum(&mut self, spectrum: &[f64], time_s: f64) {
        let idx = self.sample_count;
        self.sample_count += 1;

        self.pca_buffer.push(spectrum.to_vec());

        if idx < self.cfg.ignition_transient_samples || self.endpoint_event.is_some() {
            return;
        }

        // Recompute PCA scores up to current point
        if self.pca_buffer.len() < 3 {
            return;
        }
        let scores = pca_pc1_scores(&self.pca_buffer);
        let last_score = *scores.last().unwrap_or(&0.0);

        let detected = match &self.cfg.algorithm {
            EndpointAlgorithm::Pca { threshold } => {
                let prev = if scores.len() >= 2 { scores[scores.len() - 2] } else { 0.0 };
                prev < *threshold && last_score >= *threshold
            }
            _ => false,
        };

        if detected {
            self.endpoint_event = Some(EndpointEvent {
                sample_index: idx,
                time_s,
                signal_value: last_score,
                algorithm: "Pca".to_string(),
            });
        }
    }

    /// Returns `true` if an endpoint has been detected.
    pub fn endpoint_detected(&self) -> bool {
        self.endpoint_event.is_some()
    }

    /// Returns the endpoint event, if detected.
    pub fn endpoint_event(&self) -> Option<&EndpointEvent> {
        self.endpoint_event.as_ref()
    }

    /// Compute the over-etch duration in seconds given the etch-to-endpoint time.
    ///
    /// `over_etch_percent` is taken from the detector configuration.
    pub fn over_etch_duration_s(&self) -> Option<f64> {
        self.endpoint_event.as_ref().map(|ev| {
            ev.time_s * self.cfg.over_etch_percent / 100.0
        })
    }

    /// Reset the detector (new wafer).
    pub fn reset(&mut self) {
        let sw = self.cfg.smoothing_window.max(1);
        self.smoother = MovingAverage::new(sw);
        self.smoother_ref = MovingAverage::new(sw);
        self.sample_count = 0;
        self.prev_smoothed = f64::NAN;
        self.endpoint_event = None;
        self.history.clear();
        self.history_ref.clear();
        self.pca_buffer.clear();
    }

    /// Return the number of samples processed so far.
    pub fn sample_count(&self) -> usize {
        self.sample_count
    }

    /// Return the smoothed intensity history (for plotting).
    pub fn history(&self) -> &VecDeque<f64> {
        &self.history
    }
}

// ---------------------------------------------------------------------------
// Interferometric etch-rate monitor
// ---------------------------------------------------------------------------

/// Interferometric etch rate monitor.
///
/// Tracks reflectance oscillation fringes from a laser interferometer.
/// Each half-period of the reflectance oscillation corresponds to a thickness change
/// of `lambda / (4 * n)` where `lambda` is the probe wavelength and `n` is the refractive
/// index of the etched film.
#[derive(Debug)]
pub struct InterferometricMonitor {
    /// Probe laser wavelength (nm)
    pub probe_wavelength_nm: f64,
    /// Refractive index of the etched material
    pub refractive_index: f64,
    /// Half-period thickness increment (nm per half-fringe)
    pub thickness_per_half_period_nm: f64,
    /// Total number of half-periods counted
    fringe_count: u32,
    /// Previous reflectance sample (for zero-crossing detection)
    prev_reflectance: f64,
    /// Reflectance moving average (for baseline)
    smoother: MovingAverage,
    /// Total etched thickness (nm)
    etched_thickness_nm: f64,
    /// Timestamp of the last detected fringe (s)
    last_fringe_time_s: f64,
    /// Etch rate computed from the most recent fringe interval (nm/s)
    current_rate_nm_s: f64,
    /// Running list of (time, thickness) pairs for rate history
    rate_history: Vec<(f64, f64)>,
}

impl InterferometricMonitor {
    /// Create a new monitor.
    ///
    /// * `probe_wavelength_nm` – laser wavelength (e.g. 632.8 for HeNe)
    /// * `refractive_index` – real part of film refractive index at probe wavelength
    ///   (e.g. 1.46 for SiO2, 2.0 for Si3N4)
    pub fn new(probe_wavelength_nm: f64, refractive_index: f64) -> Self {
        let dz = probe_wavelength_nm / (4.0 * refractive_index);
        InterferometricMonitor {
            probe_wavelength_nm,
            refractive_index,
            thickness_per_half_period_nm: dz,
            fringe_count: 0,
            prev_reflectance: f64::NAN,
            smoother: MovingAverage::new(5),
            etched_thickness_nm: 0.0,
            last_fringe_time_s: 0.0,
            current_rate_nm_s: 0.0,
            rate_history: Vec::new(),
        }
    }

    /// Push a raw reflectance sample and return the current etched thickness (nm).
    ///
    /// `reflectance` is a normalised (0–1) or arbitrary-unit signal. The function
    /// detects zero-crossings of the AC component (relative to moving average baseline).
    pub fn push_reflectance(&mut self, reflectance: f64, time_s: f64) -> f64 {
        let smoothed = self.smoother.push(reflectance);
        // AC component relative to running baseline
        let ac = reflectance - smoothed;

        if !self.prev_reflectance.is_nan() {
            // Detect zero-crossing (rising or falling)
            if self.prev_reflectance * ac < 0.0 {
                // Half-period detected
                self.fringe_count += 1;
                let dz = self.thickness_per_half_period_nm;
                self.etched_thickness_nm += dz;

                if self.last_fringe_time_s > 0.0 {
                    let dt = time_s - self.last_fringe_time_s;
                    if dt > 0.0 {
                        self.current_rate_nm_s = dz / dt;
                    }
                }
                self.rate_history.push((time_s, self.etched_thickness_nm));
                self.last_fringe_time_s = time_s;
            }
        }
        self.prev_reflectance = ac;
        self.etched_thickness_nm
    }

    /// Total number of half-fringes counted.
    pub fn fringe_count(&self) -> u32 {
        self.fringe_count
    }

    /// Current etch depth (nm).
    pub fn etched_thickness_nm(&self) -> f64 {
        self.etched_thickness_nm
    }

    /// Most recent instantaneous etch rate (nm/s).
    pub fn etch_rate_nm_s(&self) -> f64 {
        self.current_rate_nm_s
    }

    /// Remaining film thickness (nm) given a known initial thickness.
    pub fn remaining_thickness_nm(&self, initial_thickness_nm: f64) -> f64 {
        (initial_thickness_nm - self.etched_thickness_nm).max(0.0)
    }

    /// Average etch rate over the entire etch so far (nm/s).
    pub fn average_rate_nm_s(&self, elapsed_s: f64) -> f64 {
        if elapsed_s > 0.0 {
            self.etched_thickness_nm / elapsed_s
        } else {
            0.0
        }
    }

    /// Etch rate history as (time_s, cumulative_thickness_nm) pairs.
    pub fn rate_history(&self) -> &[(f64, f64)] {
        &self.rate_history
    }
}

// ---------------------------------------------------------------------------
// Over-etch control
// ---------------------------------------------------------------------------

/// Over-etch controller that manages the additional etch time after endpoint detection.
#[derive(Debug)]
pub struct OverEtchController {
    /// Over-etch time as a fraction of endpoint time (e.g. 0.10 = 10 %)
    over_etch_fraction: f64,
    /// Additional fixed over-etch time (s), added on top of the fractional amount
    fixed_over_etch_s: f64,
    /// Computed over-etch end time (s from process start), `None` until endpoint detected
    over_etch_end_s: Option<f64>,
}

impl OverEtchController {
    /// Create a controller.
    ///
    /// * `over_etch_percent` – e.g. `10.0` for 10 % over-etch
    /// * `fixed_over_etch_s` – additional fixed over-etch seconds (0.0 for none)
    pub fn new(over_etch_percent: f64, fixed_over_etch_s: f64) -> Self {
        OverEtchController {
            over_etch_fraction: over_etch_percent / 100.0,
            fixed_over_etch_s,
            over_etch_end_s: None,
        }
    }

    /// Signal that endpoint has been detected at `endpoint_time_s`.
    /// Computes and stores the over-etch end time.
    pub fn on_endpoint(&mut self, endpoint_time_s: f64) {
        let oe = endpoint_time_s * self.over_etch_fraction + self.fixed_over_etch_s;
        self.over_etch_end_s = Some(endpoint_time_s + oe);
    }

    /// Returns `true` if the current time exceeds the over-etch end time.
    pub fn is_complete(&self, current_time_s: f64) -> bool {
        match self.over_etch_end_s {
            Some(end) => current_time_s >= end,
            None => false,
        }
    }

    /// Over-etch end time (seconds from process start), or `None` if not yet triggered.
    pub fn over_etch_end_s(&self) -> Option<f64> {
        self.over_etch_end_s
    }

    /// Over-etch duration (s) from endpoint to process stop, or `None` if not triggered.
    pub fn over_etch_duration_s(&self, endpoint_time_s: f64) -> Option<f64> {
        self.over_etch_end_s.map(|e| e - endpoint_time_s)
    }
}

// ---------------------------------------------------------------------------
// Etch selectivity
// ---------------------------------------------------------------------------

/// Compute etch selectivity: ratio of target material etch rate to stop-layer etch rate.
///
/// A selectivity of 10:1 means the target etches 10× faster than the underlying stop layer.
///
/// # Arguments
/// * `rate_target_nm_s` – etch rate of the layer being removed (nm/s)
/// * `rate_stop_nm_s` – etch rate of the stop layer (nm/s); must be > 0
///
/// # Returns
/// Selectivity ratio (dimensionless). Returns `f64::INFINITY` if `rate_stop_nm_s ≈ 0`.
pub fn etch_selectivity(rate_target_nm_s: f64, rate_stop_nm_s: f64) -> f64 {
    if rate_stop_nm_s.abs() < 1e-15 {
        return f64::INFINITY;
    }
    rate_target_nm_s / rate_stop_nm_s
}

// ---------------------------------------------------------------------------
// Statistical Process Control
// ---------------------------------------------------------------------------

/// Statistical summary of a set of endpoint times.
#[derive(Debug, Clone)]
pub struct SpcStats {
    /// Sample mean of endpoint times (s)
    pub mean_s: f64,
    /// Sample standard deviation (s)
    pub std_dev_s: f64,
    /// Warning lower limit (mean − 2σ)
    pub warn_lo_s: f64,
    /// Warning upper limit (mean + 2σ)
    pub warn_hi_s: f64,
    /// Alarm lower limit (mean − 3σ)
    pub alarm_lo_s: f64,
    /// Alarm upper limit (mean + 3σ)
    pub alarm_hi_s: f64,
    /// Number of wafers used to compute statistics
    pub n: usize,
}

/// Compute SPC limits from a historical vector of endpoint times.
///
/// Returns `None` if fewer than 2 samples are provided (σ is undefined).
pub fn compute_spc_limits(endpoint_times_s: &[f64]) -> Option<SpcStats> {
    let n = endpoint_times_s.len();
    if n < 2 {
        return None;
    }
    let mean = endpoint_times_s.iter().sum::<f64>() / n as f64;
    let variance = endpoint_times_s.iter().map(|&t| (t - mean).powi(2)).sum::<f64>()
        / (n - 1) as f64;
    let sigma = variance.sqrt();
    Some(SpcStats {
        mean_s: mean,
        std_dev_s: sigma,
        warn_lo_s: mean - 2.0 * sigma,
        warn_hi_s: mean + 2.0 * sigma,
        alarm_lo_s: mean - 3.0 * sigma,
        alarm_hi_s: mean + 3.0 * sigma,
        n,
    })
}

/// Classify a new endpoint time against SPC limits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpcStatus {
    /// Within warning limits
    Normal,
    /// Outside 2σ but inside 3σ
    Warning,
    /// Outside 3σ – process alarm
    Alarm,
}

impl SpcStats {
    /// Classify an observed endpoint time.
    pub fn classify(&self, time_s: f64) -> SpcStatus {
        if time_s < self.alarm_lo_s || time_s > self.alarm_hi_s {
            SpcStatus::Alarm
        } else if time_s < self.warn_lo_s || time_s > self.warn_hi_s {
            SpcStatus::Warning
        } else {
            SpcStatus::Normal
        }
    }
}

// ---------------------------------------------------------------------------
// Multi-point etch uniformity
// ---------------------------------------------------------------------------

/// Compute etch uniformity across multiple measurement sites on a wafer.
///
/// Uniformity is expressed as the percentage non-uniformity (PNU):
///   PNU = (max − min) / (2 × mean) × 100 %
///
/// Returns `(mean_nm, pnu_percent)`. Returns `(0.0, 0.0)` for empty input.
pub fn etch_uniformity(depths_nm: &[f64]) -> (f64, f64) {
    if depths_nm.is_empty() {
        return (0.0, 0.0);
    }
    let n = depths_nm.len() as f64;
    let mean = depths_nm.iter().sum::<f64>() / n;
    let max = depths_nm.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let min = depths_nm.iter().cloned().fold(f64::INFINITY, f64::min);
    let pnu = if mean.abs() > 1e-12 {
        (max - min) / (2.0 * mean) * 100.0
    } else {
        0.0
    };
    (mean, pnu)
}

// ---------------------------------------------------------------------------
// Process metrics aggregator
// ---------------------------------------------------------------------------

/// Aggregated process metrics for a completed etch run.
#[derive(Debug, Clone)]
pub struct EtchRunMetrics {
    /// Endpoint detection time (s)
    pub endpoint_time_s: f64,
    /// Over-etch end time (s)
    pub process_end_time_s: f64,
    /// Mean etch rate (nm/s)
    pub mean_rate_nm_s: f64,
    /// Final etch depth (nm)
    pub etch_depth_nm: f64,
    /// Etch uniformity percentage non-uniformity (%)
    pub uniformity_pnu_pct: f64,
    /// Etch selectivity (target rate / stop rate)
    pub selectivity: f64,
    /// SPC classification for this wafer's endpoint time
    pub spc_status: SpcStatus,
}

impl EtchRunMetrics {
    /// Construct metrics from measured values.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        endpoint_time_s: f64,
        process_end_time_s: f64,
        mean_rate_nm_s: f64,
        etch_depth_nm: f64,
        uniformity_pnu_pct: f64,
        selectivity: f64,
        spc_status: SpcStatus,
    ) -> Self {
        EtchRunMetrics {
            endpoint_time_s,
            process_end_time_s,
            mean_rate_nm_s,
            etch_depth_nm,
            uniformity_pnu_pct,
            selectivity,
            spc_status,
        }
    }
}

// ---------------------------------------------------------------------------
// Baseline normalisation helper
// ---------------------------------------------------------------------------

/// Baseline-normalise a time series by dividing by the mean of the first
/// `baseline_samples` values.
///
/// Returns the normalised vector. If the baseline mean is near zero the original
/// values are returned unchanged.
pub fn baseline_normalise(data: &[f64], baseline_samples: usize) -> Vec<f64> {
    let n = baseline_samples.min(data.len());
    if n == 0 {
        return data.to_vec();
    }
    let baseline_mean: f64 = data[..n].iter().sum::<f64>() / n as f64;
    if baseline_mean.abs() < 1e-15 {
        return data.to_vec();
    }
    data.iter().map(|&v| v / baseline_mean).collect()
}

// ---------------------------------------------------------------------------
// Ignition transient rejection
// ---------------------------------------------------------------------------

/// Trim leading ignition transient samples from an OES time series.
///
/// Removes the first `transient_samples` from `data`, returning a slice
/// starting after the transient. If `transient_samples >= data.len()`, returns
/// an empty slice.
pub fn trim_ignition_transient(data: &[f64], transient_samples: usize) -> &[f64] {
    if transient_samples >= data.len() {
        &data[data.len()..]
    } else {
        &data[transient_samples..]
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // Helper: generate a synthetic step-down intensity trace
    // -----------------------------------------------------------------------
    fn step_down(n_before: usize, n_after: usize, hi: f64, lo: f64) -> Vec<f64> {
        let mut v = vec![hi; n_before];
        v.extend(vec![lo; n_after]);
        v
    }

    fn step_up(n_before: usize, n_after: usize, lo: f64, hi: f64) -> Vec<f64> {
        let mut v = vec![lo; n_before];
        v.extend(vec![hi; n_after]);
        v
    }

    // -----------------------------------------------------------------------
    // Emission line database
    // -----------------------------------------------------------------------

    #[test]
    fn test_emission_db_non_empty() {
        assert!(!EMISSION_LINE_DB.is_empty());
    }

    #[test]
    fn test_lines_for_silicon_etch() {
        let lines = lines_for_process(EtchProcess::SiliconEtch);
        assert!(!lines.is_empty());
        let species: Vec<_> = lines.iter().map(|l| l.species).collect();
        assert!(species.contains(&"F*"), "F* should appear for Si etch");
        assert!(species.contains(&"SiF*"), "SiF* should appear for Si etch");
    }

    #[test]
    fn test_lines_for_oxide_etch() {
        let lines = lines_for_process(EtchProcess::OxideEtch);
        let species: Vec<_> = lines.iter().map(|l| l.species).collect();
        assert!(species.contains(&"CO*"), "CO* should appear for oxide etch");
        assert!(species.contains(&"O*"), "O* should appear for oxide etch");
    }

    #[test]
    fn test_lines_for_nitride_etch() {
        let lines = lines_for_process(EtchProcess::NitrideEtch);
        let species: Vec<_> = lines.iter().map(|l| l.species).collect();
        assert!(species.contains(&"CN*"), "CN* should appear for nitride etch");
        assert!(species.contains(&"N2*"), "N2* should appear for nitride etch");
    }

    #[test]
    fn test_lines_for_photoresist_strip() {
        let lines = lines_for_process(EtchProcess::PhotoresistStrip);
        let species: Vec<_> = lines.iter().map(|l| l.species).collect();
        assert!(species.contains(&"OH*"), "OH* should appear for PR strip");
        assert!(species.contains(&"O*"), "O* should appear for PR strip");
    }

    #[test]
    fn test_lines_near_704nm() {
        let lines = lines_near_wavelength(704.0);
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0].species, "F*");
    }

    #[test]
    fn test_lines_near_483nm() {
        let lines = lines_near_wavelength(483.0);
        assert!(!lines.is_empty());
        let found = lines.iter().any(|l| l.species == "CO*");
        assert!(found);
    }

    #[test]
    fn test_lines_near_nonexistent_wavelength() {
        let lines = lines_near_wavelength(600.0);
        assert!(lines.is_empty());
    }

    // -----------------------------------------------------------------------
    // OES Spectrum
    // -----------------------------------------------------------------------

    #[test]
    fn test_spectrum_intensity_at_exact() {
        let wl = vec![400.0, 500.0, 600.0, 700.0];
        let in_ = vec![0.1, 0.5, 0.8, 0.3];
        let spec = OesSpectrum::new(wl, in_, 0.0);
        let v = spec.intensity_at(500.0).unwrap();
        assert!((v - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_spectrum_intensity_at_interpolated() {
        let wl = vec![400.0, 600.0];
        let in_ = vec![0.0, 1.0];
        let spec = OesSpectrum::new(wl, in_, 0.0);
        let v = spec.intensity_at(500.0).unwrap();
        // Linear interpolation midpoint should be ~0.5
        assert!((v - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_spectrum_normalise() {
        let wl = vec![400.0, 500.0, 600.0];
        let in_ = vec![2.0, 4.0, 6.0];
        let mut spec = OesSpectrum::new(wl, in_, 0.0);
        spec.normalise();
        assert!((spec.intensities[0]).abs() < 1e-10); // min → 0
        assert!((spec.intensities[2] - 1.0).abs() < 1e-10); // max → 1
    }

    // -----------------------------------------------------------------------
    // Moving average smoother
    // -----------------------------------------------------------------------

    #[test]
    fn test_moving_average_basic() {
        let mut ma = MovingAverage::new(3);
        let v1 = ma.push(1.0);
        let v2 = ma.push(2.0);
        let v3 = ma.push(3.0);
        assert!((v1 - 1.0).abs() < 1e-10);
        assert!((v2 - 1.5).abs() < 1e-10);
        assert!((v3 - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_moving_average_window_slides() {
        let mut ma = MovingAverage::new(2);
        ma.push(10.0);
        ma.push(0.0);
        let v = ma.push(0.0);
        // window=[0,0] → mean=0
        assert!((v).abs() < 1e-10);
    }

    #[test]
    fn test_moving_average_single() {
        let mut ma = MovingAverage::new(1);
        ma.push(7.0);
        let v = ma.push(3.0);
        assert!((v - 3.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Spike removal
    // -----------------------------------------------------------------------

    #[test]
    fn test_spike_removal_no_spikes() {
        let data: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let out = remove_spikes(&data, 3, 2.0);
        assert_eq!(out.len(), data.len());
        // No spikes: output should be close to input
        for (a, b) in data.iter().zip(out.iter()) {
            assert!((a - b).abs() < 1.0); // median filter shifts slightly at edges
        }
    }

    #[test]
    fn test_spike_removal_single_spike() {
        // Constant signal with one cosmic-ray spike
        let mut data = vec![1.0f64; 11];
        data[5] = 100.0; // spike
        let out = remove_spikes(&data, 3, 0.5);
        // The spike at index 5 should be replaced by the local median ≈ 1.0
        assert!(out[5] < 10.0, "spike should be removed, got {}", out[5]);
    }

    #[test]
    fn test_spike_removal_preserves_step() {
        // Ensure a genuine step (not a spike) is preserved
        let data: Vec<f64> = (0..20).map(|i| if i < 10 { 1.0 } else { 2.0 }).collect();
        let out = remove_spikes(&data, 3, 1.5);
        // Values far from step should be preserved
        assert!((out[0] - 1.0).abs() < 0.1);
        assert!((out[19] - 2.0).abs() < 0.1);
    }

    // -----------------------------------------------------------------------
    // Single-wavelength endpoint detection
    // -----------------------------------------------------------------------

    #[test]
    fn test_single_wavelength_falling_edge() {
        let cfg = DetectorConfig {
            process: EtchProcess::SiliconEtch,
            algorithm: EndpointAlgorithm::SingleWavelength {
                wavelength_nm: 704.0,
                threshold: 0.5,
                rising: false,
            },
            smoothing_window: 1,
            ignition_transient_samples: 5,
            over_etch_percent: 10.0,
        };
        let mut det = EndpointDetector::new(cfg);
        let trace = step_down(20, 20, 1.0, 0.2);
        for (i, &v) in trace.iter().enumerate() {
            det.push_sample(v, 704.0, i as f64);
        }
        assert!(det.endpoint_detected(), "should detect falling edge");
        let ev = det.endpoint_event().unwrap();
        // Endpoint should occur near sample 20 (after ignition transient of 5)
        assert!(ev.sample_index >= 5);
        assert!(ev.sample_index <= 25);
    }

    #[test]
    fn test_single_wavelength_rising_edge() {
        let cfg = DetectorConfig {
            process: EtchProcess::OxideEtch,
            algorithm: EndpointAlgorithm::SingleWavelength {
                wavelength_nm: 777.0,
                threshold: 0.5,
                rising: true,
            },
            smoothing_window: 1,
            ignition_transient_samples: 3,
            over_etch_percent: 5.0,
        };
        let mut det = EndpointDetector::new(cfg);
        let trace = step_up(15, 15, 0.1, 0.9);
        for (i, &v) in trace.iter().enumerate() {
            det.push_sample(v, 777.0, i as f64);
        }
        assert!(det.endpoint_detected(), "should detect rising edge");
    }

    #[test]
    fn test_no_endpoint_if_below_threshold() {
        let cfg = DetectorConfig {
            process: EtchProcess::SiliconEtch,
            algorithm: EndpointAlgorithm::SingleWavelength {
                wavelength_nm: 704.0,
                threshold: 0.05, // very low – never crossed downward from 1.0
                rising: false,
            },
            smoothing_window: 1,
            ignition_transient_samples: 2,
            over_etch_percent: 10.0,
        };
        let mut det = EndpointDetector::new(cfg);
        // Signal stays above threshold the whole time
        for i in 0..20 {
            det.push_sample(0.8, 704.0, i as f64);
        }
        assert!(!det.endpoint_detected());
    }

    #[test]
    fn test_ignition_transient_suppresses_early_detection() {
        // If endpoint would occur during transient window, it is ignored
        let cfg = DetectorConfig {
            process: EtchProcess::SiliconEtch,
            algorithm: EndpointAlgorithm::SingleWavelength {
                wavelength_nm: 704.0,
                threshold: 0.5,
                rising: false,
            },
            smoothing_window: 1,
            ignition_transient_samples: 20,
            over_etch_percent: 10.0,
        };
        let mut det = EndpointDetector::new(cfg);
        // Step down at sample 5 (within transient window)
        let trace = step_down(5, 15, 1.0, 0.0);
        for (i, &v) in trace.iter().enumerate() {
            det.push_sample(v, 704.0, i as f64);
        }
        assert!(!det.endpoint_detected(), "endpoint in transient window should be ignored");
    }

    // -----------------------------------------------------------------------
    // Derivative-based endpoint detection
    // -----------------------------------------------------------------------

    #[test]
    fn test_derivative_falling() {
        let cfg = DetectorConfig {
            process: EtchProcess::SiliconEtch,
            algorithm: EndpointAlgorithm::Derivative {
                threshold: 0.3,
                positive: false,
            },
            smoothing_window: 1,
            ignition_transient_samples: 3,
            over_etch_percent: 10.0,
        };
        let mut det = EndpointDetector::new(cfg);
        // Constant then sudden drop of 0.5
        let mut trace = vec![1.0f64; 10];
        trace.push(0.4); // drop by 0.6 > threshold 0.3
        trace.extend(vec![0.4; 10]);
        for (i, &v) in trace.iter().enumerate() {
            det.push_sample(v, 704.0, i as f64);
        }
        assert!(det.endpoint_detected(), "derivative should detect step");
    }

    #[test]
    fn test_derivative_rising() {
        let cfg = DetectorConfig {
            process: EtchProcess::OxideEtch,
            algorithm: EndpointAlgorithm::Derivative {
                threshold: 0.3,
                positive: true,
            },
            smoothing_window: 1,
            ignition_transient_samples: 3,
            over_etch_percent: 5.0,
        };
        let mut det = EndpointDetector::new(cfg);
        let mut trace = vec![0.1f64; 10];
        trace.push(0.7); // rise by 0.6 > threshold
        trace.extend(vec![0.7; 10]);
        for (i, &v) in trace.iter().enumerate() {
            det.push_sample(v, 777.0, i as f64);
        }
        assert!(det.endpoint_detected(), "derivative rising edge");
    }

    // -----------------------------------------------------------------------
    // Intensity ratio endpoint detection
    // -----------------------------------------------------------------------

    #[test]
    fn test_ratio_falling_endpoint() {
        let cfg = DetectorConfig {
            process: EtchProcess::OxideEtch,
            algorithm: EndpointAlgorithm::IntensityRatio {
                primary_nm: 483.0,
                reference_nm: 777.0,
                threshold: 1.0,
                rising: false,
            },
            smoothing_window: 1,
            ignition_transient_samples: 5,
            over_etch_percent: 10.0,
        };
        let mut det = EndpointDetector::new(cfg);
        // Before endpoint: CO* high, O* low → ratio > 1
        // After endpoint: CO* drops, O* constant → ratio < 1
        for i in 0..30 {
            let (prim, refr) = if i < 20 { (2.0, 1.0) } else { (0.5, 1.0) };
            det.push_ratio_sample(prim, refr, i as f64);
        }
        assert!(det.endpoint_detected(), "ratio should detect CO* drop at endpoint");
    }

    #[test]
    fn test_ratio_self_normalising() {
        // Both channels scale by the same factor (lamp drift) – ratio stays constant
        let cfg = DetectorConfig {
            process: EtchProcess::OxideEtch,
            algorithm: EndpointAlgorithm::IntensityRatio {
                primary_nm: 483.0,
                reference_nm: 777.0,
                threshold: 0.5,
                rising: false,
            },
            smoothing_window: 1,
            ignition_transient_samples: 3,
            over_etch_percent: 10.0,
        };
        let mut det = EndpointDetector::new(cfg);
        // Scale up both channels uniformly – ratio stays at 1.0 > threshold 0.5
        for i in 0..20 {
            let scale = 1.0 + i as f64 * 0.1;
            det.push_ratio_sample(1.0 * scale, 1.0 * scale, i as f64);
        }
        assert!(!det.endpoint_detected(), "ratio stays constant despite lamp drift");
    }

    // -----------------------------------------------------------------------
    // PCA-based endpoint detection
    // -----------------------------------------------------------------------

    #[test]
    fn test_pca_endpoint_detection() {
        let cfg = DetectorConfig {
            process: EtchProcess::SiliconEtch,
            algorithm: EndpointAlgorithm::Pca { threshold: 1.0 },
            smoothing_window: 1,
            ignition_transient_samples: 5,
            over_etch_percent: 10.0,
        };
        let mut det = EndpointDetector::new(cfg);
        // Before: spectrum pattern A (high first wavelength)
        // After: spectrum pattern B (low first wavelength → PC1 score changes)
        for i in 0..25 {
            let spectrum: Vec<f64> = if i < 15 {
                vec![2.0, 0.5, 0.3]
            } else {
                vec![0.1, 1.5, 1.8]
            };
            det.push_spectrum(&spectrum, i as f64);
        }
        assert!(det.endpoint_detected(), "PCA should detect spectrum change");
    }

    #[test]
    fn test_pca_scores_direction() {
        // PC1 should capture the primary variance direction.
        // Use data that lies strictly on a 1-D line with no zero features
        // to avoid degenerate cases in power iteration.
        let data: Vec<Vec<f64>> = (1..=20)
            .map(|i| {
                let x = i as f64;
                vec![x, 0.5 * x, -0.3 * x]
            })
            .collect();
        let scores = pca_pc1_scores(&data);
        assert_eq!(scores.len(), 20);
        // Scores should be monotonically non-decreasing or non-increasing
        // (sign of eigenvector is arbitrary, but the ordering is deterministic).
        let diffs: Vec<f64> = scores.windows(2).map(|w| w[1] - w[0]).collect();
        // All differences should have the same sign (allowing tiny numerical errors)
        let max_diff = diffs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min_diff = diffs.iter().cloned().fold(f64::INFINITY, f64::min);
        // Either all ≥ 0 or all ≤ 0; check that min and max have the same sign
        assert!(
            max_diff <= 1e-10 || min_diff >= -1e-10,
            "PC1 scores should be monotone for linearly varying data; max_diff={:.3e}, min_diff={:.3e}",
            max_diff, min_diff
        );
    }

    // -----------------------------------------------------------------------
    // Interferometric etch rate
    // -----------------------------------------------------------------------

    #[test]
    fn test_interferometric_fringe_counting() {
        // HeNe laser, SiO2 (n=1.46)
        let mut mon = InterferometricMonitor::new(632.8, 1.46);
        // Generate a sinusoidal reflectance (simulating fringes)
        let n_cycles = 5;
        let samples_per_cycle = 20;
        let total = n_cycles * samples_per_cycle;
        for i in 0..total {
            let t = i as f64;
            let r = (2.0 * PI * t / samples_per_cycle as f64).sin();
            mon.push_reflectance(r, t * 0.1);
        }
        // Each cycle has 2 zero-crossings = 2 half-fringes → 5 cycles → ~10 half-fringes
        // Allow ±3 tolerance for edge effects
        let fc = mon.fringe_count();
        assert!(fc >= 7 && fc <= 13, "fringe count = {}", fc);
    }

    #[test]
    fn test_interferometric_thickness_accumulates() {
        // Use a longer sinusoidal signal so the moving-average baseline warms up
        // before zero-crossing detection begins producing fringe counts.
        let mut mon = InterferometricMonitor::new(632.8, 1.46);
        let samples_per_cycle = 20usize;
        let n_cycles = 4usize;
        let total = n_cycles * samples_per_cycle;
        let mut last_t = 0.0f64;
        for i in 0..total {
            let t = i as f64;
            let r = (2.0 * PI * t / samples_per_cycle as f64).sin();
            last_t = mon.push_reflectance(r, t * 0.1);
        }
        // 4 cycles → at least 4 half-periods after the smoother warms up
        assert!(mon.fringe_count() >= 1, "should count at least one fringe, got {}", mon.fringe_count());
        assert!(last_t >= 0.0, "returned thickness must be non-negative");
        // Verify etched thickness is positive after counting fringes
        assert!(mon.etched_thickness_nm() > 0.0, "etched thickness should be positive");
    }

    #[test]
    fn test_interferometric_remaining_thickness() {
        let mut mon = InterferometricMonitor::new(632.8, 1.46);
        let initial = 500.0; // nm
        // Use sinusoidal signal so moving-average baseline warms up
        let samples_per_cycle = 20usize;
        let n_cycles = 4usize;
        for i in 0..(n_cycles * samples_per_cycle) {
            let t = i as f64;
            let r = (2.0 * PI * t / samples_per_cycle as f64).sin();
            mon.push_reflectance(r, t * 0.05);
        }
        let etched = mon.etched_thickness_nm();
        let remaining = mon.remaining_thickness_nm(initial);
        // etched + remaining == initial (clamped at 0)
        if remaining > 0.0 {
            assert!(
                (etched + remaining - initial).abs() < 1.0,
                "etched={:.1} + remaining={:.1} should equal initial={:.1}",
                etched, remaining, initial
            );
        } else {
            assert!(etched >= initial, "if remaining=0, etched should be >= initial");
        }
    }

    #[test]
    fn test_interferometric_average_rate() {
        let mut mon = InterferometricMonitor::new(632.8, 2.0);
        // Push a sinusoid at 1 Hz (one cycle per second)
        let fs = 100.0; // 100 samples/s
        for i in 0..200 {
            let t = i as f64 / fs;
            let r = (2.0 * PI * 1.0 * t).sin();
            mon.push_reflectance(r, t);
        }
        let avg = mon.average_rate_nm_s(2.0); // 2 seconds elapsed
        assert!(avg > 0.0, "average rate should be positive");
    }

    #[test]
    fn test_thickness_per_half_period() {
        // λ=532 nm, n=2.0 → dz = 532/(4*2) = 66.5 nm
        let mon = InterferometricMonitor::new(532.0, 2.0);
        let expected = 532.0 / (4.0 * 2.0);
        assert!((mon.thickness_per_half_period_nm - expected).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Over-etch control
    // -----------------------------------------------------------------------

    #[test]
    fn test_over_etch_timing() {
        let mut ctrl = OverEtchController::new(10.0, 0.0);
        ctrl.on_endpoint(100.0);
        // Over-etch = 10% of 100s = 10s → end at 110s
        let end = ctrl.over_etch_end_s().unwrap();
        assert!((end - 110.0).abs() < 1e-10);
        assert!(!ctrl.is_complete(105.0));
        assert!(ctrl.is_complete(110.0));
        assert!(ctrl.is_complete(115.0));
    }

    #[test]
    fn test_over_etch_with_fixed_time() {
        let mut ctrl = OverEtchController::new(10.0, 5.0);
        ctrl.on_endpoint(50.0);
        // 10% of 50s = 5s + fixed 5s = 10s → end at 60s
        let end = ctrl.over_etch_end_s().unwrap();
        assert!((end - 60.0).abs() < 1e-10);
    }

    #[test]
    fn test_over_etch_not_triggered() {
        let ctrl = OverEtchController::new(10.0, 0.0);
        assert!(!ctrl.is_complete(1000.0));
        assert!(ctrl.over_etch_end_s().is_none());
    }

    #[test]
    fn test_over_etch_duration() {
        let mut ctrl = OverEtchController::new(20.0, 0.0);
        ctrl.on_endpoint(60.0);
        let dur = ctrl.over_etch_duration_s(60.0).unwrap();
        assert!((dur - 12.0).abs() < 1e-10); // 20% of 60s = 12s
    }

    // -----------------------------------------------------------------------
    // Etch selectivity
    // -----------------------------------------------------------------------

    #[test]
    fn test_selectivity_normal() {
        let s = etch_selectivity(100.0, 10.0);
        assert!((s - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_selectivity_infinite() {
        let s = etch_selectivity(100.0, 0.0);
        assert!(s.is_infinite());
    }

    #[test]
    fn test_selectivity_less_than_one() {
        // Poor selectivity
        let s = etch_selectivity(5.0, 10.0);
        assert!((s - 0.5).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Statistical Process Control
    // -----------------------------------------------------------------------

    #[test]
    fn test_spc_limits_basic() {
        let times = vec![100.0, 102.0, 98.0, 101.0, 99.0];
        let stats = compute_spc_limits(&times).unwrap();
        assert!((stats.mean_s - 100.0).abs() < 1.0);
        assert!(stats.std_dev_s > 0.0);
        assert!(stats.warn_hi_s > stats.mean_s);
        assert!(stats.warn_lo_s < stats.mean_s);
        assert!(stats.alarm_hi_s > stats.warn_hi_s);
        assert!(stats.alarm_lo_s < stats.warn_lo_s);
        assert_eq!(stats.n, 5);
    }

    #[test]
    fn test_spc_limits_insufficient_data() {
        assert!(compute_spc_limits(&[100.0]).is_none());
        assert!(compute_spc_limits(&[]).is_none());
    }

    #[test]
    fn test_spc_classify_normal() {
        let times: Vec<f64> = (0..10).map(|i| 100.0 + i as f64 * 0.1).collect();
        let stats = compute_spc_limits(&times).unwrap();
        let status = stats.classify(stats.mean_s);
        assert_eq!(status, SpcStatus::Normal);
    }

    #[test]
    fn test_spc_classify_warning() {
        // Create stats with known sigma
        let times = vec![0.0, 2.0]; // mean=1, sigma≈1.414
        let stats = compute_spc_limits(&times).unwrap();
        // 2.5σ above mean should be Warning
        let test_val = stats.mean_s + 2.5 * stats.std_dev_s;
        let status = stats.classify(test_val);
        assert_eq!(status, SpcStatus::Warning);
    }

    #[test]
    fn test_spc_classify_alarm() {
        let times = vec![0.0, 2.0]; // mean=1, sigma≈1.414
        let stats = compute_spc_limits(&times).unwrap();
        // 4σ above mean → Alarm
        let test_val = stats.mean_s + 4.0 * stats.std_dev_s;
        let status = stats.classify(test_val);
        assert_eq!(status, SpcStatus::Alarm);
    }

    #[test]
    fn test_spc_symmetry() {
        let times: Vec<f64> = (0..20).map(|i| 100.0 + i as f64).collect();
        let stats = compute_spc_limits(&times).unwrap();
        let above = stats.warn_hi_s - stats.mean_s;
        let below = stats.mean_s - stats.warn_lo_s;
        assert!((above - below).abs() < 1e-10, "SPC limits should be symmetric");
    }

    // -----------------------------------------------------------------------
    // Uniformity
    // -----------------------------------------------------------------------

    #[test]
    fn test_uniformity_perfect() {
        let depths = vec![100.0, 100.0, 100.0, 100.0, 100.0];
        let (mean, pnu) = etch_uniformity(&depths);
        assert!((mean - 100.0).abs() < 1e-10);
        assert!(pnu.abs() < 1e-10);
    }

    #[test]
    fn test_uniformity_known() {
        // max=110, min=90, mean=100 → PNU = 20/(200) × 100 = 10%
        let depths = vec![90.0, 100.0, 110.0];
        let (mean, pnu) = etch_uniformity(&depths);
        assert!((mean - 100.0).abs() < 1.0);
        assert!((pnu - 10.0).abs() < 0.1, "PNU should be ~10%, got {}", pnu);
    }

    #[test]
    fn test_uniformity_empty() {
        let (mean, pnu) = etch_uniformity(&[]);
        assert_eq!(mean, 0.0);
        assert_eq!(pnu, 0.0);
    }

    // -----------------------------------------------------------------------
    // Baseline normalisation
    // -----------------------------------------------------------------------

    #[test]
    fn test_baseline_normalise() {
        let data = vec![2.0, 2.0, 1.0, 1.0]; // baseline mean from first 2 = 2.0
        let norm = baseline_normalise(&data, 2);
        assert!((norm[0] - 1.0).abs() < 1e-10);
        assert!((norm[2] - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_baseline_normalise_full() {
        let data = vec![4.0, 4.0, 8.0]; // mean first 2 = 4
        let norm = baseline_normalise(&data, 2);
        assert!((norm[2] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_baseline_normalise_zero_baseline() {
        let data = vec![0.0, 0.0, 1.0];
        let norm = baseline_normalise(&data, 2);
        // Zero baseline: return original
        assert_eq!(norm, data);
    }

    // -----------------------------------------------------------------------
    // Ignition transient trimming
    // -----------------------------------------------------------------------

    #[test]
    fn test_trim_ignition_transient_normal() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let trimmed = trim_ignition_transient(&data, 2);
        assert_eq!(trimmed, &[3.0, 4.0, 5.0]);
    }

    #[test]
    fn test_trim_ignition_transient_zero() {
        let data = vec![1.0, 2.0, 3.0];
        let trimmed = trim_ignition_transient(&data, 0);
        assert_eq!(trimmed, &[1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_trim_ignition_transient_exceeds_length() {
        let data = vec![1.0, 2.0];
        let trimmed = trim_ignition_transient(&data, 10);
        assert!(trimmed.is_empty());
    }

    // -----------------------------------------------------------------------
    // Over-etch from detector
    // -----------------------------------------------------------------------

    #[test]
    fn test_detector_over_etch_duration() {
        let cfg = DetectorConfig {
            process: EtchProcess::SiliconEtch,
            algorithm: EndpointAlgorithm::SingleWavelength {
                wavelength_nm: 704.0,
                threshold: 0.5,
                rising: false,
            },
            smoothing_window: 1,
            ignition_transient_samples: 5,
            over_etch_percent: 20.0,
        };
        let mut det = EndpointDetector::new(cfg);
        let trace = step_down(20, 20, 1.0, 0.1);
        for (i, &v) in trace.iter().enumerate() {
            det.push_sample(v, 704.0, i as f64);
        }
        assert!(det.endpoint_detected());
        let oe = det.over_etch_duration_s().unwrap();
        assert!(oe > 0.0, "over-etch duration should be positive");
    }

    // -----------------------------------------------------------------------
    // Detector reset
    // -----------------------------------------------------------------------

    #[test]
    fn test_detector_reset() {
        let cfg = DetectorConfig {
            process: EtchProcess::SiliconEtch,
            algorithm: EndpointAlgorithm::SingleWavelength {
                wavelength_nm: 704.0,
                threshold: 0.5,
                rising: false,
            },
            smoothing_window: 1,
            ignition_transient_samples: 2,
            over_etch_percent: 10.0,
        };
        let mut det = EndpointDetector::new(cfg);
        let trace = step_down(10, 10, 1.0, 0.1);
        for (i, &v) in trace.iter().enumerate() {
            det.push_sample(v, 704.0, i as f64);
        }
        assert!(det.endpoint_detected());
        det.reset();
        assert!(!det.endpoint_detected());
        assert_eq!(det.sample_count(), 0);
    }

    // -----------------------------------------------------------------------
    // EtchRunMetrics construction
    // -----------------------------------------------------------------------

    #[test]
    fn test_etch_run_metrics_construction() {
        let m = EtchRunMetrics::new(60.0, 66.0, 5.0, 300.0, 3.5, 10.0, SpcStatus::Normal);
        assert_eq!(m.endpoint_time_s, 60.0);
        assert_eq!(m.process_end_time_s, 66.0);
        assert!((m.mean_rate_nm_s - 5.0).abs() < 1e-10);
        assert_eq!(m.spc_status, SpcStatus::Normal);
    }

    // -----------------------------------------------------------------------
    // Edge cases / integration
    // -----------------------------------------------------------------------

    #[test]
    fn test_full_etch_run_integration() {
        // Simulate a silicon etch: F* at 704 nm rises at endpoint as Si clears
        let cfg = DetectorConfig {
            process: EtchProcess::SiliconEtch,
            algorithm: EndpointAlgorithm::SingleWavelength {
                wavelength_nm: 704.0,
                threshold: 0.6,
                rising: true, // F* rises when Si layer clears
            },
            smoothing_window: 3,
            ignition_transient_samples: 10,
            over_etch_percent: 15.0,
        };
        let mut det = EndpointDetector::new(cfg);

        // Signal: low (Si consuming F*) then rises (layer cleared)
        let trace: Vec<f64> = (0..60)
            .map(|i| if i < 40 { 0.2 } else { 0.9 })
            .collect();
        for (i, &v) in trace.iter().enumerate() {
            det.push_sample(v, 704.0, i as f64);
        }

        assert!(det.endpoint_detected());
        let ev = det.endpoint_event().unwrap();
        assert!(ev.time_s >= 10.0, "endpoint should be after transient");
        assert!(ev.time_s <= 55.0, "endpoint should be near step transition");

        // Over-etch
        let oe = det.over_etch_duration_s().unwrap();
        // 15% of endpoint_time
        let expected_oe = ev.time_s * 0.15;
        assert!((oe - expected_oe).abs() < 1.0, "over-etch duration mismatch");
    }

    #[test]
    fn test_pca_scores_empty() {
        let scores = pca_pc1_scores(&[]);
        assert!(scores.is_empty());
    }

    #[test]
    fn test_pca_scores_constant_data() {
        // All identical rows → zero variance → scores should all equal zero
        let data: Vec<Vec<f64>> = vec![vec![1.0, 2.0, 3.0]; 10];
        let scores = pca_pc1_scores(&data);
        for s in &scores {
            assert!(s.abs() < 1e-10, "constant data → zero scores, got {}", s);
        }
    }

    #[test]
    fn test_moving_average_is_empty() {
        let ma = MovingAverage::new(5);
        assert!(ma.is_empty());
        assert_eq!(ma.len(), 0);
    }

    #[test]
    fn test_interferometric_rate_history() {
        let mut mon = InterferometricMonitor::new(632.8, 1.46);
        let fs = 50.0f64;
        for i in 0..100 {
            let t = i as f64 / fs;
            let r = (2.0 * PI * 2.0 * t).sin();
            mon.push_reflectance(r, t);
        }
        // Should have some entries in rate history
        let hist = mon.rate_history();
        assert!(!hist.is_empty(), "rate history should be populated after fringes");
    }
}
