//! # Solar Flare Predictor and Space Weather Analyzer
//!
//! This module implements solar flare prediction and space weather analysis
//! from solar radio flux and X-ray measurements. It provides tools for
//! monitoring and forecasting solar activity that affects radio communications,
//! satellite operations, and power grid stability.
//!
//! ## Key Physics
//!
//! - **GOES X-ray Classification**: Solar flares are classified by their peak
//!   soft X-ray flux in the 0.1-0.8 nm band as measured by GOES satellites.
//!   Classes are A, B, C, M, X with each class representing a 10x increase
//!   in flux. Sub-classes range from 1.0 to 9.9.
//!
//! - **Solar Radio Burst Types**: Solar radio emissions are classified into
//!   five types (I-V) based on their spectral characteristics:
//!   - Type I: Noise storms, narrow-band, long-duration
//!   - Type II: Slow-drift (~1 MHz/s), associated with CME-driven shocks
//!   - Type III: Fast-drift (~100 MHz/s), electron beam along field lines
//!   - Type IV: Broadband continuum emission
//!   - Type V: Short continuum following Type III bursts
//!
//! - **F10.7 Solar Flux Index**: The 10.7 cm (2800 MHz) solar radio flux
//!   is a widely used proxy for overall solar activity, measured in Solar
//!   Flux Units (1 SFU = 10^-22 W/m²/Hz).
//!
//! - **Neupert Effect**: The time derivative of soft X-ray flux is
//!   approximately proportional to hard X-ray emission during the impulsive
//!   phase of a solar flare.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::solar_flare_predictor::*;
//!
//! // Classify a flare by its peak X-ray flux
//! let detector = SolarFlareDetector::new(SpaceWeatherConfig::default());
//! let (class, sub_level) = detector.goes_classification(1e-5);
//! assert!(matches!(class, FlareClass::M));
//! assert!((sub_level - 1.0).abs() < 0.1);
//!
//! // Estimate CME arrival time for a fast CME
//! let predictor = GeomagneticStormPredictor;
//! let hours = predictor.cme_arrival_time_hours(1000.0, 1.0);
//! assert!((hours - 41.7).abs() < 1.0);
//! ```

/// X-ray wavelength channel selection for GOES satellite measurements.
///
/// GOES satellites carry two X-ray sensors (XRS) measuring solar flux
/// in two wavelength bands. The long-wavelength channel is the standard
/// reference for flare classification.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum XrayChannel {
    /// Long wavelength channel: 0.1-0.8 nm (1-8 Angstroms).
    /// This is the standard channel used for GOES flare classification.
    Long,
    /// Short wavelength channel: 0.05-0.4 nm (0.5-4 Angstroms).
    /// Higher energy photons, useful for temperature diagnostics.
    Short,
}

/// GOES solar flare classification based on peak soft X-ray flux.
///
/// Each class represents a factor-of-10 increase in peak flux measured
/// in the 0.1-0.8 nm band:
///
/// | Class | Peak Flux (W/m²)    |
/// |-------|---------------------|
/// | A     | < 10^-7             |
/// | B     | 10^-7 to < 10^-6   |
/// | C     | 10^-6 to < 10^-5   |
/// | M     | 10^-5 to < 10^-4   |
/// | X     | >= 10^-4            |
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlareClass {
    /// A-class: very minor flares, peak flux < 10^-7 W/m².
    A,
    /// B-class: minor flares, peak flux 10^-7 to < 10^-6 W/m².
    B,
    /// C-class: small flares, peak flux 10^-6 to < 10^-5 W/m².
    /// Can cause brief HF radio absorption at high latitudes.
    C,
    /// M-class: moderate flares, peak flux 10^-5 to < 10^-4 W/m².
    /// Can cause brief HF radio blackouts and minor radiation storms.
    M,
    /// X-class: major flares, peak flux >= 10^-4 W/m².
    /// Can cause planet-wide HF blackouts and long-lasting radiation storms.
    X,
}

impl FlareClass {
    /// Classify a solar flare based on its peak X-ray flux in W/m².
    ///
    /// Uses the GOES classification scale where each letter class
    /// represents a decade (factor of 10) increase in flux.
    ///
    /// # Arguments
    ///
    /// * `flux_wm2` - Peak X-ray flux in Watts per square meter.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use r4w_core::solar_flare_predictor::FlareClass;
    ///
    /// assert_eq!(FlareClass::from_peak_flux(1e-5), FlareClass::M);
    /// assert_eq!(FlareClass::from_peak_flux(1e-4), FlareClass::X);
    /// assert_eq!(FlareClass::from_peak_flux(1e-6), FlareClass::C);
    /// assert_eq!(FlareClass::from_peak_flux(5e-8), FlareClass::A);
    /// ```
    pub fn from_peak_flux(flux_wm2: f64) -> FlareClass {
        if flux_wm2 >= 1e-4 {
            FlareClass::X
        } else if flux_wm2 >= 1e-5 {
            FlareClass::M
        } else if flux_wm2 >= 1e-6 {
            FlareClass::C
        } else if flux_wm2 >= 1e-7 {
            FlareClass::B
        } else {
            FlareClass::A
        }
    }

    /// Return the numeric sub-class value (1.0 to 9.9) for a given flux.
    ///
    /// The sub-class is the flux value divided by the class floor.
    /// For example, 3.5e-5 W/m² is M3.5, and 7.2e-4 W/m² is X7.2.
    ///
    /// # Arguments
    ///
    /// * `flux_wm2` - Peak X-ray flux in Watts per square meter.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use r4w_core::solar_flare_predictor::FlareClass;
    ///
    /// let sub = FlareClass::numeric_class(3.5e-5);
    /// assert!((sub - 3.5).abs() < 0.01);
    /// ```
    pub fn numeric_class(flux_wm2: f64) -> f64 {
        let class = FlareClass::from_peak_flux(flux_wm2);
        let floor = match class {
            FlareClass::A => 1e-8,
            FlareClass::B => 1e-7,
            FlareClass::C => 1e-6,
            FlareClass::M => 1e-5,
            FlareClass::X => 1e-4,
        };
        flux_wm2 / floor
    }

    /// Format the flare classification as a string (e.g., "M5.3", "X1.0").
    ///
    /// # Arguments
    ///
    /// * `flux_wm2` - Peak X-ray flux in Watts per square meter.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use r4w_core::solar_flare_predictor::FlareClass;
    ///
    /// assert_eq!(FlareClass::to_class_string(1e-5), "M1.0");
    /// assert_eq!(FlareClass::to_class_string(5.3e-5), "M5.3");
    /// ```
    pub fn to_class_string(flux_wm2: f64) -> String {
        let class = FlareClass::from_peak_flux(flux_wm2);
        let sub = FlareClass::numeric_class(flux_wm2);
        let letter = match class {
            FlareClass::A => "A",
            FlareClass::B => "B",
            FlareClass::C => "C",
            FlareClass::M => "M",
            FlareClass::X => "X",
        };
        format!("{}{:.1}", letter, sub)
    }
}

/// Solar radio burst classification based on spectral morphology.
///
/// Solar radio bursts are classified into five types based on their
/// appearance in dynamic spectra (frequency vs time plots). Each type
/// is associated with different physical processes in the solar corona.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RadioBurstType {
    /// Type I: Noise storms. Narrow-band, long-duration emissions from
    /// active regions. Frequencies typically 50-300 MHz.
    TypeI,
    /// Type II: Slow-drift bursts (~0.5-2 MHz/s). Associated with
    /// CME-driven MHD shocks propagating through the corona.
    /// Strong indicator of CME activity.
    TypeII,
    /// Type III: Fast-drift bursts (~20-200 MHz/s). Produced by
    /// relativistic electron beams traveling along open magnetic
    /// field lines. Most common type during active periods.
    TypeIII,
    /// Type IV: Broadband continuum emission. Associated with
    /// post-flare magnetic restructuring and trapped electrons.
    TypeIV,
    /// Type V: Short-duration continuum following Type III bursts.
    /// Smooth, broadband emission lasting seconds to minutes.
    TypeV,
}

/// Configuration parameters for space weather monitoring.
///
/// Controls the detection sensitivity and measurement parameters
/// for X-ray flux analysis and flare detection.
#[derive(Debug, Clone)]
pub struct SpaceWeatherConfig {
    /// Sample rate of the input data in Hz.
    pub sample_rate_hz: f64,
    /// X-ray channel selection (Long or Short wavelength).
    pub xray_channel: XrayChannel,
    /// Baseline (quiet-sun) X-ray flux level in W/m².
    /// Typical quiet-sun level is ~1e-8 W/m² (A1.0).
    pub baseline_flux_wm2: f64,
    /// Detection threshold as a multiple of the baseline flux.
    /// Flare onset is detected when flux exceeds baseline * threshold_factor.
    pub detection_threshold_factor: f64,
}

impl Default for SpaceWeatherConfig {
    /// Default configuration for GOES long-channel X-ray monitoring.
    ///
    /// - Sample rate: 1 Hz (one measurement per second)
    /// - Channel: Long (0.1-0.8 nm), standard for classification
    /// - Baseline: 1e-8 W/m² (A1.0, typical quiet-sun level)
    /// - Threshold: 10x baseline for onset detection
    fn default() -> Self {
        Self {
            sample_rate_hz: 1.0,
            xray_channel: XrayChannel::Long,
            baseline_flux_wm2: 1e-8,
            detection_threshold_factor: 10.0,
        }
    }
}

/// Solar flare detector for X-ray flux time series analysis.
///
/// Detects flare onsets, measures peak flux, classifies events by the
/// GOES scale, and computes temporal characteristics (rise time, decay time,
/// duration). Also implements the Neupert effect for hard X-ray estimation.
///
/// # Example
///
/// ```rust
/// use r4w_core::solar_flare_predictor::*;
///
/// let config = SpaceWeatherConfig {
///     sample_rate_hz: 1.0,
///     baseline_flux_wm2: 1e-8,
///     detection_threshold_factor: 10.0,
///     ..Default::default()
/// };
/// let detector = SolarFlareDetector::new(config);
///
/// // Simulate a C-class flare
/// let mut xray: Vec<f64> = vec![1e-8; 100]; // quiet sun
/// for i in 100..200 {
///     xray.push(1e-8 + 5e-6 * ((i - 100) as f64 / 100.0)); // rising
/// }
/// let onsets = detector.detect_flare_onset(&xray, 10.0);
/// assert!(!onsets.is_empty());
/// ```
pub struct SolarFlareDetector {
    config: SpaceWeatherConfig,
}

impl SolarFlareDetector {
    /// Create a new solar flare detector with the given configuration.
    ///
    /// # Arguments
    ///
    /// * `config` - Space weather monitoring configuration.
    pub fn new(config: SpaceWeatherConfig) -> Self {
        Self { config }
    }

    /// Detect flare onset times in an X-ray flux time series.
    ///
    /// A flare onset is detected when the flux exceeds the baseline level
    /// multiplied by the threshold factor, and the flux is increasing
    /// (positive derivative). Returns the sample indices of detected onsets.
    ///
    /// # Arguments
    ///
    /// * `xray_flux` - Time series of X-ray flux measurements in W/m².
    /// * `threshold_factor` - Multiple of baseline flux for onset detection.
    ///
    /// # Returns
    ///
    /// Vector of sample indices where flare onsets are detected.
    pub fn detect_flare_onset(&self, xray_flux: &[f64], threshold_factor: f64) -> Vec<usize> {
        if xray_flux.len() < 2 {
            return Vec::new();
        }

        let threshold = self.config.baseline_flux_wm2 * threshold_factor;
        let mut onsets = Vec::new();
        let mut in_flare = false;

        for i in 1..xray_flux.len() {
            let above_threshold = xray_flux[i] > threshold;
            let rising = xray_flux[i] > xray_flux[i - 1];
            let was_below = xray_flux[i - 1] <= threshold;

            if above_threshold && rising && was_below && !in_flare {
                onsets.push(i);
                in_flare = true;
            } else if !above_threshold {
                in_flare = false;
            }
        }

        onsets
    }

    /// Find the peak flux value and its index after a flare onset.
    ///
    /// Searches forward from the onset index to find the maximum flux value,
    /// stopping when the flux begins a sustained decline.
    ///
    /// # Arguments
    ///
    /// * `xray_flux` - Time series of X-ray flux measurements in W/m².
    /// * `onset_idx` - Sample index of the flare onset.
    ///
    /// # Returns
    ///
    /// Tuple of (peak_index, peak_flux_value).
    pub fn flare_peak_flux(&self, xray_flux: &[f64], onset_idx: usize) -> (usize, f64) {
        let mut peak_idx = onset_idx;
        let mut peak_val = xray_flux[onset_idx];

        for i in onset_idx..xray_flux.len() {
            if xray_flux[i] > peak_val {
                peak_val = xray_flux[i];
                peak_idx = i;
            }
        }

        (peak_idx, peak_val)
    }

    /// Compute the flare duration as the full width at half maximum (FWHM).
    ///
    /// Measures the time span during which the flux exceeds the specified
    /// half-peak level, returned in units of samples.
    ///
    /// # Arguments
    ///
    /// * `xray_flux` - Time series of X-ray flux measurements in W/m².
    /// * `onset_idx` - Sample index of the flare onset.
    /// * `half_peak_level` - Flux level at which to measure duration (typically
    ///   half of peak flux above background).
    ///
    /// # Returns
    ///
    /// Duration in samples above the half-peak level.
    pub fn flare_duration(
        &self,
        xray_flux: &[f64],
        onset_idx: usize,
        half_peak_level: f64,
    ) -> f64 {
        let mut start = None;
        let mut end = onset_idx;

        for i in onset_idx..xray_flux.len() {
            if xray_flux[i] >= half_peak_level && start.is_none() {
                start = Some(i);
            }
            if start.is_some() && xray_flux[i] >= half_peak_level {
                end = i;
            }
        }

        match start {
            Some(s) => (end - s) as f64,
            None => 0.0,
        }
    }

    /// Classify a flare by its peak X-ray flux using the GOES scale.
    ///
    /// Returns both the letter class (A/B/C/M/X) and the numeric
    /// sub-level (1.0-9.9+).
    ///
    /// # Arguments
    ///
    /// * `peak_flux_wm2` - Peak X-ray flux in Watts per square meter.
    ///
    /// # Returns
    ///
    /// Tuple of (FlareClass, sub-level).
    ///
    /// # Examples
    ///
    /// ```rust
    /// use r4w_core::solar_flare_predictor::*;
    ///
    /// let det = SolarFlareDetector::new(SpaceWeatherConfig::default());
    /// let (class, sub) = det.goes_classification(1e-5);
    /// assert_eq!(class, FlareClass::M);
    /// assert!((sub - 1.0).abs() < 0.1);
    /// ```
    pub fn goes_classification(&self, peak_flux_wm2: f64) -> (FlareClass, f64) {
        let class = FlareClass::from_peak_flux(peak_flux_wm2);
        let sub = FlareClass::numeric_class(peak_flux_wm2);
        (class, sub)
    }

    /// Compute the rise time of a flare from onset to peak.
    ///
    /// Rise time is defined as the time interval between flare onset
    /// and peak flux, expressed in seconds.
    ///
    /// # Arguments
    ///
    /// * `xray_flux` - Time series of X-ray flux measurements (unused but kept for API consistency).
    /// * `onset_idx` - Sample index of the flare onset.
    /// * `peak_idx` - Sample index of the peak flux.
    /// * `sample_rate` - Measurement sample rate in Hz.
    ///
    /// # Returns
    ///
    /// Rise time in seconds.
    pub fn rise_time(
        &self,
        _xray_flux: &[f64],
        onset_idx: usize,
        peak_idx: usize,
        sample_rate: f64,
    ) -> f64 {
        (peak_idx as f64 - onset_idx as f64) / sample_rate
    }

    /// Compute the e-folding decay time after the flare peak.
    ///
    /// The decay time is estimated as the time for the flux to fall
    /// from its peak value to 1/e (~36.8%) of the peak value above
    /// the baseline. If the flux does not decay to 1/e within the
    /// available data, estimates from the available decay rate.
    ///
    /// # Arguments
    ///
    /// * `xray_flux` - Time series of X-ray flux measurements in W/m².
    /// * `peak_idx` - Sample index of the peak flux.
    /// * `sample_rate` - Measurement sample rate in Hz.
    ///
    /// # Returns
    ///
    /// Estimated e-folding decay time in seconds.
    pub fn decay_time(&self, xray_flux: &[f64], peak_idx: usize, sample_rate: f64) -> f64 {
        if peak_idx >= xray_flux.len() {
            return 0.0;
        }

        let peak_val = xray_flux[peak_idx];
        let baseline = self.config.baseline_flux_wm2;
        let peak_above_baseline = peak_val - baseline;

        if peak_above_baseline <= 0.0 {
            return 0.0;
        }

        // Target: flux drops to baseline + peak_above_baseline / e
        let target = baseline + peak_above_baseline / std::f64::consts::E;

        for i in peak_idx..xray_flux.len() {
            if xray_flux[i] <= target {
                return (i as f64 - peak_idx as f64) / sample_rate;
            }
        }

        // If we don't reach 1/e in the data, estimate from end
        let end_val = xray_flux[xray_flux.len() - 1];
        let end_above = end_val - baseline;
        if end_above > 0.0 && end_above < peak_above_baseline {
            let fraction_remaining = end_above / peak_above_baseline;
            let elapsed_samples = (xray_flux.len() - 1 - peak_idx) as f64;
            // From exponential decay: fraction = exp(-t/tau)
            // tau = -t / ln(fraction)
            let ln_frac = fraction_remaining.ln();
            if ln_frac < 0.0 {
                return -(elapsed_samples / sample_rate) / ln_frac;
            }
        }

        0.0
    }

    /// Compute the Neupert effect: the time derivative of soft X-ray flux.
    ///
    /// The Neupert effect states that the time derivative of the soft
    /// X-ray emission is approximately proportional to the hard X-ray
    /// (or microwave) emission during the impulsive phase of a solar flare.
    /// This is because hard X-rays trace the energy input from accelerated
    /// electrons, while soft X-rays trace the thermal response.
    ///
    /// The output is the numerical derivative (forward difference) of the
    /// input flux, scaled by the sample rate.
    ///
    /// # Arguments
    ///
    /// * `xray_flux` - Time series of soft X-ray flux measurements in W/m².
    /// * `sample_rate` - Measurement sample rate in Hz.
    ///
    /// # Returns
    ///
    /// Vector of derivative values (length = input length - 1).
    /// Positive values during the impulsive phase approximate hard X-ray flux.
    pub fn neupert_effect(&self, xray_flux: &[f64], sample_rate: f64) -> Vec<f64> {
        if xray_flux.len() < 2 {
            return Vec::new();
        }

        let mut derivative = Vec::with_capacity(xray_flux.len() - 1);
        for i in 1..xray_flux.len() {
            derivative.push((xray_flux[i] - xray_flux[i - 1]) * sample_rate);
        }

        derivative
    }
}

/// Solar radio flux and burst analyzer.
///
/// Provides tools for analyzing solar radio emissions at various frequencies,
/// including the F10.7 solar flux index, solar flux unit conversions, and
/// radio burst detection and classification.
pub struct SolarRadioAnalyzer;

impl SolarRadioAnalyzer {
    /// Convert Solar Flux Units (SFU) to W/m²/Hz.
    ///
    /// 1 SFU = 10^-22 W/m²/Hz. The SFU is the standard unit for
    /// reporting solar radio flux, particularly the F10.7 index.
    ///
    /// # Arguments
    ///
    /// * `sfu` - Solar flux in Solar Flux Units.
    ///
    /// # Returns
    ///
    /// Equivalent flux in W/m²/Hz.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use r4w_core::solar_flare_predictor::SolarRadioAnalyzer;
    ///
    /// let flux = SolarRadioAnalyzer::solar_flux_unit_to_wm2hz(1.0);
    /// assert!((flux - 1e-22).abs() < 1e-30);
    /// ```
    pub fn solar_flux_unit_to_wm2hz(sfu: f64) -> f64 {
        sfu * 1e-22
    }

    /// Estimate the sunspot number (SSN) from the F10.7 solar flux index.
    ///
    /// Uses the empirical inverse relation derived from the forward formula:
    ///   F10.7 = 63 + 0.73*SSN + 0.0009*SSN²
    ///
    /// This inverts using the quadratic formula.
    ///
    /// # Arguments
    ///
    /// * `f107` - F10.7 solar flux index in SFU.
    ///
    /// # Returns
    ///
    /// Estimated International Sunspot Number (SSN).
    ///
    /// # Examples
    ///
    /// ```rust
    /// use r4w_core::solar_flare_predictor::SolarRadioAnalyzer;
    ///
    /// // At solar minimum, F10.7 ≈ 65-70 → SSN near 0-10
    /// let ssn = SolarRadioAnalyzer::f107_to_sunspot(70.0);
    /// assert!(ssn >= 0.0 && ssn < 20.0);
    /// ```
    pub fn f107_to_sunspot(f107: f64) -> f64 {
        // F10.7 = 63 + 0.73*SSN + 0.0009*SSN²
        // 0.0009*SSN² + 0.73*SSN + (63 - F10.7) = 0
        // a=0.0009, b=0.73, c=63-f107
        let a = 0.0009;
        let b = 0.73;
        let c = 63.0 - f107;

        let discriminant = b * b - 4.0 * a * c;
        if discriminant < 0.0 {
            return 0.0;
        }

        let ssn = (-b + discriminant.sqrt()) / (2.0 * a);
        ssn.max(0.0)
    }

    /// Detect radio bursts in a signal time series.
    ///
    /// Identifies contiguous regions where the signal power exceeds
    /// the background level by more than the specified threshold.
    ///
    /// # Arguments
    ///
    /// * `signal` - Radio signal power time series (linear power).
    /// * `background` - Estimated background (quiet) power level.
    /// * `threshold_db` - Detection threshold in dB above background.
    ///
    /// # Returns
    ///
    /// Vector of (start_index, end_index) tuples for each detected burst.
    pub fn detect_radio_burst(
        signal: &[f64],
        background: f64,
        threshold_db: f64,
    ) -> Vec<(usize, usize)> {
        if signal.is_empty() || background <= 0.0 {
            return Vec::new();
        }

        let threshold_linear = background * 10.0_f64.powf(threshold_db / 10.0);
        let mut bursts = Vec::new();
        let mut burst_start: Option<usize> = None;

        for (i, &val) in signal.iter().enumerate() {
            if val > threshold_linear {
                if burst_start.is_none() {
                    burst_start = Some(i);
                }
            } else if let Some(start) = burst_start {
                bursts.push((start, i - 1));
                burst_start = None;
            }
        }

        // Handle burst at end of data
        if let Some(start) = burst_start {
            bursts.push((start, signal.len() - 1));
        }

        bursts
    }

    /// Classify a solar radio burst type from its dynamic spectrum.
    ///
    /// Analyzes the frequency drift pattern in a 2D dynamic spectrum
    /// (time vs frequency) to classify the burst type (I-V).
    ///
    /// # Classification criteria
    ///
    /// - **Type I**: Narrow-band, no significant drift
    /// - **Type II**: Slow drift (< 5 MHz/s)
    /// - **Type III**: Fast drift (> 20 MHz/s)
    /// - **Type IV**: Broadband, no clear drift
    /// - **Type V**: Moderate drift, appears after Type III pattern
    ///
    /// # Arguments
    ///
    /// * `dynamic_spectrum` - 2D array [time][frequency] of spectral power.
    /// * `freq_axis` - Frequency values in MHz for each column.
    ///
    /// # Returns
    ///
    /// Classified radio burst type.
    pub fn classify_burst_type(
        dynamic_spectrum: &[Vec<f64>],
        freq_axis: &[f64],
    ) -> RadioBurstType {
        if dynamic_spectrum.is_empty() || freq_axis.is_empty() {
            return RadioBurstType::TypeI;
        }

        let n_time = dynamic_spectrum.len();
        if n_time < 2 {
            return RadioBurstType::TypeI;
        }

        let n_freq = freq_axis.len();

        // Find peak frequency at each time step
        let mut peak_freqs = Vec::with_capacity(n_time);
        let mut peak_powers = Vec::with_capacity(n_time);
        let mut bandwidth_occupied = Vec::with_capacity(n_time);

        for row in dynamic_spectrum.iter() {
            if row.len() < n_freq {
                continue;
            }
            let mut max_val = f64::NEG_INFINITY;
            let mut max_idx = 0;
            let mut above_count = 0;
            let mean: f64 = row.iter().sum::<f64>() / row.len() as f64;

            for (j, &val) in row.iter().enumerate() {
                if val > max_val {
                    max_val = val;
                    max_idx = j;
                }
                if val > mean * 2.0 {
                    above_count += 1;
                }
            }

            if max_idx < freq_axis.len() {
                peak_freqs.push(freq_axis[max_idx]);
            }
            peak_powers.push(max_val);
            bandwidth_occupied.push(above_count as f64 / n_freq as f64);
        }

        if peak_freqs.len() < 2 {
            return RadioBurstType::TypeI;
        }

        // Compute drift rate (MHz/s) - using a time step of 1 second
        let total_freq_drift = peak_freqs.last().unwrap() - peak_freqs.first().unwrap();
        let total_time = peak_freqs.len() as f64 - 1.0;
        let drift_rate = (total_freq_drift / total_time).abs();

        // Average fractional bandwidth
        let avg_bandwidth: f64 =
            bandwidth_occupied.iter().sum::<f64>() / bandwidth_occupied.len() as f64;

        // Classification based on drift rate and bandwidth
        if avg_bandwidth > 0.6 {
            // Broadband continuum
            RadioBurstType::TypeIV
        } else if drift_rate > 20.0 {
            RadioBurstType::TypeIII
        } else if drift_rate > 2.0 {
            RadioBurstType::TypeII
        } else if avg_bandwidth > 0.3 && drift_rate < 5.0 {
            RadioBurstType::TypeV
        } else {
            RadioBurstType::TypeI
        }
    }

    /// Compute the frequency drift rate of an emission feature.
    ///
    /// # Arguments
    ///
    /// * `start_freq` - Starting frequency in MHz.
    /// * `end_freq` - Ending frequency in MHz.
    /// * `duration_s` - Duration of the drift in seconds.
    ///
    /// # Returns
    ///
    /// Drift rate in MHz/s.
    pub fn drift_rate(start_freq: f64, end_freq: f64, duration_s: f64) -> f64 {
        if duration_s == 0.0 {
            return 0.0;
        }
        (end_freq - start_freq) / duration_s
    }
}

/// Geomagnetic storm predictor for space weather forecasting.
///
/// Estimates geomagnetic indices (Kp, Dst) and predicts the impact
/// of solar eruptions (CMEs, proton events) on Earth's magnetosphere.
pub struct GeomagneticStormPredictor;

impl GeomagneticStormPredictor {
    /// Estimate the Kp geomagnetic index from magnetic field variations.
    ///
    /// The Kp index is a quasi-logarithmic measure of geomagnetic activity
    /// on a scale from 0 (quiet) to 9 (extremely disturbed). This function
    /// provides an approximate Kp from the range of horizontal magnetic
    /// field variation over a 3-hour period.
    ///
    /// The conversion uses the standard K-to-nT conversion table scaled
    /// to lower-latitude observatories.
    ///
    /// # Arguments
    ///
    /// * `h_component_nt` - Time series of horizontal magnetic field in nT.
    /// * `sample_rate_hz` - Sample rate in Hz.
    ///
    /// # Returns
    ///
    /// Estimated Kp index (0.0 to 9.0).
    pub fn kp_index_from_variation(h_component_nt: &[f64], _sample_rate_hz: f64) -> f64 {
        if h_component_nt.is_empty() {
            return 0.0;
        }

        let max_val = h_component_nt
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let min_val = h_component_nt
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let range_nt = max_val - min_val;

        // Approximate K-index from range in nT
        // Based on standard IAGA K-to-nT conversion for mid-latitude stations
        // K: 0   1    2    3     4     5     6      7      8      9
        // nT: 0  5   10   20    40    70    120    200    330    500+
        let kp = if range_nt < 5.0 {
            range_nt / 5.0 * 1.0
        } else if range_nt < 10.0 {
            1.0 + (range_nt - 5.0) / 5.0
        } else if range_nt < 20.0 {
            2.0 + (range_nt - 10.0) / 10.0
        } else if range_nt < 40.0 {
            3.0 + (range_nt - 20.0) / 20.0
        } else if range_nt < 70.0 {
            4.0 + (range_nt - 40.0) / 30.0
        } else if range_nt < 120.0 {
            5.0 + (range_nt - 70.0) / 50.0
        } else if range_nt < 200.0 {
            6.0 + (range_nt - 120.0) / 80.0
        } else if range_nt < 330.0 {
            7.0 + (range_nt - 200.0) / 130.0
        } else if range_nt < 500.0 {
            8.0 + (range_nt - 330.0) / 170.0
        } else {
            9.0
        };

        kp.min(9.0).max(0.0)
    }

    /// Compute the Disturbance Storm Time (Dst) index.
    ///
    /// The Dst index measures the globally-averaged depression of the
    /// horizontal magnetic field component caused by the ring current.
    /// It is computed as the average of horizontal field variations
    /// from multiple stations around the magnetic equator.
    ///
    /// # Arguments
    ///
    /// * `h_variations_nt` - Horizontal field deviations from quiet baseline in nT.
    ///   Negative values indicate storm-time depression.
    ///
    /// # Returns
    ///
    /// Estimated Dst index in nT (negative during storms).
    pub fn dst_index(h_variations_nt: &[f64]) -> f64 {
        if h_variations_nt.is_empty() {
            return 0.0;
        }

        // Dst is essentially the average H-component depression
        let sum: f64 = h_variations_nt.iter().sum();
        sum / h_variations_nt.len() as f64
    }

    /// Estimate CME arrival time at Earth using ballistic propagation.
    ///
    /// Assumes constant-speed propagation of the CME through the
    /// heliosphere. In reality, CMEs decelerate due to interaction
    /// with the solar wind, so this gives a lower bound on travel time.
    ///
    /// # Arguments
    ///
    /// * `speed_km_s` - CME speed at the Sun in km/s.
    /// * `distance_au` - Distance from Sun in Astronomical Units (1 AU = Earth).
    ///
    /// # Returns
    ///
    /// Estimated arrival time in hours.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use r4w_core::solar_flare_predictor::GeomagneticStormPredictor;
    ///
    /// // A fast CME at 1000 km/s takes ~42 hours to reach Earth
    /// let hours = GeomagneticStormPredictor::cme_arrival_time_hours(1000.0, 1.0);
    /// assert!((hours - 41.7).abs() < 1.0);
    /// ```
    pub fn cme_arrival_time_hours(speed_km_s: f64, distance_au: f64) -> f64 {
        if speed_km_s <= 0.0 {
            return f64::INFINITY;
        }

        // 1 AU = 1.496e8 km
        let distance_km = distance_au * 1.496e8;
        let time_seconds = distance_km / speed_km_s;
        time_seconds / 3600.0
    }

    /// Estimate the probability of a CME being associated with a flare.
    ///
    /// Based on statistical studies of flare-CME associations.
    /// Higher class flares have significantly higher CME association rates.
    ///
    /// # Arguments
    ///
    /// * `flare_class` - GOES flare classification.
    ///
    /// # Returns
    ///
    /// Probability of CME association (0.0 to 1.0).
    pub fn flare_to_cme_probability(flare_class: &FlareClass) -> f64 {
        match flare_class {
            FlareClass::A => 0.01,
            FlareClass::B => 0.05,
            FlareClass::C => 0.20,
            FlareClass::M => 0.50,
            FlareClass::X => 0.90,
        }
    }

    /// Estimate the probability of a solar proton event (SPE).
    ///
    /// Solar proton events are energetic particle events that can
    /// damage satellites and create radiation hazards. The probability
    /// depends on both the flare class and any associated CME speed.
    ///
    /// # Arguments
    ///
    /// * `flare_class` - GOES flare classification.
    /// * `cme_speed_km_s` - Estimated CME speed in km/s (0 if no CME detected).
    ///
    /// # Returns
    ///
    /// Probability of a significant proton event (>10 MeV flux > 10 pfu).
    pub fn proton_event_probability(flare_class: &FlareClass, cme_speed_km_s: f64) -> f64 {
        let base_prob: f64 = match flare_class {
            FlareClass::A => 0.001,
            FlareClass::B => 0.005,
            FlareClass::C => 0.02,
            FlareClass::M => 0.10,
            FlareClass::X => 0.40,
        };

        // CME speed enhancement factor
        // Fast CMEs (>1000 km/s) significantly increase proton event probability
        let cme_factor: f64 = if cme_speed_km_s > 1500.0 {
            3.0
        } else if cme_speed_km_s > 1000.0 {
            2.0
        } else if cme_speed_km_s > 500.0 {
            1.5
        } else {
            1.0
        };

        (base_prob * cme_factor).min(1.0_f64)
    }
}

/// Solar cycle model for long-term activity prediction.
///
/// Models the approximately 11-year solar cycle using empirical
/// relationships between sunspot number, F10.7 flux, and cycle phase.
pub struct SolarCycleModel;

impl SolarCycleModel {
    /// Predict the sunspot number using a McNish-Lincoln-like method.
    ///
    /// Models the solar cycle as a skewed sinusoidal function with
    /// a typical period of ~132 months (11 years). The amplitude
    /// parameter scales the peak sunspot number.
    ///
    /// # Arguments
    ///
    /// * `month_since_minimum` - Months elapsed since the last solar minimum.
    /// * `amplitude` - Expected peak sunspot number for this cycle (e.g., 180 for a strong cycle).
    ///
    /// # Returns
    ///
    /// Predicted monthly sunspot number.
    ///
    /// # Examples
    ///
    /// ```rust
    /// use r4w_core::solar_flare_predictor::SolarCycleModel;
    ///
    /// // At minimum (month 0), SSN should be near 0
    /// let ssn_min = SolarCycleModel::sunspot_number_prediction(0.0, 180.0);
    /// assert!(ssn_min < 10.0);
    ///
    /// // At maximum (~66 months), SSN should be near amplitude
    /// let ssn_max = SolarCycleModel::sunspot_number_prediction(66.0, 180.0);
    /// assert!(ssn_max > 100.0);
    /// ```
    pub fn sunspot_number_prediction(month_since_minimum: f64, amplitude: f64) -> f64 {
        // Simple parameterized solar cycle model
        // Peak occurs around month 60 (5 years after minimum)
        // Full cycle period ~132 months (~11 years)
        // Using sin(pi * t / 132) which peaks at t=66
        let period = 132.0; // ~11 years in months
        let phase = std::f64::consts::PI * month_since_minimum / period;

        // sin(phase) peaks at phase = PI/2, i.e. month = period/2 = 66
        let raw = phase.sin();

        // Apply amplitude scaling and ensure non-negative
        // sin² gives a sharper, more realistic peak shape
        let ssn = amplitude * raw * raw;
        ssn.max(0.0)
    }

    /// Compute the F10.7 solar flux index from the sunspot number.
    ///
    /// Uses the empirical quadratic relationship:
    ///   F10.7 = 63 + 0.73*SSN + 0.0009*SSN²
    ///
    /// This relationship was established by Tapping (2013) and provides
    /// a good fit to historical data.
    ///
    /// # Arguments
    ///
    /// * `ssn` - International Sunspot Number.
    ///
    /// # Returns
    ///
    /// F10.7 solar flux index in SFU (Solar Flux Units).
    ///
    /// # Examples
    ///
    /// ```rust
    /// use r4w_core::solar_flare_predictor::SolarCycleModel;
    ///
    /// // At solar minimum (SSN ≈ 0), F10.7 ≈ 63 SFU
    /// let f107_min = SolarCycleModel::f107_from_ssn(0.0);
    /// assert!((f107_min - 63.0).abs() < 0.1);
    ///
    /// // At moderate activity (SSN ≈ 100)
    /// let f107 = SolarCycleModel::f107_from_ssn(100.0);
    /// assert!(f107 > 130.0 && f107 < 150.0);
    /// ```
    pub fn f107_from_ssn(ssn: f64) -> f64 {
        63.0 + 0.73 * ssn + 0.0009 * ssn * ssn
    }

    /// Determine the current phase of the solar cycle.
    ///
    /// Returns a normalized phase value where 0.0 represents solar
    /// minimum and 1.0 represents solar maximum.
    ///
    /// # Arguments
    ///
    /// * `current_ssn` - Current sunspot number.
    /// * `solar_max_ssn` - Expected or observed solar maximum sunspot number.
    ///
    /// # Returns
    ///
    /// Cycle phase from 0.0 (minimum) to 1.0 (maximum).
    ///
    /// # Examples
    ///
    /// ```rust
    /// use r4w_core::solar_flare_predictor::SolarCycleModel;
    ///
    /// let phase = SolarCycleModel::cycle_phase(180.0, 180.0);
    /// assert!((phase - 1.0).abs() < 1e-10);
    ///
    /// let phase = SolarCycleModel::cycle_phase(0.0, 180.0);
    /// assert!((phase - 0.0).abs() < 1e-10);
    /// ```
    pub fn cycle_phase(current_ssn: f64, solar_max_ssn: f64) -> f64 {
        if solar_max_ssn <= 0.0 {
            return 0.0;
        }

        let phase = current_ssn / solar_max_ssn;
        phase.min(1.0).max(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ==========================================================
    // GOES Classification Tests
    // ==========================================================

    #[test]
    fn test_flare_class_m1() {
        assert_eq!(FlareClass::from_peak_flux(1e-5), FlareClass::M);
    }

    #[test]
    fn test_flare_class_x1() {
        assert_eq!(FlareClass::from_peak_flux(1e-4), FlareClass::X);
    }

    #[test]
    fn test_flare_class_c1() {
        assert_eq!(FlareClass::from_peak_flux(1e-6), FlareClass::C);
    }

    #[test]
    fn test_flare_class_b1() {
        assert_eq!(FlareClass::from_peak_flux(1e-7), FlareClass::B);
    }

    #[test]
    fn test_flare_class_a() {
        assert_eq!(FlareClass::from_peak_flux(5e-9), FlareClass::A);
    }

    #[test]
    fn test_flare_class_x_large() {
        assert_eq!(FlareClass::from_peak_flux(1e-3), FlareClass::X);
    }

    #[test]
    fn test_numeric_class_m1() {
        let sub = FlareClass::numeric_class(1e-5);
        assert!((sub - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_numeric_class_m5_3() {
        let sub = FlareClass::numeric_class(5.3e-5);
        assert!((sub - 5.3).abs() < 0.01);
    }

    #[test]
    fn test_numeric_class_x2() {
        let sub = FlareClass::numeric_class(2e-4);
        assert!((sub - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_class_string_m1() {
        assert_eq!(FlareClass::to_class_string(1e-5), "M1.0");
    }

    #[test]
    fn test_class_string_x1() {
        assert_eq!(FlareClass::to_class_string(1e-4), "X1.0");
    }

    #[test]
    fn test_class_string_c5() {
        assert_eq!(FlareClass::to_class_string(5e-6), "C5.0");
    }

    // ==========================================================
    // SFU Conversion Tests
    // ==========================================================

    #[test]
    fn test_sfu_to_wm2hz() {
        let flux = SolarRadioAnalyzer::solar_flux_unit_to_wm2hz(1.0);
        assert!((flux - 1e-22).abs() < 1e-30);
    }

    #[test]
    fn test_sfu_conversion_100() {
        let flux = SolarRadioAnalyzer::solar_flux_unit_to_wm2hz(100.0);
        assert!((flux - 1e-20).abs() < 1e-28);
    }

    // ==========================================================
    // F10.7 / Sunspot Number Tests
    // ==========================================================

    #[test]
    fn test_f107_to_sunspot_low() {
        let ssn = SolarRadioAnalyzer::f107_to_sunspot(70.0);
        assert!(ssn >= 0.0 && ssn < 20.0);
    }

    #[test]
    fn test_f107_to_sunspot_roundtrip() {
        let ssn_original = 100.0;
        let f107 = SolarCycleModel::f107_from_ssn(ssn_original);
        let ssn_recovered = SolarRadioAnalyzer::f107_to_sunspot(f107);
        assert!((ssn_recovered - ssn_original).abs() < 1.0);
    }

    #[test]
    fn test_f107_from_ssn_minimum() {
        let f107 = SolarCycleModel::f107_from_ssn(0.0);
        assert!((f107 - 63.0).abs() < 0.1);
    }

    #[test]
    fn test_f107_from_ssn_moderate() {
        let f107 = SolarCycleModel::f107_from_ssn(100.0);
        // 63 + 73 + 9 = 145
        assert!((f107 - 145.0).abs() < 1.0);
    }

    // ==========================================================
    // Flare Onset Detection Tests
    // ==========================================================

    #[test]
    fn test_flare_onset_detection_rising_signal() {
        let config = SpaceWeatherConfig {
            sample_rate_hz: 1.0,
            baseline_flux_wm2: 1e-8,
            detection_threshold_factor: 10.0,
            xray_channel: XrayChannel::Long,
        };
        let detector = SolarFlareDetector::new(config);

        // Quiet sun followed by a flare
        let mut xray: Vec<f64> = vec![1e-8; 50];
        for i in 0..50 {
            xray.push(1e-8 + (i as f64 + 1.0) * 1e-7);
        }

        let onsets = detector.detect_flare_onset(&xray, 10.0);
        assert!(!onsets.is_empty());
        // Onset should be detected around sample 50
        assert!(onsets[0] >= 50 && onsets[0] <= 55);
    }

    #[test]
    fn test_no_flare_in_quiet_data() {
        let config = SpaceWeatherConfig::default();
        let detector = SolarFlareDetector::new(config);

        let xray = vec![1e-8; 100];
        let onsets = detector.detect_flare_onset(&xray, 10.0);
        assert!(onsets.is_empty());
    }

    #[test]
    fn test_flare_peak_flux() {
        let config = SpaceWeatherConfig::default();
        let detector = SolarFlareDetector::new(config);

        let mut xray = vec![1e-8; 10];
        xray.extend_from_slice(&[1e-7, 5e-7, 1e-6, 5e-6, 1e-5, 5e-6, 1e-6, 5e-7, 1e-7]);

        let (peak_idx, peak_val) = detector.flare_peak_flux(&xray, 10);
        assert_eq!(peak_idx, 14);
        assert!((peak_val - 1e-5).abs() < 1e-10);
    }

    // ==========================================================
    // Rise Time / Decay Time Tests
    // ==========================================================

    #[test]
    fn test_rise_time_calculation() {
        let config = SpaceWeatherConfig::default();
        let detector = SolarFlareDetector::new(config);

        let xray = vec![0.0; 100]; // dummy
        let rise = detector.rise_time(&xray, 10, 50, 1.0);
        assert!((rise - 40.0).abs() < 0.01);
    }

    #[test]
    fn test_rise_time_less_than_decay_typical_flare() {
        let config = SpaceWeatherConfig {
            sample_rate_hz: 1.0,
            baseline_flux_wm2: 1e-8,
            ..Default::default()
        };
        let detector = SolarFlareDetector::new(config);

        // Simulate a flare with fast rise (10 samples) and slow decay (90 samples)
        let mut xray = vec![1e-8; 10]; // quiet
        // Rise phase: 10 samples
        for i in 0..10 {
            xray.push(1e-8 + (i as f64 + 1.0) / 10.0 * 1e-5);
        }
        // Decay phase: 90 samples, exponential decay
        let peak_val = 1e-5;
        for i in 0..90 {
            let decay = peak_val * (-i as f64 / 30.0).exp() + 1e-8;
            xray.push(decay);
        }

        let onset_idx = 10;
        let peak_idx = 19;

        let rise = detector.rise_time(&xray, onset_idx, peak_idx, 1.0);
        let decay = detector.decay_time(&xray, peak_idx, 1.0);

        assert!(rise < decay, "Rise time ({}) should be less than decay time ({})", rise, decay);
    }

    // ==========================================================
    // CME Arrival Time Tests
    // ==========================================================

    #[test]
    fn test_cme_arrival_time_1000kms() {
        let hours = GeomagneticStormPredictor::cme_arrival_time_hours(1000.0, 1.0);
        // 1 AU = 1.496e8 km, at 1000 km/s: 1.496e8 / 1000 = 149600 s ≈ 41.6 hours
        assert!((hours - 41.6).abs() < 1.0);
    }

    #[test]
    fn test_cme_arrival_time_fast_cme() {
        let hours = GeomagneticStormPredictor::cme_arrival_time_hours(2000.0, 1.0);
        assert!((hours - 20.8).abs() < 1.0);
    }

    #[test]
    fn test_cme_arrival_zero_speed() {
        let hours = GeomagneticStormPredictor::cme_arrival_time_hours(0.0, 1.0);
        assert!(hours.is_infinite());
    }

    // ==========================================================
    // Kp Index Tests
    // ==========================================================

    #[test]
    fn test_kp_index_quiet() {
        let h_data = vec![0.0; 100];
        let kp = GeomagneticStormPredictor::kp_index_from_variation(&h_data, 1.0);
        assert!(kp >= 0.0 && kp <= 1.0);
    }

    #[test]
    fn test_kp_index_range() {
        // Moderate variation: 50 nT range → Kp ~4-5
        let h_data: Vec<f64> = (0..100).map(|i| 25.0 * (i as f64 * 0.1).sin()).collect();
        let kp = GeomagneticStormPredictor::kp_index_from_variation(&h_data, 1.0);
        assert!(kp >= 0.0 && kp <= 9.0);
    }

    #[test]
    fn test_kp_index_extreme_storm() {
        // Very large variation: 600 nT
        let h_data = vec![0.0, 600.0];
        let kp = GeomagneticStormPredictor::kp_index_from_variation(&h_data, 1.0);
        assert!((kp - 9.0).abs() < 0.01);
    }

    // ==========================================================
    // Radio Burst Detection Tests
    // ==========================================================

    #[test]
    fn test_type_ii_drift_rate() {
        let rate = SolarRadioAnalyzer::drift_rate(200.0, 100.0, 100.0);
        assert!((rate - (-1.0)).abs() < 0.01);
    }

    #[test]
    fn test_type_iii_drift_rate() {
        let rate = SolarRadioAnalyzer::drift_rate(200.0, 0.0, 2.0);
        assert!((rate - (-100.0)).abs() < 0.01);
    }

    #[test]
    fn test_detect_radio_burst_single() {
        let mut signal = vec![1.0; 50];
        for _ in 0..10 {
            signal.push(100.0); // burst
        }
        signal.extend(vec![1.0; 50]);

        let bursts = SolarRadioAnalyzer::detect_radio_burst(&signal, 1.0, 10.0);
        assert_eq!(bursts.len(), 1);
        assert_eq!(bursts[0].0, 50);
        assert_eq!(bursts[0].1, 59);
    }

    #[test]
    fn test_detect_radio_burst_empty() {
        let bursts = SolarRadioAnalyzer::detect_radio_burst(&[], 1.0, 10.0);
        assert!(bursts.is_empty());
    }

    #[test]
    fn test_classify_burst_type_iii() {
        // Create a dynamic spectrum with fast frequency drift (Type III)
        // Freq axis: 50 to 548 MHz (100 bins, 5 MHz spacing)
        // Peak drifts from high freq to low freq at ~50 MHz/time step (> 20 MHz/s threshold)
        let n_time = 10;
        let n_freq = 100;
        let freq_axis: Vec<f64> = (0..n_freq).map(|i| 50.0 + i as f64 * 5.0).collect();

        let mut spectrum = Vec::new();
        for t in 0..n_time {
            let mut row = vec![1.0; n_freq];
            // Fast downward drift: start at high freq bin, move 10 bins per step
            let peak_bin = (n_freq - 1).saturating_sub(t * 10);
            row[peak_bin] = 100.0;
            if peak_bin > 0 {
                row[peak_bin - 1] = 50.0;
            }
            spectrum.push(row);
        }

        let burst_type = SolarRadioAnalyzer::classify_burst_type(&spectrum, &freq_axis);
        assert_eq!(burst_type, RadioBurstType::TypeIII);
    }

    // ==========================================================
    // Solar Cycle Tests
    // ==========================================================

    #[test]
    fn test_solar_cycle_minimum() {
        let ssn = SolarCycleModel::sunspot_number_prediction(0.0, 180.0);
        assert!(ssn < 10.0, "SSN at minimum should be near 0, got {}", ssn);
    }

    #[test]
    fn test_solar_cycle_maximum() {
        let ssn = SolarCycleModel::sunspot_number_prediction(60.0, 180.0);
        assert!(ssn > 100.0, "SSN near maximum should be > 100, got {}", ssn);
    }

    #[test]
    fn test_cycle_phase_at_maximum() {
        let phase = SolarCycleModel::cycle_phase(180.0, 180.0);
        assert!((phase - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_cycle_phase_at_minimum() {
        let phase = SolarCycleModel::cycle_phase(0.0, 180.0);
        assert!((phase - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_cycle_phase_half() {
        let phase = SolarCycleModel::cycle_phase(90.0, 180.0);
        assert!((phase - 0.5).abs() < 1e-10);
    }

    // ==========================================================
    // Flare-CME Probability Tests
    // ==========================================================

    #[test]
    fn test_cme_probability_increases_with_class() {
        let prob_c = GeomagneticStormPredictor::flare_to_cme_probability(&FlareClass::C);
        let prob_m = GeomagneticStormPredictor::flare_to_cme_probability(&FlareClass::M);
        let prob_x = GeomagneticStormPredictor::flare_to_cme_probability(&FlareClass::X);

        assert!(prob_c < prob_m, "C ({}) should be < M ({})", prob_c, prob_m);
        assert!(prob_m < prob_x, "M ({}) should be < X ({})", prob_m, prob_x);
    }

    #[test]
    fn test_cme_probability_x_class_high() {
        let prob = GeomagneticStormPredictor::flare_to_cme_probability(&FlareClass::X);
        assert!(prob >= 0.8);
    }

    // ==========================================================
    // Neupert Effect Tests
    // ==========================================================

    #[test]
    fn test_neupert_effect_rising_phase() {
        let config = SpaceWeatherConfig::default();
        let detector = SolarFlareDetector::new(config);

        // Rising flux: derivative should be positive
        let xray: Vec<f64> = (0..50).map(|i| 1e-8 + i as f64 * 1e-7).collect();
        let deriv = detector.neupert_effect(&xray, 1.0);

        assert!(!deriv.is_empty());
        // All derivatives should be positive during rise
        for &d in &deriv {
            assert!(d > 0.0, "Derivative should be positive during impulsive phase, got {}", d);
        }
    }

    #[test]
    fn test_neupert_effect_length() {
        let config = SpaceWeatherConfig::default();
        let detector = SolarFlareDetector::new(config);

        let xray = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let deriv = detector.neupert_effect(&xray, 1.0);
        assert_eq!(deriv.len(), 4);
    }

    #[test]
    fn test_neupert_effect_empty() {
        let config = SpaceWeatherConfig::default();
        let detector = SolarFlareDetector::new(config);

        let deriv = detector.neupert_effect(&[], 1.0);
        assert!(deriv.is_empty());
    }

    // ==========================================================
    // Proton Event Probability Tests
    // ==========================================================

    #[test]
    fn test_proton_event_x_greater_than_m() {
        let prob_m = GeomagneticStormPredictor::proton_event_probability(&FlareClass::M, 0.0);
        let prob_x = GeomagneticStormPredictor::proton_event_probability(&FlareClass::X, 0.0);
        assert!(prob_x > prob_m, "X-class ({}) should have higher proton event prob than M-class ({})", prob_x, prob_m);
    }

    #[test]
    fn test_proton_event_cme_enhancement() {
        let prob_no_cme = GeomagneticStormPredictor::proton_event_probability(&FlareClass::M, 0.0);
        let prob_fast_cme = GeomagneticStormPredictor::proton_event_probability(&FlareClass::M, 1500.0);
        assert!(prob_fast_cme > prob_no_cme);
    }

    #[test]
    fn test_proton_event_probability_bounded() {
        let prob = GeomagneticStormPredictor::proton_event_probability(&FlareClass::X, 2000.0);
        assert!(prob <= 1.0);
        assert!(prob >= 0.0);
    }

    // ==========================================================
    // Dst Index Tests
    // ==========================================================

    #[test]
    fn test_dst_index_quiet() {
        let h_var = vec![0.0; 100];
        let dst = GeomagneticStormPredictor::dst_index(&h_var);
        assert!((dst - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_dst_index_storm() {
        // During storms, H component is depressed (negative)
        let h_var = vec![-100.0; 100];
        let dst = GeomagneticStormPredictor::dst_index(&h_var);
        assert!((dst - (-100.0)).abs() < 1e-10);
    }

    // ==========================================================
    // Configuration and Edge Case Tests
    // ==========================================================

    #[test]
    fn test_default_config() {
        let config = SpaceWeatherConfig::default();
        assert_eq!(config.sample_rate_hz, 1.0);
        assert_eq!(config.xray_channel, XrayChannel::Long);
        assert!((config.baseline_flux_wm2 - 1e-8).abs() < 1e-15);
        assert!((config.detection_threshold_factor - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_goes_classification_via_detector() {
        let detector = SolarFlareDetector::new(SpaceWeatherConfig::default());

        let (class, sub) = detector.goes_classification(1e-5);
        assert_eq!(class, FlareClass::M);
        assert!((sub - 1.0).abs() < 0.1);

        let (class, sub) = detector.goes_classification(1e-4);
        assert_eq!(class, FlareClass::X);
        assert!((sub - 1.0).abs() < 0.1);

        let (class, sub) = detector.goes_classification(1e-6);
        assert_eq!(class, FlareClass::C);
        assert!((sub - 1.0).abs() < 0.1);
    }

    #[test]
    fn test_flare_duration() {
        let config = SpaceWeatherConfig::default();
        let detector = SolarFlareDetector::new(config);

        // Triangular flare shape
        let mut xray = vec![1e-8; 10];
        for i in 0..10 {
            xray.push(1e-8 + (i as f64 + 1.0) * 1e-6);
        }
        for i in (0..10).rev() {
            xray.push(1e-8 + i as f64 * 1e-6);
        }
        xray.extend(vec![1e-8; 10]);

        let half_peak = 5e-6;
        let dur = detector.flare_duration(&xray, 10, half_peak);
        assert!(dur > 0.0);
    }

    #[test]
    fn test_drift_rate_zero_duration() {
        let rate = SolarRadioAnalyzer::drift_rate(100.0, 200.0, 0.0);
        assert!((rate - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_xray_channel_enum() {
        let long = XrayChannel::Long;
        let short = XrayChannel::Short;
        assert_ne!(long, short);
    }

    #[test]
    fn test_radio_burst_type_enum() {
        let t2 = RadioBurstType::TypeII;
        let t3 = RadioBurstType::TypeIII;
        assert_ne!(t2, t3);
    }
}
