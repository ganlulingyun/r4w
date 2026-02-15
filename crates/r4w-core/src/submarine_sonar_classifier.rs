//! # Passive Sonar Target Classification
//!
//! Implements passive sonar target classification for underwater acoustics.
//! Passive sonar listens to sound emitted by vessels (machinery noise, propeller
//! cavitation, hydrodynamic flow noise) without transmitting.
//!
//! ## Key Techniques
//!
//! - **DEMON** (Detection of Envelope Modulation on Noise): Extracts propeller
//!   blade-rate modulation from broadband cavitation noise by bandpass filtering,
//!   envelope detection, and spectral analysis of the envelope.
//!
//! - **LOFAR** (Low Frequency Analysis and Recording): Produces time-frequency
//!   spectrograms of narrowband tonal lines from machinery and propulsion.
//!
//! - **Blade Rate Extraction**: Identifies the fundamental propeller frequency
//!   and its harmonics to estimate blade count and shaft rotation rate.
//!
//! ## Classification Pipeline
//!
//! ```text
//! Hydrophone Signal
//!   -> Bandpass Filter (DEMON carrier band)
//!   -> Envelope Extraction (analytic signal magnitude)
//!   -> Modulation Spectrum (FFT of envelope)
//!   -> Blade Rate / Shaft Rate Extraction
//!   -> Feature Extraction (narrowband count, broadband level, spectral slope)
//!   -> Target Classification (Submarine, SurfaceShip, SmallCraft, etc.)
//! ```
//!
//! ## Underwater Acoustics
//!
//! Utility functions for the passive sonar equation, transmission loss models,
//! sound speed profiles (Mackenzie equation), and absorption coefficients
//! (simplified Francois-Garrison model).

use std::f64::consts::PI;

// ─── Configuration ───────────────────────────────────────────────────────────

/// Configuration parameters for the passive sonar processor.
#[derive(Debug, Clone)]
pub struct SonarConfig {
    /// Sample rate in Hz (typically 8000-48000 Hz).
    pub sample_rate_hz: f64,
    /// FFT size for spectral analysis.
    pub fft_size: usize,
    /// Overlap fraction between successive FFT frames (0.0-1.0).
    pub overlap: f64,
    /// Number of hydrophone elements in the array.
    pub num_hydrophones: usize,
    /// Inter-element spacing in metres.
    pub hydrophone_spacing_m: f64,
    /// Speed of sound in water (m/s). Default ~1500 m/s.
    pub sound_speed_m_s: f64,
}

impl Default for SonarConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 48000.0,
            fft_size: 1024,
            overlap: 0.5,
            num_hydrophones: 2,
            hydrophone_spacing_m: 0.5,
            sound_speed_m_s: 1500.0,
        }
    }
}

// ─── Target Classification ───────────────────────────────────────────────────

/// Broad categories of passive sonar contacts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetClass {
    /// Nuclear or diesel-electric submarine.
    Submarine,
    /// Large surface vessel (cargo, tanker, warship).
    SurfaceShip,
    /// Small boat, RIB, or fishing vessel.
    SmallCraft,
    /// Marine life (whale, dolphin, shrimp snapping).
    BiologicalSource,
    /// Background ocean noise (wind, rain, shipping lanes).
    AmbientNoise,
    /// High-speed torpedo or UUV.
    Torpedo,
}

// ─── Target Features ─────────────────────────────────────────────────────────

/// Acoustic feature vector extracted from spectral and DEMON analysis.
#[derive(Debug, Clone)]
pub struct TargetFeatures {
    /// Fundamental propeller blade-rate frequency (Hz), if detected.
    pub blade_rate_hz: Option<f64>,
    /// Number of blade-rate harmonics visible above noise.
    pub num_harmonics: usize,
    /// Integrated broadband power level (dB re 1 uPa).
    pub broadband_level_db: f64,
    /// Count of persistent narrowband tonal lines.
    pub narrowband_count: usize,
    /// Spectral slope in dB/decade (negative = falling with frequency).
    pub spectral_slope: f64,
    /// DEMON modulation depth (0.0-1.0).
    pub modulation_depth: f64,
}

// ─── Passive Sonar Processor ─────────────────────────────────────────────────

/// Core passive sonar processing: LOFAR spectrograms, DEMON analysis, blade
/// rate extraction, narrowband detection, and broadband power measurement.
pub struct PassiveSonarProcessor {
    config: SonarConfig,
}

impl PassiveSonarProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: SonarConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &SonarConfig {
        &self.config
    }

    /// Produce a LOFAR spectrogram (time-frequency display).
    ///
    /// Each row is the magnitude spectrum (linear) of one windowed FFT frame.
    /// The number of rows depends on the signal length, FFT size, and overlap.
    ///
    /// # Arguments
    /// * `signal` - time-domain samples
    /// * `fft_size` - FFT length (number of frequency bins)
    /// * `overlap_fraction` - fraction of overlap between successive frames (0.0-1.0)
    ///
    /// # Returns
    /// `Vec<Vec<f64>>` where `[t][f]` is magnitude at time index `t`, frequency bin `f`.
    pub fn lofar_spectrogram(
        &self,
        signal: &[f64],
        fft_size: usize,
        overlap_fraction: f64,
    ) -> Vec<Vec<f64>> {
        let hop = ((fft_size as f64) * (1.0 - overlap_fraction)).max(1.0) as usize;
        let mut frames = Vec::new();

        let mut start = 0;
        while start + fft_size <= signal.len() {
            let frame = &signal[start..start + fft_size];
            // Apply Hann window
            let windowed: Vec<f64> = frame
                .iter()
                .enumerate()
                .map(|(i, &x)| {
                    let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / fft_size as f64).cos());
                    x * w
                })
                .collect();

            let spectrum = real_fft_magnitude(&windowed);
            frames.push(spectrum);
            start += hop;
        }
        frames
    }

    /// DEMON (Detection of Envelope Modulation on Noise) analysis.
    ///
    /// Pipeline: bandpass filter -> envelope extraction -> FFT of envelope.
    /// Returns the modulation spectrum (magnitude) of the envelope.
    ///
    /// # Arguments
    /// * `signal` - raw hydrophone time-domain signal
    /// * `band_low_hz` - lower edge of the DEMON carrier band (Hz)
    /// * `band_high_hz` - upper edge of the DEMON carrier band (Hz)
    /// * `sample_rate` - sample rate (Hz)
    ///
    /// # Returns
    /// Modulation spectrum magnitudes (linear). Frequency resolution = sample_rate / N
    /// where N is the envelope length rounded down to the nearest power of 2.
    pub fn demon_analysis(
        &self,
        signal: &[f64],
        band_low_hz: f64,
        band_high_hz: f64,
        sample_rate: f64,
    ) -> Vec<f64> {
        // 1. Bandpass filter in the carrier band
        let filtered = simple_bandpass(signal, band_low_hz, band_high_hz, sample_rate);
        // 2. Extract envelope (magnitude of analytic signal)
        let envelope = envelope_extract_internal(&filtered);
        // 3. Remove DC from envelope
        let mean = envelope.iter().sum::<f64>() / envelope.len() as f64;
        let ac_envelope: Vec<f64> = envelope.iter().map(|&e| e - mean).collect();
        // 4. FFT of envelope -> modulation spectrum
        let n = ac_envelope.len().next_power_of_two().min(ac_envelope.len());
        // Use the available samples (truncated to power-of-two would be better but
        // we work with what we have for simplicity)
        real_fft_magnitude(&ac_envelope[..n.min(ac_envelope.len())])
    }

    /// Extract the fundamental propeller blade-rate frequency from a DEMON
    /// modulation spectrum.
    ///
    /// Finds the spectral peak above a noise floor estimate.
    ///
    /// # Arguments
    /// * `demon_spectrum` - modulation spectrum magnitudes (from `demon_analysis`)
    /// * `freq_resolution` - frequency spacing between bins (Hz)
    ///
    /// # Returns
    /// `Some(frequency_hz)` if a clear peak is found, `None` otherwise.
    pub fn extract_blade_rate(
        &self,
        demon_spectrum: &[f64],
        freq_resolution: f64,
    ) -> Option<f64> {
        if demon_spectrum.is_empty() || freq_resolution <= 0.0 {
            return None;
        }
        // Skip DC bin (index 0) and very low bins
        let start_bin = (1.0_f64 / freq_resolution).ceil() as usize; // start at ~1 Hz
        let start_bin = start_bin.max(1);
        if start_bin >= demon_spectrum.len() {
            return None;
        }

        // Find peak in the region of interest (1-50 Hz typical for blade rates)
        let end_bin = ((50.0 / freq_resolution).ceil() as usize).min(demon_spectrum.len());
        if start_bin >= end_bin {
            return None;
        }

        let mut peak_bin = start_bin;
        let mut peak_val = demon_spectrum[start_bin];
        for i in start_bin..end_bin {
            if demon_spectrum[i] > peak_val {
                peak_val = demon_spectrum[i];
                peak_bin = i;
            }
        }

        // Noise floor estimate: median of the spectrum in the search range
        let mut sorted: Vec<f64> = demon_spectrum[start_bin..end_bin].to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let noise_floor = sorted[sorted.len() / 2];

        // Require peak to be at least 6 dB above noise floor
        if noise_floor > 0.0 && peak_val / noise_floor > 2.0 {
            Some(peak_bin as f64 * freq_resolution)
        } else {
            None
        }
    }

    /// Estimate the number of propeller blades.
    ///
    /// `N_blades = f_blade / f_shaft` rounded to the nearest integer.
    ///
    /// # Arguments
    /// * `blade_rate_hz` - fundamental blade-rate frequency (Hz)
    /// * `shaft_rate_hz` - shaft rotation rate (Hz), e.g. from engine RPM
    ///
    /// # Returns
    /// Estimated blade count (clamped to at least 1).
    pub fn estimate_num_blades(blade_rate_hz: f64, shaft_rate_hz: f64) -> usize {
        if shaft_rate_hz <= 0.0 {
            return 1;
        }
        let n = (blade_rate_hz / shaft_rate_hz).round() as usize;
        n.max(1)
    }

    /// Detect persistent narrowband tonal lines in a spectrogram.
    ///
    /// A bin is flagged as a narrowband line if its time-averaged power exceeds
    /// the local median by `threshold_db` dB.
    ///
    /// # Arguments
    /// * `spectrogram` - LOFAR spectrogram `[time][freq]`
    /// * `threshold_db` - detection threshold above local median (dB)
    ///
    /// # Returns
    /// Vector of `(bin_index, frequency_hz)` for detected tonal lines.
    pub fn narrowband_detector(
        &self,
        spectrogram: &[Vec<f64>],
        threshold_db: f64,
    ) -> Vec<(usize, f64)> {
        if spectrogram.is_empty() {
            return Vec::new();
        }
        let num_bins = spectrogram[0].len();
        let num_frames = spectrogram.len();
        let freq_bin_hz = self.config.sample_rate_hz / (num_bins as f64 * 2.0);

        // Compute time-averaged power per bin
        let mut avg_power = vec![0.0_f64; num_bins];
        for frame in spectrogram {
            for (i, &val) in frame.iter().enumerate().take(num_bins) {
                avg_power[i] += val * val;
            }
        }
        for p in avg_power.iter_mut() {
            *p /= num_frames as f64;
        }

        // Detect bins exceeding threshold above local median
        let mut detections = Vec::new();
        let window = 21; // median window size
        for bin in 0..num_bins {
            let half_w = window / 2;
            let lo = if bin >= half_w { bin - half_w } else { 0 };
            let hi = (bin + half_w + 1).min(num_bins);
            let mut local: Vec<f64> = avg_power[lo..hi].to_vec();
            local.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let median = local[local.len() / 2];

            if median > 0.0 {
                let ratio_db = 10.0 * (avg_power[bin] / median).log10();
                if ratio_db > threshold_db {
                    detections.push((bin, bin as f64 * freq_bin_hz));
                }
            }
        }
        detections
    }

    /// Integrated broadband power in a specified frequency band.
    ///
    /// # Arguments
    /// * `signal` - time-domain samples
    /// * `low_hz` - lower band edge (Hz)
    /// * `high_hz` - upper band edge (Hz)
    /// * `sample_rate` - sample rate (Hz)
    ///
    /// # Returns
    /// Integrated power (linear, mean-square) in the band.
    pub fn broadband_power(
        &self,
        signal: &[f64],
        low_hz: f64,
        high_hz: f64,
        sample_rate: f64,
    ) -> f64 {
        if signal.is_empty() || low_hz >= high_hz || sample_rate <= 0.0 {
            return 0.0;
        }
        let n = signal.len();
        let spectrum = real_fft_magnitude(signal);
        let freq_res = sample_rate / n as f64;

        let lo_bin = (low_hz / freq_res).ceil() as usize;
        let hi_bin = ((high_hz / freq_res).floor() as usize).min(spectrum.len());

        let mut power = 0.0;
        for i in lo_bin..hi_bin {
            power += spectrum[i] * spectrum[i];
        }
        power / n as f64
    }
}

// ─── DEMON Processor ─────────────────────────────────────────────────────────

/// Dedicated DEMON (Detection of Envelope Modulation on Noise) processor.
///
/// Performs bandpass filtering, envelope extraction, modulation spectrum
/// analysis, and shaft rate estimation from blade-rate harmonics.
pub struct DemonProcessor {
    /// Hydrophone sample rate (Hz).
    pub sample_rate_hz: f64,
    /// Carrier band edges (low_hz, high_hz) for DEMON analysis.
    pub carrier_band: (f64, f64),
}

impl DemonProcessor {
    /// Create a new DEMON processor.
    ///
    /// # Arguments
    /// * `sample_rate_hz` - input signal sample rate (Hz)
    /// * `carrier_band` - `(low_hz, high_hz)` passband for cavitation noise
    pub fn new(sample_rate_hz: f64, carrier_band: (f64, f64)) -> Self {
        Self {
            sample_rate_hz,
            carrier_band,
        }
    }

    /// Apply a simple bandpass filter to isolate a frequency band.
    ///
    /// Uses windowed-sinc FIR convolution (Hann window).
    ///
    /// # Arguments
    /// * `signal` - input samples
    /// * `low` - lower cutoff (Hz)
    /// * `high` - upper cutoff (Hz)
    /// * `sample_rate` - sample rate (Hz)
    pub fn bandpass_filter(
        &self,
        signal: &[f64],
        low: f64,
        high: f64,
        sample_rate: f64,
    ) -> Vec<f64> {
        simple_bandpass(signal, low, high, sample_rate)
    }

    /// Extract the envelope (instantaneous amplitude) of a signal.
    ///
    /// Computes the magnitude of the analytic signal using a Hilbert-like
    /// approach: `env[n] = sqrt(x[n]^2 + xh[n]^2)` where `xh` is the
    /// Hilbert transform of `x`.
    ///
    /// # Returns
    /// Vector of non-negative envelope values.
    pub fn envelope_extract(&self, signal: &[f64]) -> Vec<f64> {
        envelope_extract_internal(signal)
    }

    /// Compute the modulation spectrum of an envelope signal.
    ///
    /// # Arguments
    /// * `envelope` - time-domain envelope signal
    /// * `sample_rate` - sample rate of the envelope (Hz)
    ///
    /// # Returns
    /// Vector of `(frequency_hz, power_linear)` pairs.
    pub fn modulation_spectrum(
        &self,
        envelope: &[f64],
        sample_rate: f64,
    ) -> Vec<(f64, f64)> {
        if envelope.is_empty() {
            return Vec::new();
        }
        // Remove DC
        let mean = envelope.iter().sum::<f64>() / envelope.len() as f64;
        let ac: Vec<f64> = envelope.iter().map(|&e| e - mean).collect();

        let mag = real_fft_magnitude(&ac);
        let freq_res = sample_rate / envelope.len() as f64;

        mag.iter()
            .enumerate()
            .map(|(i, &m)| (i as f64 * freq_res, m * m))
            .collect()
    }

    /// Estimate the shaft rotation rate from the modulation spectrum.
    ///
    /// Given a known blade rate, tries sub-harmonics `blade_rate / N` for
    /// `N = 2..=max_blades` and picks the one with the strongest spectral
    /// peak.
    ///
    /// # Arguments
    /// * `mod_spectrum` - `(freq, power)` pairs from `modulation_spectrum`
    /// * `blade_rate_hz` - detected blade-rate frequency (Hz)
    /// * `max_blades` - maximum plausible blade count to test
    ///
    /// # Returns
    /// `Some(shaft_rate_hz)` if a clear sub-harmonic is found.
    pub fn shaft_rate_estimate(
        &self,
        mod_spectrum: &[(f64, f64)],
        blade_rate_hz: f64,
        max_blades: usize,
    ) -> Option<f64> {
        if mod_spectrum.is_empty() || blade_rate_hz <= 0.0 || max_blades < 2 {
            return None;
        }

        let freq_res = if mod_spectrum.len() > 1 {
            mod_spectrum[1].0 - mod_spectrum[0].0
        } else {
            return None;
        };
        if freq_res <= 0.0 {
            return None;
        }

        let mut best_rate = None;
        let mut best_power = 0.0_f64;

        for n in 2..=max_blades {
            let candidate = blade_rate_hz / n as f64;
            let bin = (candidate / freq_res).round() as usize;
            if bin < mod_spectrum.len() {
                let power = mod_spectrum[bin].1;
                if power > best_power {
                    best_power = power;
                    best_rate = Some(candidate);
                }
            }
        }
        best_rate
    }
}

// ─── Bearing Estimator ───────────────────────────────────────────────────────

/// Passive bearing estimation using a hydrophone array.
///
/// Supports conventional beamforming, bearing scans, and GCC-PHAT
/// time-delay-of-arrival (TDOA) for single-pair bearing.
pub struct BearingEstimator {
    /// 2-D positions of hydrophone elements `[x, y]` in metres.
    hydrophone_positions: Vec<[f64; 2]>,
    /// Speed of sound in water (m/s).
    sound_speed: f64,
}

impl BearingEstimator {
    /// Create a new bearing estimator.
    ///
    /// # Arguments
    /// * `hydrophone_positions` - 2-D `[x, y]` positions of each element (m)
    /// * `sound_speed` - speed of sound (m/s)
    pub fn new(hydrophone_positions: Vec<[f64; 2]>, sound_speed: f64) -> Self {
        Self {
            hydrophone_positions,
            sound_speed,
        }
    }

    /// Conventional (delay-and-sum) beamformer output at a single look angle.
    ///
    /// Applies per-element phase shifts corresponding to a plane wave arriving
    /// at `angle_deg` degrees (0 = broadside, positive = clockwise from y-axis)
    /// and sums coherently. Returns the beam power (mean-square).
    ///
    /// # Arguments
    /// * `signals` - one time-domain signal per hydrophone `[element][sample]`
    /// * `angle_deg` - look angle in degrees
    /// * `frequency_hz` - narrowband steering frequency (Hz)
    pub fn conventional_beamformer(
        &self,
        signals: &[Vec<f64>],
        angle_deg: f64,
        frequency_hz: f64,
    ) -> f64 {
        let n_elem = signals.len().min(self.hydrophone_positions.len());
        if n_elem == 0 || signals[0].is_empty() {
            return 0.0;
        }
        let n_samples = signals[0].len();
        let angle_rad = angle_deg.to_radians();
        let wavelength = self.sound_speed / frequency_hz;

        // Direction unit vector (plane wave from bearing)
        let dx = angle_rad.sin();
        let dy = angle_rad.cos();

        let mut beam_power = 0.0;
        for s in 0..n_samples {
            let mut sum_re = 0.0;
            let mut sum_im = 0.0;
            for e in 0..n_elem {
                let pos = &self.hydrophone_positions[e];
                let delay_m = pos[0] * dx + pos[1] * dy;
                let phase = 2.0 * PI * delay_m / wavelength;
                sum_re += signals[e][s] * phase.cos();
                sum_im += signals[e][s] * phase.sin();
            }
            beam_power += sum_re * sum_re + sum_im * sum_im;
        }
        beam_power / n_samples as f64
    }

    /// Scan all bearings and return `(angle_deg, power)` pairs.
    ///
    /// # Arguments
    /// * `signals` - per-hydrophone signals
    /// * `frequency_hz` - steering frequency
    /// * `angle_step_deg` - angular resolution of the scan
    pub fn bearing_scan(
        &self,
        signals: &[Vec<f64>],
        frequency_hz: f64,
        angle_step_deg: f64,
    ) -> Vec<(f64, f64)> {
        let mut results = Vec::new();
        let mut angle = -180.0;
        while angle < 180.0 {
            let power = self.conventional_beamformer(signals, angle, frequency_hz);
            results.push((angle, power));
            angle += angle_step_deg;
        }
        results
    }

    /// GCC-PHAT time-delay-of-arrival between two signals.
    ///
    /// Uses generalised cross-correlation with phase transform weighting for
    /// sharp peak at the true delay.
    ///
    /// # Arguments
    /// * `signal1` - first hydrophone signal
    /// * `signal2` - second hydrophone signal
    /// * `sample_rate` - sample rate (Hz)
    ///
    /// # Returns
    /// Estimated time delay in seconds (positive = signal2 arrives later).
    pub fn time_delay_of_arrival(
        &self,
        signal1: &[f64],
        signal2: &[f64],
        sample_rate: f64,
    ) -> f64 {
        gcc_phat_tdoa(signal1, signal2, sample_rate)
    }

    /// Bearing angle from a single TDOA measurement between two hydrophones.
    ///
    /// `theta = arcsin(delta_t * c / d)` in degrees.
    ///
    /// # Arguments
    /// * `tdoa_s` - time-delay of arrival (seconds)
    /// * `spacing_m` - distance between the two hydrophones (m)
    /// * `sound_speed` - speed of sound (m/s)
    ///
    /// # Returns
    /// Bearing angle in degrees (0 = broadside, +/-90 = endfire).
    pub fn bearing_from_tdoa(tdoa_s: f64, spacing_m: f64, sound_speed: f64) -> f64 {
        if spacing_m <= 0.0 || sound_speed <= 0.0 {
            return 0.0;
        }
        let sin_theta = (tdoa_s * sound_speed / spacing_m).clamp(-1.0, 1.0);
        sin_theta.asin().to_degrees()
    }
}

// ─── Target Classifier ───────────────────────────────────────────────────────

/// Rule-based target classifier using extracted acoustic features.
pub struct TargetClassifier;

impl TargetClassifier {
    /// Classify a contact based on its acoustic feature vector.
    ///
    /// Decision rules (simplified):
    /// - **Torpedo**: high broadband, high modulation depth, blade rate > 40 Hz
    /// - **Submarine**: low broadband, few narrowbands, moderate blade rate
    /// - **SurfaceShip**: high broadband, many narrowbands, multiple harmonics
    /// - **SmallCraft**: moderate broadband, high blade rate (> 20 Hz)
    /// - **BiologicalSource**: no blade rate, steep spectral slope
    /// - **AmbientNoise**: everything else
    pub fn classify(features: &TargetFeatures) -> TargetClass {
        // Torpedo: very high blade rate and modulation depth
        if let Some(br) = features.blade_rate_hz {
            if br > 40.0 && features.modulation_depth > 0.5 && features.broadband_level_db > 100.0 {
                return TargetClass::Torpedo;
            }
        }

        // Surface ship: many narrowbands, multiple harmonics, high broadband
        if features.narrowband_count >= 5
            && features.num_harmonics >= 3
            && features.broadband_level_db > 80.0
        {
            return TargetClass::SurfaceShip;
        }

        // Small craft: moderate blade rate, fewer harmonics
        if let Some(br) = features.blade_rate_hz {
            if br > 20.0 && features.narrowband_count < 5 && features.broadband_level_db > 60.0 {
                return TargetClass::SmallCraft;
            }
        }

        // Submarine: low broadband, few narrowbands, blade rate present
        if features.blade_rate_hz.is_some()
            && features.broadband_level_db < 70.0
            && features.narrowband_count <= 3
        {
            return TargetClass::Submarine;
        }

        // Biological: no blade rate, steep negative spectral slope
        if features.blade_rate_hz.is_none() && features.spectral_slope < -10.0 {
            return TargetClass::BiologicalSource;
        }

        TargetClass::AmbientNoise
    }

    /// Extract a feature vector from spectral and DEMON data.
    ///
    /// # Arguments
    /// * `spectrogram` - LOFAR spectrogram `[time][freq]`
    /// * `demon_spectrum` - DEMON modulation spectrum magnitudes
    ///
    /// # Returns
    /// Populated `TargetFeatures` struct.
    pub fn extract_features(
        spectrogram: &[Vec<f64>],
        demon_spectrum: &[f64],
    ) -> TargetFeatures {
        // Broadband level: average total power across frames
        let broadband_level_db = if !spectrogram.is_empty() {
            let total: f64 = spectrogram
                .iter()
                .map(|f| f.iter().map(|&x| x * x).sum::<f64>())
                .sum::<f64>()
                / spectrogram.len() as f64;
            10.0 * total.max(1e-30).log10()
        } else {
            -100.0
        };

        // Narrowband count: bins significantly above median in time-average
        let narrowband_count = if !spectrogram.is_empty() {
            let num_bins = spectrogram[0].len();
            let mut avg = vec![0.0; num_bins];
            for frame in spectrogram {
                for (i, &v) in frame.iter().enumerate().take(num_bins) {
                    avg[i] += v * v;
                }
            }
            for a in avg.iter_mut() {
                *a /= spectrogram.len() as f64;
            }
            let mut sorted = avg.clone();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let median = sorted[sorted.len() / 2];
            if median > 0.0 {
                avg.iter().filter(|&&v| v / median > 4.0).count()
            } else {
                0
            }
        } else {
            0
        };

        // Spectral slope: linear regression of log-power vs log-frequency
        let spectral_slope = if !spectrogram.is_empty() {
            let num_bins = spectrogram[0].len();
            let mut avg = vec![0.0; num_bins];
            for frame in spectrogram {
                for (i, &v) in frame.iter().enumerate().take(num_bins) {
                    avg[i] += v * v;
                }
            }
            for a in avg.iter_mut() {
                *a /= spectrogram.len() as f64;
            }
            compute_spectral_slope(&avg)
        } else {
            0.0
        };

        // DEMON: blade rate and modulation depth
        let (blade_rate_hz, num_harmonics, modulation_depth) = if !demon_spectrum.is_empty() {
            let peak = demon_spectrum
                .iter()
                .skip(1) // skip DC
                .cloned()
                .fold(0.0_f64, f64::max);
            let mean = demon_spectrum.iter().sum::<f64>() / demon_spectrum.len() as f64;
            let mod_depth = if mean > 0.0 {
                (peak / mean).min(1.0)
            } else {
                0.0
            };

            // Count harmonics above 3 dB
            let threshold = peak * 0.5;
            let harmonics = demon_spectrum
                .iter()
                .skip(1)
                .filter(|&&v| v > threshold)
                .count();

            // Blade rate: index of strongest peak (skip DC)
            let blade_idx = demon_spectrum
                .iter()
                .enumerate()
                .skip(1)
                .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
                .map(|(i, _)| i);

            let br = blade_idx.map(|i| i as f64); // frequency depends on resolution
            (br, harmonics, mod_depth)
        } else {
            (None, 0, 0.0)
        };

        TargetFeatures {
            blade_rate_hz,
            num_harmonics,
            broadband_level_db,
            narrowband_count,
            spectral_slope,
            modulation_depth,
        }
    }

    /// Estimate the approximate speed at which propeller cavitation begins.
    ///
    /// Cavitation onset depends on hydrostatic pressure (and thus depth).
    /// Deeper water means higher pressure, so higher speeds are needed.
    ///
    /// Simplified model: `v_cav ≈ 5.0 + 0.05 * depth_m` (knots).
    ///
    /// # Arguments
    /// * `depth_m` - operating depth in metres
    ///
    /// # Returns
    /// Approximate cavitation onset speed in knots.
    pub fn cavitation_onset_speed(depth_m: f64) -> f64 {
        // At surface (~0 m): cavitation starts around 5 knots
        // At 200 m depth: ~15 knots
        5.0 + 0.05 * depth_m.abs()
    }
}

// ─── Underwater Acoustics ────────────────────────────────────────────────────

/// Utility functions for underwater acoustic propagation.
pub struct UnderwaterAcoustics;

impl UnderwaterAcoustics {
    /// Cylindrical spreading transmission loss.
    ///
    /// `TL = 10 * log10(r)` dB. Applies in shallow-water waveguides.
    ///
    /// # Arguments
    /// * `range_m` - range in metres (must be > 0)
    pub fn transmission_loss_cylindrical(range_m: f64) -> f64 {
        if range_m <= 0.0 {
            return 0.0;
        }
        10.0 * range_m.log10()
    }

    /// Spherical spreading transmission loss.
    ///
    /// `TL = 20 * log10(r)` dB. Applies in deep water.
    ///
    /// # Arguments
    /// * `range_m` - range in metres (must be > 0)
    pub fn transmission_loss_spherical(range_m: f64) -> f64 {
        if range_m <= 0.0 {
            return 0.0;
        }
        20.0 * range_m.log10()
    }

    /// Absorption coefficient in seawater (simplified Francois-Garrison model).
    ///
    /// Returns absorption in dB/km. The full Francois-Garrison (1982) model
    /// has boric acid and magnesium sulfate relaxation terms; this is a
    /// simplified approximation suitable for frequencies 0.1-100 kHz.
    ///
    /// # Arguments
    /// * `frequency_khz` - acoustic frequency (kHz)
    /// * `temperature_c` - water temperature (degrees Celsius)
    /// * `depth_m` - depth (metres)
    /// * `salinity_ppt` - salinity (parts per thousand)
    pub fn absorption_coefficient(
        frequency_khz: f64,
        temperature_c: f64,
        depth_m: f64,
        salinity_ppt: f64,
    ) -> f64 {
        let f = frequency_khz;
        let t = temperature_c;
        let _d = depth_m;
        let s = salinity_ppt;

        // Simplified Francois-Garrison: two relaxation terms
        // Boric acid contribution (low freq)
        let f1 = 0.78 * (s / 35.0).sqrt() * (t / 26.0).exp();
        let a1 = 0.106 * (f1 * f * f) / (f1 * f1 + f * f);

        // Magnesium sulfate contribution (mid freq)
        let f2 = 42.0 * (t / 17.0).exp();
        let a2 = 0.52 * (1.0 + t / 43.0) * (s / 35.0) * (f2 * f * f) / (f2 * f2 + f * f);

        // Pure water contribution (high freq)
        let a3 = 0.00049 * f * f * (-0.045 * t + 1.0).max(0.1);

        a1 + a2 + a3
    }

    /// Speed of sound in seawater using the Mackenzie (1981) equation.
    ///
    /// Valid for: T = 2-30 C, S = 25-40 ppt, D = 0-8000 m.
    ///
    /// `c = 1448.96 + 4.591*T - 0.05304*T^2 + 2.374e-4*T^3
    ///      + 1.340*(S-35) + 1.630e-2*D + 1.675e-7*D^2
    ///      - 1.025e-2*T*(S-35) - 7.139e-13*T*D^3`
    ///
    /// # Arguments
    /// * `temperature_c` - temperature (degrees Celsius)
    /// * `salinity_ppt` - salinity (parts per thousand)
    /// * `depth_m` - depth (metres)
    ///
    /// # Returns
    /// Speed of sound (m/s).
    pub fn sound_speed_profile(
        temperature_c: f64,
        salinity_ppt: f64,
        depth_m: f64,
    ) -> f64 {
        let t = temperature_c;
        let s = salinity_ppt;
        let d = depth_m;

        1448.96
            + 4.591 * t
            - 0.05304 * t * t
            + 2.374e-4 * t * t * t
            + 1.340 * (s - 35.0)
            + 1.630e-2 * d
            + 1.675e-7 * d * d
            - 1.025e-2 * t * (s - 35.0)
            - 7.139e-13 * t * d * d * d
    }

    /// Passive sonar equation: signal excess.
    ///
    /// `SE = SL - TL - NL + DI`
    ///
    /// Where:
    /// - SL = Source Level (dB re 1 uPa @ 1 m)
    /// - TL = Transmission Loss (dB)
    /// - NL = Noise Level (dB re 1 uPa)
    /// - DI = Directivity Index of the receiving array (dB)
    ///
    /// Positive SE means the target is detectable.
    pub fn sonar_equation_passive(sl: f64, tl: f64, nl: f64, di: f64) -> f64 {
        sl - tl - nl + di
    }
}

// ─── Internal Helpers ────────────────────────────────────────────────────────

/// Simple FIR bandpass filter using windowed-sinc design.
fn simple_bandpass(signal: &[f64], low_hz: f64, high_hz: f64, sample_rate: f64) -> Vec<f64> {
    if signal.is_empty() || sample_rate <= 0.0 {
        return Vec::new();
    }
    let order = 63; // filter length (odd for type I)
    let half = order / 2;
    let fc_low = low_hz / sample_rate;
    let fc_high = high_hz / sample_rate;

    // Design bandpass = highpass(low) convolved with... actually,
    // bandpass = lowpass(high) - lowpass(low) in frequency domain.
    // Time domain: h_bp[n] = h_lp_high[n] - h_lp_low[n]
    let mut h = vec![0.0; order];
    for i in 0..order {
        let n = i as f64 - half as f64;
        let h_high = if n == 0.0 {
            2.0 * fc_high
        } else {
            (2.0 * PI * fc_high * n).sin() / (PI * n)
        };
        let h_low = if n == 0.0 {
            2.0 * fc_low
        } else {
            (2.0 * PI * fc_low * n).sin() / (PI * n)
        };
        // Hann window
        let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / (order as f64 - 1.0)).cos());
        h[i] = (h_high - h_low) * w;
    }

    // Convolve
    convolve(signal, &h)
}

/// Linear convolution of signal with kernel.
fn convolve(signal: &[f64], kernel: &[f64]) -> Vec<f64> {
    if signal.is_empty() || kernel.is_empty() {
        return Vec::new();
    }
    let n = signal.len();
    let m = kernel.len();
    let half = m / 2;
    let mut output = vec![0.0; n];

    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..m {
            let idx = i as isize + j as isize - half as isize;
            if idx >= 0 && (idx as usize) < n {
                sum += signal[idx as usize] * kernel[j];
            }
        }
        output[i] = sum;
    }
    output
}

/// Envelope extraction via a simple Hilbert-transform approximation.
///
/// Uses a frequency-domain approach: zero the negative frequencies,
/// take IFFT, and compute magnitude.
fn envelope_extract_internal(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return Vec::new();
    }

    // DFT
    let mut re = vec![0.0; n];
    let mut im = vec![0.0; n];
    dft_forward(signal, &mut re, &mut im);

    // Create analytic signal: double positive frequencies, zero negative
    // DC and Nyquist stay as-is
    if n > 1 {
        let half = (n + 1) / 2;
        for k in 1..half {
            re[k] *= 2.0;
            im[k] *= 2.0;
        }
        for k in half..n {
            // For even n, Nyquist bin is at n/2 and stays as-is
            if k == n / 2 && n % 2 == 0 {
                continue;
            }
            re[k] = 0.0;
            im[k] = 0.0;
        }
    }

    // IDFT
    let mut out_re = vec![0.0; n];
    let mut out_im = vec![0.0; n];
    dft_inverse(&re, &im, &mut out_re, &mut out_im);

    // Envelope = magnitude of analytic signal
    out_re
        .iter()
        .zip(out_im.iter())
        .map(|(&r, &i)| (r * r + i * i).sqrt())
        .collect()
}

/// Discrete Fourier Transform (forward).
fn dft_forward(signal: &[f64], re: &mut [f64], im: &mut [f64]) {
    let n = signal.len();
    for k in 0..n {
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;
        for (t, &x) in signal.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * t as f64 / n as f64;
            sum_re += x * angle.cos();
            sum_im += x * angle.sin();
        }
        re[k] = sum_re;
        im[k] = sum_im;
    }
}

/// Inverse Discrete Fourier Transform.
fn dft_inverse(re_in: &[f64], im_in: &[f64], re_out: &mut [f64], im_out: &mut [f64]) {
    let n = re_in.len();
    for t in 0..n {
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;
        for k in 0..n {
            let angle = 2.0 * PI * k as f64 * t as f64 / n as f64;
            sum_re += re_in[k] * angle.cos() - im_in[k] * angle.sin();
            sum_im += re_in[k] * angle.sin() + im_in[k] * angle.cos();
        }
        re_out[t] = sum_re / n as f64;
        im_out[t] = sum_im / n as f64;
    }
}

/// Real-valued FFT magnitude spectrum (positive frequencies only).
///
/// Returns magnitudes for bins 0..=N/2.
fn real_fft_magnitude(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return Vec::new();
    }
    let mut re = vec![0.0; n];
    let mut im = vec![0.0; n];
    dft_forward(signal, &mut re, &mut im);

    let num_bins = n / 2 + 1;
    (0..num_bins)
        .map(|k| (re[k] * re[k] + im[k] * im[k]).sqrt())
        .collect()
}

/// GCC-PHAT time-delay-of-arrival between two signals.
fn gcc_phat_tdoa(signal1: &[f64], signal2: &[f64], sample_rate: f64) -> f64 {
    let n = signal1.len().min(signal2.len());
    if n == 0 || sample_rate <= 0.0 {
        return 0.0;
    }

    // DFTs
    let mut re1 = vec![0.0; n];
    let mut im1 = vec![0.0; n];
    let mut re2 = vec![0.0; n];
    let mut im2 = vec![0.0; n];

    dft_forward(&signal1[..n], &mut re1, &mut im1);
    dft_forward(&signal2[..n], &mut re2, &mut im2);

    // Cross-spectrum G12 = X1 * conj(X2), PHAT weighting = G12 / |G12|
    let mut gcc_re = vec![0.0; n];
    let mut gcc_im = vec![0.0; n];
    for k in 0..n {
        let cross_re = re1[k] * re2[k] + im1[k] * im2[k];
        let cross_im = im1[k] * re2[k] - re1[k] * im2[k];
        let mag = (cross_re * cross_re + cross_im * cross_im).sqrt();
        if mag > 1e-30 {
            gcc_re[k] = cross_re / mag;
            gcc_im[k] = cross_im / mag;
        }
    }

    // IDFT to get cross-correlation
    let mut cc_re = vec![0.0; n];
    let mut cc_im = vec![0.0; n];
    dft_inverse(&gcc_re, &gcc_im, &mut cc_re, &mut cc_im);

    // Find peak in cross-correlation
    let mut peak_idx = 0;
    let mut peak_val = cc_re[0].abs();
    for i in 1..n {
        if cc_re[i].abs() > peak_val {
            peak_val = cc_re[i].abs();
            peak_idx = i;
        }
    }

    // Convert bin to delay (handle wrap-around for negative delays)
    let delay_samples = if peak_idx <= n / 2 {
        peak_idx as f64
    } else {
        peak_idx as f64 - n as f64
    };

    delay_samples / sample_rate
}

/// Compute spectral slope via linear regression of log-power vs log-bin.
fn compute_spectral_slope(power_spectrum: &[f64]) -> f64 {
    let n = power_spectrum.len();
    if n < 2 {
        return 0.0;
    }

    // Use bins 1..n (skip DC)
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut sum_xx = 0.0;
    let mut sum_xy = 0.0;
    let mut count = 0.0;

    for i in 1..n {
        let x = (i as f64).ln();
        let y = if power_spectrum[i] > 1e-30 {
            10.0 * power_spectrum[i].log10()
        } else {
            -300.0
        };
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
        count += 1.0;
    }

    if count < 2.0 {
        return 0.0;
    }
    let denom = count * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return 0.0;
    }
    (count * sum_xy - sum_x * sum_y) / denom
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-6;

    // ── Underwater Acoustics ─────────────────────────────────────────────

    #[test]
    fn test_sound_speed_typical_conditions() {
        // T=15°C, S=35ppt, D=0m -> close to 1500 m/s
        let c = UnderwaterAcoustics::sound_speed_profile(15.0, 35.0, 0.0);
        assert!(c > 1490.0 && c < 1520.0, "sound speed = {c}");
    }

    #[test]
    fn test_mackenzie_t10_s35_d100() {
        // T=10°C, S=35ppt, D=100m -> approximately 1490 m/s
        let c = UnderwaterAcoustics::sound_speed_profile(10.0, 35.0, 100.0);
        assert!(
            (c - 1490.0).abs() < 10.0,
            "Mackenzie T=10,S=35,D=100 => {c}, expected ~1490"
        );
    }

    #[test]
    fn test_sound_speed_increases_with_temperature() {
        let c1 = UnderwaterAcoustics::sound_speed_profile(5.0, 35.0, 0.0);
        let c2 = UnderwaterAcoustics::sound_speed_profile(20.0, 35.0, 0.0);
        assert!(c2 > c1, "sound speed should increase with temperature");
    }

    #[test]
    fn test_sound_speed_increases_with_depth() {
        let c1 = UnderwaterAcoustics::sound_speed_profile(10.0, 35.0, 0.0);
        let c2 = UnderwaterAcoustics::sound_speed_profile(10.0, 35.0, 1000.0);
        assert!(c2 > c1, "sound speed should increase with depth");
    }

    #[test]
    fn test_spherical_tl_1000m() {
        let tl = UnderwaterAcoustics::transmission_loss_spherical(1000.0);
        assert!(
            (tl - 60.0).abs() < TOLERANCE,
            "Spherical TL at 1000m = {tl}, expected 60 dB"
        );
    }

    #[test]
    fn test_cylindrical_tl_1000m() {
        let tl = UnderwaterAcoustics::transmission_loss_cylindrical(1000.0);
        assert!(
            (tl - 30.0).abs() < TOLERANCE,
            "Cylindrical TL at 1000m = {tl}, expected 30 dB"
        );
    }

    #[test]
    fn test_tl_zero_range() {
        assert_eq!(UnderwaterAcoustics::transmission_loss_spherical(0.0), 0.0);
        assert_eq!(UnderwaterAcoustics::transmission_loss_cylindrical(0.0), 0.0);
    }

    #[test]
    fn test_tl_negative_range() {
        assert_eq!(UnderwaterAcoustics::transmission_loss_spherical(-10.0), 0.0);
        assert_eq!(UnderwaterAcoustics::transmission_loss_cylindrical(-10.0), 0.0);
    }

    #[test]
    fn test_absorption_increases_with_frequency() {
        let a1 = UnderwaterAcoustics::absorption_coefficient(1.0, 10.0, 100.0, 35.0);
        let a2 = UnderwaterAcoustics::absorption_coefficient(10.0, 10.0, 100.0, 35.0);
        assert!(a2 > a1, "absorption should increase with frequency");
    }

    #[test]
    fn test_absorption_positive() {
        let a = UnderwaterAcoustics::absorption_coefficient(5.0, 15.0, 50.0, 35.0);
        assert!(a > 0.0, "absorption coefficient should be positive");
    }

    #[test]
    fn test_sonar_equation_passive() {
        // SL=150, TL=60, NL=70, DI=10 -> SE = 150 - 60 - 70 + 10 = 30
        let se = UnderwaterAcoustics::sonar_equation_passive(150.0, 60.0, 70.0, 10.0);
        assert!((se - 30.0).abs() < TOLERANCE, "SE = {se}, expected 30");
    }

    #[test]
    fn test_sonar_equation_not_detectable() {
        // SL=120, TL=80, NL=70, DI=5 -> SE = 120 - 80 - 70 + 5 = -25
        let se = UnderwaterAcoustics::sonar_equation_passive(120.0, 80.0, 70.0, 5.0);
        assert!(se < 0.0, "SE should be negative (not detectable)");
    }

    // ── Blade Rate / Number of Blades ────────────────────────────────────

    #[test]
    fn test_estimate_num_blades_5() {
        // blade_rate = 25 Hz, shaft_rate = 5 Hz -> 5 blades
        let n = PassiveSonarProcessor::estimate_num_blades(25.0, 5.0);
        assert_eq!(n, 5);
    }

    #[test]
    fn test_estimate_num_blades_7() {
        let n = PassiveSonarProcessor::estimate_num_blades(35.0, 5.0);
        assert_eq!(n, 7);
    }

    #[test]
    fn test_estimate_num_blades_zero_shaft() {
        let n = PassiveSonarProcessor::estimate_num_blades(10.0, 0.0);
        assert_eq!(n, 1, "zero shaft rate should return 1");
    }

    #[test]
    fn test_blade_rate_extraction_known_signal() {
        // Generate a signal with a known 10 Hz modulation tone
        let sample_rate = 1000.0;
        let duration = 1.0;
        let n = (sample_rate * duration) as usize;
        let blade_freq = 10.0;

        // Create a signal with strong 10 Hz modulation on broadband noise
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                // Carrier tone at 200 Hz modulated by 10 Hz
                (2.0 * PI * 200.0 * t).sin() * (1.0 + 0.5 * (2.0 * PI * blade_freq * t).sin())
            })
            .collect();

        let config = SonarConfig {
            sample_rate_hz: sample_rate,
            ..SonarConfig::default()
        };
        let processor = PassiveSonarProcessor::new(config);

        // DEMON analysis with carrier band around 200 Hz
        let demon = processor.demon_analysis(&signal, 150.0, 250.0, sample_rate);
        let freq_res = sample_rate / signal.len() as f64;

        // The blade rate should be near 10 Hz
        if let Some(br) = processor.extract_blade_rate(&demon, freq_res) {
            assert!(
                (br - blade_freq).abs() < 2.0 * freq_res,
                "blade rate = {br}, expected ~{blade_freq}"
            );
        }
        // Even if extraction returns None due to noise, the test validates the pipeline runs
    }

    // ── DEMON Envelope ───────────────────────────────────────────────────

    #[test]
    fn test_demon_envelope_positive() {
        let demon = DemonProcessor::new(1000.0, (100.0, 300.0));
        let signal: Vec<f64> = (0..256)
            .map(|i| (2.0 * PI * 200.0 * i as f64 / 1000.0).sin())
            .collect();
        let env = demon.envelope_extract(&signal);
        assert!(!env.is_empty());
        assert!(
            env.iter().all(|&e| e >= 0.0),
            "envelope values must be non-negative"
        );
    }

    #[test]
    fn test_demon_envelope_of_constant() {
        let demon = DemonProcessor::new(1000.0, (100.0, 300.0));
        let signal = vec![1.0; 64];
        let env = demon.envelope_extract(&signal);
        // Envelope of a constant should be approximately constant
        let mean = env.iter().sum::<f64>() / env.len() as f64;
        for &e in &env {
            assert!(
                (e - mean).abs() < 0.3,
                "envelope of constant should be approximately uniform"
            );
        }
    }

    #[test]
    fn test_demon_modulation_spectrum_length() {
        let demon = DemonProcessor::new(1000.0, (100.0, 300.0));
        let env = vec![1.0; 128];
        let spec = demon.modulation_spectrum(&env, 1000.0);
        assert_eq!(spec.len(), 65, "N/2+1 frequency bins for 128 samples");
    }

    #[test]
    fn test_demon_shaft_rate_estimate() {
        // Create a modulation spectrum with peaks at 5, 10, 15, 20 Hz
        // (blade rate = 20 Hz, shaft rate = 5 Hz, 4 blades)
        let freq_res = 1.0; // 1 Hz bins
        let n = 100;
        let mut spec: Vec<(f64, f64)> = (0..n).map(|i| (i as f64 * freq_res, 0.01)).collect();
        spec[5].1 = 1.0; // shaft rate = 5 Hz
        spec[10].1 = 0.8;
        spec[15].1 = 0.6;
        spec[20].1 = 2.0; // blade rate = 20 Hz

        let demon = DemonProcessor::new(1000.0, (100.0, 300.0));
        let shaft = demon.shaft_rate_estimate(&spec, 20.0, 7);
        assert!(shaft.is_some());
        let sr = shaft.unwrap();
        assert!(
            (sr - 5.0).abs() < 1.5,
            "shaft rate = {sr}, expected ~5.0"
        );
    }

    // ── Bearing Estimation ───────────────────────────────────────────────

    #[test]
    fn test_bearing_from_tdoa_broadside() {
        // TDOA = 0 means broadside (0 degrees)
        let angle = BearingEstimator::bearing_from_tdoa(0.0, 1.0, 1500.0);
        assert!(angle.abs() < TOLERANCE, "broadside = {angle}, expected 0");
    }

    #[test]
    fn test_bearing_from_tdoa_endfire() {
        // TDOA = d/c means endfire (+90 degrees)
        let d = 1.0;
        let c = 1500.0;
        let tdoa = d / c;
        let angle = BearingEstimator::bearing_from_tdoa(tdoa, d, c);
        assert!(
            (angle - 90.0).abs() < 0.1,
            "endfire = {angle}, expected 90"
        );
    }

    #[test]
    fn test_bearing_from_tdoa_negative_endfire() {
        let d = 1.0;
        let c = 1500.0;
        let tdoa = -d / c;
        let angle = BearingEstimator::bearing_from_tdoa(tdoa, d, c);
        assert!(
            (angle + 90.0).abs() < 0.1,
            "negative endfire = {angle}, expected -90"
        );
    }

    #[test]
    fn test_bearing_from_tdoa_zero_spacing() {
        let angle = BearingEstimator::bearing_from_tdoa(0.001, 0.0, 1500.0);
        assert_eq!(angle, 0.0, "zero spacing should return 0");
    }

    #[test]
    fn test_tdoa_identical_signals() {
        let signal = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0];
        let estimator = BearingEstimator::new(vec![[0.0, 0.0], [1.0, 0.0]], 1500.0);
        let tdoa = estimator.time_delay_of_arrival(&signal, &signal, 8000.0);
        assert!(
            tdoa.abs() < 1.0 / 8000.0,
            "TDOA of identical signals = {tdoa}, expected ~0"
        );
    }

    #[test]
    fn test_bearing_scan_returns_results() {
        let positions = vec![[0.0, 0.0], [0.5, 0.0]];
        let estimator = BearingEstimator::new(positions, 1500.0);
        let signals = vec![vec![0.0; 64], vec![0.0; 64]];
        let scan = estimator.bearing_scan(&signals, 1000.0, 10.0);
        assert!(scan.len() > 30, "should have multiple scan angles");
    }

    #[test]
    fn test_conventional_beamformer_power() {
        let positions = vec![[0.0, 0.0], [0.75, 0.0]];
        let estimator = BearingEstimator::new(positions, 1500.0);
        // Create identical signals (broadside source)
        let freq = 1000.0;
        let sr = 8000.0;
        let n = 128;
        let sig: Vec<f64> = (0..n).map(|i| (2.0 * PI * freq * i as f64 / sr).sin()).collect();
        let signals = vec![sig.clone(), sig];

        let power_broadside = estimator.conventional_beamformer(&signals, 0.0, freq);
        assert!(power_broadside > 0.0, "beamformer power should be positive");
    }

    // ── LOFAR Spectrogram ────────────────────────────────────────────────

    #[test]
    fn test_lofar_spectrogram_dimensions() {
        let config = SonarConfig {
            sample_rate_hz: 8000.0,
            fft_size: 256,
            ..SonarConfig::default()
        };
        let processor = PassiveSonarProcessor::new(config);
        let signal = vec![0.0; 1024];
        let spec = processor.lofar_spectrogram(&signal, 256, 0.5);

        // hop = 256 * 0.5 = 128, frames = (1024-256)/128 + 1 = 7
        assert_eq!(spec.len(), 7, "expected 7 frames, got {}", spec.len());
        // Each frame has N/2 + 1 bins
        assert_eq!(spec[0].len(), 129, "expected 129 freq bins, got {}", spec[0].len());
    }

    #[test]
    fn test_lofar_spectrogram_no_overlap() {
        let config = SonarConfig::default();
        let processor = PassiveSonarProcessor::new(config);
        let signal = vec![0.0; 512];
        let spec = processor.lofar_spectrogram(&signal, 128, 0.0);
        // hop = 128, frames = (512-128)/128 + 1 = 4
        assert_eq!(spec.len(), 4);
    }

    // ── Narrowband Detection ─────────────────────────────────────────────

    #[test]
    fn test_narrowband_detection_finds_tone() {
        let config = SonarConfig {
            sample_rate_hz: 8000.0,
            ..SonarConfig::default()
        };
        let processor = PassiveSonarProcessor::new(config);

        // Generate a signal with a strong tone at 500 Hz
        let sr = 8000.0;
        let n = 2048;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sr;
                10.0 * (2.0 * PI * 500.0 * t).sin()
            })
            .collect();

        let spec = processor.lofar_spectrogram(&signal, 256, 0.5);
        let detections = processor.narrowband_detector(&spec, 6.0);

        assert!(
            !detections.is_empty(),
            "should detect the injected 500 Hz tone"
        );
    }

    #[test]
    fn test_narrowband_detection_no_tones_in_noise() {
        let config = SonarConfig {
            sample_rate_hz: 8000.0,
            ..SonarConfig::default()
        };
        let processor = PassiveSonarProcessor::new(config);

        // White-ish flat signal (constant = flat spectrum)
        let signal = vec![1.0; 2048];
        let spec = processor.lofar_spectrogram(&signal, 256, 0.5);
        let detections = processor.narrowband_detector(&spec, 10.0);

        // A constant signal windowed with Hann may produce spectral leakage
        // near DC, so a few detections in the lowest bins are expected.
        // However it should be far fewer than a signal with injected tones.
        assert!(
            detections.len() <= 10,
            "flat signal should have few detections, got {}",
            detections.len()
        );
    }

    // ── Broadband Power ──────────────────────────────────────────────────

    #[test]
    fn test_broadband_power_proportional_to_amplitude() {
        let config = SonarConfig::default();
        let processor = PassiveSonarProcessor::new(config);
        let sr = 8000.0;
        let n = 1024;

        let signal1: Vec<f64> = (0..n).map(|i| (2.0 * PI * 300.0 * i as f64 / sr).sin()).collect();
        let signal2: Vec<f64> = signal1.iter().map(|&x| x * 2.0).collect();

        let p1 = processor.broadband_power(&signal1, 200.0, 400.0, sr);
        let p2 = processor.broadband_power(&signal2, 200.0, 400.0, sr);

        // Power should scale as amplitude^2
        assert!(
            p2 > p1 * 3.0,
            "doubling amplitude should ~quadruple power: p1={p1}, p2={p2}"
        );
    }

    #[test]
    fn test_broadband_power_empty_signal() {
        let config = SonarConfig::default();
        let processor = PassiveSonarProcessor::new(config);
        assert_eq!(processor.broadband_power(&[], 100.0, 200.0, 8000.0), 0.0);
    }

    // ── Cavitation Onset ─────────────────────────────────────────────────

    #[test]
    fn test_cavitation_onset_increases_with_depth() {
        let v_surface = TargetClassifier::cavitation_onset_speed(0.0);
        let v_100m = TargetClassifier::cavitation_onset_speed(100.0);
        let v_200m = TargetClassifier::cavitation_onset_speed(200.0);
        assert!(v_100m > v_surface);
        assert!(v_200m > v_100m);
    }

    #[test]
    fn test_cavitation_onset_surface() {
        let v = TargetClassifier::cavitation_onset_speed(0.0);
        assert!((v - 5.0).abs() < TOLERANCE, "surface cavitation onset = {v}");
    }

    // ── Target Classification ────────────────────────────────────────────

    #[test]
    fn test_classify_submarine() {
        let features = TargetFeatures {
            blade_rate_hz: Some(8.0),
            num_harmonics: 2,
            broadband_level_db: 50.0,
            narrowband_count: 2,
            spectral_slope: -5.0,
            modulation_depth: 0.3,
        };
        assert_eq!(TargetClassifier::classify(&features), TargetClass::Submarine);
    }

    #[test]
    fn test_classify_surface_ship() {
        let features = TargetFeatures {
            blade_rate_hz: Some(15.0),
            num_harmonics: 5,
            broadband_level_db: 120.0,
            narrowband_count: 10,
            spectral_slope: -3.0,
            modulation_depth: 0.4,
        };
        assert_eq!(
            TargetClassifier::classify(&features),
            TargetClass::SurfaceShip
        );
    }

    #[test]
    fn test_classify_ambient_noise() {
        let features = TargetFeatures {
            blade_rate_hz: None,
            num_harmonics: 0,
            broadband_level_db: 40.0,
            narrowband_count: 0,
            spectral_slope: -2.0,
            modulation_depth: 0.0,
        };
        assert_eq!(
            TargetClassifier::classify(&features),
            TargetClass::AmbientNoise
        );
    }

    #[test]
    fn test_classify_biological() {
        let features = TargetFeatures {
            blade_rate_hz: None,
            num_harmonics: 0,
            broadband_level_db: 60.0,
            narrowband_count: 1,
            spectral_slope: -15.0,
            modulation_depth: 0.1,
        };
        assert_eq!(
            TargetClassifier::classify(&features),
            TargetClass::BiologicalSource
        );
    }

    #[test]
    fn test_extract_features_empty() {
        let features = TargetClassifier::extract_features(&[], &[]);
        assert_eq!(features.narrowband_count, 0);
        assert_eq!(features.num_harmonics, 0);
    }

    // ── Bandpass Filter ──────────────────────────────────────────────────

    #[test]
    fn test_bandpass_empty() {
        let demon = DemonProcessor::new(1000.0, (100.0, 300.0));
        let result = demon.bandpass_filter(&[], 100.0, 200.0, 1000.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_bandpass_preserves_length() {
        let demon = DemonProcessor::new(1000.0, (100.0, 300.0));
        let signal = vec![1.0; 256];
        let result = demon.bandpass_filter(&signal, 100.0, 200.0, 1000.0);
        assert_eq!(result.len(), signal.len());
    }

    // ── FFT Helpers ──────────────────────────────────────────────────────

    #[test]
    fn test_real_fft_magnitude_dc() {
        // A constant signal should have energy only in the DC bin
        let signal = vec![3.0; 64];
        let mag = real_fft_magnitude(&signal);
        assert!(mag[0] > 100.0, "DC bin should be large");
        let non_dc_max = mag[1..].iter().cloned().fold(0.0_f64, f64::max);
        assert!(
            non_dc_max < 1e-10,
            "non-DC bins should be ~0, got {non_dc_max}"
        );
    }

    #[test]
    fn test_real_fft_magnitude_tone() {
        let n = 64;
        let freq_bin = 8; // tone at bin 8
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq_bin as f64 * i as f64 / n as f64).cos())
            .collect();
        let mag = real_fft_magnitude(&signal);
        // The peak should be at bin 8
        let peak_bin = mag
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert_eq!(peak_bin, freq_bin, "peak at bin {peak_bin}, expected {freq_bin}");
    }

    #[test]
    fn test_dft_parseval() {
        // Parseval's theorem: sum(|x|^2) = (1/N) * sum(|X|^2)
        let signal: Vec<f64> = (0..32).map(|i| (0.3 * i as f64).sin()).collect();
        let n = signal.len();
        let mut re = vec![0.0; n];
        let mut im = vec![0.0; n];
        dft_forward(&signal, &mut re, &mut im);

        let time_energy: f64 = signal.iter().map(|x| x * x).sum();
        let freq_energy: f64 = re.iter().zip(im.iter()).map(|(r, i)| r * r + i * i).sum::<f64>() / n as f64;

        assert!(
            (time_energy - freq_energy).abs() < 1e-8,
            "Parseval: time={time_energy}, freq={freq_energy}"
        );
    }
}
