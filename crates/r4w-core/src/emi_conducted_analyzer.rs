//! Conducted EMI measurement and analysis per CISPR standards for EMC compliance testing.
//!
//! This module provides tools for analyzing conducted electromagnetic interference (EMI)
//! on power supply lines per CISPR 16 measurement methodology. It supports multiple
//! detector types (peak, quasi-peak, average, RMS), CISPR frequency bands, Class A/B
//! emission limit checking, LISN correction, and narrowband vs broadband classification.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::emi_conducted_analyzer::{
//!     EmiConductedAnalyzer, DetectorType, CisprClass, FrequencyBand,
//! };
//!
//! // Create an analyzer for Band B (150 kHz – 30 MHz), peak detector
//! let analyzer = EmiConductedAnalyzer::builder()
//!     .band(FrequencyBand::BandB)
//!     .detector(DetectorType::Peak)
//!     .build();
//!
//! // Generate some test samples (in practice these come from a spectrum analyzer)
//! let sample_rate = 60_000_000.0; // 60 MHz
//! let num_samples = 1024;
//! let samples: Vec<(f64, f64)> = (0..num_samples)
//!     .map(|i| {
//!         let t = i as f64 / sample_rate;
//!         let freq = 1_000_000.0; // 1 MHz tone
//!         let phase = 2.0 * std::f64::consts::PI * freq * t;
//!         (phase.cos() * 0.001, phase.sin() * 0.001)
//!     })
//!     .collect();
//!
//! // Measure emissions
//! let result = analyzer.measure(&samples, sample_rate);
//! assert!(!result.measurements.is_empty());
//!
//! // Check compliance against Class B limits
//! let report = analyzer.check_compliance(&result, CisprClass::ClassB);
//! println!("Pass: {}, worst margin: {:.1} dB", report.pass, report.worst_margin_db);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// CISPR detector types per CISPR 16-1-1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectorType {
    /// Peak detector – captures instantaneous maximum.
    Peak,
    /// Quasi-peak detector – weighted response per CISPR charge/discharge constants.
    QuasiPeak,
    /// Average detector – true RMS-calibrated average (CISPR 16-1-1 §4.5).
    Average,
    /// RMS detector – root-mean-square over dwell time.
    Rms,
}

/// CISPR frequency bands for conducted emissions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrequencyBand {
    /// Band A: 9 kHz – 150 kHz.
    BandA,
    /// Band B: 150 kHz – 30 MHz.
    BandB,
}

impl FrequencyBand {
    /// Start frequency in Hz.
    pub fn start_hz(&self) -> f64 {
        match self {
            FrequencyBand::BandA => 9_000.0,
            FrequencyBand::BandB => 150_000.0,
        }
    }

    /// Stop frequency in Hz.
    pub fn stop_hz(&self) -> f64 {
        match self {
            FrequencyBand::BandA => 150_000.0,
            FrequencyBand::BandB => 30_000_000.0,
        }
    }

    /// Resolution bandwidth (RBW) in Hz per CISPR 16.
    pub fn rbw_hz(&self) -> f64 {
        match self {
            FrequencyBand::BandA => 200.0,
            FrequencyBand::BandB => 9_000.0,
        }
    }

    /// Minimum dwell time in seconds per frequency point per CISPR 16-2-1.
    pub fn dwell_time_s(&self) -> f64 {
        match self {
            FrequencyBand::BandA => 0.1,  // 100 ms
            FrequencyBand::BandB => 0.01, // 10 ms
        }
    }
}

/// CISPR emission class.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CisprClass {
    /// Class A – industrial/commercial environment.
    ClassA,
    /// Class B – residential environment (stricter limits).
    ClassB,
}

/// Classification of an emission as narrowband or broadband.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EmissionType {
    /// Narrowband – spectral line within one RBW bin.
    Narrowband,
    /// Broadband – noise-like emission spanning multiple RBW bins.
    Broadband,
}

/// A single frequency-domain measurement point.
#[derive(Debug, Clone)]
pub struct MeasurementPoint {
    /// Center frequency in Hz.
    pub frequency_hz: f64,
    /// Measured amplitude in dBuV.
    pub amplitude_dbuv: f64,
    /// Detector used for this measurement.
    pub detector: DetectorType,
    /// Emission classification.
    pub emission_type: EmissionType,
}

/// Result of a conducted EMI measurement sweep.
#[derive(Debug, Clone)]
pub struct MeasurementResult {
    /// Individual measurement points across the band.
    pub measurements: Vec<MeasurementPoint>,
    /// Frequency band.
    pub band: FrequencyBand,
    /// Detector type applied.
    pub detector: DetectorType,
}

/// A single limit-check entry.
#[derive(Debug, Clone)]
pub struct LimitCheckEntry {
    /// Center frequency in Hz.
    pub frequency_hz: f64,
    /// Measured level in dBuV.
    pub measured_dbuv: f64,
    /// Applicable limit in dBuV.
    pub limit_dbuv: f64,
    /// Margin (limit - measured). Negative means exceedance.
    pub margin_db: f64,
    /// True if measured <= limit.
    pub pass: bool,
}

/// Overall compliance report.
#[derive(Debug, Clone)]
pub struct ComplianceReport {
    /// True if all points pass.
    pub pass: bool,
    /// Worst-case margin in dB (most negative = worst).
    pub worst_margin_db: f64,
    /// Frequency of worst-case margin in Hz.
    pub worst_frequency_hz: f64,
    /// Per-point results.
    pub entries: Vec<LimitCheckEntry>,
    /// CISPR class tested.
    pub cispr_class: CisprClass,
}

/// LISN (Line Impedance Stabilization Network) correction model.
///
/// Models the 50 ohm / 50 uH V-network per CISPR 16-1-2.
#[derive(Debug, Clone)]
pub struct LisnCorrection {
    /// Inductance in henries.
    inductance_h: f64,
    /// Resistance in ohms.
    resistance_ohm: f64,
}

impl Default for LisnCorrection {
    fn default() -> Self {
        Self {
            inductance_h: 50e-6,   // 50 uH
            resistance_ohm: 50.0,  // 50 ohm
        }
    }
}

impl LisnCorrection {
    /// Create a LISN model with the given inductance and resistance.
    pub fn new(inductance_h: f64, resistance_ohm: f64) -> Self {
        Self {
            inductance_h,
            resistance_ohm,
        }
    }

    /// Compute the LISN impedance magnitude at a given frequency.
    pub fn impedance_magnitude(&self, freq_hz: f64) -> f64 {
        let omega = 2.0 * PI * freq_hz;
        let xl = omega * self.inductance_h;
        // Parallel combination of R and jwL: |Z| = R * wL / sqrt(R^2 + (wL)^2)
        let r = self.resistance_ohm;
        (r * xl) / (r * r + xl * xl).sqrt()
    }

    /// Correction factor in dB to normalize measurement to 50 ohm reference.
    ///
    /// `correction_db = 20 * log10(50 / |Z_lisn(f)|)`
    pub fn correction_db(&self, freq_hz: f64) -> f64 {
        let z = self.impedance_magnitude(freq_hz);
        if z <= 0.0 {
            return 0.0;
        }
        20.0 * (50.0 / z).log10()
    }
}

// ---------------------------------------------------------------------------
// Builder
// ---------------------------------------------------------------------------

/// Builder for [`EmiConductedAnalyzer`].
#[derive(Debug, Clone)]
pub struct EmiConductedAnalyzerBuilder {
    band: FrequencyBand,
    detector: DetectorType,
    lisn: Option<LisnCorrection>,
    num_points: usize,
}

impl Default for EmiConductedAnalyzerBuilder {
    fn default() -> Self {
        Self {
            band: FrequencyBand::BandB,
            detector: DetectorType::Peak,
            lisn: None,
            num_points: 200,
        }
    }
}

impl EmiConductedAnalyzerBuilder {
    /// Set the CISPR frequency band.
    pub fn band(mut self, band: FrequencyBand) -> Self {
        self.band = band;
        self
    }

    /// Set the detector type.
    pub fn detector(mut self, detector: DetectorType) -> Self {
        self.detector = detector;
        self
    }

    /// Set a custom LISN correction model.
    pub fn lisn(mut self, lisn: LisnCorrection) -> Self {
        self.lisn = Some(lisn);
        self
    }

    /// Set the number of measurement frequency points.
    pub fn num_points(mut self, n: usize) -> Self {
        self.num_points = n;
        self
    }

    /// Build the analyzer.
    pub fn build(self) -> EmiConductedAnalyzer {
        EmiConductedAnalyzer {
            band: self.band,
            detector: self.detector,
            lisn: self.lisn.unwrap_or_default(),
            num_points: self.num_points,
        }
    }
}

// ---------------------------------------------------------------------------
// Main analyzer
// ---------------------------------------------------------------------------

/// Conducted EMI analyzer per CISPR 16 measurement methodology.
///
/// Processes IQ samples captured from a conducted emission test setup
/// (e.g., through a LISN) and produces frequency-domain measurements
/// suitable for comparison against CISPR emission limits.
#[derive(Debug, Clone)]
pub struct EmiConductedAnalyzer {
    band: FrequencyBand,
    detector: DetectorType,
    lisn: LisnCorrection,
    num_points: usize,
}

impl EmiConductedAnalyzer {
    /// Create a builder.
    pub fn builder() -> EmiConductedAnalyzerBuilder {
        EmiConductedAnalyzerBuilder::default()
    }

    /// Get the configured frequency band.
    pub fn band(&self) -> FrequencyBand {
        self.band
    }

    /// Get the configured detector type.
    pub fn detector(&self) -> DetectorType {
        self.detector
    }

    /// Get the LISN correction model.
    pub fn lisn(&self) -> &LisnCorrection {
        &self.lisn
    }

    /// Get the number of measurement points.
    pub fn num_points(&self) -> usize {
        self.num_points
    }

    /// Resolution bandwidth for the configured band.
    pub fn rbw_hz(&self) -> f64 {
        self.band.rbw_hz()
    }

    /// Dwell time per frequency point for the configured band.
    pub fn dwell_time_s(&self) -> f64 {
        self.band.dwell_time_s()
    }

    /// Perform a measurement sweep over the configured band.
    ///
    /// `samples` are complex IQ tuples `(re, im)` captured at `sample_rate` Hz.
    /// The analyzer computes the spectrum via DFT, applies the selected detector,
    /// LISN correction, and returns measurement points across the band.
    pub fn measure(
        &self,
        samples: &[(f64, f64)],
        sample_rate: f64,
    ) -> MeasurementResult {
        let n = samples.len();
        if n == 0 {
            return MeasurementResult {
                measurements: Vec::new(),
                band: self.band,
                detector: self.detector,
            };
        }

        // Compute DFT magnitudes (Hann-windowed).
        let spectrum = windowed_dft_magnitude(samples);
        let freq_resolution = sample_rate / n as f64;

        // Generate logarithmically-spaced measurement frequencies.
        let freqs = log_spaced_frequencies(
            self.band.start_hz(),
            self.band.stop_hz(),
            self.num_points,
        );

        let rbw = self.band.rbw_hz();

        let mut measurements = Vec::with_capacity(freqs.len());

        for &freq in &freqs {
            // Determine bin range covering +/- RBW/2 around the center frequency.
            let bin_lo = ((freq - rbw / 2.0) / freq_resolution).floor().max(0.0) as usize;
            let bin_hi =
                ((freq + rbw / 2.0) / freq_resolution).ceil().min((n - 1) as f64) as usize;

            if bin_lo >= spectrum.len() || bin_hi >= spectrum.len() || bin_lo > bin_hi {
                continue;
            }

            // Apply detector across bins in the RBW window.
            let raw_amplitude = self.apply_detector(&spectrum[bin_lo..=bin_hi]);

            // Convert to dBuV: V_peak -> dBuV = 20*log10(V / 1uV)
            // We assume the sample magnitudes are in volts.
            let amplitude_uv = raw_amplitude * 1e6; // V -> uV
            let amplitude_dbuv = if amplitude_uv > 0.0 {
                20.0 * amplitude_uv.log10()
            } else {
                -999.0
            };

            // Apply LISN correction.
            let corrected_dbuv = amplitude_dbuv + self.lisn.correction_db(freq);

            // Classify emission type.
            let emission_type = classify_emission(&spectrum, bin_lo, bin_hi);

            measurements.push(MeasurementPoint {
                frequency_hz: freq,
                amplitude_dbuv: corrected_dbuv,
                detector: self.detector,
                emission_type,
            });
        }

        MeasurementResult {
            measurements,
            band: self.band,
            detector: self.detector,
        }
    }

    /// Check measured results against CISPR emission limits.
    pub fn check_compliance(
        &self,
        result: &MeasurementResult,
        class: CisprClass,
    ) -> ComplianceReport {
        let mut entries = Vec::with_capacity(result.measurements.len());
        let mut worst_margin = f64::INFINITY;
        let mut worst_freq = 0.0;

        for m in &result.measurements {
            let limit = emission_limit_dbuv(m.frequency_hz, class, self.detector);
            let margin = limit - m.amplitude_dbuv;
            let pass = margin >= 0.0;

            if margin < worst_margin {
                worst_margin = margin;
                worst_freq = m.frequency_hz;
            }

            entries.push(LimitCheckEntry {
                frequency_hz: m.frequency_hz,
                measured_dbuv: m.amplitude_dbuv,
                limit_dbuv: limit,
                margin_db: margin,
                pass,
            });
        }

        if entries.is_empty() {
            worst_margin = 0.0;
        }

        let all_pass = entries.iter().all(|e| e.pass);

        ComplianceReport {
            pass: all_pass,
            worst_margin_db: worst_margin,
            worst_frequency_hz: worst_freq,
            entries,
            cispr_class: class,
        }
    }

    /// Compute margin-to-limit for each measurement point.
    pub fn margins(
        &self,
        result: &MeasurementResult,
        class: CisprClass,
    ) -> Vec<(f64, f64)> {
        result
            .measurements
            .iter()
            .map(|m| {
                let limit = emission_limit_dbuv(m.frequency_hz, class, self.detector);
                (m.frequency_hz, limit - m.amplitude_dbuv)
            })
            .collect()
    }

    /// Find the worst-case (minimum) margin across all measurement points.
    pub fn worst_margin(
        &self,
        result: &MeasurementResult,
        class: CisprClass,
    ) -> Option<(f64, f64)> {
        self.margins(result, class)
            .into_iter()
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
    }

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /// Apply the configured detector to a slice of magnitude values.
    fn apply_detector(&self, magnitudes: &[f64]) -> f64 {
        if magnitudes.is_empty() {
            return 0.0;
        }
        match self.detector {
            DetectorType::Peak => {
                magnitudes
                    .iter()
                    .cloned()
                    .fold(0.0_f64, f64::max)
            }
            DetectorType::QuasiPeak => {
                // Simplified quasi-peak: weighted combination between peak and average.
                // True QP requires charge/discharge time constants; we approximate
                // as 0.7 * peak + 0.3 * average (conservative estimate).
                let peak = magnitudes.iter().cloned().fold(0.0_f64, f64::max);
                let avg = magnitudes.iter().sum::<f64>() / magnitudes.len() as f64;
                0.7 * peak + 0.3 * avg
            }
            DetectorType::Average => {
                magnitudes.iter().sum::<f64>() / magnitudes.len() as f64
            }
            DetectorType::Rms => {
                let sum_sq: f64 = magnitudes.iter().map(|m| m * m).sum();
                (sum_sq / magnitudes.len() as f64).sqrt()
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// CISPR emission limit in dBuV for conducted emissions.
///
/// Limits per CISPR 11 / CISPR 32 for the given frequency, class, and detector.
pub fn emission_limit_dbuv(freq_hz: f64, class: CisprClass, detector: DetectorType) -> f64 {
    // Limits are piecewise by frequency range and class.
    // Source: CISPR 32 Table A.3 (Class A) and Table A.4 (Class B).
    match (class, detector) {
        // ----- Class A -----
        (CisprClass::ClassA, DetectorType::QuasiPeak) => {
            if freq_hz < 150_000.0 {
                // Band A: 79 dBuV
                79.0
            } else if freq_hz < 500_000.0 {
                // 150 kHz - 500 kHz: 66 dBuV
                66.0
            } else {
                // 500 kHz - 30 MHz: 56 dBuV
                56.0
            }
        }
        (CisprClass::ClassA, DetectorType::Average) => {
            if freq_hz < 150_000.0 {
                66.0
            } else if freq_hz < 500_000.0 {
                56.0
            } else {
                46.0
            }
        }
        (CisprClass::ClassA, DetectorType::Peak) => {
            // Peak limits are typically QP + ~13 dB for conducted.
            if freq_hz < 150_000.0 {
                92.0
            } else if freq_hz < 500_000.0 {
                79.0
            } else {
                73.0
            }
        }
        (CisprClass::ClassA, DetectorType::Rms) => {
            // RMS ~ Average limits.
            if freq_hz < 150_000.0 {
                66.0
            } else if freq_hz < 500_000.0 {
                56.0
            } else {
                46.0
            }
        }
        // ----- Class B -----
        (CisprClass::ClassB, DetectorType::QuasiPeak) => {
            if freq_hz < 150_000.0 {
                // Band A: 66 dBuV
                66.0
            } else if freq_hz < 500_000.0 {
                // 150 kHz - 500 kHz: 56 dBuV
                56.0
            } else {
                // 500 kHz - 5 MHz: 56 dBuV; 5 - 30 MHz: 60 dBuV.
                if freq_hz < 5_000_000.0 {
                    56.0
                } else {
                    60.0
                }
            }
        }
        (CisprClass::ClassB, DetectorType::Average) => {
            if freq_hz < 150_000.0 {
                56.0
            } else if freq_hz < 500_000.0 {
                46.0
            } else if freq_hz < 5_000_000.0 {
                46.0
            } else {
                50.0
            }
        }
        (CisprClass::ClassB, DetectorType::Peak) => {
            if freq_hz < 150_000.0 {
                79.0
            } else if freq_hz < 500_000.0 {
                69.0
            } else if freq_hz < 5_000_000.0 {
                69.0
            } else {
                73.0
            }
        }
        (CisprClass::ClassB, DetectorType::Rms) => {
            if freq_hz < 150_000.0 {
                56.0
            } else if freq_hz < 500_000.0 {
                46.0
            } else if freq_hz < 5_000_000.0 {
                46.0
            } else {
                50.0
            }
        }
    }
}

/// Generate logarithmically-spaced frequencies between `start` and `stop`.
fn log_spaced_frequencies(start: f64, stop: f64, n: usize) -> Vec<f64> {
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return vec![(start + stop) / 2.0];
    }
    let log_start = start.ln();
    let log_stop = stop.ln();
    let mut freqs: Vec<f64> = (0..n)
        .map(|i| (log_start + (log_stop - log_start) * i as f64 / (n - 1) as f64).exp())
        .collect();
    // Clamp endpoints to avoid floating-point drift from exp(ln(x)).
    freqs[0] = start;
    freqs[n - 1] = stop;
    freqs
}

/// Compute Hann-windowed DFT magnitude spectrum.
///
/// Returns magnitude (linear, in same units as input) for each positive-frequency bin.
fn windowed_dft_magnitude(samples: &[(f64, f64)]) -> Vec<f64> {
    let n = samples.len();
    if n == 0 {
        return Vec::new();
    }

    // Apply Hann window.
    let windowed: Vec<(f64, f64)> = samples
        .iter()
        .enumerate()
        .map(|(i, &(re, im))| {
            let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos());
            (re * w, im * w)
        })
        .collect();

    // DFT via direct computation (O(N^2); fine for typical EMI test sizes).
    let half = n / 2 + 1;
    let mut magnitudes = Vec::with_capacity(half);

    for k in 0..half {
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;
        for (i, &(re, im)) in windowed.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
            let (sin_a, cos_a) = angle.sin_cos();
            sum_re += re * cos_a - im * sin_a;
            sum_im += re * sin_a + im * cos_a;
        }
        let mag = (sum_re * sum_re + sum_im * sum_im).sqrt() / n as f64;
        magnitudes.push(mag);
    }

    magnitudes
}

/// Classify an emission as narrowband or broadband by examining spectral shape.
///
/// If the peak bin dominates the energy in the RBW window (>60% of total),
/// it is classified as narrowband; otherwise broadband.
fn classify_emission(spectrum: &[f64], bin_lo: usize, bin_hi: usize) -> EmissionType {
    if bin_lo >= spectrum.len() || bin_hi >= spectrum.len() || bin_lo > bin_hi {
        return EmissionType::Broadband;
    }

    let slice = &spectrum[bin_lo..=bin_hi];
    let total: f64 = slice.iter().map(|m| m * m).sum();
    if total <= 0.0 {
        return EmissionType::Broadband;
    }

    let peak_sq = slice.iter().cloned().fold(0.0_f64, f64::max).powi(2);

    if peak_sq / total > 0.6 {
        EmissionType::Narrowband
    } else {
        EmissionType::Broadband
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Helper: generate a single-tone complex signal.
    fn tone(freq_hz: f64, sample_rate: f64, num_samples: usize, amplitude: f64) -> Vec<(f64, f64)> {
        (0..num_samples)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * PI * freq_hz * t;
                (amplitude * phase.cos(), amplitude * phase.sin())
            })
            .collect()
    }

    // Helper: generate white noise.
    fn noise(num_samples: usize, amplitude: f64, seed: u64) -> Vec<(f64, f64)> {
        // Simple LCG pseudo-random for reproducibility.
        let mut state = seed;
        (0..num_samples)
            .map(|_| {
                state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                let r1 = ((state >> 33) as f64) / (u32::MAX as f64) - 0.5;
                state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                let r2 = ((state >> 33) as f64) / (u32::MAX as f64) - 0.5;
                (amplitude * r1, amplitude * r2)
            })
            .collect()
    }

    #[test]
    fn test_builder_defaults() {
        let analyzer = EmiConductedAnalyzer::builder().build();
        assert_eq!(analyzer.band(), FrequencyBand::BandB);
        assert_eq!(analyzer.detector(), DetectorType::Peak);
        assert_eq!(analyzer.num_points(), 200);
    }

    #[test]
    fn test_builder_custom() {
        let analyzer = EmiConductedAnalyzer::builder()
            .band(FrequencyBand::BandA)
            .detector(DetectorType::Average)
            .num_points(50)
            .build();
        assert_eq!(analyzer.band(), FrequencyBand::BandA);
        assert_eq!(analyzer.detector(), DetectorType::Average);
        assert_eq!(analyzer.num_points(), 50);
    }

    #[test]
    fn test_frequency_band_params() {
        assert_eq!(FrequencyBand::BandA.start_hz(), 9_000.0);
        assert_eq!(FrequencyBand::BandA.stop_hz(), 150_000.0);
        assert_eq!(FrequencyBand::BandA.rbw_hz(), 200.0);
        assert_eq!(FrequencyBand::BandA.dwell_time_s(), 0.1);

        assert_eq!(FrequencyBand::BandB.start_hz(), 150_000.0);
        assert_eq!(FrequencyBand::BandB.stop_hz(), 30_000_000.0);
        assert_eq!(FrequencyBand::BandB.rbw_hz(), 9_000.0);
        assert_eq!(FrequencyBand::BandB.dwell_time_s(), 0.01);
    }

    #[test]
    fn test_lisn_default() {
        let lisn = LisnCorrection::default();
        // At very high frequency, impedance should approach 50 ohm.
        let z_high = lisn.impedance_magnitude(30_000_000.0);
        assert!((z_high - 50.0).abs() < 1.0, "High-freq impedance should be ~50 ohm, got {z_high}");

        // At low frequency, impedance should be small (wL << R).
        let z_low = lisn.impedance_magnitude(1_000.0);
        assert!(z_low < 10.0, "Low-freq impedance should be small, got {z_low}");
    }

    #[test]
    fn test_lisn_correction_db() {
        let lisn = LisnCorrection::default();
        // At 30 MHz, correction should be near 0 dB (impedance ~ 50 ohm).
        let corr = lisn.correction_db(30_000_000.0);
        assert!(corr.abs() < 1.0, "High-freq correction should be ~0 dB, got {corr}");

        // At 10 kHz, correction should be positive (impedance < 50 ohm -> boost needed).
        let corr_low = lisn.correction_db(10_000.0);
        assert!(corr_low > 0.0, "Low-freq correction should be positive, got {corr_low}");
    }

    #[test]
    fn test_lisn_custom() {
        let lisn = LisnCorrection::new(100e-6, 50.0);
        let z = lisn.impedance_magnitude(1_000_000.0);
        assert!(z > 0.0);
    }

    #[test]
    fn test_emission_limits_class_a_qp() {
        let limit_low = emission_limit_dbuv(100_000.0, CisprClass::ClassA, DetectorType::QuasiPeak);
        assert_eq!(limit_low, 79.0);

        let limit_mid = emission_limit_dbuv(300_000.0, CisprClass::ClassA, DetectorType::QuasiPeak);
        assert_eq!(limit_mid, 66.0);

        let limit_high = emission_limit_dbuv(10_000_000.0, CisprClass::ClassA, DetectorType::QuasiPeak);
        assert_eq!(limit_high, 56.0);
    }

    #[test]
    fn test_emission_limits_class_b_qp() {
        let limit = emission_limit_dbuv(1_000_000.0, CisprClass::ClassB, DetectorType::QuasiPeak);
        assert_eq!(limit, 56.0);

        let limit_high = emission_limit_dbuv(10_000_000.0, CisprClass::ClassB, DetectorType::QuasiPeak);
        assert_eq!(limit_high, 60.0);
    }

    #[test]
    fn test_emission_limits_class_b_average() {
        let limit = emission_limit_dbuv(1_000_000.0, CisprClass::ClassB, DetectorType::Average);
        assert_eq!(limit, 46.0);

        let limit_high = emission_limit_dbuv(10_000_000.0, CisprClass::ClassB, DetectorType::Average);
        assert_eq!(limit_high, 50.0);
    }

    #[test]
    fn test_peak_detector() {
        let analyzer = EmiConductedAnalyzer::builder()
            .detector(DetectorType::Peak)
            .build();
        let mags = vec![1.0, 3.0, 2.0, 5.0, 4.0];
        let result = analyzer.apply_detector(&mags);
        assert_eq!(result, 5.0);
    }

    #[test]
    fn test_average_detector() {
        let analyzer = EmiConductedAnalyzer::builder()
            .detector(DetectorType::Average)
            .build();
        let mags = vec![2.0, 4.0, 6.0, 8.0];
        let result = analyzer.apply_detector(&mags);
        assert!((result - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_rms_detector() {
        let analyzer = EmiConductedAnalyzer::builder()
            .detector(DetectorType::Rms)
            .build();
        let mags = vec![3.0, 4.0];
        let result = analyzer.apply_detector(&mags);
        let expected = ((9.0 + 16.0) / 2.0_f64).sqrt();
        assert!((result - expected).abs() < 1e-10);
    }

    #[test]
    fn test_quasi_peak_detector() {
        let analyzer = EmiConductedAnalyzer::builder()
            .detector(DetectorType::QuasiPeak)
            .build();
        let mags = vec![1.0, 5.0, 3.0];
        let result = analyzer.apply_detector(&mags);
        let peak = 5.0;
        let avg = 3.0;
        let expected = 0.7 * peak + 0.3 * avg;
        assert!((result - expected).abs() < 1e-10);
    }

    #[test]
    fn test_measure_empty_samples() {
        let analyzer = EmiConductedAnalyzer::builder().build();
        let result = analyzer.measure(&[], 1_000_000.0);
        assert!(result.measurements.is_empty());
    }

    #[test]
    fn test_measure_returns_points() {
        let sample_rate = 60_000_000.0;
        let samples = tone(1_000_000.0, sample_rate, 2048, 0.01);
        let analyzer = EmiConductedAnalyzer::builder()
            .band(FrequencyBand::BandB)
            .detector(DetectorType::Peak)
            .num_points(50)
            .build();
        let result = analyzer.measure(&samples, sample_rate);
        assert!(!result.measurements.is_empty());
        assert!(result.measurements.len() <= 50);
        // All frequencies should be within Band B.
        for m in &result.measurements {
            assert!(m.frequency_hz >= FrequencyBand::BandB.start_hz());
            assert!(m.frequency_hz <= FrequencyBand::BandB.stop_hz());
        }
    }

    #[test]
    fn test_compliance_pass() {
        // Very low amplitude signal should pass Class A limits easily.
        let sample_rate = 60_000_000.0;
        let samples = tone(1_000_000.0, sample_rate, 2048, 1e-9); // ~1 nV
        let analyzer = EmiConductedAnalyzer::builder()
            .band(FrequencyBand::BandB)
            .detector(DetectorType::Peak)
            .num_points(20)
            .build();
        let result = analyzer.measure(&samples, sample_rate);
        let report = analyzer.check_compliance(&result, CisprClass::ClassA);
        assert!(report.pass, "Very weak signal should pass Class A: worst margin = {} dB", report.worst_margin_db);
        assert!(report.worst_margin_db > 0.0);
    }

    #[test]
    fn test_compliance_fail() {
        // Very high amplitude signal should fail Class B limits.
        let sample_rate = 60_000_000.0;
        let samples = tone(1_000_000.0, sample_rate, 2048, 100.0); // 100 V
        let analyzer = EmiConductedAnalyzer::builder()
            .band(FrequencyBand::BandB)
            .detector(DetectorType::Peak)
            .num_points(20)
            .build();
        let result = analyzer.measure(&samples, sample_rate);
        let report = analyzer.check_compliance(&result, CisprClass::ClassB);
        assert!(!report.pass, "Very strong signal should fail Class B");
        assert!(report.worst_margin_db < 0.0);
    }

    #[test]
    fn test_margins_computation() {
        let sample_rate = 60_000_000.0;
        let samples = tone(1_000_000.0, sample_rate, 2048, 0.001);
        let analyzer = EmiConductedAnalyzer::builder()
            .band(FrequencyBand::BandB)
            .num_points(10)
            .build();
        let result = analyzer.measure(&samples, sample_rate);
        let margins = analyzer.margins(&result, CisprClass::ClassA);
        assert_eq!(margins.len(), result.measurements.len());
        for (freq, _margin) in &margins {
            assert!(*freq > 0.0);
        }
    }

    #[test]
    fn test_worst_margin() {
        let sample_rate = 60_000_000.0;
        let samples = tone(1_000_000.0, sample_rate, 2048, 0.001);
        let analyzer = EmiConductedAnalyzer::builder()
            .band(FrequencyBand::BandB)
            .num_points(10)
            .build();
        let result = analyzer.measure(&samples, sample_rate);
        let worst = analyzer.worst_margin(&result, CisprClass::ClassB);
        assert!(worst.is_some());
        let (freq, margin) = worst.unwrap();
        assert!(freq > 0.0);
        // For a 1 mV signal the margin should be well above 0.
        assert!(margin > 0.0, "1 mV signal should pass Class B, margin = {margin}");
    }

    #[test]
    fn test_log_spaced_frequencies() {
        let freqs = log_spaced_frequencies(1_000.0, 1_000_000.0, 10);
        assert_eq!(freqs.len(), 10);
        assert!((freqs[0] - 1_000.0).abs() < 0.1);
        assert!((freqs[9] - 1_000_000.0).abs() < 1.0);
        // Should be monotonically increasing.
        for i in 1..freqs.len() {
            assert!(freqs[i] > freqs[i - 1]);
        }
    }

    #[test]
    fn test_narrowband_classification() {
        // A pure tone should produce a narrowband classification in the peak bin region.
        let sample_rate = 1_000_000.0;
        let freq = 100_000.0;
        let samples = tone(freq, sample_rate, 512, 1.0);
        let spectrum = windowed_dft_magnitude(&samples);

        // Find the peak bin.
        let peak_bin = spectrum
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        let lo = peak_bin.saturating_sub(2);
        let hi = (peak_bin + 2).min(spectrum.len() - 1);
        let et = classify_emission(&spectrum, lo, hi);
        assert_eq!(et, EmissionType::Narrowband, "Pure tone should be classified as narrowband");
    }

    #[test]
    fn test_broadband_classification() {
        // White noise should tend toward broadband classification.
        let samples = noise(512, 1.0, 42);
        let spectrum = windowed_dft_magnitude(&samples);
        let lo = 10;
        let hi = 50;
        let et = classify_emission(&spectrum, lo, hi);
        // Noise energy is spread out, so peak_sq / total should be low.
        assert_eq!(et, EmissionType::Broadband, "White noise should be classified as broadband");
    }

    #[test]
    fn test_band_a_measurement() {
        // Band A requires sample rate >= 300 kHz to cover up to 150 kHz.
        let sample_rate = 400_000.0;
        let samples = tone(50_000.0, sample_rate, 2048, 0.001);
        let analyzer = EmiConductedAnalyzer::builder()
            .band(FrequencyBand::BandA)
            .detector(DetectorType::Average)
            .num_points(20)
            .build();
        let result = analyzer.measure(&samples, sample_rate);
        assert!(!result.measurements.is_empty());
        assert_eq!(result.band, FrequencyBand::BandA);
        for m in &result.measurements {
            assert!(m.frequency_hz >= 9_000.0);
            assert!(m.frequency_hz <= 150_000.0);
        }
    }
}
