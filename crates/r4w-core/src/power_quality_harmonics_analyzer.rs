//! Power quality harmonics analyzer for mains voltage and current signals.
//!
//! Implements THD/TDD calculation, harmonic extraction, and power quality analysis
//! per IEC 61000-4-7. Uses DFT-based harmonic decomposition to extract magnitude
//! and phase of each harmonic component, compute distortion metrics, estimate
//! power factor, and detect interharmonics.
//!
//! # Example
//!
//! ```
//! use r4w_core::power_quality_harmonics_analyzer::{PowerQualityAnalyzer, HarmonicResult};
//!
//! // 50 Hz fundamental, 1 kHz sample rate, analyze up to 10th harmonic
//! let analyzer = PowerQualityAnalyzer::new(50.0, 1000.0, 10);
//!
//! // Generate a pure 50 Hz sine (one full cycle = 20 samples at 1 kHz)
//! let n = 1000;
//! let signal: Vec<f64> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / 1000.0;
//!         (2.0 * std::f64::consts::PI * 50.0 * t).sin()
//!     })
//!     .collect();
//!
//! let result = analyzer.analyze(&signal);
//! // A pure sine has ~0% THD
//! assert!(result.thd_percent < 1.0, "THD should be near zero for a pure sine");
//! // RMS of a unit sine is 1/sqrt(2) ~ 0.7071
//! assert!((result.rms - std::f64::consts::FRAC_1_SQRT_2).abs() < 0.01);
//! ```

use std::f64::consts::PI;

// -- Complex arithmetic helpers using (f64, f64) tuples --

/// Complex number represented as (real, imaginary).
type Complex = (f64, f64);

#[inline]
fn c_add(a: Complex, b: Complex) -> Complex {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_mul(a: Complex, b: Complex) -> Complex {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_mag(a: Complex) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

#[inline]
fn c_phase(a: Complex) -> f64 {
    a.1.atan2(a.0)
}

#[inline]
fn c_from_polar(mag: f64, phase: f64) -> Complex {
    (mag * phase.cos(), mag * phase.sin())
}

// -- DFT at a specific frequency --

/// Compute the DFT coefficient at a given frequency bin (not necessarily integer).
/// This is the Goertzel-style single-bin DFT:
///   X(f) = sum_{n=0}^{N-1} x[n] * exp(-j * 2 * pi * f * n / fs)
fn dft_at_frequency(signal: &[f64], freq_hz: f64, sample_rate: f64) -> Complex {
    let n = signal.len();
    let mut acc: Complex = (0.0, 0.0);
    for (i, &x) in signal.iter().enumerate() {
        let phase = -2.0 * PI * freq_hz * i as f64 / sample_rate;
        acc = c_add(acc, c_mul((x, 0.0), c_from_polar(1.0, phase)));
    }
    // Normalise by N
    (acc.0 / n as f64, acc.1 / n as f64)
}

// -- Public types --

/// Information about a single harmonic component.
#[derive(Debug, Clone)]
pub struct HarmonicComponent {
    /// Harmonic order (1 = fundamental, 2 = 2nd harmonic, etc.)
    pub order: usize,
    /// Frequency in Hz
    pub frequency_hz: f64,
    /// Peak magnitude (amplitude)
    pub magnitude: f64,
    /// Phase in radians
    pub phase_rad: f64,
    /// Individual harmonic distortion as a percentage of the fundamental
    pub ihd_percent: f64,
}

/// Result of an interharmonic analysis.
#[derive(Debug, Clone)]
pub struct Interharmonic {
    /// Frequency in Hz
    pub frequency_hz: f64,
    /// Peak magnitude
    pub magnitude: f64,
    /// Phase in radians
    pub phase_rad: f64,
}

/// Complete result of a power quality analysis.
#[derive(Debug, Clone)]
pub struct HarmonicResult {
    /// THD as a percentage (ratio of harmonic RMS to fundamental RMS x 100)
    pub thd_percent: f64,
    /// TDD as a percentage (ratio of harmonic RMS to rated/demand current x 100)
    pub tdd_percent: f64,
    /// RMS value of the entire signal
    pub rms: f64,
    /// RMS of the fundamental component only
    pub fundamental_rms: f64,
    /// Measured fundamental frequency (may deviate from nominal)
    pub measured_fundamental_hz: f64,
    /// Frequency deviation from nominal (Hz)
    pub frequency_deviation_hz: f64,
    /// Harmonic components (order 1..=max_harmonic)
    pub harmonics: Vec<HarmonicComponent>,
    /// Detected interharmonics
    pub interharmonics: Vec<Interharmonic>,
}

/// Result of power factor estimation.
#[derive(Debug, Clone)]
pub struct PowerFactorResult {
    /// Overall (true / total) power factor
    pub power_factor: f64,
    /// Displacement power factor (cosine of phase difference at fundamental)
    pub displacement_pf: f64,
    /// Distortion power factor (accounts for harmonics)
    pub distortion_pf: f64,
    /// Active power in watts (assuming unit impedance)
    pub active_power: f64,
    /// Apparent power in VA
    pub apparent_power: f64,
    /// Reactive power in VAr
    pub reactive_power: f64,
}

/// Simplified flicker severity result.
#[derive(Debug, Clone)]
pub struct FlickerResult {
    /// Short-term flicker severity index Pst (simplified)
    pub pst: f64,
    /// Peak-to-peak voltage fluctuation as a percentage
    pub delta_v_percent: f64,
}

/// Power quality analyzer with configurable parameters.
///
/// Analyses mains voltage or current waveforms for harmonic content,
/// THD, TDD, power factor, and flicker.
#[derive(Debug, Clone)]
pub struct PowerQualityAnalyzer {
    /// Nominal fundamental frequency (e.g. 50.0 or 60.0 Hz)
    pub fundamental_freq: f64,
    /// Sample rate in Hz
    pub sample_rate: f64,
    /// Maximum harmonic order to analyse
    pub max_harmonic: usize,
    /// Rated / demand current for TDD calculation (default: 1.0 normalised)
    pub rated_current: f64,
}

impl PowerQualityAnalyzer {
    /// Create a new analyzer.
    ///
    /// * `fundamental_freq` - Nominal mains frequency (e.g. 50.0 or 60.0 Hz).
    /// * `sample_rate` - Sampling rate of the digitised waveform in Hz.
    /// * `max_harmonic` - Highest harmonic order to extract (e.g. 40 for IEC 61000-4-7).
    pub fn new(fundamental_freq: f64, sample_rate: f64, max_harmonic: usize) -> Self {
        assert!(fundamental_freq > 0.0, "fundamental frequency must be positive");
        assert!(sample_rate > 0.0, "sample rate must be positive");
        assert!(max_harmonic >= 1, "max_harmonic must be at least 1");
        Self {
            fundamental_freq,
            sample_rate,
            max_harmonic,
            rated_current: 1.0,
        }
    }

    /// Set the rated/demand current used for TDD calculation.
    pub fn with_rated_current(mut self, rated: f64) -> Self {
        assert!(rated > 0.0, "rated current must be positive");
        self.rated_current = rated;
        self
    }

    // -- Frequency deviation measurement --

    /// Estimate actual fundamental frequency by sweeping DFT near nominal.
    fn estimate_fundamental_frequency(&self, signal: &[f64]) -> f64 {
        let f0 = self.fundamental_freq;
        // Search +/-2 Hz in 0.05 Hz steps
        let search_lo = (f0 - 2.0).max(0.1);
        let search_hi = f0 + 2.0;
        let step = 0.05;

        let mut best_freq = f0;
        let mut best_mag = 0.0_f64;
        let mut freq = search_lo;
        while freq <= search_hi {
            let mag = c_mag(dft_at_frequency(signal, freq, self.sample_rate));
            if mag > best_mag {
                best_mag = mag;
                best_freq = freq;
            }
            freq += step;
        }

        // Refine with a finer sweep +/-0.05 Hz in 0.005 Hz steps
        let fine_lo = (best_freq - 0.05).max(0.1);
        let fine_hi = best_freq + 0.05;
        let fine_step = 0.005;
        freq = fine_lo;
        while freq <= fine_hi {
            let mag = c_mag(dft_at_frequency(signal, freq, self.sample_rate));
            if mag > best_mag {
                best_mag = mag;
                best_freq = freq;
            }
            freq += fine_step;
        }

        best_freq
    }

    // -- Core harmonic analysis --

    /// Analyse a real-valued signal and return full harmonic results.
    pub fn analyze(&self, signal: &[f64]) -> HarmonicResult {
        assert!(!signal.is_empty(), "signal must not be empty");

        // 1. Measure actual fundamental frequency
        let measured_f0 = self.estimate_fundamental_frequency(signal);
        let freq_dev = measured_f0 - self.fundamental_freq;

        // 2. Extract harmonics via single-bin DFT at each harmonic frequency
        let mut harmonics = Vec::with_capacity(self.max_harmonic);
        for h in 1..=self.max_harmonic {
            let freq = measured_f0 * h as f64;
            // Skip harmonics above Nyquist
            if freq > self.sample_rate / 2.0 {
                break;
            }
            let coeff = dft_at_frequency(signal, freq, self.sample_rate);
            // DFT gives amplitude/2 for a real signal (positive frequency only),
            // so multiply by 2 to get peak amplitude.
            let magnitude = c_mag(coeff) * 2.0;
            let phase = c_phase(coeff);
            harmonics.push(HarmonicComponent {
                order: h,
                frequency_hz: freq,
                magnitude,
                phase_rad: phase,
                ihd_percent: 0.0, // filled below
            });
        }

        // 3. Compute individual harmonic distortion percentages
        let fundamental_mag = if !harmonics.is_empty() {
            harmonics[0].magnitude
        } else {
            1.0
        };
        for h in &mut harmonics {
            if h.order == 1 {
                h.ihd_percent = 100.0; // fundamental is 100% of itself
            } else if fundamental_mag > 1e-15 {
                h.ihd_percent = (h.magnitude / fundamental_mag) * 100.0;
            }
        }

        // 4. THD per IEC 61000-4-7: sqrt( sum(V_h^2) ) / V_1 x 100
        let sum_sq_harmonics: f64 = harmonics
            .iter()
            .filter(|h| h.order >= 2)
            .map(|h| h.magnitude * h.magnitude)
            .sum();
        let thd = if fundamental_mag > 1e-15 {
            (sum_sq_harmonics.sqrt() / fundamental_mag) * 100.0
        } else {
            0.0
        };

        // 5. TDD: sqrt( sum(I_h^2) ) / I_rated x 100
        let rated_peak = self.rated_current * std::f64::consts::SQRT_2;
        let tdd = if rated_peak > 1e-15 {
            (sum_sq_harmonics.sqrt() / rated_peak) * 100.0
        } else {
            0.0
        };

        // 6. RMS of the full signal
        let rms = (signal.iter().map(|&x| x * x).sum::<f64>() / signal.len() as f64).sqrt();

        // 7. RMS of fundamental only (peak / sqrt(2))
        let fundamental_rms = fundamental_mag / std::f64::consts::SQRT_2;

        // 8. Interharmonic detection
        let interharmonics = self.detect_interharmonics(signal, measured_f0, &harmonics);

        HarmonicResult {
            thd_percent: thd,
            tdd_percent: tdd,
            rms,
            fundamental_rms,
            measured_fundamental_hz: measured_f0,
            frequency_deviation_hz: freq_dev,
            harmonics,
            interharmonics,
        }
    }

    /// Extract harmonic magnitudes and phases as (magnitude, phase_rad) tuples.
    /// Index 0 = fundamental, index 1 = 2nd harmonic, etc.
    pub fn extract_harmonics(&self, signal: &[f64]) -> Vec<(f64, f64)> {
        let result = self.analyze(signal);
        result
            .harmonics
            .iter()
            .map(|h| (h.magnitude, h.phase_rad))
            .collect()
    }

    /// Compute THD percentage for a signal.
    pub fn compute_thd(&self, signal: &[f64]) -> f64 {
        self.analyze(signal).thd_percent
    }

    /// Compute TDD percentage for a signal.
    pub fn compute_tdd(&self, signal: &[f64]) -> f64 {
        self.analyze(signal).tdd_percent
    }

    /// Compute RMS value of a signal.
    pub fn compute_rms(&self, signal: &[f64]) -> f64 {
        assert!(!signal.is_empty());
        (signal.iter().map(|&x| x * x).sum::<f64>() / signal.len() as f64).sqrt()
    }

    /// Compute individual harmonic distortion percentages.
    /// Returns a Vec where index 0 = fundamental (100%), index 1 = 2nd harmonic %, etc.
    pub fn individual_harmonic_distortion(&self, signal: &[f64]) -> Vec<f64> {
        let result = self.analyze(signal);
        result.harmonics.iter().map(|h| h.ihd_percent).collect()
    }

    // -- Interharmonic detection --

    fn detect_interharmonics(
        &self,
        signal: &[f64],
        measured_f0: f64,
        harmonics: &[HarmonicComponent],
    ) -> Vec<Interharmonic> {
        let mut interharmonics = Vec::new();
        let nyquist = self.sample_rate / 2.0;

        let fundamental_mag = if !harmonics.is_empty() {
            harmonics[0].magnitude
        } else {
            return interharmonics;
        };

        // Threshold: interharmonic must be at least 0.1% of fundamental to report
        let threshold = fundamental_mag * 0.001;

        // Check at half-integer multiples: 1.5f0, 2.5f0, ...
        for h in 0..self.max_harmonic {
            let freq = measured_f0 * (h as f64 + 1.5);
            if freq > nyquist {
                break;
            }
            let coeff = dft_at_frequency(signal, freq, self.sample_rate);
            let magnitude = c_mag(coeff) * 2.0;
            if magnitude > threshold {
                interharmonics.push(Interharmonic {
                    frequency_hz: freq,
                    magnitude,
                    phase_rad: c_phase(coeff),
                });
            }
        }

        interharmonics
    }

    // -- Frequency deviation --

    /// Measure frequency deviation from nominal fundamental.
    pub fn measure_frequency_deviation(&self, signal: &[f64]) -> f64 {
        let measured = self.estimate_fundamental_frequency(signal);
        measured - self.fundamental_freq
    }

    // -- Power factor estimation --

    /// Estimate power factor from synchronised voltage and current waveforms.
    ///
    /// Both signals must have the same length and sample rate.
    pub fn estimate_power_factor(&self, voltage: &[f64], current: &[f64]) -> PowerFactorResult {
        assert_eq!(
            voltage.len(),
            current.len(),
            "voltage and current must be the same length"
        );
        assert!(!voltage.is_empty());

        let measured_f0 = self.estimate_fundamental_frequency(voltage);

        // Fundamental components
        let v1 = dft_at_frequency(voltage, measured_f0, self.sample_rate);
        let i1 = dft_at_frequency(current, measured_f0, self.sample_rate);
        let v1_mag = c_mag(v1) * 2.0;
        let i1_mag = c_mag(i1) * 2.0;
        let v1_phase = c_phase(v1);
        let i1_phase = c_phase(i1);

        // Displacement power factor
        let phi1 = v1_phase - i1_phase;
        let displacement_pf = phi1.cos();

        // RMS values
        let v_rms =
            (voltage.iter().map(|&x| x * x).sum::<f64>() / voltage.len() as f64).sqrt();
        let i_rms =
            (current.iter().map(|&x| x * x).sum::<f64>() / current.len() as f64).sqrt();

        // Fundamental RMS
        let v1_rms = v1_mag / std::f64::consts::SQRT_2;
        let i1_rms = i1_mag / std::f64::consts::SQRT_2;

        // Distortion power factor = (V1_rms * I1_rms) / (V_rms * I_rms)
        let apparent = v_rms * i_rms;
        let distortion_pf = if apparent > 1e-15 {
            (v1_rms * i1_rms) / apparent
        } else {
            1.0
        };

        // Active power = sum(x[n]*y[n]) / N
        let active_power: f64 = voltage
            .iter()
            .zip(current.iter())
            .map(|(&v, &i)| v * i)
            .sum::<f64>()
            / voltage.len() as f64;

        let apparent_power = v_rms * i_rms;
        let pf = if apparent_power > 1e-15 {
            active_power / apparent_power
        } else {
            1.0
        };

        let reactive_power =
            if apparent_power * apparent_power > active_power * active_power {
                (apparent_power * apparent_power - active_power * active_power).sqrt()
            } else {
                0.0
            };

        PowerFactorResult {
            power_factor: pf,
            displacement_pf: displacement_pf.abs(),
            distortion_pf: distortion_pf.min(1.0),
            active_power,
            apparent_power,
            reactive_power,
        }
    }

    // -- Flicker severity --

    /// Simplified short-term flicker severity (Pst) estimation.
    ///
    /// Uses the envelope of the signal to detect amplitude modulations
    /// characteristic of voltage flicker. This is a simplified model;
    /// a full IEC 61000-4-15 implementation requires the flickermeter's
    /// weighting filters and statistical classifier.
    pub fn estimate_flicker(&self, signal: &[f64]) -> FlickerResult {
        assert!(!signal.is_empty());

        // Compute envelope via sliding RMS over one fundamental cycle
        let cycle_samples = (self.sample_rate / self.fundamental_freq).round() as usize;
        let cycle_samples = cycle_samples.max(1);

        let mut envelope = Vec::with_capacity(signal.len().saturating_sub(cycle_samples));
        for i in 0..signal.len().saturating_sub(cycle_samples) {
            let sum_sq: f64 = signal[i..i + cycle_samples]
                .iter()
                .map(|&x| x * x)
                .sum();
            envelope.push((sum_sq / cycle_samples as f64).sqrt());
        }

        if envelope.is_empty() {
            return FlickerResult {
                pst: 0.0,
                delta_v_percent: 0.0,
            };
        }

        // Mean envelope
        let mean_env = envelope.iter().sum::<f64>() / envelope.len() as f64;

        // Peak-to-peak fluctuation
        let max_env = envelope.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min_env = envelope.iter().cloned().fold(f64::INFINITY, f64::min);
        let delta_v = max_env - min_env;

        let delta_v_percent = if mean_env > 1e-15 {
            (delta_v / mean_env) * 100.0
        } else {
            0.0
        };

        // Simplified Pst: proportional to relative fluctuation
        let pst = delta_v_percent;

        FlickerResult {
            pst,
            delta_v_percent,
        }
    }
}

// -- Tests --

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_1_SQRT_2;

    /// Helper: generate a sine wave at a given frequency, amplitude, phase.
    fn sine_wave(freq: f64, amplitude: f64, phase: f64, sample_rate: f64, n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                amplitude * (2.0 * PI * freq * t + phase).sin()
            })
            .collect()
    }

    /// Helper: sum multiple sine waves together.
    fn sum_signals(signals: &[Vec<f64>]) -> Vec<f64> {
        let n = signals[0].len();
        let mut out = vec![0.0; n];
        for sig in signals {
            for (o, &s) in out.iter_mut().zip(sig.iter()) {
                *o += s;
            }
        }
        out
    }

    // 1. Pure sine: THD near 0%
    #[test]
    fn test_pure_sine_thd_near_zero() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10);
        let signal = sine_wave(50.0, 1.0, 0.0, 10000.0, 10000);
        let result = analyzer.analyze(&signal);
        assert!(result.thd_percent < 0.5, "THD = {}", result.thd_percent);
    }

    // 2. RMS of pure sine = amplitude / sqrt(2)
    #[test]
    fn test_rms_pure_sine() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10);
        let signal = sine_wave(50.0, 1.0, 0.0, 10000.0, 10000);
        let result = analyzer.analyze(&signal);
        assert!(
            (result.rms - FRAC_1_SQRT_2).abs() < 0.01,
            "RMS = {}",
            result.rms
        );
    }

    // 3. Known THD with 3rd harmonic at 10% amplitude
    #[test]
    fn test_known_thd_third_harmonic() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10);
        let fundamental = sine_wave(50.0, 1.0, 0.0, 10000.0, 10000);
        let third = sine_wave(150.0, 0.10, 0.0, 10000.0, 10000);
        let signal = sum_signals(&[fundamental, third]);
        let result = analyzer.analyze(&signal);
        assert!(
            (result.thd_percent - 10.0).abs() < 1.0,
            "THD = {}",
            result.thd_percent
        );
    }

    // 4. Multiple harmonics THD
    #[test]
    fn test_multiple_harmonics_thd() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10);
        let h1 = sine_wave(50.0, 1.0, 0.0, 10000.0, 10000);
        let h3 = sine_wave(150.0, 0.05, 0.0, 10000.0, 10000);
        let h5 = sine_wave(250.0, 0.03, 0.0, 10000.0, 10000);
        let signal = sum_signals(&[h1, h3, h5]);
        let result = analyzer.analyze(&signal);
        let expected = (0.05_f64.powi(2) + 0.03_f64.powi(2)).sqrt() * 100.0;
        assert!(
            (result.thd_percent - expected).abs() < 1.0,
            "THD = {}, expected ~{}",
            result.thd_percent,
            expected
        );
    }

    // 5. Individual harmonic distortion
    #[test]
    fn test_individual_harmonic_distortion() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10);
        let h1 = sine_wave(50.0, 1.0, 0.0, 10000.0, 10000);
        let h3 = sine_wave(150.0, 0.10, 0.0, 10000.0, 10000);
        let signal = sum_signals(&[h1, h3]);
        let ihd = analyzer.individual_harmonic_distortion(&signal);
        assert!((ihd[0] - 100.0).abs() < 0.1, "Fundamental should be 100%");
        assert!((ihd[2] - 10.0).abs() < 1.0, "3rd harmonic IHD = {}", ihd[2]);
    }

    // 6. Frequency deviation detection
    #[test]
    fn test_frequency_deviation() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10);
        let signal = sine_wave(50.5, 1.0, 0.0, 10000.0, 10000);
        let result = analyzer.analyze(&signal);
        assert!(
            (result.frequency_deviation_hz - 0.5).abs() < 0.1,
            "deviation = {}",
            result.frequency_deviation_hz
        );
    }

    // 7. 60 Hz system
    #[test]
    fn test_60hz_system() {
        let analyzer = PowerQualityAnalyzer::new(60.0, 10000.0, 10);
        let signal = sine_wave(60.0, 1.0, 0.0, 10000.0, 10000);
        let result = analyzer.analyze(&signal);
        assert!(result.thd_percent < 0.5);
        assert!((result.measured_fundamental_hz - 60.0).abs() < 0.1);
    }

    // 8. TDD calculation
    #[test]
    fn test_tdd_calculation() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10).with_rated_current(2.0);
        let h1 = sine_wave(50.0, 1.0, 0.0, 10000.0, 10000);
        let h3 = sine_wave(150.0, 0.20, 0.0, 10000.0, 10000);
        let signal = sum_signals(&[h1, h3]);
        let result = analyzer.analyze(&signal);
        let expected_tdd = (0.20 / (2.0 * std::f64::consts::SQRT_2)) * 100.0;
        assert!(
            (result.tdd_percent - expected_tdd).abs() < 1.0,
            "TDD = {}, expected ~{}",
            result.tdd_percent,
            expected_tdd
        );
    }

    // 9. Power factor: resistive load (PF near 1.0)
    #[test]
    fn test_power_factor_resistive() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10);
        let n = 10000;
        let voltage = sine_wave(50.0, 1.0, 0.0, 10000.0, n);
        let current = sine_wave(50.0, 0.5, 0.0, 10000.0, n);
        let pf = analyzer.estimate_power_factor(&voltage, &current);
        assert!(
            (pf.power_factor - 1.0).abs() < 0.02,
            "PF = {}",
            pf.power_factor
        );
        assert!(
            (pf.displacement_pf - 1.0).abs() < 0.02,
            "DPF = {}",
            pf.displacement_pf
        );
    }

    // 10. Power factor: inductive load (PF near cos(45 deg) = 0.707)
    #[test]
    fn test_power_factor_inductive() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10);
        let n = 10000;
        let voltage = sine_wave(50.0, 1.0, 0.0, 10000.0, n);
        let current = sine_wave(50.0, 0.5, -PI / 4.0, 10000.0, n);
        let pf = analyzer.estimate_power_factor(&voltage, &current);
        assert!(
            (pf.displacement_pf - FRAC_1_SQRT_2).abs() < 0.05,
            "DPF = {}",
            pf.displacement_pf
        );
    }

    // 11. Harmonic extraction returns correct number of harmonics
    #[test]
    fn test_extract_harmonics_count() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 5);
        let signal = sine_wave(50.0, 1.0, 0.0, 10000.0, 10000);
        let harmonics = analyzer.extract_harmonics(&signal);
        assert_eq!(
            harmonics.len(),
            5,
            "Expected 5 harmonics, got {}",
            harmonics.len()
        );
    }

    // 12. Fundamental magnitude extraction
    #[test]
    fn test_fundamental_magnitude() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10);
        let amplitude = 3.5;
        let signal = sine_wave(50.0, amplitude, 0.0, 10000.0, 10000);
        let result = analyzer.analyze(&signal);
        assert!(
            (result.harmonics[0].magnitude - amplitude).abs() < 0.05,
            "magnitude = {}",
            result.harmonics[0].magnitude
        );
    }

    // 13. Fundamental RMS
    #[test]
    fn test_fundamental_rms() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10);
        let signal = sine_wave(50.0, 1.0, 0.0, 10000.0, 10000);
        let result = analyzer.analyze(&signal);
        assert!(
            (result.fundamental_rms - FRAC_1_SQRT_2).abs() < 0.01,
            "fundamental_rms = {}",
            result.fundamental_rms
        );
    }

    // 14. Interharmonic detection (inject a 125 Hz component = 2.5 * f0)
    #[test]
    fn test_interharmonic_detection() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10);
        let h1 = sine_wave(50.0, 1.0, 0.0, 10000.0, 10000);
        let ih = sine_wave(125.0, 0.05, 0.0, 10000.0, 10000);
        let signal = sum_signals(&[h1, ih]);
        let result = analyzer.analyze(&signal);
        let found = result
            .interharmonics
            .iter()
            .any(|ih| (ih.frequency_hz - 125.0).abs() < 1.0);
        assert!(
            found,
            "Should detect interharmonic at 125 Hz, found: {:?}",
            result.interharmonics
        );
    }

    // 15. Flicker: steady signal has near-zero flicker
    #[test]
    fn test_flicker_steady_signal() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10);
        let signal = sine_wave(50.0, 1.0, 0.0, 10000.0, 10000);
        let flicker = analyzer.estimate_flicker(&signal);
        assert!(flicker.pst < 1.0, "Pst = {}", flicker.pst);
        assert!(
            flicker.delta_v_percent < 1.0,
            "delta_v = {}",
            flicker.delta_v_percent
        );
    }

    // 16. Flicker: amplitude-modulated signal has detectable flicker
    #[test]
    fn test_flicker_amplitude_modulated() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10);
        let n = 10000;
        let fs = 10000.0;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let modulation = 1.0 + 0.10 * (2.0 * PI * 10.0 * t).sin();
                modulation * (2.0 * PI * 50.0 * t).sin()
            })
            .collect();
        let flicker = analyzer.estimate_flicker(&signal);
        assert!(
            flicker.delta_v_percent > 1.0,
            "Should detect voltage fluctuation, got {}%",
            flicker.delta_v_percent
        );
    }

    // 17. compute_thd convenience method
    #[test]
    fn test_compute_thd_method() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10);
        let h1 = sine_wave(50.0, 1.0, 0.0, 10000.0, 10000);
        let h5 = sine_wave(250.0, 0.07, 0.0, 10000.0, 10000);
        let signal = sum_signals(&[h1, h5]);
        let thd = analyzer.compute_thd(&signal);
        assert!((thd - 7.0).abs() < 1.0, "THD = {}", thd);
    }

    // 18. compute_rms convenience method
    #[test]
    fn test_compute_rms_method() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10);
        let signal = sine_wave(50.0, 2.0, 0.0, 10000.0, 10000);
        let rms = analyzer.compute_rms(&signal);
        let expected = 2.0 / std::f64::consts::SQRT_2;
        assert!((rms - expected).abs() < 0.02, "RMS = {}", rms);
    }

    // 19. Active and apparent power
    #[test]
    fn test_power_calculations() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10);
        let n = 10000;
        let voltage = sine_wave(50.0, 1.0, 0.0, 10000.0, n);
        let current = sine_wave(50.0, 1.0, 0.0, 10000.0, n);
        let pf = analyzer.estimate_power_factor(&voltage, &current);
        assert!(
            (pf.apparent_power - 0.5).abs() < 0.02,
            "apparent = {}",
            pf.apparent_power
        );
        assert!(
            (pf.active_power - pf.apparent_power).abs() < 0.02,
            "active = {}, apparent = {}",
            pf.active_power,
            pf.apparent_power
        );
    }

    // 20. Harmonic phase extraction
    #[test]
    fn test_harmonic_phase() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10);
        let h1 = sine_wave(50.0, 1.0, 0.0, 10000.0, 10000);
        let h3 = sine_wave(150.0, 0.10, PI / 3.0, 10000.0, 10000);
        let signal = sum_signals(&[h1, h3]);
        let result = analyzer.analyze(&signal);
        // sin(wt + phi) = cos(wt + phi - pi/2), DFT phase = phi - pi/2
        let h3_phase = result.harmonics[2].phase_rad;
        let expected_phase = PI / 3.0 - PI / 2.0; // = -pi/6
        let phase_err = (h3_phase - expected_phase).abs();
        let phase_err = phase_err.min((phase_err - 2.0 * PI).abs());
        assert!(
            phase_err < 0.2,
            "phase = {}, expected ~ {}",
            h3_phase,
            expected_phase
        );
    }

    // 21. Reactive power for inductive load
    #[test]
    fn test_reactive_power() {
        let analyzer = PowerQualityAnalyzer::new(50.0, 10000.0, 10);
        let n = 10000;
        let voltage = sine_wave(50.0, 1.0, 0.0, 10000.0, n);
        let current = sine_wave(50.0, 1.0, -PI / 2.0, 10000.0, n);
        let pf = analyzer.estimate_power_factor(&voltage, &current);
        assert!(
            pf.active_power.abs() < 0.02,
            "active power should be ~0, got {}",
            pf.active_power
        );
        assert!(
            (pf.reactive_power - 0.5).abs() < 0.02,
            "reactive = {}",
            pf.reactive_power
        );
    }
}
