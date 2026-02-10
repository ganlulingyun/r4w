//! # Phasor Measurement Unit (PMU) — IEEE C37.118 Synchrophasor
//!
//! This module implements synchrophasor measurement per IEEE C37.118 for power
//! grid monitoring. It provides DFT-based phasor extraction, frequency and
//! rate-of-change-of-frequency (ROCOF) estimation, Total Vector Error (TVE)
//! computation, symmetrical-component (sequence) analysis, fault detection,
//! and C37.118 data-frame formatting.
//!
//! All functionality uses only the Rust standard library.
//!
//! # Example
//!
//! ```
//! use r4w_core::phasor_measurement_unit::{PmuConfig, PhasorMeasurementUnit};
//!
//! // 60 Hz system, 4800 Sa/s, 60 reports per second
//! let cfg = PmuConfig {
//!     nominal_freq: 60.0,
//!     sample_rate: 4800.0,
//!     reporting_rate: 60,
//! };
//! let pmu = PhasorMeasurementUnit::new(cfg);
//!
//! // Synthesise one reporting window of a 1.0 pu, 60 Hz voltage signal
//! let n = pmu.window_len();
//! let samples: Vec<f64> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / 4800.0;
//!         1.0 * (2.0 * std::f64::consts::PI * 60.0 * t).cos()
//!     })
//!     .collect();
//!
//! let phasor = pmu.estimate_phasor(&samples);
//! assert!((phasor.magnitude - 1.0).abs() < 0.01);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration & data types
// ---------------------------------------------------------------------------

/// PMU configuration parameters.
#[derive(Debug, Clone)]
pub struct PmuConfig {
    /// Nominal system frequency in Hz (typically 50.0 or 60.0).
    pub nominal_freq: f64,
    /// ADC / digitiser sample rate in samples per second.
    pub sample_rate: f64,
    /// Synchrophasor reporting rate in frames per second (e.g. 30, 60).
    pub reporting_rate: u32,
}

/// A single phasor value (magnitude + angle).
#[derive(Debug, Clone, Copy)]
pub struct Phasor {
    /// RMS magnitude (peak / sqrt(2) for sinusoidal signals, but here we
    /// report the peak-amplitude phasor consistent with C37.118 convention
    /// that a 1 pu peak sinusoid yields magnitude = 1.0).
    pub magnitude: f64,
    /// Phase angle in radians, range (-pi, pi].
    pub angle_rad: f64,
    /// UTC timestamp in microseconds since epoch.
    pub timestamp_us: u64,
}

/// Complete PMU measurement for one reporting instant.
#[derive(Debug, Clone)]
pub struct PmuMeasurement {
    /// Voltage phasor.
    pub voltage_phasor: Phasor,
    /// Current phasor.
    pub current_phasor: Phasor,
    /// Estimated frequency in Hz.
    pub frequency: f64,
    /// Rate of change of frequency in Hz/s.
    pub rocof: f64,
}

/// Three-phase symmetrical (sequence) components.
#[derive(Debug, Clone, Copy)]
pub struct SequenceComponents {
    /// Positive-sequence phasor.
    pub positive: (f64, f64),
    /// Negative-sequence phasor.
    pub negative: (f64, f64),
    /// Zero-sequence phasor.
    pub zero: (f64, f64),
}

/// Fault detection thresholds and results.
#[derive(Debug, Clone)]
pub struct FaultDetector {
    /// Overcurrent threshold (per-unit or absolute).
    pub overcurrent_threshold: f64,
    /// Undervoltage threshold (per-unit or absolute).
    pub undervoltage_threshold: f64,
}

/// Events detected by the fault detector.
#[derive(Debug, Clone, PartialEq)]
pub enum FaultEvent {
    /// Current magnitude exceeds overcurrent threshold.
    Overcurrent { magnitude: f64 },
    /// Voltage magnitude falls below undervoltage threshold.
    Undervoltage { magnitude: f64 },
}

/// Simplified C37.118 data frame representation.
#[derive(Debug, Clone)]
pub struct SynchrophasorFrame {
    /// Sync word (0xAA01 for data frame).
    pub sync: u16,
    /// Frame size in bytes.
    pub frame_size: u16,
    /// PMU/PDC ID code.
    pub id_code: u16,
    /// Second-of-century timestamp.
    pub soc: u32,
    /// Fraction of second (raw 24-bit value).
    pub frac_sec: u32,
    /// Voltage phasor (magnitude, angle_rad).
    pub voltage: (f64, f64),
    /// Current phasor (magnitude, angle_rad).
    pub current: (f64, f64),
    /// Frequency deviation from nominal in mHz.
    pub freq_deviation_mhz: f64,
    /// ROCOF in Hz/s * 100 (integer centi-Hz/s representation).
    pub rocof_100: i32,
    /// CRC-CCITT placeholder.
    pub crc: u16,
}

// ---------------------------------------------------------------------------
// Core PMU implementation
// ---------------------------------------------------------------------------

/// Synchrophasor measurement unit.
///
/// Provides DFT-based phasor extraction, frequency / ROCOF estimation,
/// TVE calculation, sequence-component analysis, fault detection and
/// C37.118 frame formatting.
#[derive(Debug, Clone)]
pub struct PhasorMeasurementUnit {
    config: PmuConfig,
    /// Number of samples in one reporting window.
    window_len: usize,
}

impl PhasorMeasurementUnit {
    /// Create a new PMU from configuration.
    pub fn new(config: PmuConfig) -> Self {
        let window_len = (config.sample_rate / config.reporting_rate as f64).round() as usize;
        Self { config, window_len }
    }

    /// Number of samples in one reporting window.
    pub fn window_len(&self) -> usize {
        self.window_len
    }

    /// Access the configuration.
    pub fn config(&self) -> &PmuConfig {
        &self.config
    }

    // -- Phasor extraction ------------------------------------------------

    /// Estimate the phasor at the nominal frequency using a single-bin DFT.
    ///
    /// The input `samples` should contain exactly `window_len()` real-valued
    /// samples.  Returns a [`Phasor`] whose magnitude equals the peak
    /// amplitude of the sinusoidal component at `nominal_freq`.
    pub fn estimate_phasor(&self, samples: &[f64]) -> Phasor {
        self.estimate_phasor_at(samples, self.config.nominal_freq, 0)
    }

    /// Estimate the phasor at an arbitrary frequency with a given timestamp.
    pub fn estimate_phasor_at(&self, samples: &[f64], freq_hz: f64, timestamp_us: u64) -> Phasor {
        let n = samples.len();
        if n == 0 {
            return Phasor {
                magnitude: 0.0,
                angle_rad: 0.0,
                timestamp_us,
            };
        }

        // Single-bin DFT at the target frequency
        let mut re = 0.0_f64;
        let mut im = 0.0_f64;
        for (i, &s) in samples.iter().enumerate() {
            let phase = 2.0 * PI * freq_hz * (i as f64) / self.config.sample_rate;
            re += s * phase.cos();
            im -= s * phase.sin();
        }

        // Scale: factor of 2/N converts to peak amplitude
        let scale = 2.0 / n as f64;
        re *= scale;
        im *= scale;

        let magnitude = (re * re + im * im).sqrt();
        let angle_rad = im.atan2(re);

        Phasor {
            magnitude,
            angle_rad,
            timestamp_us,
        }
    }

    // -- Frequency estimation ---------------------------------------------

    /// Estimate frequency via DFT phase difference between two consecutive
    /// windows of `window_len()` samples each.
    ///
    /// `prev` and `curr` are successive reporting windows.
    pub fn estimate_frequency(&self, prev: &[f64], curr: &[f64]) -> f64 {
        let p0 = self.estimate_phasor(prev);
        let p1 = self.estimate_phasor(curr);

        let mut delta_phi = p1.angle_rad - p0.angle_rad;
        // Unwrap
        while delta_phi > PI {
            delta_phi -= 2.0 * PI;
        }
        while delta_phi < -PI {
            delta_phi += 2.0 * PI;
        }

        let dt = self.window_len as f64 / self.config.sample_rate;
        self.config.nominal_freq + delta_phi / (2.0 * PI * dt)
    }

    /// Simple zero-crossing frequency estimator.
    ///
    /// Counts positive-going zero crossings in `samples` and derives frequency.
    pub fn estimate_frequency_zc(&self, samples: &[f64]) -> f64 {
        if samples.len() < 2 {
            return self.config.nominal_freq;
        }
        let mut crossings = 0u32;
        for i in 1..samples.len() {
            if samples[i - 1] < 0.0 && samples[i] >= 0.0 {
                crossings += 1;
            }
        }
        let duration = (samples.len() - 1) as f64 / self.config.sample_rate;
        if duration <= 0.0 {
            return self.config.nominal_freq;
        }
        crossings as f64 / duration
    }

    // -- ROCOF estimation -------------------------------------------------

    /// Rate of change of frequency (df/dt) in Hz/s.
    ///
    /// Requires three consecutive windows.
    pub fn estimate_rocof(&self, win0: &[f64], win1: &[f64], win2: &[f64]) -> f64 {
        let f1 = self.estimate_frequency(win0, win1);
        let f2 = self.estimate_frequency(win1, win2);
        let dt = self.window_len as f64 / self.config.sample_rate;
        (f2 - f1) / dt
    }

    // -- Total Vector Error -----------------------------------------------

    /// Compute Total Vector Error between a measured phasor and a reference
    /// phasor, per IEEE C37.118.
    ///
    /// TVE = |X_meas - X_ref| / |X_ref|
    ///
    /// Both phasors are expressed as (magnitude, angle_rad).
    pub fn total_vector_error(measured: &Phasor, reference: &Phasor) -> f64 {
        let (xm_re, xm_im) = (
            measured.magnitude * measured.angle_rad.cos(),
            measured.magnitude * measured.angle_rad.sin(),
        );
        let (xr_re, xr_im) = (
            reference.magnitude * reference.angle_rad.cos(),
            reference.magnitude * reference.angle_rad.sin(),
        );
        let diff_re = xm_re - xr_re;
        let diff_im = xm_im - xr_im;
        let error_mag = (diff_re * diff_re + diff_im * diff_im).sqrt();
        let ref_mag = (xr_re * xr_re + xr_im * xr_im).sqrt();
        if ref_mag == 0.0 {
            return f64::INFINITY;
        }
        error_mag / ref_mag
    }

    // -- Sequence components ----------------------------------------------

    /// Compute positive-, negative-, and zero-sequence components from
    /// three-phase phasors (a, b, c).
    ///
    /// Each input phasor is (magnitude, angle_rad).
    /// Uses the Fortescue transformation:
    ///   a = 1 at 120 degrees (complex rotation operator)
    ///   V0 = (Va + Vb + Vc) / 3
    ///   V1 = (Va + a*Vb + a^2*Vc) / 3
    ///   V2 = (Va + a^2*Vb + a*Vc) / 3
    pub fn sequence_components(
        phase_a: (f64, f64),
        phase_b: (f64, f64),
        phase_c: (f64, f64),
    ) -> SequenceComponents {
        // Convert polar to rectangular
        let va = (
            phase_a.0 * phase_a.1.cos(),
            phase_a.0 * phase_a.1.sin(),
        );
        let vb = (
            phase_b.0 * phase_b.1.cos(),
            phase_b.0 * phase_b.1.sin(),
        );
        let vc = (
            phase_c.0 * phase_c.1.cos(),
            phase_c.0 * phase_c.1.sin(),
        );

        // Complex rotation operator a = 1 at 120 degrees
        let a_re = (2.0 * PI / 3.0).cos();
        let a_im = (2.0 * PI / 3.0).sin();
        // a^2 = 1 at 240 degrees
        let a2_re = (4.0 * PI / 3.0).cos();
        let a2_im = (4.0 * PI / 3.0).sin();

        // Complex multiply helper: (x_re, x_im) * (y_re, y_im)
        let cmul = |xr: f64, xi: f64, yr: f64, yi: f64| -> (f64, f64) {
            (xr * yr - xi * yi, xr * yi + xi * yr)
        };

        // Zero sequence: V0 = (Va + Vb + Vc) / 3
        let v0_re = (va.0 + vb.0 + vc.0) / 3.0;
        let v0_im = (va.1 + vb.1 + vc.1) / 3.0;

        // Positive sequence: V1 = (Va + a*Vb + a^2*Vc) / 3
        let a_vb = cmul(a_re, a_im, vb.0, vb.1);
        let a2_vc = cmul(a2_re, a2_im, vc.0, vc.1);
        let v1_re = (va.0 + a_vb.0 + a2_vc.0) / 3.0;
        let v1_im = (va.1 + a_vb.1 + a2_vc.1) / 3.0;

        // Negative sequence: V2 = (Va + a^2*Vb + a*Vc) / 3
        let a2_vb = cmul(a2_re, a2_im, vb.0, vb.1);
        let a_vc = cmul(a_re, a_im, vc.0, vc.1);
        let v2_re = (va.0 + a2_vb.0 + a_vc.0) / 3.0;
        let v2_im = (va.1 + a2_vb.1 + a_vc.1) / 3.0;

        // Convert back to polar (magnitude, angle)
        let to_polar = |re: f64, im: f64| -> (f64, f64) {
            ((re * re + im * im).sqrt(), im.atan2(re))
        };

        SequenceComponents {
            zero: to_polar(v0_re, v0_im),
            positive: to_polar(v1_re, v1_im),
            negative: to_polar(v2_re, v2_im),
        }
    }

    // -- C37.118 frame formatting -----------------------------------------

    /// Format a [`PmuMeasurement`] into a simplified IEEE C37.118 data frame.
    pub fn synchrophasor_frame(
        &self,
        measurement: &PmuMeasurement,
        id_code: u16,
    ) -> SynchrophasorFrame {
        let soc = (measurement.voltage_phasor.timestamp_us / 1_000_000) as u32;
        let frac_us = (measurement.voltage_phasor.timestamp_us % 1_000_000) as u32;
        // Encode fraction as 24-bit value: frac = frac_us * time_base / 1e6
        // Using time_base = 1_000_000 so frac_sec == frac_us (fits in 24 bits for < 1s).
        let frac_sec = frac_us & 0x00FF_FFFF;

        let freq_deviation_mhz = (measurement.frequency - self.config.nominal_freq) * 1000.0;
        let rocof_100 = (measurement.rocof * 100.0).round() as i32;

        // Simplified frame size: header(14) + phasors(32) + freq(4) + rocof(4) + crc(2) = 56
        let frame_size: u16 = 56;

        // CRC placeholder -- real C37.118 uses CRC-CCITT
        let crc = crc_ccitt_placeholder(frame_size, soc, frac_sec);

        SynchrophasorFrame {
            sync: 0xAA01,
            frame_size,
            id_code,
            soc,
            frac_sec,
            voltage: (
                measurement.voltage_phasor.magnitude,
                measurement.voltage_phasor.angle_rad,
            ),
            current: (
                measurement.current_phasor.magnitude,
                measurement.current_phasor.angle_rad,
            ),
            freq_deviation_mhz,
            rocof_100,
            crc,
        }
    }

    // -- Full measurement helper ------------------------------------------

    /// Perform a complete PMU measurement from voltage and current windows.
    ///
    /// * `v_prev`, `v_curr` -- two consecutive voltage windows
    /// * `i_curr` -- current window aligned with `v_curr`
    /// * `v_prev2` -- the window before `v_prev` (needed for ROCOF).
    ///   Pass `None` to set ROCOF to 0.
    /// * `timestamp_us` -- UTC timestamp for the measurement.
    pub fn measure(
        &self,
        v_prev2: Option<&[f64]>,
        v_prev: &[f64],
        v_curr: &[f64],
        i_curr: &[f64],
        timestamp_us: u64,
    ) -> PmuMeasurement {
        let voltage_phasor = self.estimate_phasor_at(v_curr, self.config.nominal_freq, timestamp_us);
        let current_phasor = self.estimate_phasor_at(i_curr, self.config.nominal_freq, timestamp_us);
        let frequency = self.estimate_frequency(v_prev, v_curr);
        let rocof = match v_prev2 {
            Some(w0) => self.estimate_rocof(w0, v_prev, v_curr),
            None => 0.0,
        };
        PmuMeasurement {
            voltage_phasor,
            current_phasor,
            frequency,
            rocof,
        }
    }
}

// ---------------------------------------------------------------------------
// FaultDetector
// ---------------------------------------------------------------------------

impl FaultDetector {
    /// Create a new fault detector with given thresholds.
    pub fn new(overcurrent_threshold: f64, undervoltage_threshold: f64) -> Self {
        Self {
            overcurrent_threshold,
            undervoltage_threshold,
        }
    }

    /// Analyse a measurement and return any fault events detected.
    pub fn detect(&self, measurement: &PmuMeasurement) -> Vec<FaultEvent> {
        let mut events = Vec::new();
        if measurement.current_phasor.magnitude > self.overcurrent_threshold {
            events.push(FaultEvent::Overcurrent {
                magnitude: measurement.current_phasor.magnitude,
            });
        }
        if measurement.voltage_phasor.magnitude < self.undervoltage_threshold {
            events.push(FaultEvent::Undervoltage {
                magnitude: measurement.voltage_phasor.magnitude,
            });
        }
        events
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Trivial CRC placeholder (not a real CRC-CCITT implementation).
fn crc_ccitt_placeholder(frame_size: u16, soc: u32, frac_sec: u32) -> u16 {
    let mut crc: u16 = 0xFFFF;
    // Mix in some frame bytes so the value is not constant
    crc ^= frame_size;
    crc ^= (soc & 0xFFFF) as u16;
    crc ^= ((soc >> 16) & 0xFFFF) as u16;
    crc ^= (frac_sec & 0xFFFF) as u16;
    crc
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a pure cosine window at a given frequency and amplitude.
    fn cosine_window(freq: f64, amplitude: f64, phase: f64, sample_rate: f64, n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                amplitude * (2.0 * PI * freq * t + phase).cos()
            })
            .collect()
    }

    fn default_config() -> PmuConfig {
        PmuConfig {
            nominal_freq: 60.0,
            sample_rate: 4800.0,
            reporting_rate: 60,
        }
    }

    #[test]
    fn test_window_len() {
        let pmu = PhasorMeasurementUnit::new(default_config());
        // 4800 / 60 = 80 samples per window
        assert_eq!(pmu.window_len(), 80);
    }

    #[test]
    fn test_phasor_magnitude_unity() {
        let pmu = PhasorMeasurementUnit::new(default_config());
        let n = pmu.window_len();
        let samples = cosine_window(60.0, 1.0, 0.0, 4800.0, n);
        let p = pmu.estimate_phasor(&samples);
        assert!(
            (p.magnitude - 1.0).abs() < 0.01,
            "Expected magnitude ~1.0, got {}",
            p.magnitude
        );
    }

    #[test]
    fn test_phasor_magnitude_scaled() {
        let pmu = PhasorMeasurementUnit::new(default_config());
        let n = pmu.window_len();
        let samples = cosine_window(60.0, 5.0, 0.0, 4800.0, n);
        let p = pmu.estimate_phasor(&samples);
        assert!(
            (p.magnitude - 5.0).abs() < 0.05,
            "Expected magnitude ~5.0, got {}",
            p.magnitude
        );
    }

    #[test]
    fn test_phasor_angle_zero() {
        let pmu = PhasorMeasurementUnit::new(default_config());
        let n = pmu.window_len();
        let samples = cosine_window(60.0, 1.0, 0.0, 4800.0, n);
        let p = pmu.estimate_phasor(&samples);
        assert!(
            p.angle_rad.abs() < 0.05,
            "Expected angle ~0 rad, got {}",
            p.angle_rad
        );
    }

    #[test]
    fn test_phasor_angle_nonzero() {
        let pmu = PhasorMeasurementUnit::new(default_config());
        let n = pmu.window_len();
        let phase = PI / 4.0; // 45 degrees
        let samples = cosine_window(60.0, 1.0, phase, 4800.0, n);
        let p = pmu.estimate_phasor(&samples);
        assert!(
            (p.angle_rad - phase).abs() < 0.05,
            "Expected angle ~{:.3} rad, got {:.3}",
            phase,
            p.angle_rad
        );
    }

    #[test]
    fn test_frequency_estimation_nominal() {
        let pmu = PhasorMeasurementUnit::new(default_config());
        let n = pmu.window_len();
        let w0 = cosine_window(60.0, 1.0, 0.0, 4800.0, n);
        // Second window starts where first left off -- same freq, continuous phase
        let w1: Vec<f64> = (0..n)
            .map(|i| {
                let t = (i + n) as f64 / 4800.0;
                (2.0 * PI * 60.0 * t).cos()
            })
            .collect();
        let f = pmu.estimate_frequency(&w0, &w1);
        assert!(
            (f - 60.0).abs() < 0.1,
            "Expected ~60 Hz, got {}",
            f
        );
    }

    #[test]
    fn test_frequency_estimation_off_nominal() {
        let pmu = PhasorMeasurementUnit::new(default_config());
        let n = pmu.window_len();
        let actual_freq = 60.05; // slightly off nominal
        let w0: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / 4800.0;
                (2.0 * PI * actual_freq * t).cos()
            })
            .collect();
        let w1: Vec<f64> = (0..n)
            .map(|i| {
                let t = (i + n) as f64 / 4800.0;
                (2.0 * PI * actual_freq * t).cos()
            })
            .collect();
        let f = pmu.estimate_frequency(&w0, &w1);
        assert!(
            (f - actual_freq).abs() < 0.02,
            "Expected ~{} Hz, got {}",
            actual_freq,
            f
        );
    }

    #[test]
    fn test_frequency_zero_crossing() {
        let pmu = PhasorMeasurementUnit::new(default_config());
        // Use enough samples for several cycles -- 10 cycles at 60 Hz
        let n = (10.0 * 4800.0 / 60.0) as usize; // 800 samples
        let samples = cosine_window(60.0, 1.0, 0.0, 4800.0, n);
        let f = pmu.estimate_frequency_zc(&samples);
        // Zero-crossing is coarse; accept 1 Hz tolerance
        assert!(
            (f - 60.0).abs() < 1.0,
            "Expected ~60 Hz from ZC, got {}",
            f
        );
    }

    #[test]
    fn test_rocof_steady_state() {
        let pmu = PhasorMeasurementUnit::new(default_config());
        let n = pmu.window_len();
        // Three consecutive windows at constant 60 Hz
        let gen = |offset: usize| -> Vec<f64> {
            (0..n)
                .map(|i| {
                    let t = (i + offset) as f64 / 4800.0;
                    (2.0 * PI * 60.0 * t).cos()
                })
                .collect()
        };
        let w0 = gen(0);
        let w1 = gen(n);
        let w2 = gen(2 * n);
        let rocof = pmu.estimate_rocof(&w0, &w1, &w2);
        assert!(
            rocof.abs() < 0.5,
            "Expected ROCOF ~0 Hz/s at steady state, got {}",
            rocof
        );
    }

    #[test]
    fn test_tve_identical() {
        let measured = Phasor {
            magnitude: 1.0,
            angle_rad: 0.3,
            timestamp_us: 0,
        };
        let reference = Phasor {
            magnitude: 1.0,
            angle_rad: 0.3,
            timestamp_us: 0,
        };
        let tve = PhasorMeasurementUnit::total_vector_error(&measured, &reference);
        assert!(
            tve < 1e-12,
            "TVE should be ~0 for identical phasors, got {}",
            tve
        );
    }

    #[test]
    fn test_tve_magnitude_error() {
        let measured = Phasor {
            magnitude: 1.01,
            angle_rad: 0.0,
            timestamp_us: 0,
        };
        let reference = Phasor {
            magnitude: 1.0,
            angle_rad: 0.0,
            timestamp_us: 0,
        };
        let tve = PhasorMeasurementUnit::total_vector_error(&measured, &reference);
        // TVE ~ 1% for 1% magnitude error at same angle
        assert!(
            (tve - 0.01).abs() < 0.001,
            "Expected TVE ~0.01, got {}",
            tve
        );
    }

    #[test]
    fn test_sequence_components_balanced() {
        // Balanced three-phase: equal magnitudes, 120 degrees apart
        let phase_a = (1.0, 0.0_f64);
        let phase_b = (1.0, -2.0 * PI / 3.0);
        let phase_c = (1.0, 2.0 * PI / 3.0);
        let seq = PhasorMeasurementUnit::sequence_components(phase_a, phase_b, phase_c);
        // Positive sequence ~ 1.0, negative and zero ~ 0
        assert!(
            (seq.positive.0 - 1.0).abs() < 0.01,
            "Expected positive seq ~1.0, got {}",
            seq.positive.0
        );
        assert!(
            seq.negative.0 < 0.01,
            "Expected negative seq ~0, got {}",
            seq.negative.0
        );
        assert!(
            seq.zero.0 < 0.01,
            "Expected zero seq ~0, got {}",
            seq.zero.0
        );
    }

    #[test]
    fn test_fault_detector_overcurrent() {
        let detector = FaultDetector::new(10.0, 0.8);
        let meas = PmuMeasurement {
            voltage_phasor: Phasor {
                magnitude: 1.0,
                angle_rad: 0.0,
                timestamp_us: 0,
            },
            current_phasor: Phasor {
                magnitude: 15.0,
                angle_rad: 0.0,
                timestamp_us: 0,
            },
            frequency: 60.0,
            rocof: 0.0,
        };
        let events = detector.detect(&meas);
        assert_eq!(events.len(), 1);
        assert!(matches!(events[0], FaultEvent::Overcurrent { magnitude } if (magnitude - 15.0).abs() < 1e-9));
    }

    #[test]
    fn test_fault_detector_undervoltage() {
        let detector = FaultDetector::new(10.0, 0.8);
        let meas = PmuMeasurement {
            voltage_phasor: Phasor {
                magnitude: 0.5,
                angle_rad: 0.0,
                timestamp_us: 0,
            },
            current_phasor: Phasor {
                magnitude: 5.0,
                angle_rad: 0.0,
                timestamp_us: 0,
            },
            frequency: 60.0,
            rocof: 0.0,
        };
        let events = detector.detect(&meas);
        assert_eq!(events.len(), 1);
        assert!(matches!(events[0], FaultEvent::Undervoltage { magnitude } if (magnitude - 0.5).abs() < 1e-9));
    }

    #[test]
    fn test_synchrophasor_frame_format() {
        let pmu = PhasorMeasurementUnit::new(default_config());
        let meas = PmuMeasurement {
            voltage_phasor: Phasor {
                magnitude: 1.0,
                angle_rad: 0.1,
                timestamp_us: 1_700_000_500_000, // 1700000 seconds + 500000 us
            },
            current_phasor: Phasor {
                magnitude: 0.5,
                angle_rad: -0.2,
                timestamp_us: 1_700_000_500_000,
            },
            frequency: 60.01,
            rocof: 0.5,
        };
        let frame = pmu.synchrophasor_frame(&meas, 42);
        assert_eq!(frame.sync, 0xAA01);
        assert_eq!(frame.id_code, 42);
        assert_eq!(frame.soc, 1_700_000);
        assert_eq!(frame.frac_sec, 500_000);
        assert!((frame.freq_deviation_mhz - 10.0).abs() < 0.1);
        assert_eq!(frame.rocof_100, 50); // 0.5 * 100
    }

    #[test]
    fn test_50hz_system() {
        let cfg = PmuConfig {
            nominal_freq: 50.0,
            sample_rate: 10000.0,
            reporting_rate: 50,
        };
        let pmu = PhasorMeasurementUnit::new(cfg);
        assert_eq!(pmu.window_len(), 200); // 10000 / 50
        let n = pmu.window_len();
        let samples = cosine_window(50.0, 1.0, 0.0, 10000.0, n);
        let p = pmu.estimate_phasor(&samples);
        assert!(
            (p.magnitude - 1.0).abs() < 0.01,
            "50 Hz system phasor magnitude: {}",
            p.magnitude
        );
    }

    #[test]
    fn test_empty_samples() {
        let pmu = PhasorMeasurementUnit::new(default_config());
        let p = pmu.estimate_phasor(&[]);
        assert_eq!(p.magnitude, 0.0);
        assert_eq!(p.angle_rad, 0.0);
    }

    #[test]
    fn test_fault_detector_no_fault() {
        let detector = FaultDetector::new(10.0, 0.8);
        let meas = PmuMeasurement {
            voltage_phasor: Phasor {
                magnitude: 1.0,
                angle_rad: 0.0,
                timestamp_us: 0,
            },
            current_phasor: Phasor {
                magnitude: 5.0,
                angle_rad: 0.0,
                timestamp_us: 0,
            },
            frequency: 60.0,
            rocof: 0.0,
        };
        let events = detector.detect(&meas);
        assert!(events.is_empty(), "No faults should be detected");
    }

    #[test]
    fn test_measure_complete() {
        let pmu = PhasorMeasurementUnit::new(default_config());
        let n = pmu.window_len();
        let gen = |offset: usize, freq: f64, amp: f64| -> Vec<f64> {
            (0..n)
                .map(|i| {
                    let t = (i + offset) as f64 / 4800.0;
                    amp * (2.0 * PI * freq * t).cos()
                })
                .collect()
        };
        let v0 = gen(0, 60.0, 1.0);
        let v1 = gen(n, 60.0, 1.0);
        let v2 = gen(2 * n, 60.0, 1.0);
        let i2 = gen(2 * n, 60.0, 0.5);

        let m = pmu.measure(Some(&v0), &v1, &v2, &i2, 1_000_000);
        assert!((m.voltage_phasor.magnitude - 1.0).abs() < 0.02);
        assert!((m.current_phasor.magnitude - 0.5).abs() < 0.02);
        assert!((m.frequency - 60.0).abs() < 0.1);
        assert!(m.rocof.abs() < 1.0);
        assert_eq!(m.voltage_phasor.timestamp_us, 1_000_000);
    }
}
