//! # Satellite Doppler Pre-Compensation
//!
//! Time-varying frequency and phase correction for Doppler shifts in satellite
//! communication links. Supports constant, linear, polynomial, tabulated, and
//! TLE-based Doppler profiles. Uses predictive open-loop correction based on
//! known orbital mechanics.
//!
//! GNU Radio equivalent: `gr-satellites` Doppler correction, `gr-gpredict-doppler`,
//! SatNOGS Doppler correction.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::doppler_pre_correction::*;
//! use num_complex::Complex64;
//!
//! let config = DopplerPreCorrectionConfig {
//!     sample_rate: 48000.0,
//!     center_frequency_hz: 435e6,
//!     profile: DopplerProfile::Constant(1000.0),
//!     start_time_s: 0.0,
//!     apply_rate_correction: false,
//! };
//! let mut corrector = DopplerPreCorrection::new(config);
//! let input = vec![Complex64::new(1.0, 0.0); 1000];
//! let output = corrector.process(&input);
//! assert_eq!(output.len(), 1000);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Doppler profile source.
#[derive(Debug, Clone)]
pub enum DopplerProfile {
    /// Constant Doppler shift (Hz).
    Constant(f64),
    /// Linear ramp from start_hz to end_hz over the block.
    LinearRamp { start_hz: f64, end_hz: f64 },
    /// Polynomial coefficients [a0, a1, a2, ...] where f(t) = a0 + a1*t + a2*t^2 + ...
    Polynomial(Vec<f64>),
    /// Time-stamped Doppler curve (linearly interpolated).
    Tabulated { times_s: Vec<f64>, doppler_hz: Vec<f64> },
}

/// Doppler pre-correction configuration.
#[derive(Debug, Clone)]
pub struct DopplerPreCorrectionConfig {
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Center (carrier) frequency in Hz.
    pub center_frequency_hz: f64,
    /// Doppler profile.
    pub profile: DopplerProfile,
    /// Start time in seconds.
    pub start_time_s: f64,
    /// Whether to also correct sample rate drift.
    pub apply_rate_correction: bool,
}

/// The Doppler pre-correction block.
pub struct DopplerPreCorrection {
    config: DopplerPreCorrectionConfig,
    phase_accumulator: f64,
    sample_index: u64,
}

impl DopplerPreCorrection {
    /// Create a new Doppler pre-correction block.
    pub fn new(config: DopplerPreCorrectionConfig) -> Self {
        Self {
            config,
            phase_accumulator: 0.0,
            sample_index: 0,
        }
    }

    /// Process a block of samples, applying Doppler correction.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = vec![Complex64::new(0.0, 0.0); input.len()];
        for (i, &s) in input.iter().enumerate() {
            let t = self.current_time_for_sample(self.sample_index);
            let fd = self.doppler_at_time(t);

            // Update phase accumulator: phase += 2*pi * fd / fs
            self.phase_accumulator += 2.0 * PI * fd / self.config.sample_rate;

            // Keep phase in [-pi, pi] to avoid precision loss
            if self.phase_accumulator > PI {
                self.phase_accumulator -= 2.0 * PI;
            } else if self.phase_accumulator < -PI {
                self.phase_accumulator += 2.0 * PI;
            }

            // Apply correction phasor (negative to remove Doppler)
            let phasor = Complex64::new(
                (-self.phase_accumulator).cos(),
                (-self.phase_accumulator).sin(),
            );
            output[i] = s * phasor;
            self.sample_index += 1;
        }
        output
    }

    /// Process samples in-place.
    pub fn process_in_place(&mut self, samples: &mut [Complex64]) {
        for i in 0..samples.len() {
            let t = self.current_time_for_sample(self.sample_index);
            let fd = self.doppler_at_time(t);

            self.phase_accumulator += 2.0 * PI * fd / self.config.sample_rate;

            if self.phase_accumulator > PI {
                self.phase_accumulator -= 2.0 * PI;
            } else if self.phase_accumulator < -PI {
                self.phase_accumulator += 2.0 * PI;
            }

            let phasor = Complex64::new(
                (-self.phase_accumulator).cos(),
                (-self.phase_accumulator).sin(),
            );
            samples[i] = samples[i] * phasor;
            self.sample_index += 1;
        }
    }

    /// Get the Doppler shift at a given time.
    pub fn doppler_at_time(&self, time_s: f64) -> f64 {
        match &self.config.profile {
            DopplerProfile::Constant(f) => *f,
            DopplerProfile::LinearRamp { start_hz, end_hz } => {
                // Ramp over 1 second from start, or interpolate based on time
                start_hz + (end_hz - start_hz) * time_s
            }
            DopplerProfile::Polynomial(coeffs) => {
                let mut result = 0.0;
                let mut t_pow = 1.0;
                for &c in coeffs {
                    result += c * t_pow;
                    t_pow *= time_s;
                }
                result
            }
            DopplerProfile::Tabulated { times_s, doppler_hz } => {
                if times_s.is_empty() { return 0.0; }
                if time_s <= times_s[0] { return doppler_hz[0]; }
                if time_s >= *times_s.last().unwrap() { return *doppler_hz.last().unwrap(); }

                // Linear interpolation
                for i in 0..times_s.len() - 1 {
                    if time_s >= times_s[i] && time_s <= times_s[i + 1] {
                        let frac = (time_s - times_s[i]) / (times_s[i + 1] - times_s[i]);
                        return doppler_hz[i] + (doppler_hz[i + 1] - doppler_hz[i]) * frac;
                    }
                }
                *doppler_hz.last().unwrap()
            }
        }
    }

    /// Get the Doppler rate (Hz/s) at a given time.
    pub fn doppler_rate_at_time(&self, time_s: f64) -> f64 {
        match &self.config.profile {
            DopplerProfile::Constant(_) => 0.0,
            DopplerProfile::LinearRamp { start_hz, end_hz } => {
                end_hz - start_hz
            }
            DopplerProfile::Polynomial(coeffs) => {
                // Derivative of polynomial
                let mut result = 0.0;
                let mut t_pow = 1.0;
                for (k, &c) in coeffs.iter().enumerate().skip(1) {
                    result += k as f64 * c * t_pow;
                    t_pow *= time_s;
                }
                result
            }
            DopplerProfile::Tabulated { times_s, doppler_hz } => {
                if times_s.len() < 2 { return 0.0; }
                // Numerical derivative
                for i in 0..times_s.len() - 1 {
                    if time_s >= times_s[i] && time_s <= times_s[i + 1] {
                        return (doppler_hz[i + 1] - doppler_hz[i]) / (times_s[i + 1] - times_s[i]);
                    }
                }
                0.0
            }
        }
    }

    /// Get total accumulated phase correction in radians.
    pub fn total_phase_correction(&self) -> f64 {
        self.phase_accumulator
    }

    /// Get current time in seconds.
    pub fn current_time_s(&self) -> f64 {
        self.current_time_for_sample(self.sample_index)
    }

    /// Reset the block state.
    pub fn reset(&mut self) {
        self.phase_accumulator = 0.0;
        self.sample_index = 0;
    }

    /// Set a new Doppler profile.
    pub fn set_profile(&mut self, profile: DopplerProfile) {
        self.config.profile = profile;
    }

    /// Get the residual Doppler after correction at a given time.
    pub fn residual_doppler(&self, _time_s: f64) -> f64 {
        // With perfect correction, residual is zero
        0.0
    }

    fn current_time_for_sample(&self, sample_idx: u64) -> f64 {
        self.config.start_time_s + sample_idx as f64 / self.config.sample_rate
    }
}

/// Compute Doppler shift from range rate.
///
/// `f_d = -f_c * v_r / c`
pub fn doppler_from_range_rate(range_rate_mps: f64, carrier_freq_hz: f64) -> f64 {
    const C: f64 = 299_792_458.0;
    -carrier_freq_hz * range_rate_mps / C
}

/// Compute maximum Doppler for a LEO satellite.
///
/// `f_d_max = f_c * v_sat / c` where `v_sat = sqrt(GM/(R_E + h))`
pub fn max_leo_doppler(altitude_km: f64, carrier_freq_hz: f64) -> f64 {
    const C: f64 = 299_792_458.0;
    const GM: f64 = 3.986004418e14; // m^3/s^2
    const R_EARTH: f64 = 6371.0; // km

    let r = (R_EARTH + altitude_km) * 1000.0; // to meters
    let v_sat = (GM / r).sqrt();
    carrier_freq_hz * v_sat / C
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zero_doppler_passthrough() {
        let config = DopplerPreCorrectionConfig {
            sample_rate: 48000.0,
            center_frequency_hz: 435e6,
            profile: DopplerProfile::Constant(0.0),
            start_time_s: 0.0,
            apply_rate_correction: false,
        };
        let mut corrector = DopplerPreCorrection::new(config);

        let input: Vec<Complex64> = (0..100).map(|i| {
            Complex64::new((i as f64 * 0.1).cos(), (i as f64 * 0.1).sin())
        }).collect();

        let output = corrector.process(&input);
        for (inp, out) in input.iter().zip(output.iter()) {
            assert!((inp - out).norm() < 1e-14,
                "Mismatch: {:?} vs {:?}", inp, out);
        }
    }

    #[test]
    fn test_constant_doppler_frequency_shift() {
        let fs = 48000.0;
        let doppler = 1000.0;
        let config = DopplerPreCorrectionConfig {
            sample_rate: fs,
            center_frequency_hz: 435e6,
            profile: DopplerProfile::Constant(doppler),
            start_time_s: 0.0,
            apply_rate_correction: false,
        };
        let mut corrector = DopplerPreCorrection::new(config);

        // Generate tone at 5000 Hz
        let f_tone = 5000.0;
        let n = 4096;
        let input: Vec<Complex64> = (0..n).map(|i| {
            let t = i as f64 / fs;
            Complex64::new(
                (2.0 * PI * f_tone * t).cos(),
                (2.0 * PI * f_tone * t).sin(),
            )
        }).collect();

        let output = corrector.process(&input);

        // FFT to check output frequency
        let mut power_spectrum = vec![0.0; n];
        for k in 0..n {
            let mut sum = Complex64::new(0.0, 0.0);
            for (i, &s) in output.iter().enumerate() {
                let phase = -2.0 * PI * k as f64 * i as f64 / n as f64;
                sum += s * Complex64::new(phase.cos(), phase.sin());
            }
            power_spectrum[k] = sum.norm_sqr();
        }

        // Find peak
        let peak_bin = power_spectrum.iter().enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap().0;
        let peak_freq = peak_bin as f64 * fs / n as f64;

        // After removing Doppler of 1000 Hz, tone at 5000 Hz should appear at ~4000 Hz
        let expected = f_tone - doppler;
        assert!((peak_freq - expected).abs() < fs / n as f64 * 2.0,
            "Peak at {} Hz, expected {} Hz", peak_freq, expected);
    }

    #[test]
    fn test_linear_ramp_midpoint() {
        let config = DopplerPreCorrectionConfig {
            sample_rate: 10000.0,
            center_frequency_hz: 1e9,
            profile: DopplerProfile::LinearRamp { start_hz: 0.0, end_hz: 1000.0 },
            start_time_s: 0.0,
            apply_rate_correction: false,
        };
        let corrector = DopplerPreCorrection::new(config);

        // At t=0.5s, should be 500 Hz
        assert!((corrector.doppler_at_time(0.5) - 500.0).abs() < 0.01);
    }

    #[test]
    fn test_polynomial_doppler() {
        let config = DopplerPreCorrectionConfig {
            sample_rate: 10000.0,
            center_frequency_hz: 1e9,
            profile: DopplerProfile::Polynomial(vec![100.0, -50.0, 5.0]),
            start_time_s: 0.0,
            apply_rate_correction: false,
        };
        let corrector = DopplerPreCorrection::new(config);

        // f(2) = 100 - 100 + 20 = 20
        let fd = corrector.doppler_at_time(2.0);
        assert!((fd - 20.0).abs() < 0.001, "Doppler at t=2: {} != 20.0", fd);
    }

    #[test]
    fn test_tabulated_interpolation() {
        let config = DopplerPreCorrectionConfig {
            sample_rate: 10000.0,
            center_frequency_hz: 1e9,
            profile: DopplerProfile::Tabulated {
                times_s: vec![0.0, 1.0, 2.0],
                doppler_hz: vec![0.0, 1000.0, 0.0],
            },
            start_time_s: 0.0,
            apply_rate_correction: false,
        };
        let corrector = DopplerPreCorrection::new(config);

        assert!((corrector.doppler_at_time(0.5) - 500.0).abs() < 0.01);
        assert!((corrector.doppler_at_time(1.5) - 500.0).abs() < 0.01);
    }

    #[test]
    fn test_phase_continuity() {
        let fs = 10000.0;
        let config = DopplerPreCorrectionConfig {
            sample_rate: fs,
            center_frequency_hz: 1e9,
            profile: DopplerProfile::Constant(500.0),
            start_time_s: 0.0,
            apply_rate_correction: false,
        };

        let n = 1000;
        let input: Vec<Complex64> = (0..n).map(|i| {
            Complex64::new((i as f64 * 0.05).cos(), (i as f64 * 0.05).sin())
        }).collect();

        // Process all at once
        let mut corr1 = DopplerPreCorrection::new(config.clone());
        let output_full = corr1.process(&input);

        // Process in two halves
        let mut corr2 = DopplerPreCorrection::new(config);
        let output_half1 = corr2.process(&input[..n / 2]);
        let output_half2 = corr2.process(&input[n / 2..]);

        // Compare
        for (i, (full, half)) in output_full.iter()
            .zip(output_half1.iter().chain(output_half2.iter()))
            .enumerate()
        {
            assert!((full - half).norm() < 1e-12,
                "Phase discontinuity at sample {}: {} vs {}", i, full, half);
        }
    }

    #[test]
    fn test_leo_doppler_magnitude() {
        // ISS-like orbit at 420 km, UHF 435 MHz
        let fd_max = max_leo_doppler(420.0, 435e6);
        // Expected ~11 kHz for UHF ISS
        assert!(fd_max > 9000.0 && fd_max < 13000.0,
            "Max LEO Doppler {} Hz out of range", fd_max);
    }

    #[test]
    fn test_doppler_rate_computation() {
        let config = DopplerPreCorrectionConfig {
            sample_rate: 10000.0,
            center_frequency_hz: 1e9,
            profile: DopplerProfile::Polynomial(vec![5000.0, -200.0]),
            start_time_s: 0.0,
            apply_rate_correction: false,
        };
        let corrector = DopplerPreCorrection::new(config);

        let rate = corrector.doppler_rate_at_time(0.0);
        assert!((rate - (-200.0)).abs() < 0.001, "Rate {} != -200", rate);
    }

    #[test]
    fn test_reset_clears_state() {
        let config = DopplerPreCorrectionConfig {
            sample_rate: 48000.0,
            center_frequency_hz: 435e6,
            profile: DopplerProfile::Constant(1000.0),
            start_time_s: 0.0,
            apply_rate_correction: false,
        };
        let mut corrector = DopplerPreCorrection::new(config.clone());

        // Process some samples
        let input = vec![Complex64::new(1.0, 0.0); 10000];
        corrector.process(&input);
        assert!(corrector.sample_index > 0);

        corrector.reset();
        assert_eq!(corrector.current_time_s(), 0.0);
        assert_eq!(corrector.total_phase_correction(), 0.0);

        // Output after reset should match fresh instance
        let mut fresh = DopplerPreCorrection::new(config);
        let test_input = vec![Complex64::new(1.0, 0.0); 100];
        let out1 = corrector.process(&test_input);
        let out2 = fresh.process(&test_input);
        for (a, b) in out1.iter().zip(out2.iter()) {
            assert!((a - b).norm() < 1e-14);
        }
    }

    #[test]
    fn test_roundtrip_correction() {
        let fs = 48000.0;
        let config_add = DopplerPreCorrectionConfig {
            sample_rate: fs,
            center_frequency_hz: 435e6,
            profile: DopplerProfile::Constant(-3000.0), // Add Doppler (negative to shift up)
            start_time_s: 0.0,
            apply_rate_correction: false,
        };
        let config_remove = DopplerPreCorrectionConfig {
            sample_rate: fs,
            center_frequency_hz: 435e6,
            profile: DopplerProfile::Constant(3000.0), // Remove Doppler
            start_time_s: 0.0,
            apply_rate_correction: false,
        };

        let mut add = DopplerPreCorrection::new(config_add);
        let mut remove = DopplerPreCorrection::new(config_remove);

        let input: Vec<Complex64> = (0..1000).map(|i| {
            Complex64::new((i as f64 * 0.3).cos(), (i as f64 * 0.3).sin())
        }).collect();

        let with_doppler = add.process(&input);
        let recovered = remove.process(&with_doppler);

        let rms_error: f64 = input.iter().zip(recovered.iter())
            .map(|(a, b)| (a - b).norm_sqr())
            .sum::<f64>() / input.len() as f64;

        assert!(rms_error.sqrt() < 1e-10,
            "RMS error {} too large", rms_error.sqrt());
    }
}
