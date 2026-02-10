//! Chip-Scale Atomic Clock (CSAC) and crystal oscillator reference source simulation.
//!
//! This module models reference oscillators with realistic Allan deviation, phase noise,
//! frequency aging, and temperature sensitivity. It supports five oscillator types from
//! low-cost TCXOs to high-stability cesium standards.
//!
//! # Oscillator Types
//!
//! | Type     | Typical ADEV (1s) | Aging/day       | Use case              |
//! |----------|-------------------|-----------------|-----------------------|
//! | TCXO     | 1e-9              | 0.5 ppm/year    | Consumer GPS, IoT     |
//! | OCXO     | 1e-12             | 5e-10/day       | Base stations, radar  |
//! | Rubidium | 1e-11             | 5e-11/month     | Telecom, test equip   |
//! | CSAC     | 3e-10             | 9e-10/month     | Portable precision    |
//! | Cesium   | 1e-11             | ~0 (primary)    | Metrology, UTC        |
//!
//! # Example
//!
//! ```rust
//! use r4w_core::csac_reference_oscillator::{CsacReferenceOscillator, OscillatorType};
//!
//! // Create a CSAC oscillator at 10 MHz
//! let mut osc = CsacReferenceOscillator::new(OscillatorType::Csac, 10_000_000.0);
//!
//! // Generate 100 samples at 1-second intervals
//! let samples: Vec<(f64, f64)> = (0..100)
//!     .map(|i| osc.sample(i as f64))
//!     .collect();
//!
//! // Compute Allan deviation from frequency data
//! let freqs: Vec<f64> = (0..1000).map(|i| {
//!     let (re, im) = osc.sample(i as f64);
//!     let phase = im.atan2(re);
//!     10_000_000.0 + phase * 10.0
//! }).collect();
//! let adev = CsacReferenceOscillator::allan_deviation(&freqs, 1.0);
//! assert!(!adev.is_empty());
//! ```

use std::f64::consts::PI;

/// Oscillator technology type, each with characteristic stability parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OscillatorType {
    /// Temperature-Compensated Crystal Oscillator
    Tcxo,
    /// Oven-Controlled Crystal Oscillator
    Ocxo,
    /// Rubidium vapor cell atomic oscillator
    Rubidium,
    /// Chip-Scale Atomic Clock
    Csac,
    /// Cesium beam primary frequency standard
    Cesium,
}

/// Allan deviation noise coefficients (h0, h_neg1, h_neg2).
///
/// These correspond to white FM noise, flicker FM noise, and random walk FM noise
/// power spectral density coefficients, respectively.
#[derive(Debug, Clone, Copy)]
pub struct NoiseCoefficients {
    /// h0: White frequency noise coefficient
    pub h0: f64,
    /// h_{-1}: Flicker frequency noise coefficient
    pub h_neg1: f64,
    /// h_{-2}: Random walk frequency noise coefficient
    pub h_neg2: f64,
}

/// Frequency aging model parameters.
#[derive(Debug, Clone, Copy)]
pub struct AgingModel {
    /// Linear drift rate (fractional frequency per second)
    pub linear_rate: f64,
    /// Logarithmic drift coefficient (fractional frequency)
    pub log_coefficient: f64,
}

/// Temperature sensitivity parameters.
#[derive(Debug, Clone, Copy)]
pub struct TemperatureModel {
    /// Temperature coefficient (fractional frequency per degree C)
    pub coefficient: f64,
    /// Reference temperature (degrees C)
    pub reference_temp: f64,
    /// Current temperature (degrees C)
    pub current_temp: f64,
}

/// A simulated reference oscillator with Allan deviation, aging, temperature,
/// and phase noise models.
///
/// The oscillator produces complex (I/Q) samples representing the output signal
/// with accumulated phase noise and frequency drift.
#[derive(Debug, Clone)]
pub struct CsacReferenceOscillator {
    /// Oscillator technology type
    osc_type: OscillatorType,
    /// Nominal output frequency in Hz
    nominal_freq: f64,
    /// Allan deviation noise coefficients
    noise_coeffs: NoiseCoefficients,
    /// Frequency aging model
    aging: AgingModel,
    /// Temperature sensitivity model
    temperature: TemperatureModel,
    /// Accumulated phase error (radians)
    accumulated_phase_error: f64,
    /// Simple PRNG state for deterministic noise generation
    rng_state: u64,
    /// Start time reference for aging computation
    start_time: f64,
}

impl OscillatorType {
    /// Returns the default noise coefficients for this oscillator type.
    ///
    /// Values are representative of typical devices as documented in the
    /// oscillator stability literature.
    pub fn default_noise_coefficients(&self) -> NoiseCoefficients {
        match self {
            OscillatorType::Tcxo => NoiseCoefficients {
                h0: 2.0e-19,
                h_neg1: 3.0e-21,
                h_neg2: 1.0e-24,
            },
            OscillatorType::Ocxo => NoiseCoefficients {
                h0: 2.0e-25,
                h_neg1: 1.0e-26,
                h_neg2: 4.0e-29,
            },
            OscillatorType::Rubidium => NoiseCoefficients {
                h0: 2.0e-23,
                h_neg1: 4.0e-26,
                h_neg2: 1.5e-29,
            },
            OscillatorType::Csac => NoiseCoefficients {
                h0: 1.8e-20,
                h_neg1: 7.0e-23,
                h_neg2: 2.0e-25,
            },
            OscillatorType::Cesium => NoiseCoefficients {
                h0: 2.0e-23,
                h_neg1: 1.0e-27,
                h_neg2: 3.0e-32,
            },
        }
    }

    /// Returns the default aging model for this oscillator type.
    pub fn default_aging_model(&self) -> AgingModel {
        match self {
            OscillatorType::Tcxo => AgingModel {
                linear_rate: 5.0e-7 / (365.25 * 86400.0), // 0.5 ppm/year
                log_coefficient: 1.0e-8,
            },
            OscillatorType::Ocxo => AgingModel {
                linear_rate: 5.0e-10 / 86400.0, // 5e-10/day
                log_coefficient: 1.0e-11,
            },
            OscillatorType::Rubidium => AgingModel {
                linear_rate: 5.0e-11 / (30.0 * 86400.0), // 5e-11/month
                log_coefficient: 2.0e-13,
            },
            OscillatorType::Csac => AgingModel {
                linear_rate: 9.0e-10 / (30.0 * 86400.0), // 9e-10/month
                log_coefficient: 3.0e-12,
            },
            OscillatorType::Cesium => AgingModel {
                linear_rate: 0.0, // Primary standard, no aging
                log_coefficient: 0.0,
            },
        }
    }

    /// Returns the default temperature sensitivity model for this oscillator type.
    pub fn default_temperature_model(&self) -> TemperatureModel {
        match self {
            OscillatorType::Tcxo => TemperatureModel {
                coefficient: 1.0e-6,    // 1 ppm/degC
                reference_temp: 25.0,
                current_temp: 25.0,
            },
            OscillatorType::Ocxo => TemperatureModel {
                coefficient: 1.0e-9,    // 1 ppb/degC (oven-controlled)
                reference_temp: 25.0,
                current_temp: 25.0,
            },
            OscillatorType::Rubidium => TemperatureModel {
                coefficient: 5.0e-11,   // Very low, atomic reference
                reference_temp: 25.0,
                current_temp: 25.0,
            },
            OscillatorType::Csac => TemperatureModel {
                coefficient: 3.0e-10,   // ~0.3 ppb/degC
                reference_temp: 25.0,
                current_temp: 25.0,
            },
            OscillatorType::Cesium => TemperatureModel {
                coefficient: 1.0e-13,   // Negligible
                reference_temp: 25.0,
                current_temp: 25.0,
            },
        }
    }
}

impl CsacReferenceOscillator {
    /// Creates a new reference oscillator with default parameters for the given type.
    ///
    /// # Arguments
    ///
    /// * `osc_type` - Oscillator technology type
    /// * `nominal_freq` - Nominal output frequency in Hz (e.g. 10_000_000.0 for 10 MHz)
    pub fn new(osc_type: OscillatorType, nominal_freq: f64) -> Self {
        Self {
            osc_type,
            nominal_freq,
            noise_coeffs: osc_type.default_noise_coefficients(),
            aging: osc_type.default_aging_model(),
            temperature: osc_type.default_temperature_model(),
            accumulated_phase_error: 0.0,
            rng_state: 0x5DEE_CE66_D1A4_F681,
            start_time: 0.0,
        }
    }

    /// Creates a new oscillator with custom noise coefficients.
    pub fn with_noise_coefficients(mut self, coeffs: NoiseCoefficients) -> Self {
        self.noise_coeffs = coeffs;
        self
    }

    /// Creates a new oscillator with a custom aging model.
    pub fn with_aging_model(mut self, aging: AgingModel) -> Self {
        self.aging = aging;
        self
    }

    /// Creates a new oscillator with a custom temperature model.
    pub fn with_temperature_model(mut self, temp: TemperatureModel) -> Self {
        self.temperature = temp;
        self
    }

    /// Sets the PRNG seed for deterministic noise generation.
    pub fn with_seed(mut self, seed: u64) -> Self {
        self.rng_state = seed;
        self
    }

    /// Sets the start time for aging computation.
    pub fn with_start_time(mut self, t0: f64) -> Self {
        self.start_time = t0;
        self
    }

    /// Sets the current ambient temperature in degrees Celsius.
    pub fn set_temperature(&mut self, temp_c: f64) {
        self.temperature.current_temp = temp_c;
    }

    /// Returns the oscillator type.
    pub fn oscillator_type(&self) -> OscillatorType {
        self.osc_type
    }

    /// Returns the nominal frequency in Hz.
    pub fn nominal_frequency(&self) -> f64 {
        self.nominal_freq
    }

    /// Returns the current noise coefficients.
    pub fn noise_coefficients(&self) -> &NoiseCoefficients {
        &self.noise_coeffs
    }

    /// Returns the current aging model.
    pub fn aging_model(&self) -> &AgingModel {
        &self.aging
    }

    /// Returns the current temperature model.
    pub fn temperature_model(&self) -> &TemperatureModel {
        &self.temperature
    }

    /// Computes the fractional frequency offset from aging at elapsed time `dt` seconds.
    ///
    /// The aging model combines a linear drift with a logarithmic component:
    /// `y_aging(dt) = linear_rate * dt + log_coefficient * ln(1 + dt)`
    pub fn aging_offset(&self, dt: f64) -> f64 {
        if dt <= 0.0 {
            return 0.0;
        }
        self.aging.linear_rate * dt + self.aging.log_coefficient * (1.0 + dt).ln()
    }

    /// Computes the fractional frequency offset due to temperature deviation.
    ///
    /// `y_temp = coefficient * (T_current - T_reference)`
    pub fn temperature_offset(&self) -> f64 {
        self.temperature.coefficient * (self.temperature.current_temp - self.temperature.reference_temp)
    }

    /// Generates a pseudo-random Gaussian sample using the Box-Muller transform.
    ///
    /// Uses an internal xorshift64* PRNG for deterministic generation.
    fn gaussian_sample(&mut self) -> f64 {
        let u1 = self.uniform_sample();
        let u2 = self.uniform_sample();
        // Box-Muller transform
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }

    /// Generates a uniform [0, 1) sample from the internal PRNG.
    fn uniform_sample(&mut self) -> f64 {
        // xorshift64*
        self.rng_state ^= self.rng_state >> 12;
        self.rng_state ^= self.rng_state << 25;
        self.rng_state ^= self.rng_state >> 27;
        let val = self.rng_state.wrapping_mul(0x2545_F491_4F6C_DD1D);
        // Convert to [0, 1), avoid exact 0 for log safety
        (val >> 11) as f64 / (1u64 << 53) as f64 + f64::EPSILON
    }

    /// Computes the phase noise standard deviation for a given averaging time `tau`.
    ///
    /// Based on the Allan variance model:
    /// `sigma_y^2(tau) = h0 / (2*tau) + 2*ln(2)*h_{-1} + (2*pi^2/3)*h_{-2}*tau`
    ///
    /// The phase noise standard deviation in radians is then:
    /// `sigma_phi = 2*pi*f0*tau*sigma_y`
    pub fn phase_noise_std(&self, tau: f64) -> f64 {
        if tau <= 0.0 {
            return 0.0;
        }
        let sigma_y_sq = self.allan_variance_model(tau);
        let sigma_y = sigma_y_sq.sqrt();
        2.0 * PI * self.nominal_freq * tau * sigma_y
    }

    /// Computes the modeled Allan variance for a given averaging time `tau`.
    ///
    /// `sigma_y^2(tau) = h0 / (2*tau) + 2*ln(2)*h_{-1} + (2*pi^2/3)*h_{-2}*tau`
    pub fn allan_variance_model(&self, tau: f64) -> f64 {
        if tau <= 0.0 {
            return 0.0;
        }
        let h0 = self.noise_coeffs.h0;
        let h1 = self.noise_coeffs.h_neg1;
        let h2 = self.noise_coeffs.h_neg2;
        h0 / (2.0 * tau) + 2.0 * 2.0_f64.ln() * h1 + (2.0 * PI * PI / 3.0) * h2 * tau
    }

    /// Generates a complex (I/Q) output sample at time `t` seconds.
    ///
    /// The output includes:
    /// - Nominal frequency tone
    /// - Phase noise from Allan deviation parameters
    /// - Frequency aging drift
    /// - Temperature-induced offset
    ///
    /// Returns `(re, im)` as a complex sample.
    pub fn sample(&mut self, t: f64) -> (f64, f64) {
        let dt = t - self.start_time;

        // Fractional frequency offsets
        let y_aging = self.aging_offset(dt);
        let y_temp = self.temperature_offset();

        // Phase noise: accumulate random walk with step size scaled by noise model
        let sigma_y_sq = self.allan_variance_model(1.0);
        let sigma_y = sigma_y_sq.sqrt();
        let freq_noise = sigma_y * self.gaussian_sample();

        self.accumulated_phase_error += 2.0 * PI * self.nominal_freq * freq_noise;

        // Total instantaneous frequency
        let f_total = self.nominal_freq * (1.0 + y_aging + y_temp);

        // Total phase
        let phase = 2.0 * PI * f_total * t + self.accumulated_phase_error;

        (phase.cos(), phase.sin())
    }

    /// Generates a batch of complex samples at uniform time intervals.
    ///
    /// # Arguments
    ///
    /// * `start_time` - Time of the first sample (seconds)
    /// * `sample_rate` - Sample rate in Hz
    /// * `num_samples` - Number of samples to generate
    pub fn generate_samples(
        &mut self,
        start_time: f64,
        sample_rate: f64,
        num_samples: usize,
    ) -> Vec<(f64, f64)> {
        let dt = 1.0 / sample_rate;
        (0..num_samples)
            .map(|i| {
                let t = start_time + i as f64 * dt;
                self.sample(t)
            })
            .collect()
    }

    /// Computes the Allan deviation from a series of frequency measurements.
    ///
    /// # Arguments
    ///
    /// * `freq_data` - Frequency measurements at uniform intervals
    /// * `tau0` - Base sampling interval in seconds
    ///
    /// # Returns
    ///
    /// Vector of `(tau, adev)` pairs for averaging factors 1, 2, 4, 8, ...
    pub fn allan_deviation(freq_data: &[f64], tau0: f64) -> Vec<(f64, f64)> {
        if freq_data.len() < 3 || tau0 <= 0.0 {
            return Vec::new();
        }

        let n = freq_data.len();
        let mut results = Vec::new();
        let mut m = 1usize;

        while m <= n / 2 {
            let tau = m as f64 * tau0;

            // Compute non-overlapping Allan variance
            // Average frequency over blocks of m samples
            let num_blocks = n / m;
            if num_blocks < 2 {
                break;
            }

            let mut block_avgs: Vec<f64> = Vec::with_capacity(num_blocks);
            for b in 0..num_blocks {
                let start = b * m;
                let end = start + m;
                let avg: f64 = freq_data[start..end].iter().sum::<f64>() / m as f64;
                block_avgs.push(avg);
            }

            let mut sum = 0.0;
            let mut count = 0usize;
            for i in 0..block_avgs.len() - 1 {
                let diff = block_avgs[i + 1] - block_avgs[i];
                sum += diff * diff;
                count += 1;
            }

            if count > 0 {
                let avar = sum / (2.0 * count as f64);
                results.push((tau, avar.sqrt()));
            }

            m *= 2;
        }

        results
    }

    /// Computes the overlapping Allan deviation from frequency data.
    ///
    /// More statistically efficient than the non-overlapping version.
    ///
    /// # Arguments
    ///
    /// * `freq_data` - Frequency measurements at uniform intervals
    /// * `tau0` - Base sampling interval in seconds
    ///
    /// # Returns
    ///
    /// Vector of `(tau, adev)` pairs
    pub fn overlapping_allan_deviation(freq_data: &[f64], tau0: f64) -> Vec<(f64, f64)> {
        if freq_data.len() < 3 || tau0 <= 0.0 {
            return Vec::new();
        }

        let n = freq_data.len();
        let mut results = Vec::new();
        let mut m = 1usize;

        while 2 * m < n {
            let tau = m as f64 * tau0;

            // Compute overlapping block averages
            let num_avgs = n - m + 1;
            let mut block_avgs: Vec<f64> = Vec::with_capacity(num_avgs);
            let mut running_sum: f64 = freq_data[..m].iter().sum();
            block_avgs.push(running_sum / m as f64);

            for i in 1..num_avgs {
                running_sum += freq_data[i + m - 1] - freq_data[i - 1];
                block_avgs.push(running_sum / m as f64);
            }

            let mut sum = 0.0;
            let mut count = 0usize;
            for i in 0..block_avgs.len().saturating_sub(m) {
                let diff = block_avgs[i + m] - block_avgs[i];
                sum += diff * diff;
                count += 1;
            }

            if count > 0 {
                let avar = sum / (2.0 * count as f64);
                results.push((tau, avar.sqrt()));
            }

            m *= 2;
        }

        results
    }

    /// Computes the Modified Allan Deviation (MDEV) from frequency data.
    ///
    /// MDEV is able to distinguish between white and flicker phase noise,
    /// unlike the standard Allan deviation.
    ///
    /// # Arguments
    ///
    /// * `freq_data` - Frequency measurements at uniform intervals
    /// * `tau0` - Base sampling interval in seconds
    ///
    /// # Returns
    ///
    /// Vector of `(tau, mdev)` pairs
    pub fn modified_allan_deviation(freq_data: &[f64], tau0: f64) -> Vec<(f64, f64)> {
        if freq_data.len() < 4 || tau0 <= 0.0 {
            return Vec::new();
        }

        // First, convert frequency data to phase data
        // x[i] = tau0 * sum(y[0..i])
        let n = freq_data.len();
        let mut phase_data: Vec<f64> = Vec::with_capacity(n + 1);
        phase_data.push(0.0);
        for i in 0..n {
            phase_data.push(phase_data[i] + freq_data[i] * tau0);
        }

        Self::modified_allan_deviation_from_phase(&phase_data, tau0)
    }

    /// Computes the Modified Allan Deviation from phase data.
    ///
    /// # Arguments
    ///
    /// * `phase_data` - Phase measurements (seconds of time error) at uniform intervals
    /// * `tau0` - Base sampling interval in seconds
    fn modified_allan_deviation_from_phase(phase_data: &[f64], tau0: f64) -> Vec<(f64, f64)> {
        let n = phase_data.len();
        if n < 4 || tau0 <= 0.0 {
            return Vec::new();
        }

        let mut results = Vec::new();
        let mut m = 1usize;

        while 3 * m < n {
            let tau = m as f64 * tau0;

            // MDEV uses a second difference averaged over m points
            let mut outer_sum = 0.0;
            let mut outer_count = 0usize;

            for j in 0..n.saturating_sub(3 * m) {
                let mut inner_sum = 0.0;
                for i in j..j + m {
                    if i + 2 * m < phase_data.len() {
                        inner_sum += phase_data[i + 2 * m] - 2.0 * phase_data[i + m] + phase_data[i];
                    }
                }
                outer_sum += inner_sum * inner_sum;
                outer_count += 1;
            }

            if outer_count > 0 {
                let mvar = outer_sum
                    / (2.0 * (m as f64).powi(2) * tau.powi(2) * outer_count as f64);
                results.push((tau, mvar.sqrt()));
            }

            m *= 2;
        }

        results
    }

    /// Computes the Time Interval Error (TIE) from phase/time error data.
    ///
    /// TIE measures the maximum peak-to-peak variation of the time error
    /// over a given observation window.
    ///
    /// # Arguments
    ///
    /// * `phase_data` - Phase/time error samples at uniform intervals (seconds)
    /// * `tau0` - Base sampling interval in seconds
    ///
    /// # Returns
    ///
    /// Vector of `(window_duration, tie)` pairs in seconds
    pub fn time_interval_error(phase_data: &[f64], tau0: f64) -> Vec<(f64, f64)> {
        if phase_data.len() < 2 || tau0 <= 0.0 {
            return Vec::new();
        }

        let n = phase_data.len();
        let mut results = Vec::new();
        let mut window = 2usize; // Need at least 2 samples for meaningful TIE

        while window <= n {
            let duration = window as f64 * tau0;
            let mut max_tie = 0.0_f64;

            for start in 0..=n.saturating_sub(window) {
                let end = (start + window).min(n);
                let slice = &phase_data[start..end];
                if slice.is_empty() {
                    continue;
                }
                let min = slice.iter().cloned().fold(f64::INFINITY, f64::min);
                let max = slice.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                let tie = max - min;
                if tie > max_tie {
                    max_tie = tie;
                }
            }

            results.push((duration, max_tie));

            // Increase window size: double each step
            if window < 8 {
                window += 2;
            } else {
                window *= 2;
            }
        }

        results
    }

    /// Computes the theoretical Allan deviation from the oscillator's noise model.
    ///
    /// Returns `(tau, adev)` pairs for logarithmically spaced tau values.
    pub fn theoretical_adev(&self, tau_min: f64, tau_max: f64, num_points: usize) -> Vec<(f64, f64)> {
        if num_points == 0 || tau_min <= 0.0 || tau_max <= tau_min {
            return Vec::new();
        }

        let log_min = tau_min.ln();
        let log_max = tau_max.ln();

        (0..num_points)
            .map(|i| {
                let tau = if num_points == 1 {
                    tau_min
                } else {
                    (log_min + (log_max - log_min) * i as f64 / (num_points - 1) as f64).exp()
                };
                let adev = self.allan_variance_model(tau).sqrt();
                (tau, adev)
            })
            .collect()
    }

    /// Resets the internal accumulated phase error and PRNG state.
    pub fn reset(&mut self) {
        self.accumulated_phase_error = 0.0;
        self.rng_state = 0x5DEE_CE66_D1A4_F681;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-12;

    #[test]
    fn test_create_all_oscillator_types() {
        let types = [
            OscillatorType::Tcxo,
            OscillatorType::Ocxo,
            OscillatorType::Rubidium,
            OscillatorType::Csac,
            OscillatorType::Cesium,
        ];
        for osc_type in &types {
            let osc = CsacReferenceOscillator::new(*osc_type, 10_000_000.0);
            assert_eq!(osc.oscillator_type(), *osc_type);
            assert!((osc.nominal_frequency() - 10_000_000.0).abs() < TOLERANCE);
        }
    }

    #[test]
    fn test_noise_coefficients_positive() {
        let types = [
            OscillatorType::Tcxo,
            OscillatorType::Ocxo,
            OscillatorType::Rubidium,
            OscillatorType::Csac,
            OscillatorType::Cesium,
        ];
        for osc_type in &types {
            let coeffs = osc_type.default_noise_coefficients();
            assert!(coeffs.h0 >= 0.0, "h0 must be non-negative for {:?}", osc_type);
            assert!(coeffs.h_neg1 >= 0.0, "h_neg1 must be non-negative for {:?}", osc_type);
            assert!(coeffs.h_neg2 >= 0.0, "h_neg2 must be non-negative for {:?}", osc_type);
        }
    }

    #[test]
    fn test_ocxo_more_stable_than_tcxo() {
        let tcxo_coeffs = OscillatorType::Tcxo.default_noise_coefficients();
        let ocxo_coeffs = OscillatorType::Ocxo.default_noise_coefficients();
        // OCXO should have lower noise coefficients than TCXO
        assert!(ocxo_coeffs.h0 < tcxo_coeffs.h0);
    }

    #[test]
    fn test_cesium_no_aging() {
        let aging = OscillatorType::Cesium.default_aging_model();
        assert!((aging.linear_rate).abs() < TOLERANCE);
        assert!((aging.log_coefficient).abs() < TOLERANCE);
    }

    #[test]
    fn test_aging_offset_zero_at_start() {
        let osc = CsacReferenceOscillator::new(OscillatorType::Csac, 10_000_000.0);
        assert!((osc.aging_offset(0.0)).abs() < TOLERANCE);
    }

    #[test]
    fn test_aging_offset_increases_with_time() {
        let osc = CsacReferenceOscillator::new(OscillatorType::Tcxo, 10_000_000.0);
        let offset_1h = osc.aging_offset(3600.0);
        let offset_24h = osc.aging_offset(86400.0);
        assert!(offset_24h > offset_1h, "Aging should increase with time");
        assert!(offset_1h > 0.0, "TCXO aging should be positive");
    }

    #[test]
    fn test_temperature_offset_at_reference() {
        let osc = CsacReferenceOscillator::new(OscillatorType::Csac, 10_000_000.0);
        // At reference temperature, offset should be zero
        assert!((osc.temperature_offset()).abs() < TOLERANCE);
    }

    #[test]
    fn test_temperature_offset_away_from_reference() {
        let mut osc = CsacReferenceOscillator::new(OscillatorType::Tcxo, 10_000_000.0);
        osc.set_temperature(35.0); // 10 degC above reference
        let offset = osc.temperature_offset();
        // TCXO: 1e-6 * 10 = 1e-5
        assert!((offset - 1.0e-5).abs() < 1e-10);
    }

    #[test]
    fn test_sample_produces_unit_magnitude() {
        let mut osc = CsacReferenceOscillator::new(OscillatorType::Csac, 10_000_000.0)
            .with_seed(42);
        let (re, im) = osc.sample(0.0);
        let mag = (re * re + im * im).sqrt();
        // Should be very close to 1.0 (pure tone)
        assert!(
            (mag - 1.0).abs() < 1e-10,
            "Sample magnitude should be ~1.0, got {}",
            mag
        );
    }

    #[test]
    fn test_deterministic_with_seed() {
        let mut osc1 = CsacReferenceOscillator::new(OscillatorType::Csac, 10_000_000.0)
            .with_seed(12345);
        let mut osc2 = CsacReferenceOscillator::new(OscillatorType::Csac, 10_000_000.0)
            .with_seed(12345);

        for t in 0..50 {
            let s1 = osc1.sample(t as f64 * 0.001);
            let s2 = osc2.sample(t as f64 * 0.001);
            assert!(
                (s1.0 - s2.0).abs() < TOLERANCE && (s1.1 - s2.1).abs() < TOLERANCE,
                "Samples should be identical with same seed at t={}",
                t
            );
        }
    }

    #[test]
    fn test_different_seeds_produce_different_output() {
        let mut osc1 = CsacReferenceOscillator::new(OscillatorType::Csac, 10_000_000.0)
            .with_seed(111);
        let mut osc2 = CsacReferenceOscillator::new(OscillatorType::Csac, 10_000_000.0)
            .with_seed(222);

        // After a few samples, accumulated phase noise should differ
        let mut differ = false;
        for t in 0..100 {
            let s1 = osc1.sample(t as f64 * 0.1);
            let s2 = osc2.sample(t as f64 * 0.1);
            if (s1.0 - s2.0).abs() > 1e-6 || (s1.1 - s2.1).abs() > 1e-6 {
                differ = true;
                break;
            }
        }
        assert!(differ, "Different seeds should produce different samples");
    }

    #[test]
    fn test_generate_samples_length() {
        let mut osc = CsacReferenceOscillator::new(OscillatorType::Ocxo, 10_000_000.0);
        let samples = osc.generate_samples(0.0, 1000.0, 500);
        assert_eq!(samples.len(), 500);
    }

    #[test]
    fn test_allan_deviation_basic() {
        // Create white noise frequency data and check ADEV returns results
        let n = 1000;
        let tau0 = 1.0;
        // Simple synthetic frequency data with known drift
        let freq_data: Vec<f64> = (0..n)
            .map(|i| 10_000_000.0 + 0.001 * (i as f64 / n as f64))
            .collect();
        let adev = CsacReferenceOscillator::allan_deviation(&freq_data, tau0);
        assert!(!adev.is_empty(), "ADEV should return results");
        // Check that tau values increase
        for i in 1..adev.len() {
            assert!(adev[i].0 > adev[i - 1].0, "Tau values should increase");
        }
        // Check that ADEV values are positive
        for (tau, ad) in &adev {
            assert!(*tau > 0.0, "Tau must be positive");
            assert!(*ad >= 0.0, "ADEV must be non-negative");
        }
    }

    #[test]
    fn test_allan_deviation_empty_input() {
        let adev = CsacReferenceOscillator::allan_deviation(&[], 1.0);
        assert!(adev.is_empty());
        let adev2 = CsacReferenceOscillator::allan_deviation(&[1.0, 2.0], 1.0);
        assert!(adev2.is_empty(), "Need at least 3 samples");
    }

    #[test]
    fn test_overlapping_allan_deviation() {
        let n = 500;
        let tau0 = 1.0;
        let freq_data: Vec<f64> = (0..n)
            .map(|i| 10_000_000.0 + 0.0001 * ((i as f64 * 0.1).sin()))
            .collect();
        let adev = CsacReferenceOscillator::overlapping_allan_deviation(&freq_data, tau0);
        assert!(!adev.is_empty(), "Overlapping ADEV should return results");
        for (tau, ad) in &adev {
            assert!(*tau > 0.0);
            assert!(*ad >= 0.0);
        }
    }

    #[test]
    fn test_modified_allan_deviation() {
        let n = 500;
        let tau0 = 1.0;
        let freq_data: Vec<f64> = (0..n)
            .map(|i| 10_000_000.0 + 1e-6 * ((i as f64 * 0.05).sin()))
            .collect();
        let mdev = CsacReferenceOscillator::modified_allan_deviation(&freq_data, tau0);
        assert!(!mdev.is_empty(), "MDEV should return results");
        for (tau, md) in &mdev {
            assert!(*tau > 0.0);
            assert!(*md >= 0.0);
        }
    }

    #[test]
    fn test_time_interval_error() {
        // Phase data with known range
        let phase_data: Vec<f64> = (0..100)
            .map(|i| 1e-9 * (i as f64 * 0.1).sin())
            .collect();
        let tie = CsacReferenceOscillator::time_interval_error(&phase_data, 1.0);
        assert!(!tie.is_empty(), "TIE should return results");
        // TIE should be non-decreasing with window size (peak-to-peak over larger windows)
        for i in 1..tie.len() {
            assert!(
                tie[i].1 >= tie[i - 1].1 - 1e-15,
                "TIE should be non-decreasing: at i={}, {} < {}",
                i,
                tie[i].1,
                tie[i - 1].1
            );
        }
    }

    #[test]
    fn test_time_interval_error_empty() {
        let tie = CsacReferenceOscillator::time_interval_error(&[], 1.0);
        assert!(tie.is_empty());
        let tie2 = CsacReferenceOscillator::time_interval_error(&[0.0], 1.0);
        assert!(tie2.is_empty(), "Need at least 2 samples");
    }

    #[test]
    fn test_phase_noise_std_positive() {
        let osc = CsacReferenceOscillator::new(OscillatorType::Csac, 10_000_000.0);
        let sigma = osc.phase_noise_std(1.0);
        assert!(sigma > 0.0, "Phase noise std should be positive");
        // CSAC at 1s should produce modest phase noise
        assert!(sigma < 1e3, "Phase noise should be bounded");
    }

    #[test]
    fn test_theoretical_adev_curve() {
        let osc = CsacReferenceOscillator::new(OscillatorType::Csac, 10_000_000.0);
        let curve = osc.theoretical_adev(0.001, 10000.0, 20);
        assert_eq!(curve.len(), 20);
        // First tau should be near tau_min
        assert!((curve[0].0 - 0.001).abs() < 1e-6);
        // All ADEV values should be positive
        for (_, adev) in &curve {
            assert!(*adev > 0.0);
        }
    }

    #[test]
    fn test_allan_variance_model_consistency() {
        // The variance model should be consistent: OCXO < TCXO at typical tau
        let tcxo = CsacReferenceOscillator::new(OscillatorType::Tcxo, 10e6);
        let ocxo = CsacReferenceOscillator::new(OscillatorType::Ocxo, 10e6);
        let tau = 1.0;
        assert!(
            ocxo.allan_variance_model(tau) < tcxo.allan_variance_model(tau),
            "OCXO should have lower Allan variance than TCXO at tau=1s"
        );
    }

    #[test]
    fn test_reset_restores_initial_state() {
        let mut osc = CsacReferenceOscillator::new(OscillatorType::Csac, 10_000_000.0)
            .with_seed(999);
        // Generate some samples to accumulate phase error
        for i in 0..50 {
            osc.sample(i as f64 * 0.01);
        }
        // Reset
        osc.reset();
        // Now generate again with a fresh oscillator with the default seed
        let mut osc2 = CsacReferenceOscillator::new(OscillatorType::Csac, 10_000_000.0);
        let s1 = osc.sample(0.0);
        let s2 = osc2.sample(0.0);
        assert!(
            (s1.0 - s2.0).abs() < 1e-10 && (s1.1 - s2.1).abs() < 1e-10,
            "Reset should restore initial PRNG state"
        );
    }

    #[test]
    fn test_builder_pattern() {
        let custom_noise = NoiseCoefficients {
            h0: 1e-20,
            h_neg1: 1e-22,
            h_neg2: 1e-25,
        };
        let custom_aging = AgingModel {
            linear_rate: 1e-12,
            log_coefficient: 1e-14,
        };
        let custom_temp = TemperatureModel {
            coefficient: 5e-8,
            reference_temp: 20.0,
            current_temp: 30.0,
        };
        let osc = CsacReferenceOscillator::new(OscillatorType::Csac, 10_000_000.0)
            .with_noise_coefficients(custom_noise)
            .with_aging_model(custom_aging)
            .with_temperature_model(custom_temp)
            .with_seed(42)
            .with_start_time(100.0);

        assert!((osc.noise_coefficients().h0 - 1e-20).abs() < TOLERANCE);
        assert!((osc.aging_model().linear_rate - 1e-12).abs() < TOLERANCE);
        assert!((osc.temperature_model().coefficient - 5e-8).abs() < TOLERANCE);
    }

    #[test]
    fn test_negative_aging_dt_returns_zero() {
        let osc = CsacReferenceOscillator::new(OscillatorType::Tcxo, 10_000_000.0);
        assert!((osc.aging_offset(-100.0)).abs() < TOLERANCE);
    }
}
