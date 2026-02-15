//! Fiber Optic Gyroscope (FOG) Signal Processing
//!
//! Implements signal processing for interferometric fiber optic gyroscopes (IFOGs)
//! based on the Sagnac effect for inertial rotation sensing.
//!
//! ## Sagnac Effect
//!
//! When a fiber coil rotates, counter-propagating light beams experience a differential
//! phase shift proportional to the rotation rate:
//!
//! ```text
//! delta_phi = (8 * pi * A * N * Omega) / (lambda * c)
//!           = (4 * pi * L * R * Omega) / (lambda * c)
//! ```
//!
//! where A is the enclosed area per turn, N is the number of turns, L is the fiber
//! length, R is the coil radius, lambda is the wavelength, and c is the speed of light.
//!
//! ## Processing Modes
//!
//! - **Open-loop**: Direct extraction of Sagnac phase from modulated detector signal
//!   using synchronous demodulation at modulation harmonics
//! - **Closed-loop**: Serrodyne (digital phase ramp) feedback to null the Sagnac phase,
//!   with rotation rate derived from the ramp slope
//!
//! ## Noise Sources
//!
//! - Shot noise (photon statistics)
//! - Relative intensity noise (RIN) from source
//! - Thermal phase noise (Shupe effect)
//! - Rayleigh backscattering (coherent and incoherent)
//! - Magnetic sensitivity (Faraday effect)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::fiber_optic_gyroscope_processor::{FogConfig, SagnacEffect, FogProcessor};
//!
//! let config = FogConfig::navigation_grade();
//! let sagnac = SagnacEffect::new(&config);
//!
//! // Scale factor for this gyro
//! let sf = sagnac.scale_factor();
//! println!("Scale factor: {:.2} rad/(rad/s)", sf);
//!
//! // Phase shift for Earth rate at 45 degrees latitude
//! let omega_e = 7.2921e-5_f64;
//! let phi = sagnac.phase_shift(omega_e * (45.0_f64).to_radians().cos());
//! println!("Sagnac phase for Earth rate: {:.6} rad", phi);
//! ```

use std::f64::consts::PI;

/// Speed of light in vacuum (m/s).
const C: f64 = 299_792_458.0;

/// Planck's constant (J*s).
const H_PLANCK: f64 = 6.626_070_15e-34;

/// Earth rotation rate (rad/s).
const OMEGA_EARTH: f64 = 7.2921e-5;

// ---------------------------------------------------------------------------
// FogConfig
// ---------------------------------------------------------------------------

/// Configuration parameters for a fiber optic gyroscope.
#[derive(Debug, Clone)]
pub struct FogConfig {
    /// Total fiber length in the coil (m).
    pub fiber_length_m: f64,
    /// Coil diameter (m).
    pub coil_diameter_m: f64,
    /// Source wavelength (nm).
    pub wavelength_nm: f64,
    /// Number of fiber turns in the coil.
    pub num_turns: usize,
    /// Refractive index of the fiber core (typically ~1.46).
    pub fiber_index: f64,
}

impl FogConfig {
    /// Create a navigation-grade FOG configuration.
    ///
    /// Typical parameters: 1 km fiber, 15 cm diameter, 1550 nm, ~2122 turns.
    pub fn navigation_grade() -> Self {
        let diameter = 0.15;
        let length = 1000.0;
        let turns = (length / (PI * diameter)).round() as usize;
        Self {
            fiber_length_m: length,
            coil_diameter_m: diameter,
            wavelength_nm: 1550.0,
            num_turns: turns,
            fiber_index: 1.46,
        }
    }

    /// Create a tactical-grade FOG configuration.
    ///
    /// Typical: 200 m fiber, 8 cm diameter, 1310 nm.
    pub fn tactical_grade() -> Self {
        let diameter = 0.08;
        let length = 200.0;
        let turns = (length / (PI * diameter)).round() as usize;
        Self {
            fiber_length_m: length,
            coil_diameter_m: diameter,
            wavelength_nm: 1310.0,
            num_turns: turns,
            fiber_index: 1.46,
        }
    }

    /// Create a rate-grade (consumer) FOG configuration.
    pub fn rate_grade() -> Self {
        let diameter = 0.04;
        let length = 50.0;
        let turns = (length / (PI * diameter)).round() as usize;
        Self {
            fiber_length_m: length,
            coil_diameter_m: diameter,
            wavelength_nm: 850.0,
            num_turns: turns,
            fiber_index: 1.46,
        }
    }

    /// Coil radius in meters.
    pub fn coil_radius_m(&self) -> f64 {
        self.coil_diameter_m / 2.0
    }

    /// Wavelength in meters.
    pub fn wavelength_m(&self) -> f64 {
        self.wavelength_nm * 1.0e-9
    }

    /// Enclosed area per single turn (m^2).
    pub fn area_per_turn(&self) -> f64 {
        PI * self.coil_radius_m() * self.coil_radius_m()
    }

    /// Total enclosed area A*N (m^2).
    pub fn total_area(&self) -> f64 {
        self.area_per_turn() * self.num_turns as f64
    }

    /// Optical frequency (Hz).
    pub fn optical_frequency(&self) -> f64 {
        C / self.wavelength_m()
    }

    /// Fiber eigenfrequency f_e = c / (2 * n * L) (Hz).
    /// This is the proper modulation frequency for maximum sensitivity.
    pub fn eigenfrequency(&self) -> f64 {
        C / (2.0 * self.fiber_index * self.fiber_length_m)
    }
}

// ---------------------------------------------------------------------------
// SagnacEffect
// ---------------------------------------------------------------------------

/// Sagnac effect calculator for a given FOG configuration.
#[derive(Debug, Clone)]
pub struct SagnacEffect {
    config: FogConfig,
}

impl SagnacEffect {
    /// Create a new Sagnac effect calculator.
    pub fn new(config: &FogConfig) -> Self {
        Self {
            config: config.clone(),
        }
    }

    /// Compute the Sagnac phase shift for a given rotation rate (rad/s).
    ///
    /// delta_phi = (8 * pi * A * N * Omega) / (lambda * c)
    pub fn phase_shift(&self, omega_rad_per_s: f64) -> f64 {
        let a = self.config.area_per_turn();
        let n = self.config.num_turns as f64;
        let lambda = self.config.wavelength_m();
        (8.0 * PI * a * n * omega_rad_per_s) / (lambda * C)
    }

    /// Compute the Sagnac phase shift using the alternate L*R formulation.
    ///
    /// delta_phi = (4 * pi * L * R * Omega) / (lambda * c)
    pub fn phase_shift_lr(&self, omega_rad_per_s: f64) -> f64 {
        let l = self.config.fiber_length_m;
        let r = self.config.coil_radius_m();
        let lambda = self.config.wavelength_m();
        (4.0 * PI * l * r * omega_rad_per_s) / (lambda * C)
    }

    /// Scale factor: SF = delta_phi / Omega = 8*pi*A*N / (lambda*c) [rad/(rad/s)].
    ///
    /// Equivalent to 2*pi*L*D / (lambda*c) when L = N*pi*D exactly.
    /// This formulation uses A*N for consistency with `phase_shift()`.
    pub fn scale_factor(&self) -> f64 {
        let a = self.config.area_per_turn();
        let n = self.config.num_turns as f64;
        let lambda = self.config.wavelength_m();
        (8.0 * PI * a * n) / (lambda * C)
    }

    /// Compute rotation rate from a measured Sagnac phase shift (rad/s).
    pub fn rotation_rate_from_phase(&self, delta_phi: f64) -> f64 {
        delta_phi / self.scale_factor()
    }

    /// Minimum detectable rotation rate limited by shot noise.
    ///
    /// delta_Omega_shot = (lambda * c) / (4 * A * N * sqrt(eta * P * tau / (h * f)))
    ///
    /// * `optical_power_w` - Detected optical power (W)
    /// * `detector_efficiency` - Quantum efficiency (0..1)
    /// * `integration_time_s` - Integration time (s)
    pub fn shot_noise_limit(
        &self,
        optical_power_w: f64,
        detector_efficiency: f64,
        integration_time_s: f64,
    ) -> f64 {
        let a = self.config.area_per_turn();
        let n = self.config.num_turns as f64;
        let lambda = self.config.wavelength_m();
        let f_opt = self.config.optical_frequency();

        let photon_rate = detector_efficiency * optical_power_w * integration_time_s
            / (H_PLANCK * f_opt);
        let denom = 4.0 * a * n * photon_rate.sqrt();
        (lambda * C) / denom
    }

    /// Compute the expected Earth rate component for a horizontal FOG at given latitude.
    ///
    /// Omega_horizontal = Omega_E * cos(latitude)
    pub fn earth_rate_horizontal(&self, latitude_deg: f64) -> f64 {
        OMEGA_EARTH * latitude_deg.to_radians().cos()
    }

    /// Compute the expected Sagnac phase from Earth rotation at a given latitude.
    pub fn earth_rate_phase(&self, latitude_deg: f64) -> f64 {
        self.phase_shift(self.earth_rate_horizontal(latitude_deg))
    }

    /// Compute latitude from measured horizontal Earth rate component.
    /// Returns None if the measured rate exceeds Earth rate magnitude.
    pub fn latitude_from_earth_rate(&self, measured_omega: f64) -> Option<f64> {
        let ratio = measured_omega / OMEGA_EARTH;
        if ratio.abs() > 1.0 {
            return None;
        }
        Some(ratio.acos().to_degrees())
    }
}

// ---------------------------------------------------------------------------
// FogProcessor
// ---------------------------------------------------------------------------

/// Fiber optic gyroscope signal processor.
///
/// Performs phase modulation, demodulation (open-loop and closed-loop),
/// and rotation rate extraction.
#[derive(Debug, Clone)]
pub struct FogProcessor {
    config: FogConfig,
    /// Phase modulation frequency (Hz). Typically the fiber eigenfrequency.
    pub modulation_frequency_hz: f64,
    /// Modulation depth (rad). Typically pi/2 for max sensitivity.
    pub modulation_depth: f64,
    /// Sample rate of the digitized detector signal (Hz).
    pub sample_rate_hz: f64,
    /// Integration time for rate estimation (s).
    pub integration_time_s: f64,
    sagnac: SagnacEffect,
}

impl FogProcessor {
    /// Create a new FOG processor with default modulation parameters.
    pub fn new(config: &FogConfig) -> Self {
        let f_e = config.eigenfrequency();
        let sagnac = SagnacEffect::new(config);
        Self {
            config: config.clone(),
            modulation_frequency_hz: f_e,
            modulation_depth: PI / 2.0, // optimal
            sample_rate_hz: 10.0 * f_e, // 10x oversampling
            integration_time_s: 0.01,
            sagnac,
        }
    }

    /// Create with explicit parameters.
    pub fn with_params(
        config: &FogConfig,
        modulation_frequency_hz: f64,
        modulation_depth: f64,
        sample_rate_hz: f64,
        integration_time_s: f64,
    ) -> Self {
        let sagnac = SagnacEffect::new(config);
        Self {
            config: config.clone(),
            modulation_frequency_hz,
            modulation_depth,
            sample_rate_hz,
            integration_time_s,
            sagnac,
        }
    }

    /// Reference to the Sagnac effect calculator.
    pub fn sagnac(&self) -> &SagnacEffect {
        &self.sagnac
    }

    /// Generate a sinusoidal phase modulation waveform.
    ///
    /// phi_m(t) = modulation_depth * sin(2*pi*f_m*t)
    pub fn generate_modulation_sinusoidal(&self, num_samples: usize) -> Vec<f64> {
        let dt = 1.0 / self.sample_rate_hz;
        (0..num_samples)
            .map(|i| {
                let t = i as f64 * dt;
                self.modulation_depth * (2.0 * PI * self.modulation_frequency_hz * t).sin()
            })
            .collect()
    }

    /// Generate a square wave phase modulation waveform.
    ///
    /// Alternates between +modulation_depth and -modulation_depth at f_m.
    pub fn generate_modulation_square(&self, num_samples: usize) -> Vec<f64> {
        let dt = 1.0 / self.sample_rate_hz;
        let period = 1.0 / self.modulation_frequency_hz;
        (0..num_samples)
            .map(|i| {
                let t = i as f64 * dt;
                let phase_in_cycle = (t % period) / period;
                if phase_in_cycle < 0.5 {
                    self.modulation_depth
                } else {
                    -self.modulation_depth
                }
            })
            .collect()
    }

    /// Simulate the detector output for a given Sagnac phase and modulation.
    ///
    /// V[n] = V0 * (1 + cos(phi_s + phi_m[n] - phi_m_delayed[n]))
    ///
    /// where phi_m_delayed is phi_m delayed by the transit time tau = n*L/c.
    ///
    /// * `sagnac_phase` - The Sagnac phase shift (rad)
    /// * `modulation` - Phase modulation waveform
    /// * `v0` - Detector voltage scale factor
    pub fn simulate_detector_output(
        &self,
        sagnac_phase: f64,
        modulation: &[f64],
        v0: f64,
    ) -> Vec<f64> {
        let tau = self.config.fiber_index * self.config.fiber_length_m / C;
        let delay_samples = (tau * self.sample_rate_hz).round() as usize;

        modulation
            .iter()
            .enumerate()
            .map(|(i, &phi_m)| {
                let phi_m_delayed = if i >= delay_samples {
                    modulation[i - delay_samples]
                } else {
                    0.0
                };
                // Interference: proportional to cos of total phase difference
                v0 * (1.0 + (sagnac_phase + phi_m - phi_m_delayed).cos())
            })
            .collect()
    }

    /// Open-loop demodulation: extract Sagnac phase using synchronous demodulation.
    ///
    /// Computes first and second harmonic amplitudes:
    /// - H1 proportional to J1(phi_m_eff) * sin(phi_s)
    /// - H2 proportional to J2(phi_m_eff) * cos(phi_s)
    ///
    /// Returns (sin_component, cos_component, estimated_phase_rad).
    pub fn demodulate_open_loop(
        &self,
        detector_signal: &[f64],
    ) -> (f64, f64, f64) {
        let n = detector_signal.len();
        if n == 0 {
            return (0.0, 0.0, 0.0);
        }

        let dt = 1.0 / self.sample_rate_hz;
        let f_m = self.modulation_frequency_hz;

        // Synchronous demodulation at 1st harmonic (f_m)
        let mut h1_sin = 0.0;
        let mut h1_cos = 0.0;
        // Synchronous demodulation at 2nd harmonic (2*f_m)
        let mut h2_sin = 0.0;
        let mut h2_cos = 0.0;

        for i in 0..n {
            let t = i as f64 * dt;
            let ref1_sin = (2.0 * PI * f_m * t).sin();
            let ref1_cos = (2.0 * PI * f_m * t).cos();
            let ref2_sin = (2.0 * PI * 2.0 * f_m * t).sin();
            let ref2_cos = (2.0 * PI * 2.0 * f_m * t).cos();

            h1_sin += detector_signal[i] * ref1_sin;
            h1_cos += detector_signal[i] * ref1_cos;
            h2_sin += detector_signal[i] * ref2_sin;
            h2_cos += detector_signal[i] * ref2_cos;
        }

        let n_f = n as f64;
        h1_sin /= n_f;
        h1_cos /= n_f;
        h2_sin /= n_f;
        h2_cos /= n_f;

        // Amplitude of first and second harmonics
        let h1_amp = (h1_sin * h1_sin + h1_cos * h1_cos).sqrt();
        let h2_amp = (h2_sin * h2_sin + h2_cos * h2_cos).sqrt();

        // The Sagnac phase can be estimated from the ratio:
        // tan(phi_s) ~ (H1 * J2(phi_eff)) / (H2 * J1(phi_eff))
        // For small phi_s: phi_s ~ H1 / (J1 * V0) (approximately)
        // We use atan2 for robustness:
        let estimated_phase = h1_amp.atan2(h2_amp);

        (h1_amp, h2_amp, estimated_phase)
    }

    /// Closed-loop (serrodyne) processing.
    ///
    /// Applies a digital phase ramp to null the Sagnac phase. The ramp accumulates
    /// phase, resetting at 2*pi boundaries. The rotation rate is derived from
    /// the ramp slope.
    ///
    /// Returns a `SerrodyneResult` with the ramp history and estimated rotation rates.
    pub fn process_closed_loop(
        &self,
        detector_signal: &[f64],
        loop_gain: f64,
    ) -> SerrodyneResult {
        let n = detector_signal.len();
        let dt = 1.0 / self.sample_rate_hz;

        let mut ramp_phase = 0.0_f64;
        let mut ramp_history = Vec::with_capacity(n);
        let mut phase_increments = Vec::with_capacity(n);

        for i in 0..n {
            // Simple error signal: deviation from the bias point
            // In a real system this would come from demodulation
            let t = i as f64 * dt;
            let ref_sin = (2.0 * PI * self.modulation_frequency_hz * t).sin();
            let error = detector_signal[i] * ref_sin;

            // Feedback: adjust ramp to null the error
            let increment = loop_gain * error * dt;
            ramp_phase += increment;

            // 2*pi reset (serrodyne)
            while ramp_phase > 2.0 * PI {
                ramp_phase -= 2.0 * PI;
            }
            while ramp_phase < -2.0 * PI {
                ramp_phase += 2.0 * PI;
            }

            ramp_history.push(ramp_phase);
            phase_increments.push(increment);
        }

        // Estimate rotation rate from average phase increment rate
        let total_phase: f64 = phase_increments.iter().sum();
        let total_time = n as f64 * dt;
        let avg_ramp_slope = total_phase / total_time;

        // Omega = ramp_slope * lambda * c / (4 * pi * A * N)
        // But ramp_slope IS the Sagnac phase rate, so Omega = ramp_slope / SF
        let sf = self.sagnac.scale_factor();
        let estimated_omega = avg_ramp_slope / sf;

        SerrodyneResult {
            ramp_history,
            phase_increments,
            estimated_rotation_rate: estimated_omega,
            avg_ramp_slope,
        }
    }

    /// Compute rotation rate from open-loop phase estimate.
    pub fn rotation_rate_from_phase(&self, phase_rad: f64) -> f64 {
        self.sagnac.rotation_rate_from_phase(phase_rad)
    }
}

/// Result of closed-loop serrodyne processing.
#[derive(Debug, Clone)]
pub struct SerrodyneResult {
    /// Phase ramp history at each sample.
    pub ramp_history: Vec<f64>,
    /// Phase increment at each sample.
    pub phase_increments: Vec<f64>,
    /// Estimated rotation rate from ramp slope (rad/s).
    pub estimated_rotation_rate: f64,
    /// Average ramp slope (rad/s).
    pub avg_ramp_slope: f64,
}

// ---------------------------------------------------------------------------
// Noise models
// ---------------------------------------------------------------------------

/// FOG noise source models.
#[derive(Debug, Clone)]
pub struct FogNoiseModel {
    config: FogConfig,
}

impl FogNoiseModel {
    /// Create a noise model for the given configuration.
    pub fn new(config: &FogConfig) -> Self {
        Self {
            config: config.clone(),
        }
    }

    /// Shot noise limited rotation rate (rad/s).
    ///
    /// delta_Omega_shot = (lambda*c) / (4*A*N*sqrt(eta*P*tau/(h*f)))
    pub fn shot_noise_rate(
        &self,
        optical_power_w: f64,
        detector_efficiency: f64,
        integration_time_s: f64,
    ) -> f64 {
        let sagnac = SagnacEffect::new(&self.config);
        sagnac.shot_noise_limit(optical_power_w, detector_efficiency, integration_time_s)
    }

    /// Relative Intensity Noise (RIN) contribution to phase noise (rad).
    ///
    /// delta_phi_rin = sqrt(RIN_dB_per_Hz_linear * bandwidth)
    ///
    /// * `rin_db_per_hz` - Source RIN in dB/Hz (typically -110 to -140 dB/Hz)
    /// * `detection_bandwidth_hz` - Detection bandwidth
    pub fn rin_phase_noise(&self, rin_db_per_hz: f64, detection_bandwidth_hz: f64) -> f64 {
        let rin_linear = 10.0_f64.powf(rin_db_per_hz / 10.0);
        (rin_linear * detection_bandwidth_hz).sqrt()
    }

    /// Thermal (Shupe) phase noise (rad).
    ///
    /// Approximate model: delta_phi_shupe = alpha_thermal * dT * L_eff
    ///
    /// * `thermal_coeff` - Thermal phase coefficient (rad/(m*K)), typically ~7e-6
    /// * `temperature_gradient_k` - Temperature gradient across coil (K)
    /// * `effective_length_m` - Effective thermal length (fraction of total)
    pub fn shupe_phase_noise(
        &self,
        thermal_coeff: f64,
        temperature_gradient_k: f64,
        effective_length_m: f64,
    ) -> f64 {
        thermal_coeff * temperature_gradient_k * effective_length_m
    }

    /// Rayleigh backscattering noise power (relative).
    ///
    /// * `scatter_coeff` - Rayleigh scattering coefficient (1/m), typically ~4.5e-5 for 1550nm
    /// * `capture_fraction` - Fraction recaptured by fiber, typically ~1e-3
    pub fn rayleigh_backscatter_power(
        &self,
        scatter_coeff: f64,
        capture_fraction: f64,
    ) -> f64 {
        scatter_coeff * capture_fraction * self.config.fiber_length_m
    }

    /// Faraday effect phase shift (rad).
    ///
    /// delta_phi = V * N * B * L_per_turn
    ///
    /// * `verdet_constant` - Verdet constant (rad/(T*m)), ~1.0 for silica at 1550nm
    /// * `magnetic_field_t` - Magnetic field strength (Tesla)
    pub fn faraday_phase_shift(
        &self,
        verdet_constant: f64,
        magnetic_field_t: f64,
    ) -> f64 {
        verdet_constant * self.config.num_turns as f64 * magnetic_field_t
            * PI * self.config.coil_diameter_m
    }

    /// Total noise floor estimate (rad/s) combining major sources.
    pub fn total_noise_floor(
        &self,
        optical_power_w: f64,
        detector_efficiency: f64,
        integration_time_s: f64,
        rin_db_per_hz: f64,
        detection_bandwidth_hz: f64,
    ) -> f64 {
        let shot = self.shot_noise_rate(optical_power_w, detector_efficiency, integration_time_s);
        let rin_phase = self.rin_phase_noise(rin_db_per_hz, detection_bandwidth_hz);
        let sf = SagnacEffect::new(&self.config).scale_factor();
        let rin_rate = rin_phase / sf;

        // RSS combination
        (shot * shot + rin_rate * rin_rate).sqrt()
    }
}

// ---------------------------------------------------------------------------
// Allan Variance
// ---------------------------------------------------------------------------

/// Allan variance computation for rotation rate stability analysis.
#[derive(Debug, Clone)]
pub struct AllanVariance;

impl AllanVariance {
    /// Compute the overlapping Allan variance from a time series of rotation rate samples.
    ///
    /// Returns a vector of (tau, allan_variance) pairs for each cluster size.
    ///
    /// * `data` - Time series of rotation rate measurements (rad/s)
    /// * `sample_period_s` - Time between samples (s)
    /// * `max_clusters` - Maximum number of tau values to compute (logarithmically spaced)
    pub fn compute(
        data: &[f64],
        sample_period_s: f64,
        max_clusters: usize,
    ) -> Vec<(f64, f64)> {
        let n = data.len();
        if n < 3 {
            return vec![];
        }

        // Generate logarithmically spaced cluster sizes
        let max_m = n / 2;
        let num_points = max_clusters.min(max_m);
        if num_points == 0 {
            return vec![];
        }

        let log_min = 0.0_f64; // m=1
        let log_max = (max_m as f64).ln();
        let mut cluster_sizes = Vec::with_capacity(num_points);

        for i in 0..num_points {
            let log_m = log_min + (log_max - log_min) * i as f64 / (num_points.max(1) - 1).max(1) as f64;
            let m = log_m.exp().round() as usize;
            if m >= 1 && m <= max_m && !cluster_sizes.contains(&m) {
                cluster_sizes.push(m);
            }
        }
        cluster_sizes.sort();
        cluster_sizes.dedup();

        let mut results = Vec::with_capacity(cluster_sizes.len());

        for &m in &cluster_sizes {
            let tau = m as f64 * sample_period_s;

            // Compute overlapping Allan variance
            // AVAR(tau) = 1/(2*tau^2*(N-2m)) * sum((x[i+2m] - 2*x[i+m] + x[i])^2)
            // where x[i] are the cumulative angle (integrated rate) samples
            //
            // Equivalently for rate data:
            // Compute cluster averages, then AVAR from their differences

            // Cluster averages
            let num_clusters = n - m + 1;
            if num_clusters < 2 {
                continue;
            }

            let mut averages = Vec::with_capacity(num_clusters);
            let mut running_sum = 0.0;
            for j in 0..m {
                running_sum += data[j];
            }
            averages.push(running_sum / m as f64);

            for j in 1..num_clusters {
                running_sum += data[j + m - 1];
                running_sum -= data[j - 1];
                averages.push(running_sum / m as f64);
            }

            // Allan variance: AVAR = 1/(2*(K-1)) * sum((avg[i+1] - avg[i])^2)
            // where K is the number of cluster averages
            let k = averages.len();
            if k < 2 {
                continue;
            }
            let mut sum_sq = 0.0;
            for j in 0..k - 1 {
                let diff = averages[j + 1] - averages[j];
                sum_sq += diff * diff;
            }
            let avar = sum_sq / (2.0 * (k - 1) as f64);

            results.push((tau, avar));
        }

        results
    }

    /// Compute the Allan deviation (square root of Allan variance).
    pub fn compute_adev(
        data: &[f64],
        sample_period_s: f64,
        max_clusters: usize,
    ) -> Vec<(f64, f64)> {
        Self::compute(data, sample_period_s, max_clusters)
            .into_iter()
            .map(|(tau, avar)| (tau, avar.sqrt()))
            .collect()
    }

    /// Estimate Angle Random Walk (ARW) from Allan deviation data.
    ///
    /// ARW is found at the point where tau = 1 s on the ADEV plot (slope -1/2 region).
    /// Returns ARW in rad/sqrt(s).
    pub fn estimate_arw(adev_data: &[(f64, f64)]) -> Option<f64> {
        if adev_data.is_empty() {
            return None;
        }

        // Find the adev value closest to tau = 1.0
        // ARW = adev(tau=1) * sqrt(1) = adev(tau=1)
        let mut best_idx = 0;
        let mut best_dist = f64::MAX;
        for (i, &(tau, _)) in adev_data.iter().enumerate() {
            let dist = (tau - 1.0).abs();
            if dist < best_dist {
                best_dist = dist;
                best_idx = i;
            }
        }

        // Interpolate if we have bracketing points
        let (tau, adev) = adev_data[best_idx];
        // ARW = adev * sqrt(tau) at the -1/2 slope region
        Some(adev * tau.sqrt())
    }

    /// Estimate bias instability from Allan deviation data.
    ///
    /// Bias instability is the minimum of the ADEV curve, occurring at the
    /// transition from -1/2 to 0 slope (flat region).
    pub fn estimate_bias_instability(adev_data: &[(f64, f64)]) -> Option<(f64, f64)> {
        if adev_data.is_empty() {
            return None;
        }

        let mut min_adev = f64::MAX;
        let mut min_tau = 0.0;

        for &(tau, adev) in adev_data {
            if adev < min_adev {
                min_adev = adev;
                min_tau = tau;
            }
        }

        Some((min_tau, min_adev))
    }

    /// Estimate Rate Random Walk (RRW) from Allan deviation data.
    ///
    /// RRW is identified by the +1/2 slope region at long tau.
    /// Returns RRW in rad/s/sqrt(s) estimated from the longest tau.
    pub fn estimate_rrw(adev_data: &[(f64, f64)]) -> Option<f64> {
        if adev_data.len() < 2 {
            return None;
        }

        // Use the last point, assuming it's in the +1/2 slope region
        let (tau, adev) = adev_data[adev_data.len() - 1];
        // RRW = adev / sqrt(tau/3)
        Some(adev / (tau / 3.0).sqrt())
    }
}

// ---------------------------------------------------------------------------
// Scale factor calibration
// ---------------------------------------------------------------------------

/// Scale factor calibration and temperature compensation.
#[derive(Debug, Clone)]
pub struct ScaleFactorCalibration {
    /// Reference scale factor at reference temperature.
    pub sf_reference: f64,
    /// Reference temperature (degrees C).
    pub reference_temperature_c: f64,
    /// Temperature coefficient (1/K).
    pub temp_coefficient: f64,
    /// Measured calibration points: (input_rate, measured_output).
    pub calibration_points: Vec<(f64, f64)>,
}

impl ScaleFactorCalibration {
    /// Create a new calibration from a known scale factor.
    pub fn new(sf_reference: f64, reference_temperature_c: f64) -> Self {
        Self {
            sf_reference,
            reference_temperature_c,
            temp_coefficient: 7.0e-6, // typical for silica fiber
            calibration_points: Vec::new(),
        }
    }

    /// Add a calibration point: (known_input_rate_rad_s, measured_phase_rad).
    pub fn add_point(&mut self, input_rate: f64, measured_output: f64) {
        self.calibration_points.push((input_rate, measured_output));
    }

    /// Compute scale factor from calibration points using least-squares fit.
    ///
    /// Returns (estimated_sf, sf_error, nonlinearity_ppm).
    pub fn compute_scale_factor(&self) -> Option<(f64, f64, f64)> {
        let n = self.calibration_points.len();
        if n < 2 {
            return None;
        }

        // Linear least squares: output = SF * input + offset
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xy = 0.0;
        let mut sum_xx = 0.0;

        for &(x, y) in &self.calibration_points {
            sum_x += x;
            sum_y += y;
            sum_xy += x * y;
            sum_xx += x * x;
        }

        let n_f = n as f64;
        let denom = n_f * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return None;
        }

        let sf_est = (n_f * sum_xy - sum_x * sum_y) / denom;
        let _offset = (sum_y - sf_est * sum_x) / n_f;

        // Scale factor error relative to reference
        let sf_error = (sf_est - self.sf_reference) / self.sf_reference;

        // Nonlinearity: max deviation from linear fit (in ppm)
        let mut max_dev = 0.0_f64;
        let mut max_output = 0.0_f64;
        for &(x, y) in &self.calibration_points {
            let predicted = sf_est * x + _offset;
            let dev = (y - predicted).abs();
            max_dev = max_dev.max(dev);
            max_output = max_output.max(y.abs());
        }

        let nonlinearity_ppm = if max_output > 0.0 {
            max_dev / max_output * 1.0e6
        } else {
            0.0
        };

        Some((sf_est, sf_error, nonlinearity_ppm))
    }

    /// Temperature-compensated scale factor.
    ///
    /// SF(T) = SF0 * (1 + alpha * (T - T0))
    pub fn compensated_scale_factor(&self, temperature_c: f64) -> f64 {
        self.sf_reference
            * (1.0 + self.temp_coefficient * (temperature_c - self.reference_temperature_c))
    }

    /// Compute scale factor stability (ppm) from a series of SF measurements.
    pub fn scale_factor_stability(sf_measurements: &[f64]) -> Option<f64> {
        if sf_measurements.len() < 2 {
            return None;
        }

        let mean: f64 = sf_measurements.iter().sum::<f64>() / sf_measurements.len() as f64;
        let variance: f64 = sf_measurements.iter().map(|&x| (x - mean) * (x - mean)).sum::<f64>()
            / (sf_measurements.len() - 1) as f64;
        let std_dev = variance.sqrt();

        Some(std_dev / mean * 1.0e6) // ppm
    }
}

// ---------------------------------------------------------------------------
// Earth rate sensing / North finding
// ---------------------------------------------------------------------------

/// Earth rate sensor for north-finding applications.
#[derive(Debug, Clone)]
pub struct EarthRateSensor {
    sagnac: SagnacEffect,
}

impl EarthRateSensor {
    /// Create a new Earth rate sensor.
    pub fn new(config: &FogConfig) -> Self {
        Self {
            sagnac: SagnacEffect::new(config),
        }
    }

    /// Expected horizontal Earth rate at given latitude (rad/s).
    pub fn expected_horizontal_rate(&self, latitude_deg: f64) -> f64 {
        OMEGA_EARTH * latitude_deg.to_radians().cos()
    }

    /// Expected vertical Earth rate at given latitude (rad/s).
    pub fn expected_vertical_rate(&self, latitude_deg: f64) -> f64 {
        OMEGA_EARTH * latitude_deg.to_radians().sin()
    }

    /// Determine heading from two orthogonal horizontal FOG measurements.
    ///
    /// * `omega_x` - Measured rate along platform X axis (rad/s)
    /// * `omega_y` - Measured rate along platform Y axis (rad/s)
    /// * `latitude_deg` - Known latitude (degrees)
    ///
    /// Returns heading in degrees (0 = North, 90 = East).
    pub fn compute_heading(
        &self,
        omega_x: f64,
        omega_y: f64,
        latitude_deg: f64,
    ) -> f64 {
        let omega_h = self.expected_horizontal_rate(latitude_deg);
        if omega_h.abs() < 1e-12 {
            return 0.0; // undefined at poles
        }

        // The horizontal Earth rate projects onto the platform axes as:
        // omega_x = omega_h * cos(heading)
        // omega_y = omega_h * sin(heading)
        let heading_rad = omega_y.atan2(omega_x);
        let mut heading_deg = heading_rad.to_degrees();
        if heading_deg < 0.0 {
            heading_deg += 360.0;
        }
        heading_deg
    }

    /// Estimate latitude from a vertical-axis FOG measurement.
    ///
    /// * `omega_vertical` - Measured vertical component of Earth rate (rad/s)
    ///
    /// Returns latitude in degrees, or None if measurement exceeds Earth rate.
    pub fn estimate_latitude(&self, omega_vertical: f64) -> Option<f64> {
        let ratio = omega_vertical / OMEGA_EARTH;
        if ratio.abs() > 1.0 {
            return None;
        }
        Some(ratio.asin().to_degrees())
    }

    /// Compute the Sagnac phase from Earth rate for sensitivity assessment.
    pub fn earth_rate_phase(&self, latitude_deg: f64) -> f64 {
        self.sagnac.earth_rate_phase(latitude_deg)
    }

    /// Check if the FOG can detect Earth rate given its noise floor.
    ///
    /// * `noise_floor_rad_s` - Minimum detectable rotation rate (rad/s)
    /// * `latitude_deg` - Operating latitude (degrees)
    ///
    /// Returns the SNR (Earth rate / noise floor).
    pub fn earth_rate_snr(&self, noise_floor_rad_s: f64, latitude_deg: f64) -> f64 {
        let omega_h = self.expected_horizontal_rate(latitude_deg).abs();
        omega_h / noise_floor_rad_s
    }
}

// ---------------------------------------------------------------------------
// Bessel function approximations (needed for modulation analysis)
// ---------------------------------------------------------------------------

/// Bessel function of the first kind, order 0 (J0).
/// Uses polynomial approximation for |x| <= 8 and asymptotic for |x| > 8.
fn bessel_j0(x: f64) -> f64 {
    let ax = x.abs();
    if ax < 8.0 {
        let y = x * x;
        let num = 57568490574.0 + y * (-13362590354.0 + y * (651619640.7
            + y * (-11214424.18 + y * (77392.33017 + y * (-184.9052456)))));
        let den = 57568490411.0 + y * (1029532985.0 + y * (9494680.718
            + y * (59272.64853 + y * (267.8532712 + y * 1.0))));
        num / den
    } else {
        let z = 8.0 / ax;
        let y = z * z;
        let xx = ax - 0.785398164;
        let p0 = 1.0 + y * (-0.1098628627e-2 + y * (0.2734510407e-4
            + y * (-0.2073370639e-5 + y * 0.2093887211e-6)));
        let q0 = -0.1562499995e-1 + y * (0.1430488765e-3 + y * (-0.6911147651e-5
            + y * (0.7621095161e-6 + y * (-0.934935152e-7))));
        (0.636619772 / ax).sqrt() * (xx.cos() * p0 - z * xx.sin() * q0)
    }
}

/// Bessel function of the first kind, order 1 (J1).
fn bessel_j1(x: f64) -> f64 {
    let ax = x.abs();
    if ax < 8.0 {
        let y = x * x;
        let num = x * (72362614232.0 + y * (-7895059235.0 + y * (242396853.1
            + y * (-2972611.439 + y * (15704.48260 + y * (-30.16036606))))));
        let den = 144725228442.0 + y * (2300535178.0 + y * (18583304.74
            + y * (99447.43394 + y * (376.9991397 + y * 1.0))));
        num / den
    } else {
        let z = 8.0 / ax;
        let y = z * z;
        let xx = ax - 2.356194491;
        let p1 = 1.0 + y * (0.183105e-2 + y * (-0.3516396496e-4
            + y * (0.2457520174e-5 + y * (-0.240337019e-6))));
        let q1 = 0.04687499995 + y * (-0.2002690873e-3 + y * (0.8449199096e-5
            + y * (-0.88228987e-6 + y * 0.105787412e-6)));
        let ans = (0.636619772 / ax).sqrt() * (xx.cos() * p1 - z * xx.sin() * q1);
        if x < 0.0 { -ans } else { ans }
    }
}

/// Bessel function of the first kind, order n (Jn) via upward recurrence.
fn bessel_jn(n: i32, x: f64) -> f64 {
    match n {
        0 => bessel_j0(x),
        1 => bessel_j1(x),
        _ if n < 0 => {
            let val = bessel_jn(-n, x);
            if n % 2 == 0 { val } else { -val }
        }
        _ => {
            // Miller's downward recurrence for accuracy
            let n_u = n as usize;
            let ax = x.abs();
            if ax < 1e-30 {
                return 0.0;
            }

            // Start from a sufficiently high order
            let m = 2 * ((n_u + (40.0 * ax.sqrt()) as usize) / 2);
            let mut j_prev = 0.0_f64;
            let mut j_curr = 1.0e-30_f64;
            let mut result = 0.0;
            let mut sum = 0.0;

            for k in (0..=m).rev() {
                let j_next = 2.0 * (k + 1) as f64 / ax * j_curr - j_prev;
                j_prev = j_curr;
                j_curr = j_next;

                if k == n_u {
                    result = j_curr;
                }

                if k % 2 == 0 {
                    sum += j_curr;
                }
            }

            // Normalize using J0 relation: J0(x) + 2*sum(J2k(x)) = 1
            // Actually: 2*sum - j_curr = 1 (j_curr is now J0)
            let normalization = 2.0 * sum - j_curr;
            let val = result / normalization;

            if x < 0.0 && n % 2 != 0 {
                -val
            } else {
                val
            }
        }
    }
}

/// Effective modulation depth analysis.
///
/// For sinusoidal modulation at the eigenfrequency, the effective phase
/// modulation depth depends on the Bessel functions of the modulation depth.
pub fn modulation_analysis(modulation_depth: f64) -> ModulationAnalysis {
    let j0 = bessel_j0(modulation_depth);
    let j1 = bessel_j1(modulation_depth);
    let j2 = bessel_jn(2, modulation_depth);

    // Optimal modulation depth maximizes J1(phi_m)
    // This occurs near phi_m ≈ 1.84 (first maximum of J1)
    let optimal_depth = 1.8412;

    ModulationAnalysis {
        modulation_depth,
        j0,
        j1,
        j2,
        first_harmonic_sensitivity: j1.abs(),
        second_harmonic_sensitivity: j2.abs(),
        optimal_depth,
    }
}

/// Result of modulation depth analysis.
#[derive(Debug, Clone)]
pub struct ModulationAnalysis {
    /// Applied modulation depth (rad).
    pub modulation_depth: f64,
    /// J0(phi_m) - DC component.
    pub j0: f64,
    /// J1(phi_m) - First harmonic sensitivity.
    pub j1: f64,
    /// J2(phi_m) - Second harmonic sensitivity.
    pub j2: f64,
    /// |J1(phi_m)| - first harmonic magnitude.
    pub first_harmonic_sensitivity: f64,
    /// |J2(phi_m)| - second harmonic magnitude.
    pub second_harmonic_sensitivity: f64,
    /// Optimal modulation depth for maximum J1 (~1.8412 rad).
    pub optimal_depth: f64,
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;
    const EPSILON_LOOSE: f64 = 1e-3;

    // --- FogConfig tests ---

    #[test]
    fn test_navigation_grade_config() {
        let cfg = FogConfig::navigation_grade();
        assert!((cfg.fiber_length_m - 1000.0).abs() < EPSILON);
        assert!((cfg.coil_diameter_m - 0.15).abs() < EPSILON);
        assert!((cfg.wavelength_nm - 1550.0).abs() < EPSILON);
        assert!(cfg.num_turns > 2000);
        assert!((cfg.fiber_index - 1.46).abs() < EPSILON);
    }

    #[test]
    fn test_tactical_grade_config() {
        let cfg = FogConfig::tactical_grade();
        assert!((cfg.fiber_length_m - 200.0).abs() < EPSILON);
        assert!((cfg.coil_diameter_m - 0.08).abs() < EPSILON);
        assert!((cfg.wavelength_nm - 1310.0).abs() < EPSILON);
        assert!(cfg.num_turns > 700);
    }

    #[test]
    fn test_rate_grade_config() {
        let cfg = FogConfig::rate_grade();
        assert!((cfg.fiber_length_m - 50.0).abs() < EPSILON);
        assert!((cfg.coil_diameter_m - 0.04).abs() < EPSILON);
    }

    #[test]
    fn test_config_geometry() {
        let cfg = FogConfig::navigation_grade();
        let r = cfg.coil_radius_m();
        assert!((r - 0.075).abs() < EPSILON);

        let area = cfg.area_per_turn();
        let expected_area = PI * 0.075 * 0.075;
        assert!((area - expected_area).abs() < 1e-10);

        let total_area = cfg.total_area();
        assert!((total_area - expected_area * cfg.num_turns as f64).abs() < 1e-6);
    }

    #[test]
    fn test_wavelength_conversion() {
        let cfg = FogConfig::navigation_grade();
        let lambda = cfg.wavelength_m();
        assert!((lambda - 1.55e-6).abs() < 1e-12);
    }

    #[test]
    fn test_eigenfrequency() {
        let cfg = FogConfig::navigation_grade();
        let f_e = cfg.eigenfrequency();
        // f_e = c / (2*n*L) = 3e8 / (2*1.46*1000) ≈ 102740 Hz
        let expected = C / (2.0 * 1.46 * 1000.0);
        assert!((f_e - expected).abs() < 1.0);
        assert!(f_e > 1e5); // should be ~100 kHz
    }

    #[test]
    fn test_optical_frequency() {
        let cfg = FogConfig::navigation_grade();
        let f_opt = cfg.optical_frequency();
        // c / 1550nm ≈ 1.935e14 Hz
        assert!(f_opt > 1.9e14);
        assert!(f_opt < 2.0e14);
    }

    // --- SagnacEffect tests ---

    #[test]
    fn test_sagnac_phase_zero_rotation() {
        let cfg = FogConfig::navigation_grade();
        let sagnac = SagnacEffect::new(&cfg);
        let phase = sagnac.phase_shift(0.0);
        assert!((phase).abs() < 1e-20);
    }

    #[test]
    fn test_sagnac_phase_earth_rate() {
        let cfg = FogConfig::navigation_grade();
        let sagnac = SagnacEffect::new(&cfg);
        let phase = sagnac.phase_shift(OMEGA_EARTH);
        // For nav-grade FOG, Earth rate should produce measurable phase
        assert!(phase.abs() > 1e-6);
        assert!(phase.abs() < 1.0); // but not huge
    }

    #[test]
    fn test_sagnac_phase_formulations_agree() {
        let cfg = FogConfig::navigation_grade();
        let sagnac = SagnacEffect::new(&cfg);
        let omega = 0.01; // 0.01 rad/s

        let phi_an = sagnac.phase_shift(omega);
        let phi_lr = sagnac.phase_shift_lr(omega);

        // Both formulations should give similar results
        // They differ slightly because L != N * pi * D exactly for discrete turns
        // but for our preset they should be close
        let rel_diff = ((phi_an - phi_lr) / phi_an).abs();
        assert!(rel_diff < 0.01, "Formulations differ by {:.4}%", rel_diff * 100.0);
    }

    #[test]
    fn test_sagnac_linearity() {
        let cfg = FogConfig::navigation_grade();
        let sagnac = SagnacEffect::new(&cfg);

        let phi1 = sagnac.phase_shift(0.001);
        let phi2 = sagnac.phase_shift(0.002);
        assert!((phi2 / phi1 - 2.0).abs() < 1e-10, "Sagnac phase should be linear with rate");
    }

    #[test]
    fn test_scale_factor() {
        let cfg = FogConfig::navigation_grade();
        let sagnac = SagnacEffect::new(&cfg);
        let sf = sagnac.scale_factor();
        // SF = 8*pi*A*N / (lambda*c) where A = pi*R^2
        let r = cfg.coil_radius_m();
        let a = PI * r * r;
        let expected = 8.0 * PI * a * cfg.num_turns as f64 / (1.55e-6 * C);
        assert!((sf - expected).abs() < 1.0);
        // Navigation grade: ~2.03 rad/(rad/s)
        // SF = 8*pi * pi*0.075^2 * 2122 / (1.55e-6 * 3e8) ≈ 2.03
        assert!(sf > 1.0, "Navigation-grade SF should be >1, got {}", sf);
        assert!(sf < 10.0, "Navigation-grade SF should be <10, got {}", sf);
    }

    #[test]
    fn test_rotation_rate_roundtrip() {
        let cfg = FogConfig::navigation_grade();
        let sagnac = SagnacEffect::new(&cfg);
        let omega_in = 0.005;
        let phase = sagnac.phase_shift(omega_in);
        let omega_out = sagnac.rotation_rate_from_phase(phase);
        assert!((omega_out - omega_in).abs() < 1e-12);
    }

    #[test]
    fn test_shot_noise_limit() {
        let cfg = FogConfig::navigation_grade();
        let sagnac = SagnacEffect::new(&cfg);

        let shot_limit = sagnac.shot_noise_limit(100e-6, 0.8, 1.0);
        // Should be very small for nav-grade with reasonable power
        assert!(shot_limit > 0.0);
        assert!(shot_limit < 1e-4, "Shot noise limit = {} rad/s", shot_limit);
    }

    #[test]
    fn test_shot_noise_improves_with_power() {
        let cfg = FogConfig::navigation_grade();
        let sagnac = SagnacEffect::new(&cfg);

        let shot_low = sagnac.shot_noise_limit(10e-6, 0.8, 1.0);
        let shot_high = sagnac.shot_noise_limit(100e-6, 0.8, 1.0);
        assert!(shot_high < shot_low, "Higher power should reduce shot noise");
    }

    #[test]
    fn test_earth_rate_horizontal() {
        let cfg = FogConfig::navigation_grade();
        let sagnac = SagnacEffect::new(&cfg);

        // At equator: full Earth rate
        let omega_0 = sagnac.earth_rate_horizontal(0.0);
        assert!((omega_0 - OMEGA_EARTH).abs() < 1e-10);

        // At pole: zero horizontal component
        let omega_90 = sagnac.earth_rate_horizontal(90.0);
        assert!(omega_90.abs() < 1e-10);

        // At 45 degrees: cos(45) of Earth rate
        let omega_45 = sagnac.earth_rate_horizontal(45.0);
        assert!((omega_45 - OMEGA_EARTH * (PI / 4.0).cos()).abs() < 1e-10);
    }

    #[test]
    fn test_latitude_from_earth_rate() {
        let cfg = FogConfig::navigation_grade();
        let sagnac = SagnacEffect::new(&cfg);

        // At 30 degrees latitude
        let omega_h = sagnac.earth_rate_horizontal(30.0);
        let lat = sagnac.latitude_from_earth_rate(omega_h).unwrap();
        assert!((lat - 30.0).abs() < 1e-6);

        // Invalid: rate exceeds Earth rate
        assert!(sagnac.latitude_from_earth_rate(OMEGA_EARTH * 2.0).is_none());
    }

    // --- FogProcessor tests ---

    #[test]
    fn test_processor_creation() {
        let cfg = FogConfig::navigation_grade();
        let proc = FogProcessor::new(&cfg);
        assert!(proc.modulation_frequency_hz > 0.0);
        assert!((proc.modulation_depth - PI / 2.0).abs() < EPSILON);
        assert!(proc.sample_rate_hz > proc.modulation_frequency_hz);
    }

    #[test]
    fn test_sinusoidal_modulation() {
        let cfg = FogConfig::tactical_grade();
        let proc = FogProcessor::new(&cfg);
        // Use enough samples to cover several full modulation cycles
        let samples_per_cycle = (proc.sample_rate_hz / proc.modulation_frequency_hz).ceil() as usize;
        let num_samples = samples_per_cycle * 10;
        let mod_signal = proc.generate_modulation_sinusoidal(num_samples);

        assert_eq!(mod_signal.len(), num_samples);

        // Check amplitude matches modulation depth (within discretization tolerance)
        let max_val = mod_signal.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min_val = mod_signal.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!((max_val - proc.modulation_depth).abs() < 0.1,
            "Max {:.6} should be near mod depth {:.6}", max_val, proc.modulation_depth);
        assert!((min_val + proc.modulation_depth).abs() < 0.1,
            "Min {:.6} should be near -mod depth {:.6}", min_val, proc.modulation_depth);
    }

    #[test]
    fn test_square_modulation() {
        let cfg = FogConfig::tactical_grade();
        let proc = FogProcessor::new(&cfg);
        let mod_signal = proc.generate_modulation_square(1000);

        assert_eq!(mod_signal.len(), 1000);

        // Should only contain +/- modulation_depth
        for &v in &mod_signal {
            assert!(
                (v - proc.modulation_depth).abs() < EPSILON
                    || (v + proc.modulation_depth).abs() < EPSILON,
                "Square wave value {} not +/- modulation_depth",
                v
            );
        }
    }

    #[test]
    fn test_detector_output_zero_rotation() {
        let cfg = FogConfig::tactical_grade();
        let proc = FogProcessor::new(&cfg);
        let modulation = proc.generate_modulation_sinusoidal(2000);
        let detector = proc.simulate_detector_output(0.0, &modulation, 1.0);

        assert_eq!(detector.len(), 2000);
        // All values should be non-negative (cos ranges -1 to 1, so 1+cos ranges 0 to 2)
        for &v in &detector {
            assert!(v >= -EPSILON, "Detector output should be non-negative: {}", v);
        }
    }

    #[test]
    fn test_detector_output_with_rotation() {
        let cfg = FogConfig::tactical_grade();
        let proc = FogProcessor::new(&cfg);
        let modulation = proc.generate_modulation_sinusoidal(2000);

        let det_zero = proc.simulate_detector_output(0.0, &modulation, 1.0);
        let det_rot = proc.simulate_detector_output(0.5, &modulation, 1.0);

        // Different rotation should produce different detector signals
        let mean_zero: f64 = det_zero.iter().sum::<f64>() / det_zero.len() as f64;
        let mean_rot: f64 = det_rot.iter().sum::<f64>() / det_rot.len() as f64;
        assert!(
            (mean_zero - mean_rot).abs() > 1e-6,
            "Different rotation rates should produce different mean detector outputs"
        );
    }

    #[test]
    fn test_open_loop_demodulation() {
        let cfg = FogConfig::tactical_grade();
        let proc = FogProcessor::new(&cfg);
        let modulation = proc.generate_modulation_sinusoidal(10000);
        let detector = proc.simulate_detector_output(0.1, &modulation, 1.0);

        let (h1, h2, phase) = proc.demodulate_open_loop(&detector);
        // We should get non-zero harmonics
        assert!(h1 > 0.0 || h2 > 0.0, "Demodulation should extract harmonics");
        // Phase should be finite
        assert!(phase.is_finite());
    }

    #[test]
    fn test_open_loop_empty_signal() {
        let cfg = FogConfig::tactical_grade();
        let proc = FogProcessor::new(&cfg);
        let (h1, h2, phase) = proc.demodulate_open_loop(&[]);
        assert!((h1).abs() < EPSILON);
        assert!((h2).abs() < EPSILON);
        assert!((phase).abs() < EPSILON);
    }

    #[test]
    fn test_closed_loop_processing() {
        let cfg = FogConfig::tactical_grade();
        let proc = FogProcessor::new(&cfg);

        let modulation = proc.generate_modulation_sinusoidal(5000);
        let sagnac_phase = 0.01;
        let detector = proc.simulate_detector_output(sagnac_phase, &modulation, 1.0);

        let result = proc.process_closed_loop(&detector, 1.0);
        assert_eq!(result.ramp_history.len(), 5000);
        assert_eq!(result.phase_increments.len(), 5000);
        assert!(result.estimated_rotation_rate.is_finite());
    }

    // --- Noise model tests ---

    #[test]
    fn test_noise_model_shot() {
        let cfg = FogConfig::navigation_grade();
        let noise = FogNoiseModel::new(&cfg);

        let shot = noise.shot_noise_rate(100e-6, 0.8, 1.0);
        assert!(shot > 0.0);
        assert!(shot < 1e-4);
    }

    #[test]
    fn test_noise_model_rin() {
        let cfg = FogConfig::navigation_grade();
        let noise = FogNoiseModel::new(&cfg);

        let rin = noise.rin_phase_noise(-120.0, 1e6);
        assert!(rin > 0.0);
        assert!(rin.is_finite());
    }

    #[test]
    fn test_noise_model_shupe() {
        let cfg = FogConfig::navigation_grade();
        let noise = FogNoiseModel::new(&cfg);

        let shupe = noise.shupe_phase_noise(7e-6, 0.1, 100.0);
        assert!(shupe > 0.0);
        assert!((shupe - 7e-6 * 0.1 * 100.0).abs() < 1e-12);
    }

    #[test]
    fn test_noise_model_rayleigh() {
        let cfg = FogConfig::navigation_grade();
        let noise = FogNoiseModel::new(&cfg);

        let back = noise.rayleigh_backscatter_power(4.5e-5, 1e-3);
        assert!(back > 0.0);
        assert!((back - 4.5e-5 * 1e-3 * 1000.0).abs() < 1e-10);
    }

    #[test]
    fn test_noise_model_faraday() {
        let cfg = FogConfig::navigation_grade();
        let noise = FogNoiseModel::new(&cfg);

        let faraday = noise.faraday_phase_shift(1.0, 50e-6);
        assert!(faraday > 0.0);
        // V * N * B * circumference
        let expected = 1.0 * cfg.num_turns as f64 * 50e-6 * PI * 0.15;
        assert!((faraday - expected).abs() < 1e-10);
    }

    #[test]
    fn test_total_noise_floor() {
        let cfg = FogConfig::navigation_grade();
        let noise = FogNoiseModel::new(&cfg);

        let total = noise.total_noise_floor(100e-6, 0.8, 1.0, -120.0, 1e6);
        assert!(total > 0.0);
        assert!(total.is_finite());
    }

    // --- Allan variance tests ---

    #[test]
    fn test_allan_variance_constant_signal() {
        // A constant signal should have very low Allan variance
        let data: Vec<f64> = vec![1.0; 1000];
        let avar = AllanVariance::compute(&data, 0.01, 20);
        assert!(!avar.is_empty());
        for &(_, av) in &avar {
            assert!(av < 1e-20, "Constant signal AVAR should be ~0, got {}", av);
        }
    }

    #[test]
    fn test_allan_variance_noisy_signal() {
        // Generate white noise-like signal using simple deterministic method
        let n = 2000;
        let mut data = Vec::with_capacity(n);
        let mut val = 0.12345_f64;
        for _ in 0..n {
            val = (val * 6364136223846793005.0 + 1442695040888963407.0) % (1u64 << 32) as f64;
            data.push(val / (1u64 << 32) as f64 - 0.5);
        }

        let avar = AllanVariance::compute(&data, 0.01, 20);
        assert!(!avar.is_empty());

        // For white noise, AVAR should decrease with tau
        if avar.len() >= 2 {
            let (_, av_short) = avar[0];
            let (_, av_long) = avar[avar.len() / 2];
            // White noise Allan variance decreases as 1/tau
            assert!(
                av_long < av_short * 2.0,
                "AVAR should generally decrease: short={}, long={}",
                av_short,
                av_long
            );
        }
    }

    #[test]
    fn test_allan_variance_short_data() {
        let data = vec![1.0, 2.0];
        let avar = AllanVariance::compute(&data, 0.01, 10);
        // Very short data, may produce few or no points
        assert!(avar.len() <= 1);
    }

    #[test]
    fn test_allan_deviation() {
        let data: Vec<f64> = (0..500).map(|i| (i as f64 * 0.01).sin() * 0.001).collect();
        let adev = AllanVariance::compute_adev(&data, 0.01, 10);
        assert!(!adev.is_empty());
        for &(_, ad) in &adev {
            assert!(ad >= 0.0);
        }
    }

    #[test]
    fn test_estimate_arw() {
        let adev_data = vec![
            (0.1, 1e-4),
            (0.5, 5e-5),
            (1.0, 3e-5),
            (5.0, 2e-5),
            (10.0, 1.5e-5),
        ];
        let arw = AllanVariance::estimate_arw(&adev_data);
        assert!(arw.is_some());
        let arw_val = arw.unwrap();
        assert!(arw_val > 0.0);
        // At tau=1.0, adev=3e-5, so ARW = 3e-5 * sqrt(1) = 3e-5
        assert!((arw_val - 3e-5).abs() < 1e-6);
    }

    #[test]
    fn test_estimate_bias_instability() {
        let adev_data = vec![
            (0.1, 1e-4),
            (1.0, 3e-5),
            (10.0, 1e-5),
            (100.0, 5e-6),
            (1000.0, 1e-5),
        ];
        let (tau_min, adev_min) = AllanVariance::estimate_bias_instability(&adev_data).unwrap();
        assert!((tau_min - 100.0).abs() < EPSILON);
        assert!((adev_min - 5e-6).abs() < 1e-10);
    }

    #[test]
    fn test_estimate_rrw() {
        let adev_data = vec![
            (1.0, 1e-4),
            (10.0, 3e-4),
            (100.0, 1e-3),
        ];
        let rrw = AllanVariance::estimate_rrw(&adev_data);
        assert!(rrw.is_some());
        assert!(rrw.unwrap() > 0.0);
    }

    // --- Scale factor calibration tests ---

    #[test]
    fn test_scale_factor_calibration() {
        let cfg = FogConfig::navigation_grade();
        let sagnac = SagnacEffect::new(&cfg);
        let sf = sagnac.scale_factor();

        let mut cal = ScaleFactorCalibration::new(sf, 25.0);

        // Generate ideal calibration points
        for i in 0..10 {
            let omega = (i as f64 - 5.0) * 0.001;
            let measured = sf * omega;
            cal.add_point(omega, measured);
        }

        let (sf_est, sf_err, nonlin) = cal.compute_scale_factor().unwrap();
        assert!((sf_est - sf).abs() / sf < 1e-6, "Estimated SF should match: {} vs {}", sf_est, sf);
        assert!(sf_err.abs() < 1e-6);
        assert!(nonlin < 1.0); // ppm
    }

    #[test]
    fn test_temperature_compensation() {
        let cfg = FogConfig::navigation_grade();
        let sagnac = SagnacEffect::new(&cfg);
        let sf = sagnac.scale_factor();

        let cal = ScaleFactorCalibration::new(sf, 25.0);
        let sf_25 = cal.compensated_scale_factor(25.0);
        let sf_50 = cal.compensated_scale_factor(50.0);

        assert!((sf_25 - sf).abs() < EPSILON);
        assert!(sf_50 != sf_25); // Temperature should change SF
        let relative_change = (sf_50 - sf_25).abs() / sf_25;
        assert!(relative_change < 0.01); // Should be small
    }

    #[test]
    fn test_scale_factor_stability() {
        let measurements = vec![1e6, 1.0001e6, 0.9999e6, 1.00005e6, 0.99995e6];
        let stability = ScaleFactorCalibration::scale_factor_stability(&measurements);
        assert!(stability.is_some());
        let ppm = stability.unwrap();
        assert!(ppm > 0.0);
        assert!(ppm < 1000.0); // reasonable ppm
    }

    // --- Earth rate sensor tests ---

    #[test]
    fn test_earth_rate_horizontal_vertical() {
        let cfg = FogConfig::navigation_grade();
        let ers = EarthRateSensor::new(&cfg);

        let h = ers.expected_horizontal_rate(45.0);
        let v = ers.expected_vertical_rate(45.0);

        // h^2 + v^2 should equal OMEGA_EARTH^2
        let total = (h * h + v * v).sqrt();
        assert!((total - OMEGA_EARTH).abs() < 1e-12);
    }

    #[test]
    fn test_heading_computation() {
        let cfg = FogConfig::navigation_grade();
        let ers = EarthRateSensor::new(&cfg);

        let lat = 45.0;
        let omega_h = ers.expected_horizontal_rate(lat);

        // FOG aligned with North: omega_x = omega_h, omega_y = 0
        let heading = ers.compute_heading(omega_h, 0.0, lat);
        assert!(heading.abs() < 1.0 || (heading - 360.0).abs() < 1.0, "North heading: {}", heading);

        // FOG pointing East: omega_x = 0, omega_y = omega_h
        let heading_east = ers.compute_heading(0.0, omega_h, lat);
        assert!((heading_east - 90.0).abs() < 1.0, "East heading: {}", heading_east);
    }

    #[test]
    fn test_latitude_estimation() {
        let cfg = FogConfig::navigation_grade();
        let ers = EarthRateSensor::new(&cfg);

        let omega_v = ers.expected_vertical_rate(30.0);
        let lat_est = ers.estimate_latitude(omega_v).unwrap();
        assert!((lat_est - 30.0).abs() < 1e-6);
    }

    #[test]
    fn test_earth_rate_snr() {
        let cfg = FogConfig::navigation_grade();
        let ers = EarthRateSensor::new(&cfg);

        let snr = ers.earth_rate_snr(1e-6, 45.0);
        // Earth rate ~ 5.15e-5 at 45 deg, noise floor 1e-6
        assert!(snr > 10.0, "Navigation-grade FOG should easily detect Earth rate, SNR={}", snr);
    }

    // --- Bessel function tests ---

    #[test]
    fn test_bessel_j0_at_zero() {
        assert!((bessel_j0(0.0) - 1.0).abs() < 1e-8);
    }

    #[test]
    fn test_bessel_j1_at_zero() {
        assert!(bessel_j1(0.0).abs() < 1e-8);
    }

    #[test]
    fn test_bessel_j0_known_values() {
        // J0(2.4048) ≈ 0 (first zero)
        assert!(bessel_j0(2.4048).abs() < 1e-3);
        // J0(1.0) ≈ 0.7652
        assert!((bessel_j0(1.0) - 0.7652).abs() < EPSILON_LOOSE);
    }

    #[test]
    fn test_bessel_j1_known_values() {
        // J1(3.8317) ≈ 0 (first zero)
        assert!(bessel_j1(3.8317).abs() < 1e-3);
        // J1(1.0) ≈ 0.4401
        assert!((bessel_j1(1.0) - 0.4401).abs() < EPSILON_LOOSE);
    }

    #[test]
    fn test_bessel_jn_order2() {
        // J2(0) = 0
        assert!(bessel_jn(2, 0.0).abs() < 1e-8);
        // J2(5.1356) ≈ 0 (first zero)
        assert!(bessel_jn(2, 5.1356).abs() < 0.01);
    }

    // --- Modulation analysis tests ---

    #[test]
    fn test_modulation_analysis_optimal() {
        let analysis = modulation_analysis(1.8412); // optimal depth
        assert!(analysis.first_harmonic_sensitivity > 0.5);
        assert!((analysis.optimal_depth - 1.8412).abs() < EPSILON);
    }

    #[test]
    fn test_modulation_analysis_zero() {
        let analysis = modulation_analysis(0.0);
        assert!((analysis.j0 - 1.0).abs() < 1e-6);
        assert!(analysis.j1.abs() < 1e-6);
        assert!(analysis.j2.abs() < 1e-6);
    }

    #[test]
    fn test_modulation_analysis_pi_half() {
        let analysis = modulation_analysis(PI / 2.0);
        // At pi/2: J0 ≈ 0.472, J1 ≈ 0.567, J2 ≈ 0.353
        assert!(analysis.j0 > 0.0);
        assert!(analysis.j1 > 0.0);
        assert!(analysis.j2 > 0.0);
    }
}
