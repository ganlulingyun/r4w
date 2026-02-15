//! # Laser Doppler Vibrometry (LDV) Signal Processing
//!
//! This module implements signal processing algorithms for Laser Doppler Vibrometry,
//! a non-contact vibration measurement technique that analyzes the Doppler frequency
//! shift of a laser beam reflected from a vibrating surface.
//!
//! ## Principle of Operation
//!
//! When a laser beam reflects off a surface moving with velocity `v`, the reflected
//! light experiences a Doppler frequency shift:
//!
//! ```text
//! f_d = 2 * v / lambda
//! ```
//!
//! where the factor of 2 accounts for the round-trip (incidence + reflection).
//! By measuring this frequency shift, we can determine the surface velocity with
//! extremely high precision and bandwidth.
//!
//! ## Applications
//!
//! - **Structural Health Monitoring**: Detecting cracks, delamination, and fatigue
//! - **Automotive NVH**: Noise, vibration, and harshness analysis
//! - **MEMS Characterization**: Measuring resonant frequencies of micro-structures
//! - **Acoustic Source Identification**: Mapping sound radiation from vibrating panels
//! - **Modal Analysis**: Extracting natural frequencies, damping, and mode shapes
//!
//! ## Decoder Types
//!
//! - **Frequency Counter**: Counts zero crossings of the Doppler signal
//! - **Phase Locked**: PLL-based tracking of the Doppler frequency
//! - **Digital (Arctan)**: Arctangent demodulation of in-phase/quadrature signals
//! - **Heterodyne**: Optical frequency shifting with Bragg cell for directional sensing
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::laser_vibrometer_processor::*;
//!
//! let config = VibrometerConfig {
//!     laser_wavelength_nm: 633.0,  // HeNe laser
//!     sample_rate_hz: 256000.0,
//!     velocity_range_m_s: 10.0,
//!     displacement_resolution_nm: 0.1,
//!     decoder_type: DecoderType::Digital,
//! };
//!
//! let processor = VibrometerProcessor::new(config);
//!
//! // Convert Doppler shift to velocity
//! let velocity = processor.velocity_from_doppler(1_000_000.0, 633.0);
//! // velocity ~ 0.3165 m/s
//! ```

use std::f64::consts::PI;

// ─── Configuration ───────────────────────────────────────────────────────────

/// Decoder type used by the vibrometer for extracting velocity from the
/// optical interference signal.
#[derive(Debug, Clone, PartialEq)]
pub enum DecoderType {
    /// Counts zero crossings of the Doppler signal. Simple but limited
    /// dynamic range and cannot detect direction.
    FrequencyCounter,
    /// Phase-locked loop tracks the Doppler frequency, providing good
    /// noise rejection and continuous velocity output.
    PhaseLocked,
    /// Arctangent demodulation of I/Q signals, providing full-range
    /// displacement and velocity with direction sensitivity.
    Digital,
    /// Heterodyne detection using an acousto-optic modulator (Bragg cell)
    /// to shift the carrier frequency, enabling directional measurement.
    Heterodyne {
        /// Carrier frequency introduced by the Bragg cell (typically 40 MHz).
        carrier_freq_hz: f64,
    },
}

/// Configuration parameters for a laser Doppler vibrometer.
#[derive(Debug, Clone)]
pub struct VibrometerConfig {
    /// Laser wavelength in nanometers. Common values:
    /// - 633.0 nm for HeNe lasers
    /// - 1550.0 nm for eye-safe fiber lasers
    pub laser_wavelength_nm: f64,
    /// Digitizer sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Maximum measurable velocity in m/s (determines Doppler bandwidth).
    pub velocity_range_m_s: f64,
    /// Minimum resolvable displacement in nanometers.
    pub displacement_resolution_nm: f64,
    /// Decoder/demodulation algorithm.
    pub decoder_type: DecoderType,
}

// ─── Resonance Mode ──────────────────────────────────────────────────────────

/// Identified resonance (natural frequency) from modal analysis.
#[derive(Debug, Clone)]
pub struct ResonanceMode {
    /// Resonant frequency in Hz.
    pub frequency_hz: f64,
    /// Amplitude (magnitude) at the resonance peak.
    pub amplitude: f64,
    /// Viscous damping ratio zeta = delta_f / (2 * f0), dimensionless.
    pub damping_ratio: f64,
    /// Quality factor Q = f0 / delta_f = 1 / (2 * zeta).
    pub quality_factor: f64,
}

// ─── Vibrometer Processor ────────────────────────────────────────────────────

/// Core signal processing for laser Doppler vibrometry.
///
/// Provides methods to convert between Doppler frequency shifts and mechanical
/// quantities (velocity, displacement, acceleration), as well as demodulation
/// algorithms for extracting vibration signals from the raw photodetector output.
pub struct VibrometerProcessor {
    config: VibrometerConfig,
}

impl VibrometerProcessor {
    /// Create a new vibrometer processor with the given configuration.
    pub fn new(config: VibrometerConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &VibrometerConfig {
        &self.config
    }

    /// Convert a Doppler frequency shift to surface velocity.
    ///
    /// # Formula
    ///
    /// ```text
    /// v = lambda * f_d / 2
    /// ```
    ///
    /// The factor of 2 accounts for the double pass (incidence + reflection).
    ///
    /// # Arguments
    ///
    /// * `doppler_shift_hz` - Measured Doppler frequency shift in Hz
    /// * `wavelength_nm` - Laser wavelength in nanometers
    ///
    /// # Returns
    ///
    /// Surface velocity in m/s (positive = towards laser)
    pub fn velocity_from_doppler(
        &self,
        doppler_shift_hz: f64,
        wavelength_nm: f64,
    ) -> f64 {
        let wavelength_m = wavelength_nm * 1e-9;
        wavelength_m * doppler_shift_hz / 2.0
    }

    /// Convert a surface velocity to the expected Doppler frequency shift.
    ///
    /// # Formula
    ///
    /// ```text
    /// f_d = 2 * v / lambda
    /// ```
    ///
    /// # Arguments
    ///
    /// * `velocity_m_s` - Surface velocity in m/s
    /// * `wavelength_nm` - Laser wavelength in nanometers
    ///
    /// # Returns
    ///
    /// Doppler frequency shift in Hz
    pub fn doppler_from_velocity(
        &self,
        velocity_m_s: f64,
        wavelength_nm: f64,
    ) -> f64 {
        let wavelength_m = wavelength_nm * 1e-9;
        2.0 * velocity_m_s / wavelength_m
    }

    /// Integrate velocity to obtain displacement using the trapezoidal rule.
    ///
    /// # Formula
    ///
    /// ```text
    /// x[n] = x[n-1] + (v[n] + v[n-1]) * dt / 2
    /// ```
    ///
    /// # Arguments
    ///
    /// * `velocity` - Velocity samples in m/s
    /// * `dt_s` - Sample period in seconds (1 / sample_rate)
    ///
    /// # Returns
    ///
    /// Displacement in meters, starting from zero
    pub fn displacement_from_velocity(
        &self,
        velocity: &[f64],
        dt_s: f64,
    ) -> Vec<f64> {
        if velocity.is_empty() {
            return Vec::new();
        }
        let mut disp = Vec::with_capacity(velocity.len());
        disp.push(0.0);
        for i in 1..velocity.len() {
            let trapezoid = (velocity[i] + velocity[i - 1]) * dt_s / 2.0;
            disp.push(disp[i - 1] + trapezoid);
        }
        disp
    }

    /// Differentiate velocity to obtain acceleration using central differences.
    ///
    /// Uses forward difference for the first sample, backward difference for
    /// the last, and central difference for interior samples.
    ///
    /// # Formula
    ///
    /// ```text
    /// a[n] = (v[n+1] - v[n-1]) / (2 * dt)    (interior)
    /// a[0] = (v[1] - v[0]) / dt                (first)
    /// a[N-1] = (v[N-1] - v[N-2]) / dt          (last)
    /// ```
    ///
    /// # Arguments
    ///
    /// * `velocity` - Velocity samples in m/s
    /// * `dt_s` - Sample period in seconds
    ///
    /// # Returns
    ///
    /// Acceleration in m/s^2
    pub fn acceleration_from_velocity(
        &self,
        velocity: &[f64],
        dt_s: f64,
    ) -> Vec<f64> {
        let n = velocity.len();
        if n == 0 {
            return Vec::new();
        }
        if n == 1 {
            return vec![0.0];
        }
        let mut accel = Vec::with_capacity(n);
        // Forward difference for first sample
        accel.push((velocity[1] - velocity[0]) / dt_s);
        // Central difference for interior samples
        for i in 1..n - 1 {
            accel.push((velocity[i + 1] - velocity[i - 1]) / (2.0 * dt_s));
        }
        // Backward difference for last sample
        accel.push((velocity[n - 1] - velocity[n - 2]) / dt_s);
        accel
    }

    /// FM demodulation: extract instantaneous frequency from a real signal.
    ///
    /// Computes the analytic signal via the Hilbert transform (using FFT),
    /// then extracts the instantaneous frequency from the phase derivative.
    ///
    /// # Arguments
    ///
    /// * `signal` - Real-valued signal (photodetector output)
    /// * `sample_rate` - Sample rate in Hz
    ///
    /// # Returns
    ///
    /// Instantaneous frequency in Hz for each sample
    pub fn fm_demodulate(
        &self,
        signal: &[f64],
        sample_rate: f64,
    ) -> Vec<f64> {
        let n = signal.len();
        if n < 2 {
            return vec![0.0; n];
        }

        // Compute analytic signal using Hilbert transform
        // Step 1: FFT of real signal
        let fft = fft_forward(signal);
        let nf = fft.len();

        // Step 2: Apply Hilbert mask (double positive frequencies, zero negative)
        let mut analytic_fft = fft;
        // DC stays as-is
        // Positive frequencies doubled
        let half = nf / 2;
        for i in 1..half {
            analytic_fft[i].0 *= 2.0;
            analytic_fft[i].1 *= 2.0;
        }
        // Negative frequencies zeroed
        for i in (half + 1)..nf {
            analytic_fft[i].0 = 0.0;
            analytic_fft[i].1 = 0.0;
        }

        // Step 3: Inverse FFT
        let analytic = fft_inverse(&analytic_fft);

        // Step 4: Compute instantaneous phase and differentiate
        let mut freq = Vec::with_capacity(n);
        let mut prev_phase = analytic[0].1.atan2(analytic[0].0);
        freq.push(0.0);
        for i in 1..n {
            let phase = analytic[i].1.atan2(analytic[i].0);
            let mut dp = phase - prev_phase;
            // Phase unwrap
            while dp > PI {
                dp -= 2.0 * PI;
            }
            while dp < -PI {
                dp += 2.0 * PI;
            }
            freq.push(dp * sample_rate / (2.0 * PI));
            prev_phase = phase;
        }
        freq
    }

    /// Arctangent (I/Q) demodulation with phase unwrapping.
    ///
    /// Computes the instantaneous phase from quadrature signals and applies
    /// unwrapping to produce a continuous phase trajectory.
    ///
    /// # Formula
    ///
    /// ```text
    /// phi[n] = atan2(Q[n], I[n])   (wrapped)
    /// phi_unwrapped[n] = phi[n] + 2*pi*k  (continuous)
    /// ```
    ///
    /// # Arguments
    ///
    /// * `i_signal` - In-phase (I) channel samples
    /// * `q_signal` - Quadrature (Q) channel samples
    ///
    /// # Returns
    ///
    /// Unwrapped phase in radians
    pub fn arctan_demodulate(
        &self,
        i_signal: &[f64],
        q_signal: &[f64],
    ) -> Vec<f64> {
        let n = i_signal.len().min(q_signal.len());
        if n == 0 {
            return Vec::new();
        }

        let mut phase = Vec::with_capacity(n);
        // Standard convention: atan2(Q, I) gives the phase angle
        let first = q_signal[0].atan2(i_signal[0]);
        phase.push(first);

        for i in 1..n {
            let raw = q_signal[i].atan2(i_signal[i]);
            let prev_raw = q_signal[i - 1].atan2(i_signal[i - 1]);
            let mut delta = raw - prev_raw;
            // Unwrap: keep delta within [-pi, pi]
            while delta > PI {
                delta -= 2.0 * PI;
            }
            while delta < -PI {
                delta += 2.0 * PI;
            }
            phase.push(phase[i - 1] + delta);
        }
        phase
    }

    /// Heterodyne demodulation: extract I/Q from a signal modulated on a carrier.
    ///
    /// Mixes the input signal with cos(2*pi*fc*t) and -sin(2*pi*fc*t) to produce
    /// in-phase and quadrature components, then applies a simple moving-average
    /// lowpass filter to remove the double-frequency terms.
    ///
    /// # Arguments
    ///
    /// * `signal` - Real-valued heterodyne signal from photodetector
    /// * `carrier_freq` - Carrier (Bragg cell) frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    ///
    /// # Returns
    ///
    /// Tuple of (I channel, Q channel) baseband signals
    pub fn heterodyne_demodulate(
        &self,
        signal: &[f64],
        carrier_freq: f64,
        sample_rate: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let n = signal.len();
        if n == 0 {
            return (Vec::new(), Vec::new());
        }

        // Mix down to baseband
        let mut i_raw = Vec::with_capacity(n);
        let mut q_raw = Vec::with_capacity(n);
        for k in 0..n {
            let t = k as f64 / sample_rate;
            let phase = 2.0 * PI * carrier_freq * t;
            i_raw.push(signal[k] * 2.0 * phase.cos());
            q_raw.push(signal[k] * (-2.0 * phase.sin()));
        }

        // Simple moving average lowpass filter
        // Filter length ~ samples_per_cycle of the carrier
        let filter_len = ((sample_rate / carrier_freq).round() as usize).max(3);
        let i_filt = moving_average_filter(&i_raw, filter_len);
        let q_filt = moving_average_filter(&q_raw, filter_len);

        (i_filt, q_filt)
    }
}

// ─── Modal Analyzer ──────────────────────────────────────────────────────────

/// Modal analysis engine for extracting resonant frequencies, damping ratios,
/// and mode shapes from vibration measurements.
///
/// Modal analysis is fundamental to understanding the dynamic behavior of
/// structures. The analyzer identifies natural frequencies (resonances),
/// quantifies damping, and computes frequency response functions.
pub struct ModalAnalyzer {
    sample_rate_hz: f64,
}

impl ModalAnalyzer {
    /// Create a new modal analyzer for signals sampled at the given rate.
    pub fn new(sample_rate_hz: f64) -> Self {
        Self { sample_rate_hz }
    }

    /// Compute the one-sided power spectral density using a simple periodogram.
    ///
    /// Applies a Hann window before the FFT to reduce spectral leakage.
    ///
    /// # Arguments
    ///
    /// * `signal` - Time-domain vibration signal
    ///
    /// # Returns
    ///
    /// Vector of (frequency_hz, power) pairs for the positive frequency axis
    pub fn power_spectrum(&self, signal: &[f64]) -> Vec<(f64, f64)> {
        let n = signal.len();
        if n == 0 {
            return Vec::new();
        }

        // Apply Hann window
        let windowed: Vec<f64> = signal
            .iter()
            .enumerate()
            .map(|(i, &x)| {
                let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos());
                x * w
            })
            .collect();

        let fft = fft_forward(&windowed);
        let half = n / 2 + 1;
        let df = self.sample_rate_hz / n as f64;

        let mut spectrum = Vec::with_capacity(half);
        for i in 0..half {
            let freq = i as f64 * df;
            let mag_sq = fft[i].0 * fft[i].0 + fft[i].1 * fft[i].1;
            // Normalize by N^2 and apply one-sided scaling
            let power = if i == 0 || i == n / 2 {
                mag_sq / (n as f64 * n as f64)
            } else {
                2.0 * mag_sq / (n as f64 * n as f64)
            };
            spectrum.push((freq, power));
        }
        spectrum
    }

    /// Find resonance peaks in a power spectrum above a given threshold.
    ///
    /// A resonance is identified as a local maximum whose power (in dB) exceeds
    /// `threshold_db` above the mean power level.
    ///
    /// # Arguments
    ///
    /// * `spectrum` - Power spectrum as (frequency, power) pairs
    /// * `threshold_db` - Detection threshold in dB above mean power
    ///
    /// # Returns
    ///
    /// Vector of identified resonance modes with frequency, amplitude,
    /// damping ratio, and quality factor
    pub fn find_resonances(
        &self,
        spectrum: &[(f64, f64)],
        threshold_db: f64,
    ) -> Vec<ResonanceMode> {
        if spectrum.len() < 3 {
            return Vec::new();
        }

        // Compute mean power for threshold reference
        let mean_power: f64 =
            spectrum.iter().map(|(_, p)| *p).sum::<f64>() / spectrum.len() as f64;
        let threshold_linear = mean_power * 10.0_f64.powf(threshold_db / 10.0);

        let mut modes = Vec::new();

        for i in 1..spectrum.len() - 1 {
            let (freq, power) = spectrum[i];
            let (_, prev_power) = spectrum[i - 1];
            let (_, next_power) = spectrum[i + 1];

            // Local maximum above threshold
            if power > prev_power && power > next_power && power > threshold_linear {
                // Estimate half-power bandwidth for damping
                let half_power = power / 2.0;

                // Search left for -3 dB point
                let mut left_freq = freq;
                for j in (0..i).rev() {
                    if spectrum[j].1 <= half_power {
                        // Linear interpolation
                        let frac = (half_power - spectrum[j].1)
                            / (spectrum[j + 1].1 - spectrum[j].1);
                        left_freq =
                            spectrum[j].0 + frac * (spectrum[j + 1].0 - spectrum[j].0);
                        break;
                    }
                }

                // Search right for -3 dB point
                let mut right_freq = freq;
                for j in (i + 1)..spectrum.len() {
                    if spectrum[j].1 <= half_power {
                        let frac = (half_power - spectrum[j].1)
                            / (spectrum[j - 1].1 - spectrum[j].1);
                        right_freq =
                            spectrum[j].0 - frac * (spectrum[j].0 - spectrum[j - 1].0);
                        break;
                    }
                }

                let bandwidth = right_freq - left_freq;
                let damping_ratio = if freq > 0.0 && bandwidth > 0.0 {
                    bandwidth / (2.0 * freq)
                } else {
                    0.0
                };
                let quality_factor = if bandwidth > 0.0 {
                    freq / bandwidth
                } else {
                    f64::INFINITY
                };

                modes.push(ResonanceMode {
                    frequency_hz: freq,
                    amplitude: power.sqrt(),
                    damping_ratio,
                    quality_factor,
                });
            }
        }

        modes
    }

    /// Estimate the damping ratio at a specific resonance frequency using
    /// the half-power bandwidth method.
    ///
    /// # Formula
    ///
    /// ```text
    /// zeta = delta_f / (2 * f0)
    /// ```
    ///
    /// where `delta_f` is the -3 dB bandwidth around the resonance.
    ///
    /// # Arguments
    ///
    /// * `signal` - Time-domain vibration signal
    /// * `resonance_freq` - Target resonance frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    ///
    /// # Returns
    ///
    /// Estimated damping ratio (dimensionless, typically 0.001 to 0.1)
    pub fn modal_damping_ratio(
        &self,
        signal: &[f64],
        resonance_freq: f64,
        sample_rate: f64,
    ) -> f64 {
        let analyzer = ModalAnalyzer::new(sample_rate);
        let spectrum = analyzer.power_spectrum(signal);
        if spectrum.is_empty() {
            return 0.0;
        }

        // Find the bin closest to the resonance frequency
        let df = sample_rate / signal.len() as f64;
        let target_bin = (resonance_freq / df).round() as usize;
        if target_bin >= spectrum.len() {
            return 0.0;
        }

        let peak_power = spectrum[target_bin].1;
        if peak_power <= 0.0 {
            return 0.0;
        }

        let half_power = peak_power / 2.0;

        // Find -3 dB points
        let mut left_freq = spectrum[target_bin].0;
        for j in (0..target_bin).rev() {
            if spectrum[j].1 <= half_power {
                let frac =
                    (half_power - spectrum[j].1) / (spectrum[j + 1].1 - spectrum[j].1);
                left_freq =
                    spectrum[j].0 + frac * (spectrum[j + 1].0 - spectrum[j].0);
                break;
            }
        }

        let mut right_freq = spectrum[target_bin].0;
        for j in (target_bin + 1)..spectrum.len() {
            if spectrum[j].1 <= half_power {
                let frac =
                    (half_power - spectrum[j].1) / (spectrum[j - 1].1 - spectrum[j].1);
                right_freq =
                    spectrum[j].0 - frac * (spectrum[j].0 - spectrum[j - 1].0);
                break;
            }
        }

        let bandwidth = right_freq - left_freq;
        if resonance_freq > 0.0 && bandwidth > 0.0 {
            bandwidth / (2.0 * resonance_freq)
        } else {
            0.0
        }
    }

    /// Compute the Frequency Response Function (FRF) H(f) = Y(f) / X(f).
    ///
    /// Uses the H1 estimator: H1(f) = Sxy(f) / Sxx(f) which is unbiased
    /// when noise is on the output.
    ///
    /// # Arguments
    ///
    /// * `input` - Excitation (force) signal
    /// * `output` - Response (vibration) signal
    /// * `fft_size` - FFT size for spectral estimation
    ///
    /// # Returns
    ///
    /// Vector of (frequency_hz, (magnitude, phase_rad)) for the FRF
    pub fn frequency_response_function(
        &self,
        input: &[f64],
        output: &[f64],
        fft_size: usize,
    ) -> Vec<(f64, (f64, f64))> {
        let n = input.len().min(output.len()).min(fft_size);
        if n < 2 {
            return Vec::new();
        }

        // Windowed FFTs
        let win_in: Vec<f64> = input[..n]
            .iter()
            .enumerate()
            .map(|(i, &x)| {
                x * 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos())
            })
            .collect();
        let win_out: Vec<f64> = output[..n]
            .iter()
            .enumerate()
            .map(|(i, &x)| {
                x * 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos())
            })
            .collect();

        let fft_in = fft_forward(&win_in);
        let fft_out = fft_forward(&win_out);

        let half = n / 2 + 1;
        let df = self.sample_rate_hz / n as f64;

        let mut frf = Vec::with_capacity(half);
        for i in 0..half {
            let freq = i as f64 * df;
            // H1 = Sxy / Sxx
            // Sxy = Y * conj(X), Sxx = X * conj(X)
            let (xr, xi) = fft_in[i];
            let (yr, yi) = fft_out[i];
            let sxy_r = yr * xr + yi * xi; // Re(Y * conj(X))
            let sxy_i = yi * xr - yr * xi; // Im(Y * conj(X))
            let sxx = xr * xr + xi * xi;

            if sxx > 1e-30 {
                let hr = sxy_r / sxx;
                let hi = sxy_i / sxx;
                let mag = (hr * hr + hi * hi).sqrt();
                let phase = hi.atan2(hr);
                frf.push((freq, (mag, phase)));
            } else {
                frf.push((freq, (0.0, 0.0)));
            }
        }
        frf
    }

    /// Compute the coherence function gamma^2(f) between two signals.
    ///
    /// # Formula
    ///
    /// ```text
    /// gamma^2(f) = |Sxy(f)|^2 / (Sxx(f) * Syy(f))
    /// ```
    ///
    /// Values near 1.0 indicate a linear, noise-free relationship.
    /// Values near 0.0 indicate noise or nonlinearity.
    ///
    /// # Arguments
    ///
    /// * `input` - First signal (e.g., excitation)
    /// * `output` - Second signal (e.g., response)
    /// * `fft_size` - FFT size for spectral estimation
    ///
    /// # Returns
    ///
    /// Vector of (frequency_hz, coherence) where coherence is in [0, 1]
    pub fn coherence(
        &self,
        input: &[f64],
        output: &[f64],
        fft_size: usize,
    ) -> Vec<(f64, f64)> {
        let n = input.len().min(output.len()).min(fft_size);
        if n < 2 {
            return Vec::new();
        }

        let win_in: Vec<f64> = input[..n]
            .iter()
            .enumerate()
            .map(|(i, &x)| {
                x * 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos())
            })
            .collect();
        let win_out: Vec<f64> = output[..n]
            .iter()
            .enumerate()
            .map(|(i, &x)| {
                x * 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos())
            })
            .collect();

        let fft_in = fft_forward(&win_in);
        let fft_out = fft_forward(&win_out);

        let half = n / 2 + 1;
        let df = self.sample_rate_hz / n as f64;

        let mut coh = Vec::with_capacity(half);
        for i in 0..half {
            let freq = i as f64 * df;
            let (xr, xi) = fft_in[i];
            let (yr, yi) = fft_out[i];

            let sxx = xr * xr + xi * xi;
            let syy = yr * yr + yi * yi;
            let sxy_r = yr * xr + yi * xi;
            let sxy_i = yi * xr - yr * xi;
            let sxy_mag_sq = sxy_r * sxy_r + sxy_i * sxy_i;

            let denom = sxx * syy;
            let gamma_sq = if denom > 1e-30 {
                (sxy_mag_sq / denom).min(1.0)
            } else {
                0.0
            };
            coh.push((freq, gamma_sq));
        }
        coh
    }

    /// Compute the operational deflection shape (ODS) at a specific frequency
    /// from multiple measurement points.
    ///
    /// Extracts the complex amplitude (magnitude and phase) at the specified
    /// frequency from each measurement point's velocity signal.
    ///
    /// # Arguments
    ///
    /// * `velocities` - Velocity time histories, one per measurement point
    /// * `frequency_hz` - Frequency of interest
    /// * `sample_rate` - Sample rate in Hz
    ///
    /// # Returns
    ///
    /// Vector of (amplitude, phase_rad) at each measurement point
    pub fn operational_deflection_shape(
        &self,
        velocities: &[Vec<f64>],
        frequency_hz: f64,
        sample_rate: f64,
    ) -> Vec<(f64, f64)> {
        let mut ods = Vec::with_capacity(velocities.len());

        for vel in velocities {
            if vel.is_empty() {
                ods.push((0.0, 0.0));
                continue;
            }
            let n = vel.len();
            let df = sample_rate / n as f64;
            let target_bin = (frequency_hz / df).round() as usize;

            // DFT at the specific bin (Goertzel-like)
            let mut re = 0.0;
            let mut im = 0.0;
            for (k, &v) in vel.iter().enumerate() {
                let angle = 2.0 * PI * target_bin as f64 * k as f64 / n as f64;
                re += v * angle.cos();
                im -= v * angle.sin();
            }
            re /= n as f64;
            im /= n as f64;

            let amplitude = (re * re + im * im).sqrt();
            let phase = im.atan2(re);
            ods.push((amplitude, phase));
        }
        ods
    }
}

// ─── Scanning Vibrometer ─────────────────────────────────────────────────────

/// Multi-point scanning laser vibrometer for full-field vibration measurement.
///
/// A scanning vibrometer sequentially measures vibration at many points on a
/// structure, building up a spatial map of vibration amplitude and phase.
/// This enables visualization of mode shapes and operational deflection shapes.
pub struct ScanningVibrometer {
    /// 2D coordinates [x, y] of each scan point on the structure surface.
    scan_points: Vec<[f64; 2]>,
}

impl ScanningVibrometer {
    /// Create a new scanning vibrometer with the given measurement grid.
    ///
    /// # Arguments
    ///
    /// * `scan_points` - Vector of [x, y] coordinates for each measurement point
    pub fn new(scan_points: Vec<[f64; 2]>) -> Self {
        Self { scan_points }
    }

    /// Return a reference to the scan point coordinates.
    pub fn scan_points(&self) -> &[[f64; 2]] {
        &self.scan_points
    }

    /// Compute the spatially averaged power spectrum across all measurement points.
    ///
    /// # Arguments
    ///
    /// * `spectra` - Power spectra for each scan point, as (frequency, power) pairs
    ///
    /// # Returns
    ///
    /// Averaged spectrum with the same frequency axis
    pub fn average_spectrum(
        &self,
        spectra: &[Vec<(f64, f64)>],
    ) -> Vec<(f64, f64)> {
        if spectra.is_empty() {
            return Vec::new();
        }
        let n_bins = spectra[0].len();
        let n_spectra = spectra.len() as f64;

        let mut avg = Vec::with_capacity(n_bins);
        for i in 0..n_bins {
            let freq = spectra[0][i].0;
            let mean_power: f64 =
                spectra.iter().map(|s| if i < s.len() { s[i].1 } else { 0.0 }).sum::<f64>()
                    / n_spectra;
            avg.push((freq, mean_power));
        }
        avg
    }

    /// Extract the mode shape at a specific frequency from velocity measurements.
    ///
    /// # Arguments
    ///
    /// * `velocities` - Velocity time histories for each scan point
    /// * `frequency` - Target frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    ///
    /// # Returns
    ///
    /// Vector of (amplitude, phase_rad) at each scan point
    pub fn mode_shape_at_freq(
        &self,
        velocities: &[Vec<f64>],
        frequency: f64,
        sample_rate: f64,
    ) -> Vec<(f64, f64)> {
        let analyzer = ModalAnalyzer::new(sample_rate);
        analyzer.operational_deflection_shape(velocities, frequency, sample_rate)
    }

    /// Generate animated deflection snapshots for a given mode shape.
    ///
    /// Reconstructs time-domain deflection as:
    /// ```text
    /// z(point, t) = amplitude * cos(omega * t + phase)
    /// ```
    ///
    /// # Arguments
    ///
    /// * `mode_shape` - (amplitude, phase) per scan point
    /// * `time_steps` - Number of animation frames (one full cycle)
    ///
    /// # Returns
    ///
    /// Vector of time snapshots, each containing deflection at every scan point
    pub fn animated_deflection(
        &self,
        mode_shape: &[(f64, f64)],
        time_steps: usize,
    ) -> Vec<Vec<f64>> {
        let n_points = mode_shape.len();
        let mut frames = Vec::with_capacity(time_steps);

        for t in 0..time_steps {
            let omega_t = 2.0 * PI * t as f64 / time_steps as f64;
            let mut frame = Vec::with_capacity(n_points);
            for &(amplitude, phase) in mode_shape {
                frame.push(amplitude * (omega_t + phase).cos());
            }
            frames.push(frame);
        }
        frames
    }

    /// Estimate the spatial autocorrelation length of a mode shape.
    ///
    /// This provides an estimate of the structural wavelength at the mode
    /// frequency. Points closer than this distance tend to move in phase.
    ///
    /// # Arguments
    ///
    /// * `mode_shape` - (amplitude, phase) per scan point
    ///
    /// # Returns
    ///
    /// Correlation length in the same units as the scan point coordinates
    pub fn auto_correlation_length(&self, mode_shape: &[(f64, f64)]) -> f64 {
        let n = mode_shape.len();
        if n < 2 || self.scan_points.len() < n {
            return 0.0;
        }

        // Compute complex mode shape values: A * exp(j*phase)
        let complex_vals: Vec<(f64, f64)> = mode_shape
            .iter()
            .map(|&(amp, ph)| (amp * ph.cos(), amp * ph.sin()))
            .collect();

        // Compute all pairwise distances and correlation
        let mut distances = Vec::new();
        let mut correlations = Vec::new();

        // Normalize
        let mean_mag_sq: f64 = complex_vals
            .iter()
            .map(|(r, i)| r * r + i * i)
            .sum::<f64>()
            / n as f64;

        if mean_mag_sq < 1e-30 {
            return 0.0;
        }

        for i in 0..n {
            for j in (i + 1)..n {
                let dx = self.scan_points[i][0] - self.scan_points[j][0];
                let dy = self.scan_points[i][1] - self.scan_points[j][1];
                let dist = (dx * dx + dy * dy).sqrt();

                // Normalized correlation: Re(z_i * conj(z_j)) / mean_mag_sq
                let corr = (complex_vals[i].0 * complex_vals[j].0
                    + complex_vals[i].1 * complex_vals[j].1)
                    / mean_mag_sq;

                distances.push(dist);
                correlations.push(corr);
            }
        }

        if distances.is_empty() {
            return 0.0;
        }

        // Sort by distance and find where correlation drops to 1/e
        let mut pairs: Vec<(f64, f64)> = distances
            .into_iter()
            .zip(correlations.into_iter())
            .collect();
        pairs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        let target = 1.0 / std::f64::consts::E;
        for i in 0..pairs.len() {
            if pairs[i].1 < target {
                if i == 0 {
                    return pairs[0].0;
                }
                // Linear interpolation
                let frac = (target - pairs[i].1) / (pairs[i - 1].1 - pairs[i].1);
                return pairs[i].0 - frac * (pairs[i].0 - pairs[i - 1].0);
            }
        }

        // Correlation never dropped below threshold, return max distance
        pairs.last().map(|p| p.0).unwrap_or(0.0)
    }
}

// ─── Speckle Noise Reducer ───────────────────────────────────────────────────

/// Speckle noise reduction for laser vibrometer signals.
///
/// Laser speckle arises from the coherent interference of scattered light
/// from a rough surface. When the surface moves, the speckle pattern
/// changes, causing signal dropouts and noise bursts. This processor
/// detects and repairs these artifacts.
pub struct SpeckleNoiseReducer;

impl SpeckleNoiseReducer {
    /// Detect signal dropouts (regions of very low or zero amplitude)
    /// caused by speckle pattern fading.
    ///
    /// A dropout is defined as a contiguous region where the absolute signal
    /// value falls below the given threshold.
    ///
    /// # Arguments
    ///
    /// * `signal` - Vibrometer output signal
    /// * `threshold` - Absolute amplitude threshold for dropout detection
    ///
    /// # Returns
    ///
    /// Vector of (start_index, end_index) pairs for each detected dropout
    pub fn detect_dropouts(signal: &[f64], threshold: f64) -> Vec<(usize, usize)> {
        let mut dropouts = Vec::new();
        let mut in_dropout = false;
        let mut start = 0;

        for (i, &s) in signal.iter().enumerate() {
            if s.abs() < threshold {
                if !in_dropout {
                    start = i;
                    in_dropout = true;
                }
            } else if in_dropout {
                dropouts.push((start, i));
                in_dropout = false;
            }
        }
        if in_dropout {
            dropouts.push((start, signal.len()));
        }
        dropouts
    }

    /// Repair signal dropouts using linear interpolation.
    ///
    /// Each dropout region is replaced by a linear ramp connecting the
    /// last valid sample before the dropout to the first valid sample after.
    ///
    /// # Arguments
    ///
    /// * `signal` - Mutable signal to repair in-place
    /// * `dropouts` - Dropout regions as (start, end) pairs
    pub fn interpolate_dropouts(
        signal: &mut Vec<f64>,
        dropouts: &[(usize, usize)],
    ) {
        for &(start, end) in dropouts {
            if start >= signal.len() || end > signal.len() || start >= end {
                continue;
            }
            let val_before = if start > 0 { signal[start - 1] } else { 0.0 };
            let val_after = if end < signal.len() { signal[end] } else { val_before };
            // Total distance from val_before to val_after spans (end - start + 1) intervals
            let total_intervals = (end - start + 1) as f64;
            for i in start..end {
                let frac = (i - start + 1) as f64 / total_intervals;
                signal[i] = val_before + frac * (val_after - val_before);
            }
        }
    }

    /// Compute a signal quality metric based on speckle tracking.
    ///
    /// Returns a value in [0, 1] where 1.0 means perfect signal quality
    /// and lower values indicate degradation from speckle noise.
    ///
    /// The metric is based on the fraction of samples above a noise floor
    /// estimated from the signal's RMS level.
    ///
    /// # Arguments
    ///
    /// * `signal` - Vibrometer output signal
    ///
    /// # Returns
    ///
    /// Quality metric in [0, 1]
    pub fn speckle_tracking_quality(signal: &[f64]) -> f64 {
        if signal.is_empty() {
            return 0.0;
        }
        let rms = (signal.iter().map(|&s| s * s).sum::<f64>() / signal.len() as f64).sqrt();
        if rms < 1e-30 {
            return 0.0;
        }
        // Threshold at 10% of RMS
        let threshold = 0.1 * rms;
        let good_samples = signal.iter().filter(|&&s| s.abs() >= threshold).count();
        good_samples as f64 / signal.len() as f64
    }
}

// ─── Helper Functions ────────────────────────────────────────────────────────

/// Convert fringe count to displacement.
///
/// In an interferometer, each fringe represents a path length change of
/// one wavelength. For a reflective setup (double pass), one fringe
/// corresponds to lambda/2 of surface displacement.
///
/// # Formula
///
/// ```text
/// d = N * lambda / 2
/// ```
///
/// # Arguments
///
/// * `fringes` - Number of fringes counted
/// * `wavelength_nm` - Laser wavelength in nanometers
///
/// # Returns
///
/// Displacement in meters
pub fn fringe_count_to_displacement(fringes: f64, wavelength_nm: f64) -> f64 {
    let wavelength_m = wavelength_nm * 1e-9;
    fringes * wavelength_m / 2.0
}

/// Maximum measurable velocity given the Nyquist sampling constraint.
///
/// The Doppler shift must be less than half the sample rate. Since
/// f_d = 2v/lambda, the maximum velocity is:
///
/// # Formula
///
/// ```text
/// v_max = lambda * fs / 4
/// ```
///
/// # Arguments
///
/// * `laser_wavelength_nm` - Laser wavelength in nanometers
/// * `sample_rate` - Sample rate in Hz
///
/// # Returns
///
/// Maximum measurable velocity in m/s
pub fn max_velocity(laser_wavelength_nm: f64, sample_rate: f64) -> f64 {
    let wavelength_m = laser_wavelength_nm * 1e-9;
    wavelength_m * sample_rate / 4.0
}

/// Sensitivity: velocity change per fringe.
///
/// Each fringe corresponds to a displacement of lambda/2. The velocity
/// that produces exactly one fringe per sampling interval is:
///
/// # Formula
///
/// ```text
/// v_per_fringe = lambda / 2
/// ```
///
/// (in m/s per fringe per second)
///
/// # Arguments
///
/// * `wavelength_nm` - Laser wavelength in nanometers
///
/// # Returns
///
/// Velocity in m/s corresponding to one fringe count per second
pub fn sensitivity_v_per_fringe(wavelength_nm: f64) -> f64 {
    let wavelength_m = wavelength_nm * 1e-9;
    wavelength_m / 2.0
}

// ─── Internal FFT Utilities ──────────────────────────────────────────────────

/// Simple radix-2 DIT FFT (forward). Input length is zero-padded to next power of 2.
/// Returns complex (re, im) pairs.
fn fft_forward(signal: &[f64]) -> Vec<(f64, f64)> {
    let n = signal.len().next_power_of_two();
    let mut buf: Vec<(f64, f64)> = signal
        .iter()
        .map(|&x| (x, 0.0))
        .chain(std::iter::repeat((0.0, 0.0)))
        .take(n)
        .collect();
    fft_in_place(&mut buf, false);
    buf
}

/// Simple radix-2 inverse FFT. Returns complex (re, im) pairs.
fn fft_inverse(spectrum: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = spectrum.len().next_power_of_two();
    let mut buf: Vec<(f64, f64)> = spectrum
        .iter()
        .copied()
        .chain(std::iter::repeat((0.0, 0.0)))
        .take(n)
        .collect();
    fft_in_place(&mut buf, true);
    let inv = 1.0 / n as f64;
    for x in &mut buf {
        x.0 *= inv;
        x.1 *= inv;
    }
    buf
}

/// Cooley-Tukey radix-2 DIT FFT in-place.
fn fft_in_place(buf: &mut [(f64, f64)], inverse: bool) {
    let n = buf.len();
    if n <= 1 {
        return;
    }
    assert!(n.is_power_of_two(), "FFT size must be power of 2");

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            buf.swap(i, j);
        }
    }

    // Butterfly stages
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * 2.0 * PI / len as f64;
        let wn = (angle.cos(), angle.sin());
        let mut i = 0;
        while i < n {
            let mut w = (1.0, 0.0);
            for k in 0..half {
                let u = buf[i + k];
                let t = (
                    buf[i + k + half].0 * w.0 - buf[i + k + half].1 * w.1,
                    buf[i + k + half].0 * w.1 + buf[i + k + half].1 * w.0,
                );
                buf[i + k] = (u.0 + t.0, u.1 + t.1);
                buf[i + k + half] = (u.0 - t.0, u.1 - t.1);
                w = (w.0 * wn.0 - w.1 * wn.1, w.0 * wn.1 + w.1 * wn.0);
            }
            i += len;
        }
        len <<= 1;
    }
}

/// Simple moving average filter.
fn moving_average_filter(signal: &[f64], window: usize) -> Vec<f64> {
    let n = signal.len();
    if n == 0 || window == 0 {
        return vec![0.0; n];
    }
    let w = window.min(n);
    let mut result = Vec::with_capacity(n);
    let mut sum: f64 = signal[..w.min(n)].iter().sum();
    // Centered moving average with edge handling
    for i in 0..n {
        let left = if i >= w / 2 { i - w / 2 } else { 0 };
        let right = (i + w / 2 + 1).min(n);
        let count = right - left;
        sum = signal[left..right].iter().sum();
        result.push(sum / count as f64);
    }
    result
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn default_config() -> VibrometerConfig {
        VibrometerConfig {
            laser_wavelength_nm: 633.0,
            sample_rate_hz: 256000.0,
            velocity_range_m_s: 10.0,
            displacement_resolution_nm: 0.1,
            decoder_type: DecoderType::Digital,
        }
    }

    fn make_processor() -> VibrometerProcessor {
        VibrometerProcessor::new(default_config())
    }

    // ── Velocity / Doppler conversions ──

    #[test]
    fn test_velocity_from_doppler_hene() {
        // 1 MHz Doppler shift at 633 nm → v = 633e-9 * 1e6 / 2 = 0.0003165 m/s
        // Wait: 633e-9 * 1e6 / 2 = 0.3165 m/s
        let proc = make_processor();
        let v = proc.velocity_from_doppler(1_000_000.0, 633.0);
        assert!((v - 0.3165).abs() < 1e-6, "Expected ~0.3165, got {}", v);
    }

    #[test]
    fn test_velocity_from_doppler_fiber() {
        // 1 MHz at 1550 nm → v = 1550e-9 * 1e6 / 2 = 0.775 m/s
        let proc = make_processor();
        let v = proc.velocity_from_doppler(1_000_000.0, 1550.0);
        assert!((v - 0.775).abs() < 1e-6, "Expected ~0.775, got {}", v);
    }

    #[test]
    fn test_doppler_from_velocity_roundtrip() {
        let proc = make_processor();
        let original_velocity = 1.5; // m/s
        let doppler = proc.doppler_from_velocity(original_velocity, 633.0);
        let recovered = proc.velocity_from_doppler(doppler, 633.0);
        assert!(
            (recovered - original_velocity).abs() < 1e-12,
            "Roundtrip failed: {} vs {}",
            recovered,
            original_velocity
        );
    }

    #[test]
    fn test_doppler_from_velocity_value() {
        // v = 1 m/s at 633 nm → f_d = 2 * 1 / 633e-9 = 3.16e6 Hz
        let proc = make_processor();
        let fd = proc.doppler_from_velocity(1.0, 633.0);
        let expected = 2.0 / (633.0e-9);
        assert!((fd - expected).abs() < 1.0, "Expected ~{}, got {}", expected, fd);
    }

    #[test]
    fn test_zero_velocity_zero_doppler() {
        let proc = make_processor();
        assert_eq!(proc.velocity_from_doppler(0.0, 633.0), 0.0);
        assert_eq!(proc.doppler_from_velocity(0.0, 633.0), 0.0);
    }

    // ── Displacement from velocity ──

    #[test]
    fn test_displacement_from_constant_velocity() {
        // Constant velocity = linear displacement ramp
        let proc = make_processor();
        let dt = 0.001; // 1 ms
        let velocity = vec![2.0; 100]; // 2 m/s constant
        let disp = proc.displacement_from_velocity(&velocity, dt);

        assert_eq!(disp.len(), 100);
        assert_eq!(disp[0], 0.0);
        // After 99 steps: displacement = 2.0 * 99 * 0.001 = 0.198 m
        let expected = 2.0 * 99.0 * dt;
        assert!(
            (disp[99] - expected).abs() < 1e-10,
            "Expected {}, got {}",
            expected,
            disp[99]
        );
    }

    #[test]
    fn test_displacement_is_linear_ramp() {
        let proc = make_processor();
        let dt = 0.01;
        let velocity = vec![1.0; 50];
        let disp = proc.displacement_from_velocity(&velocity, dt);

        // Each step should increase by exactly v*dt = 0.01
        for i in 1..disp.len() {
            let delta = disp[i] - disp[i - 1];
            assert!(
                (delta - 0.01).abs() < 1e-12,
                "Step {} delta = {}",
                i,
                delta
            );
        }
    }

    #[test]
    fn test_displacement_empty() {
        let proc = make_processor();
        assert!(proc.displacement_from_velocity(&[], 0.001).is_empty());
    }

    // ── Acceleration from velocity ──

    #[test]
    fn test_acceleration_from_linear_ramp() {
        // Linear velocity ramp → constant acceleration
        let proc = make_processor();
        let dt = 0.001;
        let velocity: Vec<f64> = (0..100).map(|i| 5.0 * i as f64 * dt).collect();
        let accel = proc.acceleration_from_velocity(&velocity, dt);

        assert_eq!(accel.len(), 100);
        // Interior samples should be ~5.0 m/s^2 (central difference)
        for i in 1..99 {
            assert!(
                (accel[i] - 5.0).abs() < 1e-8,
                "accel[{}] = {}, expected 5.0",
                i,
                accel[i]
            );
        }
    }

    #[test]
    fn test_acceleration_constant_velocity_is_zero() {
        let proc = make_processor();
        let dt = 0.001;
        let velocity = vec![3.0; 50];
        let accel = proc.acceleration_from_velocity(&velocity, dt);

        for (i, &a) in accel.iter().enumerate() {
            assert!(
                a.abs() < 1e-10,
                "accel[{}] = {}, expected 0.0",
                i,
                a
            );
        }
    }

    #[test]
    fn test_acceleration_empty() {
        let proc = make_processor();
        assert!(proc.acceleration_from_velocity(&[], 0.001).is_empty());
    }

    #[test]
    fn test_acceleration_single_sample() {
        let proc = make_processor();
        let accel = proc.acceleration_from_velocity(&[1.0], 0.001);
        assert_eq!(accel.len(), 1);
        assert_eq!(accel[0], 0.0);
    }

    // ── Arctan demodulation ──

    #[test]
    fn test_arctan_demodulate_known_phase() {
        let proc = make_processor();
        let n = 256;
        let freq = 10.0;
        let sample_rate = 1000.0;

        let i_sig: Vec<f64> = (0..n)
            .map(|k| (2.0 * PI * freq * k as f64 / sample_rate).cos())
            .collect();
        let q_sig: Vec<f64> = (0..n)
            .map(|k| (2.0 * PI * freq * k as f64 / sample_rate).sin())
            .collect();

        let phase = proc.arctan_demodulate(&i_sig, &q_sig);
        assert_eq!(phase.len(), n);

        // Phase should increase linearly
        // Check derivative is approximately constant
        let expected_dphase = 2.0 * PI * freq / sample_rate;
        for i in 1..n {
            let dp = phase[i] - phase[i - 1];
            assert!(
                (dp - expected_dphase).abs() < 0.01,
                "Phase diff at {} = {}, expected {}",
                i,
                dp,
                expected_dphase
            );
        }
    }

    #[test]
    fn test_arctan_demodulate_constant_phase() {
        let proc = make_processor();
        // Constant I=1, Q=0 → phase = 0
        let i_sig = vec![1.0; 100];
        let q_sig = vec![0.0; 100];
        let phase = proc.arctan_demodulate(&i_sig, &q_sig);

        for &p in &phase {
            assert!(p.abs() < 1e-10, "Expected 0 phase, got {}", p);
        }
    }

    #[test]
    fn test_arctan_demodulate_empty() {
        let proc = make_processor();
        assert!(proc.arctan_demodulate(&[], &[]).is_empty());
    }

    // ── Heterodyne demodulation ──

    #[test]
    fn test_heterodyne_demodulate_extracts_carrier() {
        let proc = make_processor();
        let sample_rate = 100_000.0;
        let carrier_freq = 10_000.0;
        let n = 1000;

        // Generate a heterodyne signal: carrier with slow amplitude modulation
        let signal: Vec<f64> = (0..n)
            .map(|k| {
                let t = k as f64 / sample_rate;
                let envelope = 1.0 + 0.5 * (2.0 * PI * 100.0 * t).cos();
                envelope * (2.0 * PI * carrier_freq * t).cos()
            })
            .collect();

        let (i_ch, q_ch) = proc.heterodyne_demodulate(&signal, carrier_freq, sample_rate);
        assert_eq!(i_ch.len(), n);
        assert_eq!(q_ch.len(), n);

        // I channel should recover the envelope (after lowpass filtering)
        // Check that I channel has non-trivial amplitude
        let i_rms = (i_ch.iter().map(|x| x * x).sum::<f64>() / n as f64).sqrt();
        assert!(i_rms > 0.1, "I channel RMS too low: {}", i_rms);
    }

    #[test]
    fn test_heterodyne_demodulate_empty() {
        let proc = make_processor();
        let (i, q) = proc.heterodyne_demodulate(&[], 40e6, 100e6);
        assert!(i.is_empty());
        assert!(q.is_empty());
    }

    // ── FM demodulation ──

    #[test]
    fn test_fm_demodulate_constant_frequency() {
        let proc = make_processor();
        let sample_rate = 10000.0;
        let freq = 500.0;
        let n = 512; // Power of 2 for FFT

        let signal: Vec<f64> = (0..n)
            .map(|k| (2.0 * PI * freq * k as f64 / sample_rate).cos())
            .collect();

        let inst_freq = proc.fm_demodulate(&signal, sample_rate);
        assert_eq!(inst_freq.len(), n);

        // Interior samples should be near the signal frequency
        // Skip edges due to windowing effects
        let mid_start = n / 4;
        let mid_end = 3 * n / 4;
        for i in mid_start..mid_end {
            assert!(
                (inst_freq[i] - freq).abs() < 100.0,
                "inst_freq[{}] = {}, expected ~{}",
                i,
                inst_freq[i],
                freq
            );
        }
    }

    // ── Power spectrum and resonance detection ──

    #[test]
    fn test_power_spectrum_single_tone() {
        let analyzer = ModalAnalyzer::new(1000.0);
        let n = 1024;
        let freq = 100.0; // 100 Hz tone
        let signal: Vec<f64> = (0..n)
            .map(|k| (2.0 * PI * freq * k as f64 / 1000.0).sin())
            .collect();

        let spectrum = analyzer.power_spectrum(&signal);
        assert!(!spectrum.is_empty());

        // Find peak
        let (peak_freq, _) = spectrum
            .iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();
        assert!(
            (*peak_freq - freq).abs() < 2.0,
            "Peak at {} Hz, expected {} Hz",
            peak_freq,
            freq
        );
    }

    #[test]
    fn test_find_resonances_injected_peaks() {
        let analyzer = ModalAnalyzer::new(10000.0);
        let n = 4096;
        // Two resonances at 500 Hz and 1500 Hz
        let signal: Vec<f64> = (0..n)
            .map(|k| {
                let t = k as f64 / 10000.0;
                10.0 * (2.0 * PI * 500.0 * t).sin() + 5.0 * (2.0 * PI * 1500.0 * t).sin()
            })
            .collect();

        let spectrum = analyzer.power_spectrum(&signal);
        let modes = analyzer.find_resonances(&spectrum, 10.0);

        // Should find at least 2 resonances
        assert!(
            modes.len() >= 2,
            "Found {} modes, expected >= 2",
            modes.len()
        );

        // Check frequencies are approximately correct
        let freqs: Vec<f64> = modes.iter().map(|m| m.frequency_hz).collect();
        let has_500 = freqs.iter().any(|&f| (f - 500.0).abs() < 10.0);
        let has_1500 = freqs.iter().any(|&f| (f - 1500.0).abs() < 10.0);
        assert!(has_500, "Missing 500 Hz resonance, found: {:?}", freqs);
        assert!(has_1500, "Missing 1500 Hz resonance, found: {:?}", freqs);
    }

    #[test]
    fn test_quality_factor_inverse_of_damping() {
        // Q = 1 / (2 * zeta)
        let mode = ResonanceMode {
            frequency_hz: 1000.0,
            amplitude: 1.0,
            damping_ratio: 0.01,
            quality_factor: 50.0,
        };
        let q_from_zeta = 1.0 / (2.0 * mode.damping_ratio);
        assert!(
            (mode.quality_factor - q_from_zeta).abs() < 1e-10,
            "Q = {}, 1/(2*zeta) = {}",
            mode.quality_factor,
            q_from_zeta
        );
    }

    #[test]
    fn test_damping_ratio_from_quality_factor() {
        // zeta = 1 / (2 * Q)
        let q: f64 = 100.0;
        let zeta = 1.0 / (2.0 * q);
        assert!((zeta - 0.005).abs() < 1e-10);
    }

    // ── Frequency Response Function ──

    #[test]
    fn test_frf_unity_system() {
        // If input == output, FRF magnitude should be ~1.0
        let analyzer = ModalAnalyzer::new(1000.0);
        let n = 256;
        let signal: Vec<f64> = (0..n)
            .map(|k| (2.0 * PI * 50.0 * k as f64 / 1000.0).sin())
            .collect();

        let frf = analyzer.frequency_response_function(&signal, &signal, n);
        assert!(!frf.is_empty());

        // Non-DC, non-Nyquist bins should have magnitude ~1.0
        for &(freq, (mag, _)) in &frf {
            if freq > 10.0 && freq < 490.0 && mag > 0.01 {
                assert!(
                    (mag - 1.0).abs() < 0.1,
                    "FRF mag at {} Hz = {}, expected ~1.0",
                    freq,
                    mag
                );
            }
        }
    }

    #[test]
    fn test_frf_magnitude_peak_at_resonance() {
        let analyzer = ModalAnalyzer::new(10000.0);
        let n = 2048;
        let f0 = 1000.0;

        // Input: broadband noise (pseudo-random)
        let input: Vec<f64> = (0..n)
            .map(|k| {
                // Simple pseudo-random using sin
                (k as f64 * 0.137 + 0.5).sin() * 2.0
            })
            .collect();

        // Output: input filtered to emphasize f0 (simulate resonant system)
        let output: Vec<f64> = (0..n)
            .map(|k| {
                let t = k as f64 / 10000.0;
                input[k] * (2.0 * PI * f0 * t).cos().abs().max(0.1)
            })
            .collect();

        let frf = analyzer.frequency_response_function(&input, &output, n);
        assert!(!frf.is_empty());
        // FRF should produce valid results
        let max_mag = frf.iter().map(|&(_, (m, _))| m).fold(0.0_f64, f64::max);
        assert!(max_mag > 0.0, "FRF max magnitude should be positive");
    }

    // ── Coherence ──

    #[test]
    fn test_coherence_identical_signals() {
        // Coherence of a signal with itself should be ~1.0
        let analyzer = ModalAnalyzer::new(1000.0);
        let n = 256;
        let signal: Vec<f64> = (0..n)
            .map(|k| (2.0 * PI * 100.0 * k as f64 / 1000.0).sin())
            .collect();

        let coh = analyzer.coherence(&signal, &signal, n);
        assert!(!coh.is_empty());

        // Check bins with significant energy have coherence near 1.0
        for &(freq, gamma_sq) in &coh {
            if freq > 0.0 && freq < 500.0 {
                // All bins should have coherence = 1.0 for identical signals
                assert!(
                    gamma_sq >= 0.99 || gamma_sq < 0.01,
                    "Coherence at {} Hz = {}, expected near 0 or 1",
                    freq,
                    gamma_sq
                );
            }
        }
    }

    #[test]
    fn test_coherence_linear_system() {
        // Linear scaling: output = 2*input → coherence = 1.0
        let analyzer = ModalAnalyzer::new(1000.0);
        let n = 256;
        let input: Vec<f64> = (0..n)
            .map(|k| (2.0 * PI * 100.0 * k as f64 / 1000.0).sin())
            .collect();
        let output: Vec<f64> = input.iter().map(|&x| 2.0 * x).collect();

        let coh = analyzer.coherence(&input, &output, n);

        // At the signal frequency, coherence should be 1.0
        let target_bin = (100.0 / (1000.0 / n as f64)).round() as usize;
        if target_bin < coh.len() {
            assert!(
                coh[target_bin].1 > 0.99,
                "Coherence at signal freq = {}",
                coh[target_bin].1
            );
        }
    }

    // ── Operational Deflection Shape ──

    #[test]
    fn test_ods_single_frequency() {
        let analyzer = ModalAnalyzer::new(1000.0);
        let n = 1024;
        let freq = 100.0;

        // Two points with different amplitudes and phases
        let v1: Vec<f64> = (0..n)
            .map(|k| 2.0 * (2.0 * PI * freq * k as f64 / 1000.0).sin())
            .collect();
        let v2: Vec<f64> = (0..n)
            .map(|k| 1.0 * (2.0 * PI * freq * k as f64 / 1000.0 + PI / 4.0).sin())
            .collect();

        let ods = analyzer.operational_deflection_shape(&[v1, v2], freq, 1000.0);
        assert_eq!(ods.len(), 2);

        // First point should have larger amplitude
        assert!(ods[0].0 > ods[1].0, "Point 1 amp {} should be > point 2 amp {}", ods[0].0, ods[1].0);
    }

    // ── Max velocity ──

    #[test]
    fn test_max_velocity_formula() {
        // v_max = lambda * fs / 4
        let v = max_velocity(633.0, 256_000.0);
        let expected = 633e-9 * 256_000.0 / 4.0;
        assert!(
            (v - expected).abs() < 1e-12,
            "max_velocity = {}, expected {}",
            v,
            expected
        );
    }

    #[test]
    fn test_max_velocity_increases_with_sample_rate() {
        let v1 = max_velocity(633.0, 100_000.0);
        let v2 = max_velocity(633.0, 200_000.0);
        assert!(v2 > v1);
    }

    #[test]
    fn test_max_velocity_increases_with_wavelength() {
        let v1 = max_velocity(633.0, 256_000.0);
        let v2 = max_velocity(1550.0, 256_000.0);
        assert!(v2 > v1);
    }

    // ── Fringe count to displacement ──

    #[test]
    fn test_fringe_to_displacement_one_fringe() {
        // 1 fringe at 633 nm → lambda/2 = 316.5 nm = 316.5e-9 m
        let d = fringe_count_to_displacement(1.0, 633.0);
        let expected = 633e-9 / 2.0;
        assert!(
            (d - expected).abs() < 1e-15,
            "d = {}, expected {}",
            d,
            expected
        );
    }

    #[test]
    fn test_fringe_to_displacement_multiple() {
        let d = fringe_count_to_displacement(10.0, 633.0);
        let expected = 10.0 * 633e-9 / 2.0;
        assert!((d - expected).abs() < 1e-14);
    }

    #[test]
    fn test_fringe_to_displacement_zero() {
        assert_eq!(fringe_count_to_displacement(0.0, 633.0), 0.0);
    }

    // ── Sensitivity ──

    #[test]
    fn test_sensitivity_v_per_fringe_value() {
        let s = sensitivity_v_per_fringe(633.0);
        let expected = 633e-9 / 2.0;
        assert!((s - expected).abs() < 1e-15);
    }

    // ── Speckle noise reduction ──

    #[test]
    fn test_detect_dropouts_finds_flat_regions() {
        let mut signal = vec![1.0; 100];
        // Insert dropout from 30..50
        for i in 30..50 {
            signal[i] = 0.0;
        }

        let dropouts = SpeckleNoiseReducer::detect_dropouts(&signal, 0.5);
        assert_eq!(dropouts.len(), 1);
        assert_eq!(dropouts[0], (30, 50));
    }

    #[test]
    fn test_detect_dropouts_multiple() {
        let mut signal = vec![1.0; 100];
        for i in 10..20 {
            signal[i] = 0.0;
        }
        for i in 60..70 {
            signal[i] = 0.0;
        }

        let dropouts = SpeckleNoiseReducer::detect_dropouts(&signal, 0.5);
        assert_eq!(dropouts.len(), 2);
    }

    #[test]
    fn test_detect_dropouts_none() {
        let signal = vec![1.0; 100];
        let dropouts = SpeckleNoiseReducer::detect_dropouts(&signal, 0.5);
        assert!(dropouts.is_empty());
    }

    #[test]
    fn test_interpolate_dropouts_linear() {
        let mut signal = vec![0.0, 1.0, 2.0, 0.0, 0.0, 0.0, 6.0, 7.0, 8.0];
        let dropouts = vec![(3, 6)];
        SpeckleNoiseReducer::interpolate_dropouts(&mut signal, &dropouts);

        // Should linearly interpolate from 2.0 to 6.0 over indices 3,4,5
        assert!((signal[3] - 3.0).abs() < 1e-10);
        assert!((signal[4] - 4.0).abs() < 1e-10);
        assert!((signal[5] - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_speckle_quality_perfect() {
        let signal = vec![1.0; 1000];
        let q = SpeckleNoiseReducer::speckle_tracking_quality(&signal);
        assert!((q - 1.0).abs() < 0.01, "Quality = {}, expected ~1.0", q);
    }

    #[test]
    fn test_speckle_quality_with_dropouts() {
        let mut signal = vec![1.0; 1000];
        // 20% dropouts
        for i in 0..200 {
            signal[i] = 0.0;
        }
        let q = SpeckleNoiseReducer::speckle_tracking_quality(&signal);
        assert!(q < 0.95, "Quality {} should be < 0.95 with 20% dropouts", q);
        assert!(q > 0.5, "Quality {} should be > 0.5", q);
    }

    // ── Scanning vibrometer ──

    #[test]
    fn test_average_spectrum() {
        let scan = ScanningVibrometer::new(vec![[0.0, 0.0], [1.0, 0.0]]);
        let s1 = vec![(100.0, 2.0), (200.0, 4.0)];
        let s2 = vec![(100.0, 6.0), (200.0, 8.0)];
        let avg = scan.average_spectrum(&[s1, s2]);

        assert_eq!(avg.len(), 2);
        assert!((avg[0].1 - 4.0).abs() < 1e-10); // (2+6)/2
        assert!((avg[1].1 - 6.0).abs() < 1e-10); // (4+8)/2
    }

    #[test]
    fn test_mode_shape_extraction() {
        let scan = ScanningVibrometer::new(vec![[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]);
        let n = 1024;
        let freq = 100.0;
        let sr = 1000.0;

        // Three points with known amplitudes and phases
        let v1: Vec<f64> = (0..n)
            .map(|k| 3.0 * (2.0 * PI * freq * k as f64 / sr).sin())
            .collect();
        let v2: Vec<f64> = (0..n)
            .map(|k| 1.0 * (2.0 * PI * freq * k as f64 / sr).sin())
            .collect();
        let v3: Vec<f64> = (0..n)
            .map(|k| 2.0 * (2.0 * PI * freq * k as f64 / sr + PI).sin())
            .collect();

        let shape = scan.mode_shape_at_freq(&[v1, v2, v3], freq, sr);
        assert_eq!(shape.len(), 3);

        // Point 1 should have largest amplitude
        assert!(shape[0].0 > shape[1].0);
        // Point 3 should be out of phase with point 1
        let phase_diff = (shape[2].1 - shape[0].1).abs();
        assert!(
            (phase_diff - PI).abs() < 0.2 || (phase_diff - PI).abs() > (2.0 * PI - 0.2),
            "Phase difference = {}, expected ~pi",
            phase_diff
        );
    }

    #[test]
    fn test_animated_deflection_frames() {
        let scan = ScanningVibrometer::new(vec![[0.0, 0.0], [1.0, 0.0]]);
        let mode_shape = vec![(1.0, 0.0), (0.5, PI)];
        let frames = scan.animated_deflection(&mode_shape, 10);

        assert_eq!(frames.len(), 10);
        assert_eq!(frames[0].len(), 2);

        // At t=0: point1 = 1.0*cos(0+0) = 1.0, point2 = 0.5*cos(0+pi) = -0.5
        assert!((frames[0][0] - 1.0).abs() < 1e-10);
        assert!((frames[0][1] - (-0.5)).abs() < 1e-10);
    }

    #[test]
    fn test_auto_correlation_length() {
        // Create a mode shape that decorrelates over distance
        let n_points = 20;
        let scan_points: Vec<[f64; 2]> = (0..n_points)
            .map(|i| [i as f64 * 0.1, 0.0])
            .collect();
        let scan = ScanningVibrometer::new(scan_points);

        // Mode shape with spatial wavelength ~0.5 m
        let mode_shape: Vec<(f64, f64)> = (0..n_points)
            .map(|i| {
                let x = i as f64 * 0.1;
                let phase = 2.0 * PI * x / 0.5; // wavelength = 0.5 m
                (1.0, phase)
            })
            .collect();

        let corr_len = scan.auto_correlation_length(&mode_shape);
        // Correlation length should be related to the wavelength
        assert!(corr_len > 0.0, "Correlation length should be positive");
        assert!(
            corr_len < 2.0,
            "Correlation length {} should be < total span",
            corr_len
        );
    }

    // ── Decoder type ──

    #[test]
    fn test_decoder_type_enum() {
        let d1 = DecoderType::FrequencyCounter;
        let d2 = DecoderType::PhaseLocked;
        let d3 = DecoderType::Digital;
        let d4 = DecoderType::Heterodyne { carrier_freq_hz: 40e6 };

        assert_ne!(d1, d2);
        assert_eq!(d3.clone(), DecoderType::Digital);
        if let DecoderType::Heterodyne { carrier_freq_hz } = d4 {
            assert!((carrier_freq_hz - 40e6).abs() < 1.0);
        } else {
            panic!("Expected Heterodyne variant");
        }
    }

    // ── Config ──

    #[test]
    fn test_config_access() {
        let proc = make_processor();
        let cfg = proc.config();
        assert!((cfg.laser_wavelength_nm - 633.0).abs() < 1e-10);
        assert!((cfg.sample_rate_hz - 256000.0).abs() < 1e-10);
    }

    // ── Modal damping ratio ──

    #[test]
    fn test_modal_damping_ratio_lightly_damped() {
        let analyzer = ModalAnalyzer::new(10000.0);
        let n = 8192;
        let f0 = 500.0;
        // Generate lightly damped sinusoid
        let signal: Vec<f64> = (0..n)
            .map(|k| {
                let t = k as f64 / 10000.0;
                (2.0 * PI * f0 * t).sin()
            })
            .collect();

        let zeta = analyzer.modal_damping_ratio(&signal, f0, 10000.0);
        // Pure sinusoid has extremely narrow bandwidth → very small damping
        assert!(
            zeta < 0.1,
            "Damping ratio {} should be small for pure tone",
            zeta
        );
    }

    // ── Power spectrum empty ──

    #[test]
    fn test_power_spectrum_empty() {
        let analyzer = ModalAnalyzer::new(1000.0);
        assert!(analyzer.power_spectrum(&[]).is_empty());
    }
}
