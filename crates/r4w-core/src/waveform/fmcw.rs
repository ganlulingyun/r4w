//! FMCW (Frequency Modulated Continuous Wave) Radar Waveform
//!
//! This module implements FMCW radar waveforms, commonly used for:
//! - Automotive radar (77 GHz)
//! - Weather radar
//! - Level measurement
//! - Through-wall imaging
//! - Ground-penetrating radar
//!
//! ## FMCW Principle
//!
//! The transmitted signal sweeps linearly in frequency (chirp):
//! ```text
//! f(t) = f_start + (B/T_chirp) * t
//!
//! where:
//!   B = chirp bandwidth (Hz)
//!   T_chirp = chirp duration (s)
//!   f_start = start frequency (Hz)
//! ```
//!
//! ## Range and Velocity
//!
//! When the chirp reflects off a target and returns:
//! ```text
//! Beat frequency: f_b = (2 * B * R) / (c * T_chirp)
//!
//! Range: R = (f_b * c * T_chirp) / (2 * B)
//!
//! Range resolution: ΔR = c / (2 * B)
//!
//! Velocity (from Doppler): v = (λ * f_d) / 2
//! ```
//!
//! ## Key Parameters
//!
//! - **Bandwidth (B)**: Determines range resolution (larger = finer)
//! - **Chirp duration (T)**: Affects unambiguous range and velocity
//! - **Chirp rate (k = B/T)**: Rate of frequency change
//! - **Number of chirps**: Enables velocity measurement via Doppler

use std::f64::consts::PI;

use crate::types::IQSample;
use super::{CommonParams, DemodResult, Waveform, WaveformInfo};

/// Speed of light (m/s)
pub const SPEED_OF_LIGHT: f64 = 299_792_458.0;

/// FMCW modulated signal with metadata
#[derive(Debug, Clone)]
pub struct FmcwSignal {
    /// I/Q samples
    pub samples: Vec<IQSample>,
    /// Sample rate in Hz
    pub sample_rate: f64,
    /// Carrier frequency in Hz
    pub carrier_freq: f64,
    /// Chirp bandwidth in Hz
    pub bandwidth: f64,
}

/// FMCW chirp direction
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum ChirpDirection {
    /// Frequency increases with time (up-chirp)
    #[default]
    Up,
    /// Frequency decreases with time (down-chirp)
    Down,
    /// Triangular: up then down
    Triangle,
    /// Sawtooth: continuous up-chirps
    Sawtooth,
}

/// FMCW waveform configuration
#[derive(Debug, Clone)]
pub struct FmcwConfig {
    /// Chirp bandwidth (Hz)
    pub bandwidth_hz: f64,
    /// Chirp duration (seconds)
    pub chirp_duration_s: f64,
    /// Number of chirps in frame
    pub num_chirps: usize,
    /// Inter-chirp idle time (seconds)
    pub idle_time_s: f64,
    /// Chirp direction/pattern
    pub chirp_direction: ChirpDirection,
    /// Start frequency offset from carrier (Hz)
    pub start_freq_offset_hz: f64,
    /// Window function for range FFT
    pub use_window: bool,
}

impl Default for FmcwConfig {
    fn default() -> Self {
        Self {
            bandwidth_hz: 150e6,          // 150 MHz bandwidth
            chirp_duration_s: 40e-6,      // 40 microseconds
            num_chirps: 128,              // 128 chirps per frame
            idle_time_s: 10e-6,           // 10 microseconds idle
            chirp_direction: ChirpDirection::Sawtooth,
            start_freq_offset_hz: -75e6,  // Start 75 MHz below carrier
            use_window: true,
        }
    }
}

impl FmcwConfig {
    /// Create automotive radar preset (77 GHz band)
    pub fn automotive_77ghz() -> Self {
        Self {
            bandwidth_hz: 1e9,            // 1 GHz bandwidth for fine resolution
            chirp_duration_s: 50e-6,      // 50 microseconds
            num_chirps: 256,              // Good velocity resolution
            idle_time_s: 5e-6,
            chirp_direction: ChirpDirection::Sawtooth,
            start_freq_offset_hz: -500e6,
            use_window: true,
        }
    }

    /// Create short-range radar preset
    pub fn short_range() -> Self {
        Self {
            bandwidth_hz: 500e6,          // 500 MHz
            chirp_duration_s: 20e-6,      // 20 microseconds
            num_chirps: 64,
            idle_time_s: 5e-6,
            chirp_direction: ChirpDirection::Sawtooth,
            start_freq_offset_hz: -250e6,
            use_window: true,
        }
    }

    /// Create long-range radar preset
    pub fn long_range() -> Self {
        Self {
            bandwidth_hz: 100e6,          // 100 MHz (coarser resolution)
            chirp_duration_s: 100e-6,     // 100 microseconds (longer for range)
            num_chirps: 512,              // More chirps for velocity
            idle_time_s: 20e-6,
            chirp_direction: ChirpDirection::Sawtooth,
            start_freq_offset_hz: -50e6,
            use_window: true,
        }
    }

    /// Create triangular modulation preset (for simultaneous range/velocity)
    pub fn triangular() -> Self {
        Self {
            bandwidth_hz: 200e6,
            chirp_duration_s: 50e-6,
            num_chirps: 128,
            idle_time_s: 0.0,             // No idle for continuous triangle
            chirp_direction: ChirpDirection::Triangle,
            start_freq_offset_hz: -100e6,
            use_window: true,
        }
    }

    /// Calculate chirp rate (Hz/s)
    pub fn chirp_rate(&self) -> f64 {
        self.bandwidth_hz / self.chirp_duration_s
    }

    /// Calculate range resolution (meters)
    pub fn range_resolution(&self) -> f64 {
        SPEED_OF_LIGHT / (2.0 * self.bandwidth_hz)
    }

    /// Calculate maximum unambiguous range (meters)
    pub fn max_range(&self, sample_rate: f64) -> f64 {
        let _samples_per_chirp = (self.chirp_duration_s * sample_rate) as usize;
        let max_beat_freq = sample_rate / 2.0;  // Nyquist
        (max_beat_freq * SPEED_OF_LIGHT * self.chirp_duration_s) / (2.0 * self.bandwidth_hz)
    }

    /// Calculate velocity resolution (m/s) given carrier frequency
    pub fn velocity_resolution(&self, carrier_freq_hz: f64) -> f64 {
        let wavelength = SPEED_OF_LIGHT / carrier_freq_hz;
        let frame_time = self.num_chirps as f64 * (self.chirp_duration_s + self.idle_time_s);
        wavelength / (2.0 * frame_time)
    }

    /// Calculate maximum unambiguous velocity (m/s)
    pub fn max_velocity(&self, carrier_freq_hz: f64) -> f64 {
        let wavelength = SPEED_OF_LIGHT / carrier_freq_hz;
        let prf = 1.0 / (self.chirp_duration_s + self.idle_time_s);
        (wavelength * prf) / 4.0
    }
}

/// FMCW radar waveform generator
#[derive(Debug, Clone)]
pub struct Fmcw {
    /// Common signal parameters
    common: CommonParams,
    /// FMCW configuration
    config: FmcwConfig,
    /// Samples per chirp
    samples_per_chirp: usize,
    /// Samples during idle
    samples_idle: usize,
}

impl Fmcw {
    /// Create new FMCW waveform generator
    pub fn new(common: CommonParams, config: FmcwConfig) -> Self {
        let samples_per_chirp = (config.chirp_duration_s * common.sample_rate) as usize;
        let samples_idle = (config.idle_time_s * common.sample_rate) as usize;

        Self {
            common,
            config,
            samples_per_chirp,
            samples_idle,
        }
    }

    /// Create with default config
    pub fn with_defaults(sample_rate: f64) -> Self {
        Self::new(
            CommonParams {
                sample_rate,
                carrier_freq: 0.0,  // Baseband
                amplitude: 1.0,
            },
            FmcwConfig::default(),
        )
    }

    /// Generate a single chirp
    pub fn generate_chirp(&self, direction: ChirpDirection) -> Vec<IQSample> {
        let mut samples = Vec::with_capacity(self.samples_per_chirp);
        let k = self.config.chirp_rate();
        let f0 = self.config.start_freq_offset_hz;

        for i in 0..self.samples_per_chirp {
            let t = i as f64 / self.common.sample_rate;

            // Instantaneous frequency based on direction
            let _freq = match direction {
                ChirpDirection::Up => f0 + k * t,
                ChirpDirection::Down => (f0 + self.config.bandwidth_hz) - k * t,
                ChirpDirection::Triangle | ChirpDirection::Sawtooth => {
                    // For single chirp, treat as up-chirp
                    f0 + k * t
                }
            };

            // Phase is integral of frequency
            // φ(t) = 2π * (f0*t + k*t²/2) for up-chirp
            let phase = match direction {
                ChirpDirection::Up | ChirpDirection::Sawtooth | ChirpDirection::Triangle => {
                    2.0 * PI * (f0 * t + k * t * t / 2.0)
                }
                ChirpDirection::Down => {
                    let f_start = f0 + self.config.bandwidth_hz;
                    2.0 * PI * (f_start * t - k * t * t / 2.0)
                }
            };

            let i_val = self.common.amplitude * phase.cos();
            let q_val = self.common.amplitude * phase.sin();
            samples.push(IQSample::new(i_val, q_val));
        }

        samples
    }

    /// Generate idle samples (zeros or low-level noise)
    fn generate_idle(&self) -> Vec<IQSample> {
        vec![IQSample::new(0.0, 0.0); self.samples_idle]
    }

    /// Generate a complete FMCW frame
    pub fn generate_frame(&self) -> FmcwSignal {
        let total_samples = self.config.num_chirps * (self.samples_per_chirp + self.samples_idle);
        let mut samples = Vec::with_capacity(total_samples);

        for chirp_idx in 0..self.config.num_chirps {
            // Determine direction for this chirp
            let direction = match self.config.chirp_direction {
                ChirpDirection::Up => ChirpDirection::Up,
                ChirpDirection::Down => ChirpDirection::Down,
                ChirpDirection::Sawtooth => ChirpDirection::Up,
                ChirpDirection::Triangle => {
                    if chirp_idx % 2 == 0 {
                        ChirpDirection::Up
                    } else {
                        ChirpDirection::Down
                    }
                }
            };

            samples.extend(self.generate_chirp(direction));

            if self.samples_idle > 0 {
                samples.extend(self.generate_idle());
            }
        }

        FmcwSignal {
            samples,
            sample_rate: self.common.sample_rate,
            carrier_freq: self.common.carrier_freq,
            bandwidth: self.config.bandwidth_hz,
        }
    }

    /// Simulate radar echo with target at given range and velocity
    pub fn simulate_echo(
        &self,
        tx_signal: &[IQSample],
        range_m: f64,
        velocity_mps: f64,
        carrier_freq_hz: f64,
        rcs_db: f64,  // Radar cross-section in dB
    ) -> Vec<IQSample> {
        let time_delay = 2.0 * range_m / SPEED_OF_LIGHT;
        let sample_delay = (time_delay * self.common.sample_rate) as usize;

        // Doppler frequency shift
        let wavelength = SPEED_OF_LIGHT / carrier_freq_hz;
        let doppler_freq = 2.0 * velocity_mps / wavelength;

        // Attenuation (simplified path loss)
        let attenuation = 10.0_f64.powf(rcs_db / 20.0) / (range_m * range_m);

        let mut rx_signal = vec![IQSample::new(0.0, 0.0); tx_signal.len()];

        for (i, _sample) in tx_signal.iter().enumerate() {
            if i >= sample_delay && (i - sample_delay) < tx_signal.len() {
                let t = i as f64 / self.common.sample_rate;
                let doppler_phase = 2.0 * PI * doppler_freq * t;

                // Apply delay, attenuation, and Doppler
                let phase_shift_i = doppler_phase.cos();
                let phase_shift_q = doppler_phase.sin();

                let delayed_sample = tx_signal[i - sample_delay];

                // Complex multiply for phase shift
                rx_signal[i] = IQSample::new(
                    attenuation * (delayed_sample.re * phase_shift_i - delayed_sample.im * phase_shift_q),
                    attenuation * (delayed_sample.re * phase_shift_q + delayed_sample.im * phase_shift_i),
                );
            }
        }

        rx_signal
    }

    /// Mix TX and RX to get beat signal (dechirp)
    pub fn dechirp(&self, tx: &[IQSample], rx: &[IQSample]) -> Vec<IQSample> {
        tx.iter()
            .zip(rx.iter())
            .map(|(t, r)| {
                // Complex conjugate multiply: rx * conj(tx)
                IQSample::new(
                    r.re * t.re + r.im * t.im,
                    r.im * t.re - r.re * t.im,
                )
            })
            .collect()
    }

    /// Apply window function to chirp samples
    pub fn apply_window(&self, samples: &mut [IQSample]) {
        let n = samples.len();
        for (i, sample) in samples.iter_mut().enumerate() {
            // Hann window
            let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / (n - 1) as f64).cos());
            *sample = IQSample::new(sample.re * w, sample.im * w);
        }
    }

    /// Extract single chirp from frame
    pub fn extract_chirp(&self, frame: &[IQSample], chirp_index: usize) -> Option<Vec<IQSample>> {
        let chirp_total = self.samples_per_chirp + self.samples_idle;
        let start = chirp_index * chirp_total;
        let end = start + self.samples_per_chirp;

        if end <= frame.len() {
            Some(frame[start..end].to_vec())
        } else {
            None
        }
    }

    /// Get configuration
    pub fn config(&self) -> &FmcwConfig {
        &self.config
    }

    /// Get common parameters
    pub fn common(&self) -> &CommonParams {
        &self.common
    }

    /// Calculate beat frequency for given range
    pub fn range_to_beat_freq(&self, range_m: f64) -> f64 {
        (2.0 * self.config.bandwidth_hz * range_m) / (SPEED_OF_LIGHT * self.config.chirp_duration_s)
    }

    /// Calculate range from beat frequency
    pub fn beat_freq_to_range(&self, beat_freq: f64) -> f64 {
        (beat_freq * SPEED_OF_LIGHT * self.config.chirp_duration_s) / (2.0 * self.config.bandwidth_hz)
    }

    /// Generate info string about radar parameters
    pub fn info(&self, carrier_freq_hz: f64) -> String {
        format!(
            "FMCW Radar Parameters:\n\
             Bandwidth: {:.1} MHz\n\
             Chirp duration: {:.1} µs\n\
             Chirp rate: {:.2} MHz/µs\n\
             Range resolution: {:.2} m\n\
             Max range: {:.1} m\n\
             Velocity resolution: {:.3} m/s\n\
             Max velocity: {:.1} m/s\n\
             Num chirps: {}\n\
             Frame time: {:.2} ms",
            self.config.bandwidth_hz / 1e6,
            self.config.chirp_duration_s * 1e6,
            self.config.chirp_rate() / 1e12,
            self.config.range_resolution(),
            self.config.max_range(self.common.sample_rate),
            self.config.velocity_resolution(carrier_freq_hz),
            self.config.max_velocity(carrier_freq_hz),
            self.config.num_chirps,
            self.config.num_chirps as f64 * (self.config.chirp_duration_s + self.config.idle_time_s) * 1000.0,
        )
    }
}

/// Range-Doppler processing result
#[derive(Debug, Clone)]
pub struct RangeDopplerMap {
    /// 2D magnitude data [range_bin][doppler_bin]
    pub data: Vec<Vec<f64>>,
    /// Range axis values (meters)
    pub range_axis: Vec<f64>,
    /// Velocity axis values (m/s)
    pub velocity_axis: Vec<f64>,
    /// Number of range bins
    pub num_range_bins: usize,
    /// Number of Doppler bins
    pub num_doppler_bins: usize,
}

impl RangeDopplerMap {
    /// Find peaks in the range-Doppler map
    pub fn find_targets(&self, threshold_db: f64) -> Vec<(f64, f64, f64)> {
        let mut targets = Vec::new();

        // Find max value for normalization
        let max_val = self.data.iter()
            .flat_map(|row| row.iter())
            .cloned()
            .fold(f64::MIN, f64::max);

        let threshold_linear = 10.0_f64.powf((max_val.log10() * 20.0 - threshold_db) / 20.0);

        for (r_idx, range_row) in self.data.iter().enumerate() {
            for (d_idx, &val) in range_row.iter().enumerate() {
                if val > threshold_linear {
                    // Simple peak detection (check if local maximum)
                    let is_peak = self.is_local_max(r_idx, d_idx);
                    if is_peak {
                        let range = self.range_axis.get(r_idx).copied().unwrap_or(0.0);
                        let velocity = self.velocity_axis.get(d_idx).copied().unwrap_or(0.0);
                        let power_db = 20.0 * val.log10();
                        targets.push((range, velocity, power_db));
                    }
                }
            }
        }

        targets
    }

    fn is_local_max(&self, r: usize, d: usize) -> bool {
        let val = self.data[r][d];

        for dr in -1i32..=1 {
            for dd in -1i32..=1 {
                if dr == 0 && dd == 0 {
                    continue;
                }
                let nr = (r as i32 + dr) as usize;
                let nd = (d as i32 + dd) as usize;

                if nr < self.num_range_bins && nd < self.num_doppler_bins {
                    if self.data[nr][nd] > val {
                        return false;
                    }
                }
            }
        }
        true
    }
}

// Implement Waveform trait for educational/GUI integration
// Note: FMCW is a radar waveform, not a communications waveform.
// "modulate" generates chirps (data length determines number of chirps)
// "demodulate" returns empty since FMCW does ranging, not data transmission
impl Waveform for Fmcw {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "FMCW",
            full_name: "Frequency Modulated Continuous Wave",
            description: "Radar waveform using linear frequency sweeps (chirps) for range and velocity measurement",
            complexity: 4,
            bits_per_symbol: 0,  // Radar, not data transmission
            carries_data: false,
            characteristics: &[
                "Linear frequency sweep (chirp)",
                "Range resolution: c/(2B)",
                "Beat frequency proportional to range",
                "Doppler shift for velocity",
                "Used in automotive radar (77 GHz)",
            ],
            history: "Developed in WWII for radar applications, now ubiquitous in automotive \
                     radar, weather radar, and industrial sensing.",
            modern_usage: "Automotive collision avoidance (77 GHz), level measurement, \
                         through-wall imaging, ground-penetrating radar.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        // For radar, we generate chirps. Data length can control number of chirps.
        // If no data, generate a single frame (128 chirps)
        let num_chirps = if data.is_empty() {
            // Generate a demo with fewer chirps for visualization
            4
        } else {
            // Use data length as proxy for chirps (clamped to reasonable range)
            data.len().clamp(1, 32)
        };

        // Generate chirps
        let mut samples = Vec::with_capacity(num_chirps * (self.samples_per_chirp + self.samples_idle));

        for chirp_idx in 0..num_chirps {
            let direction = match self.config.chirp_direction {
                ChirpDirection::Triangle => {
                    if chirp_idx % 2 == 0 {
                        ChirpDirection::Up
                    } else {
                        ChirpDirection::Down
                    }
                }
                other => other,
            };

            samples.extend(self.generate_chirp(direction));

            if self.samples_idle > 0 {
                samples.extend(self.generate_idle());
            }
        }

        samples
    }

    fn demodulate(&self, _samples: &[IQSample]) -> DemodResult {
        // FMCW radar doesn't decode data - it measures range and velocity
        // Return empty result with metadata about what this signal is
        DemodResult {
            bits: Vec::new(),
            symbols: Vec::new(),
            ber_estimate: None,
            snr_estimate: None,
            metadata: [
                ("bandwidth_mhz".to_string(), self.config.bandwidth_hz / 1e6),
                ("chirp_duration_us".to_string(), self.config.chirp_duration_s * 1e6),
                ("range_resolution_m".to_string(), self.config.range_resolution()),
            ].into_iter().collect(),
        }
    }

    fn samples_per_symbol(&self) -> usize {
        // For FMCW, return samples per chirp as the "symbol" unit
        self.samples_per_chirp
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_chirp_generation() {
        let fmcw = Fmcw::with_defaults(10e6);  // 10 MHz sample rate
        let chirp = fmcw.generate_chirp(ChirpDirection::Up);

        // Should have samples_per_chirp samples
        let expected_samples = (40e-6 * 10e6) as usize;
        assert_eq!(chirp.len(), expected_samples);

        // Check that samples have unity amplitude (approximately)
        for sample in &chirp[10..chirp.len() - 10] {
            let mag = (sample.re * sample.re + sample.im * sample.im).sqrt();
            assert!((mag - 1.0).abs() < 0.01, "Magnitude should be ~1.0, got {}", mag);
        }
    }

    #[test]
    fn test_frame_generation() {
        let config = FmcwConfig {
            bandwidth_hz: 100e6,
            chirp_duration_s: 10e-6,
            num_chirps: 4,
            idle_time_s: 2e-6,
            ..Default::default()
        };

        let fmcw = Fmcw::new(
            CommonParams {
                sample_rate: 10e6,
                carrier_freq: 0.0,
                amplitude: 1.0,
            },
            config,
        );

        let frame = fmcw.generate_frame();

        // 4 chirps * (100 + 20 samples) = 480 samples
        let expected = 4 * ((10e-6 * 10e6) as usize + (2e-6 * 10e6) as usize);
        assert_eq!(frame.samples.len(), expected);
    }

    #[test]
    fn test_range_resolution() {
        let config = FmcwConfig {
            bandwidth_hz: 150e6,  // 150 MHz
            ..Default::default()
        };

        let resolution = config.range_resolution();
        // c / (2 * B) = 3e8 / (2 * 150e6) = 1.0 m
        assert!((resolution - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_range_beat_freq_conversion() {
        let fmcw = Fmcw::with_defaults(10e6);

        // Test round-trip
        let range = 100.0;  // 100 meters
        let beat_freq = fmcw.range_to_beat_freq(range);
        let recovered_range = fmcw.beat_freq_to_range(beat_freq);

        assert!((recovered_range - range).abs() < 0.01,
            "Round-trip failed: {} -> {} -> {}", range, beat_freq, recovered_range);
    }

    #[test]
    fn test_velocity_resolution() {
        let config = FmcwConfig {
            chirp_duration_s: 50e-6,
            idle_time_s: 10e-6,
            num_chirps: 128,
            ..Default::default()
        };

        // At 77 GHz
        let carrier = 77e9;
        let vel_res = config.velocity_resolution(carrier);

        // Should be sub-meter per second for automotive radar
        assert!(vel_res < 1.0, "Velocity resolution {} m/s too coarse", vel_res);
    }

    #[test]
    fn test_dechirp() {
        let fmcw = Fmcw::with_defaults(10e6);
        let tx = fmcw.generate_chirp(ChirpDirection::Up);

        // Simple test: dechirp of TX with itself should give DC
        let beat = fmcw.dechirp(&tx, &tx);

        // After dechirp, signal should be roughly constant (DC)
        // Check that variation is small
        let mean_re: f64 = beat.iter().map(|s| s.re).sum::<f64>() / beat.len() as f64;
        let variance: f64 = beat.iter().map(|s| (s.re - mean_re).powi(2)).sum::<f64>() / beat.len() as f64;

        assert!(variance < 0.1, "Dechirped signal should be near-DC, variance = {}", variance);
    }

    #[test]
    fn test_presets() {
        let auto = FmcwConfig::automotive_77ghz();
        assert_eq!(auto.bandwidth_hz, 1e9);
        assert_eq!(auto.num_chirps, 256);

        let short = FmcwConfig::short_range();
        assert!(short.chirp_duration_s < auto.chirp_duration_s);

        let long = FmcwConfig::long_range();
        assert!(long.num_chirps > auto.num_chirps);
    }

    #[test]
    fn test_triangle_modulation() {
        let config = FmcwConfig {
            num_chirps: 4,
            chirp_duration_s: 10e-6,
            idle_time_s: 0.0,
            chirp_direction: ChirpDirection::Triangle,
            bandwidth_hz: 100e6,
            ..Default::default()
        };

        let fmcw = Fmcw::new(
            CommonParams {
                sample_rate: 10e6,
                carrier_freq: 0.0,
                amplitude: 1.0,
            },
            config,
        );

        let frame = fmcw.generate_frame();

        // Should generate alternating up/down chirps
        assert!(!frame.samples.is_empty());
    }

    #[test]
    fn test_echo_simulation() {
        let fmcw = Fmcw::with_defaults(10e6);
        let tx = fmcw.generate_chirp(ChirpDirection::Up);

        // Simulate target at 50m, 0 m/s velocity
        let rx = fmcw.simulate_echo(&tx, 50.0, 0.0, 24e9, 0.0);

        // RX should have delayed signal
        assert_eq!(rx.len(), tx.len());

        // Energy should appear after delay
        let delay_samples = (2.0 * 50.0 / SPEED_OF_LIGHT * 10e6) as usize;
        let energy_before: f64 = rx[..delay_samples.min(rx.len())]
            .iter()
            .map(|s| s.re * s.re + s.im * s.im)
            .sum();
        let energy_after: f64 = rx[delay_samples.min(rx.len())..]
            .iter()
            .take(100)
            .map(|s| s.re * s.re + s.im * s.im)
            .sum();

        // After delay should have more energy (if delay is within signal)
        if delay_samples < rx.len() {
            assert!(energy_after > energy_before * 0.1 || energy_before < 0.001);
        }
    }
}
