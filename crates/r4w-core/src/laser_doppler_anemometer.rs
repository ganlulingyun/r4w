//! Laser Doppler Anemometry (LDA/LDV) signal processing for non-intrusive fluid velocity
//! measurement.
//!
//! Laser Doppler Anemometry uses the interference fringe pattern formed at the crossing
//! of two coherent laser beams to measure the velocity of seed particles in a fluid flow.
//! As a particle crosses the measurement volume it scatters light whose frequency is
//! Doppler-shifted proportionally to the component of velocity perpendicular to the
//! fringes.
//!
//! # Physics
//!
//! Two beams of wavelength lambda cross at half-angle theta. The resulting fringe spacing is
//!
//! ```text
//! d_f = lambda / (2 * sin(theta))
//! ```
//!
//! A particle crossing the fringes at velocity v produces a Doppler frequency
//!
//! ```text
//! f_D = v / d_f = 2 * v * sin(theta) / lambda
//! ```
//!
//! The detected signal is a Gaussian-enveloped cosine burst ("Doppler burst"):
//!
//! ```text
//! s(t) = A * exp(-2*(t - t0)^2 / tau^2) * cos(2*pi*f_D*t + phi) + noise
//! ```
//!
//! # Frequency shifting (Bragg cell)
//!
//! A Bragg cell applies a frequency shift f_shift (typically 40 MHz) to one beam so that
//! the fringe pattern moves at a known velocity. This resolves flow direction ambiguity
//! and allows measurement of negative (reverse-flow) velocities.
//!
//! # Features
//!
//! - [`LdaConfig`] — optical geometry and derived fringe parameters
//! - [`MeasurementVolume`] — ellipsoidal probe volume dimensions and fringe count
//! - [`BurstGenerator`] — synthetic Doppler burst signals for testing
//! - [`BurstDetector`] — threshold-based burst identification in continuous signals
//! - [`FrequencyEstimator`] — FFT peak, zero-crossing, and autocorrelation methods
//! - [`BraggShifter`] — frequency-shift processing for directional measurement
//! - [`VelocityProcessor`] — ensemble statistics, bias correction, turbulence metrics
//! - [`TwoComponentLda`] — simultaneous u,v measurement with coincidence windowing
//!
//! # Example
//!
//! ```
//! use r4w_core::laser_doppler_anemometer::{LdaConfig, BurstGenerator, FrequencyEstimator};
//!
//! let config = LdaConfig::new(532.0, 5.0, 200.0, 50.0, 0.1);
//! let gen = BurstGenerator::new(&config, 48000.0);
//! let burst = gen.generate(3.5, 0.002, 0.0, 1.0, 0.0);
//! let est = FrequencyEstimator::new(48000.0);
//! let freq = est.fft_peak(&burst, 1024);
//! let vel = config.velocity_from_frequency(freq);
//! assert!((vel - 3.5).abs() < 0.5);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Optical configuration of a single-component LDA system.
#[derive(Debug, Clone)]
pub struct LdaConfig {
    /// Laser wavelength in nanometres.
    pub wavelength_nm: f64,
    /// Half-angle between the two beams in degrees.
    pub beam_angle_deg: f64,
    /// Lens focal length in millimetres.
    pub focal_length_mm: f64,
    /// Distance between the two parallel beams at the lens, in millimetres.
    pub beam_separation_mm: f64,
    /// 1/e^2 beam diameter at the measurement volume in millimetres.
    pub beam_diameter_mm: f64,
}

impl LdaConfig {
    /// Create a new LDA configuration.
    ///
    /// * `wavelength_nm` - Laser wavelength (nm)
    /// * `beam_angle_deg` - Half-angle between beams (degrees)
    /// * `focal_length_mm` - Focal length (mm)
    /// * `beam_separation_mm` - Beam separation at lens (mm)
    /// * `beam_diameter_mm` - 1/e^2 beam diameter at measurement volume (mm)
    pub fn new(
        wavelength_nm: f64,
        beam_angle_deg: f64,
        focal_length_mm: f64,
        beam_separation_mm: f64,
        beam_diameter_mm: f64,
    ) -> Self {
        Self {
            wavelength_nm,
            beam_angle_deg,
            focal_length_mm,
            beam_separation_mm,
            beam_diameter_mm,
        }
    }

    /// Half-angle in radians.
    pub fn half_angle_rad(&self) -> f64 {
        self.beam_angle_deg * PI / 180.0
    }

    /// Wavelength in metres.
    pub fn wavelength_m(&self) -> f64 {
        self.wavelength_nm * 1e-9
    }

    /// Fringe spacing d_f = lambda / (2 * sin(theta)).
    pub fn fringe_spacing(&self) -> f64 {
        self.wavelength_m() / (2.0 * self.half_angle_rad().sin())
    }

    /// Doppler frequency for a given velocity: f_D = v / d_f.
    pub fn doppler_frequency(&self, velocity: f64) -> f64 {
        velocity / self.fringe_spacing()
    }

    /// Velocity from a measured Doppler frequency: v = f_D * d_f.
    pub fn velocity_from_frequency(&self, freq: f64) -> f64 {
        freq * self.fringe_spacing()
    }

    /// Return the measurement volume geometry.
    pub fn measurement_volume(&self) -> MeasurementVolume {
        MeasurementVolume::from_config(self)
    }
}

// ---------------------------------------------------------------------------
// Measurement volume
// ---------------------------------------------------------------------------

/// Ellipsoidal measurement volume formed at the beam crossing point.
#[derive(Debug, Clone)]
pub struct MeasurementVolume {
    /// Dimension along beam bisector (flow direction) in mm.
    pub dx_mm: f64,
    /// Dimension perpendicular to both beams in mm (= beam diameter).
    pub dy_mm: f64,
    /// Dimension along beam axis in mm.
    pub dz_mm: f64,
    /// Number of fringes in the measurement volume.
    pub num_fringes: f64,
    /// Volume in mm^3 (prolate ellipsoid: pi/6 * dx * dy * dz).
    pub volume_mm3: f64,
}

impl MeasurementVolume {
    /// Compute measurement volume from an [`LdaConfig`].
    pub fn from_config(cfg: &LdaConfig) -> Self {
        let theta = cfg.half_angle_rad();
        let d_beam = cfg.beam_diameter_mm;
        let dx = d_beam / theta.cos();
        let dy = d_beam;
        let dz = d_beam / theta.sin();
        let fringe_spacing_mm = cfg.fringe_spacing() * 1e3; // m -> mm
        let n_f = dx / fringe_spacing_mm;
        let vol = PI / 6.0 * dx * dy * dz;
        Self {
            dx_mm: dx,
            dy_mm: dy,
            dz_mm: dz,
            num_fringes: n_f,
            volume_mm3: vol,
        }
    }
}

// ---------------------------------------------------------------------------
// Burst generator
// ---------------------------------------------------------------------------

/// Generate synthetic Doppler burst signals for algorithm testing.
#[derive(Debug, Clone)]
pub struct BurstGenerator {
    fringe_spacing: f64,
    sample_rate: f64,
}

impl BurstGenerator {
    /// Create a new burst generator.
    ///
    /// * `config` - LDA optical configuration
    /// * `sample_rate` - Sampling rate in Hz
    pub fn new(config: &LdaConfig, sample_rate: f64) -> Self {
        Self {
            fringe_spacing: config.fringe_spacing(),
            sample_rate,
        }
    }

    /// Generate a single Doppler burst.
    ///
    /// * `velocity` - Particle velocity in m/s
    /// * `duration` - Total signal duration in seconds
    /// * `t0` - Centre of the Gaussian envelope (seconds from start)
    /// * `amplitude` - Peak amplitude
    /// * `noise_std` - Standard deviation of additive Gaussian noise
    ///
    /// Returns a vector of samples.
    pub fn generate(
        &self,
        velocity: f64,
        duration: f64,
        t0: f64,
        amplitude: f64,
        noise_std: f64,
    ) -> Vec<f64> {
        let n = (duration * self.sample_rate) as usize;
        let f_d = velocity / self.fringe_spacing;
        // Gaussian envelope width: ~1/4 of duration gives a nice burst shape
        let tau = duration / 4.0;
        let dt = 1.0 / self.sample_rate;
        let mut rng = SimpleRng::new(42);
        (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                let envelope = (-2.0 * (t - t0).powi(2) / tau.powi(2)).exp();
                let carrier = (2.0 * PI * f_d * t).cos();
                let noise = if noise_std > 0.0 {
                    rng.gaussian() * noise_std
                } else {
                    0.0
                };
                amplitude * envelope * carrier + noise
            })
            .collect()
    }

    /// Generate a burst with a Bragg frequency shift applied.
    pub fn generate_shifted(
        &self,
        velocity: f64,
        duration: f64,
        t0: f64,
        amplitude: f64,
        noise_std: f64,
        shift_freq: f64,
    ) -> Vec<f64> {
        let n = (duration * self.sample_rate) as usize;
        let f_d = velocity / self.fringe_spacing;
        let f_eff = f_d + shift_freq;
        let tau = duration / 4.0;
        let dt = 1.0 / self.sample_rate;
        let mut rng = SimpleRng::new(42);
        (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                let envelope = (-2.0 * (t - t0).powi(2) / tau.powi(2)).exp();
                let carrier = (2.0 * PI * f_eff * t).cos();
                let noise = if noise_std > 0.0 {
                    rng.gaussian() * noise_std
                } else {
                    0.0
                };
                amplitude * envelope * carrier + noise
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Burst detector
// ---------------------------------------------------------------------------

/// Result of burst detection.
#[derive(Debug, Clone)]
pub struct DetectedBurst {
    /// Start sample index.
    pub start: usize,
    /// End sample index (exclusive).
    pub end: usize,
    /// Peak envelope amplitude within the burst.
    pub peak_amplitude: f64,
}

/// Threshold-based burst detector operating on the signal envelope.
#[derive(Debug, Clone)]
pub struct BurstDetector {
    /// Detection threshold (linear amplitude).
    pub threshold: f64,
    /// Minimum burst length in samples.
    pub min_length: usize,
}

impl BurstDetector {
    pub fn new(threshold: f64, min_length: usize) -> Self {
        Self {
            threshold,
            min_length,
        }
    }

    /// Compute signal envelope using a sliding-window RMS.
    pub fn envelope(signal: &[f64], window: usize) -> Vec<f64> {
        let n = signal.len();
        if n == 0 || window == 0 {
            return vec![0.0; n];
        }
        let w = window.min(n);
        let mut env = vec![0.0; n];
        let mut sum_sq = 0.0;
        for i in 0..w {
            sum_sq += signal[i] * signal[i];
        }
        env[w / 2] = (sum_sq / w as f64).sqrt();
        for i in 1..=(n - w) {
            sum_sq -= signal[i - 1] * signal[i - 1];
            sum_sq += signal[i + w - 1] * signal[i + w - 1];
            if sum_sq < 0.0 {
                sum_sq = 0.0;
            }
            let idx = i + w / 2;
            if idx < n {
                env[idx] = (sum_sq / w as f64).sqrt();
            }
        }
        env
    }

    /// Detect bursts in a signal. Returns a list of detected burst regions.
    pub fn detect(&self, signal: &[f64], envelope_window: usize) -> Vec<DetectedBurst> {
        let env = Self::envelope(signal, envelope_window);
        let mut bursts = Vec::new();
        let mut in_burst = false;
        let mut start = 0usize;
        let mut peak = 0.0f64;

        for (i, &e) in env.iter().enumerate() {
            if !in_burst && e >= self.threshold {
                in_burst = true;
                start = i;
                peak = e;
            } else if in_burst {
                if e >= self.threshold {
                    if e > peak {
                        peak = e;
                    }
                } else {
                    let len = i - start;
                    if len >= self.min_length {
                        bursts.push(DetectedBurst {
                            start,
                            end: i,
                            peak_amplitude: peak,
                        });
                    }
                    in_burst = false;
                }
            }
        }
        // Handle burst extending to end of signal
        if in_burst {
            let len = env.len() - start;
            if len >= self.min_length {
                bursts.push(DetectedBurst {
                    start,
                    end: env.len(),
                    peak_amplitude: peak,
                });
            }
        }
        bursts
    }
}

// ---------------------------------------------------------------------------
// Pedestal removal
// ---------------------------------------------------------------------------

/// Remove the low-frequency pedestal (DC/Gaussian envelope) from a Doppler burst.
///
/// Uses a simple first-order IIR highpass filter: y[n] = alpha*(y[n-1] + x[n] - x[n-1])
pub fn remove_pedestal(signal: &[f64], alpha: f64) -> Vec<f64> {
    if signal.is_empty() {
        return Vec::new();
    }
    let mut out = vec![0.0; signal.len()];
    out[0] = signal[0];
    for i in 1..signal.len() {
        out[i] = alpha * (out[i - 1] + signal[i] - signal[i - 1]);
    }
    out
}

// ---------------------------------------------------------------------------
// Frequency estimation
// ---------------------------------------------------------------------------

/// Frequency estimation methods for Doppler burst signals.
#[derive(Debug, Clone)]
pub struct FrequencyEstimator {
    sample_rate: f64,
}

impl FrequencyEstimator {
    pub fn new(sample_rate: f64) -> Self {
        Self { sample_rate }
    }

    /// FFT-based frequency estimation with Hann windowing and parabolic interpolation.
    ///
    /// Returns the estimated dominant frequency in Hz.
    pub fn fft_peak(&self, signal: &[f64], fft_size: usize) -> f64 {
        let n = fft_size.min(signal.len());
        // Apply Hann window
        let windowed: Vec<f64> = (0..n)
            .map(|i| {
                let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos());
                signal[i] * w
            })
            .collect();

        // Compute magnitude spectrum via DFT (real input, only positive frequencies)
        let half = n / 2;
        let mut mag = vec![0.0; half + 1];
        for k in 0..=half {
            let mut re = 0.0;
            let mut im = 0.0;
            for (i, &s) in windowed.iter().enumerate() {
                let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
                re += s * angle.cos();
                im += s * angle.sin();
            }
            mag[k] = (re * re + im * im).sqrt();
        }

        // Find peak (skip DC bin)
        let mut peak_bin = 1;
        let mut peak_val = mag[1];
        for k in 2..=half {
            if mag[k] > peak_val {
                peak_val = mag[k];
                peak_bin = k;
            }
        }

        // Parabolic interpolation for sub-bin accuracy
        let delta = if peak_bin > 0 && peak_bin < half {
            let alpha = mag[peak_bin - 1];
            let beta = mag[peak_bin];
            let gamma = mag[peak_bin + 1];
            let denom = alpha - 2.0 * beta + gamma;
            if denom.abs() > 1e-30 {
                0.5 * (alpha - gamma) / denom
            } else {
                0.0
            }
        } else {
            0.0
        };

        (peak_bin as f64 + delta) * self.sample_rate / n as f64
    }

    /// Zero-crossing frequency estimation.
    ///
    /// Counts zero crossings and estimates frequency as crossings / (2 * duration).
    pub fn zero_crossing(&self, signal: &[f64]) -> f64 {
        if signal.len() < 2 {
            return 0.0;
        }
        let mut crossings = 0usize;
        for i in 1..signal.len() {
            if (signal[i] >= 0.0 && signal[i - 1] < 0.0)
                || (signal[i] < 0.0 && signal[i - 1] >= 0.0)
            {
                crossings += 1;
            }
        }
        let duration = signal.len() as f64 / self.sample_rate;
        crossings as f64 / (2.0 * duration)
    }

    /// Autocorrelation-based frequency estimation.
    ///
    /// Computes the autocorrelation and finds the first peak after the origin to
    /// determine the dominant period.
    pub fn autocorrelation(&self, signal: &[f64]) -> f64 {
        let n = signal.len();
        if n < 4 {
            return 0.0;
        }
        // Compute autocorrelation for lags 0..n/2
        let max_lag = n / 2;
        let mut acf = vec![0.0; max_lag];
        for lag in 0..max_lag {
            let mut sum = 0.0;
            for i in 0..(n - lag) {
                sum += signal[i] * signal[i + lag];
            }
            acf[lag] = sum;
        }

        // Normalise by lag-0
        if acf[0].abs() < 1e-30 {
            return 0.0;
        }
        for i in 1..max_lag {
            acf[i] /= acf[0];
        }

        // Find first local maximum after the first zero crossing.
        // A local maximum is a point where acf[i-1] <= acf[i] >= acf[i+1].
        let mut past_first_zero = false;
        let mut peak_lag = 0usize;
        for i in 1..max_lag {
            if !past_first_zero {
                if acf[i] < 0.0 {
                    past_first_zero = true;
                }
                continue;
            }
            // Check for local maximum
            if i + 1 < max_lag && acf[i] >= acf[i - 1] && acf[i] >= acf[i + 1] && acf[i] > 0.0 {
                peak_lag = i;
                break;
            }
        }

        if peak_lag == 0 {
            return 0.0;
        }
        self.sample_rate / peak_lag as f64
    }
}

// ---------------------------------------------------------------------------
// SNR estimation
// ---------------------------------------------------------------------------

/// Estimate the SNR of a Doppler burst.
///
/// * `signal` - The burst signal samples
/// * `noise_samples` - Samples from a noise-only region
///
/// Returns SNR in dB.
pub fn estimate_snr(signal: &[f64], noise_samples: &[f64]) -> f64 {
    let sig_power: f64 = signal.iter().map(|s| s * s).sum::<f64>() / signal.len() as f64;
    let noise_power: f64 =
        noise_samples.iter().map(|s| s * s).sum::<f64>() / noise_samples.len().max(1) as f64;
    if noise_power < 1e-30 {
        return 100.0; // effectively infinite SNR
    }
    10.0 * (sig_power / noise_power).log10()
}

// ---------------------------------------------------------------------------
// Bragg cell frequency shifting
// ---------------------------------------------------------------------------

/// Bragg cell frequency shift processor for directional velocity measurement.
#[derive(Debug, Clone)]
pub struct BraggShifter {
    /// Shift frequency in Hz (typically 40 MHz).
    pub shift_freq: f64,
    /// Fringe spacing in metres.
    pub fringe_spacing: f64,
}

impl BraggShifter {
    pub fn new(shift_freq: f64, fringe_spacing: f64) -> Self {
        Self {
            shift_freq,
            fringe_spacing,
        }
    }

    /// From an LDA config and shift frequency.
    pub fn from_config(config: &LdaConfig, shift_freq: f64) -> Self {
        Self {
            shift_freq,
            fringe_spacing: config.fringe_spacing(),
        }
    }

    /// Velocity from the measured (shifted) frequency.
    ///
    /// v = (f_measured - f_shift) * d_f
    pub fn velocity(&self, measured_freq: f64) -> f64 {
        (measured_freq - self.shift_freq) * self.fringe_spacing
    }

    /// Expected measured frequency for a given velocity.
    ///
    /// f_measured = v / d_f + f_shift
    pub fn expected_freq(&self, velocity: f64) -> f64 {
        velocity / self.fringe_spacing + self.shift_freq
    }
}

// ---------------------------------------------------------------------------
// Statistical processing / velocity processor
// ---------------------------------------------------------------------------

/// A single validated velocity measurement.
#[derive(Debug, Clone, Copy)]
pub struct VelocityMeasurement {
    /// Measured velocity in m/s.
    pub velocity: f64,
    /// Transit time through the measurement volume in seconds (for bias correction).
    pub transit_time: f64,
    /// Arrival time (seconds from start of acquisition).
    pub arrival_time: f64,
    /// SNR of the burst in dB.
    pub snr_db: f64,
}

/// Ensemble velocity statistics with optional bias correction.
#[derive(Debug, Clone)]
pub struct VelocityStats {
    /// Unweighted mean velocity.
    pub mean: f64,
    /// Transit-time-weighted mean velocity.
    pub mean_weighted: f64,
    /// RMS velocity fluctuation (unweighted).
    pub rms_fluctuation: f64,
    /// Turbulence intensity TI = u_rms / |U_mean|.
    pub turbulence_intensity: f64,
    /// Skewness of the velocity distribution.
    pub skewness: f64,
    /// Kurtosis (flatness factor) of the velocity distribution.
    pub kurtosis: f64,
    /// Number of valid measurements.
    pub count: usize,
}

/// Process an ensemble of velocity measurements.
pub struct VelocityProcessor {
    measurements: Vec<VelocityMeasurement>,
    /// Minimum SNR threshold for data validation (dB).
    pub snr_threshold: f64,
    /// Velocity range for validation [min, max] in m/s.
    pub velocity_range: (f64, f64),
}

impl VelocityProcessor {
    pub fn new() -> Self {
        Self {
            measurements: Vec::new(),
            snr_threshold: 0.0,
            velocity_range: (f64::NEG_INFINITY, f64::INFINITY),
        }
    }

    /// Set the SNR threshold for burst validation.
    pub fn with_snr_threshold(mut self, threshold_db: f64) -> Self {
        self.snr_threshold = threshold_db;
        self
    }

    /// Set the acceptable velocity range.
    pub fn with_velocity_range(mut self, min: f64, max: f64) -> Self {
        self.velocity_range = (min, max);
        self
    }

    /// Add a measurement (validated against thresholds before storage).
    pub fn add(&mut self, m: VelocityMeasurement) -> bool {
        if self.validate(&m) {
            self.measurements.push(m);
            true
        } else {
            false
        }
    }

    /// Add a batch of measurements.
    pub fn add_batch(&mut self, batch: &[VelocityMeasurement]) -> usize {
        let mut added = 0;
        for m in batch {
            if self.add(*m) {
                added += 1;
            }
        }
        added
    }

    /// Validate a single measurement.
    pub fn validate(&self, m: &VelocityMeasurement) -> bool {
        m.snr_db >= self.snr_threshold
            && m.velocity >= self.velocity_range.0
            && m.velocity <= self.velocity_range.1
    }

    /// Number of stored measurements.
    pub fn count(&self) -> usize {
        self.measurements.len()
    }

    /// Compute ensemble statistics.
    pub fn statistics(&self) -> VelocityStats {
        let n = self.measurements.len();
        if n == 0 {
            return VelocityStats {
                mean: 0.0,
                mean_weighted: 0.0,
                rms_fluctuation: 0.0,
                turbulence_intensity: 0.0,
                skewness: 0.0,
                kurtosis: 0.0,
                count: 0,
            };
        }

        // Unweighted mean
        let sum: f64 = self.measurements.iter().map(|m| m.velocity).sum();
        let mean = sum / n as f64;

        // Transit-time weighted mean (weight = transit_time)
        let total_weight: f64 = self.measurements.iter().map(|m| m.transit_time).sum();
        let mean_weighted = if total_weight > 0.0 {
            self.measurements
                .iter()
                .map(|m| m.velocity * m.transit_time)
                .sum::<f64>()
                / total_weight
        } else {
            mean
        };

        // Fluctuation moments
        let var: f64 = self
            .measurements
            .iter()
            .map(|m| (m.velocity - mean).powi(2))
            .sum::<f64>()
            / n as f64;
        let rms = var.sqrt();

        let ti = if mean.abs() > 1e-30 {
            rms / mean.abs()
        } else {
            0.0
        };

        let std_dev = rms;
        let (skewness, kurtosis) = if std_dev > 1e-30 && n > 2 {
            let m3: f64 = self
                .measurements
                .iter()
                .map(|m| ((m.velocity - mean) / std_dev).powi(3))
                .sum::<f64>()
                / n as f64;
            let m4: f64 = self
                .measurements
                .iter()
                .map(|m| ((m.velocity - mean) / std_dev).powi(4))
                .sum::<f64>()
                / n as f64;
            (m3, m4)
        } else {
            (0.0, 0.0)
        };

        VelocityStats {
            mean,
            mean_weighted,
            rms_fluctuation: rms,
            turbulence_intensity: ti,
            skewness,
            kurtosis,
            count: n,
        }
    }

    /// Compute a histogram (probability density function) of velocities.
    ///
    /// Returns (bin_centres, counts_normalised).
    pub fn velocity_pdf(&self, num_bins: usize) -> (Vec<f64>, Vec<f64>) {
        if self.measurements.is_empty() || num_bins == 0 {
            return (Vec::new(), Vec::new());
        }
        let velocities: Vec<f64> = self.measurements.iter().map(|m| m.velocity).collect();
        let vmin = velocities.iter().cloned().fold(f64::INFINITY, f64::min);
        let vmax = velocities
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        if (vmax - vmin).abs() < 1e-30 {
            return (vec![vmin], vec![1.0]);
        }
        let bin_width = (vmax - vmin) / num_bins as f64;
        let mut counts = vec![0usize; num_bins];
        for &v in &velocities {
            let idx = ((v - vmin) / bin_width) as usize;
            let idx = idx.min(num_bins - 1);
            counts[idx] += 1;
        }
        let total = velocities.len() as f64 * bin_width;
        let centres: Vec<f64> = (0..num_bins)
            .map(|i| vmin + (i as f64 + 0.5) * bin_width)
            .collect();
        let pdf: Vec<f64> = counts.iter().map(|&c| c as f64 / total).collect();
        (centres, pdf)
    }

    /// Compute Reynolds stress <u'v'> from paired two-component measurements.
    ///
    /// Takes two slices of equal-length velocity fluctuations (u', v').
    pub fn reynolds_stress(u_prime: &[f64], v_prime: &[f64]) -> f64 {
        let n = u_prime.len().min(v_prime.len());
        if n == 0 {
            return 0.0;
        }
        let sum: f64 = (0..n).map(|i| u_prime[i] * v_prime[i]).sum();
        sum / n as f64
    }

    /// Velocity bias correction using transit-time weighting.
    ///
    /// In LDA, faster particles are over-represented because they cross the
    /// measurement volume more frequently. Transit-time weighting compensates
    /// by weighting each burst by its transit time (proportional to 1/|v|).
    pub fn bias_corrected_mean(&self) -> f64 {
        if self.measurements.is_empty() {
            return 0.0;
        }
        let total_weight: f64 = self.measurements.iter().map(|m| m.transit_time).sum();
        if total_weight <= 0.0 {
            return self.statistics().mean;
        }
        self.measurements
            .iter()
            .map(|m| m.velocity * m.transit_time)
            .sum::<f64>()
            / total_weight
    }

    /// Inverse-velocity weighting as an alternative bias correction.
    pub fn inverse_velocity_weighted_mean(&self) -> f64 {
        if self.measurements.is_empty() {
            return 0.0;
        }
        let mut sum_wv = 0.0;
        let mut sum_w = 0.0;
        for m in &self.measurements {
            let w = if m.velocity.abs() > 1e-30 {
                1.0 / m.velocity.abs()
            } else {
                1.0
            };
            sum_wv += w * m.velocity;
            sum_w += w;
        }
        if sum_w > 0.0 {
            sum_wv / sum_w
        } else {
            0.0
        }
    }
}

impl Default for VelocityProcessor {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Two-component LDA
// ---------------------------------------------------------------------------

/// Two-component LDA processor for simultaneous u,v velocity measurement.
#[derive(Debug, Clone)]
pub struct TwoComponentLda {
    /// Configuration for the first component (e.g., green beam pair).
    pub config_u: LdaConfig,
    /// Configuration for the second component (e.g., blue beam pair).
    pub config_v: LdaConfig,
    /// Coincidence window in seconds.
    pub coincidence_window: f64,
}

/// Paired two-component measurement.
#[derive(Debug, Clone, Copy)]
pub struct TwoComponentMeasurement {
    pub u_velocity: f64,
    pub v_velocity: f64,
    pub arrival_time: f64,
}

impl TwoComponentLda {
    pub fn new(config_u: LdaConfig, config_v: LdaConfig, coincidence_window: f64) -> Self {
        Self {
            config_u,
            config_v,
            coincidence_window,
        }
    }

    /// Pair measurements from two channels using a coincidence window.
    ///
    /// Each channel provides (velocity, arrival_time) tuples. Measurements are
    /// paired if their arrival times fall within the coincidence window.
    pub fn find_coincident(
        &self,
        u_meas: &[(f64, f64)],
        v_meas: &[(f64, f64)],
    ) -> Vec<TwoComponentMeasurement> {
        let mut pairs = Vec::new();
        let mut v_idx = 0;
        for &(u_vel, u_time) in u_meas {
            // Advance v_idx past measurements too early
            while v_idx < v_meas.len() && v_meas[v_idx].1 < u_time - self.coincidence_window {
                v_idx += 1;
            }
            // Check candidates within window
            let mut j = v_idx;
            let mut best_dt = f64::INFINITY;
            let mut best_j = None;
            while j < v_meas.len() && v_meas[j].1 <= u_time + self.coincidence_window {
                let dt = (v_meas[j].1 - u_time).abs();
                if dt < best_dt {
                    best_dt = dt;
                    best_j = Some(j);
                }
                j += 1;
            }
            if let Some(bj) = best_j {
                pairs.push(TwoComponentMeasurement {
                    u_velocity: u_vel,
                    v_velocity: v_meas[bj].0,
                    arrival_time: (u_time + v_meas[bj].1) / 2.0,
                });
            }
        }
        pairs
    }

    /// Compute Reynolds stress from coincident measurements.
    pub fn reynolds_stress(&self, pairs: &[TwoComponentMeasurement]) -> f64 {
        if pairs.is_empty() {
            return 0.0;
        }
        let u_mean: f64 = pairs.iter().map(|p| p.u_velocity).sum::<f64>() / pairs.len() as f64;
        let v_mean: f64 = pairs.iter().map(|p| p.v_velocity).sum::<f64>() / pairs.len() as f64;
        let stress: f64 = pairs
            .iter()
            .map(|p| (p.u_velocity - u_mean) * (p.v_velocity - v_mean))
            .sum::<f64>()
            / pairs.len() as f64;
        stress
    }
}

// ---------------------------------------------------------------------------
// Spectral analysis of velocity time series
// ---------------------------------------------------------------------------

/// Compute the power spectrum of an irregularly sampled velocity time series
/// using the slot correlation technique.
///
/// * `times` - Arrival times in seconds
/// * `velocities` - Velocity values in m/s
/// * `num_lags` - Number of correlation lags to compute
/// * `slot_width` - Width of each time slot in seconds
///
/// Returns (frequencies, power) vectors.
pub fn velocity_power_spectrum(
    times: &[f64],
    velocities: &[f64],
    num_lags: usize,
    slot_width: f64,
) -> (Vec<f64>, Vec<f64>) {
    let n = times.len().min(velocities.len());
    if n < 2 || num_lags == 0 {
        return (Vec::new(), Vec::new());
    }

    let mean_vel = velocities[..n].iter().sum::<f64>() / n as f64;
    let flucts: Vec<f64> = velocities[..n].iter().map(|v| v - mean_vel).collect();

    // Slot correlation: for each lag slot, average products of pairs
    let mut acf = vec![0.0; num_lags];
    let mut counts = vec![0usize; num_lags];

    for i in 0..n {
        for j in (i + 1)..n {
            let dt = times[j] - times[i];
            if dt < 0.0 {
                continue;
            }
            let slot = (dt / slot_width) as usize;
            if slot < num_lags {
                acf[slot] += flucts[i] * flucts[j];
                counts[slot] += 1;
            }
        }
    }

    // Normalise each slot
    for k in 0..num_lags {
        if counts[k] > 0 {
            acf[k] /= counts[k] as f64;
        }
    }

    // DFT of the autocorrelation to get power spectrum
    let mut freqs = Vec::with_capacity(num_lags / 2 + 1);
    let mut power = Vec::with_capacity(num_lags / 2 + 1);
    let df = 1.0 / (num_lags as f64 * slot_width);

    for k in 0..=num_lags / 2 {
        let mut re = 0.0;
        for (m, &a) in acf.iter().enumerate() {
            re += a * (2.0 * PI * k as f64 * m as f64 / num_lags as f64).cos();
        }
        freqs.push(k as f64 * df);
        power.push(re.abs() * slot_width);
    }

    (freqs, power)
}

/// Compute the integral time scale from a velocity autocorrelation function.
///
/// The integral time scale is the integral of the normalised ACF from zero
/// lag to the first zero crossing.
pub fn integral_time_scale(
    times: &[f64],
    velocities: &[f64],
    num_lags: usize,
    slot_width: f64,
) -> f64 {
    let n = times.len().min(velocities.len());
    if n < 2 || num_lags == 0 {
        return 0.0;
    }

    let mean_vel = velocities[..n].iter().sum::<f64>() / n as f64;
    let flucts: Vec<f64> = velocities[..n].iter().map(|v| v - mean_vel).collect();

    let mut acf = vec![0.0; num_lags];
    let mut counts = vec![0usize; num_lags];

    for i in 0..n {
        for j in (i + 1)..n {
            let dt = times[j] - times[i];
            if dt < 0.0 {
                continue;
            }
            let slot = (dt / slot_width) as usize;
            if slot < num_lags {
                acf[slot] += flucts[i] * flucts[j];
                counts[slot] += 1;
            }
        }
    }

    // Variance (lag 0)
    let var: f64 = flucts.iter().map(|f| f * f).sum::<f64>() / n as f64;
    if var < 1e-30 {
        return 0.0;
    }

    for k in 0..num_lags {
        if counts[k] > 0 {
            acf[k] /= counts[k] as f64;
        }
        acf[k] /= var; // normalise
    }

    // Integrate to first zero crossing using trapezoidal rule
    let mut integral = 0.0;
    for k in 1..num_lags {
        if acf[k] < 0.0 {
            // Interpolate zero crossing
            let frac = acf[k - 1] / (acf[k - 1] - acf[k]);
            integral += 0.5 * acf[k - 1] * frac * slot_width;
            break;
        }
        integral += 0.5 * (acf[k - 1] + acf[k]) * slot_width;
    }
    integral
}

// ---------------------------------------------------------------------------
// Data validation
// ---------------------------------------------------------------------------

/// Validate a measurement against multiple criteria.
#[derive(Debug, Clone)]
pub struct DataValidator {
    /// Minimum SNR in dB.
    pub min_snr_db: f64,
    /// Acceptable velocity range.
    pub velocity_min: f64,
    pub velocity_max: f64,
    /// Maximum allowed deviation from expected velocity (for comparison checks).
    pub comparison_tolerance: f64,
}

impl DataValidator {
    pub fn new(min_snr_db: f64, velocity_min: f64, velocity_max: f64) -> Self {
        Self {
            min_snr_db,
            velocity_min,
            velocity_max,
            comparison_tolerance: f64::INFINITY,
        }
    }

    /// Set the comparison tolerance for multi-component checks.
    pub fn with_comparison_tolerance(mut self, tol: f64) -> Self {
        self.comparison_tolerance = tol;
        self
    }

    /// Check if a measurement passes SNR and velocity range criteria.
    pub fn is_valid(&self, velocity: f64, snr_db: f64) -> bool {
        snr_db >= self.min_snr_db && velocity >= self.velocity_min && velocity <= self.velocity_max
    }

    /// Check if measured velocity is within tolerance of an expected value.
    pub fn comparison_check(&self, measured: f64, expected: f64) -> bool {
        (measured - expected).abs() <= self.comparison_tolerance
    }
}

// ---------------------------------------------------------------------------
// Simple PRNG for noise generation (no external dependencies)
// ---------------------------------------------------------------------------

struct SimpleRng {
    state: u64,
}

impl SimpleRng {
    fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(1),
        }
    }

    fn next_u64(&mut self) -> u64 {
        // xorshift64
        self.state ^= self.state << 13;
        self.state ^= self.state >> 7;
        self.state ^= self.state << 17;
        self.state
    }

    fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn gaussian(&mut self) -> f64 {
        // Box-Muller
        let u1 = self.uniform().max(1e-30);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn test_config() -> LdaConfig {
        // Green Nd:YAG 532 nm, 5 degree half-angle, 200 mm focal length,
        // 50 mm beam separation, 0.1 mm beam diameter at focus
        LdaConfig::new(532.0, 5.0, 200.0, 50.0, 0.1)
    }

    // --- LdaConfig tests ---

    #[test]
    fn test_half_angle_rad() {
        let cfg = test_config();
        let expected = 5.0 * PI / 180.0;
        assert!((cfg.half_angle_rad() - expected).abs() < 1e-12);
    }

    #[test]
    fn test_wavelength_m() {
        let cfg = test_config();
        assert!((cfg.wavelength_m() - 532e-9).abs() < 1e-18);
    }

    #[test]
    fn test_fringe_spacing() {
        let cfg = test_config();
        let d_f = cfg.fringe_spacing();
        // d_f = 532e-9 / (2 * sin(5 deg))
        let expected = 532e-9 / (2.0 * (5.0_f64 * PI / 180.0).sin());
        assert!(
            (d_f - expected).abs() < 1e-12,
            "fringe spacing: {} vs expected {}",
            d_f,
            expected
        );
    }

    #[test]
    fn test_doppler_frequency() {
        let cfg = test_config();
        let v = 10.0; // 10 m/s
        let f_d = cfg.doppler_frequency(v);
        let expected = v / cfg.fringe_spacing();
        assert!((f_d - expected).abs() < 1e-6);
    }

    #[test]
    fn test_velocity_from_frequency_roundtrip() {
        let cfg = test_config();
        let v_in = 5.5;
        let f_d = cfg.doppler_frequency(v_in);
        let v_out = cfg.velocity_from_frequency(f_d);
        assert!((v_in - v_out).abs() < 1e-12);
    }

    #[test]
    fn test_zero_velocity_gives_zero_freq() {
        let cfg = test_config();
        assert_eq!(cfg.doppler_frequency(0.0), 0.0);
    }

    #[test]
    fn test_negative_velocity() {
        let cfg = test_config();
        let v = -3.0;
        let f_d = cfg.doppler_frequency(v);
        assert!(f_d < 0.0);
        let v_back = cfg.velocity_from_frequency(f_d);
        assert!((v - v_back).abs() < 1e-12);
    }

    // --- Measurement volume tests ---

    #[test]
    fn test_measurement_volume_dimensions() {
        let cfg = test_config();
        let mv = cfg.measurement_volume();
        // dy should equal beam diameter
        assert!((mv.dy_mm - cfg.beam_diameter_mm).abs() < 1e-12);
        // dx > dy (dx = d / cos(theta))
        assert!(mv.dx_mm > mv.dy_mm);
        // dz > dy (dz = d / sin(theta))
        assert!(mv.dz_mm > mv.dy_mm);
    }

    #[test]
    fn test_num_fringes_positive() {
        let cfg = test_config();
        let mv = cfg.measurement_volume();
        assert!(mv.num_fringes > 0.0, "num fringes = {}", mv.num_fringes);
    }

    #[test]
    fn test_volume_positive() {
        let cfg = test_config();
        let mv = cfg.measurement_volume();
        assert!(mv.volume_mm3 > 0.0);
    }

    #[test]
    fn test_dz_larger_than_dx() {
        // For small angles, dz >> dx
        let cfg = test_config();
        let mv = cfg.measurement_volume();
        assert!(mv.dz_mm > mv.dx_mm);
    }

    // --- Burst generator tests ---

    #[test]
    fn test_burst_length() {
        let cfg = test_config();
        let gen = BurstGenerator::new(&cfg, 1_000_000.0);
        let burst = gen.generate(5.0, 0.001, 0.0005, 1.0, 0.0);
        assert_eq!(burst.len(), 1000);
    }

    #[test]
    fn test_burst_gaussian_envelope() {
        let cfg = test_config();
        let gen = BurstGenerator::new(&cfg, 100_000.0);
        let burst = gen.generate(1.0, 0.01, 0.005, 1.0, 0.0);
        // Centre should have higher amplitude than edges
        let mid = burst.len() / 2;
        let centre_power: f64 = burst[mid - 10..mid + 10]
            .iter()
            .map(|s| s * s)
            .sum::<f64>()
            / 20.0;
        let edge_power: f64 = burst[0..20].iter().map(|s| s * s).sum::<f64>() / 20.0;
        assert!(centre_power > edge_power);
    }

    #[test]
    fn test_burst_with_noise() {
        let cfg = test_config();
        let gen = BurstGenerator::new(&cfg, 100_000.0);
        let clean = gen.generate(5.0, 0.001, 0.0005, 1.0, 0.0);
        let noisy = gen.generate(5.0, 0.001, 0.0005, 1.0, 0.1);
        // Noisy signal should differ from clean
        let diff: f64 = clean
            .iter()
            .zip(noisy.iter())
            .map(|(a, b)| (a - b).powi(2))
            .sum();
        assert!(diff > 0.0);
    }

    #[test]
    fn test_burst_shifted() {
        let cfg = test_config();
        let gen = BurstGenerator::new(&cfg, 1_000_000.0);
        let normal = gen.generate(5.0, 0.001, 0.0005, 1.0, 0.0);
        let shifted = gen.generate_shifted(5.0, 0.001, 0.0005, 1.0, 0.0, 40e6);
        // They should differ since one has 40 MHz carrier shift
        let diff: f64 = normal
            .iter()
            .zip(shifted.iter())
            .map(|(a, b)| (a - b).powi(2))
            .sum();
        assert!(diff > 0.0);
    }

    // --- Burst detector tests ---

    #[test]
    fn test_envelope_computation() {
        let signal: Vec<f64> = (0..100).map(|i| if i >= 40 && i < 60 { 1.0 } else { 0.0 }).collect();
        let env = BurstDetector::envelope(&signal, 5);
        // Envelope should peak in the middle region
        let mid_val = env[50];
        let edge_val = env[10];
        assert!(mid_val > edge_val);
    }

    #[test]
    fn test_detect_single_burst() {
        let cfg = test_config();
        let gen = BurstGenerator::new(&cfg, 100_000.0);
        let burst = gen.generate(5.0, 0.01, 0.005, 1.0, 0.0);
        let detector = BurstDetector::new(0.1, 10);
        let detected = detector.detect(&burst, 50);
        assert!(
            !detected.is_empty(),
            "Should detect at least one burst"
        );
    }

    #[test]
    fn test_no_burst_in_noise() {
        let mut rng = SimpleRng::new(123);
        let noise: Vec<f64> = (0..1000).map(|_| rng.gaussian() * 0.01).collect();
        let detector = BurstDetector::new(0.5, 10);
        let detected = detector.detect(&noise, 20);
        assert!(detected.is_empty(), "Should not detect bursts in low noise");
    }

    // --- Pedestal removal test ---

    #[test]
    fn test_pedestal_removal() {
        // Signal with DC offset
        let n = 200;
        let signal: Vec<f64> = (0..n)
            .map(|i| 1.0 + (2.0 * PI * 10.0 * i as f64 / n as f64).cos())
            .collect();
        let filtered = remove_pedestal(&signal, 0.99);
        // Mean should be reduced
        let mean_orig: f64 = signal.iter().sum::<f64>() / n as f64;
        let mean_filt: f64 = filtered.iter().sum::<f64>() / n as f64;
        assert!(
            mean_filt.abs() < mean_orig.abs(),
            "Pedestal removal should reduce DC: {} vs {}",
            mean_filt,
            mean_orig
        );
    }

    #[test]
    fn test_pedestal_removal_empty() {
        let result = remove_pedestal(&[], 0.99);
        assert!(result.is_empty());
    }

    // --- Frequency estimation tests ---

    #[test]
    fn test_fft_peak_pure_tone() {
        let fs = 10000.0;
        let f0 = 1234.0;
        let n = 2048;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f0 * i as f64 / fs).cos())
            .collect();
        let est = FrequencyEstimator::new(fs);
        let freq = est.fft_peak(&signal, n);
        assert!(
            (freq - f0).abs() < 10.0,
            "FFT peak {} should be near {}",
            freq,
            f0
        );
    }

    #[test]
    fn test_zero_crossing_pure_tone() {
        let fs = 10000.0;
        let f0 = 500.0;
        let n = 4000;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f0 * i as f64 / fs).cos())
            .collect();
        let est = FrequencyEstimator::new(fs);
        let freq = est.zero_crossing(&signal);
        assert!(
            (freq - f0).abs() < 20.0,
            "Zero-crossing {} should be near {}",
            freq,
            f0
        );
    }

    #[test]
    fn test_autocorrelation_pure_tone() {
        let fs = 10000.0;
        let f0 = 800.0;
        let n = 2000;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f0 * i as f64 / fs).cos())
            .collect();
        let est = FrequencyEstimator::new(fs);
        let freq = est.autocorrelation(&signal);
        assert!(
            (freq - f0).abs() < 50.0,
            "Autocorrelation {} should be near {}",
            freq,
            f0
        );
    }

    #[test]
    fn test_frequency_estimator_short_signal() {
        let est = FrequencyEstimator::new(1000.0);
        assert_eq!(est.zero_crossing(&[1.0]), 0.0);
        assert_eq!(est.autocorrelation(&[1.0, 2.0]), 0.0);
    }

    // --- SNR estimation test ---

    #[test]
    fn test_snr_estimation() {
        let signal: Vec<f64> = (0..1000)
            .map(|i| (2.0 * PI * 100.0 * i as f64 / 10000.0).cos())
            .collect();
        let noise: Vec<f64> = vec![0.01; 1000];
        let snr = estimate_snr(&signal, &noise);
        // Signal power ~0.5, noise power ~0.0001, SNR ~37 dB
        assert!(snr > 30.0, "SNR should be high: {} dB", snr);
    }

    #[test]
    fn test_snr_zero_noise() {
        let signal = vec![1.0; 100];
        let noise = vec![0.0; 100];
        let snr = estimate_snr(&signal, &noise);
        assert_eq!(snr, 100.0);
    }

    // --- Bragg shifter tests ---

    #[test]
    fn test_bragg_velocity_positive() {
        let cfg = test_config();
        let bragg = BraggShifter::from_config(&cfg, 40e6);
        // Forward flow at 5 m/s
        let v = 5.0;
        let f_meas = bragg.expected_freq(v);
        assert!(f_meas > 40e6); // frequency above shift
        let v_back = bragg.velocity(f_meas);
        assert!((v - v_back).abs() < 1e-10);
    }

    #[test]
    fn test_bragg_velocity_negative() {
        let cfg = test_config();
        let bragg = BraggShifter::from_config(&cfg, 40e6);
        // Reverse flow at -3 m/s
        let v = -3.0;
        let f_meas = bragg.expected_freq(v);
        assert!(f_meas < 40e6); // frequency below shift
        let v_back = bragg.velocity(f_meas);
        assert!((v - v_back).abs() < 1e-10);
    }

    #[test]
    fn test_bragg_zero_velocity() {
        let cfg = test_config();
        let bragg = BraggShifter::from_config(&cfg, 40e6);
        let f_meas = bragg.expected_freq(0.0);
        assert!((f_meas - 40e6).abs() < 1e-6);
    }

    // --- VelocityProcessor tests ---

    #[test]
    fn test_velocity_processor_empty() {
        let proc = VelocityProcessor::new();
        let stats = proc.statistics();
        assert_eq!(stats.count, 0);
        assert_eq!(stats.mean, 0.0);
    }

    #[test]
    fn test_velocity_processor_stats() {
        let mut proc = VelocityProcessor::new();
        for i in 0..100 {
            proc.add(VelocityMeasurement {
                velocity: 10.0 + (i as f64 - 50.0) * 0.1,
                transit_time: 1e-4,
                arrival_time: i as f64 * 0.01,
                snr_db: 20.0,
            });
        }
        let stats = proc.statistics();
        assert_eq!(stats.count, 100);
        assert!((stats.mean - 10.0).abs() < 0.5);
        assert!(stats.rms_fluctuation > 0.0);
    }

    #[test]
    fn test_velocity_processor_snr_filter() {
        let mut proc = VelocityProcessor::new().with_snr_threshold(10.0);
        let accepted = proc.add(VelocityMeasurement {
            velocity: 5.0,
            transit_time: 1e-4,
            arrival_time: 0.0,
            snr_db: 15.0,
        });
        assert!(accepted);
        let rejected = proc.add(VelocityMeasurement {
            velocity: 5.0,
            transit_time: 1e-4,
            arrival_time: 0.01,
            snr_db: 5.0,
        });
        assert!(!rejected);
        assert_eq!(proc.count(), 1);
    }

    #[test]
    fn test_velocity_processor_range_filter() {
        let mut proc = VelocityProcessor::new().with_velocity_range(-10.0, 50.0);
        let accepted = proc.add(VelocityMeasurement {
            velocity: 25.0,
            transit_time: 1e-4,
            arrival_time: 0.0,
            snr_db: 20.0,
        });
        assert!(accepted);
        let rejected = proc.add(VelocityMeasurement {
            velocity: 100.0,
            transit_time: 1e-4,
            arrival_time: 0.01,
            snr_db: 20.0,
        });
        assert!(!rejected);
    }

    #[test]
    fn test_turbulence_intensity() {
        let mut proc = VelocityProcessor::new();
        // Constant velocity => TI = 0
        for i in 0..50 {
            proc.add(VelocityMeasurement {
                velocity: 10.0,
                transit_time: 1e-4,
                arrival_time: i as f64 * 0.01,
                snr_db: 20.0,
            });
        }
        let stats = proc.statistics();
        assert!(
            stats.turbulence_intensity < 1e-10,
            "TI should be ~0 for constant velocity"
        );
    }

    #[test]
    fn test_velocity_pdf() {
        let mut proc = VelocityProcessor::new();
        for i in 0..1000 {
            proc.add(VelocityMeasurement {
                velocity: 10.0 + (i % 10) as f64,
                transit_time: 1e-4,
                arrival_time: i as f64 * 0.001,
                snr_db: 20.0,
            });
        }
        let (centres, pdf) = proc.velocity_pdf(20);
        assert!(!centres.is_empty());
        assert_eq!(centres.len(), pdf.len());
        // PDF should integrate to approximately 1
        let bin_width = centres[1] - centres[0];
        let integral: f64 = pdf.iter().map(|p| p * bin_width).sum();
        assert!(
            (integral - 1.0).abs() < 0.1,
            "PDF integral = {}",
            integral
        );
    }

    #[test]
    fn test_reynolds_stress() {
        // Perfectly correlated fluctuations
        let u_prime = vec![1.0, -1.0, 1.0, -1.0];
        let v_prime = vec![1.0, -1.0, 1.0, -1.0];
        let rs = VelocityProcessor::reynolds_stress(&u_prime, &v_prime);
        assert!((rs - 1.0).abs() < 1e-12);

        // Anti-correlated
        let v_anti = vec![-1.0, 1.0, -1.0, 1.0];
        let rs2 = VelocityProcessor::reynolds_stress(&u_prime, &v_anti);
        assert!((rs2 - (-1.0)).abs() < 1e-12);
    }

    #[test]
    fn test_bias_corrected_mean() {
        let mut proc = VelocityProcessor::new();
        // Higher velocity particles have shorter transit times
        proc.add(VelocityMeasurement {
            velocity: 20.0,
            transit_time: 0.5e-4, // shorter transit
            arrival_time: 0.0,
            snr_db: 20.0,
        });
        proc.add(VelocityMeasurement {
            velocity: 10.0,
            transit_time: 1.0e-4, // longer transit
            arrival_time: 0.01,
            snr_db: 20.0,
        });
        let unweighted = proc.statistics().mean;
        let weighted = proc.bias_corrected_mean();
        // Transit-time weighting should give more weight to the slower particle
        assert!(
            weighted < unweighted,
            "Bias-corrected {} should be less than unweighted {}",
            weighted,
            unweighted
        );
    }

    #[test]
    fn test_inverse_velocity_weighted_mean() {
        let mut proc = VelocityProcessor::new();
        proc.add(VelocityMeasurement {
            velocity: 20.0,
            transit_time: 1e-4,
            arrival_time: 0.0,
            snr_db: 20.0,
        });
        proc.add(VelocityMeasurement {
            velocity: 10.0,
            transit_time: 1e-4,
            arrival_time: 0.01,
            snr_db: 20.0,
        });
        let ivm = proc.inverse_velocity_weighted_mean();
        let simple_mean = proc.statistics().mean;
        // Inverse-velocity weighting gives more weight to slower particles
        assert!(ivm < simple_mean);
    }

    // --- Data validator tests ---

    #[test]
    fn test_data_validator() {
        let validator = DataValidator::new(10.0, 0.0, 100.0);
        assert!(validator.is_valid(50.0, 20.0));
        assert!(!validator.is_valid(50.0, 5.0)); // low SNR
        assert!(!validator.is_valid(-5.0, 20.0)); // out of range
        assert!(!validator.is_valid(150.0, 20.0)); // out of range
    }

    #[test]
    fn test_comparison_check() {
        let validator = DataValidator::new(0.0, -100.0, 100.0).with_comparison_tolerance(0.5);
        assert!(validator.comparison_check(10.3, 10.0));
        assert!(!validator.comparison_check(11.0, 10.0));
    }

    // --- Two-component LDA tests ---

    #[test]
    fn test_coincidence_pairing() {
        let cfg_u = test_config();
        let cfg_v = LdaConfig::new(488.0, 5.0, 200.0, 50.0, 0.1);
        let lda2 = TwoComponentLda::new(cfg_u, cfg_v, 0.001);

        let u_meas = vec![
            (5.0, 0.010),
            (5.1, 0.020),
            (4.9, 0.030),
        ];
        let v_meas = vec![
            (2.0, 0.0105), // within 0.001 of u[0]
            (2.1, 0.021),  // within 0.001 of u[1]
            (2.2, 0.050),  // no match
        ];

        let pairs = lda2.find_coincident(&u_meas, &v_meas);
        assert_eq!(pairs.len(), 2);
        assert!((pairs[0].u_velocity - 5.0).abs() < 1e-10);
        assert!((pairs[0].v_velocity - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_two_component_reynolds_stress() {
        let cfg_u = test_config();
        let cfg_v = LdaConfig::new(488.0, 5.0, 200.0, 50.0, 0.1);
        let lda2 = TwoComponentLda::new(cfg_u, cfg_v, 0.001);

        let pairs = vec![
            TwoComponentMeasurement {
                u_velocity: 11.0,
                v_velocity: 3.0,
                arrival_time: 0.0,
            },
            TwoComponentMeasurement {
                u_velocity: 9.0,
                v_velocity: 1.0,
                arrival_time: 0.01,
            },
        ];
        let rs = lda2.reynolds_stress(&pairs);
        // u' = [1, -1], v' = [1, -1] => <u'v'> = 1.0
        assert!((rs - 1.0).abs() < 1e-10);
    }

    // --- Spectral analysis tests ---

    #[test]
    fn test_velocity_power_spectrum() {
        // Regular sampling with a sinusoidal velocity variation
        let n = 200;
        let dt = 0.01;
        let f_mod = 5.0; // 5 Hz velocity oscillation
        let times: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        let velocities: Vec<f64> = times
            .iter()
            .map(|&t| 10.0 + 2.0 * (2.0 * PI * f_mod * t).cos())
            .collect();

        let (freqs, power) = velocity_power_spectrum(&times, &velocities, 100, dt);
        assert!(!freqs.is_empty());
        assert!(!power.is_empty());
        // Peak should be near 5 Hz
        let peak_idx = power
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        let peak_freq = freqs[peak_idx];
        assert!(
            (peak_freq - f_mod).abs() < 2.0,
            "Peak at {} Hz, expected near {}",
            peak_freq,
            f_mod
        );
    }

    #[test]
    fn test_integral_time_scale() {
        let n = 500;
        let dt = 0.001;
        let times: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        let velocities: Vec<f64> = times
            .iter()
            .map(|&t| 10.0 + (2.0 * PI * 20.0 * t).cos())
            .collect();
        let tau = integral_time_scale(&times, &velocities, 50, dt);
        assert!(tau > 0.0, "Integral time scale should be positive: {}", tau);
    }

    // --- Skewness and kurtosis tests ---

    #[test]
    fn test_gaussian_kurtosis() {
        // For a Gaussian distribution, kurtosis ~ 3.0 (excess kurtosis ~ 0)
        let mut proc = VelocityProcessor::new();
        let mut rng = SimpleRng::new(999);
        for i in 0..5000 {
            let v = 10.0 + rng.gaussian() * 2.0;
            proc.add(VelocityMeasurement {
                velocity: v,
                transit_time: 1e-4,
                arrival_time: i as f64 * 0.001,
                snr_db: 20.0,
            });
        }
        let stats = proc.statistics();
        // Kurtosis of Gaussian ~ 3.0
        assert!(
            (stats.kurtosis - 3.0).abs() < 0.5,
            "Kurtosis {} should be near 3.0",
            stats.kurtosis
        );
        // Skewness should be near 0
        assert!(
            stats.skewness.abs() < 0.3,
            "Skewness {} should be near 0",
            stats.skewness
        );
    }

    #[test]
    fn test_add_batch() {
        let mut proc = VelocityProcessor::new().with_snr_threshold(5.0);
        let batch: Vec<VelocityMeasurement> = (0..10)
            .map(|i| VelocityMeasurement {
                velocity: i as f64,
                transit_time: 1e-4,
                arrival_time: i as f64 * 0.01,
                snr_db: if i < 5 { 3.0 } else { 10.0 },
            })
            .collect();
        let added = proc.add_batch(&batch);
        assert_eq!(added, 5); // only SNR >= 5 pass
    }
}
