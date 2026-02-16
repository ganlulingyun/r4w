//! Surface Acoustic Wave (SAW) Processor
//!
//! Signal processing for Surface Acoustic Wave devices including biosensors,
//! gas sensors, RF filters, delay lines, seismology, and NDT applications.
//!
//! SAW devices exploit mechanical waves propagating on the surface of
//! piezoelectric substrates. An Interdigital Transducer (IDT) converts
//! electrical signals to acoustic waves and vice versa. The frequency
//! response of an IDT is governed by the number of finger pairs and the
//! ratio of acoustic velocity to finger pitch.
//!
//! ## Key physics
//!
//! - **Resonance**: f0 = v_SAW / (2 * p), where p is IDT pitch (finger spacing)
//! - **Sauerbrey equation**: Δf = −2 f0² Δm / (A √(ρ_q μ_q))
//! - **IDT frequency response**: H(f) = sin(N π f p / v) / (N sin(π f p / v))
//! - **Insertion loss**: IL = −20 log10(|S21|)
//! - **Rayleigh velocity**: v_R ≈ v_S (0.87 + 1.12 ν) / (1 + ν)
//! - **TCF**: f(T) = f0 (1 + TCF1 ΔT + TCF2 ΔT²)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::surface_acoustic_wave_processor::{
//!     SawConfig, SubstrateMaterial, MassLoadingDetector, IdtResponseCalculator,
//! };
//!
//! // Configure a 100 MHz SAW biosensor on LiNbO3 YZ
//! let config = SawConfig::new(SubstrateMaterial::LiNbO3Yz, 100.0e6, 50);
//! assert!((config.pitch() - 17.44e-6).abs() < 0.01e-6);
//!
//! // Compute IDT frequency response
//! let idt = IdtResponseCalculator::new(&config);
//! let h_center = idt.response(config.center_freq());
//! assert!((h_center - 1.0).abs() < 1e-6); // peak at center
//!
//! // Detect mass loading
//! let detector = MassLoadingDetector::new(&config, 1e-4);
//! let delta_f = detector.frequency_shift(1e-9); // 1 ng
//! assert!(delta_f < 0.0); // frequency decreases with added mass
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Substrate material definitions
// ---------------------------------------------------------------------------

/// Piezoelectric substrate material for SAW device.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SubstrateMaterial {
    /// Lithium niobate Y-cut Z-propagation. v = 3488 m/s, TCF ≈ −75 ppm/°C.
    LiNbO3Yz,
    /// ST-cut quartz. v = 3158 m/s, TCF ≈ 0 ppm/°C (temperature stable).
    QuartzSt,
    /// 128° rotated Y-cut lithium niobate. v = 3992 m/s, TCF ≈ −72 ppm/°C.
    LiNbO3_128Yx,
    /// Lithium tantalate 36° Y-cut. v = 4160 m/s, TCF ≈ −30 ppm/°C.
    LiTaO3_36Yx,
    /// Custom substrate with user-defined velocity and TCF.
    Custom {
        /// SAW velocity in m/s.
        velocity: f64,
        /// First-order TCF in ppm/°C.
        tcf1_ppm: f64,
        /// Second-order TCF in ppm/°C².
        tcf2_ppm: f64,
        /// Density in kg/m³.
        density: f64,
    },
}

impl SubstrateMaterial {
    /// Surface acoustic wave velocity in m/s.
    pub fn velocity(&self) -> f64 {
        match self {
            Self::LiNbO3Yz => 3488.0,
            Self::QuartzSt => 3158.0,
            Self::LiNbO3_128Yx => 3992.0,
            Self::LiTaO3_36Yx => 4160.0,
            Self::Custom { velocity, .. } => *velocity,
        }
    }

    /// First-order temperature coefficient of frequency in ppm/°C.
    pub fn tcf1_ppm(&self) -> f64 {
        match self {
            Self::LiNbO3Yz => -75.0,
            Self::QuartzSt => 0.0,
            Self::LiNbO3_128Yx => -72.0,
            Self::LiTaO3_36Yx => -30.0,
            Self::Custom { tcf1_ppm, .. } => *tcf1_ppm,
        }
    }

    /// Second-order temperature coefficient of frequency in ppm/°C².
    pub fn tcf2_ppm(&self) -> f64 {
        match self {
            Self::LiNbO3Yz => -0.034,
            Self::QuartzSt => -0.034,
            Self::LiNbO3_128Yx => -0.034,
            Self::LiTaO3_36Yx => -0.020,
            Self::Custom { tcf2_ppm, .. } => *tcf2_ppm,
        }
    }

    /// Substrate density in kg/m³.
    pub fn density(&self) -> f64 {
        match self {
            Self::LiNbO3Yz | Self::LiNbO3_128Yx => 4628.0,
            Self::QuartzSt => 2650.0,
            Self::LiTaO3_36Yx => 7456.0,
            Self::Custom { density, .. } => *density,
        }
    }
}

// ---------------------------------------------------------------------------
// SAW device configuration
// ---------------------------------------------------------------------------

/// Configuration for a SAW device.
///
/// Defines the substrate, centre frequency, and IDT geometry.
/// The IDT pitch is derived from the resonance condition f0 = v / (2p).
#[derive(Debug, Clone)]
pub struct SawConfig {
    /// Substrate material.
    substrate: SubstrateMaterial,
    /// Design center frequency in Hz.
    center_freq: f64,
    /// Number of IDT finger pairs (N).
    finger_pairs: usize,
    /// IDT pitch (finger spacing) in metres, derived from f0 and v.
    pitch: f64,
}

impl SawConfig {
    /// Create a new SAW configuration.
    ///
    /// * `substrate` — piezoelectric material
    /// * `center_freq` — design centre frequency in Hz (> 0)
    /// * `finger_pairs` — number of IDT finger pairs (≥ 1)
    pub fn new(substrate: SubstrateMaterial, center_freq: f64, finger_pairs: usize) -> Self {
        assert!(center_freq > 0.0, "center frequency must be positive");
        assert!(finger_pairs >= 1, "need at least 1 finger pair");
        let v = substrate.velocity();
        let pitch = v / (2.0 * center_freq);
        Self {
            substrate,
            center_freq,
            finger_pairs,
            pitch,
        }
    }

    /// Create a SAW configuration from a specific pitch.
    pub fn from_pitch(substrate: SubstrateMaterial, pitch: f64, finger_pairs: usize) -> Self {
        assert!(pitch > 0.0, "pitch must be positive");
        assert!(finger_pairs >= 1, "need at least 1 finger pair");
        let v = substrate.velocity();
        let center_freq = v / (2.0 * pitch);
        Self {
            substrate,
            center_freq,
            finger_pairs,
            pitch,
        }
    }

    /// Substrate material.
    pub fn substrate(&self) -> SubstrateMaterial {
        self.substrate
    }

    /// Design center frequency in Hz.
    pub fn center_freq(&self) -> f64 {
        self.center_freq
    }

    /// Number of IDT finger pairs.
    pub fn finger_pairs(&self) -> usize {
        self.finger_pairs
    }

    /// IDT pitch (finger spacing) in metres.
    pub fn pitch(&self) -> f64 {
        self.pitch
    }

    /// SAW velocity in m/s.
    pub fn velocity(&self) -> f64 {
        self.substrate.velocity()
    }

    /// Approximate fractional 3-dB bandwidth ≈ 0.89 / N.
    pub fn fractional_bandwidth(&self) -> f64 {
        0.89 / self.finger_pairs as f64
    }

    /// Approximate 3-dB bandwidth in Hz.
    pub fn bandwidth_hz(&self) -> f64 {
        self.fractional_bandwidth() * self.center_freq
    }

    /// Wavelength at centre frequency in metres (= 2 × pitch).
    pub fn wavelength(&self) -> f64 {
        2.0 * self.pitch
    }
}

// ---------------------------------------------------------------------------
// Resonance model
// ---------------------------------------------------------------------------

/// Computes SAW resonance frequency from velocity and IDT pitch.
///
/// f0 = v_SAW / (2 p)
///
/// Also provides harmonic frequencies: f_n = (2n−1) f0 for odd harmonics.
pub struct SawResonanceModel {
    velocity: f64,
    pitch: f64,
    f0: f64,
}

impl SawResonanceModel {
    /// Create from explicit velocity and pitch.
    pub fn new(velocity: f64, pitch: f64) -> Self {
        assert!(velocity > 0.0 && pitch > 0.0);
        let f0 = velocity / (2.0 * pitch);
        Self {
            velocity,
            pitch,
            f0,
        }
    }

    /// Create from a `SawConfig`.
    pub fn from_config(cfg: &SawConfig) -> Self {
        Self::new(cfg.velocity(), cfg.pitch())
    }

    /// Fundamental resonance frequency in Hz.
    pub fn fundamental(&self) -> f64 {
        self.f0
    }

    /// n-th odd harmonic frequency: f_n = (2n−1) f0.
    ///
    /// `n` = 1 → fundamental, `n` = 2 → 3rd harmonic, etc.
    pub fn harmonic(&self, n: usize) -> f64 {
        assert!(n >= 1);
        (2 * n - 1) as f64 * self.f0
    }

    /// Pitch required for a given target frequency.
    pub fn pitch_for_freq(&self, freq: f64) -> f64 {
        self.velocity / (2.0 * freq)
    }

    /// Velocity used by this model.
    pub fn velocity(&self) -> f64 {
        self.velocity
    }

    /// Pitch used by this model.
    pub fn pitch(&self) -> f64 {
        self.pitch
    }
}

// ---------------------------------------------------------------------------
// Mass loading detector (Sauerbrey equation)
// ---------------------------------------------------------------------------

/// Detects frequency shifts due to mass loading using the Sauerbrey equation.
///
/// Δf = −2 f0² Δm / (A √(ρ_q μ_q))
///
/// where:
/// - f0: fundamental resonance frequency (Hz)
/// - Δm: mass change (kg)
/// - A: active area (m²)
/// - ρ_q: quartz density (2650 kg/m³)
/// - μ_q: shear modulus of AT-cut quartz (2.947 × 10¹⁰ Pa)
///
/// For SAW sensors, an equivalent sensitivity is used:
/// Δf / f0 = −Δm / (ρ h A) × k_s
///
/// This implementation uses the Sauerbrey constant approach.
pub struct MassLoadingDetector {
    f0: f64,
    active_area: f64,
    /// Sauerbrey constant = 2 f0² / (A √(ρ_q μ_q))
    sauerbrey_const: f64,
}

/// Quartz density in kg/m³ (AT-cut).
const QUARTZ_DENSITY: f64 = 2650.0;
/// Shear modulus of AT-cut quartz in Pa.
const QUARTZ_SHEAR_MODULUS: f64 = 2.947e10;

impl MassLoadingDetector {
    /// Create a new mass loading detector.
    ///
    /// * `config` — SAW device configuration (provides f0)
    /// * `active_area` — sensing area in m² (> 0)
    pub fn new(config: &SawConfig, active_area: f64) -> Self {
        assert!(active_area > 0.0, "active area must be positive");
        let f0 = config.center_freq();
        let denominator = active_area * (QUARTZ_DENSITY * QUARTZ_SHEAR_MODULUS).sqrt();
        let sauerbrey_const = 2.0 * f0 * f0 / denominator;
        Self {
            f0,
            active_area,
            sauerbrey_const,
        }
    }

    /// Frequency shift in Hz for a given mass change in kg.
    ///
    /// Returns a negative value for mass addition (frequency decreases).
    pub fn frequency_shift(&self, delta_m: f64) -> f64 {
        -self.sauerbrey_const * delta_m
    }

    /// Mass change in kg that would produce a given frequency shift.
    pub fn mass_from_shift(&self, delta_f: f64) -> f64 {
        -delta_f / self.sauerbrey_const
    }

    /// Mass sensitivity in Hz/kg.
    pub fn sensitivity(&self) -> f64 {
        self.sauerbrey_const
    }

    /// Mass sensitivity in Hz/ng.
    pub fn sensitivity_hz_per_ng(&self) -> f64 {
        self.sauerbrey_const * 1e-9
    }

    /// Fundamental frequency in Hz.
    pub fn f0(&self) -> f64 {
        self.f0
    }

    /// Active area in m².
    pub fn active_area(&self) -> f64 {
        self.active_area
    }
}

// ---------------------------------------------------------------------------
// IDT frequency response calculator
// ---------------------------------------------------------------------------

/// Computes the frequency response of an Interdigital Transducer (IDT).
///
/// H(f) = sin(N π Δf / f0) / (N sin(π Δf / f0))
///
/// where Δf = f − f0 and f0 = v/(2p) is the centre frequency.
///
/// This is a Dirichlet kernel (periodic sinc) centred at f0 with:
/// - Peak of 1.0 at f = f0
/// - First nulls at f0 ± f0/N
/// - Sidelobes at approximately −13.3 dB (unweighted)
pub struct IdtResponseCalculator {
    n: f64,
    f0: f64,
}

impl IdtResponseCalculator {
    /// Create from a `SawConfig`.
    pub fn new(config: &SawConfig) -> Self {
        Self {
            n: config.finger_pairs() as f64,
            f0: config.center_freq(),
        }
    }

    /// Create from explicit parameters.
    pub fn from_params(finger_pairs: usize, pitch: f64, velocity: f64) -> Self {
        let f0 = velocity / (2.0 * pitch);
        Self {
            n: finger_pairs as f64,
            f0,
        }
    }

    /// Magnitude of the IDT frequency response at frequency `f` Hz.
    ///
    /// Returns |H(f)| in [0, 1].
    pub fn response(&self, f: f64) -> f64 {
        let x = PI * (f - self.f0) / self.f0;
        let sin_x = x.sin();
        if sin_x.abs() < 1e-15 {
            // At f = f0 (and its periodic repetitions), use L'Hôpital: limit = 1.0
            1.0
        } else {
            let num = (self.n * x).sin();
            (num / (self.n * sin_x)).abs()
        }
    }

    /// Magnitude in dB.
    pub fn response_db(&self, f: f64) -> f64 {
        let mag = self.response(f);
        if mag < 1e-30 {
            -300.0
        } else {
            20.0 * mag.log10()
        }
    }

    /// Compute frequency response over a range of frequencies.
    ///
    /// Returns Vec of (frequency, magnitude) pairs.
    pub fn sweep(&self, f_start: f64, f_stop: f64, num_points: usize) -> Vec<(f64, f64)> {
        assert!(num_points >= 2);
        let step = (f_stop - f_start) / (num_points - 1) as f64;
        (0..num_points)
            .map(|i| {
                let f = f_start + i as f64 * step;
                (f, self.response(f))
            })
            .collect()
    }

    /// Compute the -3 dB bandwidth numerically.
    pub fn bandwidth_3db(&self, center: f64) -> f64 {
        let threshold = 1.0 / 2.0_f64.sqrt(); // -3 dB
        let step = center / (self.n * 100.0);
        // Search upward from center
        let mut f_upper = center;
        while self.response(f_upper) > threshold && f_upper < center * 2.0 {
            f_upper += step;
        }
        // Search downward from center
        let mut f_lower = center;
        while self.response(f_lower) > threshold && f_lower > 0.0 {
            f_lower -= step;
        }
        f_upper - f_lower
    }
}

// ---------------------------------------------------------------------------
// Insertion loss calculator
// ---------------------------------------------------------------------------

/// Computes insertion loss from S-parameter data.
///
/// IL = −20 log10(|S21|)
///
/// S21 is the forward transmission coefficient, typically measured with
/// a vector network analyzer.
pub struct InsertionLossCalculator;

impl InsertionLossCalculator {
    /// Insertion loss in dB from S21 magnitude (linear).
    pub fn from_s21_magnitude(s21_mag: f64) -> f64 {
        assert!(s21_mag >= 0.0, "S21 magnitude cannot be negative");
        if s21_mag < 1e-30 {
            return 600.0; // effectively infinite loss
        }
        -20.0 * s21_mag.log10()
    }

    /// Insertion loss in dB from complex S21.
    pub fn from_s21_complex(s21_real: f64, s21_imag: f64) -> f64 {
        let mag = (s21_real * s21_real + s21_imag * s21_imag).sqrt();
        Self::from_s21_magnitude(mag)
    }

    /// S21 magnitude from insertion loss in dB.
    pub fn s21_from_il(il_db: f64) -> f64 {
        10.0_f64.powf(-il_db / 20.0)
    }

    /// Return loss in dB from S11 magnitude.
    pub fn return_loss(s11_mag: f64) -> f64 {
        assert!(s11_mag >= 0.0);
        if s11_mag < 1e-30 {
            return 600.0;
        }
        -20.0 * s11_mag.log10()
    }

    /// Compute insertion loss over a frequency sweep of S21 data.
    ///
    /// Input: slice of (frequency, s21_magnitude) pairs.
    /// Output: Vec of (frequency, IL_dB).
    pub fn sweep(s21_data: &[(f64, f64)]) -> Vec<(f64, f64)> {
        s21_data
            .iter()
            .map(|&(f, mag)| (f, Self::from_s21_magnitude(mag)))
            .collect()
    }

    /// Minimum insertion loss (best passband point) from a sweep.
    pub fn min_il(s21_data: &[(f64, f64)]) -> Option<(f64, f64)> {
        s21_data
            .iter()
            .map(|&(f, mag)| (f, Self::from_s21_magnitude(mag)))
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
    }
}

// ---------------------------------------------------------------------------
// Group delay extractor
// ---------------------------------------------------------------------------

/// Extracts group delay from frequency-domain S21 phase data.
///
/// τ_g(f) = −dφ/dω = −(1/2π) dφ/df
///
/// Uses central finite differences on the unwrapped phase.
pub struct GroupDelayExtractor;

impl GroupDelayExtractor {
    /// Unwrap phase to remove 2π discontinuities.
    pub fn unwrap_phase(phase: &[f64]) -> Vec<f64> {
        if phase.is_empty() {
            return vec![];
        }
        let mut unwrapped = vec![phase[0]];
        for i in 1..phase.len() {
            let mut diff = phase[i] - phase[i - 1];
            // Wrap to (-π, π]
            while diff > PI {
                diff -= 2.0 * PI;
            }
            while diff <= -PI {
                diff += 2.0 * PI;
            }
            unwrapped.push(unwrapped[i - 1] + diff);
        }
        unwrapped
    }

    /// Compute group delay from (frequency, phase) data.
    ///
    /// Phase should be in radians. Returns (frequency, group_delay_seconds) pairs.
    /// Uses central finite differences where possible, forward/backward at edges.
    pub fn compute(freq_phase: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n = freq_phase.len();
        if n < 2 {
            return vec![];
        }

        let freqs: Vec<f64> = freq_phase.iter().map(|&(f, _)| f).collect();
        let phases: Vec<f64> = freq_phase.iter().map(|&(_, p)| p).collect();
        let unwrapped = Self::unwrap_phase(&phases);

        let mut result = Vec::with_capacity(n);
        for i in 0..n {
            let tau = if i == 0 {
                // Forward difference
                let df = freqs[1] - freqs[0];
                let dp = unwrapped[1] - unwrapped[0];
                -dp / (2.0 * PI * df)
            } else if i == n - 1 {
                // Backward difference
                let df = freqs[n - 1] - freqs[n - 2];
                let dp = unwrapped[n - 1] - unwrapped[n - 2];
                -dp / (2.0 * PI * df)
            } else {
                // Central difference
                let df = freqs[i + 1] - freqs[i - 1];
                let dp = unwrapped[i + 1] - unwrapped[i - 1];
                -dp / (2.0 * PI * df)
            };
            result.push((freqs[i], tau));
        }
        result
    }

    /// Compute group delay from complex S21 data: (frequency, real, imag).
    pub fn from_complex_s21(data: &[(f64, f64, f64)]) -> Vec<(f64, f64)> {
        let freq_phase: Vec<(f64, f64)> = data
            .iter()
            .map(|&(f, re, im)| (f, im.atan2(re)))
            .collect();
        Self::compute(&freq_phase)
    }

    /// Average group delay over a frequency range.
    pub fn average_delay(group_delay: &[(f64, f64)], f_low: f64, f_high: f64) -> f64 {
        let in_band: Vec<f64> = group_delay
            .iter()
            .filter(|&&(f, _)| f >= f_low && f <= f_high)
            .map(|&(_, d)| d)
            .collect();
        if in_band.is_empty() {
            return 0.0;
        }
        in_band.iter().sum::<f64>() / in_band.len() as f64
    }

    /// Group delay ripple (max − min) over a frequency range.
    pub fn delay_ripple(group_delay: &[(f64, f64)], f_low: f64, f_high: f64) -> f64 {
        let in_band: Vec<f64> = group_delay
            .iter()
            .filter(|&&(f, _)| f >= f_low && f <= f_high)
            .map(|&(_, d)| d)
            .collect();
        if in_band.len() < 2 {
            return 0.0;
        }
        let min = in_band.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = in_band.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        max - min
    }
}

// ---------------------------------------------------------------------------
// Sensor response tracker
// ---------------------------------------------------------------------------

/// Tracks resonance frequency shift over time for SAW sensor applications.
///
/// Maintains a time-series of measurements and provides smoothed output
/// using an exponential moving average.
#[derive(Debug, Clone)]
pub struct SensorResponseTracker {
    /// Time series of (timestamp_seconds, frequency_hz) measurements.
    measurements: Vec<(f64, f64)>,
    /// Baseline frequency in Hz.
    baseline_freq: f64,
    /// EMA smoothing factor (0 < α ≤ 1).
    alpha: f64,
    /// Current smoothed frequency.
    smoothed: f64,
}

impl SensorResponseTracker {
    /// Create a new tracker with a baseline frequency and EMA smoothing.
    ///
    /// * `baseline_freq` — reference frequency in Hz
    /// * `alpha` — EMA smoothing factor, 0 < α ≤ 1 (1 = no smoothing)
    pub fn new(baseline_freq: f64, alpha: f64) -> Self {
        let alpha = alpha.clamp(0.001, 1.0);
        Self {
            measurements: Vec::new(),
            baseline_freq,
            alpha,
            smoothed: baseline_freq,
        }
    }

    /// Add a measurement (timestamp in seconds, frequency in Hz).
    pub fn add_measurement(&mut self, time: f64, frequency: f64) {
        self.measurements.push((time, frequency));
        self.smoothed = self.alpha * frequency + (1.0 - self.alpha) * self.smoothed;
    }

    /// Current frequency shift from baseline in Hz.
    pub fn current_shift(&self) -> f64 {
        self.smoothed - self.baseline_freq
    }

    /// Current smoothed frequency in Hz.
    pub fn current_frequency(&self) -> f64 {
        self.smoothed
    }

    /// Relative frequency shift (Δf / f0).
    pub fn relative_shift(&self) -> f64 {
        (self.smoothed - self.baseline_freq) / self.baseline_freq
    }

    /// Baseline frequency.
    pub fn baseline(&self) -> f64 {
        self.baseline_freq
    }

    /// Update baseline to current smoothed value.
    pub fn reset_baseline(&mut self) {
        self.baseline_freq = self.smoothed;
    }

    /// Number of measurements recorded.
    pub fn num_measurements(&self) -> usize {
        self.measurements.len()
    }

    /// All raw measurements as (time, frequency) pairs.
    pub fn measurements(&self) -> &[(f64, f64)] {
        &self.measurements
    }

    /// Compute the drift rate (Hz/s) using linear regression on raw measurements.
    pub fn drift_rate(&self) -> f64 {
        let n = self.measurements.len();
        if n < 2 {
            return 0.0;
        }
        let n_f = n as f64;
        let sum_t: f64 = self.measurements.iter().map(|m| m.0).sum();
        let sum_f: f64 = self.measurements.iter().map(|m| m.1).sum();
        let sum_tf: f64 = self.measurements.iter().map(|m| m.0 * m.1).sum();
        let sum_t2: f64 = self.measurements.iter().map(|m| m.0 * m.0).sum();
        let denom = n_f * sum_t2 - sum_t * sum_t;
        if denom.abs() < 1e-30 {
            return 0.0;
        }
        (n_f * sum_tf - sum_t * sum_f) / denom
    }
}

// ---------------------------------------------------------------------------
// Temperature compensator
// ---------------------------------------------------------------------------

/// Compensates SAW frequency for temperature using TCF coefficients.
///
/// f(T) = f0 × (1 + TCF1 × ΔT + TCF2 × ΔT²)
///
/// where ΔT = T − T_ref (typically T_ref = 25 °C).
#[derive(Debug, Clone)]
pub struct TemperatureCompensator {
    /// Reference temperature in °C.
    t_ref: f64,
    /// First-order TCF (per °C, not ppm).
    tcf1: f64,
    /// Second-order TCF (per °C², not ppm).
    tcf2: f64,
}

impl TemperatureCompensator {
    /// Create from a substrate material with default reference temperature 25 °C.
    pub fn from_substrate(substrate: SubstrateMaterial) -> Self {
        Self {
            t_ref: 25.0,
            tcf1: substrate.tcf1_ppm() * 1e-6,
            tcf2: substrate.tcf2_ppm() * 1e-6,
        }
    }

    /// Create with explicit TCF values.
    ///
    /// * `tcf1_ppm` — first-order TCF in ppm/°C
    /// * `tcf2_ppm` — second-order TCF in ppm/°C²
    /// * `t_ref` — reference temperature in °C
    pub fn new(tcf1_ppm: f64, tcf2_ppm: f64, t_ref: f64) -> Self {
        Self {
            t_ref,
            tcf1: tcf1_ppm * 1e-6,
            tcf2: tcf2_ppm * 1e-6,
        }
    }

    /// Predicted frequency at temperature T given base frequency f0.
    pub fn compensated_freq(&self, f0: f64, temperature: f64) -> f64 {
        let dt = temperature - self.t_ref;
        f0 * (1.0 + self.tcf1 * dt + self.tcf2 * dt * dt)
    }

    /// Frequency shift due to temperature change from reference.
    pub fn freq_shift(&self, f0: f64, temperature: f64) -> f64 {
        self.compensated_freq(f0, temperature) - f0
    }

    /// Correct a measured frequency back to the reference temperature.
    ///
    /// Given f_meas at temperature T, return what f would be at T_ref.
    pub fn correct_to_reference(&self, f_meas: f64, temperature: f64) -> f64 {
        let dt = temperature - self.t_ref;
        let scale = 1.0 + self.tcf1 * dt + self.tcf2 * dt * dt;
        f_meas / scale
    }

    /// Reference temperature in °C.
    pub fn reference_temp(&self) -> f64 {
        self.t_ref
    }

    /// Turnover temperature (temperature at which df/dT = 0).
    ///
    /// Only meaningful when tcf2 ≠ 0. Returns None if no turnover exists.
    pub fn turnover_temperature(&self) -> Option<f64> {
        if self.tcf2.abs() < 1e-30 {
            return None;
        }
        let dt = -self.tcf1 / (2.0 * self.tcf2);
        Some(self.t_ref + dt)
    }
}

// ---------------------------------------------------------------------------
// Love wave processor
// ---------------------------------------------------------------------------

/// Processes Love wave (SH surface mode) in layered structures.
///
/// Love waves exist in a slow guiding layer on a fast substrate. They are
/// shear-horizontal (SH) modes with no sagittal component, making them
/// ideal for liquid-phase biosensors (no energy loss into the fluid).
///
/// Dispersion relation: tan(k_L h) = μ_S κ_S / (μ_L κ_L)
///
/// where:
/// - h = guiding layer thickness
/// - k_L = ω/v_L (layer wavenumber)
/// - κ_S = sqrt((ω/v_S)² − β²), κ_L = sqrt(β² − (ω/v_L)²)
/// - β = ω/v_phase is the propagation constant
#[derive(Debug, Clone)]
pub struct LoveWaveProcessor {
    /// Shear velocity of guiding layer in m/s.
    v_layer: f64,
    /// Shear velocity of substrate in m/s.
    v_substrate: f64,
    /// Density of guiding layer in kg/m³.
    rho_layer: f64,
    /// Density of substrate in kg/m³.
    rho_substrate: f64,
    /// Shear modulus of guiding layer = rho * v².
    mu_layer: f64,
    /// Shear modulus of substrate = rho * v².
    mu_substrate: f64,
}

impl LoveWaveProcessor {
    /// Create a Love wave processor.
    ///
    /// * `v_layer` — shear velocity of the guiding layer (m/s)
    /// * `v_substrate` — shear velocity of the substrate (m/s), must be > v_layer
    /// * `rho_layer` — density of guiding layer (kg/m³)
    /// * `rho_substrate` — density of substrate (kg/m³)
    pub fn new(
        v_layer: f64,
        v_substrate: f64,
        rho_layer: f64,
        rho_substrate: f64,
    ) -> Self {
        assert!(
            v_substrate > v_layer,
            "substrate must be faster than guiding layer for Love wave trapping"
        );
        Self {
            v_layer,
            v_substrate,
            rho_layer,
            rho_substrate,
            mu_layer: rho_layer * v_layer * v_layer,
            mu_substrate: rho_substrate * v_substrate * v_substrate,
        }
    }

    /// Solve for the Love wave phase velocity given frequency and layer thickness.
    ///
    /// Uses bisection on the dispersion relation.
    /// Returns phase velocity in m/s, or None if no solution found.
    pub fn phase_velocity(&self, freq: f64, thickness: f64) -> Option<f64> {
        let omega = 2.0 * PI * freq;
        // Phase velocity is between v_layer and v_substrate
        let v_lo = self.v_layer * 1.0001;
        let v_hi = self.v_substrate * 0.9999;

        // Dispersion function: F(v) = tan(κ_L h) - μ_S κ_S / (μ_L κ_L) = 0
        // where κ_L = sqrt((ω/v_L)² − (ω/v)²), κ_S = sqrt((ω/v)² − (ω/v_S)²)
        let dispersion = |v: f64| -> f64 {
            let beta = omega / v;
            let kl_sq = (omega / self.v_layer).powi(2) - beta * beta;
            let ks_sq = beta * beta - (omega / self.v_substrate).powi(2);
            if kl_sq <= 0.0 || ks_sq <= 0.0 {
                return f64::NAN;
            }
            let kl = kl_sq.sqrt();
            let ks = ks_sq.sqrt();
            (kl * thickness).tan() - self.mu_substrate * ks / (self.mu_layer * kl)
        };

        // Bisection search
        let mut lo = v_lo;
        let mut hi = v_hi;
        let f_lo = dispersion(lo);
        let f_hi = dispersion(hi);
        if f_lo.is_nan() || f_hi.is_nan() {
            return None;
        }
        if f_lo * f_hi > 0.0 {
            // Try a coarse scan to find a sign change
            let steps = 1000;
            let dv = (hi - lo) / steps as f64;
            let mut found = false;
            for i in 0..steps {
                let va = lo + i as f64 * dv;
                let vb = va + dv;
                let fa = dispersion(va);
                let fb = dispersion(vb);
                if fa.is_nan() || fb.is_nan() {
                    continue;
                }
                if fa * fb <= 0.0 {
                    lo = va;
                    hi = vb;
                    found = true;
                    break;
                }
            }
            if !found {
                return None;
            }
        }

        // Bisection
        for _ in 0..100 {
            let mid = 0.5 * (lo + hi);
            let f_mid = dispersion(mid);
            if f_mid.is_nan() || (hi - lo) < 1e-10 * mid {
                return Some(mid);
            }
            let f_lo = dispersion(lo);
            if f_lo * f_mid <= 0.0 {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        Some(0.5 * (lo + hi))
    }

    /// Optimal guiding layer thickness for maximum mass sensitivity.
    ///
    /// Approximately h_opt ≈ λ_L / 4 where λ_L = v_layer / freq.
    pub fn optimal_thickness(&self, freq: f64) -> f64 {
        self.v_layer / (4.0 * freq)
    }

    /// Layer thickness normalised to wavelength: h / λ.
    pub fn normalized_thickness(&self, freq: f64, thickness: f64) -> f64 {
        let wavelength = self.v_layer / freq;
        thickness / wavelength
    }

    /// Guiding layer velocity.
    pub fn layer_velocity(&self) -> f64 {
        self.v_layer
    }

    /// Substrate velocity.
    pub fn substrate_velocity(&self) -> f64 {
        self.v_substrate
    }
}

// ---------------------------------------------------------------------------
// Rayleigh wave dispersion
// ---------------------------------------------------------------------------

/// Computes Rayleigh wave velocity as a function of Poisson's ratio.
///
/// The Rayleigh wave characteristic equation yields:
///
/// v_R ≈ v_S × (0.87 + 1.12 ν) / (1 + ν)
///
/// where v_S is the shear wave velocity and ν is Poisson's ratio.
/// This is Viktorov's approximation, accurate to ~0.5% for 0 ≤ ν ≤ 0.5.
pub struct RayleighWaveDispersion;

impl RayleighWaveDispersion {
    /// Rayleigh wave velocity from shear velocity and Poisson's ratio.
    ///
    /// Uses Viktorov's approximation: v_R ≈ v_S (0.87 + 1.12ν) / (1 + ν)
    pub fn velocity(v_shear: f64, poisson: f64) -> f64 {
        assert!(
            (0.0..=0.5).contains(&poisson),
            "Poisson's ratio must be in [0, 0.5]"
        );
        v_shear * (0.87 + 1.12 * poisson) / (1.0 + poisson)
    }

    /// Ratio v_R / v_S.
    pub fn velocity_ratio(poisson: f64) -> f64 {
        (0.87 + 1.12 * poisson) / (1.0 + poisson)
    }

    /// Shear velocity from compressional velocity and Poisson's ratio.
    ///
    /// v_S = v_P × √((1 − 2ν) / (2(1 − ν)))
    pub fn shear_from_compressional(v_p: f64, poisson: f64) -> f64 {
        assert!((0.0..0.5).contains(&poisson));
        v_p * ((1.0 - 2.0 * poisson) / (2.0 * (1.0 - poisson))).sqrt()
    }

    /// Rayleigh velocity directly from compressional velocity and Poisson's ratio.
    pub fn from_compressional(v_p: f64, poisson: f64) -> f64 {
        let v_s = Self::shear_from_compressional(v_p, poisson);
        Self::velocity(v_s, poisson)
    }

    /// Compute Rayleigh velocity vs Poisson's ratio curve.
    ///
    /// Returns Vec of (poisson_ratio, v_R / v_S).
    pub fn dispersion_curve(num_points: usize) -> Vec<(f64, f64)> {
        assert!(num_points >= 2);
        let step = 0.5 / (num_points - 1) as f64;
        (0..num_points)
            .map(|i| {
                let nu = i as f64 * step;
                (nu, Self::velocity_ratio(nu))
            })
            .collect()
    }

    /// Penetration depth of Rayleigh wave ≈ 1 wavelength.
    pub fn penetration_depth(v_rayleigh: f64, freq: f64) -> f64 {
        v_rayleigh / freq
    }
}

// ---------------------------------------------------------------------------
// Helper: SAW filter design utilities
// ---------------------------------------------------------------------------

/// Compute the number of finger pairs needed for a target fractional bandwidth.
///
/// BW/f0 ≈ 0.89/N → N ≈ 0.89/BW_frac
pub fn finger_pairs_for_bandwidth(fractional_bw: f64) -> usize {
    assert!(fractional_bw > 0.0 && fractional_bw <= 1.0);
    (0.89 / fractional_bw).ceil() as usize
}

/// Compute IDT pitch for a given frequency and substrate.
pub fn pitch_for_frequency(freq: f64, substrate: SubstrateMaterial) -> f64 {
    substrate.velocity() / (2.0 * freq)
}

/// SAW delay line time delay.
///
/// t = L / v_SAW where L is the propagation path length.
pub fn delay_line_time(path_length: f64, velocity: f64) -> f64 {
    path_length / velocity
}

/// Required path length for a given delay.
pub fn path_length_for_delay(delay: f64, velocity: f64) -> f64 {
    delay * velocity
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-6;

    // --- SubstrateMaterial ---

    #[test]
    fn test_linbo3_yz_velocity() {
        assert!((SubstrateMaterial::LiNbO3Yz.velocity() - 3488.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_quartz_st_velocity() {
        assert!((SubstrateMaterial::QuartzSt.velocity() - 3158.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_linbo3_128yx_velocity() {
        assert!((SubstrateMaterial::LiNbO3_128Yx.velocity() - 3992.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_litao3_velocity() {
        assert!((SubstrateMaterial::LiTaO3_36Yx.velocity() - 4160.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_custom_substrate() {
        let s = SubstrateMaterial::Custom {
            velocity: 5000.0,
            tcf1_ppm: -10.0,
            tcf2_ppm: -0.01,
            density: 3000.0,
        };
        assert!((s.velocity() - 5000.0).abs() < TOLERANCE);
        assert!((s.tcf1_ppm() - (-10.0)).abs() < TOLERANCE);
        assert!((s.density() - 3000.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_quartz_tcf_near_zero() {
        // ST-quartz is famous for near-zero TCF
        assert!(SubstrateMaterial::QuartzSt.tcf1_ppm().abs() < 1.0);
    }

    // --- SawConfig ---

    #[test]
    fn test_config_pitch() {
        // f0 = 100 MHz on LiNbO3 YZ: p = 3488 / (2 * 100e6) = 17.44 μm
        let cfg = SawConfig::new(SubstrateMaterial::LiNbO3Yz, 100.0e6, 50);
        let expected_pitch = 3488.0 / (2.0 * 100.0e6);
        assert!((cfg.pitch() - expected_pitch).abs() < 1e-12);
    }

    #[test]
    fn test_config_from_pitch() {
        let pitch = 17.44e-6;
        let cfg = SawConfig::from_pitch(SubstrateMaterial::LiNbO3Yz, pitch, 50);
        assert!((cfg.center_freq() - 3488.0 / (2.0 * pitch)).abs() < 1.0);
    }

    #[test]
    fn test_config_wavelength() {
        let cfg = SawConfig::new(SubstrateMaterial::QuartzSt, 200.0e6, 100);
        assert!((cfg.wavelength() - 2.0 * cfg.pitch()).abs() < 1e-15);
    }

    #[test]
    fn test_fractional_bandwidth() {
        let cfg = SawConfig::new(SubstrateMaterial::QuartzSt, 100.0e6, 89);
        assert!((cfg.fractional_bandwidth() - 0.01).abs() < 0.001);
    }

    // --- SawResonanceModel ---

    #[test]
    fn test_resonance_fundamental() {
        let model = SawResonanceModel::new(3488.0, 17.44e-6);
        let expected = 3488.0 / (2.0 * 17.44e-6);
        assert!((model.fundamental() - expected).abs() / expected < 1e-6);
    }

    #[test]
    fn test_resonance_harmonics() {
        let model = SawResonanceModel::new(3488.0, 17.44e-6);
        let f0 = model.fundamental();
        assert!((model.harmonic(1) - f0).abs() < TOLERANCE);
        assert!((model.harmonic(2) - 3.0 * f0).abs() < 1.0);
        assert!((model.harmonic(3) - 5.0 * f0).abs() < 1.0);
    }

    #[test]
    fn test_resonance_from_config() {
        let cfg = SawConfig::new(SubstrateMaterial::LiNbO3Yz, 100.0e6, 50);
        let model = SawResonanceModel::from_config(&cfg);
        assert!((model.fundamental() - 100.0e6).abs() < 1.0);
    }

    #[test]
    fn test_pitch_for_freq() {
        let model = SawResonanceModel::new(3488.0, 17.44e-6);
        let p = model.pitch_for_freq(200.0e6);
        let expected = 3488.0 / (2.0 * 200.0e6);
        assert!((p - expected).abs() < 1e-15);
    }

    // --- MassLoadingDetector ---

    #[test]
    fn test_mass_loading_negative_shift() {
        let cfg = SawConfig::new(SubstrateMaterial::LiNbO3Yz, 100.0e6, 50);
        let det = MassLoadingDetector::new(&cfg, 1e-4);
        let df = det.frequency_shift(1e-9); // 1 ng
        assert!(df < 0.0); // frequency decreases
    }

    #[test]
    fn test_mass_loading_zero_mass() {
        let cfg = SawConfig::new(SubstrateMaterial::LiNbO3Yz, 100.0e6, 50);
        let det = MassLoadingDetector::new(&cfg, 1e-4);
        assert!((det.frequency_shift(0.0)).abs() < TOLERANCE);
    }

    #[test]
    fn test_mass_roundtrip() {
        let cfg = SawConfig::new(SubstrateMaterial::LiNbO3Yz, 100.0e6, 50);
        let det = MassLoadingDetector::new(&cfg, 1e-4);
        let mass = 5e-9;
        let df = det.frequency_shift(mass);
        let recovered = det.mass_from_shift(df);
        assert!((recovered - mass).abs() / mass < 1e-10);
    }

    #[test]
    fn test_sensitivity_positive() {
        let cfg = SawConfig::new(SubstrateMaterial::LiNbO3Yz, 100.0e6, 50);
        let det = MassLoadingDetector::new(&cfg, 1e-4);
        assert!(det.sensitivity() > 0.0);
    }

    #[test]
    fn test_higher_freq_more_sensitive() {
        let cfg_lo = SawConfig::new(SubstrateMaterial::LiNbO3Yz, 100.0e6, 50);
        let cfg_hi = SawConfig::new(SubstrateMaterial::LiNbO3Yz, 500.0e6, 50);
        let det_lo = MassLoadingDetector::new(&cfg_lo, 1e-4);
        let det_hi = MassLoadingDetector::new(&cfg_hi, 1e-4);
        // Higher frequency → higher Sauerbrey constant
        assert!(det_hi.sensitivity() > det_lo.sensitivity());
    }

    // --- IdtResponseCalculator ---

    #[test]
    fn test_idt_peak_at_center() {
        let cfg = SawConfig::new(SubstrateMaterial::LiNbO3Yz, 100.0e6, 50);
        let idt = IdtResponseCalculator::new(&cfg);
        let h = idt.response(cfg.center_freq());
        assert!((h - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_idt_response_symmetric() {
        let cfg = SawConfig::new(SubstrateMaterial::LiNbO3Yz, 100.0e6, 50);
        let idt = IdtResponseCalculator::new(&cfg);
        let f0 = cfg.center_freq();
        let df = 0.5e6;
        let h_above = idt.response(f0 + df);
        let h_below = idt.response(f0 - df);
        assert!((h_above - h_below).abs() < 0.01);
    }

    #[test]
    fn test_idt_nulls_near_expected() {
        // First null at approximately f0 ± f0/N
        let cfg = SawConfig::new(SubstrateMaterial::LiNbO3Yz, 100.0e6, 50);
        let idt = IdtResponseCalculator::new(&cfg);
        let f0 = cfg.center_freq();
        let f_null = f0 + f0 / 50.0; // f0 + f0/N
        let h = idt.response(f_null);
        assert!(h < 0.05); // should be near zero
    }

    #[test]
    fn test_idt_sweep() {
        let cfg = SawConfig::new(SubstrateMaterial::LiNbO3Yz, 100.0e6, 50);
        let idt = IdtResponseCalculator::new(&cfg);
        let sweep = idt.sweep(90.0e6, 110.0e6, 201);
        assert_eq!(sweep.len(), 201);
        // Peak should be near center
        let (peak_f, peak_mag) = sweep
            .iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();
        assert!((peak_f - 100.0e6).abs() < 0.2e6);
        assert!(*peak_mag > 0.95);
    }

    #[test]
    fn test_idt_response_db() {
        let cfg = SawConfig::new(SubstrateMaterial::LiNbO3Yz, 100.0e6, 50);
        let idt = IdtResponseCalculator::new(&cfg);
        let db = idt.response_db(cfg.center_freq());
        assert!(db.abs() < 0.01); // 0 dB at center
    }

    #[test]
    fn test_idt_bandwidth_3db() {
        let cfg = SawConfig::new(SubstrateMaterial::LiNbO3Yz, 100.0e6, 50);
        let idt = IdtResponseCalculator::new(&cfg);
        let bw = idt.bandwidth_3db(cfg.center_freq());
        // Should be approximately 0.89/N * f0 = 0.89/50 * 100MHz ≈ 1.78 MHz
        let expected = cfg.bandwidth_hz();
        assert!((bw - expected).abs() / expected < 0.2);
    }

    // --- InsertionLossCalculator ---

    #[test]
    fn test_il_zero_loss() {
        let il = InsertionLossCalculator::from_s21_magnitude(1.0);
        assert!(il.abs() < TOLERANCE);
    }

    #[test]
    fn test_il_half_power() {
        // |S21| = 0.5 → IL = 6.02 dB
        let il = InsertionLossCalculator::from_s21_magnitude(0.5);
        assert!((il - 6.0206).abs() < 0.001);
    }

    #[test]
    fn test_il_from_complex() {
        // S21 = 0.6 + 0.8j → |S21| = 1.0 → IL = 0
        let il = InsertionLossCalculator::from_s21_complex(0.6, 0.8);
        assert!(il.abs() < 0.001);
    }

    #[test]
    fn test_il_roundtrip() {
        let il_db = 3.5;
        let s21 = InsertionLossCalculator::s21_from_il(il_db);
        let recovered = InsertionLossCalculator::from_s21_magnitude(s21);
        assert!((recovered - il_db).abs() < 1e-10);
    }

    #[test]
    fn test_return_loss() {
        let rl = InsertionLossCalculator::return_loss(0.1); // S11 = 0.1 → RL = 20 dB
        assert!((rl - 20.0).abs() < 0.01);
    }

    #[test]
    fn test_il_sweep() {
        let data = vec![(90.0e6, 0.8), (100.0e6, 0.95), (110.0e6, 0.7)];
        let il = InsertionLossCalculator::sweep(&data);
        assert_eq!(il.len(), 3);
        // Min IL at the highest S21
        let min = InsertionLossCalculator::min_il(&data).unwrap();
        assert!((min.0 - 100.0e6).abs() < TOLERANCE);
    }

    // --- GroupDelayExtractor ---

    #[test]
    fn test_unwrap_phase() {
        // Linear phase with wrapping
        let phase: Vec<f64> = (0..10)
            .map(|i| {
                let p = -0.5 * i as f64;
                // Wrap to [-pi, pi)
                p - (p / (2.0 * PI)).round() * 2.0 * PI
            })
            .collect();
        let unwrapped = GroupDelayExtractor::unwrap_phase(&phase);
        // Unwrapped should be monotonic decreasing
        for i in 1..unwrapped.len() {
            assert!(unwrapped[i] <= unwrapped[i - 1] + 0.01);
        }
    }

    #[test]
    fn test_group_delay_linear_phase() {
        // Linear phase: φ(f) = -2π τ f → τ_g = τ (constant)
        let tau = 1e-6; // 1 μs
        let freq_phase: Vec<(f64, f64)> = (0..100)
            .map(|i| {
                let f = 90.0e6 + i as f64 * 0.2e6;
                let phase = -2.0 * PI * tau * f;
                (f, phase)
            })
            .collect();
        let gd = GroupDelayExtractor::compute(&freq_phase);
        assert!(!gd.is_empty());
        // All group delays should be approximately tau
        for &(_, d) in &gd[1..gd.len() - 1] {
            assert!((d - tau).abs() < tau * 0.01);
        }
    }

    #[test]
    fn test_group_delay_from_complex_s21() {
        // Constant delay of 0.5 μs
        let tau = 0.5e-6;
        let data: Vec<(f64, f64, f64)> = (0..50)
            .map(|i| {
                let f = 95.0e6 + i as f64 * 0.2e6;
                let phase = -2.0 * PI * tau * f;
                (f, phase.cos(), phase.sin())
            })
            .collect();
        let gd = GroupDelayExtractor::from_complex_s21(&data);
        let avg = GroupDelayExtractor::average_delay(&gd, 95.0e6, 105.0e6);
        assert!((avg - tau).abs() < tau * 0.05);
    }

    #[test]
    fn test_group_delay_ripple() {
        // Constant delay → zero ripple
        let tau = 1e-6;
        let freq_phase: Vec<(f64, f64)> = (0..50)
            .map(|i| {
                let f = 90.0e6 + i as f64 * 0.4e6;
                (f, -2.0 * PI * tau * f)
            })
            .collect();
        let gd = GroupDelayExtractor::compute(&freq_phase);
        let ripple = GroupDelayExtractor::delay_ripple(&gd, 91.0e6, 109.0e6);
        assert!(ripple < tau * 0.02);
    }

    // --- SensorResponseTracker ---

    #[test]
    fn test_tracker_baseline() {
        let tracker = SensorResponseTracker::new(100.0e6, 0.5);
        assert!((tracker.current_shift()).abs() < TOLERANCE);
        assert!((tracker.current_frequency() - 100.0e6).abs() < TOLERANCE);
    }

    #[test]
    fn test_tracker_shift() {
        let mut tracker = SensorResponseTracker::new(100.0e6, 1.0); // α=1 → no smoothing
        tracker.add_measurement(0.0, 99.999e6);
        assert!((tracker.current_shift() - (-1000.0)).abs() < 1.0);
    }

    #[test]
    fn test_tracker_smoothing() {
        let mut tracker = SensorResponseTracker::new(100.0e6, 0.1);
        // First measurement
        tracker.add_measurement(0.0, 100.001e6);
        // With α=0.1, smoothed = 0.1 * 100.001e6 + 0.9 * 100.0e6
        let expected = 0.1 * 100.001e6 + 0.9 * 100.0e6;
        assert!((tracker.current_frequency() - expected).abs() < 1.0);
    }

    #[test]
    fn test_tracker_drift_rate() {
        let mut tracker = SensorResponseTracker::new(100.0e6, 1.0);
        // Linear drift: 10 Hz/s
        for i in 0..100 {
            let t = i as f64;
            let f = 100.0e6 + 10.0 * t;
            tracker.add_measurement(t, f);
        }
        let drift = tracker.drift_rate();
        assert!((drift - 10.0).abs() < 0.1);
    }

    #[test]
    fn test_tracker_reset_baseline() {
        let mut tracker = SensorResponseTracker::new(100.0e6, 1.0);
        tracker.add_measurement(0.0, 99.999e6);
        tracker.reset_baseline();
        assert!((tracker.current_shift()).abs() < 1.0);
    }

    #[test]
    fn test_tracker_relative_shift() {
        let mut tracker = SensorResponseTracker::new(100.0e6, 1.0);
        tracker.add_measurement(0.0, 99.99e6);
        // Relative shift = (99.99e6 - 100e6) / 100e6 = -1e-4
        assert!((tracker.relative_shift() - (-1e-4)).abs() < 1e-8);
    }

    // --- TemperatureCompensator ---

    #[test]
    fn test_temp_comp_at_reference() {
        let comp = TemperatureCompensator::from_substrate(SubstrateMaterial::LiNbO3Yz);
        let f = comp.compensated_freq(100.0e6, 25.0);
        assert!((f - 100.0e6).abs() < TOLERANCE);
    }

    #[test]
    fn test_temp_comp_linbo3_shift() {
        // LiNbO3 YZ: TCF = -75 ppm/°C
        let comp = TemperatureCompensator::from_substrate(SubstrateMaterial::LiNbO3Yz);
        let f = comp.compensated_freq(100.0e6, 35.0); // +10°C
        // Expected: 100e6 * (1 + (-75e-6)*10) = 100e6 * (1 - 750e-6) ≈ 99.925 MHz
        let expected = 100.0e6 * (1.0 - 750.0e-6);
        assert!((f - expected).abs() / expected < 1e-3);
    }

    #[test]
    fn test_temp_comp_quartz_stable() {
        // ST-quartz: TCF ≈ 0 → frequency almost unchanged
        let comp = TemperatureCompensator::from_substrate(SubstrateMaterial::QuartzSt);
        let f = comp.compensated_freq(100.0e6, 50.0); // +25°C
        assert!((f - 100.0e6).abs() / 100.0e6 < 1e-3);
    }

    #[test]
    fn test_temp_correction_roundtrip() {
        let comp = TemperatureCompensator::from_substrate(SubstrateMaterial::LiNbO3Yz);
        let f0 = 100.0e6;
        let temp = 45.0;
        let f_shifted = comp.compensated_freq(f0, temp);
        let f_corrected = comp.correct_to_reference(f_shifted, temp);
        assert!((f_corrected - f0).abs() < 1.0);
    }

    #[test]
    fn test_turnover_temperature() {
        // Custom substrate with parabolic TCF
        let comp = TemperatureCompensator::new(-10.0, 0.05, 25.0);
        let t_to = comp.turnover_temperature();
        assert!(t_to.is_some());
        // TCF1 = -10e-6, TCF2 = 0.05e-6
        // dT = -TCF1/(2*TCF2) = 10e-6/(2*0.05e-6) = 100
        // T_turnover = 25 + 100 = 125°C
        let expected = 25.0 + 10.0 / (2.0 * 0.05);
        assert!((t_to.unwrap() - expected).abs() < 0.1);
    }

    // --- LoveWaveProcessor ---

    #[test]
    fn test_love_wave_optimal_thickness() {
        // SiO2 (v=3764 m/s) on ST-quartz (v=5000 m/s) at 100 MHz
        let love = LoveWaveProcessor::new(3764.0, 5000.0, 2200.0, 2650.0);
        let h_opt = love.optimal_thickness(100.0e6);
        // λ_L/4 = 3764 / (4 * 100e6) = 9.41 μm
        let expected = 3764.0 / (4.0 * 100.0e6);
        assert!((h_opt - expected).abs() < 1e-12);
    }

    #[test]
    fn test_love_wave_phase_velocity_bounds() {
        // Phase velocity must be between layer and substrate velocities
        let love = LoveWaveProcessor::new(3000.0, 5000.0, 2200.0, 2650.0);
        if let Some(v) = love.phase_velocity(100.0e6, 7.5e-6) {
            assert!(v >= 3000.0 && v <= 5000.0);
        }
    }

    #[test]
    fn test_love_wave_normalized_thickness() {
        let love = LoveWaveProcessor::new(3000.0, 5000.0, 2200.0, 2650.0);
        let hn = love.normalized_thickness(100.0e6, 15e-6);
        // λ = 3000/100e6 = 30 μm, h/λ = 15/30 = 0.5
        assert!((hn - 0.5).abs() < 1e-6);
    }

    // --- RayleighWaveDispersion ---

    #[test]
    fn test_rayleigh_velocity_ratio() {
        // For ν = 0.25 (common rock): v_R/v_S ≈ 0.9194
        let ratio = RayleighWaveDispersion::velocity_ratio(0.25);
        // (0.87 + 1.12*0.25) / 1.25 = (0.87+0.28)/1.25 = 1.15/1.25 = 0.92
        assert!((ratio - 0.92).abs() < 0.01);
    }

    #[test]
    fn test_rayleigh_velocity_absolute() {
        let v_s = 3000.0;
        let v_r = RayleighWaveDispersion::velocity(v_s, 0.25);
        assert!(v_r < v_s); // Rayleigh is always slower than shear
        assert!(v_r > 0.0);
    }

    #[test]
    fn test_rayleigh_zero_poisson() {
        // ν = 0: ratio = 0.87/1.0 = 0.87
        let ratio = RayleighWaveDispersion::velocity_ratio(0.0);
        assert!((ratio - 0.87).abs() < TOLERANCE);
    }

    #[test]
    fn test_shear_from_compressional() {
        // v_S = v_P * sqrt((1-2ν)/(2(1-ν)))
        // For ν=0.25: sqrt(0.5/1.5) = sqrt(1/3) ≈ 0.5774
        let v_s = RayleighWaveDispersion::shear_from_compressional(6000.0, 0.25);
        let expected = 6000.0 * (0.5 / 1.5_f64).sqrt();
        assert!((v_s - expected).abs() < 0.1);
    }

    #[test]
    fn test_rayleigh_from_compressional() {
        let v_r = RayleighWaveDispersion::from_compressional(6000.0, 0.25);
        let v_s = RayleighWaveDispersion::shear_from_compressional(6000.0, 0.25);
        let v_r_direct = RayleighWaveDispersion::velocity(v_s, 0.25);
        assert!((v_r - v_r_direct).abs() < TOLERANCE);
    }

    #[test]
    fn test_rayleigh_dispersion_curve() {
        let curve = RayleighWaveDispersion::dispersion_curve(11);
        assert_eq!(curve.len(), 11);
        // Ratio should increase with Poisson's ratio (monotonic)
        for i in 1..curve.len() {
            assert!(curve[i].1 >= curve[i - 1].1 - TOLERANCE);
        }
    }

    #[test]
    fn test_penetration_depth() {
        let depth = RayleighWaveDispersion::penetration_depth(2760.0, 100.0e6);
        // 2760 / 100e6 = 27.6 μm
        assert!((depth - 27.6e-6).abs() < 0.1e-6);
    }

    // --- Helper functions ---

    #[test]
    fn test_finger_pairs_for_bandwidth() {
        let n = finger_pairs_for_bandwidth(0.02); // 2% BW
        // 0.89/0.02 = 44.5 → 45
        assert_eq!(n, 45);
    }

    #[test]
    fn test_pitch_for_frequency() {
        let p = pitch_for_frequency(100.0e6, SubstrateMaterial::LiNbO3Yz);
        assert!((p - 3488.0 / 200.0e6).abs() < 1e-15);
    }

    #[test]
    fn test_delay_line() {
        // 10 mm path on LiNbO3 YZ → t = 0.01 / 3488 ≈ 2.867 μs
        let t = delay_line_time(0.01, 3488.0);
        assert!((t - 0.01 / 3488.0).abs() < 1e-12);
    }

    #[test]
    fn test_delay_path_roundtrip() {
        let v = 3488.0;
        let delay = 1e-6;
        let path = path_length_for_delay(delay, v);
        let recovered = delay_line_time(path, v);
        assert!((recovered - delay).abs() < 1e-18);
    }
}
