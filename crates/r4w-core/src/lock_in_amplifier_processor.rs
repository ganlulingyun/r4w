//! Lock-In Amplifier (LIA) Signal Processor
//!
//! Implements a software-defined lock-in amplifier for extracting weak signals
//! buried in noise using phase-sensitive detection (PSD). Lock-in amplifiers
//! are widely used in scientific instrumentation to measure signals at a known
//! reference frequency while rejecting out-of-band noise.
//!
//! # Theory of Operation
//!
//! The lock-in amplifier multiplies the input signal V_in(t) by a reference
//! signal at frequency ω_ref, then low-pass filters the result:
//!
//! ```text
//! X = 2 * LPF[ V_in(t) * cos(ω_ref * t + φ) ]
//! Y = 2 * LPF[ V_in(t) * sin(ω_ref * t + φ) ]
//! R = √(X² + Y²)        (magnitude)
//! θ = atan2(Y, X)        (phase)
//! ```
//!
//! For a signal V_in = A*cos(ω_ref*t + δ), this yields:
//! - X = A * cos(δ - φ)
//! - Y = A * sin(δ - φ)
//! - R = A  (independent of phase)
//! - θ = δ - φ
//!
//! The low-pass filter with time constant τ acts as a narrow bandpass filter
//! centered at ω_ref, with noise equivalent bandwidth NEBW = 1/(4τ).
//!
//! # Common Instrument Presets
//!
//! - **SR830**: Stanford Research Systems, DC–102 kHz, classic analog design
//! - **SR844**: Stanford Research Systems, 25 kHz–200 MHz RF lock-in
//! - **HF2LI**: Zurich Instruments, DC–50 MHz, high-frequency digital LIA
//!
//! # References
//!
//! - Zurich Instruments "Principles of Lock-in Detection" (2016)
//! - Stanford Research SR830 Manual, Chapter 1
//! - Meade, M.L. "Lock-in Amplifiers: Principles and Applications" (1983)

use std::f64::consts::{PI, SQRT_2};

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// Number of cascaded RC low-pass filter stages.
///
/// More stages give steeper roll-off and higher dynamic reserve at the cost
/// of slower settling after a step change in signal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilterOrder {
    /// Single RC stage, 6 dB/octave roll-off, NEBW = 1/(4τ)
    First = 1,
    /// Two cascaded RC stages, 12 dB/octave, NEBW = 1/(8τ)
    Second = 2,
    /// Three cascaded RC stages, 18 dB/octave, NEBW = 3/(32τ)
    Third = 3,
    /// Four cascaded RC stages, 24 dB/octave, NEBW = 5/(64τ)
    Fourth = 4,
}

/// Harmonic of the reference frequency to lock to.
///
/// Used for nonlinear response analysis (e.g., 2f in AFM).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Harmonic {
    /// Fundamental — lock to ω_ref
    First = 1,
    /// Second harmonic — lock to 2ω_ref
    Second = 2,
    /// Third harmonic — lock to 3ω_ref
    Third = 3,
    /// Fourth harmonic — lock to 4ω_ref
    Fourth = 4,
}

/// Sensitivity (full-scale) range for the input amplifier.
///
/// Values represent the full-scale input voltage that maps to a normalised
/// output of 1.0.  Auto-ranging adjusts this automatically.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Sensitivity {
    /// 1 nV full-scale
    NanoVolt1 = 0,
    /// 2 nV full-scale
    NanoVolt2,
    /// 5 nV full-scale
    NanoVolt5,
    /// 10 nV full-scale
    NanoVolt10,
    /// 20 nV full-scale
    NanoVolt20,
    /// 50 nV full-scale
    NanoVolt50,
    /// 100 nV full-scale
    NanoVolt100,
    /// 200 nV full-scale
    NanoVolt200,
    /// 500 nV full-scale
    NanoVolt500,
    /// 1 µV full-scale
    MicroVolt1,
    /// 2 µV full-scale
    MicroVolt2,
    /// 5 µV full-scale
    MicroVolt5,
    /// 10 µV full-scale
    MicroVolt10,
    /// 20 µV full-scale
    MicroVolt20,
    /// 50 µV full-scale
    MicroVolt50,
    /// 100 µV full-scale
    MicroVolt100,
    /// 200 µV full-scale
    MicroVolt200,
    /// 500 µV full-scale
    MicroVolt500,
    /// 1 mV full-scale
    MilliVolt1,
    /// 2 mV full-scale
    MilliVolt2,
    /// 5 mV full-scale
    MilliVolt5,
    /// 10 mV full-scale
    MilliVolt10,
    /// 20 mV full-scale
    MilliVolt20,
    /// 50 mV full-scale
    MilliVolt50,
    /// 100 mV full-scale
    MilliVolt100,
    /// 200 mV full-scale
    MilliVolt200,
    /// 500 mV full-scale
    MilliVolt500,
    /// 1 V full-scale
    Volt1,
}

impl Sensitivity {
    /// Return the full-scale voltage in Volts.
    pub fn full_scale_volts(self) -> f64 {
        match self {
            Sensitivity::NanoVolt1   => 1e-9,
            Sensitivity::NanoVolt2   => 2e-9,
            Sensitivity::NanoVolt5   => 5e-9,
            Sensitivity::NanoVolt10  => 10e-9,
            Sensitivity::NanoVolt20  => 20e-9,
            Sensitivity::NanoVolt50  => 50e-9,
            Sensitivity::NanoVolt100 => 100e-9,
            Sensitivity::NanoVolt200 => 200e-9,
            Sensitivity::NanoVolt500 => 500e-9,
            Sensitivity::MicroVolt1  => 1e-6,
            Sensitivity::MicroVolt2  => 2e-6,
            Sensitivity::MicroVolt5  => 5e-6,
            Sensitivity::MicroVolt10 => 10e-6,
            Sensitivity::MicroVolt20 => 20e-6,
            Sensitivity::MicroVolt50 => 50e-6,
            Sensitivity::MicroVolt100=> 100e-6,
            Sensitivity::MicroVolt200=> 200e-6,
            Sensitivity::MicroVolt500=> 500e-6,
            Sensitivity::MilliVolt1  => 1e-3,
            Sensitivity::MilliVolt2  => 2e-3,
            Sensitivity::MilliVolt5  => 5e-3,
            Sensitivity::MilliVolt10 => 10e-3,
            Sensitivity::MilliVolt20 => 20e-3,
            Sensitivity::MilliVolt50 => 50e-3,
            Sensitivity::MilliVolt100=> 100e-3,
            Sensitivity::MilliVolt200=> 200e-3,
            Sensitivity::MilliVolt500=> 500e-3,
            Sensitivity::Volt1       => 1.0,
        }
    }

    /// Return the best sensitivity that still fits the given amplitude.
    /// Selects the smallest full-scale range whose value is >= `amplitude`.
    pub fn best_fit(amplitude: f64) -> Sensitivity {
        use Sensitivity::*;
        let ranges = [
            NanoVolt1, NanoVolt2, NanoVolt5, NanoVolt10, NanoVolt20,
            NanoVolt50, NanoVolt100, NanoVolt200, NanoVolt500,
            MicroVolt1, MicroVolt2, MicroVolt5, MicroVolt10, MicroVolt20,
            MicroVolt50, MicroVolt100, MicroVolt200, MicroVolt500,
            MilliVolt1, MilliVolt2, MilliVolt5, MilliVolt10, MilliVolt20,
            MilliVolt50, MilliVolt100, MilliVolt200, MilliVolt500,
            Volt1,
        ];
        for &r in &ranges {
            if r.full_scale_volts() >= amplitude {
                return r;
            }
        }
        Volt1
    }
}

// ---------------------------------------------------------------------------
// Cascaded RC low-pass filter (IIR)
// ---------------------------------------------------------------------------

/// State for one first-order IIR (RC) low-pass filter stage.
#[derive(Debug, Clone)]
struct RcStage {
    alpha: f64,
    state: f64,
}

impl RcStage {
    /// Create a single RC stage.
    ///
    /// `alpha` = dt / (tau + dt) where tau is the time constant and dt = 1/fs.
    fn new(alpha: f64) -> Self {
        RcStage { alpha, state: 0.0 }
    }

    fn reset(&mut self) {
        self.state = 0.0;
    }

    #[inline]
    fn process(&mut self, x: f64) -> f64 {
        self.state += self.alpha * (x - self.state);
        self.state
    }
}

/// Cascaded RC low-pass filter with 1–4 stages.
#[derive(Debug, Clone)]
struct CascadedRcFilter {
    stages: Vec<RcStage>,
}

impl CascadedRcFilter {
    /// Create a cascaded filter.
    ///
    /// * `tau_s` – time constant in seconds
    /// * `fs`    – sample rate in Hz
    /// * `order` – number of cascaded RC stages
    fn new(tau_s: f64, fs: f64, order: FilterOrder) -> Self {
        let dt = 1.0 / fs;
        let alpha = dt / (tau_s + dt);
        let n = order as usize;
        CascadedRcFilter {
            stages: (0..n).map(|_| RcStage::new(alpha)).collect(),
        }
    }

    fn reset(&mut self) {
        for s in &mut self.stages {
            s.reset();
        }
    }

    #[inline]
    fn process(&mut self, mut x: f64) -> f64 {
        for stage in &mut self.stages {
            x = stage.process(x);
        }
        x
    }
}

// ---------------------------------------------------------------------------
// AC coupling (highpass) input pre-filter
// ---------------------------------------------------------------------------

/// Single-pole highpass IIR — AC coupling to remove DC offset from the input.
#[derive(Debug, Clone)]
struct AcCoupling {
    alpha: f64, // alpha_hp = tau / (tau + dt)
    state: f64,
    prev_in: f64,
}

impl AcCoupling {
    /// Create an AC coupling stage with a cutoff at `fc_hz`.
    fn new(fc_hz: f64, fs: f64) -> Self {
        let tau = 1.0 / (2.0 * PI * fc_hz);
        let dt  = 1.0 / fs;
        let alpha = tau / (tau + dt);
        AcCoupling { alpha, state: 0.0, prev_in: 0.0 }
    }

    fn reset(&mut self) {
        self.state  = 0.0;
        self.prev_in = 0.0;
    }

    #[inline]
    fn process(&mut self, x: f64) -> f64 {
        let y = self.alpha * (self.state + x - self.prev_in);
        self.prev_in = x;
        self.state   = y;
        y
    }
}

// ---------------------------------------------------------------------------
// Narrow bandpass pre-filter around f_ref
// ---------------------------------------------------------------------------

/// Second-order bandpass IIR pre-filter (biquad) centred at `f0_hz`.
///
/// Transfer function:
/// ```text
///   H(z) = b0 + b1*z⁻¹ + b2*z⁻²
///           -------------------------
///           1  + a1*z⁻¹ + a2*z⁻²
/// ```
/// using the bilinear transform of an analog 2nd-order BPF with Q=`q`.
#[derive(Debug, Clone)]
struct BandpassPreFilter {
    b0: f64,
    b1: f64,
    b2: f64,
    a1: f64,
    a2: f64,
    x1: f64,
    x2: f64,
    y1: f64,
    y2: f64,
    enabled: bool,
}

impl BandpassPreFilter {
    /// Create a bandpass pre-filter.
    ///
    /// * `f0_hz` – centre frequency in Hz
    /// * `q`     – quality factor (bandwidth = f0/Q)
    /// * `fs`    – sample rate in Hz
    ///
    /// If `q <= 0.0` the filter is disabled (pass-through).
    fn new(f0_hz: f64, q: f64, fs: f64) -> Self {
        if q <= 0.0 || f0_hz <= 0.0 {
            return BandpassPreFilter {
                b0: 1.0, b1: 0.0, b2: 0.0,
                a1: 0.0, a2: 0.0,
                x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0,
                enabled: false,
            };
        }
        // Bilinear transform bandpass design
        let w0 = 2.0 * PI * f0_hz / fs;
        let alpha = w0.sin() / (2.0 * q);
        let cos_w0 = w0.cos();

        let b0 =  alpha;
        let b1 =  0.0;
        let b2 = -alpha;
        let a0 =  1.0 + alpha;
        let a1 = -2.0 * cos_w0;
        let a2 =  1.0 - alpha;

        BandpassPreFilter {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1: a1 / a0,
            a2: a2 / a0,
            x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0,
            enabled: true,
        }
    }

    fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
    }

    #[inline]
    fn process(&mut self, x: f64) -> f64 {
        if !self.enabled {
            return x;
        }
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
              - self.a1 * self.y1 - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }
}

// ---------------------------------------------------------------------------
// LIA output snapshot
// ---------------------------------------------------------------------------

/// A single output record from the lock-in amplifier.
#[derive(Debug, Clone)]
pub struct LiaOutput {
    /// In-phase component X (V)
    pub x: f64,
    /// Quadrature component Y (V)
    pub y: f64,
    /// Amplitude R = √(X²+Y²) (V)
    pub r: f64,
    /// Phase θ = atan2(Y, X) (radians, -π..π)
    pub theta: f64,
    /// Normalised amplitude in full-scale units (0..1)
    pub r_normalised: f64,
    /// Current sensitivity full-scale value (V)
    pub sensitivity_v: f64,
}

// ---------------------------------------------------------------------------
// Preset configurations
// ---------------------------------------------------------------------------

/// Preset LIA configurations modelling commercial instruments.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LiaPreset {
    /// Stanford Research SR830 (DC – 102 kHz).
    /// Single-slope roll-off up to 4th order, typical tau 100 µs – 30 ks.
    Sr830,
    /// Stanford Research SR844 RF lock-in (25 kHz – 200 MHz).
    /// Designed for RF signals; wider pre-filter, lower time constants.
    Sr844,
    /// Zurich Instruments HF2LI (DC – 50 MHz) digital LIA.
    /// Very wide frequency range, programmable filter order.
    Hf2Li,
}

/// Configuration parameters for the lock-in amplifier.
#[derive(Debug, Clone)]
pub struct LiaConfig {
    /// Reference frequency in Hz.
    pub reference_freq_hz: f64,
    /// Phase offset applied to the internal reference oscillator (radians).
    pub reference_phase_rad: f64,
    /// Low-pass filter time constant τ (seconds).
    pub time_constant_s: f64,
    /// Number of cascaded RC stages (filter order / roll-off).
    pub filter_order: FilterOrder,
    /// Which harmonic of the reference to detect.
    pub harmonic: Harmonic,
    /// Sensitivity (full-scale) setting.
    pub sensitivity: Sensitivity,
    /// Enable auto-ranging to adjust sensitivity automatically.
    pub auto_range: bool,
    /// Enable AC coupling (highpass) on the input.
    /// Set to `Some(fc_hz)` to specify cut-off frequency.
    pub ac_coupling_hz: Option<f64>,
    /// Enable bandpass pre-filter around f_ref with given Q factor.
    /// Set to `None` to disable.
    pub prefilter_q: Option<f64>,
    /// Sample rate in Hz.
    pub sample_rate_hz: f64,
}

impl LiaConfig {
    /// Create a default configuration with a given reference frequency and
    /// sample rate.
    pub fn new(reference_freq_hz: f64, sample_rate_hz: f64) -> Self {
        LiaConfig {
            reference_freq_hz,
            reference_phase_rad: 0.0,
            time_constant_s: 1e-3,
            filter_order: FilterOrder::First,
            harmonic: Harmonic::First,
            sensitivity: Sensitivity::MilliVolt1,
            auto_range: false,
            ac_coupling_hz: None,
            prefilter_q: None,
            sample_rate_hz,
        }
    }

    /// Apply a preset that mirrors a commercial instrument's defaults.
    pub fn with_preset(mut self, preset: LiaPreset) -> Self {
        match preset {
            LiaPreset::Sr830 => {
                self.filter_order   = FilterOrder::Second;
                self.time_constant_s = 300e-6;
                self.ac_coupling_hz = Some(1.0);
                self.prefilter_q    = None;
            }
            LiaPreset::Sr844 => {
                self.filter_order   = FilterOrder::First;
                self.time_constant_s = 10e-6;
                self.ac_coupling_hz = None;
                self.prefilter_q    = Some(5.0);
            }
            LiaPreset::Hf2Li => {
                self.filter_order   = FilterOrder::Fourth;
                self.time_constant_s = 1e-6;
                self.ac_coupling_hz = None;
                self.prefilter_q    = Some(10.0);
            }
        }
        self
    }
}

// ---------------------------------------------------------------------------
// Main LIA processor
// ---------------------------------------------------------------------------

/// Software lock-in amplifier for phase-sensitive detection.
///
/// # Example
///
/// ```rust
/// use r4w_core::lock_in_amplifier_processor::{LiaConfig, LockInAmplifier};
///
/// let fs = 100_000.0;   // 100 kHz sample rate
/// let f_ref = 1_000.0;  // 1 kHz reference
/// let config = LiaConfig::new(f_ref, fs);
/// let mut lia = LockInAmplifier::new(config);
///
/// // Feed samples; here a pure 1 kHz sine at 10 mV
/// let mut outputs = Vec::new();
/// for n in 0..1000 {
///     let t = n as f64 / fs;
///     let sample = 0.01 * (2.0 * std::f64::consts::PI * f_ref * t).sin();
///     outputs.push(lia.process(sample));
/// }
///
/// // After settling, R should be near 10 mV
/// let last = outputs.last().unwrap();
/// assert!((last.r - 0.01).abs() < 0.005);
/// ```
#[derive(Debug, Clone)]
pub struct LockInAmplifier {
    config: LiaConfig,

    // Reference oscillator phase accumulator (radians)
    phase_acc: f64,
    // Angular step per sample for harmonic * f_ref
    phase_step: f64,

    // Low-pass filter chains for X and Y
    lpf_x: CascadedRcFilter,
    lpf_y: CascadedRcFilter,

    // Optional input AC coupling
    ac_coupler: Option<AcCoupling>,

    // Optional bandpass pre-filter
    prefilter: BandpassPreFilter,

    // Sample counter (for auto-range settling)
    sample_count: u64,
}

impl LockInAmplifier {
    /// Create a new lock-in amplifier from a [`LiaConfig`].
    pub fn new(config: LiaConfig) -> Self {
        let fs = config.sample_rate_hz;
        let f_detect = config.reference_freq_hz * (config.harmonic as i32 as f64);
        let phase_step = 2.0 * PI * f_detect / fs;

        let lpf_x = CascadedRcFilter::new(config.time_constant_s, fs, config.filter_order);
        let lpf_y = CascadedRcFilter::new(config.time_constant_s, fs, config.filter_order);

        let ac_coupler = config.ac_coupling_hz.map(|fc| AcCoupling::new(fc, fs));

        let prefilter = match config.prefilter_q {
            Some(q) => BandpassPreFilter::new(config.reference_freq_hz, q, fs),
            None    => BandpassPreFilter::new(0.0, -1.0, fs), // disabled
        };

        LockInAmplifier {
            config,
            phase_acc: 0.0,
            phase_step,
            lpf_x,
            lpf_y,
            ac_coupler,
            prefilter,
            sample_count: 0,
        }
    }

    /// Process a single input sample and return the LIA output.
    ///
    /// Internally:
    /// 1. AC-couple and bandpass-filter the input.
    /// 2. Multiply by reference cosine (X) and reference sine (Y).
    /// 3. Low-pass filter X and Y.
    /// 4. Calculate R and θ.
    pub fn process(&mut self, input: f64) -> LiaOutput {
        // --- Input conditioning ---
        let v = if let Some(ref mut ac) = self.ac_coupler {
            ac.process(input)
        } else {
            input
        };
        let v = self.prefilter.process(v);

        // --- Reference oscillator ---
        let phi = self.phase_acc + self.config.reference_phase_rad;
        let ref_cos = phi.cos();
        let ref_sin = phi.sin();

        // Advance phase accumulator, wrap to avoid floating-point drift
        self.phase_acc += self.phase_step;
        if self.phase_acc >= 2.0 * PI {
            self.phase_acc -= 2.0 * PI;
        }

        // --- Phase-sensitive detection (factor of 2 for amplitude normalisation) ---
        let mixed_x = 2.0 * v * ref_cos;
        let mixed_y = 2.0 * v * ref_sin;

        // --- Low-pass filtering ---
        let x = self.lpf_x.process(mixed_x);
        let y = self.lpf_y.process(mixed_y);

        // --- Output calculation ---
        let r = (x * x + y * y).sqrt();
        let theta = y.atan2(x);

        // --- Auto-ranging ---
        if self.config.auto_range && self.sample_count % 1024 == 0 {
            self.config.sensitivity = Sensitivity::best_fit(r);
        }
        self.sample_count += 1;

        let sensitivity_v = self.config.sensitivity.full_scale_volts();
        let r_normalised = if sensitivity_v > 0.0 { r / sensitivity_v } else { 0.0 };

        LiaOutput { x, y, r, theta, r_normalised, sensitivity_v }
    }

    /// Process a buffer of samples, returning one [`LiaOutput`] per sample.
    pub fn process_block(&mut self, input: &[f64]) -> Vec<LiaOutput> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    /// Reset all internal state (filter state, phase accumulator).
    pub fn reset(&mut self) {
        self.phase_acc  = 0.0;
        self.lpf_x.reset();
        self.lpf_y.reset();
        if let Some(ref mut ac) = self.ac_coupler {
            ac.reset();
        }
        self.prefilter.reset();
        self.sample_count = 0;
    }

    /// Update the reference frequency at runtime (re-initialises the phase step).
    ///
    /// The phase accumulator is reset so phase continuity is not guaranteed.
    pub fn set_reference_freq(&mut self, freq_hz: f64) {
        self.config.reference_freq_hz = freq_hz;
        let f_detect = freq_hz * (self.config.harmonic as i32 as f64);
        self.phase_step = 2.0 * PI * f_detect / self.config.sample_rate_hz;
        self.phase_acc  = 0.0;
    }

    /// Update the reference phase offset (radians) without resetting state.
    pub fn set_reference_phase(&mut self, phase_rad: f64) {
        self.config.reference_phase_rad = phase_rad;
    }

    /// Update the filter time constant and rebuild the LPF.
    pub fn set_time_constant(&mut self, tau_s: f64) {
        self.config.time_constant_s = tau_s;
        let fs = self.config.sample_rate_hz;
        let order = self.config.filter_order;
        self.lpf_x = CascadedRcFilter::new(tau_s, fs, order);
        self.lpf_y = CascadedRcFilter::new(tau_s, fs, order);
    }

    /// Return the noise equivalent bandwidth (Hz) for the current settings.
    ///
    /// ```text
    /// Order 1: NEBW = 1 / (4τ)
    /// Order 2: NEBW = 1 / (8τ)
    /// Order 3: NEBW = 3 / (32τ)  ≈ 0.09375 / τ
    /// Order 4: NEBW = 5 / (64τ)  ≈ 0.07813 / τ
    /// ```
    pub fn noise_equivalent_bandwidth_hz(&self) -> f64 {
        let tau = self.config.time_constant_s;
        match self.config.filter_order {
            FilterOrder::First  => 1.0 / (4.0 * tau),
            FilterOrder::Second => 1.0 / (8.0 * tau),
            FilterOrder::Third  => 3.0 / (32.0 * tau),
            FilterOrder::Fourth => 5.0 / (64.0 * tau),
        }
    }

    /// Dynamic reserve in dB.
    ///
    /// Defined as `20 * log10(V_interferer_max / V_signal_fullscale)`.
    /// Estimated conservatively as `20 * n * 10 * log10(fs / (2 * NEBW))` dB
    /// where n is the filter order, representing how many decades of
    /// attenuation the LPF provides beyond the noise bandwidth.
    ///
    /// A practical rule: each additional filter order adds ~6–12 dB of
    /// dynamic reserve.
    pub fn dynamic_reserve_db(&self) -> f64 {
        let nebw = self.noise_equivalent_bandwidth_hz();
        let fs   = self.config.sample_rate_hz;
        let n    = self.config.filter_order as i32 as f64;
        // Attenuation factor at Nyquist relative to the noise BW
        let ratio = fs / (2.0 * nebw);
        if ratio <= 1.0 {
            return 0.0;
        }
        n * 10.0 * ratio.log10()
    }

    /// Estimate SNR improvement factor (linear).
    ///
    /// `SNR_out / SNR_in = sqrt(fs / (2 * NEBW))`
    pub fn snr_improvement_factor(&self) -> f64 {
        let nebw = self.noise_equivalent_bandwidth_hz();
        let fs   = self.config.sample_rate_hz;
        (fs / (2.0 * nebw)).sqrt()
    }

    /// SNR improvement in dB.
    pub fn snr_improvement_db(&self) -> f64 {
        10.0 * self.snr_improvement_factor().log10()
    }

    /// Return the current configuration (read-only reference).
    pub fn config(&self) -> &LiaConfig {
        &self.config
    }

    /// Return the current phase accumulator value (radians).
    pub fn phase_accumulator(&self) -> f64 {
        self.phase_acc
    }
}

// ---------------------------------------------------------------------------
// Convenience builder
// ---------------------------------------------------------------------------

/// Builder for constructing a [`LockInAmplifier`] step by step.
pub struct LiaBuilder {
    config: LiaConfig,
}

impl LiaBuilder {
    /// Start with the mandatory reference frequency and sample rate.
    pub fn new(reference_freq_hz: f64, sample_rate_hz: f64) -> Self {
        LiaBuilder {
            config: LiaConfig::new(reference_freq_hz, sample_rate_hz),
        }
    }

    /// Set the filter time constant (seconds).
    pub fn time_constant(mut self, tau_s: f64) -> Self {
        self.config.time_constant_s = tau_s;
        self
    }

    /// Set the filter order (number of cascaded RC stages).
    pub fn filter_order(mut self, order: FilterOrder) -> Self {
        self.config.filter_order = order;
        self
    }

    /// Select the harmonic of the reference.
    pub fn harmonic(mut self, h: Harmonic) -> Self {
        self.config.harmonic = h;
        self
    }

    /// Set the reference phase offset (radians).
    pub fn phase(mut self, phi_rad: f64) -> Self {
        self.config.reference_phase_rad = phi_rad;
        self
    }

    /// Set the sensitivity range.
    pub fn sensitivity(mut self, s: Sensitivity) -> Self {
        self.config.sensitivity = s;
        self
    }

    /// Enable auto-ranging.
    pub fn auto_range(mut self) -> Self {
        self.config.auto_range = true;
        self
    }

    /// Enable AC coupling with a highpass cutoff at `fc_hz`.
    pub fn ac_coupling(mut self, fc_hz: f64) -> Self {
        self.config.ac_coupling_hz = Some(fc_hz);
        self
    }

    /// Enable bandpass pre-filter with quality factor `q`.
    pub fn prefilter_q(mut self, q: f64) -> Self {
        self.config.prefilter_q = Some(q);
        self
    }

    /// Apply a preset (SR830, SR844, HF2LI).
    pub fn preset(mut self, p: LiaPreset) -> Self {
        self.config = self.config.with_preset(p);
        self
    }

    /// Finalise and build the [`LockInAmplifier`].
    pub fn build(self) -> LockInAmplifier {
        LockInAmplifier::new(self.config)
    }
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/// Compute the noise equivalent bandwidth (Hz) for a cascaded RC filter.
///
/// * `tau_s`  – time constant in seconds
/// * `order`  – filter order
pub fn nebw_hz(tau_s: f64, order: FilterOrder) -> f64 {
    match order {
        FilterOrder::First  => 1.0 / (4.0 * tau_s),
        FilterOrder::Second => 1.0 / (8.0 * tau_s),
        FilterOrder::Third  => 3.0 / (32.0 * tau_s),
        FilterOrder::Fourth => 5.0 / (64.0 * tau_s),
    }
}

/// Minimum integration time needed to achieve a target SNR improvement (dB)
/// given the sample rate and filter order.
///
/// Derived from `SNR_improvement_dB = 10 * log10(fs / (2 * NEBW))`.
pub fn min_time_constant_for_snr(
    target_snr_improvement_db: f64,
    fs: f64,
    order: FilterOrder,
) -> f64 {
    // SNR_improvement = fs / (2 * NEBW)
    // 10^(dB/10) = fs / (2 * NEBW)
    // NEBW = fs / (2 * 10^(dB/10))
    let snr_linear = 10_f64.powf(target_snr_improvement_db / 10.0);
    let target_nebw = fs / (2.0 * snr_linear);
    // Invert NEBW formula for tau
    match order {
        FilterOrder::First  => 1.0 / (4.0 * target_nebw),
        FilterOrder::Second => 1.0 / (8.0 * target_nebw),
        FilterOrder::Third  => 3.0 / (32.0 * target_nebw),
        FilterOrder::Fourth => 5.0 / (64.0 * target_nebw),
    }
}

/// Generate one period of ideal reference sine and cosine waveforms.
///
/// Returns `(cos_vec, sin_vec)` each of length `n_samples`.
pub fn reference_waveform(f_hz: f64, fs: f64, n_samples: usize) -> (Vec<f64>, Vec<f64>) {
    let step = 2.0 * PI * f_hz / fs;
    let cos_v: Vec<f64> = (0..n_samples).map(|i| (step * i as f64).cos()).collect();
    let sin_v: Vec<f64> = (0..n_samples).map(|i| (step * i as f64).sin()).collect();
    (cos_v, sin_v)
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const FS: f64 = 100_000.0; // 100 kHz sample rate
    const F_REF: f64 = 1_000.0; // 1 kHz reference

    /// Generate a pure sine at `f_hz` with amplitude `a`.
    fn sine_signal(f_hz: f64, amp: f64, n: usize, fs: f64) -> Vec<f64> {
        (0..n)
            .map(|i| amp * (2.0 * PI * f_hz * i as f64 / fs).sin())
            .collect()
    }

    /// Generate a pure cosine at `f_hz` with amplitude `a`.
    fn cosine_signal(f_hz: f64, amp: f64, n: usize, fs: f64) -> Vec<f64> {
        (0..n)
            .map(|i| amp * (2.0 * PI * f_hz * i as f64 / fs).cos())
            .collect()
    }

    // ------------------------------------------------------------------
    // 1. Basic construction
    // ------------------------------------------------------------------
    #[test]
    fn test_basic_construction() {
        let config = LiaConfig::new(F_REF, FS);
        let lia = LockInAmplifier::new(config);
        assert!((lia.config().reference_freq_hz - F_REF).abs() < 1e-9);
        assert!((lia.config().sample_rate_hz - FS).abs() < 1e-9);
    }

    // ------------------------------------------------------------------
    // 2. Amplitude recovery from in-phase sine
    // ------------------------------------------------------------------
    #[test]
    fn test_amplitude_recovery_sine() {
        let amp = 0.01; // 10 mV
        let n = 10_000; // 100 ms
        let signal = sine_signal(F_REF, amp, n, FS);

        // Reference is cosine ⇒ a sine input produces Y component
        let config = LiaConfig::new(F_REF, FS);
        let mut lia = LockInAmplifier::new(config);
        let outputs = lia.process_block(&signal);
        let last = outputs.last().unwrap();
        // R should converge to amp (within 5 %)
        assert!(
            (last.r - amp).abs() < 0.05 * amp,
            "R={} expected~{}", last.r, amp
        );
    }

    // ------------------------------------------------------------------
    // 3. Amplitude recovery from in-phase cosine
    // ------------------------------------------------------------------
    #[test]
    fn test_amplitude_recovery_cosine() {
        let amp = 0.005; // 5 mV
        let n = 10_000;
        let signal = cosine_signal(F_REF, amp, n, FS);

        let config = LiaConfig::new(F_REF, FS);
        let mut lia = LockInAmplifier::new(config);
        let outputs = lia.process_block(&signal);
        let last = outputs.last().unwrap();
        assert!(
            (last.r - amp).abs() < 0.05 * amp,
            "R={} expected~{}", last.r, amp
        );
    }

    // ------------------------------------------------------------------
    // 4. Zero output for DC input
    // ------------------------------------------------------------------
    #[test]
    fn test_dc_input_rejection() {
        // DC input produces 2*DC*cos(wt) at the PSD multiplier output.
        // The LPF must attenuate this term strongly. With tau=10ms the
        // RC pole is at ~16 Hz, attenuating 1 kHz by factor ~62, so
        // the residual ripple at steady-state is small.
        let dc: Vec<f64> = vec![1.0; 50_000]; // 0.5 s = 50 tau
        let mut config = LiaConfig::new(F_REF, FS);
        config.time_constant_s = 10e-3; // 10 ms tau → pole at ~16 Hz
        let mut lia = LockInAmplifier::new(config);
        let outputs = lia.process_block(&dc);
        let last = outputs.last().unwrap();
        // At 1 kHz with 16 Hz pole, attenuation ≈ 62× → residual ~0.032 for 2*1 input
        assert!(last.r < 0.05, "R={} should be small for DC with long tau", last.r);
    }

    // ------------------------------------------------------------------
    // 5. Off-frequency signal rejected
    // ------------------------------------------------------------------
    #[test]
    fn test_off_frequency_rejection() {
        // Signal at 1.5 kHz beats with 1 kHz reference to produce 500 Hz component.
        // With tau=10 ms the LPF pole is at ~16 Hz, strongly attenuating 500 Hz.
        let f_noise = F_REF * 1.5; // 1.5 kHz
        let signal = sine_signal(f_noise, 1.0, 50_000, FS); // 0.5 s settling
        let mut config = LiaConfig::new(F_REF, FS);
        config.time_constant_s = 10e-3;
        let mut lia = LockInAmplifier::new(config);
        let outputs = lia.process_block(&signal);
        let last = outputs.last().unwrap();
        // 500 Hz attenuated by ~32× at 16 Hz pole → residual ~0.03 for unit amplitude
        assert!(last.r < 0.05, "R={} – off-freq signal should be rejected", last.r);
    }

    // ------------------------------------------------------------------
    // 6. Phase measurement
    // ------------------------------------------------------------------
    #[test]
    fn test_phase_measurement() {
        // For V_in = A*sin(ωt + φ) and reference cos(ωt):
        //   X = 2*LPF[A*sin(ωt+φ)*cos(ωt)] = A*sin(φ)
        //   Y = 2*LPF[A*sin(ωt+φ)*sin(ωt)] = A*cos(φ)
        //   θ = atan2(Y,X) = atan2(cos φ, sin φ) = π/2 - φ
        let phi_deg = 45.0_f64;
        let phi_rad = phi_deg.to_radians();
        let n = 20_000;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / FS;
                0.01 * (2.0 * PI * F_REF * t + phi_rad).sin()
            })
            .collect();

        let config = LiaConfig::new(F_REF, FS);
        let mut lia = LockInAmplifier::new(config);
        let outputs = lia.process_block(&signal);
        let last = outputs.last().unwrap();
        // Expected: θ = π/2 - φ = π/2 - π/4 = π/4 ≈ 0.785
        let expected_theta = PI / 2.0 - phi_rad;
        let diff = (last.theta - expected_theta).abs();
        let diff = if diff > PI { 2.0 * PI - diff } else { diff };
        assert!(diff < 0.1, "theta={:.3} expected~{:.3}", last.theta, expected_theta);
    }

    // ------------------------------------------------------------------
    // 7. NEBW — first order
    // ------------------------------------------------------------------
    #[test]
    fn test_nebw_first_order() {
        let tau = 1e-3; // 1 ms
        let nebw = nebw_hz(tau, FilterOrder::First);
        // Expected: 1/(4*0.001) = 250 Hz
        assert!((nebw - 250.0).abs() < 1.0, "NEBW={} expected 250", nebw);
    }

    // ------------------------------------------------------------------
    // 8. NEBW — second order
    // ------------------------------------------------------------------
    #[test]
    fn test_nebw_second_order() {
        let tau = 1e-3;
        let nebw = nebw_hz(tau, FilterOrder::Second);
        // Expected: 1/(8*0.001) = 125 Hz
        assert!((nebw - 125.0).abs() < 1.0, "NEBW={} expected 125", nebw);
    }

    // ------------------------------------------------------------------
    // 9. NEBW — third order
    // ------------------------------------------------------------------
    #[test]
    fn test_nebw_third_order() {
        let tau = 1e-3;
        let nebw = nebw_hz(tau, FilterOrder::Third);
        let expected = 3.0 / (32.0 * tau);
        assert!((nebw - expected).abs() < 0.1);
    }

    // ------------------------------------------------------------------
    // 10. NEBW — fourth order
    // ------------------------------------------------------------------
    #[test]
    fn test_nebw_fourth_order() {
        let tau = 1e-3;
        let nebw = nebw_hz(tau, FilterOrder::Fourth);
        let expected = 5.0 / (64.0 * tau);
        assert!((nebw - expected).abs() < 0.1);
    }

    // ------------------------------------------------------------------
    // 11. NEBW decreases with filter order
    // ------------------------------------------------------------------
    #[test]
    fn test_nebw_decreases_with_order() {
        let tau = 1e-3;
        let n1 = nebw_hz(tau, FilterOrder::First);
        let n2 = nebw_hz(tau, FilterOrder::Second);
        let n3 = nebw_hz(tau, FilterOrder::Third);
        let n4 = nebw_hz(tau, FilterOrder::Fourth);
        assert!(n1 > n2, "1st order NEBW should exceed 2nd");
        assert!(n2 > n3, "2nd order NEBW should exceed 3rd");
        assert!(n3 > n4, "3rd order NEBW should exceed 4th");
    }

    // ------------------------------------------------------------------
    // 12. SNR improvement is positive
    // ------------------------------------------------------------------
    #[test]
    fn test_snr_improvement_positive() {
        let config = LiaConfig::new(F_REF, FS);
        let lia = LockInAmplifier::new(config);
        let snr_db = lia.snr_improvement_db();
        assert!(snr_db > 0.0, "SNR improvement should be positive");
    }

    // ------------------------------------------------------------------
    // 13. SNR improvement increases with time constant
    // ------------------------------------------------------------------
    #[test]
    fn test_snr_improvement_vs_tau() {
        let mut config1 = LiaConfig::new(F_REF, FS);
        config1.time_constant_s = 1e-3;
        let lia1 = LockInAmplifier::new(config1);

        let mut config2 = LiaConfig::new(F_REF, FS);
        config2.time_constant_s = 10e-3;
        let lia2 = LockInAmplifier::new(config2);

        assert!(
            lia2.snr_improvement_db() > lia1.snr_improvement_db(),
            "Larger tau should give more SNR improvement"
        );
    }

    // ------------------------------------------------------------------
    // 14. Dynamic reserve is positive
    // ------------------------------------------------------------------
    #[test]
    fn test_dynamic_reserve_positive() {
        let config = LiaConfig::new(F_REF, FS);
        let lia = LockInAmplifier::new(config);
        assert!(lia.dynamic_reserve_db() > 0.0);
    }

    // ------------------------------------------------------------------
    // 15. Dynamic reserve increases with filter order
    // ------------------------------------------------------------------
    #[test]
    fn test_dynamic_reserve_vs_order() {
        let make = |order| {
            let mut cfg = LiaConfig::new(F_REF, FS);
            cfg.filter_order = order;
            cfg.time_constant_s = 1e-3;
            LockInAmplifier::new(cfg).dynamic_reserve_db()
        };
        let dr1 = make(FilterOrder::First);
        let dr4 = make(FilterOrder::Fourth);
        assert!(dr4 > dr1, "4th-order should have higher dynamic reserve");
    }

    // ------------------------------------------------------------------
    // 16. Reset clears state
    // ------------------------------------------------------------------
    #[test]
    fn test_reset() {
        let amp = 0.1;
        let signal = sine_signal(F_REF, amp, 5000, FS);
        let config = LiaConfig::new(F_REF, FS);
        let mut lia = LockInAmplifier::new(config);
        lia.process_block(&signal);
        lia.reset();
        // After reset, processing a single zero should give (near) zero output
        let out = lia.process(0.0);
        assert!(out.r < 1e-10, "After reset R should be ~0, got {}", out.r);
    }

    // ------------------------------------------------------------------
    // 17. Sensitivity::best_fit selects correct range
    // ------------------------------------------------------------------
    #[test]
    fn test_sensitivity_best_fit_nanoscale() {
        let s = Sensitivity::best_fit(3e-9); // 3 nV
        // Should select NanoVolt5 (5e-9 >= 3e-9)
        assert!((s.full_scale_volts() - 5e-9).abs() < 1e-12);
    }

    #[test]
    fn test_sensitivity_best_fit_millivolts() {
        let s = Sensitivity::best_fit(15e-3); // 15 mV
        // Should select MilliVolt20 (20e-3 >= 15e-3)
        assert!((s.full_scale_volts() - 20e-3).abs() < 1e-9);
    }

    #[test]
    fn test_sensitivity_best_fit_large() {
        let s = Sensitivity::best_fit(2.0); // 2 V → clamps to Volt1
        assert!((s.full_scale_volts() - 1.0).abs() < 1e-9);
    }

    // ------------------------------------------------------------------
    // 18. Normalised output within 0..2 for on-frequency signal
    // ------------------------------------------------------------------
    #[test]
    fn test_normalised_output_range() {
        let amp = 0.5e-3; // 500 µV
        let signal = sine_signal(F_REF, amp, 10_000, FS);

        let mut config = LiaConfig::new(F_REF, FS);
        config.sensitivity = Sensitivity::MilliVolt1; // 1 mV full scale
        let mut lia = LockInAmplifier::new(config);
        let outputs = lia.process_block(&signal);
        let last = outputs.last().unwrap();
        // R_norm = 0.5 mV / 1 mV = ~0.5
        assert!(last.r_normalised < 1.5, "normalised output={}", last.r_normalised);
        assert!(last.r_normalised > 0.3);
    }

    // ------------------------------------------------------------------
    // 19. Second harmonic detection
    // ------------------------------------------------------------------
    #[test]
    fn test_second_harmonic_detection() {
        let amp = 0.01;
        let n = 20_000;
        // Signal at 2*F_REF
        let signal = sine_signal(2.0 * F_REF, amp, n, FS);

        let mut config = LiaConfig::new(F_REF, FS);
        config.harmonic = Harmonic::Second;
        config.time_constant_s = 1e-3;
        let mut lia = LockInAmplifier::new(config);
        let outputs = lia.process_block(&signal);
        let last = outputs.last().unwrap();
        assert!(
            (last.r - amp).abs() < 0.05 * amp,
            "2f R={} expected~{}", last.r, amp
        );
    }

    // ------------------------------------------------------------------
    // 20. Third harmonic detection
    // ------------------------------------------------------------------
    #[test]
    fn test_third_harmonic_detection() {
        let amp = 0.01;
        let n = 20_000;
        let signal = sine_signal(3.0 * F_REF, amp, n, FS);

        let mut config = LiaConfig::new(F_REF, FS);
        config.harmonic = Harmonic::Third;
        config.time_constant_s = 1e-3;
        let mut lia = LockInAmplifier::new(config);
        let outputs = lia.process_block(&signal);
        let last = outputs.last().unwrap();
        assert!(
            (last.r - amp).abs() < 0.05 * amp,
            "3f R={} expected~{}", last.r, amp
        );
    }

    // ------------------------------------------------------------------
    // 21. 2f harmonic rejects fundamental
    // ------------------------------------------------------------------
    #[test]
    fn test_second_harmonic_rejects_fundamental() {
        // A 1f (1 kHz) signal beating against 2f (2 kHz) reference produces
        // components at 1 kHz and 3 kHz. With tau=10 ms (pole at 16 Hz),
        // the 1 kHz beat is attenuated by ~62×, so residual < 0.033 for unit amp.
        let n = 100_000; // 1 s of signal for good settling
        let signal = sine_signal(F_REF, 1.0, n, FS);

        let mut config = LiaConfig::new(F_REF, FS);
        config.harmonic = Harmonic::Second;
        config.time_constant_s = 10e-3;
        let mut lia = LockInAmplifier::new(config);
        let outputs = lia.process_block(&signal);
        // Average last 1000 samples to reduce instantaneous ripple
        let n_avg = 1000;
        let r_avg = outputs[outputs.len() - n_avg..]
            .iter()
            .map(|o| o.r)
            .sum::<f64>()
            / n_avg as f64;
        assert!(r_avg < 0.05, "2f LIA should reject 1f signal, R_avg={}", r_avg);
    }

    // ------------------------------------------------------------------
    // 22. Builder API
    // ------------------------------------------------------------------
    #[test]
    fn test_builder() {
        let lia = LiaBuilder::new(F_REF, FS)
            .time_constant(5e-3)
            .filter_order(FilterOrder::Second)
            .harmonic(Harmonic::First)
            .sensitivity(Sensitivity::MilliVolt10)
            .build();

        assert!((lia.config().time_constant_s - 5e-3).abs() < 1e-10);
        assert_eq!(lia.config().filter_order, FilterOrder::Second);
        assert!((lia.config().sensitivity.full_scale_volts() - 10e-3).abs() < 1e-9);
    }

    // ------------------------------------------------------------------
    // 23. SR830 preset applies correct defaults
    // ------------------------------------------------------------------
    #[test]
    fn test_preset_sr830() {
        let config = LiaConfig::new(F_REF, FS).with_preset(LiaPreset::Sr830);
        assert_eq!(config.filter_order, FilterOrder::Second);
        assert!(config.ac_coupling_hz.is_some());
    }

    // ------------------------------------------------------------------
    // 24. SR844 preset applies bandpass pre-filter
    // ------------------------------------------------------------------
    #[test]
    fn test_preset_sr844() {
        let config = LiaConfig::new(F_REF, FS).with_preset(LiaPreset::Sr844);
        assert_eq!(config.filter_order, FilterOrder::First);
        assert!(config.prefilter_q.is_some());
        assert!(config.ac_coupling_hz.is_none());
    }

    // ------------------------------------------------------------------
    // 25. HF2LI preset uses 4th-order filter
    // ------------------------------------------------------------------
    #[test]
    fn test_preset_hf2li() {
        let config = LiaConfig::new(F_REF, FS).with_preset(LiaPreset::Hf2Li);
        assert_eq!(config.filter_order, FilterOrder::Fourth);
    }

    // ------------------------------------------------------------------
    // 26. AC coupling removes DC offset
    // ------------------------------------------------------------------
    #[test]
    fn test_ac_coupling_removes_dc() {
        let dc_offset = 5.0; // 5 V DC
        let amp = 0.01;
        let n = 30_000;
        let signal: Vec<f64> = (0..n)
            .map(|i| dc_offset + amp * (2.0 * PI * F_REF * i as f64 / FS).sin())
            .collect();

        let mut config = LiaConfig::new(F_REF, FS);
        config.ac_coupling_hz = Some(10.0);
        config.time_constant_s = 1e-3;
        let mut lia = LockInAmplifier::new(config);
        let outputs = lia.process_block(&signal);
        let last = outputs.last().unwrap();
        // Amplitude should still be ~amp
        assert!(
            (last.r - amp).abs() < 0.1 * amp + 1e-4,
            "R={} with DC offset, expected~{}", last.r, amp
        );
    }

    // ------------------------------------------------------------------
    // 27. set_reference_freq changes detection frequency
    // ------------------------------------------------------------------
    #[test]
    fn test_set_reference_freq() {
        let new_freq = 2_000.0;
        let config = LiaConfig::new(F_REF, FS);
        let mut lia = LockInAmplifier::new(config);
        lia.set_reference_freq(new_freq);
        assert!((lia.config().reference_freq_hz - new_freq).abs() < 1e-9);

        // Now it should detect the new frequency
        let amp = 0.01;
        let signal = sine_signal(new_freq, amp, 10_000, FS);
        let outputs = lia.process_block(&signal);
        let last = outputs.last().unwrap();
        assert!(
            (last.r - amp).abs() < 0.05 * amp,
            "After freq change R={} expected~{}", last.r, amp
        );
    }

    // ------------------------------------------------------------------
    // 28. set_reference_phase adjusts X/Y components
    // ------------------------------------------------------------------
    #[test]
    fn test_set_reference_phase() {
        let amp = 0.01;
        // Cosine signal: should appear as X when ref phase = 0
        let signal = cosine_signal(F_REF, amp, 10_000, FS);

        let config = LiaConfig::new(F_REF, FS);
        let mut lia = LockInAmplifier::new(config);
        let outputs = lia.process_block(&signal);
        let out0 = outputs.last().unwrap().clone();

        // Rotate reference by 90 degrees
        lia.reset();
        lia.set_reference_phase(PI / 2.0);
        let outputs2 = lia.process_block(&signal);
        let out90 = outputs2.last().unwrap();

        // R should be similar in both cases
        assert!((out0.r - out90.r).abs() < 0.001 * amp + 1e-6,
            "R should be phase-independent");
    }

    // ------------------------------------------------------------------
    // 29. set_time_constant rebuilds LPF
    // ------------------------------------------------------------------
    #[test]
    fn test_set_time_constant() {
        let config = LiaConfig::new(F_REF, FS);
        let mut lia = LockInAmplifier::new(config);
        lia.set_time_constant(50e-3);
        // NEBW should now be 1/(4*0.05) = 5 Hz
        let nebw = lia.noise_equivalent_bandwidth_hz();
        assert!((nebw - 5.0).abs() < 0.1, "NEBW={} expected 5 Hz", nebw);
    }

    // ------------------------------------------------------------------
    // 30. Bandpass pre-filter allows on-frequency signal
    // ------------------------------------------------------------------
    #[test]
    fn test_prefilter_passes_on_frequency() {
        let amp = 0.01;
        let n = 15_000;
        let signal = sine_signal(F_REF, amp, n, FS);

        let mut config = LiaConfig::new(F_REF, FS);
        config.prefilter_q = Some(3.0);
        config.time_constant_s = 1e-3;
        let mut lia = LockInAmplifier::new(config);
        let outputs = lia.process_block(&signal);
        let last = outputs.last().unwrap();
        // Pre-filter has some insertion loss at Q=3; allow 20% deviation
        assert!(
            (last.r - amp).abs() < 0.25 * amp,
            "Pre-filtered R={} expected~{}", last.r, amp
        );
    }

    // ------------------------------------------------------------------
    // 31. min_time_constant_for_snr returns positive value
    // ------------------------------------------------------------------
    #[test]
    fn test_min_time_constant_for_snr() {
        let tau = min_time_constant_for_snr(30.0, FS, FilterOrder::Second);
        assert!(tau > 0.0);
        assert!(tau < 1.0); // sanity upper bound
    }

    // ------------------------------------------------------------------
    // 32. reference_waveform utility produces correct lengths
    // ------------------------------------------------------------------
    #[test]
    fn test_reference_waveform_length() {
        let n = 100;
        let (c, s) = reference_waveform(F_REF, FS, n);
        assert_eq!(c.len(), n);
        assert_eq!(s.len(), n);
    }

    // ------------------------------------------------------------------
    // 33. reference_waveform orthogonality
    // ------------------------------------------------------------------
    #[test]
    fn test_reference_waveform_orthogonality() {
        // One full period of cos and sin should be orthogonal (dot product ~ 0)
        let n = (FS / F_REF) as usize; // samples per period
        let (c, s) = reference_waveform(F_REF, FS, n);
        let dot: f64 = c.iter().zip(s.iter()).map(|(a, b)| a * b).sum::<f64>() / n as f64;
        assert!(dot.abs() < 1e-10, "cos·sin should be ~0, got {}", dot);
    }

    // ------------------------------------------------------------------
    // 34. reference_waveform unit amplitude
    // ------------------------------------------------------------------
    #[test]
    fn test_reference_waveform_amplitude() {
        let n = 1000;
        let (c, s) = reference_waveform(F_REF, FS, n);
        let max_cos = c.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let max_sin = s.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!((max_cos - 1.0).abs() < 0.01, "cos peak={}", max_cos);
        assert!((max_sin - 1.0).abs() < 0.01, "sin peak={}", max_sin);
    }

    // ------------------------------------------------------------------
    // 35. Higher-order filter gives lower NEBW from the LIA method
    // ------------------------------------------------------------------
    #[test]
    fn test_lia_nebw_vs_order() {
        let make = |order| {
            let mut cfg = LiaConfig::new(F_REF, FS);
            cfg.filter_order = order;
            LockInAmplifier::new(cfg).noise_equivalent_bandwidth_hz()
        };
        let n1 = make(FilterOrder::First);
        let n2 = make(FilterOrder::Second);
        let n4 = make(FilterOrder::Fourth);
        assert!(n2 < n1);
        assert!(n4 < n2);
    }

    // ------------------------------------------------------------------
    // 36. Sensitivity full-scale values are monotonically increasing
    // ------------------------------------------------------------------
    #[test]
    fn test_sensitivity_full_scale_monotone() {
        use Sensitivity::*;
        let ranges = [
            NanoVolt1, NanoVolt2, NanoVolt5, NanoVolt10, NanoVolt20,
            NanoVolt50, NanoVolt100, NanoVolt200, NanoVolt500,
            MicroVolt1, MicroVolt2, MicroVolt5, MicroVolt10, MicroVolt20,
            MicroVolt50, MicroVolt100, MicroVolt200, MicroVolt500,
            MilliVolt1, MilliVolt2, MilliVolt5, MilliVolt10, MilliVolt20,
            MilliVolt50, MilliVolt100, MilliVolt200, MilliVolt500,
            Volt1,
        ];
        for w in ranges.windows(2) {
            assert!(
                w[0].full_scale_volts() < w[1].full_scale_volts(),
                "Sensitivity ranges not monotone at {:?}", w[0]
            );
        }
    }

    // ------------------------------------------------------------------
    // 37. X component dominates for in-phase cosine input
    // ------------------------------------------------------------------
    #[test]
    fn test_x_dominates_for_cosine_input() {
        let amp = 0.01;
        let n = 15_000;
        let signal = cosine_signal(F_REF, amp, n, FS);
        let config = LiaConfig::new(F_REF, FS);
        let mut lia = LockInAmplifier::new(config);
        let outputs = lia.process_block(&signal);
        let last = outputs.last().unwrap();
        // For cosine input with cosine reference, X should dominate
        assert!(last.x.abs() > last.y.abs(),
            "X={:.4} Y={:.4}: X should dominate for cosine input", last.x, last.y);
    }

    // ------------------------------------------------------------------
    // 38. Y component dominates for in-phase sine input
    // ------------------------------------------------------------------
    #[test]
    fn test_y_dominates_for_sine_input() {
        let amp = 0.01;
        let n = 15_000;
        let signal = sine_signal(F_REF, amp, n, FS);
        let config = LiaConfig::new(F_REF, FS);
        let mut lia = LockInAmplifier::new(config);
        let outputs = lia.process_block(&signal);
        let last = outputs.last().unwrap();
        // For sine input with cosine reference, Y should dominate
        assert!(last.y.abs() > last.x.abs(),
            "X={:.4} Y={:.4}: Y should dominate for sine input", last.x, last.y);
    }

    // ------------------------------------------------------------------
    // 39. Dual amplitude/phase: sum of 90-degree components gives amplitude
    // ------------------------------------------------------------------
    #[test]
    fn test_r_equals_amplitude_regardless_of_phase() {
        let amp = 0.02;
        // Use a long signal and longer tau for better settling; 50 000 samples = 50 tau
        let n = 50_000;
        // Try several phase offsets and confirm R ≈ amp each time
        for phi_deg in [0.0_f64, 30.0, 60.0, 90.0, 120.0, 180.0] {
            let phi = phi_deg.to_radians();
            let signal: Vec<f64> = (0..n)
                .map(|i| amp * (2.0 * PI * F_REF * i as f64 / FS + phi).sin())
                .collect();
            let mut config = LiaConfig::new(F_REF, FS);
            config.time_constant_s = 1e-3;
            let mut lia = LockInAmplifier::new(config);
            let outputs = lia.process_block(&signal);
            // Average the last 1000 samples to smooth 2f ripple
            let n_avg = 1000;
            let r_avg = outputs[outputs.len() - n_avg..]
                .iter()
                .map(|o| o.r)
                .sum::<f64>()
                / n_avg as f64;
            assert!(
                (r_avg - amp).abs() < 0.08 * amp,
                "phi={}° R_avg={:.5} expected~{}", phi_deg, r_avg, amp
            );
        }
    }

    // ------------------------------------------------------------------
    // 40. Auto-range selects a non-trivial sensitivity
    // ------------------------------------------------------------------
    #[test]
    fn test_auto_range() {
        let amp = 50e-6; // 50 µV
        let n = 20_000;
        let signal = sine_signal(F_REF, amp, n, FS);

        let mut config = LiaConfig::new(F_REF, FS);
        config.auto_range = true;
        config.sensitivity = Sensitivity::Volt1; // Start with large range
        let mut lia = LockInAmplifier::new(config);
        lia.process_block(&signal);
        // After processing, auto-range should have selected something < 1 V
        assert!(
            lia.config().sensitivity.full_scale_volts() < 1.0,
            "Auto-range should have reduced sensitivity"
        );
    }
}
