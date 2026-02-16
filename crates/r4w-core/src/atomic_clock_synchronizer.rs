//! Signal processing for atomic clock frequency standards and synchronization.
//!
//! This module implements the core signal processing algorithms used in atomic
//! frequency standards (cesium beam, rubidium cell, hydrogen maser, optical clocks)
//! and precision timing systems. Applications include GPS ground stations,
//! telecommunications timing (Stratum 1), VLBI correlation, IEEE 1588 PTP,
//! and national metrology laboratories.
//!
//! # Background
//!
//! Atomic clocks derive their frequency from quantum transitions in atoms.
//! The SI second is defined as 9,192,631,770 periods of the cesium-133
//! hyperfine transition. Clock comparison, stability analysis, and ensemble
//! timescale algorithms are essential for maintaining UTC and distributing
//! precise time.
//!
//! # Components
//!
//! | Struct / Function | Purpose |
//! |---|---|
//! | [`AtomicClockConfig`] | Clock type, nominal frequency, and stability parameters |
//! | [`AllanVarianceCalculator`] | ADEV, MDEV, TDEV from time/frequency data |
//! | [`FrequencyStabilityAnalyzer`] | Noise type identification from ADEV slope |
//! | [`PhaseComparator`] | Dual-mixer time difference (DMTD) measurement simulation |
//! | [`PllFrequencyLock`] | PLL for disciplining local oscillator to reference |
//! | [`CesiumBeamModel`] | Ramsey fringe pattern simulation |
//! | [`RubidiumCellModel`] | Optical pumping / microwave interrogation model |
//! | [`TimescaleAlgorithm`] | ALGOS-like weighted clock ensemble averaging |
//! | [`LeapSecondHandler`] | UTC leap second insertion/deletion tracking |
//! | [`TwoWayTimeTransfer`] | Satellite two-way time transfer delay computation |
//!
//! # Physics
//!
//! - Cs-133 hyperfine: f0 = 9,192,631,770 Hz (definition of the second)
//! - Rb-87 hyperfine: f0 = 6,834,682,610.904 Hz
//! - H maser: f0 = 1,420,405,751.768 Hz
//! - Allan variance: sigma_y^2(tau) = (1/2) * <(y_{n+1} - y_n)^2>
//! - Ramsey interrogation: P = (1/2)(1 + C*cos(2*pi*delta_f*T))
//!
//! # Example
//!
//! ```rust
//! use r4w_core::atomic_clock_synchronizer::{
//!     AtomicClockConfig, ClockType, AllanVarianceCalculator,
//!     CesiumBeamModel, TimescaleAlgorithm, ClockInput,
//! };
//!
//! // Configure a cesium beam standard
//! let cs_config = AtomicClockConfig::new(ClockType::CesiumBeam);
//! assert!((cs_config.nominal_frequency_hz - 9_192_631_770.0).abs() < 1.0);
//!
//! // Compute Allan deviation
//! let mut adev = AllanVarianceCalculator::new(1.0);
//! for i in 0..100 {
//!     adev.push_frequency(1e-12 * (i as f64 % 3.0 - 1.0));
//! }
//! let result = adev.compute_adev(1);
//! assert!(result.is_some());
//!
//! // Simulate Ramsey fringes
//! let cs_model = CesiumBeamModel::new(0.005, 0.95);
//! let prob = cs_model.transition_probability(0.0);
//! assert!((prob - 0.975).abs() < 0.01);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Cesium-133 hyperfine transition frequency in Hz (SI definition of the second).
pub const CESIUM_FREQUENCY_HZ: f64 = 9_192_631_770.0;

/// Rubidium-87 ground-state hyperfine frequency in Hz.
pub const RUBIDIUM_FREQUENCY_HZ: f64 = 6_834_682_610.904;

/// Hydrogen 21-cm hyperfine transition frequency in Hz.
pub const HYDROGEN_MASER_FREQUENCY_HZ: f64 = 1_420_405_751.768;

/// Strontium-87 optical clock transition frequency in Hz (698 nm).
pub const STRONTIUM_OPTICAL_FREQUENCY_HZ: f64 = 429_228_004_229_873.0;

/// Speed of light in m/s.
const SPEED_OF_LIGHT: f64 = 299_792_458.0;

// ---------------------------------------------------------------------------
// ClockType & AtomicClockConfig
// ---------------------------------------------------------------------------

/// Type of atomic frequency standard.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockType {
    /// Cesium beam tube (Ramsey interrogation). ADEV floor ~1e-14 at 10^4 s.
    CesiumBeam,
    /// Rubidium gas cell (optical pumping). ADEV floor ~1e-12 at 10^2 s.
    RubidiumCell,
    /// Active hydrogen maser. ADEV floor ~1e-15 at 10^3 s.
    HydrogenMaser,
    /// Optical lattice clock (Sr, Yb). ADEV floor ~1e-18 at 10^3 s.
    OpticalClock,
}

/// Configuration for an atomic clock.
#[derive(Debug, Clone)]
pub struct AtomicClockConfig {
    /// Type of atomic standard.
    pub clock_type: ClockType,
    /// Nominal transition frequency in Hz.
    pub nominal_frequency_hz: f64,
    /// ADEV floor (best achievable stability) as fractional frequency.
    pub adev_floor: f64,
    /// ADEV floor averaging time in seconds.
    pub adev_floor_tau: f64,
    /// Frequency accuracy (systematic uncertainty) as fractional frequency.
    pub accuracy: f64,
    /// Drift rate in fractional frequency per day (for masers with cavity drift).
    pub drift_rate_per_day: f64,
}

impl AtomicClockConfig {
    /// Create a configuration with typical parameters for the given clock type.
    pub fn new(clock_type: ClockType) -> Self {
        match clock_type {
            ClockType::CesiumBeam => Self {
                clock_type,
                nominal_frequency_hz: CESIUM_FREQUENCY_HZ,
                adev_floor: 1e-14,
                adev_floor_tau: 1e4,
                accuracy: 5e-15,
                drift_rate_per_day: 0.0, // primary standard, no drift by definition
            },
            ClockType::RubidiumCell => Self {
                clock_type,
                nominal_frequency_hz: RUBIDIUM_FREQUENCY_HZ,
                adev_floor: 1e-12,
                adev_floor_tau: 1e2,
                accuracy: 5e-11,
                drift_rate_per_day: 1e-12,
            },
            ClockType::HydrogenMaser => Self {
                clock_type,
                nominal_frequency_hz: HYDROGEN_MASER_FREQUENCY_HZ,
                adev_floor: 1e-15,
                adev_floor_tau: 1e3,
                accuracy: 1e-12, // limited by cavity pulling
                drift_rate_per_day: 1e-15,
            },
            ClockType::OpticalClock => Self {
                clock_type,
                nominal_frequency_hz: STRONTIUM_OPTICAL_FREQUENCY_HZ,
                adev_floor: 1e-18,
                adev_floor_tau: 1e3,
                accuracy: 1e-18,
                drift_rate_per_day: 0.0,
            },
        }
    }

    /// Create a custom configuration.
    pub fn custom(
        clock_type: ClockType,
        nominal_frequency_hz: f64,
        adev_floor: f64,
        adev_floor_tau: f64,
        accuracy: f64,
        drift_rate_per_day: f64,
    ) -> Self {
        Self {
            clock_type,
            nominal_frequency_hz,
            adev_floor,
            adev_floor_tau,
            accuracy,
            drift_rate_per_day,
        }
    }
}

// ---------------------------------------------------------------------------
// AllanVarianceCalculator
// ---------------------------------------------------------------------------

/// Result of an Allan deviation computation at a particular averaging time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdevPoint {
    /// Averaging time tau in seconds.
    pub tau: f64,
    /// Allan deviation (square root of Allan variance), dimensionless.
    pub adev: f64,
    /// Number of overlapping samples used in the computation.
    pub count: usize,
}

/// Result of a Modified Allan deviation computation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MdevPoint {
    /// Averaging time tau in seconds.
    pub tau: f64,
    /// Modified Allan deviation, dimensionless.
    pub mdev: f64,
    /// Number of samples used.
    pub count: usize,
}

/// Result of a Time deviation computation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TdevPoint {
    /// Averaging time tau in seconds.
    pub tau: f64,
    /// Time deviation in seconds.
    pub tdev: f64,
    /// Number of samples used.
    pub count: usize,
}

/// Computes Allan variance (ADEV), Modified Allan deviation (MDEV), and
/// Time deviation (TDEV) from frequency or phase data.
///
/// # Allan Variance
///
/// The overlapping Allan variance is:
/// ```text
/// sigma_y^2(tau) = 1/(2*m^2*(N-2m)) * sum_{i=0}^{N-2m-1} (x_{i+2m} - 2*x_{i+m} + x_i)^2
/// ```
/// where x_i are phase samples and tau = m * tau_0.
///
/// For frequency data y_i, phase is reconstructed as x_i = sum(y_j * tau_0).
///
/// # Modified Allan Deviation
///
/// MDEV uses additional averaging to distinguish white PM from flicker PM:
/// ```text
/// Mod sigma_y^2(tau) = 1/(2*m^4*n*(N-3m+1)) * sum ...
/// ```
#[derive(Debug, Clone)]
pub struct AllanVarianceCalculator {
    /// Base sampling interval in seconds.
    tau0: f64,
    /// Phase data x_i in seconds (accumulated from frequency data).
    phase_data: Vec<f64>,
}

impl AllanVarianceCalculator {
    /// Create a new calculator with base sampling interval `tau0` (seconds).
    pub fn new(tau0: f64) -> Self {
        assert!(tau0 > 0.0, "tau0 must be positive");
        Self {
            tau0,
            phase_data: vec![0.0], // x_0 = 0
        }
    }

    /// Push a fractional frequency sample y_i = (f - f_nom) / f_nom.
    /// Internally accumulates phase: x_{i+1} = x_i + y_i * tau_0.
    pub fn push_frequency(&mut self, y: f64) {
        let last_phase = *self.phase_data.last().unwrap();
        self.phase_data.push(last_phase + y * self.tau0);
    }

    /// Push a batch of fractional frequency samples.
    pub fn push_frequency_batch(&mut self, ys: &[f64]) {
        for &y in ys {
            self.push_frequency(y);
        }
    }

    /// Push a phase sample directly (in seconds).
    pub fn push_phase(&mut self, x: f64) {
        self.phase_data.push(x);
    }

    /// Push a batch of phase samples.
    pub fn push_phase_batch(&mut self, xs: &[f64]) {
        self.phase_data.extend_from_slice(xs);
    }

    /// Create from existing phase data (in seconds).
    pub fn from_phase_data(tau0: f64, phase_data: Vec<f64>) -> Self {
        assert!(tau0 > 0.0, "tau0 must be positive");
        assert!(phase_data.len() >= 3, "need at least 3 phase samples");
        Self { tau0, phase_data }
    }

    /// Number of phase samples stored.
    pub fn num_phase_samples(&self) -> usize {
        self.phase_data.len()
    }

    /// Compute overlapping Allan deviation for averaging factor m.
    /// tau = m * tau_0.
    ///
    /// Uses the phase-data formulation:
    /// ```text
    /// sigma_y^2(tau) = 1/(2 * tau^2 * (N - 2m)) * sum (x_{i+2m} - 2*x_{i+m} + x_i)^2
    /// ```
    pub fn compute_adev(&self, m: usize) -> Option<AdevPoint> {
        let n = self.phase_data.len();
        if m == 0 || n < 2 * m + 1 {
            return None;
        }

        let tau = m as f64 * self.tau0;
        let count = n - 2 * m;
        let mut sum = 0.0_f64;

        for i in 0..count {
            let diff = self.phase_data[i + 2 * m] - 2.0 * self.phase_data[i + m]
                + self.phase_data[i];
            sum += diff * diff;
        }

        let avar = sum / (2.0 * tau * tau * count as f64);
        Some(AdevPoint {
            tau,
            adev: avar.sqrt(),
            count,
        })
    }

    /// Compute ADEV for a range of averaging factors: m = 1, 2, 4, 8, ...
    /// up to max_m (or limited by data length).
    pub fn compute_adev_spectrum(&self, max_m: Option<usize>) -> Vec<AdevPoint> {
        let n = self.phase_data.len();
        let limit = max_m.unwrap_or(n / 3);
        let mut results = Vec::new();
        let mut m = 1;
        while m <= limit && n >= 2 * m + 1 {
            if let Some(point) = self.compute_adev(m) {
                results.push(point);
            }
            m *= 2;
        }
        results
    }

    /// Compute Modified Allan Deviation for averaging factor m.
    ///
    /// MDEV uses a second round of averaging to distinguish white phase noise
    /// from flicker phase noise. The formula uses phase data:
    /// ```text
    /// Mod sigma_y^2(tau) = 1/(2*m^2*tau^2*n) * sum_{j=0}^{n-1} (sum_{i=j}^{j+m-1} (x_{i+2m} - 2*x_{i+m} + x_i))^2
    /// ```
    /// where n = N - 3m + 1.
    pub fn compute_mdev(&self, m: usize) -> Option<MdevPoint> {
        let n_phase = self.phase_data.len();
        if m == 0 || n_phase < 3 * m + 1 {
            return None;
        }

        let tau = m as f64 * self.tau0;
        let n = n_phase - 3 * m; // number of outer terms

        let mut sum = 0.0_f64;
        for j in 0..=n {
            let mut inner_sum = 0.0_f64;
            for i in j..j + m {
                inner_sum +=
                    self.phase_data[i + 2 * m] - 2.0 * self.phase_data[i + m] + self.phase_data[i];
            }
            sum += inner_sum * inner_sum;
        }

        let mvar = sum / (2.0 * (m as f64).powi(2) * tau * tau * (n + 1) as f64);
        Some(MdevPoint {
            tau,
            mdev: mvar.sqrt(),
            count: n + 1,
        })
    }

    /// Compute Time Deviation for averaging factor m.
    ///
    /// TDEV = tau / sqrt(3) * MDEV(tau)
    pub fn compute_tdev(&self, m: usize) -> Option<TdevPoint> {
        self.compute_mdev(m).map(|mdev_point| {
            let tau = mdev_point.tau;
            TdevPoint {
                tau,
                tdev: tau / 3.0_f64.sqrt() * mdev_point.mdev,
                count: mdev_point.count,
            }
        })
    }
}

// ---------------------------------------------------------------------------
// FrequencyStabilityAnalyzer
// ---------------------------------------------------------------------------

/// Power-law noise type identified from ADEV slope.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NoiseType {
    /// White phase modulation: ADEV ~ tau^(-1), S_phi ~ f^0.
    WhitePhase,
    /// Flicker phase modulation: ADEV ~ tau^(-1), S_phi ~ f^(-1).
    /// Distinguished from WhitePhase by MDEV slope.
    FlickerPhase,
    /// White frequency modulation: ADEV ~ tau^(-1/2), S_y ~ f^0.
    WhiteFrequency,
    /// Flicker frequency modulation: ADEV ~ tau^0, S_y ~ f^(-1).
    FlickerFrequency,
    /// Random walk frequency modulation: ADEV ~ tau^(+1/2), S_y ~ f^(-2).
    RandomWalkFrequency,
    /// Frequency drift: ADEV ~ tau^(+1), deterministic.
    FrequencyDrift,
}

impl NoiseType {
    /// Expected ADEV slope (exponent mu where ADEV ~ tau^mu).
    pub fn adev_slope(self) -> f64 {
        match self {
            NoiseType::WhitePhase => -1.0,
            NoiseType::FlickerPhase => -1.0,
            NoiseType::WhiteFrequency => -0.5,
            NoiseType::FlickerFrequency => 0.0,
            NoiseType::RandomWalkFrequency => 0.5,
            NoiseType::FrequencyDrift => 1.0,
        }
    }

    /// Power spectral density exponent alpha where S_y(f) ~ f^alpha.
    pub fn psd_exponent(self) -> i32 {
        match self {
            NoiseType::WhitePhase => 2,
            NoiseType::FlickerPhase => 1,
            NoiseType::WhiteFrequency => 0,
            NoiseType::FlickerFrequency => -1,
            NoiseType::RandomWalkFrequency => -2,
            NoiseType::FrequencyDrift => -2, // plus deterministic component
        }
    }
}

/// Identifies the dominant noise type from ADEV data by fitting a power law.
///
/// Given ADEV measurements at multiple tau values, performs a log-log
/// least-squares fit to determine the slope mu in ADEV ~ tau^mu,
/// then maps mu to the corresponding [`NoiseType`].
#[derive(Debug, Clone)]
pub struct FrequencyStabilityAnalyzer {
    points: Vec<(f64, f64)>, // (tau, adev)
}

impl FrequencyStabilityAnalyzer {
    /// Create a new analyzer.
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// Add an ADEV measurement point.
    pub fn add_point(&mut self, tau: f64, adev: f64) {
        if tau > 0.0 && adev > 0.0 {
            self.points.push((tau, adev));
        }
    }

    /// Add points from ADEV computation results.
    pub fn add_adev_points(&mut self, points: &[AdevPoint]) {
        for p in points {
            self.add_point(p.tau, p.adev);
        }
    }

    /// Compute the log-log slope of ADEV vs tau using least-squares.
    ///
    /// Returns (slope, intercept) where ADEV = 10^intercept * tau^slope.
    /// Returns None if fewer than 2 points.
    pub fn fit_slope(&self) -> Option<(f64, f64)> {
        if self.points.len() < 2 {
            return None;
        }

        let n = self.points.len() as f64;
        let mut sum_x = 0.0_f64;
        let mut sum_y = 0.0_f64;
        let mut sum_xx = 0.0_f64;
        let mut sum_xy = 0.0_f64;

        for &(tau, adev) in &self.points {
            let x = tau.log10();
            let y = adev.log10();
            sum_x += x;
            sum_y += y;
            sum_xx += x * x;
            sum_xy += x * y;
        }

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return None;
        }

        let slope = (n * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / n;
        Some((slope, intercept))
    }

    /// Identify the dominant noise type from the ADEV slope.
    pub fn identify_noise_type(&self) -> Option<NoiseType> {
        let (slope, _) = self.fit_slope()?;

        // Map slope to noise type with tolerance bands
        if slope < -0.75 {
            // slope ~ -1: could be white PM or flicker PM
            // (ADEV alone cannot distinguish; need MDEV)
            Some(NoiseType::WhitePhase)
        } else if slope < -0.25 {
            // slope ~ -0.5: white FM
            Some(NoiseType::WhiteFrequency)
        } else if slope < 0.25 {
            // slope ~ 0: flicker FM
            Some(NoiseType::FlickerFrequency)
        } else if slope < 0.75 {
            // slope ~ +0.5: random walk FM
            Some(NoiseType::RandomWalkFrequency)
        } else {
            // slope ~ +1: frequency drift
            Some(NoiseType::FrequencyDrift)
        }
    }

    /// Return the number of data points.
    pub fn num_points(&self) -> usize {
        self.points.len()
    }
}

impl Default for FrequencyStabilityAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// PhaseComparator (DMTD)
// ---------------------------------------------------------------------------

/// Simulates a Dual-Mixer Time Difference (DMTD) phase comparator.
///
/// DMTD is the standard technique for measuring phase differences between
/// high-frequency oscillators with sub-picosecond resolution. Two oscillators
/// at frequency f are mixed with a common transfer oscillator at f + delta_f
/// (beat note offset), producing low-frequency beat signals whose phase
/// difference is magnified by f / delta_f.
///
/// # Operation
///
/// The measurement resolution enhancement factor is:
/// ```text
/// M = f_signal / f_beat = f_signal / delta_f
/// ```
///
/// A phase difference of dt seconds at the signal frequency appears as
/// dt * M seconds in the beat note.
#[derive(Debug, Clone)]
pub struct PhaseComparator {
    /// Signal frequency in Hz.
    signal_freq: f64,
    /// Beat frequency (transfer oscillator offset) in Hz.
    beat_freq: f64,
    /// Accumulated phase of reference channel (radians).
    ref_phase: f64,
    /// Accumulated phase of DUT channel (radians).
    dut_phase: f64,
    /// Measurement history (time differences in seconds).
    measurements: Vec<f64>,
}

impl PhaseComparator {
    /// Create a new DMTD phase comparator.
    ///
    /// * `signal_freq` - Nominal frequency of oscillators being compared (Hz).
    /// * `beat_freq` - Beat frequency / transfer oscillator offset (Hz).
    pub fn new(signal_freq: f64, beat_freq: f64) -> Self {
        assert!(signal_freq > 0.0, "signal frequency must be positive");
        assert!(beat_freq > 0.0, "beat frequency must be positive");
        assert!(
            beat_freq < signal_freq,
            "beat freq must be less than signal freq"
        );
        Self {
            signal_freq,
            beat_freq,
            ref_phase: 0.0,
            dut_phase: 0.0,
            measurements: Vec::new(),
        }
    }

    /// Resolution enhancement factor M = f_signal / f_beat.
    pub fn multiplication_factor(&self) -> f64 {
        self.signal_freq / self.beat_freq
    }

    /// Single-shot measurement resolution in seconds.
    ///
    /// Assuming the beat note zero-crossing can be timed to ~1 ns,
    /// the equivalent resolution at the signal frequency is 1ns / M.
    pub fn resolution_seconds(&self) -> f64 {
        1e-9 / self.multiplication_factor()
    }

    /// Simulate a DMTD measurement.
    ///
    /// * `ref_freq_offset` - Reference oscillator fractional frequency offset.
    /// * `dut_freq_offset` - DUT oscillator fractional frequency offset.
    /// * `measurement_interval` - Gate time in seconds.
    ///
    /// Returns the measured time difference in seconds.
    pub fn measure(
        &mut self,
        ref_freq_offset: f64,
        dut_freq_offset: f64,
        measurement_interval: f64,
    ) -> f64 {
        // Phase accumulated during the measurement interval
        let ref_phase_inc =
            2.0 * PI * self.signal_freq * (1.0 + ref_freq_offset) * measurement_interval;
        let dut_phase_inc =
            2.0 * PI * self.signal_freq * (1.0 + dut_freq_offset) * measurement_interval;

        self.ref_phase += ref_phase_inc;
        self.dut_phase += dut_phase_inc;

        // Phase difference converted to time
        let phase_diff = self.dut_phase - self.ref_phase;
        let time_diff = phase_diff / (2.0 * PI * self.signal_freq);

        self.measurements.push(time_diff);
        time_diff
    }

    /// Return all accumulated measurements.
    pub fn measurements(&self) -> &[f64] {
        &self.measurements
    }

    /// Reset the comparator state.
    pub fn reset(&mut self) {
        self.ref_phase = 0.0;
        self.dut_phase = 0.0;
        self.measurements.clear();
    }

    /// Compute the mean time difference from all measurements.
    pub fn mean_time_difference(&self) -> Option<f64> {
        if self.measurements.is_empty() {
            return None;
        }
        let sum: f64 = self.measurements.iter().sum();
        Some(sum / self.measurements.len() as f64)
    }
}

// ---------------------------------------------------------------------------
// PllFrequencyLock
// ---------------------------------------------------------------------------

/// Phase-locked loop for disciplining a local oscillator to a reference.
///
/// Implements a second-order type-2 PLL with proportional-integral loop
/// filter. Used to lock a VCXO or synthesizer to an atomic reference,
/// providing the short-term stability of the local oscillator with the
/// long-term accuracy of the reference.
///
/// # Transfer function
///
/// The loop filter is:
/// ```text
/// F(s) = Kp + Ki/s
/// ```
///
/// Natural frequency and damping:
/// ```text
/// omega_n = sqrt(Kd * Ko * Ki)
/// zeta = (Kp / 2) * sqrt(Kd * Ko / Ki)
/// ```
#[derive(Debug, Clone)]
pub struct PllFrequencyLock {
    /// Proportional gain.
    kp: f64,
    /// Integral gain.
    ki: f64,
    /// Phase detector gain (rad/rad).
    kd: f64,
    /// VCO gain (Hz/V or Hz/unit).
    ko: f64,
    /// Loop filter integrator state.
    integrator: f64,
    /// Current phase error (radians).
    phase_error: f64,
    /// Current frequency correction (fractional).
    freq_correction: f64,
    /// Accumulated local phase (radians).
    local_phase: f64,
    /// Sample interval (seconds).
    dt: f64,
    /// Lock indicator (true when phase error is small).
    locked: bool,
    /// Lock threshold in radians.
    lock_threshold: f64,
}

impl PllFrequencyLock {
    /// Create a PLL with specified bandwidth and damping.
    ///
    /// * `bandwidth_hz` - Loop noise bandwidth in Hz.
    /// * `damping` - Damping ratio (typically 0.707 for critically damped).
    /// * `dt` - Update interval in seconds.
    pub fn new(bandwidth_hz: f64, damping: f64, dt: f64) -> Self {
        assert!(bandwidth_hz > 0.0, "bandwidth must be positive");
        assert!(damping > 0.0, "damping must be positive");
        assert!(dt > 0.0, "dt must be positive");

        // Derive gains from bandwidth and damping
        // omega_n = 2 * pi * BW / (damping + 1/(4*damping))
        let omega_n = 2.0 * PI * bandwidth_hz / (damping + 1.0 / (4.0 * damping));
        let kd = 1.0; // normalized phase detector
        let ko = 1.0; // normalized VCO

        let ki = omega_n * omega_n / (kd * ko);
        let kp = 2.0 * damping * omega_n / (kd * ko);

        Self {
            kp,
            ki,
            kd,
            ko,
            integrator: 0.0,
            phase_error: 0.0,
            freq_correction: 0.0,
            local_phase: 0.0,
            dt,
            locked: false,
            lock_threshold: 0.1, // ~6 degrees
        }
    }

    /// Process one sample: compare reference phase to local phase, update loop.
    ///
    /// * `ref_phase` - Reference oscillator phase in radians.
    ///
    /// Returns the current frequency correction (fractional).
    pub fn update(&mut self, ref_phase: f64) -> f64 {
        // Phase detector
        self.phase_error = ref_phase - self.local_phase;

        // Wrap to [-pi, pi]
        self.phase_error = wrap_phase(self.phase_error);

        let detector_output = self.kd * self.phase_error;

        // Loop filter (PI)
        self.integrator += self.ki * detector_output * self.dt;
        let filter_output = self.kp * detector_output + self.integrator;

        // VCO
        self.freq_correction = self.ko * filter_output;

        // Update local phase
        self.local_phase += 2.0 * PI * self.freq_correction * self.dt;

        // Lock detection
        self.locked = self.phase_error.abs() < self.lock_threshold;

        self.freq_correction
    }

    /// Check if the PLL is locked (phase error below threshold).
    pub fn is_locked(&self) -> bool {
        self.locked
    }

    /// Current phase error in radians.
    pub fn phase_error(&self) -> f64 {
        self.phase_error
    }

    /// Current frequency correction (fractional frequency offset).
    pub fn frequency_correction(&self) -> f64 {
        self.freq_correction
    }

    /// Set the lock detection threshold in radians.
    pub fn set_lock_threshold(&mut self, threshold: f64) {
        self.lock_threshold = threshold.abs();
    }

    /// Reset the PLL state.
    pub fn reset(&mut self) {
        self.integrator = 0.0;
        self.phase_error = 0.0;
        self.freq_correction = 0.0;
        self.local_phase = 0.0;
        self.locked = false;
    }

    /// Return the natural frequency of the loop in Hz.
    pub fn natural_frequency(&self) -> f64 {
        (self.kd * self.ko * self.ki).sqrt() / (2.0 * PI)
    }

    /// Return the damping ratio.
    pub fn damping_ratio(&self) -> f64 {
        (self.kp / 2.0) * (self.kd * self.ko / self.ki).sqrt()
    }
}

// ---------------------------------------------------------------------------
// CesiumBeamModel
// ---------------------------------------------------------------------------

/// Simulates the Ramsey fringe pattern of a cesium beam frequency standard.
///
/// In a cesium beam clock, atoms pass through two microwave cavities separated
/// by a drift region. The transition probability as a function of frequency
/// detuning follows the Ramsey pattern:
///
/// ```text
/// P(delta_f) = (1/2) * [1 + C * cos(2*pi*delta_f*T)] * sinc^2(pi*delta_f*t)
/// ```
///
/// where:
/// - T is the Ramsey time (drift region transit time)
/// - t is the single-cavity interaction time
/// - C is the fringe contrast (0 to 1)
/// - delta_f = f - f_cesium
#[derive(Debug, Clone)]
pub struct CesiumBeamModel {
    /// Ramsey drift time T in seconds (typical: 1-10 ms).
    ramsey_time: f64,
    /// Single cavity interaction time t in seconds (typical: T/100).
    cavity_time: f64,
    /// Fringe contrast (0 to 1). Limited by velocity distribution.
    contrast: f64,
    /// Cesium hyperfine frequency.
    f0: f64,
}

impl CesiumBeamModel {
    /// Create a cesium beam model.
    ///
    /// * `ramsey_time` - Drift region transit time in seconds.
    /// * `contrast` - Fringe contrast factor (0 to 1).
    pub fn new(ramsey_time: f64, contrast: f64) -> Self {
        assert!(ramsey_time > 0.0);
        assert!((0.0..=1.0).contains(&contrast));
        Self {
            ramsey_time,
            cavity_time: ramsey_time / 100.0,
            contrast,
            f0: CESIUM_FREQUENCY_HZ,
        }
    }

    /// Create with explicit cavity interaction time.
    pub fn with_cavity_time(ramsey_time: f64, cavity_time: f64, contrast: f64) -> Self {
        assert!(ramsey_time > 0.0);
        assert!(cavity_time > 0.0);
        assert!((0.0..=1.0).contains(&contrast));
        Self {
            ramsey_time,
            cavity_time,
            contrast,
            f0: CESIUM_FREQUENCY_HZ,
        }
    }

    /// Compute transition probability at frequency f (Hz).
    pub fn transition_probability(&self, delta_f: f64) -> f64 {
        let ramsey_arg = PI * delta_f * self.ramsey_time;
        let cavity_arg = PI * delta_f * self.cavity_time;

        // Ramsey fringe: (1/2)(1 + C*cos(2*pi*delta_f*T))
        let ramsey_fringe = 0.5 * (1.0 + self.contrast * (2.0 * ramsey_arg).cos());

        // Single-cavity sinc envelope: sinc^2(pi*delta_f*t)
        let sinc_env = if cavity_arg.abs() < 1e-15 {
            1.0
        } else {
            let s = cavity_arg.sin() / cavity_arg;
            s * s
        };

        ramsey_fringe * sinc_env
    }

    /// Compute the Ramsey fringe pattern over a frequency range.
    ///
    /// Returns Vec of (delta_f, probability) pairs.
    pub fn compute_fringes(&self, delta_f_range: f64, num_points: usize) -> Vec<(f64, f64)> {
        let mut result = Vec::with_capacity(num_points);
        for i in 0..num_points {
            let delta_f = -delta_f_range + 2.0 * delta_f_range * (i as f64) / (num_points as f64 - 1.0);
            result.push((delta_f, self.transition_probability(delta_f)));
        }
        result
    }

    /// Central fringe width (FWHM) in Hz, approximately 1/(2*T).
    pub fn fringe_width_hz(&self) -> f64 {
        1.0 / (2.0 * self.ramsey_time)
    }

    /// The Q factor of the atomic resonance: f0 / delta_f.
    pub fn quality_factor(&self) -> f64 {
        self.f0 / self.fringe_width_hz()
    }

    /// Nominal cesium frequency.
    pub fn nominal_frequency(&self) -> f64 {
        self.f0
    }

    /// Ramsey time.
    pub fn ramsey_time(&self) -> f64 {
        self.ramsey_time
    }
}

// ---------------------------------------------------------------------------
// RubidiumCellModel
// ---------------------------------------------------------------------------

/// Simulates a rubidium gas cell frequency standard.
///
/// Rubidium clocks use optical pumping with a Rb-85 discharge lamp and
/// Rb-87 filter cell to prepare atoms, then microwave interrogation at the
/// 6.834 GHz ground-state hyperfine transition. The absorption signal
/// (double resonance) is modeled as a Lorentzian.
///
/// # Buffer gas shift
///
/// Buffer gases (N2, Ar) are added to reduce Doppler broadening but cause
/// a collisional frequency shift:
/// ```text
/// delta_f = P * (alpha + beta * (T - T0))
/// ```
/// where P is buffer gas pressure, alpha is the pressure coefficient,
/// and beta is the temperature coefficient of the shift.
#[derive(Debug, Clone)]
pub struct RubidiumCellModel {
    /// Rb-87 hyperfine frequency.
    f0: f64,
    /// Linewidth of the microwave resonance in Hz (FWHM).
    linewidth: f64,
    /// Signal contrast (peak absorption change, 0 to 1).
    contrast: f64,
    /// Buffer gas pressure shift in Hz.
    pressure_shift: f64,
    /// Temperature coefficient of pressure shift in Hz/K.
    temp_coefficient: f64,
    /// Reference temperature in Celsius.
    ref_temp_c: f64,
}

impl RubidiumCellModel {
    /// Create a rubidium cell model.
    ///
    /// * `linewidth` - Microwave resonance linewidth in Hz (typical: 500-2000 Hz).
    /// * `contrast` - Signal contrast (typical: 0.01-0.05 for optical pumping).
    pub fn new(linewidth: f64, contrast: f64) -> Self {
        assert!(linewidth > 0.0);
        assert!(contrast > 0.0 && contrast <= 1.0);
        Self {
            f0: RUBIDIUM_FREQUENCY_HZ,
            linewidth,
            contrast,
            pressure_shift: 0.0,
            temp_coefficient: 0.0,
            ref_temp_c: 25.0,
        }
    }

    /// Set buffer gas parameters.
    ///
    /// * `pressure_shift` - Frequency shift in Hz at reference temperature.
    /// * `temp_coefficient` - Temperature sensitivity of shift in Hz/K.
    /// * `ref_temp_c` - Reference temperature in Celsius.
    pub fn set_buffer_gas(
        &mut self,
        pressure_shift: f64,
        temp_coefficient: f64,
        ref_temp_c: f64,
    ) {
        self.pressure_shift = pressure_shift;
        self.temp_coefficient = temp_coefficient;
        self.ref_temp_c = ref_temp_c;
    }

    /// Lorentzian absorption signal as a function of frequency detuning.
    ///
    /// Returns a value between 0 (off resonance) and contrast (on resonance).
    pub fn absorption_signal(&self, delta_f: f64) -> f64 {
        let half_width = self.linewidth / 2.0;
        self.contrast * (half_width * half_width) / (delta_f * delta_f + half_width * half_width)
    }

    /// Effective center frequency including buffer gas shift at temperature T.
    pub fn effective_frequency(&self, temp_c: f64) -> f64 {
        let delta_t = temp_c - self.ref_temp_c;
        self.f0 + self.pressure_shift + self.temp_coefficient * delta_t
    }

    /// Discriminator signal (derivative of Lorentzian) for frequency servo.
    ///
    /// This is the error signal used to lock to the center:
    /// ```text
    /// D(delta_f) = -2 * C * gamma^2 * delta_f / (delta_f^2 + gamma^2)^2
    /// ```
    pub fn discriminator_signal(&self, delta_f: f64) -> f64 {
        let gamma = self.linewidth / 2.0;
        let denom = delta_f * delta_f + gamma * gamma;
        -2.0 * self.contrast * gamma * gamma * delta_f / (denom * denom)
    }

    /// Compute the absorption profile over a frequency range.
    pub fn compute_profile(&self, delta_f_range: f64, num_points: usize) -> Vec<(f64, f64)> {
        let mut result = Vec::with_capacity(num_points);
        for i in 0..num_points {
            let df = -delta_f_range + 2.0 * delta_f_range * (i as f64) / (num_points as f64 - 1.0);
            result.push((df, self.absorption_signal(df)));
        }
        result
    }

    /// Nominal Rb-87 frequency.
    pub fn nominal_frequency(&self) -> f64 {
        self.f0
    }

    /// Resonance Q factor.
    pub fn quality_factor(&self) -> f64 {
        self.f0 / self.linewidth
    }
}

// ---------------------------------------------------------------------------
// TimescaleAlgorithm
// ---------------------------------------------------------------------------

/// Input data from a single clock for timescale computation.
#[derive(Debug, Clone)]
pub struct ClockInput {
    /// Clock identifier.
    pub id: usize,
    /// Phase reading (time difference from ensemble) in seconds.
    pub phase: f64,
    /// Frequency offset (fractional) estimated for this clock.
    pub frequency_offset: f64,
    /// Weight (inverse variance or manually assigned).
    pub weight: f64,
    /// Whether this clock is considered valid/healthy.
    pub valid: bool,
}

/// Result of a timescale ensemble computation.
#[derive(Debug, Clone)]
pub struct TimescaleResult {
    /// Weighted average phase (ensemble time offset) in seconds.
    pub ensemble_phase: f64,
    /// Weighted average frequency offset (fractional).
    pub ensemble_frequency: f64,
    /// Normalized weights used for each clock.
    pub weights: Vec<f64>,
    /// Number of valid clocks used.
    pub num_valid: usize,
    /// Detected anomalous clock IDs (if any).
    pub anomalous_clocks: Vec<usize>,
}

/// Implements an ALGOS-like timescale algorithm for combining multiple clocks
/// into a stable ensemble.
///
/// The algorithm:
/// 1. Weights each clock inversely proportional to its estimated variance
/// 2. Computes a weighted average phase and frequency
/// 3. Detects and excludes outlier clocks
/// 4. Limits maximum weight to prevent single-clock dominance
///
/// This is similar to the AT1 algorithm used by NIST for UTC(NIST) and the
/// ALGOS algorithm used by BIPM for TAI/UTC.
#[derive(Debug, Clone)]
pub struct TimescaleAlgorithm {
    /// Maximum weight any single clock can have (prevents dominance).
    max_weight_fraction: f64,
    /// Outlier detection threshold (in units of weighted RMS).
    outlier_threshold: f64,
    /// History of ensemble results.
    history: Vec<TimescaleResult>,
}

impl TimescaleAlgorithm {
    /// Create a new timescale algorithm.
    ///
    /// * `max_weight_fraction` - Maximum weight for any single clock (0 to 1, typically 0.3-0.5).
    /// * `outlier_threshold` - Outlier rejection threshold in sigma units (typically 3.0-5.0).
    pub fn new(max_weight_fraction: f64, outlier_threshold: f64) -> Self {
        assert!(
            (0.0..=1.0).contains(&max_weight_fraction),
            "max weight must be 0-1"
        );
        assert!(outlier_threshold > 0.0, "threshold must be positive");
        Self {
            max_weight_fraction,
            outlier_threshold,
            history: Vec::new(),
        }
    }

    /// Compute ensemble timescale from clock inputs.
    ///
    /// Performs weighted averaging with outlier detection and weight capping.
    pub fn compute(&mut self, clocks: &[ClockInput]) -> TimescaleResult {
        // Filter valid clocks
        let valid_clocks: Vec<&ClockInput> = clocks.iter().filter(|c| c.valid).collect();
        let n = valid_clocks.len();

        if n == 0 {
            let result = TimescaleResult {
                ensemble_phase: 0.0,
                ensemble_frequency: 0.0,
                weights: vec![0.0; clocks.len()],
                num_valid: 0,
                anomalous_clocks: Vec::new(),
            };
            self.history.push(result.clone());
            return result;
        }

        // Normalize weights with iterative capping.
        // Repeatedly cap any weight above max_weight_fraction and redistribute
        // the excess among uncapped clocks until no weight exceeds the cap.
        let total_weight: f64 = valid_clocks.iter().map(|c| c.weight).sum();
        let mut norm_weights: Vec<f64> = if total_weight > 0.0 {
            valid_clocks.iter().map(|c| c.weight / total_weight).collect()
        } else {
            vec![1.0 / n as f64; n]
        };

        // Iterative capping: fix weights above cap, redistribute remainder
        let mut capped = vec![false; n];
        for _ in 0..n {
            let mut any_capped = false;
            let mut excess = 0.0_f64;
            let mut uncapped_sum = 0.0_f64;
            for i in 0..n {
                if !capped[i] && norm_weights[i] > self.max_weight_fraction {
                    excess += norm_weights[i] - self.max_weight_fraction;
                    norm_weights[i] = self.max_weight_fraction;
                    capped[i] = true;
                    any_capped = true;
                }
                if !capped[i] {
                    uncapped_sum += norm_weights[i];
                }
            }
            if !any_capped || uncapped_sum <= 0.0 {
                break;
            }
            // Redistribute excess proportionally among uncapped clocks
            for i in 0..n {
                if !capped[i] {
                    norm_weights[i] += excess * norm_weights[i] / uncapped_sum;
                }
            }
        }

        // Final re-normalization to ensure sum = 1
        let total: f64 = norm_weights.iter().sum();
        if total > 0.0 && (total - 1.0).abs() > 1e-15 {
            for w in &mut norm_weights {
                *w /= total;
            }
        }

        // Weighted average phase
        let ensemble_phase: f64 = valid_clocks
            .iter()
            .zip(norm_weights.iter())
            .map(|(c, &w)| c.phase * w)
            .sum();

        // Weighted average frequency
        let ensemble_frequency: f64 = valid_clocks
            .iter()
            .zip(norm_weights.iter())
            .map(|(c, &w)| c.frequency_offset * w)
            .sum();

        // Outlier detection: compute weighted RMS residual
        let weighted_rms: f64 = {
            let var: f64 = valid_clocks
                .iter()
                .zip(norm_weights.iter())
                .map(|(c, &w)| {
                    let residual = c.phase - ensemble_phase;
                    w * residual * residual
                })
                .sum();
            var.sqrt()
        };

        let mut anomalous_clocks = Vec::new();
        if weighted_rms > 0.0 {
            for (i, c) in valid_clocks.iter().enumerate() {
                let residual = (c.phase - ensemble_phase).abs();
                if residual > self.outlier_threshold * weighted_rms {
                    anomalous_clocks.push(c.id);
                    // In a real implementation, we would re-compute without this clock
                    // For simplicity, we flag it but don't re-iterate
                    if i < norm_weights.len() {
                        norm_weights[i] = 0.0;
                    }
                }
            }
        }

        // Build full weight vector (indexed by clock input order)
        let mut full_weights = vec![0.0; clocks.len()];
        let mut valid_idx = 0;
        for (i, c) in clocks.iter().enumerate() {
            if c.valid && valid_idx < norm_weights.len() {
                full_weights[i] = norm_weights[valid_idx];
                valid_idx += 1;
            }
        }

        let result = TimescaleResult {
            ensemble_phase,
            ensemble_frequency,
            weights: full_weights,
            num_valid: n,
            anomalous_clocks,
        };
        self.history.push(result.clone());
        result
    }

    /// Return the history of ensemble computations.
    pub fn history(&self) -> &[TimescaleResult] {
        &self.history
    }

    /// Reset the algorithm state.
    pub fn reset(&mut self) {
        self.history.clear();
    }
}

// ---------------------------------------------------------------------------
// LeapSecondHandler
// ---------------------------------------------------------------------------

/// Leap second event type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LeapSecondType {
    /// Positive leap second: 23:59:60 is inserted.
    Insertion,
    /// Negative leap second: 23:59:59 is skipped (never used to date).
    Deletion,
}

/// A leap second event.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LeapSecondEvent {
    /// Year of the leap second.
    pub year: u32,
    /// Month (1-12), typically June (6) or December (12).
    pub month: u8,
    /// Day of month (always last day: 30 for June, 31 for December).
    pub day: u8,
    /// Type of leap second.
    pub leap_type: LeapSecondType,
}

/// Tracks UTC leap second history and computes UTC-TAI offset.
///
/// International Atomic Time (TAI) runs continuously without leap seconds.
/// UTC = TAI - offset, where offset increases by 1 with each positive leap
/// second. As of 2017, UTC = TAI - 37s.
///
/// Note: From 2035 onwards, UTC will no longer use leap seconds (ITU-R
/// decision 2023), transitioning to a continuous timescale.
#[derive(Debug, Clone)]
pub struct LeapSecondHandler {
    /// Chronological list of leap second events.
    events: Vec<LeapSecondEvent>,
    /// Initial TAI-UTC offset (at epoch before first event).
    initial_offset: i32,
}

impl LeapSecondHandler {
    /// Create with the standard leap second table.
    ///
    /// Includes all leap seconds from 1972 to 2017 (total offset: 37s).
    pub fn new() -> Self {
        // Subset of key leap seconds for the table
        let events = vec![
            LeapSecondEvent { year: 1972, month: 6, day: 30, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1972, month: 12, day: 31, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1973, month: 12, day: 31, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1974, month: 12, day: 31, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1975, month: 12, day: 31, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1976, month: 12, day: 31, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1977, month: 12, day: 31, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1978, month: 12, day: 31, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1979, month: 12, day: 31, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1981, month: 6, day: 30, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1982, month: 6, day: 30, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1983, month: 6, day: 30, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1985, month: 6, day: 30, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1987, month: 12, day: 31, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1989, month: 12, day: 31, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1990, month: 12, day: 31, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1992, month: 6, day: 30, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1993, month: 6, day: 30, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1994, month: 6, day: 30, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1995, month: 12, day: 31, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1997, month: 6, day: 30, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 1998, month: 12, day: 31, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 2005, month: 12, day: 31, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 2008, month: 12, day: 31, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 2012, month: 6, day: 30, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 2015, month: 6, day: 30, leap_type: LeapSecondType::Insertion },
            LeapSecondEvent { year: 2016, month: 12, day: 31, leap_type: LeapSecondType::Insertion },
        ];

        Self {
            events,
            initial_offset: 10, // TAI-UTC = 10s at 1972-01-01
        }
    }

    /// Create an empty handler (no leap seconds loaded).
    pub fn empty() -> Self {
        Self {
            events: Vec::new(),
            initial_offset: 10,
        }
    }

    /// Add a leap second event.
    pub fn add_event(&mut self, event: LeapSecondEvent) {
        self.events.push(event);
        // Keep sorted by date
        self.events.sort_by(|a, b| {
            a.year.cmp(&b.year)
                .then(a.month.cmp(&b.month))
                .then(a.day.cmp(&b.day))
        });
    }

    /// Compute TAI-UTC offset at a given date.
    ///
    /// Returns the number of leap seconds accumulated up to and including
    /// the given date.
    pub fn tai_utc_offset(&self, year: u32, month: u8, day: u8) -> i32 {
        let mut offset = self.initial_offset;
        for event in &self.events {
            if (event.year, event.month, event.day) <= (year, month, day) {
                match event.leap_type {
                    LeapSecondType::Insertion => offset += 1,
                    LeapSecondType::Deletion => offset -= 1,
                }
            }
        }
        offset
    }

    /// Convert a UTC timestamp to TAI.
    ///
    /// * `utc_seconds` - Seconds since an epoch (e.g., Unix epoch + offset).
    /// * `year`, `month`, `day` - Date for looking up leap second offset.
    ///
    /// Returns TAI seconds = UTC seconds + TAI-UTC offset.
    pub fn utc_to_tai(&self, utc_seconds: f64, year: u32, month: u8, day: u8) -> f64 {
        utc_seconds + self.tai_utc_offset(year, month, day) as f64
    }

    /// Convert TAI to UTC (inverse of utc_to_tai).
    pub fn tai_to_utc(&self, tai_seconds: f64, year: u32, month: u8, day: u8) -> f64 {
        tai_seconds - self.tai_utc_offset(year, month, day) as f64
    }

    /// GPS time offset from UTC at a given date.
    ///
    /// GPS time started at UTC 1980-01-06 with TAI-GPS = 19s.
    /// GPS-UTC = TAI-UTC - 19.
    pub fn gps_utc_offset(&self, year: u32, month: u8, day: u8) -> i32 {
        self.tai_utc_offset(year, month, day) - 19
    }

    /// Number of leap second events in the table.
    pub fn num_events(&self) -> usize {
        self.events.len()
    }

    /// Get the list of all events.
    pub fn events(&self) -> &[LeapSecondEvent] {
        &self.events
    }

    /// Current TAI-UTC offset (based on last event).
    pub fn current_offset(&self) -> i32 {
        if let Some(last) = self.events.last() {
            self.tai_utc_offset(last.year, last.month, last.day)
        } else {
            self.initial_offset
        }
    }
}

impl Default for LeapSecondHandler {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// TwoWayTimeTransfer
// ---------------------------------------------------------------------------

/// Satellite Two-Way Time and Frequency Transfer (TWSTFT) computation.
///
/// TWSTFT determines the clock difference between two ground stations by
/// exchanging signals through a geostationary satellite. Each station
/// simultaneously transmits and receives, and the clock offset is derived
/// from the difference in measured pseudo-ranges:
///
/// ```text
/// delta_T = (T_a - T_b) / 2 + corrections
/// ```
///
/// where T_a and T_b are the measured time intervals at stations A and B.
///
/// Corrections account for:
/// - Sagnac effect (Earth rotation)
/// - Ionospheric delay (frequency-dependent)
/// - Tropospheric delay
/// - Satellite transponder delay asymmetry
/// - Equipment delays
#[derive(Debug, Clone)]
pub struct TwoWayTimeTransfer {
    /// Satellite altitude above Earth's surface in meters.
    satellite_altitude_m: f64,
    /// Earth radius in meters.
    earth_radius_m: f64,
    /// Transponder turnaround delay in seconds.
    transponder_delay_s: f64,
    /// Station A equipment delay in seconds.
    station_a_delay_s: f64,
    /// Station B equipment delay in seconds.
    station_b_delay_s: f64,
}

impl TwoWayTimeTransfer {
    /// Create a TWSTFT computation for a geostationary satellite.
    ///
    /// * `satellite_altitude_m` - Satellite altitude (typical GEO: 35,786 km).
    pub fn new(satellite_altitude_m: f64) -> Self {
        Self {
            satellite_altitude_m,
            earth_radius_m: 6_371_000.0,
            transponder_delay_s: 0.0,
            station_a_delay_s: 0.0,
            station_b_delay_s: 0.0,
        }
    }

    /// Create with typical GEO satellite parameters.
    pub fn geo_default() -> Self {
        Self::new(35_786_000.0)
    }

    /// Set transponder turnaround delay.
    pub fn set_transponder_delay(&mut self, delay_s: f64) {
        self.transponder_delay_s = delay_s;
    }

    /// Set station equipment delays.
    pub fn set_station_delays(&mut self, station_a_s: f64, station_b_s: f64) {
        self.station_a_delay_s = station_a_s;
        self.station_b_delay_s = station_b_s;
    }

    /// Compute one-way propagation delay from a ground station to the satellite.
    ///
    /// * `elevation_deg` - Elevation angle from station to satellite in degrees.
    ///
    /// Returns delay in seconds.
    pub fn one_way_delay(&self, elevation_deg: f64) -> f64 {
        let el_rad = elevation_deg * PI / 180.0;
        let r_e = self.earth_radius_m;
        let h = self.satellite_altitude_m;

        // Standard slant range formula:
        // d = -R_e*sin(el) + sqrt((R_e*sin(el))^2 + 2*R_e*h + h^2)
        let sin_el = el_rad.sin();
        let slant_range = -r_e * sin_el
            + (r_e * r_e * sin_el * sin_el + 2.0 * r_e * h + h * h).sqrt();

        slant_range / SPEED_OF_LIGHT
    }

    /// Compute the clock offset between two stations.
    ///
    /// * `t_a` - Time interval measured at station A (seconds).
    /// * `t_b` - Time interval measured at station B (seconds).
    /// * `sagnac_correction` - Sagnac effect correction in seconds.
    ///
    /// Returns the clock difference (station A - station B) in seconds.
    pub fn compute_clock_offset(
        &self,
        t_a: f64,
        t_b: f64,
        sagnac_correction: f64,
    ) -> f64 {
        // Basic TWSTFT equation:
        // delta_clock = (T_a - T_b) / 2 + (delay_b - delay_a) / 2 + sagnac
        let equipment_asymmetry = (self.station_b_delay_s - self.station_a_delay_s) / 2.0;
        (t_a - t_b) / 2.0 + equipment_asymmetry + sagnac_correction
    }

    /// Compute the Sagnac correction for two ground stations.
    ///
    /// The Sagnac effect due to Earth's rotation causes an asymmetry in the
    /// round-trip path. For equatorial stations separated by longitude delta_lambda:
    ///
    /// ```text
    /// delta_sagnac = 2 * omega_e * A / c^2
    /// ```
    ///
    /// where omega_e is Earth's rotation rate and A is the area of the
    /// Earth-station-satellite triangle projected onto the equatorial plane.
    ///
    /// * `lat_a_deg` - Latitude of station A in degrees.
    /// * `lon_a_deg` - Longitude of station A in degrees.
    /// * `lat_b_deg` - Latitude of station B in degrees.
    /// * `lon_b_deg` - Longitude of station B in degrees.
    ///
    /// Returns Sagnac correction in seconds.
    pub fn sagnac_correction(
        &self,
        lat_a_deg: f64,
        lon_a_deg: f64,
        lat_b_deg: f64,
        lon_b_deg: f64,
    ) -> f64 {
        let omega_e = 7.2921159e-5; // Earth rotation rate (rad/s)
        let r_s = self.earth_radius_m + self.satellite_altitude_m;

        let lat_a = lat_a_deg * PI / 180.0;
        let lon_a = lon_a_deg * PI / 180.0;
        let lat_b = lat_b_deg * PI / 180.0;
        let lon_b = lon_b_deg * PI / 180.0;

        // Approximate area using cross product in equatorial plane
        // For stations on the surface, the projected area contribution is:
        // A ~ r_s * R_e * cos(lat) * delta_lon / 2
        let delta_lon = lon_b - lon_a;
        let avg_cos_lat = (lat_a.cos() + lat_b.cos()) / 2.0;
        let area = r_s * self.earth_radius_m * avg_cos_lat * delta_lon.abs() / 2.0;

        2.0 * omega_e * area / (SPEED_OF_LIGHT * SPEED_OF_LIGHT)
    }

    /// Compute ionospheric delay at a given frequency.
    ///
    /// The ionospheric delay is proportional to TEC / f^2:
    /// ```text
    /// delta_t = 40.3 * TEC / (c * f^2)
    /// ```
    ///
    /// * `tec` - Total Electron Content in TECU (10^16 electrons/m^2).
    /// * `freq_hz` - Signal frequency in Hz.
    ///
    /// Returns delay in seconds.
    pub fn ionospheric_delay(tec: f64, freq_hz: f64) -> f64 {
        // TEC in electrons/m^2 = tec * 1e16
        // delay = 40.3 * TEC_electrons_m2 / (c * f^2)
        40.3 * tec * 1e16 / (SPEED_OF_LIGHT * freq_hz * freq_hz)
    }

    /// Compute the differential ionospheric delay for dual-frequency TWSTFT.
    ///
    /// Using two frequencies allows estimating and removing the ionospheric
    /// delay. Returns the TEC estimate from the dual-frequency measurement.
    ///
    /// * `delay_f1` - Measured delay at frequency f1 (seconds).
    /// * `delay_f2` - Measured delay at frequency f2 (seconds).
    /// * `f1_hz` - First frequency in Hz.
    /// * `f2_hz` - Second frequency in Hz.
    pub fn dual_frequency_tec(delay_f1: f64, delay_f2: f64, f1_hz: f64, f2_hz: f64) -> f64 {
        // delay1 - delay2 = 40.3 * TEC * 1e16 / c * (1/f1^2 - 1/f2^2)
        let delta_delay = delay_f1 - delay_f2;
        let freq_factor = 1.0 / (f1_hz * f1_hz) - 1.0 / (f2_hz * f2_hz);
        if freq_factor.abs() < 1e-30 {
            return 0.0;
        }
        delta_delay * SPEED_OF_LIGHT / (40.3 * 1e16 * freq_factor)
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Wrap a phase angle to the range [-pi, pi].
fn wrap_phase(phase: f64) -> f64 {
    let mut p = phase % (2.0 * PI);
    if p > PI {
        p -= 2.0 * PI;
    } else if p < -PI {
        p += 2.0 * PI;
    }
    p
}

/// Convert fractional frequency offset to parts-per-billion.
pub fn fractional_to_ppb(y: f64) -> f64 {
    y * 1e9
}

/// Convert parts-per-billion to fractional frequency offset.
pub fn ppb_to_fractional(ppb: f64) -> f64 {
    ppb * 1e-9
}

/// Convert fractional frequency offset to parts-per-trillion.
pub fn fractional_to_ppt(y: f64) -> f64 {
    y * 1e12
}

/// Compute the expected ADEV for a given noise type at a given tau.
///
/// * `noise_type` - The noise type.
/// * `h_coefficient` - The power spectral density coefficient.
/// * `f_h` - High-frequency cutoff (for WPM/FPM).
/// * `tau` - Averaging time.
pub fn theoretical_adev(noise_type: NoiseType, h_coefficient: f64, f_h: f64, tau: f64) -> f64 {
    match noise_type {
        NoiseType::WhitePhase => {
            // ADEV^2 = 3*f_h*h2 / (4*pi^2*tau^2)
            let var = 3.0 * f_h * h_coefficient / (4.0 * PI * PI * tau * tau);
            var.abs().sqrt()
        }
        NoiseType::FlickerPhase => {
            // ADEV^2 ~ h1 * [1.038 + 3*ln(2*pi*f_h*tau)] / (4*pi^2*tau^2)
            let inner = 1.038 + 3.0 * (2.0 * PI * f_h * tau).ln();
            let var = h_coefficient * inner / (4.0 * PI * PI * tau * tau);
            var.abs().sqrt()
        }
        NoiseType::WhiteFrequency => {
            // ADEV^2 = h0 / (2*tau)
            let var = h_coefficient / (2.0 * tau);
            var.abs().sqrt()
        }
        NoiseType::FlickerFrequency => {
            // ADEV^2 = 2*ln(2)*h_{-1}
            let var = 2.0 * 2.0_f64.ln() * h_coefficient;
            var.abs().sqrt()
        }
        NoiseType::RandomWalkFrequency => {
            // ADEV^2 = (2*pi^2/3)*h_{-2}*tau
            let var = 2.0 * PI * PI * h_coefficient * tau / 3.0;
            var.abs().sqrt()
        }
        NoiseType::FrequencyDrift => {
            // For linear drift d: ADEV = d*tau / sqrt(2)
            let var = h_coefficient * h_coefficient * tau * tau / 2.0;
            var.sqrt()
        }
    }
}

/// Compute phase noise spectral density from ADEV for white FM noise.
///
/// S_y(f) = h0 = 2 * tau * ADEV^2 (for white FM)
pub fn adev_to_phase_noise_white_fm(adev: f64, tau: f64) -> f64 {
    2.0 * tau * adev * adev
}

/// Compute time error (MTIE-like) from TDEV.
///
/// Maximum Time Interval Error approximation:
/// MTIE ~ 3 * TDEV for Gaussian noise processes.
pub fn tdev_to_mtie_approx(tdev: f64) -> f64 {
    3.0 * tdev
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -- AtomicClockConfig tests --

    #[test]
    fn test_cesium_config() {
        let config = AtomicClockConfig::new(ClockType::CesiumBeam);
        assert_eq!(config.nominal_frequency_hz, CESIUM_FREQUENCY_HZ);
        assert_eq!(config.clock_type, ClockType::CesiumBeam);
        assert!(config.adev_floor < 1e-13);
        assert_eq!(config.drift_rate_per_day, 0.0); // primary standard
    }

    #[test]
    fn test_rubidium_config() {
        let config = AtomicClockConfig::new(ClockType::RubidiumCell);
        assert_eq!(config.nominal_frequency_hz, RUBIDIUM_FREQUENCY_HZ);
        assert!(config.adev_floor > 1e-13); // worse than cesium
    }

    #[test]
    fn test_hydrogen_maser_config() {
        let config = AtomicClockConfig::new(ClockType::HydrogenMaser);
        assert_eq!(config.nominal_frequency_hz, HYDROGEN_MASER_FREQUENCY_HZ);
        assert!(config.adev_floor < config.accuracy); // better short-term
    }

    #[test]
    fn test_optical_clock_config() {
        let config = AtomicClockConfig::new(ClockType::OpticalClock);
        assert_eq!(config.nominal_frequency_hz, STRONTIUM_OPTICAL_FREQUENCY_HZ);
        assert!(config.adev_floor <= 1e-18);
    }

    #[test]
    fn test_custom_config() {
        let config = AtomicClockConfig::custom(
            ClockType::CesiumBeam,
            9_192_631_770.0,
            5e-14,
            5000.0,
            1e-14,
            0.0,
        );
        assert_eq!(config.adev_floor, 5e-14);
    }

    // -- AllanVarianceCalculator tests --

    #[test]
    fn test_adev_basic() {
        let mut calc = AllanVarianceCalculator::new(1.0);
        // Push frequency samples: alternating +1e-12 and -1e-12
        for i in 0..100 {
            let y = if i % 2 == 0 { 1e-12 } else { -1e-12 };
            calc.push_frequency(y);
        }
        let result = calc.compute_adev(1).unwrap();
        assert_eq!(result.tau, 1.0);
        assert!(result.adev > 0.0);
        assert!(result.count > 0);
    }

    #[test]
    fn test_adev_constant_frequency() {
        // Constant frequency should give ~0 ADEV
        let mut calc = AllanVarianceCalculator::new(1.0);
        for _ in 0..100 {
            calc.push_frequency(1e-10); // constant offset
        }
        let result = calc.compute_adev(1).unwrap();
        // ADEV should be very small (numerical noise only)
        assert!(result.adev < 1e-20);
    }

    #[test]
    fn test_adev_insufficient_data() {
        let mut calc = AllanVarianceCalculator::new(1.0);
        calc.push_frequency(1e-12);
        calc.push_frequency(-1e-12);
        // Need at least 2*m+1 = 3 phase samples for m=1, we have 3 (including x_0)
        let result = calc.compute_adev(1);
        assert!(result.is_some());

        // But m=2 needs 5 phase samples, we only have 3
        let result = calc.compute_adev(2);
        assert!(result.is_none());
    }

    #[test]
    fn test_adev_spectrum() {
        let mut calc = AllanVarianceCalculator::new(1.0);
        for i in 0..1000 {
            let y = 1e-12 * ((i as f64) * 0.01).sin();
            calc.push_frequency(y);
        }
        let spectrum = calc.compute_adev_spectrum(None);
        assert!(spectrum.len() >= 3); // at least m=1,2,4
        // Tau should double each step
        for i in 1..spectrum.len() {
            assert!((spectrum[i].tau / spectrum[i - 1].tau - 2.0).abs() < 0.01);
        }
    }

    #[test]
    fn test_adev_from_phase_data() {
        // Linear phase = constant frequency, ADEV should be ~0
        let phase_data: Vec<f64> = (0..100).map(|i| i as f64 * 1e-9).collect();
        let calc = AllanVarianceCalculator::from_phase_data(1.0, phase_data);
        let result = calc.compute_adev(1).unwrap();
        assert!(result.adev < 1e-20);
    }

    #[test]
    fn test_mdev_basic() {
        let mut calc = AllanVarianceCalculator::new(1.0);
        for i in 0..200 {
            let y = 1e-12 * ((i as f64) * 0.1).sin();
            calc.push_frequency(y);
        }
        let result = calc.compute_mdev(1).unwrap();
        assert!(result.tau == 1.0);
        assert!(result.mdev > 0.0);
    }

    #[test]
    fn test_mdev_insufficient_data() {
        let mut calc = AllanVarianceCalculator::new(1.0);
        calc.push_frequency(1e-12);
        calc.push_frequency(-1e-12);
        // MDEV m=1 needs 3*1+1=4 phase samples, we have 3
        assert!(calc.compute_mdev(1).is_none());
    }

    #[test]
    fn test_tdev_from_mdev() {
        let mut calc = AllanVarianceCalculator::new(1.0);
        for i in 0..200 {
            let y = 1e-12 * ((i as f64) * 0.05).sin();
            calc.push_frequency(y);
        }
        let mdev = calc.compute_mdev(2).unwrap();
        let tdev = calc.compute_tdev(2).unwrap();
        // TDEV = tau / sqrt(3) * MDEV
        let expected_tdev = mdev.tau / 3.0_f64.sqrt() * mdev.mdev;
        assert!((tdev.tdev - expected_tdev).abs() < 1e-30);
    }

    #[test]
    fn test_push_phase_batch() {
        let mut calc = AllanVarianceCalculator::new(1.0);
        let phases: Vec<f64> = (0..50).map(|i| i as f64 * 1e-9).collect();
        calc.push_phase_batch(&phases);
        assert_eq!(calc.num_phase_samples(), 51); // 1 initial + 50 pushed
    }

    // -- FrequencyStabilityAnalyzer tests --

    #[test]
    fn test_noise_type_white_fm() {
        let mut analyzer = FrequencyStabilityAnalyzer::new();
        // White FM: ADEV ~ tau^(-0.5)
        for &tau in &[1.0, 2.0, 4.0, 8.0, 16.0] {
            let adev = 1e-12 / (tau as f64).sqrt();
            analyzer.add_point(tau, adev);
        }
        let noise = analyzer.identify_noise_type().unwrap();
        assert_eq!(noise, NoiseType::WhiteFrequency);
    }

    #[test]
    fn test_noise_type_flicker_fm() {
        let mut analyzer = FrequencyStabilityAnalyzer::new();
        // Flicker FM: ADEV ~ tau^0 (constant)
        for &tau in &[1.0, 2.0, 4.0, 8.0, 16.0] {
            analyzer.add_point(tau, 1e-13);
        }
        let noise = analyzer.identify_noise_type().unwrap();
        assert_eq!(noise, NoiseType::FlickerFrequency);
    }

    #[test]
    fn test_noise_type_random_walk() {
        let mut analyzer = FrequencyStabilityAnalyzer::new();
        // Random walk FM: ADEV ~ tau^(+0.5)
        for &tau in &[1.0, 2.0, 4.0, 8.0, 16.0] {
            let adev = 1e-14 * (tau as f64).sqrt();
            analyzer.add_point(tau, adev);
        }
        let noise = analyzer.identify_noise_type().unwrap();
        assert_eq!(noise, NoiseType::RandomWalkFrequency);
    }

    #[test]
    fn test_noise_type_slope_values() {
        assert_eq!(NoiseType::WhitePhase.adev_slope(), -1.0);
        assert_eq!(NoiseType::WhiteFrequency.adev_slope(), -0.5);
        assert_eq!(NoiseType::FlickerFrequency.adev_slope(), 0.0);
        assert_eq!(NoiseType::RandomWalkFrequency.adev_slope(), 0.5);
        assert_eq!(NoiseType::FrequencyDrift.adev_slope(), 1.0);
    }

    #[test]
    fn test_analyzer_fit_slope() {
        let mut analyzer = FrequencyStabilityAnalyzer::new();
        // Perfect power law: adev = 1e-12 * tau^(-0.5)
        for &tau in &[1.0, 10.0, 100.0, 1000.0] {
            analyzer.add_point(tau, 1e-12 * tau.powf(-0.5));
        }
        let (slope, _) = analyzer.fit_slope().unwrap();
        assert!((slope - (-0.5)).abs() < 0.01);
    }

    #[test]
    fn test_analyzer_insufficient_points() {
        let mut analyzer = FrequencyStabilityAnalyzer::new();
        analyzer.add_point(1.0, 1e-12);
        assert!(analyzer.fit_slope().is_none());
    }

    // -- PhaseComparator tests --

    #[test]
    fn test_dmtd_multiplication_factor() {
        let dmtd = PhaseComparator::new(10e6, 10.0);
        assert_eq!(dmtd.multiplication_factor(), 1e6);
    }

    #[test]
    fn test_dmtd_resolution() {
        let dmtd = PhaseComparator::new(10e6, 10.0);
        // Resolution should be 1ns / 1e6 = 1 femtosecond
        assert!((dmtd.resolution_seconds() - 1e-15).abs() < 1e-20);
    }

    #[test]
    fn test_dmtd_zero_offset() {
        let mut dmtd = PhaseComparator::new(10e6, 10.0);
        // Equal offsets should give zero time difference
        let td = dmtd.measure(1e-12, 1e-12, 1.0);
        assert!(td.abs() < 1e-25);
    }

    #[test]
    fn test_dmtd_nonzero_offset() {
        let mut dmtd = PhaseComparator::new(10e6, 10.0);
        let td = dmtd.measure(0.0, 1e-12, 1.0);
        // DUT offset of 1e-12 * 10e6 Hz * 1s = 1e-5 cycles = td * f
        assert!(td.abs() > 0.0);
        // td = (dut_phase - ref_phase) / (2*pi*f) = 1e-12 * 1.0 = 1e-12 seconds
        assert!((td - 1e-12).abs() < 1e-15);
    }

    #[test]
    fn test_dmtd_measurement_accumulation() {
        let mut dmtd = PhaseComparator::new(10e6, 10.0);
        for _ in 0..10 {
            dmtd.measure(0.0, 1e-13, 1.0);
        }
        assert_eq!(dmtd.measurements().len(), 10);
        assert!(dmtd.mean_time_difference().is_some());
    }

    // -- PllFrequencyLock tests --

    #[test]
    fn test_pll_creation() {
        let pll = PllFrequencyLock::new(1.0, 0.707, 0.001);
        assert!(pll.natural_frequency() > 0.0);
        assert!((pll.damping_ratio() - 0.707).abs() < 0.01);
    }

    #[test]
    fn test_pll_lock_acquisition() {
        let mut pll = PllFrequencyLock::new(10.0, 0.707, 0.001);
        // Feed constant reference phase ramp (frequency offset)
        let freq_offset = 5.0; // Hz
        for i in 0..5000 {
            let t = i as f64 * 0.001;
            let ref_phase = 2.0 * PI * freq_offset * t;
            pll.update(ref_phase);
        }
        // After settling, PLL should be locked
        assert!(pll.is_locked());
        assert!(pll.phase_error().abs() < 0.2);
    }

    #[test]
    fn test_pll_reset() {
        let mut pll = PllFrequencyLock::new(1.0, 0.707, 0.001);
        pll.update(1.0);
        pll.reset();
        assert_eq!(pll.phase_error(), 0.0);
        assert_eq!(pll.frequency_correction(), 0.0);
        assert!(!pll.is_locked());
    }

    // -- CesiumBeamModel tests --

    #[test]
    fn test_cesium_on_resonance() {
        let model = CesiumBeamModel::new(0.005, 1.0);
        let prob = model.transition_probability(0.0);
        // On resonance: P = (1/2)(1 + 1.0 * cos(0)) * sinc^2(0) = 1.0
        assert!((prob - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_cesium_fringe_width() {
        let model = CesiumBeamModel::new(0.005, 0.95);
        let width = model.fringe_width_hz();
        // FWHM ~ 1/(2*T) = 1/(2*0.005) = 100 Hz
        assert!((width - 100.0).abs() < 0.01);
    }

    #[test]
    fn test_cesium_quality_factor() {
        let model = CesiumBeamModel::new(0.005, 0.95);
        let q = model.quality_factor();
        // Q = 9.192e9 / 100 ~ 9.2e7
        assert!(q > 9e7);
    }

    #[test]
    fn test_cesium_fringe_pattern_symmetry() {
        let model = CesiumBeamModel::new(0.005, 0.95);
        let p_plus = model.transition_probability(50.0);
        let p_minus = model.transition_probability(-50.0);
        assert!((p_plus - p_minus).abs() < 1e-10);
    }

    #[test]
    fn test_cesium_compute_fringes() {
        let model = CesiumBeamModel::new(0.005, 0.95);
        let fringes = model.compute_fringes(500.0, 101);
        assert_eq!(fringes.len(), 101);
        // Central point should be near max
        let center = fringes[50];
        assert!(center.1 > 0.9);
    }

    // -- RubidiumCellModel tests --

    #[test]
    fn test_rb_on_resonance() {
        let model = RubidiumCellModel::new(1000.0, 0.03);
        let signal = model.absorption_signal(0.0);
        // On resonance: Lorentzian peak = contrast
        assert!((signal - 0.03).abs() < 1e-10);
    }

    #[test]
    fn test_rb_off_resonance_rolloff() {
        let model = RubidiumCellModel::new(1000.0, 0.03);
        let half_width = model.absorption_signal(500.0);
        // At FWHM point: signal should be contrast/2
        assert!((half_width - 0.015).abs() < 1e-10);
    }

    #[test]
    fn test_rb_quality_factor() {
        let model = RubidiumCellModel::new(1000.0, 0.03);
        let q = model.quality_factor();
        // Q = 6.834e9 / 1000 ~ 6.8e6
        assert!(q > 6e6);
    }

    #[test]
    fn test_rb_buffer_gas_shift() {
        let mut model = RubidiumCellModel::new(1000.0, 0.03);
        model.set_buffer_gas(500.0, 0.1, 25.0);
        let f_eff_25 = model.effective_frequency(25.0);
        let f_eff_35 = model.effective_frequency(35.0);
        // 10 degree change * 0.1 Hz/K = 1 Hz shift
        assert!((f_eff_35 - f_eff_25 - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_rb_discriminator_zero_crossing() {
        let model = RubidiumCellModel::new(1000.0, 0.03);
        // Discriminator should be zero on resonance
        let d = model.discriminator_signal(0.0);
        assert!(d.abs() < 1e-15);
    }

    #[test]
    fn test_rb_discriminator_antisymmetric() {
        let model = RubidiumCellModel::new(1000.0, 0.03);
        let d_plus = model.discriminator_signal(100.0);
        let d_minus = model.discriminator_signal(-100.0);
        // Discriminator is odd function
        assert!((d_plus + d_minus).abs() < 1e-15);
    }

    // -- TimescaleAlgorithm tests --

    #[test]
    fn test_timescale_equal_weights() {
        let mut algo = TimescaleAlgorithm::new(0.5, 3.0);
        let clocks = vec![
            ClockInput { id: 0, phase: 1e-9, frequency_offset: 1e-13, weight: 1.0, valid: true },
            ClockInput { id: 1, phase: -1e-9, frequency_offset: -1e-13, weight: 1.0, valid: true },
        ];
        let result = algo.compute(&clocks);
        assert_eq!(result.num_valid, 2);
        // Average phase should be ~0
        assert!(result.ensemble_phase.abs() < 1e-15);
    }

    #[test]
    fn test_timescale_weight_capping() {
        // With 4 clocks and max_weight 0.4, the dominant clock should be capped
        let mut algo = TimescaleAlgorithm::new(0.4, 3.0);
        let clocks = vec![
            ClockInput { id: 0, phase: 0.0, frequency_offset: 0.0, weight: 100.0, valid: true },
            ClockInput { id: 1, phase: 1e-9, frequency_offset: 1e-13, weight: 1.0, valid: true },
            ClockInput { id: 2, phase: -1e-9, frequency_offset: -1e-13, weight: 1.0, valid: true },
            ClockInput { id: 3, phase: 0.5e-9, frequency_offset: 0.5e-13, weight: 1.0, valid: true },
        ];
        let result = algo.compute(&clocks);
        // Weight of the dominant clock 0 should be capped at max_weight_fraction
        assert!(result.weights[0] <= 0.401); // allow small numerical tolerance
        // All weights should sum to ~1.0
        let total: f64 = result.weights.iter().sum();
        assert!((total - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_timescale_invalid_clocks() {
        let mut algo = TimescaleAlgorithm::new(0.5, 3.0);
        let clocks = vec![
            ClockInput { id: 0, phase: 1e-9, frequency_offset: 0.0, weight: 1.0, valid: true },
            ClockInput { id: 1, phase: 0.0, frequency_offset: 0.0, weight: 1.0, valid: false },
        ];
        let result = algo.compute(&clocks);
        assert_eq!(result.num_valid, 1);
        assert!(result.weights[1] == 0.0); // invalid clock gets zero weight
    }

    #[test]
    fn test_timescale_no_valid_clocks() {
        let mut algo = TimescaleAlgorithm::new(0.5, 3.0);
        let clocks = vec![
            ClockInput { id: 0, phase: 0.0, frequency_offset: 0.0, weight: 1.0, valid: false },
        ];
        let result = algo.compute(&clocks);
        assert_eq!(result.num_valid, 0);
        assert_eq!(result.ensemble_phase, 0.0);
    }

    // -- LeapSecondHandler tests --

    #[test]
    fn test_leap_second_count() {
        let handler = LeapSecondHandler::new();
        assert_eq!(handler.num_events(), 27);
    }

    #[test]
    fn test_tai_utc_offset_2017() {
        let handler = LeapSecondHandler::new();
        // After 2016-12-31 leap second: TAI-UTC = 37
        let offset = handler.tai_utc_offset(2017, 1, 1);
        assert_eq!(offset, 37);
    }

    #[test]
    fn test_tai_utc_offset_1972() {
        let handler = LeapSecondHandler::new();
        // Before 1972-06-30: TAI-UTC = 10
        let offset = handler.tai_utc_offset(1972, 1, 1);
        assert_eq!(offset, 10);
        // After 1972-06-30: TAI-UTC = 11
        let offset = handler.tai_utc_offset(1972, 7, 1);
        assert_eq!(offset, 11);
    }

    #[test]
    fn test_gps_utc_offset() {
        let handler = LeapSecondHandler::new();
        // GPS-UTC in 2017 = 37 - 19 = 18
        assert_eq!(handler.gps_utc_offset(2017, 6, 1), 18);
    }

    #[test]
    fn test_utc_tai_roundtrip() {
        let handler = LeapSecondHandler::new();
        let utc = 1000000.0;
        let tai = handler.utc_to_tai(utc, 2020, 1, 1);
        let utc_back = handler.tai_to_utc(tai, 2020, 1, 1);
        assert!((utc - utc_back).abs() < 1e-15);
    }

    #[test]
    fn test_add_leap_event() {
        let mut handler = LeapSecondHandler::empty();
        handler.add_event(LeapSecondEvent {
            year: 2030,
            month: 6,
            day: 30,
            leap_type: LeapSecondType::Insertion,
        });
        assert_eq!(handler.num_events(), 1);
        assert_eq!(handler.tai_utc_offset(2030, 7, 1), 11);
    }

    // -- TwoWayTimeTransfer tests --

    #[test]
    fn test_twstft_one_way_delay() {
        let tw = TwoWayTimeTransfer::geo_default();
        // GEO at 90 deg elevation (directly overhead): ~35786 km / c ~ 0.119 s
        let delay = tw.one_way_delay(90.0);
        assert!((delay - 0.1194).abs() < 0.001);
    }

    #[test]
    fn test_twstft_low_elevation_longer() {
        let tw = TwoWayTimeTransfer::geo_default();
        let delay_high = tw.one_way_delay(60.0);
        let delay_low = tw.one_way_delay(10.0);
        assert!(delay_low > delay_high);
    }

    #[test]
    fn test_twstft_clock_offset_symmetric() {
        let tw = TwoWayTimeTransfer::geo_default();
        // If T_a = T_b, clock offset should be ~0 (no Sagnac)
        let offset = tw.compute_clock_offset(0.250, 0.250, 0.0);
        assert!(offset.abs() < 1e-15);
    }

    #[test]
    fn test_twstft_clock_offset_asymmetric() {
        let tw = TwoWayTimeTransfer::geo_default();
        // T_a - T_b = 2 ns means clock difference of 1 ns
        let offset = tw.compute_clock_offset(0.250_001, 0.249_999, 0.0);
        assert!((offset - 1e-6).abs() < 1e-12);
    }

    #[test]
    fn test_ionospheric_delay() {
        // TEC = 10 TECU at 14 GHz Ku-band
        let delay = TwoWayTimeTransfer::ionospheric_delay(10.0, 14e9);
        // Expected ~ 40.3 * 10 * 1e16 / (3e8 * (14e9)^2) ~ 6.85e-11 s
        assert!(delay > 1e-12);
        assert!(delay < 1e-9);
    }

    #[test]
    fn test_ionospheric_delay_frequency_dependence() {
        let delay_low = TwoWayTimeTransfer::ionospheric_delay(10.0, 10e9);
        let delay_high = TwoWayTimeTransfer::ionospheric_delay(10.0, 14e9);
        // Lower frequency = more delay (1/f^2 dependence)
        assert!(delay_low > delay_high);
        // Ratio should be (14/10)^2 = 1.96
        let ratio = delay_low / delay_high;
        assert!((ratio - (14.0_f64 / 10.0).powi(2)).abs() < 0.01);
    }

    #[test]
    fn test_sagnac_correction_same_location() {
        let tw = TwoWayTimeTransfer::geo_default();
        let sagnac = tw.sagnac_correction(40.0, -74.0, 40.0, -74.0);
        // Same location: Sagnac should be ~0
        assert!(sagnac.abs() < 1e-15);
    }

    #[test]
    fn test_sagnac_nonzero_for_separated_stations() {
        let tw = TwoWayTimeTransfer::geo_default();
        // Washington DC (39N, 77W) to Paris (49N, 2E)
        let sagnac = tw.sagnac_correction(39.0, -77.0, 49.0, 2.0);
        // Sagnac effect for widely separated stations should be ~100-300 ns
        assert!(sagnac.abs() > 1e-10);
    }

    #[test]
    fn test_dual_frequency_tec() {
        // Generate consistent delays for known TEC
        let tec = 15.0; // TECU
        let f1 = 14.0e9;
        let f2 = 11.0e9;
        let delay1 = TwoWayTimeTransfer::ionospheric_delay(tec, f1);
        let delay2 = TwoWayTimeTransfer::ionospheric_delay(tec, f2);
        let recovered_tec = TwoWayTimeTransfer::dual_frequency_tec(delay1, delay2, f1, f2);
        assert!((recovered_tec - tec).abs() < 0.01);
    }

    // -- Utility function tests --

    #[test]
    fn test_fractional_to_ppb() {
        assert_eq!(fractional_to_ppb(1e-9), 1.0);
        assert_eq!(fractional_to_ppb(5e-12), 0.005);
    }

    #[test]
    fn test_ppb_to_fractional() {
        assert_eq!(ppb_to_fractional(1.0), 1e-9);
    }

    #[test]
    fn test_fractional_to_ppt() {
        assert_eq!(fractional_to_ppt(1e-12), 1.0);
    }

    #[test]
    fn test_theoretical_adev_white_fm() {
        // White FM: ADEV = sqrt(h0 / (2*tau))
        let h0 = 1e-24;
        let tau = 1.0;
        let adev = theoretical_adev(NoiseType::WhiteFrequency, h0, 0.0, tau);
        let expected = (h0 / (2.0 * tau)).sqrt();
        assert!((adev - expected).abs() < 1e-20);
    }

    #[test]
    fn test_theoretical_adev_flicker_fm() {
        // Flicker FM: ADEV = sqrt(2*ln(2)*h_{-1}), independent of tau
        let h_minus1 = 1e-26;
        let adev_1 = theoretical_adev(NoiseType::FlickerFrequency, h_minus1, 0.0, 1.0);
        let adev_10 = theoretical_adev(NoiseType::FlickerFrequency, h_minus1, 0.0, 10.0);
        assert!((adev_1 - adev_10).abs() < 1e-20);
    }

    #[test]
    fn test_adev_to_phase_noise() {
        let adev = 1e-12;
        let tau = 1.0;
        let s_y = adev_to_phase_noise_white_fm(adev, tau);
        // S_y = 2*tau*adev^2 = 2 * 1 * 1e-24 = 2e-24
        assert!((s_y - 2e-24).abs() < 1e-35);
    }

    #[test]
    fn test_wrap_phase() {
        assert!((wrap_phase(0.0)).abs() < 1e-15);
        assert!((wrap_phase(PI) - PI).abs() < 1e-15);
        assert!((wrap_phase(3.0 * PI) - PI).abs() < 1e-10);
        assert!((wrap_phase(-3.0 * PI) + PI).abs() < 1e-10);
    }

    #[test]
    fn test_noise_type_psd_exponent() {
        assert_eq!(NoiseType::WhitePhase.psd_exponent(), 2);
        assert_eq!(NoiseType::FlickerPhase.psd_exponent(), 1);
        assert_eq!(NoiseType::WhiteFrequency.psd_exponent(), 0);
        assert_eq!(NoiseType::FlickerFrequency.psd_exponent(), -1);
        assert_eq!(NoiseType::RandomWalkFrequency.psd_exponent(), -2);
    }
}
