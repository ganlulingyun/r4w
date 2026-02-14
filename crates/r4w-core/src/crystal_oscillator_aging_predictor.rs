//! Kalman-filtered frequency drift prediction for temperature-compensated crystal oscillators.
//!
//! This module provides tools for modelling, measuring, and compensating the long-term
//! frequency drift of TCXOs and OCXOs. It combines classical aging models
//! (logarithmic and exponential), Allan deviation analysis, polynomial temperature
//! compensation, and a two-state Kalman filter to produce predictive frequency
//! steering corrections.
//!
//! # Background
//!
//! Crystal oscillators age over time: their resonant frequency drifts due to mass
//! transfer at the electrode-quartz interface, stress relaxation, and contamination.
//! For a TCXO the aging rate is typically 0.5 -- 2 ppm/year; for an OCXO it can
//! be as low as 0.5 ppb/day. The drift is usually monotonic and well-modelled by
//! a logarithmic or exponential curve.
//!
//! Temperature also affects the instantaneous frequency. Even "compensated"
//! oscillators have a residual frequency-vs-temperature characteristic that is
//! well-described by a cubic polynomial around the turnover temperature.
//!
//! By combining a physical aging model with Kalman-filtered real-time observations
//! we can predict and pre-correct the drift, maintaining the oscillator within a
//! tight tolerance window without costly disciplining hardware.
//!
//! # Components
//!
//! | Struct / Function | Purpose |
//! |---|---|
//! | [`OscillatorConfig`] | Nominal frequency, aging rate, temperature coefficients |
//! | [`AgingModel`] | Logarithmic / exponential / linear aging prediction |
//! | [`AllanDevianceCalculator`] | Overlapping Allan deviation (ADEV) computation |
//! | [`TemperatureCompensator`] | Polynomial TCXO compensation model |
//! | [`KalmanFrequencyTracker`] | 2-state Kalman filter for frequency + drift |
//! | [`FrequencySteering`] | Predictive corrections to maintain target frequency |
//! | [`frequency_offset_ppb`] | Convert absolute offset to ppb |
//! | [`phase_noise_to_jitter`] | Convert SSB phase noise (dBc/Hz) to RMS jitter |
//! | [`aging_rate_from_data`] | Least-squares aging rate estimate from frequency samples |
//!
//! # Example
//!
//! ```rust
//! use r4w_core::crystal_oscillator_aging_predictor::{
//!     OscillatorConfig, AgingModel, AgingModelType, KalmanFrequencyTracker,
//!     FrequencySteering, frequency_offset_ppb,
//! };
//!
//! // Configure a 10 MHz TCXO with 1 ppm/year logarithmic aging
//! let config = OscillatorConfig {
//!     nominal_freq_hz: 10_000_000.0,
//!     aging_rate_ppb_per_day: 2.74,  // ~1 ppm/year
//!     temp_coefficients: [0.0, 0.0, -0.035e-6, 0.0],  // cubic ppm/C^3 residual
//!     turnover_temp_c: 25.0,
//! };
//!
//! // Predict aging offset after 90 days
//! let model = AgingModel::new(AgingModelType::Logarithmic, config.aging_rate_ppb_per_day);
//! let offset_ppb = model.predict(90.0);
//! assert!(offset_ppb.abs() > 0.0);
//!
//! // Track with Kalman filter
//! let mut tracker = KalmanFrequencyTracker::new(config.nominal_freq_hz, 1.0);
//! for day in 0..30 {
//!     let measured = config.nominal_freq_hz + (day as f64) * 0.027;
//!     tracker.update(measured);
//! }
//! let (est_freq, est_drift) = tracker.state();
//! assert!(est_freq > config.nominal_freq_hz);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// OscillatorConfig
// ---------------------------------------------------------------------------

/// Configuration parameters for a crystal oscillator.
///
/// Captures the key electrical and environmental characteristics needed for
/// aging prediction and temperature compensation.
#[derive(Debug, Clone, PartialEq)]
pub struct OscillatorConfig {
    /// Nominal (nameplate) frequency in Hz, e.g. 10 000 000.0 for a 10 MHz unit.
    pub nominal_freq_hz: f64,

    /// Long-term aging rate in parts-per-billion per day.
    ///
    /// Positive means the frequency *increases* over time (most AT-cut crystals).
    /// Typical values: TCXO 0.5 -- 5 ppb/day, OCXO 0.01 -- 0.5 ppb/day.
    pub aging_rate_ppb_per_day: f64,

    /// Polynomial temperature coefficients `[a0, a1, a2, a3]` such that the
    /// fractional frequency offset is:
    ///
    /// ```text
    /// df/f = a0 + a1*(T-T0) + a2*(T-T0)^2 + a3*(T-T0)^3
    /// ```
    ///
    /// where `T0` is [`turnover_temp_c`](Self::turnover_temp_c). Units are
    /// dimensionless (i.e. 1e-6 means 1 ppm).
    pub temp_coefficients: [f64; 4],

    /// Turnover temperature in degrees Celsius. This is the temperature at
    /// which the frequency-temperature curve has zero slope (for AT-cut
    /// crystals, typically 20 -- 30 C).
    pub turnover_temp_c: f64,
}

impl OscillatorConfig {
    /// Create a default TCXO configuration at the given nominal frequency.
    ///
    /// Uses typical TCXO aging (1 ppm/year ~ 2.74 ppb/day) and a cubic
    /// temperature characteristic with turnover at 25 C.
    pub fn default_tcxo(nominal_freq_hz: f64) -> Self {
        Self {
            nominal_freq_hz,
            aging_rate_ppb_per_day: 2.74,
            temp_coefficients: [0.0, 0.0, -0.035e-6, 0.0],
            turnover_temp_c: 25.0,
        }
    }

    /// Create a default OCXO configuration at the given nominal frequency.
    ///
    /// Uses typical OCXO aging (5e-10/day ~ 0.5 ppb/day) and a tighter
    /// temperature curve.
    pub fn default_ocxo(nominal_freq_hz: f64) -> Self {
        Self {
            nominal_freq_hz,
            aging_rate_ppb_per_day: 0.5,
            temp_coefficients: [0.0, 0.0, -0.004e-6, 0.0],
            turnover_temp_c: 25.0,
        }
    }
}

// ---------------------------------------------------------------------------
// AgingModel
// ---------------------------------------------------------------------------

/// The mathematical form used to model long-term frequency aging.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AgingModelType {
    /// `offset(t) = A * ln(1 + B*t)` -- good for initial run-in period.
    Logarithmic,
    /// `offset(t) = A * (1 - exp(-t/tau))` -- approaches asymptote.
    Exponential,
    /// `offset(t) = rate * t` -- simplest constant-rate model.
    Linear,
}

/// Aging model that predicts cumulative frequency offset (in ppb) as a
/// function of elapsed time (in days).
///
/// # Models
///
/// - **Logarithmic**: `A * ln(1 + B*t)` where `B = 0.1` (shape factor) and
///   `A` is scaled from the specified daily rate.
/// - **Exponential**: `A * (1 - exp(-t/tau))` with `tau = 365` days.
/// - **Linear**: `rate * t`.
#[derive(Debug, Clone)]
pub struct AgingModel {
    model_type: AgingModelType,
    /// Rate parameter in ppb/day.
    rate_ppb_per_day: f64,
}

impl AgingModel {
    /// Create a new aging model.
    ///
    /// `rate_ppb_per_day` is the nominal daily aging rate. The model type
    /// determines the curve shape.
    pub fn new(model_type: AgingModelType, rate_ppb_per_day: f64) -> Self {
        Self {
            model_type,
            rate_ppb_per_day,
        }
    }

    /// Predict cumulative frequency offset in ppb after `days` elapsed.
    ///
    /// Returns the total integrated drift, not the instantaneous rate.
    pub fn predict(&self, days: f64) -> f64 {
        match self.model_type {
            AgingModelType::Logarithmic => {
                // A * ln(1 + B*t),  choose B = 0.1 day^-1
                // At t=1 day: A * ln(1.1) ~ A * 0.0953
                // We want the *average* rate over the first day to equal rate_ppb_per_day,
                // so A = rate_ppb_per_day / ln(1 + B).
                let b = 0.1_f64;
                let a = self.rate_ppb_per_day / (1.0 + b).ln();
                a * (1.0 + b * days).ln()
            }
            AgingModelType::Exponential => {
                // A * (1 - exp(-t/tau)),  tau = 365 days
                // derivative at t=0 is A/tau = rate_ppb_per_day  =>  A = rate * tau
                let tau = 365.0_f64;
                let a = self.rate_ppb_per_day * tau;
                a * (1.0 - (-days / tau).exp())
            }
            AgingModelType::Linear => self.rate_ppb_per_day * days,
        }
    }

    /// Return the instantaneous aging rate (ppb/day) at the given elapsed time.
    pub fn instantaneous_rate(&self, days: f64) -> f64 {
        match self.model_type {
            AgingModelType::Logarithmic => {
                let b = 0.1_f64;
                let a = self.rate_ppb_per_day / (1.0 + b).ln();
                a * b / (1.0 + b * days)
            }
            AgingModelType::Exponential => {
                let tau = 365.0_f64;
                let a = self.rate_ppb_per_day * tau;
                (a / tau) * (-days / tau).exp()
            }
            AgingModelType::Linear => self.rate_ppb_per_day,
        }
    }

    /// Return the model type.
    pub fn model_type(&self) -> AgingModelType {
        self.model_type
    }

    /// Return the base rate (ppb/day).
    pub fn rate(&self) -> f64 {
        self.rate_ppb_per_day
    }
}

// ---------------------------------------------------------------------------
// AllanDevianceCalculator
// ---------------------------------------------------------------------------

/// Computes overlapping Allan deviation (ADEV) from frequency measurement data.
///
/// Allan deviation is the standard metric for characterising oscillator
/// short-term stability. It is defined as:
///
/// ```text
/// ADEV(tau) = sqrt( 1/(2*(N-2m)) * sum_{i=0}^{N-2m-1} (y_{i+2m} - 2*y_{i+m} + y_i)^2 )
/// ```
///
/// where `y_i` are fractional frequency samples, `tau = m * tau_0`, and
/// `tau_0` is the base sampling interval.
#[derive(Debug, Clone)]
pub struct AllanDevianceCalculator {
    /// Base sampling interval in seconds (tau_0).
    tau0: f64,
    /// Collected fractional-frequency samples.
    samples: Vec<f64>,
}

/// A single Allan deviation result at a particular averaging time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AllanDeviationPoint {
    /// Averaging time tau in seconds.
    pub tau: f64,
    /// Allan deviation (dimensionless fractional frequency).
    pub adev: f64,
    /// Number of overlapping samples used.
    pub num_samples: usize,
}

impl AllanDevianceCalculator {
    /// Create a new calculator with the given base sample interval (seconds).
    pub fn new(tau0: f64) -> Self {
        assert!(tau0 > 0.0, "tau0 must be positive");
        Self {
            tau0,
            samples: Vec::new(),
        }
    }

    /// Add a fractional-frequency sample `y_i = (f_i - f_nom) / f_nom`.
    pub fn push(&mut self, y: f64) {
        self.samples.push(y);
    }

    /// Add a batch of fractional-frequency samples.
    pub fn push_batch(&mut self, ys: &[f64]) {
        self.samples.extend_from_slice(ys);
    }

    /// Compute overlapping Allan deviation for averaging factor `m` (tau = m * tau0).
    ///
    /// Returns `None` if there are insufficient samples (need at least `2*m + 1`).
    pub fn compute_adev(&self, m: usize) -> Option<AllanDeviationPoint> {
        let n = self.samples.len();
        if m == 0 || n < 2 * m + 1 {
            return None;
        }

        let mut sum = 0.0_f64;
        let count = n - 2 * m;
        for i in 0..count {
            let diff = self.samples[i + 2 * m] - 2.0 * self.samples[i + m] + self.samples[i];
            sum += diff * diff;
        }

        let variance = sum / (2.0 * count as f64);
        let adev = variance.sqrt();
        let tau = m as f64 * self.tau0;

        Some(AllanDeviationPoint {
            tau,
            adev,
            num_samples: count,
        })
    }

    /// Compute ADEV across a range of averaging factors, using octave spacing
    /// (m = 1, 2, 4, 8, ...) up to the maximum supported by the data.
    ///
    /// Returns a vector of [`AllanDeviationPoint`]s sorted by ascending tau.
    pub fn compute_octave_adevs(&self) -> Vec<AllanDeviationPoint> {
        let n = self.samples.len();
        let mut results = Vec::new();
        let mut m = 1usize;
        while 2 * m + 1 <= n {
            if let Some(pt) = self.compute_adev(m) {
                results.push(pt);
            }
            m *= 2;
        }
        results
    }

    /// Compute ADEV for every integer averaging factor from 1 to `max_m`.
    pub fn compute_all_adevs(&self, max_m: usize) -> Vec<AllanDeviationPoint> {
        let n = self.samples.len();
        let limit = max_m.min((n - 1) / 2);
        let mut results = Vec::new();
        for m in 1..=limit {
            if let Some(pt) = self.compute_adev(m) {
                results.push(pt);
            }
        }
        results
    }

    /// Return the number of stored samples.
    pub fn len(&self) -> usize {
        self.samples.len()
    }

    /// Return whether no samples have been collected.
    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    /// Return the base sampling interval.
    pub fn tau0(&self) -> f64 {
        self.tau0
    }

    /// Clear all stored samples.
    pub fn clear(&mut self) {
        self.samples.clear();
    }
}

// ---------------------------------------------------------------------------
// TemperatureCompensator
// ---------------------------------------------------------------------------

/// Polynomial temperature compensation model for crystal oscillators.
///
/// Models the fractional frequency offset as a function of temperature:
///
/// ```text
/// df/f = a0 + a1*(T - T0) + a2*(T - T0)^2 + a3*(T - T0)^3
/// ```
///
/// This is the standard cubic model for AT-cut crystals (the dominant
/// mode in TCXOs and OCXOs). The compensator can compute a correction
/// value to apply to measured frequencies to remove the temperature effect.
#[derive(Debug, Clone)]
pub struct TemperatureCompensator {
    /// Polynomial coefficients `[a0, a1, a2, a3]` (dimensionless fractional).
    coefficients: [f64; 4],
    /// Reference (turnover) temperature in Celsius.
    reference_temp_c: f64,
}

impl TemperatureCompensator {
    /// Create a new compensator.
    ///
    /// * `coefficients` -- `[a0, a1, a2, a3]` for the cubic polynomial
    /// * `reference_temp_c` -- turnover / reference temperature (Celsius)
    pub fn new(coefficients: [f64; 4], reference_temp_c: f64) -> Self {
        Self {
            coefficients,
            reference_temp_c,
        }
    }

    /// Create a compensator directly from an [`OscillatorConfig`].
    pub fn from_config(config: &OscillatorConfig) -> Self {
        Self::new(config.temp_coefficients, config.turnover_temp_c)
    }

    /// Compute the fractional frequency offset (dimensionless) at temperature `temp_c`.
    pub fn offset_at(&self, temp_c: f64) -> f64 {
        let dt = temp_c - self.reference_temp_c;
        let [a0, a1, a2, a3] = self.coefficients;
        a0 + a1 * dt + a2 * dt * dt + a3 * dt * dt * dt
    }

    /// Compute the correction factor to *remove* the temperature-induced offset.
    ///
    /// Multiply the measured frequency by `(1.0 - correction)` to obtain the
    /// temperature-corrected frequency.
    pub fn correction_at(&self, temp_c: f64) -> f64 {
        -self.offset_at(temp_c)
    }

    /// Apply temperature compensation to a measured frequency.
    ///
    /// Returns the corrected frequency: `f_meas * (1.0 + correction)`.
    pub fn compensate(&self, measured_freq_hz: f64, temp_c: f64) -> f64 {
        let correction = self.correction_at(temp_c);
        measured_freq_hz * (1.0 + correction)
    }

    /// Return the polynomial coefficients.
    pub fn coefficients(&self) -> &[f64; 4] {
        &self.coefficients
    }

    /// Return the reference temperature.
    pub fn reference_temp_c(&self) -> f64 {
        self.reference_temp_c
    }

    /// Update the polynomial coefficients (e.g. after a calibration run).
    pub fn set_coefficients(&mut self, coefficients: [f64; 4]) {
        self.coefficients = coefficients;
    }
}

// ---------------------------------------------------------------------------
// KalmanFrequencyTracker
// ---------------------------------------------------------------------------

/// Two-state Kalman filter for tracking oscillator frequency and drift rate.
///
/// The state vector is `[frequency_offset_hz, drift_rate_hz_per_step]`.
/// The measurement is the observed frequency offset from nominal.
///
/// # State-space model
///
/// ```text
/// State transition:
///   x[k] = F * x[k-1] + w,    w ~ N(0, Q)
///
///   F = | 1  dt |     Q = | q1  0  |
///       | 0  1  |         |  0  q2 |
///
/// Measurement:
///   z[k] = H * x[k] + v,  v ~ N(0, R)
///   H = [1, 0]
/// ```
///
/// `dt` is the measurement interval (in the same time units as the drift rate).
#[derive(Debug, Clone)]
pub struct KalmanFrequencyTracker {
    /// Nominal frequency (Hz) -- subtracted from measurements.
    nominal_freq_hz: f64,
    /// Time step between updates (e.g. 1.0 for daily measurements in days).
    dt: f64,
    /// State estimate: `[frequency_offset, drift_rate]`.
    x: [f64; 2],
    /// Error covariance (2x2 symmetric, stored row-major: [p00, p01, p10, p11]).
    p: [f64; 4],
    /// Process noise covariance diagonal `[q_freq, q_drift]`.
    q: [f64; 2],
    /// Measurement noise variance.
    r: f64,
    /// Number of updates processed.
    update_count: u64,
}

impl KalmanFrequencyTracker {
    /// Create a new tracker.
    ///
    /// * `nominal_freq_hz` -- the target (nameplate) frequency
    /// * `dt` -- measurement interval (same time unit as drift rate)
    ///
    /// Uses sensible default noise parameters. Tune with
    /// [`set_process_noise`] and [`set_measurement_noise`].
    pub fn new(nominal_freq_hz: f64, dt: f64) -> Self {
        Self {
            nominal_freq_hz,
            dt,
            x: [0.0, 0.0],
            // Initial covariance: high uncertainty
            p: [1e6, 0.0, 0.0, 1e3],
            // Process noise: small
            q: [1e-2, 1e-6],
            // Measurement noise
            r: 1.0,
            update_count: 0,
        }
    }

    /// Set process noise variances `[q_freq, q_drift]`.
    pub fn set_process_noise(&mut self, q_freq: f64, q_drift: f64) {
        self.q = [q_freq, q_drift];
    }

    /// Set measurement noise variance.
    pub fn set_measurement_noise(&mut self, r: f64) {
        self.r = r;
    }

    /// Predict step: propagate state and covariance forward by one time step.
    ///
    /// Usually called internally by [`update`](Self::update), but exposed for
    /// manual predict-only operation.
    pub fn predict(&mut self) {
        let dt = self.dt;
        // x_pred = F * x
        let x0 = self.x[0] + dt * self.x[1];
        let x1 = self.x[1];
        self.x = [x0, x1];

        // P_pred = F * P * F^T + Q
        let [p00, p01, p10, p11] = self.p;
        let fp00 = p00 + dt * p10 + dt * (p01 + dt * p11);
        let fp01 = p01 + dt * p11;
        let fp10 = p10 + dt * p11;
        let fp11 = p11;

        self.p = [
            fp00 + self.q[0],
            fp01,
            fp10,
            fp11 + self.q[1],
        ];
    }

    /// Update step: incorporate a new frequency measurement (in Hz).
    ///
    /// Internally calls [`predict`](Self::predict) first, then applies the
    /// Kalman correction.
    pub fn update(&mut self, measured_freq_hz: f64) {
        // Predict
        self.predict();

        // Innovation: z - H*x,  H = [1, 0]
        let z = measured_freq_hz - self.nominal_freq_hz;
        let innovation = z - self.x[0];

        // Innovation covariance: S = H*P*H' + R = P[0,0] + R
        let s = self.p[0] + self.r;

        if s.abs() < 1e-30 {
            // Degenerate -- skip update
            self.update_count += 1;
            return;
        }

        // Kalman gain: K = P*H' / S = [P[0,0], P[1,0]]' / S
        let k0 = self.p[0] / s;
        let k1 = self.p[2] / s;

        // State update
        self.x[0] += k0 * innovation;
        self.x[1] += k1 * innovation;

        // Covariance update: P = (I - K*H) * P
        let [p00, p01, p10, p11] = self.p;
        self.p = [
            (1.0 - k0) * p00,
            (1.0 - k0) * p01,
            p10 - k1 * p00,
            p11 - k1 * p01,
        ];

        self.update_count += 1;
    }

    /// Return the current state estimate `(frequency_offset_hz, drift_rate_hz_per_step)`.
    ///
    /// The frequency offset is relative to [`nominal_freq_hz`]; the drift rate
    /// is in Hz per time step.
    pub fn state(&self) -> (f64, f64) {
        (self.nominal_freq_hz + self.x[0], self.x[1])
    }

    /// Return the estimated frequency offset from nominal (Hz).
    pub fn frequency_offset(&self) -> f64 {
        self.x[0]
    }

    /// Return the estimated drift rate (Hz per time step).
    pub fn drift_rate(&self) -> f64 {
        self.x[1]
    }

    /// Return the diagonal of the error covariance `(var_freq, var_drift)`.
    pub fn covariance_diagonal(&self) -> (f64, f64) {
        (self.p[0], self.p[3])
    }

    /// Return the number of updates processed.
    pub fn update_count(&self) -> u64 {
        self.update_count
    }

    /// Predict the frequency at `steps_ahead` time steps into the future
    /// (without updating state).
    pub fn predict_frequency(&self, steps_ahead: f64) -> f64 {
        let offset = self.x[0] + self.dt * steps_ahead * self.x[1];
        self.nominal_freq_hz + offset
    }

    /// Reset the filter to its initial state.
    pub fn reset(&mut self) {
        self.x = [0.0, 0.0];
        self.p = [1e6, 0.0, 0.0, 1e3];
        self.update_count = 0;
    }
}

// ---------------------------------------------------------------------------
// FrequencySteering
// ---------------------------------------------------------------------------

/// Predictive frequency steering controller.
///
/// Combines an [`AgingModel`], [`TemperatureCompensator`], and
/// [`KalmanFrequencyTracker`] to compute the total correction (in Hz) that
/// should be applied to a programmable oscillator (e.g. via a DAC-controlled
/// varactor) to keep it on its nominal frequency.
///
/// The correction has three components:
/// 1. **Aging correction** -- predicted long-term drift from the aging model.
/// 2. **Temperature correction** -- instantaneous polynomial compensation.
/// 3. **Kalman residual** -- any remaining offset detected by the tracker.
#[derive(Debug, Clone)]
pub struct FrequencySteering {
    config: OscillatorConfig,
    aging_model: AgingModel,
    temp_compensator: TemperatureCompensator,
    tracker: KalmanFrequencyTracker,
    /// Elapsed time in days since power-on / calibration.
    elapsed_days: f64,
    /// Time step per update in days.
    step_days: f64,
}

impl FrequencySteering {
    /// Create a new steering controller.
    ///
    /// * `config` -- oscillator parameters
    /// * `aging_type` -- which aging curve to use
    /// * `step_days` -- time between updates (in days); e.g. 1.0 for daily
    pub fn new(config: OscillatorConfig, aging_type: AgingModelType, step_days: f64) -> Self {
        let aging_model = AgingModel::new(aging_type, config.aging_rate_ppb_per_day);
        let temp_compensator = TemperatureCompensator::from_config(&config);
        let tracker = KalmanFrequencyTracker::new(config.nominal_freq_hz, step_days);

        Self {
            config,
            aging_model,
            temp_compensator,
            tracker,
            elapsed_days: 0.0,
            step_days,
        }
    }

    /// Process a new measurement and return the total steering correction (Hz).
    ///
    /// * `measured_freq_hz` -- observed oscillator frequency
    /// * `temp_c` -- current ambient temperature (Celsius)
    ///
    /// The returned value should be *subtracted* from the oscillator control
    /// input to bring it back to nominal.
    pub fn update(&mut self, measured_freq_hz: f64, temp_c: f64) -> f64 {
        self.elapsed_days += self.step_days;

        // Temperature-correct the measurement before feeding the Kalman filter
        let corrected = self.temp_compensator.compensate(measured_freq_hz, temp_c);
        self.tracker.update(corrected);

        self.total_correction(temp_c)
    }

    /// Compute the total predicted correction (Hz) at the current time and
    /// temperature *without* consuming a measurement. Useful for open-loop
    /// prediction.
    pub fn total_correction(&self, temp_c: f64) -> f64 {
        let aging_ppb = self.aging_model.predict(self.elapsed_days);
        let aging_hz = aging_ppb * 1e-9 * self.config.nominal_freq_hz;

        let temp_offset = self.temp_compensator.offset_at(temp_c);
        let temp_hz = temp_offset * self.config.nominal_freq_hz;

        let kalman_offset_hz = self.tracker.frequency_offset();

        aging_hz + temp_hz + kalman_offset_hz
    }

    /// Return the current elapsed time (days).
    pub fn elapsed_days(&self) -> f64 {
        self.elapsed_days
    }

    /// Return a reference to the internal Kalman tracker.
    pub fn tracker(&self) -> &KalmanFrequencyTracker {
        &self.tracker
    }

    /// Return a mutable reference to the internal Kalman tracker.
    pub fn tracker_mut(&mut self) -> &mut KalmanFrequencyTracker {
        &mut self.tracker
    }

    /// Return a reference to the aging model.
    pub fn aging_model(&self) -> &AgingModel {
        &self.aging_model
    }

    /// Return a reference to the temperature compensator.
    pub fn temp_compensator(&self) -> &TemperatureCompensator {
        &self.temp_compensator
    }

    /// Reset the controller (elapsed time, Kalman state).
    pub fn reset(&mut self) {
        self.elapsed_days = 0.0;
        self.tracker.reset();
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Convert an absolute frequency offset to parts-per-billion.
///
/// ```text
/// ppb = (measured - nominal) / nominal * 1e9
/// ```
pub fn frequency_offset_ppb(nominal_hz: f64, measured_hz: f64) -> f64 {
    (measured_hz - nominal_hz) / nominal_hz * 1e9
}

/// Convert single-sideband phase noise spectral density (dBc/Hz) at a given
/// offset frequency to RMS time jitter (seconds).
///
/// Uses the approximation for white phase noise dominated by a single offset:
///
/// ```text
/// jitter_rms = sqrt(2 * 10^(L/10)) / (2 * pi * f_carrier)
/// ```
///
/// where `L` is the SSB phase noise in dBc/Hz and `f_carrier` is the
/// oscillator frequency.
///
/// For a more accurate result over a bandwidth, integrate the phase noise
/// profile; this function gives a quick single-point estimate.
pub fn phase_noise_to_jitter(phase_noise_dbc_per_hz: f64, carrier_freq_hz: f64) -> f64 {
    let power_linear = 2.0 * 10.0_f64.powf(phase_noise_dbc_per_hz / 10.0);
    let rms_radians = power_linear.abs().sqrt();
    rms_radians / (2.0 * PI * carrier_freq_hz)
}

/// Estimate the linear aging rate (ppb/day) from a series of frequency
/// measurements using least-squares fitting.
///
/// * `frequencies_hz` -- measured frequencies, one per time step
/// * `nominal_hz` -- nominal frequency
/// * `step_days` -- time between consecutive measurements (days)
///
/// Returns the best-fit linear slope in ppb/day, or `None` if fewer than
/// 2 data points are provided.
pub fn aging_rate_from_data(
    frequencies_hz: &[f64],
    nominal_hz: f64,
    step_days: f64,
) -> Option<f64> {
    let n = frequencies_hz.len();
    if n < 2 {
        return None;
    }

    // Convert to fractional ppb offsets
    let nf = n as f64;
    let mut sum_t = 0.0_f64;
    let mut sum_y = 0.0_f64;
    let mut sum_tt = 0.0_f64;
    let mut sum_ty = 0.0_f64;

    for (i, &f) in frequencies_hz.iter().enumerate() {
        let t = i as f64 * step_days;
        let y = (f - nominal_hz) / nominal_hz * 1e9; // ppb
        sum_t += t;
        sum_y += y;
        sum_tt += t * t;
        sum_ty += t * y;
    }

    let denom = nf * sum_tt - sum_t * sum_t;
    if denom.abs() < 1e-30 {
        return None;
    }

    let slope = (nf * sum_ty - sum_t * sum_y) / denom;
    Some(slope)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const NOMINAL_10MHZ: f64 = 10_000_000.0;

    // --- OscillatorConfig tests ---

    #[test]
    fn test_default_tcxo_config() {
        let cfg = OscillatorConfig::default_tcxo(NOMINAL_10MHZ);
        assert_eq!(cfg.nominal_freq_hz, NOMINAL_10MHZ);
        assert!((cfg.aging_rate_ppb_per_day - 2.74).abs() < 1e-10);
        assert_eq!(cfg.turnover_temp_c, 25.0);
    }

    #[test]
    fn test_default_ocxo_config() {
        let cfg = OscillatorConfig::default_ocxo(NOMINAL_10MHZ);
        assert_eq!(cfg.nominal_freq_hz, NOMINAL_10MHZ);
        assert!((cfg.aging_rate_ppb_per_day - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_config_clone_eq() {
        let cfg1 = OscillatorConfig::default_tcxo(NOMINAL_10MHZ);
        let cfg2 = cfg1.clone();
        assert_eq!(cfg1, cfg2);
    }

    // --- AgingModel tests ---

    #[test]
    fn test_linear_aging_zero_at_origin() {
        let model = AgingModel::new(AgingModelType::Linear, 2.0);
        assert!((model.predict(0.0)).abs() < 1e-15);
    }

    #[test]
    fn test_linear_aging_proportional() {
        let model = AgingModel::new(AgingModelType::Linear, 3.0);
        assert!((model.predict(10.0) - 30.0).abs() < 1e-10);
        assert!((model.predict(100.0) - 300.0).abs() < 1e-10);
    }

    #[test]
    fn test_logarithmic_aging_monotonic() {
        let model = AgingModel::new(AgingModelType::Logarithmic, 2.74);
        let mut prev = model.predict(0.0);
        for day in 1..=365 {
            let cur = model.predict(day as f64);
            assert!(cur > prev, "logarithmic aging should be monotonically increasing");
            prev = cur;
        }
    }

    #[test]
    fn test_logarithmic_aging_decelerates() {
        let model = AgingModel::new(AgingModelType::Logarithmic, 2.74);
        let rate_day1 = model.instantaneous_rate(1.0);
        let rate_day100 = model.instantaneous_rate(100.0);
        assert!(rate_day1 > rate_day100, "logarithmic aging rate should decrease over time");
    }

    #[test]
    fn test_exponential_aging_approaches_asymptote() {
        let model = AgingModel::new(AgingModelType::Exponential, 2.0);
        let at_1yr = model.predict(365.0);
        let at_10yr = model.predict(3650.0);
        // After 10 years the exponential should be close to saturation
        // Asymptote = rate * tau = 2.0 * 365 = 730 ppb
        let asymptote = 2.0 * 365.0;
        assert!((at_10yr - asymptote).abs() < 1.0, "10-year value should be near asymptote");
        assert!(at_10yr > at_1yr);
    }

    #[test]
    fn test_exponential_initial_rate() {
        let rate = 5.0;
        let model = AgingModel::new(AgingModelType::Exponential, rate);
        let inst = model.instantaneous_rate(0.0);
        assert!((inst - rate).abs() < 1e-10, "initial exponential rate should match parameter");
    }

    #[test]
    fn test_aging_model_accessors() {
        let model = AgingModel::new(AgingModelType::Linear, 1.5);
        assert_eq!(model.model_type(), AgingModelType::Linear);
        assert!((model.rate() - 1.5).abs() < 1e-15);
    }

    // --- AllanDevianceCalculator tests ---

    #[test]
    fn test_adev_constant_frequency() {
        // Constant frequency => zero ADEV
        let mut calc = AllanDevianceCalculator::new(1.0);
        for _ in 0..100 {
            calc.push(0.0);
        }
        let pt = calc.compute_adev(1).unwrap();
        assert!(pt.adev < 1e-15, "ADEV of constant frequency should be ~zero");
    }

    #[test]
    fn test_adev_insufficient_samples() {
        let mut calc = AllanDevianceCalculator::new(1.0);
        calc.push(0.0);
        calc.push(0.0);
        // m=1 needs 2*1+1 = 3 samples, we have 2
        assert!(calc.compute_adev(1).is_none());
    }

    #[test]
    fn test_adev_linear_drift() {
        // Linear drift y_i = a*i should give constant ADEV(m=1) = 0
        // because second differences of a linear sequence are zero.
        let mut calc = AllanDevianceCalculator::new(1.0);
        for i in 0..50 {
            calc.push(1e-9 * i as f64);
        }
        let pt = calc.compute_adev(1).unwrap();
        assert!(pt.adev < 1e-20, "ADEV of linear drift should be zero");
    }

    #[test]
    fn test_adev_octave_coverage() {
        let mut calc = AllanDevianceCalculator::new(1.0);
        for i in 0..128 {
            calc.push(1e-9 * (i as f64).sin());
        }
        let octaves = calc.compute_octave_adevs();
        // We should get m = 1, 2, 4, 8, 16, 32 (63 < 128/2)
        assert!(octaves.len() >= 5, "should have at least 5 octave points");
        // Taus should be powers of 2
        for (i, pt) in octaves.iter().enumerate() {
            let expected_tau = (1usize << i) as f64;
            assert!((pt.tau - expected_tau).abs() < 1e-10);
        }
    }

    #[test]
    fn test_adev_push_batch() {
        let mut calc = AllanDevianceCalculator::new(1.0);
        let data: Vec<f64> = (0..20).map(|i| 1e-10 * (i as f64)).collect();
        calc.push_batch(&data);
        assert_eq!(calc.len(), 20);
        assert!(!calc.is_empty());
    }

    #[test]
    fn test_adev_clear() {
        let mut calc = AllanDevianceCalculator::new(1.0);
        calc.push(1.0);
        calc.push(2.0);
        calc.clear();
        assert!(calc.is_empty());
        assert_eq!(calc.len(), 0);
    }

    #[test]
    fn test_adev_tau0_accessor() {
        let calc = AllanDevianceCalculator::new(0.5);
        assert!((calc.tau0() - 0.5).abs() < 1e-15);
    }

    // --- TemperatureCompensator tests ---

    #[test]
    fn test_temp_compensator_at_reference() {
        let comp = TemperatureCompensator::new([0.0, 0.0, -0.035e-6, 0.0], 25.0);
        // At reference temperature, offset should be zero (a0=0)
        assert!(comp.offset_at(25.0).abs() < 1e-20);
    }

    #[test]
    fn test_temp_compensator_cubic_symmetry() {
        // With only a2 (quadratic) coefficient, offset should be symmetric
        let comp = TemperatureCompensator::new([0.0, 0.0, -0.035e-6, 0.0], 25.0);
        let off_plus10 = comp.offset_at(35.0);
        let off_minus10 = comp.offset_at(15.0);
        assert!((off_plus10 - off_minus10).abs() < 1e-20, "quadratic should be symmetric");
    }

    #[test]
    fn test_temp_compensator_correction_negates_offset() {
        let comp = TemperatureCompensator::new([1e-6, 0.0, 0.0, 0.0], 25.0);
        let offset = comp.offset_at(30.0);
        let correction = comp.correction_at(30.0);
        assert!((offset + correction).abs() < 1e-20);
    }

    #[test]
    fn test_temp_compensator_compensate_frequency() {
        let comp = TemperatureCompensator::new([0.0, 1e-6, 0.0, 0.0], 25.0);
        let nominal = 10_000_000.0;
        // At 26 C (dT=1), offset = 1e-6 * 1 = 1e-6 => measured is high by 10 Hz
        let measured = nominal * (1.0 + 1e-6);
        let corrected = comp.compensate(measured, 26.0);
        // correction = -1e-6, so corrected ~ measured * (1 - 1e-6) ~ nominal
        assert!((corrected - nominal).abs() < 0.02);
    }

    #[test]
    fn test_temp_compensator_from_config() {
        let cfg = OscillatorConfig::default_tcxo(NOMINAL_10MHZ);
        let comp = TemperatureCompensator::from_config(&cfg);
        assert_eq!(comp.reference_temp_c(), 25.0);
        assert_eq!(comp.coefficients(), &cfg.temp_coefficients);
    }

    #[test]
    fn test_temp_compensator_set_coefficients() {
        let mut comp = TemperatureCompensator::new([0.0; 4], 25.0);
        comp.set_coefficients([1e-6, 0.0, 0.0, 0.0]);
        assert!((comp.offset_at(25.0) - 1e-6).abs() < 1e-20);
    }

    // --- KalmanFrequencyTracker tests ---

    #[test]
    fn test_kalman_initial_state() {
        let tracker = KalmanFrequencyTracker::new(NOMINAL_10MHZ, 1.0);
        let (freq, drift) = tracker.state();
        assert!((freq - NOMINAL_10MHZ).abs() < 1e-10);
        assert!(drift.abs() < 1e-10);
        assert_eq!(tracker.update_count(), 0);
    }

    #[test]
    fn test_kalman_converges_to_constant_offset() {
        let mut tracker = KalmanFrequencyTracker::new(NOMINAL_10MHZ, 1.0);
        tracker.set_measurement_noise(0.01);
        let true_offset = 5.0; // 5 Hz above nominal
        for _ in 0..100 {
            tracker.update(NOMINAL_10MHZ + true_offset);
        }
        let (est_freq, _) = tracker.state();
        assert!(
            (est_freq - (NOMINAL_10MHZ + true_offset)).abs() < 0.1,
            "Kalman should converge to constant offset, got {}",
            est_freq - NOMINAL_10MHZ
        );
    }

    #[test]
    fn test_kalman_tracks_linear_drift() {
        let mut tracker = KalmanFrequencyTracker::new(NOMINAL_10MHZ, 1.0);
        tracker.set_process_noise(1e-2, 1e-4);
        tracker.set_measurement_noise(0.1);

        let drift_per_step = 0.1; // Hz per step
        for i in 0..200 {
            let measured = NOMINAL_10MHZ + drift_per_step * i as f64;
            tracker.update(measured);
        }
        let est_drift = tracker.drift_rate();
        // Allow 20% tolerance on drift estimate
        assert!(
            (est_drift - drift_per_step).abs() < drift_per_step * 0.2,
            "drift estimate {} should be near true drift {}",
            est_drift, drift_per_step
        );
    }

    #[test]
    fn test_kalman_predict_frequency() {
        let mut tracker = KalmanFrequencyTracker::new(NOMINAL_10MHZ, 1.0);
        tracker.set_measurement_noise(0.01);
        let drift = 0.05;
        for i in 0..100 {
            tracker.update(NOMINAL_10MHZ + drift * i as f64);
        }
        let pred_10 = tracker.predict_frequency(10.0);
        let (est_freq, est_drift) = tracker.state();
        let expected = est_freq + 10.0 * est_drift;
        assert!((pred_10 - expected).abs() < 1e-6);
    }

    #[test]
    fn test_kalman_reset() {
        let mut tracker = KalmanFrequencyTracker::new(NOMINAL_10MHZ, 1.0);
        tracker.update(NOMINAL_10MHZ + 100.0);
        tracker.update(NOMINAL_10MHZ + 200.0);
        assert!(tracker.update_count() == 2);
        tracker.reset();
        assert_eq!(tracker.update_count(), 0);
        assert!(tracker.frequency_offset().abs() < 1e-10);
    }

    #[test]
    fn test_kalman_covariance_decreases() {
        let mut tracker = KalmanFrequencyTracker::new(NOMINAL_10MHZ, 1.0);
        let (var0, _) = tracker.covariance_diagonal();
        for _ in 0..50 {
            tracker.update(NOMINAL_10MHZ);
        }
        let (var50, _) = tracker.covariance_diagonal();
        assert!(var50 < var0, "covariance should decrease with measurements");
    }

    // --- FrequencySteering tests ---

    #[test]
    fn test_steering_initial_correction() {
        let cfg = OscillatorConfig::default_tcxo(NOMINAL_10MHZ);
        let steering = FrequencySteering::new(cfg, AgingModelType::Linear, 1.0);
        // At t=0, at reference temp, correction should be ~zero
        let correction = steering.total_correction(25.0);
        assert!(correction.abs() < 1e-6, "initial correction should be near zero");
    }

    #[test]
    fn test_steering_update_advances_time() {
        let cfg = OscillatorConfig::default_tcxo(NOMINAL_10MHZ);
        let mut steering = FrequencySteering::new(cfg.clone(), AgingModelType::Linear, 1.0);
        steering.update(NOMINAL_10MHZ, 25.0);
        assert!((steering.elapsed_days() - 1.0).abs() < 1e-10);
        steering.update(NOMINAL_10MHZ, 25.0);
        assert!((steering.elapsed_days() - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_steering_reset() {
        let cfg = OscillatorConfig::default_tcxo(NOMINAL_10MHZ);
        let mut steering = FrequencySteering::new(cfg, AgingModelType::Linear, 1.0);
        steering.update(NOMINAL_10MHZ + 10.0, 30.0);
        steering.update(NOMINAL_10MHZ + 20.0, 30.0);
        steering.reset();
        assert!(steering.elapsed_days().abs() < 1e-15);
        assert_eq!(steering.tracker().update_count(), 0);
    }

    // --- Helper function tests ---

    #[test]
    fn test_frequency_offset_ppb_zero() {
        assert!(frequency_offset_ppb(NOMINAL_10MHZ, NOMINAL_10MHZ).abs() < 1e-10);
    }

    #[test]
    fn test_frequency_offset_ppb_positive() {
        // 1 Hz offset on 10 MHz = 100 ppb
        let ppb = frequency_offset_ppb(NOMINAL_10MHZ, NOMINAL_10MHZ + 1.0);
        assert!((ppb - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_frequency_offset_ppb_negative() {
        let ppb = frequency_offset_ppb(NOMINAL_10MHZ, NOMINAL_10MHZ - 1.0);
        assert!((ppb - (-100.0)).abs() < 1e-6);
    }

    #[test]
    fn test_phase_noise_to_jitter_typical() {
        // -100 dBc/Hz at 10 MHz carrier
        let jitter = phase_noise_to_jitter(-100.0, NOMINAL_10MHZ);
        // Should be in the femtoseconds-to-picoseconds range
        assert!(jitter > 0.0);
        assert!(jitter < 1e-9, "jitter should be sub-nanosecond for -100 dBc/Hz");
    }

    #[test]
    fn test_phase_noise_to_jitter_decreases_with_frequency() {
        let j1 = phase_noise_to_jitter(-100.0, 1e6);
        let j2 = phase_noise_to_jitter(-100.0, 1e9);
        assert!(j1 > j2, "higher carrier frequency should yield less jitter");
    }

    #[test]
    fn test_aging_rate_from_data_linear() {
        // Synthesize data with known 2 ppb/day aging
        let rate_ppb = 2.0;
        let step = 1.0;
        let data: Vec<f64> = (0..100)
            .map(|i| {
                NOMINAL_10MHZ + rate_ppb * 1e-9 * NOMINAL_10MHZ * (i as f64) * step
            })
            .collect();
        let est = aging_rate_from_data(&data, NOMINAL_10MHZ, step).unwrap();
        assert!(
            (est - rate_ppb).abs() < 0.01,
            "estimated rate {} should be near true rate {}",
            est, rate_ppb
        );
    }

    #[test]
    fn test_aging_rate_from_data_insufficient() {
        assert!(aging_rate_from_data(&[NOMINAL_10MHZ], NOMINAL_10MHZ, 1.0).is_none());
        assert!(aging_rate_from_data(&[], NOMINAL_10MHZ, 1.0).is_none());
    }

    #[test]
    fn test_aging_rate_from_data_constant() {
        let data = vec![NOMINAL_10MHZ; 50];
        let est = aging_rate_from_data(&data, NOMINAL_10MHZ, 1.0).unwrap();
        assert!(est.abs() < 1e-10, "constant frequency should yield ~zero aging rate");
    }
}
