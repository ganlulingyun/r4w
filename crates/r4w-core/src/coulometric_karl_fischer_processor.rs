//! Coulometric Karl Fischer titration signal processing for moisture determination.
//!
//! Karl Fischer (KF) titration is the reference method for determining water content
//! in a wide range of samples (chemicals, pharmaceuticals, petrochemicals, food).
//! In coulometric KF, iodine is generated electrochemically at a platinum anode
//! and reacts stoichiometrically with water according to:
//!
//! ```text
//! H2O + I2 + [imidazole] + SO2 + 2 CH3OH  →  [imidazolium]SO4CH3 + 2 HI
//! ```
//!
//! By Faraday's law the mass of water is directly proportional to the total
//! electric charge passed through the generating cell:
//!
//! ```text
//! m_H2O = (Q × M_H2O) / (n × F)
//!
//! where:
//!   Q     = total charge (coulombs)
//!   M_H2O = 18.015 g/mol (molar mass of water)
//!   n     = 2 (electrons transferred per mole I2)
//!   F     = 96485.332 C/mol (Faraday constant)
//! ```
//!
//! This gives the theoretical cell constant of 10.722 µg H2O per mC of charge.
//!
//! # Components
//!
//! | Struct / Function | Purpose |
//! |---|---|
//! | [`KfConfig`] | Cell and sample parameters |
//! | [`EndpointCriteria`] | Endpoint detection thresholds |
//! | [`TitrationData`] | Time-series current and charge data |
//! | [`KfProcessor`] | Core titration signal processor |
//! | [`ResultStatistics`] | Statistical summary of determinations |
//! | [`ControlChart`] | Shewhart control chart (mean, UCL, LCL) |
//!
//! # Example
//!
//! ```rust
//! use r4w_core::coulometric_karl_fischer_processor::{
//!     KfConfig, KfProcessor, EndpointCriteria,
//! };
//!
//! let config = KfConfig::default_coulometric(250.0); // 250 mg sample
//! let criteria = EndpointCriteria::default();
//! let mut proc = KfProcessor::new(config, criteria);
//!
//! // Simulate a 100-ppm sample titration
//! let signal = KfProcessor::generate_titration_signal(100.0, 250.0, 0.01, 200.0);
//! let charge = KfProcessor::integrate_current(&signal.times_s, &signal.currents_ua);
//! let water_ug = KfProcessor::charge_to_water(charge);
//! let ppm = KfProcessor::moisture_content_ppm(water_ug, 250.0);
//! assert!((ppm - 100.0).abs() < 5.0); // within 5 ppm
//! ```

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Faraday constant (C/mol).
const FARADAY: f64 = 96_485.332;

/// Molar mass of water (g/mol).
const M_H2O: f64 = 18.015;

/// Number of electrons transferred per mole of I2 generated.
const N_ELECTRONS: f64 = 2.0;

/// Theoretical cell constant: µg H2O per mC of charge.
///
/// From Faraday's law the mass of water (in grams) produced by Q coulombs
/// is `m = Q * M_H2O / (n * F)`. Converting grams to µg and C to mC:
///
/// ```text
/// cell_constant = M_H2O / (n * F) * 1e6 (µg/g) / 1e3 (mC/C)
///               = M_H2O / (n * F) * 1e3
///               = 18.015 / (2 × 96485.332) * 1e3
///               ≈ 0.09336 µg/mC
/// ```
///
/// Equivalently, 1 C of charge produces ~93.36 µg of water, and
/// 1 mg of water requires ~10.72 C (the "water equivalent" in coulombs).
const THEORETICAL_CELL_CONSTANT: f64 = M_H2O / (N_ELECTRONS * FARADAY) * 1e3;

// ---------------------------------------------------------------------------
// KfConfig
// ---------------------------------------------------------------------------

/// Configuration for a coulometric Karl Fischer titration cell.
#[derive(Debug, Clone, PartialEq)]
pub struct KfConfig {
    /// Cell constant in µg H2O per mC of charge.
    ///
    /// The theoretical value is ~0.09336 µg/mC (equivalently 93.36 µg/C).
    /// Practical cells are calibrated and may deviate slightly due to side
    /// reactions.
    pub cell_constant_ug_per_mc: f64,

    /// Sample mass in milligrams.
    pub sample_mass_mg: f64,

    /// Electrode surface area in cm².
    pub electrode_area_cm2: f64,

    /// Stirrer speed in revolutions per minute.
    pub stirrer_speed_rpm: f64,

    /// Endpoint drift limit in µg/min.
    ///
    /// The titration is considered complete when the drift rate falls below
    /// this threshold. Typical value: 10--20 µg/min.
    pub endpoint_drift_ug_min: f64,

    /// Ambient temperature in degrees Celsius.
    pub temperature_c: f64,
}

impl KfConfig {
    /// Create a default coulometric configuration for the given sample mass.
    ///
    /// Uses the theoretical cell constant (~0.09336 µg/mC), standard Pt electrode
    /// area (1.0 cm²), 400 rpm stirrer, 10 µg/min endpoint drift, and 25 °C.
    pub fn default_coulometric(sample_mass_mg: f64) -> Self {
        Self {
            cell_constant_ug_per_mc: THEORETICAL_CELL_CONSTANT,
            sample_mass_mg,
            electrode_area_cm2: 1.0,
            stirrer_speed_rpm: 400.0,
            endpoint_drift_ug_min: 10.0,
            temperature_c: 25.0,
        }
    }

    /// Create a configuration with a custom cell constant from calibration.
    pub fn with_cell_constant(mut self, cell_constant: f64) -> Self {
        self.cell_constant_ug_per_mc = cell_constant;
        self
    }

    /// Create a configuration for oven-method (solid samples).
    ///
    /// Uses a tighter endpoint drift (5 µg/min) because oven releases moisture
    /// more slowly and requires longer stabilization.
    pub fn oven_method(sample_mass_mg: f64, oven_temp_c: f64) -> Self {
        Self {
            cell_constant_ug_per_mc: THEORETICAL_CELL_CONSTANT,
            sample_mass_mg,
            electrode_area_cm2: 1.0,
            stirrer_speed_rpm: 400.0,
            endpoint_drift_ug_min: 5.0,
            temperature_c: oven_temp_c,
        }
    }
}

// ---------------------------------------------------------------------------
// EndpointCriteria
// ---------------------------------------------------------------------------

/// Criteria for determining the titration endpoint.
#[derive(Debug, Clone, PartialEq)]
pub struct EndpointCriteria {
    /// Maximum allowed drift rate at endpoint (µg/min).
    pub drift_limit_ug_min: f64,

    /// Minimum elapsed time before endpoint may be declared (seconds).
    ///
    /// Prevents false endpoints from noise in the first few seconds.
    pub minimum_time_s: f64,

    /// Time the drift must remain below the limit to confirm endpoint (seconds).
    pub stabilization_time_s: f64,
}

impl EndpointCriteria {
    /// Default criteria: 10 µg/min drift, 30 s minimum, 15 s stabilization.
    pub fn default() -> Self {
        Self {
            drift_limit_ug_min: 10.0,
            minimum_time_s: 30.0,
            stabilization_time_s: 15.0,
        }
    }

    /// Strict criteria for low-moisture samples.
    pub fn strict() -> Self {
        Self {
            drift_limit_ug_min: 5.0,
            minimum_time_s: 60.0,
            stabilization_time_s: 30.0,
        }
    }

    /// Relaxed criteria for high-moisture samples.
    pub fn relaxed() -> Self {
        Self {
            drift_limit_ug_min: 20.0,
            minimum_time_s: 15.0,
            stabilization_time_s: 10.0,
        }
    }
}

// ---------------------------------------------------------------------------
// TitrationData
// ---------------------------------------------------------------------------

/// Time-series data from a coulometric Karl Fischer titration.
#[derive(Debug, Clone)]
pub struct TitrationData {
    /// Elapsed time values in seconds.
    pub times_s: Vec<f64>,

    /// Measured current values in microamperes (µA).
    pub currents_ua: Vec<f64>,

    /// Cumulative charge in millicoulombs (mC), computed from integrated current.
    pub cumulative_charge_mc: Vec<f64>,
}

impl TitrationData {
    /// Create a new empty `TitrationData`.
    pub fn new() -> Self {
        Self {
            times_s: Vec::new(),
            currents_ua: Vec::new(),
            cumulative_charge_mc: Vec::new(),
        }
    }

    /// Number of data points recorded.
    pub fn len(&self) -> usize {
        self.times_s.len()
    }

    /// Whether no data has been recorded.
    pub fn is_empty(&self) -> bool {
        self.times_s.is_empty()
    }

    /// Add a data point. The cumulative charge is updated automatically via
    /// trapezoidal integration from the previous point.
    pub fn push(&mut self, time_s: f64, current_ua: f64) {
        if self.times_s.is_empty() {
            self.times_s.push(time_s);
            self.currents_ua.push(current_ua);
            self.cumulative_charge_mc.push(0.0);
        } else {
            let prev_t = *self.times_s.last().unwrap();
            let prev_i = *self.currents_ua.last().unwrap();
            let prev_q = *self.cumulative_charge_mc.last().unwrap();
            let dt = time_s - prev_t;
            // Trapezoidal: dQ = 0.5 * (I_prev + I_cur) * dt
            // current in µA, time in s → charge in µC, convert to mC (/1000)
            let dq_mc = 0.5 * (prev_i + current_ua) * dt / 1000.0;
            self.times_s.push(time_s);
            self.currents_ua.push(current_ua);
            self.cumulative_charge_mc.push(prev_q + dq_mc);
        }
    }

    /// Total charge accumulated (mC).
    pub fn total_charge_mc(&self) -> f64 {
        self.cumulative_charge_mc.last().copied().unwrap_or(0.0)
    }

    /// Total elapsed time (seconds).
    pub fn total_time_s(&self) -> f64 {
        if self.times_s.len() < 2 {
            return 0.0;
        }
        self.times_s.last().unwrap() - self.times_s.first().unwrap()
    }
}

// ---------------------------------------------------------------------------
// ResultStatistics
// ---------------------------------------------------------------------------

/// Statistical summary of a set of determinations.
#[derive(Debug, Clone, PartialEq)]
pub struct ResultStatistics {
    /// Number of determinations.
    pub n: usize,
    /// Mean value.
    pub mean: f64,
    /// Standard deviation (sample, n-1 denominator).
    pub std_dev: f64,
    /// Relative standard deviation (%) = 100 * std_dev / mean.
    pub rsd_percent: f64,
    /// Minimum value.
    pub min: f64,
    /// Maximum value.
    pub max: f64,
    /// 95% confidence interval half-width (t * s / sqrt(n)).
    pub confidence_95_half: f64,
}

// ---------------------------------------------------------------------------
// ControlChart
// ---------------------------------------------------------------------------

/// Shewhart control chart parameters from repeated measurements.
#[derive(Debug, Clone, PartialEq)]
pub struct ControlChart {
    /// Center line (mean of all values).
    pub center_line: f64,
    /// Upper control limit (mean + 3 sigma).
    pub ucl: f64,
    /// Lower control limit (mean - 3 sigma).
    pub lcl: f64,
    /// Upper warning limit (mean + 2 sigma).
    pub uwl: f64,
    /// Lower warning limit (mean - 2 sigma).
    pub lwl: f64,
    /// Standard deviation used.
    pub sigma: f64,
    /// Number of points.
    pub n: usize,
    /// Which values (indices) are out of control (outside UCL/LCL).
    pub out_of_control: Vec<usize>,
}

// ---------------------------------------------------------------------------
// KfProcessor
// ---------------------------------------------------------------------------

/// Core processor for coulometric Karl Fischer titration signal processing.
///
/// Provides methods covering the full titration workflow: signal generation,
/// current integration, endpoint detection, drift correction, moisture
/// calculation, and quality-control statistics.
#[derive(Debug, Clone)]
pub struct KfProcessor {
    /// Cell and sample configuration.
    pub config: KfConfig,
    /// Endpoint detection criteria.
    pub criteria: EndpointCriteria,
    /// Blank value in µg H2O (background moisture measured in an empty cell).
    pub blank_value_ug: f64,
    /// History of determinations (µg H2O) for repeatability / control chart.
    pub determination_history: Vec<f64>,
    /// Cell background drift history (µg/min values over time).
    pub drift_history: Vec<f64>,
    /// Cumulative charge consumed from the generating electrode (mC).
    pub total_electrode_charge_mc: f64,
    /// Maximum capacity of the generating electrode (mC).
    pub electrode_capacity_mc: f64,
}

impl KfProcessor {
    /// Create a new processor with the given configuration and endpoint criteria.
    pub fn new(config: KfConfig, criteria: EndpointCriteria) -> Self {
        Self {
            config,
            criteria,
            blank_value_ug: 0.0,
            determination_history: Vec::new(),
            drift_history: Vec::new(),
            total_electrode_charge_mc: 0.0,
            // Typical generator electrode capacity: ~1000 mg H2O worth of charge.
            // 1000 mg = 1e6 µg → charge = 1e6 / 0.09336 ≈ 10,711,204 mC ≈ 10712 C
            electrode_capacity_mc: 1_000_000.0 / THEORETICAL_CELL_CONSTANT,
        }
    }

    // -----------------------------------------------------------------------
    // Fundamental conversions
    // -----------------------------------------------------------------------

    /// Convert total charge (mC) to water mass (µg) using Faraday's law.
    ///
    /// ```text
    /// m_H2O = Q * M_H2O / (n * F)
    /// ```
    ///
    /// where Q is in coulombs. Since `charge_mc` is in millicoulombs we
    /// multiply by 1e-3 to get C, then multiply the result by 1e6 to get µg.
    ///
    /// Equivalently: `m_ug = charge_mc * THEORETICAL_CELL_CONSTANT`
    pub fn charge_to_water(charge_mc: f64) -> f64 {
        charge_mc * THEORETICAL_CELL_CONSTANT
    }

    /// Convert water mass (µg) back to charge (mC).
    pub fn water_to_charge(water_ug: f64) -> f64 {
        water_ug / THEORETICAL_CELL_CONSTANT
    }

    /// Integrate current (µA) over time (s) using the trapezoidal rule.
    ///
    /// Returns total charge in millicoulombs (mC).
    ///
    /// The integral of current (µA) over time (s) gives charge in
    /// microcoulombs (µC); dividing by 1000 converts to mC.
    pub fn integrate_current(times_s: &[f64], currents_ua: &[f64]) -> f64 {
        assert_eq!(times_s.len(), currents_ua.len(), "time and current arrays must match");
        if times_s.len() < 2 {
            return 0.0;
        }
        let mut total_uc = 0.0;
        for i in 1..times_s.len() {
            let dt = times_s[i] - times_s[i - 1];
            total_uc += 0.5 * (currents_ua[i - 1] + currents_ua[i]) * dt;
        }
        total_uc / 1000.0 // µC → mC
    }

    /// Calculate moisture content in parts per million (ppm, µg/g).
    ///
    /// ```text
    /// ppm = (water_ug / sample_mass_mg) * 1000
    /// ```
    ///
    /// (1 mg = 1000 µg, so dividing µg by mg gives parts-per-thousand;
    /// multiplying by 1000 gives ppm.)
    pub fn moisture_content_ppm(water_ug: f64, sample_mass_mg: f64) -> f64 {
        if sample_mass_mg <= 0.0 {
            return 0.0;
        }
        (water_ug / sample_mass_mg) * 1000.0
    }

    /// Calculate moisture content as a percentage (% w/w).
    ///
    /// ```text
    /// % = (water_ug / sample_mass_mg) / 10
    /// ```
    pub fn moisture_content_percent(water_ug: f64, sample_mass_mg: f64) -> f64 {
        if sample_mass_mg <= 0.0 {
            return 0.0;
        }
        (water_ug / sample_mass_mg) / 10.0
    }

    // -----------------------------------------------------------------------
    // Endpoint detection
    // -----------------------------------------------------------------------

    /// Detect the endpoint of a titration from time-series data.
    ///
    /// Returns `Some((time_s, index))` of the first endpoint, or `None` if the
    /// endpoint was never reached. The drift rate is estimated as the water
    /// recovery rate (µg/min) over a sliding window equal to the stabilization
    /// time.
    pub fn endpoint_detection(
        &self,
        data: &TitrationData,
    ) -> Option<(f64, usize)> {
        if data.len() < 3 {
            return None;
        }
        let stab = self.criteria.stabilization_time_s;
        let min_t = self.criteria.minimum_time_s;
        let drift_limit = self.criteria.drift_limit_ug_min;
        let cell_k = self.config.cell_constant_ug_per_mc;

        // Walk through data; at each point, look backwards by stabilization_time_s
        // and compute drift rate (µg/min) = delta_water / delta_time * 60.
        for i in 1..data.len() {
            let t_i = data.times_s[i];
            if t_i < min_t {
                continue;
            }

            // Find index of the point approximately stab seconds before
            let t_start = t_i - stab;
            if t_start < 0.0 {
                continue;
            }
            // Binary-ish search for the closest point to t_start
            let j = match data.times_s[..i]
                .iter()
                .position(|&t| t >= t_start)
            {
                Some(idx) => idx,
                None => continue,
            };
            let dt = t_i - data.times_s[j];
            if dt < stab * 0.5 {
                continue; // not enough window
            }
            let dq = data.cumulative_charge_mc[i] - data.cumulative_charge_mc[j];
            let dw_ug = dq * cell_k;
            let drift_ug_min = (dw_ug / dt) * 60.0;

            if drift_ug_min.abs() < drift_limit {
                return Some((t_i, i));
            }
        }
        None
    }

    // -----------------------------------------------------------------------
    // Drift correction
    // -----------------------------------------------------------------------

    /// Subtract background drift from a gross water result.
    ///
    /// `background_drift_ug_min` is the cell's background moisture ingress rate
    /// measured before the titration. `titration_time_s` is the titration
    /// duration. Returns the corrected water value in µg.
    pub fn drift_correction(
        gross_water_ug: f64,
        background_drift_ug_min: f64,
        titration_time_s: f64,
    ) -> f64 {
        let drift_contribution = background_drift_ug_min * (titration_time_s / 60.0);
        gross_water_ug - drift_contribution
    }

    // -----------------------------------------------------------------------
    // Recovery rate
    // -----------------------------------------------------------------------

    /// Compute the water recovery rate (µg/s) at each time point.
    ///
    /// Uses central differences for interior points and forward/backward
    /// differences at the boundaries.
    pub fn recovery_rate(data: &TitrationData, cell_constant: f64) -> Vec<f64> {
        let n = data.len();
        if n < 2 {
            return vec![0.0; n];
        }
        let mut rates = vec![0.0; n];
        // Forward difference at start
        {
            let dt = data.times_s[1] - data.times_s[0];
            let dq = data.cumulative_charge_mc[1] - data.cumulative_charge_mc[0];
            if dt > 0.0 {
                rates[0] = (dq * cell_constant) / dt;
            }
        }
        // Central differences
        for i in 1..n - 1 {
            let dt = data.times_s[i + 1] - data.times_s[i - 1];
            let dq = data.cumulative_charge_mc[i + 1] - data.cumulative_charge_mc[i - 1];
            if dt > 0.0 {
                rates[i] = (dq * cell_constant) / dt;
            }
        }
        // Backward difference at end
        {
            let dt = data.times_s[n - 1] - data.times_s[n - 2];
            let dq = data.cumulative_charge_mc[n - 1] - data.cumulative_charge_mc[n - 2];
            if dt > 0.0 {
                rates[n - 1] = (dq * cell_constant) / dt;
            }
        }
        rates
    }

    // -----------------------------------------------------------------------
    // Signal generation (simulation)
    // -----------------------------------------------------------------------

    /// Generate a simulated coulometric KF titration signal.
    ///
    /// Models the current profile as a rapid rise followed by exponential decay
    /// as the water is consumed, plus a constant background drift current.
    ///
    /// # Arguments
    ///
    /// * `moisture_ppm` - Target moisture content of the simulated sample.
    /// * `sample_mass_mg` - Sample mass in mg.
    /// * `time_step_s` - Time step between data points (seconds).
    /// * `duration_s` - Total simulation duration (seconds).
    ///
    /// # Returns
    ///
    /// A [`TitrationData`] with the simulated current profile and integrated charge.
    pub fn generate_titration_signal(
        moisture_ppm: f64,
        sample_mass_mg: f64,
        time_step_s: f64,
        duration_s: f64,
    ) -> TitrationData {
        // Total water in the sample (µg)
        let total_water_ug = moisture_ppm * sample_mass_mg / 1000.0;
        // Total charge needed (mC)
        let total_charge_mc = total_water_ug / THEORETICAL_CELL_CONSTANT;
        // Total charge in µC (for current in µA * time in s)
        let total_charge_uc = total_charge_mc * 1000.0;

        // Time constant for exponential current decay.
        // Typical: most of the water reacts in the first 30--60 s.
        let tau = duration_s / 5.0;

        // Peak current: set so that the integral of I(t) = I_peak * exp(-t/tau)
        // from 0 to duration equals total_charge_uc.
        // Integral = I_peak * tau * (1 - exp(-duration/tau))
        let integral_factor = tau * (1.0 - (-duration_s / tau).exp());
        let i_peak = if integral_factor > 0.0 {
            total_charge_uc / integral_factor
        } else {
            0.0
        };

        // Background drift current (µA) corresponding to ~10 µg/min
        let drift_ug_min = 10.0;
        let drift_current_ua = drift_ug_min / 60.0 / THEORETICAL_CELL_CONSTANT * 1000.0;

        let mut data = TitrationData::new();
        let n_points = (duration_s / time_step_s).ceil() as usize + 1;

        for k in 0..n_points {
            let t = k as f64 * time_step_s;
            if t > duration_s {
                break;
            }
            // Exponentially decaying titration current + background drift
            let i_titration = i_peak * (-t / tau).exp();
            let current = i_titration + drift_current_ua;
            data.push(t, current);
        }

        data
    }

    /// Generate a titration curve shape as (time, current) pairs for a given
    /// total water amount. This is a simplified wrapper around
    /// [`generate_titration_signal`].
    pub fn titration_curve(
        water_ug: f64,
        duration_s: f64,
        time_step_s: f64,
    ) -> Vec<(f64, f64)> {
        let ppm = 100.0; // arbitrary; we scale to match water_ug
        let mass = water_ug / ppm * 1000.0;
        let data = Self::generate_titration_signal(ppm, mass, time_step_s, duration_s);
        data.times_s
            .into_iter()
            .zip(data.currents_ua.into_iter())
            .collect()
    }

    // -----------------------------------------------------------------------
    // Blank and calibration
    // -----------------------------------------------------------------------

    /// Perform a blank determination: titrate an empty cell and record the
    /// measured water as the blank value.
    pub fn blank_determination(&mut self, blank_data: &TitrationData) {
        let q = blank_data.total_charge_mc();
        let water = q * self.config.cell_constant_ug_per_mc;
        self.blank_value_ug = water;
    }

    /// Apply blank correction to a gross result.
    pub fn apply_blank_correction(&self, gross_water_ug: f64) -> f64 {
        gross_water_ug - self.blank_value_ug
    }

    /// Record a completed determination for repeatability tracking.
    pub fn record_determination(&mut self, water_ug: f64) {
        self.determination_history.push(water_ug);
    }

    // -----------------------------------------------------------------------
    // Repeatability and statistics
    // -----------------------------------------------------------------------

    /// Check repeatability of multiple determinations.
    ///
    /// Returns the RSD (%) of the values. Per pharmacopoeia guidelines the
    /// RSD should typically be < 1% for coulometric KF on liquid samples.
    pub fn repeatability_check(values: &[f64]) -> f64 {
        if values.len() < 2 {
            return 0.0;
        }
        let stats = Self::result_statistics(values);
        stats.rsd_percent
    }

    /// Compute full statistics for a set of determinations.
    pub fn result_statistics(values: &[f64]) -> ResultStatistics {
        let n = values.len();
        if n == 0 {
            return ResultStatistics {
                n: 0,
                mean: 0.0,
                std_dev: 0.0,
                rsd_percent: 0.0,
                min: 0.0,
                max: 0.0,
                confidence_95_half: 0.0,
            };
        }
        let mean = values.iter().sum::<f64>() / n as f64;
        let min = values.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        let std_dev = if n > 1 {
            let var = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (n - 1) as f64;
            var.sqrt()
        } else {
            0.0
        };

        let rsd_percent = if mean.abs() > 1e-15 {
            100.0 * std_dev / mean.abs()
        } else {
            0.0
        };

        // t-value for 95% CI (two-tailed) -- use lookup for small n
        let t_95 = t_value_95(n);
        let confidence_95_half = if n > 1 {
            t_95 * std_dev / (n as f64).sqrt()
        } else {
            0.0
        };

        ResultStatistics {
            n,
            mean,
            std_dev,
            rsd_percent,
            min,
            max,
            confidence_95_half,
        }
    }

    // -----------------------------------------------------------------------
    // Control chart
    // -----------------------------------------------------------------------

    /// Build a Shewhart control chart from repeated measurements.
    ///
    /// Returns mean, UCL (mean + 3σ), LCL (mean - 3σ), and warning limits
    /// (±2σ), plus indices of any out-of-control points.
    pub fn control_chart(values: &[f64]) -> ControlChart {
        let stats = Self::result_statistics(values);
        let sigma = stats.std_dev;
        let mean = stats.mean;
        let ucl = mean + 3.0 * sigma;
        let lcl = mean - 3.0 * sigma;
        let uwl = mean + 2.0 * sigma;
        let lwl = mean - 2.0 * sigma;

        let out_of_control: Vec<usize> = values
            .iter()
            .enumerate()
            .filter(|(_i, &v)| v > ucl || v < lcl)
            .map(|(i, _)| i)
            .collect();

        ControlChart {
            center_line: mean,
            ucl,
            lcl,
            uwl,
            lwl,
            sigma,
            n: values.len(),
            out_of_control,
        }
    }

    // -----------------------------------------------------------------------
    // Sample introduction and environmental corrections
    // -----------------------------------------------------------------------

    /// Correct for atmospheric moisture introduced during sample injection.
    ///
    /// When the cell is opened to inject a liquid sample, ambient air enters
    /// the cell. The correction depends on the injection duration and the
    /// ambient humidity.
    ///
    /// # Arguments
    ///
    /// * `injection_time_s` - Duration the cell is open (seconds).
    /// * `cell_volume_ml` - Cell headspace volume (mL).
    /// * `ambient_rh_percent` - Ambient relative humidity (%).
    /// * `temperature_c` - Ambient temperature (°C).
    ///
    /// # Returns
    ///
    /// Estimated atmospheric water ingress in µg.
    pub fn sample_introduction_correction(
        injection_time_s: f64,
        cell_volume_ml: f64,
        ambient_rh_percent: f64,
        temperature_c: f64,
    ) -> f64 {
        // Saturation vapor pressure (Antoine equation for water, T in °C)
        // P_sat (mbar) ≈ 6.1078 * exp(17.27 * T / (T + 237.3))
        let p_sat_mbar = 6.1078 * (17.27 * temperature_c / (temperature_c + 237.3)).exp();
        // Absolute humidity (g/m³) = 2.167 * P_w / (T + 273.15)
        // where P_w = RH/100 * P_sat in mbar
        let p_w = ambient_rh_percent / 100.0 * p_sat_mbar;
        let abs_humidity_g_m3 = 2.167 * p_w / (temperature_c + 273.15);

        // Volume of air exchanged ≈ cell_volume * (injection_time / characteristic_time)
        // Assume ~50% air exchange in the injection time at 1 atm
        let exchange_fraction = 0.5_f64.min(injection_time_s / 10.0);
        let air_volume_ml = cell_volume_ml * exchange_fraction;
        let air_volume_m3 = air_volume_ml * 1e-6;

        // Water mass in µg = abs_humidity (g/m³) * volume (m³) * 1e6 µg/g
        abs_humidity_g_m3 * air_volume_m3 * 1e6
    }

    // -----------------------------------------------------------------------
    // Cell conditioning / background drift
    // -----------------------------------------------------------------------

    /// Monitor the cell background drift by recording drift values over time.
    ///
    /// Adds a drift measurement (µg/min) and returns the trend (slope of drift
    /// vs measurement number). An increasing trend indicates reagent
    /// degradation or a seal leak.
    pub fn cell_conditioning_monitor(&mut self, drift_ug_min: f64) -> f64 {
        self.drift_history.push(drift_ug_min);
        // Simple linear regression slope on drift_history
        let n = self.drift_history.len();
        if n < 2 {
            return 0.0;
        }
        let x_mean = (n - 1) as f64 / 2.0;
        let y_mean: f64 = self.drift_history.iter().sum::<f64>() / n as f64;
        let mut num = 0.0;
        let mut den = 0.0;
        for (i, &y) in self.drift_history.iter().enumerate() {
            let x = i as f64;
            num += (x - x_mean) * (y - y_mean);
            den += (x - x_mean).powi(2);
        }
        if den.abs() < 1e-15 {
            0.0
        } else {
            num / den
        }
    }

    // -----------------------------------------------------------------------
    // Reagent capacity
    // -----------------------------------------------------------------------

    /// Track cumulative electrode usage and estimate remaining capacity.
    ///
    /// # Arguments
    ///
    /// * `charge_used_mc` - Charge consumed in the latest titration (mC).
    ///
    /// # Returns
    ///
    /// Remaining capacity as a fraction (0.0 -- 1.0).
    pub fn reagent_capacity_check(&mut self, charge_used_mc: f64) -> f64 {
        self.total_electrode_charge_mc += charge_used_mc;
        let remaining = 1.0 - (self.total_electrode_charge_mc / self.electrode_capacity_mc);
        remaining.max(0.0)
    }

    /// Remaining reagent capacity as water equivalent in µg.
    pub fn remaining_capacity_ug(&self) -> f64 {
        let remaining_mc = self.electrode_capacity_mc - self.total_electrode_charge_mc;
        remaining_mc.max(0.0) * THEORETICAL_CELL_CONSTANT
    }

    // -----------------------------------------------------------------------
    // Coulometric efficiency
    // -----------------------------------------------------------------------

    /// Calculate the coulometric efficiency.
    ///
    /// Compares the measured water (from a known standard) against the
    /// theoretical yield from Faraday's law. An efficiency near 100% indicates
    /// that all charge goes to generating I2 that reacts with water.
    ///
    /// # Arguments
    ///
    /// * `measured_water_ug` - Water found by the titration (µg).
    /// * `charge_mc` - Total charge passed (mC).
    ///
    /// # Returns
    ///
    /// Efficiency as a percentage (ideally ~100%).
    pub fn coulometric_efficiency(measured_water_ug: f64, charge_mc: f64) -> f64 {
        let theoretical = Self::charge_to_water(charge_mc);
        if theoretical.abs() < 1e-15 {
            return 0.0;
        }
        (measured_water_ug / theoretical) * 100.0
    }

    // -----------------------------------------------------------------------
    // Temperature correction
    // -----------------------------------------------------------------------

    /// Apply temperature correction to a moisture result.
    ///
    /// The endpoint detection is affected by solvent vapor pressure, which
    /// changes with temperature. The correction factor is approximately
    /// 0.5% per °C deviation from 25 °C for methanol-based reagents.
    ///
    /// # Arguments
    ///
    /// * `water_ug` - Uncorrected water value (µg).
    /// * `actual_temp_c` - Actual cell temperature (°C).
    /// * `reference_temp_c` - Reference temperature (°C), typically 25.
    ///
    /// # Returns
    ///
    /// Temperature-corrected water value (µg).
    pub fn temperature_correction(
        water_ug: f64,
        actual_temp_c: f64,
        reference_temp_c: f64,
    ) -> f64 {
        let dt = actual_temp_c - reference_temp_c;
        // Correction factor: ~0.5% per °C
        let correction_factor = 1.0 + 0.005 * dt;
        water_ug / correction_factor
    }

    // -----------------------------------------------------------------------
    // Bipotentiometric detection
    // -----------------------------------------------------------------------

    /// Model the bipotentiometric detector response.
    ///
    /// In bipotentiometric detection, a small constant polarization current
    /// (typically 5--20 µA) is applied between two indicator Pt electrodes.
    /// When excess I2 is present (after endpoint), the voltage drops sharply
    /// because both electrodes can participate in the I2/I- redox couple.
    ///
    /// # Arguments
    ///
    /// * `i2_concentration` - Excess I2 concentration (arbitrary units, 0 = no excess).
    /// * `polarization_current_ua` - Applied polarization current (µA).
    /// * `electrode_area_cm2` - Indicator electrode area (cm²).
    ///
    /// # Returns
    ///
    /// Indicator voltage (mV). High voltage (>300 mV) means before endpoint;
    /// low voltage (<100 mV) means after endpoint (excess I2).
    pub fn detector_response_bipotentiometric(
        i2_concentration: f64,
        polarization_current_ua: f64,
        electrode_area_cm2: f64,
    ) -> f64 {
        // Simplified Butler-Volmer-like response:
        // At low [I2], the electrode is polarized and the overpotential is large.
        // At high [I2], both anodic and cathodic processes are facile → low V.
        //
        // V ≈ V_max / (1 + k * [I2])
        let v_max = 700.0; // mV, maximum polarization voltage at zero I2
        let k = 50.0 / electrode_area_cm2; // sensitivity, inversely proportional to area
        let current_factor = polarization_current_ua / 10.0; // normalized to 10 µA
        let voltage = (v_max * current_factor) / (1.0 + k * i2_concentration);
        voltage.min(v_max)
    }

    // -----------------------------------------------------------------------
    // Oven method correction
    // -----------------------------------------------------------------------

    /// Correct results for oven (heating) method where solid samples are heated
    /// to release moisture which is carried to the KF cell by a dry gas.
    ///
    /// The correction accounts for:
    /// 1. Transfer efficiency (not all released moisture reaches the cell).
    /// 2. Decomposition byproducts that may interfere with KF chemistry.
    ///
    /// # Arguments
    ///
    /// * `measured_water_ug` - Water measured in the KF cell (µg).
    /// * `transfer_efficiency` - Fraction of released moisture that reaches cell (0--1).
    /// * `decomposition_factor` - Fraction of measured water that is actually from
    ///   decomposition products, not true moisture (0--1, typically 0--0.05).
    ///
    /// # Returns
    ///
    /// Corrected sample moisture in µg.
    pub fn oven_method_correction(
        measured_water_ug: f64,
        transfer_efficiency: f64,
        decomposition_factor: f64,
    ) -> f64 {
        let eff = transfer_efficiency.max(0.01).min(1.0);
        let true_moisture = measured_water_ug * (1.0 - decomposition_factor);
        true_moisture / eff
    }

    // -----------------------------------------------------------------------
    // Full titration workflow
    // -----------------------------------------------------------------------

    /// Process a complete titration: integrate, detect endpoint, correct, and
    /// calculate moisture content.
    ///
    /// Returns `(moisture_ppm, water_ug_corrected, endpoint_time_s)` or `None`
    /// if no endpoint was found.
    pub fn process_titration(
        &mut self,
        data: &TitrationData,
        background_drift_ug_min: f64,
    ) -> Option<(f64, f64, f64)> {
        // 1. Detect endpoint
        let (endpoint_t, _idx) = self.endpoint_detection(data)?;

        // 2. Total charge at endpoint
        // Interpolate charge at endpoint time
        let charge_mc = Self::interpolate_charge_at_time(data, endpoint_t);

        // 3. Gross water
        let gross_water = charge_mc * self.config.cell_constant_ug_per_mc;

        // 4. Blank correction
        let after_blank = self.apply_blank_correction(gross_water);

        // 5. Drift correction
        let corrected = Self::drift_correction(after_blank, background_drift_ug_min, endpoint_t);

        // 6. Temperature correction
        let temp_corrected = Self::temperature_correction(
            corrected,
            self.config.temperature_c,
            25.0,
        );

        // 7. PPM
        let ppm = Self::moisture_content_ppm(temp_corrected, self.config.sample_mass_mg);

        // 8. Track electrode usage
        let _ = self.reagent_capacity_check(charge_mc);

        // 9. Record determination
        self.record_determination(temp_corrected);

        Some((ppm, temp_corrected, endpoint_t))
    }

    /// Interpolate the cumulative charge at a specific time point.
    fn interpolate_charge_at_time(data: &TitrationData, time_s: f64) -> f64 {
        if data.is_empty() {
            return 0.0;
        }
        // Find bracketing indices
        let n = data.len();
        if time_s <= data.times_s[0] {
            return data.cumulative_charge_mc[0];
        }
        if time_s >= data.times_s[n - 1] {
            return data.cumulative_charge_mc[n - 1];
        }
        for i in 1..n {
            if data.times_s[i] >= time_s {
                let t0 = data.times_s[i - 1];
                let t1 = data.times_s[i];
                let q0 = data.cumulative_charge_mc[i - 1];
                let q1 = data.cumulative_charge_mc[i];
                let frac = (time_s - t0) / (t1 - t0);
                return q0 + frac * (q1 - q0);
            }
        }
        data.cumulative_charge_mc[n - 1]
    }
}

// ---------------------------------------------------------------------------
// Helper: Student's t-value for 95% CI (two-tailed)
// ---------------------------------------------------------------------------

/// Lookup table for Student's t-distribution, 95% confidence (two-tailed).
///
/// Index by degrees of freedom (n-1). For df > 30, use 1.96 (normal approx).
fn t_value_95(n: usize) -> f64 {
    if n <= 1 {
        return 12.706; // df=1 (essentially useless but formally correct)
    }
    let df = n - 1;
    // Tabulated t values for df = 1..30
    const T_TABLE: [f64; 30] = [
        12.706, 4.303, 3.182, 2.776, 2.571, // df 1-5
        2.447, 2.365, 2.306, 2.262, 2.228, // df 6-10
        2.201, 2.179, 2.160, 2.145, 2.131, // df 11-15
        2.120, 2.110, 2.101, 2.093, 2.086, // df 16-20
        2.080, 2.074, 2.069, 2.064, 2.060, // df 21-25
        2.056, 2.052, 2.048, 2.045, 2.042, // df 26-30
    ];
    if df <= 30 {
        T_TABLE[df - 1]
    } else {
        1.96
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // Constants and basic conversions
    // -----------------------------------------------------------------------

    #[test]
    fn test_theoretical_cell_constant() {
        // 18.015 / (2 * 96485.332) * 1e3 ≈ 0.09336 µg/mC
        let expected = 18.015 / (2.0 * 96485.332) * 1e3;
        assert!((THEORETICAL_CELL_CONSTANT - expected).abs() < 1e-6);
        // 1 C produces ~93.36 µg → 1 mC produces ~0.09336 µg
        assert!((THEORETICAL_CELL_CONSTANT - 0.09336).abs() < 0.001);
    }

    #[test]
    fn test_charge_to_water_basic() {
        // 1 mC should give ~10.72 µg H2O
        let w = KfProcessor::charge_to_water(1.0);
        assert!((w - THEORETICAL_CELL_CONSTANT).abs() < 1e-6);
    }

    #[test]
    fn test_charge_to_water_zero() {
        assert_eq!(KfProcessor::charge_to_water(0.0), 0.0);
    }

    #[test]
    fn test_water_to_charge_roundtrip() {
        let water = 250.0; // µg
        let charge = KfProcessor::water_to_charge(water);
        let recovered = KfProcessor::charge_to_water(charge);
        assert!((recovered - water).abs() < 1e-9);
    }

    #[test]
    fn test_charge_to_water_realistic_100ppm() {
        // 100 ppm in 250 mg sample = 25 µg water
        let water_ug = 25.0;
        let charge_mc = KfProcessor::water_to_charge(water_ug);
        // charge = 25 / 0.09336 ≈ 267.8 mC
        assert!((charge_mc - 25.0 / THEORETICAL_CELL_CONSTANT).abs() < 0.01);
        assert!(charge_mc > 250.0 && charge_mc < 300.0);
    }

    // -----------------------------------------------------------------------
    // Moisture content calculations
    // -----------------------------------------------------------------------

    #[test]
    fn test_moisture_content_ppm() {
        // 25 µg water / 250 mg sample = 100 ppm
        let ppm = KfProcessor::moisture_content_ppm(25.0, 250.0);
        assert!((ppm - 100.0).abs() < 1e-9);
    }

    #[test]
    fn test_moisture_content_ppm_zero_mass() {
        assert_eq!(KfProcessor::moisture_content_ppm(25.0, 0.0), 0.0);
    }

    #[test]
    fn test_moisture_content_percent() {
        // 25 µg / 250 mg / 10 = 0.01%
        let pct = KfProcessor::moisture_content_percent(25.0, 250.0);
        assert!((pct - 0.01).abs() < 1e-9);
    }

    #[test]
    fn test_moisture_content_percent_1pct() {
        // 1% = 10000 ppm → water_ug = 10000 * mass_mg / 1000 = 10 * mass
        // For 100 mg: water = 1000 µg
        let pct = KfProcessor::moisture_content_percent(1000.0, 100.0);
        assert!((pct - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_ppm_percent_relationship() {
        let water = 50.0;
        let mass = 200.0;
        let ppm = KfProcessor::moisture_content_ppm(water, mass);
        let pct = KfProcessor::moisture_content_percent(water, mass);
        // ppm = pct * 10000
        assert!((ppm - pct * 10_000.0).abs() < 1e-9);
    }

    // -----------------------------------------------------------------------
    // Current integration
    // -----------------------------------------------------------------------

    #[test]
    fn test_integrate_constant_current() {
        // 100 µA for 10 seconds = 1000 µC = 1.0 mC
        let times: Vec<f64> = (0..=100).map(|i| i as f64 * 0.1).collect();
        let currents = vec![100.0; times.len()];
        let q = KfProcessor::integrate_current(&times, &currents);
        assert!((q - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_integrate_linearly_increasing_current() {
        // I(t) = 10*t µA from t=0 to t=10 s
        // Integral = ∫0→10 10t dt = 5*100 = 500 µC = 0.5 mC
        let n = 1000;
        let times: Vec<f64> = (0..=n).map(|i| i as f64 * 10.0 / n as f64).collect();
        let currents: Vec<f64> = times.iter().map(|&t| 10.0 * t).collect();
        let q = KfProcessor::integrate_current(&times, &currents);
        assert!((q - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_integrate_single_point() {
        let q = KfProcessor::integrate_current(&[0.0], &[100.0]);
        assert_eq!(q, 0.0);
    }

    #[test]
    fn test_integrate_two_points() {
        // Trapezoidal: 0.5 * (100 + 200) * 1.0 = 150 µC = 0.15 mC
        let q = KfProcessor::integrate_current(&[0.0, 1.0], &[100.0, 200.0]);
        assert!((q - 0.15).abs() < 1e-9);
    }

    // -----------------------------------------------------------------------
    // TitrationData
    // -----------------------------------------------------------------------

    #[test]
    fn test_titration_data_push_and_charge() {
        let mut data = TitrationData::new();
        assert!(data.is_empty());
        assert_eq!(data.len(), 0);

        data.push(0.0, 100.0); // t=0, 100 µA
        assert_eq!(data.len(), 1);
        assert_eq!(data.total_charge_mc(), 0.0); // no interval yet

        data.push(1.0, 100.0); // t=1, 100 µA
        // Charge = 0.5 * (100+100) * 1.0 / 1000 = 0.1 mC
        assert!((data.total_charge_mc() - 0.1).abs() < 1e-9);

        data.push(2.0, 100.0); // t=2, 100 µA
        // Additional: 0.5 * (100+100) * 1.0 / 1000 = 0.1 mC → total 0.2 mC
        assert!((data.total_charge_mc() - 0.2).abs() < 1e-9);
    }

    #[test]
    fn test_titration_data_total_time() {
        let mut data = TitrationData::new();
        assert_eq!(data.total_time_s(), 0.0);
        data.push(5.0, 100.0);
        assert_eq!(data.total_time_s(), 0.0); // only one point
        data.push(15.0, 50.0);
        assert!((data.total_time_s() - 10.0).abs() < 1e-9);
    }

    // -----------------------------------------------------------------------
    // Drift correction
    // -----------------------------------------------------------------------

    #[test]
    fn test_drift_correction() {
        // Background drift: 10 µg/min, titration time: 120 s = 2 min
        // Drift contribution = 10 * 2 = 20 µg
        let gross = 100.0;
        let corrected = KfProcessor::drift_correction(gross, 10.0, 120.0);
        assert!((corrected - 80.0).abs() < 1e-9);
    }

    #[test]
    fn test_drift_correction_zero_drift() {
        let gross = 50.0;
        let corrected = KfProcessor::drift_correction(gross, 0.0, 300.0);
        assert!((corrected - 50.0).abs() < 1e-9);
    }

    // -----------------------------------------------------------------------
    // Temperature correction
    // -----------------------------------------------------------------------

    #[test]
    fn test_temperature_correction_at_reference() {
        // No correction at reference temperature
        let w = KfProcessor::temperature_correction(100.0, 25.0, 25.0);
        assert!((w - 100.0).abs() < 1e-9);
    }

    #[test]
    fn test_temperature_correction_above_reference() {
        // At 30 °C (5 °C above), correction factor = 1 + 0.005*5 = 1.025
        // Corrected = 100 / 1.025 ≈ 97.56
        let w = KfProcessor::temperature_correction(100.0, 30.0, 25.0);
        assert!((w - 100.0 / 1.025).abs() < 1e-6);
        assert!(w < 100.0); // higher temp → correction reduces result
    }

    #[test]
    fn test_temperature_correction_below_reference() {
        // At 20 °C (5 °C below), correction factor = 1 - 0.025 = 0.975
        let w = KfProcessor::temperature_correction(100.0, 20.0, 25.0);
        assert!((w - 100.0 / 0.975).abs() < 1e-6);
        assert!(w > 100.0); // lower temp → correction increases result
    }

    // -----------------------------------------------------------------------
    // Coulometric efficiency
    // -----------------------------------------------------------------------

    #[test]
    fn test_coulometric_efficiency_100pct() {
        let charge = 1.0; // mC
        let theoretical_water = KfProcessor::charge_to_water(charge);
        let eff = KfProcessor::coulometric_efficiency(theoretical_water, charge);
        assert!((eff - 100.0).abs() < 1e-9);
    }

    #[test]
    fn test_coulometric_efficiency_partial() {
        let charge = 2.0;
        let theoretical = KfProcessor::charge_to_water(charge);
        // Only 95% efficient: some charge went to side reactions
        let measured = theoretical * 0.95;
        let eff = KfProcessor::coulometric_efficiency(measured, charge);
        assert!((eff - 95.0).abs() < 1e-6);
    }

    #[test]
    fn test_coulometric_efficiency_zero_charge() {
        let eff = KfProcessor::coulometric_efficiency(10.0, 0.0);
        assert_eq!(eff, 0.0);
    }

    // -----------------------------------------------------------------------
    // Bipotentiometric detector
    // -----------------------------------------------------------------------

    #[test]
    fn test_detector_no_i2() {
        // No excess I2 → high voltage
        let v = KfProcessor::detector_response_bipotentiometric(0.0, 10.0, 1.0);
        assert!(v > 600.0); // should be near V_max
    }

    #[test]
    fn test_detector_excess_i2() {
        // Large excess I2 → low voltage
        let v = KfProcessor::detector_response_bipotentiometric(10.0, 10.0, 1.0);
        assert!(v < 50.0);
    }

    #[test]
    fn test_detector_voltage_decreases_with_i2() {
        let v1 = KfProcessor::detector_response_bipotentiometric(0.1, 10.0, 1.0);
        let v2 = KfProcessor::detector_response_bipotentiometric(1.0, 10.0, 1.0);
        let v3 = KfProcessor::detector_response_bipotentiometric(5.0, 10.0, 1.0);
        assert!(v1 > v2);
        assert!(v2 > v3);
    }

    #[test]
    fn test_detector_current_scaling() {
        // Higher polarization current → higher voltage
        let v_low = KfProcessor::detector_response_bipotentiometric(0.5, 5.0, 1.0);
        let v_high = KfProcessor::detector_response_bipotentiometric(0.5, 20.0, 1.0);
        assert!(v_high > v_low);
    }

    // -----------------------------------------------------------------------
    // Oven method correction
    // -----------------------------------------------------------------------

    #[test]
    fn test_oven_correction_perfect_transfer() {
        let corrected = KfProcessor::oven_method_correction(100.0, 1.0, 0.0);
        assert!((corrected - 100.0).abs() < 1e-9);
    }

    #[test]
    fn test_oven_correction_partial_transfer() {
        // 90% transfer efficiency, no decomposition
        let corrected = KfProcessor::oven_method_correction(90.0, 0.9, 0.0);
        assert!((corrected - 100.0).abs() < 1e-9);
    }

    #[test]
    fn test_oven_correction_with_decomposition() {
        // 100% transfer, 5% decomposition → true moisture = 95% of measured / 1.0
        let corrected = KfProcessor::oven_method_correction(100.0, 1.0, 0.05);
        assert!((corrected - 95.0).abs() < 1e-9);
    }

    #[test]
    fn test_oven_correction_combined() {
        // 80% transfer, 10% decomposition: measured=100 → true = 100*0.9/0.8 = 112.5
        let corrected = KfProcessor::oven_method_correction(100.0, 0.8, 0.1);
        assert!((corrected - 112.5).abs() < 1e-9);
    }

    // -----------------------------------------------------------------------
    // Sample introduction correction
    // -----------------------------------------------------------------------

    #[test]
    fn test_sample_introduction_positive() {
        // Should return a positive value (atmospheric moisture entering cell)
        let correction = KfProcessor::sample_introduction_correction(
            3.0,   // 3 seconds open
            10.0,  // 10 mL cell
            50.0,  // 50% RH
            25.0,  // 25 °C
        );
        assert!(correction > 0.0);
        // Should be small: a few µg at most
        assert!(correction < 10.0);
    }

    #[test]
    fn test_sample_introduction_zero_rh() {
        let correction = KfProcessor::sample_introduction_correction(3.0, 10.0, 0.0, 25.0);
        assert!((correction).abs() < 1e-12);
    }

    #[test]
    fn test_sample_introduction_increases_with_rh() {
        let c1 = KfProcessor::sample_introduction_correction(3.0, 10.0, 30.0, 25.0);
        let c2 = KfProcessor::sample_introduction_correction(3.0, 10.0, 80.0, 25.0);
        assert!(c2 > c1);
    }

    // -----------------------------------------------------------------------
    // Signal generation
    // -----------------------------------------------------------------------

    #[test]
    fn test_generate_titration_signal_shape() {
        let data = KfProcessor::generate_titration_signal(100.0, 250.0, 0.1, 200.0);
        assert!(data.len() > 100);
        // Current should start high and decrease
        assert!(data.currents_ua[0] > data.currents_ua[data.len() - 1]);
        // All currents should be positive
        assert!(data.currents_ua.iter().all(|&i| i > 0.0));
    }

    #[test]
    fn test_generate_titration_signal_water_recovery() {
        // Generate signal for 100 ppm in 250 mg → 25 µg water
        let data = KfProcessor::generate_titration_signal(100.0, 250.0, 0.1, 300.0);
        let charge = data.total_charge_mc();
        let water = KfProcessor::charge_to_water(charge);
        // Should recover the expected water amount (plus some drift)
        // Drift adds ~10 µg/min * 5 min = 50 µg extra, so total > 25
        assert!(water > 20.0);
    }

    #[test]
    fn test_generate_titration_signal_higher_moisture_more_charge() {
        let low = KfProcessor::generate_titration_signal(50.0, 250.0, 0.1, 200.0);
        let high = KfProcessor::generate_titration_signal(500.0, 250.0, 0.1, 200.0);
        assert!(high.total_charge_mc() > low.total_charge_mc());
    }

    #[test]
    fn test_titration_curve_pairs() {
        let curve = KfProcessor::titration_curve(25.0, 100.0, 0.5);
        assert!(!curve.is_empty());
        // First current should be higher than last
        assert!(curve.first().unwrap().1 > curve.last().unwrap().1);
    }

    // -----------------------------------------------------------------------
    // Recovery rate
    // -----------------------------------------------------------------------

    #[test]
    fn test_recovery_rate_constant_current() {
        let mut data = TitrationData::new();
        for i in 0..100 {
            data.push(i as f64, 100.0); // 100 µA constant
        }
        let cell_k = THEORETICAL_CELL_CONSTANT;
        let rates = KfProcessor::recovery_rate(&data, cell_k);
        // Constant current → constant recovery rate
        // rate = (dQ/dt) * cell_k, dQ/dt = 100 µA * (1s) / 1000 = 0.1 mC/s
        // rate = 0.1 * 10.722 ≈ 1.072 µg/s
        let expected_rate = 0.1 * cell_k;
        for &r in rates[1..rates.len() - 1].iter() {
            assert!((r - expected_rate).abs() < 0.1);
        }
    }

    // -----------------------------------------------------------------------
    // Blank determination and correction
    // -----------------------------------------------------------------------

    #[test]
    fn test_blank_determination() {
        let config = KfConfig::default_coulometric(250.0);
        let criteria = EndpointCriteria::default();
        let mut proc = KfProcessor::new(config, criteria);

        // Simulate a blank with 1 mC of charge → ~10.72 µg
        let mut blank = TitrationData::new();
        blank.push(0.0, 100.0);
        blank.push(10.0, 100.0); // 10s at 100µA = 1000 µC = 1 mC
        proc.blank_determination(&blank);
        assert!((proc.blank_value_ug - THEORETICAL_CELL_CONSTANT).abs() < 0.1);
    }

    #[test]
    fn test_blank_correction_applied() {
        let config = KfConfig::default_coulometric(250.0);
        let criteria = EndpointCriteria::default();
        let mut proc = KfProcessor::new(config, criteria);
        proc.blank_value_ug = 5.0;

        let corrected = proc.apply_blank_correction(25.0);
        assert!((corrected - 20.0).abs() < 1e-9);
    }

    // -----------------------------------------------------------------------
    // Repeatability and statistics
    // -----------------------------------------------------------------------

    #[test]
    fn test_result_statistics_basic() {
        let values = vec![100.0, 102.0, 98.0, 101.0, 99.0];
        let stats = KfProcessor::result_statistics(&values);
        assert_eq!(stats.n, 5);
        assert!((stats.mean - 100.0).abs() < 1e-9);
        assert!(stats.std_dev > 0.0);
        assert!(stats.rsd_percent < 5.0); // should be ~1.58%
        assert!((stats.min - 98.0).abs() < 1e-9);
        assert!((stats.max - 102.0).abs() < 1e-9);
        assert!(stats.confidence_95_half > 0.0);
    }

    #[test]
    fn test_result_statistics_identical_values() {
        let values = vec![50.0, 50.0, 50.0, 50.0];
        let stats = KfProcessor::result_statistics(&values);
        assert_eq!(stats.std_dev, 0.0);
        assert_eq!(stats.rsd_percent, 0.0);
    }

    #[test]
    fn test_result_statistics_empty() {
        let stats = KfProcessor::result_statistics(&[]);
        assert_eq!(stats.n, 0);
        assert_eq!(stats.mean, 0.0);
    }

    #[test]
    fn test_repeatability_check() {
        // Good repeatability: RSD < 1%
        let values = vec![100.0, 100.5, 99.5, 100.2, 99.8];
        let rsd = KfProcessor::repeatability_check(&values);
        assert!(rsd < 1.0);
    }

    #[test]
    fn test_repeatability_check_poor() {
        // Poor repeatability: RSD > 5%
        let values = vec![100.0, 110.0, 90.0, 105.0, 95.0];
        let rsd = KfProcessor::repeatability_check(&values);
        assert!(rsd > 5.0);
    }

    // -----------------------------------------------------------------------
    // Control chart
    // -----------------------------------------------------------------------

    #[test]
    fn test_control_chart_in_control() {
        let values = vec![100.0, 101.0, 99.0, 100.5, 99.5, 100.2, 99.8];
        let chart = KfProcessor::control_chart(&values);
        assert!((chart.center_line - 100.0).abs() < 0.5);
        assert!(chart.ucl > chart.center_line);
        assert!(chart.lcl < chart.center_line);
        assert!(chart.uwl > chart.center_line && chart.uwl < chart.ucl);
        assert!(chart.lwl < chart.center_line && chart.lwl > chart.lcl);
        assert!(chart.out_of_control.is_empty());
    }

    #[test]
    fn test_control_chart_out_of_control() {
        let mut values = vec![100.0; 20];
        values[10] = 200.0; // outlier
        let chart = KfProcessor::control_chart(&values);
        assert!(!chart.out_of_control.is_empty());
        assert!(chart.out_of_control.contains(&10));
    }

    #[test]
    fn test_control_chart_limits_relationship() {
        let values = vec![50.0, 52.0, 48.0, 51.0, 49.0, 50.5, 49.5];
        let chart = KfProcessor::control_chart(&values);
        assert!((chart.ucl - chart.center_line - 3.0 * chart.sigma).abs() < 1e-9);
        assert!((chart.center_line - chart.lcl - 3.0 * chart.sigma).abs() < 1e-9);
        assert!((chart.uwl - chart.center_line - 2.0 * chart.sigma).abs() < 1e-9);
    }

    // -----------------------------------------------------------------------
    // Cell conditioning monitor
    // -----------------------------------------------------------------------

    #[test]
    fn test_cell_conditioning_stable() {
        let config = KfConfig::default_coulometric(250.0);
        let criteria = EndpointCriteria::default();
        let mut proc = KfProcessor::new(config, criteria);

        // Constant drift → slope should be ~0
        for _ in 0..10 {
            let slope = proc.cell_conditioning_monitor(10.0);
            let _ = slope;
        }
        let slope = proc.cell_conditioning_monitor(10.0);
        assert!(slope.abs() < 0.01);
    }

    #[test]
    fn test_cell_conditioning_increasing() {
        let config = KfConfig::default_coulometric(250.0);
        let criteria = EndpointCriteria::default();
        let mut proc = KfProcessor::new(config, criteria);

        // Increasing drift → positive slope
        for i in 0..10 {
            proc.cell_conditioning_monitor(10.0 + i as f64);
        }
        let slope = proc.cell_conditioning_monitor(20.0);
        assert!(slope > 0.0);
    }

    // -----------------------------------------------------------------------
    // Reagent capacity
    // -----------------------------------------------------------------------

    #[test]
    fn test_reagent_capacity_initial() {
        let config = KfConfig::default_coulometric(250.0);
        let criteria = EndpointCriteria::default();
        let mut proc = KfProcessor::new(config, criteria);

        // Use a tiny amount
        let remaining = proc.reagent_capacity_check(1.0);
        assert!(remaining > 0.99);
    }

    #[test]
    fn test_reagent_capacity_depleted() {
        let config = KfConfig::default_coulometric(250.0);
        let criteria = EndpointCriteria::default();
        let mut proc = KfProcessor::new(config, criteria);

        // Use all capacity
        let remaining = proc.reagent_capacity_check(proc.electrode_capacity_mc);
        assert!((remaining).abs() < 1e-9);
    }

    #[test]
    fn test_remaining_capacity_ug() {
        let config = KfConfig::default_coulometric(250.0);
        let criteria = EndpointCriteria::default();
        let proc = KfProcessor::new(config, criteria);
        let capacity_ug = proc.remaining_capacity_ug();
        // Should be ~1e6 µg = 1000 mg = 1 g
        assert!((capacity_ug - 1_000_000.0).abs() < 1.0);
    }

    // -----------------------------------------------------------------------
    // Endpoint detection
    // -----------------------------------------------------------------------

    #[test]
    fn test_endpoint_detection_decaying_signal() {
        let config = KfConfig::default_coulometric(250.0);
        let criteria = EndpointCriteria {
            drift_limit_ug_min: 15.0,
            minimum_time_s: 20.0,
            stabilization_time_s: 10.0,
        };
        let proc = KfProcessor::new(config, criteria);

        // Build a signal that decays to near-zero
        let data = KfProcessor::generate_titration_signal(50.0, 250.0, 0.5, 200.0);
        let endpoint = proc.endpoint_detection(&data);
        // Should find an endpoint after the signal decays
        assert!(endpoint.is_some(), "endpoint should be detected");
        let (t, _idx) = endpoint.unwrap();
        assert!(t > 20.0); // after minimum time
    }

    #[test]
    fn test_endpoint_detection_too_short() {
        let config = KfConfig::default_coulometric(250.0);
        let criteria = EndpointCriteria::default(); // min 30s
        let proc = KfProcessor::new(config, criteria);

        // Only 5 seconds of data
        let mut data = TitrationData::new();
        for i in 0..50 {
            data.push(i as f64 * 0.1, 10.0);
        }
        let endpoint = proc.endpoint_detection(&data);
        assert!(endpoint.is_none());
    }

    // -----------------------------------------------------------------------
    // KfConfig constructors
    // -----------------------------------------------------------------------

    #[test]
    fn test_default_coulometric_config() {
        let config = KfConfig::default_coulometric(500.0);
        assert!((config.cell_constant_ug_per_mc - THEORETICAL_CELL_CONSTANT).abs() < 1e-6);
        assert_eq!(config.sample_mass_mg, 500.0);
        assert_eq!(config.stirrer_speed_rpm, 400.0);
        assert_eq!(config.endpoint_drift_ug_min, 10.0);
        assert_eq!(config.temperature_c, 25.0);
    }

    #[test]
    fn test_oven_method_config() {
        let config = KfConfig::oven_method(100.0, 150.0);
        assert_eq!(config.endpoint_drift_ug_min, 5.0);
        assert_eq!(config.temperature_c, 150.0);
    }

    #[test]
    fn test_with_cell_constant() {
        let config = KfConfig::default_coulometric(250.0).with_cell_constant(10.75);
        assert!((config.cell_constant_ug_per_mc - 10.75).abs() < 1e-9);
    }

    // -----------------------------------------------------------------------
    // EndpointCriteria presets
    // -----------------------------------------------------------------------

    #[test]
    fn test_endpoint_criteria_strict_tighter_than_relaxed() {
        let strict = EndpointCriteria::strict();
        let relaxed = EndpointCriteria::relaxed();
        assert!(strict.drift_limit_ug_min < relaxed.drift_limit_ug_min);
        assert!(strict.minimum_time_s > relaxed.minimum_time_s);
        assert!(strict.stabilization_time_s > relaxed.stabilization_time_s);
    }

    // -----------------------------------------------------------------------
    // t-value lookup
    // -----------------------------------------------------------------------

    #[test]
    fn test_t_value_95_known_values() {
        assert!((t_value_95(2) - 12.706).abs() < 0.001); // df=1
        assert!((t_value_95(3) - 4.303).abs() < 0.001); // df=2
        assert!((t_value_95(11) - 2.228).abs() < 0.001); // df=10
        assert!((t_value_95(31) - 2.042).abs() < 0.001); // df=30
        assert!((t_value_95(100) - 1.96).abs() < 0.001); // large df → normal
    }

    // -----------------------------------------------------------------------
    // Integration: full workflow
    // -----------------------------------------------------------------------

    #[test]
    fn test_full_workflow_100ppm() {
        let config = KfConfig::default_coulometric(250.0);
        let criteria = EndpointCriteria {
            drift_limit_ug_min: 15.0,
            minimum_time_s: 20.0,
            stabilization_time_s: 10.0,
        };
        let mut proc = KfProcessor::new(config, criteria);

        // Set a small blank
        proc.blank_value_ug = 2.0;

        let data = KfProcessor::generate_titration_signal(100.0, 250.0, 0.5, 200.0);
        let result = proc.process_titration(&data, 10.0);
        assert!(result.is_some());

        let (ppm, water_ug, endpoint_t) = result.unwrap();
        // With drift correction and blank removal, the result should be in a
        // reasonable range. The simulated signal includes background drift
        // which is then subtracted, so expect something in the neighborhood
        // of the target.
        assert!(ppm > 0.0);
        assert!(water_ug > 0.0);
        assert!(endpoint_t > 20.0);
    }

    #[test]
    fn test_determination_history_tracking() {
        let config = KfConfig::default_coulometric(250.0);
        let criteria = EndpointCriteria::default();
        let mut proc = KfProcessor::new(config, criteria);

        proc.record_determination(25.0);
        proc.record_determination(24.5);
        proc.record_determination(25.5);
        assert_eq!(proc.determination_history.len(), 3);

        let stats = KfProcessor::result_statistics(&proc.determination_history);
        assert_eq!(stats.n, 3);
        assert!((stats.mean - 25.0).abs() < 0.01);
    }

    // -----------------------------------------------------------------------
    // Charge interpolation
    // -----------------------------------------------------------------------

    #[test]
    fn test_interpolate_charge_at_time() {
        let mut data = TitrationData::new();
        data.push(0.0, 100.0);
        data.push(10.0, 100.0);
        // Charge at t=10 = 0.5*(100+100)*10/1000 = 1.0 mC

        // Interpolate at t=5 (midpoint)
        let q = KfProcessor::interpolate_charge_at_time(&data, 5.0);
        assert!((q - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_interpolate_charge_before_start() {
        let mut data = TitrationData::new();
        data.push(1.0, 100.0);
        data.push(2.0, 100.0);
        let q = KfProcessor::interpolate_charge_at_time(&data, 0.0);
        assert_eq!(q, data.cumulative_charge_mc[0]);
    }

    #[test]
    fn test_interpolate_charge_after_end() {
        let mut data = TitrationData::new();
        data.push(0.0, 100.0);
        data.push(1.0, 100.0);
        let q = KfProcessor::interpolate_charge_at_time(&data, 10.0);
        assert_eq!(q, data.cumulative_charge_mc[1]);
    }
}
