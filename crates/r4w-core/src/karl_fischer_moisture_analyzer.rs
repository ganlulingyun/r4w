//! # Karl Fischer Moisture Analyzer
//!
//! Karl Fischer titration moisture determination processor implementing coulometric
//! and volumetric KF titration analysis, endpoint detection via biamperometric
//! (dual-platinum electrode) current monitoring, moisture content calculation from
//! consumed reagent, drift correction, sample weight normalization, oven method
//! (for solids), and compliance with ASTM E203/E1064 standards.
//!
//! ## Key Components
//!
//! - **KarlFischerConfig** - Configuration for coulometric vs volumetric titration
//! - **CoulometricTitrator** - Coulometric KF: generates iodine electrochemically,
//!   measures charge (coulombs) consumed via Faraday's law
//! - **VolumetricTitrator** - Volumetric KF: dispenses reagent from burette,
//!   measures volume consumed
//! - **EndpointDetector** - Biamperometric endpoint detection monitoring polarization
//!   current between dual Pt electrodes
//! - **DriftCorrector** - Background moisture drift rate estimation and correction
//! - **OvenMethod** - Indirect KF for solids: heat sample, sweep moisture into
//!   titration cell with carrier gas
//! - **MoistureCalculation** - Compute ppm/percent moisture from titration data
//!   and sample weight
//! - **TitrationCurve** - Records current/voltage vs time/volume, finds inflection
//!   points
//!
//! ## Physics
//!
//! The Karl Fischer reaction consumes water stoichiometrically:
//!
//! ```text
//! I₂ + SO₂ + 3 Py + CH₃OH + H₂O → 2 PyH⁺I⁻ + PySO₃CH₃
//! ```
//!
//! - Faraday constant: F = 96485.3329 C/mol
//! - Water molar mass: M(H₂O) = 18.01528 g/mol
//! - Coulometric: 1 mol H₂O = 2F coulombs (two-electron oxidation of I⁻ to I₂)
//! - Volumetric: water mass = volume × reagent concentration (mg H₂O / mL)
//! - Typical endpoint: 10–50 µA sustained for 10–30 seconds (biamperometric)
//! - Typical drift: 1–20 µg/min background moisture ingress
//!
//! ## Standards
//!
//! - ASTM E203: Standard Test Method for Water Using Volumetric KF Titration
//! - ASTM E1064: Standard Test Method for Water in Organic Liquids by Coulometric KF
//! - ISO 760: Determination of Water — Karl Fischer Method (General)
//! - ASTM D6304: Standard Test Method for Determination of Water in Petroleum
//!   Products, Lubricating Oils, and Additives by Coulometric KF Titration

// ============================================================================
// Physical Constants
// ============================================================================

/// Faraday constant in coulombs per mole of electrons (CODATA 2018).
const FARADAY_CONSTANT: f64 = 96485.3329;

/// Molar mass of water in grams per mole.
const WATER_MOLAR_MASS: f64 = 18.01528;

/// Number of electrons transferred per mole of water in the KF reaction.
/// The oxidation 2I⁻ → I₂ + 2e⁻ requires 2 electrons per I₂, and 1 mol I₂
/// reacts with 1 mol H₂O.
const ELECTRONS_PER_WATER: f64 = 2.0;

/// Coulombs required per milligram of water (derived constant).
/// Q = (2 × F) / M(H₂O) × 1e-3 = 2 × 96485.3329 / 18.01528 × 0.001
/// ≈ 10.722 C/mg (but we compute from first principles).
const COULOMBS_PER_MG_WATER: f64 =
    ELECTRONS_PER_WATER * FARADAY_CONSTANT / (WATER_MOLAR_MASS * 1000.0);

/// Standard one-component volumetric reagent concentration (mg H₂O / mL).
const STANDARD_REAGENT_ONE_COMPONENT: f64 = 1.0;

/// Standard two-component volumetric reagent concentration (mg H₂O / mL).
const STANDARD_REAGENT_TWO_COMPONENT: f64 = 5.0;

/// Minimum endpoint persistence time in seconds.
const MIN_ENDPOINT_PERSISTENCE_S: f64 = 5.0;

/// Default endpoint current threshold in microamperes.
const DEFAULT_ENDPOINT_THRESHOLD_UA: f64 = 20.0;

/// Default endpoint persistence duration in seconds.
const DEFAULT_ENDPOINT_PERSISTENCE_S: f64 = 20.0;

// ============================================================================
// Enumerations
// ============================================================================

/// Titration method type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TitrationMethod {
    /// Coulometric: iodine generated electrochemically; best for low moisture (1 µg – 200 mg).
    Coulometric,
    /// Volumetric: reagent dispensed from burette; best for higher moisture (100 µg – 100%).
    Volumetric,
}

/// Reagent type for volumetric titrations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ReagentType {
    /// One-component reagent (iodine, SO₂, base, alcohol in one solution). ~1 mg/mL.
    OneComponent,
    /// Two-component reagent (separate iodine/SO₂ and alcohol/base). ~5 mg/mL.
    TwoComponent,
    /// Custom concentration.
    Custom(f64),
}

impl ReagentType {
    /// Returns the reagent concentration in mg H₂O per mL.
    pub fn concentration_mg_per_ml(&self) -> f64 {
        match self {
            ReagentType::OneComponent => STANDARD_REAGENT_ONE_COMPONENT,
            ReagentType::TwoComponent => STANDARD_REAGENT_TWO_COMPONENT,
            ReagentType::Custom(c) => *c,
        }
    }
}

/// Oven temperature program for indirect KF (solids).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OvenProgram {
    /// Isothermal hold at the given temperature (°C).
    Isothermal(f64),
    /// Linear ramp from start to end temperature (°C) over given minutes.
    Ramp { start_c: f64, end_c: f64, duration_min: f64 },
    /// Step program: hold at temperature for given minutes, then jump.
    Step { temperatures_c: [f64; 4], durations_min: [f64; 4], num_steps: usize },
}

/// Endpoint detection mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EndpointMode {
    /// Biamperometric (dual platinum electrode, constant voltage).
    Biamperometric,
    /// Bivoltametric (dual platinum electrode, constant current).
    Bivoltametric,
    /// Photometric (colorimetric iodine detection).
    Photometric,
}

/// Drift correction strategy.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DriftStrategy {
    /// No drift correction applied.
    None,
    /// Linear drift model (constant µg/min).
    Linear,
    /// Exponential decay model (drift decreases as cell equilibrates).
    Exponential,
}

// ============================================================================
// Configuration
// ============================================================================

/// Configuration for a Karl Fischer titration system.
#[derive(Debug, Clone)]
pub struct KarlFischerConfig {
    /// Titration method (coulometric or volumetric).
    pub method: TitrationMethod,
    /// Reagent type (only relevant for volumetric).
    pub reagent_type: ReagentType,
    /// Endpoint detection mode.
    pub endpoint_mode: EndpointMode,
    /// Endpoint current threshold in microamperes.
    pub endpoint_threshold_ua: f64,
    /// Endpoint persistence time in seconds.
    pub endpoint_persistence_s: f64,
    /// Drift correction strategy.
    pub drift_strategy: DriftStrategy,
    /// Maximum allowed drift rate in µg/min before warning.
    pub max_drift_ug_per_min: f64,
    /// Polarization voltage for biamperometric detection (mV).
    pub polarization_voltage_mv: f64,
    /// Stirring speed (RPM). Affects mixing and drift.
    pub stirring_rpm: f64,
}

impl Default for KarlFischerConfig {
    fn default() -> Self {
        Self {
            method: TitrationMethod::Coulometric,
            reagent_type: ReagentType::TwoComponent,
            endpoint_mode: EndpointMode::Biamperometric,
            endpoint_threshold_ua: DEFAULT_ENDPOINT_THRESHOLD_UA,
            endpoint_persistence_s: DEFAULT_ENDPOINT_PERSISTENCE_S,
            polarization_voltage_mv: 100.0,
            drift_strategy: DriftStrategy::Linear,
            max_drift_ug_per_min: 20.0,
            stirring_rpm: 400.0,
        }
    }
}

impl KarlFischerConfig {
    /// Create a configuration preset for coulometric titration (ASTM E1064).
    pub fn coulometric() -> Self {
        Self {
            method: TitrationMethod::Coulometric,
            reagent_type: ReagentType::OneComponent,
            endpoint_threshold_ua: 10.0,
            endpoint_persistence_s: 20.0,
            ..Default::default()
        }
    }

    /// Create a configuration preset for volumetric titration (ASTM E203).
    pub fn volumetric() -> Self {
        Self {
            method: TitrationMethod::Volumetric,
            reagent_type: ReagentType::TwoComponent,
            endpoint_threshold_ua: 20.0,
            endpoint_persistence_s: 10.0,
            ..Default::default()
        }
    }

    /// Create a configuration for petroleum products (ASTM D6304).
    pub fn petroleum() -> Self {
        Self {
            method: TitrationMethod::Coulometric,
            reagent_type: ReagentType::OneComponent,
            endpoint_threshold_ua: 10.0,
            endpoint_persistence_s: 30.0,
            max_drift_ug_per_min: 10.0,
            ..Default::default()
        }
    }
}

// ============================================================================
// Standalone Functions
// ============================================================================

/// Convert coulombs of charge consumed to milligrams of water.
///
/// Uses Faraday's law: m(mg) = Q × M(H₂O) / (n × F) × 1000
/// where n = 2 electrons per mole of water.
pub fn coulombs_to_water_mass(charge_coulombs: f64) -> f64 {
    charge_coulombs * WATER_MOLAR_MASS * 1000.0
        / (ELECTRONS_PER_WATER * FARADAY_CONSTANT)
}

/// Convert volume of reagent consumed to milligrams of water.
///
/// m(mg) = volume_ml × reagent_mg_per_ml
pub fn volume_to_water_mass(volume_ml: f64, reagent_mg_per_ml: f64) -> f64 {
    volume_ml * reagent_mg_per_ml
}

/// Calculate moisture content in parts per million (ppm, w/w).
///
/// ppm = (water_mg / sample_g) × 1000 = (water_mg / sample_mg) × 1e6
pub fn moisture_ppm(water_mg: f64, sample_g: f64) -> f64 {
    if sample_g <= 0.0 {
        return 0.0;
    }
    (water_mg / sample_g) * 1000.0
}

/// Calculate moisture content as weight percent.
///
/// % = (water_mg / (sample_g × 1000)) × 100
pub fn moisture_percent(water_mg: f64, sample_g: f64) -> f64 {
    if sample_g <= 0.0 {
        return 0.0;
    }
    (water_mg / (sample_g * 1000.0)) * 100.0
}

/// Estimate background drift rate from a series of readings.
///
/// Takes a slice of (time_s, cumulative_water_ug) pairs measured during
/// the pre-titration conditioning period.
///
/// Returns drift rate in µg/min using linear regression.
pub fn drift_rate(background_readings: &[(f64, f64)]) -> f64 {
    let n = background_readings.len();
    if n < 2 {
        return 0.0;
    }
    // Convert time from seconds to minutes for µg/min
    let nf = n as f64;
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut sum_xy = 0.0;
    let mut sum_xx = 0.0;
    for &(t_s, water_ug) in background_readings {
        let t_min = t_s / 60.0;
        sum_x += t_min;
        sum_y += water_ug;
        sum_xy += t_min * water_ug;
        sum_xx += t_min * t_min;
    }
    let denom = nf * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-15 {
        return 0.0;
    }
    (nf * sum_xy - sum_x * sum_y) / denom
}

/// Correct gross water for background drift.
///
/// net_water_mg = gross_water_mg − (drift_rate_ug_per_min × elapsed_min / 1000)
pub fn net_water(gross_water_mg: f64, drift_rate_ug_per_min: f64, elapsed_min: f64) -> f64 {
    let drift_mg = drift_rate_ug_per_min * elapsed_min / 1000.0;
    let net = gross_water_mg - drift_mg;
    if net < 0.0 { 0.0 } else { net }
}

/// Check whether the endpoint has been reached.
///
/// The endpoint is considered reached when the indicator current drops
/// below the threshold and remains there for at least `duration_s` seconds.
pub fn endpoint_reached(current_ua: f64, threshold_ua: f64, duration_s: f64) -> bool {
    current_ua <= threshold_ua && duration_s >= MIN_ENDPOINT_PERSISTENCE_S
}

/// Compute the theoretical charge required to titrate a given mass of water.
///
/// Q(C) = m(mg) × (n × F) / (M(H₂O) × 1000)
pub fn water_mass_to_coulombs(water_mg: f64) -> f64 {
    water_mg * ELECTRONS_PER_WATER * FARADAY_CONSTANT
        / (WATER_MOLAR_MASS * 1000.0)
}

/// Convert ppm moisture to percent.
pub fn ppm_to_percent(ppm: f64) -> f64 {
    ppm / 10000.0
}

/// Convert percent moisture to ppm.
pub fn percent_to_ppm(percent: f64) -> f64 {
    percent * 10000.0
}

/// Calculate the minimum sample weight (g) to achieve a given precision
/// at a target moisture level.
///
/// For a minimum water amount `min_water_mg` and target moisture `target_ppm`:
/// sample_g = min_water_mg × 1e6 / (target_ppm × 1e3) = min_water_mg × 1000 / target_ppm
pub fn min_sample_weight_g(target_ppm: f64, min_water_mg: f64) -> f64 {
    if target_ppm <= 0.0 {
        return 0.0;
    }
    min_water_mg * 1000.0 / target_ppm
}

/// Compute the coulometric current needed to complete titration in a given time.
///
/// I(A) = Q(C) / t(s)
pub fn required_current(water_mg: f64, time_s: f64) -> f64 {
    if time_s <= 0.0 {
        return 0.0;
    }
    water_mass_to_coulombs(water_mg) / time_s
}

// ============================================================================
// Titration Curve
// ============================================================================

/// A single data point on the titration curve.
#[derive(Debug, Clone, Copy)]
pub struct TitrationPoint {
    /// Elapsed time in seconds.
    pub time_s: f64,
    /// Indicator current in microamperes.
    pub current_ua: f64,
    /// Volume of reagent dispensed in mL (volumetric only).
    pub volume_ml: f64,
    /// Cumulative charge in coulombs (coulometric only).
    pub charge_c: f64,
}

/// Records and analyzes a titration curve (current vs time/volume).
#[derive(Debug, Clone)]
pub struct TitrationCurve {
    /// Recorded data points.
    pub points: Vec<TitrationPoint>,
}

impl TitrationCurve {
    /// Create a new empty titration curve.
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// Add a data point to the curve.
    pub fn add_point(&mut self, point: TitrationPoint) {
        self.points.push(point);
    }

    /// Return the total elapsed time in seconds.
    pub fn elapsed_s(&self) -> f64 {
        if self.points.is_empty() {
            return 0.0;
        }
        let first = self.points.first().unwrap().time_s;
        let last = self.points.last().unwrap().time_s;
        last - first
    }

    /// Return the total charge consumed in coulombs.
    pub fn total_charge_c(&self) -> f64 {
        if self.points.is_empty() {
            return 0.0;
        }
        self.points.last().unwrap().charge_c
    }

    /// Return the total volume dispensed in mL.
    pub fn total_volume_ml(&self) -> f64 {
        if self.points.is_empty() {
            return 0.0;
        }
        self.points.last().unwrap().volume_ml
    }

    /// Find the minimum indicator current across the curve.
    pub fn min_current_ua(&self) -> f64 {
        self.points
            .iter()
            .map(|p| p.current_ua)
            .fold(f64::MAX, f64::min)
    }

    /// Find the maximum indicator current across the curve.
    pub fn max_current_ua(&self) -> f64 {
        self.points
            .iter()
            .map(|p| p.current_ua)
            .fold(f64::MIN, f64::max)
    }

    /// Compute the first derivative of current with respect to time (µA/s).
    /// Returns a vector of (time, dI/dt) pairs.
    pub fn derivative_current_time(&self) -> Vec<(f64, f64)> {
        if self.points.len() < 2 {
            return Vec::new();
        }
        let mut result = Vec::with_capacity(self.points.len() - 1);
        for i in 0..self.points.len() - 1 {
            let dt = self.points[i + 1].time_s - self.points[i].time_s;
            if dt.abs() < 1e-12 {
                continue;
            }
            let di = self.points[i + 1].current_ua - self.points[i].current_ua;
            let mid_t = (self.points[i].time_s + self.points[i + 1].time_s) / 2.0;
            result.push((mid_t, di / dt));
        }
        result
    }

    /// Find inflection points where the rate of current change reverses sign.
    /// Returns indices into the derivative vector.
    pub fn inflection_points(&self) -> Vec<usize> {
        let deriv = self.derivative_current_time();
        if deriv.len() < 2 {
            return Vec::new();
        }
        let mut inflections = Vec::new();
        for i in 0..deriv.len() - 1 {
            let sign_a = deriv[i].1.signum();
            let sign_b = deriv[i + 1].1.signum();
            if sign_a != sign_b && sign_a != 0.0 && sign_b != 0.0 {
                inflections.push(i + 1);
            }
        }
        inflections
    }

    /// Estimate the endpoint time by finding when current first drops below
    /// the threshold and stays there for the persistence duration.
    pub fn find_endpoint_time(&self, threshold_ua: f64, persistence_s: f64) -> Option<f64> {
        if self.points.is_empty() {
            return None;
        }
        let mut below_since: Option<f64> = None;
        for p in &self.points {
            if p.current_ua <= threshold_ua {
                match below_since {
                    None => below_since = Some(p.time_s),
                    Some(start) => {
                        if p.time_s - start >= persistence_s {
                            return Some(start);
                        }
                    }
                }
            } else {
                below_since = None;
            }
        }
        None
    }

    /// Compute the average current over the curve.
    pub fn average_current_ua(&self) -> f64 {
        if self.points.is_empty() {
            return 0.0;
        }
        let sum: f64 = self.points.iter().map(|p| p.current_ua).sum();
        sum / self.points.len() as f64
    }

    /// Compute the current standard deviation.
    pub fn current_std_dev(&self) -> f64 {
        let n = self.points.len();
        if n < 2 {
            return 0.0;
        }
        let mean = self.average_current_ua();
        let variance: f64 = self.points
            .iter()
            .map(|p| (p.current_ua - mean) * (p.current_ua - mean))
            .sum::<f64>()
            / (n as f64 - 1.0);
        variance.sqrt()
    }
}

// ============================================================================
// Endpoint Detector
// ============================================================================

/// Biamperometric (or bivoltametric/photometric) endpoint detector.
///
/// Monitors the polarization current between dual platinum electrodes.
/// When excess iodine appears (all water consumed), the current drops
/// permanently below the threshold, signaling the endpoint.
#[derive(Debug, Clone)]
pub struct EndpointDetector {
    /// Detection mode.
    pub mode: EndpointMode,
    /// Current threshold in µA.
    pub threshold_ua: f64,
    /// Required persistence time in seconds.
    pub persistence_s: f64,
    /// Time when current first dropped below threshold (None = currently above).
    below_since: Option<f64>,
    /// Whether endpoint has been reached.
    reached: bool,
    /// Endpoint time if reached.
    endpoint_time: Option<f64>,
    /// Current readings history for filtering.
    history: Vec<(f64, f64)>,
    /// Number of readings to average for noise filtering.
    filter_window: usize,
}

impl EndpointDetector {
    /// Create a new endpoint detector.
    pub fn new(mode: EndpointMode, threshold_ua: f64, persistence_s: f64) -> Self {
        Self {
            mode,
            threshold_ua,
            persistence_s: persistence_s.max(MIN_ENDPOINT_PERSISTENCE_S),
            below_since: None,
            reached: false,
            endpoint_time: None,
            history: Vec::new(),
            filter_window: 5,
        }
    }

    /// Create from a KarlFischerConfig.
    pub fn from_config(config: &KarlFischerConfig) -> Self {
        Self::new(
            config.endpoint_mode,
            config.endpoint_threshold_ua,
            config.endpoint_persistence_s,
        )
    }

    /// Set the filter window size for noise smoothing.
    pub fn set_filter_window(&mut self, window: usize) {
        self.filter_window = window.max(1);
    }

    /// Process a new current reading.
    ///
    /// Returns `true` if the endpoint has been reached.
    pub fn process(&mut self, time_s: f64, current_ua: f64) -> bool {
        if self.reached {
            return true;
        }
        self.history.push((time_s, current_ua));

        // Apply moving average filter
        let filtered = self.filtered_current();

        if filtered <= self.threshold_ua {
            match self.below_since {
                None => {
                    self.below_since = Some(time_s);
                }
                Some(start) => {
                    if time_s - start >= self.persistence_s {
                        self.reached = true;
                        self.endpoint_time = Some(start);
                    }
                }
            }
        } else {
            self.below_since = None;
        }
        self.reached
    }

    /// Get the filtered (moving-average) current from recent history.
    fn filtered_current(&self) -> f64 {
        let n = self.history.len();
        if n == 0 {
            return 0.0;
        }
        let window = self.filter_window.min(n);
        let sum: f64 = self.history[n - window..]
            .iter()
            .map(|&(_, i)| i)
            .sum();
        sum / window as f64
    }

    /// Returns true if the endpoint has been reached.
    pub fn is_reached(&self) -> bool {
        self.reached
    }

    /// Returns the endpoint time if reached.
    pub fn endpoint_time(&self) -> Option<f64> {
        self.endpoint_time
    }

    /// Reset the detector for a new titration.
    pub fn reset(&mut self) {
        self.below_since = None;
        self.reached = false;
        self.endpoint_time = None;
        self.history.clear();
    }

    /// Return the number of readings processed.
    pub fn readings_count(&self) -> usize {
        self.history.len()
    }
}

// ============================================================================
// Drift Corrector
// ============================================================================

/// Estimates and corrects background moisture drift.
///
/// During pre-titration conditioning, background moisture slowly enters the
/// titration cell. The drift corrector estimates this rate and subtracts it
/// from the gross result.
#[derive(Debug, Clone)]
pub struct DriftCorrector {
    /// Drift correction strategy.
    pub strategy: DriftStrategy,
    /// Background readings: (time_s, cumulative_water_ug).
    readings: Vec<(f64, f64)>,
    /// Computed drift rate in µg/min.
    drift_rate: f64,
    /// Whether the drift rate has been computed.
    computed: bool,
    /// Exponential time constant for exponential model (minutes).
    exp_tau_min: f64,
    /// Initial drift rate for exponential model (µg/min).
    exp_initial_rate: f64,
}

impl DriftCorrector {
    /// Create a new drift corrector.
    pub fn new(strategy: DriftStrategy) -> Self {
        Self {
            strategy,
            readings: Vec::new(),
            drift_rate: 0.0,
            computed: false,
            exp_tau_min: 5.0,
            exp_initial_rate: 0.0,
        }
    }

    /// Add a background drift reading.
    pub fn add_reading(&mut self, time_s: f64, cumulative_water_ug: f64) {
        self.readings.push((time_s, cumulative_water_ug));
        self.computed = false;
    }

    /// Compute the drift rate from accumulated readings.
    pub fn compute(&mut self) -> f64 {
        match self.strategy {
            DriftStrategy::None => {
                self.drift_rate = 0.0;
            }
            DriftStrategy::Linear => {
                self.drift_rate = drift_rate(&self.readings);
            }
            DriftStrategy::Exponential => {
                self.compute_exponential();
            }
        }
        self.computed = true;
        self.drift_rate
    }

    /// Compute exponential drift model parameters.
    fn compute_exponential(&mut self) {
        if self.readings.len() < 2 {
            self.drift_rate = 0.0;
            return;
        }
        // Use the last two readings to estimate instantaneous rate
        let n = self.readings.len();
        let (t1, w1) = self.readings[n - 2];
        let (t2, w2) = self.readings[n - 1];
        let dt_min = (t2 - t1) / 60.0;
        if dt_min.abs() < 1e-12 {
            self.drift_rate = 0.0;
            return;
        }
        let current_rate = (w2 - w1) / dt_min;
        // Estimate initial rate from first readings
        if n >= 3 {
            let (t0, w0) = self.readings[0];
            let (t1b, w1b) = self.readings[1];
            let dt0_min = (t1b - t0) / 60.0;
            if dt0_min.abs() > 1e-12 {
                self.exp_initial_rate = (w1b - w0) / dt0_min;
            }
        }
        self.drift_rate = current_rate;
    }

    /// Get the computed drift rate in µg/min.
    pub fn get_drift_rate(&self) -> f64 {
        self.drift_rate
    }

    /// Correct a gross water measurement for drift.
    ///
    /// Returns the net water in mg.
    pub fn correct(&self, gross_water_mg: f64, elapsed_min: f64) -> f64 {
        match self.strategy {
            DriftStrategy::None => gross_water_mg,
            DriftStrategy::Linear => {
                net_water(gross_water_mg, self.drift_rate, elapsed_min)
            }
            DriftStrategy::Exponential => {
                // Integral of exponential drift: R0 × τ × (1 - e^(-t/τ))
                let integral_ug = if self.exp_tau_min > 0.0 {
                    self.drift_rate * self.exp_tau_min
                        * (1.0 - (-elapsed_min / self.exp_tau_min).exp())
                } else {
                    self.drift_rate * elapsed_min
                };
                let net = gross_water_mg - integral_ug / 1000.0;
                if net < 0.0 { 0.0 } else { net }
            }
        }
    }

    /// Check whether drift rate is within acceptable limits.
    pub fn is_stable(&self, max_drift_ug_per_min: f64) -> bool {
        self.drift_rate.abs() <= max_drift_ug_per_min
    }

    /// Reset the drift corrector.
    pub fn reset(&mut self) {
        self.readings.clear();
        self.drift_rate = 0.0;
        self.computed = false;
    }

    /// Number of readings collected.
    pub fn readings_count(&self) -> usize {
        self.readings.len()
    }
}

// ============================================================================
// Moisture Calculation
// ============================================================================

/// Result of a moisture determination.
#[derive(Debug, Clone)]
pub struct MoistureResult {
    /// Gross water found (mg).
    pub gross_water_mg: f64,
    /// Net water after drift correction (mg).
    pub net_water_mg: f64,
    /// Drift rate used for correction (µg/min).
    pub drift_rate_ug_per_min: f64,
    /// Elapsed titration time (minutes).
    pub elapsed_min: f64,
    /// Sample weight (g).
    pub sample_g: f64,
    /// Moisture content in ppm.
    pub ppm: f64,
    /// Moisture content in weight percent.
    pub percent: f64,
    /// Method used.
    pub method: TitrationMethod,
}

/// Performs moisture content calculations from titration data.
#[derive(Debug, Clone)]
pub struct MoistureCalculation {
    config: KarlFischerConfig,
}

impl MoistureCalculation {
    /// Create a new moisture calculation engine.
    pub fn new(config: KarlFischerConfig) -> Self {
        Self { config }
    }

    /// Calculate moisture from coulometric titration data.
    pub fn from_coulometric(
        &self,
        charge_c: f64,
        sample_g: f64,
        drift_rate_ug_per_min: f64,
        elapsed_min: f64,
    ) -> MoistureResult {
        let gross_mg = coulombs_to_water_mass(charge_c);
        let net_mg = net_water(gross_mg, drift_rate_ug_per_min, elapsed_min);
        MoistureResult {
            gross_water_mg: gross_mg,
            net_water_mg: net_mg,
            drift_rate_ug_per_min,
            elapsed_min,
            sample_g,
            ppm: moisture_ppm(net_mg, sample_g),
            percent: moisture_percent(net_mg, sample_g),
            method: TitrationMethod::Coulometric,
        }
    }

    /// Calculate moisture from volumetric titration data.
    pub fn from_volumetric(
        &self,
        volume_ml: f64,
        sample_g: f64,
        drift_rate_ug_per_min: f64,
        elapsed_min: f64,
    ) -> MoistureResult {
        let conc = self.config.reagent_type.concentration_mg_per_ml();
        let gross_mg = volume_to_water_mass(volume_ml, conc);
        let net_mg = net_water(gross_mg, drift_rate_ug_per_min, elapsed_min);
        MoistureResult {
            gross_water_mg: gross_mg,
            net_water_mg: net_mg,
            drift_rate_ug_per_min,
            elapsed_min,
            sample_g,
            ppm: moisture_ppm(net_mg, sample_g),
            percent: moisture_percent(net_mg, sample_g),
            method: TitrationMethod::Volumetric,
        }
    }

    /// Calculate from a completed titration curve.
    pub fn from_curve(
        &self,
        curve: &TitrationCurve,
        sample_g: f64,
        drift_rate_ug_per_min: f64,
    ) -> MoistureResult {
        let elapsed_min = curve.elapsed_s() / 60.0;
        match self.config.method {
            TitrationMethod::Coulometric => {
                self.from_coulometric(
                    curve.total_charge_c(),
                    sample_g,
                    drift_rate_ug_per_min,
                    elapsed_min,
                )
            }
            TitrationMethod::Volumetric => {
                self.from_volumetric(
                    curve.total_volume_ml(),
                    sample_g,
                    drift_rate_ug_per_min,
                    elapsed_min,
                )
            }
        }
    }
}

// ============================================================================
// Coulometric Titrator
// ============================================================================

/// Coulometric Karl Fischer titrator.
///
/// Generates iodine electrochemically at the generator electrode.
/// The charge consumed (coulombs) is directly proportional to the amount
/// of water titrated via Faraday's law.
#[derive(Debug, Clone)]
pub struct CoulometricTitrator {
    config: KarlFischerConfig,
    /// Generator electrode current in amperes.
    generator_current_a: f64,
    /// Cumulative charge in coulombs.
    total_charge_c: f64,
    /// Elapsed time in seconds.
    elapsed_s: f64,
    /// Endpoint detector.
    endpoint: EndpointDetector,
    /// Drift corrector.
    drift: DriftCorrector,
    /// Titration curve recorder.
    curve: TitrationCurve,
    /// Whether titration is active.
    active: bool,
    /// Last timestamp for integration.
    last_time_s: f64,
}

impl CoulometricTitrator {
    /// Create a new coulometric titrator with default generator current (400 mA).
    pub fn new(config: KarlFischerConfig) -> Self {
        let endpoint = EndpointDetector::from_config(&config);
        let drift = DriftCorrector::new(config.drift_strategy);
        Self {
            config,
            generator_current_a: 0.400,
            total_charge_c: 0.0,
            elapsed_s: 0.0,
            endpoint,
            drift,
            curve: TitrationCurve::new(),
            active: false,
            last_time_s: 0.0,
        }
    }

    /// Set the generator electrode current.
    pub fn set_generator_current(&mut self, current_a: f64) {
        self.generator_current_a = current_a.abs();
    }

    /// Start the titration.
    pub fn start(&mut self) {
        self.active = true;
        self.total_charge_c = 0.0;
        self.elapsed_s = 0.0;
        self.last_time_s = 0.0;
        self.endpoint.reset();
        self.curve = TitrationCurve::new();
    }

    /// Process a time step: integrate current and check endpoint.
    ///
    /// `time_s` - current time in seconds
    /// `indicator_current_ua` - current measured at indicator electrode (µA)
    ///
    /// Returns true if endpoint reached.
    pub fn process(&mut self, time_s: f64, indicator_current_ua: f64) -> bool {
        if !self.active {
            return false;
        }
        // Integrate charge: Q += I × Δt
        if self.last_time_s > 0.0 || self.elapsed_s > 0.0 {
            let dt = time_s - self.last_time_s;
            if dt > 0.0 {
                self.total_charge_c += self.generator_current_a * dt;
                self.elapsed_s = time_s;
            }
        } else {
            self.elapsed_s = time_s;
        }
        self.last_time_s = time_s;

        // Record point
        self.curve.add_point(TitrationPoint {
            time_s,
            current_ua: indicator_current_ua,
            volume_ml: 0.0,
            charge_c: self.total_charge_c,
        });

        // Check endpoint
        if self.endpoint.process(time_s, indicator_current_ua) {
            self.active = false;
            return true;
        }
        false
    }

    /// Get the water mass determined so far (mg).
    pub fn water_mg(&self) -> f64 {
        coulombs_to_water_mass(self.total_charge_c)
    }

    /// Get the total charge consumed (coulombs).
    pub fn total_charge(&self) -> f64 {
        self.total_charge_c
    }

    /// Get the elapsed time in seconds.
    pub fn elapsed_s(&self) -> f64 {
        self.elapsed_s
    }

    /// Calculate the final moisture result.
    pub fn result(&self, sample_g: f64) -> MoistureResult {
        let calc = MoistureCalculation::new(self.config.clone());
        let dr = self.drift.get_drift_rate();
        let elapsed_min = self.elapsed_s / 60.0;
        calc.from_coulometric(self.total_charge_c, sample_g, dr, elapsed_min)
    }

    /// Get a reference to the titration curve.
    pub fn curve(&self) -> &TitrationCurve {
        &self.curve
    }

    /// Get a reference to the drift corrector.
    pub fn drift(&self) -> &DriftCorrector {
        &self.drift
    }

    /// Get a mutable reference to the drift corrector.
    pub fn drift_mut(&mut self) -> &mut DriftCorrector {
        &mut self.drift
    }

    /// Check if titration is currently active.
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Reset the titrator for a new determination.
    pub fn reset(&mut self) {
        self.total_charge_c = 0.0;
        self.elapsed_s = 0.0;
        self.last_time_s = 0.0;
        self.active = false;
        self.endpoint.reset();
        self.drift.reset();
        self.curve = TitrationCurve::new();
    }
}

// ============================================================================
// Volumetric Titrator
// ============================================================================

/// Volumetric Karl Fischer titrator.
///
/// Dispenses reagent from a motorized burette. The volume of reagent consumed,
/// multiplied by its concentration, gives the water mass.
#[derive(Debug, Clone)]
pub struct VolumetricTitrator {
    config: KarlFischerConfig,
    /// Burette resolution (mL per step).
    burette_resolution_ml: f64,
    /// Maximum burette volume (mL).
    burette_max_ml: f64,
    /// Total volume dispensed (mL).
    total_volume_ml: f64,
    /// Elapsed time in seconds.
    elapsed_s: f64,
    /// Endpoint detector.
    endpoint: EndpointDetector,
    /// Drift corrector.
    drift: DriftCorrector,
    /// Titration curve recorder.
    curve: TitrationCurve,
    /// Whether titration is active.
    active: bool,
    /// Last timestamp.
    last_time_s: f64,
    /// Dispensing rate (mL/s). Decreases as endpoint approaches.
    dispense_rate_ml_s: f64,
}

impl VolumetricTitrator {
    /// Create a new volumetric titrator.
    pub fn new(config: KarlFischerConfig) -> Self {
        let endpoint = EndpointDetector::from_config(&config);
        let drift = DriftCorrector::new(config.drift_strategy);
        Self {
            config,
            burette_resolution_ml: 0.001, // 1 µL
            burette_max_ml: 5.0,
            total_volume_ml: 0.0,
            elapsed_s: 0.0,
            endpoint,
            drift,
            curve: TitrationCurve::new(),
            active: false,
            last_time_s: 0.0,
            dispense_rate_ml_s: 0.1,
        }
    }

    /// Set the burette resolution.
    pub fn set_burette_resolution(&mut self, resolution_ml: f64) {
        self.burette_resolution_ml = resolution_ml.abs();
    }

    /// Set the maximum burette volume.
    pub fn set_burette_max(&mut self, max_ml: f64) {
        self.burette_max_ml = max_ml.abs();
    }

    /// Start the titration.
    pub fn start(&mut self) {
        self.active = true;
        self.total_volume_ml = 0.0;
        self.elapsed_s = 0.0;
        self.last_time_s = 0.0;
        self.endpoint.reset();
        self.curve = TitrationCurve::new();
    }

    /// Dispense a volume increment and check endpoint.
    ///
    /// Returns true if endpoint reached.
    pub fn process(
        &mut self,
        time_s: f64,
        volume_increment_ml: f64,
        indicator_current_ua: f64,
    ) -> bool {
        if !self.active {
            return false;
        }
        self.total_volume_ml += volume_increment_ml;
        self.elapsed_s = time_s;
        self.last_time_s = time_s;

        self.curve.add_point(TitrationPoint {
            time_s,
            current_ua: indicator_current_ua,
            volume_ml: self.total_volume_ml,
            charge_c: 0.0,
        });

        if self.endpoint.process(time_s, indicator_current_ua) {
            self.active = false;
            return true;
        }
        false
    }

    /// Get the water mass determined so far (mg).
    pub fn water_mg(&self) -> f64 {
        volume_to_water_mass(
            self.total_volume_ml,
            self.config.reagent_type.concentration_mg_per_ml(),
        )
    }

    /// Get the total volume dispensed (mL).
    pub fn total_volume(&self) -> f64 {
        self.total_volume_ml
    }

    /// Get the elapsed time in seconds.
    pub fn elapsed_s(&self) -> f64 {
        self.elapsed_s
    }

    /// Calculate the final moisture result.
    pub fn result(&self, sample_g: f64) -> MoistureResult {
        let calc = MoistureCalculation::new(self.config.clone());
        let dr = self.drift.get_drift_rate();
        let elapsed_min = self.elapsed_s / 60.0;
        calc.from_volumetric(self.total_volume_ml, sample_g, dr, elapsed_min)
    }

    /// Get a reference to the titration curve.
    pub fn curve(&self) -> &TitrationCurve {
        &self.curve
    }

    /// Get a mutable reference to the drift corrector.
    pub fn drift_mut(&mut self) -> &mut DriftCorrector {
        &mut self.drift
    }

    /// Check if titration is active.
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Remaining burette volume in mL.
    pub fn remaining_volume(&self) -> f64 {
        (self.burette_max_ml - self.total_volume_ml).max(0.0)
    }

    /// Reset for a new determination.
    pub fn reset(&mut self) {
        self.total_volume_ml = 0.0;
        self.elapsed_s = 0.0;
        self.last_time_s = 0.0;
        self.active = false;
        self.endpoint.reset();
        self.drift.reset();
        self.curve = TitrationCurve::new();
    }
}

// ============================================================================
// Oven Method
// ============================================================================

/// Oven method for indirect Karl Fischer determination of solids.
///
/// The sample is heated in an oven, and a dry carrier gas (nitrogen) sweeps
/// the released moisture into the titration cell. The oven temperature program
/// and gas flow rate affect the release kinetics.
#[derive(Debug, Clone)]
pub struct OvenMethod {
    /// Temperature program.
    pub program: OvenProgram,
    /// Carrier gas flow rate (mL/min).
    pub gas_flow_ml_per_min: f64,
    /// Sample weight (g).
    pub sample_g: f64,
    /// Current temperature (°C).
    current_temp_c: f64,
    /// Elapsed time (minutes).
    elapsed_min: f64,
    /// Cumulative water released (mg).
    cumulative_water_mg: f64,
    /// Time-water release profile: (time_min, water_mg).
    release_profile: Vec<(f64, f64)>,
    /// Whether the oven run is complete.
    complete: bool,
}

impl OvenMethod {
    /// Create a new oven method determination.
    pub fn new(program: OvenProgram, sample_g: f64) -> Self {
        let initial_temp = match program {
            OvenProgram::Isothermal(t) => t,
            OvenProgram::Ramp { start_c, .. } => start_c,
            OvenProgram::Step { temperatures_c, .. } => temperatures_c[0],
        };
        Self {
            program,
            gas_flow_ml_per_min: 200.0,
            sample_g,
            current_temp_c: initial_temp,
            elapsed_min: 0.0,
            cumulative_water_mg: 0.0,
            release_profile: Vec::new(),
            complete: false,
        }
    }

    /// Set the carrier gas flow rate.
    pub fn set_gas_flow(&mut self, flow_ml_per_min: f64) {
        self.gas_flow_ml_per_min = flow_ml_per_min.abs();
    }

    /// Update the oven state at a given time.
    /// `water_mg` is the incremental water detected by the KF cell.
    pub fn update(&mut self, time_min: f64, water_mg: f64) {
        self.elapsed_min = time_min;
        self.cumulative_water_mg += water_mg;
        self.release_profile.push((time_min, self.cumulative_water_mg));

        // Update temperature based on program
        self.current_temp_c = self.temperature_at(time_min);
    }

    /// Get the oven temperature at a given time (minutes).
    pub fn temperature_at(&self, time_min: f64) -> f64 {
        match self.program {
            OvenProgram::Isothermal(t) => t,
            OvenProgram::Ramp { start_c, end_c, duration_min } => {
                if duration_min <= 0.0 {
                    return end_c;
                }
                let frac = (time_min / duration_min).min(1.0).max(0.0);
                start_c + frac * (end_c - start_c)
            }
            OvenProgram::Step { temperatures_c, durations_min, num_steps } => {
                let mut cum_time = 0.0;
                for i in 0..num_steps.min(4) {
                    cum_time += durations_min[i];
                    if time_min <= cum_time {
                        return temperatures_c[i];
                    }
                }
                // After all steps, hold at last temperature
                if num_steps > 0 {
                    temperatures_c[num_steps.min(4) - 1]
                } else {
                    25.0
                }
            }
        }
    }

    /// Mark the oven run as complete.
    pub fn finish(&mut self) {
        self.complete = true;
    }

    /// Get the total water released (mg).
    pub fn total_water_mg(&self) -> f64 {
        self.cumulative_water_mg
    }

    /// Get the moisture result.
    pub fn moisture_ppm(&self) -> f64 {
        moisture_ppm(self.cumulative_water_mg, self.sample_g)
    }

    /// Get the moisture percent.
    pub fn moisture_percent(&self) -> f64 {
        moisture_percent(self.cumulative_water_mg, self.sample_g)
    }

    /// Get the current temperature.
    pub fn current_temp(&self) -> f64 {
        self.current_temp_c
    }

    /// Get the elapsed time in minutes.
    pub fn elapsed_min(&self) -> f64 {
        self.elapsed_min
    }

    /// Get the release profile.
    pub fn release_profile(&self) -> &[(f64, f64)] {
        &self.release_profile
    }

    /// Determine if the moisture release has plateaued (< threshold µg/min for duration).
    pub fn is_plateau(&self, threshold_ug_per_min: f64, duration_min: f64) -> bool {
        let profile = &self.release_profile;
        if profile.len() < 3 {
            return false;
        }
        let n = profile.len();
        // Check if recent rates are below threshold
        let (t_end, w_end) = profile[n - 1];
        // Find the point `duration_min` ago
        let t_start = t_end - duration_min;
        if t_start < 0.0 {
            return false;
        }
        // Find closest point at or before t_start
        let mut start_idx = 0;
        for (i, &(t, _)) in profile.iter().enumerate() {
            if t >= t_start {
                start_idx = i;
                break;
            }
        }
        let (t_s, w_s) = profile[start_idx];
        let dt = t_end - t_s;
        if dt < 1e-6 {
            return false;
        }
        let rate_mg_per_min = (w_end - w_s) / dt;
        let rate_ug_per_min = rate_mg_per_min * 1000.0;
        rate_ug_per_min.abs() <= threshold_ug_per_min
    }

    /// Whether the run is complete.
    pub fn is_complete(&self) -> bool {
        self.complete
    }
}

// ============================================================================
// Statistics and Validation
// ============================================================================

/// Statistics from a series of replicate moisture determinations.
#[derive(Debug, Clone)]
pub struct ReplicateStatistics {
    /// Number of replicates.
    pub n: usize,
    /// Mean moisture (ppm).
    pub mean_ppm: f64,
    /// Standard deviation (ppm).
    pub std_dev_ppm: f64,
    /// Relative standard deviation (%).
    pub rsd_percent: f64,
    /// Minimum value (ppm).
    pub min_ppm: f64,
    /// Maximum value (ppm).
    pub max_ppm: f64,
    /// Range (ppm).
    pub range_ppm: f64,
}

/// Compute statistics from a series of replicate moisture measurements.
pub fn replicate_statistics(values_ppm: &[f64]) -> ReplicateStatistics {
    let n = values_ppm.len();
    if n == 0 {
        return ReplicateStatistics {
            n: 0,
            mean_ppm: 0.0,
            std_dev_ppm: 0.0,
            rsd_percent: 0.0,
            min_ppm: 0.0,
            max_ppm: 0.0,
            range_ppm: 0.0,
        };
    }
    let sum: f64 = values_ppm.iter().sum();
    let mean = sum / n as f64;
    let min = values_ppm.iter().cloned().fold(f64::MAX, f64::min);
    let max = values_ppm.iter().cloned().fold(f64::MIN, f64::max);

    let std_dev = if n > 1 {
        let var: f64 = values_ppm.iter().map(|&x| (x - mean) * (x - mean)).sum::<f64>()
            / (n as f64 - 1.0);
        var.sqrt()
    } else {
        0.0
    };

    let rsd = if mean.abs() > 1e-15 {
        (std_dev / mean) * 100.0
    } else {
        0.0
    };

    ReplicateStatistics {
        n,
        mean_ppm: mean,
        std_dev_ppm: std_dev,
        rsd_percent: rsd,
        min_ppm: min,
        max_ppm: max,
        range_ppm: max - min,
    }
}

/// Check whether a measurement passes ASTM repeatability criteria.
///
/// ASTM E203 specifies that the range of duplicate determinations should
/// not exceed a fraction of the mean (typically 5-10% RSD for low moisture).
pub fn passes_repeatability(values_ppm: &[f64], max_rsd_percent: f64) -> bool {
    let stats = replicate_statistics(values_ppm);
    if stats.n < 2 {
        return true; // Single measurement always passes
    }
    stats.rsd_percent <= max_rsd_percent
}

/// Calculate the blank correction value from a series of blank titrations.
///
/// Returns the average blank in mg H₂O.
pub fn blank_correction(blank_water_mg: &[f64]) -> f64 {
    if blank_water_mg.is_empty() {
        return 0.0;
    }
    let sum: f64 = blank_water_mg.iter().sum();
    sum / blank_water_mg.len() as f64
}

/// Apply blank correction to a moisture measurement.
pub fn apply_blank(gross_water_mg: f64, blank_mg: f64) -> f64 {
    let net = gross_water_mg - blank_mg;
    if net < 0.0 { 0.0 } else { net }
}

/// Compute the reagent titer (actual concentration) from a water standard check.
///
/// titer = (known_water_mg / measured_volume_ml) / nominal_concentration
pub fn compute_titer(
    known_water_mg: f64,
    measured_volume_ml: f64,
    nominal_concentration: f64,
) -> f64 {
    if measured_volume_ml.abs() < 1e-12 || nominal_concentration.abs() < 1e-12 {
        return 1.0;
    }
    let actual_conc = known_water_mg / measured_volume_ml;
    actual_conc / nominal_concentration
}

/// Apply titer correction to a volumetric result.
pub fn apply_titer(water_mg: f64, titer: f64) -> f64 {
    water_mg * titer
}

/// Estimate the expected titration time for a given sample.
///
/// For coulometric: t(s) ≈ Q(C) / I(A)
/// For volumetric: t(s) ≈ V(mL) / dispensing_rate(mL/s)
pub fn estimated_titration_time_s(
    expected_water_mg: f64,
    method: TitrationMethod,
    current_a_or_rate_ml_s: f64,
) -> f64 {
    if current_a_or_rate_ml_s <= 0.0 {
        return 0.0;
    }
    match method {
        TitrationMethod::Coulometric => {
            let q = water_mass_to_coulombs(expected_water_mg);
            q / current_a_or_rate_ml_s
        }
        TitrationMethod::Volumetric => {
            // Assume standard two-component reagent
            let volume_ml = expected_water_mg / STANDARD_REAGENT_TWO_COMPONENT;
            volume_ml / current_a_or_rate_ml_s
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-6;
    const LOOSE_TOL: f64 = 1e-3;

    // -------------------------------------------------------------------
    // Physical constants and conversions
    // -------------------------------------------------------------------

    #[test]
    fn test_faraday_constant() {
        assert!((FARADAY_CONSTANT - 96485.3329).abs() < 0.001);
    }

    #[test]
    fn test_water_molar_mass() {
        assert!((WATER_MOLAR_MASS - 18.01528).abs() < 0.0001);
    }

    #[test]
    fn test_coulombs_per_mg_water_derived() {
        // Q/mg = 2F / (M × 1000) should be ~10.722 C/mg
        let expected = 2.0 * 96485.3329 / (18.01528 * 1000.0);
        assert!((COULOMBS_PER_MG_WATER - expected).abs() < 1e-6);
    }

    #[test]
    fn test_coulombs_to_water_mass_zero() {
        assert!((coulombs_to_water_mass(0.0) - 0.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_coulombs_to_water_mass_one_mole() {
        // 2 × 96485 C should give 18015.28 mg (1 mole) of water
        let q = 2.0 * FARADAY_CONSTANT;
        let mass = coulombs_to_water_mass(q);
        assert!((mass - 18015.28).abs() < 0.1);
    }

    #[test]
    fn test_coulombs_to_water_mass_small() {
        // 10.722 C ≈ 1 mg H₂O
        let mass = coulombs_to_water_mass(COULOMBS_PER_MG_WATER);
        assert!((mass - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_water_mass_to_coulombs_roundtrip() {
        let original_mg = 5.0;
        let q = water_mass_to_coulombs(original_mg);
        let recovered = coulombs_to_water_mass(q);
        assert!((recovered - original_mg).abs() < TOLERANCE);
    }

    #[test]
    fn test_volume_to_water_mass() {
        // 2 mL × 5 mg/mL = 10 mg
        let mass = volume_to_water_mass(2.0, 5.0);
        assert!((mass - 10.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_volume_to_water_mass_zero() {
        assert!((volume_to_water_mass(0.0, 5.0) - 0.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_volume_to_water_mass_one_component() {
        let mass = volume_to_water_mass(1.0, STANDARD_REAGENT_ONE_COMPONENT);
        assert!((mass - 1.0).abs() < TOLERANCE);
    }

    // -------------------------------------------------------------------
    // Moisture ppm and percent
    // -------------------------------------------------------------------

    #[test]
    fn test_moisture_ppm_basic() {
        // 1 mg water in 1 g sample = 1000 ppm
        let ppm = moisture_ppm(1.0, 1.0);
        assert!((ppm - 1000.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_moisture_ppm_typical() {
        // 0.1 mg water in 10 g sample = 10 ppm
        let ppm = moisture_ppm(0.1, 10.0);
        assert!((ppm - 10.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_moisture_ppm_zero_sample() {
        assert!((moisture_ppm(1.0, 0.0) - 0.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_moisture_ppm_negative_sample() {
        assert!((moisture_ppm(1.0, -1.0) - 0.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_moisture_percent_basic() {
        // 10 mg water in 1 g sample = 1%
        let pct = moisture_percent(10.0, 1.0);
        assert!((pct - 1.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_moisture_percent_high() {
        // 500 mg water in 1 g sample = 50%
        let pct = moisture_percent(500.0, 1.0);
        assert!((pct - 50.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_moisture_percent_zero_sample() {
        assert!((moisture_percent(1.0, 0.0) - 0.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_ppm_to_percent() {
        assert!((ppm_to_percent(10000.0) - 1.0).abs() < TOLERANCE);
        assert!((ppm_to_percent(100.0) - 0.01).abs() < TOLERANCE);
    }

    #[test]
    fn test_percent_to_ppm() {
        assert!((percent_to_ppm(1.0) - 10000.0).abs() < TOLERANCE);
        assert!((percent_to_ppm(0.01) - 100.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_ppm_percent_roundtrip() {
        let ppm = 500.0;
        let pct = ppm_to_percent(ppm);
        let back = percent_to_ppm(pct);
        assert!((back - ppm).abs() < TOLERANCE);
    }

    // -------------------------------------------------------------------
    // Drift rate
    // -------------------------------------------------------------------

    #[test]
    fn test_drift_rate_two_points() {
        // 0 µg at 0s, 60 µg at 60s → 60 µg/min
        let readings = [(0.0, 0.0), (60.0, 60.0)];
        let rate = drift_rate(&readings);
        assert!((rate - 60.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_drift_rate_constant() {
        // Constant 10 µg/min over 3 minutes
        let readings = [
            (0.0, 0.0),
            (60.0, 10.0),
            (120.0, 20.0),
            (180.0, 30.0),
        ];
        let rate = drift_rate(&readings);
        assert!((rate - 10.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_drift_rate_single_point() {
        let readings = [(0.0, 0.0)];
        let rate = drift_rate(&readings);
        assert!((rate - 0.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_drift_rate_empty() {
        let readings: [(f64, f64); 0] = [];
        let rate = drift_rate(&readings);
        assert!((rate - 0.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_drift_rate_negative() {
        // Decreasing drift (impossible physically but tests math)
        let readings = [(0.0, 100.0), (60.0, 50.0)];
        let rate = drift_rate(&readings);
        assert!(rate < 0.0);
    }

    // -------------------------------------------------------------------
    // Net water
    // -------------------------------------------------------------------

    #[test]
    fn test_net_water_no_drift() {
        let net = net_water(5.0, 0.0, 10.0);
        assert!((net - 5.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_net_water_with_drift() {
        // 5 mg gross, 10 µg/min drift, 5 min → drift = 0.05 mg
        let net = net_water(5.0, 10.0, 5.0);
        assert!((net - 4.95).abs() < TOLERANCE);
    }

    #[test]
    fn test_net_water_clamps_zero() {
        // More drift than water → clamped to 0
        let net = net_water(0.01, 100.0, 5.0);
        assert!((net - 0.0).abs() < TOLERANCE);
    }

    // -------------------------------------------------------------------
    // Endpoint detection function
    // -------------------------------------------------------------------

    #[test]
    fn test_endpoint_reached_true() {
        assert!(endpoint_reached(5.0, 10.0, 20.0));
    }

    #[test]
    fn test_endpoint_reached_current_too_high() {
        assert!(!endpoint_reached(15.0, 10.0, 20.0));
    }

    #[test]
    fn test_endpoint_reached_duration_too_short() {
        assert!(!endpoint_reached(5.0, 10.0, 3.0));
    }

    #[test]
    fn test_endpoint_reached_exact_threshold() {
        assert!(endpoint_reached(10.0, 10.0, 5.0));
    }

    // -------------------------------------------------------------------
    // Reagent type
    // -------------------------------------------------------------------

    #[test]
    fn test_reagent_one_component() {
        assert!((ReagentType::OneComponent.concentration_mg_per_ml() - 1.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_reagent_two_component() {
        assert!((ReagentType::TwoComponent.concentration_mg_per_ml() - 5.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_reagent_custom() {
        let r = ReagentType::Custom(3.5);
        assert!((r.concentration_mg_per_ml() - 3.5).abs() < TOLERANCE);
    }

    // -------------------------------------------------------------------
    // KarlFischerConfig
    // -------------------------------------------------------------------

    #[test]
    fn test_config_default() {
        let cfg = KarlFischerConfig::default();
        assert_eq!(cfg.method, TitrationMethod::Coulometric);
        assert!((cfg.endpoint_threshold_ua - 20.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_config_coulometric() {
        let cfg = KarlFischerConfig::coulometric();
        assert_eq!(cfg.method, TitrationMethod::Coulometric);
        assert!((cfg.endpoint_threshold_ua - 10.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_config_volumetric() {
        let cfg = KarlFischerConfig::volumetric();
        assert_eq!(cfg.method, TitrationMethod::Volumetric);
        assert_eq!(cfg.reagent_type, ReagentType::TwoComponent);
    }

    #[test]
    fn test_config_petroleum() {
        let cfg = KarlFischerConfig::petroleum();
        assert_eq!(cfg.method, TitrationMethod::Coulometric);
        assert!((cfg.endpoint_persistence_s - 30.0).abs() < TOLERANCE);
    }

    // -------------------------------------------------------------------
    // TitrationCurve
    // -------------------------------------------------------------------

    #[test]
    fn test_curve_empty() {
        let curve = TitrationCurve::new();
        assert!((curve.elapsed_s() - 0.0).abs() < TOLERANCE);
        assert!((curve.total_charge_c() - 0.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_curve_add_points() {
        let mut curve = TitrationCurve::new();
        curve.add_point(TitrationPoint { time_s: 0.0, current_ua: 100.0, volume_ml: 0.0, charge_c: 0.0 });
        curve.add_point(TitrationPoint { time_s: 10.0, current_ua: 50.0, volume_ml: 0.0, charge_c: 5.0 });
        assert_eq!(curve.points.len(), 2);
        assert!((curve.elapsed_s() - 10.0).abs() < TOLERANCE);
        assert!((curve.total_charge_c() - 5.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_curve_min_max_current() {
        let mut curve = TitrationCurve::new();
        curve.add_point(TitrationPoint { time_s: 0.0, current_ua: 100.0, volume_ml: 0.0, charge_c: 0.0 });
        curve.add_point(TitrationPoint { time_s: 5.0, current_ua: 200.0, volume_ml: 0.0, charge_c: 2.0 });
        curve.add_point(TitrationPoint { time_s: 10.0, current_ua: 50.0, volume_ml: 0.0, charge_c: 5.0 });
        assert!((curve.min_current_ua() - 50.0).abs() < TOLERANCE);
        assert!((curve.max_current_ua() - 200.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_curve_average_current() {
        let mut curve = TitrationCurve::new();
        curve.add_point(TitrationPoint { time_s: 0.0, current_ua: 10.0, volume_ml: 0.0, charge_c: 0.0 });
        curve.add_point(TitrationPoint { time_s: 1.0, current_ua: 20.0, volume_ml: 0.0, charge_c: 0.0 });
        curve.add_point(TitrationPoint { time_s: 2.0, current_ua: 30.0, volume_ml: 0.0, charge_c: 0.0 });
        assert!((curve.average_current_ua() - 20.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_curve_current_std_dev() {
        let mut curve = TitrationCurve::new();
        for v in &[10.0, 20.0, 30.0] {
            curve.add_point(TitrationPoint { time_s: 0.0, current_ua: *v, volume_ml: 0.0, charge_c: 0.0 });
        }
        assert!(curve.current_std_dev() > 0.0);
    }

    #[test]
    fn test_curve_derivative() {
        let mut curve = TitrationCurve::new();
        curve.add_point(TitrationPoint { time_s: 0.0, current_ua: 100.0, volume_ml: 0.0, charge_c: 0.0 });
        curve.add_point(TitrationPoint { time_s: 1.0, current_ua: 80.0, volume_ml: 0.0, charge_c: 0.0 });
        curve.add_point(TitrationPoint { time_s: 2.0, current_ua: 60.0, volume_ml: 0.0, charge_c: 0.0 });
        let deriv = curve.derivative_current_time();
        assert_eq!(deriv.len(), 2);
        // dI/dt = -20 µA/s
        assert!((deriv[0].1 - (-20.0)).abs() < TOLERANCE);
    }

    #[test]
    fn test_curve_inflection_points() {
        let mut curve = TitrationCurve::new();
        // Current decreasing then increasing → inflection
        curve.add_point(TitrationPoint { time_s: 0.0, current_ua: 100.0, volume_ml: 0.0, charge_c: 0.0 });
        curve.add_point(TitrationPoint { time_s: 1.0, current_ua: 80.0, volume_ml: 0.0, charge_c: 0.0 });
        curve.add_point(TitrationPoint { time_s: 2.0, current_ua: 50.0, volume_ml: 0.0, charge_c: 0.0 });
        curve.add_point(TitrationPoint { time_s: 3.0, current_ua: 70.0, volume_ml: 0.0, charge_c: 0.0 });
        let infl = curve.inflection_points();
        assert!(!infl.is_empty());
    }

    #[test]
    fn test_curve_find_endpoint_time() {
        let mut curve = TitrationCurve::new();
        // High current, then drops below threshold for >5s
        for i in 0..20 {
            let t = i as f64;
            let current = if t < 5.0 { 100.0 } else { 5.0 };
            curve.add_point(TitrationPoint { time_s: t, current_ua: current, volume_ml: 0.0, charge_c: 0.0 });
        }
        let ep = curve.find_endpoint_time(10.0, 5.0);
        assert!(ep.is_some());
        assert!((ep.unwrap() - 5.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_curve_no_endpoint() {
        let mut curve = TitrationCurve::new();
        for i in 0..10 {
            curve.add_point(TitrationPoint { time_s: i as f64, current_ua: 100.0, volume_ml: 0.0, charge_c: 0.0 });
        }
        assert!(curve.find_endpoint_time(10.0, 5.0).is_none());
    }

    #[test]
    fn test_curve_total_volume() {
        let mut curve = TitrationCurve::new();
        curve.add_point(TitrationPoint { time_s: 0.0, current_ua: 100.0, volume_ml: 0.0, charge_c: 0.0 });
        curve.add_point(TitrationPoint { time_s: 10.0, current_ua: 50.0, volume_ml: 2.5, charge_c: 0.0 });
        assert!((curve.total_volume_ml() - 2.5).abs() < TOLERANCE);
    }

    // -------------------------------------------------------------------
    // EndpointDetector
    // -------------------------------------------------------------------

    #[test]
    fn test_endpoint_detector_basic() {
        let mut det = EndpointDetector::new(EndpointMode::Biamperometric, 10.0, 5.0);
        assert!(!det.is_reached());
        assert_eq!(det.endpoint_time(), None);
    }

    #[test]
    fn test_endpoint_detector_process_above_threshold() {
        let mut det = EndpointDetector::new(EndpointMode::Biamperometric, 10.0, 5.0);
        det.set_filter_window(1);
        for i in 0..10 {
            assert!(!det.process(i as f64, 50.0));
        }
    }

    #[test]
    fn test_endpoint_detector_process_reaches() {
        let mut det = EndpointDetector::new(EndpointMode::Biamperometric, 10.0, 5.0);
        det.set_filter_window(1);
        // Start above, then drop below
        det.process(0.0, 50.0);
        det.process(1.0, 50.0);
        det.process(2.0, 5.0);
        det.process(3.0, 5.0);
        det.process(4.0, 5.0);
        det.process(5.0, 5.0);
        det.process(6.0, 5.0);
        let reached = det.process(7.0, 5.0);
        assert!(reached);
        assert!(det.is_reached());
        assert!(det.endpoint_time().is_some());
    }

    #[test]
    fn test_endpoint_detector_reset() {
        let mut det = EndpointDetector::new(EndpointMode::Biamperometric, 10.0, 5.0);
        det.set_filter_window(1);
        // Reach endpoint
        for i in 0..10 {
            det.process(i as f64, 5.0);
        }
        det.reset();
        assert!(!det.is_reached());
        assert_eq!(det.endpoint_time(), None);
        assert_eq!(det.readings_count(), 0);
    }

    #[test]
    fn test_endpoint_detector_interrupted() {
        let mut det = EndpointDetector::new(EndpointMode::Biamperometric, 10.0, 5.0);
        det.set_filter_window(1);
        det.process(0.0, 5.0);
        det.process(1.0, 5.0);
        det.process(2.0, 5.0);
        // Current goes back above threshold
        det.process(3.0, 50.0);
        det.process(4.0, 5.0);
        det.process(5.0, 5.0);
        // Timer restarted, so not reached yet
        assert!(!det.is_reached());
    }

    #[test]
    fn test_endpoint_detector_from_config() {
        let cfg = KarlFischerConfig::coulometric();
        let det = EndpointDetector::from_config(&cfg);
        assert!((det.threshold_ua - 10.0).abs() < TOLERANCE);
    }

    // -------------------------------------------------------------------
    // DriftCorrector
    // -------------------------------------------------------------------

    #[test]
    fn test_drift_corrector_none() {
        let mut dc = DriftCorrector::new(DriftStrategy::None);
        dc.add_reading(0.0, 0.0);
        dc.add_reading(60.0, 10.0);
        dc.compute();
        assert!((dc.get_drift_rate() - 0.0).abs() < TOLERANCE);
        assert!((dc.correct(5.0, 10.0) - 5.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_drift_corrector_linear() {
        let mut dc = DriftCorrector::new(DriftStrategy::Linear);
        dc.add_reading(0.0, 0.0);
        dc.add_reading(60.0, 10.0);
        dc.add_reading(120.0, 20.0);
        let rate = dc.compute();
        assert!((rate - 10.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_drift_corrector_correct() {
        let mut dc = DriftCorrector::new(DriftStrategy::Linear);
        dc.add_reading(0.0, 0.0);
        dc.add_reading(60.0, 10.0);
        dc.compute();
        // 5 mg gross, 10 µg/min drift, 5 min → net = 5 - 0.05 = 4.95
        let net = dc.correct(5.0, 5.0);
        assert!((net - 4.95).abs() < TOLERANCE);
    }

    #[test]
    fn test_drift_corrector_is_stable() {
        let mut dc = DriftCorrector::new(DriftStrategy::Linear);
        dc.add_reading(0.0, 0.0);
        dc.add_reading(60.0, 5.0);
        dc.compute();
        assert!(dc.is_stable(20.0));
        assert!(dc.is_stable(5.0));
        assert!(!dc.is_stable(3.0));
    }

    #[test]
    fn test_drift_corrector_exponential() {
        let mut dc = DriftCorrector::new(DriftStrategy::Exponential);
        dc.add_reading(0.0, 0.0);
        dc.add_reading(60.0, 10.0);
        dc.add_reading(120.0, 15.0);
        dc.compute();
        // Should compute some rate
        assert!(dc.get_drift_rate() > 0.0);
    }

    #[test]
    fn test_drift_corrector_reset() {
        let mut dc = DriftCorrector::new(DriftStrategy::Linear);
        dc.add_reading(0.0, 0.0);
        dc.add_reading(60.0, 10.0);
        dc.compute();
        dc.reset();
        assert_eq!(dc.readings_count(), 0);
        assert!((dc.get_drift_rate() - 0.0).abs() < TOLERANCE);
    }

    // -------------------------------------------------------------------
    // MoistureCalculation
    // -------------------------------------------------------------------

    #[test]
    fn test_moisture_calc_coulometric() {
        let cfg = KarlFischerConfig::coulometric();
        let calc = MoistureCalculation::new(cfg);
        let q = water_mass_to_coulombs(1.0); // 1 mg water
        let result = calc.from_coulometric(q, 10.0, 0.0, 5.0);
        assert!((result.gross_water_mg - 1.0).abs() < LOOSE_TOL);
        assert!((result.ppm - 100.0).abs() < LOOSE_TOL);
    }

    #[test]
    fn test_moisture_calc_volumetric() {
        let cfg = KarlFischerConfig::volumetric();
        let calc = MoistureCalculation::new(cfg);
        // 2 mL × 5 mg/mL = 10 mg water in 10 g sample = 1000 ppm
        let result = calc.from_volumetric(2.0, 10.0, 0.0, 5.0);
        assert!((result.gross_water_mg - 10.0).abs() < TOLERANCE);
        assert!((result.ppm - 1000.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_moisture_calc_with_drift() {
        let cfg = KarlFischerConfig::coulometric();
        let calc = MoistureCalculation::new(cfg);
        let q = water_mass_to_coulombs(5.0);
        let result = calc.from_coulometric(q, 10.0, 10.0, 5.0);
        // Drift removes 10 µg/min × 5 min = 50 µg = 0.05 mg
        assert!((result.net_water_mg - 4.95).abs() < LOOSE_TOL);
    }

    #[test]
    fn test_moisture_calc_from_curve() {
        let cfg = KarlFischerConfig::coulometric();
        let calc = MoistureCalculation::new(cfg);
        let mut curve = TitrationCurve::new();
        let q = water_mass_to_coulombs(2.0);
        curve.add_point(TitrationPoint { time_s: 0.0, current_ua: 100.0, volume_ml: 0.0, charge_c: 0.0 });
        curve.add_point(TitrationPoint { time_s: 60.0, current_ua: 10.0, volume_ml: 0.0, charge_c: q });
        let result = calc.from_curve(&curve, 5.0, 0.0);
        assert!((result.gross_water_mg - 2.0).abs() < LOOSE_TOL);
    }

    // -------------------------------------------------------------------
    // CoulometricTitrator
    // -------------------------------------------------------------------

    #[test]
    fn test_coulometric_titrator_new() {
        let cfg = KarlFischerConfig::coulometric();
        let ct = CoulometricTitrator::new(cfg);
        assert!(!ct.is_active());
        assert!((ct.total_charge() - 0.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_coulometric_titrator_start() {
        let cfg = KarlFischerConfig::coulometric();
        let mut ct = CoulometricTitrator::new(cfg);
        ct.start();
        assert!(ct.is_active());
    }

    #[test]
    fn test_coulometric_titrator_process() {
        let cfg = KarlFischerConfig::coulometric();
        let mut ct = CoulometricTitrator::new(cfg);
        ct.set_generator_current(0.400);
        ct.start();
        // Process for 10 seconds with high indicator current
        for i in 0..10 {
            ct.process(i as f64, 100.0);
        }
        assert!(ct.is_active());
        assert!(ct.total_charge() > 0.0);
        assert!(ct.water_mg() > 0.0);
    }

    #[test]
    fn test_coulometric_titrator_endpoint() {
        let mut cfg = KarlFischerConfig::coulometric();
        cfg.endpoint_persistence_s = 5.0;
        let mut ct = CoulometricTitrator::new(cfg);
        ct.set_generator_current(0.1);
        ct.start();
        // High current for 5 seconds
        for i in 0..5 {
            ct.process(i as f64, 100.0);
        }
        // Low current for 10 seconds → should reach endpoint
        let mut reached = false;
        for i in 5..20 {
            if ct.process(i as f64, 5.0) {
                reached = true;
                break;
            }
        }
        assert!(reached);
        assert!(!ct.is_active());
    }

    #[test]
    fn test_coulometric_titrator_result() {
        let cfg = KarlFischerConfig::coulometric();
        let mut ct = CoulometricTitrator::new(cfg);
        ct.set_generator_current(0.400);
        ct.start();
        for i in 1..=5 {
            ct.process(i as f64, 100.0);
        }
        let result = ct.result(10.0);
        assert!(result.ppm > 0.0);
        assert_eq!(result.method, TitrationMethod::Coulometric);
    }

    #[test]
    fn test_coulometric_titrator_reset() {
        let cfg = KarlFischerConfig::coulometric();
        let mut ct = CoulometricTitrator::new(cfg);
        ct.start();
        ct.process(1.0, 100.0);
        ct.reset();
        assert!(!ct.is_active());
        assert!((ct.total_charge() - 0.0).abs() < TOLERANCE);
    }

    // -------------------------------------------------------------------
    // VolumetricTitrator
    // -------------------------------------------------------------------

    #[test]
    fn test_volumetric_titrator_new() {
        let cfg = KarlFischerConfig::volumetric();
        let vt = VolumetricTitrator::new(cfg);
        assert!(!vt.is_active());
        assert!((vt.total_volume() - 0.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_volumetric_titrator_start() {
        let cfg = KarlFischerConfig::volumetric();
        let mut vt = VolumetricTitrator::new(cfg);
        vt.start();
        assert!(vt.is_active());
    }

    #[test]
    fn test_volumetric_titrator_process() {
        let cfg = KarlFischerConfig::volumetric();
        let mut vt = VolumetricTitrator::new(cfg);
        vt.start();
        vt.process(1.0, 0.1, 100.0);
        vt.process(2.0, 0.1, 100.0);
        assert!((vt.total_volume() - 0.2).abs() < TOLERANCE);
        // 0.2 mL × 5 mg/mL = 1.0 mg
        assert!((vt.water_mg() - 1.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_volumetric_titrator_endpoint() {
        let mut cfg = KarlFischerConfig::volumetric();
        cfg.endpoint_persistence_s = 5.0;
        let mut vt = VolumetricTitrator::new(cfg);
        vt.start();
        // Dispense with high indicator current
        for i in 0..5 {
            vt.process(i as f64, 0.01, 100.0);
        }
        // Current drops
        let mut reached = false;
        for i in 5..20 {
            if vt.process(i as f64, 0.0, 5.0) {
                reached = true;
                break;
            }
        }
        assert!(reached);
    }

    #[test]
    fn test_volumetric_titrator_remaining_volume() {
        let cfg = KarlFischerConfig::volumetric();
        let mut vt = VolumetricTitrator::new(cfg);
        vt.set_burette_max(5.0);
        vt.start();
        vt.process(1.0, 1.0, 100.0);
        assert!((vt.remaining_volume() - 4.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_volumetric_titrator_result() {
        let cfg = KarlFischerConfig::volumetric();
        let mut vt = VolumetricTitrator::new(cfg);
        vt.start();
        vt.process(1.0, 0.5, 100.0);
        let result = vt.result(10.0);
        // 0.5 mL × 5 mg/mL = 2.5 mg in 10 g = 250 ppm
        assert!((result.ppm - 250.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_volumetric_titrator_reset() {
        let cfg = KarlFischerConfig::volumetric();
        let mut vt = VolumetricTitrator::new(cfg);
        vt.start();
        vt.process(1.0, 0.5, 100.0);
        vt.reset();
        assert!(!vt.is_active());
        assert!((vt.total_volume() - 0.0).abs() < TOLERANCE);
    }

    // -------------------------------------------------------------------
    // OvenMethod
    // -------------------------------------------------------------------

    #[test]
    fn test_oven_isothermal() {
        let oven = OvenMethod::new(OvenProgram::Isothermal(150.0), 1.0);
        assert!((oven.current_temp() - 150.0).abs() < TOLERANCE);
        assert!((oven.temperature_at(10.0) - 150.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_oven_ramp() {
        let oven = OvenMethod::new(
            OvenProgram::Ramp { start_c: 100.0, end_c: 200.0, duration_min: 10.0 },
            1.0,
        );
        assert!((oven.temperature_at(0.0) - 100.0).abs() < TOLERANCE);
        assert!((oven.temperature_at(5.0) - 150.0).abs() < TOLERANCE);
        assert!((oven.temperature_at(10.0) - 200.0).abs() < TOLERANCE);
        // Beyond ramp
        assert!((oven.temperature_at(15.0) - 200.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_oven_step_program() {
        let oven = OvenMethod::new(
            OvenProgram::Step {
                temperatures_c: [100.0, 150.0, 200.0, 0.0],
                durations_min: [5.0, 5.0, 5.0, 0.0],
                num_steps: 3,
            },
            1.0,
        );
        assert!((oven.temperature_at(3.0) - 100.0).abs() < TOLERANCE);
        assert!((oven.temperature_at(7.0) - 150.0).abs() < TOLERANCE);
        assert!((oven.temperature_at(12.0) - 200.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_oven_update() {
        let mut oven = OvenMethod::new(OvenProgram::Isothermal(150.0), 1.0);
        oven.update(1.0, 0.5);
        oven.update(2.0, 0.3);
        assert!((oven.total_water_mg() - 0.8).abs() < TOLERANCE);
        assert!((oven.elapsed_min() - 2.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_oven_moisture_ppm() {
        let mut oven = OvenMethod::new(OvenProgram::Isothermal(150.0), 10.0);
        oven.update(1.0, 1.0); // 1 mg water
        // 1 mg / 10 g = 100 ppm
        assert!((oven.moisture_ppm() - 100.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_oven_moisture_percent() {
        let mut oven = OvenMethod::new(OvenProgram::Isothermal(150.0), 1.0);
        oven.update(1.0, 10.0); // 10 mg water in 1 g = 1%
        assert!((oven.moisture_percent() - 1.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_oven_plateau() {
        let mut oven = OvenMethod::new(OvenProgram::Isothermal(150.0), 1.0);
        for i in 0..20 {
            // Release all water in first 5 min, then plateau
            let water = if i < 5 { 0.2 } else { 0.0 };
            oven.update(i as f64, water);
        }
        // Should be plateau from minute 5 onwards over 10 min window
        assert!(oven.is_plateau(1.0, 5.0));
    }

    #[test]
    fn test_oven_not_plateau() {
        let mut oven = OvenMethod::new(OvenProgram::Isothermal(150.0), 1.0);
        for i in 0..10 {
            oven.update(i as f64, 0.5); // Continuous release
        }
        assert!(!oven.is_plateau(1.0, 5.0));
    }

    #[test]
    fn test_oven_release_profile() {
        let mut oven = OvenMethod::new(OvenProgram::Isothermal(150.0), 1.0);
        oven.update(1.0, 0.1);
        oven.update(2.0, 0.2);
        let profile = oven.release_profile();
        assert_eq!(profile.len(), 2);
        assert!((profile[1].1 - 0.3).abs() < TOLERANCE);
    }

    #[test]
    fn test_oven_complete() {
        let mut oven = OvenMethod::new(OvenProgram::Isothermal(150.0), 1.0);
        assert!(!oven.is_complete());
        oven.finish();
        assert!(oven.is_complete());
    }

    // -------------------------------------------------------------------
    // Statistics and validation
    // -------------------------------------------------------------------

    #[test]
    fn test_replicate_statistics_basic() {
        let values = [100.0, 102.0, 98.0, 101.0, 99.0];
        let stats = replicate_statistics(&values);
        assert_eq!(stats.n, 5);
        assert!((stats.mean_ppm - 100.0).abs() < TOLERANCE);
        assert!(stats.std_dev_ppm > 0.0);
        assert!(stats.rsd_percent > 0.0);
    }

    #[test]
    fn test_replicate_statistics_single() {
        let stats = replicate_statistics(&[50.0]);
        assert_eq!(stats.n, 1);
        assert!((stats.mean_ppm - 50.0).abs() < TOLERANCE);
        assert!((stats.std_dev_ppm - 0.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_replicate_statistics_empty() {
        let stats = replicate_statistics(&[]);
        assert_eq!(stats.n, 0);
        assert!((stats.mean_ppm - 0.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_replicate_statistics_identical() {
        let values = [100.0, 100.0, 100.0];
        let stats = replicate_statistics(&values);
        assert!((stats.std_dev_ppm - 0.0).abs() < TOLERANCE);
        assert!((stats.range_ppm - 0.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_passes_repeatability_good() {
        let values = [100.0, 101.0, 99.5, 100.5];
        assert!(passes_repeatability(&values, 5.0));
    }

    #[test]
    fn test_passes_repeatability_bad() {
        let values = [100.0, 150.0, 50.0, 200.0];
        assert!(!passes_repeatability(&values, 5.0));
    }

    #[test]
    fn test_passes_repeatability_single() {
        assert!(passes_repeatability(&[100.0], 5.0));
    }

    #[test]
    fn test_blank_correction_basic() {
        let blanks = [0.01, 0.02, 0.015];
        let blank = blank_correction(&blanks);
        assert!((blank - 0.015).abs() < TOLERANCE);
    }

    #[test]
    fn test_blank_correction_empty() {
        assert!((blank_correction(&[]) - 0.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_apply_blank() {
        assert!((apply_blank(5.0, 0.02) - 4.98).abs() < TOLERANCE);
    }

    #[test]
    fn test_apply_blank_clamps() {
        assert!((apply_blank(0.01, 0.5) - 0.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_compute_titer() {
        // Known 10 mg standard, measured 2.1 mL, nominal 5 mg/mL
        // actual conc = 10/2.1 = 4.762, titer = 4.762/5 = 0.9524
        let titer = compute_titer(10.0, 2.1, 5.0);
        assert!((titer - 10.0 / 2.1 / 5.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_apply_titer() {
        assert!((apply_titer(5.0, 1.05) - 5.25).abs() < TOLERANCE);
    }

    #[test]
    fn test_compute_titer_zero_volume() {
        assert!((compute_titer(10.0, 0.0, 5.0) - 1.0).abs() < TOLERANCE);
    }

    // -------------------------------------------------------------------
    // Utility functions
    // -------------------------------------------------------------------

    #[test]
    fn test_min_sample_weight() {
        // To get 0.5 mg H₂O from 100 ppm → need 5 g
        let weight = min_sample_weight_g(100.0, 0.5);
        assert!((weight - 5.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_min_sample_weight_zero_ppm() {
        assert!((min_sample_weight_g(0.0, 1.0) - 0.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_required_current() {
        let water_mg = 1.0;
        let time_s = 100.0;
        let i = required_current(water_mg, time_s);
        let q = water_mass_to_coulombs(water_mg);
        assert!((i - q / time_s).abs() < TOLERANCE);
    }

    #[test]
    fn test_required_current_zero_time() {
        assert!((required_current(1.0, 0.0) - 0.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_estimated_titration_time_coulometric() {
        let t = estimated_titration_time_s(1.0, TitrationMethod::Coulometric, 0.400);
        let expected = water_mass_to_coulombs(1.0) / 0.400;
        assert!((t - expected).abs() < TOLERANCE);
    }

    #[test]
    fn test_estimated_titration_time_volumetric() {
        let t = estimated_titration_time_s(5.0, TitrationMethod::Volumetric, 0.1);
        // 5 mg / 5 mg/mL = 1 mL / 0.1 mL/s = 10 s
        assert!((t - 10.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_estimated_titration_time_zero_rate() {
        assert!((estimated_titration_time_s(1.0, TitrationMethod::Coulometric, 0.0) - 0.0).abs() < TOLERANCE);
    }
}
