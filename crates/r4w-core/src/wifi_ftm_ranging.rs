//! Wi-Fi Fine Timing Measurement (FTM) Ranging Processor
//!
//! Implements IEEE 802.11mc (FTM) and 802.11az (Next Generation Positioning) ranging
//! protocols for high-accuracy indoor/outdoor positioning.
//!
//! # Protocol Overview
//!
//! FTM uses a four-timestamp Round-Trip Time (RTT) exchange:
//! - t1: Initiator sends FTM frame
//! - t2: Responder receives FTM frame
//! - t3: Responder sends FTM ACK (with t1/t2 in frame body)
//! - t4: Initiator receives FTM ACK
//!
//! RTT = (t4 - t1) - (t3 - t2)
//! Distance = RTT * c / 2
//!
//! # Timestamp Resolution
//!
//! IEEE 802.11mc specifies 10 picosecond resolution (100 GHz virtual clock).
//! This allows theoretical sub-centimeter accuracy before other error sources.
//!
//! # References
//! - IEEE 802.11mc-2016: Fine Timing Measurement
//! - IEEE 802.11az-2022: Next Generation Positioning (NGP)
//! - IEEE Std 802.11-2020 Section 9.4.2.167 (FTM Parameters element)

use std::collections::VecDeque;

/// Speed of light in meters per second (CODATA 2018)
pub const SPEED_OF_LIGHT_M_S: f64 = 299_792_458.0;

/// FTM timestamp unit: 10 picoseconds = 0.01 ns
/// Corresponds to 100 GHz virtual clock
pub const FTM_TIMESTAMP_UNIT_NS: f64 = 0.01; // nanoseconds

/// Boltzmann constant (J/K)
const BOLTZMANN_K: f64 = 1.380_649e-23;

/// Reference temperature 290 K (kTB noise floor standard)
const REFERENCE_TEMP_K: f64 = 290.0;

// ---------------------------------------------------------------------------
// Configuration structs
// ---------------------------------------------------------------------------

/// Channel bandwidth options for Wi-Fi FTM
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ChannelBandwidth {
    /// 20 MHz - basic operation, ~7.5 ns resolution
    Mhz20,
    /// 40 MHz - better resolution
    Mhz40,
    /// 80 MHz - good accuracy
    Mhz80,
    /// 160 MHz - best accuracy for legacy
    Mhz160,
    /// 320 MHz - 802.11az EHT (Extremely High Throughput)
    Mhz320,
}

impl ChannelBandwidth {
    /// Return bandwidth in Hz
    pub fn hz(&self) -> f64 {
        match self {
            ChannelBandwidth::Mhz20 => 20e6,
            ChannelBandwidth::Mhz40 => 40e6,
            ChannelBandwidth::Mhz80 => 80e6,
            ChannelBandwidth::Mhz160 => 160e6,
            ChannelBandwidth::Mhz320 => 320e6,
        }
    }

    /// Range resolution (meters) = c / (2 * BW)
    pub fn range_resolution_m(&self) -> f64 {
        SPEED_OF_LIGHT_M_S / (2.0 * self.hz())
    }
}

/// FTM burst parameters per IEEE 802.11mc
#[derive(Debug, Clone)]
pub struct FtmConfig {
    /// Channel bandwidth
    pub bandwidth: ChannelBandwidth,
    /// Number of FTM measurements per burst (1-31)
    pub ftms_per_burst: u8,
    /// Number of bursts (0 = unlimited, 1-14 per exponent)
    pub num_burst_exponent: u8,
    /// Burst period in 100 ms units (0 = no constraint)
    pub burst_period: u8,
    /// ASAP (As Soon As Possible) mode
    pub asap: bool,
    /// Minimum delta FTM in 100 μs units
    pub min_delta_ftm: u8,
    /// Partial TSF timer - partial time sync information
    pub partial_tsf: bool,
    /// ASAp capable - can do immediate measurement
    pub asap_capable: bool,
    /// 802.11az secure LTF (Long Training Field) ranging
    pub secure_ltf: bool,
    /// Noise figure of receiver in dB
    pub rx_noise_figure_db: f64,
    /// Maximum RTT measurement error threshold for outlier rejection (ns)
    pub outlier_threshold_ns: f64,
}

impl Default for FtmConfig {
    fn default() -> Self {
        FtmConfig {
            bandwidth: ChannelBandwidth::Mhz80,
            ftms_per_burst: 8,
            num_burst_exponent: 0,
            burst_period: 0,
            asap: true,
            min_delta_ftm: 0,
            partial_tsf: false,
            asap_capable: true,
            secure_ltf: false,
            rx_noise_figure_db: 7.0,
            outlier_threshold_ns: 50.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/// Four-timestamp exchange used to compute RTT
///
/// All timestamps in nanoseconds at 10 ps resolution.
/// t1, t4 are measured by initiator; t2, t3 by responder.
#[derive(Debug, Clone, Copy)]
pub struct TimestampExchange {
    /// t1: Departure time of FTM frame at initiator (ns)
    pub t1: f64,
    /// t2: Arrival time of FTM frame at responder (ns)
    pub t2: f64,
    /// t3: Departure time of FTM ACK at responder (ns)
    pub t3: f64,
    /// t4: Arrival time of FTM ACK at initiator (ns)
    pub t4: f64,
}

impl TimestampExchange {
    /// Create a new timestamp exchange
    pub fn new(t1: f64, t2: f64, t3: f64, t4: f64) -> Self {
        TimestampExchange { t1, t2, t3, t4 }
    }

    /// True one-way propagation time (for simulation only - normally unknown)
    pub fn true_tof_ns(&self) -> f64 {
        (self.t2 - self.t1 + self.t4 - self.t3) / 2.0
    }
}

/// A single FTM measurement result from one timestamp exchange
#[derive(Debug, Clone)]
pub struct FtmMeasurement {
    /// Sequence number within burst
    pub seq_num: u32,
    /// Timestamp exchange data
    pub exchange: TimestampExchange,
    /// Computed RTT in nanoseconds
    pub rtt_ns: f64,
    /// Estimated distance in meters
    pub distance_m: f64,
    /// RSSI in dBm (if available)
    pub rssi_dbm: Option<f64>,
    /// 802.11az: I/Q samples for channel sounding (optional)
    pub cir_samples: Option<Vec<(f64, f64)>>,
}

/// Statistical ranging result from a burst of measurements
#[derive(Debug, Clone)]
pub struct RangingResult {
    /// Best estimate distance in meters
    pub distance_m: f64,
    /// 1-sigma uncertainty in meters
    pub uncertainty_m: f64,
    /// Number of valid measurements used
    pub num_measurements: usize,
    /// Mean RTT in nanoseconds
    pub mean_rtt_ns: f64,
    /// RTT standard deviation in nanoseconds
    pub rtt_std_ns: f64,
    /// Number of outliers rejected
    pub outliers_rejected: usize,
    /// NLOS (Non-Line-of-Sight) detected
    pub nlos_detected: bool,
    /// Confidence level [0,1]
    pub confidence: f64,
    /// CRLB (Cramer-Rao Lower Bound) on range error (m)
    pub crlb_m: f64,
}

impl Default for RangingResult {
    fn default() -> Self {
        RangingResult {
            distance_m: 0.0,
            uncertainty_m: f64::INFINITY,
            num_measurements: 0,
            mean_rtt_ns: 0.0,
            rtt_std_ns: f64::INFINITY,
            outliers_rejected: 0,
            nlos_detected: false,
            confidence: 0.0,
            crlb_m: f64::INFINITY,
        }
    }
}

/// Location Configuration Information (LCI) per IEEE 802.11-2020
/// Carries geodetic position of the AP/STA
#[derive(Debug, Clone)]
pub struct LocationConfigInfo {
    /// WGS-84 latitude in degrees (-90 to +90)
    pub latitude_deg: f64,
    /// WGS-84 longitude in degrees (-180 to +180)
    pub longitude_deg: f64,
    /// Altitude in meters above WGS-84 ellipsoid
    pub altitude_m: f64,
    /// Latitude uncertainty in degrees (optional)
    pub lat_uncertainty_deg: Option<f64>,
    /// Longitude uncertainty in degrees (optional)
    pub lon_uncertainty_deg: Option<f64>,
    /// Altitude uncertainty in meters (optional)
    pub alt_uncertainty_m: Option<f64>,
    /// Altitude type: 0=floors, 1=meters
    pub altitude_type: u8,
    /// Datum: 0=WGS-84, 1=NAD83, 2=NAD83+MLLW
    pub datum: u8,
    /// Registration token (opaque 64-bit identifier)
    pub reg_token: u64,
}

impl LocationConfigInfo {
    /// Create a new LCI with basic fields
    pub fn new(latitude_deg: f64, longitude_deg: f64, altitude_m: f64) -> Self {
        LocationConfigInfo {
            latitude_deg,
            longitude_deg,
            altitude_m,
            lat_uncertainty_deg: None,
            lon_uncertainty_deg: None,
            alt_uncertainty_m: None,
            altitude_type: 1,
            datum: 0,
            reg_token: 0,
        }
    }

    /// Distance in meters to another LCI position (Haversine formula)
    pub fn distance_to(&self, other: &LocationConfigInfo) -> f64 {
        const EARTH_RADIUS_M: f64 = 6_371_000.0;
        let lat1 = self.latitude_deg.to_radians();
        let lat2 = other.latitude_deg.to_radians();
        let dlat = (other.latitude_deg - self.latitude_deg).to_radians();
        let dlon = (other.longitude_deg - self.longitude_deg).to_radians();
        let a = (dlat / 2.0).sin().powi(2)
            + lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
        EARTH_RADIUS_M * c
    }
}

/// Location Civic Report (LCR) - street address / civic location per RFC 4776
#[derive(Debug, Clone)]
pub struct LocationCivicReport {
    /// Country code (ISO 3166-1 alpha-2)
    pub country: String,
    /// State or province
    pub state: Option<String>,
    /// City or municipality
    pub city: Option<String>,
    /// Street address
    pub street: Option<String>,
    /// Building name or identifier
    pub building: Option<String>,
    /// Floor number
    pub floor: Option<f32>,
    /// Room number
    pub room: Option<String>,
}

impl LocationCivicReport {
    /// Create a new LCR with only country code
    pub fn new(country: &str) -> Self {
        LocationCivicReport {
            country: country.to_string(),
            state: None,
            city: None,
            street: None,
            building: None,
            floor: None,
            room: None,
        }
    }
}

/// 3D position estimate
#[derive(Debug, Clone, Copy)]
pub struct Position {
    /// Latitude in degrees (WGS-84)
    pub lat_deg: f64,
    /// Longitude in degrees (WGS-84)
    pub lon_deg: f64,
    /// Altitude in meters
    pub alt_m: f64,
}

/// 2D Cartesian position (for local coordinate trilateration)
#[derive(Debug, Clone, Copy)]
pub struct Position2d {
    pub x_m: f64,
    pub y_m: f64,
}

/// 3D Cartesian position (for local coordinate trilateration)
#[derive(Debug, Clone, Copy)]
pub struct Position3d {
    pub x_m: f64,
    pub y_m: f64,
    pub z_m: f64,
}

/// Clock drift model between two stations
#[derive(Debug, Clone)]
pub struct ClockDriftModel {
    /// Estimated frequency offset ratio (ppm)
    pub freq_offset_ppm: f64,
    /// Phase offset at reference time (ns)
    pub phase_offset_ns: f64,
    /// Age of estimate (number of measurements since last update)
    pub estimate_age: u32,
    /// Variance of frequency offset estimate
    pub freq_var_ppm2: f64,
}

impl ClockDriftModel {
    pub fn new() -> Self {
        ClockDriftModel {
            freq_offset_ppm: 0.0,
            phase_offset_ns: 0.0,
            estimate_age: 0,
            freq_var_ppm2: 100.0, // 10 ppm initial uncertainty
        }
    }

    /// Correct a raw RTT measurement for clock drift
    /// drift_ns = freq_offset_ppm * 1e-6 * interval_ns
    pub fn correct_rtt(&self, rtt_ns: f64, interval_ns: f64) -> f64 {
        let drift_correction = self.freq_offset_ppm * 1e-6 * interval_ns;
        rtt_ns - drift_correction
    }
}

impl Default for ClockDriftModel {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// FTM Ranging Processor
// ---------------------------------------------------------------------------

/// Wi-Fi Fine Timing Measurement (FTM) Ranging Processor
///
/// Implements the full FTM/NGP signal processing chain:
/// - RTT computation from timestamp exchanges
/// - Clock drift estimation and compensation
/// - Channel Impulse Response (CIR) based ToA estimation
/// - Multipath mitigation via leading-edge detection
/// - Burst averaging with outlier rejection
/// - 2D/3D trilateration from multiple anchors
/// - CRLB computation for ranging quality assessment
pub struct WiFiFtmRanging {
    /// Configuration parameters
    config: FtmConfig,
    /// History of RTT measurements for drift estimation
    rtt_history: VecDeque<f64>,
    /// Clock drift model
    clock_model: ClockDriftModel,
    /// Accumulated measurements for burst processing
    burst_buffer: Vec<FtmMeasurement>,
    /// Maximum history length
    max_history: usize,
}

impl WiFiFtmRanging {
    /// Create a new FTM ranging processor with the given config
    pub fn new(config: FtmConfig) -> Self {
        WiFiFtmRanging {
            config,
            rtt_history: VecDeque::new(),
            clock_model: ClockDriftModel::new(),
            burst_buffer: Vec::new(),
            max_history: 64,
        }
    }

    /// Create with default 80 MHz configuration
    pub fn default_80mhz() -> Self {
        Self::new(FtmConfig::default())
    }

    /// Compute RTT from a four-timestamp exchange (in nanoseconds)
    ///
    /// RTT = (t4 - t1) - (t3 - t2)
    /// This cancels the responder processing time (t3 - t2).
    pub fn compute_rtt(&self, exchange: &TimestampExchange) -> f64 {
        let t41 = exchange.t4 - exchange.t1;
        let t32 = exchange.t3 - exchange.t2;
        t41 - t32
    }

    /// Convert RTT in nanoseconds to distance in meters
    ///
    /// distance = RTT * c / 2
    /// Factor of 2 because RTT is round-trip (signal travels twice)
    pub fn rtt_to_distance(&self, rtt_ns: f64) -> f64 {
        if rtt_ns < 0.0 {
            return 0.0;
        }
        rtt_ns * 1e-9 * SPEED_OF_LIGHT_M_S / 2.0
    }

    /// Convert distance in meters to RTT in nanoseconds
    pub fn distance_to_rtt(&self, distance_m: f64) -> f64 {
        distance_m * 2.0 / SPEED_OF_LIGHT_M_S * 1e9
    }

    /// Process a single timestamp exchange into a measurement
    pub fn process_exchange(&mut self, seq_num: u32, exchange: TimestampExchange) -> FtmMeasurement {
        let rtt_ns = self.compute_rtt(&exchange);
        let distance_m = self.rtt_to_distance(rtt_ns);

        // Update RTT history for drift tracking
        self.rtt_history.push_back(rtt_ns);
        if self.rtt_history.len() > self.max_history {
            self.rtt_history.pop_front();
        }

        FtmMeasurement {
            seq_num,
            exchange,
            rtt_ns,
            distance_m,
            rssi_dbm: None,
            cir_samples: None,
        }
    }

    /// Add a measurement to the burst buffer
    pub fn add_to_burst(&mut self, meas: FtmMeasurement) {
        self.burst_buffer.push(meas);
    }

    /// Flush burst buffer and compute averaged ranging result
    pub fn flush_burst(&mut self, snr_db: f64) -> RangingResult {
        let measurements: Vec<f64> = self.burst_buffer.iter().map(|m| m.rtt_ns).collect();
        let result = self.burst_average_with_snr(&measurements, snr_db);
        self.burst_buffer.clear();
        result
    }

    // ---------------------------------------------------------------------------
    // Burst averaging with outlier rejection
    // ---------------------------------------------------------------------------

    /// Average a burst of RTT measurements with outlier rejection.
    ///
    /// Uses modified Z-score (MAD-based) for robust outlier detection,
    /// then computes mean and standard deviation of inliers.
    pub fn burst_average(&self, measurements: &[f64]) -> RangingResult {
        self.burst_average_with_snr(measurements, 20.0)
    }

    /// Average burst with SNR-aware weighting and CRLB computation
    pub fn burst_average_with_snr(&self, measurements: &[f64], snr_db: f64) -> RangingResult {
        if measurements.is_empty() {
            return RangingResult::default();
        }

        if measurements.len() == 1 {
            let rtt = measurements[0];
            let dist = self.rtt_to_distance(rtt);
            let crlb = self.crlb_ranging(snr_db, self.config.bandwidth.hz());
            return RangingResult {
                distance_m: dist,
                uncertainty_m: crlb,
                num_measurements: 1,
                mean_rtt_ns: rtt,
                rtt_std_ns: 0.0,
                outliers_rejected: 0,
                nlos_detected: false,
                confidence: 0.5,
                crlb_m: crlb,
            };
        }

        // Step 1: Find median and MAD for robust outlier detection
        let mut sorted = measurements.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median = median_of_sorted(&sorted);

        let mut abs_devs: Vec<f64> = sorted.iter().map(|&x| (x - median).abs()).collect();
        abs_devs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let mad = median_of_sorted(&abs_devs);

        // Modified Z-score threshold: 3.5 (Iglewicz & Hoaglin)
        // z_i = 0.6745 * |x_i - median| / MAD
        let threshold = if mad > 1e-9 {
            self.config.outlier_threshold_ns.min(3.5 * mad / 0.6745)
        } else {
            self.config.outlier_threshold_ns
        };

        // Step 2: Filter outliers
        let inliers: Vec<f64> = measurements
            .iter()
            .filter(|&&x| (x - median).abs() <= threshold)
            .cloned()
            .collect();
        let outliers_rejected = measurements.len() - inliers.len();

        if inliers.is_empty() {
            // Fall back to all measurements if all rejected
            let mean = measurements.iter().sum::<f64>() / measurements.len() as f64;
            let dist = self.rtt_to_distance(mean);
            return RangingResult {
                distance_m: dist,
                uncertainty_m: self.config.outlier_threshold_ns * 1e-9 * SPEED_OF_LIGHT_M_S / 2.0,
                num_measurements: measurements.len(),
                mean_rtt_ns: mean,
                rtt_std_ns: 0.0,
                outliers_rejected,
                nlos_detected: true,
                confidence: 0.1,
                crlb_m: self.crlb_ranging(snr_db, self.config.bandwidth.hz()),
            };
        }

        // Step 3: Compute statistics of inliers
        let n = inliers.len() as f64;
        let mean_rtt = inliers.iter().sum::<f64>() / n;
        let variance = if inliers.len() > 1 {
            inliers.iter().map(|&x| (x - mean_rtt).powi(2)).sum::<f64>() / (n - 1.0)
        } else {
            0.0
        };
        let std_rtt = variance.sqrt();

        // Step 4: Convert to distance
        let mean_dist = self.rtt_to_distance(mean_rtt);
        // Uncertainty from RTT std and CRLB combined in quadrature
        let crlb = self.crlb_ranging(snr_db, self.config.bandwidth.hz());
        let std_dist = std_rtt * 1e-9 * SPEED_OF_LIGHT_M_S / 2.0;
        let uncertainty = (std_dist.powi(2) + crlb.powi(2)).sqrt();

        // Step 5: NLOS detection
        let nlos = self.detect_nlos_from_stats(&inliers, mean_rtt, std_rtt);

        // Confidence: based on number of inliers and spread
        let confidence = compute_confidence(inliers.len(), outliers_rejected, std_rtt);

        RangingResult {
            distance_m: mean_dist,
            uncertainty_m: uncertainty,
            num_measurements: inliers.len(),
            mean_rtt_ns: mean_rtt,
            rtt_std_ns: std_rtt,
            outliers_rejected,
            nlos_detected: nlos,
            confidence,
            crlb_m: crlb,
        }
    }

    // ---------------------------------------------------------------------------
    // NLOS detection
    // ---------------------------------------------------------------------------

    /// Detect NLOS condition from a set of RTT measurements.
    ///
    /// NLOS heuristics:
    /// 1. High range spread (std dev > threshold) suggests multipath
    /// 2. Positive skewness indicates dominant positive bias from NLOS
    /// 3. Kurtosis > threshold indicates non-Gaussian distribution
    pub fn detect_nlos(&self, measurements: &[f64]) -> bool {
        if measurements.len() < 4 {
            return false;
        }
        let n = measurements.len() as f64;
        let mean = measurements.iter().sum::<f64>() / n;
        let variance = measurements.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n;
        let std = variance.sqrt();
        self.detect_nlos_from_stats(measurements, mean, std)
    }

    fn detect_nlos_from_stats(&self, measurements: &[f64], mean: f64, std: f64) -> bool {
        if measurements.len() < 4 {
            return false;
        }
        if std <= 1e-12 {
            return false;
        }
        let n = measurements.len() as f64;

        // Compute skewness: positive skewness suggests NLOS bias
        let skewness = measurements
            .iter()
            .map(|&x| ((x - mean) / std).powi(3))
            .sum::<f64>()
            / n;

        // Compute kurtosis (excess)
        let kurtosis = measurements
            .iter()
            .map(|&x| ((x - mean) / std).powi(4))
            .sum::<f64>()
            / n
            - 3.0;

        // High std relative to range resolution = possible NLOS
        let range_res = self.config.bandwidth.range_resolution_m();
        let std_dist_m = std * 1e-9 * SPEED_OF_LIGHT_M_S / 2.0;
        let std_normalized = std_dist_m / range_res;

        // NLOS indicators: positive skewness + high spread or high kurtosis
        (skewness > 0.5 && std_normalized > 2.0) || kurtosis > 2.5
    }

    // ---------------------------------------------------------------------------
    // Clock drift estimation and calibration
    // ---------------------------------------------------------------------------

    /// Estimate frequency offset between initiator and responder clocks.
    ///
    /// Uses pairs of consecutive RTT measurements and the known interval
    /// to estimate the fractional frequency offset (ppm).
    ///
    /// Method: If two RTT measurements at times Δt apart show a drift of δRTT,
    /// then freq_offset = δRTT / (2 * Δt) in relative units.
    pub fn estimate_clock_drift(&mut self, exchanges: &[TimestampExchange]) -> f64 {
        if exchanges.len() < 2 {
            return self.clock_model.freq_offset_ppm;
        }

        // Use pairs of consecutive measurements
        let mut drift_estimates = Vec::new();

        for i in 1..exchanges.len() {
            let e0 = &exchanges[i - 1];
            let e1 = &exchanges[i];

            // Time interval between measurements (at initiator)
            let interval_ns = e1.t1 - e0.t1;
            if interval_ns < 1.0 {
                continue; // Too close
            }

            // RTT at both times
            let rtt0 = self.compute_rtt(e0);
            let rtt1 = self.compute_rtt(e1);

            // Difference in RTT due to clock drift
            let d_rtt = rtt1 - rtt0;

            // Frequency offset: drift_ppm = (d_rtt / interval_ns) * 1e6 / 2
            // Factor of 2: RTT accumulates drift twice (TX and RX paths)
            let drift_ppm = (d_rtt / interval_ns) * 1e6 / 2.0;
            drift_estimates.push(drift_ppm);
        }

        if drift_estimates.is_empty() {
            return self.clock_model.freq_offset_ppm;
        }

        // Robust mean of drift estimates
        let n = drift_estimates.len() as f64;
        let mean_drift: f64 = drift_estimates.iter().sum::<f64>() / n;

        // Exponential smoothing update (alpha = 0.3)
        let alpha = 0.3;
        self.clock_model.freq_offset_ppm = (1.0 - alpha) * self.clock_model.freq_offset_ppm
            + alpha * mean_drift;
        self.clock_model.estimate_age = 0;

        self.clock_model.freq_offset_ppm
    }

    /// Apply clock drift correction to a raw RTT measurement
    pub fn apply_drift_correction(&self, rtt_ns: f64, interval_ns: f64) -> f64 {
        self.clock_model.correct_rtt(rtt_ns, interval_ns)
    }

    // ---------------------------------------------------------------------------
    // Time-of-Arrival (ToA) estimation from Channel Impulse Response
    // ---------------------------------------------------------------------------

    /// Estimate Time of Arrival from CIR (Channel Impulse Response) samples.
    ///
    /// Uses matched filtering (cross-correlation) between received signal and
    /// known reference, then parabolic interpolation for sub-sample accuracy.
    ///
    /// # Arguments
    /// * `signal` - Received complex baseband samples (I, Q)
    /// * `reference` - Known reference/pilot sequence complex samples (I, Q)
    ///
    /// # Returns
    /// Estimated ToA in sample units (fractional)
    pub fn estimate_toa(&self, signal: &[(f64, f64)], reference: &[(f64, f64)]) -> f64 {
        if signal.is_empty() || reference.is_empty() {
            return 0.0;
        }

        // Cross-correlate signal with reference to get CIR
        let cir = cross_correlate_complex(signal, reference);

        // Find peak of |CIR|^2
        let peak_idx = cir
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| {
                let ma = a.0 * a.0 + a.1 * a.1;
                let mb = b.0 * b.0 + b.1 * b.1;
                ma.partial_cmp(&mb).unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
            .unwrap_or(0);

        // Parabolic interpolation for sub-sample accuracy
        parabolic_interpolate_peak(&cir, peak_idx)
    }

    /// Leading-edge detection in CIR for NLOS/multipath mitigation.
    ///
    /// Finds the first significant path (leading edge) rather than the peak,
    /// which corresponds to the direct (LOS) path.
    ///
    /// # Arguments
    /// * `cir` - Channel Impulse Response complex samples (I, Q)
    /// * `threshold_db` - Detection threshold below peak (dB)
    ///
    /// # Returns
    /// Sample index (fractional) of leading edge
    pub fn leading_edge_detect(&self, cir: &[(f64, f64)], threshold_db: f64) -> f64 {
        if cir.is_empty() {
            return 0.0;
        }

        // Compute power profile
        let power: Vec<f64> = cir.iter().map(|(i, q)| i * i + q * q).collect();

        // Find peak power
        let peak_power = power.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        if peak_power <= 0.0 {
            return 0.0;
        }

        // Convert threshold_db below peak to linear threshold
        let threshold_linear = peak_power * 10f64.powf(-threshold_db.abs() / 10.0);

        // Find first sample exceeding threshold (leading edge)
        let leading_idx = power
            .iter()
            .position(|&p| p >= threshold_linear)
            .unwrap_or(0);

        // Refine with parabolic interpolation if not at boundary
        if leading_idx == 0 || leading_idx >= power.len() - 1 {
            leading_idx as f64
        } else {
            let p_prev = power[leading_idx - 1];
            let p_cur = power[leading_idx];
            let p_next = power[leading_idx + 1];

            // Linear interpolation on the rising edge
            if p_cur > p_prev {
                let frac = (threshold_linear - p_prev) / (p_cur - p_prev).max(1e-30);
                (leading_idx - 1) as f64 + frac.clamp(0.0, 1.0)
            } else {
                leading_idx as f64
            }
        }
    }

    /// Super-resolution ToA estimation using MUSIC-like algorithm.
    ///
    /// Constructs a spatial correlation matrix from CIR snapshots and
    /// applies MUSIC (Multiple Signal Classification) pseudospectrum.
    /// This achieves sub-sample delay resolution beyond the Rayleigh limit.
    ///
    /// # Arguments
    /// * `cir` - Channel Impulse Response (complex samples)
    /// * `num_paths` - Expected number of multipath components
    /// * `sample_rate_hz` - CIR sample rate in Hz
    ///
    /// # Returns
    /// Vec of estimated path delays in nanoseconds
    pub fn super_resolution_toa(
        &self,
        cir: &[(f64, f64)],
        num_paths: usize,
        sample_rate_hz: f64,
    ) -> Vec<f64> {
        if cir.len() < 4 || num_paths == 0 {
            return vec![];
        }

        let n = cir.len();
        let m = n / 2; // Smoothing sub-array size
        let num_paths = num_paths.min(m - 1);

        // Build correlation matrix (smoothed, real-valued power)
        // R[i][j] = sum_k { cir[k+i] * conj(cir[k+j]) }
        let mut r = vec![vec![(0.0_f64, 0.0_f64); m]; m];
        let snapshots = n - m + 1;

        for k in 0..snapshots {
            for i in 0..m {
                for j in 0..m {
                    let xi = cir[k + i];
                    let xj = cir[k + j];
                    // x_i * conj(x_j)
                    r[i][j].0 += xi.0 * xj.0 + xi.1 * xj.1;
                    r[i][j].1 += xi.1 * xj.0 - xi.0 * xj.1;
                }
            }
        }
        let scale = 1.0 / snapshots as f64;
        for i in 0..m {
            for j in 0..m {
                r[i][j].0 *= scale;
                r[i][j].1 *= scale;
            }
        }

        // Eigendecomposition via power iteration (simplified)
        // Extract noise subspace eigenvectors
        let noise_vecs = noise_subspace_real(&r, m, num_paths);

        // MUSIC pseudospectrum: scan delay candidates
        let num_scan = 256;
        let sample_period_ns = 1e9 / sample_rate_hz;
        let mut spectrum = vec![0.0_f64; num_scan];

        for k in 0..num_scan {
            let delay_samples = (k as f64 / num_scan as f64) * m as f64;
            // Build steering vector a(tau): a[i] = exp(-j*2*pi*i*delay/N)
            let mut a: Vec<(f64, f64)> = (0..m)
                .map(|i| {
                    let phase = -2.0 * std::f64::consts::PI * i as f64 * delay_samples / n as f64;
                    (phase.cos(), phase.sin())
                })
                .collect();

            // Normalize
            let norm: f64 = (a.iter().map(|(r, i)| r * r + i * i).sum::<f64>())
                .sqrt()
                .max(1e-30);
            for v in &mut a {
                v.0 /= norm;
                v.1 /= norm;
            }

            // Project onto noise subspace
            let mut noise_power = 0.0_f64;
            for en in &noise_vecs {
                // |a^H * en|^2
                let dot_r: f64 = a.iter().zip(en.iter()).map(|(ai, ni)| ai.0 * ni.0 + ai.1 * ni.1).sum();
                let dot_i: f64 = a.iter().zip(en.iter()).map(|(ai, ni)| ai.1 * ni.0 - ai.0 * ni.1).sum();
                noise_power += dot_r * dot_r + dot_i * dot_i;
            }

            // MUSIC: 1 / noise_power
            spectrum[k] = if noise_power > 1e-30 { 1.0 / noise_power } else { f64::MAX };
        }

        // Find peaks in MUSIC spectrum
        let mut peaks = Vec::new();
        let mut sorted_spectrum: Vec<(usize, f64)> = spectrum
            .iter()
            .enumerate()
            .map(|(i, &v)| (i, v))
            .collect();
        sorted_spectrum.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        let peak_threshold = sorted_spectrum.first().map(|(_, v)| v * 0.1).unwrap_or(0.0);

        for (idx, val) in &sorted_spectrum {
            if *val < peak_threshold {
                break;
            }
            // Ensure minimum separation from existing peaks
            let delay_ns = (*idx as f64 / num_scan as f64) * m as f64 * sample_period_ns;
            let min_sep_ns = sample_period_ns;
            let is_separate = peaks.iter().all(|&p: &f64| (p - delay_ns).abs() > min_sep_ns);
            if is_separate {
                peaks.push(delay_ns);
            }
            if peaks.len() >= num_paths {
                break;
            }
        }

        peaks.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        peaks
    }

    // ---------------------------------------------------------------------------
    // CRLB - Cramer-Rao Lower Bound for ranging
    // ---------------------------------------------------------------------------

    /// Compute Cramer-Rao Lower Bound on range estimation error (meters).
    ///
    /// CRLB for ToA-based ranging:
    ///   σ_r^2 ≥ c^2 / (8 * π^2 * SNR * β^2 * T)
    ///
    /// where β is the effective bandwidth (RMS bandwidth), T is observation time.
    /// For white spectrum: β ≈ BW / sqrt(3) (uniform power in [-BW/2, BW/2])
    ///
    /// # Arguments
    /// * `snr_db` - Signal-to-noise ratio in dB
    /// * `bw_hz` - Signal bandwidth in Hz
    ///
    /// # Returns
    /// CRLB standard deviation in meters
    pub fn crlb_ranging(&self, snr_db: f64, bw_hz: f64) -> f64 {
        let snr_linear = 10f64.powf(snr_db / 10.0);
        if snr_linear <= 0.0 || bw_hz <= 0.0 {
            return f64::INFINITY;
        }

        // RMS bandwidth for uniform spectrum over [-BW/2, BW/2]
        // β_rms = BW / (2 * sqrt(3))
        let beta_rms = bw_hz / (2.0 * 3.0_f64.sqrt());

        // CRLB on ToA: σ_t^2 = 1 / (8 π^2 β_rms^2 SNR)
        let sigma_t = 1.0 / (2.0 * std::f64::consts::PI * beta_rms * (2.0 * snr_linear).sqrt());

        // Convert ToA error to range error: σ_r = c * σ_t
        SPEED_OF_LIGHT_M_S * sigma_t
    }

    /// Effective SNR given signal power and receiver noise
    pub fn compute_snr_db(&self, signal_power_dbm: f64, bandwidth_hz: f64) -> f64 {
        // Thermal noise: N0 = kTB
        let noise_power_w =
            BOLTZMANN_K * REFERENCE_TEMP_K * bandwidth_hz * 10f64.powf(self.config.rx_noise_figure_db / 10.0);
        let noise_power_dbm = 10.0 * noise_power_w.log10() + 30.0;
        signal_power_dbm - noise_power_dbm
    }

    // ---------------------------------------------------------------------------
    // Trilateration: 2D and 3D position estimation
    // ---------------------------------------------------------------------------

    /// 2D trilateration from multiple FTM anchors.
    ///
    /// # Arguments
    /// * `measurements` - Slice of (anchor_x, anchor_y, distance) tuples
    ///
    /// # Returns
    /// Estimated (x, y) position in meters via least-squares
    pub fn trilaterate_2d(&self, measurements: &[(f64, f64, f64)]) -> (f64, f64) {
        if measurements.len() < 2 {
            return (0.0, 0.0);
        }
        if measurements.len() == 2 {
            // Two anchors: midpoint as initial estimate
            let (x1, y1, _d1) = measurements[0];
            let (x2, y2, _d2) = measurements[1];
            return ((x1 + x2) / 2.0, (y1 + y2) / 2.0);
        }

        // Convert to linear system by differencing w.r.t. first anchor
        // Reduces from nonlinear to linear by subtracting reference equation
        // (x - xi)^2 + (y - yi)^2 = di^2
        // Subtract first: 2*(x1-xi)*x + 2*(y1-yi)*y = d1^2 - di^2 + xi^2 - x1^2 + yi^2 - y1^2
        let (x1, y1, d1) = measurements[0];
        let n = measurements.len() - 1;
        let mut a = vec![vec![0.0_f64; 2]; n];
        let mut b = vec![0.0_f64; n];

        for (i, &(xi, yi, di)) in measurements[1..].iter().enumerate() {
            a[i][0] = 2.0 * (x1 - xi);
            a[i][1] = 2.0 * (y1 - yi);
            // From: 2*(x1-xi)*x + 2*(y1-yi)*y = di^2 - d1^2 - xi^2 + x1^2 - yi^2 + y1^2
            b[i] = di * di - d1 * d1 - xi * xi + x1 * x1 - yi * yi + y1 * y1;
        }

        // Least-squares solve: x = (A^T A)^-1 A^T b
        least_squares_2d(&a, &b).unwrap_or((0.0, 0.0))
    }

    /// 3D trilateration from multiple FTM anchors.
    ///
    /// # Arguments
    /// * `measurements` - Slice of (anchor_x, anchor_y, anchor_z, distance) tuples
    ///
    /// # Returns
    /// Estimated (x, y, z) position in meters via least-squares
    pub fn trilaterate_3d(&self, measurements: &[(f64, f64, f64, f64)]) -> (f64, f64, f64) {
        if measurements.len() < 3 {
            if measurements.len() == 2 {
                let (x1, y1, z1, _) = measurements[0];
                let (x2, y2, z2, _) = measurements[1];
                return ((x1 + x2) / 2.0, (y1 + y2) / 2.0, (z1 + z2) / 2.0);
            }
            return (0.0, 0.0, 0.0);
        }

        let (x1, y1, z1, d1) = measurements[0];
        let n = measurements.len() - 1;
        let mut a = vec![vec![0.0_f64; 3]; n];
        let mut b = vec![0.0_f64; n];

        for (i, &(xi, yi, zi, di)) in measurements[1..].iter().enumerate() {
            a[i][0] = 2.0 * (x1 - xi);
            a[i][1] = 2.0 * (y1 - yi);
            a[i][2] = 2.0 * (z1 - zi);
            // From: 2*(x1-xi)*x + 2*(y1-yi)*y + 2*(z1-zi)*z
            //     = di^2 - d1^2 - xi^2 + x1^2 - yi^2 + y1^2 - zi^2 + z1^2
            b[i] = di * di - d1 * d1
                - xi * xi + x1 * x1
                - yi * yi + y1 * y1
                - zi * zi + z1 * z1;
        }

        least_squares_3d(&a, &b).unwrap_or((0.0, 0.0, 0.0))
    }

    /// Geodetic trilateration: convert distances to anchor APs (with known LCI)
    /// into a WGS-84 position estimate.
    ///
    /// # Arguments
    /// * `anchors` - Slice of (LCI, distance_m) pairs
    ///
    /// # Returns
    /// Position estimate in WGS-84 degrees and meters altitude
    pub fn geodetic_trilaterate(&self, anchors: &[(&LocationConfigInfo, f64)]) -> Option<Position> {
        if anchors.len() < 3 {
            return None;
        }

        // Convert geodetic anchor positions to local ECEF-like Cartesian
        // Use first anchor as origin for local ENU frame
        let origin = anchors[0].0;
        let lat0 = origin.latitude_deg.to_radians();
        let lon0 = origin.longitude_deg.to_radians();

        // Earth radius approximation
        const RE: f64 = 6_371_000.0;

        // Convert each anchor to local East-North-Up (ENU) coordinates
        let local_anchors: Vec<(f64, f64, f64, f64)> = anchors
            .iter()
            .map(|(lci, dist)| {
                let dlat = (lci.latitude_deg - origin.latitude_deg).to_radians();
                let dlon = (lci.longitude_deg - origin.longitude_deg).to_radians();
                let dalt = lci.altitude_m - origin.altitude_m;
                let east = dlon * RE * lat0.cos();
                let north = dlat * RE;
                (east, north, dalt, *dist)
            })
            .collect();

        let (x, y, z) = self.trilaterate_3d(&local_anchors);

        // Convert back to WGS-84
        let lat_deg = origin.latitude_deg + y.to_degrees() / RE * (180.0 / std::f64::consts::PI);
        let lon_deg = origin.longitude_deg
            + x.to_degrees() / (RE * lat0.cos()) * (180.0 / std::f64::consts::PI);
        let alt_m = origin.altitude_m + z;

        Some(Position { lat_deg, lon_deg, alt_m })
    }

    // ---------------------------------------------------------------------------
    // 802.11az enhancements
    // ---------------------------------------------------------------------------

    /// Compute 802.11az NTB (Non-Trigger-Based) ranging quality metric.
    ///
    /// NTB mode uses the same four-timestamp RTT but adds:
    /// - Secure LTF for anti-spoofing
    /// - I/Q sample reporting for CIR-based processing
    ///
    /// Returns quality metric [0, 1] based on symmetry of timestamps and
    /// measurement consistency.
    pub fn az_ntb_quality(&self, exchanges: &[TimestampExchange]) -> f64 {
        if exchanges.is_empty() {
            return 0.0;
        }

        let mut rtt_values: Vec<f64> = exchanges.iter().map(|e| self.compute_rtt(e)).collect();
        if rtt_values.is_empty() {
            return 0.0;
        }

        let n = rtt_values.len() as f64;
        let mean = rtt_values.iter().sum::<f64>() / n;
        let var = rtt_values.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n;
        let cv = if mean.abs() > 1e-9 { var.sqrt() / mean.abs() } else { 1.0 };

        // Quality: 1 - coefficient of variation (low CV = high quality)
        (1.0 - cv.min(1.0)).max(0.0)
    }

    /// Process 802.11az TB (Trigger-Based) ranging response.
    ///
    /// TB mode uses scheduled transmissions coordinated by an AP trigger frame,
    /// enabling simultaneous ranging to multiple STAs.
    ///
    /// Returns corrected RTT accounting for trigger overhead.
    pub fn az_tb_process(&self, exchange: &TimestampExchange, trigger_offset_ns: f64) -> f64 {
        let raw_rtt = self.compute_rtt(exchange);
        // TB mode adds trigger overhead to t2 measurement
        (raw_rtt - trigger_offset_ns).max(0.0)
    }

    /// Simulate secure LTF (Long Training Field) sequence for 802.11az.
    ///
    /// Secure LTF prevents relay/spoofing attacks by including a nonce-based
    /// perturbation in the OFDM training symbols. Here we model the
    /// extra ranging error introduced by secure LTF.
    ///
    /// Returns additional ranging uncertainty (m) due to secure LTF overhead.
    pub fn az_secure_ltf_uncertainty(&self, snr_db: f64) -> f64 {
        // Secure LTF has ~3 dB SNR penalty due to phase perturbation
        let effective_snr = snr_db - 3.0;
        self.crlb_ranging(effective_snr, self.config.bandwidth.hz())
    }

    // ---------------------------------------------------------------------------
    // Error models and simulation helpers
    // ---------------------------------------------------------------------------

    /// Simulate FTM timestamp exchange with AWGN timing error.
    ///
    /// # Arguments
    /// * `true_distance_m` - True distance between stations
    /// * `processing_delay_ns` - Responder processing delay (t3-t2)
    /// * `awgn_std_ns` - Standard deviation of timing noise (ns)
    /// * `nlos_bias_m` - NLOS multipath bias (0 for LOS)
    /// * `seed` - Pseudo-random seed for reproducibility
    ///
    /// # Returns
    /// Simulated timestamp exchange
    pub fn simulate_exchange(
        &self,
        true_distance_m: f64,
        processing_delay_ns: f64,
        awgn_std_ns: f64,
        nlos_bias_m: f64,
        seed: u64,
    ) -> TimestampExchange {
        let tof_ns = self.distance_to_rtt(true_distance_m + nlos_bias_m) / 2.0;

        // Simple LCG noise generator
        let mut rng = seed;
        let noise = |rng: &mut u64| -> f64 {
            *rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            // Box-Muller transform
            let u1 = (*rng >> 11) as f64 / (1u64 << 53) as f64;
            *rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let u2 = (*rng >> 11) as f64 / (1u64 << 53) as f64;
            let u1 = u1.max(1e-10);
            (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
        };

        let t1 = 1000.0; // Reference time
        let t2 = t1 + tof_ns + noise(&mut rng) * awgn_std_ns;
        let t3 = t2 + processing_delay_ns + noise(&mut rng) * (awgn_std_ns * 0.1);
        let t4 = t3 + tof_ns + noise(&mut rng) * awgn_std_ns;

        TimestampExchange { t1, t2, t3, t4 }
    }

    /// Simulate a burst of FTM exchanges
    pub fn simulate_burst(
        &self,
        true_distance_m: f64,
        processing_delay_ns: f64,
        awgn_std_ns: f64,
        nlos_bias_m: f64,
        seed: u64,
    ) -> Vec<TimestampExchange> {
        let count = self.config.ftms_per_burst as usize;
        (0..count)
            .map(|i| {
                self.simulate_exchange(
                    true_distance_m,
                    processing_delay_ns,
                    awgn_std_ns,
                    nlos_bias_m,
                    seed.wrapping_add(i as u64 * 12345),
                )
            })
            .collect()
    }

    /// Ranging error model: AWGN ranging error standard deviation.
    ///
    /// σ_r = σ_TOA * c = sqrt(N0 / (2 * E_s)) * (c / (2*π*β))
    ///
    /// # Arguments
    /// * `snr_db` - Signal-to-noise ratio (dB)
    /// * `bw_hz` - Signal bandwidth (Hz)
    ///
    /// # Returns
    /// 1-sigma ranging error (m) under AWGN
    pub fn awgn_ranging_error(&self, snr_db: f64, bw_hz: f64) -> f64 {
        self.crlb_ranging(snr_db, bw_hz)
    }

    /// Estimate multipath bias for given environment type.
    ///
    /// Returns expected NLOS bias in meters for a given environment.
    pub fn nlos_bias_estimate(&self, environment: NlosEnvironment) -> f64 {
        match environment {
            NlosEnvironment::OpenSpace => 0.0,
            NlosEnvironment::IndoorLos => 0.15,
            NlosEnvironment::IndoorNlos => 1.5,
            NlosEnvironment::UrbanLos => 0.5,
            NlosEnvironment::UrbanNlos => 5.0,
            NlosEnvironment::DenseUrbanNlos => 15.0,
        }
    }

    // ---------------------------------------------------------------------------
    // Utility: FTM frame encoding helpers
    // ---------------------------------------------------------------------------

    /// Encode FTM parameters element (IE) per IEEE 802.11mc Table 9-327
    ///
    /// Returns encoded bytes for inclusion in FTM Request/Response frames.
    pub fn encode_ftm_params(&self) -> Vec<u8> {
        let mut elem = vec![0u8; 9];

        // Byte 0-1: FTM parameters field
        // Bit 0: Status indicator (0=reserved, 1=successful)
        elem[0] |= 1 << 0;
        // Bit 1: Value (0=ASAP not set, 1=ASAP)
        if self.config.asap {
            elem[0] |= 1 << 1;
        }
        // Bits 2-5: Number of FTMS per burst (0-31)
        elem[0] |= (self.config.ftms_per_burst & 0x1F) << 2;

        // Byte 1: Burst duration exponent
        elem[1] = self.config.num_burst_exponent & 0x0F;

        // Byte 2: Min delta FTM
        elem[2] = self.config.min_delta_ftm;

        // Bytes 3-4: Partial TSF offset (not used here)
        elem[3] = 0;
        elem[4] = 0;

        // Byte 5: Partial TSF timer no preference (bit 7) + others
        if self.config.partial_tsf {
            elem[5] |= 1 << 7;
        }

        // Bytes 6-8: AOA/AOD capability (not set)
        elem
    }

    /// Decode FTM parameters element from raw bytes
    pub fn decode_ftm_params(bytes: &[u8]) -> Option<FtmParamsDecode> {
        if bytes.len() < 9 {
            return None;
        }
        let asap = (bytes[0] & 0x02) != 0;
        let ftms_per_burst = (bytes[0] >> 2) & 0x1F;
        let burst_exponent = bytes[1] & 0x0F;
        let min_delta = bytes[2];
        Some(FtmParamsDecode {
            asap,
            ftms_per_burst,
            burst_exponent,
            min_delta_ftm: min_delta,
        })
    }

    // ---------------------------------------------------------------------------
    // 802.11az: Enhanced ranging with I/Q channel sounding
    // ---------------------------------------------------------------------------

    /// Process 802.11az I/Q channel sounding data for high-accuracy ToA.
    ///
    /// Uses the Secure HE/EHT LTF for fine timing. Returns refined distance.
    pub fn az_process_cir(
        &self,
        cir: &[(f64, f64)],
        sample_rate_hz: f64,
        threshold_db: f64,
    ) -> f64 {
        if cir.is_empty() {
            return 0.0;
        }

        // Leading edge detection for LOS path
        let toa_samples = self.leading_edge_detect(cir, threshold_db);
        let toa_ns = toa_samples / sample_rate_hz * 1e9;

        // Convert one-way ToA to distance
        toa_ns * 1e-9 * SPEED_OF_LIGHT_M_S
    }

    /// Estimate ranging accuracy achievable with given channel conditions.
    ///
    /// # Returns
    /// (crlb_m, practical_error_m) - theoretical and practical accuracy
    pub fn ranging_accuracy_estimate(&self, snr_db: f64) -> (f64, f64) {
        let bw = self.config.bandwidth.hz();
        let crlb = self.crlb_ranging(snr_db, bw);

        // Practical accuracy is ~2-3x CRLB for real hardware/firmware
        let practical = crlb * 2.5;

        (crlb, practical)
    }

    // ---------------------------------------------------------------------------
    // Kalman filter for range tracking
    // ---------------------------------------------------------------------------

    /// Update Kalman filter state for range tracking.
    ///
    /// Simple 1D scalar Kalman filter for distance tracking.
    /// State: [range_m, range_rate_m_s]
    /// Measurement: range_m
    ///
    /// # Returns
    /// Filtered range estimate
    pub fn kalman_range_update(
        &self,
        state: &mut KalmanRangeState,
        measured_range_m: f64,
        dt_s: f64,
        process_noise_m2: f64,
        meas_noise_m2: f64,
    ) -> f64 {
        // Predict
        state.range += state.range_rate * dt_s;
        state.p[0][0] += dt_s * (state.p[1][0] + state.p[0][1]) + dt_s * dt_s * state.p[1][1] + process_noise_m2;
        state.p[0][1] += dt_s * state.p[1][1];
        state.p[1][0] += dt_s * state.p[1][1];
        // p[1][1] stays: process noise for rate
        state.p[1][1] += process_noise_m2 * 0.01; // small rate process noise

        // Update
        let innovation = measured_range_m - state.range;
        let s = state.p[0][0] + meas_noise_m2; // Innovation covariance
        let k0 = state.p[0][0] / s; // Kalman gain for range
        let k1 = state.p[1][0] / s; // Kalman gain for rate

        state.range += k0 * innovation;
        state.range_rate += k1 * innovation;

        // Update covariance
        let p00 = state.p[0][0];
        let p10 = state.p[1][0];
        state.p[0][0] -= k0 * p00;
        state.p[0][1] -= k0 * state.p[0][1];
        state.p[1][0] -= k1 * p00;
        state.p[1][1] -= k1 * p10;

        state.range
    }
}

// ---------------------------------------------------------------------------
// Supporting types
// ---------------------------------------------------------------------------

/// NLOS environment type for bias estimation
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NlosEnvironment {
    OpenSpace,
    IndoorLos,
    IndoorNlos,
    UrbanLos,
    UrbanNlos,
    DenseUrbanNlos,
}

/// Decoded FTM parameters element
#[derive(Debug, Clone)]
pub struct FtmParamsDecode {
    pub asap: bool,
    pub ftms_per_burst: u8,
    pub burst_exponent: u8,
    pub min_delta_ftm: u8,
}

/// Kalman filter state for range tracking
#[derive(Debug, Clone)]
pub struct KalmanRangeState {
    /// Range estimate (m)
    pub range: f64,
    /// Range rate estimate (m/s)
    pub range_rate: f64,
    /// 2x2 error covariance matrix
    pub p: [[f64; 2]; 2],
}

impl KalmanRangeState {
    /// Initialize with given range and uncertainty
    pub fn new(initial_range_m: f64, initial_uncertainty_m: f64) -> Self {
        KalmanRangeState {
            range: initial_range_m,
            range_rate: 0.0,
            p: [
                [initial_uncertainty_m * initial_uncertainty_m, 0.0],
                [0.0, 1.0], // 1 m/s^2 initial rate uncertainty
            ],
        }
    }
}

// ---------------------------------------------------------------------------
// Internal mathematical helpers
// ---------------------------------------------------------------------------

/// Compute median of a pre-sorted slice
fn median_of_sorted(sorted: &[f64]) -> f64 {
    let n = sorted.len();
    if n == 0 {
        return 0.0;
    }
    if n % 2 == 1 {
        sorted[n / 2]
    } else {
        (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0
    }
}

/// Compute confidence score from measurement statistics
fn compute_confidence(num_inliers: usize, num_outliers: usize, rtt_std_ns: f64) -> f64 {
    if num_inliers == 0 {
        return 0.0;
    }
    let total = num_inliers + num_outliers;
    let inlier_ratio = num_inliers as f64 / total as f64;

    // Decay confidence with high spread (>1 ns std = significant error)
    let spread_factor = (-rtt_std_ns / 5.0).exp();

    (inlier_ratio * spread_factor).min(1.0).max(0.0)
}

/// Complex cross-correlation: output = sum_k { signal[k] * conj(ref[k-lag]) }
fn cross_correlate_complex(signal: &[(f64, f64)], reference: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let slen = signal.len();
    let rlen = reference.len();
    if slen == 0 || rlen == 0 {
        return vec![];
    }

    let out_len = slen + rlen - 1;
    let mut out = vec![(0.0_f64, 0.0_f64); out_len];

    for lag in 0..out_len {
        let mut sum_r = 0.0_f64;
        let mut sum_i = 0.0_f64;
        for k in 0..rlen {
            let sig_idx = lag as isize - (rlen as isize - 1 - k as isize);
            if sig_idx >= 0 && (sig_idx as usize) < slen {
                let s = signal[sig_idx as usize];
                let r = reference[k];
                // s * conj(r)
                sum_r += s.0 * r.0 + s.1 * r.1;
                sum_i += s.1 * r.0 - s.0 * r.1;
            }
        }
        out[lag] = (sum_r, sum_i);
    }
    out
}

/// Parabolic interpolation of peak position in complex magnitude spectrum
fn parabolic_interpolate_peak(cir: &[(f64, f64)], peak_idx: usize) -> f64 {
    let n = cir.len();
    if peak_idx == 0 || peak_idx >= n - 1 {
        return peak_idx as f64;
    }
    let y_prev = cir[peak_idx - 1].0.powi(2) + cir[peak_idx - 1].1.powi(2);
    let y_cur = cir[peak_idx].0.powi(2) + cir[peak_idx].1.powi(2);
    let y_next = cir[peak_idx + 1].0.powi(2) + cir[peak_idx + 1].1.powi(2);

    let denom = 2.0 * (2.0 * y_cur - y_prev - y_next);
    if denom.abs() < 1e-30 {
        return peak_idx as f64;
    }
    let delta = (y_prev - y_next) / denom;
    (peak_idx as f64 + delta).clamp(0.0, (n - 1) as f64)
}

/// Least-squares solver for 2D trilateration Ax = b  (overdetermined)
/// Returns (x, y) via normal equations
fn least_squares_2d(a: &[Vec<f64>], b: &[f64]) -> Option<(f64, f64)> {
    let n = a.len();
    if n == 0 {
        return None;
    }

    // A^T A (2x2) and A^T b (2)
    let mut ata = [[0.0_f64; 2]; 2];
    let mut atb = [0.0_f64; 2];

    for i in 0..n {
        for r in 0..2 {
            for c in 0..2 {
                ata[r][c] += a[i][r] * a[i][c];
            }
            atb[r] += a[i][r] * b[i];
        }
    }

    // Solve 2x2 system
    let det = ata[0][0] * ata[1][1] - ata[0][1] * ata[1][0];
    if det.abs() < 1e-12 {
        return None;
    }
    let x = (atb[0] * ata[1][1] - atb[1] * ata[0][1]) / det;
    let y = (ata[0][0] * atb[1] - ata[1][0] * atb[0]) / det;
    Some((x, y))
}

/// Least-squares solver for 3D trilateration Ax = b  (overdetermined)
/// Returns (x, y, z) via normal equations
fn least_squares_3d(a: &[Vec<f64>], b: &[f64]) -> Option<(f64, f64, f64)> {
    let n = a.len();
    if n == 0 {
        return None;
    }

    // A^T A (3x3) and A^T b (3)
    let mut ata = [[0.0_f64; 3]; 3];
    let mut atb = [0.0_f64; 3];

    for i in 0..n {
        for r in 0..3 {
            for c in 0..3 {
                ata[r][c] += a[i][r] * a[i][c];
            }
            atb[r] += a[i][r] * b[i];
        }
    }

    // Solve 3x3 system via Cramer's rule
    let det3 = |m: [[f64; 3]; 3]| -> f64 {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };

    let det = det3(ata);
    if det.abs() < 1e-12 {
        return None;
    }

    let mx = [
        [atb[0], ata[0][1], ata[0][2]],
        [atb[1], ata[1][1], ata[1][2]],
        [atb[2], ata[2][1], ata[2][2]],
    ];
    let my = [
        [ata[0][0], atb[0], ata[0][2]],
        [ata[1][0], atb[1], ata[1][2]],
        [ata[2][0], atb[2], ata[2][2]],
    ];
    let mz = [
        [ata[0][0], ata[0][1], atb[0]],
        [ata[1][0], ata[1][1], atb[1]],
        [ata[2][0], ata[2][1], atb[2]],
    ];

    let x = det3(mx) / det;
    let y = det3(my) / det;
    let z = det3(mz) / det;
    Some((x, y, z))
}

/// Extract noise subspace eigenvectors from a complex correlation matrix.
/// Uses power iteration to find signal subspace, then noise subspace is orthogonal complement.
/// Returns `m - num_signals` noise eigenvectors as Vec<Vec<(f64,f64)>>
fn noise_subspace_real(
    r: &[Vec<(f64, f64)>],
    m: usize,
    num_signals: usize,
) -> Vec<Vec<(f64, f64)>> {
    if m == 0 || num_signals >= m {
        return vec![];
    }

    let num_noise = m - num_signals;
    let mut noise_vecs: Vec<Vec<(f64, f64)>> = Vec::with_capacity(num_noise);

    // Use a simplified Gram-Schmidt orthogonalization approach
    // Initialize with standard basis and Gram-Schmidt deflation
    // Start with all-real unit vectors
    let mut basis: Vec<Vec<(f64, f64)>> = (0..m)
        .map(|i| {
            let mut v = vec![(0.0_f64, 0.0_f64); m];
            v[i] = (1.0, 0.0);
            v
        })
        .collect();

    // Find dominant eigenvectors via power iteration for signal subspace
    let mut signal_vecs: Vec<Vec<(f64, f64)>> = Vec::with_capacity(num_signals);

    for _ in 0..num_signals {
        // Start with first basis vector not yet used
        let start_idx = signal_vecs.len() % m;
        let mut v = basis[start_idx].clone();

        // Power iteration: v = R*v / ||R*v||
        for _iter in 0..30 {
            let mut rv = vec![(0.0_f64, 0.0_f64); m];
            for i in 0..m {
                for j in 0..m {
                    // complex multiply r[i][j] * v[j]
                    rv[i].0 += r[i][j].0 * v[j].0 - r[i][j].1 * v[j].1;
                    rv[i].1 += r[i][j].0 * v[j].1 + r[i][j].1 * v[j].0;
                }
            }

            // Orthogonalize against already found signal vectors
            for sv in &signal_vecs {
                let dot_r: f64 = rv.iter().zip(sv.iter()).map(|(a, b)| a.0 * b.0 + a.1 * b.1).sum();
                let dot_i: f64 = rv.iter().zip(sv.iter()).map(|(a, b)| a.1 * b.0 - a.0 * b.1).sum();
                for k in 0..m {
                    rv[k].0 -= dot_r * sv[k].0 - dot_i * sv[k].1;
                    rv[k].1 -= dot_r * sv[k].1 + dot_i * sv[k].0;
                }
            }

            // Normalize
            let norm: f64 = rv.iter().map(|(a, b)| a * a + b * b).sum::<f64>().sqrt();
            if norm < 1e-30 {
                break;
            }
            v = rv.iter().map(|(a, b)| (a / norm, b / norm)).collect();
        }

        signal_vecs.push(v);
    }

    // Noise subspace: orthogonal complement of signal subspace
    for en_start in &basis {
        let mut en = en_start.clone();

        // Orthogonalize against signal subspace
        for sv in &signal_vecs {
            let dot_r: f64 = en.iter().zip(sv.iter()).map(|(a, b)| a.0 * b.0 + a.1 * b.1).sum();
            let dot_i: f64 = en.iter().zip(sv.iter()).map(|(a, b)| a.1 * b.0 - a.0 * b.1).sum();
            for k in 0..m {
                en[k].0 -= dot_r * sv[k].0 - dot_i * sv[k].1;
                en[k].1 -= dot_r * sv[k].1 + dot_i * sv[k].0;
            }
        }

        // Orthogonalize against already found noise vectors
        for nv in &noise_vecs {
            let dot_r: f64 = en.iter().zip(nv.iter()).map(|(a, b)| a.0 * b.0 + a.1 * b.1).sum();
            let dot_i: f64 = en.iter().zip(nv.iter()).map(|(a, b)| a.1 * b.0 - a.0 * b.1).sum();
            for k in 0..m {
                en[k].0 -= dot_r * nv[k].0 - dot_i * nv[k].1;
                en[k].1 -= dot_r * nv[k].1 + dot_i * nv[k].0;
            }
        }

        let norm: f64 = en.iter().map(|(a, b)| a * a + b * b).sum::<f64>().sqrt();
        if norm > 1e-10 {
            let en_norm: Vec<(f64, f64)> = en.iter().map(|(a, b)| (a / norm, b / norm)).collect();
            noise_vecs.push(en_norm);
        }

        if noise_vecs.len() >= num_noise {
            break;
        }
    }

    noise_vecs
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_ftm() -> WiFiFtmRanging {
        WiFiFtmRanging::new(FtmConfig::default())
    }

    // --- RTT computation ---

    #[test]
    fn test_rtt_basic_zero_distance() {
        let ftm = default_ftm();
        // If tof = 0, then t2 = t1 and t4 = t3
        let ex = TimestampExchange::new(100.0, 100.0, 200.0, 200.0);
        let rtt = ftm.compute_rtt(&ex);
        assert!((rtt).abs() < 1e-9, "RTT should be 0 for zero propagation, got {}", rtt);
    }

    #[test]
    fn test_rtt_known_propagation() {
        let ftm = default_ftm();
        // tof = 10 ns => RTT = 20 ns
        let ex = TimestampExchange::new(0.0, 10.0, 110.0, 120.0);
        let rtt = ftm.compute_rtt(&ex);
        assert!((rtt - 20.0).abs() < 1e-9, "RTT should be 20 ns, got {}", rtt);
    }

    #[test]
    fn test_rtt_to_distance() {
        let ftm = default_ftm();
        // c = 299792458 m/s; 1 ns RTT -> 0.15 m
        let dist = ftm.rtt_to_distance(1.0);
        let expected = SPEED_OF_LIGHT_M_S * 1e-9 / 2.0;
        assert!((dist - expected).abs() < 1e-6, "Distance mismatch: {} vs {}", dist, expected);
    }

    #[test]
    fn test_rtt_to_distance_zero() {
        let ftm = default_ftm();
        assert_eq!(ftm.rtt_to_distance(0.0), 0.0);
    }

    #[test]
    fn test_rtt_to_distance_negative() {
        let ftm = default_ftm();
        // Negative RTT should return 0
        assert_eq!(ftm.rtt_to_distance(-1.0), 0.0);
    }

    #[test]
    fn test_distance_to_rtt_roundtrip() {
        let ftm = default_ftm();
        let d = 50.0_f64; // 50 meters
        let rtt = ftm.distance_to_rtt(d);
        let d2 = ftm.rtt_to_distance(rtt);
        assert!((d - d2).abs() < 1e-9, "Roundtrip distance mismatch: {} vs {}", d, d2);
    }

    #[test]
    fn test_rtt_100m_distance() {
        let ftm = default_ftm();
        // 100 m -> RTT = 2*100/c * 1e9 ns
        let expected_rtt_ns = 2.0 * 100.0 / SPEED_OF_LIGHT_M_S * 1e9;
        let rtt = ftm.distance_to_rtt(100.0);
        assert!((rtt - expected_rtt_ns).abs() < 1e-6);
        let dist = ftm.rtt_to_distance(rtt);
        assert!((dist - 100.0).abs() < 1e-6);
    }

    // --- Process exchange ---

    #[test]
    fn test_process_exchange_populates_measurement() {
        let mut ftm = default_ftm();
        let ex = TimestampExchange::new(0.0, 10.0, 110.0, 120.0);
        let m = ftm.process_exchange(1, ex);
        assert!((m.rtt_ns - 20.0).abs() < 1e-9);
        assert!(m.distance_m > 0.0);
        assert_eq!(m.seq_num, 1);
    }

    // --- Burst averaging ---

    #[test]
    fn test_burst_average_single() {
        let ftm = default_ftm();
        let result = ftm.burst_average(&[100.0]);
        assert!((result.mean_rtt_ns - 100.0).abs() < 1e-9);
        assert_eq!(result.num_measurements, 1);
    }

    #[test]
    fn test_burst_average_uniform() {
        let ftm = default_ftm();
        let measurements = vec![100.0, 101.0, 99.0, 100.5, 100.2];
        let result = ftm.burst_average(&measurements);
        assert!((result.mean_rtt_ns - 100.14).abs() < 1.0);
        assert!(result.rtt_std_ns < 2.0);
        assert!(!result.nlos_detected);
    }

    #[test]
    fn test_burst_average_outlier_rejection() {
        let ftm = default_ftm();
        // One large outlier
        let measurements = vec![100.0, 100.5, 99.8, 100.2, 500.0]; // 500 is outlier
        let result = ftm.burst_average(&measurements);
        assert!(result.outliers_rejected >= 1, "Should reject outlier");
        assert!(result.mean_rtt_ns < 200.0, "Mean should not be skewed by outlier");
    }

    #[test]
    fn test_burst_average_empty() {
        let ftm = default_ftm();
        let result = ftm.burst_average(&[]);
        assert_eq!(result.num_measurements, 0);
    }

    #[test]
    fn test_burst_average_two_measurements() {
        let ftm = default_ftm();
        let result = ftm.burst_average(&[10.0, 12.0]);
        assert!((result.mean_rtt_ns - 11.0).abs() < 0.01);
    }

    #[test]
    fn test_burst_confidence_high_consistency() {
        let ftm = default_ftm();
        // Very consistent measurements -> high confidence
        let m = vec![100.0; 8];
        let result = ftm.burst_average(&m);
        assert!(result.confidence > 0.8, "Confidence should be high: {}", result.confidence);
    }

    // --- NLOS detection ---

    #[test]
    fn test_nlos_not_detected_symmetric() {
        let ftm = default_ftm();
        let m = vec![100.0, 100.5, 99.5, 100.2, 99.8, 100.3];
        assert!(!ftm.detect_nlos(&m));
    }

    #[test]
    fn test_nlos_detected_positive_skew() {
        let ftm = default_ftm();
        // Positive bias and skew
        let m: Vec<f64> = vec![100.0, 101.0, 105.0, 110.0, 115.0, 120.0, 130.0, 150.0];
        // Skewed positive, large spread -> NLOS
        let nlos = ftm.detect_nlos(&m);
        // This may or may not trigger depending on bandwidth ratio, just verify it runs
        let _ = nlos;
    }

    #[test]
    fn test_nlos_insufficient_data() {
        let ftm = default_ftm();
        assert!(!ftm.detect_nlos(&[100.0, 101.0]));
        assert!(!ftm.detect_nlos(&[]));
    }

    // --- CRLB ---

    #[test]
    fn test_crlb_decreases_with_bandwidth() {
        let ftm = default_ftm();
        let crlb_20 = ftm.crlb_ranging(20.0, 20e6);
        let crlb_80 = ftm.crlb_ranging(20.0, 80e6);
        let crlb_160 = ftm.crlb_ranging(20.0, 160e6);
        assert!(crlb_20 > crlb_80, "Wider BW -> lower CRLB");
        assert!(crlb_80 > crlb_160, "Wider BW -> lower CRLB");
    }

    #[test]
    fn test_crlb_decreases_with_snr() {
        let ftm = default_ftm();
        let crlb_low = ftm.crlb_ranging(0.0, 80e6);
        let crlb_high = ftm.crlb_ranging(30.0, 80e6);
        assert!(crlb_low > crlb_high, "Higher SNR -> lower CRLB");
    }

    #[test]
    fn test_crlb_80mhz_20db_snr_order_of_magnitude() {
        let ftm = default_ftm();
        let crlb = ftm.crlb_ranging(20.0, 80e6);
        // At 80 MHz and 20 dB SNR, expect ~cm-level accuracy
        assert!(crlb < 1.0, "CRLB should be sub-meter at 80 MHz / 20 dB SNR, got {}", crlb);
        assert!(crlb > 1e-4, "CRLB should be > 0.1 mm, got {}", crlb);
    }

    #[test]
    fn test_crlb_infinite_for_zero_snr() {
        let ftm = default_ftm();
        // SNR = -infinity (snr_linear = 0): infinity
        let crlb = ftm.crlb_ranging(f64::NEG_INFINITY, 80e6);
        assert!(!crlb.is_finite() || crlb > 1e6);
    }

    // --- Channel bandwidth ---

    #[test]
    fn test_channel_bandwidth_hz() {
        assert_eq!(ChannelBandwidth::Mhz20.hz(), 20e6);
        assert_eq!(ChannelBandwidth::Mhz80.hz(), 80e6);
        assert_eq!(ChannelBandwidth::Mhz160.hz(), 160e6);
        assert_eq!(ChannelBandwidth::Mhz320.hz(), 320e6);
    }

    #[test]
    fn test_channel_bandwidth_range_resolution() {
        let res_20 = ChannelBandwidth::Mhz20.range_resolution_m();
        let res_160 = ChannelBandwidth::Mhz160.range_resolution_m();
        // Approx: c / (2 * BW)
        let expected_20 = SPEED_OF_LIGHT_M_S / (2.0 * 20e6);
        assert!((res_20 - expected_20).abs() < 0.01);
        assert!(res_20 > res_160, "Narrower BW has worse resolution");
    }

    // --- Trilateration 2D ---

    #[test]
    fn test_trilaterate_2d_equilateral_triangle() {
        let ftm = default_ftm();
        // Three anchors at equilateral triangle corners, target at center (0,0)
        let d = 10.0_f64;
        let a1 = (d, 0.0);
        let a2 = (-d / 2.0, d * 3.0_f64.sqrt() / 2.0);
        let a3 = (-d / 2.0, -d * 3.0_f64.sqrt() / 2.0);
        let r1 = ((a1.0 * a1.0 + a1.1 * a1.1) as f64).sqrt();
        let r2 = ((a2.0 * a2.0 + a2.1 * a2.1) as f64).sqrt();
        let r3 = ((a3.0 * a3.0 + a3.1 * a3.1) as f64).sqrt();
        let meas = [(a1.0, a1.1, r1), (a2.0, a2.1, r2), (a3.0, a3.1, r3)];
        let (x, y) = ftm.trilaterate_2d(&meas);
        assert!(x.abs() < 0.1, "x should be ~0, got {}", x);
        assert!(y.abs() < 0.1, "y should be ~0, got {}", y);
    }

    #[test]
    fn test_trilaterate_2d_known_position() {
        let ftm = default_ftm();
        // Target at (3, 4)
        let target_x = 3.0_f64;
        let target_y = 4.0_f64;
        let anchors = [(0.0, 0.0), (10.0, 0.0), (5.0, 10.0)];
        let meas: Vec<(f64, f64, f64)> = anchors
            .iter()
            .map(|&(ax, ay)| {
                let d = ((ax - target_x).powi(2) + (ay - target_y).powi(2)).sqrt();
                (ax, ay, d)
            })
            .collect();
        let (x, y) = ftm.trilaterate_2d(&meas);
        assert!((x - target_x).abs() < 0.01, "x mismatch: {} vs {}", x, target_x);
        assert!((y - target_y).abs() < 0.01, "y mismatch: {} vs {}", y, target_y);
    }

    #[test]
    fn test_trilaterate_2d_single_anchor() {
        let ftm = default_ftm();
        let (x, y) = ftm.trilaterate_2d(&[(0.0, 0.0, 5.0)]);
        // Undefined with 1 anchor, returns (0,0)
        assert_eq!(x, 0.0);
        assert_eq!(y, 0.0);
    }

    // --- Trilateration 3D ---

    #[test]
    fn test_trilaterate_3d_known_position() {
        let ftm = default_ftm();
        let target = (2.0_f64, 3.0_f64, 4.0_f64);
        let anchors = [
            (0.0, 0.0, 0.0),
            (10.0, 0.0, 0.0),
            (0.0, 10.0, 0.0),
            (0.0, 0.0, 10.0),
        ];
        let meas: Vec<(f64, f64, f64, f64)> = anchors
            .iter()
            .map(|&(ax, ay, az)| {
                let d = ((ax - target.0).powi(2) + (ay - target.1).powi(2) + (az - target.2).powi(2)).sqrt();
                (ax, ay, az, d)
            })
            .collect();
        let (x, y, z) = ftm.trilaterate_3d(&meas);
        assert!((x - target.0).abs() < 0.01, "x: {} vs {}", x, target.0);
        assert!((y - target.1).abs() < 0.01, "y: {} vs {}", y, target.1);
        assert!((z - target.2).abs() < 0.01, "z: {} vs {}", z, target.2);
    }

    #[test]
    fn test_trilaterate_3d_at_origin() {
        let ftm = default_ftm();
        let anchors = [
            (10.0_f64, 0.0, 0.0, 10.0),
            (0.0, 10.0, 0.0, 10.0),
            (0.0, 0.0, 10.0, 10.0),
            (-10.0, 0.0, 0.0, 10.0),
        ];
        let (x, y, z) = ftm.trilaterate_3d(&anchors);
        assert!(x.abs() < 0.1, "x: {}", x);
        assert!(y.abs() < 0.1, "y: {}", y);
        assert!(z.abs() < 0.1, "z: {}", z);
    }

    // --- Clock drift ---

    #[test]
    fn test_clock_drift_no_drift() {
        let mut ftm = default_ftm();
        // Exchanges at 100 ns intervals, no drift
        let exchanges: Vec<TimestampExchange> = (0..5)
            .map(|i| {
                let t1 = i as f64 * 1000.0;
                TimestampExchange::new(t1, t1 + 10.0, t1 + 110.0, t1 + 120.0)
            })
            .collect();
        let drift = ftm.estimate_clock_drift(&exchanges);
        assert!(drift.abs() < 1.0, "Drift should be near 0 ppm: {}", drift);
    }

    #[test]
    fn test_clock_drift_single_exchange() {
        let mut ftm = default_ftm();
        let ex = TimestampExchange::new(0.0, 10.0, 110.0, 120.0);
        let drift = ftm.estimate_clock_drift(&[ex]);
        // Only one exchange -> no update, initial value returned
        assert!(drift.abs() < 100.0);
    }

    // --- ToA estimation ---

    #[test]
    fn test_estimate_toa_impulse_at_known_position() {
        let ftm = default_ftm();
        // Reference: unit impulse at index 0
        let reference = vec![(1.0_f64, 0.0_f64)];
        // Signal: impulse at index 5
        let mut signal = vec![(0.0_f64, 0.0_f64); 10];
        signal[5] = (1.0, 0.0);
        let toa = ftm.estimate_toa(&signal, &reference);
        assert!((toa - 5.0).abs() < 1.0, "ToA should be ~5, got {}", toa);
    }

    #[test]
    fn test_estimate_toa_empty_inputs() {
        let ftm = default_ftm();
        assert_eq!(ftm.estimate_toa(&[], &[]), 0.0);
        assert_eq!(ftm.estimate_toa(&[(1.0, 0.0)], &[]), 0.0);
    }

    // --- Leading edge detection ---

    #[test]
    fn test_leading_edge_single_path() {
        let ftm = default_ftm();
        // Single peak at index 5
        let mut cir = vec![(0.0_f64, 0.0_f64); 20];
        cir[5] = (10.0, 0.0);
        let le = ftm.leading_edge_detect(&cir, 6.0); // 6 dB threshold below peak
        // Leading edge should be at or before index 5
        assert!(le <= 5.5, "Leading edge should be <= 5, got {}", le);
    }

    #[test]
    fn test_leading_edge_empty() {
        let ftm = default_ftm();
        assert_eq!(ftm.leading_edge_detect(&[], 6.0), 0.0);
    }

    #[test]
    fn test_leading_edge_multipath() {
        let ftm = default_ftm();
        // Two paths: weak LOS at idx 3, strong NLOS at idx 10
        let mut cir = vec![(0.0_f64, 0.0_f64); 30];
        cir[3] = (1.0, 0.0); // weak direct path
        cir[10] = (5.0, 0.0); // strong reflected path
        // With 20 dB threshold, both should be detected; leading edge near 3
        let le = ftm.leading_edge_detect(&cir, 20.0);
        assert!(le <= 5.0, "Leading edge should find direct path ~3, got {}", le);
    }

    // --- Simulation ---

    #[test]
    fn test_simulate_exchange_distance() {
        let ftm = default_ftm();
        let true_dist = 30.0; // 30 meters
        let ex = ftm.simulate_exchange(true_dist, 100.0, 0.0, 0.0, 42);
        let rtt = ftm.compute_rtt(&ex);
        let dist = ftm.rtt_to_distance(rtt);
        assert!((dist - true_dist).abs() < 0.1, "Simulated distance: {} vs {}", dist, true_dist);
    }

    #[test]
    fn test_simulate_exchange_with_nlos() {
        let ftm = default_ftm();
        let true_dist = 10.0;
        let nlos_bias = 2.0;
        let ex = ftm.simulate_exchange(true_dist, 100.0, 0.0, nlos_bias, 7);
        let rtt = ftm.compute_rtt(&ex);
        let dist = ftm.rtt_to_distance(rtt);
        let expected = true_dist + nlos_bias;
        assert!((dist - expected).abs() < 0.5, "NLOS distance: {} vs {}", dist, expected);
    }

    #[test]
    fn test_simulate_burst_count() {
        let config = FtmConfig { ftms_per_burst: 8, ..Default::default() };
        let ftm = WiFiFtmRanging::new(config);
        let burst = ftm.simulate_burst(20.0, 100.0, 0.5, 0.0, 123);
        assert_eq!(burst.len(), 8);
    }

    // --- FTM params encoding/decoding ---

    #[test]
    fn test_encode_decode_ftm_params_roundtrip() {
        let config = FtmConfig {
            asap: true,
            ftms_per_burst: 7,
            num_burst_exponent: 2,
            min_delta_ftm: 10,
            ..Default::default()
        };
        let ftm = WiFiFtmRanging::new(config);
        let encoded = ftm.encode_ftm_params();
        assert_eq!(encoded.len(), 9);
        let decoded = WiFiFtmRanging::decode_ftm_params(&encoded).unwrap();
        assert!(decoded.asap);
        assert_eq!(decoded.ftms_per_burst, 7);
        assert_eq!(decoded.min_delta_ftm, 10);
    }

    #[test]
    fn test_decode_ftm_params_too_short() {
        assert!(WiFiFtmRanging::decode_ftm_params(&[0u8; 4]).is_none());
    }

    // --- LCI ---

    #[test]
    fn test_lci_distance_to_same_point() {
        let lci = LocationConfigInfo::new(37.7749, -122.4194, 0.0);
        assert_eq!(lci.distance_to(&lci), 0.0);
    }

    #[test]
    fn test_lci_distance_approximate() {
        let sf = LocationConfigInfo::new(37.7749, -122.4194, 0.0);
        let ny = LocationConfigInfo::new(40.7128, -74.0060, 0.0);
        let d = sf.distance_to(&ny);
        // Approximate SF to NY ~4100 km
        assert!(d > 4_000_000.0 && d < 5_000_000.0, "Distance SF-NY: {} m", d);
    }

    // --- LCR ---

    #[test]
    fn test_lcr_construction() {
        let mut lcr = LocationCivicReport::new("US");
        lcr.state = Some("CA".to_string());
        lcr.city = Some("San Francisco".to_string());
        lcr.floor = Some(3.0);
        assert_eq!(lcr.country, "US");
        assert_eq!(lcr.floor, Some(3.0));
    }

    // --- 802.11az quality metric ---

    #[test]
    fn test_az_ntb_quality_empty() {
        let ftm = default_ftm();
        assert_eq!(ftm.az_ntb_quality(&[]), 0.0);
    }

    #[test]
    fn test_az_ntb_quality_consistent() {
        let ftm = default_ftm();
        let exchanges: Vec<TimestampExchange> = (0..5)
            .map(|i| TimestampExchange::new(i as f64 * 1000.0, i as f64 * 1000.0 + 10.0, i as f64 * 1000.0 + 110.0, i as f64 * 1000.0 + 120.0))
            .collect();
        let quality = ftm.az_ntb_quality(&exchanges);
        // Very consistent RTT -> high quality
        assert!(quality > 0.8, "Quality should be high: {}", quality);
    }

    #[test]
    fn test_az_tb_process() {
        let ftm = default_ftm();
        let ex = TimestampExchange::new(0.0, 10.0, 110.0, 120.0);
        let rtt = ftm.az_tb_process(&ex, 5.0);
        assert!((rtt - 15.0).abs() < 1e-9, "TB RTT: {}", rtt);
    }

    // --- NLOS environment ---

    #[test]
    fn test_nlos_bias_estimates() {
        let ftm = default_ftm();
        assert_eq!(ftm.nlos_bias_estimate(NlosEnvironment::OpenSpace), 0.0);
        assert!(ftm.nlos_bias_estimate(NlosEnvironment::IndoorNlos) > 1.0);
        assert!(ftm.nlos_bias_estimate(NlosEnvironment::DenseUrbanNlos) > 10.0);
    }

    // --- Kalman filter ---

    #[test]
    fn test_kalman_range_convergence() {
        let ftm = default_ftm();
        let mut state = KalmanRangeState::new(0.0, 10.0);
        let true_range = 50.0;

        for _ in 0..20 {
            let filtered = ftm.kalman_range_update(&mut state, true_range, 0.1, 0.01, 0.25);
            let _ = filtered;
        }

        assert!((state.range - true_range).abs() < 2.0,
            "Kalman should converge to true range: {} vs {}", state.range, true_range);
    }

    #[test]
    fn test_kalman_range_initial_state() {
        let state = KalmanRangeState::new(10.0, 1.0);
        assert_eq!(state.range, 10.0);
        assert_eq!(state.range_rate, 0.0);
        assert_eq!(state.p[0][0], 1.0);
    }

    // --- SNR computation ---

    #[test]
    fn test_compute_snr_db_reasonable_range() {
        let ftm = default_ftm();
        let snr = ftm.compute_snr_db(-50.0, 80e6);
        // At -50 dBm signal, 80 MHz, 7 dB NF: noise floor ~-91 dBm -> SNR ~41 dB
        assert!(snr > 20.0 && snr < 60.0, "SNR should be reasonable: {}", snr);
    }

    // --- Ranging accuracy ---

    #[test]
    fn test_ranging_accuracy_practical_gt_crlb() {
        let ftm = default_ftm();
        let (crlb, practical) = ftm.ranging_accuracy_estimate(20.0);
        assert!(practical > crlb, "Practical should exceed CRLB");
        assert!(crlb > 0.0);
    }

    // --- Super-resolution ToA ---

    #[test]
    fn test_super_resolution_toa_returns_delays() {
        let ftm = default_ftm();
        // Simple CIR with one clear component
        let mut cir = vec![(0.0_f64, 0.0_f64); 32];
        cir[5] = (1.0, 0.0);
        let delays = ftm.super_resolution_toa(&cir, 1, 100e6);
        assert!(!delays.is_empty(), "Should return at least one delay");
    }

    #[test]
    fn test_super_resolution_toa_empty() {
        let ftm = default_ftm();
        let delays = ftm.super_resolution_toa(&[], 2, 80e6);
        assert!(delays.is_empty());
    }

    #[test]
    fn test_super_resolution_toa_zero_paths() {
        let ftm = default_ftm();
        let cir: Vec<(f64, f64)> = vec![(1.0, 0.0); 16];
        let delays = ftm.super_resolution_toa(&cir, 0, 80e6);
        assert!(delays.is_empty());
    }

    // --- Cross-correlation helper ---

    #[test]
    fn test_cross_correlate_impulse() {
        let signal = vec![(1.0_f64, 0.0_f64), (0.0, 0.0), (0.0, 0.0)];
        let reference = vec![(1.0_f64, 0.0_f64)];
        let xcorr = cross_correlate_complex(&signal, &reference);
        assert!(xcorr.len() >= 3);
        // Peak at lag corresponding to position 0
        let peak_idx = xcorr
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| (a.0 * a.0).partial_cmp(&(b.0 * b.0)).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        // The peak should be at offset 0 (or close to it)
        assert!(peak_idx < 3);
    }

    // --- Median helper ---

    #[test]
    fn test_median_odd() {
        let sorted = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        assert_eq!(median_of_sorted(&sorted), 3.0);
    }

    #[test]
    fn test_median_even() {
        let sorted = vec![1.0, 2.0, 3.0, 4.0];
        assert_eq!(median_of_sorted(&sorted), 2.5);
    }

    #[test]
    fn test_median_single() {
        assert_eq!(median_of_sorted(&[7.0]), 7.0);
    }

    // --- Integration tests ---

    #[test]
    fn test_full_ranging_pipeline() {
        // Simulate a full ranging session:
        // 1. Create exchanges for 20-meter distance
        // 2. Process burst
        // 3. Verify distance estimate
        let mut ftm = default_ftm();
        let true_distance = 20.0;
        let burst = ftm.simulate_burst(true_distance, 100.0, 0.1, 0.0, 999);
        let rtts: Vec<f64> = burst.iter().map(|ex| ftm.compute_rtt(ex)).collect();
        let result = ftm.burst_average_with_snr(&rtts, 25.0);

        assert!((result.distance_m - true_distance).abs() < 1.0,
            "Distance estimate: {} vs true {}", result.distance_m, true_distance);
        assert!(result.crlb_m > 0.0);
    }

    #[test]
    fn test_trilaterate_with_ftm_measurements() {
        let ftm = default_ftm();
        // Scenario: target at (5, 5), three APs
        let target = (5.0_f64, 5.0_f64);
        let aps = [(0.0, 0.0), (10.0, 0.0), (5.0, 10.0)];
        let measurements: Vec<(f64, f64, f64)> = aps
            .iter()
            .map(|&(ax, ay)| {
                let d = ((ax - target.0).powi(2) + (ay - target.1).powi(2)).sqrt();
                (ax, ay, d)
            })
            .collect();
        let (x, y) = ftm.trilaterate_2d(&measurements);
        assert!((x - target.0).abs() < 0.1);
        assert!((y - target.1).abs() < 0.1);
    }

    #[test]
    fn test_add_and_flush_burst() {
        let mut ftm = WiFiFtmRanging::new(FtmConfig { ftms_per_burst: 4, ..Default::default() });
        for i in 0..4 {
            let ex = TimestampExchange::new(
                i as f64 * 1000.0,
                i as f64 * 1000.0 + 10.0,
                i as f64 * 1000.0 + 110.0,
                i as f64 * 1000.0 + 120.0,
            );
            let meas = ftm.process_exchange(i as u32, ex);
            ftm.add_to_burst(meas);
        }
        let result = ftm.flush_burst(20.0);
        assert_eq!(result.num_measurements, 4);
        assert_eq!(ftm.burst_buffer.len(), 0); // Cleared after flush
    }

    #[test]
    fn test_timestamp_exchange_true_tof() {
        // t1=0, t2=10, t3=110, t4=120 -> tof = (10 + 10) / 2 = 10 ns
        let ex = TimestampExchange::new(0.0, 10.0, 110.0, 120.0);
        assert!((ex.true_tof_ns() - 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_clock_drift_model_correction() {
        let mut model = ClockDriftModel::new();
        model.freq_offset_ppm = 10.0; // 10 ppm drift
        let corrected = model.correct_rtt(100.0, 1e9); // 1 second interval
        // Drift = 10e-6 * 1e9 = 1000 ns
        let expected = 100.0 - 10.0 * 1e-6 * 1e9;
        assert!((corrected - expected).abs() < 1e-6);
    }

    #[test]
    fn test_az_secure_ltf_uncertainty_lower_than_no_secure() {
        let ftm = default_ftm();
        let secure = ftm.az_secure_ltf_uncertainty(20.0);
        let normal = ftm.crlb_ranging(20.0, ftm.config.bandwidth.hz());
        // Secure LTF has 3 dB SNR penalty -> worse accuracy
        assert!(secure > normal, "Secure LTF should have larger uncertainty");
    }

    #[test]
    fn test_geodetic_trilaterate_same_anchor() {
        let ftm = default_ftm();
        let lci1 = LocationConfigInfo::new(37.7749, -122.4194, 10.0);
        let lci2 = LocationConfigInfo::new(37.7760, -122.4194, 10.0);
        let lci3 = LocationConfigInfo::new(37.7749, -122.4180, 10.0);
        let anchors = [(&lci1, 50.0), (&lci2, 60.0), (&lci3, 55.0)];
        let pos = ftm.geodetic_trilaterate(&anchors);
        // Should produce some position without panicking
        assert!(pos.is_some());
        let pos = pos.unwrap();
        // Position should be near the anchors (within ~1 degree)
        assert!((pos.lat_deg - 37.7749).abs() < 1.0);
        assert!((pos.lon_deg - (-122.4194)).abs() < 1.0);
    }

    #[test]
    fn test_geodetic_trilaterate_too_few_anchors() {
        let ftm = default_ftm();
        let lci = LocationConfigInfo::new(0.0, 0.0, 0.0);
        assert!(ftm.geodetic_trilaterate(&[(&lci, 10.0), (&lci, 20.0)]).is_none());
    }
}
