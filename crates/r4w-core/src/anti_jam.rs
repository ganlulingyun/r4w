//! Jamming Detection and Anti-Jamming Mode Switching (MF-044)
//!
//! Provides real-time jamming detection and automatic switching to
//! anti-jamming (AJ) modes for resilient communications.
//!
//! # Detection Methods
//!
//! - **Power-based**: Detect abnormally high received power levels
//! - **SNR-based**: Detect degraded signal-to-noise ratio
//! - **BER-based**: Detect elevated bit error rates
//! - **Spectral**: Detect narrowband or swept jammers
//! - **Statistical**: Detect anomalous signal characteristics
//!
//! # Anti-Jamming Modes
//!
//! - **Frequency Hopping**: Switch to FHSS mode with crypto-secure hop pattern
//! - **Spread Spectrum**: Increase spreading factor for processing gain
//! - **Power Boost**: Increase transmit power to overcome jamming
//! - **Adaptive Nulling**: Steer antenna nulls toward jammer (requires array)
//! - **Channel Avoidance**: Blacklist jammed frequencies
//!
//! # Example
//!
//! ```rust
//! use r4w_core::anti_jam::{JammingDetector, AntiJamController, JamType, AjMode};
//!
//! let mut detector = JammingDetector::new();
//!
//! // Update with received signal characteristics
//! detector.update_power_dbm(-50.0);  // Received power
//! detector.update_noise_floor_dbm(-100.0);  // Noise floor
//! detector.update_ber(0.15);  // Bit error rate
//!
//! // Check for jamming
//! if let Some(jam_type) = detector.detect() {
//!     println!("Jamming detected: {:?}", jam_type);
//!
//!     // Switch to anti-jamming mode
//!     let mut controller = AntiJamController::new();
//!     let mode = controller.select_countermeasure(&jam_type);
//!     println!("Switching to: {:?}", mode);
//! }
//! ```

use std::collections::VecDeque;
use std::time::{Duration, Instant};

/// Maximum history size for detection
const HISTORY_SIZE: usize = 64;

/// Default jamming power threshold (dB above noise)
const DEFAULT_JAM_POWER_THRESHOLD_DB: f64 = 20.0;

/// Default SNR threshold for jamming detection
const DEFAULT_SNR_THRESHOLD_DB: f64 = 3.0;

/// Default BER threshold for jamming detection
const DEFAULT_BER_THRESHOLD: f64 = 0.10;

/// Types of jamming
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum JamType {
    /// No jamming detected
    None,
    /// Broadband noise jamming
    Broadband,
    /// Narrowband (spot) jamming
    Narrowband,
    /// Swept frequency jamming
    Swept,
    /// Pulse jamming
    Pulse,
    /// Follower/responsive jamming
    Follower,
    /// Smart/protocol-aware jamming
    Smart,
    /// Unknown jamming type
    Unknown,
}

/// Jamming severity level
#[derive(Debug, Clone, Copy, PartialEq, Ord, PartialOrd, Eq)]
pub enum JamSeverity {
    /// No jamming
    None = 0,
    /// Minor interference
    Low = 1,
    /// Moderate jamming, communication degraded
    Medium = 2,
    /// Severe jamming, communication difficult
    High = 3,
    /// Critical jamming, communication impossible
    Critical = 4,
}

/// Anti-jamming modes
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AjMode {
    /// Normal operation (no countermeasures)
    Normal,
    /// Frequency hopping spread spectrum
    FrequencyHopping {
        /// Hop rate (hops per second)
        hop_rate: u32,
        /// Number of channels
        num_channels: u16,
    },
    /// Direct sequence spread spectrum
    DirectSequence {
        /// Spreading factor (chips per bit)
        spreading_factor: u16,
    },
    /// Power boost mode
    PowerBoost {
        /// Additional power in dB
        boost_db: f64,
    },
    /// Channel avoidance
    ChannelAvoidance {
        /// Number of blacklisted channels
        blacklist_count: usize,
    },
    /// Combined FHSS + DS-SS
    Hybrid {
        /// Hop rate
        hop_rate: u32,
        /// Spreading factor
        spreading_factor: u16,
    },
    /// Emergency low-power burst mode
    EmergencyBurst {
        /// Burst duration in microseconds
        burst_us: u32,
    },
}

/// Jamming detector configuration
#[derive(Debug, Clone)]
pub struct DetectorConfig {
    /// Power threshold above noise floor (dB)
    pub power_threshold_db: f64,
    /// Minimum SNR threshold (dB)
    pub snr_threshold_db: f64,
    /// BER threshold for detection
    pub ber_threshold: f64,
    /// Detection window (samples)
    pub window_size: usize,
    /// Confirmation count (consecutive detections)
    pub confirm_count: usize,
    /// Spectral flatness threshold
    pub flatness_threshold: f64,
}

impl Default for DetectorConfig {
    fn default() -> Self {
        Self {
            power_threshold_db: DEFAULT_JAM_POWER_THRESHOLD_DB,
            snr_threshold_db: DEFAULT_SNR_THRESHOLD_DB,
            ber_threshold: DEFAULT_BER_THRESHOLD,
            window_size: 16,
            confirm_count: 3,
            flatness_threshold: 0.8,
        }
    }
}

/// Jamming detection state
#[derive(Debug)]
pub struct JammingDetector {
    /// Configuration
    config: DetectorConfig,
    /// Power history (dBm)
    power_history: VecDeque<f64>,
    /// Noise floor history (dBm)
    noise_history: VecDeque<f64>,
    /// BER history
    ber_history: VecDeque<f64>,
    /// Spectral samples (for narrowband detection)
    spectral_history: VecDeque<Vec<f64>>,
    /// Current estimated noise floor (dBm)
    noise_floor_dbm: f64,
    /// Detection counter
    detection_count: usize,
    /// Last detected jam type
    last_jam_type: JamType,
    /// Last detection time
    last_detection: Option<Instant>,
    /// Current severity
    severity: JamSeverity,
    /// Baseline power (learned during normal operation)
    baseline_power_dbm: f64,
    /// Baseline BER
    baseline_ber: f64,
}

impl Default for JammingDetector {
    fn default() -> Self {
        Self::new()
    }
}

impl JammingDetector {
    /// Create a new jamming detector with default configuration
    pub fn new() -> Self {
        Self::with_config(DetectorConfig::default())
    }

    /// Create a new jamming detector with custom configuration
    pub fn with_config(config: DetectorConfig) -> Self {
        Self {
            config,
            power_history: VecDeque::with_capacity(HISTORY_SIZE),
            noise_history: VecDeque::with_capacity(HISTORY_SIZE),
            ber_history: VecDeque::with_capacity(HISTORY_SIZE),
            spectral_history: VecDeque::with_capacity(16),
            noise_floor_dbm: -100.0,
            detection_count: 0,
            last_jam_type: JamType::None,
            last_detection: None,
            severity: JamSeverity::None,
            baseline_power_dbm: -90.0,
            baseline_ber: 0.001,
        }
    }

    /// Update with received power measurement
    pub fn update_power_dbm(&mut self, power_dbm: f64) {
        if self.power_history.len() >= HISTORY_SIZE {
            self.power_history.pop_front();
        }
        self.power_history.push_back(power_dbm);
    }

    /// Update noise floor estimate
    pub fn update_noise_floor_dbm(&mut self, noise_dbm: f64) {
        if self.noise_history.len() >= HISTORY_SIZE {
            self.noise_history.pop_front();
        }
        self.noise_history.push_back(noise_dbm);

        // Exponential moving average for noise floor
        let alpha = 0.1;
        self.noise_floor_dbm = alpha * noise_dbm + (1.0 - alpha) * self.noise_floor_dbm;
    }

    /// Update with bit error rate measurement
    pub fn update_ber(&mut self, ber: f64) {
        if self.ber_history.len() >= HISTORY_SIZE {
            self.ber_history.pop_front();
        }
        self.ber_history.push_back(ber);
    }

    /// Update with spectral snapshot (power per frequency bin)
    pub fn update_spectral(&mut self, spectrum: Vec<f64>) {
        if self.spectral_history.len() >= 16 {
            self.spectral_history.pop_front();
        }
        self.spectral_history.push_back(spectrum);
    }

    /// Learn baseline during known-good conditions
    pub fn learn_baseline(&mut self) {
        if !self.power_history.is_empty() {
            self.baseline_power_dbm =
                self.power_history.iter().sum::<f64>() / self.power_history.len() as f64;
        }
        if !self.ber_history.is_empty() {
            self.baseline_ber =
                self.ber_history.iter().sum::<f64>() / self.ber_history.len() as f64;
        }
    }

    /// Perform jamming detection
    pub fn detect(&mut self) -> Option<JamType> {
        let power_jam = self.check_power_jamming();
        let snr_jam = self.check_snr_degradation();
        let ber_jam = self.check_ber_elevation();
        let spectral_jam = self.check_spectral_anomaly();

        // Combine detection results
        let jam_type = self.classify_jamming(power_jam, snr_jam, ber_jam, spectral_jam);

        if jam_type != JamType::None {
            self.detection_count += 1;
            if self.detection_count >= self.config.confirm_count {
                self.last_jam_type = jam_type;
                self.last_detection = Some(Instant::now());
                self.update_severity();
                return Some(jam_type);
            }
        } else {
            // Decay detection counter
            self.detection_count = self.detection_count.saturating_sub(1);
            if self.detection_count == 0 {
                self.severity = JamSeverity::None;
            }
        }

        None
    }

    /// Check for power-based jamming
    fn check_power_jamming(&self) -> bool {
        if self.power_history.len() < 3 {
            return false;
        }

        let avg_power: f64 =
            self.power_history.iter().sum::<f64>() / self.power_history.len() as f64;

        // Check if power is significantly above baseline or noise floor
        let above_noise = avg_power - self.noise_floor_dbm;
        let above_baseline = avg_power - self.baseline_power_dbm;

        above_noise > self.config.power_threshold_db
            || above_baseline > self.config.power_threshold_db / 2.0
    }

    /// Check for SNR degradation
    fn check_snr_degradation(&self) -> bool {
        if self.power_history.is_empty() {
            return false;
        }

        let avg_power: f64 =
            self.power_history.iter().sum::<f64>() / self.power_history.len() as f64;
        let snr = avg_power - self.noise_floor_dbm;

        snr < self.config.snr_threshold_db
    }

    /// Check for BER elevation
    fn check_ber_elevation(&self) -> bool {
        if self.ber_history.len() < 3 {
            return false;
        }

        let avg_ber: f64 = self.ber_history.iter().sum::<f64>() / self.ber_history.len() as f64;
        let ber_increase = avg_ber / self.baseline_ber.max(0.0001);

        avg_ber > self.config.ber_threshold || ber_increase > 10.0
    }

    /// Check for spectral anomalies (narrowband or swept jamming)
    fn check_spectral_anomaly(&self) -> Option<JamType> {
        if self.spectral_history.len() < 2 {
            return None;
        }

        let latest = self.spectral_history.back()?;
        if latest.is_empty() {
            return None;
        }

        // Calculate spectral flatness
        let mean = latest.iter().sum::<f64>() / latest.len() as f64;
        let geometric_mean = latest.iter().map(|&x| x.max(1e-10).ln()).sum::<f64>()
            / latest.len() as f64;
        let geometric_mean = geometric_mean.exp();

        let flatness = geometric_mean / mean.max(1e-10);

        // Low flatness indicates narrowband jamming
        if flatness < self.config.flatness_threshold {
            // Check if peak is moving (swept jammer)
            if self.spectral_history.len() >= 3 {
                let prev = &self.spectral_history[self.spectral_history.len() - 2];
                let prev_prev = &self.spectral_history[self.spectral_history.len() - 3];

                let curr_peak = argmax(latest);
                let prev_peak = argmax(prev);
                let prev_prev_peak = argmax(prev_prev);

                // Check for consistent movement
                if curr_peak != prev_peak && prev_peak != prev_prev_peak {
                    return Some(JamType::Swept);
                }
            }
            return Some(JamType::Narrowband);
        }

        None
    }

    /// Classify jamming type from multiple indicators
    fn classify_jamming(
        &self,
        power_jam: bool,
        snr_jam: bool,
        ber_jam: bool,
        spectral_jam: Option<JamType>,
    ) -> JamType {
        // Priority: spectral detection is most specific
        if let Some(spectral) = spectral_jam {
            return spectral;
        }

        // Power + SNR + BER = likely broadband
        if power_jam && snr_jam && ber_jam {
            return JamType::Broadband;
        }

        // High power but good SNR = narrowband (not covering our signal)
        if power_jam && !snr_jam && !ber_jam {
            return JamType::Narrowband;
        }

        // SNR and BER degraded = some form of interference
        if snr_jam || ber_jam {
            return JamType::Unknown;
        }

        JamType::None
    }

    /// Update severity based on current conditions
    fn update_severity(&mut self) {
        if self.ber_history.is_empty() {
            return;
        }

        let avg_ber: f64 = self.ber_history.iter().sum::<f64>() / self.ber_history.len() as f64;

        self.severity = if avg_ber > 0.5 {
            JamSeverity::Critical
        } else if avg_ber > 0.25 {
            JamSeverity::High
        } else if avg_ber > 0.10 {
            JamSeverity::Medium
        } else if avg_ber > self.config.ber_threshold {
            JamSeverity::Low
        } else {
            JamSeverity::None
        };
    }

    /// Get current jamming severity
    pub fn severity(&self) -> JamSeverity {
        self.severity
    }

    /// Get last detected jam type
    pub fn last_jam_type(&self) -> JamType {
        self.last_jam_type
    }

    /// Get time since last detection
    pub fn time_since_detection(&self) -> Option<Duration> {
        self.last_detection.map(|t| t.elapsed())
    }

    /// Reset detector state
    pub fn reset(&mut self) {
        self.power_history.clear();
        self.noise_history.clear();
        self.ber_history.clear();
        self.spectral_history.clear();
        self.detection_count = 0;
        self.last_jam_type = JamType::None;
        self.severity = JamSeverity::None;
    }
}

/// Anti-jamming controller
#[derive(Debug)]
pub struct AntiJamController {
    /// Current AJ mode
    current_mode: AjMode,
    /// Previous mode (for fallback)
    previous_mode: AjMode,
    /// Channel blacklist
    blacklist: Vec<u16>,
    /// Maximum blacklist size
    max_blacklist: usize,
    /// Mode switch count
    switch_count: u32,
    /// Last mode switch time
    last_switch: Option<Instant>,
    /// Minimum time between switches
    min_switch_interval: Duration,
    /// Current hop rate (if in FHSS mode)
    hop_rate: u32,
    /// Available channels
    num_channels: u16,
}

impl Default for AntiJamController {
    fn default() -> Self {
        Self::new()
    }
}

impl AntiJamController {
    /// Create a new anti-jamming controller
    pub fn new() -> Self {
        Self {
            current_mode: AjMode::Normal,
            previous_mode: AjMode::Normal,
            blacklist: Vec::new(),
            max_blacklist: 64,
            switch_count: 0,
            last_switch: None,
            min_switch_interval: Duration::from_millis(100),
            hop_rate: 100,
            num_channels: 50,
        }
    }

    /// Select countermeasure based on jam type
    pub fn select_countermeasure(&mut self, jam_type: &JamType) -> AjMode {
        let new_mode = match jam_type {
            JamType::None => AjMode::Normal,

            JamType::Narrowband => {
                // Avoid the jammed frequency
                AjMode::ChannelAvoidance {
                    blacklist_count: self.blacklist.len(),
                }
            }

            JamType::Broadband => {
                // Use spread spectrum for processing gain
                AjMode::DirectSequence { spreading_factor: 64 }
            }

            JamType::Swept => {
                // Fast frequency hopping to outrun the sweep
                AjMode::FrequencyHopping {
                    hop_rate: 5000, // Fast hopping
                    num_channels: self.num_channels,
                }
            }

            JamType::Follower => {
                // Hybrid approach: fast hopping + spreading
                AjMode::Hybrid {
                    hop_rate: 10000,
                    spreading_factor: 16,
                }
            }

            JamType::Pulse => {
                // Time the transmissions between pulses
                AjMode::EmergencyBurst { burst_us: 100 }
            }

            JamType::Smart | JamType::Unknown => {
                // Maximum protection: hybrid mode
                AjMode::Hybrid {
                    hop_rate: self.hop_rate * 2,
                    spreading_factor: 32,
                }
            }
        };

        self.switch_mode(new_mode)
    }

    /// Select countermeasure based on severity
    pub fn select_by_severity(&mut self, severity: JamSeverity) -> AjMode {
        let new_mode = match severity {
            JamSeverity::None => AjMode::Normal,

            JamSeverity::Low => AjMode::FrequencyHopping {
                hop_rate: self.hop_rate,
                num_channels: self.num_channels,
            },

            JamSeverity::Medium => AjMode::FrequencyHopping {
                hop_rate: self.hop_rate * 2,
                num_channels: self.num_channels,
            },

            JamSeverity::High => AjMode::Hybrid {
                hop_rate: self.hop_rate * 4,
                spreading_factor: 16,
            },

            JamSeverity::Critical => AjMode::Hybrid {
                hop_rate: self.hop_rate * 10,
                spreading_factor: 64,
            },
        };

        self.switch_mode(new_mode)
    }

    /// Switch to a new mode
    fn switch_mode(&mut self, new_mode: AjMode) -> AjMode {
        // Check minimum switch interval
        if let Some(last) = self.last_switch {
            if last.elapsed() < self.min_switch_interval && new_mode != AjMode::Normal {
                return self.current_mode;
            }
        }

        if new_mode != self.current_mode {
            self.previous_mode = self.current_mode;
            self.current_mode = new_mode;
            self.switch_count += 1;
            self.last_switch = Some(Instant::now());
        }

        self.current_mode
    }

    /// Blacklist a channel
    pub fn blacklist_channel(&mut self, channel: u16) {
        if !self.blacklist.contains(&channel) {
            if self.blacklist.len() >= self.max_blacklist {
                self.blacklist.remove(0); // Remove oldest
            }
            self.blacklist.push(channel);
        }
    }

    /// Clear a channel from blacklist
    pub fn whitelist_channel(&mut self, channel: u16) {
        self.blacklist.retain(|&c| c != channel);
    }

    /// Check if a channel is blacklisted
    pub fn is_blacklisted(&self, channel: u16) -> bool {
        self.blacklist.contains(&channel)
    }

    /// Get current mode
    pub fn current_mode(&self) -> AjMode {
        self.current_mode
    }

    /// Get previous mode
    pub fn previous_mode(&self) -> AjMode {
        self.previous_mode
    }

    /// Get mode switch count
    pub fn switch_count(&self) -> u32 {
        self.switch_count
    }

    /// Get blacklisted channels
    pub fn blacklist(&self) -> &[u16] {
        &self.blacklist
    }

    /// Return to normal mode
    pub fn return_to_normal(&mut self) {
        self.switch_mode(AjMode::Normal);
    }

    /// Return to previous mode
    pub fn fallback(&mut self) {
        let prev = self.previous_mode;
        self.switch_mode(prev);
    }

    /// Configure base hop rate
    pub fn set_base_hop_rate(&mut self, rate: u32) {
        self.hop_rate = rate;
    }

    /// Configure number of channels
    pub fn set_num_channels(&mut self, channels: u16) {
        self.num_channels = channels;
    }
}

/// Find index of maximum value
fn argmax(slice: &[f64]) -> usize {
    slice
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(i, _)| i)
        .unwrap_or(0)
}

/// Jamming statistics for reporting
#[derive(Debug, Default, Clone)]
pub struct JammingStats {
    /// Total jamming events detected
    pub events_detected: u64,
    /// Time spent in jammed state (seconds)
    pub jammed_time_s: f64,
    /// Mode switches performed
    pub mode_switches: u32,
    /// Current severity
    pub current_severity: u8,
    /// Channels blacklisted
    pub channels_blacklisted: usize,
    /// Most common jam type
    pub primary_jam_type: u8,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_power_jamming_detection() {
        let mut detector = JammingDetector::new();

        // Normal conditions
        for _ in 0..10 {
            detector.update_power_dbm(-80.0);
            detector.update_noise_floor_dbm(-100.0);
            detector.update_ber(0.001);
        }
        detector.learn_baseline();

        // No jamming yet
        assert_eq!(detector.detect(), None);

        // Introduce jamming (high power)
        for _ in 0..10 {
            detector.update_power_dbm(-50.0); // 30 dB above normal
            detector.update_noise_floor_dbm(-60.0);
            detector.update_ber(0.15);
        }

        // Need multiple detect() calls to confirm (confirm_count = 3 by default)
        let mut detected = false;
        for _ in 0..5 {
            if detector.detect().is_some() {
                detected = true;
                break;
            }
        }
        assert!(detected, "Should detect jamming after confirmation");
    }

    #[test]
    fn test_severity_levels() {
        let mut detector = JammingDetector::new();

        // Set up initial conditions
        detector.update_noise_floor_dbm(-100.0);

        // Simulate jamming with moderate BER
        for _ in 0..10 {
            detector.update_power_dbm(-50.0);
            detector.update_noise_floor_dbm(-60.0);
            detector.update_ber(0.12);  // Above threshold
        }

        // Trigger detection (multiple times to confirm)
        for _ in 0..5 {
            detector.detect();
        }

        // Should have some severity after detection
        assert!(detector.severity() >= JamSeverity::Low,
                "Should detect at least low severity, got {:?}", detector.severity());

        // Fill history with critical BER (need more than HISTORY_SIZE to flush old values)
        for _ in 0..70 {  // HISTORY_SIZE is 64
            detector.update_ber(0.55);
        }
        for _ in 0..5 {
            detector.detect();
        }

        assert_eq!(detector.severity(), JamSeverity::Critical);
    }

    #[test]
    fn test_controller_mode_selection() {
        let mut controller = AntiJamController::new();
        // Set min_switch_interval to 0 for testing
        controller.min_switch_interval = Duration::from_millis(0);

        // Normal operation
        let mode = controller.select_countermeasure(&JamType::None);
        assert_eq!(mode, AjMode::Normal);

        // Narrowband jamming - should use channel avoidance
        let mode = controller.select_countermeasure(&JamType::Narrowband);
        assert!(matches!(mode, AjMode::ChannelAvoidance { .. }),
                "Expected ChannelAvoidance, got {:?}", mode);

        // Swept jamming - should use fast hopping
        let mode = controller.select_countermeasure(&JamType::Swept);
        assert!(matches!(mode, AjMode::FrequencyHopping { hop_rate, .. } if hop_rate >= 5000),
                "Expected FrequencyHopping with hop_rate >= 5000, got {:?}", mode);
    }

    #[test]
    fn test_severity_based_selection() {
        let mut controller = AntiJamController::new();

        let mode = controller.select_by_severity(JamSeverity::None);
        assert_eq!(mode, AjMode::Normal);

        let mode = controller.select_by_severity(JamSeverity::Critical);
        matches!(mode, AjMode::Hybrid { .. });
    }

    #[test]
    fn test_channel_blacklist() {
        let mut controller = AntiJamController::new();

        assert!(!controller.is_blacklisted(5));

        controller.blacklist_channel(5);
        controller.blacklist_channel(10);
        controller.blacklist_channel(15);

        assert!(controller.is_blacklisted(5));
        assert!(controller.is_blacklisted(10));
        assert!(!controller.is_blacklisted(20));

        controller.whitelist_channel(5);
        assert!(!controller.is_blacklisted(5));
    }

    #[test]
    fn test_mode_switch_count() {
        let mut controller = AntiJamController::new();

        assert_eq!(controller.switch_count(), 0);

        controller.select_countermeasure(&JamType::Broadband);
        std::thread::sleep(Duration::from_millis(150)); // Wait for min interval

        controller.select_countermeasure(&JamType::Narrowband);

        assert!(controller.switch_count() >= 1);
    }

    #[test]
    fn test_detector_reset() {
        let mut detector = JammingDetector::new();

        // Setup jamming conditions
        for _ in 0..10 {
            detector.update_power_dbm(-50.0);
            detector.update_noise_floor_dbm(-100.0);
            detector.update_ber(0.25);  // High BER
        }

        // Trigger detection multiple times
        for _ in 0..5 {
            detector.detect();
        }

        // After reset, should be clean
        detector.reset();
        assert_eq!(detector.severity(), JamSeverity::None);
        assert_eq!(detector.last_jam_type(), JamType::None);
    }
}
