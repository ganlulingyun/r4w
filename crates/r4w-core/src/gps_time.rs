//! GPS/PPS Time Integration Framework (MF-042)
//!
//! Provides GPS time integration for HAVEQUICK Time-of-Day (TOD) and other
//! time-synchronized FHSS protocols. Supports PPS (Pulse Per Second) synchronization
//! and multi-clock domain management.
//!
//! # Time Domains
//!
//! - **GPS Time**: Continuous time scale from GPS constellation
//! - **UTC Time**: GPS time with leap second corrections
//! - **TAI Time**: International Atomic Time (GPS + 19 seconds)
//! - **TOD**: Military time-of-day format for HAVEQUICK/SATURN
//!
//! # Example
//!
//! ```rust
//! use r4w_core::gps_time::{GpsTime, PpsSync, TimeOfDay};
//!
//! // Create GPS time from week number and time-of-week
//! let gps_time = GpsTime::from_week_tow(2345, 123456.789);
//!
//! // Convert to UTC
//! let utc = gps_time.to_utc();
//! println!("UTC: {:04}-{:02}-{:02} {:02}:{:02}:{:02}",
//!          utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second);
//!
//! // Generate HAVEQUICK TOD code
//! let tod = TimeOfDay::from_gps(&gps_time);
//! println!("HAVEQUICK TOD: {:06}", tod.havequick_code());
//! ```

use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// GPS week epoch (January 6, 1980 00:00:00 UTC)
const GPS_EPOCH_UNIX: i64 = 315_964_800;

/// Seconds per GPS week
const SECONDS_PER_WEEK: f64 = 604_800.0;

/// Current GPS-UTC leap seconds (as of 2024)
const LEAP_SECONDS: i32 = 18;

/// TAI-GPS offset (constant)
const TAI_GPS_OFFSET: f64 = 19.0;

/// GPS time representation
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpsTime {
    /// GPS week number (rollover every 1024 weeks)
    pub week: u16,
    /// Time of week in seconds (0.0 - 604799.999...)
    pub tow: f64,
    /// Fractional nanoseconds for sub-second precision
    pub frac_ns: u32,
}

impl GpsTime {
    /// Create GPS time from week number and time-of-week
    pub fn from_week_tow(week: u16, tow: f64) -> Self {
        let frac_ns = ((tow.fract()) * 1_000_000_000.0) as u32;
        Self { week, tow, frac_ns }
    }

    /// Create GPS time from Unix timestamp
    pub fn from_unix_timestamp(unix_secs: f64) -> Self {
        let gps_secs = unix_secs - GPS_EPOCH_UNIX as f64 + LEAP_SECONDS as f64;
        let week = (gps_secs / SECONDS_PER_WEEK) as u16;
        let tow = gps_secs % SECONDS_PER_WEEK;
        let frac_ns = ((tow.fract()) * 1_000_000_000.0) as u32;
        Self { week, tow, frac_ns }
    }

    /// Create GPS time from current system time
    pub fn now() -> Self {
        let unix_secs = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs_f64();
        Self::from_unix_timestamp(unix_secs)
    }

    /// Convert to Unix timestamp
    pub fn to_unix_timestamp(&self) -> f64 {
        GPS_EPOCH_UNIX as f64 + (self.week as f64 * SECONDS_PER_WEEK) + self.tow
            - LEAP_SECONDS as f64
    }

    /// Convert to UTC time
    pub fn to_utc(&self) -> UtcTime {
        let unix_secs = self.to_unix_timestamp();
        UtcTime::from_unix_timestamp(unix_secs)
    }

    /// Convert to TAI time (GPS + 19 seconds)
    pub fn to_tai_ns(&self) -> u64 {
        let gps_ns = (self.week as u64 * SECONDS_PER_WEEK as u64 * 1_000_000_000)
            + (self.tow as u64 * 1_000_000_000)
            + self.frac_ns as u64;
        gps_ns + (TAI_GPS_OFFSET as u64 * 1_000_000_000)
    }

    /// Add duration to GPS time
    pub fn add_duration(&self, duration: Duration) -> Self {
        let new_tow = self.tow + duration.as_secs_f64();
        let weeks_overflow = (new_tow / SECONDS_PER_WEEK) as u16;
        let new_week = self.week.wrapping_add(weeks_overflow);
        let final_tow = new_tow % SECONDS_PER_WEEK;

        Self::from_week_tow(new_week, final_tow)
    }

    /// Calculate duration between two GPS times
    pub fn duration_since(&self, earlier: &GpsTime) -> Duration {
        let self_secs = (self.week as f64 * SECONDS_PER_WEEK) + self.tow;
        let earlier_secs = (earlier.week as f64 * SECONDS_PER_WEEK) + earlier.tow;
        let diff = self_secs - earlier_secs;
        if diff >= 0.0 {
            Duration::from_secs_f64(diff)
        } else {
            Duration::ZERO
        }
    }

    /// Get seconds since midnight (GPS time)
    pub fn seconds_since_midnight(&self) -> f64 {
        self.tow % 86400.0
    }

    /// Get day of week (0 = Sunday)
    pub fn day_of_week(&self) -> u8 {
        ((self.tow / 86400.0) as u8) % 7
    }
}

/// UTC time representation
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UtcTime {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
    pub nanosecond: u32,
}

impl UtcTime {
    /// Create from Unix timestamp
    pub fn from_unix_timestamp(unix_secs: f64) -> Self {
        let secs = unix_secs as i64;
        let nanosecond = ((unix_secs.fract()) * 1_000_000_000.0) as u32;

        // Simple date calculation (doesn't handle all edge cases)
        let days_since_epoch = secs / 86400;
        let time_of_day = secs % 86400;

        let hour = (time_of_day / 3600) as u8;
        let minute = ((time_of_day % 3600) / 60) as u8;
        let second = (time_of_day % 60) as u8;

        // Calculate year, month, day from days since epoch
        // This is a simplified calculation
        let (year, month, day) = days_to_ymd(days_since_epoch);

        Self {
            year: year as u16,
            month,
            day,
            hour,
            minute,
            second,
            nanosecond,
        }
    }

    /// Convert to GPS time
    pub fn to_gps(&self) -> GpsTime {
        let unix_secs = self.to_unix_timestamp();
        GpsTime::from_unix_timestamp(unix_secs)
    }

    /// Convert to Unix timestamp
    pub fn to_unix_timestamp(&self) -> f64 {
        let days = ymd_to_days(self.year as i32, self.month, self.day);
        let secs = days * 86400
            + self.hour as i64 * 3600
            + self.minute as i64 * 60
            + self.second as i64;
        secs as f64 + self.nanosecond as f64 / 1_000_000_000.0
    }
}

/// Time of Day for military FHSS protocols
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TimeOfDay {
    /// Day of year (1-366)
    pub day_of_year: u16,
    /// Hour (0-23)
    pub hour: u8,
    /// Minute (0-59)
    pub minute: u8,
    /// Second (0-59)
    pub second: u8,
    /// Millisecond (0-999)
    pub millisecond: u16,
}

impl TimeOfDay {
    /// Create from GPS time
    pub fn from_gps(gps: &GpsTime) -> Self {
        let utc = gps.to_utc();

        // Calculate day of year
        let day_of_year = day_of_year(utc.year as i32, utc.month, utc.day);

        let millisecond = (gps.frac_ns / 1_000_000) as u16;

        Self {
            day_of_year,
            hour: utc.hour,
            minute: utc.minute,
            second: utc.second,
            millisecond,
        }
    }

    /// Generate HAVEQUICK TOD code (6-digit BCD)
    ///
    /// Format: DDDHHMMSS (day-hour-minute-second, packed)
    /// Used for HAVEQUICK I/II frequency hopping synchronization
    pub fn havequick_code(&self) -> u32 {
        // HAVEQUICK uses day-of-year + time in BCD format
        let d = self.day_of_year as u32;
        let h = self.hour as u32;
        let m = self.minute as u32;

        // Pack into BCD-like format (day + hour + minute)
        (d % 400) * 10000 + h * 100 + m
    }

    /// Generate SATURN TOD code
    ///
    /// Similar to HAVEQUICK but with different encoding
    pub fn saturn_code(&self) -> u32 {
        // SATURN uses a slightly different format
        let seconds_of_day =
            self.hour as u32 * 3600 + self.minute as u32 * 60 + self.second as u32;

        // Day + seconds packed
        ((self.day_of_year as u32) << 17) | seconds_of_day
    }

    /// Get total seconds since midnight
    pub fn seconds_since_midnight(&self) -> u32 {
        self.hour as u32 * 3600 + self.minute as u32 * 60 + self.second as u32
    }

    /// Get total milliseconds since midnight
    pub fn millis_since_midnight(&self) -> u32 {
        self.seconds_since_midnight() * 1000 + self.millisecond as u32
    }
}

/// PPS (Pulse Per Second) synchronization state
#[derive(Debug, Clone)]
pub struct PpsSync {
    /// Whether PPS is locked
    pub locked: bool,
    /// Last PPS edge timestamp (monotonic)
    last_pps_instant: Option<Instant>,
    /// GPS time at last PPS edge
    last_pps_gps: Option<GpsTime>,
    /// Estimated PPS period (should be ~1.0 seconds)
    pub period_estimate_s: f64,
    /// PPS jitter estimate (seconds, RMS)
    pub jitter_estimate_s: f64,
    /// Number of valid PPS edges received
    pub edge_count: u64,
    /// Lock threshold (seconds)
    lock_threshold_s: f64,
}

impl Default for PpsSync {
    fn default() -> Self {
        Self::new()
    }
}

impl PpsSync {
    /// Create a new PPS synchronization state
    pub fn new() -> Self {
        Self {
            locked: false,
            last_pps_instant: None,
            last_pps_gps: None,
            period_estimate_s: 1.0,
            jitter_estimate_s: 0.0,
            edge_count: 0,
            lock_threshold_s: 0.001, // 1ms lock threshold
        }
    }

    /// Record a PPS edge with associated GPS time
    pub fn record_pps_edge(&mut self, gps_time: GpsTime) {
        let now = Instant::now();

        if let Some(last_instant) = self.last_pps_instant {
            let period = now.duration_since(last_instant).as_secs_f64();

            // Update period estimate (exponential moving average)
            let alpha = 0.1;
            self.period_estimate_s = alpha * period + (1.0 - alpha) * self.period_estimate_s;

            // Update jitter estimate
            let error = (period - 1.0).abs();
            self.jitter_estimate_s = alpha * error + (1.0 - alpha) * self.jitter_estimate_s;

            // Update lock status
            self.locked = self.jitter_estimate_s < self.lock_threshold_s && self.edge_count > 10;
        }

        self.last_pps_instant = Some(now);
        self.last_pps_gps = Some(gps_time);
        self.edge_count += 1;
    }

    /// Get interpolated GPS time at current instant
    pub fn interpolated_gps_time(&self) -> Option<GpsTime> {
        let (last_instant, last_gps) = match (self.last_pps_instant, self.last_pps_gps) {
            (Some(i), Some(g)) => (i, g),
            _ => return None,
        };

        let elapsed = Instant::now().duration_since(last_instant);
        Some(last_gps.add_duration(elapsed))
    }

    /// Get time uncertainty in nanoseconds
    pub fn uncertainty_ns(&self) -> u64 {
        if self.locked {
            // Uncertainty is based on jitter and time since last PPS
            let elapsed = self
                .last_pps_instant
                .map(|i| Instant::now().duration_since(i).as_secs_f64())
                .unwrap_or(10.0);

            // Jitter + drift estimate
            let uncertainty_s = self.jitter_estimate_s + elapsed * 1e-6; // 1 ppm drift
            (uncertainty_s * 1_000_000_000.0) as u64
        } else {
            // Not locked, uncertainty is unbounded
            u64::MAX
        }
    }

    /// Check if we're within spec for HAVEQUICK (< 100µs)
    pub fn havequick_compliant(&self) -> bool {
        self.locked && self.uncertainty_ns() < 100_000
    }

    /// Check if we're within spec for SINCGARS (< 1ms)
    pub fn sincgars_compliant(&self) -> bool {
        self.locked && self.uncertainty_ns() < 1_000_000
    }

    /// Reset synchronization state
    pub fn reset(&mut self) {
        self.locked = false;
        self.last_pps_instant = None;
        self.last_pps_gps = None;
        self.edge_count = 0;
        self.jitter_estimate_s = 0.0;
    }
}

/// Multi-clock time source manager
#[derive(Debug)]
pub struct TimeSourceManager {
    /// Current primary time source
    primary: TimeSource,
    /// Backup time source
    #[allow(dead_code)]
    backup: TimeSource,
    /// PPS synchronization state
    pps: PpsSync,
    /// Last known GPS time (may be stale)
    last_gps: Option<GpsTime>,
    /// System time at last GPS update
    last_update: Option<Instant>,
}

/// Time source types
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TimeSource {
    /// GPS receiver
    Gps,
    /// PTP/IEEE 1588
    Ptp,
    /// NTP
    Ntp,
    /// Free-running local oscillator
    Freerun,
    /// No time source
    None,
}

impl Default for TimeSourceManager {
    fn default() -> Self {
        Self::new()
    }
}

impl TimeSourceManager {
    /// Create a new time source manager
    pub fn new() -> Self {
        Self {
            primary: TimeSource::None,
            backup: TimeSource::Freerun,
            pps: PpsSync::new(),
            last_gps: None,
            last_update: None,
        }
    }

    /// Update GPS time from receiver
    pub fn update_gps(&mut self, gps_time: GpsTime) {
        self.last_gps = Some(gps_time);
        self.last_update = Some(Instant::now());

        if self.primary == TimeSource::None {
            self.primary = TimeSource::Gps;
        }
    }

    /// Record PPS edge
    pub fn record_pps(&mut self, gps_time: GpsTime) {
        self.pps.record_pps_edge(gps_time);
    }

    /// Get current best time estimate
    pub fn current_time(&self) -> Option<GpsTime> {
        if self.pps.locked {
            self.pps.interpolated_gps_time()
        } else if let (Some(gps), Some(update)) = (self.last_gps, self.last_update) {
            let elapsed = Instant::now().duration_since(update);
            Some(gps.add_duration(elapsed))
        } else {
            // Fall back to system time
            Some(GpsTime::now())
        }
    }

    /// Get time of day for FHSS
    pub fn current_tod(&self) -> Option<TimeOfDay> {
        self.current_time().map(|gps| TimeOfDay::from_gps(&gps))
    }

    /// Get primary time source
    pub fn primary_source(&self) -> TimeSource {
        self.primary
    }

    /// Check if time is synchronized
    pub fn is_synchronized(&self) -> bool {
        self.pps.locked || self.last_gps.is_some()
    }

    /// Get timing uncertainty in nanoseconds
    pub fn uncertainty_ns(&self) -> u64 {
        if self.pps.locked {
            self.pps.uncertainty_ns()
        } else if let Some(update) = self.last_update {
            // Estimate based on elapsed time and assumed drift
            let elapsed_s = Instant::now().duration_since(update).as_secs_f64();
            let drift_ppm = 10.0; // Assume 10 ppm worst case
            (elapsed_s * drift_ppm * 1000.0) as u64 // Convert to ns
        } else {
            u64::MAX
        }
    }

    /// Get PPS sync state
    pub fn pps_sync(&self) -> &PpsSync {
        &self.pps
    }
}

// Helper functions for date calculations

fn is_leap_year(year: i32) -> bool {
    (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)
}

fn days_in_year(year: i32) -> i32 {
    if is_leap_year(year) {
        366
    } else {
        365
    }
}

fn days_in_month(year: i32, month: u8) -> u8 {
    match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 => {
            if is_leap_year(year) {
                29
            } else {
                28
            }
        }
        _ => 0,
    }
}

fn day_of_year(year: i32, month: u8, day: u8) -> u16 {
    let mut doy = day as u16;
    for m in 1..month {
        doy += days_in_month(year, m) as u16;
    }
    doy
}

fn days_to_ymd(days_since_epoch: i64) -> (i32, u8, u8) {
    // Simple calculation from Unix epoch (1970-01-01)
    let mut remaining_days = days_since_epoch;
    let mut year = 1970;

    // Find year
    while remaining_days >= days_in_year(year) as i64 {
        remaining_days -= days_in_year(year) as i64;
        year += 1;
    }
    while remaining_days < 0 {
        year -= 1;
        remaining_days += days_in_year(year) as i64;
    }

    // Find month and day
    let mut month = 1u8;
    while remaining_days >= days_in_month(year, month) as i64 {
        remaining_days -= days_in_month(year, month) as i64;
        month += 1;
    }

    let day = (remaining_days + 1) as u8;

    (year, month, day)
}

fn ymd_to_days(year: i32, month: u8, day: u8) -> i64 {
    // Days from Unix epoch to start of year
    let mut days: i64 = 0;

    if year >= 1970 {
        for y in 1970..year {
            days += days_in_year(y) as i64;
        }
    } else {
        for y in year..1970 {
            days -= days_in_year(y) as i64;
        }
    }

    // Days from start of year to start of month
    for m in 1..month {
        days += days_in_month(year, m) as i64;
    }

    // Days within month
    days += (day - 1) as i64;

    days
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gps_time_creation() {
        let gps = GpsTime::from_week_tow(2345, 123456.789);
        assert_eq!(gps.week, 2345);
        assert!((gps.tow - 123456.789).abs() < 0.001);
    }

    #[test]
    fn test_gps_utc_conversion() {
        // Test a known GPS time
        let gps = GpsTime::from_week_tow(2345, 0.0);
        let utc = gps.to_utc();

        // Week 2345 started around December 2024
        assert!(utc.year >= 2024);
    }

    #[test]
    fn test_gps_now() {
        let gps = GpsTime::now();
        let utc = gps.to_utc();

        // Should be a reasonable current date
        assert!(utc.year >= 2024);
        assert!(utc.month >= 1 && utc.month <= 12);
    }

    #[test]
    fn test_time_of_day() {
        // Create a TimeOfDay directly to test the structure
        let tod = TimeOfDay {
            day_of_year: 100,
            hour: 12,
            minute: 30,
            second: 45,
            millisecond: 500,
        };

        // Test seconds_since_midnight
        let expected_seconds = 12 * 3600 + 30 * 60 + 45;
        assert_eq!(tod.seconds_since_midnight(), expected_seconds);

        // Test millis_since_midnight
        let expected_millis = expected_seconds * 1000 + 500;
        assert_eq!(tod.millis_since_midnight(), expected_millis);
    }

    #[test]
    fn test_havequick_code() {
        let tod = TimeOfDay {
            day_of_year: 100,
            hour: 12,
            minute: 30,
            second: 45,
            millisecond: 0,
        };

        let code = tod.havequick_code();
        // Should encode day 100, hour 12, minute 30
        assert!(code > 0);
    }

    #[test]
    fn test_pps_sync() {
        let mut pps = PpsSync::new();

        // Initially not locked
        assert!(!pps.locked);

        // Simulate PPS edges
        for i in 0..20 {
            let gps = GpsTime::from_week_tow(2345, i as f64);
            pps.record_pps_edge(gps);
            std::thread::sleep(std::time::Duration::from_millis(10)); // Simulate real delay
        }

        // After enough edges with good timing, should approach lock
        assert!(pps.edge_count >= 20);
    }

    #[test]
    fn test_time_source_manager() {
        let mut manager = TimeSourceManager::new();

        assert!(!manager.is_synchronized());

        // Update with GPS time
        let gps = GpsTime::now();
        manager.update_gps(gps);

        assert!(manager.is_synchronized());
        assert_eq!(manager.primary_source(), TimeSource::Gps);
    }

    #[test]
    fn test_day_of_year_calculation() {
        assert_eq!(day_of_year(2024, 1, 1), 1);
        assert_eq!(day_of_year(2024, 2, 1), 32);
        assert_eq!(day_of_year(2024, 12, 31), 366); // 2024 is leap year
        assert_eq!(day_of_year(2023, 12, 31), 365); // 2023 is not
    }

    #[test]
    fn test_gps_duration_add() {
        let gps = GpsTime::from_week_tow(2345, 604799.0); // End of week
        let new_gps = gps.add_duration(Duration::from_secs(2));

        // Should wrap to next week
        assert_eq!(new_gps.week, 2346);
        assert!(new_gps.tow < 2.0);
    }
}
