//! Timing advance estimator for TDMA uplink synchronization.
//!
//! In TDMA systems, a mobile station must transmit its uplink burst *earlier*
//! than the nominal slot boundary so that the signal arrives at the base
//! station aligned with the expected time slot. The required pre-advance is
//! called the **timing advance (TA)** and equals half the round-trip delay
//! (RTD) between the base station and the mobile.
//!
//! This module provides [`TimingAdvanceEstimator`], which:
//!
//! * Estimates round-trip delay from cross-correlation of a known reference
//!   signal and the received signal.
//! * Converts the RTD into a timing advance expressed in samples, microseconds,
//!   and symbols.
//! * Applies exponential-moving-average (EMA) filtering for smooth updates.
//! * Supports LTE-style quantised TA values (0..=1282, step = 16*T_s).
//! * Limits the rate of change between successive updates.
//! * Maintains a history buffer for timing-drift analysis.
//!
//! # Example
//!
//! ```
//! use r4w_core::timing_advance_estimator::{TaConfig, TimingAdvanceEstimator};
//!
//! let config = TaConfig {
//!     sample_rate: 1_920_000.0,
//!     symbol_rate: 15_000.0,
//!     max_advance_symbols: 200.0,
//!     update_interval: 1,
//! };
//!
//! let mut estimator = TimingAdvanceEstimator::new(config);
//!
//! // Simulate a reference signal and a received copy delayed by 100 samples.
//! let n = 512;
//! let delay = 100usize;
//! let reference: Vec<f64> = (0..n).map(|i| (i as f64 * 0.1).sin()).collect();
//! let mut received = vec![0.0f64; n];
//! for i in delay..n {
//!     received[i] = reference[i - delay];
//! }
//!
//! let rtd = estimator.estimate_round_trip(&reference, &received);
//! assert!((rtd - delay as f64).abs() < 1.0);
//!
//! let ta = estimator.compute_advance(rtd);
//! assert!((ta.advance_samples - (delay as f64 / 2.0)).abs() < 0.5);
//! ```

// --- Configuration -----------------------------------------------------------

/// Configuration parameters for the timing-advance estimator.
#[derive(Debug, Clone)]
pub struct TaConfig {
    /// System sample rate in Hz.
    pub sample_rate: f64,
    /// Symbol rate in symbols/s.
    pub symbol_rate: f64,
    /// Maximum allowable timing advance in symbols.
    pub max_advance_symbols: f64,
    /// Number of measurements between TA updates (>= 1).
    pub update_interval: usize,
}

impl Default for TaConfig {
    fn default() -> Self {
        Self {
            sample_rate: 1_920_000.0,
            symbol_rate: 15_000.0,
            max_advance_symbols: 200.0,
            update_interval: 1,
        }
    }
}

// --- TimingAdvance result ----------------------------------------------------

/// A resolved timing-advance measurement.
#[derive(Debug, Clone, Copy)]
pub struct TimingAdvance {
    /// Advance expressed in samples at the configured sample rate.
    pub advance_samples: f64,
    /// Advance expressed in microseconds.
    pub advance_us: f64,
    /// Advance expressed in symbols.
    pub advance_symbols: f64,
    /// Confidence metric in [0, 1] derived from correlation peak quality.
    pub confidence: f64,
}

impl Default for TimingAdvance {
    fn default() -> Self {
        Self {
            advance_samples: 0.0,
            advance_us: 0.0,
            advance_symbols: 0.0,
            confidence: 0.0,
        }
    }
}

// --- Estimator ---------------------------------------------------------------

/// Speed of light in metres per second.
const SPEED_OF_LIGHT: f64 = 299_792_458.0;

/// LTE base sampling period T_s = 1 / (15000 * 2048) ~ 32.55 ns.
const LTE_TS: f64 = 1.0 / (15_000.0 * 2048.0);

/// Maximum LTE TA index value.
const LTE_TA_MAX: u32 = 1282;

/// Timing advance estimator for TDMA uplink synchronization.
///
/// See the [module-level documentation](self) for a full overview.
pub struct TimingAdvanceEstimator {
    config: TaConfig,
    /// Smoothed advance in samples (EMA-filtered).
    filtered_advance: f64,
    /// EMA smoothing factor alpha in (0, 1].
    alpha: f64,
    /// Maximum allowed change in samples per update.
    max_rate: f64,
    /// Number of measurements received since last applied update.
    measurement_count: usize,
    /// History of applied TA values (advance_samples, timestamp index).
    history: Vec<(f64, usize)>,
    /// Monotonically increasing update counter (used as timestamp).
    update_counter: usize,
    /// Whether the estimator has received at least one measurement.
    initialised: bool,
}

impl TimingAdvanceEstimator {
    // -- Construction ---------------------------------------------------------

    /// Create a new estimator with the given configuration.
    ///
    /// The EMA smoothing factor defaults to 0.3 and the maximum rate of
    /// change defaults to 4 samples per update.
    pub fn new(config: TaConfig) -> Self {
        Self {
            config,
            filtered_advance: 0.0,
            alpha: 0.3,
            max_rate: 4.0,
            measurement_count: 0,
            history: Vec::new(),
            update_counter: 0,
            initialised: false,
        }
    }

    /// Set the EMA smoothing factor alpha. Clamped to (0, 1].
    pub fn set_alpha(&mut self, alpha: f64) {
        self.alpha = alpha.clamp(f64::MIN_POSITIVE, 1.0);
    }

    /// Set the maximum rate of change in samples per update.
    pub fn set_max_rate(&mut self, max_rate: f64) {
        self.max_rate = max_rate.abs();
    }

    // -- Round-trip delay estimation ------------------------------------------

    /// Estimate round-trip delay in **samples** by cross-correlating
    /// `reference` and `received`.
    ///
    /// Returns the lag (in samples) of the correlation peak. A positive
    /// value means the received signal is delayed with respect to the
    /// reference.
    pub fn estimate_round_trip(&self, reference: &[f64], received: &[f64]) -> f64 {
        let n = reference.len().min(received.len());
        if n == 0 {
            return 0.0;
        }

        // Compute cross-correlation for non-negative lags.
        let mut best_lag: usize = 0;
        let mut best_val: f64 = f64::NEG_INFINITY;

        for lag in 0..n {
            let mut sum = 0.0;
            for i in 0..(n - lag) {
                sum += reference[i] * received[i + lag];
            }
            if sum > best_val {
                best_val = sum;
                best_lag = lag;
            }
        }

        // Parabolic interpolation around the peak for sub-sample accuracy.
        if best_lag > 0 && best_lag < n - 1 {
            let y_m1 = cross_corr_at_lag(reference, received, best_lag - 1, n);
            let y_0 = best_val;
            let y_p1 = cross_corr_at_lag(reference, received, best_lag + 1, n);
            let denom = 2.0 * (2.0 * y_0 - y_m1 - y_p1);
            if denom.abs() > 1e-30 {
                let delta = (y_m1 - y_p1) / denom;
                return best_lag as f64 + delta;
            }
        }

        best_lag as f64
    }

    // -- TA computation -------------------------------------------------------

    /// Convert a round-trip delay (in samples) to a [`TimingAdvance`].
    ///
    /// TA = RTD / 2 (the mobile must pre-advance by half the round trip).
    pub fn compute_advance(&self, rtd_samples: f64) -> TimingAdvance {
        let half = rtd_samples / 2.0;
        let samples_per_symbol = self.config.sample_rate / self.config.symbol_rate;
        let advance_symbols = half / samples_per_symbol;

        // Clamp to maximum.
        let clamped_symbols = advance_symbols.min(self.config.max_advance_symbols);
        let clamped_samples = clamped_symbols * samples_per_symbol;

        let advance_us = clamped_samples / self.config.sample_rate * 1e6;

        // Confidence: 1.0 when within limits, reduced proportionally when
        // the raw value exceeds max_advance_symbols.
        let confidence = if advance_symbols <= 0.0 {
            0.0
        } else if advance_symbols <= self.config.max_advance_symbols {
            1.0
        } else {
            (self.config.max_advance_symbols / advance_symbols).min(1.0)
        };

        TimingAdvance {
            advance_samples: clamped_samples,
            advance_us,
            advance_symbols: clamped_symbols,
            confidence,
        }
    }

    // -- Filtered update ------------------------------------------------------

    /// Feed a new timing measurement (advance in samples) into the
    /// estimator, applying EMA smoothing and rate-of-change limiting.
    ///
    /// Returns the current (filtered) [`TimingAdvance`] if an update was
    /// applied according to the configured `update_interval`, or `None`
    /// if the measurement was accumulated but no update was emitted yet.
    pub fn update(&mut self, advance_samples: f64) -> Option<TimingAdvance> {
        self.measurement_count += 1;

        if self.measurement_count < self.config.update_interval {
            return None;
        }

        // Reset counter.
        self.measurement_count = 0;

        if !self.initialised {
            self.filtered_advance = advance_samples;
            self.initialised = true;
        } else {
            // EMA.
            let raw = self.alpha * advance_samples + (1.0 - self.alpha) * self.filtered_advance;

            // Rate-of-change limiting.
            let delta = raw - self.filtered_advance;
            let clamped_delta = delta.clamp(-self.max_rate, self.max_rate);
            self.filtered_advance += clamped_delta;
        }

        self.update_counter += 1;
        self.history
            .push((self.filtered_advance, self.update_counter));

        // Build TA from the filtered advance (treat as half-RTD already).
        let samples_per_symbol = self.config.sample_rate / self.config.symbol_rate;
        let advance_symbols = self.filtered_advance / samples_per_symbol;
        let advance_us = self.filtered_advance / self.config.sample_rate * 1e6;

        Some(TimingAdvance {
            advance_samples: self.filtered_advance,
            advance_us,
            advance_symbols,
            confidence: if self.filtered_advance >= 0.0 {
                1.0
            } else {
                0.0
            },
        })
    }

    // -- Range estimation -----------------------------------------------------

    /// Estimate one-way distance in metres from a round-trip delay in
    /// samples.
    ///
    /// distance = c * RTD_seconds / 2
    pub fn range_from_rtd(&self, rtd_samples: f64) -> f64 {
        let rtd_seconds = rtd_samples / self.config.sample_rate;
        SPEED_OF_LIGHT * rtd_seconds / 2.0
    }

    // -- LTE-style quantised TA -----------------------------------------------

    /// Quantise a timing advance (in seconds) to an LTE TA index.
    ///
    /// In LTE the TA command is an integer in 0..=1282 where each step
    /// equals 16 * T_s (T_s = 1/(15000*2048) ~ 32.55 ns).
    ///
    /// Returns `None` if the advance exceeds the representable range.
    pub fn quantise_lte(advance_seconds: f64) -> Option<u32> {
        let step = 16.0 * LTE_TS;
        let index = (advance_seconds / step).round() as i64;
        if index < 0 || index > LTE_TA_MAX as i64 {
            None
        } else {
            Some(index as u32)
        }
    }

    /// Convert an LTE TA index back to seconds.
    pub fn dequantise_lte(ta_index: u32) -> f64 {
        ta_index as f64 * 16.0 * LTE_TS
    }

    // -- History / drift analysis ---------------------------------------------

    /// Return a reference to the history of (advance_samples, update_index)
    /// pairs.
    pub fn history(&self) -> &[(f64, usize)] {
        &self.history
    }

    /// Compute the drift rate in samples per update over the most recent
    /// `window` entries using a least-squares linear fit.
    ///
    /// Returns `None` if fewer than 2 entries are available.
    pub fn drift_rate(&self, window: usize) -> Option<f64> {
        let data = if self.history.len() <= window {
            &self.history[..]
        } else {
            &self.history[self.history.len() - window..]
        };

        let n = data.len();
        if n < 2 {
            return None;
        }

        // Simple linear regression: y = a + b*x, return b.
        let mut sx: f64 = 0.0;
        let mut sy: f64 = 0.0;
        let mut sxx: f64 = 0.0;
        let mut sxy: f64 = 0.0;

        for &(val, idx) in data {
            let x = idx as f64;
            sx += x;
            sy += val;
            sxx += x * x;
            sxy += x * val;
        }

        let nf = n as f64;
        let denom = nf * sxx - sx * sx;
        if denom.abs() < 1e-30 {
            return None;
        }
        Some((nf * sxy - sx * sy) / denom)
    }

    // -- Reset ----------------------------------------------------------------

    /// Reset the estimator to its initial state, clearing history and
    /// filtered value.
    pub fn reset(&mut self) {
        self.filtered_advance = 0.0;
        self.measurement_count = 0;
        self.history.clear();
        self.update_counter = 0;
        self.initialised = false;
    }

    /// Return the current filtered advance in samples, or 0 if no
    /// measurement has been applied yet.
    pub fn current_advance_samples(&self) -> f64 {
        self.filtered_advance
    }
}

// --- Helper ------------------------------------------------------------------

/// Cross-correlation at a single non-negative lag.
fn cross_corr_at_lag(a: &[f64], b: &[f64], lag: usize, n: usize) -> f64 {
    let mut sum = 0.0;
    for i in 0..(n - lag) {
        sum += a[i] * b[i + lag];
    }
    sum
}

// --- Tests -------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> TaConfig {
        TaConfig {
            sample_rate: 1_000_000.0,
            symbol_rate: 10_000.0,
            max_advance_symbols: 100.0,
            update_interval: 1,
        }
    }

    // 1. Basic round-trip estimation with a known delay.
    #[test]
    fn test_estimate_round_trip_known_delay() {
        let cfg = default_config();
        let est = TimingAdvanceEstimator::new(cfg);

        let n = 256;
        let delay = 40usize;
        let reference: Vec<f64> = (0..n).map(|i| ((i as f64 * i as f64) * 0.01).sin()).collect();
        let mut received = vec![0.0; n];
        for i in delay..n {
            received[i] = reference[i - delay];
        }

        let rtd = est.estimate_round_trip(&reference, &received);
        assert!(
            (rtd - delay as f64).abs() < 1.0,
            "Expected RTD ~ {delay}, got {rtd}"
        );
    }

    // 2. compute_advance halves RTD.
    #[test]
    fn test_compute_advance_halves_rtd() {
        let cfg = default_config();
        let est = TimingAdvanceEstimator::new(cfg);

        let rtd = 80.0;
        let ta = est.compute_advance(rtd);
        assert!(
            (ta.advance_samples - 40.0).abs() < 1e-9,
            "advance_samples should be RTD/2"
        );
    }

    // 3. advance_us is consistent with advance_samples.
    #[test]
    fn test_advance_us_consistency() {
        let cfg = default_config();
        let est = TimingAdvanceEstimator::new(cfg.clone());

        let ta = est.compute_advance(200.0); // 200 samples RTD -> 100 samples advance
        let expected_us = 100.0 / cfg.sample_rate * 1e6;
        assert!(
            (ta.advance_us - expected_us).abs() < 1e-6,
            "advance_us mismatch"
        );
    }

    // 4. advance_symbols is consistent.
    #[test]
    fn test_advance_symbols_consistency() {
        let cfg = default_config();
        let est = TimingAdvanceEstimator::new(cfg.clone());

        let ta = est.compute_advance(200.0);
        let samples_per_symbol = cfg.sample_rate / cfg.symbol_rate; // 100
        let expected_symbols = 100.0 / samples_per_symbol; // 1.0
        assert!(
            (ta.advance_symbols - expected_symbols).abs() < 1e-9,
            "advance_symbols mismatch"
        );
    }

    // 5. Clamping to max_advance_symbols reduces confidence.
    #[test]
    fn test_clamping_reduces_confidence() {
        let cfg = default_config(); // max_advance_symbols = 100
        let est = TimingAdvanceEstimator::new(cfg);

        // RTD = 40_000 samples -> advance = 20_000 samples = 200 symbols (exceeds 100)
        let ta = est.compute_advance(40_000.0);
        assert!(
            ta.advance_symbols <= 100.0 + 1e-9,
            "Should be clamped to 100 symbols"
        );
        assert!(ta.confidence < 1.0, "Confidence should be reduced");
        assert!(ta.confidence > 0.0, "Confidence should still be positive");
    }

    // 6. EMA update filtering.
    #[test]
    fn test_ema_filtering() {
        let cfg = default_config();
        let mut est = TimingAdvanceEstimator::new(cfg);
        est.set_alpha(0.5);
        est.set_max_rate(1000.0); // large rate limit so it does not interfere

        // First update initialises.
        let ta1 = est.update(100.0).unwrap();
        assert!((ta1.advance_samples - 100.0).abs() < 1e-9);

        // Second update: EMA = 0.5*200 + 0.5*100 = 150.
        let ta2 = est.update(200.0).unwrap();
        assert!(
            (ta2.advance_samples - 150.0).abs() < 1e-9,
            "EMA mismatch: got {}",
            ta2.advance_samples
        );
    }

    // 7. Rate-of-change limiting.
    #[test]
    fn test_rate_of_change_limiting() {
        let cfg = default_config();
        let mut est = TimingAdvanceEstimator::new(cfg);
        est.set_alpha(1.0); // No smoothing so raw = measurement
        est.set_max_rate(5.0);

        est.update(100.0); // initialise
        let ta = est.update(200.0).unwrap(); // jump of +100, but limited to +5
        assert!(
            (ta.advance_samples - 105.0).abs() < 1e-9,
            "Rate should be limited to +5: got {}",
            ta.advance_samples
        );
    }

    // 8. Update interval suppresses intermediate outputs.
    #[test]
    fn test_update_interval() {
        let mut cfg = default_config();
        cfg.update_interval = 3;
        let mut est = TimingAdvanceEstimator::new(cfg);
        est.set_max_rate(1000.0);

        assert!(est.update(10.0).is_none());
        assert!(est.update(20.0).is_none());
        let ta = est.update(30.0);
        assert!(ta.is_some(), "Should emit on 3rd measurement");
    }

    // 9. Range estimation from RTD.
    #[test]
    fn test_range_from_rtd() {
        let cfg = default_config();
        let est = TimingAdvanceEstimator::new(cfg);

        // RTD of 1000 samples at 1 MHz = 1 ms round trip
        let range = est.range_from_rtd(1000.0);
        let expected = SPEED_OF_LIGHT * 1e-3 / 2.0; // ~149.9 km
        assert!(
            (range - expected).abs() < 1.0,
            "range mismatch: {range} vs {expected}"
        );
    }

    // 10. LTE quantisation round-trip.
    #[test]
    fn test_lte_quantise_roundtrip() {
        let step = 16.0 * LTE_TS;
        let advance_sec = 100.0 * step; // exactly 100 steps

        let idx = TimingAdvanceEstimator::quantise_lte(advance_sec).unwrap();
        assert_eq!(idx, 100);

        let recovered = TimingAdvanceEstimator::dequantise_lte(idx);
        assert!(
            (recovered - advance_sec).abs() < 1e-15,
            "LTE roundtrip failed"
        );
    }

    // 11. LTE quantisation out of range returns None.
    #[test]
    fn test_lte_quantise_out_of_range() {
        let too_large = 2000.0 * 16.0 * LTE_TS;
        assert!(TimingAdvanceEstimator::quantise_lte(too_large).is_none());
        assert!(TimingAdvanceEstimator::quantise_lte(-1.0).is_none());
    }

    // 12. History tracking and drift rate.
    #[test]
    fn test_history_and_drift() {
        let cfg = default_config();
        let mut est = TimingAdvanceEstimator::new(cfg);
        est.set_alpha(1.0);
        est.set_max_rate(1000.0);

        // Feed a linearly increasing sequence: 0, 10, 20, ...
        for i in 0..10 {
            est.update(i as f64 * 10.0);
        }

        assert_eq!(est.history().len(), 10);

        let drift = est.drift_rate(10).unwrap();
        // slope should be ~10 samples per update
        assert!(
            (drift - 10.0).abs() < 0.1,
            "Drift rate should be ~10, got {drift}"
        );
    }

    // 13. Reset clears state.
    #[test]
    fn test_reset() {
        let cfg = default_config();
        let mut est = TimingAdvanceEstimator::new(cfg);
        est.update(50.0);
        assert!(!est.history().is_empty());

        est.reset();
        assert!(est.history().is_empty());
        assert!((est.current_advance_samples()).abs() < 1e-15);
    }

    // 14. Zero-length signals.
    #[test]
    fn test_empty_signals() {
        let cfg = default_config();
        let est = TimingAdvanceEstimator::new(cfg);
        let rtd = est.estimate_round_trip(&[], &[]);
        assert!((rtd).abs() < 1e-15);
    }

    // 15. Zero RTD yields zero advance.
    #[test]
    fn test_zero_rtd() {
        let cfg = default_config();
        let est = TimingAdvanceEstimator::new(cfg);
        let ta = est.compute_advance(0.0);
        assert!((ta.advance_samples).abs() < 1e-15);
        assert!((ta.advance_us).abs() < 1e-15);
    }
}
