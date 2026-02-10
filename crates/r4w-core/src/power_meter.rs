//! Sliding-Window RMS Power Measurement with dBm Output and Carrier Detection
//!
//! Provides real-time power measurement of IQ signals using a sliding window,
//! with output in watts, dBm, and dBFS. Includes carrier detection with
//! hysteresis and a peak meter with configurable attack/decay.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::power_meter::{PowerMeter, watts_to_dbm, dbm_to_watts};
//!
//! // Create a power meter with a 64-sample sliding window
//! let mut meter = PowerMeter::new(64);
//!
//! // Feed a unit-amplitude CW tone (I=1, Q=0)
//! for _ in 0..64 {
//!     meter.update((1.0, 0.0));
//! }
//!
//! // Unit amplitude => 1 W into 1 ohm => 30 dBm
//! let p = meter.power_watts();
//! assert!((p - 1.0).abs() < 1e-10);
//! assert!((meter.power_dbm() - 30.0).abs() < 1e-6);
//!
//! // dBFS: 0 dBFS means 1.0 RMS (full scale)
//! assert!(meter.power_dbfs().abs() < 1e-6);
//!
//! // Conversion helpers round-trip
//! let dbm = watts_to_dbm(0.001);
//! assert!((dbm - 0.0).abs() < 1e-10); // 1 mW = 0 dBm
//! assert!((dbm_to_watts(dbm) - 0.001).abs() < 1e-14);
//! ```

use std::collections::VecDeque;

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Compute RMS power of a slice of IQ samples in one shot (watts, 1-ohm).
///
/// Returns `|I|^2 + |Q|^2` averaged over all samples.
pub fn rms_power(samples: &[(f64, f64)]) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }
    let sum: f64 = samples.iter().map(|&(i, q)| i * i + q * q).sum();
    sum / samples.len() as f64
}

/// Convert power in watts to dBm: `10 * log10(watts * 1000)`.
///
/// Returns `f64::NEG_INFINITY` for non-positive watts.
pub fn watts_to_dbm(watts: f64) -> f64 {
    if watts <= 0.0 {
        f64::NEG_INFINITY
    } else {
        10.0 * (watts * 1000.0).log10()
    }
}

/// Convert dBm to watts: `10^(dbm/10) / 1000`.
pub fn dbm_to_watts(dbm: f64) -> f64 {
    10.0_f64.powf(dbm / 10.0) / 1000.0
}

/// Crest factor (peak-to-RMS ratio) in dB.
///
/// Defined as `20 * log10(peak_amplitude / rms_amplitude)` where amplitudes
/// are the magnitude of IQ samples. Returns 0.0 for empty or zero-power input.
pub fn crest_factor(samples: &[(f64, f64)]) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }
    let rms_pwr = rms_power(samples);
    if rms_pwr <= 0.0 {
        return 0.0;
    }
    let rms_amp = rms_pwr.sqrt();
    let peak_amp = samples
        .iter()
        .map(|&(i, q)| (i * i + q * q).sqrt())
        .fold(0.0_f64, f64::max);
    if peak_amp <= 0.0 {
        return 0.0;
    }
    20.0 * (peak_amp / rms_amp).log10()
}

// ---------------------------------------------------------------------------
// PowerMeter
// ---------------------------------------------------------------------------

/// Sliding-window RMS power meter for IQ signals.
///
/// Maintains a circular buffer of instantaneous power values (`|I|^2 + |Q|^2`)
/// and computes the mean over the window. Power is reported in watts (1-ohm
/// reference impedance), dBm, and dBFS.
#[derive(Debug, Clone)]
pub struct PowerMeter {
    /// Ring buffer of instantaneous power values
    buffer: VecDeque<f64>,
    /// Maximum window size
    window_size: usize,
    /// Running sum of power values in the buffer
    running_sum: f64,
    /// Peak power ever observed (watts)
    peak_watts: f64,
}

impl PowerMeter {
    /// Create a new `PowerMeter` with the given sliding-window size (in samples).
    ///
    /// # Panics
    ///
    /// Panics if `window_size` is zero.
    pub fn new(window_size: usize) -> Self {
        assert!(window_size > 0, "window_size must be > 0");
        Self {
            buffer: VecDeque::with_capacity(window_size),
            window_size,
            running_sum: 0.0,
            peak_watts: 0.0,
        }
    }

    /// Feed a single IQ sample `(I, Q)` into the meter.
    pub fn update(&mut self, sample: (f64, f64)) {
        let (i, q) = sample;
        let power = i * i + q * q;

        // Slide the window
        if self.buffer.len() == self.window_size {
            if let Some(old) = self.buffer.pop_front() {
                self.running_sum -= old;
            }
        }
        self.buffer.push_back(power);
        self.running_sum += power;

        // Track peak
        let current = self.power_watts();
        if current > self.peak_watts {
            self.peak_watts = current;
        }
    }

    /// Feed multiple IQ samples at once.
    pub fn update_batch(&mut self, samples: &[(f64, f64)]) {
        for &s in samples {
            self.update(s);
        }
    }

    /// Instantaneous RMS power in watts (1-ohm reference).
    ///
    /// This is the mean of `|I|^2 + |Q|^2` over the current window.
    pub fn power_watts(&self) -> f64 {
        if self.buffer.is_empty() {
            return 0.0;
        }
        self.running_sum / self.buffer.len() as f64
    }

    /// Power in dBm: `10 * log10(power_watts * 1000)`.
    pub fn power_dbm(&self) -> f64 {
        watts_to_dbm(self.power_watts())
    }

    /// Power relative to full scale in dBFS.
    ///
    /// 0 dBFS corresponds to 1.0 RMS (i.e. power_watts = 1.0).
    pub fn power_dbfs(&self) -> f64 {
        let w = self.power_watts();
        if w <= 0.0 {
            f64::NEG_INFINITY
        } else {
            10.0 * w.log10()
        }
    }

    /// Peak power observed since creation or last reset, in dBm.
    pub fn peak_power_dbm(&self) -> f64 {
        watts_to_dbm(self.peak_watts)
    }

    /// Reset the meter, clearing all state and peak tracking.
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.running_sum = 0.0;
        self.peak_watts = 0.0;
    }
}

// ---------------------------------------------------------------------------
// CarrierDetector
// ---------------------------------------------------------------------------

/// Carrier presence detector with hysteresis.
///
/// Declares a carrier present when power exceeds `threshold_dbm` and absent
/// when it drops below `threshold_dbm - hysteresis_db`.
#[derive(Debug, Clone)]
pub struct CarrierDetector {
    threshold_dbm: f64,
    hysteresis_db: f64,
    detected: bool,
}

impl CarrierDetector {
    /// Create a new carrier detector.
    ///
    /// - `threshold_dbm`: power level above which carrier is declared present.
    /// - `hysteresis_db`: dB below threshold at which carrier is declared absent.
    pub fn new(threshold_dbm: f64, hysteresis_db: f64) -> Self {
        Self {
            threshold_dbm,
            hysteresis_db: hysteresis_db.abs(),
            detected: false,
        }
    }

    /// Update with a new power reading (dBm) and return whether a carrier is
    /// currently detected.
    pub fn update(&mut self, power_dbm: f64) -> bool {
        if self.detected {
            // Drop below threshold minus hysteresis to clear
            if power_dbm < self.threshold_dbm - self.hysteresis_db {
                self.detected = false;
            }
        } else {
            // Rise above threshold to detect
            if power_dbm >= self.threshold_dbm {
                self.detected = true;
            }
        }
        self.detected
    }

    /// Query current detection state without feeding a new measurement.
    pub fn is_detected(&self) -> bool {
        self.detected
    }
}

// ---------------------------------------------------------------------------
// PeakMeter
// ---------------------------------------------------------------------------

/// Peak-hold envelope meter with attack and decay time constants.
///
/// Tracks the envelope of IQ samples in dB using asymmetric time constants:
/// fast attack (short rise time) and slow decay (long fall time).
#[derive(Debug, Clone)]
pub struct PeakMeter {
    /// Attack coefficient: `1.0 / attack_samples`
    attack_coeff: f64,
    /// Decay coefficient: `1.0 / decay_samples`
    decay_coeff: f64,
    /// Current envelope level (linear power)
    envelope: f64,
    /// Peak level seen (linear power)
    peak: f64,
}

impl PeakMeter {
    /// Create a new peak meter.
    ///
    /// - `attack_samples`: number of samples for the envelope to rise to ~63%
    ///   of a step input. Must be >= 1.
    /// - `decay_samples`: number of samples for the envelope to fall to ~37%
    ///   after the signal disappears. Must be >= 1.
    pub fn new(attack_samples: usize, decay_samples: usize) -> Self {
        let attack = attack_samples.max(1);
        let decay = decay_samples.max(1);
        Self {
            attack_coeff: 1.0 / attack as f64,
            decay_coeff: 1.0 / decay as f64,
            envelope: 0.0,
            peak: 0.0,
        }
    }

    /// Feed one IQ sample and return the current envelope level in dB
    /// (relative to 1.0 full-scale power).
    pub fn update(&mut self, sample: (f64, f64)) -> f64 {
        let (i, q) = sample;
        let power = i * i + q * q;

        if power > self.envelope {
            // Attack: rise toward signal
            self.envelope += self.attack_coeff * (power - self.envelope);
        } else {
            // Decay: fall toward signal
            self.envelope += self.decay_coeff * (power - self.envelope);
        }

        if self.envelope > self.peak {
            self.peak = self.envelope;
        }

        self.envelope_db()
    }

    /// Current envelope level in dB (relative to full scale).
    fn envelope_db(&self) -> f64 {
        if self.envelope <= 0.0 {
            f64::NEG_INFINITY
        } else {
            10.0 * self.envelope.log10()
        }
    }

    /// Peak level observed since creation or last reset, in dB.
    pub fn peak_db(&self) -> f64 {
        if self.peak <= 0.0 {
            f64::NEG_INFINITY
        } else {
            10.0 * self.peak.log10()
        }
    }

    /// Reset envelope and peak tracking.
    pub fn reset(&mut self) {
        self.envelope = 0.0;
        self.peak = 0.0;
    }
}

// ---------------------------------------------------------------------------
// Factory functions
// ---------------------------------------------------------------------------

/// Create a wideband power meter with a 1024-sample window.
pub fn wideband_power_meter() -> PowerMeter {
    PowerMeter::new(1024)
}

/// Create a narrowband power meter with a 4096-sample window.
pub fn narrowband_power_meter() -> PowerMeter {
    PowerMeter::new(4096)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unit_amplitude_is_0_dbfs() {
        // A constant I=1,Q=0 tone has power = 1.0 W => 0 dBFS
        let mut meter = PowerMeter::new(64);
        for _ in 0..64 {
            meter.update((1.0, 0.0));
        }
        let dbfs = meter.power_dbfs();
        assert!(
            dbfs.abs() < 1e-6,
            "Expected 0 dBFS for unit amplitude, got {}",
            dbfs
        );
    }

    #[test]
    fn test_dbm_conversion_correctness() {
        // 1 W into 1 ohm => 30 dBm
        let mut meter = PowerMeter::new(32);
        for _ in 0..32 {
            meter.update((1.0, 0.0));
        }
        assert!(
            (meter.power_dbm() - 30.0).abs() < 1e-6,
            "1 W should be 30 dBm, got {}",
            meter.power_dbm()
        );

        // 0.001 W => 0 dBm
        let mut meter2 = PowerMeter::new(32);
        // amplitude = sqrt(0.001) ≈ 0.031623
        let amp = 0.001_f64.sqrt();
        for _ in 0..32 {
            meter2.update((amp, 0.0));
        }
        assert!(
            (meter2.power_dbm() - 0.0).abs() < 1e-4,
            "1 mW should be 0 dBm, got {}",
            meter2.power_dbm()
        );
    }

    #[test]
    fn test_watts_to_dbm_roundtrip() {
        let test_values = [0.001, 0.01, 0.1, 1.0, 10.0, 100.0];
        for &w in &test_values {
            let dbm = watts_to_dbm(w);
            let back = dbm_to_watts(dbm);
            assert!(
                (back - w).abs() < 1e-10,
                "Round-trip failed for {} W: got {} W via {} dBm",
                w,
                back,
                dbm
            );
        }
    }

    #[test]
    fn test_sliding_window_tracks_power_changes() {
        let mut meter = PowerMeter::new(16);
        // Fill with low-power signal (amplitude 0.1 => power 0.01)
        for _ in 0..16 {
            meter.update((0.1, 0.0));
        }
        let low = meter.power_watts();
        assert!(
            (low - 0.01).abs() < 1e-10,
            "Low power: expected 0.01, got {}",
            low
        );

        // Now feed high-power signal (amplitude 1.0 => power 1.0)
        for _ in 0..16 {
            meter.update((1.0, 0.0));
        }
        let high = meter.power_watts();
        assert!(
            (high - 1.0).abs() < 1e-10,
            "High power: expected 1.0, got {}",
            high
        );

        // Power should have risen substantially
        assert!(high > low * 10.0);
    }

    #[test]
    fn test_carrier_detection_with_hysteresis() {
        let mut det = CarrierDetector::new(-10.0, 5.0);
        // Below threshold: no carrier
        assert!(!det.update(-20.0));
        assert!(!det.is_detected());

        // Above threshold: carrier detected
        assert!(det.update(-5.0));
        assert!(det.is_detected());

        // Drop to within hysteresis band: still detected
        assert!(det.update(-12.0));
        assert!(det.is_detected());

        // Drop below threshold - hysteresis (-10 - 5 = -15): carrier lost
        assert!(!det.update(-16.0));
        assert!(!det.is_detected());
    }

    #[test]
    fn test_peak_meter_attack_decay() {
        // Fast attack (1 sample), slow decay (100 samples)
        let mut pm = PeakMeter::new(1, 100);

        // Feed a loud sample
        let db = pm.update((1.0, 0.0));
        assert!(db > -1.0, "Attack should bring envelope up quickly, got {} dB", db);

        // Feed silence: envelope should decay slowly
        let mut prev = pm.update((0.0, 0.0));
        for _ in 0..10 {
            let cur = pm.update((0.0, 0.0));
            // Envelope should be falling (getting more negative in dB)
            assert!(
                cur <= prev + 1e-6,
                "Envelope should decay: prev={}, cur={}",
                prev,
                cur
            );
            prev = cur;
        }

        // Peak should still remember the loud moment
        assert!(
            pm.peak_db() > -1.0,
            "Peak should remember the loud signal, got {} dB",
            pm.peak_db()
        );
    }

    #[test]
    fn test_rms_power_oneshot() {
        // Constant amplitude signal: power = amplitude^2
        let samples: Vec<(f64, f64)> = vec![(0.5, 0.0); 100];
        let p = rms_power(&samples);
        assert!(
            (p - 0.25).abs() < 1e-10,
            "Expected 0.25 W, got {}",
            p
        );

        // Empty input
        assert_eq!(rms_power(&[]), 0.0);
    }

    #[test]
    fn test_crest_factor_cw_tone() {
        // A pure CW tone (constant amplitude) has crest factor = 0 dB
        // because peak == RMS for a constant-envelope signal.
        let samples: Vec<(f64, f64)> = vec![(1.0, 0.0); 256];
        let cf = crest_factor(&samples);
        assert!(
            cf.abs() < 0.01,
            "CW tone crest factor should be ~0 dB, got {} dB",
            cf
        );
    }

    #[test]
    fn test_wideband_factory() {
        let mut meter = wideband_power_meter();
        // Should have window_size = 1024
        assert_eq!(meter.window_size, 1024);
        // Feed some samples and verify it works
        for _ in 0..1024 {
            meter.update((0.5, 0.5));
        }
        // power = 0.25 + 0.25 = 0.5
        assert!(
            (meter.power_watts() - 0.5).abs() < 1e-10,
            "Wideband meter power incorrect: {}",
            meter.power_watts()
        );
    }

    #[test]
    fn test_reset_clears_state() {
        let mut meter = PowerMeter::new(32);
        for _ in 0..32 {
            meter.update((1.0, 0.0));
        }
        assert!(meter.power_watts() > 0.5);
        assert!(meter.peak_power_dbm() > 0.0);

        meter.reset();
        assert_eq!(meter.power_watts(), 0.0);
        assert_eq!(meter.peak_power_dbm(), f64::NEG_INFINITY);
        assert!(meter.buffer.is_empty());
    }
}
