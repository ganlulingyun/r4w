//! Electronic Support Measures (ESM) receiver for detecting, measuring, and
//! classifying RF emitters.
//!
//! An ESM receiver passively intercepts radar and communications signals,
//! extracts pulse descriptors (TOA, frequency, pulse width, amplitude, AOA),
//! estimates pulse repetition intervals, classifies scan patterns, and
//! deinterleaves overlapping pulse trains from multiple emitters.
//!
//! # Example
//!
//! ```
//! use r4w_core::esm_receiver::{EsmReceiver, EsmConfig, ScanType};
//!
//! let config = EsmConfig {
//!     center_freq: 9.4e9,
//!     bandwidth: 2.0e9,
//!     sensitivity_dbm: -60.0,
//!     min_pulse_width_us: 0.5,
//!     max_pulse_width_us: 100.0,
//! };
//!
//! let mut rx = EsmReceiver::new(config);
//!
//! // Synthesize a simple pulsed signal: 1 MHz sample rate, pulse every 2000 samples
//! let sample_rate = 1_000_000.0;
//! let mut samples = vec![(0.0_f64, 0.0_f64); 12_000];
//! for pulse_idx in 0..5 {
//!     let start = pulse_idx * 2000;
//!     for i in start..start + 50 {
//!         if i < samples.len() {
//!             samples[i] = (0.8, 0.0);
//!         }
//!     }
//! }
//!
//! let pulses = rx.detect_pulses(&samples, sample_rate);
//! assert!(pulses.len() >= 4, "should detect multiple pulses");
//!
//! if pulses.len() >= 2 {
//!     let pri = EsmReceiver::measure_pri(&pulses);
//!     assert!(pri.is_some(), "should estimate a PRI");
//!     let scan = EsmReceiver::classify_scan(&pulses);
//!     assert_eq!(scan, ScanType::Fixed);
//! }
//! ```

use std::collections::HashMap;
use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for the ESM receiver.
#[derive(Debug, Clone)]
pub struct EsmConfig {
    /// Center frequency in Hz.
    pub center_freq: f64,
    /// Instantaneous bandwidth in Hz.
    pub bandwidth: f64,
    /// Receiver sensitivity in dBm (detection threshold).
    pub sensitivity_dbm: f64,
    /// Minimum measurable pulse width in microseconds.
    pub min_pulse_width_us: f64,
    /// Maximum measurable pulse width in microseconds.
    pub max_pulse_width_us: f64,
}

impl Default for EsmConfig {
    fn default() -> Self {
        Self {
            center_freq: 9.4e9,
            bandwidth: 2.0e9,
            sensitivity_dbm: -60.0,
            min_pulse_width_us: 0.1,
            max_pulse_width_us: 1000.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Pulse descriptor
// ---------------------------------------------------------------------------

/// Describes a single detected pulse.
#[derive(Debug, Clone)]
pub struct PulseDescriptor {
    /// Time of arrival in seconds relative to the start of the observation.
    pub toa: f64,
    /// Measured frequency in Hz (center of energy).
    pub frequency: f64,
    /// Pulse width in microseconds.
    pub pulse_width: f64,
    /// Peak amplitude (linear magnitude).
    pub amplitude: f64,
    /// Angle of arrival in degrees, if available.
    pub aoa: Option<f64>,
}

// ---------------------------------------------------------------------------
// Scan type classification
// ---------------------------------------------------------------------------

/// Radar antenna scan type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanType {
    /// No scanning - fixed beam (constant amplitude).
    Fixed,
    /// Mechanically rotating antenna (periodic amplitude modulation).
    Rotating,
    /// Conical scan (sinusoidal amplitude variation at the nutating frequency).
    Conical,
    /// Electronically scanned array (amplitude may jump between beams).
    Electronic,
    /// Not enough data to classify.
    Unknown,
}

// ---------------------------------------------------------------------------
// Emitter report
// ---------------------------------------------------------------------------

/// Summary report for a classified emitter.
#[derive(Debug, Clone)]
pub struct EmitterReport {
    /// Unique emitter identifier assigned by the receiver.
    pub emitter_id: u32,
    /// Estimated carrier frequency in Hz.
    pub frequency: f64,
    /// Estimated pulse repetition interval in seconds.
    pub pri: f64,
    /// Typical pulse width in microseconds.
    pub pulse_width: f64,
    /// Classified scan type.
    pub scan_type: ScanType,
}

// ---------------------------------------------------------------------------
// Emitter database (known threat signatures)
// ---------------------------------------------------------------------------

/// A known emitter signature used for threat identification.
#[derive(Debug, Clone)]
pub struct EmitterSignature {
    /// Human-readable emitter name or NATO designation.
    pub name: String,
    /// Nominal frequency in Hz.
    pub frequency: f64,
    /// Frequency tolerance for matching (Hz).
    pub freq_tolerance: f64,
    /// Nominal PRI in seconds.
    pub pri: f64,
    /// PRI tolerance for matching (seconds).
    pub pri_tolerance: f64,
    /// Nominal pulse width in microseconds.
    pub pulse_width: f64,
    /// Pulse width tolerance (microseconds).
    pub pw_tolerance: f64,
}

/// Database of known emitter signatures.
#[derive(Debug, Clone, Default)]
pub struct EmitterDatabase {
    signatures: Vec<EmitterSignature>,
}

impl EmitterDatabase {
    /// Create an empty database.
    pub fn new() -> Self {
        Self {
            signatures: Vec::new(),
        }
    }

    /// Add a known signature.
    pub fn add(&mut self, sig: EmitterSignature) {
        self.signatures.push(sig);
    }

    /// Return the number of stored signatures.
    pub fn len(&self) -> usize {
        self.signatures.len()
    }

    /// Returns true if the database contains no signatures.
    pub fn is_empty(&self) -> bool {
        self.signatures.is_empty()
    }

    /// Find the best matching signature for a measured emitter report.
    ///
    /// Returns `Some((name, score))` where score is in `[0, 1]` (1 = perfect match).
    pub fn match_emitter(&self, report: &EmitterReport) -> Option<(String, f64)> {
        let mut best: Option<(String, f64)> = None;
        for sig in &self.signatures {
            let freq_err = (report.frequency - sig.frequency).abs();
            if freq_err > sig.freq_tolerance {
                continue;
            }
            let pri_err = (report.pri - sig.pri).abs();
            if pri_err > sig.pri_tolerance {
                continue;
            }
            let pw_err = (report.pulse_width - sig.pulse_width).abs();
            if pw_err > sig.pw_tolerance {
                continue;
            }
            // Score: average of how close each parameter is to nominal (1 = exact).
            let freq_score = 1.0 - freq_err / sig.freq_tolerance;
            let pri_score = 1.0 - pri_err / sig.pri_tolerance;
            let pw_score = 1.0 - pw_err / sig.pw_tolerance;
            let score = (freq_score + pri_score + pw_score) / 3.0;
            if best.as_ref().map_or(true, |(_, s)| score > *s) {
                best = Some((sig.name.clone(), score));
            }
        }
        best
    }
}

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------

/// Running statistics for the ESM receiver.
#[derive(Debug, Clone, Default)]
pub struct EsmStatistics {
    /// Total pulses detected.
    pub pulse_count: u64,
    /// Number of distinct emitters identified.
    pub emitter_count: u32,
    /// Estimated intercept probability (0.0 - 1.0).
    pub intercept_probability: f64,
}

// ---------------------------------------------------------------------------
// ESM Receiver
// ---------------------------------------------------------------------------

/// An Electronic Support Measures receiver.
///
/// Detects pulses, measures PRI, classifies scan patterns, deinterleaves
/// multiple emitters, and matches against a threat database.
pub struct EsmReceiver {
    config: EsmConfig,
    stats: EsmStatistics,
    next_emitter_id: u32,
}

impl EsmReceiver {
    /// Create a new ESM receiver with the given configuration.
    pub fn new(config: EsmConfig) -> Self {
        Self {
            config,
            stats: EsmStatistics::default(),
            next_emitter_id: 1,
        }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &EsmConfig {
        &self.config
    }

    /// Return the accumulated statistics.
    pub fn stats(&self) -> &EsmStatistics {
        &self.stats
    }

    // -- Threshold computation ------------------------------------------------

    /// Convert sensitivity in dBm to a linear amplitude threshold.
    fn amplitude_threshold(&self) -> f64 {
        // P(dBm) = 20 log10(A) + 10  (simplified for normalised amplitude)
        // A = 10^((P - 10) / 20)
        // We use a practical mapping: threshold = 10^(sensitivity_dbm / 20)
        // Since sensitivity_dbm is negative (e.g. -60), this gives a small
        // positive number.  The caller passes normalised IQ so we simply use
        // the magnitude directly.
        10.0_f64.powf(self.config.sensitivity_dbm / 20.0)
    }

    // -- Pulse detection ------------------------------------------------------

    /// Detect pulses in a buffer of IQ samples.
    ///
    /// `samples` is a slice of `(I, Q)` pairs.  `sample_rate` is in Hz.
    ///
    /// Returns a vector of [`PulseDescriptor`]s sorted by time of arrival.
    pub fn detect_pulses(
        &mut self,
        samples: &[(f64, f64)],
        sample_rate: f64,
    ) -> Vec<PulseDescriptor> {
        let threshold = self.amplitude_threshold();
        let min_samples =
            (self.config.min_pulse_width_us * 1e-6 * sample_rate).round() as usize;
        let max_samples =
            (self.config.max_pulse_width_us * 1e-6 * sample_rate).round() as usize;

        let mut pulses = Vec::new();
        let n = samples.len();
        let mut i = 0;

        while i < n {
            let (re, im) = samples[i];
            let mag = (re * re + im * im).sqrt();
            if mag >= threshold {
                // Start of a pulse - find where it ends.
                let start = i;
                let mut peak_mag = mag;
                let mut peak_phase = im.atan2(re);
                i += 1;
                while i < n {
                    let (re2, im2) = samples[i];
                    let m = (re2 * re2 + im2 * im2).sqrt();
                    if m < threshold {
                        break;
                    }
                    if m > peak_mag {
                        peak_mag = m;
                        peak_phase = im2.atan2(re2);
                    }
                    i += 1;
                }
                let pulse_len = i - start;
                if pulse_len >= min_samples.max(1) && pulse_len <= max_samples.max(1) {
                    let toa = start as f64 / sample_rate;
                    let pw_us = pulse_len as f64 / sample_rate * 1e6;

                    // Simple frequency estimate: use phase of peak sample as a
                    // proxy; a real system would do an FFT inside the pulse.
                    let freq = self.config.center_freq
                        + peak_phase / (2.0 * PI) * self.config.bandwidth;

                    pulses.push(PulseDescriptor {
                        toa,
                        frequency: freq,
                        pulse_width: pw_us,
                        amplitude: peak_mag,
                        aoa: None,
                    });
                }
            } else {
                i += 1;
            }
        }

        self.stats.pulse_count += pulses.len() as u64;
        pulses
    }

    // -- PRI estimation -------------------------------------------------------

    /// Estimate the Pulse Repetition Interval from a sequence of pulse
    /// descriptors.
    ///
    /// Returns `Some(pri)` in seconds when a consistent interval is found, or
    /// `None` if fewer than 2 pulses are provided or the intervals are too
    /// inconsistent (jitter > 25 %).
    pub fn measure_pri(pulses: &[PulseDescriptor]) -> Option<f64> {
        if pulses.len() < 2 {
            return None;
        }
        let mut intervals: Vec<f64> = pulses
            .windows(2)
            .map(|w| w[1].toa - w[0].toa)
            .collect();
        intervals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        // Median interval
        let median = if intervals.len() % 2 == 0 {
            (intervals[intervals.len() / 2 - 1] + intervals[intervals.len() / 2]) / 2.0
        } else {
            intervals[intervals.len() / 2]
        };

        if median <= 0.0 {
            return None;
        }

        // Check jitter: reject if MAD > 25% of median
        let mad: f64 = intervals
            .iter()
            .map(|dt| (dt - median).abs())
            .sum::<f64>()
            / intervals.len() as f64;
        if mad / median > 0.25 {
            return None;
        }

        Some(median)
    }

    // -- Scan classification --------------------------------------------------

    /// Classify the antenna scan type from amplitude variation across pulses.
    ///
    /// Uses coefficient of variation (CV) and periodicity of amplitude to
    /// distinguish fixed, rotating, conical, and electronic scan patterns.
    pub fn classify_scan(pulses: &[PulseDescriptor]) -> ScanType {
        if pulses.len() < 4 {
            return ScanType::Unknown;
        }
        let amps: Vec<f64> = pulses.iter().map(|p| p.amplitude).collect();
        let mean = amps.iter().sum::<f64>() / amps.len() as f64;
        if mean <= 0.0 {
            return ScanType::Unknown;
        }
        let variance =
            amps.iter().map(|a| (a - mean).powi(2)).sum::<f64>() / amps.len() as f64;
        let cv = variance.sqrt() / mean;

        if cv < 0.05 {
            // Nearly constant amplitude => fixed beam.
            return ScanType::Fixed;
        }

        // Look for periodicity in amplitude via simple autocorrelation.
        let n = amps.len();
        let norm: f64 = amps.iter().map(|a| (a - mean).powi(2)).sum();
        if norm == 0.0 {
            return ScanType::Fixed;
        }

        let mut best_lag = 0;
        let mut best_corr = 0.0_f64;
        for lag in 2..n / 2 {
            let mut r = 0.0;
            for j in 0..n - lag {
                r += (amps[j] - mean) * (amps[j + lag] - mean);
            }
            r /= norm;
            if r > best_corr {
                best_corr = r;
                best_lag = lag;
            }
        }

        if best_corr > 0.6 && best_lag > 0 {
            // Check if the waveform looks sinusoidal (conical) vs. peaked
            // (rotating).  Conical scans show smooth sinusoidal amplitude.
            // Use the CV as a rough discriminator.
            if cv < 0.3 {
                return ScanType::Conical;
            }
            return ScanType::Rotating;
        }

        // High CV but no strong periodicity => likely electronic scan.
        if cv > 0.15 {
            return ScanType::Electronic;
        }

        ScanType::Unknown
    }

    // -- Deinterleaving -------------------------------------------------------

    /// Deinterleave an interleaved pulse train into groups likely originating
    /// from different emitters.
    ///
    /// Uses simple PRI-based histogram clustering: for each pair of successive
    /// pulses the TOA difference is quantised into bins of `bin_width` seconds.
    /// Clusters above a threshold are treated as distinct emitters.
    ///
    /// Returns a map from a cluster identifier to the pulse indices belonging
    /// to that cluster.
    pub fn deinterleave(
        pulses: &[PulseDescriptor],
        bin_width: f64,
    ) -> HashMap<u32, Vec<usize>> {
        if pulses.is_empty() || bin_width <= 0.0 {
            return HashMap::new();
        }

        // Build a histogram of all TOA differences (not just consecutive).
        let mut histogram: HashMap<i64, Vec<(usize, usize)>> = HashMap::new();
        for i in 0..pulses.len() {
            for j in (i + 1)..pulses.len() {
                let dt = pulses[j].toa - pulses[i].toa;
                if dt <= 0.0 {
                    continue;
                }
                let bin = (dt / bin_width).round() as i64;
                histogram.entry(bin).or_default().push((i, j));
            }
        }

        // Find dominant bins (those with count >= 2 pairs).
        let mut dominant_bins: Vec<(i64, usize)> = histogram
            .iter()
            .filter(|(_, pairs)| pairs.len() >= 2)
            .map(|(bin, pairs)| (*bin, pairs.len()))
            .collect();
        dominant_bins.sort_by(|a, b| b.1.cmp(&a.1));

        // Assign pulses to clusters greedily.
        let mut assignments: HashMap<u32, Vec<usize>> = HashMap::new();
        let mut assigned = vec![false; pulses.len()];
        let mut cluster_id = 0_u32;

        for (bin, _) in dominant_bins.iter().take(16) {
            if let Some(pairs) = histogram.get(bin) {
                let mut indices: Vec<usize> = Vec::new();
                for &(a, b) in pairs {
                    if !assigned[a] {
                        indices.push(a);
                    }
                    if !assigned[b] {
                        indices.push(b);
                    }
                }
                indices.sort_unstable();
                indices.dedup();
                if indices.len() >= 2 {
                    for &idx in &indices {
                        assigned[idx] = true;
                    }
                    assignments.insert(cluster_id, indices);
                    cluster_id += 1;
                }
            }
        }

        // Any remaining unassigned pulses go into a "residual" cluster.
        let residual: Vec<usize> = (0..pulses.len())
            .filter(|i| !assigned[*i])
            .collect();
        if !residual.is_empty() {
            assignments.insert(cluster_id, residual);
        }

        assignments
    }

    // -- Emitter report generation --------------------------------------------

    /// Build an [`EmitterReport`] from a group of pulses believed to belong to
    /// a single emitter.
    pub fn build_report(&mut self, pulses: &[PulseDescriptor]) -> Option<EmitterReport> {
        if pulses.is_empty() {
            return None;
        }
        let freq = pulses.iter().map(|p| p.frequency).sum::<f64>() / pulses.len() as f64;
        let pw = pulses.iter().map(|p| p.pulse_width).sum::<f64>() / pulses.len() as f64;
        let pri = Self::measure_pri(pulses).unwrap_or(0.0);
        let scan = Self::classify_scan(pulses);

        let id = self.next_emitter_id;
        self.next_emitter_id += 1;
        self.stats.emitter_count += 1;

        // Simple intercept probability: fraction of expected pulses actually
        // captured.  If PRI > 0, expected count is observation_window / PRI.
        if pri > 0.0 && pulses.len() >= 2 {
            let window = pulses.last().unwrap().toa - pulses.first().unwrap().toa;
            if window > 0.0 {
                let expected = window / pri;
                let actual = pulses.len() as f64;
                self.stats.intercept_probability =
                    (actual / expected).min(1.0);
            }
        }

        Some(EmitterReport {
            emitter_id: id,
            frequency: freq,
            pri,
            pulse_width: pw,
            scan_type: scan,
        })
    }

    /// Convenience: detect, deinterleave, and classify in one call.
    pub fn process(
        &mut self,
        samples: &[(f64, f64)],
        sample_rate: f64,
        bin_width: f64,
    ) -> Vec<EmitterReport> {
        let pulses = self.detect_pulses(samples, sample_rate);
        let clusters = Self::deinterleave(&pulses, bin_width);
        let mut reports = Vec::new();
        for (_cid, indices) in &clusters {
            let group: Vec<PulseDescriptor> =
                indices.iter().map(|&i| pulses[i].clone()).collect();
            if let Some(r) = self.build_report(&group) {
                reports.push(r);
            }
        }
        reports
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a pulsed IQ signal.
    fn make_pulse_train(
        _sample_rate: f64,
        num_pulses: usize,
        pri_samples: usize,
        pulse_width_samples: usize,
        amplitude: f64,
    ) -> Vec<(f64, f64)> {
        let total = num_pulses * pri_samples + pulse_width_samples;
        let mut samples = vec![(0.0, 0.0); total];
        for p in 0..num_pulses {
            let start = p * pri_samples;
            for s in start..start + pulse_width_samples {
                if s < samples.len() {
                    samples[s] = (amplitude, 0.0);
                }
            }
        }
        samples
    }

    #[test]
    fn test_default_config() {
        let cfg = EsmConfig::default();
        assert!(cfg.bandwidth > 0.0);
        assert!(cfg.sensitivity_dbm < 0.0);
        assert!(cfg.min_pulse_width_us < cfg.max_pulse_width_us);
    }

    #[test]
    fn test_detect_single_pulse() {
        let config = EsmConfig {
            sensitivity_dbm: -20.0,
            min_pulse_width_us: 1.0,
            max_pulse_width_us: 200.0,
            ..EsmConfig::default()
        };
        let mut rx = EsmReceiver::new(config);
        let sr = 1_000_000.0;
        // Single pulse of 10 samples at amplitude 0.5
        let mut samples = vec![(0.0, 0.0); 1000];
        for i in 100..110 {
            samples[i] = (0.5, 0.0);
        }
        let pulses = rx.detect_pulses(&samples, sr);
        assert_eq!(pulses.len(), 1);
        let pw = pulses[0].pulse_width;
        assert!((pw - 10.0).abs() < 1.0, "pulse width ~10 us, got {pw}");
    }

    #[test]
    fn test_detect_multiple_pulses() {
        let config = EsmConfig {
            sensitivity_dbm: -20.0,
            min_pulse_width_us: 1.0,
            max_pulse_width_us: 200.0,
            ..EsmConfig::default()
        };
        let mut rx = EsmReceiver::new(config);
        let sr = 1_000_000.0;
        let samples = make_pulse_train(sr, 5, 2000, 50, 0.8);
        let pulses = rx.detect_pulses(&samples, sr);
        assert_eq!(pulses.len(), 5);
    }

    #[test]
    fn test_below_threshold_not_detected() {
        let config = EsmConfig {
            sensitivity_dbm: -10.0, // threshold ~0.316
            min_pulse_width_us: 1.0,
            max_pulse_width_us: 200.0,
            ..EsmConfig::default()
        };
        let mut rx = EsmReceiver::new(config);
        let sr = 1_000_000.0;
        // Amplitude 0.1 is below threshold
        let samples = make_pulse_train(sr, 3, 1000, 20, 0.1);
        let pulses = rx.detect_pulses(&samples, sr);
        assert!(pulses.is_empty());
    }

    #[test]
    fn test_measure_pri_constant() {
        let pulses: Vec<PulseDescriptor> = (0..6)
            .map(|i| PulseDescriptor {
                toa: i as f64 * 0.001, // 1 ms PRI
                frequency: 9.4e9,
                pulse_width: 10.0,
                amplitude: 0.5,
                aoa: None,
            })
            .collect();
        let pri = EsmReceiver::measure_pri(&pulses).unwrap();
        assert!(
            (pri - 0.001).abs() < 1e-9,
            "expected ~1 ms PRI, got {pri}"
        );
    }

    #[test]
    fn test_measure_pri_too_few() {
        let pulses = vec![PulseDescriptor {
            toa: 0.0,
            frequency: 9.4e9,
            pulse_width: 10.0,
            amplitude: 0.5,
            aoa: None,
        }];
        assert!(EsmReceiver::measure_pri(&pulses).is_none());
        assert!(EsmReceiver::measure_pri(&[]).is_none());
    }

    #[test]
    fn test_classify_scan_fixed() {
        let pulses: Vec<PulseDescriptor> = (0..10)
            .map(|i| PulseDescriptor {
                toa: i as f64 * 0.001,
                frequency: 9.4e9,
                pulse_width: 10.0,
                amplitude: 1.0, // constant
                aoa: None,
            })
            .collect();
        assert_eq!(EsmReceiver::classify_scan(&pulses), ScanType::Fixed);
    }

    #[test]
    fn test_classify_scan_unknown_few_pulses() {
        let pulses: Vec<PulseDescriptor> = (0..2)
            .map(|i| PulseDescriptor {
                toa: i as f64 * 0.001,
                frequency: 9.4e9,
                pulse_width: 10.0,
                amplitude: 1.0,
                aoa: None,
            })
            .collect();
        assert_eq!(EsmReceiver::classify_scan(&pulses), ScanType::Unknown);
    }

    #[test]
    fn test_classify_scan_rotating() {
        // Simulate a rotating scan: strong periodic amplitude modulation with
        // a peak at every 20th pulse (period = 20 pulses).
        let n = 80;
        let period = 20.0;
        let pulses: Vec<PulseDescriptor> = (0..n)
            .map(|i| {
                let phase = (i as f64 % period) / period;
                let amp = (-(phase - 0.5).powi(2) * 50.0).exp();
                PulseDescriptor {
                    toa: i as f64 * 0.001,
                    frequency: 9.4e9,
                    pulse_width: 10.0,
                    amplitude: 0.1 + amp,
                    aoa: None,
                }
            })
            .collect();
        let scan = EsmReceiver::classify_scan(&pulses);
        assert!(
            scan == ScanType::Rotating || scan == ScanType::Conical,
            "expected Rotating or Conical, got {scan:?}"
        );
    }

    #[test]
    fn test_deinterleave_two_emitters() {
        // Two emitters with different PRIs.
        let pri_a = 0.001; // 1 ms
        let pri_b = 0.0007; // 0.7 ms
        let mut pulses: Vec<PulseDescriptor> = Vec::new();
        for i in 0..10 {
            pulses.push(PulseDescriptor {
                toa: i as f64 * pri_a,
                frequency: 9.4e9,
                pulse_width: 10.0,
                amplitude: 0.8,
                aoa: None,
            });
        }
        for i in 0..12 {
            pulses.push(PulseDescriptor {
                toa: i as f64 * pri_b + 0.0001, // slight offset
                frequency: 5.6e9,
                pulse_width: 5.0,
                amplitude: 0.6,
                aoa: None,
            });
        }
        // Sort by TOA (as a real receiver would see them).
        pulses.sort_by(|a, b| a.toa.partial_cmp(&b.toa).unwrap());

        let clusters = EsmReceiver::deinterleave(&pulses, 0.0001);
        // We expect at least 2 clusters.
        assert!(
            clusters.len() >= 2,
            "expected >=2 clusters, got {}",
            clusters.len()
        );
    }

    #[test]
    fn test_emitter_database_match() {
        let mut db = EmitterDatabase::new();
        db.add(EmitterSignature {
            name: "Generic-X".into(),
            frequency: 9.4e9,
            freq_tolerance: 100e6,
            pri: 0.001,
            pri_tolerance: 0.0002,
            pulse_width: 10.0,
            pw_tolerance: 5.0,
        });
        assert_eq!(db.len(), 1);
        assert!(!db.is_empty());

        let report = EmitterReport {
            emitter_id: 1,
            frequency: 9.38e9,
            pri: 0.00095,
            pulse_width: 11.0,
            scan_type: ScanType::Fixed,
        };
        let m = db.match_emitter(&report);
        assert!(m.is_some());
        let (name, score) = m.unwrap();
        assert_eq!(name, "Generic-X");
        assert!(score > 0.5, "score should be > 0.5, got {score}");
    }

    #[test]
    fn test_emitter_database_no_match() {
        let mut db = EmitterDatabase::new();
        db.add(EmitterSignature {
            name: "Narrow".into(),
            frequency: 3.0e9,
            freq_tolerance: 10e6,
            pri: 0.005,
            pri_tolerance: 0.0001,
            pulse_width: 50.0,
            pw_tolerance: 2.0,
        });
        let report = EmitterReport {
            emitter_id: 2,
            frequency: 9.4e9, // way off
            pri: 0.001,
            pulse_width: 10.0,
            scan_type: ScanType::Fixed,
        };
        assert!(db.match_emitter(&report).is_none());
    }

    #[test]
    fn test_statistics_updated() {
        let config = EsmConfig {
            sensitivity_dbm: -20.0,
            min_pulse_width_us: 1.0,
            max_pulse_width_us: 200.0,
            ..EsmConfig::default()
        };
        let mut rx = EsmReceiver::new(config);
        assert_eq!(rx.stats().pulse_count, 0);

        let sr = 1_000_000.0;
        let samples = make_pulse_train(sr, 4, 2000, 30, 0.8);
        let pulses = rx.detect_pulses(&samples, sr);
        assert_eq!(rx.stats().pulse_count, pulses.len() as u64);
        assert!(rx.stats().pulse_count >= 4);
    }

    #[test]
    fn test_build_report() {
        let config = EsmConfig::default();
        let mut rx = EsmReceiver::new(config);
        let pulses: Vec<PulseDescriptor> = (0..5)
            .map(|i| PulseDescriptor {
                toa: i as f64 * 0.002,
                frequency: 9.4e9,
                pulse_width: 15.0,
                amplitude: 0.7,
                aoa: Some(45.0),
            })
            .collect();
        let report = rx.build_report(&pulses).unwrap();
        assert_eq!(report.emitter_id, 1);
        assert!((report.frequency - 9.4e9).abs() < 1.0);
        assert!((report.pri - 0.002).abs() < 1e-9);
        assert!((report.pulse_width - 15.0).abs() < 0.01);
        assert_eq!(report.scan_type, ScanType::Fixed);
    }

    #[test]
    fn test_process_end_to_end() {
        let config = EsmConfig {
            sensitivity_dbm: -20.0,
            min_pulse_width_us: 1.0,
            max_pulse_width_us: 200.0,
            ..EsmConfig::default()
        };
        let mut rx = EsmReceiver::new(config);
        let sr = 1_000_000.0;
        let samples = make_pulse_train(sr, 8, 2000, 40, 0.9);
        let reports = rx.process(&samples, sr, 0.0001);
        assert!(
            !reports.is_empty(),
            "should produce at least one emitter report"
        );
        assert!(rx.stats().pulse_count >= 8);
    }
}
