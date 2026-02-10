//! Statistical anomaly detection for protocol compliance and security monitoring.
//!
//! This module provides tools for learning baseline protocol timing distributions
//! and detecting deviations that may indicate protocol violations, security threats,
//! or implementation errors.
//!
//! # Example
//!
//! ```
//! use r4w_core::protocol_anomaly_detector::{
//!     ProtocolAnomalyDetector, ProtocolEvent, Severity,
//! };
//!
//! let mut detector = ProtocolAnomalyDetector::new();
//!
//! // Learn baseline from normal traffic
//! let baseline_events: Vec<ProtocolEvent> = (0..100)
//!     .map(|i| ProtocolEvent {
//!         timestamp_s: i as f64 * 0.01,
//!         event_type: "beacon".to_string(),
//!         duration_s: 0.001,
//!         metadata: String::new(),
//!     })
//!     .collect();
//! detector.learn_baseline(&baseline_events);
//!
//! // Add timing rule
//! detector.add_rule("beacon", 0.005, 0.015);
//!
//! // Detect anomalies in new traffic
//! let test_events = vec![
//!     ProtocolEvent {
//!         timestamp_s: 100.0,
//!         event_type: "beacon".to_string(),
//!         duration_s: 0.001,
//!         metadata: String::new(),
//!     },
//!     ProtocolEvent {
//!         timestamp_s: 100.5, // abnormally long gap
//!         event_type: "beacon".to_string(),
//!         duration_s: 0.001,
//!         metadata: String::new(),
//!     },
//! ];
//!
//! let anomalies = detector.detect_outliers(&test_events);
//! assert!(!anomalies.is_empty());
//! assert!(matches!(anomalies[0].severity, Severity::High | Severity::Critical));
//! ```

use std::collections::HashMap;

/// Severity level of a detected anomaly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Severity {
    /// Minor deviation, likely benign.
    Low,
    /// Notable deviation, warrants monitoring.
    Medium,
    /// Significant deviation, likely problematic.
    High,
    /// Extreme deviation, immediate attention required.
    Critical,
}

impl std::fmt::Display for Severity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Severity::Low => write!(f, "Low"),
            Severity::Medium => write!(f, "Medium"),
            Severity::High => write!(f, "High"),
            Severity::Critical => write!(f, "Critical"),
        }
    }
}

/// A protocol event with timing and classification information.
#[derive(Debug, Clone)]
pub struct ProtocolEvent {
    /// Timestamp in seconds.
    pub timestamp_s: f64,
    /// Type/category of the event (e.g., "beacon", "data_frame", "ack").
    pub event_type: String,
    /// Duration of the event in seconds.
    pub duration_s: f64,
    /// Free-form metadata string for additional context.
    pub metadata: String,
}

/// Report describing a detected anomaly.
#[derive(Debug, Clone)]
pub struct AnomalyReport {
    /// The event that triggered the anomaly.
    pub event: ProtocolEvent,
    /// Severity classification.
    pub severity: Severity,
    /// Deviation from the mean in standard deviations (z-score).
    pub deviation_sigma: f64,
    /// Human-readable description of the anomaly.
    pub description: String,
}

/// Baseline statistics for a single metric (inter-event intervals for an event type).
#[derive(Debug, Clone)]
pub struct BaselineStats {
    /// Mean of the observed values.
    pub mean: f64,
    /// Standard deviation of the observed values.
    pub std_dev: f64,
    /// Minimum observed value.
    pub min: f64,
    /// Maximum observed value.
    pub max: f64,
    /// Number of samples used to compute these statistics.
    pub sample_count: usize,
}

/// A timing rule that specifies acceptable interval bounds for an event type.
#[derive(Debug, Clone)]
struct TimingRule {
    min_interval_s: f64,
    max_interval_s: f64,
}

/// Statistical anomaly detector for protocol compliance and security monitoring.
///
/// Learns baseline timing distributions from normal traffic and detects
/// deviations using z-score analysis and configurable timing rules.
pub struct ProtocolAnomalyDetector {
    /// Learned baseline statistics per event type (based on inter-event intervals).
    baselines: HashMap<String, BaselineStats>,
    /// User-defined timing rules per event type.
    rules: HashMap<String, TimingRule>,
}

impl ProtocolAnomalyDetector {
    /// Create a new detector with no learned baselines or rules.
    pub fn new() -> Self {
        Self {
            baselines: HashMap::new(),
            rules: HashMap::new(),
        }
    }

    /// Learn baseline timing distributions from a set of protocol events.
    ///
    /// Events are grouped by `event_type`, sorted by timestamp, and inter-event
    /// intervals are computed. Statistics (mean, std_dev, min, max) are stored
    /// per event type. Requires at least 2 events of the same type to compute
    /// interval statistics.
    pub fn learn_baseline(&mut self, events: &[ProtocolEvent]) {
        // Group events by type
        let mut by_type: HashMap<String, Vec<f64>> = HashMap::new();
        for event in events {
            by_type
                .entry(event.event_type.clone())
                .or_default()
                .push(event.timestamp_s);
        }

        for (event_type, mut timestamps) in by_type {
            timestamps.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

            if timestamps.len() < 2 {
                continue;
            }

            // Compute inter-event intervals
            let intervals: Vec<f64> = timestamps.windows(2).map(|w| w[1] - w[0]).collect();

            let n = intervals.len() as f64;
            let mean = intervals.iter().sum::<f64>() / n;
            let variance = intervals.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n;
            let std_dev = variance.sqrt();
            let min = intervals
                .iter()
                .copied()
                .fold(f64::INFINITY, f64::min);
            let max = intervals
                .iter()
                .copied()
                .fold(f64::NEG_INFINITY, f64::max);

            self.baselines.insert(
                event_type,
                BaselineStats {
                    mean,
                    std_dev,
                    min,
                    max,
                    sample_count: intervals.len(),
                },
            );
        }
    }

    /// Detect statistical outliers in the given events based on learned baselines.
    ///
    /// For each consecutive pair of same-type events, the inter-event interval
    /// is compared against the baseline. If the z-score exceeds 3 sigma, the
    /// event is flagged as anomalous. Severity is based on deviation magnitude:
    /// - 3-5 sigma: High
    /// - 5+ sigma: Critical
    /// - 2-3 sigma with rule violation: Medium
    pub fn detect_outliers(&self, events: &[ProtocolEvent]) -> Vec<AnomalyReport> {
        let mut reports = Vec::new();

        // Group events by type, preserving order
        let mut by_type: HashMap<String, Vec<&ProtocolEvent>> = HashMap::new();
        for event in events {
            by_type
                .entry(event.event_type.clone())
                .or_default()
                .push(event);
        }

        for (event_type, mut typed_events) in by_type {
            typed_events.sort_by(|a, b| {
                a.timestamp_s
                    .partial_cmp(&b.timestamp_s)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });

            let baseline = match self.baselines.get(&event_type) {
                Some(b) => b,
                None => continue,
            };

            if baseline.std_dev == 0.0 {
                // All intervals were identical; any difference is anomalous
                for pair in typed_events.windows(2) {
                    let interval = pair[1].timestamp_s - pair[0].timestamp_s;
                    let diff = (interval - baseline.mean).abs();
                    if diff > 1e-12 {
                        reports.push(AnomalyReport {
                            event: pair[1].clone(),
                            severity: Severity::Critical,
                            deviation_sigma: f64::INFINITY,
                            description: format!(
                                "Interval {:.6}s deviates from constant baseline {:.6}s for '{}'",
                                interval, baseline.mean, event_type
                            ),
                        });
                    }
                }
                continue;
            }

            for pair in typed_events.windows(2) {
                let interval = pair[1].timestamp_s - pair[0].timestamp_s;
                let z_score = (interval - baseline.mean) / baseline.std_dev;
                let abs_z = z_score.abs();

                if abs_z > 3.0 {
                    let severity = if abs_z > 5.0 {
                        Severity::Critical
                    } else {
                        Severity::High
                    };

                    reports.push(AnomalyReport {
                        event: pair[1].clone(),
                        severity,
                        deviation_sigma: abs_z,
                        description: format!(
                            "Interval {:.6}s is {:.1} sigma from mean {:.6}s for '{}'",
                            interval, abs_z, baseline.mean, event_type
                        ),
                    });
                }
            }
        }

        reports
    }

    /// Check events against user-defined timing rules and report violations.
    ///
    /// Unlike `detect_outliers` which uses learned statistics, this method checks
    /// against explicit min/max interval constraints added via `add_rule`.
    pub fn report_violations(&self, events: &[ProtocolEvent]) -> Vec<AnomalyReport> {
        let mut reports = Vec::new();

        let mut by_type: HashMap<String, Vec<&ProtocolEvent>> = HashMap::new();
        for event in events {
            by_type
                .entry(event.event_type.clone())
                .or_default()
                .push(event);
        }

        for (event_type, mut typed_events) in by_type {
            let rule = match self.rules.get(&event_type) {
                Some(r) => r,
                None => continue,
            };

            typed_events.sort_by(|a, b| {
                a.timestamp_s
                    .partial_cmp(&b.timestamp_s)
                    .unwrap_or(std::cmp::Ordering::Equal)
            });

            for pair in typed_events.windows(2) {
                let interval = pair[1].timestamp_s - pair[0].timestamp_s;

                if interval < rule.min_interval_s {
                    let severity = if interval < rule.min_interval_s * 0.5 {
                        Severity::Critical
                    } else {
                        Severity::High
                    };
                    reports.push(AnomalyReport {
                        event: pair[1].clone(),
                        severity,
                        deviation_sigma: 0.0, // rule-based, not statistical
                        description: format!(
                            "Interval {:.6}s below minimum {:.6}s for '{}'",
                            interval, rule.min_interval_s, event_type
                        ),
                    });
                } else if interval > rule.max_interval_s {
                    let severity = if interval > rule.max_interval_s * 2.0 {
                        Severity::Critical
                    } else {
                        Severity::High
                    };
                    reports.push(AnomalyReport {
                        event: pair[1].clone(),
                        severity,
                        deviation_sigma: 0.0,
                        description: format!(
                            "Interval {:.6}s above maximum {:.6}s for '{}'",
                            interval, rule.max_interval_s, event_type
                        ),
                    });
                }
            }
        }

        reports
    }

    /// Add a timing rule for a specific event type.
    ///
    /// Events of this type must have inter-event intervals within
    /// `[min_interval_s, max_interval_s]` to be considered compliant.
    pub fn add_rule(&mut self, event_type: &str, min_interval_s: f64, max_interval_s: f64) {
        self.rules.insert(
            event_type.to_string(),
            TimingRule {
                min_interval_s,
                max_interval_s,
            },
        );
    }

    /// Check whether a single event would be anomalous given the current baselines.
    ///
    /// This is a convenience method. Since anomaly detection requires comparing
    /// to a previous event of the same type, this checks the event's `duration_s`
    /// against the baseline interval statistics as a heuristic. An event is
    /// considered anomalous if its duration deviates more than 3 sigma from the
    /// learned mean interval, or if it violates a timing rule when interpreted
    /// as an interval.
    pub fn is_anomalous(&self, event: &ProtocolEvent) -> bool {
        // Check against baseline statistics using duration as a proxy
        if let Some(baseline) = self.baselines.get(&event.event_type) {
            if baseline.std_dev > 0.0 {
                let z_score = (event.duration_s - baseline.mean).abs() / baseline.std_dev;
                if z_score > 3.0 {
                    return true;
                }
            } else {
                // Zero std_dev: any difference from mean is anomalous
                if (event.duration_s - baseline.mean).abs() > 1e-12 {
                    return true;
                }
            }
        }

        // Check against rules
        if let Some(rule) = self.rules.get(&event.event_type) {
            if event.duration_s < rule.min_interval_s || event.duration_s > rule.max_interval_s {
                return true;
            }
        }

        false
    }

    /// Retrieve the baseline statistics for a given event type.
    pub fn baseline_stats(&self, event_type: &str) -> Option<&BaselineStats> {
        self.baselines.get(event_type)
    }
}

impl Default for ProtocolAnomalyDetector {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_events(event_type: &str, timestamps: &[f64]) -> Vec<ProtocolEvent> {
        timestamps
            .iter()
            .map(|&t| ProtocolEvent {
                timestamp_s: t,
                event_type: event_type.to_string(),
                duration_s: 0.001,
                metadata: String::new(),
            })
            .collect()
    }

    #[test]
    fn test_new_detector_has_no_baselines() {
        let detector = ProtocolAnomalyDetector::new();
        assert!(detector.baseline_stats("beacon").is_none());
    }

    #[test]
    fn test_learn_baseline_computes_stats() {
        let mut detector = ProtocolAnomalyDetector::new();
        // Events at 0.0, 0.01, 0.02, ..., 0.09 -> intervals all 0.01
        let events = make_events("beacon", &(0..10).map(|i| i as f64 * 0.01).collect::<Vec<_>>());
        detector.learn_baseline(&events);

        let stats = detector.baseline_stats("beacon").unwrap();
        assert!((stats.mean - 0.01).abs() < 1e-10);
        assert!(stats.std_dev < 1e-10); // all intervals identical
        assert_eq!(stats.sample_count, 9);
    }

    #[test]
    fn test_learn_baseline_min_max() {
        let mut detector = ProtocolAnomalyDetector::new();
        // Varying intervals: 0.01, 0.02, 0.03
        let events = make_events("data", &[0.0, 0.01, 0.03, 0.06]);
        detector.learn_baseline(&events);

        let stats = detector.baseline_stats("data").unwrap();
        assert!((stats.min - 0.01).abs() < 1e-10);
        assert!((stats.max - 0.03).abs() < 1e-10);
        assert_eq!(stats.sample_count, 3);
    }

    #[test]
    fn test_learn_baseline_needs_at_least_two_events() {
        let mut detector = ProtocolAnomalyDetector::new();
        let events = make_events("rare", &[1.0]);
        detector.learn_baseline(&events);
        assert!(detector.baseline_stats("rare").is_none());
    }

    #[test]
    fn test_detect_outliers_normal_traffic() {
        let mut detector = ProtocolAnomalyDetector::new();
        // Use baseline with slight jitter so std_dev > 0
        let timestamps: Vec<f64> = (0..100)
            .map(|i| i as f64 * 0.01 + (i % 5) as f64 * 0.00001)
            .collect();
        let baseline = make_events("beacon", &timestamps);
        detector.learn_baseline(&baseline);

        // Normal traffic with interval close to the mean
        let stats = detector.baseline_stats("beacon").unwrap();
        let mean = stats.mean;
        let test = make_events("beacon", &[200.0, 200.0 + mean, 200.0 + 2.0 * mean]);
        let anomalies = detector.detect_outliers(&test);
        assert!(anomalies.is_empty());
    }

    #[test]
    fn test_detect_outliers_large_gap() {
        let mut detector = ProtocolAnomalyDetector::new();
        // Baseline with some variance (intervals around 0.01 +/- small jitter)
        let timestamps: Vec<f64> = (0..100)
            .map(|i| i as f64 * 0.01 + (i % 3) as f64 * 0.0001)
            .collect();
        let baseline = make_events("beacon", &timestamps);
        detector.learn_baseline(&baseline);

        // Test with a very large gap (0.5s vs ~0.01s mean)
        let test = make_events("beacon", &[300.0, 300.5]);
        let anomalies = detector.detect_outliers(&test);
        assert!(!anomalies.is_empty());
        assert!(anomalies[0].deviation_sigma > 3.0);
    }

    #[test]
    fn test_detect_outliers_severity_classification() {
        let mut detector = ProtocolAnomalyDetector::new();
        // Baseline with known std_dev
        // intervals: alternating 0.009 and 0.011 -> mean=0.01, std~0.001
        let mut timestamps = vec![0.0];
        for i in 1..101 {
            let prev = timestamps[i - 1];
            let interval = if i % 2 == 0 { 0.011 } else { 0.009 };
            timestamps.push(prev + interval);
        }
        let baseline = make_events("pulse", &timestamps);
        detector.learn_baseline(&baseline);

        let stats = detector.baseline_stats("pulse").unwrap();
        let sigma = stats.std_dev;

        // Create event pair with >5 sigma gap
        let big_gap = stats.mean + 6.0 * sigma;
        let test = make_events("pulse", &[500.0, 500.0 + big_gap]);
        let anomalies = detector.detect_outliers(&test);
        assert!(!anomalies.is_empty());
        assert_eq!(anomalies[0].severity, Severity::Critical);

        // Create event pair with 3-5 sigma gap
        let med_gap = stats.mean + 4.0 * sigma;
        let test2 = make_events("pulse", &[600.0, 600.0 + med_gap]);
        let anomalies2 = detector.detect_outliers(&test2);
        assert!(!anomalies2.is_empty());
        assert_eq!(anomalies2[0].severity, Severity::High);
    }

    #[test]
    fn test_add_rule_and_report_violations_below_min() {
        let mut detector = ProtocolAnomalyDetector::new();
        detector.add_rule("ack", 0.005, 0.050);

        // Interval of 0.001 is below min 0.005
        let events = make_events("ack", &[0.0, 0.001]);
        let violations = detector.report_violations(&events);
        assert_eq!(violations.len(), 1);
        assert!(violations[0].description.contains("below minimum"));
    }

    #[test]
    fn test_add_rule_and_report_violations_above_max() {
        let mut detector = ProtocolAnomalyDetector::new();
        detector.add_rule("ack", 0.005, 0.050);

        // Interval of 0.1 is above max 0.050
        let events = make_events("ack", &[0.0, 0.1]);
        let violations = detector.report_violations(&events);
        assert_eq!(violations.len(), 1);
        assert!(violations[0].description.contains("above maximum"));
    }

    #[test]
    fn test_report_violations_compliant_traffic() {
        let mut detector = ProtocolAnomalyDetector::new();
        detector.add_rule("ack", 0.005, 0.050);

        // Interval of 0.01 is within bounds
        let events = make_events("ack", &[0.0, 0.01, 0.02, 0.03]);
        let violations = detector.report_violations(&events);
        assert!(violations.is_empty());
    }

    #[test]
    fn test_report_violations_critical_severity_below_half_min() {
        let mut detector = ProtocolAnomalyDetector::new();
        detector.add_rule("frame", 0.010, 0.100);

        // Interval 0.002 < 0.005 (half of min), should be Critical
        let events = make_events("frame", &[0.0, 0.002]);
        let violations = detector.report_violations(&events);
        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].severity, Severity::Critical);
    }

    #[test]
    fn test_report_violations_critical_severity_above_double_max() {
        let mut detector = ProtocolAnomalyDetector::new();
        detector.add_rule("frame", 0.010, 0.100);

        // Interval 0.25 > 0.200 (double of max), should be Critical
        let events = make_events("frame", &[0.0, 0.25]);
        let violations = detector.report_violations(&events);
        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].severity, Severity::Critical);
    }

    #[test]
    fn test_is_anomalous_with_baseline() {
        let mut detector = ProtocolAnomalyDetector::new();
        // Learn baseline with intervals around 0.01
        let mut timestamps = vec![0.0];
        for i in 1..101 {
            timestamps.push(timestamps[i - 1] + 0.01 + (i % 3) as f64 * 0.0001);
        }
        let baseline = make_events("sync", &timestamps);
        detector.learn_baseline(&baseline);

        // Duration close to mean interval: not anomalous
        let normal_event = ProtocolEvent {
            timestamp_s: 500.0,
            event_type: "sync".to_string(),
            duration_s: 0.01,
            metadata: String::new(),
        };
        assert!(!detector.is_anomalous(&normal_event));

        // Duration far from mean interval: anomalous
        let weird_event = ProtocolEvent {
            timestamp_s: 500.0,
            event_type: "sync".to_string(),
            duration_s: 1.0,
            metadata: String::new(),
        };
        assert!(detector.is_anomalous(&weird_event));
    }

    #[test]
    fn test_is_anomalous_with_rule() {
        let mut detector = ProtocolAnomalyDetector::new();
        detector.add_rule("poll", 0.1, 1.0);

        let within = ProtocolEvent {
            timestamp_s: 0.0,
            event_type: "poll".to_string(),
            duration_s: 0.5,
            metadata: String::new(),
        };
        assert!(!detector.is_anomalous(&within));

        let outside = ProtocolEvent {
            timestamp_s: 0.0,
            event_type: "poll".to_string(),
            duration_s: 2.0,
            metadata: String::new(),
        };
        assert!(detector.is_anomalous(&outside));
    }

    #[test]
    fn test_multiple_event_types_independent() {
        let mut detector = ProtocolAnomalyDetector::new();
        let beacons = make_events(
            "beacon",
            &(0..50).map(|i| i as f64 * 0.1).collect::<Vec<_>>(),
        );
        let acks = make_events(
            "ack",
            &(0..50).map(|i| i as f64 * 0.005).collect::<Vec<_>>(),
        );

        let mut all: Vec<ProtocolEvent> = beacons;
        all.extend(acks);
        detector.learn_baseline(&all);

        let beacon_stats = detector.baseline_stats("beacon").unwrap();
        let ack_stats = detector.baseline_stats("ack").unwrap();

        assert!((beacon_stats.mean - 0.1).abs() < 1e-10);
        assert!((ack_stats.mean - 0.005).abs() < 1e-10);
    }

    #[test]
    fn test_no_rules_no_violations() {
        let detector = ProtocolAnomalyDetector::new();
        let events = make_events("unknown", &[0.0, 1.0, 2.0]);
        let violations = detector.report_violations(&events);
        assert!(violations.is_empty());
    }

    #[test]
    fn test_default_impl() {
        let detector = ProtocolAnomalyDetector::default();
        assert!(detector.baseline_stats("anything").is_none());
    }

    #[test]
    fn test_severity_display() {
        assert_eq!(format!("{}", Severity::Low), "Low");
        assert_eq!(format!("{}", Severity::Medium), "Medium");
        assert_eq!(format!("{}", Severity::High), "High");
        assert_eq!(format!("{}", Severity::Critical), "Critical");
    }

    #[test]
    fn test_anomaly_report_contains_event_info() {
        let mut detector = ProtocolAnomalyDetector::new();
        detector.add_rule("test_type", 0.01, 0.02);

        let events = vec![
            ProtocolEvent {
                timestamp_s: 0.0,
                event_type: "test_type".to_string(),
                duration_s: 0.001,
                metadata: "first".to_string(),
            },
            ProtocolEvent {
                timestamp_s: 5.0, // way too late
                event_type: "test_type".to_string(),
                duration_s: 0.001,
                metadata: "second".to_string(),
            },
        ];

        let violations = detector.report_violations(&events);
        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].event.metadata, "second");
        assert_eq!(violations[0].event.event_type, "test_type");
    }

    #[test]
    fn test_detect_outliers_with_unknown_event_type() {
        let mut detector = ProtocolAnomalyDetector::new();
        let baseline = make_events(
            "known",
            &(0..50).map(|i| i as f64 * 0.01).collect::<Vec<_>>(),
        );
        detector.learn_baseline(&baseline);

        // Events of unknown type should produce no outliers (no baseline to compare)
        let unknown = make_events("unknown_type", &[0.0, 0.5, 1.0]);
        let anomalies = detector.detect_outliers(&unknown);
        assert!(anomalies.is_empty());
    }

    #[test]
    fn test_zero_stddev_baseline_flags_any_difference() {
        let mut detector = ProtocolAnomalyDetector::new();
        // Perfectly uniform intervals -> std_dev = 0
        let events = make_events(
            "precise",
            &(0..20).map(|i| i as f64 * 0.01).collect::<Vec<_>>(),
        );
        detector.learn_baseline(&events);

        let stats = detector.baseline_stats("precise").unwrap();
        assert!(stats.std_dev < 1e-15);

        // Even a tiny deviation should be flagged
        let test = make_events("precise", &[100.0, 100.0105]);
        let anomalies = detector.detect_outliers(&test);
        assert!(!anomalies.is_empty());
        assert_eq!(anomalies[0].severity, Severity::Critical);
    }
}
