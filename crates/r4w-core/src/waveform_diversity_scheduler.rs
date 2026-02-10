//! # Waveform Diversity Scheduler
//!
//! Schedules heterogeneous waveforms across time/frequency slots based on
//! channel conditions and application requirements. The scheduler maintains a
//! registry of [`WaveformProfile`]s, each describing a modulation scheme's
//! performance envelope (minimum SNR, throughput, latency, spectral
//! efficiency). Given a [`ChannelCondition`] snapshot and a
//! [`ScheduleRequest`], [`WaveformDiversityScheduler::schedule`] selects the
//! best-fit waveform, assigns a time/frequency slot, and returns a
//! [`ScheduleResult`].
//!
//! Feedback via [`update_metrics`](WaveformDiversityScheduler::update_metrics)
//! allows the scheduler to track actual BER and throughput per waveform,
//! enabling empirical scoring adjustments.
//!
//! # Example
//!
//! ```
//! use r4w_core::waveform_diversity_scheduler::{
//!     WaveformDiversityScheduler, WaveformProfile, ChannelCondition, ScheduleRequest,
//! };
//!
//! let mut sched = WaveformDiversityScheduler::new(10, 4); // 10 time slots, 4 freq slots
//!
//! sched.register_waveform(WaveformProfile {
//!     name: "BPSK".into(),
//!     modulation: "PSK-2".into(),
//!     min_snr_db: 3.0,
//!     max_throughput_bps: 1_000_000.0,
//!     latency_ms: 1.0,
//!     spectral_efficiency: 1.0,
//! });
//!
//! sched.register_waveform(WaveformProfile {
//!     name: "QPSK".into(),
//!     modulation: "PSK-4".into(),
//!     min_snr_db: 6.0,
//!     max_throughput_bps: 2_000_000.0,
//!     latency_ms: 1.5,
//!     spectral_efficiency: 2.0,
//! });
//!
//! let channel = ChannelCondition {
//!     snr_db: 12.0,
//!     interference_db: -20.0,
//!     doppler_hz: 50.0,
//!     multipath_spread_us: 1.0,
//! };
//!
//! let request = ScheduleRequest {
//!     min_throughput: 500_000.0,
//!     max_latency: 5.0,
//!     priority: 1,
//! };
//!
//! let result = sched.schedule(&channel, &request).expect("should schedule");
//! assert!(result.expected_throughput > 0.0);
//! ```

use std::collections::HashMap;
use std::fmt;

// ---------------------------------------------------------------------------
// Data types
// ---------------------------------------------------------------------------

/// Describes the performance envelope of one waveform / modulation scheme.
#[derive(Debug, Clone)]
pub struct WaveformProfile {
    /// Human-readable name, e.g. `"16-QAM"`.
    pub name: String,
    /// Modulation identifier, e.g. `"QAM-16"`.
    pub modulation: String,
    /// Minimum required SNR (dB) for reliable operation.
    pub min_snr_db: f64,
    /// Maximum achievable throughput (bits/s) under ideal conditions.
    pub max_throughput_bps: f64,
    /// Typical processing latency (milliseconds).
    pub latency_ms: f64,
    /// Spectral efficiency (bits/s/Hz).
    pub spectral_efficiency: f64,
}

/// A snapshot of current channel conditions.
#[derive(Debug, Clone, Copy)]
pub struct ChannelCondition {
    /// Signal-to-noise ratio (dB).
    pub snr_db: f64,
    /// Interference power relative to noise floor (dB, typically negative).
    pub interference_db: f64,
    /// Doppler spread (Hz).
    pub doppler_hz: f64,
    /// Multipath delay spread (microseconds).
    pub multipath_spread_us: f64,
}

/// Application-level requirements for a scheduling decision.
#[derive(Debug, Clone, Copy)]
pub struct ScheduleRequest {
    /// Minimum acceptable throughput (bits/s).
    pub min_throughput: f64,
    /// Maximum acceptable latency (milliseconds).
    pub max_latency: f64,
    /// Priority level (lower number = higher priority, 0 is highest).
    pub priority: u32,
}

/// The output of a scheduling decision.
#[derive(Debug, Clone)]
pub struct ScheduleResult {
    /// Name of the selected waveform.
    pub selected_waveform: String,
    /// Expected throughput (bits/s) under current conditions.
    pub expected_throughput: f64,
    /// Expected bit-error rate.
    pub expected_ber: f64,
    /// Assigned time-slot index.
    pub time_slot: usize,
    /// Assigned frequency-slot index.
    pub freq_slot: usize,
}

impl fmt::Display for ScheduleResult {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}  T={} F={}  {:.0} bps  BER={:.2e}",
            self.selected_waveform,
            self.time_slot,
            self.freq_slot,
            self.expected_throughput,
            self.expected_ber,
        )
    }
}

/// Tracked per-waveform actual performance metrics.
#[derive(Debug, Clone, Default)]
pub struct WaveformMetrics {
    /// Measured bit-error rate (exponential moving average).
    pub actual_ber: f64,
    /// Measured throughput (bits/s, exponential moving average).
    pub actual_throughput: f64,
    /// Number of times this waveform has been selected.
    pub usage_count: u64,
}

/// Scheduler-wide aggregate statistics.
#[derive(Debug, Clone, Default)]
pub struct SchedulerStatistics {
    /// Total number of scheduling decisions made.
    pub total_schedules: u64,
    /// Number of waveform switches (selected waveform differs from previous).
    pub switching_count: u64,
    /// Running average throughput across all decisions.
    pub average_throughput: f64,
    /// Per-waveform usage counts (name -> count).
    pub per_waveform_usage: HashMap<String, u64>,
}

/// Scored waveform entry returned by [`WaveformDiversityScheduler::rank_waveforms`].
#[derive(Debug, Clone)]
pub struct RankedWaveform {
    /// Waveform name.
    pub name: String,
    /// Composite score (higher is better).
    pub score: f64,
    /// Whether the waveform meets the request constraints.
    pub feasible: bool,
}

/// Per-user result in a multi-user schedule.
#[derive(Debug, Clone)]
pub struct UserSchedule {
    /// User identifier (index into the input slice).
    pub user_index: usize,
    /// The scheduling result, or `None` if no waveform could be assigned.
    pub result: Option<ScheduleResult>,
}

// ---------------------------------------------------------------------------
// Scheduler
// ---------------------------------------------------------------------------

/// Allocates waveforms to time/frequency slots.
///
/// The scheduler keeps a grid of `num_time_slots x num_freq_slots` cells.
/// Each call to [`schedule`](Self::schedule) finds the best waveform and
/// claims the next available slot.
pub struct WaveformDiversityScheduler {
    profiles: Vec<WaveformProfile>,
    metrics: HashMap<String, WaveformMetrics>,
    stats: SchedulerStatistics,
    last_selected: Option<String>,
    /// Grid dimensions.
    num_time_slots: usize,
    num_freq_slots: usize,
    /// Occupied slot bitmap (time_slot * num_freq_slots + freq_slot).
    occupied: Vec<bool>,
    /// Running throughput sum for average calculation.
    throughput_sum: f64,
}

impl WaveformDiversityScheduler {
    /// Create a new scheduler with the given slot grid dimensions.
    pub fn new(num_time_slots: usize, num_freq_slots: usize) -> Self {
        let total = num_time_slots * num_freq_slots;
        Self {
            profiles: Vec::new(),
            metrics: HashMap::new(),
            stats: SchedulerStatistics::default(),
            last_selected: None,
            num_time_slots,
            num_freq_slots,
            occupied: vec![false; total],
            throughput_sum: 0.0,
        }
    }

    /// Register a waveform profile. Duplicate names overwrite the previous entry.
    pub fn register_waveform(&mut self, profile: WaveformProfile) {
        // Remove existing entry with the same name if present.
        self.profiles.retain(|p| p.name != profile.name);
        self.metrics
            .entry(profile.name.clone())
            .or_insert_with(WaveformMetrics::default);
        self.profiles.push(profile);
    }

    /// Number of registered waveform profiles.
    pub fn waveform_count(&self) -> usize {
        self.profiles.len()
    }

    /// Retrieve a snapshot of the scheduler statistics.
    pub fn statistics(&self) -> &SchedulerStatistics {
        &self.stats
    }

    /// Retrieve metrics for a specific waveform by name.
    pub fn metrics_for(&self, name: &str) -> Option<&WaveformMetrics> {
        self.metrics.get(name)
    }

    /// Reset the slot grid so all slots are available again.
    pub fn reset_slots(&mut self) {
        self.occupied.fill(false);
    }

    // -- scoring helpers ---------------------------------------------------

    /// Estimate effective SNR considering interference and Doppler degradation.
    fn effective_snr(channel: &ChannelCondition) -> f64 {
        // Simple model: effective_snr = snr - max(0, interference) - doppler_penalty
        let interference_penalty = channel.interference_db.max(0.0);
        let doppler_penalty = (channel.doppler_hz.abs() / 1000.0).min(6.0);
        channel.snr_db - interference_penalty - doppler_penalty
    }

    /// Estimate BER from SNR margin using a simplified exponential model:
    /// BER ~ 0.5 * exp(-0.5 * margin).
    fn estimate_ber(snr_margin_db: f64) -> f64 {
        if snr_margin_db <= 0.0 {
            0.5
        } else {
            (0.5 * (-0.5 * snr_margin_db).exp()).min(0.5)
        }
    }

    /// Estimate achievable throughput given SNR margin and profile.
    fn estimate_throughput(profile: &WaveformProfile, snr_margin_db: f64) -> f64 {
        if snr_margin_db <= 0.0 {
            return 0.0;
        }
        // Throughput ramps from 0 to max as margin grows.
        let fraction = (snr_margin_db / 20.0).min(1.0);
        profile.max_throughput_bps * fraction
    }

    /// Score a single waveform for the given conditions and request.
    /// Returns `(score, feasible, expected_throughput, expected_ber)`.
    fn score_waveform(
        &self,
        profile: &WaveformProfile,
        channel: &ChannelCondition,
        request: &ScheduleRequest,
    ) -> (f64, bool, f64, f64) {
        let eff_snr = Self::effective_snr(channel);
        let margin = eff_snr - profile.min_snr_db;
        let throughput = Self::estimate_throughput(profile, margin);
        let ber = Self::estimate_ber(margin);

        let meets_throughput = throughput >= request.min_throughput;
        let meets_latency = profile.latency_ms <= request.max_latency;
        let feasible = meets_throughput && meets_latency && margin > 0.0;

        // Composite score: throughput weight + spectral efficiency + latency bonus + feedback bonus.
        let mut score = 0.0;
        if margin > 0.0 {
            // Throughput component (normalised).
            score += throughput / 1_000_000.0;
            // Spectral efficiency bonus.
            score += profile.spectral_efficiency * 0.5;
            // Latency bonus: prefer lower latency.
            score += (10.0 - profile.latency_ms).max(0.0) * 0.1;
            // Feedback adjustment: reward waveforms with good actual BER.
            if let Some(m) = self.metrics.get(&profile.name) {
                if m.usage_count > 0 && m.actual_ber < 1e-3 {
                    score += 0.5;
                }
            }
        }

        (score, feasible, throughput, ber)
    }

    /// Rank all registered waveforms for the given conditions and request.
    /// Results are sorted descending by score.
    pub fn rank_waveforms(
        &self,
        channel: &ChannelCondition,
        request: &ScheduleRequest,
    ) -> Vec<RankedWaveform> {
        let mut ranked: Vec<RankedWaveform> = self
            .profiles
            .iter()
            .map(|p| {
                let (score, feasible, _, _) = self.score_waveform(p, channel, request);
                RankedWaveform {
                    name: p.name.clone(),
                    score,
                    feasible,
                }
            })
            .collect();
        ranked.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap_or(std::cmp::Ordering::Equal));
        ranked
    }

    /// Find the next unoccupied slot. Returns `(time_slot, freq_slot)`.
    fn next_free_slot(&self) -> Option<(usize, usize)> {
        for (idx, &occ) in self.occupied.iter().enumerate() {
            if !occ {
                let t = idx / self.num_freq_slots;
                let f = idx % self.num_freq_slots;
                return Some((t, f));
            }
        }
        None
    }

    /// Schedule a single waveform for the given channel conditions and request.
    ///
    /// Returns `None` if no waveform is feasible or no slot is available.
    pub fn schedule(
        &mut self,
        channel: &ChannelCondition,
        request: &ScheduleRequest,
    ) -> Option<ScheduleResult> {
        let (time_slot, freq_slot) = self.next_free_slot()?;

        // Score all profiles and pick the best feasible one.
        let mut best: Option<(f64, usize, f64, f64)> = None; // (score, idx, throughput, ber)
        for (i, profile) in self.profiles.iter().enumerate() {
            let (score, feasible, throughput, ber) = self.score_waveform(profile, channel, request);
            if !feasible {
                continue;
            }
            if best.is_none() || score > best.unwrap().0 {
                best = Some((score, i, throughput, ber));
            }
        }

        let (_, idx, throughput, ber) = best?;

        let name = self.profiles[idx].name.clone();

        // Claim slot.
        self.occupied[time_slot * self.num_freq_slots + freq_slot] = true;

        // Update statistics.
        self.stats.total_schedules += 1;
        if let Some(ref last) = self.last_selected {
            if *last != name {
                self.stats.switching_count += 1;
            }
        }
        self.last_selected = Some(name.clone());
        *self.stats.per_waveform_usage.entry(name.clone()).or_insert(0) += 1;
        self.throughput_sum += throughput;
        self.stats.average_throughput = self.throughput_sum / self.stats.total_schedules as f64;

        // Update per-waveform usage count.
        if let Some(m) = self.metrics.get_mut(&name) {
            m.usage_count += 1;
        }

        Some(ScheduleResult {
            selected_waveform: name,
            expected_throughput: throughput,
            expected_ber: ber,
            time_slot,
            freq_slot,
        })
    }

    /// Feed back actual performance measurements for a waveform.
    ///
    /// Uses an exponential moving average with `alpha = 0.3`.
    pub fn update_metrics(&mut self, waveform_name: &str, actual_ber: f64, actual_throughput: f64) {
        let alpha = 0.3;
        let entry = self
            .metrics
            .entry(waveform_name.to_string())
            .or_insert_with(WaveformMetrics::default);
        if entry.usage_count == 0 {
            entry.actual_ber = actual_ber;
            entry.actual_throughput = actual_throughput;
        } else {
            entry.actual_ber = alpha * actual_ber + (1.0 - alpha) * entry.actual_ber;
            entry.actual_throughput =
                alpha * actual_throughput + (1.0 - alpha) * entry.actual_throughput;
        }
    }

    /// Schedule waveforms for multiple users simultaneously.
    ///
    /// Users are served in priority order (lower `priority` value first).
    /// Each user receives the best feasible waveform and a unique slot.
    pub fn multi_user_schedule(
        &mut self,
        users: &[(ChannelCondition, ScheduleRequest)],
    ) -> Vec<UserSchedule> {
        // Build index list sorted by priority (ascending).
        let mut indices: Vec<usize> = (0..users.len()).collect();
        indices.sort_by_key(|&i| users[i].1.priority);

        let mut results = vec![
            UserSchedule {
                user_index: 0,
                result: None,
            };
            users.len()
        ];

        for &i in &indices {
            let (ref ch, ref req) = users[i];
            let res = self.schedule(ch, req);
            results[i] = UserSchedule {
                user_index: i,
                result: res,
            };
        }
        results
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: build a scheduler with three common waveforms.
    fn make_scheduler() -> WaveformDiversityScheduler {
        let mut s = WaveformDiversityScheduler::new(10, 4);
        s.register_waveform(WaveformProfile {
            name: "BPSK".into(),
            modulation: "PSK-2".into(),
            min_snr_db: 3.0,
            max_throughput_bps: 1_000_000.0,
            latency_ms: 1.0,
            spectral_efficiency: 1.0,
        });
        s.register_waveform(WaveformProfile {
            name: "QPSK".into(),
            modulation: "PSK-4".into(),
            min_snr_db: 6.0,
            max_throughput_bps: 2_000_000.0,
            latency_ms: 1.5,
            spectral_efficiency: 2.0,
        });
        s.register_waveform(WaveformProfile {
            name: "16-QAM".into(),
            modulation: "QAM-16".into(),
            min_snr_db: 12.0,
            max_throughput_bps: 4_000_000.0,
            latency_ms: 2.0,
            spectral_efficiency: 4.0,
        });
        s
    }

    fn good_channel() -> ChannelCondition {
        ChannelCondition {
            snr_db: 20.0,
            interference_db: -30.0,
            doppler_hz: 10.0,
            multipath_spread_us: 0.5,
        }
    }

    fn poor_channel() -> ChannelCondition {
        ChannelCondition {
            snr_db: 4.0,
            interference_db: -10.0,
            doppler_hz: 5.0,
            multipath_spread_us: 0.2,
        }
    }

    fn default_request() -> ScheduleRequest {
        ScheduleRequest {
            min_throughput: 10_000.0,
            max_latency: 10.0,
            priority: 1,
        }
    }

    // -- Test 1 --
    #[test]
    fn test_register_and_count() {
        let mut s = WaveformDiversityScheduler::new(4, 2);
        assert_eq!(s.waveform_count(), 0);
        s.register_waveform(WaveformProfile {
            name: "BPSK".into(),
            modulation: "PSK-2".into(),
            min_snr_db: 3.0,
            max_throughput_bps: 1e6,
            latency_ms: 1.0,
            spectral_efficiency: 1.0,
        });
        assert_eq!(s.waveform_count(), 1);
        // Re-register with same name replaces.
        s.register_waveform(WaveformProfile {
            name: "BPSK".into(),
            modulation: "PSK-2".into(),
            min_snr_db: 4.0,
            max_throughput_bps: 1.2e6,
            latency_ms: 1.0,
            spectral_efficiency: 1.0,
        });
        assert_eq!(s.waveform_count(), 1);
    }

    // -- Test 2 --
    #[test]
    fn test_schedule_good_channel_selects_high_order() {
        let mut s = make_scheduler();
        let res = s.schedule(&good_channel(), &default_request()).unwrap();
        // With 20 dB SNR, 16-QAM should be feasible and preferred (higher throughput).
        assert_eq!(res.selected_waveform, "16-QAM");
        assert!(res.expected_throughput > 0.0);
    }

    // -- Test 3 --
    #[test]
    fn test_schedule_poor_channel_selects_robust() {
        let mut s = make_scheduler();
        let res = s.schedule(&poor_channel(), &default_request()).unwrap();
        // With 4 dB SNR, only BPSK (min 3 dB) is feasible.
        assert_eq!(res.selected_waveform, "BPSK");
    }

    // -- Test 4 --
    #[test]
    fn test_schedule_returns_none_when_no_feasible() {
        let mut s = make_scheduler();
        let terrible = ChannelCondition {
            snr_db: -5.0,
            interference_db: 0.0,
            doppler_hz: 0.0,
            multipath_spread_us: 0.0,
        };
        let res = s.schedule(&terrible, &default_request());
        assert!(res.is_none());
    }

    // -- Test 5 --
    #[test]
    fn test_slot_allocation_and_exhaustion() {
        let mut s = WaveformDiversityScheduler::new(2, 2); // 4 total slots
        s.register_waveform(WaveformProfile {
            name: "BPSK".into(),
            modulation: "PSK-2".into(),
            min_snr_db: 3.0,
            max_throughput_bps: 1e6,
            latency_ms: 1.0,
            spectral_efficiency: 1.0,
        });
        let ch = good_channel();
        let req = default_request();

        // Fill all 4 slots.
        for i in 0..4 {
            let res = s.schedule(&ch, &req);
            assert!(res.is_some(), "slot {i} should succeed");
        }
        // 5th should fail.
        assert!(s.schedule(&ch, &req).is_none());
    }

    // -- Test 6 --
    #[test]
    fn test_reset_slots() {
        let mut s = WaveformDiversityScheduler::new(1, 1);
        s.register_waveform(WaveformProfile {
            name: "BPSK".into(),
            modulation: "PSK-2".into(),
            min_snr_db: 3.0,
            max_throughput_bps: 1e6,
            latency_ms: 1.0,
            spectral_efficiency: 1.0,
        });
        let ch = good_channel();
        let req = default_request();
        assert!(s.schedule(&ch, &req).is_some());
        assert!(s.schedule(&ch, &req).is_none());
        s.reset_slots();
        assert!(s.schedule(&ch, &req).is_some());
    }

    // -- Test 7 --
    #[test]
    fn test_rank_waveforms_order() {
        let s = make_scheduler();
        let ranked = s.rank_waveforms(&good_channel(), &default_request());
        assert_eq!(ranked.len(), 3);
        // Should be sorted descending by score; highest first.
        assert!(ranked[0].score >= ranked[1].score);
        assert!(ranked[1].score >= ranked[2].score);
        // All should be feasible at 20 dB SNR.
        assert!(ranked.iter().all(|r| r.feasible));
    }

    // -- Test 8 --
    #[test]
    fn test_update_metrics_ema() {
        let mut s = make_scheduler();
        // Prime usage count so EMA kicks in.
        let ch = good_channel();
        let req = default_request();
        let _ = s.schedule(&ch, &req);

        // Feed back metrics.
        s.update_metrics("16-QAM", 1e-4, 3.5e6);
        let m = s.metrics_for("16-QAM").unwrap();
        let first_ber = m.actual_ber;
        assert!(first_ber > 0.0);
        assert!(m.actual_throughput > 0.0);
        drop(m);

        // Second update should blend.
        s.update_metrics("16-QAM", 1e-5, 3.8e6);
        let m2 = s.metrics_for("16-QAM").unwrap();
        assert!(m2.actual_ber < first_ber, "BER should decrease after better sample");
    }

    // -- Test 9 --
    #[test]
    fn test_statistics_tracking() {
        let mut s = make_scheduler();
        let ch = good_channel();
        let req = default_request();

        let _ = s.schedule(&ch, &req); // 16-QAM
        let _ = s.schedule(&poor_channel(), &default_request()); // BPSK => switch

        let stats = s.statistics();
        assert_eq!(stats.total_schedules, 2);
        assert!(stats.switching_count >= 1);
        assert!(stats.average_throughput > 0.0);
        assert!(!stats.per_waveform_usage.is_empty());
    }

    // -- Test 10 --
    #[test]
    fn test_multi_user_schedule() {
        let mut s = make_scheduler();
        let users = vec![
            (good_channel(), ScheduleRequest { min_throughput: 100_000.0, max_latency: 10.0, priority: 2 }),
            (poor_channel(), ScheduleRequest { min_throughput: 5_000.0, max_latency: 10.0, priority: 0 }),
            (good_channel(), ScheduleRequest { min_throughput: 500_000.0, max_latency: 10.0, priority: 1 }),
        ];
        let results = s.multi_user_schedule(&users);
        assert_eq!(results.len(), 3);
        // All three should get a slot (we have 40 slots).
        for us in &results {
            assert!(us.result.is_some(), "user {} should be scheduled", us.user_index);
        }
        // Verify each user got a unique slot.
        let slots: Vec<(usize, usize)> = results
            .iter()
            .filter_map(|u| u.result.as_ref().map(|r| (r.time_slot, r.freq_slot)))
            .collect();
        for i in 0..slots.len() {
            for j in (i + 1)..slots.len() {
                assert_ne!(slots[i], slots[j], "slots must be unique");
            }
        }
    }

    // -- Test 11 --
    #[test]
    fn test_latency_constraint_filters_slow_waveform() {
        let mut s = WaveformDiversityScheduler::new(4, 4);
        s.register_waveform(WaveformProfile {
            name: "SlowWF".into(),
            modulation: "OFDM".into(),
            min_snr_db: 5.0,
            max_throughput_bps: 10e6,
            latency_ms: 50.0,
            spectral_efficiency: 6.0,
        });
        s.register_waveform(WaveformProfile {
            name: "FastWF".into(),
            modulation: "PSK-2".into(),
            min_snr_db: 3.0,
            max_throughput_bps: 1e6,
            latency_ms: 0.5,
            spectral_efficiency: 1.0,
        });
        let ch = good_channel();
        let req = ScheduleRequest {
            min_throughput: 100_000.0,
            max_latency: 5.0, // Excludes SlowWF
            priority: 0,
        };
        let res = s.schedule(&ch, &req).unwrap();
        assert_eq!(res.selected_waveform, "FastWF");
    }

    // -- Test 12 --
    #[test]
    fn test_schedule_result_display() {
        let r = ScheduleResult {
            selected_waveform: "QPSK".into(),
            expected_throughput: 1_500_000.0,
            expected_ber: 1e-5,
            time_slot: 3,
            freq_slot: 1,
        };
        let s = format!("{r}");
        assert!(s.contains("QPSK"));
        assert!(s.contains("T=3"));
        assert!(s.contains("F=1"));
    }

    // -- Test 13 --
    #[test]
    fn test_effective_snr_degradation() {
        let clean = ChannelCondition { snr_db: 20.0, interference_db: -30.0, doppler_hz: 0.0, multipath_spread_us: 0.0 };
        let noisy = ChannelCondition { snr_db: 20.0, interference_db: 5.0, doppler_hz: 3000.0, multipath_spread_us: 5.0 };
        let eff_clean = WaveformDiversityScheduler::effective_snr(&clean);
        let eff_noisy = WaveformDiversityScheduler::effective_snr(&noisy);
        assert!(eff_clean > eff_noisy, "interference and Doppler should degrade effective SNR");
    }
}
