//! Multi-carrier resource allocation for LTE/5G carrier aggregation.
//!
//! This module implements a carrier aggregation scheduler that manages
//! resource allocation across multiple component carriers. It supports
//! various scheduling policies (Round Robin, Max Throughput, Proportional Fair,
//! Load Balance) and handles cross-carrier scheduling with PCC/SCC coordination.
//!
//! # Example
//!
//! ```
//! use r4w_core::carrier_aggregation_scheduler::{
//!     CarrierAggregationScheduler, ComponentCarrier, UserEquipment,
//!     SchedulingPolicy,
//! };
//!
//! // Create two component carriers (PCC + SCC)
//! let cc0 = ComponentCarrier {
//!     id: 0,
//!     center_freq_hz: 2.14e9,
//!     bandwidth_hz: 20e6,
//!     num_prbs: 100,
//!     mcs_index: 15,
//!     cqi: 10,
//!     is_primary: true,
//! };
//! let cc1 = ComponentCarrier {
//!     id: 1,
//!     center_freq_hz: 2.66e9,
//!     bandwidth_hz: 10e6,
//!     num_prbs: 50,
//!     mcs_index: 12,
//!     cqi: 8,
//!     is_primary: false,
//! };
//!
//! let mut scheduler = CarrierAggregationScheduler::new(vec![cc0, cc1]);
//!
//! // Add a user equipment with demand and channel quality per carrier
//! let ue = UserEquipment {
//!     id: 0,
//!     demand_bps: 50e6,
//!     supported_carriers: vec![0, 1],
//!     channel_quality: vec![(0, 10), (1, 8)],
//! };
//! scheduler.add_ue(ue);
//! scheduler.set_policy(SchedulingPolicy::ProportionalFair);
//!
//! let result = scheduler.allocate();
//! assert!(result.throughput_bps > 0.0);
//! assert!(result.fairness_index >= 0.0 && result.fairness_index <= 1.0);
//! ```

use std::collections::HashMap;

/// A component carrier in the aggregated set.
#[derive(Debug, Clone)]
pub struct ComponentCarrier {
    /// Carrier identifier.
    pub id: usize,
    /// Center frequency in Hz.
    pub center_freq_hz: f64,
    /// Bandwidth in Hz.
    pub bandwidth_hz: f64,
    /// Number of Physical Resource Blocks available.
    pub num_prbs: usize,
    /// Current Modulation and Coding Scheme index (0-28).
    pub mcs_index: u8,
    /// Channel Quality Indicator (1-15).
    pub cqi: u8,
    /// Whether this is the Primary Component Carrier (PCC).
    pub is_primary: bool,
}

/// Scheduling policy for resource allocation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedulingPolicy {
    /// Each UE gets an equal share of PRBs in rotation.
    RoundRobin,
    /// Allocate PRBs to UEs with the best channel quality first.
    MaxThroughput,
    /// Balance between throughput and fairness using PF metric.
    ProportionalFair,
    /// Distribute load evenly across carriers.
    LoadBalance,
}

/// A user equipment requesting resources.
#[derive(Debug, Clone)]
pub struct UserEquipment {
    /// UE identifier.
    pub id: usize,
    /// Requested data rate in bits per second.
    pub demand_bps: f64,
    /// List of carrier IDs this UE can use.
    pub supported_carriers: Vec<usize>,
    /// Per-carrier channel quality as (carrier_id, cqi) pairs.
    pub channel_quality: Vec<(usize, u8)>,
}

/// Per-carrier allocation details.
#[derive(Debug, Clone)]
pub struct CarrierAllocation {
    /// Carrier ID.
    pub carrier_id: usize,
    /// Map from UE id to number of PRBs allocated on this carrier.
    pub ue_prb_allocations: HashMap<usize, usize>,
    /// Total PRBs allocated on this carrier.
    pub total_prbs_used: usize,
    /// Throughput on this carrier in bps.
    pub throughput_bps: f64,
}

/// Result of a scheduling allocation round.
#[derive(Debug, Clone)]
pub struct AllocationResult {
    /// Per-carrier allocation details.
    pub carrier_allocations: Vec<CarrierAllocation>,
    /// Total aggregate throughput across all carriers in bps.
    pub throughput_bps: f64,
    /// Jain's fairness index across UEs (0.0 to 1.0).
    pub fairness_index: f64,
}

/// CQI-to-spectral-efficiency mapping table (3GPP TS 36.213 Table 7.2.3-1 approx).
/// Index 0 is unused; CQI values 1..15 map to spectral efficiency in bps/Hz.
const CQI_TO_EFFICIENCY: [f64; 16] = [
    0.0,    // CQI 0: out of range
    0.1523, // CQI 1
    0.2344, // CQI 2
    0.3770, // CQI 3
    0.6016, // CQI 4
    0.8770, // CQI 5
    1.1758, // CQI 6
    1.4766, // CQI 7
    1.9141, // CQI 8
    2.4063, // CQI 9
    2.7305, // CQI 10
    3.3223, // CQI 11
    3.9023, // CQI 12
    4.5234, // CQI 13
    5.1152, // CQI 14
    5.5547, // CQI 15
];

/// Bandwidth of a single PRB in Hz (LTE: 180 kHz = 12 subcarriers x 15 kHz).
const PRB_BANDWIDTH_HZ: f64 = 180_000.0;

/// Map CQI to a recommended MCS index (simplified).
fn cqi_to_mcs(cqi: u8) -> u8 {
    let cqi_clamped = cqi.clamp(1, 15);
    // Roughly: MCS ~ 2*(CQI-1), capped at 28
    ((cqi_clamped as u16 - 1) * 2).min(28) as u8
}

/// Compute spectral efficiency for a given CQI value.
fn cqi_spectral_efficiency(cqi: u8) -> f64 {
    let idx = cqi.clamp(0, 15) as usize;
    CQI_TO_EFFICIENCY[idx]
}

/// Compute throughput for a number of PRBs at a given CQI.
fn prb_throughput(num_prbs: usize, cqi: u8) -> f64 {
    let eff = cqi_spectral_efficiency(cqi);
    // Each PRB is 180 kHz wide.
    // Throughput = num_prbs * efficiency * PRB_bandwidth
    num_prbs as f64 * eff * PRB_BANDWIDTH_HZ
}

/// Compute Jain's fairness index for a set of throughput values.
///
/// J(x1,...,xn) = (sum(xi))^2 / (n * sum(xi^2))
/// Returns 1.0 for perfectly fair allocation, 1/n for maximally unfair.
fn jains_fairness_index(values: &[f64]) -> f64 {
    if values.is_empty() {
        return 1.0;
    }
    let n = values.len() as f64;
    let sum: f64 = values.iter().sum();
    let sum_sq: f64 = values.iter().map(|x| x * x).sum();
    if sum_sq == 0.0 {
        return 1.0;
    }
    (sum * sum) / (n * sum_sq)
}

/// Multi-carrier resource allocation scheduler for LTE/5G carrier aggregation.
///
/// Manages 2-5 component carriers and allocates Physical Resource Blocks (PRBs)
/// to user equipment based on the configured scheduling policy. Supports
/// cross-carrier scheduling with Primary Component Carrier (PCC) and
/// Secondary Component Carrier (SCC) coordination.
pub struct CarrierAggregationScheduler {
    carriers: Vec<ComponentCarrier>,
    ues: Vec<UserEquipment>,
    policy: SchedulingPolicy,
    /// Tracks average throughput per UE for proportional fair scheduling.
    avg_throughput: HashMap<usize, f64>,
    /// Exponential moving average factor for PF scheduler.
    pf_alpha: f64,
}

impl CarrierAggregationScheduler {
    /// Create a new scheduler with the given component carriers.
    ///
    /// # Panics
    ///
    /// Panics if `carriers` is empty or has more than 5 entries.
    pub fn new(carriers: Vec<ComponentCarrier>) -> Self {
        assert!(
            !carriers.is_empty() && carriers.len() <= 5,
            "Carrier aggregation supports 1-5 component carriers (got {})",
            carriers.len()
        );
        Self {
            carriers,
            ues: Vec::new(),
            policy: SchedulingPolicy::RoundRobin,
            avg_throughput: HashMap::new(),
            pf_alpha: 0.001,
        }
    }

    /// Add a user equipment to the scheduler.
    pub fn add_ue(&mut self, ue: UserEquipment) {
        self.avg_throughput.insert(ue.id, 1.0); // Initialize with small value to avoid div-by-zero
        self.ues.push(ue);
    }

    /// Set the scheduling policy.
    pub fn set_policy(&mut self, policy: SchedulingPolicy) {
        self.policy = policy;
    }

    /// Run the allocation algorithm and return the result.
    pub fn allocate(&mut self) -> AllocationResult {
        match self.policy {
            SchedulingPolicy::RoundRobin => self.allocate_round_robin(),
            SchedulingPolicy::MaxThroughput => self.allocate_max_throughput(),
            SchedulingPolicy::ProportionalFair => self.allocate_proportional_fair(),
            SchedulingPolicy::LoadBalance => self.allocate_load_balance(),
        }
    }

    /// Balance load across carriers by redistributing PRB allocations.
    ///
    /// This adjusts the carrier MCS index based on current demand distribution
    /// and attempts to increase throughput on overloaded carriers.
    pub fn balance_load(&mut self) {
        if self.ues.is_empty() || self.carriers.len() < 2 {
            return;
        }

        // Compute per-carrier demand
        let mut carrier_demand: HashMap<usize, f64> = HashMap::new();
        for carrier in &self.carriers {
            carrier_demand.insert(carrier.id, 0.0);
        }

        for ue in &self.ues {
            let num_supported = ue.supported_carriers.len().max(1) as f64;
            let per_carrier_demand = ue.demand_bps / num_supported;
            for &cid in &ue.supported_carriers {
                if let Some(d) = carrier_demand.get_mut(&cid) {
                    *d += per_carrier_demand;
                }
            }
        }

        // Compute average demand
        let total_demand: f64 = carrier_demand.values().sum();
        let avg_demand = total_demand / self.carriers.len() as f64;

        // Adjust MCS indices: overloaded carriers get higher MCS to increase per-PRB throughput
        for carrier in &mut self.carriers {
            let demand = carrier_demand.get(&carrier.id).copied().unwrap_or(0.0);
            if demand > avg_demand * 1.2 {
                carrier.mcs_index = (carrier.mcs_index + 1).min(28);
            } else if demand < avg_demand * 0.8 && carrier.mcs_index > 0 {
                carrier.mcs_index = carrier.mcs_index.saturating_sub(1);
            }
        }
    }

    /// Total theoretical capacity across all carriers in bps.
    pub fn total_capacity_bps(&self) -> f64 {
        self.carriers
            .iter()
            .map(|cc| prb_throughput(cc.num_prbs, cc.cqi))
            .sum()
    }

    /// Utilization of a specific carrier (0.0 to 1.0).
    ///
    /// Computed as the fraction of capacity consumed by UE demand.
    /// Returns 0.0 if no UEs are registered or the carrier is not found.
    pub fn carrier_utilization(&self, carrier_id: usize) -> f64 {
        let carrier = match self.carriers.iter().find(|c| c.id == carrier_id) {
            Some(c) => c,
            None => return 0.0,
        };
        if self.ues.is_empty() || carrier.num_prbs == 0 {
            return 0.0;
        }

        let ue_count = self
            .ues
            .iter()
            .filter(|ue| ue.supported_carriers.contains(&carrier_id))
            .count();

        if ue_count == 0 {
            return 0.0;
        }

        let capacity = prb_throughput(carrier.num_prbs, carrier.cqi);
        if capacity == 0.0 {
            return 0.0;
        }

        let demand: f64 = self
            .ues
            .iter()
            .filter(|ue| ue.supported_carriers.contains(&carrier_id))
            .map(|ue| {
                ue.demand_bps / ue.supported_carriers.len().max(1) as f64
            })
            .sum();

        (demand / capacity).min(1.0)
    }

    /// Return a reference to the component carriers.
    pub fn carriers(&self) -> &[ComponentCarrier] {
        &self.carriers
    }

    /// Return a reference to the registered UEs.
    pub fn ues(&self) -> &[UserEquipment] {
        &self.ues
    }

    /// Get the primary component carrier.
    pub fn primary_carrier(&self) -> Option<&ComponentCarrier> {
        self.carriers.iter().find(|c| c.is_primary)
    }

    /// Get secondary component carriers.
    pub fn secondary_carriers(&self) -> Vec<&ComponentCarrier> {
        self.carriers.iter().filter(|c| !c.is_primary).collect()
    }

    // ---- Private scheduling implementations ----

    /// Round-robin: divide PRBs equally among UEs on each carrier.
    fn allocate_round_robin(&mut self) -> AllocationResult {
        let mut carrier_allocs = Vec::new();
        let mut ue_throughputs: HashMap<usize, f64> = HashMap::new();

        for carrier in &self.carriers {
            let eligible_ues: Vec<&UserEquipment> = self
                .ues
                .iter()
                .filter(|ue| ue.supported_carriers.contains(&carrier.id))
                .collect();

            let mut ue_prb_allocations = HashMap::new();
            let mut total_prbs_used = 0;
            let mut carrier_tp = 0.0;

            if !eligible_ues.is_empty() {
                let prbs_per_ue = carrier.num_prbs / eligible_ues.len();
                let remainder = carrier.num_prbs % eligible_ues.len();

                for (i, ue) in eligible_ues.iter().enumerate() {
                    let prbs = prbs_per_ue + if i < remainder { 1 } else { 0 };
                    let cqi = ue
                        .channel_quality
                        .iter()
                        .find(|(cid, _)| *cid == carrier.id)
                        .map(|(_, q)| *q)
                        .unwrap_or(carrier.cqi);
                    let tp = prb_throughput(prbs, cqi);
                    ue_prb_allocations.insert(ue.id, prbs);
                    total_prbs_used += prbs;
                    carrier_tp += tp;
                    *ue_throughputs.entry(ue.id).or_insert(0.0) += tp;
                }
            }

            carrier_allocs.push(CarrierAllocation {
                carrier_id: carrier.id,
                ue_prb_allocations,
                total_prbs_used,
                throughput_bps: carrier_tp,
            });
        }

        let total_tp: f64 = carrier_allocs.iter().map(|a| a.throughput_bps).sum();
        let tp_values: Vec<f64> = ue_throughputs.values().copied().collect();
        let fairness = jains_fairness_index(&tp_values);

        self.update_avg_throughput(&ue_throughputs);

        AllocationResult {
            carrier_allocations: carrier_allocs,
            throughput_bps: total_tp,
            fairness_index: fairness,
        }
    }

    /// Max throughput: assign PRBs to UEs with the best channel quality first.
    fn allocate_max_throughput(&mut self) -> AllocationResult {
        let mut carrier_allocs = Vec::new();
        let mut ue_throughputs: HashMap<usize, f64> = HashMap::new();

        for carrier in &self.carriers {
            let mut eligible: Vec<(usize, u8, f64)> = self
                .ues
                .iter()
                .filter(|ue| ue.supported_carriers.contains(&carrier.id))
                .map(|ue| {
                    let cqi = ue
                        .channel_quality
                        .iter()
                        .find(|(cid, _)| *cid == carrier.id)
                        .map(|(_, q)| *q)
                        .unwrap_or(1);
                    (ue.id, cqi, ue.demand_bps)
                })
                .collect();

            // Sort by CQI descending (best channel first)
            eligible.sort_by(|a, b| b.1.cmp(&a.1));

            let mut ue_prb_allocations = HashMap::new();
            let mut remaining_prbs = carrier.num_prbs;
            let mut carrier_tp = 0.0;

            for (ue_id, cqi, demand) in &eligible {
                if remaining_prbs == 0 {
                    break;
                }
                let eff = cqi_spectral_efficiency(*cqi);
                let needed_prbs = if eff > 0.0 {
                    let tp_per_prb = eff * PRB_BANDWIDTH_HZ;
                    let already = ue_throughputs.get(ue_id).copied().unwrap_or(0.0);
                    let still_needed = (demand - already).max(0.0);
                    (still_needed / tp_per_prb).ceil() as usize
                } else {
                    remaining_prbs
                };
                let alloc = needed_prbs.min(remaining_prbs);
                let tp = prb_throughput(alloc, *cqi);
                ue_prb_allocations.insert(*ue_id, alloc);
                remaining_prbs -= alloc;
                carrier_tp += tp;
                *ue_throughputs.entry(*ue_id).or_insert(0.0) += tp;
            }

            let total_prbs_used = carrier.num_prbs - remaining_prbs;
            carrier_allocs.push(CarrierAllocation {
                carrier_id: carrier.id,
                ue_prb_allocations,
                total_prbs_used,
                throughput_bps: carrier_tp,
            });
        }

        let total_tp: f64 = carrier_allocs.iter().map(|a| a.throughput_bps).sum();
        let tp_values: Vec<f64> = ue_throughputs.values().copied().collect();
        let fairness = jains_fairness_index(&tp_values);

        self.update_avg_throughput(&ue_throughputs);

        AllocationResult {
            carrier_allocations: carrier_allocs,
            throughput_bps: total_tp,
            fairness_index: fairness,
        }
    }

    /// Proportional Fair: balance between throughput and fairness using PF metric.
    ///
    /// PF metric for UE i on carrier c: CQI_efficiency(i,c) / avg_throughput(i)
    fn allocate_proportional_fair(&mut self) -> AllocationResult {
        let mut carrier_allocs = Vec::new();
        let mut ue_throughputs: HashMap<usize, f64> = HashMap::new();

        for carrier in &self.carriers {
            let mut pf_scores: Vec<(usize, f64, u8)> = self
                .ues
                .iter()
                .filter(|ue| ue.supported_carriers.contains(&carrier.id))
                .map(|ue| {
                    let cqi = ue
                        .channel_quality
                        .iter()
                        .find(|(cid, _)| *cid == carrier.id)
                        .map(|(_, q)| *q)
                        .unwrap_or(1);
                    let eff = cqi_spectral_efficiency(cqi);
                    let avg = self.avg_throughput.get(&ue.id).copied().unwrap_or(1.0);
                    let pf = eff / avg;
                    (ue.id, pf, cqi)
                })
                .collect();

            // Sort by PF score descending
            pf_scores.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

            let mut ue_prb_allocations = HashMap::new();
            let mut remaining_prbs = carrier.num_prbs;
            let mut carrier_tp = 0.0;

            if !pf_scores.is_empty() {
                let total_pf: f64 = pf_scores.iter().map(|(_, pf, _)| pf).sum();
                if total_pf > 0.0 {
                    for (i, (ue_id, pf, cqi)) in pf_scores.iter().enumerate() {
                        let share = if i == pf_scores.len() - 1 {
                            // Last UE gets remainder
                            remaining_prbs
                        } else {
                            let frac = pf / total_pf;
                            let prbs = (frac * carrier.num_prbs as f64).round() as usize;
                            prbs.min(remaining_prbs)
                        };
                        let tp = prb_throughput(share, *cqi);
                        ue_prb_allocations.insert(*ue_id, share);
                        remaining_prbs = remaining_prbs.saturating_sub(share);
                        carrier_tp += tp;
                        *ue_throughputs.entry(*ue_id).or_insert(0.0) += tp;
                    }
                }
            }

            let total_prbs_used = carrier.num_prbs - remaining_prbs;
            carrier_allocs.push(CarrierAllocation {
                carrier_id: carrier.id,
                ue_prb_allocations,
                total_prbs_used,
                throughput_bps: carrier_tp,
            });
        }

        let total_tp: f64 = carrier_allocs.iter().map(|a| a.throughput_bps).sum();
        let tp_values: Vec<f64> = ue_throughputs.values().copied().collect();
        let fairness = jains_fairness_index(&tp_values);

        self.update_avg_throughput(&ue_throughputs);

        AllocationResult {
            carrier_allocations: carrier_allocs,
            throughput_bps: total_tp,
            fairness_index: fairness,
        }
    }

    /// Load balance: distribute UEs across carriers to even out utilization.
    fn allocate_load_balance(&mut self) -> AllocationResult {
        let mut carrier_allocs = Vec::new();
        let mut ue_throughputs: HashMap<usize, f64> = HashMap::new();

        // Track how many PRBs are reserved per carrier
        let mut carrier_reserved: HashMap<usize, usize> = HashMap::new();
        for carrier in &self.carriers {
            carrier_reserved.insert(carrier.id, 0);
        }

        // Build UE-to-carrier assignments: ue_id -> [(carrier_id, prbs)]
        let mut ue_carrier_assignment: HashMap<usize, Vec<(usize, usize)>> = HashMap::new();

        // Sort UEs by demand descending so high-demand UEs get placed first
        let mut ue_order: Vec<usize> = (0..self.ues.len()).collect();
        ue_order.sort_by(|&a, &b| {
            self.ues[b]
                .demand_bps
                .partial_cmp(&self.ues[a].demand_bps)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        for &ue_idx in &ue_order {
            let ue = &self.ues[ue_idx];
            let mut remaining_demand = ue.demand_bps;

            // Sort supported carriers by remaining capacity (descending)
            let mut carrier_caps: Vec<(usize, usize, u8)> = ue
                .supported_carriers
                .iter()
                .filter_map(|&cid| {
                    let carrier = self.carriers.iter().find(|c| c.id == cid)?;
                    let reserved = carrier_reserved.get(&cid).copied().unwrap_or(0);
                    let available = carrier.num_prbs.saturating_sub(reserved);
                    let cqi = ue
                        .channel_quality
                        .iter()
                        .find(|(c, _)| *c == cid)
                        .map(|(_, q)| *q)
                        .unwrap_or(carrier.cqi);
                    Some((cid, available, cqi))
                })
                .collect();

            carrier_caps.sort_by(|a, b| b.1.cmp(&a.1));

            let assignments = ue_carrier_assignment.entry(ue.id).or_default();

            for (cid, available, cqi) in carrier_caps {
                if remaining_demand <= 0.0 || available == 0 {
                    break;
                }
                let eff = cqi_spectral_efficiency(cqi);
                let tp_per_prb = eff * PRB_BANDWIDTH_HZ;
                let needed = if tp_per_prb > 0.0 {
                    (remaining_demand / tp_per_prb).ceil() as usize
                } else {
                    available
                };
                let alloc = needed.min(available);
                assignments.push((cid, alloc));
                *carrier_reserved.entry(cid).or_insert(0) += alloc;
                remaining_demand -= prb_throughput(alloc, cqi);
            }
        }

        // Build allocation results per carrier
        for carrier in &self.carriers {
            let mut ue_prb_allocations = HashMap::new();
            let mut total_prbs_used = 0;
            let mut carrier_tp = 0.0;

            for ue in &self.ues {
                if let Some(assignments) = ue_carrier_assignment.get(&ue.id) {
                    for &(cid, prbs) in assignments {
                        if cid == carrier.id && prbs > 0 {
                            let cqi = ue
                                .channel_quality
                                .iter()
                                .find(|(c, _)| *c == carrier.id)
                                .map(|(_, q)| *q)
                                .unwrap_or(carrier.cqi);
                            let tp = prb_throughput(prbs, cqi);
                            ue_prb_allocations.insert(ue.id, prbs);
                            total_prbs_used += prbs;
                            carrier_tp += tp;
                            *ue_throughputs.entry(ue.id).or_insert(0.0) += tp;
                        }
                    }
                }
            }

            carrier_allocs.push(CarrierAllocation {
                carrier_id: carrier.id,
                ue_prb_allocations,
                total_prbs_used,
                throughput_bps: carrier_tp,
            });
        }

        let total_tp: f64 = carrier_allocs.iter().map(|a| a.throughput_bps).sum();
        let tp_values: Vec<f64> = ue_throughputs.values().copied().collect();
        let fairness = jains_fairness_index(&tp_values);

        self.update_avg_throughput(&ue_throughputs);

        AllocationResult {
            carrier_allocations: carrier_allocs,
            throughput_bps: total_tp,
            fairness_index: fairness,
        }
    }

    /// Update the exponential moving average of per-UE throughput (for PF scheduling).
    fn update_avg_throughput(&mut self, current: &HashMap<usize, f64>) {
        let alpha = self.pf_alpha;
        for ue in &self.ues {
            let cur = current.get(&ue.id).copied().unwrap_or(0.0);
            let avg = self.avg_throughput.entry(ue.id).or_insert(1.0);
            *avg = (1.0 - alpha) * (*avg) + alpha * cur;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_carrier(id: usize, num_prbs: usize, cqi: u8, is_primary: bool) -> ComponentCarrier {
        ComponentCarrier {
            id,
            center_freq_hz: 2.14e9 + id as f64 * 20e6,
            bandwidth_hz: 20e6,
            num_prbs,
            mcs_index: cqi_to_mcs(cqi),
            cqi,
            is_primary,
        }
    }

    fn make_ue(
        id: usize,
        demand: f64,
        carriers: Vec<usize>,
        cqi_per_carrier: Vec<(usize, u8)>,
    ) -> UserEquipment {
        UserEquipment {
            id,
            demand_bps: demand,
            supported_carriers: carriers,
            channel_quality: cqi_per_carrier,
        }
    }

    #[test]
    fn test_new_scheduler_single_carrier() {
        let cc = make_carrier(0, 100, 10, true);
        let sched = CarrierAggregationScheduler::new(vec![cc]);
        assert_eq!(sched.carriers().len(), 1);
    }

    #[test]
    fn test_new_scheduler_two_carriers() {
        let cc0 = make_carrier(0, 100, 10, true);
        let cc1 = make_carrier(1, 50, 8, false);
        let sched = CarrierAggregationScheduler::new(vec![cc0, cc1]);
        assert_eq!(sched.carriers().len(), 2);
    }

    #[test]
    #[should_panic(expected = "Carrier aggregation supports")]
    fn test_new_scheduler_too_many_carriers() {
        let carriers: Vec<_> = (0..6).map(|i| make_carrier(i, 50, 10, i == 0)).collect();
        CarrierAggregationScheduler::new(carriers);
    }

    #[test]
    #[should_panic(expected = "Carrier aggregation supports")]
    fn test_new_scheduler_empty_carriers() {
        CarrierAggregationScheduler::new(vec![]);
    }

    #[test]
    fn test_total_capacity() {
        let cc0 = make_carrier(0, 100, 10, true);
        let cc1 = make_carrier(1, 50, 10, false);
        let sched = CarrierAggregationScheduler::new(vec![cc0, cc1]);
        let cap = sched.total_capacity_bps();
        let expected = 150.0 * CQI_TO_EFFICIENCY[10] * PRB_BANDWIDTH_HZ;
        assert!(
            (cap - expected).abs() < 1.0,
            "Expected ~{}, got {}",
            expected,
            cap
        );
    }

    #[test]
    fn test_round_robin_single_ue() {
        let cc = make_carrier(0, 100, 10, true);
        let mut sched = CarrierAggregationScheduler::new(vec![cc]);
        sched.set_policy(SchedulingPolicy::RoundRobin);
        sched.add_ue(make_ue(0, 50e6, vec![0], vec![(0, 10)]));

        let result = sched.allocate();
        assert_eq!(result.carrier_allocations.len(), 1);
        assert_eq!(result.carrier_allocations[0].total_prbs_used, 100);
        assert!(result.throughput_bps > 0.0);
        assert!((result.fairness_index - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_round_robin_two_ues() {
        let cc = make_carrier(0, 100, 10, true);
        let mut sched = CarrierAggregationScheduler::new(vec![cc]);
        sched.set_policy(SchedulingPolicy::RoundRobin);
        sched.add_ue(make_ue(0, 50e6, vec![0], vec![(0, 10)]));
        sched.add_ue(make_ue(1, 50e6, vec![0], vec![(0, 10)]));

        let result = sched.allocate();
        let alloc = &result.carrier_allocations[0];
        assert_eq!(*alloc.ue_prb_allocations.get(&0).unwrap(), 50);
        assert_eq!(*alloc.ue_prb_allocations.get(&1).unwrap(), 50);
        assert!((result.fairness_index - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_round_robin_remainder_prbs() {
        let cc = make_carrier(0, 101, 10, true);
        let mut sched = CarrierAggregationScheduler::new(vec![cc]);
        sched.set_policy(SchedulingPolicy::RoundRobin);
        sched.add_ue(make_ue(0, 50e6, vec![0], vec![(0, 10)]));
        sched.add_ue(make_ue(1, 50e6, vec![0], vec![(0, 10)]));

        let result = sched.allocate();
        let alloc = &result.carrier_allocations[0];
        let prbs0 = *alloc.ue_prb_allocations.get(&0).unwrap();
        let prbs1 = *alloc.ue_prb_allocations.get(&1).unwrap();
        assert_eq!(prbs0 + prbs1, 101);
        assert!((prbs0 == 51 && prbs1 == 50) || (prbs0 == 50 && prbs1 == 51));
    }

    #[test]
    fn test_max_throughput_policy() {
        let cc = make_carrier(0, 100, 10, true);
        let mut sched = CarrierAggregationScheduler::new(vec![cc]);
        sched.set_policy(SchedulingPolicy::MaxThroughput);
        sched.add_ue(make_ue(0, 80e6, vec![0], vec![(0, 15)]));
        sched.add_ue(make_ue(1, 30e6, vec![0], vec![(0, 3)]));

        let result = sched.allocate();
        let alloc = &result.carrier_allocations[0];
        let prbs0 = alloc.ue_prb_allocations.get(&0).copied().unwrap_or(0);
        let prbs1 = alloc.ue_prb_allocations.get(&1).copied().unwrap_or(0);
        assert!(prbs0 > 0);
        assert!(result.throughput_bps > 0.0);
        assert!(
            prbs0 >= prbs1,
            "UE0 (CQI=15) should get >= PRBs than UE1 (CQI=3)"
        );
    }

    #[test]
    fn test_proportional_fair_policy() {
        let cc = make_carrier(0, 100, 10, true);
        let mut sched = CarrierAggregationScheduler::new(vec![cc]);
        sched.set_policy(SchedulingPolicy::ProportionalFair);
        sched.add_ue(make_ue(0, 50e6, vec![0], vec![(0, 12)]));
        sched.add_ue(make_ue(1, 50e6, vec![0], vec![(0, 8)]));

        let result = sched.allocate();
        assert!(result.throughput_bps > 0.0);
        let alloc = &result.carrier_allocations[0];
        assert!(alloc.ue_prb_allocations.get(&0).copied().unwrap_or(0) > 0);
        assert!(alloc.ue_prb_allocations.get(&1).copied().unwrap_or(0) > 0);
    }

    #[test]
    fn test_load_balance_policy() {
        let cc0 = make_carrier(0, 100, 10, true);
        let cc1 = make_carrier(1, 100, 10, false);
        let mut sched = CarrierAggregationScheduler::new(vec![cc0, cc1]);
        sched.set_policy(SchedulingPolicy::LoadBalance);
        sched.add_ue(make_ue(0, 80e6, vec![0, 1], vec![(0, 10), (1, 10)]));

        let result = sched.allocate();
        assert!(result.throughput_bps > 0.0);
        let used0 = result.carrier_allocations[0].total_prbs_used;
        let used1 = result.carrier_allocations[1].total_prbs_used;
        assert!(
            used0 > 0 || used1 > 0,
            "At least one carrier should be used"
        );
    }

    #[test]
    fn test_carrier_utilization_no_ues() {
        let cc = make_carrier(0, 100, 10, true);
        let sched = CarrierAggregationScheduler::new(vec![cc]);
        assert!((sched.carrier_utilization(0) - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_carrier_utilization_with_ues() {
        let cc = make_carrier(0, 100, 10, true);
        let mut sched = CarrierAggregationScheduler::new(vec![cc]);
        let cap = sched.total_capacity_bps();
        sched.add_ue(make_ue(0, cap, vec![0], vec![(0, 10)]));
        let util = sched.carrier_utilization(0);
        assert!(
            (util - 1.0).abs() < 0.01,
            "Expected utilization ~1.0, got {}",
            util
        );
    }

    #[test]
    fn test_carrier_utilization_nonexistent() {
        let cc = make_carrier(0, 100, 10, true);
        let sched = CarrierAggregationScheduler::new(vec![cc]);
        assert!((sched.carrier_utilization(999) - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_primary_and_secondary_carriers() {
        let cc0 = make_carrier(0, 100, 10, true);
        let cc1 = make_carrier(1, 50, 8, false);
        let cc2 = make_carrier(2, 50, 8, false);
        let sched = CarrierAggregationScheduler::new(vec![cc0, cc1, cc2]);

        let pcc = sched.primary_carrier().unwrap();
        assert_eq!(pcc.id, 0);
        assert!(pcc.is_primary);

        let sccs = sched.secondary_carriers();
        assert_eq!(sccs.len(), 2);
        assert!(sccs.iter().all(|c| !c.is_primary));
    }

    #[test]
    fn test_balance_load() {
        let cc0 = make_carrier(0, 100, 10, true);
        let cc1 = make_carrier(1, 100, 10, false);
        let mut sched = CarrierAggregationScheduler::new(vec![cc0, cc1]);
        sched.add_ue(make_ue(0, 100e6, vec![0], vec![(0, 10)]));
        sched.add_ue(make_ue(1, 100e6, vec![0], vec![(0, 10)]));

        let mcs_before = sched.carriers()[0].mcs_index;
        sched.balance_load();
        let mcs_after = sched.carriers()[0].mcs_index;
        assert!(
            mcs_after >= mcs_before,
            "Overloaded carrier MCS should increase or stay: {} -> {}",
            mcs_before,
            mcs_after
        );
    }

    #[test]
    fn test_jains_fairness_equal() {
        let vals = vec![100.0, 100.0, 100.0];
        let fi = jains_fairness_index(&vals);
        assert!((fi - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_jains_fairness_unequal() {
        let vals = vec![100.0, 0.0];
        let fi = jains_fairness_index(&vals);
        assert!((fi - 0.5).abs() < 1e-9);
    }

    #[test]
    fn test_cqi_to_efficiency_bounds() {
        assert_eq!(cqi_spectral_efficiency(0), 0.0);
        assert!(cqi_spectral_efficiency(15) > 5.0);
        assert!(cqi_spectral_efficiency(1) > 0.0);
    }

    #[test]
    fn test_multi_carrier_allocation() {
        let cc0 = make_carrier(0, 100, 10, true);
        let cc1 = make_carrier(1, 50, 12, false);
        let mut sched = CarrierAggregationScheduler::new(vec![cc0, cc1]);
        sched.set_policy(SchedulingPolicy::RoundRobin);

        sched.add_ue(make_ue(0, 60e6, vec![0, 1], vec![(0, 10), (1, 12)]));
        sched.add_ue(make_ue(1, 40e6, vec![0], vec![(0, 8)]));

        let result = sched.allocate();
        assert_eq!(result.carrier_allocations.len(), 2);
        let alloc0 = &result.carrier_allocations[0];
        assert_eq!(alloc0.total_prbs_used, 100);
        let alloc1 = &result.carrier_allocations[1];
        assert_eq!(alloc1.total_prbs_used, 50);
        assert_eq!(*alloc1.ue_prb_allocations.get(&0).unwrap(), 50);
        assert!(result.throughput_bps > 0.0);
    }

    #[test]
    fn test_allocate_no_ues() {
        let cc = make_carrier(0, 100, 10, true);
        let mut sched = CarrierAggregationScheduler::new(vec![cc]);
        let result = sched.allocate();
        assert_eq!(result.throughput_bps, 0.0);
        assert!((result.fairness_index - 1.0).abs() < 1e-9);
    }
}
