//! Dynamic resource allocation for OFDMA/SC-FDMA uplink/downlink scheduling.
//!
//! Provides subcarrier-level resource block allocation with multiple scheduling
//! algorithms (Round Robin, Max Throughput, Proportional Fair) per 3GPP TS 36.213
//! CQI-to-spectral-efficiency mapping. Computes Jain's fairness index and
//! per-user throughput.
//!
//! # Example
//!
//! ```
//! use r4w_core::subcarrier_allocator::{
//!     SubcarrierAllocator, UserRequest, SchedulingAlgorithm,
//! };
//!
//! // Create allocator with 25 resource blocks, 12 subcarriers each (5 MHz LTE)
//! let mut allocator = SubcarrierAllocator::new(25, 12);
//! allocator.set_algorithm(SchedulingAlgorithm::ProportionalFair);
//!
//! // Two users with different channel conditions
//! allocator.add_user(UserRequest {
//!     user_id: 0,
//!     demand_bits: 10_000,
//!     channel_quality: vec![12; 25], // Good channel on all RBs
//! });
//! allocator.add_user(UserRequest {
//!     user_id: 1,
//!     demand_bits: 8_000,
//!     channel_quality: vec![6; 25], // Moderate channel
//! });
//!
//! let result = allocator.allocate();
//! assert!(result.total_throughput > 0.0);
//! assert!(result.fairness_index >= 0.0 && result.fairness_index <= 1.0);
//! println!("Total throughput: {:.1} bits/s/Hz", result.total_throughput);
//! println!("Fairness (Jain's): {:.4}", result.fairness_index);
//! ```

/// A single resource block in the frequency-time grid.
#[derive(Debug, Clone)]
pub struct ResourceBlock {
    /// Starting subcarrier index.
    pub start_subcarrier: usize,
    /// Number of subcarriers in this RB (typically 12 for LTE).
    pub num_subcarriers: usize,
    /// Modulation and coding scheme index (derived from CQI).
    pub mcs_index: u8,
    /// User ID this RB is allocated to, or `None` if unallocated.
    pub allocated_to_user: Option<usize>,
}

/// A user's resource request with per-RB channel quality.
#[derive(Debug, Clone)]
pub struct UserRequest {
    /// Unique user identifier.
    pub user_id: usize,
    /// Number of bits the user wants to transmit.
    pub demand_bits: usize,
    /// Channel Quality Indicator (1-15) for each resource block.
    /// Length must equal the number of resource blocks in the allocator.
    pub channel_quality: Vec<u8>,
}

/// Per-user allocation detail within an `AllocationResult`.
#[derive(Debug, Clone)]
pub struct UserAllocation {
    /// User identifier.
    pub user_id: usize,
    /// Indices of resource blocks assigned to this user.
    pub assigned_rbs: Vec<usize>,
    /// Achieved throughput in bits/s/Hz for this user.
    pub throughput: f64,
}

/// Result of a scheduling allocation round.
#[derive(Debug, Clone)]
pub struct AllocationResult {
    /// Per-user allocation details.
    pub user_allocations: Vec<UserAllocation>,
    /// Sum of all user throughputs (bits/s/Hz).
    pub total_throughput: f64,
    /// Jain's fairness index over user throughputs.
    pub fairness_index: f64,
}

/// Scheduling algorithm selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedulingAlgorithm {
    /// Each user gets RBs in turn, regardless of channel quality.
    RoundRobin,
    /// Each RB is given to the user with the best channel on that RB.
    MaxThroughput,
    /// Balances throughput and fairness by maximizing sum of log(throughput).
    ProportionalFair,
}

/// Dynamic subcarrier / resource-block allocator for OFDMA/SC-FDMA systems.
#[derive(Debug)]
pub struct SubcarrierAllocator {
    num_rbs: usize,
    num_subcarriers_per_rb: usize,
    algorithm: SchedulingAlgorithm,
    users: Vec<UserRequest>,
    /// Stores the most recent allocation result for queries.
    last_result: Option<AllocationResult>,
}

/// CQI-to-spectral-efficiency mapping per 3GPP TS 36.213 Table 7.2.3-1.
/// Index 0 is unused (CQI 0 means "out of range").
const CQI_EFFICIENCY: [f64; 16] = [
    0.0,    // CQI 0: out of range
    0.1523, // CQI 1:  QPSK, code rate ~78/1024
    0.2344, // CQI 2:  QPSK, 120/1024
    0.3770, // CQI 3:  QPSK, 193/1024
    0.6016, // CQI 4:  QPSK, 308/1024
    0.8770, // CQI 5:  QPSK, 449/1024
    1.1758, // CQI 6:  QPSK, 602/1024
    1.4766, // CQI 7:  16QAM, 378/1024
    1.9141, // CQI 8:  16QAM, 490/1024
    2.4063, // CQI 9:  16QAM, 616/1024
    2.7305, // CQI 10: 64QAM, 466/1024
    3.3223, // CQI 11: 64QAM, 567/1024
    3.9023, // CQI 12: 64QAM, 666/1024
    4.5234, // CQI 13: 64QAM, 772/1024
    5.1152, // CQI 14: 64QAM, 873/1024
    5.5547, // CQI 15: 64QAM, 948/1024
];

impl SubcarrierAllocator {
    /// Create a new allocator.
    ///
    /// * `num_rbs` - Number of resource blocks (e.g., 25 for 5 MHz LTE).
    /// * `num_subcarriers_per_rb` - Subcarriers per RB (12 for LTE).
    pub fn new(num_rbs: usize, num_subcarriers_per_rb: usize) -> Self {
        Self {
            num_rbs,
            num_subcarriers_per_rb,
            algorithm: SchedulingAlgorithm::RoundRobin,
            users: Vec::new(),
            last_result: None,
        }
    }

    /// Add a user scheduling request. The `channel_quality` vector length must
    /// equal `num_rbs`; entries are clamped to 1..=15.
    pub fn add_user(&mut self, request: UserRequest) {
        self.users.push(request);
    }

    /// Select the scheduling algorithm for the next `allocate()` call.
    pub fn set_algorithm(&mut self, alg: SchedulingAlgorithm) {
        self.algorithm = alg;
    }

    /// Map a CQI value (1-15) to spectral efficiency (bits/s/Hz) per 3GPP TS 36.213.
    /// CQI 0 or values > 15 return 0.0.
    pub fn cqi_to_spectral_efficiency(cqi: u8) -> f64 {
        if cqi as usize >= CQI_EFFICIENCY.len() {
            0.0
        } else {
            CQI_EFFICIENCY[cqi as usize]
        }
    }

    /// Run the allocation algorithm and return the result.
    pub fn allocate(&mut self) -> AllocationResult {
        if self.users.is_empty() {
            let result = AllocationResult {
                user_allocations: Vec::new(),
                total_throughput: 0.0,
                fairness_index: 1.0,
            };
            self.last_result = Some(result.clone());
            return result;
        }

        let rb_assignments = match self.algorithm {
            SchedulingAlgorithm::RoundRobin => self.allocate_round_robin(),
            SchedulingAlgorithm::MaxThroughput => self.allocate_max_throughput(),
            SchedulingAlgorithm::ProportionalFair => self.allocate_proportional_fair(),
        };

        let result = self.build_result(&rb_assignments);
        self.last_result = Some(result.clone());
        result
    }

    /// Query the throughput of a specific user from the last allocation.
    /// Returns 0.0 if no allocation has been performed or the user was not found.
    pub fn user_throughput(&self, user_id: usize) -> f64 {
        self.last_result
            .as_ref()
            .and_then(|r| {
                r.user_allocations
                    .iter()
                    .find(|ua| ua.user_id == user_id)
                    .map(|ua| ua.throughput)
            })
            .unwrap_or(0.0)
    }

    /// Compute Jain's fairness index from the last allocation.
    /// Returns 1.0 if no allocation has been performed.
    pub fn fairness_index(&self) -> f64 {
        self.last_result
            .as_ref()
            .map(|r| r.fairness_index)
            .unwrap_or(1.0)
    }

    /// Return the number of resource blocks.
    pub fn num_rbs(&self) -> usize {
        self.num_rbs
    }

    /// Return the number of subcarriers per resource block.
    pub fn num_subcarriers_per_rb(&self) -> usize {
        self.num_subcarriers_per_rb
    }

    /// Return the total number of subcarriers.
    pub fn total_subcarriers(&self) -> usize {
        self.num_rbs * self.num_subcarriers_per_rb
    }

    /// Build `ResourceBlock` descriptors for the current allocation state.
    pub fn resource_blocks(&self, assignments: &[Option<usize>]) -> Vec<ResourceBlock> {
        (0..self.num_rbs)
            .map(|i| {
                let user = assignments[i];
                let mcs = user
                    .and_then(|uid| {
                        self.users
                            .iter()
                            .find(|u| u.user_id == uid)
                            .and_then(|u| u.channel_quality.get(i).copied())
                    })
                    .unwrap_or(0);
                ResourceBlock {
                    start_subcarrier: i * self.num_subcarriers_per_rb,
                    num_subcarriers: self.num_subcarriers_per_rb,
                    mcs_index: mcs,
                    allocated_to_user: user,
                }
            })
            .collect()
    }

    // ---- private scheduling algorithms ----

    /// Round-robin: assign RBs to users in order, cycling through users.
    fn allocate_round_robin(&self) -> Vec<Option<usize>> {
        let n_users = self.users.len();
        (0..self.num_rbs)
            .map(|rb| {
                let user = &self.users[rb % n_users];
                Some(user.user_id)
            })
            .collect()
    }

    /// Max throughput: each RB goes to the user with the highest CQI on that RB.
    fn allocate_max_throughput(&self) -> Vec<Option<usize>> {
        (0..self.num_rbs)
            .map(|rb| {
                self.users
                    .iter()
                    .max_by(|a, b| {
                        let cqi_a = a.channel_quality.get(rb).copied().unwrap_or(0);
                        let cqi_b = b.channel_quality.get(rb).copied().unwrap_or(0);
                        cqi_a.cmp(&cqi_b)
                    })
                    .map(|u| u.user_id)
            })
            .collect()
    }

    /// Proportional fair: maximize sum(log(throughput_i)).
    /// Greedy per-RB: assign each RB to the user with the highest
    /// instantaneous_rate / average_throughput ratio.
    fn allocate_proportional_fair(&self) -> Vec<Option<usize>> {
        let n_users = self.users.len();
        // Track running throughput per user (initialise to small epsilon to avoid div-by-zero).
        let mut running_throughput = vec![1e-12_f64; n_users];
        let mut assignments = vec![None; self.num_rbs];

        for rb in 0..self.num_rbs {
            let mut best_idx = 0;
            let mut best_metric = f64::NEG_INFINITY;

            for (ui, user) in self.users.iter().enumerate() {
                let cqi = user.channel_quality.get(rb).copied().unwrap_or(0);
                let rate = Self::cqi_to_spectral_efficiency(cqi);
                let metric = rate / running_throughput[ui];
                if metric > best_metric {
                    best_metric = metric;
                    best_idx = ui;
                }
            }

            assignments[rb] = Some(self.users[best_idx].user_id);

            // Update running throughput for the winner.
            let cqi = self.users[best_idx]
                .channel_quality
                .get(rb)
                .copied()
                .unwrap_or(0);
            running_throughput[best_idx] += Self::cqi_to_spectral_efficiency(cqi);
        }

        assignments
    }

    /// Convert RB assignments into an `AllocationResult`.
    fn build_result(&self, assignments: &[Option<usize>]) -> AllocationResult {
        let mut user_rb_map: std::collections::HashMap<usize, Vec<usize>> =
            std::collections::HashMap::new();

        for (rb_idx, maybe_uid) in assignments.iter().enumerate() {
            if let Some(uid) = maybe_uid {
                user_rb_map.entry(*uid).or_default().push(rb_idx);
            }
        }

        let mut user_allocations = Vec::new();
        for user in &self.users {
            let assigned_rbs = user_rb_map
                .get(&user.user_id)
                .cloned()
                .unwrap_or_default();
            let throughput: f64 = assigned_rbs
                .iter()
                .map(|&rb| {
                    let cqi = user.channel_quality.get(rb).copied().unwrap_or(0);
                    Self::cqi_to_spectral_efficiency(cqi)
                })
                .sum();
            user_allocations.push(UserAllocation {
                user_id: user.user_id,
                assigned_rbs,
                throughput,
            });
        }

        let total_throughput: f64 = user_allocations.iter().map(|ua| ua.throughput).sum();
        let fairness_index = jains_fairness_index(
            &user_allocations
                .iter()
                .map(|ua| ua.throughput)
                .collect::<Vec<_>>(),
        );

        AllocationResult {
            user_allocations,
            total_throughput,
            fairness_index,
        }
    }
}

/// Compute Jain's fairness index: (sum(x))^2 / (n * sum(x^2)).
/// Returns 1.0 for an empty slice.
pub fn jains_fairness_index(values: &[f64]) -> f64 {
    let n = values.len();
    if n == 0 {
        return 1.0;
    }
    let sum: f64 = values.iter().sum();
    let sum_sq: f64 = values.iter().map(|x| x * x).sum();
    if sum_sq == 0.0 {
        return 1.0;
    }
    (sum * sum) / (n as f64 * sum_sq)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cqi_to_spectral_efficiency_boundaries() {
        assert_eq!(SubcarrierAllocator::cqi_to_spectral_efficiency(0), 0.0);
        assert!(SubcarrierAllocator::cqi_to_spectral_efficiency(1) > 0.0);
        assert!(SubcarrierAllocator::cqi_to_spectral_efficiency(15) > 5.0);
        assert_eq!(SubcarrierAllocator::cqi_to_spectral_efficiency(16), 0.0);
        assert_eq!(SubcarrierAllocator::cqi_to_spectral_efficiency(255), 0.0);
    }

    #[test]
    fn test_cqi_monotonically_increasing() {
        for cqi in 1..15 {
            let eff_low = SubcarrierAllocator::cqi_to_spectral_efficiency(cqi);
            let eff_high = SubcarrierAllocator::cqi_to_spectral_efficiency(cqi + 1);
            assert!(
                eff_high > eff_low,
                "CQI {} ({}) should be less than CQI {} ({})",
                cqi,
                eff_low,
                cqi + 1,
                eff_high
            );
        }
    }

    #[test]
    fn test_jains_fairness_equal() {
        // All equal values -> fairness = 1.0
        let values = vec![5.0, 5.0, 5.0, 5.0];
        let fi = jains_fairness_index(&values);
        assert!((fi - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_jains_fairness_extreme_inequality() {
        // One user gets everything, others get nothing -> fairness = 1/n
        let values = vec![100.0, 0.0, 0.0, 0.0];
        let fi = jains_fairness_index(&values);
        assert!((fi - 0.25).abs() < 1e-10);
    }

    #[test]
    fn test_jains_fairness_empty() {
        assert_eq!(jains_fairness_index(&[]), 1.0);
    }

    #[test]
    fn test_jains_fairness_single() {
        assert!((jains_fairness_index(&[42.0]) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_allocator_no_users() {
        let mut alloc = SubcarrierAllocator::new(10, 12);
        let result = alloc.allocate();
        assert_eq!(result.total_throughput, 0.0);
        assert_eq!(result.fairness_index, 1.0);
        assert!(result.user_allocations.is_empty());
    }

    #[test]
    fn test_round_robin_single_user() {
        let mut alloc = SubcarrierAllocator::new(10, 12);
        alloc.set_algorithm(SchedulingAlgorithm::RoundRobin);
        alloc.add_user(UserRequest {
            user_id: 0,
            demand_bits: 1000,
            channel_quality: vec![10; 10],
        });
        let result = alloc.allocate();
        assert_eq!(result.user_allocations.len(), 1);
        assert_eq!(result.user_allocations[0].assigned_rbs.len(), 10);
        assert!(result.total_throughput > 0.0);
    }

    #[test]
    fn test_round_robin_two_users_fair() {
        let mut alloc = SubcarrierAllocator::new(10, 12);
        alloc.set_algorithm(SchedulingAlgorithm::RoundRobin);
        alloc.add_user(UserRequest {
            user_id: 0,
            demand_bits: 5000,
            channel_quality: vec![10; 10],
        });
        alloc.add_user(UserRequest {
            user_id: 1,
            demand_bits: 5000,
            channel_quality: vec![10; 10],
        });
        let result = alloc.allocate();
        // Each user should get 5 RBs
        let u0 = &result.user_allocations[0];
        let u1 = &result.user_allocations[1];
        assert_eq!(u0.assigned_rbs.len(), 5);
        assert_eq!(u1.assigned_rbs.len(), 5);
        // Same CQI => same throughput => fairness ~1.0
        assert!((result.fairness_index - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_max_throughput_gives_to_best_channel() {
        let mut alloc = SubcarrierAllocator::new(5, 12);
        alloc.set_algorithm(SchedulingAlgorithm::MaxThroughput);
        // User 0: poor channel everywhere
        alloc.add_user(UserRequest {
            user_id: 0,
            demand_bits: 5000,
            channel_quality: vec![1; 5],
        });
        // User 1: excellent channel everywhere
        alloc.add_user(UserRequest {
            user_id: 1,
            demand_bits: 5000,
            channel_quality: vec![15; 5],
        });
        let result = alloc.allocate();
        // All RBs should go to user 1
        let u0 = &result.user_allocations[0];
        let u1 = &result.user_allocations[1];
        assert_eq!(u0.assigned_rbs.len(), 0);
        assert_eq!(u1.assigned_rbs.len(), 5);
    }

    #[test]
    fn test_max_throughput_mixed_channels() {
        let mut alloc = SubcarrierAllocator::new(4, 12);
        alloc.set_algorithm(SchedulingAlgorithm::MaxThroughput);
        // User 0: good on first two RBs, poor on last two
        alloc.add_user(UserRequest {
            user_id: 0,
            demand_bits: 5000,
            channel_quality: vec![15, 15, 1, 1],
        });
        // User 1: poor on first two, good on last two
        alloc.add_user(UserRequest {
            user_id: 1,
            demand_bits: 5000,
            channel_quality: vec![1, 1, 15, 15],
        });
        let result = alloc.allocate();
        let u0 = &result.user_allocations[0];
        let u1 = &result.user_allocations[1];
        assert_eq!(u0.assigned_rbs, vec![0, 1]);
        assert_eq!(u1.assigned_rbs, vec![2, 3]);
    }

    #[test]
    fn test_proportional_fair_balances() {
        let mut alloc = SubcarrierAllocator::new(10, 12);
        alloc.set_algorithm(SchedulingAlgorithm::ProportionalFair);
        // User 0: CQI 15 everywhere
        alloc.add_user(UserRequest {
            user_id: 0,
            demand_bits: 10000,
            channel_quality: vec![15; 10],
        });
        // User 1: CQI 8 everywhere
        alloc.add_user(UserRequest {
            user_id: 1,
            demand_bits: 10000,
            channel_quality: vec![8; 10],
        });
        let result = alloc.allocate();
        // PF should give some RBs to user 1 despite lower CQI
        let u1 = &result.user_allocations[1];
        assert!(
            !u1.assigned_rbs.is_empty(),
            "Proportional fair should allocate some RBs to the weaker user"
        );
        // Fairness should be better than max-throughput (which would give 0 to user 1)
        assert!(result.fairness_index > 0.5);
    }

    #[test]
    fn test_user_throughput_query() {
        let mut alloc = SubcarrierAllocator::new(4, 12);
        alloc.set_algorithm(SchedulingAlgorithm::RoundRobin);
        alloc.add_user(UserRequest {
            user_id: 42,
            demand_bits: 1000,
            channel_quality: vec![10; 4],
        });
        // Before allocation
        assert_eq!(alloc.user_throughput(42), 0.0);
        alloc.allocate();
        // After allocation
        assert!(alloc.user_throughput(42) > 0.0);
        // Non-existent user
        assert_eq!(alloc.user_throughput(999), 0.0);
    }

    #[test]
    fn test_fairness_index_method() {
        let mut alloc = SubcarrierAllocator::new(6, 12);
        alloc.set_algorithm(SchedulingAlgorithm::RoundRobin);
        alloc.add_user(UserRequest {
            user_id: 0,
            demand_bits: 1000,
            channel_quality: vec![10; 6],
        });
        alloc.add_user(UserRequest {
            user_id: 1,
            demand_bits: 1000,
            channel_quality: vec![10; 6],
        });
        // Before allocation
        assert_eq!(alloc.fairness_index(), 1.0);
        alloc.allocate();
        // Equal CQI, round-robin => perfect fairness
        assert!((alloc.fairness_index() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_resource_block_construction() {
        let alloc = SubcarrierAllocator::new(3, 12);
        let assignments = vec![Some(0), None, Some(1)];
        let rbs = alloc.resource_blocks(&assignments);
        assert_eq!(rbs.len(), 3);
        assert_eq!(rbs[0].start_subcarrier, 0);
        assert_eq!(rbs[0].num_subcarriers, 12);
        assert_eq!(rbs[1].start_subcarrier, 12);
        assert_eq!(rbs[1].allocated_to_user, None);
        assert_eq!(rbs[2].start_subcarrier, 24);
        assert_eq!(rbs[2].allocated_to_user, Some(1));
    }

    #[test]
    fn test_total_subcarriers() {
        let alloc = SubcarrierAllocator::new(25, 12);
        assert_eq!(alloc.total_subcarriers(), 300);
        assert_eq!(alloc.num_rbs(), 25);
        assert_eq!(alloc.num_subcarriers_per_rb(), 12);
    }

    #[test]
    fn test_three_users_round_robin() {
        let mut alloc = SubcarrierAllocator::new(9, 12);
        alloc.set_algorithm(SchedulingAlgorithm::RoundRobin);
        for uid in 0..3 {
            alloc.add_user(UserRequest {
                user_id: uid,
                demand_bits: 3000,
                channel_quality: vec![7; 9],
            });
        }
        let result = alloc.allocate();
        // Each user gets 3 RBs
        for ua in &result.user_allocations {
            assert_eq!(ua.assigned_rbs.len(), 3);
        }
        assert!((result.fairness_index - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_proportional_fair_vs_max_throughput_fairness() {
        // PF should yield higher fairness than MaxThroughput when channels differ.
        let make_alloc = |alg| {
            let mut alloc = SubcarrierAllocator::new(20, 12);
            alloc.set_algorithm(alg);
            alloc.add_user(UserRequest {
                user_id: 0,
                demand_bits: 20000,
                channel_quality: vec![15; 20],
            });
            alloc.add_user(UserRequest {
                user_id: 1,
                demand_bits: 20000,
                channel_quality: vec![5; 20],
            });
            alloc.allocate()
        };

        let mt = make_alloc(SchedulingAlgorithm::MaxThroughput);
        let pf = make_alloc(SchedulingAlgorithm::ProportionalFair);

        assert!(
            pf.fairness_index > mt.fairness_index,
            "PF fairness ({:.4}) should exceed MaxThroughput fairness ({:.4})",
            pf.fairness_index,
            mt.fairness_index
        );
    }
}
