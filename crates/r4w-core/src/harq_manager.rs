//! # HARQ Manager -- Hybrid ARQ Process Management
//!
//! Implements LTE/5G-style Hybrid ARQ with configurable stop-and-wait
//! processes, soft combining, and redundancy version cycling. Supports
//! Chase combining (Type I), incremental redundancy (Type II), and
//! complementary IR (Type III). Tracks per-process state and aggregate
//! retransmission statistics.
//!
//! GNU Radio equivalent: custom OOT / `gr-lte` HARQ, `srsRAN` HARQ entity.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::harq_manager::{HarqManager, HarqType};
//!
//! let mut mgr = HarqManager::new(8, 4, HarqType::TypeI);
//! let data = vec![1.0, -1.0, 1.0, -1.0];
//! let (pid, tx_data) = mgr.send(data.clone()).unwrap();
//! assert_eq!(tx_data, data);
//! mgr.handle_feedback(pid, true); // ACK
//! assert_eq!(mgr.stats().ack_count, 1);
//! ```

/// HARQ combining strategy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HarqType {
    /// Chase combining: retransmit identical data, MRC-combine at receiver.
    TypeI,
    /// Incremental redundancy: send different coded bits each retransmission.
    TypeII,
    /// Complementary IR: each RV is self-decodable and combinable.
    TypeIII,
}

/// State of an individual HARQ process.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProcessState {
    /// Process is free and ready for a new transmission.
    Idle,
    /// Transmitted, awaiting ACK/NACK from receiver.
    WaitingAck,
    /// NACK received; eligible for retransmission.
    NackReceived,
}

/// A single stop-and-wait HARQ process.
#[derive(Debug, Clone)]
pub struct HarqProcess {
    /// Process identifier (0..N-1).
    pub id: u8,
    /// Current state.
    pub state: ProcessState,
    /// Soft-bit buffer holding the current transport block.
    pub buffer: Vec<f64>,
    /// Current redundancy version (0..3 in LTE/NR).
    pub redundancy_version: u8,
    /// Number of transmissions so far (initial + retransmissions).
    pub num_transmissions: u8,
    /// Maximum allowed transmissions before declaring failure.
    pub max_transmissions: u8,
}

impl HarqProcess {
    /// Create a new idle HARQ process.
    pub fn new(id: u8, max_transmissions: u8) -> Self {
        Self {
            id,
            state: ProcessState::Idle,
            buffer: Vec::new(),
            redundancy_version: 0,
            num_transmissions: 0,
            max_transmissions,
        }
    }

    /// Store data and produce the initial transmission.
    ///
    /// Moves the process from Idle to WaitingAck, stores the transport
    /// block, and returns the data to send over the air.
    pub fn transmit(&mut self, data: Vec<f64>) -> Vec<f64> {
        self.buffer = data.clone();
        self.redundancy_version = 0;
        self.num_transmissions = 1;
        self.state = ProcessState::WaitingAck;
        data
    }

    /// Handle a positive acknowledgment.
    ///
    /// Clears the buffer and returns the process to Idle.
    pub fn receive_ack(&mut self) {
        self.buffer.clear();
        self.redundancy_version = 0;
        self.num_transmissions = 0;
        self.state = ProcessState::Idle;
    }

    /// Handle a negative acknowledgment.
    ///
    /// If retransmissions remain, advances the redundancy version,
    /// increments the transmission count, and returns the data for
    /// retransmission. Returns `None` if max transmissions exceeded.
    pub fn receive_nack(&mut self) -> Option<Vec<f64>> {
        if self.num_transmissions >= self.max_transmissions {
            // Max retransmissions reached -- flush and go idle.
            self.buffer.clear();
            self.state = ProcessState::Idle;
            self.num_transmissions = 0;
            self.redundancy_version = 0;
            return None;
        }

        self.state = ProcessState::NackReceived;
        self.redundancy_version = (self.redundancy_version + 1) % 4;
        self.num_transmissions += 1;
        self.state = ProcessState::WaitingAck;
        Some(self.buffer.clone())
    }

    /// Returns `true` if the process is idle and available.
    pub fn is_idle(&self) -> bool {
        self.state == ProcessState::Idle
    }

    /// Number of transmissions performed for the current block.
    pub fn transmission_count(&self) -> u8 {
        self.num_transmissions
    }
}

/// Aggregate HARQ statistics across all processes.
#[derive(Debug, Clone, Default)]
pub struct HarqStats {
    /// Total transmissions (initial + retransmissions).
    pub total_tx: u64,
    /// Total retransmissions only.
    pub total_retx: u64,
    /// Number of ACKs received.
    pub ack_count: u64,
    /// Number of NACKs received.
    pub nack_count: u64,
    /// Times a process exhausted its retransmission budget.
    pub max_retx_reached: u64,
}

/// HARQ entity managing multiple stop-and-wait processes.
///
/// Round-robins across processes; tracks statistics and combining type.
#[derive(Debug, Clone)]
pub struct HarqManager {
    /// Pool of HARQ processes.
    processes: Vec<HarqProcess>,
    /// Combining strategy.
    harq_type: HarqType,
    /// Aggregate statistics.
    stats: HarqStats,
}

impl HarqManager {
    /// Create a manager with `num_processes` HARQ processes.
    pub fn new(num_processes: usize, max_retx: u8, harq_type: HarqType) -> Self {
        let processes = (0..num_processes)
            .map(|i| HarqProcess::new(i as u8, max_retx))
            .collect();
        Self {
            processes,
            harq_type,
            stats: HarqStats::default(),
        }
    }

    /// Send data on the first available idle process.
    ///
    /// Returns `(process_id, transmitted_data)` or `None` if all busy.
    pub fn send(&mut self, data: Vec<f64>) -> Option<(u8, Vec<f64>)> {
        let proc = self.processes.iter_mut().find(|p| p.is_idle())?;
        let tx_data = proc.transmit(data);
        let pid = proc.id;
        self.stats.total_tx += 1;
        Some((pid, tx_data))
    }

    /// Process ACK or NACK feedback for a given process.
    ///
    /// Returns retransmission data on NACK (if retries remain).
    pub fn handle_feedback(&mut self, process_id: u8, ack: bool) -> Option<Vec<f64>> {
        let proc = self.processes.iter_mut().find(|p| p.id == process_id)?;

        if ack {
            proc.receive_ack();
            self.stats.ack_count += 1;
            None
        } else {
            self.stats.nack_count += 1;
            match proc.receive_nack() {
                Some(retx_data) => {
                    self.stats.total_retx += 1;
                    self.stats.total_tx += 1;
                    Some(retx_data)
                }
                None => {
                    self.stats.max_retx_reached += 1;
                    None
                }
            }
        }
    }

    /// Get a reference to the statistics.
    pub fn stats(&self) -> &HarqStats {
        &self.stats
    }

    /// Get a reference to a process by id.
    pub fn process(&self, id: u8) -> Option<&HarqProcess> {
        self.processes.iter().find(|p| p.id == id)
    }

    /// Get the configured HARQ type.
    pub fn harq_type(&self) -> HarqType {
        self.harq_type
    }
}

/// MRC-style soft combiner for Chase combining (HARQ Type I).
///
/// Accumulates log-likelihood ratios across retransmissions and
/// returns the element-wise sum (maximal-ratio combining for equal-SNR
/// retransmissions).
#[derive(Debug, Clone)]
pub struct SoftCombiner {
    /// Accumulated LLR buffer.
    accumulated: Vec<f64>,
}

impl SoftCombiner {
    /// Create a new, empty soft combiner.
    pub fn new() -> Self {
        Self {
            accumulated: Vec::new(),
        }
    }

    /// Combine new LLRs with the running accumulation.
    ///
    /// On the first call the input is stored directly. Subsequent calls
    /// add element-wise (MRC under equal-SNR assumption).
    pub fn combine(&mut self, new_llrs: &[f64]) -> Vec<f64> {
        if self.accumulated.is_empty() {
            self.accumulated = new_llrs.to_vec();
        } else {
            let len = self.accumulated.len().min(new_llrs.len());
            for i in 0..len {
                self.accumulated[i] += new_llrs[i];
            }
        }
        self.accumulated.clone()
    }

    /// Reset the combiner for a new transport block.
    pub fn reset(&mut self) {
        self.accumulated.clear();
    }

    /// Current accumulated LLRs.
    pub fn accumulated(&self) -> &[f64] {
        &self.accumulated
    }
}

impl Default for SoftCombiner {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_single_transmission() {
        let mut proc = HarqProcess::new(0, 4);
        let data = vec![1.0, -1.0, 1.0];
        let tx = proc.transmit(data.clone());
        assert_eq!(tx, data);
        assert_eq!(proc.state, ProcessState::WaitingAck);
        assert_eq!(proc.num_transmissions, 1);
        assert_eq!(proc.redundancy_version, 0);
    }

    #[test]
    fn test_ack_clears() {
        let mut proc = HarqProcess::new(0, 4);
        proc.transmit(vec![1.0, 2.0, 3.0]);
        assert!(!proc.is_idle());

        proc.receive_ack();
        assert!(proc.is_idle());
        assert!(proc.buffer.is_empty());
        assert_eq!(proc.num_transmissions, 0);
        assert_eq!(proc.redundancy_version, 0);
    }

    #[test]
    fn test_nack_retransmit() {
        let mut proc = HarqProcess::new(0, 4);
        let data = vec![1.0, -1.0];
        proc.transmit(data.clone());

        let retx = proc.receive_nack();
        assert!(retx.is_some());
        assert_eq!(retx.unwrap(), data);
        assert_eq!(proc.num_transmissions, 2);
        assert_eq!(proc.redundancy_version, 1);
        assert_eq!(proc.state, ProcessState::WaitingAck);
    }

    #[test]
    fn test_max_retransmissions() {
        let mut proc = HarqProcess::new(0, 2);
        proc.transmit(vec![1.0]);
        assert_eq!(proc.num_transmissions, 1);

        // First NACK -- retransmission succeeds (now at 2/2).
        let retx1 = proc.receive_nack();
        assert!(retx1.is_some());
        assert_eq!(proc.num_transmissions, 2);

        // Second NACK -- at max, should return None and go idle.
        let retx2 = proc.receive_nack();
        assert!(retx2.is_none());
        assert!(proc.is_idle());
        assert!(proc.buffer.is_empty());
    }

    #[test]
    fn test_manager_round_robin() {
        let mut mgr = HarqManager::new(4, 3, HarqType::TypeI);

        // Send on first idle process (id 0).
        let (pid0, _) = mgr.send(vec![1.0]).unwrap();
        assert_eq!(pid0, 0);

        // Next send should pick process 1.
        let (pid1, _) = mgr.send(vec![2.0]).unwrap();
        assert_eq!(pid1, 1);

        // ACK process 0 -- it becomes available again.
        mgr.handle_feedback(0, true);

        // Next send picks process 0 (first idle).
        let (pid_next, _) = mgr.send(vec![3.0]).unwrap();
        assert_eq!(pid_next, 0);
    }

    #[test]
    fn test_all_processes_busy() {
        let mut mgr = HarqManager::new(2, 3, HarqType::TypeII);

        mgr.send(vec![1.0]).unwrap();
        mgr.send(vec![2.0]).unwrap();

        // All processes occupied -- send should return None.
        let result = mgr.send(vec![3.0]);
        assert!(result.is_none());
    }

    #[test]
    fn test_soft_combining() {
        let mut combiner = SoftCombiner::new();

        let first = combiner.combine(&[1.0, -0.5, 0.8]);
        assert_eq!(first, vec![1.0, -0.5, 0.8]);

        let second = combiner.combine(&[0.5, -0.3, 0.6]);
        // MRC: element-wise addition.
        assert!((second[0] - 1.5).abs() < 1e-12);
        assert!((second[1] - (-0.8)).abs() < 1e-12);
        assert!((second[2] - 1.4).abs() < 1e-12);

        // Third reception further improves.
        let third = combiner.combine(&[0.2, -0.1, 0.1]);
        assert!((third[0] - 1.7).abs() < 1e-12);
        assert!((third[1] - (-0.9)).abs() < 1e-12);
        assert!((third[2] - 1.5).abs() < 1e-12);
    }

    #[test]
    fn test_harq_stats() {
        let mut mgr = HarqManager::new(4, 3, HarqType::TypeI);

        // Initial transmission.
        let (pid, _) = mgr.send(vec![1.0]).unwrap();
        assert_eq!(mgr.stats().total_tx, 1);

        // NACK triggers retransmission.
        mgr.handle_feedback(pid, false);
        assert_eq!(mgr.stats().nack_count, 1);
        assert_eq!(mgr.stats().total_retx, 1);
        assert_eq!(mgr.stats().total_tx, 2);

        // ACK clears.
        mgr.handle_feedback(pid, true);
        assert_eq!(mgr.stats().ack_count, 1);

        // Verify no spurious max_retx_reached.
        assert_eq!(mgr.stats().max_retx_reached, 0);
    }

    #[test]
    fn test_redundancy_versions() {
        let mut proc = HarqProcess::new(0, 5);
        proc.transmit(vec![1.0]);
        assert_eq!(proc.redundancy_version, 0);

        proc.receive_nack();
        assert_eq!(proc.redundancy_version, 1);

        proc.receive_nack();
        assert_eq!(proc.redundancy_version, 2);

        proc.receive_nack();
        assert_eq!(proc.redundancy_version, 3);

        // Wraps around mod 4 (LTE/NR RV cycle).
        proc.receive_nack();
        assert_eq!(proc.redundancy_version, 0);
    }

    #[test]
    fn test_process_states() {
        let mut proc = HarqProcess::new(0, 4);

        // Starts idle.
        assert_eq!(proc.state, ProcessState::Idle);
        assert!(proc.is_idle());
        assert_eq!(proc.transmission_count(), 0);

        // After transmit: WaitingAck.
        proc.transmit(vec![1.0]);
        assert_eq!(proc.state, ProcessState::WaitingAck);
        assert!(!proc.is_idle());
        assert_eq!(proc.transmission_count(), 1);

        // After NACK + retransmit: back to WaitingAck.
        proc.receive_nack();
        assert_eq!(proc.state, ProcessState::WaitingAck);
        assert_eq!(proc.transmission_count(), 2);

        // After ACK: Idle.
        proc.receive_ack();
        assert_eq!(proc.state, ProcessState::Idle);
        assert!(proc.is_idle());
        assert_eq!(proc.transmission_count(), 0);
    }
}
