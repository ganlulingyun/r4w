//! Trellis Metrics — Branch Metric Computation for Trellis-Based Decoding
//!
//! Computes branch metrics for Viterbi and BCJR/SISO decoders by measuring
//! the distance between received signal points and expected constellation
//! points. Supports Euclidean, squared Euclidean, Manhattan, and Hamming
//! (hard/soft) distance metrics.
//!
//! GNU Radio equivalent: `gr::trellis::metrics_x` and
//! `gr::trellis::viterbi_combined_xx`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::trellis_metrics::{TrellisMetrics, MetricType};
//! use num_complex::Complex64;
//!
//! // BPSK constellation: {-1, +1}
//! let constellation = vec![
//!     Complex64::new(-1.0, 0.0),
//!     Complex64::new(1.0, 0.0),
//! ];
//! let metrics = TrellisMetrics::new(constellation, 1, MetricType::SquaredEuclidean);
//!
//! // Received point near +1
//! let m = metrics.compute(&[Complex64::new(0.9, 0.1)]);
//! assert!(m[1] < m[0]); // closer to +1 than -1
//! ```

use num_complex::Complex64;

/// Type of distance metric.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MetricType {
    /// Euclidean distance: sqrt(sum |r_k - s_k|^2)
    Euclidean,
    /// Squared Euclidean distance: sum |r_k - s_k|^2
    SquaredEuclidean,
    /// Manhattan (L1) distance: sum (|Re(r_k - s_k)| + |Im(r_k - s_k)|)
    Manhattan,
    /// Hamming distance on hard decisions.
    HammingHard,
    /// Soft Hamming: sum of |soft_bit - reference_bit| for soft inputs.
    HammingSoft,
}

/// Branch metric calculator for trellis decoding.
#[derive(Debug, Clone)]
pub struct TrellisMetrics {
    /// Constellation points (output alphabet).
    constellation: Vec<Complex64>,
    /// Observation dimensionality (usually 1).
    dimensionality: usize,
    /// Metric type.
    metric_type: MetricType,
}

impl TrellisMetrics {
    /// Create a new trellis metric calculator.
    ///
    /// * `constellation` - Expected output symbols (one per FSM output label)
    /// * `dimensionality` - Number of complex samples per observation (usually 1)
    /// * `metric_type` - Distance metric to use
    pub fn new(
        constellation: Vec<Complex64>,
        dimensionality: usize,
        metric_type: MetricType,
    ) -> Self {
        Self {
            constellation,
            dimensionality,
            metric_type,
        }
    }

    /// Number of constellation points (output alphabet size).
    pub fn alphabet_size(&self) -> usize {
        self.constellation.len() / self.dimensionality.max(1)
    }

    /// Compute branch metrics for a single observation.
    ///
    /// Returns one metric value per constellation point (alphabet_size).
    pub fn compute(&self, observation: &[Complex64]) -> Vec<f64> {
        let dim = self.dimensionality.max(1);
        let num_symbols = self.constellation.len() / dim;
        let mut metrics = Vec::with_capacity(num_symbols);

        for s in 0..num_symbols {
            let s_start = s * dim;
            let mut dist = 0.0;
            for d in 0..dim.min(observation.len()) {
                let diff = observation[d] - self.constellation[s_start + d];
                match self.metric_type {
                    MetricType::Euclidean | MetricType::SquaredEuclidean => {
                        dist += diff.norm_sqr();
                    }
                    MetricType::Manhattan => {
                        dist += diff.re.abs() + diff.im.abs();
                    }
                    MetricType::HammingHard => {
                        // Hard decision: count mismatches
                        let rx_bit: f64 = if observation[d].re >= 0.0 { 1.0 } else { 0.0 };
                        let ref_bit: f64 = if self.constellation[s_start + d].re >= 0.0 {
                            1.0
                        } else {
                            0.0
                        };
                        if (rx_bit - ref_bit).abs() > 0.5 {
                            dist += 1.0;
                        }
                    }
                    MetricType::HammingSoft => {
                        dist += (observation[d].re - self.constellation[s_start + d].re).abs();
                    }
                }
            }
            if self.metric_type == MetricType::Euclidean {
                dist = dist.sqrt();
            }
            metrics.push(dist);
        }

        metrics
    }

    /// Compute metrics for a batch of observations.
    pub fn compute_batch(&self, observations: &[Complex64]) -> Vec<Vec<f64>> {
        let dim = self.dimensionality.max(1);
        observations
            .chunks(dim)
            .map(|chunk| self.compute(chunk))
            .collect()
    }
}

/// Combined Viterbi decoder with integrated metric computation.
///
/// Performs metric computation and Viterbi path search in one pass
/// without materializing the full metric array.
#[derive(Debug, Clone)]
pub struct ViterbiCombined {
    /// Number of states in the FSM.
    num_states: usize,
    /// State transition table: next_state[current_state][input_symbol].
    next_state: Vec<Vec<usize>>,
    /// Output table: output_symbol[current_state][input_symbol].
    output_symbol: Vec<Vec<usize>>,
    /// Number of input symbols per state.
    num_inputs: usize,
    /// Constellation for metric computation.
    constellation: Vec<Complex64>,
    /// Metric type.
    metric_type: MetricType,
    /// Traceback depth.
    traceback_depth: usize,
}

impl ViterbiCombined {
    /// Create a new combined Viterbi decoder.
    ///
    /// * `num_states` - Number of trellis states
    /// * `num_inputs` - Number of input symbols per state transition
    /// * `next_state` - State transition table [state][input] -> next_state
    /// * `output_symbol` - Output symbol table [state][input] -> constellation_index
    /// * `constellation` - Output alphabet
    /// * `metric_type` - Distance metric
    /// * `traceback_depth` - Traceback depth (typically 5 * constraint_length)
    pub fn new(
        num_states: usize,
        num_inputs: usize,
        next_state: Vec<Vec<usize>>,
        output_symbol: Vec<Vec<usize>>,
        constellation: Vec<Complex64>,
        metric_type: MetricType,
        traceback_depth: usize,
    ) -> Self {
        Self {
            num_states,
            next_state,
            output_symbol,
            num_inputs,
            constellation,
            metric_type,
            traceback_depth,
        }
    }

    /// Decode received IQ samples to output symbols using Viterbi algorithm.
    pub fn decode(&mut self, received: &[Complex64]) -> Vec<usize> {
        if received.is_empty() {
            return vec![];
        }

        let tm = TrellisMetrics::new(
            self.constellation.clone(),
            1,
            self.metric_type,
        );

        let n = received.len();

        // Path metrics
        let mut path_metric = vec![f64::INFINITY; self.num_states];
        path_metric[0] = 0.0; // Start in state 0

        // Traceback storage
        let mut traceback: Vec<Vec<(usize, usize)>> = Vec::with_capacity(n);

        for t in 0..n {
            let branch_metrics = tm.compute(&[received[t]]);
            let mut new_metric = vec![f64::INFINITY; self.num_states];
            let mut tb_entry = vec![(0usize, 0usize); self.num_states];

            for s in 0..self.num_states {
                if path_metric[s] == f64::INFINITY {
                    continue;
                }
                for i in 0..self.num_inputs {
                    if s >= self.next_state.len() || i >= self.next_state[s].len() {
                        continue;
                    }
                    let ns = self.next_state[s][i];
                    let out = self.output_symbol[s][i];
                    let bm = if out < branch_metrics.len() {
                        branch_metrics[out]
                    } else {
                        f64::INFINITY
                    };
                    let total = path_metric[s] + bm;
                    if total < new_metric[ns] {
                        new_metric[ns] = total;
                        tb_entry[ns] = (s, i);
                    }
                }
            }

            path_metric = new_metric;
            traceback.push(tb_entry);
        }

        // Traceback from best final state
        let mut state = path_metric
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, _)| i)
            .unwrap_or(0);

        let mut decoded = vec![0usize; n];
        for t in (0..n).rev() {
            let (prev_state, input) = traceback[t][state];
            decoded[t] = input;
            state = prev_state;
        }

        decoded
    }
}

/// Compute soft LLR values from branch metrics for iterative decoding.
pub fn metrics_to_llr(metrics_0: f64, metrics_1: f64) -> f64 {
    // LLR = ln(P(bit=0)/P(bit=1)) ≈ -(d_0^2 - d_1^2) / (2*sigma^2)
    // Simplified: just use metric difference
    metrics_1 - metrics_0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bpsk_constellation() -> Vec<Complex64> {
        vec![Complex64::new(-1.0, 0.0), Complex64::new(1.0, 0.0)]
    }

    fn qpsk_constellation() -> Vec<Complex64> {
        let s = std::f64::consts::FRAC_1_SQRT_2;
        vec![
            Complex64::new(s, s),
            Complex64::new(-s, s),
            Complex64::new(-s, -s),
            Complex64::new(s, -s),
        ]
    }

    #[test]
    fn test_euclidean_bpsk() {
        let tm = TrellisMetrics::new(bpsk_constellation(), 1, MetricType::Euclidean);
        let m = tm.compute(&[Complex64::new(0.9, 0.0)]);
        assert_eq!(m.len(), 2);
        assert!(m[1] < m[0], "closer to +1: m0={}, m1={}", m[0], m[1]);
    }

    #[test]
    fn test_squared_euclidean_qpsk() {
        let tm = TrellisMetrics::new(qpsk_constellation(), 1, MetricType::SquaredEuclidean);
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let m = tm.compute(&[Complex64::new(s + 0.1, s + 0.1)]);
        assert_eq!(m.len(), 4);
        // Should be closest to point 0 (s, s)
        let min_idx = m
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert_eq!(min_idx, 0);
    }

    #[test]
    fn test_manhattan_metric() {
        let tm = TrellisMetrics::new(bpsk_constellation(), 1, MetricType::Manhattan);
        let m = tm.compute(&[Complex64::new(0.5, 0.3)]);
        assert_eq!(m.len(), 2);
        // Distance to -1: |0.5-(-1)| + |0.3-0| = 1.5 + 0.3 = 1.8
        // Distance to +1: |0.5-1| + |0.3-0| = 0.5 + 0.3 = 0.8
        assert!((m[0] - 1.8).abs() < 1e-10);
        assert!((m[1] - 0.8).abs() < 1e-10);
    }

    #[test]
    fn test_zero_distance() {
        let tm = TrellisMetrics::new(bpsk_constellation(), 1, MetricType::SquaredEuclidean);
        let m = tm.compute(&[Complex64::new(1.0, 0.0)]);
        assert!((m[1]).abs() < 1e-20, "exact match should give 0 metric");
    }

    #[test]
    fn test_batch_computation() {
        let tm = TrellisMetrics::new(bpsk_constellation(), 1, MetricType::SquaredEuclidean);
        let observations = vec![
            Complex64::new(0.8, 0.0),
            Complex64::new(-0.7, 0.0),
            Complex64::new(0.9, 0.1),
        ];
        let batch = tm.compute_batch(&observations);
        assert_eq!(batch.len(), 3);
        assert!(batch[0][1] < batch[0][0]); // near +1
        assert!(batch[1][0] < batch[1][1]); // near -1
        assert!(batch[2][1] < batch[2][0]); // near +1
    }

    #[test]
    fn test_hamming_hard() {
        let tm = TrellisMetrics::new(bpsk_constellation(), 1, MetricType::HammingHard);
        // Positive received → bit=1, compared to -1 (bit=0) → distance 1
        let m = tm.compute(&[Complex64::new(0.5, 0.0)]);
        assert!((m[0] - 1.0).abs() < 1e-10); // mismatch with -1
        assert!((m[1] - 0.0).abs() < 1e-10); // match with +1
    }

    #[test]
    fn test_hamming_soft() {
        let tm = TrellisMetrics::new(bpsk_constellation(), 1, MetricType::HammingSoft);
        let m = tm.compute(&[Complex64::new(0.5, 0.0)]);
        // |0.5 - (-1)| = 1.5
        // |0.5 - 1| = 0.5
        assert!((m[0] - 1.5).abs() < 1e-10);
        assert!((m[1] - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_viterbi_combined_simple() {
        // Simple rate-1/2 convolutional code: 2 states, 2 inputs
        // State 0: input 0 → state 0, output 0 (symbol 0 = -1)
        //          input 1 → state 1, output 1 (symbol 1 = +1)
        // State 1: input 0 → state 0, output 1
        //          input 1 → state 1, output 0
        let next_state = vec![vec![0, 1], vec![0, 1]];
        let output_symbol = vec![vec![0, 1], vec![1, 0]];
        let constellation = bpsk_constellation();

        let mut vit = ViterbiCombined::new(
            2,
            2,
            next_state.clone(),
            output_symbol.clone(),
            constellation.clone(),
            MetricType::SquaredEuclidean,
            10,
        );

        // Encode: inputs [1, 0, 1, 0] through trellis
        // State 0, input 1 → state 1, output 1 (+1)
        // State 1, input 0 → state 0, output 1 (+1)
        // State 0, input 1 → state 1, output 1 (+1)
        // State 1, input 0 → state 0, output 1 (+1)
        let received = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(1.0, 0.0),
            Complex64::new(1.0, 0.0),
            Complex64::new(1.0, 0.0),
        ];
        let decoded = vit.decode(&received);
        assert_eq!(decoded.len(), 4);
        assert_eq!(decoded[0], 1);
        assert_eq!(decoded[1], 0);
    }

    #[test]
    fn test_metrics_to_llr() {
        let llr = metrics_to_llr(0.1, 2.0);
        assert!(llr > 0.0, "bit 0 closer → positive LLR");
        let llr2 = metrics_to_llr(2.0, 0.1);
        assert!(llr2 < 0.0, "bit 1 closer → negative LLR");
    }

    #[test]
    fn test_alphabet_size() {
        let tm = TrellisMetrics::new(qpsk_constellation(), 1, MetricType::Euclidean);
        assert_eq!(tm.alphabet_size(), 4);

        let tm2 = TrellisMetrics::new(
            vec![Complex64::new(0.0, 0.0); 6],
            2,
            MetricType::Euclidean,
        );
        assert_eq!(tm2.alphabet_size(), 3); // 6 / 2
    }
}
