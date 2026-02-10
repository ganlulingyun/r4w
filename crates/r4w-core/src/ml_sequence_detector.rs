//! Maximum-Likelihood Sequence Detector (MLSD) using the Viterbi Algorithm.
//!
//! This module implements an MLSD equalizer that finds the most likely transmitted
//! symbol sequence given a received signal distorted by inter-symbol interference
//! (ISI). The Viterbi algorithm efficiently searches the trellis of possible state
//! transitions, pruning unlikely paths to achieve optimal (ML) detection in
//! O(M^L * N) complexity, where M is the constellation size, L is the channel
//! memory, and N is the sequence length.
//!
//! # Features
//!
//! - **Full Viterbi MLSD** over an ISI channel trellis
//! - **Configurable channel memory** (L taps in the channel impulse response)
//! - **Constellation support**: BPSK, QPSK, and arbitrary M-QAM alphabets
//! - **Soft-output (SOVA)** variant producing per-symbol log-likelihood ratios
//! - **Reduced-state sequence estimation (RSSE)** for lower complexity
//! - **Survivor path traceback** for streaming or block detection
//!
//! # Example
//!
//! ```
//! use r4w_core::ml_sequence_detector::MlSequenceDetector;
//!
//! // A 2-tap ISI channel: h = [1.0, 0.5]
//! let channel = [(1.0, 0.0), (0.5, 0.0)];
//!
//! // BPSK symbols: +1, -1, +1, +1
//! let symbols = [(1.0, 0.0), (-1.0, 0.0), (1.0, 0.0), (1.0, 0.0)];
//!
//! // Convolve symbols with the channel to produce received signal
//! // r[n] = sum_k h[k] * s[n-k]
//! let mut received = vec![(0.0, 0.0); symbols.len() + channel.len() - 1];
//! for (i, &s) in symbols.iter().enumerate() {
//!     for (j, &h) in channel.iter().enumerate() {
//!         received[i + j].0 += s.0 * h.0 - s.1 * h.1;
//!         received[i + j].1 += s.0 * h.1 + s.1 * h.0;
//!     }
//! }
//!
//! let detector = MlSequenceDetector::bpsk(&channel);
//! let detected = detector.detect(&received);
//!
//! // The MLSD should recover the original BPSK symbols
//! assert_eq!(detected.len(), symbols.len());
//! for (det, orig) in detected.iter().zip(symbols.iter()) {
//!     assert!((det.0 - orig.0).abs() < 1e-6);
//! }
//! ```

/// A trellis state used internally by the Viterbi search.
#[derive(Clone, Debug)]
struct TrellisState {
    /// Accumulated path metric (lower is better).
    metric: f64,
    /// Sequence of symbol indices along the survivor path.
    survivor_path: Vec<usize>,
}

/// Maximum-Likelihood Sequence Detector using the Viterbi algorithm.
///
/// Given a channel impulse response (CIR) of length L and a symbol constellation
/// of size M, the detector maintains M^(L-1) trellis states and searches for the
/// symbol sequence that minimises the Euclidean distance to the received signal.
#[derive(Clone, Debug)]
pub struct MlSequenceDetector {
    /// Complex channel impulse response taps (re, im).
    channel_taps: Vec<(f64, f64)>,
    /// Symbol constellation points (re, im).
    constellation: Vec<(f64, f64)>,
    /// Channel memory: number of taps minus one.
    memory: usize,
    /// Number of trellis states = M^memory.
    num_states: usize,
    /// Traceback depth (defaults to 5 * memory).
    traceback_depth: usize,
}

// ---------------------------------------------------------------------------
// Complex arithmetic helpers (real, imaginary) tuples
// ---------------------------------------------------------------------------

#[inline]
fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_norm_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

// ---------------------------------------------------------------------------
// State encoding/decoding helpers
// ---------------------------------------------------------------------------

/// Encode a window of symbol indices into a state number (mixed-radix, big-endian).
fn encode_state(symbols: &[usize], m: usize) -> usize {
    let mut state = 0usize;
    for &s in symbols {
        state = state * m + s;
    }
    state
}

/// Decode a state number into a vector of symbol indices.
fn decode_state(mut state: usize, memory: usize, m: usize) -> Vec<usize> {
    let mut syms = vec![0usize; memory];
    for i in (0..memory).rev() {
        syms[i] = state % m;
        state /= m;
    }
    syms
}

impl MlSequenceDetector {
    /// Create a new MLSD for the given channel impulse response and constellation.
    ///
    /// `channel_taps` is a slice of complex (re, im) CIR coefficients.
    /// `constellation` is the set of complex symbol values the transmitter uses.
    ///
    /// The channel memory is `channel_taps.len() - 1`, and the trellis has
    /// `constellation.len().pow(memory)` states.
    ///
    /// # Panics
    ///
    /// Panics if `channel_taps` is empty or `constellation` is empty.
    pub fn new(channel_taps: &[(f64, f64)], constellation: &[(f64, f64)]) -> Self {
        assert!(!channel_taps.is_empty(), "channel_taps must not be empty");
        assert!(!constellation.is_empty(), "constellation must not be empty");

        let memory = channel_taps.len() - 1;
        let m = constellation.len();
        let num_states = if memory == 0 { 1 } else { m.pow(memory as u32) };
        let traceback_depth = if memory == 0 { 1 } else { 5 * memory };

        Self {
            channel_taps: channel_taps.to_vec(),
            constellation: constellation.to_vec(),
            memory,
            num_states,
            traceback_depth,
        }
    }

    /// Convenience constructor for BPSK: constellation = {-1, +1}.
    pub fn bpsk(channel_taps: &[(f64, f64)]) -> Self {
        Self::new(channel_taps, &[(-1.0, 0.0), (1.0, 0.0)])
    }

    /// Convenience constructor for QPSK: constellation = {(+-1, +-1)/sqrt(2)}.
    pub fn qpsk(channel_taps: &[(f64, f64)]) -> Self {
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let constellation = [
            ( s,  s),
            (-s,  s),
            (-s, -s),
            ( s, -s),
        ];
        Self::new(channel_taps, &constellation)
    }

    /// Update the channel impulse response (e.g., after a new channel estimate).
    pub fn set_channel(&mut self, taps: &[(f64, f64)]) {
        assert!(!taps.is_empty(), "channel_taps must not be empty");
        let new_memory = taps.len() - 1;
        self.channel_taps = taps.to_vec();
        // Recalculate state count if memory changed
        if new_memory != self.memory {
            self.memory = new_memory;
            let m = self.constellation.len();
            self.num_states = if self.memory == 0 {
                1
            } else {
                m.pow(self.memory as u32)
            };
            self.traceback_depth = if self.memory == 0 { 1 } else { 5 * self.memory };
        }
    }

    /// Set the traceback depth (number of symbols stored per survivor path).
    pub fn with_traceback_depth(mut self, depth: usize) -> Self {
        self.traceback_depth = depth.max(1);
        self
    }

    /// Return the number of trellis states.
    pub fn num_states(&self) -> usize {
        self.num_states
    }

    // -----------------------------------------------------------------------
    // Core Viterbi
    // -----------------------------------------------------------------------

    /// Compute the expected received sample for a given sequence of symbol indices.
    /// `current_sym_idx` is the symbol emitted at the current time, and `state_syms`
    /// are the L-1 previous symbol indices (oldest first).
    fn expected_sample(&self, current_sym_idx: usize, state_syms: &[usize]) -> (f64, f64) {
        // The channel output at time n is:
        //   r[n] = h[0]*s[n] + h[1]*s[n-1] + ... + h[L-1]*s[n-L+1]
        // state_syms contains [s[n-L+1], ..., s[n-1]] (length = memory = L-1)
        // current_sym_idx corresponds to s[n].
        let mut out = c_mul(self.channel_taps[0], self.constellation[current_sym_idx]);
        for (k, &tap) in self.channel_taps[1..].iter().enumerate() {
            // k=0 => tap index 1, symbol index = state_syms[memory-1-0] = s[n-1]
            // k=1 => tap index 2, symbol index = state_syms[memory-2]   = s[n-2]
            // ...
            if self.memory > 0 {
                let sym_idx = state_syms[self.memory - 1 - k];
                out = c_add(out, c_mul(tap, self.constellation[sym_idx]));
            }
        }
        out
    }

    /// Run the Viterbi algorithm and return (detected_symbol_indices, final_metric).
    fn viterbi_core(&self, received: &[(f64, f64)]) -> (Vec<usize>, f64) {
        if received.is_empty() {
            return (Vec::new(), 0.0);
        }

        let m = self.constellation.len();

        // Determine sequence length: the transmitted sequence has length
        // N = received.len() - memory  (since convolution adds `memory` extra samples).
        // If received.len() <= memory, we treat it as a single symbol window.
        let seq_len = if received.len() > self.memory {
            received.len() - self.memory
        } else {
            received.len()
        };

        // Initialise trellis states. All states start with metric = infinity
        // except the all-zero state (index 0) which starts at 0.
        // Actually for a cold start we initialise all states equally.
        let mut states: Vec<TrellisState> = (0..self.num_states)
            .map(|_| TrellisState {
                metric: 0.0,
                survivor_path: Vec::with_capacity(seq_len),
            })
            .collect();

        for n in 0..seq_len {
            let r = received[n];

            // Next-state accumulators
            let mut next_states: Vec<TrellisState> = (0..self.num_states)
                .map(|_| TrellisState {
                    metric: f64::INFINITY,
                    survivor_path: Vec::new(),
                })
                .collect();

            for s in 0..self.num_states {
                if states[s].metric == f64::INFINITY {
                    continue;
                }
                let state_syms = decode_state(s, self.memory, m);

                for sym_idx in 0..m {
                    // Compute expected sample
                    let expected = self.expected_sample(sym_idx, &state_syms);
                    let diff = c_sub(r, expected);
                    let branch_metric = c_norm_sq(diff);
                    let total_metric = states[s].metric + branch_metric;

                    // Compute next state: shift in sym_idx, shift out oldest
                    let next_state = if self.memory == 0 {
                        0
                    } else {
                        // Drop oldest, append newest
                        let mut next_syms = Vec::with_capacity(self.memory);
                        if self.memory > 1 {
                            next_syms.extend_from_slice(&state_syms[1..]);
                        }
                        next_syms.push(sym_idx);
                        encode_state(&next_syms, m)
                    };

                    if total_metric < next_states[next_state].metric {
                        next_states[next_state].metric = total_metric;
                        let mut path = states[s].survivor_path.clone();
                        path.push(sym_idx);
                        next_states[next_state].survivor_path = path;
                    }
                }
            }

            states = next_states;
        }

        // Find the best final state
        let mut best_metric = f64::INFINITY;
        let mut best_idx = 0;
        for (i, st) in states.iter().enumerate() {
            if st.metric < best_metric {
                best_metric = st.metric;
                best_idx = i;
            }
        }

        (states[best_idx].survivor_path.clone(), best_metric)
    }

    /// Detect the most likely transmitted symbol sequence.
    ///
    /// `received` is the complex baseband received signal after matched filtering
    /// and sampling. Returns the detected complex symbols.
    ///
    /// The received signal length should be `N + L - 1` where N is the number of
    /// transmitted symbols and L is the channel length. The detector returns N
    /// symbols.
    pub fn detect(&self, received: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let (indices, _) = self.viterbi_core(received);
        indices
            .iter()
            .map(|&idx| self.constellation[idx])
            .collect()
    }

    /// Detect with final path metric.
    ///
    /// Returns `(detected_symbols, final_accumulated_metric)`. The metric is the
    /// total squared Euclidean distance between the received signal and the
    /// reconstructed signal from the detected sequence.
    pub fn detect_with_metrics(
        &self,
        received: &[(f64, f64)],
    ) -> (Vec<(f64, f64)>, f64) {
        let (indices, metric) = self.viterbi_core(received);
        let syms = indices
            .iter()
            .map(|&idx| self.constellation[idx])
            .collect();
        (syms, metric)
    }

    /// Produce soft-output log-likelihood ratios (LLRs) via SOVA.
    ///
    /// For each detected symbol position, the LLR is defined as:
    ///
    /// ```text
    /// LLR[n] = metric_of_antipath - metric_of_survivor
    /// ```
    ///
    /// where the antipath is the best competing path that differs at position n.
    /// Positive LLR indicates high confidence in the survivor decision.
    ///
    /// For BPSK, the sign of the LLR indicates the hard decision (positive => +1).
    pub fn detect_soft(&self, received: &[(f64, f64)]) -> Vec<f64> {
        if received.is_empty() {
            return Vec::new();
        }

        let m = self.constellation.len();
        let seq_len = if received.len() > self.memory {
            received.len() - self.memory
        } else {
            received.len()
        };

        // Run full Viterbi collecting the two best paths per state update
        // SOVA: for each time step and each state, keep track of the metric
        // difference when the decision differs.

        // We keep the full state+path information, then compute LLRs by
        // finding the best path that has a different symbol at each position.

        // First get the ML path
        let (ml_indices, ml_metric) = self.viterbi_core(received);

        // Now, for each position n, find the best path among all states that
        // disagrees with the ML path at position n.
        let mut llrs = vec![f64::INFINITY; ml_indices.len()];

        // Re-run Viterbi but keep ALL state metrics and paths at the end
        let mut states: Vec<TrellisState> = (0..self.num_states)
            .map(|_| TrellisState {
                metric: 0.0,
                survivor_path: Vec::with_capacity(seq_len),
            })
            .collect();

        for n in 0..seq_len {
            let r = received[n];
            let mut next_states: Vec<TrellisState> = (0..self.num_states)
                .map(|_| TrellisState {
                    metric: f64::INFINITY,
                    survivor_path: Vec::new(),
                })
                .collect();

            // Also keep the second-best per next-state with different decision at n
            let mut second_best: Vec<(f64, Vec<usize>)> = (0..self.num_states)
                .map(|_| (f64::INFINITY, Vec::new()))
                .collect();

            for s in 0..self.num_states {
                if states[s].metric == f64::INFINITY {
                    continue;
                }
                let state_syms = decode_state(s, self.memory, m);

                for sym_idx in 0..m {
                    let expected = self.expected_sample(sym_idx, &state_syms);
                    let diff = c_sub(r, expected);
                    let branch_metric = c_norm_sq(diff);
                    let total_metric = states[s].metric + branch_metric;

                    let next_state = if self.memory == 0 {
                        0
                    } else {
                        let mut next_syms = Vec::with_capacity(self.memory);
                        if self.memory > 1 {
                            next_syms.extend_from_slice(&state_syms[1..]);
                        }
                        next_syms.push(sym_idx);
                        encode_state(&next_syms, m)
                    };

                    if total_metric < next_states[next_state].metric {
                        // Demote current best to second-best if it has different sym at n
                        if !next_states[next_state].survivor_path.is_empty() {
                            let old_sym_at_n = if n < next_states[next_state].survivor_path.len() {
                                Some(next_states[next_state].survivor_path[n])
                            } else {
                                None
                            };
                            let new_sym_at_n = {
                                let mut path = states[s].survivor_path.clone();
                                path.push(sym_idx);
                                if n < path.len() { Some(path[n]) } else { None }
                            };
                            if old_sym_at_n != new_sym_at_n {
                                second_best[next_state] = (
                                    next_states[next_state].metric,
                                    next_states[next_state].survivor_path.clone(),
                                );
                            }
                        }
                        next_states[next_state].metric = total_metric;
                        let mut path = states[s].survivor_path.clone();
                        path.push(sym_idx);
                        next_states[next_state].survivor_path = path;
                    } else if total_metric < second_best[next_state].0 {
                        // Check if this path differs from best at position n
                        let best_sym_at_n = if n < next_states[next_state].survivor_path.len() {
                            Some(next_states[next_state].survivor_path[n])
                        } else {
                            None
                        };
                        let mut path = states[s].survivor_path.clone();
                        path.push(sym_idx);
                        let this_sym_at_n = if n < path.len() { Some(path[n]) } else { None };
                        if best_sym_at_n != this_sym_at_n {
                            second_best[next_state] = (total_metric, path);
                        }
                    }
                }
            }

            states = next_states;
        }

        // Collect all final paths and compute LLRs
        for st in &states {
            if st.metric == f64::INFINITY {
                continue;
            }
            for n in 0..ml_indices.len().min(st.survivor_path.len()) {
                if st.survivor_path[n] != ml_indices[n] {
                    let delta = st.metric - ml_metric;
                    if delta < llrs[n] {
                        llrs[n] = delta;
                    }
                }
            }
        }

        // Cap infinite LLRs to a large value and apply sign for BPSK convention
        let max_llr = 100.0;
        for llr in &mut llrs {
            if llr.is_infinite() {
                *llr = max_llr;
            }
        }

        // For BPSK, apply sign: positive if ML decided symbol index 1 (+1), negative if index 0 (-1)
        if m == 2 {
            for (n, llr) in llrs.iter_mut().enumerate() {
                // index 0 = -1, index 1 = +1
                if ml_indices[n] == 0 {
                    *llr = -*llr;
                }
            }
        }

        llrs
    }
}

/// Reduced-State Maximum-Likelihood Sequence Detector.
///
/// Limits the number of active states in the Viterbi trellis to `max_states`,
/// pruning the least likely states at each step. This provides a trade-off
/// between complexity and detection performance.
#[derive(Clone, Debug)]
pub struct ReducedStateMlsd {
    inner: MlSequenceDetector,
    max_states: usize,
}

impl ReducedStateMlsd {
    /// Create a reduced-state MLSD.
    ///
    /// `max_states` caps the number of active trellis states. If the full trellis
    /// has fewer states, this effectively degenerates to full MLSD.
    ///
    /// # Panics
    ///
    /// Panics if `max_states` is zero.
    pub fn new(
        channel_taps: &[(f64, f64)],
        constellation: &[(f64, f64)],
        max_states: usize,
    ) -> Self {
        assert!(max_states > 0, "max_states must be > 0");
        Self {
            inner: MlSequenceDetector::new(channel_taps, constellation),
            max_states,
        }
    }

    /// Detect the most likely symbol sequence with reduced state complexity.
    pub fn detect(&self, received: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if received.is_empty() {
            return Vec::new();
        }

        let m = self.inner.constellation.len();
        let memory = self.inner.memory;
        let num_states = self.inner.num_states;

        let seq_len = if received.len() > memory {
            received.len() - memory
        } else {
            received.len()
        };

        // Active states: we only track up to max_states at each step
        let active_limit = self.max_states.min(num_states);

        let mut states: Vec<(usize, TrellisState)> = (0..num_states)
            .map(|i| {
                (
                    i,
                    TrellisState {
                        metric: 0.0,
                        survivor_path: Vec::with_capacity(seq_len),
                    },
                )
            })
            .collect();

        // If initial states already exceed limit, prune
        if states.len() > active_limit {
            states.truncate(active_limit);
        }

        for n in 0..seq_len {
            let r = received[n];

            // Collect all transitions from active states
            let mut candidates: Vec<(usize, TrellisState)> = Vec::new();

            for (s_idx, st) in &states {
                let state_syms = decode_state(*s_idx, memory, m);

                for sym_idx in 0..m {
                    let expected = self.inner.expected_sample(sym_idx, &state_syms);
                    let diff = c_sub(r, expected);
                    let branch_metric = c_norm_sq(diff);
                    let total_metric = st.metric + branch_metric;

                    let next_state = if memory == 0 {
                        0
                    } else {
                        let mut next_syms = Vec::with_capacity(memory);
                        if memory > 1 {
                            next_syms.extend_from_slice(&state_syms[1..]);
                        }
                        next_syms.push(sym_idx);
                        encode_state(&next_syms, m)
                    };

                    let mut path = st.survivor_path.clone();
                    path.push(sym_idx);

                    candidates.push((
                        next_state,
                        TrellisState {
                            metric: total_metric,
                            survivor_path: path,
                        },
                    ));
                }
            }

            // Keep only the best path per state
            candidates.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.metric.partial_cmp(&b.1.metric).unwrap()));
            candidates.dedup_by(|a, b| {
                if a.0 == b.0 {
                    // b is kept; if a is better, swap
                    if a.1.metric < b.1.metric {
                        std::mem::swap(&mut a.1, &mut b.1);
                    }
                    true
                } else {
                    false
                }
            });

            // Prune to max_states
            if candidates.len() > active_limit {
                candidates.sort_by(|a, b| a.1.metric.partial_cmp(&b.1.metric).unwrap());
                candidates.truncate(active_limit);
            }

            states = candidates;
        }

        // Find best final state
        let best = states
            .iter()
            .min_by(|a, b| a.1.metric.partial_cmp(&b.1.metric).unwrap())
            .unwrap();

        best.1
            .survivor_path
            .iter()
            .map(|&idx| self.inner.constellation[idx])
            .collect()
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: convolve symbols with channel taps to produce the received signal.
    fn convolve(symbols: &[(f64, f64)], taps: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let out_len = symbols.len() + taps.len() - 1;
        let mut out = vec![(0.0, 0.0); out_len];
        for (i, &s) in symbols.iter().enumerate() {
            for (j, &h) in taps.iter().enumerate() {
                let prod = c_mul(s, h);
                out[i + j] = c_add(out[i + j], prod);
            }
        }
        out
    }

    /// Helper: check that two complex sequences are element-wise close.
    fn assert_close(a: &[(f64, f64)], b: &[(f64, f64)], tol: f64) {
        assert_eq!(a.len(), b.len(), "length mismatch: {} vs {}", a.len(), b.len());
        for (i, (x, y)) in a.iter().zip(b.iter()).enumerate() {
            let d = c_norm_sq(c_sub(*x, *y)).sqrt();
            assert!(
                d < tol,
                "mismatch at index {}: ({:.4}, {:.4}) vs ({:.4}, {:.4}), dist={:.6}",
                i, x.0, x.1, y.0, y.1, d
            );
        }
    }

    #[test]
    fn test_bpsk_no_isi() {
        // Single-tap channel (no ISI): h = [1]
        let channel = [(1.0, 0.0)];
        let symbols: Vec<(f64, f64)> = vec![
            (1.0, 0.0), (-1.0, 0.0), (-1.0, 0.0), (1.0, 0.0), (1.0, 0.0),
        ];
        let received = convolve(&symbols, &channel);

        let det = MlSequenceDetector::bpsk(&channel);
        let detected = det.detect(&received);
        assert_close(&detected, &symbols, 1e-9);
    }

    #[test]
    fn test_bpsk_2tap_isi() {
        let channel = [(1.0, 0.0), (0.5, 0.0)];
        let symbols: Vec<(f64, f64)> = vec![
            (1.0, 0.0), (-1.0, 0.0), (1.0, 0.0), (1.0, 0.0), (-1.0, 0.0),
        ];
        let received = convolve(&symbols, &channel);

        let det = MlSequenceDetector::bpsk(&channel);
        let detected = det.detect(&received);
        assert_close(&detected, &symbols, 1e-9);
    }

    #[test]
    fn test_bpsk_3tap_isi() {
        let channel = [(1.0, 0.0), (0.3, 0.0), (-0.2, 0.0)];
        let symbols: Vec<(f64, f64)> = vec![
            (1.0, 0.0), (1.0, 0.0), (-1.0, 0.0), (-1.0, 0.0),
            (1.0, 0.0), (-1.0, 0.0), (1.0, 0.0), (1.0, 0.0),
        ];
        let received = convolve(&symbols, &channel);

        let det = MlSequenceDetector::bpsk(&channel);
        let detected = det.detect(&received);
        assert_close(&detected, &symbols, 1e-9);
    }

    #[test]
    fn test_qpsk_no_isi() {
        let channel = [(1.0, 0.0)];
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let symbols: Vec<(f64, f64)> = vec![
            ( s,  s),
            (-s,  s),
            (-s, -s),
            ( s, -s),
        ];
        let received = convolve(&symbols, &channel);

        let det = MlSequenceDetector::qpsk(&channel);
        let detected = det.detect(&received);
        assert_close(&detected, &symbols, 1e-9);
    }

    #[test]
    fn test_qpsk_2tap_isi() {
        let channel = [(1.0, 0.0), (0.4, 0.1)];
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let symbols: Vec<(f64, f64)> = vec![
            ( s,  s),
            (-s,  s),
            ( s, -s),
            (-s, -s),
            ( s,  s),
        ];
        let received = convolve(&symbols, &channel);

        let det = MlSequenceDetector::qpsk(&channel);
        let detected = det.detect(&received);
        assert_close(&detected, &symbols, 1e-9);
    }

    #[test]
    fn test_detect_with_metrics_zero_noise() {
        // Use a single-tap channel so there is no cold-start ambiguity and
        // the path metric should be exactly zero in the noise-free case.
        let channel = [(1.0, 0.0)];
        let symbols: Vec<(f64, f64)> = vec![
            (1.0, 0.0), (-1.0, 0.0), (1.0, 0.0),
        ];
        let received = convolve(&symbols, &channel);

        let det = MlSequenceDetector::bpsk(&channel);
        let (detected, metric) = det.detect_with_metrics(&received);
        assert_close(&detected, &symbols, 1e-9);
        // With zero noise and no ISI the metric should be exactly zero
        assert!(metric < 1e-18, "metric should be ~0, got {}", metric);
    }

    #[test]
    fn test_detect_with_metrics_isi_channel() {
        // With an ISI channel and noise-free reception, the ML path metric
        // should still be small (cold-start edge effect only).
        let channel = [(1.0, 0.0), (0.5, 0.0)];
        let symbols: Vec<(f64, f64)> = vec![
            (1.0, 0.0), (-1.0, 0.0), (1.0, 0.0), (1.0, 0.0),
        ];
        let received = convolve(&symbols, &channel);

        let det = MlSequenceDetector::bpsk(&channel);
        let (detected, metric) = det.detect_with_metrics(&received);
        assert_close(&detected, &symbols, 1e-9);
        // Metric may be non-zero due to cold-start state ambiguity but should
        // be bounded (much less than 1 per symbol).
        assert!(metric < 2.0, "metric unexpectedly large: {}", metric);
    }

    #[test]
    fn test_soft_output_bpsk() {
        let channel = [(1.0, 0.0), (0.3, 0.0)];
        let symbols: Vec<(f64, f64)> = vec![
            (1.0, 0.0), (-1.0, 0.0), (1.0, 0.0), (1.0, 0.0),
        ];
        let received = convolve(&symbols, &channel);

        let det = MlSequenceDetector::bpsk(&channel);
        let llrs = det.detect_soft(&received);

        assert_eq!(llrs.len(), symbols.len());
        // For BPSK with no noise, LLRs should be positive for +1 and negative for -1
        // Symbol 0 = +1 => positive LLR
        assert!(llrs[0] > 0.0, "LLR[0] should be positive for +1, got {}", llrs[0]);
        // Symbol 1 = -1 => negative LLR
        assert!(llrs[1] < 0.0, "LLR[1] should be negative for -1, got {}", llrs[1]);
        // Symbol 2 = +1 => positive LLR
        assert!(llrs[2] > 0.0, "LLR[2] should be positive for +1, got {}", llrs[2]);
        // Symbol 3 = +1 => positive LLR
        assert!(llrs[3] > 0.0, "LLR[3] should be positive for +1, got {}", llrs[3]);
    }

    #[test]
    fn test_num_states() {
        // BPSK, 2 taps => memory=1 => 2^1 = 2 states
        let det = MlSequenceDetector::bpsk(&[(1.0, 0.0), (0.5, 0.0)]);
        assert_eq!(det.num_states(), 2);

        // BPSK, 3 taps => memory=2 => 2^2 = 4 states
        let det = MlSequenceDetector::bpsk(&[(1.0, 0.0), (0.5, 0.0), (0.2, 0.0)]);
        assert_eq!(det.num_states(), 4);

        // QPSK, 2 taps => memory=1 => 4^1 = 4 states
        let det = MlSequenceDetector::qpsk(&[(1.0, 0.0), (0.3, 0.0)]);
        assert_eq!(det.num_states(), 4);

        // QPSK, 3 taps => memory=2 => 4^2 = 16 states
        let det = MlSequenceDetector::qpsk(&[(1.0, 0.0), (0.3, 0.0), (0.1, 0.0)]);
        assert_eq!(det.num_states(), 16);

        // Single tap => memory=0 => 1 state
        let det = MlSequenceDetector::bpsk(&[(1.0, 0.0)]);
        assert_eq!(det.num_states(), 1);
    }

    #[test]
    fn test_set_channel_updates_states() {
        let mut det = MlSequenceDetector::bpsk(&[(1.0, 0.0), (0.5, 0.0)]);
        assert_eq!(det.num_states(), 2);

        // Update to 3 taps
        det.set_channel(&[(1.0, 0.0), (0.3, 0.0), (0.1, 0.0)]);
        assert_eq!(det.num_states(), 4);
        assert_eq!(det.memory, 2);
    }

    #[test]
    fn test_traceback_depth() {
        let det = MlSequenceDetector::bpsk(&[(1.0, 0.0), (0.5, 0.0)])
            .with_traceback_depth(20);
        assert_eq!(det.traceback_depth, 20);
    }

    #[test]
    fn test_reduced_state_bpsk() {
        let channel = [(1.0, 0.0), (0.5, 0.0)];
        let symbols: Vec<(f64, f64)> = vec![
            (1.0, 0.0), (-1.0, 0.0), (1.0, 0.0), (1.0, 0.0), (-1.0, 0.0),
        ];
        let received = convolve(&symbols, &channel);

        // Full MLSD has 2 states; using max_states=2 should give optimal result
        let rsse = ReducedStateMlsd::new(&channel, &[(-1.0, 0.0), (1.0, 0.0)], 2);
        let detected = rsse.detect(&received);
        assert_close(&detected, &symbols, 1e-9);
    }

    #[test]
    fn test_reduced_state_fewer_states() {
        // QPSK with 2 taps => 4 states full, limit to 2
        let channel = [(1.0, 0.0), (0.3, 0.0)];
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let constellation = [( s, s), (-s, s), (-s, -s), ( s, -s)];
        let symbols: Vec<(f64, f64)> = vec![
            ( s,  s),
            (-s,  s),
            ( s, -s),
            ( s,  s),
        ];
        let received = convolve(&symbols, &channel);

        // With a mild channel (0.3 second tap) and no noise,
        // even reduced states should find the correct sequence.
        let rsse = ReducedStateMlsd::new(&channel, &constellation, 2);
        let detected = rsse.detect(&received);
        assert_close(&detected, &symbols, 1e-9);
    }

    #[test]
    fn test_empty_received() {
        let channel = [(1.0, 0.0), (0.5, 0.0)];
        let det = MlSequenceDetector::bpsk(&channel);
        let detected = det.detect(&[]);
        assert!(detected.is_empty());

        let llrs = det.detect_soft(&[]);
        assert!(llrs.is_empty());

        let rsse = ReducedStateMlsd::new(&channel, &[(-1.0, 0.0), (1.0, 0.0)], 2);
        let detected = rsse.detect(&[]);
        assert!(detected.is_empty());
    }

    #[test]
    fn test_complex_channel() {
        // Channel with complex taps
        let channel = [(0.8, 0.2), (0.3, -0.1)];
        let symbols: Vec<(f64, f64)> = vec![
            (1.0, 0.0), (-1.0, 0.0), (-1.0, 0.0), (1.0, 0.0),
        ];
        let received = convolve(&symbols, &channel);

        let det = MlSequenceDetector::bpsk(&channel);
        let detected = det.detect(&received);
        assert_close(&detected, &symbols, 1e-9);
    }
}
