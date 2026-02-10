//! Maximum Likelihood Sequence Estimation (MLSE) equalizer using the Viterbi algorithm.
//!
//! This module implements MLSE for equalization of frequency-selective (ISI) channels.
//! The Viterbi algorithm searches a trellis whose states correspond to channel memory
//! contents, finding the most-likely transmitted symbol sequence given the received samples
//! and an estimated channel impulse response.
//!
//! # Features
//! - Configurable channel taps and constellation (BPSK, QPSK)
//! - Full-state and reduced-state (RSSE) sequence estimation
//! - Euclidean branch metric computation
//! - Survivor path management with configurable traceback depth
//! - BER estimation utility
//!
//! # Example
//! ```
//! use r4w_core::sequential_detection_mlse::{MlseEqualizer, Constellation};
//!
//! // Two-tap channel: h = [1.0, 0.5]
//! let channel_taps = vec![(1.0, 0.0), (0.5, 0.0)];
//! let eq = MlseEqualizer::new(channel_taps, Constellation::Bpsk, None);
//!
//! // Transmit BPSK symbols through channel
//! let symbols = vec![(1.0, 0.0), (-1.0, 0.0), (1.0, 0.0), (1.0, 0.0)];
//! // Convolve with channel
//! let received: Vec<(f64, f64)> = (0..symbols.len())
//!     .map(|n| {
//!         let mut acc = (0.0, 0.0);
//!         for (k, h) in eq.channel_taps().iter().enumerate() {
//!             if n >= k {
//!                 let s = symbols[n - k];
//!                 acc.0 += h.0 * s.0 - h.1 * s.1;
//!                 acc.1 += h.0 * s.1 + h.1 * s.0;
//!             }
//!         }
//!         acc
//!     })
//!     .collect();
//!
//! let detected = eq.equalize(&received);
//! assert_eq!(detected.len(), symbols.len());
//! // In a noiseless channel the detector should recover the original symbols
//! for (det, orig) in detected.iter().zip(symbols.iter()) {
//!     assert!((det.0 - orig.0).abs() < 1e-6);
//! }
//! ```

use std::f64;

// ---------------------------------------------------------------------------
// Complex arithmetic helpers (using (f64,f64) tuples)
// ---------------------------------------------------------------------------

#[inline]
fn cx_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn cx_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn cx_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn cx_norm_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

// ---------------------------------------------------------------------------
// Constellation
// ---------------------------------------------------------------------------

/// Supported constellations for the MLSE equalizer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Constellation {
    /// Binary Phase-Shift Keying: {+1, -1}
    Bpsk,
    /// Quadrature Phase-Shift Keying: {+-1 +-j} / sqrt(2)
    Qpsk,
}

impl Constellation {
    /// Return the set of constellation points.
    pub fn points(&self) -> Vec<(f64, f64)> {
        match self {
            Constellation::Bpsk => vec![(1.0, 0.0), (-1.0, 0.0)],
            Constellation::Qpsk => {
                let s = std::f64::consts::FRAC_1_SQRT_2;
                vec![(s, s), (-s, s), (-s, -s), (s, -s)]
            }
        }
    }

    /// Number of constellation points (alphabet size M).
    pub fn size(&self) -> usize {
        match self {
            Constellation::Bpsk => 2,
            Constellation::Qpsk => 4,
        }
    }

    /// Bits per symbol.
    pub fn bits_per_symbol(&self) -> usize {
        match self {
            Constellation::Bpsk => 1,
            Constellation::Qpsk => 2,
        }
    }
}

// ---------------------------------------------------------------------------
// Trellis state
// ---------------------------------------------------------------------------

/// A trellis state represents the channel memory contents (the last L-1 transmitted symbols
/// encoded as constellation indices).
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct TrellisState {
    /// Symbol indices in the channel memory, oldest first. Length = channel_memory.
    pub memory: Vec<usize>,
}

impl TrellisState {
    /// Encode the state as a single integer for indexing.
    pub fn to_index(&self, alphabet_size: usize) -> usize {
        let mut idx = 0usize;
        for &m in &self.memory {
            idx = idx * alphabet_size + m;
        }
        idx
    }

    /// Decode a state index back into memory contents.
    pub fn from_index(index: usize, memory_len: usize, alphabet_size: usize) -> Self {
        let mut mem = vec![0usize; memory_len];
        let mut rem = index;
        for i in (0..memory_len).rev() {
            mem[i] = rem % alphabet_size;
            rem /= alphabet_size;
        }
        TrellisState { memory: mem }
    }
}

// ---------------------------------------------------------------------------
// MLSE Equalizer
// ---------------------------------------------------------------------------

/// Maximum Likelihood Sequence Estimation equalizer.
///
/// Uses the Viterbi algorithm over a trellis whose states represent channel memory to
/// find the most-likely transmitted symbol sequence.
#[derive(Debug, Clone)]
pub struct MlseEqualizer {
    /// Channel impulse response taps h[0], h[1], ..., h[L-1].
    channel_taps: Vec<(f64, f64)>,
    /// The constellation used for transmission.
    constellation: Constellation,
    /// Traceback depth (in symbols). Defaults to 5 * channel_memory if not set.
    traceback_depth: usize,
    /// Number of trellis states (M^(L-1)).
    num_states: usize,
    /// Channel memory length L-1.
    channel_memory: usize,
    /// Optional: maximum number of retained states per trellis stage for RSSE.
    /// `None` means full-state (all states retained).
    max_states: Option<usize>,
}

impl MlseEqualizer {
    /// Create a new MLSE equalizer.
    ///
    /// * `channel_taps` - estimated channel impulse response (complex taps).
    /// * `constellation` - the modulation constellation.
    /// * `traceback_depth` - optional traceback depth; defaults to `5 * (taps.len() - 1)`.
    pub fn new(
        channel_taps: Vec<(f64, f64)>,
        constellation: Constellation,
        traceback_depth: Option<usize>,
    ) -> Self {
        assert!(!channel_taps.is_empty(), "channel_taps must be non-empty");
        let l = channel_taps.len(); // L taps -> memory = L-1
        let channel_memory = if l > 1 { l - 1 } else { 0 };
        let m = constellation.size();
        let num_states = m.pow(channel_memory as u32);
        let tb = traceback_depth.unwrap_or(5 * channel_memory.max(1));
        MlseEqualizer {
            channel_taps,
            constellation,
            traceback_depth: tb,
            num_states,
            channel_memory,
            max_states: None,
        }
    }

    /// Enable reduced-state sequence estimation (RSSE).
    ///
    /// At each trellis stage only the `max_states` best states are kept.
    pub fn with_rsse(mut self, max_states: usize) -> Self {
        assert!(max_states > 0, "max_states must be > 0");
        self.max_states = Some(max_states);
        self
    }

    /// Read-only access to channel taps.
    pub fn channel_taps(&self) -> &[(f64, f64)] {
        &self.channel_taps
    }

    /// The constellation in use.
    pub fn constellation(&self) -> Constellation {
        self.constellation
    }

    /// Number of trellis states.
    pub fn num_states(&self) -> usize {
        self.num_states
    }

    /// Channel memory length (L-1).
    pub fn channel_memory(&self) -> usize {
        self.channel_memory
    }

    /// Traceback depth.
    pub fn traceback_depth(&self) -> usize {
        self.traceback_depth
    }

    /// Update the channel taps (e.g. after re-estimation).
    pub fn set_channel_taps(&mut self, taps: Vec<(f64, f64)>) {
        assert!(!taps.is_empty());
        let l = taps.len();
        self.channel_memory = if l > 1 { l - 1 } else { 0 };
        let m = self.constellation.size();
        self.num_states = m.pow(self.channel_memory as u32);
        self.channel_taps = taps;
    }

    // -----------------------------------------------------------------------
    // Branch metric
    // -----------------------------------------------------------------------

    /// Compute the Euclidean branch metric (squared distance) between the received
    /// sample `r` and the hypothesized noiseless output for transmitting symbol with
    /// index `sym_idx` when the channel memory contains `state`.
    fn branch_metric(
        &self,
        r: (f64, f64),
        sym_idx: usize,
        state: &TrellisState,
    ) -> f64 {
        let points = self.constellation.points();
        // Hypothesized channel output: y = h[0]*sym + sum_{k=1}^{L-1} h[k]*state[k-1]
        let sym = points[sym_idx];
        let mut y = cx_mul(self.channel_taps[0], sym);
        for k in 1..self.channel_taps.len() {
            let past_sym = points[state.memory[k - 1]];
            y = cx_add(y, cx_mul(self.channel_taps[k], past_sym));
        }
        cx_norm_sq(cx_sub(r, y))
    }

    // -----------------------------------------------------------------------
    // Viterbi search
    // -----------------------------------------------------------------------

    /// Run the Viterbi algorithm on `received` samples and return the detected symbols.
    pub fn equalize(&self, received: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if received.is_empty() {
            return Vec::new();
        }

        let n = received.len();
        let m = self.constellation.size();
        let points = self.constellation.points();

        // Path metrics: one f64 per state. Flat-start: all states equally likely.
        let mut path_metric = vec![0.0f64; self.num_states];

        // Survivor table: for each time step, store (prev_state_idx, symbol_idx) per state.
        let mut survivors: Vec<Vec<(usize, usize)>> = Vec::with_capacity(n);

        for t in 0..n {
            let mut new_metric = vec![f64::INFINITY; self.num_states];
            let mut new_survivor = vec![(0usize, 0usize); self.num_states];

            // If RSSE is active, determine which states are active.
            let active_states: Vec<usize> = if let Some(max_s) = self.max_states {
                let mut state_metrics: Vec<(usize, f64)> = path_metric
                    .iter()
                    .enumerate()
                    .filter(|(_, &pm)| pm < f64::INFINITY)
                    .map(|(i, &pm)| (i, pm))
                    .collect();
                state_metrics.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
                state_metrics.truncate(max_s);
                state_metrics.into_iter().map(|(i, _)| i).collect()
            } else {
                (0..self.num_states).collect()
            };

            for &si in &active_states {
                if path_metric[si] == f64::INFINITY {
                    continue;
                }
                let state = TrellisState::from_index(si, self.channel_memory, m);

                for sym_idx in 0..m {
                    let bm = self.branch_metric(received[t], sym_idx, &state);
                    let total = path_metric[si] + bm;

                    // Compute next state: shift memory left, append sym_idx
                    let next_state = if self.channel_memory > 0 {
                        let mut mem = state.memory.clone();
                        mem.remove(0);
                        mem.push(sym_idx);
                        TrellisState { memory: mem }.to_index(m)
                    } else {
                        0
                    };

                    if total < new_metric[next_state] {
                        new_metric[next_state] = total;
                        new_survivor[next_state] = (si, sym_idx);
                    }
                }
            }

            path_metric = new_metric;
            survivors.push(new_survivor);
        }

        // Traceback: find best final state
        let best_final = path_metric
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, _)| i)
            .unwrap_or(0);

        let mut symbol_indices = vec![0usize; n];
        let mut state = best_final;
        for t in (0..n).rev() {
            let (prev_state, sym_idx) = survivors[t][state];
            symbol_indices[t] = sym_idx;
            state = prev_state;
        }

        symbol_indices.iter().map(|&si| points[si]).collect()
    }

    /// Enumerate all trellis states for inspection.
    pub fn enumerate_states(&self) -> Vec<TrellisState> {
        let m = self.constellation.size();
        (0..self.num_states)
            .map(|i| TrellisState::from_index(i, self.channel_memory, m))
            .collect()
    }

    /// Compute the expected channel output for a given symbol and state.
    pub fn expected_output(&self, sym_idx: usize, state: &TrellisState) -> (f64, f64) {
        let points = self.constellation.points();
        let sym = points[sym_idx];
        let mut y = cx_mul(self.channel_taps[0], sym);
        for k in 1..self.channel_taps.len() {
            let past_sym = points[state.memory[k - 1]];
            y = cx_add(y, cx_mul(self.channel_taps[k], past_sym));
        }
        y
    }
}

// ---------------------------------------------------------------------------
// BER estimation utility
// ---------------------------------------------------------------------------

/// Estimate the bit error rate between two symbol sequences (sliced to the nearest
/// constellation point).
///
/// Returns `(bit_errors, total_bits, ber)`.
pub fn estimate_ber(
    transmitted: &[(f64, f64)],
    detected: &[(f64, f64)],
    constellation: Constellation,
) -> (usize, usize, f64) {
    let points = constellation.points();
    let bps = constellation.bits_per_symbol();
    let len = transmitted.len().min(detected.len());
    let mut bit_errors = 0usize;

    for i in 0..len {
        let tx_idx = nearest_point(transmitted[i], &points);
        let rx_idx = nearest_point(detected[i], &points);
        bit_errors += (tx_idx ^ rx_idx).count_ones() as usize;
    }

    let total_bits = len * bps;
    let ber = if total_bits > 0 {
        bit_errors as f64 / total_bits as f64
    } else {
        0.0
    };
    (bit_errors, total_bits, ber)
}

/// Find the index of the nearest constellation point.
fn nearest_point(sample: (f64, f64), points: &[(f64, f64)]) -> usize {
    let mut best = 0;
    let mut best_dist = f64::INFINITY;
    for (i, &p) in points.iter().enumerate() {
        let d = cx_norm_sq(cx_sub(sample, p));
        if d < best_dist {
            best_dist = d;
            best = i;
        }
    }
    best
}

/// Convolve a symbol sequence with a channel impulse response (for testing).
pub fn convolve_channel(
    symbols: &[(f64, f64)],
    channel: &[(f64, f64)],
) -> Vec<(f64, f64)> {
    let out_len = symbols.len() + channel.len() - 1;
    let mut output = vec![(0.0, 0.0); out_len];
    for (n, s) in symbols.iter().enumerate() {
        for (k, h) in channel.iter().enumerate() {
            let prod = cx_mul(*s, *h);
            output[n + k] = cx_add(output[n + k], prod);
        }
    }
    output
}

/// Convolve and truncate to the same length as symbols (causal, no tail).
pub fn convolve_channel_truncated(
    symbols: &[(f64, f64)],
    channel: &[(f64, f64)],
) -> Vec<(f64, f64)> {
    let n = symbols.len();
    (0..n)
        .map(|i| {
            let mut acc = (0.0, 0.0);
            for (k, h) in channel.iter().enumerate() {
                if i >= k {
                    acc = cx_add(acc, cx_mul(*h, symbols[i - k]));
                }
            }
            acc
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Helper: check that two complex values are close.
    fn assert_cx_close(a: (f64, f64), b: (f64, f64), tol: f64) {
        assert!(
            (a.0 - b.0).abs() < tol && (a.1 - b.1).abs() < tol,
            "expected ({}, {}) ~ ({}, {})",
            a.0, a.1, b.0, b.1
        );
    }

    // -----------------------------------------------------------------------
    // Complex arithmetic
    // -----------------------------------------------------------------------

    #[test]
    fn test_cx_add() {
        assert_eq!(cx_add((1.0, 2.0), (3.0, 4.0)), (4.0, 6.0));
    }

    #[test]
    fn test_cx_mul() {
        // (1+2j)(3+4j) = (1*3 - 2*4) + (1*4 + 2*3)j = -5 + 10j
        assert_cx_close(cx_mul((1.0, 2.0), (3.0, 4.0)), (-5.0, 10.0), 1e-12);
    }

    #[test]
    fn test_cx_norm_sq() {
        assert!((cx_norm_sq((3.0, 4.0)) - 25.0).abs() < 1e-12);
    }

    // -----------------------------------------------------------------------
    // Constellation
    // -----------------------------------------------------------------------

    #[test]
    fn test_bpsk_constellation() {
        let c = Constellation::Bpsk;
        assert_eq!(c.size(), 2);
        assert_eq!(c.bits_per_symbol(), 1);
        let pts = c.points();
        assert_eq!(pts.len(), 2);
        assert_cx_close(pts[0], (1.0, 0.0), 1e-12);
        assert_cx_close(pts[1], (-1.0, 0.0), 1e-12);
    }

    #[test]
    fn test_qpsk_constellation() {
        let c = Constellation::Qpsk;
        assert_eq!(c.size(), 4);
        assert_eq!(c.bits_per_symbol(), 2);
        let pts = c.points();
        assert_eq!(pts.len(), 4);
        // All points should have unit energy
        for p in &pts {
            assert!((cx_norm_sq(*p) - 1.0).abs() < 1e-12);
        }
    }

    // -----------------------------------------------------------------------
    // Trellis state
    // -----------------------------------------------------------------------

    #[test]
    fn test_trellis_state_roundtrip() {
        for idx in 0..8 {
            let s = TrellisState::from_index(idx, 3, 2);
            assert_eq!(s.to_index(2), idx);
        }
    }

    #[test]
    fn test_trellis_state_from_index() {
        // 2 memory slots, alphabet size 2 -> states 0..4
        let s = TrellisState::from_index(3, 2, 2);
        assert_eq!(s.memory, vec![1, 1]);
    }

    // -----------------------------------------------------------------------
    // Equalizer construction
    // -----------------------------------------------------------------------

    #[test]
    fn test_equalizer_new_bpsk() {
        let eq = MlseEqualizer::new(
            vec![(1.0, 0.0), (0.5, 0.0)],
            Constellation::Bpsk,
            None,
        );
        assert_eq!(eq.channel_memory(), 1);
        assert_eq!(eq.num_states(), 2); // 2^1
        assert_eq!(eq.traceback_depth(), 5);
    }

    #[test]
    fn test_equalizer_new_qpsk_3tap() {
        let eq = MlseEqualizer::new(
            vec![(1.0, 0.0), (0.3, 0.1), (0.1, -0.05)],
            Constellation::Qpsk,
            Some(20),
        );
        assert_eq!(eq.channel_memory(), 2);
        assert_eq!(eq.num_states(), 16); // 4^2
        assert_eq!(eq.traceback_depth(), 20);
    }

    #[test]
    fn test_single_tap_channel() {
        // Single-tap channel -> no ISI, memory = 0, 1 state.
        let eq = MlseEqualizer::new(
            vec![(1.0, 0.0)],
            Constellation::Bpsk,
            None,
        );
        assert_eq!(eq.channel_memory(), 0);
        assert_eq!(eq.num_states(), 1);
    }

    // -----------------------------------------------------------------------
    // Equalization - noiseless BPSK
    // -----------------------------------------------------------------------

    #[test]
    fn test_equalize_bpsk_noiseless_2tap() {
        let channel = vec![(1.0, 0.0), (0.5, 0.0)];
        let eq = MlseEqualizer::new(channel.clone(), Constellation::Bpsk, None);

        let symbols: Vec<(f64, f64)> = vec![
            (1.0, 0.0), (-1.0, 0.0), (1.0, 0.0), (1.0, 0.0),
            (-1.0, 0.0), (-1.0, 0.0), (1.0, 0.0), (-1.0, 0.0),
        ];

        let received = convolve_channel_truncated(&symbols, &channel);
        let detected = eq.equalize(&received);

        assert_eq!(detected.len(), symbols.len());
        for (_i, (det, orig)) in detected.iter().zip(symbols.iter()).enumerate() {
            assert_cx_close(*det, *orig, 1e-6);
        }
    }

    #[test]
    fn test_equalize_bpsk_noiseless_3tap() {
        let channel = vec![(1.0, 0.0), (0.4, 0.0), (0.2, 0.0)];
        let eq = MlseEqualizer::new(channel.clone(), Constellation::Bpsk, Some(15));

        let symbols: Vec<(f64, f64)> = vec![
            (1.0, 0.0), (1.0, 0.0), (-1.0, 0.0), (1.0, 0.0),
            (-1.0, 0.0), (-1.0, 0.0), (1.0, 0.0), (1.0, 0.0),
            (-1.0, 0.0), (1.0, 0.0),
        ];

        let received = convolve_channel_truncated(&symbols, &channel);
        let detected = eq.equalize(&received);

        for (det, orig) in detected.iter().zip(symbols.iter()) {
            assert_cx_close(*det, *orig, 1e-6);
        }
    }

    // -----------------------------------------------------------------------
    // Equalization - noiseless QPSK
    // -----------------------------------------------------------------------

    #[test]
    fn test_equalize_qpsk_noiseless_2tap() {
        let channel = vec![(1.0, 0.0), (0.3, 0.1)];
        let eq = MlseEqualizer::new(channel.clone(), Constellation::Qpsk, None);

        let s = std::f64::consts::FRAC_1_SQRT_2;
        let symbols: Vec<(f64, f64)> = vec![
            (s, s), (-s, s), (-s, -s), (s, -s),
            (s, s), (s, -s), (-s, s), (-s, -s),
        ];

        let received = convolve_channel_truncated(&symbols, &channel);
        let detected = eq.equalize(&received);

        for (det, orig) in detected.iter().zip(symbols.iter()) {
            assert_cx_close(*det, *orig, 1e-6);
        }
    }

    // -----------------------------------------------------------------------
    // RSSE
    // -----------------------------------------------------------------------

    #[test]
    fn test_rsse_reduces_without_breaking() {
        let channel = vec![(1.0, 0.0), (0.5, 0.0)];
        let eq = MlseEqualizer::new(channel.clone(), Constellation::Bpsk, None)
            .with_rsse(2); // Keep all 2 states -> same as full

        let symbols: Vec<(f64, f64)> = vec![
            (1.0, 0.0), (-1.0, 0.0), (1.0, 0.0), (-1.0, 0.0),
        ];
        let received = convolve_channel_truncated(&symbols, &channel);
        let detected = eq.equalize(&received);

        for (det, orig) in detected.iter().zip(symbols.iter()) {
            assert_cx_close(*det, *orig, 1e-6);
        }
    }

    #[test]
    fn test_rsse_with_fewer_states() {
        // 3-tap QPSK -> 16 states, keep only 8
        let channel = vec![(1.0, 0.0), (0.2, 0.0), (0.1, 0.0)];
        let eq = MlseEqualizer::new(channel.clone(), Constellation::Qpsk, Some(20))
            .with_rsse(8);

        let s = std::f64::consts::FRAC_1_SQRT_2;
        let symbols: Vec<(f64, f64)> = vec![
            (s, s), (-s, s), (-s, -s), (s, -s),
            (s, s), (s, -s),
        ];

        let received = convolve_channel_truncated(&symbols, &channel);
        let detected = eq.equalize(&received);

        // With weak ISI and no noise, RSSE should still recover correctly
        for (det, orig) in detected.iter().zip(symbols.iter()) {
            assert_cx_close(*det, *orig, 1e-6);
        }
    }

    // -----------------------------------------------------------------------
    // BER estimation
    // -----------------------------------------------------------------------

    #[test]
    fn test_ber_zero_errors() {
        let syms = vec![(1.0, 0.0), (-1.0, 0.0), (1.0, 0.0)];
        let (errs, bits, ber) = estimate_ber(&syms, &syms, Constellation::Bpsk);
        assert_eq!(errs, 0);
        assert_eq!(bits, 3);
        assert!((ber - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_ber_all_errors_bpsk() {
        let tx = vec![(1.0, 0.0), (1.0, 0.0)];
        let rx = vec![(-1.0, 0.0), (-1.0, 0.0)];
        let (errs, bits, ber) = estimate_ber(&tx, &rx, Constellation::Bpsk);
        assert_eq!(errs, 2);
        assert_eq!(bits, 2);
        assert!((ber - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_ber_partial_errors() {
        let tx = vec![(1.0, 0.0), (-1.0, 0.0), (1.0, 0.0), (1.0, 0.0)];
        let rx = vec![(1.0, 0.0), (1.0, 0.0), (1.0, 0.0), (-1.0, 0.0)];
        let (errs, bits, ber) = estimate_ber(&tx, &rx, Constellation::Bpsk);
        assert_eq!(errs, 2); // 2 bit flips
        assert_eq!(bits, 4);
        assert!((ber - 0.5).abs() < 1e-12);
    }

    // -----------------------------------------------------------------------
    // Convolution helper
    // -----------------------------------------------------------------------

    #[test]
    fn test_convolve_channel() {
        let syms = vec![(1.0, 0.0), (-1.0, 0.0)];
        let ch = vec![(1.0, 0.0), (0.5, 0.0)];
        let out = convolve_channel(&syms, &ch);
        // Expected: [1*1, 1*0.5+(-1)*1, (-1)*0.5] = [1, -0.5, -0.5]
        assert_eq!(out.len(), 3);
        assert_cx_close(out[0], (1.0, 0.0), 1e-12);
        assert_cx_close(out[1], (-0.5, 0.0), 1e-12);
        assert_cx_close(out[2], (-0.5, 0.0), 1e-12);
    }

    #[test]
    fn test_convolve_channel_truncated() {
        let syms = vec![(1.0, 0.0), (-1.0, 0.0), (1.0, 0.0)];
        let ch = vec![(1.0, 0.0), (0.5, 0.0)];
        let out = convolve_channel_truncated(&syms, &ch);
        assert_eq!(out.len(), 3);
        // t=0: h[0]*s[0] = 1
        // t=1: h[0]*s[1] + h[1]*s[0] = -1 + 0.5 = -0.5
        // t=2: h[0]*s[2] + h[1]*s[1] = 1 + (-0.5) = 0.5
        assert_cx_close(out[0], (1.0, 0.0), 1e-12);
        assert_cx_close(out[1], (-0.5, 0.0), 1e-12);
        assert_cx_close(out[2], (0.5, 0.0), 1e-12);
    }

    // -----------------------------------------------------------------------
    // Edge cases
    // -----------------------------------------------------------------------

    #[test]
    fn test_equalize_empty_input() {
        let eq = MlseEqualizer::new(
            vec![(1.0, 0.0), (0.5, 0.0)],
            Constellation::Bpsk,
            None,
        );
        let result = eq.equalize(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_equalize_single_symbol() {
        let channel = vec![(1.0, 0.0), (0.3, 0.0)];
        let eq = MlseEqualizer::new(channel.clone(), Constellation::Bpsk, None);
        let symbols = vec![(1.0, 0.0)];
        let received = convolve_channel_truncated(&symbols, &channel);
        let detected = eq.equalize(&received);
        assert_eq!(detected.len(), 1);
        assert_cx_close(detected[0], (1.0, 0.0), 1e-6);
    }

    // -----------------------------------------------------------------------
    // Enumerate states
    // -----------------------------------------------------------------------

    #[test]
    fn test_enumerate_states_bpsk_2tap() {
        let eq = MlseEqualizer::new(
            vec![(1.0, 0.0), (0.5, 0.0)],
            Constellation::Bpsk,
            None,
        );
        let states = eq.enumerate_states();
        assert_eq!(states.len(), 2);
        assert_eq!(states[0].memory, vec![0]);
        assert_eq!(states[1].memory, vec![1]);
    }

    // -----------------------------------------------------------------------
    // Expected output
    // -----------------------------------------------------------------------

    #[test]
    fn test_expected_output() {
        let eq = MlseEqualizer::new(
            vec![(1.0, 0.0), (0.5, 0.0)],
            Constellation::Bpsk,
            None,
        );
        // sym=0 (+1), state memory=[0] (prev was +1)
        // expected: 1.0*1.0 + 0.5*1.0 = 1.5
        let state = TrellisState { memory: vec![0] };
        let y = eq.expected_output(0, &state);
        assert_cx_close(y, (1.5, 0.0), 1e-12);

        // sym=1 (-1), state memory=[0] (prev was +1)
        // expected: 1.0*(-1.0) + 0.5*1.0 = -0.5
        let y2 = eq.expected_output(1, &state);
        assert_cx_close(y2, (-0.5, 0.0), 1e-12);
    }

    // -----------------------------------------------------------------------
    // Set channel taps
    // -----------------------------------------------------------------------

    #[test]
    fn test_set_channel_taps() {
        let mut eq = MlseEqualizer::new(
            vec![(1.0, 0.0), (0.5, 0.0)],
            Constellation::Bpsk,
            None,
        );
        assert_eq!(eq.channel_memory(), 1);

        eq.set_channel_taps(vec![(1.0, 0.0), (0.3, 0.0), (0.1, 0.0)]);
        assert_eq!(eq.channel_memory(), 2);
        assert_eq!(eq.num_states(), 4); // 2^2
    }
}
