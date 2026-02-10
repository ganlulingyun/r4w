//! # Differential Phasor
//!
//! Computes the differential phasor: `y[n] = x[n] * conj(x[n-1])`.
//! Extracts phase change between consecutive samples, essential for
//! differential PSK (DPSK) demodulation and instantaneous frequency
//! estimation.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::diff_phasor::{diff_phasor, DiffPhasor};
//!
//! let samples = vec![
//!     (1.0, 0.0),   // 0°
//!     (0.0, 1.0),   // 90°
//!     (-1.0, 0.0),  // 180°
//! ];
//! let result = diff_phasor(&samples);
//! assert_eq!(result.len(), 2);
//! // Phase diff should be ~90° each step
//! ```

/// Compute differential phasor: y[n] = x[n] * conj(x[n-1]).
///
/// Output length is input length - 1.
pub fn diff_phasor(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    if input.len() < 2 {
        return Vec::new();
    }
    let mut output = Vec::with_capacity(input.len() - 1);
    for i in 1..input.len() {
        let (re, im) = complex_mul_conj(input[i], input[i - 1]);
        output.push((re, im));
    }
    output
}

/// Compute differential phasor with custom lag.
///
/// y[n] = x[n] * conj(x[n-lag]).
pub fn diff_phasor_lag(input: &[(f64, f64)], lag: usize) -> Vec<(f64, f64)> {
    if lag == 0 || input.len() <= lag {
        return Vec::new();
    }
    let mut output = Vec::with_capacity(input.len() - lag);
    for i in lag..input.len() {
        output.push(complex_mul_conj(input[i], input[i - lag]));
    }
    output
}

/// Extract instantaneous phase difference (radians) from differential phasor.
pub fn diff_phase(input: &[(f64, f64)]) -> Vec<f64> {
    let dp = diff_phasor(input);
    dp.iter().map(|&(re, im)| im.atan2(re)).collect()
}

/// Extract instantaneous frequency (normalized, cycles/sample) from phase differences.
pub fn inst_freq(input: &[(f64, f64)]) -> Vec<f64> {
    let phases = diff_phase(input);
    phases
        .iter()
        .map(|&p| p / (2.0 * std::f64::consts::PI))
        .collect()
}

/// Stateful differential phasor for streaming.
#[derive(Debug, Clone)]
pub struct DiffPhasor {
    prev: (f64, f64),
    count: u64,
}

impl DiffPhasor {
    /// Create a new stateful differential phasor.
    pub fn new() -> Self {
        Self {
            prev: (1.0, 0.0),
            count: 0,
        }
    }

    /// Process a single sample, returning the differential phasor output.
    pub fn process_one(&mut self, sample: (f64, f64)) -> (f64, f64) {
        let result = complex_mul_conj(sample, self.prev);
        self.prev = sample;
        self.count += 1;
        result
    }

    /// Process a block of samples.
    pub fn process(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut output = Vec::with_capacity(input.len());
        for &s in input {
            output.push(self.process_one(s));
        }
        output
    }

    /// Process and extract phase differences (radians).
    pub fn process_phase(&mut self, input: &[(f64, f64)]) -> Vec<f64> {
        self.process(input)
            .iter()
            .map(|&(re, im)| im.atan2(re))
            .collect()
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.prev = (1.0, 0.0);
        self.count = 0;
    }

    /// Get total samples processed.
    pub fn count(&self) -> u64 {
        self.count
    }
}

impl Default for DiffPhasor {
    fn default() -> Self {
        Self::new()
    }
}

/// DPSK demodulator using differential phasor.
///
/// Maps phase differences to symbol indices for M-DPSK.
#[derive(Debug, Clone)]
pub struct DpskDemod {
    phasor: DiffPhasor,
    m: usize,
    phase_step: f64,
}

impl DpskDemod {
    /// Create a new DPSK demodulator for M-ary modulation.
    pub fn new(m: usize) -> Self {
        let phase_step = 2.0 * std::f64::consts::PI / m as f64;
        Self {
            phasor: DiffPhasor::new(),
            m,
            phase_step,
        }
    }

    /// Demodulate samples to symbol indices (0..M-1).
    pub fn demodulate(&mut self, input: &[(f64, f64)]) -> Vec<usize> {
        let dp = self.phasor.process(input);
        dp.iter()
            .map(|&(re, im)| {
                let mut phase = im.atan2(re);
                if phase < 0.0 {
                    phase += 2.0 * std::f64::consts::PI;
                }
                ((phase / self.phase_step).round() as usize) % self.m
            })
            .collect()
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.phasor.reset();
    }
}

/// Complex multiply: a * conj(b).
#[inline]
fn complex_mul_conj(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    // conj(b) = (b.0, -b.1)
    // a * conj(b) = (a.0*b.0 + a.1*b.1, a.1*b.0 - a.0*b.1)
    (a.0 * b.0 + a.1 * b.1, a.1 * b.0 - a.0 * b.1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_diff_phasor_basic() {
        // Constant signal → diff phasor has zero phase.
        let input = vec![(1.0, 0.0); 5];
        let dp = diff_phasor(&input);
        assert_eq!(dp.len(), 4);
        for &(re, im) in &dp {
            assert!((re - 1.0).abs() < 1e-10);
            assert!(im.abs() < 1e-10);
        }
    }

    #[test]
    fn test_diff_phasor_90deg() {
        // 90° phase steps.
        let input = vec![
            (1.0, 0.0),
            (0.0, 1.0),
            (-1.0, 0.0),
            (0.0, -1.0),
        ];
        let dp = diff_phasor(&input);
        assert_eq!(dp.len(), 3);
        // Each should be ~(0, 1) → 90° rotation.
        for &(re, im) in &dp {
            assert!(re.abs() < 1e-10);
            assert!((im - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_diff_phase() {
        let input = vec![
            (1.0, 0.0),
            (0.0, 1.0),
            (-1.0, 0.0),
        ];
        let phases = diff_phase(&input);
        assert_eq!(phases.len(), 2);
        for &p in &phases {
            assert!((p - PI / 2.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_inst_freq() {
        // Constant tone at 0.25 cycles/sample (= π/2 rad/sample).
        let n = 20;
        let freq = 0.25;
        let input: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * freq * i as f64;
                (phase.cos(), phase.sin())
            })
            .collect();
        let freqs = inst_freq(&input);
        for &f in &freqs {
            assert!((f - freq).abs() < 1e-10);
        }
    }

    #[test]
    fn test_diff_phasor_lag() {
        let input = vec![
            (1.0, 0.0),
            (0.0, 1.0),
            (-1.0, 0.0),
            (0.0, -1.0),
            (1.0, 0.0),
        ];
        // Lag 2: each pair is 180° apart.
        let dp = diff_phasor_lag(&input, 2);
        assert_eq!(dp.len(), 3);
        for &(re, im) in &dp {
            assert!((re - (-1.0)).abs() < 1e-10);
            assert!(im.abs() < 1e-10);
        }
    }

    #[test]
    fn test_stateful_matches_batch() {
        let input = vec![
            (1.0, 0.0),
            (0.7071, 0.7071),
            (0.0, 1.0),
            (-0.7071, 0.7071),
        ];
        let batch = diff_phasor(&input);
        let mut dp = DiffPhasor::new();
        // First sample produces a result against initial (1,0).
        let _ = dp.process_one(input[0]);
        let streaming: Vec<(f64, f64)> = input[1..]
            .iter()
            .map(|&s| dp.process_one(s))
            .collect();
        for (b, s) in batch.iter().zip(streaming.iter()) {
            assert!((b.0 - s.0).abs() < 1e-6);
            assert!((b.1 - s.1).abs() < 1e-6);
        }
    }

    #[test]
    fn test_dpsk_demod_bpsk() {
        // BPSK: 0 → phase 0, 1 → phase π.
        let mut demod = DpskDemod::new(2);
        let input = vec![
            (1.0, 0.0),  // reference
            (1.0, 0.0),  // symbol 0 (no phase change)
            (-1.0, 0.0), // symbol 1 (π phase change)
            (-1.0, 0.0), // symbol 0 (no phase change from prev: -1*conj(-1) = 1)
        ];
        let symbols = demod.demodulate(&input);
        assert_eq!(symbols[1], 0); // no change
        assert_eq!(symbols[2], 1); // π change
    }

    #[test]
    fn test_dpsk_demod_qpsk() {
        let mut demod = DpskDemod::new(4);
        // Symbol 1 → 90° phase increment.
        let input = vec![
            (1.0, 0.0),
            (0.0, 1.0), // +90° → symbol 1
        ];
        let symbols = demod.demodulate(&input);
        assert_eq!(symbols[1], 1);
    }

    #[test]
    fn test_empty_input() {
        assert!(diff_phasor(&[]).is_empty());
        assert!(diff_phasor(&[(1.0, 0.0)]).is_empty());
        assert!(diff_phasor_lag(&[(1.0, 0.0)], 1).is_empty());
        assert!(diff_phase(&[]).is_empty());
    }

    #[test]
    fn test_process_phase() {
        let mut dp = DiffPhasor::new();
        let input = vec![
            (1.0, 0.0),
            (0.0, 1.0),
            (-1.0, 0.0),
        ];
        let phases = dp.process_phase(&input);
        assert_eq!(phases.len(), 3);
        // Second and third should be ~π/2.
        assert!((phases[1] - PI / 2.0).abs() < 1e-10);
        assert!((phases[2] - PI / 2.0).abs() < 1e-10);
        assert_eq!(dp.count(), 3);
    }
}
