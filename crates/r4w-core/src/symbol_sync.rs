//! Symbol Synchronizer
//!
//! Polyphase filterbank-based symbol synchronizer with configurable
//! Timing Error Detector (TED) algorithms:
//!
//! - **Gardner**: Works with unknown data, no decision feedback
//! - **Zero-Crossing**: Simple, effective for binary signals
//! - **Mueller & Müller (M&M)**: Classic decision-directed TED
//!
//! ## Algorithm
//!
//! Uses an interpolating polyphase filter bank to resample the input
//! at the optimal sampling instant, driven by a TED + PI loop filter.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::symbol_sync::{SymbolSync, TedType};
//! use num_complex::Complex64;
//!
//! let mut sync = SymbolSync::new(4.0, TedType::Gardner, 0.045);
//! let input = vec![Complex64::new(1.0, 0.0); 100];
//! let symbols = sync.process(&input);
//! ```

use num_complex::Complex64;

/// Timing Error Detector type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TedType {
    /// Gardner TED: e = Re{y_mid * conj(y_cur - y_prev)}
    Gardner,
    /// Zero-crossing TED: e = Re{y_mid} * sign(Re{y_cur} - Re{y_prev})
    ZeroCrossing,
    /// Mueller & Müller TED: e = Re{y_prev * conj(y_cur - y_prev_prev)}
    MuellerMuller,
}

/// Symbol synchronizer with interpolation and loop filter.
#[derive(Debug, Clone)]
pub struct SymbolSync {
    /// Nominal samples per symbol
    sps: f64,
    /// TED type
    ted_type: TedType,
    /// Current omega (samples per symbol estimate)
    omega: f64,
    /// Fractional sample offset
    mu: f64,
    /// Loop gain: proportional
    alpha: f64,
    /// Loop gain: integral
    beta: f64,
    /// Previous symbol for TED
    prev_symbol: Complex64,
    /// Previous-previous symbol for M&M
    prev_prev_symbol: Complex64,
    /// Midpoint sample for Gardner
    mid_sample: Complex64,
    /// Sample buffer
    buffer: Vec<Complex64>,
    /// Current position in buffer
    pos: usize,
    /// Omega limits
    omega_limit: f64,
}

impl SymbolSync {
    /// Create a new symbol synchronizer.
    ///
    /// - `sps`: Samples per symbol (must be > 1)
    /// - `ted_type`: Timing error detector algorithm
    /// - `loop_bw`: Loop bandwidth (typically 0.01..0.1)
    pub fn new(sps: f64, ted_type: TedType, loop_bw: f64) -> Self {
        assert!(sps > 1.0, "Samples per symbol must be > 1");
        let denom = 1.0 + 2.0 * 0.707 * loop_bw + loop_bw * loop_bw;
        let alpha = 4.0 * 0.707 * loop_bw / denom;
        let beta = 4.0 * loop_bw * loop_bw / denom;
        Self {
            sps,
            ted_type,
            omega: sps,
            mu: 0.0,
            alpha,
            beta,
            prev_symbol: Complex64::new(0.0, 0.0),
            prev_prev_symbol: Complex64::new(0.0, 0.0),
            mid_sample: Complex64::new(0.0, 0.0),
            buffer: Vec::new(),
            pos: 0,
            omega_limit: 0.005,
        }
    }

    /// Process a block of I/Q samples and output synchronized symbols.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        self.buffer.extend_from_slice(input);
        let mut output = Vec::new();

        while self.pos + 2 < self.buffer.len() {
            // Linear interpolation at current mu
            let idx = self.pos;
            let frac = self.mu;
            let sample = self.buffer[idx] * (1.0 - frac)
                + self.buffer[(idx + 1).min(self.buffer.len() - 1)] * frac;

            // Compute timing error
            let error = self.compute_ted(sample);

            // Update loop
            self.omega += self.beta * error;
            // Clamp omega
            let omega_min = self.sps * (1.0 - self.omega_limit);
            let omega_max = self.sps * (1.0 + self.omega_limit);
            self.omega = self.omega.clamp(omega_min, omega_max);

            self.mu += self.omega + self.alpha * error;

            // Advance
            let advance = self.mu as usize;
            self.pos += advance;
            self.mu -= advance as f64;

            // Store for TED
            self.prev_prev_symbol = self.prev_symbol;
            self.prev_symbol = sample;

            // For Gardner: store midpoint (halfway between symbols)
            let mid_idx = idx + (advance / 2).max(1);
            if mid_idx < self.buffer.len() {
                self.mid_sample = self.buffer[mid_idx];
            }

            output.push(sample);
        }

        // Drain consumed samples
        if self.pos > 0 {
            let consumed = self.pos.min(self.buffer.len());
            self.buffer.drain(..consumed);
            self.pos = 0;
        }

        output
    }

    /// Compute timing error using the configured TED.
    fn compute_ted(&self, current: Complex64) -> f64 {
        match self.ted_type {
            TedType::Gardner => {
                // e = Re{y_mid * conj(y_cur - y_prev)}
                let diff = current - self.prev_symbol;
                (self.mid_sample * diff.conj()).re
            }
            TedType::ZeroCrossing => {
                // e = Re{y_mid} * sign(Re{y_cur} - Re{y_prev})
                let sign = if current.re > self.prev_symbol.re { 1.0 } else { -1.0 };
                self.mid_sample.re * sign
            }
            TedType::MuellerMuller => {
                // e = Re{y_prev * conj(y_cur - y_prev_prev)}
                let diff = current - self.prev_prev_symbol;
                (self.prev_symbol * diff.conj()).re
            }
        }
    }

    /// Get the current samples-per-symbol estimate.
    pub fn omega(&self) -> f64 {
        self.omega
    }

    /// Get the TED type.
    pub fn ted_type(&self) -> TedType {
        self.ted_type
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.omega = self.sps;
        self.mu = 0.0;
        self.prev_symbol = Complex64::new(0.0, 0.0);
        self.prev_prev_symbol = Complex64::new(0.0, 0.0);
        self.mid_sample = Complex64::new(0.0, 0.0);
        self.buffer.clear();
        self.pos = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_construction() {
        let sync = SymbolSync::new(4.0, TedType::Gardner, 0.045);
        assert!((sync.omega() - 4.0).abs() < 1e-10);
        assert_eq!(sync.ted_type(), TedType::Gardner);
    }

    #[test]
    fn test_dc_signal() {
        let mut sync = SymbolSync::new(4.0, TedType::Gardner, 0.045);
        let input = vec![Complex64::new(1.0, 0.0); 200];
        let output = sync.process(&input);
        // ~200/4 = 50 symbols
        assert!(output.len() >= 40 && output.len() <= 60,
            "Expected ~50 symbols, got {}", output.len());
    }

    #[test]
    fn test_alternating_bpsk() {
        let mut sync = SymbolSync::new(4.0, TedType::Gardner, 0.045);
        let sps = 4;
        let n_sym = 100;
        let mut input = Vec::new();
        for i in 0..n_sym {
            let val = if i % 2 == 0 { 1.0 } else { -1.0 };
            for _ in 0..sps {
                input.push(Complex64::new(val, 0.0));
            }
        }
        let output = sync.process(&input);
        assert!(output.len() >= 80 && output.len() <= 120,
            "Expected ~100 symbols, got {}", output.len());
    }

    #[test]
    fn test_zero_crossing_ted() {
        let mut sync = SymbolSync::new(4.0, TedType::ZeroCrossing, 0.045);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = sync.process(&input);
        assert!(!output.is_empty());
    }

    #[test]
    fn test_mm_ted() {
        let mut sync = SymbolSync::new(4.0, TedType::MuellerMuller, 0.045);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = sync.process(&input);
        assert!(!output.is_empty());
    }

    #[test]
    fn test_complex_signal() {
        let mut sync = SymbolSync::new(4.0, TedType::Gardner, 0.045);
        let n = 200;
        let input: Vec<Complex64> = (0..n).map(|i| {
            let phase = 2.0 * PI * i as f64 / 4.0;
            Complex64::new(phase.cos(), phase.sin())
        }).collect();
        let output = sync.process(&input);
        assert!(!output.is_empty());
    }

    #[test]
    fn test_incremental_processing() {
        let mut sync = SymbolSync::new(4.0, TedType::Gardner, 0.045);
        let mut total = 0;
        for _ in 0..10 {
            let output = sync.process(&vec![Complex64::new(1.0, 0.0); 40]);
            total += output.len();
        }
        // 400 samples at 4 sps → ~100 symbols
        assert!(total >= 80 && total <= 120,
            "Expected ~100 symbols, got {}", total);
    }

    #[test]
    fn test_reset() {
        let mut sync = SymbolSync::new(4.0, TedType::Gardner, 0.045);
        sync.process(&vec![Complex64::new(1.0, 0.0); 50]);
        sync.reset();
        assert!((sync.omega() - 4.0).abs() < 1e-10);
    }
}
