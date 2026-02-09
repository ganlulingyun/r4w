//! Mueller & Müller Clock Recovery
//!
//! Symbol timing recovery using the Mueller & Müller (M&M) timing error
//! detector. Works on PAM/FSK/BPSK signals and is commonly used in
//! packet radio (APRS, AX.25) receivers after FM demodulation.
//!
//! ## Algorithm
//!
//! The M&M TED computes the timing error from consecutive symbol samples:
//! ```text
//! e[n] = y[n-1] * (y[n] - y[n-2])
//! ```
//! where y[n] is the current symbol sample and y[n-1], y[n-2] are previous.
//!
//! A PI (proportional-integral) loop filter adjusts the sampling phase.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::clock_recovery_mm::ClockRecoveryMM;
//!
//! let mut cr = ClockRecoveryMM::new(4.0, 0.01); // 4 samples/symbol, loop BW 0.01
//! let input = vec![1.0; 100]; // DC signal
//! let symbols = cr.process(&input);
//! // Output has approximately input.len() / sps symbols
//! ```

/// Mueller & Müller Clock Recovery.
#[derive(Debug, Clone)]
pub struct ClockRecoveryMM {
    /// Nominal samples per symbol
    omega: f64,
    /// Current samples per symbol estimate
    omega_mid: f64,
    /// Sample counter
    mu: f64,
    /// Loop gain: proportional
    gain_mu: f64,
    /// Loop gain: integral (omega adjustment)
    gain_omega: f64,
    /// Omega limits
    omega_relative_limit: f64,
    /// Previous symbol samples for TED
    d_prev: f64,
    d_prev_prev: f64,
    /// Interpolation buffer
    buffer: Vec<f64>,
    /// Read position
    read_pos: usize,
}

impl ClockRecoveryMM {
    /// Create a new M&M clock recovery.
    ///
    /// - `sps`: Nominal samples per symbol
    /// - `loop_bw`: Loop bandwidth (typically 0.01..0.1)
    pub fn new(sps: f64, loop_bw: f64) -> Self {
        assert!(sps > 1.0, "Samples per symbol must be > 1");
        // Compute loop gains from bandwidth
        let denom = 1.0 + 2.0 * 0.707 * loop_bw + loop_bw * loop_bw;
        let gain_mu = 4.0 * 0.707 * loop_bw / denom;
        let gain_omega = 4.0 * loop_bw * loop_bw / denom;

        Self {
            omega: sps,
            omega_mid: sps,
            mu: 0.0,
            gain_mu,
            gain_omega,
            omega_relative_limit: 0.005,
            d_prev: 0.0,
            d_prev_prev: 0.0,
            buffer: Vec::new(),
            read_pos: 0,
        }
    }

    /// Process a block of real samples and output recovered symbols.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        self.buffer.extend_from_slice(input);
        let mut output = Vec::new();

        while self.read_pos + 2 < self.buffer.len() {
            // Linear interpolation at current mu
            let idx = self.read_pos;
            if idx + 1 >= self.buffer.len() {
                break;
            }

            let frac = self.mu;
            let sample = self.buffer[idx] * (1.0 - frac) + self.buffer[idx + 1] * frac;

            // M&M TED: e = d_prev * (sample - d_prev_prev)
            let error = self.d_prev * (sample - self.d_prev_prev);

            // Update state
            self.d_prev_prev = self.d_prev;
            self.d_prev = sample;

            // Loop filter: adjust omega and mu
            self.omega += self.gain_omega * error;

            // Clamp omega
            let omega_min = self.omega_mid * (1.0 - self.omega_relative_limit);
            let omega_max = self.omega_mid * (1.0 + self.omega_relative_limit);
            self.omega = self.omega.clamp(omega_min, omega_max);

            self.mu += self.omega + self.gain_mu * error;

            // Advance read position by integer part of mu
            let advance = self.mu as usize;
            self.read_pos += advance;
            self.mu -= advance as f64;

            output.push(sample);
        }

        // Remove consumed samples
        if self.read_pos > 0 {
            let consumed = self.read_pos.min(self.buffer.len());
            self.buffer.drain(..consumed);
            self.read_pos = 0;
        }

        output
    }

    /// Get the current samples-per-symbol estimate.
    pub fn omega(&self) -> f64 {
        self.omega
    }

    /// Get the current fractional timing offset.
    pub fn mu(&self) -> f64 {
        self.mu
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.omega = self.omega_mid;
        self.mu = 0.0;
        self.d_prev = 0.0;
        self.d_prev_prev = 0.0;
        self.buffer.clear();
        self.read_pos = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_basic_construction() {
        let cr = ClockRecoveryMM::new(4.0, 0.01);
        assert!((cr.omega() - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_dc_signal() {
        let mut cr = ClockRecoveryMM::new(4.0, 0.01);
        let input = vec![1.0; 100];
        let output = cr.process(&input);
        // Should produce approximately 100/4 = 25 symbols
        assert!(output.len() >= 20 && output.len() <= 30,
            "Expected ~25 symbols, got {}", output.len());
    }

    #[test]
    fn test_alternating_symbols() {
        let mut cr = ClockRecoveryMM::new(4.0, 0.05);
        let sps = 4;
        let n_symbols = 50;
        // Generate alternating +1/-1 at 4 samples/symbol
        let mut input = Vec::with_capacity(sps * n_symbols);
        for i in 0..n_symbols {
            let val = if i % 2 == 0 { 1.0 } else { -1.0 };
            for _ in 0..sps {
                input.push(val);
            }
        }
        let output = cr.process(&input);
        // Should recover approximately n_symbols
        assert!(output.len() >= 40 && output.len() <= 60,
            "Expected ~50 symbols, got {}", output.len());
    }

    #[test]
    fn test_sinusoidal_input() {
        let mut cr = ClockRecoveryMM::new(8.0, 0.01);
        let n = 200;
        let input: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * i as f64 / 8.0).sin())
            .collect();
        let output = cr.process(&input);
        assert!(output.len() >= 20 && output.len() <= 30,
            "Expected ~25 symbols, got {}", output.len());
    }

    #[test]
    fn test_output_rate() {
        let mut cr = ClockRecoveryMM::new(10.0, 0.01);
        let input = vec![1.0; 500];
        let output = cr.process(&input);
        // Should be approximately 500/10 = 50
        assert!(output.len() >= 40 && output.len() <= 60,
            "Expected ~50 symbols, got {}", output.len());
    }

    #[test]
    fn test_reset() {
        let mut cr = ClockRecoveryMM::new(4.0, 0.01);
        cr.process(&vec![1.0; 50]);
        cr.reset();
        assert!((cr.omega() - 4.0).abs() < 1e-10);
        assert!((cr.mu() - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_incremental_processing() {
        let mut cr = ClockRecoveryMM::new(4.0, 0.01);
        let mut total = 0;
        for _ in 0..10 {
            let output = cr.process(&[1.0; 40]);
            total += output.len();
        }
        // 400 samples at 4 sps → ~100 symbols
        assert!(total >= 80 && total <= 120,
            "Expected ~100 symbols, got {}", total);
    }
}
