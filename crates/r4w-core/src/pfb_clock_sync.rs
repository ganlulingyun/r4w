//! Polyphase Filterbank Clock Synchronizer
//!
//! Industrial-grade symbol timing recovery using a polyphase filterbank
//! with matched filter and derivative matched filter for timing error detection.
//!
//! ## Algorithm
//!
//! 1. Input samples are fed through a polyphase filter bank (N filter arms)
//! 2. The matched filter output provides the interpolated symbol sample
//! 3. The derivative filter output provides the timing error signal
//! 4. A PI loop filter adjusts the filter arm index and fractional offset
//!
//! This is the standard approach used in GNU Radio's `pfb_clock_sync_ccf`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::pfb_clock_sync::PfbClockSync;
//! use num_complex::Complex64;
//!
//! let sps = 4.0;
//! let mut sync = PfbClockSync::new(sps, 0.0628, 32);
//! let input = vec![Complex64::new(1.0, 0.0); 200];
//! let symbols = sync.process(&input);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Polyphase filterbank clock synchronizer.
#[derive(Debug, Clone)]
pub struct PfbClockSync {
    /// Samples per symbol
    sps: f64,
    /// Number of filter arms (= number of polyphase branches)
    nfilts: usize,
    /// Matched filter coefficients per arm
    filters: Vec<Vec<f64>>,
    /// Derivative filter coefficients per arm
    diff_filters: Vec<Vec<f64>>,
    /// Delay line for polyphase filtering
    delay_line: Vec<Complex64>,
    /// Taps per arm
    taps_per_arm: usize,
    /// Current filter arm index
    arm_index: usize,
    /// Fractional arm offset
    arm_frac: f64,
    /// Rate accumulator (how fast we advance through arms)
    rate: f64,
    /// Nominal rate
    rate_nominal: f64,
    /// Loop filter: proportional gain
    alpha: f64,
    /// Loop filter: integral gain
    beta: f64,
    /// Maximum rate deviation
    max_rate_deviation: f64,
    /// Previous symbol for TED
    prev_symbol: Complex64,
    /// Sample counter for decimation
    sample_count: usize,
    /// Samples per symbol as integer for decimation
    osps: usize,
}

impl PfbClockSync {
    /// Create a new PFB clock synchronizer.
    ///
    /// - `sps`: Samples per symbol (typically 2-8)
    /// - `loop_bw`: Loop bandwidth (typically 2*PI/100 = 0.0628)
    /// - `nfilts`: Number of polyphase filter arms (typically 32)
    pub fn new(sps: f64, loop_bw: f64, nfilts: usize) -> Self {
        assert!(sps >= 1.5, "SPS must be >= 1.5");
        assert!(nfilts >= 2, "Must have at least 2 filter arms");

        let osps = sps.round() as usize;
        let taps_per_arm = (sps.ceil() as usize).max(2);
        let total_taps = nfilts * taps_per_arm;

        // Design root-raised-cosine matched filter
        let prototype = Self::design_rrc(total_taps, nfilts as f64 * sps, 0.35);

        // Build polyphase filter bank
        let mut filters = vec![vec![0.0; taps_per_arm]; nfilts];
        for i in 0..total_taps {
            let arm = i % nfilts;
            let tap = i / nfilts;
            if tap < taps_per_arm {
                filters[arm][tap] = prototype[i];
            }
        }

        // Build derivative filter bank (finite differences of matched filter)
        let mut diff_filters = vec![vec![0.0; taps_per_arm]; nfilts];
        for arm in 0..nfilts {
            for tap in 0..taps_per_arm {
                let next_arm = (arm + 1) % nfilts;
                let prev_arm = if arm == 0 { nfilts - 1 } else { arm - 1 };
                diff_filters[arm][tap] =
                    (filters[next_arm][tap] - filters[prev_arm][tap]) / 2.0;
            }
        }

        // PI loop filter gains
        let denom = 1.0 + 2.0 * 0.707 * loop_bw + loop_bw * loop_bw;
        let alpha = 4.0 * 0.707 * loop_bw / denom / nfilts as f64;
        let beta = 4.0 * loop_bw * loop_bw / denom / nfilts as f64;

        Self {
            sps,
            nfilts,
            filters,
            diff_filters,
            delay_line: vec![Complex64::new(0.0, 0.0); taps_per_arm],
            taps_per_arm,
            arm_index: nfilts / 2,
            arm_frac: 0.0,
            rate: nfilts as f64 / sps,
            rate_nominal: nfilts as f64 / sps,
            alpha,
            beta,
            max_rate_deviation: 0.1,
            prev_symbol: Complex64::new(0.0, 0.0),
            sample_count: 0,
            osps,
        }
    }

    /// Process input samples, returning synchronized symbols.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::new();

        for &sample in input {
            // Shift new sample into delay line
            self.delay_line.pop();
            self.delay_line.insert(0, sample);

            self.sample_count += 1;
            if self.sample_count < self.osps {
                continue;
            }
            self.sample_count = 0;

            // Filter with current arm
            let arm = self.arm_index.min(self.nfilts - 1);
            let symbol = self.filter_arm(arm);
            let deriv = self.diff_filter_arm(arm);

            // Timing error = Re{deriv * conj(symbol)}
            let error = (deriv * symbol.conj()).re;

            // Update loop
            self.arm_frac += self.alpha * error;
            self.rate += self.beta * error;

            // Clamp rate
            let rate_min = self.rate_nominal * (1.0 - self.max_rate_deviation);
            let rate_max = self.rate_nominal * (1.0 + self.max_rate_deviation);
            self.rate = self.rate.clamp(rate_min, rate_max);

            // Advance arm index
            let advance = self.arm_frac + self.rate;
            let arm_step = advance.floor() as i64;
            self.arm_frac = advance - arm_step as f64;

            let new_arm = self.arm_index as i64 + arm_step;
            if new_arm >= self.nfilts as i64 {
                self.arm_index = (new_arm % self.nfilts as i64) as usize;
                // Skip a sample (we went past all arms)
                self.sample_count = self.osps.saturating_sub(1);
            } else if new_arm < 0 {
                self.arm_index = ((new_arm % self.nfilts as i64) + self.nfilts as i64) as usize;
            } else {
                self.arm_index = new_arm as usize;
            }

            self.prev_symbol = symbol;
            output.push(symbol);
        }

        output
    }

    /// Apply matched filter for a given arm.
    fn filter_arm(&self, arm: usize) -> Complex64 {
        let taps = &self.filters[arm];
        let mut sum = Complex64::new(0.0, 0.0);
        for (i, &tap) in taps.iter().enumerate() {
            if i < self.delay_line.len() {
                sum += self.delay_line[i] * tap;
            }
        }
        sum
    }

    /// Apply derivative filter for a given arm.
    fn diff_filter_arm(&self, arm: usize) -> Complex64 {
        let taps = &self.diff_filters[arm];
        let mut sum = Complex64::new(0.0, 0.0);
        for (i, &tap) in taps.iter().enumerate() {
            if i < self.delay_line.len() {
                sum += self.delay_line[i] * tap;
            }
        }
        sum
    }

    /// Design a root-raised cosine filter.
    fn design_rrc(num_taps: usize, sps: f64, rolloff: f64) -> Vec<f64> {
        let mut taps = vec![0.0; num_taps];
        let mid = (num_taps - 1) as f64 / 2.0;

        for i in 0..num_taps {
            let t = (i as f64 - mid) / sps;
            if t.abs() < 1e-12 {
                taps[i] = 1.0 - rolloff + 4.0 * rolloff / PI;
            } else if (t.abs() - 1.0 / (4.0 * rolloff)).abs() < 1e-12 {
                taps[i] = rolloff / (2.0_f64).sqrt()
                    * ((1.0 + 2.0 / PI) * (PI / (4.0 * rolloff)).sin()
                        + (1.0 - 2.0 / PI) * (PI / (4.0 * rolloff)).cos());
            } else {
                let pi_t = PI * t;
                let num = (pi_t * (1.0 - rolloff)).sin()
                    + 4.0 * rolloff * t * (pi_t * (1.0 + rolloff)).cos();
                let den = pi_t * (1.0 - (4.0 * rolloff * t).powi(2));
                if den.abs() > 1e-20 {
                    taps[i] = num / den;
                }
            }
        }

        // Normalize energy
        let energy: f64 = taps.iter().map(|&t| t * t).sum();
        if energy > 0.0 {
            let scale = 1.0 / energy.sqrt();
            for tap in &mut taps {
                *tap *= scale;
            }
        }

        taps
    }

    /// Get current timing estimate (filter arm index / nfilts).
    pub fn timing_offset(&self) -> f64 {
        self.arm_index as f64 / self.nfilts as f64
    }

    /// Get number of filter arms.
    pub fn nfilts(&self) -> usize {
        self.nfilts
    }

    /// Get the current rate estimate.
    pub fn rate(&self) -> f64 {
        self.rate * self.sps / self.nfilts as f64
    }

    /// Reset the synchronizer state.
    pub fn reset(&mut self) {
        self.arm_index = self.nfilts / 2;
        self.arm_frac = 0.0;
        self.rate = self.rate_nominal;
        self.prev_symbol = Complex64::new(0.0, 0.0);
        self.sample_count = 0;
        self.delay_line.fill(Complex64::new(0.0, 0.0));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_construction() {
        let sync = PfbClockSync::new(4.0, 0.0628, 32);
        assert_eq!(sync.nfilts(), 32);
        assert!(sync.timing_offset() >= 0.0 && sync.timing_offset() <= 1.0);
    }

    #[test]
    fn test_dc_signal() {
        let mut sync = PfbClockSync::new(4.0, 0.0628, 32);
        let input = vec![Complex64::new(1.0, 0.0); 400];
        let output = sync.process(&input);
        // ~400/4 = 100 symbols
        assert!(
            output.len() >= 70 && output.len() <= 130,
            "Expected ~100 symbols, got {}",
            output.len()
        );
    }

    #[test]
    fn test_bpsk_signal() {
        let mut sync = PfbClockSync::new(4.0, 0.0628, 32);
        let sps = 4;
        let n_sym = 200;
        let mut input = Vec::new();
        for i in 0..n_sym {
            let val = if i % 2 == 0 { 1.0 } else { -1.0 };
            for _ in 0..sps {
                input.push(Complex64::new(val, 0.0));
            }
        }
        let output = sync.process(&input);
        assert!(
            output.len() >= 150 && output.len() <= 250,
            "Expected ~200 symbols, got {}",
            output.len()
        );
    }

    #[test]
    fn test_qpsk_signal() {
        let mut sync = PfbClockSync::new(4.0, 0.0628, 32);
        let sps = 4;
        let symbols = [
            Complex64::new(1.0, 1.0),
            Complex64::new(-1.0, 1.0),
            Complex64::new(-1.0, -1.0),
            Complex64::new(1.0, -1.0),
        ];
        let mut input = Vec::new();
        for i in 0..100 {
            let sym = symbols[i % 4];
            for _ in 0..sps {
                input.push(sym);
            }
        }
        let output = sync.process(&input);
        assert!(
            output.len() >= 70 && output.len() <= 130,
            "Expected ~100 symbols, got {}",
            output.len()
        );
    }

    #[test]
    fn test_incremental() {
        let mut sync = PfbClockSync::new(4.0, 0.0628, 32);
        let mut total = 0;
        for _ in 0..10 {
            let chunk = vec![Complex64::new(1.0, 0.0); 40];
            total += sync.process(&chunk).len();
        }
        // 400 samples at 4 sps → ~100 symbols
        assert!(
            total >= 70 && total <= 130,
            "Expected ~100 symbols, got {}",
            total
        );
    }

    #[test]
    fn test_different_nfilts() {
        let mut sync16 = PfbClockSync::new(4.0, 0.0628, 16);
        let mut sync64 = PfbClockSync::new(4.0, 0.0628, 64);
        let input = vec![Complex64::new(1.0, 0.0); 200];
        let out16 = sync16.process(&input);
        let out64 = sync64.process(&input);
        // Both should produce similar number of symbols
        assert!(!out16.is_empty());
        assert!(!out64.is_empty());
    }

    #[test]
    fn test_reset() {
        let mut sync = PfbClockSync::new(4.0, 0.0628, 32);
        sync.process(&vec![Complex64::new(1.0, 0.0); 100]);
        sync.reset();
        assert_eq!(sync.arm_index, 16); // nfilts/2
        assert_eq!(sync.sample_count, 0);
    }

    #[test]
    fn test_rrc_filter_design() {
        let taps = PfbClockSync::design_rrc(64, 4.0, 0.35);
        assert_eq!(taps.len(), 64);
        // Energy should be normalized
        let energy: f64 = taps.iter().map(|&t| t * t).sum();
        assert!(
            (energy - 1.0).abs() < 0.1,
            "Filter energy {} not near 1.0",
            energy
        );
    }
}
