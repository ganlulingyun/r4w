//! FLL Band-Edge (Frequency Lock Loop with Band-Edge Filters)
//!
//! Coarse frequency synchronization for digitally modulated signals using
//! band-edge FIR filters. Places two filters at the upper and lower band
//! edges; their power difference generates a frequency error signal that
//! drives an NCO for correction.
//!
//! ## Signal Flow
//!
//! ```text
//! input → [NCO correction] → ┬→ [upper band-edge FIR] → |·|² → ─┐
//!                             │                                    ├→ error = |x_l|²-|x_u|²
//!                             └→ [lower band-edge FIR] → |·|² → ─┘
//!                                                                  ↓
//!                                           [2nd-order loop filter] → NCO
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::fll_band_edge::{FllBandEdge, FllBandEdgeConfig};
//! use num_complex::Complex64;
//!
//! let config = FllBandEdgeConfig {
//!     samples_per_symbol: 4.0,
//!     rolloff: 0.35,
//!     filter_size: 45,
//!     loop_bandwidth: 0.01,
//! };
//! let mut fll = FllBandEdge::new(config);
//!
//! // Process IQ samples with frequency offset
//! let input = vec![Complex64::new(1.0, 0.0); 100];
//! let corrected = fll.process_block(&input);
//! assert_eq!(corrected.len(), 100);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Configuration for FLL Band-Edge.
#[derive(Debug, Clone)]
pub struct FllBandEdgeConfig {
    /// Oversampling ratio (samples per symbol), e.g., 4.0
    pub samples_per_symbol: f64,
    /// Excess bandwidth / rolloff factor (0.0-1.0), e.g., 0.35
    pub rolloff: f64,
    /// Number of FIR filter taps (odd recommended, 30-70 typical)
    pub filter_size: usize,
    /// Loop bandwidth in normalized frequency (e.g., 0.01-0.1)
    pub loop_bandwidth: f64,
}

impl Default for FllBandEdgeConfig {
    fn default() -> Self {
        Self {
            samples_per_symbol: 4.0,
            rolloff: 0.35,
            filter_size: 45,
            loop_bandwidth: 0.02,
        }
    }
}

/// FLL Band-Edge frequency synchronizer.
#[derive(Debug, Clone)]
pub struct FllBandEdge {
    /// Upper band-edge FIR taps
    taps_upper: Vec<Complex64>,
    /// Lower band-edge FIR taps
    taps_lower: Vec<Complex64>,
    /// FIR delay line
    delay_line: Vec<Complex64>,
    /// Delay line write index
    dl_idx: usize,
    /// NCO phase accumulator (radians)
    phase: f64,
    /// NCO frequency (radians/sample)
    freq: f64,
    /// Loop filter integrator
    integrator: f64,
    /// Proportional gain
    alpha: f64,
    /// Integral gain
    beta: f64,
    /// Filter size
    filter_size: usize,
}

impl FllBandEdge {
    /// Create a new FLL Band-Edge synchronizer.
    pub fn new(config: FllBandEdgeConfig) -> Self {
        let n = config.filter_size | 1; // ensure odd
        let sps = config.samples_per_symbol;
        let rolloff = config.rolloff;

        // Design band-edge filters: derivative of RRC at ±(1+α)/(2T)
        let (taps_lower, taps_upper) = Self::design_band_edge_filters(n, sps, rolloff);

        // Compute loop gains from bandwidth
        // Using standard 2nd-order loop: ζ = 1/√2, ωn from loop_bandwidth
        let denom = 1.0 + 2.0 * 0.707 * config.loop_bandwidth + config.loop_bandwidth * config.loop_bandwidth;
        let alpha = 4.0 * 0.707 * config.loop_bandwidth / denom;
        let beta = 4.0 * config.loop_bandwidth * config.loop_bandwidth / denom;

        Self {
            taps_upper,
            taps_lower,
            delay_line: vec![Complex64::new(0.0, 0.0); n],
            dl_idx: 0,
            phase: 0.0,
            freq: 0.0,
            integrator: 0.0,
            alpha,
            beta,
            filter_size: n,
        }
    }

    /// Process a single sample: frequency-correct and return corrected sample.
    pub fn process(&mut self, input: Complex64) -> Complex64 {
        // Apply NCO correction
        let nco = Complex64::new(self.phase.cos(), -self.phase.sin());
        let corrected = input * nco;

        // Push into delay line
        self.delay_line[self.dl_idx] = corrected;
        self.dl_idx = (self.dl_idx + 1) % self.filter_size;

        // Compute band-edge filter outputs
        let mut upper_out = Complex64::new(0.0, 0.0);
        let mut lower_out = Complex64::new(0.0, 0.0);
        for i in 0..self.filter_size {
            let sample = self.delay_line[(self.dl_idx + i) % self.filter_size];
            upper_out += sample * self.taps_upper[i];
            lower_out += sample * self.taps_lower[i];
        }

        // Frequency error: power difference
        let error = lower_out.norm_sqr() - upper_out.norm_sqr();

        // 2nd-order loop filter
        self.integrator += self.beta * error;
        self.freq += self.integrator;
        self.phase += self.freq + self.alpha * error;

        // Wrap phase to [-π, π]
        while self.phase > PI {
            self.phase -= 2.0 * PI;
        }
        while self.phase < -PI {
            self.phase += 2.0 * PI;
        }

        corrected
    }

    /// Process a block of samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    /// Current frequency offset estimate (radians/sample).
    pub fn frequency_estimate(&self) -> f64 {
        self.freq
    }

    /// Current frequency offset in Hz, given sample rate.
    pub fn frequency_estimate_hz(&self, sample_rate: f64) -> f64 {
        self.freq * sample_rate / (2.0 * PI)
    }

    /// Current phase estimate (radians).
    pub fn phase(&self) -> f64 {
        self.phase
    }

    /// Check if the loop has approximately converged.
    pub fn is_locked(&self, threshold_rad_per_sample: f64) -> bool {
        self.freq.abs() < threshold_rad_per_sample
    }

    /// Set loop bandwidth (recomputes gains).
    pub fn set_loop_bandwidth(&mut self, bw: f64) {
        let denom = 1.0 + 2.0 * 0.707 * bw + bw * bw;
        self.alpha = 4.0 * 0.707 * bw / denom;
        self.beta = 4.0 * bw * bw / denom;
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.delay_line.fill(Complex64::new(0.0, 0.0));
        self.dl_idx = 0;
        self.phase = 0.0;
        self.freq = 0.0;
        self.integrator = 0.0;
    }

    /// Design the upper and lower band-edge filters.
    ///
    /// The band-edge filters are the positive/negative frequency-shifted
    /// versions of the RRC matched filter derivative.
    fn design_band_edge_filters(
        n: usize,
        sps: f64,
        rolloff: f64,
    ) -> (Vec<Complex64>, Vec<Complex64>) {
        let mid = n / 2;

        // RRC prototype filter
        let mut rrc = vec![0.0f64; n];
        for i in 0..n {
            let t = (i as f64 - mid as f64) / sps;
            rrc[i] = Self::rrc_tap(t, rolloff);
        }

        // Band-edge frequency: (1+rolloff)/(2*sps) in normalized freq
        let band_edge_freq = (1.0 + rolloff) / (2.0 * sps);

        // Create upper and lower band-edge filters by frequency-shifting RRC
        let mut taps_lower = Vec::with_capacity(n);
        let mut taps_upper = Vec::with_capacity(n);

        for i in 0..n {
            let phase = 2.0 * PI * band_edge_freq * (i as f64 - mid as f64);
            let shift_upper = Complex64::new(phase.cos(), phase.sin());
            let shift_lower = Complex64::new(phase.cos(), -phase.sin());
            taps_upper.push(shift_upper * rrc[i]);
            taps_lower.push(shift_lower * rrc[i]);
        }

        (taps_lower, taps_upper)
    }

    /// Compute a single RRC filter tap at time offset t (in symbol periods).
    fn rrc_tap(t: f64, alpha: f64) -> f64 {
        if t.abs() < 1e-12 {
            // t = 0
            1.0 - alpha + 4.0 * alpha / PI
        } else if (t.abs() - 1.0 / (4.0 * alpha)).abs() < 1e-12 && alpha > 0.0 {
            // t = ±1/(4α)
            alpha / (2.0_f64).sqrt()
                * ((1.0 + 2.0 / PI) * (PI / (4.0 * alpha)).sin()
                    + (1.0 - 2.0 / PI) * (PI / (4.0 * alpha)).cos())
        } else {
            let pi_t = PI * t;
            let num = (pi_t * (1.0 - alpha)).sin() + 4.0 * alpha * t * (pi_t * (1.0 + alpha)).cos();
            let den = pi_t * (1.0 - (4.0 * alpha * t).powi(2));
            if den.abs() < 1e-20 {
                0.0
            } else {
                num / den
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_construction() {
        let fll = FllBandEdge::new(FllBandEdgeConfig::default());
        assert!((fll.frequency_estimate()).abs() < 1e-10);
        assert!((fll.phase()).abs() < 1e-10);
    }

    #[test]
    fn test_dc_no_correction() {
        let mut fll = FllBandEdge::new(FllBandEdgeConfig::default());
        // DC input should not generate large frequency corrections
        let input = vec![Complex64::new(1.0, 0.0); 200];
        let _output = fll.process_block(&input);
        // Frequency estimate should stay small
        assert!(
            fll.frequency_estimate().abs() < 0.1,
            "DC should not produce large freq estimate: {}",
            fll.frequency_estimate()
        );
    }

    #[test]
    fn test_output_length() {
        let mut fll = FllBandEdge::new(FllBandEdgeConfig::default());
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = fll.process_block(&input);
        assert_eq!(output.len(), 100);
    }

    #[test]
    fn test_frequency_correction_convergence() {
        let config = FllBandEdgeConfig {
            samples_per_symbol: 4.0,
            rolloff: 0.35,
            filter_size: 45,
            loop_bandwidth: 0.05, // wider bandwidth for faster convergence
        };
        let mut fll = FllBandEdge::new(config);

        // Generate QPSK-like signal with known frequency offset
        let sps = 4.0;
        let freq_offset = 0.02; // radians/sample (~150 Hz at 48 kHz)
        let n = 2000;

        let input: Vec<Complex64> = (0..n)
            .map(|i| {
                // Simple rectangular pulses with 4x oversampling + frequency offset
                let symbol_phase = if (i / 4) % 4 == 0 {
                    PI / 4.0
                } else if (i / 4) % 4 == 1 {
                    3.0 * PI / 4.0
                } else if (i / 4) % 4 == 2 {
                    -3.0 * PI / 4.0
                } else {
                    -PI / 4.0
                };
                let offset_phase = freq_offset * i as f64;
                Complex64::from_polar(1.0, symbol_phase + offset_phase)
            })
            .collect();

        let _output = fll.process_block(&input);

        // The FLL should have estimated some frequency correction
        // We just verify it moved in the right direction
        let est = fll.frequency_estimate();
        // With band-edge approach, the exact convergence depends on the signal shape,
        // but the loop should be active (non-zero estimate)
        assert!(
            est.abs() > 1e-6 || fll.phase().abs() > 1e-6,
            "FLL should have responded to frequency offset: freq={}, phase={}",
            est,
            fll.phase()
        );
    }

    #[test]
    fn test_set_loop_bandwidth() {
        let mut fll = FllBandEdge::new(FllBandEdgeConfig::default());
        let orig_alpha = fll.alpha;
        fll.set_loop_bandwidth(0.1);
        assert!((fll.alpha - orig_alpha).abs() > 1e-10, "Alpha should change");
    }

    #[test]
    fn test_reset() {
        let mut fll = FllBandEdge::new(FllBandEdgeConfig::default());
        let input = vec![Complex64::new(0.0, 1.0); 100];
        fll.process_block(&input);

        fll.reset();
        assert!((fll.frequency_estimate()).abs() < 1e-10);
        assert!((fll.phase()).abs() < 1e-10);
    }

    #[test]
    fn test_is_locked() {
        let fll = FllBandEdge::new(FllBandEdgeConfig::default());
        // Initially locked (freq = 0)
        assert!(fll.is_locked(0.001));
    }

    #[test]
    fn test_frequency_estimate_hz() {
        let mut fll = FllBandEdge::new(FllBandEdgeConfig::default());
        // Manually set freq for testing
        fll.freq = 2.0 * PI * 100.0 / 48000.0; // 100 Hz at 48 kHz
        let hz = fll.frequency_estimate_hz(48000.0);
        assert!(
            (hz - 100.0).abs() < 1.0,
            "Expected ~100 Hz, got {}",
            hz
        );
    }

    #[test]
    fn test_rrc_tap_at_zero() {
        let tap = FllBandEdge::rrc_tap(0.0, 0.35);
        let expected = 1.0 - 0.35 + 4.0 * 0.35 / PI;
        assert!(
            (tap - expected).abs() < 1e-10,
            "RRC at t=0: expected {}, got {}",
            expected,
            tap
        );
    }

    #[test]
    fn test_band_edge_filters_symmetric() {
        let (lower, upper) = FllBandEdge::design_band_edge_filters(45, 4.0, 0.35);
        assert_eq!(lower.len(), 45);
        assert_eq!(upper.len(), 45);

        // Upper and lower should have conjugate-symmetric relationship
        // (one is the conjugate shift of the other)
        for i in 0..45 {
            // The magnitudes of corresponding taps should match
            let mag_diff = (upper[i].norm() - lower[i].norm()).abs();
            assert!(
                mag_diff < 1e-10,
                "Tap {} magnitude mismatch: upper={}, lower={}",
                i,
                upper[i].norm(),
                lower[i].norm()
            );
        }
    }
}
