//! Symbol Timing Recovery (Clock Recovery)
//!
//! Implements Mueller & Muller timing error detector with a proportional-integral
//! loop filter for symbol clock recovery. This is essential for any digital
//! receiver to sample at the correct instant within each symbol period.
//!
//! ## Algorithm
//!
//! The Mueller & Muller (M&M) timing error detector computes:
//! ```text
//! e[n] = Re{ conj(x[n]) * x[n-1] - conj(x[n-1]) * x[n-2] * d[n-1] }
//! ```
//! Where x[n] are the interpolated samples and d[n] are the decision symbols.
//!
//! The timing error drives a loop filter that adjusts the interpolation point.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::clock_recovery::{MuellerMuller, MuellerMullerConfig};
//! use num_complex::Complex64;
//!
//! let config = MuellerMullerConfig {
//!     sps: 4.0,           // 4 samples per symbol
//!     loop_bw: 0.01,      // Loop bandwidth
//!     damping: 0.707,     // Critically damped
//!     max_deviation: 1.5,  // Max timing offset in samples
//! };
//! let mut mm = MuellerMuller::new(config);
//!
//! // Feed oversampled signal, get symbol-rate output
//! let oversampled = vec![Complex64::new(1.0, 0.0); 400]; // 100 symbols at 4 sps
//! let (symbols, _timing_error) = mm.process_block(&oversampled);
//! ```

use num_complex::Complex64;

/// Configuration for Mueller & Muller clock recovery.
#[derive(Debug, Clone)]
pub struct MuellerMullerConfig {
    /// Samples per symbol (oversampling ratio, e.g., 4.0)
    pub sps: f64,
    /// Loop bandwidth (radians per symbol, typical: 0.005 to 0.05)
    pub loop_bw: f64,
    /// Damping factor (typical: 0.707)
    pub damping: f64,
    /// Maximum timing deviation in samples
    pub max_deviation: f64,
}

impl Default for MuellerMullerConfig {
    fn default() -> Self {
        Self {
            sps: 4.0,
            loop_bw: 0.01,
            damping: 0.707,
            max_deviation: 1.5,
        }
    }
}

/// Mueller & Muller symbol timing recovery.
///
/// Takes oversampled complex baseband input and outputs symbol-rate samples
/// at the optimal sampling instants.
#[derive(Debug, Clone)]
pub struct MuellerMuller {
    config: MuellerMullerConfig,
    /// Fractional sample offset (mu)
    mu: f64,
    /// Sample counter within symbol
    sample_count: f64,
    /// Previous symbol estimate
    prev_symbol: Complex64,
    /// Symbol before previous
    prev_prev_symbol: Complex64,
    /// Previous decision
    prev_decision: Complex64,
    /// Proportional gain (alpha)
    alpha: f64,
    /// Integral gain (beta)
    beta: f64,
    /// Accumulated timing adjustment
    timing_offset: f64,
}

impl MuellerMuller {
    /// Create a new Mueller & Muller clock recovery block.
    pub fn new(config: MuellerMullerConfig) -> Self {
        // Compute loop filter gains
        let bw = config.loop_bw;
        let damp = config.damping;
        let denom = 1.0 + 2.0 * damp * bw + bw * bw;
        let alpha = (4.0 * damp * bw) / denom;
        let beta = (4.0 * bw * bw) / denom;

        Self {
            config,
            mu: 0.0,
            sample_count: 0.0,
            prev_symbol: Complex64::new(0.0, 0.0),
            prev_prev_symbol: Complex64::new(0.0, 0.0),
            prev_decision: Complex64::new(0.0, 0.0),
            alpha,
            beta,
            timing_offset: 0.0,
        }
    }

    /// Get the current timing offset in samples.
    pub fn timing_offset(&self) -> f64 {
        self.timing_offset
    }

    /// Get the current fractional sample offset.
    pub fn mu(&self) -> f64 {
        self.mu
    }

    /// Make a hard decision on a complex sample (nearest QPSK point).
    fn decide(sample: Complex64) -> Complex64 {
        let re = if sample.re >= 0.0 { 1.0 } else { -1.0 };
        let im = if sample.im >= 0.0 { 1.0 } else { -1.0 };
        Complex64::new(re, im)
    }

    /// Linear interpolation between two samples.
    fn interpolate(s0: Complex64, s1: Complex64, mu: f64) -> Complex64 {
        s0 * (1.0 - mu) + s1 * mu
    }

    /// Process a block of oversampled input, producing symbol-rate output.
    ///
    /// Returns (symbols, timing_errors) where timing_errors can be used
    /// for diagnostics/plotting.
    pub fn process_block(&mut self, input: &[Complex64]) -> (Vec<Complex64>, Vec<f64>) {
        let mut symbols = Vec::new();
        let mut timing_errors = Vec::new();
        let sps = self.config.sps;

        let mut i = 0usize;
        while i + 1 < input.len() {
            self.sample_count += 1.0;

            if self.sample_count >= sps + self.timing_offset {
                self.sample_count -= sps + self.timing_offset;
                self.timing_offset = 0.0;

                // Interpolate at the estimated optimal point
                let mu_clamped = self.mu.clamp(0.0, 1.0);
                let interp = if i + 1 < input.len() {
                    Self::interpolate(input[i], input[i + 1], mu_clamped)
                } else {
                    input[i]
                };

                // Mueller & Muller timing error
                let decision = Self::decide(interp);
                let error = (interp.conj() * self.prev_decision
                    - self.prev_symbol.conj() * decision)
                    .re;

                // Update loop filter
                self.timing_offset = self.alpha * error;
                self.mu += self.beta * error;

                // Clamp mu
                if self.mu > self.config.max_deviation {
                    self.mu = self.config.max_deviation;
                }
                if self.mu < -self.config.max_deviation {
                    self.mu = -self.config.max_deviation;
                }

                // Store for next iteration
                self.prev_prev_symbol = self.prev_symbol;
                self.prev_symbol = interp;
                self.prev_decision = decision;

                symbols.push(interp);
                timing_errors.push(error);
            }

            i += 1;
        }

        (symbols, timing_errors)
    }

    /// Reset the clock recovery to initial state.
    pub fn reset(&mut self) {
        self.mu = 0.0;
        self.sample_count = 0.0;
        self.prev_symbol = Complex64::new(0.0, 0.0);
        self.prev_prev_symbol = Complex64::new(0.0, 0.0);
        self.prev_decision = Complex64::new(0.0, 0.0);
        self.timing_offset = 0.0;
    }
}

/// Frequency-Translating FIR Filter.
///
/// Combines frequency translation (mixing), FIR filtering, and decimation
/// in a single efficient block. This is the most commonly used front-end
/// block in SDR receivers.
///
/// ## Operation
///
/// ```text
/// input → NCO mix (shift to baseband) → FIR filter → decimate → output
/// ```
///
/// All three operations happen in one pass for efficiency.
#[derive(Debug, Clone)]
pub struct FreqXlatingFirFilter {
    /// FIR filter taps
    taps: Vec<f64>,
    /// Decimation factor
    decimation: usize,
    /// Center frequency offset in Hz
    center_freq: f64,
    /// Sample rate in Hz
    sample_rate: f64,
    /// NCO phase accumulator
    nco_phase: f64,
    /// NCO phase increment per sample
    nco_inc: f64,
    /// FIR delay line
    delay_line: Vec<Complex64>,
    /// Current position in delay line
    delay_idx: usize,
    /// Decimation counter
    decim_count: usize,
}

impl FreqXlatingFirFilter {
    /// Create a new frequency-translating FIR filter.
    ///
    /// # Arguments
    /// * `taps` - FIR filter coefficients (lowpass for channel selection)
    /// * `center_freq` - Frequency to shift to baseband (Hz)
    /// * `sample_rate` - Input sample rate (Hz)
    /// * `decimation` - Decimation factor (output rate = input rate / decimation)
    pub fn new(taps: Vec<f64>, center_freq: f64, sample_rate: f64, decimation: usize) -> Self {
        let nco_inc = -2.0 * std::f64::consts::PI * center_freq / sample_rate;
        let len = taps.len();
        Self {
            taps,
            decimation: decimation.max(1),
            center_freq,
            sample_rate,
            nco_phase: 0.0,
            nco_inc,
            delay_line: vec![Complex64::new(0.0, 0.0); len],
            delay_idx: 0,
            decim_count: 0,
        }
    }

    /// Create with automatic lowpass filter design.
    ///
    /// # Arguments
    /// * `center_freq` - Frequency to shift to baseband (Hz)
    /// * `sample_rate` - Input sample rate (Hz)
    /// * `channel_bw` - Channel bandwidth (Hz, used for filter cutoff)
    /// * `decimation` - Decimation factor
    /// * `num_taps` - Number of FIR filter taps
    pub fn with_auto_filter(
        center_freq: f64,
        sample_rate: f64,
        channel_bw: f64,
        decimation: usize,
        num_taps: usize,
    ) -> Self {
        // Design lowpass filter with cutoff at half channel bandwidth
        let cutoff = channel_bw / 2.0;
        let normalized_cutoff = cutoff / sample_rate;

        // Windowed sinc lowpass design (Blackman window)
        let m = num_taps - 1;
        let taps: Vec<f64> = (0..num_taps)
            .map(|i| {
                let n = i as f64 - m as f64 / 2.0;
                let sinc = if n.abs() < 1e-10 {
                    2.0 * normalized_cutoff
                } else {
                    (2.0 * std::f64::consts::PI * normalized_cutoff * n).sin()
                        / (std::f64::consts::PI * n)
                };
                // Blackman window
                let window = 0.42 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / m as f64).cos()
                    + 0.08 * (4.0 * std::f64::consts::PI * i as f64 / m as f64).cos();
                sinc * window
            })
            .collect();

        Self::new(taps, center_freq, sample_rate, decimation)
    }

    /// Get the current center frequency.
    pub fn center_freq(&self) -> f64 {
        self.center_freq
    }

    /// Set a new center frequency (retune).
    pub fn set_center_freq(&mut self, freq: f64) {
        self.center_freq = freq;
        self.nco_inc = -2.0 * std::f64::consts::PI * freq / self.sample_rate;
    }

    /// Get decimation factor.
    pub fn decimation(&self) -> usize {
        self.decimation
    }

    /// Process a block of input samples.
    ///
    /// Returns decimated, frequency-translated, filtered output.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len() / self.decimation + 1);

        for &sample in input {
            // Mix with NCO
            let nco = Complex64::new(self.nco_phase.cos(), self.nco_phase.sin());
            let mixed = sample * nco;
            self.nco_phase += self.nco_inc;

            // Wrap phase
            if self.nco_phase > std::f64::consts::PI {
                self.nco_phase -= 2.0 * std::f64::consts::PI;
            }
            if self.nco_phase < -std::f64::consts::PI {
                self.nco_phase += 2.0 * std::f64::consts::PI;
            }

            // Insert into FIR delay line
            self.delay_line[self.delay_idx] = mixed;

            // Decimation check
            self.decim_count += 1;
            if self.decim_count >= self.decimation {
                self.decim_count = 0;

                // Compute FIR output
                let mut sum = Complex64::new(0.0, 0.0);
                let len = self.taps.len();
                for j in 0..len {
                    let idx = (self.delay_idx + len - j) % len;
                    sum += self.delay_line[idx] * self.taps[j];
                }
                output.push(sum);
            }

            self.delay_idx = (self.delay_idx + 1) % self.taps.len();
        }

        output
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        self.nco_phase = 0.0;
        self.decim_count = 0;
        for s in self.delay_line.iter_mut() {
            *s = Complex64::new(0.0, 0.0);
        }
        self.delay_idx = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_mm_basic_recovery() {
        let config = MuellerMullerConfig {
            sps: 4.0,
            loop_bw: 0.02,
            damping: 0.707,
            max_deviation: 1.5,
        };
        let mut mm = MuellerMuller::new(config);

        // Generate oversampled BPSK signal (4 samples/symbol)
        let symbols: Vec<f64> = vec![1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0];
        let sps = 4;
        let mut oversampled = Vec::new();
        for &sym in &symbols {
            for _ in 0..sps {
                oversampled.push(Complex64::new(sym, 0.0));
            }
        }

        let (recovered, _errors) = mm.process_block(&oversampled);

        // Should recover approximately the right number of symbols
        assert!(
            recovered.len() >= symbols.len() - 2,
            "Should recover most symbols: got {}, expected ~{}",
            recovered.len(),
            symbols.len()
        );
    }

    #[test]
    fn test_mm_reset() {
        let config = MuellerMullerConfig::default();
        let mut mm = MuellerMuller::new(config);

        let input = vec![Complex64::new(1.0, 0.0); 100];
        let _ = mm.process_block(&input);

        mm.reset();
        assert_eq!(mm.mu(), 0.0);
        assert_eq!(mm.timing_offset(), 0.0);
    }

    #[test]
    fn test_freq_xlating_basic() {
        let sample_rate = 100000.0; // 100 kHz
        let center_freq = 10000.0; // 10 kHz signal
        let decimation = 4;

        // Simple lowpass taps (moving average)
        let taps = vec![0.25; 4];

        let mut filter = FreqXlatingFirFilter::new(taps, center_freq, sample_rate, decimation);

        // Generate a tone at the center frequency
        let input: Vec<Complex64> = (0..1000)
            .map(|i| {
                let phase = 2.0 * PI * center_freq * i as f64 / sample_rate;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        let output = filter.process_block(&input);

        // Output should be decimated (1/4 the samples)
        assert_eq!(output.len(), 250);

        // After mixing, the tone should be at DC (baseband)
        // The output should have significant energy
        let power: f64 = output.iter().map(|s| s.norm_sqr()).sum::<f64>() / output.len() as f64;
        assert!(power > 0.01, "Output should have energy: power={:.4}", power);
    }

    #[test]
    fn test_freq_xlating_auto_filter() {
        let filter = FreqXlatingFirFilter::with_auto_filter(
            5000.0,   // 5 kHz center
            100000.0, // 100 kHz sample rate
            2000.0,   // 2 kHz channel BW
            10,       // Decimate by 10
            63,       // 63 taps
        );

        assert_eq!(filter.decimation(), 10);
        assert_eq!(filter.center_freq(), 5000.0);
    }

    #[test]
    fn test_freq_xlating_retune() {
        let taps = vec![1.0; 4];
        let mut filter = FreqXlatingFirFilter::new(taps, 1000.0, 100000.0, 1);

        assert_eq!(filter.center_freq(), 1000.0);

        filter.set_center_freq(2000.0);
        assert_eq!(filter.center_freq(), 2000.0);
    }

    #[test]
    fn test_freq_xlating_reset() {
        let taps = vec![0.25; 4];
        let mut filter = FreqXlatingFirFilter::new(taps, 1000.0, 100000.0, 1);

        let input = vec![Complex64::new(1.0, 0.0); 100];
        let _ = filter.process_block(&input);

        filter.reset();
        // After reset, filter should behave identically to a fresh one
        let output1 = filter.process_block(&input);

        let mut filter2 = FreqXlatingFirFilter::new(vec![0.25; 4], 1000.0, 100000.0, 1);
        let output2 = filter2.process_block(&input);

        assert_eq!(output1.len(), output2.len());
    }
}
