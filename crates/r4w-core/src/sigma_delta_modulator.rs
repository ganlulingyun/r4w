//! # Sigma-Delta Modulation for Oversampled DAC/ADC
//!
//! Implements sigma-delta (SD) modulation and demodulation for oversampled
//! digital-to-analog conversion with noise shaping. Sigma-delta converters
//! trade amplitude resolution for sample rate, pushing quantization noise
//! out of the signal band through loop-filter feedback.
//!
//! ## Applications
//!
//! - **Audio DACs**: High-fidelity audio converters (1-bit DSD, multi-bit delta-sigma)
//!   use SD modulation to achieve 24-bit dynamic range from simple 1-bit quantizers.
//! - **Audio ADCs**: Microphone preamps and recording interfaces oversample at
//!   64x-256x and noise-shape to achieve high resolution with simple analog front-ends.
//! - **IoT Sensors**: Low-power temperature, pressure, and strain-gauge ADCs use
//!   SD architectures (e.g., ADS1220, MAX11200) for high resolution at low cost.
//! - **Instrumentation**: Precision measurement systems leverage high-order SD
//!   modulators to achieve 20+ effective bits with minimal analog complexity.
//!
//! ## Architecture
//!
//! The modulator uses the Cascade of Integrators with Feedback (CIFB) topology:
//!
//! - **First order**: Single integrator with error feedback.
//!   `y[n] = sign(x[n] + s1[n-1])`, where `s1` accumulates `x - y`.
//! - **Second order**: Two cascaded integrators with distributed feedback.
//!   Noise transfer function: `(1 - z^-1)^2`.
//! - **Third order**: Three cascaded integrators (CIFB).
//!   Noise transfer function: `(1 - z^-1)^3`.
//!
//! The demodulator uses CIC (Cascaded Integrator-Comb) decimation to recover
//! the baseband signal from the oversampled 1-bit stream.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::sigma_delta_modulator::{
//!     SigmaDeltaOrder, SigmaDeltaModulator, SigmaDeltaDemodulator,
//!     snr_theoretical, enob,
//! };
//!
//! let mut modulator = SigmaDeltaModulator::new(SigmaDeltaOrder::Second, 64);
//! let signal: Vec<f64> = (0..640)
//!     .map(|i| 0.3 * (2.0 * std::f64::consts::PI * 0.01 * i as f64).sin())
//!     .collect();
//!
//! let bitstream = modulator.modulate(&signal);
//! assert!(bitstream.iter().all(|&b| b == 1 || b == -1));
//!
//! let demod = SigmaDeltaDemodulator::new(2, 64);
//! let recovered = demod.demodulate(&bitstream);
//!
//! // Theoretical performance
//! let snr = snr_theoretical(2, 64);
//! let bits = enob(2, 64);
//! assert!(snr > 60.0);
//! assert!(bits > 10.0);
//! ```

use std::f64::consts::PI;

/// Order of the sigma-delta modulator loop filter.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SigmaDeltaOrder {
    /// First-order: single integrator, NTF = (1 - z^{-1}).
    First,
    /// Second-order: two cascaded integrators, NTF = (1 - z^{-1})^2.
    Second,
    /// Third-order: three cascaded integrators (CIFB), NTF = (1 - z^{-1})^3.
    Third,
}

impl SigmaDeltaOrder {
    /// Return the numeric order (1, 2, or 3).
    pub fn as_usize(self) -> usize {
        match self {
            SigmaDeltaOrder::First => 1,
            SigmaDeltaOrder::Second => 2,
            SigmaDeltaOrder::Third => 3,
        }
    }
}

/// Sigma-delta modulator converting continuous-valued samples into a 1-bit
/// (+1 / -1) stream via noise-shaping feedback.
///
/// The modulator maintains internal integrator states that accumulate the
/// difference between input and quantized output, shaping quantization noise
/// to higher frequencies where it can be removed by decimation filtering.
#[derive(Debug, Clone)]
pub struct SigmaDeltaModulator {
    order: SigmaDeltaOrder,
    osr: usize,
    /// Integrator state vector; length equals the order.
    integrators: Vec<f64>,
}

impl SigmaDeltaModulator {
    /// Create a new sigma-delta modulator.
    ///
    /// # Arguments
    ///
    /// * `order` - Loop filter order (First, Second, or Third).
    /// * `osr` - Oversampling ratio. The input is assumed to already be at the
    ///   oversampled rate; this value is stored for metadata queries.
    pub fn new(order: SigmaDeltaOrder, osr: usize) -> Self {
        let n = order.as_usize();
        Self {
            order,
            osr: osr.max(1),
            integrators: vec![0.0; n],
        }
    }

    /// Modulate an input signal into a 1-bit stream.
    ///
    /// Input samples should be in the range `[-1.0, 1.0]`. Values outside
    /// this range will still be processed but may cause integrator saturation
    /// in higher-order modulators.
    ///
    /// Returns a vector of `+1` and `-1` values (one output per input sample).
    pub fn modulate(&mut self, input: &[f64]) -> Vec<i8> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            let y = self.process_one(x);
            output.push(y);
        }
        output
    }

    /// Process a single input sample through the modulator, returning +1 or -1.
    fn process_one(&mut self, x: f64) -> i8 {
        match self.order {
            SigmaDeltaOrder::First => {
                // First order: y[n] = sign(x[n] + s1[n-1])
                // s1[n] = s1[n-1] + x[n] - y[n]
                let v = x + self.integrators[0];
                let y: i8 = if v >= 0.0 { 1 } else { -1 };
                self.integrators[0] = v - y as f64;
                y
            }
            SigmaDeltaOrder::Second => {
                // Second order: two cascaded integrators with error feedback
                // s1[n] = s1[n-1] + x[n]
                // s2[n] = s2[n-1] + s1[n]
                // y[n] = sign(s2[n])
                // Error feedback: subtract y from both integrators
                self.integrators[0] += x;
                self.integrators[1] += self.integrators[0];
                let y: i8 = if self.integrators[1] >= 0.0 { 1 } else { -1 };
                let yf = y as f64;
                self.integrators[0] -= yf;
                self.integrators[1] -= yf;
                y
            }
            SigmaDeltaOrder::Third => {
                // Third order CIFB with integrator clamping for 1-bit stability.
                // Pure (1-z^{-1})^3 NTF has max gain 8, exceeding Lee's criterion
                // for single-bit (max NTF gain < 1.5). We use distributed feedback
                // with integrator clamping to prevent state divergence.
                //
                // Feed-forward coefficients: a1=1, a2=1, a3=1
                // Feedback from quantizer: b1=1, b2=2, b3=1
                // This gives a flatter NTF with lower out-of-band gain.
                self.integrators[0] += x;
                let s0 = self.integrators[0];
                self.integrators[1] += s0;
                let s1 = self.integrators[1];
                self.integrators[2] += s1;

                let y: i8 = if self.integrators[2] >= 0.0 { 1 } else { -1 };
                let yf = y as f64;

                // Distributed feedback
                self.integrators[0] -= yf;
                self.integrators[1] -= 2.0 * yf;
                self.integrators[2] -= yf;

                // Clamp integrators to prevent divergence (standard stabilization)
                let clamp = 2.0;
                for s in self.integrators.iter_mut() {
                    *s = s.clamp(-clamp, clamp);
                }
                y
            }
        }
    }

    /// Reset all integrator states to zero.
    pub fn reset(&mut self) {
        self.integrators.fill(0.0);
    }

    /// Return the modulator order as a numeric value (1, 2, or 3).
    pub fn order(&self) -> usize {
        self.order.as_usize()
    }

    /// Return the oversampling ratio.
    pub fn osr(&self) -> usize {
        self.osr
    }
}

/// Sigma-delta demodulator using CIC decimation to recover the baseband
/// signal from an oversampled 1-bit stream.
///
/// The CIC filter order matches the modulator order for optimal noise
/// cancellation, and the decimation factor equals the oversampling ratio.
#[derive(Debug, Clone)]
pub struct SigmaDeltaDemodulator {
    /// CIC filter order (number of integrator/comb stages).
    cic_order: usize,
    /// Decimation factor (oversampling ratio).
    osr: usize,
}

impl SigmaDeltaDemodulator {
    /// Create a demodulator matching a modulator's parameters.
    ///
    /// # Arguments
    ///
    /// * `order` - CIC filter order (should match the modulator order for best results).
    /// * `osr` - Oversampling ratio / decimation factor.
    pub fn new(order: usize, osr: usize) -> Self {
        Self {
            cic_order: order.max(1),
            osr: osr.max(2),
        }
    }

    /// Demodulate a 1-bit stream (+1/-1) back to analog samples.
    ///
    /// Uses a CIC (sinc) decimation filter. The output length is
    /// approximately `bitstream.len() / osr`.
    pub fn demodulate(&self, bitstream: &[i8]) -> Vec<f64> {
        let n = bitstream.len();
        if n == 0 {
            return Vec::new();
        }

        // Convert i8 stream to f64
        let samples: Vec<f64> = bitstream.iter().map(|&b| b as f64).collect();

        // Apply CIC decimation: cascade of integrate-and-dump stages
        // Each stage: running sum over osr samples, then downsample
        let mut current = samples;

        for _stage in 0..self.cic_order {
            // Integrate (cumulative sum)
            let mut integrated = Vec::with_capacity(current.len());
            let mut acc = 0.0;
            for &s in &current {
                acc += s;
                integrated.push(acc);
            }

            // Comb + decimate: y[k] = integrated[k*R] - integrated[(k-1)*R]
            let out_len = integrated.len() / self.osr;
            let mut decimated = Vec::with_capacity(out_len);
            for k in 0..out_len {
                let idx = (k + 1) * self.osr - 1;
                let prev_idx = if k == 0 {
                    None
                } else {
                    Some(k * self.osr - 1)
                };
                let val = match prev_idx {
                    Some(pi) => integrated[idx] - integrated[pi],
                    None => integrated[idx],
                };
                decimated.push(val);
            }

            current = decimated;
        }

        // Normalize by OSR^order to get unity gain at DC
        let norm = (self.osr as f64).powi(self.cic_order as i32);
        current.iter().map(|&v| v / norm).collect()
    }
}

/// Compute the theoretical SNR in dB for an ideal sigma-delta modulator.
///
/// The formula is:
///
/// ```text
/// SNR = 6.02*B + 1.76 + (2N+1)*10*log10(OSR) - 10*log10(pi^(2N) / (2N+1))
/// ```
///
/// where `N` is the modulator order, `B` is the quantizer bits (1 for single-bit),
/// and `OSR` is the oversampling ratio.
///
/// # Arguments
///
/// * `order` - Modulator order (1, 2, or 3).
/// * `osr` - Oversampling ratio.
pub fn snr_theoretical(order: usize, osr: usize) -> f64 {
    let n = order as f64;
    let b = 1.0_f64; // 1-bit quantizer
    let osr_f = osr as f64;

    6.02 * b + 1.76
        + (2.0 * n + 1.0) * 10.0 * osr_f.log10()
        - 10.0 * (PI.powf(2.0 * n) / (2.0 * n + 1.0)).log10()
}

/// Compute the effective number of bits (ENOB) for a sigma-delta modulator.
///
/// ```text
/// ENOB = (SNR_dB - 1.76) / 6.02
/// ```
///
/// # Arguments
///
/// * `order` - Modulator order (1, 2, or 3).
/// * `osr` - Oversampling ratio.
pub fn enob(order: usize, osr: usize) -> f64 {
    (snr_theoretical(order, osr) - 1.76) / 6.02
}

#[cfg(test)]
mod tests {
    use super::*;

    // 1. First order modulate
    #[test]
    fn test_first_order_modulate() {
        let mut m = SigmaDeltaModulator::new(SigmaDeltaOrder::First, 64);
        let input = vec![0.0; 128];
        let output = m.modulate(&input);
        assert_eq!(output.len(), 128);
        // For zero input, first-order should alternate around zero
        let sum: f64 = output.iter().map(|&b| b as f64).sum::<f64>();
        let avg = sum / output.len() as f64;
        assert!(avg.abs() < 0.1, "DC=0 input should average near zero, got {avg}");
    }

    // 2. Second order modulate
    #[test]
    fn test_second_order_modulate() {
        let mut m = SigmaDeltaModulator::new(SigmaDeltaOrder::Second, 64);
        let input = vec![0.25; 512];
        let output = m.modulate(&input);
        assert_eq!(output.len(), 512);
        let avg: f64 = output.iter().map(|&b| b as f64).sum::<f64>() / output.len() as f64;
        assert!(
            (avg - 0.25).abs() < 0.1,
            "Second order DC=0.25: avg={avg}, expected ~0.25"
        );
    }

    // 3. Third order modulate
    #[test]
    fn test_third_order_modulate() {
        let mut m = SigmaDeltaModulator::new(SigmaDeltaOrder::Third, 64);
        // Use smaller amplitude for third-order stability and longer signal
        let input = vec![-0.2; 4096];
        let output = m.modulate(&input);
        assert_eq!(output.len(), 4096);
        // Use last quarter to avoid transient
        let tail = &output[3072..];
        let avg: f64 = tail.iter().map(|&b| b as f64).sum::<f64>() / tail.len() as f64;
        assert!(
            (avg - (-0.2)).abs() < 0.25,
            "Third order DC=-0.2: avg={avg}, expected ~-0.2"
        );
    }

    // 4. Output is +/-1 only
    #[test]
    fn test_output_is_plus_minus_one() {
        for order in [SigmaDeltaOrder::First, SigmaDeltaOrder::Second, SigmaDeltaOrder::Third] {
            let mut m = SigmaDeltaModulator::new(order, 32);
            let input: Vec<f64> = (0..256)
                .map(|i| 0.5 * (2.0 * PI * 0.02 * i as f64).sin())
                .collect();
            let output = m.modulate(&input);
            for &b in &output {
                assert!(b == 1 || b == -1, "Output must be +1 or -1, got {b}");
            }
        }
    }

    // 5. Roundtrip quality: demodulate recovers signal approximately
    #[test]
    fn test_roundtrip_recovery() {
        let osr = 64;
        let n_samples = osr * 100; // 100 output samples
        let dc_level = 0.4;
        let input = vec![dc_level; n_samples];

        let mut m = SigmaDeltaModulator::new(SigmaDeltaOrder::Second, osr);
        let bitstream = m.modulate(&input);

        let d = SigmaDeltaDemodulator::new(2, osr);
        let recovered = d.demodulate(&bitstream);

        assert!(!recovered.is_empty(), "Demodulator should produce output");

        // Skip initial transient, check second half
        let start = recovered.len() / 2;
        let tail = &recovered[start..];
        let avg: f64 = tail.iter().sum::<f64>() / tail.len() as f64;
        assert!(
            (avg - dc_level).abs() < 0.15,
            "Roundtrip DC recovery: avg={avg}, expected ~{dc_level}"
        );
    }

    // 6. SNR improves with order
    #[test]
    fn test_snr_improves_with_order() {
        let osr = 64;
        let snr1 = snr_theoretical(1, osr);
        let snr2 = snr_theoretical(2, osr);
        let snr3 = snr_theoretical(3, osr);
        assert!(
            snr2 > snr1,
            "Second-order SNR ({snr2:.1}) should exceed first-order ({snr1:.1})"
        );
        assert!(
            snr3 > snr2,
            "Third-order SNR ({snr3:.1}) should exceed second-order ({snr2:.1})"
        );
    }

    // 7. SNR improves with OSR
    #[test]
    fn test_snr_improves_with_osr() {
        let snr_32 = snr_theoretical(2, 32);
        let snr_64 = snr_theoretical(2, 64);
        let snr_128 = snr_theoretical(2, 128);
        assert!(
            snr_64 > snr_32,
            "OSR=64 SNR ({snr_64:.1}) should exceed OSR=32 ({snr_32:.1})"
        );
        assert!(
            snr_128 > snr_64,
            "OSR=128 SNR ({snr_128:.1}) should exceed OSR=64 ({snr_64:.1})"
        );
    }

    // 8. Reset clears state
    #[test]
    fn test_reset_clears_state() {
        let mut m = SigmaDeltaModulator::new(SigmaDeltaOrder::Second, 64);
        // Drive with a strong signal to build up integrator state
        m.modulate(&vec![0.9; 500]);

        m.reset();

        // After reset, integrators should be zero. Processing zero should
        // behave identically to a fresh modulator.
        let mut fresh = SigmaDeltaModulator::new(SigmaDeltaOrder::Second, 64);
        let input = vec![0.0; 20];
        let out_reset = m.modulate(&input);
        let out_fresh = fresh.modulate(&input);
        assert_eq!(out_reset, out_fresh, "Reset modulator should match fresh modulator");
    }

    // 9. Theoretical SNR computation
    #[test]
    fn test_theoretical_snr_computation() {
        // First order, OSR=256: formula gives ~74.9 dB
        let snr = snr_theoretical(1, 256);
        assert!(
            snr > 70.0 && snr < 85.0,
            "First-order OSR=256 SNR should be ~75 dB, got {snr:.1}"
        );

        // Second order, OSR=64: expected ~67 dB
        let snr2 = snr_theoretical(2, 64);
        assert!(
            snr2 > 55.0 && snr2 < 100.0,
            "Second-order OSR=64 SNR should be ~67 dB, got {snr2:.1}"
        );
    }

    // 10. ENOB computation
    #[test]
    fn test_enob_computation() {
        let bits = enob(2, 64);
        assert!(
            bits > 10.0 && bits < 20.0,
            "Second-order OSR=64 ENOB should be 10-20 bits, got {bits:.1}"
        );

        // ENOB should increase with order and OSR
        let bits_low = enob(1, 32);
        let bits_high = enob(3, 128);
        assert!(
            bits_high > bits_low,
            "Higher order+OSR should give more ENOB: {bits_high:.1} > {bits_low:.1}"
        );
    }

    // Bonus: sine wave modulation
    #[test]
    fn test_sine_wave_modulation() {
        let osr = 64;
        let n_samples = osr * 200; // 12800 samples
        let freq = 1.0 / (n_samples as f64); // One full cycle over the entire signal
        let input: Vec<f64> = (0..n_samples)
            .map(|i| 0.4 * (2.0 * PI * freq * i as f64).sin())
            .collect();

        let mut m = SigmaDeltaModulator::new(SigmaDeltaOrder::Second, osr);
        let bitstream = m.modulate(&input);
        assert_eq!(bitstream.len(), n_samples);

        // Verify bit density tracks the sine wave amplitude:
        // Positive half-cycle (first half) should have more +1s than -1s
        let half = n_samples / 2;
        let pos_half_sum: f64 = bitstream[..half]
            .iter()
            .map(|&b| b as f64)
            .sum();
        let neg_half_sum: f64 = bitstream[half..]
            .iter()
            .map(|&b| b as f64)
            .sum();
        assert!(
            pos_half_sum > neg_half_sum,
            "Positive half-cycle sum ({pos_half_sum:.1}) should exceed \
             negative half-cycle sum ({neg_half_sum:.1})"
        );
    }
}
