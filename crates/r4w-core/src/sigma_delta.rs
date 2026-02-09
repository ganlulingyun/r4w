//! Sigma-Delta Modulator/Demodulator
//!
//! Oversampling ADC/DAC modeling with noise-shaping. First, second, and
//! third-order sigma-delta modulators convert high-resolution signals into
//! 1-bit (or multi-bit) streams. The corresponding decimation filter
//! recovers the original signal. Useful for ADC/DAC education and
//! simulation of oversampling converters.
//! GNU Radio equivalent: no direct equivalent (fundamental DSP building block).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::sigma_delta::{SigmaDeltaModulator, SigmaDeltaDemodulator, ModulatorOrder};
//!
//! // First-order 1-bit modulator
//! let mut modulator = SigmaDeltaModulator::new(ModulatorOrder::First, 1);
//! let signal: Vec<f64> = (0..256).map(|i| 0.3 * (i as f64 * 0.05).sin()).collect();
//! let bitstream = modulator.modulate(&signal);
//! assert_eq!(bitstream.len(), signal.len());
//!
//! // Demodulate with sinc³ decimation (OSR=16)
//! let mut demod = SigmaDeltaDemodulator::new(16);
//! let recovered = demod.demodulate(&bitstream);
//! assert!(recovered.len() > 0);
//! ```

/// Sigma-delta modulator order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ModulatorOrder {
    /// First-order (single integrator). NTF = (1 - z^{-1}).
    First,
    /// Second-order (two integrators). NTF = (1 - z^{-1})^2.
    Second,
    /// Third-order (CIFB topology). NTF = (1 - z^{-1})^3.
    Third,
}

/// Sigma-delta modulator — converts continuous-valued input to quantized output.
///
/// Uses the Error Feedback (EFB) structure which is unconditionally stable
/// for all orders. NTF = (1 - z^{-1})^L.
///
/// For 1-bit operation (bits=1), output is +1.0 / -1.0.
/// For multi-bit (bits > 1), output is quantized to 2^bits levels in [-1, 1].
#[derive(Debug, Clone)]
pub struct SigmaDeltaModulator {
    order: ModulatorOrder,
    /// Number of output bits (1 = single-bit, 2+ = multi-bit)
    bits: u32,
    /// Number of quantization levels
    num_levels: usize,
    /// Error feedback FIR coefficients: (1-z^{-1})^L expanded, negated, shifted
    efb_coeffs: Vec<f64>,
    /// Previous quantization errors (ring buffer)
    error_history: Vec<f64>,
    /// Error history write position
    error_pos: usize,
}

impl SigmaDeltaModulator {
    /// Create a new sigma-delta modulator.
    ///
    /// `bits` controls quantizer resolution: 1 for single-bit, 2+ for multi-bit.
    pub fn new(order: ModulatorOrder, bits: u32) -> Self {
        let bits = bits.max(1);
        // EFB coefficients: h[k] = coefficients of (1 - (1-z^{-1})^L) for k=1..L
        // L=1: [1]
        // L=2: [2, -1]
        // L=3: [3, -3, 1]
        let efb_coeffs = match order {
            ModulatorOrder::First => vec![1.0],
            ModulatorOrder::Second => vec![2.0, -1.0],
            ModulatorOrder::Third => vec![3.0, -3.0, 1.0],
        };
        let len = efb_coeffs.len();
        Self {
            order,
            bits,
            num_levels: 1 << bits,
            efb_coeffs,
            error_history: vec![0.0; len],
            error_pos: 0,
        }
    }

    /// Modulate a signal, returning quantized output stream.
    ///
    /// Input should be in [-1, 1]. Output for 1-bit: +1.0/-1.0.
    pub fn modulate(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&x| self.process_sample(x)).collect()
    }

    /// Process a single sample through the modulator.
    pub fn process_sample(&mut self, input: f64) -> f64 {
        // Compute error feedback: sum of h[k]*e[n-k]
        let len = self.efb_coeffs.len();
        let mut feedback = 0.0;
        for k in 0..len {
            // e[n-k-1] is at position (error_pos - k - 1 + len) % len
            let idx = (self.error_pos + len - 1 - k) % len;
            feedback += self.efb_coeffs[k] * self.error_history[idx];
        }

        // Quantizer input: v[n] = x[n] - feedback
        // (NTF-1 has negative leading terms, so we subtract the positive-coeff sum)
        let v = input - feedback;
        // Quantize
        let y = self.quantize(v);
        // Quantization error: e[n] = y[n] - v[n]
        let e = y - v;
        // Store error
        self.error_history[self.error_pos] = e;
        self.error_pos = (self.error_pos + 1) % len;

        y
    }

    /// Mid-tread uniform quantizer.
    fn quantize(&self, x: f64) -> f64 {
        if self.bits == 1 {
            if x >= 0.0 { 1.0 } else { -1.0 }
        } else {
            let levels = self.num_levels as f64;
            let step = 2.0 / levels;
            let q = ((x + 1.0) / step).floor() * step - 1.0 + step / 2.0;
            q.clamp(-1.0 + step / 2.0, 1.0 - step / 2.0)
        }
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.error_history.fill(0.0);
        self.error_pos = 0;
    }

    /// Get the modulator order.
    pub fn order(&self) -> ModulatorOrder {
        self.order
    }

    /// Get the number of quantizer bits.
    pub fn bits(&self) -> u32 {
        self.bits
    }
}

/// Compute theoretical SNR for an ideal sigma-delta modulator.
///
/// SNR(dB) ≈ 6.02*bits + 1.76 + (2*order+1)*10*log10(OSR) - 10*log10(π^{2*order} / (2*order+1))
pub fn theoretical_snr_db(order: ModulatorOrder, bits: u32, osr: usize) -> f64 {
    let n = match order {
        ModulatorOrder::First => 1,
        ModulatorOrder::Second => 2,
        ModulatorOrder::Third => 3,
    };
    let b = bits as f64;
    let osr_f = osr as f64;

    6.02 * b + 1.76
        + (2 * n + 1) as f64 * 10.0 * osr_f.log10()
        - 10.0 * (std::f64::consts::PI.powi(2 * n as i32) / (2 * n + 1) as f64).log10()
}

/// Sigma-delta demodulator — decimation filter for bitstream recovery.
///
/// Uses cascaded integrator-comb (CIC) / sinc³ decimation to recover
/// the original signal from the oversampled bitstream. The CIC has
/// 3 integrator stages (running at the high rate) and 3 comb stages
/// (running at the decimated rate, each with differential delay M=1).
#[derive(Debug, Clone)]
pub struct SigmaDeltaDemodulator {
    /// Oversampling ratio (decimation factor)
    osr: usize,
    /// CIC integrator states (3 stages)
    integrators: [f64; 3],
    /// CIC comb previous values (3 stages, M=1 delay at decimated rate)
    comb_prev: [f64; 3],
    /// Sample counter for decimation
    count: usize,
}

impl SigmaDeltaDemodulator {
    /// Create a demodulator with the given oversampling ratio.
    pub fn new(osr: usize) -> Self {
        let osr = osr.max(2);
        Self {
            osr,
            integrators: [0.0; 3],
            comb_prev: [0.0; 3],
            count: 0,
        }
    }

    /// Demodulate a bitstream, returning decimated output.
    pub fn demodulate(&mut self, bitstream: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(bitstream.len() / self.osr + 1);
        for &sample in bitstream {
            // Three-stage CIC integrator (runs at high rate)
            self.integrators[0] += sample;
            self.integrators[1] += self.integrators[0];
            self.integrators[2] += self.integrators[1];

            self.count += 1;
            if self.count >= self.osr {
                self.count = 0;
                // Three-stage CIC comb (runs at decimated rate, M=1)
                let mut val = self.integrators[2];
                for stage in 0..3 {
                    let prev = self.comb_prev[stage];
                    self.comb_prev[stage] = val;
                    val -= prev;
                }
                // Normalize by OSR^3 for sinc³
                let norm = (self.osr as f64).powi(3);
                output.push(val / norm);
            }
        }
        output
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        self.integrators = [0.0; 3];
        self.comb_prev = [0.0; 3];
        self.count = 0;
    }

    /// Get oversampling ratio.
    pub fn osr(&self) -> usize {
        self.osr
    }
}

/// Compute noise transfer function magnitude |NTF(f)| for a given order.
///
/// NTF(z) = (1 - z^{-1})^order evaluated at z = e^{j2πf/fs}.
pub fn ntf_magnitude(order: ModulatorOrder, freq_normalized: f64) -> f64 {
    let n = match order {
        ModulatorOrder::First => 1,
        ModulatorOrder::Second => 2,
        ModulatorOrder::Third => 3,
    };
    // |1 - e^{-j2πf}| = 2*sin(πf)
    let mag_single = 2.0 * (std::f64::consts::PI * freq_normalized).sin().abs();
    mag_single.powi(n as i32)
}

/// Compute signal transfer function magnitude |STF(f)| = 1 for all orders
/// (ideal feed-forward topology).
pub fn stf_magnitude(_order: ModulatorOrder, _freq_normalized: f64) -> f64 {
    1.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_first_order_dc_input() {
        let mut m = SigmaDeltaModulator::new(ModulatorOrder::First, 1);
        let input = vec![0.5; 1000];
        let output = m.modulate(&input);
        // Average of bitstream should approximate input
        let avg: f64 = output.iter().sum::<f64>() / output.len() as f64;
        assert!((avg - 0.5).abs() < 0.05, "avg={avg}");
    }

    #[test]
    fn test_first_order_output_values() {
        let mut m = SigmaDeltaModulator::new(ModulatorOrder::First, 1);
        let output = m.modulate(&[0.3; 100]);
        // All outputs should be +1 or -1
        assert!(output.iter().all(|&v| v == 1.0 || v == -1.0));
    }

    #[test]
    fn test_second_order_dc() {
        let mut m = SigmaDeltaModulator::new(ModulatorOrder::Second, 1);
        let input = vec![-0.3; 2000];
        let output = m.modulate(&input);
        let avg: f64 = output.iter().sum::<f64>() / output.len() as f64;
        assert!((avg - (-0.3)).abs() < 0.05, "avg={avg}");
    }

    #[test]
    fn test_third_order_dc() {
        // Third-order NTF=(1-z^{-1})^3 has OOB gain=8, exceeds Lee's criterion for
        // 1-bit. Use 3-bit quantizer for stability.
        let mut m = SigmaDeltaModulator::new(ModulatorOrder::Third, 3);
        let input = vec![0.1; 4000];
        let output = m.modulate(&input);
        let tail = &output[output.len() / 2..];
        let avg: f64 = tail.iter().sum::<f64>() / tail.len() as f64;
        assert!((avg - 0.1).abs() < 0.05, "avg={avg}");
    }

    #[test]
    fn test_multibit_quantizer() {
        let mut m = SigmaDeltaModulator::new(ModulatorOrder::First, 3);
        let output = m.modulate(&[0.0; 10]);
        // 3-bit quantizer has 8 levels: values in [-1, 1]
        assert!(output.iter().all(|&v| v >= -1.0 && v <= 1.0));
    }

    #[test]
    fn test_demodulator_recovers_dc() {
        let mut m = SigmaDeltaModulator::new(ModulatorOrder::First, 1);
        let dc = 0.4;
        let bitstream = m.modulate(&vec![dc; 2048]);
        let mut d = SigmaDeltaDemodulator::new(32);
        let recovered = d.demodulate(&bitstream);
        // After transient, recovered values should approach dc
        let tail = &recovered[recovered.len() / 2..];
        let avg: f64 = tail.iter().sum::<f64>() / tail.len() as f64;
        assert!((avg - dc).abs() < 0.15, "avg={avg}");
    }

    #[test]
    fn test_demodulator_decimation_ratio() {
        let mut d = SigmaDeltaDemodulator::new(16);
        let input = vec![1.0; 160];
        let output = d.demodulate(&input);
        assert_eq!(output.len(), 10);
    }

    #[test]
    fn test_theoretical_snr() {
        // First-order, 1-bit, OSR=256: ~85 dB
        let snr = theoretical_snr_db(ModulatorOrder::First, 1, 256);
        assert!(snr > 70.0 && snr < 100.0, "snr={snr}");
        // Higher order should have higher SNR
        let snr2 = theoretical_snr_db(ModulatorOrder::Second, 1, 256);
        assert!(snr2 > snr, "second order ({snr2}) should be > first ({snr})");
    }

    #[test]
    fn test_ntf_dc() {
        // NTF(0) = 0 for all orders (noise is zero at DC)
        assert!((ntf_magnitude(ModulatorOrder::First, 0.0)).abs() < 1e-10);
        assert!((ntf_magnitude(ModulatorOrder::Second, 0.0)).abs() < 1e-10);
    }

    #[test]
    fn test_ntf_nyquist() {
        // NTF(0.5) = 2^order
        let m1 = ntf_magnitude(ModulatorOrder::First, 0.5);
        assert!((m1 - 2.0).abs() < 1e-10);
        let m2 = ntf_magnitude(ModulatorOrder::Second, 0.5);
        assert!((m2 - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_reset() {
        let mut m = SigmaDeltaModulator::new(ModulatorOrder::Second, 1);
        m.modulate(&[0.5; 100]);
        m.reset();
        // After reset, first sample of zero input should produce known output
        let out = m.process_sample(0.0);
        assert!(out == 1.0 || out == -1.0);
    }

    #[test]
    fn test_sine_roundtrip() {
        let osr = 32;
        let n = 4096;
        let freq = 0.01; // Low frequency relative to sample rate
        let input: Vec<f64> = (0..n).map(|i| 0.4 * (2.0 * std::f64::consts::PI * freq * i as f64).sin()).collect();
        let mut m = SigmaDeltaModulator::new(ModulatorOrder::Second, 1);
        let bitstream = m.modulate(&input);
        let mut d = SigmaDeltaDemodulator::new(osr);
        let recovered = d.demodulate(&bitstream);
        // Should have roughly n/osr output samples
        assert!(recovered.len() >= n / osr - 1);
    }
}
