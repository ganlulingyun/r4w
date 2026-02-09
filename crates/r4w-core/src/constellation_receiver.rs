//! Constellation Receiver — Combined Carrier Recovery + Symbol Decision
//!
//! High-level receiver block that combines AGC, carrier recovery (Costas loop),
//! and symbol demapping into a single processing chain. Suitable for
//! BPSK/QPSK/8PSK reception.
//!
//! ## Signal Flow
//!
//! ```text
//! input → AGC → Costas Loop → Soft/Hard Decision → symbols/LLRs
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::constellation_receiver::{ConstellationReceiver, ReceiverConfig};
//! use num_complex::Complex64;
//! use std::f64::consts::PI;
//!
//! let config = ReceiverConfig::qpsk(0.05);
//! let mut rx = ConstellationReceiver::new(config);
//!
//! // Process QPSK signal with carrier offset
//! let samples: Vec<Complex64> = (0..200)
//!     .map(|i| {
//!         let sym_angle = PI / 4.0 * (1 + 2 * (i % 4)) as f64;
//!         let carrier = 0.03 * i as f64;
//!         Complex64::new((sym_angle + carrier).cos(), (sym_angle + carrier).sin())
//!     })
//!     .collect();
//!
//! let result = rx.process_block(&samples);
//! assert_eq!(result.symbols.len(), samples.len());
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

use crate::agc::{Agc, AgcConfig};
use crate::costas_loop::{CostasConfig, CostasLoop};
use crate::symbol_mapping::{Modulation, SymbolMapper};

/// Receiver configuration.
#[derive(Debug, Clone)]
pub struct ReceiverConfig {
    /// Modulation type.
    pub modulation: Modulation,
    /// Costas loop bandwidth (normalized).
    pub loop_bw: f64,
    /// Costas loop damping factor.
    pub damping: f64,
    /// AGC target amplitude.
    pub agc_target: f64,
    /// AGC tracking rate.
    pub agc_rate: f64,
    /// Enable soft-decision output (LLRs).
    pub soft_output: bool,
    /// Noise variance estimate for soft decisions.
    pub noise_variance: f64,
}

impl ReceiverConfig {
    /// Create a BPSK receiver configuration.
    pub fn bpsk(loop_bw: f64) -> Self {
        Self {
            modulation: Modulation::Bpsk,
            loop_bw,
            damping: 0.707,
            agc_target: 1.0,
            agc_rate: 0.01,
            soft_output: false,
            noise_variance: 0.1,
        }
    }

    /// Create a QPSK receiver configuration.
    pub fn qpsk(loop_bw: f64) -> Self {
        Self {
            modulation: Modulation::Qpsk,
            loop_bw,
            damping: 0.707,
            agc_target: 1.0,
            agc_rate: 0.01,
            soft_output: false,
            noise_variance: 0.1,
        }
    }

    /// Create an 8PSK receiver configuration.
    pub fn psk8(loop_bw: f64) -> Self {
        Self {
            modulation: Modulation::Psk8,
            loop_bw,
            damping: 0.707,
            agc_target: 1.0,
            agc_rate: 0.01,
            soft_output: false,
            noise_variance: 0.1,
        }
    }
}

/// Output from the constellation receiver.
#[derive(Debug, Clone)]
pub struct ReceiverOutput {
    /// Corrected complex symbols (after AGC + carrier recovery).
    pub symbols: Vec<Complex64>,
    /// Hard-decision bits (if not soft output).
    pub bits: Vec<u8>,
    /// Soft-decision LLRs (if soft_output enabled).
    pub llrs: Vec<f64>,
    /// Frequency offset estimate (Hz, if sample_rate provided).
    pub freq_offset_rad: f64,
    /// Current AGC gain.
    pub agc_gain: f64,
    /// Lock metric (lower = better lock).
    pub lock_metric: f64,
}

/// Combined constellation receiver.
#[derive(Debug, Clone)]
pub struct ConstellationReceiver {
    config: ReceiverConfig,
    agc: Agc,
    costas: CostasLoop,
    mapper: SymbolMapper,
}

impl ConstellationReceiver {
    /// Create a new constellation receiver.
    pub fn new(config: ReceiverConfig) -> Self {
        let agc = Agc::new(AgcConfig {
            target_amplitude: config.agc_target,
            rate: config.agc_rate,
            ..Default::default()
        });

        let order = match config.modulation {
            Modulation::Bpsk => 2,
            Modulation::Qpsk => 4,
            Modulation::Psk8 => 8,
            Modulation::Qam16 => 4, // Use QPSK-like carrier recovery for QAM
            Modulation::Qam64 => 4,
        };

        let costas = CostasLoop::new(CostasConfig {
            order,
            loop_bw: config.loop_bw,
            damping: config.damping,
            ..Default::default()
        });

        let mapper = SymbolMapper::new(config.modulation);

        Self {
            config,
            agc,
            costas,
            mapper,
        }
    }

    /// Process a single sample.
    ///
    /// Returns the carrier-corrected, AGC-normalized symbol.
    pub fn process_sample(&mut self, input: Complex64) -> Complex64 {
        let agc_out = self.agc.process(input);
        self.costas.process_sample(agc_out)
    }

    /// Process a block of samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> ReceiverOutput {
        let mut symbols = Vec::with_capacity(input.len());

        for &s in input {
            symbols.push(self.process_sample(s));
        }

        // Demap symbols to bits
        let (bits, llrs) = if self.config.soft_output {
            let llrs = self.mapper.demap_soft(&symbols, self.config.noise_variance);
            // Also get hard decisions from LLRs
            let bits = llrs.iter().map(|&l| if l > 0.0 { 0u8 } else { 1u8 }).collect();
            (bits, llrs)
        } else {
            let bits = self.mapper.demap_hard(&symbols);
            (bits, Vec::new())
        };

        ReceiverOutput {
            symbols,
            bits,
            llrs,
            freq_offset_rad: self.costas.frequency(),
            agc_gain: self.agc.gain(),
            lock_metric: self.costas.phase_error().abs(),
        }
    }

    /// Get current frequency offset estimate (radians/sample).
    pub fn frequency_offset(&self) -> f64 {
        self.costas.frequency()
    }

    /// Get current frequency offset in Hz.
    pub fn frequency_offset_hz(&self, sample_rate: f64) -> f64 {
        self.costas.frequency_hz(sample_rate)
    }

    /// Get current AGC gain.
    pub fn agc_gain(&self) -> f64 {
        self.agc.gain()
    }

    /// Check if carrier is locked.
    pub fn is_locked(&self) -> bool {
        self.costas.phase_error().abs() < 0.3
    }

    /// Set noise variance for soft decisions.
    pub fn set_noise_variance(&mut self, variance: f64) {
        self.config.noise_variance = variance;
    }

    /// Reset all internal state.
    pub fn reset(&mut self) {
        self.agc.reset();
        self.costas.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn generate_bpsk_signal(
        num_symbols: usize,
        carrier_offset: f64,
        amplitude: f64,
    ) -> Vec<Complex64> {
        let symbols = [1.0f64, -1.0];
        (0..num_symbols)
            .map(|i| {
                let sym = symbols[i % 2];
                let phase = carrier_offset * i as f64;
                Complex64::new(
                    amplitude * sym * phase.cos(),
                    amplitude * sym * phase.sin(),
                )
            })
            .collect()
    }

    #[test]
    fn test_bpsk_receiver_basic() {
        let config = ReceiverConfig::bpsk(0.1);
        let mut rx = ConstellationReceiver::new(config);

        // Clean BPSK signal, no offset
        let signal: Vec<Complex64> = (0..100)
            .map(|i| {
                let sym = if i % 2 == 0 { 1.0 } else { -1.0 };
                Complex64::new(sym, 0.0)
            })
            .collect();

        let result = rx.process_block(&signal);
        assert_eq!(result.symbols.len(), 100);
        assert_eq!(result.bits.len(), 100); // BPSK: 1 bit/symbol
    }

    #[test]
    fn test_bpsk_receiver_with_offset() {
        let config = ReceiverConfig::bpsk(0.1);
        let mut rx = ConstellationReceiver::new(config);

        let signal = generate_bpsk_signal(500, 0.05, 1.0);
        let result = rx.process_block(&signal);

        // Should converge to track the offset
        assert!(
            (result.freq_offset_rad - 0.05).abs() < 0.05,
            "Should track frequency offset: got {:.4}",
            result.freq_offset_rad
        );
    }

    #[test]
    fn test_qpsk_receiver() {
        let config = ReceiverConfig::qpsk(0.08);
        let mut rx = ConstellationReceiver::new(config);

        // QPSK signal
        let qpsk_angles = [PI / 4.0, 3.0 * PI / 4.0, 5.0 * PI / 4.0, 7.0 * PI / 4.0];
        let signal: Vec<Complex64> = (0..200)
            .map(|i| {
                let angle = qpsk_angles[i % 4];
                Complex64::new(angle.cos(), angle.sin())
            })
            .collect();

        let result = rx.process_block(&signal);
        assert_eq!(result.symbols.len(), 200);
        assert_eq!(result.bits.len(), 400); // QPSK: 2 bits/symbol
    }

    #[test]
    fn test_receiver_agc() {
        let config = ReceiverConfig::bpsk(0.05);
        let mut rx = ConstellationReceiver::new(config);

        // Weak BPSK signal
        let signal: Vec<Complex64> = (0..500)
            .map(|i| {
                let sym = if i % 2 == 0 { 0.01 } else { -0.01 };
                Complex64::new(sym, 0.0)
            })
            .collect();

        let result = rx.process_block(&signal);

        // AGC should have increased gain significantly above 1.0
        assert!(
            result.agc_gain > 3.0,
            "AGC should increase gain for weak signal: got {:.1}",
            result.agc_gain
        );
    }

    #[test]
    fn test_soft_output() {
        let mut config = ReceiverConfig::bpsk(0.05);
        config.soft_output = true;
        config.noise_variance = 0.1;
        let mut rx = ConstellationReceiver::new(config);

        let signal: Vec<Complex64> = (0..100)
            .map(|i| {
                let sym = if i % 2 == 0 { 1.0 } else { -1.0 };
                Complex64::new(sym, 0.0)
            })
            .collect();

        let result = rx.process_block(&signal);
        assert!(!result.llrs.is_empty(), "Soft output should produce LLRs");
        assert_eq!(result.llrs.len(), 100); // BPSK: 1 LLR/symbol
    }

    #[test]
    fn test_receiver_reset() {
        let config = ReceiverConfig::bpsk(0.05);
        let mut rx = ConstellationReceiver::new(config);

        let signal = generate_bpsk_signal(200, 0.05, 1.0);
        let _ = rx.process_block(&signal);
        assert!(rx.frequency_offset().abs() > 0.01);

        rx.reset();
        assert!(rx.frequency_offset().abs() < 1e-10);
    }

    #[test]
    fn test_8psk_receiver() {
        let config = ReceiverConfig::psk8(0.05);
        let mut rx = ConstellationReceiver::new(config);

        // 8PSK signal
        let signal: Vec<Complex64> = (0..200)
            .map(|i| {
                let angle = 2.0 * PI * (i % 8) as f64 / 8.0;
                Complex64::new(angle.cos(), angle.sin())
            })
            .collect();

        let result = rx.process_block(&signal);
        assert_eq!(result.symbols.len(), 200);
        assert_eq!(result.bits.len(), 600); // 8PSK: 3 bits/symbol
    }

    #[test]
    fn test_frequency_offset_hz() {
        let config = ReceiverConfig::bpsk(0.05);
        let mut rx = ConstellationReceiver::new(config);

        // Process signal with known offset
        let signal = generate_bpsk_signal(500, 0.05, 1.0);
        let _ = rx.process_block(&signal);

        let hz = rx.frequency_offset_hz(48000.0);
        // 0.05 rad/sample * 48000 / (2*pi) ≈ 382 Hz
        let expected_hz = 0.05 * 48000.0 / (2.0 * PI);
        assert!(
            (hz - expected_hz).abs() < 50.0,
            "Hz offset: got {hz:.1}, expected ~{expected_hz:.1}"
        );
    }
}
