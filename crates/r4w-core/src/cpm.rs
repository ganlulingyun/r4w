//! Continuous Phase Modulation (CPM) — GFSK, GMSK, MSK
//!
//! CPM produces a constant-envelope signal with smooth phase transitions,
//! enabling efficient nonlinear power amplifiers. Used in GSM (GMSK),
//! Bluetooth (GFSK), and various military/satellite links.
//!
//! ## Phase response shapes
//!
//! - **LREC**: Rectangular pulse → MSK when h=0.5
//! - **LRC**: Raised cosine
//! - **Gaussian**: Gaussian pulse → GMSK when h=0.5, BT=0.3 (GSM)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cpm::{CpmModulator, CpmConfig, CpmDemodulator};
//!
//! let config = CpmConfig::gmsk_gsm(8);
//! let mut modulator = CpmModulator::new(config.clone());
//! let symbols: Vec<i8> = vec![1, -1, 1, 1, -1, -1, 1, -1];
//! let iq = modulator.modulate(&symbols);
//! assert_eq!(iq.len(), symbols.len() * 8);
//!
//! let mut demod = CpmDemodulator::new(config);
//! let recovered = demod.demodulate_noncoherent(&iq);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Phase response shape for CPM.
#[derive(Debug, Clone, PartialEq)]
pub enum CpmType {
    /// Rectangular (L-REC). h=0.5 yields MSK.
    Lrec,
    /// Raised cosine (L-RC).
    Lrc,
    /// Gaussian. BT=0.3 + h=0.5 yields standard GMSK (GSM).
    Gaussian { bt: f64 },
}

/// CPM configuration.
#[derive(Debug, Clone)]
pub struct CpmConfig {
    pub cpm_type: CpmType,
    pub modulation_index: f64,
    pub samples_per_symbol: usize,
    pub pulse_duration: usize,
    pub alphabet_size: usize,
}

impl CpmConfig {
    /// MSK: h=0.5, LREC, L=1
    pub fn msk(sps: usize) -> Self {
        Self {
            cpm_type: CpmType::Lrec,
            modulation_index: 0.5,
            samples_per_symbol: sps,
            pulse_duration: 1,
            alphabet_size: 2,
        }
    }

    /// GMSK for GSM: h=0.5, Gaussian BT=0.3, L=3
    pub fn gmsk_gsm(sps: usize) -> Self {
        Self {
            cpm_type: CpmType::Gaussian { bt: 0.3 },
            modulation_index: 0.5,
            samples_per_symbol: sps,
            pulse_duration: 3,
            alphabet_size: 2,
        }
    }

    /// GFSK for Bluetooth: h=0.5, Gaussian BT=0.5, L=2
    pub fn gfsk_bluetooth(sps: usize) -> Self {
        Self {
            cpm_type: CpmType::Gaussian { bt: 0.5 },
            modulation_index: 0.5,
            samples_per_symbol: sps,
            pulse_duration: 2,
            alphabet_size: 2,
        }
    }

    /// Generic CPM.
    pub fn new(cpm_type: CpmType, h: f64, sps: usize, pulse_dur: usize, m: usize) -> Self {
        Self {
            cpm_type,
            modulation_index: h,
            samples_per_symbol: sps,
            pulse_duration: pulse_dur,
            alphabet_size: m,
        }
    }
}

/// CPM Modulator — generates constant-envelope IQ from symbol stream.
#[derive(Debug, Clone)]
pub struct CpmModulator {
    config: CpmConfig,
    /// Phase pulse g(t) — frequency shaping (integrated to get phase).
    freq_pulse: Vec<f64>,
    /// Accumulated phase.
    phase: f64,
    /// Symbol history for partial-response overlap.
    symbol_history: Vec<i8>,
}

impl CpmModulator {
    pub fn new(config: CpmConfig) -> Self {
        let freq_pulse = Self::compute_freq_pulse(&config);
        let history_len = config.pulse_duration;
        Self {
            config,
            freq_pulse,
            phase: 0.0,
            symbol_history: vec![0; history_len],
        }
    }

    /// Compute frequency pulse g(t) for the given CPM type.
    fn compute_freq_pulse(config: &CpmConfig) -> Vec<f64> {
        let len = config.pulse_duration * config.samples_per_symbol;
        let sps = config.samples_per_symbol as f64;
        let l = config.pulse_duration as f64;

        let mut pulse = vec![0.0; len];

        match &config.cpm_type {
            CpmType::Lrec => {
                // Rectangular: constant 1/(2*L*T) over L symbols
                let val = 1.0 / (2.0 * l * sps);
                for p in pulse.iter_mut() {
                    *p = val;
                }
            }
            CpmType::Lrc => {
                // Raised cosine
                for (i, p) in pulse.iter_mut().enumerate() {
                    let t = i as f64 / sps - l / 2.0;
                    *p = (1.0 - (2.0 * PI * t / l).cos()) / (2.0 * l * sps);
                }
            }
            CpmType::Gaussian { bt } => {
                // Gaussian frequency pulse
                let sigma = (2.0 * PI * bt).recip() * (2.0_f64.ln()).sqrt();
                for (i, p) in pulse.iter_mut().enumerate() {
                    let t = i as f64 / sps - l / 2.0;
                    let gauss = (-t * t / (2.0 * sigma * sigma)).exp()
                        / (sigma * (2.0 * PI).sqrt());
                    *p = gauss / (2.0 * sps);
                }
                // Normalize so total area = 0.5 (phase change of h*pi per symbol)
                let sum: f64 = pulse.iter().sum();
                if sum > 1e-10 {
                    let scale = 0.5 / sum;
                    for p in pulse.iter_mut() {
                        *p *= scale;
                    }
                }
            }
        }

        pulse
    }

    /// Modulate symbol stream to IQ samples.
    ///
    /// Symbols are from the alphabet {-(M-1), -(M-3), ..., (M-3), (M-1)}.
    /// For binary (M=2): symbols are {-1, +1}.
    pub fn modulate(&mut self, symbols: &[i8]) -> Vec<Complex64> {
        let sps = self.config.samples_per_symbol;
        let h = self.config.modulation_index;
        let pulse_len = self.freq_pulse.len();
        let mut output = Vec::with_capacity(symbols.len() * sps);

        for &sym in symbols {
            // Shift symbol history
            self.symbol_history.rotate_left(1);
            *self.symbol_history.last_mut().unwrap() = sym;

            // Generate sps samples for this symbol period
            for k in 0..sps {
                // Instantaneous frequency from overlapping pulses
                let mut inst_freq = 0.0;
                for (si, &s) in self.symbol_history.iter().enumerate() {
                    // Pulse index for this symbol's contribution
                    let pulse_offset = (self.config.pulse_duration - 1 - si) * sps + k;
                    if pulse_offset < pulse_len {
                        inst_freq += s as f64 * self.freq_pulse[pulse_offset];
                    }
                }

                self.phase += 2.0 * PI * h * inst_freq;
                // Keep phase bounded
                while self.phase > PI {
                    self.phase -= 2.0 * PI;
                }
                while self.phase < -PI {
                    self.phase += 2.0 * PI;
                }

                output.push(Complex64::from_polar(1.0, self.phase));
            }
        }

        output
    }

    pub fn config(&self) -> &CpmConfig {
        &self.config
    }

    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.symbol_history.fill(0);
    }
}

/// CPM Demodulator — recovers symbols from CPM signal.
#[derive(Debug, Clone)]
pub struct CpmDemodulator {
    config: CpmConfig,
    prev_sample: Complex64,
}

impl CpmDemodulator {
    pub fn new(config: CpmConfig) -> Self {
        Self {
            config,
            prev_sample: Complex64::new(1.0, 0.0),
        }
    }

    /// Non-coherent demodulation via differential phase (quadrature demod + integrate-and-dump).
    pub fn demodulate_noncoherent(&mut self, samples: &[Complex64]) -> Vec<i8> {
        let sps = self.config.samples_per_symbol;
        let h = self.config.modulation_index;

        // Phase difference (FM discriminator)
        let mut phase_diffs = Vec::with_capacity(samples.len());
        for &s in samples {
            let diff = s * self.prev_sample.conj();
            phase_diffs.push(diff.arg());
            self.prev_sample = s;
        }

        // Integrate-and-dump over each symbol period
        let num_symbols = phase_diffs.len() / sps;
        let mut symbols = Vec::with_capacity(num_symbols);

        for i in 0..num_symbols {
            let start = i * sps;
            let end = start + sps;
            let sum: f64 = phase_diffs[start..end].iter().sum();
            let avg = sum / sps as f64;

            // Decision: positive → +1, negative → -1
            if self.config.alphabet_size <= 2 {
                symbols.push(if avg >= 0.0 { 1 } else { -1 });
            } else {
                // M-ary: quantize to nearest symbol level
                let m = self.config.alphabet_size as f64;
                let step = PI * h / (m / 2.0);
                let level = (avg / step).round() as i8;
                let max_level = (self.config.alphabet_size as i8) - 1;
                symbols.push(level.clamp(-max_level, max_level));
            }
        }

        symbols
    }

    pub fn config(&self) -> &CpmConfig {
        &self.config
    }

    pub fn reset(&mut self) {
        self.prev_sample = Complex64::new(1.0, 0.0);
    }
}

/// Compute the power spectral density of a CPM signal for given configuration.
/// Returns (frequencies, psd_db) for visualization.
pub fn cpm_spectrum(config: &CpmConfig, num_symbols: usize) -> (Vec<f64>, Vec<f64>) {
    let mut mod_ = CpmModulator::new(config.clone());
    // Generate random symbols
    let symbols: Vec<i8> = (0..num_symbols)
        .map(|i| if i % 3 == 0 { 1 } else { -1 })
        .collect();
    let iq = mod_.modulate(&symbols);

    let n = iq.len().next_power_of_two();
    let mut padded: Vec<Complex64> = iq;
    padded.resize(n, Complex64::new(0.0, 0.0));

    // Simple DFT magnitude (for spectrum visualization)
    let mut spectrum = vec![0.0f64; n];
    let sps = config.samples_per_symbol as f64;

    for k in 0..n {
        let mut sum = Complex64::new(0.0, 0.0);
        for (i, &s) in padded.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
            sum += s * Complex64::from_polar(1.0, angle);
        }
        let mag_sq = sum.norm_sqr() / (n as f64 * n as f64);
        spectrum[k] = 10.0 * (mag_sq.max(1e-20)).log10();
    }

    let freqs: Vec<f64> = (0..n)
        .map(|k| {
            let f = k as f64 / n as f64;
            if f > 0.5 { f - 1.0 } else { f }
        })
        .map(|f| f * sps)
        .collect();

    (freqs, spectrum)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_msk_constant_envelope() {
        let config = CpmConfig::msk(8);
        let mut mod_ = CpmModulator::new(config);
        let symbols = vec![1, -1, 1, 1, -1, -1, 1, -1, 1, -1];
        let iq = mod_.modulate(&symbols);

        // CPM should be constant envelope
        for s in &iq {
            let mag = s.norm();
            assert!(
                (mag - 1.0).abs() < 1e-10,
                "Envelope should be 1.0, got {}",
                mag
            );
        }
    }

    #[test]
    fn test_gmsk_constant_envelope() {
        let config = CpmConfig::gmsk_gsm(8);
        let mut mod_ = CpmModulator::new(config);
        let symbols = vec![1, -1, 1, 1, -1, 1, -1, -1];
        let iq = mod_.modulate(&symbols);

        for s in &iq {
            let mag = s.norm();
            assert!(
                (mag - 1.0).abs() < 1e-10,
                "GMSK envelope should be 1.0, got {}",
                mag
            );
        }
    }

    #[test]
    fn test_msk_output_length() {
        let sps = 4;
        let config = CpmConfig::msk(sps);
        let mut mod_ = CpmModulator::new(config);
        let symbols = vec![1, -1, 1];
        let iq = mod_.modulate(&symbols);
        assert_eq!(iq.len(), 3 * sps);
    }

    #[test]
    fn test_msk_roundtrip() {
        let sps = 8;
        let config = CpmConfig::msk(sps);
        let mut mod_ = CpmModulator::new(config.clone());
        let mut demod = CpmDemodulator::new(config);

        let symbols = vec![1, -1, 1, 1, -1, -1, 1, -1, 1, 1, -1, 1];
        let iq = mod_.modulate(&symbols);
        let recovered = demod.demodulate_noncoherent(&iq);

        // Skip first symbol (transient from pulse overlap)
        let skip = 1;
        let mut correct = 0;
        for i in skip..symbols.len().min(recovered.len()) {
            if symbols[i] == recovered[i] {
                correct += 1;
            }
        }
        let ber = 1.0 - correct as f64 / (symbols.len() - skip) as f64;
        assert!(ber < 0.2, "MSK BER should be low, got {}", ber);
    }

    #[test]
    fn test_gmsk_modulator_output() {
        // GMSK with partial response (L=3) has ISI — non-coherent demod
        // is suboptimal. Just verify modulator produces correct length
        // and constant envelope.
        let sps = 8;
        let config = CpmConfig::gmsk_gsm(sps);
        let mut mod_ = CpmModulator::new(config);

        let symbols = vec![1, -1, 1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, 1, -1];
        let iq = mod_.modulate(&symbols);
        assert_eq!(iq.len(), symbols.len() * sps);

        // Constant envelope
        for s in &iq {
            assert!((s.norm() - 1.0).abs() < 1e-10);
        }

        // Phase should be continuous
        for i in 1..iq.len() {
            let diff = (iq[i] * iq[i - 1].conj()).arg().abs();
            assert!(diff < 0.5, "Phase discontinuity at {}: {}", i, diff);
        }
    }

    #[test]
    fn test_bluetooth_gfsk() {
        let config = CpmConfig::gfsk_bluetooth(4);
        let mut mod_ = CpmModulator::new(config);
        let symbols = vec![1, -1, 1, 1, -1];
        let iq = mod_.modulate(&symbols);
        assert_eq!(iq.len(), 5 * 4);
        // Constant envelope
        for s in &iq {
            assert!((s.norm() - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_cpm_phase_continuity() {
        let config = CpmConfig::msk(16);
        let mut mod_ = CpmModulator::new(config);
        let symbols = vec![1, -1, 1, -1, 1, 1, -1, -1];
        let iq = mod_.modulate(&symbols);

        // Phase should be continuous — no jumps > pi*h per sample
        for i in 1..iq.len() {
            let phase_diff = (iq[i] * iq[i - 1].conj()).arg().abs();
            assert!(
                phase_diff < 0.5,
                "Phase jump at sample {} too large: {}",
                i,
                phase_diff
            );
        }
    }

    #[test]
    fn test_cpm_reset() {
        let config = CpmConfig::msk(4);
        let mut mod_ = CpmModulator::new(config);
        let symbols = vec![1, -1, 1];
        mod_.modulate(&symbols);
        mod_.reset();

        let iq = mod_.modulate(&[1]);
        // After reset, first sample should start near phase 0
        assert!(iq[0].im.abs() < 0.5, "Phase should start near 0 after reset");
    }

    #[test]
    fn test_demod_reset() {
        let config = CpmConfig::msk(4);
        let mut demod = CpmDemodulator::new(config);
        let iq: Vec<Complex64> = (0..16).map(|i| Complex64::from_polar(1.0, i as f64 * 0.1)).collect();
        demod.demodulate_noncoherent(&iq);
        demod.reset();
        assert_eq!(demod.prev_sample, Complex64::new(1.0, 0.0));
    }

    #[test]
    fn test_lrc_constant_envelope() {
        let config = CpmConfig::new(CpmType::Lrc, 0.5, 8, 2, 2);
        let mut mod_ = CpmModulator::new(config);
        let symbols = vec![1, -1, 1, 1, -1];
        let iq = mod_.modulate(&symbols);
        for s in &iq {
            assert!((s.norm() - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_freq_pulse_normalization() {
        // For Gaussian, the frequency pulse should integrate to 0.5
        let config = CpmConfig::gmsk_gsm(16);
        let pulse = CpmModulator::compute_freq_pulse(&config);
        let sum: f64 = pulse.iter().sum();
        assert!(
            (sum - 0.5).abs() < 0.01,
            "Gaussian pulse integral should be ~0.5, got {}",
            sum
        );
    }

    #[test]
    fn test_cpm_spectrum_runs() {
        let config = CpmConfig::msk(4);
        let (freqs, psd) = cpm_spectrum(&config, 32);
        assert!(!freqs.is_empty());
        assert_eq!(freqs.len(), psd.len());
    }
}
