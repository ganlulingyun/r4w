//! Single-Sideband (SSB) Modem and Hilbert Transform
//!
//! SSB modulation removes one sideband, halving the bandwidth of a DSB-AM
//! signal. Classic amateur radio mode (USB for HF, LSB for < 10 MHz).
//!
//! ## Blocks
//!
//! - **HilbertTransform**: FIR-based analytic signal generator
//! - **SsbModulator**: Generates USB or LSB from real audio
//! - **SsbDemodulator**: Recovers audio from SSB signal using product detection
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ssb_modem::{SsbModulator, SsbDemodulator, SidebandMode};
//!
//! let mut modulator = SsbModulator::new(SidebandMode::Usb, 31);
//! let audio: Vec<f64> = (0..200).map(|i| (i as f64 * 0.1).sin()).collect();
//! let iq = modulator.modulate(&audio);
//! assert_eq!(iq.len(), audio.len());
//!
//! let mut demod = SsbDemodulator::new(SidebandMode::Usb);
//! let recovered = demod.demodulate(&iq);
//! assert_eq!(recovered.len(), iq.len());
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Sideband selection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SidebandMode {
    /// Upper sideband (frequencies above carrier).
    Usb,
    /// Lower sideband (frequencies below carrier).
    Lsb,
}

/// FIR Hilbert transform — generates the analytic signal from a real signal.
///
/// The Hilbert transform shifts all frequency components by -90 degrees,
/// creating the imaginary part of the analytic signal: z(t) = x(t) + j*H{x(t)}.
#[derive(Debug, Clone)]
pub struct HilbertTransform {
    /// FIR taps for the Hilbert transform.
    taps: Vec<f64>,
    /// Delay line buffer.
    buffer: Vec<f64>,
    /// Current write position in the circular buffer.
    write_pos: usize,
    /// Filter order (number of taps).
    order: usize,
    /// Group delay in samples.
    group_delay: usize,
}

impl HilbertTransform {
    /// Create a Hilbert transform FIR filter of given order.
    ///
    /// Order must be odd for a Type III FIR (antisymmetric).
    pub fn new(order: usize) -> Self {
        let order = if order % 2 == 0 { order + 1 } else { order };
        let taps = Self::design_taps(order);
        let buffer = vec![0.0; order];
        Self {
            taps,
            buffer,
            write_pos: 0,
            order,
            group_delay: order / 2,
        }
    }

    /// Design Hilbert transform FIR taps with a Blackman window.
    fn design_taps(order: usize) -> Vec<f64> {
        let m = order / 2;
        let mut taps = vec![0.0; order];

        for i in 0..order {
            let n = i as f64 - m as f64;
            if n.abs() < 1e-10 {
                taps[i] = 0.0; // Center tap is zero for Hilbert
            } else if (n as i64) % 2 != 0 {
                // Odd samples: 2/(pi*n)
                taps[i] = 2.0 / (PI * n);
            } else {
                taps[i] = 0.0; // Even samples are zero
            }

            // Apply Blackman window
            let w = 0.42 - 0.5 * (2.0 * PI * i as f64 / (order - 1) as f64).cos()
                + 0.08 * (4.0 * PI * i as f64 / (order - 1) as f64).cos();
            taps[i] *= w;
        }

        taps
    }

    /// Process a single real sample, returning (delayed_real, hilbert_imag).
    pub fn process_sample(&mut self, input: f64) -> (f64, f64) {
        // Store input in circular buffer
        self.buffer[self.write_pos] = input;

        // Compute Hilbert (imaginary part)
        let mut hilbert = 0.0;
        for (k, &tap) in self.taps.iter().enumerate() {
            let idx = (self.write_pos + self.order - k) % self.order;
            hilbert += self.buffer[idx] * tap;
        }

        // Delayed real part (center of filter)
        let delayed_idx = (self.write_pos + self.order - self.group_delay) % self.order;
        let delayed = self.buffer[delayed_idx];

        self.write_pos = (self.write_pos + 1) % self.order;

        (delayed, hilbert)
    }

    /// Process a block of real samples, returning the analytic signal.
    pub fn process(&mut self, input: &[f64]) -> Vec<Complex64> {
        input
            .iter()
            .map(|&x| {
                let (re, im) = self.process_sample(x);
                Complex64::new(re, im)
            })
            .collect()
    }

    /// Get the group delay in samples.
    pub fn group_delay(&self) -> usize {
        self.group_delay
    }

    /// Get the filter order.
    pub fn order(&self) -> usize {
        self.order
    }

    pub fn reset(&mut self) {
        self.buffer.fill(0.0);
        self.write_pos = 0;
    }
}

/// SSB Modulator — generates single-sideband complex signal from real audio.
///
/// Uses Hilbert transform (phasing method):
/// - USB: z(t) = x(t) + j*H{x(t)}
/// - LSB: z(t) = x(t) - j*H{x(t)}
#[derive(Debug, Clone)]
pub struct SsbModulator {
    /// USB or LSB mode.
    mode: SidebandMode,
    /// Hilbert transform filter.
    hilbert: HilbertTransform,
}

impl SsbModulator {
    pub fn new(mode: SidebandMode, hilbert_order: usize) -> Self {
        Self {
            mode,
            hilbert: HilbertTransform::new(hilbert_order),
        }
    }

    /// Modulate real audio to SSB IQ samples.
    pub fn modulate(&mut self, audio: &[f64]) -> Vec<Complex64> {
        audio
            .iter()
            .map(|&x| {
                let (re, im) = self.hilbert.process_sample(x);
                match self.mode {
                    SidebandMode::Usb => Complex64::new(re, im),
                    SidebandMode::Lsb => Complex64::new(re, -im),
                }
            })
            .collect()
    }

    pub fn mode(&self) -> SidebandMode {
        self.mode
    }

    pub fn set_mode(&mut self, mode: SidebandMode) {
        self.mode = mode;
    }

    pub fn reset(&mut self) {
        self.hilbert.reset();
    }
}

/// SSB Demodulator — recovers audio from SSB IQ signal using product detection.
///
/// For USB: audio = Re{z(t)} (real part of baseband)
/// For LSB: audio = Re{z(t)} (same, since LSB was conjugated on TX)
#[derive(Debug, Clone)]
pub struct SsbDemodulator {
    /// USB or LSB mode.
    mode: SidebandMode,
    /// DC removal filter state.
    dc_avg: f64,
    /// DC removal alpha.
    dc_alpha: f64,
}

impl SsbDemodulator {
    pub fn new(mode: SidebandMode) -> Self {
        Self {
            mode,
            dc_avg: 0.0,
            dc_alpha: 0.001,
        }
    }

    /// Demodulate SSB IQ samples to real audio.
    pub fn demodulate(&mut self, iq: &[Complex64]) -> Vec<f64> {
        iq.iter()
            .map(|&z| {
                let audio = match self.mode {
                    SidebandMode::Usb => z.re,
                    SidebandMode::Lsb => z.re,
                };
                // DC removal
                self.dc_avg += self.dc_alpha * (audio - self.dc_avg);
                audio - self.dc_avg
            })
            .collect()
    }

    /// Demodulate with BFO (Beat Frequency Oscillator) offset.
    ///
    /// For CW reception within SSB: mixes with local oscillator at `bfo_freq_hz`.
    pub fn demodulate_with_bfo(
        &mut self,
        iq: &[Complex64],
        bfo_freq_hz: f64,
        sample_rate: f64,
    ) -> Vec<f64> {
        let omega = 2.0 * PI * bfo_freq_hz / sample_rate;
        iq.iter()
            .enumerate()
            .map(|(i, &z)| {
                let lo = Complex64::from_polar(1.0, omega * i as f64);
                let mixed = z * lo;
                let audio = mixed.re;
                self.dc_avg += self.dc_alpha * (audio - self.dc_avg);
                audio - self.dc_avg
            })
            .collect()
    }

    pub fn mode(&self) -> SidebandMode {
        self.mode
    }

    pub fn set_mode(&mut self, mode: SidebandMode) {
        self.mode = mode;
    }

    pub fn reset(&mut self) {
        self.dc_avg = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hilbert_transform_analytic() {
        let mut ht = HilbertTransform::new(31);
        // Feed a cosine — the imaginary part should approximate sine
        let freq = 0.1; // Normalized frequency
        let n = 500;
        let input: Vec<f64> = (0..n).map(|i| (2.0 * PI * freq * i as f64).cos()).collect();

        let analytic = ht.process(&input);
        assert_eq!(analytic.len(), n);

        // After settling (skip group delay + some),
        // imaginary should approximate sin(2*pi*f*n)
        let skip = ht.group_delay() + 50;
        for i in skip..n - 50 {
            let expected_im = (2.0 * PI * freq * (i as f64 - ht.group_delay() as f64)).sin();
            let actual_im = analytic[i].im;
            assert!(
                (actual_im - expected_im).abs() < 0.15,
                "Hilbert at {} should be ~{:.3}, got {:.3}",
                i,
                expected_im,
                actual_im
            );
        }
    }

    #[test]
    fn test_hilbert_transform_group_delay() {
        let ht = HilbertTransform::new(31);
        assert_eq!(ht.group_delay(), 15);
        assert_eq!(ht.order(), 31);
    }

    #[test]
    fn test_hilbert_even_order_rounded() {
        let ht = HilbertTransform::new(30);
        assert_eq!(ht.order(), 31); // Rounded up to odd
    }

    #[test]
    fn test_hilbert_reset() {
        let mut ht = HilbertTransform::new(15);
        let input = vec![1.0; 20];
        ht.process(&input);
        ht.reset();
        // After reset, should behave like new
        let (re, im) = ht.process_sample(0.0);
        assert!((re).abs() < 1e-10);
        assert!((im).abs() < 1e-10);
    }

    #[test]
    fn test_ssb_modulator_usb() {
        let mut mod_usb = SsbModulator::new(SidebandMode::Usb, 31);
        let audio: Vec<f64> = (0..200).map(|i| (i as f64 * 0.1).sin()).collect();
        let iq = mod_usb.modulate(&audio);
        assert_eq!(iq.len(), 200);
        // USB: imaginary part should be Hilbert of real input
        // After settling, signal should have positive frequency content
    }

    #[test]
    fn test_ssb_modulator_lsb() {
        let mut mod_lsb = SsbModulator::new(SidebandMode::Lsb, 31);
        let audio: Vec<f64> = (0..200).map(|i| (i as f64 * 0.1).sin()).collect();
        let iq = mod_lsb.modulate(&audio);
        assert_eq!(iq.len(), 200);
    }

    #[test]
    fn test_ssb_usb_vs_lsb_conjugate() {
        let mut mod_usb = SsbModulator::new(SidebandMode::Usb, 31);
        let mut mod_lsb = SsbModulator::new(SidebandMode::Lsb, 31);
        let audio: Vec<f64> = (0..100).map(|i| (i as f64 * 0.2).sin()).collect();

        let iq_usb = mod_usb.modulate(&audio);
        let iq_lsb = mod_lsb.modulate(&audio);

        // USB and LSB should have same real parts but opposite imaginary
        for (u, l) in iq_usb.iter().zip(iq_lsb.iter()) {
            assert!((u.re - l.re).abs() < 1e-10);
            assert!((u.im + l.im).abs() < 1e-10);
        }
    }

    #[test]
    fn test_ssb_demodulator_basic() {
        let mut demod = SsbDemodulator::new(SidebandMode::Usb);
        let iq: Vec<Complex64> = (0..100)
            .map(|i| Complex64::new((i as f64 * 0.1).sin(), (i as f64 * 0.1).cos()))
            .collect();
        let audio = demod.demodulate(&iq);
        assert_eq!(audio.len(), 100);
    }

    #[test]
    fn test_ssb_roundtrip() {
        let mut modulator = SsbModulator::new(SidebandMode::Usb, 63);
        let mut demod = SsbDemodulator::new(SidebandMode::Usb);

        // 1 kHz tone at 8 kHz sample rate
        let n = 1000;
        let freq = 1000.0 / 8000.0;
        let audio: Vec<f64> = (0..n).map(|i| (2.0 * PI * freq * i as f64).sin()).collect();

        let iq = modulator.modulate(&audio);
        let recovered = demod.demodulate(&iq);

        // After Hilbert settling, recovered should correlate with original
        let skip = 100; // Skip transient
        let mut corr = 0.0;
        let mut power_orig = 0.0;
        let mut power_recv = 0.0;
        for i in skip..n - 50 {
            // Account for Hilbert group delay
            let delay = modulator.hilbert.group_delay();
            if i >= delay {
                let orig = audio[i - delay];
                let recv = recovered[i];
                corr += orig * recv;
                power_orig += orig * orig;
                power_recv += recv * recv;
            }
        }
        if power_orig > 0.0 && power_recv > 0.0 {
            let normalized = corr / (power_orig.sqrt() * power_recv.sqrt());
            assert!(
                normalized > 0.8,
                "SSB roundtrip correlation should be high: got {}",
                normalized
            );
        }
    }

    #[test]
    fn test_ssb_demod_with_bfo() {
        let mut demod = SsbDemodulator::new(SidebandMode::Usb);
        let iq: Vec<Complex64> = (0..200)
            .map(|i| Complex64::new((i as f64 * 0.05).cos(), (i as f64 * 0.05).sin()))
            .collect();
        let audio = demod.demodulate_with_bfo(&iq, 700.0, 8000.0);
        assert_eq!(audio.len(), 200);
        // BFO mixing should produce audio-frequency beat note
    }

    #[test]
    fn test_ssb_mode_switch() {
        let mut modulator = SsbModulator::new(SidebandMode::Usb, 31);
        assert_eq!(modulator.mode(), SidebandMode::Usb);
        modulator.set_mode(SidebandMode::Lsb);
        assert_eq!(modulator.mode(), SidebandMode::Lsb);
    }

    #[test]
    fn test_ssb_demod_reset() {
        let mut demod = SsbDemodulator::new(SidebandMode::Usb);
        let iq = vec![Complex64::new(1.0, 0.0); 100];
        demod.demodulate(&iq);
        demod.reset();
        // After reset, DC average should be zero
        let audio = demod.demodulate(&[Complex64::new(0.0, 0.0)]);
        assert!(audio[0].abs() < 0.01);
    }
}
