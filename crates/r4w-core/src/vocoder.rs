//! Vocoder — Voice Codec Processing
//!
//! Multi-codec voice encoder/decoder supporting simplified CODEC2-style
//! LPC-based vocoding for digital voice communications. Essential for
//! amateur radio (DMR, D-STAR, System Fusion), satellite communications,
//! and low-bitrate voice links.
//! GNU Radio equivalent: `gr::vocoder` (CODEC2, CVSD, G.721/723, GSM).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::vocoder::{Vocoder, VocoderMode};
//!
//! let mut encoder = Vocoder::new(VocoderMode::Lpc10, 8000);
//! let audio = vec![0.0f32; 160]; // 20ms frame at 8 kHz
//! let encoded = encoder.encode(&audio);
//! let decoded = encoder.decode(&encoded);
//! assert_eq!(decoded.len(), 160);
//! ```

use std::f32::consts::PI;

/// Voice codec mode / bitrate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VocoderMode {
    /// LPC-10 style vocoder (~2400 bps).
    Lpc10,
    /// Simplified CODEC2 1600 bps mode.
    Codec2_1600,
    /// Simplified CODEC2 3200 bps mode.
    Codec2_3200,
    /// CVSD (Continuously Variable Slope Delta) 16 kbps.
    Cvsd,
    /// Mu-law PCM companding (64 kbps).
    MuLaw,
}

impl VocoderMode {
    /// Nominal bitrate in bits per second.
    pub fn bitrate(&self) -> u32 {
        match self {
            VocoderMode::Lpc10 => 2400,
            VocoderMode::Codec2_1600 => 1600,
            VocoderMode::Codec2_3200 => 3200,
            VocoderMode::Cvsd => 16000,
            VocoderMode::MuLaw => 64000,
        }
    }

    /// Frame size in samples at given sample rate.
    pub fn frame_size(&self, sample_rate: u32) -> usize {
        match self {
            VocoderMode::Lpc10 => (sample_rate as usize) / 50, // 20ms
            VocoderMode::Codec2_1600 => (sample_rate as usize) * 40 / 1000, // 40ms
            VocoderMode::Codec2_3200 => (sample_rate as usize) / 50, // 20ms
            VocoderMode::Cvsd => 1, // sample-by-sample
            VocoderMode::MuLaw => 1,
        }
    }
}

/// LPC analysis parameters for a single frame.
#[derive(Debug, Clone)]
pub struct LpcFrame {
    /// LPC coefficients (order 10).
    pub coeffs: Vec<f32>,
    /// Pitch period in samples (0 = unvoiced).
    pub pitch: u16,
    /// Frame energy (log scale).
    pub energy: f32,
    /// Voiced flag.
    pub voiced: bool,
}

/// Voice codec encoder/decoder.
#[derive(Debug, Clone)]
pub struct Vocoder {
    mode: VocoderMode,
    sample_rate: u32,
    lpc_order: usize,
    /// CVSD state: integrator value.
    cvsd_integrator: f32,
    /// CVSD state: step size.
    cvsd_step: f32,
    /// CVSD state: previous bits for slope detection.
    cvsd_prev_bits: u8,
    /// Synthesis filter memory.
    synth_memory: Vec<f32>,
}

impl Vocoder {
    /// Create a new vocoder.
    pub fn new(mode: VocoderMode, sample_rate: u32) -> Self {
        let lpc_order = 10;
        Self {
            mode,
            sample_rate,
            lpc_order,
            cvsd_integrator: 0.0,
            cvsd_step: 0.01,
            cvsd_prev_bits: 0,
            synth_memory: vec![0.0; lpc_order],
        }
    }

    /// Get the codec mode.
    pub fn mode(&self) -> VocoderMode {
        self.mode
    }

    /// Get frame size in samples.
    pub fn frame_size(&self) -> usize {
        self.mode.frame_size(self.sample_rate)
    }

    /// Encode audio samples to compressed bytes.
    pub fn encode(&mut self, samples: &[f32]) -> Vec<u8> {
        match self.mode {
            VocoderMode::Lpc10 | VocoderMode::Codec2_1600 | VocoderMode::Codec2_3200 => {
                self.encode_lpc(samples)
            }
            VocoderMode::Cvsd => self.encode_cvsd(samples),
            VocoderMode::MuLaw => self.encode_mulaw(samples),
        }
    }

    /// Decode compressed bytes to audio samples.
    pub fn decode(&mut self, data: &[u8]) -> Vec<f32> {
        match self.mode {
            VocoderMode::Lpc10 | VocoderMode::Codec2_1600 | VocoderMode::Codec2_3200 => {
                self.decode_lpc(data)
            }
            VocoderMode::Cvsd => self.decode_cvsd(data),
            VocoderMode::MuLaw => self.decode_mulaw(data),
        }
    }

    /// Perform LPC analysis on a frame.
    pub fn analyze_lpc(&self, frame: &[f32]) -> LpcFrame {
        let energy = frame.iter().map(|&s| s * s).sum::<f32>() / frame.len() as f32;
        let log_energy = if energy > 1e-10 { energy.log10() * 10.0 } else { -100.0 };

        // Autocorrelation
        let mut r = vec![0.0f32; self.lpc_order + 1];
        for k in 0..=self.lpc_order {
            for i in k..frame.len() {
                r[k] += frame[i] * frame[i - k];
            }
        }

        // Levinson-Durbin
        let coeffs = levinson_durbin(&r, self.lpc_order);

        // Pitch detection (autocorrelation method)
        let min_lag = self.sample_rate as usize / 400; // 400 Hz max
        let max_lag = self.sample_rate as usize / 50;  // 50 Hz min
        let (pitch, voiced) = detect_pitch(frame, min_lag, max_lag);

        LpcFrame {
            coeffs,
            pitch,
            energy: log_energy,
            voiced,
        }
    }

    /// Synthesize audio from LPC parameters.
    pub fn synthesize_lpc(&mut self, frame: &LpcFrame, num_samples: usize) -> Vec<f32> {
        let mut output = vec![0.0f32; num_samples];

        // Generate excitation
        let excitation: Vec<f32> = if frame.voiced && frame.pitch > 0 {
            // Impulse train
            let mut exc = vec![0.0f32; num_samples];
            let period = frame.pitch as usize;
            if period > 0 {
                for i in (0..num_samples).step_by(period) {
                    exc[i] = 1.0;
                }
            }
            exc
        } else {
            // White noise for unvoiced
            let mut rng = 12345u32;
            (0..num_samples)
                .map(|_| {
                    rng = rng.wrapping_mul(1103515245).wrapping_add(12345);
                    ((rng >> 16) as f32 / 32768.0) - 1.0
                })
                .collect()
        };

        // Scale excitation by energy
        let gain = (10.0f32.powf(frame.energy / 20.0)).min(10.0);

        // All-pole synthesis filter
        for i in 0..num_samples {
            output[i] = excitation[i] * gain;
            for k in 0..frame.coeffs.len().min(self.lpc_order) {
                if i > k {
                    output[i] -= frame.coeffs[k] * output[i - k - 1];
                } else if k < self.synth_memory.len() {
                    let mem_idx = self.synth_memory.len() - 1 - (k - i);
                    if mem_idx < self.synth_memory.len() {
                        output[i] -= frame.coeffs[k] * self.synth_memory[mem_idx];
                    }
                }
            }
        }

        // Update filter memory
        let start = if num_samples >= self.lpc_order {
            num_samples - self.lpc_order
        } else {
            0
        };
        for (j, item) in self.synth_memory.iter_mut().enumerate() {
            if start + j < num_samples {
                *item = output[start + j];
            }
        }

        output
    }

    fn encode_lpc(&mut self, samples: &[f32]) -> Vec<u8> {
        let frame = self.analyze_lpc(samples);
        // Pack LPC frame into bytes
        let mut bytes = Vec::new();
        // Voiced flag + pitch (2 bytes)
        let pitch_val = if frame.voiced { frame.pitch } else { 0 };
        bytes.push((pitch_val >> 8) as u8);
        bytes.push((pitch_val & 0xFF) as u8);
        // Energy (2 bytes, fixed point)
        let energy_q = ((frame.energy + 100.0) * 100.0) as u16;
        bytes.push((energy_q >> 8) as u8);
        bytes.push((energy_q & 0xFF) as u8);
        // LPC coefficients (2 bytes each, Q15)
        for &c in &frame.coeffs {
            let q = (c.clamp(-1.0, 1.0) * 32767.0) as i16;
            bytes.push((q >> 8) as u8);
            bytes.push((q & 0xFF) as u8);
        }
        bytes
    }

    fn decode_lpc(&mut self, data: &[u8]) -> Vec<f32> {
        if data.len() < 4 {
            return vec![0.0; self.frame_size()];
        }
        // Unpack
        let pitch = ((data[0] as u16) << 8) | (data[1] as u16);
        let energy_q = ((data[2] as u16) << 8) | (data[3] as u16);
        let energy = (energy_q as f32 / 100.0) - 100.0;
        let voiced = pitch > 0;

        let mut coeffs = Vec::new();
        let mut i = 4;
        while i + 1 < data.len() && coeffs.len() < self.lpc_order {
            let q = ((data[i] as i16) << 8) | (data[i + 1] as i16);
            coeffs.push(q as f32 / 32767.0);
            i += 2;
        }
        while coeffs.len() < self.lpc_order {
            coeffs.push(0.0);
        }

        let frame = LpcFrame {
            coeffs,
            pitch,
            energy,
            voiced,
        };
        self.synthesize_lpc(&frame, self.frame_size())
    }

    fn encode_cvsd(&mut self, samples: &[f32]) -> Vec<u8> {
        let mut bits = Vec::new();
        let mut byte = 0u8;
        let mut bit_count = 0;

        for &s in samples {
            let bit = if s > self.cvsd_integrator { 1u8 } else { 0u8 };

            // Slope overload detection (3 consecutive same bits)
            self.cvsd_prev_bits = ((self.cvsd_prev_bits << 1) | bit) & 0x07;
            if self.cvsd_prev_bits == 0x07 || self.cvsd_prev_bits == 0x00 {
                self.cvsd_step = (self.cvsd_step * 1.5).min(0.5);
            } else {
                self.cvsd_step = (self.cvsd_step * 0.9).max(0.001);
            }

            if bit == 1 {
                self.cvsd_integrator += self.cvsd_step;
            } else {
                self.cvsd_integrator -= self.cvsd_step;
            }
            self.cvsd_integrator *= 0.999; // Leaky integrator

            byte = (byte << 1) | bit;
            bit_count += 1;
            if bit_count == 8 {
                bits.push(byte);
                byte = 0;
                bit_count = 0;
            }
        }
        if bit_count > 0 {
            byte <<= 8 - bit_count;
            bits.push(byte);
        }
        bits
    }

    fn decode_cvsd(&mut self, data: &[u8]) -> Vec<f32> {
        let mut output = Vec::new();

        for &byte in data {
            for bit_idx in (0..8).rev() {
                let bit = (byte >> bit_idx) & 1;

                self.cvsd_prev_bits = ((self.cvsd_prev_bits << 1) | bit) & 0x07;
                if self.cvsd_prev_bits == 0x07 || self.cvsd_prev_bits == 0x00 {
                    self.cvsd_step = (self.cvsd_step * 1.5).min(0.5);
                } else {
                    self.cvsd_step = (self.cvsd_step * 0.9).max(0.001);
                }

                if bit == 1 {
                    self.cvsd_integrator += self.cvsd_step;
                } else {
                    self.cvsd_integrator -= self.cvsd_step;
                }
                self.cvsd_integrator *= 0.999;

                output.push(self.cvsd_integrator);
            }
        }
        output
    }

    fn encode_mulaw(&self, samples: &[f32]) -> Vec<u8> {
        samples.iter().map(|&s| mulaw_encode(s)).collect()
    }

    fn decode_mulaw(&self, data: &[u8]) -> Vec<f32> {
        data.iter().map(|&b| mulaw_decode(b)).collect()
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.cvsd_integrator = 0.0;
        self.cvsd_step = 0.01;
        self.cvsd_prev_bits = 0;
        self.synth_memory = vec![0.0; self.lpc_order];
    }
}

/// Mu-law encode a single sample [-1.0, 1.0] to u8.
pub fn mulaw_encode(sample: f32) -> u8 {
    const MU: f32 = 255.0;
    let s = sample.clamp(-1.0, 1.0);
    let sign = if s < 0.0 { 0x80u8 } else { 0x00u8 };
    let magnitude = ((1.0 + MU * s.abs()).ln() / (1.0 + MU).ln() * 127.0) as u8;
    sign | magnitude
}

/// Mu-law decode a u8 to sample [-1.0, 1.0].
pub fn mulaw_decode(byte: u8) -> f32 {
    const MU: f32 = 255.0;
    let sign = if byte & 0x80 != 0 { -1.0f32 } else { 1.0 };
    let magnitude = (byte & 0x7F) as f32 / 127.0;
    sign * ((1.0 + MU).powf(magnitude) - 1.0) / MU
}

/// Levinson-Durbin recursion for LPC coefficient computation.
fn levinson_durbin(r: &[f32], order: usize) -> Vec<f32> {
    let mut a = vec![0.0f32; order];
    let mut a_prev = vec![0.0f32; order];

    if r[0].abs() < 1e-10 {
        return a;
    }

    a[0] = -r[1] / r[0];
    let mut err = r[0] * (1.0 - a[0] * a[0]);

    for m in 1..order {
        let mut sum = r[m + 1];
        for k in 0..m {
            sum += a[k] * r[m - k];
        }

        if err.abs() < 1e-10 {
            break;
        }

        let lambda = -sum / err;
        a_prev[..m].copy_from_slice(&a[..m]);

        for k in 0..m {
            a[k] = a_prev[k] + lambda * a_prev[m - 1 - k];
        }
        a[m] = lambda;
        err *= 1.0 - lambda * lambda;
    }

    a
}

/// Simple autocorrelation pitch detector.
fn detect_pitch(frame: &[f32], min_lag: usize, max_lag: usize) -> (u16, bool) {
    let n = frame.len();
    if n < max_lag + 1 {
        return (0, false);
    }

    let max_lag = max_lag.min(n - 1);
    let mut best_lag = 0usize;
    let mut best_corr = 0.0f32;
    let r0: f32 = frame.iter().map(|&s| s * s).sum();

    if r0 < 1e-10 {
        return (0, false);
    }

    for lag in min_lag..=max_lag {
        let mut corr = 0.0f32;
        for i in 0..n - lag {
            corr += frame[i] * frame[i + lag];
        }
        corr /= r0;

        if corr > best_corr {
            best_corr = corr;
            best_lag = lag;
        }
    }

    let voiced = best_corr > 0.3;
    let pitch = if voiced { best_lag as u16 } else { 0 };
    (pitch, voiced)
}

/// Compute speech quality metric (SNR-like).
pub fn speech_quality(original: &[f32], reconstructed: &[f32]) -> f32 {
    let n = original.len().min(reconstructed.len());
    if n == 0 {
        return 0.0;
    }

    let signal_power: f32 = original[..n].iter().map(|&s| s * s).sum::<f32>() / n as f32;
    let noise_power: f32 = original[..n]
        .iter()
        .zip(reconstructed[..n].iter())
        .map(|(&a, &b)| (a - b) * (a - b))
        .sum::<f32>()
        / n as f32;

    if noise_power < 1e-10 {
        return 100.0;
    }
    10.0 * (signal_power / noise_power).log10()
}

/// Generate a simple test tone for vocoder testing.
pub fn generate_test_tone(frequency: f32, sample_rate: u32, num_samples: usize) -> Vec<f32> {
    (0..num_samples)
        .map(|i| (2.0 * PI * frequency * i as f32 / sample_rate as f32).sin() * 0.5)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mulaw_roundtrip() {
        for val in [-1.0, -0.5, 0.0, 0.5, 1.0] {
            let encoded = mulaw_encode(val);
            let decoded = mulaw_decode(encoded);
            assert!((decoded - val).abs() < 0.02, "mu-law roundtrip failed for {}", val);
        }
    }

    #[test]
    fn test_mulaw_encode_decode_zero() {
        let encoded = mulaw_encode(0.0);
        let decoded = mulaw_decode(encoded);
        assert!(decoded.abs() < 0.01);
    }

    #[test]
    fn test_vocoder_mulaw() {
        let mut voc = Vocoder::new(VocoderMode::MuLaw, 8000);
        let tone = generate_test_tone(440.0, 8000, 160);
        let encoded = voc.encode(&tone);
        let decoded = voc.decode(&encoded);
        assert_eq!(decoded.len(), encoded.len());
        let snr = speech_quality(&tone, &decoded);
        assert!(snr > 20.0, "mu-law SNR too low: {}", snr);
    }

    #[test]
    fn test_vocoder_cvsd_encode_decode() {
        let mut voc = Vocoder::new(VocoderMode::Cvsd, 8000);
        let tone = generate_test_tone(300.0, 8000, 80);
        let encoded = voc.encode(&tone);
        assert_eq!(encoded.len(), 10); // 80 bits = 10 bytes
    }

    #[test]
    fn test_vocoder_cvsd_roundtrip() {
        let mut enc = Vocoder::new(VocoderMode::Cvsd, 16000);
        let mut dec = Vocoder::new(VocoderMode::Cvsd, 16000);
        let tone = generate_test_tone(200.0, 16000, 160);
        let data = enc.encode(&tone);
        let out = dec.decode(&data);
        assert_eq!(out.len(), data.len() * 8);
    }

    #[test]
    fn test_lpc_analysis() {
        let voc = Vocoder::new(VocoderMode::Lpc10, 8000);
        let tone = generate_test_tone(200.0, 8000, 160);
        let frame = voc.analyze_lpc(&tone);
        assert_eq!(frame.coeffs.len(), 10);
        assert!(frame.energy > -100.0);
    }

    #[test]
    fn test_lpc_encode_decode() {
        let mut voc = Vocoder::new(VocoderMode::Lpc10, 8000);
        let tone = generate_test_tone(300.0, 8000, 160);
        let encoded = voc.encode(&tone);
        // 2 bytes pitch + 2 bytes energy + 10*2 bytes coeffs = 24 bytes
        assert_eq!(encoded.len(), 24);
        let decoded = voc.decode(&encoded);
        assert_eq!(decoded.len(), 160);
    }

    #[test]
    fn test_vocoder_mode_bitrate() {
        assert_eq!(VocoderMode::Lpc10.bitrate(), 2400);
        assert_eq!(VocoderMode::Codec2_1600.bitrate(), 1600);
        assert_eq!(VocoderMode::Codec2_3200.bitrate(), 3200);
        assert_eq!(VocoderMode::Cvsd.bitrate(), 16000);
        assert_eq!(VocoderMode::MuLaw.bitrate(), 64000);
    }

    #[test]
    fn test_frame_sizes() {
        assert_eq!(VocoderMode::Lpc10.frame_size(8000), 160);
        assert_eq!(VocoderMode::Codec2_1600.frame_size(8000), 320);
        assert_eq!(VocoderMode::Codec2_3200.frame_size(8000), 160);
    }

    #[test]
    fn test_levinson_durbin() {
        // Simple test: autocorrelation of sine should give valid coeffs
        let r = vec![1.0, 0.5, 0.2, 0.1, 0.05];
        let coeffs = levinson_durbin(&r, 4);
        assert_eq!(coeffs.len(), 4);
        // Coefficients should be bounded
        for &c in &coeffs {
            assert!(c.abs() <= 1.0 + 1e-6, "LPC coeff out of range: {}", c);
        }
    }

    #[test]
    fn test_speech_quality() {
        let original = vec![1.0, 0.0, -1.0, 0.0];
        let perfect = original.clone();
        let snr = speech_quality(&original, &perfect);
        assert!(snr > 90.0);
    }

    #[test]
    fn test_generate_test_tone() {
        let tone = generate_test_tone(1000.0, 8000, 80);
        assert_eq!(tone.len(), 80);
        // Peak should be ~0.5
        let peak = tone.iter().map(|s| s.abs()).fold(0.0f32, f32::max);
        assert!((peak - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_vocoder_reset() {
        let mut voc = Vocoder::new(VocoderMode::Cvsd, 8000);
        let tone = generate_test_tone(440.0, 8000, 80);
        let _ = voc.encode(&tone);
        voc.reset();
        assert_eq!(voc.cvsd_integrator, 0.0);
    }
}
