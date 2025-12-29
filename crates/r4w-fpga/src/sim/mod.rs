//! Simulated FPGA backend for development and testing
//!
//! This module provides a software simulation of FPGA operations,
//! allowing development and testing without actual hardware.

use std::collections::HashMap;
use std::f64::consts::PI;

use crate::error::{FpgaError, FpgaResult};
use crate::traits::FpgaAccelerator;
use crate::types::{
    FpgaCapabilities, FpgaInfo, FpgaPlatform, IpCore, IpCoreType, IQSample, StreamConfig,
    StreamHandle, StreamStats,
};

/// Simulated FPGA for development without hardware
///
/// This provides a software implementation of all FPGA operations,
/// useful for:
/// - Development without hardware access
/// - Unit testing
/// - Algorithm verification
/// - Cross-platform development
pub struct SimulatedFpga {
    /// Simulated register file
    registers: HashMap<usize, u32>,

    /// Active streams
    streams: HashMap<StreamHandle, SimStream>,

    /// Next stream handle ID
    next_stream_id: u32,

    /// Waveform ID mapping
    waveform_ids: HashMap<String, u32>,

    /// Simulated capabilities
    capabilities: FpgaCapabilities,
}

struct SimStream {
    #[allow(dead_code)]
    config: StreamConfig,
    stats: StreamStats,
    tx_buffer: Vec<IQSample>,
    #[allow(dead_code)]
    rx_buffer: Vec<IQSample>,
}

impl SimulatedFpga {
    /// Create a new simulated FPGA
    pub fn new() -> Self {
        let mut waveform_ids = HashMap::new();
        waveform_ids.insert("BPSK".to_string(), 1);
        waveform_ids.insert("QPSK".to_string(), 2);
        waveform_ids.insert("8-PSK".to_string(), 3);
        waveform_ids.insert("16-QAM".to_string(), 4);
        waveform_ids.insert("LoRa".to_string(), 5);

        Self {
            registers: HashMap::new(),
            streams: HashMap::new(),
            next_stream_id: 1,
            waveform_ids,
            capabilities: FpgaCapabilities {
                max_fft_size: 4096,
                max_fir_taps: 256,
                supported_waveforms: vec![
                    "BPSK".to_string(),
                    "QPSK".to_string(),
                    "8-PSK".to_string(),
                    "16-QAM".to_string(),
                    "LoRa".to_string(),
                ],
                dma_buffer_size: 256 * 1024,
                clock_frequency_hz: 100_000_000,
                dsp_blocks: 128, // Simulated
                logic_cells: 100_000, // Simulated
                bram_bytes: 512 * 1024, // Simulated
                ip_cores: vec![
                    IpCore {
                        name: "sim_fft_4096".to_string(),
                        core_type: IpCoreType::Fft { size: 4096 },
                        version: "1.0.0".to_string(),
                        base_address: 0x4000_0000,
                        address_size: 0x1000,
                        available: true,
                    },
                    IpCore {
                        name: "sim_fir_256".to_string(),
                        core_type: IpCoreType::Fir { max_taps: 256 },
                        version: "1.0.0".to_string(),
                        base_address: 0x4001_0000,
                        address_size: 0x1000,
                        available: true,
                    },
                    IpCore {
                        name: "sim_chirp_gen".to_string(),
                        core_type: IpCoreType::ChirpGenerator,
                        version: "1.0.0".to_string(),
                        base_address: 0x4002_0000,
                        address_size: 0x1000,
                        available: true,
                    },
                    IpCore {
                        name: "sim_chirp_corr".to_string(),
                        core_type: IpCoreType::ChirpCorrelator,
                        version: "1.0.0".to_string(),
                        base_address: 0x4003_0000,
                        address_size: 0x1000,
                        available: true,
                    },
                ],
                dma_channels: 2,
                supports_streaming: true,
                supports_interrupts: true,
            },
        }
    }

    /// Create with custom capabilities
    pub fn with_capabilities(capabilities: FpgaCapabilities) -> Self {
        let mut sim = Self::new();
        sim.capabilities = capabilities;
        sim
    }

    /// Software FFT implementation (Cooley-Tukey radix-2)
    fn software_fft(&self, samples: &[IQSample], inverse: bool) -> Vec<IQSample> {
        let n = samples.len();
        if n <= 1 {
            return samples.to_vec();
        }

        // Ensure power of 2
        assert!(n.is_power_of_two(), "FFT size must be power of 2");

        // Bit-reversal permutation
        let mut result: Vec<IQSample> = samples.to_vec();
        let mut j = 0;
        for i in 0..n {
            if i < j {
                result.swap(i, j);
            }
            let mut m = n >> 1;
            while m >= 1 && j >= m {
                j -= m;
                m >>= 1;
            }
            j += m;
        }

        // Cooley-Tukey iterative FFT
        let mut len = 2;
        while len <= n {
            let half = len / 2;
            let angle_mult = if inverse {
                2.0 * PI / len as f64
            } else {
                -2.0 * PI / len as f64
            };

            for i in (0..n).step_by(len) {
                for j in 0..half {
                    let angle = angle_mult * j as f64;
                    let twiddle = IQSample::new(angle.cos(), angle.sin());

                    let a = result[i + j];
                    let b_re = result[i + j + half].re * twiddle.re - result[i + j + half].im * twiddle.im;
                    let b_im = result[i + j + half].re * twiddle.im + result[i + j + half].im * twiddle.re;

                    result[i + j] = IQSample::new(a.re + b_re, a.im + b_im);
                    result[i + j + half] = IQSample::new(a.re - b_re, a.im - b_im);
                }
            }
            len *= 2;
        }

        // Normalize for inverse FFT
        if inverse {
            let scale = 1.0 / n as f64;
            for sample in &mut result {
                *sample = IQSample::new(sample.re * scale, sample.im * scale);
            }
        }

        result
    }

    /// Software FIR filter implementation
    fn software_fir(&self, samples: &[IQSample], taps: &[f32]) -> Vec<IQSample> {
        let mut result = Vec::with_capacity(samples.len());

        for i in 0..samples.len() {
            let mut acc_re = 0.0f64;
            let mut acc_im = 0.0f64;

            for (j, &tap) in taps.iter().enumerate() {
                if i >= j {
                    acc_re += samples[i - j].re * tap as f64;
                    acc_im += samples[i - j].im * tap as f64;
                }
            }

            result.push(IQSample::new(acc_re, acc_im));
        }

        result
    }

    /// Generate chirp signal
    fn generate_chirp_signal(&self, sf: u8, upchirp: bool) -> Vec<IQSample> {
        let n = 1usize << sf; // 2^sf samples
        let mut samples = Vec::with_capacity(n);

        for i in 0..n {
            let t = i as f64 / n as f64;
            // Chirp phase: for upchirp, frequency increases linearly
            // Phase = 2*pi * (f0*t + 0.5*chirp_rate*t^2)
            // For LoRa, we use normalized frequency
            let phase = if upchirp {
                2.0 * PI * (0.5 * t * t * n as f64)
            } else {
                2.0 * PI * (t - 0.5 * t * t) * n as f64
            };

            samples.push(IQSample::new(phase.cos(), phase.sin()));
        }

        samples
    }
}

impl Default for SimulatedFpga {
    fn default() -> Self {
        Self::new()
    }
}

impl FpgaAccelerator for SimulatedFpga {
    fn info(&self) -> FpgaInfo {
        FpgaInfo {
            platform: FpgaPlatform::Simulated,
            device: "R4W Simulated FPGA".to_string(),
            bitstream_version: Some("1.0.0-sim".to_string()),
            bitstream_name: Some("r4w_sim".to_string()),
            bitstream_timestamp: None,
            driver_version: Some(env!("CARGO_PKG_VERSION").to_string()),
        }
    }

    fn is_available(&self) -> bool {
        true // Simulation is always available
    }

    fn capabilities(&self) -> FpgaCapabilities {
        self.capabilities.clone()
    }

    fn fft(&self, samples: &[IQSample], inverse: bool) -> FpgaResult<Vec<IQSample>> {
        let n = samples.len();

        if !n.is_power_of_two() {
            return Err(FpgaError::ConfigError(format!(
                "FFT size must be power of 2, got {}",
                n
            )));
        }

        if n > self.capabilities.max_fft_size {
            return Err(FpgaError::ConfigError(format!(
                "FFT size {} exceeds maximum {}",
                n, self.capabilities.max_fft_size
            )));
        }

        Ok(self.software_fft(samples, inverse))
    }

    fn fir_filter(&self, samples: &[IQSample], taps: &[f32]) -> FpgaResult<Vec<IQSample>> {
        if taps.len() > self.capabilities.max_fir_taps {
            return Err(FpgaError::ConfigError(format!(
                "FIR tap count {} exceeds maximum {}",
                taps.len(),
                self.capabilities.max_fir_taps
            )));
        }

        Ok(self.software_fir(samples, taps))
    }

    fn complex_multiply(&self, a: &[IQSample], b: &[IQSample]) -> FpgaResult<Vec<IQSample>> {
        if a.len() != b.len() {
            return Err(FpgaError::BufferSizeMismatch {
                expected: a.len(),
                actual: b.len(),
            });
        }

        let result: Vec<IQSample> = a
            .iter()
            .zip(b.iter())
            .map(|(x, y)| {
                // Complex multiplication: (a + bi)(c + di) = (ac - bd) + (ad + bc)i
                IQSample::new(x.re * y.re - x.im * y.im, x.re * y.im + x.im * y.re)
            })
            .collect();

        Ok(result)
    }

    fn modulate(&self, waveform_id: u32, bits: &[bool]) -> FpgaResult<Vec<IQSample>> {
        // Simple simulation: map bits to constellation points
        match waveform_id {
            1 => {
                // BPSK
                Ok(bits
                    .iter()
                    .map(|&b| {
                        if b {
                            IQSample::new(1.0, 0.0)
                        } else {
                            IQSample::new(-1.0, 0.0)
                        }
                    })
                    .collect())
            }
            2 => {
                // QPSK
                let mut samples = Vec::with_capacity(bits.len() / 2);
                for chunk in bits.chunks(2) {
                    if chunk.len() == 2 {
                        let re = if chunk[0] { 0.707 } else { -0.707 };
                        let im = if chunk[1] { 0.707 } else { -0.707 };
                        samples.push(IQSample::new(re, im));
                    }
                }
                Ok(samples)
            }
            5 => {
                // LoRa - simplified, uses chirps
                // In real implementation, this would be much more complex
                Err(FpgaError::NotSupported(
                    "LoRa modulation requires chirp generation".to_string(),
                ))
            }
            _ => Err(FpgaError::NotSupported(format!(
                "Waveform ID {} not supported",
                waveform_id
            ))),
        }
    }

    fn demodulate(&self, waveform_id: u32, samples: &[IQSample]) -> FpgaResult<Vec<bool>> {
        match waveform_id {
            1 => {
                // BPSK
                Ok(samples.iter().map(|s| s.re > 0.0).collect())
            }
            2 => {
                // QPSK
                let mut bits = Vec::with_capacity(samples.len() * 2);
                for s in samples {
                    bits.push(s.re > 0.0);
                    bits.push(s.im > 0.0);
                }
                Ok(bits)
            }
            _ => Err(FpgaError::NotSupported(format!(
                "Waveform ID {} not supported",
                waveform_id
            ))),
        }
    }

    fn waveform_id(&self, name: &str) -> Option<u32> {
        self.waveform_ids.get(name).copied()
    }

    fn generate_chirp(&self, sf: u8, upchirp: bool) -> FpgaResult<Vec<IQSample>> {
        if !(5..=12).contains(&sf) {
            return Err(FpgaError::ConfigError(format!(
                "Spreading factor must be 5-12, got {}",
                sf
            )));
        }

        Ok(self.generate_chirp_signal(sf, upchirp))
    }

    fn chirp_correlate(&self, samples: &[IQSample], sf: u8) -> FpgaResult<(u32, f32)> {
        if !(5..=12).contains(&sf) {
            return Err(FpgaError::ConfigError(format!(
                "Spreading factor must be 5-12, got {}",
                sf
            )));
        }

        let n = 1usize << sf;
        if samples.len() < n {
            return Err(FpgaError::BufferSizeMismatch {
                expected: n,
                actual: samples.len(),
            });
        }

        // Generate downchirp for correlation
        let downchirp = self.generate_chirp_signal(sf, false);

        // Multiply by downchirp
        let dechirped: Vec<IQSample> = samples[..n]
            .iter()
            .zip(downchirp.iter())
            .map(|(s, d)| IQSample::new(s.re * d.re - s.im * d.im, s.re * d.im + s.im * d.re))
            .collect();

        // FFT
        let spectrum = self.software_fft(&dechirped, false);

        // Find peak
        let mut max_mag = 0.0f64;
        let mut max_idx = 0u32;

        for (i, s) in spectrum.iter().enumerate() {
            let mag = (s.re * s.re + s.im * s.im).sqrt();
            if mag > max_mag {
                max_mag = mag;
                max_idx = i as u32;
            }
        }

        Ok((max_idx, max_mag as f32))
    }

    fn start_stream(&mut self, config: StreamConfig) -> FpgaResult<StreamHandle> {
        let handle = StreamHandle(self.next_stream_id);
        self.next_stream_id += 1;

        self.streams.insert(
            handle,
            SimStream {
                config,
                stats: StreamStats::default(),
                tx_buffer: Vec::new(),
                rx_buffer: Vec::new(),
            },
        );

        Ok(handle)
    }

    fn stop_stream(&mut self, handle: StreamHandle) -> FpgaResult<()> {
        self.streams
            .remove(&handle)
            .ok_or_else(|| FpgaError::StreamError(format!("Stream {:?} not found", handle)))?;
        Ok(())
    }

    fn write_stream(&mut self, handle: StreamHandle, samples: &[IQSample]) -> FpgaResult<usize> {
        let stream = self
            .streams
            .get_mut(&handle)
            .ok_or_else(|| FpgaError::StreamError(format!("Stream {:?} not found", handle)))?;

        stream.tx_buffer.extend_from_slice(samples);
        stream.stats.samples_transferred += samples.len() as u64;

        Ok(samples.len())
    }

    fn read_stream(&mut self, handle: StreamHandle, buffer: &mut [IQSample]) -> FpgaResult<usize> {
        let stream = self
            .streams
            .get_mut(&handle)
            .ok_or_else(|| FpgaError::StreamError(format!("Stream {:?} not found", handle)))?;

        // In simulation, we just return what was written (loopback)
        let available = stream.tx_buffer.len().min(buffer.len());

        for (i, sample) in stream.tx_buffer.drain(..available).enumerate() {
            buffer[i] = sample;
        }

        stream.stats.samples_transferred += available as u64;

        Ok(available)
    }

    fn stream_stats(&self, handle: StreamHandle) -> FpgaResult<StreamStats> {
        self.streams
            .get(&handle)
            .map(|s| s.stats.clone())
            .ok_or_else(|| FpgaError::StreamError(format!("Stream {:?} not found", handle)))
    }

    fn read_register(&self, address: usize) -> FpgaResult<u32> {
        Ok(*self.registers.get(&address).unwrap_or(&0))
    }

    fn write_register(&mut self, address: usize, value: u32) -> FpgaResult<()> {
        self.registers.insert(address, value);
        Ok(())
    }

    fn reset(&mut self) -> FpgaResult<()> {
        self.registers.clear();
        self.streams.clear();
        self.next_stream_id = 1;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulated_fpga_available() {
        let fpga = SimulatedFpga::new();
        assert!(fpga.is_available());
    }

    #[test]
    fn test_fft_roundtrip() {
        let fpga = SimulatedFpga::new();

        // Create test signal
        let samples: Vec<IQSample> = (0..64)
            .map(|i| {
                let phase = 2.0 * PI * (i as f64) / 64.0;
                IQSample::new(phase.cos(), phase.sin())
            })
            .collect();

        // Forward FFT
        let spectrum = fpga.fft(&samples, false).unwrap();
        assert_eq!(spectrum.len(), 64);

        // Inverse FFT
        let recovered = fpga.fft(&spectrum, true).unwrap();
        assert_eq!(recovered.len(), 64);

        // Check roundtrip (with tolerance for floating point)
        for (orig, rec) in samples.iter().zip(recovered.iter()) {
            assert!((orig.re - rec.re).abs() < 1e-5);
            assert!((orig.im - rec.im).abs() < 1e-5);
        }
    }

    #[test]
    fn test_complex_multiply() {
        let fpga = SimulatedFpga::new();

        let a = vec![IQSample::new(1.0, 2.0), IQSample::new(3.0, 4.0)];
        let b = vec![IQSample::new(5.0, 6.0), IQSample::new(7.0, 8.0)];

        let result = fpga.complex_multiply(&a, &b).unwrap();

        // (1+2i)(5+6i) = 5 + 6i + 10i + 12i² = 5 + 16i - 12 = -7 + 16i
        assert!((result[0].re - (-7.0)).abs() < 1e-5);
        assert!((result[0].im - 16.0).abs() < 1e-5);

        // (3+4i)(7+8i) = 21 + 24i + 28i + 32i² = 21 + 52i - 32 = -11 + 52i
        assert!((result[1].re - (-11.0)).abs() < 1e-5);
        assert!((result[1].im - 52.0).abs() < 1e-5);
    }

    #[test]
    fn test_bpsk_modulation() {
        let fpga = SimulatedFpga::new();

        let bits = vec![true, false, true, true, false];
        let waveform_id = fpga.waveform_id("BPSK").unwrap();

        let modulated = fpga.modulate(waveform_id, &bits).unwrap();
        assert_eq!(modulated.len(), 5);

        // BPSK: true -> (1, 0), false -> (-1, 0)
        assert!(modulated[0].re > 0.0);
        assert!(modulated[1].re < 0.0);
        assert!(modulated[2].re > 0.0);
        assert!(modulated[3].re > 0.0);
        assert!(modulated[4].re < 0.0);
    }

    #[test]
    fn test_chirp_generation() {
        let fpga = SimulatedFpga::new();

        let chirp = fpga.generate_chirp(7, true).unwrap();
        assert_eq!(chirp.len(), 128); // 2^7 = 128

        // All samples should have unit magnitude (approximately)
        for sample in &chirp {
            let mag = (sample.re * sample.re + sample.im * sample.im).sqrt();
            assert!((mag - 1.0).abs() < 1e-5);
        }
    }

    #[test]
    fn test_stream_operations() {
        let mut fpga = SimulatedFpga::new();

        let config = StreamConfig::default();
        let handle = fpga.start_stream(config).unwrap();

        // Write some samples
        let samples = vec![IQSample::new(1.0, 0.0); 100];
        let written = fpga.write_stream(handle, &samples).unwrap();
        assert_eq!(written, 100);

        // Read them back
        let mut buffer = vec![IQSample::new(0.0, 0.0); 50];
        let read = fpga.read_stream(handle, &mut buffer).unwrap();
        assert_eq!(read, 50);

        // Check stats
        let stats = fpga.stream_stats(handle).unwrap();
        assert!(stats.samples_transferred > 0);

        // Stop stream
        fpga.stop_stream(handle).unwrap();
    }

    #[test]
    fn test_register_operations() {
        let mut fpga = SimulatedFpga::new();

        // Write and read back
        fpga.write_register(0x1000, 0xDEADBEEF).unwrap();
        let value = fpga.read_register(0x1000).unwrap();
        assert_eq!(value, 0xDEADBEEF);

        // Unwritten register should be 0
        let value = fpga.read_register(0x2000).unwrap();
        assert_eq!(value, 0);
    }
}
