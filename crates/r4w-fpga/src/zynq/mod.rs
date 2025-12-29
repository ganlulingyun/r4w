//! Xilinx Zynq FPGA backend
//!
//! This module provides hardware acceleration on Xilinx Zynq-7000 and
//! Zynq UltraScale+ platforms using memory-mapped I/O and UIO drivers.
//!
//! # Requirements
//!
//! - Linux with `/dev/mem` or `/dev/uio*` access
//! - Root privileges or appropriate group membership
//! - Compatible bitstream loaded to PL (Programmable Logic)
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────┐
//! │                    Zynq SoC                             │
//! │  ┌────────────────┐        ┌──────────────────────────┐ │
//! │  │   ARM CPU      │◄──────►│   FPGA Fabric (PL)       │ │
//! │  │  (PS - Linux)  │  AXI   │   ┌─────┐ ┌─────┐        │ │
//! │  │                │  Bus   │   │ FFT │ │ FIR │ ...    │ │
//! │  └───────┬────────┘        │   └─────┘ └─────┘        │ │
//! │          │                 └──────────────────────────┘ │
//! │          ▼                                              │
//! │  ┌─────────────┐                                        │
//! │  │  /dev/mem   │  Memory-mapped registers               │
//! │  │  /dev/uio*  │  Userspace I/O (interrupts)            │
//! │  └─────────────┘                                        │
//! └─────────────────────────────────────────────────────────┘
//! ```

mod config;
mod dma;
mod mmap;
mod registers;
mod uio;

pub use config::ZynqConfig;

use std::fs::File;
use std::io::{self, Read};
use std::sync::{Arc, Mutex};

use crate::error::{FpgaError, FpgaResult};
use crate::traits::FpgaAccelerator;
use crate::types::{
    FpgaCapabilities, FpgaInfo, FpgaPlatform, IpCore, IpCoreType, IQSample, StreamConfig,
    StreamHandle, StreamStats,
};

use mmap::MemoryRegion;

/// Zynq FPGA accelerator implementation
pub struct ZynqFpga {
    /// Platform variant
    platform: FpgaPlatform,

    /// Memory-mapped register regions
    regions: Vec<MemoryRegion>,

    /// Device capabilities (detected from bitstream)
    capabilities: FpgaCapabilities,

    /// Configuration
    config: ZynqConfig,

    /// DMA controller (if available)
    dma: Option<Arc<Mutex<dma::DmaController>>>,

    /// UIO devices for interrupts
    uio_devices: Vec<uio::UioDevice>,

    /// Active streams
    streams: std::collections::HashMap<StreamHandle, ZynqStream>,

    /// Next stream handle
    next_stream_id: u32,
}

struct ZynqStream {
    config: StreamConfig,
    stats: StreamStats,
    dma_channel: Option<usize>,
}

impl ZynqFpga {
    /// Auto-detect Zynq platform and create accelerator
    pub fn auto_detect() -> FpgaResult<Self> {
        // Check if we're on a Zynq platform
        if !Self::is_platform_available() {
            return Err(FpgaError::PlatformNotSupported);
        }

        let config = ZynqConfig::auto_detect()?;
        Self::new(config)
    }

    /// Create with specific configuration
    pub fn new(config: ZynqConfig) -> FpgaResult<Self> {
        // Detect platform type
        let platform = Self::detect_platform()?;

        // Map register regions
        let regions = Self::map_regions(&config)?;

        // Detect capabilities from mapped regions
        let capabilities = Self::detect_capabilities(&regions, &config)?;

        // Initialize DMA if available
        let dma = if config.enable_dma {
            match dma::DmaController::new(&config) {
                Ok(controller) => Some(Arc::new(Mutex::new(controller))),
                Err(e) => {
                    tracing::warn!("DMA initialization failed: {}", e);
                    None
                }
            }
        } else {
            None
        };

        // Open UIO devices for interrupts
        let uio_devices = if config.enable_interrupts {
            uio::discover_uio_devices()?
        } else {
            vec![]
        };

        Ok(Self {
            platform,
            regions,
            capabilities,
            config,
            dma,
            uio_devices,
            streams: std::collections::HashMap::new(),
            next_stream_id: 1,
        })
    }

    /// Check if Zynq platform is available
    pub fn is_platform_available() -> bool {
        // Check for device tree compatible string
        if let Ok(mut file) = File::open("/sys/firmware/devicetree/base/compatible") {
            let mut contents = String::new();
            if file.read_to_string(&mut contents).is_ok() {
                return contents.contains("xlnx,zynq") || contents.contains("xlnx,zynqmp");
            }
        }

        // Fallback: check for Zynq-specific devices
        std::path::Path::new("/sys/class/fpga_manager").exists()
    }

    /// Detect the specific Zynq platform variant
    fn detect_platform() -> FpgaResult<FpgaPlatform> {
        if let Ok(mut file) = File::open("/sys/firmware/devicetree/base/compatible") {
            let mut contents = String::new();
            if file.read_to_string(&mut contents).is_ok() {
                if contents.contains("xlnx,zynqmp") {
                    // Extract part number if available
                    let part = Self::read_part_number().unwrap_or_else(|| "unknown".to_string());
                    return Ok(FpgaPlatform::XilinxZynqUltraScale { part });
                } else if contents.contains("xlnx,zynq") {
                    let part = Self::read_part_number().unwrap_or_else(|| "xc7z020".to_string());
                    return Ok(FpgaPlatform::XilinxZynq7000 { part });
                }
            }
        }

        Err(FpgaError::DeviceNotFound(
            "Could not detect Zynq platform".to_string(),
        ))
    }

    fn read_part_number() -> Option<String> {
        // Try to read from device tree or FPGA manager
        let paths = [
            "/sys/firmware/devicetree/base/model",
            "/sys/class/fpga_manager/fpga0/name",
        ];

        for path in paths {
            if let Ok(mut file) = File::open(path) {
                let mut contents = String::new();
                if file.read_to_string(&mut contents).is_ok() {
                    return Some(contents.trim().to_string());
                }
            }
        }

        None
    }

    fn map_regions(config: &ZynqConfig) -> FpgaResult<Vec<MemoryRegion>> {
        let mut regions = Vec::new();

        for (name, base, size) in &config.register_regions {
            match MemoryRegion::new(name.clone(), *base, *size) {
                Ok(region) => {
                    tracing::info!("Mapped region {} at 0x{:08x}", name, base);
                    regions.push(region);
                }
                Err(e) => {
                    tracing::warn!("Failed to map region {}: {}", name, e);
                    // Continue with other regions
                }
            }
        }

        if regions.is_empty() {
            return Err(FpgaError::MmapFailed {
                address: 0,
                reason: "No register regions could be mapped".to_string(),
            });
        }

        Ok(regions)
    }

    fn detect_capabilities(
        regions: &[MemoryRegion],
        config: &ZynqConfig,
    ) -> FpgaResult<FpgaCapabilities> {
        // Try to read capabilities from IP cores
        let mut ip_cores = Vec::new();

        // Check for R4W FFT core
        if let Some(fft_info) = Self::probe_fft_core(regions, config) {
            ip_cores.push(fft_info);
        }

        // Check for R4W FIR core
        if let Some(fir_info) = Self::probe_fir_core(regions, config) {
            ip_cores.push(fir_info);
        }

        // Check for chirp generator
        if let Some(chirp_info) = Self::probe_chirp_core(regions, config) {
            ip_cores.push(chirp_info);
        }

        // Build capabilities based on detected cores
        let max_fft_size = ip_cores
            .iter()
            .filter_map(|c| {
                if let IpCoreType::Fft { size } = c.core_type {
                    Some(size)
                } else {
                    None
                }
            })
            .max()
            .unwrap_or(1024);

        let max_fir_taps = ip_cores
            .iter()
            .filter_map(|c| {
                if let IpCoreType::Fir { max_taps } = c.core_type {
                    Some(max_taps)
                } else {
                    None
                }
            })
            .max()
            .unwrap_or(64);

        Ok(FpgaCapabilities {
            max_fft_size,
            max_fir_taps,
            supported_waveforms: vec!["LoRa".to_string()], // Based on IP cores
            dma_buffer_size: config.dma_buffer_size,
            clock_frequency_hz: config.fabric_clock_hz,
            dsp_blocks: 220, // Typical for Zynq-7020
            logic_cells: 85_000,
            bram_bytes: 560 * 1024,
            ip_cores,
            dma_channels: if config.enable_dma { 2 } else { 0 },
            supports_streaming: config.enable_dma,
            supports_interrupts: config.enable_interrupts,
        })
    }

    fn probe_fft_core(regions: &[MemoryRegion], config: &ZynqConfig) -> Option<IpCore> {
        // Look for FFT core at expected address
        let fft_base = config.fft_base_addr?;

        for region in regions {
            if region.contains(fft_base) {
                // Try to read version register
                if let Ok(version) = region.read32(fft_base + 0x00) {
                    if version != 0 && version != 0xFFFFFFFF {
                        // Read size register
                        let size = region.read32(fft_base + 0x04).unwrap_or(1024) as usize;

                        return Some(IpCore {
                            name: "r4w_fft".to_string(),
                            core_type: IpCoreType::Fft { size },
                            version: format!("{}.{}.{}", version >> 16, (version >> 8) & 0xFF, version & 0xFF),
                            base_address: fft_base,
                            address_size: 0x1000,
                            available: true,
                        });
                    }
                }
            }
        }

        None
    }

    fn probe_fir_core(regions: &[MemoryRegion], config: &ZynqConfig) -> Option<IpCore> {
        let fir_base = config.fir_base_addr?;

        for region in regions {
            if region.contains(fir_base) {
                if let Ok(version) = region.read32(fir_base + 0x00) {
                    if version != 0 && version != 0xFFFFFFFF {
                        let max_taps = region.read32(fir_base + 0x04).unwrap_or(64) as usize;

                        return Some(IpCore {
                            name: "r4w_fir".to_string(),
                            core_type: IpCoreType::Fir { max_taps },
                            version: format!("{}.{}.{}", version >> 16, (version >> 8) & 0xFF, version & 0xFF),
                            base_address: fir_base,
                            address_size: 0x1000,
                            available: true,
                        });
                    }
                }
            }
        }

        None
    }

    fn probe_chirp_core(regions: &[MemoryRegion], config: &ZynqConfig) -> Option<IpCore> {
        let chirp_base = config.chirp_base_addr?;

        for region in regions {
            if region.contains(chirp_base) {
                if let Ok(version) = region.read32(chirp_base + 0x00) {
                    if version != 0 && version != 0xFFFFFFFF {
                        return Some(IpCore {
                            name: "r4w_chirp".to_string(),
                            core_type: IpCoreType::ChirpGenerator,
                            version: format!("{}.{}.{}", version >> 16, (version >> 8) & 0xFF, version & 0xFF),
                            base_address: chirp_base,
                            address_size: 0x1000,
                            available: true,
                        });
                    }
                }
            }
        }

        None
    }

    /// Read a 32-bit register at the given address
    fn read_reg(&self, address: usize) -> FpgaResult<u32> {
        for region in &self.regions {
            if region.contains(address) {
                return region
                    .read32(address)
                    .map_err(|e| FpgaError::OpenFailed(e));
            }
        }
        Err(FpgaError::InvalidAddress(address))
    }

    /// Write a 32-bit register at the given address
    fn write_reg(&self, address: usize, value: u32) -> FpgaResult<()> {
        for region in &self.regions {
            if region.contains(address) {
                return region
                    .write32(address, value)
                    .map_err(|e| FpgaError::OpenFailed(e));
            }
        }
        Err(FpgaError::InvalidAddress(address))
    }
}

impl FpgaAccelerator for ZynqFpga {
    fn info(&self) -> FpgaInfo {
        let (bitstream_version, bitstream_name) = Self::read_bitstream_info();

        FpgaInfo {
            platform: self.platform.clone(),
            device: self.platform.name().to_string(),
            bitstream_version,
            bitstream_name,
            bitstream_timestamp: None,
            driver_version: Some(env!("CARGO_PKG_VERSION").to_string()),
        }
    }

    fn is_available(&self) -> bool {
        !self.regions.is_empty()
    }

    fn capabilities(&self) -> FpgaCapabilities {
        self.capabilities.clone()
    }

    fn fft(&self, samples: &[IQSample], inverse: bool) -> FpgaResult<Vec<IQSample>> {
        let n = samples.len();

        // Validate size
        if !n.is_power_of_two() {
            return Err(FpgaError::ConfigError(
                "FFT size must be power of 2".to_string(),
            ));
        }
        if n > self.capabilities.max_fft_size {
            return Err(FpgaError::ConfigError(format!(
                "FFT size {} exceeds hardware maximum {}",
                n, self.capabilities.max_fft_size
            )));
        }

        // Find FFT core
        let fft_core = self
            .capabilities
            .ip_cores
            .iter()
            .find(|c| matches!(c.core_type, IpCoreType::Fft { .. }))
            .ok_or_else(|| FpgaError::IpCoreNotAvailable("FFT".to_string()))?;

        let base = fft_core.base_address;

        // Configure FFT
        // Register layout (example):
        // 0x00: Control (bit 0: start, bit 1: inverse)
        // 0x04: Size (log2)
        // 0x08: Status (bit 0: done, bit 1: error)
        // 0x10: Input data (write samples here)
        // 0x14: Output data (read results here)

        let log2_n = n.trailing_zeros();
        self.write_reg(base + 0x04, log2_n)?;

        let control = if inverse { 0x02 } else { 0x00 };

        // If DMA is available, use it for bulk transfer
        if let Some(ref dma) = self.dma {
            let mut dma_lock = dma.lock().map_err(|_| {
                FpgaError::DmaError("Failed to acquire DMA lock".to_string())
            })?;

            // Transfer samples to FPGA
            dma_lock.write_samples(samples)?;

            // Start FFT
            self.write_reg(base + 0x00, control | 0x01)?;

            // Wait for completion
            self.wait_for_done(base + 0x08, 1000)?;

            // Read results
            let result = dma_lock.read_samples(n)?;
            Ok(result)
        } else {
            // Fallback: register-based transfer (slower)
            let mut result = Vec::with_capacity(n);

            // Write samples via registers
            for sample in samples {
                // Pack I/Q as two 16-bit values
                let i_fixed = (sample.i * 32767.0) as i16;
                let q_fixed = (sample.q * 32767.0) as i16;
                let packed = ((i_fixed as u32) << 16) | (q_fixed as u16 as u32);
                self.write_reg(base + 0x10, packed)?;
            }

            // Start FFT
            self.write_reg(base + 0x00, control | 0x01)?;

            // Wait for completion
            self.wait_for_done(base + 0x08, 1000)?;

            // Read results
            for _ in 0..n {
                let packed = self.read_reg(base + 0x14)?;
                let i_fixed = (packed >> 16) as i16;
                let q_fixed = packed as i16;
                result.push(IQSample::new(
                    i_fixed as f32 / 32767.0,
                    q_fixed as f32 / 32767.0,
                ));
            }

            Ok(result)
        }
    }

    fn fir_filter(&self, samples: &[IQSample], taps: &[f32]) -> FpgaResult<Vec<IQSample>> {
        if taps.len() > self.capabilities.max_fir_taps {
            return Err(FpgaError::ConfigError(format!(
                "FIR tap count {} exceeds hardware maximum {}",
                taps.len(),
                self.capabilities.max_fir_taps
            )));
        }

        let fir_core = self
            .capabilities
            .ip_cores
            .iter()
            .find(|c| matches!(c.core_type, IpCoreType::Fir { .. }))
            .ok_or_else(|| FpgaError::IpCoreNotAvailable("FIR".to_string()))?;

        let base = fir_core.base_address;

        // Load filter taps
        for (i, &tap) in taps.iter().enumerate() {
            let tap_fixed = (tap * 32767.0) as i32;
            self.write_reg(base + 0x100 + i * 4, tap_fixed as u32)?;
        }

        // Set number of taps
        self.write_reg(base + 0x04, taps.len() as u32)?;

        // Process samples (similar to FFT)
        let mut result = Vec::with_capacity(samples.len());

        for sample in samples {
            let i_fixed = (sample.i * 32767.0) as i16;
            let q_fixed = (sample.q * 32767.0) as i16;
            let packed = ((i_fixed as u32) << 16) | (q_fixed as u16 as u32);
            self.write_reg(base + 0x10, packed)?;

            // Read filtered output
            let out_packed = self.read_reg(base + 0x14)?;
            let out_i = (out_packed >> 16) as i16;
            let out_q = out_packed as i16;
            result.push(IQSample::new(
                out_i as f32 / 32767.0,
                out_q as f32 / 32767.0,
            ));
        }

        Ok(result)
    }

    fn complex_multiply(&self, a: &[IQSample], b: &[IQSample]) -> FpgaResult<Vec<IQSample>> {
        if a.len() != b.len() {
            return Err(FpgaError::BufferSizeMismatch {
                expected: a.len(),
                actual: b.len(),
            });
        }

        // Use software fallback if no dedicated hardware
        // Complex multiplication is typically done in DSP slices
        let result: Vec<IQSample> = a
            .iter()
            .zip(b.iter())
            .map(|(x, y)| IQSample::new(x.i * y.i - x.q * y.q, x.i * y.q + x.q * y.i))
            .collect();

        Ok(result)
    }

    fn modulate(&self, _waveform_id: u32, _bits: &[bool]) -> FpgaResult<Vec<IQSample>> {
        // Modulation is typically handled by waveform-specific IP cores
        Err(FpgaError::NotSupported(
            "Hardware modulation not implemented".to_string(),
        ))
    }

    fn demodulate(&self, _waveform_id: u32, _samples: &[IQSample]) -> FpgaResult<Vec<bool>> {
        Err(FpgaError::NotSupported(
            "Hardware demodulation not implemented".to_string(),
        ))
    }

    fn waveform_id(&self, name: &str) -> Option<u32> {
        match name {
            "LoRa" => Some(1),
            _ => None,
        }
    }

    fn generate_chirp(&self, sf: u8, upchirp: bool) -> FpgaResult<Vec<IQSample>> {
        let chirp_core = self
            .capabilities
            .ip_cores
            .iter()
            .find(|c| matches!(c.core_type, IpCoreType::ChirpGenerator))
            .ok_or_else(|| FpgaError::IpCoreNotAvailable("ChirpGenerator".to_string()))?;

        let base = chirp_core.base_address;

        // Configure chirp generator
        // 0x00: Control (bit 0: start, bit 1: upchirp)
        // 0x04: Spreading factor
        // 0x08: Status

        self.write_reg(base + 0x04, sf as u32)?;

        let control = if upchirp { 0x02 } else { 0x00 };
        self.write_reg(base + 0x00, control | 0x01)?;

        // Wait for completion
        self.wait_for_done(base + 0x08, 100)?;

        // Read chirp samples
        let n = 1usize << sf;
        let mut result = Vec::with_capacity(n);

        for _ in 0..n {
            let packed = self.read_reg(base + 0x10)?;
            let i_fixed = (packed >> 16) as i16;
            let q_fixed = packed as i16;
            result.push(IQSample::new(
                i_fixed as f32 / 32767.0,
                q_fixed as f32 / 32767.0,
            ));
        }

        Ok(result)
    }

    fn chirp_correlate(&self, samples: &[IQSample], sf: u8) -> FpgaResult<(u32, f32)> {
        // Check for correlator IP
        let correlator = self
            .capabilities
            .ip_cores
            .iter()
            .find(|c| matches!(c.core_type, IpCoreType::ChirpCorrelator));

        if let Some(core) = correlator {
            let base = core.base_address;

            // Write samples
            let n = 1usize << sf;
            for sample in samples.iter().take(n) {
                let i_fixed = (sample.i * 32767.0) as i16;
                let q_fixed = (sample.q * 32767.0) as i16;
                let packed = ((i_fixed as u32) << 16) | (q_fixed as u16 as u32);
                self.write_reg(base + 0x10, packed)?;
            }

            // Start correlation
            self.write_reg(base + 0x04, sf as u32)?;
            self.write_reg(base + 0x00, 0x01)?;

            // Wait
            self.wait_for_done(base + 0x08, 100)?;

            // Read result
            let symbol = self.read_reg(base + 0x20)?;
            let magnitude = self.read_reg(base + 0x24)?;

            Ok((symbol, f32::from_bits(magnitude)))
        } else {
            // Fall back to FFT-based correlation
            Err(FpgaError::NotSupported(
                "Chirp correlator not available, use FFT method".to_string(),
            ))
        }
    }

    fn start_stream(&mut self, config: StreamConfig) -> FpgaResult<StreamHandle> {
        let dma = self
            .dma
            .as_ref()
            .ok_or_else(|| FpgaError::StreamError("DMA not available".to_string()))?;

        let mut dma_lock = dma.lock().map_err(|_| {
            FpgaError::StreamError("Failed to acquire DMA lock".to_string())
        })?;

        let channel = dma_lock.allocate_channel(config.buffer_size)?;

        let handle = StreamHandle(self.next_stream_id);
        self.next_stream_id += 1;

        self.streams.insert(
            handle,
            ZynqStream {
                config,
                stats: StreamStats::default(),
                dma_channel: Some(channel),
            },
        );

        Ok(handle)
    }

    fn stop_stream(&mut self, handle: StreamHandle) -> FpgaResult<()> {
        let stream = self
            .streams
            .remove(&handle)
            .ok_or_else(|| FpgaError::StreamError(format!("Stream {:?} not found", handle)))?;

        if let (Some(dma), Some(channel)) = (&self.dma, stream.dma_channel) {
            let mut dma_lock = dma.lock().map_err(|_| {
                FpgaError::StreamError("Failed to acquire DMA lock".to_string())
            })?;
            dma_lock.release_channel(channel)?;
        }

        Ok(())
    }

    fn write_stream(&mut self, handle: StreamHandle, samples: &[IQSample]) -> FpgaResult<usize> {
        let dma = self
            .dma
            .as_ref()
            .ok_or_else(|| FpgaError::StreamError("DMA not available".to_string()))?;

        let stream = self
            .streams
            .get_mut(&handle)
            .ok_or_else(|| FpgaError::StreamError(format!("Stream {:?} not found", handle)))?;

        let channel = stream
            .dma_channel
            .ok_or_else(|| FpgaError::StreamError("No DMA channel".to_string()))?;

        let mut dma_lock = dma.lock().map_err(|_| {
            FpgaError::StreamError("Failed to acquire DMA lock".to_string())
        })?;

        let written = dma_lock.write_to_channel(channel, samples)?;
        stream.stats.samples_transferred += written as u64;

        Ok(written)
    }

    fn read_stream(&mut self, handle: StreamHandle, buffer: &mut [IQSample]) -> FpgaResult<usize> {
        let dma = self
            .dma
            .as_ref()
            .ok_or_else(|| FpgaError::StreamError("DMA not available".to_string()))?;

        let stream = self
            .streams
            .get_mut(&handle)
            .ok_or_else(|| FpgaError::StreamError(format!("Stream {:?} not found", handle)))?;

        let channel = stream
            .dma_channel
            .ok_or_else(|| FpgaError::StreamError("No DMA channel".to_string()))?;

        let mut dma_lock = dma.lock().map_err(|_| {
            FpgaError::StreamError("Failed to acquire DMA lock".to_string())
        })?;

        let read = dma_lock.read_from_channel(channel, buffer)?;
        stream.stats.samples_transferred += read as u64;

        Ok(read)
    }

    fn stream_stats(&self, handle: StreamHandle) -> FpgaResult<StreamStats> {
        self.streams
            .get(&handle)
            .map(|s| s.stats.clone())
            .ok_or_else(|| FpgaError::StreamError(format!("Stream {:?} not found", handle)))
    }

    fn read_register(&self, address: usize) -> FpgaResult<u32> {
        self.read_reg(address)
    }

    fn write_register(&mut self, address: usize, value: u32) -> FpgaResult<()> {
        self.write_reg(address, value)
    }

    fn reset(&mut self) -> FpgaResult<()> {
        // Stop all streams
        let handles: Vec<_> = self.streams.keys().cloned().collect();
        for handle in handles {
            let _ = self.stop_stream(handle);
        }

        // Reset IP cores
        for core in &self.capabilities.ip_cores {
            // Write reset bit (typically bit 31 of control register)
            let _ = self.write_reg(core.base_address, 0x8000_0000);
        }

        // Clear reset
        for core in &self.capabilities.ip_cores {
            let _ = self.write_reg(core.base_address, 0x0000_0000);
        }

        Ok(())
    }
}

impl ZynqFpga {
    fn wait_for_done(&self, status_addr: usize, timeout_ms: u32) -> FpgaResult<()> {
        let start = std::time::Instant::now();
        let timeout = std::time::Duration::from_millis(timeout_ms as u64);

        loop {
            let status = self.read_reg(status_addr)?;
            if status & 0x01 != 0 {
                return Ok(());
            }

            if start.elapsed() > timeout {
                return Err(FpgaError::Timeout {
                    address: status_addr,
                    timeout_ms,
                });
            }

            std::thread::sleep(std::time::Duration::from_micros(10));
        }
    }

    fn read_bitstream_info() -> (Option<String>, Option<String>) {
        // Try to read from FPGA manager
        let version = std::fs::read_to_string("/sys/class/fpga_manager/fpga0/firmware")
            .ok()
            .map(|s| s.trim().to_string());

        let name = std::fs::read_to_string("/sys/class/fpga_manager/fpga0/name")
            .ok()
            .map(|s| s.trim().to_string());

        (version, name)
    }
}

impl Drop for ZynqFpga {
    fn drop(&mut self) {
        // Clean up streams
        let handles: Vec<_> = self.streams.keys().cloned().collect();
        for handle in handles {
            let _ = self.stop_stream(handle);
        }
    }
}
