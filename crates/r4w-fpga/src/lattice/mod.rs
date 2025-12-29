//! Lattice FPGA backend (iCE40 and ECP5)
//!
//! This module provides hardware acceleration on Lattice FPGAs using
//! FTDI-based SPI communication. Supports:
//!
//! - **iCE40**: Ultra-low power, open-source toolchain (Yosys + IceStorm)
//! - **ECP5**: Mid-range, open-source toolchain (Yosys + Trellis)
//!
//! # Hardware Setup
//!
//! Typical setup uses an FTDI chip (FT2232H) for SPI communication:
//!
//! ```text
//! ┌─────────────┐     USB      ┌─────────────┐     SPI      ┌─────────────┐
//! │    Host     │◄────────────►│   FT2232H   │◄────────────►│   Lattice   │
//! │    (PC)     │              │   (FTDI)    │              │    FPGA     │
//! └─────────────┘              └─────────────┘              └─────────────┘
//! ```
//!
//! # Open Source Toolchain
//!
//! Lattice FPGAs are supported by the fully open-source toolchain:
//! - **Yosys**: Synthesis (Verilog -> netlist)
//! - **nextpnr**: Place and route
//! - **IceStorm**: iCE40 bitstream tools
//! - **Trellis**: ECP5 bitstream tools
//!
//! # Example
//!
//! ```rust,ignore
//! use r4w_fpga::lattice::{LatticeFpga, LatticeConfig};
//!
//! // Auto-detect connected Lattice FPGA
//! let fpga = LatticeFpga::auto_detect()?;
//!
//! // Or specify configuration
//! let config = LatticeConfig::ice40_hx8k();
//! let fpga = LatticeFpga::new(config)?;
//! ```

mod config;
mod ftdi;
mod spi;

pub use config::LatticeConfig;

use crate::error::{FpgaError, FpgaResult};
use crate::traits::FpgaAccelerator;
use crate::types::{
    FpgaCapabilities, FpgaInfo, FpgaPlatform, IQSample, StreamConfig, StreamHandle, StreamStats,
};

/// Lattice FPGA accelerator implementation
///
/// Communicates with Lattice FPGAs via FTDI SPI interface.
pub struct LatticeFpga {
    /// FPGA variant
    platform: FpgaPlatform,

    /// Configuration
    config: LatticeConfig,

    /// SPI interface
    spi: spi::SpiInterface,

    /// Detected capabilities
    capabilities: FpgaCapabilities,

    /// Whether FPGA is configured with bitstream
    configured: bool,
}

impl LatticeFpga {
    /// Auto-detect connected Lattice FPGA
    pub fn auto_detect() -> FpgaResult<Self> {
        // Try to find an FTDI device with Lattice FPGA attached
        let ftdi_devices = ftdi::enumerate_devices()?;

        if ftdi_devices.is_empty() {
            return Err(FpgaError::DeviceNotFound(
                "No FTDI devices found".to_string(),
            ));
        }

        // Try each device
        for device in ftdi_devices {
            if let Ok(fpga) = Self::try_connect(&device) {
                return Ok(fpga);
            }
        }

        Err(FpgaError::DeviceNotFound(
            "No Lattice FPGA detected on FTDI devices".to_string(),
        ))
    }

    /// Create with specific configuration
    pub fn new(config: LatticeConfig) -> FpgaResult<Self> {
        let spi = spi::SpiInterface::new(&config)?;

        let platform = config.platform.clone();
        let capabilities = Self::build_capabilities(&config);

        Ok(Self {
            platform,
            config,
            spi,
            capabilities,
            configured: false,
        })
    }

    /// Check if Lattice platform is available
    pub fn is_platform_available() -> bool {
        // Check for FTDI USB devices
        ftdi::enumerate_devices()
            .map(|devices| !devices.is_empty())
            .unwrap_or(false)
    }

    /// Try to connect to a specific FTDI device
    fn try_connect(device: &ftdi::FtdiDevice) -> FpgaResult<Self> {
        let config = LatticeConfig::from_ftdi_device(device)?;
        Self::new(config)
    }

    /// Load a bitstream to the FPGA
    pub fn load_bitstream(&mut self, bitstream: &[u8]) -> FpgaResult<()> {
        tracing::info!("Loading bitstream ({} bytes)...", bitstream.len());

        // Reset FPGA
        self.spi.reset_fpga()?;

        // Send bitstream via SPI
        self.spi.send_bitstream(bitstream)?;

        // Wait for configuration to complete
        self.spi.wait_config_done(1000)?;

        self.configured = true;

        // Re-detect capabilities from configured FPGA
        if let Ok(caps) = self.detect_ip_cores() {
            self.capabilities = caps;
        }

        tracing::info!("Bitstream loaded successfully");
        Ok(())
    }

    /// Load bitstream from file
    pub fn load_bitstream_file(&mut self, path: &std::path::Path) -> FpgaResult<()> {
        let bitstream = std::fs::read(path).map_err(|e| {
            FpgaError::BitstreamError(format!("Failed to read bitstream file: {}", e))
        })?;

        self.load_bitstream(&bitstream)
    }

    fn build_capabilities(config: &LatticeConfig) -> FpgaCapabilities {
        let (logic_cells, bram_bytes, dsp_blocks) = match &config.platform {
            FpgaPlatform::LatticeIce40 { variant } => {
                match variant.as_str() {
                    "hx1k" => (1280, 8 * 1024, 0),
                    "hx4k" => (3520, 20 * 1024, 0),
                    "hx8k" => (7680, 32 * 1024, 0),
                    "lp384" => (384, 0, 0),
                    "lp1k" => (1280, 8 * 1024, 0),
                    "lp4k" => (3520, 20 * 1024, 0),
                    "lp8k" => (7680, 32 * 1024, 0),
                    "up5k" => (5280, 30 * 1024, 8),
                    _ => (1000, 8 * 1024, 0),
                }
            }
            FpgaPlatform::LatticeEcp5 { variant } => {
                match variant.as_str() {
                    "12k" | "lfe5u-12" => (12_000, 180 * 1024, 28),
                    "25k" | "lfe5u-25" => (24_000, 504 * 1024, 28),
                    "45k" | "lfe5u-45" => (44_000, 936 * 1024, 64),
                    "85k" | "lfe5u-85" => (84_000, 1944 * 1024, 156),
                    _ => (24_000, 504 * 1024, 28),
                }
            }
            _ => (1000, 8 * 1024, 0),
        };

        FpgaCapabilities {
            // Lattice FPGAs are smaller, so more conservative limits
            max_fft_size: 512,  // Limited by BRAM
            max_fir_taps: 32,   // Limited by logic
            supported_waveforms: vec!["LoRa".to_string()], // Focus on LoRa for small FPGAs
            dma_buffer_size: 4 * 1024, // SPI-based, smaller buffers
            clock_frequency_hz: config.clock_frequency_hz,
            dsp_blocks,
            logic_cells,
            bram_bytes,
            ip_cores: vec![], // Detected after bitstream load
            dma_channels: 0,  // No DMA via SPI
            supports_streaming: false,
            supports_interrupts: false,
        }
    }

    fn detect_ip_cores(&self) -> FpgaResult<FpgaCapabilities> {
        let mut caps = self.capabilities.clone();
        caps.ip_cores.clear();

        // Read ID register from potential IP core locations
        // IP cores are memory-mapped in the FPGA's internal address space

        // For now, return basic capabilities
        // Full implementation would probe SPI registers

        Ok(caps)
    }
}

impl FpgaAccelerator for LatticeFpga {
    fn info(&self) -> FpgaInfo {
        FpgaInfo {
            platform: self.platform.clone(),
            device: match &self.platform {
                FpgaPlatform::LatticeIce40 { variant } => format!("iCE40 {}", variant),
                FpgaPlatform::LatticeEcp5 { variant } => format!("ECP5 {}", variant),
                _ => "Unknown Lattice".to_string(),
            },
            bitstream_version: None,
            bitstream_name: None,
            bitstream_timestamp: None,
            driver_version: Some(env!("CARGO_PKG_VERSION").to_string()),
        }
    }

    fn is_available(&self) -> bool {
        self.configured
    }

    fn capabilities(&self) -> FpgaCapabilities {
        self.capabilities.clone()
    }

    fn fft(&self, samples: &[IQSample], inverse: bool) -> FpgaResult<Vec<IQSample>> {
        if !self.configured {
            return Err(FpgaError::BitstreamError(
                "FPGA not configured".to_string(),
            ));
        }

        let n = samples.len();
        if !n.is_power_of_two() || n > self.capabilities.max_fft_size {
            return Err(FpgaError::ConfigError(format!(
                "Invalid FFT size: {} (max {})",
                n, self.capabilities.max_fft_size
            )));
        }

        // Write samples via SPI
        self.spi.write_samples(samples)?;

        // Start FFT
        let control = if inverse { 0x03 } else { 0x01 };
        self.spi.write_register(0x00, control)?;

        // Wait for completion
        self.spi.wait_for_done(0x08, 1000)?;

        // Read results
        self.spi.read_samples(n)
    }

    fn fir_filter(&self, samples: &[IQSample], taps: &[f32]) -> FpgaResult<Vec<IQSample>> {
        if !self.configured {
            return Err(FpgaError::BitstreamError(
                "FPGA not configured".to_string(),
            ));
        }

        if taps.len() > self.capabilities.max_fir_taps {
            return Err(FpgaError::ConfigError(format!(
                "Too many FIR taps: {} (max {})",
                taps.len(),
                self.capabilities.max_fir_taps
            )));
        }

        // Load taps
        self.spi.write_taps(taps)?;

        // Process samples
        let mut result = Vec::with_capacity(samples.len());
        for chunk in samples.chunks(64) {
            self.spi.write_samples(chunk)?;
            self.spi.write_register(0x00, 0x01)?;
            self.spi.wait_for_done(0x08, 100)?;
            result.extend(self.spi.read_samples(chunk.len())?);
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

        // Software fallback for Lattice (no dedicated complex multiplier)
        Ok(a.iter()
            .zip(b.iter())
            .map(|(x, y)| IQSample::new(x.i * y.i - x.q * y.q, x.i * y.q + x.q * y.i))
            .collect())
    }

    fn modulate(&self, _waveform_id: u32, _bits: &[bool]) -> FpgaResult<Vec<IQSample>> {
        Err(FpgaError::NotSupported(
            "Modulation not yet implemented for Lattice".to_string(),
        ))
    }

    fn demodulate(&self, _waveform_id: u32, _samples: &[IQSample]) -> FpgaResult<Vec<bool>> {
        Err(FpgaError::NotSupported(
            "Demodulation not yet implemented for Lattice".to_string(),
        ))
    }

    fn waveform_id(&self, name: &str) -> Option<u32> {
        match name {
            "LoRa" => Some(1),
            _ => None,
        }
    }

    fn generate_chirp(&self, sf: u8, upchirp: bool) -> FpgaResult<Vec<IQSample>> {
        if !self.configured {
            return Err(FpgaError::BitstreamError(
                "FPGA not configured".to_string(),
            ));
        }

        // Configure chirp generator
        self.spi.write_register(0x04, sf as u32)?;

        let control = if upchirp { 0x03 } else { 0x01 };
        self.spi.write_register(0x00, control)?;

        self.spi.wait_for_done(0x08, 100)?;

        let n = 1usize << sf;
        self.spi.read_samples(n)
    }

    fn chirp_correlate(&self, _samples: &[IQSample], _sf: u8) -> FpgaResult<(u32, f32)> {
        Err(FpgaError::NotSupported(
            "Chirp correlation not yet implemented for Lattice".to_string(),
        ))
    }

    fn start_stream(&mut self, _config: StreamConfig) -> FpgaResult<StreamHandle> {
        Err(FpgaError::NotSupported(
            "Streaming not supported via SPI".to_string(),
        ))
    }

    fn stop_stream(&mut self, _handle: StreamHandle) -> FpgaResult<()> {
        Err(FpgaError::NotSupported(
            "Streaming not supported via SPI".to_string(),
        ))
    }

    fn write_stream(&mut self, _handle: StreamHandle, _samples: &[IQSample]) -> FpgaResult<usize> {
        Err(FpgaError::NotSupported(
            "Streaming not supported via SPI".to_string(),
        ))
    }

    fn read_stream(
        &mut self,
        _handle: StreamHandle,
        _buffer: &mut [IQSample],
    ) -> FpgaResult<usize> {
        Err(FpgaError::NotSupported(
            "Streaming not supported via SPI".to_string(),
        ))
    }

    fn stream_stats(&self, _handle: StreamHandle) -> FpgaResult<StreamStats> {
        Err(FpgaError::NotSupported(
            "Streaming not supported via SPI".to_string(),
        ))
    }

    fn read_register(&self, address: usize) -> FpgaResult<u32> {
        self.spi.read_register(address as u16)
    }

    fn write_register(&mut self, address: usize, value: u32) -> FpgaResult<()> {
        self.spi.write_register(address as u16, value)
    }

    fn reset(&mut self) -> FpgaResult<()> {
        self.spi.reset_fpga()?;
        self.configured = false;
        Ok(())
    }
}
