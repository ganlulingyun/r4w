//! Zynq configuration

use crate::error::{FpgaError, FpgaResult};

/// Configuration for Zynq FPGA
#[derive(Debug, Clone)]
pub struct ZynqConfig {
    /// Named register regions: (name, base_address, size)
    pub register_regions: Vec<(String, usize, usize)>,

    /// FFT IP core base address (if present)
    pub fft_base_addr: Option<usize>,

    /// FIR IP core base address (if present)
    pub fir_base_addr: Option<usize>,

    /// Chirp generator base address (if present)
    pub chirp_base_addr: Option<usize>,

    /// DMA controller base address
    pub dma_base_addr: Option<usize>,

    /// Enable DMA transfers
    pub enable_dma: bool,

    /// Enable interrupt support via UIO
    pub enable_interrupts: bool,

    /// DMA buffer size in bytes
    pub dma_buffer_size: usize,

    /// FPGA fabric clock frequency in Hz
    pub fabric_clock_hz: u64,

    /// Path to device memory (usually /dev/mem)
    pub dev_mem_path: String,
}

impl Default for ZynqConfig {
    fn default() -> Self {
        Self {
            register_regions: vec![
                // Default R4W IP core region
                ("r4w_ip".to_string(), 0x4000_0000, 0x0010_0000),
            ],
            fft_base_addr: Some(0x4000_0000),
            fir_base_addr: Some(0x4001_0000),
            chirp_base_addr: Some(0x4002_0000),
            dma_base_addr: Some(0x4040_0000),
            enable_dma: true,
            enable_interrupts: true,
            dma_buffer_size: 256 * 1024,
            fabric_clock_hz: 100_000_000,
            dev_mem_path: "/dev/mem".to_string(),
        }
    }
}

impl ZynqConfig {
    /// Create configuration from device tree
    pub fn auto_detect() -> FpgaResult<Self> {
        let mut config = Self::default();

        // Try to read from device tree overlays
        if let Ok(regions) = Self::parse_device_tree() {
            config.register_regions = regions;
        }

        Ok(config)
    }

    /// Create configuration for specific IP layout
    pub fn with_ip_cores(
        fft_base: Option<usize>,
        fir_base: Option<usize>,
        chirp_base: Option<usize>,
    ) -> Self {
        let mut config = Self::default();
        config.fft_base_addr = fft_base;
        config.fir_base_addr = fir_base;
        config.chirp_base_addr = chirp_base;

        // Add regions for each IP core
        config.register_regions.clear();
        if let Some(addr) = fft_base {
            config.register_regions.push(("fft".to_string(), addr, 0x1000));
        }
        if let Some(addr) = fir_base {
            config.register_regions.push(("fir".to_string(), addr, 0x1000));
        }
        if let Some(addr) = chirp_base {
            config.register_regions.push(("chirp".to_string(), addr, 0x1000));
        }

        config
    }

    /// Parse device tree for IP core addresses
    fn parse_device_tree() -> FpgaResult<Vec<(String, usize, usize)>> {
        // Look for R4W IP cores in device tree
        let base_path = "/sys/firmware/devicetree/base/amba_pl";

        if !std::path::Path::new(base_path).exists() {
            return Err(FpgaError::DeviceNotFound(
                "No PL device tree entries".to_string(),
            ));
        }

        let mut regions = Vec::new();

        // Iterate through amba_pl children
        if let Ok(entries) = std::fs::read_dir(base_path) {
            for entry in entries.filter_map(|e| e.ok()) {
                let path = entry.path();
                let name = entry.file_name().to_string_lossy().to_string();

                // Check for r4w IP cores
                if name.contains("r4w") || name.contains("fft") || name.contains("fir") {
                    // Read reg property for base address and size
                    let reg_path = path.join("reg");
                    if let Ok(reg_data) = std::fs::read(&reg_path) {
                        if reg_data.len() >= 8 {
                            // Device tree is big-endian
                            let base = u32::from_be_bytes([
                                reg_data[0],
                                reg_data[1],
                                reg_data[2],
                                reg_data[3],
                            ]) as usize;
                            let size = u32::from_be_bytes([
                                reg_data[4],
                                reg_data[5],
                                reg_data[6],
                                reg_data[7],
                            ]) as usize;

                            regions.push((name, base, size));
                        }
                    }
                }
            }
        }

        if regions.is_empty() {
            Err(FpgaError::DeviceNotFound(
                "No R4W IP cores found in device tree".to_string(),
            ))
        } else {
            Ok(regions)
        }
    }

    /// Builder: set DMA buffer size
    pub fn dma_buffer_size(mut self, size: usize) -> Self {
        self.dma_buffer_size = size;
        self
    }

    /// Builder: disable DMA
    pub fn disable_dma(mut self) -> Self {
        self.enable_dma = false;
        self
    }

    /// Builder: disable interrupts
    pub fn disable_interrupts(mut self) -> Self {
        self.enable_interrupts = false;
        self
    }

    /// Builder: set fabric clock
    pub fn fabric_clock(mut self, hz: u64) -> Self {
        self.fabric_clock_hz = hz;
        self
    }
}
