//! Lattice FPGA configuration

use crate::error::{FpgaError, FpgaResult};
use crate::types::FpgaPlatform;

use super::ftdi::FtdiDevice;

/// Configuration for Lattice FPGA
#[derive(Debug, Clone)]
pub struct LatticeConfig {
    /// Platform variant
    pub platform: FpgaPlatform,

    /// FTDI device index (for multiple devices)
    pub ftdi_index: usize,

    /// FTDI interface (A or B on FT2232H)
    pub ftdi_interface: u8,

    /// SPI clock frequency in Hz
    pub spi_clock_hz: u32,

    /// FPGA fabric clock frequency in Hz
    pub clock_frequency_hz: u64,

    /// Reset GPIO pin (on FTDI)
    pub reset_pin: u8,

    /// Config done GPIO pin (on FTDI)
    pub cdone_pin: u8,

    /// Chip select GPIO pin (on FTDI)
    pub cs_pin: u8,
}

impl Default for LatticeConfig {
    fn default() -> Self {
        Self {
            platform: FpgaPlatform::LatticeIce40 {
                variant: "hx8k".to_string(),
            },
            ftdi_index: 0,
            ftdi_interface: 0, // Interface A
            spi_clock_hz: 6_000_000, // 6 MHz
            clock_frequency_hz: 12_000_000, // 12 MHz crystal
            reset_pin: 7,  // FTDI GPIOL3
            cdone_pin: 6,  // FTDI GPIOL2
            cs_pin: 4,     // FTDI GPIOL0
        }
    }
}

impl LatticeConfig {
    /// Configuration for iCE40-HX8K breakout board
    pub fn ice40_hx8k() -> Self {
        Self {
            platform: FpgaPlatform::LatticeIce40 {
                variant: "hx8k".to_string(),
            },
            ..Default::default()
        }
    }

    /// Configuration for iCE40-HX4K (e.g., iCEstick)
    pub fn ice40_hx4k() -> Self {
        Self {
            platform: FpgaPlatform::LatticeIce40 {
                variant: "hx4k".to_string(),
            },
            ..Default::default()
        }
    }

    /// Configuration for iCE40-HX1K (e.g., iCEstick)
    pub fn ice40_hx1k() -> Self {
        Self {
            platform: FpgaPlatform::LatticeIce40 {
                variant: "hx1k".to_string(),
            },
            ..Default::default()
        }
    }

    /// Configuration for iCE40-UP5K (UltraPlus)
    pub fn ice40_up5k() -> Self {
        Self {
            platform: FpgaPlatform::LatticeIce40 {
                variant: "up5k".to_string(),
            },
            spi_clock_hz: 3_000_000, // Slower for UP5K
            clock_frequency_hz: 48_000_000, // Internal oscillator
            ..Default::default()
        }
    }

    /// Configuration for ECP5 Evaluation Board
    pub fn ecp5_evn() -> Self {
        Self {
            platform: FpgaPlatform::LatticeEcp5 {
                variant: "lfe5u-45".to_string(),
            },
            spi_clock_hz: 10_000_000, // ECP5 supports faster SPI
            clock_frequency_hz: 100_000_000,
            ..Default::default()
        }
    }

    /// Configuration for ECP5 ULX3S board
    pub fn ecp5_ulx3s(variant: &str) -> Self {
        Self {
            platform: FpgaPlatform::LatticeEcp5 {
                variant: variant.to_string(),
            },
            spi_clock_hz: 10_000_000,
            clock_frequency_hz: 25_000_000, // ULX3S crystal
            ..Default::default()
        }
    }

    /// Configuration for OrangeCrab (ECP5)
    pub fn ecp5_orangecrab() -> Self {
        Self {
            platform: FpgaPlatform::LatticeEcp5 {
                variant: "lfe5u-25".to_string(),
            },
            spi_clock_hz: 10_000_000,
            clock_frequency_hz: 48_000_000,
            ..Default::default()
        }
    }

    /// Create configuration from detected FTDI device
    pub fn from_ftdi_device(device: &FtdiDevice) -> FpgaResult<Self> {
        // Try to identify the board from FTDI description
        let mut config = Self::default();
        config.ftdi_index = device.index;

        // Match known board descriptions
        let desc = device.description.to_lowercase();

        if desc.contains("icestick") {
            config.platform = FpgaPlatform::LatticeIce40 {
                variant: "hx1k".to_string(),
            };
        } else if desc.contains("ice40-hx8k") || desc.contains("hx8k") {
            config.platform = FpgaPlatform::LatticeIce40 {
                variant: "hx8k".to_string(),
            };
        } else if desc.contains("up5k") || desc.contains("ultraplus") {
            config.platform = FpgaPlatform::LatticeIce40 {
                variant: "up5k".to_string(),
            };
        } else if desc.contains("ecp5") || desc.contains("ulx3s") {
            config.platform = FpgaPlatform::LatticeEcp5 {
                variant: "lfe5u-25".to_string(),
            };
        } else if desc.contains("orangecrab") {
            config.platform = FpgaPlatform::LatticeEcp5 {
                variant: "lfe5u-25".to_string(),
            };
        } else {
            // Unknown board, use default iCE40
            tracing::warn!(
                "Unknown FTDI device: {}, assuming iCE40-HX8K",
                device.description
            );
        }

        Ok(config)
    }

    /// Builder: set SPI clock
    pub fn spi_clock(mut self, hz: u32) -> Self {
        self.spi_clock_hz = hz;
        self
    }

    /// Builder: set fabric clock
    pub fn fabric_clock(mut self, hz: u64) -> Self {
        self.clock_frequency_hz = hz;
        self
    }

    /// Builder: set FTDI interface
    pub fn interface(mut self, iface: u8) -> Self {
        self.ftdi_interface = iface;
        self
    }

    /// Builder: set GPIO pins
    pub fn gpio_pins(mut self, reset: u8, cdone: u8, cs: u8) -> Self {
        self.reset_pin = reset;
        self.cdone_pin = cdone;
        self.cs_pin = cs;
        self
    }
}
