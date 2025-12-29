//! SPI interface for Lattice FPGA communication

use crate::types::IQSample;

use super::config::LatticeConfig;
use crate::error::{FpgaError, FpgaResult};

/// SPI interface for FPGA communication
pub struct SpiInterface {
    /// Configuration
    config: LatticeConfig,

    /// Whether the interface is open
    open: bool,

    #[cfg(feature = "lattice")]
    /// FTDI context (when using libftdi)
    ftdi_ctx: *mut libftdi1_sys::ftdi_context,
}

impl SpiInterface {
    /// Create a new SPI interface
    pub fn new(config: &LatticeConfig) -> FpgaResult<Self> {
        #[cfg(feature = "lattice")]
        {
            Self::new_with_libftdi(config)
        }

        #[cfg(not(feature = "lattice"))]
        {
            Ok(Self {
                config: config.clone(),
                open: false,
            })
        }
    }

    #[cfg(feature = "lattice")]
    fn new_with_libftdi(config: &LatticeConfig) -> FpgaResult<Self> {
        use libftdi1_sys as ftdi;
        use std::ptr;

        unsafe {
            let ctx = ftdi::ftdi_new();
            if ctx.is_null() {
                return Err(FpgaError::DeviceNotFound(
                    "Failed to create FTDI context".to_string(),
                ));
            }

            // Set interface
            let iface = match config.ftdi_interface {
                0 => ftdi::ftdi_interface_INTERFACE_A,
                1 => ftdi::ftdi_interface_INTERFACE_B,
                2 => ftdi::ftdi_interface_INTERFACE_C,
                3 => ftdi::ftdi_interface_INTERFACE_D,
                _ => ftdi::ftdi_interface_INTERFACE_A,
            };
            ftdi::ftdi_set_interface(ctx, iface);

            // Open device
            let ret = ftdi::ftdi_usb_open_dev(ctx, ptr::null_mut()); // Would need actual device
            if ret < 0 {
                ftdi::ftdi_free(ctx);
                return Err(FpgaError::DeviceNotFound(
                    "Failed to open FTDI device".to_string(),
                ));
            }

            // Configure for MPSSE (SPI) mode
            ftdi::ftdi_set_bitmode(ctx, 0, ftdi::ftdi_mpsse_mode_BITMODE_RESET as u8);
            ftdi::ftdi_set_bitmode(ctx, 0, ftdi::ftdi_mpsse_mode_BITMODE_MPSSE as u8);

            // Purge buffers
            ftdi::ftdi_usb_purge_buffers(ctx);

            Ok(Self {
                config: config.clone(),
                open: true,
                ftdi_ctx: ctx,
            })
        }
    }

    /// Reset the FPGA
    pub fn reset_fpga(&self) -> FpgaResult<()> {
        // Assert reset pin (low)
        self.set_gpio(self.config.reset_pin, false)?;

        // Wait
        std::thread::sleep(std::time::Duration::from_millis(10));

        // Deassert reset (high)
        self.set_gpio(self.config.reset_pin, true)?;

        // Wait for FPGA to be ready
        std::thread::sleep(std::time::Duration::from_millis(50));

        Ok(())
    }

    /// Send bitstream to FPGA
    pub fn send_bitstream(&self, bitstream: &[u8]) -> FpgaResult<()> {
        // Assert CS
        self.set_gpio(self.config.cs_pin, false)?;

        // Send bitstream in chunks
        const CHUNK_SIZE: usize = 4096;

        for chunk in bitstream.chunks(CHUNK_SIZE) {
            self.spi_write(chunk)?;
        }

        // Deassert CS
        self.set_gpio(self.config.cs_pin, true)?;

        Ok(())
    }

    /// Wait for configuration to complete
    pub fn wait_config_done(&self, timeout_ms: u32) -> FpgaResult<()> {
        let start = std::time::Instant::now();
        let timeout = std::time::Duration::from_millis(timeout_ms as u64);

        loop {
            if self.get_gpio(self.config.cdone_pin)? {
                return Ok(());
            }

            if start.elapsed() > timeout {
                return Err(FpgaError::BitstreamError(
                    "Configuration timeout - CDONE not asserted".to_string(),
                ));
            }

            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }

    /// Write a register via SPI
    pub fn write_register(&self, address: u16, value: u32) -> FpgaResult<()> {
        // SPI register protocol:
        // Byte 0: Command (0x80 | write flag)
        // Byte 1-2: Address (big-endian)
        // Byte 3-6: Value (big-endian)

        let cmd = [
            0x80, // Write command
            (address >> 8) as u8,
            address as u8,
            (value >> 24) as u8,
            (value >> 16) as u8,
            (value >> 8) as u8,
            value as u8,
        ];

        self.set_gpio(self.config.cs_pin, false)?;
        self.spi_write(&cmd)?;
        self.set_gpio(self.config.cs_pin, true)?;

        Ok(())
    }

    /// Read a register via SPI
    pub fn read_register(&self, address: u16) -> FpgaResult<u32> {
        // SPI register protocol:
        // Byte 0: Command (0x00 | read flag)
        // Byte 1-2: Address (big-endian)
        // Then read 4 bytes

        let cmd = [
            0x00, // Read command
            (address >> 8) as u8,
            address as u8,
        ];

        self.set_gpio(self.config.cs_pin, false)?;
        self.spi_write(&cmd)?;

        let mut response = [0u8; 4];
        self.spi_read(&mut response)?;
        self.set_gpio(self.config.cs_pin, true)?;

        let value = u32::from_be_bytes(response);
        Ok(value)
    }

    /// Wait for done flag in a register
    pub fn wait_for_done(&self, status_addr: u16, timeout_ms: u32) -> FpgaResult<()> {
        let start = std::time::Instant::now();
        let timeout = std::time::Duration::from_millis(timeout_ms as u64);

        loop {
            let status = self.read_register(status_addr)?;
            if status & 0x01 != 0 {
                return Ok(());
            }

            if start.elapsed() > timeout {
                return Err(FpgaError::Timeout {
                    address: status_addr as usize,
                    timeout_ms,
                });
            }

            std::thread::sleep(std::time::Duration::from_micros(100));
        }
    }

    /// Write samples to FPGA
    pub fn write_samples(&self, samples: &[IQSample]) -> FpgaResult<()> {
        // Pack samples as 16-bit I/Q pairs
        let mut data = Vec::with_capacity(samples.len() * 4);

        for sample in samples {
            let i = (sample.i * 32767.0) as i16;
            let q = (sample.q * 32767.0) as i16;
            data.extend_from_slice(&i.to_be_bytes());
            data.extend_from_slice(&q.to_be_bytes());
        }

        // Write data command
        let cmd = [0x40, 0x00, 0x10]; // Write to data port at 0x0010
        self.set_gpio(self.config.cs_pin, false)?;
        self.spi_write(&cmd)?;
        self.spi_write(&data)?;
        self.set_gpio(self.config.cs_pin, true)?;

        Ok(())
    }

    /// Read samples from FPGA
    pub fn read_samples(&self, count: usize) -> FpgaResult<Vec<IQSample>> {
        // Read data command
        let cmd = [0x00, 0x00, 0x14]; // Read from data port at 0x0014
        self.set_gpio(self.config.cs_pin, false)?;
        self.spi_write(&cmd)?;

        // Read packed samples
        let mut data = vec![0u8; count * 4];
        self.spi_read(&mut data)?;
        self.set_gpio(self.config.cs_pin, true)?;

        // Unpack samples
        let mut samples = Vec::with_capacity(count);
        for chunk in data.chunks(4) {
            if chunk.len() == 4 {
                let i = i16::from_be_bytes([chunk[0], chunk[1]]);
                let q = i16::from_be_bytes([chunk[2], chunk[3]]);
                samples.push(IQSample::new(
                    i as f32 / 32767.0,
                    q as f32 / 32767.0,
                ));
            }
        }

        Ok(samples)
    }

    /// Write FIR filter taps
    pub fn write_taps(&self, taps: &[f32]) -> FpgaResult<()> {
        // Write taps command
        let cmd = [0x40, 0x01, 0x00]; // Write to taps at 0x0100
        self.set_gpio(self.config.cs_pin, false)?;
        self.spi_write(&cmd)?;

        // Pack taps as 16-bit fixed point
        for &tap in taps {
            let fixed = (tap * 32767.0) as i16;
            self.spi_write(&fixed.to_be_bytes())?;
        }

        self.set_gpio(self.config.cs_pin, true)?;

        Ok(())
    }

    // Low-level SPI operations

    fn spi_write(&self, _data: &[u8]) -> FpgaResult<()> {
        #[cfg(feature = "lattice")]
        {
            // Real implementation would use FTDI MPSSE
            Ok(())
        }

        #[cfg(not(feature = "lattice"))]
        {
            Err(FpgaError::NotSupported(
                "Lattice support not compiled in".to_string(),
            ))
        }
    }

    fn spi_read(&self, _buffer: &mut [u8]) -> FpgaResult<()> {
        #[cfg(feature = "lattice")]
        {
            // Real implementation would use FTDI MPSSE
            Ok(())
        }

        #[cfg(not(feature = "lattice"))]
        {
            Err(FpgaError::NotSupported(
                "Lattice support not compiled in".to_string(),
            ))
        }
    }

    fn set_gpio(&self, _pin: u8, _high: bool) -> FpgaResult<()> {
        #[cfg(feature = "lattice")]
        {
            // Real implementation would use FTDI GPIO
            Ok(())
        }

        #[cfg(not(feature = "lattice"))]
        {
            Ok(()) // No-op when not compiled with lattice
        }
    }

    fn get_gpio(&self, _pin: u8) -> FpgaResult<bool> {
        #[cfg(feature = "lattice")]
        {
            // Real implementation would read FTDI GPIO
            Ok(true)
        }

        #[cfg(not(feature = "lattice"))]
        {
            Ok(true) // Always return true when not compiled with lattice
        }
    }
}

impl Drop for SpiInterface {
    fn drop(&mut self) {
        #[cfg(feature = "lattice")]
        {
            if self.open && !self.ftdi_ctx.is_null() {
                unsafe {
                    libftdi1_sys::ftdi_usb_close(self.ftdi_ctx);
                    libftdi1_sys::ftdi_free(self.ftdi_ctx);
                }
            }
        }
    }
}

// Safety: SpiInterface is Send if FTDI operations are serialized
unsafe impl Send for SpiInterface {}
