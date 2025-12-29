//! DMA controller for high-bandwidth data transfer

use std::collections::HashMap;
use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::os::unix::io::AsRawFd;
use std::ptr;

use crate::types::IQSample;

use super::config::ZynqConfig;
use super::mmap::MemoryRegion;
use crate::error::{FpgaError, FpgaResult};
use crate::types::DmaBuffer;

/// AXI DMA register offsets
mod regs {
    pub const MM2S_DMACR: usize = 0x00; // MM2S DMA Control
    pub const MM2S_DMASR: usize = 0x04; // MM2S DMA Status
    pub const MM2S_SA: usize = 0x18; // MM2S Source Address
    pub const MM2S_SA_MSB: usize = 0x1C; // MM2S Source Address MSB
    pub const MM2S_LENGTH: usize = 0x28; // MM2S Transfer Length

    pub const S2MM_DMACR: usize = 0x30; // S2MM DMA Control
    pub const S2MM_DMASR: usize = 0x34; // S2MM DMA Status
    pub const S2MM_DA: usize = 0x48; // S2MM Destination Address
    pub const S2MM_DA_MSB: usize = 0x4C; // S2MM Destination Address MSB
    pub const S2MM_LENGTH: usize = 0x58; // S2MM Transfer Length

    // Control register bits
    pub const DMACR_RS: u32 = 1 << 0; // Run/Stop
    pub const DMACR_RESET: u32 = 1 << 2; // Soft Reset

    // Status register bits
    pub const DMASR_HALTED: u32 = 1 << 0;
    pub const DMASR_IDLE: u32 = 1 << 1;
    pub const DMASR_IOC_IRQ: u32 = 1 << 12; // Interrupt on Complete
    pub const DMASR_ERR_IRQ: u32 = 1 << 14; // Error Interrupt
}

/// DMA channel state
struct DmaChannel {
    /// Buffer for this channel
    buffer: DmaBuffer,

    /// Whether channel is allocated
    allocated: bool,

    /// Current write position
    write_pos: usize,

    /// Current read position
    read_pos: usize,
}

/// AXI DMA controller
pub struct DmaController {
    /// Memory-mapped DMA registers
    regs: MemoryRegion,

    /// DMA buffer pool
    channels: HashMap<usize, DmaChannel>,

    /// Buffer size per channel
    buffer_size: usize,

    /// Next channel ID
    next_channel: usize,

    /// CMA (Contiguous Memory Allocator) device
    cma_file: Option<File>,
}

impl DmaController {
    /// Create a new DMA controller
    pub fn new(config: &ZynqConfig) -> FpgaResult<Self> {
        let dma_base = config.dma_base_addr.ok_or_else(|| {
            FpgaError::ConfigError("DMA base address not configured".to_string())
        })?;

        let regs = MemoryRegion::new("dma".to_string(), dma_base, 0x1000)
            .map_err(|e| FpgaError::MmapFailed {
                address: dma_base,
                reason: e.to_string(),
            })?;

        // Try to open CMA device for DMA buffer allocation
        let cma_file = OpenOptions::new()
            .read(true)
            .write(true)
            .open("/dev/cma")
            .or_else(|_| OpenOptions::new().read(true).write(true).open("/dev/udmabuf0"))
            .ok();

        let mut controller = Self {
            regs,
            channels: HashMap::new(),
            buffer_size: config.dma_buffer_size,
            next_channel: 0,
            cma_file,
        };

        // Reset DMA engine
        controller.reset()?;

        Ok(controller)
    }

    /// Reset the DMA controller
    pub fn reset(&mut self) -> FpgaResult<()> {
        // Reset MM2S
        self.regs
            .write32(regs::MM2S_DMACR, regs::DMACR_RESET)
            .map_err(FpgaError::OpenFailed)?;

        // Reset S2MM
        self.regs
            .write32(regs::S2MM_DMACR, regs::DMACR_RESET)
            .map_err(FpgaError::OpenFailed)?;

        // Wait for reset to complete
        let timeout = std::time::Duration::from_millis(100);
        let start = std::time::Instant::now();

        loop {
            let mm2s_cr = self.regs.read32(regs::MM2S_DMACR).map_err(FpgaError::OpenFailed)?;
            let s2mm_cr = self.regs.read32(regs::S2MM_DMACR).map_err(FpgaError::OpenFailed)?;

            if (mm2s_cr & regs::DMACR_RESET) == 0 && (s2mm_cr & regs::DMACR_RESET) == 0 {
                break;
            }

            if start.elapsed() > timeout {
                return Err(FpgaError::DmaError("DMA reset timeout".to_string()));
            }

            std::thread::sleep(std::time::Duration::from_micros(10));
        }

        Ok(())
    }

    /// Allocate a DMA channel
    pub fn allocate_channel(&mut self, buffer_size: usize) -> FpgaResult<usize> {
        let buffer = self.allocate_buffer(buffer_size)?;

        let channel_id = self.next_channel;
        self.next_channel += 1;

        self.channels.insert(
            channel_id,
            DmaChannel {
                buffer,
                allocated: true,
                write_pos: 0,
                read_pos: 0,
            },
        );

        Ok(channel_id)
    }

    /// Release a DMA channel
    pub fn release_channel(&mut self, channel: usize) -> FpgaResult<()> {
        self.channels.remove(&channel).ok_or_else(|| {
            FpgaError::DmaError(format!("Channel {} not found", channel))
        })?;
        Ok(())
    }

    /// Allocate a DMA buffer
    fn allocate_buffer(&self, size: usize) -> FpgaResult<DmaBuffer> {
        // Try to allocate from CMA
        if let Some(ref _cma) = self.cma_file {
            // CMA allocation would go here
            // For now, fall back to regular allocation
        }

        // Fallback: allocate regular memory (won't work for real DMA)
        // This is for simulation/testing only
        let layout = std::alloc::Layout::from_size_align(size, 4096).map_err(|_| {
            FpgaError::DmaError("Invalid buffer layout".to_string())
        })?;

        let ptr = unsafe { std::alloc::alloc_zeroed(layout) };
        if ptr.is_null() {
            return Err(FpgaError::DmaError("Buffer allocation failed".to_string()));
        }

        Ok(DmaBuffer {
            phys_addr: ptr as usize, // In real system, this would be physical
            virt_addr: ptr,
            size,
            in_use: false,
        })
    }

    /// Write samples to FPGA via DMA
    pub fn write_samples(&mut self, samples: &[IQSample]) -> FpgaResult<()> {
        let buffer_size = samples.len() * std::mem::size_of::<IQSample>();

        // Allocate temporary buffer if needed
        let buffer = self.allocate_buffer(buffer_size)?;

        // Copy samples to DMA buffer
        unsafe {
            let dst = buffer.virt_addr as *mut IQSample;
            ptr::copy_nonoverlapping(samples.as_ptr(), dst, samples.len());
        }

        // Start MM2S transfer
        self.start_mm2s_transfer(buffer.phys_addr, buffer_size)?;

        // Wait for completion
        self.wait_mm2s_complete(1000)?;

        // Free buffer
        unsafe {
            let layout = std::alloc::Layout::from_size_align(buffer.size, 4096).unwrap();
            std::alloc::dealloc(buffer.virt_addr, layout);
        }

        Ok(())
    }

    /// Read samples from FPGA via DMA
    pub fn read_samples(&mut self, count: usize) -> FpgaResult<Vec<IQSample>> {
        let buffer_size = count * std::mem::size_of::<IQSample>();

        // Allocate buffer
        let buffer = self.allocate_buffer(buffer_size)?;

        // Start S2MM transfer
        self.start_s2mm_transfer(buffer.phys_addr, buffer_size)?;

        // Wait for completion
        self.wait_s2mm_complete(1000)?;

        // Copy samples from buffer
        let mut samples = vec![IQSample::new(0.0, 0.0); count];
        unsafe {
            let src = buffer.virt_addr as *const IQSample;
            ptr::copy_nonoverlapping(src, samples.as_mut_ptr(), count);
        }

        // Free buffer
        unsafe {
            let layout = std::alloc::Layout::from_size_align(buffer.size, 4096).unwrap();
            std::alloc::dealloc(buffer.virt_addr, layout);
        }

        Ok(samples)
    }

    /// Write samples to a channel
    pub fn write_to_channel(&mut self, channel: usize, samples: &[IQSample]) -> FpgaResult<usize> {
        let ch = self.channels.get_mut(&channel).ok_or_else(|| {
            FpgaError::DmaError(format!("Channel {} not found", channel))
        })?;

        let sample_size = std::mem::size_of::<IQSample>();
        let available = (ch.buffer.size - ch.write_pos) / sample_size;
        let to_write = samples.len().min(available);

        if to_write == 0 {
            return Ok(0);
        }

        unsafe {
            let dst = ch.buffer.virt_addr.add(ch.write_pos) as *mut IQSample;
            ptr::copy_nonoverlapping(samples.as_ptr(), dst, to_write);
        }

        ch.write_pos += to_write * sample_size;

        Ok(to_write)
    }

    /// Read samples from a channel
    pub fn read_from_channel(
        &mut self,
        channel: usize,
        buffer: &mut [IQSample],
    ) -> FpgaResult<usize> {
        let ch = self.channels.get_mut(&channel).ok_or_else(|| {
            FpgaError::DmaError(format!("Channel {} not found", channel))
        })?;

        let sample_size = std::mem::size_of::<IQSample>();
        let available = (ch.write_pos - ch.read_pos) / sample_size;
        let to_read = buffer.len().min(available);

        if to_read == 0 {
            return Ok(0);
        }

        unsafe {
            let src = ch.buffer.virt_addr.add(ch.read_pos) as *const IQSample;
            ptr::copy_nonoverlapping(src, buffer.as_mut_ptr(), to_read);
        }

        ch.read_pos += to_read * sample_size;

        // Reset positions if buffer is empty
        if ch.read_pos == ch.write_pos {
            ch.read_pos = 0;
            ch.write_pos = 0;
        }

        Ok(to_read)
    }

    fn start_mm2s_transfer(&mut self, phys_addr: usize, length: usize) -> FpgaResult<()> {
        // Set source address
        self.regs
            .write32(regs::MM2S_SA, phys_addr as u32)
            .map_err(FpgaError::OpenFailed)?;

        // Start the engine
        self.regs
            .write32(regs::MM2S_DMACR, regs::DMACR_RS)
            .map_err(FpgaError::OpenFailed)?;

        // Set length to start transfer
        self.regs
            .write32(regs::MM2S_LENGTH, length as u32)
            .map_err(FpgaError::OpenFailed)?;

        Ok(())
    }

    fn start_s2mm_transfer(&mut self, phys_addr: usize, length: usize) -> FpgaResult<()> {
        // Set destination address
        self.regs
            .write32(regs::S2MM_DA, phys_addr as u32)
            .map_err(FpgaError::OpenFailed)?;

        // Start the engine
        self.regs
            .write32(regs::S2MM_DMACR, regs::DMACR_RS)
            .map_err(FpgaError::OpenFailed)?;

        // Set length to start transfer
        self.regs
            .write32(regs::S2MM_LENGTH, length as u32)
            .map_err(FpgaError::OpenFailed)?;

        Ok(())
    }

    fn wait_mm2s_complete(&self, timeout_ms: u32) -> FpgaResult<()> {
        let timeout = std::time::Duration::from_millis(timeout_ms as u64);
        let start = std::time::Instant::now();

        loop {
            let status = self.regs.read32(regs::MM2S_DMASR).map_err(FpgaError::OpenFailed)?;

            if status & regs::DMASR_ERR_IRQ != 0 {
                return Err(FpgaError::DmaError("MM2S transfer error".to_string()));
            }

            if status & regs::DMASR_IOC_IRQ != 0 {
                // Clear interrupt
                self.regs
                    .write32(regs::MM2S_DMASR, regs::DMASR_IOC_IRQ)
                    .map_err(FpgaError::OpenFailed)?;
                return Ok(());
            }

            if start.elapsed() > timeout {
                return Err(FpgaError::DmaError("MM2S transfer timeout".to_string()));
            }

            std::thread::sleep(std::time::Duration::from_micros(10));
        }
    }

    fn wait_s2mm_complete(&self, timeout_ms: u32) -> FpgaResult<()> {
        let timeout = std::time::Duration::from_millis(timeout_ms as u64);
        let start = std::time::Instant::now();

        loop {
            let status = self.regs.read32(regs::S2MM_DMASR).map_err(FpgaError::OpenFailed)?;

            if status & regs::DMASR_ERR_IRQ != 0 {
                return Err(FpgaError::DmaError("S2MM transfer error".to_string()));
            }

            if status & regs::DMASR_IOC_IRQ != 0 {
                // Clear interrupt
                self.regs
                    .write32(regs::S2MM_DMASR, regs::DMASR_IOC_IRQ)
                    .map_err(FpgaError::OpenFailed)?;
                return Ok(());
            }

            if start.elapsed() > timeout {
                return Err(FpgaError::DmaError("S2MM transfer timeout".to_string()));
            }

            std::thread::sleep(std::time::Duration::from_micros(10));
        }
    }
}

impl Drop for DmaController {
    fn drop(&mut self) {
        // Reset DMA on cleanup
        let _ = self.reset();

        // Free channel buffers
        for (_, channel) in self.channels.drain() {
            unsafe {
                if let Ok(layout) =
                    std::alloc::Layout::from_size_align(channel.buffer.size, 4096)
                {
                    std::alloc::dealloc(channel.buffer.virt_addr, layout);
                }
            }
        }
    }
}
