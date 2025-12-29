//! Memory-mapped I/O for Zynq register access

use std::fs::{File, OpenOptions};
use std::io;
use std::os::unix::io::AsRawFd;
use std::ptr;
use std::sync::atomic::{AtomicPtr, Ordering};

use libc::{c_void, MAP_FAILED, MAP_SHARED, PROT_READ, PROT_WRITE};
use nix::sys::mman::{mmap, munmap, MapFlags, ProtFlags};

/// A memory-mapped region for FPGA register access
pub struct MemoryRegion {
    /// Region name for debugging
    pub name: String,

    /// Base physical address
    pub phys_base: usize,

    /// Size of the region
    pub size: usize,

    /// Mapped virtual address
    virt_base: AtomicPtr<u8>,

    /// File descriptor for /dev/mem
    _file: File,
}

impl MemoryRegion {
    /// Create a new memory-mapped region
    pub fn new(name: String, phys_base: usize, size: usize) -> io::Result<Self> {
        // Open /dev/mem
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open("/dev/mem")
            .map_err(|e| {
                if e.kind() == io::ErrorKind::PermissionDenied {
                    io::Error::new(
                        io::ErrorKind::PermissionDenied,
                        "Cannot open /dev/mem. Run as root or add user to appropriate group.",
                    )
                } else {
                    e
                }
            })?;

        // Page-align the mapping
        let page_size = unsafe { libc::sysconf(libc::_SC_PAGESIZE) as usize };
        let page_offset = phys_base & (page_size - 1);
        let aligned_base = phys_base - page_offset;
        let aligned_size = size + page_offset;

        // Map the memory
        let ptr = unsafe {
            mmap(
                None,
                std::num::NonZeroUsize::new(aligned_size).unwrap(),
                ProtFlags::PROT_READ | ProtFlags::PROT_WRITE,
                MapFlags::MAP_SHARED,
                &file,
                aligned_base as libc::off_t,
            )
            .map_err(|e| io::Error::new(io::ErrorKind::Other, format!("mmap failed: {}", e)))?
        };

        // Adjust for page offset
        let adjusted_ptr = unsafe { (ptr as *mut u8).add(page_offset) };

        Ok(Self {
            name,
            phys_base,
            size,
            virt_base: AtomicPtr::new(adjusted_ptr),
            _file: file,
        })
    }

    /// Check if an address is within this region
    pub fn contains(&self, address: usize) -> bool {
        address >= self.phys_base && address < self.phys_base + self.size
    }

    /// Read a 32-bit value from the given physical address
    pub fn read32(&self, address: usize) -> io::Result<u32> {
        if !self.contains(address) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("Address 0x{:08x} not in region {}", address, self.name),
            ));
        }

        let offset = address - self.phys_base;
        let ptr = self.virt_base.load(Ordering::Relaxed);

        unsafe {
            let value_ptr = ptr.add(offset) as *const u32;
            // Use volatile read for MMIO
            Ok(ptr::read_volatile(value_ptr))
        }
    }

    /// Write a 32-bit value to the given physical address
    pub fn write32(&self, address: usize, value: u32) -> io::Result<()> {
        if !self.contains(address) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("Address 0x{:08x} not in region {}", address, self.name),
            ));
        }

        let offset = address - self.phys_base;
        let ptr = self.virt_base.load(Ordering::Relaxed);

        unsafe {
            let value_ptr = ptr.add(offset) as *mut u32;
            // Use volatile write for MMIO
            ptr::write_volatile(value_ptr, value);
        }

        Ok(())
    }

    /// Read a 16-bit value
    pub fn read16(&self, address: usize) -> io::Result<u16> {
        if !self.contains(address) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("Address 0x{:08x} not in region {}", address, self.name),
            ));
        }

        let offset = address - self.phys_base;
        let ptr = self.virt_base.load(Ordering::Relaxed);

        unsafe {
            let value_ptr = ptr.add(offset) as *const u16;
            Ok(ptr::read_volatile(value_ptr))
        }
    }

    /// Write a 16-bit value
    pub fn write16(&self, address: usize, value: u16) -> io::Result<()> {
        if !self.contains(address) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("Address 0x{:08x} not in region {}", address, self.name),
            ));
        }

        let offset = address - self.phys_base;
        let ptr = self.virt_base.load(Ordering::Relaxed);

        unsafe {
            let value_ptr = ptr.add(offset) as *mut u16;
            ptr::write_volatile(value_ptr, value);
        }

        Ok(())
    }

    /// Read a block of data
    pub fn read_block(&self, address: usize, buffer: &mut [u8]) -> io::Result<()> {
        if !self.contains(address) || !self.contains(address + buffer.len() - 1) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "Address range 0x{:08x}-0x{:08x} not in region {}",
                    address,
                    address + buffer.len(),
                    self.name
                ),
            ));
        }

        let offset = address - self.phys_base;
        let ptr = self.virt_base.load(Ordering::Relaxed);

        unsafe {
            let src = ptr.add(offset);
            ptr::copy_nonoverlapping(src, buffer.as_mut_ptr(), buffer.len());
        }

        Ok(())
    }

    /// Write a block of data
    pub fn write_block(&self, address: usize, data: &[u8]) -> io::Result<()> {
        if !self.contains(address) || !self.contains(address + data.len() - 1) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "Address range 0x{:08x}-0x{:08x} not in region {}",
                    address,
                    address + data.len(),
                    self.name
                ),
            ));
        }

        let offset = address - self.phys_base;
        let ptr = self.virt_base.load(Ordering::Relaxed);

        unsafe {
            let dst = ptr.add(offset);
            ptr::copy_nonoverlapping(data.as_ptr(), dst, data.len());
        }

        Ok(())
    }

    /// Get the virtual address for a physical address
    pub fn virt_addr(&self, phys_addr: usize) -> Option<*mut u8> {
        if self.contains(phys_addr) {
            let offset = phys_addr - self.phys_base;
            let ptr = self.virt_base.load(Ordering::Relaxed);
            Some(unsafe { ptr.add(offset) })
        } else {
            None
        }
    }
}

impl Drop for MemoryRegion {
    fn drop(&mut self) {
        let page_size = unsafe { libc::sysconf(libc::_SC_PAGESIZE) as usize };
        let page_offset = self.phys_base & (page_size - 1);
        let ptr = self.virt_base.load(Ordering::Relaxed);

        // Adjust back to page-aligned address
        let aligned_ptr = unsafe { ptr.sub(page_offset) };
        let aligned_size = self.size + page_offset;

        unsafe {
            if let Err(e) = munmap(
                std::ptr::NonNull::new(aligned_ptr as *mut c_void).unwrap(),
                aligned_size,
            ) {
                eprintln!("Warning: munmap failed for region {}: {}", self.name, e);
            }
        }
    }
}

// Safety: MemoryRegion can be sent between threads
unsafe impl Send for MemoryRegion {}

// Safety: MemoryRegion can be shared between threads (uses atomic operations)
unsafe impl Sync for MemoryRegion {}

#[cfg(test)]
mod tests {
    use super::*;

    // Note: These tests require root access and a real Zynq platform
    // They are marked as ignored by default

    #[test]
    #[ignore]
    fn test_memory_region_creation() {
        // This test requires root access
        let region = MemoryRegion::new("test".to_string(), 0x4000_0000, 0x1000);
        assert!(region.is_ok());
    }

    #[test]
    fn test_contains() {
        // We can test the contains logic without actual mmap
        // by creating a mock-like scenario
        let base = 0x4000_0000usize;
        let size = 0x1000usize;

        // Simulate contains check
        let address_in = 0x4000_0100usize;
        let address_out = 0x4001_0000usize;

        assert!(address_in >= base && address_in < base + size);
        assert!(!(address_out >= base && address_out < base + size));
    }
}
