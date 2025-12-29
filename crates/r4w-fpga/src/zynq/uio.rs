//! UIO (Userspace I/O) support for FPGA interrupts

use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::os::unix::io::AsRawFd;
use std::path::Path;

use crate::error::{FpgaError, FpgaResult};

/// A UIO device for interrupt handling
pub struct UioDevice {
    /// Device name (e.g., "uio0")
    pub name: String,

    /// Device file
    file: File,

    /// Physical address
    pub phys_addr: Option<usize>,

    /// Address size
    pub size: Option<usize>,

    /// IRQ number
    pub irq: Option<u32>,
}

impl UioDevice {
    /// Open a UIO device by name
    pub fn open(name: &str) -> io::Result<Self> {
        let path = format!("/dev/{}", name);
        let file = OpenOptions::new().read(true).write(true).open(&path)?;

        let mut device = Self {
            name: name.to_string(),
            file,
            phys_addr: None,
            size: None,
            irq: None,
        };

        // Read device information from sysfs
        device.read_sysfs_info()?;

        Ok(device)
    }

    /// Read device info from sysfs
    fn read_sysfs_info(&mut self) -> io::Result<()> {
        let sysfs_base = format!("/sys/class/uio/{}", self.name);

        // Read physical address
        let addr_path = format!("{}/maps/map0/addr", sysfs_base);
        if let Ok(addr_str) = std::fs::read_to_string(&addr_path) {
            let addr_str = addr_str.trim().trim_start_matches("0x");
            if let Ok(addr) = usize::from_str_radix(addr_str, 16) {
                self.phys_addr = Some(addr);
            }
        }

        // Read size
        let size_path = format!("{}/maps/map0/size", sysfs_base);
        if let Ok(size_str) = std::fs::read_to_string(&size_path) {
            let size_str = size_str.trim().trim_start_matches("0x");
            if let Ok(size) = usize::from_str_radix(size_str, 16) {
                self.size = Some(size);
            }
        }

        // Read IRQ info
        let event_path = format!("{}/event", sysfs_base);
        if Path::new(&event_path).exists() {
            // UIO uses event file for interrupt count
            self.irq = Some(0); // Actual IRQ number would need additional parsing
        }

        Ok(())
    }

    /// Wait for an interrupt
    pub fn wait_interrupt(&mut self) -> io::Result<u32> {
        let mut buf = [0u8; 4];
        self.file.read_exact(&mut buf)?;
        Ok(u32::from_ne_bytes(buf))
    }

    /// Wait for interrupt with timeout
    pub fn wait_interrupt_timeout(&mut self, timeout_ms: u32) -> io::Result<Option<u32>> {
        use std::os::unix::io::AsRawFd;

        let fd = self.file.as_raw_fd();

        // Use poll for timeout
        let mut pollfd = libc::pollfd {
            fd,
            events: libc::POLLIN,
            revents: 0,
        };

        let result = unsafe { libc::poll(&mut pollfd, 1, timeout_ms as i32) };

        if result < 0 {
            return Err(io::Error::last_os_error());
        }

        if result == 0 {
            return Ok(None); // Timeout
        }

        if pollfd.revents & libc::POLLIN != 0 {
            let count = self.wait_interrupt()?;
            Ok(Some(count))
        } else {
            Ok(None)
        }
    }

    /// Enable interrupts
    pub fn enable_interrupt(&mut self) -> io::Result<()> {
        let buf = 1u32.to_ne_bytes();
        self.file.write_all(&buf)?;
        Ok(())
    }

    /// Disable interrupts
    pub fn disable_interrupt(&mut self) -> io::Result<()> {
        let buf = 0u32.to_ne_bytes();
        self.file.write_all(&buf)?;
        Ok(())
    }

    /// Get the raw file descriptor
    pub fn as_raw_fd(&self) -> i32 {
        self.file.as_raw_fd()
    }
}

/// Discover available UIO devices
pub fn discover_uio_devices() -> FpgaResult<Vec<UioDevice>> {
    let mut devices = Vec::new();

    let uio_class = Path::new("/sys/class/uio");
    if !uio_class.exists() {
        return Ok(devices);
    }

    if let Ok(entries) = std::fs::read_dir(uio_class) {
        for entry in entries.filter_map(|e| e.ok()) {
            let name = entry.file_name().to_string_lossy().to_string();
            if name.starts_with("uio") {
                match UioDevice::open(&name) {
                    Ok(device) => {
                        tracing::debug!("Found UIO device: {}", name);
                        devices.push(device);
                    }
                    Err(e) => {
                        tracing::warn!("Failed to open UIO device {}: {}", name, e);
                    }
                }
            }
        }
    }

    Ok(devices)
}

/// Find a UIO device by its name property
pub fn find_uio_by_name(name: &str) -> FpgaResult<Option<UioDevice>> {
    let uio_class = Path::new("/sys/class/uio");
    if !uio_class.exists() {
        return Ok(None);
    }

    if let Ok(entries) = std::fs::read_dir(uio_class) {
        for entry in entries.filter_map(|e| e.ok()) {
            let uio_name = entry.file_name().to_string_lossy().to_string();

            // Check the name property
            let name_path = entry.path().join("name");
            if let Ok(dev_name) = std::fs::read_to_string(&name_path) {
                if dev_name.trim() == name {
                    return UioDevice::open(&uio_name)
                        .map(Some)
                        .map_err(|e| FpgaError::OpenFailed(e));
                }
            }
        }
    }

    Ok(None)
}

/// UIO-based interrupt handler that can be polled
pub struct UioInterruptHandler {
    devices: Vec<UioDevice>,
    poll_fds: Vec<libc::pollfd>,
}

impl UioInterruptHandler {
    /// Create a new interrupt handler
    pub fn new(devices: Vec<UioDevice>) -> Self {
        let poll_fds: Vec<libc::pollfd> = devices
            .iter()
            .map(|d| libc::pollfd {
                fd: d.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            })
            .collect();

        Self { devices, poll_fds }
    }

    /// Enable all interrupts
    pub fn enable_all(&mut self) -> io::Result<()> {
        for device in &mut self.devices {
            device.enable_interrupt()?;
        }
        Ok(())
    }

    /// Wait for any interrupt
    pub fn wait_any(&mut self, timeout_ms: i32) -> io::Result<Option<usize>> {
        let result = unsafe {
            libc::poll(self.poll_fds.as_mut_ptr(), self.poll_fds.len() as u64, timeout_ms)
        };

        if result < 0 {
            return Err(io::Error::last_os_error());
        }

        if result == 0 {
            return Ok(None); // Timeout
        }

        // Find which device had an interrupt
        for (i, pfd) in self.poll_fds.iter().enumerate() {
            if pfd.revents & libc::POLLIN != 0 {
                // Read to acknowledge
                let _ = self.devices[i].wait_interrupt();
                return Ok(Some(i));
            }
        }

        Ok(None)
    }

    /// Get device reference by index
    pub fn device(&self, index: usize) -> Option<&UioDevice> {
        self.devices.get(index)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_discover_uio_devices() {
        // This will just return empty on systems without UIO
        let result = discover_uio_devices();
        assert!(result.is_ok());
    }
}
