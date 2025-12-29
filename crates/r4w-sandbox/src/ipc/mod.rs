//! Inter-process communication for isolated waveforms.
//!
//! This module provides secure IPC mechanisms for communication
//! between sandboxed waveform processes, including:
//!
//! - Shared memory channels for I/Q samples
//! - Unix domain sockets for control messages
//! - Message queues for event notification

use crate::error::{Result, SandboxError};
use std::os::unix::net::UnixStream;

/// A shared memory channel for passing I/Q samples between processes.
///
/// Uses POSIX shared memory for zero-copy sample transfer between
/// isolated waveform processes.
pub struct SharedMemoryChannel {
    #[allow(dead_code)]
    name: String,
    size: usize,
    #[cfg(target_os = "linux")]
    #[allow(dead_code)]
    fd: std::os::fd::OwnedFd,
    ptr: *mut u8,
}

impl SharedMemoryChannel {
    /// Create a new shared memory channel.
    ///
    /// # Arguments
    /// * `name` - Unique name for the shared memory segment (e.g., "/r4w-samples-0")
    /// * `size` - Size in bytes
    /// * `create` - True to create, false to attach to existing
    #[cfg(target_os = "linux")]
    pub fn new(name: &str, size: usize, create: bool) -> Result<Self> {
        use nix::fcntl::OFlag;
        use nix::sys::mman::{mmap, shm_open, MapFlags, ProtFlags};
        use nix::sys::stat::Mode;

        let flags = if create {
            OFlag::O_RDWR | OFlag::O_CREAT | OFlag::O_EXCL
        } else {
            OFlag::O_RDWR
        };

        // Create/open shared memory object
        let fd = shm_open(name, flags, Mode::S_IRUSR | Mode::S_IWUSR)
            .map_err(|e| SandboxError::IpcError(format!("shm_open failed: {}", e)))?;

        // Set size if creating
        if create {
            nix::unistd::ftruncate(&fd, size as i64)
                .map_err(|e| SandboxError::IpcError(format!("ftruncate failed: {}", e)))?;
        }

        // Map into address space
        let ptr = unsafe {
            mmap(
                None,
                std::num::NonZeroUsize::new(size).unwrap(),
                ProtFlags::PROT_READ | ProtFlags::PROT_WRITE,
                MapFlags::MAP_SHARED,
                &fd,
                0,
            )
            .map_err(|e| SandboxError::IpcError(format!("mmap failed: {}", e)))?
        };

        Ok(Self {
            name: name.to_string(),
            size,
            fd,
            ptr: ptr.as_ptr() as *mut u8,
        })
    }

    #[cfg(not(target_os = "linux"))]
    pub fn new(_name: &str, _size: usize, _create: bool) -> Result<Self> {
        Err(SandboxError::IpcError(
            "shared memory not available on this platform".to_string(),
        ))
    }

    /// Get a slice to read from the shared memory.
    pub fn as_slice(&self) -> &[u8] {
        unsafe { std::slice::from_raw_parts(self.ptr, self.size) }
    }

    /// Get a mutable slice to write to the shared memory.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        unsafe { std::slice::from_raw_parts_mut(self.ptr, self.size) }
    }

    /// Get the size of the channel.
    pub fn size(&self) -> usize {
        self.size
    }

    /// Get the name of the channel.
    pub fn name(&self) -> &str {
        &self.name
    }
}

impl Drop for SharedMemoryChannel {
    fn drop(&mut self) {
        #[cfg(target_os = "linux")]
        unsafe {
            // Unmap memory (fd is closed automatically via OwnedFd drop)
            libc::munmap(self.ptr as *mut libc::c_void, self.size);
            // Optionally unlink (only creator should do this)
            // nix::sys::mman::shm_unlink(&self.name).ok();
        }
    }
}

// SharedMemoryChannel is Send + Sync
unsafe impl Send for SharedMemoryChannel {}
unsafe impl Sync for SharedMemoryChannel {}

/// A ring buffer in shared memory for I/Q sample streaming.
pub struct SampleRingBuffer {
    channel: SharedMemoryChannel,
    /// Capacity in samples
    capacity: usize,
}

/// Ring buffer header stored at the start of shared memory
#[repr(C)]
struct RingBufferHeader {
    /// Write position (producer)
    write_pos: std::sync::atomic::AtomicUsize,
    /// Read position (consumer)
    read_pos: std::sync::atomic::AtomicUsize,
    /// Capacity in samples
    capacity: usize,
    /// Sample size in bytes
    sample_size: usize,
}

impl SampleRingBuffer {
    /// Create a new ring buffer for I/Q samples.
    ///
    /// # Arguments
    /// * `name` - Shared memory name
    /// * `capacity` - Number of samples
    /// * `sample_size` - Size of each sample in bytes
    /// * `create` - True to create, false to attach
    pub fn new(name: &str, capacity: usize, sample_size: usize, create: bool) -> Result<Self> {
        let header_size = std::mem::size_of::<RingBufferHeader>();
        let data_size = capacity * sample_size;
        let total_size = header_size + data_size;

        let mut channel = SharedMemoryChannel::new(name, total_size, create)?;

        if create {
            // Initialize header
            let header = channel.as_mut_slice().as_mut_ptr() as *mut RingBufferHeader;
            unsafe {
                (*header).write_pos = std::sync::atomic::AtomicUsize::new(0);
                (*header).read_pos = std::sync::atomic::AtomicUsize::new(0);
                (*header).capacity = capacity;
                (*header).sample_size = sample_size;
            }
        }

        Ok(Self { channel, capacity })
    }

    /// Get the capacity in samples.
    pub fn capacity(&self) -> usize {
        self.capacity
    }

    /// Get the number of samples available to read.
    pub fn available(&self) -> usize {
        let header = self.header();
        let write = header.write_pos.load(std::sync::atomic::Ordering::Acquire);
        let read = header.read_pos.load(std::sync::atomic::Ordering::Acquire);

        if write >= read {
            write - read
        } else {
            self.capacity - read + write
        }
    }

    /// Get the number of free slots for writing.
    pub fn free(&self) -> usize {
        self.capacity - self.available() - 1 // -1 to distinguish full from empty
    }

    fn header(&self) -> &RingBufferHeader {
        unsafe { &*(self.channel.as_slice().as_ptr() as *const RingBufferHeader) }
    }
}

/// A Unix domain socket pair for control messages between sandboxes.
pub struct ControlChannel {
    socket: UnixStream,
}

impl ControlChannel {
    /// Create a connected pair of control channels.
    pub fn pair() -> Result<(Self, Self)> {
        let (a, b) = UnixStream::pair()
            .map_err(|e| SandboxError::IpcError(format!("socketpair failed: {}", e)))?;

        Ok((Self { socket: a }, Self { socket: b }))
    }

    /// Connect to an existing control socket path.
    pub fn connect(path: &str) -> Result<Self> {
        let socket = UnixStream::connect(path)
            .map_err(|e| SandboxError::IpcError(format!("connect failed: {}", e)))?;

        Ok(Self { socket })
    }

    /// Send a control message.
    pub fn send(&mut self, msg: &ControlMessage) -> Result<()> {
        use std::io::Write;

        let data = serde_json::to_vec(msg)
            .map_err(|e| SandboxError::IpcError(format!("serialize failed: {}", e)))?;

        // Write length prefix
        let len = data.len() as u32;
        self.socket
            .write_all(&len.to_le_bytes())
            .map_err(|e| SandboxError::IpcError(format!("write failed: {}", e)))?;

        // Write data
        self.socket
            .write_all(&data)
            .map_err(|e| SandboxError::IpcError(format!("write failed: {}", e)))?;

        Ok(())
    }

    /// Receive a control message.
    pub fn recv(&mut self) -> Result<ControlMessage> {
        use std::io::Read;

        // Read length prefix
        let mut len_buf = [0u8; 4];
        self.socket
            .read_exact(&mut len_buf)
            .map_err(|e| SandboxError::IpcError(format!("read failed: {}", e)))?;
        let len = u32::from_le_bytes(len_buf) as usize;

        // Sanity check
        if len > 1024 * 1024 {
            return Err(SandboxError::IpcError("message too large".to_string()));
        }

        // Read data
        let mut data = vec![0u8; len];
        self.socket
            .read_exact(&mut data)
            .map_err(|e| SandboxError::IpcError(format!("read failed: {}", e)))?;

        serde_json::from_slice(&data)
            .map_err(|e| SandboxError::IpcError(format!("deserialize failed: {}", e)))
    }

    /// Get the underlying socket for async operations.
    pub fn into_socket(self) -> UnixStream {
        self.socket
    }
}

/// Control messages between sandboxed waveforms.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum ControlMessage {
    /// Start processing
    Start,
    /// Stop processing
    Stop,
    /// Pause processing
    Pause,
    /// Resume processing
    Resume,
    /// Configuration update
    Configure(serde_json::Value),
    /// Status request
    StatusRequest,
    /// Status response
    StatusResponse(WaveformStatus),
    /// Error notification
    Error(String),
    /// Shutdown request
    Shutdown,
}

/// Status of a sandboxed waveform.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct WaveformStatus {
    /// Waveform name
    pub waveform: String,
    /// Current state
    pub state: String,
    /// Samples processed
    pub samples_processed: u64,
    /// Errors encountered
    pub errors: u64,
    /// CPU usage percentage
    pub cpu_percent: f32,
    /// Memory usage in bytes
    pub memory_bytes: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_control_channel_pair() {
        let (mut a, mut b) = ControlChannel::pair().unwrap();

        // Send from a to b
        a.send(&ControlMessage::Start).unwrap();
        let msg = b.recv().unwrap();
        assert!(matches!(msg, ControlMessage::Start));

        // Send from b to a
        b.send(&ControlMessage::Stop).unwrap();
        let msg = a.recv().unwrap();
        assert!(matches!(msg, ControlMessage::Stop));
    }

    #[test]
    fn test_control_message_serialization() {
        let status = WaveformStatus {
            waveform: "BPSK".to_string(),
            state: "running".to_string(),
            samples_processed: 1000000,
            errors: 0,
            cpu_percent: 45.2,
            memory_bytes: 128 * 1024 * 1024,
        };

        let msg = ControlMessage::StatusResponse(status.clone());
        let json = serde_json::to_string(&msg).unwrap();
        let parsed: ControlMessage = serde_json::from_str(&json).unwrap();

        if let ControlMessage::StatusResponse(s) = parsed {
            assert_eq!(s.waveform, "BPSK");
            assert_eq!(s.samples_processed, 1000000);
        } else {
            panic!("wrong message type");
        }
    }
}
