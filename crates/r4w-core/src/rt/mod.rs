//! # Real-Time Primitives
//!
//! This module provides real-time safe primitives for SDR applications:
//!
//! - **Lock-free ring buffers**: SPSC queues for producer/consumer patterns
//! - **Buffer pools**: Pre-allocated memory to avoid runtime allocation
//! - **RT thread spawning**: Thread priority and CPU affinity configuration
//! - **Memory locking**: Prevent page faults in RT paths
//!
//! ## Design Principles
//!
//! 1. **No allocation in hot path**: All memory pre-allocated
//! 2. **Lock-free data structures**: No mutexes in streaming paths
//! 3. **Bounded latency**: Worst-case guarantees where possible
//! 4. **Cache-friendly**: Data structures aligned to cache lines
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::rt::{RingBuffer, BufferPool};
//!
//! // Create a ring buffer for streaming samples
//! let ring: RingBuffer<f32> = RingBuffer::new(1024);
//!
//! // Producer thread
//! ring.push(1.0).unwrap();
//! ring.push_slice(&[2.0, 3.0, 4.0]);
//!
//! // Consumer thread
//! let sample = ring.pop().unwrap();
//! let mut buf = [0.0f32; 3];
//! ring.pop_slice(&mut buf);
//! ```

pub mod alloc_audit;
mod latency;
mod pool;
mod ringbuffer;
mod stats;
mod thread;

pub use alloc_audit::{AllocationTracker, AuditResult, audit, print_audit_summary};
pub use latency::{LatencyHistogram, LatencyPercentiles, LatencyStatistics};
pub use pool::{BufferHandle, BufferPool, PoolError};
pub use ringbuffer::{RingBuffer, RingError};
pub use stats::{ProcessingTimer, RtStats, RtStatsSnapshot, Watermarks};
pub use thread::{RtConfig, RtError, RtPriority, spawn_rt_thread};

use std::alloc::{alloc, dealloc, Layout};
use std::ptr;

/// Cache line size for alignment (64 bytes on most x86/ARM platforms).
pub const CACHE_LINE_SIZE: usize = 64;

/// Align a value up to the nearest multiple of alignment.
#[inline]
pub const fn align_up(value: usize, alignment: usize) -> usize {
    (value + alignment - 1) & !(alignment - 1)
}

/// Memory-locked buffer that won't be paged out.
///
/// On Linux, uses `mlock()` to prevent page faults during RT operation.
/// Falls back to regular allocation on unsupported platforms.
#[derive(Debug)]
pub struct LockedBuffer<T> {
    ptr: *mut T,
    len: usize,
    layout: Layout,
    locked: bool,
}

impl<T> LockedBuffer<T> {
    /// Allocate a new locked buffer.
    ///
    /// The buffer is aligned to cache line size and memory-locked if supported.
    pub fn new(len: usize) -> Option<Self> {
        if len == 0 {
            return None;
        }

        let size = len * std::mem::size_of::<T>();
        let layout = Layout::from_size_align(size, CACHE_LINE_SIZE).ok()?;

        let ptr = unsafe { alloc(layout) as *mut T };
        if ptr.is_null() {
            return None;
        }

        // Zero-initialize
        unsafe {
            ptr::write_bytes(ptr, 0, len);
        }

        // Try to lock memory (Linux-specific)
        let locked = Self::try_mlock(ptr, size);

        Some(Self {
            ptr,
            len,
            layout,
            locked,
        })
    }

    /// Check if memory is locked.
    #[inline]
    pub fn is_locked(&self) -> bool {
        self.locked
    }

    /// Get buffer length.
    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Check if buffer is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Get a slice of the buffer.
    #[inline]
    pub fn as_slice(&self) -> &[T] {
        unsafe { std::slice::from_raw_parts(self.ptr, self.len) }
    }

    /// Get a mutable slice of the buffer.
    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        unsafe { std::slice::from_raw_parts_mut(self.ptr, self.len) }
    }

    /// Get raw pointer.
    #[inline]
    pub fn as_ptr(&self) -> *const T {
        self.ptr
    }

    /// Get raw mutable pointer.
    #[inline]
    pub fn as_mut_ptr(&mut self) -> *mut T {
        self.ptr
    }

    #[cfg(target_os = "linux")]
    fn try_mlock(ptr: *mut T, size: usize) -> bool {
        unsafe { libc::mlock(ptr as *const libc::c_void, size) == 0 }
    }

    #[cfg(not(target_os = "linux"))]
    fn try_mlock(_ptr: *mut T, _size: usize) -> bool {
        // Memory locking not supported on this platform
        false
    }

    #[cfg(target_os = "linux")]
    fn try_munlock(ptr: *mut T, size: usize) {
        unsafe {
            libc::munlock(ptr as *const libc::c_void, size);
        }
    }

    #[cfg(not(target_os = "linux"))]
    fn try_munlock(_ptr: *mut T, _size: usize) {}
}

impl<T> Drop for LockedBuffer<T> {
    fn drop(&mut self) {
        if self.locked {
            let size = self.len * std::mem::size_of::<T>();
            Self::try_munlock(self.ptr, size);
        }
        unsafe {
            dealloc(self.ptr as *mut u8, self.layout);
        }
    }
}

// LockedBuffer is Send + Sync if T is
unsafe impl<T: Send> Send for LockedBuffer<T> {}
unsafe impl<T: Sync> Sync for LockedBuffer<T> {}

/// Zero a slice of memory securely.
///
/// Uses volatile writes to prevent compiler optimization.
pub fn secure_zero<T: Copy + Default>(slice: &mut [T]) {
    for item in slice.iter_mut() {
        unsafe {
            std::ptr::write_volatile(item, T::default());
        }
    }
    std::sync::atomic::compiler_fence(std::sync::atomic::Ordering::SeqCst);
}

/// Pre-touch memory pages to avoid page faults on first access.
///
/// Useful for pre-allocating buffers before entering RT context.
pub fn prefault_pages<T>(slice: &mut [T]) {
    let page_size = 4096; // Typical page size
    let elem_size = std::mem::size_of::<T>();
    let elements_per_page = page_size / elem_size.max(1);

    for i in (0..slice.len()).step_by(elements_per_page.max(1)) {
        unsafe {
            let ptr = slice.as_mut_ptr().add(i);
            std::ptr::write_volatile(ptr as *mut u8, 0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_align_up() {
        assert_eq!(align_up(0, 64), 0);
        assert_eq!(align_up(1, 64), 64);
        assert_eq!(align_up(64, 64), 64);
        assert_eq!(align_up(65, 64), 128);
    }

    #[test]
    fn test_locked_buffer() {
        let buf: LockedBuffer<f32> = LockedBuffer::new(1024).unwrap();
        assert_eq!(buf.len(), 1024);
        assert!(!buf.is_empty());
        // May or may not be locked depending on permissions
    }

    #[test]
    fn test_locked_buffer_access() {
        let mut buf: LockedBuffer<u32> = LockedBuffer::new(100).unwrap();
        let slice = buf.as_mut_slice();
        slice[0] = 42;
        slice[99] = 99;

        let slice = buf.as_slice();
        assert_eq!(slice[0], 42);
        assert_eq!(slice[99], 99);
    }

    #[test]
    fn test_secure_zero() {
        let mut data = [1u8, 2, 3, 4, 5];
        secure_zero(&mut data);
        assert!(data.iter().all(|&x| x == 0));
    }

    #[test]
    fn test_prefault_pages() {
        let mut data = vec![0u8; 8192]; // 2 pages
        prefault_pages(&mut data);
        // Should not panic
    }
}
