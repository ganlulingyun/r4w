//! # Pre-Allocated Buffer Pool
//!
//! Provides pre-allocated buffers to avoid runtime memory allocation
//! in real-time signal processing paths.
//!
//! ## Design
//!
//! - Fixed number of fixed-size buffers allocated at creation
//! - Lock-free buffer acquisition using atomic operations
//! - Buffers automatically returned to pool when handles are dropped
//! - Optional memory locking to prevent page faults
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::rt::BufferPool;
//!
//! // Create a pool of 4 buffers, each holding 1024 f32 samples
//! let pool: BufferPool<f32> = BufferPool::new(4, 1024);
//!
//! // Acquire a buffer
//! let mut buf = pool.acquire().unwrap();
//! buf.as_mut_slice()[0] = 1.0;
//!
//! // Buffer is returned to pool when `buf` is dropped
//! ```

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

/// Error type for buffer pool operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PoolError {
    /// No buffers available in the pool
    Exhausted,
    /// Pool configuration is invalid
    InvalidConfig,
}

impl std::fmt::Display for PoolError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PoolError::Exhausted => write!(f, "buffer pool exhausted"),
            PoolError::InvalidConfig => write!(f, "invalid pool configuration"),
        }
    }
}

impl std::error::Error for PoolError {}

/// Internal pool state shared between pool and handles.
struct PoolInner<T> {
    /// The actual buffer storage
    buffers: Vec<Vec<T>>,
    /// Bitmap tracking which buffers are available (1 = available)
    available: AtomicU64,
    /// Number of buffers
    count: usize,
    /// Size of each buffer
    buffer_size: usize,
}

/// A pre-allocated pool of fixed-size buffers.
///
/// Buffers are acquired with `acquire()` and automatically returned
/// to the pool when the `BufferHandle` is dropped.
///
/// # Limits
///
/// Maximum 64 buffers (due to bitmap implementation).
pub struct BufferPool<T> {
    inner: Arc<PoolInner<T>>,
}

impl<T: Default + Clone> BufferPool<T> {
    /// Create a new buffer pool.
    ///
    /// # Arguments
    ///
    /// * `count` - Number of buffers (max 64)
    /// * `buffer_size` - Size of each buffer in elements
    ///
    /// # Example
    ///
    /// ```rust
    /// use r4w_core::rt::BufferPool;
    ///
    /// let pool: BufferPool<f32> = BufferPool::new(8, 256);
    /// assert_eq!(pool.buffer_count(), 8);
    /// assert_eq!(pool.buffer_size(), 256);
    /// ```
    pub fn new(count: usize, buffer_size: usize) -> Self {
        let count = count.min(64).max(1);

        // Pre-allocate all buffers
        let buffers: Vec<Vec<T>> = (0..count)
            .map(|_| vec![T::default(); buffer_size])
            .collect();

        // All buffers start as available
        let available = if count == 64 {
            AtomicU64::new(u64::MAX)
        } else {
            AtomicU64::new((1u64 << count) - 1)
        };

        Self {
            inner: Arc::new(PoolInner {
                buffers,
                available,
                count,
                buffer_size,
            }),
        }
    }

    /// Get the number of buffers in the pool.
    #[inline]
    pub fn buffer_count(&self) -> usize {
        self.inner.count
    }

    /// Get the size of each buffer.
    #[inline]
    pub fn buffer_size(&self) -> usize {
        self.inner.buffer_size
    }

    /// Get the number of available buffers.
    pub fn available(&self) -> usize {
        self.inner.available.load(Ordering::Relaxed).count_ones() as usize
    }

    /// Check if all buffers are in use.
    #[inline]
    pub fn is_exhausted(&self) -> bool {
        self.available() == 0
    }

    /// Try to acquire a buffer from the pool.
    ///
    /// Returns `None` if no buffers are available.
    pub fn try_acquire(&self) -> Option<BufferHandle<T>> {
        loop {
            let available = self.inner.available.load(Ordering::Acquire);
            if available == 0 {
                return None;
            }

            // Find first available buffer
            let index = available.trailing_zeros() as usize;
            let mask = 1u64 << index;

            // Try to claim it
            match self.inner.available.compare_exchange_weak(
                available,
                available & !mask,
                Ordering::AcqRel,
                Ordering::Relaxed,
            ) {
                Ok(_) => {
                    // Successfully acquired
                    return Some(BufferHandle {
                        pool: Arc::clone(&self.inner),
                        index,
                    });
                }
                Err(_) => {
                    // Another thread grabbed it, retry
                    continue;
                }
            }
        }
    }

    /// Acquire a buffer, returning an error if none available.
    pub fn acquire(&self) -> Result<BufferHandle<T>, PoolError> {
        self.try_acquire().ok_or(PoolError::Exhausted)
    }

    /// Acquire a buffer, spinning until one is available.
    ///
    /// Use with caution in RT contexts - this will busy-wait.
    pub fn acquire_blocking(&self) -> BufferHandle<T> {
        loop {
            if let Some(handle) = self.try_acquire() {
                return handle;
            }
            std::hint::spin_loop();
        }
    }
}

impl<T> Clone for BufferPool<T> {
    fn clone(&self) -> Self {
        Self {
            inner: Arc::clone(&self.inner),
        }
    }
}

// Safety: Pool is thread-safe due to atomic operations
unsafe impl<T: Send> Send for BufferPool<T> {}
unsafe impl<T: Send> Sync for BufferPool<T> {}

/// Handle to a buffer from a pool.
///
/// The buffer is automatically returned to the pool when this handle is dropped.
/// Access the buffer contents via `as_slice()` or `as_mut_slice()`.
pub struct BufferHandle<T> {
    pool: Arc<PoolInner<T>>,
    index: usize,
}

impl<T> BufferHandle<T> {
    /// Get the buffer as a slice.
    #[inline]
    pub fn as_slice(&self) -> &[T] {
        &self.pool.buffers[self.index]
    }

    /// Get the buffer as a mutable slice.
    ///
    /// # Safety
    ///
    /// The handle holds exclusive access to this buffer, so mutable
    /// access is safe even though the underlying storage is shared.
    #[inline]
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        // Safety: We have exclusive access to this buffer index
        unsafe {
            let ptr = self.pool.buffers.as_ptr().add(self.index) as *mut Vec<T>;
            (*ptr).as_mut_slice()
        }
    }

    /// Get the buffer length.
    #[inline]
    pub fn len(&self) -> usize {
        self.pool.buffer_size
    }

    /// Check if buffer is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.pool.buffer_size == 0
    }

    /// Get the buffer index in the pool.
    #[inline]
    pub fn index(&self) -> usize {
        self.index
    }
}

impl<T> Drop for BufferHandle<T> {
    fn drop(&mut self) {
        // Return buffer to pool
        let mask = 1u64 << self.index;
        self.pool.available.fetch_or(mask, Ordering::Release);
    }
}

impl<T> std::ops::Deref for BufferHandle<T> {
    type Target = [T];

    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl<T> std::ops::DerefMut for BufferHandle<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.as_mut_slice()
    }
}

// Safety: Handle provides exclusive access to its buffer
unsafe impl<T: Send> Send for BufferHandle<T> {}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    #[test]
    fn test_pool_creation() {
        let pool: BufferPool<f32> = BufferPool::new(4, 1024);
        assert_eq!(pool.buffer_count(), 4);
        assert_eq!(pool.buffer_size(), 1024);
        assert_eq!(pool.available(), 4);
    }

    #[test]
    fn test_acquire_release() {
        let pool: BufferPool<i32> = BufferPool::new(2, 10);

        assert_eq!(pool.available(), 2);

        {
            let buf1 = pool.acquire().unwrap();
            assert_eq!(pool.available(), 1);
            assert_eq!(buf1.len(), 10);

            let _buf2 = pool.acquire().unwrap();
            assert_eq!(pool.available(), 0);

            // Pool should be exhausted
            assert!(pool.try_acquire().is_none());
            assert!(matches!(pool.acquire(), Err(PoolError::Exhausted)));

            drop(buf1);
            assert_eq!(pool.available(), 1);
        }

        // Both released
        assert_eq!(pool.available(), 2);
    }

    #[test]
    fn test_buffer_access() {
        let pool: BufferPool<u32> = BufferPool::new(1, 100);

        let mut buf = pool.acquire().unwrap();

        // Write to buffer
        buf.as_mut_slice()[0] = 42;
        buf.as_mut_slice()[99] = 99;

        assert_eq!(buf.as_slice()[0], 42);
        assert_eq!(buf.as_slice()[99], 99);

        // Deref trait
        assert_eq!(buf[0], 42);
    }

    #[test]
    fn test_pool_threaded() {
        let pool = BufferPool::<u64>::new(4, 256);
        let pool1 = pool.clone();
        let pool2 = pool.clone();

        let t1 = thread::spawn(move || {
            for _ in 0..1000 {
                let mut buf = pool1.acquire_blocking();
                buf[0] = 1;
                // Hold briefly
                std::hint::spin_loop();
            }
        });

        let t2 = thread::spawn(move || {
            for _ in 0..1000 {
                let mut buf = pool2.acquire_blocking();
                buf[0] = 2;
                // Hold briefly
                std::hint::spin_loop();
            }
        });

        t1.join().unwrap();
        t2.join().unwrap();

        assert_eq!(pool.available(), 4);
    }

    #[test]
    fn test_max_buffers() {
        let pool: BufferPool<u8> = BufferPool::new(64, 16);
        assert_eq!(pool.buffer_count(), 64);

        // Acquire all 64
        let handles: Vec<_> = (0..64)
            .map(|_| pool.acquire().unwrap())
            .collect();

        assert!(pool.is_exhausted());
        assert_eq!(pool.available(), 0);

        drop(handles);
        assert_eq!(pool.available(), 64);
    }

    #[test]
    fn test_buffer_index() {
        let pool: BufferPool<i32> = BufferPool::new(4, 10);

        let buf0 = pool.acquire().unwrap();
        let buf1 = pool.acquire().unwrap();

        // Indices should be different
        assert_ne!(buf0.index(), buf1.index());

        // Indices should be valid
        assert!(buf0.index() < 4);
        assert!(buf1.index() < 4);
    }
}
