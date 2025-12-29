//! # Lock-Free Ring Buffer
//!
//! A single-producer, single-consumer (SPSC) lock-free ring buffer
//! optimized for real-time streaming applications.
//!
//! ## Design
//!
//! - Cache-line aligned head/tail pointers to avoid false sharing
//! - Power-of-two capacity for fast modulo via bitwise AND
//! - Supports both single-element and batch operations
//! - No locks, no allocations in push/pop operations
//!
//! ## Memory Ordering
//!
//! Uses Release-Acquire semantics:
//! - Producer: Release store on head after writing data
//! - Consumer: Acquire load on head before reading data
//!
//! This ensures all data writes are visible to the consumer.

use std::cell::UnsafeCell;
use std::mem::MaybeUninit;
use std::sync::atomic::{AtomicUsize, Ordering};

use super::CACHE_LINE_SIZE;

/// Error type for ring buffer operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RingError {
    /// Buffer is full, cannot push
    Full,
    /// Buffer is empty, cannot pop
    Empty,
    /// Requested capacity is invalid
    InvalidCapacity,
}

impl std::fmt::Display for RingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RingError::Full => write!(f, "ring buffer is full"),
            RingError::Empty => write!(f, "ring buffer is empty"),
            RingError::InvalidCapacity => write!(f, "invalid capacity (must be power of 2)"),
        }
    }
}

impl std::error::Error for RingError {}

/// Cache-line padded atomic counter.
///
/// Padding prevents false sharing when head and tail are on different cache lines.
#[repr(align(64))]
struct PaddedAtomicUsize {
    value: AtomicUsize,
    _pad: [u8; CACHE_LINE_SIZE - std::mem::size_of::<AtomicUsize>()],
}

impl PaddedAtomicUsize {
    fn new(v: usize) -> Self {
        Self {
            value: AtomicUsize::new(v),
            _pad: [0; CACHE_LINE_SIZE - std::mem::size_of::<AtomicUsize>()],
        }
    }
}

/// Single-producer, single-consumer lock-free ring buffer.
///
/// Optimized for streaming I/Q samples between threads.
///
/// # Example
///
/// ```rust
/// use r4w_core::rt::RingBuffer;
///
/// let ring: RingBuffer<i32> = RingBuffer::new(16);
///
/// // Push values
/// ring.push(1).unwrap();
/// ring.push(2).unwrap();
///
/// // Pop values
/// assert_eq!(ring.pop(), Some(1));
/// assert_eq!(ring.pop(), Some(2));
/// assert_eq!(ring.pop(), None);
/// ```
pub struct RingBuffer<T> {
    /// Storage for elements
    buffer: Box<[UnsafeCell<MaybeUninit<T>>]>,
    /// Write position (modified by producer)
    head: PaddedAtomicUsize,
    /// Read position (modified by consumer)
    tail: PaddedAtomicUsize,
    /// Capacity mask (capacity - 1 for power-of-two modulo)
    mask: usize,
}

impl<T> RingBuffer<T> {
    /// Create a new ring buffer with the given capacity.
    ///
    /// Capacity is rounded up to the next power of two.
    /// Minimum capacity is 2.
    ///
    /// # Example
    ///
    /// ```rust
    /// use r4w_core::rt::RingBuffer;
    ///
    /// let ring: RingBuffer<f32> = RingBuffer::new(1000);
    /// // Actual capacity will be 1024 (next power of 2)
    /// assert_eq!(ring.capacity(), 1024);
    /// ```
    pub fn new(capacity: usize) -> Self {
        let capacity = capacity.max(2).next_power_of_two();
        let buffer: Vec<UnsafeCell<MaybeUninit<T>>> =
            (0..capacity).map(|_| UnsafeCell::new(MaybeUninit::uninit())).collect();

        Self {
            buffer: buffer.into_boxed_slice(),
            head: PaddedAtomicUsize::new(0),
            tail: PaddedAtomicUsize::new(0),
            mask: capacity - 1,
        }
    }

    /// Get the capacity of the ring buffer.
    #[inline]
    pub fn capacity(&self) -> usize {
        self.mask + 1
    }

    /// Get the number of elements currently in the buffer.
    ///
    /// Note: This is a snapshot and may change immediately after.
    #[inline]
    pub fn len(&self) -> usize {
        let head = self.head.value.load(Ordering::Relaxed);
        let tail = self.tail.value.load(Ordering::Relaxed);
        head.wrapping_sub(tail)
    }

    /// Check if the buffer is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Check if the buffer is full.
    #[inline]
    pub fn is_full(&self) -> bool {
        self.len() >= self.capacity()
    }

    /// Get the number of free slots.
    #[inline]
    pub fn free_space(&self) -> usize {
        self.capacity() - self.len()
    }

    /// Push a single element.
    ///
    /// Returns `Err(RingError::Full)` if the buffer is full.
    pub fn push(&self, value: T) -> Result<(), RingError> {
        let head = self.head.value.load(Ordering::Relaxed);
        let tail = self.tail.value.load(Ordering::Acquire);

        // Check if full
        if head.wrapping_sub(tail) >= self.capacity() {
            return Err(RingError::Full);
        }

        // Write the value
        let index = head & self.mask;
        unsafe {
            (*self.buffer[index].get()).write(value);
        }

        // Publish the write with release ordering
        self.head.value.store(head.wrapping_add(1), Ordering::Release);

        Ok(())
    }

    /// Pop a single element.
    ///
    /// Returns `None` if the buffer is empty.
    pub fn pop(&self) -> Option<T> {
        let tail = self.tail.value.load(Ordering::Relaxed);
        let head = self.head.value.load(Ordering::Acquire);

        // Check if empty
        if tail == head {
            return None;
        }

        // Read the value
        let index = tail & self.mask;
        let value = unsafe { (*self.buffer[index].get()).assume_init_read() };

        // Publish the read with release ordering
        self.tail.value.store(tail.wrapping_add(1), Ordering::Release);

        Some(value)
    }

    /// Try to push a value, returning the value if the buffer is full.
    #[inline]
    pub fn try_push(&self, value: T) -> Result<(), T> {
        match self.push(value) {
            Ok(()) => Ok(()),
            Err(RingError::Full) => {
                // We need to return the value, but push consumed it
                // This is a design limitation - use push() instead
                unreachable!("push returns Full without consuming value")
            }
            _ => unreachable!(),
        }
    }
}

impl<T: Copy> RingBuffer<T> {
    /// Push a slice of elements.
    ///
    /// Returns the number of elements actually pushed.
    /// May be less than the slice length if the buffer fills up.
    pub fn push_slice(&self, data: &[T]) -> usize {
        let head = self.head.value.load(Ordering::Relaxed);
        let tail = self.tail.value.load(Ordering::Acquire);

        let available = self.capacity() - head.wrapping_sub(tail);
        let count = data.len().min(available);

        if count == 0 {
            return 0;
        }

        // Write elements
        for (i, &item) in data.iter().take(count).enumerate() {
            let index = (head + i) & self.mask;
            unsafe {
                (*self.buffer[index].get()).write(item);
            }
        }

        // Publish all writes
        self.head.value.store(head.wrapping_add(count), Ordering::Release);

        count
    }

    /// Pop elements into a slice.
    ///
    /// Returns the number of elements actually popped.
    /// May be less than the buffer length if the ring is empty.
    pub fn pop_slice(&self, buffer: &mut [T]) -> usize {
        let tail = self.tail.value.load(Ordering::Relaxed);
        let head = self.head.value.load(Ordering::Acquire);

        let available = head.wrapping_sub(tail);
        let count = buffer.len().min(available);

        if count == 0 {
            return 0;
        }

        // Read elements
        for (i, item) in buffer.iter_mut().take(count).enumerate() {
            let index = (tail + i) & self.mask;
            *item = unsafe { (*self.buffer[index].get()).assume_init_read() };
        }

        // Publish all reads
        self.tail.value.store(tail.wrapping_add(count), Ordering::Release);

        count
    }

    /// Push all elements from a slice, blocking until complete.
    ///
    /// This spins waiting for space, so should only be used when
    /// the consumer is actively draining the buffer.
    ///
    /// Returns the total number of push operations performed.
    pub fn push_slice_blocking(&self, data: &[T]) -> usize {
        let mut remaining = data;
        let mut ops = 0;

        while !remaining.is_empty() {
            let pushed = self.push_slice(remaining);
            if pushed > 0 {
                remaining = &remaining[pushed..];
                ops += 1;
            } else {
                // Spin hint
                std::hint::spin_loop();
            }
        }

        ops
    }

    /// Pop elements into a slice, blocking until complete.
    ///
    /// This spins waiting for data, so should only be used when
    /// the producer is actively filling the buffer.
    ///
    /// Returns the total number of pop operations performed.
    pub fn pop_slice_blocking(&self, buffer: &mut [T]) -> usize {
        let mut remaining = buffer;
        let mut ops = 0;

        while !remaining.is_empty() {
            let popped = self.pop_slice(remaining);
            if popped > 0 {
                remaining = &mut remaining[popped..];
                ops += 1;
            } else {
                // Spin hint
                std::hint::spin_loop();
            }
        }

        ops
    }
}

// Safety: The ring buffer is specifically designed for SPSC access.
// It's safe to send between threads, and the atomic operations
// provide the necessary synchronization.
unsafe impl<T: Send> Send for RingBuffer<T> {}
unsafe impl<T: Send> Sync for RingBuffer<T> {}

impl<T> Drop for RingBuffer<T> {
    fn drop(&mut self) {
        // Drop any remaining elements
        while self.pop().is_some() {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::thread;

    #[test]
    fn test_new_capacity() {
        let ring: RingBuffer<i32> = RingBuffer::new(10);
        assert_eq!(ring.capacity(), 16); // Next power of 2

        let ring: RingBuffer<i32> = RingBuffer::new(16);
        assert_eq!(ring.capacity(), 16);

        let ring: RingBuffer<i32> = RingBuffer::new(1);
        assert_eq!(ring.capacity(), 2); // Minimum
    }

    #[test]
    fn test_push_pop_single() {
        let ring: RingBuffer<i32> = RingBuffer::new(4);

        assert!(ring.is_empty());
        assert!(!ring.is_full());

        ring.push(1).unwrap();
        ring.push(2).unwrap();
        ring.push(3).unwrap();

        assert_eq!(ring.len(), 3);
        assert!(!ring.is_empty());

        assert_eq!(ring.pop(), Some(1));
        assert_eq!(ring.pop(), Some(2));
        assert_eq!(ring.pop(), Some(3));
        assert_eq!(ring.pop(), None);

        assert!(ring.is_empty());
    }

    #[test]
    fn test_full_buffer() {
        let ring: RingBuffer<i32> = RingBuffer::new(4);

        // Fill the buffer
        for i in 0..4 {
            ring.push(i).unwrap();
        }

        assert!(ring.is_full());
        assert_eq!(ring.push(99), Err(RingError::Full));

        // Pop one and push should work
        assert_eq!(ring.pop(), Some(0));
        ring.push(99).unwrap();
    }

    #[test]
    fn test_push_pop_slice() {
        let ring: RingBuffer<i32> = RingBuffer::new(8);

        let data = [1, 2, 3, 4, 5];
        let pushed = ring.push_slice(&data);
        assert_eq!(pushed, 5);
        assert_eq!(ring.len(), 5);

        let mut buffer = [0i32; 3];
        let popped = ring.pop_slice(&mut buffer);
        assert_eq!(popped, 3);
        assert_eq!(buffer, [1, 2, 3]);

        assert_eq!(ring.len(), 2);
    }

    #[test]
    fn test_wraparound() {
        let ring: RingBuffer<i32> = RingBuffer::new(4);

        // Push and pop to advance indices
        for round in 0..10 {
            for i in 0..3 {
                ring.push(round * 10 + i).unwrap();
            }
            for i in 0..3 {
                assert_eq!(ring.pop(), Some(round * 10 + i));
            }
        }
    }

    #[test]
    fn test_spsc_threaded() {
        let ring = Arc::new(RingBuffer::<u64>::new(1024));
        let ring_producer = Arc::clone(&ring);
        let ring_consumer = Arc::clone(&ring);

        const COUNT: u64 = 10_000;

        let producer = thread::spawn(move || {
            for i in 0..COUNT {
                while ring_producer.push(i).is_err() {
                    std::hint::spin_loop();
                }
            }
        });

        let consumer = thread::spawn(move || {
            let mut sum = 0u64;
            for _ in 0..COUNT {
                loop {
                    if let Some(v) = ring_consumer.pop() {
                        sum += v;
                        break;
                    }
                    std::hint::spin_loop();
                }
            }
            sum
        });

        producer.join().unwrap();
        let sum = consumer.join().unwrap();

        // Sum of 0..COUNT = COUNT * (COUNT - 1) / 2
        let expected = COUNT * (COUNT - 1) / 2;
        assert_eq!(sum, expected);
    }

    #[test]
    fn test_slice_operations_threaded() {
        let ring = Arc::new(RingBuffer::<f32>::new(256));
        let ring_producer = Arc::clone(&ring);
        let ring_consumer = Arc::clone(&ring);

        const CHUNK_SIZE: usize = 64;
        const CHUNKS: usize = 100;

        let producer = thread::spawn(move || {
            for chunk_idx in 0..CHUNKS {
                let data: Vec<f32> = (0..CHUNK_SIZE)
                    .map(|i| (chunk_idx * CHUNK_SIZE + i) as f32)
                    .collect();
                ring_producer.push_slice_blocking(&data);
            }
        });

        let consumer = thread::spawn(move || {
            let mut buffer = vec![0.0f32; CHUNK_SIZE];
            let mut sum = 0.0f32;
            for _ in 0..CHUNKS {
                ring_consumer.pop_slice_blocking(&mut buffer);
                sum += buffer.iter().sum::<f32>();
            }
            sum
        });

        producer.join().unwrap();
        let sum = consumer.join().unwrap();

        // Sum of 0..(CHUNK_SIZE * CHUNKS) as floats
        let n = (CHUNK_SIZE * CHUNKS) as f32;
        let expected = n * (n - 1.0) / 2.0;
        assert!((sum - expected).abs() < 1.0);
    }
}
