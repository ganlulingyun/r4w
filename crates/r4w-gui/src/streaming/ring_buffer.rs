//! Efficient ring buffer for sliding window visualization

use r4w_core::types::IQSample;

/// Efficient ring buffer for sliding window visualization.
/// Avoids allocations in the hot path by pre-allocating storage.
#[derive(Debug, Clone)]
pub struct RingBuffer {
    /// Internal storage (allocated once)
    buffer: Vec<IQSample>,
    /// Current write position (next slot to write)
    head: usize,
    /// Number of valid samples (0 to capacity)
    len: usize,
}

impl RingBuffer {
    /// Create a new ring buffer with the specified capacity
    pub fn new(capacity: usize) -> Self {
        Self {
            buffer: vec![IQSample::new(0.0, 0.0); capacity],
            head: 0,
            len: 0,
        }
    }

    /// Push a single sample into the buffer
    #[inline]
    pub fn push(&mut self, sample: IQSample) {
        self.buffer[self.head] = sample;
        self.head = (self.head + 1) % self.buffer.len();
        if self.len < self.buffer.len() {
            self.len += 1;
        }
    }

    /// Push a batch of samples efficiently
    pub fn push_batch(&mut self, samples: &[IQSample]) {
        for &sample in samples {
            self.push(sample);
        }
    }

    /// Get samples in order (oldest to newest) as a new Vec.
    /// This allocates - use sparingly in rendering code.
    pub fn as_slice_ordered(&self) -> Vec<IQSample> {
        if self.len == 0 {
            return Vec::new();
        }

        let mut result = Vec::with_capacity(self.len);

        if self.len < self.buffer.len() {
            // Not full yet, samples are in order from 0..len
            result.extend_from_slice(&self.buffer[..self.len]);
        } else {
            // Full buffer: oldest samples start at head, wrap around
            result.extend_from_slice(&self.buffer[self.head..]);
            result.extend_from_slice(&self.buffer[..self.head]);
        }

        result
    }

    /// Iterate over samples in order (oldest to newest) without allocation.
    /// Returns an iterator that yields references to samples.
    pub fn iter_ordered(&self) -> impl Iterator<Item = &IQSample> {
        let (first, second) = if self.len < self.buffer.len() {
            (&self.buffer[..self.len], &[][..])
        } else {
            (&self.buffer[self.head..], &self.buffer[..self.head])
        };
        first.iter().chain(second.iter())
    }

    /// Get the most recent N samples (newest first).
    /// Returns fewer samples if buffer doesn't have that many.
    pub fn recent(&self, count: usize) -> Vec<IQSample> {
        let ordered = self.as_slice_ordered();
        let start = ordered.len().saturating_sub(count);
        ordered[start..].to_vec()
    }

    /// Number of valid samples in the buffer
    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Check if buffer is empty
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Maximum capacity of the buffer
    #[inline]
    pub fn capacity(&self) -> usize {
        self.buffer.len()
    }

    /// Clear all samples from the buffer
    pub fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }

    /// Resize the buffer capacity. Clears existing data.
    pub fn resize(&mut self, new_capacity: usize) {
        self.buffer = vec![IQSample::new(0.0, 0.0); new_capacity];
        self.head = 0;
        self.len = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_buffer() {
        let buf = RingBuffer::new(10);
        assert_eq!(buf.capacity(), 10);
        assert_eq!(buf.len(), 0);
        assert!(buf.is_empty());
    }

    #[test]
    fn test_push_single() {
        let mut buf = RingBuffer::new(3);
        buf.push(IQSample::new(1.0, 0.0));
        assert_eq!(buf.len(), 1);

        buf.push(IQSample::new(2.0, 0.0));
        assert_eq!(buf.len(), 2);

        buf.push(IQSample::new(3.0, 0.0));
        assert_eq!(buf.len(), 3);

        // Now wrap around
        buf.push(IQSample::new(4.0, 0.0));
        assert_eq!(buf.len(), 3); // Still 3, oldest overwritten
    }

    #[test]
    fn test_ordered_not_full() {
        let mut buf = RingBuffer::new(5);
        buf.push(IQSample::new(1.0, 0.0));
        buf.push(IQSample::new(2.0, 0.0));
        buf.push(IQSample::new(3.0, 0.0));

        let ordered = buf.as_slice_ordered();
        assert_eq!(ordered.len(), 3);
        assert_eq!(ordered[0].re, 1.0);
        assert_eq!(ordered[1].re, 2.0);
        assert_eq!(ordered[2].re, 3.0);
    }

    #[test]
    fn test_ordered_wrapped() {
        let mut buf = RingBuffer::new(3);
        buf.push(IQSample::new(1.0, 0.0));
        buf.push(IQSample::new(2.0, 0.0));
        buf.push(IQSample::new(3.0, 0.0));
        buf.push(IQSample::new(4.0, 0.0)); // Overwrites 1.0
        buf.push(IQSample::new(5.0, 0.0)); // Overwrites 2.0

        let ordered = buf.as_slice_ordered();
        assert_eq!(ordered.len(), 3);
        assert_eq!(ordered[0].re, 3.0); // Oldest remaining
        assert_eq!(ordered[1].re, 4.0);
        assert_eq!(ordered[2].re, 5.0); // Newest
    }

    #[test]
    fn test_recent() {
        let mut buf = RingBuffer::new(5);
        for i in 1..=5 {
            buf.push(IQSample::new(i as f64, 0.0));
        }

        let recent = buf.recent(2);
        assert_eq!(recent.len(), 2);
        assert_eq!(recent[0].re, 4.0);
        assert_eq!(recent[1].re, 5.0);
    }

    #[test]
    fn test_clear() {
        let mut buf = RingBuffer::new(5);
        buf.push_batch(&[
            IQSample::new(1.0, 0.0),
            IQSample::new(2.0, 0.0),
            IQSample::new(3.0, 0.0),
        ]);
        assert_eq!(buf.len(), 3);

        buf.clear();
        assert_eq!(buf.len(), 0);
        assert!(buf.is_empty());
    }

    #[test]
    fn test_iter_ordered() {
        let mut buf = RingBuffer::new(3);
        buf.push(IQSample::new(1.0, 0.0));
        buf.push(IQSample::new(2.0, 0.0));
        buf.push(IQSample::new(3.0, 0.0));
        buf.push(IQSample::new(4.0, 0.0)); // Wrap

        let values: Vec<f64> = buf.iter_ordered().map(|s| s.re).collect();
        assert_eq!(values, vec![2.0, 3.0, 4.0]);
    }
}
