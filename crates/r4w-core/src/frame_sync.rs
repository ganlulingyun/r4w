//! Frame Synchronizer
//!
//! Detects frame boundaries in a bit stream using known sync word patterns.
//! After detecting a sync word, extracts fixed-length frames from the stream.
//!
//! ## Algorithm
//!
//! 1. Shift each incoming bit into a register
//! 2. Compare register against sync word (Hamming distance)
//! 3. On match: extract N data bits as a frame
//! 4. Optionally lock to frame timing for reduced false alarms
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::frame_sync::FrameSync;
//!
//! let sync_word = vec![true, false, true, true, false, true, false, false];
//! let mut fs = FrameSync::new(&sync_word, 64, 1); // 64-bit frames, 1 bit error tolerance
//!
//! let mut stream = vec![false; 20]; // some noise
//! stream.extend_from_slice(&sync_word); // sync word
//! stream.extend(vec![true; 64]); // frame data
//! stream.extend(vec![false; 10]); // more noise
//!
//! let frames = fs.process(&stream);
//! assert_eq!(frames.len(), 1);
//! assert_eq!(frames[0].data.len(), 64);
//! ```

/// A detected frame with metadata.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    /// Frame data bits (after sync word)
    pub data: Vec<bool>,
    /// Bit index where sync word was detected
    pub sync_index: usize,
    /// Hamming distance at detection (0 = perfect match)
    pub hamming_distance: usize,
}

/// Frame synchronizer state.
#[derive(Debug, Clone, PartialEq)]
enum SyncState {
    /// Searching for sync word
    Searching,
    /// Collecting frame data
    Collecting { remaining: usize, sync_idx: usize, hamming: usize },
}

/// Frame synchronizer.
#[derive(Debug, Clone)]
pub struct FrameSync {
    /// Sync word pattern
    sync_word: Vec<bool>,
    /// Frame length in bits (data only, excluding sync word)
    frame_length: usize,
    /// Maximum allowed Hamming distance for sync detection
    max_hamming: usize,
    /// Shift register for sync word detection
    shift_reg: Vec<bool>,
    /// Current state
    state: SyncState,
    /// Frame data accumulator
    frame_data: Vec<bool>,
    /// Total bits processed
    bit_count: usize,
    /// Total frames detected
    frames_detected: usize,
    /// Whether to use locked mode (expect sync at regular intervals)
    locked_mode: bool,
    /// Expected next sync position (for locked mode)
    next_sync_at: Option<usize>,
}

impl FrameSync {
    /// Create a new frame synchronizer.
    ///
    /// - `sync_word`: Known sync pattern
    /// - `frame_length`: Number of data bits per frame (after sync)
    /// - `max_hamming`: Maximum Hamming distance for sync detection
    pub fn new(sync_word: &[bool], frame_length: usize, max_hamming: usize) -> Self {
        assert!(!sync_word.is_empty());
        assert!(frame_length > 0);

        Self {
            sync_word: sync_word.to_vec(),
            frame_length,
            max_hamming,
            shift_reg: vec![false; sync_word.len()],
            state: SyncState::Searching,
            frame_data: Vec::with_capacity(frame_length),
            bit_count: 0,
            frames_detected: 0,
            locked_mode: false,
            next_sync_at: None,
        }
    }

    /// Enable locked mode: after first detection, expect sync at regular intervals.
    pub fn set_locked_mode(&mut self, enabled: bool) {
        self.locked_mode = enabled;
    }

    /// Process a stream of bits and extract frames.
    pub fn process(&mut self, bits: &[bool]) -> Vec<Frame> {
        let mut frames = Vec::new();

        for &bit in bits {
            self.bit_count += 1;

            match self.state {
                SyncState::Searching => {
                    // Shift in new bit
                    self.shift_reg.push(bit);
                    if self.shift_reg.len() > self.sync_word.len() {
                        self.shift_reg.remove(0);
                    }

                    if self.shift_reg.len() < self.sync_word.len() {
                        continue;
                    }

                    // Check for sync word
                    let hamming = self.hamming_distance();
                    if hamming <= self.max_hamming {
                        self.state = SyncState::Collecting {
                            remaining: self.frame_length,
                            sync_idx: self.bit_count - self.sync_word.len(),
                            hamming,
                        };
                        self.frame_data.clear();

                        if self.locked_mode {
                            self.next_sync_at = Some(
                                self.bit_count + self.frame_length,
                            );
                        }
                    }
                }
                SyncState::Collecting {
                    remaining,
                    sync_idx,
                    hamming,
                } => {
                    self.frame_data.push(bit);
                    let remaining = remaining - 1;

                    if remaining == 0 {
                        frames.push(Frame {
                            data: self.frame_data.clone(),
                            sync_index: sync_idx,
                            hamming_distance: hamming,
                        });
                        self.frames_detected += 1;
                        self.frame_data.clear();
                        self.state = SyncState::Searching;
                    } else {
                        self.state = SyncState::Collecting {
                            remaining,
                            sync_idx,
                            hamming,
                        };
                    }
                }
            }
        }

        frames
    }

    /// Compute Hamming distance between shift register and sync word.
    fn hamming_distance(&self) -> usize {
        self.shift_reg
            .iter()
            .zip(self.sync_word.iter())
            .filter(|(&a, &b)| a != b)
            .count()
    }

    /// Get total frames detected.
    pub fn frames_detected(&self) -> usize {
        self.frames_detected
    }

    /// Get sync word length.
    pub fn sync_word_len(&self) -> usize {
        self.sync_word.len()
    }

    /// Get frame length.
    pub fn frame_length(&self) -> usize {
        self.frame_length
    }

    /// Check if currently collecting frame data.
    pub fn is_collecting(&self) -> bool {
        matches!(self.state, SyncState::Collecting { .. })
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.shift_reg.fill(false);
        self.state = SyncState::Searching;
        self.frame_data.clear();
        self.bit_count = 0;
        self.frames_detected = 0;
        self.next_sync_at = None;
    }
}

/// Utility: Create a sync word from a hex string (MSB first).
pub fn sync_word_from_hex(hex: &str) -> Vec<bool> {
    let hex = hex.trim_start_matches("0x").trim_start_matches("0X");
    let mut bits = Vec::new();
    for ch in hex.chars() {
        let nibble = u8::from_str_radix(&ch.to_string(), 16).unwrap_or(0);
        for i in (0..4).rev() {
            bits.push((nibble >> i) & 1 != 0);
        }
    }
    bits
}

#[cfg(test)]
mod tests {
    use super::*;

    fn barker7() -> Vec<bool> {
        // Barker-7: +1 +1 +1 -1 -1 +1 -1 → true true true false false true false
        vec![true, true, true, false, false, true, false]
    }

    #[test]
    fn test_basic_sync() {
        let sync = barker7();
        let mut fs = FrameSync::new(&sync, 8, 0);

        let mut stream = vec![false; 10];
        stream.extend_from_slice(&sync);
        stream.extend(vec![true, false, true, false, true, false, true, false]); // 8-bit frame

        let frames = fs.process(&stream);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].data.len(), 8);
        assert_eq!(frames[0].data, vec![true, false, true, false, true, false, true, false]);
        assert_eq!(frames[0].hamming_distance, 0);
    }

    #[test]
    fn test_multiple_frames() {
        // Use a longer sync word that won't appear in data
        let sync = barker7();
        let mut fs = FrameSync::new(&sync, 8, 0);

        let mut stream = Vec::new();
        // Frame 1
        stream.extend_from_slice(&sync);
        stream.extend_from_slice(&[true, true, false, false, true, true, false, false]);
        // Gap
        stream.extend(vec![false; 5]);
        // Frame 2
        stream.extend_from_slice(&sync);
        stream.extend_from_slice(&[false, true, false, true, false, true, false, true]);

        let frames = fs.process(&stream);
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].data, vec![true, true, false, false, true, true, false, false]);
        assert_eq!(frames[1].data, vec![false, true, false, true, false, true, false, true]);
    }

    #[test]
    fn test_hamming_tolerance() {
        let sync = vec![true, true, true, true];
        let mut fs = FrameSync::new(&sync, 4, 1); // Allow 1 bit error

        let mut stream = Vec::new();
        // Sync with 1 bit error
        stream.extend_from_slice(&[true, false, true, true]);
        stream.extend_from_slice(&[true, false, true, false]);

        let frames = fs.process(&stream);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].hamming_distance, 1);
    }

    #[test]
    fn test_no_false_detect_strict() {
        let sync = barker7();
        let mut fs = FrameSync::new(&sync, 8, 0);

        // Random-ish data without sync word
        let stream = vec![false; 100];
        let frames = fs.process(&stream);
        assert!(frames.is_empty());
    }

    #[test]
    fn test_incremental_feeding() {
        let sync = vec![true, false, true, false];
        let mut fs = FrameSync::new(&sync, 4, 0);

        // Feed bit by bit
        let mut all_frames = Vec::new();
        let mut stream: Vec<bool> = vec![false; 5];
        stream.extend_from_slice(&sync);
        stream.extend_from_slice(&[true, true, false, false]);

        for &bit in &stream {
            all_frames.extend(fs.process(&[bit]));
        }
        assert_eq!(all_frames.len(), 1);
        assert_eq!(all_frames[0].data, vec![true, true, false, false]);
    }

    #[test]
    fn test_sync_word_from_hex() {
        let bits = sync_word_from_hex("A5");
        assert_eq!(bits, vec![true, false, true, false, false, true, false, true]);
    }

    #[test]
    fn test_sync_word_from_hex_0x() {
        let bits = sync_word_from_hex("0xFF");
        assert_eq!(bits.len(), 8);
        assert!(bits.iter().all(|&b| b));
    }

    #[test]
    fn test_is_collecting() {
        let sync = vec![true, false];
        let mut fs = FrameSync::new(&sync, 4, 0);
        assert!(!fs.is_collecting());

        fs.process(&[true, false]); // Sync detected
        assert!(fs.is_collecting());

        fs.process(&[true, true, false, false]); // Frame complete
        assert!(!fs.is_collecting());
    }

    #[test]
    fn test_reset() {
        let sync = vec![true, false, true, false];
        let mut fs = FrameSync::new(&sync, 4, 0);

        let mut stream = sync.clone();
        stream.extend_from_slice(&[true; 4]);
        fs.process(&stream);
        assert_eq!(fs.frames_detected(), 1);

        fs.reset();
        assert_eq!(fs.frames_detected(), 0);
        assert!(!fs.is_collecting());
    }
}
