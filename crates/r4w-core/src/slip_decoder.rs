//! # SLIP (Serial Line Internet Protocol) Frame Codec
//!
//! Implements RFC 1055 SLIP framing with byte-stuffing for packet transport
//! over serial links, plus COBS (Consistent Overhead Byte Stuffing) as an
//! alternative framing scheme.
//!
//! SLIP uses special byte values to delimit frames:
//! - `END` (0xC0) marks frame boundaries
//! - `ESC` (0xDB) followed by `ESC_END` (0xDC) encodes a literal 0xC0
//! - `ESC` (0xDB) followed by `ESC_ESC` (0xDD) encodes a literal 0xDB
//!
//! Complex samples use `(f64, f64)` tuples representing (real, imaginary).
//!
//! # Example
//!
//! ```
//! use r4w_core::slip_decoder::{SlipEncoder, SlipDecoder, SlipEvent};
//!
//! // Encode a payload
//! let payload = b"Hello SLIP!";
//! let encoded = SlipEncoder::encode(payload);
//!
//! // Decode it back
//! let mut decoder = SlipDecoder::new();
//! for &byte in &encoded {
//!     if let Some(event) = decoder.feed(byte) {
//!         match event {
//!             SlipEvent::Frame(data) => {
//!                 assert_eq!(&data, payload);
//!             }
//!             SlipEvent::Error(_) => panic!("unexpected error"),
//!         }
//!     }
//! }
//! assert_eq!(decoder.stats().frames_decoded, 1);
//! ```

/// SLIP END delimiter byte (0xC0).
pub const END: u8 = 0xC0;

/// SLIP ESC byte (0xDB).
pub const ESC: u8 = 0xDB;

/// SLIP escaped END replacement (0xDC) — follows ESC to represent a literal END.
pub const ESC_END: u8 = 0xDC;

/// SLIP escaped ESC replacement (0xDD) — follows ESC to represent a literal ESC.
pub const ESC_ESC: u8 = 0xDD;

// ---------------------------------------------------------------------------
// SlipError
// ---------------------------------------------------------------------------

/// Errors that can occur during SLIP decoding.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SlipError {
    /// An ESC byte was followed by an unexpected byte (not ESC_END or ESC_ESC).
    InvalidEscapeSequence(u8),
    /// A frame ended while an ESC was still pending.
    UnexpectedEndAfterEsc,
}

impl std::fmt::Display for SlipError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SlipError::InvalidEscapeSequence(b) => {
                write!(f, "invalid SLIP escape: ESC followed by 0x{:02X}", b)
            }
            SlipError::UnexpectedEndAfterEsc => {
                write!(f, "frame ended with pending ESC byte")
            }
        }
    }
}

impl std::error::Error for SlipError {}

// ---------------------------------------------------------------------------
// SlipEvent
// ---------------------------------------------------------------------------

/// An event emitted by [`SlipDecoder::feed`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SlipEvent {
    /// A complete frame was decoded.
    Frame(Vec<u8>),
    /// An error was detected while decoding.
    Error(SlipError),
}

// ---------------------------------------------------------------------------
// SlipStats
// ---------------------------------------------------------------------------

/// Statistics collected by a [`SlipDecoder`].
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SlipStats {
    /// Total number of successfully decoded frames.
    pub frames_decoded: usize,
    /// Total number of errors encountered.
    pub error_count: usize,
    /// Lengths of each successfully decoded frame (in order).
    pub frame_lengths: Vec<usize>,
    /// Total bytes consumed (including framing overhead).
    pub bytes_consumed: usize,
}

// ---------------------------------------------------------------------------
// SlipDecoder
// ---------------------------------------------------------------------------

/// Stateful SLIP frame decoder.
///
/// Feed bytes one at a time with [`feed`](SlipDecoder::feed) or in batch with
/// [`decode_all`](SlipDecoder::decode_all). Complete frames and errors are
/// returned as [`SlipEvent`] values.
pub struct SlipDecoder {
    buf: Vec<u8>,
    in_escape: bool,
    /// Whether we have started receiving data for the current frame.
    started: bool,
    stats: SlipStats,
}

impl SlipDecoder {
    /// Creates a new decoder in its initial state.
    pub fn new() -> Self {
        Self {
            buf: Vec::new(),
            in_escape: false,
            started: false,
            stats: SlipStats::default(),
        }
    }

    /// Feed a single byte into the decoder.
    ///
    /// Returns `Some(SlipEvent)` when a frame is completed or an error is
    /// detected; `None` if more bytes are needed.
    pub fn feed(&mut self, byte: u8) -> Option<SlipEvent> {
        self.stats.bytes_consumed += 1;

        if self.in_escape {
            self.in_escape = false;
            match byte {
                ESC_END => {
                    self.buf.push(END);
                    return None;
                }
                ESC_ESC => {
                    self.buf.push(ESC);
                    return None;
                }
                END => {
                    // ESC followed by END — report error, then emit frame if
                    // buffer non-empty (treat the END as a delimiter).
                    self.stats.error_count += 1;
                    let err = SlipEvent::Error(SlipError::UnexpectedEndAfterEsc);
                    // Reset state — the END terminates whatever was in progress.
                    self.buf.clear();
                    self.started = false;
                    return Some(err);
                }
                _ => {
                    self.stats.error_count += 1;
                    // Per common practice, drop the invalid pair and keep
                    // decoding the current frame.
                    return Some(SlipEvent::Error(SlipError::InvalidEscapeSequence(byte)));
                }
            }
        }

        match byte {
            END => {
                if !self.started {
                    // Leading END — just marks the start of a new frame.
                    self.started = true;
                    return None;
                }
                if self.buf.is_empty() {
                    // Back-to-back ENDs with no data — ignore.
                    return None;
                }
                // Complete frame.
                let frame = std::mem::take(&mut self.buf);
                self.stats.frames_decoded += 1;
                self.stats.frame_lengths.push(frame.len());
                self.started = false;
                Some(SlipEvent::Frame(frame))
            }
            ESC => {
                self.in_escape = true;
                self.started = true;
                None
            }
            _ => {
                self.started = true;
                self.buf.push(byte);
                None
            }
        }
    }

    /// Decode all complete frames (and errors) from `data`.
    pub fn decode_all(&mut self, data: &[u8]) -> Vec<SlipEvent> {
        let mut events = Vec::new();
        for &b in data {
            if let Some(ev) = self.feed(b) {
                events.push(ev);
            }
        }
        events
    }

    /// Returns a snapshot of the accumulated statistics.
    pub fn stats(&self) -> &SlipStats {
        &self.stats
    }

    /// Resets the decoder to its initial state, clearing any partial frame and
    /// statistics.
    pub fn reset(&mut self) {
        self.buf.clear();
        self.in_escape = false;
        self.started = false;
        self.stats = SlipStats::default();
    }

    /// Returns `true` if the decoder is currently inside an incomplete frame.
    pub fn is_partial(&self) -> bool {
        self.started && !self.buf.is_empty()
    }

    /// Attempt to resynchronise by discarding partial state and searching for
    /// the next END delimiter.
    ///
    /// Feeds bytes from `stream` until an END is consumed (marking a clean
    /// boundary), then returns the number of bytes skipped.  If the stream
    /// runs out before an END is found, all bytes are consumed and the
    /// decoder remains in a "searching" state.
    pub fn sync_recover(&mut self, stream: &[u8]) -> usize {
        // Discard any partial frame.
        self.buf.clear();
        self.in_escape = false;
        self.started = false;

        for (i, &b) in stream.iter().enumerate() {
            self.stats.bytes_consumed += 1;
            if b == END {
                self.started = true;
                return i + 1;
            }
        }
        stream.len()
    }
}

impl Default for SlipDecoder {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// SlipEncoder
// ---------------------------------------------------------------------------

/// SLIP frame encoder.
///
/// All methods are stateless and operate on complete payloads.
pub struct SlipEncoder;

impl SlipEncoder {
    /// Encode `payload` into a SLIP frame.
    ///
    /// The returned buffer starts and ends with an END delimiter and contains
    /// the byte-stuffed payload in between.
    pub fn encode(payload: &[u8]) -> Vec<u8> {
        // Worst case: every byte is stuffed (x2) plus two END delimiters.
        let mut out = Vec::with_capacity(payload.len() * 2 + 2);
        out.push(END);
        for &b in payload {
            match b {
                END => {
                    out.push(ESC);
                    out.push(ESC_END);
                }
                ESC => {
                    out.push(ESC);
                    out.push(ESC_ESC);
                }
                _ => out.push(b),
            }
        }
        out.push(END);
        out
    }

    /// Encode `payload` *without* a leading END delimiter.
    ///
    /// Some implementations omit the leading END for efficiency on clean links.
    pub fn encode_no_leading_end(payload: &[u8]) -> Vec<u8> {
        let mut out = Vec::with_capacity(payload.len() * 2 + 1);
        for &b in payload {
            match b {
                END => {
                    out.push(ESC);
                    out.push(ESC_END);
                }
                ESC => {
                    out.push(ESC);
                    out.push(ESC_ESC);
                }
                _ => out.push(b),
            }
        }
        out.push(END);
        out
    }
}

// ---------------------------------------------------------------------------
// COBS (Consistent Overhead Byte Stuffing)
// ---------------------------------------------------------------------------

/// COBS encoder / decoder — an alternative to SLIP framing.
///
/// COBS eliminates a specific byte value (typically 0x00) from the payload so
/// that it can be used unambiguously as a frame delimiter.  The overhead is at
/// most one byte per 254 payload bytes.
pub struct Cobs;

impl Cobs {
    /// Encode `data` using COBS, eliminating all 0x00 bytes.
    ///
    /// The returned buffer does **not** include a trailing zero delimiter;
    /// callers should append one if using 0x00 as a frame delimiter.
    pub fn encode(data: &[u8]) -> Vec<u8> {
        let mut out = Vec::with_capacity(data.len() + data.len() / 254 + 2);
        let mut code_idx = 0usize;
        out.push(0); // placeholder for first code byte
        let mut code: u8 = 1;

        for &b in data {
            if b == 0 {
                out[code_idx] = code;
                code = 1;
                code_idx = out.len();
                out.push(0); // placeholder
            } else {
                out.push(b);
                code += 1;
                if code == 0xFF {
                    out[code_idx] = code;
                    code = 1;
                    code_idx = out.len();
                    out.push(0); // placeholder
                }
            }
        }
        out[code_idx] = code;
        out
    }

    /// Decode a COBS-encoded buffer, restoring the original data.
    ///
    /// `encoded` must **not** contain the trailing zero delimiter.
    ///
    /// Returns `None` if the encoding is invalid (e.g., a code byte points
    /// past the end of the buffer, or a zero byte is encountered in the
    /// encoded region).
    pub fn decode(encoded: &[u8]) -> Option<Vec<u8>> {
        if encoded.is_empty() {
            return Some(Vec::new());
        }

        let mut out = Vec::with_capacity(encoded.len());
        let mut idx = 0usize;

        while idx < encoded.len() {
            let code = encoded[idx] as usize;
            if code == 0 {
                return None; // unexpected zero
            }
            idx += 1;

            for _ in 1..code {
                if idx >= encoded.len() {
                    return None;
                }
                if encoded[idx] == 0 {
                    return None; // unexpected zero in data region
                }
                out.push(encoded[idx]);
                idx += 1;
            }

            // Insert a zero between groups, but only if this is NOT the
            // last group AND the code was < 0xFF (a 0xFF code means the
            // 254-byte run had no implicit zero).
            if code < 0xFF && idx < encoded.len() {
                out.push(0);
            }
        }

        Some(out)
    }
}

// ---------------------------------------------------------------------------
// Frame delimiter detection in noisy streams
// ---------------------------------------------------------------------------

/// Scan a noisy byte stream and return the byte indices of every END (0xC0)
/// delimiter found.
///
/// This is useful for a first pass over a received buffer to locate potential
/// frame boundaries before attempting full SLIP decoding.
pub fn find_end_delimiters(data: &[u8]) -> Vec<usize> {
    data.iter()
        .enumerate()
        .filter_map(|(i, &b)| if b == END { Some(i) } else { None })
        .collect()
}

/// Extract candidate SLIP frames from a raw byte buffer.
///
/// Returns a vector of byte slices, one per pair of END delimiters that
/// contain at least one byte of payload.  The returned slices do **not**
/// include the END delimiters themselves.
pub fn extract_raw_frames(data: &[u8]) -> Vec<&[u8]> {
    let positions = find_end_delimiters(data);
    let mut frames = Vec::new();
    let mut prev: Option<usize> = None;
    for &pos in &positions {
        if let Some(start) = prev {
            let slice = &data[start + 1..pos];
            if !slice.is_empty() {
                frames.push(slice);
            }
        }
        prev = Some(pos);
    }
    frames
}

// ---------------------------------------------------------------------------
// Complex-sample utility (f64, f64)
// ---------------------------------------------------------------------------

/// Convert a slice of payload bytes to complex baseband samples.
///
/// Each byte is mapped to a BPSK-like constellation point per bit:
/// bit 1 -> (1.0, 0.0), bit 0 -> (-1.0, 0.0).
///
/// This is a convenience function for testing SLIP-framed payloads in a DSP
/// pipeline.
pub fn bytes_to_bpsk(data: &[u8]) -> Vec<(f64, f64)> {
    let mut samples = Vec::with_capacity(data.len() * 8);
    for &byte in data {
        for bit_idx in (0..8).rev() {
            let bit = (byte >> bit_idx) & 1;
            let val = if bit == 1 { 1.0 } else { -1.0 };
            samples.push((val, 0.0));
        }
    }
    samples
}

/// Convert BPSK complex samples back to bytes (hard decision).
///
/// Expects `samples.len()` to be a multiple of 8.
pub fn bpsk_to_bytes(samples: &[(f64, f64)]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(samples.len() / 8);
    for chunk in samples.chunks(8) {
        let mut byte = 0u8;
        for (i, &(re, _im)) in chunk.iter().enumerate() {
            if re >= 0.0 {
                byte |= 1 << (7 - i);
            }
        }
        bytes.push(byte);
    }
    bytes
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ---- Encoder tests ----

    #[test]
    fn test_encode_empty_payload() {
        let encoded = SlipEncoder::encode(b"");
        assert_eq!(encoded, vec![END, END]);
    }

    #[test]
    fn test_encode_simple_payload() {
        let encoded = SlipEncoder::encode(b"AB");
        assert_eq!(encoded, vec![END, b'A', b'B', END]);
    }

    #[test]
    fn test_encode_payload_with_end_byte() {
        let encoded = SlipEncoder::encode(&[0x01, END, 0x02]);
        assert_eq!(encoded, vec![END, 0x01, ESC, ESC_END, 0x02, END]);
    }

    #[test]
    fn test_encode_payload_with_esc_byte() {
        let encoded = SlipEncoder::encode(&[0x01, ESC, 0x02]);
        assert_eq!(encoded, vec![END, 0x01, ESC, ESC_ESC, 0x02, END]);
    }

    #[test]
    fn test_encode_all_special_bytes() {
        let encoded = SlipEncoder::encode(&[END, ESC, END, ESC]);
        assert_eq!(
            encoded,
            vec![END, ESC, ESC_END, ESC, ESC_ESC, ESC, ESC_END, ESC, ESC_ESC, END]
        );
    }

    #[test]
    fn test_encode_no_leading_end() {
        let encoded = SlipEncoder::encode_no_leading_end(b"Hi");
        assert_eq!(encoded, vec![b'H', b'i', END]);
    }

    // ---- Decoder tests ----

    #[test]
    fn test_decode_simple_roundtrip() {
        let payload = b"Hello SLIP!";
        let encoded = SlipEncoder::encode(payload);
        let mut dec = SlipDecoder::new();
        let events = dec.decode_all(&encoded);
        assert_eq!(events, vec![SlipEvent::Frame(payload.to_vec())]);
        assert_eq!(dec.stats().frames_decoded, 1);
        assert_eq!(dec.stats().frame_lengths, vec![payload.len()]);
    }

    #[test]
    fn test_decode_multiple_frames() {
        let mut buf = Vec::new();
        buf.extend_from_slice(&SlipEncoder::encode(b"one"));
        buf.extend_from_slice(&SlipEncoder::encode(b"two"));
        buf.extend_from_slice(&SlipEncoder::encode(b"three"));

        let mut dec = SlipDecoder::new();
        let events = dec.decode_all(&buf);
        assert_eq!(events.len(), 3);
        assert_eq!(events[0], SlipEvent::Frame(b"one".to_vec()));
        assert_eq!(events[1], SlipEvent::Frame(b"two".to_vec()));
        assert_eq!(events[2], SlipEvent::Frame(b"three".to_vec()));
        assert_eq!(dec.stats().frames_decoded, 3);
    }

    #[test]
    fn test_decode_byte_stuffed_end() {
        let payload = vec![0x01, END, 0x02];
        let encoded = SlipEncoder::encode(&payload);
        let mut dec = SlipDecoder::new();
        let events = dec.decode_all(&encoded);
        assert_eq!(events, vec![SlipEvent::Frame(payload)]);
    }

    #[test]
    fn test_decode_byte_stuffed_esc() {
        let payload = vec![0x01, ESC, 0x02];
        let encoded = SlipEncoder::encode(&payload);
        let mut dec = SlipDecoder::new();
        let events = dec.decode_all(&encoded);
        assert_eq!(events, vec![SlipEvent::Frame(payload)]);
    }

    #[test]
    fn test_decode_invalid_escape_sequence() {
        // ESC followed by 0x42 is invalid.
        let data = vec![END, 0x01, ESC, 0x42, 0x02, END];
        let mut dec = SlipDecoder::new();
        let events = dec.decode_all(&data);
        // Should get an error AND then a frame (the 0x42 and 0x02 remain).
        assert!(events.iter().any(|e| matches!(e, SlipEvent::Error(SlipError::InvalidEscapeSequence(0x42)))));
        // Frame should still be produced with the bytes that were accumulated.
        assert!(events.iter().any(|e| matches!(e, SlipEvent::Frame(_))));
    }

    #[test]
    fn test_decode_esc_at_end_of_frame() {
        // ESC immediately followed by END — error.
        let data = vec![END, 0x01, ESC, END];
        let mut dec = SlipDecoder::new();
        let events = dec.decode_all(&data);
        assert!(events.iter().any(|e| matches!(e, SlipEvent::Error(SlipError::UnexpectedEndAfterEsc))));
        assert_eq!(dec.stats().error_count, 1);
    }

    #[test]
    fn test_decode_ignores_leading_ends() {
        // Multiple leading ENDs should not produce empty frames.
        let data = vec![END, END, END, b'A', END];
        let mut dec = SlipDecoder::new();
        let events = dec.decode_all(&data);
        assert_eq!(events, vec![SlipEvent::Frame(vec![b'A'])]);
    }

    #[test]
    fn test_decoder_reset() {
        let mut dec = SlipDecoder::new();
        dec.decode_all(&SlipEncoder::encode(b"test"));
        assert_eq!(dec.stats().frames_decoded, 1);
        dec.reset();
        assert_eq!(dec.stats().frames_decoded, 0);
        assert!(!dec.is_partial());
    }

    #[test]
    fn test_decoder_is_partial() {
        let mut dec = SlipDecoder::new();
        assert!(!dec.is_partial());
        dec.feed(END);
        assert!(!dec.is_partial());
        dec.feed(b'X');
        assert!(dec.is_partial());
        dec.feed(END);
        assert!(!dec.is_partial());
    }

    #[test]
    fn test_stats_bytes_consumed() {
        let encoded = SlipEncoder::encode(b"abc");
        let len = encoded.len();
        let mut dec = SlipDecoder::new();
        dec.decode_all(&encoded);
        assert_eq!(dec.stats().bytes_consumed, len);
    }

    // ---- Sync recovery ----

    #[test]
    fn test_sync_recover_skips_garbage() {
        let mut dec = SlipDecoder::new();
        // Feed some garbage first to create a partial frame.
        dec.feed(b'x');
        dec.feed(b'y');
        assert!(dec.is_partial());

        let remaining = vec![0xFF, 0xFE, 0xFD, END, b'A', END];
        let skipped = dec.sync_recover(&remaining[..]);
        assert_eq!(skipped, 4); // 3 garbage bytes + the END itself
        assert!(!dec.is_partial());

        // Now feed the rest — should decode frame [A].
        let events = dec.decode_all(&remaining[skipped..]);
        assert_eq!(events, vec![SlipEvent::Frame(vec![b'A'])]);
    }

    // ---- COBS tests ----

    #[test]
    fn test_cobs_roundtrip_simple() {
        let data = b"Hello COBS!";
        let encoded = Cobs::encode(data);
        // Encoded should contain no zero bytes.
        assert!(!encoded.contains(&0u8));
        let decoded = Cobs::decode(&encoded).expect("valid COBS");
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_cobs_roundtrip_with_zeros() {
        let data = vec![0x00, 0x01, 0x00, 0x00, 0x02, 0x03, 0x00];
        let encoded = Cobs::encode(&data);
        assert!(!encoded.contains(&0u8));
        let decoded = Cobs::decode(&encoded).expect("valid COBS");
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_cobs_empty() {
        let encoded = Cobs::encode(b"");
        let decoded = Cobs::decode(&encoded).expect("valid COBS");
        assert!(decoded.is_empty());
    }

    #[test]
    fn test_cobs_all_zeros() {
        let data = vec![0u8; 5];
        let encoded = Cobs::encode(&data);
        assert!(!encoded.contains(&0u8));
        let decoded = Cobs::decode(&encoded).expect("valid COBS");
        assert_eq!(decoded, data);
    }

    // ---- Delimiter / raw frame extraction ----

    #[test]
    fn test_find_end_delimiters() {
        let data = vec![0x01, END, 0x02, 0x03, END, END];
        let positions = find_end_delimiters(&data);
        assert_eq!(positions, vec![1, 4, 5]);
    }

    #[test]
    fn test_extract_raw_frames() {
        let data = vec![END, b'A', b'B', END, END, b'X', END];
        let frames = extract_raw_frames(&data);
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0], &[b'A', b'B']);
        assert_eq!(frames[1], &[b'X']);
    }

    // ---- Complex-sample helpers ----

    #[test]
    fn test_bpsk_roundtrip() {
        let data = vec![0xA5, 0x3C];
        let samples = bytes_to_bpsk(&data);
        assert_eq!(samples.len(), 16);
        let recovered = bpsk_to_bytes(&samples);
        assert_eq!(recovered, data);
    }

    // ---- Large frame stress test ----

    #[test]
    fn test_large_frame_roundtrip() {
        let payload: Vec<u8> = (0..=255).cycle().take(4096).collect();
        let encoded = SlipEncoder::encode(&payload);
        let mut dec = SlipDecoder::new();
        let events = dec.decode_all(&encoded);
        assert_eq!(events.len(), 1);
        if let SlipEvent::Frame(ref f) = events[0] {
            assert_eq!(f, &payload);
        } else {
            panic!("expected Frame event");
        }
    }
}
