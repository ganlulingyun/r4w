//! Packet Analyzer with Annotated Hex Dump
//!
//! Provides tools for analyzing and displaying Meshtastic packets with
//! field-level annotations for debugging and education.
//!
//! ## Example Output
//!
//! ```text
//! === Meshtastic Packet Analysis ===
//!
//! Header (16 bytes):
//!   00-03: ff ff ff ff  │ To: BROADCAST (0xffffffff)
//!   04-07: a1 b2 c3 d4  │ From: 0xa1b2c3d4
//!   08-0b: 12 34 00 00  │ ID: 0x00003412
//!   0c:    63           │ Flags: hop_limit=3, want_ack=0, via_mqtt=0, hop_start=3
//!   0d:    1a           │ Channel hash: 0x1a
//!   0e:    00           │ Next hop: 0 (broadcast)
//!   0f:    00           │ Relay node: 0
//!
//! Payload (23 bytes):
//!   10-1f: 08 c8 01 12 0d 48 65 6c 6c 6f 20 4d 65 73 68 21
//!   20-26: 1a 06 ...
//! ```

use super::{WireHeader, WIRE_HEADER_SIZE};
use std::fmt::Write as FmtWrite;

/// Packet analysis result with structured field information
#[derive(Debug, Clone)]
pub struct PacketAnalysis {
    /// Raw packet bytes
    pub raw: Vec<u8>,
    /// Parsed header (if valid)
    pub header: Option<WireHeader>,
    /// Payload bytes (after header)
    pub payload: Vec<u8>,
    /// Encrypted flag (based on payload pattern)
    pub encrypted: bool,
    /// Validation messages
    pub messages: Vec<AnalysisMessage>,
}

/// Analysis message type
#[derive(Debug, Clone)]
pub enum MessageLevel {
    Info,
    Warning,
    Error,
}

/// Analysis message
#[derive(Debug, Clone)]
pub struct AnalysisMessage {
    pub level: MessageLevel,
    pub message: String,
}

impl PacketAnalysis {
    /// Analyze raw packet bytes
    pub fn analyze(bytes: &[u8]) -> Self {
        let mut messages = Vec::new();

        if bytes.len() < WIRE_HEADER_SIZE {
            messages.push(AnalysisMessage {
                level: MessageLevel::Error,
                message: format!(
                    "Packet too short: {} bytes (need {} for header)",
                    bytes.len(),
                    WIRE_HEADER_SIZE
                ),
            });
            return Self {
                raw: bytes.to_vec(),
                header: None,
                payload: Vec::new(),
                encrypted: false,
                messages,
            };
        }

        let header = WireHeader::from_bytes(bytes);
        let payload = bytes[WIRE_HEADER_SIZE..].to_vec();

        // Check for common issues
        if let Some(ref h) = header {
            // Check hop limit
            if h.flags.hop_limit() == 0 && h.flags.hop_start() > 0 {
                messages.push(AnalysisMessage {
                    level: MessageLevel::Warning,
                    message: "Hop limit exhausted - packet should not be forwarded".into(),
                });
            }

            // Check for self-addressing
            if h.from == h.to && !h.is_broadcast() {
                messages.push(AnalysisMessage {
                    level: MessageLevel::Warning,
                    message: "Source equals destination (self-addressed)".into(),
                });
            }

            // Check for zero source
            if h.from == 0 {
                messages.push(AnalysisMessage {
                    level: MessageLevel::Warning,
                    message: "Source node ID is zero (uninitialized?)".into(),
                });
            }
        }

        // Check if likely encrypted (no protobuf field marker at start)
        let encrypted = if !payload.is_empty() {
            // Protobuf typically starts with field tags (0x08, 0x10, 0x12, 0x18, etc.)
            let first_byte = payload[0];
            // Encrypted data is pseudo-random, unlikely to be a valid protobuf field tag
            first_byte > 0x7F || (first_byte & 0x07) > 5
        } else {
            false
        };

        if encrypted {
            messages.push(AnalysisMessage {
                level: MessageLevel::Info,
                message: "Payload appears to be encrypted".into(),
            });
        }

        Self {
            raw: bytes.to_vec(),
            header,
            payload,
            encrypted,
            messages,
        }
    }

    /// Format analysis as annotated hex dump
    pub fn format_hex_dump(&self) -> String {
        let mut out = String::new();

        writeln!(out, "=== Meshtastic Packet Analysis ===").unwrap();
        writeln!(out, "Total size: {} bytes", self.raw.len()).unwrap();
        writeln!(out).unwrap();

        // Header section
        if let Some(ref h) = self.header {
            writeln!(out, "Header (16 bytes):").unwrap();

            // To field (bytes 0-3)
            let to_str = if h.is_broadcast() {
                "BROADCAST".to_string()
            } else {
                format!("0x{:08x}", h.to)
            };
            writeln!(
                out,
                "  00-03: {:02x} {:02x} {:02x} {:02x}  │ To: {} (0x{:08x})",
                self.raw[0], self.raw[1], self.raw[2], self.raw[3],
                to_str, h.to
            ).unwrap();

            // From field (bytes 4-7)
            writeln!(
                out,
                "  04-07: {:02x} {:02x} {:02x} {:02x}  │ From: 0x{:08x}",
                self.raw[4], self.raw[5], self.raw[6], self.raw[7],
                h.from
            ).unwrap();

            // ID field (bytes 8-11)
            writeln!(
                out,
                "  08-0b: {:02x} {:02x} {:02x} {:02x}  │ ID: 0x{:08x}",
                self.raw[8], self.raw[9], self.raw[10], self.raw[11],
                h.id
            ).unwrap();

            // Flags byte (byte 12)
            writeln!(
                out,
                "  0c:    {:02x}           │ Flags: hop_limit={}, want_ack={}, via_mqtt={}, hop_start={}",
                self.raw[12],
                h.flags.hop_limit(),
                h.flags.want_ack() as u8,
                h.flags.via_mqtt() as u8,
                h.flags.hop_start()
            ).unwrap();

            // Channel hash (byte 13)
            writeln!(
                out,
                "  0d:    {:02x}           │ Channel hash: 0x{:02x}",
                self.raw[13], h.channel_hash
            ).unwrap();

            // Next hop (byte 14)
            let next_hop_str = if h.next_hop == 0 {
                "0 (broadcast/direct)".to_string()
            } else {
                format!("0x{:02x}", h.next_hop)
            };
            writeln!(
                out,
                "  0e:    {:02x}           │ Next hop: {}",
                self.raw[14], next_hop_str
            ).unwrap();

            // Relay node (byte 15)
            let relay_str = if h.relay_node == 0 {
                "0 (not set)".to_string()
            } else {
                format!("0x{:02x}", h.relay_node)
            };
            writeln!(
                out,
                "  0f:    {:02x}           │ Relay node: {}",
                self.raw[15], relay_str
            ).unwrap();
        } else {
            writeln!(out, "Header: INVALID (packet too short)").unwrap();
        }

        writeln!(out).unwrap();

        // Payload section
        if !self.payload.is_empty() {
            writeln!(
                out,
                "Payload ({} bytes){}:",
                self.payload.len(),
                if self.encrypted { " [ENCRYPTED]" } else { "" }
            ).unwrap();

            format_hex_lines(&mut out, &self.payload, WIRE_HEADER_SIZE);
        } else {
            writeln!(out, "Payload: (empty)").unwrap();
        }

        // Messages
        if !self.messages.is_empty() {
            writeln!(out).unwrap();
            writeln!(out, "Notes:").unwrap();
            for msg in &self.messages {
                let prefix = match msg.level {
                    MessageLevel::Info => "[INFO]",
                    MessageLevel::Warning => "[WARN]",
                    MessageLevel::Error => "[ERROR]",
                };
                writeln!(out, "  {} {}", prefix, msg.message).unwrap();
            }
        }

        out
    }

    /// Get a compact one-line summary
    pub fn summary(&self) -> String {
        match &self.header {
            Some(h) => {
                let dest = if h.is_broadcast() {
                    "BCAST".to_string()
                } else {
                    format!("{:08x}", h.to)
                };
                format!(
                    "{:08x} -> {} id={:08x} ch={:02x} hops={}/{} payload={}B{}",
                    h.from,
                    dest,
                    h.id,
                    h.channel_hash,
                    h.flags.hop_limit(),
                    h.flags.hop_start(),
                    self.payload.len(),
                    if self.encrypted { " [ENC]" } else { "" }
                )
            }
            None => format!("INVALID ({} bytes)", self.raw.len()),
        }
    }
}

/// Format bytes as hex lines with offset
fn format_hex_lines(out: &mut String, bytes: &[u8], base_offset: usize) {
    const BYTES_PER_LINE: usize = 16;

    for (i, chunk) in bytes.chunks(BYTES_PER_LINE).enumerate() {
        let offset = base_offset + i * BYTES_PER_LINE;

        // Offset
        write!(out, "  {:02x}-{:02x}: ", offset, offset + chunk.len() - 1).unwrap();

        // Hex bytes
        for byte in chunk {
            write!(out, "{:02x} ", byte).unwrap();
        }

        // Padding for incomplete lines
        for _ in chunk.len()..BYTES_PER_LINE {
            write!(out, "   ").unwrap();
        }

        // ASCII representation
        write!(out, " │ ").unwrap();
        for byte in chunk {
            let c = if *byte >= 0x20 && *byte < 0x7F {
                *byte as char
            } else {
                '.'
            };
            write!(out, "{}", c).unwrap();
        }

        writeln!(out).unwrap();
    }
}

/// Parse and analyze a hex string (space-separated or continuous)
pub fn analyze_hex_string(hex: &str) -> Result<PacketAnalysis, String> {
    let cleaned: String = hex
        .chars()
        .filter(|c| c.is_ascii_hexdigit())
        .collect();

    if cleaned.len() % 2 != 0 {
        return Err("Hex string must have even number of characters".into());
    }

    let bytes: Result<Vec<u8>, _> = (0..cleaned.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&cleaned[i..i+2], 16))
        .collect();

    match bytes {
        Ok(data) => Ok(PacketAnalysis::analyze(&data)),
        Err(e) => Err(format!("Invalid hex: {}", e)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_analyze_broadcast() {
        // Create a broadcast packet
        let header = WireHeader::broadcast(0xa1b2c3d4, 0x1234, 3, 0x1a);
        let mut packet = header.to_bytes().to_vec();
        packet.extend_from_slice(b"Hello Mesh!");

        let analysis = PacketAnalysis::analyze(&packet);

        assert!(analysis.header.is_some());
        let h = analysis.header.unwrap();
        assert!(h.is_broadcast());
        assert_eq!(h.from, 0xa1b2c3d4);
        assert_eq!(h.id, 0x1234);
        assert_eq!(h.channel_hash, 0x1a);
        assert_eq!(analysis.payload, b"Hello Mesh!");
    }

    #[test]
    fn test_analyze_short_packet() {
        let analysis = PacketAnalysis::analyze(&[0x01, 0x02, 0x03]);
        assert!(analysis.header.is_none());
        assert!(!analysis.messages.is_empty());
    }

    #[test]
    fn test_hex_dump_format() {
        let header = WireHeader::broadcast(0xaabbccdd, 0x5678, 3, 0xff);
        let mut packet = header.to_bytes().to_vec();
        packet.extend_from_slice(&[0x08, 0x01, 0x12, 0x05, b'T', b'e', b's', b't', b'!']);

        let analysis = PacketAnalysis::analyze(&packet);
        let dump = analysis.format_hex_dump();

        assert!(dump.contains("BROADCAST"));
        assert!(dump.contains("0xaabbccdd"));
        assert!(dump.contains("hop_limit=3"));
        assert!(dump.contains("Payload (9 bytes)"));
    }

    #[test]
    fn test_analyze_hex_string() {
        // A simple 16-byte header only
        let hex = "ff ff ff ff a1 b2 c3 d4 12 34 00 00 63 1a 00 00";
        let result = analyze_hex_string(hex);
        assert!(result.is_ok());

        let analysis = result.unwrap();
        assert!(analysis.header.is_some());
    }

    #[test]
    fn test_summary() {
        let header = WireHeader::broadcast(0xdeadbeef, 0x9999, 5, 0x42);
        let mut packet = header.to_bytes().to_vec();
        packet.extend_from_slice(&[1, 2, 3, 4, 5]);

        let analysis = PacketAnalysis::analyze(&packet);
        let summary = analysis.summary();

        assert!(summary.contains("deadbeef"));
        assert!(summary.contains("BCAST"));
        assert!(summary.contains("5B"));
    }

    #[test]
    fn test_encrypted_detection() {
        let header = WireHeader::broadcast(0x12345678, 0x0001, 3, 0x00);
        let mut packet = header.to_bytes().to_vec();
        // Random-looking bytes (not valid protobuf)
        packet.extend_from_slice(&[0xf3, 0xa2, 0x81, 0xc9, 0x7e, 0xb4]);

        let analysis = PacketAnalysis::analyze(&packet);
        assert!(analysis.encrypted);
    }
}
