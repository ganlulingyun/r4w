//! AX.25 Protocol Decoder
//!
//! Decodes AX.25 amateur radio protocol frames from HDLC data.
//! AX.25 is the data link layer protocol used by APRS, packet radio,
//! and amateur satellite communications.
//!
//! ## Frame Format
//!
//! ```text
//! | Dest Addr (7) | Src Addr (7) | Digipeaters (0-56) | Control (1-2) | PID (1) | Info (N) |
//! ```
//!
//! Address bytes are ASCII shifted left by 1 bit. The SSID byte contains
//! the SSID in bits 1-4 and the address extension bit in bit 0.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ax25::{Ax25Decoder, Ax25Frame, FrameType};
//!
//! let mut decoder = Ax25Decoder::new();
//!
//! // Build a minimal UI frame
//! let mut raw = Vec::new();
//! // Destination: "APRS  " (shifted left 1)
//! raw.extend_from_slice(&[0x82, 0xA0, 0xA4, 0xA6, 0x40, 0x40, 0x60]);
//! // Source: "N0CALL" (shifted left 1, last address bit set)
//! raw.extend_from_slice(&[0x9C, 0x60, 0x86, 0x82, 0x98, 0x98, 0xE1]);
//! // Control: UI (0x03), PID: No layer 3 (0xF0)
//! raw.extend_from_slice(&[0x03, 0xF0]);
//! // Info
//! raw.extend_from_slice(b"Hello APRS!");
//!
//! let frame = decoder.decode(&raw).unwrap();
//! assert_eq!(frame.dest.callsign, "APRS");
//! assert_eq!(frame.source.callsign, "N0CALL");
//! ```

use std::fmt;

/// AX.25 station address (callsign + SSID).
#[derive(Debug, Clone, PartialEq)]
pub struct Ax25Address {
    /// Callsign (up to 6 characters, space-trimmed)
    pub callsign: String,
    /// Secondary Station Identifier (0-15)
    pub ssid: u8,
    /// Has-been-repeated flag (for digipeater addresses)
    pub h_bit: bool,
}

impl Ax25Address {
    /// Decode an AX.25 address from 7 raw bytes.
    fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < 7 {
            return None;
        }

        // Characters are ASCII shifted left 1 bit
        let callsign: String = bytes[..6]
            .iter()
            .map(|&b| (b >> 1) as char)
            .collect::<String>()
            .trim()
            .to_string();

        let ssid_byte = bytes[6];
        let ssid = (ssid_byte >> 1) & 0x0F;
        let h_bit = ssid_byte & 0x80 != 0;

        Some(Self {
            callsign,
            ssid,
            h_bit,
        })
    }

    /// Encode this address into 7 AX.25 bytes.
    pub fn to_bytes(&self, last: bool) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(7);
        let padded = format!("{:<6}", &self.callsign[..self.callsign.len().min(6)]);

        for ch in padded.chars().take(6) {
            bytes.push((ch as u8) << 1);
        }

        let mut ssid_byte = (self.ssid & 0x0F) << 1;
        if self.h_bit {
            ssid_byte |= 0x80;
        }
        // Set reserved bits (0x60)
        ssid_byte |= 0x60;
        if last {
            ssid_byte |= 0x01; // Address extension bit
        }
        bytes.push(ssid_byte);

        bytes
    }
}

impl fmt::Display for Ax25Address {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.ssid > 0 {
            write!(f, "{}-{}", self.callsign, self.ssid)
        } else {
            write!(f, "{}", self.callsign)
        }
    }
}

/// AX.25 frame type.
#[derive(Debug, Clone, PartialEq)]
pub enum FrameType {
    /// Unnumbered Information (UI) frame
    UI,
    /// Set Asynchronous Balanced Mode (SABM/SABME)
    SABM { extended: bool },
    /// Disconnect (DISC)
    DISC,
    /// Disconnected Mode (DM)
    DM,
    /// Unnumbered Acknowledge (UA)
    UA,
    /// Frame Reject (FRMR)
    FRMR,
    /// Information frame (I)
    IFrame { nr: u8, ns: u8, poll: bool },
    /// Receive Ready (RR)
    RR { nr: u8, poll: bool },
    /// Receive Not Ready (RNR)
    RNR { nr: u8, poll: bool },
    /// Reject (REJ)
    REJ { nr: u8, poll: bool },
    /// Unknown control field
    Unknown(u8),
}

/// Protocol Identifier (PID) values.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Pid {
    /// No layer 3 protocol (0xF0)
    NoLayer3,
    /// AX.25 layer 3 (0x01)
    Ax25Layer3,
    /// IP (0xCC)
    Ip,
    /// ARP (0xCD)
    Arp,
    /// Compressed TCP/IP (0x06)
    CompressedTcpIp,
    /// Uncompressed TCP/IP (0x07)
    UncompressedTcpIp,
    /// Other
    Other(u8),
}

impl From<u8> for Pid {
    fn from(byte: u8) -> Self {
        match byte {
            0xF0 => Pid::NoLayer3,
            0x01 => Pid::Ax25Layer3,
            0xCC => Pid::Ip,
            0xCD => Pid::Arp,
            0x06 => Pid::CompressedTcpIp,
            0x07 => Pid::UncompressedTcpIp,
            other => Pid::Other(other),
        }
    }
}

/// A decoded AX.25 frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Ax25Frame {
    /// Destination address
    pub dest: Ax25Address,
    /// Source address
    pub source: Ax25Address,
    /// Digipeater path (0-8 addresses)
    pub digipeaters: Vec<Ax25Address>,
    /// Frame type (decoded from control field)
    pub frame_type: FrameType,
    /// Raw control byte(s)
    pub control: u8,
    /// Protocol Identifier (only for I and UI frames)
    pub pid: Option<Pid>,
    /// Information field
    pub info: Vec<u8>,
}

impl Ax25Frame {
    /// Get the info field as a UTF-8 string (lossy).
    pub fn info_string(&self) -> String {
        String::from_utf8_lossy(&self.info).to_string()
    }

    /// Check if this is an APRS frame (UI frame with PID 0xF0).
    pub fn is_aprs(&self) -> bool {
        self.frame_type == FrameType::UI && self.pid == Some(Pid::NoLayer3)
    }
}

impl fmt::Display for Ax25Frame {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}>{}:", self.source, self.dest)?;
        if !self.digipeaters.is_empty() {
            for (i, digi) in self.digipeaters.iter().enumerate() {
                if i > 0 {
                    write!(f, ",")?;
                }
                write!(f, "{}", digi)?;
                if digi.h_bit {
                    write!(f, "*")?;
                }
            }
            write!(f, ":")?;
        }
        write!(f, "{}", self.info_string())
    }
}

/// AX.25 frame decoder.
#[derive(Debug, Clone)]
pub struct Ax25Decoder {
    /// Total frames decoded
    frames_decoded: usize,
    /// Total decode errors
    decode_errors: usize,
}

impl Ax25Decoder {
    /// Create a new AX.25 decoder.
    pub fn new() -> Self {
        Self {
            frames_decoded: 0,
            decode_errors: 0,
        }
    }

    /// Decode an AX.25 frame from raw bytes (after HDLC deframing).
    pub fn decode(&mut self, data: &[u8]) -> Option<Ax25Frame> {
        // Minimum: dest(7) + src(7) + control(1) = 15 bytes
        if data.len() < 15 {
            self.decode_errors += 1;
            return None;
        }

        // Destination address
        let dest = Ax25Address::from_bytes(&data[0..7])?;

        // Source address
        let source = Ax25Address::from_bytes(&data[7..14])?;

        // Check if source is the last address (bit 0 of SSID byte)
        let mut pos = 14;
        let mut digipeaters = Vec::new();

        if data[13] & 0x01 == 0 {
            // More addresses follow (digipeaters)
            while pos + 7 <= data.len() {
                let digi = Ax25Address::from_bytes(&data[pos..pos + 7])?;
                let is_last = data[pos + 6] & 0x01 != 0;
                digipeaters.push(digi);
                pos += 7;
                if is_last {
                    break;
                }
            }
        }

        if pos >= data.len() {
            self.decode_errors += 1;
            return None;
        }

        // Control field
        let control = data[pos];
        pos += 1;

        let frame_type = Self::decode_control(control);

        // PID (only for I and UI frames)
        let pid = match frame_type {
            FrameType::UI | FrameType::IFrame { .. } => {
                if pos < data.len() {
                    let pid_byte = data[pos];
                    pos += 1;
                    Some(Pid::from(pid_byte))
                } else {
                    None
                }
            }
            _ => None,
        };

        // Info field (rest of data)
        let info = if pos < data.len() {
            data[pos..].to_vec()
        } else {
            Vec::new()
        };

        self.frames_decoded += 1;

        Some(Ax25Frame {
            dest,
            source,
            digipeaters,
            frame_type,
            control,
            pid,
            info,
        })
    }

    /// Decode multiple frames from an HDLC deframer output.
    pub fn decode_frames(&mut self, hdlc_frames: &[Vec<u8>]) -> Vec<Ax25Frame> {
        hdlc_frames
            .iter()
            .filter_map(|frame| self.decode(frame))
            .collect()
    }

    /// Decode the control field into a FrameType.
    fn decode_control(control: u8) -> FrameType {
        if control & 0x01 == 0 {
            // I frame: bit 0 = 0
            let ns = (control >> 1) & 0x07;
            let poll = control & 0x10 != 0;
            let nr = (control >> 5) & 0x07;
            FrameType::IFrame { nr, ns, poll }
        } else if control & 0x02 == 0 {
            // S frame: bits 0-1 = 01
            let nr = (control >> 5) & 0x07;
            let poll = control & 0x10 != 0;
            match (control >> 2) & 0x03 {
                0 => FrameType::RR { nr, poll },
                1 => FrameType::RNR { nr, poll },
                2 => FrameType::REJ { nr, poll },
                _ => FrameType::Unknown(control),
            }
        } else {
            // U frame: bits 0-1 = 11
            match control & 0xEF {
                0x03 => FrameType::UI,
                0x2F => FrameType::SABM { extended: false },
                0x6F => FrameType::SABM { extended: true },
                0x43 => FrameType::DISC,
                0x0F => FrameType::DM,
                0x63 => FrameType::UA,
                0x87 => FrameType::FRMR,
                _ => FrameType::Unknown(control),
            }
        }
    }

    /// Total frames decoded.
    pub fn frames_decoded(&self) -> usize {
        self.frames_decoded
    }

    /// Total decode errors.
    pub fn decode_errors(&self) -> usize {
        self.decode_errors
    }

    /// Reset counters.
    pub fn reset(&mut self) {
        self.frames_decoded = 0;
        self.decode_errors = 0;
    }
}

impl Default for Ax25Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_ui_frame(dest: &str, src: &str, info: &[u8]) -> Vec<u8> {
        let dest_addr = Ax25Address {
            callsign: dest.to_string(),
            ssid: 0,
            h_bit: false,
        };
        let src_addr = Ax25Address {
            callsign: src.to_string(),
            ssid: 0,
            h_bit: false,
        };

        let mut frame = Vec::new();
        frame.extend(dest_addr.to_bytes(false));
        frame.extend(src_addr.to_bytes(true));
        frame.push(0x03); // UI control
        frame.push(0xF0); // No layer 3
        frame.extend_from_slice(info);
        frame
    }

    #[test]
    fn test_decode_ui_frame() {
        let mut decoder = Ax25Decoder::new();
        let raw = make_ui_frame("APRS", "N0CALL", b"!4903.50N/07201.75W-Test");
        let frame = decoder.decode(&raw).unwrap();

        assert_eq!(frame.dest.callsign, "APRS");
        assert_eq!(frame.source.callsign, "N0CALL");
        assert_eq!(frame.frame_type, FrameType::UI);
        assert_eq!(frame.pid, Some(Pid::NoLayer3));
        assert_eq!(frame.info, b"!4903.50N/07201.75W-Test");
        assert!(frame.is_aprs());
    }

    #[test]
    fn test_decode_with_ssid() {
        let mut decoder = Ax25Decoder::new();

        let dest = Ax25Address {
            callsign: "APRS".into(),
            ssid: 0,
            h_bit: false,
        };
        let src = Ax25Address {
            callsign: "W1AW".into(),
            ssid: 7,
            h_bit: false,
        };

        let mut raw = Vec::new();
        raw.extend(dest.to_bytes(false));
        raw.extend(src.to_bytes(true));
        raw.push(0x03);
        raw.push(0xF0);
        raw.extend_from_slice(b"Test");

        let frame = decoder.decode(&raw).unwrap();
        assert_eq!(frame.source.callsign, "W1AW");
        assert_eq!(frame.source.ssid, 7);
    }

    #[test]
    fn test_decode_with_digipeaters() {
        let mut decoder = Ax25Decoder::new();

        let dest = Ax25Address {
            callsign: "APRS".into(),
            ssid: 0,
            h_bit: false,
        };
        let src = Ax25Address {
            callsign: "N0CALL".into(),
            ssid: 0,
            h_bit: false,
        };
        let digi1 = Ax25Address {
            callsign: "WIDE1".into(),
            ssid: 1,
            h_bit: true,
        };
        let digi2 = Ax25Address {
            callsign: "WIDE2".into(),
            ssid: 1,
            h_bit: false,
        };

        let mut raw = Vec::new();
        raw.extend(dest.to_bytes(false));
        raw.extend(src.to_bytes(false)); // not last (digipeaters follow)
        raw.extend(digi1.to_bytes(false));
        raw.extend(digi2.to_bytes(true)); // last address
        raw.push(0x03);
        raw.push(0xF0);
        raw.extend_from_slice(b"Test");

        let frame = decoder.decode(&raw).unwrap();
        assert_eq!(frame.digipeaters.len(), 2);
        assert_eq!(frame.digipeaters[0].callsign, "WIDE1");
        assert!(frame.digipeaters[0].h_bit);
        assert_eq!(frame.digipeaters[1].callsign, "WIDE2");
    }

    #[test]
    fn test_i_frame() {
        let mut decoder = Ax25Decoder::new();

        let mut raw = make_ui_frame("TEST", "N0CALL", b"");
        // Replace control byte: I frame with N(S)=3, N(R)=5, P=1
        // ns=3 → bits 1-3 = 011, poll=1 → bit 4, nr=5 → bits 5-7 = 101
        // = 0b10110110 = 0xB6
        let ctrl_pos = 14; // after dest(7) + src(7)
        raw[ctrl_pos] = 0xB6;
        // PID is already there from make_ui_frame

        let frame = decoder.decode(&raw).unwrap();
        assert_eq!(
            frame.frame_type,
            FrameType::IFrame {
                nr: 5,
                ns: 3,
                poll: true
            }
        );
    }

    #[test]
    fn test_s_frames() {
        let mut decoder = Ax25Decoder::new();

        // RR: 0b_nr_P_00_01
        let mut raw = make_ui_frame("TEST", "N0CALL", b"");
        raw[14] = 0x01; // RR, nr=0, poll=0
        raw.remove(15); // Remove PID (S frames don't have it)
        let frame = decoder.decode(&raw).unwrap();
        assert_eq!(
            frame.frame_type,
            FrameType::RR {
                nr: 0,
                poll: false
            }
        );
    }

    #[test]
    fn test_address_encoding() {
        let addr = Ax25Address {
            callsign: "N0CALL".into(),
            ssid: 5,
            h_bit: false,
        };
        let bytes = addr.to_bytes(true);
        assert_eq!(bytes.len(), 7);

        // Decode back
        let decoded = Ax25Address::from_bytes(&bytes).unwrap();
        assert_eq!(decoded.callsign, "N0CALL");
        assert_eq!(decoded.ssid, 5);
    }

    #[test]
    fn test_display() {
        let mut decoder = Ax25Decoder::new();
        let raw = make_ui_frame("APRS", "N0CALL", b"Hello!");
        let frame = decoder.decode(&raw).unwrap();

        let display = format!("{}", frame);
        assert!(display.contains("N0CALL"));
        assert!(display.contains("APRS"));
        assert!(display.contains("Hello!"));
    }

    #[test]
    fn test_too_short() {
        let mut decoder = Ax25Decoder::new();
        assert!(decoder.decode(&[0; 10]).is_none());
        assert_eq!(decoder.decode_errors(), 1);
    }

    #[test]
    fn test_pid_types() {
        assert_eq!(Pid::from(0xF0), Pid::NoLayer3);
        assert_eq!(Pid::from(0xCC), Pid::Ip);
        assert_eq!(Pid::from(0xCD), Pid::Arp);
        assert_eq!(Pid::from(0x42), Pid::Other(0x42));
    }

    #[test]
    fn test_reset() {
        let mut decoder = Ax25Decoder::new();
        let raw = make_ui_frame("APRS", "N0CALL", b"Test");
        decoder.decode(&raw);
        assert_eq!(decoder.frames_decoded(), 1);
        decoder.reset();
        assert_eq!(decoder.frames_decoded(), 0);
    }
}
