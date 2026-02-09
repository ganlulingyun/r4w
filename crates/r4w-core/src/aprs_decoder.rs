//! APRS Decoder — Automatic Packet Reporting System
//!
//! Decodes APRS (Automatic Packet Reporting System) packets used in
//! amateur radio for position reporting, messaging, weather, and
//! telemetry. Operates on 144.390 MHz (NA) / 144.800 MHz (EU) using
//! AX.25 framing over 1200 baud AFSK (Bell 202).
//! GNU Radio equivalent: `gr-satellites` / `direwolf` / `multimon-ng`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::aprs_decoder::{AprsDecoder, AprsPacket, parse_aprs};
//!
//! let raw = "N0CALL>APRS,WIDE1-1:!4903.50N/07201.75W-PHG2360";
//! let packet = parse_aprs(raw).unwrap();
//! assert_eq!(packet.source, "N0CALL");
//! assert_eq!(packet.destination, "APRS");
//! ```

/// APRS packet structure.
#[derive(Debug, Clone)]
pub struct AprsPacket {
    /// Source callsign with SSID.
    pub source: String,
    /// Destination callsign.
    pub destination: String,
    /// Digipeater path.
    pub path: Vec<String>,
    /// Information field content.
    pub info: String,
    /// Parsed data type.
    pub data_type: AprsDataType,
}

/// APRS data type identifier.
#[derive(Debug, Clone, PartialEq)]
pub enum AprsDataType {
    /// Position without timestamp.
    PositionNoTime(AprsPosition),
    /// Position with timestamp.
    PositionWithTime(AprsPosition),
    /// Message.
    Message(AprsMessage),
    /// Weather report.
    Weather(AprsWeather),
    /// Status report.
    Status(String),
    /// Telemetry.
    Telemetry,
    /// MIC-E encoded position.
    MicE,
    /// Object.
    Object,
    /// Raw/unknown.
    Unknown,
}

/// APRS position data.
#[derive(Debug, Clone, PartialEq)]
pub struct AprsPosition {
    /// Latitude in degrees (positive = North).
    pub lat: f64,
    /// Longitude in degrees (positive = East).
    pub lon: f64,
    /// Symbol table identifier.
    pub symbol_table: char,
    /// Symbol code.
    pub symbol_code: char,
    /// Comment/extension text.
    pub comment: String,
}

/// APRS message.
#[derive(Debug, Clone, PartialEq)]
pub struct AprsMessage {
    /// Addressee (9 chars padded).
    pub addressee: String,
    /// Message text.
    pub text: String,
    /// Message number (for ack/rej).
    pub msg_no: Option<String>,
}

/// APRS weather data.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct AprsWeather {
    /// Wind direction in degrees.
    pub wind_dir: Option<u16>,
    /// Wind speed in mph.
    pub wind_speed: Option<u16>,
    /// Wind gust in mph.
    pub wind_gust: Option<u16>,
    /// Temperature in Fahrenheit.
    pub temperature: Option<i16>,
    /// Rainfall last hour in hundredths of an inch.
    pub rain_1h: Option<u16>,
    /// Rainfall last 24h.
    pub rain_24h: Option<u16>,
    /// Humidity (0-100).
    pub humidity: Option<u8>,
    /// Barometric pressure in tenths of millibars.
    pub pressure: Option<u32>,
}

/// APRS decoder with AX.25 framing.
#[derive(Debug, Clone)]
pub struct AprsDecoder {
    /// Decoded packets.
    packets: Vec<AprsPacket>,
    /// Packet count.
    packet_count: usize,
    /// Error count.
    error_count: usize,
}

impl AprsDecoder {
    /// Create new APRS decoder.
    pub fn new() -> Self {
        Self {
            packets: Vec::new(),
            packet_count: 0,
            error_count: 0,
        }
    }

    /// Decode an AX.25 UI frame into an APRS packet.
    pub fn decode_ax25(&mut self, frame: &[u8]) -> Option<AprsPacket> {
        // Minimum AX.25 frame: dest(7) + src(7) + control(1) + pid(1) + info(1)
        if frame.len() < 17 {
            self.error_count += 1;
            return None;
        }

        // Extract destination (bytes 0-6)
        let destination = decode_ax25_callsign(&frame[0..7]);
        // Extract source (bytes 7-13)
        let source = decode_ax25_callsign(&frame[7..14]);

        // Extract digipeater path
        let mut path = Vec::new();
        let mut idx = 14;
        // Check if address extension bit indicates more addresses
        while idx + 7 <= frame.len() && (frame[idx - 1] & 0x01) == 0 {
            if idx + 7 > frame.len() {
                break;
            }
            path.push(decode_ax25_callsign(&frame[idx..idx + 7]));
            idx += 7;
        }

        // Skip control + PID
        idx += 2;
        if idx > frame.len() {
            self.error_count += 1;
            return None;
        }

        let info = String::from_utf8_lossy(&frame[idx..]).to_string();

        let data_type = classify_aprs(&info);

        let packet = AprsPacket {
            source,
            destination,
            path,
            info,
            data_type,
        };

        self.packets.push(packet.clone());
        self.packet_count += 1;
        Some(packet)
    }

    /// Get all decoded packets.
    pub fn packets(&self) -> &[AprsPacket] {
        &self.packets
    }

    /// Packet count.
    pub fn packet_count(&self) -> usize {
        self.packet_count
    }

    /// Error count.
    pub fn error_count(&self) -> usize {
        self.error_count
    }

    /// Reset.
    pub fn reset(&mut self) {
        self.packets.clear();
        self.packet_count = 0;
        self.error_count = 0;
    }
}

impl Default for AprsDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Parse APRS from TNC2 format string: "SRC>DST,PATH:info"
pub fn parse_aprs(tnc2: &str) -> Option<AprsPacket> {
    let (header, info) = tnc2.split_once(':')?;
    let (source_part, rest) = header.split_once('>')?;

    let parts: Vec<&str> = rest.split(',').collect();
    let destination = parts.first()?.to_string();
    let path: Vec<String> = parts[1..].iter().map(|s| s.to_string()).collect();

    let data_type = classify_aprs(info);

    Some(AprsPacket {
        source: source_part.to_string(),
        destination,
        path,
        info: info.to_string(),
        data_type,
    })
}

/// Classify APRS data type from info field.
fn classify_aprs(info: &str) -> AprsDataType {
    if info.is_empty() {
        return AprsDataType::Unknown;
    }

    match info.as_bytes()[0] {
        b'!' | b'=' => {
            // Position without timestamp
            if let Some(pos) = parse_position(&info[1..]) {
                AprsDataType::PositionNoTime(pos)
            } else {
                AprsDataType::Unknown
            }
        }
        b'/' | b'@' => {
            // Position with timestamp
            if info.len() > 8 {
                if let Some(pos) = parse_position(&info[8..]) {
                    AprsDataType::PositionWithTime(pos)
                } else {
                    AprsDataType::Unknown
                }
            } else {
                AprsDataType::Unknown
            }
        }
        b':' => {
            // Message
            if info.len() > 11 && info.as_bytes().get(10) == Some(&b':') {
                let addressee = info[1..10].trim().to_string();
                let rest = &info[11..];
                let (text, msg_no) = if let Some(idx) = rest.rfind('{') {
                    (rest[..idx].to_string(), Some(rest[idx + 1..].to_string()))
                } else {
                    (rest.to_string(), None)
                };
                AprsDataType::Message(AprsMessage {
                    addressee,
                    text,
                    msg_no,
                })
            } else {
                AprsDataType::Unknown
            }
        }
        b'>' => AprsDataType::Status(info[1..].to_string()),
        b'T' => AprsDataType::Telemetry,
        b'\x1c' | b'\x1d' | b'`' | b'\'' => AprsDataType::MicE,
        b';' => AprsDataType::Object,
        _ => AprsDataType::Unknown,
    }
}

/// Parse APRS position from "DDMM.MMN/DDDMM.MMW" format.
fn parse_position(s: &str) -> Option<AprsPosition> {
    if s.len() < 19 {
        return None;
    }

    let lat_deg: f64 = s[0..2].parse().ok()?;
    let lat_min: f64 = s[2..7].parse().ok()?;
    let lat_ns = s.as_bytes()[7] as char;
    let symbol_table = s.as_bytes()[8] as char;
    let lon_deg: f64 = s[9..12].parse().ok()?;
    let lon_min: f64 = s[12..17].parse().ok()?;
    let lon_ew = s.as_bytes()[17] as char;
    let symbol_code = s.as_bytes()[18] as char;

    let mut lat = lat_deg + lat_min / 60.0;
    if lat_ns == 'S' {
        lat = -lat;
    }

    let mut lon = lon_deg + lon_min / 60.0;
    if lon_ew == 'W' {
        lon = -lon;
    }

    let comment = if s.len() > 19 {
        s[19..].to_string()
    } else {
        String::new()
    };

    Some(AprsPosition {
        lat,
        lon,
        symbol_table,
        symbol_code,
        comment,
    })
}

/// Decode AX.25 callsign from 7 bytes.
fn decode_ax25_callsign(data: &[u8]) -> String {
    if data.len() < 7 {
        return String::new();
    }
    let call: String = data[0..6]
        .iter()
        .map(|&b| (b >> 1) as char)
        .collect::<String>()
        .trim()
        .to_string();
    let ssid = (data[6] >> 1) & 0x0F;
    if ssid == 0 {
        call
    } else {
        format!("{}-{}", call, ssid)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_tnc2() {
        let packet = parse_aprs("N0CALL>APRS,WIDE1-1:!4903.50N/07201.75W-Test").unwrap();
        assert_eq!(packet.source, "N0CALL");
        assert_eq!(packet.destination, "APRS");
        assert_eq!(packet.path, vec!["WIDE1-1"]);
        assert!(matches!(packet.data_type, AprsDataType::PositionNoTime(_)));
    }

    #[test]
    fn test_parse_position() {
        let pos = parse_position("4903.50N/07201.75W-PHG2360").unwrap();
        assert!((pos.lat - 49.058333).abs() < 0.001);
        assert!((pos.lon - (-72.029167)).abs() < 0.001);
        assert_eq!(pos.symbol_table, '/');
        assert_eq!(pos.symbol_code, '-');
    }

    #[test]
    fn test_parse_position_south_east() {
        let pos = parse_position("3348.50S/15101.75E/Test").unwrap();
        assert!(pos.lat < 0.0);
        assert!(pos.lon > 0.0);
    }

    #[test]
    fn test_parse_message() {
        let packet = parse_aprs("N0CALL>APRS::BLN1     :Hello World{123").unwrap();
        if let AprsDataType::Message(msg) = &packet.data_type {
            assert_eq!(msg.addressee, "BLN1");
            assert_eq!(msg.text, "Hello World");
            assert_eq!(msg.msg_no, Some("123".to_string()));
        } else {
            panic!("Expected Message type");
        }
    }

    #[test]
    fn test_parse_status() {
        let packet = parse_aprs("N0CALL>APRS:>En route to work").unwrap();
        assert!(matches!(packet.data_type, AprsDataType::Status(_)));
    }

    #[test]
    fn test_decoder_creation() {
        let decoder = AprsDecoder::new();
        assert_eq!(decoder.packet_count(), 0);
        assert_eq!(decoder.error_count(), 0);
        assert!(decoder.packets().is_empty());
    }

    #[test]
    fn test_decoder_reset() {
        let mut decoder = AprsDecoder::new();
        decoder.packet_count = 5;
        decoder.error_count = 2;
        decoder.reset();
        assert_eq!(decoder.packet_count(), 0);
        assert_eq!(decoder.error_count(), 0);
    }

    #[test]
    fn test_decode_ax25_callsign() {
        // "N0CALL" shifted left by 1 = 0x9C, 0x60, 0x86, 0x82, 0x98, 0x98
        let data = [0x9C, 0x60, 0x86, 0x82, 0x98, 0x98, 0x60]; // SSID=0
        let call = decode_ax25_callsign(&data);
        assert_eq!(call, "N0CALL");
    }

    #[test]
    fn test_default_impl() {
        let _decoder = AprsDecoder::default();
    }

    #[test]
    fn test_classify_unknown() {
        let dt = classify_aprs("xyz unknown data");
        assert!(matches!(dt, AprsDataType::Unknown));
    }

    #[test]
    fn test_classify_telemetry() {
        let dt = classify_aprs("T#001,100,200,300,400,500,10101010");
        assert!(matches!(dt, AprsDataType::Telemetry));
    }

    #[test]
    fn test_parse_invalid() {
        assert!(parse_aprs("no_arrow_here").is_none());
        assert!(parse_aprs("SRC>DST").is_none()); // No colon
    }
}
