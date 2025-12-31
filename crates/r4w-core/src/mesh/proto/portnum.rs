//! Meshtastic Port Numbers
//!
//! Port numbers identify the application-layer protocol for a message.
//! These correspond to the `PortNum` enum in Meshtastic protobufs.

use serde::{Deserialize, Serialize};

/// Meshtastic application port numbers
///
/// These identify the type of payload in a Data message.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[repr(u32)]
pub enum PortNum {
    /// Unused/invalid port
    Unknown = 0,
    /// Text message (UTF-8 string)
    Text = 1,
    /// Remote hardware GPIO control
    RemoteHardware = 2,
    /// Position data (lat/lon/alt)
    Position = 3,
    /// Node info (user details)
    NodeInfo = 4,
    /// Routing protocol messages
    Routing = 5,
    /// Admin channel (configuration)
    Admin = 6,
    /// Compressed text message
    TextMessageCompressed = 7,
    /// Waypoint data
    Waypoint = 8,
    /// Audio data
    Audio = 9,
    /// Detection sensor data
    DetectionSensor = 10,
    /// Reply to a message
    Reply = 32,
    /// IP tunnel (TCP/IP over mesh)
    IpTunnel = 33,
    /// Paxcounter (people counting)
    Paxcounter = 34,
    /// Serial port data
    Serial = 64,
    /// Store and forward messages
    StoreForward = 65,
    /// Range test application
    RangeTest = 66,
    /// Telemetry data (device/environment metrics)
    Telemetry = 67,
    /// ZPS (zone/position system)
    Zps = 68,
    /// Simulator data
    Simulator = 69,
    /// Traceroute
    Traceroute = 70,
    /// Neighbor info
    NeighborInfo = 71,
    /// ATAK plugin
    AtakPlugin = 72,
    /// Map report
    MapReport = 73,
    /// Power stress test
    PowerStress = 74,
    /// Private application (start of private range)
    PrivateApp = 256,
    /// ATAK forwarder
    AtakForwarder = 257,
    /// Maximum valid port number
    Max = 511,
}

impl PortNum {
    /// Create from u32 value
    pub fn from_u32(value: u32) -> Self {
        match value {
            0 => PortNum::Unknown,
            1 => PortNum::Text,
            2 => PortNum::RemoteHardware,
            3 => PortNum::Position,
            4 => PortNum::NodeInfo,
            5 => PortNum::Routing,
            6 => PortNum::Admin,
            7 => PortNum::TextMessageCompressed,
            8 => PortNum::Waypoint,
            9 => PortNum::Audio,
            10 => PortNum::DetectionSensor,
            32 => PortNum::Reply,
            33 => PortNum::IpTunnel,
            34 => PortNum::Paxcounter,
            64 => PortNum::Serial,
            65 => PortNum::StoreForward,
            66 => PortNum::RangeTest,
            67 => PortNum::Telemetry,
            68 => PortNum::Zps,
            69 => PortNum::Simulator,
            70 => PortNum::Traceroute,
            71 => PortNum::NeighborInfo,
            72 => PortNum::AtakPlugin,
            73 => PortNum::MapReport,
            74 => PortNum::PowerStress,
            256 => PortNum::PrivateApp,
            257 => PortNum::AtakForwarder,
            511 => PortNum::Max,
            _ => PortNum::Unknown,
        }
    }

    /// Convert to u32 value
    pub fn as_u32(self) -> u32 {
        self as u32
    }

    /// Check if this is a private application port
    pub fn is_private(self) -> bool {
        (self as u32) >= 256 && (self as u32) < 512
    }
}

impl Default for PortNum {
    fn default() -> Self {
        PortNum::Unknown
    }
}

impl From<u32> for PortNum {
    fn from(value: u32) -> Self {
        Self::from_u32(value)
    }
}

impl From<PortNum> for u32 {
    fn from(value: PortNum) -> Self {
        value.as_u32()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_portnum_roundtrip() {
        assert_eq!(PortNum::from_u32(1), PortNum::Text);
        assert_eq!(PortNum::Text.as_u32(), 1);

        assert_eq!(PortNum::from_u32(67), PortNum::Telemetry);
        assert_eq!(PortNum::Telemetry.as_u32(), 67);
    }

    #[test]
    fn test_portnum_unknown() {
        assert_eq!(PortNum::from_u32(999), PortNum::Unknown);
    }

    #[test]
    fn test_portnum_private() {
        assert!(!PortNum::Text.is_private());
        assert!(PortNum::PrivateApp.is_private());
        assert!(PortNum::AtakForwarder.is_private());
    }
}
