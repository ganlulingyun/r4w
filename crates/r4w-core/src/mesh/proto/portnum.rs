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
    /// Alert notification
    Alert = 11,
    /// Key verification for PKC
    KeyVerification = 12,
    /// Reply to a message
    Reply = 32,
    /// IP tunnel (TCP/IP over mesh)
    IpTunnel = 33,
    /// Paxcounter (people counting)
    Paxcounter = 34,
    /// Store and forward++ (improved)
    StoreForwardPlusPlus = 35,
    /// Node status
    NodeStatus = 36,
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
    /// Reticulum tunnel
    ReticulumTunnel = 76,
    /// Cayenne LPP
    Cayenne = 77,
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
            11 => PortNum::Alert,
            12 => PortNum::KeyVerification,
            32 => PortNum::Reply,
            33 => PortNum::IpTunnel,
            34 => PortNum::Paxcounter,
            35 => PortNum::StoreForwardPlusPlus,
            36 => PortNum::NodeStatus,
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
            76 => PortNum::ReticulumTunnel,
            77 => PortNum::Cayenne,
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

    #[test]
    fn test_portnum_values_match_meshtastic() {
        // trace:MESH-014 | ai:claude
        // Verify all PortNum values match official Meshtastic protobufs
        assert_eq!(PortNum::Unknown.as_u32(), 0);
        assert_eq!(PortNum::Text.as_u32(), 1);
        assert_eq!(PortNum::RemoteHardware.as_u32(), 2);
        assert_eq!(PortNum::Position.as_u32(), 3);
        assert_eq!(PortNum::NodeInfo.as_u32(), 4);
        assert_eq!(PortNum::Routing.as_u32(), 5);
        assert_eq!(PortNum::Admin.as_u32(), 6);
        assert_eq!(PortNum::TextMessageCompressed.as_u32(), 7);
        assert_eq!(PortNum::Waypoint.as_u32(), 8);
        assert_eq!(PortNum::Audio.as_u32(), 9);
        assert_eq!(PortNum::DetectionSensor.as_u32(), 10);
        assert_eq!(PortNum::Alert.as_u32(), 11);
        assert_eq!(PortNum::KeyVerification.as_u32(), 12);
        assert_eq!(PortNum::Reply.as_u32(), 32);
        assert_eq!(PortNum::IpTunnel.as_u32(), 33);
        assert_eq!(PortNum::Paxcounter.as_u32(), 34);
        assert_eq!(PortNum::StoreForwardPlusPlus.as_u32(), 35);
        assert_eq!(PortNum::NodeStatus.as_u32(), 36);
        assert_eq!(PortNum::Serial.as_u32(), 64);
        assert_eq!(PortNum::StoreForward.as_u32(), 65);
        assert_eq!(PortNum::RangeTest.as_u32(), 66);
        assert_eq!(PortNum::Telemetry.as_u32(), 67);
        assert_eq!(PortNum::Traceroute.as_u32(), 70);
        assert_eq!(PortNum::NeighborInfo.as_u32(), 71);
        assert_eq!(PortNum::AtakPlugin.as_u32(), 72);
        assert_eq!(PortNum::MapReport.as_u32(), 73);
        assert_eq!(PortNum::PowerStress.as_u32(), 74);
        assert_eq!(PortNum::ReticulumTunnel.as_u32(), 76);
        assert_eq!(PortNum::Cayenne.as_u32(), 77);
        assert_eq!(PortNum::PrivateApp.as_u32(), 256);
        assert_eq!(PortNum::AtakForwarder.as_u32(), 257);
        assert_eq!(PortNum::Max.as_u32(), 511);
    }

    #[test]
    fn test_new_portnum_roundtrip() {
        assert_eq!(PortNum::from_u32(11), PortNum::Alert);
        assert_eq!(PortNum::from_u32(12), PortNum::KeyVerification);
        assert_eq!(PortNum::from_u32(35), PortNum::StoreForwardPlusPlus);
        assert_eq!(PortNum::from_u32(36), PortNum::NodeStatus);
        assert_eq!(PortNum::from_u32(76), PortNum::ReticulumTunnel);
        assert_eq!(PortNum::from_u32(77), PortNum::Cayenne);
    }
}
