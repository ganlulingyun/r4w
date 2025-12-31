//! Meshtastic Protobuf Messages
//!
//! Defines the core message types using prost derive macros.
//! These are compatible with Meshtastic's protobuf definitions.

use super::PortNum;
use prost::Message;

/// Data message - wrapper for all application payloads
///
/// This is the decrypted payload inside a MeshPacket.
#[derive(Clone, PartialEq, Message)]
pub struct Data {
    /// Port number identifying the payload type
    #[prost(enumeration = "i32", tag = "1")]
    pub portnum: i32,

    /// The actual payload bytes
    #[prost(bytes = "vec", tag = "2")]
    pub payload: Vec<u8>,

    /// Whether this wants an ACK
    #[prost(bool, tag = "3")]
    pub want_response: bool,

    /// Destination node (for directed messages)
    #[prost(fixed32, tag = "4")]
    pub dest: u32,

    /// Source node
    #[prost(fixed32, tag = "5")]
    pub source: u32,

    /// Request ID for tracking responses
    #[prost(fixed32, tag = "6")]
    pub request_id: u32,

    /// Reply ID (if this is a response)
    #[prost(fixed32, tag = "7")]
    pub reply_id: u32,

    /// Emoji reaction (for text messages)
    #[prost(fixed32, tag = "8")]
    pub emoji: u32,
}

impl Data {
    /// Create a new Data message with the given port and payload
    pub fn new(portnum: PortNum, payload: Vec<u8>) -> Self {
        Self {
            portnum: portnum.as_u32() as i32,
            payload,
            want_response: false,
            dest: 0,
            source: 0,
            request_id: 0,
            reply_id: 0,
            emoji: 0,
        }
    }

    /// Create a text message
    pub fn text(message: &str) -> Self {
        Self::new(PortNum::Text, message.as_bytes().to_vec())
    }

    /// Create a position message
    pub fn position(pos: Position) -> Self {
        Self::new(PortNum::Position, pos.encode_to_vec())
    }

    /// Create a user/nodeinfo message
    pub fn user(user: User) -> Self {
        Self::new(PortNum::NodeInfo, user.encode_to_vec())
    }

    /// Create a telemetry message
    pub fn telemetry(telemetry: Telemetry) -> Self {
        Self::new(PortNum::Telemetry, telemetry.encode_to_vec())
    }

    /// Get the port number as enum
    pub fn port(&self) -> PortNum {
        PortNum::from_u32(self.portnum as u32)
    }

    /// Decode the payload as a Position message
    pub fn decode_position(&self) -> Option<Position> {
        if self.port() == PortNum::Position {
            Position::decode(self.payload.as_slice()).ok()
        } else {
            None
        }
    }

    /// Decode the payload as a User message
    pub fn decode_user(&self) -> Option<User> {
        if self.port() == PortNum::NodeInfo {
            User::decode(self.payload.as_slice()).ok()
        } else {
            None
        }
    }

    /// Decode the payload as a Telemetry message
    pub fn decode_telemetry(&self) -> Option<Telemetry> {
        if self.port() == PortNum::Telemetry {
            Telemetry::decode(self.payload.as_slice()).ok()
        } else {
            None
        }
    }

    /// Decode the payload as a text string
    pub fn decode_text(&self) -> Option<String> {
        if self.port() == PortNum::Text {
            String::from_utf8(self.payload.clone()).ok()
        } else {
            None
        }
    }
}

/// Position message - GPS coordinates
#[derive(Clone, PartialEq, Message)]
pub struct Position {
    /// Latitude in degrees * 1e7 (integer for precision)
    #[prost(sfixed32, tag = "1")]
    pub latitude_i: i32,

    /// Longitude in degrees * 1e7 (integer for precision)
    #[prost(sfixed32, tag = "2")]
    pub longitude_i: i32,

    /// Altitude in meters above sea level
    #[prost(int32, tag = "3")]
    pub altitude: i32,

    /// Time the position was recorded (seconds since 1970)
    #[prost(fixed32, tag = "4")]
    pub time: u32,

    /// Location source (GPS, manual, etc.)
    #[prost(enumeration = "i32", tag = "5")]
    pub location_source: i32,

    /// Altitude source
    #[prost(enumeration = "i32", tag = "6")]
    pub altitude_source: i32,

    /// GPS timestamp (for precision timing)
    #[prost(fixed32, tag = "7")]
    pub timestamp: u32,

    /// Timestamp in milliseconds since boot
    #[prost(int32, tag = "8")]
    pub timestamp_millis_adjust: i32,

    /// Altitude in meters * 1000 (HAE - Height Above Ellipsoid)
    #[prost(sint32, tag = "9")]
    pub altitude_hae: i32,

    /// Geoidal separation in meters * 1000
    #[prost(sint32, tag = "10")]
    pub altitude_geoidal_separation: i32,

    /// Position Dilution of Precision * 100
    #[prost(uint32, tag = "11")]
    pub pdop: u32,

    /// Horizontal Dilution of Precision * 100
    #[prost(uint32, tag = "12")]
    pub hdop: u32,

    /// Vertical Dilution of Precision * 100
    #[prost(uint32, tag = "13")]
    pub vdop: u32,

    /// GPS accuracy estimate in mm
    #[prost(uint32, tag = "14")]
    pub gps_accuracy: u32,

    /// Ground speed in m/s * 100
    #[prost(uint32, tag = "15")]
    pub ground_speed: u32,

    /// Ground track (heading) in degrees * 100000
    #[prost(uint32, tag = "16")]
    pub ground_track: u32,

    /// Number of satellites used
    #[prost(uint32, tag = "17")]
    pub sats_in_view: u32,

    /// Sensor ID (for multi-sensor setups)
    #[prost(uint32, tag = "18")]
    pub sensor_id: u32,

    /// Sequence number for this position
    #[prost(uint32, tag = "19")]
    pub seq_number: u32,

    /// Precision bits for latitude
    #[prost(int32, tag = "20")]
    pub precision_bits: i32,
}

impl Position {
    /// Create a position from floating point coordinates
    pub fn from_coords(lat: f64, lon: f64, alt: i32) -> Self {
        Self {
            latitude_i: (lat * 1e7) as i32,
            longitude_i: (lon * 1e7) as i32,
            altitude: alt,
            ..Default::default()
        }
    }

    /// Get latitude as floating point degrees
    pub fn latitude(&self) -> f64 {
        self.latitude_i as f64 / 1e7
    }

    /// Get longitude as floating point degrees
    pub fn longitude(&self) -> f64 {
        self.longitude_i as f64 / 1e7
    }
}

/// User message - node information
#[derive(Clone, PartialEq, Message)]
pub struct User {
    /// Node ID as string (usually hex like "!aabbccdd")
    #[prost(string, tag = "1")]
    pub id: String,

    /// Long name (up to 40 chars)
    #[prost(string, tag = "2")]
    pub long_name: String,

    /// Short name (up to 4 chars, displayed on device)
    #[prost(string, tag = "3")]
    pub short_name: String,

    /// MAC address (for BLE/WiFi)
    #[prost(bytes = "vec", tag = "4")]
    pub macaddr: Vec<u8>,

    /// Hardware model enum
    #[prost(enumeration = "i32", tag = "5")]
    pub hw_model: i32,

    /// Whether this node is licensed (ham radio)
    #[prost(bool, tag = "6")]
    pub is_licensed: bool,

    /// Role of this node in the mesh
    #[prost(enumeration = "i32", tag = "7")]
    pub role: i32,
}

impl User {
    /// Create a new user with names
    pub fn new(id: &str, short_name: &str, long_name: &str) -> Self {
        Self {
            id: id.to_string(),
            short_name: short_name.chars().take(4).collect(),
            long_name: long_name.chars().take(40).collect(),
            ..Default::default()
        }
    }
}

/// Telemetry message - device and environment metrics
#[derive(Clone, PartialEq, Message)]
pub struct Telemetry {
    /// Timestamp when telemetry was recorded
    #[prost(fixed32, tag = "1")]
    pub time: u32,

    /// The telemetry variant (oneof in protobuf)
    #[prost(oneof = "TelemetryVariant", tags = "2, 3, 4, 5, 6")]
    pub variant: Option<TelemetryVariant>,
}

/// Telemetry variant - one of several metric types
#[derive(Clone, PartialEq, prost::Oneof)]
pub enum TelemetryVariant {
    /// Device metrics (battery, uptime, etc.)
    #[prost(message, tag = "2")]
    DeviceMetrics(DeviceMetrics),

    /// Environment metrics (temperature, humidity, etc.)
    #[prost(message, tag = "3")]
    EnvironmentMetrics(EnvironmentMetrics),

    /// Air quality metrics
    #[prost(message, tag = "4")]
    AirQualityMetrics(AirQualityMetrics),

    /// Power metrics (voltage, current per channel)
    #[prost(message, tag = "5")]
    PowerMetrics(PowerMetrics),

    /// Local stats
    #[prost(message, tag = "6")]
    LocalStats(LocalStats),
}

/// Device metrics - battery, uptime, channel utilization
#[derive(Clone, PartialEq, Message)]
pub struct DeviceMetrics {
    /// Battery level (0-100, or 101 for powered)
    #[prost(uint32, tag = "1")]
    pub battery_level: u32,

    /// Battery voltage in volts
    #[prost(float, tag = "2")]
    pub voltage: f32,

    /// Channel utilization (0.0 - 1.0)
    #[prost(float, tag = "3")]
    pub channel_utilization: f32,

    /// Air utilization TX (0.0 - 1.0)
    #[prost(float, tag = "4")]
    pub air_util_tx: f32,

    /// Uptime in seconds
    #[prost(uint32, tag = "5")]
    pub uptime_seconds: u32,
}

impl DeviceMetrics {
    /// Create from our internal DeviceMetrics type
    pub fn from_internal(dm: &super::super::telemetry::DeviceMetrics) -> Self {
        Self {
            battery_level: dm.battery_level.unwrap_or(0) as u32,
            voltage: dm.voltage.unwrap_or(0.0),
            channel_utilization: dm.channel_utilization.unwrap_or(0.0),
            air_util_tx: dm.air_util_tx.unwrap_or(0.0),
            uptime_seconds: dm.uptime_seconds.unwrap_or(0),
        }
    }

    /// Convert to internal DeviceMetrics type
    pub fn to_internal(&self) -> super::super::telemetry::DeviceMetrics {
        super::super::telemetry::DeviceMetrics {
            battery_level: if self.battery_level > 0 {
                Some(self.battery_level.min(100) as u8)
            } else {
                None
            },
            voltage: if self.voltage > 0.0 {
                Some(self.voltage)
            } else {
                None
            },
            channel_utilization: if self.channel_utilization > 0.0 {
                Some(self.channel_utilization)
            } else {
                None
            },
            air_util_tx: if self.air_util_tx > 0.0 {
                Some(self.air_util_tx)
            } else {
                None
            },
            uptime_seconds: if self.uptime_seconds > 0 {
                Some(self.uptime_seconds)
            } else {
                None
            },
        }
    }
}

/// Environment metrics - temperature, humidity, pressure
#[derive(Clone, PartialEq, Message)]
pub struct EnvironmentMetrics {
    /// Temperature in Celsius
    #[prost(float, tag = "1")]
    pub temperature: f32,

    /// Relative humidity (0-100%)
    #[prost(float, tag = "2")]
    pub relative_humidity: f32,

    /// Barometric pressure in hPa
    #[prost(float, tag = "3")]
    pub barometric_pressure: f32,

    /// Gas resistance in ohms (for air quality)
    #[prost(float, tag = "4")]
    pub gas_resistance: f32,

    /// Voltage (for environment sensor)
    #[prost(float, tag = "5")]
    pub voltage: f32,

    /// Current in mA
    #[prost(float, tag = "6")]
    pub current: f32,

    /// IAQ (Indoor Air Quality) index
    #[prost(uint32, tag = "7")]
    pub iaq: u32,

    /// Distance in mm (for range sensors)
    #[prost(float, tag = "8")]
    pub distance: f32,

    /// Lux (light level)
    #[prost(float, tag = "9")]
    pub lux: f32,

    /// White light level
    #[prost(float, tag = "10")]
    pub white_lux: f32,

    /// IR light level
    #[prost(float, tag = "11")]
    pub ir_lux: f32,

    /// UV light level
    #[prost(float, tag = "12")]
    pub uv_lux: f32,

    /// Wind direction in degrees
    #[prost(uint32, tag = "13")]
    pub wind_direction: u32,

    /// Wind speed in m/s
    #[prost(float, tag = "14")]
    pub wind_speed: f32,

    /// Weight in kg
    #[prost(float, tag = "15")]
    pub weight: f32,

    /// Wind gust speed in m/s
    #[prost(float, tag = "16")]
    pub wind_gust: f32,

    /// Wind lull (minimum) in m/s
    #[prost(float, tag = "17")]
    pub wind_lull: f32,
}

impl EnvironmentMetrics {
    /// Create from our internal EnvironmentMetrics type
    pub fn from_internal(em: &super::super::telemetry::EnvironmentMetrics) -> Self {
        Self {
            temperature: em.temperature.unwrap_or(0.0),
            relative_humidity: em.relative_humidity.unwrap_or(0.0),
            barometric_pressure: em.barometric_pressure.unwrap_or(0.0),
            gas_resistance: em.gas_resistance.unwrap_or(0.0),
            ..Default::default()
        }
    }

    /// Convert to internal EnvironmentMetrics type
    pub fn to_internal(&self) -> super::super::telemetry::EnvironmentMetrics {
        super::super::telemetry::EnvironmentMetrics {
            temperature: if self.temperature != 0.0 {
                Some(self.temperature)
            } else {
                None
            },
            relative_humidity: if self.relative_humidity != 0.0 {
                Some(self.relative_humidity)
            } else {
                None
            },
            barometric_pressure: if self.barometric_pressure != 0.0 {
                Some(self.barometric_pressure)
            } else {
                None
            },
            gas_resistance: if self.gas_resistance != 0.0 {
                Some(self.gas_resistance)
            } else {
                None
            },
            iaq: if self.iaq != 0 {
                Some((self.iaq as u16).min(u16::MAX))
            } else {
                None
            },
            distance: if self.distance != 0.0 {
                Some(self.distance)
            } else {
                None
            },
            lux: if self.lux != 0.0 {
                Some(self.lux)
            } else {
                None
            },
            uv_index: if self.uv_lux != 0.0 {
                Some(self.uv_lux)
            } else {
                None
            },
            wind_speed: if self.wind_speed != 0.0 {
                Some(self.wind_speed)
            } else {
                None
            },
            wind_direction: if self.wind_direction != 0 {
                Some(self.wind_direction as u16)
            } else {
                None
            },
            weight: if self.weight != 0.0 {
                Some(self.weight)
            } else {
                None
            },
        }
    }
}

/// Air quality metrics
#[derive(Clone, PartialEq, Message)]
pub struct AirQualityMetrics {
    /// PM1.0 standard (µg/m³)
    #[prost(uint32, tag = "1")]
    pub pm10_standard: u32,

    /// PM2.5 standard (µg/m³)
    #[prost(uint32, tag = "2")]
    pub pm25_standard: u32,

    /// PM10.0 standard (µg/m³)
    #[prost(uint32, tag = "3")]
    pub pm100_standard: u32,

    /// PM1.0 environmental
    #[prost(uint32, tag = "4")]
    pub pm10_environmental: u32,

    /// PM2.5 environmental
    #[prost(uint32, tag = "5")]
    pub pm25_environmental: u32,

    /// PM10.0 environmental
    #[prost(uint32, tag = "6")]
    pub pm100_environmental: u32,

    /// Particle count 0.3µm per 0.1L
    #[prost(uint32, tag = "7")]
    pub particles_03um: u32,

    /// Particle count 0.5µm per 0.1L
    #[prost(uint32, tag = "8")]
    pub particles_05um: u32,

    /// Particle count 1.0µm per 0.1L
    #[prost(uint32, tag = "9")]
    pub particles_10um: u32,

    /// Particle count 2.5µm per 0.1L
    #[prost(uint32, tag = "10")]
    pub particles_25um: u32,

    /// Particle count 5.0µm per 0.1L
    #[prost(uint32, tag = "11")]
    pub particles_50um: u32,

    /// Particle count 10.0µm per 0.1L
    #[prost(uint32, tag = "12")]
    pub particles_100um: u32,

    /// CO2 in ppm
    #[prost(uint32, tag = "13")]
    pub co2: u32,
}

/// Power metrics - per-channel voltage and current
#[derive(Clone, PartialEq, Message)]
pub struct PowerMetrics {
    /// Channel 1 voltage
    #[prost(float, tag = "1")]
    pub ch1_voltage: f32,

    /// Channel 1 current in mA
    #[prost(float, tag = "2")]
    pub ch1_current: f32,

    /// Channel 2 voltage
    #[prost(float, tag = "3")]
    pub ch2_voltage: f32,

    /// Channel 2 current in mA
    #[prost(float, tag = "4")]
    pub ch2_current: f32,

    /// Channel 3 voltage
    #[prost(float, tag = "5")]
    pub ch3_voltage: f32,

    /// Channel 3 current in mA
    #[prost(float, tag = "6")]
    pub ch3_current: f32,
}

/// Local statistics
#[derive(Clone, PartialEq, Message)]
pub struct LocalStats {
    /// Uptime in seconds
    #[prost(uint32, tag = "1")]
    pub uptime_seconds: u32,

    /// Channel utilization percentage
    #[prost(float, tag = "2")]
    pub channel_utilization: f32,

    /// Air utilization TX percentage
    #[prost(float, tag = "3")]
    pub air_util_tx: f32,

    /// Number of packets transmitted
    #[prost(uint32, tag = "4")]
    pub num_packets_tx: u32,

    /// Number of packets received (good)
    #[prost(uint32, tag = "5")]
    pub num_packets_rx: u32,

    /// Number of packets received (bad CRC)
    #[prost(uint32, tag = "6")]
    pub num_packets_rx_bad: u32,

    /// Number of online nodes
    #[prost(uint32, tag = "7")]
    pub num_online_nodes: u32,

    /// Number of total nodes
    #[prost(uint32, tag = "8")]
    pub num_total_nodes: u32,
}

/// Routing message - used for ACKs and routing errors
#[derive(Clone, PartialEq, Message)]
pub struct Routing {
    /// Routing variant
    #[prost(oneof = "RoutingVariant", tags = "1, 2")]
    pub variant: Option<RoutingVariant>,
}

/// Routing variant
#[derive(Clone, PartialEq, prost::Oneof)]
pub enum RoutingVariant {
    /// Route request
    #[prost(message, tag = "1")]
    RouteRequest(RouteDiscovery),

    /// Route reply/error
    #[prost(message, tag = "2")]
    RouteReply(RouteDiscovery),
}

/// Route discovery message
#[derive(Clone, PartialEq, Message)]
pub struct RouteDiscovery {
    /// Route through these nodes
    #[prost(fixed32, repeated, tag = "1")]
    pub route: Vec<u32>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_data_text_roundtrip() {
        let data = Data::text("Hello Meshtastic!");
        let bytes = data.encode_to_vec();

        let decoded = Data::decode(bytes.as_slice()).unwrap();
        assert_eq!(decoded.port(), PortNum::Text);
        assert_eq!(decoded.decode_text(), Some("Hello Meshtastic!".to_string()));
    }

    #[test]
    fn test_position_roundtrip() {
        let pos = Position::from_coords(37.422, -122.084, 10);
        let data = Data::position(pos.clone());
        let bytes = data.encode_to_vec();

        let decoded = Data::decode(bytes.as_slice()).unwrap();
        assert_eq!(decoded.port(), PortNum::Position);

        let decoded_pos = decoded.decode_position().unwrap();
        assert!((decoded_pos.latitude() - 37.422).abs() < 0.0001);
        assert!((decoded_pos.longitude() - (-122.084)).abs() < 0.0001);
        assert_eq!(decoded_pos.altitude, 10);
    }

    #[test]
    fn test_user_roundtrip() {
        let user = User::new("!aabbccdd", "TEST", "Test Node");
        let data = Data::user(user);
        let bytes = data.encode_to_vec();

        let decoded = Data::decode(bytes.as_slice()).unwrap();
        assert_eq!(decoded.port(), PortNum::NodeInfo);

        let decoded_user = decoded.decode_user().unwrap();
        assert_eq!(decoded_user.id, "!aabbccdd");
        assert_eq!(decoded_user.short_name, "TEST");
        assert_eq!(decoded_user.long_name, "Test Node");
    }

    #[test]
    fn test_device_metrics_roundtrip() {
        let dm = DeviceMetrics {
            battery_level: 85,
            voltage: 3.7,
            channel_utilization: 0.15,
            air_util_tx: 0.05,
            uptime_seconds: 3600,
        };

        let telemetry = Telemetry {
            time: 12345,
            variant: Some(TelemetryVariant::DeviceMetrics(dm.clone())),
        };

        let data = Data::telemetry(telemetry);
        let bytes = data.encode_to_vec();

        let decoded = Data::decode(bytes.as_slice()).unwrap();
        assert_eq!(decoded.port(), PortNum::Telemetry);

        let decoded_telemetry = decoded.decode_telemetry().unwrap();
        assert_eq!(decoded_telemetry.time, 12345);

        if let Some(TelemetryVariant::DeviceMetrics(decoded_dm)) = decoded_telemetry.variant {
            assert_eq!(decoded_dm.battery_level, 85);
            assert!((decoded_dm.voltage - 3.7).abs() < 0.01);
        } else {
            panic!("Expected DeviceMetrics variant");
        }
    }

    #[test]
    fn test_environment_metrics_roundtrip() {
        let em = EnvironmentMetrics {
            temperature: 22.5,
            relative_humidity: 65.0,
            barometric_pressure: 1013.25,
            ..Default::default()
        };

        let telemetry = Telemetry {
            time: 12345,
            variant: Some(TelemetryVariant::EnvironmentMetrics(em)),
        };

        let bytes = telemetry.encode_to_vec();
        let decoded = Telemetry::decode(bytes.as_slice()).unwrap();

        if let Some(TelemetryVariant::EnvironmentMetrics(decoded_em)) = decoded.variant {
            assert!((decoded_em.temperature - 22.5).abs() < 0.01);
            assert!((decoded_em.relative_humidity - 65.0).abs() < 0.01);
        } else {
            panic!("Expected EnvironmentMetrics variant");
        }
    }
}
