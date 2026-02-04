//! GNSS signal types and data structures
//!
//! Common types used across all GNSS constellation implementations.

use serde::{Deserialize, Serialize};

/// GNSS constellation identifier
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum GnssConstellation {
    /// US Global Positioning System
    Gps,
    /// Russian GLONASS
    Glonass,
    /// European Galileo
    Galileo,
    /// Chinese BeiDou
    BeiDou,
}

impl std::fmt::Display for GnssConstellation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Gps => write!(f, "GPS"),
            Self::Glonass => write!(f, "GLONASS"),
            Self::Galileo => write!(f, "Galileo"),
            Self::BeiDou => write!(f, "BeiDou"),
        }
    }
}

/// GNSS signal type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum GnssSignal {
    /// GPS L1 C/A (1575.42 MHz, BPSK(1))
    GpsL1Ca,
    /// GPS L5 (1176.45 MHz, QPSK, 10.23 Mchip/s)
    GpsL5,
    /// GLONASS L1OF (1602 MHz + k*562.5 kHz, FDMA)
    GlonassL1of,
    /// Galileo E1 (1575.42 MHz, CBOC(6,1,1/11))
    GalileoE1,
}

impl std::fmt::Display for GnssSignal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::GpsL1Ca => write!(f, "GPS-L1CA"),
            Self::GpsL5 => write!(f, "GPS-L5"),
            Self::GlonassL1of => write!(f, "GLONASS-L1OF"),
            Self::GalileoE1 => write!(f, "Galileo-E1"),
        }
    }
}

impl GnssSignal {
    /// Get the constellation for this signal
    pub fn constellation(&self) -> GnssConstellation {
        match self {
            Self::GpsL1Ca | Self::GpsL5 => GnssConstellation::Gps,
            Self::GlonassL1of => GnssConstellation::Glonass,
            Self::GalileoE1 => GnssConstellation::Galileo,
        }
    }

    /// Get the carrier frequency in Hz
    pub fn carrier_frequency_hz(&self) -> f64 {
        match self {
            Self::GpsL1Ca => 1_575_420_000.0,
            Self::GpsL5 => 1_176_450_000.0,
            Self::GlonassL1of => 1_602_000_000.0, // Center, actual = 1602 + k*0.5625 MHz
            Self::GalileoE1 => 1_575_420_000.0,
        }
    }

    /// Get the chipping rate in chips/second
    pub fn chipping_rate(&self) -> f64 {
        match self {
            Self::GpsL1Ca => 1_023_000.0,
            Self::GpsL5 => 10_230_000.0,
            Self::GlonassL1of => 511_000.0,
            Self::GalileoE1 => 1_023_000.0,
        }
    }

    /// Get the code length in chips
    pub fn code_length(&self) -> usize {
        match self {
            Self::GpsL1Ca => 1023,
            Self::GpsL5 => 10230,
            Self::GlonassL1of => 511,
            Self::GalileoE1 => 4092,
        }
    }

    /// Get the code period in seconds
    pub fn code_period_s(&self) -> f64 {
        self.code_length() as f64 / self.chipping_rate()
    }

    /// Get the navigation data rate in bps
    pub fn nav_data_rate_bps(&self) -> f64 {
        match self {
            Self::GpsL1Ca => 50.0,
            Self::GpsL5 => 50.0,
            Self::GlonassL1of => 50.0,
            Self::GalileoE1 => 250.0,
        }
    }
}

/// PRN (Pseudo-Random Noise) identifier
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PrnId {
    /// Constellation
    pub constellation: GnssConstellation,
    /// PRN number (1-based: GPS 1-32, GLONASS 1-24, Galileo 1-50)
    pub number: u8,
}

impl PrnId {
    pub fn gps(number: u8) -> Self {
        assert!(number >= 1 && number <= 32, "GPS PRN must be 1-32");
        Self { constellation: GnssConstellation::Gps, number }
    }

    pub fn glonass(number: u8) -> Self {
        assert!(number >= 1 && number <= 24, "GLONASS PRN must be 1-24");
        Self { constellation: GnssConstellation::Glonass, number }
    }

    pub fn galileo(number: u8) -> Self {
        assert!(number >= 1 && number <= 50, "Galileo PRN must be 1-50");
        Self { constellation: GnssConstellation::Galileo, number }
    }
}

impl std::fmt::Display for PrnId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}{:02}", match self.constellation {
            GnssConstellation::Gps => "G",
            GnssConstellation::Glonass => "R",
            GnssConstellation::Galileo => "E",
            GnssConstellation::BeiDou => "C",
        }, self.number)
    }
}

/// Result of signal acquisition (2D search)
#[derive(Debug, Clone, Serialize)]
pub struct AcquisitionResult {
    /// PRN that was searched
    pub prn: u8,
    /// Whether the signal was detected
    pub detected: bool,
    /// Estimated code phase in chips (0 to code_length-1)
    pub code_phase: f64,
    /// Estimated Doppler frequency in Hz
    pub doppler_hz: f64,
    /// Peak-to-noise ratio (detection metric)
    pub peak_metric: f64,
    /// Detection threshold used
    pub threshold: f64,
    /// Carrier-to-noise density ratio estimate (dB-Hz)
    pub cn0_estimate: Option<f64>,
}

/// State of a tracking channel
#[derive(Debug, Clone, Serialize)]
pub struct TrackingState {
    /// PRN being tracked
    pub prn: u8,
    /// Current code phase in chips
    pub code_phase: f64,
    /// Current carrier frequency offset in Hz
    pub carrier_freq_hz: f64,
    /// Current carrier phase in radians
    pub carrier_phase_rad: f64,
    /// Prompt correlator I (in-phase)
    pub prompt_i: f64,
    /// Prompt correlator Q (quadrature)
    pub prompt_q: f64,
    /// C/N0 estimate in dB-Hz
    pub cn0_dbhz: f64,
    /// Whether carrier lock is achieved
    pub carrier_lock: bool,
    /// Whether code lock is achieved
    pub code_lock: bool,
    /// Whether bit synchronization is achieved
    pub bit_sync: bool,
    /// Number of milliseconds tracked
    pub ms_count: u64,
}

/// GPS LNAV navigation frame data
#[derive(Debug, Clone, Serialize)]
pub struct NavFrame {
    /// Subframe ID (1-5)
    pub subframe_id: u8,
    /// Time of week from HOW (Hand-Over Word) in seconds
    pub tow: u32,
    /// Raw 30-bit words (10 words per subframe)
    pub words: [u32; 10],
    /// Whether parity check passed
    pub parity_ok: bool,
}

/// GPS LNAV subframe 1 data (clock correction)
#[derive(Debug, Clone, Serialize)]
pub struct SubframeClock {
    /// GPS week number (10-bit)
    pub week_number: u16,
    /// SV accuracy (URA index)
    pub sv_accuracy: u8,
    /// SV health
    pub sv_health: u8,
    /// Clock correction parameters
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
    /// Group delay (Tgd)
    pub tgd: f64,
}

/// GNSS signal parameters for display
#[derive(Debug, Clone, Serialize)]
pub struct GnssSignalInfo {
    pub signal: GnssSignal,
    pub carrier_freq_mhz: f64,
    pub chipping_rate_mchips: f64,
    pub code_length: usize,
    pub code_period_ms: f64,
    pub data_rate_bps: f64,
    pub modulation: &'static str,
    pub multiple_access: &'static str,
    pub processing_gain_db: f64,
}

impl GnssSignalInfo {
    pub fn from_signal(signal: GnssSignal) -> Self {
        let (modulation, multiple_access) = match signal {
            GnssSignal::GpsL1Ca => ("BPSK(1)", "CDMA"),
            GnssSignal::GpsL5 => ("QPSK(10)", "CDMA"),
            GnssSignal::GlonassL1of => ("BPSK(0.5)", "FDMA"),
            GnssSignal::GalileoE1 => ("CBOC(6,1,1/11)", "CDMA"),
        };
        let code_length = signal.code_length();
        Self {
            signal,
            carrier_freq_mhz: signal.carrier_frequency_hz() / 1e6,
            chipping_rate_mchips: signal.chipping_rate() / 1e6,
            code_length,
            code_period_ms: signal.code_period_s() * 1000.0,
            data_rate_bps: signal.nav_data_rate_bps(),
            modulation,
            multiple_access,
            processing_gain_db: 10.0 * (code_length as f64).log10(),
        }
    }
}
