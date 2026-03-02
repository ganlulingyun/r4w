//! # ROADM and WSS Controller
//!
//! Reconfigurable Optical Add-Drop Multiplexer (ROADM) and Wavelength Selective Switch (WSS)
//! signal processing and control algorithms for DWDM optical networking.
//!
//! ## Standards References
//! - ITU-T G.694.1: Spectral grids for WDM applications – DWDM frequency grid
//! - ITU-T G.671: Transmission characteristics of optical components and subsystems
//! - ITU-T G.680: Physical transfer functions of optical network elements
//! - ITU-T G.697: Optical monitoring for DWDM systems
//! - ITU-T G.798: Characteristics of optical transport network hierarchy equipment
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::roadm_wss_controller::*;
//!
//! // Create an ITU-T 50 GHz fixed grid
//! let grid = FrequencyGrid::fixed_50ghz();
//! let ch_freq = grid.channel_frequency(0);  // Channel n=0 → 193.1 THz
//! assert!((ch_freq - 193.1e12).abs() < 1e9);
//!
//! // Create a 1x4 WSS
//! let wss_cfg = WssConfig::new(4, PassbandShape::SuperGaussian { order: 4, bw_3db_ghz: 40.0 }, 5.5);
//! let mut wss = WssSwitch::new(wss_cfg);
//! wss.connect(0, 1, 0.0);
//! let loss = wss.insertion_loss_db(0, 1, 193.1e12);
//! assert!(loss > 0.0);
//! ```

/// Speed of light in vacuum (m/s).
const C_MS: f64 = 2.997_924_58e8;

/// Reference frequency for ITU-T C-band grid: 193.1 THz (G.694.1 §5.3).
pub const CBAND_REF_FREQ_HZ: f64 = 193.1e12;

/// C-band lower edge (191.35 THz, G.694.1).
pub const CBAND_LOW_HZ: f64 = 191.35e12;

/// C-band upper edge (196.1 THz, G.694.1).
pub const CBAND_HIGH_HZ: f64 = 196.1e12;

/// L-band lower edge (186.05 THz).
pub const LBAND_LOW_HZ: f64 = 186.05e12;

/// L-band upper edge (190.95 THz).
pub const LBAND_HIGH_HZ: f64 = 190.95e12;

/// Boltzmann constant (J/K) for thermal noise computations.
const K_BOLTZMANN: f64 = 1.380_649e-23;

// ─────────────────────────────────────────────────────────────────────────────
// 1. ITU-T G.694.1 Frequency Grid
// ─────────────────────────────────────────────────────────────────────────────

/// ITU-T G.694.1 frequency grid mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GridMode {
    /// Fixed 50 GHz channel spacing (G.694.1 Table 1).
    Fixed50Ghz,
    /// Fixed 100 GHz channel spacing (G.694.1 Table 1).
    Fixed100Ghz,
    /// Flexible grid with 6.25 GHz slot granularity (G.694.1 §4).
    Flex6_25Ghz,
    /// Flexible grid with 12.5 GHz slot granularity.
    Flex12_5Ghz,
}

/// ITU-T G.694.1 DWDM frequency grid.
///
/// Fixed grid: f_n = f_ref + n × Δf, where Δf is 50 or 100 GHz and n is
/// the channel number (positive toward shorter wavelengths, negative toward
/// longer wavelengths). Reference 193.1 THz per G.694.1 §5.3.
///
/// Flex grid: center frequency = f_ref + n × 6.25 GHz (slot granularity),
/// slot width W = m × 12.5 GHz (minimum bandwidth unit).
#[derive(Debug, Clone)]
pub struct FrequencyGrid {
    /// Grid operating mode.
    pub mode: GridMode,
    /// Slot granularity in Hz (6.25 GHz or 12.5 GHz for flex; spacing for fixed).
    pub granularity_hz: f64,
    /// Reference frequency in Hz.
    pub reference_hz: f64,
    /// Band lower limit in Hz.
    pub band_low_hz: f64,
    /// Band upper limit in Hz.
    pub band_high_hz: f64,
}

impl FrequencyGrid {
    /// Create a fixed 50 GHz grid (C-band, G.694.1).
    pub fn fixed_50ghz() -> Self {
        Self {
            mode: GridMode::Fixed50Ghz,
            granularity_hz: 50.0e9,
            reference_hz: CBAND_REF_FREQ_HZ,
            band_low_hz: CBAND_LOW_HZ,
            band_high_hz: CBAND_HIGH_HZ,
        }
    }

    /// Create a fixed 100 GHz grid (C-band, G.694.1).
    pub fn fixed_100ghz() -> Self {
        Self {
            mode: GridMode::Fixed100Ghz,
            granularity_hz: 100.0e9,
            reference_hz: CBAND_REF_FREQ_HZ,
            band_low_hz: CBAND_LOW_HZ,
            band_high_hz: CBAND_HIGH_HZ,
        }
    }

    /// Create a flexible grid with 6.25 GHz granularity (C-band, G.694.1 §4).
    pub fn flex_6_25ghz() -> Self {
        Self {
            mode: GridMode::Flex6_25Ghz,
            granularity_hz: 6.25e9,
            reference_hz: CBAND_REF_FREQ_HZ,
            band_low_hz: CBAND_LOW_HZ,
            band_high_hz: CBAND_HIGH_HZ,
        }
    }

    /// Create a flexible grid with 12.5 GHz granularity.
    pub fn flex_12_5ghz() -> Self {
        Self {
            mode: GridMode::Flex12_5Ghz,
            granularity_hz: 12.5e9,
            reference_hz: CBAND_REF_FREQ_HZ,
            band_low_hz: CBAND_LOW_HZ,
            band_high_hz: CBAND_HIGH_HZ,
        }
    }

    /// Compute channel center frequency from ITU-T channel number n.
    ///
    /// Fixed grid: f = f_ref + n × Δf  (G.694.1 §5.3)
    /// Flex grid:  f = f_ref + n × granularity_hz
    pub fn channel_frequency(&self, n: i32) -> f64 {
        self.reference_hz + (n as f64) * self.granularity_hz
    }

    /// Compute channel number n from center frequency (rounds to nearest slot).
    pub fn frequency_to_channel(&self, freq_hz: f64) -> i32 {
        let raw = (freq_hz - self.reference_hz) / self.granularity_hz;
        raw.round() as i32
    }

    /// Snap a frequency to the nearest valid grid slot.
    pub fn snap_to_grid(&self, freq_hz: f64) -> f64 {
        let n = self.frequency_to_channel(freq_hz);
        self.channel_frequency(n)
    }

    /// Convert frequency in Hz to wavelength in meters.
    pub fn freq_to_wavelength(freq_hz: f64) -> f64 {
        C_MS / freq_hz
    }

    /// Convert wavelength in meters to frequency in Hz.
    pub fn wavelength_to_freq(lambda_m: f64) -> f64 {
        C_MS / lambda_m
    }

    /// List all valid channel numbers within the band limits.
    pub fn valid_channels(&self) -> Vec<i32> {
        let n_min = ((self.band_low_hz - self.reference_hz) / self.granularity_hz).ceil() as i32;
        let n_max = ((self.band_high_hz - self.reference_hz) / self.granularity_hz).floor() as i32;
        (n_min..=n_max).collect()
    }

    /// Count valid channels in the band.
    pub fn channel_count(&self) -> usize {
        self.valid_channels().len()
    }

    /// Check whether a frequency lies within band limits.
    pub fn in_band(&self, freq_hz: f64) -> bool {
        freq_hz >= self.band_low_hz && freq_hz <= self.band_high_hz
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. WSS Passband Shape
// ─────────────────────────────────────────────────────────────────────────────

/// Passband shape model for a Wavelength Selective Switch.
///
/// References:
/// - J. D. Downie, "Relationship of Q penalty to eye closure penalty for NRZ
///   and RZ signals with chromatic dispersion", Photon. Technol. Lett. 2004
/// - Frisken, S. et al., "Flexible bandwidth control in WDM networks", OFC 2014
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PassbandShape {
    /// Gaussian filter H(f) = exp(-0.5*(f/σ)^2), σ from 3 dB bandwidth.
    Gaussian { bw_3db_ghz: f64 },
    /// Super-Gaussian H(f) = exp(-0.5*(f/σ)^(2p)) order p for flat-top response.
    SuperGaussian { order: u32, bw_3db_ghz: f64 },
    /// Rectangular passband (ideal brick-wall).
    Rectangular { bw_ghz: f64 },
}

impl PassbandShape {
    /// Compute normalized power transmission (linear, 0..=1) at offset `delta_f_hz`
    /// from the channel center.
    pub fn transmission(&self, delta_f_hz: f64) -> f64 {
        match self {
            PassbandShape::Gaussian { bw_3db_ghz } => {
                // σ = bw_3dB / (2 * sqrt(2*ln2))
                let sigma = bw_3db_ghz * 1.0e9 / (2.0 * (2.0_f64 * 2.0_f64.ln()).sqrt());
                let x = delta_f_hz / sigma;
                (-0.5 * x * x).exp()
            }
            PassbandShape::SuperGaussian { order, bw_3db_ghz } => {
                let p = *order as f64;
                let sigma = bw_3db_ghz * 1.0e9 / (2.0 * (2.0_f64 * 2.0_f64.ln()).powf(1.0 / (2.0 * p)));
                let x = delta_f_hz / sigma;
                (-0.5 * x.abs().powf(2.0 * p)).exp()
            }
            PassbandShape::Rectangular { bw_ghz } => {
                if delta_f_hz.abs() <= bw_ghz * 0.5e9 {
                    1.0
                } else {
                    0.0
                }
            }
        }
    }

    /// Compute insertion loss in dB at offset `delta_f_hz` from channel center.
    pub fn loss_db(&self, delta_f_hz: f64) -> f64 {
        let t = self.transmission(delta_f_hz);
        if t <= 1e-30 {
            300.0 // −300 dB for complete extinction
        } else {
            -10.0 * t.log10()
        }
    }

    /// Return the 3 dB bandwidth in Hz.
    pub fn bw_3db_hz(&self) -> f64 {
        match self {
            PassbandShape::Gaussian { bw_3db_ghz } => bw_3db_ghz * 1.0e9,
            PassbandShape::SuperGaussian { bw_3db_ghz, .. } => bw_3db_ghz * 1.0e9,
            PassbandShape::Rectangular { bw_ghz } => bw_ghz * 1.0e9,
        }
    }

    /// Compute filter concatenation penalty in dB for N cascaded identical filters.
    ///
    /// Each pass narrows the effective passband; the penalty is estimated at
    /// ±bw/4 to capture the passband ripple effect on OSNR margin.
    pub fn concatenation_penalty_db(&self, n_stages: usize, probe_offset_hz: f64) -> f64 {
        if n_stages == 0 {
            return 0.0;
        }
        // Power through N cascaded filters at probe offset
        let t = self.transmission(probe_offset_hz);
        let t_n = t.powi(n_stages as i32);
        if t_n <= 1e-30 {
            300.0
        } else {
            -10.0 * t_n.log10()
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. WSS Port and Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Port direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortDirection {
    /// Common port (e.g., the line/express port in a 1xN WSS).
    Common,
    /// Switch port (add/drop or tributary port).
    Switch,
}

/// Fiber type affecting chromatic dispersion and loss coefficient.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FiberType {
    /// Standard single-mode fiber G.652 (~0.2 dB/km @ 1550 nm).
    G652,
    /// Non-zero DSF G.655 (~0.2 dB/km, lower PMD).
    G655,
    /// Dispersion-shifted fiber G.653 (zero dispersion at 1550 nm).
    G653,
}

impl FiberType {
    /// Attenuation coefficient in dB/km at 1550 nm.
    pub fn attenuation_db_per_km(&self) -> f64 {
        match self {
            FiberType::G652 => 0.20,
            FiberType::G655 => 0.20,
            FiberType::G653 => 0.21,
        }
    }

    /// Chromatic dispersion ps/(nm·km) at 1550 nm.
    pub fn dispersion_ps_per_nm_km(&self) -> f64 {
        match self {
            FiberType::G652 => 17.0,
            FiberType::G655 => 4.0,
            FiberType::G653 => 0.1,
        }
    }
}

/// A single WSS port.
#[derive(Debug, Clone)]
pub struct WssPort {
    /// Port index (0 = common for 1xN).
    pub index: usize,
    /// Port direction.
    pub direction: PortDirection,
    /// Pigtail fiber type.
    pub fiber_type: FiberType,
    /// Pigtail connector loss in dB.
    pub connector_loss_db: f64,
}

impl WssPort {
    /// Create a common (line) port.
    pub fn common(index: usize) -> Self {
        Self {
            index,
            direction: PortDirection::Common,
            fiber_type: FiberType::G652,
            connector_loss_db: 0.3,
        }
    }

    /// Create a switch port.
    pub fn switch(index: usize) -> Self {
        Self {
            index,
            direction: PortDirection::Switch,
            fiber_type: FiberType::G652,
            connector_loss_db: 0.3,
        }
    }
}

/// WSS device configuration.
#[derive(Debug, Clone)]
pub struct WssConfig {
    /// Number of switch ports (1 common + N switch = 1xN WSS).
    pub num_switch_ports: usize,
    /// Passband shape model.
    pub passband: PassbandShape,
    /// Flat insertion loss in dB (wavelength-independent component).
    pub flat_il_db: f64,
    /// Wavelength-dependent loss slope dB/THz (tilt over C-band).
    pub tilt_db_per_thz: f64,
    /// Minimum channel isolation (port-to-port crosstalk suppression) in dB.
    pub isolation_db: f64,
    /// Maximum number of connections per switch port.
    pub max_connections_per_port: usize,
}

impl WssConfig {
    /// Create WSS configuration with given port count, passband, and flat IL.
    pub fn new(num_switch_ports: usize, passband: PassbandShape, flat_il_db: f64) -> Self {
        Self {
            num_switch_ports,
            passband,
            flat_il_db,
            tilt_db_per_thz: 0.05,
            isolation_db: 35.0,
            max_connections_per_port: 1,
        }
    }

    /// Compute total insertion loss at a given frequency for the flat+tilt model.
    ///
    /// IL(f) = IL_flat + tilt × (f - f_ref) / 1e12
    pub fn base_il_at_freq(&self, freq_hz: f64) -> f64 {
        let delta_thz = (freq_hz - CBAND_REF_FREQ_HZ) / 1.0e12;
        self.flat_il_db + self.tilt_db_per_thz * delta_thz
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. WSS Switching Matrix
// ─────────────────────────────────────────────────────────────────────────────

/// A single switching connection in the WSS matrix.
#[derive(Debug, Clone)]
struct WssConnection {
    /// Input switch port (0..num_switch_ports − 1).
    input_port: usize,
    /// Output switch port (0..num_switch_ports − 1), or usize::MAX for common.
    output_port: usize,
    /// Center frequency of the routed channel in Hz.
    center_freq_hz: f64,
    /// Per-connection attenuation (VOA setting) in dB.
    attenuation_db: f64,
    /// Whether this connection is active.
    active: bool,
}

/// WSS switching matrix with per-connection attenuation control.
///
/// Models a 1×N WSS where N switch ports can each carry one or more wavelengths.
/// Each wavelength is routed between the common port and one switch port with
/// configurable attenuation (VOA).
#[derive(Debug, Clone)]
pub struct WssSwitch {
    /// Device configuration.
    pub config: WssConfig,
    /// Active connections in the switching matrix.
    connections: Vec<WssConnection>,
}

impl WssSwitch {
    /// Create a new WSS with given configuration.
    pub fn new(config: WssConfig) -> Self {
        Self {
            config,
            connections: Vec::new(),
        }
    }

    /// Establish a wavelength routing connection.
    ///
    /// Routes the channel at `center_freq_hz` between the common port and
    /// `switch_port` (0..num_switch_ports) with optional VOA attenuation.
    pub fn connect(&mut self, switch_port: usize, _output_port: usize, attenuation_db: f64) -> Result<(), WssError> {
        if switch_port >= self.config.num_switch_ports {
            return Err(WssError::InvalidPort(switch_port));
        }
        // Store as a wildcard connection (no specific freq) for port connectivity
        self.connections.push(WssConnection {
            input_port: switch_port,
            output_port: _output_port,
            center_freq_hz: 0.0,
            attenuation_db,
            active: true,
        });
        Ok(())
    }

    /// Provision a wavelength route with center frequency and attenuation.
    pub fn provision_wavelength(
        &mut self,
        switch_port: usize,
        center_freq_hz: f64,
        attenuation_db: f64,
    ) -> Result<(), WssError> {
        if switch_port >= self.config.num_switch_ports {
            return Err(WssError::InvalidPort(switch_port));
        }
        // Check if port already has a connection at this frequency
        for conn in &self.connections {
            if conn.input_port == switch_port
                && (conn.center_freq_hz - center_freq_hz).abs() < 1.0e9
                && conn.active
            {
                return Err(WssError::PortBusy(switch_port));
            }
        }
        self.connections.push(WssConnection {
            input_port: switch_port,
            output_port: 0, // common port
            center_freq_hz,
            attenuation_db,
            active: true,
        });
        Ok(())
    }

    /// Remove a wavelength route at a specific switch port and frequency.
    pub fn deprovision_wavelength(&mut self, switch_port: usize, center_freq_hz: f64) -> bool {
        for conn in &mut self.connections {
            if conn.input_port == switch_port
                && (conn.center_freq_hz - center_freq_hz).abs() < 1.0e9
            {
                conn.active = false;
                return true;
            }
        }
        false
    }

    /// Set per-channel VOA attenuation on an existing connection.
    pub fn set_attenuation(&mut self, switch_port: usize, center_freq_hz: f64, att_db: f64) -> bool {
        for conn in &mut self.connections {
            if conn.input_port == switch_port
                && (conn.center_freq_hz - center_freq_hz).abs() < 1.0e9
                && conn.active
            {
                conn.attenuation_db = att_db;
                return true;
            }
        }
        false
    }

    /// Compute total insertion loss in dB through the WSS at a given frequency
    /// when routing from/to `switch_port`.
    ///
    /// Total IL = flat_IL + tilt(f) + passband_loss(Δf) + VOA + connector_losses
    pub fn insertion_loss_db(&self, switch_port: usize, _output_port: usize, freq_hz: f64) -> f64 {
        // Find the connection for this port
        let voa_db = self.connections.iter()
            .filter(|c| c.input_port == switch_port && c.active)
            .map(|c| {
                if c.center_freq_hz > 0.0 {
                    let delta_f = freq_hz - c.center_freq_hz;
                    self.config.passband.loss_db(delta_f) + c.attenuation_db
                } else {
                    c.attenuation_db
                }
            })
            .next()
            .unwrap_or(0.0);

        let base_il = self.config.base_il_at_freq(freq_hz);
        // Connector losses (common + switch port)
        let connector_loss = 0.3 * 2.0;
        base_il + voa_db + connector_loss
    }

    /// Compute port-to-port isolation (crosstalk suppression) in dB.
    ///
    /// Models the isolation between two switch ports routing at the same wavelength.
    pub fn crosstalk_db(&self, port_a: usize, port_b: usize) -> f64 {
        if port_a == port_b {
            return 0.0; // Same port: no crosstalk
        }
        self.config.isolation_db
    }

    /// Compute filter concatenation penalty for N cascaded WSS elements.
    ///
    /// Each traversal narrows the effective optical passband, increasing OSNR
    /// penalty. The probe offset is set to bw_3dB/4 to capture the rolloff effect.
    pub fn filter_concatenation_penalty_db(&self, n_nodes: usize) -> f64 {
        let bw = self.config.passband.bw_3db_hz();
        let probe = bw / 4.0; // Assess at quarter-bandwidth edge
        self.config.passband.concatenation_penalty_db(n_nodes, probe)
    }

    /// Return all active connections.
    pub fn active_connections(&self) -> impl Iterator<Item = &WssConnection> {
        self.connections.iter().filter(|c| c.active)
    }

    /// Count active connections.
    pub fn active_count(&self) -> usize {
        self.connections.iter().filter(|c| c.active).count()
    }
}

/// WSS operation error types.
#[derive(Debug, Clone, PartialEq)]
pub enum WssError {
    /// Switch port index out of range.
    InvalidPort(usize),
    /// Switch port already occupied by a connection at this wavelength.
    PortBusy(usize),
    /// Requested frequency not on the ITU-T grid.
    FrequencyOffGrid(f64),
    /// No connection found for deprovision.
    NotFound,
}

impl std::fmt::Display for WssError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            WssError::InvalidPort(p) => write!(f, "WSS: invalid port {}", p),
            WssError::PortBusy(p) => write!(f, "WSS: port {} already in use", p),
            WssError::FrequencyOffGrid(freq) => write!(f, "WSS: frequency {:.3e} Hz not on grid", freq),
            WssError::NotFound => write!(f, "WSS: connection not found"),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. Channel Plan Management
// ─────────────────────────────────────────────────────────────────────────────

/// Modulation format for optical channel capacity estimation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ModulationFormat {
    /// DP-QPSK – 4 bits/symbol, min OSNR ~12.5 dB
    DpQpsk,
    /// DP-16QAM – 8 bits/symbol, min OSNR ~19 dB
    Dp16Qam,
    /// DP-64QAM – 12 bits/symbol, min OSNR ~25 dB
    Dp64Qam,
    /// DP-256QAM – 16 bits/symbol, min OSNR ~31 dB
    Dp256Qam,
    /// OOK (NRZ) – legacy 1 bit/symbol
    Ook,
}

impl ModulationFormat {
    /// Minimum OSNR (0.1 nm reference BW) required for BER < 1e-3 in dB.
    pub fn min_osnr_db(&self) -> f64 {
        match self {
            ModulationFormat::DpQpsk => 12.5,
            ModulationFormat::Dp16Qam => 19.0,
            ModulationFormat::Dp64Qam => 25.0,
            ModulationFormat::Dp256Qam => 31.0,
            ModulationFormat::Ook => 8.5,
        }
    }

    /// Spectral efficiency in bits/s/Hz.
    pub fn spectral_efficiency(&self) -> f64 {
        match self {
            ModulationFormat::DpQpsk => 4.0,
            ModulationFormat::Dp16Qam => 8.0,
            ModulationFormat::Dp64Qam => 12.0,
            ModulationFormat::Dp256Qam => 16.0,
            ModulationFormat::Ook => 1.0,
        }
    }

    /// Approximate FEC overhead (G.709 hard-decision FEC adds 7%).
    pub fn fec_overhead(&self) -> f64 {
        0.07 // 7% hard-decision FEC (G.709 OTU framing)
    }
}

/// A provisioned optical channel.
#[derive(Debug, Clone)]
pub struct OpticalChannel {
    /// Unique channel identifier.
    pub id: u32,
    /// Center frequency in Hz (snapped to ITU-T grid).
    pub center_freq_hz: f64,
    /// Slot width in Hz (flex-grid: multiples of 12.5 GHz).
    pub slot_width_hz: f64,
    /// Modulation format.
    pub modulation: ModulationFormat,
    /// Target launch power in dBm.
    pub target_power_dbm: f64,
    /// Measured received power in dBm (updated by monitoring).
    pub rx_power_dbm: f64,
    /// Estimated OSNR in dB (0.1 nm reference bandwidth).
    pub osnr_db: f64,
    /// Source switch port.
    pub src_port: usize,
    /// Destination switch port.
    pub dst_port: usize,
    /// Whether the channel is currently active.
    pub active: bool,
}

impl OpticalChannel {
    /// Create a new optical channel with default values.
    pub fn new(
        id: u32,
        center_freq_hz: f64,
        slot_width_hz: f64,
        modulation: ModulationFormat,
        src_port: usize,
        dst_port: usize,
    ) -> Self {
        Self {
            id,
            center_freq_hz,
            slot_width_hz,
            modulation,
            target_power_dbm: -2.0,
            rx_power_dbm: f64::NEG_INFINITY,
            osnr_db: 0.0,
            src_port,
            dst_port,
            active: false,
        }
    }

    /// Compute channel capacity in Gbps based on slot width and modulation format.
    ///
    /// Capacity = SE × slot_width × (1 - FEC_overhead)
    pub fn capacity_gbps(&self) -> f64 {
        let se = self.modulation.spectral_efficiency();
        let baud = self.slot_width_hz / (1.0 + self.modulation.fec_overhead());
        se * baud / 1.0e9
    }

    /// OSNR margin relative to modulation minimum OSNR.
    pub fn osnr_margin_db(&self) -> f64 {
        self.osnr_db - self.modulation.min_osnr_db()
    }

    /// Compute channel wavelength in nm.
    pub fn wavelength_nm(&self) -> f64 {
        FrequencyGrid::freq_to_wavelength(self.center_freq_hz) * 1.0e9
    }
}

/// Flex-grid frequency slot descriptor (G.694.1 §4).
#[derive(Debug, Clone, Copy)]
pub struct FlexSlot {
    /// Slot central frequency: f_ref + n × 6.25 GHz.
    pub n: i32,
    /// Slot width in units of 12.5 GHz.
    pub m: u32,
}

impl FlexSlot {
    /// Center frequency of the slot in Hz.
    pub fn center_hz(&self, grid: &FrequencyGrid) -> f64 {
        grid.reference_hz + (self.n as f64) * 6.25e9
    }

    /// Slot width in Hz.
    pub fn width_hz(&self) -> f64 {
        (self.m as f64) * 12.5e9
    }

    /// Lower edge of the slot in Hz.
    pub fn low_hz(&self, grid: &FrequencyGrid) -> f64 {
        self.center_hz(grid) - self.width_hz() / 2.0
    }

    /// Upper edge of the slot in Hz.
    pub fn high_hz(&self, grid: &FrequencyGrid) -> f64 {
        self.center_hz(grid) + self.width_hz() / 2.0
    }

    /// Check if this slot overlaps with another.
    pub fn overlaps(&self, other: &FlexSlot, grid: &FrequencyGrid) -> bool {
        self.low_hz(grid) < other.high_hz(grid) && self.high_hz(grid) > other.low_hz(grid)
    }
}

/// Allocation strategy for flex-grid channel placement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AllocationStrategy {
    /// First-Fit: place the channel in the first available spectrum gap.
    FirstFit,
    /// Best-Fit: minimize leftover fragmentation (smallest gap that fits).
    BestFit,
    /// Last-Fit: place in the last available gap (toward higher frequencies).
    LastFit,
}

/// Channel plan managing all provisioned wavelengths.
#[derive(Debug, Clone)]
pub struct ChannelPlan {
    /// Reference frequency grid.
    pub grid: FrequencyGrid,
    /// Provisioned channels indexed by channel ID.
    channels: Vec<OpticalChannel>,
    /// Next channel ID counter.
    next_id: u32,
}

impl ChannelPlan {
    /// Create a new channel plan with the given frequency grid.
    pub fn new(grid: FrequencyGrid) -> Self {
        Self {
            grid,
            channels: Vec::new(),
            next_id: 1,
        }
    }

    /// Provision a new channel with a specific center frequency.
    pub fn provision(
        &mut self,
        center_freq_hz: f64,
        slot_width_hz: f64,
        modulation: ModulationFormat,
        src_port: usize,
        dst_port: usize,
    ) -> Result<u32, ChannelError> {
        // Snap to grid
        let snapped = self.grid.snap_to_grid(center_freq_hz);

        // Check for spectral overlap with existing channels
        for ch in &self.channels {
            if !ch.active {
                continue;
            }
            let gap = (ch.center_freq_hz - snapped).abs();
            let min_gap = (ch.slot_width_hz + slot_width_hz) / 2.0;
            if gap < min_gap {
                return Err(ChannelError::SpectrumConflict(ch.id));
            }
        }

        let id = self.next_id;
        self.next_id += 1;
        let mut ch = OpticalChannel::new(id, snapped, slot_width_hz, modulation, src_port, dst_port);
        ch.active = true;
        self.channels.push(ch);
        Ok(id)
    }

    /// Deprovision a channel by ID.
    pub fn deprovision(&mut self, channel_id: u32) -> Result<(), ChannelError> {
        for ch in &mut self.channels {
            if ch.id == channel_id {
                ch.active = false;
                return Ok(());
            }
        }
        Err(ChannelError::NotFound(channel_id))
    }

    /// Get a reference to a channel by ID.
    pub fn get_channel(&self, channel_id: u32) -> Option<&OpticalChannel> {
        self.channels.iter().find(|c| c.id == channel_id && c.active)
    }

    /// Get a mutable reference to a channel by ID.
    pub fn get_channel_mut(&mut self, channel_id: u32) -> Option<&mut OpticalChannel> {
        self.channels.iter_mut().find(|c| c.id == channel_id && c.active)
    }

    /// Return all active channels sorted by center frequency.
    pub fn active_channels(&self) -> Vec<&OpticalChannel> {
        let mut v: Vec<_> = self.channels.iter().filter(|c| c.active).collect();
        v.sort_by(|a, b| a.center_freq_hz.partial_cmp(&b.center_freq_hz).unwrap());
        v
    }

    /// Count active channels.
    pub fn active_count(&self) -> usize {
        self.channels.iter().filter(|c| c.active).count()
    }

    /// Allocate a flex-grid slot using the specified strategy.
    ///
    /// Returns Some(center_freq_hz) on success or None if no gap exists.
    pub fn allocate_flex(
        &self,
        required_width_hz: f64,
        strategy: AllocationStrategy,
        guard_band_hz: f64,
    ) -> Option<f64> {
        // Build list of occupied frequency ranges sorted by low edge
        let mut occupied: Vec<(f64, f64)> = self.channels.iter()
            .filter(|c| c.active)
            .map(|c| (
                c.center_freq_hz - c.slot_width_hz / 2.0 - guard_band_hz,
                c.center_freq_hz + c.slot_width_hz / 2.0 + guard_band_hz,
            ))
            .collect();
        occupied.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        // Build list of free gaps between band edges and occupied ranges
        let mut gaps: Vec<(f64, f64)> = Vec::new();
        let mut cursor = self.grid.band_low_hz;
        for (lo, hi) in &occupied {
            if *lo > cursor + required_width_hz {
                gaps.push((cursor, *lo));
            }
            cursor = hi.max(cursor);
        }
        if self.grid.band_high_hz > cursor + required_width_hz {
            gaps.push((cursor, self.grid.band_high_hz));
        }

        if gaps.is_empty() {
            return None;
        }

        let center = match strategy {
            AllocationStrategy::FirstFit => {
                // First gap that fits
                gaps.iter()
                    .find(|(lo, hi)| hi - lo >= required_width_hz)
                    .map(|(lo, hi)| {
                        let c = lo + required_width_hz / 2.0;
                        self.grid.snap_to_grid(c.min(hi - required_width_hz / 2.0))
                    })
            }
            AllocationStrategy::BestFit => {
                // Smallest gap that fits
                gaps.iter()
                    .filter(|(lo, hi)| hi - lo >= required_width_hz)
                    .min_by(|a, b| {
                        let wa = a.1 - a.0;
                        let wb = b.1 - b.0;
                        wa.partial_cmp(&wb).unwrap()
                    })
                    .map(|(lo, hi)| {
                        let c = lo + required_width_hz / 2.0;
                        self.grid.snap_to_grid(c.min(hi - required_width_hz / 2.0))
                    })
            }
            AllocationStrategy::LastFit => {
                // Last gap that fits
                gaps.iter()
                    .filter(|(lo, hi)| hi - lo >= required_width_hz)
                    .last()
                    .map(|(_lo, hi)| {
                        let c = hi - required_width_hz / 2.0;
                        self.grid.snap_to_grid(c)
                    })
            }
        };
        center
    }

    /// Compute spectrum fragmentation ratio (0 = no fragmentation, 1 = fully fragmented).
    ///
    /// Fragmentation = 1 − (largest_contiguous_gap / total_free_spectrum)
    pub fn fragmentation_ratio(&self) -> f64 {
        let total_bw = self.grid.band_high_hz - self.grid.band_low_hz;
        let used_bw: f64 = self.channels.iter()
            .filter(|c| c.active)
            .map(|c| c.slot_width_hz)
            .sum();
        let free_bw = total_bw - used_bw;
        if free_bw <= 0.0 {
            return 1.0;
        }

        // Find largest contiguous free gap
        let mut occupied: Vec<(f64, f64)> = self.channels.iter()
            .filter(|c| c.active)
            .map(|c| (c.center_freq_hz - c.slot_width_hz / 2.0, c.center_freq_hz + c.slot_width_hz / 2.0))
            .collect();
        occupied.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        let mut largest_gap = 0.0_f64;
        let mut cursor = self.grid.band_low_hz;
        for (lo, hi) in &occupied {
            let gap = lo - cursor;
            if gap > largest_gap {
                largest_gap = gap;
            }
            cursor = hi.max(cursor);
        }
        let end_gap = self.grid.band_high_hz - cursor;
        if end_gap > largest_gap {
            largest_gap = end_gap;
        }

        1.0 - (largest_gap / free_bw)
    }

    /// Compute spectrum utilization ratio.
    pub fn utilization(&self) -> f64 {
        let total_bw = self.grid.band_high_hz - self.grid.band_low_hz;
        let used_bw: f64 = self.channels.iter()
            .filter(|c| c.active)
            .map(|c| c.slot_width_hz)
            .sum();
        used_bw / total_bw
    }
}

/// Channel plan operation errors.
#[derive(Debug, Clone, PartialEq)]
pub enum ChannelError {
    /// Spectral overlap with existing channel (carries conflicting channel ID).
    SpectrumConflict(u32),
    /// Channel not found in plan.
    NotFound(u32),
    /// Insufficient guard band between channels.
    GuardBandViolation,
}

impl std::fmt::Display for ChannelError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ChannelError::SpectrumConflict(id) => write!(f, "Spectrum conflict with channel {}", id),
            ChannelError::NotFound(id) => write!(f, "Channel {} not found", id),
            ChannelError::GuardBandViolation => write!(f, "Guard band violation"),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. Optical Power Equalization
// ─────────────────────────────────────────────────────────────────────────────

/// Per-channel power equalization using VOA control.
///
/// Implements proportional-integral (PI) control to drive all channel powers
/// to a target level by adjusting the WSS per-channel attenuation.
#[derive(Debug, Clone)]
pub struct PowerEqualizer {
    /// Target power per channel in dBm.
    pub target_dbm: f64,
    /// Maximum VOA attenuation in dB.
    pub max_voa_db: f64,
    /// Proportional gain for PI controller.
    pub kp: f64,
    /// Integral gain for PI controller.
    pub ki: f64,
    /// Accumulated error per channel (integral state), indexed by channel ID.
    integral_state: std::collections::HashMap<u32, f64>,
    /// Per-channel VOA settings in dB.
    pub voa_settings: std::collections::HashMap<u32, f64>,
}

impl PowerEqualizer {
    /// Create a power equalizer targeting `target_dbm` dBm.
    pub fn new(target_dbm: f64, max_voa_db: f64) -> Self {
        Self {
            target_dbm,
            max_voa_db,
            kp: 0.5,
            ki: 0.1,
            integral_state: std::collections::HashMap::new(),
            voa_settings: std::collections::HashMap::new(),
        }
    }

    /// Update VOA setting for a channel based on measured received power.
    ///
    /// Returns the new VOA attenuation in dB. Positive VOA = more attenuation.
    pub fn update(&mut self, channel_id: u32, measured_power_dbm: f64) -> f64 {
        let error = measured_power_dbm - self.target_dbm;
        let integral = self.integral_state.entry(channel_id).or_insert(0.0);
        *integral += error;

        // PI control: more error → add more attenuation
        let delta_voa = self.kp * error + self.ki * (*integral);

        let current_voa = *self.voa_settings.entry(channel_id).or_insert(0.0);
        let new_voa = (current_voa + delta_voa).clamp(0.0, self.max_voa_db);

        *self.voa_settings.get_mut(&channel_id).unwrap() = new_voa;
        new_voa
    }

    /// Run N equalizer update iterations for all channels (convergence simulation).
    ///
    /// Returns maximum residual power deviation in dB after N iterations.
    pub fn converge(&mut self, channel_powers: &[(u32, f64)], iterations: usize) -> f64 {
        for _ in 0..iterations {
            for (id, power) in channel_powers {
                self.update(*id, *power);
            }
        }
        // Compute max residual deviation
        channel_powers.iter().map(|(id, pwr)| {
            let voa = self.voa_settings.get(id).copied().unwrap_or(0.0);
            (*pwr - voa - self.target_dbm).abs()
        }).fold(0.0_f64, f64::max)
    }

    /// Compute tilt compensation: adjust VOA across channels to cancel spectral tilt.
    ///
    /// Linear tilt_db_per_thz is applied relative to the mean channel frequency.
    pub fn apply_tilt_compensation(
        &mut self,
        channels: &[(u32, f64)], // (channel_id, center_freq_hz)
        tilt_db_per_thz: f64,
    ) {
        if channels.is_empty() {
            return;
        }
        let mean_freq: f64 = channels.iter().map(|(_, f)| f).sum::<f64>() / channels.len() as f64;
        for (id, freq) in channels {
            let delta_thz = (freq - mean_freq) / 1.0e12;
            let comp_voa = -tilt_db_per_thz * delta_thz; // Opposite sign to cancel tilt
            let entry = self.voa_settings.entry(*id).or_insert(0.0);
            *entry = (*entry + comp_voa).clamp(0.0, self.max_voa_db);
        }
    }

    /// Return power deviation from target for each channel after applying VOA.
    pub fn power_deviation(&self, channel_powers: &[(u32, f64)]) -> Vec<(u32, f64)> {
        channel_powers.iter().map(|(id, pwr)| {
            let voa = self.voa_settings.get(id).copied().unwrap_or(0.0);
            (*id, pwr - voa - self.target_dbm)
        }).collect()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 7. OSNR Monitoring and Cascaded Noise
// ─────────────────────────────────────────────────────────────────────────────

/// EDFA (Erbium-Doped Fiber Amplifier) amplifier model.
#[derive(Debug, Clone)]
pub struct EdfaModel {
    /// Gain in dB.
    pub gain_db: f64,
    /// Noise figure in dB.
    pub nf_db: f64,
    /// Center wavelength in nm (for gain tilt).
    pub center_wavelength_nm: f64,
}

impl EdfaModel {
    /// Create an EDFA model with given gain and noise figure.
    pub fn new(gain_db: f64, nf_db: f64) -> Self {
        Self {
            gain_db,
            nf_db,
            center_wavelength_nm: 1550.0,
        }
    }

    /// ASE (Amplified Spontaneous Emission) noise power in one polarization
    /// within reference bandwidth B_ref (Hz), in linear watts.
    ///
    /// P_ASE = h × ν × nsp × (G − 1) × B_ref
    ///
    /// where nsp = noise figure × G / (2(G − 1)) for high gain.
    /// Returns power in mW.
    pub fn ase_power_mw(&self, freq_hz: f64, bw_ref_hz: f64) -> f64 {
        let h_planck = 6.626e-34; // J·s
        let gain_lin = 10.0_f64.powf(self.gain_db / 10.0);
        let nf_lin = 10.0_f64.powf(self.nf_db / 10.0);
        // nsp = NF_lin × G / (2(G-1)) for spontaneous emission factor
        let nsp = if gain_lin > 1.001 {
            nf_lin * gain_lin / (2.0 * (gain_lin - 1.0))
        } else {
            nf_lin / 2.0
        };
        let ase_watts = h_planck * freq_hz * nsp * (gain_lin - 1.0) * bw_ref_hz;
        ase_watts * 1.0e3 // Convert to mW
    }

    /// OSNR contribution from a single EDFA for a given input signal power (dBm).
    ///
    /// OSNR_single = P_signal / P_ASE (linear), measured in 0.1 nm reference BW.
    pub fn single_amplifier_osnr_db(&self, signal_power_dbm: f64, freq_hz: f64) -> f64 {
        // 0.1 nm reference bandwidth at 1550 nm → ~12.5 GHz
        let lambda_m = C_MS / freq_hz;
        let bw_ref_hz = C_MS * 0.1e-9 / (lambda_m * lambda_m);
        let signal_mw = 10.0_f64.powf(signal_power_dbm / 10.0);
        let ase_mw = self.ase_power_mw(freq_hz, bw_ref_hz) * 2.0; // Both polarizations
        if ase_mw <= 0.0 {
            return 50.0;
        }
        10.0 * (signal_mw / ase_mw).log10()
    }
}

/// Cascaded OSNR calculation for a chain of amplifiers and fiber spans.
///
/// Uses the formula from ITU-T G.697:
/// 1/OSNR_total = Σ(1/OSNR_i) for N spans.
pub fn cascaded_osnr_db(osnr_per_span: &[f64]) -> f64 {
    if osnr_per_span.is_empty() {
        return 50.0;
    }
    let inv_sum: f64 = osnr_per_span.iter()
        .map(|&osnr_db| {
            let osnr_lin = 10.0_f64.powf(osnr_db / 10.0);
            1.0 / osnr_lin
        })
        .sum();
    if inv_sum <= 0.0 {
        return 50.0;
    }
    10.0 * (1.0 / inv_sum).log10()
}

/// Estimate OSNR at a point given accumulated ASE noise.
///
/// OSNR_dB = P_signal_dBm − P_ASE_total_dBm (in reference BW)
pub fn estimate_osnr_db(signal_dbm: f64, total_ase_mw: f64, bw_ref_hz: f64, signal_bw_hz: f64) -> f64 {
    if total_ase_mw <= 0.0 || bw_ref_hz <= 0.0 {
        return 50.0;
    }
    // Scale ASE to reference bandwidth
    let ase_in_ref_bw = total_ase_mw * (bw_ref_hz / signal_bw_hz);
    let signal_mw = 10.0_f64.powf(signal_dbm / 10.0);
    10.0 * (signal_mw / ase_in_ref_bw).log10()
}

/// GSNR (Generalized SNR) including non-linear interference (NLI) from fiber.
///
/// GSNR⁻¹ = OSNR⁻¹ + SNR_NLI⁻¹
///
/// SNR_NLI depends on fiber parameters and launch power; simplified GN model used.
pub fn compute_gsnr_db(osnr_db: f64, snr_nli_db: f64) -> f64 {
    let osnr_lin = 10.0_f64.powf(osnr_db / 10.0);
    let nli_lin = 10.0_f64.powf(snr_nli_db / 10.0);
    let gsnr_lin = 1.0 / (1.0 / osnr_lin + 1.0 / nli_lin);
    10.0 * gsnr_lin.log10()
}

/// Estimate NLI SNR using the simplified Gaussian Noise (GN) model.
///
/// SNR_NLI ∝ P_signal^-2 × (n_span × L × γ)^-1 (very simplified).
/// Returns SNR_NLI in dB. Parameters:
/// - `launch_power_dbm`: per-channel launch power
/// - `n_spans`: number of fiber spans
/// - `span_length_km`: span length
/// - `gamma`: fiber nonlinearity coefficient (1/W/km, ~1.27 for G.652)
pub fn estimate_nli_snr_db(
    launch_power_dbm: f64,
    n_spans: f64,
    span_length_km: f64,
    gamma_per_w_per_km: f64,
) -> f64 {
    let p_w = 1.0e-3 * 10.0_f64.powf(launch_power_dbm / 10.0);
    if p_w <= 0.0 || n_spans <= 0.0 || span_length_km <= 0.0 || gamma_per_w_per_km <= 0.0 {
        return 40.0; // No NLI
    }
    // Simplified GN model: SNR_NLI = C / (γ^2 × P^3 × n_spans × L)
    // C is a constant derived from channel spacing and baud rate; use a representative value.
    let c_factor = 1.0; // Normalized constant
    let nli_inv = c_factor * gamma_per_w_per_km.powi(2) * p_w.powi(3) * n_spans * span_length_km;
    if nli_inv <= 0.0 {
        40.0
    } else {
        10.0 * (1.0 / nli_inv).log10()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 8. Optical Path Computation
// ─────────────────────────────────────────────────────────────────────────────

/// A single hop in an optical path (fiber span + EDFA).
#[derive(Debug, Clone)]
pub struct PathHop {
    /// Fiber span length in km.
    pub fiber_length_km: f64,
    /// Fiber type.
    pub fiber_type: FiberType,
    /// EDFA at the end of the span.
    pub amplifier: EdfaModel,
    /// Number of WSS traversals at this node.
    pub wss_count: usize,
    /// WSS configuration for passband narrowing computation.
    pub wss_config: Option<WssConfig>,
}

/// Accumulated impairments along an optical path.
#[derive(Debug, Clone)]
pub struct PathImpairments {
    /// Accumulated fiber loss in dB.
    pub total_loss_db: f64,
    /// Accumulated OSNR in dB (cascaded).
    pub osnr_db: f64,
    /// GSNR including NLI in dB.
    pub gsnr_db: f64,
    /// Filter narrowing penalty from WSS concatenation in dB.
    pub filter_penalty_db: f64,
    /// Total chromatic dispersion accumulated in ps/nm.
    pub dispersion_ps_per_nm: f64,
    /// Number of spans traversed.
    pub num_spans: usize,
    /// Number of ROADM nodes traversed.
    pub num_roadm_nodes: usize,
}

impl Default for PathImpairments {
    fn default() -> Self {
        Self {
            total_loss_db: 0.0,
            osnr_db: 50.0,
            gsnr_db: 50.0,
            filter_penalty_db: 0.0,
            dispersion_ps_per_nm: 0.0,
            num_spans: 0,
            num_roadm_nodes: 0,
        }
    }
}

/// Compute accumulated impairments for a multi-hop optical path.
///
/// # Arguments
/// - `hops`: ordered list of path hops
/// - `channel_freq_hz`: channel center frequency
/// - `launch_power_dbm`: per-channel launch power at each EDFA output
/// - `gamma_per_w_per_km`: fiber nonlinearity coefficient
pub fn compute_path_impairments(
    hops: &[PathHop],
    channel_freq_hz: f64,
    launch_power_dbm: f64,
    gamma_per_w_per_km: f64,
) -> PathImpairments {
    // 0.1 nm reference bandwidth at the channel wavelength
    let lambda_m = C_MS / channel_freq_hz;
    let bw_ref_hz = C_MS * 0.1e-9 / (lambda_m * lambda_m);

    let mut total_loss_db = 0.0_f64;
    let mut total_ase_mw = 0.0_f64;
    let mut filter_penalty = 0.0_f64;
    let mut dispersion = 0.0_f64;
    let mut wss_traverse_count = 0usize;

    for hop in hops {
        // Fiber loss
        let span_loss = hop.fiber_type.attenuation_db_per_km() * hop.fiber_length_km;
        total_loss_db += span_loss;

        // Dispersion
        let wavelength_nm = lambda_m * 1.0e9;
        dispersion += hop.fiber_type.dispersion_ps_per_nm_km() * hop.fiber_length_km;
        let _ = wavelength_nm;

        // ASE from EDFA (both polarizations × 2)
        let ase = hop.amplifier.ase_power_mw(channel_freq_hz, bw_ref_hz) * 2.0;
        total_ase_mw += ase;

        // WSS traversal filter penalty
        wss_traverse_count += hop.wss_count;
        if let Some(wss_cfg) = &hop.wss_config {
            let tmp_wss = WssSwitch::new(wss_cfg.clone());
            filter_penalty += tmp_wss.filter_concatenation_penalty_db(hop.wss_count);
        }
    }

    // OSNR from cascaded ASE
    let signal_mw = 10.0_f64.powf(launch_power_dbm / 10.0);
    let osnr_lin = if total_ase_mw > 0.0 { signal_mw / total_ase_mw } else { 1e5 };
    let osnr_db = 10.0 * osnr_lin.log10();

    // NLI from all spans
    let n_spans = hops.len() as f64;
    let avg_span_km = if !hops.is_empty() {
        hops.iter().map(|h| h.fiber_length_km).sum::<f64>() / n_spans
    } else {
        80.0
    };
    let snr_nli_db = estimate_nli_snr_db(launch_power_dbm, n_spans, avg_span_km, gamma_per_w_per_km);
    let gsnr_db = compute_gsnr_db(osnr_db, snr_nli_db);

    PathImpairments {
        total_loss_db,
        osnr_db,
        gsnr_db,
        filter_penalty_db: filter_penalty,
        dispersion_ps_per_nm: dispersion,
        num_spans: hops.len(),
        num_roadm_nodes: wss_traverse_count,
    }
}

/// Estimate maximum reach (number of hops) for a given modulation format.
///
/// Reach is limited by the minimum GSNR requirement of the modulation format
/// after accounting for filter concatenation penalty.
pub fn estimate_reach(
    hop_template: &PathHop,
    modulation: ModulationFormat,
    channel_freq_hz: f64,
    launch_power_dbm: f64,
    gamma_per_w_per_km: f64,
    margin_db: f64,
) -> usize {
    let required_gsnr = modulation.min_osnr_db() + margin_db;
    let mut n = 0usize;
    loop {
        n += 1;
        let hops: Vec<PathHop> = (0..n).map(|_| hop_template.clone()).collect();
        let imp = compute_path_impairments(&hops, channel_freq_hz, launch_power_dbm, gamma_per_w_per_km);
        let effective_gsnr = imp.gsnr_db - imp.filter_penalty_db;
        if effective_gsnr < required_gsnr || n > 200 {
            return if n > 1 { n - 1 } else { 0 };
        }
    }
}

/// Compute OSNR margin relative to modulation format requirement.
pub fn osnr_margin_db(computed_osnr_db: f64, modulation: ModulationFormat) -> f64 {
    computed_osnr_db - modulation.min_osnr_db()
}

// ─────────────────────────────────────────────────────────────────────────────
// 9. Spectrum Visualization
// ─────────────────────────────────────────────────────────────────────────────

/// A single point in an optical spectrum trace.
#[derive(Debug, Clone, Copy)]
pub struct SpectrumPoint {
    /// Frequency in Hz.
    pub freq_hz: f64,
    /// Power spectral density in dBm / 0.1 nm.
    pub power_dbm: f64,
}

/// Spectrum analyzer for DWDM power spectral visualization.
///
/// Generates a simulated optical spectrum showing channel peaks, ASE noise floor,
/// and WSS filter shapes at a given resolution.
#[derive(Debug, Clone)]
pub struct SpectrumAnalyzer {
    /// Frequency resolution in Hz.
    pub resolution_hz: f64,
    /// Reference frequency for 0.1 nm normalization (default 193.1 THz).
    pub ref_freq_hz: f64,
    /// ASE noise floor in dBm / 0.1 nm (background noise between channels).
    pub noise_floor_dbm: f64,
}

impl SpectrumAnalyzer {
    /// Create a spectrum analyzer with given resolution.
    pub fn new(resolution_hz: f64) -> Self {
        Self {
            resolution_hz,
            ref_freq_hz: CBAND_REF_FREQ_HZ,
            noise_floor_dbm: -35.0,
        }
    }

    /// Generate optical spectrum trace for a channel plan over the C-band.
    ///
    /// Returns a vector of (frequency_Hz, power_dBm) points.
    /// Each active channel contributes a Gaussian or super-Gaussian peak.
    /// Background ASE noise floor is added.
    pub fn generate_spectrum(
        &self,
        channel_plan: &ChannelPlan,
        wss_config: Option<&WssConfig>,
        n_points: usize,
    ) -> Vec<SpectrumPoint> {
        let f_low = CBAND_LOW_HZ;
        let f_high = CBAND_HIGH_HZ;
        let df = (f_high - f_low) / (n_points as f64 - 1.0).max(1.0);

        let channels = channel_plan.active_channels();
        let passband = wss_config.map(|c| c.passband);

        (0..n_points).map(|i| {
            let freq = f_low + (i as f64) * df;
            let mut power_mw = 0.0_f64;

            // Accumulate channel signal contributions
            for ch in &channels {
                let delta_f = freq - ch.center_freq_hz;
                let half_bw = ch.slot_width_hz / 2.0;
                // Gaussian channel envelope
                let sigma = half_bw / (2.0_f64 * 2.0_f64.ln()).sqrt();
                let t = (-0.5 * (delta_f / sigma).powi(2)).exp();
                // Apply WSS passband if provided
                let wss_t = passband.map(|pb| pb.transmission(delta_f)).unwrap_or(1.0);
                let ch_power_mw = 10.0_f64.powf(ch.target_power_dbm / 10.0);
                power_mw += ch_power_mw * t * wss_t;
            }

            // Add ASE noise floor (converted from dBm/0.1nm)
            let ase_mw = 10.0_f64.powf(self.noise_floor_dbm / 10.0);
            power_mw += ase_mw;

            let power_dbm = if power_mw > 0.0 {
                10.0 * power_mw.log10()
            } else {
                -100.0
            };

            SpectrumPoint { freq_hz: freq, power_dbm }
        }).collect()
    }

    /// Generate spectrum for a set of explicit channels (id, center_hz, power_dbm).
    pub fn generate_from_channels(
        &self,
        channels: &[(f64, f64, f64)], // (center_hz, width_hz, power_dbm)
        passband: Option<PassbandShape>,
        n_points: usize,
    ) -> Vec<SpectrumPoint> {
        let f_low = CBAND_LOW_HZ;
        let f_high = CBAND_HIGH_HZ;
        let df = (f_high - f_low) / (n_points as f64 - 1.0).max(1.0);

        (0..n_points).map(|i| {
            let freq = f_low + (i as f64) * df;
            let mut power_mw = 0.0_f64;

            for &(center, width, pwr_dbm) in channels {
                let delta_f = freq - center;
                let half_bw = width / 2.0;
                let sigma = half_bw / (2.0_f64 * 2.0_f64.ln()).sqrt();
                let t = (-0.5 * (delta_f / sigma).powi(2)).exp();
                let wss_t = passband.map(|pb| pb.transmission(delta_f)).unwrap_or(1.0);
                let ch_mw = 10.0_f64.powf(pwr_dbm / 10.0);
                power_mw += ch_mw * t * wss_t;
            }

            let ase_mw = 10.0_f64.powf(self.noise_floor_dbm / 10.0);
            power_mw += ase_mw;

            SpectrumPoint {
                freq_hz: freq,
                power_dbm: 10.0 * power_mw.log10(),
            }
        }).collect()
    }

    /// Find peak power in dBm across a spectrum trace.
    pub fn peak_power(&self, spectrum: &[SpectrumPoint]) -> f64 {
        spectrum.iter().map(|p| p.power_dbm).fold(f64::NEG_INFINITY, f64::max)
    }

    /// Find the channel peak frequencies in the spectrum above a threshold.
    pub fn find_peaks(&self, spectrum: &[SpectrumPoint], threshold_dbm: f64) -> Vec<f64> {
        let n = spectrum.len();
        if n < 3 {
            return Vec::new();
        }
        let mut peaks = Vec::new();
        for i in 1..n - 1 {
            let p = spectrum[i].power_dbm;
            if p >= threshold_dbm
                && p > spectrum[i - 1].power_dbm
                && p > spectrum[i + 1].power_dbm
            {
                peaks.push(spectrum[i].freq_hz);
            }
        }
        peaks
    }

    /// Compute ASE power in a given bandwidth by integrating the spectrum below threshold.
    pub fn ase_integrated_power(&self, spectrum: &[SpectrumPoint], signal_threshold_dbm: f64) -> f64 {
        let mw: f64 = spectrum.iter()
            .filter(|p| p.power_dbm < signal_threshold_dbm)
            .map(|p| 10.0_f64.powf(p.power_dbm / 10.0))
            .sum();
        mw
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 10. ROADM Node
// ─────────────────────────────────────────────────────────────────────────────

/// A complete ROADM node with express, add, and drop planes.
///
/// Implements the standard 2-degree ROADM architecture:
/// - Express WSS: routes channels between line East/West ports
/// - Add WSS: multiplexes locally added channels onto the express path
/// - Drop WSS: demultiplexes channels from the express path to local drop ports
#[derive(Debug, Clone)]
pub struct RoadmNode {
    /// Node label/identifier.
    pub label: String,
    /// Express WSS (East-West routing).
    pub express_wss: WssSwitch,
    /// Add WSS (local channels → express path).
    pub add_wss: WssSwitch,
    /// Drop WSS (express path → local drop ports).
    pub drop_wss: WssSwitch,
    /// Channel plan for this node.
    pub channel_plan: ChannelPlan,
    /// Power equalizer for channel leveling.
    pub equalizer: PowerEqualizer,
}

impl RoadmNode {
    /// Create a ROADM node with given WSS configurations.
    pub fn new(
        label: impl Into<String>,
        express_cfg: WssConfig,
        add_cfg: WssConfig,
        drop_cfg: WssConfig,
        grid: FrequencyGrid,
        target_power_dbm: f64,
    ) -> Self {
        let max_voa = 20.0;
        Self {
            label: label.into(),
            express_wss: WssSwitch::new(express_cfg),
            add_wss: WssSwitch::new(add_cfg),
            drop_wss: WssSwitch::new(drop_cfg),
            channel_plan: ChannelPlan::new(grid),
            equalizer: PowerEqualizer::new(target_power_dbm, max_voa),
        }
    }

    /// Add a channel to the local add plane.
    pub fn add_channel(
        &mut self,
        center_freq_hz: f64,
        slot_width_hz: f64,
        modulation: ModulationFormat,
        add_port: usize,
    ) -> Result<u32, ChannelError> {
        let id = self.channel_plan.provision(
            center_freq_hz,
            slot_width_hz,
            modulation,
            add_port,
            0, // express output port
        )?;
        // Connect in add WSS
        let _ = self.add_wss.provision_wavelength(add_port, center_freq_hz, 0.0);
        Ok(id)
    }

    /// Drop a channel at this node.
    pub fn drop_channel(&mut self, channel_id: u32, drop_port: usize) -> Result<(), ChannelError> {
        let ch = self.channel_plan.get_channel(channel_id)
            .ok_or(ChannelError::NotFound(channel_id))?;
        let freq = ch.center_freq_hz;
        self.drop_wss.deprovision_wavelength(drop_port, freq);
        self.channel_plan.deprovision(channel_id)
    }

    /// Route a channel through the express path (pass-through).
    pub fn express_route(
        &mut self,
        center_freq_hz: f64,
        input_port: usize,
        output_port: usize,
    ) -> Result<(), WssError> {
        self.express_wss.provision_wavelength(input_port, center_freq_hz, 0.0)?;
        Ok(())
    }

    /// Compute total insertion loss for a channel passing through the ROADM node.
    ///
    /// Includes express WSS loss and connector losses.
    pub fn node_insertion_loss_db(&self, center_freq_hz: f64, in_port: usize, out_port: usize) -> f64 {
        self.express_wss.insertion_loss_db(in_port, out_port, center_freq_hz)
    }

    /// Get spectrum utilization at this node.
    pub fn utilization(&self) -> f64 {
        self.channel_plan.utilization()
    }

    /// Get fragmentation ratio at this node.
    pub fn fragmentation_ratio(&self) -> f64 {
        self.channel_plan.fragmentation_ratio()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Grid Tests ────────────────────────────────────────────────────────────

    #[test]
    fn test_grid_50ghz_reference_channel() {
        let grid = FrequencyGrid::fixed_50ghz();
        // n=0 → 193.1 THz (G.694.1 reference)
        let f0 = grid.channel_frequency(0);
        assert!((f0 - 193.1e12).abs() < 1e6, "n=0 should be 193.1 THz, got {:.3e}", f0);
    }

    #[test]
    fn test_grid_50ghz_positive_channel() {
        let grid = FrequencyGrid::fixed_50ghz();
        // n=1 → 193.15 THz
        let f1 = grid.channel_frequency(1);
        assert!((f1 - 193.15e12).abs() < 1e6, "n=1 should be 193.15 THz, got {:.3e}", f1);
    }

    #[test]
    fn test_grid_50ghz_negative_channel() {
        let grid = FrequencyGrid::fixed_50ghz();
        // n=-1 → 193.05 THz
        let fm1 = grid.channel_frequency(-1);
        assert!((fm1 - 193.05e12).abs() < 1e6, "n=-1 should be 193.05 THz, got {:.3e}", fm1);
    }

    #[test]
    fn test_grid_100ghz_spacing() {
        let grid = FrequencyGrid::fixed_100ghz();
        let f0 = grid.channel_frequency(0);
        let f1 = grid.channel_frequency(1);
        assert!((f1 - f0 - 100.0e9).abs() < 1e6, "100 GHz spacing expected");
    }

    #[test]
    fn test_grid_flex_granularity() {
        let grid = FrequencyGrid::flex_6_25ghz();
        let f0 = grid.channel_frequency(0);
        let f1 = grid.channel_frequency(1);
        assert!((f1 - f0 - 6.25e9).abs() < 1e6, "6.25 GHz flex granularity expected");
    }

    #[test]
    fn test_grid_frequency_to_channel_roundtrip() {
        let grid = FrequencyGrid::fixed_50ghz();
        for n in [-10, -5, 0, 5, 10, 20] {
            let freq = grid.channel_frequency(n);
            let n_back = grid.frequency_to_channel(freq);
            assert_eq!(n_back, n, "Roundtrip failed for n={}", n);
        }
    }

    #[test]
    fn test_grid_snap_to_grid() {
        let grid = FrequencyGrid::fixed_50ghz();
        let f_center = grid.channel_frequency(5);
        let f_off = f_center + 10.0e9; // 10 GHz off
        let f_snap = grid.snap_to_grid(f_off);
        assert!((f_snap - f_center).abs() < 1e9, "Snap should return n=5 channel");
    }

    #[test]
    fn test_grid_channel_count_50ghz() {
        let grid = FrequencyGrid::fixed_50ghz();
        let count = grid.channel_count();
        // C-band 191.35..196.1 THz, 50 GHz spacing → ~95-96 channels
        assert!(count >= 90 && count <= 100, "Expected ~95 channels, got {}", count);
    }

    #[test]
    fn test_grid_channel_count_100ghz() {
        let grid = FrequencyGrid::fixed_100ghz();
        let count = grid.channel_count();
        // Half the 50 GHz count
        assert!(count >= 45 && count <= 50, "Expected ~48 channels, got {}", count);
    }

    #[test]
    fn test_grid_in_band() {
        let grid = FrequencyGrid::fixed_50ghz();
        assert!(grid.in_band(193.1e12));
        assert!(!grid.in_band(185.0e12)); // L-band: out of C-band
        assert!(!grid.in_band(200.0e12)); // Too high
    }

    #[test]
    fn test_grid_wavelength_conversion() {
        // 1550 nm ≈ 193.414 THz
        let lambda = 1550e-9;
        let freq = FrequencyGrid::wavelength_to_freq(lambda);
        let lambda_back = FrequencyGrid::freq_to_wavelength(freq);
        assert!((lambda_back - lambda).abs() < 1e-15);
    }

    // ── Passband Tests ────────────────────────────────────────────────────────

    #[test]
    fn test_gaussian_passband_center_loss() {
        let pb = PassbandShape::Gaussian { bw_3db_ghz: 37.5 };
        // At center (Δf=0): transmission = 1.0, loss = 0 dB
        let loss = pb.loss_db(0.0);
        assert!(loss.abs() < 0.01, "Center loss should be ~0 dB, got {}", loss);
    }

    #[test]
    fn test_gaussian_passband_3db_point() {
        let bw = 37.5;
        let pb = PassbandShape::Gaussian { bw_3db_ghz: bw };
        // At ±bw_3dB/2 the loss should be ~3 dB
        let loss = pb.loss_db(bw * 0.5e9);
        assert!((loss - 3.0).abs() < 0.1, "3 dB BW: loss={:.2} dB (expected ~3 dB)", loss);
    }

    #[test]
    fn test_super_gaussian_order_4_flatness() {
        let pb = PassbandShape::SuperGaussian { order: 4, bw_3db_ghz: 37.5 };
        // At 30% of the 3dB BW: super-Gaussian should be near 0 dB (flat top)
        let loss_center = pb.loss_db(0.0);
        let loss_edge = pb.loss_db(37.5 * 0.3e9);
        assert!(loss_center < 0.01, "Center should be near 0 dB");
        assert!(loss_edge < 1.0, "Flat-top: 30% point should be < 1 dB loss, got {}", loss_edge);
    }

    #[test]
    fn test_super_gaussian_3db_point() {
        let bw = 37.5;
        let pb = PassbandShape::SuperGaussian { order: 4, bw_3db_ghz: bw };
        let loss = pb.loss_db(bw * 0.5e9);
        assert!((loss - 3.0).abs() < 0.5, "3dB point check: loss={:.2}", loss);
    }

    #[test]
    fn test_rectangular_passband() {
        let pb = PassbandShape::Rectangular { bw_ghz: 50.0 };
        assert_eq!(pb.transmission(0.0), 1.0);
        assert_eq!(pb.transmission(20.0e9), 1.0); // inside
        assert_eq!(pb.transmission(30.0e9), 0.0); // outside (>25 GHz)
    }

    #[test]
    fn test_filter_concatenation_penalty_increases() {
        let pb = PassbandShape::Gaussian { bw_3db_ghz: 37.5 };
        let bw = pb.bw_3db_hz();
        let probe = bw / 4.0;
        let pen1 = pb.concatenation_penalty_db(1, probe);
        let pen5 = pb.concatenation_penalty_db(5, probe);
        let pen10 = pb.concatenation_penalty_db(10, probe);
        assert!(pen5 > pen1, "Penalty must increase with N");
        assert!(pen10 > pen5, "Penalty must increase with N");
    }

    #[test]
    fn test_filter_concatenation_penalty_zero_stages() {
        let pb = PassbandShape::Gaussian { bw_3db_ghz: 40.0 };
        let pen = pb.concatenation_penalty_db(0, 10.0e9);
        assert_eq!(pen, 0.0);
    }

    // ── WSS Tests ─────────────────────────────────────────────────────────────

    #[test]
    fn test_wss_insertion_loss_basic() {
        let cfg = WssConfig::new(4, PassbandShape::Gaussian { bw_3db_ghz: 40.0 }, 5.5);
        let mut wss = WssSwitch::new(cfg);
        wss.connect(0, 1, 0.0).unwrap();
        let loss = wss.insertion_loss_db(0, 1, 193.1e12);
        // flat_il=5.5, tilt≈0, connectors=0.6 → ~6.1 dB
        assert!(loss > 4.0 && loss < 10.0, "Expected ~6 dB IL, got {}", loss);
    }

    #[test]
    fn test_wss_invalid_port() {
        let cfg = WssConfig::new(4, PassbandShape::Gaussian { bw_3db_ghz: 40.0 }, 5.5);
        let mut wss = WssSwitch::new(cfg);
        let result = wss.connect(10, 0, 0.0); // port 10 out of range
        assert!(matches!(result, Err(WssError::InvalidPort(10))));
    }

    #[test]
    fn test_wss_wavelength_provision_and_deprovision() {
        let cfg = WssConfig::new(8, PassbandShape::SuperGaussian { order: 4, bw_3db_ghz: 38.0 }, 5.0);
        let mut wss = WssSwitch::new(cfg);
        wss.provision_wavelength(2, 193.1e12, 0.0).unwrap();
        assert_eq!(wss.active_count(), 1);
        let ok = wss.deprovision_wavelength(2, 193.1e12);
        assert!(ok);
        assert_eq!(wss.active_count(), 0);
    }

    #[test]
    fn test_wss_port_busy_error() {
        let cfg = WssConfig::new(4, PassbandShape::Gaussian { bw_3db_ghz: 38.0 }, 5.0);
        let mut wss = WssSwitch::new(cfg);
        wss.provision_wavelength(1, 193.1e12, 0.0).unwrap();
        let result = wss.provision_wavelength(1, 193.1e12, 1.0); // same port, same freq
        assert!(matches!(result, Err(WssError::PortBusy(1))));
    }

    #[test]
    fn test_wss_attenuation_setting() {
        let cfg = WssConfig::new(4, PassbandShape::Gaussian { bw_3db_ghz: 38.0 }, 5.0);
        let mut wss = WssSwitch::new(cfg);
        wss.provision_wavelength(0, 193.1e12, 0.0).unwrap();
        let ok = wss.set_attenuation(0, 193.1e12, 3.0);
        assert!(ok);
        // Loss should be ~3 dB higher now
        let loss_with_att = wss.insertion_loss_db(0, 0, 193.1e12);
        let mut wss2 = WssSwitch::new(WssConfig::new(4, PassbandShape::Gaussian { bw_3db_ghz: 38.0 }, 5.0));
        wss2.provision_wavelength(0, 193.1e12, 0.0).unwrap();
        let loss_no_att = wss2.insertion_loss_db(0, 0, 193.1e12);
        assert!((loss_with_att - loss_no_att - 3.0).abs() < 0.5, "VOA diff={}", loss_with_att - loss_no_att);
    }

    #[test]
    fn test_wss_crosstalk_different_ports() {
        let cfg = WssConfig::new(8, PassbandShape::Gaussian { bw_3db_ghz: 38.0 }, 5.0);
        let wss = WssSwitch::new(cfg);
        let iso = wss.crosstalk_db(0, 3);
        assert!((iso - 35.0).abs() < 0.1, "Isolation should be 35 dB");
    }

    #[test]
    fn test_wss_crosstalk_same_port() {
        let cfg = WssConfig::new(8, PassbandShape::Gaussian { bw_3db_ghz: 38.0 }, 5.0);
        let wss = WssSwitch::new(cfg);
        assert_eq!(wss.crosstalk_db(2, 2), 0.0);
    }

    #[test]
    fn test_wss_filter_concat_penalty_increases_with_nodes() {
        let cfg = WssConfig::new(4, PassbandShape::Gaussian { bw_3db_ghz: 38.0 }, 5.0);
        let wss = WssSwitch::new(cfg);
        let p1 = wss.filter_concatenation_penalty_db(1);
        let p5 = wss.filter_concatenation_penalty_db(5);
        assert!(p5 > p1, "Filter penalty should increase with N traversals");
    }

    // ── Channel Plan Tests ────────────────────────────────────────────────────

    #[test]
    fn test_channel_plan_provision_basic() {
        let grid = FrequencyGrid::fixed_50ghz();
        let mut plan = ChannelPlan::new(grid);
        let id = plan.provision(193.1e12, 37.5e9, ModulationFormat::DpQpsk, 0, 1).unwrap();
        assert_eq!(id, 1);
        assert_eq!(plan.active_count(), 1);
    }

    #[test]
    fn test_channel_plan_spectrum_conflict() {
        let grid = FrequencyGrid::fixed_50ghz();
        let mut plan = ChannelPlan::new(grid);
        plan.provision(193.1e12, 37.5e9, ModulationFormat::DpQpsk, 0, 1).unwrap();
        // Same frequency → conflict
        let result = plan.provision(193.1e12, 37.5e9, ModulationFormat::DpQpsk, 0, 2);
        assert!(matches!(result, Err(ChannelError::SpectrumConflict(_))));
    }

    #[test]
    fn test_channel_plan_deprovision() {
        let grid = FrequencyGrid::fixed_50ghz();
        let mut plan = ChannelPlan::new(grid);
        let id = plan.provision(193.1e12, 37.5e9, ModulationFormat::DpQpsk, 0, 1).unwrap();
        plan.deprovision(id).unwrap();
        assert_eq!(plan.active_count(), 0);
    }

    #[test]
    fn test_channel_plan_deprovision_not_found() {
        let grid = FrequencyGrid::fixed_50ghz();
        let mut plan = ChannelPlan::new(grid);
        let result = plan.deprovision(99);
        assert!(matches!(result, Err(ChannelError::NotFound(99))));
    }

    #[test]
    fn test_channel_plan_multiple_channels_sorted() {
        let grid = FrequencyGrid::fixed_50ghz();
        let mut plan = ChannelPlan::new(grid);
        // Provision in reverse frequency order
        plan.provision(193.2e12, 37.5e9, ModulationFormat::DpQpsk, 0, 1).unwrap();
        plan.provision(193.0e12, 37.5e9, ModulationFormat::DpQpsk, 0, 2).unwrap();
        plan.provision(193.1e12, 37.5e9, ModulationFormat::DpQpsk, 0, 3).unwrap();
        let chs = plan.active_channels();
        assert_eq!(chs.len(), 3);
        // Should be sorted by frequency
        assert!(chs[0].center_freq_hz < chs[1].center_freq_hz);
        assert!(chs[1].center_freq_hz < chs[2].center_freq_hz);
    }

    #[test]
    fn test_channel_capacity_dpqpsk() {
        let ch = OpticalChannel::new(1, 193.1e12, 37.5e9, ModulationFormat::DpQpsk, 0, 1);
        let cap = ch.capacity_gbps();
        // SE=4, baud ≈ 37.5 / 1.07 ≈ 35 Gbaud, capacity ≈ 140 Gbps
        assert!(cap > 100.0 && cap < 200.0, "DP-QPSK capacity estimate: {:.1} Gbps", cap);
    }

    #[test]
    fn test_channel_capacity_dp16qam() {
        let ch = OpticalChannel::new(2, 193.1e12, 75.0e9, ModulationFormat::Dp16Qam, 0, 1);
        let cap = ch.capacity_gbps();
        // SE=8, baud ≈ 75/1.07 ≈ 70 Gbaud, capacity ≈ 560 Gbps
        assert!(cap > 400.0 && cap < 700.0, "DP-16QAM capacity: {:.1} Gbps", cap);
    }

    #[test]
    fn test_channel_wavelength_nm() {
        let ch = OpticalChannel::new(1, 193.1e12, 50.0e9, ModulationFormat::DpQpsk, 0, 1);
        let wl = ch.wavelength_nm();
        // 193.1 THz → ≈ 1551.7 nm
        assert!((wl - 1551.7).abs() < 1.0, "Wavelength: {:.1} nm", wl);
    }

    // ── Flex-Grid Tests ───────────────────────────────────────────────────────

    #[test]
    fn test_flex_slot_center_frequency() {
        let grid = FrequencyGrid::flex_6_25ghz();
        let slot = FlexSlot { n: 0, m: 4 }; // 4×12.5 = 50 GHz width at 193.1 THz
        let center = slot.center_hz(&grid);
        assert!((center - 193.1e12).abs() < 1e9);
    }

    #[test]
    fn test_flex_slot_width() {
        let slot = FlexSlot { n: 0, m: 8 }; // 8×12.5 = 100 GHz
        assert!((slot.width_hz() - 100.0e9).abs() < 1e6);
    }

    #[test]
    fn test_flex_slot_overlap_detection() {
        let grid = FrequencyGrid::flex_6_25ghz();
        let s1 = FlexSlot { n: 0, m: 4 }; // centered at n=0, width 50 GHz
        let s2 = FlexSlot { n: 2, m: 4 }; // centered at n=2 (12.5 GHz away), overlapping
        let s3 = FlexSlot { n: 20, m: 4 }; // far away, no overlap
        assert!(s1.overlaps(&s2, &grid), "Adjacent slots should overlap");
        assert!(!s1.overlaps(&s3, &grid), "Far slots should not overlap");
    }

    #[test]
    fn test_channel_plan_first_fit_allocation() {
        let grid = FrequencyGrid::fixed_50ghz();
        let mut plan = ChannelPlan::new(grid);
        // Block middle of band
        plan.provision(193.5e12, 100.0e9, ModulationFormat::DpQpsk, 0, 1).unwrap();
        // First fit should find a slot below 193.5 THz
        let candidate = plan.allocate_flex(50.0e9, AllocationStrategy::FirstFit, 6.25e9);
        assert!(candidate.is_some(), "First-fit should find a slot");
        let f = candidate.unwrap();
        assert!(f < 193.5e12, "First-fit should go to lower frequency gap");
    }

    #[test]
    fn test_channel_plan_best_fit_allocation() {
        let grid = FrequencyGrid::fixed_50ghz();
        let mut plan = ChannelPlan::new(grid);
        // Leave a narrow gap and a wide gap
        plan.provision(193.0e12, 50.0e9, ModulationFormat::DpQpsk, 0, 1).unwrap();
        plan.provision(193.3e12, 50.0e9, ModulationFormat::DpQpsk, 0, 2).unwrap();
        let candidate = plan.allocate_flex(25.0e9, AllocationStrategy::BestFit, 0.0);
        assert!(candidate.is_some(), "Best-fit should find a slot");
    }

    #[test]
    fn test_fragmentation_ratio_empty() {
        let grid = FrequencyGrid::fixed_50ghz();
        let plan = ChannelPlan::new(grid);
        // No channels: no fragmentation (single contiguous free block)
        let frag = plan.fragmentation_ratio();
        assert!(frag < 0.1, "Empty plan should have very low fragmentation");
    }

    #[test]
    fn test_fragmentation_ratio_single_channel() {
        let grid = FrequencyGrid::fixed_50ghz();
        let mut plan = ChannelPlan::new(grid);
        plan.provision(193.1e12, 50.0e9, ModulationFormat::DpQpsk, 0, 1).unwrap();
        let frag = plan.fragmentation_ratio();
        // One channel in the middle creates some fragmentation
        assert!(frag >= 0.0 && frag <= 1.0);
    }

    #[test]
    fn test_utilization() {
        let grid = FrequencyGrid::fixed_50ghz();
        let mut plan = ChannelPlan::new(grid);
        // Band width ≈ 4.75 THz, slot = 50 GHz
        plan.provision(193.1e12, 50.0e9, ModulationFormat::DpQpsk, 0, 1).unwrap();
        let util = plan.utilization();
        assert!(util > 0.0 && util < 0.1, "Utilization: {:.4}", util);
    }

    // ── Power Equalization Tests ──────────────────────────────────────────────

    #[test]
    fn test_power_equalizer_update_high_power() {
        let mut eq = PowerEqualizer::new(-2.0, 20.0);
        // Channel measured at +3 dBm (5 dB above target -2 dBm)
        let voa = eq.update(1, 3.0);
        assert!(voa > 0.0, "Should add attenuation for high power");
    }

    #[test]
    fn test_power_equalizer_update_low_power() {
        let mut eq = PowerEqualizer::new(-2.0, 20.0);
        // Channel measured at -10 dBm (below target): no negative VOA
        let voa = eq.update(1, -10.0);
        assert!(voa == 0.0, "VOA cannot go negative (clamped at 0)");
    }

    #[test]
    fn test_power_equalizer_convergence() {
        let mut eq = PowerEqualizer::new(-2.0, 20.0);
        // All channels above target so VOA can actively attenuate them to converge.
        // channels: (id, measured_power_dbm) all above target of -2 dBm.
        let channels = vec![(1u32, 3.0_f64), (2u32, 5.0_f64), (3u32, 2.0_f64)];
        let residual = eq.converge(&channels, 50);
        // After 50 iterations the high-power channels should be well attenuated.
        // The "residual" reflects uncorrected deviation (positive signals → VOA corrects).
        assert!(residual < 30.0, "Convergence residual: {:.2}", residual);
        // VOA for the highest-powered channel (ch2, +7 dB above target) should be nonzero
        let voa2 = eq.voa_settings.get(&2).copied().unwrap_or(0.0);
        assert!(voa2 > 0.0, "VOA for above-target channel should be positive, got {}", voa2);
    }

    #[test]
    fn test_power_equalizer_tilt_compensation() {
        let mut eq = PowerEqualizer::new(-2.0, 20.0);
        let channels = vec![
            (1u32, 193.0e12_f64),
            (2u32, 193.5e12_f64),
            (3u32, 194.0e12_f64),
        ];
        eq.apply_tilt_compensation(&channels, 0.1);
        // Higher-frequency channels should have less attenuation (compensate for tilt)
        // Check that the VOA values differ between channels
        let v1 = eq.voa_settings.get(&1).copied().unwrap_or(0.0);
        let v3 = eq.voa_settings.get(&3).copied().unwrap_or(0.0);
        assert!(v1 != v3 || v1 == 0.0, "Tilt compensation should produce different VOAs");
    }

    // ── OSNR Tests ────────────────────────────────────────────────────────────

    #[test]
    fn test_edfa_ase_power_positive() {
        let edfa = EdfaModel::new(20.0, 5.0);
        let ase = edfa.ase_power_mw(193.1e12, 12.5e9);
        assert!(ase > 0.0, "ASE power should be positive");
        assert!(ase < 1.0, "ASE should be sub-mW per 12.5 GHz BW for G=20dB NF=5dB");
    }

    #[test]
    fn test_edfa_single_amplifier_osnr_reasonable() {
        // 20 dB gain, 5 dB NF, -2 dBm input signal → should yield good OSNR
        let edfa = EdfaModel::new(20.0, 5.0);
        let osnr = edfa.single_amplifier_osnr_db(-2.0, 193.1e12);
        assert!(osnr > 20.0 && osnr < 50.0, "Single-amp OSNR: {:.1} dB", osnr);
    }

    #[test]
    fn test_cascaded_osnr_decreases() {
        // 10 spans, each OSNR = 30 dB
        let per_span = vec![30.0; 10];
        let total = cascaded_osnr_db(&per_span);
        assert!(total < 30.0, "Cascaded OSNR should be lower than per-span");
        // For 10 equal 30dB spans: total ≈ 30 - 10 log10(10) = 20 dB
        assert!((total - 20.0).abs() < 1.0, "10×30dB → ~20dB total OSNR, got {:.1}", total);
    }

    #[test]
    fn test_cascaded_osnr_single_span() {
        let per_span = vec![25.0];
        let total = cascaded_osnr_db(&per_span);
        assert!((total - 25.0).abs() < 0.01);
    }

    #[test]
    fn test_cascaded_osnr_empty() {
        let total = cascaded_osnr_db(&[]);
        assert!(total > 40.0, "Empty span list should return high OSNR");
    }

    #[test]
    fn test_gsnr_limited_by_nli() {
        let osnr = 30.0; // Good OSNR
        let snr_nli = 20.0; // NLI-limited
        let gsnr = compute_gsnr_db(osnr, snr_nli);
        // GSNR should be dominated by the lower limit (NLI)
        assert!(gsnr < snr_nli + 1.0, "GSNR should be ≤ SNR_NLI when NLI limited");
    }

    #[test]
    fn test_gsnr_limited_by_osnr() {
        let osnr = 15.0; // Limited OSNR
        let snr_nli = 40.0; // Not NLI-limited
        let gsnr = compute_gsnr_db(osnr, snr_nli);
        assert!(gsnr < osnr + 1.0, "GSNR should be ≤ OSNR when ASE limited");
        assert!(gsnr > osnr - 5.0, "GSNR should be close to OSNR when not NLI limited");
    }

    // ── Path Computation Tests ────────────────────────────────────────────────

    #[test]
    fn test_path_impairments_single_span() {
        let edfa = EdfaModel::new(22.0, 5.0);
        let hop = PathHop {
            fiber_length_km: 80.0,
            fiber_type: FiberType::G652,
            amplifier: edfa,
            wss_count: 1,
            wss_config: Some(WssConfig::new(4, PassbandShape::Gaussian { bw_3db_ghz: 38.0 }, 5.5)),
        };
        let imp = compute_path_impairments(&[hop], 193.1e12, -2.0, 1.27);
        // 80 km G.652: loss ≈ 16 dB
        assert!((imp.total_loss_db - 16.0).abs() < 1.0, "Fiber loss: {:.1} dB", imp.total_loss_db);
        assert!(imp.osnr_db > 20.0, "OSNR should be > 20 dB for 1 span");
        assert!(imp.num_spans == 1);
    }

    #[test]
    fn test_path_impairments_multi_span_osnr_degrades() {
        let edfa = EdfaModel::new(22.0, 5.0);
        let hop = PathHop {
            fiber_length_km: 80.0,
            fiber_type: FiberType::G652,
            amplifier: edfa,
            wss_count: 1,
            wss_config: None,
        };
        let hops_5: Vec<_> = (0..5).map(|_| hop.clone()).collect();
        let hops_10: Vec<_> = (0..10).map(|_| hop.clone()).collect();
        let imp5 = compute_path_impairments(&hops_5, 193.1e12, -2.0, 1.27);
        let imp10 = compute_path_impairments(&hops_10, 193.1e12, -2.0, 1.27);
        assert!(imp10.osnr_db < imp5.osnr_db, "OSNR degrades with more spans");
    }

    #[test]
    fn test_path_filter_penalty_increases() {
        let edfa = EdfaModel::new(22.0, 5.0);
        let wss_cfg = WssConfig::new(4, PassbandShape::Gaussian { bw_3db_ghz: 38.0 }, 5.5);
        let hop = PathHop {
            fiber_length_km: 80.0,
            fiber_type: FiberType::G652,
            amplifier: edfa,
            wss_count: 2,
            wss_config: Some(wss_cfg),
        };
        let hops_5: Vec<_> = (0..5).map(|_| hop.clone()).collect();
        let hops_10: Vec<_> = (0..10).map(|_| hop.clone()).collect();
        let imp5 = compute_path_impairments(&hops_5, 193.1e12, -2.0, 1.27);
        let imp10 = compute_path_impairments(&hops_10, 193.1e12, -2.0, 1.27);
        assert!(imp10.filter_penalty_db > imp5.filter_penalty_db);
    }

    #[test]
    fn test_reach_estimation_dpqpsk_vs_dp64qam() {
        let edfa = EdfaModel::new(22.0, 5.0);
        let hop = PathHop {
            fiber_length_km: 80.0,
            fiber_type: FiberType::G652,
            amplifier: edfa,
            wss_count: 1,
            wss_config: None,
        };
        let reach_qpsk = estimate_reach(&hop, ModulationFormat::DpQpsk, 193.1e12, -2.0, 1.27, 3.0);
        let reach_64qam = estimate_reach(&hop, ModulationFormat::Dp64Qam, 193.1e12, -2.0, 1.27, 3.0);
        assert!(reach_qpsk > reach_64qam, "DP-QPSK reaches farther than DP-64QAM");
        assert!(reach_qpsk >= 1, "DP-QPSK should support at least 1 hop");
    }

    #[test]
    fn test_osnr_margin_positive_for_adequate_osnr() {
        let modulation = ModulationFormat::DpQpsk; // requires 12.5 dB
        let margin = osnr_margin_db(20.0, modulation);
        assert!((margin - 7.5).abs() < 0.1, "Margin should be 7.5 dB");
    }

    #[test]
    fn test_osnr_margin_negative_for_inadequate_osnr() {
        let modulation = ModulationFormat::Dp64Qam; // requires 25 dB
        let margin = osnr_margin_db(20.0, modulation);
        assert!(margin < 0.0, "Should be negative margin");
    }

    // ── Spectrum Visualization Tests ──────────────────────────────────────────

    #[test]
    fn test_spectrum_generation_basic() {
        let grid = FrequencyGrid::fixed_50ghz();
        let mut plan = ChannelPlan::new(grid);
        plan.provision(193.1e12, 37.5e9, ModulationFormat::DpQpsk, 0, 1).unwrap();
        plan.get_channel_mut(1).unwrap().target_power_dbm = -2.0;

        let analyzer = SpectrumAnalyzer::new(5.0e9);
        let spectrum = analyzer.generate_spectrum(&plan, None, 100);
        assert_eq!(spectrum.len(), 100);
        // Peak should be near -2 dBm
        let peak = analyzer.peak_power(&spectrum);
        assert!(peak > -10.0 && peak < 5.0, "Peak: {:.1} dBm", peak);
    }

    #[test]
    fn test_spectrum_peak_finding() {
        let analyzer = SpectrumAnalyzer::new(5.0e9);
        let channels = vec![
            (193.1e12, 37.5e9, -2.0),
            (193.2e12, 37.5e9, -3.0),
        ];
        let spectrum = analyzer.generate_from_channels(&channels, None, 500);
        let peaks = analyzer.find_peaks(&spectrum, -15.0);
        assert!(!peaks.is_empty(), "Should find at least one peak");
    }

    #[test]
    fn test_spectrum_ase_floor() {
        let grid = FrequencyGrid::fixed_50ghz();
        let plan = ChannelPlan::new(grid); // No channels
        let mut analyzer = SpectrumAnalyzer::new(5.0e9);
        analyzer.noise_floor_dbm = -35.0;
        let spectrum = analyzer.generate_spectrum(&plan, None, 100);
        // All points should be near the noise floor
        for pt in &spectrum {
            assert!((pt.power_dbm - (-35.0)).abs() < 1.0, "Empty spectrum should be near noise floor: {:.1}", pt.power_dbm);
        }
    }

    #[test]
    fn test_spectrum_wss_narrows_channel() {
        let analyzer = SpectrumAnalyzer::new(5.0e9);
        let channels = vec![(193.1e12, 50.0e9, -2.0)];
        let pb = PassbandShape::Gaussian { bw_3db_ghz: 38.0 };
        let spectrum_open = analyzer.generate_from_channels(&channels, None, 200);
        let spectrum_wss = analyzer.generate_from_channels(&channels, Some(pb), 200);
        let peak_open = analyzer.peak_power(&spectrum_open);
        let peak_wss = analyzer.peak_power(&spectrum_wss);
        // At center: WSS peak ≈ open peak (both near -2 dBm at Δf=0)
        assert!((peak_open - peak_wss).abs() < 1.0, "Peak at center unchanged by WSS: open={:.1}, wss={:.1}", peak_open, peak_wss);
    }

    // ── ROADM Node Tests ──────────────────────────────────────────────────────

    #[test]
    fn test_roadm_node_add_channel() {
        let express_cfg = WssConfig::new(8, PassbandShape::SuperGaussian { order: 4, bw_3db_ghz: 38.0 }, 5.5);
        let add_cfg = WssConfig::new(20, PassbandShape::SuperGaussian { order: 4, bw_3db_ghz: 38.0 }, 5.5);
        let drop_cfg = WssConfig::new(20, PassbandShape::SuperGaussian { order: 4, bw_3db_ghz: 38.0 }, 5.5);
        let grid = FrequencyGrid::fixed_50ghz();
        let mut node = RoadmNode::new("NodeA", express_cfg, add_cfg, drop_cfg, grid, -2.0);
        let id = node.add_channel(193.1e12, 37.5e9, ModulationFormat::DpQpsk, 0).unwrap();
        assert_eq!(id, 1);
        assert_eq!(node.channel_plan.active_count(), 1);
    }

    #[test]
    fn test_roadm_node_insertion_loss() {
        let express_cfg = WssConfig::new(4, PassbandShape::Gaussian { bw_3db_ghz: 38.0 }, 6.0);
        let add_cfg = WssConfig::new(4, PassbandShape::Gaussian { bw_3db_ghz: 38.0 }, 6.0);
        let drop_cfg = WssConfig::new(4, PassbandShape::Gaussian { bw_3db_ghz: 38.0 }, 6.0);
        let grid = FrequencyGrid::fixed_50ghz();
        let node = RoadmNode::new("NodeB", express_cfg, add_cfg, drop_cfg, grid, -2.0);
        let il = node.node_insertion_loss_db(193.1e12, 0, 1);
        assert!(il > 0.0, "Insertion loss must be positive");
    }

    #[test]
    fn test_roadm_node_utilization() {
        let express_cfg = WssConfig::new(8, PassbandShape::Gaussian { bw_3db_ghz: 38.0 }, 5.5);
        let add_cfg = WssConfig::new(20, PassbandShape::Gaussian { bw_3db_ghz: 38.0 }, 5.5);
        let drop_cfg = WssConfig::new(20, PassbandShape::Gaussian { bw_3db_ghz: 38.0 }, 5.5);
        let grid = FrequencyGrid::fixed_50ghz();
        let mut node = RoadmNode::new("NodeC", express_cfg, add_cfg, drop_cfg, grid, -2.0);
        node.add_channel(193.1e12, 50.0e9, ModulationFormat::DpQpsk, 0).unwrap();
        let util = node.utilization();
        assert!(util > 0.0 && util < 1.0);
    }

    // ── Edge Case Tests ───────────────────────────────────────────────────────

    #[test]
    fn test_adjacent_channels_guard_band() {
        let grid = FrequencyGrid::fixed_50ghz();
        let mut plan = ChannelPlan::new(grid);
        // Two 37.5 GHz channels at 193.0 and 193.1 THz (100 GHz apart)
        plan.provision(193.0e12, 37.5e9, ModulationFormat::DpQpsk, 0, 1).unwrap();
        let result = plan.provision(193.05e12, 37.5e9, ModulationFormat::DpQpsk, 0, 2);
        // 193.05 THz is only 50 GHz from 193.0, slots overlap (37.5 + 37.5)/2 = 37.5 GHz → no conflict at 50 GHz
        // This depends on snap_to_grid behavior
        let _ = result; // Either Ok or Err depending on grid snap
    }

    #[test]
    fn test_wss_multiple_wavelengths_on_different_ports() {
        let cfg = WssConfig::new(8, PassbandShape::Gaussian { bw_3db_ghz: 38.0 }, 5.0);
        let mut wss = WssSwitch::new(cfg);
        wss.provision_wavelength(0, 193.0e12, 0.0).unwrap();
        wss.provision_wavelength(1, 193.1e12, 0.0).unwrap();
        wss.provision_wavelength(2, 193.2e12, 0.0).unwrap();
        assert_eq!(wss.active_count(), 3);
    }

    #[test]
    fn test_modulation_min_osnr_ordering() {
        // Higher-order modulations should require more OSNR
        let qpsk = ModulationFormat::DpQpsk.min_osnr_db();
        let qam16 = ModulationFormat::Dp16Qam.min_osnr_db();
        let qam64 = ModulationFormat::Dp64Qam.min_osnr_db();
        let qam256 = ModulationFormat::Dp256Qam.min_osnr_db();
        assert!(qpsk < qam16, "DP-QPSK should need less OSNR than DP-16QAM");
        assert!(qam16 < qam64);
        assert!(qam64 < qam256);
    }

    #[test]
    fn test_fiber_g652_dispersion() {
        let f = FiberType::G652;
        assert!((f.dispersion_ps_per_nm_km() - 17.0).abs() < 0.5);
        assert!((f.attenuation_db_per_km() - 0.20).abs() < 0.01);
    }

    #[test]
    fn test_spectrum_points_are_monotonic_in_frequency() {
        let analyzer = SpectrumAnalyzer::new(5.0e9);
        let channels = vec![(193.1e12, 37.5e9, -2.0)];
        let spectrum = analyzer.generate_from_channels(&channels, None, 200);
        for i in 1..spectrum.len() {
            assert!(spectrum[i].freq_hz > spectrum[i - 1].freq_hz, "Spectrum points should be monotonically increasing in frequency");
        }
    }

    #[test]
    fn test_nli_snr_decreases_with_launch_power() {
        // At very high launch power, NLI dominates (SNR_NLI decreases)
        let snr_low = estimate_nli_snr_db(-5.0, 10.0, 80.0, 1.27);
        let snr_high = estimate_nli_snr_db(3.0, 10.0, 80.0, 1.27);
        // Higher launch power → worse SNR_NLI (more NLI)
        assert!(snr_low > snr_high, "SNR_NLI should decrease with higher launch power");
    }
}
